// Author: Antonio Lattanzio - emptyvessel

#include "Box3DBodyComponent.h"
#include "Box3DSubsystem.h"
#include "Box3DConversion.h"
#include "Box3DStaticGeometry.h"
#include "Box3DLog.h"
#include "Components/PrimitiveComponent.h"
#include "DrawDebugHelpers.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "PhysicsEngine/AggregateGeom.h"
#include "PhysicsEngine/BodySetup.h"
#include "PhysicsEngine/ConvexElem.h"

namespace
{
	// Cap the hull vertex count, matching the static-geometry path.
	constexpr int32 ConvexMaxHullVertices = 64;

	// Local vertex (cm) -> box3d point (m): bake scale, negate Y (see Box3DConversion.h).
	FORCEINLINE b3Vec3 ConvexLocalToBox3D(const FVector& V, const FVector& Scale)
	{
		return b3Vec3{
			static_cast<float>(V.X * Scale.X * Box3D::UnrealToMeters),
			static_cast<float>(-V.Y * Scale.Y * Box3D::UnrealToMeters),
			static_cast<float>(V.Z * Scale.Z * Box3D::UnrealToMeters) };
	}

	void GatherPrimitiveClouds(UPrimitiveComponent* Prim, const FTransform& SourceToRoot,
		const FVector& RootScale, TArray<TArray<b3Vec3>>& OutClouds)
	{
		UBodySetup* Setup = Prim ? Prim->GetBodySetup() : nullptr;
		if (Setup == nullptr)
		{
			return;
		}

		const FKAggregateGeom& Agg = Setup->AggGeom;

		// Convex: element transform places local verts into body space.
		for (const FKConvexElem& Convex : Agg.ConvexElems)
		{
			if (Convex.VertexData.Num() < 4)
			{
				continue;
			}
			const FTransform ElemTM = Convex.GetTransform();
			TArray<b3Vec3>& Cloud = OutClouds.AddDefaulted_GetRef();
			Cloud.Reserve(Convex.VertexData.Num());
			for (const FVector& V : Convex.VertexData)
			{
				Cloud.Add(ConvexLocalToBox3D(SourceToRoot.TransformPosition(ElemTM.TransformPosition(V)), RootScale));
			}
		}

		// Boxes: 8 corners into a hull (box simple collision, e.g. the default).
		for (const FKBoxElem& Box : Agg.BoxElems)
		{
			const FTransform ElemTM(Box.Rotation, Box.Center);
			const FVector He(Box.X * 0.5f, Box.Y * 0.5f, Box.Z * 0.5f);
			TArray<b3Vec3>& Cloud = OutClouds.AddDefaulted_GetRef();
			Cloud.Reserve(8);
			for (int32 Sx = -1; Sx <= 1; Sx += 2)
			for (int32 Sy = -1; Sy <= 1; Sy += 2)
			for (int32 Sz = -1; Sz <= 1; Sz += 2)
			{
				const FVector Corner(Sx * He.X, Sy * He.Y, Sz * He.Z);
				Cloud.Add(ConvexLocalToBox3D(SourceToRoot.TransformPosition(ElemTM.TransformPosition(Corner)), RootScale));
			}
		}
	}

	void GatherConvexPointClouds(USceneComponent* Root, bool bIncludeChildren,
		TArray<TArray<b3Vec3>>& OutClouds)
	{
		if (Root == nullptr)
		{
			return;
		}

		const FVector RootScale = Root->GetComponentScale();
		if (UPrimitiveComponent* RootPrim = Cast<UPrimitiveComponent>(Root))
		{
			GatherPrimitiveClouds(RootPrim, FTransform::Identity, RootScale, OutClouds);
		}

		if (!bIncludeChildren)
		{
			return;
		}

		TArray<USceneComponent*> Children;
		Root->GetChildrenComponents(/*bIncludeAllDescendants=*/true, Children);

		const FTransform RootTM = Root->GetComponentTransform();
		for (USceneComponent* Child : Children)
		{
			UPrimitiveComponent* ChildPrim = Cast<UPrimitiveComponent>(Child);
			if (ChildPrim == nullptr || ChildPrim->GetCollisionEnabled() == ECollisionEnabled::NoCollision)
			{
				continue;
			}
			GatherPrimitiveClouds(ChildPrim, ChildPrim->GetComponentTransform().GetRelativeTransform(RootTM),
				RootScale, OutClouds);
		}
	}
} // namespace

UBox3DBodyComponent::UBox3DBodyComponent()
{
	// The subsystem drives capture/interpolation; the component itself never ticks.
	PrimaryComponentTick.bCanEverTick = false;
}

void UBox3DBodyComponent::BeginPlay()
{
	Super::BeginPlay();

	Subsystem = GetWorld() ? GetWorld()->GetSubsystem<UBox3DSubsystem>() : nullptr;
	if (Subsystem == nullptr)
	{
		return;
	}

	if (Shape == EBox3DShape::Box)
	{
		// Author-specified extents are centred on the body origin by definition.
		ResolvedHalfExtent = BoxHalfExtent.GetAbs();
		ResolvedBoxCenter = FVector::ZeroVector;
	}
	else if (Shape == EBox3DShape::Auto || Shape == EBox3DShape::Convex)
	{
		ResolveAutoBoxBounds();
	}

	// Cache the convex wireframe on both server and client so both can draw the shape.
	if (Shape == EBox3DShape::Convex)
	{
		BuildConvexDebugGeometry();
	}

	// Register for debug draw in every world (server and client).
	Subsystem->RegisterBody(this);

	bSimulationEligible = ComputeSimulationEligibility();

	if (bSimulationEligible && UBox3DSubsystem::IsBox3DEnabled())
	{
		RebuildSimulationBody();
	}
}

bool UBox3DBodyComponent::ComputeSimulationEligibility()
{
	AActor* Owner = GetOwner();
	const ENetMode NetMode = GetWorld()->GetNetMode();
	if (Owner == nullptr || NetMode == NM_Client || !Owner->HasAuthority())
	{
		return false;
	}

	if (NetMode != NM_Standalone && Owner->IsNetStartupActor() && !Owner->GetIsReplicated())
	{
		UE_LOG(LogBox3D, Warning,
			TEXT("%s: level-placed body actor is not replicated; the client will simulate a duplicate, ")
			TEXT("out-of-sync body. Enable 'Replicates' on the actor in the editor."),
			*GetNameSafe(Owner));
	}

	return true;
}

void UBox3DBodyComponent::RebuildSimulationBody()
{
	if (B3_IS_NON_NULL(BodyId) || !bSimulationEligible || Subsystem == nullptr || !Subsystem->IsWorldValid())
	{
		return;
	}

	Subsystem->FlushAsyncStep();

	CreateBody();
	if (B3_IS_NULL(BodyId))
	{
		return;
	}

	if (bIsSensor && BodyType == EBox3DBodyType::Dynamic)
	{
		UE_LOG(LogBox3D, Warning,
			TEXT("%s: a trigger never collides, so this Dynamic body will fall through the level. ")
			TEXT("Use Static or Kinematic."),
			*GetNameSafe(GetOwner()));
	}

	AddShape();

	if (BodyType == EBox3DBodyType::Dynamic || BodyType == EBox3DBodyType::Kinematic)
	{
		EnforceAuthorityContract();
		EnableReplication();

		if (BodyType == EBox3DBodyType::Dynamic)
		{
			Subsystem->RegisterDynamicBody(this);   // box3d writes the actor each step

			// Flush any force/velocity requested before the body existed.
			if (!PendingLinearImpulse.IsNearlyZero())
			{
				AddImpulse(PendingLinearImpulse, /*bWake=*/true);
				PendingLinearImpulse = FVector::ZeroVector;
			}
			if (bHasPendingAngularVelocity)
			{
				SetAngularVelocity(PendingAngularVelocity, /*bWake=*/true);
				bHasPendingAngularVelocity = false;
			}
		}
		else
		{
			Subsystem->RegisterKinematicBody(this); // gameplay pose pushed in each step
		}
	}
}

void UBox3DBodyComponent::FenceAsyncStep() const
{
	if (Subsystem != nullptr)
	{
		Subsystem->FlushAsyncStep();
	}
}

void UBox3DBodyComponent::AddImpulse(const FVector& Impulse, bool bWake)
{
	if (BodyType != EBox3DBodyType::Dynamic)
	{
		return;
	}
	if (B3_IS_NULL(BodyId))
	{
		PendingLinearImpulse += Impulse; // apply once the body is built
		return;
	}
	FenceAsyncStep();
	b3Body_ApplyLinearImpulseToCenter(BodyId, Box3D::ToBox3DVector(Impulse), bWake);
}

void UBox3DBodyComponent::AddImpulseAtLocation(const FVector& Impulse, const FVector& WorldLocation, bool bWake)
{
	if (BodyType != EBox3DBodyType::Dynamic || B3_IS_NULL(BodyId))
	{
		return; // not queued: the world point would be stale by the time the body exists
	}
	FenceAsyncStep();
	b3Body_ApplyLinearImpulse(BodyId, Box3D::ToBox3DVector(Impulse),
		Box3D::ToBox3DPosition(WorldLocation), bWake);
}

void UBox3DBodyComponent::AddForce(const FVector& Force, bool bWake)
{
	if (BodyType != EBox3DBodyType::Dynamic || B3_IS_NULL(BodyId))
	{
		return; // not queued: a force lasts one step, so it would land at the wrong time
	}
	FenceAsyncStep();
	b3Body_ApplyForceToCenter(BodyId, Box3D::ToBox3DVector(Force), bWake);
}

void UBox3DBodyComponent::AddTorque(const FVector& Torque, bool bWake)
{
	if (BodyType != EBox3DBodyType::Dynamic || B3_IS_NULL(BodyId))
	{
		return;
	}
	// N*m = kg*m^2/s^2, so the length factor applies twice.
	const float TorqueToBox3D = static_cast<float>(Box3D::UnrealToMeters * Box3D::UnrealToMeters);
	FenceAsyncStep();
	b3Body_ApplyTorque(BodyId, b3MulSV(TorqueToBox3D, Box3D::ToBox3DAngular(Torque)), bWake);
}

void UBox3DBodyComponent::AddAngularImpulse(const FVector& AngularImpulse, bool bWake)
{
	if (BodyType != EBox3DBodyType::Dynamic || B3_IS_NULL(BodyId))
	{
		return;
	}
	// Same squared length factor as torque.
	const float ImpulseToBox3D = static_cast<float>(Box3D::UnrealToMeters * Box3D::UnrealToMeters);
	FenceAsyncStep();
	b3Body_ApplyAngularImpulse(BodyId, b3MulSV(ImpulseToBox3D, Box3D::ToBox3DAngular(AngularImpulse)), bWake);
}

void UBox3DBodyComponent::SetLinearVelocity(const FVector& Velocity, bool bWake)
{
	if (BodyType != EBox3DBodyType::Dynamic || B3_IS_NULL(BodyId))
	{
		return;
	}
	FenceAsyncStep();
	b3Body_SetLinearVelocity(BodyId, Box3D::ToBox3DVector(Velocity));
	if (bWake)
	{
		b3Body_SetAwake(BodyId, true);
	}
}

void UBox3DBodyComponent::SetAngularVelocity(const FVector& AngularVelocity, bool bWake)
{
	if (BodyType != EBox3DBodyType::Dynamic)
	{
		return;
	}
	if (B3_IS_NULL(BodyId))
	{
		PendingAngularVelocity = AngularVelocity;
		bHasPendingAngularVelocity = true;
		return;
	}
	// Axial, and rad/s is scale-free - no cm<->m factor here.
	FenceAsyncStep();
	b3Body_SetAngularVelocity(BodyId, Box3D::ToBox3DAngular(AngularVelocity));
	if (bWake)
	{
		b3Body_SetAwake(BodyId, true);
	}
}

void UBox3DBodyComponent::SetGravityScale(float Scale)
{
	if (B3_IS_NON_NULL(BodyId))
	{
		FenceAsyncStep();
		b3Body_SetGravityScale(BodyId, Scale);
		b3Body_SetAwake(BodyId, true); // a resting body would ignore the change
	}
}

void UBox3DBodyComponent::SetSleepEnabled(bool bEnabled)
{
	if (B3_IS_NON_NULL(BodyId))
	{
		FenceAsyncStep();
		b3Body_EnableSleep(BodyId, bEnabled);
	}
}

void UBox3DBodyComponent::TeleportBody(const FVector& Location, const FRotator& Rotation)
{
	if (B3_IS_NULL(BodyId))
	{
		return;
	}

	const FQuat Quat = Rotation.Quaternion();
	FenceAsyncStep();
	b3Body_SetTransform(BodyId, Box3D::ToBox3DPosition(Location), Box3D::ToBox3DQuat(Quat));
	b3Body_SetLinearVelocity(BodyId, b3Vec3_zero);
	b3Body_SetAngularVelocity(BodyId, b3Vec3_zero);
	b3Body_SetAwake(BodyId, true);

	PrevTransform = CurrTransform = FTransform(Quat, Location, SpawnScale);
	GetOwner()->SetActorLocationAndRotation(Location, Quat, /*bSweep=*/false, nullptr, ETeleportType::TeleportPhysics);
}

FVector UBox3DBodyComponent::GetLinearVelocity() const
{
	if (B3_IS_NULL(BodyId))
	{
		return FVector::ZeroVector;
	}
	FenceAsyncStep();
	return Box3D::FromBox3DVector(b3Body_GetLinearVelocity(BodyId));
}

FVector UBox3DBodyComponent::GetAngularVelocity() const
{
	if (B3_IS_NULL(BodyId))
	{
		return FVector::ZeroVector;
	}
	FenceAsyncStep();
	return Box3D::FromBox3DAngular(b3Body_GetAngularVelocity(BodyId));
}

float UBox3DBodyComponent::GetBodyMass() const
{
	if (B3_IS_NULL(BodyId))
	{
		return 0.0f;
	}
	FenceAsyncStep();
	return b3Body_GetMass(BodyId);
}

bool UBox3DBodyComponent::IsBodyAwake() const
{
	if (B3_IS_NULL(BodyId))
	{
		return false;
	}
	FenceAsyncStep();
	return b3Body_IsAwake(BodyId);
}

void UBox3DBodyComponent::WakeBody()
{
	if (B3_IS_NON_NULL(BodyId))
	{
		FenceAsyncStep();
		b3Body_SetAwake(BodyId, true);
	}
}

void UBox3DBodyComponent::RebuildShapes()
{
	if (B3_IS_NULL(BodyId))
	{
		return;
	}

	if (Subsystem != nullptr)
	{
		Subsystem->FlushAsyncStep();
	}

	const int32 Count = b3Body_GetShapeCount(BodyId);
	if (Count > 0)
	{
		TArray<b3ShapeId> Shapes;
		Shapes.SetNumUninitialized(Count);
		const int32 Fetched = b3Body_GetShapes(BodyId, Shapes.GetData(), Count);
		for (int32 i = 0; i < Fetched; ++i)
		{
			// Defer the mass update to the single ApplyMassFromShapes below.
			b3DestroyShape(Shapes[i], /*updateBodyMass=*/false);
		}
	}

	// Static bodies may hold tri-mesh data referenced by the shapes we just destroyed.
	for (b3MeshData* Mesh : OwnedMeshes)
	{
		if (Mesh != nullptr)
		{
			b3DestroyMesh(Mesh);
		}
	}
	OwnedMeshes.Reset();

	if (Shape == EBox3DShape::Auto || Shape == EBox3DShape::Convex)
	{
		ResolveAutoBoxBounds();
	}
	if (Shape == EBox3DShape::Convex)
	{
		BuildConvexDebugGeometry();
	}

	AddShape();
	b3Body_ApplyMassFromShapes(BodyId);
	b3Body_SetAwake(BodyId, true);
}

void UBox3DBodyComponent::TeardownSimulationBody()
{
	if (bRestoreChaosSimulation)
	{
		if (UPrimitiveComponent* Root = Cast<UPrimitiveComponent>(GetOwner()->GetRootComponent()))
		{
			Root->SetSimulatePhysics(true);
		}
		bRestoreChaosSimulation = false;
	}

	DestroyBody();
}

void UBox3DBodyComponent::EnableReplication()
{
	AActor* Owner = GetOwner();
	if (Owner == nullptr || GetWorld()->GetNetMode() == NM_Standalone)
	{
		return; // single-player: nothing to replicate
	}

	if (!Owner->GetIsReplicated())
	{
		Owner->SetReplicates(true);
		UE_LOG(LogBox3D, Log,
			TEXT("%s: enabling actor replication so clients receive box3d movement."),
			*GetNameSafe(Owner));
	}

	Owner->SetReplicateMovement(true);
}

void UBox3DBodyComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	if (Subsystem != nullptr)
	{
		Subsystem->UnregisterBody(this);
	}
	DestroyBody();

	Super::EndPlay(EndPlayReason);
}

void UBox3DBodyComponent::CreateBody()
{
	const FTransform ActorXform = GetOwner()->GetActorTransform();
	SpawnScale = ActorXform.GetScale3D();
	PrevTransform = CurrTransform = ActorXform;

	b3BodyDef Def = b3DefaultBodyDef();
	switch (BodyType)
	{
	case EBox3DBodyType::Static:    Def.type = b3_staticBody;    break;
	case EBox3DBodyType::Kinematic: Def.type = b3_kinematicBody; break;
	default:                        Def.type = b3_dynamicBody;   break;
	}
	Def.position = Box3D::ToBox3DPosition(ActorXform.GetLocation());
	Def.rotation = Box3D::ToBox3DQuat(ActorXform.GetRotation());
	Def.linearDamping = LinearDamping;
	Def.angularDamping = AngularDamping;

	Def.userData = GetOwner();

	BodyId = b3CreateBody(Subsystem->GetWorldId(), &Def);
	bWasAwake = true; // bodies are created awake
}

void UBox3DBodyComponent::AddShape()
{
	b3ShapeDef ShapeDef = b3DefaultShapeDef();
	ShapeDef.density = Box3D::ToBox3DDensity(Density);
	ShapeDef.baseMaterial.friction = Friction;
	ShapeDef.baseMaterial.restitution = Restitution;
	ShapeDef.baseMaterial.rollingResistance = RollingResistance;

	ShapeDef.userData = this;

	// A trigger with overlaps off would report nothing, so force it on there.
	ShapeDef.isSensor = bIsSensor;
	ShapeDef.enableSensorEvents = bIsSensor || bGenerateSensorEvents;
	ShapeDef.enableContactEvents = bGenerateContactEvents;
	ShapeDef.enableHitEvents = bGenerateHitEvents;

	// Opt-in collision filtering; 0/0/0 leaves the default (collide with everything).
	if (CollisionCategory != 0 || CollisionMask != 0 || CollisionGroup != 0)
	{
		b3Filter Filter = b3DefaultFilter();
		if (CollisionCategory != 0)
		{
			Filter.categoryBits = static_cast<uint64>(static_cast<uint32>(CollisionCategory));
		}
		if (CollisionMask != 0)
		{
			Filter.maskBits = static_cast<uint64>(static_cast<uint32>(CollisionMask));
		}
		Filter.groupIndex = CollisionGroup;
		ShapeDef.filter = Filter;
	}

	if (BodyType == EBox3DBodyType::Static)
	{
		const auto Source = static_cast<Box3D::StaticGeometry::ESource>(StaticSource);
		if (Box3D::StaticGeometry::AddStaticShapes(BodyId, ShapeDef, GetOwner(), Source, bInvertMeshWinding, OwnedMeshes))
		{
			return;
		}
	}

	const float M = static_cast<float>(Box3D::UnrealToMeters);

	switch (Shape)
	{
	case EBox3DShape::Sphere:
	{
		b3Sphere Sphere;
		Sphere.center = b3Vec3{ 0.0f, 0.0f, 0.0f };
		Sphere.radius = Radius * M;
		b3CreateSphereShape(BodyId, &ShapeDef, &Sphere);
		break;
	}
	case EBox3DShape::Capsule:
	{
		const float HalfH = HalfHeight * M;
		b3Capsule Capsule;
		Capsule.center1 = b3Vec3{ 0.0f, 0.0f, +HalfH };
		Capsule.center2 = b3Vec3{ 0.0f, 0.0f, -HalfH };
		Capsule.radius = Radius * M;
		b3CreateCapsuleShape(BodyId, &ShapeDef, &Capsule);
		break;
	}
	case EBox3DShape::Convex:
	{
		if (AddConvexShapes(ShapeDef))
		{
			break;
		}
		UE_LOG(LogBox3D, Warning,
			TEXT("%s: Convex shape resolved no usable hull from the root's simple collision; ")
			TEXT("falling back to a bounds box. Add convex simple collision to the mesh (a hull ")
			TEXT("needs 4+ non-coplanar verts), or use a different shape."),
			*GetNameSafe(GetOwner()));
		const b3BoxHull FallbackHull = b3MakeOffsetBoxHull(
			ResolvedHalfExtent.X * M, ResolvedHalfExtent.Y * M, ResolvedHalfExtent.Z * M,
			Box3D::ToBox3DVector(ResolvedBoxCenter));
		b3CreateHullShape(BodyId, &ShapeDef, &FallbackHull.base);
		break;
	}
	default: // Auto / Box
	{
		const b3BoxHull Hull = b3MakeOffsetBoxHull(
			ResolvedHalfExtent.X * M, ResolvedHalfExtent.Y * M, ResolvedHalfExtent.Z * M,
			Box3D::ToBox3DVector(ResolvedBoxCenter));
		b3CreateHullShape(BodyId, &ShapeDef, &Hull.base);
		break;
	}
	}
}

bool UBox3DBodyComponent::AddConvexShapes(const b3ShapeDef& ShapeDef)
{
	TArray<TArray<b3Vec3>> Clouds;
	GatherConvexPointClouds(GetOwner()->GetRootComponent(), bConvexIncludesAttachedChildren, Clouds);

	// One hull shape per element (a compound), matching the mesh's simple collision.
	int32 Created = 0;
	for (const TArray<b3Vec3>& Cloud : Clouds)
	{
		if (Cloud.Num() < 4)
		{
			continue;
		}
		b3HullData* Hull = b3CreateHull(Cloud.GetData(), Cloud.Num(), ConvexMaxHullVertices);
		if (Hull == nullptr)
		{
			continue;
		}
		b3CreateHullShape(BodyId, &ShapeDef, Hull); // box3d clones the hull; free ours after
		b3DestroyHull(Hull);
		++Created;
	}

	return Created > 0;
}

void UBox3DBodyComponent::BuildConvexDebugGeometry()
{
	ConvexDebugSegments.Reset();

	TArray<TArray<b3Vec3>> Clouds;
	GatherConvexPointClouds(GetOwner()->GetRootComponent(), bConvexIncludesAttachedChildren, Clouds);

	for (const TArray<b3Vec3>& Cloud : Clouds)
	{
		if (Cloud.Num() < 4)
		{
			continue;
		}
		b3HullData* Hull = b3CreateHull(Cloud.GetData(), Cloud.Num(), ConvexMaxHullVertices);
		if (Hull == nullptr)
		{
			continue;
		}

		const b3Vec3* Points = b3GetHullPoints(Hull);
		const b3HullHalfEdge* Edges = b3GetHullEdges(Hull);
		if (Points != nullptr && Edges != nullptr)
		{
			for (int32 E = 0; E < Hull->edgeCount; ++E)
			{
				if (E < Edges[E].twin)
				{
					// Points carry the baked scale; convert back to local Unreal cm.
					ConvexDebugSegments.Add(Box3D::FromBox3DVector(Points[Edges[E].origin]));
					ConvexDebugSegments.Add(Box3D::FromBox3DVector(Points[Edges[Edges[E].twin].origin]));
				}
			}
		}

		b3DestroyHull(Hull);
	}
}

void UBox3DBodyComponent::DestroyBody()
{
	if (B3_IS_NON_NULL(BodyId) && Subsystem != nullptr && Subsystem->IsWorldValid())
	{
		Subsystem->FlushAsyncStep();
		b3DestroyBody(BodyId);
	}
	BodyId = b3_nullBodyId;

	// Free tri-mesh data after the body: its mesh shapes referenced this memory.
	for (b3MeshData* Mesh : OwnedMeshes)
	{
		if (Mesh != nullptr)
		{
			b3DestroyMesh(Mesh);
		}
	}
	OwnedMeshes.Reset();
}

void UBox3DBodyComponent::EnforceAuthorityContract()
{
	UPrimitiveComponent* Root = Cast<UPrimitiveComponent>(GetOwner()->GetRootComponent());
	if (Root == nullptr)
	{
		return;
	}

	bRestoreChaosSimulation = Root->IsSimulatingPhysics();
	Root->SetSimulatePhysics(false);

	if (Root->Mobility != EComponentMobility::Movable)
	{
		UE_LOG(LogBox3D, Warning,
			TEXT("%s: root is not Movable; its transform can't change at runtime. Set Mobility to Movable."),
			*GetNameSafe(GetOwner()));
	}
}

void UBox3DBodyComponent::ResolveAutoBoxBounds()
{
	ResolvedHalfExtent = FVector(50.0, 50.0, 50.0);
	ResolvedBoxCenter = FVector::ZeroVector;

	USceneComponent* Root = GetOwner()->GetRootComponent();
	if (Root != nullptr)
	{
		const FVector Scale = Root->GetComponentScale();
		const FTransform RootTM = Root->GetComponentTransform();

		FBox Local(ForceInit);
		if (const UPrimitiveComponent* RootPrim = Cast<UPrimitiveComponent>(Root))
		{
			Local += RootPrim->CalcBounds(FTransform::Identity).GetBox();
		}

		if (bConvexIncludesAttachedChildren || !Root->IsA<UPrimitiveComponent>())
		{
			TArray<USceneComponent*> Children;
			Root->GetChildrenComponents(/*bIncludeAllDescendants=*/true, Children);
			for (USceneComponent* Child : Children)
			{
				if (const UPrimitiveComponent* ChildPrim = Cast<UPrimitiveComponent>(Child))
				{
					Local += ChildPrim->CalcBounds(ChildPrim->GetComponentTransform().GetRelativeTransform(RootTM)).GetBox();
				}
			}
		}

		if (Local.IsValid && !Local.GetExtent().IsNearlyZero())
		{
			ResolvedHalfExtent = Local.GetExtent() * Scale;
			ResolvedBoxCenter = Local.GetCenter() * Scale;
			return;
		}
	}

	UE_LOG(LogBox3D, Warning, TEXT("%s: could not derive Auto bounds; using 50cm default."),
		*GetNameSafe(GetOwner()));
}

void UBox3DBodyComponent::CaptureStepTransform()
{
	if (B3_IS_NULL(BodyId))
	{
		return;
	}

	// box3d has no wake event, so poll it here - we already visit every body each step.
	if (bGenerateSleepEvents)
	{
		const bool bAwake = b3Body_IsAwake(BodyId);
		if (bAwake != bWasAwake)
		{
			bWasAwake = bAwake;
			Subsystem->QueueSleepEvent(this, bAwake);
		}
	}

	FTransform NewXform = Box3D::FromBox3DTransform(b3Body_GetTransform(BodyId));
	NewXform.SetScale3D(SpawnScale); // box3d has no scale; keep the actor's.

	PrevTransform = CurrTransform;
	CurrTransform = NewXform;
}

void UBox3DBodyComponent::ApplyInterpolatedTransform(float Alpha)
{
	if (BodyType != EBox3DBodyType::Dynamic)
	{
		return;
	}

	const FVector Location = FMath::Lerp(PrevTransform.GetLocation(), CurrTransform.GetLocation(), Alpha);
	FQuat Rotation = FQuat::Slerp(PrevTransform.GetRotation(), CurrTransform.GetRotation(), Alpha);
	Rotation.Normalize();

	GetOwner()->SetActorLocationAndRotation(Location, Rotation, /*bSweep=*/false, nullptr, ETeleportType::TeleportPhysics);
}

void UBox3DBodyComponent::PushKinematicTarget(float TimeStep)
{
	if (B3_IS_NULL(BodyId))
	{
		return;
	}

	const FTransform T = GetOwner()->GetActorTransform();
	b3WorldTransform Target;
	Target.p = Box3D::ToBox3DPosition(T.GetLocation());
	Target.q = Box3D::ToBox3DQuat(T.GetRotation());
	b3Body_SetTargetTransform(BodyId, Target, TimeStep, /*wake=*/true);
}

void UBox3DBodyComponent::DrawDebug() const
{
	UWorld* World = GetWorld();
	AActor* Owner = GetOwner();
	if (World == nullptr || Owner == nullptr)
	{
		return;
	}

	const FTransform T = Owner->GetActorTransform();
	const FVector Location = T.GetLocation();
	const FQuat Rotation = T.GetRotation();

	// Colour by type: dynamic = green, static = cyan, kinematic = yellow.
	FColor Color = FColor::Green;
	switch (BodyType)
	{
	case EBox3DBodyType::Static:    Color = FColor::Cyan;   break;
	case EBox3DBodyType::Kinematic: Color = FColor::Yellow; break;
	default: break;
	}

	switch (Shape)
	{
	case EBox3DShape::Sphere:
		DrawDebugSphere(World, Location, Radius, 16, Color, false, -1.0f, 0, 1.0f);
		break;
	case EBox3DShape::Capsule:
		DrawDebugCapsule(World, Location, HalfHeight + Radius, Radius, Rotation, Color, false, -1.0f, 0, 1.0f);
		break;
	case EBox3DShape::Convex:
		if (ConvexDebugSegments.Num() >= 2)
		{
			for (int32 i = 0; i + 1 < ConvexDebugSegments.Num(); i += 2)
			{
				DrawDebugLine(World,
					Location + Rotation.RotateVector(ConvexDebugSegments[i]),
					Location + Rotation.RotateVector(ConvexDebugSegments[i + 1]),
					Color, false, -1.0f, 0, 1.0f);
			}
		}
		else // no convex collision resolved; the shape fell back to a box
		{
			DrawDebugBox(World, Location + Rotation.RotateVector(ResolvedBoxCenter),
				ResolvedHalfExtent, Rotation, Color, false, -1.0f, 0, 1.0f);
		}
		break;
	default: // Auto / Box
		DrawDebugBox(World, Location + Rotation.RotateVector(ResolvedBoxCenter),
			ResolvedHalfExtent, Rotation, Color, false, -1.0f, 0, 1.0f);
		break;
	}
}
