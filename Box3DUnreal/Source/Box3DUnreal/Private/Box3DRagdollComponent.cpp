// Author: Antonio Lattanzio - emptyvessel

#include "Box3DRagdollComponent.h"
#include "Box3DConversion.h"
#include "Box3DJoints.h"
#include "Box3DLog.h"
#include "Box3DSubsystem.h"
#include "Components/SkeletalMeshComponent.h"
#include "Engine/SkeletalMesh.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "PhysicsEngine/BodySetup.h"
#include "PhysicsEngine/ConstraintInstance.h"
#include "PhysicsEngine/PhysicsAsset.h"
#include "PhysicsEngine/PhysicsConstraintTemplate.h"
#include "PhysicsEngine/SkeletalBodySetup.h"

namespace
{
	// box3d caps twist at 0.99*pi.
	constexpr float MaxTwistRadians = 0.99f * PI;

	// The asset's cone is elliptical, box3d's circular: take the larger axis.
	float ResolveConeAngle(const FConstraintInstance& Constraint, float Scale)
	{
		const EAngularConstraintMotion Swing1 = Constraint.GetAngularSwing1Motion();
		const EAngularConstraintMotion Swing2 = Constraint.GetAngularSwing2Motion();

		if (Swing1 == ACM_Locked && Swing2 == ACM_Locked)
		{
			return 0.0f;
		}
		if (Swing1 == ACM_Free || Swing2 == ACM_Free)
		{
			return PI;
		}

		const float Degrees = FMath::Max(Constraint.GetAngularSwing1Limit(), Constraint.GetAngularSwing2Limit());
		return FMath::Clamp(FMath::DegreesToRadians(Degrees * Scale), 0.0f, PI);
	}
}

UBox3DRagdollComponent::UBox3DRagdollComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	// After the animation update, so our pose is what actually renders.
	PrimaryComponentTick.TickGroup = TG_PostPhysics;
}

void UBox3DRagdollComponent::BeginPlay()
{
	Super::BeginPlay();

	AActor* Owner = GetOwner();
	if (Owner == nullptr)
	{
		return;
	}

	if (SkeletalMeshComponentName.IsNone())
	{
		Mesh = Owner->FindComponentByClass<USkeletalMeshComponent>();
	}
	else
	{
		TArray<USkeletalMeshComponent*> Candidates;
		Owner->GetComponents(Candidates);
		for (USkeletalMeshComponent* Candidate : Candidates)
		{
			if (Candidate->GetFName() == SkeletalMeshComponentName)
			{
				Mesh = Candidate;
				break;
			}
		}
	}

	if (Mesh == nullptr)
	{
		UE_LOG(LogBox3D, Warning, TEXT("Box3D ragdoll on %s found no skeletal mesh component."),
			*Owner->GetName());
		return;
	}

	if (bStartActive)
	{
		StartRagdoll();
	}
}

void UBox3DRagdollComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	StopRagdoll();
	Super::EndPlay(EndPlayReason);
}

void UBox3DRagdollComponent::StartRagdoll()
{
	if (bActive || Mesh == nullptr)
	{
		return;
	}

	UWorld* World = GetWorld();
	Subsystem = World != nullptr ? World->GetSubsystem<UBox3DSubsystem>() : nullptr;
	if (Subsystem == nullptr || !Subsystem->IsWorldValid())
	{
		return; // client, or box3d disabled: leave the mesh animating
	}

	if (Mesh->GetPhysicsAsset() == nullptr)
	{
		UE_LOG(LogBox3D, Warning, TEXT("Box3D ragdoll: %s has no physics asset."), *Mesh->GetName());
		return;
	}

	// Bones must be posed before we read them: the bodies spawn at the current pose.
	Mesh->RefreshBoneTransforms();

	if (BuildBodies(Mesh, Subsystem) == 0)
	{
		UE_LOG(LogBox3D, Warning, TEXT("Box3D ragdoll: no usable bodies in %s's physics asset."),
			*Mesh->GetName());
		return;
	}
	BuildJoints(Mesh, Subsystem);

	// Stop the animation graph writing over the pose we are about to drive.
	Mesh->bPauseAnims = true;

	// PoseMeshFromBodies writes directly to the read buffer.
	Mesh->SetComponentSpaceTransformsDoubleBuffering(false);

	bActive = true;
	UE_LOG(LogBox3D, Log, TEXT("Box3D ragdoll started on %s: %d bodies, %d joints."),
		*Mesh->GetName(), Bones.Num(), Joints.Num());
}

int32 UBox3DRagdollComponent::BuildBodies(USkeletalMeshComponent* InMesh, UBox3DSubsystem* InSubsystem)
{
	const UPhysicsAsset* Asset = InMesh->GetPhysicsAsset();
	const FTransform ComponentToWorld = InMesh->GetComponentTransform();
	const b3WorldId WorldId = InSubsystem->GetWorldId();

	const float UniformScale = static_cast<float>(ComponentToWorld.GetScale3D().GetAbsMax());
	const float ToMeters = static_cast<float>(Box3D::UnrealToMeters) * UniformScale;

	const int32 SelfCollisionGroup = -1 - static_cast<int32>(GetUniqueID() & 0x7FFF);

	Bones.Reserve(Asset->SkeletalBodySetups.Num());

	for (const TObjectPtr<USkeletalBodySetup>& Setup : Asset->SkeletalBodySetups)
	{
		if (Setup == nullptr)
		{
			continue;
		}

		const int32 BoneIndex = InMesh->GetBoneIndex(Setup->BoneName);
		if (BoneIndex == INDEX_NONE)
		{
			continue; // asset references a bone this mesh doesn't have
		}

		const FKAggregateGeom& Agg = Setup->AggGeom;
		if (Agg.GetElementCount() == 0)
		{
			continue;
		}

		const FTransform BoneWorld = InMesh->GetBoneTransform(BoneIndex, ComponentToWorld);

		b3BodyDef BodyDef = b3DefaultBodyDef();
		BodyDef.type = b3_dynamicBody;
		BodyDef.position = Box3D::ToBox3DPosition(BoneWorld.GetLocation());
		BodyDef.rotation = Box3D::ToBox3DQuat(BoneWorld.GetRotation());
		BodyDef.angularDamping = AngularDamping;
		BodyDef.userData = GetOwner();

		const b3BodyId BodyId = b3CreateBody(WorldId, &BodyDef);
		if (B3_IS_NULL(BodyId))
		{
			continue;
		}

		b3ShapeDef ShapeDef = b3DefaultShapeDef();
		ShapeDef.density = Box3D::ToBox3DDensity(Density);
		ShapeDef.baseMaterial.friction = Friction;
		ShapeDef.baseMaterial.restitution = Restitution;

		ShapeDef.filter.groupIndex = SelfCollisionGroup;

		bool bAddedShape = false;

		for (const FKSphylElem& Sphyl : Agg.SphylElems)
		{
			const FVector Axis = Sphyl.Rotation.Quaternion().RotateVector(
				FVector(0.0, 0.0, Sphyl.Length * 0.5));

			b3Capsule Capsule;
			Capsule.center1 = Box3D::ToBox3DVector(Sphyl.Center - Axis) * UniformScale;
			Capsule.center2 = Box3D::ToBox3DVector(Sphyl.Center + Axis) * UniformScale;
			Capsule.radius = Sphyl.Radius * ToMeters;

			b3CreateCapsuleShape(BodyId, &ShapeDef, &Capsule);
			bAddedShape = true;
		}

		for (const FKSphereElem& Sphere : Agg.SphereElems)
		{
			b3Sphere Ball;
			Ball.center = Box3D::ToBox3DVector(Sphere.Center) * UniformScale;
			Ball.radius = Sphere.Radius * ToMeters;
			b3CreateSphereShape(BodyId, &ShapeDef, &Ball);
			bAddedShape = true;
		}

		for (const FKBoxElem& Box : Agg.BoxElems)
		{
			const FVector HalfExtent(Box.X * 0.5, Box.Y * 0.5, Box.Z * 0.5);
			const b3Vec3 Half = Box3D::ToBox3DVector(HalfExtent) * UniformScale;
			const b3Vec3 Offset = Box3D::ToBox3DVector(Box.Center) * UniformScale;

			// Half-extents must be positive; the Y negation flips one of them.
			const b3Transform ShapeTransform{ Offset, Box3D::ToBox3DQuat(Box.Rotation.Quaternion()) };
			const b3BoxHull Hull = b3MakeTransformedBoxHull(
				FMath::Abs(Half.x), FMath::Abs(Half.y), FMath::Abs(Half.z), ShapeTransform);
			b3CreateHullShape(BodyId, &ShapeDef, &Hull.base);
			bAddedShape = true;
		}

		if (!bAddedShape)
		{
			b3DestroyBody(BodyId); // convex-only setups aren't supported here yet
			continue;
		}

		FRagdollBone& Bone = Bones.AddDefaulted_GetRef();
		Bone.Body = BodyId;
		Bone.BoneName = Setup->BoneName;
		Bone.BoneIndex = BoneIndex;
		Bone.BoneToBody = FTransform::Identity; // body is spawned at the bone
	}

	return Bones.Num();
}

int32 UBox3DRagdollComponent::BuildJoints(USkeletalMeshComponent* InMesh, UBox3DSubsystem* InSubsystem)
{
	const UPhysicsAsset* Asset = InMesh->GetPhysicsAsset();
	const FTransform ComponentToWorld = InMesh->GetComponentTransform();
	const b3WorldId WorldId = InSubsystem->GetWorldId();

	Joints.Reserve(Asset->ConstraintSetup.Num());

	for (const TObjectPtr<UPhysicsConstraintTemplate>& Template : Asset->ConstraintSetup)
	{
		if (Template == nullptr)
		{
			continue;
		}

		const FConstraintInstance& Constraint = Template->DefaultInstance;

		// Bone1 is the child, Bone2 the parent.
		const int32 ChildEntry = FindBoneEntry(Constraint.ConstraintBone1);
		const int32 ParentEntry = FindBoneEntry(Constraint.ConstraintBone2);
		if (ChildEntry == INDEX_NONE || ParentEntry == INDEX_NONE)
		{
			continue;
		}

		const FRagdollBone& Child = Bones[ChildEntry];
		const FRagdollBone& Parent = Bones[ParentEntry];

		// Frame 1 is in child-bone space; take its origin and primary axis to world.
		const FTransform ChildBoneWorld = InMesh->GetBoneTransform(Child.BoneIndex, ComponentToWorld);
		const FTransform RefFrame = Constraint.GetRefFrame(EConstraintFrame::Frame1);

		const FVector WorldAnchor = ChildBoneWorld.TransformPosition(RefFrame.GetLocation());
		const FVector TwistAxis = ChildBoneWorld.TransformVectorNoScale(RefFrame.GetUnitAxis(EAxis::X));

		// UE twists about frame X, box3d about frame Z; BuildJointFrames remaps it.
		b3Transform FrameChild, FrameParent;
		Box3D::BuildJointFrames(Child.Body, Parent.Body, WorldAnchor, TwistAxis, FrameChild, FrameParent);

		b3SphericalJointDef Def = b3DefaultSphericalJointDef();
		Def.base.bodyIdA = Parent.Body;
		Def.base.bodyIdB = Child.Body;
		Def.base.localFrameA = FrameParent;
		Def.base.localFrameB = FrameChild;
		Def.base.collideConnected = bCollideConnected;
		Def.base.constraintHertz = ConstraintHertz;
		Def.base.constraintDampingRatio = ConstraintDampingRatio;

		const float ConeAngle = ResolveConeAngle(Constraint, SwingLimitScale);
		Def.enableConeLimit = ConeAngle < PI;
		Def.coneAngle = ConeAngle;

		const EAngularConstraintMotion TwistMotion = Constraint.GetAngularTwistMotion();
		if (TwistMotion == ACM_Free)
		{
			Def.enableTwistLimit = false;
		}
		else
		{
			const float TwistRadians = TwistMotion == ACM_Locked
				? 0.0f
				: FMath::Clamp(FMath::DegreesToRadians(Constraint.GetAngularTwistLimit()), 0.0f, MaxTwistRadians);
			Def.enableTwistLimit = true;
			Def.lowerTwistAngle = -TwistRadians;
			Def.upperTwistAngle = TwistRadians;
		}

		FBox3DJointSettings Settings;
		Settings.bCollideConnected = bCollideConnected;
		Settings.ConstraintHertz = ConstraintHertz;
		Settings.ConstraintDampingRatio = ConstraintDampingRatio;

		const FBox3DJointHandle Handle = InSubsystem->RegisterJoint(
			b3CreateSphericalJoint(WorldId, &Def), EBox3DJointType::Spherical, Settings);
		if (Handle.IsSet())
		{
			Joints.Add(Handle);
		}
	}

	return Joints.Num();
}

int32 UBox3DRagdollComponent::FindBoneEntry(FName BoneName) const
{
	for (int32 Index = 0; Index < Bones.Num(); ++Index)
	{
		if (Bones[Index].BoneName == BoneName)
		{
			return Index;
		}
	}
	return INDEX_NONE;
}

void UBox3DRagdollComponent::TickComponent(float DeltaTime, ELevelTick TickType,
	FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (bActive)
	{
		PoseMeshFromBodies();
	}
}

void UBox3DRagdollComponent::PoseMeshFromBodies()
{
	if (Mesh == nullptr || Bones.Num() == 0)
	{
		return;
	}

	TArray<FTransform>& ComponentSpace = Mesh->GetEditableComponentSpaceTransforms();
	if (ComponentSpace.Num() == 0)
	{
		return;
	}

	// Unsimulated bones keep their last animated pose and follow their parent.
	const FTransform WorldToComponent = Mesh->GetComponentTransform().Inverse();

	for (const FRagdollBone& Bone : Bones)
	{
		if (!ComponentSpace.IsValidIndex(Bone.BoneIndex) || B3_IS_NULL(Bone.Body))
		{
			continue;
		}

		const FTransform BodyWorld = Box3D::FromBox3DTransform(b3Body_GetTransform(Bone.Body));
		ComponentSpace[Bone.BoneIndex] = BodyWorld * WorldToComponent;
	}

	// Component-space edits are already in the read buffer.
	Mesh->UpdateComponentToWorld();
	Mesh->InvalidateCachedBounds();
	Mesh->UpdateBounds();
	Mesh->MarkRenderTransformDirty();
	Mesh->MarkRenderDynamicDataDirty();
}

void UBox3DRagdollComponent::AddImpulseAtLocation(const FVector& Impulse, const FVector& WorldLocation)
{
	if (!bActive)
	{
		return;
	}

	// Pick the body whose origin is nearest the hit; good enough for a shot or explosion.
	const b3Pos Target = Box3D::ToBox3DPosition(WorldLocation);
	b3BodyId Best = b3_nullBodyId;
	double BestDistSq = TNumericLimits<double>::Max();

	for (const FRagdollBone& Bone : Bones)
	{
		if (B3_IS_NULL(Bone.Body))
		{
			continue;
		}
		const b3Pos P = b3Body_GetPosition(Bone.Body);
		const double DistSq = FMath::Square(P.x - Target.x) + FMath::Square(P.y - Target.y)
			+ FMath::Square(P.z - Target.z);
		if (DistSq < BestDistSq)
		{
			BestDistSq = DistSq;
			Best = Bone.Body;
		}
	}

	if (B3_IS_NON_NULL(Best))
	{
		// kg*cm/s -> kg*m/s, matching the body component's impulse units.
		const b3Vec3 B3Impulse = Box3D::ToBox3DVector(Impulse);
		b3Body_ApplyLinearImpulse(Best, B3Impulse, Target, true);
	}
}

void UBox3DRagdollComponent::StopRagdoll()
{
	if (!bActive)
	{
		return;
	}

	// Joints first: destroying a body under a live joint is not valid.
	if (Subsystem != nullptr)
	{
		for (const FBox3DJointHandle& Handle : Joints)
		{
			Subsystem->DestroyJoint(Handle, false);
		}

		if (Subsystem->IsWorldValid())
		{
			for (const FRagdollBone& Bone : Bones)
			{
				if (B3_IS_NON_NULL(Bone.Body))
				{
					b3DestroyBody(Bone.Body);
				}
			}
		}
	}

	Joints.Reset();
	Bones.Reset();
	bActive = false;

	if (Mesh != nullptr)
	{
		Mesh->bPauseAnims = false;
		Mesh->SetComponentSpaceTransformsDoubleBuffering(true);
	}
}
