// Author: Antonio Lattanzio - emptyvessel

#include "Box3DCharacterComponent.h"
#include "Box3DConversion.h"
#include "Box3DLog.h"
#include "Box3DSubsystem.h"
#include "DrawDebugHelpers.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"

namespace
{
	// Solver iterations per step, and the delta below which it stops early (box3d units).
	constexpr int32 SolverIterations = 5;
	constexpr float SolveTolerance = 0.01f;

	// Below this the character is treated as stopped, so friction doesn't jitter it.
	constexpr float MinSpeed = 0.01f;

	const b3QueryFilter MoverFilter{ 1, ~0ull, 0, "box3d_mover" };

	// Gathers the planes b3World_CollideMover finds. Runs once per touched shape.
	bool PlaneResultCallback(b3ShapeId Shape, const b3PlaneResult* Results, int Count, void* Context)
	{
		UBox3DCharacterComponent* Self = static_cast<UBox3DCharacterComponent*>(Context);

		for (int i = 0; i < Count && Self->PlaneCount < UBox3DCharacterComponent::PlaneCapacity; ++i)
		{
			b3CollisionPlane& Plane = Self->Planes[Self->PlaneCount];
			Plane.plane = Results[i].plane;
			Plane.pushLimit = FLT_MAX; // fully rigid
			Plane.push = 0.0f;
			Plane.clipVelocity = true;

			Self->PlaneSources[Self->PlaneCount].Point =
				b3OffsetPos(Self->GetMoverPosition(), Results[i].point);
			Self->PlaneSources[Self->PlaneCount].Shape = Shape;
			++Self->PlaneCount;
		}

		return true; // keep gathering
	}
}

UBox3DCharacterComponent::UBox3DCharacterComponent()
{
	PrimaryComponentTick.bCanEverTick = false; // the subsystem steps us
}

void UBox3DCharacterComponent::BeginPlay()
{
	Super::BeginPlay();

	AActor* Owner = GetOwner();
	UWorld* World = GetWorld();
	if (Owner == nullptr || World == nullptr)
	{
		return;
	}

	// Only the authority simulates, matching the body component's contract.
	bSimulationEligible = Owner->HasAuthority();
	if (!bSimulationEligible)
	{
		return;
	}

	Subsystem = World->GetSubsystem<UBox3DSubsystem>();
	if (Subsystem == nullptr)
	{
		return;
	}

	const FVector Start = Owner->GetActorLocation();
	Position = Box3D::ToBox3DPosition(Start);
	Velocity = b3Vec3_zero;
	PrevLocation = CurrLocation = Start;

	Subsystem->RegisterCharacter(this);
}

void UBox3DCharacterComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	if (Subsystem != nullptr)
	{
		Subsystem->UnregisterCharacter(this);
	}
	Super::EndPlay(EndPlayReason);
}

b3Capsule UBox3DCharacterComponent::GetMoverCapsule() const
{
	const float R = Radius * static_cast<float>(Box3D::UnrealToMeters);
	const float H = HalfHeight * static_cast<float>(Box3D::UnrealToMeters);

	b3Capsule Capsule;
	Capsule.center1 = b3Vec3{ 0.0f, 0.0f, -H };
	Capsule.center2 = b3Vec3{ 0.0f, 0.0f, H };
	Capsule.radius = R;
	return Capsule;
}

void UBox3DCharacterComponent::SetMoveInput(const FVector& WorldDirection)
{
	const FVector Clamped = WorldDirection.SizeSquared() > 1.0
		? WorldDirection.GetSafeNormal()
		: WorldDirection;

	MoveInput = Box3D::ToBox3DDirection(Clamped);
}

void UBox3DCharacterComponent::Jump()
{
	bJumpQueued = true;
}

EBox3DCharacterState UBox3DCharacterComponent::GetState() const
{
	return bOnGround ? EBox3DCharacterState::Grounded : EBox3DCharacterState::Airborne;
}

FVector UBox3DCharacterComponent::GetVelocity() const
{
	return Box3D::FromBox3DVector(Velocity);
}

void UBox3DCharacterComponent::TeleportTo(const FVector& WorldLocation)
{
	Position = Box3D::ToBox3DPosition(WorldLocation);
	Velocity = b3Vec3_zero;
	PogoVelocity = 0.0f;
	PrevLocation = CurrLocation = WorldLocation;

	if (AActor* Owner = GetOwner())
	{
		Owner->SetActorLocation(WorldLocation);
	}
}

void UBox3DCharacterComponent::SolveMove(float TimeStep)
{
	if (Subsystem == nullptr || !Subsystem->IsWorldValid() || TimeStep <= 0.0f)
	{
		return;
	}

	const float ToM = static_cast<float>(Box3D::UnrealToMeters);

	{
		b3Vec3 Horizontal{ Velocity.x, Velocity.y, 0.0f };
		const float Speed = b3Length(Horizontal);
		if (Speed < MinSpeed)
		{
			Velocity.x = 0.0f;
			Velocity.y = 0.0f;
		}
		else if (bOnGround)
		{
			const float Control = FMath::Max(Speed, StopSpeed * ToM);
			const float Drop = Control * Friction * TimeStep;
			const float NewSpeed = FMath::Max(0.0f, Speed - Drop);
			const float Scale = NewSpeed / Speed;
			Velocity.x *= Scale;
			Velocity.y *= Scale;
		}
	}

	// --- Acceleration -----------------------------------------------------------------
	const float TopSpeed = MaxSpeed * ToM * (bSprint ? SprintMultiplier : 1.0f);

	b3Vec3 Desired = b3MulSV(TopSpeed, MoveInput);
	Desired.z = 0.0f;

	float DesiredSpeed = 0.0f;
	const b3Vec3 DesiredDir = b3GetLengthAndNormalize(&DesiredSpeed, Desired);
	DesiredSpeed = FMath::Min(DesiredSpeed, TopSpeed);

	if (DesiredSpeed > 0.0f)
	{
		const float CurrentSpeed = b3Dot(Velocity, DesiredDir);
		const float AddSpeed = DesiredSpeed - CurrentSpeed;
		if (AddSpeed > 0.0f)
		{
			const float AccelSpeed = FMath::Min(Acceleration * TopSpeed * TimeStep, AddSpeed);
			Velocity = b3MulAdd(Velocity, AccelSpeed, DesiredDir);
		}
	}

	if (bJumpQueued && bOnGround)
	{
		Velocity.z = JumpSpeed * ToM;
		PogoVelocity = 0.0f; // don't let the spring fight the jump
		bOnGround = false;
	}
	bJumpQueued = false;

	Velocity.z -= Gravity * ToM * TimeStep;

	UpdateGroundSpring(TimeStep);

	// Target for this step: velocity plus whatever the ground spring wants.
	const b3Pos Start = Position;
	b3Pos Target = Position;
	Target.x += TimeStep * Velocity.x;
	Target.y += TimeStep * Velocity.y;
	Target.z += TimeStep * (Velocity.z + PogoVelocity);

	ResolveMotion(Target);

	if (bPushDynamicBodies)
	{
		PushTouchedBodies();
	}

	// Clip against the planes we hit, so depenetration doesn't add speed.
	Velocity = b3ClipVector(Velocity, Planes, PlaneCount);

	PrevLocation = CurrLocation;
	CurrLocation = Box3D::FromBox3DPosition(Position);

	MoveInput = b3Vec3_zero; // re-supplied every frame, like AddMovementInput
}

void UBox3DCharacterComponent::UpdateGroundSpring(float TimeStep)
{
	const float ToM = static_cast<float>(Box3D::UnrealToMeters);
	const float CapsuleRadius = Radius * ToM;

	const float RestLength = PogoRestLengthScale * CapsuleRadius;
	const float RayLength = RestLength + CapsuleRadius;

	b3Pos Origin = Position;
	Origin.z -= HalfHeight * ToM;

	const b3Vec3 Translation{ 0.0f, 0.0f, -RayLength };
	const b3RayResult Hit = b3World_CastRayClosest(Subsystem->GetWorldId(), Origin, Translation, MoverFilter);

	if (!Hit.hit)
	{
		bOnGround = false;
		PogoVelocity = 0.0f;
		return;
	}

	bOnGround = true;

	// Implicit (unconditionally stable) damped spring toward RestLength.
	const float CurrentLength = Hit.fraction * RayLength;
	const float Omega = 2.0f * PI * PogoHertz;
	const float OmegaH = Omega * TimeStep;

	PogoVelocity = (PogoVelocity - Omega * OmegaH * (CurrentLength - RestLength))
		/ (1.0f + 2.0f * PogoDampingRatio * OmegaH + OmegaH * OmegaH);
}

void UBox3DCharacterComponent::ResolveMotion(const b3Pos& Target)
{
	const b3WorldId WorldId = Subsystem->GetWorldId();
	const b3Capsule Mover = GetMoverCapsule();

	for (int32 Iteration = 0; Iteration < SolverIterations; ++Iteration)
	{
		// Re-gather every iteration: moving may have found new contacts.
		PlaneCount = 0;
		b3World_CollideMover(WorldId, Position, &Mover, MoverFilter, &PlaneResultCallback, this);

		const b3Vec3 TargetDelta{
			static_cast<float>(Target.x - Position.x),
			static_cast<float>(Target.y - Position.y),
			static_cast<float>(Target.z - Position.z) };

		const b3PlaneSolverResult Solved = b3SolvePlanes(TargetDelta, Planes, PlaneCount);

		// Sweep the solved delta so we stop at geometry rather than tunnelling into it.
		const float Fraction = b3World_CastMover(WorldId, Position, &Mover, Solved.delta,
			MoverFilter, nullptr, nullptr);

		const b3Vec3 Delta = b3MulSV(Fraction, Solved.delta);
		Position = b3OffsetPos(Position, Delta);

		if (b3LengthSquared(Delta) < SolveTolerance * SolveTolerance)
		{
			break;
		}
	}
}

void UBox3DCharacterComponent::PushTouchedBodies()
{
	for (int32 Index = 0; Index < PlaneCount; ++Index)
	{
		const b3ShapeId Shape = PlaneSources[Index].Shape;
		const b3BodyId Body = b3Shape_GetBody(Shape);
		if (B3_IS_NULL(Body) || b3Body_GetType(Body) != b3_dynamicBody)
		{
			continue;
		}

		// Normal points out of the shape, so flip it to push away from the character.
		const b3Pos Point = PlaneSources[Index].Point;
		const b3Vec3 Normal = b3Neg(Planes[Index].plane.normal);

		const float InvMass = b3Body_GetInverseMass(Body);
		const b3Matrix3 InvInertia = b3Body_GetWorldInverseRotationalInertia(Body);

		const b3Vec3 R = b3SubPos(Point, b3Body_GetWorldCenter(Body));
		const b3Vec3 RxN = b3Cross(R, Normal);
		const float K = InvMass + b3Dot(RxN, b3MulMV(InvInertia, RxN));
		if (K <= 0.0f)
		{
			continue;
		}

		// Only push when the character is closing on the body.
		const b3Vec3 PointVelocity = b3Add(b3Body_GetLinearVelocity(Body),
			b3Cross(b3Body_GetAngularVelocity(Body), R));
		const float ClosingSpeed = b3Dot(b3Sub(PointVelocity, Velocity), Normal);
		const float Impulse = FMath::Max(-ClosingSpeed / K, 0.0f);
		if (Impulse <= 0.0f)
		{
			continue;
		}

		b3Body_ApplyLinearImpulse(Body, b3MulSV(Impulse, Normal), Point, true);
	}
}

void UBox3DCharacterComponent::ApplyToActor()
{
	AActor* Owner = GetOwner();
	if (Owner == nullptr)
	{
		return;
	}

	Owner->SetActorLocation(CurrLocation, /*bSweep=*/false);

	if (bDrawDebug)
	{
		DrawDebugState();
	}
}

void UBox3DCharacterComponent::DrawDebugState() const
{
	UWorld* World = GetWorld();
	if (World == nullptr)
	{
		return;
	}

	const FColor Color = bOnGround ? FColor::Green : FColor::Yellow;
	DrawDebugCapsule(World, CurrLocation, HalfHeight + Radius, Radius, FQuat::Identity,
		Color, false, -1.0f, 0, 1.0f);

	// Ground ray: where the spring is looking.
	const float RestLength = PogoRestLengthScale * Radius;
	const FVector RayStart = CurrLocation - FVector(0.0, 0.0, HalfHeight);
	DrawDebugLine(World, RayStart, RayStart - FVector(0.0, 0.0, RestLength + Radius),
		Color, false, -1.0f, 0, 1.0f);

	for (int32 Index = 0; Index < PlaneCount; ++Index)
	{
		const FVector Point = Box3D::FromBox3DPosition(PlaneSources[Index].Point);
		const FVector Normal = Box3D::FromBox3DDirection(Planes[Index].plane.normal);
		DrawDebugDirectionalArrow(World, Point, Point + Normal * 30.0, 6.0f, FColor::Magenta,
			false, -1.0f, 0, 1.0f);
	}
}
