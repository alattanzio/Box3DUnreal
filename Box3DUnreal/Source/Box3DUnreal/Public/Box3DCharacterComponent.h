// Author: Antonio Lattanzio - emptyvessel

#pragma once

#include "Components/ActorComponent.h"
#include "CoreMinimal.h"
#include <box3d/box3d.h>
#include "Box3DCharacterComponent.generated.h"

class UBox3DSubsystem;

/** Ground state, for animation and gameplay. */
UENUM(BlueprintType)
enum class EBox3DCharacterState : uint8
{
	Grounded,
	Airborne
};

/**
 * Kinematic capsule character driven by box3d's mover API.
 *
 * Ported from box3d's sample_character: each step gathers collision planes with
 * b3World_CollideMover, resolves them with b3SolvePlanes, then sweeps with
 * b3World_CastMover. Ground contact is a spring ("pogo") ray rather than a snap, which is
 * what gives smooth stair and slope handling without a separate step-up pass.
 *
 * Movement is Quake-style: friction, then acceleration toward a desired velocity capped at
 * MaxSpeed. It runs on the subsystem's fixed timestep, so it is deterministic and fits the
 * rollback engine - unlike CharacterMovementComponent.
 *
 * Server-authoritative: on a client this does nothing.
 */
UCLASS(ClassGroup = (Physics), meta = (BlueprintSpawnableComponent, DisplayName = "Box3D Character"))
class BOX3DUNREAL_API UBox3DCharacterComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UBox3DCharacterComponent();

	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

	/** Movement intent for the next step, in world space. Magnitude is clamped to 1;
	 *  call every frame like AddMovementInput. */
	UFUNCTION(BlueprintCallable, Category = "Box3D|Character")
	void SetMoveInput(const FVector& WorldDirection);

	/** Jump on the next step, if grounded. Cleared once consumed. */
	UFUNCTION(BlueprintCallable, Category = "Box3D|Character")
	void Jump();

	UFUNCTION(BlueprintCallable, Category = "Box3D|Character")
	void SetSprinting(bool bSprinting) { bSprint = bSprinting; }

	UFUNCTION(BlueprintPure, Category = "Box3D|Character")
	bool IsOnGround() const { return bOnGround; }

	UFUNCTION(BlueprintPure, Category = "Box3D|Character")
	EBox3DCharacterState GetState() const;

	/** Velocity in cm/s. */
	UFUNCTION(BlueprintPure, Category = "Box3D|Character")
	FVector GetVelocity() const;

	/** Move the character, clearing velocity. */
	UFUNCTION(BlueprintCallable, Category = "Box3D|Character")
	void TeleportTo(const FVector& WorldLocation);

	/** Subsystem hook: advance one fixed step. */
	void SolveMove(float TimeStep);

	/** Subsystem hook: write the solved position to the owning actor. */
	void ApplyToActor();

	// --- Capsule ------------------------------------------------------------------------

	/** Radius in cm. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Character|Shape", meta = (ClampMin = "1.0"))
	float Radius = 34.0f;

	/** Half the distance between the hemisphere centres, in cm. Total height is
	 *  2*(HalfHeight + Radius). */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Character|Shape", meta = (ClampMin = "0.0"))
	float HalfHeight = 44.0f;

	// --- Movement -----------------------------------------------------------------------

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Character|Movement", meta = (ClampMin = "0.0"))
	float MaxSpeed = 600.0f;

	/** Multiplier applied to MaxSpeed while sprinting. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Character|Movement", meta = (ClampMin = "1.0"))
	float SprintMultiplier = 1.5f;

	/** Acceleration as a multiple of MaxSpeed per second. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Character|Movement", meta = (ClampMin = "0.0"))
	float Acceleration = 30.0f;

	/** Ground friction, 1/s. 0 makes the character slide like ice. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Character|Movement", meta = (ClampMin = "0.0"))
	float Friction = 4.0f;

	/** Speed below which friction stops scaling and becomes a flat reduction, cm/s. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Character|Movement", meta = (ClampMin = "0.0"))
	float StopSpeed = 100.0f;

	/** Take-off speed in cm/s. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Character|Movement", meta = (ClampMin = "0.0"))
	float JumpSpeed = 500.0f;

	/** Gravity in cm/s^2, positive down. Separate from world gravity so character feel can
	 *  be tuned without touching the simulation. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Character|Movement", meta = (ClampMin = "0.0"))
	float Gravity = 1500.0f;

	// --- Ground spring ------------------------------------------------------------------

	/** Ride height as a multiple of Radius: how far the capsule floats above the ground.
	 *  This is what absorbs stairs, so it also sets the effective step height. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Character|Ground", meta = (ClampMin = "0.1"))
	float PogoRestLengthScale = 3.0f;

	/** Spring rate, Hz. Higher is stiffer and less bouncy on steps. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Character|Ground", meta = (ClampMin = "0.1"))
	float PogoHertz = 4.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Character|Ground", meta = (ClampMin = "0.0"))
	float PogoDampingRatio = 0.7f;

	/** Push dynamic bodies the character walks into. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Character|Ground")
	bool bPushDynamicBodies = true;

	/** Draw the capsule, ground ray and collision planes. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Character|Debug")
	bool bDrawDebug = false;

	/** Collision planes gathered this step, filled by the b3World_CollideMover callback.
	 *  Public because that callback is a free function. */
	static constexpr int32 PlaneCapacity = 8;

	/** Where a plane came from, for the dynamic-body push pass. */
	struct FPlaneSource
	{
		b3Pos Point;
		b3ShapeId Shape;
	};

protected:
	/** Ground spring: pushes the capsule to its rest height. Sets bOnGround. */
	void UpdateGroundSpring(float TimeStep);

	/** Resolve the target position against the gathered planes, sweeping as it goes. */
	void ResolveMotion(const b3Pos& Target);

	/** Apply an equal-and-opposite impulse to dynamic bodies the character pushed. */
	void PushTouchedBodies();

	void DrawDebugState() const;

public:
	// Written by the box3d plane callback, which is a free function.
	b3CollisionPlane Planes[PlaneCapacity] = {};
	FPlaneSource PlaneSources[PlaneCapacity] = {};
	int32 PlaneCount = 0;

	/** Capsule in box3d units, centred on the body origin. */
	b3Capsule GetMoverCapsule() const;

	/** Current position, box3d space. */
	b3Pos GetMoverPosition() const { return Position; }

private:
	UPROPERTY(Transient)
	TObjectPtr<UBox3DSubsystem> Subsystem = nullptr;

	/** Position and velocity in box3d space; the actor is written from these. */
	b3Pos Position = {};
	b3Vec3 Velocity = {};

	/** Ground spring velocity, kept between steps. */
	float PogoVelocity = 0.0f;

	/** Move intent in box3d space, magnitude <= 1. Cleared after each step. */
	b3Vec3 MoveInput = {};

	bool bOnGround = false;
	bool bSprint = false;
	bool bJumpQueued = false;

	/** Eligibility, resolved in BeginPlay. False on clients. */
	bool bSimulationEligible = false;

	/** Interpolation endpoints, same scheme as the body component. */
	FVector PrevLocation = FVector::ZeroVector;
	FVector CurrLocation = FVector::ZeroVector;
};
