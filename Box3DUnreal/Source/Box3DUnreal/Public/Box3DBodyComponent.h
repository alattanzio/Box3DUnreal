// Author: Antonio Lattanzio - emptyvessel

#pragma once

#include "CoreMinimal.h"
#include "Box3DEventTypes.h"
#include "Components/ActorComponent.h"
#include <box3d/box3d.h>
#include "Box3DBodyComponent.generated.h"

class UBox3DSubsystem;

UENUM(BlueprintType)
enum class EBox3DBodyType : uint8
{
	Static,     // Level geometry: created once, never moved.
	Kinematic,  // Driven by gameplay (push is a later milestone).
	Dynamic     // Simulated by box3d; drives the owning actor.
};

UENUM(BlueprintType)
enum class EBox3DShape : uint8
{
	Auto,    // Box derived from the root primitive's local bounds.
	Box,
	Sphere,
	Capsule,
	Convex   // Hull(s) from the mesh's simple convex collision (falls back to a box).
};

// Which cooked collision a Static body mirrors. UENUM mirror of StaticGeometry::ESource.
UENUM(BlueprintType)
enum class EBox3DStaticSource : uint8
{
	Auto,             // Complex tri-mesh if present, else simple.
	SimpleCollision,  // AggGeom convex/box/sphere/capsule.
	ComplexCollision  // Cooked tri-mesh (meshes & landscape).
};

/**
 * Add to any actor to give it a box3d rigid body. On BeginPlay it creates the body
 * at the actor's transform and, for Dynamic bodies, registers with UBox3DSubsystem
 * which steps the sim and writes the interpolated result back to the actor.
 */
UCLASS(ClassGroup = (Physics), meta = (BlueprintSpawnableComponent))
class BOX3DUNREAL_API UBox3DBodyComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UBox3DBodyComponent();

	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

	b3BodyId GetBodyId() const { return BodyId; }

	/** Subsystem hook: build this component's box3d body if it is eligible and none
	 *  exists yet. Called from BeginPlay and on a runtime box3d.Enabled -> on toggle. */
	void RebuildSimulationBody();
	/** Subsystem hook: destroy the box3d body but stay registered for a later rebuild. */
	void TeardownSimulationBody();

	/** Subsystem hook: roll prev<-curr and read the new post-step transform. */
	void CaptureStepTransform();
	/** Subsystem hook: write Lerp(prev, curr, Alpha) to the owning actor. */
	void ApplyInterpolatedTransform(float Alpha);
	/** Manual kinematic push. The step loop uses GatherKinematicTargets instead. */
	void PushKinematicTarget(float TimeStep);
	/** Subsystem hook: draw this body's shape at the owning actor's current pose. */
	void DrawDebug() const;


	/** One-shot linear impulse through the centre of mass, in Unreal kg*cm/s. */
	UFUNCTION(BlueprintCallable, Category = "Box3D|Physics")
	void AddImpulse(const FVector& Impulse, bool bWake = true);

	/** Impulse at a world point (cm), so it also spins the body. */
	UFUNCTION(BlueprintCallable, Category = "Box3D|Physics")
	void AddImpulseAtLocation(const FVector& Impulse, const FVector& WorldLocation, bool bWake = true);

	/** Force through the centre of mass, kg*cm/s^2. Lasts one step - call it every frame. */
	UFUNCTION(BlueprintCallable, Category = "Box3D|Physics")
	void AddForce(const FVector& Force, bool bWake = true);

	/** Torque about the world axes, kg*cm^2/s^2 (same units as Chaos' AddTorqueInRadians). */
	UFUNCTION(BlueprintCallable, Category = "Box3D|Physics")
	void AddTorque(const FVector& Torque, bool bWake = true);

	/** Angular impulse about the world axes, kg*cm^2/s. */
	UFUNCTION(BlueprintCallable, Category = "Box3D|Physics")
	void AddAngularImpulse(const FVector& AngularImpulse, bool bWake = true);

	/** Set the linear velocity directly, in Unreal cm/s. */
	UFUNCTION(BlueprintCallable, Category = "Box3D|Physics")
	void SetLinearVelocity(const FVector& Velocity, bool bWake = true);

	/** Set the angular velocity directly, in rad/s about the Unreal world axes. */
	UFUNCTION(BlueprintCallable, Category = "Box3D|Physics")
	void SetAngularVelocity(const FVector& AngularVelocity, bool bWake = true);

	/** Multiplier on world gravity. 0 floats, negative falls upward. */
	UFUNCTION(BlueprintCallable, Category = "Box3D|Physics")
	void SetGravityScale(float Scale);

	/** Off keeps the body simulating forever. On (default) lets it rest and cost nothing. */
	UFUNCTION(BlueprintCallable, Category = "Box3D|Physics")
	void SetSleepEnabled(bool bEnabled);

	/** Move the body and clear its velocity. The actor snaps instead of sweeping. */
	UFUNCTION(BlueprintCallable, Category = "Box3D|Physics")
	void TeleportBody(const FVector& Location, const FRotator& Rotation);


	/** False on clients - check it before trusting the getters below. */
	UFUNCTION(BlueprintPure, Category = "Box3D|Physics")
	bool HasSimulationBody() const { return B3_IS_NON_NULL(BodyId); }

	/** Linear velocity in cm/s. */
	UFUNCTION(BlueprintPure, Category = "Box3D|Physics")
	FVector GetLinearVelocity() const;

	/** Angular velocity in rad/s about the world axes. */
	UFUNCTION(BlueprintPure, Category = "Box3D|Physics")
	FVector GetAngularVelocity() const;

	/** Solved body mass in kg (density x shape volume). */
	UFUNCTION(BlueprintPure, Category = "Box3D|Physics")
	float GetBodyMass() const;

	/** False once box3d has put the body to sleep. */
	UFUNCTION(BlueprintPure, Category = "Box3D|Physics")
	bool IsBodyAwake() const;

	/** Wake a sleeping body so it resumes stepping (gravity, contacts). */
	UFUNCTION(BlueprintCallable, Category = "Box3D|Physics")
	void WakeBody();

	/**
	 * Destroy this body's shapes and rebuild them from the current geometry, keeping the
	 * body (and its transform and velocity) alive. Needed when the collision that fed the
	 * shape changed after creation — most of all for a bConvexIncludesAttachedChildren
	 * compound that just lost or gained a child. Mass is recomputed from the new shapes.
	 */
	UFUNCTION(BlueprintCallable, Category = "Box3D|Physics")
	void RebuildShapes();


	/** Fire hit events for hard collisions - impact sounds, damage, decals. Only one of the
	 *  two bodies has to have it on. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Events",
		meta = (DisplayName = "Simulation Generates Hit Events"))
	bool bGenerateHitEvents = false;

	/** Fire begin/end contact events for every touch, however gentle. Noisier than hit
	 *  events - prefer those unless you need the touch itself. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Events",
		meta = (DisplayName = "Generate Contact Events"))
	bool bGenerateContactEvents = false;

	/** Take part in overlaps. Needed on the trigger AND on whatever should be seen by it. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Events",
		meta = (DisplayName = "Generate Overlap Events"))
	bool bGenerateSensorEvents = false;

	/** Turn this body into a trigger volume: it reports overlaps and never blocks anything.
	 *  Use Static or Kinematic - a Dynamic trigger just falls out of the level. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Events",
		meta = (DisplayName = "Is Trigger"))
	bool bIsSensor = false;

	/** Fire sleep/wake events. Handy for switching off VFX or audio on settled debris. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Events",
		meta = (DisplayName = "Generate Sleep Events"))
	bool bGenerateSleepEvents = false;

	UPROPERTY(BlueprintAssignable, Category = "Box3D|Events")
	FBox3DHitSignature OnBox3DHit;

	UPROPERTY(BlueprintAssignable, Category = "Box3D|Events")
	FBox3DTouchSignature OnBox3DBeginContact;

	UPROPERTY(BlueprintAssignable, Category = "Box3D|Events")
	FBox3DTouchSignature OnBox3DEndContact;

	/** Fires on the trigger, with whatever entered in Other. */
	UPROPERTY(BlueprintAssignable, Category = "Box3D|Events")
	FBox3DTouchSignature OnBox3DBeginOverlap;

	/** Fires on the trigger. Other can be null when it left by being destroyed. */
	UPROPERTY(BlueprintAssignable, Category = "Box3D|Events")
	FBox3DTouchSignature OnBox3DEndOverlap;

	UPROPERTY(BlueprintAssignable, Category = "Box3D|Events")
	FBox3DSleepSignature OnBox3DSleep;

	UPROPERTY(BlueprintAssignable, Category = "Box3D|Events")
	FBox3DSleepSignature OnBox3DWake;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D")
	EBox3DBodyType BodyType = EBox3DBodyType::Dynamic;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D")
	EBox3DShape Shape = EBox3DShape::Auto;

	/** Box half-extents in cm (used when Shape == Box). */
	UPROPERTY(EditAnywhere, Category = "Box3D|Shape",
		meta = (EditCondition = "Shape == EBox3DShape::Box", EditConditionHides))
	FVector BoxHalfExtent = FVector(50.0, 50.0, 50.0);

	/** Radius in cm (used when Shape == Sphere or Capsule). */
	UPROPERTY(EditAnywhere, Category = "Box3D|Shape", meta = (ClampMin = "0.0",
		EditCondition = "Shape == EBox3DShape::Sphere || Shape == EBox3DShape::Capsule", EditConditionHides))
	float Radius = 50.0f;

	/**
	 * Convex only: also build hulls from every primitive attached under the root, not just
	 * the root itself — one compound body for a group of separate meshes. Use it when
	 * several components should move as one rigid piece (a welded cluster of fracture
	 * shards); leave off for an ordinary single-mesh actor.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Shape",
		meta = (EditCondition = "Shape == EBox3DShape::Convex", EditConditionHides))
	bool bConvexIncludesAttachedChildren = false;

	/** Half the distance between the capsule's hemisphere centers, in cm. */
	UPROPERTY(EditAnywhere, Category = "Box3D|Shape", meta = (ClampMin = "0.0",
		EditCondition = "Shape == EBox3DShape::Capsule", EditConditionHides))
	float HalfHeight = 50.0f;

	/** Static bodies only: which cooked collision to mirror. */
	UPROPERTY(EditAnywhere, Category = "Box3D|Static",
		meta = (EditCondition = "BodyType == EBox3DBodyType::Static", EditConditionHides))
	EBox3DStaticSource StaticSource = EBox3DStaticSource::Auto;

	/** Flip if dynamics fall through this mesh (one-sided triangles wound the wrong way). */
	UPROPERTY(EditAnywhere, Category = "Box3D|Static",
		meta = (EditCondition = "BodyType == EBox3DBodyType::Static", EditConditionHides))
	bool bInvertMeshWinding = false;

	/** Density in kg/m^3 (water ~= 1000). */
	UPROPERTY(EditAnywhere, Category = "Box3D|Material", meta = (ClampMin = "0.0"))
	float Density = 1000.0f;

	UPROPERTY(EditAnywhere, Category = "Box3D|Material", meta = (ClampMin = "0.0"))
	float Friction = 0.6f;

	UPROPERTY(EditAnywhere, Category = "Box3D|Material", meta = (ClampMin = "0.0"))
	float Restitution = 0.0f;

	/** Rolling resistance for spheres/capsules (ignored by other shapes). */
	UPROPERTY(EditAnywhere, Category = "Box3D|Material", meta = (ClampMin = "0.0"))
	float RollingResistance = 0.0f;

	UPROPERTY(EditAnywhere, Category = "Box3D|Damping", meta = (ClampMin = "0.0"))
	float LinearDamping = 0.0f;

	UPROPERTY(EditAnywhere, Category = "Box3D|Damping", meta = (ClampMin = "0.0"))
	float AngularDamping = 0.05f;

	/** Category bitmask this body belongs to. 0 = collide with everything. */
	UPROPERTY(EditAnywhere, Category = "Box3D|Collision", meta = (Bitmask))
	int32 CollisionCategory = 0;

	/** Categories this body collides with. 0 = collide with everything. */
	UPROPERTY(EditAnywhere, Category = "Box3D|Collision", meta = (Bitmask))
	int32 CollisionMask = 0;

	/** box3d group index: >0 always collide, <0 never collide within the group, 0 = off. */
	UPROPERTY(EditAnywhere, Category = "Box3D|Collision")
	int32 CollisionGroup = 0;

protected:
	/** Join the step task before touching this body's box3d state. */
	void FenceAsyncStep() const;

	void CreateBody();
	void AddShape();
	/** Attach convex hull shape(s) from the mesh's simple collision. Returns false if
	 *  the mesh has no convex/box simple collision to build a hull from. */
	bool AddConvexShapes(const b3ShapeDef& ShapeDef);
	/** Cache the convex hull wireframe (local Unreal space) for debug draw. Runs on
	 *  server and client so both can show the shape; independent of body creation. */
	void BuildConvexDebugGeometry();
	void DestroyBody();
	void EnforceAuthorityContract();
	void EnableReplication();
	bool ComputeSimulationEligibility();

	/** Resolve ResolvedHalfExtent + ResolvedBoxCenter from the root's local bounds. */
	void ResolveAutoBoxBounds();

private:
	UPROPERTY(Transient)
	TObjectPtr<UBox3DSubsystem> Subsystem = nullptr;

	b3BodyId BodyId = b3_nullBodyId;

	/** Net-role eligibility, resolved once in BeginPlay. Independent of the master switch
	 *  and world validity so a runtime enable can still build this body. */
	bool bSimulationEligible = false;

	/** Whether the root was simulating Chaos physics before box3d took it over. Restored
	 *  on teardown so a runtime disable hands the actor back instead of freezing it. */
	bool bRestoreChaosSimulation = false;

	/** box3d carries no scale, so we preserve the spawn scale when writing back. */
	FVector SpawnScale = FVector::OneVector;

	/** Tri-mesh data referenced (not cloned) by static shapes; freed after the body. */
	TArray<b3MeshData*> OwnedMeshes;

	/** Resolved box half-extents (cm) used for Auto/Box shapes; for debug draw. */
	FVector ResolvedHalfExtent = FVector(50.0, 50.0, 50.0);

	/** Local-space centre (cm) of the Auto / convex-fallback box. Non-zero whenever the
	 *  root's geometry is offset from its own origin — the normal case for fracture
	 *  pieces, whose verts stay in the source actor's space. Zero for an explicit Box. */
	FVector ResolvedBoxCenter = FVector::ZeroVector;

	/** Convex hull wireframe as line-list pairs in local Unreal space (cm, scale baked).
	 *  Consecutive elements (2i, 2i+1) are one edge's endpoints. Empty unless Shape==Convex. */
	TArray<FVector> ConvexDebugSegments;

	/** Interpolation endpoints in Unreal space (last two fixed steps). */
	FTransform PrevTransform = FTransform::Identity;
	FTransform CurrTransform = FTransform::Identity;

	/** Last awake state, for the sleep/wake events. Bodies are created awake. */
	bool bWasAwake = true;

	/** Force/velocity requested before the body existed, flushed once it is created. */
	FVector PendingLinearImpulse = FVector::ZeroVector;
	FVector PendingAngularVelocity = FVector::ZeroVector;
	bool bHasPendingAngularVelocity = false;
};
