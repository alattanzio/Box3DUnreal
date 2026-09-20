// Author: Antonio Lattanzio - emptyvessel

#pragma once

#include "CoreMinimal.h"
#include "Box3DEventTypes.h"
#include "Box3DJointTypes.h"
#include "Box3DQueryTypes.h"
#include "HAL/IConsoleManager.h"
#include "Subsystems/WorldSubsystem.h"
#include "Tasks/Task.h"
#include <box3d/box3d.h>
#include "Box3DSubsystem.generated.h"

class AActor;
class UBox3DBodyComponent;
class UBox3DCharacterComponent;
class UBox3DCollisionData;
class ULevel;

class UBox3DSubsystem;

/** Kicks the async step at TG_PrePhysics. */
USTRUCT()
struct FBox3DKickTickFunction : public FTickFunction
{
	GENERATED_BODY()

	UBox3DSubsystem* Subsystem = nullptr;

	BOX3DUNREAL_API virtual void ExecuteTick(float DeltaTime, ELevelTick TickType,
		ENamedThreads::Type CurrentThread, const FGraphEventRef& CompletionEvent) override;
	BOX3DUNREAL_API virtual FString DiagnosticMessage() override;
};

template<>
struct TStructOpsTypeTraits<FBox3DKickTickFunction> : public TStructOpsTypeTraitsBase2<FBox3DKickTickFunction>
{
	enum { WithCopy = false };
};

/** Joins the async step at TG_PostPhysics, then does the UObject work. */
USTRUCT()
struct FBox3DJoinTickFunction : public FTickFunction
{
	GENERATED_BODY()

	UBox3DSubsystem* Subsystem = nullptr;

	BOX3DUNREAL_API virtual void ExecuteTick(float DeltaTime, ELevelTick TickType,
		ENamedThreads::Type CurrentThread, const FGraphEventRef& CompletionEvent) override;
	BOX3DUNREAL_API virtual FString DiagnosticMessage() override;
};

template<>
struct TStructOpsTypeTraits<FBox3DJoinTickFunction> : public TStructOpsTypeTraitsBase2<FBox3DJoinTickFunction>
{
	enum { WithCopy = false };
};

/** What box3d's own renderer draws. Bitmask behind box3d.NativeDraw. */
enum class EBox3DDrawFlag : int32
{
	Shapes         = 1 << 0,
	Joints         = 1 << 1,
	JointExtras    = 1 << 2,
	Bounds         = 1 << 3,
	Mass           = 1 << 4,
	Sleep          = 1 << 5,
	Contacts       = 1 << 6,
	ContactNormals = 1 << 7,
	ContactForces  = 1 << 8,
	Islands        = 1 << 9,
	GraphColors    = 1 << 10,
};

/**
 * Owns the single box3d world for a UWorld and advances it on a fixed timestep.
 *
 * One instance exists per Game/PIE world, so editor, each PIE session, and
 * standalone each get an isolated simulation with correct create/destroy on the
 * world lifecycle. Dynamic body components register here to be stepped and to have
 * their owning actors driven with render-frame interpolation.
 */
UCLASS(Config = Game)
class BOX3DUNREAL_API UBox3DSubsystem : public UTickableWorldSubsystem
{
	GENERATED_BODY()

public:
	// USubsystem / UWorldSubsystem
	virtual bool DoesSupportWorldType(const EWorldType::Type WorldType) const override;
	virtual void OnWorldBeginPlay(UWorld& InWorld) override;
	virtual void Deinitialize() override;

	// FTickableGameObject (via UTickableWorldSubsystem)
	virtual void Tick(float DeltaTime) override;
	virtual TStatId GetStatId() const override;
	virtual bool IsTickable() const override;

	/** Master switch, shared by the subsystem and every body component. Off means no
	 *  box3d world, bodies or static geometry. Backed by the box3d.Enabled cvar (also
	 *  forced off at launch by -DisableBox3D); toggle at runtime to A/B against no-box3d. */
	static bool IsBox3DEnabled();

	/** The box3d world id. Only valid while IsWorldValid(). */
	b3WorldId GetWorldId() const { return WorldId; }
	bool IsWorldValid() const { return bWorldValid; }

	/** True where box3d simulates: Standalone and servers, never a pure client. Gate any
	 *  Blueprint that reads box3d state on this - on a client everything comes back empty. */
	UFUNCTION(BlueprintPure, Category = "Box3D")
	bool IsSimulationAuthority() const { return bIsAuthority; }

	/** Fixed steps taken since the world was created. Same count from the same start gives
	 *  the same state, so this is the timeline a rollback aligns to. */
	UFUNCTION(BlueprintPure, Category = "Box3D")
	int64 GetSimulationFrame() const { return SimulationFrame; }

	/** World gravity in cm/s^2. */
	UFUNCTION(BlueprintPure, Category = "Box3D")
	FVector GetGravity() const;

	/** Change world gravity, cm/s^2. Bodies already asleep stay put until disturbed. */
	UFUNCTION(BlueprintCallable, Category = "Box3D")
	void SetGravity(const FVector& NewGravity);

	/**
	 * Blast every dynamic body within Radius of Center, scaled by the shape area facing it.
	 * Falloff is the extra distance it decays over past Radius; a negative ImpulsePerArea
	 * implodes. Doesn't touch mesh shapes. Deterministic, so it is safe on a server.
	 */
	UFUNCTION(BlueprintCallable, Category = "Box3D|Physics")
	void ApplyRadialImpulse(const FVector& Center, float Radius, float Falloff, float ImpulsePerArea,
		const FBox3DQueryFilter& Filter);

	/** Every hit in the world, once each - one place to hang impact audio or decals off.
	 *  Per-body events live on UBox3DBodyComponent. */
	UPROPERTY(BlueprintAssignable, Category = "Box3D|Events")
	FBox3DHitSignature OnAnyBox3DHit;

	/** Body-component hook: queue a sleep/wake for dispatch after the step loop. */
	void QueueSleepEvent(UBox3DBodyComponent* Body, bool bAwake);

	/** djb2 digest of every dynamic body's state (transform + velocity), for determinism /
	 *  desync detection. Bodies are folded in a stable order (owner path name) so the
	 *  same world hashes the same across runs. OutBodyCount reports how many contributed.
	 *  NOTE: stable within one build/scenario; a cross-machine desync check additionally needs a
	 *  shared body ordering (networked ids - a D2 concern). Zero if no world / no dynamic bodies. */
	uint32 ComputeWorldStateHash(int32& OutBodyCount) const;


	/** Take ownership of a freshly created box3d joint and return its handle. */
	FBox3DJointHandle RegisterJoint(b3JointId Joint, EBox3DJointType Type, const FBox3DJointSettings& Settings);

	/** The box3d id behind a handle, or b3_nullJointId if it was destroyed. */
	b3JointId ResolveJoint(const FBox3DJointHandle& Handle) const;

	UFUNCTION(BlueprintCallable, Category = "Box3D|Joint")
	void DestroyJoint(FBox3DJointHandle Handle, bool bWakeBodies = true);

	/** Fires when a breakable joint exceeds its force or torque threshold. */
	UPROPERTY(BlueprintAssignable, Category = "Box3D|Joint")
	FBox3DJointBreakSignature OnBox3DJointBreak;

	/** Kinematic characters, stepped after the solve so they see settled geometry. */
	void RegisterCharacter(UBox3DCharacterComponent* Character);
	void UnregisterCharacter(UBox3DCharacterComponent* Character);

	/** All bodies register for debug draw; dynamic ones also register for sync. */
	void RegisterBody(UBox3DBodyComponent* Component);
	void RegisterDynamicBody(UBox3DBodyComponent* Component);
	void RegisterKinematicBody(UBox3DBodyComponent* Component);
	void UnregisterBody(UBox3DBodyComponent* Component);


	/** Closest shape hit along Start->End. Initial overlap at Start is ignored. */
	UFUNCTION(BlueprintCallable, Category = "Box3D|Query")
	bool RaycastClosest(const FVector& Start, const FVector& End, const FBox3DQueryFilter& Filter,
		FBox3DHitResult& OutHit) const;

	/** Every shape hit along Start->End, sorted near to far. */
	UFUNCTION(BlueprintCallable, Category = "Box3D|Query")
	bool RaycastMulti(const FVector& Start, const FVector& End, const FBox3DQueryFilter& Filter,
		TArray<FBox3DHitResult>& OutHits) const;

	/** Broadphase only: actors whose shape bounds *potentially* overlap the box. Cheap, but
	 *  a hit is not an exact overlap - use OverlapSphere/OverlapBox for a narrow-phase test. */
	UFUNCTION(BlueprintCallable, Category = "Box3D|Query")
	bool OverlapAABB(const FVector& Center, const FVector& HalfExtent, const FBox3DQueryFilter& Filter,
		TArray<AActor*>& OutActors) const;

	/** Actors exactly overlapping the sphere. */
	UFUNCTION(BlueprintCallable, Category = "Box3D|Query")
	bool OverlapSphere(const FVector& Center, float Radius, const FBox3DQueryFilter& Filter,
		TArray<AActor*>& OutActors) const;

	/** Actors exactly overlapping the oriented box. */
	UFUNCTION(BlueprintCallable, Category = "Box3D|Query")
	bool OverlapBox(const FVector& Center, const FVector& HalfExtent, const FRotator& Rotation,
		const FBox3DQueryFilter& Filter, TArray<AActor*>& OutActors) const;

	/** Sweep a sphere Start->End and return the first blocking hit. */
	UFUNCTION(BlueprintCallable, Category = "Box3D|Query")
	bool SphereCast(const FVector& Start, const FVector& End, float Radius, const FBox3DQueryFilter& Filter,
		FBox3DHitResult& OutHit) const;

	/** Sweep an oriented box Start->End (no rotation over the sweep) and return the first hit. */
	UFUNCTION(BlueprintCallable, Category = "Box3D|Query")
	bool BoxCast(const FVector& Start, const FVector& End, const FVector& HalfExtent, const FRotator& Rotation,
		const FBox3DQueryFilter& Filter, FBox3DHitResult& OutHit) const;

	// --- Async stepping ---------------------------------------------------

	/** Backed by box3d.AsyncStep; off steps inline from Tick. */
	static bool IsAsyncStepEnabled();

	void KickAsyncStep(float DeltaTime);
	void JoinAsyncStep();

	bool IsStepInFlight() const { return StepTask.IsValid(); }

	/** Join the step task. Anything touching the box3d world must call this first. */
	void FlushAsyncStep() const;

protected:
	void CreateBox3DWorld();
	void DestroyBox3DWorld();
	void StepFixed(float DeltaTime);
	void DebugDraw();

	/** box3d per-phase timings (ms), summed over the frame's steps. */
	struct FBox3DFrameProfile
	{
		float Step = 0.0f;
		float Pairs = 0.0f;
		float Collide = 0.0f;
		float Solve = 0.0f;
		float Bullets = 0.0f;
		float Sensors = 0.0f;
		int32 StepCount = 0;

		void Accumulate(const b3Profile& P);
	};

	/** b3World_Step + profile read. No UObject access, so it can run on the task thread. */
	void StepWorldOnly(FBox3DFrameProfile& Frame);

	/** Game-thread half of one step: events, joints, characters, transform capture. Must run
	 *  once per step before the next overwrites box3d's event buffers. */
	void FinishStepGameThread();

	void ApplyRenderInterpolation();

	/** POD so the task never dereferences a UObject. */
	struct FKinematicTarget
	{
		b3BodyId Body = b3_nullBodyId;
		b3WorldTransform Target = {};
	};

	void GatherKinematicTargets(TArray<FKinematicTarget>& OutTargets);

	void ApplyKinematicTargets(const TArray<FKinematicTarget>& Targets, float TimeStep) const;

	/** box3d's own renderer (box3d.NativeDraw): contacts, joint frames, islands, sleep.
	 *  Authority only - a client has no world for box3d to walk. */
	void NativeDebugDraw();

	/** Push the frame's timings and box3d's counters into "stat box3d". */
	void PublishStats(const FBox3DFrameProfile& Frame) const;

	void DrainStepEvents();
	void DispatchPendingEvents();

	void EnableSimulation();
	void DisableSimulation();
	void OnEnabledCVarChanged();

	void OnLevelAddedToWorld(ULevel* Level, UWorld* World);
	void OnLevelRemovedFromWorld(ULevel* Level, UWorld* World);
	void RegisterLevelStaticGeometry(ULevel* Level);
	void UnregisterLevelStaticGeometry(ULevel* Level);

	void LoadBakedStaticGeometry();

	/** The configured assets plus, if bAutoDiscoverBakedCollision, this map's BC_ asset. Deduped,
	 *  so listing an auto-discovered asset explicitly is harmless. */
	TArray<UBox3DCollisionData*> GatherBakedCollisionAssets() const;
	UBox3DCollisionData* FindBakedAssetForCurrentMap() const;

	/** Editor/PIE only: warn when a bake no longer matches its source level or box3d version.
	 *  A packaged build can't re-bake, so the check runs where it can still be acted on. */
	void WarnIfBakeStale(const UBox3DCollisionData* Data) const;

private:
	b3WorldId WorldId = b3_nullWorldId;
	bool bWorldValid = false;
	bool bIsAuthority = false;

	/** Mutable so the const FlushAsyncStep can clear it. */
	mutable UE::Tasks::FTask StepTask;

	/** Written by the task, read after the join - no lock needed. */
	FBox3DFrameProfile AsyncFrame;
	int32 AsyncStepCount = 0;

	FBox3DKickTickFunction KickTick;
	FBox3DJoinTickFunction JoinTick;
	bool bTickFunctionsRegistered = false;

	void RegisterStepTickFunctions();
	void UnregisterStepTickFunctions();

	/** True while the simulation is built. Tracks the master switch so the runtime
	 *  sink only rebuilds/tears down on an actual on<->off transition. */
	bool bEnabledActive = false;

	/** Fires when any cvar changes; we re-check box3d.Enabled to build or tear down. */
	FConsoleVariableSinkHandle EnabledSinkHandle;

	/** Mirrors box3d.NativeDraw / .Range / .Thickness, read once per draw. */
	int32 NativeDrawFlags = 0;
	float NativeDrawRange = 5000.0f;
	float NativeDrawThickness = 1.0f;

	/** Real time carried between frames, consumed in fixed increments. */
	double Accumulator = 0.0;

	/** Fixed steps taken since world create (see GetSimulationFrame). */
	int64 SimulationFrame = 0;

	UPROPERTY(EditAnywhere, Category = "Box3D")
	float FixedTimeStep = 1.0f / 60.0f;

	UPROPERTY(EditAnywhere, Category = "Box3D")
	int32 SubStepCount = 4;

	/** box3d solver threads. Keep at 1 for a deterministic sim (server authority / rollback):
	 *  box3d partitions the constraint graph by worker count, so a different count changes the
	 *  result. Raise only for a single-machine sim that needs no cross-peer reproducibility. */
	UPROPERTY(EditAnywhere, Config, Category = "Box3D", meta = (ClampMin = "1"))
	int32 WorkerCount = 1;

	/** Spiral-of-death guard: never simulate more than this much time per frame. */
	UPROPERTY(EditAnywhere, Category = "Box3D")
	float MaxFrameTime = 0.25f;

	/** Gravity in Unreal space (cm/s^2). Converted to box3d meters on world create. */
	UPROPERTY(EditAnywhere, Category = "Box3D")
	FVector Gravity = FVector(0.0, 0.0, -980.0);

	/** How fast a collision has to be (cm/s) before it counts as a hit. Raise it to keep
	 *  small settling taps quiet. */
	UPROPERTY(EditAnywhere, Config, Category = "Box3D|Events", meta = (ClampMin = "0.0"))
	float HitEventThreshold = 100.0f;

	UPROPERTY(Transient)
	TArray<FBox3DTouchEvent> PendingBeginContact;

	UPROPERTY(Transient)
	TArray<FBox3DTouchEvent> PendingEndContact;

	UPROPERTY(Transient)
	TArray<FBox3DTouchEvent> PendingBeginOverlap;

	UPROPERTY(Transient)
	TArray<FBox3DTouchEvent> PendingEndOverlap;

	/** Up to two entries per collision, one for each side that asked for hits. */
	UPROPERTY(Transient)
	TArray<FBox3DHitEvent> PendingHits;

	/** The same collisions, one entry each, for OnAnyBox3DHit. */
	UPROPERTY(Transient)
	TArray<FBox3DHitEvent> PendingWorldHits;

	UPROPERTY(Transient)
	TArray<FBox3DSleepEvent> PendingSleep;

	UPROPERTY(Transient)
	TArray<FBox3DSleepEvent> PendingWake;

	/** Actors carrying this tag are bulk-registered as static box3d geometry on level
	 *  load/stream-in - no per-actor UBox3DBodyComponent needed. None (default) disables
	 *  the scan; the component-per-actor path stays primary. */
	UPROPERTY(EditAnywhere, Config, Category = "Box3D|Bulk Static")
	FName StaticGeometryTag = NAME_None;

	/** Material applied to bulk-registered static bodies (the tagged-actor path has no
	 *  component to read per-actor material from). */
	UPROPERTY(EditAnywhere, Config, Category = "Box3D|Bulk Static", meta = (ClampMin = "0.0"))
	float StaticGeometryFriction = 0.6f;

	UPROPERTY(EditAnywhere, Config, Category = "Box3D|Bulk Static", meta = (ClampMin = "0.0"))
	float StaticGeometryRestitution = 0.0f;

	/** Pre-baked static collision assets to instantiate on world begin. Produced by the
	 *  bake commandlet; unlike the tag-scan path these need no runtime cooking, so they are
	 *  the packaged-build path for static geometry. The material above
	 *  (StaticGeometryFriction/Restitution) is applied to their shapes. Loaded on top of
	 *  whatever bAutoDiscoverBakedCollision finds. */
	UPROPERTY(EditAnywhere, Config, Category = "Box3D|Bulk Static")
	TArray<TSoftObjectPtr<UBox3DCollisionData>> BakedCollisionAssets;

	/** Also load the current map's baked asset (BC_<MapName> beside the map) without it being
	 *  listed above. On by default: forgetting to list an asset silently loads a level with no
	 *  static collision, which looks like a physics bug rather than a config mistake. */
	UPROPERTY(EditAnywhere, Config, Category = "Box3D|Bulk Static")
	bool bAutoDiscoverBakedCollision = true;

	/** Dynamic bodies driven each frame. Weak so a destroyed actor drops out safely. */
	TArray<TWeakObjectPtr<UBox3DBodyComponent>> DynamicBodies;

	/** Kinematic bodies pushed from their actor transform before each step. */
	TArray<TWeakObjectPtr<UBox3DBodyComponent>> KinematicBodies;

	/** Every registered body (all types), used only for debug draw. */
	TArray<TWeakObjectPtr<UBox3DBodyComponent>> AllBodies;

	/** Kinematic characters, solved each fixed step. */
	TArray<TWeakObjectPtr<UBox3DCharacterComponent>> Characters;

	/** box3d resources for one level's bulk-registered static geometry. Not UPROPERTYs -
	 *  these are raw box3d handles this subsystem owns and destroys explicitly. */
	struct FBulkStaticLevel
	{
		TArray<b3BodyId> Bodies;
		TArray<b3MeshData*> Meshes; // tri-mesh data the shapes reference; freed after bodies
	};

	void CreateBulkStaticBody(AActor* Actor, FBulkStaticLevel& Bulk);

	/** A joint this subsystem owns. Slots are reused; Serial invalidates stale handles. */
	struct FJointSlot
	{
		b3JointId Id = b3_nullJointId;
		EBox3DJointType Type = EBox3DJointType::Spherical;
		float BreakForce = 0.0f;
		float BreakTorque = 0.0f;
		bool bBreakable = false;
		int32 Serial = 0;
	};

	TArray<FJointSlot> JointSlots;

	/** Free slot indices, so a long-running world doesn't grow the array forever. */
	TArray<int32> FreeJointSlots;

	/** Break-threshold check, run once per fixed step after the solve. Destroys the joint
	 *  and queues the notification for DispatchPendingEvents. */
	void UpdateBreakableJoints();

	/** One broken joint, waiting for dispatch after the step loop. */
	struct FPendingJointBreak
	{
		FBox3DJointHandle Handle;
		float Force = 0.0f;
		float Torque = 0.0f;
	};

	TArray<FPendingJointBreak> PendingJointBreaks;

	void DestroyAllJoints();

	TMap<TWeakObjectPtr<ULevel>, FBulkStaticLevel> BulkStaticLevels;

	/** Bodies instantiated from the baked collision assets (one bucket per loaded asset).
	 *  Owned like the bulk levels: freed in DestroyBox3DWorld. */
	TArray<FBulkStaticLevel> BakedStaticBuckets;
	FDelegateHandle LevelAddedHandle;
	FDelegateHandle LevelRemovedHandle;
};
