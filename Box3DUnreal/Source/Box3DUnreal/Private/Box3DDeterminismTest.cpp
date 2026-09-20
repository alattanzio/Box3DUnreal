// Author: Antonio Lattanzio - emptyvessel


#include "Box3DConversion.h"
#include "Box3DLog.h"
#include "Box3DSnapshot.h"
#include "HAL/IConsoleManager.h"
#include <box3d/box3d.h>

namespace
{
	constexpr float TestTimeStep = 1.0f / 60.0f;
	constexpr int32 TestSubSteps = 4;
	constexpr int32 TestFrames = 300;      // 5 s at 60 Hz - long enough to stack, collide, settle
	constexpr int32 SnapshotFrame = 150;   // mid-run: bodies in flight and in contact
	constexpr int32 RollbackDepth = 6;     // a realistic reconciliation window (~100 ms of latency)

	void BuildScene(b3WorldId World, TArray<b3BodyId>& OutDynamicBodies)
	{
		// Ground: a large static box centred below the drop.
		{
			b3BodyDef Def = b3DefaultBodyDef();
			Def.type = b3_staticBody;
			Def.position = b3Pos{ 0.0, 0.0, 0.0 };
			const b3BodyId Ground = b3CreateBody(World, &Def);
			b3ShapeDef ShapeDef = b3DefaultShapeDef();
			ShapeDef.baseMaterial.friction = 0.6f;
			const b3BoxHull Hull = b3MakeBoxHull(10.0f, 10.0f, 0.5f); // 20x20x1 m slab
			b3CreateHullShape(Ground, &ShapeDef, &Hull.base);
		}

		const b3BoxHull CubeHull = b3MakeBoxHull(0.25f, 0.25f, 0.25f);
		for (int32 X = 0; X < 4; ++X)
		{
			for (int32 Y = 0; Y < 4; ++Y)
			{
				for (int32 Z = 0; Z < 4; ++Z)
				{
					b3BodyDef Def = b3DefaultBodyDef();
					Def.type = b3_dynamicBody;
					Def.position = b3Pos{ -1.8 + X * 1.2, -1.8 + Y * 1.2, 3.0 + Z * 1.2 };

					b3BodyId Body = b3CreateBody(World, &Def);
					b3ShapeDef ShapeDef = b3DefaultShapeDef();
					ShapeDef.density = 500.0f;
					ShapeDef.baseMaterial.friction = 0.5f;
					ShapeDef.baseMaterial.restitution = 0.1f;
					b3CreateHullShape(Body, &ShapeDef, &CubeHull.base);

					// Fixed per-cube spin (rad/s), derived from the lattice index - no RNG.
					b3Body_SetAngularVelocity(Body, b3Vec3{
						static_cast<float>(X - 2), static_cast<float>(Y - 2), static_cast<float>(Z - 2) });

					OutDynamicBodies.Add(Body);
				}
			}
		}
	}

	uint32 HashWorld(const TArray<b3BodyId>& Bodies)
	{
		uint32 Hash = B3_HASH_INIT;
		for (const b3BodyId Body : Bodies)
		{
			Hash = Box3D::HashBody(Hash, Body);
		}
		return Hash;
	}

	b3WorldId MakeWorld()
	{
		b3WorldDef Def = b3DefaultWorldDef();
		Def.gravity = b3Vec3{ 0.0f, 0.0f, -9.8f };
		Def.workerCount = 1; // the deterministic path (matches the subsystem)
		return b3CreateWorld(&Def);
	}

	void RunDeterminismTest()
	{
		// Authored in meters; the length unit is global state a live world may have set.
		const Box3D::FScopedLengthUnits Units(1.0f);

		b3WorldId WorldA = MakeWorld();
		b3WorldId WorldB = MakeWorld();
		TArray<b3BodyId> BodiesA, BodiesB;
		BuildScene(WorldA, BodiesA);
		BuildScene(WorldB, BodiesB);

		int32 FirstDivergence = -1;
		for (int32 Frame = 0; Frame < TestFrames; ++Frame)
		{
			b3World_Step(WorldA, TestTimeStep, TestSubSteps);
			b3World_Step(WorldB, TestTimeStep, TestSubSteps);
			if (FirstDivergence < 0 && HashWorld(BodiesA) != HashWorld(BodiesB))
			{
				FirstDivergence = Frame;
				break;
			}
		}

		if (FirstDivergence < 0)
		{
			UE_LOG(LogBox3D, Log, TEXT("box3d.DeterminismTest: PASS - %d bodies identical across %d frames on two worlds."),
				BodiesA.Num(), TestFrames);
		}
		else
		{
			UE_LOG(LogBox3D, Error, TEXT("box3d.DeterminismTest: FAIL - two worlds diverged at frame %d. ")
				TEXT("Check worker count, fast-math, or uninitialised inputs."), FirstDivergence);
		}

		b3DestroyWorld(WorldA);
		b3DestroyWorld(WorldB);
	}

	// Largest per-body position difference (metres) between two body sets at their current pose.
	double MaxPositionDrift(const TArray<b3BodyId>& A, const TArray<b3BodyId>& B)
	{
		double Max = 0.0;
		for (int32 i = 0; i < A.Num(); ++i)
		{
			const b3Pos P = b3Body_GetTransform(A[i]).p;
			const b3Pos Q = b3Body_GetTransform(B[i]).p;
			Max = FMath::Max(Max, FMath::Sqrt(
				(P.x - Q.x) * (P.x - Q.x) + (P.y - Q.y) * (P.y - Q.y) + (P.z - Q.z) * (P.z - Q.z)));
		}
		return Max;
	}

	void RunSnapshotTestOnce(bool bWarmStarting)
	{
		const int32 TailFrames = TestFrames - SnapshotFrame;

		// Reference: one clean run start to finish, snapshotting bodies at the mid-point.
		b3WorldId Ref = MakeWorld();
		b3World_EnableWarmStarting(Ref, bWarmStarting);
		TArray<b3BodyId> RefBodies;
		BuildScene(Ref, RefBodies);
		for (int32 Frame = 0; Frame < SnapshotFrame; ++Frame)
		{
			b3World_Step(Ref, TestTimeStep, TestSubSteps);
		}
		TArray<Box3D::FBodyState> Snapshot;
		Snapshot.Reserve(RefBodies.Num());
		for (const b3BodyId Body : RefBodies)
		{
			Snapshot.Add(Box3D::CaptureBodyState(Body));
		}

		b3WorldId Redo = MakeWorld();
		b3World_EnableWarmStarting(Redo, bWarmStarting);
		TArray<b3BodyId> RedoBodies;
		BuildScene(Redo, RedoBodies);
		for (int32 i = 0; i < RedoBodies.Num(); ++i)
		{
			Box3D::RestoreBodyState(RedoBodies[i], Snapshot[i]);
		}

		int32 FirstMismatch = -1;
		double ShortDrift = 0.0;
		for (int32 Frame = 0; Frame < TailFrames; ++Frame)
		{
			b3World_Step(Ref, TestTimeStep, TestSubSteps);
			b3World_Step(Redo, TestTimeStep, TestSubSteps);
			if (FirstMismatch < 0 && HashWorld(RefBodies) != HashWorld(RedoBodies))
			{
				FirstMismatch = Frame;
			}
			if (Frame == RollbackDepth - 1)
			{
				ShortDrift = MaxPositionDrift(RefBodies, RedoBodies);
			}
		}
		const double LongDrift = MaxPositionDrift(RefBodies, RedoBodies);

		if (FirstMismatch < 0)
		{
			UE_LOG(LogBox3D, Log, TEXT("box3d.SnapshotTest [warmStart=%s]: BIT-EXACT restore over %d frames."),
				bWarmStarting ? TEXT("on") : TEXT("off"), TailFrames);
		}
		else
		{
			UE_LOG(LogBox3D, Warning, TEXT("box3d.SnapshotTest [warmStart=%s]: hash diverges at tail frame %d; ")
				TEXT("drift @%d-frame rollback = %.2f cm, @%d-frame tail = %.2f cm. Kinematic-only snapshot loses ")
				TEXT("contact/warm-start history (expected) - the %d-frame number is the realistic rollback budget."),
				bWarmStarting ? TEXT("on") : TEXT("off"), FirstMismatch,
				RollbackDepth, ShortDrift * 100.0, TailFrames, LongDrift * 100.0, RollbackDepth);
		}

		b3DestroyWorld(Ref);
		b3DestroyWorld(Redo);
	}

	void RunSnapshotTest()
	{
		const Box3D::FScopedLengthUnits Units(1.0f); // see RunDeterminismTest

		UE_LOG(LogBox3D, Log, TEXT("box3d.SnapshotTest: snapshot at frame %d, compare %d-frame tail (restore vs. reference)."),
			SnapshotFrame, TestFrames - SnapshotFrame);
		RunSnapshotTestOnce(/*bWarmStarting=*/true);
		RunSnapshotTestOnce(/*bWarmStarting=*/false);
	}

	FAutoConsoleCommand GBox3DDeterminismTest(
		TEXT("box3d.DeterminismTest"),
		TEXT("Step two independent box3d worlds in lockstep and assert identical state hashes every frame."),
		FConsoleCommandDelegate::CreateStatic(&RunDeterminismTest));

	FAutoConsoleCommand GBox3DSnapshotTest(
		TEXT("box3d.SnapshotTest"),
		TEXT("Measure snapshot/restore fidelity: how far a restored+resimulated world drifts from the reference."),
		FConsoleCommandDelegate::CreateStatic(&RunSnapshotTest));
} // namespace
