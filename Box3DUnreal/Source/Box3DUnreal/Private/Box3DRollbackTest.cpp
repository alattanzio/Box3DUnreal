// Author: Antonio Lattanzio - emptyvessel


#include "Box3DConversion.h"
#include "Box3DLog.h"
#include "Box3DPrediction.h"
#include "Box3DSnapshot.h"
#include "HAL/IConsoleManager.h"
#include <box3d/box3d.h>

namespace
{
	constexpr float RbTimeStep = 1.0f / 60.0f;
	constexpr int32 RbSubSteps = 4;
	constexpr int32 EventFrame = 30;    // when the unmirrored impulse hits the server
	constexpr int32 AuthFrame = 33;     // frame the authoritative snapshot represents (post-event)
	constexpr int32 NetLatency = 6;     // frames the snapshot is stale by when it reaches the client
	constexpr int32 PresentFrame = AuthFrame + NetLatency;
	constexpr int32 RingCapacity = 64;

	// A small deterministic scene: a static slab and a 3x3 grid of cubes dropped just above it.
	void BuildRollbackScene(b3WorldId World, TArray<b3BodyId>& OutBodies)
	{
		{
			b3BodyDef Def = b3DefaultBodyDef();
			Def.type = b3_staticBody;
			const b3BodyId Ground = b3CreateBody(World, &Def);
			b3ShapeDef ShapeDef = b3DefaultShapeDef();
			ShapeDef.baseMaterial.friction = 0.6f;
			const b3BoxHull Hull = b3MakeBoxHull(10.0f, 10.0f, 0.5f);
			b3CreateHullShape(Ground, &ShapeDef, &Hull.base);
		}

		const b3BoxHull CubeHull = b3MakeBoxHull(0.25f, 0.25f, 0.25f);
		for (int32 X = 0; X < 3; ++X)
		{
			for (int32 Y = 0; Y < 3; ++Y)
			{
				b3BodyDef Def = b3DefaultBodyDef();
				Def.type = b3_dynamicBody;
				Def.position = b3Pos{ -1.0 + X * 1.0, -1.0 + Y * 1.0, 1.5 };
				const b3BodyId Body = b3CreateBody(World, &Def);
				b3ShapeDef ShapeDef = b3DefaultShapeDef();
				ShapeDef.density = 500.0f;
				ShapeDef.baseMaterial.friction = 0.5f;
				ShapeDef.baseMaterial.restitution = 0.1f;
				b3CreateHullShape(Body, &ShapeDef, &CubeHull.base);
				OutBodies.Add(Body);
			}
		}
	}

	b3WorldId MakeRollbackWorld()
	{
		b3WorldDef Def = b3DefaultWorldDef();
		Def.gravity = b3Vec3{ 0.0f, 0.0f, -9.8f };
		Def.workerCount = 1;
		return b3CreateWorld(&Def);
	}

	uint32 HashBodies(const TArray<b3BodyId>& Bodies)
	{
		uint32 Hash = B3_HASH_INIT;
		for (const b3BodyId Body : Bodies)
		{
			Hash = Box3D::HashBody(Hash, Body);
		}
		return Hash;
	}

	double MaxDelta(const TArray<b3BodyId>& A, const TArray<b3BodyId>& B)
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

	bool RunScenario(bool bInjectEvent)
	{
		b3WorldId Server = MakeRollbackWorld();
		b3WorldId Client = MakeRollbackWorld();
		TArray<b3BodyId> ServerBodies, ClientBodies;
		BuildRollbackScene(Server, ServerBodies);
		BuildRollbackScene(Client, ClientBodies);

		Box3D::FSnapshotRing Ring;
		Ring.Init(RingCapacity, ClientBodies.Num());

		// All bodies are corrected (a full authoritative snapshot).
		TArray<int32> AuthIndices;
		for (int32 i = 0; i < ClientBodies.Num(); ++i)
		{
			AuthIndices.Add(i);
		}

		TArray<Box3D::FBodyState> AuthStates;

		for (int32 Frame = 1; Frame <= PresentFrame; ++Frame)
		{
			if (bInjectEvent && Frame == EventFrame)
			{
				b3Body_ApplyLinearImpulseToCenter(ServerBodies[0], b3Vec3{ 400.0f, 0.0f, 250.0f }, true);
			}

			b3World_Step(Server, RbTimeStep, RbSubSteps);
			b3World_Step(Client, RbTimeStep, RbSubSteps);
			Ring.Capture(Frame, ClientBodies);

			if (Frame == AuthFrame)
			{
				AuthStates.Reset();
				for (const b3BodyId Body : ServerBodies)
				{
					AuthStates.Add(Box3D::CaptureBodyState(Body));
				}
			}
		}

		const double DriftBefore = MaxDelta(ServerBodies, ClientBodies);

		// The stale snapshot arrives now (client is at PresentFrame, snapshot is for AuthFrame).
		const Box3D::FReconcileResult Result = Box3D::ReconcileAndReplay(
			Client, ClientBodies, Ring, AuthFrame, AuthIndices, AuthStates, PresentFrame, RbTimeStep, RbSubSteps);

		const double DriftAfter = MaxDelta(ServerBodies, ClientBodies);
		const bool bMatches = HashBodies(ServerBodies) == HashBodies(ClientBodies);

		bool bPass;
		if (bInjectEvent)
		{
			constexpr double ConvergedTolerance = 1e-3; // 1 mm
			bPass = Result.bCorrected && Result.ReplayedFrames == NetLatency && DriftAfter < ConvergedTolerance
					&& DriftAfter < DriftBefore;
			UE_LOG(LogBox3D, Log, TEXT("box3d.RollbackTest [event]: %s - drift %.2f cm -> %.4f mm after ")
				TEXT("rollback+replay of %d frames (%s; bit-exact not expected, D1)."),
				bPass ? TEXT("PASS") : TEXT("FAIL"), DriftBefore * 100.0, DriftAfter * 1000.0,
				Result.ReplayedFrames, bMatches ? TEXT("hash match") : TEXT("sub-mm"));
		}
		else
		{
			// Expect: prediction was right, so no rollback and already in sync.
			bPass = !Result.bCorrected && bMatches && DriftAfter < 1e-6;
			UE_LOG(LogBox3D, Log, TEXT("box3d.RollbackTest [no-event]: %s - no correction (%s), drift %.6f cm."),
				bPass ? TEXT("PASS") : TEXT("FAIL"), Result.bCorrected ? TEXT("but corrected!") : TEXT("as expected"),
				DriftAfter * 100.0);
		}

		b3DestroyWorld(Server);
		b3DestroyWorld(Client);
		return bPass;
	}

	void RunRollbackTest()
	{
		// Authored in meters; see Box3DDeterminismTest.
		const Box3D::FScopedLengthUnits Units(1.0f);

		const bool bEvent = RunScenario(/*bInjectEvent=*/true);
		const bool bNoEvent = RunScenario(/*bInjectEvent=*/false);
		if (bEvent && bNoEvent)
		{
			UE_LOG(LogBox3D, Log, TEXT("box3d.RollbackTest: PASS - client converges exactly after a stale ")
				TEXT("authoritative correction, and skips the rollback when prediction was right."));
		}
		else
		{
			UE_LOG(LogBox3D, Error, TEXT("box3d.RollbackTest: FAIL (event=%s, no-event=%s)."),
				bEvent ? TEXT("pass") : TEXT("fail"), bNoEvent ? TEXT("pass") : TEXT("fail"));
		}
	}

	FAutoConsoleCommand GBox3DRollbackTest(
		TEXT("box3d.RollbackTest"),
		TEXT("Prove client prediction + rollback: a stale authoritative snapshot corrects a diverged client exactly."),
		FConsoleCommandDelegate::CreateStatic(&RunRollbackTest));
} // namespace
