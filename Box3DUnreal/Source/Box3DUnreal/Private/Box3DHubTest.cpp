// Author: Antonio Lattanzio - emptyvessel

#include "Box3DConversion.h"
#include "Box3DJoints.h"
#include "Box3DLog.h"
#include <box3d/box3d.h>

/**
 * `box3d.HubTest` — can one body be held by several joints at once?
 *
 * A chain joint carries load along a single path. A *hub* takes several supports into one body,
 * and the solver has to satisfy all of them against the same mass. Nothing in Box3DJointTest
 * covers that: its chain is one-parent-per-link, which is the case that already works.
 *
 * The question is not academic. A multi-leg spring tower is a hub — three legs into one crown —
 * and `FTDTowerCourse::FootprintScale` exists precisely because the tower could not express one
 * and had to fake a tripod as a single wide, light body. Whether that fake can be replaced with
 * real legs that share load is decided here. See TowerDefenseCore/Docs/TowerJointGraph.md.
 *
 * Headless and self-checking, like its sibling: raw b3 calls, a world per case, no level content
 * and no actors. That last part is the point. An earlier version of this experiment lived in
 * TowerDefenseCore as a console command that spawned real actors, and six consecutive runs
 * measured nothing but its own bugs — missing owners, unconverted force units, static geometry
 * that never got a shape, joint anchors outside the body they held. Every one of those failure
 * modes is in the actor/component layer, and none of them is what was being asked about.
 *
 *   box3d.HubTest
 */
// A named namespace nested in an anonymous one, rather than a bare anonymous one: UE builds these
// files in a unity translation unit, so an anonymous `Step`/`Check`/`MakeBox` here collides with
// Box3DJointTest.cpp's identically named helpers. The outer anonymous namespace keeps everything
// internal to the TU as before; the inner name is what separates the two sets.
namespace
{
namespace Box3DHubTest
{
	constexpr float HubTestTimeStep = 1.0f / 60.0f;
	constexpr int32 HubTestSubSteps = 4;
	constexpr int32 HubSettleFrames = 240;

	struct FHubTally
	{
		int32 Passed = 0;
		int32 Failed = 0;
	};

	void Check(FHubTally& Tally, bool bCondition, const FString& What)
	{
		if (bCondition)
		{
			++Tally.Passed;
			UE_LOG(LogBox3D, Log, TEXT("  PASS  %s"), *What);
		}
		else
		{
			++Tally.Failed;
			UE_LOG(LogBox3D, Error, TEXT("  FAIL  %s"), *What);
		}
	}

	b3WorldId MakeHubWorld()
	{
		b3WorldDef Def = b3DefaultWorldDef();
		Def.gravity = b3Vec3{ 0.0f, 0.0f, -9.8f };
		Def.workerCount = 1; // the deterministic path, matching the subsystem
		return b3CreateWorld(&Def);
	}

	void Step(b3WorldId World, int32 Frames = HubSettleFrames)
	{
		for (int32 Frame = 0; Frame < Frames; ++Frame)
		{
			b3World_Step(World, HubTestTimeStep, HubTestSubSteps);
		}
	}

	/** A box at Position (metres). Density is kg/m^3 here, so masses read as real kilograms. */
	b3BodyId MakeBox(b3WorldId World, const b3Pos& Position, const b3Vec3& HalfExtent,
		float Density, bool bStatic)
	{
		b3BodyDef Def = b3DefaultBodyDef();
		Def.type = bStatic ? b3_staticBody : b3_dynamicBody;
		Def.position = Position;

		const b3BodyId Body = b3CreateBody(World, &Def);

		b3ShapeDef ShapeDef = b3DefaultShapeDef();
		ShapeDef.density = Density;

		const b3BoxHull Hull = b3MakeBoxHull(HalfExtent.x, HalfExtent.y, HalfExtent.z);
		b3CreateHullShape(Body, &ShapeDef, &Hull.base);
		return Body;
	}

	/**
	 * A hub held by `SupportCount` spherical joints to static ground, all at one height.
	 *
	 * The supports are anchor *points*, not bodies. A leg would be a second dynamic body and a
	 * second joint in series, which reintroduces the thing that made the actor-based version
	 * useless: a sag anywhere folds the whole rig and the hub joints never come under load. The
	 * question here is only whether N constraints can hold one mass; legs are a separate question
	 * and only worth asking once this one passes.
	 *
	 * Sizes and density are a real tower's: UTDTowerMaterial defaults to 320 kg/m^3 and
	 * FTDTowerCourse to a 380uu-tall, 320uu-radius tier — 3.8m by 3.2m in this world's metres.
	 */
	b3BodyId BuildHubScene(b3WorldId World, int32 SupportCount, float DensityScale,
		TArray<b3JointId>& OutJoints, float& OutMassKg)
	{
		constexpr float HubHalfWidth = 3.2f;    // metres, the course's 320uu radius
		constexpr float HubHalfHeight = 1.9f;   // one 380uu tier tall
		constexpr float TowerDensity = 320.f;   // UTDTowerMaterial's own default
		constexpr float HubZ = 10.f;

		// Inside the hub's own footprint. Anchors out beyond the body they hold pull on its
		// corners, which is a different and much harder problem than supporting it.
		constexpr float AnchorRadius = HubHalfWidth * 0.7f;

		const b3Vec3 HubHalf{ HubHalfWidth, HubHalfWidth, HubHalfHeight };
		const float Density = TowerDensity * DensityScale;

		const b3BodyId Ground = MakeBox(World, b3Pos{ 0.0, 0.0, HubZ },
			b3Vec3{ 0.5f, 0.5f, 0.5f }, TowerDensity, true);

		const b3BodyId Hub = MakeBox(World, b3Pos{ 0.0, 0.0, HubZ }, HubHalf, Density, false);

		OutMassKg = b3Body_GetMass(Hub);

		for (int32 Index = 0; Index < SupportCount; ++Index)
		{
			const float Angle = (2.f * PI * static_cast<float>(Index)) / static_cast<float>(SupportCount);
			const b3Vec3 Offset{ FMath::Cos(Angle) * AnchorRadius, FMath::Sin(Angle) * AnchorRadius, 0.f };

			// Local frames written directly rather than through BuildJointFrames, as the chain test
			// does: both bodies start at the same origin, so each frame is just the offset.
			b3SphericalJointDef Def = b3DefaultSphericalJointDef();
			Def.base.bodyIdA = Ground;
			Def.base.bodyIdB = Hub;
			Def.base.localFrameA = b3Transform{ Offset, b3Quat_identity };
			Def.base.localFrameB = b3Transform{ Offset, b3Quat_identity };
			Def.base.collideConnected = false;

			OutJoints.Add(b3CreateSphericalJoint(World, &Def));
		}

		return Hub;
	}

	/** How far the hub has moved from where it was built, in metres. */
	double HubDrop(b3BodyId Hub)
	{
		constexpr double HubZ = 10.0;
		return HubZ - b3Body_GetTransform(Hub).p.z;
	}

	/**
	 * The headline case: does a hub on N joints stay where it was put?
	 *
	 * A tolerance in centimetres rather than "did not explode". A joint is a soft constraint, so
	 * some settling is expected and correct; what matters is that it settles rather than creeping.
	 */
	void TestHubHolds(FHubTally& Tally, int32 SupportCount)
	{
		UE_LOG(LogBox3D, Log, TEXT(" -- hub on %d support(s) --"), SupportCount);

		const Box3D::FScopedLengthUnits Units(1.0f);
		const b3WorldId World = MakeHubWorld();

		TArray<b3JointId> Joints;
		float MassKg = 0.f;
		const b3BodyId Hub = BuildHubScene(World, SupportCount, 1.f, Joints, MassKg);

		Step(World);

		const b3Pos P = b3Body_GetTransform(Hub).p;
		const bool bFinite = FMath::IsFinite(P.x) && FMath::IsFinite(P.y) && FMath::IsFinite(P.z);
		Check(Tally, bFinite, TEXT("the hub stayed finite"));

		const double Drop = HubDrop(Hub);
		UE_LOG(LogBox3D, Log, TEXT("  hub mass %.0fkg, settled %.4fm below its anchor."), MassKg, Drop);

		Check(Tally, Drop < 0.10,
			FString::Printf(TEXT("the hub is held within 10cm (%.4fm)"), Drop));

		// Settled, not still moving. A creeping hub passes a one-shot position check and then
		// keeps going, which on a tower is a crown that sinks through its own legs over a minute.
		const double Before = HubDrop(Hub);
		Step(World, 120);
		const double Creep = HubDrop(Hub) - Before;

		Check(Tally, FMath::Abs(Creep) < 0.01,
			FString::Printf(TEXT("the hub has settled rather than creeping (%.5fm over 2s)"), Creep));

		b3DestroyWorld(World);
	}

	/**
	 * Cutting a support moves its share onto the others — the mechanic, not just the physics.
	 *
	 * This is what makes legs worth having at all: if the survivors do not take up the load, the
	 * legs were scenery and a player cutting one would see nothing happen.
	 */
	void TestHubLoadSharing(FHubTally& Tally)
	{
		UE_LOG(LogBox3D, Log, TEXT(" -- hub load sharing --"));

		const Box3D::FScopedLengthUnits Units(1.0f);
		const b3WorldId World = MakeHubWorld();

		TArray<b3JointId> Joints;
		float MassKg = 0.f;
		const b3BodyId Hub = BuildHubScene(World, 3, 1.f, Joints, MassKg);

		Step(World);

		auto TotalForce = [&Joints]()
		{
			double Sum = 0.0;
			for (const b3JointId Joint : Joints)
			{
				if (B3_IS_NON_NULL(Joint))
				{
					Sum += b3Length(b3Joint_GetConstraintForce(Joint));
				}
			}
			return Sum;
		};

		// Weight is the figure to expect, and printing it is what makes a wrong unit obvious rather
		// than something to be argued about later.
		const double Expected = MassKg * 9.8;
		const double Before = TotalForce();

		UE_LOG(LogBox3D, Log,
			TEXT("  3 supports carry %.0fN total; the hub weighs %.0fN."), Before, Expected);

		Check(Tally, Before > Expected * 0.5 && Before < Expected * 2.0,
			FString::Printf(TEXT("the supports carry roughly the hub's weight (%.0fN vs %.0fN)"),
				Before, Expected));

		// Cut one. The remaining two must pick up its share.
		const double FirstShare = b3Length(b3Joint_GetConstraintForce(Joints[0]));
		b3DestroyJoint(Joints[0], /*wakeAttached=*/true);
		Joints[0] = b3_nullJointId;

		Step(World, 120);

		const double After = TotalForce();

		UE_LOG(LogBox3D, Log,
			TEXT("  cut a support carrying %.0fN; the remaining two now carry %.0fN (were %.0fN)."),
			FirstShare, After, Before - FirstShare);

		Check(Tally, After > (Before - FirstShare),
			FString::Printf(TEXT("the survivors took up the cut support's load (%.0fN > %.0fN)"),
				After, Before - FirstShare));

		// **Two supports do not hold a hub up, and that is the finding.**
		//
		// Two spherical joints leave one free rotational axis -- the line between them -- and
		// nothing here opposes rotation about it: the cone limit constrains each joint's own
		// swing, not the shared axis, and gravity has a moment arm the moment the centre of mass
		// leaves that line. So the hub keeps turning. Measured: 0.50m in the first two seconds,
		// then another 0.61m in the next two, still accelerating.
		//
		// Two earlier versions of this check tried to assert otherwise -- first that the hub would
		// stay level, then that it would find a new rest -- and both were wishful. What is
		// *actually* true is that the joints keep holding while the body rotates, so the failure
		// is a topple rather than a detachment. That is the thing worth asserting.
		const double TwoSupportDrop = HubDrop(Hub);

		const b3Pos Rest = b3Body_GetTransform(Hub).p;
		Check(Tally, FMath::IsFinite(Rest.x) && FMath::IsFinite(Rest.y) && FMath::IsFinite(Rest.z),
			TEXT("the hub stayed finite on two supports"));

		Step(World, 120);
		const double FurtherDrop = HubDrop(Hub) - TwoSupportDrop;

		// The constraints are still doing their job -- the hub is hanging from them, not falling
		// freely. Free fall over two seconds is ~19m; anything of that order means the joints let
		// go rather than the body rotating within them.
		Check(Tally, FurtherDrop < 5.0,
			FString::Printf(TEXT("the hub topples on two supports rather than detaching ")
				TEXT("(tipped %.4fm, then %.4fm over 2s -- free fall would be ~19m)"),
				TwoSupportDrop, FurtherDrop));

		UE_LOG(LogBox3D, Warning,
			TEXT("  NOTE: two supports are a hinge, not a stand. The hub rotates about the line "
				 "between them and keeps going. A leg tower needs three legs standing, or a "
				 "rigid crown, or it falls over when it is down to two -- which is a game design "
				 "decision, not a bug."));

		b3DestroyWorld(World);
	}

	/**
	 * Where the hub stops being held, so the tower knows what it may not build.
	 *
	 * Reported rather than asserted: the answer is a design constraint, not a pass or a fail. If a
	 * crown four times a tier's density holds, nothing a design can author will trouble it.
	 */
	void TestHubMassCeiling(FHubTally& Tally)
	{
		UE_LOG(LogBox3D, Log, TEXT(" -- hub mass ceiling --"));

		for (const float Scale : { 1.f, 2.f, 4.f, 8.f })
		{
			const Box3D::FScopedLengthUnits Units(1.0f);
			const b3WorldId World = MakeHubWorld();

			TArray<b3JointId> Joints;
			float MassKg = 0.f;
			const b3BodyId Hub = BuildHubScene(World, 3, Scale, Joints, MassKg);

			Step(World);

			const double Drop = HubDrop(Hub);
			UE_LOG(LogBox3D, Log, TEXT("  density x%.0f (%.0fkg): settled %.4fm."), Scale, MassKg, Drop);

			// Only the realistic end is a hard requirement. The rest is the curve.
			if (FMath::IsNearlyEqual(Scale, 1.f))
			{
				Check(Tally, Drop < 0.10,
					FString::Printf(TEXT("a tower-density hub is held (%.4fm)"), Drop));
			}

			b3DestroyWorld(World);
		}
	}

	/**
	 * Does one *weld* hold a hub that two spherical joints cannot?
	 *
	 * The design escape hatch implied by the two-support finding. If a leg tower is down to two
	 * legs it topples, which may well be what you want — but a tower whose crown is welded to one
	 * leg has a rigid connection that resists rotation, so it stands on the last leg instead. This
	 * says which of those two the physics will give you, so the design can choose rather than
	 * discover.
	 */
	void TestHubSingleWeld(FHubTally& Tally)
	{
		UE_LOG(LogBox3D, Log, TEXT(" -- hub on a single weld --"));

		const Box3D::FScopedLengthUnits Units(1.0f);
		const b3WorldId World = MakeHubWorld();

		constexpr float HubHalfWidth = 3.2f;
		constexpr float HubHalfHeight = 1.9f;
		constexpr float TowerDensity = 320.f;
		constexpr float HubZ = 10.f;

		const b3BodyId Ground = MakeBox(World, b3Pos{ 0.0, 0.0, HubZ },
			b3Vec3{ 0.5f, 0.5f, 0.5f }, TowerDensity, true);

		const b3BodyId Hub = MakeBox(World, b3Pos{ 0.0, 0.0, HubZ },
			b3Vec3{ HubHalfWidth, HubHalfWidth, HubHalfHeight }, TowerDensity, false);

		// Off-centre, so gravity has a moment arm to work with. A weld under the centre of mass
		// would hold trivially and prove nothing about resisting rotation.
		const b3Vec3 Offset{ HubHalfWidth * 0.7f, 0.f, 0.f };

		b3WeldJointDef Def = b3DefaultWeldJointDef();
		Def.base.bodyIdA = Ground;
		Def.base.bodyIdB = Hub;
		Def.base.localFrameA = b3Transform{ Offset, b3Quat_identity };
		Def.base.localFrameB = b3Transform{ Offset, b3Quat_identity };
		Def.base.collideConnected = false;

		const b3JointId Weld = b3CreateWeldJoint(World, &Def);

		Step(World);
		const double Drop = HubDrop(Hub);

		Step(World, 120);
		const double Creep = HubDrop(Hub) - Drop;

		UE_LOG(LogBox3D, Log,
			TEXT("  one off-centre weld: settled %.4fm, then %.5fm over 2s (%.0fN)."),
			Drop, Creep, b3Length(b3Joint_GetConstraintForce(Weld)));

		Check(Tally, Drop < 0.10,
			FString::Printf(TEXT("a single weld holds the hub level (%.4fm)"), Drop));

		Check(Tally, FMath::Abs(Creep) < 0.01,
			FString::Printf(TEXT("the welded hub settles rather than creeping (%.5fm)"), Creep));

		b3DestroyWorld(World);
	}

	void RunHubTest()
	{
		UE_LOG(LogBox3D, Log, TEXT("box3d.HubTest: starting."));

		FHubTally Tally;

		// Two supports first: the smallest thing that is a hub rather than a chain.
		TestHubHolds(Tally, 2);
		TestHubHolds(Tally, 3);
		TestHubHolds(Tally, 6);
		TestHubLoadSharing(Tally);
		TestHubSingleWeld(Tally);
		TestHubMassCeiling(Tally);

		if (Tally.Failed == 0)
		{
			UE_LOG(LogBox3D, Log,
				TEXT("box3d.HubTest: PASS - %d checks. A joint graph is viable at tower masses."),
				Tally.Passed);
		}
		else
		{
			UE_LOG(LogBox3D, Error, TEXT("box3d.HubTest: FAIL - %d passed, %d failed."),
				Tally.Passed, Tally.Failed);
		}
	}

	FAutoConsoleCommand GBox3DHubTest(
		TEXT("box3d.HubTest"),
		TEXT("Self-checking multi-joint tests: whether one body can be held by several joints at "
			 "once, whether cutting one moves its load onto the rest, and at what mass it gives "
			 "way. Headless - needs no level content."),
		FConsoleCommandDelegate::CreateStatic(&RunHubTest));
} // namespace Box3DHubTest
} // namespace
