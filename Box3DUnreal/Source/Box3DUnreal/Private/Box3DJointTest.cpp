// Author: Antonio Lattanzio - emptyvessel


#include "Box3DConversion.h"
#include "Box3DJoints.h"
#include "Box3DLog.h"
#include <box3d/box3d.h>

namespace
{
	constexpr float JointTestTimeStep = 1.0f / 60.0f;
	constexpr int32 JointTestSubSteps = 4;

	constexpr int32 SettleFrames = 240;

	struct FJointTally
	{
		int32 Passed = 0;
		int32 Failed = 0;
	};

	void Check(FJointTally& Tally, bool bCondition, const FString& What)
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

	void CheckNear(FJointTally& Tally, double Value, double Expected, double Tolerance, const FString& What)
	{
		Check(Tally, FMath::Abs(Value - Expected) <= Tolerance,
			FString::Printf(TEXT("%s: %.4f ~= %.4f (+/- %.4f)"), *What, Value, Expected, Tolerance));
	}

	b3WorldId MakeJointWorld()
	{
		b3WorldDef Def = b3DefaultWorldDef();
		Def.gravity = b3Vec3{ 0.0f, 0.0f, -9.8f };
		Def.workerCount = 1; // the deterministic path, matching the subsystem
		return b3CreateWorld(&Def);
	}

	void Step(b3WorldId World, int32 Frames = SettleFrames)
	{
		for (int32 Frame = 0; Frame < Frames; ++Frame)
		{
			b3World_Step(World, JointTestTimeStep, JointTestSubSteps);
		}
	}

	// A body of the given half extents at Position (metres). Static bodies get no density.
	b3BodyId MakeBox(b3WorldId World, const b3Pos& Position, float HalfSize, bool bStatic)
	{
		b3BodyDef Def = b3DefaultBodyDef();
		Def.type = bStatic ? b3_staticBody : b3_dynamicBody;
		Def.position = Position;

		const b3BodyId Body = b3CreateBody(World, &Def);
		b3ShapeDef ShapeDef = b3DefaultShapeDef();
		ShapeDef.density = 500.0f;
		const b3BoxHull Hull = b3MakeBoxHull(HalfSize, HalfSize, HalfSize);
		b3CreateHullShape(Body, &ShapeDef, &Hull.base);
		return Body;
	}

	FVector ToUnrealPos(const b3Pos& P)
	{
		return FVector(P.x, P.y, P.z);
	}

	double DistanceBetween(b3BodyId A, b3BodyId B)
	{
		const b3Pos PA = b3Body_GetTransform(A).p;
		const b3Pos PB = b3Body_GetTransform(B).p;
		return FVector::Dist(ToUnrealPos(PA), ToUnrealPos(PB));
	}

	// --- Frames ---

	void TestJointFrames(FJointTally& Tally)
	{
		UE_LOG(LogBox3D, Log, TEXT(" -- joint frames --"));

		// No FScopedLengthUnits: the Unreal cm<->m factor is a separate global it would not move.
		const b3WorldId World = MakeJointWorld();

		// Off-origin and rotated so an identity-frame bug cannot pass by luck. Unreal cm.
		const FVector WorldAnchor(120.0, -80.0, 260.0);
		const FVector WorldAxis(0.0, 1.0, 0.0);

		b3BodyDef DefA = b3DefaultBodyDef();
		DefA.type = b3_staticBody;
		DefA.position = Box3D::ToBox3DPosition(FVector(100.0, -100.0, 200.0));
		DefA.rotation = Box3D::ToBox3DQuat(FRotator(0.0, 35.0, 0.0).Quaternion());
		const b3BodyId BodyA = b3CreateBody(World, &DefA);

		b3BodyDef DefB = b3DefaultBodyDef();
		DefB.type = b3_staticBody;
		DefB.position = Box3D::ToBox3DPosition(FVector(140.0, -60.0, 320.0));
		DefB.rotation = Box3D::ToBox3DQuat(FRotator(15.0, -20.0, 0.0).Quaternion());
		const b3BodyId BodyB = b3CreateBody(World, &DefB);

		b3Transform FrameA, FrameB;
		Box3D::BuildJointFrames(BodyA, BodyB, WorldAnchor, WorldAxis, FrameA, FrameB);

		auto FrameToWorld = [](b3BodyId Body, const b3Transform& Frame)
		{
			const b3WorldTransform Xf = b3Body_GetTransform(Body);
			const b3Vec3 Rotated = b3RotateVector(Xf.q, Frame.p);
			return Box3D::FromBox3DPosition(b3Pos{ Xf.p.x + Rotated.x, Xf.p.y + Rotated.y, Xf.p.z + Rotated.z });
		};

		const FVector RoundTripA = FrameToWorld(BodyA, FrameA);
		const FVector RoundTripB = FrameToWorld(BodyB, FrameB);

		CheckNear(Tally, FVector::Dist(RoundTripA, WorldAnchor), 0.0, 0.5,
			TEXT("frame A round-trips to the world anchor"));
		CheckNear(Tally, FVector::Dist(RoundTripB, WorldAnchor), 0.0, 0.5,
			TEXT("frame B round-trips to the world anchor"));

		auto FrameAxisToWorld = [](b3BodyId Body, const b3Transform& Frame)
		{
			const b3WorldTransform Xf = b3Body_GetTransform(Body);
			const b3Quat WorldQ = b3MulQuat(Xf.q, Frame.q);
			// Direction, not vector - FromBox3DVector would apply the cm<->m scale.
			return Box3D::FromBox3DDirection(b3RotateVector(WorldQ, b3Vec3{ 0.0f, 0.0f, 1.0f }));
		};

		CheckNear(Tally, FVector::DotProduct(FrameAxisToWorld(BodyA, FrameA), WorldAxis.GetSafeNormal()), 1.0, 0.01,
			TEXT("frame A Z lies along the requested axis"));
		CheckNear(Tally, FVector::DotProduct(FrameAxisToWorld(BodyB, FrameB), WorldAxis.GetSafeNormal()), 1.0, 0.01,
			TEXT("frame B Z lies along the requested axis"));

		// A zero axis must not produce a NaN frame - the library falls back to up.
		b3Transform DegenerateA, DegenerateB;
		Box3D::BuildJointFrames(BodyA, BodyB, WorldAnchor, FVector::ZeroVector, DegenerateA, DegenerateB);
		Check(Tally, FMath::IsFinite(DegenerateA.q.v.x) && FMath::IsFinite(DegenerateA.p.x),
			TEXT("zero axis falls back to a finite frame"));

		b3DestroyWorld(World);
	}

	// --- Spherical ---

	void TestSphericalJoint(FJointTally& Tally)
	{
		UE_LOG(LogBox3D, Log, TEXT(" -- spherical --"));

		const Box3D::FScopedLengthUnits Units(1.0f);
		const b3WorldId World = MakeJointWorld();

		const double ArmLength = 1.0;
		const b3Pos AnchorPos{ 0.0, 0.0, 5.0 };
		const b3Pos HangPos{ ArmLength, 0.0, 5.0 }; // out to the side, so gravity swings it

		const b3BodyId Anchor = MakeBox(World, AnchorPos, 0.1f, true);
		const b3BodyId Hanging = MakeBox(World, HangPos, 0.1f, false);

		b3SphericalJointDef Def = b3DefaultSphericalJointDef();
		Def.base.bodyIdA = Anchor;
		Def.base.bodyIdB = Hanging;
		Def.base.localFrameA = b3Transform{ b3Vec3{ 0.0f, 0.0f, 0.0f }, b3Quat_identity };
		Def.base.localFrameB = b3Transform{ b3Vec3{ -static_cast<float>(ArmLength), 0.0f, 0.0f }, b3Quat_identity };
		Def.base.collideConnected = false;
		const b3JointId Joint = b3CreateSphericalJoint(World, &Def);

		Step(World);

		// Distance to the anchor is the constraint, and it must hold to well under a mm.
		CheckNear(Tally, DistanceBetween(Anchor, Hanging), ArmLength, 0.005,
			TEXT("ball socket holds the arm length"));

		const b3Pos Settled = b3Body_GetTransform(Hanging).p;
		Check(Tally, Settled.z < AnchorPos.z - 0.5,
			FString::Printf(TEXT("unlimited ball socket swings down (z=%.3f, anchor z=%.3f)"),
				Settled.z, AnchorPos.z));

		b3DestroyJoint(Joint, true);
		Check(Tally, !b3Joint_IsValid(Joint), TEXT("the handle stops resolving once the joint is destroyed"));

		b3DestroyWorld(World);
	}

	void TestSphericalConeLimit(FJointTally& Tally)
	{
		UE_LOG(LogBox3D, Log, TEXT(" -- spherical cone limit --"));

		const Box3D::FScopedLengthUnits Units(1.0f);
		const b3WorldId World = MakeJointWorld();

		const double ArmLength = 1.0;
		const float ConeAngle = FMath::DegreesToRadians(30.0f);

		// Cone opens downward, so gravity parks the arm right on the limit.
		const b3BodyId Anchor = MakeBox(World, b3Pos{ 0.0, 0.0, 5.0 }, 0.1f, true);
		const b3BodyId Hanging = MakeBox(World, b3Pos{ ArmLength, 0.0, 5.0 }, 0.1f, false);

		// Built by hand so this case does not depend on the Unreal frame helper.
		const b3Quat DownFrame = b3MakeQuatFromAxisAngle(b3Vec3{ 0.0f, 1.0f, 0.0f }, PI);

		b3SphericalJointDef Def = b3DefaultSphericalJointDef();
		Def.base.bodyIdA = Anchor;
		Def.base.bodyIdB = Hanging;
		Def.base.localFrameA = b3Transform{ b3Vec3{ 0.0f, 0.0f, 0.0f }, DownFrame };
		Def.base.localFrameB = b3Transform{ b3Vec3{ -static_cast<float>(ArmLength), 0.0f, 0.0f }, DownFrame };
		Def.base.collideConnected = false;
		Def.enableConeLimit = true;
		Def.coneAngle = ConeAngle;
		const b3JointId Joint = b3CreateSphericalJoint(World, &Def);

		Check(Tally, b3SphericalJoint_IsConeLimitEnabled(Joint), TEXT("cone limit reads back enabled"));
		CheckNear(Tally, b3SphericalJoint_GetConeLimit(Joint), ConeAngle, 1e-4,
			TEXT("cone limit reads back the angle it was given"));

		Step(World);

		const b3Quat FrameA = b3MulQuat(b3Body_GetTransform(Anchor).q, DownFrame);
		const b3Quat FrameB = b3MulQuat(b3Body_GetTransform(Hanging).q, DownFrame);
		const double SettledAngle = b3GetSwingAngle(b3InvMulQuat(FrameA, FrameB));

		Check(Tally, SettledAngle <= ConeAngle + FMath::DegreesToRadians(3.0),
			FString::Printf(TEXT("cone limit holds the joint frame inside %.1f deg (settled %.1f deg)"),
				FMath::RadiansToDegrees(ConeAngle), FMath::RadiansToDegrees(SettledAngle)));

		Check(Tally, SettledAngle >= ConeAngle - FMath::DegreesToRadians(5.0),
			FString::Printf(TEXT("the joint frame rests on the cone limit (%.1f deg)"),
				FMath::RadiansToDegrees(SettledAngle)));

		b3DestroyJoint(Joint, true);
		b3DestroyWorld(World);
	}

	// --- Revolute ---

	void TestRevoluteLimit(FJointTally& Tally)
	{
		UE_LOG(LogBox3D, Log, TEXT(" -- revolute limit --"));

		const Box3D::FScopedLengthUnits Units(1.0f);
		const b3WorldId World = MakeJointWorld();

		const float LowerAngle = FMath::DegreesToRadians(-45.0f);
		const float UpperAngle = FMath::DegreesToRadians(45.0f);

		// Hinge about Y, arm along +X: gravity drives it onto the lower limit.
		const b3BodyId Anchor = MakeBox(World, b3Pos{ 0.0, 0.0, 5.0 }, 0.1f, true);
		const b3BodyId Arm = MakeBox(World, b3Pos{ 1.0, 0.0, 5.0 }, 0.1f, false);

		const b3Quat YFrame = b3MakeQuatFromAxisAngle(b3Vec3{ 1.0f, 0.0f, 0.0f }, -0.5f * PI);

		b3RevoluteJointDef Def = b3DefaultRevoluteJointDef();
		Def.base.bodyIdA = Anchor;
		Def.base.bodyIdB = Arm;
		Def.base.localFrameA = b3Transform{ b3Vec3{ 0.0f, 0.0f, 0.0f }, YFrame };
		Def.base.localFrameB = b3Transform{ b3Vec3{ -1.0f, 0.0f, 0.0f }, YFrame };
		Def.base.collideConnected = false;
		Def.enableLimit = true;
		Def.lowerAngle = LowerAngle;
		Def.upperAngle = UpperAngle;
		const b3JointId Joint = b3CreateRevoluteJoint(World, &Def);

		Check(Tally, b3RevoluteJoint_IsLimitEnabled(Joint), TEXT("revolute limit reads back enabled"));
		CheckNear(Tally, b3RevoluteJoint_GetLowerLimit(Joint), LowerAngle, 1e-4, TEXT("lower limit round-trips"));
		CheckNear(Tally, b3RevoluteJoint_GetUpperLimit(Joint), UpperAngle, 1e-4, TEXT("upper limit round-trips"));

		Step(World);

		const float Angle = b3RevoluteJoint_GetAngle(Joint);
		Check(Tally, Angle >= LowerAngle - FMath::DegreesToRadians(3.0f)
					&& Angle <= UpperAngle + FMath::DegreesToRadians(3.0f),
			FString::Printf(TEXT("hinge angle stays inside its limits (%.1f deg in [%.0f, %.0f])"),
				FMath::RadiansToDegrees(Angle), FMath::RadiansToDegrees(LowerAngle),
				FMath::RadiansToDegrees(UpperAngle)));

		Check(Tally, FMath::Abs(Angle) > FMath::DegreesToRadians(30.0f),
			FString::Printf(TEXT("gravity drives the arm onto a limit (%.1f deg)"),
				FMath::RadiansToDegrees(Angle)));

		CheckNear(Tally, DistanceBetween(Anchor, Arm), 1.0, 0.005, TEXT("hinge holds the arm radius"));

		b3DestroyJoint(Joint, true);
		b3DestroyWorld(World);
	}

	void TestRevoluteMotor(FJointTally& Tally)
	{
		UE_LOG(LogBox3D, Log, TEXT(" -- revolute motor --"));

		const Box3D::FScopedLengthUnits Units(1.0f);
		const b3WorldId World = MakeJointWorld();

		const float MotorSpeed = 2.0f; // rad/s

		// About Z so the arm sweeps horizontally and gravity does not fight the motor.
		const b3BodyId Anchor = MakeBox(World, b3Pos{ 0.0, 0.0, 5.0 }, 0.1f, true);
		const b3BodyId Arm = MakeBox(World, b3Pos{ 1.0, 0.0, 5.0 }, 0.1f, false);
		b3Body_SetAngularDamping(Arm, 2.0f);

		b3RevoluteJointDef Def = b3DefaultRevoluteJointDef();
		Def.base.bodyIdA = Anchor;
		Def.base.bodyIdB = Arm;
		Def.base.localFrameA = b3Transform{ b3Vec3{ 0.0f, 0.0f, 0.0f }, b3Quat_identity };
		Def.base.localFrameB = b3Transform{ b3Vec3{ -1.0f, 0.0f, 0.0f }, b3Quat_identity };
		Def.base.collideConnected = false;
		Def.enableMotor = true;
		Def.motorSpeed = MotorSpeed;
		Def.maxMotorTorque = 1000.0f;
		const b3JointId Joint = b3CreateRevoluteJoint(World, &Def);

		Check(Tally, b3RevoluteJoint_IsMotorEnabled(Joint), TEXT("motor reads back enabled"));
		CheckNear(Tally, b3RevoluteJoint_GetMotorSpeed(Joint), MotorSpeed, 1e-4, TEXT("motor speed round-trips"));

		Step(World, 30);
		const float StartAngle = b3RevoluteJoint_GetAngle(Joint);
		constexpr int32 MeasureFrames = 60;
		Step(World, MeasureFrames);
		const float EndAngle = b3RevoluteJoint_GetAngle(Joint);

		// GetAngle wraps, so rate comes from angular velocity; the angle only shows movement.
		const b3Vec3 Omega = b3Body_GetAngularVelocity(Arm);
		CheckNear(Tally, Omega.z, MotorSpeed, 0.2, TEXT("motor drives the arm at its target rate"));
		Check(Tally, !FMath::IsNearlyEqual(StartAngle, EndAngle, 1e-3f),
			TEXT("the hinge angle actually advances under the motor"));

		Check(Tally, FMath::Abs(b3RevoluteJoint_GetMotorTorque(Joint)) <= 1000.0f + 1.0f,
			TEXT("motor torque respects its maximum"));

		b3RevoluteJoint_EnableMotor(Joint, false);
		Step(World, 120);
		Check(Tally, FMath::Abs(b3Body_GetAngularVelocity(Arm).z) < MotorSpeed,
			TEXT("disabling the motor lets the arm slow down"));

		b3DestroyJoint(Joint, true);
		b3DestroyWorld(World);
	}

	// --- Prismatic ---

	void TestPrismaticJoint(FJointTally& Tally)
	{
		UE_LOG(LogBox3D, Log, TEXT(" -- prismatic --"));

		const Box3D::FScopedLengthUnits Units(1.0f);
		const b3WorldId World = MakeJointWorld();

		const float LowerTranslation = -0.5f;
		const float UpperTranslation = 0.5f;

		// Along Z, so gravity pushes it to the lower stop.
		const b3BodyId Anchor = MakeBox(World, b3Pos{ 0.0, 0.0, 5.0 }, 0.1f, true);
		const b3BodyId Slider = MakeBox(World, b3Pos{ 0.0, 0.0, 5.0 }, 0.1f, false);
		const b3Quat ZFrame = b3MakeQuatFromAxisAngle(b3Vec3{ 0.0f, 1.0f, 0.0f }, -0.5f * PI);

		b3PrismaticJointDef Def = b3DefaultPrismaticJointDef();
		Def.base.bodyIdA = Anchor;
		Def.base.bodyIdB = Slider;
		Def.base.localFrameA = b3Transform{ b3Vec3{ 0.0f, 0.0f, 0.0f }, ZFrame };
		Def.base.localFrameB = b3Transform{ b3Vec3{ 0.0f, 0.0f, 0.0f }, ZFrame };
		Def.base.collideConnected = false;
		Def.enableLimit = true;
		Def.lowerTranslation = LowerTranslation;
		Def.upperTranslation = UpperTranslation;
		const b3JointId Joint = b3CreatePrismaticJoint(World, &Def);

		Step(World);

		CheckNear(Tally, b3PrismaticJoint_GetTranslation(Joint), LowerTranslation, 0.02,
			TEXT("slider settles on its lower stop"));

		const b3Pos SliderPos = b3Body_GetTransform(Slider).p;
		CheckNear(Tally, SliderPos.x, 0.0, 0.005, TEXT("slider does not drift in X"));
		CheckNear(Tally, SliderPos.y, 0.0, 0.005, TEXT("slider does not drift in Y"));
		CheckNear(Tally, SliderPos.z, 5.0 + LowerTranslation, 0.02, TEXT("slider travelled along Z only"));

		b3PrismaticJoint_SetMaxMotorForce(Joint, 5000.0f);
		b3PrismaticJoint_SetMotorSpeed(Joint, 1.0f);
		b3PrismaticJoint_EnableMotor(Joint, true);
		b3Body_SetAwake(Slider, true);
		Step(World);

		CheckNear(Tally, b3PrismaticJoint_GetTranslation(Joint), UpperTranslation, 0.02,
			TEXT("motor drives the slider to its upper stop"));

		b3DestroyJoint(Joint, true);
		b3DestroyWorld(World);
	}

	// --- Weld ---

	void TestWeldJoint(FJointTally& Tally)
	{
		UE_LOG(LogBox3D, Log, TEXT(" -- weld --"));

		const Box3D::FScopedLengthUnits Units(1.0f);
		const b3WorldId World = MakeJointWorld();

		const b3Pos AnchorPos{ 0.0, 0.0, 5.0 };
		const b3Pos WeldedPos{ 1.0, 0.0, 5.0 };

		const b3BodyId Anchor = MakeBox(World, AnchorPos, 0.1f, true);
		const b3BodyId Welded = MakeBox(World, WeldedPos, 0.1f, false);

		const b3Quat StartRotation = b3Body_GetTransform(Welded).q;

		b3WeldJointDef Def = b3DefaultWeldJointDef();
		Def.base.bodyIdA = Anchor;
		Def.base.bodyIdB = Welded;
		Def.base.localFrameA = b3Transform{ b3Vec3{ 1.0f, 0.0f, 0.0f }, b3Quat_identity };
		Def.base.localFrameB = b3Transform{ b3Vec3{ 0.0f, 0.0f, 0.0f }, b3Quat_identity };
		Def.base.collideConnected = false;
		Def.linearHertz = 0.0f;  // 0 = rigid, the default weld
		Def.angularHertz = 0.0f;
		const b3JointId Joint = b3CreateWeldJoint(World, &Def);

		Step(World);

		const b3WorldTransform Settled = b3Body_GetTransform(Welded);
		CheckNear(Tally, FVector::Dist(ToUnrealPos(Settled.p), ToUnrealPos(WeldedPos)), 0.0, 0.01,
			TEXT("rigid weld holds position under gravity"));

		const float RotationDot = FMath::Abs(Settled.q.v.x * StartRotation.v.x + Settled.q.v.y * StartRotation.v.y
			+ Settled.q.v.z * StartRotation.v.z + Settled.q.s * StartRotation.s);
		CheckNear(Tally, RotationDot, 1.0, 0.01, TEXT("rigid weld holds rotation under gravity"));

		Check(Tally, b3Length(b3Joint_GetConstraintForce(Joint)) > 0.0f,
			TEXT("weld reports a non-zero constraint force while loaded"));

		b3DestroyJoint(Joint, true);
		b3DestroyWorld(World);
	}

	// --- Chain + determinism ---

	void BuildChainScene(b3WorldId World, TArray<b3BodyId>& OutBodies, TArray<b3JointId>& OutJoints)
	{
		constexpr int32 LinkCount = 8;
		constexpr double LinkSpacing = 0.5;

		b3BodyId Previous = MakeBox(World, b3Pos{ 0.0, 0.0, 10.0 }, 0.1f, true);

		for (int32 Index = 0; Index < LinkCount; ++Index)
		{
			const b3Pos Position{ 0.0, 0.0, 10.0 - LinkSpacing * (Index + 1) };
			const b3BodyId Link = MakeBox(World, Position, 0.1f, false);

			b3SphericalJointDef Def = b3DefaultSphericalJointDef();
			Def.base.bodyIdA = Previous;
			Def.base.bodyIdB = Link;
			Def.base.localFrameA = b3Transform{
				b3Vec3{ 0.0f, 0.0f, -static_cast<float>(LinkSpacing) * 0.5f }, b3Quat_identity };
			Def.base.localFrameB = b3Transform{ b3Vec3{ 0.0f, 0.0f, static_cast<float>(LinkSpacing) * 0.5f }, b3Quat_identity };
			Def.base.collideConnected = false;
			OutJoints.Add(b3CreateSphericalJoint(World, &Def));

			OutBodies.Add(Link);
			Previous = Link;
		}
	}

	void TestChain(FJointTally& Tally)
	{
		UE_LOG(LogBox3D, Log, TEXT(" -- chain --"));

		const Box3D::FScopedLengthUnits Units(1.0f);
		const b3WorldId World = MakeJointWorld();

		TArray<b3BodyId> Bodies;
		TArray<b3JointId> Joints;
		BuildChainScene(World, Bodies, Joints);

		// Nudge it so it swings; hanging straight down exercises little.
		b3Body_SetLinearVelocity(Bodies.Last(), b3Vec3{ 2.0f, 0.0f, 0.0f });
		Step(World, 600);

		const double MaxReach = 0.5 * Bodies.Num() + 0.05;
		double Worst = 0.0;
		bool bFinite = true;
		for (const b3BodyId Body : Bodies)
		{
			const b3Pos P = b3Body_GetTransform(Body).p;
			bFinite &= FMath::IsFinite(P.x) && FMath::IsFinite(P.y) && FMath::IsFinite(P.z);
			Worst = FMath::Max(Worst, FVector::Dist(ToUnrealPos(P), FVector(0.0, 0.0, 10.0)));
		}

		Check(Tally, bFinite, TEXT("no link went non-finite"));
		Check(Tally, Worst <= MaxReach,
			FString::Printf(TEXT("no link stretches past the chain's reach (%.3f <= %.3f m)"), Worst, MaxReach));

		const b3Pos Tail = b3Body_GetTransform(Bodies.Last()).p;
		Check(Tally, Tail.z < 10.0, TEXT("the chain hangs below its anchor"));

		b3DestroyWorld(World);
	}

	void TestJointDeterminism(FJointTally& Tally)
	{
		UE_LOG(LogBox3D, Log, TEXT(" -- joint determinism --"));

		const Box3D::FScopedLengthUnits Units(1.0f);

		// Joints solve in island order, so a solver-order dependency would show here.
		auto RunOnce = [](TArray<FVector>& OutFinal)
		{
			const b3WorldId World = MakeJointWorld();
			TArray<b3BodyId> Bodies;
			TArray<b3JointId> Joints;
			BuildChainScene(World, Bodies, Joints);
			b3Body_SetLinearVelocity(Bodies.Last(), b3Vec3{ 2.0f, 0.0f, 0.0f });
			Step(World, 300);

			for (const b3BodyId Body : Bodies)
			{
				OutFinal.Add(ToUnrealPos(b3Body_GetTransform(Body).p));
			}
			b3DestroyWorld(World);
		};

		TArray<FVector> First, Second;
		RunOnce(First);
		RunOnce(Second);

		bool bIdentical = First.Num() == Second.Num();
		for (int32 Index = 0; bIdentical && Index < First.Num(); ++Index)
		{
			bIdentical = First[Index].Equals(Second[Index], 0.0);
		}

		Check(Tally, bIdentical, TEXT("two identical jointed worlds end bit-exact"));
	}

	void RunJointTest()
	{
		UE_LOG(LogBox3D, Log, TEXT("box3d.JointTest: starting."));

		FJointTally Tally;
		TestJointFrames(Tally);
		TestSphericalJoint(Tally);
		TestSphericalConeLimit(Tally);
		TestRevoluteLimit(Tally);
		TestRevoluteMotor(Tally);
		TestPrismaticJoint(Tally);
		TestWeldJoint(Tally);
		TestChain(Tally);
		TestJointDeterminism(Tally);

		if (Tally.Failed == 0)
		{
			UE_LOG(LogBox3D, Log, TEXT("box3d.JointTest: PASS - %d checks."), Tally.Passed);
		}
		else
		{
			UE_LOG(LogBox3D, Error, TEXT("box3d.JointTest: FAIL - %d passed, %d failed."),
				Tally.Passed, Tally.Failed);
		}
	}

	FAutoConsoleCommand GBox3DJointTest(
		TEXT("box3d.JointTest"),
		TEXT("Self-checking joint tests: frames, spherical/revolute/prismatic/weld constraints, "
			 "limits, motors, a chain, and joint determinism. Headless - needs no level content."),
		FConsoleCommandDelegate::CreateStatic(&RunJointTest));
} // namespace
