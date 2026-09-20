// Author: Antonio Lattanzio - emptyvessel

#include "Box3DJoints.h"
#include "Box3DBodyComponent.h"
#include "Box3DConversion.h"
#include "Box3DLog.h"
#include "Box3DSubsystem.h"
#include "Engine/World.h"

namespace
{
	// box3d caps revolute/twist limits at 0.99*pi; clamp so a 180-degree entry is legal.
	constexpr float MaxLimitRadians = 0.99f * PI;

	FORCEINLINE float ClampLimit(float Degrees)
	{
		return FMath::Clamp(FMath::DegreesToRadians(Degrees), -MaxLimitRadians, MaxLimitRadians);
	}

	UBox3DSubsystem* GetSubsystem(const UObject* WorldContext)
	{
		const UWorld* World = GEngine != nullptr && WorldContext != nullptr
			? GEngine->GetWorldFromContextObject(WorldContext, EGetWorldErrorMode::ReturnNull)
			: nullptr;
		return World != nullptr ? World->GetSubsystem<UBox3DSubsystem>() : nullptr;
	}

	// Resolve the two bodies and their shared subsystem, or fail with one warning.
	bool ResolveBodies(UBox3DBodyComponent* A, UBox3DBodyComponent* B, const TCHAR* JointName,
		UBox3DSubsystem*& OutSubsystem, b3BodyId& OutA, b3BodyId& OutB)
	{
		if (A == nullptr || B == nullptr)
		{
			UE_LOG(LogBox3D, Warning, TEXT("%s joint needs two body components."), JointName);
			return false;
		}
		if (A == B)
		{
			UE_LOG(LogBox3D, Warning, TEXT("%s joint cannot connect a body to itself."), JointName);
			return false;
		}

		OutSubsystem = GetSubsystem(A);
		if (OutSubsystem == nullptr || !OutSubsystem->IsWorldValid())
		{
			return false; // no box3d world: client, or box3d.Enabled off. Not an error.
		}

		OutSubsystem->FlushAsyncStep();

		OutA = A->GetBodyId();
		OutB = B->GetBodyId();
		if (B3_IS_NULL(OutA) || B3_IS_NULL(OutB))
		{
			UE_LOG(LogBox3D, Warning,
				TEXT("%s joint skipped: a body has no box3d body yet. Create joints after both BeginPlay."),
				JointName);
			return false;
		}
		return true;
	}

	// Fill the parts of b3JointDef that every type shares.
	void ApplyBaseSettings(b3JointDef& Base, b3BodyId A, b3BodyId B,
		const b3Transform& FrameA, const b3Transform& FrameB, const FBox3DJointSettings& Settings)
	{
		Base.bodyIdA = A;
		Base.bodyIdB = B;
		Base.localFrameA = FrameA;
		Base.localFrameB = FrameB;
		Base.collideConnected = Settings.bCollideConnected;
		Base.constraintHertz = Settings.ConstraintHertz;
		Base.constraintDampingRatio = Settings.ConstraintDampingRatio;

		// box3d reports a joint event once past these; 0 in our settings means "never".
		Base.forceThreshold = Settings.BreakForce > 0.0f ? Settings.BreakForce : FLT_MAX;
		Base.torqueThreshold = Settings.BreakTorque > 0.0f ? Settings.BreakTorque : FLT_MAX;
	}
}

void Box3D::BuildJointFrames(b3BodyId BodyA, b3BodyId BodyB, const FVector& WorldAnchor,
	const FVector& WorldAxis, b3Transform& OutFrameA, b3Transform& OutFrameB)
{
	// Z along the axis: box3d defines every limit about the frame Z.
	FVector Axis = WorldAxis.GetSafeNormal();
	if (Axis.IsNearlyZero())
	{
		Axis = FVector::UpVector;
	}
	const FQuat WorldRotation = FRotationMatrix::MakeFromZ(Axis).ToQuat();

	const b3Quat AnchorQuat = ToBox3DQuat(WorldRotation);
	const b3Pos AnchorPos = ToBox3DPosition(WorldAnchor);

	auto ToLocal = [&AnchorPos, &AnchorQuat](b3BodyId Body)
	{
		const b3WorldTransform Xf = b3Body_GetTransform(Body);
		const b3Pos Delta{ AnchorPos.x - Xf.p.x, AnchorPos.y - Xf.p.y, AnchorPos.z - Xf.p.z };

		b3Transform Out;
		Out.p = b3InvRotateVector(Xf.q,
			b3Vec3{ static_cast<float>(Delta.x), static_cast<float>(Delta.y), static_cast<float>(Delta.z) });
		Out.q = b3InvMulQuat(Xf.q, AnchorQuat);
		return Out;
	};

	OutFrameA = ToLocal(BodyA);
	OutFrameB = ToLocal(BodyB);
}

FBox3DJointHandle UBox3DJointLibrary::CreateSphericalJoint(UBox3DBodyComponent* BodyA,
	UBox3DBodyComponent* BodyB, const FVector& WorldAnchor, const FVector& TwistAxis,
	const FBox3DJointSettings& Settings, const FBox3DSphericalJointSettings& Spherical)
{
	UBox3DSubsystem* Subsystem = nullptr;
	b3BodyId A, B;
	if (!ResolveBodies(BodyA, BodyB, TEXT("Spherical"), Subsystem, A, B))
	{
		return FBox3DJointHandle();
	}

	b3Transform FrameA, FrameB;
	Box3D::BuildJointFrames(A, B, WorldAnchor, TwistAxis, FrameA, FrameB);

	b3SphericalJointDef Def = b3DefaultSphericalJointDef();
	ApplyBaseSettings(Def.base, A, B, FrameA, FrameB, Settings);
	Def.enableConeLimit = Spherical.bEnableConeLimit;
	Def.coneAngle = FMath::Clamp(FMath::DegreesToRadians(Spherical.ConeAngle), 0.0f, PI);
	Def.enableTwistLimit = Spherical.bEnableTwistLimit;
	Def.lowerTwistAngle = ClampLimit(Spherical.LowerTwistAngle);
	Def.upperTwistAngle = ClampLimit(Spherical.UpperTwistAngle);
	Def.enableSpring = Spherical.bEnableSpring;
	Def.hertz = Spherical.Hertz;
	Def.dampingRatio = Spherical.DampingRatio;

	return Subsystem->RegisterJoint(b3CreateSphericalJoint(Subsystem->GetWorldId(), &Def),
		EBox3DJointType::Spherical, Settings);
}

FBox3DJointHandle UBox3DJointLibrary::CreateRevoluteJoint(UBox3DBodyComponent* BodyA,
	UBox3DBodyComponent* BodyB, const FVector& WorldAnchor, const FVector& WorldAxis,
	const FBox3DJointSettings& Settings, const FBox3DRevoluteJointSettings& Revolute)
{
	UBox3DSubsystem* Subsystem = nullptr;
	b3BodyId A, B;
	if (!ResolveBodies(BodyA, BodyB, TEXT("Revolute"), Subsystem, A, B))
	{
		return FBox3DJointHandle();
	}

	b3Transform FrameA, FrameB;
	Box3D::BuildJointFrames(A, B, WorldAnchor, WorldAxis, FrameA, FrameB);

	b3RevoluteJointDef Def = b3DefaultRevoluteJointDef();
	ApplyBaseSettings(Def.base, A, B, FrameA, FrameB, Settings);
	Def.enableLimit = Revolute.bEnableLimit;
	Def.lowerAngle = ClampLimit(Revolute.LowerAngle);
	Def.upperAngle = ClampLimit(Revolute.UpperAngle);
	Def.enableMotor = Revolute.bEnableMotor;

	// The frame Z already points where the caller asked, so only the units change.
	Def.motorSpeed = FMath::DegreesToRadians(Revolute.MotorSpeed);
	Def.maxMotorTorque = Revolute.MaxMotorTorque;
	Def.enableSpring = Revolute.bEnableSpring;
	Def.hertz = Revolute.Hertz;
	Def.dampingRatio = Revolute.DampingRatio;

	return Subsystem->RegisterJoint(b3CreateRevoluteJoint(Subsystem->GetWorldId(), &Def),
		EBox3DJointType::Revolute, Settings);
}

FBox3DJointHandle UBox3DJointLibrary::CreatePrismaticJoint(UBox3DBodyComponent* BodyA,
	UBox3DBodyComponent* BodyB, const FVector& WorldAnchor, const FVector& WorldAxis,
	const FBox3DJointSettings& Settings, const FBox3DPrismaticJointSettings& Prismatic)
{
	UBox3DSubsystem* Subsystem = nullptr;
	b3BodyId A, B;
	if (!ResolveBodies(BodyA, BodyB, TEXT("Prismatic"), Subsystem, A, B))
	{
		return FBox3DJointHandle();
	}

	b3Transform FrameA, FrameB;
	Box3D::BuildJointFrames(A, B, WorldAnchor, WorldAxis, FrameA, FrameB);

	const float ToMeters = static_cast<float>(Box3D::UnrealToMeters);

	b3PrismaticJointDef Def = b3DefaultPrismaticJointDef();
	ApplyBaseSettings(Def.base, A, B, FrameA, FrameB, Settings);
	Def.enableLimit = Prismatic.bEnableLimit;
	Def.lowerTranslation = Prismatic.LowerTranslation * ToMeters;
	Def.upperTranslation = Prismatic.UpperTranslation * ToMeters;
	Def.enableMotor = Prismatic.bEnableMotor;
	Def.motorSpeed = Prismatic.MotorSpeed * ToMeters;
	Def.maxMotorForce = Prismatic.MaxMotorForce;

	return Subsystem->RegisterJoint(b3CreatePrismaticJoint(Subsystem->GetWorldId(), &Def),
		EBox3DJointType::Prismatic, Settings);
}

FBox3DJointHandle UBox3DJointLibrary::CreateWeldJoint(UBox3DBodyComponent* BodyA,
	UBox3DBodyComponent* BodyB, const FVector& WorldAnchor,
	const FBox3DJointSettings& Settings, const FBox3DWeldJointSettings& Weld)
{
	UBox3DSubsystem* Subsystem = nullptr;
	b3BodyId A, B;
	if (!ResolveBodies(BodyA, BodyB, TEXT("Weld"), Subsystem, A, B))
	{
		return FBox3DJointHandle();
	}

	// A weld has no privileged axis; any consistent frame works.
	b3Transform FrameA, FrameB;
	Box3D::BuildJointFrames(A, B, WorldAnchor, FVector::UpVector, FrameA, FrameB);

	b3WeldJointDef Def = b3DefaultWeldJointDef();
	ApplyBaseSettings(Def.base, A, B, FrameA, FrameB, Settings);
	Def.linearHertz = Weld.LinearHertz;
	Def.angularHertz = Weld.AngularHertz;
	Def.linearDampingRatio = Weld.LinearDampingRatio;
	Def.angularDampingRatio = Weld.AngularDampingRatio;

	return Subsystem->RegisterJoint(b3CreateWeldJoint(Subsystem->GetWorldId(), &Def),
		EBox3DJointType::Weld, Settings);
}

void UBox3DJointLibrary::DestroyJoint(UObject* WorldContext, FBox3DJointHandle Joint, bool bWakeBodies)
{
	if (UBox3DSubsystem* Subsystem = GetSubsystem(WorldContext))
	{
		Subsystem->DestroyJoint(Joint, bWakeBodies);
	}
}

bool UBox3DJointLibrary::IsJointValid(UObject* WorldContext, FBox3DJointHandle Joint)
{
	UBox3DSubsystem* Subsystem = GetSubsystem(WorldContext);
	return Subsystem != nullptr && B3_IS_NON_NULL(Subsystem->ResolveJoint(Joint));
}

float UBox3DJointLibrary::GetJointForce(UObject* WorldContext, FBox3DJointHandle Joint)
{
	UBox3DSubsystem* Subsystem = GetSubsystem(WorldContext);
	if (Subsystem == nullptr)
	{
		return 0.0f;
	}
	Subsystem->FlushAsyncStep();
	const b3JointId Id = Subsystem->ResolveJoint(Joint);
	return B3_IS_NON_NULL(Id) ? b3Length(b3Joint_GetConstraintForce(Id)) : 0.0f;
}

float UBox3DJointLibrary::GetJointTorque(UObject* WorldContext, FBox3DJointHandle Joint)
{
	UBox3DSubsystem* Subsystem = GetSubsystem(WorldContext);
	if (Subsystem == nullptr)
	{
		return 0.0f;
	}
	Subsystem->FlushAsyncStep();
	const b3JointId Id = Subsystem->ResolveJoint(Joint);
	return B3_IS_NON_NULL(Id) ? b3Length(b3Joint_GetConstraintTorque(Id)) : 0.0f;
}

void UBox3DJointLibrary::SetMotorSpeed(UObject* WorldContext, FBox3DJointHandle Joint, float Speed)
{
	UBox3DSubsystem* Subsystem = GetSubsystem(WorldContext);
	if (Subsystem == nullptr)
	{
		return;
	}
	Subsystem->FlushAsyncStep();
	const b3JointId Id = Subsystem->ResolveJoint(Joint);
	if (B3_IS_NULL(Id))
	{
		return;
	}

	switch (b3Joint_GetType(Id))
	{
	case b3_revoluteJoint:
		b3RevoluteJoint_SetMotorSpeed(Id, FMath::DegreesToRadians(Speed));
		break;
	case b3_prismaticJoint:
		b3PrismaticJoint_SetMotorSpeed(Id, Speed * static_cast<float>(Box3D::UnrealToMeters));
		break;
	default:
		UE_LOG(LogBox3D, Warning, TEXT("SetMotorSpeed: joint type has no linear/angular motor."));
		break;
	}
	b3Joint_WakeBodies(Id);
}

void UBox3DJointLibrary::SetMotorEnabled(UObject* WorldContext, FBox3DJointHandle Joint, bool bEnabled)
{
	UBox3DSubsystem* Subsystem = GetSubsystem(WorldContext);
	if (Subsystem == nullptr)
	{
		return;
	}
	Subsystem->FlushAsyncStep();
	const b3JointId Id = Subsystem->ResolveJoint(Joint);
	if (B3_IS_NULL(Id))
	{
		return;
	}

	switch (b3Joint_GetType(Id))
	{
	case b3_revoluteJoint:  b3RevoluteJoint_EnableMotor(Id, bEnabled);  break;
	case b3_prismaticJoint: b3PrismaticJoint_EnableMotor(Id, bEnabled); break;
	default: break;
	}
	b3Joint_WakeBodies(Id);
}
