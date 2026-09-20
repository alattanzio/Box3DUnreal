// Author: Antonio Lattanzio - emptyvessel

#pragma once

#include "CoreMinimal.h"
#include "Box3DJointTypes.generated.h"

UENUM(BlueprintType)
enum class EBox3DJointType : uint8
{
	Spherical,  // Ball socket with optional cone + twist limits. The ragdoll joint.
	Revolute,   // Single-axis hinge about the frame Z axis. Doors, wheels, elbows.
	Prismatic,  // Slider along the frame Z axis.
	Weld        // Rigid attach, optionally softened by hertz.
};

/** Handle to a joint owned by UBox3DSubsystem. Index+serial, so a handle to a destroyed
 *  joint resolves to null rather than to whatever reused the slot. */
USTRUCT(BlueprintType)
struct BOX3DUNREAL_API FBox3DJointHandle
{
	GENERATED_BODY()

	UPROPERTY(BlueprintReadOnly, Category = "Box3D|Joint")
	int32 Index = INDEX_NONE;

	UPROPERTY(BlueprintReadOnly, Category = "Box3D|Joint")
	int32 Serial = 0;

	bool IsSet() const { return Index != INDEX_NONE; }

	friend bool operator==(const FBox3DJointHandle& A, const FBox3DJointHandle& B)
	{
		return A.Index == B.Index && A.Serial == B.Serial;
	}
};

/** Settings common to every joint type. */
USTRUCT(BlueprintType)
struct BOX3DUNREAL_API FBox3DJointSettings
{
	GENERATED_BODY()

	/** Let the two jointed bodies collide with each other. Off for ragdoll limbs. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Joint")
	bool bCollideConnected = false;

	/** Constraint softness, cycles/sec. 60 is box3d's default; lower is springier. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Joint", meta = (ClampMin = "0.0"))
	float ConstraintHertz = 60.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Joint", meta = (ClampMin = "0.0"))
	float ConstraintDampingRatio = 2.0f;

	/** Constraint force (N) above which the joint reports a break event. 0 = never. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Joint", meta = (ClampMin = "0.0"))
	float BreakForce = 0.0f;

	/** Constraint torque (N*m) above which the joint reports a break event. 0 = never. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Joint", meta = (ClampMin = "0.0"))
	float BreakTorque = 0.0f;

	/** Destroy the joint when a break threshold is crossed, rather than only reporting it. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Joint")
	bool bBreakable = false;
};

/** Ball socket. Cone limit is about the frame A Z axis, twist about frame B Z. */
USTRUCT(BlueprintType)
struct BOX3DUNREAL_API FBox3DSphericalJointSettings
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Joint|Spherical")
	bool bEnableConeLimit = true;

	/** Half-angle of the swing cone, degrees. Range [0, 180]. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Joint|Spherical",
		meta = (ClampMin = "0.0", ClampMax = "180.0", EditCondition = "bEnableConeLimit"))
	float ConeAngle = 45.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Joint|Spherical")
	bool bEnableTwistLimit = true;

	/** Twist limits in degrees. Clamped to +-178 (box3d caps at 0.99*pi). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Joint|Spherical",
		meta = (ClampMin = "-178.0", ClampMax = "178.0", EditCondition = "bEnableTwistLimit"))
	float LowerTwistAngle = -30.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Joint|Spherical",
		meta = (ClampMin = "-178.0", ClampMax = "178.0", EditCondition = "bEnableTwistLimit"))
	float UpperTwistAngle = 30.0f;

	/** Spring pulling frame B back toward frame A. Off leaves the joint free inside limits. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Joint|Spherical")
	bool bEnableSpring = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Joint|Spherical",
		meta = (ClampMin = "0.0", EditCondition = "bEnableSpring"))
	float Hertz = 2.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Joint|Spherical",
		meta = (ClampMin = "0.0", EditCondition = "bEnableSpring"))
	float DampingRatio = 0.5f;
};

/** Hinge about the joint frame Z axis. */
USTRUCT(BlueprintType)
struct BOX3DUNREAL_API FBox3DRevoluteJointSettings
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Joint|Revolute")
	bool bEnableLimit = false;

	/** Degrees, clamped to +-178. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Joint|Revolute",
		meta = (ClampMin = "-178.0", ClampMax = "178.0", EditCondition = "bEnableLimit"))
	float LowerAngle = -90.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Joint|Revolute",
		meta = (ClampMin = "-178.0", ClampMax = "178.0", EditCondition = "bEnableLimit"))
	float UpperAngle = 90.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Joint|Revolute")
	bool bEnableMotor = false;

	/** Target speed in deg/s. Sign follows the Unreal frame Z axis. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Joint|Revolute",
		meta = (EditCondition = "bEnableMotor"))
	float MotorSpeed = 90.0f;

	/** Maximum motor torque in N*m. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Joint|Revolute",
		meta = (ClampMin = "0.0", EditCondition = "bEnableMotor"))
	float MaxMotorTorque = 100.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Joint|Revolute")
	bool bEnableSpring = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Joint|Revolute",
		meta = (ClampMin = "0.0", EditCondition = "bEnableSpring"))
	float Hertz = 2.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Joint|Revolute",
		meta = (ClampMin = "0.0", EditCondition = "bEnableSpring"))
	float DampingRatio = 0.5f;
};

/** Slider along the joint frame Z axis. */
USTRUCT(BlueprintType)
struct BOX3DUNREAL_API FBox3DPrismaticJointSettings
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Joint|Prismatic")
	bool bEnableLimit = true;

	/** Travel limits in cm. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Joint|Prismatic",
		meta = (EditCondition = "bEnableLimit"))
	float LowerTranslation = -100.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Joint|Prismatic",
		meta = (EditCondition = "bEnableLimit"))
	float UpperTranslation = 100.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Joint|Prismatic")
	bool bEnableMotor = false;

	/** Target speed in cm/s. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Joint|Prismatic",
		meta = (EditCondition = "bEnableMotor"))
	float MotorSpeed = 100.0f;

	/** Maximum motor force in newtons. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Joint|Prismatic",
		meta = (ClampMin = "0.0", EditCondition = "bEnableMotor"))
	float MaxMotorForce = 1000.0f;
};

/** Rigid attach. Zero hertz means fully stiff. */
USTRUCT(BlueprintType)
struct BOX3DUNREAL_API FBox3DWeldJointSettings
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Joint|Weld", meta = (ClampMin = "0.0"))
	float LinearHertz = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Joint|Weld", meta = (ClampMin = "0.0"))
	float AngularHertz = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Joint|Weld", meta = (ClampMin = "0.0"))
	float LinearDampingRatio = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Joint|Weld", meta = (ClampMin = "0.0"))
	float AngularDampingRatio = 1.0f;
};

/** Fires when a joint crosses its break threshold. Force is N, torque N*m. */
DECLARE_DYNAMIC_MULTICAST_DELEGATE_ThreeParams(FBox3DJointBreakSignature,
	FBox3DJointHandle, Joint, float, Force, float, Torque);
