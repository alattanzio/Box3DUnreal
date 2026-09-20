// Author: Antonio Lattanzio - emptyvessel

#pragma once

#include "CoreMinimal.h"
#include "Box3DJointTypes.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include <box3d/box3d.h>
#include "Box3DJoints.generated.h"

class UBox3DBodyComponent;
class UBox3DSubsystem;

/**
 * Joint creation and control. The primitive layer: the joint component and the ragdoll
 * builder both go through here rather than touching box3d. Joints are owned by
 * UBox3DSubsystem, so they die with the world.
 *
 * Anchors and axes are Unreal world space (cm, Z-up). Each Create derives both local
 * frames from the anchor, so the bodies keep the relative pose they are already in.
 */
UCLASS()
class BOX3DUNREAL_API UBox3DJointLibrary : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:
	/** Ball socket at WorldAnchor. TwistAxis is the cone centre - for a limb, point it down
	 *  the bone. Unset handle if either body has no box3d body (client, or box3d off). */
	UFUNCTION(BlueprintCallable, Category = "Box3D|Joint",
		meta = (DisplayName = "Create Spherical Joint"))
	static FBox3DJointHandle CreateSphericalJoint(UBox3DBodyComponent* BodyA, UBox3DBodyComponent* BodyB,
		const FVector& WorldAnchor, const FVector& TwistAxis,
		const FBox3DJointSettings& Settings, const FBox3DSphericalJointSettings& Spherical);

	/** Hinge at WorldAnchor turning about WorldAxis. */
	UFUNCTION(BlueprintCallable, Category = "Box3D|Joint",
		meta = (DisplayName = "Create Revolute Joint"))
	static FBox3DJointHandle CreateRevoluteJoint(UBox3DBodyComponent* BodyA, UBox3DBodyComponent* BodyB,
		const FVector& WorldAnchor, const FVector& WorldAxis,
		const FBox3DJointSettings& Settings, const FBox3DRevoluteJointSettings& Revolute);

	/** Slider at WorldAnchor travelling along WorldAxis. */
	UFUNCTION(BlueprintCallable, Category = "Box3D|Joint",
		meta = (DisplayName = "Create Prismatic Joint"))
	static FBox3DJointHandle CreatePrismaticJoint(UBox3DBodyComponent* BodyA, UBox3DBodyComponent* BodyB,
		const FVector& WorldAnchor, const FVector& WorldAxis,
		const FBox3DJointSettings& Settings, const FBox3DPrismaticJointSettings& Prismatic);

	/** Rigid attach at WorldAnchor. */
	UFUNCTION(BlueprintCallable, Category = "Box3D|Joint",
		meta = (DisplayName = "Create Weld Joint"))
	static FBox3DJointHandle CreateWeldJoint(UBox3DBodyComponent* BodyA, UBox3DBodyComponent* BodyB,
		const FVector& WorldAnchor,
		const FBox3DJointSettings& Settings, const FBox3DWeldJointSettings& Weld);

	/** Destroy a joint. Safe on an already-destroyed or unset handle. */
	UFUNCTION(BlueprintCallable, Category = "Box3D|Joint")
	static void DestroyJoint(UObject* WorldContext, FBox3DJointHandle Joint, bool bWakeBodies = true);

	UFUNCTION(BlueprintPure, Category = "Box3D|Joint")
	static bool IsJointValid(UObject* WorldContext, FBox3DJointHandle Joint);

	/** Current constraint force in newtons - the number to compare against a break threshold. */
	UFUNCTION(BlueprintPure, Category = "Box3D|Joint")
	static float GetJointForce(UObject* WorldContext, FBox3DJointHandle Joint);

	/** Current constraint torque in N*m. */
	UFUNCTION(BlueprintPure, Category = "Box3D|Joint")
	static float GetJointTorque(UObject* WorldContext, FBox3DJointHandle Joint);

	/** Revolute/prismatic only. Speed is deg/s for a hinge, cm/s for a slider. */
	UFUNCTION(BlueprintCallable, Category = "Box3D|Joint")
	static void SetMotorSpeed(UObject* WorldContext, FBox3DJointHandle Joint, float Speed);

	UFUNCTION(BlueprintCallable, Category = "Box3D|Joint")
	static void SetMotorEnabled(UObject* WorldContext, FBox3DJointHandle Joint, bool bEnabled);
};

namespace Box3D
{
	/** World anchor + axis -> the two body-local joint frames. Z is put along Axis, since
	 *  box3d defines every limit about the frame Z. */
	void BOX3DUNREAL_API BuildJointFrames(b3BodyId BodyA, b3BodyId BodyB, const FVector& WorldAnchor,
		const FVector& WorldAxis, b3Transform& OutFrameA, b3Transform& OutFrameB);
}
