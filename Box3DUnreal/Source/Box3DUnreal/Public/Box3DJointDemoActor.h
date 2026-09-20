// Author: Antonio Lattanzio - emptyvessel

#pragma once

#include "Box3DJointTypes.h"
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Box3DJointDemoActor.generated.h"

class UBox3DBodyComponent;
class UStaticMesh;

UENUM(BlueprintType)
enum class EBox3DJointDemo : uint8
{
	Chain,   // Rope of capsules hanging from a fixed anchor.
	Bridge,  // Plank walkway on hinges, anchored at both ends.
	Newton,  // Newton's cradle: a row of spheres on pendulums.
	Motor    // Hinged arm driven by a revolute motor.
};

/** Body material for a demo link. Applied before the body is created, since the box3d body
 *  is built from these at BeginPlay and later edits never reach the simulation. */
struct FBox3DDemoMaterial
{
	float Restitution = 0.0f;
	float LinearDamping = 0.0f;
	float AngularDamping = 0.05f;
};

/**
 * Drop-in demo of the joint layer: builds a rig from primitive meshes at BeginPlay.
 * Everything goes through UBox3DJointLibrary, so it doubles as the worked C++ example.
 */
UCLASS(meta = (DisplayName = "Box3D Joint Demo"))
class BOX3DUNREAL_API ABox3DJointDemoActor : public AActor
{
	GENERATED_BODY()

public:
	ABox3DJointDemoActor();

	virtual void BeginPlay() override;

	UPROPERTY(EditAnywhere, Category = "Box3D|Demo")
	EBox3DJointDemo Demo = EBox3DJointDemo::Chain;

	/** Links in the chain / planks in the bridge / balls in the cradle. */
	UPROPERTY(EditAnywhere, Category = "Box3D|Demo", meta = (ClampMin = "2", ClampMax = "64"))
	int32 LinkCount = 8;

	/** Spacing between links, cm. */
	UPROPERTY(EditAnywhere, Category = "Box3D|Demo", meta = (ClampMin = "10.0"))
	float LinkSpacing = 60.0f;

	/** Mesh used for each link. Defaults to the engine cube. */
	UPROPERTY(EditAnywhere, Category = "Box3D|Demo")
	TObjectPtr<UStaticMesh> LinkMesh = nullptr;

	/** Mesh for the cradle's balls. Defaults to the engine sphere. */
	UPROPERTY(EditAnywhere, Category = "Box3D|Demo")
	TObjectPtr<UStaticMesh> SphereMesh = nullptr;

	/** Motor demo: hinge speed in deg/s. */
	UPROPERTY(EditAnywhere, Category = "Box3D|Demo",
		meta = (EditCondition = "Demo == EBox3DJointDemo::Motor", EditConditionHides))
	float MotorSpeed = 120.0f;

protected:
	/** Spawn one demo body. bSphere swaps the box for a sphere of radius HalfExtent.X -
	 *  the cradle needs it, since boxes meet at corners. */
	UBox3DBodyComponent* SpawnLink(const FVector& WorldLocation, const FVector& HalfExtent,
		bool bStatic, bool bSphere = false, const FBox3DDemoMaterial* Material = nullptr);

	void BuildChain();
	void BuildBridge();
	void BuildNewtonsCradle();
	void BuildMotor();

private:
	UPROPERTY(Transient)
	TArray<TObjectPtr<AActor>> SpawnedLinks;

	TArray<FBox3DJointHandle> DemoJoints;
};
