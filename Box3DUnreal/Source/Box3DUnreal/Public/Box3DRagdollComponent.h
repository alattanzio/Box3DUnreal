// Author: Antonio Lattanzio - emptyvessel

#pragma once

#include "Box3DJointTypes.h"
#include "Components/ActorComponent.h"
#include "CoreMinimal.h"
#include <box3d/box3d.h>
#include "Box3DRagdollComponent.generated.h"

class USkeletalMeshComponent;
class UBox3DSubsystem;

/**
 * Drives a skeletal mesh as a box3d ragdoll, built from its UPhysicsAsset: a body per
 * SkeletalBodySetup, a spherical joint per constraint. Any rig set up for Chaos ragdolls
 * works unmodified. The pose is written back through component-space bone transforms.
 *
 * Server-authoritative: on a client this does nothing and the mesh keeps animating.
 */
UCLASS(ClassGroup = (Physics), meta = (BlueprintSpawnableComponent, DisplayName = "Box3D Ragdoll"))
class BOX3DUNREAL_API UBox3DRagdollComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UBox3DRagdollComponent();

	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType,
		FActorComponentTickFunction* ThisTickFunction) override;

	/** Build the ragdoll from the mesh's physics asset and hand it over to box3d.
	 *  The mesh stops being animated from this point. */
	UFUNCTION(BlueprintCallable, Category = "Box3D|Ragdoll")
	void StartRagdoll();

	/** Destroy the bodies and joints. The mesh does not resume animating on its own. */
	UFUNCTION(BlueprintCallable, Category = "Box3D|Ragdoll")
	void StopRagdoll();

	UFUNCTION(BlueprintPure, Category = "Box3D|Ragdoll")
	bool IsRagdollActive() const { return bActive; }

	/** Impulse (kg*cm/s) at a world point - the shot that knocks the ragdoll over. Applies
	 *  to the body nearest the point. */
	UFUNCTION(BlueprintCallable, Category = "Box3D|Ragdoll")
	void AddImpulseAtLocation(const FVector& Impulse, const FVector& WorldLocation);

	/** Start simulating as soon as the component begins play. Off means StartRagdoll only. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Ragdoll")
	bool bStartActive = false;

	/** Mesh to drive. Defaults to the owner's first skeletal mesh component. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Ragdoll")
	FName SkeletalMeshComponentName = NAME_None;

	UPROPERTY(EditAnywhere, Category = "Box3D|Ragdoll|Material", meta = (ClampMin = "0.0"))
	float Density = 1000.0f;

	UPROPERTY(EditAnywhere, Category = "Box3D|Ragdoll|Material", meta = (ClampMin = "0.0"))
	float Friction = 0.6f;

	UPROPERTY(EditAnywhere, Category = "Box3D|Ragdoll|Material", meta = (ClampMin = "0.0"))
	float Restitution = 0.0f;

	UPROPERTY(EditAnywhere, Category = "Box3D|Ragdoll|Material", meta = (ClampMin = "0.0"))
	float AngularDamping = 0.15f;

	/** Joint softness. Lower is looser; raise it if limbs feel rubbery. */
	UPROPERTY(EditAnywhere, Category = "Box3D|Ragdoll|Joints", meta = (ClampMin = "0.0"))
	float ConstraintHertz = 60.0f;

	UPROPERTY(EditAnywhere, Category = "Box3D|Ragdoll|Joints", meta = (ClampMin = "0.0"))
	float ConstraintDampingRatio = 2.0f;

	/** Scales the asset's swing limits. The asset's cone is elliptical and box3d's is
	 *  circular, so the import takes the larger axis; trim this if limbs feel loose. */
	UPROPERTY(EditAnywhere, Category = "Box3D|Ragdoll|Joints", meta = (ClampMin = "0.1", ClampMax = "2.0"))
	float SwingLimitScale = 1.0f;

	/** Let bones jointed together collide. Off by default - neighbours always overlap. */
	UPROPERTY(EditAnywhere, Category = "Box3D|Ragdoll|Joints")
	bool bCollideConnected = false;

protected:
	/** One simulated bone: its box3d body plus what is needed to pose the skeleton. */
	struct FRagdollBone
	{
		b3BodyId Body = b3_nullBodyId;
		FName BoneName;
		int32 BoneIndex = INDEX_NONE;

		/** Bone space -> body space. The physics body is usually offset from the bone. */
		FTransform BoneToBody = FTransform::Identity;
	};

	/** Create the bodies for every setup in the physics asset. Returns bones created. */
	int32 BuildBodies(USkeletalMeshComponent* Mesh, UBox3DSubsystem* Subsystem);

	/** Create a spherical joint per constraint in the asset. */
	int32 BuildJoints(USkeletalMeshComponent* Mesh, UBox3DSubsystem* Subsystem);

	/** Read the simulated bodies back into the mesh's component-space bone transforms. */
	void PoseMeshFromBodies();

	int32 FindBoneEntry(FName BoneName) const;

private:
	UPROPERTY(Transient)
	TObjectPtr<USkeletalMeshComponent> Mesh = nullptr;

	UPROPERTY(Transient)
	TObjectPtr<UBox3DSubsystem> Subsystem = nullptr;

	TArray<FRagdollBone> Bones;
	TArray<FBox3DJointHandle> Joints;

	bool bActive = false;
};
