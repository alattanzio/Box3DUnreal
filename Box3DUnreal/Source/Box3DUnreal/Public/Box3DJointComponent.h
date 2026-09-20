// Author: Antonio Lattanzio - emptyvessel

#pragma once

#include "Box3DJointTypes.h"
#include "Components/ActorComponent.h"
#include "CoreMinimal.h"
#include "Box3DJointComponent.generated.h"

class UBox3DBodyComponent;

/** One joint authored in the details panel. */
USTRUCT(BlueprintType)
struct BOX3DUNREAL_API FBox3DAuthoredJoint
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Category = "Box3D|Joint")
	EBox3DJointType Type = EBox3DJointType::Revolute;

	/** Body component on this actor. Leave empty to use the actor's first body component. */
	UPROPERTY(EditAnywhere, Category = "Box3D|Joint")
	FName BodyA = NAME_None;

	/** The other actor to attach to. Leave null to joint against BodyB on this actor. */
	UPROPERTY(EditAnywhere, Category = "Box3D|Joint")
	TSoftObjectPtr<AActor> OtherActor;

	UPROPERTY(EditAnywhere, Category = "Box3D|Joint")
	FName BodyB = NAME_None;

	/** Anchor relative to this component's owner. */
	UPROPERTY(EditAnywhere, Category = "Box3D|Joint")
	FVector LocalAnchor = FVector::ZeroVector;

	/** Hinge/slider axis, or the cone centre for a spherical joint. Actor-local. */
	UPROPERTY(EditAnywhere, Category = "Box3D|Joint")
	FVector LocalAxis = FVector::UpVector;

	UPROPERTY(EditAnywhere, Category = "Box3D|Joint", meta = (ShowOnlyInnerProperties))
	FBox3DJointSettings Settings;

	UPROPERTY(EditAnywhere, Category = "Box3D|Joint",
		meta = (EditCondition = "Type == EBox3DJointType::Spherical", EditConditionHides))
	FBox3DSphericalJointSettings Spherical;

	UPROPERTY(EditAnywhere, Category = "Box3D|Joint",
		meta = (EditCondition = "Type == EBox3DJointType::Revolute", EditConditionHides))
	FBox3DRevoluteJointSettings Revolute;

	UPROPERTY(EditAnywhere, Category = "Box3D|Joint",
		meta = (EditCondition = "Type == EBox3DJointType::Prismatic", EditConditionHides))
	FBox3DPrismaticJointSettings Prismatic;

	UPROPERTY(EditAnywhere, Category = "Box3D|Joint",
		meta = (EditCondition = "Type == EBox3DJointType::Weld", EditConditionHides))
	FBox3DWeldJointSettings Weld;
};

/**
 * Details-panel authoring for joints. A thin shell over UBox3DJointLibrary - it resolves
 * the named bodies and converts anchors to world space; nothing here talks to box3d.
 *
 * Built one tick after BeginPlay, so both bodies exist whatever order they initialised in.
 */
UCLASS(ClassGroup = (Physics), meta = (BlueprintSpawnableComponent, DisplayName = "Box3D Joint"))
class BOX3DUNREAL_API UBox3DJointComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UBox3DJointComponent();

	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

	/** Handles for the authored joints, in order. An entry is unset if creation failed. */
	UFUNCTION(BlueprintPure, Category = "Box3D|Joint")
	const TArray<FBox3DJointHandle>& GetCreatedJoints() const { return CreatedJoints; }

	/** Create the authored joints now. Called automatically; safe to call again after a
	 *  DestroyJoints to rebuild. */
	UFUNCTION(BlueprintCallable, Category = "Box3D|Joint")
	void CreateJoints();

	UFUNCTION(BlueprintCallable, Category = "Box3D|Joint")
	void DestroyJoints();

protected:
	UPROPERTY(EditAnywhere, Category = "Box3D|Joint", meta = (TitleProperty = "Type"))
	TArray<FBox3DAuthoredJoint> Joints;

	/** Resolve a body component by name on Actor, falling back to the first one found. */
	UBox3DBodyComponent* FindBody(AActor* Actor, FName ComponentName) const;

private:
	TArray<FBox3DJointHandle> CreatedJoints;

	/** Set once joints are built, so a manual CreateJoints doesn't double up. */
	bool bJointsCreated = false;
};
