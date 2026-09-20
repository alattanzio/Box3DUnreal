// Author: Antonio Lattanzio - emptyvessel

#pragma once

#include "CoreMinimal.h"
#include "Box3DEventTypes.generated.h"

class AActor;
class UBox3DBodyComponent;

/**
 * A contact or overlap starting or ending, reported from one body's side: Body/Actor is who
 * the event fired on, Other* is what it touched. Both sides get their own event.
 */
USTRUCT(BlueprintType)
struct BOX3DUNREAL_API FBox3DTouchEvent
{
	GENERATED_BODY()

	/** The body this fired on. For an overlap, the sensor. */
	UPROPERTY(BlueprintReadOnly, Category = "Box3D|Events")
	TObjectPtr<UBox3DBodyComponent> Body = nullptr;

	UPROPERTY(BlueprintReadOnly, Category = "Box3D|Events")
	TObjectPtr<AActor> Actor = nullptr;

	/** Null for bulk/baked static geometry, which has no component. */
	UPROPERTY(BlueprintReadOnly, Category = "Box3D|Events")
	TObjectPtr<UBox3DBodyComponent> OtherBody = nullptr;

	/** Null for baked static geometry, which has no source actor. */
	UPROPERTY(BlueprintReadOnly, Category = "Box3D|Events")
	TObjectPtr<AActor> OtherActor = nullptr;

	/** Fixed step this happened on. */
	UPROPERTY(BlueprintReadOnly, Category = "Box3D|Events")
	int64 SimulationFrame = 0;
};

/** A collision hard enough to matter: impact sounds, damage, decals. */
USTRUCT(BlueprintType)
struct BOX3DUNREAL_API FBox3DHitEvent
{
	GENERATED_BODY()

	UPROPERTY(BlueprintReadOnly, Category = "Box3D|Events")
	TObjectPtr<UBox3DBodyComponent> Body = nullptr;

	UPROPERTY(BlueprintReadOnly, Category = "Box3D|Events")
	TObjectPtr<AActor> Actor = nullptr;

	UPROPERTY(BlueprintReadOnly, Category = "Box3D|Events")
	TObjectPtr<UBox3DBodyComponent> OtherBody = nullptr;

	UPROPERTY(BlueprintReadOnly, Category = "Box3D|Events")
	TObjectPtr<AActor> OtherActor = nullptr;

	/** Impact point in world space, cm. */
	UPROPERTY(BlueprintReadOnly, Category = "Box3D|Events")
	FVector Location = FVector::ZeroVector;

	/** Points from the other surface toward this body, like FHitResult::ImpactNormal. */
	UPROPERTY(BlueprintReadOnly, Category = "Box3D|Events")
	FVector Normal = FVector::ZeroVector;

	/** Closing speed at impact, cm/s. Scale damage or volume by this. */
	UPROPERTY(BlueprintReadOnly, Category = "Box3D|Events")
	float ApproachSpeed = 0.0f;

	UPROPERTY(BlueprintReadOnly, Category = "Box3D|Events")
	int64 SimulationFrame = 0;
};

/** A body coming to rest or waking back up. */
USTRUCT(BlueprintType)
struct BOX3DUNREAL_API FBox3DSleepEvent
{
	GENERATED_BODY()

	UPROPERTY(BlueprintReadOnly, Category = "Box3D|Events")
	TObjectPtr<UBox3DBodyComponent> Body = nullptr;

	UPROPERTY(BlueprintReadOnly, Category = "Box3D|Events")
	TObjectPtr<AActor> Actor = nullptr;

	UPROPERTY(BlueprintReadOnly, Category = "Box3D|Events")
	int64 SimulationFrame = 0;
};

DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FBox3DTouchSignature, const FBox3DTouchEvent&, Event);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FBox3DHitSignature, const FBox3DHitEvent&, Event);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FBox3DSleepSignature, const FBox3DSleepEvent&, Event);
