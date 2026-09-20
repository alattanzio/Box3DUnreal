// Author: Antonio Lattanzio - emptyvessel

#pragma once

#include "CoreMinimal.h"
#include "Box3DEventTypes.h"
#include "UObject/Object.h"
#include "Box3DEventTest.generated.h"

/** Tally target for box3d.EventTest. Blueprint delegates only bind UFUNCTIONs, so the test
 *  needs a real UObject to listen with - hence a header for a one-file class. */
UCLASS()
class UBox3DEventTestListener : public UObject
{
	GENERATED_BODY()

public:
	UFUNCTION()
	void HandleBeginContact(const FBox3DTouchEvent& Event);

	UFUNCTION()
	void HandleEndContact(const FBox3DTouchEvent& Event);

	UFUNCTION()
	void HandleHit(const FBox3DHitEvent& Event);

	UFUNCTION()
	void HandleBeginOverlap(const FBox3DTouchEvent& Event);

	UFUNCTION()
	void HandleEndOverlap(const FBox3DTouchEvent& Event);

	UFUNCTION()
	void HandleSleep(const FBox3DSleepEvent& Event);

	int32 BeginContactCount = 0;
	int32 EndContactCount = 0;
	int32 HitCount = 0;
	int32 BeginOverlapCount = 0;
	int32 EndOverlapCount = 0;
	bool bFellAsleep = false;

	/** Checked once the drop has finished. */
	UPROPERTY(Transient)
	FBox3DHitEvent FirstHit;

	UPROPERTY(Transient)
	TObjectPtr<AActor> FirstContactOther = nullptr;
};
