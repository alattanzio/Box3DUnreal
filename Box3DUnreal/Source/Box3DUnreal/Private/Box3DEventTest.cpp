// Author: Antonio Lattanzio - emptyvessel


#include "Box3DEventTest.h"

#include "Box3DBodyComponent.h"
#include "Box3DLog.h"
#include "Box3DSubsystem.h"
#include "Components/StaticMeshComponent.h"
#include "Containers/Ticker.h"
#include "Engine/StaticMesh.h"
#include "Engine/StaticMeshActor.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "HAL/IConsoleManager.h"

void UBox3DEventTestListener::HandleBeginContact(const FBox3DTouchEvent& Event)
{
	if (BeginContactCount++ == 0)
	{
		FirstContactOther = Event.OtherActor;
	}
}

void UBox3DEventTestListener::HandleEndContact(const FBox3DTouchEvent& Event)
{
	++EndContactCount;
}

void UBox3DEventTestListener::HandleHit(const FBox3DHitEvent& Event)
{
	if (HitCount++ == 0)
	{
		FirstHit = Event;
	}
}

void UBox3DEventTestListener::HandleBeginOverlap(const FBox3DTouchEvent& Event)
{
	++BeginOverlapCount;
}

void UBox3DEventTestListener::HandleEndOverlap(const FBox3DTouchEvent& Event)
{
	++EndOverlapCount;
}

void UBox3DEventTestListener::HandleSleep(const FBox3DSleepEvent& Event)
{
	bFellAsleep = true;
}

namespace
{
	// Far above any level geometry, so nothing else can take part.
	constexpr double EvtFloorZ = 50000.0;   // 100cm mesh scaled flat, top face at +50
	constexpr double EvtTriggerZ = 50250.0; // 100cm cube, spans 50200..50300
	constexpr double EvtCubeZ = 50600.0;    // falls 500cm onto the floor

	/** Covers the ~1s drop plus settling. */
	constexpr float EvtRunSeconds = 6.0f;

	struct FEventTestTally
	{
		int32 Passed = 0;
		int32 Failed = 0;
	};

	void Check(FEventTestTally& Tally, bool bCondition, const FString& What)
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

	/** A cube actor with a box3d body. Configure runs before the component registers. */
	AStaticMeshActor* SpawnCube(UWorld* World, const FVector& Location, const FVector& Scale,
		TFunctionRef<void(UBox3DBodyComponent&)> Configure)
	{
		UStaticMesh* Cube = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cube.Cube"));
		if (Cube == nullptr)
		{
			return nullptr;
		}

		const FTransform Xform(FRotator::ZeroRotator, Location, Scale);
		AStaticMeshActor* Actor = World->SpawnActorDeferred<AStaticMeshActor>(
			AStaticMeshActor::StaticClass(), Xform, nullptr, nullptr,
			ESpawnActorCollisionHandlingMethod::AlwaysSpawn);
		if (Actor == nullptr)
		{
			return nullptr;
		}

		UStaticMeshComponent* MeshComp = Actor->GetStaticMeshComponent();
		MeshComp->SetMobility(EComponentMobility::Movable);
		MeshComp->SetStaticMesh(Cube);

		UBox3DBodyComponent* Body = NewObject<UBox3DBodyComponent>(Actor);
		Configure(*Body);
		Body->RegisterComponent();

		Actor->FinishSpawning(Xform);
		return Actor;
	}

	void ReportEventTest(UBox3DEventTestListener* Listener, AActor* Floor)
	{
		FEventTestTally Tally;

		Check(Tally, Listener->HitCount > 0, TEXT("landing raised a hit"));
		if (Listener->HitCount > 0)
		{
			const FBox3DHitEvent& Hit = Listener->FirstHit;

			// ~990 cm/s from a 500cm drop. A missing m/s -> cm/s would land near 9.9.
			Check(Tally, Hit.ApproachSpeed > 300.0f && Hit.ApproachSpeed < 2000.0f,
				FString::Printf(TEXT("approach speed %.1f cm/s is in range"), Hit.ApproachSpeed));

			Check(Tally, Hit.Normal.Z > 0.9,
				FString::Printf(TEXT("hit normal %s points up out of the floor"), *Hit.Normal.ToCompactString()));

			Check(Tally, FMath::Abs(Hit.Normal.Size() - 1.0) < 0.01,
				FString::Printf(TEXT("hit normal is unit length (%.3f)"), Hit.Normal.Size()));

			Check(Tally, Hit.OtherActor == Floor, TEXT("hit resolves the other side to the floor"));
			Check(Tally, Hit.Location.Z > EvtFloorZ && Hit.Location.Z < EvtCubeZ,
				FString::Printf(TEXT("hit location Z %.1f sits between floor and drop"), Hit.Location.Z));
		}

		Check(Tally, Listener->BeginContactCount > 0, TEXT("landing raised a begin contact"));
		Check(Tally, Listener->FirstContactOther == Floor, TEXT("begin contact resolves the floor"));

		Check(Tally, Listener->BeginOverlapCount > 0, TEXT("falling through the trigger raised begin overlap"));
		Check(Tally, Listener->EndOverlapCount > 0, TEXT("leaving the trigger raised end overlap"));

		Check(Tally, Listener->bFellAsleep, TEXT("the cube reported sleeping once it settled"));

		UE_LOG(LogBox3D, Log, TEXT("box3d.EventTest: %d passed, %d failed. ")
			TEXT("(hits %d, contact begin/end %d/%d, overlap begin/end %d/%d)"),
			Tally.Passed, Tally.Failed, Listener->HitCount, Listener->BeginContactCount,
			Listener->EndContactCount, Listener->BeginOverlapCount, Listener->EndOverlapCount);
	}

	void RunEventTest(UWorld* World)
	{
		UBox3DSubsystem* Subsystem = World ? World->GetSubsystem<UBox3DSubsystem>() : nullptr;
		if (Subsystem == nullptr || !Subsystem->IsWorldValid())
		{
			UE_LOG(LogBox3D, Warning, TEXT("box3d.EventTest: no box3d world (disabled, or a client)."));
			return;
		}

		// The floor opts into nothing: one side asking is enough for a contact or a hit.
		AStaticMeshActor* Floor = SpawnCube(World, FVector(0.0, 0.0, EvtFloorZ), FVector(10.0, 10.0, 1.0),
			[](UBox3DBodyComponent& Body)
			{
				Body.BodyType = EBox3DBodyType::Static;
			});

		AStaticMeshActor* Trigger = SpawnCube(World, FVector(0.0, 0.0, EvtTriggerZ), FVector::OneVector,
			[](UBox3DBodyComponent& Body)
			{
				Body.BodyType = EBox3DBodyType::Static;
				Body.StaticSource = EBox3DStaticSource::SimpleCollision; // a hull, not a tri-mesh
				Body.bIsSensor = true;
				Body.bGenerateSensorEvents = true;
			});

		AStaticMeshActor* Cube = SpawnCube(World, FVector(0.0, 0.0, EvtCubeZ), FVector::OneVector,
			[](UBox3DBodyComponent& Body)
			{
				Body.BodyType = EBox3DBodyType::Dynamic;
				Body.bGenerateContactEvents = true;
				Body.bGenerateHitEvents = true;
				Body.bGenerateSleepEvents = true;
				Body.bGenerateSensorEvents = true; // a trigger only sees bodies that opt in
			});

		if (Floor == nullptr || Trigger == nullptr || Cube == nullptr)
		{
			UE_LOG(LogBox3D, Warning, TEXT("box3d.EventTest: could not spawn the test actors."));
			return;
		}

		UBox3DEventTestListener* Listener = NewObject<UBox3DEventTestListener>();
		Listener->AddToRoot(); // nothing else holds it for the length of the drop

		UBox3DBodyComponent* CubeBody = Cube->FindComponentByClass<UBox3DBodyComponent>();
		UBox3DBodyComponent* TriggerBody = Trigger->FindComponentByClass<UBox3DBodyComponent>();
		CubeBody->OnBox3DBeginContact.AddDynamic(Listener, &UBox3DEventTestListener::HandleBeginContact);
		CubeBody->OnBox3DEndContact.AddDynamic(Listener, &UBox3DEventTestListener::HandleEndContact);
		CubeBody->OnBox3DHit.AddDynamic(Listener, &UBox3DEventTestListener::HandleHit);
		CubeBody->OnBox3DSleep.AddDynamic(Listener, &UBox3DEventTestListener::HandleSleep);
		TriggerBody->OnBox3DBeginOverlap.AddDynamic(Listener, &UBox3DEventTestListener::HandleBeginOverlap);
		TriggerBody->OnBox3DEndOverlap.AddDynamic(Listener, &UBox3DEventTestListener::HandleEndOverlap);

		UE_LOG(LogBox3D, Log, TEXT("box3d.EventTest: dropping a cube %.0fcm through a trigger onto a floor (%.0fs)..."),
			EvtCubeZ - EvtFloorZ, EvtRunSeconds);

		TWeakObjectPtr<UWorld> WeakWorld(World);
		TWeakObjectPtr<UBox3DEventTestListener> WeakListener(Listener);
		TWeakObjectPtr<AActor> WeakFloor(Floor);
		TWeakObjectPtr<AActor> WeakTrigger(Trigger);
		TWeakObjectPtr<AActor> WeakCube(Cube);
		float Elapsed = 0.0f;

		FTSTicker::GetCoreTicker().AddTicker(FTickerDelegate::CreateLambda(
			[WeakWorld, WeakListener, WeakFloor, WeakTrigger, WeakCube, Elapsed](float DeltaTime) mutable
			{
				UBox3DEventTestListener* Target = WeakListener.Get();
				if (Target == nullptr)
				{
					return false;
				}
				if (!WeakWorld.IsValid())
				{
					UE_LOG(LogBox3D, Warning, TEXT("box3d.EventTest: world ended before the drop finished."));
					Target->RemoveFromRoot();
					return false;
				}

				Elapsed += DeltaTime;
				if (Elapsed < EvtRunSeconds)
				{
					return true;
				}

				ReportEventTest(Target, WeakFloor.Get());

				for (const TWeakObjectPtr<AActor>& Actor : { WeakFloor, WeakTrigger, WeakCube })
				{
					if (AActor* Alive = Actor.Get())
					{
						Alive->Destroy();
					}
				}
				Target->RemoveFromRoot();
				return false;
			}), 0.0f);
	}

	FAutoConsoleCommandWithWorld GBox3DEventTestCommand(
		TEXT("box3d.EventTest"),
		TEXT("Self-checking physics-event test: drop a cube through a trigger onto a floor and verify the events."),
		FConsoleCommandWithWorldDelegate::CreateStatic(&RunEventTest));
} // namespace
