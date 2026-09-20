// Author: Antonio Lattanzio - emptyvessel


#include "Box3DBodyComponent.h"
#include "Box3DLog.h"
#include "Box3DSubsystem.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Engine/StaticMeshActor.h"
#include "Engine/World.h"
#include "HAL/IConsoleManager.h"

namespace
{
	// /Engine/BasicShapes/Cube is 100cm, so unscaled half extents are 50cm.
	constexpr double TestCubeHalfSize = 50.0;
	constexpr double TestRayOffset = 500.0;

	// Far above any level geometry, so only the test cube can answer these queries.
	const FVector TestCentre(0.0, 0.0, 50000.0);

	struct FQueryTestTally
	{
		int32 Passed = 0;
		int32 Failed = 0;
	};

	void Check(FQueryTestTally& Tally, bool bCondition, const FString& What)
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

	AActor* SpawnTestCube(UWorld* World)
	{
		UStaticMesh* Cube = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cube.Cube"));
		if (Cube == nullptr)
		{
			return nullptr;
		}

		const FTransform Xform(FRotator::ZeroRotator, TestCentre, FVector::OneVector);
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

		// Static body: mirrors the cube's cooked collision, the same path level geometry takes.
		UBox3DBodyComponent* Body = NewObject<UBox3DBodyComponent>(Actor);
		Body->BodyType = EBox3DBodyType::Static;
		Body->RegisterComponent();

		Actor->FinishSpawning(Xform);
		return Actor;
	}

	void RunQueryTest(UWorld* World)
	{
		UBox3DSubsystem* Subsystem = World ? World->GetSubsystem<UBox3DSubsystem>() : nullptr;
		if (Subsystem == nullptr || !Subsystem->IsWorldValid())
		{
			UE_LOG(LogBox3D, Warning, TEXT("box3d.QueryTest: no box3d world (disabled, or a client)."));
			return;
		}

		AActor* Cube = SpawnTestCube(World);
		if (Cube == nullptr)
		{
			UE_LOG(LogBox3D, Warning, TEXT("box3d.QueryTest: could not spawn the test cube."));
			return;
		}

		FQueryTestTally Tally;
		const FBox3DQueryFilter Filter;

		const TPair<FVector, const TCHAR*> Axes[] = {
			{ FVector(1.0, 0.0, 0.0), TEXT("+X") },
			{ FVector(0.0, 1.0, 0.0), TEXT("+Y") },
			{ FVector(0.0, 0.0, 1.0), TEXT("+Z") },
		};

		for (const TPair<FVector, const TCHAR*>& Axis : Axes)
		{
			const FVector Dir = Axis.Key;
			const TCHAR* Name = Axis.Value;
			const FVector Start = TestCentre - Dir * TestRayOffset;
			const FVector End = TestCentre + Dir * TestRayOffset;

			FBox3DHitResult Hit;
			if (!Subsystem->RaycastClosest(Start, End, Filter, Hit))
			{
				Check(Tally, false, FString::Printf(TEXT("%s ray hits the cube"), Name));
				continue;
			}
			Check(Tally, true, FString::Printf(TEXT("%s ray hits the cube"), Name));

			const FVector ExpectedPoint = TestCentre - Dir * TestCubeHalfSize;
			Check(Tally, FVector::Dist(Hit.Location, ExpectedPoint) < 1.0,
				FString::Printf(TEXT("%s hit point %s ~= expected %s"), Name, *Hit.Location.ToCompactString(),
					*ExpectedPoint.ToCompactString()));

			Check(Tally, FVector::DotProduct(Hit.Normal, -Dir) > 0.99,
				FString::Printf(TEXT("%s normal %s faces the ray"), Name, *Hit.Normal.ToCompactString()));

			Check(Tally, FMath::Abs(Hit.Normal.Size() - 1.0) < 0.01,
				FString::Printf(TEXT("%s normal is unit length (%.3f)"), Name, Hit.Normal.Size()));

			const double ExpectedDistance = TestRayOffset - TestCubeHalfSize;
			Check(Tally, FMath::Abs(Hit.Distance - ExpectedDistance) < 1.0,
				FString::Printf(TEXT("%s distance %.1f ~= expected %.1f"), Name, Hit.Distance, ExpectedDistance));

			Check(Tally, Hit.HitActor == Cube, FString::Printf(TEXT("%s hit resolves to the test actor"), Name));
		}

		TArray<AActor*> Overlapped;
		Subsystem->OverlapSphere(TestCentre, 100.0f, Filter, Overlapped);
		Check(Tally, Overlapped.Contains(Cube), TEXT("OverlapSphere finds the cube"));

		TArray<AActor*> Distant;
		Subsystem->OverlapSphere(TestCentre + FVector(0.0, 0.0, 1000.0), 100.0f, Filter, Distant);
		Check(Tally, !Distant.Contains(Cube), TEXT("OverlapSphere away from the cube finds nothing"));

		FBox3DHitResult SweepHit;
		const FVector SweepStart = TestCentre - FVector(TestRayOffset, 0.0, 0.0);
		Check(Tally, Subsystem->SphereCast(SweepStart, TestCentre, 25.0f, Filter, SweepHit)
					&& SweepHit.HitActor == Cube,
			TEXT("SphereCast hits the cube"));

		Cube->Destroy();

		UE_LOG(LogBox3D, Log, TEXT("box3d.QueryTest: %d passed, %d failed."), Tally.Passed, Tally.Failed);
	}

	FAutoConsoleCommandWithWorld GBox3DQueryTestCommand(
		TEXT("box3d.QueryTest"),
		TEXT("Self-checking query smoke test: raycast/overlap/shape-cast a known cube and verify the hits."),
		FConsoleCommandWithWorldDelegate::CreateStatic(&RunQueryTest));
} // namespace
