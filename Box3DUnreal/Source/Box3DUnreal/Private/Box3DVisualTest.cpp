// Author: Antonio Lattanzio - emptyvessel


#include "Box3DBodyComponent.h"
#include "Box3DJointDemoActor.h"
#include "Box3DJoints.h"
#include "Box3DLog.h"
#include "Box3DRagdollComponent.h"
#include "Box3DSubsystem.h"
#include "Components/SkeletalMeshComponent.h"
#include "Components/StaticMeshComponent.h"
#include "DrawDebugHelpers.h"
#include "Engine/SkeletalMesh.h"
#include "Engine/StaticMesh.h"
#include "Engine/StaticMeshActor.h"
#include "Engine/World.h"
#include "GameFramework/Pawn.h"
#include "GameFramework/PlayerController.h"
#include "HAL/IConsoleManager.h"
#include "PhysicsEngine/PhysicsAsset.h"
#include "PhysicsEngine/SkeletalBodySetup.h"
#include "TimerManager.h"
#include "UObject/UObjectIterator.h"

namespace
{
	constexpr float VerdictDrawSeconds = 20.0f;

	// Wall time, not frames - the live world steps itself, so checks go on a timer.
	constexpr float SettleSeconds = 4.0f;

	TAutoConsoleVariable<FString> CVarRagdollMesh(
		TEXT("box3d.RagdollMesh"),
		TEXT(""),
		TEXT("Skeletal mesh asset path box3d.RagdollTest spawns (needs a PhysicsAsset).\n")
		TEXT("Empty tries the Lyra/Mannequin defaults."),
		ECVF_Default);

	// Tried in order when the cvar is unset; each is checked for a PhysicsAsset.
	const TCHAR* FallbackRagdollMeshes[] = {
		TEXT("/Game/Characters/Heroes/Mannequin/Meshes/SK_Mannequin.SK_Mannequin"),
		TEXT("/Game/Characters/Heroes/Mannequin/Meshes/SKM_Quinn.SKM_Quinn"),
		TEXT("/Game/Characters/Heroes/Mannequin/Meshes/SKM_Manny.SKM_Manny"),
		TEXT("/Engine/EngineMeshes/SkeletalCube.SkeletalCube"),
	};

	// --- Shared plumbing ---

	bool GetPlayerView(UWorld* World, FVector& OutLocation, FRotator& OutRotation)
	{
		APlayerController* PC = World ? World->GetFirstPlayerController() : nullptr;
		if (PC == nullptr)
		{
			return false;
		}
		if (const APawn* Pawn = PC->GetPawn())
		{
			OutLocation = Pawn->GetActorLocation();
			OutRotation = PC->GetControlRotation();
			return true;
		}
		FVector CamLoc;
		FRotator CamRot;
		PC->GetPlayerViewPoint(CamLoc, CamRot);
		OutLocation = CamLoc;
		OutRotation = CamRot;
		return true;
	}

	// Flattened so it does not aim into the floor or sky.
	bool GetStageLocation(UWorld* World, double Forward, double Up, FVector& OutLocation)
	{
		FVector RefLocation;
		FRotator RefRotation;
		if (!GetPlayerView(World, RefLocation, RefRotation))
		{
			return false;
		}
		FVector Fwd = RefRotation.Vector();
		Fwd.Z = 0.0;
		Fwd = Fwd.GetSafeNormal();
		OutLocation = RefLocation + Fwd * Forward + FVector(0.0, 0.0, Up);
		return true;
	}

	UBox3DSubsystem* GetLiveSubsystem(UWorld* World, const TCHAR* Command)
	{
		UBox3DSubsystem* Subsystem = World ? World->GetSubsystem<UBox3DSubsystem>() : nullptr;
		if (Subsystem == nullptr || !Subsystem->IsWorldValid())
		{
			UE_LOG(LogBox3D, Warning,
				TEXT("%s: no box3d world. Set 'box3d.Enabled 1', and run on the server/standalone."),
				Command);
			return nullptr;
		}
		return Subsystem;
	}

	void EnableJointVisualization()
	{
		if (IConsoleVariable* Native = IConsoleManager::Get().FindConsoleVariable(TEXT("box3d.NativeDraw")))
		{
			// shapes | joints | jointExtras
			Native->Set(7, ECVF_SetByConsole);
		}
	}

	// Verdicts drawn at the rig, so a failure shows next to what failed.
	struct FVisualTally
	{
		UWorld* World = nullptr;
		FVector Anchor = FVector::ZeroVector;
		int32 Passed = 0;
		int32 Failed = 0;
		int32 Line = 0;

		void Check(bool bCondition, const FString& What)
		{
			bCondition ? ++Passed : ++Failed;
			UE_LOG(LogBox3D, Display, TEXT("  %s  %s"), bCondition ? TEXT("PASS") : TEXT("FAIL"), *What);

			if (World != nullptr)
			{
				const FVector At = Anchor + FVector(0.0, 0.0, -Line * 18.0);
				DrawDebugString(World, At, FString::Printf(TEXT("%s %s"),
					bCondition ? TEXT("PASS") : TEXT("FAIL"), *What),
					nullptr, bCondition ? FColor::Green : FColor::Red, VerdictDrawSeconds, true);
			}
			++Line;
		}

		void CheckNear(double Value, double Expected, double Tolerance, const FString& What)
		{
			Check(FMath::Abs(Value - Expected) <= Tolerance,
				FString::Printf(TEXT("%s (%.2f vs %.2f)"), *What, Value, Expected));
		}

		void Report(const TCHAR* Name)
		{
			if (Failed == 0)
			{
				UE_LOG(LogBox3D, Display, TEXT("%s: PASS - %d checks."), Name, Passed);
			}
			else
			{
				UE_LOG(LogBox3D, Error, TEXT("%s: FAIL - %d passed, %d failed."), Name, Passed, Failed);
			}

			if (World != nullptr)
			{
				DrawDebugString(World, Anchor + FVector(0.0, 0.0, 60.0),
					FString::Printf(TEXT("%s: %d passed, %d failed"), Name, Passed, Failed),
					nullptr, Failed == 0 ? FColor::Green : FColor::Red, VerdictDrawSeconds, true);
			}
		}
	};

	UBox3DBodyComponent* SpawnVisibleBody(UWorld* World, const TCHAR* MeshPath, const FVector& Location,
		const FVector& HalfExtent, bool bStatic, EBox3DShape Shape, float Restitution = 0.0f)
	{
		UStaticMesh* Mesh = LoadObject<UStaticMesh>(nullptr, MeshPath);
		if (Mesh == nullptr)
		{
			return nullptr;
		}

		// Engine primitives are 100cm.
		const FTransform Xform(FRotator::ZeroRotator, Location, HalfExtent / 50.0);
		AStaticMeshActor* Actor = World->SpawnActorDeferred<AStaticMeshActor>(
			AStaticMeshActor::StaticClass(), Xform, nullptr, nullptr,
			ESpawnActorCollisionHandlingMethod::AlwaysSpawn);
		if (Actor == nullptr)
		{
			return nullptr;
		}

		UStaticMeshComponent* MeshComp = Actor->GetStaticMeshComponent();
		MeshComp->SetMobility(EComponentMobility::Movable);
		MeshComp->SetStaticMesh(Mesh);
		MeshComp->SetCollisionEnabled(ECollisionEnabled::NoCollision); // box3d owns collision

		UBox3DBodyComponent* Body = NewObject<UBox3DBodyComponent>(Actor);
		Body->BodyType = bStatic ? EBox3DBodyType::Static : EBox3DBodyType::Dynamic;
		Body->Shape = Shape;
		if (Shape == EBox3DShape::Sphere)
		{
			Body->Radius = static_cast<float>(HalfExtent.X);
		}
		else
		{
			Body->BoxHalfExtent = HalfExtent;
		}
		Body->Restitution = Restitution;
		Body->RegisterComponent();

		Actor->FinishSpawning(Xform);
		return Body;
	}

	UBox3DBodyComponent* SpawnCube(UWorld* World, const FVector& Location, const FVector& HalfExtent,
		bool bStatic, float Restitution = 0.0f)
	{
		return SpawnVisibleBody(World, TEXT("/Engine/BasicShapes/Cube.Cube"), Location, HalfExtent,
			bStatic, EBox3DShape::Box, Restitution);
	}

	void CheckAfterSettling(UWorld* World, TFunction<void()> Verify, float Delay = SettleSeconds)
	{
		FTimerHandle Handle;
		World->GetTimerManager().SetTimer(Handle, FTimerDelegate::CreateLambda(MoveTemp(Verify)),
			Delay, false);
	}


	void RunSphericalCase(UWorld* World, const FVector& Origin)
	{
		const double ArmLength = 150.0;

		UBox3DBodyComponent* Anchor = SpawnCube(World, Origin, FVector(15.0), true);
		UBox3DBodyComponent* Hanging = SpawnCube(World, Origin + FVector(ArmLength, 0.0, 0.0), FVector(20.0), false);
		if (Anchor == nullptr || Hanging == nullptr)
		{
			return;
		}

		FBox3DJointSettings Settings;
		FBox3DSphericalJointSettings Spherical;
		Spherical.bEnableConeLimit = false;

		UBox3DJointLibrary::CreateSphericalJoint(Anchor, Hanging, Origin, FVector::UpVector,
			Settings, Spherical);

		// Weak: these lambdas run seconds later.
		const TWeakObjectPtr<UBox3DBodyComponent> WeakAnchor(Anchor);
		const TWeakObjectPtr<UBox3DBodyComponent> WeakHanging(Hanging);

		CheckAfterSettling(World, [World, Origin, WeakAnchor, WeakHanging, ArmLength]()
		{
			FVisualTally Tally{ World, Origin + FVector(0.0, 0.0, 120.0) };
			if (!WeakAnchor.IsValid() || !WeakHanging.IsValid())
			{
				return;
			}

			const FVector AnchorPos = WeakAnchor->GetOwner()->GetActorLocation();
			const FVector HangPos = WeakHanging->GetOwner()->GetActorLocation();

			// Loose tolerance: this reads the interpolated actor pose, not the solver's.
			Tally.CheckNear(FVector::Dist(AnchorPos, HangPos), ArmLength, 5.0,
				TEXT("ball socket holds the arm length"));

			Tally.Check(HangPos.Z < AnchorPos.Z - ArmLength * 0.5,
				TEXT("unlimited ball socket swings down"));

			DrawDebugLine(World, AnchorPos, HangPos, FColor::Cyan, false, VerdictDrawSeconds, 0, 2.0f);
			Tally.Report(TEXT("spherical"));
		});
	}

	void RunRevoluteCase(UWorld* World, const FVector& Origin)
	{
		const double ArmLength = 150.0;

		UBox3DBodyComponent* Anchor = SpawnCube(World, Origin, FVector(15.0), true);
		UBox3DBodyComponent* Arm = SpawnCube(World, Origin + FVector(ArmLength, 0.0, 0.0),
			FVector(ArmLength * 0.5, 15.0, 15.0), false);
		if (Anchor == nullptr || Arm == nullptr)
		{
			return;
		}

		FBox3DJointSettings Settings;

		// About Z so gravity does not fight the motor.
		FBox3DRevoluteJointSettings Revolute;
		Revolute.bEnableMotor = true;
		Revolute.MotorSpeed = 90.0f;
		Revolute.MaxMotorTorque = 5000.0f;

		UBox3DJointLibrary::CreateRevoluteJoint(Anchor, Arm, Origin, FVector::UpVector,
			Settings, Revolute);

		const TWeakObjectPtr<UBox3DBodyComponent> WeakAnchor(Anchor);
		const TWeakObjectPtr<UBox3DBodyComponent> WeakArm(Arm);

		CheckAfterSettling(World, [World, Origin, WeakAnchor, WeakArm, ArmLength]()
		{
			FVisualTally Tally{ World, Origin + FVector(0.0, 0.0, 120.0) };
			if (!WeakAnchor.IsValid() || !WeakArm.IsValid())
			{
				return;
			}

			const FVector AnchorPos = WeakAnchor->GetOwner()->GetActorLocation();
			const FVector ArmPos = WeakArm->GetOwner()->GetActorLocation();

			Tally.CheckNear(FVector::Dist(AnchorPos, ArmPos), ArmLength, 5.0,
				TEXT("hinge holds the arm radius"));

			Tally.CheckNear(ArmPos.Z, AnchorPos.Z, 10.0, TEXT("hinge about Z keeps the arm level"));

			Tally.Check(!WeakArm->GetAngularVelocity().IsNearlyZero(),
				TEXT("motor is driving the arm"));

			DrawDebugLine(World, AnchorPos, ArmPos, FColor::Cyan, false, VerdictDrawSeconds, 0, 2.0f);
			Tally.Report(TEXT("revolute motor"));
		});
	}

	void RunPrismaticCase(UWorld* World, const FVector& Origin)
	{
		UBox3DBodyComponent* Anchor = SpawnCube(World, Origin, FVector(20.0), true);
		UBox3DBodyComponent* Slider = SpawnCube(World, Origin, FVector(30.0), false);
		if (Anchor == nullptr || Slider == nullptr)
		{
			return;
		}

		FBox3DJointSettings Settings;
		FBox3DPrismaticJointSettings Prismatic;
		Prismatic.bEnableLimit = true;
		Prismatic.LowerTranslation = -100.0f;
		Prismatic.UpperTranslation = 100.0f;

		// Along Z, so gravity parks it on the lower stop.
		UBox3DJointLibrary::CreatePrismaticJoint(Anchor, Slider, Origin, FVector::UpVector,
			Settings, Prismatic);

		const TWeakObjectPtr<UBox3DBodyComponent> WeakSlider(Slider);

		CheckAfterSettling(World, [World, Origin, WeakSlider]()
		{
			FVisualTally Tally{ World, Origin + FVector(0.0, 0.0, 150.0) };
			if (!WeakSlider.IsValid())
			{
				return;
			}

			const FVector SliderPos = WeakSlider->GetOwner()->GetActorLocation();

			Tally.CheckNear(SliderPos.X, Origin.X, 5.0, TEXT("slider does not drift in X"));
			Tally.CheckNear(SliderPos.Y, Origin.Y, 5.0, TEXT("slider does not drift in Y"));

			Tally.CheckNear(SliderPos.Z, Origin.Z - 100.0, 10.0,
				TEXT("slider settles on its lower stop"));

			DrawDebugLine(World, Origin + FVector(0, 0, 100), Origin - FVector(0, 0, 100),
				FColor::Yellow, false, VerdictDrawSeconds, 0, 2.0f);
			Tally.Report(TEXT("prismatic"));
		});
	}

	void RunWeldCase(UWorld* World, const FVector& Origin)
	{
		const FVector WeldedStart = Origin + FVector(120.0, 0.0, 0.0);

		UBox3DBodyComponent* Anchor = SpawnCube(World, Origin, FVector(20.0), true);
		UBox3DBodyComponent* Welded = SpawnCube(World, WeldedStart, FVector(30.0), false);
		if (Anchor == nullptr || Welded == nullptr)
		{
			return;
		}

		FBox3DJointSettings Settings;
		FBox3DWeldJointSettings Weld; // hertz 0 = rigid

		UBox3DJointLibrary::CreateWeldJoint(Anchor, Welded, WeldedStart, Settings, Weld);

		const TWeakObjectPtr<UBox3DBodyComponent> WeakWelded(Welded);

		CheckAfterSettling(World, [World, Origin, WeakWelded, WeldedStart]()
		{
			FVisualTally Tally{ World, Origin + FVector(0.0, 0.0, 120.0) };
			if (!WeakWelded.IsValid())
			{
				return;
			}

			// Any sag at all is the weld leaking.
			const FVector Now = WeakWelded->GetOwner()->GetActorLocation();
			Tally.CheckNear(FVector::Dist(Now, WeldedStart), 0.0, 3.0,
				TEXT("rigid weld holds position under gravity"));

			Tally.Report(TEXT("weld"));
		});
	}

	// Namespace scope so the verify lambda need not capture them.
	constexpr int32 ChainLinkCount = 10;
	constexpr double ChainLinkSpacing = 60.0;

	void RunChainCase(UWorld* World, const FVector& Origin)
	{
		constexpr int32 LinkCount = ChainLinkCount;
		constexpr double LinkSpacing = ChainLinkSpacing;

		FBox3DJointSettings Settings;
		FBox3DSphericalJointSettings Spherical;
		Spherical.bEnableConeLimit = true;
		Spherical.ConeAngle = 60.0f;

		UBox3DBodyComponent* Previous = SpawnCube(World, Origin, FVector(20.0), true);
		if (Previous == nullptr)
		{
			return;
		}

		TArray<TWeakObjectPtr<UBox3DBodyComponent>> Links;
		for (int32 Index = 0; Index < LinkCount; ++Index)
		{
			const FVector Location = Origin - FVector(0.0, 0.0, LinkSpacing * (Index + 1));
			UBox3DBodyComponent* Link = SpawnCube(World, Location, FVector(12.0, 12.0, LinkSpacing * 0.35), false);
			if (Link == nullptr)
			{
				break;
			}

			const FVector Anchor = Location + FVector(0.0, 0.0, LinkSpacing * 0.5);
			UBox3DJointLibrary::CreateSphericalJoint(Previous, Link, Anchor, FVector::UpVector,
				Settings, Spherical);

			Links.Add(Link);
			Previous = Link;
		}

		// Kick it so it swings; a chain at rest exercises little.
		if (Links.Num() > 0)
		{
			Links.Last()->SetLinearVelocity(FVector(300.0, 0.0, 0.0));
		}

		CheckAfterSettling(World, [World, Origin, Links]()
		{
			FVisualTally Tally{ World, Origin + FVector(0.0, 0.0, 120.0) };

			const double MaxReach = ChainLinkSpacing * Links.Num() + ChainLinkSpacing;
			double Worst = 0.0;
			bool bFinite = true;
			for (const TWeakObjectPtr<UBox3DBodyComponent>& Link : Links)
			{
				if (!Link.IsValid())
				{
					continue;
				}
				const FVector P = Link->GetOwner()->GetActorLocation();
				bFinite &= P.ContainsNaN() == false;
				Worst = FMath::Max(Worst, FVector::Dist(P, Origin));
			}

			Tally.Check(bFinite, TEXT("no link went non-finite"));
			Tally.Check(Worst <= MaxReach,
				FString::Printf(TEXT("no link stretches past the chain's reach (%.0f <= %.0f)"),
					Worst, MaxReach));
			Tally.Check(Links.Num() > 0 && Links.Last().IsValid()
					&& Links.Last()->GetOwner()->GetActorLocation().Z < Origin.Z,
				TEXT("the chain hangs below its anchor"));

			Tally.Report(TEXT("chain"));
		}, SettleSeconds * 2.0f); // a chain needs longer to stop swinging
	}

	void RunJointTestVisual(const TArray<FString>& Args, UWorld* World)
	{
		if (GetLiveSubsystem(World, TEXT("box3d.JointTestVisual")) == nullptr)
		{
			return;
		}

		FVector Stage;
		if (!GetStageLocation(World, 600.0, 400.0, Stage))
		{
			UE_LOG(LogBox3D, Warning, TEXT("box3d.JointTestVisual: no player to stage in front of."));
			return;
		}

		EnableJointVisualization();

		const FString Which = Args.Num() > 0 ? Args[0].ToLower() : TEXT("all");
		const bool bAll = Which == TEXT("all");

		// Spread along the player's right so the rigs do not tangle.
		FRotator ViewRotation;
		FVector Ignored;
		GetPlayerView(World, Ignored, ViewRotation);
		const FVector Right = FRotator(0.0, ViewRotation.Yaw, 0.0).Quaternion().GetRightVector();

		int32 Slot = 0;
		auto NextStage = [&Stage, &Right, &Slot]() { return Stage + Right * (Slot++ * 400.0); };

		if (bAll || Which == TEXT("spherical")) { RunSphericalCase(World, NextStage()); }
		if (bAll || Which == TEXT("revolute"))  { RunRevoluteCase(World, NextStage()); }
		if (bAll || Which == TEXT("prismatic")) { RunPrismaticCase(World, NextStage()); }
		if (bAll || Which == TEXT("weld"))      { RunWeldCase(World, NextStage()); }
		if (bAll || Which == TEXT("chain"))     { RunChainCase(World, NextStage()); }

		if (Slot == 0)
		{
			UE_LOG(LogBox3D, Warning,
				TEXT("box3d.JointTestVisual: unknown case '%s'. Use chain|spherical|revolute|prismatic|weld|all."),
				*Which);
			return;
		}

		UE_LOG(LogBox3D, Display,
			TEXT("box3d.JointTestVisual: staged %d rig(s); verdicts in ~%.0fs. box3d.NativeDraw is on."),
			Slot, SettleSeconds);
	}

	FAutoConsoleCommandWithWorldAndArgs GBox3DJointTestVisual(
		TEXT("box3d.JointTestVisual"),
		TEXT("Build the joint tests as visible actors in front of the player and verify them ")
		TEXT("in place: chain | spherical | revolute | prismatic | weld | all (default all)."),
		FConsoleCommandWithWorldAndArgsDelegate::CreateStatic(&RunJointTestVisual));


	USkeletalMesh* ResolveRagdollMesh()
	{
		const FString Configured = CVarRagdollMesh.GetValueOnGameThread();
		if (!Configured.IsEmpty())
		{
			USkeletalMesh* Mesh = LoadObject<USkeletalMesh>(nullptr, *Configured);
			if (Mesh == nullptr)
			{
				UE_LOG(LogBox3D, Warning, TEXT("box3d.RagdollTest: could not load '%s'."), *Configured);
				return nullptr;
			}
			if (Mesh->GetPhysicsAsset() == nullptr)
			{
				UE_LOG(LogBox3D, Warning,
					TEXT("box3d.RagdollTest: '%s' has no PhysicsAsset, so it cannot ragdoll."), *Configured);
				return nullptr;
			}
			return Mesh;
		}

		for (const TCHAR* Path : FallbackRagdollMeshes)
		{
			USkeletalMesh* Mesh = LoadObject<USkeletalMesh>(nullptr, Path);
			if (Mesh != nullptr && Mesh->GetPhysicsAsset() != nullptr)
			{
				return Mesh;
			}
		}

		UE_LOG(LogBox3D, Warning,
			TEXT("box3d.RagdollTest: no skeletal mesh with a PhysicsAsset found. ")
			TEXT("Set box3d.RagdollMesh to one of yours."));
		return nullptr;
	}

	UBox3DRagdollComponent* SpawnRagdoll(UWorld* World, USkeletalMesh* Mesh, const FTransform& Xform)
	{
		AActor* Actor = World->SpawnActorDeferred<AActor>(AActor::StaticClass(), Xform, nullptr, nullptr,
			ESpawnActorCollisionHandlingMethod::AlwaysSpawn);
		if (Actor == nullptr)
		{
			return nullptr;
		}

		USkeletalMeshComponent* MeshComp = NewObject<USkeletalMeshComponent>(Actor);
		MeshComp->SetSkeletalMesh(Mesh);
		MeshComp->SetCollisionEnabled(ECollisionEnabled::NoCollision); // box3d owns collision
		Actor->SetRootComponent(MeshComp);
		MeshComp->RegisterComponent();

		// bStartActive before FinishSpawning: StartRagdoll reads the pose at BeginPlay.
		UBox3DRagdollComponent* Ragdoll = NewObject<UBox3DRagdollComponent>(Actor);
		Ragdoll->bStartActive = true;
		Ragdoll->RegisterComponent();

		Actor->FinishSpawning(Xform);
		return Ragdoll;
	}

	void RunRagdollTest(const TArray<FString>& Args, UWorld* World)
	{
		if (GetLiveSubsystem(World, TEXT("box3d.RagdollTest")) == nullptr)
		{
			return;
		}

		USkeletalMesh* Mesh = ResolveRagdollMesh();
		if (Mesh == nullptr)
		{
			return;
		}

		int32 Count = 24;
		if (Args.Num() > 0)
		{
			Count = FMath::Clamp(FCString::Atoi(*Args[0]), 1, 200);
		}

		FVector Stage;
		if (!GetStageLocation(World, 700.0, 500.0, Stage))
		{
			UE_LOG(LogBox3D, Warning, TEXT("box3d.RagdollTest: no player to stage in front of."));
			return;
		}

		EnableJointVisualization();

		struct FDroppedRagdoll
		{
			TWeakObjectPtr<UBox3DRagdollComponent> Ragdoll;
			TWeakObjectPtr<USkeletalMeshComponent> Mesh;
			FName ProbeBone;
			double StartHeight = 0.0;
		};

		FName ProbeBone;
		if (const UPhysicsAsset* Asset = Mesh->GetPhysicsAsset())
		{
			for (const TObjectPtr<USkeletalBodySetup>& Setup : Asset->SkeletalBodySetups)
			{
				if (Setup != nullptr)
				{
					ProbeBone = Setup->BoneName;
					break;
				}
			}
		}

		TArray<FDroppedRagdoll> Ragdolls;
		Ragdolls.Reserve(Count);
		for (int32 Index = 0; Index < Count; ++Index)
		{
			const FVector2D Disk = FMath::RandPointInCircle(200.0f);
			const FVector Location = Stage + FVector(Disk.X, Disk.Y, Index * 120.0);
			const FRotator Rotation(0.0, FMath::FRandRange(0.0f, 360.0f), 0.0);

			if (UBox3DRagdollComponent* Ragdoll = SpawnRagdoll(World, Mesh, FTransform(Rotation, Location)))
			{
				USkeletalMeshComponent* MeshComp = Ragdoll->GetOwner()->FindComponentByClass<USkeletalMeshComponent>();
				const double StartHeight = MeshComp != nullptr && !ProbeBone.IsNone()
					? MeshComp->GetBoneLocation(ProbeBone).Z
					: Location.Z;
				Ragdolls.Add(FDroppedRagdoll{ Ragdoll, MeshComp, ProbeBone, StartHeight });
			}
		}

		UE_LOG(LogBox3D, Display, TEXT("box3d.RagdollTest: spawned %d ragdoll(s) of '%s'."),
			Ragdolls.Num(), *Mesh->GetName());

		const float Settle = SettleSeconds + Count * 0.1f;
		CheckAfterSettling(World, [World, Stage, Ragdolls, Count]()
		{
			FVisualTally Tally{ World, Stage + FVector(0.0, 0.0, 200.0) };

			Tally.Check(Ragdolls.Num() == Count,
				FString::Printf(TEXT("all %d ragdolls spawned"), Count));

			int32 Active = 0;
			int32 Finite = 0;
			int32 Landed = 0;
			for (const FDroppedRagdoll& Dropped : Ragdolls)
			{
				if (!Dropped.Ragdoll.IsValid() || !IsValid(Dropped.Ragdoll->GetOwner()))
				{
					continue;
				}
				if (Dropped.Ragdoll->IsRagdollActive())
				{
					++Active;
				}

				const USkeletalMeshComponent* MeshComp = Dropped.Mesh.Get();
				const FVector P = MeshComp != nullptr && !Dropped.ProbeBone.IsNone()
					? MeshComp->GetBoneLocation(Dropped.ProbeBone)
					: Dropped.Ragdoll->GetOwner()->GetActorLocation();
				if (!P.ContainsNaN())
				{
					++Finite;
				}
				// Fell rather than being flung up by a solver blow-up.
				if (P.Z < Dropped.StartHeight)
				{
					++Landed;
				}
			}

			Tally.Check(Active == Ragdolls.Num(),
				FString::Printf(TEXT("every ragdoll is simulating (%d/%d)"), Active, Ragdolls.Num()));

			// A NaN here usually means a degenerate constraint frame.
			Tally.Check(Finite == Ragdolls.Num(),
				FString::Printf(TEXT("no ragdoll went non-finite (%d/%d)"), Finite, Ragdolls.Num()));

			Tally.Check(Landed == Ragdolls.Num(),
				FString::Printf(TEXT("every ragdoll fell rather than exploding (%d/%d)"),
					Landed, Ragdolls.Num()));

			Tally.Report(TEXT("box3d.RagdollTest"));
		}, Settle);
	}

	FAutoConsoleCommandWithWorldAndArgs GBox3DRagdollTest(
		TEXT("box3d.RagdollTest"),
		TEXT("Drop N box3d ragdolls in front of the player (default 24) and verify they all ")
		TEXT("simulate, stay finite and land. Set the mesh with box3d.RagdollMesh."),
		FConsoleCommandWithWorldAndArgsDelegate::CreateStatic(&RunRagdollTest));

	// Proves the joints carry an impulse through the tree - finds rubbery limbs.
	void RunRagdollPunch(const TArray<FString>& Args, UWorld* World)
	{
		if (GetLiveSubsystem(World, TEXT("box3d.RagdollPunch")) == nullptr)
		{
			return;
		}

		float Strength = 50000.0f;
		if (Args.Num() > 0)
		{
			Strength = FMath::Max(0.0f, FCString::Atof(*Args[0]));
		}

		FVector ViewLocation;
		FRotator ViewRotation;
		if (!GetPlayerView(World, ViewLocation, ViewRotation))
		{
			return;
		}

		int32 Hit = 0;
		for (TObjectIterator<UBox3DRagdollComponent> It; It; ++It)
		{
			UBox3DRagdollComponent* Ragdoll = *It;
			if (!IsValid(Ragdoll) || Ragdoll->GetWorld() != World || !Ragdoll->IsRagdollActive())
			{
				continue;
			}

			const FVector Target = Ragdoll->GetOwner()->GetActorLocation();
			const FVector Direction = (Target - ViewLocation).GetSafeNormal() + FVector(0.0, 0.0, 0.3);
			Ragdoll->AddImpulseAtLocation(Direction.GetSafeNormal() * Strength, Target);
			++Hit;
		}

		UE_LOG(LogBox3D, Display, TEXT("box3d.RagdollPunch: hit %d ragdoll(s) with %.0f."), Hit, Strength);
	}

	FAutoConsoleCommandWithWorldAndArgs GBox3DRagdollPunch(
		TEXT("box3d.RagdollPunch"),
		TEXT("Impulse every active box3d ragdoll away from the player (default 50000). ")
		TEXT("Run after box3d.RagdollTest to see the joints carry the hit."),
		FConsoleCommandWithWorldAndArgsDelegate::CreateStatic(&RunRagdollPunch));
} // namespace
