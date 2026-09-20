// Author: Antonio Lattanzio - emptyvessel


#include "Box3DBodyComponent.h"
#include "Box3DCharacterComponent.h"
#include "Box3DConversion.h"
#include "Box3DLog.h"
#include "Box3DStats.h"
#include "Box3DSubsystem.h"
#include "Engine/Level.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "Tasks/Task.h"

namespace
{
	TAutoConsoleVariable<int32> CVarBox3DAsyncStep(
		TEXT("box3d.AsyncStep"),
		1,
		TEXT("Run b3World_Step on a task thread instead of the game thread (1 = on)."),
		ECVF_Default);
}

bool UBox3DSubsystem::IsAsyncStepEnabled()
{
	return CVarBox3DAsyncStep.GetValueOnGameThread() != 0;
}

void UBox3DSubsystem::FlushAsyncStep() const
{
	if (StepTask.IsValid())
	{
		// Wait runs it here if it never got scheduled, else blocks.
		StepTask.Wait();
		StepTask = {};
	}
}

void UBox3DSubsystem::GatherKinematicTargets(TArray<FKinematicTarget>& OutTargets)
{
	OutTargets.Reset(KinematicBodies.Num());

	for (int32 Index = KinematicBodies.Num() - 1; Index >= 0; --Index)
	{
		UBox3DBodyComponent* Body = KinematicBodies[Index].Get();
		if (Body == nullptr)
		{
			KinematicBodies.RemoveAtSwap(Index);
			continue;
		}

		const b3BodyId BodyId = Body->GetBodyId();
		const AActor* Owner = Body->GetOwner();
		if (B3_IS_NULL(BodyId) || Owner == nullptr)
		{
			continue;
		}

		const FTransform T = Owner->GetActorTransform();

		FKinematicTarget Target;
		Target.Body = BodyId;
		Target.Target.p = Box3D::ToBox3DPosition(T.GetLocation());
		Target.Target.q = Box3D::ToBox3DQuat(T.GetRotation());
		OutTargets.Add(Target);
	}
}

void UBox3DSubsystem::ApplyKinematicTargets(const TArray<FKinematicTarget>& Targets, float TimeStep) const
{
	// Target transform, not teleport: kinematic bodies must carry momentum into contacts.
	for (const FKinematicTarget& Target : Targets)
	{
		b3Body_SetTargetTransform(Target.Body, Target.Target, TimeStep, /*wake=*/true);
	}
}

void UBox3DSubsystem::StepWorldOnly(FBox3DFrameProfile& Frame)
{
	b3World_Step(WorldId, FixedTimeStep, SubStepCount);
	Frame.Accumulate(b3World_GetProfile(WorldId));
}

void FBox3DKickTickFunction::ExecuteTick(float DeltaTime, ELevelTick TickType,
	ENamedThreads::Type CurrentThread, const FGraphEventRef& CompletionEvent)
{
	if (Subsystem != nullptr)
	{
		Subsystem->KickAsyncStep(DeltaTime);
	}
}

FString FBox3DKickTickFunction::DiagnosticMessage()
{
	return TEXT("FBox3DKickTickFunction");
}

void FBox3DJoinTickFunction::ExecuteTick(float DeltaTime, ELevelTick TickType,
	ENamedThreads::Type CurrentThread, const FGraphEventRef& CompletionEvent)
{
	if (Subsystem != nullptr)
	{
		Subsystem->JoinAsyncStep();
	}
}

FString FBox3DJoinTickFunction::DiagnosticMessage()
{
	return TEXT("FBox3DJoinTickFunction");
}

void UBox3DSubsystem::RegisterStepTickFunctions()
{
	UWorld* World = GetWorld();
	if (bTickFunctionsRegistered || World == nullptr || World->PersistentLevel == nullptr)
	{
		return;
	}

	KickTick.Subsystem = this;
	KickTick.bCanEverTick = true;
	KickTick.bStartWithTickEnabled = true;
	KickTick.TickGroup = TG_PrePhysics;
	KickTick.EndTickGroup = TG_PrePhysics;
	KickTick.bHighPriority = true;
	KickTick.RegisterTickFunction(World->PersistentLevel);

	JoinTick.Subsystem = this;
	JoinTick.bCanEverTick = true;
	JoinTick.bStartWithTickEnabled = true;
	JoinTick.TickGroup = TG_PostPhysics;
	JoinTick.EndTickGroup = TG_PostPhysics;
	JoinTick.RegisterTickFunction(World->PersistentLevel);

	JoinTick.AddPrerequisite(this, KickTick);

	bTickFunctionsRegistered = true;
}

void UBox3DSubsystem::UnregisterStepTickFunctions()
{
	if (!bTickFunctionsRegistered)
	{
		return;
	}

	FlushAsyncStep();

	JoinTick.RemovePrerequisite(this, KickTick);
	KickTick.UnRegisterTickFunction();
	JoinTick.UnRegisterTickFunction();
	KickTick.Subsystem = nullptr;
	JoinTick.Subsystem = nullptr;
	bTickFunctionsRegistered = false;
}

void UBox3DSubsystem::KickAsyncStep(float DeltaTime)
{
	if (!bWorldValid || !IsAsyncStepEnabled())
	{
		return; // synchronous mode steps from Tick instead
	}

	Accumulator = FMath::Min(Accumulator + DeltaTime, static_cast<double>(MaxFrameTime));

	if (Accumulator < FixedTimeStep)
	{
		AsyncStepCount = 0;
		return; // not enough time banked for a step this frame
	}

	AsyncStepCount = 1;

	TArray<FKinematicTarget> Targets;
	GatherKinematicTargets(Targets);

	AsyncFrame = FBox3DFrameProfile();

	StepTask = UE::Tasks::Launch(UE_SOURCE_LOCATION,
		[this, Targets = MoveTemp(Targets)]()
		{
			ApplyKinematicTargets(Targets, FixedTimeStep);
			StepWorldOnly(AsyncFrame);
		});
}

void UBox3DSubsystem::JoinAsyncStep()
{
	if (!bWorldValid)
	{
		return;
	}

	FlushAsyncStep();

	if (AsyncStepCount == 0)
	{
		return;
	}

	// Catch-up steps run inline, one at a time, so each drain sees its own events.
	AsyncStepCount = 0;
	FinishStepGameThread();

	while (Accumulator >= FixedTimeStep)
	{
		TArray<FKinematicTarget> Targets;
		GatherKinematicTargets(Targets);
		ApplyKinematicTargets(Targets, FixedTimeStep);

		StepWorldOnly(AsyncFrame);
		FinishStepGameThread();
	}

	PublishStats(AsyncFrame);
	ApplyRenderInterpolation();

	DispatchPendingEvents();
}
