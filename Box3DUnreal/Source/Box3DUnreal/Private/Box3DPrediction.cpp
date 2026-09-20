// Author: Antonio Lattanzio - emptyvessel

#include "Box3DPrediction.h"

namespace Box3D
{
	void FSnapshotRing::Init(int32 InCapacity, int32 InBodyCount)
	{
		Capacity = FMath::Max(1, InCapacity);
		BodyCount = FMath::Max(0, InBodyCount);
		Newest = -1;
		States.SetNum(Capacity * BodyCount);
		FrameAt.Init(-1, Capacity);
	}

	int32 FSnapshotRing::FindSlot(int32 Frame) const
	{
		if (Capacity == 0)
		{
			return INDEX_NONE;
		}
		const int32 Slot = ((Frame % Capacity) + Capacity) % Capacity;
		return (FrameAt.IsValidIndex(Slot) && FrameAt[Slot] == Frame) ? Slot : INDEX_NONE;
	}

	int32 FSnapshotRing::OldestRetainedFrame() const
	{
		if (Newest < 0)
		{
			return -1;
		}
		const int32 Lowest = FMath::Max(0, Newest - Capacity + 1);
		return FindSlot(Lowest) != INDEX_NONE ? Lowest : -1;
	}

	void FSnapshotRing::Capture(int32 Frame, const TArray<b3BodyId>& Bodies)
	{
		if (Capacity == 0 || Bodies.Num() != BodyCount)
		{
			return;
		}
		const int32 Slot = ((Frame % Capacity) + Capacity) % Capacity;
		FrameAt[Slot] = Frame;
		FBodyState* Row = &States[Slot * BodyCount];
		for (int32 i = 0; i < BodyCount; ++i)
		{
			Row[i] = CaptureBodyState(Bodies[i]);
		}
		Newest = FMath::Max(Newest, Frame);
	}

	bool FSnapshotRing::GetStates(int32 Frame, TArray<FBodyState>& OutStates) const
	{
		const int32 Slot = FindSlot(Frame);
		if (Slot == INDEX_NONE)
		{
			return false;
		}
		OutStates.SetNumUninitialized(BodyCount);
		const FBodyState* Row = &States[Slot * BodyCount];
		for (int32 i = 0; i < BodyCount; ++i)
		{
			OutStates[i] = Row[i];
		}
		return true;
	}

	bool FSnapshotRing::Restore(int32 Frame, const TArray<b3BodyId>& Bodies) const
	{
		const int32 Slot = FindSlot(Frame);
		if (Slot == INDEX_NONE || Bodies.Num() != BodyCount)
		{
			return false;
		}
		const FBodyState* Row = &States[Slot * BodyCount];
		for (int32 i = 0; i < BodyCount; ++i)
		{
			RestoreBodyState(Bodies[i], Row[i]);
		}
		return true;
	}

	namespace
	{
		double PositionDelta(const b3Pos& A, const b3Pos& B)
		{
			return FMath::Sqrt((A.x - B.x) * (A.x - B.x) + (A.y - B.y) * (A.y - B.y) + (A.z - B.z) * (A.z - B.z));
		}
	}

	FReconcileResult ReconcileAndReplay(
		b3WorldId World,
		const TArray<b3BodyId>& Bodies,
		FSnapshotRing& Ring,
		int32 AuthFrame,
		const TArray<int32>& AuthIndices,
		const TArray<FBodyState>& AuthStates,
		int32 PresentFrame,
		float TimeStep,
		int32 SubStepCount,
		double PositionTolerance)
	{
		FReconcileResult Result;

		// Can't roll back past the ring window, or forward.
		if (!Ring.IsRetained(AuthFrame) || AuthFrame > PresentFrame || AuthIndices.Num() != AuthStates.Num())
		{
			return Result;
		}

		TArray<FBodyState> Predicted;
		if (!Ring.GetStates(AuthFrame, Predicted))
		{
			return Result;
		}

		double MaxError = 0.0;
		for (int32 k = 0; k < AuthIndices.Num(); ++k)
		{
			const int32 Idx = AuthIndices[k];
			if (Predicted.IsValidIndex(Idx))
			{
				MaxError = FMath::Max(MaxError, PositionDelta(Predicted[Idx].Transform.p, AuthStates[k].Transform.p));
			}
		}
		if (MaxError <= PositionTolerance)
		{
			return Result; // within tolerance: no correction, no pop
		}

		// Roll the whole world back to the authoritative frame (all bodies from the ring)...
		if (!Ring.Restore(AuthFrame, Bodies))
		{
			return Result;
		}
		// ...then stamp the authoritative bodies over their predicted state.
		for (int32 k = 0; k < AuthIndices.Num(); ++k)
		{
			const int32 Idx = AuthIndices[k];
			if (Bodies.IsValidIndex(Idx))
			{
				RestoreBodyState(Bodies[Idx], AuthStates[k]);
			}
		}
		// Overwrite the ring entry at AuthFrame so it reflects the corrected state.
		Ring.Capture(AuthFrame, Bodies);

		for (int32 Frame = AuthFrame + 1; Frame <= PresentFrame; ++Frame)
		{
			b3World_Step(World, TimeStep, SubStepCount);
			Ring.Capture(Frame, Bodies);
			++Result.ReplayedFrames;
		}

		Result.bCorrected = true;
		Result.MaxCorrection = MaxError;
		return Result;
	}
}
