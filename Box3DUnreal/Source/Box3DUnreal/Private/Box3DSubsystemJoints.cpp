// Author: Antonio Lattanzio - emptyvessel

#include "Box3DJointTypes.h"
#include "Box3DLog.h"
#include "Box3DSubsystem.h"

FBox3DJointHandle UBox3DSubsystem::RegisterJoint(b3JointId Joint, EBox3DJointType Type,
	const FBox3DJointSettings& Settings)
{
	if (B3_IS_NULL(Joint))
	{
		return FBox3DJointHandle();
	}

	const int32 Index = FreeJointSlots.Num() > 0 ? FreeJointSlots.Pop(EAllowShrinking::No)
		: JointSlots.AddDefaulted();

	FJointSlot& Slot = JointSlots[Index];
	Slot.Id = Joint;
	Slot.Type = Type;
	Slot.BreakForce = Settings.BreakForce;
	Slot.BreakTorque = Settings.BreakTorque;
	Slot.bBreakable = Settings.bBreakable && (Settings.BreakForce > 0.0f || Settings.BreakTorque > 0.0f);
	++Slot.Serial; // stale handles to this slot stop resolving

	FBox3DJointHandle Handle;
	Handle.Index = Index;
	Handle.Serial = Slot.Serial;
	return Handle;
}

b3JointId UBox3DSubsystem::ResolveJoint(const FBox3DJointHandle& Handle) const
{
	if (!JointSlots.IsValidIndex(Handle.Index))
	{
		return b3_nullJointId;
	}
	const FJointSlot& Slot = JointSlots[Handle.Index];
	if (Slot.Serial != Handle.Serial || B3_IS_NULL(Slot.Id))
	{
		return b3_nullJointId;
	}

	// Body destruction can invalidate a joint without retiring its handle.
	return b3Joint_IsValid(Slot.Id) ? Slot.Id : b3_nullJointId;
}

void UBox3DSubsystem::DestroyJoint(FBox3DJointHandle Handle, bool bWakeBodies)
{
	if (!JointSlots.IsValidIndex(Handle.Index))
	{
		return;
	}
	FJointSlot& Slot = JointSlots[Handle.Index];
	if (Slot.Serial != Handle.Serial || B3_IS_NULL(Slot.Id))
	{
		return; // already destroyed, or the handle is stale
	}

	if (bWorldValid)
	{
		FlushAsyncStep();

		// The body may already have destroyed this joint.
		if (b3Joint_IsValid(Slot.Id))
		{
			b3DestroyJoint(Slot.Id, bWakeBodies);
		}
	}
	Slot.Id = b3_nullJointId;
	Slot.bBreakable = false;
	++Slot.Serial;
	FreeJointSlots.Add(Handle.Index);
}

void UBox3DSubsystem::UpdateBreakableJoints()
{
	for (int32 Index = 0; Index < JointSlots.Num(); ++Index)
	{
		FJointSlot& Slot = JointSlots[Index];
		if (!Slot.bBreakable || B3_IS_NULL(Slot.Id))
		{
			continue;
		}

		// Reap joints destroyed with their attached body.
		if (!b3Joint_IsValid(Slot.Id))
		{
			Slot.Id = b3_nullJointId;
			Slot.bBreakable = false;
			continue;
		}

		const float Force = b3Length(b3Joint_GetConstraintForce(Slot.Id));
		const float Torque = b3Length(b3Joint_GetConstraintTorque(Slot.Id));

		const bool bOverForce = Slot.BreakForce > 0.0f && Force > Slot.BreakForce;
		const bool bOverTorque = Slot.BreakTorque > 0.0f && Torque > Slot.BreakTorque;
		if (!bOverForce && !bOverTorque)
		{
			continue;
		}

		FPendingJointBreak& Break = PendingJointBreaks.AddDefaulted_GetRef();
		Break.Handle.Index = Index;
		Break.Handle.Serial = Slot.Serial;
		Break.Force = Force;
		Break.Torque = Torque;

		// Destroy now; the delegate waits until after the loop, like every other event.
		b3DestroyJoint(Slot.Id, true);
		Slot.Id = b3_nullJointId;
		Slot.bBreakable = false;
		++Slot.Serial;
		FreeJointSlots.Add(Index);
	}
}

void UBox3DSubsystem::DestroyAllJoints()
{
	// b3DestroyWorld already frees the joints; just drop our bookkeeping.
	JointSlots.Reset();
	FreeJointSlots.Reset();
	PendingJointBreaks.Reset();
}
