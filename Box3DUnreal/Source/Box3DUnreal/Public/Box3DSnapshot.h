// Author: Antonio Lattanzio - emptyvessel

#pragma once

#include "CoreMinimal.h"
#include <box3d/box3d.h>

namespace Box3D
{
	/** One dynamic body's restorable state, in box3d space. */
	struct FBodyState
	{
		b3WorldTransform Transform{};
		b3Vec3 LinearVelocity{};
		b3Vec3 AngularVelocity{};
		bool bAwake = false;
	};

	FORCEINLINE FBodyState CaptureBodyState(b3BodyId Body)
	{
		FBodyState State;
		State.Transform = b3Body_GetTransform(Body);
		State.LinearVelocity = b3Body_GetLinearVelocity(Body);
		State.AngularVelocity = b3Body_GetAngularVelocity(Body);
		State.bAwake = b3Body_IsAwake(Body);
		return State;
	}

	FORCEINLINE void RestoreBodyState(b3BodyId Body, const FBodyState& State)
	{
		b3Body_SetTransform(Body, State.Transform.p, State.Transform.q);
		b3Body_SetLinearVelocity(Body, State.LinearVelocity);
		b3Body_SetAngularVelocity(Body, State.AngularVelocity);
		b3Body_SetAwake(Body, State.bAwake);
	}

	/** Fold one body's state into a running djb2 hash (b3Hash). Feed states in a stable order so
	 *  the same world produces the same digest across runs / peers. Start from B3_HASH_INIT. */
	FORCEINLINE uint32 HashBodyState(uint32 Hash, const FBodyState& State)
	{
		Hash = b3Hash(Hash, reinterpret_cast<const uint8_t*>(&State.Transform), sizeof(State.Transform));
		Hash = b3Hash(Hash, reinterpret_cast<const uint8_t*>(&State.LinearVelocity), sizeof(State.LinearVelocity));
		Hash = b3Hash(Hash, reinterpret_cast<const uint8_t*>(&State.AngularVelocity), sizeof(State.AngularVelocity));
		return Hash;
	}

	FORCEINLINE uint32 HashBody(uint32 Hash, b3BodyId Body)
	{
		const FBodyState State = CaptureBodyState(Body);
		return HashBodyState(Hash, State);
	}
}
