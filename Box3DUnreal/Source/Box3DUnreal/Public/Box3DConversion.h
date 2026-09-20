// Author: Antonio Lattanzio - emptyvessel

#pragma once

#include "CoreMinimal.h"
#include <box3d/box3d.h>

namespace Box3D
{
	/** cm -> box3d length units. 0.01 in Meters mode, 1.0 in Centimeters mode. */
	extern BOX3DUNREAL_API double UnrealToMeters;

	/** box3d length units -> cm. The reciprocal of the above. */
	extern BOX3DUNREAL_API double MetersToUnreal;

	/** Resolve box3d.LengthUnits and push it into box3d. Called before world creation;
	 *  a later cvar change only takes effect on the next world. Returns 1 or 100. */
	BOX3DUNREAL_API float InitializeLengthUnits();

	BOX3DUNREAL_API bool IsCentimeterMode();

	/** kg/m^3 -> box3d's kg per cubic length unit. Miss this in centimeter mode and
	 *  bodies come out 1e6 too heavy. */
	FORCEINLINE float ToBox3DDensity(float DensityKgPerCubicMeter)
	{
		const double LengthUnits = IsCentimeterMode() ? 100.0 : 1.0;
		return static_cast<float>(DensityKgPerCubicMeter / (LengthUnits * LengthUnits * LengthUnits));
	}

	/** Pin the length unit for a scope. For tests, which build worlds in fixed units while
	 *  a live world may be using the other mode. */
	struct BOX3DUNREAL_API FScopedLengthUnits
	{
		explicit FScopedLengthUnits(float UnitsPerMeter)
			: Previous(b3GetLengthUnitsPerMeter())
		{
			b3SetLengthUnitsPerMeter(UnitsPerMeter);
		}

		~FScopedLengthUnits() { b3SetLengthUnitsPerMeter(Previous); }

		FScopedLengthUnits(const FScopedLengthUnits&) = delete;
		FScopedLengthUnits& operator=(const FScopedLengthUnits&) = delete;

	private:
		float Previous;
	};

	/** World position (double translation). Use for body positions. */
	FORCEINLINE b3Pos ToBox3DPosition(const FVector& V)
	{
		return b3Pos{ V.X * UnrealToMeters, -V.Y * UnrealToMeters, V.Z * UnrealToMeters };
	}

	FORCEINLINE FVector FromBox3DPosition(const b3Pos& P)
	{
		return FVector(P.x * MetersToUnreal, -P.y * MetersToUnreal, P.z * MetersToUnreal);
	}

	/** Direction / velocity / gravity (float). No origin offset, same linear map. */
	FORCEINLINE b3Vec3 ToBox3DVector(const FVector& V)
	{
		return b3Vec3{
			static_cast<float>(V.X * UnrealToMeters),
			static_cast<float>(-V.Y * UnrealToMeters),
			static_cast<float>(V.Z * UnrealToMeters) };
	}

	FORCEINLINE FVector FromBox3DVector(const b3Vec3& V)
	{
		return FVector(V.x * MetersToUnreal, -V.y * MetersToUnreal, V.z * MetersToUnreal);
	}

	/** Unit direction / surface normal: handedness only, never the cm<->m scale (scaling a
	 *  normal by 100 is the easy mistake - it stays unit length here). */
	FORCEINLINE b3Vec3 ToBox3DDirection(const FVector& V)
	{
		return b3Vec3{ static_cast<float>(V.X), static_cast<float>(-V.Y), static_cast<float>(V.Z) };
	}

	FORCEINLINE FVector FromBox3DDirection(const b3Vec3& V)
	{
		return FVector(V.x, -V.y, V.z);
	}

	/** Angular velocity / torque / angular impulse. These are pseudovectors, so the Y-negation
	 *  reflection maps them like the quaternion's vector part: negate X and Z, not Y. No
	 *  cm<->m scale here - apply any unit factor at the call site. */
	FORCEINLINE b3Vec3 ToBox3DAngular(const FVector& V)
	{
		return b3Vec3{ static_cast<float>(-V.X), static_cast<float>(V.Y), static_cast<float>(-V.Z) };
	}

	FORCEINLINE FVector FromBox3DAngular(const b3Vec3& V)
	{
		return FVector(-V.x, V.y, -V.z);
	}

	FORCEINLINE b3Quat ToBox3DQuat(const FQuat& Q)
	{
		return b3Quat{
			b3Vec3{ static_cast<float>(-Q.X), static_cast<float>(Q.Y), static_cast<float>(-Q.Z) },
			static_cast<float>(Q.W) };
	}

	FORCEINLINE FQuat FromBox3DQuat(const b3Quat& Q)
	{
		return FQuat(-Q.v.x, Q.v.y, -Q.v.z, Q.s);
	}

	/** box3d carries no scale, so this yields a unit-scale transform. */
	FORCEINLINE FTransform FromBox3DTransform(const b3WorldTransform& T)
	{
		return FTransform(FromBox3DQuat(T.q), FromBox3DPosition(T.p));
	}

	/** Joint frames: b3Transform has a float translation, b3WorldTransform a double. */
	FORCEINLINE b3Transform ToBox3DLocalFrame(const FTransform& T)
	{
		return b3Transform{ ToBox3DVector(T.GetLocation()), ToBox3DQuat(T.GetRotation()) };
	}

	FORCEINLINE FTransform FromBox3DLocalFrame(const b3Transform& T)
	{
		return FTransform(FromBox3DQuat(T.q), FromBox3DVector(T.p));
	}
}
