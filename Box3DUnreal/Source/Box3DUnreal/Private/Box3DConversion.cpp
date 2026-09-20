// Author: Antonio Lattanzio - emptyvessel

#include "Box3DConversion.h"
#include "Box3DLog.h"
#include "HAL/IConsoleManager.h"

namespace Box3D
{
	// Meters until InitializeLengthUnits says otherwise.
	double UnrealToMeters = 0.01;
	double MetersToUnreal = 100.0;
}

namespace
{
	TAutoConsoleVariable<float> CVarBox3DLengthUnits(
		TEXT("box3d.LengthUnits"),
		1.0f,
		TEXT("box3d length units per meter. 1 = meters (default), 100 = centimeters. ")
		TEXT("Takes effect on the next world (box3d.Enabled 0/1)."),
		ECVF_Default);
}

float Box3D::InitializeLengthUnits()
{
	const float Requested = CVarBox3DLengthUnits.GetValueOnGameThread();

	float Units = 1.0f;
	if (FMath::IsNearlyEqual(Requested, 100.0f))
	{
		Units = 100.0f;
	}
	else if (!FMath::IsNearlyEqual(Requested, 1.0f))
	{
		UE_LOG(LogBox3D, Warning,
			TEXT("box3d.LengthUnits=%g is not supported (use 1 for meters or 100 for centimeters). "
				 "Falling back to meters."),
			Requested);
	}

	b3SetLengthUnitsPerMeter(Units);

	// In centimeter mode box3d already works in cm, so the boundary scale is 1.
	UnrealToMeters = Units == 100.0f ? 1.0 : 0.01;
	MetersToUnreal = 1.0 / UnrealToMeters;

	UE_LOG(LogBox3D, Log, TEXT("box3d length units: %s (%g units/meter)."),
		Units == 100.0f ? TEXT("centimeters") : TEXT("meters"), Units);

	return Units;
}

bool Box3D::IsCentimeterMode()
{
	return UnrealToMeters == 1.0;
}
