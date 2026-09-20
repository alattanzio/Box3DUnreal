// Author: Antonio Lattanzio - emptyvessel

#include "Box3DConversion.h"
#include "Box3DLog.h"
#include "Box3DSubsystem.h"
#include "DrawDebugHelpers.h"
#include "Engine/World.h"
#include "GameFramework/PlayerController.h"

namespace
{
	// Passed through box3d as the void* context on every callback.
	struct FDrawContext
	{
		UWorld* World = nullptr;
		float Thickness = 1.0f;
	};

	FORCEINLINE FColor ToUnrealColor(b3HexColor Hex)
	{
		const uint32 Value = static_cast<uint32>(Hex);
		return FColor((Value >> 16) & 0xFF, (Value >> 8) & 0xFF, Value & 0xFF, 255);
	}

	FORCEINLINE UWorld* ContextWorld(void* Context)
	{
		return static_cast<FDrawContext*>(Context)->World;
	}

	FORCEINLINE float GetThickness(void* Context)
	{
		return static_cast<FDrawContext*>(Context)->Thickness;
	}

	void DrawSegment(b3Pos P1, b3Pos P2, b3HexColor Color, void* Context)
	{
		DrawDebugLine(ContextWorld(Context), Box3D::FromBox3DPosition(P1), Box3D::FromBox3DPosition(P2),
			ToUnrealColor(Color), false, -1.0f, 0, GetThickness(Context));
	}

	void DrawPoint(b3Pos P, float Size, b3HexColor Color, void* Context)
	{
		DrawDebugPoint(ContextWorld(Context), Box3D::FromBox3DPosition(P), Size, ToUnrealColor(Color), false, -1.0f);
	}

	void DrawSphere(b3Pos P, float Radius, b3HexColor Color, float Alpha, void* Context)
	{
		DrawDebugSphere(ContextWorld(Context), Box3D::FromBox3DPosition(P),
			Radius * static_cast<float>(Box3D::MetersToUnreal), 12, ToUnrealColor(Color),
			false, -1.0f, 0, GetThickness(Context));
	}

	void DrawCapsule(b3Pos P1, b3Pos P2, float Radius, b3HexColor Color, float Alpha, void* Context)
	{
		const FVector A = Box3D::FromBox3DPosition(P1);
		const FVector B = Box3D::FromBox3DPosition(P2);
		const FVector Center = (A + B) * 0.5;
		const FVector Axis = B - A;

		// DrawDebugCapsule wants half the *total* height and a rotation putting Z on the axis.
		const double HalfAxis = Axis.Size() * 0.5;
		const float UnrealRadius = Radius * static_cast<float>(Box3D::MetersToUnreal);
		const FQuat Rotation = Axis.IsNearlyZero()
			? FQuat::Identity
			: FRotationMatrix::MakeFromZ(Axis.GetSafeNormal()).ToQuat();

		DrawDebugCapsule(ContextWorld(Context), Center, static_cast<float>(HalfAxis) + UnrealRadius,
			UnrealRadius, Rotation, ToUnrealColor(Color), false, -1.0f, 0, GetThickness(Context));
	}

	void DrawBounds(b3AABB Box, b3HexColor Color, void* Context)
	{
		FBox Bounds(ForceInit);
		Bounds += Box3D::FromBox3DVector(Box.lowerBound);
		Bounds += Box3D::FromBox3DVector(Box.upperBound);

		DrawDebugBox(ContextWorld(Context), Bounds.GetCenter(), Bounds.GetExtent(), ToUnrealColor(Color),
			false, -1.0f, 0, GetThickness(Context));
	}

	void DrawBox(b3Vec3 Extents, b3WorldTransform Transform, b3HexColor Color, void* Context)
	{
		const FTransform Xf = Box3D::FromBox3DTransform(Transform);
		const FVector Extent = Box3D::FromBox3DVector(Extents).GetAbs();

		DrawDebugBox(ContextWorld(Context), Xf.GetLocation(), Extent, Xf.GetRotation(),
			ToUnrealColor(Color), false, -1.0f, 0, GetThickness(Context));
	}

	void DrawTransform(b3WorldTransform Transform, void* Context)
	{
		// Joint frames come through here, which is the main reason to use this renderer.
		const FTransform Xf = Box3D::FromBox3DTransform(Transform);
		DrawDebugCoordinateSystem(ContextWorld(Context), Xf.GetLocation(), Xf.Rotator(), 25.0f,
			false, -1.0f, 0, GetThickness(Context));
	}

	void DrawString(b3Pos P, const char* Text, b3HexColor Color, void* Context)
	{
		DrawDebugString(ContextWorld(Context), Box3D::FromBox3DPosition(P), ANSI_TO_TCHAR(Text),
			nullptr, ToUnrealColor(Color), 0.0f);
	}
}

void UBox3DSubsystem::NativeDebugDraw()
{
	UWorld* World = GetWorld();
	if (!bWorldValid || World == nullptr)
	{
		return;
	}

	FDrawContext Context;
	Context.World = World;
	Context.Thickness = NativeDrawThickness;

	b3DebugDraw Draw = {};
	Draw.DrawSegmentFcn = &DrawSegment;
	Draw.DrawPointFcn = &DrawPoint;
	Draw.DrawSphereFcn = &DrawSphere;
	Draw.DrawCapsuleFcn = &DrawCapsule;
	Draw.DrawBoundsFcn = &DrawBounds;
	Draw.DrawBoxFcn = &DrawBox;
	Draw.DrawTransformFcn = &DrawTransform;
	Draw.DrawStringFcn = &DrawString;
	Draw.context = &Context;

	// Everything box3d will draw into; unbounded would include the far static tree.
	const float Half = NativeDrawRange * static_cast<float>(Box3D::UnrealToMeters);
	b3Pos Center{ 0.0, 0.0, 0.0 };
	if (const APlayerController* PC = World->GetFirstPlayerController())
	{
		FVector ViewLocation;
		FRotator ViewRotation;
		PC->GetPlayerViewPoint(ViewLocation, ViewRotation);
		Center = Box3D::ToBox3DPosition(ViewLocation);
	}
	Draw.drawingBounds.lowerBound = b3Vec3{
		static_cast<float>(Center.x) - Half, static_cast<float>(Center.y) - Half,
		static_cast<float>(Center.z) - Half };
	Draw.drawingBounds.upperBound = b3Vec3{
		static_cast<float>(Center.x) + Half, static_cast<float>(Center.y) + Half,
		static_cast<float>(Center.z) + Half };

	Draw.forceScale = 0.05f;
	Draw.jointScale = 1.0f;

	const int32 Flags = NativeDrawFlags;
	Draw.drawShapes = (Flags & static_cast<int32>(EBox3DDrawFlag::Shapes)) != 0;
	Draw.drawJoints = (Flags & static_cast<int32>(EBox3DDrawFlag::Joints)) != 0;
	Draw.drawJointExtras = (Flags & static_cast<int32>(EBox3DDrawFlag::JointExtras)) != 0;
	Draw.drawBounds = (Flags & static_cast<int32>(EBox3DDrawFlag::Bounds)) != 0;
	Draw.drawMass = (Flags & static_cast<int32>(EBox3DDrawFlag::Mass)) != 0;
	Draw.drawSleep = (Flags & static_cast<int32>(EBox3DDrawFlag::Sleep)) != 0;
	Draw.drawContacts = (Flags & static_cast<int32>(EBox3DDrawFlag::Contacts)) != 0;
	Draw.drawContactNormals = (Flags & static_cast<int32>(EBox3DDrawFlag::ContactNormals)) != 0;
	Draw.drawContactForces = (Flags & static_cast<int32>(EBox3DDrawFlag::ContactForces)) != 0;
	Draw.drawIslands = (Flags & static_cast<int32>(EBox3DDrawFlag::Islands)) != 0;
	Draw.drawGraphColors = (Flags & static_cast<int32>(EBox3DDrawFlag::GraphColors)) != 0;

	b3World_Draw(WorldId, &Draw, ~0ull);
}
