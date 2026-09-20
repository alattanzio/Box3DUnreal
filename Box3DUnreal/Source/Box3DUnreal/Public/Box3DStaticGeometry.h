// Author: Antonio Lattanzio - emptyvessel

#pragma once

#include "CoreMinimal.h"
#include <box3d/box3d.h>

class AActor;
struct FBox3DBakedBody;

namespace Box3D::StaticGeometry
{
	// Which cooked collision to mirror for a Static body.
	enum class ESource : uint8
	{
		Auto,             // Complex tri-mesh if present, else simple.
		SimpleCollision,  // AggGeom convex/box/sphere/capsule.
		ComplexCollision, // Cooked tri-mesh (meshes & landscape).
	};

	BOX3DUNREAL_API bool ExtractStaticCollision(
		AActor* Owner,
		ESource Source,
		bool bInvertWinding,
		FBox3DBakedBody& Out);

	BOX3DUNREAL_API bool AddBakedShapes(
		b3BodyId Body,
		const b3ShapeDef& Base,
		const FBox3DBakedBody& Baked,
		TArray<b3MeshData*>& OutOwnedMeshes);

	BOX3DUNREAL_API bool AddStaticShapes(
		b3BodyId Body,
		const b3ShapeDef& Base,
		AActor* Owner,
		ESource Source,
		bool bInvertWinding,
		TArray<b3MeshData*>& OutOwnedMeshes);

	BOX3DUNREAL_API FString GetBox3DVersionString();
}
