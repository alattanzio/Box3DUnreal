// Author: Antonio Lattanzio - emptyvessel

#pragma once

#include "CoreMinimal.h"
#include "Box3DCollisionData.generated.h"


// One shape's kind. Mirrors what the extraction produces; the loader switches on it.
UENUM()
enum class EBox3DBakedShapeKind : uint8
{
	Hull,     // Convex hull rebuilt at load from Points (b3CreateHull is deterministic).
	Mesh,     // Static tri-mesh: Points = vertices, Indices = triangles (winding fixed).
	Sphere,   // CenterA + Radius.
	Capsule   // CenterA/CenterB (hemisphere centers) + Radius.
};

USTRUCT()
struct FBox3DBakedShape
{
	GENERATED_BODY()

	UPROPERTY()
	EBox3DBakedShapeKind Kind = EBox3DBakedShapeKind::Hull;

	// Hull point cloud, or Mesh vertices. Empty for sphere/capsule.
	UPROPERTY()
	TArray<FVector3f> Points;

	// Mesh triangle indices (3 per triangle). Empty unless Kind == Mesh.
	UPROPERTY()
	TArray<int32> Indices;

	// Sphere center / capsule first hemisphere center.
	UPROPERTY()
	FVector3f CenterA = FVector3f::ZeroVector;

	// Capsule second hemisphere center (unused otherwise).
	UPROPERTY()
	FVector3f CenterB = FVector3f::ZeroVector;

	// Sphere / capsule radius (meters).
	UPROPERTY()
	float Radius = 0.0f;
};

USTRUCT()
struct FBox3DBakedBody
{
	GENERATED_BODY()

	UPROPERTY()
	FTransform WorldTransform = FTransform::Identity;

	// Source actor path within its level - metadata for rebake diffing, not used at load.
	UPROPERTY()
	FString ActorKey;

	UPROPERTY()
	TArray<FBox3DBakedShape> Shapes;
};

/**
 * Cached static collision for one level (or asset). Produced by the bake commandlet
 * from the level's Static-body actors and loaded at runtime by UBox3DSubsystem, which
 * instantiates every body directly - no per-actor components, no runtime cooking, so it
 * works in packaged builds and stays deterministic across runs.
 */
UCLASS(BlueprintType)
class BOX3DUNREAL_API UBox3DCollisionData : public UObject
{
	GENERATED_BODY()

public:
	/** The asset a map bakes to by convention: BC_<MapName> beside the map. Shared by the
	 *  bake commandlet (output) and the runtime auto-discovery (lookup), so the two can
	 *  never disagree about where a level's baked collision lives. */
	static FString DeriveAssetPackageName(const FString& MapPackageName);

	// Level (or source) this was baked from, for identification in the editor.
	UPROPERTY(VisibleAnywhere, Category = "Box3D")
	FString SourceLevel;

	// box3d version string at bake time, so a version bump can be spotted as stale.
	UPROPERTY(VisibleAnywhere, Category = "Box3D")
	FString Box3DVersion;

	// True if any body was baked from complex (tri-mesh) collision.
	UPROPERTY(VisibleAnywhere, Category = "Box3D")
	bool bContainsTriMesh = false;

	// Tag the bake was run with (-Tag=), or None for the component-only bake.
	UPROPERTY(VisibleAnywhere, Category = "Box3D")
	FName BakeTag;

	UPROPERTY(VisibleAnywhere, Category = "Box3D")
	FDateTime BakeTime = FDateTime(0);

	/** Opaque identity of the source level's files at bake time (see ComputeSourceFingerprint).
	 *  Differs from the level's current fingerprint => the bake is out of date. */
	UPROPERTY(VisibleAnywhere, Category = "Box3D")
	FString SourceFingerprint;

	UPROPERTY()
	TArray<FBox3DBakedBody> Bodies;

#if WITH_EDITOR
	/** Cheap identity of a level on disk: newest modified time + total size + file count over
	 *  the .umap AND its _ExternalActors_ folder. The external actors matter - under One File
	 *  Per Actor, moving an actor rewrites its own package and never touches the .umap, so a
	 *  .umap-only timestamp would call an edited world fresh. Empty if the map can't be found.
	 *
	 *  This is a "something changed" signal, not a geometry hash: re-saving a level without
	 *  touching collision also changes it. It only ever warns, so a false positive costs a
	 *  re-bake, while a miss would ship wrong collision. */
	static FString ComputeSourceFingerprint(const FString& MapPackageName);

	/** True if this bake can no longer be trusted: box3d version bump, source level edited
	 *  since the bake, or a bake old enough to predate fingerprinting. */
	bool IsStale(FString& OutReason) const;
#endif
};
