// Author: Antonio Lattanzio - emptyvessel

#include "Box3DStaticGeometry.h"
#include "Box3DCollisionData.h"
#include "Box3DConversion.h"
#include "Box3DLog.h"
#include "Components/PrimitiveComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "GameFramework/Actor.h"
#include "Interfaces/Interface_CollisionDataProvider.h"
#include "Interface_CollisionDataProviderCore.h"
#include "PhysicsEngine/AggregateGeom.h"
#include "PhysicsEngine/BodySetup.h"
#include "PhysicsEngine/ConvexElem.h"

namespace Box3D::StaticGeometry
{
	namespace
	{
		constexpr int32 MaxHullVertices = 64;

		// Local vertex (cm) -> box3d point (m): bake scale, negate Y (see Box3DConversion.h).
		FORCEINLINE FVector3f LocalToBaked(const FVector& V, const FVector& Scale)
		{
			return FVector3f(
				static_cast<float>(V.X * Scale.X * Box3D::UnrealToMeters),
				static_cast<float>(-V.Y * Scale.Y * Box3D::UnrealToMeters),
				static_cast<float>(V.Z * Scale.Z * Box3D::UnrealToMeters));
		}

		FORCEINLINE b3Vec3 ToB3(const FVector3f& V) { return b3Vec3{ V.X, V.Y, V.Z }; }

		FORCEINLINE bool ShouldReverseWinding(const FVector& Scale, bool bInvert)
		{
			const bool bNegativeScale = (Scale.X * Scale.Y * Scale.Z) < 0.0;
			return bNegativeScale ^ bInvert;
		}

		IInterface_CollisionDataProvider* FindTriMeshProvider(UPrimitiveComponent* Prim)
		{
			// Landscape collision components implement the provider directly.
			if (IInterface_CollisionDataProvider* Direct = Cast<IInterface_CollisionDataProvider>(Prim))
			{
				return Direct;
			}
			if (const UStaticMeshComponent* SMC = Cast<UStaticMeshComponent>(Prim))
			{
				if (UStaticMesh* Mesh = SMC->GetStaticMesh())
				{
					return Cast<IInterface_CollisionDataProvider>(Mesh);
				}
			}
			return nullptr;
		}

		// --- Stage 1 helpers: UE collision -> FBox3DBakedShape (box3d local space) ---------

		bool AppendHull(TArray<FBox3DBakedShape>& OutShapes, const TArray<FVector>& Points, const FVector& Scale)
		{
			if (Points.Num() < 4)
			{
				return false;
			}

			FBox3DBakedShape Shape;
			Shape.Kind = EBox3DBakedShapeKind::Hull;
			Shape.Points.Reserve(Points.Num());
			for (const FVector& P : Points)
			{
				Shape.Points.Add(LocalToBaked(P, Scale));
			}

			// Validate now (in the editor) so a bad cloud fails visibly here, not at load.
			TArray<b3Vec3> Converted;
			Converted.Reserve(Shape.Points.Num());
			for (const FVector3f& P : Shape.Points)
			{
				Converted.Add(ToB3(P));
			}
			b3HullData* Hull = b3CreateHull(Converted.GetData(), Converted.Num(), MaxHullVertices);
			if (Hull == nullptr)
			{
				return false;
			}
			b3DestroyHull(Hull);

			OutShapes.Add(MoveTemp(Shape));
			return true;
		}

		// Simple collision: AggGeom convex/box/sphere/capsule -> baked shapes. Returns count.
		int32 ExtractSimpleCollision(UPrimitiveComponent* Prim, const FVector& Scale, TArray<FBox3DBakedShape>& OutShapes)
		{
			UBodySetup* Setup = Prim->GetBodySetup();
			if (Setup == nullptr)
			{
				return 0;
			}

			const FKAggregateGeom& Agg = Setup->AggGeom;
			int32 Created = 0;

			// Convex: element transform places local verts into body space.
			for (const FKConvexElem& Convex : Agg.ConvexElems)
			{
				const FTransform ElemTM = Convex.GetTransform();
				TArray<FVector> Points;
				Points.Reserve(Convex.VertexData.Num());
				for (const FVector& V : Convex.VertexData)
				{
					Points.Add(ElemTM.TransformPosition(V));
				}
				Created += AppendHull(OutShapes, Points, Scale) ? 1 : 0;
			}

			// Boxes: 8 corners through the hull path.
			for (const FKBoxElem& Box : Agg.BoxElems)
			{
				const FTransform ElemTM(Box.Rotation, Box.Center);
				const FVector He(Box.X * 0.5f, Box.Y * 0.5f, Box.Z * 0.5f);
				TArray<FVector> Corners;
				Corners.Reserve(8);
				for (int32 Sx = -1; Sx <= 1; Sx += 2)
				for (int32 Sy = -1; Sy <= 1; Sy += 2)
				for (int32 Sz = -1; Sz <= 1; Sz += 2)
				{
					Corners.Add(ElemTM.TransformPosition(FVector(Sx * He.X, Sy * He.Y, Sz * He.Z)));
				}
				Created += AppendHull(OutShapes, Corners, Scale) ? 1 : 0;
			}

			// Non-uniform scale can't stay round, so radii use the min axis scale.
			const float RadialScale = static_cast<float>(Scale.GetAbsMin());
			const float M = static_cast<float>(Box3D::UnrealToMeters);
			for (const FKSphereElem& Sph : Agg.SphereElems)
			{
				FBox3DBakedShape Shape;
				Shape.Kind = EBox3DBakedShapeKind::Sphere;
				Shape.CenterA = LocalToBaked(Sph.Center, Scale);
				Shape.Radius = Sph.Radius * RadialScale * M;
				OutShapes.Add(MoveTemp(Shape));
				++Created;
			}

			// Sphyls: local-Z axis, Length spans the two hemisphere centers.
			for (const FKSphylElem& Capsule : Agg.SphylElems)
			{
				const FTransform ElemTM(Capsule.Rotation, Capsule.Center);
				const float HalfLen = Capsule.Length * 0.5f;
				FBox3DBakedShape Shape;
				Shape.Kind = EBox3DBakedShapeKind::Capsule;
				Shape.CenterA = LocalToBaked(ElemTM.TransformPosition(FVector(0, 0, +HalfLen)), Scale);
				Shape.CenterB = LocalToBaked(ElemTM.TransformPosition(FVector(0, 0, -HalfLen)), Scale);
				Shape.Radius = Capsule.Radius * RadialScale * M;
				OutShapes.Add(MoveTemp(Shape));
				++Created;
			}

			return Created;
		}

		bool ExtractComplexTriMesh(
			IInterface_CollisionDataProvider* Provider, const FVector& Scale, bool bInvertWinding,
			TArray<FBox3DBakedShape>& OutShapes)
		{
			if (Provider == nullptr || !Provider->ContainsPhysicsTriMeshData(true))
			{
				return false;
			}

			FTriMeshCollisionData TriData;
			if (!Provider->GetPhysicsTriMeshData(&TriData, /*InUseAllTriData=*/true))
			{
				return false;
			}
			if (TriData.Vertices.Num() < 3 || TriData.Indices.Num() < 1)
			{
				return false;
			}

			FBox3DBakedShape Shape;
			Shape.Kind = EBox3DBakedShapeKind::Mesh;
			Shape.Points.Reserve(TriData.Vertices.Num());
			for (const FVector3f& V : TriData.Vertices)
			{
				Shape.Points.Add(LocalToBaked(FVector(V), Scale));
			}

			const bool bReverse = ShouldReverseWinding(Scale, bInvertWinding);
			Shape.Indices.Reserve(TriData.Indices.Num() * 3);
			for (const FTriIndices& Tri : TriData.Indices)
			{
				Shape.Indices.Add(Tri.v0);
				if (bReverse)
				{
					Shape.Indices.Add(Tri.v2);
					Shape.Indices.Add(Tri.v1);
				}
				else
				{
					Shape.Indices.Add(Tri.v1);
					Shape.Indices.Add(Tri.v2);
				}
			}

			OutShapes.Add(MoveTemp(Shape));
			return true;
		}

		// --- Stage 2 helpers: FBox3DBakedShape -> box3d shape on a body -------------------

		void InstantiateHull(b3BodyId Body, const b3ShapeDef& Def, const FBox3DBakedShape& Shape)
		{
			if (Shape.Points.Num() < 4)
			{
				return;
			}
			TArray<b3Vec3> Points;
			Points.Reserve(Shape.Points.Num());
			for (const FVector3f& P : Shape.Points)
			{
				Points.Add(ToB3(P));
			}
			b3HullData* Hull = b3CreateHull(Points.GetData(), Points.Num(), MaxHullVertices);
			if (Hull == nullptr)
			{
				return;
			}
			b3CreateHullShape(Body, &Def, Hull); // box3d clones the hull; free ours after
			b3DestroyHull(Hull);
		}

		// Returns the b3MeshData the caller must free after the body (nullptr on failure).
		b3MeshData* InstantiateMesh(b3BodyId Body, const b3ShapeDef& Def, const FBox3DBakedShape& Shape)
		{
			if (Shape.Points.Num() < 3 || Shape.Indices.Num() < 3)
			{
				return nullptr;
			}

			TArray<b3Vec3> Vertices;
			Vertices.Reserve(Shape.Points.Num());
			for (const FVector3f& P : Shape.Points)
			{
				Vertices.Add(ToB3(P));
			}

			b3MeshDef MeshDef{};
			MeshDef.vertices = Vertices.GetData();
			MeshDef.indices = const_cast<int32*>(Shape.Indices.GetData());
			MeshDef.materialIndices = nullptr; // base material for all triangles
			MeshDef.weldTolerance = 0.0f;
			MeshDef.vertexCount = Vertices.Num();
			MeshDef.triangleCount = Shape.Indices.Num() / 3;
			MeshDef.weldVertices = false;
			MeshDef.useMedianSplit = false;
			MeshDef.identifyEdges = true;   // adjacency avoids ghost collisions

			b3MeshData* Mesh = b3CreateMesh(&MeshDef, nullptr, 0);
			if (Mesh == nullptr)
			{
				return nullptr;
			}

			// Scale already baked into verts, so unit shape scale.
			b3CreateMeshShape(Body, &Def, Mesh, b3Vec3{ 1.0f, 1.0f, 1.0f });
			return Mesh;
		}
	} // namespace

	bool ExtractStaticCollision(AActor* Owner, ESource Source, bool bInvertWinding, FBox3DBakedBody& Out)
	{
		UPrimitiveComponent* Prim = Owner ? Cast<UPrimitiveComponent>(Owner->GetRootComponent()) : nullptr;
		if (Prim == nullptr)
		{
			return false;
		}

		Out.WorldTransform = Owner->GetActorTransform();
		Out.ActorKey = Owner->GetPathName();

		const FVector Scale = Prim->GetComponentScale();

		if (Source == ESource::ComplexCollision || Source == ESource::Auto)
		{
			IInterface_CollisionDataProvider* Provider = FindTriMeshProvider(Prim);
			if (ExtractComplexTriMesh(Provider, Scale, bInvertWinding, Out.Shapes))
			{
				return true;
			}
			if (Source == ESource::ComplexCollision)
			{
				UE_LOG(LogBox3D, Warning,
					TEXT("%s: no complex (tri-mesh) collision available; static body has no shape. ")
					TEXT("Enable 'Allow CPU Access' / complex collision on the mesh, or use Simple/Auto."),
					*GetNameSafe(Owner));
				return false;
			}
		}

		// Auto fell through, or SimpleCollision requested.
		if (ExtractSimpleCollision(Prim, Scale, Out.Shapes) > 0)
		{
			return true;
		}

		UE_LOG(LogBox3D, Warning,
			TEXT("%s: no cooked collision found for static body (no tri-mesh, no simple primitives)."),
			*GetNameSafe(Owner));
		return false;
	}

	bool AddBakedShapes(
		b3BodyId Body, const b3ShapeDef& Base, const FBox3DBakedBody& Baked, TArray<b3MeshData*>& OutOwnedMeshes)
	{
		int32 Created = 0;
		for (const FBox3DBakedShape& Shape : Baked.Shapes)
		{
			switch (Shape.Kind)
			{
			case EBox3DBakedShapeKind::Hull:
				InstantiateHull(Body, Base, Shape);
				++Created;
				break;
			case EBox3DBakedShapeKind::Mesh:
				if (b3MeshData* Mesh = InstantiateMesh(Body, Base, Shape))
				{
					OutOwnedMeshes.Add(Mesh);
					++Created;
				}
				break;
			case EBox3DBakedShapeKind::Sphere:
			{
				b3Sphere Sphere;
				Sphere.center = ToB3(Shape.CenterA);
				Sphere.radius = Shape.Radius;
				b3CreateSphereShape(Body, &Base, &Sphere);
				++Created;
				break;
			}
			case EBox3DBakedShapeKind::Capsule:
			{
				b3Capsule Capsule;
				Capsule.center1 = ToB3(Shape.CenterA);
				Capsule.center2 = ToB3(Shape.CenterB);
				Capsule.radius = Shape.Radius;
				b3CreateCapsuleShape(Body, &Base, &Capsule);
				++Created;
				break;
			}
			}
		}
		return Created > 0;
	}

	bool AddStaticShapes(
		b3BodyId Body, const b3ShapeDef& Base, AActor* Owner, ESource Source,
		bool bInvertWinding, TArray<b3MeshData*>& OutOwnedMeshes)
	{
		FBox3DBakedBody Baked;
		if (!ExtractStaticCollision(Owner, Source, bInvertWinding, Baked))
		{
			return false;
		}
		return AddBakedShapes(Body, Base, Baked, OutOwnedMeshes);
	}

	FString GetBox3DVersionString()
	{
		const b3Version V = b3GetVersion();
		return FString::Printf(TEXT("%d.%d.%d"), V.major, V.minor, V.revision);
	}
} // namespace Box3D::StaticGeometry
