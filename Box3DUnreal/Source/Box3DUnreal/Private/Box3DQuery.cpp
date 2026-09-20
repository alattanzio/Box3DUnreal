// Author: Antonio Lattanzio - emptyvessel

#include "Box3DBodyComponent.h"
#include "Box3DConversion.h"
#include "Box3DSnapshot.h"
#include "Box3DSubsystem.h"
#include "GameFramework/Actor.h"


namespace
{
	/** 0 keeps box3d's default (query everything), matching the component's filter convention. */
	b3QueryFilter MakeQueryFilter(const FBox3DQueryFilter& In)
	{
		b3QueryFilter Filter = b3DefaultQueryFilter();
		if (In.Category != 0)
		{
			Filter.categoryBits = static_cast<uint64>(static_cast<uint32>(In.Category));
		}
		if (In.Mask != 0)
		{
			Filter.maskBits = static_cast<uint64>(static_cast<uint32>(In.Mask));
		}
		return Filter;
	}

	/** Bodies carry their actor in userData (set at creation). Baked static bodies have no
	 *  source actor and return null. The pointer is safe because a body is destroyed with
	 *  its actor (component EndPlay / level stream-out). */
	AActor* ActorFromShape(b3ShapeId ShapeId)
	{
		const b3BodyId Body = b3Shape_GetBody(ShapeId);
		if (B3_IS_NULL(Body))
		{
			return nullptr;
		}
		AActor* Actor = static_cast<AActor*>(b3Body_GetUserData(Body));
		return IsValid(Actor) ? Actor : nullptr;
	}

	FBox3DHitResult MakeHit(b3ShapeId ShapeId, const b3Pos& Point, const b3Vec3& Normal, float Fraction,
		int32 TriangleIndex, int32 ChildIndex, double Length)
	{
		FBox3DHitResult Hit;
		Hit.bHit = true;
		Hit.Location = Box3D::FromBox3DPosition(Point);
		Hit.Normal = Box3D::FromBox3DDirection(Normal);
		Hit.Fraction = Fraction;
		Hit.Distance = Fraction * Length;
		Hit.HitActor = ActorFromShape(ShapeId);
		Hit.TriangleIndex = TriangleIndex;
		Hit.ChildIndex = ChildIndex;
		return Hit;
	}

	struct FCastContext
	{
		double Length = 0.0;      // cm, Start->End; turns fraction into a distance
		FBox3DHitResult Closest;  // closest-hit casts
		TArray<FBox3DHitResult>* Hits = nullptr; // multi-hit casts
	};

	/** Closest-hit: store and return the fraction so box3d clips the cast to it. */
	float ClosestHitCallback(b3ShapeId ShapeId, b3Pos Point, b3Vec3 Normal, float Fraction, uint64_t /*MaterialId*/,
		int TriangleIndex, int ChildIndex, void* Context)
	{
		FCastContext& Ctx = *static_cast<FCastContext*>(Context);
		Ctx.Closest = MakeHit(ShapeId, Point, Normal, Fraction, TriangleIndex, ChildIndex, Ctx.Length);
		return Fraction;
	}

	/** Multi-hit: collect and return 1 so the cast continues unclipped. */
	float MultiHitCallback(b3ShapeId ShapeId, b3Pos Point, b3Vec3 Normal, float Fraction, uint64_t /*MaterialId*/,
		int TriangleIndex, int ChildIndex, void* Context)
	{
		FCastContext& Ctx = *static_cast<FCastContext*>(Context);
		Ctx.Hits->Add(MakeHit(ShapeId, Point, Normal, Fraction, TriangleIndex, ChildIndex, Ctx.Length));
		return 1.0f;
	}

	/** Overlaps report actors, so a body with no actor (baked static) is skipped. */
	bool OverlapCallback(b3ShapeId ShapeId, void* Context)
	{
		TArray<AActor*>& Actors = *static_cast<TArray<AActor*>*>(Context);
		if (AActor* Actor = ActorFromShape(ShapeId))
		{
			Actors.AddUnique(Actor); // one body can carry many shapes
		}
		return true; // keep going
	}

	/** Sphere = one point with a radius; box = its 8 corners with none. Points are relative
	 *  to the query origin, which is what keeps the query precise far from the world origin. */
	b3ShapeProxy MakeSphereProxy(b3Vec3& PointStorage, float RadiusCm)
	{
		PointStorage = b3Vec3{ 0.0f, 0.0f, 0.0f };
		b3ShapeProxy Proxy{};
		Proxy.points = &PointStorage;
		Proxy.count = 1;
		Proxy.radius = static_cast<float>(RadiusCm * Box3D::UnrealToMeters);
		return Proxy;
	}

	b3ShapeProxy MakeBoxProxy(b3Vec3 (&PointStorage)[8], const FVector& HalfExtent, const FRotator& Rotation)
	{
		const FVector H = HalfExtent.GetAbs();
		const FQuat Q = Rotation.Quaternion();
		int32 Index = 0;
		for (int32 SignX = -1; SignX <= 1; SignX += 2)
		{
			for (int32 SignY = -1; SignY <= 1; SignY += 2)
			{
				for (int32 SignZ = -1; SignZ <= 1; SignZ += 2)
				{
					const FVector Corner(SignX * H.X, SignY * H.Y, SignZ * H.Z);
					PointStorage[Index++] = Box3D::ToBox3DVector(Q.RotateVector(Corner));
				}
			}
		}

		b3ShapeProxy Proxy{};
		Proxy.points = PointStorage;
		Proxy.count = 8;
		Proxy.radius = 0.0f;
		return Proxy;
	}
}

uint32 UBox3DSubsystem::ComputeWorldStateHash(int32& OutBodyCount) const
{
	OutBodyCount = 0;
	if (!bWorldValid)
	{
		return 0;
	}

	FlushAsyncStep();

	TArray<TPair<FString, b3BodyId>> Bodies;
	Bodies.Reserve(DynamicBodies.Num());
	for (const TWeakObjectPtr<UBox3DBodyComponent>& Weak : DynamicBodies)
	{
		const UBox3DBodyComponent* Comp = Weak.Get();
		if (Comp == nullptr || B3_IS_NULL(Comp->GetBodyId()))
		{
			continue;
		}
		Bodies.Emplace(GetNameSafe(Comp->GetOwner()), Comp->GetBodyId());
	}
	Bodies.Sort([](const TPair<FString, b3BodyId>& A, const TPair<FString, b3BodyId>& B) { return A.Key < B.Key; });

	uint32 Hash = B3_HASH_INIT;
	for (const TPair<FString, b3BodyId>& Entry : Bodies)
	{
		Hash = Box3D::HashBody(Hash, Entry.Value);
	}
	OutBodyCount = Bodies.Num();
	return Hash;
}

bool UBox3DSubsystem::RaycastClosest(const FVector& Start, const FVector& End, const FBox3DQueryFilter& Filter,
	FBox3DHitResult& OutHit) const
{
	OutHit = FBox3DHitResult();
	const FVector Delta = End - Start;
	if (!bWorldValid || Delta.IsNearlyZero())
	{
		return false;
	}

	FlushAsyncStep();

	const b3RayResult Result = b3World_CastRayClosest(
		WorldId, Box3D::ToBox3DPosition(Start), Box3D::ToBox3DVector(Delta), MakeQueryFilter(Filter));
	if (!Result.hit)
	{
		return false;
	}

	OutHit = MakeHit(Result.shapeId, Result.point, Result.normal, Result.fraction, Result.triangleIndex,
		Result.childIndex, Delta.Size());
	return true;
}

bool UBox3DSubsystem::RaycastMulti(const FVector& Start, const FVector& End, const FBox3DQueryFilter& Filter,
	TArray<FBox3DHitResult>& OutHits) const
{
	OutHits.Reset();
	const FVector Delta = End - Start;
	if (!bWorldValid || Delta.IsNearlyZero())
	{
		return false;
	}

	FlushAsyncStep();

	FCastContext Ctx;
	Ctx.Length = Delta.Size();
	Ctx.Hits = &OutHits;
	b3World_CastRay(WorldId, Box3D::ToBox3DPosition(Start), Box3D::ToBox3DVector(Delta), MakeQueryFilter(Filter),
		&MultiHitCallback, &Ctx);

	// box3d reports hits in traversal order, not along the ray.
	OutHits.Sort([](const FBox3DHitResult& A, const FBox3DHitResult& B) { return A.Fraction < B.Fraction; });
	return OutHits.Num() > 0;
}

bool UBox3DSubsystem::OverlapAABB(const FVector& Center, const FVector& HalfExtent, const FBox3DQueryFilter& Filter,
	TArray<AActor*>& OutActors) const
{
	OutActors.Reset();
	if (!bWorldValid)
	{
		return false;
	}

	FlushAsyncStep();

	const FVector H = HalfExtent.GetAbs();
	const b3Vec3 A = Box3D::ToBox3DVector(Center - H);
	const b3Vec3 B = Box3D::ToBox3DVector(Center + H);

	b3AABB Bounds;
	Bounds.lowerBound = b3Vec3{ FMath::Min(A.x, B.x), FMath::Min(A.y, B.y), FMath::Min(A.z, B.z) };
	Bounds.upperBound = b3Vec3{ FMath::Max(A.x, B.x), FMath::Max(A.y, B.y), FMath::Max(A.z, B.z) };

	b3World_OverlapAABB(WorldId, Bounds, MakeQueryFilter(Filter), &OverlapCallback, &OutActors);
	return OutActors.Num() > 0;
}

bool UBox3DSubsystem::OverlapSphere(const FVector& Center, float Radius, const FBox3DQueryFilter& Filter,
	TArray<AActor*>& OutActors) const
{
	OutActors.Reset();
	if (!bWorldValid || Radius <= 0.0f)
	{
		return false;
	}

	FlushAsyncStep();

	b3Vec3 Point;
	const b3ShapeProxy Proxy = MakeSphereProxy(Point, Radius);
	b3World_OverlapShape(WorldId, Box3D::ToBox3DPosition(Center), &Proxy, MakeQueryFilter(Filter), &OverlapCallback,
		&OutActors);
	return OutActors.Num() > 0;
}

bool UBox3DSubsystem::OverlapBox(const FVector& Center, const FVector& HalfExtent, const FRotator& Rotation,
	const FBox3DQueryFilter& Filter, TArray<AActor*>& OutActors) const
{
	OutActors.Reset();
	if (!bWorldValid)
	{
		return false;
	}

	FlushAsyncStep();

	b3Vec3 Points[8];
	const b3ShapeProxy Proxy = MakeBoxProxy(Points, HalfExtent, Rotation);
	b3World_OverlapShape(WorldId, Box3D::ToBox3DPosition(Center), &Proxy, MakeQueryFilter(Filter), &OverlapCallback,
		&OutActors);
	return OutActors.Num() > 0;
}

bool UBox3DSubsystem::SphereCast(const FVector& Start, const FVector& End, float Radius,
	const FBox3DQueryFilter& Filter, FBox3DHitResult& OutHit) const
{
	OutHit = FBox3DHitResult();
	const FVector Delta = End - Start;
	if (!bWorldValid || Radius <= 0.0f || Delta.IsNearlyZero())
	{
		return false;
	}

	FlushAsyncStep();

	b3Vec3 Point;
	const b3ShapeProxy Proxy = MakeSphereProxy(Point, Radius);

	FCastContext Ctx;
	Ctx.Length = Delta.Size();
	b3World_CastShape(WorldId, Box3D::ToBox3DPosition(Start), &Proxy, Box3D::ToBox3DVector(Delta),
		MakeQueryFilter(Filter), &ClosestHitCallback, &Ctx);

	OutHit = Ctx.Closest;
	return OutHit.bHit;
}

bool UBox3DSubsystem::BoxCast(const FVector& Start, const FVector& End, const FVector& HalfExtent,
	const FRotator& Rotation, const FBox3DQueryFilter& Filter, FBox3DHitResult& OutHit) const
{
	OutHit = FBox3DHitResult();
	const FVector Delta = End - Start;
	if (!bWorldValid || Delta.IsNearlyZero())
	{
		return false;
	}

	FlushAsyncStep();

	b3Vec3 Points[8];
	const b3ShapeProxy Proxy = MakeBoxProxy(Points, HalfExtent, Rotation);

	FCastContext Ctx;
	Ctx.Length = Delta.Size();
	b3World_CastShape(WorldId, Box3D::ToBox3DPosition(Start), &Proxy, Box3D::ToBox3DVector(Delta),
		MakeQueryFilter(Filter), &ClosestHitCallback, &Ctx);

	OutHit = Ctx.Closest;
	return OutHit.bHit;
}
