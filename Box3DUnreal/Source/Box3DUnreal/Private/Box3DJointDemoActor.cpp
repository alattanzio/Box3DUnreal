// Author: Antonio Lattanzio - emptyvessel

#include "Box3DJointDemoActor.h"
#include "Box3DBodyComponent.h"
#include "Box3DJoints.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "UObject/ConstructorHelpers.h"

ABox3DJointDemoActor::ABox3DJointDemoActor()
{
	PrimaryActorTick.bCanEverTick = false;

	static ConstructorHelpers::FObjectFinder<UStaticMesh> CubeMesh(
		TEXT("/Engine/BasicShapes/Cube.Cube"));
	if (CubeMesh.Succeeded())
	{
		LinkMesh = CubeMesh.Object;
	}

	static ConstructorHelpers::FObjectFinder<UStaticMesh> BallMesh(
		TEXT("/Engine/BasicShapes/Sphere.Sphere"));
	if (BallMesh.Succeeded())
	{
		SphereMesh = BallMesh.Object;
	}
}

void ABox3DJointDemoActor::BeginPlay()
{
	Super::BeginPlay();

	switch (Demo)
	{
	case EBox3DJointDemo::Chain:  BuildChain(); break;
	case EBox3DJointDemo::Bridge: BuildBridge(); break;
	case EBox3DJointDemo::Newton: BuildNewtonsCradle(); break;
	case EBox3DJointDemo::Motor:  BuildMotor(); break;
	}
}

UBox3DBodyComponent* ABox3DJointDemoActor::SpawnLink(const FVector& WorldLocation,
	const FVector& HalfExtent, bool bStatic, bool bSphere, const FBox3DDemoMaterial* Material)
{
	UWorld* World = GetWorld();
	if (World == nullptr)
	{
		return nullptr;
	}

	FActorSpawnParameters Params;
	Params.Owner = this;
	AActor* Link = World->SpawnActor<AActor>(AActor::StaticClass(), FTransform(WorldLocation), Params);
	if (Link == nullptr)
	{
		return nullptr;
	}

	// The engine cube and sphere are both 100cm, so scale is half-extent / 50.
	UStaticMeshComponent* MeshComp = NewObject<UStaticMeshComponent>(Link);
	MeshComp->SetStaticMesh(bSphere && SphereMesh != nullptr ? SphereMesh : LinkMesh);
	MeshComp->SetRelativeScale3D(HalfExtent / 50.0);
	MeshComp->SetCollisionEnabled(ECollisionEnabled::NoCollision); // box3d owns collision
	Link->SetRootComponent(MeshComp);
	MeshComp->RegisterComponent();

	UBox3DBodyComponent* Body = NewObject<UBox3DBodyComponent>(Link);
	Body->BodyType = bStatic ? EBox3DBodyType::Static : EBox3DBodyType::Dynamic;
	if (bSphere)
	{
		Body->Shape = EBox3DShape::Sphere;
		Body->Radius = static_cast<float>(HalfExtent.X);
	}
	else
	{
		Body->Shape = EBox3DShape::Box;
		Body->BoxHalfExtent = HalfExtent;
	}

	if (Material != nullptr)
	{
		Body->Restitution = Material->Restitution;
		Body->LinearDamping = Material->LinearDamping;
		Body->AngularDamping = Material->AngularDamping;
	}

	Body->RegisterComponent();

	SpawnedLinks.Add(Link);
	return Body;
}

void ABox3DJointDemoActor::BuildChain()
{
	const FVector Origin = GetActorLocation();
	const FVector HalfExtent(10.0, 10.0, LinkSpacing * 0.4);

	FBox3DJointSettings Settings;
	Settings.bCollideConnected = false;

	FBox3DSphericalJointSettings Spherical;
	Spherical.bEnableConeLimit = true;
	Spherical.ConeAngle = 60.0f;
	Spherical.bEnableTwistLimit = false;

	// Hang the chain off a static anchor so it swings instead of falling.
	UBox3DBodyComponent* Previous = SpawnLink(Origin, FVector(20.0, 20.0, 10.0), true);

	for (int32 Index = 0; Index < LinkCount; ++Index)
	{
		const FVector Location = Origin - FVector(0.0, 0.0, LinkSpacing * (Index + 1));
		UBox3DBodyComponent* Link = SpawnLink(Location, HalfExtent, false);

		// Anchor at the midpoint, so each joint sits between the two links it connects.
		const FVector Anchor = Location + FVector(0.0, 0.0, LinkSpacing * 0.5);
		DemoJoints.Add(UBox3DJointLibrary::CreateSphericalJoint(
			Previous, Link, Anchor, FVector::UpVector, Settings, Spherical));

		Previous = Link;
	}
}

void ABox3DJointDemoActor::BuildBridge()
{
	const FVector Origin = GetActorLocation();
	const FVector HalfExtent(LinkSpacing * 0.45, 60.0, 5.0);

	FBox3DJointSettings Settings;

	// Hinges about Y let the planks sag under a load and spring back.
	FBox3DRevoluteJointSettings Revolute;
	Revolute.bEnableLimit = true;
	Revolute.LowerAngle = -30.0f;
	Revolute.UpperAngle = 30.0f;

	UBox3DBodyComponent* Previous = SpawnLink(Origin, FVector(20.0, 60.0, 20.0), true);

	for (int32 Index = 0; Index < LinkCount; ++Index)
	{
		const FVector Location = Origin + FVector(LinkSpacing * (Index + 1), 0.0, 0.0);
		UBox3DBodyComponent* Plank = SpawnLink(Location, HalfExtent, false);

		const FVector Anchor = Location - FVector(LinkSpacing * 0.5, 0.0, 0.0);
		DemoJoints.Add(UBox3DJointLibrary::CreateRevoluteJoint(
			Previous, Plank, Anchor, FVector::RightVector, Settings, Revolute));

		Previous = Plank;
	}

	// Anchor the far end too, otherwise the bridge is just a hanging chain.
	const FVector FarEnd = Origin + FVector(LinkSpacing * (LinkCount + 1), 0.0, 0.0);
	UBox3DBodyComponent* FarAnchor = SpawnLink(FarEnd, FVector(20.0, 60.0, 20.0), true);
	DemoJoints.Add(UBox3DJointLibrary::CreateRevoluteJoint(
		Previous, FarAnchor, FarEnd - FVector(LinkSpacing * 0.5, 0.0, 0.0),
		FVector::RightVector, Settings, Revolute));
}

void ABox3DJointDemoActor::BuildNewtonsCradle()
{
	const FVector Origin = GetActorLocation();
	const double ArmLength = LinkSpacing * 3.0;
	const double BallRadius = LinkSpacing * 0.5;

	FBox3DJointSettings Settings;

	// A hinge about Y keeps every ball in one plane - the whole point of the cradle.
	FBox3DRevoluteJointSettings Revolute;
	Revolute.bEnableLimit = false;

	for (int32 Index = 0; Index < LinkCount; ++Index)
	{
		// Balls just touching, so the impulse passes straight down the row.
		const double X = Index * BallRadius * 2.0;
		const FVector Pivot = Origin + FVector(X, 0.0, 0.0);
		const FVector BallLocation = Pivot - FVector(0.0, 0.0, ArmLength);

		// Perfectly elastic and undamped, so the row keeps clacking.
		FBox3DDemoMaterial Elastic;
		Elastic.Restitution = 1.0f;
		Elastic.LinearDamping = 0.0f;
		Elastic.AngularDamping = 0.0f;

		UBox3DBodyComponent* Anchor = SpawnLink(Pivot, FVector(5.0, 5.0, 5.0), true);
		UBox3DBodyComponent* Ball = SpawnLink(BallLocation,
			FVector(BallRadius, BallRadius, BallRadius), false, /*bSphere=*/true, &Elastic);

		DemoJoints.Add(UBox3DJointLibrary::CreateRevoluteJoint(
			Anchor, Ball, Pivot, FVector::RightVector, Settings, Revolute));

		if (Index == 0)
		{
			Ball->SetLinearVelocity(FVector(-LinkSpacing * 4.0, 0.0, 0.0));
		}
	}
}

void ABox3DJointDemoActor::BuildMotor()
{
	const FVector Origin = GetActorLocation();

	UBox3DBodyComponent* Anchor = SpawnLink(Origin, FVector(20.0, 20.0, 20.0), true);

	// Offset so the arm sticks out from the pivot and its swing is obvious.
	const FVector ArmLocation = Origin + FVector(LinkSpacing, 0.0, 0.0);
	UBox3DBodyComponent* Arm = SpawnLink(ArmLocation, FVector(LinkSpacing, 10.0, 10.0), false);

	FBox3DJointSettings Settings;

	FBox3DRevoluteJointSettings Revolute;
	Revolute.bEnableMotor = true;
	Revolute.MotorSpeed = MotorSpeed;
	Revolute.MaxMotorTorque = 500.0f;

	// Spin about Z: the arm sweeps horizontally, so gravity doesn't fight the motor.
	DemoJoints.Add(UBox3DJointLibrary::CreateRevoluteJoint(
		Anchor, Arm, Origin, FVector::UpVector, Settings, Revolute));
}
