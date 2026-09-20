// Author: Antonio Lattanzio - emptyvessel

#include "Box3DJointComponent.h"
#include "Box3DBodyComponent.h"
#include "Box3DJoints.h"
#include "Box3DLog.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "TimerManager.h"

UBox3DJointComponent::UBox3DJointComponent()
{
	PrimaryComponentTick.bCanEverTick = false;
}

void UBox3DJointComponent::BeginPlay()
{
	Super::BeginPlay();

	if (UWorld* World = GetWorld())
	{
		FTimerHandle Handle;
		World->GetTimerManager().SetTimerForNextTick(
			FTimerDelegate::CreateWeakLambda(this, [this] { CreateJoints(); }));
	}
}

void UBox3DJointComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	DestroyJoints();
	Super::EndPlay(EndPlayReason);
}

UBox3DBodyComponent* UBox3DJointComponent::FindBody(AActor* Actor, FName ComponentName) const
{
	if (Actor == nullptr)
	{
		return nullptr;
	}

	if (ComponentName.IsNone())
	{
		return Actor->FindComponentByClass<UBox3DBodyComponent>();
	}

	TArray<UBox3DBodyComponent*> Bodies;
	Actor->GetComponents(Bodies);
	for (UBox3DBodyComponent* Body : Bodies)
	{
		if (Body->GetFName() == ComponentName)
		{
			return Body;
		}
	}

	UE_LOG(LogBox3D, Warning, TEXT("Box3D joint: no body component '%s' on %s."),
		*ComponentName.ToString(), *Actor->GetName());
	return nullptr;
}

void UBox3DJointComponent::CreateJoints()
{
	if (bJointsCreated)
	{
		return;
	}

	AActor* Owner = GetOwner();
	if (Owner == nullptr)
	{
		return;
	}

	const FTransform OwnerXform = Owner->GetActorTransform();
	CreatedJoints.Reserve(Joints.Num());

	for (const FBox3DAuthoredJoint& Authored : Joints)
	{
		UBox3DBodyComponent* A = FindBody(Owner, Authored.BodyA);

		AActor* OtherActor = Authored.OtherActor.Get();
		UBox3DBodyComponent* B = FindBody(OtherActor != nullptr ? OtherActor : Owner, Authored.BodyB);

		const FVector WorldAnchor = OwnerXform.TransformPosition(Authored.LocalAnchor);
		const FVector WorldAxis = OwnerXform.TransformVectorNoScale(Authored.LocalAxis);

		FBox3DJointHandle Handle;
		switch (Authored.Type)
		{
		case EBox3DJointType::Spherical:
			Handle = UBox3DJointLibrary::CreateSphericalJoint(A, B, WorldAnchor, WorldAxis,
				Authored.Settings, Authored.Spherical);
			break;
		case EBox3DJointType::Revolute:
			Handle = UBox3DJointLibrary::CreateRevoluteJoint(A, B, WorldAnchor, WorldAxis,
				Authored.Settings, Authored.Revolute);
			break;
		case EBox3DJointType::Prismatic:
			Handle = UBox3DJointLibrary::CreatePrismaticJoint(A, B, WorldAnchor, WorldAxis,
				Authored.Settings, Authored.Prismatic);
			break;
		case EBox3DJointType::Weld:
			Handle = UBox3DJointLibrary::CreateWeldJoint(A, B, WorldAnchor,
				Authored.Settings, Authored.Weld);
			break;
		}
		CreatedJoints.Add(Handle);
	}

	bJointsCreated = true;
}

void UBox3DJointComponent::DestroyJoints()
{
	for (const FBox3DJointHandle& Handle : CreatedJoints)
	{
		UBox3DJointLibrary::DestroyJoint(this, Handle, true);
	}
	CreatedJoints.Reset();
	bJointsCreated = false;
}
