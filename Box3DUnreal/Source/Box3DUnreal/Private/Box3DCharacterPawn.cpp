// Author: Antonio Lattanzio - emptyvessel

#include "Box3DCharacterPawn.h"
#include "Box3DCharacterComponent.h"
#include "Camera/CameraComponent.h"
#include "Components/CapsuleComponent.h"
#include "Components/InputComponent.h"
#include "GameFramework/SpringArmComponent.h"

ABox3DCharacterPawn::ABox3DCharacterPawn()
{
	PrimaryActorTick.bCanEverTick = true;

	// Root is a plain capsule for bounds and editor picking; box3d owns collision.
	UCapsuleComponent* Capsule = CreateDefaultSubobject<UCapsuleComponent>(TEXT("Capsule"));
	Capsule->InitCapsuleSize(34.0f, 78.0f);
	Capsule->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	SetRootComponent(Capsule);

	SpringArm = CreateDefaultSubobject<USpringArmComponent>(TEXT("SpringArm"));
	SpringArm->SetupAttachment(Capsule);
	SpringArm->TargetArmLength = 400.0f;
	SpringArm->bUsePawnControlRotation = true;
	SpringArm->SocketOffset = FVector(0.0, 0.0, 60.0);

	Camera = CreateDefaultSubobject<UCameraComponent>(TEXT("Camera"));
	Camera->SetupAttachment(SpringArm, USpringArmComponent::SocketName);

	Character = CreateDefaultSubobject<UBox3DCharacterComponent>(TEXT("Box3DCharacter"));

	// The pawn is turned by the controller, not by movement.
	bUseControllerRotationYaw = false;
	AutoPossessPlayer = EAutoReceiveInput::Disabled;
}

void ABox3DCharacterPawn::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
	Super::SetupPlayerInputComponent(PlayerInputComponent);
	if (PlayerInputComponent == nullptr)
	{
		return;
	}

	PlayerInputComponent->BindAxis(TEXT("MoveForward"), this, &ABox3DCharacterPawn::MoveForward);
	PlayerInputComponent->BindAxis(TEXT("MoveRight"), this, &ABox3DCharacterPawn::MoveRight);
	PlayerInputComponent->BindAxis(TEXT("Turn"), this, &APawn::AddControllerYawInput);
	PlayerInputComponent->BindAxis(TEXT("LookUp"), this, &APawn::AddControllerPitchInput);

	PlayerInputComponent->BindAction(TEXT("Jump"), IE_Pressed, this, &ABox3DCharacterPawn::OnJumpPressed);
	PlayerInputComponent->BindAction(TEXT("Sprint"), IE_Pressed, this, &ABox3DCharacterPawn::OnSprintPressed);
	PlayerInputComponent->BindAction(TEXT("Sprint"), IE_Released, this, &ABox3DCharacterPawn::OnSprintReleased);
}

void ABox3DCharacterPawn::MoveForward(float Value)
{
	MoveAxis.X = Value;
}

void ABox3DCharacterPawn::MoveRight(float Value)
{
	MoveAxis.Y = Value;
}

void ABox3DCharacterPawn::OnJumpPressed()
{
	if (Character != nullptr)
	{
		Character->Jump();
	}
}

void ABox3DCharacterPawn::OnSprintPressed()
{
	if (Character != nullptr)
	{
		Character->SetSprinting(true);
	}
}

void ABox3DCharacterPawn::OnSprintReleased()
{
	if (Character != nullptr)
	{
		Character->SetSprinting(false);
	}
}

void ABox3DCharacterPawn::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);

	if (Character == nullptr)
	{
		return;
	}

	// Move relative to where the camera looks, flattened so looking down doesn't slow you.
	const FRotator YawOnly(0.0, GetControlRotation().Yaw, 0.0);
	const FVector Forward = FRotationMatrix(YawOnly).GetUnitAxis(EAxis::X);
	const FVector Right = FRotationMatrix(YawOnly).GetUnitAxis(EAxis::Y);

	Character->SetMoveInput(Forward * MoveAxis.X + Right * MoveAxis.Y);
	MoveAxis = FVector2D::ZeroVector;
}

FVector ABox3DCharacterPawn::GetVelocity() const
{
	return Character != nullptr ? Character->GetVelocity() : FVector::ZeroVector;
}
