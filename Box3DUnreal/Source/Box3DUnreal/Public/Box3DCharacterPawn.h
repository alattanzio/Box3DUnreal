// Author: Antonio Lattanzio - emptyvessel

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "Box3DCharacterPawn.generated.h"

class UBox3DCharacterComponent;
class UCameraComponent;
class USpringArmComponent;

/**
 * Playable pawn wrapping UBox3DCharacterComponent, with a third-person camera and
 * axis/action bindings. A working example, and what box3d.CharacterDemo possesses.
 *
 * Bindings use the legacy input axes (MoveForward/MoveRight/Turn/LookUp/Jump) so it works
 * in a fresh project with no Enhanced Input assets to author.
 */
UCLASS(meta = (DisplayName = "Box3D Character Pawn"))
class BOX3DUNREAL_API ABox3DCharacterPawn : public APawn
{
	GENERATED_BODY()

public:
	ABox3DCharacterPawn();

	virtual void SetupPlayerInputComponent(UInputComponent* PlayerInputComponent) override;
	virtual void Tick(float DeltaSeconds) override;
	virtual FVector GetVelocity() const override;

	UBox3DCharacterComponent* GetCharacter() const { return Character; }

protected:
	void MoveForward(float Value);
	void MoveRight(float Value);
	void OnJumpPressed();
	void OnSprintPressed();
	void OnSprintReleased();

private:
	UPROPERTY(VisibleAnywhere, Category = "Box3D")
	TObjectPtr<UBox3DCharacterComponent> Character = nullptr;

	UPROPERTY(VisibleAnywhere, Category = "Box3D")
	TObjectPtr<USpringArmComponent> SpringArm = nullptr;

	UPROPERTY(VisibleAnywhere, Category = "Box3D")
	TObjectPtr<UCameraComponent> Camera = nullptr;

	/** Accumulated this frame, handed to the character as one vector. */
	FVector2D MoveAxis = FVector2D::ZeroVector;
};
