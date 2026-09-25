#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "PortContainerActor.generated.h"

class UStaticMeshComponent;
class UTextRenderComponent;

UENUM(BlueprintType)
enum class ECargoOwner : uint8 { Ship, STS, AGV, RMG, Yard };

UCLASS(Blueprintable)
class PORTSIM_API APortContainerActor : public AActor
{
    GENERATED_BODY()
public:
    APortContainerActor();
    virtual void BeginPlay() override;
    void InitializeContainer(int32 Number);
    void ResetCargo(FVector Position);
    void ApplyContainerAppearance();
    void SetPhysicalParameters(float Mass, FVector CoGOffset);
    UStaticMeshComponent* GetBody() const { return Body; }

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Container") FName ContainerID;
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Container", meta=(ClampMin="1")) float MassKg=12000.f;
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Container") FVector CoGOffsetCm=FVector::ZeroVector;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Container") ECargoOwner LocationOwner=ECargoOwner::Ship;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Container") TObjectPtr<UStaticMeshComponent> Body;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Container") TObjectPtr<UStaticMeshComponent> Visual;
};
