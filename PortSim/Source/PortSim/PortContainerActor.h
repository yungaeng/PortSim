#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "ContainerSpecification.h"
#include "SpreaderTelescope.h"
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
    void ConfigureSpecification(const FPortContainerSpecification& Specification, FVector CoGOffset=FVector::ZeroVector);
    void ResetCargo(FVector Position);
    void ApplyContainerAppearance();
    void SetPhysicalParameters(float Mass, FVector CoGOffset);
    UStaticMeshComponent* GetBody() const { return Body; }

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Container") FName ContainerID;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Container") EPortContainerSize SizeClass=EPortContainerSize::Ft40;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Container") int32 LengthFt=40;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Container") FVector ExternalDimensionsCm=FVector(243.8f,1219.2f,259.1f);
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Container") float TareMassKg=3800.f;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Container") float CargoMassKg=8200.f;
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Container", meta=(ClampMin="1")) float MassKg=12000.f;
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Container") FVector CoGOffsetCm=FVector::ZeroVector;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Container") ECargoOwner LocationOwner=ECargoOwner::Ship;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Container") TObjectPtr<UStaticMeshComponent> Body;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Container") TObjectPtr<UStaticMeshComponent> Visual;
    float SensorHalfLengthCm() const { return PortSpreaderTelescope::TwistlockHalfLengthCm(ExternalDimensionsCm.Y); }
    float SensorHalfWidthCm() const { return PortSpreaderTelescope::TwistlockHalfWidthCm(ExternalDimensionsCm.X); }
};
