#pragma once
#include "Components/ActorComponent.h"
#include "ProceduralMeshComponent.h"
#include "PortWaterVisualComponent.generated.h"

class UGameViewportClient;
class UMaterialInstanceDynamic;
class UPortEnvironmentComponent;
class UProceduralMeshComponent;

UCLASS()
class UPortWaterSurfaceComponent : public UProceduralMeshComponent
{
    GENERATED_BODY()
public:
    FBoxSphereBounds CalcBounds(const FTransform& LocalToWorld) const override
    { return Super::CalcBounds(LocalToWorld).ExpandBy(100.f); }
};

/** GPU-only surface motion, presented at viewport cadence using the sea model's clock. */
UCLASS()
class UPortWaterVisualComponent : public UActorComponent
{
    GENERATED_BODY()
private:
    void BeginPlay() override;
    void EndPlay(EEndPlayReason::Type Reason) override;
    void UpdateForDraw();
    void BuildSurface();
    UPROPERTY(Transient) TObjectPtr<UPortEnvironmentComponent> Environment;
    UPROPERTY(Transient) TObjectPtr<UProceduralMeshComponent> Surface;
    UPROPERTY(Transient) TObjectPtr<UMaterialInstanceDynamic> Material;
    TWeakObjectPtr<UGameViewportClient> Viewport;
    int32 CurrentLOD=INDEX_NONE;
};
