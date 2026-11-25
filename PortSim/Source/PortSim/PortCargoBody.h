#pragma once
#include "CoreMinimal.h"
#include "Components/StaticMeshComponent.h"
#include "PortCargoBody.generated.h"

// Catch physics and attachment changes even when made through GetBody().
UCLASS()
class PORTSIM_API UPortCargoBody : public UStaticMeshComponent
{
    GENERATED_BODY()
public:
    virtual void SetSimulatePhysics(bool Enabled) override;
protected:
    virtual void OnAttachmentChanged() override;
    virtual void OnUpdateTransform(EUpdateTransformFlags UpdateTransformFlags,ETeleportType Teleport) override;
};
