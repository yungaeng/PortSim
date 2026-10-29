#pragma once
#include "Engine/GameEngine.h"
#include "PortSimGameEngine.generated.h"

/** Render pacing is independent of the fixed world/Chaos interval. */
UCLASS()
class PORTSIM_API UPortSimGameEngine : public UGameEngine
{
    GENERATED_BODY()
public:
    virtual void RedrawViewports(bool bShouldPresent=true) override;
private:
    double NextPresentationWall=0;
};
