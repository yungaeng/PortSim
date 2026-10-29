#pragma once
#include "CoreMinimal.h"
#include "Engine/EngineCustomTimeStep.h"
#include "PortSimTimeStep.generated.h"

/** Fixed world interval; playback rate controls wall-clock tick pacing only. */
UCLASS()
class PORTSIM_API UPortSimTimeStep : public UEngineCustomTimeStep
{
    GENERATED_BODY()
public:
    static constexpr double StepSeconds=1.0/60.0;
    static constexpr double MaxSimulationDelta=2.0;
    virtual bool Initialize(UEngine* InEngine) override;
    virtual void Shutdown(UEngine* InEngine) override;
    virtual bool UpdateTimeStep(UEngine* InEngine) override;
    virtual ECustomTimeStepSynchronizationState GetSynchronizationState() const override;
    void SetPlaybackRate(float Rate);
    float GetAchievedRate() const { return AchievedRate; }
private:
    bool bInitialized=false;
    float PlaybackRate=1.f;
    float AchievedRate=1.f;
    double NextFrameWall=0;
    double SampleStart=0;
    double SampleSimulation=0;
};
