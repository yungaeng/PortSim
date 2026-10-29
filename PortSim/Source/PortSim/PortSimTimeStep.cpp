#include "PortSimTimeStep.h"
#include "HAL/PlatformProcess.h"
#include "HAL/PlatformTime.h"
#include "Misc/App.h"

bool UPortSimTimeStep::Initialize(UEngine* InEngine)
{
    bInitialized=true;
    SampleStart=FPlatformTime::Seconds();
    NextFrameWall=SampleStart+StepSeconds/PlaybackRate;
    SampleSimulation=0;
    return true;
}

void UPortSimTimeStep::Shutdown(UEngine* InEngine)
{
    bInitialized=false;
}

void UPortSimTimeStep::SetPlaybackRate(float Rate)
{
    const float NewRate=FMath::IsFinite(Rate)?FMath::Clamp(Rate,1.f,16.f):1.f;
    if(NewRate==PlaybackRate) return;
    PlaybackRate=NewRate;
    NextFrameWall=FPlatformTime::Seconds()+StepSeconds/PlaybackRate;
}

ECustomTimeStepSynchronizationState UPortSimTimeStep::GetSynchronizationState() const
{
    return bInitialized?ECustomTimeStepSynchronizationState::Synchronized:ECustomTimeStepSynchronizationState::Closed;
}

bool UPortSimTimeStep::UpdateTimeStep(UEngine* InEngine)
{
    if (!bInitialized) return true;
    // Advance controllers, environment and Chaos with the same interval at every
    // playback rate. Plant-only substeps cannot correct observations/kinematic
    // targets held across an enlarged world frame (visible as extra rope sway).
    const double Deadline=NextFrameWall;
    double Now=FPlatformTime::Seconds();
    if (Now<Deadline)
    {
        FPlatformProcess::SleepNoStats(static_cast<float>(Deadline-Now));
        Now=FPlatformTime::Seconds();
    }
    // Under load, report a lower achieved rate instead of enlarging the physics
    // interval or catching up with a burst after an editor/debugger stall.
    const double SimulationDelta=StepSeconds;
    // Preserve cadence across sleep/scheduler jitter rather than adding that
    // overshoot to every interval. Bound lag to one tick: no stall catch-up queue.
    NextFrameWall=FMath::Max(Deadline+StepSeconds/PlaybackRate,Now);
    UpdateApplicationLastTime();
    FApp::SetDeltaTime(SimulationDelta);
    FApp::SetCurrentTime(FApp::GetLastTime()+SimulationDelta);
    SampleSimulation+=SimulationDelta;
    if (Now-SampleStart>=0.5)
    {
        AchievedRate=static_cast<float>(SampleSimulation/(Now-SampleStart));
        SampleStart=Now; SampleSimulation=0;
    }
    return false;
}
