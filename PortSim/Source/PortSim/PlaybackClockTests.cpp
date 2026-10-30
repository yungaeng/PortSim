#if WITH_DEV_AUTOMATION_TESTS
#include "PortSimTimeStep.h"
#include "HAL/PlatformProcess.h"
#include "Misc/App.h"
#include "Misc/AutomationTest.h"
#include "Misc/ScopeExit.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlaybackFixedWorldStepTest,"PortSim.Playback.FixedWorldStep",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FPlaybackFixedWorldStepTest::RunTest(const FString& Parameters)
{
    const double Current=FApp::GetCurrentTime(),Last=FApp::GetLastTime(),Delta=FApp::GetDeltaTime();
    ON_SCOPE_EXIT
    {
        FApp::SetCurrentTime(Last);
        FApp::UpdateLastTime();
        FApp::SetCurrentTime(Current);
        FApp::SetDeltaTime(Delta);
    };
    auto* Clock=NewObject<UPortSimTimeStep>();
    Clock->Initialize(nullptr);
    // Include transitions in both directions and an overloaded frame. Neither
    // requested speed nor delayed presentation may change the physics interval.
    for(float Rate:{1.f,2.f,4.f,8.f,16.f,8.f,1.f})
    {
        Clock->SetPlaybackRate(Rate);
        for(int32 Frame=0;Frame<3;++Frame)
        {
            if(Frame==1) FPlatformProcess::SleepNoStats(.02f);
            const double Before=FApp::GetCurrentTime();
            TestFalse(TEXT("Custom clock supplies the application time"),Clock->UpdateTimeStep(nullptr));
            TestEqual(TEXT("World delta is independent of speed and stalls"),FApp::GetDeltaTime(),UPortSimTimeStep::StepSeconds);
            TestTrue(TEXT("Application time advances by one fixed world step"),
                FMath::Abs(FApp::GetCurrentTime()-Before-UPortSimTimeStep::StepSeconds)<1.e-6);
        }
    }
    Clock->Shutdown(nullptr);
    TestTrue(TEXT("Engine time resumes after shutdown"),Clock->UpdateTimeStep(nullptr));
    FPlaybackInputGate Gate;
    Gate.Submit(10.);
    TestEqual(TEXT("External input forces 1x"),Gate.EffectiveRate(8.f,10.),1.f);
    Gate.Submit(11.);
    TestEqual(TEXT("Repeated input extends quiet period"),Gate.EffectiveRate(8.f,12.5),1.f);
    TestEqual(TEXT("Requested rate returns after wall-time quiet period"),Gate.EffectiveRate(8.f,13.),8.f);
    TestEqual(TEXT("Every input changes generation"),Gate.Revision,uint64(2));
    Clock->Initialize(nullptr); Clock->SetPlaybackRate(8.f); Clock->NotifyExternalInput();
    TestEqual(TEXT("Live clock applies input hold immediately"),Clock->GetEffectiveRate(),1.f);
    Clock->UpdateTimeStep(nullptr);
    TestEqual(TEXT("Input hold retains fixed physical step"),FApp::GetDeltaTime(),UPortSimTimeStep::StepSeconds);
    Clock->Shutdown(nullptr);
    return true;
}
#endif
