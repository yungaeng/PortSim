#if WITH_DEV_AUTOMATION_TESTS
#include "SuspensionLookahead.h"
#include "STSOperatingProfile.h"
#include "Misc/AutomationTest.h"
#include "Misc/Paths.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSuspensionLookaheadTest,"PortSim.Playback.ShortSuspensionLookahead",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FSuspensionLookaheadTest::RunTest(const FString& Parameters)
{
    FSTSOperatingProfile Profile;
    if(!TestTrue(TEXT("Load real operating profile"),Profile.Load(
        FPaths::ProjectDir()/TEXT("../Document/STS/STS_ReferenceData.json"),
        FPaths::ProjectConfigDir()/TEXT("STS_Simulation.json")))) return false;
    FSuspensionLookaheadInput Input; Input.Dynamics=Profile.Dynamics; Input.PowerW=Profile.HoistPowerW;
    FSTSSuspension Initial; Initial.Angle.X=.03;
    FSuspensionLookahead Preview; Preview.SetRequestedRate(8); Preview.Reset(Initial,Input,.5,.1);
    Preview.Tick(0,30);
    TestEqual(TEXT("Prediction does not advance displayed time"),Preview.DisplayedSeconds(),0.);
    TestEqual(TEXT("Half second is ready"),Preview.BufferedSeconds(),.5);
    TestTrue(TEXT("Prediction leaves displayed sway untouched"),Preview.DisplayedState().Angle==Initial.Angle);
    Preview.Tick(1./60.,0);
    FSTSSuspension Reference=Initial;
    for(int32 I=0;I<8;++I) Reference.Step(Input.Dynamics,1./60.,Input.LengthM,0,
        Input.Acceleration,Input.Velocity,Input.MassKg,Input.CoG,Input.PowerW);
    TestTrue(TEXT("8x replay matches the same eight 1x physics steps"),
        Preview.DisplayedState().Angle.Equals(Reference.Angle,1.e-12) &&
        Preview.DisplayedState().Rate.Equals(Reference.Rate,1.e-12));
    Input.Dynamics.Wind=FVector(5,0,0);
    const double Before=Preview.DisplayedSeconds();
    Preview.SubmitInput(Input);
    TestEqual(TEXT("Input invalidates all old future samples"),Preview.BufferedSeconds(),0.);
    TestEqual(TEXT("Input retains requested rate"),Preview.RequestedRate(),8.);
    Preview.Tick(1./60.,30);
    Reference.Step(Input.Dynamics,1./60.,Input.LengthM,0,Input.Acceleration,
        Input.Velocity,Input.MassKg,Input.CoG,Input.PowerW);
    TestTrue(TEXT("Input resumes at displayed state with one 1x step"),
        FMath::IsNearlyEqual(Preview.DisplayedSeconds()-Before,1./60.,1.e-12) &&
        Preview.DisplayedState().Angle.Equals(Reference.Angle,1.e-12));
    for(int32 I=0;I<7;++I) Preview.Tick(1./60.,30);
    Preview.Tick(0,30);
    TestTrue(TEXT("Quiet period returns to buffered replay"),Preview.Mode()==ESuspensionLookaheadMode::Replay);
    Preview.Tick(.25,0);
    TestTrue(TEXT("Exhaustion waits instead of inventing future physics"),
        Preview.Mode()==ESuspensionLookaheadMode::Preparing && Preview.BufferedSeconds()==0);
    const double Exhausted=Preview.DisplayedSeconds();
    Preview.Tick(.25,0);
    TestEqual(TEXT("Preparation does not accrue playback debt"),Preview.DisplayedSeconds(),Exhausted);
    Preview.SubmitInput(Input);
    Preview.SubmitInput(Input);
    TestTrue(TEXT("Repeated input stays live and clears future"),
        Preview.Mode()==ESuspensionLookaheadMode::Live && Preview.BufferedSeconds()==0);
    return true;
}
#endif
