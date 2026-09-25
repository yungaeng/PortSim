#if WITH_DEV_AUTOMATION_TESTS
#include "STSOperatingProfile.h"
#include "Misc/AutomationTest.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSTSPickupTest,"PortSim.STS.SensorPickup",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FSTSPickupTest::RunTest(const FString& Parameters)
{
    FSTSPickupConfig C;FSTSObservation O;O.bValid=O.bTargetVisible=true;
    O.SpreaderPosition=FVector(0,0,154.5);
    for(bool& Contact:O.CornerSeated)Contact=true;
    FSTSPickupController P;double Now=0;
    auto Tick=[&](bool Fresh=true){Now+=.1;if(Fresh)O.Timestamp=Now;P.Update(C,O,Now,.1,.5,FVector::ZeroVector,65000);};
    O.CornerSeated[2]=false;
    for(int I=0;I<20;++I)Tick();
    TestTrue(TEXT("Three contacts cannot request any lock"),P.Phase==ESTSPickupPhase::Align&&!P.RequestLocks[0]);
    O.CornerSeated[2]=true;
    for(int I=0;I<12;++I)Tick();
    TestTrue(TEXT("Lock command is not lock feedback"),P.Phase==ESTSPickupPhase::Lock&&P.RequestLocks[0]);
    for(bool& Lock:O.Locked)Lock=true;
    Tick();TestTrue(TEXT("Four lock feedbacks allow attachment"),P.Phase==ESTSPickupPhase::Attach);
    P.Attached(O.SpreaderPosition);O.SpreaderPosition.Z+=C.TrialHeight;
    for(float& Force:O.CornerLoadsN)Force=12000*9.80665/4;
    O.bCargoSupported=true;
    for(int I=0;I<20;++I)Tick();
    TestFalse(TEXT("Supported cargo cannot pass trial"),P.EstimateValid);
    O.bCargoSupported=false;O.CornerLoadsN[0]=0;
    for(int I=0;I<20;++I)Tick();
    TestFalse(TEXT("Unloaded corner blocks full hoist"),P.EstimateValid);
    for(float& Force:O.CornerLoadsN)Force=12000*9.80665/4;
    for(int I=0;I<14;++I)Tick();
    TestTrue(TEXT("Stable suspended load permits full hoist"),P.EstimateValid);
    TestTrue(TEXT("Mass derived from measurements"),FMath::Abs(P.EstimatedMass-12000)<1);

    P=FSTSPickupController();P.Attached(FVector(0,0,154.5));Tick();
    const double Before=P.StableTime;
    for(int I=0;I<3;++I)Tick(false);
    TestEqual(TEXT("Repeated samples cannot accumulate proof time"),P.StableTime,Before);
    for(int I=0;I<4;++I)Tick(false);
    TestTrue(TEXT("Stale sample faults rather than succeeds"),!P.Fault.IsEmpty()&&!P.EstimateValid);
    return true;
}
#endif
