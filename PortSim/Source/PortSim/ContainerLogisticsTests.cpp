#if WITH_DEV_AUTOMATION_TESTS
#include "ContainerSpecification.h"
#include "AGVDispatchPolicy.h"
#include "SpreaderTelescope.h"
#include "Misc/AutomationTest.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FContainerSpecificationTest,"PortSim.Logistics.ContainerSpecifications",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FContainerSpecificationTest::RunTest(const FString& Parameters)
{
    TMap<int32,int32> Counts;
    for(int32 ID=2000;ID<2600;++ID)
    {
        const auto S=PortContainerSpecification::ForSequence(ID);
        Counts.FindOrAdd(S.LengthFt)++;
        TestTrue(TEXT("Gross mass includes positive tare and cargo"),S.TareMassKg>0 && S.CargoMassKg>0 && S.GrossMassKg()<30480.f);
        TestTrue(TEXT("ISO length class has matching external length"),
            (S.LengthFt==20 && FMath::IsNearlyEqual(S.LengthCm,605.8f)) ||
            (S.LengthFt==40 && FMath::IsNearlyEqual(S.LengthCm,1219.2f)) ||
            (S.LengthFt==45 && FMath::IsNearlyEqual(S.LengthCm,1371.6f)));
    }
    TestEqual(TEXT("20 ft share"),Counts.FindRef(20),180);
    TestEqual(TEXT("40 ft share"),Counts.FindRef(40),300);
    TestEqual(TEXT("45 ft share"),Counts.FindRef(45),120);
    float Length=PortSpreaderTelescope::Length20Cm;
    for(int32 Step=0;Step<299;++Step)
        Length=PortSpreaderTelescope::Advance(Length,PortSpreaderTelescope::Length45Cm,.1f);
    TestFalse(TEXT("20 to 45 ft stroke does not finish before the public 30 second reference"),
        PortSpreaderTelescope::IsReady(Length,PortSpreaderTelescope::Length45Cm));
    Length=PortSpreaderTelescope::Advance(Length,PortSpreaderTelescope::Length45Cm,.1f);
    TestTrue(TEXT("20 to 45 ft stroke reaches its target in approximately 30 seconds"),
        PortSpreaderTelescope::IsReady(Length,PortSpreaderTelescope::Length45Cm));

    const float C20=PortSpreaderTelescope::TwistlockHalfLengthCm(PortSpreaderTelescope::Length20Cm);
    const float C40=PortSpreaderTelescope::TwistlockHalfLengthCm(PortSpreaderTelescope::Length40Cm);
    const float C45=PortSpreaderTelescope::TwistlockHalfLengthCm(PortSpreaderTelescope::Length45Cm);
    TestTrue(TEXT("Twistlock end beams have distinct ordered positions for 20/40/45 ft"),C20<C40 && C40<C45);
    TestTrue(TEXT("20 ft longitudinal twistlock centre spacing is approximately 5.852 m"),FMath::IsNearlyEqual(C20*2.f,585.2f,.2f));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAGVDispatchPolicyTest,"PortSim.Logistics.CostAwareAGVDispatch",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FAGVDispatchPolicyTest::RunTest(const FString& Parameters)
{
    const FVector Target(4500,0,0);
    const double NearUsed=AGVDispatchPolicy::Score(FVector(6500,0,0),Target,6,5);
    const double FarUnused=AGVDispatchPolicy::Score(FVector(6500,20000,0),Target,5,5);
    TestTrue(TEXT("A very distant unused AGV does not beat a nearby vehicle solely for equal work counts"),NearUsed<FarUnused);

    const double NearUsedClose=AGVDispatchPolicy::Score(FVector(6500,0,0),Target,6,5);
    const double NearUnused=AGVDispatchPolicy::Score(FVector(6500,3000,0),Target,5,5);
    TestTrue(TEXT("Fairness resolves close ETA choices toward the less-used AGV"),NearUnused<NearUsedClose);
    return true;
}
#endif
