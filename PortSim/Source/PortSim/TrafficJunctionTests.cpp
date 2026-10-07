#if WITH_DEV_AUTOMATION_TESTS
#include "TrafficJunctionPolicy.h"
#include "TrafficSpatialIndex.h"
#include "Misc/AutomationTest.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTrafficJunctionTest,"PortSim.Traffic.JunctionAdmission",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FTrafficJunctionTest::RunTest(const FString& Parameters)
{
    const TArray<FVector> Route={FVector(0,0,0),FVector(900,0,0),FVector(900,900,0),FVector(900,2600,0)};
    TArray<FBox> Exit;
    TrafficJunctionPolicy::AppendExitCorridor(Exit,Route,0,Route.Num(),[](int32){return FVector(730,210,250);});
    TestEqual(TEXT("Dogleg length alone does not clear junction"),Exit.Num(),3);
    FTrafficSpatialIndex Occupancy;
    Occupancy.Set(4,{FBox(FVector(800,2400,-100),FVector(1000,2800,100))});
    TestTrue(TEXT("Occupied downstream exit blocks admission before entering junction"),Occupancy.Query(Exit).Contains(4));
    Occupancy.Remove(4);
    TestTrue(TEXT("Cleared exit permits admission"),Occupancy.Query(Exit).IsEmpty());
    Exit.Reset();
    TrafficJunctionPolicy::AppendExitCorridor(Exit,Route,0,2,[](int32){return FVector(730,210,250);});
    TestEqual(TEXT("Staging stop never claims unowned crane handover"),Exit.Num(),1);
    const double Old=10,New=20;
    TestTrue(TEXT("Longer road wait beats vehicle ID"),TrafficJunctionPolicy::OlderRequestFirst(&Old,&New,200,100));
    TestFalse(TEXT("New request cannot bypass waiting vehicle"),TrafficJunctionPolicy::OlderRequestFirst(nullptr,&Old,1,200));
    TestTrue(TEXT("Equal waiting times have deterministic order"),TrafficJunctionPolicy::OlderRequestFirst(&Old,&Old,100,200));
    return true;
}
#endif
