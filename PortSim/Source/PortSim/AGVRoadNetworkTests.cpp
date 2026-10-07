#if WITH_DEV_AUTOMATION_TESTS
#include "AGVRoadNetwork.h"
#include "Misc/AutomationTest.h"
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAGVRoadNetworkTest,"PortSim.Traffic.ShortestRoadRoute",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FAGVRoadNetworkTest::RunTest(const FString& Parameters)
{
    FAGVRoadNetwork G;
    G.Add(FVector(0,0,0),FVector(10000,0,0));
    G.Add(FVector(0,0,0),FVector(0,5000,0));
    G.Add(FVector(0,5000,0),FVector(10000,5000,0));
    G.Add(FVector(10000,5000,0),FVector(10000,0,0));
    TArray<FVector> Route;
    auto Clear=[](FVector,FVector){return 0.;};
    TestTrue(TEXT("Connected road has route"),G.Find(FVector::ZeroVector,FVector(10000,0,0),Route,Clear));
    double Length=0; FVector From=FVector::ZeroVector;
    for(FVector P:Route) { Length+=FVector::Distance(From,P); From=P; TestEqual(TEXT("Shortest route stays on direct road"),P.Y,0.); }
    TestEqual(TEXT("Clear traffic selects shortest road distance"),Length,10000.);
    TestTrue(TEXT("Congestion detour exists"),G.Find(FVector::ZeroVector,FVector(10000,0,0),Route,
        [](FVector A,FVector B){return A.Y==0 && B.Y==0?30000.:0.;}));
    TestTrue(TEXT("Occupied direct road uses alternate corridor"),Route[0].Y>0);
    TestFalse(TEXT("No wrong-way travel on directed road"),G.Find(FVector(10000,0,0),FVector::ZeroVector,Route,Clear));
    TestFalse(TEXT("No shortcut from an off-road origin"),G.Find(FVector(5000,2000,0),FVector(10000,0,0),Route,Clear));
    TestTrue(TEXT("Reroute from a split road point"),G.Find(FVector(500,0,0),FVector(10000,0,0),Route,Clear));
    return true;
}
#endif
