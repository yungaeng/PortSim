#if WITH_DEV_AUTOMATION_TESTS
#include "AGVRoadNetwork.h"
#include "AGVTrafficGeometry.h"
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
    TestTrue(TEXT("Sensor-tolerance endpoint remains on road"),
        G.Find(FVector(-.04f,0,0),FVector(10000,0,0),Route,Clear));
    return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAGVSweptChassisTest,"PortSim.Traffic.SweptChassis",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FAGVSweptChassisTest::RunTest(const FString& Parameters)
{
    using AGVTrafficGeometry::SweptChassisIntersects;
    const FVector AlongX(1,0,0);
    TestTrue(TEXT("Parked vehicle blocks a future chord"),SweptChassisIntersects(
        FVector(0,0,0),FVector(10000,0,0),FVector(8000,0,0),AlongX));
    TestFalse(TEXT("Adjacent parking bay clears a straight lane"),SweptChassisIntersects(
        FVector(0,0,0),FVector(10000,0,0),FVector(5000,900,0),AlongX));
    TestFalse(TEXT("Empty AABB corner beside diagonal is not an obstacle"),SweptChassisIntersects(
        FVector(0,0,0),FVector(10000,10000,0),FVector(1000,9000,0),AlongX));
    TestTrue(TEXT("Rotated chassis blocks a diagonal crossing"),SweptChassisIntersects(
        FVector(0,0,0),FVector(10000,10000,0),FVector(5000,5000,0),AlongX));
    TestTrue(TEXT("Rear chassis overhang is protected"),SweptChassisIntersects(
        FVector(0,0,0),FVector(10000,0,0),FVector(-1200,0,0),AlongX));
    TArray<FVector> LaneChange;
    TestTrue(TEXT("Narrow STS lane offset uses non-overlapping S arcs"),AGVTrafficGeometry::LaneChange(
        FVector(0,0,0),FVector(0,3000,0),FVector(907,3000,0),FVector(907,6000,0),LaneChange));
    FVector Previous=FVector(0,0,0);
    for(const FVector& Point:LaneChange)
    {
        TestTrue(TEXT("Lane change always advances longitudinally"),Point.Y>Previous.Y);
        TestTrue(TEXT("Lane change stays between the two lanes"),Point.X>=0 && Point.X<=907.01f);
        Previous=Point;
    }
    TestTrue(TEXT("Lane change rejoins destination lane"),FMath::IsNearlyEqual(LaneChange.Last().X,907.,.01));
    return true;
}
#endif
