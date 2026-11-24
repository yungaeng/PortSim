#if WITH_DEV_AUTOMATION_TESTS
#include "YardCargoQuota.h"
#include "YardStackingPolicy.h"
#include "Misc/AutomationTest.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FYardCargoQuotaTest,"PortSim.Logistics.YardCargoQuota",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FYardCargoQuotaTest::RunTest(const FString& Parameters)
{
    TArray<int32> Quota;
    const TArray<int32> Capacity={1800,300,100,100};
    for(int32 Count:{0,1,600,2299,2300})
    {
        TestTrue(TEXT("Feasible total is apportioned"),YardCargoQuota::Allocate(Capacity,Count,Quota));
        int32 Sum=0;
        for(int32 I=0;I<4;++I)
        {
            Sum+=Quota[I];
            TestTrue(TEXT("Category never exceeds capacity"),Quota[I]>=0 && Quota[I]<=Capacity[I]);
            TestTrue(TEXT("Category follows actual capacity ratio"),FMath::Abs(double(Quota[I])-double(Count)*Capacity[I]/2300.)<1.);
        }
        TestEqual(TEXT("No lost bookings"),Sum,Count);
    }
    TestFalse(TEXT("Excess total rejected"),YardCargoQuota::Allocate(Capacity,2301,Quota));
    TestFalse(TEXT("Explicit bookings exceeding category rejected"),YardCargoQuota::Allocate({10,-1,3,4},1,Quota));
    TestTrue(TEXT("Unavailable categories receive nothing"),YardCargoQuota::Allocate({7,0,0,0},7,Quota));
    TestEqual(TEXT("All bookings use available category"),Quota[0],7);
    TestTrue(TEXT("Empty yard permits zero bookings"),YardCargoQuota::Allocate({0,0,0,0},0,Quota));
    TestFalse(TEXT("Empty yard rejects bookings"),YardCargoQuota::Allocate({0,0,0,0},1,Quota));
    auto Lower=FPortCargoInformation::ForSequence(3,40);
    auto Upper=Lower;
    TestTrue(TEXT("Compatible supported corrosive stack accepted"),YardStackingPolicy::MayStack(Lower,40,25000,Upper,40,20000));
    TestFalse(TEXT("Quota does not waive weight ordering"),YardStackingPolicy::MayStack(Lower,40,19000,Upper,40,20000));
    Upper.OutboundOrder=Lower.OutboundOrder+1;
    TestFalse(TEXT("Quota does not waive retrieval order"),YardStackingPolicy::MayStack(Lower,40,25000,Upper,40,20000));
    Upper=Lower;
    TestFalse(TEXT("Quota does not waive cumulative stack limit"),YardStackingPolicy::MayStack(Lower,40,25000,Upper,40,20000,Lower.PermittedSuperimposedKg));
    TestFalse(TEXT("Corrosive cargo cannot spill into dry zone"),YardStackingPolicy::ZoneAccepts(0,Lower));
    return true;
}
#endif
