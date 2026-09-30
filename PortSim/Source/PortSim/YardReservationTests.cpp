#if WITH_DEV_AUTOMATION_TESTS
#include "YardReservation.h"
#include "TerminalLayout.h"
#include "Misc/AutomationTest.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FYardReservationTest,"PortSim.Logistics.ExactReceivingCapacity",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FYardReservationTest::RunTest(const FString& Parameters)
{
    TestEqual(TEXT("Three equal 200-container vessels"),TerminalLayout::TotalVesselContainers,600);
    const TArray<int32> Sizes={4,3,2,4,3,2,1};
    int32 Total=0;
    for (int32 Crane=0;Crane<46;++Crane)
    {
        const int32 Quota=600/46+(Crane<600%46?1:0);
        TArray<int32> Selected;
        TestTrue(TEXT("Exact per-crane capacity is reachable"),SelectWholeYardStacks(Sizes,Quota,Selected));
        TSet<int32> Unique; int32 Count=0;
        for (int32 Stack:Selected) { TestFalse(TEXT("No stack reserved twice"),Unique.Contains(Stack)); Unique.Add(Stack); Count+=Sizes[Stack]; }
        TestEqual(TEXT("No excess or missing receiving slots"),Count,Quota); Total+=Count;
    }
    TestEqual(TEXT("All RMG quotas sum to exactly 600"),Total,600);
    TArray<int32> Selected;
    TestFalse(TEXT("Impossible capacity fails without a partial reservation"),SelectWholeYardStacks({2,4},3,Selected));
    TestTrue(TEXT("No partial output"),Selected.IsEmpty());
    TestTrue(TEXT("Empty reservation is supported"),SelectWholeYardStacks({},0,Selected));
    return true;
}
#endif
