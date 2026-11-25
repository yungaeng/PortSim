#if WITH_DEV_AUTOMATION_TESTS
#include "AGVDispatchDependencies.h"
#include "AGVDispatchPolicy.h"
#include "Misc/AutomationTest.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAGVDependencyDispatchTest,"PortSim.Logistics.SupportPathDispatch",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FAGVDependencyDispatchTest::RunTest(const FString& Parameters)
{
    TArray<AGVDispatchDependencies::FPriority> Priority;
    const TArray<int32> Rank={0,1,2,3,4,5,6};
    const TArray<FIntPoint> Edges={{0,1},{1,2},{2,3},{4,5},{5,6}};
    TestTrue(TEXT("Support chains respect the dependency-safe plan"),AGVDispatchDependencies::Build(Rank,Edges,7,Priority));
    TestEqual(TEXT("Foundations carry the complete critical path"),Priority[0].PathLength,3);
    TestEqual(TEXT("The final upper box retains its chain priority"),Priority[3].PathLength,3);
    TestEqual(TEXT("Unrelated shorter stacks have lower urgency"),Priority[4].PathLength,2);
    TestEqual(TEXT("Foundations count the work they unlock"),Priority[0].Unlocks,3);
    TestTrue(TEXT("A bounded run excludes later bookings from priority"),AGVDispatchDependencies::Build(Rank,Edges,5,Priority));
    TestEqual(TEXT("Outside-prefix children cannot inflate an active request"),Priority[4].PathLength,0);
    TestTrue(TEXT("Shared two-pad support graph"),AGVDispatchDependencies::Build({0,1,2,3},
        {{0,2},{1,2},{2,3},{0,3}},4,Priority));
    TestEqual(TEXT("Shared descendants are counted once"),Priority[0].Unlocks,2);
    TestFalse(TEXT("Cyclic or backwards support plans are rejected"),AGVDispatchDependencies::Build({0,1},{{1,0}},2,Priority));
    TArray<AGVDispatchPolicy::FRequest> Requests={{10,0,0,true,true,0,0},{11,8,0,true,true,3,0}};
    TestEqual(TEXT("A ready final critical box beats unrelated same-age work"),AGVDispatchPolicy::SelectExecutable(Requests),11);
    Requests[1].YardReady=false;
    TestEqual(TEXT("Critical work cannot bypass a missing support or reserved turn"),AGVDispatchPolicy::SelectExecutable(Requests),10);
    Requests[1].YardReady=true; Requests[1].SourceReady=false;
    TestEqual(TEXT("Critical work cannot pick through another ship box"),AGVDispatchPolicy::SelectExecutable(Requests),10);
    Requests[1].SourceReady=true; Requests[1].EligibleAt=2000; Requests[1].CriticalPath=8;
    TestEqual(TEXT("Capped priority lets older work beat a newly eligible long chain"),AGVDispatchPolicy::SelectExecutable(Requests),10);
    TestEqual(TEXT("Urgency is capped, not multiplied without bound"),AGVDispatchPolicy::ReadyPriority(0,100),960.);
    Requests[1].CriticalPath=0;
    Requests[1].DedicatedHazardYard=true;
    TestEqual(TEXT("Dedicated yard work starts while general cargo can use other blocks"),AGVDispatchPolicy::SelectExecutable(Requests),11);
    Requests[1].YardReady=false;
    TestEqual(TEXT("Dedicated yard priority never bypasses admission"),AGVDispatchPolicy::SelectExecutable(Requests),10);
    Requests[1].YardReady=true;
    Requests[1].EligibleAt=4000;
    TestEqual(TEXT("General cargo can age past the dedicated yard lead"),AGVDispatchPolicy::SelectExecutable(Requests),10);
    return true;
}
#endif
