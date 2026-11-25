#if WITH_DEV_AUTOMATION_TESTS
#include "PortSiteLogistics.h"
#include "PortWorkingCrane.h"
#include "PortAGVActor.h"
#include "QuayCrane.h"
#include "Dom/JsonObject.h"
#include "Engine/World.h"
#include "Misc/AutomationTest.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FEquipmentPerformanceTest,"PortSim.Metrics.EquipmentAndBerthPerformance",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FEquipmentPerformanceTest::RunTest(const FString& Parameters)
{
    const auto Init=UWorld::InitializationValues().AllowAudioPlayback(false).CreatePhysicsScene(true)
        .RequiresHitProxies(false).CreateNavigation(false).CreateAISystem(false);
    auto* World=UWorld::CreateWorld(EWorldType::Game,false,NAME_None,nullptr,true,ERHIFeatureLevel::Num,&Init);
    auto* Controller=World->SpawnActor<AQuayCrane>();Controller->bAutoRunning=true;
    auto* Site=World->SpawnActor<APortSiteLogistics>();Site->SetOwner(Controller);Site->LaneCount=9;Site->YardCraneCount=0;
    for(int I=0;I<3;++I)
    {
        auto* K=World->SpawnActor<APortWorkingCrane>();K->bSTS=true;K->bConfigured=K->bJobActive=true;
        K->Stage=3;K->Observation.bCraneClear=true;Site->Equipment.Add(K);
    }
    auto* K=Site->Equipment[0].Get();
    auto* V=World->SpawnActor<APortAGVActor>();V->InitializeVehicle(100);V->SetFMSHold(false);V->Speed=100;
    Site->Vehicles={V};Site->Jobs.SetNum(1);Site->Jobs[0].Stage=3;
    Site->IncidentMeasureClock=10;Site->AdvanceEquipmentPerformance(2,false);
    TestEqual(TEXT("Active crane operation accumulates work time"),Site->EquipmentWorkingSeconds.FindRef(K->GetName()),2.);
    TestEqual(TEXT("AGV transport accumulates work time"),Site->EquipmentWorkingSeconds.FindRef(V->GetName()),2.);
    Site->AdvanceEquipmentPerformance(5,true);
    K->Stage=5;K->bDestinationReady=false;V->SetFMSHold(true);Site->AdvanceEquipmentPerformance(3,false);
    TestEqual(TEXT("Paused and AGV-wait intervals are excluded from crane work"),Site->EquipmentWorkingSeconds.FindRef(K->GetName()),2.);
    TestEqual(TEXT("FMS waiting is excluded from AGV work"),Site->EquipmentWorkingSeconds.FindRef(V->GetName()),2.);
    K->Stage=3;K->bDestinationReady=true;K->Fault=TEXT("equipment fault");V->SetDriveFailure(true);
    Site->AdvanceEquipmentPerformance(3,false);
    TestEqual(TEXT("Fault time is excluded from actual work"),Site->EquipmentWorkingSeconds.FindRef(K->GetName()),2.);
    K->Fault.Empty();Controller->bAutoPaused=true;Site->AdvanceEquipmentPerformance(50,false);Controller->bAutoPaused=false;
    TestEqual(TEXT("Manual pause never accumulates work"),Site->EquipmentWorkingSeconds.FindRef(K->GetName()),2.);
    Site->IncidentMeasureClock=3600;Site->EquipmentWorkingSeconds.Add(K->GetName(),1800);K->CompletedJobs=30;
    const auto Crane=Site->CaptureEquipmentPerformance(K);
    TestEqual(TEXT("Actual work divided by gross observation gives utilization"),Crane->GetNumberField(TEXT("utilization_percent")),50.);
    TestEqual(TEXT("Crane productivity uses operation hours, not active hours"),Crane->GetNumberField(TEXT("crane_containers_per_hour")),30.);
    Site->Manifest.SetNum(4);Site->ExportCargo.SetNum(1);
    for(int I=0;I<4;++I){auto& C=Site->Manifest[I];C.ID=I;C.STS=C.OriginalSTS=I==3?3:0;C.State=1;}
    Site->ExportCargo[0].STS=Site->ExportCargo[0].OriginalSTS=0;
    Site->IncidentMeasureClock=100;Site->AdvanceEquipmentPerformance(.05,true);
    Site->Manifest[0].HandoverMask=Site->Manifest[1].HandoverMask=1;
    Site->Manifest[1].STS=3; // Reassignment cannot move cargo to another original berth's statistics.
    Site->IncidentMeasureClock=700;Site->AdvanceEquipmentPerformance(.05,true);
    auto Berth=Site->CaptureBerthPerformance(0);
    TestEqual(TEXT("Two containers of any size remain two boxes, not TEU"),Berth->GetNumberField(TEXT("handled_containers")),2.);
    TestEqual(TEXT("Berth productivity uses one shared elapsed interval"),Berth->GetNumberField(TEXT("containers_per_berth_hour")),12.);
    const auto Peer=Site->CaptureEquipmentPerformance(Site->Equipment[1]);
    TestEqual(TEXT("STS peers use the same berth aggregate"),Peer->GetObjectField(TEXT("berth"))->GetNumberField(TEXT("containers_per_berth_hour")),12.);
    Site->Manifest[2].HandoverMask=1;Site->ExportCargo[0].State=2;
    Site->IncidentMeasureClock=1000;Site->AdvanceEquipmentPerformance(.05,true);
    Site->IncidentMeasureClock=2000;Site->AdvanceEquipmentPerformance(.05,true);
    Berth=Site->CaptureBerthPerformance(0);
    TestEqual(TEXT("Loading and unloading both count once"),Berth->GetNumberField(TEXT("handled_containers")),4.);
    TestEqual(TEXT("A finished berth freezes at its own last completed operation"),Berth->GetNumberField(TEXT("operation_seconds")),900.);
    Site->IncidentMeasureClock=0;Site->ResetEquipmentPerformance();
    TestTrue(TEXT("Zero observation returns unknown rather than division by zero"),Site->CaptureEquipmentPerformance(K)->GetField<EJson::Null>(TEXT("utilization_percent")).IsValid());
    TestEqual(TEXT("Reset clears historical work"),Site->EquipmentWorkingSeconds.Num(),0);
    World->DestroyWorld(false);return true;
}
#endif
