#if WITH_DEV_AUTOMATION_TESTS
#include "PortSiteLogistics.h"
#include "PortAGVActor.h"
#include "PortContainerActor.h"
#include "PortWorkingCrane.h"
#include "QuayCrane.h"
#include "Dom/JsonObject.h"
#include "Engine/World.h"
#include "Misc/AutomationTest.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIncidentTimingTest,"PortSim.Incidents.TimingAndPropagation",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FIncidentTimingTest::RunTest(const FString& Parameters)
{
    const auto Init=UWorld::InitializationValues().AllowAudioPlayback(false).CreatePhysicsScene(true)
        .RequiresHitProxies(false).CreateNavigation(false).CreateAISystem(false);
    auto* World=UWorld::CreateWorld(EWorldType::Game,false,NAME_None,nullptr,true,ERHIFeatureLevel::Num,&Init);
    auto* Owner=World->SpawnActor<AQuayCrane>();Owner->bAutoRunning=true;
    auto* Site=World->SpawnActor<APortSiteLogistics>();Site->SetOwner(Owner);Site->ReportBase=TEXT("IncidentTiming");
    auto* V=World->SpawnActor<APortAGVActor>();V->InitializeVehicle(100);
    auto* Follower=World->SpawnActor<APortAGVActor>();Follower->InitializeVehicle(101);
    auto* Cargo=World->SpawnActor<APortContainerActor>();Cargo->InitializeContainer(42);
    Site->Vehicles={V,Follower};Site->Jobs.SetNum(2);Site->Manifest.SetNum(1);
    Site->Manifest[0].ID=42;Site->Manifest[0].Actor=Cargo;Site->Jobs[0].Actor=Cargo;Site->Jobs[0].Stage=3;
    Site->BeginIncidentMeasurement();const FString Key=Site->IncidentComparisonKey;
    Site->BeginIncidentMeasurement();TestEqual(TEXT("Same initial model produces the same comparison key"),Site->IncidentComparisonKey,Key);
    Site->Manifest[0].PlannedSlot=123;Site->BeginIncidentMeasurement();
    TestTrue(TEXT("A changed initial yard plan invalidates comparison"),Site->IncidentComparisonKey!=Key);
    Site->AdvanceIncidentMeasurement(10,false);Owner->bAutoPaused=true;Site->AdvanceIncidentMeasurement(50,true);
    TestEqual(TEXT("Operator pause is excluded"),Site->IncidentMeasureClock,10.);
    Owner->bAutoPaused=false;Owner->WeatherHold.Active=true;Site->AdvanceIncidentMeasurement(5,true);
    Site->Fault=TEXT("safety halt");Owner->bEmergencyStop=true;Site->AdvanceIncidentMeasurement(7,true);
    TestEqual(TEXT("Weather, safety and E-stop remain measurable"),Site->IncidentMeasureClock,22.);
    TestEqual(TEXT("Parallel halt causes count elapsed time once"),Site->IncidentHaltSeconds,12.);
    Owner->bEmergencyStop=false;Owner->WeatherHold.Active=false;Site->Fault.Empty();Site->BeginIncidentMeasurement();
    auto Step=MakeShared<FJsonObject>();Step->SetStringField(TEXT("actor_id"),V->GetName());Step->SetStringField(TEXT("action"),TEXT("fail"));
    FJsonObject Start;Start.SetStringField(TEXT("action"),TEXT("scenario_start"));Start.SetStringField(TEXT("id"),TEXT("timing"));
    Start.SetArrayField(TEXT("steps"),{MakeShared<FJsonValueObject>(Step)});FString Error;
    TestTrue(TEXT("Apply live incident"),Site->ApplyIncidentCommand(Start,Error));Site->AdvanceIncidentScenarios(.05);
    auto& S=Site->IncidentRuns[0].Steps[0];TestEqual(TEXT("Actual injection, not requested delay, starts measurement"),S.MeasureAppliedAt,0.);
    Follower->SetFMSHold(true);Site->RoadBlockers.Add(Follower->VehicleID,V->VehicleID);Site->Jobs[1].Stage=3;
    Site->AdvanceIncidentMeasurement(5,false);
    TestTrue(TEXT("Observed follower dependency is included"),S.AffectedActors.Contains(Follower->GetName()));
    TestFalse(TEXT("Faulted run cannot become normal reference"),Site->CaptureIncidentTiming()->GetBoolField(TEXT("reference_eligible")));
    V->SetDriveFailure(false);Site->AdvanceIncidentMeasurement(1,false);
    TestEqual(TEXT("Fault clearing has a separate timestamp"),S.MeasureClearedAt,6.);
    TestEqual(TEXT("Repair without progress is not work resumption"),S.MeasureResumedAt,-1.);
    V->Speed=100;Site->AdvanceIncidentMeasurement(2,false);
    TestEqual(TEXT("Actual progress confirms resumption"),S.MeasureResumedAt,8.);
    Site->IncidentCompletionTimes.Add(TEXT("C42"),11.);Site->Manifest[0].State=2;Site->AdvanceIncidentMeasurement(3,false);
    TestEqual(TEXT("Tracked cargo completion is independent of repair"),S.MeasureCompletedAt,11.);
    const auto Snapshot=Site->CaptureIncidentScenarios();
    const auto Row=Snapshot->GetArrayField(TEXT("runs"))[0]->AsObject()->GetArrayField(TEXT("steps"))[0]->AsObject();
    TestEqual(TEXT("Completion timestamp is exported"),Row->GetNumberField(TEXT("work_completed_at")),11.);
    S.State=TEXT("timeout");S.MeasureCompletedAt=-1;S.MeasureClearedAt=-1;V->SetDriveFailure(true);
    Site->IncidentCompletionTimes.Reset();Site->Manifest[0].State=1;
    Site->AdvanceIncidentMeasurement(4,false);V->SetDriveFailure(false);Site->AdvanceIncidentMeasurement(1,false);
    TestEqual(TEXT("Timeout does not fabricate zero delay or stop fault tracking"),S.MeasureClearedAt,16.);
    auto* Replacement=World->SpawnActor<APortWorkingCrane>();Replacement->CargoActor=Cargo;
    Replacement->Observation.DriveVelocity=FVector(100,0,0);Site->Equipment.Add(Replacement);
    auto& Pending=Site->IncidentRuns[0].Steps.AddDefaulted_GetRef();Pending.Command=Step;
    Pending.MeasureAppliedAt=Site->IncidentMeasureClock;Pending.AffectedActors.Add(V->GetName());
    Site->RecordRecoveryEvent(TEXT("sts_reassigned"),V,V,Replacement,Cargo);
    TestTrue(TEXT("Immediate reassignment is captured before step state becomes active"),Pending.AffectedActors.Contains(Replacement->GetName())&&Pending.AffectedCargo.Contains(Cargo->GetName()));
    Site->AdvanceIncidentMeasurement(1,false);
    TestEqual(TEXT("Replacement resumes unassigned source work"),Pending.MeasureResumedAt,17.);
    Site->IncidentCompletionTimes.Add(TEXT("C42"),18.);Site->AdvanceIncidentMeasurement(1,false);
    TestEqual(TEXT("Unassigned incident follows completion of its reassigned cargo set"),Pending.MeasureCompletedAt,18.);
    Pending.MeasureClearedAt=-1;V->SetDriveFailure(true);Site->AdvanceIncidentMeasurement(2,false);
    V->SetDriveFailure(false);Site->AdvanceIncidentMeasurement(1,false);
    TestEqual(TEXT("Equipment repair remains measurable after reassigned cargo finishes"),Pending.MeasureClearedAt,21.);
    Owner->bAutoRunning=false;Site->AdvanceIncidentMeasurement(100,false);
    TestEqual(TEXT("Finished runs do not grow after completion"),Site->IncidentMeasureClock,21.);
    World->DestroyWorld(false);return true;
}
#endif
