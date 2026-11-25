#if WITH_DEV_AUTOMATION_TESTS
#include "PortSiteLogistics.h"
#include "PortWorkingCrane.h"
#include "PortAGVActor.h"
#include "PortContainerActor.h"
#include "QuayCrane.h"
#include "Dom/JsonObject.h"
#include "Components/StaticMeshComponent.h"
#include "Misc/AutomationTest.h"
#include "Misc/Paths.h"
#include "Engine/World.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIncidentScenarioTest,"PortSim.Incidents.LiveCombinationAndRecovery",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FIncidentScenarioTest::RunTest(const FString& Parameters)
{
    const auto Init=UWorld::InitializationValues().AllowAudioPlayback(false).CreatePhysicsScene(true)
        .RequiresHitProxies(false).CreateNavigation(false).CreateAISystem(false);
    auto* World=UWorld::CreateWorld(EWorldType::Game,false,NAME_None,nullptr,true,ERHIFeatureLevel::Num,&Init);
    auto* Owner=World->SpawnActor<AQuayCrane>();Owner->bAutoRunning=true;
    auto* Site=World->SpawnActor<APortSiteLogistics>();Site->SetOwner(Owner);Site->ReportBase=TEXT("IncidentNative");
    auto* V=World->SpawnActor<APortAGVActor>();V->InitializeVehicle(100);Site->Vehicles={V};Site->Jobs.SetNum(1);
    auto Stamp=[&](FJsonObject& C){C.SetStringField(TEXT("run_id"),TEXT("IncidentNative"));C.SetStringField(TEXT("control_session"),Site->DashboardControlSession);C.SetStringField(TEXT("created_at"),FDateTime::UtcNow().ToIso8601());};
    auto Step=[&](const TCHAR* Action,double Delay,double Duration)
    {auto S=MakeShared<FJsonObject>();S->SetStringField(TEXT("actor_id"),V->GetName());S->SetStringField(TEXT("action"),Action);S->SetNumberField(TEXT("delay"),Delay);S->SetNumberField(TEXT("duration"),Duration);S->SetNumberField(TEXT("timeout"),300);return S;};
    auto Rain=Step(TEXT("set_environment"),0,0);Rain->SetStringField(TEXT("actor_id"),TEXT("environment"));
    auto Weather=MakeShared<FJsonObject>();Weather->SetNumberField(TEXT("rain_mm_h"),25);Rain->SetObjectField(TEXT("weather"),Weather);
    FJsonObject Start;Stamp(Start);Start.SetStringField(TEXT("action"),TEXT("scenario_start"));Start.SetStringField(TEXT("id"),TEXT("native"));
    TArray<TSharedPtr<FJsonValue>> Steps={MakeShared<FJsonValueObject>(Rain),MakeShared<FJsonValueObject>(Step(TEXT("fail_reboot"),0,0)),MakeShared<FJsonValueObject>(Step(TEXT("fail"),0,5)),MakeShared<FJsonValueObject>(Step(TEXT("fail"),90,0))};
    Start.SetArrayField(TEXT("steps"),Steps);FString Error;
    TestFalse(TEXT("Duplicate fault rejected before any weather or machine mutation"),Site->ApplyDashboardCommand(Start,Error));
    TestFalse(TEXT("Rejected batch did not inject drive failure"),V->bDriveFault);
    Steps.Pop();Start.SetArrayField(TEXT("steps"),Steps);
    TestTrue(TEXT("Register simultaneous weather, sensor and drive failures"),Site->ApplyDashboardCommand(Start,Error));
    Site->AdvanceIncidentScenarios(.05f);
    TestTrue(TEXT("Weather changed in the actual shared environment"),Owner->GetEnvironment()->IsRainEnabled()&&Owner->GetEnvironment()->GetRainfallMmPerHour()==25);
    TestTrue(TEXT("Independent sensor and drive failures coexist"),V->bDriveFault&&V->bIncidentSensorFault&&!V->CanAcceptWork());
    Owner->bAutoPaused=true;const double Clock=Site->IncidentClock;Site->AdvanceIncidentScenarios(30);
    TestEqual(TEXT("Manual pause freezes event/reboot timers"),Site->IncidentClock,Clock);
    Owner->bAutoPaused=false;Owner->WeatherHold.Active=true;Site->AdvanceIncidentScenarios(6);
    TestTrue(TEXT("Weather hold still permits scheduled recovery"),Site->IncidentClock>Clock);
    TestTrue(TEXT("Supplied five-second duration cannot shorten drive repair"),V->bDriveFault);
    FJsonObject Reboot;Stamp(Reboot);Reboot.SetStringField(TEXT("actor_id"),V->GetName());
    Reboot.SetStringField(TEXT("action"),TEXT("incident_clear"));Reboot.SetStringField(TEXT("fault_action"),TEXT("fail_reboot"));
    TestFalse(TEXT("Web recovery cannot bypass the sensor timer"),Site->ApplyDashboardCommand(Reboot,Error));
    Site->AdvanceIncidentScenarios(23);
    TestTrue(TEXT("Reboot holds the failed sensor for 30 simulation seconds"),
        V->bIncidentSensorFault&&V->IncidentSensorPhase==TEXT("rebooting"));
    Site->AdvanceIncidentScenarios(1.1f);
    TestFalse(TEXT("Sensor reboot starts automatically even with an old zero-duration request"),V->bIncidentSensorFault);
    Site->RefreshAGVIncidentControl();
    TestTrue(TEXT("Sensor recovery leaves the separate drive failure and road control active"),V->bDriveFault&&Site->HasIncidentRoadClosure());
    Site->AdvanceIncidentScenarios(1769);
    TestTrue(TEXT("Drive repair waits for its full 30-minute window"),V->bDriveFault);
    Site->AdvanceIncidentScenarios(1);
    TestFalse(TEXT("Drive repair executes after 30 simulation minutes"),V->bDriveFault);
    Site->RefreshAGVIncidentControl();
    Site->AdvanceIncidentScenarios(.05f);
    TestFalse(TEXT("Control releases after drive and sensor recovery"),Site->HasIncidentRoadClosure());
    TestEqual(TEXT("A repaired road still requires real follow-up cargo completion"),Site->IncidentRuns[0].Steps[2].State,FString(TEXT("awaiting_completion")));
    FJsonObject Cancel;Stamp(Cancel);Cancel.SetStringField(TEXT("action"),TEXT("scenario_cancel"));Cancel.SetStringField(TEXT("scenario_id"),TEXT("native"));
    TestTrue(TEXT("Cancellation retains already applied scenario history"),Site->ApplyDashboardCommand(Cancel,Error));
    TestTrue(TEXT("Weather is not silently reset by cancellation"),Owner->GetEnvironment()->IsRainEnabled());
    auto* Crane=World->SpawnActor<APortWorkingCrane>();Crane->bSTS=true;Site->Equipment.Add(Crane);
    FJsonObject Hoist;Stamp(Hoist);Hoist.SetStringField(TEXT("actor_id"),Crane->GetName());
    Hoist.SetStringField(TEXT("action"),TEXT("fail_hoist"));Hoist.SetNumberField(TEXT("timeout"),30);
    TestTrue(TEXT("Direct web hoist command schedules manual repair"),Site->ApplyDashboardCommand(Hoist,Error));
    Site->AdvanceIncidentScenarios(.05f);
    Site->AdvanceIncidentScenarios(7199);
    TestTrue(TEXT("Two-hour repair outlasts submitted verification timeout"),Crane->IncidentFaults.Contains(TEXT("fail_hoist")));
    Site->AdvanceIncidentScenarios(1);
    TestFalse(TEXT("Two-hour hoist repair clears the injected fault"),Crane->IncidentFaults.Contains(TEXT("fail_hoist")));
    FJsonObject Restart=Hoist;Stamp(Restart);Restart.SetStringField(TEXT("action"),TEXT("restart_controller"));
    FJsonObject Gantry=Restart;Gantry.SetStringField(TEXT("action"),TEXT("fail_gantry"));
    TestTrue(TEXT("Controller restart and gantry repair can be scheduled independently"),
        Site->ApplyDashboardCommand(Restart,Error)&&Site->ApplyDashboardCommand(Gantry,Error));
    Site->AdvanceIncidentScenarios(.05f);
    Site->AdvanceIncidentScenarios(119);
    TestTrue(TEXT("The controller stays blocked until its restart window"),
        Crane->IncidentFaults.Contains(TEXT("restart_controller")));
    Site->AdvanceIncidentScenarios(1);
    TestTrue(TEXT("Controller restart does not repair the independent gantry fault"),
        !Crane->IncidentFaults.Contains(TEXT("restart_controller"))&&Crane->bGantryFault&&Crane->IsEquipmentUnavailable());
    Owner->GetEnvironment()->AdvanceEnvironment(20);TestTrue(TEXT("Rain evolves real surface accumulation"),Owner->GetEnvironment()->GetSurfaceConditions().WaterFilmMm>0);
    World->DestroyWorld(false);return true;
}
#endif
