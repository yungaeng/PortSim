#include "PortSiteLogistics.h"
#include "QuayCrane.h"
#include "PortWorkingCrane.h"
#include "PortAGVActor.h"
#include "PortContainerActor.h"
#include "Dom/JsonObject.h"
#include "Serialization/JsonSerializer.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "HAL/FileManager.h"
#include "HAL/PlatformTime.h"

void APortSiteLogistics::RecordRecoveryEvent(const FString& Kind,AActor* Cause,AActor* From,AActor* To,
    AActor* Cargo,const FVector& OldPosition,const FVector& NewPosition)
{
    auto E=MakeShared<FJsonObject>();
    TrackIncidentImpact(Cause,From,To,Cargo);
    E->SetNumberField(TEXT("sequence"),++RecoverySequence);
    E->SetNumberField(TEXT("measurement_seconds"),IncidentMeasureClock);
    E->SetNumberField(TEXT("simulation_seconds"),SimulationTime);
    E->SetStringField(TEXT("kind"),Kind);
    E->SetStringField(TEXT("cause_actor"),GetNameSafe(Cause));
    E->SetStringField(TEXT("from_actor"),GetNameSafe(From));
    E->SetStringField(TEXT("to_actor"),GetNameSafe(To));
    E->SetStringField(TEXT("cargo"),GetNameSafe(Cargo));
    auto Position=[&](const TCHAR* Key,FVector P)
    {
        TArray<TSharedPtr<FJsonValue>> Values;
        for(int32 I=0;I<3;++I) Values.Add(MakeShared<FJsonValueNumber>(P[I]*.01));
        E->SetArrayField(Key,Values);
    };
    Position(TEXT("from_position_m"),OldPosition); Position(TEXT("to_position_m"),NewPosition);
    RecoveryEvents.Add(E); if(RecoveryEvents.Num()>128) RecoveryEvents.RemoveAt(0);
}

bool APortSiteLogistics::ApplyDashboardCommand(const FJsonObject& C,FString& Error)
{
    FString Run,Session,ActorID,Action,Created; FDateTime Sent;
    if(!C.TryGetStringField(TEXT("run_id"),Run) || Run!=FPaths::GetCleanFilename(ReportBase) ||
        !C.TryGetStringField(TEXT("control_session"),Session) || Session!=DashboardControlSession)
    { Error=TEXT("실행이 변경됐습니다. 새로고침 후 다시 실행하세요."); return false; }
    if(!C.TryGetStringField(TEXT("created_at"),Created) || !FDateTime::ParseIso8601(*Created,Sent) ||
        (FDateTime::UtcNow()-Sent).GetTotalSeconds()>10 || (Sent-FDateTime::UtcNow()).GetTotalSeconds()>2)
    { Error=TEXT("명령 유효 시간이 지났습니다."); return false; }
    if(!Fault.IsEmpty()) { Error=TEXT("전체 안전 정지 원인을 먼저 해결하세요."); return false; }
    if(C.TryGetStringField(TEXT("action"),Action))
    {
        C.TryGetStringField(TEXT("actor_id"),ActorID);
        if(Action==TEXT("fail")&&Equipment.ContainsByPredicate([&](const auto& K){return K->GetName()==ActorID;}))
            Action=TEXT("fail_sensor_repair");
        if(PortIncident::RecoverySeconds(Action)>0)
        {
            auto Step=MakeShared<FJsonObject>();
            Step->SetStringField(TEXT("actor_id"),ActorID);
            Step->SetStringField(TEXT("action"),Action);
            if(Action==TEXT("fail")&&C.HasField(TEXT("control_radius_m")))
                Step->SetField(TEXT("control_radius_m"),C.Values.FindChecked(TEXT("control_radius_m")));
            FJsonObject Scenario;
            FString CommandID;
            if(C.TryGetStringField(TEXT("id"),CommandID)) Scenario.SetStringField(TEXT("id"),CommandID);
            Scenario.SetStringField(TEXT("action"),TEXT("scenario_start"));
            Scenario.SetStringField(TEXT("name"),TEXT("장비 오류 · 자동 복구"));
            Scenario.SetArrayField(TEXT("steps"),{MakeShared<FJsonValueObject>(Step)});
            return ApplyIncidentCommand(Scenario,Error);
        }
        if(Action==TEXT("incident_clear"))
        {
            FString Kind;
            C.TryGetStringField(TEXT("fault_action"),Kind);
            if(PortIncident::RecoverySeconds(Kind)>0)
            { Error=TEXT("이 고장은 자동 복구 일정에 따라 처리됩니다."); return false; }
        }
        if(Action==TEXT("repair"))
        {
            for(const auto& R:IncidentRuns) for(const auto& S:R.Steps)
                if(S.Duration>0&&S.AppliedAt>=0&&S.RecoveredAt<0&&
                    S.Command->GetStringField(TEXT("actor_id"))==ActorID)
                { Error=TEXT("이 고장은 자동 복구 일정에 따라 처리됩니다."); return false; }
        }
    }
    return ApplyDashboardAction(C,Error);
}

bool APortSiteLogistics::ApplyDashboardAction(const FJsonObject& C,FString& Error)
{
    FString ActorID,Action;
    if(!Fault.IsEmpty()) { Error=TEXT("전체 안전 정지 원인을 먼저 해결하세요."); return false; }
    if(C.TryGetStringField(TEXT("action"),Action)&&
        (PortIncident::Extension(Action)||Action==TEXT("scenario_start")||Action==TEXT("scenario_cancel")||Action==TEXT("set_environment")||Action==TEXT("incident_clear")||
         Action==TEXT("fail_pickup")||Action==TEXT("fail_placement")||Action==TEXT("misdelivery")||
         (PortIncident::SensorFailure(Action)&&C.TryGetStringField(TEXT("actor_id"),ActorID)&&Vehicles.ContainsByPredicate([&](const auto& V){return V->GetName()==ActorID;}))))
    {
        if(!Fault.IsEmpty()){Error=TEXT("전체 안전 정지 원인을 먼저 해결하세요.");return false;}
        return ApplyIncidentCommand(C,Error);
    }
    if(!C.TryGetStringField(TEXT("actor_id"),ActorID) || !C.TryGetStringField(TEXT("action"),Action) ||
        (Action!=TEXT("fail") && Action!=TEXT("fail_reboot") && Action!=TEXT("fail_sensor_repair") && Action!=TEXT("fail_gantry") && Action!=TEXT("repair") &&
         Action!=TEXT("replace_loaded_agv") && Action!=TEXT("test_wet_worker") && Action!=TEXT("test_wet_unsecured") && Action!=TEXT("worker_enter") && Action!=TEXT("clear_worker")))
    { Error=TEXT("지원하지 않는 명령입니다."); return false; }
    if(!Fault.IsEmpty()) { Error=TEXT("전체 안전 정지 원인을 먼저 해결하세요."); return false; }
    for(const auto& Crane:Equipment) if(Crane->GetName()==ActorID)
    {
        if(Action==TEXT("replace_loaded_agv") || Action.StartsWith(TEXT("test_")) || Action==TEXT("worker_enter") || Action==TEXT("clear_worker")){Error=TEXT("AGV 전용 돌발상황 명령입니다.");return false;}
        if(!Crane->Fault.IsEmpty() && !Crane->IsEquipmentUnavailable())
        { Error=TEXT("센서 수리로 안전 오류를 해제할 수 없습니다."); return false; }
        if(Action==TEXT("repair")) {Crane->bIncidentPickupFailure=Crane->bIncidentPlacementFailure=false;Crane->RepairEquipment();}
        else if(Action==TEXT("fail_gantry")) Crane->SetGantryFailure(true);
        else
        {
            if(Action==TEXT("fail_reboot") || Action==TEXT("fail_sensor_repair"))
            {
                Crane->bAutoSensorRecovery=true;
                Crane->SensorRepairMethod=Action==TEXT("fail_reboot")?EPortSensorRepairMethod::Reboot:EPortSensorRepairMethod::Technician;
                // Only the selected automation-camera fault leaves independent
                // travel safety available. Generic required-sensor failure does not.
                Crane->bAllowSensorFaultEvacuation=Action==TEXT("fail_reboot");
            }
            Crane->SetSensorFailure(true);
            if(PortIncident::SensorFailure(Action))
                Crane->SensorRepairPhase=Action==TEXT("fail_reboot")?TEXT("rebooting"):TEXT("technician_repair");
        }
        // Also coordinate while paused, so the acknowledged assignment state
        // is visible without starting physical motion during a pause.
        ManageEquipmentFailures();
        return true;
    }
    for(const auto& V:Vehicles) if(V->GetName()==ActorID)
    {
        if(Action==TEXT("replace_loaded_agv")) return RequestLoadedAGVRecovery(Vehicles.IndexOfByKey(V),Error);
        if(IsLoadedRecoveryVehicle(Vehicles.IndexOfByKey(V)) && Action!=TEXT("repair"))
        {Error=TEXT("R/S 복구 중에는 추가 고장·돌발상황 명령을 사용할 수 없습니다.");return false;}
        if(Action==TEXT("clear_worker"))
        {
            if(!V->ClearEmergencyScenario(Error))return false;
            RecordRecoveryEvent(TEXT("worker_removed"),V,V,V); return true;
        }
        if(Action==TEXT("worker_enter") || Action==TEXT("test_wet_worker") || Action==TEXT("test_wet_unsecured"))
        {
            const bool Intrusion=Action==TEXT("worker_enter");
            double Distance=V->EmergencySettings.WorkerDistanceM;
            if(Intrusion && (!C.TryGetNumberField(TEXT("worker_distance_m"),Distance) || !FMath::IsFinite(Distance) || Distance<5 || Distance>80))
            {Error=TEXT("작업자 진입 거리는 5~80m로 입력하세요.");return false;}
            const int32 Lane=Vehicles.IndexOfByKey(V);
            const int32* Approved=RoadReservationEndWaypoint.Find(V->VehicleID);
            if(!Jobs.IsValidIndex(Lane) || !Approved || Jobs[Lane].bEquipmentWaiting ||
                (!Intrusion && (Jobs[Lane].Stage!=3 || Jobs[Lane].bRMGReserved)))
            {Error=TEXT("도로 예약을 확보하고 이동 중인 AGV에서 시작하세요. 젖음·편심 시험은 야드로 운반 중인 40ft AGV가 필요합니다.");return false;}
            const auto& J=Jobs[Lane];const FVector Direction=V->PlannedMotionDirection(J.Route.IsValidIndex(J.Waypoint)?J.Route[J.Waypoint]:V->GetActorLocation());
            double ClearStraight=0;FVector From=V->GetActorLocation();
            for(int32 N=J.Waypoint;N<=*Approved && J.Route.IsValidIndex(N);++N)
            {
                const FVector Segment=J.Route[N]-From;
                if(FVector::DotProduct(Segment.GetSafeNormal2D(),Direction)<.999)break;
                ClearStraight+=Segment.Size2D()*.01;From=J.Route[N];
            }
            if(ClearStraight<Distance+10)
            {Error=TEXT("작업자와 제동 시험을 놓을 예약된 직선 도로가 부족합니다. 더 긴 직선 구간에서 실행하세요.");return false;}
            const bool Started=Intrusion?V->StartWorkerIntrusion(Distance,Error):V->StartEmergencyScenario(Action==TEXT("test_wet_worker"),Error);
            if(Started)RecordRecoveryEvent(Intrusion?TEXT("worker_entered"):TEXT("agv_emergency_started"),V,V,V,J.Actor.Get());
            return Started;
        }
        if(V->Emergency.Active){Error=TEXT("돌발상황 시험을 먼저 종료하세요.");return false;}
        if(Action!=TEXT("fail") && Action!=TEXT("repair")) { Error=TEXT("AGV에는 해당 크레인 고장을 적용할 수 없습니다."); return false; }
        if(Action==TEXT("repair")){V->bIncidentSensorFault=V->bIncidentDestinationMismatch=false;V->IncidentRebootRemaining=0;V->IncidentSensorPhase=TEXT("healthy");}
        const bool Failed=Action==TEXT("fail"),Changed=V->bDriveFault!=Failed;
        double Clearance=1.;
        if(Failed&&C.HasField(TEXT("control_radius_m"))&&
            (!C.TryGetNumberField(TEXT("control_radius_m"),Clearance)||!FMath::IsFinite(Clearance)||Clearance<.5||Clearance>20))
        {Error=TEXT("현장 통제 반경은 차량 외곽에서 0.5~20m입니다.");return false;}
        SetAGVIncidentDriveFailure(Vehicles.IndexOfByKey(V),Failed,Clearance,Failed?PortIncident::RecoverySeconds(Action):0.);
        if(Changed) { if(Failed) ++EquipmentFaultEvents; RecordRecoveryEvent(Failed?TEXT("fault"):TEXT("repair"),V,V,V); }
        return true;
    }
    Error=TEXT("현재 실행의 제어 가능한 장비가 아닙니다."); return false;
}

void APortSiteLogistics::ProcessDashboardCommands()
{
    if(FPlatformTime::Seconds()<NextControlPoll) return;
    NextControlPoll=FPlatformTime::Seconds()+.2;
    const FString Directory=FPaths::ProjectSavedDir()/TEXT("Dashboard/commands");
    TArray<FString> Files; IFileManager::Get().FindFiles(Files,*(Directory/TEXT("*.json")),true,false);
    Files.Sort();
    int32 Count=0;
    for(const auto& File:Files)
    {
        if(++Count>8) break;
        const FString ID=FPaths::GetBaseFilename(File),Path=Directory/File;
        FGuid Guid;
        if(!FGuid::ParseExact(ID,EGuidFormats::Digits,Guid)) continue;
        if(SeenDashboardCommands.Contains(ID)) { IFileManager::Get().Delete(*Path); continue; }
        FString Text;
        if(IFileManager::Get().FileSize(*Path)>16384 || !FFileHelper::LoadFileToString(Text,*Path))
        { IFileManager::Get().Delete(*Path); continue; }
        TSharedPtr<FJsonObject> C; FString Error; bool Applied=false;
        if(FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text),C) && C.IsValid())
            Applied=ApplyDashboardCommand(*C,Error);
        else Error=TEXT("잘못된 명령 형식입니다.");
        SeenDashboardCommands.Add(ID);
        if(Applied)
            if(auto* Controller=Cast<AQuayCrane>(GetOwner())) Controller->NotifyExternalSimulationInput();
        auto Result=MakeShared<FJsonObject>(); Result->SetStringField(TEXT("id"),ID);
        Result->SetStringField(TEXT("status"),Applied?TEXT("applied"):TEXT("rejected"));
        Result->SetStringField(TEXT("message"),Error);
        Result->SetNumberField(TEXT("simulation_seconds"),SimulationTime);
        DashboardCommandResults.Add(Result);
        // Publish the acknowledgement at the next completed scene boundary,
        // even when accelerated playback has reduced routine telemetry cadence.
        NextDashboardWall=0;
        if(DashboardCommandResults.Num()>32)
        { SeenDashboardCommands.Remove(DashboardCommandResults[0]->GetStringField(TEXT("id"))); DashboardCommandResults.RemoveAt(0); }
        IFileManager::Get().Delete(*Path);
    }
}
