#include "PortSiteLogistics.h"
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
    E->SetNumberField(TEXT("sequence"),++RecoverySequence);
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
    if(!C.TryGetStringField(TEXT("actor_id"),ActorID) || !C.TryGetStringField(TEXT("action"),Action) ||
        (Action!=TEXT("fail") && Action!=TEXT("fail_reboot") && Action!=TEXT("fail_sensor_repair") && Action!=TEXT("fail_gantry") && Action!=TEXT("repair") &&
         Action!=TEXT("test_wet_worker") && Action!=TEXT("test_wet_unsecured") && Action!=TEXT("clear_worker")))
    { Error=TEXT("지원하지 않는 명령입니다."); return false; }
    if(!Fault.IsEmpty()) { Error=TEXT("전체 안전 정지 원인을 먼저 해결하세요."); return false; }
    for(const auto& Crane:Equipment) if(Crane->GetName()==ActorID)
    {
        if(Action.StartsWith(TEXT("test_")) || Action==TEXT("clear_worker")){Error=TEXT("AGV 전용 돌발상황 명령입니다.");return false;}
        if(!Crane->Fault.IsEmpty() && !Crane->IsEquipmentUnavailable())
        { Error=TEXT("센서 수리로 안전 오류를 해제할 수 없습니다."); return false; }
        if(Action==TEXT("repair")) Crane->RepairEquipment();
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
        }
        // Also coordinate while paused, so the acknowledged assignment state
        // is visible without starting physical motion during a pause.
        ManageEquipmentFailures();
        return true;
    }
    for(const auto& V:Vehicles) if(V->GetName()==ActorID)
    {
        if(Action==TEXT("clear_worker"))return V->ClearEmergencyScenario(Error);
        if(Action==TEXT("test_wet_worker") || Action==TEXT("test_wet_unsecured"))
        {
            const int32 Lane=Vehicles.IndexOfByKey(V);
            const int32* Approved=RoadReservationEndWaypoint.Find(V->VehicleID);
            if(!Jobs.IsValidIndex(Lane) || Jobs[Lane].Stage!=3 || !Approved || Jobs[Lane].bEquipmentWaiting || Jobs[Lane].bRMGReserved)
            {Error=TEXT("도로 예약을 확보하고 야드로 운반 중인 40ft AGV에서 시작하세요.");return false;}
            const auto& J=Jobs[Lane];const FVector Direction=V->PlannedMotionDirection(J.Route.IsValidIndex(J.Waypoint)?J.Route[J.Waypoint]:V->GetActorLocation());
            double ClearStraight=0;FVector From=V->GetActorLocation();
            for(int32 N=J.Waypoint;N<=*Approved && J.Route.IsValidIndex(N);++N)
            {
                const FVector Segment=J.Route[N]-From;
                if(FVector::DotProduct(Segment.GetSafeNormal2D(),Direction)<.999)break;
                ClearStraight+=Segment.Size2D()*.01;From=J.Route[N];
            }
            if(ClearStraight<V->EmergencySettings.WorkerDistanceM+10)
            {Error=TEXT("작업자와 제동 시험을 놓을 예약된 직선 도로가 부족합니다. 더 긴 직선 구간에서 실행하세요.");return false;}
            const bool Started=V->StartEmergencyScenario(Action==TEXT("test_wet_worker"),Error);
            if(Started)RecordRecoveryEvent(TEXT("agv_emergency_started"),V,V,V,J.Actor.Get());
            return Started;
        }
        if(V->Emergency.Active){Error=TEXT("돌발상황 시험을 먼저 종료하세요.");return false;}
        if(Action!=TEXT("fail") && Action!=TEXT("repair")) { Error=TEXT("AGV에는 해당 크레인 고장을 적용할 수 없습니다."); return false; }
        const bool Failed=Action==TEXT("fail"),Changed=V->bDriveFault!=Failed;
        V->SetDriveFailure(Failed);
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
        if(IFileManager::Get().FileSize(*Path)>2048 || !FFileHelper::LoadFileToString(Text,*Path))
        { IFileManager::Get().Delete(*Path); continue; }
        TSharedPtr<FJsonObject> C; FString Error; bool Applied=false;
        if(FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text),C) && C.IsValid())
            Applied=ApplyDashboardCommand(*C,Error);
        else Error=TEXT("잘못된 명령 형식입니다.");
        SeenDashboardCommands.Add(ID);
        auto Result=MakeShared<FJsonObject>(); Result->SetStringField(TEXT("id"),ID);
        Result->SetStringField(TEXT("status"),Applied?TEXT("applied"):TEXT("rejected"));
        Result->SetStringField(TEXT("message"),Error);
        Result->SetNumberField(TEXT("simulation_seconds"),SimulationTime);
        DashboardCommandResults.Add(Result);
        if(DashboardCommandResults.Num()>32)
        { SeenDashboardCommands.Remove(DashboardCommandResults[0]->GetStringField(TEXT("id"))); DashboardCommandResults.RemoveAt(0); }
        IFileManager::Get().Delete(*Path);
    }
}
