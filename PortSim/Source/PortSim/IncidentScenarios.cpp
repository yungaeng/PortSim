#include "IncidentScenarios.h"
#include "PortSiteLogistics.h"
#include "PortWorkingCrane.h"
#include "PortAGVActor.h"
#include "PortContainerActor.h"
#include "QuayCrane.h"
#include "Dom/JsonObject.h"
#include "Components/StaticMeshComponent.h"

namespace
{
    bool InRange(const FJsonObject& O,const TCHAR* Key,double Default,double Min,double Max)
    { double V=Default;return (!O.HasField(Key)||O.TryGetNumberField(Key,V))&&FMath::IsFinite(V)&&V>=Min&&V<=Max; }
    double Number(const FJsonObject& O,const TCHAR* Key,double Default=0)
    { double V=Default;O.TryGetNumberField(Key,V);return V; }
    FString String(const FJsonObject& O,const TCHAR* Key,const TCHAR* Default=TEXT(""))
    { FString V=Default;O.TryGetStringField(Key,V);return V; }
    bool Terminal(const FString& S)
    { return S==TEXT("verified")||S==TEXT("rejected")||S==TEXT("timeout")||S==TEXT("cancelled")||S==TEXT("not_exercised"); }
}

bool APortSiteLogistics::ValidateIncidentStep(const FJsonObject& C,FString& Error) const
{
    const FString ID=String(C,TEXT("actor_id")),Action=String(C,TEXT("action")),Trigger=String(C,TEXT("trigger"),TEXT("immediate"));
    bool AGV=false,Crane=false,RMG=false;
    for(const auto& V:Vehicles)if(V->GetName()==ID)AGV=true;
    for(const auto& V:Equipment)if(V->GetName()==ID){Crane=true;RMG=!V->bSTS;}
    const bool Weather=ID==TEXT("environment")&&Action==TEXT("set_environment");
    const bool Allowed=Weather || (AGV&&(PortIncident::AGVExtension(Action)||Action==TEXT("fail")||PortIncident::SensorFailure(Action)||Action==TEXT("misdelivery")||Action==TEXT("worker_enter"))) ||
        (Crane&&(PortIncident::CraneExtension(Action,RMG)||Action==TEXT("fail_gantry")||Action==TEXT("fail_pickup")||PortIncident::SensorFailure(Action)||(RMG&&Action==TEXT("fail_placement"))));
    const bool TriggerOK=Trigger==TEXT("immediate") || (AGV&&(Trigger==TEXT("moving")||Trigger==TEXT("loaded"))) ||
        (Crane&&(Trigger==TEXT("pickup")||Trigger==TEXT("placing")||Trigger==TEXT("release")||Trigger==TEXT("handover")));
    if(!Allowed||!TriggerOK||!InRange(C,TEXT("delay"),0,0,3600))
    {Error=TEXT("장비·발생 조건·시간 범위를 확인하세요.");return false;}
    if(Action==TEXT("worker_enter")&&!InRange(C,TEXT("worker_distance_m"),20,5,80))
    {Error=TEXT("작업자 거리는 5~80m입니다.");return false;}
    if(AGV&&Action==TEXT("fail")&&!InRange(C,TEXT("control_radius_m"),1,.5,20))
    {Error=TEXT("현장 통제 반경은 0.5~20m입니다.");return false;}
    if(Weather)
    {
        const TSharedPtr<FJsonObject>* W=nullptr;
        if(!C.TryGetObjectField(TEXT("weather"),W)||(*W)->Values.IsEmpty())
        {Error=TEXT("날씨 입력과 별도 해제 단계를 지정하세요.");return false;}
        for(const auto& P:(*W)->Values)
        {
            const double Max=P.Key==TEXT("rain_mm_h")||P.Key==TEXT("snow_mm_h")?100:P.Key==TEXT("wind_mps")?7:P.Key==TEXT("fog_m")?1000:P.Key==TEXT("ice")?1:-1;
            if(!InRange(**W,*P.Key,0,0,Max)){Error=TEXT("지원하지 않는 날씨 값입니다.");return false;}
        }
    }
    return true;
}

bool APortSiteLogistics::ApplyIncidentCommand(const FJsonObject& C,FString& Error)
{
    const FString Action=String(C,TEXT("action")),ID=String(C,TEXT("actor_id"));
    if(PortIncident::Extension(Action))return ApplyExtendedIncident(C,Error);
    if(Action==TEXT("incident_clear"))
    {
        const FString Kind=String(C,TEXT("fault_action"));
        if(PortIncident::Extension(Kind))return ApplyExtendedIncident(C,Error,true);
        for(const auto& V:Vehicles)if(V->GetName()==ID)
        {
            if(Kind==TEXT("fail"))SetAGVIncidentDriveFailure(Vehicles.IndexOfByKey(V),false,1,0);
            else if(PortIncident::SensorFailure(Kind))
            {
                if(!V->bIncidentSensorFault||V->IncidentRebootRemaining>=0)return true;
                V->IncidentSensorPhase=TEXT("rebooting");V->IncidentRebootRemaining=PortIncident::SensorRestartSeconds;
                RecordRecoveryEvent(TEXT("sensor_reboot_started"),V,V,V);return true;
            }
            else if(Kind==TEXT("misdelivery"))V->bIncidentDestinationMismatch=false;
            else if(Kind==TEXT("worker_enter")){if(!V->ClearEmergencyScenario(Error))return false;}
            else {Error=TEXT("지원하지 않는 복구 대상입니다.");return false;}
            if(V->bDriveFault||V->bIncidentSensorFault)V->SetFMSHold(true);
            RecordRecoveryEvent(TEXT("repair"),V,V,V);return true;
        }
        for(const auto& K:Equipment)if(K->GetName()==ID)
        {
            if(!K->Fault.IsEmpty()&&!K->IsEquipmentUnavailable()){Error=TEXT("안전 오류는 상황 해제로 지울 수 없습니다.");return false;}
            if(Kind==TEXT("fail_pickup"))
            {
                K->bIncidentPickupFailure=false;
                K->RepairPickupFailure();
                if(K->IsPickupUnavailable()){Error=TEXT("스프레더가 안전한 상태가 될 때까지 기다립니다.");return false;}
            }
            else if(Kind==TEXT("fail_placement"))K->bIncidentPlacementFailure=false;
            else if(Kind==TEXT("fail_gantry"))K->SetGantryFailure(false);
            else if(PortIncident::SensorFailure(Kind))
            {
                if(!K->bSensorFault||K->bAutoSensorRecovery)return true;
                K->SensorRepairMethod=Kind==TEXT("fail_reboot")?EPortSensorRepairMethod::Reboot:EPortSensorRepairMethod::Technician;
                K->SensorRepairElapsed=0;K->SensorRepairPhase=Kind==TEXT("fail_reboot")?TEXT("rebooting"):TEXT("technician_repair");
                K->bAutoSensorRecovery=true;
                RecordRecoveryEvent(TEXT("sensor_reboot_started"),K,K,K);return true;
            }
            else {Error=TEXT("지원하지 않는 복구 대상입니다.");return false;}
            ManageEquipmentFailures();RecordRecoveryEvent(TEXT("repair"),K,K,K);return true;
        }
        Error=TEXT("복구할 장비를 찾지 못했습니다.");return false;
    }
    if(Action==TEXT("scenario_start"))
    {
        const TArray<TSharedPtr<FJsonValue>>* Steps=nullptr;
        if(IncidentRuns.Num()>=24||!C.TryGetArrayField(TEXT("steps"),Steps)||Steps->Num()<1||Steps->Num()>12)
        {Error=TEXT("시나리오는 1~12개 상황, 실행 기록은 최대 24개입니다. 새 실행에서 초기화하세요.");return false;}
        FPortIncidentRun Run;Run.ID=String(C,TEXT("id"));if(Run.ID.IsEmpty())Run.ID=FGuid::NewGuid().ToString(EGuidFormats::Digits);
        Run.Name=String(C,TEXT("name"),TEXT("실시간 시나리오")).Left(80);Run.StartedAt=IncidentClock;
        TSet<FString> Keys;
        for(const auto& Value:*Steps)
        {
            const TSharedPtr<FJsonObject>* Step=nullptr;
            if(!Value->TryGetObject(Step)||!ValidateIncidentStep(**Step,Error))return false;
            const FString TargetID=String(**Step,TEXT("actor_id")),Kind=String(**Step,TEXT("action"));
            const FString Key=TargetID+TEXT("|")+(PortIncident::SensorFailure(Kind)?TEXT("sensor"):PortIncident::ControllerFailure(Kind)?TEXT("controller"):Kind);
            if(Kind!=TEXT("set_environment"))
            {
                if(Keys.Contains(Key)){Error=TEXT("동일 장비의 같은 오류가 중복됩니다.");return false;}
                for(const auto& Existing:IncidentRuns)for(const auto& S:Existing.Steps)
                {
                    const FString Other=String(*S.Command,TEXT("action"));
                    const bool Same=Other==Kind||(PortIncident::SensorFailure(Kind)&&PortIncident::SensorFailure(Other))||
                        (PortIncident::ControllerFailure(Kind)&&PortIncident::ControllerFailure(Other));
                    if(!Terminal(S.State)&&Same&&String(*S.Command,TEXT("actor_id"))==TargetID)
                    {Error=TEXT("해당 장비의 같은 상황이 이미 실행 중입니다.");return false;}
                }
                Keys.Add(Key);
            }
            FPortIncidentStep S;S.Command=MakeShared<FJsonObject>(**Step);S.Delay=Number(**Step,TEXT("delay"));
            S.Duration=PortIncident::RecoverySeconds(Kind);
            if(PortIncident::SensorFailure(Kind))
                for(const auto& K:Equipment)if(K->GetName()==TargetID)
                    S.Duration=FMath::Max(0.,Kind==TEXT("fail_reboot")?K->SensorRebootSeconds:K->SensorTechnicianRepairSeconds)+
                        FMath::Max(0.,K->SensorSelfCheckSeconds);
            S.Command->RemoveField(TEXT("duration"));
            S.Command->RemoveField(TEXT("timeout"));
            Run.Steps.Add(MoveTemp(S));
        }
        IncidentRuns.Add(MoveTemp(Run));bIncidentInputUsed=true;return true;
    }
    if(Action==TEXT("scenario_cancel"))
    {
        for(auto& R:IncidentRuns)if(R.ID==String(C,TEXT("scenario_id")))
        {
            R.Cancelled=true;for(auto& S:R.Steps)if(S.State==TEXT("pending")){S.State=TEXT("cancelled");S.Message=TEXT("미발생 단계 취소");}
            // Applied failures retain their recovery timers and safety state.
            return true;
        }
        Error=TEXT("현재 실행의 시나리오가 아닙니다.");return false;
    }
    if(Action==TEXT("set_environment"))
    {
        FJsonObject Check=C;Check.SetStringField(TEXT("actor_id"),TEXT("environment"));
        if(!ValidateIncidentStep(Check,Error))return false;
        auto* E=GetOwner()?GetOwner()->FindComponentByClass<UPortEnvironmentComponent>():nullptr;
        if(!E){Error=TEXT("환경 컴포넌트가 없습니다.");return false;}
        const auto& W=*C.GetObjectField(TEXT("weather"));
        if(W.HasField(TEXT("rain_mm_h"))){const double V=Number(W,TEXT("rain_mm_h"));E->SetRainfallMmPerHour(V);E->SetRainEnabled(V>0);}
        if(W.HasField(TEXT("snow_mm_h"))){const double V=Number(W,TEXT("snow_mm_h"));E->SetSnowfallMmPerHour(V);E->SetSnowEnabled(V>0);}
        if(W.HasField(TEXT("wind_mps"))){const double V=Number(W,TEXT("wind_mps"));E->SetWindSpeedMetersPerSecond(V);E->SetWindEnabled(V>0);}
        if(W.HasField(TEXT("fog_m"))){const double V=Number(W,TEXT("fog_m"));E->SetFogVisibilityMeters(V);E->SetFogEnabled(V>0);}
        if(W.HasField(TEXT("ice"))){const double V=Number(W,TEXT("ice"));E->SetIceIntensity(V);E->SetIceEnabled(V>0);}
        bIncidentInputUsed=true;RecordRecoveryEvent(TEXT("weather_changed"),GetOwner(),GetOwner(),GetOwner());return true;
    }
    for(const auto& V:Vehicles)if(V->GetName()==ID)
    {
        if(PortIncident::SensorFailure(Action))
        {
            if(V->bIncidentSensorFault){Error=TEXT("이미 센서 오류가 발생했습니다.");return false;}
            V->bIncidentSensorFault=true;
            V->IncidentRebootRemaining=PortIncident::RecoverySeconds(Action);
            V->IncidentSensorPhase=Action==TEXT("fail_reboot")?TEXT("rebooting"):TEXT("technician_repair");
            V->Sensors.bTransponderLocked=false;
            V->SetFMSHold(true);
        }
        else if(Action==TEXT("misdelivery"))
        {
            const int32 Lane=Vehicles.IndexOfByKey(V);
            if(!Jobs.IsValidIndex(Lane)||Jobs[Lane].Stage!=3||V->PayloadKg<=0)
            {Error=TEXT("야드로 화물을 운반 중인 AGV를 선택하세요.");return false;}
            V->bIncidentDestinationMismatch=true;
        }
        else return false;
        bIncidentInputUsed=true;RecordRecoveryEvent(Action,V,V,V);return true;
    }
    for(const auto& V:Equipment)if(V->GetName()==ID)
    {
        if(Action!=TEXT("fail_pickup")&&Action!=TEXT("fail_placement"))return false;
        if(Action==TEXT("fail_placement")&&V->bSTS){Error=TEXT("RMG 적재 상황입니다.");return false;}
        if(Action==TEXT("fail_pickup")&&V->RetainsLoad()){Error=TEXT("화물을 들고 있는 크레인에는 체결 실패를 적용할 수 없습니다.");return false;}
        if(!V->Fault.IsEmpty()){Error=TEXT("기존 장비 오류를 먼저 복구하세요.");return false;}
        if(Action==TEXT("fail_pickup"))V->bIncidentPickupFailure=true;else V->bIncidentPlacementFailure=true;
        bIncidentInputUsed=true;RecordRecoveryEvent(Action,V,V,V,V->CargoActor.Get());return true;
    }
    Error=TEXT("지원하지 않는 상황입니다.");return false;
}

void APortSiteLogistics::AdvanceIncidentScenarios(float Dt)
{
    if(auto* Controller=Cast<AQuayCrane>(GetOwner()))
        if(!Controller->bAutoRunning||Controller->bAutoPaused||Controller->bEmergencyStop)return;
    if(!Fault.IsEmpty()||Dt<=0)return;
    IncidentClock+=Dt; // Weather hold freezes machines, not weather/recovery scheduling.
    for(const auto& V:Vehicles)V->AdvanceNavigationIncidents(Dt);
    for(const auto& V:Vehicles)if(V->bIncidentSensorFault&&V->IncidentRebootRemaining>=0)
    {
        V->IncidentRebootRemaining=FMath::Max(0.,V->IncidentRebootRemaining-Dt);
        if(V->IncidentRebootRemaining==0)
        {
            V->bIncidentSensorFault=false;V->IncidentSensorPhase=TEXT("confirming");V->bIncidentNeedsPositionFix=true;
            const int32 Lane=Vehicles.IndexOfByKey(V);
            V->ReacquirePositionAtNode(FleetPark(Lane));
            if(Jobs.IsValidIndex(Lane)&&Jobs[Lane].Route.IsValidIndex(Jobs[Lane].Waypoint))V->ReacquirePositionAtNode(Jobs[Lane].Route[Jobs[Lane].Waypoint]);
            RecordRecoveryEvent(TEXT("sensor_rebooted"),V,V,V);
        }
    }
    for(auto& R:IncidentRuns)for(auto& S:R.Steps)
    {
        if(Terminal(S.State))continue;
        const FString Action=String(*S.Command,TEXT("action")),ID=String(*S.Command,TEXT("actor_id"));
        APortAGVActor* V=nullptr;APortWorkingCrane* K=nullptr;
        for(const auto& A:Vehicles)if(A->GetName()==ID)V=A;
        for(const auto& A:Equipment)if(A->GetName()==ID)K=A;
        if((S.State==TEXT("pending")&&IncidentClock-R.StartedAt>S.Delay+S.Timeout)||
            (S.State==TEXT("awaiting_completion")&&IncidentClock-S.RecoveredAt>S.Timeout))
        {S.State=TEXT("timeout");S.Message=TEXT("발생 조건 또는 복구 후 작업 완료 확인 시간 초과");continue;}
        if(S.State==TEXT("pending"))
        {
            if(IncidentClock-R.StartedAt<S.Delay)continue;
            const FString T=String(*S.Command,TEXT("trigger"),TEXT("immediate"));
            const bool Ready=T==TEXT("immediate")||(T==TEXT("moving")&&V&&V->Speed>10)||
                (T==TEXT("loaded")&&V&&V->PayloadKg>0&&Jobs[Vehicles.IndexOfByKey(V)].Stage==3)||
                (T==TEXT("pickup")&&K&&K->IsBusy()&&K->Stage==2&&!K->RetainsLoad())||
                (T==TEXT("placing")&&K&&K->IsBusy()&&K->Stage==5)||
                (T==TEXT("release")&&K&&K->IncidentReleaseWindow())||
                (T==TEXT("handover")&&K&&K->IncidentHandoverWindow());
            if(!Ready)continue;
            const bool Existing=(V&&((Action==TEXT("fail")&&V->bDriveFault)||(PortIncident::SensorFailure(Action)&&V->bIncidentSensorFault)||
                (Action==TEXT("misdelivery")&&V->bIncidentDestinationMismatch)||(Action==TEXT("worker_enter")&&V->Emergency.Active)))||
                (K&&((Action==TEXT("fail_gantry")&&K->bGantryFault)||(PortIncident::SensorFailure(Action)&&K->bSensorFault)||
                (Action==TEXT("fail_pickup")&&K->bIncidentPickupFailure)||(Action==TEXT("fail_placement")&&K->bIncidentPlacementFailure)))||
                (V&&V->IncidentFaults.Contains(Action))||(K&&K->IncidentFaults.Contains(Action));
            if(Existing){S.State=TEXT("rejected");S.Message=TEXT("이미 존재하던 같은 고장은 이 시나리오의 복구 대상으로 인계하지 않습니다.");continue;}
            // Keep the affected cargo even if isolation immediately reassigns its job.
            S.CompletedBefore=V?V->CompletedJobs:K?K->CompletedJobs:0;
            if(K)S.Cargo=K->CargoActor;
            if(V)
            {
                const auto& J=Jobs[Vehicles.IndexOfByKey(V)];
                S.Cargo=J.Actor;
                if(!S.Cargo.IsValid() && (J.Stage==7 || J.Stage==8) && PreparedCargo.IsValidIndex(J.STS) &&
                    Manifest.IsValidIndex(PreparedCargo[J.STS]))
                    S.Cargo=Manifest[PreparedCargo[J.STS]].Actor;
            }
            S.ExpectShip=S.Cargo.IsValid()&&S.Cargo->LocationOwner==ECargoOwner::Yard;
            if(V&&Jobs[Vehicles.IndexOfByKey(V)].ExportIndex!=INDEX_NONE)S.ExpectShip=true;
            FJsonObject Command=*S.Command;
            // Isolation may reassign work inside ApplyDashboardCommand itself.
            // Register provenance before it runs, and discard it on rejection.
            S.MeasureAppliedAt=IncidentMeasureClock;
            if(ID!=TEXT("environment"))S.AffectedActors.Add(ID);
            if(S.Cargo.IsValid())S.AffectedCargo.Add(S.Cargo->GetName());
            FString Error;
            if(!ApplyDashboardAction(Command,Error))
            {
                S.MeasureAppliedAt=-1;S.AffectedActors.Reset();S.AffectedCargo.Reset();
                // A conditional worker event waits for a safely reserved straight,
                // without changing the route, cargo or clock to force admission.
                const bool WaitForRoad=(Action==TEXT("worker_enter")||Action==TEXT("road_block"))&&V&&!V->Emergency.Active&&T!=TEXT("immediate")&&
                    (Error.Contains(TEXT("도로"))||Error.Contains(TEXT("직선")));
                if(!WaitForRoad)S.State=TEXT("rejected");S.Message=Error;continue;
            }
            S.State=TEXT("active");S.AppliedAt=IncidentClock;S.Message=TEXT("엔진 적용 · 실제 대응 확인 중");
            if(Action==TEXT("set_environment")){S.Observed=true;S.State=TEXT("verified");S.Message=TEXT("날씨 입력 적용 확인 · 노면 잔류·기상 대기는 실제 환경에서 계속 계산");}
        }
        if(S.State==TEXT("verified"))continue;
        bool Active=false,Observed=false;
        if(PortIncident::Extension(Action)){Active=ExtendedIncidentActive(Action,V,K);Observed=ExtendedIncidentObserved(Action,V,K);}
        if(V)
        {
            if(Action==TEXT("fail"))
            {
                Active=V->bDriveFault||IncidentRoadClosures.Contains(TEXT("drive:")+V->GetName());
                Observed=Active&&V->Speed<=.1f;
            }
            if(PortIncident::SensorFailure(Action)){Active=V->bIncidentSensorFault||V->IncidentSensorPhase!=TEXT("healthy");Observed=V->bIncidentSensorFault&&!V->Sensors.bTransponderLocked;}
            if(Action==TEXT("worker_enter")){Active=V->Emergency.Active;Observed=Active&&V->Emergency.Stopped;}
            if(Action==TEXT("misdelivery")){Active=V->bIncidentDestinationMismatch;Observed=Active&&V->Sensors.bFMSHold&&V->Speed<=.1f&&Jobs[Vehicles.IndexOfByKey(V)].Stage==3;}
        }
        if(K)
        {
            if(Action==TEXT("fail_pickup")){Active=K->bIncidentPickupFailure||K->IsPickupUnavailable();Observed=Active&&K->Stage==2&&K->IsBusy();}
            if(Action==TEXT("fail_placement")){Active=K->bIncidentPlacementFailure;Observed=Active&&K->Stage==5&&K->RetainsLoad();}
            if(Action==TEXT("fail_gantry")){Active=K->bGantryFault;Observed=Active&&K->IsEquipmentUnavailable();}
            if(PortIncident::SensorFailure(Action)){Active=K->bSensorFault;Observed=Active&&!K->Observation.bValid;}
        }
        S.Observed|=Observed;
        if(Active&&S.State!=TEXT("recovering")&&PortIncident::SensorFailure(Action))
            S.State=TEXT("recovering");
        if(Active&&S.State!=TEXT("recovering")&&S.Duration>0&&IncidentClock-S.AppliedAt>=S.Duration)
        {
            FJsonObject Recovery;
            Recovery.SetStringField(TEXT("actor_id"),ID);Recovery.SetStringField(TEXT("action"),TEXT("incident_clear"));Recovery.SetStringField(TEXT("fault_action"),Action);
            FString Error;if(!ApplyDashboardAction(Recovery,Error))S.Message=Error;
            else {S.State=TEXT("recovering");S.Message=TEXT("복구 요청 접수 · 재부팅·재인식·실제 복구 확인 중");}
            continue; // Verify resulting state at the next real scene boundary.
        }
        if(!Active)
        {
            if(S.RecoveredAt<0)S.RecoveredAt=IncidentClock;
            if(!S.Observed){S.State=TEXT("not_exercised");S.Message=TEXT("복구됐지만 실제 오류 대응을 관측하지 못함 · 검증 미통과");continue;}
            S.State=TEXT("awaiting_completion");S.Message=TEXT("오류 해제 · 실제 후속 작업 완료 대기");
            const bool DeliveredCargo=S.Cargo.IsValid()&&S.Cargo->LocationOwner==(S.ExpectShip?ECargoOwner::Ship:ECargoOwner::Yard)&&!S.Cargo->GetBody()->IsSimulatingPhysics();
            const bool Completed=V?V->CompletedJobs>S.CompletedBefore:K?K->CompletedJobs>S.CompletedBefore:false;
            if((S.Cargo.IsValid()?DeliveredCargo:Completed)&&IncidentClock>S.RecoveredAt+.1)
            {S.State=TEXT("verified");S.Message=TEXT("오류 대응·복구 및 실제 작업 완료 확인");}
        }
        else if(S.Observed)S.Message=S.State==TEXT("recovering")?TEXT("복구 요청 접수 · 재부팅·재인식·실제 복구 확인 중"):TEXT("실제 오류 대응 확인 · 복구 대기");
    }
}

TSharedPtr<FJsonObject> APortSiteLogistics::CaptureIncidentScenarios() const
{
    auto O=MakeShared<FJsonObject>();O->SetNumberField(TEXT("clock_seconds"),IncidentClock);
    TArray<TSharedPtr<FJsonValue>> Runs;
    for(const auto& R:IncidentRuns)
    {
        auto Run=MakeShared<FJsonObject>();Run->SetStringField(TEXT("id"),R.ID);Run->SetStringField(TEXT("name"),R.Name);
        Run->SetNumberField(TEXT("started_at"),R.StartedAt);Run->SetBoolField(TEXT("cancelled"),R.Cancelled);
        TArray<TSharedPtr<FJsonValue>> Steps;
        for(const auto& S:R.Steps)
        {
            auto Step=MakeShared<FJsonObject>(*S.Command);Step->SetStringField(TEXT("state"),S.State);Step->SetStringField(TEXT("message"),S.Message);
            Step->SetNumberField(TEXT("applied_at"),S.AppliedAt);Step->SetNumberField(TEXT("recovered_at"),S.RecoveredAt);
            Step->SetNumberField(TEXT("recovery_seconds"),S.Duration);
            if(S.Duration>0&&S.AppliedAt>=0&&S.RecoveredAt<0)
            {
                double Remaining=FMath::Max(0.,S.Duration-(IncidentClock-S.AppliedAt));
                if(PortIncident::SensorFailure(String(*S.Command,TEXT("action"))))
                    for(const auto& K:Equipment)if(K->GetName()==String(*S.Command,TEXT("actor_id"))&&K->bSensorFault)
                        Remaining=FMath::Max(0.,K->SensorRepairRemaining());
                Step->SetNumberField(TEXT("recovery_remaining_seconds"),Remaining);
            }
            Step->SetNumberField(TEXT("measurement_applied_at"),S.MeasureAppliedAt);
            Step->SetNumberField(TEXT("fault_cleared_at"),S.MeasureClearedAt);
            Step->SetNumberField(TEXT("work_resumed_at"),S.MeasureResumedAt);
            Step->SetNumberField(TEXT("work_completed_at"),S.MeasureCompletedAt);
            auto AddSet=[&](const TCHAR* Key,const TSet<FString>& Set)
            {auto Sorted=Set.Array();Sorted.Sort();TArray<TSharedPtr<FJsonValue>> Values;for(const auto& V:Sorted)Values.Add(MakeShared<FJsonValueString>(V));Step->SetArrayField(Key,Values);};
            AddSet(TEXT("affected_actors"),S.AffectedActors);AddSet(TEXT("affected_cargo"),S.AffectedCargo);
            Step->SetBoolField(TEXT("response_observed"),S.Observed);Step->SetStringField(TEXT("cargo"),GetNameSafe(S.Cargo.Get()));
            Steps.Add(MakeShared<FJsonValueObject>(Step));
        }
        Run->SetArrayField(TEXT("steps"),Steps);Runs.Add(MakeShared<FJsonValueObject>(Run));
    }
    O->SetArrayField(TEXT("runs"),Runs);return O;
}
