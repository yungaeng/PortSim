#include "PortSiteLogistics.h"
#include "PortWorkingCrane.h"
#include "PortAGVActor.h"
#include "PortContainerActor.h"
#include "AGVTrafficGeometry.h"
#include "Dom/JsonObject.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"

namespace
{
    bool IntersectsControlZone(const FPortIncidentRoadClosure& Zone,FVector From,FVector To)
    {
        if(Zone.bDriveFaultZone)
            return AGVTrafficGeometry::SweptChassisIntersects(From,To,Zone.Center,Zone.Longitudinal,730,210,0,
                Zone.HalfLength+Zone.Clearance,Zone.HalfWidth+Zone.Clearance);
        const FVector Direction=(To-From).GetSafeNormal2D(),Side(-Direction.Y,Direction.X,0);
        const FVector Margin=Direction.GetAbs()*730+Side.GetAbs()*210+FVector(0,0,500);
        const FBox Bounds=Zone.Bounds.ExpandBy(Margin);
        return Bounds.IsInsideOrOn(From)||Bounds.IsInsideOrOn(To)||FMath::LineBoxIntersection(Bounds,From,To,To-From);
    }
}

bool APortSiteLogistics::HasIncidentRoadClosure() const
{for(const auto& R:IncidentRoadClosures)if(R.Value.Active)return true;return false;}

bool APortSiteLogistics::IsIncidentYardBlocked(int32 Slot) const
{
    for(const auto& Entry:IncidentBlockedYardSlots) if(Entry.Value==Slot) return true;
    if(IncidentRoadClosures.IsEmpty()||!Yard.IsValidIndex(Slot)) return false;
    const FVector Pad=YardHandover(Yard[Slot]);
    for(const auto& Entry:IncidentRoadClosures)
        if(Entry.Value.Active&&Entry.Value.bDriveFaultZone&&
            IntersectsControlZone(Entry.Value,Pad,Pad+FVector(.1,0,0))) return true;
    return false;
}

bool APortSiteLogistics::IncidentRoadBlocked(FVector A,FVector B,bool Record)
{
    return FindIncidentRoadClosure(A,B,Record)!=nullptr;
}

FPortIncidentRoadClosure* APortSiteLogistics::FindIncidentRoadClosure(FVector A,FVector B,bool Record)
{
    for(auto& R:IncidentRoadClosures)if(R.Value.Active)
    {
        auto& Zone=R.Value;
        if(!IntersectsControlZone(Zone,A,B)) continue;
        if(Record)Zone.Responded=true;
        return &Zone;
    }
    return nullptr;
}

bool APortSiteLogistics::ApplyExtendedIncident(const FJsonObject& C,FString& Error,bool Clear)
{
    const FString ID=C.GetStringField(TEXT("actor_id")),Action=C.GetStringField(Clear?TEXT("fault_action"):TEXT("action"));
    for(const auto& V:Vehicles)if(V->GetName()==ID)
    {
        if(!PortIncident::AGVExtension(Action)){Error=TEXT("AGV에 지원하지 않는 돌발상황입니다.");return false;}
        if(!Clear&&V->IncidentFaults.Contains(Action)){Error=TEXT("이미 발생한 돌발상황입니다.");return false;}
        if(Action==TEXT("road_block"))
        {
            if(Clear)
            {
                if(auto* R=IncidentRoadClosures.Find(ID)){R->Active=false;if(R->Marker.IsValid())R->Marker->Destroy();}
            }
            else
            {
                const int32 Lane=Vehicles.IndexOfByKey(V);if(!Jobs.IsValidIndex(Lane)){Error=TEXT("배정된 도로 작업이 없습니다.");return false;}const auto& J=Jobs[Lane];
                if(!J.Route.IsValidIndex(J.Waypoint)){Error=TEXT("이동 중인 AGV의 직선 도로를 선택하세요.");return false;}
                const FVector From=V->GetActorLocation(),D=(J.Route[J.Waypoint]-From).GetSafeNormal2D();
                const double Distance=FMath::Max(3500.,FMath::Square(double(V->Speed))/(2*FMath::Max(1.f,V->BrakingCmPerSecondSquared()))+1800.);
                double Straight=0;FVector Previous=From;
                for(int32 N=J.Waypoint;J.Route.IsValidIndex(N);++N)
                {const FVector Segment=J.Route[N]-Previous;if(FVector::DotProduct(Segment.GetSafeNormal2D(),D)<.999)break;Straight+=Segment.Size2D();Previous=J.Route[N];}
                if((FMath::Abs(D.X)<.999&&FMath::Abs(D.Y)<.999)||Straight<Distance+800)
                {Error=TEXT("제동거리 뒤에 차단물을 놓을 직선 도로가 부족합니다.");return false;}
                const FVector Center=From+D*Distance,Side(-D.Y,D.X,0),E=D.GetAbs()*400+Side.GetAbs()*260+FVector(0,0,200);
                FPortIncidentRoadClosure R;R.Bounds=FBox(Center-E,Center+E);R.Cause=V;R.Center=Center;
                double Duration=0.;
                if(C.TryGetNumberField(TEXT("duration"),Duration)&&FMath::IsFinite(Duration)&&Duration>0&&Duration<=3600)
                    R.ExpectedClearAt=IncidentClock+Duration;
                // Reject insertion into an occupied chassis envelope; never push a vehicle aside.
                for(const auto& Other:TrafficVehicles)if(IsValid(Other)&&R.Bounds.ExpandBy(800).IsInsideOrOn(Other->GetActorLocation()))
                {Error=TEXT("차단 위치에 다른 AGV가 있어 안전한 도로 공간을 기다립니다.");return false;}
                auto* Marker=GetWorld()->SpawnActor<AActor>(Center,FRotator::ZeroRotator);
                auto* Mesh=NewObject<UStaticMeshComponent>(Marker);Mesh->SetStaticMesh(LoadObject<UStaticMesh>(nullptr,TEXT("/Engine/BasicShapes/Cube.Cube")));
                Mesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);Marker->SetRootComponent(Mesh);Mesh->RegisterComponent();Marker->AddInstanceComponent(Mesh);
                Marker->SetActorLocation(Center+FVector(0,0,100));Marker->SetActorScale3D(FVector(E.X*.02,E.Y*.02,2));R.Marker=Marker;
                IncidentRoadClosures.Add(ID,MoveTemp(R));
            }
        }
        if(Clear)
        {
            V->IncidentFaults.Remove(Action);
            if(Action==TEXT("localization_loss"))
            {
                const int32 Lane=Vehicles.IndexOfByKey(V);const auto& J=Jobs[Lane];
                if(J.Stage==0)V->ReacquirePositionAtNode(FleetPark(Lane));
                else if(J.Route.IsValidIndex(J.Waypoint))V->ReacquirePositionAtNode(J.Route[J.Waypoint]);
            }
            if(Action==TEXT("fms_delay"))V->bIncidentFMSPending=false;
        }
        else
        {
            V->IncidentFaults.Add(Action);V->IncidentResponses.Remove(Action);
            if(Action==TEXT("localization_loss"))
            {V->Sensors.bTransponderLocked=false;V->bIncidentNeedsPositionFix=true;V->IncidentPositionUncertaintyCm=FMath::Max(2.5,double(V->Sensors.PositionErrorCm));V->IncidentWheelMarkCm=V->Sensors.WheelOdometryCm;}
            if(Action==TEXT("fms_disconnect"))V->Sensors.bFMSConnected=false;
            if(Action==TEXT("fms_delay")){V->IncidentPendingFMSNode=INDEX_NONE;V->bIncidentFMSPending=true;}
        }
        bIncidentInputUsed=true;RecordRecoveryEvent(Clear?TEXT("incident_repaired"):Action,V,V,V);return true;
    }
    for(const auto& K:Equipment)if(K->GetName()==ID)
    {
        if(!PortIncident::CraneExtension(Action,!K->bSTS)){Error=TEXT("장비에 지원하지 않는 돌발상황입니다.");return false;}
        if(!K->Fault.IsEmpty()&&!K->IsEquipmentUnavailable()){Error=TEXT("기존 안전 오류를 먼저 해결하세요.");return false;}
        const bool ControllerActive=PortIncident::ControllerFailure(Action)&&
            (K->IncidentFaults.Contains(TEXT("equipment_failure"))||K->IncidentFaults.Contains(TEXT("restart_controller")));
        if(!Clear&&(K->IncidentFaults.Contains(Action)||ControllerActive)){Error=TEXT("이미 발생한 돌발상황입니다.");return false;}
        if(Action==TEXT("slot_occupied"))
        {
            if(Clear)IncidentBlockedYardSlots.Remove(ID);
            else
            {
                int32 Slot=INDEX_NONE;
                for(const auto& J:Jobs)if(J.Stage==4&&Equipment.IsValidIndex(J.RMG)&&Equipment[J.RMG]==K&&Yard.IsValidIndex(J.Slot)){Slot=J.Slot;break;}
                if(Slot==INDEX_NONE){Error=TEXT("실제 적재 목적지가 배정된 RMG 하강 단계에서 발생시키세요.");return false;}
                IncidentBlockedYardSlots.Add(ID,Slot);
            }
        }
        if(Clear)K->IncidentFaults.Remove(Action);else {K->IncidentFaults.Add(Action);K->IncidentResponses.Remove(Action);}
        if(PortIncident::ControllerFailure(Action))
        {K->SetEquipmentFailure(!Clear);if(!Clear)K->IncidentResponses.Add(Action);ManageEquipmentFailures();}
        bIncidentInputUsed=true;RecordRecoveryEvent(Clear?TEXT("incident_repaired"):Action,K,K,K,K->CargoActor.Get());return true;
    }
    Error=TEXT("현재 실행의 장비가 아닙니다.");return false;
}

bool APortSiteLogistics::ExtendedIncidentActive(const FString& A,const APortAGVActor* V,const APortWorkingCrane* K) const
{
    if(V)
    {
        if(A==TEXT("localization_loss"))return V->IncidentFaults.Contains(A)||V->bIncidentNeedsPositionFix;
        if(A==TEXT("fms_disconnect"))return V->IncidentFaults.Contains(A)||!V->Sensors.bFMSConnected;
        return V->IncidentFaults.Contains(A);
    }
    return K&&K->IncidentFaults.Contains(A);
}

bool APortSiteLogistics::ExtendedIncidentObserved(const FString& A,const APortAGVActor* V,const APortWorkingCrane* K) const
{
    if(A==TEXT("road_block")&&V){const auto* R=IncidentRoadClosures.Find(V->GetName());return R&&R->Responded;}
    return V?V->IncidentResponses.Contains(A):K&&K->IncidentResponses.Contains(A);
}
