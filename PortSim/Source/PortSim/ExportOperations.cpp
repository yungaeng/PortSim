#include "TerminalReplay.h"
#include "PortSiteLogistics.h"
#include "PortContainerActor.h"
#include "PortAGVActor.h"
#include "PortWorkingCrane.h"
#include "Components/StaticMeshComponent.h"

int32 APortSiteLogistics::LoadedCount() const
{
    int32 Count=0;
    for(const auto& Booking:ExportCargo) Count+=Booking.State==2;
    return Count;
}

FVector APortSiteLogistics::TransferQuay(const FSiteTransfer& Job) const
{
    return Job.ExportIndex==INDEX_NONE?CargoQuay(Job.Cargo):QuayPark(Job.STS);
}

void APortSiteLogistics::ScheduleExport()
{
    if(ExportCargo.IsEmpty()) return;
    int32 Next=INDEX_NONE;
    for(int32 I=0;I<ExportCargo.Num();++I)
    {
        const auto& Candidate=ExportCargo[I];
        if(Candidate.State!=0 || !Yard.IsValidIndex(Candidate.PlannedSlot)) continue;
        const auto& Slot=Yard[Candidate.PlannedSlot];
        const int32 STS=Candidate.STS, RMG=Slot.Crane;
        if(STSOwners[STS]!=INDEX_NONE || PreparedCargo[STS]!=INDEX_NONE || NextVehicles[STS]!=INDEX_NONE ||
            !CraneAvailable(YardCraneCount+STS) || Equipment[YardCraneCount+STS]->IsBusy() ||
            !CraneAvailable(RMG) || Equipment[RMG]->IsBusy() || RMGBusy[RMG] || BlocksBusy[Slot.Block] ||
            YardApproachOwners[YardApproachKey(Slot)]!=INDEX_NONE) continue;
        auto SameColumn=[&](const FSiteShipCargo& C)
        {
            return C.STS/3==STS/3 && C.ShipSlot.Bay==Candidate.ShipSlot.Bay &&
                C.ShipSlot.Row==Candidate.ShipSlot.Row;
        };
        // Clear this column before loading; other columns can discharge concurrently.
        if(Manifest.ContainsByPredicate([&](const FSiteShipCargo& C){return SameColumn(C) && C.State!=2;})) continue;
        if(Candidate.ShipSlot.Tier>1)
        {
            auto Supports=[&](const FSiteShipCargo& C)
            { return SameColumn(C) && C.ShipSlot.Tier==Candidate.ShipSlot.Tier-1; };
            if(!RetainedCargo.ContainsByPredicate(Supports) &&
                !ExportCargo.ContainsByPredicate([&](const FSiteShipCargo& C){return C.State==2 && Supports(C);})) continue;
        }
        if(Next==INDEX_NONE || Candidate.ShipSlot.Tier<ExportCargo[Next].ShipSlot.Tier) Next=I;
    }
    if(Next==INDEX_NONE) return;
    const int32 Lane=Jobs.IndexOfByPredicate([this](const FSiteTransfer& J)
    {
        const int32 Index=&J-Jobs.GetData();
        return J.Stage==0 && Vehicles[Index]->CanAcceptWork();
    });
    if(Lane==INDEX_NONE) return;
    auto& Booking=ExportCargo[Next];
    auto& Job=Jobs[Lane];
    Job.ExportIndex=Next; Job.Actor=Booking.Actor; Job.Slot=Booking.PlannedSlot;
    Job.STS=Booking.STS; Job.RMG=Yard[Job.Slot].Crane;
    Job.StartedAt=SimulationTime; Job.Stage=6;
    Job.bRMGReserved=true;
    Booking.State=1;
    STSOwners[Job.STS]=Lane; RMGBusy[Job.RMG]=true;
    YardApproachOwners[YardApproachKey(Yard[Job.Slot])]=Lane;
    if(!PlanRoadRoute(Lane,false)) Stop(TEXT("적하 AGV의 초기 안벽 접근 경로를 만들 수 없습니다."));
}

void APortSiteLogistics::AdvanceExport(int32 Lane,float Dt)
{
    auto& Job=Jobs[Lane];
    auto& Booking=ExportCargo[Job.ExportIndex];
    auto* Cargo=Booking.Actor.Get();
    auto* Vehicle=Vehicles[Lane].Get();
    auto* RMG=Equipment[Job.RMG].Get();
    auto* STS=Equipment[YardCraneCount+Job.STS].Get();
    if(Job.Time>12000) { Stop(TEXT("적하 작업 제한 시간 초과")); return; }
    switch(Job.Stage)
    {
    case 6:
        if(!Drive(Lane,Dt)) return;
        if(Booking.HandoverMask==0)
        {
            Job.Stage=3;
            if(!PlanRoadRoute(Lane,false)) Stop(TEXT("적하 컨테이너 야드 접근 경로 오류"));
        }
        else
        {
            FVector Target=Booking.Transform.GetLocation(),Velocity,Acceleration,Jerk;
            FQuat Rotation;
            ResolveVesselTransform(Booking,Target,Rotation,Velocity,Acceleration,Jerk);
            if(!STS->AssignCargo(Cargo,Vehicle->CargoPosition(),Target,false,false,Vehicle,true))
            { Stop(TEXT("적하 STS 인계 실패: ")+STS->Fault); return; }
            STSOwners[Job.STS]=Lane;
            Booking.HandoverMask|=2; Job.Stage=1;
        }
        break;
    case 3:
        if(!Drive(Lane,Dt)) return;
        if(!RMG->AssignCargo(Cargo,Cargo->GetActorLocation(),Vehicle->CargoPosition(),false,false,Vehicle,true))
        { Stop(TEXT("적하 RMG 인계 실패: ")+RMG->Fault); return; }
        Job.Stage=4;
        break;
    case 4:
        if(RMG->IsBusy()) return;
        if(!Cargo->GetActorLocation().Equals(Vehicle->CargoPosition(),10.f) || Vehicle->Speed>.1f)
        { Stop(TEXT("적하 RMG/AGV 정렬 오류")); return; }
        Cargo->GetBody()->SetSimulatePhysics(false);
        Cargo->SetActorLocationAndRotation(Vehicle->CargoPosition(),Vehicle->GetActorRotation(),false,nullptr,ETeleportType::TeleportPhysics);
        Cargo->AttachToComponent(Vehicle->GetRootComponent(),FAttachmentTransformRules::KeepWorldTransform);
        Cargo->LocationOwner=ECargoOwner::AGV;
        Vehicle->SetPayload(Cargo->MassKg);
        Yard[Job.Slot].Occupied=false;
        Booking.HandoverMask|=1;
        Job.Stage=5; Job.ReturnSTS=Job.STS;
        if(!PlanRoadRoute(Lane,false)) Stop(TEXT("적하 AGV 안벽 복귀 경로 오류"));
        break;
    case 5:
        if(!Drive(Lane,Dt)) return;
        RMGBusy[Job.RMG]=false;
        YardApproachOwners[YardApproachKey(Yard[Job.Slot])]=INDEX_NONE;
        Job.ReturnSTS=INDEX_NONE; Job.Stage=6;
        if(!PlanRoadRoute(Lane,false)) Stop(TEXT("적하 STS 최종 접근 경로 오류"));
        break;
    case 1:
    {
        FVector Target=Booking.Transform.GetLocation(),Velocity,Acceleration,Jerk;
        FQuat Rotation=FQuat::Identity;
        ResolveVesselTransform(Booking,Target,Rotation,Velocity,Acceleration,Jerk);
        STS->UpdateShipLandingTarget(Target);
        if(STS->IsBusy()) return;
        if(Cargo->LocationOwner!=ECargoOwner::Ship || !Cargo->GetActorLocation().Equals(Target,20.f))
        { Stop(TEXT("적하 선박 배치 오류")); return; }
        Cargo->DetachFromActor(FDetachmentTransformRules::KeepWorldTransform);
        Cargo->GetBody()->SetSimulatePhysics(false);
        Cargo->SetSecuredVesselMotion(Target,Rotation,Velocity,Acceleration,Jerk);
        Booking.State=2; Booking.HandoverMask|=4;
        Vehicle->SetPayload(0); ++Vehicle->CompletedJobs;
        Job.Actor=nullptr; Job.Stage=FSiteTransfer::ReturnToStandby;
        PORTSIM_SIM_LOG(LogTemp,Display,TEXT("SITE_LOADED: %s vessel=%d bay=%d row=%d tier=%d"),
            *Booking.ContainerName.ToString(),Booking.STS/3+1,Booking.ShipSlot.Bay,Booking.ShipSlot.Row,Booking.ShipSlot.Tier);
        if(!PlanRoadRoute(Lane,false)) Stop(TEXT("적하 완료 AGV 대기소 복귀 경로 오류"));
        break;
    }
    case FSiteTransfer::ReturnToStandby:
        if(!Drive(Lane,Dt)) return;
        STSOwners[Job.STS]=INDEX_NONE;
        Job=FSiteTransfer();
        Vehicle->SetFMSHold(false);
        break;
    default: Stop(TEXT("알 수 없는 적하 작업 단계")); break;
    }
}
