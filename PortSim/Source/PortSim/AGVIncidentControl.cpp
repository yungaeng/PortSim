#include "PortSiteLogistics.h"
#include "PortAGVActor.h"
#include "PortContainerActor.h"
#include "PortWorkingCrane.h"
#include "AGVDispatchPolicy.h"
#include "AGVApronRoads.h"
#include "AGVReference.h"

namespace
{
    FString DriveZoneKey(const APortAGVActor& Vehicle)
    {
        return TEXT("drive:")+Vehicle.GetName();
    }
}

void APortSiteLogistics::ReleaseFutureRoadClaims(int32 ID)
{
    // Physical occupancy remains in VehicleIndex even with no future lease.
    RoadReservations.Remove(ID);
    RoadReservationEndWaypoint.Remove(ID);
    ReservationIndex.Remove(ID);
    RoadTargets.Remove(ID);
    FinishedRoadSegments.Remove(ID);
    TrafficRecoveryTargets.Remove(ID);
    TrafficRecoveryOriginalReverse.Remove(ID);
    TrafficRecoveryYieldTo.Remove(ID);
    ClearPendingRoadRequest(ID);
}

void APortSiteLogistics::SetAGVIncidentDriveFailure(int32 Lane,bool Failed,double ClearanceM,double RepairDelay)
{
    auto& Vehicle=*Vehicles[Lane];
    if(Vehicle.bDriveFault==Failed) return;
    Vehicle.SetDriveFailure(Failed);
    if(!Failed) return;

    FPortIncidentRoadClosure Zone;
    Zone.Cause=&Vehicle;
    Zone.bDriveFaultZone=true;
    Zone.Center=Vehicle.GetActorLocation();
    Zone.Longitudinal=Vehicle.GetActorRightVector().GetSafeNormal2D();
    Zone.HalfLength=AGVReference::PhysicalHalfLengthCm;
    Zone.HalfWidth=AGVReference::PhysicalHalfWidthCm;
    Zone.Clearance=ClearanceM*100.;
    Zone.ExpectedClearAt=RepairDelay>0?IncidentClock+RepairDelay:-1.;

    const FVector Side(-Zone.Longitudinal.Y,Zone.Longitudinal.X,0);
    for(const auto& Entry:MovingCargo)
    {
        const auto* Cargo=Entry.Get();
        if(!IsValid(Cargo)||Cargo->LocationOwner!=ECargoOwner::AGV||Cargo->GetAttachParentActor()!=&Vehicle) continue;
        const FVector Offset=Cargo->GetActorLocation()-Zone.Center;
        Zone.HalfLength=FMath::Max(Zone.HalfLength,FMath::Abs(FVector::DotProduct(Offset,Zone.Longitudinal))+Cargo->ExternalDimensionsCm.Y*.5);
        Zone.HalfWidth=FMath::Max(Zone.HalfWidth,FMath::Abs(FVector::DotProduct(Offset,Side))+Cargo->ExternalDimensionsCm.X*.5);
    }
    const FVector Extent=Zone.Longitudinal.GetAbs()*(Zone.HalfLength+Zone.Clearance)+
        Side.GetAbs()*(Zone.HalfWidth+Zone.Clearance)+FVector(0,0,500);
    Zone.Bounds=FBox(Zone.Center-Extent,Zone.Center+Extent);
    IncidentRoadClosures.Add(DriveZoneKey(Vehicle),Zone);
    ReleaseFutureRoadClaims(Vehicle.VehicleID);
    RoadBlockers.Remove(Vehicle.VehicleID);
    RoadWaitSince.Remove(Vehicle.VehicleID);
    bIncidentInputUsed=true;
    RecordRecoveryEvent(TEXT("road_isolated"),&Vehicle,&Vehicle,&Vehicle,Jobs[Lane].Actor.Get(),Zone.Center,Zone.Center);
}

void APortSiteLogistics::RefreshAGVIncidentControl()
{
    TArray<int32> FailedPickups;
    for(auto It=IncidentRoadClosures.CreateIterator();It;++It)
    {
        auto& Zone=It.Value();
        if(!Zone.bDriveFaultZone) continue;
        auto* Vehicle=Cast<APortAGVActor>(Zone.Cause.Get());
        if(!Vehicle) {It.RemoveCurrent();continue;}
        const int32 Lane=Vehicles.IndexOfByKey(Vehicle);
        if(!Jobs.IsValidIndex(Lane)) {It.RemoveCurrent();continue;}
        if(!Vehicle->bDriveFault&&!Vehicle->bIncidentSensorFault&&!Vehicle->Emergency.Active&&
            !Vehicle->IncidentFaults.Contains(TEXT("fms_disconnect"))&&!Vehicle->bIncidentFMSPending&&
            !Vehicle->IncidentFaults.Contains(TEXT("localization_loss"))&&
            Vehicle->Dynamics.Fault.IsEmpty()&&Vehicle->Speed<=.1f)
        {
            RecordRecoveryEvent(TEXT("road_reopened"),Vehicle,Vehicle,Vehicle);
            It.RemoveCurrent();
            continue;
        }
        if(IncidentClock>=Zone.NextDispatchAttempt)
        {
            Zone.NextDispatchAttempt=IncidentClock+5.;
            FailedPickups.Add(Lane);
        }
    }
    for(int32 Lane:FailedPickups) ReassignIncidentPickup(Lane);

    // Cranes consult the bound vehicle before starting or continuing an
    // actual handover. A closure elsewhere on its route does not stop lifting.
    for(const auto& Vehicle:Vehicles)
    {
        const FVector At=Vehicle->GetActorLocation();
        Vehicle->bIncidentHandoverHold=FindIncidentRoadClosure(At,At+Vehicle->GetActorRightVector()*.1)!=nullptr;
    }
    if(LaneCount!=9||!HasIncidentRoadClosure()||IncidentClock<NextIncidentYardPlanAt) return;
    NextIncidentYardPlanAt=IncidentClock+5.;
    for(int32 Lane=0;Lane<Jobs.Num();++Lane)
    {
        const auto& Job=Jobs[Lane];
        if(Job.Stage!=9||Job.ExportIndex!=INDEX_NONE||Vehicles[Lane]->bDriveFault||
            !Manifest.IsValidIndex(Job.Cargo)||!IsIncidentYardBlocked(Job.Slot)) continue;
        const FVector Pad=YardHandover(Yard[Job.Slot]);
        auto* Zone=FindIncidentRoadClosure(Pad,Pad+FVector(.1,0,0));
        if(!Zone||!Zone->bDriveFaultZone) continue;
        TSet<int32> Affected={Job.Cargo};
        if(Manifest.IsValidIndex(Job.ExtraCargo)) Affected.Add(Job.ExtraCargo);
        ReplanYardGroup(Affected,{},Zone->Cause.Get());
    }
}

bool APortSiteLogistics::StartRoadRoute(int32 Lane)
{
    auto* Zone=LoadedDepartureIncident(Lane);
    if(PlanRoadRoute(Lane,false))
    {
        IncidentRouteWaiting.Remove(Lane);
        if(Zone) RecordIncidentDetour(Lane,*Zone);
        return true;
    }
    if(!HasIncidentRoadClosure()) return false;
    auto& Vehicle=*Vehicles[Lane];
    Jobs[Lane].Route.Reset();
    Jobs[Lane].Waypoint=0;
    IncidentRouteWaiting.Add(Lane);
    IncidentNextRouteAttempt.Add(Lane,IncidentClock+5.);
    Vehicle.bIncidentRoadHold=true;
    Vehicle.SetFMSHold(true);
    ReleaseFutureRoadClaims(Vehicle.VehicleID);
    if(Zone)
    {
        Jobs[Lane].IncidentRoadCause=GetNameSafe(Zone->Cause.Get());
        if(const auto* Cause=Cast<APortAGVActor>(Zone->Cause.Get()))
            RoadBlockers.Add(Vehicle.VehicleID,Cause->VehicleID);
        RecordRecoveryEvent(TEXT("road_waiting"),Zone->Cause.Get(),Zone->Cause.Get(),&Vehicle,
            Jobs[Lane].Actor.Get(),Zone->Center,Vehicle.GetActorLocation());
    }
    return true;
}

FPortIncidentRoadClosure* APortSiteLogistics::LoadedDepartureIncident(int32 Lane)
{
    const auto& Job=Jobs[Lane];
    if(!bReuseFreedApron||Job.Stage!=3||!Yard.IsValidIndex(Job.Slot)||!HasIncidentRoadClosure()) return nullptr;
    const float DestinationY=YardHandover(Yard[Job.Slot]).Y;
    const float DepartureY=AGVApronRoads::DepartureY(TransferQuay(Job).Y,DestinationY);
    const float Spine=AGVApronRoads::LoadedSpine(DepartureY,DestinationY);
    return FindIncidentRoadClosure(AGVApronRoads::P(Spine,DepartureY),AGVApronRoads::P(Spine,DestinationY));
}

void APortSiteLogistics::RecordIncidentDetour(int32 Lane,const FPortIncidentRoadClosure& Zone)
{
    auto& Job=Jobs[Lane];
    auto* Vehicle=Vehicles[Lane].Get();
    Vehicle->bIncidentRoadHold=false;
    RoadBlockers.Remove(Vehicle->VehicleID);
    Job.IncidentRoadCause=GetNameSafe(Zone.Cause.Get());
    Job.IncidentRerouteStage=Job.Stage;
    RecordRecoveryEvent(TEXT("road_rerouted"),Zone.Cause.Get(),Zone.Cause.Get(),Vehicle,
        Job.Actor.Get(),Zone.Center,Vehicle->GetActorLocation());
}

bool APortSiteLogistics::HoldForIncidentRoad(int32 Lane,FVector From,FVector Target,float Dt)
{
    auto& Vehicle=*Vehicles[Lane];
    const FVector Position=Vehicle.GetActorLocation();
    auto* Zone=FindIncidentRoadClosure(From,Target,true);
    Vehicle.bIncidentRoadHold=Zone!=nullptr;
    if(!Zone) return false;

    const auto* Cause=Cast<APortAGVActor>(Zone->Cause.Get());
    auto& Job=Jobs[Lane];
    const FString CauseName=GetNameSafe(Cause);
    if(Job.IncidentRoadCause!=CauseName)
        RecordRecoveryEvent(TEXT("road_waiting"),Zone->Cause.Get(),Zone->Cause.Get(),&Vehicle,Job.Actor.Get(),Zone->Center,Position);
    Job.IncidentRoadCause=CauseName;
    ReleaseFutureRoadClaims(Vehicle.VehicleID);
    Vehicle.SetFMSHold(true);
    Jobs[Lane].TrafficWaitSeconds+=Dt;
    Jobs[Lane].EquipmentDowntimeSeconds+=Dt;
    if(Cause&&Cause!=&Vehicle) RoadBlockers.Add(Vehicle.VehicleID,Cause->VehicleID);
    if(!RoadWaitSince.Contains(Vehicle.VehicleID)) RoadWaitSince.Add(Vehicle.VehicleID,TrafficClock);
    return true;
}

void APortSiteLogistics::RerouteAroundIncident(int32 Lane)
{
    auto& Job=Jobs[Lane];
    auto* Vehicle=Vehicles[Lane].Get();
    if(Job.IncidentRerouteStage!=Job.Stage&& !Vehicle->bIncidentRoadHold)
    {
        Job.IncidentRoadCause.Reset();
        Job.IncidentRerouteStage=INDEX_NONE;
    }
    if(!HasIncidentRoadClosure()) Job.bWaitForIncidentAtBay=false;
    if(!HasIncidentRoadClosure()||Vehicle->Speed>.1f||!Job.Route.IsValidIndex(Job.Waypoint)||
        TrafficRecoveryTargets.Contains(Vehicle->VehicleID)||IncidentClock<IncidentNextRouteAttempt.FindRef(Lane)) return;
    Job.bWaitForIncidentAtBay=false;
    IncidentNextRouteAttempt.Add(Lane,IncidentClock+2.);

    const int32 End=Job.Stage==3&&!Job.bRMGReserved?
        FMath::Clamp(Job.YardEntryWaypoint+1,0,Job.Route.Num()):Job.Route.Num();
    FVector Previous=Vehicle->GetActorLocation();
    FPortIncidentRoadClosure* Zone=nullptr;
    double OriginalDistance=0;
    for(int32 I=Job.Waypoint;I<End;++I)
    {
        if(!Zone) Zone=FindIncidentRoadClosure(Previous,Job.Route[I]);
        OriginalDistance+=FVector::Dist2D(Previous,Job.Route[I]);
        Previous=Job.Route[I];
    }
    // The next AGV in a queue can be blocked by a stopped vehicle whose own
    // route intersects the incident, even when its own route is clear.
    if(!Zone&&(Vehicle->Sensors.bFMSHold||Vehicle->Sensors.bControlledStop))
    {
        int32 Blocker=RoadBlockers.FindRef(Vehicle->VehicleID);
        TSet<int32> Seen={Vehicle->VehicleID};
        while(Blocker>=100&&!Seen.Contains(Blocker))
        {
            Seen.Add(Blocker);
            const int32 Index=Vehicles.IndexOfByPredicate([Blocker](const auto& Candidate)
            {return Candidate->VehicleID==Blocker;});
            if(!Jobs.IsValidIndex(Index)) break;
            const auto& Other=*Vehicles[Index];
            Zone=IncidentRoadClosures.Find(DriveZoneKey(Other));
            if(!Zone&&Other.bIncidentRoadHold&&!Jobs[Index].IncidentRoadCause.IsEmpty())
                for(auto& Entry:IncidentRoadClosures)
                    if(Entry.Value.Active&&Entry.Value.Cause.IsValid()&&
                        Entry.Value.Cause->GetName()==Jobs[Index].IncidentRoadCause)
                    {Zone=&Entry.Value;break;}
            if(Zone||(!Other.Sensors.bFMSHold&&!Other.Sensors.bControlledStop)) break;
            Blocker=RoadBlockers.FindRef(Blocker);
        }
    }
    if(!Zone) return;
    IncidentNextRouteAttempt.Add(Lane,IncidentClock+5.);
    const FVector At=Vehicle->GetActorLocation();
    const bool AtSafeBay=At.Equals(FleetPark(Lane),AGVReference::DockingAccuracyCm+1.f)&&!FindIncidentRoadClosure(At,At);
    const bool CompareWait=AtSafeBay&&Zone->ExpectedClearAt>IncidentClock;
    const double MaxCost=CompareWait?
        OriginalDistance+(Zone->ExpectedClearAt-IncidentClock)*AGVDispatchPolicy::CruiseSpeedCmPerSecond:MAX_dbl;
    if(PlanRoadRoute(Lane,true,false,MaxCost))
        RecordIncidentDetour(Lane,*Zone);
    else if(CompareWait)
    {
        Job.bWaitForIncidentAtBay=true;
        Vehicle->bIncidentRoadHold=true;
        Job.IncidentRoadCause=GetNameSafe(Zone->Cause.Get());
        Vehicle->SetFMSHold(true);
    }
}

void APortSiteLogistics::ReassignIncidentPickup(int32 Lane)
{
    if(LaneCount!=9) return;
    auto* Failed=Vehicles[Lane].Get();
    auto& Job=Jobs[Lane];
    const int32 STS=Job.STS;
    if(!Failed->bDriveFault||Failed->Speed>.1f||Failed->PayloadKg>0||Job.ExportIndex!=INDEX_NONE||
        !STSOwners.IsValidIndex(STS)||Job.bRecoveryRoutePending) return;

    auto MoveFailedToReturnJob=[&]
    {
        const bool Parked=Failed->GetActorLocation().Equals(FleetPark(Lane),AGVReference::DockingAccuracyCm+1.f);
        Job=FSiteTransfer();
        Failed->ActiveDeckOffsetCm=0;
        IncidentRouteWaiting.Remove(Lane);
        IncidentNextRouteAttempt.Remove(Lane);
        if(Parked) return;
        Job.STS=STS;
        Job.Stage=FSiteTransfer::ReturnToStandby;
        IncidentRouteWaiting.Add(Lane);
    };
    if((Job.Stage==7||Job.Stage==8)&&NextVehicles[STS]==Lane)
    {
        const int32 Cargo=PreparedCargo[STS];
        NextVehicles[STS]=INDEX_NONE;
        RecordRecoveryEvent(TEXT("standby_released"),Failed,Failed,Failed,Manifest.IsValidIndex(Cargo)?Manifest[Cargo].Actor.Get():nullptr);
        MoveFailedToReturnJob();
        return;
    }
    if(Job.Stage!=6||STSOwners[STS]!=Lane||!CraneAvailable(YardCraneCount+STS)) return;
    auto* Crane=Equipment[YardCraneCount+STS].Get();
    if(Crane->GetHandoverVehicle()!=Failed||!Job.Actor.IsValid()) return;

    TArray<TPair<double,int32>> Candidates;
    const FVector Goal=TransferQuay(Job);
    if(IncidentRoadBlocked(Goal,Goal+FVector(0,.1,0))) return;
    for(int32 I=0;I<Jobs.Num();++I)
    {
        if(I==Lane||Jobs[I].Stage!=0||TrafficClock<Jobs[I].DispatchRetryAfter||
            !Vehicles[I]->CanAcceptWork()||!ParkingDepartureAvailable(I)) continue;
        Candidates.Emplace(AGVDispatchPolicy::Score(Vehicles[I]->GetActorLocation(),Goal,STS,Vehicles[I]->CompletedJobs,0,I),I);
    }
    Candidates.StableSort([](const auto& A,const auto& B){return A.Key<B.Key;});
    for(const auto& Candidate:Candidates)
    {
        const int32 Replacement=Candidate.Value;
        auto* Vehicle=Vehicles[Replacement].Get();
        const float PreviousOffset=Vehicle->ActiveDeckOffsetCm;
        const auto PreviousJob=Jobs[Replacement];
        Jobs[Replacement]=Job;
        auto& NewJob=Jobs[Replacement];
        NewJob.bEquipmentWaiting=NewJob.bYardAdmissionSuspended;
        NewJob.IncidentRoadCause=Failed->GetName();
        NewJob.IncidentRerouteStage=NewJob.Stage;
        Vehicle->ActiveDeckOffsetCm=Job.CargoDeckOffsetCm;
        if(!PlanRoadRoute(Replacement,false,true))
        {
            Jobs[Replacement]=PreviousJob;
            Vehicle->ActiveDeckOffsetCm=PreviousOffset;
            continue;
        }
        // Commit only after the replacement has a feasible approach; the
        // cargo, physical pad and yard booking stay with the same transfer.
        STSOwners[STS]=Replacement;
        if(Yard.IsValidIndex(Job.Slot))
        {
            auto& ApproachOwner=YardApproachOwners[YardApproachKey(Yard[Job.Slot])];
            if(ApproachOwner==Lane) ApproachOwner=Replacement;
        }
        for(auto& Lease:DeferredYardLeases) if(Lease.Vehicle==Lane) Lease.Vehicle=Replacement;
        Crane->SetHandoverVehicle(Vehicle);
        RecordRecoveryEvent(TEXT("incident_pickup_reassigned"),Failed,Failed,Vehicle,Job.Actor.Get(),Failed->GetActorLocation(),Goal);
        if(Manifest.IsValidIndex(Job.ExtraCargo))
            TrackIncidentImpact(Failed,Failed,Vehicle,Manifest[Job.ExtraCargo].Actor.Get());
        MoveFailedToReturnJob();
        return;
    }
}
