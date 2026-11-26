#include "TerminalReplay.h"
#include "PortSiteLogistics.h"
#include "PortAGVActor.h"
#include "PortContainerActor.h"
#include "PortWorkingCrane.h"
#include "AGVDispatchPolicy.h"
#include "AGVReference.h"
#include "TerminalLayout.h"

bool APortSiteLogistics::ParkingDepartureAvailable(int32 Vehicle) const
{
    if(!bDepotParking || LaneCount!=9) return true;
    const auto* V=Vehicles[Vehicle].Get();const FVector From=V->GetActorLocation();
    if(!From.Equals(FleetPark(Vehicle),AGVReference::DockingAccuracyCm+1.f)) return true;
    const FVector Exit(From.X,TerminalLayout::AGVStandbyExitY(Vehicle),0),Extent(380,730,250);
    FBox Gate(From-Extent,From+Extent);Gate+=Exit-Extent;Gate+=Exit+Extent;
    TArray<int32> Candidates;
    if(bTrafficIndexReady) Candidates=ReservationIndex.Query({Gate}).Array();
    else RoadReservations.GetKeys(Candidates);
    for(int32 ID:Candidates) if(ID!=V->VehicleID)
        if(const auto* Boxes=RoadReservations.Find(ID))
            for(const auto& Box:*Boxes) if(Gate.Intersect(Box)) return false;
    return true;
}

bool APortSiteLogistics::CanReleaseParkedStandby(int32 Vehicle) const
{
    const auto& J=Jobs[Vehicle];const auto* V=Vehicles[Vehicle].Get();
    return bDepotParking && LaneCount==9 && J.Stage==7 && NextVehicles.IsValidIndex(J.STS) &&
        NextVehicles[J.STS]==Vehicle && !V->Emergency.Active && !V->bDriveFault && V->PayloadKg==0 &&
        V->Speed<=.1f && !TrafficRecoveryTargets.Contains(V->VehicleID) &&
        V->GetActorLocation().Equals(FleetPark(Vehicle),AGVReference::DockingAccuracyCm+1.f);
}

void APortSiteLogistics::ReleaseUnenteredParkingMission(int32 Vehicle)
{
    auto* V=Vehicles[Vehicle].Get();const int32 ID=V->VehicleID;
    Jobs[Vehicle]=FSiteTransfer();Jobs[Vehicle].DispatchRetryAfter=TrafficClock+60;
    V->ActiveDeckOffsetCm=0;V->SetFMSHold(false);
    V->SetFMSCommand(V->GetActorLocation(),0,0,true);
    RoadReservations.Remove(ID);RoadReservationEndWaypoint.Remove(ID);ReservationIndex.Remove(ID);
    RoadTargets.Remove(ID);RoadBlockers.Remove(ID);RoadWaitSince.Remove(ID);FinishedRoadSegments.Remove(ID);
    ClearPendingRoadRequest(ID);
}

void APortSiteLogistics::ReassignBlockedParkingVehicles()
{
    if(!bDepotParking || LaneCount!=9) return;
    for(int32 Old=0;Old<Jobs.Num();++Old)
    {
        auto* V=Vehicles[Old].Get();auto& J=Jobs[Old];
        if((J.Stage!=6 && J.Stage!=7) || J.ExportIndex!=INDEX_NONE || V->PayloadKg>0 || V->Emergency.Active ||
            V->bDriveFault || V->Speed>.1f || TrafficRecoveryTargets.Contains(V->VehicleID) ||
            !V->GetActorLocation().Equals(FleetPark(Old),AGVReference::DockingAccuracyCm+1.f)) continue;
        const double* Since=RoadWaitSince.Find(V->VehicleID);
        if(!Since || TrafficClock-*Since<45. || !V->Sensors.bFMSHold || TrafficClock<J.DispatchRetryAfter) continue;
        const int32 S=J.STS;
        const bool Standby=J.Stage==7;
        const bool WasActive=AGVThroughputController.Config.bEnabled&&AGVThroughputController.IsFleetMember(Old);
        if(Standby && !CanReleaseParkedStandby(Old)) continue;
        if(!Standby && (!Equipment.IsValidIndex(YardCraneCount+S) || !CraneAvailable(YardCraneCount+S) ||
            !STSOwners.IsValidIndex(S) || STSOwners[S]!=Old ||
            Equipment[YardCraneCount+S]->GetHandoverVehicle()!=V)) continue;

        // Ordinary queues can clear without a swap. Keep the current mission
        // and its wait age until a replacement has a genuinely usable exit.
        J.DispatchRetryAfter=TrafficClock+10.;
        if(bTrafficIndexReady && J.Route.IsValidIndex(J.Waypoint))
        {
            const FVector Target=J.Route[J.Waypoint];
            const auto Corridor=BuildRoadCorridor(*V,Target,&J.Route,J.Waypoint,J.Route.Num(),false);
            if(FindRoadCorridorBlocker(*V,Target,Corridor,&J.Route,J.Waypoint)==INDEX_NONE) continue;
        }
        const FVector Goal=Standby?QueueQuay(S):TransferQuay(J);
        TArray<TPair<double,int32>> Candidates;
        for(int32 I=0;I<Jobs.Num();++I) if(I!=Old && (WasActive||IsAGVDispatchEnabled(I)) && Vehicles[I]->CanAcceptWork())
        {
            const bool Warm=!Standby && NextVehicles.IsValidIndex(S) && NextVehicles[S]==I &&
                Jobs[I].Stage==8 && Vehicles[I]->Speed<=.1f;
            if(!Warm && (Jobs[I].Stage!=0 || TrafficClock<Jobs[I].DispatchRetryAfter || !ParkingDepartureAvailable(I))) continue;
            const FVector Position=Vehicles[I]->GetActorLocation();
            const double Cost=Warm?FVector::Dist2D(Position,Goal)/AGVDispatchPolicy::CruiseSpeedCmPerSecond:
                AGVDispatchPolicy::Score(Position,Goal,S,Vehicles[I]->CompletedJobs,0,I);
            Candidates.Emplace(Cost,I);
        }
        Candidates.StableSort([](const auto& A,const auto& B) { return A.Key<B.Key; });
        int32 Best=INDEX_NONE;
        for(const auto& Candidate:Candidates)
        {
            const int32 I=Candidate.Value;
            auto Saved=MoveTemp(Jobs[I]);
            const float SavedOffset=Vehicles[I]->ActiveDeckOffsetCm;
            Jobs[I]=J;
            Vehicles[I]->ActiveDeckOffsetCm=J.CargoDeckOffsetCm;
            if(PlanRoadRoute(I,false,true))
            {
                Best=I;
                break;
            }
            Jobs[I]=MoveTemp(Saved);
            Vehicles[I]->ActiveDeckOffsetCm=SavedOffset;
        }
        if(Best==INDEX_NONE) continue;

        if(WasActive)
        {
            AGVThroughputController.ActiveFleet[Old]=false;
            AGVThroughputController.ActiveFleet[Best]=true;
        }
        if(Standby) NextVehicles[S]=Best;
        else
        {
            if(NextVehicles[S]==Best)NextVehicles[S]=INDEX_NONE;
            STSOwners[S]=Best;
            if(Yard.IsValidIndex(J.Slot))
            {
                int32& Approach=YardApproachOwners[YardApproachKey(Yard[J.Slot])];
                if(Approach==Old)Approach=Best;
            }
            for(auto& Lease:DeferredYardLeases)if(Lease.Vehicle==Old)Lease.Vehicle=Best;
            Equipment[YardCraneCount+S]->SetHandoverVehicle(Vehicles[Best]);
            RecordRecoveryEvent(TEXT("agv_reassigned"),V,V,Vehicles[Best],J.Actor.Get());
            PORTSIM_SIM_LOG(LogTemp,Display,TEXT("AGV_BLOCKED_PICKUP_REASSIGNED: STS%d V%d -> V%d; old vehicle remains parked"),S+1,V->VehicleID,Vehicles[Best]->VehicleID);
        }
        // The old vehicle never left its verified bay, so clearing an unentered
        // request changes neither its physical pose nor any attached cargo.
        ReleaseUnenteredParkingMission(Old);
        // Let the selected vehicle request its corridor before checking
        // another replacement against the same road in the next step.
        return;
    }
}
