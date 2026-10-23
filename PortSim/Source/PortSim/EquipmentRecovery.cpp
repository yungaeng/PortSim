#include "PortSiteLogistics.h"
#include "PortWorkingCrane.h"
#include "PortContainerActor.h"
#include "PortAGVActor.h"
#include "YardStackingPolicy.h"
#include "AGVReference.h"
#include "AGVTrafficGeometry.h"
#include "TerminalLayout.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"

namespace { constexpr double RailClearanceCm=2500.; } // blockout assumption, not an OEM limit

bool APortSiteLogistics::CraneAvailable(int32 I) const
{ return Equipment.IsValidIndex(I) && IsValid(Equipment[I]) && Equipment[I]->bEnabled && Equipment[I]->Fault.IsEmpty() && !Equipment[I]->bFaultEvacuating && !Equipment[I]->bFaultEvacuationComplete; }

void APortSiteLogistics::RebuildSTSStreams()
{
    ManifestBySTS.SetNum(LaneCount);
    for(auto& Stream:ManifestBySTS) Stream.Reset();
    for(int32 I=0;I<Manifest.Num();++I) ManifestBySTS[Manifest[I].STS].Add(I);
}

bool APortSiteLogistics::CanSTSGantryMove(const APortWorkingCrane* Crane,float Y) const
{
    if(LaneCount!=9) return true;
    if(!bRecoveryRailGuard) return true;
    const int32 I=Equipment.IndexOfByKey(Crane);
    if(I<YardCraneCount) return true;
    for(int32 J=YardCraneCount;J<Equipment.Num();++J) if(J!=I)
    {
        const double OtherY=Equipment[J]->GantryWorldY();
        if((J<I && Y<OtherY+RailClearanceCm) || (J>I && Y>OtherY-RailClearanceCm)) return false;
    }
    return true;
}

bool APortSiteLogistics::CanSTSService(int32 STS,int32 Cargo,const FVector* DestinationOverride) const
{
    if(!CraneAvailable(YardCraneCount+STS) || !Manifest.IsValidIndex(Cargo)) return false;
    const auto& C=Manifest[Cargo];
    const auto* Crane=Equipment[YardCraneCount+STS].Get();
    // Unstarted work lands at the candidate crane's pad. Using C.STS here
    // incorrectly requires the replacement to travel through the failed crane.
    const FVector Destination=DestinationOverride?*DestinationOverride:
        (bCargoAlignedHandover?FVector(QuayPark(STS).X,C.Transform.GetLocation().Y,0):QuayPark(STS))+FVector(0,0,349.5f);
    if(!Crane->CanReach(C.Actor->GetActorLocation(),Destination)) return false;
    if(!bRecoveryRailGuard && STS==C.OriginalSTS) return true;
    const double Low=FMath::Min3(double(Crane->GantryWorldY()),C.Transform.GetLocation().Y,Destination.Y);
    const double High=FMath::Max3(double(Crane->GantryWorldY()),C.Transform.GetLocation().Y,Destination.Y);
    for(int32 S=0;S<LaneCount;++S) if(S!=STS)
    {
        const auto Interval=Equipment[YardCraneCount+S]->RailWorkInterval();
        if(Low<Interval.Y+RailClearanceCm && High>Interval.X-RailClearanceCm) return false;
    }
    return true;
}

bool APortSiteLogistics::JobHeldByFailure(int32 V) const
{
    const auto& J=Jobs[V];
    if(J.bRecoveryRoutePending || (J.bYardAdmissionSuspended && (J.Stage==1 || J.Stage==10))) return true;
    const auto* STS=Equipment.IsValidIndex(YardCraneCount+J.STS)?Equipment[YardCraneCount+J.STS].Get():nullptr;
    // Keep unrelated loaded/returning trips moving. An approaching vehicle
    // may still reach its reserved pad; only the stationary pickup waits.
    if(STS && STS->IsEquipmentUnavailable() && (J.Stage==1 || J.Stage==10)) return true;
    return J.Stage==4 && Equipment.IsValidIndex(J.RMG) && Equipment[J.RMG]->IsEquipmentUnavailable();
}

void APortSiteLogistics::ReleaseDeferredYardLeases()
{
    for(int32 I=DeferredYardLeases.Num()-1;I>=0;--I)
    {
        const auto Lease=DeferredYardLeases[I];
        const FVector P=Vehicles[Lease.Vehicle]->GetActorLocation(),Dock=YardHandover(Yard[Lease.Slot]);
        // Keep the old turning area until the vehicle has physically cleared it.
        if(FVector::Dist2D(P,Dock)<4000.) continue;
        const float RoadX=TerminalLayout::SiteCmX(Yard[Lease.Slot].Half?TerminalLayout::AGVInlandReturnX:12500.f);
        if(AGVTrafficGeometry::SweptChassisIntersects(Dock,FVector(RoadX,Dock.Y,0),P,
            Vehicles[Lease.Vehicle]->GetActorRightVector(),730,210,170)) continue;
        const int32 Key=YardApproachKey(Yard[Lease.Slot]);
        if(YardApproachOwners[Key]==Lease.Vehicle) YardApproachOwners[Key]=INDEX_NONE;
        DeferredYardLeases.RemoveAt(I);
    }
}

void APortSiteLogistics::ManageEquipmentFailures()
{
    if(LaneCount!=9) return;
    if(!bFailureInjected && SimulationTime>=ScheduledFailureAt && (ScheduledFailedSTS>0 || ScheduledFailedRMG>0))
    {
        for(int32 I:{ScheduledFailedSTS>0?YardCraneCount+ScheduledFailedSTS-1:INDEX_NONE,
            ScheduledFailedRMG>0?ScheduledFailedRMG-1:INDEX_NONE})
            if(Equipment.IsValidIndex(I)) { Equipment[I]->bAutoSensorRecovery=FParse::Param(FCommandLine::Get(),TEXT("PortSimAutomaticSensorRepair")); Equipment[I]->SetSensorFailure(true); }
        bFailureInjected=true;
    }
    if(bFailureInjected && !bFailureRepaired && ScheduledRepairAfter>=0 && SimulationTime>=ScheduledFailureAt+ScheduledRepairAfter)
    {
        for(int32 I:{ScheduledFailedSTS>0?YardCraneCount+ScheduledFailedSTS-1:INDEX_NONE,
            ScheduledFailedRMG>0?ScheduledFailedRMG-1:INDEX_NONE})
            if(Equipment.IsValidIndex(I)) Equipment[I]->RepairSensors();
        bFailureRepaired=true;
    }
    bool Changed=false;
    for(int32 I=0;I<Equipment.Num();++I)
    {
        auto* Crane=Equipment[I].Get();
        if(Crane->IsEquipmentUnavailable())
        {
            Crane->RecoveryStatus=Crane->bGantryFault?TEXT("gantry_blocked_repair_required"):
                Crane->bFaultEvacuating?TEXT("evacuating_with_independent_safety"):
                Crane->bFaultEvacuationComplete?TEXT("relocated_wait_repair"):Crane->RetainsLoad()?TEXT("load_retained_repair_required"):
                Crane->IsBusy()?TEXT("verification_repair_required"):TEXT("isolated_reassigning");
            if(!IsolatedEquipment.Contains(I))
            {
                IsolatedEquipment.Add(I); ++EquipmentFaultEvents;
                RecordRecoveryEvent(TEXT("fault"),Crane,Crane,Crane);
                UE_LOG(LogTemp,Display,TEXT("EQUIPMENT_SENSOR_ISOLATED: crane=%d retained=%d terminal_continues=1 gantry_fault=%d"),Crane->CraneID,Crane->RetainsLoad(),Crane->bGantryFault);
                if(!Crane->RetainsLoad() && Crane->IsBusy())
                {
                    // Abort only an unlatched, supported pickup. Released loads
                    // awaiting settling and retained loads stay with their job.
                    const bool Cancelled=Crane->CancelUnliftedJob();
                    if(Cancelled && I<YardCraneCount) for(int32 V=0;V<Jobs.Num();++V) if(Jobs[V].RMG==I && Jobs[V].Stage==4)
                    { auto& J=Jobs[V]; J.Stage=9; J.bRMGReserved=false; J.bEquipmentWaiting=true; J.RecoveryAnchor=Vehicles[V]->GetActorLocation(); RMGBusy[I]=false; }
                }
                const int32 S=I-YardCraneCount;
                if(S>=0 && PreparedCargo.IsValidIndex(S) && Manifest.IsValidIndex(PreparedCargo[S]) &&
                    !PreparedStarted[S] && NextVehicles[S]==INDEX_NONE)
                {
                    Manifest[PreparedCargo[S]].State=0; --Dispatched;
                    PreparedCargo[S]=INDEX_NONE;
                }
                NextRecoveryAttempt=0;
            }
        }
        else if(IsolatedEquipment.Remove(I)) { RecordRecoveryEvent(TEXT("repair"),Crane,Crane,Crane); Crane->RecoveryStatus=TEXT("repaired"); Changed=true; NextRecoveryAttempt=0; }
    }
    ReleaseDeferredYardLeases();
    bRecoveryRailGuard=!IsolatedEquipment.IsEmpty();
    for(const auto& Crane:Equipment) if(Crane->bFaultEvacuating || Crane->bFaultEvacuationComplete) bRecoveryRailGuard=true;
    if(!bRecoveryRailGuard) for(const auto& C:Manifest) if(C.State==1 && C.STS!=C.OriginalSTS) { bRecoveryRailGuard=true; break; }
    PlanFaultEvacuations();
    for(int32 V=0;V<Jobs.Num();++V)
    {
        auto& J=Jobs[V];
        J.bEquipmentWaiting=(Equipment.IsValidIndex(YardCraneCount+J.STS) &&
            Equipment[YardCraneCount+J.STS]->IsEquipmentUnavailable() && (J.Stage==6 || J.Stage==1 || J.Stage==10)) ||
            (Equipment.IsValidIndex(J.RMG) && Equipment[J.RMG]->IsEquipmentUnavailable() && J.Stage>=2 && J.Stage<=9);
        if(Equipment.IsValidIndex(YardCraneCount+J.STS) && Equipment[YardCraneCount+J.STS]->bRailSafetyHold) J.bEquipmentWaiting=true;
        // A failed quay pickup occupies its own pad, not a distant yard aisle.
        // Keep the cargo slot booking, but lend the unused approach to other trips.
        if((J.Stage==6 || J.Stage==1 || J.Stage==10) && Yard.IsValidIndex(J.Slot) &&
            Equipment[YardCraneCount+J.STS]->IsEquipmentUnavailable() && !J.bRMGReserved &&
            FVector::Dist2D(Vehicles[V]->GetActorLocation(),YardHandover(Yard[J.Slot]))>4000.)
        {
            int32& ApproachOwner=YardApproachOwners[YardApproachKey(Yard[J.Slot])];
            if(ApproachOwner==V)
            {
                ApproachOwner=INDEX_NONE; J.bYardAdmissionSuspended=true;
                UE_LOG(LogTemp,Display,TEXT("FAILURE_YARD_APPROACH_RELEASED: STS%d AGV%d RMG%d slot_booking_kept=1"),J.STS+1,Vehicles[V]->VehicleID,Yard[J.Slot].Crane+1);
            }
        }
        if(J.bYardAdmissionSuspended)
        {
            auto* Crane=Equipment[YardCraneCount+J.STS].Get();
            if(CraneAvailable(YardCraneCount+J.STS) && ReserveYardApproach(V))
            {
                J.bYardAdmissionSuspended=false; J.bEquipmentWaiting=false;
                Crane->SetDestinationReady(J.Stage!=6); Vehicles[V]->SetFMSHold(false);
            }
            else { J.bEquipmentWaiting=true; Crane->SetDestinationReady(false); }
        }
        if(J.bRecoveryRoutePending && SimulationTime>=NextRecoveryAttempt)
        {
            J.Stage=3;
            if(PlanRoadRoute(V,false)) { J.bRecoveryRoutePending=false; J.bEquipmentWaiting=false; Vehicles[V]->SetFMSHold(false); }
            else { J.Stage=9; J.bEquipmentWaiting=true; }
        }
    }
    if(SimulationTime<NextRecoveryAttempt) return;
    NextRecoveryAttempt=SimulationTime+1.;
    // Preserve an assigned AGV and its cargo-specific handover point. Only
    // the STS job owner changes; the container is never moved by this planner.
    for(int32 V=0;V<Jobs.Num();++V)
    {
        auto& J=Jobs[V];
        if(J.Stage!=6 && J.Stage!=1 && J.Stage!=10) continue;
        auto* Old=Equipment[YardCraneCount+J.STS].Get();
        const int32 Cargo=J.Stage==10?J.ExtraCargo:J.Cargo;
        if(!Manifest.IsValidIndex(Cargo) || Manifest[Cargo].Actor->LocationOwner!=ECargoOwner::Ship || Old->RetainsLoad()) continue;
        if(Old->IsBusy() && CraneAvailable(YardCraneCount+J.STS)) continue;
        int32 Best=INDEX_NONE; double Cost=TNumericLimits<double>::Max();
        const int32 Origin=Manifest[Cargo].OriginalSTS;
        // An approaching AGV is not at the handover yet. Preserve the failed
        // crane's assigned pad, including any measured landing adjustment.
        const FVector Destination=Old->AssignedDestination();
        for(int32 S=Origin/3*3;S<Origin/3*3+3;++S)
        {
            if(FMath::Abs(S-Origin)>1 || !CanSTSService(S,Cargo,&Destination) || Equipment[YardCraneCount+S]->IsBusy() ||
                (STSOwners[S]!=INDEX_NONE && STSOwners[S]!=V) || NextVehicles[S]!=INDEX_NONE) continue;
            if(J.Stage!=10 && Manifest.IsValidIndex(J.ExtraCargo) && !CanSTSService(S,J.ExtraCargo,&Destination)) continue;
            const double D=FMath::Abs(Equipment[YardCraneCount+S]->GantryWorldY()-Manifest[Cargo].Transform.GetLocation().Y);
            if(D<Cost) { Best=S; Cost=D; }
        }
        if(Best==INDEX_NONE) { Manifest[Cargo].RecoveryHold=TEXT("sts_no_reachable_alternative"); continue; }
        auto* Replacement=Equipment[YardCraneCount+Best].Get();
        if(!Replacement->CanReach(Manifest[Cargo].Actor->GetActorLocation(),Destination)) continue;
        if(!Replacement->AssignCargo(Manifest[Cargo].Actor.Get(),Manifest[Cargo].Actor->GetActorLocation(),Destination,false,false,Vehicles[V]))
        { Stop(TEXT("Validated STS recovery assignment rejected")); return; }
        Replacement->SetDestinationReady(J.Stage!=6 && !J.bYardAdmissionSuspended);
        STSOwners[J.STS]=INDEX_NONE; STSOwners[Best]=V;
        const int32 Previous=J.STS; J.STS=Best;
        for(int32 AssignedCargo:{Cargo,J.Stage!=10?J.ExtraCargo:INDEX_NONE}) if(Manifest.IsValidIndex(AssignedCargo))
        {
            auto& C=Manifest[AssignedCargo];
            if(C.STS!=Best) {
                RecordRecoveryEvent(TEXT("sts_reassigned"),Equipment[YardCraneCount+Origin],Equipment[YardCraneCount+C.STS],Replacement,C.Actor.Get(),
                    Equipment[YardCraneCount+C.STS]->GetActorLocation(),Replacement->GetActorLocation());
                ++C.Reassignments; ++STSReassignments; Changed=true;
            }
            C.STS=Best; C.RecoveryHold.Empty();
        }
        J.bEquipmentWaiting=J.bYardAdmissionSuspended; Vehicles[V]->SetFMSHold(false);
        UE_LOG(LogTemp,Display,TEXT("STS_FAILURE_REASSIGNED: cargo=%d old=%d new=%d agv=%d"),Manifest[Cargo].ID,Previous+1,Best+1,Vehicles[V]->VehicleID);
    }
    for(int32 I=0;I<Manifest.Num();++I)
    {
        auto& C=Manifest[I];
        if(C.State!=0) continue;
        if(CraneAvailable(YardCraneCount+C.OriginalSTS))
        {
            if(C.STS!=C.OriginalSTS) { C.STS=C.OriginalSTS; Changed=true; }
            C.RecoveryHold.Empty(); continue;
        }
        int32 Best=INDEX_NONE; double Cost=TNumericLimits<double>::Max();
        for(int32 S=C.OriginalSTS/3*3;S<C.OriginalSTS/3*3+3;++S)
        {
            if(FMath::Abs(S-C.OriginalSTS)>1 || !CanSTSService(S,I)) continue;
            const double D=FMath::Abs(Equipment[YardCraneCount+S]->GantryWorldY()-C.Transform.GetLocation().Y)+Equipment[YardCraneCount+S]->IsBusy()*5000.;
            if(D<Cost) { Cost=D; Best=S; }
        }
        if(Best!=INDEX_NONE)
        {
            if(C.STS!=Best) {
                RecordRecoveryEvent(TEXT("sts_reassigned"),Equipment[YardCraneCount+C.OriginalSTS],Equipment[YardCraneCount+C.STS],Equipment[YardCraneCount+Best],C.Actor.Get(),
                    Equipment[YardCraneCount+C.STS]->GetActorLocation(),Equipment[YardCraneCount+Best]->GetActorLocation());
                C.STS=Best; ++C.Reassignments; ++STSReassignments; Changed=true;
            }
            C.RecoveryHold.Empty();
        }
        else C.RecoveryHold=TEXT("sts_no_reachable_alternative");
    }
    if(Changed) RebuildSTSStreams();
    for(int32 I=0;I<YardCraneCount;++I) if(!CraneAvailable(I)) ReplanFailedYards(I);
    ReplanBlockedYards();
}

void APortSiteLogistics::PlanFaultEvacuations()
{
    for(int32 S=0;S<LaneCount;++S)
    {
        auto* Crane=Equipment[YardCraneCount+S].Get();
        if(Crane->Fault.IsEmpty() && Crane->bFaultEvacuationComplete)
        {
            // Returning to its original rail anchor happens before admission.
            // Neighbour reservations may delay it; the crane never crosses them.
            Crane->RecoveryStatus=TEXT("returning_to_service_position");
            bool HomeClear=true;
            const double Home=Crane->GantryHomeWorldY();
            for(int32 Other=0;Other<LaneCount;++Other) if(Other!=S)
            {
                const auto Interval=Equipment[YardCraneCount+Other]->RailWorkInterval();
                if((Other<S && Home<Interval.Y+RailClearanceCm) || (Other>S && Home>Interval.X-RailClearanceCm)) HomeClear=false;
            }
            if(HomeClear && !Crane->bFaultEvacuating) Crane->RequestFaultEvacuation(Home,this);
            continue;
        }
        if(!Crane->CanFaultEvacuate() || Crane->bFaultEvacuating || Crane->bFaultEvacuationComplete) continue;
        // The failed crane remains between its original neighbours. Moving
        // beyond an adjacent crane would itself violate non-crossing.
        const double Low=S>0?Equipment[YardCraneCount+S-1]->RailWorkInterval().Y+RailClearanceCm:-50000.;
        const double High=S+1<LaneCount?Equipment[YardCraneCount+S+1]->RailWorkInterval().X-RailClearanceCm:23000.;
        const double Current=Crane->GantryWorldY();
        if(Low>Current || High<Current || High-Low<200) continue;
        auto Blocked=[&](double Y)
        {
            int32 Count=0;
            for(const auto& C:Manifest) if(C.State!=2 && C.OriginalSTS/3==S/3 &&
                FMath::Abs(C.Transform.GetLocation().Y-Y)<RailClearanceCm) ++Count;
            return Count;
        };
        double Best=Current; int32 BestBlocked=Blocked(Current);
        for(double Candidate:{Low+50.,High-50.})
            if(Blocked(Candidate)<BestBlocked && CanSTSGantryMove(Crane,Candidate))
            { Best=Candidate; BestBlocked=Blocked(Candidate); }
        if(FMath::Abs(Best-Current)>100 && Crane->RequestFaultEvacuation(Best,this))
        {
            RecordRecoveryEvent(TEXT("evacuation_started"),Crane,Crane,Crane,nullptr,FVector(0,Current,0),FVector(0,Best,0));
            UE_LOG(LogTemp,Display,TEXT("STS_FAULT_EVACUATION: crane=%d from_y=%.2f to_y=%.2f remaining_blocked=%d"),Crane->CraneID,Current,Best,BestBlocked);
        }
    }
}

bool APortSiteLogistics::ReplanFailedYards(int32 FailedRMG)
{
    TSet<int32> Affected;
    for(int32 I=0;I<Manifest.Num();++I)
    {
        const auto& C=Manifest[I];
        if(C.State==2 || !Yard.IsValidIndex(C.PlannedSlot) || CraneAvailable(Yard[C.PlannedSlot].Crane)) continue;
        if(FailedRMG!=INDEX_NONE && Yard[C.PlannedSlot].Crane!=FailedRMG) continue;
        const auto* Failed=Equipment[Yard[C.PlannedSlot].Crane].Get();
        if(Failed->IsBusy() && Failed->CargoActor==C.Actor.Get()) continue;
        Affected.Add(I);
    }
    return ReplanYardGroup(Affected,{});
}

bool APortSiteLogistics::ReplanBlockedYards()
{
    TSet<int32> Unavailable,Active,Affected,BlockedSlots;
    AActor* Cause=nullptr;
    for(const auto& J:Jobs) if(J.Stage && J.Stage<FSiteTransfer::ToCharge)
    { Active.Add(J.Cargo); if(J.ExtraCargo!=INDEX_NONE) Active.Add(J.ExtraCargo); }
    for(int32 I=0;I<Manifest.Num();++I) if(Manifest[I].State!=2 && !CraneAvailable(YardCraneCount+Manifest[I].STS))
    { Unavailable.Add(I); if(!Cause) Cause=Equipment[YardCraneCount+Manifest[I].STS]; }
    if(Unavailable.IsEmpty()) return true;
    // A ship box under a stranded box is also unavailable until that box moves.
    bool Changed=true;
    while(Changed)
    {
        Changed=false;
        for(int32 I=0;I<Manifest.Num();++I) if(Manifest[I].Actor->LocationOwner==ECargoOwner::Ship && !Unavailable.Contains(I))
            for(int32 Above:SourceAbove[I]) if(Unavailable.Contains(Above) && Manifest[Above].Actor->LocationOwner==ECargoOwner::Ship)
            { Unavailable.Add(I); Changed=true; break; }
    }
    for(int32 I:Unavailable) BlockedSlots.Add(Manifest[I].PlannedSlot);
    TArray<int32> Order;
    for(int32 I=0;I<Manifest.Num();++I) Order.Add(I);
    Order.Sort([&](int32 A,int32 B){return Manifest[A].PlanningOrder<Manifest[B].PlanningOrder;});
    for(int32 I:Order)
    {
        const auto& C=Manifest[I];
        if(C.State==2 || !Yard.IsValidIndex(C.PlannedSlot)) continue;
        const auto& Slot=Yard[C.PlannedSlot];
        const bool Blocked=BlockedSlots.Contains(Slot.Below) ||
            (Slot.LengthFt==20 && Slot.Below!=INDEX_NONE && Yard.IsValidIndex(Slot.Partner) && BlockedSlots.Contains(Yard[Slot.Partner].Below));
        if(!Blocked) continue;
        BlockedSlots.Add(C.PlannedSlot);
        // Include every unstarted descendant, even one whose source is also
        // unavailable. Leaving its old destination fixed would orphan that
        // booking when its healthy lower support moves to a different stack.
        if(!Active.Contains(I) && C.Actor->LocationOwner==ECargoOwner::Ship) Affected.Add(I);
    }
    return ReplanYardGroup(Affected,Unavailable,Cause);
}

bool APortSiteLogistics::ReplanYardGroup(const TSet<int32>& Affected,const TSet<int32>& UnavailableSupports,AActor* Cause)
{
    if(Affected.IsEmpty()) return true;
    TMap<int32,int32> Active;
    for(int32 V=0;V<Jobs.Num();++V) if(Jobs[V].Stage && Jobs[V].Stage<FSiteTransfer::ToCharge)
    {
        if(Jobs[V].Actor.IsValid()) Active.Add(Jobs[V].Cargo,V);
        if(Jobs[V].ExtraCargo!=INDEX_NONE) Active.Add(Jobs[V].ExtraCargo,V);
    }
    for(int32 I:Affected) if(const int32* V=Active.Find(I))
    {
        const auto& J=Jobs[*V];
        if(J.Stage==3 || J.Stage==4 || J.bRecoveryRoutePending) return false;
        if(J.Stage==9 && (Vehicles[*V]->Speed>.1f || !J.Route.IsValidIndex(J.Waypoint-1))) return false;
    }
    TArray<int32> Plan,Occupant,PerRMG; TArray<double> Above;
    Plan.Reserve(Manifest.Num()); Occupant.Init(INDEX_NONE,Yard.Num()); Above.Init(0,Yard.Num()); PerRMG.Init(0,YardCraneCount);
    for(int32 I=0;I<Manifest.Num();++I)
    {
        Plan.Add(Manifest[I].PlannedSlot);
        if(!Affected.Contains(I)) { Occupant[Plan[I]]=I; ++PerRMG[Yard[Plan[I]].Crane]; }
    }
    for(int32 I=0;I<Manifest.Num();++I) if(!Affected.Contains(I))
        for(int32 B=Yard[Plan[I]].Below;B!=INDEX_NONE;B=Yard[B].Below) Above[B]+=Manifest[I].Actor->MassKg;
    TArray<int32> Order=Affected.Array(); Order.Sort([&](int32 A,int32 B){return Manifest[A].PlanningOrder<Manifest[B].PlanningOrder;});
    TMap<int32,int32> AssignedActiveCrane,NewApproachOwner;
    for(int32 I:Order)
    {
        const auto* C=Manifest[I].Actor.Get(); const auto& Info=C->CargoInformation;
        const int32* Vehicle=Active.Find(I); int32 Best=INDEX_NONE; double BestCost=TNumericLimits<double>::Max();
        for(int32 S=0;S<Yard.Num();++S)
        {
            const auto& Slot=Yard[S];
            if(!Slot.Reserved || Slot.Central || Slot.Occupied || Occupant[S]!=INDEX_NONE || Slot.LengthFt!=C->LengthFt ||
                !CraneAvailable(Slot.Crane) || !YardStackingPolicy::ZoneAccepts(Slot.Block,Info)) continue;
            if(!Equipment[Slot.Crane]->CanReach(YardHandover(Slot)+FVector(0,0,349.5f),Slot.Position)) continue;
            if(Info.HazardGroup==TEXT("none") && Info.HandlingType==TEXT("dry") &&
                (Slot.Block<5?1:Slot.Block<10?2:Slot.Block<14?3:4)!=Info.OutboundOrder) continue;
            if(Vehicle)
            {
                if(const int32* Required=AssignedActiveCrane.Find(*Vehicle); Required && *Required!=Slot.Crane) continue;
                const int32 ApproachOwner=YardApproachOwners[YardApproachKey(Slot)];
                if(ApproachOwner!=INDEX_NONE && ApproachOwner!=*Vehicle) continue;
                if(const int32* OwnerV=NewApproachOwner.Find(Slot.Crane); OwnerV && *OwnerV!=*Vehicle) continue;
                if(RMGBusy[Slot.Crane] || Equipment[Slot.Crane]->IsBusy()) continue;
                bool TurnClear=true;
                for(int32 V=0;V<Jobs.Num();++V) if(V!=*Vehicle && Yard.IsValidIndex(Jobs[V].Slot) && !Jobs[V].bYardReleased &&
                    YardApproachOwners[YardApproachKey(Yard[Jobs[V].Slot])]!=INDEX_NONE &&
                    FVector::Dist2D(YardHandover(Slot),YardHandover(Yard[Jobs[V].Slot]))<3000.) { TurnClear=false; break; }
                for(const auto& OwnerEntry:NewApproachOwner) if(OwnerEntry.Value!=*Vehicle)
                    for(int32 Other:Order) if(const int32* OwnerV=Active.Find(Other); OwnerV && *OwnerV==OwnerEntry.Value && Occupant[Plan[Other]]==Other &&
                        FVector::Dist2D(YardHandover(Slot),YardHandover(Yard[Plan[Other]]))<3000.) TurnClear=false;
                if(!TurnClear) continue;
            }
            if(Slot.Below!=INDEX_NONE)
            {
                const int32 Lower=Occupant[Slot.Below];
                if(Lower==INDEX_NONE || UnavailableSupports.Contains(Lower) || Manifest[Lower].PlanningOrder>=Manifest[I].PlanningOrder ||
                    !YardStackingPolicy::MayStack(Manifest[Lower].Actor->CargoInformation,Manifest[Lower].Actor->LengthFt,
                        Manifest[Lower].Actor->MassKg,Info,C->LengthFt,C->MassKg)) continue;
                if(Vehicle && !Yard[Slot.Below].Occupied) continue;
                if(Slot.LengthFt==20)
                {
                    const int32 OtherBelow=Yard.IsValidIndex(Slot.Partner)?Yard[Slot.Partner].Below:INDEX_NONE;
                    if(OtherBelow==INDEX_NONE || Occupant[OtherBelow]==INDEX_NONE || UnavailableSupports.Contains(Occupant[OtherBelow]) ||
                        Manifest[Occupant[OtherBelow]].PlanningOrder>=Manifest[I].PlanningOrder ||
                        (Vehicle && !Yard[OtherBelow].Occupied)) continue;
                }
            }
            bool WeightOK=true;
            for(int32 B=Slot.Below;B!=INDEX_NONE;B=Yard[B].Below)
                if(Occupant[B]==INDEX_NONE || UnavailableSupports.Contains(Occupant[B]) || Above[B]+C->MassKg>Manifest[Occupant[B]].Actor->CargoInformation.PermittedSuperimposedKg) { WeightOK=false; break; }
            if(!WeightOK) continue;
            const FVector Start=Vehicle?Vehicles[*Vehicle]->GetActorLocation():CargoQuay(I);
            const double Cost=FVector::Dist2D(Start,YardHandover(Slot))/600.+PerRMG[Slot.Crane]*240.+(Slot.Below==INDEX_NONE?600.:0.);
            if(Cost<BestCost) { BestCost=Cost; Best=S; }
        }
        if(Best==INDEX_NONE)
        {
            for(int32 Cargo:Affected) Manifest[Cargo].RecoveryHold=TEXT("no_compatible_healthy_yard");
            return false; // no manifest, job, or reservation has changed
        }
        Plan[I]=Best; Occupant[Best]=I; ++PerRMG[Yard[Best].Crane];
        for(int32 B=Yard[Best].Below;B!=INDEX_NONE;B=Yard[B].Below) Above[B]+=C->MassKg;
        if(Vehicle) { AssignedActiveCrane.Add(*Vehicle,Yard[Best].Crane); NewApproachOwner.Add(Yard[Best].Crane,*Vehicle); }
    }
    // Commit only a complete feasible plan. Fixed placed/retained loads and
    // unaffected bookings never move; all missing lower supports move together.
    for(int32 I:Order)
    {
        auto& C=Manifest[I];
        const auto& Old=Yard[C.PlannedSlot]; const auto& New=Yard[Plan[I]];
        RecordRecoveryEvent(TEXT("yard_reassigned"),Cause?Cause:Equipment[Old.Crane].Get(),Equipment[Old.Crane],Equipment[New.Crane],C.Actor.Get(),Old.Position,New.Position);
        C.PlannedSlot=Plan[I]; ++C.Reassignments; ++YardReassignments; C.RecoveryHold.Empty();
        if(!Active.Contains(I))
        {
            if(Manifest.IsValidIndex(C.PlannedPair) && !Active.Contains(C.PlannedPair)) Manifest[C.PlannedPair].PlannedPair=INDEX_NONE;
            C.PlannedPair=INDEX_NONE; C.PlannedPairSavingSeconds=C.EstimatedPairSavingSeconds=0;
        }
    }
    for(const auto& Entry:AssignedActiveCrane)
    {
        const int32 V=Entry.Key; auto& J=Jobs[V]; const int32 Old=J.Slot,New=Plan[J.Cargo];
        if(Yard.IsValidIndex(Old) && Old!=New)
        {
            DeferredYardLeases.Add({V,Old}); SlotAssigned[Old]=false;
            RMGBusy[Yard[Old].Crane]=false;
        }
        J.Slot=New; J.RMG=Yard[New].Crane; J.bRMGReserved=false; J.bYardReleased=false;
        SlotAssigned[New]=true; YardApproachOwners[YardApproachKey(Yard[New])]=V;
        J.bEquipmentWaiting=false;
        if(J.Stage==9)
        {
            J.RecoveryOriginSlot=Old; J.RecoveryAnchor=Vehicles[V]->GetActorLocation(); J.bRecoveryRoutePending=true;
        }
    }
    DispatchPriorityLimit=INDEX_NONE;
    UE_LOG(LogTemp,Display,TEXT("YARD_FAILURE_REPLAN: containers=%d active_agvs=%d atomic=1 cause=%s"),Order.Num(),AssignedActiveCrane.Num(),Cause?*Cause->GetName():TEXT("rmg_failure"));
    return true;
}
