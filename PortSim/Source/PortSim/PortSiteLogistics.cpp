#include "PortSiteLogistics.h"
#include "TerminalLayout.h"
#include "YardReservation.h"
#include "PortWorkingCrane.h"
#include "PortAGVActor.h"
#include "PortContainerActor.h"
#include "ContainerSpecification.h"
#include "AGVDispatchPolicy.h"
#include "TrafficJunctionPolicy.h"
#include "AGVRoadNetwork.h"
#include "Components/HierarchicalInstancedStaticMeshComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/World.h"
#include "Misc/Paths.h"
#include "Misc/FileHelper.h"
#include "HAL/FileManager.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"

namespace SiteLogistics
{
    constexpr float LegacyCraneY[]={-450,-350,-250,-100,100,250,350,450};
    FTransform Hidden(FTransform T) { T.SetScale3D(FVector::ZeroVector); return T; }
    FTransform YardTransform(FVector P) { return FTransform(FQuat::Identity,P,FVector(12.192,2.438,2.59)); }
}

APortSiteLogistics::APortSiteLogistics()
{
    PrimaryActorTick.bCanEverTick=false;
    bCargoAlignedHandover=!FParse::Param(FCommandLine::Get(),TEXT("PortSimFixedHandover"));
}

void APortSiteLogistics::AddShipCargo(FVector Position,int32 STS)
{
    check(STS>=0 && STS<9);
    FSiteShipCargo Record;
    Record.Transform=FTransform(FQuat::Identity,Position);
    Record.STS=STS; Record.ID=2000+Manifest.Num();
    FActorSpawnParameters Params; Params.Owner=this;
    Params.SpawnCollisionHandlingOverride=ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
    auto* Cargo=GetWorld()->SpawnActor<APortContainerActor>(Position,FRotator::ZeroRotator,Params);
    check(Cargo);
    Cargo->InitializeContainer(Record.ID);
    Cargo->ConfigureSpecification(PortContainerSpecification::ForSequence(Record.ID),
        STSProfile.bReady?STSProfile.ContainerCoG:FVector::ZeroVector);
    // Preserve the existing injected-overload test path without replacing normal
    // per-container manifest masses with the profile's nominal fallback mass.
    if(STSProfile.bReady && STSProfile.ContainerMassKg>STSProfile.RatedPayloadKg)
        Cargo->SetPhysicalParameters(STSProfile.ContainerMassKg,STSProfile.ContainerCoG);
    // Secured aboard until pickup: real collidable actors, without idle rigid-body simulation.
    Cargo->GetBody()->SetSimulatePhysics(false);
    Cargo->LocationOwner=ECargoOwner::Ship;
    Record.Actor=Cargo;
    ShipContainers.Add(Cargo);
    Manifest.Add(Record);
}
FVector APortSiteLogistics::QuayPark(int32 Lane) const
{ return FVector(TerminalLayout::SiteCmX(4500),((LaneCount==9?TerminalLayout::STSCenterY(Lane):SiteLogistics::LegacyCraneY[Lane])+8)*100,0); }
FVector APortSiteLogistics::CargoQuay(int32 CargoIndex) const
{
    // Fix the destination for the whole job using the secured manifest position,
    // never the swinging cargo or the crane's instantaneous position.
    const auto& Cargo=Manifest[CargoIndex];
    return bCargoAlignedHandover?FVector(QuayPark(Cargo.STS).X,Cargo.Transform.GetLocation().Y,0):QuayPark(Cargo.STS);
}
FVector APortSiteLogistics::QueueQuay(int32 STS) const
{
    const FVector Next=CargoQuay(PreparedCargo[STS]);
    const int32 CurrentVehicle=STSOwners[STS];
    const double CurrentY=Jobs.IsValidIndex(CurrentVehicle)?CargoQuay(Jobs[CurrentVehicle].Cargo).Y:Next.Y;
    // Queue beyond the new handover, away from the AGV still being loaded.
    // This also handles the bay order wrapping back to the start of a row.
    return Next+FVector(0,Next.Y<CurrentY?-2000:2000,0);
}
FVector APortSiteLogistics::YardHandover(const FSiteYardSlot& Slot) const
{ return Slot.Handover; }
FVector APortSiteLogistics::FleetPark(int32 Vehicle) const
{
    if(LaneCount!=9) return QuayPark(Vehicle);
    // Two 30-vehicle rows stay entirely west of the worker-rest apron (Y=238 m).
    // 23 m pitch clears the physical 13.8 m vehicle length. Spawn, return and
    // reset all share these positions, so parked vehicles cannot spill east.
    const float Y=TerminalLayout::AGVParkFirstY+(Vehicle%30)*TerminalLayout::AGVParkPitch;
    return FVector(TerminalLayout::SiteCmX(8000+(Vehicle/30)*2000),Y,0);
}

void APortSiteLogistics::Initialize(const TArray<TObjectPtr<APortWorkingCrane>>& Cranes,TArray<FSiteYardSlot> Slots,
    const TArray<UHierarchicalInstancedStaticMeshComponent*>& Palette,int32 CentralCargo,int32 FixedYard)
{
    Equipment=Cranes; Yard=MoveTemp(Slots); CentralCount=CentralCargo; YardCraneCount=Cranes.Num()-(CentralCount==0?9:8); LaneCount=Equipment.Num()-YardCraneCount;
    BaselineYard=Yard.Num()+FixedYard;
    check((LaneCount==8 || LaneCount==9) && Yard.Num()>InitialShipCount());
    // Remove TOP tiers only; filling the reservation bottom-up restores supported stacks.
    Yard.StableSort([](const FSiteYardSlot& A,const FSiteYardSlot& B) { return A.Position.Z==B.Position.Z ? A.Position.X<B.Position.X : A.Position.Z>B.Position.Z; });
    for (int32 I=0;I<Yard.Num();++I)
    {
        auto& Slot=Yard[I]; Slot.Reserved=CentralCount>0 && I<InitialShipCount();
    }
    if (CentralCount==0)
    {
        TMap<FIntPoint,int32> StackByPosition;
        TArray<TArray<int32>> Stacks;
        TArray<int32> StackCranes;
        for (int32 I=Yard.Num()-1;I>=0;--I)
        {
            const auto& Slot=Yard[I];
            const FIntPoint Key(FMath::RoundToInt(Slot.Position.X),FMath::RoundToInt(Slot.Position.Y));
            int32* Existing=StackByPosition.Find(Key);
            const int32 Stack=Existing?*Existing:Stacks.AddDefaulted();
            if (!Existing) { StackByPosition.Add(Key,Stack); StackCranes.Add(Slot.Crane); }
            Stacks[Stack].Add(I);
        }
        // Exactly one receiving slot per ship container, balanced over the available RMGs.
        // Whole-stack selection preserves supported inventory before and after reset.
        for (int32 Crane=0;Crane<YardCraneCount;++Crane)
        {
            const int32 Quota=InitialShipCount()/YardCraneCount+(Crane<InitialShipCount()%YardCraneCount?1:0);
            TArray<int32> ZoneStacks,Sizes,Selected;
            for (int32 I=0;I<Stacks.Num();++I) if (StackCranes[I]==Crane)
            { ZoneStacks.Add(I); Sizes.Add(Stacks[I].Num()); }
            if (!SelectWholeYardStacks(Sizes,Quota,Selected))
            { Stop(FString::Printf(TEXT("Cannot reserve exactly %d supported slots for RMG%d"),Quota,Crane+1)); return; }
            for (int32 LocalStack:Selected) for (int32 Slot:Stacks[ZoneStacks[LocalStack]]) Yard[Slot].Reserved=true;
        }
    }
    ReceivingCapacity=0;
    for (auto& Slot:Yard)
    {
        ReceivingCapacity+=Slot.Reserved;
        Slot.Occupied=!Slot.Reserved;
        Slot.Mesh=Palette[Slot.Color];
        const FTransform T=SiteLogistics::YardTransform(Slot.Position);
        Slot.Instance=Slot.Mesh->AddInstance(Slot.Reserved?SiteLogistics::Hidden(T):T);
    }
    Yard.StableSort([](const FSiteYardSlot& A,const FSiteYardSlot& B) { return A.Position.Z<B.Position.Z; });
    InitialYard=BaselineYard-ReceivingCapacity;
    check(ReceivingCapacity>=InitialShipCount());
    TMap<FIntVector,int32> Positions;
    for (int32 I=0;I<Yard.Num();++I)
    {
        const FVector P=Yard[I].Position;
        const FIntVector Key(FMath::RoundToInt(P.X),FMath::RoundToInt(P.Y),FMath::RoundToInt((P.Z-149.5f)/259));
        if (const int32* Below=Positions.Find(Key-FIntVector(0,0,1))) Yard[I].Below=*Below;
        Positions.Add(Key,I);
        check(Key.Z==0 || Yard[I].Below!=INDEX_NONE);
    }
    CentralSlots.Reset();
    for (int32 Cargo=0;Cargo<CentralCount;++Cargo)
    {
        const int32 Slot=Yard.IndexOfByPredicate([Cargo](const FSiteYardSlot& S)
        { return S.Block==Cargo%TerminalLayout::YardBlockCount && S.Reserved && !S.Central; });
        check(Slot!=INDEX_NONE);
        Yard[Slot].Central=true;
        CentralSlots.Add(Slot);
    }
    // Upper ship tiers leave first, so no boxes are lifted through an upper stack.
    Manifest.StableSort([](const FSiteShipCargo& A,const FSiteShipCargo& B) { return A.Transform.GetLocation().Z>B.Transform.GetLocation().Z; });
    if(!PlanYardDestinations()) { Stop(TEXT("Could not create supported pre-discharge yard plan")); return; }
    Jobs.SetNum(LaneCount==9?60:LaneCount); YardApproachOwners.Init(INDEX_NONE,TerminalLayout::YardBlockCount); BlocksBusy.Init(false,TerminalLayout::YardBlockCount); RMGBusy.Init(false,YardCraneCount);
    STSOwners.Init(INDEX_NONE,LaneCount);
    NextVehicles.Init(INDEX_NONE,LaneCount); PreparedStarted.Init(false,LaneCount);
    PreparedCargo.Init(INDEX_NONE,LaneCount); SlotAssigned.Init(false,Yard.Num());
    FActorSpawnParameters Params; Params.Owner=this;
    Params.SpawnCollisionHandlingOverride=ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
    for (int32 I=0;I<Jobs.Num();++I)
    {
        auto* Vehicle=GetWorld()->SpawnActor<APortAGVActor>(FleetPark(I),FRotator::ZeroRotator,Params);
        Vehicle->InitializeVehicle(100+I); Vehicles.Add(Vehicle);
    }
    TrafficVehicles=Vehicles;
    bReady=true;
    BeginReport();
    if(!STSProfile.bReady) Stop(TEXT("Site STS reference unavailable: ")+STSProfile.Error);
#if WITH_EDITOR
    SetActorLabel(TEXT("DGT_STS_AGV_RMG_Dispatch"));
    SetFolderPath(TEXT("PortSim/Logistics"));
#endif
    UE_LOG(LogTemp,Display,TEXT("SITE_INVENTORY: yard_before=%d, vessel_total=%d, yard_initial=%d, reserved=%d, site_ship=%d, central_ship=%d"),
        BaselineYard,InitialShipCount(),InitialYard,ReceivingCapacity,Manifest.Num(),CentralCount);
}

void APortSiteLogistics::PrepareNextCargo(int32 Lane)
{
    if (PreparedCargo[Lane]==INDEX_NONE)
    {
        if (Dispatched>=DispatchLimit) return;
        const int32 Next=Manifest.IndexOfByPredicate([Lane](const FSiteShipCargo& C) { return C.STS==Lane && C.State==0; });
        if (Next==INDEX_NONE) return;
        PreparedCargo[Lane]=Next; PreparedStarted[Lane]=false;
        Manifest[Next].State=1; ++Dispatched;
        Manifest[Next].StartedAt=SimulationTime; Manifest[Next].PreparedPausedSeconds=0;
        if(LaneCount==9 && VesselStarted[Lane/3]<0) VesselStarted[Lane/3]=SimulationTime;
        if (STSOwners[Lane]!=INDEX_NONE) ++PrefetchedJobs;
    }
    if (PreparedStarted[Lane] || Equipment[YardCraneCount+Lane]->IsBusy()) return;
    if (STSOwners[Lane]!=INDEX_NONE && Jobs[STSOwners[Lane]].Stage<2) return;
    if (STSOwners[Lane]!=INDEX_NONE && Jobs[STSOwners[Lane]].Stage==6) return;
    const int32 Index=PreparedCargo[Lane];
    auto& Record=Manifest[Index];
    if (!Equipment[YardCraneCount+Lane]->AssignCargo(Record.Actor.Get(),Record.Transform.GetLocation(),CargoQuay(Index)+FVector(0,0,349.5f),false,false))
    { Stop(TEXT("STS could not prepare its next ship container: ")+Equipment[YardCraneCount+Lane]->Fault); return; }
    Equipment[YardCraneCount+Lane]->SetDestinationReady(false);
    PreparedStarted[Lane]=true;
}

void APortSiteLogistics::ActivateVehicle(int32 Vehicle,int32 STS,bool FromQueue)
{
    auto& Job=Jobs[Vehicle];
    Job.Cargo=PreparedCargo[STS]; Job.Actor=Manifest[Job.Cargo].Actor; Job.STS=STS;
    Job.Stage=6; Job.Waypoint=0;
    Job.StartedAt=Manifest[Job.Cargo].StartedAt; Job.Time=SimulationTime-Job.StartedAt;
    Job.PausedSeconds=Manifest[Job.Cargo].PreparedPausedSeconds;
    Equipment[YardCraneCount+STS]->SetHandoverVehicle(Vehicles[Vehicle]);
    PreparedCargo[STS]=INDEX_NONE; PreparedStarted[STS]=false; STSOwners[STS]=Vehicle;
    if (FromQueue) { NextVehicles[STS]=INDEX_NONE; ++QueuedHandoffs; }
    {
    if(!PlanRoadRoute(Vehicle,false)) Stop(TEXT("No road route to STS handover"));
}

void APortSiteLogistics::ScheduleFleet()
{
    auto Nearest=[&](FVector Target)
    {
        int32 MinimumJobs=MAX_int32;
        for(int32 I=0;I<Vehicles.Num();++I) if(Jobs[I].Stage==0)
            MinimumJobs=FMath::Min(MinimumJobs,Vehicles[I]->CompletedJobs);
        int32 Best=INDEX_NONE; double BestScore=TNumericLimits<double>::Max();
        for (int32 I=0;I<Vehicles.Num();++I) if (Jobs[I].Stage==0)
        {
            // Pickup ETA is the main cost. A bounded workload penalty breaks close
            // choices toward the less-used vehicle without sending a distant AGV.
            const double Score=AGVDispatchPolicy::Score(Vehicles[I]->GetActorLocation(),Target,
                Vehicles[I]->CompletedJobs,MinimumJobs);
            if (Score<BestScore || (FMath::IsNearlyEqual(Score,BestScore) && Vehicles[I]->VehicleID<Vehicles[Best]->VehicleID))
            { Best=I; BestScore=Score; }
        }
        return Best;
    };
    // Serve all open docks before allocating their next vehicles.
    for (int32 S=0;S<LaneCount;++S)
    {
        PrepareNextCargo(S);
        if(!Fault.IsEmpty()) return;
        if (STSOwners[S]!=INDEX_NONE || PreparedCargo[S]==INDEX_NONE || !PreparedStarted[S]) continue;
        if (NextVehicles[S]!=INDEX_NONE)
        {
            if (Jobs[NextVehicles[S]].Stage==8) ActivateVehicle(NextVehicles[S],S,true);
        }
        else if (const int32 V=Nearest(CargoQuay(PreparedCargo[S])); V!=INDEX_NONE) ActivateVehicle(V,S,false);
    }
    for (int32 S=0;S<LaneCount;++S)
    {
        PrepareNextCargo(S);
        if(!Fault.IsEmpty()) return;
        if (STSOwners[S]==INDEX_NONE || NextVehicles[S]!=INDEX_NONE || PreparedCargo[S]==INDEX_NONE) continue;
        const int32 V=Nearest(CargoQuay(PreparedCargo[S]));
        if (V==INDEX_NONE) continue;
        NextVehicles[S]=V;
        // Reserve the next vehicle while it waits in its parking bay. Moving
        // it along the quay before its dock opens can interlock adjacent STSs.
        auto& Job=Jobs[V]; Job.STS=S; Job.Stage=8;
        Job.Route.Reset(); Job.Waypoint=0;
    }
}

void APortSiteLogistics::Dispatch(int32 Lane)
{
    // Each STS owns its handover; only intersecting road reservations block entry.
    int32 STS=INDEX_NONE;
    float Best=TNumericLimits<float>::Max();
    for (int32 S=0;S<LaneCount;++S)
    {
        if (LaneCount==8 && S!=Lane) continue;
        if (STSOwners[S]!=INDEX_NONE) continue;
        PrepareNextCargo(S);
        if(!Fault.IsEmpty()) return;
        if (PreparedCargo[S]==INDEX_NONE || !PreparedStarted[S]) continue;
        const float Distance=FVector::DistSquared2D(Vehicles[Lane]->GetActorLocation(),CargoQuay(PreparedCargo[S]));
        if (Distance<Best) { Best=Distance; STS=S; }
    }
    if (STS==INDEX_NONE) return;
    const int32 Index=PreparedCargo[STS];
    auto& Job=Jobs[Lane];
    Job.Cargo=Index; Job.Actor=Manifest[Index].Actor; Job.STS=STS; Job.Stage=6;
    Job.StartedAt=Manifest[Index].StartedAt; Job.Time=SimulationTime-Job.StartedAt;
    Job.PausedSeconds=Manifest[Index].PreparedPausedSeconds;
    Equipment[YardCraneCount+STS]->SetHandoverVehicle(Vehicles[Lane]);
    PreparedCargo[STS]=INDEX_NONE; PreparedStarted[STS]=false; STSOwners[STS]=Lane;
    const FVector Quay=CargoQuay(Job.Cargo);
    if (LaneCount==9 && !Vehicles[Lane]->GetActorLocation().Equals(Quay,1.f))
        Job.Route={FVector(TerminalLayout::SiteCmX(7200),Vehicles[Lane]->GetActorLocation().Y,0),FVector(TerminalLayout::SiteCmX(7200),Quay.Y,0),Quay};
    else Job.Route={Quay};
}

bool APortSiteLogistics::ReserveYardApproach(int32 Lane)
{
    auto& Job=Jobs[Lane];
    const int32 Planned=Manifest[Job.Cargo].PlannedSlot;
    if(!Yard.IsValidIndex(Planned)) { Stop(TEXT("Manifest has no valid planned yard slot")); return false; }
    const auto& Slot=Yard[Planned];
    // Keep the entry lane bounded, and never queue an upper box ahead of its support.
    if(!Slot.Reserved || Slot.Central || Slot.Occupied || SlotAssigned[Planned] ||
        YardApproachOwners[Slot.Block]!=INDEX_NONE || BlocksBusy[Slot.Block] ||
        (Slot.Below!=INDEX_NONE && !Yard[Slot.Below].Occupied) ||
        (CentralPending!=INDEX_NONE && Yard[CentralSlots[CentralPending]].Block==Slot.Block)) return false;
    Job.Slot=Planned; Job.RMG=Slot.Crane;
    SlotAssigned[Planned]=true; YardApproachOwners[Slot.Block]=Lane;
    return true;
}

bool APortSiteLogistics::ReserveYard(int32 Lane)
{
    auto& Job=Jobs[Lane];
    const int32 Planned=Manifest[Job.Cargo].PlannedSlot;
    if(!Yard.IsValidIndex(Planned)) { Stop(TEXT("Manifest has no valid planned yard slot")); return false; }
    const auto& Slot=Yard[Planned]; const int32 RMG=Slot.Crane;
    if (!Slot.Reserved || Slot.Central || Slot.Occupied || Job.Slot!=Planned || YardApproachOwners[Slot.Block]!=Lane || BlocksBusy[Slot.Block] ||
        RMGBusy[RMG] || Equipment[RMG]->IsBusy() || !Equipment[RMG]->Fault.IsEmpty() ||
        (CentralPending!=INDEX_NONE && Yard[CentralSlots[CentralPending]].Block==Slot.Block && YardApproachOwners[Slot.Block]!=Lane)) return false;
    if (Slot.Below!=INDEX_NONE && !Yard[Slot.Below].Occupied) return false;
    Job.Slot=Planned; Job.RMG=RMG;
    Job.bRMGReserved=true; RMGBusy[Job.RMG]=true;
    UE_LOG(LogTemp,Display,TEXT("SITE_JOB: C%d AGV%d follows yard plan RMG%d block%d slot%d"),
        Manifest[Job.Cargo].ID,100+Lane,Job.RMG+1,Slot.Block+1,Planned);
    return true;
}

bool APortSiteLogistics::PlanYardDestinations()
{
    TSet<int32> PlannedSlots;
    TArray<int32> PlannedPerRMG; PlannedPerRMG.Init(0,YardCraneCount);
    TArray<TArray<int32>> CargoBySTS; CargoBySTS.SetNum(LaneCount);
    for(int32 I=0;I<Manifest.Num();++I) CargoBySTS[Manifest[I].STS].Add(I);
    TArray<int32> PlanningOrder;
    for(int32 Wave=0;PlanningOrder.Num()<Manifest.Num();++Wave)
        for(int32 STS=0;STS<LaneCount;++STS)
            if(CargoBySTS[STS].IsValidIndex(Wave)) PlanningOrder.Add(CargoBySTS[STS][Wave]);
    // Runtime prepares one next box per STS. Plan in the same wave order so an
    // upper-tier destination never waits on a lower box scheduled for a later STS cycle.
    for(int32 CargoIndex:PlanningOrder)
    {
        float LowestTier=TNumericLimits<float>::Max();
        for(int32 I=0;I<Yard.Num();++I)
        {
            const auto& Slot=Yard[I];
            if(!Slot.Reserved || Slot.Central || PlannedSlots.Contains(I) ||
                (Slot.Below!=INDEX_NONE && !PlannedSlots.Contains(Slot.Below))) continue;
            LowestTier=FMath::Min(LowestTier,float(Slot.Position.Z));
        }
        int32 Best=INDEX_NONE; double BestCost=TNumericLimits<double>::Max();
        for(int32 I=0;I<Yard.Num();++I)
        {
            const auto& Slot=Yard[I];
            if(!Slot.Reserved || Slot.Central || PlannedSlots.Contains(I) ||
                !FMath::IsNearlyEqual(Slot.Position.Z,LowestTier) ||
                (Slot.Below!=INDEX_NONE && !PlannedSlots.Contains(Slot.Below))) continue;
            const double TravelSeconds=(FVector::Dist2D(CargoQuay(CargoIndex),YardHandover(Slot))+
                FVector::Dist2D(Equipment[Slot.Crane]->HeadPosition(),Slot.Position))/AGVDispatchPolicy::CruiseSpeedCmPerSecond;
            // A preplanned box queued behind another RMG cycle cannot use the
            // terminal's parallel capacity. Model that queue before pure distance.
            constexpr double EstimatedRMGServiceSeconds=240.;
            const double Cost=TravelSeconds+PlannedPerRMG[Slot.Crane]*EstimatedRMGServiceSeconds;
            if(Cost<BestCost) { Best=I; BestCost=Cost; }
        }
        if(Best==INDEX_NONE) return false;
        Manifest[CargoIndex].PlannedSlot=Best;
        PlannedSlots.Add(Best); ++PlannedPerRMG[Yard[Best].Crane];
    }
    return true;
}

bool APortSiteLogistics::PlanRoadRoute(int32 Lane,bool Congested)
{
    auto& Job=Jobs[Lane]; auto* Vehicle=Vehicles[Lane].Get();
    if(Job.Stage!=3 && Job.Stage!=5 && Job.Stage!=6) return false;
    FAGVRoadNetwork Network;
    auto P=[](float X,float Y) { return FVector(TerminalLayout::SiteCmX(X),Y,0); };
    const float South=TerminalLayout::AGVSouthReturnY,North=TerminalLayout::AGVNorthCrossY;
    // Two northbound and two southbound lanes fit inside the existing 50 m spine.
    for(float X:{12500.f,15300.f}) Network.Add(P(X,South),P(X,North));
    for(float X:{13700.f,16500.f}) Network.Add(P(X,North),P(X,South));
    Network.Add(P(3500,North),P(3500,South));
    Network.Add(P(5500,South),P(5500,North));
    Network.Add(P(7200,North),P(7200,South));
    Network.Add(P(9000,South),P(9000,North));
    // Cross parking only beyond its ends. Never invent a diagonal through cargo.
    Network.Add(P(3500,TerminalLayout::AGVSouthLoadedY),P(16500,TerminalLayout::AGVSouthLoadedY));
    Network.Add(P(16500,South),P(3500,South));
    Network.Add(P(5500,North),P(16500,North));
    Network.Add(P(9000,North),P(7200,North));
    for(int32 Block=0;Block<TerminalLayout::YardBlockCount;++Block)
    {
        const float Y=TerminalLayout::BlockY(Block)*100;
        Network.Add(P(12500,Y+2050),P(16500,Y+2050));
        Network.Add(P(16500,Y+2500),P(12500,Y+2500));
    }
    const FVector Quay=CargoQuay(Job.Cargo),Park=FleetPark(Lane);
    Network.Add(P(3500,Quay.Y),P(5500,Quay.Y),true);
    Network.Add(P(7200,Quay.Y),Quay);
    Network.Add(Park,P(Lane>=30?9000:7200,Park.Y));
    Network.Add(P(9000,Park.Y),Park);
    FVector Goal=Quay;
    if(Job.Stage==3 || Job.Stage==5)
    {
        const auto& Slot=Yard[Job.Slot]; const FVector Dock=YardHandover(Slot);
        const float Y=TerminalLayout::BlockY(Slot.Block)*100;
        const FVector Entry(Dock.X,Y+2050,0),Exit(Dock.X,Y+2500,0);
        Network.Add(P(12500,Entry.Y),Entry);
        Network.Add(Dock,Exit);
        Network.Add(Exit,P(12500,Exit.Y));
        Goal=Job.Stage==3?Entry:Park;
    }
    const int32 ID=Vehicle->VehicleID;
    auto Cost=[&](FVector A,FVector B)
    {
        if(!Congested) return 0.;
        const FVector Extent=FMath::Max(A.X,B.X)>TerminalLayout::SiteCmX(16500)+1?
            FVector(730,210,250):FVector(210,730,250);
        FBox Box(A-Extent,A+Extent); Box+=B-Extent; Box+=B+Extent;
        double Penalty=0;
        for(int32 OtherID:ReservationIndex.Query({Box}))
        {
            if(OtherID==ID) continue;
            const auto* Reservations=RoadReservations.Find(OtherID);
            if(Reservations) for(const FBox& R:*Reservations) if(Box.Intersect(R)) { Penalty+=3000; break; }
        }
        for(int32 OtherIndex:VehicleIndex.Query({Box}))
        {
            const auto* Other=TrafficVehicles[OtherIndex].Get();
            if(!IsValid(Other) || Other==Vehicle) continue;
            const FVector E=Other->GetActorForwardVector().GetAbs()*210+Other->GetActorRightVector().GetAbs()*730+FVector(0,0,250);
            if(Box.Intersect(FBox(Other->GetActorLocation()-E,Other->GetActorLocation()+E)))
                Penalty+=Other->Speed<1?12000:3000;
        }
        return Penalty;
    };
    TArray<FVector> Candidate;
    if(!Network.Find(Vehicle->GetActorLocation(),Goal,Candidate,Cost)) return false;
    if(Congested)
    {
        auto Total=[&](const TArray<FVector>& Route,int32 Begin,int32 End)
        {
            FVector From=Vehicle->GetActorLocation(); double Sum=0;
            for(int32 I=Begin;I<End;++I) { Sum+=FVector::Dist2D(From,Route[I])+Cost(From,Route[I]); From=Route[I]; }
            return Sum;
        };
        const int32 End=Job.Stage==3?Job.Route.Num()-1:Job.Route.Num();
        if(Total(Candidate,0,Candidate.Num())+500>=Total(Job.Route,Job.Waypoint,End)) return false;
        ++Job.Reroutes;
        UE_LOG(LogTemp,Display,TEXT("AGV_REROUTE: V%d stage=%d count=%d"),ID,Job.Stage,Job.Reroutes);
    }
    if(Job.Stage==3) Candidate.Add(YardHandover(Yard[Job.Slot]));
    Job.Route=MoveTemp(Candidate); Job.Waypoint=0;
    return true;
}

void APortSiteLogistics::PrepareRoute(int32 Lane,bool Return)
{
    if(LaneCount==9)
    {
        Jobs[Lane].Stage=Return?5:3;
        if(!PlanRoadRoute(Lane,false)) Stop(TEXT("No road route between berth and yard"));
        return;
    }
    auto& Job=Jobs[Lane]; const FVector Quay=CargoQuay(Job.Cargo), YardPoint=YardHandover(Yard[Job.Slot]);
    const float BlockY=TerminalLayout::BlockY(Yard[Job.Slot].Block)*100.f;
    Job.Route.Reset(); Job.Waypoint=0;
    if (!Return)
    {
        if (LaneCount==9)
        {
            // Separate inbound aisle (65 m) and outbound aisle (35 m).
            // Cross behind the south end of the fleet parking strip.
            if (Quay.Y>0)
            {
                // Cross before the worker-rest apron, not beyond the right-hand road.
                Job.Route.Add(FVector(TerminalLayout::SiteCmX(5500),Quay.Y,0));
                for (float Y=Quay.Y+3000;Y<TerminalLayout::AGVNorthCrossY-1600;Y+=3000) Job.Route.Add(FVector(TerminalLayout::SiteCmX(5500),Y,0));
                Job.Route.Add(FVector(TerminalLayout::SiteCmX(5500),TerminalLayout::AGVNorthCrossY,0));
                Job.Route.Add(FVector(TerminalLayout::SiteCmX(16500),TerminalLayout::AGVNorthCrossY,0));
                for (float Y=TerminalLayout::AGVNorthCrossY-3000;Y>BlockY+2050+1600;Y-=3000) Job.Route.Add(FVector(TerminalLayout::SiteCmX(16500),Y,0));
                Job.Route.Add(FVector(TerminalLayout::SiteCmX(16500),BlockY+2050,0));
                Job.Route.Add(FVector(YardPoint.X,BlockY+2050,0));
                Job.Route.Add(YardPoint);
                return;
            }
            Job.Route.Add(FVector(TerminalLayout::SiteCmX(3500),Quay.Y,0));
            for (float Y=Quay.Y-3000;Y>TerminalLayout::AGVSouthLoadedY+1600;Y-=3000) Job.Route.Add(FVector(TerminalLayout::SiteCmX(3500),Y,0));
            Job.Route.Add(FVector(TerminalLayout::SiteCmX(3500),TerminalLayout::AGVSouthLoadedY,0));
            Job.Route.Add(FVector(TerminalLayout::SiteCmX(12500),TerminalLayout::AGVSouthLoadedY,0));
            for (float Y=TerminalLayout::AGVSouthLoadedY+3000;Y<BlockY+2050-1600;Y+=3000) Job.Route.Add(FVector(TerminalLayout::SiteCmX(12500),Y,0));
            Job.Route.Add(FVector(TerminalLayout::SiteCmX(12500),BlockY+2050,0));
            Job.Route.Add(FVector(YardPoint.X,BlockY+2050,0));
            Job.Route.Add(YardPoint);
            return;
        }
        const float RoadX=TerminalLayout::SiteCmX(BlockY+2050>=Quay.Y?12500.f:16500.f);
        Job.Route.Add(FVector(RoadX,Quay.Y,0));
        Job.Route.Add(FVector(RoadX,BlockY+2050,0));
        Job.Route.Add(FVector(YardPoint.X,BlockY+2050,0));
        Job.Route.Add(YardPoint);
    }
    else
    {
        if (LaneCount==9)
        {
            const FVector Park=FleetPark(Lane);
            const float RoadX=TerminalLayout::SiteCmX(14500.f);
            Job.Route={FVector(YardPoint.X,BlockY+2500,0),FVector(RoadX,BlockY+2500,0)};
            for (float Y=BlockY+2500-3000;Y>TerminalLayout::AGVSouthReturnY+1600;Y-=3000)
                Job.Route.Add(FVector(RoadX,Y,0));
            Job.Route.Add(FVector(RoadX,TerminalLayout::AGVSouthReturnY,0));
            Job.Route.Add(FVector(TerminalLayout::SiteCmX(9000),TerminalLayout::AGVSouthReturnY,0));
            for (float Y=TerminalLayout::AGVSouthReturnY+3000;Y<Park.Y-1600;Y+=3000)
                Job.Route.Add(FVector(TerminalLayout::SiteCmX(9000),Y,0));
            Job.Route.Add(FVector(TerminalLayout::SiteCmX(9000),Park.Y,0));
            Job.Route.Add(Park);
            return;
        }
        Job.Route.Add(FVector(YardPoint.X,BlockY+2500,0));
        const float RoadX=TerminalLayout::SiteCmX(Quay.Y+2000>=BlockY+2500?12500.f:16500.f);
        Job.Route.Add(FVector(RoadX,BlockY+2500,0));
        Job.Route.Add(FVector(RoadX,Quay.Y+2000,0));
        Job.Route.Add(FVector(Quay.X,Quay.Y+2000,0));
        Job.Route.Add(Quay);
    }
}

void APortSiteLogistics::RegisterBerthVehicles(const TArray<TObjectPtr<APortAGVActor>>& BerthVehicles)
{ for (const auto& Vehicle:BerthVehicles) TrafficVehicles.AddUnique(Vehicle); bTrafficIndexReady=false; }

void APortSiteLogistics::IndexTrafficVehicle(int32 Index)
{
    const auto* Vehicle=TrafficVehicles[Index].Get();
    if (!IsValid(Vehicle)) { VehicleIndex.Remove(Index); return; }
    // Enclose every heading; Drive/legacy fleet can rotate after MoveVehicle returns.
    // Exact current orientation is still checked when a candidate is queried.
    const FVector Extent(FMath::Sqrt(210.*210.+730.*730.));
    const FVector Position=Vehicle->GetActorLocation();
    VehicleIndex.Set(Index,{FBox(Position-Extent,Position+Extent)});
}

void APortSiteLogistics::BeginTrafficFrame()
{
    for (int32 ID:FinishedRoadSegments)
    {
        const int32 I=Vehicles.IndexOfByPredicate([ID](const auto& V) { return V->VehicleID==ID; });
        // Keep the next segment claimed while the vehicle crosses a waypoint.
        // Releasing it for one frame lets another vehicle enter its approach corridor.
        if (Jobs.IsValidIndex(I) && Jobs[I].Route.IsValidIndex(Jobs[I].Waypoint))
        {
            // Release the completed segment behind the vehicle, retaining only its next segment.
            if (auto* Segments=RoadReservations.Find(ID); Segments && Segments->Num()>1)
            { Segments->RemoveAt(0); ReservationIndex.Set(ID,*Segments); }
            continue;
        }
        RoadReservations.Remove(ID); ReservationIndex.Remove(ID); RoadTargets.Remove(ID);
    }
    FinishedRoadSegments.Reset();
    if (!bTrafficIndexReady)
    {
        ReservationIndex.Reset(); VehicleIndex.Reset();
        for (const auto& Reservation:RoadReservations) ReservationIndex.Set(Reservation.Key,Reservation.Value);
    }
    // Refresh once per frame, including teleports/reset and externally moved vehicles.
    // Unchanged cell memberships are retained without rebuilding bucket sets.
    for (int32 I=0;I<TrafficVehicles.Num();++I) IndexTrafficVehicle(I);
    bTrafficIndexReady=true;
}

bool APortSiteLogistics::MoveVehicle(APortAGVActor* Vehicle,FVector Target,float Dt)
{
    if (!bTrafficIndexReady) BeginTrafficFrame();
    if (LaneCount==9)
    {
        const FVector Extent=Vehicle->GetActorForwardVector().GetAbs()*210.f+
            Vehicle->GetActorRightVector().GetAbs()*730.f;
        const FVector P=Vehicle->GetActorLocation();
        const FVector Min=P.ComponentMin(Target)-Extent, Max=P.ComponentMax(Target)+Extent;
        if (!TerminalLayout::AGVEnvelopeInside(Min.X,Max.X,Min.Y,Max.Y))
        {
            Vehicle->Speed=0;
            Stop(FString::Printf(TEXT("AGV route outside yard/road bounds: V%d from=%s target=%s"),
                Vehicle->VehicleID,*P.ToCompactString(),*Target.ToCompactString()));
            return false;
        }
    }
    const int32 ID=Vehicle->VehicleID;
    const int32 FleetIndex=Vehicles.IndexOfByKey(Vehicle);
    bool StraightThrough=false;
    if (Jobs.IsValidIndex(FleetIndex))
    {
        const auto& Job=Jobs[FleetIndex];
        if (Job.Route.IsValidIndex(Job.Waypoint+1))
        {
            const FVector A=(Target-Vehicle->GetActorLocation()).GetSafeNormal();
            const FVector B=(Job.Route[Job.Waypoint+1]-Target).GetSafeNormal();
            StraightThrough=FVector::DotProduct(A,B)>.999f;
        }
    }
    if (!RoadTargets.Contains(ID) || !RoadTargets[ID].Equals(Target,.01f))
    {
        const FVector Along=Vehicle->GetActorForwardVector().GetAbs(), Across=Vehicle->GetActorRightVector().GetAbs();
        const FVector Extent=Along*210.f+Across*730.f+FVector(0,0,250);
        FBox Segment(Vehicle->GetActorLocation()-Extent,Vehicle->GetActorLocation()+Extent);
        Segment+=Target-Extent; Segment+=Target+Extent;
        TArray<FBox> Segments={Segment};
        if (Jobs.IsValidIndex(FleetIndex))
        {
            const auto& Job=Jobs[FleetIndex];
            const int32 RouteEnd=(Job.Stage==3 && !Job.bRMGReserved)?Job.Route.Num()-1:Job.Route.Num();
            TrafficJunctionPolicy::AppendExitCorridor(Segments,Job.Route,Job.Waypoint,RouteEnd,
                [&](int32 NextIndex)
                {
                    return (LaneCount==9)?
                        ((FMath::Max(Job.Route[NextIndex-1].X,Job.Route[NextIndex].X)>TerminalLayout::SiteCmX(16500)+1)?FVector(730,210,250):FVector(210,730,250)):
                        ((Job.Stage==3 && NextIndex>=Job.Route.Num()-3)?FVector(730,210,250):Extent);
                });
        }

        int32 Blocker=INDEX_NONE;
        for (int32 OtherID:ReservationIndex.Query(Segments))
        {
            if (OtherID==ID) continue;
            const auto* Reservation=RoadReservations.Find(OtherID);
            check(Reservation);
            for (const FBox& A:Segments) for (const FBox& B:*Reservation)
                if (A.Intersect(B)) { Blocker=OtherID; break; }
            if (Blocker!=INDEX_NONE) break;
        }
        for (int32 OtherIndex:VehicleIndex.Query(Segments))
        {
            const auto& Other=TrafficVehicles[OtherIndex];
            if (Other==Vehicle || !IsValid(Other)) continue;
            const FVector E=Other->GetActorForwardVector().GetAbs()*210.f+Other->GetActorRightVector().GetAbs()*730.f+FVector(0,0,250);
            for (const FBox& A:Segments)
                if (A.Intersect(FBox(Other->GetActorLocation()-E,Other->GetActorLocation()+E))) { Blocker=Other->VehicleID; break; }
        }
        if (Blocker!=INDEX_NONE)
        {
            // Requests change only at a waypoint. While stopped there, retain the
            // physical vehicle envelope, not an unentered road needed by its blocker.
            // Never revoke the reservation of a vehicle already traversing a segment.
            RoadBlockers.Add(ID,Blocker);
            if(!RoadWaitSince.Contains(ID)) RoadWaitSince.Add(ID,TrafficClock);
            if(Jobs.IsValidIndex(FleetIndex)) Jobs[FleetIndex].TrafficWaitSeconds+=Dt;
            RoadReservations.Remove(ID); ReservationIndex.Remove(ID); RoadTargets.Remove(ID);
            Vehicle->Speed=0;
            return false;
        }
        RoadReservations.Add(ID,Segments);
        ReservationIndex.Set(ID,Segments);
        RoadTargets.Add(ID,Target);
        RoadBlockers.Remove(ID);
        RoadWaitSince.Remove(ID);
    }
    const bool Arrived=Vehicle->MoveToPosition(Target,Dt,!StraightThrough);
    const int32 TrafficIndex=TrafficVehicles.IndexOfByKey(Vehicle);
    if (TrafficIndex!=INDEX_NONE) IndexTrafficVehicle(TrafficIndex);
    if (Arrived) FinishedRoadSegments.Add(ID);
    return Arrived;
}

bool APortSiteLogistics::Drive(int32 Lane,float Dt)
{
    auto& Job=Jobs[Lane]; auto* Vehicle=Vehicles[Lane].Get();
    if(LaneCount==9)
    {
        const FVector P=Vehicle->GetActorLocation(),Target=Job.Route[Job.Waypoint];
        const bool YardSide=FMath::Max(P.X,Target.X)>TerminalLayout::SiteCmX(16500)+1;
        Vehicle->SetActorRotation(FRotator(0,YardSide?90:0,0));
    }
    if (!MoveVehicle(Vehicle,Job.Route[Job.Waypoint],Dt))
    {
        const double* Since=RoadWaitSince.Find(Vehicle->VehicleID);
        if(LaneCount==9 && Since && TrafficClock-*Since>=8 && TrafficClock-Job.LastRerouteAt>=10 &&
            !RoadTargets.Contains(Vehicle->VehicleID) && !(Job.Stage==3 && Job.bRMGReserved))
        {
            Job.LastRerouteAt=TrafficClock;
            PlanRoadRoute(Lane,true);
        }
        return false;
    }
    if (Job.Stage==3 && STSOwners[Job.STS]==Lane && FVector::Dist2D(Vehicle->GetActorLocation(),CargoQuay(Job.Cargo))>1600) STSOwners[Job.STS]=INDEX_NONE;
    if (Job.Stage==5 && !Job.bYardReleased && FMath::Abs(Vehicle->GetActorLocation().Y-YardHandover(Yard[Job.Slot]).Y)>450)
    { RMGBusy[Job.RMG]=false; Job.bYardReleased=true; }
    // Once clear of the yard's narrow handover lanes, align the empty vehicle
    // with the north/south road before entering the parking aisle.
    if (LaneCount!=9 && Job.Waypoint==1 && Job.Stage==5)
    { Vehicle->SetActorRotation(FRotator::ZeroRotator); IndexTrafficVehicle(TrafficVehicles.IndexOfByKey(Vehicle)); }
    if (LaneCount!=9 && Job.Stage==3 && Job.Waypoint==Job.Route.Num()-3) Vehicle->SetActorRotation(FRotator(0,90,0));
    ++Job.Waypoint;
    // Stop off the spine at the existing yard entry point, clear of the handover and exit lanes.
    if(Job.Stage==3 && !Job.bRMGReserved && Job.Waypoint==Job.Route.Num()-1) return true;
    if (Job.Waypoint<Job.Route.Num()) return false;
    if (Job.Stage==5) Vehicle->SetActorRotation(FRotator::ZeroRotator);
    return true;
}

void APortSiteLogistics::Freeze(bool Paused)
{
    for (const auto& Crane:Equipment) Crane->SetOperationPaused(Paused);
    if (Paused==bWasPaused) return;
    bWasPaused=Paused;
    // Cargo on the AGV is attached; crane-managed cargo is frozen by its crane.
    for (auto& Job:Jobs)
        if (Job.Actor.IsValid() && Job.Actor->LocationOwner==ECargoOwner::AGV && !Job.Actor->GetAttachParentActor())
            Job.Actor->GetBody()->SetSimulatePhysics(!Paused);
}

void APortSiteLogistics::Stop(const FString& Reason)
{ Fault=Reason; Freeze(true); UE_LOG(LogTemp,Error,TEXT("SITE_LOGISTICS_FAIL: %s"),*Reason); }

void APortSiteLogistics::Advance(float Dt,bool Paused)
{
    if (!bReady) return;
    SimulationTime+=Dt;
    ExportDashboard(Paused);
    if(Paused) for(int32 Index:PreparedCargo) if(Index!=INDEX_NONE) Manifest[Index].PreparedPausedSeconds+=Dt;
    for(auto& Job:Jobs) if(Job.Stage) { Job.Time+=Dt; if(Paused) Job.PausedSeconds+=Dt; }
    Freeze(Paused || !Fault.IsEmpty());
    if (Paused || !Fault.IsEmpty())
    { for(const auto& Crane:Equipment) Crane->Advance(Dt,true); return; }
    TrafficClock+=Dt;
    if (LastProgressDelivered!=Delivered) { NoProgressTime=0; LastProgressDelivered=Delivered; }
    else NoProgressTime+=Dt;
    if (NoProgressTime>500)
    {
        NoProgressTime=0;
        UE_LOG(LogTemp,Display,TEXT("TRAFFIC_DIAGNOSTIC: delivered=%d queued_handoffs=%d"),Delivered,QueuedHandoffs);
        for (int32 I=0;I<Jobs.Num();++I)
        {
            const auto& J=Jobs[I];
            if (!J.Stage) continue;
            UE_LOG(LogTemp,Display,TEXT("TRAFFIC_V%d: stage=%d sts=%d rmg=%d wp=%d/%d pos=%s target=%s blocked_by=%d"),Vehicles[I]->VehicleID,J.Stage,J.STS,J.RMG,J.Waypoint,J.Route.Num(),*Vehicles[I]->GetActorLocation().ToCompactString(),J.Route.IsValidIndex(J.Waypoint)?*J.Route[J.Waypoint].ToCompactString():TEXT("none"),RoadBlockers.FindRef(Vehicles[I]->VehicleID));
        }
    }
    for (const auto& Crane:Equipment)
    {
        // The central berth advances its reserved RMG in TickRMGTransfer.
        if (CentralReservation==INDEX_NONE || Crane!=CentralCrane(CentralReservation)) Crane->Advance(Dt,false);
        if (!Crane->Fault.IsEmpty()) { Stop(Crane->Fault); return; }
    }
    int32 Moving=0;
    for (const auto& Vehicle:Vehicles) Moving+=Vehicle->Speed>1.f;
    PeakMovingVehicles=FMath::Max(PeakMovingVehicles,Moving);
    if (LaneCount==9) ScheduleFleet();
    if(!Fault.IsEmpty()) return;
    // Age actual blocked road requests, not cargo preparation or crane service.
    // Existing reservations are never preempted by an older waiting request.
    TArray<int32> Order;
    for (int32 I=0;I<Jobs.Num();++I) Order.Add(I);
    Order.StableSort([this](int32 A,int32 B)
    {
        const double* WaitA=RoadWaitSince.Find(Vehicles[A]->VehicleID);
        const double* WaitB=RoadWaitSince.Find(Vehicles[B]->VehicleID);
        if(WaitA || WaitB) return TrafficJunctionPolicy::OlderRequestFirst(WaitA,WaitB,Vehicles[A]->VehicleID,Vehicles[B]->VehicleID);
        if(bool(Jobs[A].Stage)!=bool(Jobs[B].Stage)) return Jobs[A].Stage!=0;
        if(Jobs[A].Stage && Jobs[B].Stage && Jobs[A].StartedAt!=Jobs[B].StartedAt) return Jobs[A].StartedAt<Jobs[B].StartedAt;
        return Vehicles[A]->VehicleID<Vehicles[B]->VehicleID;
    });
    for (int32 Lane:Order)
    {
        auto& Job=Jobs[Lane]; auto* Vehicle=Vehicles[Lane].Get();
        if (Job.Stage==0) { if (LaneCount==8) Dispatch(Lane); if(!Fault.IsEmpty()) return; continue; }
        if (Job.LastStage!=Job.Stage || !Job.LastPosition.Equals(Vehicle->GetActorLocation(),10.f)) Job.StationaryTime=0;
        else Job.StationaryTime+=Dt;
        Job.LastStage=Job.Stage; Job.LastPosition=Vehicle->GetActorLocation();
        if (Job.StationaryTime>300)
        {
            Job.StationaryTime=0;
            int32 Current=Vehicle->VehicleID;
            for (int32 Depth=0;Depth<8;++Depth)
            {
                const int32 I=Current-100;
                if (!Jobs.IsValidIndex(I)) break;
                const auto& J=Jobs[I];
                UE_LOG(LogTemp,Display,TEXT("BLOCK_CHAIN: V%d stage=%d sts=%d wp=%d/%d pos=%s target=%s blocker=%d"),Current,J.Stage,J.STS,J.Waypoint,J.Route.Num(),*Vehicles[I]->GetActorLocation().ToCompactString(),J.Route.IsValidIndex(J.Waypoint)?*J.Route[J.Waypoint].ToCompactString():TEXT("none"),RoadBlockers.FindRef(Current));
                Current=RoadBlockers.FindRef(Current);
            }
        }
        if (Job.Time>12000.f) { Stop(TEXT("Shipment timeout")); return; }
        auto* Cargo=Job.Actor.Get();
        switch(Job.Stage)
        {
        case 7:
            if (Drive(Lane,Dt)) Job.Stage=8;
            break;
        case 8: break;
        case 6:
            if (!Drive(Lane,Dt)) break;
            if (!Vehicle->GetActorLocation().Equals(CargoQuay(Job.Cargo),.1f) || Vehicle->Speed>.1f)
            { Stop(TEXT("AGV did not stop at its cargo-specific STS handover")); return; }
            Equipment[YardCraneCount+Job.STS]->SetDestinationReady(true);
            Job.Stage=1; break;
        case 1:
            if (Equipment[YardCraneCount+Job.STS]->IsBusy()) break;
            if (!Cargo || !Cargo->GetActorLocation().Equals(Vehicle->CargoPosition(),10.f) || Vehicle->Speed>0)
            { Stop(TEXT("STS / AGV handover alignment")); return; }
            Cargo->GetBody()->SetSimulatePhysics(false);
            Cargo->AttachToComponent(Vehicle->GetRootComponent(),FAttachmentTransformRules::KeepWorldTransform);
            Cargo->LocationOwner=ECargoOwner::AGV;
            Vehicle->SetPayload(Cargo->MassKg);
            Manifest[Job.Cargo].HandoverMask|=1;
            RecordVesselEvent(Job.Cargo,false);
            Job.HandoverAt=SimulationTime;
            Job.STSSeconds=Equipment[YardCraneCount+Job.STS]->LastJobSeconds;
            UE_LOG(LogTemp,Display,TEXT("SITE_HANDOVER: C%d STS -> AGV%d"),Manifest[Job.Cargo].ID,100+Lane);
            Job.Stage=2;
            PrepareNextCargo(Job.STS);
            break;
        case 2:
            if (!ReserveYardApproach(Lane)) { Job.YardWaitSeconds+=Dt; break; }
            PrepareRoute(Lane,false); Job.Stage=3; break;
        case 9: // Physically stopped at the off-spine yard entry bay.
            if (!ReserveYard(Lane)) { Job.YardWaitSeconds+=Dt; break; }
            Job.Stage=3; break;
        case 3:
            if (!Drive(Lane,Dt)) break;
            if(!Job.bRMGReserved) { Job.Stage=9; break; }
            YardApproachOwners[Yard[Job.Slot].Block]=INDEX_NONE;
            // Keep the twist locks engaged until the RMG actually picks up the box.
            if (!Equipment[Job.RMG]->AssignCargo(Cargo,Vehicle->CargoPosition(),Yard[Job.Slot].Position,false,true,Vehicle))
            { Stop(TEXT("AGV / RMG reservation handover")); return; }
            Manifest[Job.Cargo].HandoverMask|=2;
            UE_LOG(LogTemp,Display,TEXT("SITE_HANDOVER: C%d AGV%d -> RMG%d"),Manifest[Job.Cargo].ID,100+Lane,Job.RMG+1);
            Job.Stage=4; break;
        case 4:
            if (Equipment[Job.RMG]->IsBusy()) break;
            if (!Cargo || Cargo->LocationOwner!=ECargoOwner::Yard || !Cargo->GetActorLocation().Equals(Yard[Job.Slot].Position,10.f))
            { Stop(TEXT("RMG yard placement mismatch")); return; }
            Yard[Job.Slot].Occupied=true;
            Cargo->GetBody()->SetSimulatePhysics(false);
            Cargo->SetActorLocation(Yard[Job.Slot].Position);
            PlacedContainers.Add(Cargo);
            Manifest[Job.Cargo].HandoverMask|=4;
            Manifest[Job.Cargo].State=2; ++Delivered; ++Vehicle->CompletedJobs;
            Vehicle->SetPayload(0);
            RecordVesselEvent(Job.Cargo,true);
            ResultsCsv+=FString::Printf(TEXT("C%d,%d,%.0f,%.0f,%.0f,%d,%d,%d,%d,%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n"),
                Manifest[Job.Cargo].ID,Cargo->LengthFt,Cargo->TareMassKg,Cargo->CargoMassKg,Cargo->MassKg,
                Yard[Job.Slot].Block+1,Job.Slot,Job.STS+1,100+Lane,Job.RMG+1,Job.StartedAt,Job.HandoverAt,SimulationTime,
                Job.Time,Job.STSSeconds,Job.PausedSeconds,Job.TrafficWaitSeconds,Job.YardWaitSeconds);
            if(!SaveReports()) { Stop(TEXT("Could not save site STS shipment report")); return; }
            UE_LOG(LogTemp,Display,TEXT("SITE_DELIVERED: C%d via AGV%d -> RMG%d; total=%d"),Manifest[Job.Cargo].ID,100+Lane,Job.RMG+1,Delivered);
            Job.Actor=nullptr;
            PrepareRoute(Lane,true); Job.Stage=5; break;
        case 5:
            if (!Drive(Lane,Dt)) break;
            if (!Job.bYardReleased) RMGBusy[Job.RMG]=false;
            Job=FSiteTransfer(); break;
        default: Stop(TEXT("Unknown dispatch stage")); return;
        }
    }
}

void APortSiteLogistics::ResetLogistics()
{
    if (!bReady) return;
    for (auto& Job:Jobs) if (Job.Actor.IsValid()) Job.Actor->DetachFromActor(FDetachmentTransformRules::KeepWorldTransform);
    PlacedContainers.Reset();
    for (const auto& Crane:Equipment) Crane->ResetOperation();
    for (auto& Cargo:Manifest)
    {
        auto* Actor=Cargo.Actor.Get();
        check(Actor);
        Actor->DetachFromActor(FDetachmentTransformRules::KeepWorldTransform);
        Actor->GetBody()->SetSimulatePhysics(false);
        Actor->SetActorLocationAndRotation(Cargo.Transform.GetLocation(),Cargo.Transform.GetRotation(),false,nullptr,ETeleportType::TeleportPhysics);
        Actor->LocationOwner=ECargoOwner::Ship;
        Cargo.State=0; Cargo.HandoverMask=0;
    }
    for (auto& Slot:Yard) if (Slot.Reserved)
    { Slot.Occupied=false; Slot.Mesh->UpdateInstanceTransform(Slot.Instance,SiteLogistics::Hidden(SiteLogistics::YardTransform(Slot.Position)),false,true,true); }
    for (int32 I=0;I<Jobs.Num();++I) { Jobs[I]=FSiteTransfer(); Vehicles[I]->ResetVehicle(FleetPark(I)); }
    STSOwners.Init(INDEX_NONE,LaneCount);
    NextVehicles.Init(INDEX_NONE,LaneCount); PreparedStarted.Init(false,LaneCount); QueuedHandoffs=0;
    NoProgressTime=0; LastProgressDelivered=0; RoadBlockers.Reset(); RoadWaitSince.Reset(); TrafficClock=0;
    YardApproachOwners.Init(INDEX_NONE,TerminalLayout::YardBlockCount); BlocksBusy.Init(false,TerminalLayout::YardBlockCount); RMGBusy.Init(false,YardCraneCount); PreparedCargo.Init(INDEX_NONE,LaneCount); SlotAssigned.Init(false,Yard.Num());
    CentralReservation=CentralPending=INDEX_NONE;
    ReservationIndex.Reset(); VehicleIndex.Reset(); bTrafficIndexReady=false;
    RoadReservations.Reset(); RoadTargets.Reset(); FinishedRoadSegments.Reset(); PeakMovingVehicles=PrefetchedJobs=Dispatched=Delivered=0; Fault.Empty(); bWasPaused=false;
    BeginReport();
    if(!STSProfile.bReady) Stop(TEXT("Site STS reference unavailable"));
}

int32 APortSiteLogistics::ShipRemaining() const
{
    int32 Count=0;
    for (const auto& Cargo:Manifest) Count+=Cargo.State==0;
    for (const auto& Job:Jobs) if (Job.Actor.IsValid() && Job.Actor->LocationOwner==ECargoOwner::Ship) ++Count;
    for (int32 Index:PreparedCargo) if (Index!=INDEX_NONE && Manifest[Index].Actor->LocationOwner==ECargoOwner::Ship) ++Count;
    return Count;
}
int32 APortSiteLogistics::InTransit() const { return Manifest.Num()-ShipRemaining()-Delivered; }

bool APortSiteLogistics::Validate(FString& Error) const
{
    if (!Fault.IsEmpty()) { Error=Fault; return false; }
    if (BaselineYard-InitialYard!=ReceivingCapacity || ReceivingCapacity<InitialShipCount()) { Error=TEXT("Insufficient receiving capacity"); return false; }
    if (LaneCount==9)
    {
        int32 VesselCounts[3]={0,0,0};
        for (const auto& Cargo:Manifest)
        {
            if (Cargo.STS<0 || Cargo.STS>=9) { Error=TEXT("Invalid berth STS assignment"); return false; }
            ++VesselCounts[Cargo.STS/3];
        }
        if (CentralCount!=0 || VesselCounts[0]!=TerminalLayout::ContainersPerVessel || VesselCounts[1]!=TerminalLayout::ContainersPerVessel ||
            VesselCounts[2]!=TerminalLayout::ContainersPerVessel || ReceivingCapacity!=TerminalLayout::TotalVesselContainers || Vehicles.Num()!=60)
        { Error=TEXT("Three equal vessels / unified fleet inventory mismatch"); return false; }
    }
    if (LaneCount==9)
        for (const auto& Vehicle:Vehicles)
        {
            const FVector P=Vehicle->GetActorLocation();
            const FVector E=Vehicle->GetActorForwardVector().GetAbs()*210.f+
                Vehicle->GetActorRightVector().GetAbs()*730.f;
            if (!TerminalLayout::AGVEnvelopeInside(P.X-E.X,P.X+E.X,P.Y-E.Y,P.Y+E.Y))
            { Error=FString::Printf(TEXT("AGV %d escaped yard/road bounds"),Vehicle->VehicleID); return false; }
        }
    int32 Ship=0,Active=0,Placed=0,Reserved=0,OccupiedReservations=0,CentralOccupied=0;
    TSet<int32> IDs,JobCargo,JobSlots,ActiveRMGs,PlannedDestinations;
    TSet<const APortContainerActor*> Actors;
    if (ShipContainers.Num()!=Manifest.Num()) { Error=TEXT("Ship actor/manifest count mismatch" ); return false; }
    for (const auto& Cargo:Manifest)
    {
        const auto* Actor=Cargo.Actor.Get();
        if (!IsValid(Actor) || Actor->GetOwner()!=this || Actors.Contains(Actor) ||
            Actor->ContainerID!=FName(*FString::Printf(TEXT("C%02d"),Cargo.ID)) || Cargo.STS<0 || Cargo.STS>=LaneCount)
        { Error=TEXT("Missing, replaced, duplicate or unassigned ship container actor"); return false; }
        if(!Yard.IsValidIndex(Cargo.PlannedSlot) || !Yard[Cargo.PlannedSlot].Reserved || Yard[Cargo.PlannedSlot].Central ||
            PlannedDestinations.Contains(Cargo.PlannedSlot))
        { Error=TEXT("Missing, duplicate or invalid pre-discharge yard destination"); return false; }
        PlannedDestinations.Add(Cargo.PlannedSlot);
        if((Actor->LengthFt!=20 && Actor->LengthFt!=40 && Actor->LengthFt!=45) ||
            !FMath::IsNearlyEqual(Actor->MassKg,Actor->TareMassKg+Actor->CargoMassKg,1.f) ||
            !FMath::IsFinite(Actor->MassKg) || Actor->MassKg<=0)
        { Error=TEXT("Invalid container size or gross mass specification"); return false; }
        Actors.Add(Actor);
        if (Cargo.State==0 && (Actor->LocationOwner!=ECargoOwner::Ship || Actor->GetAttachParentActor() ||
            Actor->GetBody()->IsSimulatingPhysics() || !Actor->GetActorLocation().Equals(Cargo.Transform.GetLocation(),.1f)))
        { Error=TEXT("Secured ship cargo moved or lost its ship state"); return false; }
        if (IDs.Contains(Cargo.ID)) { Error=TEXT("Duplicate manifest cargo ID"); return false; }
        if (Cargo.State==2 && Cargo.HandoverMask!=7) { Error=TEXT("Delivered cargo bypassed STS/AGV/RMG handover" ); return false; }
        if (Cargo.State==2 && !Actor->GetActorLocation().Equals(Yard[Cargo.PlannedSlot].Position,10.f))
        { Error=TEXT("Delivered cargo bypassed its planned yard destination"); return false; }
        IDs.Add(Cargo.ID); Ship+=Cargo.State==0; Active+=Cargo.State==1; Placed+=Cargo.State==2;
    }
    for (const auto& Slot:Yard)
    {
        Reserved+=Slot.Reserved; OccupiedReservations+=Slot.Reserved && Slot.Occupied; CentralOccupied+=Slot.Central && Slot.Occupied;
        if (Slot.Occupied && Slot.Below!=INDEX_NONE && !Yard[Slot.Below].Occupied)
        { Error=TEXT("Yard stack has an empty supporting tier"); return false; }
    }
    if (Ship+Active+Placed!=Manifest.Num() || Placed!=Delivered || Reserved!=ReceivingCapacity || OccupiedReservations!=Delivered+CentralOccupied)
    { Error=TEXT("Inventory conservation / reservation mismatch"); return false; }
    TSet<int32> CentralIDs,CentralBlocks;
    if (CentralSlots.Num()!=CentralCount) { Error=TEXT("Central yard mapping count mismatch"); return false; }
    for (int32 Slot:CentralSlots)
    {
        if (!Yard.IsValidIndex(Slot) || !Yard[Slot].Central || !Yard[Slot].Reserved || CentralIDs.Contains(Slot))
        { Error=TEXT("Duplicate or unreserved central yard destination"); return false; }
        CentralIDs.Add(Slot); CentralBlocks.Add(Yard[Slot].Block);
    }
    if (CentralCount>0 && CentralBlocks.Num()!=TerminalLayout::YardBlockCount) { Error=TEXT("Berth cargo must be distributed across all yards"); return false; }
    if (CentralReservation!=INDEX_NONE)
    {
        if (!BlocksBusy[Yard[CentralSlots[CentralReservation]].Block])
        { Error=TEXT("Central transfer lost its road or yard reservation"); return false; }
        ActiveRMGs.Add(Yard[CentralSlots[CentralReservation]].Block*2);
        ActiveRMGs.Add(Yard[CentralSlots[CentralReservation]].Block*2+1);
    }
    int32 Physical=0;
    for (int32 Lane=0;Lane<Jobs.Num();++Lane)
    {
        const auto& Job=Jobs[Lane];
        if (!Job.Stage) continue;
        if (Job.Stage==7 || Job.Stage==8)
        {
            if (!NextVehicles.IsValidIndex(Job.STS) || NextVehicles[Job.STS]!=Lane ||
                PreparedCargo[Job.STS]==INDEX_NONE || Job.Actor.IsValid() || Job.Cargo!=INDEX_NONE)
            { Error=TEXT("Invalid queued AGV reservation"); return false; }
            if (Vehicles[Lane]->Speed>0 && !RoadReservations.Contains(Vehicles[Lane]->VehicleID))
            { Error=TEXT("Queued AGV moved without a road reservation"); return false; }
            continue;
        }
        if (JobCargo.Contains(Job.Cargo)) { Error=TEXT("Duplicate job cargo"); return false; }
        JobCargo.Add(Job.Cargo);
        if (!Manifest.IsValidIndex(Job.Cargo)) { Error=TEXT("Missing job manifest entry"); return false; }
        const FVector Handover=CargoQuay(Job.Cargo);
        if (Job.Stage==6 && (Job.Route.IsEmpty() || !Job.Route.Last().Equals(Handover,.1f)))
        { Error=TEXT("AGV approach does not end at the cargo-specific handover"); return false; }
        if ((Job.Stage==1 || Job.Stage==2) &&
            (!Vehicles[Lane]->GetActorLocation().Equals(Handover,.1f) || Vehicles[Lane]->Speed>.1f))
        { Error=TEXT("AGV left its handover before cargo transfer"); return false; }
        if (Job.Slot!=INDEX_NONE)
        {
            if (Job.Slot!=Manifest[Job.Cargo].PlannedSlot || Yard[Job.Slot].Central || JobSlots.Contains(Job.Slot) ||
                (Job.bRMGReserved && !Job.bYardReleased && (ActiveRMGs.Contains(Job.RMG) || !RMGBusy[Job.RMG])))
            { Error=TEXT("Duplicate or missing slot/RMG reservation"); return false; }
            JobSlots.Add(Job.Slot);
            if (Job.bRMGReserved && !Job.bYardReleased) ActiveRMGs.Add(Job.RMG);
            else if (Job.bYardReleased && (Job.Stage!=5 || !Yard[Job.Slot].Occupied))
            { Error=TEXT("Yard released before placement/departure"); return false; }
        }
        if(Job.Slot!=INDEX_NONE && !Job.bRMGReserved &&
            ((Job.Stage!=3 && Job.Stage!=9) || YardApproachOwners[Yard[Job.Slot].Block]!=Lane))
        { Error=TEXT("Missing yard approach admission"); return false; }
        if(Job.Stage==9 && (Vehicles[Lane]->Speed>.1f || Job.Waypoint!=Job.Route.Num()-1 ||
            !Vehicles[Lane]->GetActorLocation().Equals(Job.Route[Job.Waypoint-1],.1f)))
        { Error=TEXT("AGV is not stopped inside its yard entry bay"); return false; }
        if (Job.Actor.IsValid())
        {
            ++Physical;
            if (Job.Actor!=Manifest[Job.Cargo].Actor || Job.Actor->ContainerID!=FName(*FString::Printf(TEXT("C%02d"),Manifest[Job.Cargo].ID)))
            { Error=TEXT("Cargo identity changed during handover"); return false; }
            if ((Job.Stage==2 || Job.Stage==3 || Job.Stage==9) && (Job.Actor->LocationOwner!=ECargoOwner::AGV || Job.Actor->GetAttachParentActor()!=Vehicles[Lane] ||
                !Job.Actor->GetActorLocation().Equals(Vehicles[Lane]->CargoPosition(),1.f)))
            { Error=TEXT("AGV cargo alignment/ownership mismatch"); return false; }
        }
        if (Vehicles[Lane]->Speed>0 && !RoadReservations.Contains(Vehicles[Lane]->VehicleID))
        { Error=TEXT("AGV moved without a path reservation"); return false; }
    }
    if (PlacedContainers.Num()!=Delivered) { Error=TEXT("Delivered cargo actor count mismatch" ); return false; }
    TSet<FName> PlacedIDs;
    for (const auto& Cargo:PlacedContainers)
    {
        if (!IsValid(Cargo) || Cargo->GetOwner()!=this || Cargo->LocationOwner!=ECargoOwner::Yard || Cargo->GetAttachParentActor() || PlacedIDs.Contains(Cargo->ContainerID))
        { Error=TEXT("Lost/duplicate/attached yard cargo actor" ); return false; }
        PlacedIDs.Add(Cargo->ContainerID);
    }
    for (int32 Index:PreparedCargo) if (Index!=INDEX_NONE)
    {
        if (JobCargo.Contains(Index) || Manifest[Index].State!=1) { Error=TEXT("Invalid prefetched STS cargo"); return false; }
        JobCargo.Add(Index); ++Physical;
    }
    if (Physical!=Active) { Error=TEXT("Physical/manifest cargo mismatch"); return false; }
    for (int32 I=0;I<TrafficVehicles.Num();++I)
        for (int32 J=I+1;J<TrafficVehicles.Num();++J)
        {
            const auto* A=TrafficVehicles[I].Get(); const auto* B=TrafficVehicles[J].Get();
            const FVector EA=A->GetActorForwardVector().GetAbs()*169.f+A->GetActorRightVector().GetAbs()*689.f+FVector(0,0,100);
            const FVector EB=B->GetActorForwardVector().GetAbs()*169.f+B->GetActorRightVector().GetAbs()*689.f+FVector(0,0,100);
            if (FBox(A->GetActorLocation()-EA,A->GetActorLocation()+EA).Intersect(FBox(B->GetActorLocation()-EB,B->GetActorLocation()+EB)))
            { Error=FString::Printf(TEXT("AGV%d and AGV%d traffic envelopes overlap"),A->VehicleID,B->VehicleID); return false; }
        }
    for (const auto& Crane:Equipment) if (!Crane->ValidateOperation(Error)) return false;
    return true;
}

void APortSiteLogistics::EndPlay(const EEndPlayReason::Type Reason)
{
    if (DashboardWrite.IsValid()) DashboardWrite.Wait();
    for (const auto& Cargo:ShipContainers) if (IsValid(Cargo)) Cargo->Destroy();
    PlacedContainers.Reset(); ShipContainers.Reset();
    for (const auto& Vehicle:Vehicles) if (IsValid(Vehicle)) Vehicle->Destroy();
    Super::EndPlay(Reason);
}

bool APortSiteLogistics::IsIdle() const
{
    for (const auto& Job:Jobs) if (Job.Stage) return false;
    for (int32 Index:PreparedCargo) if (Index!=INDEX_NONE) return false;
    return true;
}
APortAGVActor* APortSiteLogistics::LoadedVehicle(APortAGVActor* Preferred) const
{
    for (int32 I=0;I<Jobs.Num();++I)
        if (Vehicles[I]==Preferred && Jobs[I].Actor.IsValid() && Jobs[I].Actor->LocationOwner==ECargoOwner::AGV)
            return Vehicles[I];
    for (int32 I=0;I<Jobs.Num();++I)
        if (Jobs[I].Stage==3 && Jobs[I].Actor.IsValid() && Vehicles[I]->Speed>1.f) return Vehicles[I];
    return nullptr;
}
FString APortSiteLogistics::VehicleStatus() const
{
    int32 Moving=0,Loaded=0,Approaching=0,Loading=0,Queued=0;
    for (int32 I=0;I<Jobs.Num();++I)
    {
        Moving+=Vehicles[I]->Speed>1.f;
        Loaded+=Jobs[I].Actor.IsValid() && Jobs[I].Actor->LocationOwner==ECargoOwner::AGV;
        Approaching+=Jobs[I].Stage==6 || Jobs[I].Stage==7;
        Queued+=Jobs[I].Stage==7 || Jobs[I].Stage==8;
        Loading+=Jobs[I].Stage==1;
    }
    return FString::Printf(TEXT("AGV moving %d/60 | loaded %d | STS loading %d | next AGVs %d | yard %d/%d free | F follow"),Moving,Loaded,Loading,Queued,ReceivingCapacity-Delivered,ReceivingCapacity);
}
TArray<FVector> APortSiteLogistics::Snapshot() const
{
    TArray<FVector> Positions;
    for (const auto& Vehicle:Vehicles) Positions.Add(Vehicle->GetActorLocation());
    for (const auto& Cargo:ShipContainers) Positions.Add(Cargo->GetActorLocation());
    for (const auto& Crane:Equipment) { Positions.Add(Crane->GetActorLocation()); Positions.Add(Crane->HeadPosition()); }
    return Positions;
}

void APortSiteLogistics::BeginReport()
{
    SimulationTime=0;
    NextDashboardWall=0;
    for(int32 I=0;I<3;++I) VesselStarted[I]=VesselUnloaded[I]=VesselPlaced[I]=-1;
    ReportBase=FPaths::ProjectSavedDir()/TEXT("Results")/TEXT("Site_")+FGuid::NewGuid().ToString(EGuidFormats::Digits);
    ResultsCsv=TEXT("ContainerID,LengthFt,TareKg,CargoKg,GrossKg,PlannedYardBlock,PlannedSlot,STSLane,AGVID,RMGID,StartedAtSeconds,STSHandoverAtSeconds,FinalPlacementAtSeconds,ShipmentSeconds,STSSeconds,PausedSeconds,TrafficWaitSeconds,YardWaitSeconds\n");
    if(!SaveReports()) Stop(TEXT("Could not initialize site shipment report"));
    UE_LOG(LogTemp,Display,TEXT("SITE_REPORT: %s.csv"),*ReportBase);
}

bool APortSiteLogistics::SaveReports() const
{
    if(ReportBase.IsEmpty()) return false;
    IFileManager::Get().MakeDirectory(*FPaths::GetPath(ReportBase),true);
    const FString Snapshot=TEXT("{\"site_suspension_model\":\"reduced-order sway/yaw; assumed mounts; taut-rope tension; hoist shaft torque\",\"sts_profile\":")+
        (STSProfile.SnapshotJson.IsEmpty()?TEXT("{}"):STSProfile.SnapshotJson)+TEXT("}");
    return FFileHelper::SaveStringToFile(ResultsCsv,*(ReportBase+TEXT(".csv")),FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM) &&
        FFileHelper::SaveStringToFile(Snapshot,*(ReportBase+TEXT("_profile.json")),FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM);
}
FVector APortSiteLogistics::CentralSlot(int32 Index) const
{ return Yard[CentralSlots[Index]].Position; }
FVector APortSiteLogistics::CentralHandover(int32 Index) const
{ return YardHandover(Yard[CentralSlots[Index]]); }
APortWorkingCrane* APortSiteLogistics::CentralCrane(int32 Index) const
{
    const auto& Slot=Yard[CentralSlots[Index]];
    return Equipment[Slot.Crane];
}
bool APortSiteLogistics::ReserveCentral(int32 Index)
{
    if (CentralReservation==Index) return true;
    CentralPending=Index;
    const auto& Slot=Yard[CentralSlots[Index]];
    if (CentralReservation!=INDEX_NONE || YardApproachOwners[Slot.Block]!=INDEX_NONE || BlocksBusy[Slot.Block] || RMGBusy[Slot.Block*2] || RMGBusy[Slot.Block*2+1]) return false;
    // Reserve only the yard block; traffic paths are reserved independently.
    CentralReservation=Index; CentralPending=INDEX_NONE;
    BlocksBusy[Slot.Block]=true;
    return true;
}
void APortSiteLogistics::CompleteCentral(int32 Index,bool Occupied)
{
    check(CentralReservation==Index);
    auto& Slot=Yard[CentralSlots[Index]];
    Slot.Occupied=Occupied; BlocksBusy[Slot.Block]=false;
    CentralReservation=CentralPending=INDEX_NONE;
}
