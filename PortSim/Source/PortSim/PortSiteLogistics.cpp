#include "PortSiteLogistics.h"
#include "TerminalLayout.h"
#include "YardReservation.h"
#include "PortWorkingCrane.h"
#include "PortAGVActor.h"
#include "PortContainerActor.h"
#include "ContainerSpecification.h"
#include "AGVDispatchPolicy.h"
#include "AGVReference.h"
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
    bool AGVFootprintsOverlap(const APortAGVActor* A,const APortAGVActor* B)
    {
        constexpr float HalfWidth=169.f;
        constexpr float HalfLength=689.f;
        const FVector Delta=B->GetActorLocation()-A->GetActorLocation();
        const FVector Axes[]={A->GetActorForwardVector().GetSafeNormal2D(),
            A->GetActorRightVector().GetSafeNormal2D(),B->GetActorForwardVector().GetSafeNormal2D(),
            B->GetActorRightVector().GetSafeNormal2D()};
        auto Projection=[&](const APortAGVActor* Vehicle,const FVector& Axis)
        {
            return FMath::Abs(FVector::DotProduct(Vehicle->GetActorForwardVector(),Axis))*HalfWidth+
                FMath::Abs(FVector::DotProduct(Vehicle->GetActorRightVector(),Axis))*HalfLength;
        };
        for(const FVector& Axis:Axes)
            if(FMath::Abs(FVector::DotProduct(Delta,Axis))>
                Projection(A,Axis)+Projection(B,Axis)) return false;
        return true;
    }
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
    // Spawn, parking return, reset and road geometry share one bay layout.
    return FVector(TerminalLayout::SiteCmX(TerminalLayout::AGVParkX(Vehicle)),
        TerminalLayout::AGVParkY(Vehicle),0);
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
        Vehicle->InitializeVehicle(100+I);
        // A freshly spawned fleet must use the same perpendicular parking pose
        // as a reset vehicle.  Leaving the spawn rotation at zero made the first
        // dispatch chase the aisle sideways; later jobs happened to work because
        // ResetVehicle already restored the intended car-like heading.
        Vehicle->ResetVehicle(FleetPark(I));
        Vehicles.Add(Vehicle);
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
    Job=FSiteTransfer();
    Job.Cargo=PreparedCargo[STS]; Job.Actor=Manifest[Job.Cargo].Actor; Job.STS=STS;
    Job.Stage=6; Job.Waypoint=0;
    Job.StartedAt=Manifest[Job.Cargo].StartedAt; Job.Time=SimulationTime-Job.StartedAt;
    Job.PausedSeconds=Manifest[Job.Cargo].PreparedPausedSeconds;
    Equipment[YardCraneCount+STS]->SetHandoverVehicle(Vehicles[Vehicle]);
    PreparedCargo[STS]=INDEX_NONE; PreparedStarted[STS]=false; STSOwners[STS]=Vehicle;
    if (FromQueue) { NextVehicles[STS]=INDEX_NONE; ++QueuedHandoffs; }
    if(!PlanRoadRoute(Vehicle,false)) Stop(TEXT("No road route to STS handover"));
}

void APortSiteLogistics::AssignReturnSTS(int32 Vehicle)
{
    if(LaneCount!=9) return;
    auto& Job=Jobs[Vehicle];
    int32 Best=INDEX_NONE;
    double BestScore=TNumericLimits<double>::Max();
    for(int32 S=0;S<LaneCount;++S)
    {
        PrepareNextCargo(S);
        if(!Fault.IsEmpty()) return;
        if(PreparedCargo[S]==INDEX_NONE) continue;
        const int32 Queued=NextVehicles[S];
        // Only replace a parked, empty standby. An AGV already returning to an
        // assigned berth keeps that assignment and its prepared container.
        if(Queued!=INDEX_NONE && Jobs[Queued].Stage!=8) continue;
        const double Score=FVector::Dist2D(Vehicles[Vehicle]->GetActorLocation(),CargoQuay(PreparedCargo[S]))+
            (STSOwners[S]!=INDEX_NONE?15000.:0.);
        if(Score<BestScore) { BestScore=Score; Best=S; }
    }
    if(Best==INDEX_NONE) return;
    if(const int32 Queued=NextVehicles[Best]; Queued!=INDEX_NONE) Jobs[Queued]=FSiteTransfer();
    Job.ReturnSTS=Best; Job.ReturnCargo=PreparedCargo[Best];
    NextVehicles[Best]=Vehicle;
    UE_LOG(LogTemp,Display,TEXT("AGV_DIRECT_RETURN: V%d -> STS%d C%d"),
        Vehicles[Vehicle]->VehicleID,Best+1,Manifest[Job.ReturnCargo].ID);
}

void APortSiteLogistics::ScheduleFleet()
{
    auto Nearest=[&](FVector Target,int32 STS)->int32
    {
        const int32 PreferredPool=TerminalLayout::AGVBerthGroup(STS);
        auto InPreferredDepartureSet=[&](int32 Vehicle)
        {
            if(TerminalLayout::AGVParkRow(Vehicle)!=PreferredPool) return false;
            const float ParkY=TerminalLayout::AGVParkY(Vehicle);
            const float CrossY=TerminalLayout::AGVInboundCrossY(STS);
            constexpr float TurnClearance=1800.f;
            // All vehicles in one pool approach its merge from the same side.
            // This keeps a turning body out of the next occupied parking bay.
            return PreferredPool<2?ParkY>=CrossY+TurnClearance:
                ParkY<=CrossY-TurnClearance;
        };
        // Each vessel group first draws from one physical parking row.  This
        // prevents adjacent rows from turning into the same aisle together,
        // while the second pass keeps the whole fleet available as fallback.
        for (int32 Pass=0;Pass<2;++Pass)
        {
            int32 MinimumJobs=MAX_int32;
            for(int32 I=0;I<Vehicles.Num();++I)
                if(Jobs[I].Stage==0 && (Pass!=0 || InPreferredDepartureSet(I)))
                    MinimumJobs=FMath::Min(MinimumJobs,Vehicles[I]->CompletedJobs);
            int32 Best=INDEX_NONE; double BestScore=TNumericLimits<double>::Max();
            for (int32 I=0;I<Vehicles.Num();++I)
            {
                if (Jobs[I].Stage!=0 || (Pass==0 && !InPreferredDepartureSet(I))) continue;
                // Pickup ETA is the main cost. A bounded workload penalty breaks close
                // choices toward the less-used vehicle without sending a distant AGV.
                const double Score=AGVDispatchPolicy::Score(Vehicles[I]->GetActorLocation(),Target,STS,
                    Vehicles[I]->CompletedJobs,MinimumJobs);
                if (Best==INDEX_NONE || Score<BestScore ||
                    (FMath::IsNearlyEqual(Score,BestScore) && Vehicles[I]->VehicleID<Vehicles[Best]->VehicleID))
                { Best=I; BestScore=Score; }
            }
            if (Best!=INDEX_NONE) return Best;
        }
        return INDEX_NONE;
    };
    // Prepare every STS first. Each vessel group then admits the oldest ready
    // handover to its one-lane quay approach. The three vessel approaches run
    // concurrently, while opposite traffic inside one narrow berth lane never meets.
    for (int32 S=0;S<LaneCount;++S)
    {
        PrepareNextCargo(S);
        if(!Fault.IsEmpty()) return;
    }
    for (int32 S=0;S<LaneCount;++S)
    {
        if (STSOwners[S]!=INDEX_NONE || PreparedCargo[S]==INDEX_NONE || !PreparedStarted[S]) continue;
        const int32 Group=TerminalLayout::AGVBerthGroup(S);
        bool GroupOccupied=false;
        for(int32 GroupSTS=Group*3;GroupSTS<Group*3+3;++GroupSTS)
            GroupOccupied|=STSOwners[GroupSTS]!=INDEX_NONE;
        if(GroupOccupied) continue;
        int32 Winner=S;
        for(int32 Candidate=Group*3;Candidate<Group*3+3;++Candidate)
        {
            if(STSOwners[Candidate]!=INDEX_NONE || PreparedCargo[Candidate]==INDEX_NONE ||
                !PreparedStarted[Candidate]) continue;
            const double CandidateStarted=Manifest[PreparedCargo[Candidate]].StartedAt;
            const double WinnerStarted=Manifest[PreparedCargo[Winner]].StartedAt;
            if(CandidateStarted<WinnerStarted ||
                (FMath::IsNearlyEqual(CandidateStarted,WinnerStarted) && Candidate<Winner)) Winner=Candidate;
        }
        if(Winner!=S) continue;
        if (NextVehicles[S]!=INDEX_NONE)
        {
            if (Jobs[NextVehicles[S]].Stage==8) ActivateVehicle(NextVehicles[S],S,true);
        }
        else if (const int32 V=Nearest(CargoQuay(PreparedCargo[S]),S); V!=INDEX_NONE) ActivateVehicle(V,S,false);
    }
    for (int32 S=0;S<LaneCount;++S)
    {
        PrepareNextCargo(S);
        if(!Fault.IsEmpty()) return;
        if (STSOwners[S]==INDEX_NONE || NextVehicles[S]!=INDEX_NONE || PreparedCargo[S]==INDEX_NONE) continue;
        const int32 V=Nearest(CargoQuay(PreparedCargo[S]),S);
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
    {
        const float DockApproachY=Quay.Y+(Quay.Y<0?TerminalLayout::AGVBerthApproachOffset:
            -TerminalLayout::AGVBerthApproachOffset);
        constexpr float DockTurnRadius=1000.f;
        const float DockDirection=Quay.Y>DockApproachY?1.f:-1.f;
        const FVector ArcCenter(Quay.X+DockTurnRadius,DockApproachY+DockDirection*DockTurnRadius,0);
        Job.Route={FVector(TerminalLayout::SiteCmX(TerminalLayout::AGVBerthApproachX),Vehicles[Lane]->GetActorLocation().Y,0),
            FVector(TerminalLayout::SiteCmX(TerminalLayout::AGVBerthApproachX),DockApproachY,0),
            FVector(Quay.X+DockTurnRadius,DockApproachY,0),
            ArcCenter+FVector(-.70710678f*DockTurnRadius,-DockDirection*.70710678f*DockTurnRadius,0),
            FVector(Quay.X,DockApproachY+DockDirection*DockTurnRadius,0),Quay};
    }
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
    // Perimeter links remain available when the internal crossings are congested.
    Network.Add(P(3500,TerminalLayout::AGVSouthLoadedY),P(16500,TerminalLayout::AGVSouthLoadedY));
    Network.Add(P(16500,South),P(3500,South));
    Network.Add(P(5500,North),P(16500,North));
    Network.Add(P(9000,North),P(7200,North));
    // Cross between, never through, occupied parking bays. Alternating one-way
    // crossings avoid head-on queues and give loaded and returning AGVs shortcuts.
    for(int32 Gap=0;Gap<TerminalLayout::AGVParkBays-1;++Gap)
    {
        const float Y=TerminalLayout::AGVParkCrossingY(Gap);
        if(TerminalLayout::AGVParkCrossingToYard(Gap)) Network.Add(P(3500,Y),P(16500,Y));
        else Network.Add(P(16500,Y),P(3500,Y));
    }
    for(int32 Block=0;Block<TerminalLayout::YardBlockCount;++Block)
    {
        const float Y=TerminalLayout::BlockY(Block)*100;
        Network.Add(P(12500,Y+2050),P(16500,Y+2050));
        Network.Add(P(16500,Y+2500),P(12500,Y+2500));
    }
    const int32 DestinationCargo=Job.Stage==5 && Job.ReturnCargo!=INDEX_NONE?Job.ReturnCargo:Job.Cargo;
    const FVector Quay=CargoQuay(DestinationCargo),Park=FleetPark(Lane);
    Network.Add(P(3500,Quay.Y),P(5500,Quay.Y),true);
    Network.Add(P(7200,Quay.Y),Quay);
    Network.Add(Park,P(TerminalLayout::AGVParkExitX(Lane),Park.Y));
    Network.Add(P(TerminalLayout::AGVParkReturnX(Lane),Park.Y),Park);
    FVector Goal=Quay;
    if(Job.Stage==3 || Job.Stage==5)
    {
        const auto& Slot=Yard[Job.Slot]; const FVector Dock=YardHandover(Slot);
        const float Y=TerminalLayout::BlockY(Slot.Block)*100;
        const FVector Entry(Dock.X+1800,Y+2050,0),Exit(Dock.X,Y+2500,0);
        Network.Add(P(12500,Entry.Y),Entry);
        Network.Add(Dock,Exit);
        Network.Add(Exit,P(12500,Exit.Y));
        Goal=Job.Stage==3?Entry:(Job.ReturnSTS!=INDEX_NONE?Quay:Park);
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
        const int32 End=Job.Stage==3?Job.YardEntryWaypoint+1:Job.Route.Num();
        if(Total(Candidate,0,Candidate.Num())+500>=Total(Job.Route,Job.Waypoint,End)) return false;
        ++Job.Reroutes;
        UE_LOG(LogTemp,Display,TEXT("AGV_REROUTE: V%d stage=%d count=%d"),ID,Job.Stage,Job.Reroutes);
    }
    if(Job.Stage==3)
    {
        Job.YardEntryWaypoint=Candidate.Num()-1;
        const FVector Dock=YardHandover(Yard[Job.Slot]);
        Candidate.Add(FVector(Dock.X,Goal.Y,0));
        Candidate.Add(Dock);
    }
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
    auto& Job=Jobs[Lane];
    const FVector Quay=CargoQuay(Job.Cargo),Dock=YardHandover(Yard[Job.Slot]);
    const float BlockY=TerminalLayout::BlockY(Yard[Job.Slot].Block)*100.f;
    Job.Route.Reset(); Job.Waypoint=0;
    if(!Return)
    {
        const float RoadX=TerminalLayout::SiteCmX(BlockY+2050>=Quay.Y?12500.f:16500.f);
        Job.Route={FVector(RoadX,Quay.Y,0),FVector(RoadX,BlockY+2050,0),
            FVector(Dock.X+1800,BlockY+2050,0),FVector(Dock.X,BlockY+2050,0),Dock};
        Job.YardEntryWaypoint=2;
    }
    else
    {
        const float RoadX=TerminalLayout::SiteCmX(Quay.Y+2000>=BlockY+2500?12500.f:16500.f);
        Job.Route={FVector(Dock.X,BlockY+2500,0),FVector(RoadX,BlockY+2500,0),
            FVector(RoadX,Quay.Y+2000,0),FVector(Quay.X,Quay.Y+2000,0),Quay};
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
    const int32 FleetIndex=Vehicles.IndexOfByKey(Vehicle);
    if (Jobs.IsValidIndex(FleetIndex))
    {
        const auto& Job=Jobs[FleetIndex];
        const bool Docking=Job.Waypoint==Job.Route.Num()-1;
        const bool PrecisionApproach=(Job.Stage==6 || Job.Stage==3 || Job.Stage==5) &&
            Job.Route.Num()-Job.Waypoint<=4;
        float Acceptance=PrecisionApproach?(Job.Stage==5?100.f:500.f):-1.f;
        if(Job.Route.IsValidIndex(Job.Waypoint+1))
        {
            // Classify route geometry from immutable FMS nodes. Using the live
            // vehicle position made a straight waypoint become a "corner" as
            // soon as the chassis passed it, shrinking acceptance to 25 cm and
            // trapping the AGV in a circle around the point.
            const FVector PreviousNode=Job.Route.IsValidIndex(Job.Waypoint-1)?
                Job.Route[Job.Waypoint-1]:Vehicle->GetActorLocation();
            const FVector IntoNode=(Target-PreviousNode).GetSafeNormal2D();
            const FVector Outgoing=Job.Route[Job.Waypoint+1]-Target;
            const FVector OutOfNode=Outgoing.GetSafeNormal2D();
            // Arc control points are only about 4.4 m apart at the reference
            // minimum radius.  The normal 8 m road-node tolerance would skip
            // them and recreate a lateral right-angle command.
            if(!IntoNode.IsNearlyZero() && !OutOfNode.IsNearlyZero() &&
                FVector::DotProduct(IntoNode,OutOfNode)<.995f)
            {
                // Keep every short sampled arc chord, but let the final chord
                // hand off to its long tangent road. A minimum-radius chassis
                // otherwise circles the exact mathematical endpoint forever.
                Acceptance=Outgoing.Size2D()>=AGVReference::MinimumInnerTurnRadiusCm?
                    (Job.Stage==5?100.f:500.f):100.f;
            }
        }
        Vehicle->SetFMSCommand(Target,Job.Waypoint,Job.Route.Num(),Docking,Acceptance);
    }
    else if(!Vehicle->Sensors.FMSNextNode.Equals(Target,.01f)) Vehicle->SetFMSCommand(Target,0,1,true);
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
    auto HasIntersectionPriority=[&](const APortAGVActor* Other)
    {
        const int32 OtherIndex=Vehicles.IndexOfByKey(Other);
        if(!Jobs.IsValidIndex(FleetIndex) || !Jobs.IsValidIndex(OtherIndex)) return false;
        const auto& CurrentJob=Jobs[FleetIndex];
        const auto& OtherJob=Jobs[OtherIndex];
        if(CurrentJob.Stage==0 || CurrentJob.Stage==8 || OtherJob.Stage==0 || OtherJob.Stage==8) return false;
        auto Rank=[](int32 Stage)
        {
            if(Stage==3 || Stage==4) return 3; // loaded AGV clears the berth/yard road first
            if(Stage==5) return 2;             // empty return clears the handover next
            return 1;                          // inbound empty AGV yields at a crossing
        };
        const int32 CurrentRank=Rank(CurrentJob.Stage),OtherRank=Rank(OtherJob.Stage);
        if(CurrentRank!=OtherRank) return CurrentRank>OtherRank;
        if(CurrentJob.Route.IsValidIndex(CurrentJob.Waypoint) &&
            OtherJob.Route.IsValidIndex(OtherJob.Waypoint))
        {
            const FVector CurrentDirection=Vehicle->PlannedMotionDirection(
                CurrentJob.Route[CurrentJob.Waypoint]);
            const FVector OtherDirection=Other->PlannedMotionDirection(
                OtherJob.Route[OtherJob.Waypoint]);
            if(!CurrentDirection.IsNearlyZero() && !OtherDirection.IsNearlyZero() &&
                FVector::DotProduct(CurrentDirection,OtherDirection)>.8f)
            {
                // On a shared one-way corridor the physically leading vehicle
                // always clears first, independent of route node counts or its
                // later STS destination.
                const float OtherAhead=FVector::DotProduct(
                    Other->GetActorLocation()-Vehicle->GetActorLocation(),CurrentDirection);
                if(FMath::Abs(OtherAhead)>100.f) return OtherAhead<0.f;
            }
        }
        auto RemainingRoute=[this](int32 Index)
        {
            const auto& RouteJob=Jobs[Index];
            if(!RouteJob.Route.IsValidIndex(RouteJob.Waypoint)) return 0.f;
            float Remaining=FVector::Dist2D(Vehicles[Index]->GetActorLocation(),
                RouteJob.Route[RouteJob.Waypoint]);
            for(int32 Node=RouteJob.Waypoint+1;Node<RouteJob.Route.Num();++Node)
                Remaining+=FVector::Dist2D(RouteJob.Route[Node-1],RouteJob.Route[Node]);
            return Remaining;
        };
        const float CurrentRemaining=RemainingRoute(FleetIndex);
        const float OtherRemaining=RemainingRoute(OtherIndex);
        if(!FMath::IsNearlyEqual(CurrentRemaining,OtherRemaining,1.f))
            return CurrentRemaining<OtherRemaining;
        if(!FMath::IsNearlyEqual(CurrentJob.StartedAt,OtherJob.StartedAt))
            return CurrentJob.StartedAt<OtherJob.StartedAt;
        return Vehicle->VehicleID<Other->VehicleID;
    };
    // LiDAR follows the physical longitudinal direction selected by the motion
    // controller.  Looking along the target chord would reintroduce a virtual
    // sideways sensor direction while the vehicle is turning.
    const FVector TravelDirection=Vehicle->PlannedMotionDirection(Target);
    float AheadDistance=-1,BehindDistance=-1;
    int32 LiDARBlocker=INDEX_NONE;
    if(!TravelDirection.IsNearlyZero())
    {
        const FVector Lateral(-TravelDirection.Y,TravelDirection.X,0);
        auto ProjectedExtent=[](const APortAGVActor* Actor,const FVector& Axis)
        {
            return FMath::Abs(FVector::DotProduct(Actor->GetActorForwardVector(),Axis))*AGVReference::PhysicalHalfWidthCm+
                FMath::Abs(FVector::DotProduct(Actor->GetActorRightVector(),Axis))*AGVReference::PhysicalHalfLengthCm;
        };
        const float OwnLong=ProjectedExtent(Vehicle,TravelDirection);
        const float OwnSide=ProjectedExtent(Vehicle,Lateral);
        for(const auto& Candidate:TrafficVehicles)
        {
            const auto* Other=Candidate.Get();
            if(!IsValid(Other) || Other==Vehicle) continue;
            const int32 OtherFleetIndex=Vehicles.IndexOfByKey(Other);
            FVector OtherDirection=FVector::ZeroVector;
            if(Jobs.IsValidIndex(OtherFleetIndex))
            {
                const auto& OtherJob=Jobs[OtherFleetIndex];
                if(OtherJob.Route.IsValidIndex(OtherJob.Waypoint))
                    OtherDirection=Other->PlannedMotionDirection(OtherJob.Route[OtherJob.Waypoint]);
            }
            const bool CrossingOrHeadOn=!OtherDirection.IsNearlyZero() &&
                FVector::DotProduct(TravelDirection,OtherDirection)<.8f;
            if(CrossingOrHeadOn && HasIntersectionPriority(Other)) continue;
            const FVector Delta=Other->GetActorLocation()-Vehicle->GetActorLocation();
            const float Along=FVector::DotProduct(Delta,TravelDirection);
            const float SideClearance=FMath::Abs(FVector::DotProduct(Delta,Lateral))-OwnSide-ProjectedExtent(Other,Lateral);
            if(SideClearance>0) continue;
            const float Clearance=FMath::Max(0.f,FMath::Abs(Along)-OwnLong-ProjectedExtent(Other,TravelDirection));
            if(Along>=0 && (AheadDistance<0 || Clearance<AheadDistance))
            { AheadDistance=Clearance; LiDARBlocker=Other->VehicleID; }
            else if(Along<0 && (BehindDistance<0 || Clearance<BehindDistance)) BehindDistance=Clearance;
        }
    }
    // The AGV can move forward or reverse without turning around.  Associate
    // the leading obstacle with the physical front or rear LiDAR accordingly.
    const bool Reversing=!TravelDirection.IsNearlyZero() &&
        FVector::DotProduct(TravelDirection,Vehicle->GetActorRightVector())<0;
    const float FrontDistance=Reversing?BehindDistance:AheadDistance;
    const float RearDistance=Reversing?AheadDistance:BehindDistance;
    const float LiDARSafetyDistance=AGVReference::DynamicLiDARSafetyDistance(
        Vehicle->Speed,Vehicle->AccelerationCmPerSecondSquared());
    const float LeadingDistance=Reversing?RearDistance:FrontDistance;
    const bool LiDARStop=LeadingDistance>=0 && LeadingDistance<=LiDARSafetyDistance;
    Vehicle->UpdateLiDARObservation(FrontDistance,RearDistance,LiDARSafetyDistance,LiDARStop,
        LiDARStop?LiDARBlocker:INDEX_NONE);
    if(LiDARStop)
    {
        RoadBlockers.Add(ID,LiDARBlocker);
        RoadReservations.Remove(ID); ReservationIndex.Remove(ID); RoadTargets.Remove(ID);
        Vehicle->SetFMSHold(false);
        return false;
    }
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
        FVector Extent=Along*210.f+Across*730.f+FVector(0,0,250);
        const FVector ToTarget=(Target-Vehicle->GetActorLocation()).GetSafeNormal2D();
        const FVector Driven=Vehicle->PlannedMotionDirection(Target);
        if(!ToTarget.IsNearlyZero() && FVector::DotProduct(ToTarget,Driven)<.995f)
        {
            // The steering arc bulges perpendicular to the route chord.  A
            // radius expansion on both axes falsely intersects the next parked
            // row even though the vehicle never travels farther along the chord.
            const FVector SweepNormal(-ToTarget.Y,ToTarget.X,0);
            // A 90-degree tangent arc bows only R*(1-cos45) outside its chord;
            // adding the whole radius made safe roads intersect parked AGVs.
            constexpr float QuarterTurnSagittaFactor=1.f-.70710678f;
            Extent+=SweepNormal.GetAbs()*(AGVReference::MinimumInnerTurnRadiusCm*QuarterTurnSagittaFactor);
        }
        FBox Segment(Vehicle->GetActorLocation()-Extent,Vehicle->GetActorLocation()+Extent);
        Segment+=Target-Extent; Segment+=Target+Extent;
        TArray<FBox> Segments={Segment};
        if (Jobs.IsValidIndex(FleetIndex))
        {
            const auto& Job=Jobs[FleetIndex];
            const int32 RouteEnd=(Job.Stage==3 && !Job.bRMGReserved)?Job.YardEntryWaypoint+1:Job.Route.Num();
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
            const int32 OtherFleetIndex=OtherID-100;
            if(Vehicles.IsValidIndex(OtherFleetIndex) && HasIntersectionPriority(Vehicles[OtherFleetIndex])) continue;
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
            if(HasIntersectionPriority(Other)) continue;
            const FVector E=Other->GetActorForwardVector().GetAbs()*210.f+Other->GetActorRightVector().GetAbs()*730.f+FVector(0,0,250);
            const int32 OtherFleetIndex=Vehicles.IndexOfByKey(Other);
            const bool Parked=Jobs.IsValidIndex(OtherFleetIndex) &&
                (Jobs[OtherFleetIndex].Stage==0 || Jobs[OtherFleetIndex].Stage==8);
            if(Parked)
            {
                const FVector SegmentDirection=(Target-Vehicle->GetActorLocation()).GetSafeNormal2D();
                if(!SegmentDirection.IsNearlyZero())
                {
                    const FVector Lateral(-SegmentDirection.Y,SegmentDirection.X,0);
                    const float OwnSide=FMath::Abs(FVector::DotProduct(Vehicle->GetActorForwardVector(),Lateral))*210.f+
                        FMath::Abs(FVector::DotProduct(Vehicle->GetActorRightVector(),Lateral))*730.f;
                    const float OtherSide=FMath::Abs(FVector::DotProduct(Other->GetActorForwardVector(),Lateral))*210.f+
                        FMath::Abs(FVector::DotProduct(Other->GetActorRightVector(),Lateral))*730.f;
                    const float LateralGap=FMath::Abs(FVector::DotProduct(
                        Other->GetActorLocation()-Vehicle->GetActorLocation(),Lateral));
                    if(LateralGap>OwnSide+OtherSide+100.f) continue;
                }
            }
            // A parked body must clear the segment being entered now. It must
            // not reserve a later look-ahead segment that runs safely beside
            // its bay; that previously froze whole roads tens of metres away.
            const int32 SegmentCount=Parked?FMath::Min(1,Segments.Num()):Segments.Num();
            for (int32 SegmentIndex=0;SegmentIndex<SegmentCount;++SegmentIndex)
                if (Segments[SegmentIndex].Intersect(FBox(Other->GetActorLocation()-E,Other->GetActorLocation()+E)))
                { Blocker=Other->VehicleID; break; }
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
            Vehicle->SetFMSHold(true);
            return false;
        }
        RoadReservations.Add(ID,Segments);
        ReservationIndex.Set(ID,Segments);
        RoadTargets.Add(ID,Target);
        RoadBlockers.Remove(ID);
        RoadWaitSince.Remove(ID);
    }
    Vehicle->SetFMSHold(false);
    const FTransform BeforeMove=Vehicle->GetActorTransform();
    const float BeforeSpeed=Vehicle->Speed;
    const FAGVSensorState BeforeSensors=Vehicle->Sensors;
    bool Arrived=Vehicle->MoveToPosition(Target,Dt,!StraightThrough);
    for(const auto& Other:TrafficVehicles)
    {
        if(!IsValid(Other) || Other==Vehicle ||
            !SiteLogistics::AGVFootprintsOverlap(Vehicle,Other)) continue;
        Vehicle->SetActorTransform(BeforeMove,false,nullptr,ETeleportType::TeleportPhysics);
        Vehicle->Speed=BeforeSpeed;
        Vehicle->Sensors=BeforeSensors;
        Vehicle->SetFMSHold(true);
        RoadBlockers.Add(ID,Other->VehicleID);
        Arrived=false;
        break;
    }
    if(!Arrived && Jobs.IsValidIndex(FleetIndex))
    {
        const auto& Job=Jobs[FleetIndex];
        const bool Docking=Job.Waypoint==Job.Route.Num()-1;
        if(!Docking && Job.Route.IsValidIndex(Job.Waypoint-1))
        {
            // FMS road nodes are gates, not centimetre-sized parking spots. A
            // car-like chassis has passed an intermediate node when its centre
            // crosses the node plane while remaining within one turn radius of
            // the route. Final STS/RMG docking still requires exact position.
            const FVector Incoming=(Target-Job.Route[Job.Waypoint-1]).GetSafeNormal2D();
            if(!Incoming.IsNearlyZero())
            {
                const FVector Delta=Vehicle->GetActorLocation()-Target;
                const FVector Lateral(-Incoming.Y,Incoming.X,0);
                const float Along=FVector::DotProduct(Delta,Incoming);
                const float CrossTrack=FMath::Abs(FVector::DotProduct(Delta,Lateral));
                Arrived=Along>=0.f && CrossTrack<=AGVReference::MinimumInnerTurnRadiusCm;
            }
        }
    }
    const int32 TrafficIndex=TrafficVehicles.IndexOfByKey(Vehicle);
    if (TrafficIndex!=INDEX_NONE) IndexTrafficVehicle(TrafficIndex);
    if (Arrived) FinishedRoadSegments.Add(ID);
    return Arrived;
}

bool APortSiteLogistics::Drive(int32 Lane,float Dt)
{
    auto& Job=Jobs[Lane]; auto* Vehicle=Vehicles[Lane].Get();
    // Never enter an occupied STS handover. The assigned cargo stays in the
    // prepared slot until this empty vehicle arrives and takes ownership.
    if(Job.Stage==5 && Job.ReturnSTS!=INDEX_NONE && Job.Waypoint==Job.Route.Num()-1 &&
        (STSOwners[Job.ReturnSTS]!=INDEX_NONE || !PreparedStarted[Job.ReturnSTS]))
    { Vehicle->Speed=0; return false; }
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
    if(Job.Stage==3 && !Job.bRMGReserved && Job.Waypoint==Job.YardEntryWaypoint+1) return true;
    if (Job.Waypoint<Job.Route.Num()) return false;
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
            if (LaneCount==9 && Job.Waypoint==0)
            {
                // Release one AGV at a time from each vessel's parking row.
                // Once the leader reaches the longitudinal aisle (waypoint 1),
                // the next vehicle can pull out with a full 27 m bay spacing.
                // All three vessel pools can still depart concurrently.
                const int32 Pool=TerminalLayout::AGVBerthGroup(Job.STS);
                const float CrossY=TerminalLayout::AGVInboundCrossY(Job.STS);
                const float MyDistance=FMath::Abs(Vehicle->GetActorLocation().Y-CrossY);
                int32 DepartureLeader=INDEX_NONE;
                float LeaderDistance=MyDistance;
                for(int32 Other=0;Other<Jobs.Num();++Other)
                {
                    if(Other==Lane || Jobs[Other].Stage!=6 || Jobs[Other].Waypoint!=0 ||
                        TerminalLayout::AGVBerthGroup(Jobs[Other].STS)!=Pool) continue;
                    const float OtherDistance=FMath::Abs(Vehicles[Other]->GetActorLocation().Y-CrossY);
                    if(OtherDistance<LeaderDistance-1.f ||
                        (FMath::IsNearlyEqual(OtherDistance,LeaderDistance,1.f) &&
                            Vehicles[Other]->VehicleID<Vehicle->VehicleID))
                    { DepartureLeader=Other; LeaderDistance=OtherDistance; }
                }
                if(DepartureLeader!=INDEX_NONE)
                {
                    Vehicle->Speed=0.f;
                    Vehicle->SetFMSHold(true);
                    RoadBlockers.Add(Vehicle->VehicleID,Vehicles[DepartureLeader]->VehicleID);
                    break;
                }
            }
            if (!Drive(Lane,Dt)) break;
            if (!Vehicle->GetActorLocation().Equals(CargoQuay(Job.Cargo),AGVReference::DockingAccuracyCm) || Vehicle->Speed>.1f)
            { Stop(TEXT("AGV did not stop at its cargo-specific STS handover")); return; }
            Equipment[YardCraneCount+Job.STS]->SetDestinationReady(true);
            Job.Stage=1; break;
        case 1:
            if (Equipment[YardCraneCount+Job.STS]->IsBusy()) break;
            if (!Cargo || !Cargo->GetActorLocation().Equals(Vehicle->CargoPosition(),10.f) || Vehicle->Speed>0)
            { Stop(TEXT("STS / AGV handover alignment")); return; }
            Cargo->GetBody()->SetSimulatePhysics(false);
            Cargo->AttachToComponent(Vehicle->GetRootComponent(),FAttachmentTransformRules::KeepWorldTransform);
            Cargo->SetActorLocationAndRotation(Vehicle->CargoPosition(),Vehicle->GetActorRotation(),false,nullptr,
                ETeleportType::TeleportPhysics);
            Cargo->LocationOwner=ECargoOwner::AGV;
            Vehicle->SetPayload(Cargo->MassKg);
            if(!Vehicle->PayloadWithinReferenceLimit())
            { Stop(TEXT("AGV payload exceeds supplied 65 t reference limit")); return; }
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
            if(!Job.bRMGReserved)
            {
                Job.Stage=9;
                const int32 ID=Vehicle->VehicleID;
                RoadReservations.Remove(ID); ReservationIndex.Remove(ID); RoadTargets.Remove(ID);
                FinishedRoadSegments.Remove(ID);
                break;
            }
            YardApproachOwners[Yard[Job.Slot].Block]=INDEX_NONE;
            // Keep the twist locks engaged until the RMG actually picks up the box.
            if (!Equipment[Job.RMG]->AssignCargo(Cargo,Vehicle->CargoPosition(),Yard[Job.Slot].Position,false,true,Vehicle))
            { Stop(TEXT("AGV / RMG reservation handover")); return; }
            Manifest[Job.Cargo].HandoverMask|=2;
            UE_LOG(LogTemp,Display,TEXT("SITE_HANDOVER: C%d AGV%d -> RMG%d"),Manifest[Job.Cargo].ID,100+Lane,Job.RMG+1);
            Job.Stage=4; break;
        case 4:
            if (Equipment[Job.RMG]->IsBusy()) break;
            // Yard cranes may unload in parallel. Empty returns share the
            // parking return aisle, so admit one return at a time. A return
            // also waits for loaded traffic in its own vessel corridor to
            // clear the crossing before it starts.
            {
                bool ReturnNetworkBusy=false;
                const int32 MyGroup=TerminalLayout::AGVBerthGroup(Job.STS);
                for(int32 Other=0;Other<Jobs.Num();++Other)
                {
                    if(Other==Lane) continue;
                    const int32 OtherStage=Jobs[Other].Stage;
                    if(OtherStage==5 ||
                       (OtherStage==3 && TerminalLayout::AGVBerthGroup(Jobs[Other].STS)==MyGroup))
                    { ReturnNetworkBusy=true; break; }
                }
                if(ReturnNetworkBusy) break;
            }
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
            AssignReturnSTS(Lane);
            if(!Fault.IsEmpty()) return;
            PrepareRoute(Lane,true); Job.Stage=5; break;
        case 5:
            if (!Drive(Lane,Dt)) break;
            if (!Job.bYardReleased) RMGBusy[Job.RMG]=false;
            if(Job.ReturnSTS!=INDEX_NONE)
            {
                const int32 STS=Job.ReturnSTS;
                if(NextVehicles[STS]!=Lane || PreparedCargo[STS]!=Job.ReturnCargo)
                { Stop(TEXT("Direct return lost assigned STS cargo")); return; }
                ActivateVehicle(Lane,STS,true);
                if(!Fault.IsEmpty()) return;
            }
            else Job=FSiteTransfer();
            break;
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
    RoadReservations.Reset(); RoadTargets.Reset(); FinishedRoadSegments.Reset(); PeakMovingVehicles=PrefetchedJobs=Dispatched=Delivered=0;
    for(int32& Count:ReturnRouteCounts) Count=0;
    Fault.Empty(); bWasPaused=false;
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
            { Error=FString::Printf(TEXT("AGV %d escaped yard/road bounds at %s extent=%s yaw=%.1f"),
                Vehicle->VehicleID,*P.ToCompactString(),*E.ToCompactString(),Vehicle->GetActorRotation().Yaw); return false; }
            if(!Vehicle->PayloadWithinReferenceLimit())
            { Error=FString::Printf(TEXT("AGV %d exceeded 65 t payload limit"),Vehicle->VehicleID); return false; }
            if(Vehicle->Sensors.PositionErrorCm>AGVReference::PositionAccuracyCm+.01f)
            { Error=FString::Printf(TEXT("AGV %d localization exceeded transponder accuracy"),Vehicle->VehicleID); return false; }
            if(Vehicle->Sensors.bControlledStop && Vehicle->Speed>.1f)
            { Error=FString::Printf(TEXT("AGV %d moved through a LiDAR controlled stop"),Vehicle->VehicleID); return false; }
            if(Vehicle->Sensors.LateralSlipCm>.001f)
            { Error=FString::Printf(TEXT("AGV %d used forbidden lateral motion"),Vehicle->VehicleID); return false; }
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
        if(Job.ReturnSTS!=INDEX_NONE &&
            (Job.Stage!=5 || NextVehicles[Job.ReturnSTS]!=Lane || PreparedCargo[Job.ReturnSTS]!=Job.ReturnCargo ||
                Job.Actor.IsValid() || Job.Route.IsEmpty() || !Job.Route.Last().Equals(CargoQuay(Job.ReturnCargo),.1f)))
        { Error=TEXT("Invalid direct STS return assignment"); return false; }
        if (JobCargo.Contains(Job.Cargo)) { Error=TEXT("Duplicate job cargo"); return false; }
        JobCargo.Add(Job.Cargo);
        if (!Manifest.IsValidIndex(Job.Cargo)) { Error=TEXT("Missing job manifest entry"); return false; }
        const FVector Handover=CargoQuay(Job.Cargo);
        if (Job.Stage==6 && (Job.Route.IsEmpty() || !Job.Route.Last().Equals(Handover,.1f)))
        { Error=TEXT("AGV approach does not end at the cargo-specific handover"); return false; }
        if ((Job.Stage==1 || Job.Stage==2) &&
            (!Vehicles[Lane]->GetActorLocation().Equals(Handover,AGVReference::DockingAccuracyCm) || Vehicles[Lane]->Speed>.1f))
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
        if(Job.Stage==9 && (Vehicles[Lane]->Speed>.1f || Job.Waypoint!=Job.YardEntryWaypoint+1 ||
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
            if (SiteLogistics::AGVFootprintsOverlap(A,B))
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
