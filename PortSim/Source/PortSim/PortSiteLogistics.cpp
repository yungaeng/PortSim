#include "PortSiteLogistics.h"
#include "TerminalLayout.h"
#include "YardReservation.h"
#include "PortWorkingCrane.h"
#include "PortAGVActor.h"
#include "PortContainerActor.h"
#include "ContainerSpecification.h"
#include "AGVDispatchPolicy.h"
#include "AGVReference.h"
#include "AGVTrafficGeometry.h"
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

    void AddUniqueRoutePoint(TArray<FVector>& Route,const FVector& Point)
    {
        if(Route.IsEmpty() || !Route.Last().Equals(Point,.1f)) Route.Add(Point);
    }

    // Convert the axis-aligned FMS graph into tangent arcs.  The controller
    // follows these points with steering and longitudinal motion; it never
    // receives a diagonal shortcut across the inside of a yard block.
    TArray<FVector> RoundRoadCorners(const FVector& Start,const TArray<FVector>& Nodes)
    {
        TArray<FVector> Polyline; Polyline.Add(Start);
        for(const FVector& Node:Nodes) AddUniqueRoutePoint(Polyline,Node);
        for(int32 I=1;I+1<Polyline.Num();)
        {
            const FVector A=(Polyline[I]-Polyline[I-1]).GetSafeNormal2D();
            const FVector B=(Polyline[I+1]-Polyline[I]).GetSafeNormal2D();
            if(!A.IsNearlyZero() && !B.IsNearlyZero() && FVector::DotProduct(A,B)>.999f)
                Polyline.RemoveAt(I);
            else ++I;
        }
        if(Polyline.Num()<3) return Nodes;
        TArray<FVector> Rounded;
        for(int32 I=1;I<Polyline.Num()-1;++I)
        {
            const FVector A=Polyline[I-1],B=Polyline[I],C=Polyline[I+1];
            if(Polyline.IsValidIndex(I+2))
            {
                TArray<FVector> LaneChange;
                if(AGVTrafficGeometry::LaneChange(Rounded.IsEmpty()?A:Rounded.Last(),B,C,
                    Polyline[I+2],LaneChange))
                {
                    for(const FVector& Point:LaneChange) AddUniqueRoutePoint(Rounded,Point);
                    ++I;
                    continue;
                }
            }
            const FVector In=(B-A).GetSafeNormal2D(),Out=(C-B).GetSafeNormal2D();
            const float Dot=FVector::DotProduct(In,Out);
            const float Cross=In.X*Out.Y-In.Y*Out.X;
            const float Radius=AGVReference::MinimumInnerTurnRadiusCm;
            if(In.IsNearlyZero() || Out.IsNearlyZero() || FMath::Abs(Cross)<.01f || Dot<-.01f ||
                FVector::Dist2D(A,B)<Radius*1.1f || FVector::Dist2D(B,C)<Radius*1.1f)
            { AddUniqueRoutePoint(Rounded,B); continue; }
            const float Angle=FMath::Acos(FMath::Clamp(Dot,-1.f,1.f));
            const float Tangent=Radius*FMath::Tan(Angle*.5f);
            if(FVector::Dist2D(A,B)<=Tangent+100.f || FVector::Dist2D(B,C)<=Tangent+100.f)
            { AddUniqueRoutePoint(Rounded,B); continue; }
            const FVector InPoint=B-In*Tangent,OutPoint=B+Out*Tangent;
            const FVector Left(-In.Y,In.X,0);
            const FVector Centre=InPoint+Left*(Cross>0.f?Radius:-Radius);
            AddUniqueRoutePoint(Rounded,InPoint);
            const float StartAngle=FMath::Atan2(InPoint.Y-Centre.Y,InPoint.X-Centre.X);
            float EndAngle=FMath::Atan2(OutPoint.Y-Centre.Y,OutPoint.X-Centre.X);
            if(Cross>0.f) while(EndAngle<StartAngle) EndAngle+=2.f*PI;
            else while(EndAngle>StartAngle) EndAngle-=2.f*PI;
            const int32 Steps=FMath::Max(2,FMath::CeilToInt(FMath::Abs(EndAngle-StartAngle)/FMath::DegreesToRadians(22.5f)));
            for(int32 Step=1;Step<=Steps;++Step)
            {
                const float T=float(Step)/Steps,Theta=FMath::Lerp(StartAngle,EndAngle,T);
                AddUniqueRoutePoint(Rounded,Centre+FVector(FMath::Cos(Theta),FMath::Sin(Theta),0)*Radius);
            }
        }
        AddUniqueRoutePoint(Rounded,Polyline.Last());
        // Retain regular traffic gates on straight roads. A single 500 m
        // segment makes an entire spine exclusive to one vehicle.
        TArray<FVector> Gated;
        FVector From=Start;
        for(const FVector& To:Rounded)
        {
            const int32 Steps=FMath::Max(1,FMath::CeilToInt(FVector::Dist2D(From,To)/3000.f));
            for(int32 Step=1;Step<=Steps;++Step)
                AddUniqueRoutePoint(Gated,FMath::Lerp(From,To,double(Step)/Steps));
            From=To;
        }
        return Gated;
    }

    bool RouteIntersectsContainer(const TArray<FVector>& Route,const FVector& Start,
        const TArray<FSiteYardSlot>& Yard,int32 IgnoredSlot=INDEX_NONE)
    {
        FVector A=Start;
        for(const FVector& B:Route)
        {
            const FVector Direction=(B-A).GetSafeNormal2D();
            const FVector Normal(-Direction.Y,Direction.X,0);
            const FVector Extent=Direction.GetAbs()*AGVReference::PhysicalHalfLengthCm+
                Normal.GetAbs()*AGVReference::PhysicalHalfWidthCm+FVector(25,25,200);
            for(int32 SlotIndex=0;SlotIndex<Yard.Num();++SlotIndex)
            {
                const FSiteYardSlot& Slot=Yard[SlotIndex];
                if(Yard.IsValidIndex(IgnoredSlot) &&
                    FVector::DistSquared2D(Slot.Position,Yard[IgnoredSlot].Position)<1.f) continue;
                if(!Slot.Occupied) continue;
                const FVector ContainerExtent(690,130,260);
                const FBox ExpandedObstacle(Slot.Position-ContainerExtent-Extent,
                    Slot.Position+ContainerExtent+Extent);
                if(ExpandedObstacle.IsInsideOrOn(A) || ExpandedObstacle.IsInsideOrOn(B) ||
                    FMath::LineBoxIntersection(ExpandedObstacle,A,B,B-A))
                    return true;
            }
            A=B;
        }
        return false;
    }
}

APortSiteLogistics::APortSiteLogistics()
{
    PrimaryActorTick.bCanEverTick=false;
    // Each STS owns a painted handover bay with enough longitudinal clearance
    // for a 14.6 m AGV. Using the ship container's raw Y coordinate can place
    // adjacent STS handovers less than one vehicle length apart.
    bCargoAlignedHandover=FParse::Param(FCommandLine::Get(),TEXT("PortSimCargoAlignedHandover"));
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
{ return FVector(TerminalLayout::SiteCmX(TerminalLayout::AGVQuayHandoverX),((LaneCount==9?TerminalLayout::STSCenterY(Lane):SiteLogistics::LegacyCraneY[Lane])+8)*100,0); }
FVector APortSiteLogistics::CargoQuay(int32 CargoIndex) const
{
    // Fix the destination for the whole job using the secured manifest position,
    // never the swinging cargo or the crane's instantaneous position.
    const auto& Cargo=Manifest[CargoIndex];
    return bCargoAlignedHandover?FVector(QuayPark(Cargo.STS).X,Cargo.Transform.GetLocation().Y,0):QuayPark(Cargo.STS);
}
FVector APortSiteLogistics::QueueQuay(int32 STS) const
{
    // This is the painted 4.2 x 15 m AGV APPROACH bay beside each STS.  It is
    // independent of cargo order, so every STS can keep one empty AGV waiting.
    return FVector(TerminalLayout::SiteCmX(TerminalLayout::AGVQuayWaitingBayX),(TerminalLayout::STSCenterY(STS)+28.f)*100.f,0);
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
    Jobs.SetNum(LaneCount==9?60:LaneCount); YardApproachOwners.Init(INDEX_NONE,LaneCount==9?YardCraneCount:TerminalLayout::YardBlockCount); BlocksBusy.Init(false,TerminalLayout::YardBlockCount); RMGBusy.Init(false,YardCraneCount);
    STSOwners.Init(INDEX_NONE,LaneCount);
    NextVehicles.Init(INDEX_NONE,LaneCount); PreparedStarted.Init(false,LaneCount);
    PreparedCargo.Init(INDEX_NONE,LaneCount); SlotAssigned.Init(false,Yard.Num());
    FActorSpawnParameters Params; Params.Owner=this;
    Params.SpawnCollisionHandlingOverride=ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
    for (int32 I=0;I<Jobs.Num();++I)
    {
        auto* Vehicle=GetWorld()->SpawnActor<APortAGVActor>(FleetPark(I),FRotator::ZeroRotator,Params);
        Vehicle->InitializeVehicle(100+I);
        // A freshly spawned fleet uses the same marked-bay pose as reset.  The
        // controller steers through the bay gap instead of rotating or sliding.
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
    // Acquire the independent RMG approach before occupying the STS through
    // lane. Waiting here after pickup can block the vehicle that must deliver
    // this container's support or release this same approach.
    if(!ReserveYardApproach(Vehicle)) { Stop(TEXT("STS activation lost yard approach admission")); return; }
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
        // A reserved standby remains physical traffic even after it reaches
        // the STS bay. Never turn it into an idle vehicle on the road.
        if(Queued!=INDEX_NONE) continue;
        const double Score=FVector::Dist2D(Vehicles[Vehicle]->GetActorLocation(),CargoQuay(PreparedCargo[S]))+
            (STSOwners[S]!=INDEX_NONE?15000.:0.);
        if(Score<BestScore) { BestScore=Score; Best=S; }
    }
    if(Best==INDEX_NONE) return;
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
                const int32 Bay=I%TerminalLayout::AGVParkBays;
                const int32 Gap=Bay<TerminalLayout::AGVParkBays-1?Bay:Bay-1;
                if(Vehicles[I]->GetActorLocation().Equals(FleetPark(I),AGVReference::DockingAccuracyCm+1.f) &&
                    FMath::Abs(TerminalLayout::AGVParkCrossingY(Gap)-Target.Y)<
                        AGVReference::MinimumInnerTurnRadiusCm+100.f) continue;
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
    // Prepare every STS first.  Assignment is per STS rather than per vessel:
    // any idle AGV may serve any free handover, so the other two STSs on a ship
    // no longer wait for one group-owned vehicle to finish a full yard cycle.
    for (int32 S=0;S<LaneCount;++S)
    {
        PrepareNextCargo(S);
        if(!Fault.IsEmpty()) return;
    }
    TArray<int32> ReadySTS;
    for(int32 S=0;S<LaneCount;++S) ReadySTS.Add(S);
    ReadySTS.StableSort([&](int32 A,int32 B)
    {
        const int32 CargoA=PreparedCargo[A],CargoB=PreparedCargo[B];
        const bool ReadyA=STSOwners[A]==INDEX_NONE && CargoA!=INDEX_NONE && PreparedStarted[A];
        const bool ReadyB=STSOwners[B]==INDEX_NONE && CargoB!=INDEX_NONE && PreparedStarted[B];
        if(ReadyA!=ReadyB) return ReadyA;
        if(ReadyA && Manifest[CargoA].StartedAt!=Manifest[CargoB].StartedAt)
            return Manifest[CargoA].StartedAt<Manifest[CargoB].StartedAt;
        // Interleave south/central/north on equal-age work so all three main
        // corridors receive traffic instead of filling one vessel end first.
        const int32 InterleaveA=(A%3)*3+A/3,InterleaveB=(B%3)*3+B/3;
        return InterleaveA<InterleaveB;
    });
    for (int32 S:ReadySTS)
    {
        if (STSOwners[S]!=INDEX_NONE || PreparedCargo[S]==INDEX_NONE || !PreparedStarted[S] ||
            !YardApproachAvailable(PreparedCargo[S])) continue;
        if (NextVehicles[S]!=INDEX_NONE)
        {
            if (Jobs[NextVehicles[S]].Stage==8) ActivateVehicle(NextVehicles[S],S,true);
        }
        else if (const int32 V=Nearest(CargoQuay(PreparedCargo[S]),S); V!=INDEX_NONE) ActivateVehicle(V,S,false);
    }
    // Do not speculate a second vehicle into the road network while this STS
    // still owns an active AGV.  Every ready, unowned STS above is assigned an
    // idle vehicle immediately; an extra standby only occupies a parking-road
    // junction and can form a wait cycle with vehicles serving adjacent STSs.
    // Direct RMG-to-STS returns remain preassigned because they replace an
    // otherwise empty return trip rather than adding another departure.
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

bool APortSiteLogistics::YardApproachAvailable(int32 Cargo) const
{
    if(!Manifest.IsValidIndex(Cargo)) return false;
    const int32 Planned=Manifest[Cargo].PlannedSlot;
    if(!Yard.IsValidIndex(Planned)) return false;
    const auto& Slot=Yard[Planned];
    const bool Available=Slot.Reserved && !Slot.Central && !Slot.Occupied && !SlotAssigned[Planned] &&
        YardApproachOwners[YardApproachKey(Slot)]==INDEX_NONE && !BlocksBusy[Slot.Block] &&
        (Slot.Below==INDEX_NONE || Yard[Slot.Below].Occupied) &&
        (CentralPending==INDEX_NONE || Yard[CentralSlots[CentralPending]].Block!=Slot.Block);
    if(!Available) return false;
    // Adjacent handover aisles can share the turning envelope even though
    // their RMGs are independent. Claim that space before leaving the STS,
    // rather than admitting one vehicle into another's docking/exit turn.
    const FVector Dock=YardHandover(Slot);
    for(const auto& Other:Jobs)
    {
        if(!Yard.IsValidIndex(Other.Slot) || Other.bYardReleased) continue;
        const auto& OtherSlot=Yard[Other.Slot];
        const bool Approaching=YardApproachOwners[YardApproachKey(OtherSlot)]!=INDEX_NONE;
        if((Approaching || Other.bRMGReserved) &&
            FVector::Dist2D(Dock,YardHandover(OtherSlot))<3000.f) return false;
    }
    return true;
}

bool APortSiteLogistics::ReserveYardApproach(int32 Lane)
{
    auto& Job=Jobs[Lane];
    const int32 Planned=Manifest[Job.Cargo].PlannedSlot;
    if(!Yard.IsValidIndex(Planned)) { Stop(TEXT("Manifest has no valid planned yard slot")); return false; }
    const auto& Slot=Yard[Planned];
    if(Job.Slot==Planned && YardApproachOwners[YardApproachKey(Slot)]==Lane) return true;
    // Separate near/far RMG entry lanes; never queue an upper box before its support.
    if(!YardApproachAvailable(Job.Cargo)) return false;
    Job.Slot=Planned; Job.RMG=Slot.Crane;
    SlotAssigned[Planned]=true; YardApproachOwners[YardApproachKey(Slot)]=Lane;
    return true;
}

bool APortSiteLogistics::ReserveYard(int32 Lane)
{
    auto& Job=Jobs[Lane];
    const int32 Planned=Manifest[Job.Cargo].PlannedSlot;
    if(!Yard.IsValidIndex(Planned)) { Stop(TEXT("Manifest has no valid planned yard slot")); return false; }
    const auto& Slot=Yard[Planned]; const int32 RMG=Slot.Crane;
    if (!Slot.Reserved || Slot.Central || Slot.Occupied || Job.Slot!=Planned || YardApproachOwners[YardApproachKey(Slot)]!=Lane || BlocksBusy[Slot.Block] ||
        RMGBusy[RMG] || Equipment[RMG]->IsBusy() || !Equipment[RMG]->Fault.IsEmpty() ||
        (CentralPending!=INDEX_NONE && Yard[CentralSlots[CentralPending]].Block==Slot.Block && YardApproachOwners[YardApproachKey(Slot)]!=Lane)) return false;
    const FVector Dock=YardHandover(Slot);
    for(int32 Other=0;Other<Jobs.Num();++Other)
    {
        if(Other==Lane || !Jobs[Other].bRMGReserved || Jobs[Other].bYardReleased ||
            !Yard.IsValidIndex(Jobs[Other].Slot)) continue;
        if(FVector::Dist2D(Dock,YardHandover(Yard[Jobs[Other].Slot]))<3000.f)
            return false;
    }
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
    if(Job.Stage!=3 && Job.Stage!=5 && Job.Stage!=6 && Job.Stage!=7) return false;
    if((Job.Stage==3 || Job.Stage==5) && !Yard.IsValidIndex(Job.Slot)) return false;
    FAGVRoadNetwork Network;
    auto P=[](float X,float Y) { return FVector(TerminalLayout::SiteCmX(X),Y,0); };
    const float QuayLane=Job.Stage==3?TerminalLayout::AGVQuayOutboundLaneX:
        TerminalLayout::AGVQuayInboundLaneX;
    const float South=TerminalLayout::AGVSouthReturnY,North=TerminalLayout::AGVNorthInboundY;

    // Quay-side one-way spines and the two lanes of the inland service road.
    for(float X:{12500.f,15300.f}) Network.Add(P(X,South),P(X,North));
    for(float X:{14000.f,16700.f}) Network.Add(P(X,North),P(X,South));
    Network.Add(P(TerminalLayout::AGVInlandInboundX,South),
        P(TerminalLayout::AGVInlandInboundX,North));
    Network.Add(P(TerminalLayout::AGVInlandReturnX,North),
        P(TerminalLayout::AGVInlandReturnX,South));
    // Loaded AGVs from the three STSs around one vessel converge on that
    // vessel's nearest cross-road from either side. Segment admission below
    // serializes the short shared portion before they enter it.
    Network.Add(P(QuayLane,North),P(QuayLane,South),Job.Stage==3);
    // Parking rows at X=80/100/112 m join a crossroad longitudinally.  Do not
    // create extra turn-offs at X=72/90 m: the two opposite 90-degree turns
    // would be closer than the AGV's 5.8 m minimum-radius tangent arcs.

    // South, central and north roads are real alternatives in the graph.  The
    // shortest route (plus live reservation cost during rerouting) selects one.
    Network.Add(P(QuayLane,TerminalLayout::AGVSouthInboundY),
        P(TerminalLayout::AGVInlandInboundX,TerminalLayout::AGVSouthInboundY));
    Network.Add(P(TerminalLayout::AGVInlandReturnX,TerminalLayout::AGVSouthReturnY),
        P(QuayLane,TerminalLayout::AGVSouthReturnY));
    // The central road crosses the three standby rows.  Join it east of the
    // marked bays and use the two clear gaps around the road for access; a
    // straight x=35..630 m edge would physically run through parked AGVs.
    Network.Add(P(15300,TerminalLayout::AGVCentralInboundY),
        P(TerminalLayout::AGVInlandInboundX,TerminalLayout::AGVCentralInboundY));
    Network.Add(P(TerminalLayout::AGVInlandReturnX,TerminalLayout::AGVCentralReturnY),
        P(16700,TerminalLayout::AGVCentralReturnY));
    constexpr int32 CentralInboundGap=8;
    constexpr int32 CentralReturnGap=9;
    Network.Add(P(QuayLane,TerminalLayout::AGVParkCrossingY(CentralInboundGap)),
        P(15300,TerminalLayout::AGVParkCrossingY(CentralInboundGap)));
    Network.Add(P(16700,TerminalLayout::AGVParkCrossingY(CentralReturnGap)),
        P(QuayLane,TerminalLayout::AGVParkCrossingY(CentralReturnGap)));
    Network.Add(P(QuayLane,TerminalLayout::AGVNorthInboundY),
        P(TerminalLayout::AGVInlandReturnX,TerminalLayout::AGVNorthInboundY));
    Network.Add(P(TerminalLayout::AGVInlandReturnX,TerminalLayout::AGVNorthReturnY),
        P(QuayLane,TerminalLayout::AGVNorthReturnY));

    const FVector Park=FleetPark(Lane);
    const int32 Bay=Lane%TerminalLayout::AGVParkBays;
    const int32 DepartureGap=Bay<TerminalLayout::AGVParkBays-1?Bay:Bay-1;
    const int32 ReturnGap=Bay>0?Bay-1:0;
    auto AddParkingCrossing=[&](int32 Gap)
    {
        const float Y=TerminalLayout::AGVParkCrossingY(Gap);
        if(TerminalLayout::AGVParkCrossingToYard(Gap)) Network.Add(P(QuayLane,Y),P(16700,Y));
        else Network.Add(P(16700,Y),P(QuayLane,Y));
    };
    // The active vehicle uses only the two gaps associated with its own bay.
    // The 19 parking crossings are access aisles, not general through-roads.
    if(Job.Stage==6 || Job.Stage==7) AddParkingCrossing(DepartureGap);
    if(Job.Stage==5 && Job.ReturnSTS==INDEX_NONE) AddParkingCrossing(ReturnGap);

    const int32 QueueSTS=Job.Stage==5 && Job.ReturnSTS!=INDEX_NONE?Job.ReturnSTS:Job.STS;
    // Northward empty traffic uses the existing northbound service spine;
    // westbound parking gaps feed a single southbound quay entry lane. The
    // former x=50/55 m pair had insufficient chassis clearance for two flows.
    auto WestAccess=[&](float Y)
    {
        Network.Add(P(16700,Y),P(QuayLane,Y));
    };
    auto ClearParkingGap=[&](float BayY,bool Above)->float
    {
        float Best=Above?TerminalLayout::AGVParkY(TerminalLayout::AGVParkBays-1)+3000.f:
            TerminalLayout::AGVSouthReturnY;
        double Distance=TNumericLimits<double>::Max();
        for(int32 Gap=1;Gap<TerminalLayout::AGVParkBays-1;Gap+=2)
        {
            const float Y=TerminalLayout::AGVParkCrossingY(Gap);
            if((Above?Y-BayY:BayY-Y)<2*AGVReference::MinimumInnerTurnRadiusCm+100.f) continue;
            const double Candidate=FMath::Abs(Y-BayY);
            if(Candidate<Distance) { Best=Y; Distance=Candidate; }
        }
        return Best;
    };
    FVector Quay=QueueQuay(QueueSTS);
    const bool NeedsQueueRoad=Job.Stage==7 || (Job.Stage==5 && Job.ReturnSTS!=INDEX_NONE) ||
        (Job.Stage==6 && !Vehicle->GetActorLocation().Equals(Park,AGVReference::DockingAccuracyCm+1.f));
    if(NeedsQueueRoad)
    {
        const float ApproachY=ClearParkingGap(Quay.Y,true);
        const FVector QueueApproach(Quay.X,ApproachY,0);
        WestAccess(ApproachY);
        Network.Add(P(QuayLane,ApproachY),QueueApproach);
        Network.Add(QueueApproach,Quay,true);
    }
    if(Job.Stage!=7)
    {
        const int32 DestinationCargo=Job.Stage==5 && Job.ReturnCargo!=INDEX_NONE?Job.ReturnCargo:Job.Cargo;
        if(!Manifest.IsValidIndex(DestinationCargo)) return false;
        Quay=CargoQuay(DestinationCargo);
        if(Job.Stage==6)
        {
            // A vehicle on the southbound quay spine changes lane with two S
            // arcs. A fresh parked vehicle may instead turn directly from a
            // westbound crossing into the STS handover lane.
            const FVector Approach(Quay.X,Quay.Y+TerminalLayout::AGVBerthApproachOffset,0);
            Network.Add(P(QuayLane,Approach.Y),Approach);
            Network.Add(Approach,Quay,true);
            if(Vehicle->GetActorLocation().Equals(Park,AGVReference::DockingAccuracyCm+1.f))
                for(bool Above:{false,true})
                {
                    const float Y=ClearParkingGap(Quay.Y,Above);
                    WestAccess(Y);
                    Network.Add(FVector(Quay.X,Y,0),Quay,true);
                }
        }
    }
    const FVector ParkingDeparture(Park.X,TerminalLayout::AGVParkCrossingY(DepartureGap),0);
    const FVector ParkingReturn(Park.X,TerminalLayout::AGVParkCrossingY(ReturnGap),0);
    if(Job.Stage==5 && Job.ReturnSTS==INDEX_NONE && TerminalLayout::AGVParkCrossingToYard(ReturnGap))
        WestAccess(ClearParkingGap(ParkingReturn.Y,true));
    // Pull longitudinally into the empty space between two marked bays before
    // turning onto a crossroad.  A bay-centre shortcut would sweep through the
    // idle AGV in the adjacent row.
    Network.Add(Park,ParkingDeparture);
    Network.Add(ParkingReturn,Park);

    FVector Goal=Quay;
    FVector Dock=FVector::ZeroVector;
    if(Job.Stage==3 || Job.Stage==5)
    {
        const auto& Slot=Yard[Job.Slot]; Dock=YardHandover(Slot);
        const float BlockY=TerminalLayout::BlockY(Slot.Block)*100.f;
        const bool Far=Slot.Half!=0;
        // The AGV lane is at +/-13.2 m, outside the container rows at +/-9 m.
        // Near RMGs enter from the quay spine; far RMGs enter from the inland road.
        const FVector Entry(Dock.X+(Far?1800.f:-1800.f),Dock.Y,0);
        const FVector Exit(Dock.X,BlockY-1320.f,0);
        if(Far)
        {
            Network.Add(P(TerminalLayout::AGVInlandInboundX,Entry.Y),Entry);
            Network.Add(P(TerminalLayout::AGVInlandReturnX,Entry.Y),Entry);
            Network.Add(Dock,Entry);
            Network.Add(Entry,P(TerminalLayout::AGVInlandReturnX,Entry.Y));
            Network.Add(Exit,P(TerminalLayout::AGVInlandInboundX,Exit.Y));
            Network.Add(Exit,P(TerminalLayout::AGVInlandReturnX,Exit.Y));
            if(Job.Stage==5)
            {
                const int32 Group=Job.ReturnSTS!=INDEX_NONE?
                    TerminalLayout::AGVBerthGroup(Job.ReturnSTS):TerminalLayout::AGVParkRow(Lane);
                const float CrossY=TerminalLayout::AGVReturnCrossY(FMath::Clamp(Group,0,2));
                Network.Add(Entry,P(Entry.Y<CrossY?TerminalLayout::AGVInlandInboundX:
                    TerminalLayout::AGVInlandReturnX,Entry.Y));
            }
        }
        else
        {
            Network.Add(P(12500,Entry.Y),Entry);
            Network.Add(Dock,Entry);
            Network.Add(Entry,P(14000,Entry.Y));
            Network.Add(Exit,P(15300,Exit.Y));
            Network.Add(P(16700,Entry.Y),Entry);
            Network.Add(Exit,P(14000,Exit.Y));
            Network.Add(Exit,P(16700,Exit.Y));
            if(Job.Stage==5)
            {
                const int32 Group=Job.ReturnSTS!=INDEX_NONE?
                    TerminalLayout::AGVBerthGroup(Job.ReturnSTS):TerminalLayout::AGVParkRow(Lane);
                const float CrossY=Group==1?TerminalLayout::AGVParkCrossingY(CentralReturnGap):
                    TerminalLayout::AGVReturnCrossY(FMath::Clamp(Group,0,2));
                // Exit onto the spine travelling toward the selected crossroad.
                // Sending every near-yard exit south first forced north returns
                // through two overlapping 5.8 m turns only 7 m apart.
                Network.Add(Entry,P(Entry.Y<CrossY?15300.f:14000.f,Entry.Y));
            }
        }
        Goal=Job.Stage==3?Entry:(Job.ReturnSTS!=INDEX_NONE?QueueQuay(Job.ReturnSTS):Park);
    }

    const FVector Start=Vehicle->GetActorLocation();
    auto ConnectDockingTolerance=[&](const FVector& Anchor)
    {
        if(Start.Equals(Anchor,.1f) || FVector::Dist2D(Start,Anchor)>AGVReference::DockingAccuracyCm+1.f) return;
        const FVector Elbow(Anchor.X,Start.Y,0);
        Network.Add(Start,Elbow,true);
        Network.Add(Elbow,Anchor,true);
    };
    // A successful sensor docking is allowed +/-50 mm.  Attach that measured
    // pose back to the exact road anchor instead of treating it as off-road.
    ConnectDockingTolerance(Quay);
    ConnectDockingTolerance(Park);
    ConnectDockingTolerance(QueueQuay(QueueSTS));
    if(Job.Stage==3 || Job.Stage==5) ConnectDockingTolerance(Dock);

    const int32 ID=Vehicle->VehicleID;
    auto Cost=[&](FVector A,FVector B)
    {
        if(!Congested) return 0.;
        const FVector Direction=(B-A).GetSafeNormal2D();
        const FVector Normal(-Direction.Y,Direction.X,0);
        const FVector Extent=Direction.GetAbs()*AGVReference::PhysicalHalfLengthCm+
            Normal.GetAbs()*AGVReference::PhysicalHalfWidthCm+FVector(0,0,250);
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
            if(AGVTrafficGeometry::SweptChassisIntersects(A,B,Other->GetActorLocation(),Other->GetActorRightVector()))
                Penalty+=Other->Speed<1?12000:3000;
        }
        return Penalty;
    };
    TArray<FVector> Candidate;
    int32 SelectedReturnCorridor=INDEX_NONE;
    bool RouteFound=false;
    if(Job.Stage==5)
    {
        SelectedReturnCorridor=Job.ReturnSTS!=INDEX_NONE?
            TerminalLayout::AGVBerthGroup(Job.ReturnSTS):TerminalLayout::AGVParkRow(Lane);
        SelectedReturnCorridor=FMath::Clamp(SelectedReturnCorridor,0,
            TerminalLayout::AGVReturnCorridorCount-1);
        const bool Far=Yard[Job.Slot].Half!=0;
        // The inland central road is -164.5 m; its parking access jog is -146.5 m.
        // Only the access aisle west of the yards uses the latter coordinate.
        const float CrossY=!Far && SelectedReturnCorridor==1?
            TerminalLayout::AGVParkCrossingY(CentralReturnGap):
            TerminalLayout::AGVReturnCrossY(SelectedReturnCorridor);
        const bool ClearNorthTurn=CrossY-Start.Y>2*AGVReference::MinimumInnerTurnRadiusCm+100.f;
        const float ExitSpine=Far?(Start.Y<CrossY && ClearNorthTurn?TerminalLayout::AGVInlandInboundX:
            TerminalLayout::AGVInlandReturnX):
            (Start.Y<CrossY?15300.f:14000.f);
        const FVector Via=P(ExitSpine,CrossY);
        TArray<FVector> ToCorridor,FromCorridor;
        RouteFound=Network.Find(Start,Via,ToCorridor,Cost) &&
            Network.Find(Via,Goal,FromCorridor,Cost);
        if(RouteFound)
        {
            Candidate=MoveTemp(ToCorridor);
            Candidate.Append(FromCorridor);
        }
    }
    else if(Job.Stage==3)
    {
        // Loaded traffic leaves each vessel on its nearest south, central or
        // north corridor.  An unrestricted shortest-path search sent every
        // vessel down the south quay spine, creating a single FIFO queue and
        // leaving the other two paved roads unused.
        const bool Far=Yard[Job.Slot].Half!=0;
        const float CrossY=Far && TerminalLayout::AGVBerthGroup(Job.STS)==1?
            TerminalLayout::AGVCentralInboundY:TerminalLayout::AGVLoadedCrossY(Job.STS);
        const bool Northbound=Goal.Y>=CrossY;
        const float ArrivalSpine=Far?(Northbound?TerminalLayout::AGVInlandInboundX:
            TerminalLayout::AGVInlandReturnX):(Northbound?12500.f:14000.f);
        // Require a real crossing into the yard-side service spine. Merely
        // touching the quay end let the second search turn straight back along
        // the same road, producing an impossible U-turn during a reroute.
        const FVector Via=P(ArrivalSpine,CrossY);
        TArray<FVector> ToCorridor,FromCorridor;
        RouteFound=Network.Find(Start,Via,ToCorridor,Cost) &&
            Network.Find(Via,Goal,FromCorridor,Cost);
        if(RouteFound)
        {
            Candidate=MoveTemp(ToCorridor);
            if(!Candidate.IsEmpty() && !FromCorridor.IsEmpty() &&
                Candidate.Last().Equals(FromCorridor[0],.1f))
                FromCorridor.RemoveAt(0);
            Candidate.Append(FromCorridor);
        }
    }
    else RouteFound=Network.Find(Start,Goal,Candidate,Cost);
    if(!RouteFound)
    {
        if(!Congested)
        {
            int32 StartRoads=0,GoalRoads=0;
            for(const auto& Road:Network.Roads)
            { StartRoads+=FAGVRoadNetwork::Contains(Road,Start); GoalRoads+=FAGVRoadNetwork::Contains(Road,Goal); }
            UE_LOG(LogTemp,Error,TEXT("AGV_ROUTE_NOT_FOUND: V%d stage=%d sts=%d slot=%d start=%s roads=%d goal=%s roads=%d"),
                ID,Job.Stage,Job.STS,Job.Slot,*Start.ToCompactString(),StartRoads,*Goal.ToCompactString(),GoalRoads);
        }
        return false;
    }
    // Preserve the active route boundary before replacing the stage-3 yard
    // entry index with the candidate route's index below.  A congested reroute
    // can contain a different number of rounded corner nodes.
    const int32 ExistingRouteEnd=Job.Stage==3?
        FMath::Clamp(Job.YardEntryWaypoint+1,0,Job.Route.Num()):Job.Route.Num();
    Candidate=SiteLogistics::RoundRoadCorners(Start,Candidate);
    // Sensor docking may leave the chassis a few centimetres from the exact
    // graph anchor. Do not reserve and drive a zero-length first segment when a
    // waiting AGV is activated for the next handover.
    while(Candidate.Num()>1 && FVector::Dist2D(Start,Candidate[0])<=AGVReference::DockingAccuracyCm+1.f)
        Candidate.RemoveAt(0);
    int32 CandidateYardEntryWaypoint=INDEX_NONE;
    if(Job.Stage==3)
    {
        CandidateYardEntryWaypoint=Candidate.Num()-1;
        SiteLogistics::AddUniqueRoutePoint(Candidate,Dock);
    }
    if(SiteLogistics::RouteIntersectsContainer(Candidate,Start,Yard,Job.Stage==5?Job.Slot:INDEX_NONE))
    {
        UE_LOG(LogTemp,Error,TEXT("AGV_ROUTE_REJECTED: V%d planned path intersects an occupied yard container"),ID);
        return false;
    }
    if(Congested)
    {
        auto Total=[&](const TArray<FVector>& Route,int32 Begin,int32 End)
        {
            FVector From=Vehicle->GetActorLocation(); double Sum=0;
            Begin=FMath::Clamp(Begin,0,Route.Num());
            End=FMath::Clamp(End,Begin,Route.Num());
            for(int32 I=Begin;I<End;++I) { Sum+=FVector::Dist2D(From,Route[I])+Cost(From,Route[I]); From=Route[I]; }
            return Sum;
        };
        if(Total(Candidate,0,Candidate.Num())+500>=
            Total(Job.Route,Job.Waypoint,ExistingRouteEnd)) return false;
        ++Job.Reroutes;
        UE_LOG(LogTemp,Display,TEXT("AGV_REROUTE: V%d stage=%d count=%d"),ID,Job.Stage,Job.Reroutes);
    }
    if(Job.Stage==5 && Job.ReturnCorridor==INDEX_NONE)
    {
        Job.ReturnCorridor=SelectedReturnCorridor;
        ++ReturnRouteCounts[SelectedReturnCorridor];
    }
    if(Job.Stage==3) Job.YardEntryWaypoint=CandidateYardEntryWaypoint;
    Job.Route=MoveTemp(Candidate); Job.Waypoint=0;
    // A new route must not inherit the final bay reservation from the previous
    // delivery/return. Its complete corridor is admitted independently.
    RoadReservations.Remove(ID); ReservationIndex.Remove(ID); RoadTargets.Remove(ID);
    FinishedRoadSegments.Remove(ID);
    return true;
}
void APortSiteLogistics::PrepareRoute(int32 Lane,bool Return)
{
    if(LaneCount==9)
    {
        Jobs[Lane].Stage=Return?5:3;
        if(!PlanRoadRoute(Lane,false))
        {
            auto& Job=Jobs[Lane];
            if(Return && Job.ReturnSTS!=INDEX_NONE)
            {
                const int32 ReservedSTS=Job.ReturnSTS;
                if(NextVehicles.IsValidIndex(ReservedSTS) && NextVehicles[ReservedSTS]==Lane)
                    NextVehicles[ReservedSTS]=INDEX_NONE;
                Job.ReturnSTS=Job.ReturnCargo=INDEX_NONE;
                // The direct STS route crossed an occupied stack. Return this
                // AGV to its own bay and leave the prepared cargo available for
                // a different free vehicle instead of stopping the terminal.
                if(PlanRoadRoute(Lane,false)) return;
            }
            Stop(TEXT("No road route between berth and yard"));
        }
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
        const bool YardEntryStop=Job.Stage==3 && !Job.bRMGReserved &&
            Job.Waypoint==Job.YardEntryWaypoint;
        const bool Docking=Job.Waypoint==Job.Route.Num()-1 || YardEntryStop;
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
        float LiDARPathLimit=TNumericLimits<float>::Max();
        if(Jobs.IsValidIndex(FleetIndex))
        {
            const auto& CurrentJob=Jobs[FleetIndex];
            if(CurrentJob.Route.IsValidIndex(CurrentJob.Waypoint+1))
            {
                const FVector NextDirection=(CurrentJob.Route[CurrentJob.Waypoint+1]-Target).GetSafeNormal2D();
                if(!NextDirection.IsNearlyZero() && FVector::DotProduct(TravelDirection,NextDirection)<.999f)
                    LiDARPathLimit=FMath::Max(0.f,FVector::DotProduct(
                        Target-Vehicle->GetActorLocation(),TravelDirection)+OwnLong+100.f);
            }
        }
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
            const FVector Delta=Other->GetActorLocation()-Vehicle->GetActorLocation();
            const float Along=FVector::DotProduct(Delta,TravelDirection);
            // At a planned turn, an obstacle beyond the current road segment is
            // outside the vehicle's future LiDAR corridor. Segment reservations
            // and the post-move footprint check still protect the intersection.
            if(Along>LiDARPathLimit) continue;
            // A sensor's longitudinal projection is a broad phase too. Check
            // the planned road chord before attributing a parking-bay return
            // to the driving corridor. Include the tangent arc's bow.
            if(Along>=0 && !AGVTrafficGeometry::SweptChassisIntersects(
                Vehicle->GetActorLocation(),Target,Other->GetActorLocation(),
                Other->GetActorRightVector(),AGVReference::PhysicalHalfLengthCm,
                AGVReference::PhysicalHalfWidthCm,100.f)) continue;
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
        // Keep a traversed corridor claimed during a safety stop. Releasing it
        // here admitted opposing traffic into the stopped vehicle's exit.
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
        if(Job.Stage==3 && !Job.bRMGReserved && Job.Waypoint==Job.YardEntryWaypoint)
            StraightThrough=false;
    }
    // The whole corridor was checked before entry. Advancing a sampled arc
    // node only consumes that claim; reacquiring it from the measured pose
    // can enlarge a corner box into a parking bay and strand an admitted AGV.
    // LiDAR and the physical post-move guard still run on every movement.
    if(LaneCount==9 && RoadTargets.Contains(ID) && RoadReservations.Contains(ID) &&
        RoadReservations[ID].Num()>1)
        RoadTargets.Add(ID,Target);
    if (!RoadTargets.Contains(ID) || !RoadTargets[ID].Equals(Target,.01f))
    {
        const FVector ToTarget=(Target-Vehicle->GetActorLocation()).GetSafeNormal2D();
        const FVector SegmentNormal(-ToTarget.Y,ToTarget.X,0);
        FVector Extent=!ToTarget.IsNearlyZero()?
            ToTarget.GetAbs()*AGVReference::PhysicalHalfLengthCm+
                SegmentNormal.GetAbs()*AGVReference::PhysicalHalfWidthCm+FVector(0,0,250):
            Vehicle->GetActorForwardVector().GetAbs()*AGVReference::PhysicalHalfWidthCm+
                Vehicle->GetActorRightVector().GetAbs()*AGVReference::PhysicalHalfLengthCm+FVector(0,0,250);
        const FVector Driven=Vehicle->PlannedMotionDirection(Target);
        if(!ToTarget.IsNearlyZero() && FVector::DotProduct(ToTarget,Driven)<.995f)
        {
            // The steering arc bulges perpendicular to the route chord.  A
            // radius expansion on both axes falsely intersects the next parked
            // row even though the vehicle never travels farther along the chord.
            // A 90-degree tangent arc bows only R*(1-cos45) outside its chord;
            // adding the whole radius made safe roads intersect parked AGVs.
            constexpr float QuarterTurnSagittaFactor=1.f-.70710678f;
            Extent+=SegmentNormal.GetAbs()*(AGVReference::MinimumInnerTurnRadiusCm*QuarterTurnSagittaFactor);
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
                    if(LaneCount!=9)
                        return (Job.Stage==3 && NextIndex>=Job.Route.Num()-3)?FVector(730,210,250):Extent;
                    const FVector Direction=(Job.Route[NextIndex]-Job.Route[NextIndex-1]).GetSafeNormal2D();
                    const FVector Normal(-Direction.Y,Direction.X,0);
                    return Direction.GetAbs()*AGVReference::PhysicalHalfLengthCm+
                        Normal.GetAbs()*(AGVReference::PhysicalHalfWidthCm+170.f)+FVector(0,0,250);
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
            // Check the full corridor, including stationary vehicles beyond the
            // first chord. Otherwise an AGV can enter an exit it cannot clear.
            FVector From=Vehicle->GetActorLocation();
            for(int32 SegmentIndex=0;SegmentIndex<Segments.Num();++SegmentIndex)
            {
                FVector To=Target;
                if(SegmentIndex>0 && Jobs.IsValidIndex(FleetIndex))
                {
                    const auto& Job=Jobs[FleetIndex];
                    const int32 Next=Job.Waypoint+SegmentIndex;
                    if(!Job.Route.IsValidIndex(Next)) break;
                    To=Job.Route[Next];
                }
                if(AGVTrafficGeometry::SweptChassisIntersects(From,To,
                    Other->GetActorLocation(),Other->GetActorRightVector(),
                    AGVReference::PhysicalHalfLengthCm,AGVReference::PhysicalHalfWidthCm,170.f))
                { Blocker=Other->VehicleID; break; }
                From=To;
            }
            if(Blocker!=INDEX_NONE) break;
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
        const bool Docking=Job.Waypoint==Job.Route.Num()-1 ||
            (Job.Stage==3 && !Job.bRMGReserved && Job.Waypoint==Job.YardEntryWaypoint);
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
    // The inbound and outbound berth lanes have independent directed segments.
    // Their swept vehicle envelopes and junction exits are reserved below; an
    // additional group-wide stop can deadlock an outbound AGV that is itself
    // waiting for this inbound vehicle to clear the junction.
    // Direct returns end in the off-lane STS waiting bay. They may reach that
    // bay while the current handover is occupied; activation below creates a
    // separate final approach only after the STS becomes available.
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
    if (Job.Stage==5 && !Job.bYardReleased &&
        FVector::Dist2D(Vehicle->GetActorLocation(),YardHandover(Yard[Job.Slot]))>3000.f)
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
            UE_LOG(LogTemp,Display,TEXT("TRAFFIC_V%d: stage=%d sts=%d rmg=%d wp=%d/%d pos=%s target=%s blocked_by=%d speed=%.1f yaw=%.1f fms_hold=%d lidar_clear=%d reserved=%d"),Vehicles[I]->VehicleID,J.Stage,J.STS,J.RMG,J.Waypoint,J.Route.Num(),*Vehicles[I]->GetActorLocation().ToCompactString(),J.Route.IsValidIndex(J.Waypoint)?*J.Route[J.Waypoint].ToCompactString():TEXT("none"),RoadBlockers.FindRef(Vehicles[I]->VehicleID),Vehicles[I]->Speed,Vehicles[I]->GetActorRotation().Yaw,Vehicles[I]->Sensors.bFMSHold,Vehicles[I]->Sensors.bLiDARClear,RoadReservations.Contains(Vehicles[I]->VehicleID));
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
            if(LaneCount==9 && Job.Waypoint==0)
            {
                // Three STSs around one vessel feed one outbound cross-road.
                // Admit the AGV nearest that cross-road first; once it reaches
                // waypoint 1 the next vehicle may follow under normal segment
                // reservations. This preserves three-vessel parallelism while
                // preventing two centre-group vehicles from meeting head-on.
                const int32 Group=TerminalLayout::AGVBerthGroup(Job.STS);
                const float CrossY=TerminalLayout::AGVLoadedCrossY(Job.STS);
                const float MyDistance=FMath::Abs(Vehicle->GetActorLocation().Y-CrossY);
                int32 Leader=INDEX_NONE;
                float LeaderDistance=MyDistance;
                for(int32 Other=0;Other<Jobs.Num();++Other)
                {
                    if(Other==Lane || Jobs[Other].Stage!=3 ||
                        TerminalLayout::AGVBerthGroup(Jobs[Other].STS)!=Group) continue;
                    if(Jobs[Other].Waypoint>0)
                    {
                        const FVector Cross(TerminalLayout::SiteCmX(
                            TerminalLayout::AGVQuayOutboundLaneX),CrossY,0);
                        if(FVector::Dist2D(Vehicles[Other]->GetActorLocation(),Cross)<3000.f)
                        { Leader=Other; break; }
                        continue;
                    }
                    const float OtherDistance=FMath::Abs(
                        Vehicles[Other]->GetActorLocation().Y-CrossY);
                    if(OtherDistance<LeaderDistance-1.f ||
                        (FMath::IsNearlyEqual(OtherDistance,LeaderDistance,1.f) &&
                            Vehicles[Other]->VehicleID<Vehicle->VehicleID))
                    { Leader=Other; LeaderDistance=OtherDistance; }
                }
                if(Leader!=INDEX_NONE)
                {
                    const int32 ID=Vehicle->VehicleID;
                    Vehicle->Speed=0.f;
                    Vehicle->SetFMSHold(true);
                    RoadBlockers.Add(ID,Vehicles[Leader]->VehicleID);
                    RoadReservations.Remove(ID); ReservationIndex.Remove(ID);
                    RoadTargets.Remove(ID);
                    break;
                }
            }
            if (!Drive(Lane,Dt)) break;
            if(!Job.bRMGReserved)
            {
                Job.Stage=9;
                const int32 ID=Vehicle->VehicleID;
                RoadReservations.Remove(ID); ReservationIndex.Remove(ID); RoadTargets.Remove(ID);
                FinishedRoadSegments.Remove(ID);
                break;
            }
            YardApproachOwners[YardApproachKey(Yard[Job.Slot])]=INDEX_NONE;
            // Keep the twist locks engaged until the RMG actually picks up the box.
            if (!Equipment[Job.RMG]->AssignCargo(Cargo,Vehicle->CargoPosition(),Yard[Job.Slot].Position,false,true,Vehicle))
            { Stop(TEXT("AGV / RMG reservation handover")); return; }
            Manifest[Job.Cargo].HandoverMask|=2;
            UE_LOG(LogTemp,Display,TEXT("SITE_HANDOVER: C%d AGV%d -> RMG%d"),Manifest[Job.Cargo].ID,100+Lane,Job.RMG+1);
            Job.Stage=4; break;
        case 4:
            if (Equipment[Job.RMG]->IsBusy()) break;
            // Multiple empty returns may leave different RMGs together.  Their
            // selected south/central/north roads and intersections are guarded
            // by the same segment reservations and LiDAR checks as loaded AGVs.
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
                if(STSOwners[STS]!=INDEX_NONE || !PreparedStarted[STS] ||
                    !YardApproachAvailable(PreparedCargo[STS]))
                {
                    TArray<FVector> WaitingRoute=MoveTemp(Job.Route);
                    const int32 WaitingWaypoint=Job.Waypoint;
                    Job=FSiteTransfer();
                    Job.STS=STS; Job.Stage=8;
                    Job.Route=MoveTemp(WaitingRoute); Job.Waypoint=WaitingWaypoint;
                    Vehicle->Speed=0.f;
                    break;
                }
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
    YardApproachOwners.Init(INDEX_NONE,LaneCount==9?YardCraneCount:TerminalLayout::YardBlockCount); BlocksBusy.Init(false,TerminalLayout::YardBlockCount); RMGBusy.Init(false,YardCraneCount); PreparedCargo.Init(INDEX_NONE,LaneCount); SlotAssigned.Init(false,Yard.Num());
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
            {
                const int32 Lane=Vehicles.IndexOfByKey(Vehicle);
                const auto& J=Jobs[Lane];
                Error=FString::Printf(TEXT("AGV %d escaped yard/road bounds at %s extent=%s yaw=%.1f stage=%d wp=%d/%d target=%s"),
                    Vehicle->VehicleID,*P.ToCompactString(),*E.ToCompactString(),Vehicle->GetActorRotation().Yaw,
                    J.Stage,J.Waypoint,J.Route.Num(),J.Route.IsValidIndex(J.Waypoint)?
                        *J.Route[J.Waypoint].ToCompactString():TEXT("none")); return false;
            }
            if(!Vehicle->PayloadWithinReferenceLimit())
            { Error=FString::Printf(TEXT("AGV %d exceeded 65 t payload limit"),Vehicle->VehicleID); return false; }
            if(Vehicle->Sensors.PositionErrorCm>AGVReference::PositionAccuracyCm+.01f)
            { Error=FString::Printf(TEXT("AGV %d localization exceeded transponder accuracy"),Vehicle->VehicleID); return false; }
            if(Vehicle->Sensors.bControlledStop && Vehicle->Speed>.1f)
            { Error=FString::Printf(TEXT("AGV %d moved through a LiDAR controlled stop"),Vehicle->VehicleID); return false; }
            if(Vehicle->Sensors.LateralSlipCm>.001f)
            { Error=FString::Printf(TEXT("AGV %d used forbidden lateral motion"),Vehicle->VehicleID); return false; }
            const bool SupportXOverlap=P.X-E.X<TerminalLayout::STSLandsideSupportWorldX+
                    TerminalLayout::STSSupportHalfWidthCm &&
                P.X+E.X>TerminalLayout::STSLandsideSupportWorldX-
                    TerminalLayout::STSSupportHalfWidthCm;
            if(SupportXOverlap)
                for(int32 STS=0;STS<LaneCount;++STS)
                {
                    const float CraneY=Equipment[YardCraneCount+STS]->GetActorLocation().Y;
                    for(float Side:{-1000.f,1000.f})
                        if(FMath::Abs(P.Y-(CraneY+Side))<E.Y+250.f)
                        {
                            const int32 VehicleLane=Vehicles.IndexOfByKey(Vehicle);
                            const auto& VehicleJob=Jobs[VehicleLane];
                            Error=FString::Printf(TEXT("AGV %d crossed STS%d landside support at %s yaw=%.1f stage=%d wp=%d/%d"),
                                Vehicle->VehicleID,STS+1,*P.ToCompactString(),Vehicle->GetActorRotation().Yaw,
                                VehicleJob.Stage,VehicleJob.Waypoint,VehicleJob.Route.Num());
                            return false;
                        }
                }
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
            const FVector Queue=QueueQuay(Job.STS);
            if(Job.Route.IsEmpty() || !Job.Route.Last().Equals(Queue,.1f) ||
                (Job.Stage==8 && (!Vehicles[Lane]->GetActorLocation().Equals(Queue,
                    AGVReference::DockingAccuracyCm) || Vehicles[Lane]->Speed>.1f)))
            { Error=TEXT("Queued AGV is not routed/stopped at its STS waiting bay"); return false; }
            if (Vehicles[Lane]->Speed>0 && !RoadReservations.Contains(Vehicles[Lane]->VehicleID))
            { Error=TEXT("Queued AGV moved without a road reservation"); return false; }
            continue;
        }
        if(Job.ReturnSTS!=INDEX_NONE &&
            (Job.Stage!=5 || NextVehicles[Job.ReturnSTS]!=Lane || PreparedCargo[Job.ReturnSTS]!=Job.ReturnCargo ||
                Job.Actor.IsValid() || Job.Route.IsEmpty() || !Job.Route.Last().Equals(QueueQuay(Job.ReturnSTS),.1f)))
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
            ((Job.Stage!=6 && Job.Stage!=1 && Job.Stage!=2 && Job.Stage!=3 && Job.Stage!=9) ||
                YardApproachOwners[YardApproachKey(Yard[Job.Slot])]!=Lane))
        { Error=TEXT("Missing yard approach admission"); return false; }
        if(Job.Stage==9 && (Vehicles[Lane]->Speed>.1f || Job.Waypoint!=Job.YardEntryWaypoint+1 ||
            !Vehicles[Lane]->GetActorLocation().Equals(Job.Route[Job.Waypoint-1],
                AGVReference::DockingAccuracyCm)))
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
