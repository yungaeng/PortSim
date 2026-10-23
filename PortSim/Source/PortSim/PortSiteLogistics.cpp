#include "PortSiteLogistics.h"
#include "YardStackingPolicy.h"
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
#include "AGVApronRoads.h"
#include "AGVDispatchDependencies.h"
#include "SimulationPerformance.h"
#include "Components/HierarchicalInstancedStaticMeshComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Components/TextRenderComponent.h"
#include "Components/WidgetComponent.h"
#include "ProceduralMeshComponent.h"
#include "Engine/World.h"
#include "Misc/Paths.h"
#include "Misc/FileHelper.h"
#include "HAL/FileManager.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "Misc/ScopeExit.h"
#include "EngineUtils.h"
#include "PhysicsEngine/PhysicsSettings.h"
#include "PhysicsEngine/BodyInstance.h"
#include "PhysicalMaterials/PhysicalMaterial.h"
#include "Physics/Experimental/PhysScene_Chaos.h"

namespace SiteLogistics
{
    FString CsvField(FString Value)
    {
        Value.ReplaceInline(TEXT("\""),TEXT("\"\""));
        return TEXT("\"")+Value+TEXT("\"");
    }
    constexpr float LegacyCraneY[]={-450,-350,-250,-100,100,250,350,450};
    FTransform Hidden(FTransform T) { T.SetScale3D(FVector::ZeroVector); return T; }
    FTransform YardTransform(const FSiteYardSlot& Slot)
    { return FTransform(FQuat::Identity,Slot.Position,FVector(TerminalLayout::YardLengthCm(Slot.LengthFt)/100.f,2.438,2.59)); }
    float ReturnQueueAccessY(float QueueY,float DockY)
    {
        const float Clearance=2*AGVReference::MinimumInnerTurnRadiusCm+100.f;
        float Best=TerminalLayout::AGVParkY(TerminalLayout::AGVParkBays-1)+3000.f;
        if(FMath::Abs(Best-DockY)<Clearance) Best+=2500.f;
        double Distance=TNumericLimits<double>::Max();
        for(int32 Gap=1;Gap<TerminalLayout::AGVParkBays-1;Gap+=2)
        {
            const float Y=TerminalLayout::AGVParkCrossingY(Gap);
            if(Y-QueueY<Clearance || FMath::Abs(Y-DockY)<Clearance) continue;
            if(FMath::Abs(Y-QueueY)<Distance) { Best=Y; Distance=FMath::Abs(Y-QueueY); }
        }
        return Best;
    }
    bool AGVFootprintsOverlap(const APortAGVActor* A,const APortAGVActor* B)
    {
        constexpr float HalfWidth=AGVTrafficGeometry::BodyHalfWidthCm;
        constexpr float HalfLength=AGVTrafficGeometry::BodyHalfLengthCm;
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
    TArray<FVector> RoundRoadCorners(const FVector& Start,const TArray<FVector>& Nodes,
        TFunctionRef<bool(FVector,FVector)> SafeStop)
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
            {
                FVector Point=FMath::Lerp(From,To,double(Step)/Steps);
                const FVector Direction=(To-From).GetSafeNormal2D();
                // Keep every steering chord. Straight admission boundaries must
                // leave a whole chassis clear of road intersections and STS bays.
                if(Step<Steps && !SafeStop(Point,Direction))
                {
                    bool Found=false;
                    const FVector Previous=Gated.IsEmpty()?Start:Gated.Last();
                    for(float Offset=100;Offset<=1500 && !Found;Offset+=100)
                        for(float Sign:{-1.f,1.f})
                        {
                            const FVector Alternative=Point+Direction*(Sign*Offset);
                            if(FVector::DotProduct(Alternative-Previous,Direction)<100.f ||
                                FVector::DotProduct(To-Alternative,Direction)<100.f ||
                                !SafeStop(Alternative,Direction)) continue;
                            Point=Alternative; Found=true; break;
                        }
                    if(!Found) continue;
                }
                AddUniqueRoutePoint(Gated,Point);
            }
            From=To;
        }
        return Gated;
    }

    bool RouteIntersectsContainer(const TArray<FVector>& Route,const FVector& Start,
        const TArray<FSiteYardSlot>& Yard,int32 IgnoredSlot=INDEX_NONE)
    {
        PortSimPerformance::FScope Profile(TEXT("yard_route_safety"));
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
                if(!Slot.Occupied) continue;
                if(Yard.IsValidIndex(IgnoredSlot) &&
                    FVector::DistSquared2D(Slot.Position,Yard[IgnoredSlot].Position)<1.f) continue;
                const FVector ContainerExtent(TerminalLayout::YardLengthCm(Slot.LengthFt)*.5f+5.f,130,260);
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

void APortSiteLogistics::SetSurfaceConditions(const FPortSurfaceState& State,const FPortFrictionSettings& Settings,UPhysicalMaterial* Material)
{
    const int32 Count=TrafficVehicles.Num()+Equipment.Num()+ShipContainers.Num()+PlacedContainers.Num();
    if(!Material) return;
    const bool BindMaterial=SurfaceMaterial!=Material || SurfaceActorCount!=Count;
    SurfaceActorCount=Count;
    Surface=State; Friction=Settings; SurfaceMaterial=Material;
    auto Apply=[&](AActor* Actor)
    {
        if(!IsValid(Actor)) return;
        TInlineComponentArray<UPrimitiveComponent*> Parts(Actor);
        for(auto* Part:Parts) if(Part->GetCollisionEnabled()!=ECollisionEnabled::NoCollision) Part->SetPhysMaterialOverride(Material);
    };
    for(const auto& Vehicle:TrafficVehicles) if(IsValid(Vehicle))
    { Vehicle->Surface=State; Vehicle->Friction=Settings; if(BindMaterial) Apply(Vehicle); }
    for(const auto& Crane:Equipment) if(IsValid(Crane))
    { Crane->Surface=State; Crane->Friction=Settings; if(BindMaterial) Apply(Crane); }
    if(BindMaterial)
    {
        for(const auto& Cargo:ShipContainers) Apply(Cargo);
        for(const auto& Cargo:PlacedContainers) Apply(Cargo);
    }
}

void APortSiteLogistics::SetWindVelocity(FVector WorldVelocityMetersPerSecond)
{
    WindVelocityMetersPerSecond=WorldVelocityMetersPerSecond.ContainsNaN()?FVector::ZeroVector:
        WorldVelocityMetersPerSecond.GetClampedToMaxSize(50.f);
    for(const auto& Crane:Equipment) if(IsValid(Crane)) Crane->SetWindVelocity(WindVelocityMetersPerSecond);
    for(const auto& Vehicle:TrafficVehicles) if(IsValid(Vehicle)) Vehicle->WindVelocity=WindVelocityMetersPerSecond;
}

void APortSiteLogistics::SetFogVisibilityMeters(float VisibilityMeters)
{
    FogVisibilityMeters=FMath::IsFinite(VisibilityMeters)?FMath::Max(0.f,VisibilityMeters):MAX_flt;
    for(const auto& Crane:Equipment) if(IsValid(Crane)) Crane->SetFogVisibilityMeters(FogVisibilityMeters);
}

void APortSiteLogistics::SetRainState(const FPortRainState& State)
{
    RainState=State;
    const float CameraScale=RainState.SensorRangeScale(TEXT("camera"));
    const float LiDARScale=RainState.SensorRangeScale(TEXT("lidar"));
    for(const auto& Crane:Equipment) if(IsValid(Crane)) Crane->SetRainSensorRangeScales(CameraScale,LiDARScale);
}

void APortSiteLogistics::SetVesselMotions(const TArray<FPortVesselMotion>& Motions,const TArray<FVector>& BasePivots,const FPortSeaState& SeaState)
{
    FrameSeaState=SeaState;
    VesselMotions=Motions;
    VesselBasePivots=BasePivots;
}

bool APortSiteLogistics::ResolveVesselTransform(const FSiteShipCargo& Cargo,FVector& Position,FQuat& Rotation,FVector& Velocity,FVector& Acceleration,FVector& Jerk) const
{
    const int32 Vessel=LaneCount==9?Cargo.STS/3:INDEX_NONE;
    if(!VesselMotions.IsValidIndex(Vessel) || !VesselBasePivots.IsValidIndex(Vessel)) return false;
    const auto& Motion=VesselMotions[Vessel];
    const FVector Base=Cargo.Transform.GetLocation(),Pivot=VesselBasePivots[Vessel];
    Position=Motion.TransformPosition(Base,Pivot);
    Rotation=Motion.TransformRotation(Cargo.Transform.GetRotation());
    Velocity=Motion.VelocityAtPosition(Base,Pivot);
    Acceleration=Motion.AccelerationAtPosition(Base,Pivot);
    Jerk=Motion.JerkAtPosition(Base,Pivot);
    return true;
}

void APortSiteLogistics::ApplyVesselMotions(float FrameDurationSeconds)
{
    for(auto* Records:{&Manifest,&RetainedCargo,&ExportCargo}) for(auto& Cargo:*Records)
    {
        auto* Actor=Cargo.Actor.Get();
        if(!IsValid(Actor) || Actor->LocationOwner!=ECargoOwner::Ship || Actor->GetAttachParentActor()) continue;
        FVector Position,Velocity,Acceleration,Jerk;FQuat Rotation;
        if(!ResolveVesselTransform(Cargo,Position,Rotation,Velocity,Acceleration,Jerk)) continue;
        Actor->SetSecuredVesselMotion(Position,Rotation,Velocity,Acceleration,Jerk,FrameDurationSeconds,!bDeferringVisuals);
        if(Equipment.IsValidIndex(YardCraneCount+Cargo.STS))
            Equipment[YardCraneCount+Cargo.STS]->UpdateMovingShipSource(Actor,Actor->GetActorLocation());
    }
}

void APortSiteLogistics::AddShipCargo(FVector Position,int32 STS,int32 Wave,FName ContainerName,bool bLoadFromYard,
    bool bRetainAboard,FIntVector ImportedSlot)
{
    check(STS>=0 && STS<9);
    FSiteShipCargo Record;
    Record.Transform=FTransform(FQuat::Identity,Position);
    const int32 Sequence=Manifest.Num()+ExportCargo.Num()+RetainedCargo.Num();
    Record.STS=STS; Record.ID=2000+Sequence; Record.Wave=Wave;
    FActorSpawnParameters Params; Params.Owner=this;
    Params.SpawnCollisionHandlingOverride=ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
    auto* Cargo=GetWorld()->SpawnActor<APortContainerActor>(Position,FRotator::ZeroRotator,Params);
    check(Cargo);
    Cargo->InitializeContainer(Record.ID);
    if(!ContainerName.IsNone()) Cargo->ContainerID=ContainerName;
    Record.ContainerName=Cargo->ContainerID;
#if WITH_EDITOR
    Cargo->SetActorLabel(FString::Printf(TEXT("Container_%s"),*Record.ContainerName.ToString()));
#endif
    int32 Total=0; for(int32 I=0;I<ConfiguredShips;++I) Total+=ConfiguredCargo[I];
    Cargo->ConfigureSpecification(PortContainerSpecification::ForSequence(Record.ID,Sequence,Total),
        STSProfile.bReady?STSProfile.ContainerCoG:FVector::ZeroVector);
    if(ImportedSlot.X>=0)
    {
        // CSV slots describe single 40ft cells; never substitute a random size.
        Cargo->ConfigureSpecification(FPortContainerSpecification(),FVector::ZeroVector);
        Record.bExplicitPlacement=true;
        Record.ShipSlot.CargoID=Record.ID;
        Record.ShipSlot.Bay=ImportedSlot.X+1; Record.ShipSlot.Row=ImportedSlot.Y+1;
        Record.ShipSlot.Tier=ImportedSlot.Z+1;
        Record.ShipSlot.PositionM=FVector(Position.Y*.01-TerminalLayout::VesselCenterY(STS/3),
            Position.X*.01-TerminalLayout::VesselCenterXM,Position.Z*.01);
    }
    Cargo->CargoInformation=FPortCargoInformation::ForSequence(Record.ID,Cargo->LengthFt);
    Record.BookedGrossKg=Cargo->MassKg;
    // Preserve the existing injected-overload test path without replacing normal
    // per-container manifest masses with the profile's nominal fallback mass.
    if(STSProfile.bReady && STSProfile.ContainerMassKg>STSProfile.RatedPayloadKg)
        Cargo->SetPhysicalParameters(STSProfile.ContainerMassKg,STSProfile.ContainerCoG);
    // Secured aboard until pickup: real collidable actors, without idle rigid-body simulation.
    Cargo->GetBody()->SetSimulatePhysics(false);
    Cargo->LocationOwner=ECargoOwner::Ship;
    Cargo->SetActorHiddenInGame(Wave>0); Cargo->SetActorEnableCollision(Wave==0);
    Record.Actor=Cargo;
    ShipContainers.Add(Cargo);
    if(bLoadFromYard)
    {
        Cargo->LocationOwner=ECargoOwner::Yard;
        Cargo->SetActorHiddenInGame(true); Cargo->SetActorEnableCollision(false);
        ExportCargo.Add(Record);
        return;
    }
    if(bRetainAboard) { RetainedCargo.Add(Record); return; }
    Manifest.Add(Record);
}

bool APortSiteLogistics::PlaceExportCargo()
{
    // Ground pads keep export bookings accessible and avoid unsupported mixed stacks.
    for(auto& Booking:ExportCargo)
    {
        auto* Actor=Booking.Actor.Get();
        const int32 Index=Yard.IndexOfByPredicate([Actor](const FSiteYardSlot& Slot)
        {
            return Slot.Reserved && !Slot.Occupied && Slot.Below==INDEX_NONE &&
                Slot.LengthFt==Actor->LengthFt &&
                YardStackingPolicy::ZoneAccepts(Slot.Block,Actor->CargoInformation);
        });
        if(Index==INDEX_NONE)
        { Stop(TEXT("적하 컨테이너를 배치할 수 있는 야드 바닥 슬롯이 없습니다.")); return false; }
        auto& Slot=Yard[Index];
        Slot.Reserved=false; Slot.Occupied=true;
        Booking.PlannedSlot=Index;
        Actor->SetActorLocationAndRotation(Slot.Position,FRotator(0,90,0));
        Actor->SetActorHiddenInGame(false); Actor->SetActorEnableCollision(true);
        --ReceivingCapacity; ++InitialYard;
    }
    return true;
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
    return FVector(TerminalLayout::SiteCmX(bDepotParking?TerminalLayout::AGVStandbyX(Vehicle):TerminalLayout::AGVParkX(Vehicle)),
        bDepotParking?TerminalLayout::AGVStandbyY(Vehicle):TerminalLayout::AGVParkY(Vehicle),0);
}

void APortSiteLogistics::Initialize(const TArray<TObjectPtr<APortWorkingCrane>>& Cranes,TArray<FSiteYardSlot> Slots,
    const TArray<UHierarchicalInstancedStaticMeshComponent*>& Palette,int32 CentralCargo,int32 FixedYard)
{
    bDepotParking=!FParse::Param(FCommandLine::Get(),TEXT("PortSimLegacyParking"));
    Equipment=Cranes; Yard=MoveTemp(Slots); CentralCount=CentralCargo; YardCraneCount=Cranes.Num()-(CentralCount==0?9:8); LaneCount=Equipment.Num()-YardCraneCount;
    bReuseFreedApron=bDepotParking && LaneCount==9 && !FParse::Param(FCommandLine::Get(),TEXT("PortSimPreviousApron"));
    bPickupRoadPriority=LaneCount==9 && !FParse::Param(FCommandLine::Get(),TEXT("PortSimLegacyRoadPriority"));
    DispatchPriorityLimit=INDEX_NONE;
    BaselineYard=Yard.Num()+FixedYard;
    check((LaneCount==8 || LaneCount==9) && Yard.Num()>=InitialShipCount());
    // Remove TOP tiers only; filling the reservation bottom-up restores supported stacks.
    Yard.StableSort([](const FSiteYardSlot& A,const FSiteYardSlot& B) { return A.Position.Z==B.Position.Z ? A.Position.X<B.Position.X : A.Position.Z>B.Position.Z; });
    for (int32 I=0;I<Yard.Num();++I)
    {
        auto& Slot=Yard[I]; Slot.Reserved=CentralCount>0 && I<InitialShipCount();
    }
    if (CentralCount==0)
        for(auto& Slot:Yard) Slot.Reserved=true;
    ReceivingCapacity=0;
    for (auto& Slot:Yard)
    {
        ReceivingCapacity+=Slot.Reserved;
        Slot.Occupied=!Slot.Reserved;
        Slot.Mesh=Palette[Slot.Color];
        const FTransform T=SiteLogistics::YardTransform(Slot);
        Slot.Instance=Slot.Mesh->AddInstance(Slot.Reserved?SiteLogistics::Hidden(T):T);
    }
    Yard.StableSort([](const FSiteYardSlot& A,const FSiteYardSlot& B) { return A.Position.Z<B.Position.Z; });
    InitialYard=BaselineYard-ReceivingCapacity;
    check(ReceivingCapacity>=InitialShipCount());
    TMap<FIntVector,int32> Positions;
    TMap<FIntVector,int32> Pads;
    for (int32 I=0;I<Yard.Num();++I)
    {
        const FVector P=Yard[I].Position;
        const FIntVector Key(FMath::RoundToInt(P.X),FMath::RoundToInt(P.Y),FMath::RoundToInt((P.Z-149.5f)/259));
        if (const int32* Below=Positions.Find(Key-FIntVector(0,0,1))) Yard[I].Below=*Below;
        Positions.Add(Key,I);
        Pads.Add(FIntVector(Yard[I].Cell,Key.Z,Yard[I].Subslot),I);
        check(Key.Z==0 || Yard[I].Below!=INDEX_NONE);
    }
    for(auto& Slot:Yard) if(Slot.LengthFt==20)
    {
        const int32 Tier=FMath::RoundToInt((Slot.Position.Z-149.5f)/259);
        const int32* Partner=Pads.Find(FIntVector(Slot.Cell,Tier,1-Slot.Subslot));
        check(Partner); Slot.Partner=*Partner;
    }
    CentralSlots.Reset();
    if(!PlaceExportCargo()) return;
    for (int32 Cargo=0;Cargo<CentralCount;++Cargo)
    {
        const int32 Slot=Yard.IndexOfByPredicate([Cargo](const FSiteYardSlot& S)
        { return S.Block==Cargo%TerminalLayout::YardBlockCount && S.Reserved && !S.Central; });
        check(Slot!=INDEX_NONE);
        Yard[Slot].Central=true;
        CentralSlots.Add(Slot);
    }
    FString PlanningError;
    if(!ShipStowageAssumptions.Load(FPaths::ProjectConfigDir()/TEXT("Stowage_Planning.json"),PlanningError))
    { Stop(PlanningError); return; }
    if(LaneCount==9 && !PlanArrivalStowage()) return;
    // Plan the yard only AFTER applying the physical ship plan. Upper ship
    // tiers leave first, so no boxes are lifted through an upper stack.
    Manifest.StableSort([](const FSiteShipCargo& A,const FSiteShipCargo& B) { return A.Wave!=B.Wave?A.Wave<B.Wave:A.Transform.GetLocation().Z>B.Transform.GetLocation().Z; });
    BuildSourceAccess();
    if(!PlanYardDestinations()) { Stop(TEXT("Could not create supported pre-discharge yard plan")); return; }
    for(auto& C:Manifest) if(C.OriginalSTS==INDEX_NONE) C.OriginalSTS=C.STS;
    InitialManifestPlan=Manifest;
    FParse::Value(FCommandLine::Get(),TEXT("PortSimFailedSTS="),ScheduledFailedSTS);
    FParse::Value(FCommandLine::Get(),TEXT("PortSimFailedRMG="),ScheduledFailedRMG);
    FParse::Value(FCommandLine::Get(),TEXT("PortSimFailureAt="),ScheduledFailureAt);
    FParse::Value(FCommandLine::Get(),TEXT("PortSimRepairAfter="),ScheduledRepairAfter);
    if(LaneCount!=9) for(int32 Vessel=0;Vessel<ConfiguredShips;++Vessel)
    {
        TArray<FPortShipPlanningCargo> Bookings;
        for(const auto& C:Manifest) if(C.STS/3==Vessel)
            Bookings.Add({C.ID,C.Actor->LengthFt,C.Actor->MassKg,C.Actor->CargoInformation,
                FVector(C.Actor->CoGOffsetCm.Y,C.Actor->CoGOffsetCm.X,C.Actor->CoGOffsetCm.Z)*.01});
        ShipStowagePlans.Add(PlanShipStowage(Bookings,ShipStowageAssumptions));
    }
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

bool APortSiteLogistics::SourceAccessible(int32 Cargo) const
{
    const auto& C=Manifest[Cargo];
    if(C.PlanningOrder>=DispatchLimit || C.Actor->LocationOwner!=ECargoOwner::Ship) return false;
    for(int32 Above:SourceAbove[Cargo])
        if(Manifest[Above].Actor->LocationOwner==ECargoOwner::Ship) return false;
    if(C.Wave>0) for(const auto& Other:Manifest)
        if(Other.STS/3==C.STS/3 && Other.Wave<C.Wave && Other.Actor->LocationOwner==ECargoOwner::Ship) return false;
    return true;
}

void APortSiteLogistics::PrepareExecutableCargo(int32 Lane)
{
    if(!CraneAvailable(YardCraneCount+Lane)) return;
    const int32 Current=PreparedCargo[Lane];
    if(PreparedStarted[Lane] ||
        (Current==INDEX_NONE && Dispatched>=DispatchLimit) ||
        (STSOwners[Lane]!=INDEX_NONE && Jobs[STSOwners[Lane]].Stage==10)) return;
    TArray<AGVDispatchPolicy::FRequest> Requests;
    for(int32 I:ManifestBySTS[Lane])
    {
        auto& C=Manifest[I];
        if(C.STS!=Lane || (C.State!=0 && I!=Current) || !SourceAccessible(I) || !CanSTSService(Lane,I)) continue;
        if(C.DispatchEligibleAt<0) C.DispatchEligibleAt=SimulationTime;
        Requests.Add({I,C.PlanningOrder,C.DispatchEligibleAt,true,YardApproachAvailable(I),
            C.DispatchCriticalPath,C.DispatchUnlocks});
    }
    const int32 Best=AGVDispatchPolicy::SelectExecutable(Requests);
    // A returning vehicle owns its physical bay, not an irrevocable cargo
    // choice. Keep the placeholder if nothing is executable, but permit a
    // different accessible box at this STS as soon as its yard is ready.
    if(Best==INDEX_NONE && NextVehicles[Lane]!=INDEX_NONE) return;
    if(Best==Current) return;
    if(Current!=INDEX_NONE)
    {
        Manifest[Current].State=0; --Dispatched;
        PreparedCargo[Lane]=INDEX_NONE;
    }
    if(Best==INDEX_NONE) return;
    auto& C=Manifest[Best];
    PreparedCargo[Lane]=Best; PreparedStarted[Lane]=false;
    C.State=1; C.StartedAt=SimulationTime; C.PreparedPausedSeconds=0; ++Dispatched;
    if(VesselStarted[Lane/3]<0) VesselStarted[Lane/3]=SimulationTime;
    if(C.Wave>0) for(auto& Other:Manifest)
        if(Other.STS/3==Lane/3 && Other.Wave==C.Wave && Other.Actor->LocationOwner==ECargoOwner::Ship)
        { Other.Actor->SetActorHiddenInGame(false); Other.Actor->SetActorEnableCollision(true); }
    const int32 Returning=NextVehicles[Lane];
    if(Returning!=INDEX_NONE && Jobs[Returning].Stage==5) Jobs[Returning].ReturnCargo=Best;
    // Leave the box secured aboard until ActivateVehicle atomically claims
    // the AGV and downstream approach. No speculative suspended load.
}

void APortSiteLogistics::PrepareNextCargo(int32 Lane)
{
    if(!CraneAvailable(YardCraneCount+Lane)) return;
    if(LaneCount==9) { PrepareExecutableCargo(Lane); return; }
    if(STSOwners.IsValidIndex(Lane) && STSOwners[Lane]!=INDEX_NONE && Jobs[STSOwners[Lane]].Stage==10) return;
    if (PreparedCargo[Lane]==INDEX_NONE)
    {
        if (Dispatched>=DispatchLimit) return;
        const int32 Next=Manifest.IndexOfByPredicate([this,Lane](const FSiteShipCargo& C)
        {
            if(C.STS!=Lane || C.State!=0) return false;
            // A bounded run must unload a prefix of the dependency-safe plan;
            // otherwise fast STSs can consume its quota with upper boxes whose
            // supports lie outside the selected shipment count.
            if(LaneCount==9 && C.PlanningOrder>=DispatchLimit) return false;
            const FVector P=C.Transform.GetLocation();
            // A paired box reserved for a second pickup is still aboard. Do not
            // prefetch a lower box through it, even if its state is already 1.
            for(const auto& Above:Manifest)
                if(Above.Wave==C.Wave && Above.Actor->LocationOwner==ECargoOwner::Ship &&
                    Above.Transform.GetLocation().Z>P.Z+1.f &&
                    FVector::DistSquared2D(Above.Transform.GetLocation(),P)<1.f) return false;
            return true;
        });
        if (Next==INDEX_NONE) return;
        const int32 Wave=Manifest[Next].Wave;
        if(Wave>0)
        {
            for(const auto& C:Manifest)
                if(C.STS/3==Lane/3 && C.Wave<Wave && C.Actor->LocationOwner==ECargoOwner::Ship) return;
            for(auto& C:Manifest) if(C.STS/3==Lane/3 && C.Wave==Wave && C.State==0)
            { C.Actor->SetActorHiddenInGame(false); C.Actor->SetActorEnableCollision(true); }
        }
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
    Job.bParkingDeparture=!FromQueue;
    Job.StartedAt=Manifest[Job.Cargo].StartedAt; Job.Time=SimulationTime-Job.StartedAt;
    Job.PausedSeconds=Manifest[Job.Cargo].PreparedPausedSeconds;
    Job.TripID=NextTripID++;
    const int32 Mate=Manifest[Job.Cargo].PlannedPair;
    if(Manifest.IsValidIndex(Mate) && Manifest[Mate].State==0 && Dispatched<DispatchLimit &&
        Manifest[Mate].STS==STS && CanSTSService(STS,Mate) &&
        Manifest[Mate].PlanningOrder<DispatchLimit &&
        SourceAccessible(Mate) &&
        Yard[Manifest[Mate].PlannedSlot].Crane==Yard[Manifest[Job.Cargo].PlannedSlot].Crane &&
        (Yard[Manifest[Mate].PlannedSlot].Below==Manifest[Job.Cargo].PlannedSlot ||
            Yard[Manifest[Mate].PlannedSlot].Below==INDEX_NONE || Yard[Yard[Manifest[Mate].PlannedSlot].Below].Occupied))
    {
        Job.ExtraCargo=Mate; Job.CargoDeckOffsetCm=-YardStackingPolicy::PairPadOffsetCm;
        Manifest[Mate].State=1; Manifest[Mate].StartedAt=Job.StartedAt;
        if(Manifest[Mate].DispatchEligibleAt<0) Manifest[Mate].DispatchEligibleAt=SimulationTime;
        Manifest[Mate].VehicleAssignedAt=SimulationTime;
        Manifest[Mate].PreparedPausedSeconds=Job.PausedSeconds; ++Dispatched;
    }
    Vehicles[Vehicle]->ActiveDeckOffsetCm=Job.CargoDeckOffsetCm;
    // Acquire the independent RMG approach before occupying the STS through
    // lane. Waiting here after pickup can block the vehicle that must deliver
    // this container's support or release this same approach.
    if(!ReserveYardApproach(Vehicle)) { Stop(TEXT("STS activation lost yard approach admission")); return; }
    auto& Record=Manifest[Job.Cargo];
    Record.VehicleAssignedAt=SimulationTime;
    if(LaneCount==9)
    {
        if(!Equipment[YardCraneCount+STS]->AssignCargo(Record.Actor.Get(),Record.Actor->GetActorLocation(),
            TransferQuay(Job)+FVector(0,0,349.5f),false,false))
        { Stop(TEXT("STS could not start admitted cargo: ")+Equipment[YardCraneCount+STS]->Fault); return; }
        Equipment[YardCraneCount+STS]->SetDestinationReady(false);
        ++PrefetchedJobs; // STS pickup overlaps its assigned AGV's approach.
        UE_LOG(LogTemp,Display,TEXT("DISPATCH_ADMITTED: STS%d C%d AGV%d RMG%d queue_seconds=%.1f"),
            STS+1,Record.ID,Vehicles[Vehicle]->VehicleID,Job.RMG+1,SimulationTime-Record.DispatchEligibleAt);
    }
    Equipment[YardCraneCount+STS]->SetHandoverVehicle(Vehicles[Vehicle]);
    PreparedCargo[STS]=INDEX_NONE; PreparedStarted[STS]=false; STSOwners[STS]=Vehicle;
    if (FromQueue) { NextVehicles[STS]=INDEX_NONE; ++QueuedHandoffs; }
    if(!PlanRoadRoute(Vehicle,false)) Stop(TEXT("No road route to STS handover"));
}

void APortSiteLogistics::AssignReturnSTS(int32 Vehicle)
{
    if(LaneCount!=9 || !Vehicles[Vehicle]->CanAcceptWork()) return;
    auto& Job=Jobs[Vehicle];
    int32 Best=INDEX_NONE;
    double BestScore=TNumericLimits<double>::Max();
    for(int32 S=0;S<LaneCount;++S)
    {
        PrepareNextCargo(S);
        if(!Fault.IsEmpty()) return;
        if(PreparedCargo[S]==INDEX_NONE) continue;
        // A return vehicle must have executable work, rather than occupying
        // an approach bay while its yard support is still on a vessel.
        if(!CraneAvailable(YardCraneCount+S) || STSOwners[S]!=INDEX_NONE || Equipment[YardCraneCount+S]->IsBusy() ||
            !YardApproachAvailable(PreparedCargo[S])) continue;
        const int32 Queued=NextVehicles[S];
        // A reserved standby remains physical traffic even after it reaches
        // the STS bay. Never turn it into an idle vehicle on the road.
        if(Queued!=INDEX_NONE) continue;
        const FVector Target=CargoQuay(PreparedCargo[S]);
        const FVector From=Vehicles[Vehicle]->GetActorLocation();
        const FVector Queue=QueueQuay(S);
        const bool Far=Yard[Job.Slot].Half!=0;
        const double ReturnX=TerminalLayout::SiteCmX(Far?TerminalLayout::AGVInlandReturnX:14000.f);
        const double CrossY=Far?TerminalLayout::AGVReturnCrossY(TerminalLayout::AGVBerthGroup(S)):
            (bReuseFreedApron?AGVApronRoads::QueueAccessY(Queue.Y,From.Y):
                SiteLogistics::ReturnQueueAccessY(Queue.Y,From.Y));
        const double Score=(FMath::Abs(From.X-ReturnX)+FMath::Abs(From.Y-CrossY)+
            FMath::Abs(ReturnX-Queue.X)+FMath::Abs(Queue.Y-CrossY)+FVector::Dist2D(Queue,Target))/700.+30.;
        double IdleETA=TNumericLimits<double>::Max();
        for(int32 I=0;I<Vehicles.Num();++I) if(Jobs[I].Stage==0 && Vehicles[I]->CanAcceptWork())
            IdleETA=FMath::Min(IdleETA,AGVDispatchPolicy::EstimatedApproachDistanceCm(
                Vehicles[I]->GetActorLocation(),Target,S,bDepotParking?I:INDEX_NONE)/AGVDispatchPolicy::CruiseSpeedCmPerSecond);
        if(!AGVDispatchPolicy::UseDirectReturn(Score,IdleETA)) continue;
        if(Score<BestScore) { BestScore=Score; Best=S; }
    }
    if(Best==INDEX_NONE) return;
    Job.ReturnSTS=Best; Job.ReturnCargo=PreparedCargo[Best];
    NextVehicles[Best]=Vehicle;
    UE_LOG(LogTemp,Display,TEXT("AGV_DIRECT_RETURN: V%d -> STS%d C%d"),
        Vehicles[Vehicle]->VehicleID,Best+1,Manifest[Job.ReturnCargo].ID);
}

FVector APortSiteLogistics::ChargingBayPosition(int32 Bay) const
{
    return FleetPark(Bay);
}

void APortSiteLogistics::AdvanceFleetBatteries(float Dt)
{
    for(int32 I=0;I<Vehicles.Num();++I)
    {
        auto& Job=Jobs[I]; auto* Vehicle=Vehicles[I].Get();
        if(Vehicle->bDriveFault) continue;
        const bool AtCharger=Job.Stage==FSiteTransfer::Charging && Job.ChargingBay!=INDEX_NONE &&
            Vehicle->Speed<=.1f && Vehicle->PayloadKg<=.1f &&
            Vehicle->GetActorLocation().Equals(ChargingBayPosition(Job.ChargingBay),AGVReference::DockingAccuracyCm+1.f);
        Vehicle->AdvanceBattery(Dt,Job.Stage!=0 && Job.Stage!=FSiteTransfer::Charging,AtCharger);
        if(Job.Stage==FSiteTransfer::Charging && !Vehicle->bNeedsCharge)
        {
            // Already at its standby position: offer the fully charged AGV
            // to the dispatcher without an unnecessary empty return trip.
            Job=FSiteTransfer();
        }
        // An empty queued vehicle that became low on its previous return is
        // released before scheduling, so another charged AGV can take its place.
        if(Job.Stage==8 && !Vehicle->CanAcceptWork())
        {
            const int32 STS=Job.STS;
            if(NextVehicles[STS]==I) NextVehicles[STS]=INDEX_NONE;
            Job=FSiteTransfer(); Job.STS=STS; Job.Stage=FSiteTransfer::ReturnToStandby;
            if(!PlanRoadRoute(I,false)) { Stop(TEXT("No road route for low-battery queued AGV")); return; }
        }
    }
    for(int32 I=0;I<Vehicles.Num();++I)
    {
        if(Jobs[I].Stage!=0 || !Vehicles[I]->bNeedsCharge) continue;
        auto& Job=Jobs[I]; Job=FSiteTransfer(); Job.ChargingBay=I;
        const bool AtBay=Vehicles[I]->Speed<=.1f && Vehicles[I]->GetActorLocation().Equals(
            FleetPark(I),AGVReference::DockingAccuracyCm+1.f);
        Job.Stage=AtBay?FSiteTransfer::Charging:FSiteTransfer::ToCharge;
        if(AtBay) Vehicles[I]->AdvanceBattery(0,false,true);
        else if(!PlanRoadRoute(I,false)) { Stop(TEXT("No road route to charging bay")); return; }
    }
}

bool APortSiteLogistics::UpdateDispatchPriorities()
{
    if(!bReuseFreedApron || LaneCount!=9) return true;
    const int32 Limit=FMath::Min(DispatchLimit,Manifest.Num());
    if(DispatchPriorityLimit==Limit) return true;
    TArray<int32> Ranks; TMap<int32,int32> PlannedCargo;
    for(int32 I=0;I<Manifest.Num();++I)
    {
        Ranks.Add(Manifest[I].PlanningOrder);
        if(Manifest[I].PlannedSlot!=INDEX_NONE) PlannedCargo.Add(Manifest[I].PlannedSlot,I);
    }
    TArray<FIntPoint> Dependencies;
    for(int32 I=0;I<Manifest.Num();++I)
    {
        const int32 Planned=Manifest[I].PlannedSlot;
        if(!Yard.IsValidIndex(Planned)) continue;
        const auto& Slot=Yard[Planned];
        if(const int32* Parent=PlannedCargo.Find(Slot.Below)) Dependencies.Add(FIntPoint(*Parent,I));
        if(Slot.LengthFt==20 && Slot.Below!=INDEX_NONE && Yard.IsValidIndex(Slot.Partner))
            if(const int32* Parent=PlannedCargo.Find(Yard[Slot.Partner].Below)) Dependencies.Add(FIntPoint(*Parent,I));
    }
    TArray<AGVDispatchDependencies::FPriority> Priorities;
    if(!AGVDispatchDependencies::Build(Ranks,Dependencies,Limit,Priorities))
    { Stop(TEXT("Invalid yard support order for dispatch priority")); return false; }
    for(int32 I=0;I<Manifest.Num();++I)
    {
        Manifest[I].DispatchCriticalPath=Priorities[I].PathLength;
        Manifest[I].DispatchUnlocks=Priorities[I].Unlocks;
    }
    DispatchPriorityLimit=Limit;
    UE_LOG(LogTemp,Display,TEXT("AGV_DEPENDENCY_PRIORITY: active prefix=%d; yard support paths; age bonus capped at 960s"),Limit);
    return true;
}

void APortSiteLogistics::ScheduleFleet()
{
    PortSimPerformance::FScope Profile(TEXT("dispatch"));
    if(!UpdateDispatchPriorities()) return;
    auto Nearest=[&](FVector Target,int32 STS)->int32
    {
        // Reproduce the old three-row departure rule only for the parking A/B
        // baseline. Crane, cargo, battery and traffic controllers stay shared.
        for(int32 Pass=0;Pass<(bDepotParking?1:2);++Pass)
        {
        auto Eligible=[&](int32 I)
        {
            if(Jobs[I].Stage!=0 || !Vehicles[I]->CanAcceptWork()) return false;
            if(bDepotParking) return true;
            const int32 Pool=TerminalLayout::AGVBerthGroup(STS);
            const float Y=TerminalLayout::AGVParkY(I),Cross=TerminalLayout::AGVInboundCrossY(STS);
            if(Pass==0 && (TerminalLayout::AGVParkRow(I)!=Pool ||
                !(Pool<2?Y>=Cross+1800.f:Y<=Cross-1800.f))) return false;
            const int32 Bay=I%TerminalLayout::AGVParkBays;
            const int32 Gap=FMath::Min(Bay,TerminalLayout::AGVParkBays-2);
            return !Vehicles[I]->GetActorLocation().Equals(FleetPark(I),AGVReference::DockingAccuracyCm+1.f) ||
                FMath::Abs(TerminalLayout::AGVParkCrossingY(Gap)-Target.Y)>=
                    AGVReference::MinimumInnerTurnRadiusCm+100.f;
        };
        int32 MinimumJobs=MAX_int32;
        for(int32 I=0;I<Vehicles.Num();++I)
            if(Eligible(I))
                MinimumJobs=FMath::Min(MinimumJobs,Vehicles[I]->CompletedJobs);
        int32 Best=INDEX_NONE; double BestScore=TNumericLimits<double>::Max();
        for(int32 I=0;I<Vehicles.Num();++I)
        {
            if(!Eligible(I)) continue;
            const double Score=AGVDispatchPolicy::Score(Vehicles[I]->GetActorLocation(),Target,STS,
                Vehicles[I]->CompletedJobs,MinimumJobs,bDepotParking?I:INDEX_NONE);
            if(Best==INDEX_NONE || Score<BestScore ||
                (FMath::IsNearlyEqual(Score,BestScore) && Vehicles[I]->VehicleID<Vehicles[Best]->VehicleID))
            { Best=I; BestScore=Score; }
        }
        if(Best!=INDEX_NONE) return Best;
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
        const bool ReadyA=STSOwners[A]==INDEX_NONE && CargoA!=INDEX_NONE && !Equipment[YardCraneCount+A]->IsBusy();
        const bool ReadyB=STSOwners[B]==INDEX_NONE && CargoB!=INDEX_NONE && !Equipment[YardCraneCount+B]->IsBusy();
        if(ReadyA!=ReadyB) return ReadyA;
        if(ReadyA)
        {
            const auto& CA=Manifest[CargoA]; const auto& CB=Manifest[CargoB];
            const double PA=AGVDispatchPolicy::ReadyPriority(CA.DispatchEligibleAt,CA.DispatchCriticalPath);
            const double PB=AGVDispatchPolicy::ReadyPriority(CB.DispatchEligibleAt,CB.DispatchCriticalPath);
            if(PA!=PB) return PA>PB;
            if(CA.DispatchUnlocks!=CB.DispatchUnlocks) return CA.DispatchUnlocks>CB.DispatchUnlocks;
        }
        if(ReadyA && Manifest[CargoA].DispatchEligibleAt!=Manifest[CargoB].DispatchEligibleAt)
            return Manifest[CargoA].DispatchEligibleAt<Manifest[CargoB].DispatchEligibleAt;
        if(ReadyA && Manifest[CargoA].PlanningOrder/LaneCount!=Manifest[CargoB].PlanningOrder/LaneCount)
            return Manifest[CargoA].PlanningOrder<Manifest[CargoB].PlanningOrder;
        // Interleave south/central/north on equal-age work so all three main
        // corridors receive traffic instead of filling one vessel end first.
        const int32 InterleaveA=(A%3)*3+A/3,InterleaveB=(B%3)*3+B/3;
        return InterleaveA<InterleaveB;
    });
    for (int32 S:ReadySTS)
    {
        // Earlier assignments in this same pass may claim the selected RMG.
        // Reconsider another executable box instead of lifting the blocked one.
        PrepareNextCargo(S);
        if (!CraneAvailable(YardCraneCount+S) || STSOwners[S]!=INDEX_NONE || PreparedCargo[S]==INDEX_NONE || Equipment[YardCraneCount+S]->IsBusy() ||
            !YardApproachAvailable(PreparedCargo[S])) continue;
        if (NextVehicles[S]!=INDEX_NONE)
        {
            if (Jobs[NextVehicles[S]].Stage==8 && Vehicles[NextVehicles[S]]->CanAcceptWork()) ActivateVehicle(NextVehicles[S],S,true);
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
    Job.RouteStart=Vehicles[Lane]->GetActorLocation(); Job.bRouteStartValid=true;
    const FVector Quay=TransferQuay(Job);
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
    return YardAdmissionReason(Cargo).IsEmpty();
}

FString APortSiteLogistics::YardAdmissionReason(int32 Cargo,int32 Vehicle) const
{
    if(!Manifest.IsValidIndex(Cargo)) return TEXT("no_cargo");
    const int32 Planned=Manifest[Cargo].PlannedSlot;
    if(!Yard.IsValidIndex(Planned)) return TEXT("invalid_yard_slot");
    const auto& Slot=Yard[Planned];
    if(!CraneAvailable(Slot.Crane)) return TEXT("rmg_unavailable");
    const bool OtherPadSupported=Slot.LengthFt!=20 || Slot.Below==INDEX_NONE ||
        (Yard.IsValidIndex(Slot.Partner) && Yard[Slot.Partner].Below!=INDEX_NONE &&
            Yard[Yard[Slot.Partner].Below].Occupied);
    const bool OwnSlot=Jobs.IsValidIndex(Vehicle) && Jobs[Vehicle].Slot==Planned;
    if(!Slot.Reserved || Slot.Central || Slot.Occupied || (SlotAssigned[Planned] && !OwnSlot) ||
        (LaneCount==9 && Slot.LengthFt!=Manifest[Cargo].Actor->LengthFt)) return TEXT("yard_slot_unavailable");
    if((Slot.Below!=INDEX_NONE && !Yard[Slot.Below].Occupied) || (LaneCount==9 && !OtherPadSupported))
        return TEXT("yard_support_pending");
    const int32 ApproachOwner=YardApproachOwners[YardApproachKey(Slot)];
    if(ApproachOwner!=INDEX_NONE && ApproachOwner!=Vehicle) return TEXT("rmg_approach_reserved");
    if(BlocksBusy[Slot.Block] || (CentralPending!=INDEX_NONE && Yard[CentralSlots[CentralPending]].Block==Slot.Block))
        return TEXT("yard_block_reserved");
    for(const auto& Lease:DeferredYardLeases)
        if(FVector::Dist2D(YardHandover(Slot),YardHandover(Yard[Lease.Slot]))<3000.) return TEXT("rmg_turn_clearance");
    // Adjacent handover aisles can share the turning envelope even though
    // their RMGs are independent. Claim that space before leaving the STS,
    // rather than admitting one vehicle into another's docking/exit turn.
    const FVector Dock=YardHandover(Slot);
    for(const auto& Other:Jobs)
    {
        if(Jobs.IsValidIndex(Vehicle) && &Other==&Jobs[Vehicle]) continue;
        if(!Yard.IsValidIndex(Other.Slot) || Other.bYardReleased) continue;
        const auto& OtherSlot=Yard[Other.Slot];
        const bool Approaching=YardApproachOwners[YardApproachKey(OtherSlot)]!=INDEX_NONE;
        if((Approaching || Other.bRMGReserved) &&
            FVector::Dist2D(Dock,YardHandover(OtherSlot))<3000.f) return TEXT("rmg_turn_clearance");
    }
    return FString();
}

bool APortSiteLogistics::ReserveYardApproach(int32 Lane)
{
    auto& Job=Jobs[Lane];
    const int32 Planned=Manifest[Job.Cargo].PlannedSlot;
    if(!Yard.IsValidIndex(Planned)) { Stop(TEXT("Manifest has no valid planned yard slot")); return false; }
    const auto& Slot=Yard[Planned];
    if(Job.Slot==Planned && YardApproachOwners[YardApproachKey(Slot)]==Lane) return true;
    // Separate near/far RMG entry lanes; never queue an upper box before its support.
    if(!YardAdmissionReason(Job.Cargo,Lane).IsEmpty()) return false;
    Job.Slot=Planned; Job.RMG=Slot.Crane;
    SlotAssigned[Planned]=true; YardApproachOwners[YardApproachKey(Slot)]=Lane;
    return true;
}

void APortSiteLogistics::ReleaseYardApproachIfClear(int32 Lane)
{
    auto& Job=Jobs[Lane];
    if(Job.Stage!=5 || Job.bYardReleased || !Yard.IsValidIndex(Job.Slot)) return;
    const auto* Vehicle=Vehicles[Lane].Get();
    const auto& Slot=Yard[Job.Slot];
    const FVector Dock=YardHandover(Slot),Position=Vehicle->GetActorLocation();
    if(FVector::Dist2D(Position,Dock)<=3000.f) return;
    if(LaneCount==9)
    {
        // Far-RMG access is a long two-way aisle, not just the docking pad.
        // Releasing it 30 m from the pad admitted a loaded AGV head-on into
        // the empty vehicle still returning along the same 200 m road.
        const float RoadX=TerminalLayout::SiteCmX(Slot.Half?
            TerminalLayout::AGVInlandReturnX:12500.f);
        if(AGVTrafficGeometry::SweptChassisIntersects(Dock,FVector(RoadX,Dock.Y,0),
            Position,Vehicle->GetActorRightVector(),730,210,170)) return;
    }
    RMGBusy[Job.RMG]=false;
    const int32 Key=YardApproachKey(Slot);
    if(YardApproachOwners[Key]==Lane) YardApproachOwners[Key]=INDEX_NONE;
    Job.bYardReleased=true;
}

bool APortSiteLogistics::ReserveYard(int32 Lane)
{
    auto& Job=Jobs[Lane];
    const int32 Planned=Manifest[Job.Cargo].PlannedSlot;
    if(!Yard.IsValidIndex(Planned)) { Stop(TEXT("Manifest has no valid planned yard slot")); return false; }
    const auto& Slot=Yard[Planned]; const int32 RMG=Slot.Crane;
    if(LaneCount==9 && (Slot.LengthFt!=Manifest[Job.Cargo].Actor->LengthFt ||
        (Slot.LengthFt==20 && Slot.Below!=INDEX_NONE &&
            (!Yard.IsValidIndex(Slot.Partner) || Yard[Slot.Partner].Below==INDEX_NONE ||
                !Yard[Yard[Slot.Partner].Below].Occupied)))) return false;
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
    if(LaneCount!=9) return PlanLegacyYardDestinations();
    TArray<int32> Occupant,PerRMG,Order;
    TArray<float> AboveKg;
    Occupant.Init(INDEX_NONE,Yard.Num()); AboveKg.Init(0,Yard.Num()); PerRMG.Init(0,YardCraneCount);
    int32 MaxBatch=0;
    for(auto& C:Manifest) { C.PlannedPair=INDEX_NONE; C.EstimatedPairSavingSeconds=0; MaxBatch=FMath::Max(MaxBatch,C.Wave); }
    for(int32 Batch=0;Batch<=MaxBatch;++Batch)
    {
        TArray<TArray<int32>> Streams; Streams.SetNum(LaneCount);
        int32 Longest=0;
        for(int32 I=0;I<Manifest.Num();++I) if(Manifest[I].Wave==Batch)
        { Streams[Manifest[I].STS].Add(I); Longest=FMath::Max(Longest,Streams[Manifest[I].STS].Num()); }
        for(int32 N=0;N<Longest;++N) for(int32 Lane=0;Lane<LaneCount;++Lane)
            if(Streams[Lane].IsValidIndex(N)) Order.Add(Streams[Lane][N]);
    }
    for(int32 Rank=0;Rank<Order.Num();++Rank) Manifest[Order[Rank]].PlanningOrder=Rank;
    auto Assign=[&](int32 CargoIndex,int32 RequiredCrane,int32 RequiredSlot=INDEX_NONE)->bool
    {
        const auto* C=Manifest[CargoIndex].Actor.Get();
        if(!C || !C->CargoInformation.Valid()) return false;
        int32 Best=INDEX_NONE; double BestCost=TNumericLimits<double>::Max();
        for(int32 I=0;I<Yard.Num();++I)
        {
            const auto& S=Yard[I];
            if(!S.Reserved || S.Central || Occupant[I]!=INDEX_NONE ||
                S.LengthFt!=C->LengthFt || (RequiredSlot!=INDEX_NONE && I!=RequiredSlot) ||
                (RequiredCrane!=INDEX_NONE && S.Crane!=RequiredCrane) ||
                !YardStackingPolicy::ZoneAccepts(S.Block,C->CargoInformation)) continue;
            // Earliest bookings get the southern exit bands; special handling
            // zones remain hard constraints, and cannot be bought by a distance score.
            const auto& Info=C->CargoInformation;
            if(Info.HazardGroup==TEXT("none") && Info.HandlingType==TEXT("dry"))
            {
                const int32 Band=S.Block<5?1:S.Block<10?2:S.Block<14?3:4;
                if(Band!=Info.OutboundOrder) continue;
            }
            if(S.Below!=INDEX_NONE)
            {
                if(Occupant[S.Below]==INDEX_NONE) continue;
                if(S.LengthFt==20)
                {
                    if(!Yard.IsValidIndex(S.Partner)) continue;
                    const int32 OtherBelow=Yard[S.Partner].Below;
                    if(OtherBelow==INDEX_NONE || Occupant[OtherBelow]==INDEX_NONE ||
                        Manifest[Occupant[OtherBelow]].PlanningOrder>=Manifest[CargoIndex].PlanningOrder) continue;
                }
                // Pair candidates may be assigned ahead of their normal
                // discharge turn. They cannot support an earlier booking:
                // that can create a cycle across the nine STS streams if the
                // pair is later declined or its second box is not accessible.
                if(Manifest[Occupant[S.Below]].PlanningOrder>=Manifest[CargoIndex].PlanningOrder) continue;
                const auto* Lower=Manifest[Occupant[S.Below]].Actor.Get();
                if(!YardStackingPolicy::MayStack(Lower->CargoInformation,Lower->LengthFt,Lower->MassKg,
                    Info,C->LengthFt,C->MassKg)) continue;
            }
            bool LoadAllowed=true;
            for(int32 Below=S.Below;Below!=INDEX_NONE;Below=Yard[Below].Below)
            {
                if(Occupant[Below]==INDEX_NONE || AboveKg[Below]+C->MassKg>
                    Manifest[Occupant[Below]].Actor->CargoInformation.PermittedSuperimposedKg)
                { LoadAllowed=false; break; }
            }
            if(!LoadAllowed) continue;
            const double Travel=(FVector::Dist2D(CargoQuay(CargoIndex),YardHandover(S))+
                FVector::Dist2D(Equipment[S.Crane]->HeadPosition(),S.Position))/AGVDispatchPolicy::CruiseSpeedCmPerSecond;
            // Prefer compatible stacks, but balance cranes rather than forcing
            // all compatible bookings through the same RMG.
            const bool Completing20Cell=S.LengthFt==20 && Yard.IsValidIndex(S.Partner) && Occupant[S.Partner]!=INDEX_NONE;
            const double Cost=Travel+PerRMG[S.Crane]*240.+(S.Below==INDEX_NONE?600.:0.)-(Completing20Cell?900.:0.);
            if(Cost<BestCost) { Best=I; BestCost=Cost; }
        }
        if(Best==INDEX_NONE) return false;
        Manifest[CargoIndex].PlannedSlot=Best; Occupant[Best]=CargoIndex; ++PerRMG[Yard[Best].Crane];
        for(int32 Below=Yard[Best].Below;Below!=INDEX_NONE;Below=Yard[Below].Below) AboveKg[Below]+=C->MassKg;
        return true;
    };
    for(int32 CargoIndex:Order)
    {
        if(Manifest[CargoIndex].PlannedSlot!=INDEX_NONE) continue;
        if(!Assign(CargoIndex,INDEX_NONE)) return false;
        if(Manifest[CargoIndex].Actor->LengthFt!=20) continue;
        const auto& First=Manifest[CargoIndex];
        const int32 FirstSlot=First.PlannedSlot;
        const int32 MateSlot=Yard[FirstSlot].Partner;
        if(!Yard.IsValidIndex(MateSlot) || Occupant[MateSlot]!=INDEX_NONE) continue;
        TArray<FVector2D> ThroughPaths;
        for(int32 OtherSTS=0;OtherSTS<LaneCount;++OtherSTS)
            if(OtherSTS!=First.STS && Manifest.ContainsByPredicate([OtherSTS](const FSiteShipCargo& C){return C.STS==OtherSTS;}))
                ThroughPaths.Add(FVector2D(QuayPark(OtherSTS).Y,TerminalLayout::AGVLoadedCrossY(OtherSTS)));
        if(!YardStackingPolicy::DockOutsideThroughPaths(CargoQuay(CargoIndex).Y,ThroughPaths)) continue;
        // Only select another box from the same visible source tier and STS.
        // The boxes are lifted separately with four verified locks each.
        for(int32 Mate:Order)
        {
            if(Mate==CargoIndex || Manifest[Mate].PlannedSlot!=INDEX_NONE) continue;
            const auto& Other=Manifest[Mate];
            if(Other.STS!=First.STS || Other.Wave!=First.Wave ||
                !FMath::IsNearlyEqual(Other.Transform.GetLocation().Z,First.Transform.GetLocation().Z,.1) ||
                Other.Actor->LengthFt!=20 || !YardStackingPolicy::PairBookingsCompatible(
                    First.Actor->CargoInformation,Other.Actor->CargoInformation) ||
                !YardStackingPolicy::PairFits(First.Actor->LengthFt,Other.Actor->LengthFt,
                    First.Actor->ExternalDimensionsCm.Y,Other.Actor->ExternalDimensionsCm.Y,
                    First.Actor->MassKg,Other.Actor->MassKg,AGVReference::MaximumPayloadKg)) continue;
            const FVector Q=CargoQuay(CargoIndex),D=YardHandover(Yard[FirstSlot]);
            const float Crossing=TerminalLayout::AGVLoadedCrossY(First.STS);
            const bool Far=Yard[FirstSlot].Half!=0;
            const float InboundX=TerminalLayout::SiteCmX(Far?TerminalLayout::AGVInlandInboundX:12500.f);
            const float ReturnX=TerminalLayout::SiteCmX(Far?TerminalLayout::AGVInlandReturnX:14000.f);
            const FVector Queue=QueueQuay(First.STS);
            const float ReturnY=Far?TerminalLayout::AGVReturnCrossY(TerminalLayout::AGVBerthGroup(First.STS)):
                SiteLogistics::ReturnQueueAccessY(Queue.Y,D.Y);
            const double LoadedDistance=FMath::Abs(Q.Y-Crossing)+FMath::Abs(D.Y-Crossing)+
                FMath::Abs(InboundX-Q.X)+FMath::Abs(D.X-InboundX);
            const double ReturnDistance=FMath::Abs(D.Y-ReturnY)+FMath::Abs(Queue.Y-ReturnY)+
                FMath::Abs(ReturnX-D.X)+FMath::Abs(Queue.X-ReturnX)+FMath::Abs(Queue.X-Q.X)+FMath::Abs(Queue.Y-Q.Y);
            const double Saving=YardStackingPolicy::PairedTripSavingSeconds(LoadedDistance/600.,
                ReturnDistance/700.,150.,15.);
            if(Saving<YardStackingPolicy::MinimumPairSavingSeconds) break;
            if(!Assign(Mate,Yard[FirstSlot].Crane,MateSlot)) continue;
            if(!YardHandover(Yard[FirstSlot]).Equals(YardHandover(Yard[Manifest[Mate].PlannedSlot]),.1))
                return false;
            Manifest[CargoIndex].PlannedPair=Mate; Manifest[Mate].PlannedPair=CargoIndex;
            Manifest[CargoIndex].EstimatedPairSavingSeconds=Manifest[Mate].EstimatedPairSavingSeconds=Saving;
            Manifest[CargoIndex].PlannedPairSavingSeconds=Manifest[Mate].PlannedPairSavingSeconds=Saving;
            break;
        }
    }
    UE_LOG(LogTemp,Display,TEXT("YARD_BOOKING_PLAN: %d containers, outbound/type/hazard/size/weight constraints"),Manifest.Num());
    return true;
}

bool APortSiteLogistics::PlanLegacyYardDestinations()
{
    TSet<int32> PlannedSlots;
    TArray<int32> PlannedPerRMG; PlannedPerRMG.Init(0,YardCraneCount);
    TArray<int32> PlanningOrder;
    int32 MaxBatch=0;
    for(const auto& Cargo:Manifest) MaxBatch=FMath::Max(MaxBatch,Cargo.Wave);
    // Complete each hold batch before assigning supporting tiers to the next.
    for(int32 Batch=0;Batch<=MaxBatch;++Batch)
    {
        TArray<TArray<int32>> CargoBySTS; CargoBySTS.SetNum(LaneCount);
        int32 Longest=0;
        for(int32 I=0;I<Manifest.Num();++I) if(Manifest[I].Wave==Batch)
        { CargoBySTS[Manifest[I].STS].Add(I); Longest=FMath::Max(Longest,CargoBySTS[Manifest[I].STS].Num()); }
        for(int32 Wave=0;Wave<Longest;++Wave)
            for(int32 STS=0;STS<LaneCount;++STS)
                if(CargoBySTS[STS].IsValidIndex(Wave)) PlanningOrder.Add(CargoBySTS[STS][Wave]);
    }
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
    PortSimPerformance::FScope Profile(TEXT("road_planning"));
    auto& Job=Jobs[Lane]; auto* Vehicle=Vehicles[Lane].Get();
    // Do not replace a valid route while the chassis is midway through a turn.
    // A graph path from a point has no knowledge of its non-cardinal heading.
    if(Congested && !Job.Route.IsEmpty() &&
        FMath::Abs(Vehicle->GetActorRightVector().X)<.999f && FMath::Abs(Vehicle->GetActorRightVector().Y)<.999f) return false;
    if(Job.Stage!=3 && Job.Stage!=5 && Job.Stage!=6 && Job.Stage!=7 &&
        Job.Stage!=FSiteTransfer::ToCharge && Job.Stage!=FSiteTransfer::ReturnToStandby) return false;
    if((Job.Stage==3 || Job.Stage==5) && !Yard.IsValidIndex(Job.Slot)) return false;
    FAGVRoadNetwork Network;
    Network.RestrictNarrowCrossing(TerminalLayout::AGVCentralInboundY,TerminalLayout::AGVCentralReturnY);
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
    // Reuse the freed parking apron for loaded transport. The historical
    // mode retains each vessel's shared quay departure to its nearest crossing.
    const bool ApronLoaded=bReuseFreedApron && Job.Stage==3;
    const bool FarYard=(Job.Stage==3 || Job.Stage==5) && Yard[Job.Slot].Half!=0;
    const float LoadedCrossY=bReuseFreedApron && TerminalLayout::AGVBerthGroup(Job.STS)==1?
        TerminalLayout::AGVCentralInboundY:TerminalLayout::AGVLoadedCrossY(Job.STS);
    const float ApronDestinationY=ApronLoaded?(FarYard?LoadedCrossY:YardHandover(Yard[Job.Slot]).Y):0.f;
    float ApronSpine=0.f;
    if(ApronLoaded)
    {
        AGVApronRoads::Add(Network);
        ApronSpine=AGVApronRoads::AddDeparture(Network,TransferQuay(Job),ApronDestinationY);
    }
    else if(Job.Stage==3)
    {
        // A busy departure never reverses along the single loaded quay lane
        // to reach another crossing. That put an AGV behind its own STS and
        // formed a wait cycle with the next returning AGV's docking corridor.
        Network.Add(P(QuayLane,TransferQuay(Job).Y),
            P(QuayLane,TerminalLayout::AGVLoadedCrossY(Job.STS)));
    }
    else Network.Add(P(QuayLane,North),P(QuayLane,South));
    // Historical parking connectors below retain their original turn gaps;
    // the apron mode supplies separate loaded lanes with full turn tangents.

    // South, central and north roads are real alternatives in the graph.  The
    // shortest route (plus live reservation cost during rerouting) selects one.
    Network.Add(P(QuayLane,TerminalLayout::AGVSouthInboundY),
        P(TerminalLayout::AGVInlandInboundX,TerminalLayout::AGVSouthInboundY));
    Network.Add(P(TerminalLayout::AGVInlandReturnX,TerminalLayout::AGVSouthReturnY),
        P(QuayLane,TerminalLayout::AGVSouthReturnY));
    // Depot mode converts the old parking area into road capacity. Keep the
    // parked-row jog only for the historical comparison with occupied rows.
    Network.Add(P(bReuseFreedApron?(ApronLoaded?TerminalLayout::AGVApronNorthX:QuayLane):15300.f,TerminalLayout::AGVCentralInboundY),
        P(TerminalLayout::AGVInlandInboundX,TerminalLayout::AGVCentralInboundY));
    Network.Add(P(TerminalLayout::AGVInlandReturnX,TerminalLayout::AGVCentralReturnY),
        P(bReuseFreedApron?QuayLane:16700.f,TerminalLayout::AGVCentralReturnY));
    constexpr int32 CentralInboundGap=8;
    constexpr int32 CentralReturnGap=9;
    if(!bReuseFreedApron)
    {
        Network.Add(P(QuayLane,TerminalLayout::AGVParkCrossingY(CentralInboundGap)),
            P(15300,TerminalLayout::AGVParkCrossingY(CentralInboundGap)));
        Network.Add(P(16700,TerminalLayout::AGVParkCrossingY(CentralReturnGap)),
            P(QuayLane,TerminalLayout::AGVParkCrossingY(CentralReturnGap)));
    }
    Network.Add(P(QuayLane,TerminalLayout::AGVNorthInboundY),
        P(TerminalLayout::AGVInlandReturnX,TerminalLayout::AGVNorthInboundY));
    Network.Add(P(TerminalLayout::AGVInlandReturnX,TerminalLayout::AGVNorthReturnY),
        P(QuayLane,TerminalLayout::AGVNorthReturnY));

    const bool ChargingTrip=Job.Stage==FSiteTransfer::ToCharge || Job.Stage==FSiteTransfer::ReturnToStandby;
    const FVector Park=FleetPark(Lane);
    const float DepotEntry=TerminalLayout::AGVStandbyEntryY(Lane);
    const float DepotExit=TerminalLayout::AGVStandbyExitY(Lane);
    const int32 Bay=Lane%TerminalLayout::AGVParkBays;
    const int32 DepartureGap=FMath::Min(Bay,TerminalLayout::AGVParkBays-2),ReturnGap=FMath::Max(0,Bay-1);
    if(bDepotParking)
    {
        Network.Add(P(12500,North),P(12500,TerminalLayout::AGVStandbyNorthY));
        Network.Add(P(14000,TerminalLayout::AGVStandbyNorthY),P(14000,North));
        Network.Add(P(12500,DepotEntry),P(4800,DepotEntry));
        Network.Add(P(4800,DepotExit),P(14000,DepotExit));
        Network.Add(FVector(Park.X,DepotEntry,0),Park,true);
        Network.Add(Park,FVector(Park.X,DepotExit,0));
    }
    else
    {
        auto Crossing=[&](int32 Gap)
        {
            const float Y=TerminalLayout::AGVParkCrossingY(Gap);
            if(TerminalLayout::AGVParkCrossingToYard(Gap)) Network.Add(P(QuayLane,Y),P(16700,Y));
            else Network.Add(P(16700,Y),P(QuayLane,Y));
        };
        if(Job.Stage==6 || Job.Stage==7) Crossing(DepartureGap);
        if(Job.Stage==5 && Job.ReturnSTS==INDEX_NONE) Crossing(ReturnGap);
        Network.Add(Park,FVector(Park.X,TerminalLayout::AGVParkCrossingY(DepartureGap),0));
        Network.Add(FVector(Park.X,TerminalLayout::AGVParkCrossingY(ReturnGap),0),Park);
    }

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
        if(bReuseFreedApron)
            return Above?AGVApronRoads::QueueAccessY(BayY,
                Job.Stage==5?YardHandover(Yard[Job.Slot]).Y:BayY):BayY-1600.f;
        if(Above && Job.Stage==5 && Job.ReturnSTS!=INDEX_NONE && Yard[Job.Slot].Half==0)
            return SiteLogistics::ReturnQueueAccessY(BayY,YardHandover(Yard[Job.Slot]).Y);
        float Best=Above?TerminalLayout::AGVParkY(TerminalLayout::AGVParkBays-1)+3000.f:
            TerminalLayout::AGVSouthReturnY;
        if(Above && Job.Stage==5 && Job.ReturnSTS!=INDEX_NONE && Yard[Job.Slot].Half==0 &&
            FMath::Abs(Best-YardHandover(Yard[Job.Slot]).Y)<
                2*AGVReference::MinimumInnerTurnRadiusCm+100.f) Best+=2500.f;
        double Distance=TNumericLimits<double>::Max();
        for(int32 Gap=1;Gap<TerminalLayout::AGVParkBays-1;Gap+=2)
        {
            const float Y=TerminalLayout::AGVParkCrossingY(Gap);
            if((Above?Y-BayY:BayY-Y)<2*AGVReference::MinimumInnerTurnRadiusCm+100.f) continue;
            if(Job.Stage==5 && Job.ReturnSTS!=INDEX_NONE && Yard[Job.Slot].Half==0 &&
                FMath::Abs(Y-YardHandover(Yard[Job.Slot]).Y)<
                    2*AGVReference::MinimumInnerTurnRadiusCm+100.f) continue;
            const double Candidate=FMath::Abs(Y-BayY);
            if(Candidate<Distance) { Best=Y; Distance=Candidate; }
        }
        return Best;
    };
    FVector Quay=QueueQuay(QueueSTS);
    const float QueueAccessY=ClearParkingGap(Quay.Y,true);
    const bool NeedsQueueRoad=Job.Stage==7 || (Job.Stage==5 && Job.ReturnSTS!=INDEX_NONE) ||
        (Job.Stage==6 && !Vehicle->GetActorLocation().Equals(Park,AGVReference::DockingAccuracyCm+1.f));
    if(NeedsQueueRoad)
    {
        const float ApproachY=QueueAccessY;
        const FVector QueueApproach(Quay.X,ApproachY,0);
        WestAccess(ApproachY);
        Network.Add(P(QuayLane,ApproachY),QueueApproach);
        Network.Add(QueueApproach,Quay,true);
    }
    if(Job.Stage!=7 && !ChargingTrip)
    {
        const int32 DestinationCargo=Job.Stage==5 && Job.ReturnCargo!=INDEX_NONE?Job.ReturnCargo:Job.Cargo;
        if(Job.ExportIndex==INDEX_NONE && !Manifest.IsValidIndex(DestinationCargo)) return false;
        Quay=Job.ExportIndex!=INDEX_NONE?TransferQuay(Job):CargoQuay(DestinationCargo);
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
    if(Job.Stage==FSiteTransfer::ReturnToStandby && Job.STS!=INDEX_NONE)
    {
        // Depart the off-spine STS waiting bay longitudinally, then merge west.
        const FVector Queue=Job.ExportIndex!=INDEX_NONE?TransferQuay(Job):QueueQuay(Job.STS),Approach(Queue.X,QueueAccessY,0);
        Network.Add(Queue,Approach);
        Network.Add(Approach,P(QuayLane,QueueAccessY));
        Network.Add(P(QuayLane,QueueAccessY),P(QuayLane,South));
    }
    if(!bDepotParking && Job.Stage==5 && Job.ReturnSTS==INDEX_NONE &&
        TerminalLayout::AGVParkCrossingToYard(ReturnGap))
        WestAccess(ClearParkingGap(TerminalLayout::AGVParkCrossingY(ReturnGap),true));
    FVector Goal=Job.Stage==FSiteTransfer::ToCharge?ChargingBayPosition(Job.ChargingBay):
        (Job.Stage==FSiteTransfer::ReturnToStandby?Park:Quay);
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
                Network.Add(Entry,P(TerminalLayout::AGVInlandInboundX,Entry.Y));
        }
        else
        {
            if(ApronLoaded)
                Network.Add(P(ApronSpine,Entry.Y),Entry);
            Network.Add(P(12500,Entry.Y),Entry);
            Network.Add(P(14000,Entry.Y),Entry);
            Network.Add(Dock,Entry);
            Network.Add(Entry,P(14000,Entry.Y));
            Network.Add(Exit,P(15300,Exit.Y));
            // The x=167 m spine is only 6.2 m from this stop, shorter than
            // R + the tangent clearance. A car cannot finish that 90-degree
            // turn before docking. Enter from the x=125/140 m spines instead.
            Network.Add(Exit,P(14000,Exit.Y));
            Network.Add(Exit,P(16700,Exit.Y));
            if(Job.Stage==5)
            {
                // Both directed spines remain reachable. The live return search
                // chooses a corridor before committing the physical route.
                Network.Add(Entry,P(15300.f,Entry.Y));
            }
        }
        Goal=Job.Stage==3?Entry:(Job.ReturnSTS!=INDEX_NONE?QueueQuay(Job.ReturnSTS):Park);
    }

    // A loaded recovery starts at its old, verified RMG entry/docking anchor.
    // Retain that site's real exit roads when building the NEW destination graph.
    FVector RecoveryDock=FVector::ZeroVector,RecoveryEntry=FVector::ZeroVector;
    if(Job.Stage==3 && Yard.IsValidIndex(Job.RecoveryOriginSlot))
    {
        const auto& Old=Yard[Job.RecoveryOriginSlot]; RecoveryDock=YardHandover(Old);
        RecoveryEntry=RecoveryDock+FVector(Old.Half?1800.f:-1800.f,0,0);
        Network.Add(RecoveryDock,RecoveryEntry);
        for(float X:Old.Half?TArray<float>{TerminalLayout::AGVInlandInboundX,TerminalLayout::AGVInlandReturnX}:TArray<float>{12500.f,14000.f,15300.f,16700.f})
            Network.Add(RecoveryEntry,P(X,RecoveryEntry.Y));
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
    if(ChargingTrip && Job.ChargingBay!=INDEX_NONE) ConnectDockingTolerance(ChargingBayPosition(Job.ChargingBay));
    ConnectDockingTolerance(QueueQuay(QueueSTS));
    // Export unloading finishes at the STS transfer pad, not its waiting bay.
    // Reconnect the measured docking pose before planning the depot return.
    if(Job.Stage==FSiteTransfer::ReturnToStandby && Job.ExportIndex!=INDEX_NONE)
        ConnectDockingTolerance(TransferQuay(Job));
    if(Job.Stage==3 || Job.Stage==5) ConnectDockingTolerance(Dock);
    if(Job.Stage==3 && Yard.IsValidIndex(Job.RecoveryOriginSlot))
    { ConnectDockingTolerance(RecoveryDock); ConnectDockingTolerance(RecoveryEntry); }

    const int32 ID=Vehicle->VehicleID;
    auto Cost=[&](FVector A,FVector B)
    {
        if(!Congested && Job.Stage!=5) return 0.;
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
    const bool PastDeparture=Job.Stage==3 && Start.X>
        TerminalLayout::SiteCmX(TerminalLayout::AGVQuayOutboundLaneX)+
            2*AGVReference::MinimumInnerTurnRadiusCm;
    const bool PastReturn=Job.Stage==5 && Yard[Job.Slot].Half!=0 &&
        Start.X<TerminalLayout::SiteCmX(18000.f);
    if((Congested && (PastDeparture || PastReturn)) || (Job.Stage==3 && Job.RecoveryOriginSlot!=INDEX_NONE))
    {
        // A completed crossing is not a new destination for rerouting. Requiring
        // it again sent partially completed jobs around the terminal a second time.
        SelectedReturnCorridor=Job.ReturnCorridor;
        RouteFound=Network.Find(Start,Goal,Candidate,Cost);
    }
    else if(Job.Stage==5)
    {
        const bool Far=Yard[Job.Slot].Half!=0;
        const bool DirectNear=!Far && Job.ReturnSTS!=INDEX_NONE;
        double BestCost=TNumericLimits<double>::Max();
        FVector CurrentMotion=FVector::ZeroVector;
        if(Congested && Job.Route.IsValidIndex(Job.Waypoint))
            CurrentMotion=Vehicle->PlannedMotionDirection(Job.Route[Job.Waypoint]);
        for(int32 Corridor=0;Corridor<TerminalLayout::AGVReturnCorridorCount;++Corridor)
        {
            if(DirectNear && Corridor!=TerminalLayout::AGVBerthGroup(Job.ReturnSTS)) continue;
            const float CrossY=DirectNear?QueueAccessY:
                !Far && Corridor==1 && !bReuseFreedApron?TerminalLayout::AGVParkCrossingY(CentralReturnGap):
                TerminalLayout::AGVReturnCrossY(Corridor);
            const float TurnClearance=2*AGVReference::MinimumInnerTurnRadiusCm+100.f;
            if(FMath::Abs(CrossY-Dock.Y)<TurnClearance) continue;
            // A reroute must continue in the current one-way spine direction;
            // going back to a crossing behind it requires an illegal U-turn.
            if(FMath::Abs(CurrentMotion.Y)>.999f &&
                (CrossY-Start.Y)*CurrentMotion.Y<TurnClearance) continue;
            const float ExitSpine=Far?(Dock.Y<CrossY?TerminalLayout::AGVInlandInboundX:
                TerminalLayout::AGVInlandReturnX):(Dock.Y<CrossY?15300.f:14000.f);
            const FVector Via=P(ExitSpine,CrossY);
            TArray<FVector> ToCorridor,FromCorridor;
            if(!Network.Find(Start,Via,ToCorridor,Cost) || !Network.Find(Via,Goal,FromCorridor,Cost)) continue;
            ToCorridor.Append(FromCorridor);
            double Score=0; FVector From=Start;
            for(const FVector& To:ToCorridor)
            { Score+=FVector::Dist2D(From,To)+Cost(From,To); From=To; }
            // Small hysteresis avoids switching corridors for an insignificant
            // queue fluctuation; distance and actual occupied roads both count.
            if(Corridor!=Job.ReturnCorridor && Job.ReturnCorridor!=INDEX_NONE) Score+=3000.;
            if(Score>=BestCost) continue;
            BestCost=Score; Candidate=MoveTemp(ToCorridor);
            SelectedReturnCorridor=Corridor; RouteFound=true;
        }
    }
    else if(Job.Stage==3)
    {
        // Loaded traffic leaves each vessel on its nearest south, central or
        // north corridor.  An unrestricted shortest-path search sent every
        // vessel down the south quay spine, creating a single FIFO queue and
        // leaving the other two paved roads unused.
        const bool Far=Yard[Job.Slot].Half!=0;
        const float CrossY=ApronLoaded?ApronDestinationY:
            Far && TerminalLayout::AGVBerthGroup(Job.STS)==1?
                TerminalLayout::AGVCentralInboundY:TerminalLayout::AGVLoadedCrossY(Job.STS);
        const bool Northbound=Goal.Y>=CrossY;
        const float ArrivalSpine=Far?(Northbound?TerminalLayout::AGVInlandInboundX:
            TerminalLayout::AGVInlandReturnX):ApronLoaded?ApronSpine:(Northbound?12500.f:14000.f);
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
    Candidate=SiteLogistics::RoundRoadCorners(Start,Candidate,
        [&](FVector Point,FVector Direction)
        {
            // 7.3 m longitudinal half-body + 2.1 m crossing half-width,
            // steering margin and the 1 m road-node acceptance, rounded up.
            constexpr float JunctionClearance=1400.f;
            if(FMath::Abs(Direction.Y)>.995f)
            {
                for(float Y:{TerminalLayout::AGVSouthInboundY,TerminalLayout::AGVSouthReturnY,
                    TerminalLayout::AGVCentralInboundY,TerminalLayout::AGVCentralReturnY,
                    bReuseFreedApron?TNumericLimits<float>::Max():TerminalLayout::AGVParkCrossingY(CentralInboundGap),
                    bReuseFreedApron?TNumericLimits<float>::Max():TerminalLayout::AGVParkCrossingY(CentralReturnGap),
                    TerminalLayout::AGVNorthInboundY,TerminalLayout::AGVNorthReturnY,
                    bDepotParking?DepotEntry:TNumericLimits<float>::Max(),
                    bDepotParking?DepotExit:TNumericLimits<float>::Max()})
                    if(FMath::Abs(Point.Y-Y)<JunctionClearance) return false;
                if(LaneCount==9)
                {
                    bool ServiceSpine=false;
                    for(float X:{TerminalLayout::AGVApronNorthX,TerminalLayout::AGVApronSouthX,
                        12500.f,14000.f,15300.f,16700.f,TerminalLayout::AGVInlandInboundX,
                        TerminalLayout::AGVInlandReturnX})
                        ServiceSpine|=FMath::Abs(Point.X-TerminalLayout::SiteCmX(X))<10.f;
                    if(ServiceSpine) for(int32 Block=0;Block<TerminalLayout::YardBlockCount;++Block)
                        if(FMath::Abs(Point.Y-TerminalLayout::RMGHandoverY(Block)*100.f)<JunctionClearance)
                            return false;
                }
                for(int32 Gap=0;!bReuseFreedApron && Gap<TerminalLayout::AGVParkBays-1;++Gap)
                    if(FMath::Abs(Point.Y-TerminalLayout::AGVParkCrossingY(Gap))<JunctionClearance) return false;
                if(bReuseFreedApron && Point.X<TerminalLayout::SiteCmX(18000.f))
                {
                    // Protect actual apron merge and yard entry intersections,
                    // rather than every gap in a parking grid that no longer exists.
                    for(const auto& Active:Jobs)
                    {
                        if(Active.Stage!=3 || !Manifest.IsValidIndex(Active.Cargo)) continue;
                        const float DestinationY=Yard[Active.Slot].Half==0?YardHandover(Yard[Active.Slot]).Y:
                            TerminalLayout::AGVBerthGroup(Active.STS)==1?TerminalLayout::AGVCentralInboundY:
                                TerminalLayout::AGVLoadedCrossY(Active.STS);
                        if(FMath::Abs(Point.Y-AGVApronRoads::DepartureY(CargoQuay(Active.Cargo).Y,DestinationY))<JunctionClearance)
                            return false;
                    }
                    if(Job.Stage==3 && FMath::Abs(Point.Y-ApronDestinationY)<JunctionClearance) return false;
                    if(FMath::Abs(Point.Y-QueueAccessY)<JunctionClearance) return false;
                }
                if(FMath::Abs(Point.X-TerminalLayout::SiteCmX(TerminalLayout::AGVQuayOutboundLaneX))<100.f)
                    for(int32 STS=0;STS<9;++STS)
                        if(FMath::Abs(Point.Y-QuayPark(STS).Y)<1800.f) return false;
            }
            else if(FMath::Abs(Direction.X)>.995f)
                for(float X:{bReuseFreedApron?TerminalLayout::AGVApronNorthX:12500.f,
                    bReuseFreedApron?TerminalLayout::AGVApronSouthX:14000.f,12500.f,14000.f,15300.f,16700.f,
                    TerminalLayout::AGVInlandInboundX,TerminalLayout::AGVInlandReturnX})
                    if(FMath::Abs(Point.X-TerminalLayout::SiteCmX(X))<JunctionClearance) return false;
            return true;
        });
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
    // Rounded reroutes can extend past the road's end even when every raw
    // graph edge was valid. Reject them before replacing a safe active route.
    if(!AGVTrafficGeometry::RouteEnvelopeInside(Start,Candidate)) return false;
    if(Congested)
    {
        // The old planner retained an unrounded 90-degree corner when there
        // was not enough tangent before a yard gate. A real tire-steered AGV
        // cannot make that shortcut. Keep its original drivable route instead.
        if(!AGVTrafficGeometry::RoundedRouteHasNoSharpTurns(Start,Candidate)) return false;
        if(!Candidate.IsEmpty() && FVector::DotProduct((Candidate[0]-Start).GetSafeNormal2D(),
            Vehicle->PlannedMotionDirection(Candidate[0]))<FMath::Cos(FMath::DegreesToRadians(30.f))) return false;
    }
    if(SiteLogistics::RouteIntersectsContainer(Candidate,Start,Yard,Job.Stage==5?Job.Slot:INDEX_NONE))
    {
        UE_LOG(LogTemp,Error,TEXT("AGV_ROUTE_REJECTED: V%d planned path intersects an occupied yard container"),ID);
        return false;
    }
    if(ApronLoaded && !Congested)
    {
        double Distance=0; FVector From=Start;
        for(const FVector& Point:Candidate) { Distance+=FVector::Dist2D(From,Point); From=Point; }
        UE_LOG(LogTemp,Display,TEXT("AGV_APRON_ROUTE: V%d sts=%d yard=%s spine=%.0f distance_cm=%.1f"),
            ID,Job.STS+1,FarYard?TEXT("far"):TEXT("near"),ApronSpine,Distance);
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
    if(Job.Stage==5 && SelectedReturnCorridor!=INDEX_NONE && Job.ReturnCorridor!=SelectedReturnCorridor)
    {
        if(Job.ReturnCorridor!=INDEX_NONE)
            ReturnRouteCounts[Job.ReturnCorridor]=FMath::Max(0,ReturnRouteCounts[Job.ReturnCorridor]-1);
        Job.ReturnCorridor=SelectedReturnCorridor;
        ++ReturnRouteCounts[SelectedReturnCorridor];
    }
    if(Job.Stage==3) Job.YardEntryWaypoint=CandidateYardEntryWaypoint;
    Job.Route=MoveTemp(Candidate); Job.Waypoint=0; Job.RouteStart=Start; Job.bRouteStartValid=true;
    // A new route must not inherit the final bay reservation from the previous
    // delivery/return. Its complete corridor is admitted independently.
    RoadReservations.Remove(ID); RoadReservationEndWaypoint.Remove(ID); ReservationIndex.Remove(ID); RoadTargets.Remove(ID);
    FinishedRoadSegments.Remove(ID);
    ClearPendingRoadRequest(ID);
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
    const FVector Quay=TransferQuay(Job),Dock=YardHandover(Yard[Job.Slot]);
    const float BlockY=TerminalLayout::BlockY(Yard[Job.Slot].Block)*100.f;
    Job.Route.Reset(); Job.Waypoint=0; Job.RouteStart=Vehicles[Lane]->GetActorLocation(); Job.bRouteStartValid=true;
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

void APortSiteLogistics::ClearPendingRoadRequest(int32 ID)
{
    PendingRoadRequests.Remove(ID); PendingRoadIndex.Remove(ID);
}

double APortSiteLogistics::RoadPriorityBonus(int32 ID) const
{
    const int32 I=ID-100;
    if(!bPickupRoadPriority || !Jobs.IsValidIndex(I) || !Vehicles.IsValidIndex(I) || Vehicles[I]->VehicleID!=ID) return 0;
    const auto& J=Jobs[I];
    const bool Clearing=J.Stage==3 && STSOwners.IsValidIndex(J.STS) && STSOwners[J.STS]==I;
    const bool Pickup=Vehicles[I]->PayloadKg<=.1f &&
        (J.Stage==6 || J.Stage==7 || (J.Stage==5 && J.ReturnSTS!=INDEX_NONE));
    const int32 STS=J.Stage==5?J.ReturnSTS:J.STS;
    const int32 Cargo=J.Stage==5?J.ReturnCargo:J.Cargo;
    double HeldSeconds=0;
    if(Pickup && Manifest.IsValidIndex(Cargo) && Equipment.IsValidIndex(YardCraneCount+STS) && STS>=0)
    {
        const auto* Crane=Equipment[YardCraneCount+STS].Get();
        if(Crane->IsWaitingForAGV() && Crane->CargoActor==Manifest[Cargo].Actor.Get())
            HeldSeconds=Manifest[Cargo].STSWaitingForVehicleSeconds;
    }
    return TrafficJunctionPolicy::MissionBonus(Pickup,HeldSeconds,Clearing);
}

bool APortSiteLogistics::RoadRequestBefore(int32 A,int32 B) const
{
    const double* SinceA=RoadWaitSince.Find(A),*SinceB=RoadWaitSince.Find(B);
    if(!bPickupRoadPriority) return TrafficJunctionPolicy::OlderRequestFirst(SinceA,SinceB,A,B);
    const double PA=TrafficJunctionPolicy::PriorityTime(SinceA,TrafficClock,RoadPriorityBonus(A));
    const double PB=TrafficJunctionPolicy::PriorityTime(SinceB,TrafficClock,RoadPriorityBonus(B));
    if(PA!=PB) return PA<PB;
    if(SinceA || SinceB) return TrafficJunctionPolicy::OlderRequestFirst(SinceA,SinceB,A,B);
    const int32 IA=A-100,IB=B-100;
    if(Jobs.IsValidIndex(IA) && Jobs.IsValidIndex(IB) && Jobs[IA].StartedAt!=Jobs[IB].StartedAt)
        return Jobs[IA].StartedAt<Jobs[IB].StartedAt;
    return A<B;
}

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
    PortSimPerformance::FScope Profile(TEXT("traffic_index"));
    for(auto It=PendingRoadRequests.CreateIterator();It;++It)
    {
        const int32 I=Vehicles.IndexOfByPredicate([ID=It.Key()](const auto& V){return V->VehicleID==ID;});
        if(!Jobs.IsValidIndex(I) || !Jobs[I].Stage || !Vehicles[I]->Sensors.bFMSHold ||
            !RoadWaitSince.Contains(It.Key()))
        { PendingRoadIndex.Remove(It.Key()); It.RemoveCurrent(); }
    }
    for (int32 ID:FinishedRoadSegments)
    {
        const int32 I=Vehicles.IndexOfByPredicate([ID](const auto& V) { return V->VehicleID==ID; });
        // Keep the next segment claimed while the vehicle crosses a waypoint.
        // Releasing it for one frame lets another vehicle enter its approach corridor.
        const int32* LastApproved=RoadReservationEndWaypoint.Find(ID);
        if (Jobs.IsValidIndex(I) && Jobs[I].Route.IsValidIndex(Jobs[I].Waypoint) &&
            (LaneCount!=9 || (LastApproved && Jobs[I].Waypoint<=*LastApproved)))
        {
            // Release the completed segment behind the vehicle, retaining only its next segment.
            if (auto* Segments=RoadReservations.Find(ID); Segments && Segments->Num()>1)
            { Segments->RemoveAt(0); ReservationIndex.Set(ID,*Segments); }
            continue;
        }
        RoadReservations.Remove(ID); RoadReservationEndWaypoint.Remove(ID); ReservationIndex.Remove(ID); RoadTargets.Remove(ID);
    }
    FinishedRoadSegments.Reset();
    // A stopped wait cycle can retain mutually blocking future corridors.
    // Re-admit those corridors in normal priority order. Physical occupancy,
    // LiDAR and the post-move overlap guard remain authoritative throughout.
    if(LaneCount==9 && TrafficClock-LastCycleRecoveryAt>=10)
    {
        for(const auto& Waiting:RoadBlockers)
        {
            const TArray<int32> Cycle=TrafficJunctionPolicy::WaitCycle(RoadBlockers,Waiting.Key);
            if(Cycle.Num()<2) continue;
            bool Stopped=true;
            for(int32 ID:Cycle)
            {
                const int32 I=Vehicles.IndexOfByPredicate([ID](const auto& V){return V->VehicleID==ID;});
                if(!Jobs.IsValidIndex(I) || Vehicles[I]->Speed>.1f || Jobs[I].StationaryTime<8.f)
                { Stopped=false; break; }
            }
            if(!Stopped) continue;
            for(int32 ID:Cycle)
            {
                RoadReservations.Remove(ID); RoadReservationEndWaypoint.Remove(ID); ReservationIndex.Remove(ID); RoadTargets.Remove(ID);
                if(!RoadWaitSince.Contains(ID)) RoadWaitSince.Add(ID,TrafficClock);
            }
            LastCycleRecoveryAt=TrafficClock;
            UE_LOG(LogTemp,Display,TEXT("AGV_CYCLE_RECOVERY: re-admit %d stopped vehicles"),Cycle.Num());
            break;
        }
    }
    if (!bTrafficIndexReady)
    {
        ReservationIndex.Reset(); VehicleIndex.Reset(); PendingRoadIndex.Reset();
        for (const auto& Reservation:RoadReservations) ReservationIndex.Set(Reservation.Key,Reservation.Value);
        for (const auto& Request:PendingRoadRequests) PendingRoadIndex.Set(Request.Key,Request.Value);
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
        float Acceptance=(LaneCount==9 || PrecisionApproach)?100.f:-1.f;
        if(Job.Route.IsValidIndex(Job.Waypoint+1))
        {
            // Classify route geometry from immutable FMS nodes. Using the live
            // vehicle position made a straight waypoint become a "corner" as
            // soon as the chassis passed it, shrinking acceptance to 25 cm and
            // trapping the AGV in a circle around the point.
            const FVector PreviousNode=Job.Route.IsValidIndex(Job.Waypoint-1)?
                Job.Route[Job.Waypoint-1]:(Job.bRouteStartValid?Job.RouteStart:Vehicle->GetActorLocation());
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
                Acceptance=100.f;
            }
        }
        const FVector DockingAxis=LaneCount==9 && Docking?
            (TrafficRecoveryTargets.Contains(Vehicle->VehicleID)?
                (FMath::Abs(Vehicle->GetActorRightVector().X)>.999f?FVector(1,0,0):FVector(0,1,0)):
             Job.Stage==3?
                FVector(1,0,0):FVector(0,1,0)):FVector::ZeroVector;
        Vehicle->SetFMSCommand(Target,Job.Waypoint,Job.Route.Num(),Docking,Acceptance,DockingAxis);
        if(!Docking && Job.Route.IsValidIndex(Job.Waypoint+1) &&
            (Job.Waypoint>0 || Job.bRouteStartValid) && AGVTrafficGeometry::PassedRoadGate(
                Job.Waypoint>0?Job.Route[Job.Waypoint-1]:Job.RouteStart,Target,Job.Route[Job.Waypoint+1],
                Vehicle->GetActorLocation(),Vehicle->PlannedMotionDirection(Target)))
        { FinishedRoadSegments.Add(Vehicle->VehicleID); return true; }
        float ApproachLimit=Vehicle->SpeedLimitCmPerSecond();
        FVector From=Vehicle->GetActorLocation();
        float DistanceToCorner=0;
        for(int32 Next=Job.Waypoint;Next+1<Job.Route.Num();++Next)
        {
            const FVector Point=Job.Route[Next];
            DistanceToCorner+=FVector::Dist2D(From,Point);
            const FVector Previous=Next>0?Job.Route[Next-1]:(Job.bRouteStartValid?Job.RouteStart:From);
            const FVector Incoming=(Point-Previous).GetSafeNormal2D();
            const FVector Outgoing=(Job.Route[Next+1]-Point).GetSafeNormal2D();
            if(!Incoming.IsNearlyZero() && !Outgoing.IsNearlyZero() &&
                FVector::DotProduct(Incoming,Outgoing)<.995f)
            {
                // Brake before the tangent point, including node acceptance.
                // Waiting for a heading error allowed a 6-7 m/s vehicle to
                // enter the complete arc before it could slow to 2.5 m/s.
                const float Curve=Vehicle->CurveSpeedCmPerSecond();
                ApproachLimit=FMath::Min(ApproachLimit,FMath::Sqrt(Curve*Curve+
                    2*Vehicle->BrakingCmPerSecondSquared()*FMath::Max(0.f,DistanceToCorner-100.f-Vehicle->Speed*.4f)));
                break;
            }
            From=Point;
        }
        float EnvironmentalLiDARRangeCm=FogVisibilityMeters<MAX_flt?FogVisibilityMeters*100.f:MAX_flt;
        if(RainState.bRainEnabled)
            EnvironmentalLiDARRangeCm=FMath::Min(EnvironmentalLiDARRangeCm,10000.f*RainState.SensorRangeScale(TEXT("lidar")));
        if(EnvironmentalLiDARRangeCm<MAX_flt)
        {
            const float LocalizationAllowance=2.f*AGVReference::PositionAccuracyCm;
            const float SensorSafeSpeed=FMath::Sqrt(2.f*Vehicle->BrakingCmPerSecondSquared()*
                FMath::Max(0.f,EnvironmentalLiDARRangeCm-LocalizationAllowance));
            ApproachLimit=FMath::Min(ApproachLimit,SensorSafeSpeed);
        }
        Vehicle->SetApproachSpeedLimit(ApproachLimit);
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
    float EnvironmentalLiDARRangeCm=FogVisibilityMeters<MAX_flt?FogVisibilityMeters*100.f:MAX_flt;
    if(RainState.bRainEnabled)
        EnvironmentalLiDARRangeCm=FMath::Min(EnvironmentalLiDARRangeCm,10000.f*RainState.SensorRangeScale(TEXT("lidar")));
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
            if(EnvironmentalLiDARRangeCm<MAX_flt && Clearance>EnvironmentalLiDARRangeCm) continue;
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
        Vehicle->Speed,Vehicle->BrakingCmPerSecondSquared());
    const float LeadingDistance=Reversing?RearDistance:FrontDistance;
    // If visibility degrades while travelling above the new sensor-safe speed,
    // command one controlled stop before resuming at the visibility-limited rate.
    const bool VisibilityStop=EnvironmentalLiDARRangeCm<MAX_flt && LiDARSafetyDistance>EnvironmentalLiDARRangeCm;
    const bool LiDARStop=VisibilityStop || (LeadingDistance>=0 && LeadingDistance<=LiDARSafetyDistance);
    Vehicle->UpdateLiDARObservation(FrontDistance,RearDistance,LiDARSafetyDistance,
        EnvironmentalLiDARRangeCm<MAX_flt?EnvironmentalLiDARRangeCm:-1.f,LiDARStop,
        !VisibilityStop&&LiDARStop?LiDARBlocker:INDEX_NONE);
    if(LiDARStop)
    {
        RoadBlockers.Add(ID,LiDARBlocker);
        ClearPendingRoadRequest(ID);
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
    const int32* LastApproved=RoadReservationEndWaypoint.Find(ID);
    const bool ExistingLease=LaneCount==9 && Jobs.IsValidIndex(FleetIndex) && LastApproved &&
        Jobs[FleetIndex].Waypoint<=*LastApproved && RoadTargets.Contains(ID) &&
        RoadReservations.Contains(ID);
    bool RollingStraight=false;
    if(LaneCount==9 && Jobs.IsValidIndex(FleetIndex))
    {
        const auto& Job=Jobs[FleetIndex];
        const int32 End=Job.Stage==3 && !Job.bRMGReserved?
            FMath::Clamp(Job.YardEntryWaypoint+1,0,Job.Route.Num()):Job.Route.Num();
        if(Job.Waypoint+1<End)
        {
            const FVector Previous=Job.Waypoint>0?Job.Route[Job.Waypoint-1]:(Job.bRouteStartValid?Job.RouteStart:Vehicle->GetActorLocation());
            const FVector In=(Target-Previous).GetSafeNormal2D();
            const FVector Out=(Job.Route[Job.Waypoint+1]-Target).GetSafeNormal2D();
            RollingStraight=FVector::DotProduct(In,Out)>.995f;
        }
    }
    if(ExistingLease)
    {
        RoadTargets.Add(ID,Target);
        RoadBlockers.Remove(ID); RoadWaitSince.Remove(ID);
        ClearPendingRoadRequest(ID);
    }
    if (!RoadTargets.Contains(ID) || !RoadTargets[ID].Equals(Target,.01f) ||
        (ExistingLease && RollingStraight && *LastApproved==Jobs[FleetIndex].Waypoint))
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
                },ToTarget.IsNearlyZero()?Driven:ToTarget);
        }

        // Opportunistically extend one straight gate. On failure retain the
        // current lease and brake inside it, instead of stopping at every
        // virtual waypoint or revoking a corridor while the AGV traverses it.
        if(ExistingLease) Segments=RoadReservations.FindChecked(ID);
        const TArray<FBox> BaseSegments=Segments;
        const bool TryExtension=RollingStraight && Segments.Num()==1;
        if(TryExtension)
        {
            const auto& Job=Jobs[FleetIndex];
            const int32 Next=Job.Waypoint+1;
            const int32 End=Job.Stage==3 && !Job.bRMGReserved?
                FMath::Clamp(Job.YardEntryWaypoint+1,0,Job.Route.Num()):Job.Route.Num();
            auto NextExtent=[&](int32 Index)
            {
                const FVector Direction=(Job.Route[Index]-Job.Route[Index-1]).GetSafeNormal2D();
                const FVector Normal(-Direction.Y,Direction.X,0);
                return Direction.GetAbs()*730.f+Normal.GetAbs()*380.f+FVector(0,0,250);
            };
            const FVector E=NextExtent(Next);
            FBox Extended(Target-E,Target+E);
            Extended+=Job.Route[Next]-E; Extended+=Job.Route[Next]+E;
            Segments.Add(Extended);
            TrafficJunctionPolicy::AppendExitCorridor(Segments,Job.Route,Next,End,NextExtent);
        }
        auto FindBlocker=[&]()
        {
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
        if(Blocker==INDEX_NONE)
        {
            // An older merging request must eventually get a gap. Admission
            // order alone is insufficient: new through traffic can otherwise
            // occupy its exit every time the vehicle at the junction clears.
            // Vehicles already in that area may always drain out; existing
            // reservations are never revoked by a pending request.
            const FVector Position=Vehicle->GetActorLocation();
            const FVector BodyExtent=Vehicle->GetActorRightVector().GetAbs()*730.f+
                Vehicle->GetActorForwardVector().GetAbs()*210.f+FVector(0,0,250);
            const FBox Body(Position-BodyExtent,Position+BodyExtent);
            for(int32 WaitingID:PendingRoadIndex.Query(Segments))
            {
                const double* WaitingSince=RoadWaitSince.Find(WaitingID);
                if(!WaitingSince || TrafficClock-*WaitingSince<30.) continue;
                if(WaitingID==ID || !RoadRequestBefore(WaitingID,ID)) continue;
                const auto* Waiting=PendingRoadRequests.Find(WaitingID);
                if(!Waiting || Waiting->ContainsByPredicate([&](const FBox& Box){return Box.Intersect(Body);})) continue;
                bool Intersects=false;
                for(const FBox& A:Segments) for(const FBox& B:*Waiting) Intersects|=A.Intersect(B);
                if(Intersects && (Blocker==INDEX_NONE || RoadRequestBefore(WaitingID,Blocker))) Blocker=WaitingID;
            }
        }
        return Blocker;
        };
        int32 Blocker=FindBlocker();
        if(Blocker!=INDEX_NONE && TryExtension)
        {
            Segments=BaseSegments;
            Blocker=ExistingLease?INDEX_NONE:FindBlocker();
        }
        if (Blocker!=INDEX_NONE)
        {
            // Requests change only at a waypoint. While stopped there, retain the
            // physical vehicle envelope, not an unentered road needed by its blocker.
            // Never revoke the reservation of a vehicle already traversing a segment.
            RoadBlockers.Add(ID,Blocker);
            if(!RoadWaitSince.Contains(ID)) RoadWaitSince.Add(ID,TrafficClock);
            // Protect an actual RMG-to-spine merge, not every waiting road
            // request: broad future-route intents can otherwise lock the
            // depot and unrelated crossings. Bound the junction to 60 m.
            bool Merge=false;
            if(Jobs.IsValidIndex(FleetIndex))
            {
                const auto& J=Jobs[FleetIndex];
                Merge=J.Stage==5 && Yard.IsValidIndex(J.Slot) && FMath::Abs(ToTarget.X)>.995f &&
                    FMath::Abs(Vehicle->GetActorLocation().Y-YardHandover(Yard[J.Slot]).Y)<100.f;
            }
            if(Merge)
            {
                const FVector P=Vehicle->GetActorLocation();
                const FBox Limit(P-FVector(6000,6000,500),P+FVector(6000,6000,500));
                TArray<FBox> Junction;
                for(const FBox& Box:Segments) if(Box.Intersect(Limit)) Junction.Add(Box.Overlap(Limit));
                PendingRoadRequests.Add(ID,Junction); PendingRoadIndex.Set(ID,Junction);
            }
            else ClearPendingRoadRequest(ID);
            if(Jobs.IsValidIndex(FleetIndex)) Jobs[FleetIndex].TrafficWaitSeconds+=Dt;
            RoadReservations.Remove(ID); RoadReservationEndWaypoint.Remove(ID); ReservationIndex.Remove(ID); RoadTargets.Remove(ID);
            Vehicle->SetFMSHold(true);
            return false;
        }
        RoadReservations.Add(ID,Segments);
        if(Jobs.IsValidIndex(FleetIndex))
            RoadReservationEndWaypoint.Add(ID,Jobs[FleetIndex].Waypoint+Segments.Num()-1);
        ReservationIndex.Set(ID,Segments);
        RoadTargets.Add(ID,Target);
        RoadBlockers.Remove(ID);
        RoadWaitSince.Remove(ID);
        ClearPendingRoadRequest(ID);
    }
    Vehicle->SetFMSHold(false);
    Vehicle->SetSteeringLookAhead(Target,false);
    if(LaneCount==9 && Jobs.IsValidIndex(FleetIndex))
    {
        const auto& Job=Jobs[FleetIndex];
        const bool Docking=Job.Waypoint==Job.Route.Num()-1 ||
            (Job.Stage==3 && !Job.bRMGReserved && Job.Waypoint==Job.YardEntryWaypoint);
        const int32* Approved=RoadReservationEndWaypoint.Find(ID);
        if(Approved) StraightThrough=!Docking && Job.Waypoint<*Approved;
        if(!Docking && Approved)
        {
            const FVector Previous=Job.Route.IsValidIndex(Job.Waypoint-1)?
                Job.Route[Job.Waypoint-1]:(Job.bRouteStartValid?Job.RouteStart:Vehicle->GetActorLocation());
            FVector From=FMath::ClosestPointOnSegment(Vehicle->GetActorLocation(),Previous,Target);
            FVector Pursuit=Target;
            float Remaining=300.f;
            for(int32 Next=Job.Waypoint;Next<=*Approved && Job.Route.IsValidIndex(Next);++Next)
            {
                const FVector To=Job.Route[Next];
                const float Length=FVector::Dist2D(From,To);
                Pursuit=To;
                if(Length>=Remaining) { Pursuit=FMath::Lerp(From,To,Remaining/Length); break; }
                Remaining-=Length; From=To;
            }
            Vehicle->SetSteeringLookAhead(Pursuit,true);
        }
    }
    const FTransform BeforeMove=Vehicle->GetActorTransform();
    const float BeforeSpeed=Vehicle->Speed;
    const FAGVSensorState BeforeSensors=Vehicle->Sensors;
    const FAGVDynamicsState BeforeDynamics=Vehicle->Dynamics;
    bool Arrived=Vehicle->MoveToPosition(Target,Dt,!StraightThrough);
    for(const auto& Other:TrafficVehicles)
    {
        if(!IsValid(Other) || Other==Vehicle ||
            !SiteLogistics::AGVFootprintsOverlap(Vehicle,Other)) continue;
        Vehicle->SetActorTransform(BeforeMove,false,nullptr,ETeleportType::TeleportPhysics);
        Vehicle->Speed=BeforeSpeed;
        Vehicle->Sensors=BeforeSensors;
        Vehicle->Dynamics=BeforeDynamics;
        Vehicle->SetFMSHold(true);
        RoadBlockers.Add(ID,Other->VehicleID);
        Arrived=false;
        break;
    }
    const FVector TravelFrom=BeforeMove.GetLocation(),TravelTo=Vehicle->GetActorLocation();
    if(FMath::Abs(TravelTo.Y-TravelFrom.Y)<2 &&
        FMath::Abs(TravelFrom.Y-TerminalLayout::CentralRoadY*100.f)<TerminalLayout::CentralRoadWidthM*50.f)
    {
        const double Low=FMath::Max(FMath::Min(TravelFrom.X,TravelTo.X),double(TerminalLayout::SiteCmX(14500)));
        const double High=FMath::Min(FMath::Max(TravelFrom.X,TravelTo.X),double(TerminalLayout::SiteCmX(63500)));
        if(High>Low) CentralRoadTravelCm[Vehicle->PayloadKg>0?0:1][TravelTo.X>TravelFrom.X?0:1]+=High-Low;
    }
    if(!Arrived && Jobs.IsValidIndex(FleetIndex))
    {
        const auto& Job=Jobs[FleetIndex];
        const bool Docking=Job.Waypoint==Job.Route.Num()-1 ||
            (Job.Stage==3 && !Job.bRMGReserved && Job.Waypoint==Job.YardEntryWaypoint);
        if(!Docking && (Job.Route.IsValidIndex(Job.Waypoint-1) || Job.bRouteStartValid))
        {
            // FMS road nodes are gates, not centimetre-sized parking spots. A
            // car-like chassis has passed an intermediate node when its centre
            // crosses the node plane while remaining within one turn radius of
            // the route. Final STS/RMG docking still requires exact position.
            const FVector Incoming=(Target-(Job.Waypoint>0?Job.Route[Job.Waypoint-1]:Job.RouteStart)).GetSafeNormal2D();
            const FVector Outgoing=Job.Route.IsValidIndex(Job.Waypoint+1)?
                (Job.Route[Job.Waypoint+1]-Target).GetSafeNormal2D():Incoming;
            const bool Straight=FVector::DotProduct(Incoming,Outgoing)>.995f;
            const bool Aligned=FVector::DotProduct(Vehicle->PlannedMotionDirection(Target),Outgoing)>
                FMath::Cos(FMath::DegreesToRadians(30.f));
            if(!Incoming.IsNearlyZero() && (Straight || Aligned))
            {
                const FVector Delta=Vehicle->GetActorLocation()-Target;
                const FVector Lateral(-Incoming.Y,Incoming.X,0);
                const float Along=FVector::DotProduct(Delta,Incoming);
                const float CrossTrack=FMath::Abs(FVector::DotProduct(Delta,Lateral));
                Arrived=Along>=0.f && CrossTrack<=(Straight?AGVReference::MinimumInnerTurnRadiusCm:150.f);
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
    if(Vehicle->bDriveFault) { Vehicle->SetFMSHold(true); return false; }
    const int32 ID=Vehicle->VehicleID;
    if(const FVector* Recovery=TrafficRecoveryTargets.Find(ID))
    {
        const FVector Goal=*Recovery;
        const bool OriginalReverse=TrafficRecoveryOriginalReverse.FindChecked(ID);
        const FVector OriginalDirection=OriginalReverse?-Vehicle->GetActorRightVector():Vehicle->GetActorRightVector();
        const FVector Axis=FMath::Abs(OriginalDirection.X)>.999f?
            FVector(OriginalDirection.X>=0?1.f:-1.f,0,0):FVector(0,OriginalDirection.Y>=0?1.f:-1.f,0);
        // Use the ordinary steering, transponder and front/rear LiDAR controller.
        // No actor teleport, lateral shift or overlap exemption is introduced.
        auto SavedRoute=MoveTemp(Job.Route); const int32 SavedWaypoint=Job.Waypoint;
        const FVector Approach=Goal+Axis*300.f;
        Job.Route={Approach,Goal}; Job.Waypoint=1;
        Vehicle->Sensors.bReversing=!OriginalReverse;
        Vehicle->SetFMSCommand(Approach,2,3,false);
        const bool Clear=MoveVehicle(Vehicle,Goal,Dt);
        Job.Route=MoveTemp(SavedRoute); Job.Waypoint=SavedWaypoint;
        if(Clear)
        {
            TrafficRecoveryTargets.Remove(ID); TrafficRecoveryOriginalReverse.Remove(ID);
            Vehicle->Sensors.bReversing=OriginalReverse;
            RoadReservations.Remove(ID); RoadReservationEndWaypoint.Remove(ID);
            ReservationIndex.Remove(ID); RoadTargets.Remove(ID); FinishedRoadSegments.Remove(ID);
            RoadBlockers.Remove(ID); RoadWaitSince.Remove(ID);
            Job.StationaryTime=0;
            UE_LOG(LogTemp,Display,TEXT("FMS_RECOVERY_COMPLETE: V%d backed along its road; resuming stage=%d waypoint=%d"),ID,Job.Stage,Job.Waypoint);
        }
        return false;
    }
    if(const int32* PriorityVehicle=TrafficRecoveryYieldTo.Find(ID))
    {
        const auto* Other=Vehicles.FindByPredicate([&](const auto& V){return V->VehicleID==*PriorityVehicle;});
        if(Other && Job.Route.IsValidIndex(Job.Waypoint) &&
            AGVTrafficGeometry::SweptChassisIntersects(Vehicle->GetActorLocation(),Job.Route[Job.Waypoint],
                (*Other)->GetActorLocation(),(*Other)->GetActorRightVector(),730,210,170))
        {
            // Do not immediately drive back into the crossing just cleared by
            // reversing. The vehicle we yielded to must physically clear it.
            Vehicle->SetFMSHold(true); RoadBlockers.Add(ID,*PriorityVehicle);
            Job.TrafficWaitSeconds+=Dt;
            return false;
        }
        TrafficRecoveryYieldTo.Remove(ID);
    }
    // The second box of a paired trip reuses an already completed docking
    // route. Verify its physical endpoint before treating that route as done;
    // do not index Route[Num()] or issue another vehicle movement command.
    if(!Job.Route.IsValidIndex(Job.Waypoint))
    {
        if(!Job.Route.IsEmpty() && Job.Waypoint==Job.Route.Num() &&
            Vehicle->Speed<=.1f && Vehicle->GetActorLocation().Equals(
                Job.Route.Last(),AGVReference::DockingAccuracyCm)) return true;
        Stop(TEXT("FMS route ended without a verified stopped vehicle"));
        return false;
    }
    // The inbound and outbound berth lanes have independent directed segments.
    // Their swept vehicle envelopes and junction exits are reserved below; an
    // additional group-wide stop can deadlock an outbound AGV that is itself
    // waiting for this inbound vehicle to clear the junction.
    // Direct returns end in the off-lane STS waiting bay. They may reach that
    // bay while the current handover is occupied; activation below creates a
    // separate final approach only after the STS becomes available.
    const bool Arrived=MoveVehicle(Vehicle,Job.Route[Job.Waypoint],Dt);
    ReleaseYardApproachIfClear(Lane);
    if (!Arrived)
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
    if (Job.Stage==3 && STSOwners[Job.STS]==Lane && FVector::Dist2D(Vehicle->GetActorLocation(),TransferQuay(Job))>1600) STSOwners[Job.STS]=INDEX_NONE;
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

void APortSiteLogistics::RecoverTrafficCycle()
{
    if(LaneCount!=9 || !TrafficRecoveryTargets.IsEmpty() || TrafficClock-LastTrafficRecoveryAt<10) return;
    for(int32 Seed=0;Seed<Jobs.Num();++Seed)
    {
        TArray<int32> Chain;
        int32 Current=Vehicles[Seed]->VehicleID;
        while(Current>=100 && Jobs.IsValidIndex(Current-100) && !Chain.Contains(Current))
        {
            const int32 I=Current-100;
            if(Vehicles[I]->Emergency.Active || (!Vehicles[I]->Sensors.bFMSHold && !Vehicles[I]->Sensors.bControlledStop) ||
                Vehicles[I]->Speed>1.f || Jobs[I].StationaryTime<10) break;
            Chain.Add(Current);
            const int32* Blocker=RoadBlockers.Find(Current);
            if(!Blocker) break;
            Current=*Blocker;
        }
        const int32 CycleStart=Chain.Find(Current);
        if(CycleStart==INDEX_NONE || Chain.Num()-CycleStart<2) continue;
        for(int32 C=CycleStart;C<Chain.Num();++C)
        {
            const int32 I=Chain[C]-100; auto* Vehicle=Vehicles[I].Get();
            const bool LoadedTravel=Jobs[I].Stage==3 && !Jobs[I].bRMGReserved &&
                Vehicle->PayloadWithinReferenceLimit() && !Vehicle->bDriveFault;
            const bool EmptyTravel=(Jobs[I].Stage==5 ||
                (bReuseFreedApron && (Jobs[I].Stage==6 || Jobs[I].Stage==7)) || Jobs[I].Stage==FSiteTransfer::ToCharge ||
                Jobs[I].Stage==FSiteTransfer::ReturnToStandby) && Vehicle->PayloadKg<=.1f;
            if(Vehicle->bDriveFault || (!LoadedTravel && !EmptyTravel)) continue;
            const FVector Start=Vehicle->GetActorLocation();
            const FVector Motion=Vehicle->PlannedMotionDirection(FVector::ZeroVector);
            const bool AlongY=FMath::Abs(Motion.Y)>.999f;
            const bool AlongX=FMath::Abs(Motion.X)>.999f;
            if(!AlongY && !AlongX) continue;
            bool OnSpine=false;
            for(float X:{TerminalLayout::AGVQuayInboundLaneX,
                bReuseFreedApron?TerminalLayout::AGVApronNorthX:12500.f,
                bReuseFreedApron?TerminalLayout::AGVApronSouthX:14000.f,12500.f,14000.f,15300.f,16700.f,
                TerminalLayout::AGVInlandInboundX,TerminalLayout::AGVInlandReturnX})
                OnSpine|=FMath::Abs(Start.X-TerminalLayout::SiteCmX(X))<=10.f;
            const bool ApronCrossing=bReuseFreedApron && EmptyTravel && AlongX &&
                Jobs[I].Route.IsValidIndex(Jobs[I].Waypoint) &&
                FMath::Abs(Jobs[I].Route[Jobs[I].Waypoint].Y-Start.Y)<5.f &&
                Start.X>=TerminalLayout::SiteCmX(TerminalLayout::AGVQuayInboundLaneX) &&
                Start.X<=TerminalLayout::SiteCmX(12500.f);
            if(!(AlongY && OnSpine) && !ApronCrossing) continue;
            const FVector Axis=AlongY?FVector(0,Motion.Y>=0?1.f:-1.f,0):FVector(Motion.X>=0?1.f:-1.f,0,0);
            const FVector HalfExtent=AlongY?FVector(210,730,250):FVector(730,210,250);
            for(float Distance:{2000.f,1400.f,1000.f})
            {
                const FVector Goal=Start-Axis*Distance;
                if(!TerminalLayout::AGVEnvelopeInside(Goal.X-HalfExtent.X,Goal.X+HalfExtent.X,
                    Goal.Y-HalfExtent.Y,Goal.Y+HalfExtent.Y)) continue;
                if(ApronCrossing && (Goal.X<TerminalLayout::SiteCmX(TerminalLayout::AGVQuayInboundLaneX) ||
                    Goal.X>TerminalLayout::SiteCmX(12500.f))) continue;
                FBox Sweep(Start-HalfExtent,Start+HalfExtent);
                Sweep+=Goal-HalfExtent; Sweep+=Goal+HalfExtent;
                bool Clear=true;
                for(int32 OtherID:ReservationIndex.Query({Sweep}))
                    if(OtherID!=Vehicle->VehicleID) { Clear=false; break; }
                if(!Clear) continue;
                for(const auto& Other:TrafficVehicles)
                    if(IsValid(Other) && Other!=Vehicle && !AGVTrafficGeometry::ClearReverseSweep(
                        Start,Goal,Other->GetActorLocation(),Other->GetActorRightVector()))
                    { Clear=false; break; }
                if(!Clear) continue;
                const int32 ID=Vehicle->VehicleID;
                TrafficRecoveryTargets.Add(ID,Goal);
                TrafficRecoveryOriginalReverse.Add(ID,Vehicle->Sensors.bReversing);
                TrafficRecoveryYieldTo.Add(ID,RoadBlockers.FindChecked(ID));
                // Claim the checked reverse corridor atomically, before any
                // other vehicle in this traffic step can request that space.
                RoadReservations.Add(ID,{Sweep}); RoadReservationEndWaypoint.Add(ID,1);
                ReservationIndex.Set(ID,{Sweep}); RoadTargets.Add(ID,Goal);
                FinishedRoadSegments.Remove(ID);
                LastTrafficRecoveryAt=TrafficClock;
                UE_LOG(LogTemp,Display,TEXT("FMS_RECOVERY_START: V%d payload=%.0f kg; yield_to=V%d cycle=%d vehicles; clear reverse distance=%.0f cm target=%s"),
                    ID,Vehicle->PayloadKg,TrafficRecoveryYieldTo[ID],Chain.Num()-CycleStart,Distance,*Goal.ToCompactString());
                return;
            }
        }
    }
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
    if(bReady) ProcessDashboardCommands();
    PortSimPerformance::Report(SimulationTime,Delivered,GetWorld());
    PortSimPerformance::FScope Profile(TEXT("site_advance"));
    if(!bReady) return;
    if(Paused || !Fault.IsEmpty() || Dt<=0.f)
    { AdvanceStep(Dt,Paused); ExportDashboard(Paused); PrepareKinematicPhysicsFrame(); return; }
    // One traffic clock for editor playback and headless tests. Render-frame
    // deltas up to two seconds must not move an AGV across several reservation
    // gates before the other vehicles and LiDAR have been updated.
    constexpr double TrafficStepSeconds=.05;
    auto ApplyFreeCargoWind=[&]
    {
        // AddForce accumulates for the next Chaos frame; it is not a traffic-
        // step impulse. Adding it at every 50 ms step multiplied wind by up to
        // 40 on a two-second accelerated frame and displaced released loads.
        for(const auto& Cargo:ShipContainers) if(IsValid(Cargo))
            Cargo->ApplyWind(WindVelocityMetersPerSecond,STSProfile.Dynamics.WindDrag);
    };
    PendingSimulationSeconds+=Dt;
    const int32 Steps=FMath::FloorToInt((PendingSimulationSeconds+1.e-9)/TrafficStepSeconds);
    if(Steps<=0) { ApplyFreeCargoWind(); PrepareKinematicPhysicsFrame(); return; }
    bDeferringVisuals=true;
    for(const auto& Box:ShipContainers) if(IsValid(Box)) Box->BeginVisualFrame();
    for(const auto& Crane:Equipment) Crane->DeferRopeVisuals(true);
    ON_SCOPE_EXIT
    {
        for(const auto& Box:ShipContainers) if(IsValid(Box))
        {
            if(Box->LocationOwner==ECargoOwner::Ship && !Box->GetAttachParentActor()) Box->ApplySecuredMotionFraction(1.);
            Box->EndVisualFrame();
        }
        for(const auto& Crane:Equipment) Crane->DeferRopeVisuals(false);
        bDeferringVisuals=false;
        ExportDashboard(false);
        if(Fault.IsEmpty()) ApplyFreeCargoWind();
        PrepareKinematicPhysicsFrame();
    };
    // The environment supplies one endpoint per render frame. Reinitializing
    // that same endpoint every 50 ms creates a staircase pose with unrelated
    // analytic velocity. Interpolate it once across the whole consumed frame.
    ApplyVesselMotions(Steps*TrafficStepSeconds);
    for(int32 Step=0;Step<Steps;++Step)
    {
        PendingSimulationSeconds-=TrafficStepSeconds;
        ShipWindowStart=double(Step)/Steps; ShipWindowEnd=double(Step+1)/Steps;
        {
        PortSimPerformance::FScope VesselTiming(TEXT("secured_cargo_motion"));
        for(const auto* Records:{&Manifest,&RetainedCargo,&ExportCargo}) for(const auto& C:*Records)
        {
            auto* Box=C.Actor.Get();
            if(!Box || Box->LocationOwner!=ECargoOwner::Ship || Box->GetAttachParentActor()) continue;
            Box->SetSecuredMotionWindow(ShipWindowStart,ShipWindowEnd);
        }
        }
        BeginTrafficFrame();
        AdvanceStep(static_cast<float>(TrafficStepSeconds),false);
        if(!Fault.IsEmpty()) break;
    }
}

void APortSiteLogistics::SynchronizeShipCollision(const FBox& Query,double ControlFraction)
{
    if(!bDeferringVisuals) return;
    PortSimPerformance::FScope Timing(TEXT("ship_query_sync"));
    for(const auto& Box:ShipContainers)
        if(IsValid(Box) && Box->LocationOwner==ECargoOwner::Ship && !Box->GetAttachParentActor() &&
            Box->SecuredMotionCouldIntersect(Query))
            Box->ApplySecuredMotionFraction(ControlFraction);
}

bool APortSiteLogistics::PrepareKinematicPhysicsFrame() const
{
    PortSimPerformance::FScope Timing(TEXT("physics_frame_policy"));
    UWorld* World=GetWorld();
    const auto* Settings=UPhysicsSettings::Get();
    if(LaneCount!=9 || !World || !World->GetPhysicsScene() || Settings->bTickPhysicsAsync ||
        FParse::Param(FCommandLine::Get(),TEXT("PortSimFullPhysics")) ||
        Settings->PhysicsPrediction.bEnablePhysicsPrediction) return false;
    const FVector Gravity(0,0,World->GetGravityZ());
    // Restore the normal setup even if another Advance call in this same
    // world frame already selected the kinematic path before a body was freed.
    World->GetPhysicsScene()->SetUpForFrame(&Gravity,World->GetDeltaSeconds(),Settings->MinPhysicsDeltaTime,
        Settings->MaxPhysicsDeltaTime,Settings->MaxSubstepDeltaTime,Settings->MaxSubsteps,Settings->bSubstepping);
    // Entry's unified terminal uses explicit kinematic drives and its own
    // suspension integrator. Still check the WHOLE world: a free load, sleeping
    // rigid body, skeletal mesh or other physics system requires normal Chaos.
    TArray<UPrimitiveComponent*> KinematicParts;
    for(TActorIterator<AActor> It(World);It;++It)
    {
        TInlineComponentArray<UPrimitiveComponent*> Parts(*It);
        for(auto* Part:Parts)
        {
            if(Part->IsSimulatingPhysics() ||
                (Part->IsPhysicsStateCreated() && Part->K2_IsPhysicsCollisionEnabled() &&
                    !Part->IsStaticPhysics() &&
                    !Part->IsA<UStaticMeshComponent>() && !Part->IsA<UProceduralMeshComponent>() &&
                    !Part->IsA<UTextRenderComponent>() && !Part->IsA<UWidgetComponent>() &&
                    Part->GetAllPhysicsObjects().ContainsByPredicate([](const auto* Object){return Object!=nullptr;})))
            {
                if(PortSimPerformance::State().Enabled || GIsAutomationTesting)
                {
                    static TSet<FName> ReportedTypes;
                    const FName Type=Part->GetClass()->GetFName();
                    if(!ReportedTypes.Contains(Type))
                    {
                        ReportedTypes.Add(Type);
                        UE_LOG(LogTemp,Display,TEXT("KINEMATIC_PHYSICS_FALLBACK: %s simulating=%d"),
                            *Part->GetFullName(),Part->IsSimulatingPhysics());
                    }
                }
                return false;
            }
            if(Part->IsPhysicsStateCreated() && Part->K2_IsPhysicsCollisionEnabled() && !Part->IsStaticPhysics())
                KinematicParts.Add(Part);
        }
    }
    // A zero-time solver frame does not consume deferred kinematic targets.
    // Commit the current physical poses while there are no free bodies, so
    // enabling a released load cannot replay an AGV's old parking-to-deck motion.
    for(auto* Part:KinematicParts)
    {
        const FTransform Pose=Part->GetComponentTransform();
        auto* Previous=KinematicPhysicsPoses.Find(Part);
        if(!Previous || !Previous->Equals(Pose,.001))
        {
            if(auto* Body=Part->GetBodyInstance())
                Body->SetBodyTransform(Pose,ETeleportType::TeleportPhysics,false);
            KinematicPhysicsPoses.Add(Part,Pose);
        }
    }
    for(auto It=KinematicPhysicsPoses.CreateIterator();It;++It)
        if(!It.Key().IsValid()) It.RemoveCurrent();
    // Keep the physics scene and its query/proxy synchronization alive, using
    // Chaos's supported zero-time frame for an entirely kinematic scene. The
    // world's next SetupPhysicsTickFunctions restores the configured delta and
    // 120 Hz substeps automatically. Never change app/simulation time or the
    // global physics settings, and never shorten a frame containing a free body.
    World->GetPhysicsScene()->SetUpForFrame(&Gravity,0.f,0.f,0.f,Settings->MaxSubstepDeltaTime,
        Settings->MaxSubsteps,false);
    return true;
}

void APortSiteLogistics::AdvanceStep(float Dt,bool Paused)
{
    PortSimPerformance::FScope Profile(TEXT("traffic_step"));
    if (!bReady) return;
    if(!Paused && Fault.IsEmpty())
    {
        SimulationTime+=Dt;
    }
    if(Paused) for(int32 Index:PreparedCargo) if(Index!=INDEX_NONE) Manifest[Index].PreparedPausedSeconds+=Dt;
    if(!Paused && Fault.IsEmpty()) for(int32 STS=0;STS<LaneCount;++STS)
    {
        const auto* Crane=Equipment[YardCraneCount+STS].Get();
        if(!Crane->IsWaitingForAGV()) continue;
        for(auto& C:Manifest) if(C.Actor.Get()==Crane->CargoActor)
        { C.STSWaitingForVehicleSeconds+=Dt; break; }
    }
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
        if (!Crane->Fault.IsEmpty() && !Crane->IsEquipmentUnavailable()) { Stop(Crane->Fault); return; }
    }
    ManageEquipmentFailures();
    if(!Fault.IsEmpty()) return;
    int32 Moving=0;
    // Use the actual individually attached cargo, including both 20ft boxes.
    // A box held by the RMG no longer contributes to vehicle load/CoG.
    TMap<APortAGVActor*,double> DeckMass;
    TMap<APortAGVActor*,FVector> DeckMoment;
    for(const auto& Box:ShipContainers) if(IsValid(Box) && Box->LocationOwner==ECargoOwner::AGV)
    {
        auto* Carrier=Cast<APortAGVActor>(Box->GetAttachParentActor());
        if(!Carrier) continue;
        DeckMass.FindOrAdd(Carrier)+=Box->MassKg;
        const FVector WorldCoG=Box->GetActorLocation()+Box->GetActorQuat().RotateVector(Box->CoGOffsetCm);
        DeckMoment.FindOrAdd(Carrier,FVector::ZeroVector)+=Carrier->GetActorTransform().InverseTransformPosition(WorldCoG)*Box->MassKg;
    }
    for(const auto& Vehicle:Vehicles)
    {
        const double Mass=DeckMass.FindRef(Vehicle.Get());
        Vehicle->SetPayload(Mass);
        Vehicle->DeckCoGOffsetCm=Mass>0?DeckMoment.FindRef(Vehicle.Get())/Mass:FVector::ZeroVector;
        auto DynamicsConfig=Vehicle->DynamicsSettings; DynamicsConfig.Sanitize();
        Vehicle->Dynamics.RefreshLoads(DynamicsConfig,Vehicle->DynamicsInput());
    }
    for (const auto& Vehicle:Vehicles) Moving+=Vehicle->Speed>1.f;
    PeakMovingVehicles=FMath::Max(PeakMovingVehicles,Moving);
    if (LaneCount==9) { AdvanceFleetBatteries(Dt); if(!Fault.IsEmpty()) return; ScheduleFleet(); ScheduleExport(); }
    RecoverTrafficCycle();
    if(!Fault.IsEmpty()) return;
    // Age actual blocked road requests, with a capped pickup/STS-exit bonus.
    // Existing reservations are never preempted by a higher priority request.
    TArray<int32> Order;
    for (int32 I=0;I<Jobs.Num();++I) Order.Add(I);
    Order.StableSort([this](int32 A,int32 B)
    {
        const double* WaitA=RoadWaitSince.Find(Vehicles[A]->VehicleID);
        const double* WaitB=RoadWaitSince.Find(Vehicles[B]->VehicleID);
        if(bPickupRoadPriority)
        {
            if(bool(Jobs[A].Stage)!=bool(Jobs[B].Stage)) return Jobs[A].Stage!=0;
            return RoadRequestBefore(Vehicles[A]->VehicleID,Vehicles[B]->VehicleID);
        }
        if(WaitA || WaitB) return TrafficJunctionPolicy::OlderRequestFirst(WaitA,WaitB,Vehicles[A]->VehicleID,Vehicles[B]->VehicleID);
        if(bool(Jobs[A].Stage)!=bool(Jobs[B].Stage)) return Jobs[A].Stage!=0;
        if(Jobs[A].Stage && Jobs[B].Stage && Jobs[A].StartedAt!=Jobs[B].StartedAt) return Jobs[A].StartedAt<Jobs[B].StartedAt;
        return Vehicles[A]->VehicleID<Vehicles[B]->VehicleID;
    });
    for (int32 Lane:Order)
    {
        auto& Job=Jobs[Lane]; auto* Vehicle=Vehicles[Lane].Get();
        if(Vehicle->Emergency.Active)
        {
            const bool WasFinished=Vehicle->Emergency.Finished;
            Vehicle->AdvanceEmergencyScenario(Dt);
            IndexTrafficVehicle(TrafficVehicles.IndexOfByKey(Vehicle));
            if(!WasFinished && Vehicle->Emergency.Finished)
                RecordRecoveryEvent(TEXT("agv_emergency_finished"),Vehicle,Vehicle,Vehicle,Job.Actor.Get());
            if(Vehicle->Emergency.Finished && !Vehicle->Emergency.Stopped)
            {Stop(FString::Printf(TEXT("AGV %d emergency test ended: %s; reset required"),Vehicle->VehicleID,*Vehicle->Emergency.Outcome));return;}
            // Hold existing road/yard claims; no reroute, handover or synthetic
            // zero-speed stop can replace the physical test integration.
            continue;
        }
        if(Vehicle->bDriveFault)
        {
            Vehicle->SetFMSHold(true);
            if(Job.Stage && Job.Stage<FSiteTransfer::ToCharge) { Job.EquipmentDowntimeSeconds+=Dt; Job.bEquipmentWaiting=true; }
            continue;
        }
        if(Job.ExportIndex!=INDEX_NONE) { AdvanceExport(Lane,Dt); if(!Fault.IsEmpty()) return; continue; }
        if(Job.Stage>=FSiteTransfer::ToCharge)
        {
            if(Job.Stage==FSiteTransfer::Charging) continue;
            if(Job.LastStage!=Job.Stage || !Job.LastPosition.Equals(Vehicle->GetActorLocation(),10.f)) Job.StationaryTime=0;
            else Job.StationaryTime+=Dt;
            Job.LastStage=Job.Stage; Job.LastPosition=Vehicle->GetActorLocation();
            if(!Drive(Lane,Dt)) continue;
            if(Job.Stage==FSiteTransfer::ToCharge) { Job.Stage=FSiteTransfer::Charging; Vehicle->AdvanceBattery(0,false,true); }
            else { Job=FSiteTransfer(); Vehicle->SetFMSHold(false); }
            continue;
        }
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
        if(Job.bEquipmentWaiting) Job.EquipmentDowntimeSeconds+=Dt;
        if (Job.Time-Job.EquipmentDowntimeSeconds>12000.f && !Job.bEquipmentWaiting) { Stop(TEXT("Shipment timeout")); return; }
        if(JobHeldByFailure(Lane)) { Vehicle->SetFMSHold(true); continue; }
        auto* Cargo=Job.Actor.Get();
        switch(Job.Stage)
        {
        case 7:
            if (Drive(Lane,Dt)) Job.Stage=8;
            break;
        case 8: break;
        case 6:
            // Geometric reservations provide admission per junction; there is
            // no additional vessel-wide gate between unrelated road segments.
            if (!Drive(Lane,Dt)) break;
            if(Job.bYardAdmissionSuspended) { Vehicle->SetFMSHold(true); break; }
            // Stay in the arrival stage until a healthy crane can bind the
            // actual deck target. Repair must not skip this landing handshake.
            if(!CraneAvailable(YardCraneCount+Job.STS)) { Vehicle->SetFMSHold(true); break; }
            if (!Vehicle->GetActorLocation().Equals(TransferQuay(Job),AGVReference::DockingAccuracyCm) || Vehicle->Speed>.1f)
            { Stop(TEXT("AGV did not stop at its cargo-specific STS handover")); return; }
            if(!Equipment[YardCraneCount+Job.STS]->SetAGVLandingTarget(Vehicle->CargoPosition()))
            { Stop(TEXT("STS could not accept its AGV pad before descent")); return; }
            Equipment[YardCraneCount+Job.STS]->SetDestinationReady(true);
            Job.Stage=1; break;
        case 1:
            // Reserve the downstream crane while the STS makes its final transfer.
            // This keeps a completed handover from occupying the quay lane while
            // yard selection starts, without delaying the STS if none is free yet.
            if (Job.Slot==INDEX_NONE && Equipment[YardCraneCount+Job.STS]->Stage>=4) ReserveYard(Lane);
            if (Equipment[YardCraneCount+Job.STS]->IsBusy()) break;
            if (!Cargo || !Cargo->GetActorLocation().Equals(Vehicle->CargoPosition(),10.f) || Vehicle->Speed>0)
            { Stop(TEXT("STS / AGV handover alignment")); return; }
            Cargo->GetBody()->SetSimulatePhysics(false);
            // The physical landing check accepts up to 10 cm. Resolve that
            // remaining contact offset onto the AGV deck datum before attaching,
            // otherwise KeepWorldTransform preserves a mismatch that the 1 cm
            // ownership invariant rejects on the same frame.
            Cargo->SetActorLocationAndRotation(Vehicle->CargoPosition(),Vehicle->GetActorRotation(),false,nullptr,ETeleportType::TeleportPhysics);
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
            Manifest[Job.Cargo].STSHandoverAt=Job.HandoverAt;
            Manifest[Job.Cargo].STSServiceSeconds=Job.STSSeconds;
            UE_LOG(LogTemp,Display,TEXT("SITE_HANDOVER: C%d STS -> AGV%d"),Manifest[Job.Cargo].ID,100+Lane);
            if(Job.ExtraCargo!=INDEX_NONE)
            {
                // Re-evaluate with the measured first STS service time. Do not
                // wait for a second box if this trip no longer saves modelled time.
                const double ProductiveService=FMath::Max(0.,Job.STSSeconds-Manifest[Job.Cargo].STSWaitingForVehicleSeconds);
                const double Saving=Manifest[Job.Cargo].PlannedPairSavingSeconds+150.-ProductiveService;
                Manifest[Job.Cargo].EstimatedPairSavingSeconds=Saving;
                Manifest[Job.ExtraCargo].EstimatedPairSavingSeconds=Saving;
                if(Saving<YardStackingPolicy::MinimumPairSavingSeconds)
                {
                    Manifest[Job.ExtraCargo].State=0; Manifest[Job.ExtraCargo].VehicleAssignedAt=-1;
                    --Dispatched; Job.ExtraCargo=INDEX_NONE;
                    UE_LOG(LogTemp,Display,TEXT("AGV_PAIR_DECLINED: V%d productive_sts=%.1f saving=%.1f"),100+Lane,ProductiveService,Saving);
                }
                else
                {
                    auto& Extra=Manifest[Job.ExtraCargo];
                    Vehicle->ActiveDeckOffsetCm=YardStackingPolicy::PairPadOffsetCm;
                    if(!Equipment[YardCraneCount+Job.STS]->AssignCargo(Extra.Actor.Get(),Extra.Actor->GetActorLocation(),
                        Vehicle->CargoPosition(),false,false,Vehicle))
                    { Stop(TEXT("STS could not load second 20ft container")); return; }
                    Job.Stage=10; Job.bLoadingExtra=true; Job.bPairedTrip=true; ++PairedTrips;
                    UE_LOG(LogTemp,Display,TEXT("AGV_PAIR_LOADING: V%d C%d+C%d estimated_saving=%.1f"),100+Lane,
                        Manifest[Job.Cargo].ID,Extra.ID,Saving);
                    break;
                }
            }
            Job.Stage=2;
            PrepareNextCargo(Job.STS);
            break;
        case 10:
        {
            if(Equipment[YardCraneCount+Job.STS]->IsBusy()) break;
            auto& Extra=Manifest[Job.ExtraCargo]; auto* Second=Extra.Actor.Get();
            if(!Second || !Second->GetActorLocation().Equals(Vehicle->CargoPosition(),10.f) || Vehicle->Speed>0)
            { Stop(TEXT("Second 20ft STS/AGV handover alignment")); return; }
            Second->GetBody()->SetSimulatePhysics(false);
            Second->SetActorLocationAndRotation(Vehicle->CargoPosition(),Vehicle->GetActorRotation(),false,nullptr,ETeleportType::TeleportPhysics);
            Second->AttachToComponent(Vehicle->GetRootComponent(),FAttachmentTransformRules::KeepWorldTransform);
            Second->LocationOwner=ECargoOwner::AGV; Extra.HandoverMask|=1;
            Extra.STSHandoverAt=SimulationTime; Extra.STSServiceSeconds=Equipment[YardCraneCount+Job.STS]->LastJobSeconds;
            Vehicle->SetPayload(Cargo->MassKg+Second->MassKg);
            if(!Vehicle->PayloadWithinReferenceLimit()) { Stop(TEXT("Paired AGV payload exceeds reference")); return; }
            RecordVesselEvent(Job.ExtraCargo,false);
            Vehicle->ActiveDeckOffsetCm=Job.CargoDeckOffsetCm;
            Job.bLoadingExtra=false; Job.Stage=2;
            PrepareNextCargo(Job.STS);
            break;
        }
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
                RoadReservations.Remove(ID); RoadReservationEndWaypoint.Remove(ID); ReservationIndex.Remove(ID); RoadTargets.Remove(ID);
                FinishedRoadSegments.Remove(ID);
                break;
            }
            if(LaneCount!=9 && Job.ExtraCargo==INDEX_NONE)
                YardApproachOwners[YardApproachKey(Yard[Job.Slot])]=INDEX_NONE;
            // Keep the twist locks engaged until the RMG actually picks up the box.
            if(!CraneAvailable(Job.RMG)) { Job.Stage=9; Job.bEquipmentWaiting=true; Job.RecoveryAnchor=Vehicle->GetActorLocation(); break; }
            if (!Equipment[Job.RMG]->AssignCargo(Cargo,Vehicle->CargoPosition(),Yard[Job.Slot].Position,false,true,Vehicle))
            { Stop(TEXT("AGV / RMG reservation handover")); return; }
            Manifest[Job.Cargo].HandoverMask|=2;
            UE_LOG(LogTemp,Display,TEXT("SITE_HANDOVER: C%d AGV%d -> RMG%d"),Manifest[Job.Cargo].ID,100+Lane,Job.RMG+1);
            Job.Stage=4; break;
        case 4:
        {
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
            const int32 BelowSlot=Yard[Job.Slot].Below;
            const int32 BelowCargo=BelowSlot==INDEX_NONE?INDEX_NONE:Manifest.IndexOfByPredicate(
                [BelowSlot](const FSiteShipCargo& C){return C.PlannedSlot==BelowSlot;});
            const FString BelowID=BelowCargo==INDEX_NONE?FString():Manifest[BelowCargo].Actor->ContainerID.ToString();
            int32 YardTier=1;
            for(int32 Lower=BelowSlot;Lower!=INDEX_NONE;Lower=Yard[Lower].Below) ++YardTier;
            ResultsCsv+=FString::Printf(TEXT("C%d,%d,%.0f,%.0f,%.0f,%d,%d,%d,%d,%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%s,%s,%s,%d,%s,%d,%d,%d,%d,%.6f,%s,%d,%s,%d,%d,%d,%.3f,%.3f,%.3f,%.6f,%.6f,%.6f,%.6f\n"),
                Manifest[Job.Cargo].ID,Cargo->LengthFt,Cargo->TareMassKg,Cargo->CargoMassKg,Cargo->MassKg,
                Yard[Job.Slot].Block+1,Job.Slot,Job.STS+1,100+Lane,Job.RMG+1,Job.StartedAt,Job.HandoverAt,SimulationTime,
                Job.Time,Job.STSSeconds,Job.PausedSeconds,Job.TrafficWaitSeconds,Job.YardWaitSeconds,
                *SiteLogistics::CsvField(Cargo->CargoInformation.Goods),*SiteLogistics::CsvField(Cargo->CargoInformation.HandlingType),*SiteLogistics::CsvField(Cargo->CargoInformation.HazardGroup),
                Cargo->CargoInformation.OutboundOrder,*SiteLogistics::CsvField(Cargo->CargoInformation.DestinationPort),Cargo->CargoInformation.PortCallOrder,
                Job.TripID,Job.TripCargoOrdinal,Job.bPairedTrip?1:0,Manifest[Job.Cargo].EstimatedPairSavingSeconds,
                *SiteLogistics::CsvField(Cargo->CargoInformation.DataBasis),YardTier,*SiteLogistics::CsvField(BelowID),
                Yard[Job.Slot].Cell,Yard[Job.Slot].LengthFt==20?Yard[Job.Slot].Subslot+1:0,Yard[Job.Slot].LengthFt,
                Yard[Job.Slot].Position.X,Yard[Job.Slot].Position.Y,Yard[Job.Slot].Position.Z,
                Manifest[Job.Cargo].DispatchEligibleAt,Manifest[Job.Cargo].VehicleAssignedAt,
                FMath::Max(0.,Manifest[Job.Cargo].VehicleAssignedAt-Manifest[Job.Cargo].DispatchEligibleAt),
                Manifest[Job.Cargo].STSWaitingForVehicleSeconds);
            if(!SaveReports()) { Stop(TEXT("Could not save site STS shipment report")); return; }
            UE_LOG(LogTemp,Display,TEXT("SITE_DELIVERED: C%d via AGV%d -> RMG%d; total=%d"),Manifest[Job.Cargo].ID,100+Lane,Job.RMG+1,Delivered);
            if(Job.ExtraCargo!=INDEX_NONE)
            {
                const int32 Second=Job.ExtraCargo;
                RMGBusy[Job.RMG]=false;
                Job.Cargo=Second; Job.Actor=Manifest[Second].Actor; Job.ExtraCargo=INDEX_NONE;
                Job.Slot=Manifest[Second].PlannedSlot; Job.bRMGReserved=false; Job.bYardReleased=false;
                SlotAssigned[Job.Slot]=true;
                Job.CargoDeckOffsetCm=YardStackingPolicy::PairPadOffsetCm; Job.TripCargoOrdinal=2;
                Vehicle->ActiveDeckOffsetCm=Job.CargoDeckOffsetCm; Vehicle->SetPayload(Job.Actor->MassKg);
                Job.StartedAt=Manifest[Second].StartedAt; Job.Time=SimulationTime-Job.StartedAt;
                Job.HandoverAt=Manifest[Second].STSHandoverAt; Job.STSSeconds=Manifest[Second].STSServiceSeconds;
                Job.Stage=9;
                break;
            }
            Vehicle->ActiveDeckOffsetCm=0;
            Job.Actor=nullptr;
            AssignReturnSTS(Lane);
            if(!Fault.IsEmpty()) return;
            PrepareRoute(Lane,true); Job.Stage=5; break;
        }
        case 5:
            if (!Drive(Lane,Dt)) break;
            if (!Job.bYardReleased) RMGBusy[Job.RMG]=false;
            if(YardApproachOwners[YardApproachKey(Yard[Job.Slot])]==Lane)
                YardApproachOwners[YardApproachKey(Yard[Job.Slot])]=INDEX_NONE;
            if(Job.ReturnSTS!=INDEX_NONE)
            {
                const int32 STS=Job.ReturnSTS;
                if(NextVehicles[STS]!=Lane || PreparedCargo[STS]!=Job.ReturnCargo)
                { Stop(TEXT("Direct return lost assigned STS cargo")); return; }
                if(!Vehicle->CanAcceptWork() || STSOwners[STS]!=INDEX_NONE || Equipment[YardCraneCount+STS]->IsBusy() ||
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
    PairedTrips=0; NextTripID=1;
    DashboardControlSession=FGuid::NewGuid().ToString(EGuidFormats::Digits);
    DashboardCommandResults.Reset(); SeenDashboardCommands.Reset(); RecoveryEvents.Reset(); RecoverySequence=0; NextControlPoll=0;
    if(!InitialManifestPlan.IsEmpty()) Manifest=InitialManifestPlan;
    RebuildSTSStreams(); IsolatedEquipment.Reset(); DeferredYardLeases.Reset(); bRecoveryRailGuard=false;
    NextRecoveryAttempt=0; bFailureInjected=bFailureRepaired=false;
    EquipmentFaultEvents=STSReassignments=YardReassignments=0; DispatchPriorityLimit=INDEX_NONE;
    PendingSimulationSeconds=0;
    KinematicPhysicsPoses.Reset();
    TrafficRecoveryTargets.Reset(); TrafficRecoveryOriginalReverse.Reset(); TrafficRecoveryYieldTo.Reset(); LastTrafficRecoveryAt=-100;
    for (auto& Job:Jobs) if (Job.Actor.IsValid()) Job.Actor->DetachFromActor(FDetachmentTransformRules::KeepWorldTransform);
    PlacedContainers.Reset();
    for (const auto& Crane:Equipment) Crane->ResetOperation();
    for(auto& Booking:ExportCargo)
    {
        auto* Actor=Booking.Actor.Get();
        Actor->DetachFromActor(FDetachmentTransformRules::KeepWorldTransform);
        Actor->GetBody()->SetSimulatePhysics(false);
        Actor->SetActorLocationAndRotation(Yard[Booking.PlannedSlot].Position,FRotator(0,90,0),false,nullptr,ETeleportType::TeleportPhysics);
        Actor->LocationOwner=ECargoOwner::Yard;
        Yard[Booking.PlannedSlot].Occupied=true;
        Booking.State=0; Booking.HandoverMask=0;
    }
    for (auto& Cargo:Manifest)
    {
        auto* Actor=Cargo.Actor.Get();
        check(Actor);
        Actor->DetachFromActor(FDetachmentTransformRules::KeepWorldTransform);
        Actor->GetBody()->SetSimulatePhysics(false);
        FVector Position=Cargo.Transform.GetLocation(),Velocity=FVector::ZeroVector,Acceleration=FVector::ZeroVector,Jerk=FVector::ZeroVector;FQuat Rotation=Cargo.Transform.GetRotation();
        ResolveVesselTransform(Cargo,Position,Rotation,Velocity,Acceleration,Jerk);
        Actor->SetSecuredVesselMotion(Position,Rotation,Velocity,Acceleration,Jerk);
        Actor->LocationOwner=ECargoOwner::Ship;
        Cargo.State=0; Cargo.HandoverMask=0;
        Cargo.STSHandoverAt=-1; Cargo.STSServiceSeconds=0;
        Cargo.STSWaitingForVehicleSeconds=0;
        Cargo.DispatchEligibleAt=-1; Cargo.VehicleAssignedAt=-1;
        Cargo.EstimatedPairSavingSeconds=Cargo.PlannedPairSavingSeconds;
        Actor->SetActorHiddenInGame(Cargo.Wave>0); Actor->SetActorEnableCollision(Cargo.Wave==0);
    }
    for (auto& Slot:Yard) if (Slot.Reserved)
    { Slot.Occupied=false; Slot.Mesh->UpdateInstanceTransform(Slot.Instance,SiteLogistics::Hidden(SiteLogistics::YardTransform(Slot)),false,true,true); }
    for (int32 I=0;I<Jobs.Num();++I) { Jobs[I]=FSiteTransfer(); Vehicles[I]->ResetVehicle(FleetPark(I)); }
    STSOwners.Init(INDEX_NONE,LaneCount);
    NextVehicles.Init(INDEX_NONE,LaneCount); PreparedStarted.Init(false,LaneCount); QueuedHandoffs=0;
    NoProgressTime=0; LastProgressDelivered=0; RoadBlockers.Reset(); RoadWaitSince.Reset(); TrafficClock=0; LastCycleRecoveryAt=-10;
    YardApproachOwners.Init(INDEX_NONE,LaneCount==9?YardCraneCount:TerminalLayout::YardBlockCount); BlocksBusy.Init(false,TerminalLayout::YardBlockCount); RMGBusy.Init(false,YardCraneCount); PreparedCargo.Init(INDEX_NONE,LaneCount); SlotAssigned.Init(false,Yard.Num());
    CentralReservation=CentralPending=INDEX_NONE;
    ReservationIndex.Reset(); VehicleIndex.Reset(); bTrafficIndexReady=false;
    RoadReservations.Reset(); RoadReservationEndWaypoint.Reset(); RoadTargets.Reset(); FinishedRoadSegments.Reset(); PeakMovingVehicles=PrefetchedJobs=Dispatched=Delivered=0;
    PendingRoadRequests.Reset(); PendingRoadIndex.Reset();
    for(int32& Count:ReturnRouteCounts) Count=0;
    for(auto& Load:CentralRoadTravelCm) for(double& Distance:Load) Distance=0;
    Fault.Empty(); bWasPaused=false;
    BeginReport();
    if(!STSProfile.bReady) Stop(TEXT("Site STS reference unavailable"));
}

int32 APortSiteLogistics::ShipRemaining() const
{
    int32 Count=0;
    for (const auto& Cargo:Manifest) Count+=Cargo.Actor.IsValid() && Cargo.Actor->LocationOwner==ECargoOwner::Ship;
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
            const auto& P=Cargo.ShipSlot;
            const FVector Initial((TerminalLayout::VesselCenterXM+P.PositionM.Y)*100.,
                (TerminalLayout::VesselCenterY(Cargo.STS/3)+P.PositionM.X)*100.,P.PositionM.Z*100.);
            if(Cargo.Wave!=0 || P.CargoID!=Cargo.ID || P.Tier<1 || !Initial.Equals(Cargo.Transform.GetLocation(),.1))
            { Error=TEXT("Ship arrival actor/slot plan mismatch"); return false; }
            ++VesselCounts[Cargo.STS/3];
        }
        TSet<int32> ExportPads;
        for(const auto& Booking:ExportCargo)
        {
            const auto* Actor=Booking.Actor.Get();
            if(Booking.STS<0 || Booking.STS>=9 || !Actor || !Yard.IsValidIndex(Booking.PlannedSlot))
            { Error=TEXT("Invalid export yard booking"); return false; }
            const auto& Slot=Yard[Booking.PlannedSlot];
            if(ExportPads.Contains(Booking.PlannedSlot) || Slot.Reserved ||
                Slot.Occupied!=((Booking.HandoverMask&1)==0) || Actor->ContainerID!=Booking.ContainerName ||
                (Booking.State==0 && (Actor->LocationOwner!=ECargoOwner::Yard ||
                    !Actor->GetActorLocation().Equals(Slot.Position,.1))) ||
                (Booking.State==2 && Actor->LocationOwner!=ECargoOwner::Ship))
            { Error=TEXT("Export yard inventory mismatch"); return false; }
            ExportPads.Add(Booking.PlannedSlot);
        }
        for(const auto& Booking:RetainedCargo)
        {
            if(!Booking.Actor.IsValid() || Booking.Actor->LocationOwner!=ECargoOwner::Ship)
            { Error=TEXT("Retained ship cargo ownership mismatch"); return false; }
            ++VesselCounts[Booking.STS/3];
        }
        if (CentralCount!=0 || VesselCounts[0]!=ConfiguredCargo[0] || VesselCounts[1]!=(ConfiguredShips>1?ConfiguredCargo[1]:0) ||
            VesselCounts[2]!=(ConfiguredShips>2?ConfiguredCargo[2]:0) || ReceivingCapacity!=Yard.Num()-ExportCargo.Num() || Vehicles.Num()!=60)
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
            if(Vehicle->Sensors.bControlledStop && !Vehicle->Emergency.Active && Vehicle->Speed>.1f)
            { Error=FString::Printf(TEXT("AGV %d moved through a LiDAR controlled stop"),Vehicle->VehicleID); return false; }
            if(!Vehicle->Dynamics.Finite() || !Vehicle->Dynamics.Fault.IsEmpty())
            { Error=FString::Printf(TEXT("AGV %d tire/roll physics invalid: %s"),Vehicle->VehicleID,*Vehicle->Dynamics.Fault); return false; }
            // Physical side slip is permitted; commanded crab motion is not.
            if(AGVReference::LateralMotionEnabled)
            { Error=TEXT("Commanded AGV crab motion must remain disabled"); return false; }
            if(Vehicle->Dynamics.OverturnThreshold)
            { Error=FString::Printf(TEXT("AGV %d wheel contact/roll stability threshold exceeded"),Vehicle->VehicleID); return false; }
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
    TMap<int32,int32> CargoAtPlannedSlot;
    TMap<int32,double> PlannedSuperimposedKg;
    for(int32 I=0;I<Manifest.Num();++I) CargoAtPlannedSlot.Add(Manifest[I].PlannedSlot,I);
    for(const auto& C:Manifest) if(Yard.IsValidIndex(C.PlannedSlot))
        for(int32 Lower=Yard[C.PlannedSlot].Below;Lower!=INDEX_NONE;Lower=Yard[Lower].Below)
            PlannedSuperimposedKg.FindOrAdd(Lower)+=C.Actor->MassKg;
    if (ShipContainers.Num()!=Manifest.Num()+RetainedCargo.Num()+ExportCargo.Num()) { Error=TEXT("Cargo actor/manifest count mismatch" ); return false; }
    for (const auto& Cargo:Manifest)
    {
        const auto* Actor=Cargo.Actor.Get();
        if (!IsValid(Actor) || Actor->GetOwner()!=this || Actors.Contains(Actor) ||
            Actor->ContainerID!=Cargo.ContainerName || Cargo.STS<0 || Cargo.STS>=LaneCount)
        { Error=TEXT("Missing, replaced, duplicate or unassigned ship container actor"); return false; }
        if(!Yard.IsValidIndex(Cargo.PlannedSlot) || !Yard[Cargo.PlannedSlot].Reserved || Yard[Cargo.PlannedSlot].Central ||
            PlannedDestinations.Contains(Cargo.PlannedSlot))
        { Error=TEXT("Missing, duplicate or invalid pre-discharge yard destination"); return false; }
        PlannedDestinations.Add(Cargo.PlannedSlot);
        if(LaneCount==9)
        {
            const auto& Slot=Yard[Cargo.PlannedSlot];
            if(Slot.LengthFt!=Actor->LengthFt)
            { Error=TEXT("Container assigned outside its dedicated size bay"); return false; }
            if(Cargo.PlanningOrder==INDEX_NONE)
            { Error=TEXT("Yard plan missing its discharge order"); return false; }
            if(!Actor->CargoInformation.Valid() || !YardStackingPolicy::ZoneAccepts(Slot.Block,Actor->CargoInformation))
            { Error=TEXT("Yard booking/handling zone invalid"); return false; }
            if(PlannedSuperimposedKg.FindRef(Cargo.PlannedSlot)>Actor->CargoInformation.PermittedSuperimposedKg)
            { Error=TEXT("Yard cumulative superimposed load exceeds the booking limit"); return false; }
            if(Slot.Below!=INDEX_NONE)
            {
                if(Slot.LengthFt==20)
                {
                    if(!Yard.IsValidIndex(Slot.Partner))
                    { Error=TEXT("20ft upper layer has no partner pad"); return false; }
                    const int32 OtherBelow=Yard[Slot.Partner].Below;
                    const int32* OtherIndex=CargoAtPlannedSlot.Find(OtherBelow);
                    if(!OtherIndex || Manifest[*OtherIndex].PlanningOrder>=Cargo.PlanningOrder)
                    { Error=TEXT("20ft upper layer scheduled before both lower pads"); return false; }
                }
                const int32* LowerIndex=CargoAtPlannedSlot.Find(Slot.Below);
                if(!LowerIndex) { Error=TEXT("Yard plan has an unknown supporting container"); return false; }
                const auto* Lower=Manifest[*LowerIndex].Actor.Get();
                if(Manifest[*LowerIndex].PlanningOrder>=Cargo.PlanningOrder)
                { Error=TEXT("Yard support is scheduled after its upper container"); return false; }
                if(!YardStackingPolicy::MayStack(Lower->CargoInformation,Lower->LengthFt,Lower->MassKg,
                    Actor->CargoInformation,Actor->LengthFt,Actor->MassKg))
                { Error=TEXT("Yard outbound/type/size/weight stack constraint violated"); return false; }
            }
        }
        if((Actor->LengthFt!=20 && Actor->LengthFt!=40 && Actor->LengthFt!=45) ||
            !FMath::IsNearlyEqual(Actor->MassKg,Actor->TareMassKg+Actor->CargoMassKg,1.f) ||
            !FMath::IsFinite(Actor->MassKg) || Actor->MassKg<=0)
        { Error=TEXT("Invalid container size or gross mass specification"); return false; }
        Actors.Add(Actor);
        FVector ExpectedPosition=Cargo.Transform.GetLocation(),ExpectedVelocity=FVector::ZeroVector,ExpectedAcceleration=FVector::ZeroVector,ExpectedJerk=FVector::ZeroVector;FQuat ExpectedRotation=Cargo.Transform.GetRotation();
        ResolveVesselTransform(Cargo,ExpectedPosition,ExpectedRotation,ExpectedVelocity,ExpectedAcceleration,ExpectedJerk);
        if (Cargo.State==0 && (Actor->LocationOwner!=ECargoOwner::Ship || Actor->GetAttachParentActor() ||
            Actor->GetBody()->IsSimulatingPhysics() || !Actor->GetActorLocation().Equals(ExpectedPosition,.5f) ||
            FQuat::ErrorAutoNormalize(Actor->GetActorQuat(),ExpectedRotation)>.0001f))
        {
            Error=FString::Printf(TEXT("Secured ship cargo %d mismatch: owner=%d attached=%d physics=%d position_error_cm=%.4f rotation_error=%.6f"),
                Cargo.ID,int32(Actor->LocationOwner),Actor->GetAttachParentActor()!=nullptr,Actor->GetBody()->IsSimulatingPhysics(),
                FVector::Distance(Actor->GetActorLocation(),ExpectedPosition),FQuat::ErrorAutoNormalize(Actor->GetActorQuat(),ExpectedRotation));
            return false;
        }
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
        if(LaneCount==9 && Slot.LengthFt==20)
        {
            if(!Yard.IsValidIndex(Slot.Partner) || Yard[Slot.Partner].Cell!=Slot.Cell ||
                Yard[Slot.Partner].LengthFt!=20 || Yard[Slot.Partner].Subslot==Slot.Subslot ||
                !FMath::IsNearlyEqual(FMath::Abs(Slot.Position.X-Yard[Slot.Partner].Position.X),613.4f,.1f) ||
                !FMath::IsNearlyEqual(Slot.Position.Y,Yard[Slot.Partner].Position.Y,.01f) ||
                !FMath::IsNearlyEqual(Slot.Position.Z,Yard[Slot.Partner].Position.Z,.01f))
            { Error=TEXT("20ft cell is missing its separate longitudinal partner pad"); return false; }
            if(Slot.Occupied && Slot.Below!=INDEX_NONE &&
                (Yard[Slot.Partner].Below==INDEX_NONE || !Yard[Yard[Slot.Partner].Below].Occupied))
            { Error=TEXT("20ft upper layer placed before the lower pair was complete"); return false; }
        }
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
    TSet<int32> OccupiedChargers;
    int32 Physical=0;
    for (int32 Lane=0;Lane<Jobs.Num();++Lane)
    {
        const auto& Job=Jobs[Lane];
        const auto* BatteryVehicle=Vehicles[Lane].Get();
        if(!FMath::IsFinite(BatteryVehicle->BatteryPercent) || BatteryVehicle->BatteryPercent<0 ||
            BatteryVehicle->BatteryPercent>APortAGVActor::ChargeLimitPercent)
        { Error=TEXT("AGV battery outside 0..80 percent"); return false; }
        if(Job.ChargingBay!=INDEX_NONE)
        {
            if(OccupiedChargers.Contains(Job.ChargingBay))
            { Error=TEXT("Two AGVs reserved the same charger"); return false; }
            OccupiedChargers.Add(Job.ChargingBay);
        }
        if (!Job.Stage) continue;
        if(Job.ExportIndex!=INDEX_NONE)
        {
            if(!ExportCargo.IsValidIndex(Job.ExportIndex) ||
                (Job.Actor.IsValid() && Job.Actor!=ExportCargo[Job.ExportIndex].Actor) ||
                (Job.Actor.IsValid() && Job.Actor->LocationOwner==ECargoOwner::AGV &&
                    (Job.Actor->GetAttachParentActor()!=Vehicles[Lane] ||
                        !Job.Actor->GetActorLocation().Equals(Vehicles[Lane]->CargoPosition(),1.f))))
            { Error=TEXT("Export transfer cargo ownership mismatch"); return false; }
            continue;
        }
        if(Job.Stage>=FSiteTransfer::ToCharge)
        {
            const auto* V=Vehicles[Lane].Get();
            if(Job.Actor.IsValid() || V->PayloadKg>.1f || Job.Stage>FSiteTransfer::Charging ||
                (Job.Stage!=FSiteTransfer::ReturnToStandby && (Job.ChargingBay<0 || Job.ChargingBay>=TerminalLayout::AGVChargeBays)))
            { Error=TEXT("Invalid charging assignment"); return false; }
            if(Job.Stage==FSiteTransfer::Charging && (V->Speed>.1f || !V->GetActorLocation().Equals(
                ChargingBayPosition(Job.ChargingBay),AGVReference::DockingAccuracyCm+1.f)))
            { Error=TEXT("Charging outside stationary assigned bay"); return false; }
            continue;
        }
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
        const FVector Handover=TransferQuay(Job);
        if (Job.Stage==6 && (Job.Route.IsEmpty() || !Job.Route.Last().Equals(Handover,.1f)))
        { Error=TEXT("AGV approach does not end at the cargo-specific handover"); return false; }
        if ((Job.Stage==1 || Job.Stage==2 || Job.Stage==10) &&
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
        if(Job.bYardAdmissionSuspended &&
            ((Job.Stage!=6 && Job.Stage!=1 && Job.Stage!=10) || Job.bRMGReserved || Job.bYardReleased ||
                !Yard.IsValidIndex(Job.Slot) || !SlotAssigned[Job.Slot] || !Job.bEquipmentWaiting ||
                ((Job.Stage==1 || Job.Stage==10) &&
                    FVector::Dist2D(Vehicles[Lane]->GetActorLocation(),YardHandover(Yard[Job.Slot]))<=4000.)))
        { Error=TEXT("Invalid suspended yard admission at failed STS pickup"); return false; }
        if(Job.Slot!=INDEX_NONE && !Job.bRMGReserved && !Job.bYardAdmissionSuspended &&
            ((Job.Stage!=6 && Job.Stage!=1 && Job.Stage!=2 && Job.Stage!=3 && Job.Stage!=9 && Job.Stage!=10) ||
                YardApproachOwners[YardApproachKey(Yard[Job.Slot])]!=Lane))
        { Error=TEXT("Missing yard approach admission"); return false; }
        if(Job.Stage==9)
        {
            const bool AtEntry=Job.Waypoint==Job.YardEntryWaypoint+1 &&
                Job.Route.IsValidIndex(Job.Waypoint-1) && Vehicles[Lane]->GetActorLocation().Equals(
                    Job.Route[Job.Waypoint-1],AGVReference::DockingAccuracyCm);
            const bool AtPairedDock=Job.bPairedTrip && Job.TripCargoOrdinal==2 &&
                Yard.IsValidIndex(Job.Slot) && Job.Waypoint==Job.Route.Num() &&
                Vehicles[Lane]->GetActorLocation().Equals(YardHandover(Yard[Job.Slot]),AGVReference::DockingAccuracyCm);
            const bool AtRecoveryAnchor=(Job.bEquipmentWaiting || Job.bRecoveryRoutePending) &&
                Vehicles[Lane]->GetActorLocation().Equals(Job.RecoveryAnchor,AGVReference::DockingAccuracyCm);
            if(Vehicles[Lane]->Speed>.1f || (!AtEntry && !AtPairedDock && !AtRecoveryAnchor))
            { Error=TEXT("AGV is not stopped inside its yard entry bay or second-container dock"); return false; }
        }
        if (Job.Actor.IsValid())
        {
            ++Physical;
            FVector ExpectedDeck=Vehicles[Lane]->DeckPosition(Job.CargoDeckOffsetCm);
            if(Vehicles[Lane]->HasValidEmergencyCoG(Job.Actor.Get(),STSProfile.ContainerCoG))
                ExpectedDeck+=(Vehicles[Lane]->GetActorRightVector()*Vehicles[Lane]->Dynamics.CargoOffsetM.X-
                    Vehicles[Lane]->GetActorForwardVector()*Vehicles[Lane]->Dynamics.CargoOffsetM.Y)*100;
            if (Job.Actor!=Manifest[Job.Cargo].Actor || Job.Actor->ContainerID!=Manifest[Job.Cargo].ContainerName)
            { Error=TEXT("Cargo identity changed during handover"); return false; }
            if ((Job.Stage==2 || Job.Stage==3 || Job.Stage==9 || Job.Stage==10) && (Job.Actor->LocationOwner!=ECargoOwner::AGV || Job.Actor->GetAttachParentActor()!=Vehicles[Lane] ||
                !Job.Actor->GetActorLocation().Equals(ExpectedDeck,1.f)))
            { Error=TEXT("AGV cargo alignment/ownership mismatch"); return false; }
        }
        if(Job.ExtraCargo!=INDEX_NONE)
        {
            if(!Manifest.IsValidIndex(Job.ExtraCargo) || JobCargo.Contains(Job.ExtraCargo) ||
                Manifest[Job.ExtraCargo].State!=1 || Manifest[Job.ExtraCargo].Actor->LengthFt!=20 ||
                Job.Actor->LengthFt!=20 || Job.ExtraCargo==Job.Cargo)
            { Error=TEXT("Invalid paired AGV manifest reservation"); return false; }
            JobCargo.Add(Job.ExtraCargo); ++Physical;
            const auto* Extra=Manifest[Job.ExtraCargo].Actor.Get();
            if((Job.Stage==2 || Job.Stage==3 || Job.Stage==9 || Job.Stage==4) &&
                (Extra->LocationOwner!=ECargoOwner::AGV || Extra->GetAttachParentActor()!=Vehicles[Lane] ||
                    !Extra->GetActorLocation().Equals(Vehicles[Lane]->DeckPosition(YardStackingPolicy::PairPadOffsetCm),1.f)))
            { Error=TEXT("Second 20ft AGV ownership/alignment mismatch"); return false; }
            if(Extra->MassKg+Job.Actor->MassKg>AGVReference::MaximumPayloadKg)
            { Error=TEXT("Paired vehicle exceeds payload capacity"); return false; }
        }
        if (Vehicles[Lane]->Speed>0 && !RoadReservations.Contains(Vehicles[Lane]->VehicleID))
        { Error=TEXT("AGV moved without a path reservation"); return false; }
    }
    if (PlacedContainers.Num()!=Delivered) { Error=TEXT("Delivered cargo actor count mismatch" ); return false; }
    TSet<const APortContainerActor*> PlacedIDs;
    for (const auto& Cargo:PlacedContainers)
    {
        if (!IsValid(Cargo) || Cargo->GetOwner()!=this || Cargo->LocationOwner!=ECargoOwner::Yard || Cargo->GetAttachParentActor() || PlacedIDs.Contains(Cargo))
        { Error=TEXT("Lost/duplicate/attached yard cargo actor" ); return false; }
        PlacedIDs.Add(Cargo.Get());
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
    if(LaneCount==9) for(int32 STS=0;STS<LaneCount;++STS)
    {
        const auto* Crane=Equipment[YardCraneCount+STS].Get();
        if(!Crane->IsBusy()) continue;
        const int32 AssignedVehicle=STSOwners[STS];
        if(!Jobs.IsValidIndex(AssignedVehicle) || Jobs[AssignedVehicle].Slot==INDEX_NONE ||
            (Jobs[AssignedVehicle].Actor.Get()!=Crane->CargoActor &&
                (!Manifest.IsValidIndex(Jobs[AssignedVehicle].ExtraCargo) || Manifest[Jobs[AssignedVehicle].ExtraCargo].Actor.Get()!=Crane->CargoActor)))
        { Error=TEXT("STS started pickup without its AGV and downstream admission"); return false; }
    }
    return true;
}

void APortSiteLogistics::EndPlay(const EEndPlayReason::Type Reason)
{
    RetainedCargo.Reset();
    ExportCargo.Reset();
    if (DashboardWrite.IsValid()) DashboardWrite.Wait();
    for (const auto& Cargo:ShipContainers) if (IsValid(Cargo)) Cargo->Destroy();
    PlacedContainers.Reset(); ShipContainers.Reset();
    for (const auto& Vehicle:Vehicles) if (IsValid(Vehicle)) Vehicle->Destroy();
    Super::EndPlay(Reason);
}

bool APortSiteLogistics::IsIdle() const
{
    for(const auto& Cargo:ExportCargo) if(Cargo.State!=2) return false;
    for (const auto& Job:Jobs) if (Job.Stage) return false;
    if(LaneCount==9) for(const auto& Vehicle:Vehicles) if(Vehicle->bNeedsCharge) return false;
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
    ResultsCsv=TEXT("ContainerID,LengthFt,TareKg,CargoKg,GrossKg,PlannedYardBlock,PlannedSlot,STSLane,AGVID,RMGID,StartedAtSeconds,STSHandoverAtSeconds,FinalPlacementAtSeconds,ShipmentSeconds,STSSeconds,PausedSeconds,TrafficWaitSeconds,YardWaitSeconds,Goods,HandlingType,HazardGroup,OutboundOrder,DestinationPort,PortCallOrder,TripID,TripCargoOrdinal,PairedTrip,EstimatedPairSavingSeconds,DataBasis,PlannedYardTier,BelowContainerID,PlannedYardCell,YardSubslot,SlotLengthFt,PlannedXcm,PlannedYcm,PlannedZcm,DispatchEligibleAtSeconds,AGVAssignedAtSeconds,DispatchWaitSeconds,STSWaitingForAGVSeconds\n");
    if(!SaveReports()) Stop(TEXT("Could not initialize site shipment report"));
    UE_LOG(LogTemp,Display,TEXT("SITE_REPORT: %s.csv"),*ReportBase);
}

bool APortSiteLogistics::SaveReports() const
{
    if(ReportBase.IsEmpty()) return false;
    IFileManager::Get().MakeDirectory(*FPaths::GetPath(ReportBase),true);
    const FString Snapshot=TEXT("{\"site_suspension_model\":\"reduced-order sway/yaw; assumed mounts; taut-rope tension; hoist shaft torque\",\"sts_profile\":")+
        (STSProfile.SnapshotJson.IsEmpty()?TEXT("{}"):STSProfile.SnapshotJson)+
        TEXT(",\"ship_loading_assumptions\":")+(ShipStowageAssumptions.SnapshotJson.IsEmpty()?TEXT("{}"):ShipStowageAssumptions.SnapshotJson)+
        TEXT(",\"ship_arrival_layout\":\"")+(LaneCount==9?TEXT("applied_bay_row_tier"):TEXT("legacy_grid"))+TEXT("\"")+
        TEXT(",\"agv_parking_layout\":\"")+(bDepotParking?TEXT("depot"):TEXT("legacy_distributed"))+
        TEXT("\",\"agv_apron_layout\":\"")+(bReuseFreedApron?TEXT("reused_transport_lanes"):TEXT("previous_routes"))+
        TEXT("\",\"agv_dispatch_priority\":\"")+(bReuseFreedApron?TEXT("yard_support_path"):TEXT("oldest_ready"))+
        TEXT("\",\"agv_road_priority\":\"")+(bPickupRoadPriority?TEXT("pickup_with_aging"):TEXT("fifo"))+
        FString::Printf(TEXT("\",\"equipment_fault_events\":%d,\"sts_reassignments\":%d,\"yard_reassignments\":%d}"),EquipmentFaultEvents,STSReassignments,YardReassignments);
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
