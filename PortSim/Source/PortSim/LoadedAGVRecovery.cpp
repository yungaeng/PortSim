#include "PortSiteLogistics.h"
#include "PortAGVActor.h"
#include "PortSupportVehicle.h"
#include "PortContainerActor.h"
#include "PortWorkingCrane.h"
#include "AGVRoadNetwork.h"
#include "AGVApronRoads.h"
#include "AGVTrafficGeometry.h"
#include "AGVReference.h"
#include "TerminalLayout.h"
#include "Components/PrimitiveComponent.h"
#include "Components/StaticMeshComponent.h"
#include "EngineUtils.h"

namespace SiteLogistics
{
    TArray<FVector> RoundRoadCorners(const FVector&,const TArray<FVector>&,TFunctionRef<bool(FVector,FVector)>);
    bool RouteIntersectsContainer(const TArray<FVector>&,const FVector&,const TArray<FSiteYardSlot>&,int32);
}

namespace
{
    FBox RecoverySweep(FVector From,FVector To,FVector Extent)
    {
        Extent=Extent.GetAbs();
        FBox Box(From-Extent,From+Extent);
        Box+=To-Extent;
        Box+=To+Extent;
        return Box;
    }

    FVector ContainerWorldExtent(const APortContainerActor* Cargo,const FRotator& Rotation)
    {
        const FVector Half=Cargo->ExternalDimensionsCm*.5f;
        const FRotationMatrix Basis(Rotation);
        return Basis.GetScaledAxis(EAxis::X).GetAbs()*Half.X+
            Basis.GetScaledAxis(EAxis::Y).GetAbs()*Half.Y+
            Basis.GetScaledAxis(EAxis::Z).GetAbs()*Half.Z;
    }

    bool RecoveryPathClear(UWorld* World,FVector From,FVector To,FVector Extent,const TSet<const AActor*>& Ignored)
    {
        const FBox Sweep=RecoverySweep(From,To,Extent);
        for(TActorIterator<AActor> It(World);It;++It)
        {
            if(!IsValid(*It) || Ignored.Contains(*It)) continue;
            TInlineComponentArray<UPrimitiveComponent*> Components(*It);
            for(const auto* Component:Components)
            {
                if(!Component || Component->GetCollisionEnabled()==ECollisionEnabled::NoCollision) continue;
                const FBox Bounds=Component->Bounds.GetBox();
                if(Bounds.IsValid && Sweep.Intersect(Bounds)) return false;
            }
        }
        return true;
    }
}

bool APortSiteLogistics::IsLoadedRecoveryVehicle(int32 V) const
{
    return LoadedAGVRecoveries.ContainsByPredicate([V](const auto& R){return R.Failed==V || R.Replacement==V;});
}

bool APortSiteLogistics::ValidateLoadedAGVRecoveries(FString& Error) const
{
    TSet<int32> Assigned; TSet<const APortContainerActor*> Boxes; TSet<const APortSupportVehicle*> Stackers;
    for(const auto& R:LoadedAGVRecoveries)
    {
        const bool ReplacementInRecovery=R.Phase<=6 || (R.bRepairCancelled && R.Phase<9);
        if(!Vehicles.IsValidIndex(R.Failed) || !Vehicles.IsValidIndex(R.Replacement) ||
            R.Failed==R.Replacement || Assigned.Contains(R.Failed) || Assigned.Contains(R.Replacement) ||
            !R.Stacker.IsValid() || Stackers.Contains(R.Stacker.Get()) || R.Cargo.IsEmpty() ||
            R.Cargo.Num()!=R.DeckTransforms.Num() ||
            ReplacementInRecovery!=(Jobs[R.Replacement].Stage==FSiteTransfer::ToRecovery))
        {Error=TEXT("Invalid/duplicate loaded AGV recovery assignment");return false;}
        Assigned.Add(R.Failed);Assigned.Add(R.Replacement);Stackers.Add(R.Stacker.Get());
        for(int32 I=0;I<R.Cargo.Num();++I)
        {
            const auto* C=R.Cargo[I].Get();
            const bool OnNew=I<R.Box, OnRS=I==R.Box && R.Phase>=2 && R.Phase<=5;
            const AActor* Expected=OnNew?static_cast<AActor*>(Vehicles[R.Replacement]):OnRS?
                static_cast<AActor*>(R.Stacker.Get()):static_cast<AActor*>(Vehicles[R.Failed]);
            if(!C || Boxes.Contains(C) || C->GetAttachParentActor()!=Expected || C->GetBody()->IsSimulatingPhysics() ||
                C->LocationOwner!=(OnRS?ECargoOwner::ReachStacker:ECargoOwner::AGV))
            {Error=TEXT("Loaded recovery lost single cargo ownership");return false;}
            if(!OnRS && !C->GetActorTransform().Equals(R.DeckTransforms[I]*Expected->GetActorTransform(),1.f))
            {Error=TEXT("Loaded recovery deck alignment mismatch");return false;}
            Boxes.Add(C);
        }
    }
    return true;
}

bool APortSiteLogistics::CanRecoverLoadedAGV(int32 V,FString& Error) const
{
    if(!bLoadedAGVRecoveryEnabled)
    { Error=TEXT("새 AGV 배정과 R/S 이동·인계는 현재 잠겨 있습니다."); return false; }
    if(!Jobs.IsValidIndex(V) || !Vehicles[V]->bDriveFault)
    { Error=TEXT("적재 AGV의 주행 고장이 필요합니다."); return false; }
    if(IsLoadedRecoveryVehicle(V)) { Error=TEXT("이미 R/S 복구 작업 중입니다."); return false; }
    const auto& J=Jobs[V];
    if(Vehicles[V]->Emergency.Active || Vehicles[V]->Speed>.1f || !J.Actor.IsValid() ||
        J.Actor->LocationOwner!=ECargoOwner::AGV || J.Actor->GetAttachParentActor()!=Vehicles[V])
    { Error=TEXT("AGV가 정지하고 화물을 소유한 상태에서 배정하세요."); return false; }
    for(const auto& C:Equipment) if(C->IsBusy() && C->GetHandoverVehicle()==Vehicles[V])
    { Error=TEXT("크레인 인계가 진행 중입니다. 인계 종료 후 배정하세요."); return false; }
    if(J.Stage!=3 && J.Stage!=9 && !(J.ExportIndex!=INDEX_NONE && (J.Stage==5 || J.Stage==6)))
    { Error=TEXT("화물 운송 도로 또는 야드 진입 대기 단계에서 배정하세요."); return false; }
    return true;
}

// Directed road network plus local recovery access. This is a blockout route,
// not an OEM reach-stacker dynamics/load-chart model.
bool APortSiteLogistics::PlanLoadedRecoveryRoad(FVector Start,FVector Goal,FVector Axis,int32 Depot,TArray<FVector>& Route) const
{
    Route.Reset();
    // Road graph edges are axis aligned. Measured headings contain small
    // steering residuals even after a straight-road stop.
    if(FMath::Max(FMath::Abs(Axis.X),FMath::Abs(Axis.Y))<.999f) return false;
    Axis=FMath::Abs(Axis.X)>FMath::Abs(Axis.Y)?FVector(FMath::Sign(Axis.X),0,0):FVector(0,FMath::Sign(Axis.Y),0);
    const bool Stacker=Depot==INDEX_NONE;
    FAGVRoadNetwork N;
    auto P=[](float X,float Y){return FVector(TerminalLayout::SiteCmX(X),Y,0);};
    const float South=TerminalLayout::AGVSouthReturnY,North=TerminalLayout::AGVNorthInboundY;
    for(float X:{12500.f,15300.f,TerminalLayout::AGVInlandInboundX}) N.Add(P(X,South),P(X,North));
    for(float X:{14000.f,16700.f,TerminalLayout::AGVInlandReturnX}) N.Add(P(X,North),P(X,South));
    for(float Y:{TerminalLayout::AGVSouthInboundY,TerminalLayout::AGVCentralInboundY,TerminalLayout::AGVNorthInboundY})
        N.Add(P(12500,Y),P(TerminalLayout::AGVInlandReturnX,Y));
    for(float Y:{South,TerminalLayout::AGVCentralReturnY,TerminalLayout::AGVNorthReturnY})
        N.Add(P(TerminalLayout::AGVInlandReturnX,Y),P(12500,Y));
    AGVApronRoads::Add(N);
    N.Add(P(12500,North),P(12500,TerminalLayout::AGVStandbyNorthY));
    N.Add(P(14000,TerminalLayout::AGVStandbyNorthY),P(14000,North));
    const FVector Park=Stacker?FVector::ZeroVector:FleetPark(Depot);
    const bool AtDepot=!Stacker && Start.Equals(Park,AGVReference::DockingAccuracyCm+1.f);
    if(!Stacker)
    {
        const float EntryY=TerminalLayout::AGVStandbyEntryY(Depot);
        const float ExitY=TerminalLayout::AGVStandbyExitY(Depot);
        N.Add(P(12500,EntryY),P(4800,EntryY));
        N.Add(FVector(Park.X,EntryY,0),Park,true);
        N.Add(Park,FVector(Park.X,ExitY,0));
        N.Add(P(4800,ExitY),P(14000,ExitY));
    }
    else
    {
        // Support equipment apron connects to the inland service road at its
        // north end; never cut across occupied yard stacks.
        const FVector Access(Start.X,North,0);
        N.Add(Start,Access,true); N.Add(Access,P(TerminalLayout::AGVInlandReturnX,North),true);
        if(Goal.X>P(TerminalLayout::AGVInlandReturnX,North).X)
        {
            const FVector HomeAccess(Goal.X,North,0);
            N.Add(P(TerminalLayout::AGVInlandReturnX,North),HomeAccess,true);
            N.Add(HomeAccess,Goal,true);
        }
    }
    const auto Roads=N.Roads;
    constexpr float LocalAccess=2200.f;
    if(!AtDepot && Vehicles.IsValidIndex(Depot))
    {
        const FVector Heading=Vehicles[Depot]->GetActorRightVector();
        const FVector Along=FMath::Abs(Heading.X)>FMath::Abs(Heading.Y)?FVector(1,0,0):FVector(0,1,0);
        for(const float Sign:{1.f,-1.f})
        {
            const FVector Departure=Start+Along*(Sign*LocalAccess);
            N.Add(Start,Departure);
            for(const auto& Road:Roads)
            {
                const FVector On=FMath::ClosestPointOnSegment(Departure,Road.A,Road.B);
                if(FVector::Dist2D(Departure,On)<=3500) N.Add(Departure,On);
            }
        }
    }
    auto ObstacleCost=[&](FVector A,FVector B)
    {
        for(const auto& V:Vehicles) if(V->bDriveFault && !V->GetActorLocation().Equals(Start,10) &&
            AGVTrafficGeometry::SweptChassisIntersects(A,B,V->GetActorLocation(),V->GetActorRightVector(),Stacker?450:730,Stacker?170:210,50,730,210))
            return TNumericLimits<double>::Max();
        return 0.;
    };
    auto TryRoute=[&](const TArray<FVector>& Raw)
    {
        auto Candidate=SiteLogistics::RoundRoadCorners(Start,Raw,[](FVector,FVector){return true;});
        if(Candidate.IsEmpty() || !AGVTrafficGeometry::RoundedRouteHasNoSharpTurns(Start,Candidate) ||
            (!Stacker && !AGVTrafficGeometry::RouteEnvelopeInside(Start,Candidate))) return false;
        const FVector FinalFrom=Candidate.Num()>1?Candidate[Candidate.Num()-2]:Start;
        const FVector FinalDirection=(Candidate.Last()-FinalFrom).GetSafeNormal2D();
        if(!FinalDirection.IsNearlyZero() && FMath::Abs(FVector::DotProduct(FinalDirection,Axis))<.999f) return false;
        FVector Previous=Start;
        for(const FVector& Point:Candidate)
        {
            if(ObstacleCost(Previous,Point)>0) return false;
            Previous=Point;
        }
        if(SiteLogistics::RouteIntersectsContainer(Candidate,Start,Yard,INDEX_NONE)) return false;
        Route=MoveTemp(Candidate);
        return true;
    };
    // Depot entries and support aprons already have a directed approach to their endpoint.
    TArray<FVector> Raw;
    if(Roads.ContainsByPredicate([&](const auto& Road){return FAGVRoadNetwork::Contains(Road,Goal);}) &&
        N.Find(Start,Goal,Raw,ObstacleCost) && TryRoute(Raw)) return true;
    for(const float Sign:{1.f,-1.f})
    {
        const FVector Approach=Goal-Axis*(Sign*LocalAccess);
        TArray<FVector> AccessPoints;
        for(const auto& Road:Roads)
        {
            const FVector On=FMath::ClosestPointOnSegment(Approach,Road.A,Road.B);
            const double Spur=FVector::Dist2D(On,Approach);
            if(Spur>3500 || (Spur>.1 && Spur<AGVReference::MinimumInnerTurnRadiusCm+100)) continue;
            if(!FMath::IsNearlyEqual(On.X,Approach.X,.1) && !FMath::IsNearlyEqual(On.Y,Approach.Y,.1)) continue;
            if(!AccessPoints.ContainsByPredicate([&](const FVector& Point){return Point.Equals(On,.1);}))
                AccessPoints.Add(On);
        }
        AccessPoints.Sort([&](const FVector& A,const FVector& B)
        {return FVector::Dist2D(Start,A)+FVector::Dist2D(A,Approach)<FVector::Dist2D(Start,B)+FVector::Dist2D(B,Approach);});
        for(const FVector& On:AccessPoints)
        {
            if(!N.Find(Start,On,Raw,ObstacleCost)) continue;
            if(!On.Equals(Approach,.1)) Raw.Add(Approach);
            Raw.Add(Goal);
            if(TryRoute(Raw)) return true;
        }
    }
    return false;
}

bool APortSiteLogistics::RequestLoadedAGVRecovery(int32 Failed,FString& Error)
{
    if(!CanRecoverLoadedAGV(Failed,Error)) return false;
    auto* Old=Vehicles[Failed].Get();
    FLoadedAGVRecovery R; R.Failed=Failed;
    double Mass=0;
    for(const auto& C:ShipContainers) if(C->GetAttachParentActor()==Old && C->LocationOwner==ECargoOwner::AGV)
    {
        if(!FMath::IsFinite(C->MassKg) || C->MassKg<=0 || C->MassKg>45000 ||
            !FMath::IsFinite(C->CoGOffsetCm.X) || !FMath::IsFinite(C->CoGOffsetCm.Y) ||
            !FMath::IsFinite(C->CoGOffsetCm.Z) || !C->ExternalDimensionsCm.IsNearlyZero() ||
            FMath::Abs(C->CoGOffsetCm.X)>C->ExternalDimensionsCm.X*.5f ||
            FMath::Abs(C->CoGOffsetCm.Y)>C->ExternalDimensionsCm.Y*.5f ||
            FMath::Abs(C->CoGOffsetCm.Z)>C->ExternalDimensionsCm.Z*.5f ||
            C->GetBody()->IsSimulatingPhysics())
        {Error=TEXT("화물 중량·무게중심·고정 상태가 R/S 복구 조건을 벗어났습니다.");return false;}
        R.Cargo.Add(C); R.DeckTransforms.Add(C->GetActorTransform().GetRelativeTransform(Old->GetActorTransform())); Mass+=C->MassKg;
    }
    if(R.Cargo.IsEmpty() || Mass>AGVReference::MaximumPayloadKg)
    {Error=TEXT("인계할 화물 또는 대체 AGV 하중을 확인하세요.");return false;}
    // Parallel receiving bay, with the stacker between the two stopped decks.
    const FVector Axis=Old->GetActorRightVector();
    bool AvailableAGV=false,AvailableRS=false,AGVRouteFound=false;
    for(int32 V=0;V<Jobs.Num();++V)
        AvailableAGV|=V!=Failed && !Jobs[V].Stage && Vehicles[V]->CanAcceptWork() && !IsLoadedRecoveryVehicle(V);
    for(TActorIterator<APortSupportVehicle> It(GetWorld());It;++It)
        AvailableRS|=It->EquipmentType==EPortSupportType::ReachStacker &&
            !LoadedAGVRecoveries.ContainsByPredicate([&](const auto& Other){return Other.Stacker.Get()==*It;});
    for(int32 Side:{1,-1})
    {
        R.Target=Old->GetActorLocation()+Old->GetActorForwardVector()*float(Side*900);
        R.StackerTarget=(Old->GetActorLocation()+R.Target)*.5f;
        const FVector Extent=Old->GetActorForwardVector().GetAbs()*210+Axis.GetAbs()*730;
        if(!TerminalLayout::AGVEnvelopeInside(R.Target.X-Extent.X,R.Target.X+Extent.X,R.Target.Y-Extent.Y,R.Target.Y+Extent.Y)) continue;
        for(int32 V=0;V<Jobs.Num();++V)
        {
            if(V==Failed || Jobs[V].Stage || !Vehicles[V]->CanAcceptWork() || IsLoadedRecoveryVehicle(V)) continue;
            auto* New=Vehicles[V].Get();
            const FTransform DestinationDeck(New->GetActorRotation(),R.Target,New->GetActorScale3D());
            FBox WorkZone(ForceInit);
            WorkZone+=Old->GetActorLocation();
            WorkZone+=R.Target;
            WorkZone+=R.StackerTarget;
            TSet<const AActor*> Participants;
            Participants.Add(Old);
            Participants.Add(New);
            for(const auto& Cargo:R.Cargo) Participants.Add(Cargo.Get());
            bool GeometryClear=true;
            for(int32 Box=0;Box<R.Cargo.Num();++Box)
            {
                auto* Cargo=R.Cargo[Box].Get();
                const FVector Destination=(R.DeckTransforms[Box]*DestinationDeck).GetLocation();
                const float Height=FMath::Max(Old->CargoPosition().Z,New->CargoPosition().Z)+250.f;
                const FVector LiftOld(Cargo->GetActorLocation().X,Cargo->GetActorLocation().Y,Height);
                const FVector LiftNew(Destination.X,Destination.Y,Height);
                const FVector CargoExtent=ContainerWorldExtent(Cargo,Cargo->GetActorRotation())+FVector(100,100,100);
                WorkZone+=Cargo->GetActorLocation();
                WorkZone+=LiftOld;
                WorkZone+=LiftNew;
                WorkZone+=Destination;
                GeometryClear&=FVector::Dist2D(Cargo->GetActorLocation(),R.StackerTarget)<=850.f &&
                    FVector::Dist2D(Destination,R.StackerTarget)<=850.f &&
                    RecoveryPathClear(GetWorld(),Cargo->GetActorLocation(),LiftOld,CargoExtent,Participants) &&
                    RecoveryPathClear(GetWorld(),LiftOld,LiftNew,CargoExtent,Participants) &&
                    RecoveryPathClear(GetWorld(),LiftNew,Destination,CargoExtent,Participants);
            }
            if(!GeometryClear) continue;
            WorkZone=WorkZone.ExpandBy(FVector(1000,1000,1000));
            bool ZoneClear=true;
            for(const auto& Vehicle:TrafficVehicles)
            {
                if(!IsValid(Vehicle) || Vehicle==Old || Vehicle==New) continue;
                FVector OtherOrigin,OtherExtent;
                Vehicle->GetActorBounds(true,OtherOrigin,OtherExtent);
                if(WorkZone.Intersect(FBox(OtherOrigin-OtherExtent,OtherOrigin+OtherExtent)))
                {ZoneClear=false;break;}
            }
            if(!ZoneClear) continue;
            for(const auto& Reservation:RoadReservations)
            {
                if(Reservation.Key==Old->VehicleID || Reservation.Key==New->VehicleID) continue;
                for(const FBox& Segment:Reservation.Value) if(WorkZone.Intersect(Segment))
                {ZoneClear=false;break;}
                if(!ZoneClear) break;
            }
            if(!ZoneClear || LoadedAGVRecoveries.ContainsByPredicate(
                [&](const auto& Other){return Other.WorkZone.Intersect(WorkZone);})) continue;
            TArray<FVector> Route;
            if(!PlanLoadedRecoveryRoad(New->GetActorLocation(),R.Target,Axis,V,Route)) continue;
            AGVRouteFound=true;
            bool RouteClear=true;
            FVector Previous=New->GetActorLocation();
            for(const FVector& Point:Route)
            {
                if(!RecoveryPathClear(GetWorld(),Previous,Point,Extent,Participants))
                {RouteClear=false;break;}
                Previous=Point;
            }
            if(!RouteClear) continue;
            for(TActorIterator<APortSupportVehicle> It(GetWorld());It;++It)
            {
                if(It->EquipmentType!=EPortSupportType::ReachStacker ||
                    LoadedAGVRecoveries.ContainsByPredicate([&](const auto& Other){return Other.Stacker.Get()==*It;})) continue;
                auto* Stacker=*It;
                if(!PlanLoadedRecoveryRoad(Stacker->GetActorLocation(),R.StackerTarget,Axis,INDEX_NONE,R.StackerRoute)) continue;
                Participants.Add(Stacker);
                bool StackerRouteClear=true;
                Previous=Stacker->GetActorLocation();
                for(const FVector& Point:R.StackerRoute)
                {
                    if(!RecoveryPathClear(GetWorld(),Previous,Point,FVector(450,170,350),Participants))
                    {StackerRouteClear=false;break;}
                    Previous=Point;
                }
                if(!StackerRouteClear) {Participants.Remove(Stacker);continue;}
                R.Replacement=V; R.Stacker=Stacker; R.StackerHome=Stacker->GetActorTransform();
                R.ReplacementHome=New->GetActorTransform();
                R.WorkZone=WorkZone;
                if(!PlanLoadedRecoveryRoad(R.StackerTarget,R.StackerHome.GetLocation(),Axis,INDEX_NONE,R.StackerExitRoute))
                {Participants.Remove(Stacker);R.StackerRoute.Reset();continue;}
                R.Status=TEXT("작업 구역 확보 · 대체 AGV와 R/S 출동");
                auto& J=Jobs[V]; J=FSiteTransfer(); J.Stage=FSiteTransfer::ToRecovery; J.Route=MoveTemp(Route);
                J.RouteStart=New->GetActorLocation(); J.bRouteStartValid=true;
                LoadedAGVRecoveries.Add(R);
                RecordRecoveryEvent(TEXT("loaded_agv_dispatched"),Old,Old,New,Jobs[Failed].Actor.Get());
                return true;
            }
        }
    }
    Error=!AvailableAGV?TEXT("배정 가능한 빈 정상 AGV가 없습니다."):
        !AvailableRS?TEXT("배정 가능한 R/S가 없습니다. 다른 복구 완료 후 다시 배정하세요."):
        !AGVRouteFound?TEXT("대체 AGV의 현장 접근 경로를 확보하지 못했습니다."):
        TEXT("R/S의 현장 접근 경로를 확보하지 못했습니다.");
    return false;
}

void APortSiteLogistics::AdvanceLoadedAGVRecoveries(float Dt)
{
    if(!bLoadedAGVRecoveryEnabled) return;
    for(int32 I=LoadedAGVRecoveries.Num()-1;I>=0;--I)
    {
        auto& R=LoadedAGVRecoveries[I]; auto* Old=Vehicles[R.Failed].Get(); auto* New=Vehicles[R.Replacement].Get();
        auto* RS=R.Stacker.Get(); auto& Arrival=Jobs[R.Replacement];
        Old->SetFMSHold(true);
        if(!RS)
        {New->SetFMSHold(true);R.Status=TEXT("복구 장비 참조 오류 · 화물 유지 · 수동 확인 대기");continue;}
        if(!R.bRepairCancelled && !Old->bDriveFault && R.Phase<2)
        {
            TArray<FVector> ReturnRoute;
            if(!PlanLoadedRecoveryRoad(New->GetActorLocation(),R.ReplacementHome.GetLocation(),
                New->GetActorRightVector(),R.Replacement,ReturnRoute))
            {R.Status=TEXT("인양 전 수리 완료 · 대체 AGV 퇴출 경로 확보 대기");continue;}
            R.ReplacementExitRoute=MoveTemp(ReturnRoute);
            R.ReplacementExitWaypoint=0;
            New->SetFMSHold(true);
            auto& ReturnJob=Jobs[R.Replacement];
            ReturnJob=FSiteTransfer();
            ReturnJob.Stage=FSiteTransfer::ToRecovery;
            ReturnJob.Route=R.ReplacementExitRoute;
            ReturnJob.RouteStart=New->GetActorLocation();
            ReturnJob.bRouteStartValid=true;
            for(int32 ID:{Old->VehicleID})
            {
                RoadReservations.Remove(ID); RoadReservationEndWaypoint.Remove(ID); ReservationIndex.Remove(ID);
                RoadTargets.Remove(ID); RoadBlockers.Remove(ID); RoadWaitSince.Remove(ID);
                FinishedRoadSegments.Remove(ID); ClearPendingRoadRequest(ID);
            }
            R.bRepairCancelled=true;
            R.Phase=7;
            R.HoldSeconds=0;
            RS->StowRecoverySpreader();
            R.Status=TEXT("인양 전 수리 완료 · 붐 정리 후 안전 퇴출");
            RecordRecoveryEvent(TEXT("loaded_agv_recovery_cancelled"),Old,RS,Old,Jobs[R.Failed].Actor.Get());
        }
        if(New->bDriveFault || New->Emergency.Active || Old->Emergency.Active)
        {New->SetFMSHold(true);R.Status=TEXT("복구 장비 고장 · 화물 지지 유지 · 안전 정지");continue;}
        if(R.Phase==0)
        {
            R.Status=TEXT("작업 구역 확보 · 대체 AGV와 R/S 출동");
            if(Arrival.Route.IsValidIndex(Arrival.Waypoint))
            {
                New->SetFMSHold(false);
                if(MoveVehicle(New,Arrival.Route[Arrival.Waypoint],Dt)) ++Arrival.Waypoint;
            }
            if(R.StackerRoute.IsValidIndex(R.Waypoint))
            {
                const FVector P=RS->GetActorLocation(),Goal=R.StackerRoute[R.Waypoint];
                const FVector Next=FMath::VInterpConstantTo(P,Goal,Dt,200.f);
                bool Clear=true;
                for(const auto& V:TrafficVehicles) if(IsValid(V) &&
                    AGVTrafficGeometry::SweptChassisIntersects(P,Next,V->GetActorLocation(),V->GetActorRightVector(),450,170,50,730,210)) {Clear=false;break;}
                for(const auto& Recovery:LoadedAGVRecoveries)
                    if(&Recovery!=&R && Recovery.WorkZone.Intersect(RecoverySweep(P,Next,FVector(450,170,350)))) Clear=false;
                if(Clear)
                {
                    RS->SetActorLocation(Next);
                    if(!Next.Equals(P,.01)) RS->SetActorRotation((Next-P).Rotation());
                    if(Next.Equals(Goal,5)) ++R.Waypoint;
                }
                else R.Status=TEXT("R/S 진입 공간 확보 대기");
            }
            if(Arrival.Waypoint<Arrival.Route.Num() || R.Waypoint<R.StackerRoute.Num() || New->Speed>.1f) continue;
            New->SetFMSHold(true); R.Phase=1; R.HoldSeconds=0;
        }
        if(R.Phase>=7)
        {
            if(R.Phase==7)
            {
                New->SetFMSHold(true);
                RS->StowRecoverySpreader();
                R.HoldSeconds+=Dt;
                R.Status=R.bRepairCancelled?TEXT("인양 전 수리 완료 · 붐 정리"):TEXT("화물 인계 완료 · 붐 주행 자세 복귀");
                if(R.HoldSeconds<1) continue;
                R.StackerExitRoute.Reset();
                R.ExitWaypoint=0;
                if(!RS->GetActorLocation().Equals(R.StackerHome.GetLocation(),5) &&
                    !PlanLoadedRecoveryRoad(RS->GetActorLocation(),R.StackerHome.GetLocation(),
                        Old->GetActorRightVector(),INDEX_NONE,R.StackerExitRoute))
                {R.Status=TEXT("붐 정리 완료 · R/S 퇴출 경로 확보 대기");continue;}
                R.Phase=8;
            }
            if(R.Phase==8)
            {
                if(R.bRepairCancelled)
                {
                    R.Status=TEXT("인양 전 수리 완료 · 대체 AGV 원위치 복귀");
                    if(R.ReplacementExitRoute.IsValidIndex(R.ReplacementExitWaypoint) &&
                        MoveVehicle(New,R.ReplacementExitRoute[R.ReplacementExitWaypoint],Dt))
                        Arrival.Waypoint=++R.ReplacementExitWaypoint;
                    if(R.ReplacementExitWaypoint<R.ReplacementExitRoute.Num() || New->Speed>.1f ||
                        !New->GetActorLocation().Equals(R.ReplacementHome.GetLocation(),100)) continue;
                    New->SetFMSHold(true);
                    Arrival=FSiteTransfer();
                    RoadReservations.Remove(New->VehicleID); RoadReservationEndWaypoint.Remove(New->VehicleID);
                    ReservationIndex.Remove(New->VehicleID); RoadTargets.Remove(New->VehicleID);
                }
                else
                {
                    R.Status=TEXT("운송 승계 · 새 AGV 안전 퇴출");
                    FVector NewOrigin,NewExtent;
                    New->GetActorBounds(true,NewOrigin,NewExtent);
                    if(R.WorkZone.Intersect(FBox(NewOrigin-NewExtent,NewOrigin+NewExtent)))
                    {
                        if(Arrival.Route.IsValidIndex(Arrival.Waypoint) &&
                            MoveVehicle(New,Arrival.Route[Arrival.Waypoint],Dt)) ++Arrival.Waypoint;
                        continue;
                    }
                    New->SetFMSHold(true);
                    RoadReservations.Remove(New->VehicleID); RoadReservationEndWaypoint.Remove(New->VehicleID);
                    ReservationIndex.Remove(New->VehicleID); RoadTargets.Remove(New->VehicleID);
                }
                R.Phase=9;
                continue;
            }
            if(R.Phase==9)
            {
                R.Status=R.bRepairCancelled?TEXT("R/S 안전 퇴출 · 원래 AGV 작업 복귀 대기"):TEXT("R/S 안전 퇴출 · 작업 구역 해제 대기");
                if(!RS->GetActorLocation().Equals(R.StackerHome.GetLocation(),5))
                {
                    if(!R.StackerExitRoute.IsValidIndex(R.ExitWaypoint)) continue;
                    const FVector From=RS->GetActorLocation(),Goal=R.StackerExitRoute[R.ExitWaypoint];
                    const FVector Next=FMath::VInterpConstantTo(From,Goal,Dt,200.f);
                    TSet<const AActor*> Participants;
                    Participants.Add(Old); Participants.Add(RS);
                    for(const auto& Cargo:R.Cargo) Participants.Add(Cargo.Get());
                    bool Clear=RecoveryPathClear(GetWorld(),From,Next,FVector(450,170,350),Participants);
                    const FBox Sweep=RecoverySweep(From,Next,FVector(450,170,350));
                    for(const auto& Reservation:RoadReservations)
                    {
                        if(Reservation.Key==Old->VehicleID) continue;
                        for(const FBox& Segment:Reservation.Value) if(Sweep.Intersect(Segment)) Clear=false;
                    }
                    if(!Clear) continue;
                    RS->SetActorLocation(Next);
                    if(!Next.Equals(From,.01)) RS->SetActorRotation((Next-From).Rotation());
                    if(Next.Equals(Goal,5)) ++R.ExitWaypoint;
                    if(!RS->GetActorLocation().Equals(R.StackerHome.GetLocation(),5)) continue;
                }
                RS->SetActorTransform(R.StackerHome);
                if(R.bRepairCancelled)
                {
                    Old->SetFMSHold(false);
                    New->SetFMSHold(false);
                }
                else
                    RecordRecoveryEvent(TEXT("loaded_agv_recovery_complete"),Old,RS,RS,Arrival.Actor.Get());
                LoadedAGVRecoveries.RemoveAt(I);
            }
            continue;
        }
        New->SetFMSHold(true);
        if(Old->Speed>.1f || New->Speed>.1f || !New->GetActorLocation().Equals(R.Target,6))
        {R.Status=TEXT("정지·안착 위치 확인 대기");continue;}
        const int32 BoxIndex=FMath::Min(R.Box,R.Cargo.Num()-1);
        auto* C=R.Cargo[BoxIndex].Get();
        if(!C) {R.Status=TEXT("화물 참조 오류 · 복구 정지");continue;}
        const FTransform Destination=R.DeckTransforms[BoxIndex]*New->GetActorTransform();
        R.HoldSeconds+=Dt;
        if(R.Phase==1)
        {
            R.Status=TEXT("R/S 스프레더 정렬 · 잠금 확인");
            RS->PositionRecoverySpreader(C->GetActorLocation()+FVector(0,0,C->ExternalDimensionsCm.Z*.5f),C->GetActorRotation(),C->ExternalDimensionsCm.Y);
            if(R.HoldSeconds<3) continue;
            if(C->LocationOwner!=ECargoOwner::AGV || C->GetAttachParentActor()!=Old || C->GetBody()->IsSimulatingPhysics())
            {R.Status=TEXT("화물 소유권 확인 실패");continue;}
            C->AttachToActor(RS,FAttachmentTransformRules::KeepWorldTransform);
            C->SetLocationOwner(ECargoOwner::ReachStacker);
            Old->SetPayload(FMath::Max(0.f,Old->PayloadKg-C->MassKg));
            R.Phase=2; R.HoldSeconds=0;
            RecordRecoveryEvent(TEXT("loaded_agv_lift"),Old,Old,RS,C);
        }
        if(R.Phase>=2 && R.Phase<=4)
        {
            R.Status=R.Phase==2?TEXT("R/S 인양"):R.Phase==3?TEXT("R/S 대체 AGV로 이송"):TEXT("대체 AGV 안착");
            FVector Goal=Destination.GetLocation();
            const float Height=FMath::Max(Old->CargoPosition().Z,New->CargoPosition().Z)+250.f;
            if(R.Phase==2) {Goal=C->GetActorLocation();Goal.Z=Height;}
            if(R.Phase==3) Goal.Z=Height;
            const FVector Current=C->GetActorLocation();
            const FVector Next=FMath::VInterpConstantTo(Current,Goal,Dt,R.Phase==3?60.f:40.f);
            TSet<const AActor*> Participants;
            Participants.Add(Old); Participants.Add(New); Participants.Add(RS);
            for(const auto& Cargo:R.Cargo) Participants.Add(Cargo.Get());
            const FVector CargoExtent=ContainerWorldExtent(C,C->GetActorRotation())+FVector(100,100,100);
            if(!RecoveryPathClear(GetWorld(),Current,Next,CargoExtent,Participants))
            {R.Status=TEXT("이송 공간 침입 · 화물 지지 유지 · 안전 정지");continue;}
            C->SetActorLocation(Next);
            C->SetActorRotation(FMath::QInterpConstantTo(C->GetActorQuat(),Destination.GetRotation(),Dt,.2f));
            RS->PositionRecoverySpreader(C->GetActorLocation()+FVector(0,0,C->ExternalDimensionsCm.Z*.5f),C->GetActorRotation(),C->ExternalDimensionsCm.Y);
            if(!C->GetActorLocation().Equals(Goal,.1f)) continue;
            ++R.Phase; R.HoldSeconds=0;
        }
        if(R.Phase==5)
        {
            R.Status=TEXT("안착 확인 · 잠금 해제");
            if(R.HoldSeconds<2 || !C->GetActorTransform().Equals(Destination,.1f)) continue;
            C->AttachToActor(New,FAttachmentTransformRules::KeepWorldTransform); C->SetLocationOwner(ECargoOwner::AGV);
            New->SetPayload(New->PayloadKg+C->MassKg);
            RecordRecoveryEvent(TEXT("loaded_agv_seated"),Old,RS,New,C);
            if(++R.Box<R.Cargo.Num()) {R.Phase=1;R.HoldSeconds=0;continue;}
            R.Phase=6;
        }
        if(R.Phase==6)
        {
            // Preserve the exact remaining job and destination. A local access
            // route rejoins the original route; bookings change owners once.
            const auto Saved=Jobs[R.Failed];
            TArray<FVector> Join;
            int32 Next=Saved.Waypoint;
            if(!Saved.Route.IsValidIndex(Next)) Next=Saved.Route.Num()-1;
            if(Next<0 || !PlanLoadedRecoveryRoad(New->GetActorLocation(),Saved.Route[Next],Old->GetActorRightVector(),R.Replacement,Join))
            {R.Status=TEXT("화물 인계 완료 · 원래 운송 경로 재진입 대기");continue;}
            const int32 Added=Join.Num();
            for(int32 N=Next+1;N<Saved.Route.Num();++N) Join.Add(Saved.Route[N]);
            if(!AGVTrafficGeometry::RoundedRouteHasNoSharpTurns(New->GetActorLocation(),Join))
            {R.Status=TEXT("화물 인계 완료 · 원래 경로와의 회전 여유 확보 대기");continue;}
            Arrival=Saved;
            Arrival.Route=MoveTemp(Join); Arrival.Waypoint=0; Arrival.RouteStart=New->GetActorLocation(); Arrival.bRouteStartValid=true;
            Arrival.YardEntryWaypoint=Saved.YardEntryWaypoint>=Next?Added-1+Saved.YardEntryWaypoint-Next:Saved.YardEntryWaypoint;
            if(Arrival.Stage==9) {Arrival.Stage=3;Arrival.YardEntryWaypoint=Added-1;}
            Arrival.bEquipmentWaiting=false; Arrival.StationaryTime=0;
            New->ActiveDeckOffsetCm=Old->ActiveDeckOffsetCm;
            for(auto& LeaseOwner:STSOwners) if(LeaseOwner==R.Failed) LeaseOwner=R.Replacement;
            for(auto& LeaseOwner:NextVehicles) if(LeaseOwner==R.Failed) LeaseOwner=R.Replacement;
            for(auto& LeaseOwner:YardApproachOwners) if(LeaseOwner==R.Failed) LeaseOwner=R.Replacement;
            for(auto& Lease:DeferredYardLeases) if(Lease.Vehicle==R.Failed) Lease.Vehicle=R.Replacement;
            for(const auto& Crane:Equipment) if(Crane->GetHandoverVehicle()==Old) Crane->SetHandoverVehicle(New);
            Jobs[R.Failed]=FSiteTransfer(); Old->ActiveDeckOffsetCm=0;
            for(int32 ID:{Old->VehicleID,New->VehicleID})
            {
                RoadReservations.Remove(ID); RoadReservationEndWaypoint.Remove(ID); ReservationIndex.Remove(ID);
                RoadTargets.Remove(ID); RoadBlockers.Remove(ID); RoadWaitSince.Remove(ID); FinishedRoadSegments.Remove(ID); ClearPendingRoadRequest(ID);
            }
            RecordRecoveryEvent(TEXT("loaded_agv_resumed"),Old,Old,New,Arrival.Actor.Get());
            New->SetFMSHold(false);
            R.Phase=7;
            R.HoldSeconds=0;
        }
    }
}

void APortSiteLogistics::ResetLoadedAGVRecoveries()
{
    for(auto& R:LoadedAGVRecoveries) if(R.Stacker.IsValid())
    { R.Stacker->StowRecoverySpreader(); R.Stacker->SetActorTransform(R.StackerHome); }
    LoadedAGVRecoveries.Reset();
}
