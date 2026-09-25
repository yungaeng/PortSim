#include "QuayCrane.h"
#include "PortWorkingCrane.h"
#include "PortSiteLogistics.h"
#include "Components/StaticMeshComponent.h"
#include "TerminalLayout.h"
#include "Engine/World.h"
#include "GameFramework/SpringArmComponent.h"

namespace Fleet
{
    constexpr float DeckTop = 220.f;
    constexpr float CargoOffset = DeckTop + 129.5f;
    constexpr float LiftOffset = 154.5f;
    constexpr float SafeZ = 1900.f;
    constexpr float QuayX = 3000.f;
    constexpr float ParkX = 4500.f;
}

void AQuayCrane::BuildFleet()
{
    auto Box = [this](const FString& Name, USceneComponent* Parent, FVector P, FVector Size, bool Collision)
    {
        auto* C = NewObject<UStaticMeshComponent>(this, FName(*Name));
        AddInstanceComponent(C); C->SetupAttachment(Parent);
        C->SetStaticMesh(TrolleyMesh->GetStaticMesh()); C->SetMobility(EComponentMobility::Movable);
        C->SetRelativeLocation(P); C->SetRelativeScale3D(Size/100.f);
        C->SetCollisionProfileName(Collision ? TEXT("BlockAllDynamic") : TEXT("NoCollision"));
        C->RegisterComponent(); return C;
    };
    FActorSpawnParameters Params;
    Params.Owner=this;
    Params.SpawnCollisionHandlingOverride=ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
    for (int32 I=0; I<3; ++I)
    {
        auto* Vehicle=GetWorld()->SpawnActor<APortAGVActor>(FVector(Fleet::ParkX,-2400+2400*I,0),FRotator::ZeroRotator,Params);
        check(Vehicle);
        Vehicle->InitializeVehicle(I+1);
        AGVActors.Add(Vehicle);
        const float Y=-2400.f+I*2400.f;
        Box(FString::Printf(TEXT("Road_AGV_%d"),I),RootComponent,FVector(8750,Y,22),FVector(11500,1500,3),false);
        for (int32 Dash=0;Dash<24;++Dash)
            Box(FString::Printf(TEXT("RoadMark_%d_%d"),I,Dash),RootComponent,FVector(2800+Dash*470,Y+760,25),FVector(220,20,3),false);
    }
    CameraArm->TargetArmLength=185000.f;
    CameraArm->SetRelativeLocation(FVector(35000,0,0));
    CameraArm->SetRelativeRotation(FRotator(-52,38,0));
    ResetFleet();
}

FVector AQuayCrane::AGVCargoPosition() const
{
    return AGVActors[ActiveAGV]->CargoPosition();
}

void AQuayCrane::ResetFleet()
{
    ActiveAGV=0; FleetStep=0; RMGStep=0;
    bAGVHasCargo=false; bRMGHasCargo=false; FleetSettle=0;
    for (int32 I=0;I<AGVActors.Num();++I)
        AGVActors[I]->ResetVehicle(FVector(Fleet::ParkX,-2400+2400*I,0));
    RMGActor=nullptr; FleetRoute.Reset(); bFleetRouteActive=false; FleetWaypoint=0;
}

bool AQuayCrane::MoveAGV(EFleetDestination Destination,float Dt)
{
    auto* Vehicle=AGVActors[ActiveAGV].Get();
    const bool ToYard=Destination==EFleetDestination::Yard;
    const float X=Destination==EFleetDestination::Park?Fleet::ParkX:Fleet::QuayX;
    if (!bFleetRouteActive)
    {
        FleetRoute.Reset(); FleetWaypoint=0;
        const FVector P=Vehicle->GetActorLocation();
        const FVector Yard=SiteLogistics->CentralHandover(ActiveCargoIndex);
        const FVector Quay(X,-2400.f+ActiveAGV*2400.f,0);
        if (ToYard)
        {
            const float RoadX=Yard.Y+730>=P.Y?12500.f:16500.f;
            FleetRoute.Add(FVector(RoadX,P.Y,0));
            FleetRoute.Add(FVector(RoadX,Yard.Y+730,0));
            FleetRoute.Add(FVector(Yard.X,Yard.Y+730,0));
            FleetRoute.Add(Yard);
        }
        else if (P.X>10000.f)
        {
            FleetRoute.Add(FVector(Yard.X,Yard.Y+1180,0));
            const float RoadX=Quay.Y+1200>=Yard.Y+1180?12500.f:16500.f;
            FleetRoute.Add(FVector(RoadX,Yard.Y+1180,0));
            FleetRoute.Add(FVector(RoadX,Quay.Y+1200,0));
            FleetRoute.Add(FVector(Quay.X,Quay.Y+1200,0));
            FleetRoute.Add(Quay);
        }
        else FleetRoute.Add(Quay);
        bFleetRouteActive=true;
    }
    const bool Arrived=SiteLogistics->MoveVehicle(Vehicle,FleetRoute[FleetWaypoint],Dt);
    if (Arrived)
    {
        if (FleetRoute.Num()>1 && FleetWaypoint==0 && ToYard) Vehicle->SetActorRotation(FRotator(0,90,0));
        if (FleetRoute.Num()>1 && FleetWaypoint==FleetRoute.Num()-1 && !ToYard) Vehicle->SetActorRotation(FRotator::ZeroRotator);
        ++FleetWaypoint;
    }
    if (bAGVHasCargo) Cargo->SetWorldLocationAndRotation(AGVCargoPosition(),Vehicle->GetActorRotation());
    if (FleetWaypoint<FleetRoute.Num()) return false;
    bFleetRouteActive=false;
    return true;
}

void AQuayCrane::HoldFleetCargo(bool OnAGV)
{
    Cargo->SetPhysicsLinearVelocity(FVector::ZeroVector);
    Cargo->SetPhysicsAngularVelocityInDegrees(FVector::ZeroVector);
    Cargo->SetSimulatePhysics(false);
    bAGVHasCargo=OnAGV; bRMGHasCargo=!OnAGV;
    ContainerActors[ActiveCargoIndex]->LocationOwner=OnAGV?ECargoOwner::AGV:ECargoOwner::RMG;
}

void AQuayCrane::ReleaseFleetCargo()
{
    bAGVHasCargo=false; bRMGHasCargo=false;
    Cargo->SetSimulatePhysics(true);
    Cargo->SetPhysicsLinearVelocity(FVector::ZeroVector);
    Cargo->SetPhysicsAngularVelocityInDegrees(FVector::ZeroVector);
    Cargo->WakeAllRigidBodies();
}

bool AQuayCrane::TickRMGTransfer(FVector Source,FVector Destination,float Dt)
{
    auto* Container=ContainerActors[ActiveCargoIndex].Get();
    if (RMGStep==0)
    {
        if (bLocked || AGVActors[ActiveAGV]->Speed>0 || FVector::Dist(Cargo->GetComponentLocation(),Source)>20.f)
        { StopAutomatic(TEXT("Yard RMG pickup alignment / AGV stop.")); return false; }
        if (!RMGActor->AssignCargo(Container,Source,Destination,bAutoLoading,!bAutoLoading))
        { StopAutomatic(TEXT("Reserved yard RMG could not accept cargo.")); return false; }
        bAGVHasCargo=false; RMGStep=1;
    }
    RMGActor->Advance(Dt,false);
    bRMGHasCargo=RMGActor->bCarrying;
    if (!RMGActor->Fault.IsEmpty()) { StopAutomatic(RMGActor->Fault); return false; }
    if (RMGActor->IsBusy()) return false;
    bRMGHasCargo=false;
    if (bAutoLoading) HoldFleetCargo(true);
    else
    {
        Cargo->SetSimulatePhysics(false);
        Container->LocationOwner=ECargoOwner::Yard;
    }
    ++RMGStep;
    return true;
}

void AQuayCrane::TickFleet(float Dt)
{
    // Use the site block and road reservations for both berth and site traffic.
    if (!SiteLogistics->ReserveCentral(ActiveCargoIndex)) return;
    if (AutoStage==ETerminalStage::FleetPrepare)
    {
        if (FleetStep==0)
        {
            if (!MoveAGV(bAutoLoading?EFleetDestination::Yard:EFleetDestination::Quay,Dt)) return;
            if (!bAutoLoading) { SetAutoStage(ETerminalStage::RaiseEmpty); return; }
            FleetStep=1;
        }
        else if (FleetStep==1)
        {
            if (TickRMGTransfer(TerminalSlot(ActiveCargoIndex,false),AGVCargoPosition(),Dt)) FleetStep=2;
        }
        else if (FleetStep==2)
        {
            if (MoveAGV(EFleetDestination::Quay,Dt)) { ReleaseFleetCargo(); FleetStep=3; FleetSettle=0; }
        }
        else if (FleetStep==3)
        {
            FleetSettle+=Dt;
            if (FleetSettle>1.f && Cargo->GetPhysicsLinearVelocity().Size()<8.f)
            {
                if (FVector::Dist(Cargo->GetComponentLocation(),AGVCargoPosition())>20.f)
                { StopAutomatic(TEXT("AGV load shifted before STS pickup.")); return; }
                SetAutoStage(ETerminalStage::RaiseEmpty);
            }
        }
    }
    else if (AutoStage==ETerminalStage::FleetDeliver)
    {
        if (bAutoLoading)
        {
            if (MoveAGV(EFleetDestination::Park,Dt)) CompleteAutomaticJob();
            return;
        }
        if (FleetStep==0)
        {
            if (MoveAGV(EFleetDestination::Yard,Dt)) { FleetStep=1; RMGStep=0; }
        }
        else if (FleetStep==1)
        {
            if (TickRMGTransfer(AGVCargoPosition(),TerminalSlot(ActiveCargoIndex,false),Dt)) { FleetStep=2; FleetSettle=0; }
        }
        else if (FleetStep==2)
        {
            const FVector Destination=TerminalSlot(ActiveCargoIndex,false);
            const bool Stable=FVector::Dist(Cargo->GetComponentLocation(),Destination)<20.f && Cargo->GetPhysicsLinearVelocity().Size()<8.f && Cargo->GetUpVector().Z>.99f;
            FleetSettle=Stable?FleetSettle+Dt:0;
            if (FleetSettle>1.f) { ContainerActors[ActiveCargoIndex]->LocationOwner=ECargoOwner::Yard; JobPlacementAt=AutoElapsed; FleetStep=3; }
        }
        else if (FleetStep==3 && MoveAGV(EFleetDestination::Park,Dt)) CompleteAutomaticJob();
    }
}

FString AQuayCrane::GetFleetStatus() const
{
    if (bUnifiedTerminal && SiteLogistics)
        return FString::Printf(TEXT("Ship %d / %d | Transit %d | Yard %d | Delivered %d | %s"),
            SiteLogistics->ShipRemaining(),SiteLogistics->InitialShipCount(),SiteLogistics->InTransit(),
            SiteLogistics->InitialYard+SiteLogistics->Delivered,SiteLogistics->Delivered,
            SiteLogistics->Fault.IsEmpty()?TEXT("STS > AGV > RMG"):*SiteLogistics->Fault);
    if (AGVActors.IsEmpty()) return TEXT("");
    if (SiteLogistics)
    {
        int32 CentralShip=0,CentralYard=0;
        for (const auto& C:ContainerActors) { CentralShip+=C->LocationOwner==ECargoOwner::Ship; CentralYard+=C->LocationOwner==ECargoOwner::Yard; }
        return FString::Printf(TEXT("Ship %d | AGV / crane transit %d | Yard %d (initial %d) | Delivered %d | %s"),
            SiteLogistics->ShipRemaining()+CentralShip,SiteLogistics->InTransit()+24-CentralShip-CentralYard,
            SiteLogistics->InitialYard+SiteLogistics->Delivered+CentralYard,SiteLogistics->InitialYard,SiteLogistics->Delivered+CentralYard,
            SiteLogistics->Fault.IsEmpty()?TEXT("STS > AGV > RMG") : *SiteLogistics->Fault);
    }
    int32 Jobs=0, Faults=0;
    for (const auto& Crane:WorkingCranes) { Jobs+=Crane->CompletedJobs; Faults+=!Crane->Fault.IsEmpty(); }
    return FString::Printf(TEXT("Site: %d cranes / %d moves / %d faults | AGV 1/2/3 jobs %d / %d / %d | Active AGV %d | RMG %s | step %d"),
        WorkingCranes.Num(),Jobs,Faults,AGVActors[0]->CompletedJobs,AGVActors[1]->CompletedJobs,AGVActors[2]->CompletedJobs,ActiveAGV+1,bRMGHasCargo?TEXT("CARRY"):TEXT("READY"),RMGStep);
}
