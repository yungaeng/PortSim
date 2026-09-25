#include "QuayCrane.h"
#include "PortWorkingCrane.h"
#include "PortSiteLogistics.h"
#include "Components/StaticMeshComponent.h"
#include "TerminalLayout.h"
#include "Components/TextRenderComponent.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "GameFramework/SpringArmComponent.h"
#include "PhysicsEngine/PhysicsConstraintComponent.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "HAL/FileManager.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"

DEFINE_LOG_CATEGORY_STATIC(LogPortTerminal, Log, All);
namespace Terminal
{
    constexpr int32 Count = 24;
    constexpr float HalfHeight = 129.5f;
    constexpr float SafeHeight = 1900.f;
    constexpr float BeamHeight = 3000.f;
}

FVector AQuayCrane::TerminalSlot(int32 Index, bool bShip) const
{
    const int32 Row = Index % 4;
    if (bShip) return FVector(-2200.f + ((Index % 12) / 4) * 400.f, -2400.f + Row * 1600.f,
        200.f + Terminal::HalfHeight + (Index / 12) * 259.f);
    return SiteLogistics->CentralSlot(Index);
}

void AQuayCrane::BuildTerminal()
{
    if (bUnifiedTerminal)
    {
        // Camera/UI pawn only; all nine operational STSs use the common crane actor.
        Suspension->BreakConstraint(); TwistLock->BreakConstraint();
        Spreader->SetSimulatePhysics(false);
        TInlineComponentArray<UStaticMeshComponent*> TrainingParts(this);
        for (auto* Part:TrainingParts)
        {
            Part->SetSimulatePhysics(false);
            Part->SetHiddenInGame(true);
            Part->SetVisibility(false);
            Part->SetCollisionEnabled(ECollisionEnabled::NoCollision);
        }
    }
    auto Box = [this](const FString& Name, FVector Position, FVector Size)
    {
        auto* Mesh = NewObject<UStaticMeshComponent>(this, FName(*Name));
        AddInstanceComponent(Mesh);
        Mesh->SetStaticMesh(TrolleyMesh->GetStaticMesh());
        Mesh->SetMobility(EComponentMobility::Static);
        Mesh->SetCollisionProfileName(TEXT("BlockAll"));
        Mesh->SetWorldLocation(Position);
        Mesh->SetWorldScale3D(Size / 100.f);
        Mesh->RegisterComponent();
        return Mesh;
    };
    Box(TEXT("Quay"), FVector((TerminalLayout::QuayLeftX+TerminalLayout::QuayRightX)*0.5f,0.f,-30.f),
        FVector(TerminalLayout::QuayRightX-TerminalLayout::QuayLeftX,TerminalLayout::QuayLength,100.f));
    const float SeaRail=STSProfile.bReady?STSProfile.WatersideRailX:-850.f;
    const float LandRail=STSProfile.bReady?SeaRail+STSProfile.RailGauge:850.f;
    if(!bUnifiedTerminal) Box(TEXT("RailPier"), FVector(SeaRail,0.f,-30.f), FVector(190.f,10000.f,100.f));
    auto* Water = Box(TEXT("Water"), FVector(-30700.f,0.f,-260.f), FVector(60000.f,145000.f,20.f));
    Water->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    if (bUnifiedTerminal)
    {
        BuildTerminalSite();
        CameraArm->TargetArmLength=185000.f;
        CameraArm->SetRelativeLocation(FVector(35000,0,0));
        CameraArm->SetRelativeRotation(FRotator(-52,38,0));
        return;
    }
    FActorSpawnParameters ShipParams;
    ShipParams.Owner=this;
    ShipParams.SpawnCollisionHandlingOverride=ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
    ShipActor=GetWorld()->SpawnActor<APortShipActor>(FVector(-1800,0,0),FRotator::ZeroRotator,ShipParams);
    check(ShipActor);
#if WITH_EDITOR
    ShipActor->SetActorLabel(TEXT("Ship_01"));
    ShipActor->SetFolderPath(TEXT("PortSim/Equipment"));
#endif
    Box(TEXT("RailLeft"), FVector(SeaRail,0.f,30.f), FVector(35.f,10000.f,20.f));
    Box(TEXT("RailRight"), FVector(LandRail,0.f,30.f), FVector(35.f,10000.f,20.f));
    for (int32 I=0; I<Terminal::Count; ++I)
    {
        auto* Container=SpawnContainer(I+1,TerminalSlot(I,true));
        CargoBodies.Add(Container->GetBody());
        CargoOnShip.Add(true);
    }
    Cargo=CargoBodies[0];
    CameraArm->TargetArmLength=13500.f;
    CameraArm->SetRelativeLocation(FVector(0.f,0.f,700.f));
    CameraArm->SetRelativeRotation(FRotator(-43.f,-38.f,0.f));
    if(!STSProfile.bReady) { TravelSpeed=350.f; Acceleration=220.f; HoistSpeed=250.f; }
    BuildFleet();
    BuildTerminalSite();
}

void AQuayCrane::ResetTerminal()
{
    if (bUnifiedTerminal)
    {
        ResetSiteOperations();
        bLocked=bEmergencyStop=bAutoPaused=bAutoLoading=false; bAutoRunning=true;
        AutoStage=ETerminalStage::Idle; AutoCompleted=Deliveries=0; AutoElapsed=0;
        Status=TEXT("Three equal berths: automatic STS > AGV > available yard RMG");
        return;
    }
    if(bAutoRunning) { RecordSTSStage(TEXT("RESET")); SaveSTSReports(); }
    Suspension->BreakConstraint(); TwistLock->BreakConstraint();
    Spreader->SetSimulatePhysics(false);
    ResetFleet();
    ResetSiteOperations();
    bLocked=false; bEmergencyStop=false; bAutoPaused=false; bAutoRunning=false;
    for(bool& Locked:STSCornerLocked) Locked=false;
    STSObservation=FSTSObservation(); NextSTSSample=0;
    JobSTSSeconds=JobPrepareSeconds=JobDeliverySeconds=JobPausedSeconds=0;
    JobHandoverAt=JobPlacementAt=-1;
    ResultsPath.Reset(); ResultsCsv.Reset(); StageEventsCsv.Reset();
    AutoStage=ETerminalStage::Idle; AutoQueue.Reset(); AutoCursor=0;
    AutoCompleted=0; AutoElapsed=0.f; JobElapsed=0.f; AutoStageTime=0.f; AutoStableTime=0.f;
    Deliveries=0; ActiveCargoIndex=0; Cargo=CargoBodies[0];
    DriveInput=DriveVelocity=FVector::ZeroVector;
    TrolleyPosition=-2200.f; GantryPosition=-2400.f; RopeLength=STSBeamHeight()-STSTransferHeight();
    GantryRoot->SetRelativeLocation(FVector(0.f,GantryPosition,0.f),false,nullptr,ETeleportType::TeleportPhysics);
    TrolleyMesh->SetRelativeLocation(FVector(TrolleyPosition,0.f,STSBeamHeight()),false,nullptr,ETeleportType::TeleportPhysics);
    for (int32 I=0; I<CargoBodies.Num(); ++I)
    {
        CargoOnShip[I]=true;
        ContainerActors[I]->ResetCargo(TerminalSlot(I,true));
    }
    Spreader->SetWorldLocationAndRotation(FVector(TrolleyPosition,GantryPosition,STSTransferHeight()),FRotator::ZeroRotator,false,nullptr,ETeleportType::TeleportPhysics);
    Spreader->SetSimulatePhysics(true);
    Spreader->SetPhysicsLinearVelocity(FVector::ZeroVector);
    Spreader->SetPhysicsAngularVelocityInDegrees(FVector::ZeroVector);
    Suspension->SetWorldLocation(TrolleyMesh->GetComponentLocation());
    Suspension->SetConstrainedComponents(TrolleyMesh,NAME_None,Spreader,NAME_None);
    Suspension->SetConstraintReferencePosition(EConstraintFrame::Frame1,FVector::ZeroVector);
    Suspension->SetConstraintReferencePosition(EConstraintFrame::Frame2,FVector::ZeroVector);
    Suspension->ConstraintInstance.SetLinearLimitSize(RopeLength);
    Status=TEXT("24 aboard | U: STS > AGV > RMG > yard | L: reverse loading | P: pause");
    if(!STSProfile.bReady) Status=TEXT("STS reference error: ")+STSProfile.Error;
    UpdateRopes();
}

int32 AQuayCrane::GetShipCargoCount() const
{
    if (bUnifiedTerminal) return SiteLogistics?SiteLogistics->ShipRemaining():0;
    int32 Count=0; for (bool bShip: CargoOnShip) if (bShip) ++Count; return Count;
}

const TCHAR* AQuayCrane::GetAutoStageName() const
{
    switch(AutoStage)
    {
    case ETerminalStage::Idle: return TEXT("IDLE");
    case ETerminalStage::FleetPrepare: return TEXT("AGV / RMG PREPARE");
    case ETerminalStage::FleetDeliver: return TEXT("AGV / RMG DELIVERY");
    case ETerminalStage::RaiseEmpty: return TEXT("CLEARANCE");
    case ETerminalStage::Approach: return TEXT("APPROACH SOURCE");
    case ETerminalStage::LowerPickup: return TEXT("ALIGN / PICKUP");
    case ETerminalStage::Lift: return TEXT("LIFT");
    case ETerminalStage::Transfer: return TEXT("TRANSFER");
    case ETerminalStage::LowerPlace: return TEXT("LOWER / PLACE");
    case ETerminalStage::Retract: return TEXT("RETRACT");
    case ETerminalStage::Verify: return TEXT("VERIFY SLOT");
    case ETerminalStage::Fault: return TEXT("FAULT");
    }
    return TEXT("UNKNOWN");
}

void AQuayCrane::SetAutoStage(ETerminalStage Stage)
{
    RecordSTSStage(TEXT("COMPLETE"));
    AutoStage=Stage; AutoStageTime=0.f; AutoStableTime=0.f;
    UE_LOG(LogPortTerminal,Display,TEXT("Job C%02d %s: %s"),ActiveCargoIndex+1,bAutoLoading?TEXT("LOAD"):TEXT("UNLOAD"),GetAutoStageName());
}

void AQuayCrane::StopAutomatic(const FString& Reason)
{
    RecordSTSStage(TEXT("FAULT"));
    bAutoRunning=false; bAutoPaused=false; DriveInput=DriveVelocity=FVector::ZeroVector;
    AutoStage=ETerminalStage::Fault;
    Status=TEXT("AUTO STOP: ")+Reason+TEXT(" | Inspect cargo; R resets the scene.");
    UE_LOG(LogPortTerminal,Error,TEXT("%s"),*Status);
    SaveSTSReports();
}

bool AQuayCrane::IsTerminalSlotAvailable(int32 Index,bool bShip) const
{
    const FVector Destination=TerminalSlot(Index,bShip);
    for (int32 I=0; I<CargoBodies.Num(); ++I)
    {
        if (I==Index) continue;
        const FVector Delta=CargoBodies[I]->GetComponentLocation()-Destination;
        if (FMath::Abs(Delta.X)<(bShip?245.f:1221.f) && FMath::Abs(Delta.Y)<(bShip?1221.f:245.f) && FMath::Abs(Delta.Z)<250.f) return false;
    }
    if (bShip && Index>=12)
        return CargoOnShip[Index-12] && FVector::Dist(CargoBodies[Index-12]->GetComponentLocation(),TerminalSlot(Index-12,true))<20.f;
    return true;
}

void AQuayCrane::StartAutomatic(bool bLoad)
{
    if (!bTerminalMode) return;
    bSTSStartPending=false;
    if(!STSProfile.bReady) { Status=TEXT("Automatic operation blocked: ")+STSProfile.Error; return; }
    if (bUnifiedTerminal)
    {
        if (bLoad) { Status=TEXT("Unified berths unload automatically; R restores all three ships."); return; }
        if (bEmergencyStop || !SiteLogistics->Fault.IsEmpty()) { Status=TEXT("Clear E-stop or reset after a fault."); return; }
        bAutoPaused=false; bAutoRunning=true;
        return;
    }
    if (bAutoRunning) { Status=TEXT("A batch is already running. P pauses; R resets. Direction is unchanged."); return; }
    // Keep the original fault visible until reset, including repeated start requests.
    if (AutoStage==ETerminalStage::Fault) return;
    if (bLocked || bEmergencyStop)
    { Status=TEXT("Cannot start: release manual cargo / clear E-stop, or R after a fault."); return; }
    AutoQueue.Reset();
    if (bLoad)
    { for (int32 I=0;I<CargoBodies.Num();++I) if (!CargoOnShip[I]) AutoQueue.Add(I); }
    else
    { for (int32 I=CargoBodies.Num()-1;I>=0;--I) if (CargoOnShip[I]) AutoQueue.Add(I); }
    if (AutoQueue.IsEmpty()) { Status=bLoad?TEXT("Nothing to load: every container is aboard."):TEXT("Nothing to unload: ship is empty."); return; }
    bAutoLoading=bLoad; bAutoRunning=true; bAutoPaused=false;
    AutoCursor=0; AutoCompleted=0; AutoElapsed=0.f;
    ResultsCsv=TEXT("ContainerID,Direction,SimulationSeconds,DestinationX,DestinationY,DestinationZ,AGVID,PayloadKg,STSActiveSeconds,FleetPrepareSeconds,FleetDeliverySeconds,PausedSeconds,BatchElapsedSeconds,STSHandoverAtSeconds,FinalPlacementAtSeconds\n");
    StageEventsCsv=TEXT("ContainerID,Direction,Stage,BatchElapsedSeconds,StageActiveSeconds,Outcome\n");
    ResultsPath=FPaths::ProjectSavedDir()/TEXT("Results")/FString::Printf(TEXT("Terminal_%s_%s.csv"),bLoad?TEXT("Load"):TEXT("Unload"),*FGuid::NewGuid().ToString(EGuidFormats::Digits));
    if(!SaveSTSReports()) { StopAutomatic(TEXT("Could not create STS reports.")); return; }
    BeginAutomaticJob();
}

void AQuayCrane::BeginAutomaticJob()
{
    if (!AutoQueue.IsValidIndex(AutoCursor))
    {
        bAutoRunning=false; AutoStage=ETerminalStage::Idle; DriveInput=FVector::ZeroVector;
        Status=FString::Printf(TEXT("%s complete: %d/%d. U unload | L load | R reset. Results saved."),bAutoLoading?TEXT("Loading"):TEXT("Unloading"),AutoCompleted,AutoQueue.Num());
        UE_LOG(LogPortTerminal,Display,TEXT("BATCH_COMPLETE %s %d/%d, time %.2fs"),bAutoLoading?TEXT("LOAD"):TEXT("UNLOAD"),AutoCompleted,AutoQueue.Num(),AutoElapsed);
        return;
    }
    ActiveCargoIndex=AutoQueue[AutoCursor]; Cargo=CargoBodies[ActiveCargoIndex];
    JobElapsed=0; JobSTSSeconds=JobPrepareSeconds=JobDeliverySeconds=JobPausedSeconds=0;
    JobHandoverAt=JobPlacementAt=-1;
    if(!FMath::IsFinite(GetCargoMassKg()) || GetCargoMassKg()<=0 || GetCargoMassKg()>STSProfile.RatedPayloadKg)
    { StopAutomatic(TEXT("Payload exceeds resolved STS reference capacity or mass is invalid.")); return; }
    ActiveAGV=AutoCursor%3; FleetStep=0; RMGStep=0; FleetSettle=0; AGVActors[ActiveAGV]->Speed=0; RMGActor=SiteLogistics->CentralCrane(ActiveCargoIndex); bFleetRouteActive=false; FleetWaypoint=0;
    const FVector Handover(3000.f,-2400.f+ActiveAGV*2400.f,349.5f);
    AutoSource=bAutoLoading?Handover:TerminalSlot(ActiveCargoIndex,true);
    AutoDestination=bAutoLoading?TerminalSlot(ActiveCargoIndex,true):Handover;
    if(!STSProfile.ContainsTarget(AutoSource+FVector(0,0,154.5f)) || !STSProfile.ContainsTarget(AutoDestination+FVector(0,0,154.5f)))
    { StopAutomatic(TEXT("Source/destination outside reference STS working envelope.")); return; }
    SampleSTSSensors(true);
    if (FVector::Dist(Cargo->GetComponentLocation(),TerminalSlot(ActiveCargoIndex,!bAutoLoading))>30.f || Cargo->GetUpVector().Z<0.99f)
    { StopAutomatic(TEXT("Source container is displaced or tilted.")); return; }
    if (!IsTerminalSlotAvailable(ActiveCargoIndex,bAutoLoading))
    { StopAutomatic(TEXT("Destination occupied or lower-tier support missing.")); return; }
    if (!bAutoLoading && ActiveCargoIndex<12 && CargoOnShip[ActiveCargoIndex+12])
    { StopAutomatic(TEXT("Cannot lift a container underneath another container.")); return; }
    JobElapsed=0.f; SetAutoStage(ETerminalStage::FleetPrepare);
    Status=FString::Printf(TEXT("%s C%02d | batch %d/%d | P pause | Space emergency stop"),bAutoLoading?TEXT("Loading"):TEXT("Unloading"),ActiveCargoIndex+1,AutoCursor+1,AutoQueue.Num());
}

bool AQuayCrane::DriveSpreaderTo(FVector Target)
{
    AutoDriveTarget=Target; bAutoDriveTarget=true;
    if(!STSProfile.ContainsTarget(Target)) { StopAutomatic(TEXT("Automatic target outside STS envelope.")); return false; }
    const float TargetRope=STSBeamHeight()-Target.Z;
    const FVector P=STSObservation.DrivePosition, V=STSObservation.DriveVelocity;
    const float VerticalLimit=AutomaticHoistLimit();
    SetDriveInput((0.8f*(Target.X-P.X)-0.8f*V.X)/TravelSpeed,
        (0.8f*(Target.Y-P.Y)-0.8f*V.Y)/STSGantrySpeed(),
        (0.8f*(P.Z-TargetRope)-0.8f*V.Z)/FMath::Max(1.f,VerticalLimit));
    return FMath::Abs(Target.X-P.X)<5.f && FMath::Abs(Target.Y-P.Y)<5.f &&
        FMath::Abs(P.Z-TargetRope)<4.f && V.Size()<7.f;
}

void AQuayCrane::TickAutomatic(float Dt)
{
    if (!bAutoRunning) return;
    DriveInput=FVector::ZeroVector;
    AutoElapsed+=Dt; JobElapsed+=Dt;
    if (bAutoPaused || bEmergencyStop) { JobPausedSeconds+=Dt; return; }
    AutoStageTime+=Dt;
    if(!STSObservation.IsFresh(STSSimulationTime,STSProfile.SensorMaxAge)) { StopAutomatic(TEXT("Required STS sensor observation invalid/stale.")); return; }
    if(bLocked && !STSLoadedHoistAllowed()) { StopAutomatic(TEXT("Loaded hoist interlock: locks/load observation invalid.")); return; }
    const bool FleetStage=AutoStage==ETerminalStage::FleetPrepare || AutoStage==ETerminalStage::FleetDeliver;
    if(AutoStage==ETerminalStage::FleetPrepare) JobPrepareSeconds+=Dt;
    else if(AutoStage==ETerminalStage::FleetDeliver) JobDeliverySeconds+=Dt;
    else JobSTSSeconds+=Dt;
    if (AutoStageTime>FMath::Max(FleetStage?12000.f:0.f,STSProfile.StageTimeout)) { StopAutomatic(FString::Printf(TEXT("Timed out at %s for C%02d spreader=%s speed=%s trolley=%s cargo=%s"),GetAutoStageName(),ActiveCargoIndex+1,*Spreader->GetComponentLocation().ToString(),*Spreader->GetPhysicsLinearVelocity().ToString(),*TrolleyMesh->GetComponentLocation().ToString(),*Cargo->GetComponentLocation().ToString())); return; }
    if (FleetStage) { TickFleet(Dt); return; }
    FVector Target(TrolleyPosition,GantryPosition,STSTransferHeight());
    bool bReady=false;
    switch(AutoStage)
    {
    case ETerminalStage::RaiseEmpty:
        if (DriveSpreaderTo(Target) && STSObservation.SpreaderPosition.Z>STSTransferHeight()-25.f) SetAutoStage(ETerminalStage::Approach);
        break;
    case ETerminalStage::Approach:
        Target=FVector(AutoSource.X,AutoSource.Y,STSTransferHeight());
        bReady=DriveSpreaderTo(Target) && FVector::Dist2D(STSObservation.SpreaderPosition,Target)<25.f && STSObservation.SpreaderVelocity.Size()<20.f;
        AutoStableTime=bReady?AutoStableTime+Dt:0.f;
        if (AutoStableTime>0.5f) SetAutoStage(ETerminalStage::LowerPickup);
        break;
    case ETerminalStage::LowerPickup:
        Target=STSObservation.CargoPosition+FVector(0.f,0.f,Terminal::HalfHeight+25.f);
        bReady=DriveSpreaderTo(Target) && STSObservation.bLanded && (!bAutoLoading || STSObservation.bAGVAligned);
        AutoStableTime=bReady?AutoStableTime+Dt:0.f;
        if (AutoStableTime>STSProfile.SettleTime)
        {
            ToggleLock();
            if (!bLocked) { StopAutomatic(TEXT("Twist lock alignment failed.")); return; }
            ContainerActors[ActiveCargoIndex]->LocationOwner=ECargoOwner::STS;
            SetAutoStage(ETerminalStage::Lift);
        }
        break;
    case ETerminalStage::Lift:
        Target=FVector(AutoSource.X,AutoSource.Y,STSTransferHeight());
        if (DriveSpreaderTo(Target) && STSObservation.CargoPosition.Z>STSTransferHeight()-300.f) SetAutoStage(ETerminalStage::Transfer);
        break;
    case ETerminalStage::Transfer:
        Target=FVector(AutoDestination.X,AutoDestination.Y,STSTransferHeight());
        bReady=DriveSpreaderTo(Target) && FVector::Dist2D(STSObservation.CargoPosition,Target)<STSProfile.LandingTolerance && STSObservation.CargoVelocity.Size()<STSProfile.SettleSpeed && STSObservation.SwayDegrees<STSProfile.SwayLimitDegrees;
        AutoStableTime=bReady?AutoStableTime+Dt:0.f;
        if (AutoStableTime>0.6f)
        {
            if (!IsTerminalSlotAvailable(ActiveCargoIndex,bAutoLoading)) { StopAutomatic(TEXT("Destination became blocked.")); return; }
            if (!bAutoLoading && (!STSObservation.bAGVAligned || bAGVHasCargo))
            { StopAutomatic(TEXT("STS handover requires stopped, aligned, empty AGV.")); return; }
            SetAutoStage(ETerminalStage::LowerPlace);
        }
        break;
    case ETerminalStage::LowerPlace:
        if(!bAutoLoading && !STSObservation.bAGVAligned) { StopAutomatic(TEXT("AGV alignment lost during lowering.")); return; }
        Target=AutoDestination+FVector(0.f,0.f,Terminal::HalfHeight+25.f);
        bReady=DriveSpreaderTo(Target) && FVector::Dist(STSObservation.CargoPosition,AutoDestination)<STSProfile.LandingTolerance && STSObservation.CargoVelocity.Size()<STSProfile.SettleSpeed && STSObservation.bCargoSupported;
        AutoStableTime=bReady?AutoStableTime+Dt:0.f;
        if (AutoStableTime>STSProfile.SettleTime) { ToggleLock(); if(bLocked) { StopAutomatic(Status); return; } SetAutoStage(ETerminalStage::Retract); }
        break;
    case ETerminalStage::Retract:
        Target=FVector(AutoDestination.X,AutoDestination.Y,STSTransferHeight());
        if (DriveSpreaderTo(Target)) SetAutoStage(ETerminalStage::Verify);
        break;
    case ETerminalStage::Verify:
        bReady=!bLocked && FVector::Dist(STSObservation.CargoPosition,AutoDestination)<STSProfile.LandingTolerance &&
            STSObservation.CargoVelocity.Size()<8.f && STSObservation.bCargoSupported && Cargo->GetUpVector().Z>0.99f;
        AutoStableTime=bReady?AutoStableTime+Dt:0.f;
        if (AutoStableTime>1.f)
        {
            JobHandoverAt=AutoElapsed;
            if(bAutoLoading) JobPlacementAt=AutoElapsed;
            if (!bAutoLoading) HoldFleetCargo(true);
            else ContainerActors[ActiveCargoIndex]->LocationOwner=ECargoOwner::Ship;
            FleetStep=0; RMGStep=0; SetAutoStage(ETerminalStage::FleetDeliver);
        }
        break;
    default: break;
    }
}

void AQuayCrane::CompleteAutomaticJob()
{
    if(!FMath::IsNearlyEqual(JobElapsed,JobSTSSeconds+JobPrepareSeconds+JobDeliverySeconds+JobPausedSeconds,0.00001) ||
        JobHandoverAt<0 || JobPlacementAt<JobHandoverAt || JobPlacementAt>AutoElapsed)
    { StopAutomatic(TEXT("Job timing accounting or handover/placement event order mismatch.")); return; }
    // Commit the manifest only after the entire intermodal transfer and slot settlement.
    const FVector Destination=TerminalSlot(ActiveCargoIndex,bAutoLoading);
    if (bLocked || bAGVHasCargo || bRMGHasCargo || FVector::Dist(Cargo->GetComponentLocation(),Destination)>20.f)
    { StopAutomatic(TEXT("Final cargo ownership or physical slot mismatch.")); return; }
    SiteLogistics->CompleteCentral(ActiveCargoIndex,!bAutoLoading);
    CargoOnShip[ActiveCargoIndex]=bAutoLoading;
    ContainerActors[ActiveCargoIndex]->LocationOwner=bAutoLoading?ECargoOwner::Ship:ECargoOwner::Yard;
    ++AutoCompleted; ++Deliveries; ++AGVActors[ActiveAGV]->CompletedJobs;
    ResultsCsv+=FString::Printf(TEXT("C%02d,%s,%.6f,%.1f,%.1f,%.1f,AGV%d,%.1f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n"),ActiveCargoIndex+1,
        bAutoLoading?TEXT("Load"):TEXT("Unload"),JobElapsed,Destination.X,Destination.Y,Destination.Z,ActiveAGV+1,GetCargoMassKg(),JobSTSSeconds,JobPrepareSeconds,JobDeliverySeconds,JobPausedSeconds,AutoElapsed,JobHandoverAt,JobPlacementAt);
    RecordSTSStage(TEXT("COMPLETE"));
    if (!SaveSTSReports())
    { StopAutomatic(TEXT("Could not save job results CSV.")); return; }
    UE_LOG(LogPortTerminal,Display,TEXT("FLEET_JOB C%02d AGV%d %s owner=%s"),ActiveCargoIndex+1,ActiveAGV+1,
        bAutoLoading?TEXT("LOAD"):TEXT("UNLOAD"),bAutoLoading?TEXT("SHIP"):TEXT("YARD"));
    AutoStage=ETerminalStage::Idle; AutoStageTime=0;
    ++AutoCursor; BeginAutomaticJob();
}

void AQuayCrane::TickTerminalTest(float Dt)
{
    TerminalTestTime+=Dt;
    FString ActorError;
    if (!ValidateTerminalActors(ActorError) || !SiteLogistics->Validate(ActorError)) { StopAutomatic(ActorError); }
    auto Finish=[this](bool Pass,const FString& Why)
    {
        UE_LOG(LogPortTerminal,Display,TEXT("PORTSIM_TERMINAL_%s: %s"),Pass?TEXT("PASS"):TEXT("FAIL"),*Why);
        bTerminalTest=false; FPlatformMisc::RequestExitWithStatus(false,Pass?0:1);
    };
    if (FParse::Param(FCommandLine::Get(),TEXT("PortSimControlTest")) && AutoCompleted>0)
    { Finish(true,TEXT("Accelerated STS pickup, AGV route, yard RMG placement and vehicle return completed")); return; }
    if (TerminalTestTime>48000.f || AutoStage==ETerminalStage::Fault)
    { Finish(false,Status); return; }
    if (FParse::Param(FCommandLine::Get(),TEXT("PortSimRoundTripTest")))
    {
        if (TerminalTestStage==0 && TerminalTestTime>2.f)
        {
            StartAutomatic(false); AutoQueue.SetNum(2); TerminalTestStage=1;
        }
        else if (TerminalTestStage==1 && !bAutoRunning)
        {
            if (AutoCompleted!=2 || GetShipCargoCount()!=22) { Finish(false,TEXT("Round-trip unload inventory mismatch")); return; }
            StartAutomatic(true); TerminalTestStage=2;
        }
        else if (TerminalTestStage==2 && !bAutoRunning)
        {
            if (AutoCompleted!=2 || GetShipCargoCount()!=24) { Finish(false,TEXT("Round-trip load inventory mismatch")); return; }
            ResetTerminal();
            if (!ValidateTerminalActors(ActorError) || !SiteLogistics->Validate(ActorError)) { Finish(false,ActorError); return; }
            Finish(true,TEXT("Two yard deliveries and reverse shipments, shared traffic, conserved cargo IDs and reset"));
        }
        return;
    }
    // Exercise pause and E-stop while AGV is carrying, including RMG/cargo freeze.
    if (FleetPauseTest==0 && bAGVHasCargo && AGVActors[ActiveAGV]->Speed>100.f)
    {
        bAutoPaused=true; FleetPauseTest=1; FleetPauseStart=TerminalTestTime;
        FleetTestAGV=AGVActors[ActiveAGV]->GetActorLocation();
        FleetTestRMG=RMGActor->HeadPosition(); FleetTestCargo=Cargo->GetComponentLocation();
    }
    else if ((FleetPauseTest==1 || FleetPauseTest==2) && TerminalTestTime-FleetPauseStart>2.f)
    {
        if (FVector::Dist(FleetTestAGV,AGVActors[ActiveAGV]->GetActorLocation())>.1f ||
            FVector::Dist(FleetTestRMG,RMGActor->HeadPosition())>.1f || FVector::Dist(FleetTestCargo,Cargo->GetComponentLocation())>.1f)
        { Finish(false,TEXT("Fleet pause/E-stop moved equipment or cargo")); return; }
        bAutoPaused=false; bEmergencyStop=FleetPauseTest==1; ++FleetPauseTest; FleetPauseStart=TerminalTestTime;
    }
    if (TerminalTestStage==0 && TerminalTestTime>2.f)
    {
        if (CargoBodies.Num()!=24 || GetShipCargoCount()!=24) { Finish(false,TEXT("Initial manifest incorrect")); return; }
        StartAutomatic(false); StartAutomatic(true);
        if (bAutoLoading || AutoQueue.Num()!=24) { Finish(false,TEXT("Re-entrant start altered batch")); return; }
        TerminalTestStage=FParse::Param(FCommandLine::Get(),TEXT("PortSimFleetResetTest"))?6:1;
    }
    else if (TerminalTestStage==1 && AutoElapsed>3.f)
    {
        bAutoPaused=true; TerminalTestHold=TerminalTestTime; TestHoldPosition=FVector(TrolleyPosition,GantryPosition,RopeLength); TerminalTestStage=2;
    }
    else if (TerminalTestStage==2 && TerminalTestTime-TerminalTestHold>2.f)
    {
        if (FVector::Dist(TestHoldPosition,FVector(TrolleyPosition,GantryPosition,RopeLength))>0.1f) { Finish(false,TEXT("Pause moved drives")); return; }
        bAutoPaused=false; bEmergencyStop=true; TerminalTestHold=TerminalTestTime; TerminalTestStage=3;
    }
    else if (TerminalTestStage==3 && TerminalTestTime-TerminalTestHold>2.f)
    {
        if (FVector::Dist(TestHoldPosition,FVector(TrolleyPosition,GantryPosition,RopeLength))>0.1f) { Finish(false,TEXT("E-stop moved drives")); return; }
        bEmergencyStop=false; TerminalTestStage=4;
    }
    else if (TerminalTestStage==4 && !bAutoRunning)
    {
        if (AutoCompleted!=24 || GetShipCargoCount()!=0 || bLocked) { Finish(false,TEXT("Unloading lost/duplicated cargo")); return; }
        for (int32 I=0;I<24;++I) if (FVector::Dist(CargoBodies[I]->GetComponentLocation(),TerminalSlot(I,false))>25.f)
        { Finish(false,TEXT("Unload slot mismatch")); return; }
        for (int32 I=0;I<3;++I) if (AGVActors[I]->CompletedJobs!=8) { Finish(false,TEXT("AGV dispatch distribution incorrect")); return; }
        for (const auto& Container: ContainerActors) if (Container->LocationOwner!=ECargoOwner::Yard) { Finish(false,TEXT("Yard ownership mismatch")); return; }
        StartAutomatic(true); TerminalTestStage=5;
    }
    else if (TerminalTestStage==5 && !bAutoRunning)
    {
        if (AutoCompleted!=24 || GetShipCargoCount()!=24 || bLocked) { Finish(false,TEXT("Loading lost/duplicated cargo")); return; }
        for (int32 I=0;I<24;++I) if (FVector::Dist(CargoBodies[I]->GetComponentLocation(),TerminalSlot(I,true))>25.f)
        { Finish(false,TEXT("Load slot mismatch")); return; }
        for (int32 I=0;I<3;++I) if (AGVActors[I]->CompletedJobs!=16) { Finish(false,TEXT("Return load bypassed AGV")); return; }
        for (const auto& Container: ContainerActors) if (Container->LocationOwner!=ECargoOwner::Ship) { Finish(false,TEXT("Ship ownership mismatch")); return; }
        StartAutomatic(false); TerminalTestStage=6;
    }
    else if (TerminalTestStage==6 && bLocked)
    {
        ResetTerminal();
        if (bLocked || bAutoRunning || GetShipCargoCount()!=24 || Deliveries!=0) { Finish(false,TEXT("Reset while carrying failed")); return; }
        StartAutomatic(false); TerminalTestStage=7;
    }
    else if (TerminalTestStage==7 && bAGVHasCargo)
    {
        ResetTerminal();
        if (bAGVHasCargo || bRMGHasCargo || !CargoBodies[23]->IsSimulatingPhysics()) { Finish(false,TEXT("AGV carry reset failed")); return; }
        StartAutomatic(false); TerminalTestStage=8;
    }
    else if (TerminalTestStage==8 && bRMGHasCargo)
    {
        ResetTerminal();
        if (bAGVHasCargo || bRMGHasCargo || !CargoBodies[23]->IsSimulatingPhysics() || GetShipCargoCount()!=24)
        { Finish(false,TEXT("RMG carry reset failed")); return; }
        Finish(true,FParse::Param(FCommandLine::Get(),TEXT("PortSimFleetResetTest"))?TEXT("STS/AGV/RMG carry reset and restart passed"):TEXT("3 AGVs + 18 distributed yard blocks: 24 unload + 24 load, all physical slots and owners, each AGV 16 jobs, moving fleet pause/E-stop, STS/AGV/RMG carry reset, CSV export"));
    }
}
