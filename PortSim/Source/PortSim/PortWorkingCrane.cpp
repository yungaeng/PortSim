#include "PortWorkingCrane.h"
#include "SimulationPerformance.h"
#include "PortContainerActor.h"
#include "PortSiteLogistics.h"
#include "PortAGVActor.h"
#include "Components/StaticMeshComponent.h"
#include "Components/TextRenderComponent.h"
#include "Engine/World.h"
#include "Materials/MaterialInterface.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "Misc/Paths.h"
#include "SpreaderTelescope.h"
#include "Misc/ScopeExit.h"
#include "PhysicsEngine/BodyInstance.h"

namespace WorkingCrane { constexpr float LiftOffset=154.5f; }

APortWorkingCrane::APortWorkingCrane()
{
    for (int32 I=0;I<4;++I)
    {
        Legs.Add(Box(*FString::Printf(TEXT("Leg_%d"),I),RootComponent,FVector::ZeroVector,FVector(100),true));
        Bogies.Add(Box(*FString::Printf(TEXT("Bogie_%d"),I),RootComponent,FVector::ZeroVector,FVector(100),true));
        Ropes.Add(Box(*FString::Printf(TEXT("Rope_%d"),I),RootComponent,FVector::ZeroVector,FVector(6)));
    }
    for (int32 I=0;I<2;++I)
    {
        CrossBeams.Add(Box(*FString::Printf(TEXT("CrossBeam_%d"),I),RootComponent,FVector::ZeroVector,FVector(100)));
        Beams.Add(Box(*FString::Printf(TEXT("Beam_%d"),I),RootComponent,FVector::ZeroVector,FVector(100)));
        auto* Pad=Box(*FString::Printf(TEXT("Slot_%d"),I),RootComponent,FVector::ZeroVector,FVector(100),true);
        Pad->SetAbsolute(true,true,true); // Fixed supports do not follow gantry travel.
        Pads.Add(Pad);
    }
    Trolley=Box(TEXT("Trolley"),RootComponent,FVector::ZeroVector,FVector(420,1450,120));
    // Spreader is the unscaled moving reference frame.  The visible centre,
    // telescopic arms, end beams and twistlocks move within this common frame.
    Spreader=Box(TEXT("Spreader"),RootComponent,FVector::ZeroVector,FVector(100));
    Spreader->SetVisibility(false);
    SpreaderCenter=Box(TEXT("SpreaderCenter"),Spreader,FVector::ZeroVector,FVector(243.8f,500.f,50.f));
    for(int32 End=0;End<2;++End)
    {
        TelescopeArms.Add(Box(*FString::Printf(TEXT("TelescopeArm_%d"),End),Spreader,FVector::ZeroVector,FVector(125.f,350.f,34.f)));
        SpreaderEndBeams.Add(Box(*FString::Printf(TEXT("SpreaderEndBeam_%d"),End),Spreader,FVector::ZeroVector,FVector(243.8f,40.f,60.f)));
    }
    for(int32 Corner=0;Corner<4;++Corner)
        TwistLocks.Add(Box(*FString::Printf(TEXT("TwistLock_%d"),Corner),Spreader,FVector::ZeroVector,FVector(16.f,16.f,20.f)));
    Mast=Box(TEXT("Mast"),RootComponent,FVector::ZeroVector,FVector(100));
    NameLabel=Label(TEXT("CraneName"),FVector::ZeroVector);
    Tags.Add(TEXT("PortSim.WorkingCrane"));
}

void APortWorkingCrane::Configure(int32 Number,bool bQuayside,FVector Source,FVector Destination,bool bCreateCargo)
{
    check(!bConfigured);
    bExternalJobs=!bCreateCargo; CraneID=Number; bSTS=bQuayside; Home=GetActorLocation(); Orientation=GetActorQuat();
    Slots[0]=Source; Slots[1]=Destination;
    if(!bSTS && !STSProfile.bReady)
    {
        FString File=FPaths::ProjectConfigDir()/TEXT("RMG_Simulation.json");
        FParse::Value(FCommandLine::Get(),TEXT("PortSimRMGSettings="),File);
        STSProfile.LoadRMG(File);
    }
    BeamZ=STSProfile.bReady?STSProfile.BeamHeight:(bSTS?3000.f:2600.f);
    SafeZ=STSProfile.bReady?STSProfile.SafeHeight:1900.f;
    const float HalfGauge=STSProfile.bReady?STSProfile.RailGauge*.5f:(bSTS?850.f:1600.f);
    const float GaugeCenter=bSTS && STSProfile.bReady?STSProfile.WatersideRailX+HalfGauge:0.f;
    const float HalfBase=bSTS?1000.f:820.f;
    const float Height=bSTS?BeamZ-100.f:BeamZ-40.f;
    for (int32 I=0;I<4;++I)
    {
        Legs[I]->SetRelativeLocation(FVector(GaugeCenter+(I<2?-1:1)*HalfGauge,(I%2?-1:1)*HalfBase,bSTS?Height*.5f:Height*.5f+50.f));
        Legs[I]->SetRelativeScale3D(FVector(1,1,Height/100.f));
        Bogies[I]->SetRelativeLocation(FVector(GaugeCenter+(I<2?-1:1)*HalfGauge,(I%2?-1:1)*HalfBase,100));
        Bogies[I]->SetRelativeScale3D(bSTS?FVector(1.8,4.2,1.8):FVector(1.6,3.2,1.6));
    }
    for (int32 I=0;I<2;++I)
    {
        const float BeamCenter=STSProfile.bReady?(STSProfile.MinTrolley()+STSProfile.MaxTrolley())*.5f:(bSTS?-1500.f:0.f);
        Beams[I]->SetRelativeLocation(FVector(BeamCenter,(I?-1:1)*(bSTS?730.f:650.f),bSTS?BeamZ+100.f:BeamZ));
        CrossBeams[I]->SetRelativeLocation(FVector(GaugeCenter+(I?-1:1)*HalfGauge,0,bSTS?BeamZ-100.f:BeamZ-20.f));
        CrossBeams[I]->SetRelativeScale3D(bSTS?FVector(1.4,22,1.4):FVector(1.6,18,1.6));
        const float BeamLength=STSProfile.bReady?(STSProfile.MaxTrolley()-STSProfile.MinTrolley()+400.f)/100.f:(bSTS?125.f:34.f);
        Beams[I]->SetRelativeScale3D(FVector(BeamLength,bSTS?1.f:1.2f,1.8f));
        Pads[I]->SetWorldLocationAndRotation(Slots[I]-FVector(0,0,139.5f),Orientation,false,nullptr,ETeleportType::TeleportPhysics);
        Pads[I]->SetWorldScale3D(FVector(3.1f,13.f,.2f));
    }
    Mast->SetRelativeLocation(FVector(0,0,BeamZ+1200));
    Mast->SetRelativeScale3D(FVector(3,3,24));
    Mast->SetVisibility(false); // Use the central STS twin-boom silhouette, without a dummy mast.
    Trolley->SetRelativeScale3D(bSTS?FVector(3.6,14.5,1):FVector(4.2,14.5,1.2));
    NameLabel->SetRelativeLocation(FVector(0,0,BeamZ+150));
    NameLabel->SetWorldSize(170);
    NameLabel->SetText(FText::FromString(FString::Printf(TEXT("%s %02d"),bSTS?TEXT("CC 24-row / STS"):TEXT("TC / RMG"),CraneID)));
    auto Material=[](const TCHAR* Name) { return LoadObject<UMaterialInterface>(nullptr,*FString::Printf(TEXT("/Game/PortSim/Assets/Materials/M_%s.M_%s"),Name,Name)); };
    for (const auto& Part:Legs) Part->SetMaterial(0,Material(bSTS?TEXT("CraneYellow"):TEXT("Target")));
    for (const auto& Part:Beams) Part->SetMaterial(0,Material(bSTS?TEXT("CraneYellow"):TEXT("Target")));
    for (const auto& Part:Ropes) Part->SetMaterial(0,Material(TEXT("SiteRoad")));
    for (const auto& Part:Pads) Part->SetMaterial(0,Material(TEXT("SiteWhite")));
    for (const auto& Part:Bogies) Part->SetMaterial(0,Material(TEXT("Steel")));
    for (const auto& Part:CrossBeams) Part->SetMaterial(0,Material(bSTS?TEXT("CraneYellow"):TEXT("Target")));
    Mast->SetMaterial(0,Material(TEXT("CraneYellow")));
    Trolley->SetMaterial(0,Material(bSTS?TEXT("CraneYellow"):TEXT("Target")));
    Spreader->SetMaterial(0,Material(bSTS?TEXT("CraneYellow"):TEXT("Target")));
    SpreaderCenter->SetMaterial(0,Material(bSTS?TEXT("CraneYellow"):TEXT("Target")));
    for(const auto& Part:TelescopeArms) Part->SetMaterial(0,Material(TEXT("Steel")));
    for(const auto& Part:SpreaderEndBeams) Part->SetMaterial(0,Material(bSTS?TEXT("CraneYellow"):TEXT("Target")));
    for(const auto& Part:TwistLocks) Part->SetMaterial(0,Material(TEXT("Steel")));
    if (bCreateCargo)
    {
    FActorSpawnParameters Params;
    Params.Owner=this;
    Params.SpawnCollisionHandlingOverride=ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
    CargoActor=GetWorld()->SpawnActor<APortContainerActor>(Source,Orientation.Rotator(),Params);
    check(CargoActor);
    CargoActor->InitializeContainer(1000+Number);
    if(STSProfile.bReady) CargoActor->SetPhysicalParameters(STSProfile.ContainerMassKg,STSProfile.ContainerCoG);
    }
    bConfigured=true;
    UpdateSpreaderGeometry();
#if WITH_EDITOR
    SetActorLabel(FString::Printf(TEXT("Working_%s_%02d"),bSTS?TEXT("STS"):TEXT("RMG"),Number));
    SetFolderPath(TEXT("PortSim/WorkingEquipment"));
#endif
    ResetOperation();
    if(STSProfile.bReady) BuildSTSSensors();
    bSensorFault=FParse::Param(FCommandLine::Get(),bSTS?TEXT("PortSimSTSSensorFault"):TEXT("PortSimRMGSensorFault"));
    // Legacy all-crane regression injection remains fatal; targeted editor/API
    // and scheduled failures use equipment isolation instead.
    bAppliedSensorFault=bFatalSensorTest=bSensorFault;
    FParse::Value(FCommandLine::Get(),bSTS?TEXT("PortSimSTSLockFault="):TEXT("PortSimRMGLockFault="),LockFault);
    if(!STSProfile.bReady) Stop(TEXT("Crane profile unavailable: ")+STSProfile.Error);
}

FVector APortWorkingCrane::Local(FVector World) const { return Orientation.UnrotateVector(World-Home); }
FVector APortWorkingCrane::HeadPosition() const { return Home+Orientation.RotateVector(Head+SuspendedOffset); }

void APortWorkingCrane::ResetOperation()
{
    if (!bConfigured) return;
    FaultType=EPortCraneFault::None; RecoveryStatus.Empty(); bRailSafetyHold=false;
    bGantryFault=bAppliedGantryFault=bFaultEvacuating=bFaultEvacuationComplete=false;
    SensorRepairElapsed=0; SensorRepairPhase.Empty();
    if(!bFatalSensorTest) bSensorFault=bAppliedSensorFault=false;
    ClearSTSState();
    SuspensionState=FSTSSuspension(); SuspendedOffset=FVector::ZeroVector;
    if (bExternalJobs)
    {
        if (IsValid(CargoActor)) CargoActor->DetachFromActor(FDetachmentTransformRules::KeepWorldTransform);
        CargoActor=nullptr; bJobActive=bCarrying=bPaused=bResumePhysics=false; bDestinationReady=true;
        SourceSlot=Stage=CompletedJobs=0; Fault.Empty(); Speed=StageTime=SettleTime=0;
        Head=Local(Slots[0]); Head.Z=SafeZ; JobStartHead=Head; StageHoldTarget=Head; UpdateParts();
        for (const auto& Pad:Pads) { Pad->SetCollisionEnabled(ECollisionEnabled::NoCollision); Pad->SetVisibility(false); }
        return;
    }
    bJobActive=true;
    CargoActor->DetachFromActor(FDetachmentTransformRules::KeepWorldTransform);
    auto* Body=CargoActor->GetBody();
    Body->SetSimulatePhysics(false);
    CargoActor->SetActorLocationAndRotation(Slots[0],Orientation.Rotator(),false,nullptr,ETeleportType::TeleportPhysics);
    Body->SetSimulatePhysics(true);
    Body->SetPhysicsLinearVelocity(FVector::ZeroVector);
    Body->SetPhysicsAngularVelocityInDegrees(FVector::ZeroVector);
    CargoActor->LocationOwner=bSTS?ECargoOwner::Ship:ECargoOwner::Yard;
    SourceSlot=Stage=CompletedJobs=0;
    bCarrying=bPaused=bResumePhysics=false;
    Fault.Empty(); Speed=StageTime=0; SettleTime=0;
    Head=Local(Slots[0]); Head.Z=SafeZ; StageHoldTarget=Head;
    UpdateParts();
}

void APortWorkingCrane::SetOperationPaused(bool Paused)
{
    if (!bConfigured || bPaused==Paused) return;
    bPaused=Paused;
    if (!IsValid(CargoActor)) return;
    auto* Body=CargoActor->GetBody();
    if (Paused)
    {
        bResumePhysics=Body->IsSimulatingPhysics();
        PausedVelocity=Body->GetPhysicsLinearVelocity();
        PausedAngularVelocity=Body->GetPhysicsAngularVelocityInDegrees();
        if (bResumePhysics) Body->SetSimulatePhysics(false);
    }
    else if (bResumePhysics)
    {
        Body->SetSimulatePhysics(true);
        Body->SetPhysicsLinearVelocity(PausedVelocity);
        Body->SetPhysicsAngularVelocityInDegrees(PausedAngularVelocity);
        bResumePhysics=false;
    }
}

void APortWorkingCrane::UpdateParts()
{
    bPartsDirty=true;
    if(!bDeferParts) FlushParts();
}

void APortWorkingCrane::FlushParts()
{
    if(!bPartsDirty) return;
    bPartsDirty=false;
    PortSimPerformance::FScope Timing(TEXT("crane_parts"));
    SetActorLocation(Home+Orientation.RotateVector(FVector(0,Head.Y,0)));
    Trolley->SetRelativeLocation(FVector(Head.X,0,BeamZ));
    Spreader->SetRelativeLocationAndRotation(FVector(Head.X,0,Head.Z)+SuspendedOffset,
        FRotator(0,FMath::RadiansToDegrees(SuspensionState.Yaw),0));
    // Telescope geometry is refreshed by AdvanceTelescope only when its length
    // changes. Moving the parent already carries all sensors and lock corners.
    bRopeVisualsDirty=true;
    if(!bDeferRopeVisuals) UpdateRopeVisuals();
}

void APortWorkingCrane::DeferRopeVisuals(bool Defer)
{
    bDeferRopeVisuals=Defer;
    if(!Defer && bRopeVisualsDirty) UpdateRopeVisuals();
}

void APortWorkingCrane::UpdateRopeVisuals()
{
    bRopeVisualsDirty=false;
    const float HalfLength=PortSpreaderTelescope::TwistlockHalfLengthCm(TelescopeLengthCm);
    const float HalfWidth=PortSpreaderTelescope::TwistlockHalfWidthCm();
    for (int32 I=0;I<4;++I)
    {
        const FVector Corner((I&1)?HalfWidth:-HalfWidth,(I&2)?HalfLength:-HalfLength,0);
        const FVector Top=FVector(Head.X,0,BeamZ)+Corner;
        const FVector Bottom=FVector(Head.X,0,Head.Z)+SuspendedOffset+Spreader->GetRelativeRotation().RotateVector(Corner);
        const FVector Delta=Top-Bottom;
        Ropes[I]->SetRelativeLocation((Top+Bottom)*.5f);
        Ropes[I]->SetRelativeRotation(FRotationMatrix::MakeFromZ(Delta).Rotator());
        Ropes[I]->SetRelativeScale3D(FVector(.06f,.06f,FMath::Max(1.,Delta.Size())/100.));
    }
}

bool APortWorkingCrane::MoveHead(FVector Target,float Dt)
{
    return MoveSTS(Target,Dt);
}

bool APortWorkingCrane::DestinationClear() const
{
    const FVector Extent(120,IsValid(CargoActor)?CargoActor->ExternalDimensionsCm.Y*.5f-2.f:608.f,127.5);
    SynchronizeShipQuery(FBox(-Extent,Extent).TransformBy(FTransform(Orientation,Slots[1-SourceSlot])));
    FCollisionQueryParams Query(SCENE_QUERY_STAT(WorkingCraneSlot),false);
    Query.AddIgnoredActor(CargoActor);
    // Shrink by 2 cm so the supporting pad/ground is not treated as an obstruction.
    return !GetWorld()->OverlapBlockingTestByChannel(Slots[1-SourceSlot],Orientation,ECC_WorldDynamic,
        FCollisionShape::MakeBox(FVector(120,IsValid(CargoActor)?CargoActor->ExternalDimensionsCm.Y*.5f-2.f:608.f,127.5)),Query);
}

void APortWorkingCrane::Stop(const FString& Reason,EPortCraneFault Kind)
{
    Fault=Reason; FaultType=Kind; SetOperationPaused(true);
    if(Kind==EPortCraneFault::SensorUnavailable || Kind==EPortCraneFault::GantryUnavailable)
    { UE_LOG(LogTemp,Warning,TEXT("EQUIPMENT_SENSOR_FAULT: crane=%d %s"),CraneID,*Reason); }
    else { UE_LOG(LogTemp,Error,TEXT("WORKING_CRANE_%d: %s"),CraneID,*Reason); }
}

void APortWorkingCrane::SetSensorFailure(bool Failed)
{
    if(!Failed && bFaultEvacuating && !AxisVelocity.IsNearlyZero(1.))
    {
        bAutoSensorRecovery=true;
        SensorRepairElapsed=FMath::Max(0.,SensorRepairMethod==EPortSensorRepairMethod::Reboot?SensorRebootSeconds:SensorTechnicianRepairSeconds)+FMath::Max(0.,SensorSelfCheckSeconds);
        SensorRepairPhase=TEXT("awaiting_safe_stop"); return;
    }
    const bool NewFailure=Failed && !bAppliedSensorFault;
    bAppliedSensorFault=bSensorFault=Failed; bFatalSensorTest=false;
    if(Failed)
    {
        if(NewFailure) { SensorRepairElapsed=0; bFaultEvacuationComplete=false; AxisVelocity=FVector::ZeroVector; }
        Observation.bValid=false;
        if(Fault.IsEmpty() || IsSensorUnavailable()) Stop(TEXT("Injected sensor unavailable"),EPortCraneFault::SensorUnavailable);
    }
    else if(IsSensorUnavailable())
    {
        bFaultEvacuating=false; AxisVelocity=FVector::ZeroVector; SensorRepairPhase=TEXT("repaired");
        Fault.Empty(); FaultType=EPortCraneFault::None; RecoveryStatus=TEXT("repaired"); NextSample=0;
        UE_LOG(LogTemp,Display,TEXT("EQUIPMENT_SENSOR_REPAIRED: crane=%d retained_load=%d"),CraneID,RetainsLoad());
    }
}

bool APortWorkingCrane::RetainsLoad() const
{
    if(bCarrying || (IsValid(CargoActor) && CargoActor->GetAttachParentActor()==this)) return true;
    for(bool Locked:CornerLocked) if(Locked) return true;
    return false;
}

bool APortWorkingCrane::CancelUnliftedJob()
{
    if(!bExternalJobs || RetainsLoad() || (IsValid(CargoActor) &&
        CargoActor->LocationOwner!=ECargoOwner::Ship && CargoActor->LocationOwner!=ECargoOwner::AGV)) return false;
    // Do not detach, move, or fabricate support for the failed crane's cargo.
    ClearSTSState(); CargoActor=nullptr; HandoverAGV=nullptr; bJobActive=false;
    AxisVelocity=FVector::ZeroVector; Stage=0; StageTime=SettleTime=0;
    return true;
}

bool APortWorkingCrane::CanReach(FVector Source,FVector Destination) const
{
    return STSProfile.bReady && STSProfile.ContainsTarget(Local(Source)+FVector(0,0,WorkingCrane::LiftOffset)) &&
        STSProfile.ContainsTarget(Local(Destination)+FVector(0,0,WorkingCrane::LiftOffset));
}
float APortWorkingCrane::GantryWorldY() const { return (Home+Orientation.RotateVector(FVector(0,Head.Y,0))).Y; }
FVector2D APortWorkingCrane::RailWorkInterval() const
{
    float Low=GantryWorldY(),High=Low;
    if(bFaultEvacuating) { Low=FMath::Min(Low,FaultEvacuationTargetY); High=FMath::Max(High,FaultEvacuationTargetY); }
    if(bJobActive && !IsEquipmentUnavailable()) for(const FVector& Slot:Slots)
    { Low=FMath::Min(Low,float(Slot.Y)); High=FMath::Max(High,float(Slot.Y)); }
    return FVector2D(Low,High);
}

void APortWorkingCrane::Advance(float Dt,bool bGlobalPaused)
{
    if(bAppliedSensorFault!=bSensorFault) SetSensorFailure(bSensorFault);
    if(bAppliedGantryFault!=bGantryFault) SetGantryFailure(bGantryFault);
    PortSimPerformance::FScope Profile(TEXT("crane_advance"));
    if(!FMath::IsFinite(Dt) || Dt<=0) { if(Dt==0)AdvanceStep(0,bGlobalPaused);return; }
    if(!bJobActive) { AdvanceStep(Dt,bGlobalPaused); return; }
    // A render frame can cover a large simulation interval at low FPS/high speed.
    // Substep observations and the state machine too, not just the physical plant.
    const int32 Steps=CraneSimulationSteps::Count(Dt,60.);
    bDeferParts=true;
    ON_SCOPE_EXIT { bDeferParts=false; FlushParts(); };
    const float Step=Dt/Steps;
    for(int32 I=0;I<Steps;++I)
    {
        ShipControlFraction=double(I+1)/Steps;
        // Pickup can complete inside this outer frame. Re-evaluate ownership
        // every substep so a newly attached load is never moved back onto the
        // vessel by the remaining frame interpolation.
        if(IsValid(CargoActor) && CargoActor->LocationOwner==ECargoOwner::Ship &&
            !CargoActor->GetAttachParentActor() && Stage<=2)
        {
            CargoActor->ApplySecuredMotionFraction(double(I+1)/Steps);
            UpdateMovingShipSource(CargoActor,CargoActor->GetActorLocation());
        }
        AdvanceStep(Step,bGlobalPaused);
    }
    ShipControlFraction=1;
}

void APortWorkingCrane::SynchronizeShipQuery(const FBox& Query) const
{
    // Refresh the scene cache before queries; controller state remains at its
    // original 60/120 Hz steps even when there is no sensor sample this step.
    const_cast<APortWorkingCrane*>(this)->FlushParts();
    if(IsValid(CargoActor)) if(auto* Site=Cast<APortSiteLogistics>(CargoActor->GetOwner()))
        Site->SynchronizeShipCollision(Query,ShipControlFraction);
}

void APortWorkingCrane::AdvanceStep(float Dt,bool bGlobalPaused)
{
    if (!bConfigured) return;
    SimulationTime+=Dt;
    AdvanceFailureRecovery(Dt,bGlobalPaused);
    if(bJobActive) { JobSeconds+=Dt; if(bGlobalPaused || !bEnabled || !Fault.IsEmpty()) PausedSeconds+=Dt; }
    SetOperationPaused(bGlobalPaused || !bEnabled || !Fault.IsEmpty());
    if (bPaused || !bJobActive) return;
    AdvanceTelescope(Dt);
    {
        SampleSTS();
        if(!STSProfile.bReady || !Observation.IsFresh(SimulationTime,STSProfile.SensorMaxAge))
        {
            Stop(FString::Printf(TEXT("Required %s sensor observation invalid/stale: %s (age=%.4fs limit=%.4fs)"),
                bSTS?TEXT("STS"):TEXT("RMG"),SensorInvalidReason.IsEmpty()?TEXT("Freshness/profile check"):*SensorInvalidReason,
                SimulationTime-Observation.Timestamp,double(STSProfile.SensorMaxAge)),
                STSProfile.bReady && !bFatalSensorTest?EPortCraneFault::SensorUnavailable:EPortCraneFault::Safety);
            return;
        }
        if(bCarrying && (!Observation.AllLocked() || Observation.DynamicPayloadEstimateKg()>STSProfile.RatedPayloadKg))
        { Stop(TEXT("Loaded hoist interlock: locks/load observation invalid")); return; }
    }
    const bool bSTSNeedsAGV=AGVAtDestination() && ((Stage==5 && bDestinationReady) || Stage==6 || Stage==7);
    const bool bRMGNeedsAGV=!AGVAtDestination() && Stage<=2 && IsValid(HandoverAGV);
    if((bSTSNeedsAGV || bRMGNeedsAGV) && !Observation.bAGVAligned)
    {
        // A newly arrived vehicle can be visible one sensor period after the
        // dispatch state changes.  Keep the suspended load at its current safe
        // pose and reacquire alignment instead of turning one transient sample
        // into a terminal-wide emergency stop.
        AGVAlignmentWait+=Dt;
        SettleTime=0;
        if(AGVAlignmentWait>STSProfile.Pickup.AlignmentTimeout)
        {
            const bool bValidAGV=IsValid(HandoverAGV);
            const FVector AGVCargo=bValidAGV?HandoverAGV->CargoPosition():FVector::ZeroVector;
            const float AGVSpeed=bValidAGV?HandoverAGV->Speed:-1.f;
            const float PositionError=bValidAGV?FVector::Distance(AGVCargo,Slots[1-SourceSlot]):-1.f;
            const float HeadingError=bValidAGV?FMath::Abs(FMath::FindDeltaAngleDegrees(HandoverAGV->GetActorRotation().Yaw,Orientation.Rotator().Yaw)):-1.f;
            const bool bInLidar=bValidAGV&&STSSensorContains(TEXT("agv_position_lidar"),AGVCargo);
            UE_LOG(LogTemp,Error,TEXT("STS_ALIGNMENT_DETAIL: crane=%d stage=%d ready=%d agv=%s observed=%d speed_cmps=%.3f position_error_cm=%.3f position_limit_cm=%.3f heading_error_deg=%.3f heading_limit_deg=%.3f lidar=%d destination=%s cargo_position=%s"),
                CraneID,Stage,bDestinationReady,*GetNameSafe(HandoverAGV),Observation.bAGVAligned,AGVSpeed,PositionError,STSProfile.AGVTolerance,
                HeadingError,STSProfile.AGVHeadingTolerance,bInLidar,*Slots[1-SourceSlot].ToString(),*AGVCargo.ToString());
            UE_LOG(LogTemp,Error,TEXT("CRANE_ALIGNMENT_TIMEOUT: crane=%d kind=%s stage=%d ready=%d agv=%s observed=%d destination=%s wait=%.2f"),
                CraneID,bSTS?TEXT("STS"):TEXT("RMG"),Stage,bDestinationReady,*GetNameSafe(HandoverAGV),
                Observation.bAGVAligned,*Slots[bSTS?1-SourceSlot:SourceSlot].ToString(),AGVAlignmentWait);
            Stop(bSTS?TEXT("AGV alignment recovery timed out during STS handover"):
                TEXT("AGV alignment recovery timed out during RMG pickup"));
            return;
        }
        MoveSTS(Head,Dt);
        return;
    }
    AGVAlignmentWait=0;
    if(!bSTS)
    {
        if(!Observation.bCraneClear) {Stop(TEXT("RMG crane collision clearance violated"));return;}
        if(Stage==5 && !Observation.bStackProfileValid) {Stop(TEXT("RMG stack profile invalid or destination height mismatch"));return;}
    }
    // Pick and lift the next ship box while the vehicle is away; hold it safely
    // above the dock until the returning AGV is stopped beneath the spreader.
    if (Stage==5 && !bDestinationReady) { MoveSTS(StageHoldTarget,Dt); return; }
    if(!bRailSafetyHold) StageTime+=Dt;
    if (StageTime>STSProfile.StageTimeout) { Stop(TEXT("Job stage timed out")); return; }
    const FVector Source=Stage==3?Local(Pickup.TrialOrigin):Local(Slots[SourceSlot]);
    const FVector Destination=Local(Slots[1-SourceSlot]);
    if(Stage==2)
    {
        if(!TelescopeReady())
        {
            Pickup.Reason=TEXT("Adjusting telescopic spreader to container ISO corner positions");
            MoveSTS(FVector(Source.X,Source.Y,SafeZ),Dt);
            return;
        }
        AdvancePickup(Dt);return;
    }
    FVector Target=StageHoldTarget;
    switch(Stage)
    {
    case 0: Target.Z=SafeZ; break;
    case 1: Target=FVector(Source.X,Source.Y,SafeZ); break;
    case 2: Target=Source+FVector(0,0,WorkingCrane::LiftOffset); break;
    case 3: Target=FVector(Source.X,Source.Y,SafeZ); break;
    case 4: Target=FVector(Destination.X,Destination.Y,SafeZ); break;
    case 5:
        // Pickup retains the achieved coupling offset. Compensate it from
        // measurements so the cargo, rather than an ideal head centre, lands on the AGV.
        Target=Destination+Orientation.UnrotateVector(Observation.SpreaderPosition-Observation.CargoPosition);
        break;
    case 6: Target=FVector(Destination.X,Destination.Y,SafeZ); break;
    case 7:
        SettleTime+=Dt;
        if (SettleTime<1.5f) return;
        if (FVector::Dist(CargoActor->GetActorLocation(),Slots[1-SourceSlot])>10.f ||
            CargoActor->GetBody()->GetPhysicsLinearVelocity().Size()>5.f || CargoActor->GetActorUpVector().Z<.99f)
        {
            UE_LOG(LogTemp,Error,TEXT("CRANE_SETTLING_DETAIL: crane=%d cargo=%s position=%s destination=%s error_cm=%.3f speed_cmps=%.3f up_z=%.6f physics=%d"),
                CraneID,*GetNameSafe(CargoActor),*CargoActor->GetActorLocation().ToString(),
                *Slots[1-SourceSlot].ToString(),FVector::Dist(CargoActor->GetActorLocation(),Slots[1-SourceSlot]),
                CargoActor->GetBody()->GetPhysicsLinearVelocity().Size(),CargoActor->GetActorUpVector().Z,
                CargoActor->GetBody()->IsSimulatingPhysics());
            Stop(TEXT("Placed cargo failed physical settling check")); return;
        }
        ++CompletedJobs;
        LastJobSeconds=JobSeconds; LastPausedSeconds=PausedSeconds;
        if (bExternalJobs) { bJobActive=false; CargoActor=nullptr; Stage=0; StageTime=SettleTime=0; return; }
        SourceSlot=1-SourceSlot; Stage=0; StageTime=SettleTime=0;
        {Pickup=FSTSPickupController();for(double& Progress:LockProgress)Progress=0;}
        return;
    default: Stop(TEXT("Invalid job stage")); return;
    }
    if (!MoveHead(Target,Dt)) return;
    if (Stage==5)
    {
        {
            if(!Observation.bCargoSupported || Observation.SpreaderVelocity.Size()>STSProfile.SettleSpeed ||
                !Observation.CargoPosition.Equals(Slots[1-SourceSlot],STSProfile.LandingTolerance)) { SettleTime=0; return; }
            SettleTime+=Dt;
            if(SettleTime<STSProfile.SettleTime) return;
            for(bool& Locked:CornerLocked) Locked=false;
            for(bool& Requested:Pickup.RequestLocks) Requested=false;
        }
        if (!DestinationClear()) { Stop(TEXT("Destination became occupied")); return; }
        CargoActor->DetachFromActor(FDetachmentTransformRules::KeepWorldTransform);
        auto* Body=CargoActor->GetBody();
        // A controller frame can contain many kinematic pose updates before
        // Chaos runs. Commit the measured release pose on both sides of the
        // state change; otherwise an earlier deferred target can reappear as
        // the load becomes dynamic, even though the landing sensors passed.
        const FTransform ReleasePose=CargoActor->GetActorTransform();
        Body->BodyInstance.SetBodyTransform(ReleasePose,ETeleportType::TeleportPhysics,false);
        Body->SetSimulatePhysics(true);
        Body->BodyInstance.SetBodyTransform(ReleasePose,ETeleportType::TeleportPhysics,false);
        Body->SetPhysicsLinearVelocity(FVector::ZeroVector);
        Body->SetPhysicsAngularVelocityInDegrees(FVector::ZeroVector);
        CargoActor->LocationOwner=bSTS && (bReverseTransfer || (1-SourceSlot)==0)?ECargoOwner::Ship:ECargoOwner::Yard;
        bCarrying=false;
    }
    StageHoldTarget=Head+SuspendedOffset;
    ++Stage; StageTime=SettleTime=0;
    SampleSTS(true);
}

bool APortWorkingCrane::ValidateOperation(FString& Error) const
{
    // An empty yard is a valid idle state only when the crane profile loaded.
    // ResetOperation clears transient faults, but cannot repair invalid settings.
    if (bConfigured && !STSProfile.bReady)
    { Error=FString::Printf(TEXT("Crane %d: profile unavailable: %s"),CraneID,*STSProfile.Error); return false; }
    if(bConfigured && bExternalJobs && IsEquipmentUnavailable())
    {
        if(bCarrying && (!IsValid(CargoActor) || CargoActor->GetAttachParentActor()!=this || CargoActor->GetBody()->IsSimulatingPhysics()))
        { Error=TEXT("Isolated crane lost its retained load"); return false; }
        return true;
    }
    if (bConfigured && bExternalJobs && !bJobActive && Fault.IsEmpty()) return true;
    if (!bConfigured || !IsValid(CargoActor) || (!bExternalJobs && CargoActor->GetOwner()!=this) || !Fault.IsEmpty())
    { Error=FString::Printf(TEXT("Crane %d: configuration/cargo/fault: %s"),CraneID,*Fault); return false; }
    const FVector A=Local(Slots[0]), B=Local(Slots[1]);
    // The anti-sway controller moves the trolley against the pendulum. Validate
    // the suspended working point against the reserved cargo corridor; checking
    // the trolley alone rejects that corrective motion even while the spreader
    // remains inside the reservation.
    const FVector ControlledPosition=Head+SuspendedOffset;
    double Margin=STSProfile.Pickup.AcquisitionRadius;
    double VerticalMargin=1.;
    if(STSProfile.bReady)
    {
        FSTSDynamicsParameters WindConfig=STSProfile.Dynamics;
        WindConfig.Wind=Orientation.UnrotateVector(WindVelocityMetersPerSecond);
        const FVector WindAllowance=WindConfig.WindEquilibriumOffset(STSProfile.MaxRope,
            FMath::Max(1.f,STSProfile.SpreaderMassKg))*100.;
        Margin+=FMath::Max(FMath::Abs(WindAllowance.X),FMath::Abs(WindAllowance.Y));
        // A loaded trolley can enter the end of the reserved segment at full
        // reference speed. Include its bounded braking distance as well as the
        // wind deflection instead of treating controlled deceleration as escape.
        const double TrolleyStopping=FMath::Square(double(STSProfile.TrolleySpeed))/(2.*FMath::Max(.001,RailAccelerationLimit(STSProfile.TrolleyAcceleration)));
        const double GantryStopping=FMath::Square(double(STSProfile.GantrySpeed))/(2.*FMath::Max(.001,RailAccelerationLimit(STSProfile.GantryAcceleration)));
        Margin+=FMath::Max(TrolleyStopping,GantryStopping);
        // SafeZ is a transfer target rather than a hard mechanical travel stop.
        // A hoist that reaches it with upward velocity must decelerate before it
        // can descend toward the next target. Reserve the same physically bounded
        // stopping distance used for the horizontal axes so this normal transient
        // is not reported as a terminal-wide failure at accelerated playback.
        VerticalMargin+=FMath::Square(double(STSProfile.EmptyHoistSpeed))/(2.*STSProfile.HoistAcceleration);
    }
    if (ControlledPosition.X<FMath::Min3(A.X,B.X,JobStartHead.X)-Margin || ControlledPosition.X>FMath::Max3(A.X,B.X,JobStartHead.X)+Margin ||
        ControlledPosition.Y<FMath::Min3(A.Y,B.Y,JobStartHead.Y)-Margin || ControlledPosition.Y>FMath::Max3(A.Y,B.Y,JobStartHead.Y)+Margin || Head.Z>SafeZ+VerticalMargin)
    {
        Error=FString::Printf(TEXT("Crane left its reserved envelope: stage=%d head=%s suspended=%s source=%s destination=%s start=%s margin=%.2f vertical_margin=%.2f"),
            Stage,*Head.ToCompactString(),*ControlledPosition.ToCompactString(),*A.ToCompactString(),*B.ToCompactString(),*JobStartHead.ToCompactString(),Margin,VerticalMargin);
        return false;
    }
    if (bCarrying && (CargoActor->GetAttachParentActor()!=this || CargoActor->GetBody()->IsSimulatingPhysics() ||
        !CargoActor->GetActorTransform().Equals(LockedCargoTransform*Spreader->GetComponentTransform(),1.f)))
    { Error=TEXT("Locked cargo is detached or out of alignment"); return false; }
    if (!bCarrying && CargoActor->GetAttachParentActor() &&
        !(Stage<=2 && CargoActor->LocationOwner==ECargoOwner::AGV && !CargoActor->GetBody()->IsSimulatingPhysics()))
    { Error=TEXT("Released cargo remains attached"); return false; }
    return true;
}

void APortWorkingCrane::EndPlay(const EEndPlayReason::Type Reason)
{
    if (!bExternalJobs && IsValid(CargoActor)) CargoActor->Destroy();
    CargoActor=nullptr;
    Super::EndPlay(Reason);
}

bool APortWorkingCrane::SetAGVLandingTarget(FVector Target)
{
    if(!bSTS || !bExternalJobs || !bJobActive || bDestinationReady || Stage>5 ||
        !STSProfile.ContainsTarget(Local(Target)+FVector(0,0,WorkingCrane::LiftOffset))) return false;
    // STS stage 5 can be waiting at travel height with descent interlocked.
    // Bind the deck datum there too, while the load is safely above the pad.
    if(Stage==5 && Head.Z<Local(Target).Z+WorkingCrane::LiftOffset+100.f) return false;
    Slots[1-SourceSlot]=Target;
    Pads[1-SourceSlot]->SetWorldLocation(Target-FVector(0,0,139.5f),false,nullptr,ETeleportType::TeleportPhysics);
    SampleSTS(true);
    return true;
}

bool APortWorkingCrane::IsWaitingForAGV() const
{
    return bSTS && bJobActive && !bDestinationReady &&
        (Stage==5 || (Stage==4 && AxisVelocity.Size2D()<=1.f &&
            (Local(Slots[1-SourceSlot])-(Head+SuspendedOffset)).Size2D()<=STSProfile.LandingTolerance));
}

bool APortWorkingCrane::AssignCargo(APortContainerActor* Cargo,FVector Source,FVector Destination,bool SourceSupport,bool DestinationSupport,APortAGVActor* HandoverVehicle,bool ReverseTransfer)
{
    // Dispatch can choose an otherwise passive box in the middle of a frame.
    if(IsValid(Cargo) && Cargo->LocationOwner==ECargoOwner::Ship && !Cargo->GetAttachParentActor())
        if(const auto* Site=Cast<APortSiteLogistics>(Cargo->GetOwner()); Site && Site->IsDeferringVisuals())
        { Cargo->ApplySecuredMotionFraction(1.); Source=Cargo->GetActorLocation(); }
    if (!bExternalJobs || bJobActive || !IsValid(Cargo) || !Fault.IsEmpty()) return false;
    {
        if(!STSProfile.bReady) { Stop(TEXT("Crane profile unavailable")); return false; }
        if(!FMath::IsFinite(Cargo->MassKg) || Cargo->MassKg<=0 || Cargo->MassKg>STSProfile.RatedPayloadKg)
        { Stop(bSTS?TEXT("Payload exceeds resolved STS reference capacity"):TEXT("Payload exceeds resolved RMG reference capacity")); return false; }
        if(!STSProfile.ContainsTarget(Local(Source)+FVector(0,0,WorkingCrane::LiftOffset)) ||
            !STSProfile.ContainsTarget(Local(Destination)+FVector(0,0,WorkingCrane::LiftOffset)))
        { Stop(TEXT("Source/destination outside reference crane working envelope")); return false; }
    }
    ClearSTSState(); HandoverAGV=HandoverVehicle; bReverseTransfer=ReverseTransfer;
    CargoActor=Cargo; Slots[0]=Source; Slots[1]=Destination; SourceSlot=Stage=0;
    TelescopeTargetLengthCm=Cargo->ExternalDimensionsCm.Y;
    bDestinationReady=true;
    bJobActive=true; bCarrying=false; Speed=StageTime=SettleTime=0; JobStartHead=Head;
    StageHoldTarget=Head+SuspendedOffset;
    for (int32 I=0;I<2;++I)
    {
        const bool Support=I==0?SourceSupport:DestinationSupport;
        // This is a fixed support for the newly reserved slot, not a platform
        // travelling from the last job's tier. Do not give Chaos an interpolated
        // kinematic target (or a fictitious support velocity) between jobs.
        Pads[I]->SetWorldLocationAndRotation(Slots[I]-FVector(0,0,139.5f),Orientation,false,nullptr,ETeleportType::TeleportPhysics);
        Pads[I]->SetWorldScale3D(FVector(Cargo->ExternalDimensionsCm.X+66.f,Cargo->ExternalDimensionsCm.Y+80.f,20.f)/100.f);
        Pads[I]->SetCollisionEnabled(Support?ECollisionEnabled::QueryAndPhysics:ECollisionEnabled::NoCollision);
        Pads[I]->SetVisibility(Support);
    }
    SampleSTS(true);
    return true;
}
