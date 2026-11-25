#include "PortWorkingCrane.h"
#include "SimulationPerformance.h"
#include "CraneTravelBraking.h"
#include "HoistDriveLimits.h"
#include "PortContainerActor.h"
#include "PortAGVActor.h"
#include "PortSiteLogistics.h"
#include "AGVReference.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/World.h"
#include "Engine/StaticMesh.h"
#include "SpreaderTelescope.h"

// Site STS suspension integrates sway/yaw; sensor observations remain synthetic.
void APortWorkingCrane::SetHandoverVehicle(APortAGVActor* Vehicle)
{
    // Fleet dispatch may bind an AGV after prefetch starts, before descent is enabled.
    if(!bSTS || !bExternalJobs || !bJobActive || bDestinationReady) return;
    HandoverAGV=Vehicle;
    SampleSTS(true);
}

void APortWorkingCrane::SetDestinationReady(bool Ready)
{
    if(bDestinationReady==Ready) return;
    bDestinationReady=Ready;
    AGVAlignmentWait=0;
    // Fleet movement is updated after cranes.  Refresh immediately when the
    // dispatcher reports arrival so the first handover step cannot consume the
    // previous sensor sample from while the AGV was still approaching.
    if(bSTS && bJobActive) SampleSTS(true);
}

APortAGVActor* APortWorkingCrane::GetHandoverVehicle() const { return HandoverAGV.Get(); }

bool APortWorkingCrane::IsProductivelyWorking() const
{
    if(!bConfigured||!bJobActive||bPaused||!bEnabled||!Fault.IsEmpty()||bRailSafetyHold||bFaultEvacuating||IsWaitingForAGV())return false;
    if(IncidentFaults.Contains(TEXT("fail_hoist"))||
        (IncidentFaults.Contains(TEXT("agv_unavailable"))&&IncidentHandoverWindow()))return false;
    if(Stage==2&&(bIncidentPickupFailure||!Observation.bTargetVisible))return false;
    if(Stage==4&&!Observation.bCraneClear)return false;
    if(Stage==5&&(!bDestinationReady||bIncidentPlacementFailure||(IncidentFaults.Contains(TEXT("fail_release"))&&IncidentReleaseWindow())||
        IncidentFaults.Contains(TEXT("slot_occupied"))||IncidentFaults.Contains(TEXT("height_mismatch"))))return false;
    if((AGVAtDestination()&&Stage>=5&&Stage<=7&&!bAGVHandoverComplete) ||
        (!AGVAtDestination()&&Stage<=2&&IsValid(HandoverAGV)))
        if(!Observation.bAGVAligned)return false;
    return bSTS||Observation.bCraneClear;
}

void APortWorkingCrane::SetWindVelocity(FVector WorldVelocityMetersPerSecond)
{
    WindVelocityMetersPerSecond=WorldVelocityMetersPerSecond.ContainsNaN()?FVector::ZeroVector:
        WorldVelocityMetersPerSecond.GetClampedToMaxSize(50.f);
}

void APortWorkingCrane::SetFogVisibilityMeters(float VisibilityMeters)
{
    FogVisibilityMeters=FMath::IsFinite(VisibilityMeters)?FMath::Max(0.f,VisibilityMeters):MAX_flt;
}

void APortWorkingCrane::SetRainSensorRangeScales(float CameraScale,float LiDARScale)
{
    RainCameraRangeScale=FMath::Clamp(CameraScale,0.f,1.f);
    RainLiDARRangeScale=FMath::Clamp(LiDARScale,0.f,1.f);
}

FVector APortWorkingCrane::WindProjectedAreasM2() const
{
    if(bCarrying && IsValid(CargoActor))
        return PortSpreaderTelescope::LoadedProjectedAreasM2(TelescopeLengthCm,CargoActor->ExternalDimensionsCm);
    return PortSpreaderTelescope::EmptyProjectedAreasM2(TelescopeLengthCm);
}

void APortWorkingCrane::UpdateMovingShipSource(APortContainerActor* Cargo,FVector WorldPosition)
{
    if(!bSTS || !bJobActive || bCarrying || CargoActor!=Cargo || Stage>2 || SourceSlot<0 || SourceSlot>1) return;
    Slots[SourceSlot]=WorldPosition;
    if(Pads.IsValidIndex(SourceSlot)) Pads[SourceSlot]->SetWorldLocation(WorldPosition-FVector(0,0,139.5f));
}

void APortWorkingCrane::ClearSTSState()
{
    bRMGApproach=bRMGApproachRaised=false;
    bReverseTransfer=false;
    bAGVHandoverComplete=false;
    AxisVelocity=PlantAcceleration=FVector::ZeroVector;
    Pickup=FSTSPickupController();
    PickupRecoveryCount=PickupRecoveryStep=0; PickupRecoverySeconds=PickupRecoveryStableSeconds=0;
    PickupRecoveryPhase.Empty(); PickupRecoveryReason.Empty(); bIsolateAfterPickupRetreat=false;
    for(int32 I=0;I<4;++I){LockProgress[I]=0;PhysicalSeating[I]=false;}
    Observation=FSTSObservation(); NextSample=0;
    JobSeconds=PausedSeconds=LastJobSeconds=LastPausedSeconds=0;
    for(double& Seconds:JobStageSeconds) Seconds=0;
    for(double& Seconds:LastJobStageSeconds) Seconds=0;
    JobAGVWaitSeconds=0;
    AGVAlignmentWait=0;
    HandoverAGV=nullptr;
    for(bool& Locked:CornerLocked) Locked=false;
}

bool APortWorkingCrane::TelescopeReady() const
{
    return PortSpreaderTelescope::IsReady(TelescopeLengthCm,TelescopeTargetLengthCm);
}

void APortWorkingCrane::AdvanceTelescope(float Dt)
{
    // Retarget only with an empty spreader and finish before seating. This lets
    // the crane telescope during its high-level approach without moving the
    // end beams while locks or a payload are engaged.
    if(bCarrying || Stage>2 || Dt<=0) return;
    const float Next=PortSpreaderTelescope::Advance(TelescopeLengthCm,TelescopeTargetLengthCm,Dt);
    if(FMath::IsNearlyEqual(Next,TelescopeLengthCm)) return;
    TelescopeLengthCm=Next;
    if(TelescopeReady()) TelescopeLengthCm=TelescopeTargetLengthCm;
    UpdateSpreaderGeometry();
}

FVector APortWorkingCrane::DynamicSpreaderMount(const FSTSSensorMount& Mount,FVector ConfiguredPosition) const
{
    if(Mount.Frame!=TEXT("spreader")) return ConfiguredPosition;
    const float HalfLengthM=PortSpreaderTelescope::TwistlockHalfLengthCm(TelescopeLengthCm)*.01f;
    const float HalfWidthM=PortSpreaderTelescope::TwistlockHalfWidthCm()*.01f;
    if(Mount.Key==TEXT("twistlock_load") || Mount.Key==TEXT("twistlock_state") || Mount.Key==TEXT("landed"))
    {
        ConfiguredPosition.X=FMath::Sign(ConfiguredPosition.X)*HalfWidthM;
        ConfiguredPosition.Y=FMath::Sign(ConfiguredPosition.Y)*HalfLengthM;
    }
    else
    {
        // Corner/side cameras follow the end beams. Centre-mounted devices
        // such as TTDS and the telescope encoder stay at zero.
        if(FMath::Abs(ConfiguredPosition.X)>.1f) ConfiguredPosition.X*=HalfWidthM;
        if(FMath::Abs(ConfiguredPosition.Y)>.1f) ConfiguredPosition.Y*=HalfLengthM/5.2f;
    }
    return ConfiguredPosition;
}

void APortWorkingCrane::UpdateSpreaderSensors()
{
    for(int32 I=0;I<SpreaderSensorInstances.Num();++I)
    {
        if(!SpreaderSensorInstances[I] || !STSProfile.Dynamics.Mounts.IsValidIndex(SpreaderSensorMountIndices[I])) continue;
        const auto& Mount=STSProfile.Dynamics.Mounts[SpreaderSensorMountIndices[I]];
        SpreaderSensorInstances[I]->SetRelativeLocation(DynamicSpreaderMount(Mount,SpreaderSensorConfiguredPositions[I])*100.f);
    }
}

void APortWorkingCrane::UpdateSpreaderGeometry()
{
    if(!SpreaderCenter) return;
    const float EndY=PortSpreaderTelescope::TwistlockHalfLengthCm(TelescopeLengthCm);
    const float HalfWidth=PortSpreaderTelescope::TwistlockHalfWidthCm();
    constexpr float CentreHalfLength=PortSpreaderTelescope::CentreLengthCm*.5f;
    const float ArmLength=FMath::Max(40.f,EndY-CentreHalfLength);
    SpreaderCenter->SetRelativeLocation(FVector::ZeroVector);
    SpreaderCenter->SetRelativeScale3D(FVector(PortSpreaderTelescope::WidthCm,
        PortSpreaderTelescope::CentreLengthCm,PortSpreaderTelescope::CentreHeightCm)/100.f);
    for(int32 End=0;End<2;++End)
    {
        const float Sign=End?1.f:-1.f;
        TelescopeArms[End]->SetRelativeLocation(FVector(0,Sign*(CentreHalfLength+ArmLength*.5f),0));
        TelescopeArms[End]->SetRelativeScale3D(FVector(PortSpreaderTelescope::ArmWidthCm,
            ArmLength,PortSpreaderTelescope::ArmHeightCm)/100.f);
        SpreaderEndBeams[End]->SetRelativeLocation(FVector(0,Sign*EndY,0));
    }
    for(int32 Corner=0;Corner<4;++Corner)
        TwistLocks[Corner]->SetRelativeLocation(FVector((Corner&1)?HalfWidth:-HalfWidth,
            (Corner&2)?EndY:-EndY,PortSpreaderTelescope::TwistlockCenterZCm));
    UpdateSpreaderSensors();
}

bool APortWorkingCrane::AGVAligned() const
{
    if(!bExternalJobs) return true;
    if(!IsValid(HandoverAGV)) return false;
    if(HandoverAGV->bDriveFault||HandoverAGV->bIncidentSensorFault||HandoverAGV->bIncidentHandoverHold) return false;
    // Fog and heavy rain may hide the gantry-mounted long-range LiDAR even
    // after the AGV has completed its final docking move.  The docking node's
    // buried transponder/FMS fix is independent of optical visibility and is a
    // valid redundant confirmation, but never bypasses the physical position,
    // speed or heading interlocks below.
    const bool bDockingFix=HandoverAGV->Sensors.bFMSConnected &&
        HandoverAGV->Sensors.bDockingNode && HandoverAGV->Sensors.bTransponderLocked;
    const bool bPositionConfirmed=(AGVAtDestination()?Stage<5:Stage<2) || bDockingFix ||
        STSSensorContains(TEXT("agv_position_lidar"),HandoverAGV->CargoPosition());
    return HandoverAGV->Speed<=0.1f &&
        HandoverAGV->CargoPosition().Equals(Slots[AGVAtDestination()?1-SourceSlot:SourceSlot],STSProfile.AGVTolerance) &&
        bPositionConfirmed &&
        AGVReference::AxialHeadingErrorDegrees(HandoverAGV->GetActorRotation().Yaw,Orientation.Rotator().Yaw)<=STSProfile.AGVHeadingTolerance;
}

bool APortWorkingCrane::CargoSupported() const
{
    if(!IsValid(CargoActor)) return false;
    FCollisionQueryParams Query(SCENE_QUERY_STAT(SiteSTSSupport),false);
    // RMG's fixed reservation pads are support surfaces owned by this actor.
    if(bSTS)Query.AddIgnoredActor(this);
    Query.AddIgnoredActor(CargoActor);
    const float HalfLength=CargoActor->SensorHalfLengthCm();
    const float HalfWidth=CargoActor->SensorHalfWidthCm();
    const FVector E=Orientation.RotateVector(FVector(HalfWidth,0,0)).GetAbs()+
        Orientation.RotateVector(FVector(0,HalfLength,0)).GetAbs();
    const FVector QueryCenter=CargoActor->GetActorLocation();
    SynchronizeShipQuery(FBox(QueryCenter-E-FVector(0,0,129.5f+STSProfile.SupportTolerance),QueryCenter+E));
    for(int32 I=0;I<4;++I)
    {
        const FVector P=CargoActor->GetActorLocation()+Orientation.RotateVector(FVector((I&1)?HalfWidth:-HalfWidth,(I&2)?HalfLength:-HalfLength,0));
        FHitResult Hit;
        if(!GetWorld()->LineTraceSingleByChannel(Hit,P,P-FVector(0,0,129.5f+STSProfile.SupportTolerance),ECC_Visibility,Query) || Hit.ImpactNormal.Z<.8f) return false;
        if(AGVAtDestination() && bExternalJobs && Stage==5 && Hit.GetActor()!=HandoverAGV) return false;
    }
    return true;
}

void APortWorkingCrane::UpdateShipLandingTarget(FVector Target)
{
    if(!bSTS || !bReverseTransfer || !bJobActive) return;
    Slots[1-SourceSlot]=Target;
}

void APortWorkingCrane::SampleSTS(bool Force)
{
    if(!STSProfile.bReady || (!IsValid(CargoActor) && !bRMGApproach) || (!Force && SimulationTime<NextSample)) return;
    FlushParts();
    PortSimPerformance::FScope Timing(TEXT("crane_sensors"));
    NextSample=SimulationTime+STSProfile.SensorPeriod;
    auto& S=Observation;
    const double SampleDt=SimulationTime-S.Timestamp;
    const bool SameSample=S.Timestamp>=0 && SampleDt<=1.e-6;
    const bool HasPrevious=S.Timestamp>=0 && SampleDt>1.e-6 && SampleDt<=STSProfile.SensorMaxAge;
    const FVector PreviousJerk=S.CargoJerk;
    const FVector2D PreviousAngles=S.CargoPitchRoll;
    S.Timestamp=SimulationTime; S.bValid=!bSensorFault;
    SensorInvalidReason=bSensorFault?TEXT("Injected sensor fault"):TEXT("");
    auto InvalidSensor=[&](const FString& Reason)
    { S.bValid=false; if(SensorInvalidReason.IsEmpty()) SensorInvalidReason=Reason; };
    S.DrivePosition=Head+FVector(STSProfile.PositionBias);
    S.DriveVelocity=AxisVelocity;
    S.WindVelocityMetersPerSecond=Orientation.UnrotateVector(WindVelocityMetersPerSecond);
    S.SpreaderPosition=HeadPosition()+Orientation.RotateVector(FVector(STSProfile.PositionBias));
    S.SpreaderVelocity=SpreaderVelocity();
    if(bRMGApproach && !bJobActive)
    {
        S.SwayDegrees=SuspensionState.SwayDegrees();
        S.SwayAngle=SuspensionState.Angle; S.SwayRate=SuspensionState.Rate;
        S.SkewDegrees=FMath::RadiansToDegrees(SuspensionState.Yaw);
        S.SpreaderPitchRoll=AttitudeState.Angle; S.SpreaderPitchRollRate=AttitudeState.Rate;
        S.HoistAcceleration=PlantAcceleration.Z;
        SampleRMGEnvironment();
    }
    else
    {
    S.CargoPosition=CargoActor->GetActorLocation()+Orientation.RotateVector(STSProfile.Pickup.PoseBias);
    S.CargoVelocity=bCarrying?S.SpreaderVelocity:CargoActor->GetMotionVelocity();
    S.CargoAcceleration=bCarrying?PlantAcceleration:CargoActor->GetMotionAcceleration();
    S.CargoJerk=bCarrying?FVector::ZeroVector:CargoActor->GetMotionJerk();
    S.CargoSnap=HasPrevious?(S.CargoJerk-PreviousJerk)/SampleDt:(SameSample?S.CargoSnap:FVector::ZeroVector);
    const auto CargoLocal=(Orientation.Inverse()*CargoActor->GetActorQuat()).Rotator();
    S.CargoPitchRoll=FVector2D(CargoLocal.Pitch,CargoLocal.Roll);
    S.CargoPitchRollRate=HasPrevious?FVector2D(
        FMath::FindDeltaAngleDegrees(PreviousAngles.X,S.CargoPitchRoll.X)/SampleDt,
        FMath::FindDeltaAngleDegrees(PreviousAngles.Y,S.CargoPitchRoll.Y)/SampleDt):(SameSample?S.CargoPitchRollRate:FVector2D::ZeroVector);
    S.SpreaderPitchRoll=AttitudeState.Angle; S.SpreaderPitchRollRate=AttitudeState.Rate;
    const FVector Gap=Orientation.UnrotateVector(S.SpreaderPosition-S.CargoPosition);
    S.bLanded=FMath::Abs(Gap.X)<=STSProfile.LandingTolerance && FMath::Abs(Gap.Y)<=STSProfile.LandingTolerance &&
        FMath::Abs(Gap.Z-154.5f)<=STSProfile.SeatingTolerance &&
        (S.SpreaderVelocity-S.CargoVelocity).Size()<=STSProfile.SettleSpeed &&
        FQuat::ErrorAutoNormalize(Orientation,CargoActor->GetActorQuat())<.02f;
    // Plant-side virtual load cells: dynamic force/moment balance, independent of controller estimates.
    const FVector CoG=CargoActor->CoGOffsetCm;
    S.CornerHalfWidthCm=CargoActor->SensorHalfWidthCm();
    S.CornerHalfLengthCm=CargoActor->SensorHalfLengthCm();
    const double EffectiveG=FMath::Max(.1,9.80665+PlantAcceleration.Z*.01);
    const float Weight=bCarrying?CargoActor->MassKg*EffectiveG:0;
    const double EX=CoG.X+(154.5-CoG.Z)*PlantAcceleration.X*.01/EffectiveG;
    const double EY=CoG.Y+(154.5-CoG.Z)*PlantAcceleration.Y*.01/EffectiveG;
    for(int32 I=0;I<4;++I)
    {
        S.Locked[I]=CornerLocked[I];
        S.CornerLoadsN[I]=FMath::Max(0.,Weight*(.5+((I&1)?1:-1)*EX/(2*S.CornerHalfWidthCm))*(.5+((I&2)?1:-1)*EY/(2*S.CornerHalfLengthCm))+(bCarrying?STSProfile.MassBiasKg*9.80665/4:0));
    }
    S.bAGVAligned=AGVAligned(); S.bCargoSupported=CargoSupported(); S.SwayDegrees=SuspensionState.SwayDegrees();
    S.SwayAngle=SuspensionState.Angle; S.SwayRate=SuspensionState.Rate; S.SkewDegrees=FMath::RadiansToDegrees(SuspensionState.Yaw);
    S.HoistAcceleration=PlantAcceleration.Z;
    SamplePickupGeometry();
    if(!bSTS)SampleRMGEnvironment();
    else
    {
        const FVector Destination=Local(Slots[1-SourceSlot]);
        S.bCraneClear=!bCoordinatedSTSTravel || Stage!=4 || !bCarrying ||
            STSTransferClear(FVector(Destination.X,Destination.Y,Local(S.SpreaderPosition).Z));
    }
    }
    for(const auto& Mount:STSProfile.Dynamics.Mounts)
    {
        double Value=0; bool Required=true;
        if(Mount.Key==TEXT("trolley_encoder"))
        {
            Value=Mount.ReadPosition(S.DrivePosition.X*.01);
            S.DrivePosition.X=(Value+Mount.MeasurementOrigin)*100;
            if(Mount.MaxTraversingSpeed>0 && FMath::Abs(S.DriveVelocity.X)*.01>Mount.MaxTraversingSpeed)
                InvalidSensor(FString::Printf(TEXT("%s speed %.3f exceeds %.3f m/s"),*Mount.Key,FMath::Abs(S.DriveVelocity.X)*.01,Mount.MaxTraversingSpeed));
        }
        else if(Mount.Key==TEXT("gantry_encoder")) Value=S.DrivePosition.Y*.01;
        else if(Mount.Key==TEXT("sway_sensor")) Value=S.SwayDegrees;
        else if(Mount.Key==TEXT("hoist_encoder")) Value=(BeamZ-Head.Z)*.01;
        else if(Mount.Key==TEXT("telescope_encoder")) Value=TelescopeLengthCm*.01;
        else if(Mount.Key==TEXT("wind")) Value=S.WindVelocityMetersPerSecond.Size();
        else if(Mount.Key==TEXT("twistlock_load"))
        {
            for(double Load:S.CornerLoadsN) if(Load<Mount.Minimum || Load>Mount.Maximum)
                InvalidSensor(FString::Printf(TEXT("%s %.3f outside [%.3f, %.3f]"),*Mount.Key,Load,Mount.Minimum,Mount.Maximum));
        }
        else Required=false;
        if(Required && Mount.Key!=TEXT("twistlock_load") && (Value<Mount.Minimum || Value>Mount.Maximum))
            InvalidSensor(FString::Printf(TEXT("%s %.3f outside [%.3f, %.3f]"),*Mount.Key,Value,Mount.Minimum,Mount.Maximum));
    }
}

double APortWorkingCrane::RailAccelerationLimit(double RequestedCmps2) const
{
    // Wheel/rail adhesion limits horizontal travel, not hoist cable tension.
    const double Available=100*PortFriction::Gravity*FMath::Max(0.,
        double(Friction.Utilization*Friction.Rail.Static(Surface)-Friction.RailRollingResistance));
    return FMath::Min(RequestedCmps2,Available);
}

bool APortWorkingCrane::MoveSTS(FVector Target,float Dt)
{
    bRailSafetyHold=false;
    PortSimPerformance::FScope Timing(TEXT("crane_dynamics"));
    if(!STSProfile.ContainsTarget(Target)) { Stop(TEXT("Automatic target outside STS envelope")); return false; }
    const double ControlPayload=bCarrying?(Pickup.EstimateValid?Pickup.EstimatedMass:STSProfile.ContainerMassKg):0;
    float Hoist=STSProfile.HoistLimit(ControlPayload,bCarrying);
    if(Stage==2 || Stage==5)
    {
        const float Distance=FMath::Abs(Target.Z-Observation.DrivePosition.Z);
        Hoist=FMath::Min(Hoist,FMath::Sqrt(FMath::Square(STSProfile.ApproachSpeed)+
            2.f*STSProfile.HoistAcceleration*FMath::Max(0.f,Distance-STSProfile.ApproachDistance)));
    }
    if(Stage==2 && bCarrying) Hoist=FMath::Min(Hoist,STSProfile.ApproachSpeed);
    FSTSDynamicsParameters C=STSProfile.Dynamics;
    // The plant receives the current environment. The controller receives the
    // latest 20 Hz wind-sensor observation below, just like the other feedback.
    C.Wind=Orientation.UnrotateVector(WindVelocityMetersPerSecond);
    C.WindProjectedAreasM2=WindProjectedAreasM2();
    // True mass constrains the actuator, independently of the load estimate
    // used by the controller's reference-speed selection above.
    const double Payload=bCarrying?CargoActor->MassKg:0, Mass=STSProfile.SpreaderMassKg+Payload;
    const double PowerSpeed=HoistDriveLimits::SpeedMps(C,Mass,STSProfile.HoistAcceleration*.01,STSProfile.HoistPowerW);
    if(PowerSpeed<=0) {Stop(TEXT("Hoist motor lacks acceleration/braking torque reserve"));return false;}
    Hoist=FMath::Min(Hoist,float(PowerSpeed*100));
    const FVector Limits(STSProfile.TrolleySpeed,STSProfile.GantrySpeed,Hoist);
    const FVector Accelerations(RailAccelerationLimit(STSProfile.TrolleyAcceleration),RailAccelerationLimit(STSProfile.GantryAcceleration),STSProfile.HoistAcceleration);
    if(Accelerations.X<=0 || Accelerations.Y<=0)
    { Stop(TEXT("Insufficient wheel/rail adhesion for automatic travel")); return false; }
    // True mass/CoG below belong only to the actuator/suspension plant. Position feedback comes from sensors.
    const FVector MeasuredPosition=Local(Observation.SpreaderPosition);
    const FVector MeasuredVelocity=Orientation.UnrotateVector(Observation.SpreaderVelocity);
    // During vessel pickup the target is not stationary. Feed the measured
    // secured-cargo velocity into the drive loop so the spreader follows the
    // deck through the seating window instead of repeatedly chasing its wake.
    const FVector TargetVelocity=Stage==2 && !bCarrying?
        Orientation.UnrotateVector(Observation.CargoVelocity):FVector::ZeroVector;
    const FVector TargetAcceleration=Stage==2 && !bCarrying?
        Orientation.UnrotateVector(Observation.CargoAcceleration):FVector::ZeroVector;
    const FVector TargetJerk=Stage==2 && !bCarrying?
        Orientation.UnrotateVector(Observation.CargoJerk):FVector::ZeroVector;
    const FVector TargetSnap=Stage==2 && !bCarrying?
        Orientation.UnrotateVector(Observation.CargoSnap):FVector::ZeroVector;
    const double Age=FMath::Max(0.,SimulationTime-Observation.Timestamp);
    double SensorSwayAngle=PI*.5;
    for(const auto& Mount:STSProfile.Dynamics.Mounts) if(Mount.Key==TEXT("sway_sensor") && Mount.Maximum>0)
        SensorSwayAngle=FMath::Min(SensorSwayAngle,FMath::DegreesToRadians(Mount.Maximum));
    const double BrakingStroke=FMath::Max(FMath::Square(double(Limits.X))/(2*FMath::Max(.001,Accelerations.X)),
        FMath::Square(double(Limits.Y))/(2*FMath::Max(.001,Accelerations.Y)));
    const double SwingStroke=(.5*STSProfile.Pickup.AcquisitionRadius+BrakingStroke)*.01;
    const FVector CoG=bCarrying?Orientation.UnrotateVector(CargoActor->GetActorQuat().RotateVector(
        CargoActor->CoGOffsetCm))*.01*(Payload/Mass):FVector::ZeroVector;
    // Equal slices avoid a nanosecond remainder from float tick durations.
    // Interpolation can snap on that remainder and amplify acceleration/rope force.
    const int32 Substeps=CraneSimulationSteps::Count(Dt,120.);
    const double Step=double(Dt)/Substeps;
    if(Step<=0) return false;
    for(int32 Substep=0;Substep<Substeps;++Substep)
    {
        const FVector Previous=AxisVelocity;
        FVector2D TravelAcceleration=FVector2D::ZeroVector;
        const double Length=FMath::Max(.1,(BeamZ-Head.Z)*.01);
        const double LengthRate=-AxisVelocity.Z*.01;
        auto SensorConfig=C;
        SensorConfig.Wind=Observation.WindVelocityMetersPerSecond;
        const FVector WindOffset=SensorConfig.WindEquilibriumOffset(Length,Mass)*100.;
        for(int32 I=0;I<3;++I)
        {
            const double PredictionTime=Age+Substep*Step;
            const double PredictedTarget=Target[I]+TargetVelocity[I]*PredictionTime+
                .5*TargetAcceleration[I]*PredictionTime*PredictionTime+
                TargetJerk[I]*PredictionTime*PredictionTime*PredictionTime/6.+
                TargetSnap[I]*FMath::Pow(PredictionTime,4)/24.;
            const double PredictedTargetVelocity=TargetVelocity[I]+TargetAcceleration[I]*PredictionTime+
                .5*TargetJerk[I]*PredictionTime*PredictionTime+TargetSnap[I]*FMath::Pow(PredictionTime,3)/6.;
            const double PredictedTargetAcceleration=TargetAcceleration[I]+TargetJerk[I]*PredictionTime+.5*TargetSnap[I]*PredictionTime*PredictionTime;
            const double PredictedTargetJerk=TargetJerk[I]+TargetSnap[I]*PredictionTime;
            // Horizontal position feedback uses the trolley/gantry encoders.
            // Feeding the suspended offset into this error and again through
            // sway-rate feedback makes the trolley chase the pendulum forever.
            const double FeedbackPosition=I<2
                ? Observation.DrivePosition[I]+Observation.DriveVelocity[I]*PredictionTime
                : MeasuredPosition[I]+MeasuredVelocity[I]*PredictionTime;
            // Hold the spreader over its target under a steady wind by parking
            // the trolley/gantry upwind of the aerodynamic equilibrium offset.
            // A suspended load is displaced from its accelerating support by
            // a*L/g. Move the support by the opposite amount so the spreader,
            // not merely the drive, follows the accelerating vessel target.
            const double AccelerationOffset=I<2 && Stage==2 && !bCarrying?
                -PredictedTargetAcceleration*Length/9.80665:0.;
            const double ControlTarget=I<2?PredictedTarget-WindOffset[I]-AccelerationOffset:PredictedTarget;
            // Follow a moving suspension reference, including its measured
            // jerk/snap. Treating this as a stationary anti-sway target makes
            // the drive fight the wave-induced reference angular velocity.
            const bool TrackingSource=I<2 && Stage==2 && !bCarrying;
            const double ControlTargetVelocity=PredictedTargetVelocity+(TrackingSource?
                PredictedTargetJerk*Length/9.80665-LengthRate*(WindOffset[I]+AccelerationOffset)/Length:0.);
            const double Error=ControlTarget-FeedbackPosition;
            double Desired=.8*Error+ControlTargetVelocity-.8*(Observation.DriveVelocity[I]-ControlTargetVelocity);

            double Acceleration=Accelerations[I];
            if(I==2)
            {
                const double ShaftFactor=C.Parts*C.GearRatio/C.DrumRadius;
                const double BaseTorque=Mass*9.80665/(ShaftFactor*C.Efficiency*C.MotorCount);
                const double TorqueLimit=FMath::Min(C.MotorMaxTorque,STSProfile.HoistPowerW/(C.MotorCount*FMath::Max(1.,FMath::Abs(AxisVelocity.Z*.01*ShaftFactor))));
                if(BaseTorque>C.MotorMaxTorque) {Stop(TEXT("Hoist motor cannot hold suspended mass"));return false;}
                const double AccelLimit=FMath::Max(0.,(TorqueLimit-BaseTorque)/(Mass/(ShaftFactor*C.Efficiency*C.MotorCount)+C.MotorInertia*ShaftFactor));
                Acceleration=FMath::Min(Acceleration,AccelLimit*100.);
                // Brake within the mechanical lifting stroke. SafeZ is a
                // transfer target, not a ceiling on high-tier acquisition or
                // vessel-heave tracking above the configured transfer height.
                const double UpwardStroke=FMath::Max(0.,double(STSProfile.LiftAboveRail)-Head.Z);
                Desired=FMath::Min(Desired,FMath::Sqrt(2.*Acceleration*UpwardStroke));
            }
            if(I<2)
            {
                // Acceleration feedback damps the pendulum; velocity feedback here adds phase lag.
                // Wind feed-forward is smooth and does not chase instantaneous
                // pendulum displacement. Track that moving equilibrium faster
                // than an ordinary travel target. Angle error relative to the
                // wind equilibrium rejects changing-wind lag without treating
                // the normal downwind lean as unwanted sway.
                const double WindTrackingGain=SensorConfig.Wind.IsNearlyZero()?1.:4.;
                const double EquilibriumOffset=WindOffset[I]+AccelerationOffset;
                const double EquilibriumAngle=FMath::Asin(FMath::Clamp(EquilibriumOffset*.01/Length,-1.,1.));
                const double ObservedRope=FMath::Max(.1,(BeamZ-Observation.DrivePosition.Z)*.01);
                // Use the same damped travel loop through high-level approach
                // and acquisition. The weak legacy approach loop allowed a long
                // next-job traverse to overshoot its reserved cargo corridor.
                // Only acquisition adds the measured moving-reference terms.
                const double ReferenceSwayRate=TrackingSource?-PredictedTargetJerk*.01/9.80665:0.;
                const double Feedback=C.TravelResponseRatio>0
                    ? C.TravelAcceleration(Error*.01,(Observation.DriveVelocity[I]-ControlTargetVelocity)*.01,
                        ObservedRope,-Observation.DriveVelocity.Z*.01,Observation.SwayRate[I]-ReferenceSwayRate,
                        Observation.SwayAngle[I],EquilibriumAngle)
                    : C.HorizontalAcceleration(Error*.01,
                    (Observation.DriveVelocity[I]-ControlTargetVelocity)*.01,FMath::Max(.1,(BeamZ-Observation.DrivePosition.Z)*.01),
                    Observation.SwayRate[I],Observation.SwayAngle[I],EquilibriumAngle,WindTrackingGain);
                const double Command=Feedback*100+PredictedTargetAcceleration+(TrackingSource?
                    (TargetSnap[I]*Length+2*LengthRate*PredictedTargetJerk-
                        PlantAcceleration.Z*.01*PredictedTargetAcceleration)/9.80665:0.);
                // Reserve half of the acquisition stroke for drive correction
                // and the remainder for the suspended motion. The stopping
                // band limits the anti-sway command without disabling it or
                // changing the reserved corridor/physical acceleration limit.
                const double SafeCommand=CraneTravelBraking::Acceleration(Error,
                    AxisVelocity[I]-ControlTargetVelocity,Command,Acceleration,
                    .5*STSProfile.Pickup.AcquisitionRadius,Step);
                TravelAcceleration[I]=SafeCommand*.01;
            }
            else
            {
                if(Stage==2 && !bCarrying)
                {
                    // Track heave acceleration while the existing horizontal
                    // stroke and sway braking continue to constrain travel.
                    const double VelocityError=PredictedTargetVelocity-MeasuredVelocity[I];
                    const double Requested=PredictedTargetAcceleration+4.*Error+4.*VelocityError;
                    const double NextStroke=FMath::Max(0.,double(AxisVelocity[I]))*Step+
                        .5*Acceleration*Step*Step;
                    const double UpwardLimit=FMath::Sqrt(2.*Acceleration*
                        FMath::Max(0.,double(STSProfile.LiftAboveRail)-Head.Z-NextStroke));
                    const double Command=FMath::Min(Requested,(UpwardLimit-AxisVelocity[I])/Step);
                    AxisVelocity[I]=FMath::Clamp(AxisVelocity[I]+FMath::Clamp(Command,-Acceleration,Acceleration)*Step,
                        -double(Limits[I]),double(Limits[I]));
                }
                else AxisVelocity[I]=FMath::FInterpConstantTo(AxisVelocity[I],FMath::Clamp(Desired,-double(Limits[I]),double(Limits[I])),Step,Acceleration);
                const double Next=Head[I]+AxisVelocity[I]*Step;
                const auto* Site=IsValid(CargoActor)?Cast<APortSiteLogistics>(CargoActor->GetOwner()):nullptr;
                if(I==1 && bSTS && Site && !Site->CanSTSGantryMove(this,(Home+Orientation.RotateVector(FVector(0,Next,0))).Y))
                { AxisVelocity[I]=0; TravelAcceleration.Y=0; bRailSafetyHold=true; }
                else Head[I]=Next;
            }
        }
        const double PredictionTime=Age+Substep*Step;
        const double ObservedLength=FMath::Max(.1,(BeamZ-Observation.DrivePosition.Z-
            Observation.DriveVelocity.Z*PredictionTime)*.01);
        const auto MeasuredAngle=Observation.SwayAngle+Observation.SwayRate*PredictionTime;
        const FVector WindAcceleration=SensorConfig.AerodynamicForce(
            Orientation.UnrotateVector(Observation.SpreaderVelocity)*.01)/FMath::Max(1.,STSProfile.SpreaderMassKg+ControlPayload);
        const double MaximumAngle=FMath::Min(SensorSwayAngle,FMath::Asin(FMath::Clamp(SwingStroke/ObservedLength,0.,.95)));
        TravelAcceleration=CraneTravelBraking::SwayAcceleration(MeasuredAngle,Observation.SwayRate,
            FVector2D(WindAcceleration.X,WindAcceleration.Y),TravelAcceleration,
            FVector2D(Accelerations.X*.01,Accelerations.Y*.01),ObservedLength,-AxisVelocity.Z*.01,
            C.Damping,MaximumAngle,Step);
        for(int32 I=0;I<2;++I)
        {
            AxisVelocity[I]=FMath::Clamp(AxisVelocity[I]+TravelAcceleration[I]*100*Step,-double(Limits[I]),double(Limits[I]));
            Head[I]+=AxisVelocity[I]*Step;
        }
        PlantAcceleration=(AxisVelocity-Previous)/Step;
        const bool FollowTilt=Stage==2 && !bCarrying && Observation.bTargetVisible;
        AttitudeState.Step(STSProfile.Pickup.Attitude,Step,
            Observation.SpreaderPitchRoll+Observation.SpreaderPitchRollRate*PredictionTime,
            Observation.SpreaderPitchRollRate,
            FollowTilt?Observation.CargoPitchRoll+Observation.CargoPitchRollRate*PredictionTime:FVector2D::ZeroVector,
            FollowTilt?Observation.CargoPitchRollRate:FVector2D::ZeroVector,
            Mass,PortSpreaderTelescope::TwistlockHalfWidthCm()*.01,
            PortSpreaderTelescope::TwistlockHalfLengthCm(TelescopeLengthCm)*.01);
        SuspensionState.Step(C,Step,Length,-AxisVelocity.Z*.01,PlantAcceleration*.01,AxisVelocity*.01,Mass,CoG,STSProfile.HoistPowerW,
            AttitudeState.Angle,AttitudeState.Torque,FVector2D(PortSpreaderTelescope::TwistlockHalfWidthCm()*.01,
                PortSpreaderTelescope::TwistlockHalfLengthCm(TelescopeLengthCm)*.01));
        if(!SuspensionState.Fault.IsEmpty()) {Stop(SuspensionState.Fault);return false;}
    }
    SuspendedOffset=SuspensionState.Offset*100;
    Head.X=FMath::Clamp(Head.X,double(STSProfile.MinTrolley()),double(STSProfile.MaxTrolley()));
    Head.Z=FMath::Clamp(Head.Z,-double(STSProfile.LiftBelowRail),double(STSProfile.LiftAboveRail));
    UpdateParts();
    // Completion thresholds follow the virtual sensor/control limits.  The old
    // 0.5 cm / 0.5 cm/s gate was much tighter than either pickup or landing
    // observations and made the controller spend tens of seconds asymptotically
    // approaching an accuracy that the simulated sensor did not require.
    // STS cargo becomes rigidly attached to a moving AGV after handover, so its
    // final placement must satisfy the fleet's 1 cm ownership/alignment check.
    // RMG placement remains governed by the yard landing tolerance.
    const float PositionTolerance=Stage==2?STSProfile.Pickup.CornerTolerance:
        (Stage==5?FMath::Min(STSProfile.LandingTolerance,bSTS?1.f:5.f):5.f);
    const float VelocityTolerance=Stage==2?STSProfile.Pickup.RelativeSpeed:
        (Stage==5?FMath::Min(STSProfile.SettleSpeed,5.f):5.f);
    return MeasuredPosition.Equals(Target,PositionTolerance) && Observation.SpreaderVelocity.Size()<VelocityTolerance &&
        Observation.SwayDegrees<STSProfile.SwayLimitDegrees && FMath::Abs(Observation.SkewDegrees)<C.SkewLimit;
}

FVector APortWorkingCrane::SpreaderVelocity() const
{ return Orientation.RotateVector(AxisVelocity+SuspensionState.OffsetVelocity*100); }

void APortWorkingCrane::BuildSTSSensors()
{
    auto* Cube=LoadObject<UStaticMesh>(nullptr,TEXT("/Engine/BasicShapes/Cube.Cube"));
    for(int32 MountIndex=0;MountIndex<STSProfile.Dynamics.Mounts.Num();++MountIndex)
    {
        const auto& Mount=STSProfile.Dynamics.Mounts[MountIndex];
        auto* Parent=Mount.Frame==TEXT("spreader")?Spreader.Get():(Mount.Frame==TEXT("trolley")?Trolley.Get():RootComponent.Get());
        auto AddMarker=[&](const FString& Name,FVector ConfiguredPosition)
        {
            auto* Marker=NewObject<UStaticMeshComponent>(this,*Name);
            Marker->SetStaticMesh(Cube); Marker->SetCollisionEnabled(ECollisionEnabled::NoCollision);
            Marker->SetupAttachment(Parent); Marker->SetAbsolute(false,false,true);
            const FVector Position=Mount.Frame==TEXT("spreader")?DynamicSpreaderMount(Mount,ConfiguredPosition):ConfiguredPosition;
            Marker->SetRelativeLocation(Position*100/Parent->GetComponentScale()); Marker->SetRelativeRotation(Mount.Rotation);
            Marker->SetWorldScale3D(FVector(.14)); Marker->RegisterComponent(); AddInstanceComponent(Marker);
            if(Mount.Frame==TEXT("spreader"))
            {
                SpreaderSensorInstances.Add(Marker);
                SpreaderSensorMountIndices.Add(MountIndex);
                SpreaderSensorConfiguredPositions.Add(ConfiguredPosition);
            }
            return Marker;
        };
        SensorMarkers.Add(AddMarker(FString::Printf(TEXT("Sensor_%s"),*Mount.Key),Mount.Position));
        for(int32 ExtraIndex=0;ExtraIndex<Mount.AdditionalPositions.Num();++ExtraIndex)
            AddMarker(FString::Printf(TEXT("Sensor_%s_extra_%d"),*Mount.Key,ExtraIndex),Mount.AdditionalPositions[ExtraIndex]);
        if(Mount.Key==TEXT("twistlock_load") || Mount.Key==TEXT("twistlock_state") || Mount.Key==TEXT("landed"))
            for(int32 Corner=0;Corner<3;++Corner)
            {
                const FVector P((Corner&1)?Mount.Position.X:-Mount.Position.X,(Corner&2)?Mount.Position.Y:-Mount.Position.Y,Mount.Position.Z);
                AddMarker(FString::Printf(TEXT("Sensor_%s_%d"),*Mount.Key,Corner),P);
            }
    }
    UpdateSpreaderSensors();
}

bool APortWorkingCrane::STSSensorContains(const FString& Key,FVector Point) const
{
    const int32 Index=STSProfile.Dynamics.Mounts.IndexOfByPredicate([&](const FSTSSensorMount& M){return M.Key==Key;});
    if(!SensorMarkers.IsValidIndex(Index)) return false;
    const bool bCamera=Key.Contains(TEXT("camera"),ESearchCase::IgnoreCase);
    const bool bLiDAR=Key.Contains(TEXT("lidar"),ESearchCase::IgnoreCase);
    const auto& M=STSProfile.Dynamics.Mounts[Index]; const auto* Marker=SensorMarkers[Index].Get();
    TArray<FVector> Origins;Origins.Add(Marker->GetComponentLocation());
    for(FVector Local:M.AdditionalPositions)
    {
        Local=DynamicSpreaderMount(M,Local);
        Origins.Add(Marker->GetAttachParent()->GetComponentLocation()+Marker->GetAttachParent()->GetComponentQuat().RotateVector(Local*100));
    }
    for(const FVector& Origin:Origins)
    {
        const FVector Delta=Point-Origin;const double Distance=Delta.Size()*.01;
        const double RainScale=bCamera?RainCameraRangeScale:(bLiDAR?RainLiDARRangeScale:1.f);
        const double RainMaximum=M.Maximum*RainScale;
        const bool bFogSensitive=bCamera||bLiDAR||(!bSTS && Key==TEXT("stack_profile"));
        const double Maximum=bFogSensitive?FMath::Min(RainMaximum,double(FogVisibilityMeters)):M.Maximum;
        if(Distance>=M.Minimum && Distance<=Maximum && FVector::DotProduct(Delta.GetSafeNormal(),Marker->GetForwardVector())>=FMath::Cos(FMath::DegreesToRadians(M.Fov*.5)))
            return !bCamera || HasCameraTargetLight(Point);
    }
    return false;
}
