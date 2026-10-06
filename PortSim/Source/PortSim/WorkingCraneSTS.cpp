#include "PortWorkingCrane.h"
#include "PortContainerActor.h"
#include "PortAGVActor.h"
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
    if(Ready && bSTS && bJobActive) SampleSTS(true);
}

APortAGVActor* APortWorkingCrane::GetHandoverVehicle() const { return HandoverAGV.Get(); }

void APortWorkingCrane::ClearSTSState()
{
    AxisVelocity=PlantAcceleration=FVector::ZeroVector;
    Pickup=FSTSPickupController();
    for(int32 I=0;I<4;++I){LockProgress[I]=0;PhysicalSeating[I]=false;}
    Observation=FSTSObservation(); NextSample=0;
    JobSeconds=PausedSeconds=LastJobSeconds=LastPausedSeconds=0;
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
    constexpr float CentreHalfLength=250.f;
    const float ArmLength=FMath::Max(40.f,EndY-CentreHalfLength);
    SpreaderCenter->SetRelativeLocation(FVector::ZeroVector);
    SpreaderCenter->SetRelativeScale3D(FVector(243.8f,500.f,50.f)/100.f);
    for(int32 End=0;End<2;++End)
    {
        const float Sign=End?1.f:-1.f;
        TelescopeArms[End]->SetRelativeLocation(FVector(0,Sign*(CentreHalfLength+ArmLength*.5f),0));
        TelescopeArms[End]->SetRelativeScale3D(FVector(125.f,ArmLength,34.f)/100.f);
        SpreaderEndBeams[End]->SetRelativeLocation(FVector(0,Sign*EndY,0));
    }
    for(int32 Corner=0;Corner<4;++Corner)
        TwistLocks[Corner]->SetRelativeLocation(FVector((Corner&1)?HalfWidth:-HalfWidth,(Corner&2)?EndY:-EndY,-35.f));
    UpdateSpreaderSensors();
}

bool APortWorkingCrane::AGVAligned() const
{
    if(!bExternalJobs) return true;
    return IsValid(HandoverAGV) && HandoverAGV->Speed<=0.1f &&
        HandoverAGV->CargoPosition().Equals(Slots[bSTS?1-SourceSlot:SourceSlot],STSProfile.AGVTolerance) &&
        ((bSTS?Stage<5:Stage<2) || STSSensorContains(TEXT("agv_position_lidar"),HandoverAGV->CargoPosition())) &&
        FMath::Abs(FMath::FindDeltaAngleDegrees(HandoverAGV->GetActorRotation().Yaw,Orientation.Rotator().Yaw))<=STSProfile.AGVHeadingTolerance;
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
    for(int32 I=0;I<4;++I)
    {
        const FVector P=CargoActor->GetActorLocation()+Orientation.RotateVector(FVector((I&1)?HalfWidth:-HalfWidth,(I&2)?HalfLength:-HalfLength,0));
        FHitResult Hit;
        if(!GetWorld()->LineTraceSingleByChannel(Hit,P,P-FVector(0,0,129.5f+STSProfile.SupportTolerance),ECC_Visibility,Query) || Hit.ImpactNormal.Z<.8f) return false;
        if(bSTS && bExternalJobs && Stage==5 && Hit.GetActor()!=HandoverAGV) return false;
    }
    return true;
}

void APortWorkingCrane::SampleSTS(bool Force)
{
    if(!STSProfile.bReady || !IsValid(CargoActor) || (!Force && SimulationTime<NextSample)) return;
    NextSample=SimulationTime+STSProfile.SensorPeriod;
    auto& S=Observation; S.Timestamp=SimulationTime; S.bValid=!bSensorFault;
    S.DrivePosition=Head+FVector(STSProfile.PositionBias);
    S.DriveVelocity=AxisVelocity;
    S.SpreaderPosition=HeadPosition()+Orientation.RotateVector(FVector(STSProfile.PositionBias));
    S.SpreaderVelocity=SpreaderVelocity();
    S.CargoPosition=CargoActor->GetActorLocation()+Orientation.RotateVector(STSProfile.Pickup.PoseBias);
    S.CargoVelocity=bCarrying?S.SpreaderVelocity:CargoActor->GetBody()->GetPhysicsLinearVelocity();
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
    S.SwayRate=SuspensionState.Rate; S.SkewDegrees=FMath::RadiansToDegrees(SuspensionState.Yaw);
    S.HoistAcceleration=PlantAcceleration.Z;
    SamplePickupGeometry();
    if(!bSTS)SampleRMGEnvironment();
    for(const auto& Mount:STSProfile.Dynamics.Mounts)
    {
        double Value=0; bool Required=true;
        if(Mount.Key==TEXT("trolley_encoder"))
        {
            Value=Mount.ReadPosition(S.DrivePosition.X*.01);
            S.DrivePosition.X=(Value+Mount.MeasurementOrigin)*100;
            if(Mount.MaxTraversingSpeed>0 && FMath::Abs(S.DriveVelocity.X)*.01>Mount.MaxTraversingSpeed) S.bValid=false;
        }
        else if(Mount.Key==TEXT("gantry_encoder")) Value=S.DrivePosition.Y*.01;
        else if(Mount.Key==TEXT("sway_sensor")) Value=S.SwayDegrees;
        else if(Mount.Key==TEXT("hoist_encoder")) Value=(BeamZ-Head.Z)*.01;
        else if(Mount.Key==TEXT("telescope_encoder")) Value=TelescopeLengthCm*.01;
        else if(Mount.Key==TEXT("twistlock_load")) for(double Load:S.CornerLoadsN) {if(Load<Mount.Minimum || Load>Mount.Maximum) S.bValid=false;}
        else Required=false;
        if(Required && Mount.Key!=TEXT("twistlock_load") && (Value<Mount.Minimum || Value>Mount.Maximum)) S.bValid=false;
    }
}

bool APortWorkingCrane::MoveSTS(FVector Target,float Dt)
{
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
    const auto& C=STSProfile.Dynamics;
    Hoist=FMath::Min(Hoist,float(C.MotorMaxRPM*2*PI/60*C.DrumRadius/(C.Parts*C.GearRatio)*100));
    const FVector Limits(STSProfile.TrolleySpeed,STSProfile.GantrySpeed,Hoist);
    const FVector Accelerations(STSProfile.TrolleyAcceleration,STSProfile.GantryAcceleration,STSProfile.HoistAcceleration);
    // True mass/CoG below belong only to the actuator/suspension plant. Position feedback comes from sensors.
    const double Payload=bCarrying?CargoActor->MassKg:0, Mass=STSProfile.SpreaderMassKg+Payload;
    const FVector MeasuredPosition=Local(Observation.SpreaderPosition);
    FVector ControlledPosition=MeasuredPosition;
    // The trolley/gantry encoders close the travel loop.  Spreader cameras and
    // sway sensors independently decide final target completion.  Feeding the
    // entire suspended offset back as trolley position created a slow,
    // non-collocated loop that lingered near the target.
    const FVector MeasuredDrivePosition=Observation.DrivePosition;
    ControlledPosition.X=MeasuredDrivePosition.X;
    ControlledPosition.Y=MeasuredDrivePosition.Y;
    const double Age=FMath::Max(0.,SimulationTime-Observation.Timestamp);
    const FVector CoG=bCarrying?CargoActor->CoGOffsetCm*.01*(Payload/Mass):FVector::ZeroVector;
    // Equal slices avoid a nanosecond remainder from float tick durations.
    // Interpolation can snap on that remainder and amplify acceleration/rope force.
    const int32 Substeps=FMath::Max(1,FMath::CeilToInt(double(Dt)*120.));
    const double Step=double(Dt)/Substeps;
    if(Step<=0) return false;
    for(int32 Substep=0;Substep<Substeps;++Substep)
    {
        const FVector Previous=AxisVelocity;
        const double Length=FMath::Max(.1,(BeamZ-Head.Z)*.01);
        for(int32 I=0;I<3;++I)
        {
            const double Error=Target[I]-(ControlledPosition[I]+Observation.DriveVelocity[I]*(Age+Substep*Step));
            double Desired=.8*Error-.8*Observation.DriveVelocity[I];

            double Acceleration=Accelerations[I];
            if(I==2)
            {
                const double ShaftFactor=C.Parts*C.GearRatio/C.DrumRadius;
                const double BaseTorque=Mass*9.80665/(ShaftFactor*C.Efficiency*C.MotorCount);
                const double TorqueLimit=FMath::Min(C.MotorMaxTorque,STSProfile.HoistPowerW/(C.MotorCount*FMath::Max(1.,FMath::Abs(AxisVelocity.Z*.01*ShaftFactor))));
                if(BaseTorque>C.MotorMaxTorque) {Stop(TEXT("Hoist motor cannot hold suspended mass"));return false;}
                const double AccelLimit=FMath::Max(0.,(TorqueLimit-BaseTorque)/(Mass/(ShaftFactor*C.Efficiency*C.MotorCount)+C.MotorInertia*ShaftFactor));
                Acceleration=FMath::Min(Acceleration,AccelLimit*100.);
            }
            if(I<2)
            {
                // Acceleration feedback damps the pendulum; velocity feedback here adds phase lag.
                const double Command=C.HorizontalAcceleration(Error*.01,Observation.DriveVelocity[I]*.01,FMath::Max(.1,(BeamZ-Observation.DrivePosition.Z)*.01),Observation.SwayRate[I])*100;
                AxisVelocity[I]=FMath::Clamp(AxisVelocity[I]+FMath::Clamp(Command,-Acceleration,Acceleration)*Step,-double(Limits[I]),double(Limits[I]));
            }
            else AxisVelocity[I]=FMath::FInterpConstantTo(AxisVelocity[I],FMath::Clamp(Desired,-double(Limits[I]),double(Limits[I])),Step,Acceleration);
            Head[I]+=AxisVelocity[I]*Step;
        }
        PlantAcceleration=(AxisVelocity-Previous)/Step;
        SuspensionState.Step(C,Step,Length,-AxisVelocity.Z*.01,PlantAcceleration*.01,AxisVelocity*.01,Mass,CoG,STSProfile.HoistPowerW);
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
        if(Distance>=M.Minimum && Distance<=M.Maximum && FVector::DotProduct(Delta.GetSafeNormal(),Marker->GetForwardVector())>=FMath::Cos(FMath::DegreesToRadians(M.Fov*.5)))return true;
    }
    return false;
}
