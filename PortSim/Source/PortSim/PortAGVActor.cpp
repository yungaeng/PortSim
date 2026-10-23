#include "PortAGVActor.h"
#include "PortContainerActor.h"
#include "PortEnvironmentComponent.h"
#include "AGVReference.h"
#include "TerminalLayout.h"
#include "Components/StaticMeshComponent.h"
#include "Components/TextRenderComponent.h"
#include "Engine/StaticMesh.h"
#include "Materials/MaterialInterface.h"
#include "UObject/ConstructorHelpers.h"

APortAGVActor::APortAGVActor()
{
    Box(TEXT("AGV_Deck"),RootComponent,FVector(0,0,170),FVector(300,1380,100),true);
    static ConstructorHelpers::FObjectFinder<UStaticMesh> Cylinder(TEXT("/Engine/BasicShapes/Cylinder.Cylinder"));
    static ConstructorHelpers::FObjectFinder<UMaterialInterface> Rubber(TEXT("/Game/PortSim/Assets/Materials/M_SiteAsphalt.M_SiteAsphalt"));
    static ConstructorHelpers::FObjectFinder<UMaterialInterface> Steel(TEXT("/Game/PortSim/Assets/Materials/M_Steel.M_Steel"));
    auto WheelPart=[&](const FString& Name,FVector Position,float Diameter,float Width,UMaterialInterface* Material)
    {
        auto* Wheel=CreateDefaultSubobject<UStaticMeshComponent>(FName(*Name));
        Wheel->SetupAttachment(RootComponent);
        Wheel->SetStaticMesh(Cylinder.Object);
        Wheel->SetMobility(EComponentMobility::Movable);
        Wheel->SetRelativeLocation(Position);
        // Cylinder Z is the axle; AGVs travel along local Y, so turn the axle
        // onto local X. Preserve the existing 4.2 m chassis envelope.
        Wheel->SetRelativeRotation(FRotator(90,0,0));
        const FVector Size=Cylinder.Object->GetBounds().BoxExtent*2.f;
        Wheel->SetRelativeScale3D(FVector(Diameter/Size.X,Diameter/Size.Y,Width/Size.Z));
        Wheel->SetCollisionProfileName(TEXT("NoCollision"));
        Wheel->SetGenerateOverlapEvents(false);
        if(Material) Wheel->SetMaterial(0,Material);
        WheelMeshes.Add(Wheel);
    };
    for (int32 Side:{-1,1}) for (int32 Axle:{-1,0,1})
    {
        WheelPart(FString::Printf(TEXT("Bogie_AGV_%d_%d"),Side,Axle),
            FVector(Side*170,Axle*480,75),140,75,Rubber.Object);
        WheelPart(FString::Printf(TEXT("WheelHub_AGV_%d_%d"),Side,Axle),
            FVector(Side*208,Axle*480,75),80,4,Steel.Object);
    }
    Box(TEXT("AGV_Sensor"),RootComponent,FVector(0,670,265),FVector(110,45,90));
    Box(TEXT("Front_LiDAR"),RootComponent,FVector(0,700,250),FVector(70,35,55));
    Box(TEXT("Rear_LiDAR"),RootComponent,FVector(0,-700,250),FVector(70,35,55));
    Box(TEXT("Transponder_Antenna"),RootComponent,FVector(0,0,20),FVector(80,80,15));
    Box(TEXT("Wheel_Encoder_Left"),RootComponent,FVector(-180,0,75),FVector(30,30,30));
    Box(TEXT("Wheel_Encoder_Right"),RootComponent,FVector(180,0,75),FVector(30,30,30));
    Box(TEXT("Steering_Encoder"),RootComponent,FVector(0,500,75),FVector(35,35,35));
    NameLabel=Label(TEXT("AGVLabel"),FVector(0,0,230));
    NameLabel->SetText(FText::FromString(TEXT("AGV")));
    WorkerMarker=Box(TEXT("Emergency_Worker"),RootComponent,FVector(0,0,90),FVector(55,55,180));
    WorkerMarker->SetVisibility(false);
    WorkerLabel=Label(TEXT("Emergency_WorkerLabel"),FVector(0,0,200));
    WorkerLabel->SetText(FText::FromString(TEXT("TEST WORKER"))); WorkerLabel->SetVisibility(false);
    Tags.Add(TEXT("PortSim.AGV"));
}

void APortAGVActor::InitializeVehicle(int32 Number)
{
    VehicleID=Number;
    DisplayedBattery=-1; RefreshBatteryLabel();
#if WITH_EDITOR
    SetActorLabel(FString::Printf(TEXT("AGV_%02d"),Number));
    SetFolderPath(TEXT("PortSim/Equipment"));
#endif
}

void APortAGVActor::ResetVehicle(FVector Position)
{
    if(EmergencyCargo.IsValid())EmergencyCargo->CoGOffsetCm=OriginalEmergencyCoG;
    Emergency=FAGVEmergencyState(); EmergencyCargo.Reset();
    WorkerMarker->SetVisibility(false); WorkerLabel->SetVisibility(false);
    ActiveDeckOffsetCm=0;
    bDriveFault=false;
    DeckCoGOffsetCm=FVector::ZeroVector;
    Speed=0; CompletedJobs=0; PayloadKg=0; LastCurvaturePerCm=0;
    BatteryPercent=ChargeLimitPercent; bNeedsCharge=bCharging=false;
    DisplayedBattery=-1; RefreshBatteryLabel();
    // Park along the marked bay. Adjacent rows are separated for the vehicle
    // width in this pose; the steering controller then turns into the crossroad
    // through the 33 m gap between vehicles without any lateral crab motion.
    SetActorLocationAndRotation(Position,FRotator::ZeroRotator,false,nullptr,ETeleportType::TeleportPhysics);
    Sensors=FAGVSensorState(); Sensors.EstimatedPosition=Sensors.LastAbsolutePosition=Position;
    DockingAxis=FVector::ZeroVector;
    FMSApproachSpeedLimit=MAX_flt;
    bSteeringLookAhead=false;
    Dynamics.Reset(0,DynamicsSettings);
    Dynamics.RefreshLoads(DynamicsSettings,DynamicsInput());
    UpdateWheelVisuals();
}

void APortAGVActor::RefreshBatteryLabel()
{
    const int32 Percent=FMath::FloorToInt(BatteryPercent);
    if(DisplayedBattery==Percent && bDisplayedCharging==bCharging) return;
    DisplayedBattery=Percent; bDisplayedCharging=bCharging;
    NameLabel->SetText(FText::FromString(FString::Printf(TEXT("AGV %d | %d%% %s"),VehicleID,Percent,
        bCharging?TEXT("CHARGING"):TEXT(""))));
    NameLabel->SetTextRenderColor(bCharging?FColor::Cyan:BatteryPercent<=LowBatteryPercent?FColor::Red:FColor::White);
}

void APortAGVActor::AdvanceBattery(double Seconds,bool Operating,bool Charging)
{
    bCharging=Charging && BatteryPercent<ChargeLimitPercent;
    if(Seconds>0)
    {
        if(bCharging) BatteryPercent=FMath::Min(ChargeLimitPercent,BatteryPercent+Seconds*60./1800.);
        else if(Operating) BatteryPercent=FMath::Max(0.,BatteryPercent-Seconds*60./28800.);
    }
    if(BatteryPercent<=LowBatteryPercent) bNeedsCharge=true;
    if(BatteryPercent>=ChargeLimitPercent) { bNeedsCharge=false; bCharging=false; }
    RefreshBatteryLabel();
}

FVector APortAGVActor::CargoPosition() const
{
    return DeckPosition(ActiveDeckOffsetCm);
}

FVector APortAGVActor::DeckPosition(float OffsetCm) const
{
    return GetActorLocation()+GetActorRightVector()*OffsetCm+FVector(0,0,349.5f);
}

bool APortAGVActor::MoveToX(float X,float Dt)
{
    const FVector P=GetActorLocation(),Target(X,P.Y,P.Z);
    SetFMSCommand(Target,0,1,true);
    return MoveToPosition(Target,Dt,true);
}

FVector APortAGVActor::PlannedMotionDirection(FVector Target) const
{
    const FVector Longitudinal=GetActorRightVector().GetSafeNormal2D();
    return Sensors.bReversing?-Longitudinal:Longitudinal;
}

bool APortAGVActor::MoveToPosition(FVector Target,float Dt,bool StopAtTarget)
{
    if(bDriveFault || !Dynamics.Fault.IsEmpty()) { SetFMSHold(true); return false; }
    const FVector Previous=GetActorLocation();
    const FVector ToTarget=Target-Previous;
    const float Distance=ToTarget.Size2D();
    const float Acceptance=Sensors.bDockingNode?AGVReference::DockingAccuracyCm:FMSAcceptanceCm;
    const float AxialDot=DockingAxis.IsNearlyZero()?1.f:FMath::Abs(FVector::DotProduct(
        GetActorRightVector().GetSafeNormal2D(),DockingAxis));
    const float DockingHeadingError=FMath::RadiansToDegrees(FMath::Acos(FMath::Clamp(AxialDot,0.f,1.f)));
    const bool HeadingAligned=!Sensors.bDockingNode ||
        DockingHeadingError<=AGVReference::DockingHeadingToleranceDegrees;
    const bool AtPosition=Distance<=Acceptance && HeadingAligned;
    if(AtPosition && (!StopAtTarget || (Speed<.5f && FMath::Abs(Dynamics.V)<.005)))
    {
        // No waypoint teleport is used.  Final docking keeps the supplied
        // +/-50 mm limit; intermediate road nodes use the steering sweep.
        CorrectAtTransponder(Previous);
        Sensors.LateralSlipCm=0;
        if(StopAtTarget) ConstrainDynamicsStop();
        return true;
    }
    const FVector SteeringVector=bSteeringLookAhead && !Sensors.bDockingNode?
        SteeringLookAhead-Previous:ToTarget;
    FVector DesiredDirection=SteeringVector.GetSafeNormal2D();
    if(Sensors.bDockingNode && !DockingAxis.IsNearlyZero())
    {
        // Track the final FMS axis instead of chasing the docking point in a
        // circle.  If the vehicle passes the longitudinal station it changes
        // gear at zero speed and makes a normal car-like correction.
        const float Along=FVector::DotProduct(ToTarget,DockingAxis);
        const FVector LineCorrection=ToTarget-DockingAxis*Along;
        const FVector CurrentAxis=Sensors.bReversing?-GetActorRightVector():GetActorRightVector();
        // A car-like chassis cannot erase residual cross-track error by sliding
        // sideways at the handover point.  Keep driving through the station and
        // perform a short forward/reverse shunt until the longitudinal centreline
        // is inside the 50 mm docking tolerance.
        const float CrossTrackError=LineCorrection.Size2D();
        const float CrossTrackBand=CrossTrackError>AGVReference::DockingAccuracyCm?
            FMath::Clamp(CrossTrackError*10.f,100.f,300.f):AGVReference::DockingAccuracyCm;
        const float HeadingBand=DockingHeadingError>AGVReference::DockingHeadingToleranceDegrees?
            FMath::Clamp(2.f*AGVReference::MinimumInnerTurnRadiusCm*
                FMath::DegreesToRadians(DockingHeadingError),50.f,300.f):AGVReference::DockingAccuracyCm;
        const float ReversalBand=FMath::Max(CrossTrackBand,HeadingBand);
        const float TravelSign=FMath::Abs(Along)>ReversalBand?(Along>=0.f?1.f:-1.f):
            (FVector::DotProduct(CurrentAxis,DockingAxis)>=0.f?1.f:-1.f);
        const FVector TravelAxis=DockingAxis*TravelSign;
        const float LookAhead=FMath::Clamp(FMath::Abs(Along),100.f,1000.f);
        DesiredDirection=(TravelAxis*LookAhead+LineCorrection).GetSafeNormal2D();
        const bool Reverse=FVector::DotProduct(GetActorRightVector(),DesiredDirection)<0.f;
        if(Reverse!=Sensors.bReversing)
        {
            if(Speed>.1f) { ConstrainDynamicsStop(); return false; }
            Sensors.bReversing=Reverse;
        }
    }
    const FVector MotionDirection=PlannedMotionDirection(Target);
    Sensors.bReversing=FVector::DotProduct(MotionDirection,GetActorRightVector())<0;
    const float Dot=FMath::Clamp(FVector::DotProduct(MotionDirection,DesiredDirection),-1.f,1.f);
    const float Cross=MotionDirection.X*DesiredDirection.Y-MotionDirection.Y*DesiredDirection.X;
    const float HeadingError=FMath::Atan2(Cross,Dot);
    const float SteeringDistance=bSteeringLookAhead && !Sensors.bDockingNode?SteeringVector.Size2D():Distance;
    const float CurvatureLookAhead=FMath::Clamp(SteeringDistance,Sensors.bDockingNode?100.f:25.f,
        2.f*AGVReference::MinimumInnerTurnRadiusCm);
    const float RequestedCurvature=2.f*FMath::Sin(HeadingError)/CurvatureLookAhead;
    const float Curvature=FMath::Clamp(RequestedCurvature,
        -1.f/AGVReference::MinimumInnerTurnRadiusCm,1.f/AGVReference::MinimumInnerTurnRadiusCm);
    LastCurvaturePerCm=Curvature;
    const float HeadingLimit=FMath::Abs(HeadingError)>FMath::DegreesToRadians(5.f)?
        FMath::Min(SpeedLimitCmPerSecond(),CurveSpeedCmPerSecond()):SpeedLimitCmPerSecond();
    const float CurveLimit=FMath::Abs(Curvature)>1.e-6f?
        FMath::Sqrt(.5f*TireGripCmPerSecondSquared()/FMath::Abs(Curvature)):SpeedLimitCmPerSecond();
    const float Limit=FMath::Min3(HeadingLimit,FMSApproachSpeedLimit,CurveLimit);
    float Desired=StopAtTarget?FMath::Min(Limit,FMath::Sqrt(2.f*BrakingCmPerSecondSquared()*Distance)):Limit;
    if((Sensors.bDockingNode || FMSAcceptanceCm<=100.f) &&
        Distance<=AGVReference::DockingApproachDistanceCm)
        Desired=FMath::Min(Desired,AGVReference::DockingSpeedCmPerSecond);
    if(AtPosition && StopAtTarget) Desired=0;
    AdvanceDynamics((Sensors.bReversing?-1:1)*Desired*.01,
        (Sensors.bReversing?-1:1)*Curvature*100.,Dt);
    return false;
}

void APortAGVActor::SetFMSCommand(FVector Target,int32 NodeIndex,int32 NodeCount,bool Docking,float AcceptanceCm,
    FVector ExpectedDockingAxis)
{
    const bool NewNode=!Sensors.FMSNextNode.Equals(Target,.01f) || Sensors.FMSNodeIndex!=NodeIndex;
    if(NewNode)
    {
        if(Docking)
        {
            const FVector Axis=ExpectedDockingAxis.IsNearlyZero()?
                (Target-Sensors.FMSNextNode).GetSafeNormal2D():ExpectedDockingAxis.GetSafeNormal2D();
            if(!Axis.IsNearlyZero()) DockingAxis=Axis;
        }
        const FVector Desired=(Target-GetActorLocation()).GetSafeNormal2D();
        // Select forward/reverse once at the first node and keep that gear for
        // the full FMS route. Re-evaluating it at every sampled corner could
        // flip the longitudinal axis halfway through an arc and send the AGV
        // outside the road while it tried to turn back to the next node.
        if(NodeIndex==0 && !Desired.IsNearlyZero())
        {
            const bool Reverse=FVector::DotProduct(GetActorRightVector(),Desired)<0;
            if(Reverse!=Sensors.bReversing && Speed>.1f) ConstrainDynamicsStop();
            Sensors.bReversing=Reverse;
        }
    }
    Sensors.FMSNextNode=Target; Sensors.FMSNodeIndex=NodeIndex; Sensors.FMSNodeCount=NodeCount;
    Sensors.bFMSConnected=true; Sensors.bDockingNode=Docking;
    FMSAcceptanceCm=AcceptanceCm>0.f?AcceptanceCm:AGVReference::WaypointAcceptanceCm;
    if(!GetActorLocation().Equals(Target,AGVReference::DockingAccuracyCm)) Sensors.bTransponderLocked=false;
}

void APortAGVActor::SetFMSHold(bool Hold)
{
    Sensors.bFMSHold=Hold;
    if(Hold) ConstrainDynamicsStop();
}

void APortAGVActor::UpdateLiDARObservation(float FrontDistanceCm,float RearDistanceCm,float SafetyDistanceCm,float DetectionRangeCm,
    bool ControlledStop,int32 BlockingVehicleID)
{
    Sensors.FrontObstacleDistanceCm=FrontDistanceCm;
    Sensors.RearObstacleDistanceCm=RearDistanceCm;
    Sensors.LiDARSafetyDistanceCm=SafetyDistanceCm;
    Sensors.LiDARDetectionRangeCm=DetectionRangeCm;
    Sensors.LiDARBlockingVehicleID=BlockingVehicleID;
    Sensors.bLiDARClear=!ControlledStop;
    Sensors.bControlledStop=ControlledStop;
    if(ControlledStop) ConstrainDynamicsStop();
}

void APortAGVActor::UpdateOdometry(FVector PreviousPosition)
{
    const FVector Delta=GetActorLocation()-PreviousPosition;
    // Wheel distance is integrated from angular velocity in AdvanceDynamics.
    // Navigation remains idealized transponder/IMU fusion, not raw wheel-only dead reckoning.
    Sensors.EstimatedPosition+=Delta;
    Sensors.PositionErrorCm=FVector::Dist2D(Sensors.EstimatedPosition,GetActorLocation());
}

void APortAGVActor::CorrectAtTransponder(FVector Position)
{
    const int32 X=FMath::RoundToInt(Position.X);
    const int32 Y=FMath::RoundToInt(Position.Y);
    Sensors.CurrentTransponderID=int32(HashCombine(GetTypeHash(X),GetTypeHash(Y))&0x7fffffff);
    Sensors.LastAbsolutePosition=Position;
    const float Phase=VehicleID*.61803398875f+X*.0001f+Y*.00013f;
    const FVector BoundedError(FMath::Sin(Phase),FMath::Cos(Phase),0);
    Sensors.EstimatedPosition=Position+BoundedError*AGVReference::PositionAccuracyCm;
    Sensors.PositionErrorCm=AGVReference::PositionAccuracyCm;
    Sensors.bTransponderLocked=true;
}

float APortAGVActor::SpeedLimitCmPerSecond() const
{
    // Existing site speeds remain authoritative and stay below the 7 m/s reference maximum.
    return AGVReference::SiteSpeedLimit(PayloadKg);
}

float APortAGVActor::AccelerationCmPerSecondSquared() const
{
    const float Traction=FMath::Max(0.f,LongitudinalGripCmPerSecondSquared()*Friction.DrivenWeightFraction-
        float(100*PortFriction::Gravity*Friction.TireRollingResistance));
    // Rain/snow/ice are already included in Surface; do not derate them twice.
    return FMath::Min(AGVReference::AccelerationLimit(PayloadKg),Traction);
}

float APortAGVActor::WindResistanceNewtons() const
{
    const FVector Velocity=(Sensors.bReversing?-GetActorRightVector():GetActorRightVector())*Speed;
    const FVector Size(4.2,14.6,PayloadKg>0?4.3:2.);
    return FPortWindAerodynamics::DragForceNewtons(WindVelocity,Velocity,GetActorQuat(),Size,1.2).Size2D();
}
float APortAGVActor::TireGripCmPerSecondSquared() const
{
    // Reserve full drag magnitude conservatively for both headwind and crosswind.
    return FMath::Max(0.f,float(100*(Friction.Utilization*Friction.Tire.Static(Surface)*PortFriction::Gravity-
        WindResistanceNewtons()/(Friction.AGVEmptyMassKg+FMath::Max(0.f,PayloadKg)))));
}
float APortAGVActor::LongitudinalGripCmPerSecondSquared() const
{ return PortFriction::LongitudinalBudget(TireGripCmPerSecondSquared(),Speed*Speed*FMath::Abs(LastCurvaturePerCm)); }
float APortAGVActor::BrakingCmPerSecondSquared() const
{
    // Reserve half the adhesion for steering before entering a curve.
    return FMath::Min(AGVReference::AccelerationLimit(PayloadKg),.5f*TireGripCmPerSecondSquared());
}
float APortAGVActor::CurveSpeedCmPerSecond() const
{
    auto Input=DynamicsInput(); Input.CurvaturePerM=1./(AGVReference::MinimumInnerTurnRadiusCm*.01);
    auto Settings=DynamicsSettings; Settings.Sanitize();
    FAGVDynamicsState Loads; Loads.RefreshLoads(Settings,Input);
    const float Stability=DynamicsSettings.bStabilitySpeedControl?Loads.StabilityLimitMps*100:MAX_flt;
    return FMath::Min3(AGVReference::ReferenceCurveSpeedCmPerSecond,Stability,
        FMath::Sqrt(.5f*TireGripCmPerSecondSquared()*AGVReference::MinimumInnerTurnRadiusCm));
}

bool APortAGVActor::PayloadWithinReferenceLimit() const
{ return PayloadKg<=AGVReference::MaximumPayloadKg+KINDA_SMALL_NUMBER; }
