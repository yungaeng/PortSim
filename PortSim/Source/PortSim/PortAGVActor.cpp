#include "PortAGVActor.h"
#include "AGVReference.h"
#include "TerminalLayout.h"
#include "Components/StaticMeshComponent.h"
#include "Components/TextRenderComponent.h"
#include "Engine/StaticMesh.h"
#include "UObject/ConstructorHelpers.h"

APortAGVActor::APortAGVActor()
{
    Box(TEXT("AGV_Deck"),RootComponent,FVector(0,0,170),FVector(300,1380,100),true);
    for (int32 Side:{-1,1}) for (int32 Axle:{-1,0,1})
        Box(*FString::Printf(TEXT("Bogie_AGV_%d_%d"),Side,Axle),RootComponent,FVector(Side*170,Axle*480,75),FVector(75,160,110));
    Box(TEXT("AGV_Sensor"),RootComponent,FVector(0,670,265),FVector(110,45,90));
    Box(TEXT("Front_LiDAR"),RootComponent,FVector(0,700,250),FVector(70,35,55));
    Box(TEXT("Rear_LiDAR"),RootComponent,FVector(0,-700,250),FVector(70,35,55));
    Box(TEXT("Transponder_Antenna"),RootComponent,FVector(0,0,20),FVector(80,80,15));
    Box(TEXT("Wheel_Encoder_Left"),RootComponent,FVector(-180,0,75),FVector(30,30,30));
    Box(TEXT("Wheel_Encoder_Right"),RootComponent,FVector(180,0,75),FVector(30,30,30));
    Box(TEXT("Steering_Encoder"),RootComponent,FVector(0,500,75),FVector(35,35,35));
    NameLabel=Label(TEXT("AGVLabel"),FVector(0,0,230));
    NameLabel->SetText(FText::FromString(TEXT("AGV")));
    Tags.Add(TEXT("PortSim.AGV"));
}

void APortAGVActor::InitializeVehicle(int32 Number)
{
    VehicleID=Number;
    NameLabel->SetText(FText::FromString(FString::Printf(TEXT("AGV %d"),Number)));
#if WITH_EDITOR
    SetActorLabel(FString::Printf(TEXT("AGV_%02d"),Number));
    SetFolderPath(TEXT("PortSim/Equipment"));
#endif
}

void APortAGVActor::ResetVehicle(FVector Position)
{
    Speed=0; CompletedJobs=0; PayloadKg=0;
    // Park along the marked bay. Adjacent rows are separated for the vehicle
    // width in this pose; the steering controller then turns into the crossroad
    // through the 33 m gap between vehicles without any lateral crab motion.
    SetActorLocationAndRotation(Position,FRotator::ZeroRotator,false,nullptr,ETeleportType::TeleportPhysics);
    Sensors=FAGVSensorState(); Sensors.EstimatedPosition=Sensors.LastAbsolutePosition=Position;
    DockingAxis=FVector::ZeroVector;
}

FVector APortAGVActor::CargoPosition() const
{
    return GetActorLocation()+FVector(0,0,349.5f);
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
    const FVector Previous=GetActorLocation();
    const FVector ToTarget=Target-Previous;
    const float Distance=ToTarget.Size2D();
    const float Acceptance=Sensors.bDockingNode?AGVReference::DockingAccuracyCm:FMSAcceptanceCm;
    const float AxialDot=DockingAxis.IsNearlyZero()?1.f:FMath::Abs(FVector::DotProduct(
        GetActorRightVector().GetSafeNormal2D(),DockingAxis));
    const float DockingHeadingError=FMath::RadiansToDegrees(FMath::Acos(FMath::Clamp(AxialDot,0.f,1.f)));
    const bool HeadingAligned=!Sensors.bDockingNode ||
        DockingHeadingError<=AGVReference::DockingHeadingToleranceDegrees;
    if(Distance<=Acceptance && HeadingAligned)
    {
        // No waypoint teleport is used.  Final docking keeps the supplied
        // +/-50 mm limit; intermediate road nodes use the steering sweep.
        CorrectAtTransponder(Previous);
        Sensors.SteeringAngleDegrees=0;
        Sensors.LateralSlipCm=0;
        if(StopAtTarget) Speed=0;
        return true;
    }
    FVector DesiredDirection=ToTarget.GetSafeNormal2D();
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
            if(Speed>.1f) { Speed=0; return false; }
            Sensors.bReversing=Reverse;
        }
    }
    const FVector MotionDirection=PlannedMotionDirection(Target);
    Sensors.bReversing=FVector::DotProduct(MotionDirection,GetActorRightVector())<0;
    const float Dot=FMath::Clamp(FVector::DotProduct(MotionDirection,DesiredDirection),-1.f,1.f);
    const float Cross=MotionDirection.X*DesiredDirection.Y-MotionDirection.Y*DesiredDirection.X;
    const float HeadingError=FMath::Atan2(Cross,Dot);
    const float Acceleration=AccelerationCmPerSecondSquared();
    const float Limit=FMath::Abs(HeadingError)>FMath::DegreesToRadians(5.f)?
        FMath::Min(SpeedLimitCmPerSecond(),AGVReference::ReferenceCurveSpeedCmPerSecond):SpeedLimitCmPerSecond();
    float Desired=StopAtTarget?FMath::Min(Limit,FMath::Sqrt(2.f*Acceleration*Distance)):Limit;
    if((Sensors.bDockingNode || FMSAcceptanceCm<=100.f) &&
        Distance<=AGVReference::DockingApproachDistanceCm)
        Desired=FMath::Min(Desired,AGVReference::DockingSpeedCmPerSecond);
    Speed=FMath::FInterpConstantTo(Speed,Desired,Dt,Acceleration);
    const float Travel=FMath::Min(Distance,Speed*Dt);
    // Pure-pursuit curvature steers the longitudinal vehicle axis through the
    // target.  Curvature is capped by the published 5.8 m inner turn radius.
    // Use normal road smoothing between coarse FMS nodes.  During the final
    // low-speed shunt, use the pure-pursuit look-ahead directly so small
    // cross-track errors still produce useful steering.  Both paths remain
    // capped by the same published 5.8 m inner turning radius below.
    // A road node may be hundreds of metres away. Using that full distance as
    // pure-pursuit look-ahead makes an initial 90-degree turn radius equally
    // huge and lets the AGV drift off the road before it aligns. Cap only the
    // far look-ahead; nearby arc points retain their actual distance so the
    // controller does not circle them.
    const float CurvatureLookAhead=FMath::Clamp(Distance,100.f,
        2.f*AGVReference::MinimumInnerTurnRadiusCm);
    const float RequestedCurvature=2.f*FMath::Sin(HeadingError)/CurvatureLookAhead;
    const float Curvature=FMath::Clamp(RequestedCurvature,
        -1.f/AGVReference::MinimumInnerTurnRadiusCm,1.f/AGVReference::MinimumInnerTurnRadiusCm);
    const float YawStepRadians=Travel*Curvature;
    AddActorWorldRotation(FRotator(0,FMath::RadiansToDegrees(YawStepRadians),0));
    const FVector DrivenAxis=Sensors.bReversing?-GetActorRightVector():GetActorRightVector();
    const FVector Delta=DrivenAxis.GetSafeNormal2D()*Travel;
    SetActorLocation(Previous+Delta);
    const FVector Lateral(-DrivenAxis.Y,DrivenAxis.X,0);
    Sensors.LateralSlipCm=FMath::Abs(FVector::DotProduct(Delta,Lateral));
    Sensors.SteeringAngleDegrees=FMath::RadiansToDegrees(FMath::Atan(
        AGVReference::EffectiveWheelbaseCm*Curvature));
    UpdateOdometry(Previous);
    return false;
}

void APortAGVActor::SetFMSCommand(FVector Target,int32 NodeIndex,int32 NodeCount,bool Docking,float AcceptanceCm)
{
    const bool NewNode=!Sensors.FMSNextNode.Equals(Target,.01f) || Sensors.FMSNodeIndex!=NodeIndex;
    if(NewNode)
    {
        if(Docking)
        {
            const FVector Axis=(Target-Sensors.FMSNextNode).GetSafeNormal2D();
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
            if(Reverse!=Sensors.bReversing && Speed>.1f) Speed=0;
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
    if(Hold) Speed=0;
}

void APortAGVActor::UpdateLiDARObservation(float FrontDistanceCm,float RearDistanceCm,float SafetyDistanceCm,
    bool ControlledStop,int32 BlockingVehicleID)
{
    Sensors.FrontObstacleDistanceCm=FrontDistanceCm;
    Sensors.RearObstacleDistanceCm=RearDistanceCm;
    Sensors.LiDARSafetyDistanceCm=SafetyDistanceCm;
    Sensors.LiDARBlockingVehicleID=BlockingVehicleID;
    Sensors.bLiDARClear=!ControlledStop;
    Sensors.bControlledStop=ControlledStop;
    if(ControlledStop) Speed=0;
}

void APortAGVActor::UpdateOdometry(FVector PreviousPosition)
{
    const FVector Delta=GetActorLocation()-PreviousPosition;
    Sensors.WheelOdometryCm+=Delta.Size2D();
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
    return AGVReference::AccelerationLimit(PayloadKg);
}

bool APortAGVActor::PayloadWithinReferenceLimit() const
{ return PayloadKg<=AGVReference::MaximumPayloadKg+KINDA_SMALL_NUMBER; }
