#pragma once
#include "CoreMinimal.h"
class FJsonObject;

struct FSTSSensorMount
{
    FString Key,Frame,Unit;
    FVector Position=FVector::ZeroVector;
    FRotator Rotation=FRotator::ZeroRotator;
    double Minimum=0,Maximum=0,Fov=0;
    bool Scan=false;
    TArray<FVector> AdditionalPositions;
    double MeasurementOrigin=0,Resolution=0,MaxTraversingSpeed=0;
    TSharedPtr<FJsonObject> Reference;
    double ReadPosition(double PositionM) const
    { const double Raw=PositionM-MeasurementOrigin; return Resolution>0?FMath::RoundToDouble(Raw/Resolution)*Resolution:Raw; }
};

/** Explicit SI assumptions for a taut-rope, reduced-order suspension. */
struct FSTSDynamicsConfig
{
    bool AntiSway=true,AntiSkew=true;
    double Damping=0,SwayGain=0,SkewKp=0,SkewKd=0,SkewMaxTorque=0;
    double DrumRadius=0,GearRatio=0,Efficiency=0,Parts=0,MotorCount=0,MotorMaxTorque=0,MotorMaxRPM=0,MotorInertia=0;
    double RopeEA=0,RopeLimit=0,SkewLimit=0;
    FVector Wind=FVector::ZeroVector;
    double WindArea=0,WindDrag=0,WindMoment=0;
    TArray<FSTSSensorMount> Mounts;
    bool Load(TSharedPtr<FJsonObject> Root,FString& Error);
    double HorizontalAcceleration(double ErrorM,double VelocityMps,double LengthM,double AngularRate) const
    { return .05*ErrorM-1.5*VelocityMps+(AntiSway?SwayGain*LengthM*AngularRate:0); }
};

/** Two sway coordinates + yaw. Roll/pitch constrained; no elastic/slack-rope FEM. */
struct FSTSSuspension
{
    FVector2D Angle=FVector2D::ZeroVector,Rate=FVector2D::ZeroVector;
    double Yaw=0,YawRate=0,ControlTorque=0,MotorTorque=0,MotorRPM=0,MotorPower=0;
    double Tension[4]={0,0,0,0},Extension[4]={0,0,0,0};
    FVector Offset=FVector::ZeroVector,OffsetVelocity=FVector::ZeroVector;
    bool Saturated=false;
    FString Fault;
    void Step(const FSTSDynamicsConfig& C,double Dt,double Length,double LengthRate,
        FVector Acceleration,FVector DriveVelocity,double Mass,FVector CoG,double PowerW);
    double SwayDegrees() const { return FMath::RadiansToDegrees(Angle.Size()); }
};
