#pragma once
#include "CoreMinimal.h"
#include "WeatherFriction.h"
#include "AGVDynamics.generated.h"

// Reduced-order SI model, NOT an identified OEM vehicle. Physics axes are
// X forward, Y left, Z up. The actor uses local +Y forward / -X left.
USTRUCT(BlueprintType)
struct PORTSIM_API FAGVDynamicsSettings
{
    GENERATED_BODY()
    UPROPERTY(EditAnywhere,BlueprintReadWrite,Category="Assumptions") float WheelRadiusM=.7f;
    UPROPERTY(EditAnywhere,BlueprintReadWrite,Category="Assumptions") float WheelInertiaKgM2=400;
    UPROPERTY(EditAnywhere,BlueprintReadWrite,Category="Assumptions") float TrackM=3.4f;
    UPROPERTY(EditAnywhere,BlueprintReadWrite,Category="Assumptions") float WheelbaseM=9.6f;
    UPROPERTY(EditAnywhere,BlueprintReadWrite,Category="Assumptions") FVector EmptyCoGM=FVector(0,0,1.);
    UPROPERTY(EditAnywhere,BlueprintReadWrite,Category="Assumptions") float LongitudinalStiffnessPerLoad=12;
    UPROPERTY(EditAnywhere,BlueprintReadWrite,Category="Assumptions") float CorneringStiffnessPerLoad=4;
    UPROPERTY(EditAnywhere,BlueprintReadWrite,Category="Assumptions") float RollStiffnessNmPerRad=8000000;
    UPROPERTY(EditAnywhere,BlueprintReadWrite,Category="Assumptions") float RollDampingNmsPerRad=1600000;
    UPROPERTY(EditAnywhere,BlueprintReadWrite,Category="Assumptions") float PitchStiffnessNmPerRad=20000000;
    UPROPERTY(EditAnywhere,BlueprintReadWrite,Category="Assumptions") float PitchDampingNmsPerRad=3000000;
    UPROPERTY(EditAnywhere,BlueprintReadWrite,Category="Assumptions") float DriveTorqueEachNm=12000;
    UPROPERTY(EditAnywhere,BlueprintReadWrite,Category="Assumptions") float BrakeTorqueEachNm=50000;
    UPROPERTY(EditAnywhere,BlueprintReadWrite,Category="Control") bool bStabilitySpeedControl=true;
    UPROPERTY(EditAnywhere,BlueprintReadWrite,Category="Control") bool bTractionControl=true;
    UPROPERTY(EditAnywhere,BlueprintReadWrite,Category="Control") bool bABS=true;
    UPROPERTY(EditAnywhere,BlueprintReadWrite,Category="Control") float StabilityUtilization=.65f;
    void Sanitize();
};

struct FAGVDynamicsInput
{
    double EmptyMassKg=40000,PayloadKg=0;
    FVector CargoCoGM=FVector(0,0,3.495); // absolute cargo CoG above road, NOT container-centre offset
    double TargetSpeedMps=0,CurvaturePerM=0,AccelerationLimitMps2=1.5,BrakeDemandMps2=1.5;
    double StaticMu=.8,KineticMu=.65,RollingResistance=.012,DrivenFraction=.5;
    FVector WindForceBodyN=FVector::ZeroVector;
    // Optional planar deck contact. Disabled for ordinary rigidly attached loads.
    bool CargoContactEnabled=false,CargoSecured=true;
    double DeckStaticMu=.4,DeckKineticMu=.3,RestraintCapacityN=300000,CargoShiftLimitM=.3;
};

struct FAGVWheelDynamics
{
    double Omega=0,Angle=0,NormalN=0,FxN=0,FyN=0,TorqueNm=0,SlipRatio=0,SlipAngle=0,Steer=0;
    bool Sliding=false;
};

struct PORTSIM_API FAGVDynamicsState
{
    FVector PositionM=FVector::ZeroVector; // displacement in fixed initial vehicle frame
    double Heading=0,U=0,V=0,YawRate=0,Ax=0,Ay=0;
    double Roll=0,RollRate=0,Pitch=0,PitchRate=0,TotalMassKg=40000;
    FVector CombinedCoGM=FVector(0,0,1),IMUSpecificForceMps2=FVector(0,0,9.80665);
    FAGVWheelDynamics Wheels[6]; // rear / middle / front, left then right
    double LeftLoadN=0,RightLoadN=0,FrontLoadN=0,RearLoadN=0,LoadTransferRatio=0,MinimumRawLoadN=0;
    double EffectiveRadiusM=0,WheelDistanceM=0,GroundDistanceM=0,SideSlipDistanceM=0;
    double BrakingElapsed=0,BrakingDistanceM=0,LastBrakingSeconds=0,LastBrakingDistanceM=0;
    double MaxSlip=0,PeakSlip=0,PeakLTR=0,StabilityLimitMps=7;
    bool WheelLift=false,OverturnThreshold=false,Braking=false,LastBrakeValid=false,StabilityLimited=false;
    FString Fault;
    FVector CargoOffsetM=FVector::ZeroVector,CargoRelativeVelocityMps=FVector::ZeroVector;
    FVector CargoContactForceN=FVector::ZeroVector,CargoInertialForceN=FVector::ZeroVector,RestraintForceN=FVector::ZeroVector;
    double PeakCargoShiftM=0;
    bool CargoSliding=false,CargoUnsafe=false,RestraintOverload=false;
    void Reset(double ForwardSpeedMps=0,const FAGVDynamicsSettings& Settings=FAGVDynamicsSettings());
    void Step(const FAGVDynamicsSettings& Settings,const FAGVDynamicsInput& Input,double Seconds);
    void RefreshLoads(const FAGVDynamicsSettings& Settings,const FAGVDynamicsInput& Input);
    // Reservation/collision guards remain constraints, not measured tire braking.
    void ConstrainStop();
    bool Finite() const;
};
