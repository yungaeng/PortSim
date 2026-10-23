#pragma once
#include "CoreMinimal.h"
#include "AGVDynamics.h"
#include "AGVEmergency.generated.h"

USTRUCT(BlueprintType)
struct PORTSIM_API FAGVEmergencySettings
{
    GENERATED_BODY()
    UPROPERTY(EditAnywhere,BlueprintReadWrite) float WorkerDistanceM=20;
    UPROPERTY(EditAnywhere,BlueprintReadWrite) float WorkerRadiusM=.35f;
    UPROPERTY(EditAnywhere,BlueprintReadWrite) float ScanPeriodSeconds=.05f;
    UPROPERTY(EditAnywhere,BlueprintReadWrite) float ReactionSeconds=.2f;
    UPROPERTY(EditAnywhere,BlueprintReadWrite) float BrakeDemandMps2=6;
    UPROPERTY(EditAnywhere,BlueprintReadWrite) float CargoShiftLimitM=.3f;
    UPROPERTY(EditAnywhere,BlueprintReadWrite) float RestraintCapacityN=300000;
    UPROPERTY(EditAnywhere,BlueprintReadWrite) float TimeoutSeconds=60;
    // Explicit local test overrides, never a change to the terminal weather.
    UPROPERTY(EditAnywhere,BlueprintReadWrite) float Wetness=1;
    UPROPERTY(EditAnywhere,BlueprintReadWrite) float Ice=0;
    UPROPERTY(EditAnywhere,BlueprintReadWrite) FVector CargoCoGOffsetM=FVector(0,.8,.5);
    bool Valid() const;
};

// Shared by the real AGV and reproducible isolated scenarios. A worker enters
// the leading corridor at t=0 and remains there. Detection is geometric with
// scan/reaction latency, not an actual optical sensor or human behavior model.
struct PORTSIM_API FAGVEmergencyState
{
    bool Active=false,Detected=false,BrakeCommand=false,Stopped=false,Collision=false,Finished=false,Invalidated=false;
    bool CargoSecured=true;
    double Elapsed=0,ScanClock=0,DetectedAt=-1,BrakeAt=-1,StoppedAt=-1;
    double InitialSpeedMps=0,GapM=0,MinimumClearanceM=0,BrakingDistanceM=0,TravelM=0;
    double PeakSlip=0,PeakLTR=0,PeakInertiaN=0,PeakRestraintN=0,PeakCargoShiftM=0;
    FString Outcome=TEXT("idle");
    void Start(double InitialSpeed,double Gap,bool Secured);
    void Step(const FAGVEmergencySettings& Settings,const FAGVDynamicsSettings& Config,
        FAGVDynamicsInput Input,FAGVDynamicsState& D,double LeadingGapM,double LateralM,double RangeM,double Seconds);
    void Record(const FAGVDynamicsState& D,double NewGapM,double DistanceM);
    void Invalidate();
};
