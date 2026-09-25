#pragma once
#include "CoreMinimal.h"

struct FSTSObservation;
class FJsonObject;

/** Simulation assumptions, not OEM twistlock or camera specifications. Units: cm, seconds, kg. */
struct FSTSPickupConfig
{
    double CornerTolerance=2, VerticalTolerance=2, RelativeSpeed=3, YawTolerance=.25, TiltTolerance=.5;
    double SeatTime=.7, LockTime=.3, LockTimeout=2, AlignmentTimeout=90, AcquisitionRadius=100;
    double TrialHeight=30, TrialHold=1, TrialTimeout=60, MinimumCornerFraction=.03, MassStability=.02;
    int32 MaxAttempts=2;
    FVector PoseBias=FVector::ZeroVector;
    bool Load(TSharedPtr<FJsonObject> Root,FString& Error);
};

enum class ESTSPickupPhase : uint8 { Align, Seat, Lock, Attach, TrialLift, TrialHold, Complete, Failed };

/** Receives measurements only: deliberately has no actor, world, or true mass/CoG access. */
struct FSTSPickupController
{
    ESTSPickupPhase Phase=ESTSPickupPhase::Align;
    FVector Target=FVector::ZeroVector, TrialOrigin=FVector::ZeroVector, EstimatedCoG=FVector::ZeroVector;
    double EstimatedMass=0, StableTime=0, PhaseTime=0, LastSample=-1, AlignmentTime=0;
    int32 Attempts=1;
    bool RequestLocks[4]={false,false,false,false};
    bool EstimateValid=false;
    FString Reason=TEXT("Acquiring target pose"), Fault;
    void Update(const FSTSPickupConfig& C,const FSTSObservation& O,double Now,double Dt,
        double MaxAge,FVector NominalCargo,double Capacity);
    void Attached(FVector MeasuredSpreader);
    const TCHAR* PhaseName() const;
private:
    void Enter(ESTSPickupPhase Next,const TCHAR* Why);
    void Fail(const TCHAR* Why);
};
