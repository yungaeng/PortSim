#pragma once

#include "CoreMinimal.h"
#include "STSDynamics.h"
#include "STSPickupController.h"

/** Resolved SI/reference data at the UE boundary (cm, seconds, kg). Not a motor torque model. */
struct FSTSOperatingProfile
{
    bool bReady = false;
    FString Error;
    FString ReferencePath;
    FString SettingsPath;
    FString SnapshotJson;
    TSet<FString> SensorKeys;
    float RailGauge = 0, Outreach = 0, Backreach = 0, LiftAboveRail = 0, LiftBelowRail = 0;
    float TrolleySpeed = 0, GantrySpeed = 0, EmptyHoistSpeed = 0, HoistPowerW = 0;
    float RatedPayloadKg = 0, LoadUnitKg = 0;
    TArray<FVector2D> LoadedHoistCurve; // kg, cm/s; ascending mass

    // Explicit simulation assumptions, read from STS_Simulation.json.
    float WatersideRailX = 0, BeamHeight = 0, SafeHeight = 0, MinRope = 0, MaxRope = 0;
    float TrolleyAcceleration = 0, GantryAcceleration = 0, HoistAcceleration = 0;
    float ContainerMassKg = 0, SpreaderMassKg = 0;
    FVector ContainerCoG = FVector::ZeroVector;
    float SensorPeriod = 0, SensorMaxAge = 0, PositionBias = 0, MassBiasKg = 0;
    float LandingTolerance = 0, SeatingTolerance = 0, SettleSpeed = 0, SettleTime = 0;
    float AGVTolerance = 0, AGVHeadingTolerance = 0, SupportTolerance = 0;
    float ApproachSpeed = 0, ApproachDistance = 0, StageTimeout = 0, SwayLimitDegrees = 0;
    bool bAutoStart = true;
    FSTSDynamicsConfig Dynamics;
    FSTSPickupConfig Pickup;

    bool Load(const FString& ReferenceFile, const FString& SettingsFile);
    float HoistLimit(float PayloadKg, bool bLoaded) const;
    float MinTrolley() const { return WatersideRailX - Outreach; }
    float MaxTrolley() const { return WatersideRailX + RailGauge + Backreach; }
    bool ContainsTarget(FVector SpreaderTarget) const;
};

/** Ideal/configurable synthetic observations, separate from the physical components. */
struct FSTSObservation
{
    bool bValid = false;
    double Timestamp = -1;
    FVector DrivePosition = FVector::ZeroVector; // trolley, gantry, suspended length
    FVector DriveVelocity = FVector::ZeroVector;
    FVector SpreaderPosition = FVector::ZeroVector, SpreaderVelocity = FVector::ZeroVector;
    FVector CargoPosition = FVector::ZeroVector, CargoVelocity = FVector::ZeroVector;
    float SwayDegrees = 0;
    float CornerLoadsN[4] = {0,0,0,0}; // analytic virtual load cells including acceleration; not elastic contact reactions
    bool Locked[4] = {false,false,false,false};
    bool bLanded = false, bAGVAligned = false, bCargoSupported = false;
    bool bTargetVisible=false, CornerSeated[4]={false,false,false,false};
    FVector CornerError[4]={FVector::ZeroVector,FVector::ZeroVector,FVector::ZeroVector,FVector::ZeroVector};
    FVector2D SwayRate=FVector2D::ZeroVector;
    float RelativeYawDegrees=0, TargetTiltDegrees=0, SkewDegrees=0, HoistAcceleration=0;

    bool IsFresh(double Now, float MaxAge) const { return bValid && Now >= Timestamp && Now - Timestamp <= MaxAge; }
    bool AllLocked() const { return Locked[0] && Locked[1] && Locked[2] && Locked[3]; }
    float DynamicPayloadEstimateKg() const { return (CornerLoadsN[0]+CornerLoadsN[1]+CornerLoadsN[2]+CornerLoadsN[3])/FMath::Max(1.f,9.80665f+HoistAcceleration*.01f); }
    float PayloadEstimateKg() const { return (CornerLoadsN[0]+CornerLoadsN[1]+CornerLoadsN[2]+CornerLoadsN[3])/9.80665f; }
};
