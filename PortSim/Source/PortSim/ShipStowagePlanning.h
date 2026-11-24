#pragma once
#include "CoreMinimal.h"
#include "CargoInformation.h"

struct FPortShipStowageAssumptions
{
    FString Basis=TEXT("simulation_assumptions_not_vessel_approved");
    FString SnapshotJson;
    double LightshipKg=30000000, LightshipKGM=9, KMM=12, FreeSurfaceMomentKgM=3000000;
    double MinimumGMM=.5, MaximumGMM=4.5, MaximumHeelDegrees=5;
    double MaximumStackKg=150000, MaximumLashingN=200000, LateralAccelerationMps2=1.47;
    double DeckHeightM=12;
    double BayPitchM=18, RowPitchM=3, ContainerHeightM=2.591, Twin20PitchM=6.2;
    double MaximumLongitudinalCGM=5;
    bool UseDeckZones=false;
    int32 Bays=10, Rows=8, Tiers=8;
    bool IsEndBay(int32 Bay) const { return Bay<(Bays>=20?2:1) || Bay>=Bays-(Bays>=20?2:1); }
    bool IsPoweredRow(int32 Row) const { return Rows>=16?FMath::Abs(Row-(Rows-1)*.5)<=1.5:Row==Rows/2 || Row==(Rows-1)/2; }
    bool IsFlammableBay(int32 Bay) const { return Bay>=Bays-(Bays>=20?2:1); }
    bool IsCorrosiveBay(int32 Bay) const { return Bay<(Bays>=20?2:1); }
    bool Load(const FString& Path,FString& Error);
};
struct FPortShipPlanningCargo
{
    int32 ID=0, LengthFt=40;
    double GrossKg=0;
    FPortCargoInformation Information;
    FVector CoGOffsetM=FVector::ZeroVector; // ship axes: X longitudinal, Y transverse, Z vertical.
};
struct FPortShipPlacement
{
    int32 CargoID=0, Bay=0, Row=0, Tier=0;
    double CorrectedGMM=0, HeelDegrees=0, StackKg=0, LashingN=0;
    int32 Subslot=0, BelowCargoID=INDEX_NONE;
    FVector PositionM=FVector::ZeroVector; // longitudinal, transverse, vessel model Z datum
};
struct FPortShipStowageResult
{
    bool Feasible=false;
    FString Reason;
    TArray<FPortShipPlacement> Placements;
    double CorrectedGMM=0, HeelDegrees=0;
    double LongitudinalCGM=0;
};

// Shared by initial physical arrival placement and the dashboard. Explicit
// simulation cells/constant KM/static lashing proxy, not vessel approval.
FPortShipStowageResult PlanShipStowage(const TArray<FPortShipPlanningCargo>& Cargo,
    const FPortShipStowageAssumptions& A);
