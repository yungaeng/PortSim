#pragma once
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "STSOperatingProfile.h"
#include "PortSiteLogistics.generated.h"

class APortWorkingCrane;
class APortAGVActor;
class APortContainerActor;
class UHierarchicalInstancedStaticMeshComponent;

struct FSiteYardSlot
{
    FVector Position=FVector::ZeroVector;
    int32 Block=0, Half=0, Crane=0, Color=0, Instance=INDEX_NONE;
    FVector Handover=FVector::ZeroVector;
    int32 Below=INDEX_NONE;
    bool Reserved=false, Occupied=true, Central=false;
    TWeakObjectPtr<UHierarchicalInstancedStaticMeshComponent> Mesh;
};
struct FSiteShipCargo
{
    FTransform Transform;
    TWeakObjectPtr<APortContainerActor> Actor;
    int32 STS=0, ID=0, State=0; // same actor: 0 aboard, 1 transfer, 2 yard
    uint8 HandoverMask=0; // STS->AGV, AGV->RMG, RMG->yard
    double StartedAt=0, PreparedPausedSeconds=0;
};
struct FSiteTransfer
{
    int32 Cargo=INDEX_NONE, Slot=INDEX_NONE, RMG=INDEX_NONE, STS=INDEX_NONE, Stage=0, Waypoint=0;
    double Time=0, StartedAt=0, PausedSeconds=0, HandoverAt=-1, STSSeconds=0;
    float StationaryTime=0;
    FVector LastPosition=FVector::ZeroVector;
    int32 LastStage=-1;
    TWeakObjectPtr<APortContainerActor> Actor;
    TArray<FVector> Route;
    bool bYardReleased=false;
};

/** One manifest, conserved cargo IDs and reserved yard slots across STS -> AGV -> RMG. */
UCLASS()
class PORTSIM_API APortSiteLogistics : public AActor
{
    GENERATED_BODY()
public:
    APortSiteLogistics();
    void SetSTSProfile(const FSTSOperatingProfile& Profile) { STSProfile=Profile; }
    void AddShipCargo(FVector Position,int32 STS);
    void Initialize(const TArray<TObjectPtr<APortWorkingCrane>>& Cranes,TArray<FSiteYardSlot> Slots,
        const TArray<UHierarchicalInstancedStaticMeshComponent*>& Palette,int32 CentralCargo,int32 FixedYard);
    FVector CentralSlot(int32 Index) const;
    FVector CentralHandover(int32 Index) const;
    APortWorkingCrane* CentralCrane(int32 Index) const;
    bool ReserveCentral(int32 Index);
    void CompleteCentral(int32 Index,bool Occupied);
    bool OwnsCentral(int32 Index) const { return CentralReservation==Index; }
    void RegisterBerthVehicles(const TArray<TObjectPtr<APortAGVActor>>& BerthVehicles);
    void BeginTrafficFrame();
    bool MoveVehicle(APortAGVActor* Vehicle,FVector Target,float Dt);
    void Advance(float Dt,bool Paused);
    void ResetLogistics();
    void ExportDashboard(bool Paused, bool Force=false);
    bool Validate(FString& Error) const;
    int32 ShipRemaining() const;
    int32 InTransit() const;
    bool IsIdle() const;
    APortAGVActor* LoadedVehicle(APortAGVActor* Preferred=nullptr) const;
    FString VehicleStatus() const;
    TArray<FVector> Snapshot() const;
    int32 InitialShipCount() const { return Manifest.Num()+CentralCount; }
    virtual void EndPlay(const EEndPlayReason::Type Reason) override;

    UPROPERTY(VisibleAnywhere,BlueprintReadOnly) int32 BaselineYard=0;
    UPROPERTY(VisibleAnywhere,BlueprintReadOnly) int32 InitialYard=0;
    UPROPERTY(VisibleAnywhere,BlueprintReadOnly) int32 ReceivingCapacity=0;
    UPROPERTY(VisibleAnywhere,BlueprintReadOnly) int32 QueuedHandoffs=0;
    UPROPERTY(VisibleAnywhere,BlueprintReadOnly) int32 Delivered=0;
    UPROPERTY(VisibleAnywhere,BlueprintReadOnly) int32 DispatchLimit=MAX_int32;
    UPROPERTY(VisibleAnywhere,BlueprintReadOnly) FString Fault;
    UPROPERTY() TArray<TObjectPtr<APortAGVActor>> Vehicles;
    UPROPERTY() TArray<TObjectPtr<APortAGVActor>> TrafficVehicles;
    UPROPERTY(VisibleAnywhere,BlueprintReadOnly) int32 PeakMovingVehicles=0;
    UPROPERTY(VisibleAnywhere,BlueprintReadOnly) int32 PrefetchedJobs=0;
    UPROPERTY(VisibleAnywhere,BlueprintReadOnly) TArray<TObjectPtr<APortContainerActor>> ShipContainers;
    UPROPERTY(VisibleAnywhere,BlueprintReadOnly) TArray<TObjectPtr<APortContainerActor>> PlacedContainers;
    UPROPERTY() TArray<TObjectPtr<APortWorkingCrane>> Equipment;
private:
    FSTSOperatingProfile STSProfile;
    double SimulationTime=0;
    double NextDashboardWall=0;
    double VesselStarted[3]={-1,-1,-1}, VesselUnloaded[3]={-1,-1,-1}, VesselPlaced[3]={-1,-1,-1};
    void RecordVesselEvent(int32 CargoIndex, bool Placed);
    FString ReportBase, ResultsCsv;
    bool SaveReports() const;
    void BeginReport();
    TArray<FSiteShipCargo> Manifest;
    TArray<FSiteYardSlot> Yard;
    TArray<int32> CentralSlots;
    int32 CentralReservation=INDEX_NONE, CentralPending=INDEX_NONE;
    TArray<FSiteTransfer> Jobs;
    TArray<bool> BlocksBusy;
    TArray<bool> RMGBusy;
    TArray<int32> PreparedCargo;
    TArray<int32> STSOwners;
    TArray<int32> NextVehicles;
    TArray<bool> PreparedStarted;
    TMap<int32,TArray<FBox>> RoadReservations;
    TMap<int32,FVector> RoadTargets;
    TMap<int32,int32> RoadBlockers;
    float NoProgressTime=0;
    int32 LastProgressDelivered=0;
    TSet<int32> FinishedRoadSegments;
    TArray<bool> SlotAssigned;
    int32 CentralCount=0, Dispatched=0, LaneCount=8, YardCraneCount=36;
    bool bReady=false, bWasPaused=false;
    void Dispatch(int32 Lane);
    void ScheduleFleet();
    void ActivateVehicle(int32 Vehicle,int32 STS,bool FromQueue);
    void PrepareNextCargo(int32 Lane);
    bool ReserveYard(int32 Lane);
    void PrepareRoute(int32 Lane,bool Return);
    bool Drive(int32 Lane,float Dt);
    void Freeze(bool Paused);
    void Stop(const FString& Reason);
    FVector QuayPark(int32 Lane) const;
    FVector FleetPark(int32 Vehicle) const;
    FVector YardHandover(const FSiteYardSlot& Slot) const;
};
