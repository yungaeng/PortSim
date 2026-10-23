#pragma once
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "STSOperatingProfile.h"
#include "PortEnvironmentComponent.h"
#include "ShipStowagePlanning.h"
#include "Async/Future.h"
#include "UObject/UnrealType.h"
#include "TrafficSpatialIndex.h"
#include "PortSiteLogistics.generated.h"

class FJsonObject;
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
    int32 Cell=INDEX_NONE, LengthFt=40, Subslot=0, Partner=INDEX_NONE;
    bool Reserved=false, Occupied=true, Central=false;
    TWeakObjectPtr<UHierarchicalInstancedStaticMeshComponent> Mesh;
};
struct FSiteShipCargo
{
    bool bExplicitPlacement=false;
    FTransform Transform;
    FName ContainerName;
    FPortShipPlacement ShipSlot;
    double BookedGrossKg=0; // booking VGM proxy, before deliberate overload injection
    TWeakObjectPtr<APortContainerActor> Actor;
    int32 Wave=0;
    int32 PlannedPair=INDEX_NONE;
    int32 PlanningOrder=INDEX_NONE;
    int32 DispatchCriticalPath=0, DispatchUnlocks=0;
    double STSHandoverAt=-1, STSServiceSeconds=0;
    double STSWaitingForVehicleSeconds=0;
    double DispatchEligibleAt=-1, VehicleAssignedAt=-1;
    double EstimatedPairSavingSeconds=0;
    double PlannedPairSavingSeconds=0;
    int32 STS=0, ID=0, State=0, PlannedSlot=INDEX_NONE; // same actor: 0 aboard, 1 transfer, 2 yard
    int32 OriginalSTS=INDEX_NONE, Reassignments=0;
    FString RecoveryHold;
    uint8 HandoverMask=0; // STS->AGV, AGV->RMG, RMG->yard
    double StartedAt=0, PreparedPausedSeconds=0;
};
struct FSiteTransfer
{
    int32 ExportIndex=INDEX_NONE;
    // Stage 10 is the second STS pickup of a paired 20 ft trip.
    static constexpr int32 ToCharge=11, ReturnToStandby=12, Charging=13;
    int32 Cargo=INDEX_NONE, Slot=INDEX_NONE, RMG=INDEX_NONE, STS=INDEX_NONE, Stage=0, Waypoint=0;
    int32 ExtraCargo=INDEX_NONE;
    bool bLoadingExtra=false;
    int32 TripID=0;
    int32 TripCargoOrdinal=1;
    bool bPairedTrip=false;
    float CargoDeckOffsetCm=0;
    double Time=0, StartedAt=0, PausedSeconds=0, HandoverAt=-1, STSSeconds=0;
    float StationaryTime=0;
    FVector LastPosition=FVector::ZeroVector;
    int32 LastStage=-1;
    int32 ChargingBay=INDEX_NONE; // dedicated depot bay during ToCharge / Charging
    TWeakObjectPtr<APortContainerActor> Actor;
    TArray<FVector> Route;
    FVector RouteStart=FVector::ZeroVector;
    bool bRouteStartValid=false;
    bool bYardReleased=false, bRMGReserved=false, bParkingDeparture=false;
    double TrafficWaitSeconds=0, YardWaitSeconds=0, LastRerouteAt=-10;
    int32 Reroutes=0;
    int32 ReturnSTS=INDEX_NONE, ReturnCargo=INDEX_NONE;
    int32 YardEntryWaypoint=INDEX_NONE;
    int32 ReturnCorridor=INDEX_NONE, QuayClearWaypoint=INDEX_NONE;
    bool bEquipmentWaiting=false, bRecoveryRoutePending=false;
    bool bYardAdmissionSuspended=false;
    double EquipmentDowntimeSeconds=0;
    int32 RecoveryOriginSlot=INDEX_NONE;
    FVector RecoveryAnchor=FVector::ZeroVector;
};

/** One manifest, conserved cargo IDs and reserved yard slots across STS -> AGV -> RMG. */
UCLASS()
class PORTSIM_API APortSiteLogistics : public AActor
{
    GENERATED_BODY()
public:
    APortSiteLogistics();
    void SetSTSProfile(const FSTSOperatingProfile& Profile) { STSProfile=Profile; }
    bool CanSTSGantryMove(const APortWorkingCrane* Crane,float WorldY) const;
    void AddShipCargo(FVector Position,int32 STS,int32 Wave=0,FName ContainerName=NAME_None,bool bLoadFromYard=false,
        bool bRetainAboard=false,FIntVector ImportedSlot=FIntVector(-1));
    int32 ConfiguredShips=3;
    int32 ConfiguredCargo[3]={200,200,200};
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
    void SetWindVelocity(FVector WorldVelocityMetersPerSecond);
    void SetFogVisibilityMeters(float VisibilityMeters);
    void SetRainState(const FPortRainState& State);
    void SetSurfaceConditions(const FPortSurfaceState& State,const FPortFrictionSettings& Settings,UPhysicalMaterial* Material);
    FPortSurfaceState Surface;
    FPortFrictionSettings Friction;
    UPROPERTY() TObjectPtr<UPhysicalMaterial> SurfaceMaterial;
    int32 SurfaceActorCount=-1;
    void SetVesselMotions(const TArray<FPortVesselMotion>& Motions,const TArray<FVector>& BasePivots,const FPortSeaState& SeaState);
    bool MoveVehicle(APortAGVActor* Vehicle,FVector Target,float Dt);
    void Advance(float Dt,bool Paused);
    // Called immediately before every crane scene query inside a traffic step.
    void SynchronizeShipCollision(const FBox& Query,double ControlFraction);
    bool IsDeferringVisuals() const { return bDeferringVisuals; }
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
    int32 LoadedCount() const;
    bool UsesCargoAlignedHandover() const { return bCargoAlignedHandover; }
    int32 ReturnCorridorUse(int32 Corridor) const
    { return Corridor>=0 && Corridor<3?ReturnRouteCounts[Corridor]:0; }
    virtual void EndPlay(const EEndPlayReason::Type Reason) override;

    UPROPERTY(VisibleAnywhere,BlueprintReadOnly) int32 BaselineYard=0;
    UPROPERTY(VisibleAnywhere,BlueprintReadOnly) int32 InitialYard=0;
    UPROPERTY(VisibleAnywhere,BlueprintReadOnly) int32 ReceivingCapacity=0;
    UPROPERTY(VisibleAnywhere,BlueprintReadOnly) int32 QueuedHandoffs=0;
    UPROPERTY(VisibleAnywhere,BlueprintReadOnly) int32 Delivered=0;
    UPROPERTY(VisibleAnywhere,BlueprintReadOnly) int32 DispatchLimit=MAX_int32;
    UPROPERTY(VisibleAnywhere,BlueprintReadOnly) FString Fault;
    UPROPERTY(VisibleAnywhere,BlueprintReadOnly) int32 EquipmentFaultEvents=0;
    UPROPERTY(VisibleAnywhere,BlueprintReadOnly) int32 STSReassignments=0;
    UPROPERTY(VisibleAnywhere,BlueprintReadOnly) int32 YardReassignments=0;
    UPROPERTY() TArray<TObjectPtr<APortAGVActor>> Vehicles;
    UPROPERTY() TArray<TObjectPtr<APortAGVActor>> TrafficVehicles;
    UPROPERTY(VisibleAnywhere,BlueprintReadOnly) int32 PeakMovingVehicles=0;
    UPROPERTY(VisibleAnywhere,BlueprintReadOnly) int32 PrefetchedJobs=0;
    UPROPERTY(VisibleAnywhere,BlueprintReadOnly) int32 PairedTrips=0;
    UPROPERTY(VisibleAnywhere,BlueprintReadOnly) TArray<TObjectPtr<APortContainerActor>> ShipContainers;
    UPROPERTY(VisibleAnywhere,BlueprintReadOnly) TArray<TObjectPtr<APortContainerActor>> PlacedContainers;
    UPROPERTY() TArray<TObjectPtr<APortWorkingCrane>> Equipment;
private:
    friend class FLoadedTrafficRecoveryTest;
    friend class FApronIntersectionRecoveryTest;
    friend class FApronManifestRoutesTest;
    friend class FShipArrivalPlacementTest;
    friend class FNoEventCollisionTest;
    friend class FInlandRoadTrafficTest;
    friend class FTrafficJunctionTest;
    friend class FEquipmentRecoveryTest;
    friend class FDashboardControlTest;
    FString DashboardControlSession=FGuid::NewGuid().ToString(EGuidFormats::Digits);
    double NextControlPoll=0;
    TArray<TSharedPtr<FJsonObject>> DashboardCommandResults, RecoveryEvents;
    TSet<FString> SeenDashboardCommands;
    int32 RecoverySequence=0;
    void ProcessDashboardCommands();
    bool ApplyDashboardCommand(const FJsonObject& Command,FString& Error);
    void RecordRecoveryEvent(const FString& Kind,AActor* Cause,AActor* From,AActor* To,
        AActor* Cargo=nullptr,const FVector& OldPosition=FVector::ZeroVector,const FVector& NewPosition=FVector::ZeroVector);
    FSTSOperatingProfile STSProfile;
    double SimulationTime=0;
    double NextDashboardWall=0;
    bool bDeferringVisuals=false;
    mutable TMap<TWeakObjectPtr<UPrimitiveComponent>,FTransform> KinematicPhysicsPoses;
    double ShipWindowStart=0, ShipWindowEnd=1;
    TFuture<void> DashboardWrite;
    TMap<TWeakObjectPtr<UClass>,TArray<TFieldPath<FProperty>>> DashboardProperties;
    TUniquePtr<FJsonObject> CaptureDashboard(bool Paused);
    FTrafficSpatialIndex ReservationIndex, VehicleIndex, PendingRoadIndex;
    bool bTrafficIndexReady=false;
    void IndexTrafficVehicle(int32 Index);
    double VesselStarted[3]={-1,-1,-1}, VesselUnloaded[3]={-1,-1,-1}, VesselPlaced[3]={-1,-1,-1};
    void RecordVesselEvent(int32 CargoIndex, bool Placed);
    FString ReportBase, ResultsCsv;
    bool SaveReports() const;
    void BeginReport();
    TArray<FSiteShipCargo> Manifest;
    TArray<FSiteShipCargo> InitialManifestPlan;
    struct FDeferredYardLease { int32 Vehicle,Slot; };
    TArray<FDeferredYardLease> DeferredYardLeases;
    TSet<int32> IsolatedEquipment;
    double NextRecoveryAttempt=0;
    int32 ScheduledFailedSTS=0,ScheduledFailedRMG=0;
    double ScheduledFailureAt=60,ScheduledRepairAfter=-1;
    bool bFailureInjected=false,bFailureRepaired=false;
    bool bRecoveryRailGuard=false;
    void ManageEquipmentFailures();
    void PlanFaultEvacuations();
    bool CraneAvailable(int32 Index) const;
    bool CanSTSService(int32 STS,int32 Cargo,const FVector* DestinationOverride=nullptr) const;
    void RebuildSTSStreams();
    bool ReplanFailedYards(int32 FailedRMG=INDEX_NONE);
    bool ReplanBlockedYards();
    bool ReplanYardGroup(const TSet<int32>& Affected,const TSet<int32>& UnavailableSupports,AActor* Cause=nullptr);
    void ReleaseDeferredYardLeases();
    bool JobHeldByFailure(int32 Vehicle) const;
    // Export bookings retain their destination vessel/position separately from discharge jobs.
    TArray<FSiteShipCargo> ExportCargo;
    TArray<FSiteShipCargo> RetainedCargo;
    friend class FExportYardPlacementTest;
    bool PlaceExportCargo();
    void ScheduleExport();
    void AdvanceExport(int32 Lane,float Dt);
    FVector TransferQuay(const FSiteTransfer& Job) const;
    TArray<TArray<int32>> SourceAbove;
    TArray<TArray<int32>> ManifestBySTS;
    FPortShipStowageAssumptions ShipStowageAssumptions;
    TArray<FPortShipStowageResult> ShipStowagePlans;
    bool PlanArrivalStowage();
    void BuildSourceAccess();
    TArray<FSiteYardSlot> Yard;
    TArray<int32> CentralSlots;
    int32 CentralReservation=INDEX_NONE, CentralPending=INDEX_NONE;
    TArray<FSiteTransfer> Jobs;
    TArray<bool> BlocksBusy;
    TArray<bool> RMGBusy;
    TArray<int32> YardApproachOwners; // Own the shared RMG aisle through its outgoing vehicle's exit.
    TArray<int32> PreparedCargo;
    TArray<int32> STSOwners;
    TArray<int32> NextVehicles;
    TArray<bool> PreparedStarted;
    TMap<int32,TArray<FBox>> RoadReservations;
    TMap<int32,int32> RoadReservationEndWaypoint;
    TMap<int32,FVector> TrafficRecoveryTargets;
    TMap<int32,bool> TrafficRecoveryOriginalReverse;
    TMap<int32,int32> TrafficRecoveryYieldTo;
    double LastTrafficRecoveryAt=-100;
    TMap<int32,FVector> RoadTargets;
    TMap<int32,int32> RoadBlockers;
    TMap<int32,double> RoadWaitSince;
    TMap<int32,TArray<FBox>> PendingRoadRequests;
    void ClearPendingRoadRequest(int32 ID);
    double RoadPriorityBonus(int32 ID) const;
    bool RoadRequestBefore(int32 A,int32 B) const;
    double TrafficClock=0, LastCycleRecoveryAt=-10;
    float NoProgressTime=0;
    int32 LastProgressDelivered=0;
    TSet<int32> FinishedRoadSegments;
    TArray<bool> SlotAssigned;
    int32 CentralCount=0, Dispatched=0, LaneCount=8, YardCraneCount=46;
    int32 NextTripID=1;
    int32 ReturnRouteCounts[3]={0,0,0};
    double CentralRoadTravelCm[2][2]={{0,0},{0,0}}; // loaded/empty, east/west: actual motion, not planned routes
    bool bReady=false, bWasPaused=false;
    bool bCargoAlignedHandover=true;
    bool bDepotParking=true; // command-line A/B baseline; normal runs use depot
    bool bReuseFreedApron=true; // PreviousApron reproduces the pre-conversion depot routes
    bool bPickupRoadPriority=true; // LegacyRoadPriority reproduces FIFO for A/B runs
    int32 DispatchPriorityLimit=INDEX_NONE;
    bool UpdateDispatchPriorities();
    FVector WindVelocityMetersPerSecond=FVector::ZeroVector;
    FPortSeaState FrameSeaState;
    float FogVisibilityMeters=MAX_flt;
    FPortRainState RainState;
    TArray<FPortVesselMotion> VesselMotions;
    TArray<FVector> VesselBasePivots;
    bool ResolveVesselTransform(const FSiteShipCargo& Cargo,FVector& Position,FQuat& Rotation,FVector& Velocity,FVector& Acceleration,FVector& Jerk) const;
    void ApplyVesselMotions(float FrameDurationSeconds=0);
    void Dispatch(int32 Lane);
    void ScheduleFleet();
    void AdvanceFleetBatteries(float Dt);
    FVector ChargingBayPosition(int32 Bay) const;
    void ActivateVehicle(int32 Vehicle,int32 STS,bool FromQueue);
    void AssignReturnSTS(int32 Vehicle);
    void PrepareNextCargo(int32 Lane);
    bool SourceAccessible(int32 Cargo) const;
    void PrepareExecutableCargo(int32 Lane);
    FString YardAdmissionReason(int32 Cargo,int32 Vehicle=INDEX_NONE) const;
    bool ReserveYardApproach(int32 Lane);
    void ReleaseYardApproachIfClear(int32 Lane);
    bool YardApproachAvailable(int32 Cargo) const;
    void AdvanceStep(float Dt,bool Paused);
    bool PrepareKinematicPhysicsFrame() const;
    void RecoverTrafficCycle();
    double PendingSimulationSeconds=0;
    int32 YardApproachKey(const FSiteYardSlot& Slot) const { return LaneCount==9?Slot.Crane:Slot.Block; }
    bool ReserveYard(int32 Lane);
    bool PlanYardDestinations();
    bool PlanLegacyYardDestinations();
    void PrepareRoute(int32 Lane,bool Return);
    bool PlanRoadRoute(int32 Lane,bool Congested);
    bool Drive(int32 Lane,float Dt);
    void Freeze(bool Paused);
    void Stop(const FString& Reason);
    FVector QuayPark(int32 Lane) const;
    FVector QueueQuay(int32 STS) const;
    FVector CargoQuay(int32 CargoIndex) const;
    FVector FleetPark(int32 Vehicle) const;
    FVector YardHandover(const FSiteYardSlot& Slot) const;
};
