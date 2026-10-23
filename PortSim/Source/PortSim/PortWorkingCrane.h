#pragma once

#include "CoreMinimal.h"
#include "PortEquipmentActor.h"
#include "STSOperatingProfile.h"
#include "WeatherFriction.h"
#include "PortWorkingCrane.generated.h"

class APortContainerActor;
class UTextRenderComponent;
class APortAGVActor;
class FJsonObject;
class APortSiteLogistics;

UENUM(BlueprintType)
enum class EPortCraneFault : uint8 { None, SensorUnavailable, Safety, GantryUnavailable };

UENUM(BlueprintType)
enum class EPortSensorRepairMethod : uint8 { Reboot, Technician };

/** Independently reserved two-slot crane job. Gantry/trolley/hoist are driven;
 * cargo is locked to the spreader during transport and physically released. */
UCLASS(Blueprintable)
class PORTSIM_API APortWorkingCrane : public APortEquipmentActor
{
    GENERATED_BODY()
public:
    APortWorkingCrane();
    void Configure(int32 Number, bool bQuayside, FVector Source, FVector Destination, bool bCreateCargo=true);
    void Advance(float Dt, bool bGlobalPaused);
    bool AssignCargo(APortContainerActor* Cargo, FVector Source, FVector Destination, bool SourceSupport, bool DestinationSupport, APortAGVActor* HandoverVehicle=nullptr,bool ReverseTransfer=false);
    void UpdateShipLandingTarget(FVector Target);
    void SetSTSProfile(const FSTSOperatingProfile& Profile) { STSProfile=Profile; }
    void SetWindVelocity(FVector WorldVelocityMetersPerSecond);
    void SetFogVisibilityMeters(float VisibilityMeters);
    void SetRainSensorRangeScales(float CameraScale,float LiDARScale);
    FPortSurfaceState Surface;
    FPortFrictionSettings Friction;
    double RailAccelerationLimit(double RequestedCmps2) const;
    double SeatingFrictionDemandN=0, SeatingFrictionCapacityN=0;
    bool bSeatingFrictionOK=true;
    void UpdateMovingShipSource(APortContainerActor* Cargo,FVector WorldPosition);
    FVector GetWindVelocity() const { return WindVelocityMetersPerSecond; }
    bool HasSTSProfile() const { return bSTS && STSProfile.bReady; }
    bool HasRMGProfile() const { return !bSTS && STSProfile.bReady; }
    double LastJobSeconds=0, LastPausedSeconds=0;
    FSTSObservation Observation;
    void SetDestinationReady(bool Ready);
    void SetHandoverVehicle(APortAGVActor* Vehicle);
    bool SetAGVLandingTarget(FVector Target);
    bool IsWaitingForAGV() const;
    APortAGVActor* GetHandoverVehicle() const;
    bool IsBusy() const { return bJobActive; }
    UFUNCTION(BlueprintCallable,Category="Failures") void SetSensorFailure(bool Failed);
    UFUNCTION(BlueprintCallable,CallInEditor,Category="Failures") void RepairSensors() { SetSensorFailure(false); }
    UFUNCTION(BlueprintCallable,Category="Failures") void SetGantryFailure(bool Failed);
    UFUNCTION(BlueprintCallable,CallInEditor,Category="Failures") void RepairEquipment();
    bool IsSensorUnavailable() const { return FaultType==EPortCraneFault::SensorUnavailable && !Fault.IsEmpty(); }
    bool IsEquipmentUnavailable() const { return IsSensorUnavailable() || (FaultType==EPortCraneFault::GantryUnavailable && !Fault.IsEmpty()); }
    bool CanFaultEvacuate() const;
    bool RequestFaultEvacuation(float WorldY,APortSiteLogistics* Coordinator);
    double SensorRepairRemaining() const;
    bool RetainsLoad() const;
    bool CancelUnliftedJob();
    bool CanReach(FVector Source,FVector Destination) const;
    FVector AssignedDestination() const { return Slots[1-SourceSlot]; }
    FVector2D RailWorkInterval() const;
    float GantryWorldY() const;
    float GantryHomeWorldY() const { return Home.Y; }
    UFUNCTION(BlueprintCallable, Category="Operation") void ResetOperation();
    UFUNCTION(BlueprintCallable, Category="Operation") void SetOperationPaused(bool Paused);
    virtual void EndPlay(const EEndPlayReason::Type Reason) override;
    bool ValidateOperation(FString& Error) const;
    FVector HeadPosition() const;
    void DeferRopeVisuals(bool Defer);
    TSharedRef<FJsonObject> DashboardState() const;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Operation") bool bEnabled=true;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Operation") bool bSTS=false;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Operation") bool bCarrying=false;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Operation") int32 CraneID=0;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Operation") int32 CompletedJobs=0;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Operation") int32 Stage=0;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Operation") FString Fault;
    UPROPERTY(VisibleAnywhere,BlueprintReadOnly,Category="Failures") EPortCraneFault FaultType=EPortCraneFault::None;
    UPROPERTY(EditAnywhere,BlueprintReadWrite,Category="Failures") bool bSensorFault=false;
    UPROPERTY(EditAnywhere,BlueprintReadWrite,Category="Failures") bool bGantryFault=false;
    UPROPERTY(EditAnywhere,BlueprintReadWrite,Category="Failures|Repair") bool bAutoSensorRecovery=true;
    UPROPERTY(EditAnywhere,BlueprintReadWrite,Category="Failures|Repair") EPortSensorRepairMethod SensorRepairMethod=EPortSensorRepairMethod::Reboot;
    // Simulation assumptions, not manufacturer maintenance-time specifications.
    UPROPERTY(EditAnywhere,BlueprintReadWrite,Category="Failures|Repair",meta=(ClampMin="0",ClampMax="86400")) double SensorRebootSeconds=60;
    UPROPERTY(EditAnywhere,BlueprintReadWrite,Category="Failures|Repair",meta=(ClampMin="0",ClampMax="86400")) double SensorTechnicianRepairSeconds=900;
    UPROPERTY(EditAnywhere,BlueprintReadWrite,Category="Failures|Repair",meta=(ClampMin="0",ClampMax="60")) double SensorSelfCheckSeconds=2;
    UPROPERTY(EditAnywhere,BlueprintReadWrite,Category="Failures|Recovery",meta=(ToolTip="Only automation/perception faults independent of the travel encoder, emergency stop and anti-collision system may permit evacuation.")) bool bAllowSensorFaultEvacuation=false;
    UPROPERTY(EditAnywhere,BlueprintReadWrite,Category="Failures|Recovery") bool bIndependentTravelSafetyHealthy=true;
    UPROPERTY(VisibleAnywhere,BlueprintReadOnly,Category="Failures|Recovery") bool bFaultEvacuating=false;
    UPROPERTY(VisibleAnywhere,BlueprintReadOnly,Category="Failures|Recovery") bool bFaultEvacuationComplete=false;
    UPROPERTY(VisibleAnywhere,BlueprintReadOnly,Category="Failures|Recovery") float FaultEvacuationTargetY=0;
    UPROPERTY(VisibleAnywhere,BlueprintReadOnly,Category="Failures|Repair") double SensorRepairElapsed=0;
    UPROPERTY(VisibleAnywhere,BlueprintReadOnly,Category="Failures|Repair") FString SensorRepairPhase;
    UPROPERTY(VisibleAnywhere,BlueprintReadOnly,Category="Failures") FString RecoveryStatus;
    UPROPERTY(VisibleAnywhere,BlueprintReadOnly,Category="Failures") bool bRailSafetyHold=false;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Operation") TObjectPtr<APortContainerActor> CargoActor;
private:
    friend class FEquipmentRecoveryTest;
    bool bReverseTransfer=false;
    bool AGVAtDestination() const { return bSTS!=bReverseTransfer; }
    bool bDeferRopeVisuals=false, bRopeVisualsDirty=false;
    void UpdateRopeVisuals();
    double ShipControlFraction=1;
    bool bDeferParts=false, bPartsDirty=false;
    void FlushParts();
    void SynchronizeShipQuery(const FBox& Query) const;
    void AdvanceStep(float Dt, bool bGlobalPaused);
    FSTSSuspension SuspensionState;
    FSTSPickupController Pickup;
    double LockProgress[4]={0,0,0,0};
    bool PhysicalSeating[4]={false,false,false,false};
    FVector PlantAcceleration=FVector::ZeroVector;
    FTransform LockedCargoTransform;
    void AdvancePickup(float Dt);
    void SamplePickupGeometry();
    void SampleRMGEnvironment();

    FVector SuspendedOffset=FVector::ZeroVector;
    UPROPERTY() TArray<TObjectPtr<UStaticMeshComponent>> SensorMarkers;
    void BuildSTSSensors();
    void AdvanceTelescope(float Dt);
    void UpdateSpreaderGeometry();
    void UpdateSpreaderSensors();
    FVector DynamicSpreaderMount(const FSTSSensorMount& Mount, FVector ConfiguredPosition) const;
    bool TelescopeReady() const;
    bool STSSensorContains(const FString& Key,FVector Point) const;
    FVector SpreaderVelocity() const;
    FSTSOperatingProfile STSProfile;
    UPROPERTY() TObjectPtr<APortAGVActor> HandoverAGV;
    FVector AxisVelocity=FVector::ZeroVector;
    FVector WindVelocityMetersPerSecond=FVector::ZeroVector;
    float FogVisibilityMeters=MAX_flt;
    float RainCameraRangeScale=1.f, RainLiDARRangeScale=1.f;
    double SimulationTime=0, NextSample=0, JobSeconds=0, PausedSeconds=0;
    double AGVAlignmentWait=0;
    bool CornerLocked[4]={false,false,false,false};
    bool bAppliedSensorFault=false, bFatalSensorTest=false;
    bool bAppliedGantryFault=false;
    TWeakObjectPtr<APortSiteLogistics> RecoveryCoordinator;
    void AdvanceFailureRecovery(float Dt,bool GlobalPaused);
    bool CanServiceTravel() const;
    FString SensorInvalidReason;
    int32 LockFault=INDEX_NONE;
    void SampleSTS(bool Force=false);
    bool MoveSTS(FVector Target,float Dt);
    bool CargoSupported() const;
    bool AGVAligned() const;
    void ClearSTSState();
    UPROPERTY() TArray<TObjectPtr<UStaticMeshComponent>> Legs;
    UPROPERTY() TArray<TObjectPtr<UStaticMeshComponent>> Beams;
    UPROPERTY() TArray<TObjectPtr<UStaticMeshComponent>> Bogies;
    UPROPERTY() TArray<TObjectPtr<UStaticMeshComponent>> CrossBeams;
    UPROPERTY() TArray<TObjectPtr<UStaticMeshComponent>> Ropes;
    UPROPERTY() TArray<TObjectPtr<UStaticMeshComponent>> TelescopeArms;
    UPROPERTY() TArray<TObjectPtr<UStaticMeshComponent>> SpreaderEndBeams;
    UPROPERTY() TArray<TObjectPtr<UStaticMeshComponent>> TwistLocks;
    UPROPERTY() TObjectPtr<UStaticMeshComponent> SpreaderCenter;
    UPROPERTY() TArray<TObjectPtr<UStaticMeshComponent>> SpreaderSensorInstances;
    TArray<int32> SpreaderSensorMountIndices;
    TArray<FVector> SpreaderSensorConfiguredPositions;
    UPROPERTY() TArray<TObjectPtr<UStaticMeshComponent>> Pads;
    UPROPERTY() TObjectPtr<UStaticMeshComponent> Trolley;
    UPROPERTY() TObjectPtr<UStaticMeshComponent> Spreader;
    UPROPERTY() TObjectPtr<UStaticMeshComponent> Mast;
    UPROPERTY() TObjectPtr<UTextRenderComponent> NameLabel;
    FVector Home=FVector::ZeroVector;
    FQuat Orientation=FQuat::Identity;
    FVector Slots[2];
    FVector Head=FVector::ZeroVector;
    FVector PausedVelocity=FVector::ZeroVector;
    FVector PausedAngularVelocity=FVector::ZeroVector;
    float BeamZ=2300.f;
    float SafeZ=1900.f;
    float Speed=0.f;
    float TelescopeLengthCm=1219.2f;
    float TelescopeTargetLengthCm=1219.2f;
    float SettleTime=0.f;
    float StageTime=0.f;
    int32 SourceSlot=0;
    bool bPaused=false;
    bool bResumePhysics=false;
    bool bConfigured=false;
    bool bExternalJobs=false;
    bool bJobActive=false;
    bool bDestinationReady=true;
    FVector JobStartHead=FVector::ZeroVector;
    FVector StageHoldTarget=FVector::ZeroVector;
    FVector Local(FVector World) const;
    bool MoveHead(FVector Target, float Dt);
    void UpdateParts();
    bool DestinationClear() const;
    void Stop(const FString& Reason,EPortCraneFault Kind=EPortCraneFault::Safety);
};
