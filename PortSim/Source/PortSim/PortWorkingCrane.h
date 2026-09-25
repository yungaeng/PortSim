#pragma once

#include "CoreMinimal.h"
#include "PortEquipmentActor.h"
#include "STSOperatingProfile.h"
#include "PortWorkingCrane.generated.h"

class APortContainerActor;
class UTextRenderComponent;
class APortAGVActor;
class FJsonObject;

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
    bool AssignCargo(APortContainerActor* Cargo, FVector Source, FVector Destination, bool SourceSupport, bool DestinationSupport, APortAGVActor* HandoverVehicle=nullptr);
    void SetSTSProfile(const FSTSOperatingProfile& Profile) { STSProfile=Profile; }
    bool HasSTSProfile() const { return STSProfile.bReady; }
    double LastJobSeconds=0, LastPausedSeconds=0;
    FSTSObservation Observation;
    void SetDestinationReady(bool Ready) { bDestinationReady=Ready; }
    void SetHandoverVehicle(APortAGVActor* Vehicle);
    APortAGVActor* GetHandoverVehicle() const;
    bool IsBusy() const { return bJobActive; }
    UFUNCTION(BlueprintCallable, Category="Operation") void ResetOperation();
    UFUNCTION(BlueprintCallable, Category="Operation") void SetOperationPaused(bool Paused);
    virtual void EndPlay(const EEndPlayReason::Type Reason) override;
    bool ValidateOperation(FString& Error) const;
    FVector HeadPosition() const;
    TSharedRef<FJsonObject> DashboardState() const;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Operation") bool bEnabled=true;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Operation") bool bSTS=false;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Operation") bool bCarrying=false;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Operation") int32 CraneID=0;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Operation") int32 CompletedJobs=0;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Operation") int32 Stage=0;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Operation") FString Fault;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Operation") TObjectPtr<APortContainerActor> CargoActor;
private:
    FSTSSuspension SuspensionState;
    FSTSPickupController Pickup;
    double LockProgress[4]={0,0,0,0};
    bool PhysicalSeating[4]={false,false,false,false};
    FVector PlantAcceleration=FVector::ZeroVector;
    FTransform LockedCargoTransform;
    void AdvancePickup(float Dt);
    void SamplePickupGeometry();

    FVector SuspendedOffset=FVector::ZeroVector;
    UPROPERTY() TArray<TObjectPtr<UStaticMeshComponent>> SensorMarkers;
    void BuildSTSSensors();
    bool STSSensorContains(const FString& Key,FVector Point) const;
    FVector SpreaderVelocity() const;
    FSTSOperatingProfile STSProfile;
    UPROPERTY() TObjectPtr<APortAGVActor> HandoverAGV;
    FVector AxisVelocity=FVector::ZeroVector;
    double SimulationTime=0, NextSample=0, JobSeconds=0, PausedSeconds=0;
    bool CornerLocked[4]={false,false,false,false};
    bool bSensorFault=false;
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
    FVector Local(FVector World) const;
    bool MoveHead(FVector Target, float Dt);
    void UpdateParts();
    bool DestinationClear() const;
    void Stop(const FString& Reason);
};
