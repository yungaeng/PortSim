#pragma once

#include "CoreMinimal.h"
#include "PortEquipmentActor.h"
#include "PortAGVActor.generated.h"

class UStaticMeshComponent;
class UTextRenderComponent;

USTRUCT(BlueprintType)
struct FAGVSensorState
{
    GENERATED_BODY()

    UPROPERTY(VisibleAnywhere,BlueprintReadOnly,Category="AGV|Navigation") FVector EstimatedPosition=FVector::ZeroVector;
    UPROPERTY(VisibleAnywhere,BlueprintReadOnly,Category="AGV|Navigation") FVector LastAbsolutePosition=FVector::ZeroVector;
    UPROPERTY(VisibleAnywhere,BlueprintReadOnly,Category="AGV|Navigation") float PositionErrorCm=0;
    UPROPERTY(VisibleAnywhere,BlueprintReadOnly,Category="AGV|Navigation") float WheelOdometryCm=0;
    UPROPERTY(VisibleAnywhere,BlueprintReadOnly,Category="AGV|Navigation") float SteeringAngleDegrees=0;
    UPROPERTY(VisibleAnywhere,BlueprintReadOnly,Category="AGV|Navigation") float LateralSlipCm=0;
    UPROPERTY(VisibleAnywhere,BlueprintReadOnly,Category="AGV|Navigation") bool bReversing=false;
    UPROPERTY(VisibleAnywhere,BlueprintReadOnly,Category="AGV|Navigation") int32 CurrentTransponderID=INDEX_NONE;
    UPROPERTY(VisibleAnywhere,BlueprintReadOnly,Category="AGV|Navigation") bool bTransponderLocked=false;

    UPROPERTY(VisibleAnywhere,BlueprintReadOnly,Category="AGV|FMS") FVector FMSNextNode=FVector::ZeroVector;
    UPROPERTY(VisibleAnywhere,BlueprintReadOnly,Category="AGV|FMS") int32 FMSNodeIndex=0;
    UPROPERTY(VisibleAnywhere,BlueprintReadOnly,Category="AGV|FMS") int32 FMSNodeCount=0;
    UPROPERTY(VisibleAnywhere,BlueprintReadOnly,Category="AGV|FMS") bool bFMSConnected=true;
    UPROPERTY(VisibleAnywhere,BlueprintReadOnly,Category="AGV|FMS") bool bFMSHold=false;
    UPROPERTY(VisibleAnywhere,BlueprintReadOnly,Category="AGV|FMS") bool bDockingNode=false;

    UPROPERTY(VisibleAnywhere,BlueprintReadOnly,Category="AGV|Safety") float FrontObstacleDistanceCm=-1;
    UPROPERTY(VisibleAnywhere,BlueprintReadOnly,Category="AGV|Safety") float RearObstacleDistanceCm=-1;
    UPROPERTY(VisibleAnywhere,BlueprintReadOnly,Category="AGV|Safety") float LiDARSafetyDistanceCm=0;
    UPROPERTY(VisibleAnywhere,BlueprintReadOnly,Category="AGV|Safety") int32 LiDARBlockingVehicleID=INDEX_NONE;
    UPROPERTY(VisibleAnywhere,BlueprintReadOnly,Category="AGV|Safety") bool bLiDARClear=true;
    UPROPERTY(VisibleAnywhere,BlueprintReadOnly,Category="AGV|Safety") bool bControlledStop=false;
};

UCLASS(Blueprintable)
class PORTSIM_API APortAGVActor : public APortEquipmentActor
{
    GENERATED_BODY()
public:
    APortAGVActor();
    void InitializeVehicle(int32 Number);
    void ResetVehicle(FVector Position);
    bool MoveToX(float X, float Dt);
    bool MoveToPosition(FVector Target, float Dt, bool StopAtTarget=true);
    FVector PlannedMotionDirection(FVector Target) const;
    FVector CargoPosition() const;
    void SetPayload(float GrossMassKg) { PayloadKg=FMath::Max(0.f,GrossMassKg); }
    void SetFMSCommand(FVector Target,int32 NodeIndex,int32 NodeCount,bool Docking,float AcceptanceCm=-1.f);
    void SetFMSHold(bool Hold);
    void UpdateLiDARObservation(float FrontDistanceCm,float RearDistanceCm,float SafetyDistanceCm,
        bool ControlledStop,int32 BlockingVehicleID=INDEX_NONE);
    float SpeedLimitCmPerSecond() const;
    float AccelerationCmPerSecondSquared() const;
    bool PayloadWithinReferenceLimit() const;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="AGV") int32 VehicleID=1;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="AGV") float Speed=0;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="AGV") int32 CompletedJobs=0;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="AGV") float PayloadKg=0;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="AGV") FAGVSensorState Sensors;
private:
    void UpdateOdometry(FVector PreviousPosition);
    void CorrectAtTransponder(FVector Position);
    float FMSAcceptanceCm=800.f;
    FVector DockingAxis=FVector::ZeroVector;
    UPROPERTY() TObjectPtr<UTextRenderComponent> NameLabel;
};
