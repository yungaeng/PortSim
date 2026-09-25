#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "GameFramework/GameModeBase.h"
#include "GameFramework/HUD.h"
#include "TerminalActors.h"
#include "STSOperatingProfile.h"
#include "QuayCrane.generated.h"

class APortWorkingCrane;
class APortSiteLogistics;
class UStaticMeshComponent;
class UPhysicsConstraintComponent;
class USpringArmComponent;
class UCameraComponent;
class UPortSimTimeStep;
class UEngineCustomTimeStep;

enum class ETerminalStage : uint8
{
    Idle,
    FleetPrepare,
    FleetDeliver,
    RaiseEmpty,
    Approach,
    LowerPickup,
    Lift,
    Transfer,
    LowerPlace,
    Retract,
    Verify,
    Fault
};

/** Training prototype. Distances are cm; masses are kg. */
UCLASS()
class PORTSIM_API AQuayCrane : public APawn
{
    GENERATED_BODY()

public:
    AQuayCrane();

    virtual void BeginPlay() override;
    virtual void Tick(float DeltaSeconds) override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

    UFUNCTION(BlueprintCallable, Category="Crane")
    void ResetSimulation();

    UFUNCTION(BlueprintCallable, Category="Crane")
    void ToggleLock();

    UFUNCTION(BlueprintCallable, Category="Crane")
    void SetDriveInput(float Trolley, float Gantry, float Hoist);

    UFUNCTION(BlueprintCallable, Category="Simulation")
    void IncreaseSimulationSpeed();

    UFUNCTION(BlueprintCallable, Category="Simulation")
    void DecreaseSimulationSpeed();

    UFUNCTION(BlueprintCallable, Category="Simulation")
    void ResetSimulationSpeed();

    int32 GetSimulationSpeedStep() const;
    float GetActualPlaybackRate() const;

    UFUNCTION(BlueprintCallable, Category="UI")
    void ToggleHUD();

    UPROPERTY(BlueprintReadOnly, Category="UI")
    bool bHUDVisible = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Crane|Drive", meta=(ClampMin="1"))
    float TravelSpeed = 250.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Crane|Drive", meta=(ClampMin="1"))
    float Acceleration = 150.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Crane|Drive", meta=(ClampMin="1"))
    float HoistSpeed = 140.f;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Simulation")
    float SimulationSpeed = 1.0f;

    UPROPERTY(BlueprintReadOnly, Category="Crane|State")
    float RopeLength = 2400.f;

    UPROPERTY(BlueprintReadOnly, Category="Crane|State")
    bool bLocked = false;

    UPROPERTY(BlueprintReadOnly, Category="Crane|State")
    bool bEmergencyStop = false;

    UPROPERTY(BlueprintReadOnly, Category="Crane|State")
    int32 Deliveries = 0;

    UPROPERTY(BlueprintReadOnly, Category="Crane|State")
    FString Status;

    UFUNCTION(BlueprintCallable, Category="Terminal")
    void StartAutomatic(bool bLoad);

    FString GetFleetStatus() const;
    FString GetAGVStatus() const;
    int32 GetShipCargoCount() const;
    const TCHAR* GetAutoStageName() const;
    float GetSwayDegrees() const;
    float GetLoadHeight() const;
    FString GetSTSStatus() const;
    float GetCargoMassKg() const;

    bool bTerminalMode = false;
    bool bUnifiedTerminal = false;
    bool bAutoRunning = false;
    bool bAutoPaused = false;
    bool bAutoLoading = false;
    int32 AutoCompleted = 0;
    int32 ActiveCargoIndex = 0;
    double AutoElapsed = 0;

private:
    FSTSOperatingProfile STSProfile;
    FSTSObservation STSObservation;
    double STSSimulationTime = 0;
    double NextSTSSample = 0;
    bool bSTSStartPending = false;
    bool bSTSSensorFault = false;
    int32 STSLockFault = -1;
    bool STSCornerLocked[4] = {false,false,false,false};
    double JobSTSSeconds = 0, JobPrepareSeconds = 0, JobDeliverySeconds = 0, JobPausedSeconds = 0;
    double JobHandoverAt = -1, JobPlacementAt = -1;
    FString StageEventsCsv;
    void InitializeSTSProfile();
    void ApplySTSGeometry();
    void SampleSTSSensors(bool Force = false);
    bool IsCargoSupported() const;
    bool SaveSTSReports() const;
    void RecordSTSStage(const TCHAR* Outcome);
    float CurrentHoistLimit() const;
    float AutomaticHoistLimit() const;
    float STSBeamHeight() const;
    float STSTransferHeight() const;
    float STSGantrySpeed() const;
    bool STSLoadedHoistAllowed() const;
    UPROPERTY(VisibleInstanceOnly, Category="Terminal|Actors") TArray<TObjectPtr<APortContainerActor>> ContainerActors;
    UPROPERTY(VisibleInstanceOnly, Category="Terminal|Actors") TArray<TObjectPtr<APortAGVActor>> AGVActors;
    UPROPERTY(VisibleInstanceOnly, Category="Terminal|Actors") TObjectPtr<APortWorkingCrane> RMGActor;
    UPROPERTY(VisibleInstanceOnly, Category="Terminal|Actors") TObjectPtr<APortShipActor> ShipActor;
    APortContainerActor* SpawnContainer(int32 Number, FVector Position);
    bool ValidateTerminalActors(FString& Error) const;
    void DestroyTerminalActors();
    UPROPERTY()
    TObjectPtr<USceneComponent> GantryRoot;

    UPROPERTY()
    TObjectPtr<UStaticMeshComponent> TrolleyMesh;

    UPROPERTY()
    TObjectPtr<UStaticMeshComponent> Spreader;

    UPROPERTY()
    TObjectPtr<UStaticMeshComponent> Cargo;

    UPROPERTY()
    TObjectPtr<UPhysicsConstraintComponent> Suspension;

    UPROPERTY()
    TObjectPtr<UPhysicsConstraintComponent> TwistLock;

    UPROPERTY()
    TObjectPtr<USpringArmComponent> CameraArm;

    UPROPERTY()
    TObjectPtr<UCameraComponent> Camera;

    UPROPERTY()
    TArray<TObjectPtr<UStaticMeshComponent>> Ropes;

    UPROPERTY()
    TArray<TObjectPtr<UStaticMeshComponent>> CargoBodies;

    UStaticMeshComponent* MakeBox(const TCHAR* Name, USceneComponent* Parent, FVector Position, FVector Size);

    void BuildYard();
    void BuildTerminal();
    void BuildTerminalSite();
    void BuildSupportFleet();
    void TickFreeCamera(float WallDt);
    void MoveFreeCamera(FVector Input, FVector2D Look, float WallDt, bool Fast);
    void TestEquipmentAndCamera();
    bool bPickupTest=false, bPickupSawTrial=false;
    double PickupTestSeconds=0;
    FString PickupTestCase;
    UPROPERTY() TObjectPtr<APortWorkingCrane> PickupTestCrane;
    void BeginPickupTest();
    void TickPickupTest(float Dt);
    void FollowLoadedAGV();
    bool bFollowAGV=false;
    TWeakObjectPtr<APortAGVActor> FollowedAGV;
    FVector ProofVehicleStart=FVector::ZeroVector;
    bool bProofStarted=false;
    bool bFreeCamera=false;
    bool bMouseLooking=false;
    UPROPERTY() TArray<TObjectPtr<AActor>> SupportFleet;
    void TickSiteOperations(float Dt);
    void ResetSiteOperations();
    UPROPERTY() TObjectPtr<APortSiteLogistics> SiteLogistics;
    void FocusNextSiteCrane();
    int32 SiteCameraIndex=-1;
    void TickSiteTest(float Dt);
    UPROPERTY() TArray<TObjectPtr<APortWorkingCrane>> WorkingCranes;
    float SiteTestTime=0.f;
    float SiteTestHold=0.f;
    int32 SiteTestStage=0;
    TArray<FVector> SiteTestPositions;
    UPROPERTY() TObjectPtr<AActor> SiteActor;
    void ResetTerminal();
    void ApplyAppearance();
    void UpdateRopes();
    FVector TerminalSlot(int32 Index, bool bShip) const;
    void TickAutomatic(float Dt);
    void BeginAutomaticJob();
    void StopAutomatic(const FString& Reason);
    void SetAutoStage(ETerminalStage Stage);
    bool DriveSpreaderTo(FVector Target);
    bool IsTerminalSlotAvailable(int32 Index, bool bShip) const;
    void TickTerminalTest(float Dt);
    void TickSmokeTest(float Dt);
    void FinishSmokeTest(bool bSuccess, const FString& Reason);

    void BuildFleet();
    void ResetFleet();
    void TickFleet(float Dt);
    enum class EFleetDestination { Quay, Yard, Park };
    bool MoveAGV(EFleetDestination Destination, float Dt);
    FVector AGVCargoPosition() const;
    void HoldFleetCargo(bool OnAGV);
    void ReleaseFleetCargo();
    bool TickRMGTransfer(FVector Source, FVector Destination, float Dt);
    void CompleteAutomaticJob();
    int32 FleetWaypoint=0;
    bool bFleetRouteActive=false;
    TArray<FVector> FleetRoute;
    int32 ActiveAGV=0;
    int32 FleetStep=0;
    int32 RMGStep=0;
    float FleetSettle=0;
    bool bAGVHasCargo=false;
    bool bRMGHasCargo=false;
    int32 FleetPauseTest=0;
    float FleetPauseStart=0;
    FVector FleetTestAGV=FVector::ZeroVector;
    FVector FleetTestRMG=FVector::ZeroVector;
    FVector FleetTestCargo=FVector::ZeroVector;

    TArray<bool> CargoOnShip;
    TArray<int32> AutoQueue;
    ETerminalStage AutoStage = ETerminalStage::Idle;
    int32 AutoCursor = 0;
    double AutoStageTime = 0;
    float AutoStableTime = 0.f;
    double JobElapsed = 0;
    FVector AutoDriveTarget = FVector::ZeroVector;
    bool bAutoDriveTarget=false;
    FVector AutoSource = FVector::ZeroVector;
    FVector AutoDestination = FVector::ZeroVector;
    FString ResultsCsv;
    FString ResultsPath;
    bool bTerminalTest = false;
    int32 TerminalTestStage = 0;
    float TerminalTestTime = 0.f;
    float TerminalTestHold = 0.f;
    FVector TestHoldPosition = FVector::ZeroVector;
    void InstallPlaybackClock();
    void ApplyPlaybackRate();
    UPROPERTY() TObjectPtr<UPortSimTimeStep> PlaybackClock;
    UPROPERTY() TObjectPtr<UEngineCustomTimeStep> PreviousClock;
    double PreviousWallTick=0;
    FString PlaybackTracePath;
    FString PlaybackTrace;
    int32 PlaybackTraceFrame=0;
    double PlaybackTraceTime=0;
    void TickPlaybackRateTest();
    int32 PlaybackRateTestStep=-1;
    double PlaybackRateTestWall=0;
    double PlaybackRateTestSimulation=0;
    bool bPlaybackSweep=false;
    bool bHUDMouseReady = false;
    float HUDTestElapsed = 0.f;
    float CaptureElapsed = 0.f;
    bool bCaptureRequested = false;
    FVector DriveInput = FVector::ZeroVector;
    FVector DriveVelocity = FVector::ZeroVector;
    float TrolleyPosition = -1400.f;
    float GantryPosition = 0.f;
    float DeliverySettleTime = 0.f;
    bool bDelivered = false;
    bool bSmokeTest = false;
    int32 SmokeStage = 0;
    float SmokeTime = 0.f;
    float SmokeStageTime = 0.f;
    float MaxTestSway = 0.f;
};

UCLASS()
class PORTSIM_API APortSimGameMode : public AGameModeBase
{
    GENERATED_BODY()

public:
    APortSimGameMode();
};

UCLASS()
class PORTSIM_API APortSimHUD : public AHUD
{
    GENERATED_BODY()

public:
    virtual void DrawHUD() override;
    virtual void NotifyHitBoxClick(FName BoxName) override;
};
