#include "QuayCrane.h"
#include "PortSiteLogistics.h"
#include "PortSimTimeStep.h"
#include "Misc/FileHelper.h"
#include "HAL/PlatformTime.h"
#include "HAL/PlatformProcess.h"
#include "Camera/CameraComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Components/DirectionalLightComponent.h"
#include "Components/SkyLightComponent.h"
#include "Components/SkyAtmosphereComponent.h"
#include "Engine/Canvas.h"
#include "Engine/DirectionalLight.h"
#include "Engine/Engine.h"
#include "Engine/SkyLight.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "Materials/MaterialInterface.h"
#include "HighResScreenshot.h"
#include "UnrealClient.h"
#include "GameFramework/PlayerController.h"
#include "GameFramework/SpringArmComponent.h"
#include "GameFramework/WorldSettings.h"
#include "InputCoreTypes.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "PhysicsEngine/PhysicsConstraintComponent.h"
#include "UObject/ConstructorHelpers.h"

DEFINE_LOG_CATEGORY_STATIC(LogPortSimCrane, Log, All);

namespace Crane
{
    constexpr float SpeedLevels[] = {1.f, 2.f, 4.f, 8.f, 16.f};
    constexpr int32 SpeedLevelCount = UE_ARRAY_COUNT(SpeedLevels);
    constexpr float BeamHeight = 3000.f;
    constexpr float MinRope = 400.f;
    constexpr float MaxRope = 2800.f;
    constexpr float CargoHalfHeight = 129.5f;
    constexpr float SpreaderHalfHeight = 25.f;
    const FVector CargoStart(-1400.f, 0.f, 150.f);
}

UStaticMeshComponent* AQuayCrane::MakeBox(const TCHAR* Name, USceneComponent* Parent, FVector Position, FVector Size)
{
    static ConstructorHelpers::FObjectFinder<UStaticMesh> Cube(TEXT("/Engine/BasicShapes/Cube.Cube"));
    UStaticMeshComponent* Mesh = CreateDefaultSubobject<UStaticMeshComponent>(Name);
    Mesh->SetupAttachment(Parent);
    Mesh->SetStaticMesh(Cube.Object);
    Mesh->SetRelativeLocation(Position);
    Mesh->SetRelativeScale3D(Size / 100.f);
    Mesh->SetCollisionProfileName(TEXT("BlockAllDynamic"));
    return Mesh;
}

AQuayCrane::AQuayCrane()
{
    PrimaryActorTick.bCanEverTick = true;
    PrimaryActorTick.TickGroup = TG_PrePhysics;
    RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
    GantryRoot = CreateDefaultSubobject<USceneComponent>(TEXT("Gantry"));
    GantryRoot->SetupAttachment(RootComponent);
    for (int32 X = -1; X <= 1; X += 2)
    {
        for (int32 Y = -1; Y <= 1; Y += 2)
        {
            MakeBox(*FString::Printf(TEXT("Leg_%d_%d"), X, Y), GantryRoot,
                FVector(X * 850.f, Y * 1000.f, 1450.f), FVector(100.f, 100.f, 2900.f));
            MakeBox(*FString::Printf(TEXT("Bogie_%d_%d"), X, Y), GantryRoot,
                FVector(X * 850.f, Y * 1000.f, 100.f), FVector(180.f, 420.f, 180.f));
        }
        MakeBox(*FString::Printf(TEXT("Boom_%d"), X), GantryRoot,
            FVector(0.f, X * 730.f, 3100.f), FVector(9000.f, 100.f, 180.f));
        MakeBox(*FString::Printf(TEXT("CrossBeam_%d"), X), GantryRoot,
            FVector(X * 850.f, 0.f, 2900.f), FVector(140.f, 2200.f, 140.f));
    }
    TrolleyMesh = MakeBox(TEXT("Trolley"), GantryRoot, FVector(-1400.f, 0.f, Crane::BeamHeight), FVector(360.f, 1450.f, 100.f));
    Spreader = MakeBox(TEXT("Spreader"), RootComponent, FVector(-1400.f, 0.f, 600.f), FVector(244.f, 1220.f, 50.f));
    for (UStaticMeshComponent* Body : {Spreader.Get()})
    {
        Body->SetSimulatePhysics(true);
        Body->SetLinearDamping(0.4f);
        Body->SetAngularDamping(2.f);
        Body->BodyInstance.bUseCCD = true;
        Body->BodyInstance.PositionSolverIterationCount = 16;
        Body->BodyInstance.VelocitySolverIterationCount = 8;
    }
    Suspension = CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("Suspension"));
    Suspension->SetupAttachment(RootComponent);
    Suspension->SetLinearXLimit(LCM_Limited, RopeLength);
    Suspension->SetLinearYLimit(LCM_Limited, RopeLength);
    Suspension->SetLinearZLimit(LCM_Limited, RopeLength);
    // Equivalent suspension: lateral swing is free, spreader orientation stays level.
    Suspension->SetAngularSwing1Limit(ACM_Locked, 0.f);
    Suspension->SetAngularSwing2Limit(ACM_Locked, 0.f);
    Suspension->SetAngularTwistLimit(ACM_Locked, 0.f);
    Suspension->SetDisableCollision(true);
    Suspension->ConstraintInstance.bScaleLinearLimits = false;
    TwistLock = CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("TwistLock"));
    TwistLock->SetupAttachment(RootComponent);
    TwistLock->SetLinearXLimit(LCM_Locked, 0.f);
    TwistLock->SetLinearYLimit(LCM_Locked, 0.f);
    TwistLock->SetLinearZLimit(LCM_Locked, 0.f);
    TwistLock->SetAngularSwing1Limit(ACM_Locked, 0.f);
    TwistLock->SetAngularSwing2Limit(ACM_Locked, 0.f);
    TwistLock->SetAngularTwistLimit(ACM_Locked, 0.f);
    TwistLock->SetDisableCollision(true);
    for (int32 Index = 0; Index < 4; ++Index)
    {
        auto* Rope = MakeBox(*FString::Printf(TEXT("Rope_%d"), Index), RootComponent, FVector::ZeroVector, FVector(8.f));
        Rope->SetCollisionEnabled(ECollisionEnabled::NoCollision);
        Ropes.Add(Rope);
    }
    CameraArm = CreateDefaultSubobject<USpringArmComponent>(TEXT("CameraArm"));
    CameraArm->SetupAttachment(RootComponent);
    CameraArm->SetRelativeLocation(FVector(0.f, 0.f, 1300.f));
    CameraArm->SetRelativeRotation(FRotator(-24.f, -48.f, 0.f));
    CameraArm->TargetArmLength = 7600.f;
    CameraArm->bDoCollisionTest = false;
    Camera = CreateDefaultSubobject<UCameraComponent>(TEXT("Camera"));
    Camera->SetupAttachment(CameraArm);
    Camera->SetFieldOfView(65.f);
    Camera->PostProcessSettings.bOverride_AutoExposureBias=true;
    Camera->PostProcessSettings.AutoExposureBias=-1.5f;
}

void AQuayCrane::BuildYard()
{
    auto Box = [this](FName Name, FVector Position, FVector Size)
    {
        auto* Mesh = NewObject<UStaticMeshComponent>(this, Name);
        AddInstanceComponent(Mesh);
        Mesh->SetStaticMesh(TrolleyMesh->GetStaticMesh());
        Mesh->SetMobility(EComponentMobility::Static);
        Mesh->SetCollisionProfileName(TEXT("BlockAll"));
        Mesh->SetWorldLocation(Position);
        Mesh->SetWorldScale3D(Size / 100.f);
        Mesh->RegisterComponent();
    };
    if (bTerminalMode) BuildTerminal();
    else
    {
    Box(TEXT("Quay"), FVector(0.f, 0.f, -60.f), FVector(10000.f, 11000.f, 100.f));
    Box(TEXT("PickupPad"), FVector(-1400.f, 0.f, 5.f), FVector(800.f, 1700.f, 30.f));
    Box(TEXT("DeliveryPad"), FVector(1400.f, 0.f, 5.f), FVector(800.f, 1700.f, 30.f));
    Box(TEXT("RailLeft"), FVector(-850.f, 0.f, 0.f), FVector(35.f, 7500.f, 20.f));
    Box(TEXT("RailRight"), FVector(850.f, 0.f, 0.f), FVector(35.f, 7500.f, 20.f));
    }
    auto* Sun = GetWorld()->SpawnActor<ADirectionalLight>(FVector(0.f, 0.f, 6000.f), FRotator(-50.f, -35.f, 0.f));
    Sun->GetLightComponent()->SetMobility(EComponentMobility::Movable);
    Sun->GetLightComponent()->SetIntensity(5.f);
    CastChecked<UDirectionalLightComponent>(Sun->GetLightComponent())->SetAtmosphereSunLight(true);
    auto* Atmosphere = NewObject<USkyAtmosphereComponent>(this, TEXT("Atmosphere"));
    AddInstanceComponent(Atmosphere);
    Atmosphere->RegisterComponent();
    auto* Sky = GetWorld()->SpawnActor<ASkyLight>();
    Sky->GetLightComponent()->SetMobility(EComponentMobility::Movable);
    Sky->GetLightComponent()->SetIntensity(1.f);
    Sky->GetLightComponent()->SetRealTimeCapture(true);
    Sky->GetLightComponent()->SetLowerHemisphereColor(FLinearColor(0.25f, 0.3f, 0.4f));
}


void AQuayCrane::ApplyAppearance()
{
    auto Material = [](const TCHAR* Name) {
        return LoadObject<UMaterialInterface>(nullptr, *FString::Printf(TEXT("/Game/PortSim/Assets/Materials/M_%s.M_%s"), Name, Name));
    };
    UMaterialInterface* Yellow = Material(TEXT("CraneYellow"));
    UMaterialInterface* Steel = Material(TEXT("Steel"));
    UMaterialInterface* QuayMaterial = Material(bTerminalMode ? TEXT("SiteAsphalt") : TEXT("Quay"));
    UMaterialInterface* PickupMaterial = Material(TEXT("Pickup"));
    UMaterialInterface* TargetMaterial = Material(TEXT("Target"));
    TArray<UStaticMeshComponent*> Meshes;
    GetComponents<UStaticMeshComponent>(Meshes);
    auto AddEquipmentMeshes=[&Meshes](AActor* Actor)
    {
        if (!Actor) return;
        TArray<UStaticMeshComponent*> Parts;
        Actor->GetComponents<UStaticMeshComponent>(Parts);
        Meshes.Append(Parts);
    };
    for (const auto& Vehicle:AGVActors) AddEquipmentMeshes(Vehicle);
    AddEquipmentMeshes(ShipActor);
    for (auto* Mesh : Meshes)
    {
        const FString Name = Mesh->GetName();
        auto* Mat = Yellow;
        if (Name.StartsWith(TEXT("Rope")) || Name.StartsWith(TEXT("Rail")) || Name.StartsWith(TEXT("Bogie"))) Mat = Steel;
        if (Name == TEXT("Quay")) Mat = QuayMaterial;
        if (Name == TEXT("PickupPad")) Mat = PickupMaterial;
        if (Name.StartsWith(TEXT("DeliveryPad"))) Mat = TargetMaterial;
        if (Name.StartsWith(TEXT("Ship"))) Mat = Steel;
        if (Name == TEXT("ShipDeck") || Name == TEXT("ShipBridge")) Mat = QuayMaterial;
        if (Name == TEXT("Water")) Mat = Material(TEXT("SiteWater"));
        if (Name.StartsWith(TEXT("Road_AGV"))) Mat = Steel;
        if (Name.StartsWith(TEXT("AGV_"))) Mat = PickupMaterial;
        if (Name.StartsWith(TEXT("RMG_"))) Mat = TargetMaterial;
        if (Mat) Mesh->SetMaterial(0, Mat);
    }
}

void AQuayCrane::BeginPlay()
{
    Super::BeginPlay();
    bHUDVisible=!FParse::Param(FCommandLine::Get(),TEXT("PortSimHUDHidden"));
    // Demo always starts at a known world origin, regardless of PlayerStart placement.
    SetActorLocationAndRotation(FVector::ZeroVector, FRotator::ZeroRotator);
    Spreader->SetMassOverrideInKg(NAME_None, 5000.f);
    bSmokeTest = FParse::Param(FCommandLine::Get(), TEXT("PortSimSmokeTest"));
    bTerminalTest = FParse::Param(FCommandLine::Get(), TEXT("PortSimTerminalTest")) || FParse::Param(FCommandLine::Get(),TEXT("PortSimFleetResetTest"));
    bTerminalMode = !bSmokeTest;
    bUnifiedTerminal=bTerminalMode && !bTerminalTest && !FParse::Param(FCommandLine::Get(),TEXT("PortSimLegacyTerminal"));
    InitializeSTSProfile();
    if(FParse::Param(FCommandLine::Get(),TEXT("PortSimPickupTest"))){BeginPickupTest();return;}
    if (FParse::Param(FCommandLine::Get(),TEXT("PortSimSpeedTest")))
    {
        bool Pass=true;
        ResetSimulationSpeed();
        for (int32 I=0;I<Crane::SpeedLevelCount;++I)
        {
            Pass &= SimulationSpeed==Crane::SpeedLevels[I] && GetSimulationSpeedStep()==I;
            IncreaseSimulationSpeed();
        }
        Pass &= SimulationSpeed==16.f;
        for (int32 I=Crane::SpeedLevelCount-1;I>=0;--I)
        {
            Pass &= SimulationSpeed==Crane::SpeedLevels[I];
            DecreaseSimulationSpeed();
        }
        Pass &= SimulationSpeed==1.f;
        IncreaseSimulationSpeed(); ResetSimulationSpeed(); Pass &= SimulationSpeed==1.f;
        UE_LOG(LogPortSimCrane,Display,TEXT("PORTSIM_SPEED_%s: 1,2,4,8,16; upper/lower bounds and reset"),Pass?TEXT("PASS"):TEXT("FAIL"));
        FPlatformMisc::RequestExitWithStatus(false,Pass?0:1);
    }
    if (!bTerminalMode) Cargo=SpawnContainer(1,Crane::CargoStart)->GetBody();
#if WITH_EDITOR
    SetActorLabel(bUnifiedTerminal?TEXT("Terminal_Camera_Controller"):TEXT("STS_01"));
    SetFolderPath(TEXT("PortSim/Equipment"));
#endif
    Tags.AddUnique(bUnifiedTerminal?TEXT("PortSim.TerminalController"):TEXT("PortSim.STS"));
    BuildYard();
    ApplyAppearance();
    ResetSimulation();
    if (FParse::Param(FCommandLine::Get(),TEXT("PortSimActorTest")))
    {
        FString Error;
        bool Pass=ValidateTerminalActors(Error);
        ResetSimulation();
        Pass=ValidateTerminalActors(Error) && Pass;
        UE_LOG(LogPortSimCrane,Display,TEXT("PORTSIM_ACTORS_%s: %s"),Pass?TEXT("PASS"):TEXT("FAIL"),
            Pass?TEXT("Terminal actor ownership, vessel inventory, equipment counts and reset verified"):*Error);
        FPlatformMisc::RequestExitWithStatus(false,Pass?0:1);
    }
    int32 FocusIndex=-1;
    if (bTerminalMode && FParse::Value(FCommandLine::Get(),TEXT("PortSimSiteFocus="),FocusIndex) && FocusIndex>=0 && FocusIndex<WorkingCranes.Num())
    { SiteCameraIndex=FocusIndex-1; FocusNextSiteCrane(); }
    float RequestedPlayback=1.f;
    const bool ExplicitPlayback=FParse::Value(FCommandLine::Get(),TEXT("PortSimPlayback="),RequestedPlayback);
    if ((!bSmokeTest && !bTerminalTest && !FParse::Param(FCommandLine::Get(),TEXT("PortSimSiteTest")) && !FParse::Param(FCommandLine::Get(),TEXT("PortSimFullUnloadTest"))) || ExplicitPlayback)
    {
        InstallPlaybackClock();
        while (SimulationSpeed<RequestedPlayback && GetSimulationSpeedStep()<Crane::SpeedLevelCount-1) IncreaseSimulationSpeed();
    }
    FParse::Value(FCommandLine::Get(),TEXT("PortSimTrace="),PlaybackTracePath);
    bPlaybackSweep=FParse::Param(FCommandLine::Get(),TEXT("PortSimPlaybackSweep"));
    PreviousWallTick=FPlatformTime::Seconds();
    PlaybackTrace=TEXT("Frame,Time,TrolleyX,GantryY,Rope,SpreaderX,SpreaderY,SpreaderZ,CargoX,CargoY,CargoZ,Sway,Locked,Stage,Deliveries\n");
    bSmokeTest = FParse::Param(FCommandLine::Get(), TEXT("PortSimSmokeTest"));
    UE_LOG(LogPortSimCrane, Display, TEXT("Crane ready: gantry, trolley, hoist, Chaos suspension and twist lock."));
}

void AQuayCrane::ResetSimulation()
{
    if (bTerminalMode) { ResetTerminal(); return; }
    Suspension->BreakConstraint();
    TwistLock->BreakConstraint();
    bLocked = false;
    bEmergencyStop = false;
    bDelivered = false;
    Deliveries = 0;
    DeliverySettleTime = 0.f;
    DriveInput = DriveVelocity = FVector::ZeroVector;
    TrolleyPosition = -1400.f;
    GantryPosition = 0.f;
    RopeLength = 2400.f;
    GantryRoot->SetRelativeLocation(FVector::ZeroVector);
    TrolleyMesh->SetRelativeLocation(FVector(TrolleyPosition, 0.f, Crane::BeamHeight));
    for (UStaticMeshComponent* Body : {Spreader.Get(), Cargo.Get()})
    {
        Body->SetPhysicsLinearVelocity(FVector::ZeroVector);
        Body->SetPhysicsAngularVelocityInDegrees(FVector::ZeroVector);
        Body->SetWorldLocationAndRotation(Body == Cargo ? Crane::CargoStart : FVector(-1400.f, 0.f, 600.f),
            FRotator::ZeroRotator, false, nullptr, ETeleportType::TeleportPhysics);
        Body->WakeAllRigidBodies();
    }
    Suspension->SetWorldLocation(TrolleyMesh->GetComponentLocation());
    Suspension->SetConstrainedComponents(TrolleyMesh, NAME_None, Spreader, NAME_None);
    // Both local anchors are their body's centre. A spherical maximum distance acts as the rope.
    Suspension->SetConstraintReferencePosition(EConstraintFrame::Frame1, FVector::ZeroVector);
    Suspension->SetConstraintReferencePosition(EConstraintFrame::Frame2, FVector::ZeroVector);
    Suspension->ConstraintInstance.SetLinearLimitSize(RopeLength);
    Status = TEXT("Lower spreader onto the left container, then press F to lock.");
    UpdateRopes();
}

void AQuayCrane::SetDriveInput(float Trolley, float Gantry, float Hoist)
{
    DriveInput = FVector(FMath::Clamp(Trolley, -1.f, 1.f), FMath::Clamp(Gantry, -1.f, 1.f), FMath::Clamp(Hoist, -1.f, 1.f));
}

void AQuayCrane::ToggleLock()
{
    if (bUnifiedTerminal || !Cargo) return;
    if(STSProfile.bReady)
    {
        SampleSTSSensors(true);
        if(!STSObservation.IsFresh(STSSimulationTime,STSProfile.SensorMaxAge))
        { Status=TEXT("Lock command denied: sensor data invalid."); return; }
        if(bLocked && (!STSObservation.bCargoSupported || STSObservation.CargoVelocity.Size()>STSProfile.SettleSpeed))
        { Status=TEXT("Unlock denied: cargo must be supported and settled."); return; }
        if(!bLocked && (!STSObservation.bLanded || STSLockFault>=0 || GetCargoMassKg()>STSProfile.RatedPayloadKg))
        { Status=TEXT("Lock denied: seating / corner lock fault / payload limit."); return; }
    }
    if (bLocked)
    {
        TwistLock->BreakConstraint();
        bLocked = false;
        for(bool& Locked:STSCornerLocked) Locked=false;
        SampleSTSSensors(true);
        Cargo->WakeAllRigidBodies();
        Status = bTerminalMode ? TEXT("Twist lock released. Verifying the destination slot.") : TEXT("Unlocked. Place cargo on the right pad and let it settle.");
        return;
    }
    if (bTerminalMode && !bAutoRunning)
    {
        float Best = TNumericLimits<float>::Max();
        for (int32 I=0;I<CargoBodies.Num();++I)
        {
            const float Distance=FVector::DistSquared(Spreader->GetComponentLocation(),CargoBodies[I]->GetComponentLocation()+FVector(0,0,154.5f));
            if (Distance<Best) { Best=Distance; Cargo=CargoBodies[I]; ActiveCargoIndex=I; }
        }
    }
    FVector Gap = Spreader->GetComponentLocation() - Cargo->GetComponentLocation();
    const float RelativeSpeed = (Spreader->GetPhysicsLinearVelocity() - Cargo->GetPhysicsLinearVelocity()).Size();
    const float Alignment = FMath::Abs(FQuat::ErrorAutoNormalize(Spreader->GetComponentQuat(), Cargo->GetComponentQuat()));
    if (FMath::Abs(Gap.X) > 65.f || FMath::Abs(Gap.Y) > 65.f ||
        FMath::Abs(Gap.Z - Crane::CargoHalfHeight - Crane::SpreaderHalfHeight) > 45.f ||
        RelativeSpeed > 80.f || Alignment > 0.04f)
    {
        Status = TEXT("Lock denied: align above cargo, lower closer and stop moving.");
        return;
    }
    TwistLock->SetWorldLocation(Spreader->GetComponentLocation() - FVector(0.f, 0.f, Crane::SpreaderHalfHeight));
    TwistLock->SetConstrainedComponents(Spreader, NAME_None, Cargo, NAME_None);
    bLocked = true;
    for(bool& Locked:STSCornerLocked) Locked=true;
    SampleSTSSensors(true);
    bDelivered = false;
    DeliverySettleTime = 0.f;
    Spreader->WakeAllRigidBodies();
    Cargo->WakeAllRigidBodies();
    Status = bTerminalMode ? TEXT("Twist lock engaged. Automatic transfer follows the assigned slot.") : TEXT("Twist lock engaged. Raise, move to the right pad, lower and release.");
}

float AQuayCrane::GetSwayDegrees() const
{
    const FVector Delta = Spreader->GetComponentLocation() - TrolleyMesh->GetComponentLocation();
    return FMath::RadiansToDegrees(FMath::Atan2(Delta.Size2D(), FMath::Max(1.f, -Delta.Z)));
}

float AQuayCrane::GetLoadHeight() const
{
    return Cargo?(Cargo->GetComponentLocation().Z - Crane::CargoHalfHeight - 20.f) / 100.f:0.f;
}

void AQuayCrane::UpdateRopes()
{
    for (int32 Index = 0; Index < Ropes.Num(); ++Index)
    {
        const FVector Offset((Index & 1) ? 100.f : -100.f, (Index & 2) ? 520.f : -520.f, 0.f);
        const FVector A = TrolleyMesh->GetComponentLocation() + Offset;
        const FVector B = Spreader->GetComponentLocation() + Offset;
        const FVector Delta = A - B;
        Ropes[Index]->SetWorldLocation((A + B) * 0.5f);
        Ropes[Index]->SetWorldRotation(FRotationMatrix::MakeFromZ(Delta).Rotator());
        Ropes[Index]->SetWorldScale3D(FVector(0.06f, 0.06f, Delta.Size() / 100.f));
    }
}

void AQuayCrane::Tick(float DeltaSeconds)
{
    Super::Tick(DeltaSeconds);
    if(bPickupTest){TickPickupTest(DeltaSeconds);return;}
    // World timers, control and Chaos consume the same accelerated delta.
    // Chaos subdivides the frame into small physics steps.
    const float Dt=DeltaSeconds;
    STSSimulationTime+=Dt;
    SampleSTSSensors();
    if(bSTSStartPending && STSSimulationTime>1.f)
    { bSTSStartPending=false; StartAutomatic(false); }
    const double WallNow=FPlatformTime::Seconds();
    const float WallDt=static_cast<float>(FMath::Clamp(WallNow-PreviousWallTick,0.0,0.1));
    PreviousWallTick=WallNow;
    if (!PlaybackTracePath.IsEmpty() && Cargo)
    {
        const FVector S=Spreader->GetComponentLocation();
        const FVector C=Cargo->GetComponentLocation();
        PlaybackTrace+=FString::Printf(TEXT("%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%d,%d\n"),
            PlaybackTraceFrame,PlaybackTraceTime,TrolleyPosition,GantryPosition,RopeLength,S.X,S.Y,S.Z,C.X,C.Y,C.Z,GetSwayDegrees(),bLocked?1:0,SmokeStage,Deliveries);
        PlaybackTraceTime+=Dt;
        if (bPlaybackSweep)
        {
            const int32 LastStep=Crane::SpeedLevelCount-1;
            const int32 Period=2*LastStep;
            const int32 Phase=(PlaybackTraceFrame/600)%Period;
            SimulationSpeed=Crane::SpeedLevels[Phase<=LastStep?Phase:Period-Phase];
            ApplyPlaybackRate();
        }
        ++PlaybackTraceFrame;
    }
    if (FParse::Param(FCommandLine::Get(), TEXT("PortSimCapture")) && !FParse::Param(FCommandLine::Get(),TEXT("PortSimAGVProof")))
    {
        CaptureElapsed += WallDt;
        if (!bCaptureRequested && CaptureElapsed > 8.f)
        {
            FScreenshotRequest::RequestScreenshot(FPaths::ProjectSavedDir() / (bHUDVisible?TEXT("Screenshots/CraneLab.png"):TEXT("Screenshots/CraneLab_HUDHidden.png")), true, false);
            bCaptureRequested = true;
        }
        if (CaptureElapsed > 12.f) FPlatformMisc::RequestExit(false);
    }

    if (bTerminalTest) TickTerminalTest(Dt);
    else if (bSmokeTest) TickSmokeTest(Dt);
    else if (auto* PC = Cast<APlayerController>(GetController()))
    {
        if (!bHUDMouseReady)
        {
            PC->bShowMouseCursor=true;
            PC->bEnableClickEvents=true;
            FInputModeGameAndUI Mode;
            Mode.SetHideCursorDuringCapture(false);
            PC->SetInputMode(Mode);
            bHUDMouseReady=true;
        }
        if (PC->WasInputKeyJustPressed(EKeys::H)) ToggleHUD();
        if (bUnifiedTerminal && PC->WasInputKeyJustPressed(EKeys::F))
        { bFollowAGV=!bFollowAGV; FollowedAGV=nullptr; }
        if (FParse::Param(FCommandLine::Get(),TEXT("PortSimHUDTest")))
        {
            HUDTestElapsed+=DeltaSeconds;
            if (HUDTestElapsed>0.5f)
            {
                if (auto* PortHUD=Cast<APortSimHUD>(PC->GetHUD()))
                {
                    bool Pass=bHUDVisible;
                    PortHUD->NotifyHitBoxClick(TEXT("TogglePortHUD")); Pass &= !bHUDVisible;
                    PortHUD->NotifyHitBoxClick(TEXT("TogglePortHUD")); Pass &= bHUDVisible;
                    ToggleHUD(); Pass &= !bHUDVisible;
                    ToggleHUD(); Pass &= bHUDVisible;
                    UE_LOG(LogPortSimCrane,Display,TEXT("PORTSIM_HUD_%s: button callback and keyboard toggle handler hide/show"),Pass?TEXT("PASS"):TEXT("FAIL"));
                    FPlatformMisc::RequestExitWithStatus(false,Pass?0:1);
                }
                else { UE_LOG(LogPortSimCrane,Error,TEXT("PORTSIM_HUD_FAIL: HUD unavailable")); FPlatformMisc::RequestExitWithStatus(false,1); }
            }
        }
        auto Axis = [PC](FKey Positive, FKey Negative) { return float(PC->IsInputKeyDown(Positive)) - float(PC->IsInputKeyDown(Negative)); };
        if (PC->WasInputKeyJustPressed(EKeys::Add) || PC->WasInputKeyJustPressed(EKeys::Equals)) IncreaseSimulationSpeed();
        if (PC->WasInputKeyJustPressed(EKeys::Subtract) || PC->WasInputKeyJustPressed(EKeys::Hyphen)) DecreaseSimulationSpeed();
        if (PC->WasInputKeyJustPressed(EKeys::Zero) || PC->WasInputKeyJustPressed(EKeys::NumPadZero)) ResetSimulationSpeed();
        if (PC->WasInputKeyJustPressed(EKeys::U)) StartAutomatic(false);
        if (PC->WasInputKeyJustPressed(EKeys::L)) StartAutomatic(true);
        if (PC->WasInputKeyJustPressed(EKeys::P)) bAutoPaused=!bAutoPaused;
        if (PC->WasInputKeyJustPressed(EKeys::R)) ResetSimulation();
        if (PC->WasInputKeyJustPressed(EKeys::SpaceBar))
        {
            bEmergencyStop = !bEmergencyStop;
            Status = bEmergencyStop ? TEXT("E-STOP: drives stopped; load physics remain active. Space to resume.") : TEXT("Drives enabled.");
        }
        if (bTerminalMode && PC->WasInputKeyJustPressed(EKeys::Home))
        { bFollowAGV=false; bFreeCamera=false; CameraArm->SetRelativeLocation(FVector(35000,0,0)); CameraArm->TargetArmLength=185000.f; CameraArm->SetRelativeRotation(FRotator(-52,38,0)); }
        if (bTerminalMode && PC->WasInputKeyJustPressed(EKeys::End))
        { bFollowAGV=false; bFreeCamera=false; CameraArm->SetRelativeLocation(FVector(12000,0,500)); CameraArm->TargetArmLength=bUnifiedTerminal?65000.f:22000.f; }
        if (bTerminalMode && PC->WasInputKeyJustPressed(EKeys::Tab)) FocusNextSiteCrane();
        TickFreeCamera(WallDt);
        if (!bFreeCamera)
        {
        const float Orbit = Axis(EKeys::Right, EKeys::Left);
        const float Pitch = Axis(EKeys::Up, EKeys::Down);
        FRotator Rotation = CameraArm->GetRelativeRotation();
        Rotation.Yaw += Orbit * 40.f * WallDt;
        Rotation.Pitch = FMath::Clamp(Rotation.Pitch + Pitch * 25.f * WallDt, -80.f, -8.f);
        CameraArm->SetRelativeRotation(Rotation);
        CameraArm->TargetArmLength = FMath::Clamp(CameraArm->TargetArmLength + Axis(EKeys::PageDown, EKeys::PageUp) * FMath::Max(2500.f, CameraArm->TargetArmLength * .65f) * WallDt, 2500.f, bTerminalMode ? 220000.f : 42000.f);
        }
    }
    if (bUnifiedTerminal)
    {
        if (FParse::Param(FCommandLine::Get(),TEXT("PortSimEquipmentTest"))) { TestEquipmentAndCamera(); return; }
        SiteLogistics->BeginTrafficFrame();
        if (bAutoRunning && !bAutoPaused && !bEmergencyStop && SiteLogistics->Fault.IsEmpty()) AutoElapsed+=Dt;
        TickSiteOperations(Dt);
        if (bFollowAGV) FollowLoadedAGV();
        if (FParse::Param(FCommandLine::Get(),TEXT("PortSimAGVProof")))
        {
            CaptureElapsed+=WallDt;
            bFollowAGV=true; FollowLoadedAGV();
            if (FollowedAGV.IsValid())
            {
                if (!bProofStarted) { ProofVehicleStart=FollowedAGV->GetActorLocation(); bProofStarted=true; }
                const float Distance=FVector::Distance(ProofVehicleStart,FollowedAGV->GetActorLocation());
                if (!bCaptureRequested && Distance>1500)
                {
                    FString Error;
                    const bool Valid=SiteLogistics->Validate(Error);
                    UE_LOG(LogTemp,Display,TEXT("AGV_RENDER_PROOF_%s: AGV%d transported attached cargo %.1f cm; %s"),Valid?TEXT("PASS"):TEXT("FAIL"),FollowedAGV->VehicleID,Distance,*Error);
                    FScreenshotRequest::RequestScreenshot(FPaths::ProjectSavedDir()/TEXT("Screenshots/AGV_LoadedMotion.png"),true,false);
                    bCaptureRequested=true; CaptureElapsed=0;
                }
            }
            if ((bCaptureRequested && CaptureElapsed>2) || CaptureElapsed>180) FPlatformMisc::RequestExit(false);
        }
        AutoCompleted=Deliveries=SiteLogistics->Delivered;
        if (bAutoRunning && SiteLogistics->ShipRemaining()==0 && SiteLogistics->IsIdle())
        {
            bAutoRunning=false;
            Status=FString::Printf(TEXT("All three ships unloaded: %d containers, %.1f simulation seconds"),Deliveries,AutoElapsed);
            UE_LOG(LogPortSimCrane,Display,TEXT("TERMINAL_ALL_UNLOADED: %d containers in %.2f simulation seconds"),Deliveries,AutoElapsed);
        }
        return;
    }
    bAutoDriveTarget=false;
    if (SiteLogistics) SiteLogistics->BeginTrafficFrame();
    if (bTerminalMode) { TickAutomatic(Dt); TickSiteOperations(Dt); }
    // Prevent sleeping at a residual pendulum angle while suspended.
    Spreader->WakeAllRigidBodies();
    if (bLocked) Cargo->WakeAllRigidBodies();
    const float OldLength=RopeLength;
    const bool bDriveStopped=bEmergencyStop || (bTerminalMode && bAutoPaused) || (bTerminalMode && AutoStage==ETerminalStage::Fault);
    for(float Remaining=Dt;Remaining>KINDA_SMALL_NUMBER;)
    {
        const float Step=FMath::Min(Remaining,1.f/60.f); Remaining-=Step;
        if(bAutoDriveTarget && !bDriveStopped) DriveSpreaderTo(AutoDriveTarget);
        const float HoistLimit=AutomaticHoistLimit();
        const FVector TargetVelocity = bDriveStopped ? FVector::ZeroVector : FVector(DriveInput.X * TravelSpeed, DriveInput.Y * STSGantrySpeed(), DriveInput.Z * HoistLimit);
        if(STSProfile.bReady && !bDriveStopped)
            DriveVelocity=FVector(FMath::FInterpConstantTo(DriveVelocity.X,TargetVelocity.X,Step,STSProfile.TrolleyAcceleration),
                FMath::FInterpConstantTo(DriveVelocity.Y,TargetVelocity.Y,Step,STSProfile.GantryAcceleration),
                FMath::FInterpConstantTo(DriveVelocity.Z,TargetVelocity.Z,Step,STSProfile.HoistAcceleration));
        else DriveVelocity = bDriveStopped ? FVector::ZeroVector : FMath::VInterpConstantTo(DriveVelocity, TargetVelocity, Step, Acceleration);
        TrolleyPosition = FMath::Clamp(TrolleyPosition + DriveVelocity.X * Step, STSProfile.bReady?STSProfile.MinTrolley():(bTerminalMode ? -3500.f : -2200.f), STSProfile.bReady?STSProfile.MaxTrolley():(bTerminalMode ? 3500.f : 2200.f));
        GantryPosition = FMath::Clamp(GantryPosition + DriveVelocity.Y * Step, bTerminalMode ? -3200.f : -1800.f, bTerminalMode ? 3200.f : 1800.f);
        RopeLength = FMath::Clamp(RopeLength - DriveVelocity.Z * Step, STSProfile.bReady?STSProfile.MinRope:Crane::MinRope, STSProfile.bReady?STSProfile.MaxRope:Crane::MaxRope);
    }
    const float NewLength=RopeLength;
    GantryRoot->SetRelativeLocation(FVector(0.f,GantryPosition,0.f));
    TrolleyMesh->SetRelativeLocation(FVector(TrolleyPosition,0.f,STSBeamHeight()));
    if(!FMath::IsNearlyEqual(OldLength,NewLength))
    {
        RopeLength = NewLength;
        Suspension->ConstraintInstance.SetLinearLimitSize(RopeLength);
        Spreader->WakeAllRigidBodies();
        if (bLocked) Cargo->WakeAllRigidBodies();
    }
    UpdateRopes();
    const FVector CargoPosition = Cargo->GetComponentLocation();
    const bool bOnTarget = !bTerminalMode && !bLocked && FMath::Abs(CargoPosition.X - 1400.f) < 230.f &&
        FMath::Abs(CargoPosition.Y) < 180.f && FMath::Abs(GetLoadHeight()) < 0.15f &&
        Cargo->GetPhysicsLinearVelocity().Size() < 10.f && Cargo->GetUpVector().Z > 0.98f;
    DeliverySettleTime = bOnTarget ? DeliverySettleTime + Dt : 0.f;
    if (!bDelivered && DeliverySettleTime > 1.5f)
    {
        bDelivered = true;
        ++Deliveries;
        Status = TEXT("Delivery complete! Press R to restart the exercise.");
        UE_LOG(LogPortSimCrane, Display, TEXT("Delivery complete: %d"), Deliveries);
    }
    if (FParse::Param(FCommandLine::Get(),TEXT("PortSimRateTest"))) TickPlaybackRateTest();
}

void AQuayCrane::TickPlaybackRateTest()
{
    // Model a 30 FPS rendering workload: playback must accelerate the job clock
    // even when the engine cannot produce hundreds of frames per second.
    const double Now=FPlatformTime::Seconds();
    if (PlaybackRateTestStep<0)
    {
        ResetSimulationSpeed(); StartAutomatic(false);
        PlaybackRateTestStep=0; PlaybackRateTestWall=Now; PlaybackRateTestSimulation=AutoElapsed;
    }
    else if (Now-PlaybackRateTestWall>=2.0)
    {
        const double Actual=(AutoElapsed-PlaybackRateTestSimulation)/(Now-PlaybackRateTestWall);
        const float Expected=Crane::SpeedLevels[PlaybackRateTestStep];
        const bool Pass=bAutoRunning && FMath::Abs(Actual-Expected)<Expected*0.2;
        UE_LOG(LogPortSimCrane,Display,TEXT("PORTSIM_RATE_SAMPLE: requested=%.0f actual=%.2f jobSeconds=%.2f %s"),
            Expected,Actual,AutoElapsed,Pass?TEXT("PASS"):TEXT("FAIL"));
        if (!Pass || ++PlaybackRateTestStep==Crane::SpeedLevelCount)
        {
            UE_LOG(LogPortSimCrane,Display,TEXT("PORTSIM_RATE_%s: measured job clock at limited frame rate"),Pass?TEXT("PASS"):TEXT("FAIL"));
            FPlatformMisc::RequestExitWithStatus(false,Pass?0:1);
            return;
        }
        IncreaseSimulationSpeed();
        PlaybackRateTestWall=Now; PlaybackRateTestSimulation=AutoElapsed;
    }
    FPlatformProcess::SleepNoStats(1.f/30.f);
}

void AQuayCrane::FinishSmokeTest(bool bSuccess, const FString& Reason)
{
    if (!PlaybackTracePath.IsEmpty() && !FFileHelper::SaveStringToFile(PlaybackTrace,*PlaybackTracePath,FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM))
        bSuccess=false;
    UE_LOG(LogPortSimCrane, Display, TEXT("PORTSIM_SMOKE_%s: %s"), bSuccess ? TEXT("PASS") : TEXT("FAIL"), *Reason);
    bSmokeTest = false;
    FPlatformMisc::RequestExitWithStatus(false, bSuccess ? 0 : 1);
}

void AQuayCrane::TickSmokeTest(float Dt)
{
    SmokeTime += Dt;
    SmokeStageTime += Dt;
    SetDriveInput(0.f, 0.f, 0.f);
    if (SmokeTime > 230.f ||
        Spreader->GetComponentLocation().ContainsNaN() ||
        Cargo->GetComponentLocation().ContainsNaN())
    {
        FinishSmokeTest(
            false,
            FString::Printf(
                TEXT("Timeout at stage %d; sway %.2f; spreader %s; cargo %s; trolley %s"),
                SmokeStage,
                GetSwayDegrees(),
                *Spreader->GetComponentLocation().ToString(),
                *Cargo->GetComponentLocation().ToString(),
                *TrolleyMesh->GetComponentLocation().ToString()));
        return;
    }
    auto Next = [this]() { ++SmokeStage; SmokeStageTime = 0.f; UE_LOG(LogPortSimCrane, Display, TEXT("Smoke stage %d"), SmokeStage); };
    switch (SmokeStage)
    {
    case 0:
        if (SmokeStageTime > 1.f)
        {
            ToggleLock();
            if (bLocked) { FinishSmokeTest(false, TEXT("Lock accepted while too far away")); return; }
            Next();
        }
        break;
    case 1:
        if (RopeLength < 2730.f) SetDriveInput(0.f, 0.f, -0.5f);
        else if (SmokeStageTime > 8.f)
        {
            ToggleLock();
            if (bLocked) Next();
        }
        break;
    case 2:
        if (RopeLength > 1900.f) SetDriveInput(0.f, 0.f, 1.f);
        else if (GetLoadHeight() > 5.f) Next();
        break;
    case 3:
        // PD approach to the target includes braking distance, while load remains physically free.
        SetDriveInput(FMath::Clamp(((1400.f - TrolleyPosition) * 0.8f - DriveVelocity.X * 0.8f) / TravelSpeed, -1.f, 1.f), 0.f, 0.f);
        MaxTestSway = FMath::Max(MaxTestSway, GetSwayDegrees());
        if (FMath::Abs(TrolleyPosition - 1400.f) < 8.f && FMath::Abs(DriveVelocity.X) < 5.f) Next();
        break;
    case 4:
        if (FMath::FloorToInt(SmokeStageTime) != FMath::FloorToInt(SmokeStageTime - Dt) && FMath::FloorToInt(SmokeStageTime) % 5 == 0)
            UE_LOG(LogPortSimCrane, Display, TEXT("Settle sway=%.2f spreader=%s cargo=%s trolley=%s"), GetSwayDegrees(), *Spreader->GetComponentLocation().ToString(), *Cargo->GetComponentLocation().ToString(), *TrolleyMesh->GetComponentLocation().ToString());
        if (SmokeStageTime > 18.f && GetSwayDegrees() < 1.f) Next();
        break;
    case 5:
        if (RopeLength < 2730.f) SetDriveInput(0.f, 0.f, -0.5f);
        else if (GetLoadHeight() < 0.15f && SmokeStageTime > 15.f) { ToggleLock(); Next(); }
        break;
    case 6:
        if (Deliveries == 1)
        {
            if (MaxTestSway < 0.2f) { FinishSmokeTest(false, TEXT("No physical sway measured")); return; }
            ResetSimulation();
            if (bLocked || Deliveries != 0 || !FMath::IsNearlyEqual(RopeLength, 2400.f))
            { FinishSmokeTest(false, TEXT("Reset state incorrect")); return; }
            Next();
        }
        break;
    case 7:
        SetDriveInput(1.f, 1.f, 1.f);
        bEmergencyStop = true;
        if (SmokeStageTime > 1.f)
        {
            const bool bStopped = FMath::IsNearlyEqual(TrolleyPosition, -1400.f) && FMath::IsNearlyZero(GantryPosition) && FMath::IsNearlyEqual(RopeLength, 2400.f);
            if (!bStopped) { FinishSmokeTest(false, TEXT("Emergency stop did not hold drives")); return; }
            bEmergencyStop = false;
            Next();
        }
        break;
    case 8:
        SetDriveInput(1.f, 1.f, 1.f);
        if (SmokeStageTime > 32.f)
        {
            if (!FMath::IsNearlyEqual(TrolleyPosition, 2200.f) || !FMath::IsNearlyEqual(GantryPosition, 1800.f) || !FMath::IsNearlyEqual(RopeLength, Crane::MinRope))
            { FinishSmokeTest(false, TEXT("Positive travel/hoist limits failed")); return; }
            Next();
        }
        break;
    case 9:
        SetDriveInput(-1.f, -1.f, -1.f);
        if (SmokeStageTime > 40.f)
        {
            if (!FMath::IsNearlyEqual(TrolleyPosition, -2200.f) || !FMath::IsNearlyEqual(GantryPosition, -1800.f) || !FMath::IsNearlyEqual(RopeLength, Crane::MaxRope))
            { FinishSmokeTest(false, TEXT("Negative travel/hoist limits failed")); return; }
            FinishSmokeTest(true, FString::Printf(TEXT("Lock rejection, pickup, lift, sway %.2f deg, transfer, release, delivery, reset, emergency stop, gantry and both travel/hoist limits"), MaxTestSway));
        }
        break;
    }
}

APortSimGameMode::APortSimGameMode()
{
    DefaultPawnClass = AQuayCrane::StaticClass();
    HUDClass = APortSimHUD::StaticClass();
}

void APortSimHUD::DrawHUD()
{
    Super::DrawHUD();
    auto* CranePawn = GetOwningPlayerController() ? Cast<AQuayCrane>(GetOwningPlayerController()->GetPawn()) : nullptr;
    if (!Canvas || !CranePawn) return;
    const float Scale = FMath::Clamp(Canvas->SizeX / 1440.f, 0.65f, 1.4f);
    const FVector2D TogglePosition(Canvas->SizeX-154.f*Scale,16.f);
    const FVector2D ToggleSize(138.f*Scale,30.f*Scale);
    DrawRect(FLinearColor(0.04f,0.18f,0.24f,0.96f),TogglePosition.X,TogglePosition.Y,ToggleSize.X,ToggleSize.Y);
    DrawText(CranePawn->bHUDVisible?TEXT("HUD 숨기기 [H]"):TEXT("HUD 표시 [H]"),FLinearColor(0.4f,1.f,1.f),
        TogglePosition.X+10.f*Scale,TogglePosition.Y+7.f*Scale,GEngine->GetSmallFont(),Scale);
    AddHitBox(TogglePosition,ToggleSize,TEXT("TogglePortHUD"),true,100);
    if (!CranePawn->bHUDVisible) return;
    DrawRect(FLinearColor(0.015f, 0.03f, 0.05f, 0.88f), 16.f, 16.f, 850.f * Scale, (CranePawn->bTerminalMode ? 310.f : 208.f) * Scale);
    float Y = 28.f;
    auto Line = [this, Scale, &Y](const FString& Text, FLinearColor Color, float FontScale = 1.f)
    {
        DrawText(Text, Color, 30.f, Y, GEngine->GetSmallFont(), Scale * FontScale);
        Y += 25.f * Scale;
    };
    Line(TEXT("항만 시뮬레이터"), FLinearColor(0.2f, 0.8f, 1.f), 1.4f);
    Line(FString::Printf(TEXT("SPEED %.0fx [%d/%d] | actual %.1fx | +/- speed | 0 reset"),
        CranePawn->SimulationSpeed, CranePawn->GetSimulationSpeedStep()+1, Crane::SpeedLevelCount, CranePawn->GetActualPlaybackRate()), FLinearColor(1.f,0.85f,0.25f),1.1f);
    if (CranePawn->bUnifiedTerminal)
    {
        Line(CranePawn->GetFleetStatus(),FLinearColor(0.3f,0.8f,1.f));
        Line(CranePawn->GetAGVStatus(),FLinearColor(1.f,.85f,.25f));
        Line(TEXT("CC 9 (24-row) | TC 46 | AGV 60 | RS 4 | YT 18 | EH 2 | FL 7 | YC 74"),FLinearColor::White);
        Line(TEXT("WASD move | Q/E down/up | RMB look | Shift boost"),FLinearColor::White);
        Line(TEXT("P pause/resume | Space E-stop | R reset | U resume unloading"),FLinearColor(0.3f,1.f,0.7f));
        Line(TEXT("Home overview | End middle berth | Tab next crane"),FLinearColor::White);
        Line(FString::Printf(TEXT("Elapsed %.1f s | %s"),CranePawn->AutoElapsed,
            CranePawn->bEmergencyStop?TEXT("E-STOP"):CranePawn->bAutoPaused?TEXT("PAUSED"):TEXT("RUNNING")),FLinearColor::White);
        Line(CranePawn->Status,FLinearColor(0.4f,1.f,0.65f));
        return;
    }
    if (CranePawn->bTerminalMode)
    {
        Line(CranePawn->GetFleetStatus(), FLinearColor(0.3f,0.8f,1.f));
        Line(TEXT("U 자동 하역   L 자동 적재   P 일시정지/재개   R 전체 초기화 24"), FLinearColor(0.3f,1.f,0.7f));
        Line(FString::Printf(TEXT("선박 %d/24 | 완료 %d | C%02d | %s %s | %.1f s"),
            CranePawn->GetShipCargoCount(),CranePawn->AutoCompleted,CranePawn->ActiveCargoIndex+1,
            CranePawn->GetAutoStageName(),CranePawn->bAutoPaused?TEXT("[일시정지]"):TEXT(""),CranePawn->AutoElapsed),FLinearColor::White);
    }
    Line(TEXT("Space 비상 정지   화살표 회전   PgUp/PgDn 줌"), FLinearColor::White);
    Line(FString::Printf(TEXT("로프 %.1f m   화물 높이 %.1f m   흔들림 %.1f deg   적재 완료 %d"),
        CranePawn->RopeLength / 100.f, CranePawn->GetLoadHeight(), CranePawn->GetSwayDegrees(), CranePawn->Deliveries), FLinearColor::White);
    Line(FString::Printf(TEXT("트위스트 락: %s   구동: %s   화물: %.1f t"),
        CranePawn->bLocked ? TEXT("작동") : TEXT("개방"), CranePawn->bEmergencyStop ? TEXT("정지") : TEXT("준비"),CranePawn->GetCargoMassKg()/1000), FLinearColor(1.f, 0.75f, 0.25f));
    if(CranePawn->bTerminalMode) Line(CranePawn->GetSTSStatus(),FLinearColor(0.5f,0.85f,1.f),0.85f);
    Line(CranePawn->Status, FLinearColor(0.4f, 1.f, 0.65f));
    Line(TEXT("등가 현수·수평 스프레더 | LT 환산·센서 모델은 가정 설정 적용"), FLinearColor(0.6f, 0.7f, 0.8f), 0.85f);
}

int32 AQuayCrane::GetSimulationSpeedStep() const
{
    for (int32 I=0; I<Crane::SpeedLevelCount; ++I)
        if (SimulationSpeed<=Crane::SpeedLevels[I]) return I;
    return Crane::SpeedLevelCount-1;
}

void AQuayCrane::IncreaseSimulationSpeed()
{
    const int32 Step=FMath::Min(GetSimulationSpeedStep()+1,Crane::SpeedLevelCount-1);
    SimulationSpeed=Crane::SpeedLevels[Step];
    ApplyPlaybackRate();
}

void AQuayCrane::DecreaseSimulationSpeed()
{
    const int32 Step=FMath::Max(GetSimulationSpeedStep()-1,0);
    SimulationSpeed=Crane::SpeedLevels[Step];
    ApplyPlaybackRate();
}

void AQuayCrane::ResetSimulationSpeed()
{
    SimulationSpeed=Crane::SpeedLevels[0];
    ApplyPlaybackRate();
}

void AQuayCrane::ToggleHUD()
{
    bHUDVisible=!bHUDVisible;
}

void APortSimHUD::NotifyHitBoxClick(FName BoxName)
{
    Super::NotifyHitBoxClick(BoxName);
    if (BoxName==TEXT("TogglePortHUD"))
        if (auto* PC=GetOwningPlayerController())
            if (auto* CranePawn=Cast<AQuayCrane>(PC->GetPawn())) CranePawn->ToggleHUD();
}

void AQuayCrane::InstallPlaybackClock()
{
    if (!GEngine || PlaybackClock) return;
    GetWorld()->GetWorldSettings()->MaxUndilatedFrameTime=UPortSimTimeStep::MaxSimulationDelta;
    PreviousClock=GEngine->GetCustomTimeStep();
    PlaybackClock=NewObject<UPortSimTimeStep>(this);
    if (!GEngine->SetCustomTimeStep(PlaybackClock))
    {
        PlaybackClock=nullptr;
        GEngine->SetCustomTimeStep(PreviousClock);
        UE_LOG(LogPortSimCrane,Error,TEXT("Could not install fixed simulation clock"));
    }
    ApplyPlaybackRate();
}

void AQuayCrane::ApplyPlaybackRate()
{
    if (PlaybackClock) PlaybackClock->SetPlaybackRate(SimulationSpeed);
}

float AQuayCrane::GetActualPlaybackRate() const
{
    return PlaybackClock?PlaybackClock->GetAchievedRate():1.f;
}

void AQuayCrane::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    if (GEngine && PlaybackClock && GEngine->GetCustomTimeStep()==PlaybackClock)
        GEngine->SetCustomTimeStep(PreviousClock);
    PlaybackClock=nullptr;
    PreviousClock=nullptr;
    DestroyTerminalActors();
    Super::EndPlay(EndPlayReason);
}
