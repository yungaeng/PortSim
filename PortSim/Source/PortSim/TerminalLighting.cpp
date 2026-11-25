#include "QuayCrane.h"
#include "PortWorkingCrane.h"
#include "PortContainerActor.h"
#include "PortEnvironmentComponent.h"
#include "SpreaderTelescope.h"
#include "TerminalLayout.h"
#include "Components/DirectionalLightComponent.h"
#include "Components/HierarchicalInstancedStaticMeshComponent.h"
#include "Components/SpotLightComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/World.h"
#include "GameFramework/PlayerController.h"
#include "HAL/PlatformTime.h"
#include "Materials/MaterialInterface.h"
#include "Misc/App.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#if WITH_DEV_AUTOMATION_TESTS
#include "Misc/AutomationTest.h"
#include "Misc/Paths.h"
#include "Misc/ScopeExit.h"
#endif

namespace
{
    constexpr int32 ShadowBudget = 6;
    constexpr float ShadowRangeCm = 23000.f; // Includes the built-in 200 m STS camera.
    constexpr float ShadowRetentionCm = 1500.f;

    float WeatherAdjustedDaylight(float DaylightLux, float RainRate, float SnowRate)
    {
        // The environment has no cloud optical-depth model. Use precipitation as a
        // proxy for loss of daylight, and independently respond to low fog visibility.
        const float RainTransmission = FMath::Lerp(1.f, .12f, FMath::Clamp(RainRate / 75.f, 0.f, 1.f));
        const float SnowTransmission = FMath::Lerp(1.f, .22f, FMath::Clamp(SnowRate / 80.f, 0.f, 1.f));
        return DaylightLux * FMath::Min(RainTransmission, SnowTransmission);
    }

    bool NeedsWorkLights(bool bAlreadyOn, float DaylightLux, float FogMetres, float RainRate, float SnowRate)
    {
        return WeatherAdjustedDaylight(DaylightLux, RainRate, SnowRate) < (bAlreadyOn ? 750.f : 450.f)
            || FogMetres < (bAlreadyOn ? 550.f : 400.f);
    }

    constexpr float CameraMinimumLux = 5.f; // Synthetic pose sensor, pending camera calibration.
    constexpr float CameraAssistLux = 35.f;
    constexpr float CameraAssistRangeM = 58.f;
    constexpr float CameraAssistCone = 70.f;
    constexpr float MaxCameraAssistCandela = 18000.f;
    const FVector TrolleyMountM(0, 2.6, -1.05);

    struct FWorkLightBeam { float Candela; float RangeM; float Cone; };
    FWorkLightBeam TrolleyBeam(bool bSTS)
    {
        return bSTS ? FWorkLightBeam{74000.f, 66.f, 29.f} : FWorkLightBeam{39000.f, 43.f, 35.f};
    }

    float AmbientCameraLux(const UPortEnvironmentComponent& Environment)
    {
        return WeatherAdjustedDaylight(Environment.GetDaylightIlluminanceLux(),
            Environment.IsRainEnabled() ? Environment.GetRainfallMmPerHour() : 0.f,
            Environment.IsSnowEnabled() ? Environment.GetSnowfallMmPerHour() : 0.f);
    }

    double SpotIlluminance(FVector Source, FVector Forward, FVector Target, FVector Normal,
        const FWorkLightBeam& Beam, float FogMetres)
    {
        const FVector Ray = Target - Source;
        const double Metres = Ray.Size() * .01;
        if (Metres >= Beam.RangeM || Metres < .01) return 0.;
        const FVector Direction = Ray.GetSafeNormal();
        const double Outer = FMath::Cos(FMath::DegreesToRadians(double(Beam.Cone)));
        const double Inner = FMath::Cos(FMath::DegreesToRadians(double(Beam.Cone) * .40));
        const double Cone = FMath::Clamp((FVector::DotProduct(Forward, Direction) - Outer) / (Inner - Outer), 0., 1.);
        const double Range = 1. - FMath::Square(FMath::Square(Metres / Beam.RangeM));
        const double Incidence = FMath::Max(0., FVector::DotProduct(Normal, -Direction));
        const double Transmission = FogMetres < MAX_flt ? FMath::Exp(-3.912 * Metres / FMath::Max(1.f, FogMetres)) : 1.;
        return Beam.Candela * FMath::Square(Cone * Range) * Incidence * Transmission / FMath::Square(Metres);
    }

    USpotLightComponent* AddSpot(USceneComponent* Parent, const TCHAR* Name, const FVector& MountM,
        const FVector& AimM, float Candela, float RangeM, float ConeDegrees, float TemperatureK, float Bounce=0.f)
    {
        AActor* Owner = Parent->GetOwner();
        auto* Spot = NewObject<USpotLightComponent>(Owner, MakeUniqueObjectName(Owner, USpotLightComponent::StaticClass(), FName(Name)));
        Owner->AddInstanceComponent(Spot);
        Spot->SetupAttachment(Parent);
        Spot->SetAbsolute(false, false, true);
        Spot->SetMobility(EComponentMobility::Movable);
        // The trolley mesh is scaled to its physical size. Mount offsets are still in metres.
        Spot->SetRelativeLocationAndRotation(MountM * 100. / Parent->GetComponentScale(), (AimM - MountM).Rotation());
        Spot->SetUseInverseSquaredFalloff(true);
        Spot->SetIntensityUnits(ELightUnits::Candelas);
        Spot->SetIntensity(Candela);
        Spot->SetAttenuationRadius(RangeM * 100.f);
        Spot->SetInnerConeAngle(ConeDegrees * .40f);
        Spot->SetOuterConeAngle(ConeDegrees);
        Spot->SetUseTemperature(true);
        Spot->SetTemperature(TemperatureK);
        Spot->SetSpecularScale(.40f);
        Spot->SetCastShadows(false);
        Spot->SetCastRaytracedShadows(ECastRayTracedShadow::Disabled);
        Spot->SetIndirectLightingIntensity(Bounce);
        Spot->SetVolumetricScatteringIntensity(0.f);
        Spot->SetVisibility(false);
        Spot->RegisterComponent();
        return Spot;
    }

    UHierarchicalInstancedStaticMeshComponent* AddFixtureMesh(USceneComponent* Root, const TCHAR* Name,
        const TCHAR* Shape, const TCHAR* Material)
    {
        auto* Mesh = NewObject<UHierarchicalInstancedStaticMeshComponent>(Root->GetOwner(), FName(Name));
        Root->GetOwner()->AddInstanceComponent(Mesh);
        Mesh->SetupAttachment(Root);
        Mesh->SetMobility(Root->Mobility);
        Mesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);
        Mesh->SetCastShadow(false);
        Mesh->SetStaticMesh(LoadObject<UStaticMesh>(nullptr, Shape));
        Mesh->SetMaterial(0, LoadObject<UMaterialInterface>(nullptr, Material));
        Mesh->RegisterComponent();
        return Mesh;
    }
}

void AQuayCrane::BuildNightAmbientLight()
{
    NightAmbientLight = NewObject<UDirectionalLightComponent>(this, TEXT("PortNightAmbient"));
    AddInstanceComponent(NightAmbientLight);
    NightAmbientLight->SetMobility(EComponentMobility::Movable);
    NightAmbientLight->SetAtmosphereSunLight(false);
    NightAmbientLight->SetForwardShadingPriority(0);
    NightAmbientLight->SetWorldRotation(FRotator(-57., 145., 0.));
    NightAmbientLight->SetLightColor(FLinearColor(.43f, .61f, 1.f));
    NightAmbientLight->SetCastShadows(false);
    NightAmbientLight->SetSpecularScale(0.f);
    NightAmbientLight->SetIndirectLightingIntensity(0.f);
    NightAmbientLight->SetVolumetricScatteringIntensity(0.f);
    NightAmbientLight->SetIntensity(0.f);
    NightAmbientLight->RegisterComponent();
}

void AQuayCrane::UpdateNightAmbientLight(double SolarElevation)
{
    if (!NightAmbientLight) return;
    // Low-level skyglow from the surrounding port and city. This approximates
    // distant diffuse light while Lumen handles bounce from nearby floodlights.
    constexpr float NightAmbientLux = 3.f;
    const float Illuminance = NightAmbientLux * (1.0 - FMath::SmoothStep(-12.0, -4.0, SolarElevation));
    if (!FMath::IsNearlyEqual(NightAmbientLight->Intensity, Illuminance, .005f))
        NightAmbientLight->SetIntensity(Illuminance);
}

void AQuayCrane::BuildTerminalLighting(USceneComponent* SiteRoot)
{
    const bool bEnableCameraAssist = !FParse::Param(FCommandLine::Get(), TEXT("PortSimNoCameraWorkLights"));
    for (APortWorkingCrane* Crane : WorkingCranes)
        Crane->SetCameraLightingEnvironment(Environment, bEnableCameraAssist);
    if (!FApp::CanEverRender() || GetNetMode() == NM_DedicatedServer) return;

    auto* Poles = AddFixtureMesh(SiteRoot, TEXT("TerminalLightPoles"), TEXT("/Engine/BasicShapes/Cylinder.Cylinder"),
        TEXT("/Game/PortSim/Assets/Materials/M_Steel.M_Steel"));
    auto* Heads = AddFixtureMesh(SiteRoot, TEXT("TerminalFloodlightHousings"), TEXT("/Engine/BasicShapes/Cube.Cube"),
        TEXT("/Game/PortSim/Assets/Materials/M_SiteWhite.M_SiteWhite"));

    auto AddPole = [&](const FVector& Base, float Height, float Diameter)
    {
        Poles->AddInstance(FTransform(FRotator::ZeroRotator, (Base + FVector(0,0,Height * .5f)) * 100.,
            FVector(Diameter, Diameter, Height)));
    };
    constexpr float MastHeight = 30.f;
    auto AddMastHead = [&](const FVector& Mount, const FVector& Aim, float Candela, float Range, float Cone)
    {
        Heads->AddInstance(FTransform((Aim - Mount).Rotation(), Mount * 100., FVector(.55,.88,.35)));
        TerminalLights.Add(AddSpot(SiteRoot, TEXT("YardMastFloodlight"), Mount, Aim, Candela, Range, Cone, 4100.f, .38f));
    };
    auto AddTower = [&](float PlanX, float Y, const FVector& Inward)
    {
        const FVector Base(TerminalLayout::SiteX(PlanX), Y, .22f);
        AddPole(Base, MastHeight, .65f);
        const FVector Top = Base + FVector(0,0,MastHeight);
        const FVector Along(-Inward.Y, Inward.X, 0);
        for (float Offset : {-70.f, 0.f, 70.f})
        {
            const FVector Mount = Top + Inward * .65 + Along * (Offset / 60.f);
            const FVector Aim = Base + Inward * (Offset == 0.f ? 48. : 89.) + Along * Offset;
            AddMastHead(Mount, Aim, Offset == 0.f ? 315000.f : 630000.f, 195.f, 37.f);
        }
        if (Inward == FVector::ForwardVector)
            AddMastHead(Top - Inward * .65, Base - Inward * 50., 240000.f, 150.f, 51.f);
    };

    // Yard margins outside the AGV apron and RMG rail ends. Leave the central crossing open.
    for (float Y : {-445.f, -305.f, -190.f, -115.f, 25.f, 165.f, 305.f})
    {
        AddTower(160.f, Y, FVector::ForwardVector);
        AddTower(608.f, Y, -FVector::ForwardVector);
    }
    for (float X : {290.f, 475.f})
    {
        AddTower(X, -519.f, FVector::RightVector);
        AddTower(X, 354.f, -FVector::RightVector);
    }
    // The central road has two lane centres 4.5 m either side of this median.
    // Keep the median towers away from the perimeter intersections.
    for (float X : {265.f, 410.f, 555.f})
    {
        const FVector Base(TerminalLayout::SiteX(X), TerminalLayout::CentralRoadY, .22);
        AddPole(Base, MastHeight, .65f);
        for (double Along : {-1., 1.})
            for (double Across : {-1., 1.})
                AddMastHead(Base + FVector(Along * .75, Across * .75, MastHeight),
                    Base + FVector(Along * 43., Across * 80., 0.), 495000.f, 175.f, 38.f);
    }

    // Roadside infill at the uncovered approaches, standby area and gate.
    // Coordinates are plan metres, with the aiming points on the paved routes.
    const struct { FVector2D Pole; FVector2D Target; } RoadLights[] = {
        {{132,-477},{113,-490}}, {{132,-146},{124,-157}}, {{153,359},{132,347}},
        {{116,388},{87,405}}, {{116,489},{87,493}}, {{116,577},{87,581}},
        {{650,-146},{635,-160}}, {{653,353},{635,347}},
        {{754,609},{773,625}}, {{834,609},{817,625}},
    };
    for (const auto& Road : RoadLights)
    {
        const FVector Base(TerminalLayout::SiteX(Road.Pole.X), Road.Pole.Y, .22);
        const FVector Aim(TerminalLayout::SiteX(Road.Target.X), Road.Target.Y, .22);
        const FVector Arm = (Aim - Base).GetSafeNormal2D();
        const FVector Mount = Base + FVector(0,0,11.) + Arm * .8;
        AddPole(Base, 11.f, .24f);
        Heads->AddInstance(FTransform((Aim - Mount).Rotation(), Mount * 100., FVector(.38,.5,.22)));
        TerminalLights.Add(AddSpot(SiteRoot, TEXT("RoadFloodlight"), Mount, Aim, 27000.f, 53.f, 42.f, 3100.f));
    }

    for (APortWorkingCrane* Crane : WorkingCranes)
        Crane->AddTerminalWorkLights(TerminalLights, TerminalShadowLights);
}

void APortWorkingCrane::AddTerminalWorkLights(TArray<TWeakObjectPtr<USpotLightComponent>>& Lights,
    TArray<TWeakObjectPtr<USpotLightComponent>>& ShadowLights)
{
    const double GirderMountZ = BeamZ / 100. - 1.4;
    if (bSTS)
    {
        Lights.Add(AddSpot(RootComponent, TEXT("STSDeckFloodlight"), {-28,-10,GirderMountZ}, {-43,3,7}, 118000.f, 78.f, 37.f, 3500.f));
        Lights.Add(AddSpot(RootComponent, TEXT("STSHandoverFloodlight"), {28,10,GirderMountZ}, {46,0,1}, 100000.f, 71.f, 36.f, 3800.f));
    }
    else
        Lights.Add(AddSpot(RootComponent, TEXT("RMGGantryFloodlight"), {15,3,GirderMountZ}, {-4,0,1}, 32000.f, 48.f, 41.f, 3650.f));

    const FVector Aim(0, 0, -BeamZ / 100.);
    const FWorkLightBeam Beam = TrolleyBeam(bSTS);
    auto* Task = AddSpot(Trolley, TEXT("TrolleyTaskFloodlight"), TrolleyMountM, Aim,
        Beam.Candela, Beam.RangeM, Beam.Cone, 4400.f);
    Lights.Add(Task);
    ShadowLights.Add(Task);

    if (bCameraWorkLightsEnabled)
        for (int32 Index = 0; Index < (bSTS ? 1 : 2); ++Index)
        {
            const FVector Mount = CameraWorkLightMount(Index);
            CameraWorkLights.Add(AddSpot(Spreader, TEXT("SpreaderCameraWorkLight"), Mount, Mount - FVector::UpVector,
                0.f, CameraAssistRangeM, CameraAssistCone, 4500.f));
        }
}

void APortWorkingCrane::SetCameraLightingEnvironment(UPortEnvironmentComponent* Environment, bool bEnableAssist)
{
    CameraLightingEnvironment = Environment;
    bCameraWorkLightsEnabled = bEnableAssist;
}

FVector APortWorkingCrane::CameraWorkLightMount(int32 Index) const
{
    // A lamp above each RMG camera pair maintains side coverage on final seating.
    const double EndM = bSTS ? 0. : PortSpreaderTelescope::TwistlockHalfLengthCm(TelescopeLengthCm) * .01;
    return FVector(0., Index == 0 ? -EndM : EndM, .82);
}

FVector APortWorkingCrane::CameraTargetSurface(FVector Target) const
{
    // Pose observations use the cargo centre and corner XYs. Light falls on its roof.
    return IsValid(CargoActor) ? Target + CargoActor->GetActorUpVector() * 129.5 : Target;
}

float APortWorkingCrane::CameraWorkLightIntensity(float AmbientLux) const
{
    if (!bCameraWorkLightsEnabled || !bJobActive || bCarrying || !IsValid(CargoActor) || AmbientLux >= CameraAssistLux)
        return 0.f;
    const FVector Roof = CameraTargetSurface(CargoActor->GetActorLocation());
    const double DistanceM = FVector::DotProduct(Spreader->GetUpVector(), Spreader->GetComponentLocation() - Roof) * .01 + .82;
    // Limit glare near the box and avoid frequent light changes during descent.
    const float Intensity = FMath::Clamp(float((CameraAssistLux - AmbientLux) * FMath::Square(FMath::Max(DistanceM, 1.))), 0.f, MaxCameraAssistCandela);
    return FMath::CeilToFloat(Intensity / 25.f) * 25.f;
}

bool APortWorkingCrane::HasCameraTargetLight(FVector Target) const
{
    const auto* Environment = CameraLightingEnvironment.Get();
    if (!Environment) return true;
    const float AmbientLux = AmbientCameraLux(*Environment);
    double Lux = AmbientLux;
    if (Lux >= CameraMinimumLux) return true;

    const FVector Roof = CameraTargetSurface(Target);
    const FVector Normal = IsValid(CargoActor) ? CargoActor->GetActorUpVector() : FVector::UpVector;
    const float FogMetres = Environment->IsFogEnabled() ? Environment->GetFogVisibilityMeters() : MAX_flt;
    const FTransform& TrolleyTransform = Trolley->GetComponentTransform();
    const FVector TrolleyAim = FVector(0., 0., -BeamZ / 100.) - TrolleyMountM;
    // At this light level the existing automatic trolley floodlights are on.
    Lux += SpotIlluminance(TrolleyTransform.TransformPositionNoScale(TrolleyMountM * 100.),
        TrolleyTransform.TransformVectorNoScale(TrolleyAim.GetSafeNormal()), Roof, Normal, TrolleyBeam(bSTS), FogMetres);
    if (Lux >= CameraMinimumLux) return true;

    const float Candela = CameraWorkLightIntensity(AmbientLux);
    if (Candela <= 0.f) return false;
    const FTransform& SpreaderTransform = Spreader->GetComponentTransform();
    const FWorkLightBeam Beam{Candela, CameraAssistRangeM, CameraAssistCone};
    for (int32 Index = 0; Index < (bSTS ? 1 : 2); ++Index)
    {
        Lux += SpotIlluminance(SpreaderTransform.TransformPositionNoScale(CameraWorkLightMount(Index) * 100.),
            -Spreader->GetUpVector(), Roof, Normal, Beam, FogMetres);
    }
    return Lux >= CameraMinimumLux;
}

void APortWorkingCrane::UpdateCameraWorkLightsForDraw()
{
    if (CameraWorkLights.IsEmpty()) return;
    const auto* Environment = CameraLightingEnvironment.Get();
    const float Candela = bJobActive && !bCarrying && Environment ? CameraWorkLightIntensity(AmbientCameraLux(*Environment)) : 0.f;
    for (int32 Index = 0; Index < CameraWorkLights.Num(); ++Index)
    {
        if (auto* Light = CameraWorkLights[Index].Get())
        {
            Light->SetVisibility(Candela > 0.f);
            if (Candela <= 0.f) continue;
            const FVector Location = CameraWorkLightMount(Index) * 100.;
            if (!Light->GetRelativeLocation().Equals(Location, .1)) Light->SetRelativeLocation(Location);
            if (Light->Intensity != Candela) Light->SetIntensity(Candela);
        }
    }
}

void AQuayCrane::UpdateTerminalLightingForDraw()
{
    if (TerminalLights.IsEmpty()) return;
    const bool bOn = NeedsWorkLights(bTerminalLightsOn, TerminalDaylightLux,
        Environment->IsFogEnabled() ? Environment->GetFogVisibilityMeters() : MAX_flt,
        Environment->IsRainEnabled() ? Environment->GetRainfallMmPerHour() : 0.f,
        Environment->IsSnowEnabled() ? Environment->GetSnowfallMmPerHour() : 0.f);
    const bool bLightsChanged = bOn != bTerminalLightsOn;
    if (bLightsChanged)
    {
        bTerminalLightsOn = bOn;
        NextTerminalShadowUpdate = 0.;
        for (const auto& Entry : TerminalLights)
            if (auto* Light = Entry.Get()) Light->SetVisibility(bOn);
        if (!bOn)
            for (const auto& Entry : TerminalShadowLights)
                if (auto* Light = Entry.Get()) Light->SetCastShadows(false);
        UE_LOG(LogTemp, Display, TEXT("PORTSIM_TERMINAL_LIGHTS: on=%d fixtures=%d daylight_lux=%.1f fog_m=%.0f"),
            bOn, TerminalLights.Num(), TerminalDaylightLux,
            Environment->IsFogEnabled() ? Environment->GetFogVisibilityMeters() : 0.f);
    }
    if (bOn || bLightsChanged)
        for (APortWorkingCrane* Crane : WorkingCranes) Crane->UpdateCameraWorkLightsForDraw();

    const double WallNow = FPlatformTime::Seconds();
    if (!bOn || WallNow < NextTerminalShadowUpdate) return;
    NextTerminalShadowUpdate = WallNow + .35;
    const auto* PC = Cast<APlayerController>(GetController());
    if (!PC) return;

    FVector CameraLocation;
    FRotator ViewRotation;
    PC->GetPlayerViewPoint(CameraLocation, ViewRotation);
    struct FShadowCandidate { USpotLightComponent* Light; double Distance; };
    TArray<FShadowCandidate, TInlineAllocator<64>> Nearby;
    for (const auto& Entry : TerminalShadowLights)
    {
        if (auto* Light = Entry.Get())
        {
            const double Distance = FVector::Distance(CameraLocation, Light->GetComponentLocation());
            const double Adjusted = Distance - (Light->CastShadows ? ShadowRetentionCm : 0.f);
            if (Adjusted <= ShadowRangeCm) Nearby.Add({Light, Adjusted});
        }
    }
    Nearby.Sort([](const FShadowCandidate& A, const FShadowCandidate& B) { return A.Distance < B.Distance; });
    const int32 Count = FMath::Min(Nearby.Num(), ShadowBudget);
    for (const auto& Entry : TerminalShadowLights)
    {
        if (auto* Light = Entry.Get())
        {
            bool bCasts = false;
            for (int32 Index = 0; Index < Count; ++Index) bCasts |= Nearby[Index].Light == Light;
            if (Light->CastShadows != bCasts) Light->SetCastShadows(bCasts);
        }
    }
}

void AQuayCrane::ClearTerminalLighting()
{
    // The site and crane actors own their fixtures and destroy them on site rebuild.
    TerminalLights.Reset();
    TerminalShadowLights.Reset();
    bTerminalLightsOn = false;
    NextTerminalShadowUpdate = 0.;
}

#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTerminalWorkLightSwitchTest, "PortSim.Environment.TerminalWorkLightSwitch",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTerminalWorkLightSwitchTest::RunTest(const FString& Parameters)
{
    TestTrue(TEXT("Twilight turns the lights on"), NeedsWorkLights(false, 400.f, MAX_flt, 0.f, 0.f));
    TestFalse(TEXT("Clear daylight turns the lights off"), NeedsWorkLights(true, 20000.f, MAX_flt, 0.f, 0.f));
    TestTrue(TEXT("Fog turns the lights on during daylight"), NeedsWorkLights(false, 20000.f, 300.f, 0.f, 0.f));
    TestTrue(TEXT("Fog must clear past the release threshold"), NeedsWorkLights(true, 20000.f, 480.f, 0.f, 0.f));
    TestFalse(TEXT("The same visibility does not turn off lights back on"), NeedsWorkLights(false, 20000.f, 480.f, 0.f, 0.f));
    TestTrue(TEXT("Heavy rain darkens early twilight enough to switch on"), NeedsWorkLights(false, 2000.f, MAX_flt, 75.f, 0.f));
    TestTrue(TEXT("Heavy snow darkens early twilight enough to switch on"), NeedsWorkLights(false, 2000.f, MAX_flt, 0.f, 80.f));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCameraNightLightingTest, "PortSim.Environment.CameraNightLighting",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FCameraNightLightingTest::RunTest(const FString& Parameters)
{
    const auto Init = UWorld::InitializationValues().AllowAudioPlayback(false).CreatePhysicsScene(true)
        .RequiresHitProxies(false).CreateNavigation(false).CreateAISystem(false);
    auto* World = UWorld::CreateWorld(EWorldType::Game, false, NAME_None, nullptr, true, ERHIFeatureLevel::Num, &Init);
    ON_SCOPE_EXIT { World->DestroyWorld(false); };
    auto* Environment = NewObject<UPortEnvironmentComponent>();
    Environment->SetFogEnabled(true);
    Environment->SetFogVisibilityMeters(30.f);
    FSTSOperatingProfile Profile;
    if (!TestTrue(TEXT("Load STS operating profile"), Profile.Load(FPaths::ProjectDir() / TEXT("../Document/STS/STS_ReferenceData.json"),
        FPaths::ProjectConfigDir() / TEXT("STS_Simulation.json")))) return false;

    for (bool bSTS : {true, false})
    {
        const FVector Position(bSTS ? -1600.f : 0.f, bSTS ? 0.f : 10000.f, 149.5f);
        auto* Crane = World->SpawnActor<APortWorkingCrane>(FVector(0., Position.Y, 0.), FRotator::ZeroRotator);
        if (bSTS) Crane->SetSTSProfile(Profile);
        Crane->Configure(bSTS ? 1 : 2, bSTS, Position, Position, false);
        auto* Cargo = World->SpawnActor<APortContainerActor>(Position, FRotator::ZeroRotator);
        Cargo->InitializeContainer(bSTS ? 9401 : 9402);
        Cargo->GetBody()->SetSimulatePhysics(false);
        Crane->CargoActor = Cargo;
        Crane->bJobActive = true;
        Crane->Head = Crane->Local(Position) + FVector(0., 0., 154.5);
        Crane->UpdateParts();
        Crane->SetFogVisibilityMeters(30.f);

        Environment->ResetEnvironmentClock();
        Crane->SetCameraLightingEnvironment(Environment, false);
        Crane->SampleSTS(true);
        TestTrue(TEXT("Daylight permits camera observations"), Crane->Observation.bTargetVisible);

        Environment->AdvanceEnvironment(13.f * 3600.f);
        Crane->SampleSTS(true);
        TestFalse(TEXT("Fog attenuates distant trolley light and leaves the target dark"), Crane->Observation.bTargetVisible);
        Crane->SetCameraLightingEnvironment(Environment, true);
        Crane->SampleSTS(true);
        TestTrue(TEXT("Local camera work lights restore nighttime pose observation"), Crane->Observation.bTargetVisible);

        Crane->SetFogVisibilityMeters(.1f);
        Crane->SampleSTS(true);
        TestFalse(TEXT("Lighting cannot restore a target outside the camera visibility limit"), Crane->Observation.bTargetVisible);
    }
    return true;
}
#endif
