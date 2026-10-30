#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "WeatherFriction.h"
#include "PortEnvironmentComponent.generated.h"

class UPhysicalMaterial;

/** Reduced-order rigid vessel response to the current wind-wave and swell state. */
struct PORTSIM_API FPortVesselMotion
{
    FVector TranslationCentimeters=FVector::ZeroVector;
    FQuat Rotation=FQuat::Identity;
    FVector LinearVelocityCentimetersPerSecond=FVector::ZeroVector;
    FVector AngularVelocityRadiansPerSecond=FVector::ZeroVector;
    FVector LinearAccelerationCentimetersPerSecondSquared=FVector::ZeroVector;
    FVector AngularAccelerationRadiansPerSecondSquared=FVector::ZeroVector;
    FVector LinearJerkCentimetersPerSecondCubed=FVector::ZeroVector;
    FVector AngularJerkRadiansPerSecondCubed=FVector::ZeroVector;
    float HeaveMeters=0;
    float RollDegrees=0;
    float PitchDegrees=0;

    FVector TransformPosition(FVector BasePosition,FVector Pivot) const
    { return Pivot+TranslationCentimeters+Rotation.RotateVector(BasePosition-Pivot); }
    FQuat TransformRotation(FQuat BaseRotation) const { return Rotation*BaseRotation; }
    FVector VelocityAtPosition(FVector BasePosition,FVector Pivot) const
    {
        return LinearVelocityCentimetersPerSecond+
            FVector::CrossProduct(AngularVelocityRadiansPerSecond,Rotation.RotateVector(BasePosition-Pivot));
    }
    FVector AccelerationAtPosition(FVector BasePosition,FVector Pivot) const
    {
        const FVector Radius=Rotation.RotateVector(BasePosition-Pivot);
        return LinearAccelerationCentimetersPerSecondSquared+
            FVector::CrossProduct(AngularAccelerationRadiansPerSecondSquared,Radius)+
            FVector::CrossProduct(AngularVelocityRadiansPerSecond,FVector::CrossProduct(AngularVelocityRadiansPerSecond,Radius));
    }
    FVector JerkAtPosition(FVector BasePosition,FVector Pivot) const
    {
        const FVector Radius=Rotation.RotateVector(BasePosition-Pivot);
        const FVector OmegaRadius=FVector::CrossProduct(AngularVelocityRadiansPerSecond,Radius);
        return LinearJerkCentimetersPerSecondCubed+
            FVector::CrossProduct(AngularJerkRadiansPerSecondCubed,Radius)+
            2*FVector::CrossProduct(AngularAccelerationRadiansPerSecondSquared,OmegaRadius)+
            FVector::CrossProduct(AngularVelocityRadiansPerSecond,
                FVector::CrossProduct(AngularAccelerationRadiansPerSecondSquared,Radius))+
            FVector::CrossProduct(AngularVelocityRadiansPerSecond,
                FVector::CrossProduct(AngularVelocityRadiansPerSecond,OmegaRadius));
    }
};

/** Sheltered-port sea-state assumptions; wave and long-period swell are distinct. */
struct PORTSIM_API FPortSeaState
{
    static constexpr float MinimumWaveHeightMeters=.10f;
    static constexpr float TypicalWaveHeightMeters=.22f;
    static constexpr float MaximumWaveHeightMeters=.40f;
    static constexpr float MinimumSwellHeightMeters=.15f;
    static constexpr float TypicalSwellHeightMeters=.32f;
    static constexpr float MaximumSwellHeightMeters=.60f;
    bool bSeaMotionEnabled=true;
    float WaveHeightMeters=TypicalWaveHeightMeters;
    float WavePeriodSeconds=5.5f;
    float WaveDirectionDegrees=135.f;
    float SwellHeightMeters=TypicalSwellHeightMeters;
    float SwellPeriodSeconds=10.5f;
    float SwellDirectionDegrees=150.f;
    double ElapsedSeconds=0;
    double WavePhaseRadians=0;
    double SwellPhaseRadians=0;

    void Initialize(int32 Seed);
    void Advance(double SimulationSeconds);
    FPortVesselMotion SampleVesselMotion(int32 VesselIndex) const;
    float SignificantWaveHeightMeters() const
    { return FMath::Sqrt(FMath::Square(WaveHeightMeters)+FMath::Square(SwellHeightMeters)); }
};

/** Weather model. Own random stream never perturbs logistics randomness. */
struct PORTSIM_API FPortWindState
{
    static constexpr float MinimumSpeed=1.f;
    static constexpr float TypicalSpeed=3.5f;
    static constexpr float MaximumSpeed=7.f;
    static constexpr float MeanVariationMetersPerSecond=1.f;
    // Normal-operation gust assumptions. These are not measured DGT limits.
    // A short plateau represents the meteorological three-second gust sample.
    static constexpr float MinimumGustPeak=2.f;
    static constexpr float MaximumGustPeak=5.f;
    static constexpr float MinimumGustDuration=6.f;
    static constexpr float MaximumGustDuration=12.f;
    static constexpr float MinimumGustInterval=300.f;
    static constexpr float MaximumGustInterval=900.f;
    static constexpr float MaximumGustDirectionOffset=15.f;
    bool bWindEnabled=true;
    float WindSpeedMetersPerSecond=TypicalSpeed;
    float ReferenceWindSpeedMetersPerSecond=TypicalSpeed;
    float WindDirectionDegrees=0;
    FVector WindDirectionVector=FVector(-1,0,0);
    float TargetWindSpeedMetersPerSecond=TypicalSpeed;
    float TargetWindDirectionDegrees=0;
    double SecondsUntilTarget=0;
    bool bGustActive=false;
    float GustSpeedMetersPerSecond=0;
    float GustPeakSpeedMetersPerSecond=0;
    float GustDirectionDegrees=0;
    float GustDirectionOffsetDegrees=0;
    double GustElapsedSeconds=0;
    double GustDurationSeconds=0;
    double SecondsUntilGust=0;
    int32 GustCount=0;

    void Initialize(int32 Seed);
    void Advance(double SimulationSeconds);
    void SetReferenceSpeed(float Speed);
    FVector EffectiveVelocity() const;
    float EffectiveDirectionDegrees() const;
    static float NormalizeDirection(float Degrees);
    static float InterpolateDirection(float Current,float Target,double Seconds);
    static FString DirectionName(float Degrees);
    // Local compass convention: North=world +X, East=world +Y, Up=+Z.
    // This is a simulation convention, not a georeferenced Busan bearing.
    // Degrees describe wind FROM; this unit vector points TO (opposite direction).
    static FVector FlowDirection(float FromDegrees);
private:
    FRandomStream Random,GustRandom;
    float SampleSpeed();
    float SampleTargetSpeed();
    void SelectTarget();
    void AdvanceMean(double SimulationSeconds);
    void AdvanceGust(double SimulationSeconds);
    void StartGust();
    void ScheduleNextGust(bool Initial);
    static float GustEnvelope(double Phase);
};

/** Shared aerodynamic helpers. Inputs and outputs use SI units. */
struct PORTSIM_API FPortWindAerodynamics
{
    static constexpr double AirDensityKgPerCubicMeter=1.225;
    static FVector DragForceNewtons(FVector WindVelocityMetersPerSecond,FVector BodyVelocityCentimetersPerSecond,
        FQuat BodyRotation,FVector DimensionsMeters,double DragCoefficient);
};

/** Sensor-only fog model. Visual fog is deliberately outside this phase. */
struct PORTSIM_API FPortFogState
{
    static constexpr float DefaultVisibilityMeters=30.f;
    static constexpr float MinimumVisibilityMeters=20.f;
    static constexpr float MaximumVisibilityMeters=1000.f;
    bool bFogEnabled=false;
    float VisibilityMeters=DefaultVisibilityMeters;

    float SensorRangeMeters(const FString& SensorKey,float NominalRangeMeters) const
    {
        if(!bFogEnabled) return NominalRangeMeters;
        const bool bFogSensitive=SensorKey.Contains(TEXT("camera"),ESearchCase::IgnoreCase) ||
            SensorKey.Contains(TEXT("lidar"),ESearchCase::IgnoreCase);
        return bFogSensitive?FMath::Min(NominalRangeMeters,VisibilityMeters):NominalRangeMeters;
    }
};

/** Deterministic sensor and wet-surface rain model. Visual rain is deliberately excluded. */
struct PORTSIM_API FPortRainState
{
    static constexpr float DefaultRainfallMmPerHour=25.f;
    static constexpr float MinimumRainfallMmPerHour=1.f;
    static constexpr float MaximumRainfallMmPerHour=100.f;
    bool bRainEnabled=false;
    float RainfallMmPerHour=DefaultRainfallMmPerHour;

    float Severity() const
    { return bRainEnabled?FMath::Clamp(RainfallMmPerHour/50.f,0.f,1.f):0.f; }
    float SensorRangeScale(const FString& SensorKey) const
    {
        const float S=Severity();
        if(SensorKey.Contains(TEXT("camera"),ESearchCase::IgnoreCase)) return FMath::Lerp(1.f,.6f,S);
        if(SensorKey.Contains(TEXT("lidar"),ESearchCase::IgnoreCase)) return FMath::Lerp(1.f,.75f,S);
        return 1.f;
    }
};

/** Simulation operating assumptions, editable independently of the weather generators. */
USTRUCT(BlueprintType)
struct PORTSIM_API FPortWeatherOperatingLimits
{
    GENERATED_BODY()
    UPROPERTY(EditAnywhere,BlueprintReadWrite) float StopWindMps=10;
    UPROPERTY(EditAnywhere,BlueprintReadWrite) float ResumeWindMps=8;
    UPROPERTY(EditAnywhere,BlueprintReadWrite) float StopVisibilityM=25;
    UPROPERTY(EditAnywhere,BlueprintReadWrite) float ResumeVisibilityM=35;
    UPROPERTY(EditAnywhere,BlueprintReadWrite) float StopRainMmPerHour=50;
    UPROPERTY(EditAnywhere,BlueprintReadWrite) float ResumeRainMmPerHour=40;
    UPROPERTY(EditAnywhere,BlueprintReadWrite) float StopSnowMmPerHour=80;
    UPROPERTY(EditAnywhere,BlueprintReadWrite) float ResumeSnowMmPerHour=60;
    UPROPERTY(EditAnywhere,BlueprintReadWrite) float StopIce=.5f;
    UPROPERTY(EditAnywhere,BlueprintReadWrite) float ResumeIce=.35f;
    UPROPERTY(EditAnywhere,BlueprintReadWrite) float StopTireMu=.15f;
    UPROPERTY(EditAnywhere,BlueprintReadWrite) float ResumeTireMu=.2f;
    UPROPERTY(EditAnywhere,BlueprintReadWrite) float StopWaveHeightM=.65f;
    UPROPERTY(EditAnywhere,BlueprintReadWrite) float ResumeWaveHeightM=.55f;
    UPROPERTY(EditAnywhere,BlueprintReadWrite) float RecoverySeconds=10;
};

struct PORTSIM_API FPortWeatherHold
{
    bool Active=false;
    float RecoveryElapsed=0;
    FString Reason;
    void Update(const class UPortEnvironmentComponent& Environment,float Seconds);
};

/** One environment owned by the terminal controller and sampled by equipment. */
UCLASS(ClassGroup=(PortSim), meta=(BlueprintSpawnableComponent))
class PORTSIM_API UPortEnvironmentComponent : public UActorComponent
{
    GENERATED_BODY()
    friend class FTerminalReplayCheckpointTest;
    friend class FTerminalCheckpoint;
    friend class FTerminalReplay;
public:
    UPortEnvironmentComponent();
    UPROPERTY(EditAnywhere,BlueprintReadWrite,Category="Environment|Operations") FPortWeatherOperatingLimits OperatingLimits;
    UFUNCTION(BlueprintCallable, Category="Environment|Surface") void SetSurfaceConditions(float Wetness,float Snow,float Ice);
    UFUNCTION(BlueprintPure, Category="Environment|Surface") FPortSurfaceState GetSurfaceConditions() const;
    UFUNCTION(BlueprintCallable, Category="Environment|Surface") void SetWaterFilmMm(float Millimeters);
    UFUNCTION(BlueprintCallable, Category="Environment|Surface") void SetSnowDepthMm(float Millimeters);
    UFUNCTION(BlueprintCallable, Category="Environment|Surface") void SetFrictionSettings(const FPortFrictionSettings& Settings);
    const FPortFrictionSettings& GetFrictionSettings() const { return FrictionSettings; }
    UPhysicalMaterial* GetContactMaterial();
    UPROPERTY(EditAnywhere,BlueprintReadWrite,Category="Environment|Surface") FPortFrictionSettings FrictionSettings;
    UPROPERTY(EditAnywhere,BlueprintReadOnly,Category="Environment|Surface") FPortSurfaceState Surface;
    virtual void BeginPlay() override;
    // Called once by AQuayCrane with the already accelerated world DeltaSeconds.
    // Do not multiply speed or delta by SimulationSpeed a second time.
    void AdvanceEnvironment(float SimulationSeconds);

    UFUNCTION(BlueprintPure, Category="Environment|Wind") bool IsWindEnabled() const { return Wind.bWindEnabled; }
    UFUNCTION(BlueprintPure, Category="Environment|Wind") float GetWindSpeedMetersPerSecond() const { return Wind.WindSpeedMetersPerSecond; }
    UFUNCTION(BlueprintPure, Category="Environment|Wind") float GetWindDirectionDegrees() const { return Wind.WindDirectionDegrees; }
    UFUNCTION(BlueprintPure, Category="Environment|Wind") bool IsGustActive() const { return Wind.bGustActive; }
    UFUNCTION(BlueprintPure, Category="Environment|Wind") float GetGustSpeedMetersPerSecond() const { return Wind.GustSpeedMetersPerSecond; }
    UFUNCTION(BlueprintPure, Category="Environment|Wind") float GetEffectiveWindSpeedMetersPerSecond() const { return GetEffectiveWindVelocity().Size(); }
    UFUNCTION(BlueprintPure, Category="Environment|Wind") float GetEffectiveWindDirectionDegrees() const { return Wind.EffectiveDirectionDegrees(); }
    /** Actual wind travel direction, opposite to meteorological wind-from degrees. */
    UFUNCTION(BlueprintPure, Category="Environment|Wind") FVector GetWindDirectionVector() const { return Wind.WindDirectionVector; }
    /** Velocity in metres/second, NOT Unreal centimetres/second. OFF returns zero. */
    UFUNCTION(BlueprintPure, Category="Environment|Wind") FVector GetEffectiveWindVelocity() const;
    UFUNCTION(BlueprintPure, Category="Environment|Wind") FString GetWindDirectionName() const { return FPortWindState::DirectionName(Wind.EffectiveDirectionDegrees()); }
    UFUNCTION(BlueprintCallable, Category="Environment|Wind") void ToggleWind() { NotifyExternalInput(); Wind.bWindEnabled=!Wind.bWindEnabled; }
    UFUNCTION(BlueprintCallable, Category="Environment|Wind") void SetWindEnabled(bool Enabled) { NotifyExternalInput(); Wind.bWindEnabled=Enabled; }
    UFUNCTION(BlueprintCallable, Category="Environment|Wind") void SetWindSpeedMetersPerSecond(float Speed);
    /** User-selected slider value. Unlike live mean/gust wind, this does not drift with weather. */
    UFUNCTION(BlueprintPure, Category="Environment|Wind") float GetWindSettingMetersPerSecond() const { return Wind.ReferenceWindSpeedMetersPerSecond; }
    UFUNCTION(BlueprintPure, Category="Environment|Wind") float GetWindIntensity() const
    { return (Wind.ReferenceWindSpeedMetersPerSecond-FPortWindState::MinimumSpeed)/(FPortWindState::MaximumSpeed-FPortWindState::MinimumSpeed); }
    const FPortWindState& GetWindState() const { return Wind; }
    UFUNCTION(BlueprintPure, Category="Environment|Sea") bool IsSeaMotionEnabled() const { return Sea.bSeaMotionEnabled; }
    UFUNCTION(BlueprintPure, Category="Environment|Sea") float GetSignificantWaveHeightMeters() const { return Sea.SignificantWaveHeightMeters(); }
    UFUNCTION(BlueprintPure, Category="Environment|Sea") float GetWavePeriodSeconds() const { return Sea.WavePeriodSeconds; }
    UFUNCTION(BlueprintPure, Category="Environment|Sea") float GetSwellPeriodSeconds() const { return Sea.SwellPeriodSeconds; }
    UFUNCTION(BlueprintCallable, Category="Environment|Sea") void ToggleSeaMotion() { NotifyExternalInput(); Sea.bSeaMotionEnabled=!Sea.bSeaMotionEnabled; }
    UFUNCTION(BlueprintCallable, Category="Environment|Sea") void SetSeaMotionEnabled(bool Enabled) { NotifyExternalInput(); Sea.bSeaMotionEnabled=Enabled; }
    UFUNCTION(BlueprintCallable, Category="Environment|Sea") void SetSeaIntensity(float Intensity);
    UFUNCTION(BlueprintPure, Category="Environment|Sea") float GetSeaIntensity() const;
    FPortVesselMotion GetVesselMotion(int32 VesselIndex=0) const { return Sea.SampleVesselMotion(VesselIndex); }
    const FPortSeaState& GetSeaState() const { return Sea; }
    UFUNCTION(BlueprintPure, Category="Environment|Fog") bool IsFogEnabled() const { return Fog.bFogEnabled; }
    UFUNCTION(BlueprintPure, Category="Environment|Fog") float GetFogVisibilityMeters() const { return Fog.VisibilityMeters; }
    UFUNCTION(BlueprintCallable, Category="Environment|Fog") void ToggleFog() { NotifyExternalInput(); Fog.bFogEnabled=!Fog.bFogEnabled; }
    UFUNCTION(BlueprintCallable, Category="Environment|Fog") void SetFogEnabled(bool Enabled) { NotifyExternalInput(); Fog.bFogEnabled=Enabled; }
    UFUNCTION(BlueprintCallable, Category="Environment|Fog") void SetFogIntensity(float Intensity);
    UFUNCTION(BlueprintPure, Category="Environment|Fog") float GetFogIntensity() const;
    void SetFogVisibilityMeters(float VisibilityMeters);
    float GetFogLimitedSensorRangeMeters(const FString& SensorKey,float NominalRangeMeters) const
    { return Fog.SensorRangeMeters(SensorKey,NominalRangeMeters); }
    const FPortFogState& GetFogState() const { return Fog; }
    UFUNCTION(BlueprintPure, Category="Environment|Rain") bool IsRainEnabled() const { return Rain.bRainEnabled; }
    UFUNCTION(BlueprintPure, Category="Environment|Rain") float GetRainfallMmPerHour() const { return Rain.RainfallMmPerHour; }
    UFUNCTION(BlueprintCallable, Category="Environment|Rain") void ToggleRain() { NotifyExternalInput(); Rain.bRainEnabled=!Rain.bRainEnabled; }
    UFUNCTION(BlueprintCallable, Category="Environment|Rain") void SetRainEnabled(bool Enabled) { NotifyExternalInput(); Rain.bRainEnabled=Enabled; }
    UFUNCTION(BlueprintCallable, Category="Environment|Rain") void SetRainIntensity(float Intensity);
    UFUNCTION(BlueprintPure, Category="Environment|Rain") float GetRainIntensity() const
    { return (Rain.RainfallMmPerHour-FPortRainState::MinimumRainfallMmPerHour)/(FPortRainState::MaximumRainfallMmPerHour-FPortRainState::MinimumRainfallMmPerHour); }
    void SetRainfallMmPerHour(float RainfallMmPerHour);
    const FPortRainState& GetRainState() const { return Rain; }
    UFUNCTION(BlueprintPure, Category="Environment|Snow") bool IsSnowEnabled() const { return bSnowEnabled; }
    UFUNCTION(BlueprintPure, Category="Environment|Snow") float GetSnowIntensity() const { return SnowIntensity; }
    UFUNCTION(BlueprintCallable, Category="Environment|Snow") void SetSnowEnabled(bool Enabled) { NotifyExternalInput(); bSnowEnabled=Enabled; }
    UFUNCTION(BlueprintCallable, Category="Environment|Snow") void ToggleSnow() { NotifyExternalInput(); bSnowEnabled=!bSnowEnabled; }
    UFUNCTION(BlueprintCallable, Category="Environment|Snow") void SetSnowIntensity(float Intensity);
    UFUNCTION(BlueprintPure, Category="Environment|Snow") float GetSnowfallMmPerHour() const;
    UFUNCTION(BlueprintCallable, Category="Environment|Snow") void SetSnowfallMmPerHour(float MillimetersPerHour);
    UFUNCTION(BlueprintPure, Category="Environment|Ice") bool IsIceEnabled() const { return bIceEnabled; }
    UFUNCTION(BlueprintPure, Category="Environment|Ice") float GetIceIntensity() const { return IceIntensity; }
    UFUNCTION(BlueprintCallable, Category="Environment|Ice") void SetIceEnabled(bool Enabled) { NotifyExternalInput(); bIceEnabled=Enabled; }
    UFUNCTION(BlueprintCallable, Category="Environment|Ice") void ToggleIce() { NotifyExternalInput(); bIceEnabled=!bIceEnabled; }
    UFUNCTION(BlueprintCallable, Category="Environment|Ice") void SetIceIntensity(float Intensity);
    UPROPERTY(EditAnywhere,BlueprintReadOnly,Category="Environment|Snow") bool bSnowEnabled=false;
    UPROPERTY(EditAnywhere,BlueprintReadOnly,Category="Environment|Snow",meta=(ClampMin="0",ClampMax="1")) float SnowIntensity=1;
    UPROPERTY(EditAnywhere,BlueprintReadOnly,Category="Environment|Ice") bool bIceEnabled=false;
    UPROPERTY(EditAnywhere,BlueprintReadOnly,Category="Environment|Ice",meta=(ClampMin="0",ClampMax="1")) float IceIntensity=1;
private:
    void NotifyExternalInput();
    UPROPERTY() TObjectPtr<UPhysicalMaterial> ContactMaterial;
    FPortSurfaceState MaterialSurface;
    FPortFrictionSettings MaterialSettings;
    FPortSurfaceAccumulation Accumulation;
    FPortSurfaceState PublishedSurface;
    bool bSurfaceInitialized=false;
    void SynchronizeSurfaceInputs();
    void PublishSurface();
    FPortWindState Wind;
    FPortSeaState Sea;
    FPortFogState Fog;
    FPortRainState Rain;
    int32 LastReportedGustCount=0;
};
