#include "STSOperatingProfile.h"
#include "Dom/JsonObject.h"
#include "Serialization/JsonSerializer.h"
#include "Misc/FileHelper.h"

namespace
{
    using FObject = TSharedPtr<FJsonObject>;
    FObject Object(const FObject& Parent, const TCHAR* Key)
    {
        const FObject* Result = nullptr;
        return Parent && Parent->TryGetObjectField(Key, Result) ? *Result : nullptr;
    }
    bool Number(const FObject& Parent, const TCHAR* Key, float& Value, FString& Error, bool Positive = true)
    {
        double N = 0;
        if (!Parent || !Parent->TryGetNumberField(Key, N) || !FMath::IsFinite(N) || (Positive && N <= 0))
        { Error = FString::Printf(TEXT("Invalid/missing numeric field: %s"), Key); return false; }
        Value = static_cast<float>(N);
        if (!FMath::IsFinite(Value)) { Error = FString::Printf(TEXT("Field overflow: %s"), Key); return false; }
        return true;
    }
    bool Quantity(const FObject& Parent, const TCHAR* Key, const TCHAR* Unit, float& Value, FString& Error)
    {
        const FObject Q = Object(Parent, Key);
        FString Actual;
        if (!Q || !Q->TryGetStringField(TEXT("unit"), Actual) || Actual != Unit)
        { Error = FString::Printf(TEXT("Missing/wrong unit for %s (expected %s)"), Key, Unit); return false; }
        return Number(Q, TEXT("value"), Value, Error);
    }
    bool Read(const FString& Path, FObject& Root, FString& Error)
    {
        FString Text;
        if (!FFileHelper::LoadFileToString(Text, *Path) || !FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text), Root) || !Root)
        { Error = TEXT("Cannot read valid STS JSON: ") + Path; return false; }
        return true;
    }
}

bool FSTSOperatingProfile::Load(const FString& ReferenceFile, const FString& SettingsFile)
{
    *this = FSTSOperatingProfile();
    ReferencePath = ReferenceFile; SettingsPath = SettingsFile;
    FObject Ref, Settings;
    if (!Read(ReferenceFile, Ref, Error) || !Read(SettingsFile, Settings, Error)) return false;
    FString Kind;
    if (!Settings->TryGetStringField(TEXT("kind"), Kind) || (Kind != TEXT("simulation_assumptions_not_manufacturer_data") && Kind != TEXT("mixed_manufacturer_sensor_specs_and_simulation_assumptions")))
    { Error = TEXT("STS settings must explicitly identify simulation assumptions"); return false; }
    const auto Geometry = Object(Ref,TEXT("geometry"));
    const auto Drives = Object(Ref,TEXT("drives"));
    const auto Hoist = Object(Drives,TEXT("hoist"));
    const auto Capacity = Object(Ref,TEXT("capacity"));
    float TotalLift = 0, RatedLT = 0, MotorCount = 0, MotorKW = 0;
    if (!Quantity(Geometry,TEXT("rail_gauge"),TEXT("m"),RailGauge,Error) ||
        !Quantity(Geometry,TEXT("outreach"),TEXT("m"),Outreach,Error) ||
        !Quantity(Geometry,TEXT("backreach"),TEXT("m"),Backreach,Error) ||
        !Quantity(Geometry,TEXT("lift_above_rail"),TEXT("m"),LiftAboveRail,Error) ||
        !Quantity(Geometry,TEXT("lift_total"),TEXT("m"),TotalLift,Error) ||
        !Quantity(Object(Drives,TEXT("main_trolley")),TEXT("speed"),TEXT("m/min"),TrolleySpeed,Error) ||
        !Quantity(Object(Drives,TEXT("gantry")),TEXT("speed"),TEXT("m/min"),GantrySpeed,Error) ||
        !Quantity(Capacity,TEXT("rated_under_spreader"),TEXT("LT"),RatedLT,Error) ||
        !Number(Object(Hoist,TEXT("motors")),TEXT("count"),MotorCount,Error) ||
        !Number(Object(Hoist,TEXT("motors")),TEXT("power_each_kw"),MotorKW,Error)) return false;
    RailGauge *= 100; Outreach *= 100; Backreach *= 100;
    LiftBelowRail = (TotalLift-LiftAboveRail)*100; LiftAboveRail *= 100;
    TrolleySpeed *= 100.f/60.f; GantrySpeed *= 100.f/60.f;
    HoistPowerW = MotorCount*MotorKW*1000;

    auto Setting = [&](const TCHAR* Key,float& Out,bool Positive=true) { return Number(Settings,Key,Out,Error,Positive); };
    float Clearance=0, Rate=0;
    if (!Setting(TEXT("assumed_kg_per_LT"),LoadUnitKg) ||
        !Setting(TEXT("waterside_rail_x_m"),WatersideRailX,false) ||
        !Setting(TEXT("support_clearance_above_lift_m"),Clearance) ||
        !Setting(TEXT("transfer_spreader_height_m"),SafeHeight) ||
        !Setting(TEXT("trolley_acceleration_mps2"),TrolleyAcceleration) ||
        !Setting(TEXT("gantry_acceleration_mps2"),GantryAcceleration) ||
        !Setting(TEXT("hoist_acceleration_mps2"),HoistAcceleration) ||
        !Setting(TEXT("default_container_mass_kg"),ContainerMassKg) ||
        !Setting(TEXT("spreader_mass_kg"),SpreaderMassKg) ||
        !Setting(TEXT("sensor_sample_hz"),Rate) ||
        !Setting(TEXT("sensor_max_age_s"),SensorMaxAge) ||
        !Setting(TEXT("position_sensor_bias_m"),PositionBias,false) ||
        !Setting(TEXT("load_estimate_bias_kg"),MassBiasKg,false) ||
        !Setting(TEXT("landing_tolerance_m"),LandingTolerance) ||
        !Setting(TEXT("seating_vertical_tolerance_m"),SeatingTolerance) ||
        !Setting(TEXT("settle_speed_mps"),SettleSpeed) ||
        !Setting(TEXT("settle_time_s"),SettleTime) ||
        !Setting(TEXT("agv_position_tolerance_m"),AGVTolerance) ||
        !Setting(TEXT("agv_heading_tolerance_deg"),AGVHeadingTolerance) ||
        !Setting(TEXT("support_tolerance_m"),SupportTolerance) ||
        !Setting(TEXT("landing_speed_mps"),ApproachSpeed) ||
        !Setting(TEXT("landing_slowdown_distance_m"),ApproachDistance) ||
        !Setting(TEXT("stage_timeout_s"),StageTimeout) ||
        !Setting(TEXT("landing_sway_limit_deg"),SwayLimitDegrees)) return false;
    float CX=0,CY=0,CZ=0;
    if (!Setting(TEXT("container_cog_x_m"),CX,false) || !Setting(TEXT("container_cog_y_m"),CY,false) || !Setting(TEXT("container_cog_z_m"),CZ,false) ||
        !Settings->TryGetBoolField(TEXT("auto_start_unload"),bAutoStart))
    { if(Error.IsEmpty()) Error=TEXT("Missing auto_start_unload"); return false; }
    ContainerCoG=FVector(CX,CY,CZ)*100;
    RatedPayloadKg = RatedLT*LoadUnitKg;
    WatersideRailX *= 100; SafeHeight *= 100;
    MinRope = Clearance*100; BeamHeight = LiftAboveRail+MinRope; MaxRope = BeamHeight+LiftBelowRail;
    TrolleyAcceleration *= 100; GantryAcceleration *= 100; HoistAcceleration *= 100;
    SensorPeriod=1.f/Rate; PositionBias*=100; LandingTolerance*=100; SeatingTolerance*=100;
    SettleSpeed*=100; AGVTolerance*=100; SupportTolerance*=100; ApproachSpeed*=100; ApproachDistance*=100;
    if (LiftBelowRail<=0 || SafeHeight>=LiftAboveRail || SensorMaxAge<SensorPeriod ||
        FMath::Abs(CX)>=1.0f || FMath::Abs(CY)>=5.2f || FMath::Abs(CZ)>=1.295f ||
        LandingTolerance>=100 || SeatingTolerance>=50 || SwayLimitDegrees>=90 || Rate>1000)
    { Error=TEXT("Inconsistent STS geometry, timing or simulation tolerances"); return false; }

    const TArray<TSharedPtr<FJsonValue>>* Points=nullptr;
    if (!Hoist || !Hoist->TryGetArrayField(TEXT("speed_points"),Points))
    { Error=TEXT("Missing hoist speed_points"); return false; }
    int32 EmptyCount=0;
    for (const auto& Point:*Points)
    {
        const auto P=Point->AsObject(); FString Condition; float Speed=0,Load=0;
        if (!P || !P->TryGetStringField(TEXT("condition"),Condition) || !Number(P,TEXT("speed_m_per_min"),Speed,Error)) return false;
        if (Condition==TEXT("empty_spreader")) { EmptyHoistSpeed=Speed*100.f/60.f; ++EmptyCount; }
        else if (Condition==TEXT("loaded"))
        {
            FString Unit;
            if (!P->TryGetStringField(TEXT("payload_unit_as_drawn"),Unit) || Unit!=TEXT("LT") || !Number(P,TEXT("payload_value_as_drawn"),Load,Error))
            { Error=TEXT("Invalid loaded hoist point/units"); return false; }
            LoadedHoistCurve.Add(FVector2D(Load*LoadUnitKg,Speed*100.f/60.f));
        }
        else { Error=TEXT("Unknown hoist curve condition"); return false; }
    }
    LoadedHoistCurve.Sort([](const FVector2D& A,const FVector2D& B){return A.X<B.X;});
    if (LoadedHoistCurve.Num()<2 || EmptyCount!=1 || !FMath::IsNearlyEqual(float(LoadedHoistCurve.Last().X),RatedPayloadKg,1.f))
    { Error=TEXT("Hoist curve must reach rated payload and include one empty-spreader condition"); return false; }
    for(int32 I=1;I<LoadedHoistCurve.Num();++I)
        if(LoadedHoistCurve[I].X<=LoadedHoistCurve[I-1].X || LoadedHoistCurve[I].Y>LoadedHoistCurve[I-1].Y)
        { Error=TEXT("Hoist curve must have distinct masses and non-increasing speeds"); return false; }
    const TArray<TSharedPtr<FJsonValue>>* Sensors=nullptr;
    if(!Ref->TryGetArrayField(TEXT("sensors"),Sensors)) {Error=TEXT("Missing sensor reference");return false;}
    for(const auto& S:*Sensors)
    {
        const auto Obj=S->AsObject(); FString Key;
        if(!Obj || !Obj->TryGetStringField(TEXT("key"),Key) || SensorKeys.Contains(Key)) {Error=TEXT("Invalid/duplicate sensor key");return false;}
        SensorKeys.Add(Key);
    }
    for(const TCHAR* Required:{TEXT("trolley_encoder"),TEXT("hoist_encoder"),TEXT("twistlock_load"),TEXT("twistlock_state"),TEXT("landed"),TEXT("agv_position_lidar")})
        if(!SensorKeys.Contains(Required)) {Error=FString(TEXT("Required sensor missing: "))+Required;return false;}

    if(!Pickup.Load(Settings,Error) || !Dynamics.Load(Settings,Error)) return false;
    for(const auto& Mount:Dynamics.Mounts) if(!SensorKeys.Contains(Mount.Key)) {Error=TEXT("Unknown configured sensor mount");return false;}
    auto Snapshot=MakeShared<FJsonObject>();
    Snapshot->SetObjectField(TEXT("reference"),Ref); Snapshot->SetObjectField(TEXT("simulation_assumptions"),Settings);
    Snapshot->SetStringField(TEXT("load_model"),TEXT("Site STS: reduced-order variable-length sway/yaw, taut four-corner tension allocation, hoist shaft torque. Assumed mounts/drive data; constrained roll/pitch. Legacy central path unchanged."));
    Snapshot->SetNumberField(TEXT("resolved_payload_limit_kg"),RatedPayloadKg);
    FJsonSerializer::Serialize(Snapshot,TJsonWriterFactory<>::Create(&SnapshotJson));
    bReady=true;
    return true;
}

float FSTSOperatingProfile::HoistLimit(float PayloadKg,bool bLoaded) const
{
    if(!bReady || !FMath::IsFinite(PayloadKg) || PayloadKg<0 || (bLoaded && PayloadKg>RatedPayloadKg)) return 0;
    float Speed=EmptyHoistSpeed;
    if(bLoaded)
    {
        Speed=LoadedHoistCurve[0].Y; // Conservative below lowest documented loaded point; never use empty speed for a loaded lift.
        for(int32 I=1;I<LoadedHoistCurve.Num();++I)
        {
            if(PayloadKg<=LoadedHoistCurve[I].X)
            { const auto& A=LoadedHoistCurve[I-1]; const auto& B=LoadedHoistCurve[I]; Speed=FMath::Lerp(float(A.Y),float(B.Y),FMath::Clamp(float((PayloadKg-A.X)/(B.X-A.X)),0.f,1.f)); break; }
        }
    }
    // Ideal mechanical power envelope only. Motor torque/current dynamics remain unresolved.
    const float SuspendedMass=SpreaderMassKg+(bLoaded?PayloadKg:0.f);
    return FMath::Min(Speed,100.f*HoistPowerW/(SuspendedMass*9.80665f));
}

bool FSTSOperatingProfile::ContainsTarget(FVector P) const
{
    return bReady && !P.ContainsNaN() && P.X>=MinTrolley() && P.X<=MaxTrolley() && P.Z>=-LiftBelowRail && P.Z<=LiftAboveRail;
}
