#include "STSOperatingProfile.h"
#include "Dom/JsonObject.h"
#include "Serialization/JsonSerializer.h"
#include "Misc/FileHelper.h"

bool FSTSOperatingProfile::LoadRMG(const FString& File)
{
    *this=FSTSOperatingProfile(); SettingsPath=File;
    FString Text; TSharedPtr<FJsonObject> Root;
    if(!FFileHelper::LoadFileToString(Text,*File) || !FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text),Root) || !Root)
    {Error=TEXT("Cannot read RMG settings: ")+File;return false;}
    FString Kind;
    if(!Root->TryGetStringField(TEXT("kind"),Kind) || Kind!=TEXT("rmg_reference_selection_and_simulation_assumptions"))
    {Error=TEXT("RMG assumptions provenance missing");return false;}
    auto N=[&](const TCHAR* Key,float& Out,double Scale=1.,bool Zero=false)
    {double V=0;if(!Root->TryGetNumberField(Key,V)||!FMath::IsFinite(V)||V<0||(!Zero&&V==0)||!FMath::IsFinite(float(V*Scale)))
      {Error=FString(TEXT("Invalid RMG setting: "))+Key;return false;}Out=V*Scale;return true;};
    if(!N(TEXT("rated_payload_kg"),RatedPayloadKg,1,false))return false;
    if(!N(TEXT("rail_span_m"),RailGauge,100,false))return false;
    if(!N(TEXT("cantilever_each_m"),Outreach,100,true))return false;
    if(!N(TEXT("lift_height_m"),LiftAboveRail,100,false))return false;
    if(!N(TEXT("beam_height_m"),BeamHeight,100,false))return false;
    if(!N(TEXT("transfer_spreader_height_m"),SafeHeight,100,false))return false;
    if(!N(TEXT("empty_hoist_m_min"),EmptyHoistSpeed,1.6666666666666667,false))return false;
    if(!N(TEXT("trolley_m_min"),TrolleySpeed,1.6666666666666667,false))return false;
    if(!N(TEXT("gantry_m_min"),GantrySpeed,1.6666666666666667,false))return false;
    if(!N(TEXT("assumed_hoist_power_kw"),HoistPowerW,1000,false))return false;
    if(!N(TEXT("trolley_acceleration_mps2"),TrolleyAcceleration,100,false))return false;
    if(!N(TEXT("gantry_acceleration_mps2"),GantryAcceleration,100,false))return false;
    if(!N(TEXT("hoist_acceleration_mps2"),HoistAcceleration,100,false))return false;
    if(!N(TEXT("default_container_mass_kg"),ContainerMassKg,1,false))return false;
    if(!N(TEXT("spreader_mass_kg"),SpreaderMassKg,1,false))return false;
    if(!N(TEXT("sensor_max_age_s"),SensorMaxAge,1,false))return false;
    if(!N(TEXT("landing_tolerance_m"),LandingTolerance,100,false))return false;
    if(!N(TEXT("seating_vertical_tolerance_m"),SeatingTolerance,100,false))return false;
    if(!N(TEXT("settle_speed_mps"),SettleSpeed,100,false))return false;
    if(!N(TEXT("settle_time_s"),SettleTime,1,false))return false;
    if(!N(TEXT("agv_position_tolerance_m"),AGVTolerance,100,false))return false;
    if(!N(TEXT("agv_heading_tolerance_deg"),AGVHeadingTolerance,1,false))return false;
    if(!N(TEXT("support_tolerance_m"),SupportTolerance,100,false))return false;
    if(!N(TEXT("landing_speed_mps"),ApproachSpeed,100,false))return false;
    if(!N(TEXT("landing_slowdown_distance_m"),ApproachDistance,100,false))return false;
    if(!N(TEXT("stage_timeout_s"),StageTimeout,1,false))return false;
    if(!N(TEXT("landing_sway_limit_deg"),SwayLimitDegrees,1,false))return false;
    if(!N(TEXT("collision_margin_m"),CollisionMargin,100,false))return false;
    if(!N(TEXT("stack_height_tolerance_m"),StackHeightTolerance,100,false))return false;
    if(!Pickup.Load(Root,Error)||!Dynamics.Load(Root,Error))return false;
    float Hz=0,Loaded=0;
    if(!N(TEXT("sensor_sample_hz"),Hz)||!N(TEXT("loaded_hoist_m_min"),Loaded,100./60.))return false;
    // Selected configuration must remain within the STD ranges on brochure p5.
    if(RatedPayloadKg<40000||RatedPayloadKg>50000||RailGauge>4800||Outreach>1800||LiftAboveRail>1900||
       Loaded<50||Loaded>75||EmptyHoistSpeed<100||EmptyHoistSpeed>150||TrolleySpeed<200||TrolleySpeed>250||GantrySpeed<200||GantrySpeed>250||
       SafeHeight>LiftAboveRail||BeamHeight<=LiftAboveRail||Hz>240||SensorMaxAge<1./Hz||StageTimeout<2*Pickup.AlignmentTimeout)
    {Error=TEXT("RMG selection outside brochure ranges or inconsistent timing/geometry");return false;}
    SensorPeriod=1./Hz;Backreach=Outreach;WatersideRailX=-RailGauge*.5;LiftBelowRail=0;
    MinRope=BeamHeight-LiftAboveRail;MaxRope=BeamHeight;LoadUnitKg=1000;
    LoadedHoistCurve.Add(FVector2D(1,Loaded));LoadedHoistCurve.Add(FVector2D(RatedPayloadKg,Loaded));
    // These biases are simulation inputs, independent of manufacturer data.
    for(const auto& Entry:TArray<TPair<FString,float*>>{{TEXT("position_sensor_bias_m"),&PositionBias},{TEXT("load_estimate_bias_kg"),&MassBiasKg}})
    {
        double V=0;if(!Root->TryGetNumberField(Entry.Key,V)||!FMath::IsFinite(V)||FMath::Abs(V)>1000){Error=TEXT("Invalid RMG sensor bias");return false;}
        *Entry.Value=V*(Entry.Value==&PositionBias?100:1);
    }
    for(int32 I=0;I<3;++I)
    {
        const TCHAR* Key=I==0?TEXT("container_cog_x_m"):I==1?TEXT("container_cog_y_m"):TEXT("container_cog_z_m");
        double V=0;if(!Root->TryGetNumberField(Key,V)||!FMath::IsFinite(V)||FMath::Abs(V)>=(I==0?1.:I==1?5.2:1.295)){Error=TEXT("Invalid RMG default CoG");return false;}
        ContainerCoG[I]=V*100;
    }
    if(LandingTolerance>=100||SeatingTolerance>=50||SwayLimitDegrees>=10||CollisionMargin>1000||StackHeightTolerance>50)
    {Error=TEXT("Unsafe RMG observation tolerances");return false;}
    for(const auto& M:Dynamics.Mounts)SensorKeys.Add(M.Key);
    for(const TCHAR* Key:{TEXT("trolley_encoder"),TEXT("hoist_encoder"),TEXT("gantry_encoder"),TEXT("sway_sensor"),TEXT("spreader_camera"),TEXT("twistlock_load"),TEXT("twistlock_state"),TEXT("landed"),TEXT("agv_position_lidar"),TEXT("stack_profile"),TEXT("crane_collision_front"),TEXT("crane_collision_rear")})
        if(!SensorKeys.Contains(Key)){Error=FString(TEXT("Required RMG sensor missing: "))+Key;return false;}
    const TSharedPtr<FJsonObject>* Ref=nullptr;
    if(!Root->TryGetObjectField(TEXT("reference"),Ref)||!(*Ref)->TryGetStringField(TEXT("document"),ReferencePath))
    {Error=TEXT("RMG reference provenance missing");return false;}
    auto Snapshot=MakeShared<FJsonObject>();Snapshot->SetObjectField(TEXT("reference"),*Ref);
    Snapshot->SetObjectField(TEXT("simulation_assumptions"),Root);
    FJsonSerializer::Serialize(Snapshot,TJsonWriterFactory<>::Create(&SnapshotJson));
    bReady=true;return true;
}
