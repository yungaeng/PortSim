#include "QuayCrane.h"
#include "PortEnvironmentComponent.h"
#include "PortSiteLogistics.h"
#include "TerminalLayout.h"
#include "Misc/Paths.h"
#include "Misc/FileHelper.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "HAL/FileManager.h"
#include "Dom/JsonObject.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"

namespace
{
FString RunSettingsPath()
{
    FString Path=FPaths::ProjectSavedDir()/TEXT("SimulationSettings.json");
    FParse::Value(FCommandLine::Get(),TEXT("PortSimSettings="),Path);
    return Path;
}
}

int32 AQuayCrane::PendingCargoTotal() const
{ int32 Total=0; for(int32 I=0;I<PendingShips;++I) Total+=PendingCargo[I]; return Total; }
int32 AQuayCrane::PendingCargoLimit(int32 Vessel) const
{ return FMath::Min(TerminalLayout::ContainersPerVessel,TerminalLayout::ReceivingSlotCount(bUnifiedTerminal)-PendingCargoTotal()+PendingCargo[Vessel]); }
void AQuayCrane::SetPendingShips(int32 Count)
{
    const int32 Old=PendingShips;
    PendingShips=FMath::Clamp(Count,1,3);
    for(int32 I=Old;I<PendingShips;++I) PendingCargo[I]=GetPendingVesselInput(I)?GetPendingVesselInput(I)->Rows.Num():ManualCargo[I];
    int32 Excess=PendingCargoTotal()-TerminalLayout::ReceivingSlotCount(bUnifiedTerminal);
    for(int32 I=0;I<PendingShips && Excess>0;++I)
    { if(GetPendingVesselInput(I)) continue; const int32 Take=FMath::Min(Excess,PendingCargo[I]-1); PendingCargo[I]-=Take; Excess-=Take; }
}
void AQuayCrane::SetPendingCargo(int32 Vessel,int32 Count)
{
    if(Vessel>=0 && Vessel<PendingShips && !PendingFileMode[Vessel])
    {
        PendingCargo[Vessel]=FMath::Clamp(Count,1,PendingCargoLimit(Vessel));
        ManualCargo[Vessel]=PendingCargo[Vessel];
    }
}
void AQuayCrane::LoadRunSettings()
{
    FString Json; TSharedPtr<FJsonObject> Object;
    if(FFileHelper::LoadFileToString(Json,*RunSettingsPath()) &&
        FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Json),Object) && Object.IsValid())
    {
        double Ships=3; Object->TryGetNumberField(TEXT("ships"),Ships);
        PendingShips=FMath::IsFinite(Ships)?int32(FMath::Clamp(Ships,1.,3.)):3;
        int32 Budget=TerminalLayout::ReceivingSlotCount(bUnifiedTerminal);
        for(int32 I=0;I<3;++I)
        {
            double Count=200; Object->TryGetNumberField(FString::Printf(TEXT("cargo_%d"),I+1),Count);
            PendingCargo[I]=FMath::IsFinite(Count)?int32(FMath::Clamp(Count,1.,double(FMath::Min(TerminalLayout::ContainersPerVessel,I<PendingShips?Budget-(PendingShips-I-1):TerminalLayout::ReceivingSlotCount(bUnifiedTerminal))))):1;
            if(I<PendingShips) Budget-=PendingCargo[I];
        }
        if(Environment && !FParse::Param(FCommandLine::Get(),TEXT("unattended")))
        {
            bool Enabled=false; double Value=0;
            if(Object->TryGetBoolField(TEXT("wind_enabled"),Enabled)) Environment->SetWindEnabled(Enabled);
            if(Object->TryGetNumberField(TEXT("wind_speed_mps"),Value)) Environment->SetWindSpeedMetersPerSecond(Value);
            if(Object->TryGetBoolField(TEXT("sea_enabled"),Enabled)) Environment->SetSeaMotionEnabled(Enabled);
            if(Object->TryGetNumberField(TEXT("sea_intensity"),Value)) Environment->SetSeaIntensity(Value);
            if(Object->TryGetBoolField(TEXT("rain_enabled"),Enabled)) Environment->SetRainEnabled(Enabled);
            if(Object->TryGetNumberField(TEXT("rainfall_mm_per_hour"),Value)) Environment->SetRainfallMmPerHour(Value);
            if(Object->TryGetBoolField(TEXT("snow_enabled"),Enabled)) Environment->SetSnowEnabled(Enabled);
            if(Object->TryGetNumberField(TEXT("snow_intensity"),Value)) Environment->SetSnowIntensity(Value);
            if(Object->TryGetNumberField(TEXT("snowfall_mm_per_hour"),Value)) Environment->SetSnowfallMmPerHour(Value);
            if(Object->TryGetBoolField(TEXT("ice_enabled"),Enabled)) Environment->SetIceEnabled(Enabled);
            if(Object->TryGetNumberField(TEXT("ice_intensity"),Value)) Environment->SetIceIntensity(Value);
            if(Object->TryGetBoolField(TEXT("fog_enabled"),Enabled)) Environment->SetFogEnabled(Enabled);
            if(Object->TryGetNumberField(TEXT("fog_visibility_m"),Value)) Environment->SetFogVisibilityMeters(Value);
        }
    }
    for(int32 I=0;I<3;++I)
    {
        ManualCargo[I]=PendingCargo[I];
        if(Object.IsValid())
        {
            Object->TryGetBoolField(FString::Printf(TEXT("file_mode_%d"),I+1),PendingFileMode[I]);
            double Manual=ManualCargo[I];
            if(Object->TryGetNumberField(FString::Printf(TEXT("manual_cargo_%d"),I+1),Manual) && FMath::IsFinite(Manual))
                ManualCargo[I]=int32(FMath::Clamp(Manual,1.,double(FMath::Min(TerminalLayout::ContainersPerVessel,TerminalLayout::ReceivingSlotCount(bUnifiedTerminal)))));
        }
    }
    if(bUnifiedTerminal && Object.IsValid()) RestoreVesselInputs(*Object);
    // Command-line overrides allow repeatable tests without changing saved settings.
    int32 Ships=PendingShips;
    FParse::Value(FCommandLine::Get(),TEXT("PortSimShips="),Ships); SetPendingShips(Ships);
    for(int32 I=0;I<PendingShips;++I)
    {
        int32 Count=PendingCargo[I];
        if(FParse::Value(FCommandLine::Get(),*FString::Printf(TEXT("PortSimCargo%d="),I+1),Count))
        { PendingFileMode[I]=false; SetPendingCargo(I,Count); }
    }
    if(bUnifiedTerminal && !ApplyPendingVesselInputs())
    {
        // Invalid saved combinations must not partially apply to the live scene.
        for(int32 I=0;I<3;++I) PendingFileMode[I]=false;
        SetPendingShips(PendingShips);
        ApplyPendingVesselInputs();
    }
    if(Environment)
    {
        float Value=0;
        if(FParse::Value(FCommandLine::Get(),TEXT("PortSimWindSpeed="),Value)) Environment->SetWindSpeedMetersPerSecond(Value);
        if(FParse::Value(FCommandLine::Get(),TEXT("PortSimSeaIntensity="),Value)) Environment->SetSeaIntensity(Value);
        if(FParse::Value(FCommandLine::Get(),TEXT("PortSimFogVisibility="),Value)) Environment->SetFogVisibilityMeters(Value);
        if(FParse::Value(FCommandLine::Get(),TEXT("PortSimRainRate="),Value)) Environment->SetRainfallMmPerHour(Value);
        if(FParse::Value(FCommandLine::Get(),TEXT("PortSimSnowIntensity="),Value)) Environment->SetSnowIntensity(Value);
        if(FParse::Value(FCommandLine::Get(),TEXT("PortSimSnowRate="),Value)) Environment->SetSnowfallMmPerHour(Value);
        if(FParse::Value(FCommandLine::Get(),TEXT("PortSimIceIntensity="),Value)) Environment->SetIceIntensity(Value);
        if(FParse::Param(FCommandLine::Get(),TEXT("PortSimWind"))) Environment->SetWindEnabled(true);
        if(FParse::Param(FCommandLine::Get(),TEXT("PortSimNoWind"))) Environment->SetWindEnabled(false);
        if(FParse::Param(FCommandLine::Get(),TEXT("PortSimSea"))) Environment->SetSeaMotionEnabled(true);
        if(FParse::Param(FCommandLine::Get(),TEXT("PortSimNoSea"))) Environment->SetSeaMotionEnabled(false);
        if(FParse::Param(FCommandLine::Get(),TEXT("PortSimFog"))) Environment->SetFogEnabled(true);
        if(FParse::Param(FCommandLine::Get(),TEXT("PortSimNoFog"))) Environment->SetFogEnabled(false);
        if(FParse::Param(FCommandLine::Get(),TEXT("PortSimRain"))) Environment->SetRainEnabled(true);
        if(FParse::Param(FCommandLine::Get(),TEXT("PortSimNoRain"))) Environment->SetRainEnabled(false);
        if(FParse::Param(FCommandLine::Get(),TEXT("PortSimSnowWeather"))) Environment->SetSnowEnabled(true);
        if(FParse::Param(FCommandLine::Get(),TEXT("PortSimNoSnow"))) Environment->SetSnowEnabled(false);
        if(FParse::Param(FCommandLine::Get(),TEXT("PortSimIcing"))) Environment->SetIceEnabled(true);
        if(FParse::Param(FCommandLine::Get(),TEXT("PortSimNoIce"))) Environment->SetIceEnabled(false);
    }
    ConfiguredShips=PendingShips;
    for(int32 I=0;I<3;++I) ConfiguredCargo[I]=PendingCargo[I];
}
bool AQuayCrane::SaveRunSettings()
{
    auto Object=MakeShared<FJsonObject>(); Object->SetNumberField(TEXT("ships"),PendingShips);
    for(int32 I=0;I<3;++I)
    {
        Object->SetNumberField(FString::Printf(TEXT("cargo_%d"),I+1),PendingCargo[I]);
        Object->SetNumberField(FString::Printf(TEXT("manual_cargo_%d"),I+1),ManualCargo[I]);
        Object->SetBoolField(FString::Printf(TEXT("file_mode_%d"),I+1),PendingFileMode[I]);
    }
    SaveVesselInputs(*Object);
    if(Environment)
    {
        Object->SetBoolField(TEXT("wind_enabled"),Environment->IsWindEnabled());
        Object->SetNumberField(TEXT("wind_speed_mps"),Environment->GetWindSettingMetersPerSecond());
        Object->SetBoolField(TEXT("sea_enabled"),Environment->IsSeaMotionEnabled());
        Object->SetNumberField(TEXT("sea_intensity"),Environment->GetSeaIntensity());
        Object->SetBoolField(TEXT("rain_enabled"),Environment->IsRainEnabled());
        Object->SetNumberField(TEXT("rainfall_mm_per_hour"),Environment->GetRainfallMmPerHour());
        Object->SetBoolField(TEXT("snow_enabled"),Environment->IsSnowEnabled());
        Object->SetNumberField(TEXT("snow_intensity"),Environment->GetSnowIntensity());
        Object->SetNumberField(TEXT("snowfall_mm_per_hour"),Environment->GetSnowfallMmPerHour());
        Object->SetBoolField(TEXT("ice_enabled"),Environment->IsIceEnabled());
        Object->SetNumberField(TEXT("ice_intensity"),Environment->GetIceIntensity());
        Object->SetBoolField(TEXT("fog_enabled"),Environment->IsFogEnabled());
        Object->SetNumberField(TEXT("fog_visibility_m"),Environment->GetFogVisibilityMeters());
    }
    FString Json; FJsonSerializer::Serialize(Object,TJsonWriterFactory<>::Create(&Json));
    const FString Path=RunSettingsPath();
    bRunSettingsSaved=FFileHelper::SaveStringToFile(Json,*(Path+TEXT(".tmp")),FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM) &&
        IFileManager::Get().Move(*Path,*(Path+TEXT(".tmp")),true,true);
    return bRunSettingsSaved;
}
void AQuayCrane::AdjustStartTime(bool bForward)
{
    if(!CanAdjustStartTime()) return;
    Environment->ShiftStartTimeByHalfHour(bForward);
    LastSolarUpdate=FDateTime();
}

void AQuayCrane::StopSimulation()
{
    if(bUnifiedTerminal && !ApplyPendingVesselInputs()) return;
    SaveRunSettings();
    if (bUnifiedTerminal)
    {
        bAutoRunning=false; bAutoPaused=false;
        DestroyTerminalActors();
        ConfiguredShips=PendingShips;
        for(int32 I=0;I<3;++I) ConfiguredCargo[I]=PendingCargo[I];
        BuildTerminalSite();
        BuildConfiguredVessels();
        SiteCameraIndex=-1;
    }
    ResetSimulation(); bAutoRunning=false; bAutoPaused=false;
    AutoElapsed=0; Status=TEXT("리셋 완료 — 설정 적용됨. 실행 버튼으로 시작하세요.");
    if(SiteLogistics)
    {
        // Recreated equipment must inherit the retained weather and surface
        // before the forced reset snapshot; the next actor tick is too late.
        if(Environment)
        {
            SiteLogistics->SetWindVelocity(Environment->GetEffectiveWindVelocity());
            SiteLogistics->SetFogVisibilityMeters(Environment->IsFogEnabled()?Environment->GetFogVisibilityMeters():MAX_flt);
            SiteLogistics->SetRainState(Environment->GetRainState());
            auto* Material=Environment->GetContactMaterial();
            SiteLogistics->SetSurfaceConditions(Environment->GetSurfaceConditions(),Environment->GetFrictionSettings(),Material);
        }
        SiteLogistics->ExportDashboard(true,true);
    }
}
FString AQuayCrane::ElapsedText() const
{
    const int64 Seconds=FMath::Max<int64>(0,int64(AutoElapsed));
    return FString::Printf(TEXT("%02lld시 %02lld분 %02lld초"),Seconds/3600,(Seconds/60)%60,Seconds%60);
}

