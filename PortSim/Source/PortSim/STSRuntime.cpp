#include "QuayCrane.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/World.h"
#include "Misc/Paths.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "Misc/FileHelper.h"
#include "HAL/FileManager.h"

DEFINE_LOG_CATEGORY_STATIC(LogSTS,Log,All);

void AQuayCrane::InitializeSTSProfile()
{
    if(!bTerminalMode) return; // Preserve the isolated legacy crane regression exercise.
    FString Reference=FPaths::ConvertRelativePathToFull(FPaths::ProjectDir()/TEXT("../Document/STS/STS_ReferenceData.json"));
    if(!FPaths::FileExists(Reference)) Reference=FPaths::ProjectContentDir()/TEXT("STS/STS_ReferenceData.json");
    FString Settings=FPaths::ProjectConfigDir()/TEXT("STS_Simulation.json");
    FParse::Value(FCommandLine::Get(),TEXT("PortSimSTSReference="),Reference);
    FParse::Value(FCommandLine::Get(),TEXT("PortSimSTSSettings="),Settings);
    if(!STSProfile.Load(Reference,Settings))
    {
        UE_LOG(LogSTS,Error,TEXT("STS_PROFILE_INVALID: %s"),*STSProfile.Error);
        return;
    }
    bSTSSensorFault=FParse::Param(FCommandLine::Get(),TEXT("PortSimSTSSensorFault"));
    FParse::Value(FCommandLine::Get(),TEXT("PortSimSTSLockFault="),STSLockFault);
    TravelSpeed=STSProfile.TrolleySpeed;
    Acceleration=STSProfile.TrolleyAcceleration;
    HoistSpeed=STSProfile.EmptyHoistSpeed;
    Spreader->SetMassOverrideInKg(NAME_None,STSProfile.SpreaderMassKg);
    ApplySTSGeometry();
    bSTSStartPending=STSProfile.bAutoStart && !bTerminalTest && !FParse::Param(FCommandLine::Get(),TEXT("PortSimManualStart")) && !FParse::Param(FCommandLine::Get(),TEXT("PortSimRateTest")) && !FParse::Param(FCommandLine::Get(),TEXT("PortSimSiteTest"));
    UE_LOG(LogSTS,Display,TEXT("STS_PROFILE_APPLIED: J5300 gauge=%.2fm reach=%.2f/%.2fm trolley=%.2fm/s gantry=%.2fm/s loaded12t=%.2fcm/s sensors=%d"),
        STSProfile.RailGauge/100,STSProfile.Outreach/100,STSProfile.Backreach/100,TravelSpeed/100,STSGantrySpeed()/100,STSProfile.HoistLimit(12000,true),STSProfile.SensorKeys.Num());
    UE_LOG(LogSTS,Warning,TEXT("STS_SIMULATION_ASSUMPTIONS: LT=%.6fkg provisionally; support height=%.2fm; equivalent suspension, level spreader, synthetic sensors/static corner estimates; no wire/torque/skew actuator solver. Settings: %s"),
        STSProfile.LoadUnitKg,STSProfile.BeamHeight/100,*Settings);
}

void AQuayCrane::ApplySTSGeometry()
{
    const float Left=STSProfile.WatersideRailX, Right=Left+STSProfile.RailGauge;
    TInlineComponentArray<UStaticMeshComponent*> Meshes(this);
    for(auto* Mesh:Meshes)
    {
        const FString N=Mesh->GetName(); FVector P=Mesh->GetRelativeLocation(); FVector Size=Mesh->GetRelativeScale3D()*100;
        if(N.StartsWith(TEXT("Leg_")))
        { P.X=N.StartsWith(TEXT("Leg_-1_"))?Left:Right; P.Z=(STSProfile.BeamHeight-100)/2; Size.Z=STSProfile.BeamHeight-100; }
        else if(N.StartsWith(TEXT("Bogie_"))) P.X=N.StartsWith(TEXT("Bogie_-1_"))?Left:Right;
        else if(N.StartsWith(TEXT("CrossBeam_"))) {P.X=N==TEXT("CrossBeam_-1")?Left:Right; P.Z=STSProfile.BeamHeight-100;}
        else if(N.StartsWith(TEXT("Boom_")))
        {P.X=(STSProfile.MinTrolley()+STSProfile.MaxTrolley())/2;P.Z=STSProfile.BeamHeight+100;Size.X=STSProfile.MaxTrolley()-STSProfile.MinTrolley()+400;}
        else continue;
        Mesh->SetRelativeLocation(P); Mesh->SetRelativeScale3D(Size/100);
    }
}

float AQuayCrane::GetCargoMassKg() const { return Cargo?Cargo->GetMass():0.f; }
float AQuayCrane::STSBeamHeight() const { return STSProfile.bReady?STSProfile.BeamHeight:3000.f; }
float AQuayCrane::STSTransferHeight() const { return STSProfile.bReady?STSProfile.SafeHeight:1900.f; }
float AQuayCrane::STSGantrySpeed() const { return STSProfile.bReady?STSProfile.GantrySpeed:TravelSpeed*0.6f; }
float AQuayCrane::CurrentHoistLimit() const { return STSProfile.bReady?STSProfile.HoistLimit(bLocked?GetCargoMassKg():0,bLocked):HoistSpeed; }

float AQuayCrane::AutomaticHoistLimit() const
{
    const float Limit=CurrentHoistLimit();
    if(!STSProfile.bReady || !bAutoRunning || (AutoStage!=ETerminalStage::LowerPickup && AutoStage!=ETerminalStage::LowerPlace)) return Limit;
    const float TargetZ=(AutoStage==ETerminalStage::LowerPickup?STSObservation.CargoPosition.Z:AutoDestination.Z)+154.5f;
    const float Distance=FMath::Abs(STSObservation.SpreaderPosition.Z-TargetZ);
    // Decelerate over the available stopping distance; use landing speed only near contact.
    return FMath::Min(Limit,FMath::Sqrt(FMath::Square(STSProfile.ApproachSpeed)+
        2.f*STSProfile.HoistAcceleration*FMath::Max(0.f,Distance-STSProfile.ApproachDistance)));
}

bool AQuayCrane::IsCargoSupported() const
{
    if(!Cargo) return false;
    FCollisionQueryParams Params(SCENE_QUERY_STAT(STSSupport),false);
    Params.AddIgnoredActor(this); Params.AddIgnoredActor(Cargo->GetOwner());
    const FVector P=Cargo->GetComponentLocation();
    for(int32 I=0;I<4;++I)
    {
        const FVector Corner= P+Cargo->GetComponentQuat().RotateVector(FVector((I&1)?100:-100,(I&2)?520:-520,0));
        FHitResult Hit;
        if(!GetWorld()->LineTraceSingleByChannel(Hit,Corner,Corner-FVector(0,0,129.5f+STSProfile.SupportTolerance),ECC_Visibility,Params) || Hit.ImpactNormal.Z<0.8f) return false;
    }
    return true;
}

void AQuayCrane::SampleSTSSensors(bool Force)
{
    if(!STSProfile.bReady || !Cargo || (!Force && STSSimulationTime<NextSTSSample)) return;
    NextSTSSample=STSSimulationTime+STSProfile.SensorPeriod;
    FSTSObservation& S=STSObservation;
    S.Timestamp=STSSimulationTime;
    S.bValid=!bSTSSensorFault;
    S.DrivePosition=FVector(TrolleyPosition,GantryPosition,RopeLength)+FVector(STSProfile.PositionBias);
    S.DriveVelocity=DriveVelocity;
    S.SpreaderPosition=Spreader->GetComponentLocation()+FVector(STSProfile.PositionBias);
    S.SpreaderVelocity=Spreader->GetPhysicsLinearVelocity();
    S.CargoPosition=Cargo->GetComponentLocation();
    S.CargoVelocity=Cargo->GetPhysicsLinearVelocity();
    S.SwayDegrees=GetSwayDegrees();
    const FVector Gap=S.SpreaderPosition-S.CargoPosition;
    S.bLanded=FMath::Abs(Gap.X)<=STSProfile.LandingTolerance && FMath::Abs(Gap.Y)<=STSProfile.LandingTolerance &&
        FMath::Abs(Gap.Z-154.5f)<=STSProfile.SeatingTolerance &&
        (S.SpreaderVelocity-S.CargoVelocity).Size()<=STSProfile.SettleSpeed &&
        FQuat::ErrorAutoNormalize(Spreader->GetComponentQuat(),Cargo->GetComponentQuat())<0.02f;
    const auto* Container=ContainerActors.IsValidIndex(ActiveCargoIndex)?ContainerActors[ActiveCargoIndex].Get():nullptr;
    const FVector CoG=Container?Container->CoGOffsetCm:FVector::ZeroVector;
    const float Weight=bLocked?FMath::Max(0.f,GetCargoMassKg()+STSProfile.MassBiasKg)*9.80665f:0;
    for(int32 I=0;I<4;++I)
    {
        S.Locked[I]=STSCornerLocked[I];
        S.CornerLoadsN[I]=Weight*(0.5f+((I&1)?1:-1)*CoG.X/200.f)*(0.5f+((I&2)?1:-1)*CoG.Y/1040.f);
    }
    S.bCargoSupported=IsCargoSupported();
    S.bAGVAligned=false;
    if(AGVActors.IsValidIndex(ActiveAGV))
    {
        const auto* Vehicle=AGVActors[ActiveAGV].Get();
        const FVector Expected=bAutoLoading?AutoSource:AutoDestination;
        S.bAGVAligned=Vehicle->Speed<=0.1f && FVector::Dist(AGVCargoPosition(),Expected)<=STSProfile.AGVTolerance &&
            FMath::Abs(FMath::FindDeltaAngleDegrees(Vehicle->GetActorRotation().Yaw,0.f))<=STSProfile.AGVHeadingTolerance;
    }
}

bool AQuayCrane::STSLoadedHoistAllowed() const
{
    return STSObservation.IsFresh(STSSimulationTime,STSProfile.SensorMaxAge) && STSObservation.AllLocked() &&
        STSObservation.PayloadEstimateKg()<=STSProfile.RatedPayloadKg && GetCargoMassKg()<=STSProfile.RatedPayloadKg;
}

void AQuayCrane::RecordSTSStage(const TCHAR* Outcome)
{
    if(ResultsPath.IsEmpty() || AutoStage==ETerminalStage::Idle) return;
    StageEventsCsv+=FString::Printf(TEXT("C%02d,%s,%s,%.6f,%.6f,%s\n"),ActiveCargoIndex+1,bAutoLoading?TEXT("Load"):TEXT("Unload"),
        GetAutoStageName(),AutoElapsed,AutoStageTime,Outcome);
}

bool AQuayCrane::SaveSTSReports() const
{
    if(ResultsPath.IsEmpty()) return true;
    IFileManager::Get().MakeDirectory(*FPaths::GetPath(ResultsPath),true);
    const FString Base=ResultsPath.LeftChop(4);
    return FFileHelper::SaveStringToFile(ResultsCsv,*ResultsPath,FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM) &&
        FFileHelper::SaveStringToFile(StageEventsCsv,*(Base+TEXT("_stages.csv")),FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM) &&
        FFileHelper::SaveStringToFile(STSProfile.SnapshotJson,*(Base+TEXT("_profile.json")),FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM);
}

FString AQuayCrane::GetSTSStatus() const
{
    if(!STSProfile.bReady) return TEXT("STS reference unavailable: ")+STSProfile.Error;
    return FString::Printf(TEXT("J5300 | trolley %.1f / gantry %.1f / hoist %.2f m/s | locks %d/4 | sensors %s"),
        TravelSpeed/100,STSGantrySpeed()/100,CurrentHoistLimit()/100,
        int(STSObservation.Locked[0])+int(STSObservation.Locked[1])+int(STSObservation.Locked[2])+int(STSObservation.Locked[3]),
        STSObservation.IsFresh(STSSimulationTime,STSProfile.SensorMaxAge)?TEXT("OK"):TEXT("INVALID"));
}
