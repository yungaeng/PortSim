#include "AGVThroughputController.h"
#include "Dom/JsonObject.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "Misc/FileHelper.h"

bool FAGVThroughputConfig::Load(const FString& Path,FString& Error)
{
    FString Text;
    TSharedPtr<FJsonObject> Root;
    if(!FFileHelper::LoadFileToString(Text,*Path) ||
        !FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text),Root) || !Root.IsValid())
    {
        Error=FString::Printf(TEXT("Could not read AGV throughput controller settings: %s"),*Path);
        return false;
    }

    auto ReadNumber=[&](const TCHAR* Key,double& Value)
    {
        double Parsed=0.;
        if(!Root->TryGetNumberField(Key,Parsed))
        {
            Error=FString::Printf(TEXT("AGV throughput controller setting '%s' must be numeric"),Key);
            return false;
        }
        Value=Parsed;
        return true;
    };
    auto ReadInteger=[&](const TCHAR* Key,int32& Value)
    {
        double Parsed=0.;
        if(!Root->TryGetNumberField(Key,Parsed) || !FMath::IsFinite(Parsed) ||
            Parsed!=FMath::FloorToDouble(Parsed) || Parsed<double(MIN_int32) || Parsed>double(MAX_int32))
        {
            Error=FString::Printf(TEXT("AGV throughput controller setting '%s' must be an integer"),Key);
            return false;
        }
        Value=int32(Parsed);
        return true;
    };

    if(!Root->TryGetBoolField(TEXT("enabled"),bEnabled) ||
        !ReadInteger(TEXT("step_size"),StepSize) ||
        !ReadNumber(TEXT("settle_sec"),SettleSec) ||
        !ReadNumber(TEXT("window_sec"),WindowSec) ||
        !ReadNumber(TEXT("min_gain_ratio"),MinGainRatio) ||
        !ReadInteger(TEXT("max_agv"),MaxAGV) ||
        !ReadNumber(TEXT("battery_return_percent"),BatteryReturnPercent) ||
        !ReadNumber(TEXT("reprobe_after_sec"),ReprobeAfterSec) ||
        !ReadNumber(TEXT("reprobe_drop_ratio"),ReprobeDropRatio))
    {
        if(Error.IsEmpty()) Error=TEXT("AGV throughput controller setting 'enabled' must be boolean");
        return false;
    }
    if(StepSize<=0 || MaxAGV<=0 || MaxAGV>60 ||
        !FMath::IsFinite(SettleSec) || SettleSec<0. ||
        !FMath::IsFinite(WindowSec) || WindowSec<=0. ||
        !FMath::IsFinite(MinGainRatio) || MinGainRatio<0. ||
        !FMath::IsFinite(BatteryReturnPercent) || BatteryReturnPercent<0. || BatteryReturnPercent>=80. ||
        !FMath::IsFinite(ReprobeAfterSec) || ReprobeAfterSec<=0. ||
        !FMath::IsFinite(ReprobeDropRatio) || ReprobeDropRatio<=0. || ReprobeDropRatio>=1.)
    {
        Error=TEXT("AGV throughput controller settings are outside their valid ranges");
        return false;
    }
    return true;
}

void FAGVThroughputController::Initialize(const FAGVThroughputConfig& InConfig,int32 FleetSize,
    const TArray<int32>& STSCompletedJobs)
{
    Config=InConfig;
    // Start with one probe-size reserve so STS/RMG work can begin overlapping
    // AGV approach immediately instead of waiting for the first throughput
    // window just to create the first active fleet.
    TargetAGV=Config.bEnabled?
        FMath::Clamp(Config.StepSize,1,FMath::Min(Config.MaxAGV,FleetSize)):
        FMath::Min(Config.MaxAGV,FleetSize);
    Stage=Config.bEnabled?EAGVThroughputStage::Settling:EAGVThroughputStage::Saturated;
    LastAcceptedAGV=TargetAGV;
    SaturatedAGV=0;
    ReferenceThroughput=RecentThroughput=SaturatedThroughput=0.;
    StageElapsed=WindowElapsed=SaturatedElapsed=0.;
    WindowCompleted=0;
    bHasReference=false;
    LastSTSCompletedJobs=STSCompletedJobs;
    ActiveFleet.Init(false,FleetSize);
    if(!Config.bEnabled)
        ActiveFleet.Init(true,FleetSize);
}

int32 FAGVThroughputController::CompletedDelta(const TArray<int32>& STSCompletedJobs)
{
    if(LastSTSCompletedJobs.Num()!=STSCompletedJobs.Num())
    {
        LastSTSCompletedJobs=STSCompletedJobs;
        return 0;
    }
    int32 Delta=0;
    for(int32 I=0;I<STSCompletedJobs.Num();++I)
    {
        Delta+=FMath::Max(0,STSCompletedJobs[I]-LastSTSCompletedJobs[I]);
        LastSTSCompletedJobs[I]=STSCompletedJobs[I];
    }
    return Delta;
}

void FAGVThroughputController::BeginProbe()
{
    TargetAGV=FMath::Min(Config.MaxAGV,TargetAGV+Config.StepSize);
    Stage=EAGVThroughputStage::Settling;
    StageElapsed=WindowElapsed=SaturatedElapsed=0.;
    WindowCompleted=0;
}

void FAGVThroughputController::BeginSettling(bool bResetReference)
{
    Stage=EAGVThroughputStage::Settling;
    StageElapsed=WindowElapsed=0.;
    WindowCompleted=0;
    if(bResetReference)
    {
        bHasReference=false;
        ReferenceThroughput=0.;
    }
}

void FAGVThroughputController::FinishWindow()
{
    RecentThroughput=WindowElapsed>0.?double(WindowCompleted)*3600./WindowElapsed:0.;
    WindowElapsed=0.;
    WindowCompleted=0;

    if(!bHasReference)
    {
        ReferenceThroughput=RecentThroughput;
        LastAcceptedAGV=TargetAGV;
        bHasReference=true;
        if(TargetAGV>=Config.MaxAGV)
        {
            SaturatedAGV=TargetAGV;
            SaturatedThroughput=RecentThroughput;
            Stage=EAGVThroughputStage::Saturated;
            StageElapsed=SaturatedElapsed=0.;
        }
        else
        {
            Stage=EAGVThroughputStage::Probing;
            StageElapsed=0.;
        }
        return;
    }

    const bool Improved=ReferenceThroughput<=0.?RecentThroughput>0.:
        RecentThroughput>=ReferenceThroughput*(1.+Config.MinGainRatio);
    if(Improved)
    {
        ReferenceThroughput=RecentThroughput;
        LastAcceptedAGV=TargetAGV;
        if(TargetAGV>=Config.MaxAGV)
        {
            SaturatedAGV=TargetAGV;
            SaturatedThroughput=RecentThroughput;
            Stage=EAGVThroughputStage::Saturated;
            StageElapsed=SaturatedElapsed=0.;
        }
        else
        {
            Stage=EAGVThroughputStage::Probing;
            StageElapsed=0.;
        }
        return;
    }

    SaturatedAGV=TargetAGV;
    SaturatedThroughput=RecentThroughput;
    ReferenceThroughput=RecentThroughput;
    Stage=EAGVThroughputStage::Saturated;
    StageElapsed=SaturatedElapsed=0.;
}

void FAGVThroughputController::Advance(double Dt,const TArray<int32>& STSCompletedJobs,bool bInvalidWindow)
{
    const int32 NewCompletions=CompletedDelta(STSCompletedJobs);
    if(!Config.bEnabled || !FMath::IsFinite(Dt) || Dt<=0.) return;

    if(bInvalidWindow)
    {
        if(Stage==EAGVThroughputStage::Settling || Stage==EAGVThroughputStage::Measuring)
            BeginSettling(false);
        else if(Stage==EAGVThroughputStage::Saturated)
        {
            WindowElapsed=0.;
            WindowCompleted=0;
        }
        return;
    }

    switch(Stage)
    {
    case EAGVThroughputStage::Probing:
        BeginProbe();
        break;
    case EAGVThroughputStage::Settling:
        StageElapsed+=Dt;
        if(StageElapsed>=Config.SettleSec)
        {
            Stage=EAGVThroughputStage::Measuring;
            StageElapsed=0.;
            WindowElapsed=0.;
            WindowCompleted=0;
        }
        break;
    case EAGVThroughputStage::Measuring:
        WindowElapsed+=Dt;
        WindowCompleted+=NewCompletions;
        if(WindowElapsed>=Config.WindowSec) FinishWindow();
        break;
    case EAGVThroughputStage::Saturated:
        SaturatedElapsed+=Dt;
        WindowElapsed+=Dt;
        WindowCompleted+=NewCompletions;
        if(WindowElapsed>=Config.WindowSec)
        {
            RecentThroughput=double(WindowCompleted)*3600./WindowElapsed;
            WindowElapsed=0.;
            WindowCompleted=0;
            if(SaturatedThroughput>0. &&
                RecentThroughput<SaturatedThroughput*(1.-Config.ReprobeDropRatio))
            {
                TargetAGV=FMath::Min(LastAcceptedAGV,SaturatedAGV);
                BeginSettling(true);
                return;
            }
        }
        if(SaturatedElapsed>=Config.ReprobeAfterSec)
        {
            if(TargetAGV>=Config.MaxAGV)
            {
                TargetAGV=FMath::Max(1,LastAcceptedAGV);
                BeginSettling(true);
            }
            else
            {
                Stage=EAGVThroughputStage::Probing;
                StageElapsed=0.;
                SaturatedElapsed=0.;
            }
        }
        break;
    }
}

bool FAGVThroughputController::IsFleetMember(int32 Vehicle) const
{
    return ActiveFleet.IsValidIndex(Vehicle) && ActiveFleet[Vehicle];
}

int32 FAGVThroughputController::ActiveFleetCount() const
{
    int32 Count=0;
    for(const bool Active:ActiveFleet) Count+=Active;
    return Count;
}

const TCHAR* FAGVThroughputController::StageName() const
{
    switch(Stage)
    {
    case EAGVThroughputStage::Probing: return TEXT("Probing");
    case EAGVThroughputStage::Settling: return TEXT("Settling");
    case EAGVThroughputStage::Measuring: return TEXT("Measuring");
    case EAGVThroughputStage::Saturated: return TEXT("Saturated");
    default: return TEXT("Unknown");
    }
}
