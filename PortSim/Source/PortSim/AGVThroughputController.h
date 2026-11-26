#pragma once

#include "CoreMinimal.h"

enum class EAGVThroughputStage : uint8
{
    Probing,
    Settling,
    Measuring,
    Saturated
};

struct FAGVThroughputConfig
{
    bool bEnabled=true;
    int32 StepSize=4;
    double SettleSec=300.;
    double WindowSec=900.;
    double MinGainRatio=.03;
    int32 MaxAGV=60;
    double BatteryReturnPercent=20.;
    double ReprobeAfterSec=3600.;
    double ReprobeDropRatio=.2;

    bool Load(const FString& Path,FString& Error);
};

class FAGVThroughputController
{
public:
    FAGVThroughputConfig Config;
    EAGVThroughputStage Stage=EAGVThroughputStage::Probing;
    int32 TargetAGV=0;
    int32 LastAcceptedAGV=0;
    int32 SaturatedAGV=0;
    double ReferenceThroughput=0.;
    double RecentThroughput=0.;
    double SaturatedThroughput=0.;
    double StageElapsed=0.;
    double WindowElapsed=0.;
    double SaturatedElapsed=0.;
    int32 WindowCompleted=0;
    bool bHasReference=false;
    TArray<int32> LastSTSCompletedJobs;
    TArray<bool> ActiveFleet;

    void Initialize(const FAGVThroughputConfig& InConfig,int32 FleetSize,const TArray<int32>& STSCompletedJobs);
    void Advance(double Dt,const TArray<int32>& STSCompletedJobs,bool bInvalidWindow);
    bool IsFleetMember(int32 Vehicle) const;
    int32 ActiveFleetCount() const;
    const TCHAR* StageName() const;

private:
    void BeginProbe();
    void BeginSettling(bool bResetReference);
    void FinishWindow();
    int32 CompletedDelta(const TArray<int32>& STSCompletedJobs);
};
