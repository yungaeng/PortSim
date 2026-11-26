#if WITH_DEV_AUTOMATION_TESTS
#include "AGVThroughputController.h"
#include "Misc/AutomationTest.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAgvThroughputControllerTest,"PortSim.Logistics.AGVThroughputController",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FAgvThroughputControllerTest::RunTest(const FString& Parameters)
{
    FAGVThroughputConfig Config;
    Config.StepSize=4;
    Config.SettleSec=0.;
    Config.WindowSec=3600.;
    Config.MinGainRatio=.03;
    Config.MaxAGV=60;
    Config.ReprobeAfterSec=100000.;
    Config.ReprobeDropRatio=.2;

    TArray<int32> Counts={0};
    FAGVThroughputController Controller;
    Controller.Initialize(Config,60,Counts);
    auto Measure=[&](FAGVThroughputController& C,TArray<int32>& Completed,
        const TFunction<double(const FAGVThroughputController&)>& Rate)
    {
        int32 Guard=0;
        while(C.Stage!=EAGVThroughputStage::Measuring && Guard++<4)
            C.Advance(1.,Completed,false);
        if(C.Stage!=EAGVThroughputStage::Measuring) return false;
        Completed[0]+=FMath::RoundToInt(Rate(C)*C.Config.WindowSec/3600.);
        C.Advance(C.Config.WindowSec,Completed,false);
        return true;
    };

    int32 Guard=0;
    while(Controller.Stage!=EAGVThroughputStage::Saturated && Guard++<40)
    {
        if(!Measure(Controller,Counts,[](const auto& C){return FMath::Min(C.TargetAGV*100.,4000.);}))
        {
            AddError(TEXT("Controller did not reach a measurement window"));
            return false;
        }
    }
    TestEqual(TEXT("Throughput plateaus within one probe step of 40 AGVs"),Controller.TargetAGV,44);
    TestTrue(TEXT("Plateau stops further admission"),FString(Controller.StageName())==TEXT("Saturated"));
    TestTrue(TEXT("Reference throughput is retained"),Controller.ReferenceThroughput>=3900.);

    FAGVThroughputController Noisy;
    Noisy.Initialize(Config,60,Counts);
    Guard=0;
    while(Noisy.Stage!=EAGVThroughputStage::Saturated && Guard++<40)
    {
        if(!Measure(Noisy,Counts,[](const auto& C)
            {
                const double Base=FMath::Min(C.TargetAGV*100.,4000.);
                const double Noise=(C.TargetAGV/4)%2==0?.01:-.01;
                return Base*(1.+Noise);
            }))
        {
            AddError(TEXT("Noisy controller did not reach a measurement window"));
            return false;
        }
        TestTrue(TEXT("Small throughput noise does not saturate below the plateau"),
            Noisy.Stage!=EAGVThroughputStage::Saturated || Noisy.TargetAGV>=40);
    }
    TestEqual(TEXT("Noise still locates the plateau"),Noisy.TargetAGV,44);

    FAGVThroughputConfig FaultConfig=Config;
    FaultConfig.SettleSec=5.;
    FaultConfig.WindowSec=10.;
    FAGVThroughputController Faulted;
    TArray<int32> FaultCounts={0};
    Faulted.Initialize(FaultConfig,60,FaultCounts);
    Faulted.Advance(1.,FaultCounts,false);
    Faulted.Advance(2.,FaultCounts,false);
    Faulted.Advance(5.,FaultCounts,false);
    TestTrue(TEXT("Controller enters measuring after settling"),FString(Faulted.StageName())==TEXT("Measuring"));
    FaultCounts[0]+=2;
    Faulted.Advance(3.,FaultCounts,false);
    Faulted.Advance(1.,FaultCounts,true);
    TestTrue(TEXT("Fault discards the partial measurement window"),FString(Faulted.StageName())==TEXT("Settling"));
    TestEqual(TEXT("Fault resets the settling timer"),Faulted.StageElapsed,0.);
    TestEqual(TEXT("Fault drops completed counts from the window"),Faulted.WindowCompleted,0);
    TestEqual(TEXT("Fault clears elapsed window time"),Faulted.WindowElapsed,0.);

    FAGVThroughputController Original;
    TArray<int32> ReplayCounts={0};
    Original.Initialize(FaultConfig,60,ReplayCounts);
    Original.Advance(1.,ReplayCounts,false);
    Original.Advance(5.,ReplayCounts,false);
    ReplayCounts[0]+=2;
    Original.Advance(3.,ReplayCounts,false);
    TestTrue(TEXT("Checkpoint captures a partial measurement window"),
        Original.Stage==EAGVThroughputStage::Measuring && Original.WindowElapsed==3. &&
        Original.WindowCompleted==2);
    const FAGVThroughputController Saved=Original;
    const TArray<int32> SavedCounts=ReplayCounts;
    ReplayCounts[0]+=4;
    Original.Advance(7.,ReplayCounts,false);
    const double ExpectedRecent=Original.RecentThroughput;
    Original=Saved;
    ReplayCounts=SavedCounts;
    ReplayCounts[0]+=4;
    Original.Advance(7.,ReplayCounts,false);
    TestEqual(TEXT("Restored window continues with identical throughput"),Original.RecentThroughput,ExpectedRecent);
    TestTrue(TEXT("Restored controller reaches the same next state"),Original.Stage==EAGVThroughputStage::Probing);
    return true;
}
#endif
