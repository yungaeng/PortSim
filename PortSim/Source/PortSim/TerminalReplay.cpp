#include "TerminalReplay.h"
#include "TerminalCheckpoint.h"
#include "QuayCrane.h"
#include "PortSiteLogistics.h"
#include "PortSimTimeStep.h"
#include "HAL/PlatformTime.h"
#include "HAL/IConsoleManager.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "Misc/ScopeExit.h"
#include "SimulationPerformance.h"
#include "Misc/OutputDeviceRedirector.h"

namespace
{
    bool Predicting=false; // Game thread only. UObject work never crosses threads.
    TArray<FTerminalReplayLog> PendingLogs;
    TAutoConsoleVariable<int32> Enabled(TEXT("PortSim.BufferedPlayback"),1,
        TEXT("Checkpoint replay for the unified terminal. Chaos bodies use the existing fixed-step live clock."));
    TAutoConsoleVariable<int32> Window(TEXT("PortSim.ReplayWindowSteps"),16,TEXT("Fixed 1/60-second checkpoint window, 1..60."));
    TAutoConsoleVariable<int32> Budget(TEXT("PortSim.ReplayBudgetSteps"),16,TEXT("Maximum prediction steps per presentation, 1..60."));
}

bool FTerminalReplay::IsPredicting() { return Predicting; }
void FTerminalReplay::Log(FName Category,ELogVerbosity::Type Verbosity,FString Message)
{
    if(Predicting) PendingLogs.Add({Category,Verbosity,MoveTemp(Message)});
    else if(GLog) GLog->Serialize(*Message,Verbosity,Category);
}
double FTerminalReplay::BufferedSeconds() const
{ int32 Steps=0; for(int32 N:FutureSteps) Steps+=N; return Steps/60.; }
void FTerminalReplay::Invalidate()
{
    Future.Reset(); FutureSteps.Reset(); FutureLogs.Reset(); Current.Reset(); Credit=0; bBoundary=false; ++Revision; State=TEXT("live_input_hold");
}

// Same model ordering as the live unified-terminal branch, with fixed dt and
// no camera/input, world timers, Chaos integration, or file publication.
void AQuayCrane::AdvanceTerminalReplayStep(float Dt)
{
    const bool Frozen=bAutoPaused || bEmergencyStop || !bAutoRunning || !SiteLogistics->Fault.IsEmpty();
    if(!Frozen) Environment->AdvanceEnvironment(Dt);
    if(bAutoRunning) WeatherHold.Update(*Environment,Frozen?0.f:Dt);
    else WeatherHold=FPortWeatherHold();
    if(!Frozen && !WeatherHold.Active) ApplySeaMotion();
    SiteLogistics->SetWindVelocity(Environment->GetEffectiveWindVelocity());
    SiteLogistics->SetFogVisibilityMeters(Environment->IsFogEnabled()?Environment->GetFogVisibilityMeters():MAX_flt);
    SiteLogistics->SetRainState(Environment->GetRainState());
    SiteLogistics->SetSurfaceConditions(Environment->GetSurfaceConditions(),Environment->GetFrictionSettings(),Environment->GetContactMaterial());
    STSSimulationTime+=Dt;
    SampleSTSSensors();
    SiteLogistics->BeginTrafficFrame();
    if(bAutoRunning && !IsOperationPaused() && !bEmergencyStop && SiteLogistics->Fault.IsEmpty()) AutoElapsed+=Dt;
    SiteLogistics->Advance(Dt,IsOperationPaused() || bEmergencyStop || !bAutoRunning);
    AutoCompleted=Deliveries=SiteLogistics->Delivered;
    if(bAutoRunning && SiteLogistics->ShipRemaining()==0 && SiteLogistics->IsIdle())
    {
        bAutoRunning=false;
        Status=FString::Printf(TEXT("All configured ships unloaded: %d containers, %.1f simulation seconds"),Deliveries,AutoElapsed);
    }
}

bool FTerminalReplay::Tick(AQuayCrane& Owner,double WallSeconds)
{
    auto* Clock=Owner.PlaybackClock.Get();
    auto* Site=Owner.SiteLogistics.Get();
    if(!Clock || !Site) return false;
    const double Now=FPlatformTime::Seconds();
    if(SampleWall==0 || Site->SimulationTime<SampleSimulation)
    { SampleWall=Now; SampleSimulation=Site->SimulationTime; ActualRate=0; }
    else if(Now-SampleWall>=.5)
    {
        ActualRate=float((Site->SimulationTime-SampleSimulation)/(Now-SampleWall));
        SampleWall=Now; SampleSimulation=Site->SimulationTime;
    }
    Clock->SetBufferedPlayback(false);
    Clock->SetLiveFallback(false);
    Site->ProcessDashboardCommands(); // Accepted commands invalidate before future use.
    // Test policies must be installed before calculating the first future.
    const bool Full=FParse::Param(FCommandLine::Get(),TEXT("PortSimFullUnloadTest"));
    if(Full || FParse::Param(FCommandLine::Get(),TEXT("PortSimSiteTest")))
    {
        int32 Limit=Site->Vehicles.Num()*2;
        FParse::Value(FCommandLine::Get(),TEXT("PortSimSiteShipmentLimit="),Limit);
        Site->DispatchLimit=(Full || FParse::Param(FCommandLine::Get(),TEXT("PortSimContinuousTrafficTest")))?
            Site->InitialShipCount():FMath::Clamp(Limit,9,Site->InitialShipCount());
    }
    if(FParse::Param(FCommandLine::Get(),TEXT("PortSimFirstWaveTest")))
    {
        int32 Limit=9; FParse::Value(FCommandLine::Get(),TEXT("PortSimWaveShipmentCount="),Limit);
        Site->DispatchLimit=FMath::Clamp(Limit,1,Site->InitialShipCount());
    }
    else if(FParse::Param(FCommandLine::Get(),TEXT("PortSimCarLikeTest"))) Site->DispatchLimit=1;
    if(!Enabled.GetValueOnGameThread() || FParse::Param(FCommandLine::Get(),TEXT("PortSimNoReplay")) ||
        !Owner.bAutoRunning || Owner.bAutoPaused || Owner.bEmergencyStop || !Site->Fault.IsEmpty() ||
        Clock->GetEffectiveRate()<=1.f)
    {
        Future.Reset(); FutureSteps.Reset(); FutureLogs.Reset(); Current.Reset(); Credit=0; bBoundary=false;
        State=Clock->GetEffectiveRate()<Owner.SimulationSpeed?TEXT("live_input_hold"):TEXT("live");
        return false;
    }
    if(!FTerminalCheckpoint::CanPredict(Owner) || (bBoundary && Future.IsEmpty()))
    {
        Future.Reset(); FutureSteps.Reset(); FutureLogs.Reset(); Current.Reset(); Credit=0; bBoundary=false; State=TEXT("live_chaos");
        return false;
    }
    Clock->SetBufferedPlayback(true);
    if(!Current) Current=FTerminalCheckpoint::Capture(Owner);
    auto Displayed=Current;
    if(!Displayed) { Invalidate(); Clock->SetLiveFallback(true); return false; }
    const int32 StepsPerPicture=FMath::Clamp(FMath::RoundToInt(Owner.SimulationSpeed),1,16);
    const int32 Target=FMath::Max(StepsPerPicture*2,FMath::Clamp(Window.GetValueOnGameThread(),1,60));
    int32 Available=FMath::RoundToInt(BufferedSeconds()*60.);
    bool Restored=true;
    if(!bBoundary && Available<Target)
    {
        Predicting=true;
        ON_SCOPE_EXIT { PendingLogs.Reset(); Predicting=false; Restored=Displayed->Restore(); };
        if(!Future.IsEmpty() && !Future.Last()->Restore())
        { Future.Reset(); FutureSteps.Reset(); FutureLogs.Reset(); bBoundary=true; }
        else
        {
            const int32 Count=FMath::Min(Target-Available,FMath::Max(StepsPerPicture,FMath::Clamp(Budget.GetValueOnGameThread(),1,60)));
            for(int32 I=0;I<Count;)
            {
                PendingLogs.Reset();
                const int32 Batch=FMath::Min(StepsPerPicture,Count-I);
                for(int32 J=0;J<Batch;++J)
                {
                    Owner.AdvanceTerminalReplayStep(1.f/60.f); ++I;
                    if(!FTerminalCheckpoint::CanPredict(Owner)) { bBoundary=true; break; }
                }
                if(bBoundary) break;
                auto Next=FTerminalCheckpoint::Capture(Owner);
                if(!Next) { bBoundary=true; break; }
                Future.Add(MoveTemp(Next)); FutureSteps.Add(Batch); Available+=Batch;
                FutureLogs.Add(MoveTemp(PendingLogs));
                if(!Owner.bAutoRunning || !Site->Fault.IsEmpty()) { bBoundary=true; break; }
            }
        }
    }
    if(!Restored)
    {
        Invalidate(); Site->Stop(TEXT("Checkpoint object lifecycle changed during prediction"));
        return true;
    }
    if(Future.IsEmpty())
    {
        // A release happened on the first predicted step. The displayed
        // checkpoint has already been restored; execute that step live once.
        bBoundary=false; State=TEXT("live_chaos"); Clock->SetBufferedPlayback(false);
        Current.Reset();
        return false;
    }
    if(FCString::Strcmp(State,TEXT("replay"))!=0 && Available<Target && !bBoundary)
    {
        State=TEXT("preparing"); Credit=0;
    }
    else
    {
        State=TEXT("replay");
        Credit+=FMath::Clamp(WallSeconds,0.,.05)*Owner.SimulationSpeed;
        const int32 Wanted=FMath::FloorToInt((Credit+1.e-9)*60.);
        int32 Take=0,Consumed=0;
        while(Take<FutureSteps.Num() && Consumed+FutureSteps[Take]<=Wanted)
        { Consumed+=FutureSteps[Take]; ++Take; }
        if(Take>0)
        {
            const FString PreviousResults=Site->ResultsCsv;
            if(!Future[Take-1]->Restore()) { Invalidate(); Site->Stop(TEXT("Replay checkpoint expired")); return true; }
            Current=Future[Take-1];
            for(int32 I=0;I<Take;++I) for(auto& Entry:FutureLogs[I])
                Log(Entry.Category,Entry.Verbosity,MoveTemp(Entry.Message));
            FutureLogs.RemoveAt(0,Take);
            Future.RemoveAt(0,Take); FutureSteps.RemoveAt(0,Take); DisplayedSteps+=Consumed; Credit-=Consumed/60.;
            if(Site->ResultsCsv!=PreviousResults && !Site->SaveReports()) Site->Stop(TEXT("Committed replay report failed"));
        }
        if(Wanted>Consumed && Future.IsEmpty()) { Credit=0; State=TEXT("preparing"); }
    }
    // These outputs are now derived exclusively from the displayed checkpoint.
    Site->ExportDashboard(Owner.IsOperationPaused());
    PortSimPerformance::Report(Site->SimulationTime,Site->Delivered,Owner.GetWorld());
    return true;
}
