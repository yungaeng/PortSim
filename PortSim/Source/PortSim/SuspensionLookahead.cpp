#include "SuspensionLookahead.h"

void FSuspensionLookahead::Reset(const FSTSSuspension& State,const FSuspensionLookaheadInput& Input,
    double WindowSeconds,double QuietSeconds)
{
    Current=State; Held=Input; Future.Reset(); DisplayedSteps=0; ++Revision;
    WindowSteps=FMath::Clamp(FMath::RoundToInt((FMath::IsFinite(WindowSeconds)?
        FMath::Clamp(WindowSeconds,StepSeconds,2.):.5)/StepSeconds),1,120);
    QuietDelay=FMath::IsFinite(QuietSeconds)?FMath::Clamp(QuietSeconds,0.,60.):2.;
    QuietElapsed=PendingSeconds=0;
    CurrentMode=Current.Fault.IsEmpty()?ESuspensionLookaheadMode::Preparing:ESuspensionLookaheadMode::Fault;
}

void FSuspensionLookahead::SubmitInput(const FSuspensionLookaheadInput& Input)
{
    Held=Input; ++Revision; Future.Reset();
    // Discard fractional replay credit too: an input starts a new 1x epoch.
    QuietElapsed=PendingSeconds=0;
    CurrentMode=Current.Fault.IsEmpty()?ESuspensionLookaheadMode::Live:ESuspensionLookaheadMode::Fault;
}

void FSuspensionLookahead::SetRequestedRate(double NewRate)
{
    Rate=FMath::IsFinite(NewRate)?FMath::Clamp(NewRate,1.,16.):1.;
    PendingSeconds=0;
}

void FSuspensionLookahead::Integrate(FSTSSuspension& State) const
{
    // Require a usable numeric profile before entering the existing solver.
    const auto& C=Held.Dynamics;
    if(!FMath::IsFinite(Held.LengthM) || Held.LengthM<.1 ||
        !FMath::IsFinite(Held.MassKg) || Held.MassKg<=0 ||
        !FMath::IsFinite(Held.LengthRateMps) || !FMath::IsFinite(Held.PowerW) || Held.PowerW<=0 ||
        Held.Acceleration.ContainsNaN() || Held.Velocity.ContainsNaN() || Held.CoG.ContainsNaN() ||
        C.Wind.ContainsNaN() || C.DrumRadius<=0 || C.GearRatio<=0 || C.Parts<=0 ||
        C.Efficiency<=0 || C.MotorCount<=0 || C.RopeEA<=0)
    {
        State.Fault=TEXT("Lookahead requires a valid suspension profile and finite inputs");
        return;
    }
    State.Step(C,StepSeconds,Held.LengthM,Held.LengthRateMps,Held.Acceleration,
        Held.Velocity,Held.MassKg,Held.CoG,Held.PowerW);
}

void FSuspensionLookahead::Prepare(int32 Budget)
{
    FSTSSuspension Predicted=Future.IsEmpty()?Current:Future.Last();
    for(int32 I=0;I<FMath::Clamp(Budget,0,120) && Future.Num()<WindowSteps;++I)
    {
        // Keep a predicted fault at its own future time; do not fault the
        // displayed state early, or produce samples past that fault.
        if(!Predicted.Fault.IsEmpty()) break;
        Integrate(Predicted);
        Future.Add(Predicted);
    }
}

void FSuspensionLookahead::Tick(double WallSeconds,int32 PredictionBudgetSteps)
{
    if(!FMath::IsFinite(WallSeconds) || WallSeconds<0 || CurrentMode==ESuspensionLookaheadMode::Fault) return;
    // Bound work after a debugger stall. This is a wall-time playback policy,
    // never an enlarged physical integration interval.
    const double Elapsed=FMath::Min(WallSeconds,.25);
    if(CurrentMode==ESuspensionLookaheadMode::Live)
    {
        PendingSeconds+=Elapsed;
        while(PendingSeconds+1.e-12>=StepSeconds)
        {
            Integrate(Current); ++DisplayedSteps; PendingSeconds-=StepSeconds;
            if(!Current.Fault.IsEmpty()) { CurrentMode=ESuspensionLookaheadMode::Fault; return; }
        }
        QuietElapsed+=Elapsed;
        if(QuietElapsed>=QuietDelay)
        {
            CurrentMode=ESuspensionLookaheadMode::Preparing; PendingSeconds=0;
        }
        return;
    }
    Prepare(PredictionBudgetSteps);
    if(CurrentMode==ESuspensionLookaheadMode::Preparing)
    {
        if(Future.Num()==WindowSteps || (!Future.IsEmpty() && !Future.Last().Fault.IsEmpty()))
            CurrentMode=ESuspensionLookaheadMode::Replay;
        // Preparation time is a visible wait, not accumulated replay debt.
        return;
    }
    PendingSeconds+=Elapsed*Rate;
    while(PendingSeconds+1.e-12>=StepSeconds)
    {
        if(Future.IsEmpty())
        {
            CurrentMode=ESuspensionLookaheadMode::Preparing; PendingSeconds=0; return;
        }
        Current=MoveTemp(Future[0]); Future.RemoveAt(0);
        ++DisplayedSteps; PendingSeconds-=StepSeconds;
        if(!Current.Fault.IsEmpty())
        { Future.Reset(); CurrentMode=ESuspensionLookaheadMode::Fault; PendingSeconds=0; return; }
    }
}
