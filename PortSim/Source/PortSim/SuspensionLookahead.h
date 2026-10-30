#pragma once
#include "STSDynamics.h"
#include "Async/Future.h"

// Isolated prototype: this owns a reduced-order suspension, not a live world.
// Inputs are held constant over the prediction window. SubmitInput invalidates
// all future samples; no actor, sensor, Chaos scene or telemetry is advanced.
struct FSuspensionLookaheadInput
{
    FSTSDynamicsParameters Dynamics;
    double LengthM=20, LengthRateMps=0, MassKg=17000, PowerW=1.e6;
    FVector Acceleration=FVector::ZeroVector, Velocity=FVector::ZeroVector, CoG=FVector::ZeroVector;
};

enum class ESuspensionLookaheadMode : uint8 { Live, Preparing, Replay, Fault };

class FSuspensionLookahead
{
public:
    static constexpr double StepSeconds=1./60.;
    void Reset(const FSTSSuspension& State,const FSuspensionLookaheadInput& Input,
        double WindowSeconds=.5,double QuietSeconds=2.);
    // Call for every accepted external control event, including repeated events.
    // The displayed state is retained; an unconsumed future is never committed.
    void SubmitInput(const FSuspensionLookaheadInput& Input);
    void SetRequestedRate(double Rate);
    void Tick(double WallSeconds,int32 PredictionBudgetSteps=8);
    // Optional worker owns numeric copies only; stale generations are discarded.
    void SetBackgroundPrediction(bool Enabled) { bBackgroundPrediction=Enabled; }
    const FSTSSuspension& DisplayedState() const { return Current; }
    ESuspensionLookaheadMode Mode() const { return CurrentMode; }
    double DisplayedSeconds() const { return DisplayedSteps*StepSeconds; }
    double CalculatedSeconds() const { return DisplayedSeconds()+BufferedSeconds(); }
    double BufferedSeconds() const { return Future.Num()*StepSeconds; }
    double RequestedRate() const { return Rate; }
    uint64 InputRevision() const { return Revision; }
private:
    void Integrate(FSTSSuspension& State) const;
    static void IntegrateState(FSTSSuspension& State,const FSuspensionLookaheadInput& Input);
    void Prepare(int32 Budget);
    struct FPrediction
    {
        uint64 Revision=0, FirstStep=0;
        TArray<FSTSSuspension> States;
    };
    TFuture<FPrediction> Prediction;
    bool bBackgroundPrediction=false;
    FSTSSuspension Current;
    FSuspensionLookaheadInput Held;
    TArray<FSTSSuspension> Future;
    ESuspensionLookaheadMode CurrentMode=ESuspensionLookaheadMode::Live;
    int32 WindowSteps=30;
    uint64 DisplayedSteps=0, Revision=0;
    double Rate=1, QuietDelay=2, QuietElapsed=0, PendingSeconds=0;
};
