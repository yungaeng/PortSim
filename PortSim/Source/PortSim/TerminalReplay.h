#pragma once
#include "CoreMinimal.h"

class AQuayCrane;
class AActor;
class UWorld;
// Forward-only fixed-step execution. Explicit checkpoint capture/restore lives
// in FTerminalCheckpoint and is never part of accelerated playback.
class FTerminalReplay
{
public:
    ~FTerminalReplay();
    bool Tick(AQuayCrane& Owner,double WallSeconds);
    void Invalidate();
    const TCHAR* Mode() const { return State; }
    double BufferedSeconds() const;
    double DisplayedSeconds() const { return DisplayedSteps/60.; }
    uint64 Generation() const { return Revision; }
    float AchievedRate() const { return ActualRate; }
    static bool IsPredicting();
    static bool IsAdvancingPhysics();
    static void Log(FName Category,ELogVerbosity::Type Verbosity,FString Message);
private:
    friend class FTerminalReplayCheckpointTest;
    bool AdvanceLivePhysics(AQuayCrane& Owner);
    const TCHAR* State=TEXT("live");
    uint64 Revision=0,DisplayedSteps=0;
    double Credit=0;
    double SampleWall=0,SampleSimulation=0;
    float ActualRate=0;
    double DiagnosticWall=0, DiagnosticModel=0, DiagnosticPhysics=0, DiagnosticFrame=0, DiagnosticDropped=0;
    uint64 DiagnosticSteps=0, DiagnosticBatches=0, DiagnosticBudgetHits=0, DiagnosticStepHits=0;
    double DiagnosticSolver=0, DiagnosticContactSync=0, DiagnosticTransition=0;
    uint64 DiagnosticPhysicsCalls=0;
    int32 DiagnosticFreeCargoPeak=0;
    void ResetDiagnostics(double Wall=0);
    TWeakObjectPtr<UWorld> PhysicsWorld;
    TArray<TWeakObjectPtr<AActor>> PhysicsActors;
    FDelegateHandle PhysicsSpawnHandle,PhysicsDestroyHandle;
    bool bPhysicsActorsDirty=true;
    void ReleasePhysicsRegistry();
};

#define PORTSIM_SIM_LOG(Category,Verbosity,Format,...) \
    do { if(!Category.IsSuppressed(ELogVerbosity::Verbosity)) \
        FTerminalReplay::Log(Category.GetCategoryName(),ELogVerbosity::Verbosity,FString::Printf(Format,##__VA_ARGS__)); } while(0)
