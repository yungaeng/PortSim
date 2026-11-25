#include "TerminalReplay.h"
#include "PortContainerActor.h"
#include "PortSupportVehicle.h"
#include "Components/StaticMeshComponent.h"
#include "Components/TextRenderComponent.h"
#include "Components/WidgetComponent.h"
#include "ProceduralMeshComponent.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "PhysicsEngine/PhysicsSettings.h"
#include "PhysicsEngine/BodyInstance.h"
#include "PBDRigidsSolver.h"
#include "Physics/Experimental/PhysScene_Chaos.h"
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
    TAutoConsoleVariable<int32> BatchPhysics(TEXT("PortSim.BatchLivePhysics"),1,
        TEXT("Batch forward-only terminal cargo physics at fixed 1/60-second steps."));
    TAutoConsoleVariable<int32> LivePhysicsSteps(TEXT("PortSim.LivePhysicsMaxSteps"),64,
        TEXT("Hard limit on forward steps per engine frame (1..120); independent of requested playback rate."));
    TAutoConsoleVariable<float> LivePhysicsBudget(TEXT("PortSim.LivePhysicsBudgetMs"),50.f,
        TEXT("Wall-time budget for forward computation per engine frame (1..100 ms); one step always completes."));
    TAutoConsoleVariable<int32> InlinePhysics(TEXT("PortSim.InlineBatchPhysics"),1,
        TEXT("Execute synchronous terminal solver dispatch inline during a batch; restore the world threading mode afterwards."));
    bool AdvancingPhysics=false;
    TAutoConsoleVariable<int32> Enabled(TEXT("PortSim.BufferedPlayback"),1,
        TEXT("Forward-only fixed-step terminal playback."));
}

bool FTerminalReplay::IsPredicting() { return false; }
FTerminalReplay::~FTerminalReplay() { ReleasePhysicsRegistry(); }
void FTerminalReplay::ReleasePhysicsRegistry()
{
    if(auto* World=PhysicsWorld.Get())
    {
        World->RemoveOnActorSpawnedHandler(PhysicsSpawnHandle);
        World->RemoveOnActorDestroyedHandler(PhysicsDestroyHandle);
    }
    PhysicsActors.Reset(); PhysicsWorld.Reset(); bPhysicsActorsDirty=true;
}
bool FTerminalReplay::IsAdvancingPhysics() { return AdvancingPhysics; }
void FTerminalReplay::Log(FName Category,ELogVerbosity::Type Verbosity,FString Message)
{
    if(GLog) GLog->Serialize(*Message,Verbosity,Category);
}
double FTerminalReplay::BufferedSeconds() const
{ return 0.; } // Retained for dashboard protocol compatibility.
void FTerminalReplay::ResetDiagnostics(double Wall)
{
    DiagnosticWall=Wall;
    DiagnosticModel=DiagnosticPhysics=DiagnosticFrame=DiagnosticDropped=0;
    DiagnosticSteps=DiagnosticBatches=DiagnosticBudgetHits=DiagnosticStepHits=0;
    DiagnosticSolver=DiagnosticContactSync=DiagnosticTransition=0;
    DiagnosticPhysicsCalls=0; DiagnosticFreeCargoPeak=0;
}
void FTerminalReplay::Invalidate()
{
    bPhysicsActorsDirty=true;
    Credit=0; ++Revision; State=TEXT("live_input_hold");
    ResetDiagnostics();
}

// Same model ordering as the live unified-terminal branch, with fixed dt and
// no camera/input or world timers. The caller supplies physics and publication.
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

bool FTerminalReplay::AdvanceLivePhysics(AQuayCrane& Owner)
{
    const double FrameStarted=FPlatformTime::Seconds();
    if(!BatchPhysics.GetValueOnGameThread() && !FParse::Param(FCommandLine::Get(),TEXT("PortSimBatchLivePhysics"))) return false;
    auto* World=Owner.GetWorld();
    auto* Scene=World?World->GetPhysicsScene():nullptr;
    const auto* Settings=UPhysicsSettings::Get();
    auto* Solver=Scene?Scene->GetSolver():nullptr;
    if(!Solver || World->GetNetMode()!=NM_Standalone || Settings->bTickPhysicsAsync ||
        Settings->PhysicsPrediction.bEnablePhysicsPrediction || !Scene->IsCompletionEventComplete()) return false;
    // Terminal cargo and explicitly managed R/S bodies may be dynamic. External actors require their
    // own engine ticks and callbacks and retain the standard world path.
    PortSimPerformance::FScope EligibilityTiming(TEXT("forward_frame"));
    if(PhysicsWorld.Get()!=World)
    {
        ReleasePhysicsRegistry(); PhysicsWorld=World;
        PhysicsSpawnHandle=World->AddOnActorSpawnedHandler(FOnActorSpawned::FDelegate::CreateLambda(
            [this](AActor*) { bPhysicsActorsDirty=true; }));
        PhysicsDestroyHandle=World->AddOnActorDestroyedHandler(FOnActorDestroyed::FDelegate::CreateLambda(
            [this](AActor*) { bPhysicsActorsDirty=true; }));
    }
    if(bPhysicsActorsDirty)
    {
        PhysicsActors.Reset();
        for(TActorIterator<AActor> It(World);It;++It)
        {
            auto* Box=Cast<APortContainerActor>(*It);
            if(Box && Box->GetOwner()==Owner.SiteLogistics && Box->GetClass()==APortContainerActor::StaticClass() &&
                Box->GetComponents().Num()==2 && IsValid(Box->Body) && IsValid(Box->Visual) &&
                Box->Visual->GetCollisionEnabled()==ECollisionEnabled::NoCollision && !Box->Visual->IsAnySimulatingPhysics()) continue;
            PhysicsActors.Add(*It);
        }
        // Prune weak pose keys when inventory changes, never every display frame.
        auto& Poses=Owner.SiteLogistics->KinematicPhysicsPoses;
        for(auto It=Poses.CreateIterator();It;++It) if(!It.Key().IsValid()) It.RemoveCurrent();
        bPhysicsActorsDirty=false;
    }
    TArray<UPrimitiveComponent*> RecoveryBodies;
    TArray<UPrimitiveComponent*> KinematicParts;
    bool HasOtherPhysics=false;
    for(const auto& Actor:PhysicsActors)
    {
        AActor* const CurrentActor=Actor.Get(); if(!IsValid(CurrentActor)) continue;
        // Owned cargo is maintained by physics/attachment lifecycle callbacks.
        // Only externally supplied cargo needs reconciliation at frame entry.
        auto* Cargo=Cast<APortContainerActor>(CurrentActor);
        const bool OwnedCargo=Cargo && Cargo->GetOwner()==Owner.SiteLogistics;
        const bool TerminalCargo=OwnedCargo || (Cargo && Owner.SiteLogistics->ShipContainers.Contains(Cargo));
        if(TerminalCargo && !OwnedCargo) Owner.SiteLogistics->RefreshCargoActivity(Cargo);
        // The normal container has only its tracked body and non-colliding
        // artwork. Lifecycle/transform callbacks maintain its activity and
        // dirty pose, so do not enumerate components or commit idle bodies.
        // Extended actors still take the full compatibility check below.
        if(OwnedCargo && Cargo->GetComponents().Num()==2 && IsValid(Cargo->Body) && IsValid(Cargo->Visual) &&
            Cargo->Visual->GetCollisionEnabled()==ECollisionEnabled::NoCollision && !Cargo->Visual->IsAnySimulatingPhysics())
            continue;
        const auto* RS=Cast<APortSupportVehicle>(CurrentActor);
        const bool ManagedRS=RS && Owner.SupportFleet.ContainsByPredicate([&](const auto& A){return A.Get()==RS;});
        TInlineComponentArray<UPrimitiveComponent*> Parts(CurrentActor);
        for(auto* Part:Parts)
        {
            // Include dormant R/S parts: a rollover can enable them mid-batch.
            if(ManagedRS) RecoveryBodies.Add(Part);
            if(Part->IsAnySimulatingPhysics() && !(ManagedRS && RS->OwnsRecoveryPhysicsBody(Part)) &&
                (!TerminalCargo || Part!=Cargo->GetBody())) return false;
            if(Part->IsPhysicsStateCreated() && Part->K2_IsPhysicsCollisionEnabled() && !Part->IsStaticPhysics())
            {
                KinematicParts.Add(Part);
                if(!Part->IsA<UStaticMeshComponent>() && !Part->IsA<UProceduralMeshComponent>() &&
                    !Part->IsA<UTextRenderComponent>() && !Part->IsA<UWidgetComponent>() &&
                    Part->GetAllPhysicsObjects().ContainsByPredicate([](const auto* Object) { return Object!=nullptr; }))
                    HasOtherPhysics=true;
            }
        }
    }
    PortSimPerformance::FScope Timing(TEXT("live_physics_batch"));
    // StartFrame followed immediately by WaitPhysScenes has no useful overlap
    // with the game thread. Avoid scheduling and waiting for a worker task for
    // every tiny solver substep; retain Chaos contacts, iterations and callbacks.
    const auto PreviousThreading=Solver->GetThreadingMode();
    const bool UseInlinePhysics=InlinePhysics.GetValueOnGameThread()!=0 && !HasOtherPhysics;
    if(UseInlinePhysics) Solver->SetThreadingMode_External(Chaos::EThreadingModeTemp::SingleThread);
    ON_SCOPE_EXIT { if(UseInlinePhysics) Solver->SetThreadingMode_External(PreviousThreading); };
    // A legacy rounded 0.008333 causes ceil((1/60)/maxDt)==3, not 2.
    // Normalize only that known old value; preserve custom substep settings.
    const float PhysicsSubstep=FMath::IsNearlyEqual(Settings->MaxSubstepDeltaTime,.008333f,1.e-9f)?
        1.f/120.f:Settings->MaxSubstepDeltaTime;
    const FVector Gravity(0,0,World->GetGravityZ());
    // The enclosing world's physics tick must not integrate the same time again.
    ON_SCOPE_EXIT { Scene->SetUpForFrame(&Gravity,0.f,0.f,0.f,Settings->MaxSubstepDeltaTime,Settings->MaxSubsteps,false); };
    Owner.PlaybackClock->SetBufferedPlayback(true);
    State=TEXT("live_physics_batch");
    const int32 MaxSteps=FMath::Clamp(LivePhysicsSteps.GetValueOnGameThread(),1,120);
    const float RequestedBudget=LivePhysicsBudget.GetValueOnGameThread();
    const double BudgetSeconds=(FMath::IsFinite(RequestedBudget)?FMath::Clamp(RequestedBudget,1.f,100.f):50.f)/1000.;
    // Drop excess wall-clock demand, never simulation steps. A slow machine
    // runs below the requested rate instead of building an unbounded catch-up queue.
    DiagnosticDropped+=FMath::Max(0.,Credit-MaxSteps*UPortSimTimeStep::StepSeconds);
    Credit=FMath::Clamp(Credit,0.,MaxSteps*UPortSimTimeStep::StepSeconds);
    const int32 Steps=FMath::Min(MaxSteps,FMath::FloorToInt((Credit+1.e-9)/UPortSimTimeStep::StepSeconds));
    const double Started=FPlatformTime::Seconds();
    const uint64 InputRevision=Revision;
    bool SkippedSolver=false;
    bool WorldChanged=false;
    const FDelegateHandle SpawnHandle=World->AddOnActorSpawnedHandler(
        FOnActorSpawned::FDelegate::CreateLambda([&](AActor*) { WorldChanged=true; }));
    ON_SCOPE_EXIT { World->RemoveOnActorSpawnedHandler(SpawnHandle); };
    const bool ForcePhysics=HasOtherPhysics || FParse::Param(FCommandLine::Get(),TEXT("PortSimFullPhysics"));
    auto CommitKinematicPoses=[&](bool Force)
    {
        PortSimPerformance::FScope SyncTiming(TEXT("forward_kinematic_commit"));
        // A zero-time world frame does not consume queued kinematic targets.
        // Clear historical targets before a new free body can contact them.
        auto& Poses=Owner.SiteLogistics->KinematicPhysicsPoses;
        auto Commit=[&](UPrimitiveComponent* Part)
        {
            if(!IsValid(Part) || Part->IsAnySimulatingPhysics()) return;
            const FTransform Pose=Part->GetComponentTransform();
            const FTransform* Previous=Poses.Find(Part);
            if(!Force && Previous && Previous->Equals(Pose,0.)) return;
            if(auto* Body=Part->GetBodyInstance())
            {
                Body->SetBodyTransform(Pose,ETeleportType::TeleportPhysics,false);
                Poses.Add(Part,Pose);
            }
        };
        for(auto* Part:KinematicParts) Commit(Part);
        for(const auto& Box:Owner.SiteLogistics->DirtyKinematicCargo) if(Box.IsValid()) Commit(Box->GetBody());
        Owner.SiteLogistics->DirtyKinematicCargo.Reset();
    };
    {
        TGuardValue<bool> Guard(AdvancingPhysics,true);
        for(int32 I=0;I<Steps;++I)
        {
            const double ModelStarted=FPlatformTime::Seconds();
            Owner.AdvanceTerminalReplayStep(1.f/60.f);
            DiagnosticModel+=FPlatformTime::Seconds()-ModelStarted;
            int32 FreeCargoCount=0;
            for(const auto& Cargo:Owner.SiteLogistics->FreeCargo)
                if(Cargo.IsValid() && Cargo->GetBody()->IsSimulatingPhysics()) ++FreeCargoCount;
            DiagnosticFreeCargoPeak=FMath::Max(DiagnosticFreeCargoPeak,FreeCargoCount);
            const bool HasRecoveryBody=RecoveryBodies.ContainsByPredicate(
                [](const auto* Part) { return IsValid(Part) && Part->IsAnySimulatingPhysics(); });
            const bool NeedsSolver=ForcePhysics || WorldChanged || FreeCargoCount>0 || HasRecoveryBody;
            if(NeedsSolver)
            {
                const double PhysicsStarted=FPlatformTime::Seconds();
                PortSimPerformance::FScope PhysicsTiming(TEXT("forward_chaos_step"));
                if(SkippedSolver)
                {
                    const double TransitionStarted=FPlatformTime::Seconds();
                    CommitKinematicPoses(true);
                    DiagnosticTransition+=FPlatformTime::Seconds()-TransitionStarted;
                }
                const double ContactStarted=FPlatformTime::Seconds();
                Owner.SiteLogistics->SynchronizeFreeCargoCollision(1.f/60.f);
                for(auto* Part:RecoveryBodies) if(IsValid(Part) && Part->IsAnySimulatingPhysics())
                {
                    FBox Sweep=Part->Bounds.GetBox();
                    const FVector Travel=Part->GetPhysicsLinearVelocity()/60.;
                    Sweep+=FBox(Sweep.Min+Travel,Sweep.Max+Travel);
                    Owner.SiteLogistics->SynchronizeShipCollision(Sweep.ExpandBy(Part->Bounds.SphereRadius+100.),1.);
                }
                DiagnosticContactSync+=FPlatformTime::Seconds()-ContactStarted;
                const double SolverStarted=FPlatformTime::Seconds();
                Scene->SetUpForFrame(&Gravity,1.f/60.f,Settings->MinPhysicsDeltaTime,
                    Settings->MaxPhysicsDeltaTime,PhysicsSubstep,Settings->MaxSubsteps,Settings->bSubstepping);
                Scene->StartFrame();
                Scene->WaitPhysScenes();
                Scene->EndFrame(); // Publish real contact/velocity results before the next controller observation.
                DiagnosticSolver+=FPlatformTime::Seconds()-SolverStarted;
                ++DiagnosticPhysicsCalls;
                DiagnosticPhysics+=FPlatformTime::Seconds()-PhysicsStarted;
            }
            // No free bodies means there is no rigid-body time to integrate.
            // Controllers, suspension and collision queries still run at their
            // original cadence; release is checked after every controller step.
            SkippedSolver=!NeedsSolver;
            ++DisplayedSteps;
            ++DiagnosticSteps;
            if(Revision!=InputRevision) break;
            Credit-=UPortSimTimeStep::StepSeconds;
            if(WorldChanged) break; // Revalidate externally spawned objects next frame.
            if(!Owner.bAutoRunning || Owner.bAutoPaused || Owner.bEmergencyStop || !Owner.SiteLogistics->Fault.IsEmpty()) break;
            if(I+1==MaxSteps) ++DiagnosticStepHits;
            if(FPlatformTime::Seconds()-Started>=BudgetSeconds)
            { if(I+1<Steps) ++DiagnosticBudgetHits; break; }
        }
    }
    Owner.SiteLogistics->PublishCargoPresentation();
    if(SkippedSolver) CommitKinematicPoses(false);
    Owner.SiteLogistics->ExportDashboard(Owner.IsOperationPaused());
    PortSimPerformance::Report(Owner.SiteLogistics->SimulationTime,Owner.SiteLogistics->Delivered,World);
    DiagnosticFrame+=FPlatformTime::Seconds()-FrameStarted;
    ++DiagnosticBatches;
    const double Now=FPlatformTime::Seconds();
    if(DiagnosticWall==0) DiagnosticWall=FrameStarted;
    if(Now-DiagnosticWall>=5.)
    {
        const double Wall=Now-DiagnosticWall;
        UE_LOG(LogTemp,Display,TEXT("PORTSIM_PLAYBACK_COST: requested=%.1fx achieved=%.3fx frames=%llu steps=%llu steps_per_frame=%.2f model_ms_per_step=%.3f physics_ms_per_step=%.3f frame_overhead_ms=%.3f outside_batch_ms=%.3f budget_hits=%llu step_cap_hits=%llu dropped_sim_s=%.3f"),
            Owner.SimulationSpeed,DiagnosticSteps*UPortSimTimeStep::StepSeconds/Wall,
            DiagnosticBatches,DiagnosticSteps,double(DiagnosticSteps)/FMath::Max(uint64(1),DiagnosticBatches),
            DiagnosticModel*1000/FMath::Max(uint64(1),DiagnosticSteps),DiagnosticPhysics*1000/FMath::Max(uint64(1),DiagnosticSteps),
            FMath::Max(0.,DiagnosticFrame-DiagnosticModel-DiagnosticPhysics)*1000/FMath::Max(uint64(1),DiagnosticBatches),
            FMath::Max(0.,Wall-DiagnosticFrame)*1000/FMath::Max(uint64(1),DiagnosticBatches),
            DiagnosticBudgetHits,DiagnosticStepHits,DiagnosticDropped);
        UE_LOG(LogTemp,Display,TEXT("PORTSIM_PHYSICS_COST: sim_s=%.3f free_cargo_peak=%d physics_steps=%llu solver_ms_per_call=%.3f contact_sync_ms_per_call=%.3f transition_ms=%.3f inline=%d max_substep_s=%.9f"),
            Owner.SiteLogistics->SimulationTime,DiagnosticFreeCargoPeak,DiagnosticPhysicsCalls,
            DiagnosticSolver*1000/FMath::Max(uint64(1),DiagnosticPhysicsCalls),
            DiagnosticContactSync*1000/FMath::Max(uint64(1),DiagnosticPhysicsCalls),
            DiagnosticTransition*1000,UseInlinePhysics?1:0,PhysicsSubstep);
        ResetDiagnostics(Now);
    }
    return true;
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
    Site->ProcessDashboardCommands(); // Accepted commands invalidate accumulated demand.
    // Install dispatch policies before the first forward step.
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
        Credit=0;
        ResetDiagnostics();
        // Disabled batching is a safe single-step fallback, not a second
        // accelerated path with a different budget or catch-up policy.
        if(Clock->GetEffectiveRate()>1.f) Clock->SetLiveFallback(true);
        State=Clock->GetEffectiveRate()<Owner.SimulationSpeed?TEXT("live_input_hold"):TEXT("live");
        return false;
    }
    if(FMath::IsFinite(WallSeconds) && WallSeconds>0.) Credit+=FMath::Min(WallSeconds,1.)*Clock->GetEffectiveRate();
    // All accelerated steps share one forward solver and bounded demand policy.
    if(AdvanceLivePhysics(Owner)) return true;
    Credit=0;
    ResetDiagnostics();
    Clock->SetLiveFallback(true);
    State=TEXT("live_external_physics");
    return false;
}
