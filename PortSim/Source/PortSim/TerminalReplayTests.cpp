#if WITH_DEV_AUTOMATION_TESTS
#include "TerminalCheckpoint.h"
#include "TerminalReplay.h"
#include "QuayCrane.h"
#include "PortSiteLogistics.h"
#include "PortAGVActor.h"
#include "PortWorkingCrane.h"
#include "PortContainerActor.h"
#include "PortSimTimeStep.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/World.h"
#include "Misc/AutomationTest.h"
#include "Misc/CommandLine.h"
#include "Misc/ScopeExit.h"
#include "HAL/IConsoleManager.h"
#include "Physics/Experimental/PhysScene_Chaos.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTerminalReplayCheckpointTest,"PortSim.Playback.TerminalCheckpoint",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FTerminalReplayCheckpointTest::RunTest(const FString& Parameters)
{
    const FString Command=FCommandLine::Get();
    FCommandLine::Append(TEXT(" -PortSimShips=1 -PortSimCargo1=9 -PortSimEnvironmentSeed=166290927"));
    ON_SCOPE_EXIT { FCommandLine::Set(*Command); };
    auto* BatchVariable=IConsoleManager::Get().FindConsoleVariable(TEXT("PortSim.BatchLivePhysics"));
    const int32 PreviousBatch=BatchVariable->GetInt();
    BatchVariable->Set(0,ECVF_SetByCode); // Exercise the unsupported/disabled fallback first.
    ON_SCOPE_EXIT { BatchVariable->Set(PreviousBatch,ECVF_SetByCode); };
    // This regression exercises a deliberately small sixteen-step limit,
    // independently of the interactive playback default.
    auto* StepVariable=IConsoleManager::Get().FindConsoleVariable(TEXT("PortSim.LivePhysicsMaxSteps"));
    const int32 PreviousSteps=StepVariable->GetInt();
    StepVariable->Set(16,ECVF_SetByCode);
    ON_SCOPE_EXIT { StepVariable->Set(PreviousSteps,ECVF_SetByCode); };
    const auto Init=UWorld::InitializationValues().AllowAudioPlayback(false).CreatePhysicsScene(true)
        .RequiresHitProxies(false).CreateNavigation(false).CreateAISystem(false);
    auto* World=UWorld::CreateWorld(EWorldType::Game,false,NAME_None,nullptr,true,ERHIFeatureLevel::Num,&Init);
    ON_SCOPE_EXIT { World->DestroyWorld(false); };
    auto* Owner=World->SpawnActor<AQuayCrane>();
    Owner->DispatchBeginPlay();
    Owner->StartAutomatic(false);
    auto* Site=Owner->SiteLogistics.Get();
    if(!TestNotNull(TEXT("Real terminal initialized"),Site)) return false;
    auto Initial=FTerminalCheckpoint::Capture(*Owner);
    if(!TestTrue(TEXT("Initial terminal has no Chaos bodies"),Initial.IsValid())) return false;
    Site->AGVThroughputController.Stage=EAGVThroughputStage::Measuring;
    Site->AGVThroughputController.TargetAGV=12;
    Site->AGVThroughputController.LastAcceptedAGV=8;
    Site->AGVThroughputController.ReferenceThroughput=42.;
    Site->AGVThroughputController.RecentThroughput=39.;
    Site->AGVThroughputController.StageElapsed=17.;
    Site->AGVThroughputController.WindowElapsed=123.;
    Site->AGVThroughputController.WindowCompleted=5;
    Site->AGVThroughputController.ActiveFleet[0]=true;
    auto ControllerCheckpoint=FTerminalCheckpoint::Capture(*Owner);
    Site->AGVThroughputController.Stage=EAGVThroughputStage::Saturated;
    Site->AGVThroughputController.TargetAGV=60;
    Site->AGVThroughputController.ReferenceThroughput=0.;
    Site->AGVThroughputController.WindowElapsed=0.;
    Site->AGVThroughputController.WindowCompleted=0;
    Site->AGVThroughputController.ActiveFleet[0]=false;
    TestTrue(TEXT("Controller checkpoint restores"),ControllerCheckpoint->Restore());
    TestTrue(TEXT("Controller stage restored"),FString(Site->AGVThroughputController.StageName())==TEXT("Measuring"));
    TestEqual(TEXT("Controller target restored"),Site->AGVThroughputController.TargetAGV,12);
    TestEqual(TEXT("Controller baseline restored"),Site->AGVThroughputController.ReferenceThroughput,42.);
    TestEqual(TEXT("Controller partial window restored"),Site->AGVThroughputController.WindowElapsed,123.);
    TestEqual(TEXT("Controller completion count restored"),Site->AGVThroughputController.WindowCompleted,5);
    TestTrue(TEXT("Active fleet roster restored"),Site->AGVThroughputController.IsFleetMember(0));
    TestTrue(TEXT("Original checkpoint restores controller state"),Initial->Restore());
    const double InitialTime=Site->SimulationTime;
    const FDateTime InitialLocalTime=Owner->Environment->GetLocalDateTime();
    auto* Vehicle=Site->Vehicles[0].Get();
    const FVector InitialPosition=Vehicle->GetActorLocation();
    Vehicle->Sensors.WheelOdometryCm=123;
    Site->RoadReservations.Add(123,{FBox(FVector::ZeroVector,FVector(100))});
    Site->Manifest[0].State=1;
    Site->Manifest[0].Actor->SetLocationOwner(ECargoOwner::AGV);
    Vehicle->SetActorLocation(InitialPosition+FVector(1000,2000,0));
    Owner->Environment->AdvanceEnvironment(.1f);
    TestTrue(TEXT("Checkpoint restores immediately"),Initial->Restore());
    TestEqual(TEXT("Environment clock restored"),Owner->Environment->GetLocalDateTime(),InitialLocalTime);
    TestTrue(TEXT("Position restored"),Vehicle->GetActorLocation().Equals(InitialPosition,1.e-6));
    TestEqual(TEXT("Sensor clock restored"),Vehicle->Sensors.WheelOdometryCm,0.f);
    TestFalse(TEXT("Reservation changes discarded"),Site->RoadReservations.Contains(123));
    TestEqual(TEXT("Cargo ownership restored"),Site->Manifest[0].Actor->LocationOwner,ECargoOwner::Ship);
    for(int32 I=0;I<8;++I) Owner->AdvanceTerminalReplayStep(1.f/60.f);
    const double ReferenceTime=Site->SimulationTime;
    const FDateTime ReferenceLocalTime=Owner->Environment->GetLocalDateTime();
    const FVector ReferencePosition=Vehicle->GetActorLocation();
    const auto ReferenceSensors=Vehicle->Sensors;
    const auto ReferenceWind=Owner->Environment->GetEffectiveWindVelocity();
    Initial->Restore();
    for(int32 I=0;I<8;++I) Owner->AdvanceTerminalReplayStep(1.f/60.f);
    TestEqual(TEXT("Recalculation preserves simulation time"),Site->SimulationTime,ReferenceTime);
    TestEqual(TEXT("Recalculation preserves environment clock"),Owner->Environment->GetLocalDateTime(),ReferenceLocalTime);
    TestTrue(TEXT("Recalculation preserves AGV position"),Vehicle->GetActorLocation().Equals(ReferencePosition,1.e-6));
    TestEqual(TEXT("Recalculation preserves sensor odometry"),Vehicle->Sensors.WheelOdometryCm,ReferenceSensors.WheelOdometryCm);
    TestTrue(TEXT("Random weather state restored"),Owner->Environment->GetEffectiveWindVelocity().Equals(ReferenceWind,1.e-9));
    Initial->Restore();
    Owner->SimulationSpeed=8; Owner->ApplyPlaybackRate();
    // Bypass the setup input hold only in this test, via a fresh clock.
    Owner->PlaybackClock->Shutdown(nullptr); Owner->PlaybackClock->Initialize(nullptr);
    // InputGate is reset by initialization; no actual wall-clock sleeps needed.
    BatchVariable->Set(1,ECVF_SetByCode);
    TestTrue(TEXT("Zero demand does no speculative work"),Owner->TerminalReplay.Tick(*Owner,0.));
    TestEqual(TEXT("Zero demand preserves time"),Site->SimulationTime,InitialTime);
    TestEqual(TEXT("No checkpoint buffer"),Owner->TerminalReplay.BufferedSeconds(),0.);
    for(const int32 Rate : {2,4,8,16})
    {
        Owner->TerminalReplay.Invalidate();
        Owner->SimulationSpeed=Rate; Owner->ApplyPlaybackRate();
        Owner->PlaybackClock->Shutdown(nullptr); Owner->PlaybackClock->Initialize(nullptr);
        const double Start=Owner->TerminalReplay.DisplayedSeconds();
        TestTrue(TEXT("Stall uses forward execution"),Owner->TerminalReplay.Tick(*Owner,2.));
        const double Advanced=Owner->TerminalReplay.DisplayedSeconds()-Start;
        TestTrue(TEXT("All rates share sixteen-step cap"),Advanced>0 && Advanced<=16./60.+1.e-9);
        TestTrue(TEXT("All rates share bounded demand accounting"),
            FMath::Abs(Advanced+Owner->TerminalReplay.Credit-16./60.)<1.e-9);
        TestEqual(TEXT("No future snapshots allocated"),Owner->TerminalReplay.BufferedSeconds(),0.);
    }
    Owner->SimulationSpeed=8; Owner->ApplyPlaybackRate();
    const double Displayed=Site->SimulationTime;
    Owner->Environment->SetWindSpeedMetersPerSecond(7.f);
    TestEqual(TEXT("External input discards all future"),Owner->TerminalReplay.BufferedSeconds(),0.);
    TestEqual(TEXT("Input retains displayed time"),Site->SimulationTime,Displayed);
    TestEqual(TEXT("Requested speed preserved"),Owner->SimulationSpeed,8.f);
    TestEqual(TEXT("Actual clock switches to 1x"),Owner->GetEffectivePlaybackRate(),1.f);
    Owner->bAutoPaused=true;
    TestFalse(TEXT("Pause returns control to paused live path"),Owner->TerminalReplay.Tick(*Owner,1./60.));
    TestEqual(TEXT("Pause has no retained prediction"),Owner->TerminalReplay.BufferedSeconds(),0.);
    Owner->bAutoPaused=false;
    Initial->Restore();
    auto* Body=Site->Manifest[0].Actor->GetBody();
    Body->SetSimulatePhysics(true); Body->PutAllRigidBodiesToSleep();
    TestFalse(TEXT("Sleeping Chaos body blocks prediction"),FTerminalCheckpoint::CanPredict(*Owner));
    Body->SetSimulatePhysics(false);
    BatchVariable->Set(1,ECVF_SetByCode);
    const double PreviousDrag=Site->STSProfile.Dynamics.WindDrag;
    Site->STSProfile.Dynamics.WindDrag=0;
    ON_SCOPE_EXIT { Site->STSProfile.Dynamics.WindDrag=PreviousDrag; };
    auto* Drop=World->SpawnActor<APortContainerActor>();
    Site->ShipContainers.Add(Drop);
    Drop->LocationOwner=ECargoOwner::Yard;
    auto* DropBody=Drop->GetBody();
    DropBody->SetLinearDamping(0.f); // Analytic free-fall reference, independent of cargo drag tuning.
    auto ResetDrop=[&](double Height=100000.)
    {
        DropBody->SetSimulatePhysics(false);
        Drop->SetActorLocation(FVector(1000000,1000000,Height));
        DropBody->SetSimulatePhysics(true);
        DropBody->SetPhysicsLinearVelocity(FVector::ZeroVector);
        DropBody->SetPhysicsAngularVelocityInDegrees(FVector::ZeroVector);
    };
    ResetDrop();
    Owner->TerminalReplay.Invalidate();
    // Complete a requested number of fixed steps across bounded presentations,
    // including machines where the wall-time budget ends a batch early.
    auto AdvanceSteps=[&](int32 Count,int32 Batch)
    {
        while(Count>0)
        {
            const uint64 Before=Owner->TerminalReplay.DisplayedSteps;
            Owner->TerminalReplay.Credit=FMath::Min(Count,Batch)/60.;
            if(!Owner->TerminalReplay.AdvanceLivePhysics(*Owner)) return false;
            const int32 Completed=int32(Owner->TerminalReplay.DisplayedSteps-Before);
            if(Completed<=0 || Completed>FMath::Min(Count,Batch)) return false;
            Count-=Completed;
        }
        return true;
    };
    TestTrue(TEXT("Supported free cargo advances without checkpoint rewind"),AdvanceSteps(60,16));
    const double FullZ=Drop->GetActorLocation().Z,FullVz=DropBody->GetPhysicsLinearVelocity().Z;
    TestTrue(FString::Printf(TEXT("Batch advances one second of real gravity z=%.6f vz=%.6f gravity=%.6f"),FullZ,FullVz,World->GetGravityZ()),FullZ<99520. && FullZ>99480. && FullVz<-970. && FullVz>-990.);
    auto* Scene=World->GetPhysicsScene();
    Scene->StartFrame(); Scene->WaitPhysScenes(); Scene->EndFrame();
    TestTrue(TEXT("Enclosing zero-time physics frame does not integrate twice"),FMath::Abs(Drop->GetActorLocation().Z-FullZ)<1.e-6);
    ResetDrop();
    TestTrue(TEXT("Split fixed-step physics batch succeeds"),AdvanceSteps(60,4));
    TestTrue(TEXT("Batch partition preserves real physics position"),FMath::Abs(Drop->GetActorLocation().Z-FullZ)<.01);
    TestTrue(TEXT("Batch partition preserves real physics velocity"),FMath::Abs(DropBody->GetPhysicsLinearVelocity().Z-FullVz)<.01);
    auto* Floor=World->SpawnActor<APortContainerActor>();
    Floor->GetBody()->SetSimulatePhysics(false);
    Floor->SetActorLocation(FVector(1000000,1000000,0));
    ResetDrop(1000.);
    TestTrue(TEXT("Contact batch succeeds"),AdvanceSteps(120,16));
    const double ContactZ=Drop->GetActorLocation().Z;
    TestTrue(TEXT("Real collision supports the falling cargo"),ContactZ>250. && ContactZ<270.);
    ResetDrop(1000.);
    for(int32 I=0;I<120;++I)
    {
        Owner->TerminalReplay.Credit=1./60.;
        Owner->TerminalReplay.AdvanceLivePhysics(*Owner);
        Scene->StartFrame(); Scene->WaitPhysScenes(); Scene->EndFrame();
    }
    TestTrue(TEXT("Contact result matches individual fixed physics frames"),FMath::Abs(Drop->GetActorLocation().Z-ContactZ)<.1);
    Floor->Destroy();
    ResetDrop();
    Owner->SimulationSpeed=16; Owner->ApplyPlaybackRate();
    Owner->PlaybackClock->Shutdown(nullptr); Owner->PlaybackClock->Initialize(nullptr);
    Owner->TerminalReplay.Invalidate();
    const double BeforePhysicsDebt=Owner->TerminalReplay.DisplayedSeconds();
    TestTrue(TEXT("Delayed cargo playback uses forward physics"),Owner->TerminalReplay.Tick(*Owner,.25));
    const double Advanced=Owner->TerminalReplay.DisplayedSeconds()-BeforePhysicsDebt;
    TestTrue(TEXT("Physics work is bounded to sixteen fixed steps"),Advanced>0. && Advanced<=16./60.+1.e-9);
    TestTrue(TEXT("Overload drops excess wall-time demand"),
        FMath::Abs(Advanced+Owner->TerminalReplay.Credit-16./60.)<1.e-9);
    TestTrue(TEXT("Next presentation consumes only bounded physics debt"),Owner->TerminalReplay.Tick(*Owner,0.));
    TestTrue(TEXT("No unbounded catch-up survives a stall"),
        Owner->TerminalReplay.DisplayedSeconds()-BeforePhysicsDebt<=16./60.+1.e-9);
    Owner->bAutoPaused=true;
    TestFalse(TEXT("Paused cargo returns to paused world path"),Owner->TerminalReplay.Tick(*Owner,5.));
    TestEqual(TEXT("Paused time is not retained as physics debt"),Owner->TerminalReplay.Credit,0.);
    Owner->bAutoPaused=false;
    Site->ShipContainers.Remove(Drop);
    TestFalse(TEXT("External free bodies exclude terminal physics batching"),Owner->TerminalReplay.AdvanceLivePhysics(*Owner));
    Drop->Destroy();
    Owner->TerminalReplay.Invalidate(); Initial.Reset();
    return true;
}
#endif
