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

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTerminalReplayCheckpointTest,"PortSim.Playback.TerminalCheckpoint",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FTerminalReplayCheckpointTest::RunTest(const FString& Parameters)
{
    const FString Command=FCommandLine::Get();
    FCommandLine::Append(TEXT(" -PortSimShips=1 -PortSimCargo1=9 -PortSimEnvironmentSeed=166290927"));
    ON_SCOPE_EXIT { FCommandLine::Set(*Command); };
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
    const double InitialTime=Site->SimulationTime;
    auto* Vehicle=Site->Vehicles[0].Get();
    const FVector InitialPosition=Vehicle->GetActorLocation();
    Vehicle->Sensors.WheelOdometryCm=123;
    Site->RoadReservations.Add(123,{FBox(FVector::ZeroVector,FVector(100))});
    Site->Manifest[0].State=1;
    Site->Manifest[0].Actor->LocationOwner=ECargoOwner::AGV;
    Vehicle->SetActorLocation(InitialPosition+FVector(1000,2000,0));
    Owner->Environment->AdvanceEnvironment(.1f);
    TestTrue(TEXT("Checkpoint restores immediately"),Initial->Restore());
    TestTrue(TEXT("Position restored"),Vehicle->GetActorLocation().Equals(InitialPosition,1.e-6));
    TestEqual(TEXT("Sensor clock restored"),Vehicle->Sensors.WheelOdometryCm,0.f);
    TestFalse(TEXT("Reservation changes discarded"),Site->RoadReservations.Contains(123));
    TestEqual(TEXT("Cargo ownership restored"),Site->Manifest[0].Actor->LocationOwner,ECargoOwner::Ship);
    for(int32 I=0;I<8;++I) Owner->AdvanceTerminalReplayStep(1.f/60.f);
    const double ReferenceTime=Site->SimulationTime;
    const FVector ReferencePosition=Vehicle->GetActorLocation();
    const auto ReferenceSensors=Vehicle->Sensors;
    const auto ReferenceWind=Owner->Environment->GetEffectiveWindVelocity();
    Initial->Restore();
    for(int32 I=0;I<8;++I) Owner->AdvanceTerminalReplayStep(1.f/60.f);
    TestEqual(TEXT("Recalculation preserves simulation time"),Site->SimulationTime,ReferenceTime);
    TestTrue(TEXT("Recalculation preserves AGV position"),Vehicle->GetActorLocation().Equals(ReferencePosition,1.e-6));
    TestEqual(TEXT("Recalculation preserves sensor odometry"),Vehicle->Sensors.WheelOdometryCm,ReferenceSensors.WheelOdometryCm);
    TestTrue(TEXT("Random weather state restored"),Owner->Environment->GetEffectiveWindVelocity().Equals(ReferenceWind,1.e-9));
    Initial->Restore();
    Owner->SimulationSpeed=8; Owner->ApplyPlaybackRate();
    // Bypass the setup input hold only in this test, via a fresh clock.
    Owner->PlaybackClock->Shutdown(nullptr); Owner->PlaybackClock->Initialize(nullptr);
    // InputGate is reset by initialization; no actual wall-clock sleeps needed.
    TestTrue(TEXT("Real terminal prepares future"),Owner->TerminalReplay.Tick(*Owner,0.));
    TestEqual(TEXT("Prediction does not expose future simulation time"),Site->SimulationTime,InitialTime);
    TestTrue(TEXT("Preparation leaves displayed geometry unchanged"),Vehicle->GetActorLocation().Equals(InitialPosition,1.e-6));
    TestTrue(TEXT("Future buffer contains prepared physics"),Owner->TerminalReplay.BufferedSeconds()>0.);
    TestTrue(TEXT("Real terminal enters buffered replay"),Owner->TerminalReplay.Tick(*Owner,1./60.));
    TestEqual(TEXT("8x display matches eight reference physics steps"),Site->SimulationTime,ReferenceTime);
    TestTrue(TEXT("Replay geometry equals continuous model"),Vehicle->GetActorLocation().Equals(ReferencePosition,1.e-6));
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
    Owner->TerminalReplay.Invalidate(); Initial.Reset();
    return true;
}
#endif
