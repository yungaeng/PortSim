#if WITH_DEV_AUTOMATION_TESTS
#include "PortWorkingCrane.h"
#include "PortContainerActor.h"
#include "PortAGVActor.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/World.h"
#include "Misc/AutomationTest.h"
#include "Misc/ScopeExit.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSTSDepartureClearanceTest,"PortSim.STS.DepartureClearance",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FSTSDepartureClearanceTest::RunTest(const FString& Parameters)
{
    const auto Init=UWorld::InitializationValues().AllowAudioPlayback(false).CreatePhysicsScene(true)
        .RequiresHitProxies(false).CreateNavigation(false).CreateAISystem(false);
    auto* World=UWorld::CreateWorld(EWorldType::Game,false,NAME_None,nullptr,true,ERHIFeatureLevel::Num,&Init);
    ON_SCOPE_EXIT { World->DestroyWorld(false); };
    auto* Crane=World->SpawnActor<APortWorkingCrane>();
    auto* Vehicle=World->SpawnActor<APortAGVActor>();
    auto* Cargo=World->SpawnActor<APortContainerActor>();
    Crane->Configure(1,true,Vehicle->CargoPosition(),Vehicle->CargoPosition(),false);
    Cargo->GetBody()->SetSimulatePhysics(false);
    Cargo->SetActorLocation(Vehicle->CargoPosition());
    Crane->CargoActor=Cargo; Crane->HandoverAGV=Vehicle;
    Crane->bExternalJobs=true; Crane->bCarrying=false; Crane->Stage=6; Crane->SourceSlot=0;
    Crane->Observation.bValid=true; Crane->Observation.Timestamp=Crane->SimulationTime;
    for(int32 I=0;I<4;++I)
    { Crane->CornerLocked[I]=false; Crane->Observation.Locked[I]=false; Crane->Pickup.RequestLocks[I]=false; }
    const double Top=Cargo->GetBody()->Bounds.GetBox().Max.Z;
    const auto SetGap=[&](double Gap)
    { Crane->Spreader->AddWorldOffset(FVector(0,0,Top+Gap-Crane->Spreader->Bounds.GetBox().Min.Z)); };
    SetGap(49);
    TestFalse(TEXT("Spreader still inside departure clearance"),Crane->CanFinishSTSRetraction());
    SetGap(51);
    TestTrue(TEXT("Cleared spreader can finish without full safe-height retraction"),Crane->CanFinishSTSRetraction());
    Crane->CornerLocked[0]=true;
    TestFalse(TEXT("Remaining lock prevents departure"),Crane->CanFinishSTSRetraction());
    Crane->CornerLocked[0]=false; Crane->Observation.bValid=false;
    TestFalse(TEXT("Invalid sensor sample prevents early departure"),Crane->CanFinishSTSRetraction());
    Crane->Observation.bValid=true; Crane->bReverseTransfer=true;
    TestFalse(TEXT("Ship loading retains normal retraction"),Crane->CanFinishSTSRetraction());
    return true;
}
#endif
