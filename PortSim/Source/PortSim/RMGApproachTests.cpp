#if WITH_DEV_AUTOMATION_TESTS
#include "PortWorkingCrane.h"
#include "PortContainerActor.h"
#include "Engine/World.h"
#include "Misc/AutomationTest.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRMGApproachTest,"PortSim.RMG.PreArrivalApproach",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FRMGApproachTest::RunTest(const FString& Parameters)
{
    const auto Init=UWorld::InitializationValues().AllowAudioPlayback(false).CreatePhysicsScene(true)
        .RequiresHitProxies(false).CreateNavigation(false).CreateAISystem(false);
    UWorld* World=UWorld::CreateWorld(EWorldType::Game,false,NAME_None,nullptr,true,ERHIFeatureLevel::Num,&Init);
    if(!TestNotNull(TEXT("Approach test world"),World)) return false;
    auto* Crane=World->SpawnActor<APortWorkingCrane>(FVector(18301.38,-13500,0),FRotator(0,90,0));
    const FVector Source(16336.77,-12180,349.5),Destination(18301.38,-14100,149.5);
    Crane->Configure(19,false,Source,Destination,false);
    Crane->Head.Z=Crane->SafeZ-200;
    Crane->UpdateParts();
    const FVector Dock=Source+Crane->Orientation.RotateVector(FVector(-500,0,0));
    TestTrue(TEXT("Reserved RMG accepts approach without AGV or cargo"),Crane->PrepareRMGApproach(Dock));
    const FVector Before=Crane->HeadPosition();
    Crane->Advance(1,true);
    TestTrue(TEXT("Pause prevents pre-arrival motion"),Crane->HeadPosition().Equals(Before,.01));
    Crane->Advance(.5f,false);
    TestTrue(TEXT("Retracts before horizontal travel"),FVector::Dist2D(Crane->HeadPosition(),Before)<1);
    TestTrue(TEXT("Retracts upward"),Crane->HeadPosition().Z>Before.Z);
    for(int32 I=0;I<1200 && Crane->Fault.IsEmpty();++I) Crane->Advance(.05f,false);
    TestTrue(TEXT("Empty approach retains valid sensor observations"),Crane->Fault.IsEmpty());
    TestTrue(TEXT("Crane travels before AGV arrival"),FVector::Dist2D(Crane->HeadPosition(),Before)>100);
    const FVector Measured=Crane->Local(Crane->HeadPosition());
    TestTrue(TEXT("Approach reaches dock at safe height"),Measured.Equals(Crane->RMGApproachTarget,10));
    TestFalse(TEXT("Approach never carries cargo"),Crane->bCarrying);
    TestFalse(TEXT("Approach does not start pickup"),Crane->IsBusy());
    auto* Cargo=World->SpawnActor<APortContainerActor>();
    TestTrue(TEXT("Arriving cargo can start its normal interlocked job"),Crane->AssignCargo(Cargo,Source,Destination,false,true));
    TestFalse(TEXT("Cargo assignment ends approach"),Crane->bRMGApproach);
    Crane->ResetOperation();
    TestFalse(TEXT("Reset clears approach"),Crane->bRMGApproach);
    TestTrue(TEXT("A new reservation can approach again"),Crane->PrepareRMGApproach(Dock));
    Crane->CancelRMGApproach();
    const FVector Cancelled=Crane->HeadPosition();
    Crane->Advance(1,false);
    TestTrue(TEXT("Cancellation prevents further travel"),Crane->HeadPosition().Equals(Cancelled,.01));
    TestTrue(TEXT("Approach can restart after cancellation"),Crane->PrepareRMGApproach(Dock));
    Crane->SetSensorFailure(true);
    Crane->Advance(.1f,false);
    TestFalse(TEXT("Sensor failure stops the empty approach"),Crane->Fault.IsEmpty());
    Crane->Destroy(); Cargo->Destroy(); World->DestroyWorld(false);
    return true;
}
#endif
