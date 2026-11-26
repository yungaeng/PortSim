#if WITH_DEV_AUTOMATION_TESTS
#include "PortSiteLogistics.h"
#include "PortEnvironmentComponent.h"
#include "PortAGVActor.h"
#include "PortWorkingCrane.h"
#include "PortContainerActor.h"
#include "Engine/World.h"
#include "Dom/JsonObject.h"
#include "Misc/AutomationTest.h"
#include "Misc/Paths.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FDashboardEnvironmentTest,"PortSim.Dashboard.EnvironmentObservation",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FDashboardEnvironmentTest::RunTest(const FString& Parameters)
{
    const auto Init=UWorld::InitializationValues().AllowAudioPlayback(false).CreatePhysicsScene(true)
        .RequiresHitProxies(false).CreateNavigation(false).CreateAISystem(false);
    auto* W=UWorld::CreateWorld(EWorldType::Game,false,NAME_None,nullptr,true,ERHIFeatureLevel::Num,&Init);
    auto* Owner=W->SpawnActor<AActor>();auto* E=NewObject<UPortEnvironmentComponent>(Owner);
    Owner->AddInstanceComponent(E);E->RegisterComponent();
    auto* Site=W->SpawnActor<APortSiteLogistics>();Site->SetOwner(Owner);
    E->SetRainEnabled(true);E->SetRainfallMmPerHour(30);E->SetSnowEnabled(true);E->SetSnowfallMmPerHour(10);
    E->SetWindSpeedMetersPerSecond(5);E->SetWaterFilmMm(.5);
    auto Snapshot=Site->CaptureEnvironment();
    TestEqual(TEXT("Read-only web snapshot observes the Unreal rainfall"),Snapshot->GetNumberField(TEXT("rain_mm_h")),30.);
    TestEqual(TEXT("Read-only web snapshot observes actual water film"),Snapshot->GetNumberField(TEXT("water_film_mm")),.5);
    TestTrue(TEXT("Current coefficient comes from the same environment state"),FMath::IsNearlyEqual(
        Snapshot->GetNumberField(TEXT("tire_mu_static")),double(E->GetFrictionSettings().Tire.Static(E->GetSurfaceConditions())),.00001));
    W->DestroyWorld(false);return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAdvanceYardBookingTest,"PortSim.Logistics.BoundedNextYardService",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FAdvanceYardBookingTest::RunTest(const FString& Parameters)
{
    const auto Init=UWorld::InitializationValues().AllowAudioPlayback(false).CreatePhysicsScene(true)
        .RequiresHitProxies(false).CreateNavigation(false).CreateAISystem(false);
    auto* W=UWorld::CreateWorld(EWorldType::Game,false,NAME_None,nullptr,true,ERHIFeatureLevel::Num,&Init);
    auto* Site=W->SpawnActor<APortSiteLogistics>();Site->LaneCount=9;Site->YardCraneCount=1;
    auto* R=W->SpawnActor<APortWorkingCrane>();R->Configure(1,false,FVector(-1000,0,349.5),FVector(1000,0,149.5),false);
    Site->Equipment.Add(R);Site->BlocksBusy.Init(false,1);Site->YardApproachOwners.Init(0,1);
    Site->RMGBusy.Init(true,1);Site->SlotAssigned.Init(false,3);Site->Jobs.SetNum(3);
    Site->NextVehicles.Init(INDEX_NONE,9);
    for(int32 I=0;I<3;++I)
    {
        auto* V=W->SpawnActor<APortAGVActor>(FVector(I*2000,5000,0),FRotator::ZeroRotator);V->InitializeVehicle(100+I);Site->Vehicles.Add(V);
        auto* Box=W->SpawnActor<APortContainerActor>();Box->InitializeContainer(9900+I);
        FSiteShipCargo C;C.Actor=Box;C.PlannedSlot=I;Site->Manifest.Add(C);
        FSiteYardSlot S;S.Reserved=true;S.Occupied=false;S.Crane=0;S.Block=0;S.Handover=FVector(1000,1000,0);Site->Yard.Add(S);
        Site->Jobs[I].Cargo=I;Site->Jobs[I].Actor=Box;Site->Jobs[I].Stage=6;
    }
    Site->Jobs[2].Stage=7;Site->Jobs[2].STS=1;
    TestTrue(TEXT("One standby reserves its entire berth group's quay queue"),Site->HasBerthQueueTraffic(0));
    TestFalse(TEXT("The current vehicle is ignored when checking its own return"),Site->HasBerthQueueTraffic(0,2));
    Site->Jobs[2]=FSiteTransfer();Site->Jobs[2].Stage=8;Site->Jobs[2].STS=1;Site->NextVehicles[1]=2;
    TestTrue(TEXT("A returned AGV waiting at the quay also occupies the berth queue"),Site->HasBerthQueueTraffic(0));
    Site->Jobs[2]=FSiteTransfer();Site->NextVehicles[1]=INDEX_NONE;
    Site->YardApproachOwners[0]=INDEX_NONE;
    Site->RMGBusy[0]=false;
    const FTransform BookedPose=Site->Vehicles[1]->GetActorTransform();
    TestTrue(TEXT("An STS pickup reserves a supported yard service"),Site->ReserveYardApproach(1,true));
    TestTrue(TEXT("Advance booking leaves the physical approach available"),
        Site->Jobs[1].bYardAdvanceBooking && Site->SlotAssigned[1] &&
        Site->YardApproachOwners[0]==INDEX_NONE && !Site->RMGBusy[0] &&
        Site->Vehicles[1]->GetActorTransform().Equals(BookedPose));
    TestTrue(TEXT("Eligible reservation starts the RMG approach before AGV arrival"),Site->ReserveYard(1,true));
    TestTrue(TEXT("RMG approaches while the AGV stays at its current position"),
        Site->Jobs[1].bRMGReserved && Site->RMGBusy[0] && R->RailWorkInterval().Y>=999.f &&
        Site->YardApproachOwners[0]==INDEX_NONE && Site->Vehicles[1]->GetActorTransform().Equals(BookedPose));
    TestFalse(TEXT("An unpromoted pickup cannot create an unlimited service queue"),Site->CanBookNextYardService(2));
    Site->Jobs[1].Stage=2;
    TestTrue(TEXT("Loaded departure promotes its own service to approach ownership"),Site->ReserveYardApproach(1));
    TestTrue(TEXT("Loaded departure promotes the prepositioned RMG service"),
        Site->YardApproachOwners[0]==1 && !Site->Jobs[1].bYardAdvanceBooking && Site->RMGBusy[0]);
    Site->Jobs[1].bRMGReserved=false;Site->Jobs[1].RMG=INDEX_NONE;Site->RMGBusy[0]=false;
    R->CancelRMGApproach();
    Site->Jobs[1].Stage=6;
    Site->SlotAssigned[1]=false;
    Site->YardApproachOwners[0]=0;
    Site->RMGBusy[0]=true;
    auto& Prior=Site->Jobs[0];Prior.RMG=0;Prior.Stage=3;Prior.Slot=0;Prior.bRMGReserved=true;
    TestTrue(TEXT("One next STS pickup may overlap the admitted vehicle's yard journey"),Site->CanBookNextYardService(1));
    TestTrue(TEXT("Create active prior crane job"),R->AssignCargo(Site->Manifest[0].Actor.Get(),FVector(-1000,0,349.5),FVector(1000,0,149.5),false,true));
    Prior.Stage=4;R->Stage=2;R->bCarrying=false;
    TestFalse(TEXT("Cannot speculate before the prior pickup is verified"),Site->CanBookNextYardService(1));
    R->Stage=3;R->bCarrying=true;
    Site->Yard[1].Below=0;
    TestFalse(TEXT("Missing support remains a hard prohibition"),Site->CanBookNextYardService(1));
    Site->Yard[1].Below=INDEX_NONE;
    const FTransform Before=Site->Vehicles[1]->GetActorTransform();
    TestTrue(TEXT("Next STS job can reserve one future service"),Site->ReserveYardApproach(1,true));
    TestTrue(TEXT("Actual approach stays with outgoing job; vehicle remains at STS"),
        Site->YardApproachOwners[0]==0 && Site->Jobs[1].bYardAdvanceBooking && Site->Vehicles[1]->GetActorTransform().Equals(Before));
    TestFalse(TEXT("A third job cannot join the same future queue"),Site->CanBookNextYardService(2));
    TestFalse(TEXT("Future booking does not authorize driving to the yard"),Site->ReserveYardApproach(1));
    Site->YardApproachOwners[0]=INDEX_NONE;Prior.bYardReleased=true;Site->RMGBusy[0]=false;
    TestFalse(TEXT("New arrivals cannot steal the released approach from the booked job"),Site->YardApproachAvailable(2));
    TestTrue(TEXT("Only actual release promotes the booked job to road admission"),Site->ReserveYardApproach(1));
    TestTrue(TEXT("Promotion transfers ownership and clears future flag"),Site->YardApproachOwners[0]==1 && !Site->Jobs[1].bYardAdvanceBooking);
    W->DestroyWorld(false);return true;
}
#endif
