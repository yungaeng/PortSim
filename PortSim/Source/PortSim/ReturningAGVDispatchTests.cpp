#if WITH_DEV_AUTOMATION_TESTS
#include "PortSiteLogistics.h"
#include "PortAGVActor.h"
#include "TerminalLayout.h"
#include "Engine/World.h"
#include "Misc/AutomationTest.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FReturningAGVDispatchTest,"PortSim.Logistics.ReturningAGVDispatch",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FReturningAGVDispatchTest::RunTest(const FString& Parameters)
{
    const auto Init=UWorld::InitializationValues().AllowAudioPlayback(false).CreatePhysicsScene(true)
        .RequiresHitProxies(false).CreateNavigation(false).CreateAISystem(false);
    auto* World=UWorld::CreateWorld(EWorldType::Game,false,NAME_None,nullptr,true,ERHIFeatureLevel::Num,&Init);
    if(!TestNotNull(TEXT("Returning dispatch world"),World)) return false;
    auto* Site=World->SpawnActor<APortSiteLogistics>();
    Site->LaneCount=9; Site->bDepotParking=true; Site->bReuseFreedApron=true; Site->bCargoAlignedHandover=false;
    Site->Jobs.SetNum(2); Site->NextVehicles.Init(INDEX_NONE,9); Site->STSOwners.Init(INDEX_NONE,9);
    Site->PreparedCargo.Init(INDEX_NONE,9); Site->PreparedCargo[4]=0;
    FSiteShipCargo Cargo; Cargo.STS=4; Site->Manifest.Add(Cargo);
    for(int32 I=0;I<2;++I)
    {
        auto* V=World->SpawnActor<APortAGVActor>(); V->InitializeVehicle(100+I);
        Site->Vehicles.Add(V); V->ResetVehicle(Site->FleetPark(I));
    }
    auto* Returning=Site->Vehicles[1].Get();
    Returning->SetActorLocationAndRotation(FVector(TerminalLayout::SiteCmX(TerminalLayout::AGVQuayInboundLaneX),
        Site->QuayPark(4).Y+8000,0),FRotator(0,180,0));
    Returning->CompletedJobs=100;
    auto& Job=Site->Jobs[1]; Job.Stage=5; Job.bYardReleased=true;
    Job.Route={Returning->GetActorLocation()+FVector(0,-2000,0),Site->FleetPark(1)};
    const auto Pose=Returning->GetActorTransform(); const auto OldRoute=Job.Route;
    const int32 ID=Returning->VehicleID;
    Site->RoadReservations.Add(ID,{FBox(Pose.GetLocation()-FVector(500),Pose.GetLocation()+FVector(500))});
    Site->RoadReservationEndWaypoint.Add(ID,1);
    TestTrue(TEXT("Empty charged yard return can receive work after releasing its RMG"),Site->CanRedirectEmptyReturn(1));
    TestEqual(TEXT("Nearby return beats a distant depot AGV despite more completed jobs"),Site->NearestDispatchVehicle(4,false),1);
    TestTrue(TEXT("Selection preserves the return mission, reservation and physical pose"),
        Job.Stage==5 && Job.Route==OldRoute && Site->RoadReservations.Contains(ID) &&
        Site->RoadReservationEndWaypoint.FindRef(ID)==1 && Returning->GetActorTransform().Equals(Pose));
    Returning->BatteryPercent=20;
    TestEqual(TEXT("Low battery return cannot displace a charged depot AGV"),Site->NearestDispatchVehicle(4,false),0);
    Returning->BatteryPercent=60; Returning->bNeedsCharge=true;
    TestFalse(TEXT("Charge latch must finish its charge cycle"),Site->CanRedirectEmptyReturn(1));
    Returning->bNeedsCharge=false; Returning->bCharging=true;
    TestFalse(TEXT("Charging AGV is excluded"),Site->CanRedirectEmptyReturn(1)); Returning->bCharging=false;
    Job.bYardReleased=false;
    TestFalse(TEXT("RMG exit lease cannot be discarded"),Site->CanRedirectEmptyReturn(1)); Job.bYardReleased=true;
    Site->NextVehicles[2]=1;
    TestFalse(TEXT("Another STS reservation is retained"),Site->CanRedirectEmptyReturn(1)); Site->NextVehicles[2]=INDEX_NONE;
    Job.ReturnSTS=2;
    TestFalse(TEXT("A direct STS return keeps its assignment"),Site->CanRedirectEmptyReturn(1)); Job.ReturnSTS=INDEX_NONE;
    Returning->SetPayload(1000);
    TestFalse(TEXT("Loaded vehicles cannot be redirected"),Site->CanRedirectEmptyReturn(1)); Returning->SetPayload(0);
    Site->TrafficRecoveryTargets.Add(ID,Pose.GetLocation());
    TestFalse(TEXT("Traffic recovery keeps its route"),Site->CanRedirectEmptyReturn(1)); Site->TrafficRecoveryTargets.Remove(ID);
    Returning->SetActorRotation(FRotator(0,135,0));
    TestFalse(TEXT("A return finishes its turn before rerouting"),Site->CanRedirectEmptyReturn(1)); Returning->SetActorTransform(Pose);
    Job.Stage=FSiteTransfer::ReturnToStandby;
    TestEqual(TEXT("Standby return also competes by current road distance"),Site->NearestDispatchVehicle(4,false),1);
    Job.Stage=FSiteTransfer::ToCharge;
    TestTrue(TEXT("Charge-bound return needs sufficient battery and no charge latch"),Site->CanRedirectEmptyReturn(1));
    Returning->bNeedsCharge=true;
    TestFalse(TEXT("Necessary charging trip cannot be interrupted"),Site->CanRedirectEmptyReturn(1)); Returning->bNeedsCharge=false;
    Job=FSiteTransfer(); Job.Stage=6; Job.STS=4; Job.Cargo=0;
    TestTrue(TEXT("Selected return can commit its new STS route"),Site->PlanRoadRoute(1,false));
    TestTrue(TEXT("Commit releases obsolete road reservations without moving the AGV"),
        !Site->RoadReservations.Contains(ID) && !Site->RoadReservationEndWaypoint.Contains(ID) &&
        !Job.Route.IsEmpty() && Job.Route.Last().Equals(Site->QuayPark(4),.1f) && Returning->GetActorTransform().Equals(Pose));
    for(int32 STS=0;STS<9;++STS)
    {
        Site->Manifest[0].STS=STS;
        auto& Pickup=Site->Jobs[0]; Pickup=FSiteTransfer();
        Pickup.Stage=6; Pickup.STS=STS; Pickup.Cargo=0; Pickup.bParkingDeparture=true;
        TestTrue(FString::Printf(TEXT("Depot route reaches expanded STS %d"),STS+1),Site->PlanRoadRoute(0,false));
    }
    World->DestroyWorld(false); return true;
}
#endif
