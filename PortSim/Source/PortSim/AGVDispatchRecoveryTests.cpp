#if WITH_DEV_AUTOMATION_TESTS
#include "PortSiteLogistics.h"
#include "PortAGVActor.h"
#include "PortWorkingCrane.h"
#include "PortContainerActor.h"
#include "TerminalLayout.h"
#include "Engine/World.h"
#include "Misc/AutomationTest.h"
#include "Misc/Paths.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBlockedPickupDispatchTest,"PortSim.Traffic.BlockedParkingDispatch",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FBlockedPickupDispatchTest::RunTest(const FString& Parameters)
{
    const auto Init=UWorld::InitializationValues().AllowAudioPlayback(false).CreatePhysicsScene(true)
        .RequiresHitProxies(false).CreateNavigation(false).CreateAISystem(false);
    auto* World=UWorld::CreateWorld(EWorldType::Game,false,NAME_None,nullptr,true,ERHIFeatureLevel::Num,&Init);
    if(!TestNotNull(TEXT("Dispatch recovery world"),World))return false;
    auto* Site=World->SpawnActor<APortSiteLogistics>();
    Site->LaneCount=9;Site->YardCraneCount=0;Site->bDepotParking=true;Site->bReuseFreedApron=true;
    Site->bCargoAlignedHandover=false;
    Site->STSOwners.Init(INDEX_NONE,9);Site->NextVehicles.Init(INDEX_NONE,9);
    Site->Jobs.SetNum(2);Site->TrafficClock=100;
    for(int I=0;I<2;++I)
    {
        auto* V=World->SpawnActor<APortAGVActor>();V->InitializeVehicle(100+I);
        Site->Vehicles.Add(V);Site->TrafficVehicles.Add(V);V->ResetVehicle(Site->FleetPark(I));
    }
    auto* Old=Site->Vehicles[0].Get();auto* Replacement=Site->Vehicles[1].Get();
    const auto OldPose=Old->GetActorTransform(),NewPose=Replacement->GetActorTransform();
    const FVector Gate=Old->GetActorLocation()+FVector(0,1000,0);
    // Keep this reservation inside the original bay throat. A wider box also
    // clips the replacement's turn and leaves two separate blockers in the test.
    Site->RoadReservations.Add(999,{FBox(Gate-FVector(100,200,200),Gate+FVector(100,200,200))});
    TestFalse(TEXT("Occupied departure cannot receive another idle dispatch"),Site->ParkingDepartureAvailable(0));
    TestTrue(TEXT("Unblocked neighbouring departure is usable"),Site->ParkingDepartureAvailable(1));
    auto Blocked=[&](int Stage)
    {
        Site->Jobs[0]=FSiteTransfer();Site->Jobs[0].Stage=Stage;Site->Jobs[0].STS=0;
        Site->NextVehicles[0]=Stage==7?0:INDEX_NONE;
        Site->RoadWaitSince.Add(100,0);Old->SetFMSHold(true);
    };
    Blocked(7);Old->Speed=1;Site->ReassignBlockedParkingVehicles();
    TestEqual(TEXT("Moving AGV keeps its mission"),Site->Jobs[0].Stage,7);
    TestFalse(TEXT("Direct returns cannot evict a moving standby"),Site->CanReleaseParkedStandby(0));
    Old->Speed=0;Site->TrafficRecoveryTargets.Add(100,OldPose.GetLocation());
    Site->ReassignBlockedParkingVehicles();
    TestTrue(TEXT("An active road recovery keeps its original mission"),Site->Jobs[0].Stage==7 && !Site->CanReleaseParkedStandby(0));
    Site->TrafficRecoveryTargets.Remove(100);
    Old->Speed=0;Old->SetActorLocation(OldPose.GetLocation()+FVector(0,100,0));Site->ReassignBlockedParkingVehicles();
    TestEqual(TEXT("AGV outside its own bay is never cancelled"),Site->Jobs[0].Stage,7);
    Old->SetActorTransform(OldPose);
    TestTrue(TEXT("Only an unentered parked standby can yield to a nearby return"),Site->CanReleaseParkedStandby(0));
    Site->Jobs[1].Stage=8;
    Site->ReassignBlockedParkingVehicles();
    TestTrue(TEXT("Blocked standby keeps its assignment when no replacement can leave"),
        Site->Jobs[0].Stage==7 && Site->NextVehicles[0]==0 && Site->RoadWaitSince.Contains(100));
    Site->Jobs[1]=FSiteTransfer();
    Site->NextVehicles[0]=INDEX_NONE;
    Site->ReleaseUnenteredParkingMission(0);
    TestTrue(TEXT("Unentered standby releases its queue and receives a retry cooldown"),
        Site->Jobs[0].Stage==0 && Site->NextVehicles[0]==INDEX_NONE && Site->Jobs[0].DispatchRetryAfter>Site->TrafficClock);
    TestTrue(TEXT("Cancellation leaves the physical vehicle in its bay"),Old->GetActorTransform().Equals(OldPose));

    FSTSOperatingProfile Profile;
    if(!TestTrue(TEXT("Reference profile loads"),Profile.Load(FPaths::ProjectDir()/TEXT("../Document/STS/STS_ReferenceData.json"),
        FPaths::ProjectConfigDir()/TEXT("STS_Simulation.json")))) {World->DestroyWorld(false);return false;}
    const FVector Home(Site->QuayPark(0).X-2800,TerminalLayout::STSCenterY(0)*100.f,0);
    const FVector Source=Home+FVector(-2800,0,569.5);
    auto* Crane=World->SpawnActor<APortWorkingCrane>(Home,FRotator::ZeroRotator);Crane->SetSTSProfile(Profile);
    Crane->Configure(1,true,Source,Site->QuayPark(0)+FVector(0,0,349.5),false);
    Site->Equipment.Add(Crane);
    auto* Cargo=World->SpawnActor<APortContainerActor>(Source,FRotator::ZeroRotator);
    Cargo->InitializeContainer(9001);
    FSiteShipCargo M;M.Actor=Cargo;M.ID=9001;M.STS=0;M.State=1;M.PlannedSlot=0;
    M.Transform=Cargo->GetActorTransform();Site->Manifest.Add(M);
    FSiteYardSlot Slot;Slot.Crane=0;Slot.Block=0;Slot.Occupied=false;Site->Yard.Add(Slot);
    Site->YardApproachOwners.Add(0);Site->DeferredYardLeases.Add({0,0});
    Blocked(6);Site->Jobs[0].Cargo=Site->Jobs[0].Slot=Site->Jobs[0].RMG=0;
    Site->Jobs[0].Actor=Cargo;Site->Jobs[0].TripID=77;Site->Jobs[0].bYardAdvanceBooking=true;
    Site->STSOwners[0]=0;
    TestTrue(TEXT("STS pickup binds the parked original vehicle"),Crane->AssignCargo(Cargo,Cargo->GetActorLocation(),
        Site->QuayPark(0)+FVector(0,0,349.5),false,false,Old));
    Crane->SetDestinationReady(false);
    const auto CargoPose=Cargo->GetActorTransform();
    const float ExitY=TerminalLayout::AGVStandbyExitY(1);
    const FVector ExitObstruction(TerminalLayout::SiteCmX(7800),ExitY,0);
    Site->RoadReservations.Add(998,{FBox(ExitObstruction-FVector(100,100,250),ExitObstruction+FVector(100,100,250))});
    TestTrue(TEXT("Replacement has an open bay throat but a blocked exit corridor"),Site->ParkingDepartureAvailable(1));
    Site->ReassignBlockedParkingVehicles();
    TestTrue(TEXT("A replacement cannot move a pickup into the same traffic queue"),
        Site->STSOwners[0]==0 && Site->Jobs[1].Stage==0 && Crane->GetHandoverVehicle()==Old &&
        Site->RoadWaitSince.Contains(100) && Site->RecoveryEvents.IsEmpty());
    Site->RoadReservations.Remove(998);
    Site->ReservationIndex.Remove(998);
    Site->ReassignBlockedParkingVehicles();
    TestEqual(TEXT("Waiting pickup is not checked again every frame"),Site->STSOwners[0],0);
    Site->TrafficClock+=10.;
    Site->ReassignBlockedParkingVehicles();
    TestTrue(TEXT("Replacement retains exact cargo, trip and admission state"),
        Site->Jobs[1].Stage==6 && Site->Jobs[1].Actor.Get()==Cargo && Site->Jobs[1].TripID==77 && Site->Jobs[1].bYardAdvanceBooking);
    TestTrue(TEXT("Crane and yard leases consistently bind the replacement"),
        Site->STSOwners[0]==1 && Crane->GetHandoverVehicle()==Replacement &&
        Site->YardApproachOwners[0]==1 && Site->DeferredYardLeases[0].Vehicle==1);
    TestTrue(TEXT("Replacement follows a real route and original stays parked"),
        !Site->Jobs[1].Route.IsEmpty() && Site->Jobs[0].Stage==0 &&
        Old->GetActorTransform().Equals(OldPose) && Replacement->GetActorTransform().Equals(NewPose));
    TestTrue(TEXT("Reassignment never moves or duplicates the container"),Cargo->GetActorTransform().Equals(CargoPose) && Site->Manifest.Num()==1);
    TestTrue(TEXT("Old unentered road request is released"),!Site->RoadWaitSince.Contains(100) && !Old->Sensors.bFMSHold);
    TestEqual(TEXT("One recovery event is retained"),Site->RecoveryEvents.Num(),1);

    // Like V118 in the terminal trace, bay 2 has enough straight aisle
    // between the two turns to stop clear of the depot's southbound spine.
    auto* Departing=World->SpawnActor<APortAGVActor>();
    Departing->InitializeVehicle(102);
    Site->Vehicles.Add(Departing);
    Site->TrafficVehicles.Add(Departing);
    Departing->ResetVehicle(Site->FleetPark(2));
    auto& Departure=Site->Jobs.AddDefaulted_GetRef();
    Departure.Stage=7;
    Departure.STS=0;
    const FVector SecondTurn(TerminalLayout::SiteCmX(TerminalLayout::AGVDepotOutboundX),TerminalLayout::AGVStandbyExitY(2),0);
    const FBox BusySpine(SecondTurn-FVector(300,300,250),SecondTurn+FVector(300,300,250));
    Site->RoadReservations.Add(997,{BusySpine});
    Site->ReservationIndex.Set(997,{BusySpine});
    if(TestTrue(TEXT("Depot route can start while the separate spine turn is occupied"),Site->PlanRoadRoute(2,false,true)))
    {
        Site->MoveVehicle(Departing,Departure.Route[0],.05f);
        const int32* Approved=Site->RoadReservationEndWaypoint.Find(102);
        const auto* Lease=Site->RoadReservations.Find(102);
        TestTrue(TEXT("The first lease reaches a straight stop before the spine"),Approved && Lease &&
            Departure.Route.IsValidIndex(*Approved) &&
            FMath::IsNearlyEqual(Departure.Route[*Approved].Y,SecondTurn.Y,.1) &&
            Departure.Route[*Approved].X<SecondTurn.X-1400.);
        if(Lease) TestFalse(TEXT("The first lease does not claim the occupied spine"),
            Lease->ContainsByPredicate([&](const FBox& Segment) { return Segment.Intersect(BusySpine); }));
    }
    World->DestroyWorld(false);return true;
}
#endif
