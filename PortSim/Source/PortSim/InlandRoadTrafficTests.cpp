#if WITH_DEV_AUTOMATION_TESTS
#include "PortSiteLogistics.h"
#include "PortAGVActor.h"
#include "PortContainerActor.h"
#include "PortWorkingCrane.h"
#include "TerminalLayout.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/World.h"
#include "Misc/AutomationTest.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FInlandRoadTrafficTest,"PortSim.Traffic.InlandRoadQueues",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FInlandRoadTrafficTest::RunTest(const FString& Parameters)
{
    const auto Init=UWorld::InitializationValues().AllowAudioPlayback(false).CreatePhysicsScene(true)
        .RequiresHitProxies(false).CreateNavigation(false).CreateAISystem(false);
    UWorld* World=UWorld::CreateWorld(EWorldType::Game,false,NAME_None,nullptr,true,ERHIFeatureLevel::Num,&Init);
    if(!TestNotNull(TEXT("Inland traffic verification world"),World)) return false;
    auto* Site=World->SpawnActor<APortSiteLogistics>();
    auto* A=World->SpawnActor<APortAGVActor>();
    auto* B=World->SpawnActor<APortAGVActor>();
    A->InitializeVehicle(100); B->InitializeVehicle(101);
    Site->LaneCount=9; Site->bDepotParking=Site->bReuseFreedApron=Site->bCargoAlignedHandover=true;
    Site->Vehicles={A,B}; Site->TrafficVehicles={A,B}; Site->Jobs.SetNum(2);
    // V106 was stopped by V128, 77 m ahead on the same northbound road,
    // despite a clear first traffic gate and a clear front LiDAR.
    const float NorthX=TerminalLayout::SiteCmX(TerminalLayout::AGVInlandInboundX);
    A->ResetVehicle(FVector(NorthX,7535.02f,0)); B->ResetVehicle(FVector(NorthX,15246.98f,0));
    auto& JA=Site->Jobs[0]; auto& JB=Site->Jobs[1];
    JA.Stage=JB.Stage=5;
    JA.Route={FVector(NorthX,9710.37,0),FVector(NorthX,11792.82,0),FVector(NorthX,15475.27,0),FVector(NorthX,20000,0)};
    JB.Route={FVector(NorthX,18249.41,0),FVector(NorthX,21153.53,0),FVector(NorthX,24057.65,0)};
    Site->BeginTrafficFrame(); Site->MoveVehicle(A,JA.Route[0],.05f);
    TestTrue(TEXT("Recorded same-direction follower moves while the first gate is clear"),A->Speed>0);
    TestFalse(TEXT("A distant same-direction leader does not cause FMS hold"),A->Sensors.bFMSHold);
    TestTrue(TEXT("Explicit front LiDAR remains clear"),A->Sensors.bLiDARClear);

    A->ResetVehicle(FVector(NorthX,0,0)); B->ResetVehicle(FVector(NorthX+5000,20000,0));
    JA.Route={FVector(NorthX,3000,0),FVector(NorthX,6000,0),FVector(NorthX,9000,0),FVector(NorthX,12000,0)};
    JA.Waypoint=0;
    Site->RoadReservations.Reset(); Site->RoadReservationEndWaypoint.Reset(); Site->RoadTargets.Reset();
    Site->bTrafficIndexReady=false;
    for(int Step=0;Step<600 && JA.Waypoint==0;++Step)
    { Site->TrafficClock+=.05; Site->BeginTrafficFrame(); Site->Drive(0,.05f); }
    TestTrue(TEXT("Empty straight road advances across a virtual traffic gate"),JA.Waypoint>0);
    TestTrue(TEXT("Rolling lease keeps driving through a clear gate instead of stopping"),A->Speed>300.f);

    // Recorded near-RMG merge starvation: new through traffic must queue
    // outside an older request's area, while vehicles inside can drain out.
    const float MergeSpine=TerminalLayout::SiteCmX(15300.f);
    A->ResetVehicle(FVector(MergeSpine,21200,0));
    B->ResetVehicle(FVector(16335.39f,22820,0)); B->SetActorRotation(FRotator(0,90,0)); B->SetFMSHold(true);
    JA.Route={FVector(MergeSpine,24500,0),FVector(MergeSpine,27700,0)};
    JA.Waypoint=0;
    Site->RoadReservations.Reset(); Site->RoadReservationEndWaypoint.Reset(); Site->RoadTargets.Reset();
    Site->TrafficClock=100; Site->RoadWaitSince.Add(101,0.);
    Site->PendingRoadRequests.Add(101,{FBox(FVector(MergeSpine-380,22300,-250),FVector(17100,28500,250))});
    Site->bTrafficIndexReady=false; Site->BeginTrafficFrame(); Site->MoveVehicle(A,JA.Route[0],.05f);
    TestTrue(TEXT("New through traffic leaves a gap for an older merge request"),A->Sensors.bFMSHold);
    TestEqual(TEXT("FMS identifies the actual waiting merge vehicle"),Site->RoadBlockers.FindRef(100),101);
    TestFalse(TEXT("A through-road waiter does not claim unrelated future junctions"),Site->PendingRoadRequests.Contains(100));
    A->ResetVehicle(FVector(MergeSpine,24500,0));
    JA.Route={FVector(MergeSpine,27700,0),FVector(MergeSpine,30700,0)};
    Site->BeginTrafficFrame(); Site->MoveVehicle(A,JA.Route[0],.05f);
    TestTrue(TEXT("A vehicle already inside the merge area can drain out"),A->Speed>0 && !A->Sensors.bFMSHold);
    Site->PendingRoadRequests.Reset(); Site->PendingRoadIndex.Reset(); Site->RoadWaitSince.Reset();

    Site->Jobs.Init(FSiteTransfer(),2); Site->Yard.SetNum(2); Site->Manifest.SetNum(2);
    Site->SlotAssigned.Init(false,2); Site->BlocksBusy.Init(false,1);
    Site->RMGBusy.Init(true,1); Site->YardApproachOwners.Init(0,1);
    // Admission now also checks that the destination crane is operational.
    Site->Equipment.Add(World->SpawnActor<APortWorkingCrane>());
    const FVector Dock(TerminalLayout::RMGHandoverX(1)*100.f,-8680,0);
    for(auto& Slot:Site->Yard)
    { Slot.Half=1; Slot.Crane=Slot.Block=0; Slot.Handover=Dock; Slot.Reserved=true; }
    Site->Yard[1].Occupied=false;
    auto* Cargo=World->SpawnActor<APortContainerActor>(); Cargo->GetBody()->SetSimulatePhysics(false);
    Site->Manifest[1].Actor=Cargo; Site->Manifest[1].PlannedSlot=1;
    Site->Jobs[0].Stage=5; Site->Jobs[0].Slot=Site->Jobs[0].RMG=0; Site->Jobs[0].bRMGReserved=true;
    Site->Jobs[1].Stage=2; Site->Jobs[1].Cargo=1;
    // V156/V117 met on this same aisle after its owner was released just 30 m
    // from the pad. The outgoing vehicle is still 100 m from the main road.
    A->ResetVehicle(FVector(48435.07f,-8680,0)); A->SetActorRotation(FRotator(0,-90,0));
    Site->ReleaseYardApproachIfClear(0);
    TestEqual(TEXT("Far RMG keeps its aisle while the outgoing vehicle is still inside"),Site->YardApproachOwners[0],0);
    TestFalse(TEXT("Next loaded AGV cannot enter against the outgoing vehicle"),Site->ReserveYardApproach(1));
    A->ResetVehicle(FVector(NorthX,Dock.Y+2000,0));
    Site->ReleaseYardApproachIfClear(0);
    TestEqual(TEXT("A fully cleared main-road merge releases the aisle"),Site->YardApproachOwners[0],INDEX_NONE);
    TestTrue(TEXT("Next loaded AGV is admitted after the exit clears"),Site->ReserveYardApproach(1));

    Site->Jobs.Init(FSiteTransfer(),2); Site->YardApproachOwners.Init(INDEX_NONE,1);
    Site->Manifest[0].Actor=Cargo; Site->Manifest[0].STS=4;
    Site->Manifest[0].Transform=FTransform(FVector(-4900,-15100,400));
    Site->Jobs[0].Stage=5; Site->Jobs[0].Cargo=Site->Jobs[0].Slot=0; Site->Jobs[0].STS=4;
    A->ResetVehicle(Dock); B->ResetVehicle(FVector(TerminalLayout::SiteCmX(4800),60000,0));
    Site->RoadReservations.Reset(); Site->RoadReservationEndWaypoint.Reset(); Site->RoadTargets.Reset();
    Site->bTrafficIndexReady=false; Site->BeginTrafficFrame();
    TestTrue(TEXT("Uncongested far RMG has a legal depot return"),Site->PlanRoadRoute(0,false));
    TestEqual(TEXT("Empty north corridor is preferred by actual route distance"),Site->Jobs[0].ReturnCorridor,2);
    const FBox Busy(FVector(NorthX-380,-6000,-250),FVector(NorthX+380,33000,250));
    for(int ID=900;ID<905;++ID) Site->RoadReservations.Add(ID,{Busy});
    Site->bTrafficIndexReady=false; Site->BeginTrafficFrame(); A->ResetVehicle(Dock);
    TestTrue(TEXT("Congested return can select another existing cross-terminal road"),Site->PlanRoadRoute(0,false));
    TestTrue(TEXT("Live initial planning diverts from the occupied north spine"),Site->Jobs[0].ReturnCorridor!=2);
    TestTrue(TEXT("Return still ends at its own depot bay"),Site->Jobs[0].Route.Last().Equals(Site->FleetPark(0),.1));

    Site->RoadReservations.Reset();
    Site->bTrafficIndexReady=false;
    B->ResetVehicle(FVector(NorthX,19000,0));
    B->SetActorRotation(FRotator::ZeroRotator);
    Site->BeginTrafficFrame();
    Site->SetAGVIncidentDriveFailure(1,true,1.,0.);
    TestTrue(TEXT("A failed vehicle on the north spine leaves another return corridor available"),Site->PlanRoadRoute(0,false));
    TestTrue(TEXT("Return uses a different crossing under actual incident control"),Site->Jobs[0].ReturnCorridor!=2);
    FVector ClearFrom=A->GetActorLocation();
    for(const FVector Point:Site->Jobs[0].Route)
    {
        TestFalse(TEXT("Every planned chord clears the isolated chassis"),Site->IncidentRoadBlocked(ClearFrom,Point));
        ClearFrom=Point;
    }

    // Recorded V112/V123/V141 merge cycle, with V131 preventing another
    // reverse manoeuvre behind V112. All four poses come from the stalled run.
    auto* Recorded=World->SpawnActor<APortSiteLogistics>();
    Recorded->LaneCount=9;
    Recorded->bDepotParking=Recorded->bReuseFreedApron=true;
    Recorded->TrafficClock=3000;
    Recorded->Jobs.SetNum(4);
    Recorded->Yard.SetNum(4);
    Recorded->RMGBusy.Init(true,4);
    Recorded->YardApproachOwners.Init(INDEX_NONE,4);
    Recorded->STSOwners.Init(INDEX_NONE,9);
    const FVector RecordedPositions[]={
        FVector(13888.01541,25802.06964,0.00000),
        FVector(16338.05487,29820.04230,0.00000),
        FVector(16336.90881,26320.04814,0.00000),
        FVector(13887.97942,24306.84449,0.00000)
    };
    const double RecordedYaw[]={179.97276359,89.97920087,-90.01581030,-179.98405740};
    const TArray<TArray<FVector>> RecordedRoutes={
        {
            FVector(13887.98018,32769.99933,0.00000),
            FVector(13843.83029,32991.95571,0.00000),
            FVector(13718.10211,33180.12123,0.00000),
            FVector(13529.93649,33305.84941,0.00000),
            FVector(13307.98028,33349.99930,0.00000),
            FVector(11926.38938,33349.99925,0.00000),
            FVector(11704.43298,33394.14914,0.00000),
            FVector(11516.26740,33519.87732,0.00000),
            FVector(11390.53918,33708.04295,0.00000),
            FVector(11346.38943,33929.99929,0.00000),
            FVector(11346.38942,36909.99921,0.00000),
            FVector(11346.38941,38689.99915,0.00000),
            FVector(11346.38939,42869.99904,0.00000),
            FVector(11302.23951,43091.95544,0.00000),
            FVector(11176.51132,43280.12096,0.00000),
            FVector(10988.34570,43405.84914,0.00000),
            FVector(10766.38936,43449.99899,0.00000),
            FVector(8204.77423,43449.99903,0.00000),
            FVector(7982.81783,43405.84914,0.00000)
        },
        {
            FVector(14467.99034,29820.00951,0.00000),
            FVector(14288.75795,29848.39426,0.00000),
            FVector(14127.06961,29930.77615,0.00000),
            FVector(13998.75270,30059.09084,0.00000),
            FVector(13916.36794,30220.77791,0.00000),
            FVector(13887.98021,30400.00968,0.00000),
            FVector(13887.98016,32769.99927,0.00000),
            FVector(13843.83027,32991.95567,0.00000),
            FVector(13718.10209,33180.12118,0.00000),
            FVector(13529.93647,33305.84937,0.00000),
            FVector(13307.98012,33349.99922,0.00000),
            FVector(11926.38938,33349.99925,0.00000),
            FVector(11704.43298,33394.14914,0.00000),
            FVector(11516.26740,33519.87732,0.00000),
            FVector(11390.53918,33708.04295,0.00000),
            FVector(11346.38943,33929.99929,0.00000),
            FVector(11346.38942,36909.99921,0.00000),
            FVector(11346.38941,39889.99913,0.00000),
            FVector(11346.38939,42869.99904,0.00000)
        },
        {
            FVector(14467.99168,26320.01095,0.00000),
            FVector(14288.75898,26348.39536,0.00000),
            FVector(14127.07027,26430.77701,0.00000),
            FVector(13998.75299,26559.09184,0.00000),
            FVector(13916.36809,26720.77895,0.00000),
            FVector(13887.98019,26900.01099,0.00000),
            FVector(13887.98018,28335.00516,0.00000),
            FVector(13887.98016,32769.99927,0.00000),
            FVector(13843.83027,32991.95567,0.00000),
            FVector(13718.10209,33180.12118,0.00000),
            FVector(13529.93647,33305.84937,0.00000),
            FVector(13307.98012,33349.99922,0.00000),
            FVector(11926.38938,33349.99925,0.00000),
            FVector(11704.43298,33394.14914,0.00000),
            FVector(11516.26740,33519.87732,0.00000),
            FVector(11390.53918,33708.04295,0.00000),
            FVector(11346.38943,33929.99929,0.00000),
            FVector(11346.38943,36697.49922,0.00000),
            FVector(11346.38942,39464.99916,0.00000)
        },
        {
            FVector(13887.97970,27727.89607,0.00000),
            FVector(13887.98011,32769.99927,0.00000),
            FVector(13843.83022,32991.95566,0.00000),
            FVector(13718.10204,33180.12123,0.00000),
            FVector(13529.93654,33305.84942,0.00000),
            FVector(13307.98021,33349.99930,0.00000),
            FVector(11926.38938,33349.99925,0.00000),
            FVector(11704.43298,33394.14914,0.00000),
            FVector(11516.26740,33519.87732,0.00000),
            FVector(11390.53918,33708.04295,0.00000),
            FVector(11346.38943,33929.99929,0.00000),
            FVector(11346.38942,36519.99923,0.00000),
            FVector(11346.38942,39109.99916,0.00000),
            FVector(11346.38941,41699.99909,0.00000),
            FVector(11346.38941,44289.99903,0.00000),
            FVector(11346.38940,48179.99893,0.00000),
            FVector(11346.38939,49469.99889,0.00000),
            FVector(11302.23951,49691.95529,0.00000),
            FVector(11176.51132,49880.12081,0.00000)
        },
    };
    for(int32 I=0;I<4;++I)
    {
        auto* Vehicle=World->SpawnActor<APortAGVActor>();
        Vehicle->InitializeVehicle(100+I);
        Vehicle->ResetVehicle(RecordedPositions[I]);
        Vehicle->SetActorRotation(FRotator(0,RecordedYaw[I],0));
        Vehicle->SetFMSHold(true);
        Recorded->Vehicles.Add(Vehicle);
        Recorded->TrafficVehicles.Add(Vehicle);
        auto& Job=Recorded->Jobs[I];
        Job.Stage=5;
        Job.Slot=Job.RMG=I;
        Job.Route=RecordedRoutes[I];
        Job.RouteStart=RecordedPositions[I];
        Job.bRouteStartValid=true;
        auto& YardSlot=Recorded->Yard[I];
        YardSlot.Crane=I;
        YardSlot.Handover=RecordedPositions[I];
    }
    Recorded->RoadBlockers={{100,101},{101,102},{102,100},{103,100}};
    Recorded->RoadWaitSince={{100,950},{101,800},{102,790},{103,900}};
    for(int32 I:{1,2})
    {
        const auto& Job=Recorded->Jobs[I];
        const FVector Position=Recorded->Vehicles[I]->GetActorLocation();
        const FBox Limit(Position-FVector(6000,6000,500),Position+FVector(6000,6000,500));
        const auto Corridor=Recorded->BuildRoadCorridor(*Recorded->Vehicles[I],Job.Route[0],
            &Job.Route,0,Job.Route.Num(),false);
        TArray<FBox> Pending;
        for(const auto& Segment:Corridor)
            if(Segment.Intersect(Limit)) Pending.Add(Segment.Overlap(Limit));
        Recorded->PendingRoadRequests.Add(100+I,Pending);
    }
    bool AllAdvanced=false;
    for(int32 Step=0;Step<1800 && !AllAdvanced;++Step)
    {
        Recorded->TrafficClock+=.05;
        Recorded->BeginTrafficFrame();
        AllAdvanced=true;
        for(int32 I=0;I<3;++I)
        {
            if(Recorded->Jobs[I].Route.IsValidIndex(Recorded->Jobs[I].Waypoint))
                Recorded->Drive(I,.05f);
            AllAdvanced&=FVector::Dist2D(RecordedPositions[I],
                Recorded->Vehicles[I]->GetActorLocation())>1400.;
        }
    }
    TestTrue(TEXT("Recorded merge cycle drains using ordinary road reservations"),AllAdvanced);
    TestTrue(TEXT("The through vehicle clears the merge without reversing into its follower"),
        Recorded->Vehicles[0]->GetActorLocation().Y>RecordedPositions[0].Y+1400.);
    TestTrue(TEXT("The close follower keeps its physical position during merge recovery"),
        Recorded->Vehicles[3]->GetActorLocation().Equals(RecordedPositions[3],.01));

    World->DestroyWorld(false);
    return true;
}
#endif
