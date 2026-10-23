#if WITH_DEV_AUTOMATION_TESTS
#include "AGVApronRoads.h"
#include "PortSiteLogistics.h"
#include "PortAGVActor.h"
#include "Engine/World.h"
#include "Misc/AutomationTest.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAGVApronRoadTest,"PortSim.Traffic.ReusedParkingApron",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FAGVApronRoadTest::RunTest(const FString& Parameters)
{
    using namespace TerminalLayout;
    auto Clear=[](FVector,FVector){ return 0.; };
    TestTrue(TEXT("Both central road lanes include the whole chassis inside the rendered pavement"),
        FMath::Abs(AGVCentralInboundY-CentralRoadY*100)+210<CentralRoadWidthM*50 &&
        FMath::Abs(AGVCentralReturnY-CentralRoadY*100)+210<CentralRoadWidthM*50);
    TestTrue(TEXT("Opposing central road reservations do not overlap"),AGVCentralInboundY-AGVCentralReturnY>2*(210+170));
    TestTrue(TEXT("Opposing loaded lanes have full reservation clearance"),
        SiteCmX(AGVApronSouthX-AGVApronNorthX)>2*(210.f+170.f));
    TestTrue(TEXT("Empty return spine stays separate from loaded traffic"),
        SiteCmX(12500.f-AGVApronSouthX)>2*(210.f+170.f));
    int Northbound=0,Southbound=0;
    const float RecordedDeparture=AGVApronRoads::DepartureY(-15200.f,AGVCentralInboundY);
    TestTrue(TEXT("Recorded central intersection leaves chassis and turning clearance"),
        FMath::Abs(RecordedDeparture-AGVCentralReturnY)>=1800.f);
    for(int STS=0;STS<9;++STS) for(int Block=0;Block<YardBlockCount;++Block)
    {
        const FVector Quay=AGVApronRoads::P(AGVQuayHandoverX,(STSCenterY(STS)+8.f)*100.f);
        const float TargetY=RMGHandoverY(Block)*100.f;
        FAGVRoadNetwork Network; AGVApronRoads::Add(Network);
        const float Spine=AGVApronRoads::AddDeparture(Network,Quay,TargetY);
        const FVector Via=AGVApronRoads::P(Spine,TargetY);
        const FVector Entry(RMGHandoverX(0)*100.f-1800.f,TargetY,0);
        Network.Add(Via,Entry);
        TArray<FVector> To,From;
        TestTrue(TEXT("Every STS reaches every near-yard aisle through the freed area"),Network.Find(Quay,Via,To,Clear));
        TestTrue(TEXT("Apron connects directly to legal near-yard entry"),Network.Find(Via,Entry,From,Clear));
        const float Depart=AGVApronRoads::DepartureY(Quay.Y,TargetY);
        TestTrue(TEXT("Consecutive corner tangents do not overlap"),
            FMath::Abs(TargetY-Depart)>=2*AGVReference::MinimumInnerTurnRadiusCm+100.f);
        TestTrue(TEXT("Departure remains clear of the south boundary"),Depart>=AGVSouthLoadedY);
        if(Spine==AGVApronNorthX) ++Northbound; else ++Southbound;
        double Length=0; FVector Previous=Quay;
        for(const FVector& Point:To)
        { Length+=FVector::Dist2D(Previous,Point); Previous=Point; }
        TestTrue(TEXT("Loaded apron approach has no terminal-wide detour"),
            FMath::IsNearlyEqual(Length,FMath::Abs(Quay.Y-Depart)+FMath::Abs(TargetY-Depart)+FMath::Abs(Via.X-Quay.X),.1));
    }
    TestTrue(TEXT("Both directions of the converted area are used"),Northbound>0 && Southbound>0);
    FAGVRoadNetwork Directed; AGVApronRoads::Add(Directed);
    TArray<FVector> Route;
    TestFalse(TEXT("No southbound shortcut on the northbound apron lane"),Directed.Find(
        AGVApronRoads::P(AGVApronNorthX,10000),AGVApronRoads::P(AGVApronNorthX,-10000),Route,Clear));
    for(float Y:{AGVSouthLoadedY,AGVCentralInboundY,AGVNorthInboundY})
        TestTrue(TEXT("All three cross-terminal roads have direct apron access"),Directed.Find(
            AGVApronRoads::P(AGVApronSouthX,Y),AGVApronRoads::P(AGVInlandReturnX,Y),Route,Clear));
    TestTrue(TEXT("Central corridor traverses the full gap between both yard regions"),Directed.Find(
        AGVApronRoads::P(AGVApronNorthX,AGVCentralInboundY),AGVApronRoads::P(AGVInlandInboundX,AGVCentralInboundY),Route,Clear));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FApronManifestRoutesTest,"PortSim.Traffic.ApronManifestRoutes",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FApronManifestRoutesTest::RunTest(const FString& Parameters)
{
    using namespace TerminalLayout;
    const auto Init=UWorld::InitializationValues().AllowAudioPlayback(false)
        .CreatePhysicsScene(true).RequiresHitProxies(false).CreateNavigation(false).CreateAISystem(false);
    UWorld* World=UWorld::CreateWorld(EWorldType::Game,false,NAME_None,nullptr,true,ERHIFeatureLevel::Num,&Init);
    if(!TestNotNull(TEXT("Authored-manifest routing world"),World)) return false;
    auto* Site=World->SpawnActor<APortSiteLogistics>();
    auto* Vehicle=World->SpawnActor<APortAGVActor>(); Vehicle->InitializeVehicle(100);
    Site->LaneCount=9; Site->bReuseFreedApron=true; Site->bDepotParking=true;
    Site->bCargoAlignedHandover=true;
    Site->Vehicles={Vehicle}; Site->TrafficVehicles={Vehicle};
    Site->Jobs.SetNum(1); Site->Manifest.SetNum(1); Site->Yard.SetNum(1);
    for(int Ship=0;Ship<3;++Ship) for(int Bay=0;Bay<10;++Bay) for(int Half=0;Half<2;++Half)
    {
        const float Y=(VesselCenterY(Ship)-81.f+Bay*18.f)*100.f;
        const int STS=Ship*3+FMath::Clamp(FMath::RoundToInt((Y/100.f-(VesselCenterY(Ship)-70.f))/70.f),0,2);
        auto& Cargo=Site->Manifest[0]; Cargo.STS=STS; Cargo.Transform=FTransform(FVector(-4900,Y,400));
        auto& Slot=Site->Yard[0]; Slot.Block=Half?19:8; Slot.Half=Half; Slot.Occupied=false;
        Slot.Handover=FVector(RMGHandoverX(Half)*100.f,RMGHandoverY(Slot.Block)*100.f,0);
        Site->Jobs[0]=FSiteTransfer(); auto& Job=Site->Jobs[0];
        Job.Stage=3; Job.STS=STS; Job.Cargo=Job.Slot=0;
        const FVector Start=AGVApronRoads::P(AGVQuayHandoverX,Y);
        Vehicle->ResetVehicle(Start);
        const bool Found=Site->PlanRoadRoute(0,false);
        TestTrue(TEXT("Every authored source bay has a complete operational near/far route"),Found);
        if(!Found || Job.Route.IsEmpty()) continue;
        // Bay Y=-151m lies exactly on the central road. It must still leave
        // longitudinally before turning; an east-only graph shortcut has no
        // initial steering tangent and makes a car turn outside its route.
        TestTrue(FString::Printf(TEXT("STS departure tangent ship=%d bay=%d half=%d start=%s first=%s"),
            Ship,Bay,Half,*Start.ToCompactString(),*Job.Route[0].ToCompactString()),
            FMath::Abs(Job.Route[0].X-Start.X)<1.f && FMath::Abs(Job.Route[0].Y-Start.Y)>100.f);
        TestTrue(TEXT("Operational route ends at the actual RMG handover"),Job.Route.Last().Equals(Slot.Handover,.1f));
    }
    World->DestroyWorld(false);
    return true;
}
#endif
