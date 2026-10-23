#if WITH_DEV_AUTOMATION_TESTS
#include "PortSiteLogistics.h"
#include "PortWorkingCrane.h"
#include "PortContainerActor.h"
#include "PortAGVActor.h"
#include "TerminalLayout.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/World.h"
#include "Misc/AutomationTest.h"
#include "Misc/Paths.h"
#include "Dom/JsonObject.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FEquipmentRecoveryTest,"PortSim.Recovery.EquipmentFailures",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FEquipmentRecoveryTest::RunTest(const FString& Parameters)
{
    const auto Init=UWorld::InitializationValues().AllowAudioPlayback(false).CreatePhysicsScene(true)
        .RequiresHitProxies(false).CreateNavigation(false).CreateAISystem(false);
    auto* World=UWorld::CreateWorld(EWorldType::Game,false,NAME_None,nullptr,true,ERHIFeatureLevel::Num,&Init);
    if(!TestNotNull(TEXT("Recovery world"),World)) return false;
    auto* Site=World->SpawnActor<APortSiteLogistics>(); Site->LaneCount=9; Site->YardCraneCount=2;
    FSTSOperatingProfile Profile;
    if(!TestTrue(TEXT("STS recovery reference loads"),Profile.Load(FPaths::ProjectDir()/TEXT("../Document/STS/STS_ReferenceData.json"),
        FPaths::ProjectConfigDir()/TEXT("STS_Simulation.json")))) { World->DestroyWorld(false); return false; }
    for(int32 I=0;I<2;++I)
    {
        auto* R=World->SpawnActor<APortWorkingCrane>(FVector(TerminalLayout::RMGHomeX(0)*100,TerminalLayout::BlockY(I)*100,0),FRotator(0,90,0));
        R->Configure(I+1,false,R->GetActorLocation()+FVector(0,0,149.5),R->GetActorLocation()+FVector(100,0,149.5),false);
        Site->Equipment.Add(R);
    }
    for(int32 S=0;S<9;++S)
    {
        const FVector Home(1200,S*7000,0);
        auto* C=World->SpawnActor<APortWorkingCrane>(Home,FRotator::ZeroRotator);
        C->SetSTSProfile(Profile); C->Configure(S+3,true,Home+FVector(-2800,0,569.5),Home+FVector(2800,0,349.5),false);
        Site->Equipment.Add(C);
    }
    Site->RMGBusy.Init(false,2); Site->YardApproachOwners.Init(INDEX_NONE,2);
    Site->BlocksBusy.Init(false,TerminalLayout::YardBlockCount);
    Site->STSOwners.Init(INDEX_NONE,9); Site->NextVehicles.Init(INDEX_NONE,9);
    Site->PreparedCargo.Init(INDEX_NONE,9); Site->PreparedStarted.Init(false,9);
    auto* V=World->SpawnActor<APortAGVActor>(); V->InitializeVehicle(100); Site->Vehicles.Add(V); Site->TrafficVehicles.Add(V); Site->Jobs.AddDefaulted();
    auto Cargo=[&](int32 Number,float Mass,FVector Position)
    {
        FActorSpawnParameters Spawn; Spawn.Owner=Site; Spawn.SpawnCollisionHandlingOverride=ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
        auto* C=World->SpawnActor<APortContainerActor>(Position,FRotator::ZeroRotator,Spawn);
        C->InitializeContainer(Number); C->ConfigureSpecification(FPortContainerSpecification()); C->SetPhysicalParameters(Mass,FVector::ZeroVector);
        C->CargoInformation=FPortCargoInformation(); C->CargoInformation.OutboundOrder=1; C->GetBody()->SetSimulatePhysics(false);
        FSiteShipCargo M; M.Actor=C; M.ID=Number; M.ContainerName=C->ContainerID; M.STS=M.OriginalSTS=1;
        M.PlanningOrder=Site->Manifest.Num(); M.Transform=FTransform(Position); M.PlannedSlot=M.PlanningOrder;
        Site->Manifest.Add(M); Site->ShipContainers.Add(C); return C;
    };
    auto* Box=Cargo(9001,24000,FVector(-1600,4000,569.5));
    auto* Upper=Cargo(9002,12000,FVector(-1600,4000,828.5));
    for(int32 I=0;I<4;++I)
    {
        FSiteYardSlot S; S.Crane=I/2; S.Block=I/2; S.LengthFt=40; S.Reserved=true; S.Occupied=false;
        S.Position=FVector(TerminalLayout::YardBayX(0)*100,TerminalLayout::BlockY(S.Block)*100,149.5+(I%2)*259);
        S.Handover=FVector(TerminalLayout::RMGHandoverX(0)*100,TerminalLayout::RMGHandoverY(S.Block)*100,0);
        S.Below=I%2?I-1:INDEX_NONE; Site->Yard.Add(S);
    }
    Site->SlotAssigned.Init(false,4); Site->BuildSourceAccess();
    // Real fixed-pad coordinates exposed a defect hidden by cargo-aligned
    // tests: the candidate must not use the failed STS's pad as its destination.
    {
        auto* Fixed=World->SpawnActor<APortSiteLogistics>(); Fixed->LaneCount=9; Fixed->YardCraneCount=0;
        Fixed->bCargoAlignedHandover=false; Fixed->bRecoveryRailGuard=true;
        for(int32 S=0;S<9;++S)
        {
            const FVector Home(1200,TerminalLayout::STSCenterY(S)*100.,0);
            auto* Crane=World->SpawnActor<APortWorkingCrane>(Home,FRotator::ZeroRotator);
            Crane->SetSTSProfile(Profile); Crane->Configure(S+1,true,Home+FVector(-2800,0,569.5),Home+FVector(2800,0,349.5),false);
            Fixed->Equipment.Add(Crane);
        }
        auto Copy=Site->Manifest[0]; const FVector Pose=Box->GetActorLocation();
        const double FailedY=TerminalLayout::STSCenterY(1)*100.;
        Copy.Transform.SetLocation(FVector(-1600,FailedY-3000,569.5)); Fixed->Manifest.Add(Copy);
        Box->SetActorLocation(Copy.Transform.GetLocation()); Fixed->Equipment[1]->SetSensorFailure(true);
        const FVector FailedPad=Fixed->QuayPark(1)+FVector(0,0,349.5);
        TestFalse(TEXT("Failed fixed pad cannot be used through the stopped crane"),Fixed->CanSTSService(0,0,&FailedPad));
        TestTrue(TEXT("South adjacent STS can serve reachable cargo at its own fixed pad"),Fixed->CanSTSService(0,0));
        Fixed->Manifest[0].Transform.SetLocation(FVector(-1600,FailedY+3000,569.5)); Box->SetActorLocation(Fixed->Manifest[0].Transform.GetLocation());
        TestTrue(TEXT("North adjacent STS can serve reachable cargo at its own fixed pad"),Fixed->CanSTSService(2,0));
        Fixed->Manifest[0].Transform.SetLocation(FVector(-1600,FailedY,569.5)); Box->SetActorLocation(Fixed->Manifest[0].Transform.GetLocation());
        TestFalse(TEXT("Changing pads never permits crossing a physically stranded crane"),Fixed->CanSTSService(0,0));
        Box->SetActorLocation(Pose);
        for(const auto& TempCrane:Fixed->Equipment) TempCrane->Destroy();
        Fixed->Destroy();
    }
    auto* Failed=Site->Equipment[3].Get(); const FVector Before=Box->GetActorLocation();
    Failed->SetSensorFailure(true); Site->SimulationTime=2; Site->ManageEquipmentFailures();
    TestTrue(TEXT("Sensor fault is local, unrelated terminal work remains available"),Site->Fault.IsEmpty() && Failed->IsSensorUnavailable() && Site->CraneAvailable(2));
    TestEqual(TEXT("Reachable work moves to the adjacent STS"),Site->Manifest[0].STS,0);
    TestTrue(TEXT("Planning does not move the container"),Box->GetActorLocation().Equals(Before));
    Site->Manifest[0].Transform.SetLocation(FVector(-1600,7000,569.5)); Box->SetActorLocation(Site->Manifest[0].Transform.GetLocation());
    Site->SimulationTime=4; Site->ManageEquipmentFailures();
    TestEqual(TEXT("Failed crane cannot be crossed to reach its exact occupied bay"),Site->Manifest[0].RecoveryHold,FString(TEXT("sts_no_reachable_alternative")));
    TestFalse(TEXT("Rail movement cannot pass the stopped crane"),Site->CanSTSGantryMove(Site->Equipment[2],7000));
    Failed->RepairSensors(); Site->SimulationTime=6; Site->ManageEquipmentFailures();
    TestEqual(TEXT("Repair restores unstarted work to its original service zone"),Site->Manifest[0].STS,1);

    Site->PreparedCargo[1]=0; Site->Manifest[0].State=1; Site->NextVehicles[1]=0;
    Site->Jobs[0].Stage=5; Site->Jobs[0].STS=1; Site->Jobs[0].ReturnSTS=1; Site->Jobs[0].ReturnCargo=0;
    Failed->SetSensorFailure(true); Site->SimulationTime=6.5; Site->ManageEquipmentFailures();
    TestTrue(TEXT("Fault preserves the cargo placeholder of an AGV already returning to its queue"),
        Site->PreparedCargo[1]==0 && Site->Manifest[0].State==1 && Site->Jobs[0].ReturnCargo==0);
    Failed->RepairSensors(); Site->SimulationTime=7; Site->ManageEquipmentFailures();
    Site->PreparedCargo[1]=INDEX_NONE; Site->NextVehicles[1]=INDEX_NONE; Site->Jobs[0]=FSiteTransfer(); Site->Manifest[0].State=0;

    Site->Manifest[0].Transform.SetLocation(Before); Box->SetActorLocation(Before); Site->Manifest[0].State=1;
    Site->Jobs[0].Stage=6; Site->Jobs[0].STS=1; Site->Jobs[0].Cargo=0; Site->Jobs[0].Actor=Box; Site->Jobs[0].Slot=0; Site->Jobs[0].TripID=17;
    Site->STSOwners[1]=0;
    Site->YardApproachOwners[0]=0; Site->SlotAssigned[0]=true; Site->Jobs[0].RMG=0;
    const FVector AssignedPad(2800,4000,349.5);
    TestTrue(TEXT("An actual unlifted pickup can be assigned"),Failed->AssignCargo(Box,Before,AssignedPad,false,false,V));
    Failed->SetSensorFailure(true); Site->SimulationTime=8; Site->ManageEquipmentFailures();
    TestTrue(TEXT("Active supported pickup retains its AGV and trip identity"),Site->Jobs[0].STS==0 && Site->Jobs[0].TripID==17 && Site->Jobs[0].Actor.Get()==Box && Site->STSOwners[0]==0);
    TestTrue(TEXT("Old STS releases the job without lifting or teleporting its cargo"),!Failed->IsBusy() && Box->LocationOwner==ECargoOwner::Ship && Box->GetActorLocation().Equals(Before));
    TestTrue(TEXT("Approaching AGV keeps its assigned handover, not its current position"),Site->Equipment[2]->AssignedDestination().Equals(AssignedPad));
    TestTrue(TEXT("Fault releases the distant approach without losing the cargo slot"),Site->YardApproachOwners[0]==INDEX_NONE && Site->SlotAssigned[0] && Site->Jobs[0].bYardAdmissionSuspended);
    Site->YardApproachOwners[0]=1; Site->SimulationTime=8.5; Site->ManageEquipmentFailures();
    TestTrue(TEXT("Recovered pickup cannot steal an approach now owned by another AGV"),Site->Jobs[0].bYardAdmissionSuspended && Site->YardApproachOwners[0]==1);
    Site->YardApproachOwners[0]=INDEX_NONE; Site->SimulationTime=8.75; Site->ManageEquipmentFailures();
    TestTrue(TEXT("Recovered pickup reacquires its own booked slot and resumes"),!Site->Jobs[0].bYardAdmissionSuspended && Site->YardApproachOwners[0]==0);
    Site->Equipment[2]->CancelUnliftedJob(); Site->STSOwners[0]=INDEX_NONE; Site->STSOwners[1]=0;
    Site->Jobs[0].STS=1; Site->Jobs[0].Stage=1; Site->Manifest[0].STS=1; Failed->RepairSensors();
    Failed->AssignCargo(Box,Before,V->CargoPosition(),false,false,V);
    Failed->bCarrying=true; for(bool& Lock:Failed->CornerLocked) Lock=true;
    Box->AttachToComponent(Failed->Spreader,FAttachmentTransformRules::KeepWorldTransform); Box->LocationOwner=ECargoOwner::STS;
    const FTransform Held=Box->GetActorTransform(); Failed->SetSensorFailure(true); Site->SimulationTime=10; Site->ManageEquipmentFailures();
    TestTrue(TEXT("Suspended cargo stays attached, and only its job is held"),Site->Fault.IsEmpty() && Site->JobHeldByFailure(0) && Box->GetAttachParentActor()==Failed && Box->GetActorTransform().Equals(Held));
    Site->Jobs[0].Stage=3;
    TestFalse(TEXT("Previous loaded trips from the same STS are not held by its later fault"),Site->JobHeldByFailure(0));
    Site->Jobs[0].Stage=6;
    TestFalse(TEXT("An approaching AGV can clear the road and reach its reserved pad"),Site->JobHeldByFailure(0));
    Site->Jobs[0].Stage=1;
    TestFalse(TEXT("Retained load cannot be cancelled for reassignment"),Failed->CancelUnliftedJob());
    Failed->RepairSensors(); TestTrue(TEXT("Repair preserves locks and load for controller resumption"),Failed->RetainsLoad() && Box->GetAttachParentActor()==Failed);
    Box->DetachFromActor(FDetachmentTransformRules::KeepWorldTransform); Box->LocationOwner=ECargoOwner::Ship; Failed->ResetOperation();
    Site->Jobs[0]=FSiteTransfer(); Site->STSOwners.Init(INDEX_NONE,9); Site->Manifest[0].State=0;
    Site->YardApproachOwners.Init(INDEX_NONE,2); Site->SlotAssigned.Init(false,4);
    Site->Manifest[0].PlannedSlot=0; Site->Manifest[1].PlannedSlot=1;
    Site->Manifest[0].Transform.SetLocation(FVector(-1600,7000,569.5)); Box->SetActorLocation(Site->Manifest[0].Transform.GetLocation());
    Site->Manifest[1].STS=Site->Manifest[1].OriginalSTS=0; Site->BuildSourceAccess();
    Failed->SetSensorFailure(true);
    auto ExtraTier=Site->Yard[1]; ExtraTier.Below=1; ExtraTier.Position.Z+=259; Site->Yard.Add(ExtraTier);
    auto ExtraGround=Site->Yard[2]; ExtraGround.Position.X+=1800; Site->Yard.Add(ExtraGround);
    auto* FailedUpper=Cargo(9003,6000,FVector(-1600,7000,1087.5)); Site->Manifest[2].PlannedSlot=4;
    Site->SlotAssigned.Init(false,6); Site->BuildSourceAccess();
    const FTransform SupportedBefore=Upper->GetActorTransform();
    TestTrue(TEXT("Healthy STS work can escape a yard support reserved for a failed STS"),Site->ReplanBlockedYards());
    TestTrue(TEXT("Only the dependent healthy booking changes to a supported ground slot"),Site->Manifest[0].PlannedSlot==0 && Site->Manifest[1].PlannedSlot==2 && Site->Yard[2].Below==INDEX_NONE);
    TestTrue(TEXT("Dependency recovery changes bookings, never physical container poses"),Upper->GetActorTransform().Equals(SupportedBefore));
    TSet<int32> PlannedSupports;
    for(const auto& C:Site->Manifest) PlannedSupports.Add(C.PlannedSlot);
    bool CompleteSupports=true;
    for(const auto& C:Site->Manifest) if(Site->Yard[C.PlannedSlot].Below!=INDEX_NONE && !PlannedSupports.Contains(Site->Yard[C.PlannedSlot].Below)) CompleteSupports=false;
    TestTrue(TEXT("Unavailable upper bookings follow a relocated healthy support without becoming orphaned"),CompleteSupports && Site->Manifest[2].PlannedSlot!=4);
    Site->Manifest.Pop(); Site->ShipContainers.Pop(); FailedUpper->Destroy(); Site->Yard.SetNum(4); Site->SlotAssigned.Init(false,4);
    Failed->RepairSensors(); Site->Manifest[0].Transform.SetLocation(Before); Box->SetActorLocation(Before);
    Site->Manifest[1].STS=Site->Manifest[1].OriginalSTS=1; Site->BuildSourceAccess();
    Site->Manifest[0].PlannedSlot=0; Site->Manifest[1].PlannedSlot=1;
    Site->Equipment[0]->SetSensorFailure(true); Site->Equipment[1]->SetSensorFailure(true);
    TestFalse(TEXT("No healthy compatible yard leaves a pending plan"),Site->ReplanFailedYards(0));
    TestTrue(TEXT("Failed planning rolls back both lower and dependent upper destinations"),Site->Manifest[0].PlannedSlot==0 && Site->Manifest[1].PlannedSlot==1);
    Site->Equipment[1]->RepairSensors();
    TestTrue(TEXT("Whole affected stack can be replanned to the other RMG"),Site->ReplanFailedYards(0));
    TestTrue(TEXT("Dependent upper follows its lower support, preserving order and mass"),Site->Manifest[0].PlannedSlot==2 && Site->Manifest[1].PlannedSlot==3 && Site->Yard[Site->Manifest[1].PlannedSlot].Below==Site->Manifest[0].PlannedSlot);
    Site->Manifest[0].PlannedSlot=0; Site->Manifest[1].PlannedSlot=1; Site->Manifest[0].State=1;
    auto& J=Site->Jobs[0]; J.Stage=3; J.Cargo=0; J.Actor=Box; J.STS=1; J.Slot=0; J.RMG=0;
    V->SetActorLocation(Site->Yard[0].Handover+FVector(-1800,0,0)); V->Speed=100;
    Box->AttachToComponent(V->GetRootComponent(),FAttachmentTransformRules::KeepWorldTransform); Box->LocationOwner=ECargoOwner::AGV;
    TestFalse(TEXT("Moving loaded vehicle does not have its route replaced mid-segment"),Site->ReplanFailedYards(0));
    V->Speed=0; J.Stage=9; J.Route={V->GetActorLocation()}; J.Waypoint=1; J.YardEntryWaypoint=0;
    Site->YardApproachOwners[0]=0; Site->SlotAssigned[0]=true;
    const FTransform VehiclePose=V->GetActorTransform();
    TestTrue(TEXT("Stopped loaded AGV receives a feasible new destination"),Site->ReplanFailedYards(0));
    TestTrue(TEXT("Old approach lease is retained while new approach is reserved"),!Site->DeferredYardLeases.IsEmpty() && Site->YardApproachOwners[0]==0 && Site->YardApproachOwners[1]==0);
    J.Stage=3; TestTrue(TEXT("Actual road graph connects the old yard exit to the new entry"),Site->PlanRoadRoute(0,false));
    TestTrue(TEXT("Route replacement does not move or detach vehicle cargo"),V->GetActorTransform().Equals(VehiclePose) && Box->GetAttachParentActor()==V);
    Box->DetachFromActor(FDetachmentTransformRules::KeepWorldTransform); Box->LocationOwner=ECargoOwner::Ship;
    Site->Jobs[0]=FSiteTransfer(); Site->DeferredYardLeases.Reset(); Site->YardApproachOwners.Init(INDEX_NONE,2); Site->Manifest.Reset();
    Site->Yard.Reset(); Site->SlotAssigned.Init(false,8);
    for(int32 I=0;I<8;++I)
    {
        FSiteYardSlot S; S.Crane=I/4; S.Block=I/4; S.LengthFt=20; S.Reserved=true; S.Occupied=false;
        S.Partner=I^1; S.Below=I%4>=2?I-2:INDEX_NONE;
        S.Position=FVector(TerminalLayout::YardBayX(0)*100+(I%2?306.7:-306.7),TerminalLayout::BlockY(S.Block)*100,149.5+(I%4/2)*259);
        S.Handover=FVector(TerminalLayout::RMGHandoverX(0)*100,TerminalLayout::RMGHandoverY(S.Block)*100,0); Site->Yard.Add(S);
    }
    for(int32 I=0;I<4;++I)
    {
        auto* C=Cargo(9010+I,I<2?24000:12000,FVector(-1600,I*2000,569.5));
        FPortContainerSpecification Short; Short.Size=EPortContainerSize::Ft20; Short.LengthFt=20; Short.LengthCm=605.8;
        C->ConfigureSpecification(Short); C->SetPhysicalParameters(I<2?24000:12000,FVector::ZeroVector);
    }
    TestTrue(TEXT("20ft stack can migrate as a complete two-pad dependency group"),Site->ReplanFailedYards(0));
    TestTrue(TEXT("Both lower pads precede each reassigned 20ft upper pad"),Site->Manifest[0].PlannedSlot==4 && Site->Manifest[1].PlannedSlot==5 && Site->Manifest[2].PlannedSlot==6 && Site->Manifest[3].PlannedSlot==7);
    Site->ReportBase=FPaths::ProjectSavedDir()/TEXT("Results/DashboardControlUnit");
    auto Command=MakeShared<FJsonObject>();
    Command->SetStringField(TEXT("run_id"),TEXT("DashboardControlUnit"));
    Command->SetStringField(TEXT("control_session"),Site->DashboardControlSession);
    Command->SetStringField(TEXT("created_at"),FDateTime::UtcNow().ToIso8601());
    Command->SetStringField(TEXT("actor_id"),V->GetName()); Command->SetStringField(TEXT("action"),TEXT("fail"));
    FString Error;
    Box->AttachToComponent(V->GetRootComponent(),FAttachmentTransformRules::KeepWorldTransform);
    const FTransform CargoPose=Box->GetActorTransform(),VehicleBefore=V->GetActorTransform(); V->SetPayload(12000);
    TestTrue(TEXT("Web command applies AGV equipment isolation"),Site->ApplyDashboardCommand(*Command,Error) && V->bDriveFault && Site->Fault.IsEmpty());
    TestFalse(TEXT("Failed AGV cannot accept a new dispatch"),V->CanAcceptWork());
    V->SetFMSHold(false);
    TestFalse(TEXT("A controller cannot accidentally drive a failed vehicle"),V->MoveToPosition(V->GetActorLocation()+FVector(0,1000,0),1));
    TestTrue(TEXT("Failed AGV preserves vehicle and loaded cargo poses"),V->GetActorTransform().Equals(VehicleBefore) && Box->GetActorTransform().Equals(CargoPose) && Box->GetAttachParentActor()==V);
    Command->SetStringField(TEXT("action"),TEXT("repair"));
    TestTrue(TEXT("Web repair resumes the same vehicle and attached load"),Site->ApplyDashboardCommand(*Command,Error) && !V->bDriveFault && Box->GetAttachParentActor()==V);
    Command->SetStringField(TEXT("control_session"),TEXT("previous-session"));
    TestFalse(TEXT("Old browser sessions cannot mutate a reset engine"),Site->ApplyDashboardCommand(*Command,Error));
    Command->SetStringField(TEXT("control_session"),Site->DashboardControlSession);
    Command->SetStringField(TEXT("created_at"),TEXT("2020-01-01T00:00:00Z"));
    TestFalse(TEXT("Expired queued commands cannot replay"),Site->ApplyDashboardCommand(*Command,Error));
    Command->SetStringField(TEXT("created_at"),FDateTime::UtcNow().ToIso8601());
    Command->SetStringField(TEXT("actor_id"),Failed->GetName());
    AddExpectedError(TEXT("Safety interlock"),EAutomationExpectedErrorFlags::Contains,1);
    Failed->Stop(TEXT("Safety interlock"));
    TestFalse(TEXT("Web sensor repair cannot clear a safety fault"),Site->ApplyDashboardCommand(*Command,Error));
    Failed->ResetOperation();
    Failed->bAutoSensorRecovery=true; Failed->bAllowSensorFaultEvacuation=false;
    Failed->SensorRepairMethod=EPortSensorRepairMethod::Reboot;
    Failed->SensorRebootSeconds=2; Failed->SensorSelfCheckSeconds=1;
    Failed->SetSensorFailure(true); Failed->Advance(20,true);
    TestEqual(TEXT("A global pause does not consume sensor repair time"),Failed->SensorRepairElapsed,0.);
    Failed->Advance(1,false); TestTrue(TEXT("Reboot delay is not an immediate reset"),Failed->IsSensorUnavailable());
    Failed->Advance(2.1f,false); TestFalse(TEXT("Reboot and self-check resume the same crane automatically"),Failed->IsEquipmentUnavailable());
    Failed->SensorRepairMethod=EPortSensorRepairMethod::Technician; Failed->SensorTechnicianRepairSeconds=4;
    Failed->SetSensorFailure(true); Failed->Advance(3,false);
    TestTrue(TEXT("Technician repair has its own duration"),Failed->IsSensorUnavailable());
    Failed->Advance(2.1f,false); TestFalse(TEXT("Technician repair completes after repair and self-check"),Failed->IsEquipmentUnavailable());
    Failed->SetGantryFailure(true); Failed->Advance(100,false);
    TestTrue(TEXT("Gantry failure is local and cannot be cleared by a sensor reboot"),Failed->IsEquipmentUnavailable() && !Failed->IsSensorUnavailable());
    TestFalse(TEXT("Broken gantry cannot evacuate"),Failed->CanFaultEvacuate());
    Failed->RepairEquipment();
    TestFalse(TEXT("Explicit equipment repair clears the mechanical fault"),Failed->IsEquipmentUnavailable());

    Failed->Head.Y=0; Failed->UpdateParts(); Site->bRecoveryRailGuard=true;
    Failed->bAutoSensorRecovery=false; Failed->bAllowSensorFaultEvacuation=true; Failed->SetSensorFailure(true);
    const double RailStart=Failed->GantryWorldY();
    TestTrue(TEXT("Independent safety allows empty retracted crane service travel"),Failed->RequestFaultEvacuation(RailStart+200,Site));
    Failed->Advance(1,false);
    TestTrue(TEXT("Service drive advances continuously at bounded speed, without teleporting"),Failed->GantryWorldY()>RailStart && Failed->GantryWorldY()<RailStart+30);
    const double BeforePause=Failed->GantryWorldY(); Failed->Advance(5,true);
    TestEqual(TEXT("Pausing freezes evacuation too"),double(Failed->GantryWorldY()),BeforePause);
    Failed->CornerLocked[0]=true;
    TestFalse(TEXT("A retained lock forbids evacuation even for an automation-camera fault"),Failed->CanFaultEvacuate());
    Failed->CornerLocked[0]=false;
    Failed->RepairSensors(); Failed->Advance(3,false);
    TestTrue(TEXT("Repair brakes the service drive before normal scheduling"),Failed->Fault.IsEmpty() && Failed->bFaultEvacuationComplete && !Site->CraneAvailable(3));
    Site->PlanFaultEvacuations();
    TestTrue(TEXT("Repaired displaced crane reserves a no-crossing return to its original rail anchor"),Failed->bFaultEvacuating && FMath::IsNearlyEqual(Failed->FaultEvacuationTargetY,Failed->GantryHomeWorldY(),1.f));
    Failed->Advance(5,false);
    TestTrue(TEXT("Only after physical return may the repaired crane accept work"),!Failed->bFaultEvacuating && !Failed->bFaultEvacuationComplete && Site->CraneAvailable(3));
    Failed->SetSensorFailure(true); Failed->bIndependentTravelSafetyHealthy=false;
    TestFalse(TEXT("Failed independent travel safety forbids evacuation"),Failed->RequestFaultEvacuation(RailStart+200,Site));
    Failed->RepairEquipment(); Failed->ResetOperation();
    World->DestroyWorld(false); return true;
}
#endif
