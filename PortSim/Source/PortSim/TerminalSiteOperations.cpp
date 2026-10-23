#include "QuayCrane.h"
#include "TerminalLayout.h"
#include "PortWorkingCrane.h"
#include "PortSiteLogistics.h"
#include "PortAGVActor.h"
#include "PortContainerActor.h"
#include "ContainerSpecification.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "GameFramework/SpringArmComponent.h"

void AQuayCrane::ResetSiteOperations()
{
    if (SiteLogistics) SiteLogistics->ResetLogistics();
}
void AQuayCrane::TickSiteOperations(float Dt)
{
    if (RMGActor) RMGActor->SetOperationPaused(bAutoPaused || bEmergencyStop || AutoStage==ETerminalStage::Fault);
    if (!SiteLogistics || (bTerminalTest && !FParse::Param(FCommandLine::Get(),TEXT("PortSimMixedTest")))) return;
    if (bTerminalTest) SiteLogistics->DispatchLimit=16;
    const bool Full=FParse::Param(FCommandLine::Get(),TEXT("PortSimFullUnloadTest"));
    const bool Continuous=FParse::Param(FCommandLine::Get(),TEXT("PortSimContinuousTrafficTest"));
    const bool Testing=Full || FParse::Param(FCommandLine::Get(),TEXT("PortSimSiteTest"));
    int32 TestShipments=SiteLogistics->Vehicles.Num()*2;
    FParse::Value(FCommandLine::Get(),TEXT("PortSimSiteShipmentLimit="),TestShipments);
    TestShipments=FMath::Clamp(TestShipments,9,SiteLogistics->InitialShipCount());
    if (Testing) SiteLogistics->DispatchLimit=(Full || Continuous)?SiteLogistics->InitialShipCount():TestShipments;
    const bool FirstWave=FParse::Param(FCommandLine::Get(),TEXT("PortSimFirstWaveTest"));
    const bool CarLike=FParse::Param(FCommandLine::Get(),TEXT("PortSimCarLikeTest"));
    int32 WaveShipmentCount=9;
    FParse::Value(FCommandLine::Get(),TEXT("PortSimWaveShipmentCount="),WaveShipmentCount);
    WaveShipmentCount=FMath::Clamp(WaveShipmentCount,1,SiteLogistics->InitialShipCount());
    if (FirstWave) SiteLogistics->DispatchLimit=WaveShipmentCount;
    else if (CarLike) SiteLogistics->DispatchLimit=1;
    SiteLogistics->Advance(Dt,bAutoPaused || bEmergencyStop || (bUnifiedTerminal && !bAutoRunning));
    if(FParse::Param(FCommandLine::Get(),TEXT("PortSimOperationsTest")))
    {
        SiteTestTime+=Dt;
        FString Error;
        if(!SiteLogistics->Validate(Error) || SiteTestTime>12000)
        {
            UE_LOG(LogTemp,Error,TEXT("PORTSIM_OPERATIONS_FAIL: %s"),*Error);
            FPlatformMisc::RequestExitWithStatus(false,1);
        }
        else if(SiteLogistics->Delivered==SiteLogistics->InitialShipCount() && SiteLogistics->IsIdle())
        {
            const int32 Loaded=SiteLogistics->LoadedCount();
            SiteLogistics->ExportDashboard(false,true);
            SiteLogistics->ResetLogistics();
            if(!SiteLogistics->Validate(Error) || SiteLogistics->LoadedCount()!=0)
            {
                UE_LOG(LogTemp,Error,TEXT("PORTSIM_OPERATIONS_FAIL: reset %s"),*Error);
                FPlatformMisc::RequestExitWithStatus(false,1);
            }
            else
            {
                UE_LOG(LogTemp,Display,TEXT("PORTSIM_OPERATIONS_PASS: discharged=%d loaded=%d reset_verified=1"),SiteLogistics->InitialShipCount(),Loaded);
                FPlatformMisc::RequestExitWithStatus(false,0);
            }
        }
        return;
    }
    if (FirstWave || CarLike)
    {
        SiteTestTime+=Dt;
        FString Error;
        if (!SiteLogistics->Validate(Error) || SiteTestTime>5000)
        {
            UE_LOG(LogTemp,Error,TEXT("%s_FAIL: %s"),CarLike?TEXT("PORTSIM_AGV_CARLIKE"):TEXT("PORTSIM_FIRST_WAVE"),
                Error.IsEmpty()?TEXT("simulation time limit exceeded"):*Error);
            FPlatformMisc::RequestExitWithStatus(false,1);
        }
        else if (CarLike && SiteLogistics->LoadedVehicle()!=nullptr)
        {
            SiteLogistics->ExportDashboard(false,true);
            UE_LOG(LogTemp,Display,TEXT("PORTSIM_AGV_CARLIKE_PASS: sts_to_agv=1 vehicle=%d lateral_motion=disabled simulation_seconds=%.6f step=%.6f"),
                SiteLogistics->LoadedVehicle()->VehicleID,SiteTestTime,Dt);
            FPlatformMisc::RequestExitWithStatus(false,0);
        }
        else if (SiteLogistics->Delivered==WaveShipmentCount)
        {
            if (!SiteLogistics->ReturnCorridorUse(1) || !SiteLogistics->ReturnCorridorUse(2))
            {
                UE_LOG(LogTemp,Error,TEXT("PORTSIM_FIRST_WAVE_FAIL: alternate AGV return corridors unused (south=%d central=%d north=%d)"),
                    SiteLogistics->ReturnCorridorUse(0),SiteLogistics->ReturnCorridorUse(1),SiteLogistics->ReturnCorridorUse(2));
                FPlatformMisc::RequestExitWithStatus(false,1);
                return;
            }
            SiteLogistics->ExportDashboard(false,true);
            UE_LOG(LogTemp,Display,TEXT("PORTSIM_FIRST_WAVE_PASS: policy=%s delivered=%d simulation_seconds=%.6f step=%.6f return_routes=%d/%d/%d"),
                SiteLogistics->UsesCargoAlignedHandover()?TEXT("cargo_aligned"):TEXT("fixed"),WaveShipmentCount,SiteTestTime,Dt,
                SiteLogistics->ReturnCorridorUse(0),SiteLogistics->ReturnCorridorUse(1),SiteLogistics->ReturnCorridorUse(2));
            FPlatformMisc::RequestExitWithStatus(false,0);
        }
        return;
    }
    if (Testing) TickSiteTest(Dt);
}
void AQuayCrane::TickSiteTest(float Dt)
{
    if (SiteTestTime==0) UE_LOG(LogTemp,Display,TEXT("SITE_TEST_STEP: %.6f simulation seconds per frame"),Dt);
    SiteTestTime+=Dt;
    const bool Full=FParse::Param(FCommandLine::Get(),TEXT("PortSimFullUnloadTest"));
    const bool Continuous=FParse::Param(FCommandLine::Get(),TEXT("PortSimContinuousTrafficTest"));
    int32 TestShipments=SiteLogistics->Vehicles.Num()*2;
    FParse::Value(FCommandLine::Get(),TEXT("PortSimSiteShipmentLimit="),TestShipments);
    const int32 Expected=Full?SiteLogistics->InitialShipCount():FMath::Clamp(TestShipments,9,SiteLogistics->InitialShipCount());
    auto Finish=[](bool Pass,const FString& Why)
    {
        UE_LOG(LogTemp,Display,TEXT("PORTSIM_SITE_%s: %s"),Pass?TEXT("PASS"):TEXT("FAIL"),*Why);
        FPlatformMisc::RequestExitWithStatus(false,Pass?0:1);
    };
    FString Error;
    for(const auto& Crane:WorkingCranes)
    {
        if(Crane->bSTS && !Crane->HasSTSProfile()) { Finish(false,TEXT("Site STS missing reference profile")); return; }
        if(Crane->bCarrying && !Crane->Observation.AllLocked())
        { Finish(false,TEXT("Site crane carried cargo without four locks")); return; }
        if(!Crane->bSTS && Crane->HasSTSProfile()) { Finish(false,TEXT("STS reference incorrectly assigned to RMG")); return; }
        if(!Crane->bSTS && !Crane->HasRMGProfile()) { Finish(false,TEXT("Site RMG missing independent reference profile")); return; }
    }
    int32 SizeCounts[3]={0,0,0};
    for(const auto& Container:SiteLogistics->ShipContainers)
    {
        const auto* Carrier=Cast<APortAGVActor>(Container->GetAttachParentActor());
        const bool TestCoG=Carrier && Carrier->HasValidEmergencyCoG(Container,STSProfile.ContainerCoG);
        if(!FMath::IsNearlyEqual(Container->MassKg,Container->TareMassKg+Container->CargoMassKg,1.f) ||
            (!Container->CoGOffsetCm.Equals(STSProfile.ContainerCoG) && !TestCoG))
        { Finish(false,TEXT("Site container gross mass/CoG specification mismatch")); return; }
        SizeCounts[Container->LengthFt==20?0:Container->LengthFt==40?1:2]++;
    }
    const FIntVector ExpectedSizes=PortContainerSpecification::CountsForManifest(SiteLogistics->ShipContainers.Num());
    if(SizeCounts[0]!=ExpectedSizes.X || SizeCounts[1]!=ExpectedSizes.Y || SizeCounts[2]!=ExpectedSizes.Z)
    { Finish(false,TEXT("Container manifest does not match rounded 28/70/2 quotas")); return; }
    if (!SiteLogistics->Validate(Error)) { Finish(false,Error); return; }
    if(FParse::Param(FCommandLine::Get(),TEXT("PortSimSTSAGVFault")))
        for(const auto& Crane:WorkingCranes)
            if(Crane->bSTS && Crane->bCarrying && Crane->Stage==5 && Crane->Observation.bAGVAligned && IsValid(Crane->GetHandoverVehicle()))
            { Crane->GetHandoverVehicle()->AddActorWorldOffset(FVector(100,0,0)); break; }
    if (SiteTestTime>((Full || Continuous)?200000.f:14000.f)) { Finish(false,TEXT("Integrated logistics timeout")); return; }
    if(Continuous && SiteTestStage==5 && !bTrafficFlowCheckpoint && SiteLogistics->Delivered>=217)
    {
        int32 Waiting=0,InlandWaiting=0;
        for(const auto& V:SiteLogistics->Vehicles) if(V->Sensors.bFMSHold)
        { ++Waiting; InlandWaiting+=V->GetActorLocation().X>50000.f; }
        UE_LOG(LogTemp,Display,TEXT("FMS_FLOW_CHECKPOINT: delivered=%d fms_waiting=%d inland_x_over_500m_waiting=%d"),
            SiteLogistics->Delivered,Waiting,InlandWaiting);
        bTrafficFlowCheckpoint=true;
    }
    bool Moving=false;
    for (const auto& Vehicle:SiteLogistics->Vehicles) Moving|=Vehicle->Speed>1.f;
    auto Freeze=[&](bool Emergency)
    {
        bAutoPaused=!Emergency; bEmergencyStop=Emergency;
        SiteLogistics->Advance(0,true);
        SiteTestPositions=SiteLogistics->Snapshot(); SiteTestHold=SiteTestTime;
    };
    if (SiteTestStage==0 && Moving) { Freeze(false); SiteTestStage=1; }
    else if ((SiteTestStage==1 || SiteTestStage==3) && SiteTestTime-SiteTestHold>1.f)
    {
        const auto Current=SiteLogistics->Snapshot();
        if (Current.Num()!=SiteTestPositions.Num()) { Finish(false,TEXT("Pause changed actors")); return; }
        for (int32 I=0;I<Current.Num();++I)
            if (!Current[I].Equals(SiteTestPositions[I],.1f)) { Finish(false,FString::Printf(TEXT("Pause/E-stop moved AGV, crane or cargo: index=%d before=%s after=%s"),I,*SiteTestPositions[I].ToString(),*Current[I].ToString())); return; }
        bAutoPaused=bEmergencyStop=false; ++SiteTestStage;
    }
    else if (SiteTestStage==2 && Moving) { Freeze(true); SiteTestStage=3; }
    else if (SiteTestStage==4 && Moving)
    {
        SiteLogistics->ResetLogistics();
        if (SiteLogistics->Delivered || SiteLogistics->InTransit() || !SiteLogistics->Validate(Error))
        { Finish(false,TEXT("In-transit reset: ")+Error); return; }
        SiteTestStage=5;
    }
    else if (SiteTestStage==5 && SiteLogistics->Delivered>=Expected && (Continuous || SiteLogistics->IsIdle()))
    {
        int32 UsedAGVs=0,MinimumJobs=MAX_int32,MaximumJobs=0;
        for (const auto& Vehicle:SiteLogistics->Vehicles) if(Vehicle->CompletedJobs>0)
        { ++UsedAGVs; MinimumJobs=FMath::Min(MinimumJobs,Vehicle->CompletedJobs); MaximumJobs=FMath::Max(MaximumJobs,Vehicle->CompletedJobs); }
        if(UsedAGVs<(bUnifiedTerminal?9:8)) { Finish(false,TEXT("Cost-aware dispatch used too few AGVs for concurrent STS service")); return; }
        const int32 RequiredPrefetch=FMath::Clamp(Expected-(bUnifiedTerminal?9:8),0,bUnifiedTerminal?9:6);
        if (SiteLogistics->PeakMovingVehicles<2 || SiteLogistics->PrefetchedJobs<RequiredPrefetch)
        { Finish(false,TEXT("AGVs did not move concurrently or STSs did not prepare ahead")); return; }
        // Yard-support admission can correctly select parked vehicles instead
        // of direct STS returns. Queue hits are a metric, not a completion goal.
        if (bUnifiedTerminal && SiteLogistics->PlacedContainers.Num()!=SiteLogistics->Delivered)
        { Finish(false,TEXT("Completed shipments are missing physical yard containers")); return; }
        if (Full && (SiteLogistics->ShipRemaining()!=0 || SiteLogistics->InTransit()!=0 || SiteLogistics->PlacedContainers.Num()!=Expected))
        { Finish(false,TEXT("Full vessel inventory was not physically stored")); return; }
        UE_LOG(LogTemp,Display,TEXT("RECEIVING_METRICS: capacity=%d delivered=%d queued_handoffs=%d elapsed_simulation=%.1f"),SiteLogistics->ReceivingCapacity,SiteLogistics->Delivered,SiteLogistics->QueuedHandoffs,SiteTestTime);
        UE_LOG(LogTemp,Display,TEXT("DISPATCH_METRICS: peak_moving_agvs=%d prefetched_jobs=%d used_agvs=%d jobs_per_used_agv=%d..%d"),
            SiteLogistics->PeakMovingVehicles,SiteLogistics->PrefetchedJobs,UsedAGVs,MinimumJobs,MaximumJobs);
        const FString Summary=FString::Printf(TEXT("yard %d -> %d; %d vessel containers; %d/%d AGVs selected by ETA plus bounded fairness, %d complete planned STS/AGV/RMG shipments; mixed sizes, concurrent traffic, STS prefetch, physical placement, pause/E-stop and reset"),
            SiteLogistics->BaselineYard,SiteLogistics->InitialYard,SiteLogistics->InitialShipCount(),UsedAGVs,SiteLogistics->Vehicles.Num(),SiteLogistics->Delivered);
        if(Continuous) SiteLogistics->ExportDashboard(false,true);
        SiteLogistics->ResetLogistics();
        if (!SiteLogistics->Validate(Error) || SiteLogistics->Delivered || SiteLogistics->InTransit())
        { Finish(false,TEXT("Completed reset: ")+Error); return; }
        SiteTestStage=6; Finish(true,Summary);
    }
}
void AQuayCrane::FocusNextSiteCrane()
{
    const int32 ExtraViews=bUnifiedTerminal?1+ConfiguredShips:0;
    const int32 ViewCount=ExtraViews+WorkingCranes.Num();
    if(ViewCount==0) return;
    bFreeCamera=false; bFollowAGV=false;
    SiteCameraIndex=(SiteCameraIndex+1)%ViewCount;
    if(bUnifiedTerminal && SiteCameraIndex==0)
    {
        CameraArm->SetRelativeLocation(FVector(TerminalLayout::SiteCmX(7800),50050,0));
        CameraArm->TargetArmLength=42000.f;
        CameraArm->SetRelativeRotation(FRotator(-65,38,0));
        CameraViewName=TEXT("AGV 대기소");
        return;
    }
    if(bUnifiedTerminal && SiteCameraIndex<=ConfiguredShips)
    {
        const int32 Vessel=SiteCameraIndex-1;
        CameraArm->SetRelativeLocation(FVector(2500,TerminalLayout::VesselCenterY(Vessel)*100.f,1000));
        CameraArm->TargetArmLength=40000.f;
        CameraArm->SetRelativeRotation(FRotator(-55,38,0));
        CameraViewName=FString::Printf(TEXT("선박 %d 안벽"),Vessel+1);
        return;
    }
    const auto* Crane=WorkingCranes[SiteCameraIndex-ExtraViews].Get();
    CameraArm->SetRelativeLocation(Crane->GetActorLocation()+FVector(0,0,Crane->bSTS?2300:800));
    CameraArm->TargetArmLength=Crane->bSTS?20000.f:10500.f;
    CameraArm->SetRelativeRotation(FRotator(-35,Crane->bSTS?38.f:128.f,0));
    CameraViewName=FString::Printf(TEXT("%s %d"),Crane->bSTS?TEXT("STS"):TEXT("RMG"),SiteCameraIndex-ExtraViews+1);
}
