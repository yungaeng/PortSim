#include "QuayCrane.h"
#include "PortWorkingCrane.h"
#include "PortSiteLogistics.h"
#include "PortAGVActor.h"
#include "PortContainerActor.h"
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
    const bool Testing=Full || FParse::Param(FCommandLine::Get(),TEXT("PortSimSiteTest"));
    if (Testing) SiteLogistics->DispatchLimit=Full?SiteLogistics->InitialShipCount():SiteLogistics->Vehicles.Num()*2;
    const bool FirstWave=FParse::Param(FCommandLine::Get(),TEXT("PortSimFirstWaveTest"));
    const bool CarLike=FParse::Param(FCommandLine::Get(),TEXT("PortSimCarLikeTest"));
    if (FirstWave) SiteLogistics->DispatchLimit=9;
    else if (CarLike) SiteLogistics->DispatchLimit=1;
    SiteLogistics->Advance(Dt,bAutoPaused || bEmergencyStop);
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
        else if (SiteLogistics->Delivered==9)
        {
            if (!SiteLogistics->ReturnCorridorUse(1) || !SiteLogistics->ReturnCorridorUse(2))
            {
                UE_LOG(LogTemp,Error,TEXT("PORTSIM_FIRST_WAVE_FAIL: alternate AGV return corridors unused (south=%d central=%d north=%d)"),
                    SiteLogistics->ReturnCorridorUse(0),SiteLogistics->ReturnCorridorUse(1),SiteLogistics->ReturnCorridorUse(2));
                FPlatformMisc::RequestExitWithStatus(false,1);
                return;
            }
            SiteLogistics->ExportDashboard(false,true);
            UE_LOG(LogTemp,Display,TEXT("PORTSIM_FIRST_WAVE_PASS: policy=%s delivered=9 simulation_seconds=%.6f step=%.6f return_routes=%d/%d/%d"),
                SiteLogistics->UsesCargoAlignedHandover()?TEXT("cargo_aligned"):TEXT("fixed"),SiteTestTime,Dt,
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
    const int32 Expected=Full?SiteLogistics->InitialShipCount():SiteLogistics->Vehicles.Num()*2;
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
        if(!FMath::IsNearlyEqual(Container->MassKg,Container->TareMassKg+Container->CargoMassKg,1.f) ||
            !Container->CoGOffsetCm.Equals(STSProfile.ContainerCoG))
        { Finish(false,TEXT("Site container gross mass/CoG specification mismatch")); return; }
        SizeCounts[Container->LengthFt==20?0:Container->LengthFt==40?1:2]++;
    }
    if(SiteLogistics->ShipContainers.Num()>=10 && (!SizeCounts[0] || !SizeCounts[1] || !SizeCounts[2]))
    { Finish(false,TEXT("20/40/45 ft mixed manifest is incomplete")); return; }
    if (!SiteLogistics->Validate(Error)) { Finish(false,Error); return; }
    if(FParse::Param(FCommandLine::Get(),TEXT("PortSimSTSAGVFault")))
        for(const auto& Crane:WorkingCranes)
            if(Crane->bSTS && Crane->bCarrying && Crane->Stage==5 && Crane->Observation.bAGVAligned && IsValid(Crane->GetHandoverVehicle()))
            { Crane->GetHandoverVehicle()->AddActorWorldOffset(FVector(100,0,0)); break; }
    if (SiteTestTime>(Full?200000.f:14000.f)) { Finish(false,TEXT("Integrated logistics timeout")); return; }
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
    else if (SiteTestStage==5 && SiteLogistics->Delivered==Expected && SiteLogistics->IsIdle())
    {
        int32 UsedAGVs=0,MinimumJobs=MAX_int32,MaximumJobs=0;
        for (const auto& Vehicle:SiteLogistics->Vehicles) if(Vehicle->CompletedJobs>0)
        { ++UsedAGVs; MinimumJobs=FMath::Min(MinimumJobs,Vehicle->CompletedJobs); MaximumJobs=FMath::Max(MaximumJobs,Vehicle->CompletedJobs); }
        if(UsedAGVs<(bUnifiedTerminal?9:8)) { Finish(false,TEXT("Cost-aware dispatch used too few AGVs for concurrent STS service")); return; }
        if (SiteLogistics->PeakMovingVehicles<2 || SiteLogistics->PrefetchedJobs<(bUnifiedTerminal?9:6))
        { Finish(false,TEXT("AGVs did not move concurrently or STSs did not prepare ahead")); return; }
        if (bUnifiedTerminal && SiteLogistics->QueuedHandoffs<9)
        { Finish(false,TEXT("Next AGVs were not preassigned to the STS queues")); return; }
        if (Full && (SiteLogistics->ShipRemaining()!=0 || SiteLogistics->InTransit()!=0 || SiteLogistics->PlacedContainers.Num()!=Expected))
        { Finish(false,TEXT("Full vessel inventory was not physically stored")); return; }
        UE_LOG(LogTemp,Display,TEXT("RECEIVING_METRICS: capacity=%d delivered=%d queued_handoffs=%d elapsed_simulation=%.1f"),SiteLogistics->ReceivingCapacity,SiteLogistics->Delivered,SiteLogistics->QueuedHandoffs,SiteTestTime);
        UE_LOG(LogTemp,Display,TEXT("DISPATCH_METRICS: peak_moving_agvs=%d prefetched_jobs=%d used_agvs=%d jobs_per_used_agv=%d..%d"),
            SiteLogistics->PeakMovingVehicles,SiteLogistics->PrefetchedJobs,UsedAGVs,MinimumJobs,MaximumJobs);
        const FString Summary=FString::Printf(TEXT("yard %d -> %d; %d vessel containers; %d/%d AGVs selected by ETA plus bounded fairness, %d complete planned STS/AGV/RMG shipments; mixed sizes, concurrent traffic, STS prefetch, physical placement, pause/E-stop and reset"),
            SiteLogistics->BaselineYard,SiteLogistics->InitialYard,SiteLogistics->InitialShipCount(),UsedAGVs,SiteLogistics->Vehicles.Num(),SiteLogistics->Delivered);
        SiteLogistics->ResetLogistics();
        if (!SiteLogistics->Validate(Error) || SiteLogistics->Delivered || SiteLogistics->InTransit())
        { Finish(false,TEXT("Completed reset: ")+Error); return; }
        SiteTestStage=6; Finish(true,Summary);
    }
}
void AQuayCrane::FocusNextSiteCrane()
{
    if (WorkingCranes.IsEmpty()) return;
    bFreeCamera=false;
    bFollowAGV=false;
    SiteCameraIndex=(SiteCameraIndex+1)%WorkingCranes.Num();
    const auto* Crane=WorkingCranes[SiteCameraIndex].Get();
    CameraArm->SetRelativeLocation(Crane->GetActorLocation()+FVector(0,0,Crane->bSTS?2300:800));
    CameraArm->TargetArmLength=Crane->bSTS?20000.f:10500.f;
    CameraArm->SetRelativeRotation(FRotator(-35,Crane->bSTS?38.f:128.f,0));
}
