#include "PortWorkingCrane.h"
#include "PortSiteLogistics.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/World.h"

void APortWorkingCrane::SetGantryFailure(bool Failed)
{
    if(bAppliedGantryFault==Failed && bGantryFault==Failed) return;
    bAppliedGantryFault=bGantryFault=Failed;
    bFaultEvacuating=false; AxisVelocity=FVector::ZeroVector;
    if(Failed)
    {
        if(Fault.IsEmpty() || IsEquipmentUnavailable()) Stop(TEXT("Gantry drive unavailable"),EPortCraneFault::GantryUnavailable);
    }
    else if(FaultType==EPortCraneFault::GantryUnavailable)
    {
        Fault.Empty(); FaultType=EPortCraneFault::None;
        if(bSensorFault) Stop(TEXT("Injected sensor unavailable"),EPortCraneFault::SensorUnavailable);
    }
}

void APortWorkingCrane::RepairEquipment()
{
    // Never use an equipment repair to clear an overload/collision/lock fault.
    if(!Fault.IsEmpty() && !IsEquipmentUnavailable()) return;
    if(bGantryFault) SetGantryFailure(false);
    SetSensorFailure(false);
}

double APortWorkingCrane::SensorRepairRemaining() const
{
    if(!bSensorFault || !bAutoSensorRecovery) return -1;
    const double Repair=SensorRepairMethod==EPortSensorRepairMethod::Reboot?SensorRebootSeconds:SensorTechnicianRepairSeconds;
    return FMath::Max(0.,FMath::Max(0.,Repair)+FMath::Max(0.,SensorSelfCheckSeconds)-SensorRepairElapsed);
}

bool APortWorkingCrane::CanFaultEvacuate() const
{
    return CanServiceTravel() && IsSensorUnavailable() && bSensorFault && bAllowSensorFaultEvacuation;
}

bool APortWorkingCrane::CanServiceTravel() const
{
    return bConfigured && bSTS && bEnabled && (Fault.IsEmpty() || IsSensorUnavailable()) &&
        bIndependentTravelSafetyHealthy && !bGantryFault && !bFatalSensorTest &&
        !bJobActive && !RetainsLoad() && Head.Z>=SafeZ-5 && STSProfile.bReady;
}

bool APortWorkingCrane::RequestFaultEvacuation(float WorldY,APortSiteLogistics* Coordinator)
{
    const bool Returning=Fault.IsEmpty() && bFaultEvacuationComplete && FMath::Abs(WorldY-Home.Y)<1;
    if((Returning?!CanServiceTravel():!CanFaultEvacuate()) || bFaultEvacuating || (!Returning && bFaultEvacuationComplete) || !IsValid(Coordinator) ||
        !FMath::IsFinite(WorldY) || !Coordinator->CanSTSGantryMove(this,WorldY)) return false;
    RecoveryCoordinator=Coordinator; FaultEvacuationTargetY=WorldY; bFaultEvacuating=true;
    return true;
}

void APortWorkingCrane::AdvanceFailureRecovery(float Dt,bool GlobalPaused)
{
    if(GlobalPaused || !bEnabled || Dt<=0 || (!IsEquipmentUnavailable() && !bFaultEvacuating)) return;
    if(bSensorFault && bAutoSensorRecovery && !bFatalSensorTest)
    {
        SensorRepairElapsed+=Dt;
        const double Repair=FMath::Max(0.,SensorRepairMethod==EPortSensorRepairMethod::Reboot?SensorRebootSeconds:SensorTechnicianRepairSeconds);
        SensorRepairPhase=SensorRepairElapsed<Repair?
            (SensorRepairMethod==EPortSensorRepairMethod::Reboot?TEXT("rebooting"):TEXT("technician_repair")):TEXT("self_check");
    }
    if(bFaultEvacuating)
    {
        if(!CanServiceTravel() || !RecoveryCoordinator.IsValid())
        { bFaultEvacuating=false; AxisVelocity=FVector::ZeroVector; }
        else
        {
            const int32 Steps=CraneSimulationSteps::Count(Dt,120.);
            const double Step=double(Dt)/Steps;
            const double Acceleration=FMath::Min(10.,RailAccelerationLimit(STSProfile.GantryAcceleration));
            if(Acceleration<=0) { Stop(TEXT("Insufficient wheel/rail adhesion for service travel")); bFaultEvacuating=false; return; }
            const double Limit=FMath::Min(25.,double(STSProfile.GantrySpeed)); // 0.25 m/s service crawl assumption
            for(int32 I=0;I<Steps;++I)
            {
                const double Error=FaultEvacuationTargetY-GantryWorldY();
                const double Previous=AxisVelocity.Y;
                const bool RepairReady=SensorRepairRemaining()==0;
                if(RepairReady) SensorRepairPhase=TEXT("awaiting_safe_stop");
                const double Desired=Acceleration>0 && !RepairReady?FMath::Sign(Error)*FMath::Min(Limit,FMath::Sqrt(2*Acceleration*FMath::Abs(Error))):0;
                const double Velocity=FMath::FInterpConstantTo(Previous,Desired,Step,FMath::Max(0.,Acceleration));
                const double Delta=(Previous+Velocity)*.5*Step;
                const double Stopping=Acceleration>0?Velocity*FMath::Abs(Velocity)/(2*Acceleration):0;
                bool Clear=RecoveryCoordinator->CanSTSGantryMove(this,GantryWorldY()+Delta+Stopping);
                // Independent service anti-collision model checks the full
                // stopping sweep of all four bogies, including live AGVs.
                FCollisionQueryParams Query(SCENE_QUERY_STAT(STSServiceDrive),false); Query.AddIgnoredActor(this);
                for(const auto& Bogie:Bogies) if(Clear)
                {
                    const FVector Start=Bogie->Bounds.Origin+FVector(0,0,20);
                    const FVector Extent(Bogie->Bounds.BoxExtent.X+10,Bogie->Bounds.BoxExtent.Y+10,FMath::Max(1.,Bogie->Bounds.BoxExtent.Z-30));
                    Clear=!GetWorld()->SweepTestByChannel(Start,Start+FVector(0,Delta+Stopping,0),FQuat::Identity,
                        ECC_WorldDynamic,FCollisionShape::MakeBox(Extent),Query);
                }
                bRailSafetyHold=!Clear;
                AxisVelocity=FVector(0,Clear?Velocity:0,0);
                if(Clear) Head.Y+=Delta;
                PlantAcceleration=FVector(0,(AxisVelocity.Y-Previous)/Step,0);
                auto Config=STSProfile.Dynamics; Config.Wind=Orientation.UnrotateVector(WindVelocityMetersPerSecond);
                SuspensionState.Step(Config,Step,FMath::Max(.1,(BeamZ-Head.Z)*.01),0,
                    PlantAcceleration*.01,AxisVelocity*.01,STSProfile.SpreaderMassKg,FVector::ZeroVector,STSProfile.HoistPowerW);
                if(!SuspensionState.Fault.IsEmpty()) { Stop(SuspensionState.Fault); bFaultEvacuating=false; return; }
                SuspendedOffset=SuspensionState.Offset*100.; UpdateParts();
                if(RepairReady && FMath::Abs(AxisVelocity.Y)<1)
                { bFaultEvacuating=false; bFaultEvacuationComplete=FMath::Abs(GantryWorldY()-Home.Y)>5; AxisVelocity=FVector::ZeroVector; break; }
                if(FMath::Abs(Error)<2 && FMath::Abs(AxisVelocity.Y)<2)
                {
                    AxisVelocity=FVector::ZeroVector; bFaultEvacuating=false; bFaultEvacuationComplete=!Fault.IsEmpty();
                    UE_LOG(LogTemp,Display,TEXT("STS_FAULT_EVACUATED: crane=%d y_cm=%.2f"),CraneID,GantryWorldY());
                    break;
                }
                if(!Clear) break;
            }
        }
    }
    // Repair time is simulation time. A paused simulation cannot finish it.
    // The normal controller resamples and validates sensors before any lift.
    if(IsSensorUnavailable() && bSensorFault && bAutoSensorRecovery && !bFatalSensorTest &&
        SensorRepairRemaining()==0 && (!bFaultEvacuating || AxisVelocity.IsNearlyZero(1.)))
        SetSensorFailure(false);
}
