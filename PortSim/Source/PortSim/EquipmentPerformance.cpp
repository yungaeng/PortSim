#include "EquipmentPerformance.h"
#include "PortSiteLogistics.h"
#include "PortWorkingCrane.h"
#include "PortAGVActor.h"
#include "PortSupportVehicle.h"
#include "QuayCrane.h"
#include "Dom/JsonObject.h"

void APortSiteLogistics::ResetEquipmentPerformance()
{
    EquipmentWorkingSeconds.Reset();
    for(auto& B:BerthPerformance)B=FPortBerthPerformance();
}

void APortSiteLogistics::AdvanceEquipmentPerformance(float Dt,bool Paused)
{
    if(!FMath::IsFinite(Dt)||Dt<=0)return;
    if(const auto* Controller=Cast<AQuayCrane>(GetOwner()))
        if(!Controller->bAutoRunning||Controller->bAutoPaused)return;
    if(LaneCount==9)
    {
        int32 Planned[3]={},Handled[3]={};bool Started[3]={};
        for(const auto& C:Manifest)
        {
            const int32 B=(C.OriginalSTS==INDEX_NONE?C.STS:C.OriginalSTS)/3;
            if(B<0||B>=3)continue;
            ++Planned[B];Handled[B]+=bool(C.HandoverMask&1);Started[B]|=C.State!=0||bool(C.HandoverMask&1);
        }
        for(const auto& C:ExportCargo)
        {
            const int32 B=(C.OriginalSTS==INDEX_NONE?C.STS:C.OriginalSTS)/3;
            if(B<0||B>=3)continue;
            ++Planned[B];Handled[B]+=C.State==2;Started[B]|=C.State!=0;
        }
        for(int32 I=0;I<3;++I)
        {
            auto& B=BerthPerformance[I];B.Planned=Planned[I];B.Handled=Handled[I];
            if(B.StartedAt<0&&Started[I])B.StartedAt=IncidentMeasureClock;
            if(B.CompletedAt<0&&B.StartedAt>=0&&B.Planned>0&&B.Handled==B.Planned)B.CompletedAt=IncidentMeasureClock;
        }
    }
    // Observation time is the separate operational clock, which includes
    // weather/fault waits. Only productive controller intervals enter work time.
    if(Paused||!Fault.IsEmpty())return;
    for(const auto& K:Equipment)if(IsValid(K)&&K->IsProductivelyWorking())
        EquipmentWorkingSeconds.FindOrAdd(K->GetName())+=Dt;
    for(int32 I=0;I<Vehicles.Num();++I)
    {
        const auto* V=Vehicles[I].Get();if(!V||!Jobs.IsValidIndex(I))continue;
        const auto& J=Jobs[I];
        if(!J.Stage||J.Stage==FSiteTransfer::Charging||V->bDriveFault||V->bIncidentSensorFault||V->Emergency.Active)continue;
        bool Working=V->Speed>.1f&&!V->Sensors.bFMSHold&&!V->bIncidentRoadHold&&!J.bEquipmentWaiting;
        if(!Working)for(const auto& K:Equipment)
            if(K->GetHandoverVehicle()==V&&K->IsProductivelyWorking()&&
                ((K->bSTS&&K->Stage==5)||(!K->bSTS&&K->Stage==2))) {Working=true;break;}
        if(Working)EquipmentWorkingSeconds.FindOrAdd(V->GetName())+=Dt;
    }
    TSet<FString> Seen;
    for(const auto& R:LoadedAGVRecoveries)if(R.Stacker.IsValid())
    {
        const auto* RS=R.Stacker.Get();
        if(!Seen.Contains(RS->GetName())&&RS->PhysicsFault.IsEmpty()&&
            (FMath::Abs(RS->TireDynamics.U)>.001||RS->HydraulicVelocityMps.Size()>.001))
            EquipmentWorkingSeconds.FindOrAdd(RS->GetName())+=Dt;
        Seen.Add(RS->GetName());
    }
}

TSharedPtr<FJsonObject> APortSiteLogistics::CaptureBerthPerformance(int32 Berth) const
{
    if(LaneCount!=9||Berth<0||Berth>=3)return nullptr;
    const auto& B=BerthPerformance[Berth];auto O=MakeShared<FJsonObject>();
    const double Seconds=B.Seconds(IncidentMeasureClock);
    O->SetNumberField(TEXT("berth_id"),Berth+1);O->SetNumberField(TEXT("handled_containers"),B.Handled);
    O->SetNumberField(TEXT("planned_containers"),B.Planned);O->SetNumberField(TEXT("operation_seconds"),Seconds);
    O->SetNumberField(TEXT("started_at"),B.StartedAt);O->SetNumberField(TEXT("completed_at"),B.CompletedAt);
    if(Seconds>0)O->SetNumberField(TEXT("containers_per_berth_hour"),B.Handled*3600./Seconds);
    else O->SetField(TEXT("containers_per_berth_hour"),MakeShared<FJsonValueNull>());
    return O;
}

TSharedPtr<FJsonObject> APortSiteLogistics::CaptureEquipmentPerformance(const AActor* Actor) const
{
    const auto* K=Cast<APortWorkingCrane>(Actor);const auto* V=Cast<APortAGVActor>(Actor);
    const auto* RS=Cast<APortSupportVehicle>(Actor);
    if((!K||!Equipment.Contains(K))&&(!V||!Vehicles.Contains(V))&&!RS)return nullptr;
    auto O=MakeShared<FJsonObject>();const double Observed=IncidentMeasureClock;
    const double Working=FMath::Clamp(EquipmentWorkingSeconds.FindRef(Actor->GetName()),0.,Observed);
    O->SetNumberField(TEXT("schema_version"),1);O->SetNumberField(TEXT("observed_seconds"),Observed);
    O->SetNumberField(TEXT("working_seconds"),Working);O->SetStringField(TEXT("time_basis"),TEXT("operational_simulation"));
    if(Observed>0)O->SetNumberField(TEXT("utilization_percent"),100*Working/Observed);
    else O->SetField(TEXT("utilization_percent"),MakeShared<FJsonValueNull>());
    if(K)
    {
        O->SetNumberField(TEXT("handled_containers"),K->CompletedJobs);
        if(Observed>0)O->SetNumberField(TEXT("crane_containers_per_hour"),K->CompletedJobs*3600./Observed);
        else O->SetField(TEXT("crane_containers_per_hour"),MakeShared<FJsonValueNull>());
        const int32 Index=Equipment.IndexOfByKey(K);
        if(K->bSTS&&Index>=YardCraneCount)
            if(auto Berth=CaptureBerthPerformance((Index-YardCraneCount)/3))O->SetObjectField(TEXT("berth"),Berth);
    }
    return O;
}
