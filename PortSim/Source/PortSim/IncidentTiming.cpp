#include "PortSiteLogistics.h"
#include "PortAGVActor.h"
#include "PortWorkingCrane.h"
#include "PortContainerActor.h"
#include "QuayCrane.h"
#include "Dom/JsonObject.h"
#include "HAL/FileManager.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Misc/SecureHash.h"
#include "UObject/UnrealType.h"

namespace
{
    FString Text(const FJsonObject& O,const TCHAR* Key)
    { FString V;O.TryGetStringField(Key,V);return V; }
    void EditableValues(FString& Out,const UObject* Object)
    {
        if(!Object)return;
        for(TFieldIterator<FProperty> P(Object->GetClass(),EFieldIteratorFlags::ExcludeSuper);P;++P)
            if(P->HasAnyPropertyFlags(CPF_Edit))
            { Out+=P->GetName()+TEXT("=");P->ExportText_InContainer(0,Out,Object,Object,const_cast<UObject*>(Object),PPF_None);Out+=TEXT("\n"); }
    }
}

void APortSiteLogistics::BeginIncidentMeasurement()
{
    IncidentMeasureClock=IncidentHaltSeconds=0;IncidentCompletionTimes.Reset();bIncidentReferenceClean=true;
    FString Basis=TEXT("incident-timing-v1\n")+STSProfile.SnapshotJson+ShipStowageAssumptions.SnapshotJson;
    Basis+=FString::Printf(TEXT("\nlanes=%d vehicles=%d depot=%d apron=%d priority=%d central=%d\n"),
        LaneCount,Vehicles.Num(),bDepotParking,bReuseFreedApron,bPickupRoadPriority,CentralCount);
    TArray<FString> Files;
    IFileManager::Get().FindFilesRecursive(Files,*(FPaths::ProjectDir()/TEXT("Source/PortSim")),TEXT("*.cpp"),true,false);
    IFileManager::Get().FindFilesRecursive(Files,*(FPaths::ProjectDir()/TEXT("Source/PortSim")),TEXT("*.h"),true,false);
    IFileManager::Get().FindFilesRecursive(Files,*FPaths::ProjectConfigDir(),TEXT("*.json"),true,false);
    Files.Sort();
    for(const auto& File:Files){FString Value;if(FFileHelper::LoadFileToString(Value,*File))Basis+=FPaths::GetCleanFilename(File)+TEXT("\n")+Value;}
    for(const auto& C:Manifest)
    {
        Basis+=FString::Printf(TEXT("\nC%d|%d|%d|%s|"),C.ID,C.STS,C.PlannedSlot,*C.Transform.ToString());
        if(C.Actor.IsValid()){Basis+=FString::Printf(TEXT("%d|%.6f|%s|%s"),C.Actor->LengthFt,C.Actor->MassKg,*C.Actor->CoGOffsetCm.ToString(),*C.Actor->GetActorTransform().ToString());EditableValues(Basis,C.Actor.Get());}
    }
    for(const auto& C:ExportCargo)Basis+=FString::Printf(TEXT("\nexport=%d|%d|%d|%s"),C.ID,C.STS,C.PlannedSlot,*C.Transform.ToString());
    for(const auto& Y:Yard)Basis+=FString::Printf(TEXT("\nslot=%s|%d|%d|%d"),*Y.Position.ToString(),Y.LengthFt,Y.Occupied,Y.Below);
    for(const auto& K:Equipment)EditableValues(Basis,K);
    for(const auto& V:Vehicles)EditableValues(Basis,V);
    if(const auto* Controller=Cast<AQuayCrane>(GetOwner()))
    {
        const auto* E=Controller->GetEnvironment();EditableValues(Basis,E);IncidentEnvironmentRevision=E->InputRevision;
        const auto& Sea=E->GetSeaState();const auto& Wind=E->GetWindState();
        Basis+=FString::Printf(TEXT("\nenv=%d|%d|%.9f|%.9f|%d|%.9f|%d|%.9f|%.9f|%.9f|%.9f|%.9f|%.9f|%.9f"),
            E->InitializedSeed,E->IsWindEnabled(),Wind.ReferenceWindSpeedMetersPerSecond,Wind.SecondsUntilTarget,
            E->IsRainEnabled(),E->GetRainfallMmPerHour(),E->IsFogEnabled(),E->GetFogVisibilityMeters(),
            Sea.ElapsedSeconds,Sea.WaveHeightMeters,Sea.SwellHeightMeters,Sea.WavePhaseRadians,Sea.SwellPhaseRadians,double(Sea.bSeaMotionEnabled));
    }
    FTCHARToUTF8 Bytes(*Basis);uint8 Digest[20];FSHA1::HashBuffer(Bytes.Get(),Bytes.Length(),Digest);
    IncidentComparisonKey=BytesToHex(Digest,20);
}

void APortSiteLogistics::TrackIncidentImpact(AActor* Cause,AActor* From,AActor* To,AActor* Cargo)
{
    for(auto& Run:IncidentRuns)for(auto& S:Run.Steps)
    {
        if(S.MeasureAppliedAt<0 || S.MeasureCompletedAt>=0)continue;
        const bool Related=(Cause&&S.AffectedActors.Contains(Cause->GetName())) ||
            (From&&S.AffectedActors.Contains(From->GetName())) || (Cargo&&S.AffectedCargo.Contains(Cargo->GetName()));
        if(!Related)continue;
        for(auto* A:{Cause,From,To})if(A)S.AffectedActors.Add(A->GetName());
        if(Cargo)S.AffectedCargo.Add(Cargo->GetName());
    }
}

void APortSiteLogistics::AdvanceIncidentMeasurement(float Dt,bool Paused)
{
    const auto* Controller=Cast<AQuayCrane>(GetOwner());
    const bool Running=Controller?(Controller->bAutoRunning&&!Controller->bAutoPaused):(!Paused||!Fault.IsEmpty());
    if(!Running||!FMath::IsFinite(Dt)||Dt<=0)return;
    // This clock advances even when a fault, E-stop or weather hold freezes
    // the logistics clock. Manual pause and not-started/finished runs exclude time.
    IncidentMeasureClock+=Dt;
    const bool Halt=Paused||!Fault.IsEmpty();if(Halt)IncidentHaltSeconds+=Dt;
    if(bIncidentInputUsed||!Fault.IsEmpty()||EquipmentFaultEvents)bIncidentReferenceClean=false;
    if(Controller&&Controller->GetEnvironment()->InputRevision!=IncidentEnvironmentRevision)bIncidentReferenceClean=false;
    for(const auto& V:Vehicles)if(V->bDriveFault||V->bIncidentSensorFault||V->Emergency.Active||!V->IncidentFaults.IsEmpty())bIncidentReferenceClean=false;
    for(const auto& K:Equipment)if(!K->Fault.IsEmpty()||K->bSensorFault||K->bGantryFault||K->bIncidentPickupFailure||K->bIncidentPlacementFailure||!K->IncidentFaults.IsEmpty())bIncidentReferenceClean=false;
    TMap<int32,const APortAGVActor*> VehicleByID;
    if(!IncidentRuns.IsEmpty()&&!RoadBlockers.IsEmpty())
        for(const auto& Vehicle:Vehicles) VehicleByID.Add(Vehicle->VehicleID,Vehicle.Get());
    for(auto& Run:IncidentRuns)for(auto& S:Run.Steps)
    {
        if(S.MeasureAppliedAt<0||(S.MeasureCompletedAt>=0&&S.MeasureClearedAt>=0)||Text(*S.Command,TEXT("action"))==TEXT("set_environment"))continue;
        const FString ID=Text(*S.Command,TEXT("actor_id")),Action=Text(*S.Command,TEXT("action"));
        APortAGVActor* V=nullptr;APortWorkingCrane* K=nullptr;
        for(const auto& A:Vehicles)if(A->GetName()==ID)V=A;
        for(const auto& A:Equipment)if(A->GetName()==ID)K=A;
        bool Active=PortIncident::Extension(Action)?ExtendedIncidentActive(Action,V,K):true;
        if(V)
        {
            if(Action==TEXT("fail"))Active=V->bDriveFault;
            if(PortIncident::SensorFailure(Action))Active=V->bIncidentSensorFault||V->IncidentSensorPhase!=TEXT("healthy");
            if(Action==TEXT("worker_enter"))Active=V->Emergency.Active;
            if(Action==TEXT("misdelivery"))Active=V->bIncidentDestinationMismatch;
        }
        if(K)
        {
            if(Action==TEXT("fail_gantry"))Active=K->bGantryFault;
            if(PortIncident::SensorFailure(Action))Active=K->bSensorFault||K->IsSensorUnavailable();
            if(Action==TEXT("fail_pickup"))Active=K->bIncidentPickupFailure;
            if(Action==TEXT("fail_placement"))Active=K->bIncidentPlacementFailure;
        }
        if(!Active&&S.MeasureClearedAt<0)
        {S.MeasureClearedAt=IncidentMeasureClock;S.ResumeStage=V?Jobs[Vehicles.IndexOfByKey(V)].Stage:K?K->Stage:INDEX_NONE;S.ResumeCompleted=V?V->CompletedJobs:K?K->CompletedJobs:0;}
        if(S.MeasureCompletedAt>=0)continue; // Finished cargo does not imply the failed equipment was repaired.
        bool Resumed=false;
        if(!Halt&&!Active)
        {
            if(V&&V->Dynamics.Fault.IsEmpty()&&!V->bDriveFault&&!V->bIncidentSensorFault&&!V->Sensors.bFMSHold)
                Resumed=V->Speed>1||V->CompletedJobs>S.ResumeCompleted||Jobs[Vehicles.IndexOfByKey(V)].Stage!=S.ResumeStage;
            if(K&&K->Fault.IsEmpty()&&!K->IsEquipmentUnavailable())
                Resumed=K->Observation.DriveVelocity.Size()>1||K->CompletedJobs>S.ResumeCompleted||K->Stage!=S.ResumeStage;
        }
        // An isolated STS may still be broken while a replacement actually
        // resumes its cargo. Track the work, rather than requiring the old STS.
        if(S.Cargo.IsValid())
        {
            for(const auto& C:Manifest)if(C.Actor==S.Cargo)
            {
                const FString CargoID=FString::Printf(TEXT("C%d"),C.ID);
                if(const auto* End=IncidentCompletionTimes.Find(CargoID))
                {S.MeasureCompletedAt=*End;Resumed=true;}
                if(Equipment.IsValidIndex(YardCraneCount+C.STS))
                {
                    const auto* Replacement=Equipment[YardCraneCount+C.STS].Get();
                    if(!Halt&&Replacement!=K&&Replacement->CargoActor==S.Cargo&&Replacement->Fault.IsEmpty()&&Replacement->Observation.DriveVelocity.Size()>1)
                    {Resumed=true;S.AffectedActors.Add(Replacement->GetName());}
                }
            }
            for(const auto& C:ExportCargo)if(C.Actor==S.Cargo&&C.State==2)
            {S.MeasureCompletedAt=IncidentMeasureClock;Resumed=true;}
        }
        else if(S.State==TEXT("verified")){S.MeasureCompletedAt=IncidentMeasureClock;Resumed=true;}
        if(!Halt)for(const auto& A:Equipment)if(A.Get()!=K&&IsValid(A->CargoActor.Get())&&
            S.AffectedCargo.Contains(A->CargoActor->GetName())&&A->Fault.IsEmpty()&&A->Observation.DriveVelocity.Size()>1)
        {Resumed=true;S.AffectedActors.Add(A->GetName());}
        if(!S.Cargo.IsValid()&&!S.AffectedCargo.IsEmpty())
        {
            int32 Finished=0;double Last=0;
            for(const auto& C:Manifest)if(C.Actor.IsValid()&&S.AffectedCargo.Contains(C.Actor->GetName()))
                if(const auto* End=IncidentCompletionTimes.Find(FString::Printf(TEXT("C%d"),C.ID)))
                {++Finished;Last=FMath::Max(Last,*End);}
            if(Finished==S.AffectedCargo.Num()){S.MeasureCompletedAt=Last;Resumed=true;}
        }
        if(Resumed&&S.MeasureResumedAt<0)S.MeasureResumedAt=S.MeasureCompletedAt>=0?
            FMath::Min(IncidentMeasureClock,S.MeasureCompletedAt):IncidentMeasureClock;
        // Propagate observed traffic waits to a fixed point, with unique actor
        // sets. Parallel delays are never added together as elapsed time.
        for(int32 Pass=0;Pass<Vehicles.Num();++Pass)
        {
            const int32 Before=S.AffectedActors.Num();
            for(const auto& B:RoadBlockers)
            {
                const auto* Waiting=VehicleByID.FindRef(B.Key);
                const auto* Blocking=VehicleByID.FindRef(B.Value);
                if(Waiting&&Blocking&&Waiting->Sensors.bFMSHold&&S.AffectedActors.Contains(Blocking->GetName()))
                    S.AffectedActors.Add(Waiting->GetName());
            }
            if(Before==S.AffectedActors.Num())break;
        }
        for(int32 I=0;I<Jobs.Num();++I)
        {
            const auto& J=Jobs[I];if(!J.Stage)continue;
            if(J.bEquipmentWaiting&&Equipment.IsValidIndex(J.RMG)&&S.AffectedActors.Contains(Equipment[J.RMG]->GetName()))S.AffectedActors.Add(Vehicles[I]->GetName());
            if(S.AffectedActors.Contains(Vehicles[I]->GetName()))
            {
                if(J.Actor.IsValid())S.AffectedCargo.Add(J.Actor->GetName());
                if(Equipment.IsValidIndex(YardCraneCount+J.STS)&&Equipment[YardCraneCount+J.STS]->IsWaitingForAGV())S.AffectedActors.Add(Equipment[YardCraneCount+J.STS]->GetName());
            }
        }
    }
}

TSharedPtr<FJsonObject> APortSiteLogistics::CaptureIncidentTiming() const
{
    auto O=MakeShared<FJsonObject>();O->SetNumberField(TEXT("schema_version"),1);
    O->SetNumberField(TEXT("clock_seconds"),IncidentMeasureClock);O->SetNumberField(TEXT("halt_seconds"),IncidentHaltSeconds);
    O->SetStringField(TEXT("comparison_key"),IncidentComparisonKey);
    O->SetBoolField(TEXT("reference_eligible"),bIncidentReferenceClean&&!bIncidentInputUsed&&Fault.IsEmpty());
    if(const auto* Controller=Cast<AQuayCrane>(GetOwner()))O->SetNumberField(TEXT("environment_seed"),Controller->GetEnvironment()->InitializedSeed);
    TArray<TSharedPtr<FJsonValue>> Cargo;
    for(const auto& C:Manifest)
    {
        auto Row=MakeShared<FJsonObject>();const FString ID=FString::Printf(TEXT("C%d"),C.ID);
        Row->SetStringField(TEXT("id"),ID);Row->SetStringField(TEXT("actor_id"),GetNameSafe(C.Actor.Get()));
        const auto* End=IncidentCompletionTimes.Find(ID);Row->SetNumberField(TEXT("completed_at"),End?*End:-1);
        Cargo.Add(MakeShared<FJsonValueObject>(Row));
    }
    O->SetArrayField(TEXT("cargo"),Cargo);
    return O;
}
