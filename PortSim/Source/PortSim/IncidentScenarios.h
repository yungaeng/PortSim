#pragma once
#include "CoreMinimal.h"

class FJsonObject;
class AActor;
class APortContainerActor;
namespace PortIncident
{
    inline constexpr double SensorRestartSeconds=30;
    inline constexpr double SensorServiceSeconds=900;
    inline bool SensorFailure(const FString& Action)
    { return Action==TEXT("fail_reboot")||Action==TEXT("fail_sensor_repair"); }
    inline bool ControllerFailure(const FString& Action)
    { return Action==TEXT("equipment_failure")||Action==TEXT("restart_controller"); }
    inline double RecoverySeconds(const FString& Action)
    {
        if(Action==TEXT("fail")||Action==TEXT("fail_release")) return 1800;
        if(Action==TEXT("equipment_failure")||Action==TEXT("fail_pickup")) return 1200;
        if(Action==TEXT("restart_controller")) return 120;
        if(Action==TEXT("fail_gantry")) return 3600;
        if(Action==TEXT("fail_hoist")) return 7200;
        if(Action==TEXT("fail_reboot")) return SensorRestartSeconds;
        if(Action==TEXT("fail_sensor_repair")) return SensorServiceSeconds;
        if(Action==TEXT("misdelivery")||Action==TEXT("fail_placement")||
            Action==TEXT("slot_occupied")||Action==TEXT("height_mismatch")) return 300;
        if(Action==TEXT("fms_disconnect")||Action==TEXT("agv_unavailable")) return 60;
        if(Action==TEXT("localization_loss")||Action==TEXT("fms_delay")) return 30;
        return 0;
    }
    inline bool AGVExtension(const FString& A)
    {return A==TEXT("localization_loss")||A==TEXT("fms_delay")||A==TEXT("fms_disconnect")||A==TEXT("road_block");}
    inline bool CraneExtension(const FString& A,bool RMG)
    {return ControllerFailure(A)||A==TEXT("fail_release")||(!RMG&&(A==TEXT("fail_hoist")||A==TEXT("agv_unavailable")))||(RMG&&(A==TEXT("slot_occupied")||A==TEXT("height_mismatch")));}
    inline bool Extension(const FString& A){return AGVExtension(A)||CraneExtension(A,false)||CraneExtension(A,true);}
}
struct FPortIncidentRoadClosure
{
    FBox Bounds=FBox(ForceInit);
    TWeakObjectPtr<AActor> Cause;
    FVector Center=FVector::ZeroVector, Longitudinal=FVector::ZeroVector;
    double HalfLength=0,HalfWidth=0,Clearance=0,ExpectedClearAt=-1,NextDispatchAttempt=0;
    bool Active=true,Responded=false;
    bool bDriveFaultZone=false;
    TWeakObjectPtr<AActor> Marker;
};
struct FPortIncidentStep
{
    TSharedPtr<FJsonObject> Command;
    FString State=TEXT("pending"),Message;
    double Delay=0,Duration=0,Timeout=1800,AppliedAt=-1,RecoveredAt=-1;
    double MeasureAppliedAt=-1,MeasureClearedAt=-1,MeasureResumedAt=-1,MeasureCompletedAt=-1;
    TSet<FString> AffectedActors,AffectedCargo;
    int32 ResumeStage=INDEX_NONE,ResumeCompleted=0;
    int32 CompletedBefore=0;
    bool Observed=false;
    bool ExpectShip=false;
    TWeakObjectPtr<APortContainerActor> Cargo;
};
struct FPortIncidentRun
{
    FString ID,Name;
    double StartedAt=0;
    bool Cancelled=false;
    TArray<FPortIncidentStep> Steps;
};
