#include "PortAGVActor.h"
#include "PortContainerActor.h"
#include "AGVReference.h"
#include "Components/StaticMeshComponent.h"
#include "Components/TextRenderComponent.h"
#include "Dom/JsonObject.h"

bool APortAGVActor::StartEmergencyScenario(bool Secured,FString& Error)
{
    if(Emergency.Active || bDriveFault || !Dynamics.Fault.IsEmpty() || !EmergencySettings.Valid() ||
        Speed<100 || Sensors.bFMSHold || Sensors.bControlledStop || FMath::Abs(Dynamics.YawRate)>.02 || FMath::Abs(LastCurvaturePerCm)>.00001)
    { Error=TEXT("정상 주행 중인 직선 구간의 AGV를 선택하세요(1m/s 이상). 진행 중 시험·고장·대기 상태에서는 시작할 수 없습니다.");return false; }
    TArray<AActor*> Attached;GetAttachedActors(Attached); APortContainerActor* Box=nullptr;
    for(auto* A:Attached)if(auto* C=Cast<APortContainerActor>(A);C && C->LocationOwner==ECargoOwner::AGV)
    { if(Box){Error=TEXT("통합 시험은 40ft 화물 한 개를 운반할 때 시작하세요.");return false;}Box=C; }
    if(!Box || Box->LengthFt!=40){Error=TEXT("AGV에 실제로 적재된 40ft 화물이 필요합니다.");return false;}
    const FVector BodyCoG=EmergencySettings.CargoCoGOffsetM;
    const FVector WorldOffset=GetActorRightVector()*BodyCoG.X*100-GetActorForwardVector()*BodyCoG.Y*100+FVector::UpVector*BodyCoG.Z*100;
    const FVector BoxOffset=Box->GetActorQuat().UnrotateVector(WorldOffset);
    if(BoxOffset.GetAbs().X>Box->ExternalDimensionsCm.X*.49 || BoxOffset.GetAbs().Y>Box->ExternalDimensionsCm.Y*.49 ||
        BoxOffset.GetAbs().Z>Box->ExternalDimensionsCm.Z*.49)
    {Error=TEXT("시험 CoG가 컨테이너 내부 범위를 벗어났습니다.");return false;}
    EmergencyCargo=Box;OriginalEmergencyCoG=Box->CoGOffsetCm;AppliedEmergencyCoG=BoxOffset;Box->CoGOffsetCm=BoxOffset;
    DeckCoGOffsetCm=GetActorTransform().InverseTransformPosition(Box->GetActorLocation()+WorldOffset);
    EmergencyDirection=(Sensors.bReversing?-1:1)*GetActorRightVector();
    EmergencyWorkerPosition=GetActorLocation()+EmergencyDirection*(AGVReference::PhysicalHalfLengthCm+
        (EmergencySettings.WorkerDistanceM+EmergencySettings.WorkerRadiusM)*100);
    Dynamics.CargoOffsetM=Dynamics.CargoRelativeVelocityMps=FVector::ZeroVector;
    Dynamics.PeakCargoShiftM=0;Dynamics.CargoUnsafe=Dynamics.RestraintOverload=false;
    Emergency.Start(Dynamics.U,EmergencySettings.WorkerDistanceM,Secured);
    WorkerMarker->SetVisibility(true);WorkerLabel->SetVisibility(true);
    WorkerMarker->SetWorldLocation(EmergencyWorkerPosition+FVector(0,0,90));
    WorkerLabel->SetWorldLocation(EmergencyWorkerPosition+FVector(0,0,210));
    return true;
}

bool APortAGVActor::ClearEmergencyScenario(FString& Error)
{
    if(!Emergency.Active){Error=TEXT("진행 중인 돌발상황 시험이 없습니다.");return false;}
    if(!Emergency.Stopped || Emergency.Collision || Dynamics.CargoUnsafe || Dynamics.PeakCargoShiftM>.001)
    {Error=TEXT("안전 정지·화물 위치가 유지된 시험만 해제할 수 있습니다. 충돌·화물 이동·해석 한계 초과 시 시뮬레이션을 초기화하세요.");return false;}
    if(EmergencyCargo.IsValid())
    {
        EmergencyCargo->CoGOffsetCm=OriginalEmergencyCoG;
        DeckCoGOffsetCm=GetActorTransform().InverseTransformPosition(EmergencyCargo->GetActorLocation()+
            EmergencyCargo->GetActorQuat().RotateVector(OriginalEmergencyCoG));
    }
    // Preserve the just-completed record for the UI until the next test/reset.
    Emergency.Active=false;EmergencyCargo.Reset();WorkerMarker->SetVisibility(false);WorkerLabel->SetVisibility(false);
    Sensors.bControlledStop=false;Sensors.bLiDARClear=true;Sensors.bFMSHold=false;
    return true;
}

bool APortAGVActor::HasValidEmergencyCoG(const APortContainerActor* Cargo,FVector Baseline) const
{
    return Emergency.Active && EmergencyCargo.Get()==Cargo && Cargo && Cargo->LocationOwner==ECargoOwner::AGV &&
        Cargo->GetAttachParentActor()==this && OriginalEmergencyCoG.Equals(Baseline) && Cargo->CoGOffsetCm.Equals(AppliedEmergencyCoG);
}

void APortAGVActor::AdvanceEmergencyScenario(double Seconds)
{
    if(!Emergency.Active || Emergency.Finished || Seconds<=0)return;
    const int32 Steps=FMath::CeilToInt(Seconds*120);const double Dt=Seconds/Steps;
    for(int32 J=0;J<Steps && !Emergency.Finished;++J)
    {
        const FVector Before=GetActorLocation(),OldOffset=Dynamics.CargoOffsetM;
        AdvanceDynamics(0,0,Dt);
        const FVector Shift=Dynamics.CargoOffsetM-OldOffset;
        if(EmergencyCargo.IsValid())EmergencyCargo->AddActorWorldOffset((GetActorRightVector()*Shift.X-GetActorForwardVector()*Shift.Y)*100);
        // Keep the cargo CoG used by the next substep consistent with its visual pose.
        DeckCoGOffsetCm+=FVector(-Shift.Y,Shift.X,0)*100;
        const FVector Delta=(EmergencyWorkerPosition-GetActorLocation())*.01;
        const FVector Motion=EmergencyDirection;
        const double Gap=FVector::DotProduct(Delta,Motion)-AGVReference::PhysicalHalfLengthCm*.01-EmergencySettings.WorkerRadiusM;
        const double Side=FMath::Abs(FVector::CrossProduct(Motion,Delta).Z);
        Emergency.Record(Dynamics,Side<=2.1+EmergencySettings.WorkerRadiusM?Gap:Emergency.MinimumClearanceM,FVector::Dist2D(Before,GetActorLocation())*.01);
        Sensors.FrontObstacleDistanceCm=Sensors.bReversing?-1:FMath::Max(0.,Gap)*100;
        Sensors.RearObstacleDistanceCm=Sensors.bReversing?FMath::Max(0.,Gap)*100:-1;
        Sensors.bControlledStop=Emergency.BrakeCommand;Sensors.bLiDARClear=!Emergency.Detected;
        WorkerMarker->SetWorldLocation(EmergencyWorkerPosition+FVector(0,0,90));
        WorkerLabel->SetWorldLocation(EmergencyWorkerPosition+FVector(0,0,210));
    }
}

TSharedRef<FJsonObject> APortAGVActor::EmergencySnapshot() const
{
    auto O=MakeShared<FJsonObject>();const auto& E=Emergency;
    O->SetBoolField(TEXT("active"),E.Active);O->SetBoolField(TEXT("detected"),E.Detected);O->SetBoolField(TEXT("brake_command"),E.BrakeCommand);
    O->SetBoolField(TEXT("finished"),E.Finished);O->SetBoolField(TEXT("stopped"),E.Stopped);O->SetBoolField(TEXT("collision"),E.Collision);
    O->SetBoolField(TEXT("invalidated"),E.Invalidated);O->SetBoolField(TEXT("cargo_secured"),E.CargoSecured);O->SetStringField(TEXT("outcome"),E.Outcome);
    O->SetNumberField(TEXT("elapsed_s"),E.Elapsed);O->SetNumberField(TEXT("detected_at_s"),E.DetectedAt);O->SetNumberField(TEXT("brake_at_s"),E.BrakeAt);
    O->SetNumberField(TEXT("stopped_at_s"),E.StoppedAt);O->SetNumberField(TEXT("braking_distance_m"),E.BrakingDistanceM);
    O->SetNumberField(TEXT("total_travel_m"),E.TravelM);O->SetNumberField(TEXT("minimum_clearance_m"),E.MinimumClearanceM);
    O->SetNumberField(TEXT("peak_inertia_n"),E.PeakInertiaN);O->SetNumberField(TEXT("peak_restraint_n"),E.PeakRestraintN);
    O->SetNumberField(TEXT("peak_cargo_shift_m"),E.PeakCargoShiftM);O->SetNumberField(TEXT("peak_slip"),E.PeakSlip);O->SetNumberField(TEXT("peak_ltr"),E.PeakLTR);
    O->SetNumberField(TEXT("wetness_override"),EmergencySettings.Wetness);O->SetNumberField(TEXT("ice_override"),EmergencySettings.Ice);
    O->SetStringField(TEXT("basis"),TEXT("local surface/CoG test override; geometric stationary worker; assumed scan/reaction/brake/guide parameters"));
    TArray<TSharedPtr<FJsonValue>> Position;for(double N:{EmergencyWorkerPosition.X*.01,EmergencyWorkerPosition.Y*.01,EmergencyWorkerPosition.Z*.01})Position.Add(MakeShared<FJsonValueNumber>(N));
    O->SetArrayField(TEXT("worker_position_m"),Position);return O;
}
