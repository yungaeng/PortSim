#if WITH_DEV_AUTOMATION_TESTS
#include "AGVDynamics.h"
#include "PortAGVActor.h"
#include "PortContainerActor.h"
#include "Components/StaticMeshComponent.h"
#include "Misc/AutomationTest.h"
#include "Misc/Paths.h"
#include "Misc/FileHelper.h"
#include "HAL/FileManager.h"
#include "Dom/JsonObject.h"
#include "Serialization/JsonSerializer.h"
#include "Serialization/JsonWriter.h"
#include "Engine/World.h"
#include <limits>

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAGVDynamicsTest,"PortSim.AGVPhysics.TireAndStability",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FAGVDynamicsTest::RunTest(const FString& Parameters)
{
    FAGVDynamicsSettings C; FAGVDynamicsInput I; I.PayloadKg=30000;
    FAGVDynamicsState D; D.RefreshLoads(C,I);
    TestTrue(TEXT("Six wheel normal loads conserve total weight"),FMath::IsNearlyEqual(D.LeftLoadN+D.RightLoadN,70000*PortFriction::Gravity,.001));
    TestTrue(TEXT("Combined CoG includes empty chassis and cargo height above road"),FMath::IsNearlyEqual(D.CombinedCoGM.Z,(40000+30000*3.495)/70000.,1.e-6));
    I.CargoCoGM=FVector(1,.8,4.5); D.RefreshLoads(C,I);
    TestTrue(TEXT("Forward and left CoG raise corresponding loads"),D.LeftLoadN>D.RightLoadN && D.FrontLoadN>D.RearLoadN);
    I.CurvaturePerM=.125;
    D.RefreshLoads(C,I); const double High=D.StabilityLimitMps; I.CargoCoGM=FVector(0,0,3.495); D.RefreshLoads(C,I);
    TestTrue(TEXT("High eccentric CoG lowers allowable corner speed"),High<D.StabilityLimitMps);
    D.Roll=.3; D.RefreshLoads(C,I);
    TestTrue(TEXT("Spring reaction can unload inside wheels; tire forces cannot use negative normal load"),D.WheelLift && D.LeftLoadN>=0 && D.RightLoadN>=0);
    D.Reset(); I=FAGVDynamicsInput(); I.TargetSpeedMps=4;
    for(int J=0;J<240;++J) D.Step(C,I,1./120.);
    TestTrue(TEXT("Drive torque accelerates chassis without commanded side motion"),D.U>1 && FMath::Abs(D.V)<1.e-7 && FMath::Abs(D.PositionM.Y)<1.e-7);
    TestTrue(TEXT("Wheel rotation is a separate state"),FMath::Abs(D.Wheels[0].Omega)>0);
    const auto Paused=D; D.Step(C,I,0);
    TestEqual(TEXT("Paused clock freezes tire integration"),D.PositionM,Paused.PositionM);
    I.StaticMu=.08; I.KineticMu=.05; C.bTractionControl=false; I.AccelerationLimitMps2=3;
    D.Reset(); for(int J=0;J<240;++J) D.Step(C,I,1./120.);
    TestTrue(TEXT("Excess drive torque produces real wheel spin on ice"),D.PeakSlip>.1 && FMath::Abs(D.WheelDistanceM)>D.GroundDistanceM);
    for(const auto& W:D.Wheels)
        TestTrue(TEXT("Combined longitudinal/lateral forces stay inside tire friction circle"),FMath::Sqrt(W.FxN*W.FxN+W.FyN*W.FyN)<=I.StaticMu*W.NormalN+1);
    C=FAGVDynamicsSettings(); I=FAGVDynamicsInput(); I.TargetSpeedMps=-2;
    D.Reset(); for(int J=0;J<240;++J) D.Step(C,I,1./120.);
    TestTrue(TEXT("Reverse drive follows the longitudinal axis"),D.U<-.5 && FMath::Abs(D.V)<1.e-7);
    FAGVDynamicsState Small,Large; I.TargetSpeedMps=3; I.CurvaturePerM=.03;
    for(int J=0;J<120;++J) Small.Step(C,I,1./120.);
    for(int J=0;J<30;++J) Large.Step(C,I,1./30.);
    TestTrue(TEXT("Fixed-substep physics is repeatable across playback grouping"),FVector::Dist(Small.PositionM,Large.PositionM)<.01 && FMath::Abs(Small.U-Large.U)<.01);
    Small.Braking=true; Small.ConstrainStop();
    TestTrue(TEXT("Reservation freeze is not claimed as measured tire braking"),!Small.LastBrakeValid && !Small.Braking && Small.U==0);
    auto Bad=C; Bad.TrackM=0; Bad.EmptyCoGM.Z=std::numeric_limits<double>::quiet_NaN(); Bad.Sanitize();
    TestTrue(TEXT("Malformed editable geometry is sanitized"),Bad.TrackM>=1 && !Bad.EmptyCoGM.ContainsNaN());
    I.TargetSpeedMps=std::numeric_limits<double>::quiet_NaN(); Small.Step(C,I,.1);
    TestTrue(TEXT("Non-finite input is rejected without moving"),!Small.Fault.IsEmpty() && Small.Finite());

    const auto Init=UWorld::InitializationValues().AllowAudioPlayback(false).CreatePhysicsScene(true).RequiresHitProxies(false).CreateNavigation(false).CreateAISystem(false);
    auto* World=UWorld::CreateWorld(EWorldType::Game,false,NAME_None,nullptr,true,ERHIFeatureLevel::Num,&Init);
    if(!TestNotNull(TEXT("AGV physics world"),World)) return false;
    auto* A=World->SpawnActor<APortAGVActor>(); A->ResetVehicle(FVector::ZeroVector);
    A->SetPayload(30000); A->DeckCoGOffsetCm=FVector(80,100,400);
    const auto Body=A->DynamicsInput();
    TestTrue(TEXT("Actor +Y forward / -X left converted once into physics axes"),Body.CargoCoGM.Equals(FVector(1,-.8,4),1.e-6));
    A->DeckCoGOffsetCm=FVector(0,0,349.5);
    A->AdvanceDynamics(3,0,1);
    TestTrue(TEXT("The live actor actually moves from tire forces"),A->GetActorLocation().Y>10 && FMath::Abs(A->GetActorLocation().X)<.01);
    A->AdvanceDynamics(3,.1,.1);
    TestTrue(TEXT("Steering observation comes from actual front contacts, not a bicycle command"),
        FMath::IsNearlyEqual(double(A->Sensors.SteeringAngleDegrees),FMath::RadiansToDegrees((A->Dynamics.Wheels[4].Steer+A->Dynamics.Wheels[5].Steer)*.5),1.e-4));
    auto* Cargo=World->SpawnActor<APortContainerActor>(); Cargo->GetBody()->SetSimulatePhysics(false);
    Cargo->AttachToActor(A,FAttachmentTransformRules::KeepRelativeTransform); Cargo->SetActorRelativeLocation(FVector(0,400,349.5));
    A->SetActorLocationAndRotation(FVector::ZeroVector,FRotator::ZeroRotator);
    A->Dynamics.U=2; A->Dynamics.V=1; A->Dynamics.YawRate=.1; A->Dynamics.CombinedCoGM=FVector(1,.5,2);
    // World CoG=(-50,100) cm, cargo=(0,400), v_CG=(-100,200),
    // omega x arm=(-30,5). This must not use the old bicycle encoder proxy.
    TestTrue(TEXT("Attached cargo velocity includes actual translation and yaw about eccentric CoG"),
        Cargo->GetMotionVelocity().Equals(FVector(-130,205,0),1.e-4));
    A->ResetVehicle(FVector::ZeroVector);
    TestTrue(TEXT("Reset clears slip, wheel spin, load and braking history"),A->Dynamics.U==0 && A->Dynamics.PeakSlip==0 && !A->Dynamics.LastBrakeValid && A->PayloadKg==0);
    World->DestroyWorld(false); return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAGVBenchTest,"PortSim.AGVPhysics.ReproducibleBenchmarks",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FAGVBenchTest::RunTest(const FString& Parameters)
{
    FString Text; TSharedPtr<FJsonObject> Config;
    if(!FFileHelper::LoadFileToString(Text,*(FPaths::ProjectConfigDir()/TEXT("AGV_PhysicsBench.json"))) ||
        !FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text),Config))
        { AddError(TEXT("AGV_PhysicsBench.json missing or invalid")); return false; }
    const TArray<TSharedPtr<FJsonValue>>* Cases=nullptr;
    if(!Config->TryGetArrayField(TEXT("cases"),Cases) || Cases->IsEmpty()) { AddError(TEXT("Bench cases missing")); return false; }
    FPortFrictionSettings Friction; Friction.Load(FPaths::ProjectConfigDir()/TEXT("Weather_Friction.json"));
    TArray<TSharedPtr<FJsonValue>> Results; TMap<FString,double> BrakeDistances; bool AllValid=true;
    for(const auto& Case:*Cases)
    {
        const auto P=Case->AsObject();
        if(!P.IsValid()) { AddError(TEXT("Bench case must be an object")); AllValid=false; continue; }
        const TCHAR* Numbers[]={TEXT("wetness"),TEXT("ice"),TEXT("payload_kg"),TEXT("initial_speed_mps"),TEXT("target_speed_mps"),TEXT("radius_m"),TEXT("seconds"),TEXT("brake_demand_mps2")};
        bool Valid=true; for(const auto* Key:Numbers) { double N; Valid &= P->TryGetNumberField(Key,N) && FMath::IsFinite(N); }
        const TArray<TSharedPtr<FJsonValue>>* CoG=nullptr; FString ID,Kind; bool Stability=false,ABS=false;
        Valid &= P->TryGetStringField(TEXT("id"),ID) && P->TryGetStringField(TEXT("kind"),Kind) &&
            P->TryGetBoolField(TEXT("stability_control"),Stability) && P->TryGetBoolField(TEXT("abs"),ABS) &&
            P->TryGetArrayField(TEXT("cargo_cog_m"),CoG) && CoG->Num()==3;
        if(!Valid) { AddError(TEXT("Incomplete bench case")); AllValid=false; continue; }
        FAGVDynamicsSettings C; C.bABS=ABS; C.bStabilitySpeedControl=Stability;
        FAGVDynamicsInput I; I.EmptyMassKg=Friction.AGVEmptyMassKg; I.PayloadKg=P->GetNumberField(TEXT("payload_kg"));
        I.CargoCoGM=FVector((*CoG)[0]->AsNumber(),(*CoG)[1]->AsNumber(),(*CoG)[2]->AsNumber());
        I.TargetSpeedMps=P->GetNumberField(TEXT("target_speed_mps")); const double Radius=P->GetNumberField(TEXT("radius_m"));
        I.CurvaturePerM=FMath::Abs(Radius)>.001?1/Radius:0; I.BrakeDemandMps2=P->GetNumberField(TEXT("brake_demand_mps2"));
        I.RollingResistance=Friction.TireRollingResistance; I.DrivenFraction=Friction.DrivenWeightFraction;
        FPortSurfaceState Surface; Surface.Wetness=P->GetNumberField(TEXT("wetness")); Surface.Ice=P->GetNumberField(TEXT("ice")); Surface.Sanitize();
        I.StaticMu=Friction.Tire.Static(Surface); I.KineticMu=Friction.Tire.Kinetic(Surface);
        const double Duration=P->GetNumberField(TEXT("seconds")),Initial=P->GetNumberField(TEXT("initial_speed_mps"));
        if(Duration<=0 || Duration>120 || I.PayloadKg<0 || I.PayloadKg>65000 || FMath::Abs(Initial)>7 ||
            FMath::Abs(I.TargetSpeedMps)>7 || (Radius!=0 && FMath::Abs(Radius)<5.8) || I.CargoCoGM.ContainsNaN() || I.CargoCoGM.Z<0)
        { AddError(TEXT("Bench case outside modeled range")); AllValid=false; continue; }
        FAGVDynamicsState D; D.Reset(Initial,C); TArray<TSharedPtr<FJsonValue>> Samples;
        double Elapsed=0; bool ContactLimit=false;
        for(int J=0;J<FMath::CeilToInt(Duration*120);++J)
        {
            D.Step(C,I,1./120.); Elapsed=(J+1)/120.;
            ContactLimit|=D.OverturnThreshold;
            if(J%12==0 || D.LastBrakeValid || ContactLimit || !D.Fault.IsEmpty())
            {
                auto S=MakeShared<FJsonObject>(); S->SetNumberField(TEXT("time_s"),Elapsed);
                S->SetNumberField(TEXT("speed_mps"),D.U); S->SetNumberField(TEXT("distance_m"),D.GroundDistanceM);
                S->SetNumberField(TEXT("slip_ratio"),D.MaxSlip); S->SetNumberField(TEXT("ltr"),D.LoadTransferRatio);
                S->SetNumberField(TEXT("roll_deg"),FMath::RadiansToDegrees(D.Roll));
                S->SetNumberField(TEXT("lateral_mps2"),D.Ay); S->SetNumberField(TEXT("left_kn"),D.LeftLoadN*.001); S->SetNumberField(TEXT("right_kn"),D.RightLoadN*.001);
                Samples.Add(MakeShared<FJsonValueObject>(S));
            }
            if(ContactLimit || !D.Fault.IsEmpty() || (Kind==TEXT("brake") && D.LastBrakeValid)) break;
        }
        const bool Passed=D.Finite() && D.Fault.IsEmpty() && (Kind!=TEXT("brake") || D.LastBrakeValid);
        TestTrue(*FString::Printf(TEXT("Bench %s finite; braking stops or contact threshold ends the turn"),*ID),Passed); AllValid &= Passed;
        auto R=MakeShared<FJsonObject>(); R->SetObjectField(TEXT("input"),P); R->SetBoolField(TEXT("valid"),Passed);
        R->SetBoolField(TEXT("contact_threshold"),ContactLimit); R->SetBoolField(TEXT("brake_completed"),D.LastBrakeValid);
        R->SetNumberField(TEXT("mu_static"),I.StaticMu); R->SetNumberField(TEXT("mu_kinetic"),I.KineticMu);
        R->SetNumberField(TEXT("elapsed_s"),Elapsed); R->SetNumberField(TEXT("peak_slip"),D.PeakSlip); R->SetNumberField(TEXT("peak_ltr"),D.PeakLTR);
        R->SetNumberField(TEXT("final_speed_mps"),D.U); R->SetNumberField(TEXT("braking_distance_m"),D.LastBrakingDistanceM);
        R->SetNumberField(TEXT("braking_seconds"),D.LastBrakingSeconds); R->SetArrayField(TEXT("samples"),Samples);
        Results.Add(MakeShared<FJsonValueObject>(R));
        if(Kind==TEXT("brake")) BrakeDistances.Add(ID,D.LastBrakingDistanceM);
    }
    if(BrakeDistances.Contains(TEXT("dry_brake")) && BrakeDistances.Contains(TEXT("wet_brake")) && BrakeDistances.Contains(TEXT("ice_brake")))
        TestTrue(TEXT("Equal initial speed/load/torque: dry < wet < ice braking distance"),
            BrakeDistances[TEXT("dry_brake")]<BrakeDistances[TEXT("wet_brake")] && BrakeDistances[TEXT("wet_brake")]<BrakeDistances[TEXT("ice_brake")]);
    auto Report=MakeShared<FJsonObject>(); Report->SetNumberField(TEXT("schema_version"),1);
    Report->SetStringField(TEXT("generated_at"),FDateTime::UtcNow().ToIso8601()); Report->SetBoolField(TEXT("valid"),AllValid && !HasAnyErrors());
    Report->SetStringField(TEXT("model"),TEXT("six_contact_planar_tire_roll_pitch")); Report->SetArrayField(TEXT("cases"),Results);
    FString Json; FJsonSerializer::Serialize(Report,TJsonWriterFactory<>::Create(&Json));
    const FString Directory=FPaths::ProjectSavedDir()/TEXT("Tests"); IFileManager::Get().MakeDirectory(*Directory,true);
    TestTrue(TEXT("Bench report saved"),FFileHelper::SaveStringToFile(Json,*(Directory/TEXT("AGVDynamics_Report.json")),FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM));
    return !HasAnyErrors();
}
#endif
