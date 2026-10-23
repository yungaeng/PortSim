#if WITH_DEV_AUTOMATION_TESTS
#include "AGVEmergency.h"
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

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAGVEmergencyPhysicsTest,"PortSim.AGVPhysics.Emergency.CoupledInvariants",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FAGVEmergencyPhysicsTest::RunTest(const FString& Parameters)
{
    FAGVDynamicsSettings C;C.bABS=false;FAGVDynamicsInput I;I.PayloadKg=30000;I.CargoContactEnabled=true;
    I.CargoSecured=false;I.DeckStaticMu=I.DeckKineticMu=0;I.StaticMu=I.KineticMu=I.RollingResistance=0;
    I.WindForceBodyN=FVector(-70000,0,0);I.TargetSpeedMps=2;I.CargoShiftLimitM=1;
    FAGVDynamicsState D;D.Reset(2,C);
    for(int J=0;J<120;++J)D.Step(C,I,1./120.);
    TestTrue(TEXT("Internal deck forces preserve system linear momentum"),FMath::IsNearlyEqual(D.U,1.,.01));
    TestTrue(TEXT("Frictionless cargo keeps its inertial trajectory while chassis slows"),FMath::Abs(D.PositionM.X+D.CargoOffsetM.X-2.)<.03);
    TestTrue(TEXT("Sliding cargo changes the combined CoG and chassis motion"),D.CargoOffsetM.X>.6 && D.PositionM.X<1.3);
    I.CargoSecured=true;I.RestraintCapacityN=1.e6;D.Reset(2,C);D.Step(C,I,.5);
    TestTrue(TEXT("A secured cargo does not slide even when deck friction is zero"),D.CargoOffsetM.IsZero() && D.RestraintForceN.Size2D()>1000);
    FAGVEmergencySettings E;FAGVEmergencyState S;S.Start(6,20,true);D.Reset(6,C);I=FAGVDynamicsInput();I.PayloadKg=30000;
    const auto Initial=D;S.Step(E,C,I,D,20,0,100,0);
    TestTrue(TEXT("Paused scenario freezes wheel and perception clocks"),S.Elapsed==0 && D.PositionM==Initial.PositionM);
    for(int J=0;J<29;++J)S.Step(E,C,I,D,20,0,100,1./120.);
    TestTrue(TEXT("Detection occurs on scan and physical brake waits for reaction delay"),S.Detected && !S.BrakeCommand && D.U>5);
    for(int J=0;J<3;++J)S.Step(E,C,I,D,20,0,100,1./120.);
    TestTrue(TEXT("Brake command does not zero velocity"),S.BrakeCommand && D.U>1);
    S.Start(6,20,true);D.Reset(6,C);for(int J=0;J<120;++J)S.Step(E,C,I,D,20,10,100,1./120.);
    TestFalse(TEXT("A worker outside the leading swept width is not detected"),S.Detected);

    const auto Init=UWorld::InitializationValues().AllowAudioPlayback(false).CreatePhysicsScene(true).RequiresHitProxies(false).CreateNavigation(false).CreateAISystem(false);
    auto* World=UWorld::CreateWorld(EWorldType::Game,false,NAME_None,nullptr,true,ERHIFeatureLevel::Num,&Init);
    if(!TestNotNull(TEXT("Emergency actor integration world"),World))return false;
    auto* A=World->SpawnActor<APortAGVActor>();A->ResetVehicle(FVector::ZeroVector);
    auto* Box=World->SpawnActor<APortContainerActor>();Box->GetBody()->SetSimulatePhysics(false);
    Box->SetActorLocation(A->CargoPosition());Box->AttachToActor(A,FAttachmentTransformRules::KeepWorldTransform);Box->LocationOwner=ECargoOwner::AGV;
    A->SetPayload(Box->MassKg);A->DeckCoGOffsetCm=FVector(0,0,349.5);A->AdvanceDynamics(6,0,4);
    const FVector OriginalCoG=Box->CoGOffsetCm;FString Error;
    TestTrue(TEXT("Loaded live AGV accepts a geometric worker test"),A->StartEmergencyScenario(true,Error));
    TestTrue(TEXT("Only the explicit test cargo and original booking match the CoG override"),A->HasValidEmergencyCoG(Box,OriginalCoG));
    TestFalse(TEXT("The exception cannot mask a different booking CoG"),A->HasValidEmergencyCoG(Box,OriginalCoG+FVector(1,0,0)));
    const FVector Before=A->GetActorLocation();A->AdvanceEmergencyScenario(.1);
    TestTrue(TEXT("Real actor keeps moving during perception delay"),FVector::Dist2D(Before,A->GetActorLocation())>1 && A->Speed>0);
    for(int J=0;J<7200 && !A->Emergency.Finished;++J)A->AdvanceEmergencyScenario(1./120.);
    TestTrue(TEXT("Live actor physically stops ahead of worker with valid tire braking"),A->Emergency.Stopped && !A->Emergency.Collision && A->Dynamics.LastBrakeValid);
    TestFalse(TEXT("Stopped scenario cannot accept new fleet work before explicit clearance"),A->CanAcceptWork());
    TestTrue(TEXT("Clearing safe secured test restores original cargo CoG"),A->ClearEmergencyScenario(Error) && Box->CoGOffsetCm==OriginalCoG);
    TestTrue(TEXT("Completion record remains visible after release"),!A->Emergency.Active && A->Emergency.Stopped);
    A->AdvanceDynamics(6,0,4);TestTrue(TEXT("Second live scenario starts"),A->StartEmergencyScenario(false,Error));
    A->SetFMSHold(true);TestTrue(TEXT("An external nonphysical freeze invalidates the result"),A->Emergency.Invalidated && !A->Emergency.Stopped);
    TestFalse(TEXT("Invalid test cannot resume through ordinary clearance"),A->ClearEmergencyScenario(Error));
    A->ResetVehicle(FVector::ZeroVector);TestEqual(TEXT("Reset restores original CoG after unsafe test"),Box->CoGOffsetCm,OriginalCoG);
    World->DestroyWorld(false);return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAGVEmergencyBenchTest,"PortSim.AGVPhysics.Emergency.ReproducibleScenarios",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FAGVEmergencyBenchTest::RunTest(const FString& Parameters)
{
    FString Text;TSharedPtr<FJsonObject> Root;
    if(!FFileHelper::LoadFileToString(Text,*(FPaths::ProjectConfigDir()/TEXT("AGV_EmergencyBench.json"))) ||
        !FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text),Root) || !Root.IsValid())
    {AddError(TEXT("Emergency bench configuration missing or malformed"));return false;}
    const TArray<TSharedPtr<FJsonValue>>* Cases=nullptr;const TSharedPtr<FJsonObject>* Defaults=nullptr;
    if(!Root->TryGetArrayField(TEXT("cases"),Cases) || Cases->IsEmpty() || !Root->TryGetObjectField(TEXT("defaults"),Defaults))
    {AddError(TEXT("Emergency bench cases/defaults missing"));return false;}
    FPortFrictionSettings Friction;Friction.Load(FPaths::ProjectConfigDir()/TEXT("Weather_Friction.json"));
    TArray<TSharedPtr<FJsonValue>> Results;TMap<FString,double> Distances,LTR,Shifts;
    for(const auto& Case:*Cases)
    {
        const auto Entry=Case->AsObject();if(!Entry.IsValid()){AddError(TEXT("Case must be an object"));continue;}
        auto P=MakeShared<FJsonObject>();P->Values=(*Defaults)->Values;for(const auto& V:Entry->Values)P->SetField(V.Key,V.Value);
        const TCHAR* Keys[]={TEXT("speed_mps"),TEXT("payload_kg"),TEXT("wetness"),TEXT("ice"),TEXT("worker_distance_m"),TEXT("lidar_range_m"),
            TEXT("scan_period_s"),TEXT("reaction_s"),TEXT("brake_demand_mps2"),TEXT("restraint_capacity_n"),TEXT("cargo_shift_limit_m"),TEXT("timeout_s")};
        bool Valid=true;for(const auto* K:Keys){double N;Valid&=P->TryGetNumberField(K,N)&&FMath::IsFinite(N);}
        FString ID,Expected;bool Secured=false;const TArray<TSharedPtr<FJsonValue>>* CoG=nullptr;
        Valid&=P->TryGetStringField(TEXT("id"),ID)&&P->TryGetStringField(TEXT("expected"),Expected)&&P->TryGetBoolField(TEXT("cargo_secured"),Secured)&&
            P->TryGetArrayField(TEXT("cargo_cog_m"),CoG)&&CoG->Num()==3;
        if(!Valid){AddError(TEXT("Incomplete emergency case"));continue;}
        FAGVEmergencySettings E;E.WorkerDistanceM=P->GetNumberField(TEXT("worker_distance_m"));E.ReactionSeconds=P->GetNumberField(TEXT("reaction_s"));
        E.ScanPeriodSeconds=P->GetNumberField(TEXT("scan_period_s"));E.BrakeDemandMps2=P->GetNumberField(TEXT("brake_demand_mps2"));
        E.RestraintCapacityN=P->GetNumberField(TEXT("restraint_capacity_n"));E.CargoShiftLimitM=P->GetNumberField(TEXT("cargo_shift_limit_m"));
        E.TimeoutSeconds=P->GetNumberField(TEXT("timeout_s"));E.Wetness=P->GetNumberField(TEXT("wetness"));E.Ice=P->GetNumberField(TEXT("ice"));
        FAGVDynamicsSettings C;FAGVDynamicsInput I;I.EmptyMassKg=Friction.AGVEmptyMassKg;I.PayloadKg=P->GetNumberField(TEXT("payload_kg"));
        I.CargoCoGM=FVector((*CoG)[0]->AsNumber(),(*CoG)[1]->AsNumber(),(*CoG)[2]->AsNumber());
        I.DrivenFraction=Friction.DrivenWeightFraction;I.RollingResistance=Friction.TireRollingResistance;
        FPortSurfaceState Surface;Surface.Wetness=E.Wetness;Surface.Ice=E.Ice;
        I.StaticMu=Friction.Tire.Static(Surface);I.KineticMu=Friction.Tire.Kinetic(Surface);
        I.DeckStaticMu=Friction.Steel.Static(Surface);I.DeckKineticMu=Friction.Steel.Kinetic(Surface);
        const double Speed=P->GetNumberField(TEXT("speed_mps")),Range=P->GetNumberField(TEXT("lidar_range_m"));
        if(!E.Valid() || Speed<=0 || Speed>7 || Range<=0 || Range>200 || I.PayloadKg<=0 || I.PayloadKg>65000 ||
            I.CargoCoGM.ContainsNaN() || I.CargoCoGM.Z<0 || FMath::Abs(I.CargoCoGM.Y)>1.2)
        {AddError(TEXT("Emergency case outside modeled range"));continue;}
        FAGVDynamicsState D;D.Reset(Speed,C);FAGVEmergencyState S;S.Start(Speed,E.WorkerDistanceM,Secured);
        const FVector Worker(E.WorkerDistanceM+7.3+E.WorkerRadiusM,0,0);TArray<TSharedPtr<FJsonValue>> Samples;
        auto Gap=[&](){const FVector Delta=Worker-D.PositionM;return Delta.X*FMath::Cos(D.Heading)+Delta.Y*FMath::Sin(D.Heading)-7.3-E.WorkerRadiusM;};
        for(int J=0;J<FMath::CeilToInt(E.TimeoutSeconds*120) && !S.Finished;++J)
        {
            const FVector Before=D.PositionM;const FVector Delta=Worker-D.PositionM;
            const double Side=-Delta.X*FMath::Sin(D.Heading)+Delta.Y*FMath::Cos(D.Heading);
            S.Step(E,C,I,D,Gap(),Side,Range,1./120.);S.Record(D,Gap(),FVector::Dist2D(Before,D.PositionM));
            if(J%12==0 || S.Finished)
            {
                auto V=MakeShared<FJsonObject>();V->SetNumberField(TEXT("time_s"),S.Elapsed);V->SetNumberField(TEXT("speed_mps"),FMath::Sqrt(D.U*D.U+D.V*D.V));
                V->SetNumberField(TEXT("gap_m"),S.GapM);V->SetBoolField(TEXT("detected"),S.Detected);V->SetBoolField(TEXT("brake"),S.BrakeCommand);
                V->SetNumberField(TEXT("slip"),D.MaxSlip);V->SetNumberField(TEXT("left_kn"),D.LeftLoadN*.001);V->SetNumberField(TEXT("right_kn"),D.RightLoadN*.001);
                V->SetNumberField(TEXT("cargo_shift_m"),D.CargoOffsetM.Size2D());V->SetNumberField(TEXT("deck_force_kn"),D.CargoContactForceN.Size2D()*.001);
                Samples.Add(MakeShared<FJsonValueObject>(V));
            }
        }
        const bool Passed=D.Finite() && D.Fault.IsEmpty() && S.Finished && S.Outcome==Expected && !S.Invalidated;
        TestTrue(*FString::Printf(TEXT("Emergency %s expected %s; actual %s"),*ID,*Expected,*S.Outcome),Passed);
        if(S.BrakeCommand)TestTrue(TEXT("Reaction delay is honored"),S.BrakeAt-S.DetectedAt+1.e-6>=E.ReactionSeconds);
        if(Secured)TestEqual(TEXT("Secured cargo does not slide"),D.PeakCargoShiftM,0.);
        auto R=MakeShared<FJsonObject>();R->SetObjectField(TEXT("input"),P);R->SetBoolField(TEXT("valid"),Passed);R->SetBoolField(TEXT("cargo_secured"),Secured);
        R->SetStringField(TEXT("outcome"),S.Outcome);R->SetBoolField(TEXT("stopped"),S.Stopped);R->SetBoolField(TEXT("collision"),S.Collision);
        R->SetNumberField(TEXT("mu_static"),I.StaticMu);R->SetNumberField(TEXT("mu_kinetic"),I.KineticMu);
        R->SetNumberField(TEXT("deck_mu_static"),I.DeckStaticMu);R->SetNumberField(TEXT("deck_mu_kinetic"),I.DeckKineticMu);
        R->SetNumberField(TEXT("detected_at_s"),S.DetectedAt);R->SetNumberField(TEXT("brake_at_s"),S.BrakeAt);R->SetNumberField(TEXT("stopped_at_s"),S.StoppedAt);
        R->SetNumberField(TEXT("braking_distance_m"),S.BrakingDistanceM);R->SetNumberField(TEXT("minimum_clearance_m"),S.MinimumClearanceM);
        R->SetNumberField(TEXT("peak_slip"),S.PeakSlip);R->SetNumberField(TEXT("peak_ltr"),S.PeakLTR);R->SetNumberField(TEXT("peak_cargo_shift_m"),S.PeakCargoShiftM);
        R->SetNumberField(TEXT("peak_inertia_n"),S.PeakInertiaN);R->SetNumberField(TEXT("peak_restraint_n"),S.PeakRestraintN);R->SetArrayField(TEXT("samples"),Samples);
        Results.Add(MakeShared<FJsonValueObject>(R));Distances.Add(ID,S.BrakingDistanceM);LTR.Add(ID,S.PeakLTR);Shifts.Add(ID,S.PeakCargoShiftM);
    }
    if(Distances.Contains(TEXT("dry_secured")) && Distances.Contains(TEXT("wet_secured")))
        TestTrue(TEXT("Wet road requires more braking travel under identical inputs"),Distances[TEXT("wet_secured")]>Distances[TEXT("dry_secured")]);
    if(LTR.Contains(TEXT("wet_centered")) && LTR.Contains(TEXT("wet_secured")))
        TestTrue(TEXT("Eccentric cargo creates more lateral load imbalance than centered cargo"),LTR[TEXT("wet_secured")]>LTR[TEXT("wet_centered")]);
    if(Shifts.Contains(TEXT("wet_unsecured")))TestTrue(TEXT("Unsecured wet cargo exhibits actual relative displacement"),Shifts[TEXT("wet_unsecured")]>.3);
    auto Report=MakeShared<FJsonObject>();Report->SetNumberField(TEXT("schema_version"),1);Report->SetStringField(TEXT("generated_at"),FDateTime::UtcNow().ToIso8601());
    Report->SetBoolField(TEXT("valid"),!HasAnyErrors());Report->SetStringField(TEXT("model"),TEXT("coupled_tire_cog_deck_worker"));Report->SetArrayField(TEXT("cases"),Results);
    FString Json;FJsonSerializer::Serialize(Report,TJsonWriterFactory<>::Create(&Json));const FString Directory=FPaths::ProjectSavedDir()/TEXT("Tests");
    IFileManager::Get().MakeDirectory(*Directory,true);TestTrue(TEXT("Integrated report saved"),FFileHelper::SaveStringToFile(Json,*(Directory/TEXT("AGVEmergency_Report.json")),FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM));
    return !HasAnyErrors();
}
#endif
