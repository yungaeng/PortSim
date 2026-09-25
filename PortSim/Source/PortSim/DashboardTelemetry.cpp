#include "PortSiteLogistics.h"
#include "PortWorkingCrane.h"
#include "PortContainerActor.h"
#include "PortAGVActor.h"
#include "Components/SceneComponent.h"
#include "Components/StaticMeshComponent.h"
#include "EngineUtils.h"
#include "Engine/World.h"
#include "Dom/JsonObject.h"
#include "Serialization/JsonSerializer.h"
#include "Serialization/JsonWriter.h"
#include "Misc/Paths.h"
#include "Misc/FileHelper.h"
#include "HAL/FileManager.h"
#include "HAL/PlatformTime.h"
#include "UObject/UnrealType.h"

namespace Dashboard
{
    using FObject=TSharedRef<FJsonObject>;
    FObject Object() { return MakeShared<FJsonObject>(); }
    TSharedPtr<FJsonValue> Value(FObject O) { return MakeShared<FJsonValueObject>(O); }
    void Vector(FObject O,const TCHAR* Key,FVector V,float Scale=1.f)
    {
        TArray<TSharedPtr<FJsonValue>> A;
        for(int32 I=0;I<3;++I) A.Add(MakeShared<FJsonValueNumber>(V[I]*Scale));
        O->SetArrayField(Key,A);
    }
    FObject Actor(AActor* A,const FString& Type,const FString& Label)
    {
        auto O=Object();
        O->SetStringField(TEXT("id"),A->GetName());
        O->SetStringField(TEXT("name"),Label);
        O->SetStringField(TEXT("type"),Type);
        O->SetStringField(TEXT("class"),A->GetClass()->GetName());
        Vector(O,TEXT("position_m"),A->GetActorLocation(),.01f);
        Vector(O,TEXT("rotation_deg"),A->GetActorRotation().Euler());
        O->SetStringField(TEXT("parent"),GetNameSafe(A->GetAttachParentActor()));
        O->SetStringField(TEXT("state"),TEXT("available"));
        // Only project-declared scalar reflected values; never traverse UObject graphs.
        auto Properties=Object();
        for(TFieldIterator<FProperty> It(A->GetClass());It;++It)
        {
            const FProperty* P=*It;
            if(!P->GetOwnerStruct()->GetName().StartsWith(TEXT("Port"))) continue;
            if(P->IsA<FNumericProperty>() || P->IsA<FBoolProperty>() || P->IsA<FStrProperty>() || P->IsA<FNameProperty>() || P->IsA<FEnumProperty>())
            {
                FString Text;
                P->ExportText_InContainer(0,Text,A,A,A,PPF_None);
                Properties->SetStringField(P->GetName(),Text);
            }
        }
        O->SetObjectField(TEXT("properties"),Properties);
        TArray<TSharedPtr<FJsonValue>> Components;
        TInlineComponentArray<USceneComponent*> Parts(A);
        for(auto* C:Parts)
        {
            auto Part=Object();
            Part->SetStringField(TEXT("name"),C->GetName());
            Part->SetStringField(TEXT("class"),C->GetClass()->GetName());
            Vector(Part,TEXT("local_position_m"),C->GetRelativeLocation(),.01f);
            Vector(Part,TEXT("world_position_m"),C->GetComponentLocation(),.01f);
            Components.Add(Value(Part));
        }
        O->SetArrayField(TEXT("components"),Components);
        return O;
    }
}

TSharedRef<FJsonObject> APortWorkingCrane::DashboardState() const
{
    using namespace Dashboard;
    auto O=Object();
    O->SetStringField(TEXT("state"),!Fault.IsEmpty()?TEXT("fault"):(bPaused?TEXT("paused"):(bJobActive?TEXT("working"):TEXT("idle"))));
    O->SetStringField(TEXT("fault"),Fault);
    O->SetNumberField(TEXT("stage"),Stage);
    O->SetBoolField(TEXT("busy"),bJobActive);
    O->SetBoolField(TEXT("carrying"),bCarrying);
    O->SetBoolField(TEXT("destination_ready"),bDestinationReady);
    O->SetNumberField(TEXT("completed"),CompletedJobs);
    O->SetNumberField(TEXT("job_seconds"),JobSeconds);
    O->SetNumberField(TEXT("stage_seconds"),StageTime);
    O->SetNumberField(TEXT("paused_seconds"),PausedSeconds);
    O->SetNumberField(TEXT("last_job_seconds"),LastJobSeconds);
    O->SetStringField(TEXT("cargo"),GetNameSafe(CargoActor));
    O->SetStringField(TEXT("agv"),GetNameSafe(HandoverAGV));
    O->SetNumberField(TEXT("payload_kg"),bCarrying && IsValid(CargoActor)?CargoActor->MassKg:0);
    Vector(O,TEXT("head_position_m"),HeadPosition(),.01f);
    Vector(O,TEXT("head_local_m"),Head,.01f);
    Vector(O,TEXT("source_m"),Slots[SourceSlot],.01f);
    Vector(O,TEXT("destination_m"),Slots[1-SourceSlot],.01f);
    O->SetNumberField(TEXT("distance_to_destination_m"),FVector::Dist(HeadPosition(),Slots[1-SourceSlot]+FVector(0,0,154.5f))*.01);
    O->SetNumberField(TEXT("beam_height_m"),BeamZ*.01);
    O->SetNumberField(TEXT("safe_height_m"),SafeZ*.01);
    O->SetStringField(TEXT("model"),TEXT("kinematic_level_spreader"));
    if(bSTS)
    {
        O->SetStringField(TEXT("model"),TEXT("reduced_order_taut_rope_sway_yaw"));
        const auto& C=STSProfile.Dynamics;
        const auto& D=SuspensionState;
        auto Physics=Object();
        Physics->SetStringField(TEXT("provenance"),TEXT("unvalidated_simulation_assumptions"));
        Physics->SetBoolField(TEXT("anti_sway"),C.AntiSway); Physics->SetBoolField(TEXT("anti_skew"),C.AntiSkew);
        Physics->SetNumberField(TEXT("sway_deg"),D.SwayDegrees());
        Physics->SetNumberField(TEXT("skew_deg"),FMath::RadiansToDegrees(D.Yaw));
        Physics->SetNumberField(TEXT("skew_rate_deg_s"),FMath::RadiansToDegrees(D.YawRate));
        Physics->SetNumberField(TEXT("skew_control_torque_nm"),D.ControlTorque);
        Physics->SetNumberField(TEXT("motor_torque_each_nm"),D.MotorTorque);
        Physics->SetNumberField(TEXT("motor_rpm"),D.MotorRPM);
        Physics->SetNumberField(TEXT("motor_power_w"),D.MotorPower);
        Physics->SetNumberField(TEXT("motor_count"),C.MotorCount);
        Physics->SetNumberField(TEXT("reeving_parts_per_corner"),C.Parts);
        Physics->SetBoolField(TEXT("saturated"),D.Saturated);
        Physics->SetBoolField(TEXT("active"),bJobActive);
        Vector(Physics,TEXT("offset_m"),D.Offset);
        TArray<TSharedPtr<FJsonValue>> Wires;
        for(int32 I=0;I<4;++I)
        {
            auto Wire=Object();
            Wire->SetNumberField(TEXT("corner"),I);
            Wire->SetNumberField(TEXT("tension_n"),D.Tension[I]);
            Wire->SetNumberField(TEXT("estimated_extension_m"),D.Extension[I]);
            Wire->SetNumberField(TEXT("limit_n"),C.RopeLimit);
            const FVector Corner((I&1)?100:-100,(I&2)?520:-520,0);
            Vector(Wire,TEXT("top_m"),Home+Orientation.RotateVector(FVector(Head.X,Head.Y,BeamZ)+Corner),.01f);
            Vector(Wire,TEXT("bottom_m"),HeadPosition()+Orientation.RotateVector(FRotator(0,FMath::RadiansToDegrees(D.Yaw),0).RotateVector(Corner)),.01f);
            Wires.Add(Value(Wire));
        }
        Physics->SetArrayField(TEXT("wires"),Wires); O->SetObjectField(TEXT("dynamics"),Physics);
        TArray<TSharedPtr<FJsonValue>> Sensors;
        for(int32 I=0;I<C.Mounts.Num() && I<SensorMarkers.Num();++I)
        {
            const auto& M=C.Mounts[I];const auto* Marker=SensorMarkers[I].Get();auto Sensor=Object();
            Sensor->SetStringField(TEXT("key"),M.Key); Sensor->SetStringField(TEXT("frame"),M.Frame);
            Sensor->SetStringField(TEXT("unit"),M.Unit);Sensor->SetStringField(TEXT("provenance"),TEXT("mixed_source_specs_and_unverified_installation"));
            if(M.Reference.IsValid()) Sensor->SetObjectField(TEXT("reference"),M.Reference);
            Vector(Sensor,TEXT("mount_position_m"),M.Position);
            Vector(Sensor,TEXT("world_position_m"),Marker->GetComponentLocation(),.01f);
            Vector(Sensor,TEXT("forward"),Marker->GetForwardVector());
            Sensor->SetNumberField(TEXT("minimum"),M.Minimum);Sensor->SetNumberField(TEXT("maximum"),M.Maximum);Sensor->SetNumberField(TEXT("fov_deg"),M.Fov);
            Sensor->SetBoolField(TEXT("scan"),M.Scan);
            Sensor->SetNumberField(TEXT("geometry_sample_wall_hz"),1);
            if(M.Key==TEXT("twistlock_load") || M.Key==TEXT("twistlock_state"))
            {
                TArray<TSharedPtr<FJsonValue>> Values;
                for(int32 CornerIndex=0;CornerIndex<4;++CornerIndex)
                    if(M.Key==TEXT("twistlock_load")) Values.Add(MakeShared<FJsonValueNumber>(Observation.CornerLoadsN[CornerIndex]));
                    else Values.Add(MakeShared<FJsonValueBoolean>(Observation.Locked[CornerIndex]));
                Sensor->SetArrayField(TEXT("values"),Values);
            }
            if(M.Key==TEXT("wind"))Sensor->SetNumberField(TEXT("value"),C.Wind.Size());
            if(M.Key==TEXT("trolley_encoder"))Sensor->SetNumberField(TEXT("value"),M.ReadPosition(Observation.DrivePosition.X*.01));
            if(M.Key==TEXT("hoist_encoder"))Sensor->SetNumberField(TEXT("value"),(BeamZ-Head.Z)*.01);
            if(M.Key==TEXT("telescope_encoder"))Sensor->SetNumberField(TEXT("value"),12.192);
            if(M.Key==TEXT("landed"))Sensor->SetBoolField(TEXT("value"),Observation.bLanded);
            if(M.Key==TEXT("agv_position_lidar")) Sensor->SetBoolField(TEXT("agv_in_range"),IsValid(HandoverAGV)&&STSSensorContains(M.Key,HandoverAGV->CargoPosition()));
            TArray<FVector> Locations;Locations.Add(M.Position);Locations.Append(M.AdditionalPositions);
            TArray<TSharedPtr<FJsonValue>> Instances;
            for(int32 InstanceIndex=0;InstanceIndex<Locations.Num();++InstanceIndex)
            {
                const FVector Origin=Marker->GetAttachParent()->GetComponentLocation()+Marker->GetAttachParent()->GetComponentQuat().RotateVector(Locations[InstanceIndex]*100);
                auto Instance=Object();Vector(Instance,TEXT("mount_position_m"),Locations[InstanceIndex]);Vector(Instance,TEXT("world_position_m"),Origin,.01f);
                TArray<TSharedPtr<FJsonValue>> Rays;
                if(M.Scan) for(int32 Ray=0;Ray<9;++Ray)
                {
                    const FVector Direction=Marker->GetComponentQuat().RotateVector(FRotator(0,-M.Fov*.5+M.Fov*Ray/8.,0).Vector());
                    const FVector End=Origin+Direction*M.Maximum*100.;
                    FCollisionQueryParams Query(SCENE_QUERY_STAT(STSDashboardScanner),false);Query.AddIgnoredActor(this);
                    FHitResult Hit;const bool Found=GetWorld()->LineTraceSingleByChannel(Hit,Origin+Direction*M.Minimum*100.,End,ECC_Visibility,Query);
                    auto R=Object();Vector(R,TEXT("end_m"),Found?Hit.ImpactPoint:End,.01f);
                    if(Found)R->SetNumberField(TEXT("distance_m"),FVector::Dist(Origin,Hit.ImpactPoint)*.01);else R->SetField(TEXT("distance_m"),MakeShared<FJsonValueNull>());
                    Rays.Add(Value(R));
                }
                Instance->SetArrayField(TEXT("rays"),Rays);Instances.Add(Value(Instance));
                if(InstanceIndex==0)Sensor->SetArrayField(TEXT("rays"),Rays);
            }
            Sensor->SetArrayField(TEXT("instances"),Instances);Sensors.Add(Value(Sensor));
        }
        O->SetArrayField(TEXT("mounted_sensors"),Sensors);
        Vector(O,TEXT("axis_velocity_mps"),AxisVelocity,.01f);
        O->SetNumberField(TEXT("rope_length_m"),(BeamZ-Head.Z)*.01);
        const double ControlMass=bCarrying?(Pickup.EstimateValid?Pickup.EstimatedMass:STSProfile.ContainerMassKg):0;
        O->SetNumberField(TEXT("hoist_limit_mps"),STSProfile.HoistLimit(ControlMass,bCarrying)*.01);
        O->SetNumberField(TEXT("rated_payload_kg"),STSProfile.RatedPayloadKg);
        O->SetNumberField(TEXT("trolley_limit_mps"),STSProfile.TrolleySpeed*.01);
        O->SetNumberField(TEXT("gantry_limit_mps"),STSProfile.GantrySpeed*.01);
        O->SetNumberField(TEXT("hoist_power_w"),STSProfile.HoistPowerW);
        auto S=Object();
        // Idle observations may still refer to the previous cargo. Mark them inactive.
        S->SetBoolField(TEXT("active"),bJobActive);
        S->SetBoolField(TEXT("valid"),bJobActive && Observation.IsFresh(SimulationTime,STSProfile.SensorMaxAge));
        S->SetNumberField(TEXT("timestamp"),Observation.Timestamp);
        S->SetNumberField(TEXT("age_seconds"),Observation.Timestamp>=0?SimulationTime-Observation.Timestamp:-1);
        S->SetBoolField(TEXT("landed"),Observation.bLanded);
        S->SetBoolField(TEXT("agv_aligned"),Observation.bAGVAligned);
        S->SetBoolField(TEXT("cargo_supported"),Observation.bCargoSupported);
        Vector(S,TEXT("drive_position_m"),Observation.DrivePosition,.01f);
        Vector(S,TEXT("drive_velocity_mps"),Observation.DriveVelocity,.01f);
        S->SetNumberField(TEXT("sway_deg"),Observation.SwayDegrees);
        S->SetNumberField(TEXT("hoist_acceleration_mps2"),Observation.HoistAcceleration*.01);
        auto Pick=Object();
        Pick->SetStringField(TEXT("phase"),Stage<2?TEXT("approach"):Pickup.PhaseName());
        Pick->SetStringField(TEXT("reason"),Pickup.Reason);
        Pick->SetBoolField(TEXT("target_visible"),Observation.bTargetVisible);
        Pick->SetBoolField(TEXT("verified"),Pickup.EstimateValid);
        Pick->SetNumberField(TEXT("attempts"),Pickup.Attempts);
        Pick->SetNumberField(TEXT("stable_seconds"),Pickup.StableTime);
        Pick->SetNumberField(TEXT("estimated_mass_kg"),Pickup.EstimatedMass);
        Pick->SetNumberField(TEXT("relative_yaw_deg"),Observation.RelativeYawDegrees);
        Pick->SetNumberField(TEXT("target_tilt_deg"),Observation.TargetTiltDegrees);
        Vector(Pick,TEXT("estimated_cog_m"),Pickup.EstimatedCoG,.01);
        Vector(Pick,TEXT("command_target_m"),Pickup.Target,.01);
        TArray<TSharedPtr<FJsonValue>> Corners;
        for(int32 I=0;I<4;++I)
        {
            auto Corner=Object();Corner->SetNumberField(TEXT("corner"),I+1);
            Corner->SetBoolField(TEXT("seated"),Observation.CornerSeated[I]);
            Corner->SetBoolField(TEXT("lock_requested"),Pickup.RequestLocks[I]);
            Corner->SetBoolField(TEXT("locked"),Observation.Locked[I]);
            Vector(Corner,TEXT("error_m"),Observation.CornerError[I],.01);Corners.Add(Value(Corner));
        }
        Pick->SetArrayField(TEXT("corners"),Corners);O->SetObjectField(TEXT("pickup"),Pick);
        TArray<TSharedPtr<FJsonValue>> Loads,Locks;
        for(int32 I=0;I<4;++I)
        {
            Loads.Add(MakeShared<FJsonValueNumber>(Observation.CornerLoadsN[I]));
            Locks.Add(MakeShared<FJsonValueBoolean>(Observation.Locked[I]));
        }
        S->SetArrayField(TEXT("corner_loads_n"),Loads); S->SetArrayField(TEXT("locks"),Locks);
        O->SetObjectField(TEXT("observation"),S);
    }
    else
    {
        O->SetNumberField(TEXT("speed_mps"),Speed*.01);
        O->SetNumberField(TEXT("horizontal_limit_mps"),3.5);
        O->SetNumberField(TEXT("vertical_limit_mps"),2);
        O->SetNumberField(TEXT("acceleration_mps2"),1.5);
    }
    return O;
}

void APortSiteLogistics::RecordVesselEvent(int32 CargoIndex,bool Placed)
{
    if(LaneCount!=9) return; // Legacy berth inventory is a different experiment.
    const int32 Vessel=Manifest[CargoIndex].STS/3;
    for(const auto& C:Manifest)
        if(C.STS/3==Vessel && (Placed?C.State!=2:!(C.HandoverMask&1))) return;
    (Placed?VesselPlaced[Vessel]:VesselUnloaded[Vessel])=SimulationTime;
}

void APortSiteLogistics::ExportDashboard(bool Paused,bool Force)
{
    using namespace Dashboard;
    if(!bReady || (!Force && FPlatformTime::Seconds()<NextDashboardWall)) return;
    NextDashboardWall=FPlatformTime::Seconds()+1.;
    auto Root=Object();
    Root->SetNumberField(TEXT("schema_version"),1);
    Root->SetStringField(TEXT("generated_at"),FDateTime::UtcNow().ToIso8601());
    Root->SetStringField(TEXT("run_id"),FPaths::GetCleanFilename(ReportBase));
    Root->SetNumberField(TEXT("simulation_seconds"),SimulationTime);
    Root->SetBoolField(TEXT("paused"),Paused);
    Root->SetBoolField(TEXT("unified"),LaneCount==9);
    Root->SetStringField(TEXT("fault"),Fault);
    Root->SetNumberField(TEXT("delivered"),Delivered);
    Root->SetNumberField(TEXT("initial_ship"),InitialShipCount());
    Root->SetNumberField(TEXT("initial_yard"),InitialYard);
    Root->SetNumberField(TEXT("peak_moving_agvs"),PeakMovingVehicles);
    Root->SetNumberField(TEXT("prefetched_jobs"),PrefetchedJobs);
    TSharedPtr<FJsonObject> Profile;
    if(FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(STSProfile.SnapshotJson),Profile)) Root->SetObjectField(TEXT("applied_profile"),Profile);
    TArray<TSharedPtr<FJsonValue>> Actors;
    for(TActorIterator<AActor> It(GetWorld());It;++It)
    {
        auto* A=*It;
        auto O=Actor(A,TEXT("other"),A->GetName());
        if(auto* Crane=Cast<APortWorkingCrane>(A))
        {
            const bool STS=Crane->bSTS;
            O->SetStringField(TEXT("type"),STS?TEXT("sts"):TEXT("rmg"));
            O->SetStringField(TEXT("name"),FString::Printf(TEXT("%s-%02d"),STS?TEXT("STS"):TEXT("RMG"),STS?Crane->CraneID-YardCraneCount:Crane->CraneID));
            auto Data=Crane->DashboardState();
            for(const auto& P:Data->Values) O->SetField(P.Key,P.Value);
        }
        else if(auto* V=Cast<APortAGVActor>(A))
        {
            O->SetStringField(TEXT("type"),TEXT("agv"));
            O->SetStringField(TEXT("name"),FString::Printf(TEXT("AGV-%03d"),V->VehicleID));
            O->SetNumberField(TEXT("speed_mps"),V->Speed*.01);
            O->SetNumberField(TEXT("speed_limit_mps"),4.5);
            O->SetNumberField(TEXT("acceleration_mps2"),1.8);
            O->SetNumberField(TEXT("completed"),V->CompletedJobs);
            O->SetStringField(TEXT("state"),V->Speed>0?TEXT("working"):TEXT("idle"));
            O->SetNumberField(TEXT("payload_kg"),0);
            for(const auto& C:ShipContainers) if(C->GetAttachParentActor()==V)
            { O->SetStringField(TEXT("cargo"),C->GetName()); O->SetNumberField(TEXT("payload_kg"),C->MassKg); }
            const int32 Lane=Vehicles.IndexOfByKey(V);
            if(Jobs.IsValidIndex(Lane))
            {
                const auto& J=Jobs[Lane];
                O->SetNumberField(TEXT("stage"),J.Stage);
                O->SetStringField(TEXT("state"),!Fault.IsEmpty()?TEXT("fault"):(Paused?TEXT("paused"):(J.Stage?TEXT("working"):TEXT("idle"))));
                if(J.STS!=INDEX_NONE) O->SetStringField(TEXT("sts"),Equipment[YardCraneCount+J.STS]->GetName());
                O->SetNumberField(TEXT("job_seconds"),J.Time);
                TArray<TSharedPtr<FJsonValue>> Route;
                double Remaining=0; FVector P=V->GetActorLocation();
                for(int32 I=J.Waypoint;I<J.Route.Num();++I)
                { auto Point=Object(); Vector(Point,TEXT("position_m"),J.Route[I],.01f); Route.Add(Value(Point)); Remaining+=FVector::Dist(P,J.Route[I])*.01; P=J.Route[I]; }
                O->SetArrayField(TEXT("route"),Route);
                O->SetNumberField(TEXT("route_remaining_m"),Remaining);
            }
        }
        else if(auto* C=Cast<APortContainerActor>(A))
        {
            O->SetStringField(TEXT("type"),TEXT("container"));
            O->SetStringField(TEXT("name"),C->ContainerID.ToString());
            O->SetNumberField(TEXT("mass_kg"),C->MassKg);
            Vector(O,TEXT("cog_m"),C->CoGOffsetCm,.01f);
            O->SetStringField(TEXT("state"),StaticEnum<ECargoOwner>()->GetNameStringByValue(int64(C->LocationOwner)));
            O->SetBoolField(TEXT("simulating_physics"),C->GetBody()->IsSimulatingPhysics());
            if(C->GetBody()->IsSimulatingPhysics()) Vector(O,TEXT("velocity_mps"),C->GetBody()->GetPhysicsLinearVelocity(),.01f);
        }
        Actors.Add(Value(O));
    }
    TArray<TSharedPtr<FJsonValue>> Vessels;
    if(LaneCount==9) for(int32 V=0;V<3;++V)
    {
        auto O=Object(); int32 Total=0,Unloaded=0,Placed=0;
        for(const auto& C:Manifest) if(C.STS/3==V) { ++Total; Unloaded+=bool(C.HandoverMask&1); Placed+=C.State==2; }
        O->SetStringField(TEXT("id"),FString::Printf(TEXT("vessel-%d"),V+1));
        O->SetStringField(TEXT("name"),FString::Printf(TEXT("VESSEL-%02d"),V+1));
        O->SetStringField(TEXT("type"),TEXT("vessel"));
        O->SetStringField(TEXT("class"),TEXT("Logical vessel / manifest group"));
        Vector(O,TEXT("position_m"),FVector(-34,-350+V*350,0));
        O->SetNumberField(TEXT("initial_count"),Total);
        O->SetNumberField(TEXT("unloaded"),Unloaded); O->SetNumberField(TEXT("placed"),Placed);
        O->SetNumberField(TEXT("started_at"),VesselStarted[V]);
        O->SetNumberField(TEXT("unloaded_at"),VesselUnloaded[V]);
        O->SetNumberField(TEXT("placed_at"),VesselPlaced[V]);
        O->SetStringField(TEXT("state"),Unloaded==Total?TEXT("complete"):TEXT("working"));
        Vessels.Add(Value(O)); Actors.Add(Value(O));
    }
    for(int32 B=0;B<18;++B)
    {
        auto O=Object(); int32 Total=0,Occupied=0;
        for(const auto& S:Yard) if(S.Block==B) { ++Total; Occupied+=S.Occupied; }
        O->SetStringField(TEXT("id"),FString::Printf(TEXT("yard-%d"),B+1));
        O->SetStringField(TEXT("name"),FString::Printf(TEXT("YARD-%02d"),B+1));
        O->SetStringField(TEXT("type"),TEXT("yard"));
        O->SetStringField(TEXT("class"),TEXT("Instanced yard block"));
        Vector(O,TEXT("position_m"),FVector(300,-460+B*44,0));
        O->SetNumberField(TEXT("slots"),Total); O->SetNumberField(TEXT("occupied"),Occupied);
        O->SetStringField(TEXT("state"),TEXT("available")); Actors.Add(Value(O));
    }
    Root->SetArrayField(TEXT("actors"),Actors); Root->SetArrayField(TEXT("vessels"),Vessels);
    FString Json;
    FJsonSerializer::Serialize(Root,TJsonWriterFactory<TCHAR,TCondensedJsonPrintPolicy<TCHAR>>::Create(&Json));
    const FString Directory=FPaths::ProjectSavedDir()/TEXT("Dashboard");
    IFileManager::Get().MakeDirectory(*Directory,true);
    // Publish by rename so HTTP readers cannot observe a half-written snapshot.
    const FString Temporary=Directory/TEXT("state.tmp"), Final=Directory/TEXT("state.json");
    if(FFileHelper::SaveStringToFile(Json,*Temporary,FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM))
        IFileManager::Get().Move(*Final,*Temporary,true,true);
}
