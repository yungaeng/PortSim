#include "PortAGVActor.h"
#include "Dom/JsonObject.h"

TSharedRef<FJsonObject> APortAGVActor::DynamicsSnapshot() const
{
    auto O=MakeShared<FJsonObject>(); const auto& D=Dynamics; const auto& C=DynamicsSettings;
    auto Vector=[&](const TCHAR* Key,FVector V)
    { TArray<TSharedPtr<FJsonValue>> A; for(double N:{V.X,V.Y,V.Z}) A.Add(MakeShared<FJsonValueNumber>(N)); O->SetArrayField(Key,A); };
    O->SetStringField(TEXT("model"),TEXT("six_contact_planar_tire_roll_pitch"));
    O->SetStringField(TEXT("provenance"),TEXT("unvalidated_simulation_assumptions"));
    O->SetStringField(TEXT("axes"),TEXT("X forward, Y left, Z up; CoG is above road"));
    O->SetNumberField(TEXT("mass_kg"),D.TotalMassKg); Vector(TEXT("combined_cog_m"),D.CombinedCoGM);
    Vector(TEXT("cargo_cog_m"),DynamicsInput().CargoCoGM);
    Vector(TEXT("velocity_body_mps"),FVector(D.U,D.V,0));
    Vector(TEXT("acceleration_body_mps2"),FVector(D.Ax,D.Ay,0));
    Vector(TEXT("imu_specific_force_mps2"),D.IMUSpecificForceMps2);
    Vector(TEXT("imu_gyro_rad_s"),FVector(D.RollRate,D.PitchRate,D.YawRate));
    O->SetStringField(TEXT("sensor_basis"),TEXT("ideal synthetic; no bias/noise/latency; planar specific force includes +g in Z"));
    O->SetNumberField(TEXT("yaw_rate_rad_s"),D.YawRate); O->SetNumberField(TEXT("turn_radius_m"),D.EffectiveRadiusM);
    O->SetNumberField(TEXT("roll_deg"),FMath::RadiansToDegrees(D.Roll)); O->SetNumberField(TEXT("pitch_deg"),FMath::RadiansToDegrees(D.Pitch));
    O->SetNumberField(TEXT("left_load_n"),D.LeftLoadN); O->SetNumberField(TEXT("right_load_n"),D.RightLoadN);
    O->SetNumberField(TEXT("front_load_n"),D.FrontLoadN); O->SetNumberField(TEXT("rear_load_n"),D.RearLoadN);
    O->SetNumberField(TEXT("load_transfer_ratio"),D.LoadTransferRatio); O->SetNumberField(TEXT("peak_ltr"),D.PeakLTR);
    O->SetNumberField(TEXT("max_slip_ratio"),D.MaxSlip); O->SetNumberField(TEXT("peak_slip_ratio"),D.PeakSlip);
    O->SetBoolField(TEXT("wheel_lift"),D.WheelLift); O->SetBoolField(TEXT("overturn_threshold"),D.OverturnThreshold);
    O->SetBoolField(TEXT("stability_limited"),D.StabilityLimited);
    O->SetNumberField(TEXT("stability_speed_limit_mps"),D.StabilityLimitMps);
    O->SetBoolField(TEXT("braking"),D.Braking); O->SetNumberField(TEXT("braking_elapsed_s"),D.BrakingElapsed);
    O->SetNumberField(TEXT("braking_distance_m"),D.BrakingDistanceM);
    O->SetBoolField(TEXT("last_brake_valid"),D.LastBrakeValid);
    O->SetNumberField(TEXT("last_braking_seconds"),D.LastBrakingSeconds);
    O->SetNumberField(TEXT("last_braking_distance_m"),D.LastBrakingDistanceM);
    O->SetBoolField(TEXT("physical_emergency_braking"),Emergency.Active && Emergency.BrakeCommand);
    O->SetBoolField(TEXT("legacy_stop_constraint"),!Emergency.Active && (Sensors.bFMSHold || Sensors.bControlledStop));
    Vector(TEXT("cargo_shift_m"),D.CargoOffsetM);Vector(TEXT("cargo_relative_velocity_mps"),D.CargoRelativeVelocityMps);
    Vector(TEXT("cargo_inertia_n"),D.CargoInertialForceN);Vector(TEXT("deck_contact_force_n"),D.CargoContactForceN);
    Vector(TEXT("restraint_force_n"),D.RestraintForceN);
    O->SetBoolField(TEXT("cargo_sliding"),D.CargoSliding);O->SetBoolField(TEXT("cargo_unsafe"),D.CargoUnsafe);
    O->SetStringField(TEXT("fault"),D.Fault);
    auto Settings=MakeShared<FJsonObject>();
    Settings->SetNumberField(TEXT("wheel_radius_m"),C.WheelRadiusM); Settings->SetNumberField(TEXT("wheel_inertia_kg_m2"),C.WheelInertiaKgM2);
    Settings->SetNumberField(TEXT("longitudinal_stiffness_per_load"),C.LongitudinalStiffnessPerLoad);
    Settings->SetNumberField(TEXT("cornering_stiffness_per_load"),C.CorneringStiffnessPerLoad);
    TArray<TSharedPtr<FJsonValue>> EmptyCoG;
    for(double N:{C.EmptyCoGM.X,C.EmptyCoGM.Y,C.EmptyCoGM.Z}) EmptyCoG.Add(MakeShared<FJsonValueNumber>(N));
    Settings->SetArrayField(TEXT("empty_cog_m"),EmptyCoG);
    Settings->SetNumberField(TEXT("stability_utilization"),C.StabilityUtilization);
    Settings->SetNumberField(TEXT("integration_hz"),240);
    Settings->SetNumberField(TEXT("track_m"),C.TrackM); Settings->SetNumberField(TEXT("wheelbase_m"),C.WheelbaseM);
    Settings->SetNumberField(TEXT("roll_stiffness_nm_rad"),C.RollStiffnessNmPerRad); Settings->SetNumberField(TEXT("roll_damping_nms_rad"),C.RollDampingNmsPerRad);
    Settings->SetNumberField(TEXT("pitch_stiffness_nm_rad"),C.PitchStiffnessNmPerRad); Settings->SetNumberField(TEXT("pitch_damping_nms_rad"),C.PitchDampingNmsPerRad);
    Settings->SetNumberField(TEXT("drive_torque_each_nm"),C.DriveTorqueEachNm); Settings->SetNumberField(TEXT("brake_torque_each_nm"),C.BrakeTorqueEachNm);
    Settings->SetBoolField(TEXT("stability_control"),C.bStabilitySpeedControl); Settings->SetBoolField(TEXT("traction_control"),C.bTractionControl); Settings->SetBoolField(TEXT("abs"),C.bABS);
    O->SetObjectField(TEXT("settings"),Settings);
    TArray<TSharedPtr<FJsonValue>> Wheels;
    for(int J=0;J<6;++J)
    {
        const auto& W=D.Wheels[J]; auto P=MakeShared<FJsonObject>();
        P->SetStringField(TEXT("name"),FString::Printf(TEXT("%s_%s"),J/2==0?TEXT("rear"):J/2==1?TEXT("middle"):TEXT("front"),J%2==0?TEXT("left"):TEXT("right")));
        P->SetNumberField(TEXT("rpm"),W.Omega*60/(2*PI)); P->SetNumberField(TEXT("normal_n"),W.NormalN);
        P->SetNumberField(TEXT("longitudinal_n"),W.FxN); P->SetNumberField(TEXT("lateral_n"),W.FyN);
        P->SetNumberField(TEXT("applied_torque_nm"),W.TorqueNm); P->SetNumberField(TEXT("slip_ratio"),W.SlipRatio);
        P->SetNumberField(TEXT("slip_angle_deg"),FMath::RadiansToDegrees(W.SlipAngle)); P->SetBoolField(TEXT("sliding"),W.Sliding);
        P->SetNumberField(TEXT("steering_deg"),FMath::RadiansToDegrees(W.Steer));
        Wheels.Add(MakeShared<FJsonValueObject>(P));
    }
    O->SetArrayField(TEXT("wheels"),Wheels); return O;
}
