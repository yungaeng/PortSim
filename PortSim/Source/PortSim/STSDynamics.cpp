#include "STSDynamics.h"
#include "Dom/JsonObject.h"

bool FSTSDynamicsConfig::Load(TSharedPtr<FJsonObject> Root,FString& Error)
{
    const TSharedPtr<FJsonObject>* Config=nullptr;
    if(!Root || !Root->TryGetObjectField(TEXT("dynamics"),Config)) { Error=TEXT("Missing dynamics assumptions"); return false; }
    auto O=*Config;
    auto N=[&](const TCHAR* Key,double& V,bool Zero=false)
    { if(!O->TryGetNumberField(Key,V) || !FMath::IsFinite(V) || V<0 || (!Zero && V==0)) {Error=FString(TEXT("Invalid dynamics field: "))+Key;return false;} return true; };
    if(!O->TryGetBoolField(TEXT("anti_sway"),AntiSway) || !O->TryGetBoolField(TEXT("anti_skew"),AntiSkew) ||
        !N(TEXT("passive_damping_per_s"),Damping,true) || !N(TEXT("anti_sway_velocity_gain"),SwayGain) ||
        !N(TEXT("skew_kp_nm_per_rad"),SkewKp) || !N(TEXT("skew_kd_nms_per_rad"),SkewKd) || !N(TEXT("skew_max_torque_nm"),SkewMaxTorque) ||
        !N(TEXT("drum_radius_m"),DrumRadius) || !N(TEXT("gear_ratio"),GearRatio) || !N(TEXT("efficiency"),Efficiency) ||
        !N(TEXT("reeving_parts_per_corner"),Parts) || !N(TEXT("motor_count"),MotorCount) || !N(TEXT("motor_max_torque_nm"),MotorMaxTorque) ||
        !N(TEXT("motor_max_rpm"),MotorMaxRPM) || !N(TEXT("motor_inertia_kgm2"),MotorInertia) ||
        !N(TEXT("rope_axial_rigidity_n"),RopeEA) || !N(TEXT("rope_tension_limit_n"),RopeLimit) || !N(TEXT("landing_skew_limit_deg"),SkewLimit) ||
        !N(TEXT("wind_area_m2"),WindArea) || !N(TEXT("wind_drag_coefficient"),WindDrag)) return false;
    if(!O->TryGetNumberField(TEXT("wind_x_mps"),Wind.X) || !O->TryGetNumberField(TEXT("wind_y_mps"),Wind.Y) ||
        !O->TryGetNumberField(TEXT("wind_yaw_moment_nm"),WindMoment) || Wind.ContainsNaN() || !FMath::IsFinite(WindMoment) ||
        Efficiency>1 || Parts!=FMath::FloorToDouble(Parts) || MotorCount!=2 || SkewLimit>=10 || GearRatio>1000 || DrumRadius>10 || Damping>10 || Wind.Size()>50)
    { Error=TEXT("Inconsistent dynamics assumptions"); return false; }
    const TArray<TSharedPtr<FJsonValue>>* Sensors=nullptr;
    if(!Root->TryGetArrayField(TEXT("sensor_mounts"),Sensors) || Sensors->Num()!=14) {Error=TEXT("Expected 14 sensor mounts");return false;}
    TSet<FString> Keys;
    for(const auto& V:*Sensors)
    {
        auto S=V->AsObject(); FSTSSensorMount M;
        if(!S || !S->TryGetStringField(TEXT("key"),M.Key) || !S->TryGetStringField(TEXT("frame"),M.Frame) ||
            !S->TryGetStringField(TEXT("unit"),M.Unit) || !S->TryGetBoolField(TEXT("scan"),M.Scan) ||
            !S->TryGetNumberField(TEXT("minimum"),M.Minimum) || !S->TryGetNumberField(TEXT("maximum"),M.Maximum) ||
            !S->TryGetNumberField(TEXT("fov_deg"),M.Fov)) { Error=TEXT("Incomplete sensor mount");return false; }
        const TArray<TSharedPtr<FJsonValue>>* P=nullptr; const TArray<TSharedPtr<FJsonValue>>* R=nullptr;
        if(!S->TryGetArrayField(TEXT("position_m"),P) || !S->TryGetArrayField(TEXT("rotation_deg"),R) || P->Num()!=3 || R->Num()!=3)
        {Error=TEXT("Sensor mount requires XYZ and pitch/yaw/roll");return false;}
        double Angles[3];
        for(int32 I=0;I<3;++I) if(!(*P)[I]->TryGetNumber(M.Position[I]) || !(*R)[I]->TryGetNumber(Angles[I]) || !FMath::IsFinite(M.Position[I]) || !FMath::IsFinite(Angles[I]))
        {Error=TEXT("Invalid sensor pose");return false;}
        M.Rotation=FRotator(Angles[0],Angles[1],Angles[2]);
        if(Keys.Contains(M.Key) || !FMath::IsFinite(M.Minimum) || !FMath::IsFinite(M.Maximum) || !FMath::IsFinite(M.Fov) ||
            M.Maximum<=M.Minimum || M.Fov<0 || M.Fov>360 || (M.Scan && (M.Fov<=0 || M.Minimum<0 || M.Maximum>500)) ||
            (M.Frame!=TEXT("gantry") && M.Frame!=TEXT("trolley") && M.Frame!=TEXT("spreader")))
        {Error=TEXT("Invalid sensor range/frame or duplicate key");return false;}
        const TArray<TSharedPtr<FJsonValue>>* Extra=nullptr;
        if(S->HasField(TEXT("additional_positions_m")))
        {
            if(!S->TryGetArrayField(TEXT("additional_positions_m"),Extra) || Extra->Num()>3) {Error=TEXT("Invalid additional sensor mounts");return false;}
            for(const auto& Item:*Extra)
            {
                const TArray<TSharedPtr<FJsonValue>>* XYZ=nullptr;FVector Position;
                if(!Item->TryGetArray(XYZ) || XYZ->Num()!=3) {Error=TEXT("Additional mount requires XYZ");return false;}
                for(int32 Axis=0;Axis<3;++Axis) if(!(*XYZ)[Axis]->TryGetNumber(Position[Axis]) || !FMath::IsFinite(Position[Axis])) {Error=TEXT("Invalid additional mount position");return false;}
                M.AdditionalPositions.Add(Position);
            }
        }
        auto OptionalNumber=[&](const TCHAR* Key,double& Number)
        { return !S->HasField(Key) || (S->TryGetNumberField(Key,Number) && FMath::IsFinite(Number)); };
        if(!OptionalNumber(TEXT("measurement_origin_m"),M.MeasurementOrigin) || !OptionalNumber(TEXT("resolution_m"),M.Resolution) ||
            !OptionalNumber(TEXT("max_traversing_speed_mps"),M.MaxTraversingSpeed) || M.Resolution<0 || M.MaxTraversingSpeed<0)
        {Error=TEXT("Invalid sensor measurement specification");return false;}
        M.Reference=S;
        Keys.Add(M.Key); Mounts.Add(M);
    }
    return true;
}

void FSTSSuspension::Step(const FSTSDynamicsConfig& C,double Dt,double L,double Ldot,FVector A,FVector V,double M,FVector CoG,double PowerW)
{
    if(Dt<=0 || M<=0 || L<.1) return;
    constexpr double G=9.80665;
    const FVector WindRelative=C.Wind-V;
    const FVector WindForce=.5*1.225*C.WindDrag*C.WindArea*WindRelative.Size()*WindRelative;
    const FVector OldOffset=Offset;
    for(double Remaining=Dt;Remaining>1.e-9;)
    {
        const double H=FMath::Min(Remaining,1./240.); Remaining-=H;
        for(int32 I=0;I<2;++I)
        {
            const double Acc=-(G*FMath::Sin(Angle[I])+A[I]*FMath::Cos(Angle[I]))/L-
                (C.Damping+2*Ldot/L)*Rate[I]+WindForce[I]/(M*L);
            Rate[I]+=Acc*H; Angle[I]+=Rate[I]*H;
        }
        const double Inertia=M*(2.44*2.44+12.2*12.2)/12.;
        const double PassiveK=M*G*(1.+5.2*5.2)/L;
        ControlTorque=C.AntiSkew?FMath::Clamp(-C.SkewKp*Yaw-C.SkewKd*YawRate,-C.SkewMaxTorque,C.SkewMaxTorque):0;
        const double External=C.WindMoment+M*(CoG.Y*A.X-CoG.X*A.Y);
        YawRate+=(External+ControlTorque-PassiveK*FMath::Sin(Yaw)-C.Damping*Inertia*YawRate)/Inertia*H;
        Yaw+=YawRate*H;
    }
    Offset=FVector(L*FMath::Sin(Angle.X),L*FMath::Sin(Angle.Y),L*(1-FMath::Cos(Angle.X)*FMath::Cos(Angle.Y)));
    OffsetVelocity=(Offset-OldOffset)/Dt;
    const double Vertical=M*FMath::Max(.1,G+A.Z+L*Rate.SizeSquared());
    // Roll/pitch are constrained. Equal-compliance four-corner force/moment allocation.
    // With the centre of mass below the support plane, positive horizontal
    // acceleration increases the reaction on the positive-axis corner.
    const double EX=CoG.X+1.545*A.X/FMath::Max(.1,G+A.Z), EY=CoG.Y+1.545*A.Y/FMath::Max(.1,G+A.Z);
    for(int32 I=0;I<4;++I)
    {
        const double Fraction=(.5+((I&1)?1:-1)*EX/2.)*(.5+((I&2)?1:-1)*EY/10.4);
        const FVector Corner((I&1)?1:-1,(I&2)?5.2:-5.2,0);
        const FVector Delta=FRotator(0,FMath::RadiansToDegrees(Yaw),0).RotateVector(Corner)+Offset-Corner-FVector(0,0,L);
        const double Cos=FMath::Max(.1,-Delta.Z/Delta.Size());
        Tension[I]=FMath::Max(0.,Vertical*Fraction/(Cos*C.Parts));
        Extension[I]=Tension[I]*Delta.Size()/C.RopeEA;
        if(Fraction<=0 || Tension[I]>C.RopeLimit) Fault=TEXT("Suspension rope slack/overload outside taut-rope model");
    }
    const double ShaftFactor=C.Parts*C.GearRatio/C.DrumRadius;
    MotorRPM=V.Z*ShaftFactor*60/(2*PI);
    MotorTorque=Vertical*C.DrumRadius/(C.Parts*C.GearRatio*C.Efficiency*C.MotorCount)+C.MotorInertia*A.Z*ShaftFactor;
    MotorPower=MotorTorque*V.Z*ShaftFactor*C.MotorCount;
    Saturated=FMath::Abs(MotorTorque)>C.MotorMaxTorque || FMath::Abs(MotorRPM)>C.MotorMaxRPM || FMath::Abs(MotorPower)>PowerW;
    if(SwayDegrees()>20 || FMath::Abs(FMath::RadiansToDegrees(Yaw))>15 || Offset.ContainsNaN()) Fault=TEXT("Suspension attitude outside reduced-order model envelope");
}
