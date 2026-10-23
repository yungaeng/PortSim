#include "AGVDynamics.h"

namespace
{
    double Bounded(double V,double Fallback,double Low,double High)
    { return FMath::IsFinite(V)?FMath::Clamp(V,Low,High):Fallback; }
}

void FAGVDynamicsSettings::Sanitize()
{
    WheelRadiusM=Bounded(WheelRadiusM,.7,.1,2); WheelInertiaKgM2=Bounded(WheelInertiaKgM2,400,20,10000);
    TrackM=Bounded(TrackM,3.4,1,4.2); WheelbaseM=Bounded(WheelbaseM,9.6,3,14);
    if(EmptyCoGM.ContainsNaN()) EmptyCoGM=FVector(0,0,1);
    EmptyCoGM.X=Bounded(EmptyCoGM.X,0,-WheelbaseM*.45,WheelbaseM*.45);
    EmptyCoGM.Y=Bounded(EmptyCoGM.Y,0,-TrackM*.45,TrackM*.45); EmptyCoGM.Z=Bounded(EmptyCoGM.Z,1,.1,4);
    LongitudinalStiffnessPerLoad=Bounded(LongitudinalStiffnessPerLoad,12,1,30);
    CorneringStiffnessPerLoad=Bounded(CorneringStiffnessPerLoad,4,.5,8);
    RollStiffnessNmPerRad=Bounded(RollStiffnessNmPerRad,8.e6,1.e5,1.e8);
    RollDampingNmsPerRad=Bounded(RollDampingNmsPerRad,1.6e6,1000,1.e7);
    PitchStiffnessNmPerRad=Bounded(PitchStiffnessNmPerRad,2.e7,1.e5,1.e8);
    PitchDampingNmsPerRad=Bounded(PitchDampingNmsPerRad,3.e6,1000,1.e7);
    DriveTorqueEachNm=Bounded(DriveTorqueEachNm,12000,0,100000);
    BrakeTorqueEachNm=Bounded(BrakeTorqueEachNm,50000,0,100000);
    StabilityUtilization=Bounded(StabilityUtilization,.65,.1,.95);
}

void FAGVDynamicsState::Reset(double Speed,const FAGVDynamicsSettings& Settings)
{
    *this=FAGVDynamicsState(); U=FMath::IsFinite(Speed)?Speed:0;
    for(auto& W:Wheels) W.Omega=U/FMath::Max(.1f,Settings.WheelRadiusM);
}

void FAGVDynamicsState::RefreshLoads(const FAGVDynamicsSettings& C,const FAGVDynamicsInput& I)
{
    const double Empty=FMath::Max(1.,I.EmptyMassKg),Cargo=FMath::Max(0.,I.PayloadKg);
    TotalMassKg=Empty+Cargo;
    CombinedCoGM=(C.EmptyCoGM*Empty+(I.CargoCoGM+(I.CargoContactEnabled?CargoOffsetM:FVector::ZeroVector))*Cargo)/TotalMassKg;
    const double Weight=TotalMassKg*PortFriction::Gravity;
    // Equal axle spring load sharing, plus fore/aft and left/right moments.
    // Dynamic spring reactions feed wheel normal loads and hence tire forces.
    const double RollReaction=C.RollStiffnessNmPerRad*Roll+C.RollDampingNmsPerRad*RollRate;
    const double PitchReaction=C.PitchStiffnessNmPerRad*Pitch+C.PitchDampingNmsPerRad*PitchRate;
    const double LeftFraction=.5+CombinedCoGM.Y/C.TrackM-RollReaction/(Weight*C.TrackM);
    const double FrontFraction=1./3.+CombinedCoGM.X/C.WheelbaseM-PitchReaction/(Weight*C.WheelbaseM);
    const double RearFraction=1./3.-CombinedCoGM.X/C.WheelbaseM+PitchReaction/(Weight*C.WheelbaseM);
    const double Fractions[3]={RearFraction,1./3.,FrontFraction};
    MinimumRawLoadN=TNumericLimits<double>::Max(); double Sum=0;
    for(int A=0;A<3;++A) for(int S=0;S<2;++S)
    {
        auto& W=Wheels[A*2+S]; const double Raw=Weight*Fractions[A]*(S==0?LeftFraction:1-LeftFraction);
        MinimumRawLoadN=FMath::Min(MinimumRawLoadN,Raw); W.NormalN=FMath::Max(0.,Raw); Sum+=W.NormalN;
    }
    LeftLoadN=RightLoadN=FrontLoadN=RearLoadN=0;
    for(int J=0;J<6;++J)
    {
        auto& W=Wheels[J]; W.NormalN*=Weight/FMath::Max(1.,Sum);
        (J%2==0?LeftLoadN:RightLoadN)+=W.NormalN;
        if(J/2==0) RearLoadN+=W.NormalN; if(J/2==2) FrontLoadN+=W.NormalN;
    }
    LoadTransferRatio=(RightLoadN-LeftLoadN)/Weight;
    WheelLift=MinimumRawLoadN<=0; PeakLTR=FMath::Max(PeakLTR,FMath::Abs(LoadTransferRatio));
    // This marks departure from the small-angle/contact model, not a 3D tumble.
    OverturnThreshold=WheelLift || FMath::Abs(Roll)>FMath::DegreesToRadians(30.);
    const double LateralMargin=FMath::Max(0.,C.TrackM*.5-FMath::Abs(CombinedCoGM.Y));
    StabilityLimitMps=FMath::Abs(I.CurvaturePerM)>1.e-6?
        FMath::Sqrt(C.StabilityUtilization*PortFriction::Gravity*LateralMargin/
            (FMath::Max(.1,CombinedCoGM.Z)*FMath::Abs(I.CurvaturePerM))):7.;
}

void FAGVDynamicsState::ConstrainStop()
{
    U=V=YawRate=Ax=Ay=0; RollRate=PitchRate=0;
    for(auto& W:Wheels) { W.Omega=0; W.FxN=W.FyN=W.TorqueNm=W.SlipRatio=W.SlipAngle=0; W.Sliding=false; }
    if(Braking) { LastBrakeValid=false; Braking=false; BrakingElapsed=BrakingDistanceM=0; }
    IMUSpecificForceMps2=FVector(0,0,PortFriction::Gravity);
}

bool FAGVDynamicsState::Finite() const
{
    if(PositionM.ContainsNaN() || CombinedCoGM.ContainsNaN() || IMUSpecificForceMps2.ContainsNaN() ||
        CargoOffsetM.ContainsNaN() || CargoRelativeVelocityMps.ContainsNaN() || CargoContactForceN.ContainsNaN()) return false;
    for(double D:{Heading,U,V,YawRate,Roll,RollRate,Pitch,PitchRate,Ax,Ay,TotalMassKg}) if(!FMath::IsFinite(D)) return false;
    for(const auto& W:Wheels) for(double D:{W.Omega,W.NormalN,W.FxN,W.FyN,W.SlipRatio}) if(!FMath::IsFinite(D)) return false;
    return true;
}

void FAGVDynamicsState::Step(const FAGVDynamicsSettings& Settings,const FAGVDynamicsInput& Input,double Seconds)
{
    if(!FMath::IsFinite(Seconds) || Seconds<=0 || Seconds>10 || !Fault.IsEmpty()) return;
    FAGVDynamicsSettings C=Settings; C.Sanitize(); FAGVDynamicsInput I=Input;
    if(I.CargoCoGM.ContainsNaN() || I.WindForceBodyN.ContainsNaN()) { Fault=TEXT("Invalid AGV CoG/wind input"); return; }
    for(double D:{I.EmptyMassKg,I.PayloadKg,I.TargetSpeedMps,I.CurvaturePerM,I.AccelerationLimitMps2,
        I.BrakeDemandMps2,I.StaticMu,I.KineticMu,I.RollingResistance,I.DrivenFraction,
        I.DeckStaticMu,I.DeckKineticMu,I.RestraintCapacityN,I.CargoShiftLimitM})
        if(!FMath::IsFinite(D)) { Fault=TEXT("Invalid AGV dynamics input"); return; }
    I.StaticMu=FMath::Clamp(I.StaticMu,0.,2.); I.KineticMu=FMath::Clamp(I.KineticMu,0.,I.StaticMu);
    I.DrivenFraction=FMath::Clamp(I.DrivenFraction,0.,1.); I.CurvaturePerM=FMath::Clamp(I.CurvaturePerM,-1./5.8,1./5.8);
    I.RollingResistance=FMath::Clamp(I.RollingResistance,0.,.1);
    I.DeckStaticMu=FMath::Clamp(I.DeckStaticMu,0.,2.); I.DeckKineticMu=FMath::Clamp(I.DeckKineticMu,0.,I.DeckStaticMu);
    const int Steps=FMath::Max(1,FMath::CeilToInt(Seconds*240.)); const double Dt=Seconds/Steps;
    for(int Step=0;Step<Steps;++Step)
    {
        RefreshLoads(C,I);
        const FVector OldCoG=CombinedCoGM;
        const double CargoFraction=FMath::Clamp(I.PayloadKg/TotalMassKg,0.,.95);
        const FVector RelativeV=I.CargoContactEnabled?CargoRelativeVelocityMps:FVector::ZeroVector;
        double Target=FMath::Clamp(I.TargetSpeedMps,-7.,7.);
        StabilityLimited=C.bStabilitySpeedControl && FMath::Abs(Target)>StabilityLimitMps;
        if(C.bStabilitySpeedControl) Target=FMath::Clamp(Target,-StabilityLimitMps,StabilityLimitMps);
        const double OldU=U,OldV=V,OldYaw=YawRate;
        const bool Slowing=U*Target<0 || FMath::Abs(Target)<FMath::Abs(U)-.01;
        const double DesiredAcceleration=FMath::Sign(Target-U)*FMath::Min(FMath::Abs((Target-U)/.25),
            FMath::Max(0.,Slowing?I.BrakeDemandMps2:I.AccelerationLimitMps2));
        if(Slowing && !Braking && FMath::Abs(U)>.05)
        { Braking=true; BrakingElapsed=BrakingDistanceM=0; LastBrakeValid=false; }
        if(Braking && !Slowing && FMath::Abs(Target)>.05) { Braking=false; LastBrakeValid=false; }
        double ForceX=0,ForceY=0,Moment=0; MaxSlip=0;
        for(int J=0;J<6;++J)
        {
            auto& W=Wheels[J]; const double X=(J/2-1)*C.WheelbaseM*.5,Y=(J%2==0?1:-1)*C.TrackM*.5;
            W.Steer=FMath::Atan2(X*I.CurvaturePerM,1-Y*I.CurvaturePerM);
            const double CS=FMath::Cos(W.Steer),SN=FMath::Sin(W.Steer);
            const double RX=X-CombinedCoGM.X,RY=Y-CombinedCoGM.Y;
            const double VX=CS*(U-CargoFraction*RelativeV.X-YawRate*RY)+SN*(V-CargoFraction*RelativeV.Y+YawRate*RX);
            const double VY=-SN*(U-CargoFraction*RelativeV.X-YawRate*RY)+CS*(V-CargoFraction*RelativeV.Y+YawRate*RX);
            const double Denom=FMath::Max(.5,FMath::Abs(VX));
            const double PreviousSlip=(C.WheelRadiusM*W.Omega-VX)/Denom;
            const double Mu=I.KineticMu+(I.StaticMu-I.KineticMu)*FMath::Exp(-FMath::Max(0.,FMath::Abs(PreviousSlip)-.08)*8);
            const double Rolling=Slowing?0:I.RollingResistance*TotalMassKg*PortFriction::Gravity*FMath::Sign(Target);
            const double LoadShare=W.NormalN/(TotalMassKg*PortFriction::Gravity);
            double Torque=(TotalMassKg*DesiredAcceleration+Rolling)*C.WheelRadiusM*LoadShare;
            if(Slowing)
            {
                Torque=FMath::Clamp(Torque,-double(C.BrakeTorqueEachNm),double(C.BrakeTorqueEachNm));
                if(C.bABS && PreviousSlip*FMath::Sign(U)<-.12) Torque*=.2;
            }
            else
            {
                Torque=FMath::Clamp(Torque,-double(C.DriveTorqueEachNm),double(C.DriveTorqueEachNm));
                if(C.bTractionControl && PreviousSlip*FMath::Sign(Target)>.12) Torque*=.2;
                // Assumed motor overspeed governor, using the 7 m/s reference
                // at the wheel perimeter; free wheelspin cannot grow forever.
                if(Torque*W.Omega>0 && FMath::Abs(W.Omega)*C.WheelRadiusM>=7) Torque=0;
            }
            W.TorqueNm=Torque;
            // Implicit longitudinal tire/wheel step. Avoid artificial wheel
            // chatter at low speed without replacing wheel speed by body speed.
            const double Stiff=C.LongitudinalStiffnessPerLoad*W.NormalN/Denom;
            const double FreeOmega=W.Omega+Dt*Torque/C.WheelInertiaKgM2;
            double FX=Stiff*(C.WheelRadiusM*FreeOmega-VX)/
                (1+Stiff*Dt*(C.WheelRadiusM*C.WheelRadiusM/C.WheelInertiaKgM2+1./FMath::Max(1.,TotalMassKg*LoadShare)));
            double FY=-C.CorneringStiffnessPerLoad*W.NormalN*VY/Denom;
            const double Limit=Mu*W.NormalN;
            if(!Slowing) FX=FMath::Clamp(FX,-Limit*I.DrivenFraction,Limit*I.DrivenFraction);
            const double Demand=FMath::Sqrt(FX*FX+FY*FY);
            W.Sliding=Demand>Limit+1 || FMath::Abs(PreviousSlip)>.1;
            if(Demand>Limit && Demand>0) { FX*=Limit/Demand; FY*=Limit/Demand; }
            const double NewOmega=W.Omega+Dt*(Torque-C.WheelRadiusM*FX)/C.WheelInertiaKgM2;
            // A service brake cannot motor the wheel backwards through zero.
            W.Omega=Slowing && NewOmega*FMath::Sign(U)<0?0:NewOmega;
            W.Angle=FMath::Fmod(W.Angle+W.Omega*Dt,2*PI);
            W.FxN=FX; W.FyN=FY; W.SlipRatio=(C.WheelRadiusM*W.Omega-VX)/Denom;
            W.SlipAngle=FMath::Atan2(VY,Denom); MaxSlip=FMath::Max(MaxSlip,FMath::Abs(W.SlipRatio));
            const double BodyFX=CS*FX-SN*FY,BodyFY=SN*FX+CS*FY;
            ForceX+=BodyFX; ForceY+=BodyFY; Moment+=RX*BodyFY-RY*BodyFX;
        }
        const double Rolling=I.RollingResistance*TotalMassKg*PortFriction::Gravity*FMath::Clamp(U/.05,-1.,1.);
        ForceX+=I.WindForceBodyN.X-Rolling; ForceY+=I.WindForceBodyN.Y;
        Ax=ForceX/TotalMassKg; Ay=ForceY/TotalMassKg;
        // U/V integrate total system momentum; internal deck forces cancel.
        // Sliding mass moves the chassis origin relative to the system CoG.
        // Planar translation with yaw pseudo-forces; no cargo rotation or 3D fall.
        if(I.CargoContactEnabled && I.PayloadKg>0)
        {
            const double Mass=I.PayloadKg,Normal=Mass*PortFriction::Gravity;
            const FVector Arm=I.CargoCoGM+CargoOffsetM-CombinedCoGM;
            const double Iz=TotalMassKg*(C.WheelbaseM*C.WheelbaseM+C.TrackM*C.TrackM)/12.;
            const double Alpha=Moment/Iz;
            const FVector RequiredA(Ax-Alpha*Arm.Y-YawRate*YawRate*Arm.X,
                Ay+Alpha*Arm.X-YawRate*YawRate*Arm.Y,0);
            const FVector RequiredForce=RequiredA*Mass;
            CargoInertialForceN=-RequiredForce; RestraintForceN=FVector::ZeroVector;
            if(I.CargoSecured || (CargoRelativeVelocityMps.Size2D()<1.e-5 && RequiredForce.Size2D()<=I.DeckStaticMu*Normal))
            {
                CargoContactForceN=RequiredForce.GetClampedToMaxSize(I.DeckStaticMu*Normal);
                if(I.CargoSecured) RestraintForceN=RequiredForce-CargoContactForceN;
                RestraintOverload|=RestraintForceN.Size2D()>FMath::Max(0.,I.RestraintCapacityN);
                CargoRelativeVelocityMps=FVector::ZeroVector; CargoSliding=false;
            }
            else
            {
                // The no-slip impulse at this substep either stops relative
                // motion or saturates the kinetic Coulomb friction circle.
                const FVector Needed=(RequiredA-CargoRelativeVelocityMps*((1-CargoFraction)/Dt))*Mass;
                CargoContactForceN=Needed.GetClampedToMaxSize(I.DeckKineticMu*Normal);
                FVector RelativeA=(CargoContactForceN/Mass-RequiredA)/(1-CargoFraction);
                RelativeA+=FVector(2*YawRate*CargoRelativeVelocityMps.Y,-2*YawRate*CargoRelativeVelocityMps.X,0);
                CargoRelativeVelocityMps+=RelativeA*Dt;
                CargoOffsetM+=CargoRelativeVelocityMps*Dt;
                CargoSliding=CargoRelativeVelocityMps.Size2D()>1.e-5;
            }
            PeakCargoShiftM=FMath::Max(PeakCargoShiftM,CargoOffsetM.Size2D());
            CargoUnsafe|=PeakCargoShiftM>FMath::Max(.001,I.CargoShiftLimitM) || RestraintOverload;
        }
        U+=(Ax+OldYaw*OldV)*Dt; V+=(Ay-OldYaw*OldU)*Dt;
        const double Iz=TotalMassKg*(C.WheelbaseM*C.WheelbaseM+C.TrackM*C.TrackM)/12.;
        YawRate+=Moment/Iz*Dt;
        const double Height=FMath::Max(.1,CombinedCoGM.Z);
        const double Ir=TotalMassKg*(C.TrackM*C.TrackM/12.+Height*Height);
        const double Ip=TotalMassKg*(C.WheelbaseM*C.WheelbaseM/12.+Height*Height);
        // Semi-implicit spring/damper integration. Static CoG offset is already
        // carried in the static load split, not added again as a second moment.
        RollRate=(RollRate+Dt*(TotalMassKg*Ay*Height-C.RollStiffnessNmPerRad*Roll)/Ir)/(1+Dt*C.RollDampingNmsPerRad/Ir);
        PitchRate=(PitchRate+Dt*(TotalMassKg*Ax*Height-C.PitchStiffnessNmPerRad*Pitch)/Ip)/(1+Dt*C.PitchDampingNmsPerRad/Ip);
        Roll+=RollRate*Dt; Pitch+=PitchRate*Dt;
        const double MidHeading=Heading+(OldYaw+YawRate)*.25*Dt;
        const double Forward=(OldU+U)*.5*Dt,Side=(OldV+V)*.5*Dt;
        const double NewHeading=Heading+(OldYaw+YawRate)*.5*Dt;
        if(I.CargoContactEnabled)RefreshLoads(C,I);
        auto RotatedCoG=[](double H,FVector CoG){return FVector(FMath::Cos(H)*CoG.X-FMath::Sin(H)*CoG.Y,
            FMath::Sin(H)*CoG.X+FMath::Cos(H)*CoG.Y,0);};
        // U/V describe the combined CoG. Return displacement of the geometric
        // chassis origin, avoiding spurious translation as eccentric mass yaws.
        const FVector OriginStep=FVector(FMath::Cos(MidHeading)*Forward-FMath::Sin(MidHeading)*Side,
            FMath::Sin(MidHeading)*Forward+FMath::Cos(MidHeading)*Side,0)+RotatedCoG(Heading,OldCoG)-RotatedCoG(NewHeading,CombinedCoGM);
        PositionM+=OriginStep;
        Heading+=(OldYaw+YawRate)*.5*Dt;
        const double Distance=I.CargoContactEnabled?OriginStep.Size2D():FMath::Sqrt(Forward*Forward+Side*Side);
        GroundDistanceM+=Distance; SideSlipDistanceM+=FMath::Abs(Side);
        double MeanOmega=0; for(const auto& W:Wheels) MeanOmega+=W.Omega/6.;
        WheelDistanceM+=MeanOmega*C.WheelRadiusM*Dt;
        EffectiveRadiusM=FMath::Abs(YawRate)>1.e-5?FMath::Abs(U/YawRate):0;
        IMUSpecificForceMps2=FVector(Ax,Ay,PortFriction::Gravity)-(CargoRelativeVelocityMps-RelativeV)*(I.CargoContactEnabled?CargoFraction/Dt:0);
        PeakSlip=FMath::Max(PeakSlip,MaxSlip);
        if(Braking) { BrakingElapsed+=Dt; BrakingDistanceM+=Distance; }
        if(FMath::Abs(Target)<.001 && FMath::Abs(U)<.005 && FMath::Abs(V)<.005 && FMath::Abs(YawRate)<.005 &&
            (!I.CargoContactEnabled || CargoRelativeVelocityMps.Size2D()<.005))
        {
            if(Braking) { LastBrakeValid=true; LastBrakingSeconds=BrakingElapsed; LastBrakingDistanceM=BrakingDistanceM; Braking=false; }
            U=V=YawRate=0; for(auto& W:Wheels) W.Omega=0;
        }
        RefreshLoads(C,I);
        if(!Finite()) { Fault=TEXT("AGV tire/roll dynamics became non-finite"); ConstrainStop(); break; }
    }
}
