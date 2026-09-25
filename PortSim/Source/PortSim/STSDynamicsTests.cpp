#if WITH_DEV_AUTOMATION_TESTS
#include "STSDynamics.h"
#include "STSOperatingProfile.h"
#include "Misc/AutomationTest.h"
#include "Misc/Paths.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSTSDynamicsTest,"PortSim.STS.SuspensionAndDrives",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FSTSDynamicsTest::RunTest(const FString& Parameters)
{
    FSTSOperatingProfile P;
    if(!TestTrue(TEXT("Read full assumed dynamics profile"),P.Load(FPaths::ProjectDir()/TEXT("../Document/STS/STS_ReferenceData.json"),FPaths::ProjectConfigDir()/TEXT("STS_Simulation.json")))) return false;
    auto C=P.Dynamics; FSTSSuspension D;
    D.Step(C,.05,20,0,FVector::ZeroVector,FVector::ZeroVector,17000,FVector::ZeroVector,P.HoistPowerW);
    double Total=0;for(double T:D.Tension)Total+=T*C.Parts;
    TestTrue(TEXT("Static vertical rope forces balance suspended weight"),FMath::Abs(Total-17000*9.80665)<.01);
    const double Expected=17000*9.80665*C.DrumRadius/(C.Parts*C.GearRatio*C.Efficiency*C.MotorCount);
    TestTrue(TEXT("Per-motor static shaft torque includes ratio and efficiency"),FMath::Abs(D.MotorTorque-Expected)<.01);
    FSTSSuspension E;E.Step(C,.05,20,0,FVector::ZeroVector,FVector::ZeroVector,17000,FVector(.3,0,0),P.HoistPowerW);
    TestTrue(TEXT("Eccentric load changes individual line tensions"),E.Tension[1]>E.Tension[0]);
    FSTSSuspension Up;Up.Step(C,.05,20,-1,FVector(0,0,1),FVector(0,0,1),17000,FVector::ZeroVector,P.HoistPowerW);
    TestTrue(TEXT("Upward acceleration raises line tension and motor torque"),Up.Tension[0]>D.Tension[0] && Up.MotorTorque>D.MotorTorque);
    FSTSSuspension Horizontal;Horizontal.Step(C,.01,20,0,FVector(1,0,0),FVector::ZeroVector,17000,FVector::ZeroVector,P.HoistPowerW);
    TestTrue(TEXT("Below-plane mass transfers load toward positive acceleration"),Horizontal.Tension[1]>Horizontal.Tension[0]);
    FSTSSuspension On,Off;On.Yaw=Off.Yaw=.08;
    auto NoControl=C;NoControl.AntiSkew=false;
    for(int32 I=0;I<2400;++I) {On.Step(C,1./120.,20,0,FVector::ZeroVector,FVector::ZeroVector,17000,FVector::ZeroVector,P.HoistPowerW);Off.Step(NoControl,1./120.,20,0,FVector::ZeroVector,FVector::ZeroVector,17000,FVector::ZeroVector,P.HoistPowerW);}
    TestTrue(TEXT("Skew controller removes an initial yaw disturbance"),FMath::Abs(On.Yaw)<.001 && FMath::Abs(On.Yaw)+FMath::Abs(On.YawRate)<FMath::Abs(Off.Yaw)+FMath::Abs(Off.YawRate));
    auto RunSway=[&](bool Controlled,double H)
    {
        FSTSSuspension S;S.Angle.X=.05;double Velocity=0;
        for(int32 I=0;I<FMath::RoundToInt(30/H);++I)
        {
            const double A=Controlled?FMath::Clamp(3.*20*S.Rate.X,-1.5,1.5):0;
            Velocity+=A*H;
            S.Step(C,H,20,0,FVector(A,0,0),FVector(Velocity,0,0),17000,FVector::ZeroVector,P.HoistPowerW);
        }
        return S.Angle.SizeSquared()+S.Rate.SizeSquared();
    };
    TestTrue(TEXT("Positive trolley acceleration feedback damps sway"),RunSway(true,1./120.)<RunSway(false,1./120.));
    FSTSSuspension Fine,Coarse;Fine.Angle.X=Coarse.Angle.X=.05;
    for(int32 I=0;I<1200;++I)Fine.Step(C,1./120.,20,0,FVector::ZeroVector,FVector::ZeroVector,17000,FVector::ZeroVector,P.HoistPowerW);
    for(int32 I=0;I<200;++I)Coarse.Step(C,.05,20,0,FVector::ZeroVector,FVector::ZeroVector,17000,FVector::ZeroVector,P.HoistPowerW);
    TestTrue(TEXT("Fixed internal steps are consistent across outer tick rates"),FMath::Abs(Fine.Angle.X-Coarse.Angle.X)<.0001);
    for(double Length:{12.,25.,40.})
    {
        FSTSSuspension Travel; double X=0,Velocity=0,Peak=0;
        for(int32 I=0;I<36000;++I)
        {
            constexpr double H=1./120.;
            const double Command=C.HorizontalAcceleration(40-X-Travel.Offset.X,Velocity,Length,Travel.Rate.X);
            const double Next=FMath::Clamp(Velocity+FMath::Clamp(Command,-.8,.8)*H,-4.,4.);
            const double A=(Next-Velocity)/H;Velocity=Next;X+=Velocity*H;
            Travel.Step(C,H,Length,0,FVector(A,0,0),FVector(Velocity,0,0),17000,FVector::ZeroVector,P.HoistPowerW);
            Peak=FMath::Max(Peak,Travel.SwayDegrees());
        }
        TestTrue(FString::Printf(TEXT("40m closed-loop travel settles at rope length %.0fm"),Length),FMath::Abs(40-X-Travel.Offset.X)<.05 && FMath::Abs(Velocity)<.05 && Peak<20 && Travel.Fault.IsEmpty());
    }
    auto WindConfig=C;WindConfig.Wind.X=5;FSTSSuspension WindState;
    for(int32 I=0;I<1200;++I)WindState.Step(WindConfig,1./120.,20,0,FVector::ZeroVector,FVector::ZeroVector,17000,FVector::ZeroVector,P.HoistPowerW);
    TestTrue(TEXT("Crosswind changes suspended position"),WindState.Offset.X>.001);
    auto LimitedMotor=C;LimitedMotor.MotorMaxTorque=1;FSTSSuspension MotorState;
    MotorState.Step(LimitedMotor,.05,20,0,FVector::ZeroVector,FVector::ZeroVector,17000,FVector::ZeroVector,P.HoistPowerW);
    TestTrue(TEXT("Motor saturation is exposed independently of rope limits"),MotorState.Saturated);
    auto LowLimit=C;LowLimit.RopeLimit=1;FSTSSuspension Fault;
    Fault.Step(LowLimit,.05,20,0,FVector::ZeroVector,FVector::ZeroVector,17000,FVector::ZeroVector,P.HoistPowerW);
    TestFalse(TEXT("Rope overload faults instead of masking forces"),Fault.Fault.IsEmpty());
    TestEqual(TEXT("All sensor families have mounted/ranged assumptions"),C.Mounts.Num(),14);
    const auto* Boom=C.Mounts.FindByPredicate([](const FSTSSensorMount& M){return M.Key==TEXT("boom_collision_lidar");});
    const auto* Encoder=C.Mounts.FindByPredicate([](const FSTSSensorMount& M){return M.Key==TEXT("trolley_encoder");});
    if(!TestNotNull(TEXT("AOS reference present"),Boom) || !TestNotNull(TEXT("KH53 reference present"),Encoder))return false;
    TestEqual(TEXT("AOS supports the manufacturer's scan wider than 180 degrees"),Boom->Fov,190.);
    TestEqual(TEXT("AOS nominal range remains distinct from 10 percent remission range"),Boom->Maximum,80.);
    TestEqual(TEXT("Two boom scanners use the gantry frame"),Boom->Frame,FString(TEXT("gantry")));
    TestEqual(TEXT("Second boom scanner is instantiated"),Boom->AdditionalPositions.Num(),1);
    TestTrue(TEXT("Boom scanner positions straddle the boom"),Boom->Position.Y*Boom->AdditionalPositions[0].Y<0);
    TestTrue(TEXT("KH53 reports 0.1mm increments relative to its configured zero"),FMath::Abs(Encoder->ReadPosition(Encoder->MeasurementOrigin+.00016)-.0002)<1.e-8);
    TestTrue(TEXT("Signed trolley coordinate maps to nonnegative device travel"),Encoder->ReadPosition(-50)>0);
    TestEqual(TEXT("KH53 speed validity limit"),Encoder->MaxTraversingSpeed,6.6);
    return true;
}
#endif
