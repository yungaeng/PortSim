#if WITH_DEV_AUTOMATION_TESTS
#include "YardStackingPolicy.h"
#include "ContainerSpecification.h"
#include "ShipStowagePlanning.h"
#include "CraneTravelBraking.h"
#include "STSOperatingProfile.h"
#include "Misc/AutomationTest.h"
#include "Misc/Paths.h"
#include "TerminalLayout.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTrialTravelDeadlineTest,"PortSim.Stowage.TrialTravelAndVerificationDeadlines",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FTrialTravelDeadlineTest::RunTest(const FString& Parameters)
{
    FSTSPickupConfig Config; Config.TrialLiftTimeout=90;
    FSTSPickupController Controller; Controller.Attached(FVector::ZeroVector);
    FSTSObservation Observation; Observation.bValid=true;
    for(bool& Lock:Observation.Locked) Lock=true;
    for(float& Force:Observation.CornerLoadsN) Force=12000*9.80665/4;
    Observation.SpreaderPosition=FVector(Config.TrialHorizontalTolerance+1,0,Config.TrialHeight);
    Observation.Timestamp=61;
    Controller.Update(Config,Observation,61,61,.15,FVector::ZeroVector,65000);
    TestTrue(TEXT("Unsettled travel remains bounded and cannot permit full hoist"),
        Controller.Fault.IsEmpty() && Controller.Phase==ESTSPickupPhase::TrialLift && !Controller.EstimateValid);
    Observation.SpreaderPosition.X=0; Observation.Timestamp=62;
    Controller.Update(Config,Observation,62,1,.15,FVector::ZeroVector,65000);
    TestTrue(TEXT("Measured settling begins the separate load verification"),Controller.Phase==ESTSPickupPhase::TrialHold);
    auto Unbalanced=Controller;
    Observation.CornerLoadsN[0]=0; Observation.Timestamp=123;
    Unbalanced.Update(Config,Observation,123,61,.15,FVector::ZeroVector,65000);
    TestFalse(TEXT("Extended travel never extends or bypasses load verification"),Unbalanced.Fault.IsEmpty());
    for(float& Force:Observation.CornerLoadsN) Force=12000*9.80665/4;
    Observation.Timestamp=63.1;
    Controller.Update(Config,Observation,63.1,1.1,.15,FVector::ZeroVector,65000);
    TestTrue(TEXT("Only a measured stable four-corner load permits full hoist"),Controller.EstimateValid);
    FSTSPickupController Stuck; Stuck.Attached(FVector::ZeroVector);
    Observation.Timestamp=91; Observation.SpreaderPosition.X=Config.TrialHorizontalTolerance+1;
    Stuck.Update(Config,Observation,91,91,.15,FVector::ZeroVector,65000);
    TestFalse(TEXT("Travel still fails at its explicit deadline"),Stuck.Fault.IsEmpty());
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCraneSwayStoppingTest,"PortSim.Stowage.MeasuredSwayStopping",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FCraneSwayStoppingTest::RunTest(const FString& Parameters)
{
    FSTSOperatingProfile Profile;
    if(!Profile.LoadRMG(FPaths::ProjectConfigDir()/TEXT("RMG_Simulation.json")))
    { AddError(Profile.Error); return false; }
    auto Config=Profile.Dynamics;
    Config.Wind=FVector(-7.234,-3.275,0);
    double MaximumDegrees=0;
    for(const auto& Mount:Config.Mounts) if(Mount.Key==TEXT("sway_sensor")) MaximumDegrees=Mount.Maximum;
    FSTSSuspension Plant;
    Plant.Angle=FVector2D(-.046829,-.038045); Plant.Rate=FVector2D(-.073462,-.069151);
    FVector Velocity=FVector::ZeroVector,Distance=FVector::ZeroVector;
    FVector2D MeasuredAngle=Plant.Angle,MeasuredRate=Plant.Rate;
    const FVector2D Available(Profile.TrolleyAcceleration*.01,Profile.GantryAcceleration*.01);
    constexpr double Step=1./120.,Length=7.;
    for(int32 N=0;N<1440;++N)
    {
        if(N%6==0) { MeasuredAngle=Plant.Angle; MeasuredRate=Plant.Rate; }
        const double Age=(N%6)*Step;
        const FVector WindAcceleration=Config.AerodynamicForce(Velocity)/Profile.SpreaderMassKg;
        const FVector2D Command=CraneTravelBraking::SwayAcceleration(MeasuredAngle+MeasuredRate*Age,
            MeasuredRate,FVector2D(WindAcceleration.X,WindAcceleration.Y),Available,Available,
            Length,0,Config.Damping,FMath::DegreesToRadians(MaximumDegrees),Step);
        if(FMath::Abs(Command.X)>Available.X+1.e-9 || FMath::Abs(Command.Y)>Available.Y+1.e-9)
        { AddError(TEXT("Sway braking exceeds the real motor acceleration")); return false; }
        const FVector Previous=Velocity;
        Velocity.X=FMath::Clamp(Velocity.X+Command.X*Step,-Profile.TrolleySpeed*.01,Profile.TrolleySpeed*.01);
        Velocity.Y=FMath::Clamp(Velocity.Y+Command.Y*Step,-Profile.GantrySpeed*.01,Profile.GantrySpeed*.01);
        Distance+=Velocity*Step;
        Plant.Step(Config,Step,Length,0,(Velocity-Previous)/Step,Velocity,Profile.SpreaderMassKg,
            FVector::ZeroVector,Profile.HoistPowerW);
        if(!Plant.Fault.IsEmpty() || Plant.SwayDegrees()>MaximumDegrees+.01)
        { AddError(TEXT("Combined diagonal sway escaped the configured sensor range")); return false; }
    }
    TestTrue(TEXT("Sensor-safe braking still makes forward progress"),Distance.X>5 && Distance.Y>5);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCraneTravelStoppingTest,"PortSim.Stowage.CraneTravelStopping",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FCraneTravelStoppingTest::RunTest(const FString& Parameters)
{
    // A full-speed drive approaching from either side must stay in its
    // correction stroke even if the sway controller keeps requesting thrust.
    for(double Direction:{-1.,1.})
    {
        double Position=-Direction*4000.,Velocity=Direction*600.;
        constexpr double Step=1./120.,Radius=250.,MotorAcceleration=100.;
        for(int32 N=0;N<2400;++N)
        {
            const double Command=CraneTravelBraking::Acceleration(-Position,Velocity,
                Direction*MotorAcceleration,MotorAcceleration,Radius,Step);
            TestTrue(TEXT("Stopping never exceeds motor acceleration"),FMath::Abs(Command)<=MotorAcceleration);
            Velocity+=Command*Step;
            Velocity=FMath::Clamp(Velocity,-600.,600.);
            Position+=Velocity*Step;
            if(Direction*Position>Radius+1.)
            { AddError(TEXT("Full-speed drive escaped the stopping/correction stroke")); return false; }
        }
    }
    TestEqual(TEXT("Low-speed anti-sway correction remains available"),
        CraneTravelBraking::Acceleration(0,10,30,100,250,1./120.),30.);
    TestEqual(TEXT("Opposite correction remains available"),
        CraneTravelBraking::Acceleration(0,-10,-30,100,250,1./120.),-30.);
    TestEqual(TEXT("No actuator acceleration is invented"),
        CraneTravelBraking::Acceleration(100,50,100,0,250,1./120.),0.);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FYardBookingPolicyTest,"PortSim.Stowage.YardSafetyAndRetrieval",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FYardBookingPolicyTest::RunTest(const FString& Parameters)
{
    for(int32 Bay=0;Bay<30;Bay+=2) if(TerminalLayout::IsOperationalYardBay(Bay))
    {
        const int32 Ft=TerminalLayout::YardSizeFt(Bay);
        TestTrue(TEXT("Each bay has one dedicated supported size"),Ft==20 || Ft==40 || Ft==45);
        if(Ft==20)
        {
            const float Left=TerminalLayout::YardPadOffsetCm(Ft,0),Right=TerminalLayout::YardPadOffsetCm(Ft,1);
            TestTrue(TEXT("Two 20ft boxes leave a physical gap"),Right-Left-TerminalLayout::YardLengthCm(20)>7.5f);
            TestTrue(TEXT("Both pads fit inside one 40ft cell"),
                FMath::IsNearlyEqual(Right-Left+TerminalLayout::YardLengthCm(20),1219.2f,.01f));
        }
        else TestEqual(TEXT("Long boxes occupy the whole cell"),TerminalLayout::YardPads(Ft),1);
    }
    TestTrue(TEXT("Split 20ft capacity counts individual boxes"),TerminalLayout::ReceivingSlotCount()>TerminalLayout::ReceivingSlotCount(false));
    FPortCargoInformation Lower,Upper;
    Lower.OutboundOrder=3; Upper.OutboundOrder=1;
    TestTrue(TEXT("Earlier outbound light box can go above later heavy box"),YardStackingPolicy::MayStack(Lower,40,25000,Upper,40,15000));
    Upper.OutboundOrder=4;
    TestFalse(TEXT("Do not bury an earlier booking below a later one"),YardStackingPolicy::MayStack(Lower,40,25000,Upper,40,15000));
    Upper.OutboundOrder=1;
    TestFalse(TEXT("Heavy upper rejected despite earlier outbound order"),YardStackingPolicy::MayStack(Lower,40,15000,Upper,40,25000));
    TestFalse(TEXT("Different corner footprints cannot share a stack"),YardStackingPolicy::MayStack(Lower,20,25000,Upper,40,15000));
    TestFalse(TEXT("45ft cannot stand on a 20ft pad"),YardStackingPolicy::MayStack(Lower,20,25000,Upper,45,15000));
    TestFalse(TEXT("45ft cannot stand on a 40ft pad"),YardStackingPolicy::MayStack(Lower,40,25000,Upper,45,15000));
    Upper.HazardGroup=TEXT("flammable");
    TestFalse(TEXT("Hazard group mismatch rejected"),YardStackingPolicy::MayStack(Lower,40,25000,Upper,40,15000));
    TestTrue(TEXT("Flammable simulation zone"),YardStackingPolicy::ZoneAccepts(21,Upper));
    TestFalse(TEXT("Flammable cannot use dry/reefer/corrosive zones"),YardStackingPolicy::ZoneAccepts(22,Upper));
    Upper.HazardGroup=TEXT("none"); Upper.HandlingType=TEXT("reefer");
    TestFalse(TEXT("Reefer cannot enter an unpowered simulation zone"),YardStackingPolicy::ZoneAccepts(4,Upper));
    TestTrue(TEXT("Reefer allocated to designated zone"),YardStackingPolicy::ZoneAccepts(19,Upper));
    Upper.HandlingType=TEXT("dry"); Lower.PermittedSuperimposedKg=30000;
    TestTrue(TEXT("Superimposed load boundary accepted"),YardStackingPolicy::MayStack(Lower,40,25000,Upper,40,15000,15000));
    TestFalse(TEXT("Accumulated upper load is checked"),YardStackingPolicy::MayStack(Lower,40,25000,Upper,40,15000,15001));
    for(int32 I=2000;I<2600;++I) for(int32 Size:{20,40,45})
        TestTrue(TEXT("Generated booking is valid"),FPortCargoInformation::ForSequence(I,Size).Valid());
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPairedTransportPolicyTest,"PortSim.Stowage.PairedTransportEconomics",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FPairedTransportPolicyTest::RunTest(const FString& Parameters)
{
    TestTrue(TEXT("Two 20ft footprints fit separate deck pads"),YardStackingPolicy::PairFits(20,20,605.8,605.8,15000,17000,65000));
    TestFalse(TEXT("Two heavy boxes cannot exceed vehicle capacity"),YardStackingPolicy::PairFits(20,20,605.8,605.8,33000,33000,65000));
    TestFalse(TEXT("40ft is never disguised as one of the two 20ft boxes"),YardStackingPolicy::PairFits(20,40,605.8,1219.2,15000,17000,65000));
    TestFalse(TEXT("Oversize pair must not overlap on deck"),YardStackingPolicy::PairFits(20,20,700,700,15000,17000,65000));
    TestTrue(TEXT("Long transport can repay second pickup wait"),YardStackingPolicy::PairedTripSavingSeconds(250,200,180,15)>YardStackingPolicy::MinimumPairSavingSeconds);
    TestTrue(TEXT("Short trip remains individual"),YardStackingPolicy::PairedTripSavingSeconds(40,35,180,15)<0);
    TestTrue(TEXT("Pair dock outside other STS departure paths is usable"),YardStackingPolicy::DockOutsideThroughPaths(0,{FVector2D(3000,8000)},1800));
    TestFalse(TEXT("Pair loading cannot obstruct another STS departure"),YardStackingPolicy::DockOutsideThroughPaths(5000,{FVector2D(3000,8000)},1800));
    FPortCargoInformation A,B; B.HazardGroup=TEXT("corrosive");
    TestFalse(TEXT("No paired transport of simulated DG"),YardStackingPolicy::PairBookingsCompatible(A,B));
    A.HandlingType=B.HandlingType=TEXT("reefer"); B.HazardGroup=TEXT("none"); A.TemperatureC=4; B.TemperatureC=-18;
    TestFalse(TEXT("Different reefer temperature bookings are not paired"),YardStackingPolicy::PairBookingsCompatible(A,B));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FShipStowageTest,"PortSim.Stowage.ShipGMStackAndLashing",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FShipStowageTest::RunTest(const FString& Parameters)
{
    FPortShipStowageAssumptions A; A.Bays=A.Rows=1; A.LightshipKg=10000000; A.MaximumLashingN=1000000;
    FPortCargoInformation Early,Late,Middle; Early.PortCallOrder=1; Late.PortCallOrder=3; Middle.PortCallOrder=2;
    TArray<FPortShipPlanningCargo> Cargo={{1,40,10000,Early},{2,40,30000,Late},{3,40,20000,Middle}};
    const auto P=PlanShipStowage(Cargo,A);
    TestTrue(TEXT("Valid port and weight ordering has a plan"),P.Feasible);
    if(P.Placements.Num()!=3) { AddError(TEXT("Missing complete ship plan")); return false; }
    TestEqual(TEXT("Latest port loaded at bottom"),P.Placements[0].CargoID,2);
    TestEqual(TEXT("Earliest port accessible at top"),P.Placements[2].CargoID,1);
    const double Moment=10000000*9.+30000*(12+.5*2.591)+20000*(12+1.5*2.591)+10000*(12+2.5*2.591);
    const double ExpectedGM=12.-Moment/10060000.-3000000./10060000.;
    TestTrue(TEXT("GM includes vertical weight moments and free surface correction"),FMath::IsNearlyEqual(P.CorrectedGMM,ExpectedGM,1.e-9));
    A.MaximumStackKg=40000;
    TestFalse(TEXT("Stack capacity blocks otherwise well-ordered cargo"),PlanShipStowage(Cargo,A).Feasible);
    A.MaximumStackKg=150000; A.MaximumLashingN=1;
    TestFalse(TEXT("Lashing moment limit blocks the lift plan"),PlanShipStowage(Cargo,A).Feasible);
    A.MaximumLashingN=1000000; A.MinimumGMM=4;
    TestFalse(TEXT("Unsafe GM cannot be overruled by destination priority"),PlanShipStowage(Cargo,A).Feasible);
    A.MinimumGMM=.5; const auto Duplicate=Cargo[0]; Cargo.Add(Duplicate);
    TestFalse(TEXT("Duplicate cargo IDs are rejected"),PlanShipStowage(Cargo,A).Feasible);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FShipLargeCapacityTest,"PortSim.Stowage.ThreeThousandContainerVessel",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FShipLargeCapacityTest::RunTest(const FString& Parameters)
{
    FPortShipStowageAssumptions A; FString Error;
    if(!TestTrue(TEXT("Expanded vessel assumptions load"),A.Load(FPaths::ProjectConfigDir()/TEXT("Stowage_Planning.json"),Error))) return false;
    A.Bays=TerminalLayout::VesselBays; A.Rows=TerminalLayout::VesselRows; A.Tiers=TerminalLayout::VesselTiers;
    TArray<FPortShipPlanningCargo> Cargo;
    for(int32 I=0;I<TerminalLayout::ContainersPerVessel;++I)
    {
        const auto Spec=PortContainerSpecification::ForSequence(2000+I,I,TerminalLayout::ContainersPerVessel);
        Cargo.Add({2000+I,Spec.LengthFt,Spec.GrossMassKg(),FPortCargoInformation::ForSequence(2000+I,Spec.LengthFt)});
    }
    const auto Plan=PlanShipStowage(Cargo,A);
    if(!TestTrue(TEXT("3000 mixed-size bookings have a simultaneous supported plan"),Plan.Feasible))
    { AddError(Plan.Reason); return false; }
    TestEqual(TEXT("No booking is hidden or dropped"),Plan.Placements.Num(),3000);
    FSTSOperatingProfile Profile;
    if(!TestTrue(TEXT("STS profile loads"),Profile.Load(FPaths::ProjectDir()/TEXT("../Document/STS/STS_ReferenceData.json"),FPaths::ProjectConfigDir()/TEXT("STS_Simulation.json")))) return false;
    TSet<FIntVector> Slots;
    for(const auto& P:Plan.Placements)
    {
        const auto Spec=PortContainerSpecification::ForSequence(P.CargoID,P.CargoID-2000,3000);
        TestFalse(TEXT("Physical subslots are unique"),Slots.Contains(FIntVector((P.Bay-1)*2+(P.Subslot==2),P.Row,P.Tier)));
        Slots.Add(FIntVector((P.Bay-1)*2+(P.Subslot==2),P.Row,P.Tier));
        TestTrue(TEXT("Cargo stays inside the deck width"),FMath::Abs(P.PositionM.Y)+Spec.WidthCm*.005<=22.);
        TestTrue(TEXT("Cargo clears the expanded bow and aft superstructure"),FMath::Abs(P.PositionM.X)+Spec.LengthCm*.005<=154.9);
        const FVector CraneLocal((TerminalLayout::VesselCenterXM+P.PositionM.Y)*100.-1200.,0,P.PositionM.Z*100.+Spec.HeightCm*.5);
        TestTrue(TEXT("Every box is inside existing STS trolley and lift limits"),Profile.ContainsTarget(CraneLocal));
        TestTrue(TEXT("Stack and lashing limits remain enforced"),P.StackKg<=A.MaximumStackKg && P.LashingN<=A.MaximumLashingN);
    }
    TestTrue(TEXT("Stability limits remain enforced"),Plan.CorrectedGMM>=A.MinimumGMM && Plan.CorrectedGMM<=A.MaximumGMM && Plan.HeelDegrees<=A.MaximumHeelDegrees);
    return true;
}
#endif
