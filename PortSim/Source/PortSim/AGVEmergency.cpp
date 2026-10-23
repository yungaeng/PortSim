#include "AGVEmergency.h"

bool FAGVEmergencySettings::Valid() const
{
    for(double V:{double(WorkerDistanceM),double(WorkerRadiusM),double(ScanPeriodSeconds),double(ReactionSeconds),
        double(BrakeDemandMps2),double(CargoShiftLimitM),double(RestraintCapacityN),double(TimeoutSeconds),double(Wetness),double(Ice)})
        if(!FMath::IsFinite(V))return false;
    return WorkerDistanceM>0 && WorkerDistanceM<=100 && WorkerRadiusM>0 && WorkerRadiusM<=1 &&
        ScanPeriodSeconds>=.001 && ScanPeriodSeconds<=1 && ReactionSeconds>=0 && ReactionSeconds<=3 &&
        BrakeDemandMps2>0 && BrakeDemandMps2<=10 && CargoShiftLimitM>0 && CargoShiftLimitM<=1 &&
        RestraintCapacityN>=0 && TimeoutSeconds>=1 && TimeoutSeconds<=120 && Wetness>=0 && Wetness<=1 &&
        Ice>=0 && Ice<=1 && !CargoCoGOffsetM.ContainsNaN();
}

void FAGVEmergencyState::Start(double Speed,double Gap,bool Secured)
{
    *this=FAGVEmergencyState(); Active=true; InitialSpeedMps=Speed; GapM=MinimumClearanceM=Gap;
    CargoSecured=Secured; Outcome=TEXT("worker_entered");
}

void FAGVEmergencyState::Step(const FAGVEmergencySettings& C,const FAGVDynamicsSettings& Settings,
    FAGVDynamicsInput I,FAGVDynamicsState& D,double Gap,double Side,double Range,double Dt)
{
    if(!Active || Finished || Dt<=0)return;
    if(!C.Valid() || !FMath::IsFinite(Dt) || Dt>1 || !FMath::IsFinite(Gap) || !FMath::IsFinite(Side) || !FMath::IsFinite(Range))
    { Invalidate(); return; }
    GapM=Gap; Elapsed+=Dt; ScanClock+=Dt;
    if(!Detected && ScanClock+1.e-9>=C.ScanPeriodSeconds)
    {
        ScanClock=FMath::Fmod(ScanClock,double(C.ScanPeriodSeconds));
        if(Gap>=0 && Gap<=Range && FMath::Abs(Side)<=2.1+C.WorkerRadiusM)
        { Detected=true; DetectedAt=Elapsed; Outcome=TEXT("detected_reaction_pending"); }
    }
    if(Detected && !BrakeCommand && Elapsed-DetectedAt+1.e-9>=C.ReactionSeconds)
    { BrakeCommand=true; BrakeAt=Elapsed; D.Braking=false; D.LastBrakeValid=false; Outcome=TEXT("physical_braking"); }
    I.TargetSpeedMps=BrakeCommand?0:InitialSpeedMps;
    I.BrakeDemandMps2=C.BrakeDemandMps2;
    I.CargoContactEnabled=true; I.CargoSecured=CargoSecured;
    I.CargoShiftLimitM=C.CargoShiftLimitM; I.RestraintCapacityN=C.RestraintCapacityN;
    D.Step(Settings,I,Dt);
    if(Elapsed>=C.TimeoutSeconds){Finished=true;Outcome=TEXT("timeout");}
}

void FAGVEmergencyState::Record(const FAGVDynamicsState& D,double Gap,double Distance)
{
    if(!Active || Invalidated)return;
    GapM=Gap; MinimumClearanceM=FMath::Min(MinimumClearanceM,Gap);
    TravelM+=Distance; if(BrakeCommand && !Stopped)BrakingDistanceM+=Distance;
    PeakSlip=FMath::Max(PeakSlip,D.MaxSlip); PeakLTR=FMath::Max(PeakLTR,FMath::Abs(D.LoadTransferRatio));
    PeakInertiaN=FMath::Max(PeakInertiaN,D.CargoInertialForceN.Size2D());
    PeakRestraintN=FMath::Max(PeakRestraintN,D.RestraintForceN.Size2D()); PeakCargoShiftM=FMath::Max(PeakCargoShiftM,D.PeakCargoShiftM);
    if(Gap<=0){Collision=true;Finished=true;Outcome=TEXT("worker_contact");}
    else if(!D.Fault.IsEmpty() || !D.Finite()){Finished=true;Outcome=TEXT("dynamics_invalid");}
    else if(D.CargoUnsafe){Finished=true;Outcome=D.RestraintOverload?TEXT("restraint_overload"):TEXT("cargo_shift_limit");}
    else if(D.OverturnThreshold){Finished=true;Outcome=TEXT("wheel_contact_limit");}
    else if(BrakeCommand && FMath::Abs(D.U)<.005 && FMath::Abs(D.V)<.005 && FMath::Abs(D.YawRate)<.005 && D.CargoRelativeVelocityMps.Size2D()<.005)
    { Stopped=true;Finished=true;StoppedAt=Elapsed;Outcome=TEXT("stopped_clear"); }
}

void FAGVEmergencyState::Invalidate()
{ Invalidated=true;Finished=true;Outcome=TEXT("external_constraint_invalidated"); }
