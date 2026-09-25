#include "STSPickupController.h"
#include "STSOperatingProfile.h"
#include "Dom/JsonObject.h"

bool FSTSPickupConfig::Load(TSharedPtr<FJsonObject> Root,FString& Error)
{
    const TSharedPtr<FJsonObject>* Value=nullptr;
    if(!Root || !Root->TryGetObjectField(TEXT("pickup_control"),Value)) {Error=TEXT("Missing pickup_control assumptions");return false;}
    auto O=*Value;
    auto N=[&](const TCHAR* Key,double& V,double Scale=1.)
    {double X=0;if(!O->TryGetNumberField(Key,X)||!FMath::IsFinite(X)||X<=0){Error=FString(TEXT("Invalid pickup setting: "))+Key;return false;} V=X*Scale;return true;};
    double Count=0;
    if(!N(TEXT("corner_tolerance_m"),CornerTolerance,100)||!N(TEXT("vertical_tolerance_m"),VerticalTolerance,100)||
        !N(TEXT("relative_speed_mps"),RelativeSpeed,100)||!N(TEXT("yaw_tolerance_deg"),YawTolerance)||!N(TEXT("tilt_tolerance_deg"),TiltTolerance)||
        !N(TEXT("seat_time_s"),SeatTime)||!N(TEXT("lock_time_s"),LockTime)||!N(TEXT("lock_timeout_s"),LockTimeout)||
        !N(TEXT("alignment_timeout_s"),AlignmentTimeout)||!N(TEXT("acquisition_radius_m"),AcquisitionRadius,100)||
        !N(TEXT("trial_height_m"),TrialHeight,100)||!N(TEXT("trial_hold_s"),TrialHold)||!N(TEXT("trial_timeout_s"),TrialTimeout)||
        !N(TEXT("minimum_corner_fraction"),MinimumCornerFraction)||!N(TEXT("mass_stability_fraction"),MassStability)||!N(TEXT("max_attempts"),Count))return false;
    for(int32 I=0;I<3;++I)
    {
        const TCHAR* Key=I==0?TEXT("pose_bias_x_m"):I==1?TEXT("pose_bias_y_m"):TEXT("pose_bias_z_m");
        double X=0;if(!O->TryGetNumberField(Key,X)||!FMath::IsFinite(X)||FMath::Abs(X)>1){Error=TEXT("Invalid pickup pose bias");return false;}PoseBias[I]=X*100;
    }
    if(Count!=FMath::FloorToDouble(Count)||Count>5||LockTimeout<=LockTime||CornerTolerance>10||VerticalTolerance>10||
        TrialHeight<20||TrialHeight>100||MinimumCornerFraction>=.25||MassStability>=.5||YawTolerance>2||TiltTolerance>2||AcquisitionRadius>200)
    {Error=TEXT("Inconsistent pickup assumptions");return false;}
    MaxAttempts=int32(Count);return true;
}

void FSTSPickupController::Enter(ESTSPickupPhase Next,const TCHAR* Why)
{Phase=Next;StableTime=PhaseTime=0;Reason=Why;}
void FSTSPickupController::Fail(const TCHAR* Why)
{Fault=Why;Enter(ESTSPickupPhase::Failed,Why);}
const TCHAR* FSTSPickupController::PhaseName() const
{
    switch(Phase){case ESTSPickupPhase::Align:return TEXT("align");case ESTSPickupPhase::Seat:return TEXT("seat");
    case ESTSPickupPhase::Lock:return TEXT("lock");case ESTSPickupPhase::Attach:return TEXT("attach");
    case ESTSPickupPhase::TrialLift:return TEXT("trial_lift");case ESTSPickupPhase::TrialHold:return TEXT("trial_hold");
    case ESTSPickupPhase::Complete:return TEXT("complete");default:return TEXT("failed");}
}
void FSTSPickupController::Attached(FVector MeasuredSpreader)
{TrialOrigin=MeasuredSpreader;Enter(ESTSPickupPhase::TrialLift,TEXT("Trial lift to configured height; full hoist withheld"));}

void FSTSPickupController::Update(const FSTSPickupConfig& C,const FSTSObservation& O,double Now,double Dt,
    double MaxAge,FVector NominalCargo,double Capacity)
{
    if(Phase==ESTSPickupPhase::Failed||Phase==ESTSPickupPhase::Complete)return;
    PhaseTime+=Dt;
    if(!O.IsFresh(Now,MaxAge)){Fail(TEXT("Required STS sensor observation invalid/stale"));return;}
    const bool Fresh=O.Timestamp>LastSample;
    const double SampleDt=Fresh&&LastSample>=0?FMath::Clamp(O.Timestamp-LastSample,0.,Dt):0;
    if(Fresh)LastSample=O.Timestamp;
    const bool Trial=Phase==ESTSPickupPhase::TrialLift||Phase==ESTSPickupPhase::TrialHold;
    if(!Trial)
    {
        AlignmentTime+=Dt;
        if(AlignmentTime>C.AlignmentTimeout)
        {
            if(Attempts>=C.MaxAttempts){Fail(TEXT("Pickup alignment attempts exhausted"));return;}
            ++Attempts;AlignmentTime=0;
            for(bool& R:RequestLocks)R=false;
            Enter(ESTSPickupPhase::Align,TEXT("Reacquiring target; alignment retry"));
        }
        Target=O.bTargetVisible?O.CargoPosition+FVector(0,0,154.5):O.SpreaderPosition;
        if(O.bTargetVisible&&FVector::Dist(O.CargoPosition,NominalCargo)>C.AcquisitionRadius)
        {Fail(TEXT("Pickup target outside acquisition window"));return;}
        bool Seated=O.bTargetVisible&&FMath::Abs(O.RelativeYawDegrees)<=C.YawTolerance&&O.TargetTiltDegrees<=C.TiltTolerance&&
            (O.SpreaderVelocity-O.CargoVelocity).Size()<=C.RelativeSpeed;
        for(int32 I=0;I<4;++I)Seated &= O.CornerSeated[I]&&O.CornerError[I].Size2D()<=C.CornerTolerance&&FMath::Abs(O.CornerError[I].Z)<=C.VerticalTolerance;
        if(Phase==ESTSPickupPhase::Align)
        {
            Reason=O.bTargetVisible?TEXT("Correcting measured relative position and velocity"):TEXT("Target pose unavailable; holding position");
            if(Seated)Enter(ESTSPickupPhase::Seat,TEXT("Verify four independent seating contacts"));

        }
        else if(Phase==ESTSPickupPhase::Seat)
        {
            if(!Seated){Enter(ESTSPickupPhase::Align,TEXT("Seating lost; correct alignment"));return;}
            StableTime+=SampleDt;
            if(StableTime>=C.SeatTime)Enter(ESTSPickupPhase::Lock,TEXT("Command four locks; await individual feedback"));
        }
        else if(Phase==ESTSPickupPhase::Lock)
        {
            for(int32 I=0;I<4;++I)RequestLocks[I]=Seated&&O.CornerSeated[I];
            if(!Seated){for(bool& R:RequestLocks)R=false;Enter(ESTSPickupPhase::Align,TEXT("Contact lost before attachment"));return;}
            if(O.AllLocked())Enter(ESTSPickupPhase::Attach,TEXT("Four lock feedback signals confirmed"));
            else if(PhaseTime>C.LockTimeout)Fail(TEXT("Twist lock alignment failed"));
        }
        return;
    }
    Target=TrialOrigin+FVector(0,0,C.TrialHeight);
    if(!O.AllLocked()){Fail(TEXT("Trial lift lost lock feedback"));return;}
    if(PhaseTime>C.TrialTimeout){Fail(TEXT("Trial lift verification timed out"));return;}
    const double Sum=O.CornerLoadsN[0]+O.CornerLoadsN[1]+O.CornerLoadsN[2]+O.CornerLoadsN[3];
    const double EffectiveG=9.80665+O.HoistAcceleration*.01;
    const double Mass=EffectiveG>1?Sum/EffectiveG:0;
    if(Mass>Capacity){Fail(TEXT("Measured pickup load exceeds rated capacity"));return;}
    const bool Stationary=O.SpreaderPosition.Equals(Target,1.)&&O.SpreaderVelocity.Size()<=C.RelativeSpeed;
    if(Phase==ESTSPickupPhase::TrialLift)
    {
        if(Stationary&&!O.bCargoSupported)Enter(ESTSPickupPhase::TrialHold,TEXT("Verify suspended load distribution before full hoist"));
        return;
    }
    bool Balanced=Mass>100&&EffectiveG>1&&FMath::Abs(O.HoistAcceleration)<10;
    for(float Load:O.CornerLoadsN)Balanced &= FMath::IsFinite(Load)&&Load>0&&Load>=Sum*C.MinimumCornerFraction;
    if(!Stationary||O.bCargoSupported||!Balanced)
    {StableTime=0;Reason=TEXT("Trial hold: motion, support or corner-load check not satisfied");return;}
    if(!Fresh)return;
    if(EstimatedMass>0&&FMath::Abs(Mass-EstimatedMass)>C.MassStability*EstimatedMass)StableTime=0;
    EstimatedMass=Mass;
    EstimatedCoG=FVector::ZeroVector;
    for(int32 I=0;I<4;++I)EstimatedCoG+=FVector((I&1)?100:-100,(I&2)?520:-520,0)*(O.CornerLoadsN[I]/Sum);
    StableTime+=SampleDt;
    if(StableTime>=C.TrialHold){EstimateValid=true;Enter(ESTSPickupPhase::Complete,TEXT("Trial load verified; full hoist permitted"));}
}
