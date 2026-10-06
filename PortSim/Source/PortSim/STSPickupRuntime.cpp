#include "PortWorkingCrane.h"
#include "PortContainerActor.h"
#include "Components/StaticMeshComponent.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "SpreaderTelescope.h"

void APortWorkingCrane::SamplePickupGeometry()
{
    // Virtual geometric pose/contact sensors. No image recognition or manufacturer accuracy claim.
    auto& O=Observation;const auto& C=STSProfile.Pickup;
    const float CargoHalfLength=CargoActor->SensorHalfLengthCm();
    const float CargoHalfWidth=CargoActor->SensorHalfWidthCm();
    const float SpreaderHalfLength=PortSpreaderTelescope::TwistlockHalfLengthCm(TelescopeLengthCm);
    const float SpreaderHalfWidth=PortSpreaderTelescope::TwistlockHalfWidthCm();
    O.bTargetVisible=STSSensorContains(TEXT("spreader_camera"),CargoActor->GetActorLocation());
    if(!bSTS)
    {
        // Four corner-mounted cameras observe corner targets, not the box centre
        // which leaves their downward cones during the final descent.
        O.bTargetVisible=true;
        for(int32 I=0;I<4;++I)
            O.bTargetVisible &= STSSensorContains(TEXT("spreader_camera"),CargoActor->GetActorLocation()+
                CargoActor->GetActorQuat().RotateVector(FVector((I&1)?CargoHalfWidth:-CargoHalfWidth,(I&2)?CargoHalfLength:-CargoHalfLength,0)));
    }
    O.bTargetVisible &= !FParse::Param(FCommandLine::Get(),bSTS?TEXT("PortSimSTSPoseFault"):TEXT("PortSimRMGPoseFault"));
    const FQuat SpreaderRotation=Spreader->GetComponentQuat(), CargoRotation=CargoActor->GetActorQuat();
    O.RelativeYawDegrees=FMath::FindDeltaAngleDegrees(CargoRotation.Rotator().Yaw,SpreaderRotation.Rotator().Yaw);
    O.TargetTiltDegrees=FMath::RadiansToDegrees(FMath::Acos(FMath::Clamp(CargoActor->GetActorUpVector().Z,-1.,1.)));
    bool All=true;
    for(int32 I=0;I<4;++I)
    {
        const FVector SpreaderCorner((I&1)?SpreaderHalfWidth:-SpreaderHalfWidth,(I&2)?SpreaderHalfLength:-SpreaderHalfLength,0);
        const FVector CargoCorner((I&1)?CargoHalfWidth:-CargoHalfWidth,(I&2)?CargoHalfLength:-CargoHalfLength,0);
        const FVector RotationGap=SpreaderRotation.RotateVector(SpreaderCorner)-CargoRotation.RotateVector(CargoCorner)-FVector(0,0,154.5);
        const FVector ActualError=Orientation.UnrotateVector(HeadPosition()-CargoActor->GetActorLocation()+RotationGap);
        PhysicalSeating[I]=ActualError.Size2D()<=C.CornerTolerance && FMath::Abs(ActualError.Z)<=C.VerticalTolerance && O.TargetTiltDegrees<=C.TiltTolerance;
        O.CornerError[I]=Orientation.UnrotateVector(O.SpreaderPosition-O.CargoPosition+RotationGap);
        int32 Missing=INDEX_NONE;FParse::Value(FCommandLine::Get(),bSTS?TEXT("PortSimSTSSeatFault="):TEXT("PortSimRMGSeatFault="),Missing);
        O.CornerSeated[I]=PhysicalSeating[I]&&I!=Missing; All &= O.CornerSeated[I];
    }
    O.bLanded=All&&(O.SpreaderVelocity-O.CargoVelocity).Size()<=C.RelativeSpeed;
}

void APortWorkingCrane::AdvancePickup(float Dt)
{
    Pickup.Update(STSProfile.Pickup,Observation,SimulationTime,Dt,STSProfile.SensorMaxAge,Slots[SourceSlot],STSProfile.RatedPayloadKg);
    if(!Pickup.Fault.IsEmpty()){Stop(Pickup.Fault);return;}
    if(Pickup.Phase==ESTSPickupPhase::Complete)
    {
        UE_LOG(LogTemp,Display,TEXT("%s_PICKUP_VERIFIED: crane=%d measured_kg=%.2f cog_cm=%s attempts=%d"),bSTS?TEXT("STS"):TEXT("RMG"),CraneID,Pickup.EstimatedMass,*Pickup.EstimatedCoG.ToCompactString(),Pickup.Attempts);
        Stage=3;StageTime=SettleTime=0;return;
    }
    if(Pickup.Phase==ESTSPickupPhase::Attach)
    {
        if(!Observation.AllLocked()){Stop(TEXT("Pickup attachment requires four lock feedback signals"));return;}
        if(bDestinationReady&&!DestinationClear()){Stop(TEXT("Destination slot occupied"));return;}
        // Mechanical coupling preserves the achieved pose; never snap a misaligned box onto the head.
        const FVector BeforeAttachment=CargoActor->GetActorLocation();
        CargoActor->DetachFromActor(FDetachmentTransformRules::KeepWorldTransform);
        CargoActor->GetBody()->SetSimulatePhysics(false);
        CargoActor->AttachToComponent(Spreader,FAttachmentTransformRules::KeepWorldTransform);
        if(!BeforeAttachment.Equals(CargoActor->GetActorLocation(),.01))
        {Stop(TEXT("Pickup attachment changed the achieved cargo position"));return;}
        LockedCargoTransform=CargoActor->GetActorTransform().GetRelativeTransform(Spreader->GetComponentTransform());
        CargoActor->LocationOwner=bSTS?ECargoOwner::STS:ECargoOwner::RMG;bCarrying=true;
        Pickup.Attached(Observation.SpreaderPosition);SampleSTS(true);return;
    }
    MoveSTS(Local(Pickup.Target),Dt);
    if(!Fault.IsEmpty())return;
    // Lock actuators respond separately and require the physical contact, not just a pose estimate.
    if(!bCarrying)for(int32 I=0;I<4;++I)
    {
        if(Pickup.RequestLocks[I]&&PhysicalSeating[I])LockProgress[I]+=Dt;
        else LockProgress[I]=0;
        CornerLocked[I]=LockProgress[I]>=STSProfile.Pickup.LockTime&&I!=LockFault;
    }
}
