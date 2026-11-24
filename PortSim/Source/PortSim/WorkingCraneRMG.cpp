#include "TerminalReplay.h"
#include "PortWorkingCrane.h"
#include "PortContainerActor.h"
#include "PortAGVActor.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/World.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"

bool APortWorkingCrane::SecureRMGToAGV()
{
    if(bSTS || !bExternalJobs || !AGVAtDestination() || Stage!=5 || !bCarrying ||
        !IsValid(CargoActor) || !IsValid(HandoverAGV) ||
        !Observation.IsFresh(SimulationTime,STSProfile.SensorMaxAge) ||
        !Observation.bAGVAligned || !AGVAligned() || !Observation.bCargoSupported || !CargoSupported() ||
        !Observation.AllLocked() || !Observation.bLanded ||
        Observation.SpreaderVelocity.Size()>STSProfile.SettleSpeed ||
        !CargoActor->GetActorLocation().Equals(HandoverAGV->CargoPosition(),FMath::Min(10.f,STSProfile.LandingTolerance)) ||
        CargoActor->GetActorUpVector().Z<.99f ||
        CargoActor->GetAttachParentActor()!=this || CargoActor->GetBody()->IsSimulatingPhysics() ||
        !DestinationClear()) return false;
    // In this model, a verified rigid attachment represents the AGV cargo
    // restraint. Preserve the measured seated pose; never snap an unsafe load.
    if(!CargoActor->AttachToComponent(HandoverAGV->GetRootComponent(),FAttachmentTransformRules::KeepWorldTransform) ||
        CargoActor->GetAttachParentActor()!=HandoverAGV)
    {
        Stop(TEXT("RMG receiving AGV cargo restraint failed"));
        return false;
    }
    CargoActor->SetLocationOwner(ECargoOwner::AGV);
    HandoverAGV->SetPayload(CargoActor->MassKg);
    bAGVHandoverComplete=true;
    return true;
}

void APortWorkingCrane::SampleRMGEnvironment()
{
    auto& O=Observation;
    O.bStackProfileValid=false;O.StackTopZ=0;O.bCraneClear=true;O.CraneDistance=-1;
    FCollisionQueryParams Query(SCENE_QUERY_STAT(RMGSensors),false);
    Query.AddIgnoredActor(this);Query.AddIgnoredActor(CargoActor);
    // Four downward range samples at the target footprint, within the trolley
    // sensor's configured field of view. Expected support height is not a measurement.
    if(Stage>=4)
    {
        FCollisionQueryParams ProfileQuery(SCENE_QUERY_STAT(RMGStackProfile),false);
        ProfileQuery.AddIgnoredActor(CargoActor); // Keep the fixed reservation pad.
        bool Valid=true;
        const float HalfLength=IsValid(CargoActor)?CargoActor->SensorHalfLengthCm():520.f;
        const float HalfWidth=IsValid(CargoActor)?CargoActor->SensorHalfWidthCm():113.f;
        const FVector E=Orientation.RotateVector(FVector(HalfWidth,0,0)).GetAbs()+
            Orientation.RotateVector(FVector(0,HalfLength,0)).GetAbs();
        SynchronizeShipQuery(FBox(FVector(Slots[1-SourceSlot].X-E.X,Slots[1-SourceSlot].Y-E.Y,Home.Z-100),
            FVector(Slots[1-SourceSlot].X+E.X,Slots[1-SourceSlot].Y+E.Y,Home.Z+BeamZ-100)));
        for(int32 I=0;I<4;++I)
        {
            const FVector Point=Slots[1-SourceSlot]+Orientation.RotateVector(FVector((I&1)?HalfWidth:-HalfWidth,(I&2)?HalfLength:-HalfLength,0));
            const FVector Origin(Point.X,Point.Y,Home.Z+BeamZ-100);
            FHitResult Hit;
            const bool Found=GetWorld()->LineTraceSingleByChannel(Hit,Origin,FVector(Point.X,Point.Y,Home.Z-100),ECC_Visibility,ProfileQuery);
            const bool InView=Found && STSSensorContains(TEXT("stack_profile"),Hit.ImpactPoint);
            const bool CornerValid=Found && Hit.ImpactNormal.Z>.8 && InView &&
                FMath::Abs(Hit.ImpactPoint.Z-(Slots[1-SourceSlot].Z-129.5))<=STSProfile.StackHeightTolerance;
            if(Stage==5 && !CornerValid)
                PORTSIM_SIM_LOG(LogTemp,Warning,TEXT("RMG_STACK_SAMPLE: crane=%d corner=%d found=%d actor=%s component=%s point=%s normal=%s expected_z_cm=%.3f in_view=%d"),
                    CraneID,I,Found,*GetNameSafe(Hit.GetActor()),*GetNameSafe(Hit.GetComponent()),*Hit.ImpactPoint.ToString(),
                    *Hit.ImpactNormal.ToString(),Slots[1-SourceSlot].Z-129.5,InView);
            Valid &= CornerValid;
            if(Found)O.StackTopZ=FMath::Max(O.StackTopZ,float(Hit.ImpactPoint.Z));
        }
        O.bStackProfileValid=Valid && !FParse::Param(FCommandLine::Get(),TEXT("PortSimRMGStackFault"));
    }
    // Bogie lasers look along both rail directions. Only another working crane
    // is the crane-to-crane target; these are not pedestrian safety scanners.
    for(int32 I=0;I<STSProfile.Dynamics.Mounts.Num();++I)
    {
        const auto& M=STSProfile.Dynamics.Mounts[I];
        if(!M.Key.StartsWith(TEXT("crane_collision_")) || !SensorMarkers.IsValidIndex(I))continue;
        const auto* Marker=SensorMarkers[I].Get();
        const FVector Direction=Marker->GetForwardVector();
        const double Toward=FMath::Max(0.,FVector::DotProduct(Orientation.RotateVector(AxisVelocity),Direction));
        const double StopDistance=STSProfile.CollisionMargin+Toward*STSProfile.SensorMaxAge+Toward*Toward/(2*FMath::Max(.001,RailAccelerationLimit(STSProfile.GantryAcceleration)));
        const double Maximum=FMath::Min(M.Maximum,double(FogVisibilityMeters));
        if(Maximum<M.Maximum && (Maximum<=M.Minimum || Maximum*100<StopDistance))
        {
            // A clear return cannot establish clearance past the visible rail.
            O.bCraneClear=false;
            continue;
        }
        TArray<FVector> Origins;Origins.Add(M.Position);Origins.Append(M.AdditionalPositions);
        for(const FVector& LocalOrigin:Origins)
        {
            const FVector Origin=Marker->GetAttachParent()->GetComponentLocation()+Marker->GetAttachParent()->GetComponentQuat().RotateVector(LocalOrigin*100);
            const FVector Begin=Origin+Direction*M.Minimum*100;
            const FVector End=Origin+Direction*Maximum*100;
            FBox QueryBounds(ForceInit); QueryBounds+=Begin; QueryBounds+=End;
            SynchronizeShipQuery(QueryBounds);
            FHitResult Hit;
            if(GetWorld()->LineTraceSingleByChannel(Hit,Begin,End,ECC_Visibility,Query) && Cast<APortWorkingCrane>(Hit.GetActor()))
            {
                const float Distance=FVector::Dist(Origin,Hit.ImpactPoint);
                O.CraneDistance=O.CraneDistance<0?Distance:FMath::Min(O.CraneDistance,Distance);
                if(Distance<StopDistance)O.bCraneClear=false;
            }
        }
    }
}
