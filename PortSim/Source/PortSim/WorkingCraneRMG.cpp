#include "PortWorkingCrane.h"
#include "PortContainerActor.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/World.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"

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
        for(int32 I=0;I<4;++I)
        {
            const FVector Point=Slots[1-SourceSlot]+Orientation.RotateVector(FVector((I&1)?HalfWidth:-HalfWidth,(I&2)?HalfLength:-HalfLength,0));
            const FVector Origin(Point.X,Point.Y,Home.Z+BeamZ-100);
            FHitResult Hit;
            const bool Found=GetWorld()->LineTraceSingleByChannel(Hit,Origin,FVector(Point.X,Point.Y,Home.Z-100),ECC_Visibility,ProfileQuery);
            Valid &= Found && Hit.ImpactNormal.Z>.8 && STSSensorContains(TEXT("stack_profile"),Hit.ImpactPoint) &&
                FMath::Abs(Hit.ImpactPoint.Z-(Slots[1-SourceSlot].Z-129.5))<=STSProfile.StackHeightTolerance;
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
        TArray<FVector> Origins;Origins.Add(M.Position);Origins.Append(M.AdditionalPositions);
        for(const FVector& LocalOrigin:Origins)
        {
            const FVector Origin=Marker->GetAttachParent()->GetComponentLocation()+Marker->GetAttachParent()->GetComponentQuat().RotateVector(LocalOrigin*100);
            FHitResult Hit;
            if(GetWorld()->LineTraceSingleByChannel(Hit,Origin+Marker->GetForwardVector()*M.Minimum*100,Origin+Marker->GetForwardVector()*M.Maximum*100,ECC_Visibility,Query) && Cast<APortWorkingCrane>(Hit.GetActor()))
            {
                const float Distance=FVector::Dist(Origin,Hit.ImpactPoint);
                O.CraneDistance=O.CraneDistance<0?Distance:FMath::Min(O.CraneDistance,Distance);
                const double Toward=FMath::Max(0.,FVector::DotProduct(Orientation.RotateVector(AxisVelocity),Marker->GetForwardVector()));
                const double StopDistance=STSProfile.CollisionMargin+Toward*STSProfile.SensorMaxAge+Toward*Toward/(2*STSProfile.GantryAcceleration);
                if(Distance<StopDistance)O.bCraneClear=false;
            }
        }
    }
}
