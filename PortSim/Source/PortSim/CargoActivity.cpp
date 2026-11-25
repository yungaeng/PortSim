#include "SimulationPerformance.h"
#include "PortSiteLogistics.h"
#include "PortContainerActor.h"
#include "PortWorkingCrane.h"
#include "TerminalReplay.h"
#include "Components/StaticMeshComponent.h"

void APortSiteLogistics::RemoveCargoActivity(APortContainerActor* Cargo)
{
    InvalidateShipRemaining();
    DirtyKinematicCargo.Remove(Cargo);
    FreeCargo.Remove(Cargo); MovingCargo.Remove(Cargo); FixedCargo.Remove(Cargo);
    CargoMotionVersions.Remove(Cargo);
    IndexedCargoCount=-1;
    bStaticShipCollisionReady=false;
}

void APortSiteLogistics::RefreshCargoActivity(APortContainerActor* Cargo)
{
    if(!IsValid(Cargo) || !Cargo->GetBody()) return;
    DirtyKinematicCargo.Add(Cargo);
    CargoMotionVersions.Remove(Cargo);
    FreeCargo.Remove(Cargo); MovingCargo.Remove(Cargo); FixedCargo.Remove(Cargo);
    if(Cargo->GetBody()->IsSimulatingPhysics()) FreeCargo.Add(Cargo);
    else if(Cargo->GetAttachParentActor()) MovingCargo.Add(Cargo);
    else FixedCargo.Add(Cargo);
}

void APortSiteLogistics::RebuildCargoActivity()
{
    InvalidateShipRemaining();
    DirtyKinematicCargo.Reset();
    FreeCargo.Reset(); MovingCargo.Reset(); FixedCargo.Reset();
    for(const auto& Cargo:ShipContainers) RefreshCargoActivity(Cargo);
    CargoMotionVersions.Reset(); IndexedCargoCount=-1;
    bStaticShipCollisionReady=false;
}

void APortSiteLogistics::UpdateCargoMotion(APortContainerActor* Actor)
{
    if(!IsValid(Actor) || Actor->LocationOwner!=ECargoOwner::Ship || Actor->GetAttachParentActor() ||
        Actor->GetBody()->IsSimulatingPhysics()) return;
    const auto* Source=CargoSources.Find(Actor);
    if(!Source || CargoMotionVersions.FindRef(Actor)==CargoMotionVersion) return;
    const int32 Vessel=Source->STS/3;
    if(!CurrentCargoMotions.IsValidIndex(Vessel) || !VesselBasePivots.IsValidIndex(Vessel)) return;
    const FVector Base=Source->Transform.GetLocation(), Pivot=VesselBasePivots[Vessel];
    auto Set=[&](const FPortVesselMotion& Motion,float Duration)
    {
        Actor->SetSecuredVesselMotion(Motion.TransformPosition(Base,Pivot),
            Motion.TransformRotation(Source->Transform.GetRotation()),Motion.VelocityAtPosition(Base,Pivot),
            Motion.AccelerationAtPosition(Base,Pivot),Motion.JerkAtPosition(Base,Pivot),Duration,false);
    };
    // Both endpoints come from common ship state, never an out-of-date actor pose.
    Set(PreviousCargoMotions.IsValidIndex(Vessel)?PreviousCargoMotions[Vessel]:CurrentCargoMotions[Vessel],0);
    Set(CurrentCargoMotions[Vessel],CargoMotionDuration);
    Actor->SetSecuredMotionWindow(ShipWindowStart,ShipWindowEnd);
    CargoMotionVersions.Add(Actor,CargoMotionVersion);
}

void APortSiteLogistics::PublishCargoPresentation()
{
    PortSimPerformance::FScope Timing(TEXT("cargo_presentation"));
    // Attached cargo follows its carrier and free cargo is published by Chaos.
    // Only a crane's selected source needs vessel interpolation for presentation.
    // Do not touch thousands of unstarted boxes or completed yard placements.
    for(const auto& Crane:Equipment) if(IsValid(Crane))
    {
        auto* Box=Crane->CargoActor.Get();
        if(IsValid(Box) && Box->LocationOwner==ECargoOwner::Ship && !Box->GetAttachParentActor())
        { UpdateCargoMotion(Box); Box->ApplySecuredMotionFraction(1.); }
    }
    // Unselected boxes retain their last physical pose. Keep previous bay bounds
    // in the query index as well as the current vessel sweep: resetting to the
    // latest frame alone would miss stale colliders during sensor queries.
    for(const auto& Crane:Equipment) if(IsValid(Crane)) Crane->DeferRopeVisuals(false);
}

void APortSiteLogistics::SynchronizeFreeCargoCollision(float Dt)
{
    // Chaos owns sleeping/active body selection. Include sleeping free bodies:
    // a moving ship can contact and wake them even when their velocity is zero.
    for(const auto& Cargo:FreeCargo) if(auto* Box=Cargo.Get())
    {
        const FBox Bounds=Box->GetBody()->Bounds.GetBox();
        const FVector Travel=Box->GetBody()->GetPhysicsLinearVelocity()*Dt;
        FBox Swept=Bounds; Swept+=FBox(Bounds.Min+Travel,Bounds.Max+Travel);
        SynchronizeShipCollision(Swept.ExpandBy(Box->ExternalDimensionsCm.Size()+100.),1.);
    }
}
