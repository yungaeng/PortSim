#include "QuayCrane.h"
#include "PortWorkingCrane.h"
#include "PortSiteLogistics.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "PhysicsEngine/PhysicsConstraintComponent.h"

APortContainerActor* AQuayCrane::SpawnContainer(int32 Number,FVector Position)
{
    FActorSpawnParameters Params;
    Params.Owner=this; // Lifetime ownership only; no transform attachment to the STS.
    Params.SpawnCollisionHandlingOverride=ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
    auto* Container=GetWorld()->SpawnActor<APortContainerActor>(Position,FRotator::ZeroRotator,Params);
    check(Container);
    Container->InitializeContainer(Number);
    if(STSProfile.bReady) Container->SetPhysicalParameters(STSProfile.ContainerMassKg,STSProfile.ContainerCoG);
    ContainerActors.Add(Container);
    return Container;
}

bool AQuayCrane::ValidateTerminalActors(FString& Error) const
{
    if (bUnifiedTerminal)
    {
        if (!IsValid(SiteLogistics) || WorkingCranes.Num()!=55 || SupportFleet.Num()!=105 || ContainerActors.Num()!=0 || AGVActors.Num()!=0 || ShipActor)
        { Error=TEXT("Legacy central berth still exists or unified equipment count is wrong"); return false; }
        return SiteLogistics->Validate(Error);
    }
    const int32 Expected=bTerminalMode?24:1;
    if (ContainerActors.Num()!=Expected) { Error=TEXT("Container actor count mismatch"); return false; }
    TSet<FName> IDs;
    TSet<const AActor*> Actors;
    for (int32 I=0;I<ContainerActors.Num();++I)
    {
        const auto* Container=ContainerActors[I].Get();
        if (!IsValid(Container) || Container->GetOwner()!=this ||
            (Container->GetAttachParentActor() && !(Container->LocationOwner==ECargoOwner::RMG && Container->GetAttachParentActor()==RMGActor)) ||
            Container->GetBody()->GetOwner()!=Container || Container->GetRootComponent()!=Container->GetBody() ||
            Container->Visual->GetOwner()!=Container || Container->ContainerID.IsNone() ||
            IDs.Contains(Container->ContainerID) || Actors.Contains(Container) ||
            !Container->GetActorLocation().Equals(Container->GetBody()->GetComponentLocation(),.01f))
        { Error=TEXT("Container ID, actor transform or component ownership mismatch"); return false; }
        if (bTerminalMode && (!CargoBodies.IsValidIndex(I) || CargoBodies[I]!=Container->GetBody()))
        { Error=TEXT("Container manifest references the wrong actor"); return false; }
        IDs.Add(Container->ContainerID); Actors.Add(Container);
    }
    // Verify there are no orphan/duplicate container actors after reset.
    int32 SpawnedContainers=0;
    for (TActorIterator<APortContainerActor> It(GetWorld());It;++It)
        if (It->GetOwner()==this) ++SpawnedContainers;
    if (SpawnedContainers!=Expected) { Error=TEXT("Duplicate spawned container actor"); return false; }
    if (!bTerminalMode) return true;
    for (TActorIterator<APortRMGActor> It(GetWorld());It;++It)
        if (It->GetOwner()==this) { Error=TEXT("Obsolete central RMG still exists"); return false; }
    if (AGVActors.Num()!=3 || WorkingCranes.Num()!=44 || !IsValid(SiteLogistics) || !IsValid(ShipActor) || ShipActor->GetOwner()!=this)
    { Error=TEXT("Fleet/ship actor ownership mismatch"); return false; }
    TArray<const AActor*> Equipment={ShipActor.Get()};
    for (const auto& Crane:WorkingCranes) Equipment.Add(Crane);
    for (int32 I=0;I<AGVActors.Num();++I)
    {
        if (!IsValid(AGVActors[I]) || AGVActors[I]->VehicleID!=I+1)
        { Error=TEXT("AGV actor/ID mismatch"); return false; }
        Equipment.Add(AGVActors[I]);
    }
    for (const auto* Actor:Equipment)
    {
        if (Actor->GetOwner()!=this || Actor->GetAttachParentActor() || Actors.Contains(Actor))
        { Error=TEXT("Equipment is attached to STS or duplicated"); return false; }
        TInlineComponentArray<UStaticMeshComponent*> Parts(Actor);
        for (const auto* Part:Parts)
            if (Part->GetOwner()!=Actor) { Error=TEXT("Equipment component belongs to another actor"); return false; }
        Actors.Add(Actor);
    }
    return true;
}

void AQuayCrane::DestroyTerminalActors()
{
    // Constraints span actors; release them before destroying their physics bodies.
    if (Suspension) Suspension->BreakConstraint();
    if (TwistLock) TwistLock->BreakConstraint();
    auto Destroy=[](AActor* Actor) { if (IsValid(Actor) && !Actor->IsActorBeingDestroyed()) Actor->Destroy(); };
    for (const auto& Container:ContainerActors) Destroy(Container);
    for (const auto& Vehicle:AGVActors) Destroy(Vehicle);
    for (const auto& Vehicle:SupportFleet) Destroy(Vehicle);
    SupportFleet.Reset();
    Destroy(SiteLogistics); SiteLogistics=nullptr;
    for (const auto& Crane:WorkingCranes) Destroy(Crane);
    WorkingCranes.Reset();
    Destroy(ShipActor); Destroy(SiteActor); SiteActor=nullptr;
    ContainerActors.Reset(); CargoBodies.Reset(); AGVActors.Reset();
    RMGActor=nullptr; ShipActor=nullptr; Cargo=nullptr;
}
