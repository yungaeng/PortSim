#include "PortCargoBody.h"
#include "PortContainerActor.h"
#include "PortSiteLogistics.h"

void UPortCargoBody::OnUpdateTransform(EUpdateTransformFlags Flags,ETeleportType Teleport)
{
    Super::OnUpdateTransform(Flags,Teleport);
    if(auto* Cargo=Cast<APortContainerActor>(GetOwner()))
        if(auto* Site=Cast<APortSiteLogistics>(Cargo->GetOwner())) Site->DirtyKinematicCargo.Add(Cargo);
}

void UPortCargoBody::SetSimulatePhysics(bool Enabled)
{
    // A new manipulation/physics session must be movable before attachment.
    // Verified fixed placements use genuine static Chaos bodies in between.
    if(Mobility==EComponentMobility::Static) SetMobility(EComponentMobility::Movable);
    Super::SetSimulatePhysics(Enabled);
    if(auto* Cargo=Cast<APortContainerActor>(GetOwner()))
        if(auto* Site=Cast<APortSiteLogistics>(Cargo->GetOwner())) Site->RefreshCargoActivity(Cargo);
}

void UPortCargoBody::OnAttachmentChanged()
{
    Super::OnAttachmentChanged();
    if(auto* Cargo=Cast<APortContainerActor>(GetOwner()))
        if(auto* Site=Cast<APortSiteLogistics>(Cargo->GetOwner())) Site->RefreshCargoActivity(Cargo);
}
