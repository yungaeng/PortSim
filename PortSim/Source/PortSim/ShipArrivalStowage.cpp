#include "PortSiteLogistics.h"
#include "PortContainerActor.h"
#include "TerminalLayout.h"

void APortSiteLogistics::BuildSourceAccess()
{
    ++SourceAccessRevision;
    SourceAbove.Reset(); SourceAbove.SetNum(Manifest.Num());
    ManifestBySTS.Reset(); ManifestBySTS.SetNum(FMath::Max(9,LaneCount));
    for(int32 I=0;I<Manifest.Num();++I) ManifestBySTS[Manifest[I].STS].Add(I);
    FTrafficSpatialIndex StackIndex;
    for(int32 I=0;I<Manifest.Num();++I)
    {
        const auto& C=Manifest[I]; const FVector P=C.Transform.GetLocation(), Half=C.Actor->ExternalDimensionsCm*.5;
        StackIndex.Set(I,{FBox(P-Half,P+Half)});
    }
    for(int32 I=0;I<Manifest.Num();++I)
    {
    const auto& C=Manifest[I]; const FVector P=C.Transform.GetLocation(), Half=C.Actor->ExternalDimensionsCm*.5;
    TArray<int32> Candidates=StackIndex.Query({FBox(P-Half,P+Half)}).Array(); Candidates.Sort();
    for(int32 J:Candidates)
    {
        const auto& Lower=Manifest[I]; const auto& Upper=Manifest[J];
        const FVector Delta=Upper.Transform.GetLocation()-Lower.Transform.GetLocation();
        if(Upper.Wave==Lower.Wave && Delta.Z>1. &&
            FMath::Abs(Delta.X)<(Upper.Actor->ExternalDimensionsCm.X+Lower.Actor->ExternalDimensionsCm.X)*.5-.1 &&
            FMath::Abs(Delta.Y)<(Upper.Actor->ExternalDimensionsCm.Y+Lower.Actor->ExternalDimensionsCm.Y)*.5-.1)
            SourceAbove[I].Add(J);
    }
    }
}

bool APortSiteLogistics::PlanArrivalStowage()
{
    if(!FMath::IsNearlyEqual(ShipStowageAssumptions.DeckHeightM,double(TerminalLayout::VesselDeckTopM),.001))
    { Stop(TEXT("Arrival deck height must match the authored ShipDeck top surface")); return false; }
    ShipStowagePlans.Reset();
    ShipStowageAssumptions.Bays=TerminalLayout::VesselBays;
    ShipStowageAssumptions.Rows=TerminalLayout::VesselRows;
    ShipStowageAssumptions.BayPitchM=TerminalLayout::VesselBayPitchM;
    ShipStowageAssumptions.RowPitchM=TerminalLayout::VesselRowPitchM;
    // Authored hull/deck clearances. Grow the real visible stack, never hide
    // another booking batch at the same occupied coordinates as the old waves.
    constexpr int32 MaximumArrivalTiers=TerminalLayout::VesselTiers;
    TMap<int32,int32> ByID;
    for(int32 I=0;I<Manifest.Num();++I) ByID.Add(Manifest[I].ID,I);
    for(int32 Vessel=0;Vessel<ConfiguredShips;++Vessel)
    {
        if(Manifest.ContainsByPredicate([Vessel](const FSiteShipCargo& C){return C.STS/3==Vessel && C.bExplicitPlacement;}) ||
            RetainedCargo.ContainsByPredicate([Vessel](const FSiteShipCargo& C){return C.STS/3==Vessel && C.bExplicitPlacement;}) ||
            ExportCargo.ContainsByPredicate([Vessel](const FSiteShipCargo& C){return C.STS/3==Vessel && C.bExplicitPlacement;}))
        {
            FPortShipStowageResult Plan; Plan.Feasible=true;
            for(const auto& C:Manifest) if(C.STS/3==Vessel) Plan.Placements.Add(C.ShipSlot);
            for(const auto& C:RetainedCargo) if(C.STS/3==Vessel) Plan.Placements.Add(C.ShipSlot);
            ShipStowagePlans.Add(MoveTemp(Plan));
            continue;
        }
        TArray<FPortShipPlanningCargo> Bookings;
        for(const auto& C:Manifest) if(C.STS/3==Vessel)
            Bookings.Add({C.ID,C.Actor->LengthFt,C.BookedGrossKg,C.Actor->CargoInformation,
                FVector(C.Actor->CoGOffsetCm.Y,C.Actor->CoGOffsetCm.X,C.Actor->CoGOffsetCm.Z)*.01});
        FPortShipStowageResult Plan;
        auto Layout=ShipStowageAssumptions;
        for(Layout.Tiers=3;Layout.Tiers<=MaximumArrivalTiers;++Layout.Tiers)
        {
            Plan=PlanShipStowage(Bookings,Layout,true);
            if(!Plan.Feasible) Plan=PlanShipStowage(Bookings,Layout);
            if(Plan.Feasible) break;
        }
        if(!Plan.Feasible || Plan.Placements.Num()!=Bookings.Num())
        {
            Stop(FString::Printf(TEXT("Vessel %d arrival stowage failed (no overlapping cargo waves): %s"),Vessel+1,*Plan.Reason));
            return false;
        }
        // Validate the whole vessel before changing any of its actors.
        for(const auto& P:Plan.Placements)
        {
            const int32* I=ByID.Find(P.CargoID);
            const auto* C=I?Manifest[*I].Actor.Get():nullptr;
            if(!C || FMath::Abs(P.PositionM.X)+C->ExternalDimensionsCm.Y*.005>
                    TerminalLayout::VesselLengthM*.5-12. ||
                FMath::Abs(P.PositionM.Y)+C->ExternalDimensionsCm.X*.005>22. ||
                P.PositionM.Z+C->ExternalDimensionsCm.Z*.005>40.)
            { Stop(TEXT("Arrival plan exceeds authored hull/deck/hoist envelope")); return false; }
        }
        for(const auto& P:Plan.Placements)
        {
            auto& C=Manifest[ByID[P.CargoID]];
            // Planner X is longitudinal; actor/terminal Y is longitudinal.
            const FVector Position((TerminalLayout::VesselCenterXM+P.PositionM.Y)*100.,
                (TerminalLayout::VesselCenterY(Vessel)+P.PositionM.X)*100.,P.PositionM.Z*100.);
            C.Transform.SetLocation(Position); C.Transform.SetRotation(FQuat::Identity);
            IndexedCargoCount=-1;
            C.ShipSlot=P; C.Wave=0;
            C.STS=Vessel*3+TerminalLayout::VesselDischargeZone(P.PositionM.X);
            C.OriginalSTS=C.STS;
            C.Actor->SetActorLocationAndRotation(Position,FQuat::Identity,false,nullptr,ETeleportType::TeleportPhysics);
            C.Actor->SetActorHiddenInGame(false); C.Actor->SetActorEnableCollision(true);
        }
        UE_LOG(LogTemp,Display,TEXT("SHIP_ARRIVAL_STOWAGE: vessel=%d applied=%d bays=%d rows=%d tier_limit=%d gm_m=%.4f heel_deg=%.4f lcg_m=%.4f"),
            Vessel+1,Plan.Placements.Num(),Layout.Bays,Layout.Rows,Layout.Tiers,Plan.CorrectedGMM,Plan.HeelDegrees,Plan.LongitudinalCGM);
        ShipStowagePlans.Add(MoveTemp(Plan));
    }
    return true;
}
