#if WITH_DEV_AUTOMATION_TESTS
#include "PortSiteLogistics.h"
#include "PortContainerActor.h"
#include "PortShipActor.h"
#include "Components/StaticMeshComponent.h"
#include "ContainerSpecification.h"
#include "TerminalLayout.h"
#include "Engine/World.h"
#include "Misc/AutomationTest.h"
#include "Misc/Paths.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FShipArrivalPlacementTest,"PortSim.Stowage.ArrivalActorsAndAccess",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FShipArrivalPlacementTest::RunTest(const FString& Parameters)
{
    const auto Init=UWorld::InitializationValues().AllowAudioPlayback(false).CreatePhysicsScene(true)
        .RequiresHitProxies(false).CreateNavigation(false).CreateAISystem(false);
    UWorld* World=UWorld::CreateWorld(EWorldType::Game,false,NAME_None,nullptr,true,ERHIFeatureLevel::Num,&Init);
    if(!TestNotNull(TEXT("Arrival placement world"),World)) return false;
    auto* Site=World->SpawnActor<APortSiteLogistics>();
    // Includes a vessel beyond the old 200-box grid: no hidden overlapping wave.
    Site->ConfiguredCargo[0]=201; Site->ConfiguredCargo[1]=200; Site->ConfiguredCargo[2]=199;
    FString Error;
    TestTrue(TEXT("Explicit ship assumptions load"),Site->ShipStowageAssumptions.Load(
        FPaths::ProjectConfigDir()/TEXT("Stowage_Planning.json"),Error));
    TestTrue(TEXT("The authored deck declares its fixture zones"),Site->ShipStowageAssumptions.UseDeckZones);
    for(int32 V=0;V<3;++V) for(int32 N=0;N<Site->ConfiguredCargo[V];++N)
        Site->AddShipCargo(FVector(-4900,TerminalLayout::VesselCenterY(V)*100.,569.5),V*3,N/200);
    if(!TestTrue(TEXT("All bookings have a physical arrival plan"),Site->PlanArrivalStowage()))
    { AddError(Site->Fault); World->DestroyWorld(false); return false; }
    Site->BuildSourceAccess();
    TMap<int32,int32> ByID; FIntVector Counts(0,0,0); TSet<int32> STSs; int32 ShortPairs=0,Supported=0;
    for(int32 I=0;I<Site->Manifest.Num();++I) ByID.Add(Site->Manifest[I].ID,I);
    for(int32 I=0;I<Site->Manifest.Num();++I)
    {
        auto& C=Site->Manifest[I]; const auto* Actor=C.Actor.Get();
        STSs.Add(C.STS); (Actor->LengthFt==20?Counts.X:Actor->LengthFt==40?Counts.Y:Counts.Z)++;
        TestEqual(TEXT("No synthetic overlapping arrival batches"),C.Wave,0);
        TestFalse(TEXT("Every arrival booking is visible"),Actor->IsHidden());
        TestTrue(TEXT("The calculated plan is applied to the actor"),Actor->GetActorLocation().Equals(C.Transform.GetLocation(),.01));
        TestTrue(TEXT("The plan has a real ship slot"),C.ShipSlot.CargoID==C.ID && C.ShipSlot.Tier>=1);
        TestTrue(TEXT("Only short containers occupy a 20ft subslot"),Actor->LengthFt==20?
            (C.ShipSlot.Subslot==1 || C.ShipSlot.Subslot==2):C.ShipSlot.Subslot==0);
        const bool EndBay=Site->ShipStowageAssumptions.IsEndBay(C.ShipSlot.Bay-1);
        if(Actor->LengthFt==45) TestTrue(TEXT("45ft needs an enabled long cell"),EndBay);
        if(Actor->CargoInformation.HandlingType==TEXT("reefer")) TestTrue(TEXT("Reefers have a powered slot"),
            Site->ShipStowageAssumptions.IsPoweredRow(C.ShipSlot.Row-1));
        if(Actor->CargoInformation.HazardGroup==TEXT("flammable")) TestTrue(TEXT("Flammable group in declared end bays"),Site->ShipStowageAssumptions.IsFlammableBay(C.ShipSlot.Bay-1));
        if(Actor->CargoInformation.HazardGroup==TEXT("corrosive")) TestTrue(TEXT("Corrosive group in opposite end bays"),Site->ShipStowageAssumptions.IsCorrosiveBay(C.ShipSlot.Bay-1));
        if(C.ShipSlot.BelowCargoID!=INDEX_NONE)
        {
            ++Supported; const int32* LowerIndex=ByID.Find(C.ShipSlot.BelowCargoID);
            if(!TestNotNull(TEXT("Ship supporting booking exists"),LowerIndex)) continue;
            const auto& Lower=Site->Manifest[*LowerIndex];
            TestEqual(TEXT("No mixed-size unsupported stack"),Lower.Actor->LengthFt,Actor->LengthFt);
            TestTrue(TEXT("Later destinations below earlier calls"),Lower.Actor->CargoInformation.PortCallOrder>=Actor->CargoInformation.PortCallOrder);
            TestTrue(TEXT("Heavy below light within compatible stack"),Lower.BookedGrossKg>=C.BookedGrossKg);
            TestTrue(TEXT("Real footprint blocks the lower pickup"),Site->SourceAbove[*LowerIndex].Contains(I));
            TestFalse(TEXT("A covered container cannot be picked"),Site->SourceAccessible(*LowerIndex));
            TestTrue(TEXT("Physical corner support has no gap or penetration"),FMath::IsNearlyEqual(
                C.Transform.GetLocation().Z-Lower.Transform.GetLocation().Z,Actor->ExternalDimensionsCm.Z,.01));
        }
        for(int32 J=0;J<I;++J)
        {
            const auto& Other=Site->Manifest[J];
            const FVector D=(C.Transform.GetLocation()-Other.Transform.GetLocation()).GetAbs();
            const FVector Half=(Actor->ExternalDimensionsCm+Other.Actor->ExternalDimensionsCm)*.5;
            TestFalse(TEXT("No two arrival containers interpenetrate"),D.X<Half.X-.1 && D.Y<Half.Y-.1 && D.Z<Half.Z-.1);
            if(Actor->LengthFt==20 && Other.Actor->LengthFt==20 && C.STS/3==Other.STS/3 &&
                C.ShipSlot.Bay==Other.ShipSlot.Bay && C.ShipSlot.Row==Other.ShipSlot.Row &&
                C.ShipSlot.Tier==Other.ShipSlot.Tier && C.ShipSlot.Subslot!=Other.ShipSlot.Subslot) ++ShortPairs;
        }
    }
    TestTrue(TEXT("All original size quotas are preserved"),Counts==PortContainerSpecification::CountsForManifest(600));
    TestEqual(TEXT("All nine STSs can service the real arrival plan"),STSs.Num(),9);
    TestTrue(TEXT("20ft end-to-end cells and supported tiers are exercised"),ShortPairs>0 && Supported>0);
    for(int32 V=0;V<3;++V)
    {
        const auto& A=Site->ShipStowageAssumptions;
        auto* Hull=World->SpawnActor<APortShipActor>(FVector(TerminalLayout::VesselOriginXM*100.,
            TerminalLayout::VesselCenterY(V)*100.,TerminalLayout::VesselOriginZM*100.),FRotator::ZeroRotator);
        TArray<UStaticMeshComponent*> Parts; Hull->GetComponents(Parts);
        const auto* HullPart=Parts.FindByPredicate([](const auto* Part){return Part->GetFName()==TEXT("ShipHull");});
        if(!TestNotNull(TEXT("Physical hull centre is available"),HullPart)) continue;
        const double HullCentreX=(*HullPart)->GetComponentLocation().X*.01;
        const auto* Deck=Parts.FindByPredicate([](const auto* Part){return Part->GetFName()==TEXT("ShipDeck");});
        if(!TestNotNull(TEXT("Physical deck surface is available"),Deck)) continue;
        const double DeckTop=(*Deck)->CalcBounds((*Deck)->GetComponentTransform()).GetBox().Max.Z*.01;
        TestTrue(TEXT("Planning uses the physical deck TOP, not its centre"),FMath::IsNearlyEqual(A.DeckHeightM,DeckTop,1.e-6));
        double Mass=A.LightshipKg,Vertical=A.LightshipKg*A.LightshipKGM,Longitudinal=0,Transverse=0;
        for(const auto& C:Site->Manifest) if(C.STS/3==V)
        {
            const FVector P=C.Actor->GetActorLocation()*.01;
            if(C.ShipSlot.Tier==1) TestTrue(TEXT("Ground-tier cargo rests on the actual deck without penetration or floating"),
                FMath::IsNearlyEqual(P.Z-C.Actor->ExternalDimensionsCm.Z*.005,DeckTop,1.e-6));
            Mass+=C.BookedGrossKg; Vertical+=C.BookedGrossKg*(P.Z+C.Actor->CoGOffsetCm.Z*.01);
            Longitudinal+=C.BookedGrossKg*(P.Y-TerminalLayout::VesselCenterY(V)+C.Actor->CoGOffsetCm.Y*.01);
            Transverse+=C.BookedGrossKg*(P.X-HullCentreX+C.Actor->CoGOffsetCm.X*.01);
        }
        const auto& Plan=Site->ShipStowagePlans[V];
        TestTrue(TEXT("Reported GM matches the actually placed actor heights"),FMath::IsNearlyEqual(
            Plan.CorrectedGMM,A.KMM-Vertical/Mass-A.FreeSurfaceMomentKgM/Mass,1.e-8));
        TestTrue(TEXT("Longitudinal balance uses actual ship coordinates"),FMath::IsNearlyEqual(Plan.LongitudinalCGM,Longitudinal/Mass,1.e-8));
        const double Heel=FMath::RadiansToDegrees(FMath::Atan2(FMath::Abs(Transverse)/Mass,Plan.CorrectedGMM));
        TestTrue(TEXT("Reported heel uses the actual ShipHull centreline"),FMath::IsNearlyEqual(Plan.HeelDegrees,Heel,1.e-8));
    }
    // Clearance changes availability; it does not bypass the remaining upper boxes.
    for(auto& C:Site->Manifest) C.Actor->SetLocationOwner(ECargoOwner::STS);
    Site->Manifest[0].Actor->SetLocationOwner(ECargoOwner::Ship);
    TestTrue(TEXT("Pickup becomes available after the physical upper boxes leave"),Site->SourceAccessible(0));
    World->DestroyWorld(false);
    return true;
}
#endif
