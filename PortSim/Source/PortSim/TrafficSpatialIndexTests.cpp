#if WITH_DEV_AUTOMATION_TESTS
#include "TrafficSpatialIndex.h"
#include "Misc/AutomationTest.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTrafficSpatialIndexTest,"PortSim.Traffic.SpatialIndex",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FTrafficSpatialIndexTest::RunTest(const FString& Parameters)
{
    FTrafficSpatialIndex Index;
    const FBox Negative(FVector(-4000,-4000,-250),FVector(-2000,-2000,250));
    const FBox Touching(FVector(-2000,-3000,-250),FVector(0,-1000,250));
    Index.Set(7,{Negative,Negative});
    TestTrue(TEXT("Closed negative cell boundaries retain touching envelopes"),Index.Query({Touching}).Contains(7));
    Index.Set(7,{FBox(FVector(10000),FVector(11000))});
    TestFalse(TEXT("Same-frame movement removes old occupancy"),Index.Query({Negative}).Contains(7));
    TestTrue(TEXT("Same-frame movement publishes new occupancy"),Index.Query({FBox(FVector(10000),FVector(11000))}).Contains(7));
    Index.Remove(7);
    TestTrue(TEXT("Removal clears every occupied cell"),Index.Query({FBox(FVector(-20000),FVector(20000))}).IsEmpty());

    // Compare indexed narrow-phase results against the original exhaustive scan.
    FRandomStream Random(7319);
    TArray<TArray<FBox>> Reservations;
    for (int32 I=0;I<120;++I)
    {
        TArray<FBox> Segments;
        for (int32 J=0;J<2;++J)
        {
            const FVector P(Random.FRandRange(-55000,55000),Random.FRandRange(-55000,55000),0);
            const FVector E(Random.FRandRange(210,4000),Random.FRandRange(730,7000),250);
            Segments.Add(FBox(P-E,P+E));
        }
        Reservations.Add(Segments); Index.Set(I,Segments);
    }
    int64 Candidates=0;
    for (int32 Q=0;Q<400;++Q)
    {
        const FVector P(Random.FRandRange(-55000,55000),Random.FRandRange(-55000,55000),0);
        const FVector E(730,3000,250);
        const FBox Query(P-E,P+E);
        const auto Found=Index.Query({Query}); Candidates+=Found.Num();
        for (int32 I=0;I<Reservations.Num();++I)
        {
            bool Brute=false;
            for (const auto& B:Reservations[I]) Brute|=Query.Intersect(B);
            if (Brute && !Found.Contains(I)) { AddError(TEXT("Spatial index missed a conflicting reservation")); return false; }
        }
    }
    TestTrue(TEXT("Local queries reduce reservation candidates"),Candidates<400*120/4);
    Index.Reset();
    TestTrue(TEXT("Reset removes all previous reservations"),Index.Query({FBox(FVector(-100000),FVector(100000))}).IsEmpty());
    AddInfo(FString::Printf(TEXT("Candidates: %lld / %d exhaustive reservation checks"),Candidates,400*120));
    return true;
}
#endif
