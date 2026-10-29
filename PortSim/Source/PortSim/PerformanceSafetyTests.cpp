#if WITH_DEV_AUTOMATION_TESTS
#include "PortContainerActor.h"
#include "PortAGVActor.h"
#include "PortSiteLogistics.h"
#include "PortWorkingCrane.h"
#include "STSDynamics.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/World.h"
#include "Misc/AutomationTest.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FNoEventCollisionTest,"PortSim.Performance.SensorQueriesWithoutOverlapEvents",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FNoEventCollisionTest::RunTest(const FString& Parameters)
{
    TestEqual(TEXT("50 ms uses exactly three 60 Hz steps"),CraneSimulationSteps::Count(.05f,60.),3);
    TestEqual(TEXT("60 Hz control uses two 120 Hz plant steps"),CraneSimulationSteps::Count(1.f/60.f,120.),2);
    TestEqual(TEXT("Real excess time still requires another step"),CraneSimulationSteps::Count(.0501f,60.),4);
    const auto Init=UWorld::InitializationValues().AllowAudioPlayback(false).CreatePhysicsScene(true)
        .RequiresHitProxies(false).CreateNavigation(false).CreateAISystem(false);
    UWorld* World=UWorld::CreateWorld(EWorldType::Game,false,NAME_None,nullptr,true,ERHIFeatureLevel::Num,&Init);
    if(!TestNotNull(TEXT("Collision verification world"),World)) return false;
    auto* Cargo=World->SpawnActor<APortContainerActor>();
    Cargo->SetSecuredVesselMotion(FVector(0,0,200),FQuat::Identity,FVector::ZeroVector);
    TestFalse(TEXT("Unused overlap callbacks are disabled"),Cargo->Body->GetGenerateOverlapEvents());
    TestEqual(TEXT("Physical and query collision remain enabled"),Cargo->Body->GetCollisionEnabled(),ECollisionEnabled::QueryAndPhysics);
    auto Trace=[&](double X)
    {
        FHitResult Hit;
        return World->LineTraceSingleByChannel(Hit,FVector(X-1000,0,200),FVector(X+1000,0,200),ECC_Visibility)
            && Hit.GetActor()==Cargo;
    };
    TestTrue(TEXT("LiDAR-style trace still hits the secured container"),Trace(0));
    // Cross the actual surface. A probe wholly inside a triangle-only cube
    // need not intersect its triangles, regardless of overlap-event settings.
    const FVector Surface=Cargo->Body->Bounds.Origin+FVector(Cargo->Body->Bounds.BoxExtent.X,0,0);
    const auto Probe=[&]() { return World->OverlapBlockingTestByChannel(Surface,FQuat::Identity,
        ECC_WorldDynamic,FCollisionShape::MakeBox(FVector(20))); };
    const bool WithoutEvents=Probe();
    Cargo->Body->SetGenerateOverlapEvents(true);
    const bool WithEvents=Probe();
    Cargo->Body->SetGenerateOverlapEvents(false);
    TestTrue(TEXT("Explicit destination surface overlap blocks in both event modes"),WithoutEvents && WithEvents);
    const FVector VisualBefore=Cargo->Visual->GetComponentLocation();
    Cargo->BeginVisualFrame();
    Cargo->SetSecuredVesselMotion(FVector(4000,0,200),FQuat::Identity,FVector(10,0,0));
    TestFalse(TEXT("Old sensor location is clear immediately after motion"),Trace(0));
    TestTrue(TEXT("New sensor location blocks in the same frame"),Trace(4000));
    TestTrue(TEXT("Artwork can wait while the physical obstacle moves"),Cargo->Visual->GetComponentLocation().Equals(VisualBefore));
    Cargo->EndVisualFrame();
    TestTrue(TEXT("Artwork catches up before presenting the frame"),Cargo->Visual->GetComponentLocation().Equals(VisualBefore+FVector(4000,0,0)));
    TestTrue(TEXT("Artwork retains its parent after the batch"),Cargo->Visual->GetAttachParent()==Cargo->Body);
    TestTrue(TEXT("Moving an obstacle retains its measured velocity"),Cargo->GetMotionVelocity().Equals(FVector(10,0,0)));
    auto* AGV=World->SpawnActor<APortAGVActor>(FVector(10000,0,0),FRotator::ZeroRotator);
    TArray<UStaticMeshComponent*> Parts; AGV->GetComponents(Parts);
    int32 Blocking=0;
    for(const auto* Part:Parts)
    {
        TestFalse(TEXT("AGV visual/deck parts need no overlap callbacks"),Part->GetGenerateOverlapEvents());
        Blocking+=Part->GetCollisionEnabled()==ECollisionEnabled::QueryAndPhysics;
    }
    TestTrue(TEXT("AGV deck retains blocking physics collision"),Blocking>0);
    auto* RMG=World->SpawnActor<APortWorkingCrane>(FVector(18301.38,-13500,0),FRotator(0,90,0));
    const FVector Source(16336.77,-12180,349.5);
    RMG->Configure(19,false,Source,FVector(18301.38,-14100,667.5),false);
    Cargo->Body->SetSimulatePhysics(false);
    TestTrue(TEXT("RMG accepts the previous upper tier reservation"),
        RMG->AssignCargo(Cargo,Source,FVector(18301.38,-14100,667.5),false,true));
    RMG->ResetOperation();
    TestTrue(TEXT("RMG accepts a new ground tier reservation"),
        RMG->AssignCargo(Cargo,Source,FVector(18301.38,-14400,149.5),false,true));
    TArray<UStaticMeshComponent*> CraneParts; RMG->GetComponents(CraneParts);
    for(auto* Part:CraneParts) if(Part->GetFName()==TEXT("Slot_1"))
    {
        TestTrue(TEXT("Reserved support physics pose immediately matches the new fixed tier"),
            Part->BodyInstance.GetUnrealWorldTransform().GetLocation().Equals(FVector(18301.38,-14400,10),.01));
        FHitResult Hit;
        TestTrue(TEXT("Downward range measurement sees the actual ground support at 20 cm"),
            World->LineTraceSingleByChannel(Hit,FVector(18301.38,-14400,2500),FVector(18301.38,-14400,-100),ECC_Visibility)
            && Hit.GetComponent()==Part && FMath::Abs(Hit.ImpactPoint.Z-20.)<.01);
    }
    RMG->Destroy();
    // The lazy query broad phase must produce the same hits as refreshing
    // every secured actor, even with Hermite overshoot, rotation, old poses
    // still in the query and partial traffic-step windows.
    auto* Site=World->SpawnActor<APortSiteLogistics>();
    Site->bDeferringVisuals=true;
    FActorSpawnParameters Spawn; Spawn.Owner=Site;
    for(int32 I=0;I<12;++I)
    {
        auto* Box=World->SpawnActor<APortContainerActor>(FVector::ZeroVector,FRotator::ZeroRotator,Spawn);
        const FVector Base(-20000,I*2500-15000,1000);
        Box->SetSecuredVesselMotion(Base,FQuat::Identity,FVector(-1800,400,0));
        Box->SetSecuredVesselMotion(Base+FVector(700,200,100),FRotator(5,40,3).Quaternion(),
            FVector(2000,-300,0),FVector::ZeroVector,FVector::ZeroVector,2.,false);
        Box->SetSecuredMotionWindow(.2,.8);
        Site->ShipContainers.Add(Box);
    }
    for(int32 Sample=0;Sample<60;++Sample)
    {
        const double Fraction=double(Sample%11)/10.;
        for(const auto& Box:Site->ShipContainers) Box->ApplySecuredMotionFraction(Fraction);
        const FVector Center=Site->ShipContainers[Sample%12]->GetActorLocation();
        const FVector Begin=Center-FVector(1800,0,0),End=Center+FVector(1800,0,0);
        FHitResult ExpectedHit,ActualHit;
        const bool Expected=World->LineTraceSingleByChannel(ExpectedHit,Begin,End,ECC_Visibility);
        for(const auto& Box:Site->ShipContainers) Box->ApplySecuredMotionFraction(Sample%2?0.:1.);
        FBox Bounds(ForceInit); Bounds+=Begin; Bounds+=End;
        Site->SynchronizeShipCollision(Bounds,Fraction);
        const bool Actual=World->LineTraceSingleByChannel(ActualHit,Begin,End,ECC_Visibility);
        TestTrue(TEXT("Broad phase retains the same moving obstacle and contact point"),
            Expected && Actual && ExpectedHit.GetActor()==ActualHit.GetActor() &&
            ExpectedHit.ImpactPoint.Equals(ActualHit.ImpactPoint,.001));
    }
    Site->bDeferringVisuals=false;
    Site->LaneCount=9;
    World->DeltaTimeSeconds=1.f/60.f;
    TestFalse(TEXT("Fixed world frames use normal Chaos without scanning the kinematic world"),Site->PrepareKinematicPhysicsFrame());
    World->DeltaTimeSeconds=0.f;
    TestTrue(TEXT("A purely kinematic world uses a query-only synchronization frame"),Site->PrepareKinematicPhysicsFrame());
    Site->ShipContainers[0]->Body->SetSimulatePhysics(true);
    TestFalse(TEXT("A free rigid body retains normal physics integration"),Site->PrepareKinematicPhysicsFrame());
    Site->ShipContainers[0]->Body->PutRigidBodyToSleep();
    TestFalse(TEXT("Even a sleeping free body retains normal physics integration"),Site->PrepareKinematicPhysicsFrame());
    World->DestroyWorld(false);
    return true;
}
#endif
