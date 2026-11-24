#if WITH_DEV_AUTOMATION_TESTS
#include "PortContainerActor.h"
#include "PortAGVActor.h"
#include "PortSiteLogistics.h"
#include "PortWorkingCrane.h"
#include "STSDynamics.h"
#include "UnchangedUpdates.h"
#include "Misc/ScopeExit.h"
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

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUnchangedCargoTest,"PortSim.Performance.UnchangedCargoInterpolation",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FUnchangedCargoTest::RunTest(const FString& Parameters)
{
    PortSimUpdates::SkipUnchanged(); // Register the A/B switch.
    auto* Switch=IConsoleManager::Get().FindConsoleVariable(TEXT("PortSim.SkipUnchangedUpdates"));
    const int32 Previous=Switch->GetInt();
    ON_SCOPE_EXIT { Switch->Set(Previous,ECVF_SetByCode); };
    const auto Init=UWorld::InitializationValues().AllowAudioPlayback(false).CreatePhysicsScene(true)
        .RequiresHitProxies(false).CreateNavigation(false).CreateAISystem(false);
    auto* World=UWorld::CreateWorld(EWorldType::Game,false,NAME_None,nullptr,true,ERHIFeatureLevel::Num,&Init);
    ON_SCOPE_EXIT { World->DestroyWorld(false); };
    auto* Box=World->SpawnActor<APortContainerActor>();
    for(int32 Frame=0;Frame<4;++Frame)
    {
        // Includes stationary endpoints and a new frame at the same alpha.
        Box->SetSecuredVesselMotion(FVector(1000+Frame*100,0,300),FQuat::Identity,FVector::ZeroVector);
        Box->SetSecuredVesselMotion(FVector(1200+Frame*100,50,320),FRotator(1,3,2).Quaternion(),
            FVector(10,2,1),FVector(1,2,3),FVector(3,2,1),.05,false);
        Box->SetSecuredMotionWindow(.2,.8);
        for(double Alpha:{0.,.5,1.,1.,.5})
        {
            Switch->Set(0,ECVF_SetByCode); Box->ApplySecuredMotionFraction(Alpha);
            const FTransform Expected=Box->GetActorTransform();
            const FVector Velocity=Box->GetMotionVelocity(),Acceleration=Box->GetMotionAcceleration(),Jerk=Box->GetMotionJerk();
            Switch->Set(1,ECVF_SetByCode);
            Box->SetActorLocation(FVector(-10000,0,0)); // Reuse must still correct external pose changes.
            Box->ApplySecuredMotionFraction(Alpha);
            // Reapplying the transform normalizes its quaternion again. Permit
            // rounding noise while retaining a sub-nanometre position bound.
            TestTrue(TEXT("Reused pose matches uncached interpolation"),Box->GetActorTransform().Equals(Expected,1.e-8));
            TestTrue(TEXT("Velocity preserved"),Box->GetMotionVelocity().Equals(Velocity,0.));
            TestTrue(TEXT("Acceleration preserved"),Box->GetMotionAcceleration().Equals(Acceleration,0.));
            TestTrue(TEXT("Jerk preserved"),Box->GetMotionJerk().Equals(Jerk,0.));
        }
    }
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FActiveCargoPresentationTest,"PortSim.Performance.ActiveCargoPresentation",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FActiveCargoPresentationTest::RunTest(const FString& Parameters)
{
    const auto Init=UWorld::InitializationValues().AllowAudioPlayback(false).CreatePhysicsScene(true)
        .RequiresHitProxies(false).CreateNavigation(false).CreateAISystem(false);
    auto* World=UWorld::CreateWorld(EWorldType::Game,false,NAME_None,nullptr,true,ERHIFeatureLevel::Num,&Init);
    ON_SCOPE_EXIT { World->DestroyWorld(false); };
    auto* Site=World->SpawnActor<APortSiteLogistics>();
    Site->LaneCount=9;
    Site->AddShipCargo(FVector(-20000,0,1000),0);
    Site->AddShipCargo(FVector(-20000,2500,1000),0);
    Site->AddShipCargo(FVector(-20000,5000,1000),0);
    auto* Pending=Site->ShipContainers[0].Get();
    auto* Active=Site->ShipContainers[1].Get();
    auto* Completed=Site->ShipContainers[2].Get();
    Completed->LocationOwner=ECargoOwner::Yard;
    const FVector PendingPose=Pending->GetActorLocation(),ActivePose=Active->GetActorLocation(),CompletedPose=Completed->GetActorLocation();
    auto* Crane=World->SpawnActor<APortWorkingCrane>();
    Crane->CargoActor=Active; Site->Equipment.Add(Crane);
    TArray<FPortVesselMotion> Motions; Motions.SetNum(1);
    TArray<FVector> Pivots; Pivots.Add(FVector::ZeroVector);
    FPortSeaState Sea;
    Site->SetVesselMotions(Motions,Pivots,Sea); Site->ApplyVesselMotions(.05f);
    Motions[0].TranslationCentimeters=FVector(10000,0,0);
    Site->SetVesselMotions(Motions,Pivots,Sea); Site->ApplyVesselMotions(.05f);
    Site->PublishCargoPresentation();
    TestTrue(TEXT("Unstarted cargo is excluded from presentation"),Pending->GetActorLocation().Equals(PendingPose,0.));
    TestTrue(TEXT("Selected source follows the vessel"),Active->GetActorLocation().Equals(ActivePose+FVector(10000,0,0),.001));
    TestTrue(TEXT("Completed yard cargo is excluded"),Completed->GetActorLocation().Equals(CompletedPose,0.));
    // Publishing must retain old colliders in the broad phase even when the
    // current vessel sweep is far away from their unrefreshed physical poses.
    Site->SynchronizeShipCollision(FBox(PendingPose-FVector(100),PendingPose+FVector(100)),1.);
    TestTrue(TEXT("Query at stale pose synchronizes pending collider"),Pending->GetActorLocation().Equals(PendingPose+FVector(10000,0,0),.001));
    Sea.bSeaMotionEnabled=false; Motions[0]=FPortVesselMotion();
    Site->SetVesselMotions(Motions,Pivots,Sea);
    Site->ApplyVesselMotions(.05f); Site->ApplyVesselMotions(.05f);
    Site->SynchronizeShipCollision(FBox(PendingPose-FVector(100),PendingPose+FVector(10100,100,100)),1.);
    const uint64 Version=Site->CargoMotionVersion;
    for(int32 I=0;I<20;++I) Site->ApplyVesselMotions(.05f);
    TestEqual(TEXT("Sea OFF reuses stationary motion instead of recalculating cargo"),Site->CargoMotionVersion,Version);
    TestTrue(TEXT("Sea OFF queries return the fixed base pose"),Pending->GetActorLocation().Equals(PendingPose,.001));
    Sea.bSeaMotionEnabled=true; Motions[0].TranslationCentimeters=FVector(200,0,0);
    Site->SetVesselMotions(Motions,Pivots,Sea); Site->ApplyVesselMotions(.05f); Site->PublishCargoPresentation();
    TestTrue(TEXT("Sea ON invalidates stationary cache"),Site->CargoMotionVersion>Version);
    TestTrue(TEXT("Active source resumes vessel motion"),Active->GetActorLocation().Equals(ActivePose+FVector(200,0,0),.001));
    return true;
}

#endif
