#include "PortWorkingCrane.h"
#include "PortContainerActor.h"
#include "PortAGVActor.h"
#include "Components/StaticMeshComponent.h"
#include "Components/TextRenderComponent.h"
#include "Engine/World.h"
#include "Materials/MaterialInterface.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"

namespace WorkingCrane { constexpr float LiftOffset=154.5f; }

APortWorkingCrane::APortWorkingCrane()
{
    for (int32 I=0;I<4;++I)
    {
        Legs.Add(Box(*FString::Printf(TEXT("Leg_%d"),I),RootComponent,FVector::ZeroVector,FVector(100),true));
        Bogies.Add(Box(*FString::Printf(TEXT("Bogie_%d"),I),RootComponent,FVector::ZeroVector,FVector(100),true));
        Ropes.Add(Box(*FString::Printf(TEXT("Rope_%d"),I),RootComponent,FVector::ZeroVector,FVector(6)));
    }
    for (int32 I=0;I<2;++I)
    {
        CrossBeams.Add(Box(*FString::Printf(TEXT("CrossBeam_%d"),I),RootComponent,FVector::ZeroVector,FVector(100)));
        Beams.Add(Box(*FString::Printf(TEXT("Beam_%d"),I),RootComponent,FVector::ZeroVector,FVector(100)));
        auto* Pad=Box(*FString::Printf(TEXT("Slot_%d"),I),RootComponent,FVector::ZeroVector,FVector(100),true);
        Pad->SetAbsolute(true,true,true); // Fixed supports do not follow gantry travel.
        Pads.Add(Pad);
    }
    Trolley=Box(TEXT("Trolley"),RootComponent,FVector::ZeroVector,FVector(420,1450,120));
    Spreader=Box(TEXT("Spreader"),RootComponent,FVector::ZeroVector,FVector(244,1220,50));
    Mast=Box(TEXT("Mast"),RootComponent,FVector::ZeroVector,FVector(100));
    NameLabel=Label(TEXT("CraneName"),FVector::ZeroVector);
    Tags.Add(TEXT("PortSim.WorkingCrane"));
}

void APortWorkingCrane::Configure(int32 Number,bool bQuayside,FVector Source,FVector Destination,bool bCreateCargo)
{
    check(!bConfigured);
    bExternalJobs=!bCreateCargo; CraneID=Number; bSTS=bQuayside; Home=GetActorLocation(); Orientation=GetActorQuat();
    Slots[0]=Source; Slots[1]=Destination;
    BeamZ=bSTS && STSProfile.bReady?STSProfile.BeamHeight:(bSTS?3000.f:2600.f);
    SafeZ=bSTS && STSProfile.bReady?STSProfile.SafeHeight:1900.f;
    const float HalfGauge=bSTS && STSProfile.bReady?STSProfile.RailGauge*.5f:(bSTS?850.f:1600.f);
    const float GaugeCenter=bSTS && STSProfile.bReady?STSProfile.WatersideRailX+HalfGauge:0.f;
    const float HalfBase=bSTS?1000.f:820.f;
    const float Height=bSTS?BeamZ-100.f:2560.f;
    for (int32 I=0;I<4;++I)
    {
        Legs[I]->SetRelativeLocation(FVector(GaugeCenter+(I<2?-1:1)*HalfGauge,(I%2?-1:1)*HalfBase,bSTS?Height*.5f:1330.f));
        Legs[I]->SetRelativeScale3D(FVector(1,1,Height/100.f));
        Bogies[I]->SetRelativeLocation(FVector(GaugeCenter+(I<2?-1:1)*HalfGauge,(I%2?-1:1)*HalfBase,100));
        Bogies[I]->SetRelativeScale3D(bSTS?FVector(1.8,4.2,1.8):FVector(1.6,3.2,1.6));
    }
    for (int32 I=0;I<2;++I)
    {
        const float BeamCenter=bSTS && STSProfile.bReady?(STSProfile.MinTrolley()+STSProfile.MaxTrolley())*.5f:(bSTS?-1500.f:0.f);
        Beams[I]->SetRelativeLocation(FVector(BeamCenter,(I?-1:1)*(bSTS?730.f:650.f),bSTS?BeamZ+100.f:2600.f));
        CrossBeams[I]->SetRelativeLocation(FVector(GaugeCenter+(I?-1:1)*HalfGauge,0,bSTS?BeamZ-100.f:2580.f));
        CrossBeams[I]->SetRelativeScale3D(bSTS?FVector(1.4,22,1.4):FVector(1.6,18,1.6));
        const float BeamLength=bSTS && STSProfile.bReady?(STSProfile.MaxTrolley()-STSProfile.MinTrolley()+400.f)/100.f:(bSTS?125.f:34.f);
        Beams[I]->SetRelativeScale3D(FVector(BeamLength,bSTS?1.f:1.2f,1.8f));
        Pads[I]->SetWorldLocationAndRotation(Slots[I]-FVector(0,0,139.5f),Orientation);
        Pads[I]->SetWorldScale3D(FVector(3.1f,13.f,.2f));
    }
    Mast->SetRelativeLocation(FVector(0,0,BeamZ+1200));
    Mast->SetRelativeScale3D(FVector(3,3,24));
    Mast->SetVisibility(false); // Use the central STS twin-boom silhouette, without a dummy mast.
    Trolley->SetRelativeScale3D(bSTS?FVector(3.6,14.5,1):FVector(4.2,14.5,1.2));
    NameLabel->SetRelativeLocation(FVector(0,0,BeamZ+150));
    NameLabel->SetWorldSize(170);
    NameLabel->SetText(FText::FromString(FString::Printf(TEXT("%s %02d"),bSTS?TEXT("CC 24-row / STS"):TEXT("TC / RMG"),CraneID)));
    auto Material=[](const TCHAR* Name) { return LoadObject<UMaterialInterface>(nullptr,*FString::Printf(TEXT("/Game/PortSim/Assets/Materials/M_%s.M_%s"),Name,Name)); };
    for (const auto& Part:Legs) Part->SetMaterial(0,Material(bSTS?TEXT("CraneYellow"):TEXT("Target")));
    for (const auto& Part:Beams) Part->SetMaterial(0,Material(bSTS?TEXT("CraneYellow"):TEXT("Target")));
    for (const auto& Part:Ropes) Part->SetMaterial(0,Material(TEXT("SiteRoad")));
    for (const auto& Part:Pads) Part->SetMaterial(0,Material(TEXT("SiteWhite")));
    for (const auto& Part:Bogies) Part->SetMaterial(0,Material(TEXT("Steel")));
    for (const auto& Part:CrossBeams) Part->SetMaterial(0,Material(bSTS?TEXT("CraneYellow"):TEXT("Target")));
    Mast->SetMaterial(0,Material(TEXT("CraneYellow")));
    Trolley->SetMaterial(0,Material(bSTS?TEXT("CraneYellow"):TEXT("Target")));
    Spreader->SetMaterial(0,Material(bSTS?TEXT("CraneYellow"):TEXT("Target")));
    if (bCreateCargo)
    {
    FActorSpawnParameters Params;
    Params.Owner=this;
    Params.SpawnCollisionHandlingOverride=ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
    CargoActor=GetWorld()->SpawnActor<APortContainerActor>(Source,Orientation.Rotator(),Params);
    check(CargoActor);
    CargoActor->InitializeContainer(1000+Number);
    if(bSTS && STSProfile.bReady) CargoActor->SetPhysicalParameters(STSProfile.ContainerMassKg,STSProfile.ContainerCoG);
    }
    bConfigured=true;
#if WITH_EDITOR
    SetActorLabel(FString::Printf(TEXT("Working_%s_%02d"),bSTS?TEXT("STS"):TEXT("RMG"),Number));
    SetFolderPath(TEXT("PortSim/WorkingEquipment"));
#endif
    ResetOperation();
    if(bSTS && STSProfile.bReady) BuildSTSSensors();
    bSensorFault=FParse::Param(FCommandLine::Get(),TEXT("PortSimSTSSensorFault"));
    FParse::Value(FCommandLine::Get(),TEXT("PortSimSTSLockFault="),LockFault);
    if(bSTS && !STSProfile.bReady) Stop(TEXT("STS profile unavailable: ")+STSProfile.Error);
}

FVector APortWorkingCrane::Local(FVector World) const { return Orientation.UnrotateVector(World-Home); }
FVector APortWorkingCrane::HeadPosition() const { return Home+Orientation.RotateVector(Head+SuspendedOffset); }

void APortWorkingCrane::ResetOperation()
{
    if (!bConfigured) return;
    ClearSTSState();
    SuspensionState=FSTSSuspension(); SuspendedOffset=FVector::ZeroVector;
    if (bExternalJobs)
    {
        if (IsValid(CargoActor)) CargoActor->DetachFromActor(FDetachmentTransformRules::KeepWorldTransform);
        CargoActor=nullptr; bJobActive=bCarrying=bPaused=bResumePhysics=false; bDestinationReady=true;
        SourceSlot=Stage=CompletedJobs=0; Fault.Empty(); Speed=StageTime=SettleTime=0;
        Head=Local(Slots[0]); Head.Z=SafeZ; JobStartHead=Head; UpdateParts();
        for (const auto& Pad:Pads) { Pad->SetCollisionEnabled(ECollisionEnabled::NoCollision); Pad->SetVisibility(false); }
        return;
    }
    bJobActive=true;
    CargoActor->DetachFromActor(FDetachmentTransformRules::KeepWorldTransform);
    auto* Body=CargoActor->GetBody();
    Body->SetSimulatePhysics(false);
    CargoActor->SetActorLocationAndRotation(Slots[0],Orientation.Rotator(),false,nullptr,ETeleportType::TeleportPhysics);
    Body->SetSimulatePhysics(true);
    Body->SetPhysicsLinearVelocity(FVector::ZeroVector);
    Body->SetPhysicsAngularVelocityInDegrees(FVector::ZeroVector);
    CargoActor->LocationOwner=bSTS?ECargoOwner::Ship:ECargoOwner::Yard;
    SourceSlot=Stage=CompletedJobs=0;
    bCarrying=bPaused=bResumePhysics=false;
    Fault.Empty(); Speed=StageTime=0; SettleTime=0;
    Head=Local(Slots[0]); Head.Z=SafeZ;
    UpdateParts();
}

void APortWorkingCrane::SetOperationPaused(bool Paused)
{
    if (!bConfigured || bPaused==Paused) return;
    bPaused=Paused;
    if (!IsValid(CargoActor)) return;
    auto* Body=CargoActor->GetBody();
    if (Paused)
    {
        bResumePhysics=Body->IsSimulatingPhysics();
        PausedVelocity=Body->GetPhysicsLinearVelocity();
        PausedAngularVelocity=Body->GetPhysicsAngularVelocityInDegrees();
        if (bResumePhysics) Body->SetSimulatePhysics(false);
    }
    else if (bResumePhysics)
    {
        Body->SetSimulatePhysics(true);
        Body->SetPhysicsLinearVelocity(PausedVelocity);
        Body->SetPhysicsAngularVelocityInDegrees(PausedAngularVelocity);
        bResumePhysics=false;
    }
}

void APortWorkingCrane::UpdateParts()
{
    SetActorLocation(Home+Orientation.RotateVector(FVector(0,Head.Y,0)));
    Trolley->SetRelativeLocation(FVector(Head.X,0,BeamZ));
    Spreader->SetRelativeLocation(FVector(Head.X,0,Head.Z)+SuspendedOffset);
    Spreader->SetRelativeRotation(FRotator(0,bSTS?FMath::RadiansToDegrees(SuspensionState.Yaw):0,0));
    for (int32 I=0;I<4;++I)
    {
        const FVector Corner((I&1)?100:-100,(I&2)?520:-520,0);
        const FVector Top=FVector(Head.X,0,BeamZ)+Corner;
        const FVector Bottom=FVector(Head.X,0,Head.Z)+SuspendedOffset+Spreader->GetRelativeRotation().RotateVector(Corner);
        const FVector Delta=Top-Bottom;
        Ropes[I]->SetRelativeLocation((Top+Bottom)*.5f);
        Ropes[I]->SetRelativeRotation(FRotationMatrix::MakeFromZ(Delta).Rotator());
        Ropes[I]->SetRelativeScale3D(FVector(.06f,.06f,FMath::Max(1.,Delta.Size())/100.));
    }
}

bool APortWorkingCrane::MoveHead(FVector Target,float Dt)
{
    if(bSTS) return MoveSTS(Target,Dt);
    const float Distance=FVector::Dist(Head,Target);
    const float Limit=FMath::Abs(Target.Z-Head.Z)>1.f?200.f:350.f;
    Speed=FMath::FInterpConstantTo(Speed,FMath::Min(Limit,FMath::Sqrt(2*150.f*Distance)),Dt,150.f);
    Head=FMath::VInterpConstantTo(Head,Target,Dt,Speed);
    UpdateParts();
    if (!Head.Equals(Target,.1f)) return false;
    Head=Target; Speed=0; UpdateParts(); return true;
}

bool APortWorkingCrane::DestinationClear() const
{
    FCollisionQueryParams Query(SCENE_QUERY_STAT(WorkingCraneSlot),false);
    Query.AddIgnoredActor(CargoActor);
    // Shrink by 2 cm so the supporting pad/ground is not treated as an obstruction.
    return !GetWorld()->OverlapBlockingTestByChannel(Slots[1-SourceSlot],Orientation,ECC_WorldDynamic,
        FCollisionShape::MakeBox(FVector(120,608,127.5)),Query);
}

void APortWorkingCrane::Stop(const FString& Reason)
{
    Fault=Reason; SetOperationPaused(true);
    UE_LOG(LogTemp,Error,TEXT("WORKING_CRANE_%d: %s"),CraneID,*Reason);
}

void APortWorkingCrane::Advance(float Dt,bool bGlobalPaused)
{
    if (!bConfigured) return;
    SimulationTime+=Dt;
    if(bJobActive) { JobSeconds+=Dt; if(bGlobalPaused || !bEnabled || !Fault.IsEmpty()) PausedSeconds+=Dt; }
    SetOperationPaused(bGlobalPaused || !bEnabled || !Fault.IsEmpty());
    if (bPaused || !bJobActive) return;
    if(bSTS)
    {
        SampleSTS();
        if(!STSProfile.bReady || !Observation.IsFresh(SimulationTime,STSProfile.SensorMaxAge))
        { Stop(TEXT("Required STS sensor observation invalid/stale")); return; }
        if(bCarrying && (!Observation.AllLocked() || Observation.DynamicPayloadEstimateKg()>STSProfile.RatedPayloadKg))
        { Stop(TEXT("Loaded hoist interlock: locks/load observation invalid")); return; }
        if(((Stage==5 && bDestinationReady) || Stage==6 || Stage==7) && !Observation.bAGVAligned)
        { Stop(TEXT("AGV alignment lost during STS handover")); return; }
    }
    // Pick and lift the next ship box while the vehicle is away; hold it safely
    // above the dock until the returning AGV is stopped beneath the spreader.
    if (Stage==5 && !bDestinationReady) { if(bSTS) MoveSTS(Head,Dt); return; }
    StageTime+=Dt;
    if (StageTime>(bSTS?STSProfile.StageTimeout:180.f)) { Stop(TEXT("Job stage timed out")); return; }
    if(bSTS && Stage==2){AdvancePickup(Dt);return;}
    const FVector Source=bSTS && Stage==3?Local(Pickup.TrialOrigin):Local(Slots[SourceSlot]);
    const FVector Destination=Local(Slots[1-SourceSlot]);
    FVector Target=Head;
    switch(Stage)
    {
    case 0: Target.Z=SafeZ; break;
    case 1: Target=FVector(Source.X,Source.Y,SafeZ); break;
    case 2: Target=Source+FVector(0,0,WorkingCrane::LiftOffset); break;
    case 3: Target=FVector(Source.X,Source.Y,SafeZ); break;
    case 4: Target=FVector(Destination.X,Destination.Y,SafeZ); break;
    case 5:
        // Pickup retains the achieved coupling offset. Compensate it from
        // measurements so the cargo, rather than an ideal head centre, lands on the AGV.
        Target=Destination+(bSTS?Orientation.UnrotateVector(Observation.SpreaderPosition-Observation.CargoPosition):FVector(0,0,WorkingCrane::LiftOffset));
        break;
    case 6: Target=FVector(Destination.X,Destination.Y,SafeZ); break;
    case 7:
        SettleTime+=Dt;
        if (SettleTime<1.5f) return;
        if (FVector::Dist(CargoActor->GetActorLocation(),Slots[1-SourceSlot])>10.f ||
            CargoActor->GetBody()->GetPhysicsLinearVelocity().Size()>5.f || CargoActor->GetActorUpVector().Z<.99f)
        { Stop(TEXT("Placed cargo failed physical settling check")); return; }
        ++CompletedJobs;
        LastJobSeconds=JobSeconds; LastPausedSeconds=PausedSeconds;
        if (bExternalJobs) { bJobActive=false; CargoActor=nullptr; Stage=0; StageTime=SettleTime=0; return; }
        SourceSlot=1-SourceSlot; Stage=0; StageTime=SettleTime=0;
        if(bSTS){Pickup=FSTSPickupController();for(double& Progress:LockProgress)Progress=0;}
        return;
    default: Stop(TEXT("Invalid job stage")); return;
    }
    if (!MoveHead(Target,Dt)) return;
    if (Stage==2)
    {
        if (FVector::Dist(CargoActor->GetActorLocation(),Slots[SourceSlot])>10.f ||
            CargoActor->GetActorUpVector().Z<.99f || CargoActor->GetBody()->GetPhysicsLinearVelocity().Size()>5.f)
        {
            if (StageTime>30.f) Stop(FString::Printf(TEXT("Pickup alignment: error %.2f cm, speed %.2f, up %.3f"),
                FVector::Dist(CargoActor->GetActorLocation(),Slots[SourceSlot]),CargoActor->GetBody()->GetPhysicsLinearVelocity().Size(),CargoActor->GetActorUpVector().Z));
            return;
        }
        if (bDestinationReady && !DestinationClear()) { Stop(TEXT("Destination slot occupied")); return; }
        CargoActor->DetachFromActor(FDetachmentTransformRules::KeepWorldTransform);
        auto* Body=CargoActor->GetBody();
        Body->SetSimulatePhysics(false);
        CargoActor->SetActorLocationAndRotation(HeadPosition()-FVector(0,0,WorkingCrane::LiftOffset),Spreader->GetComponentRotation());
        CargoActor->AttachToComponent(Spreader,FAttachmentTransformRules::KeepWorldTransform);
        CargoActor->LocationOwner=bSTS?ECargoOwner::STS:ECargoOwner::RMG;
        bCarrying=true;
    }
    if (Stage==5)
    {
        if(bSTS)
        {
            if(!Observation.bCargoSupported || Observation.SpreaderVelocity.Size()>STSProfile.SettleSpeed ||
                !Observation.CargoPosition.Equals(Slots[1-SourceSlot],STSProfile.LandingTolerance)) { SettleTime=0; return; }
            SettleTime+=Dt;
            if(SettleTime<STSProfile.SettleTime) return;
            for(bool& Locked:CornerLocked) Locked=false;
            for(bool& Requested:Pickup.RequestLocks) Requested=false;
        }
        if (!DestinationClear()) { Stop(TEXT("Destination became occupied")); return; }
        CargoActor->DetachFromActor(FDetachmentTransformRules::KeepWorldTransform);
        auto* Body=CargoActor->GetBody();
        Body->SetSimulatePhysics(true);
        Body->SetPhysicsLinearVelocity(FVector::ZeroVector);
        Body->SetPhysicsAngularVelocityInDegrees(FVector::ZeroVector);
        CargoActor->LocationOwner=bSTS && (1-SourceSlot)==0?ECargoOwner::Ship:ECargoOwner::Yard;
        bCarrying=false;
    }
    ++Stage; StageTime=SettleTime=0;
    if(bSTS) SampleSTS(true);
}

bool APortWorkingCrane::ValidateOperation(FString& Error) const
{
    if (bConfigured && bExternalJobs && !bJobActive && Fault.IsEmpty()) return true;
    if (!bConfigured || !IsValid(CargoActor) || (!bExternalJobs && CargoActor->GetOwner()!=this) || !Fault.IsEmpty())
    { Error=FString::Printf(TEXT("Crane %d: configuration/cargo/fault: %s"),CraneID,*Fault); return false; }
    const FVector A=Local(Slots[0]), B=Local(Slots[1]);
    const double Margin=bSTS?STSProfile.Pickup.AcquisitionRadius:1;
    if (Head.X<FMath::Min3(A.X,B.X,JobStartHead.X)-Margin || Head.X>FMath::Max3(A.X,B.X,JobStartHead.X)+Margin ||
        Head.Y<FMath::Min3(A.Y,B.Y,JobStartHead.Y)-Margin || Head.Y>FMath::Max3(A.Y,B.Y,JobStartHead.Y)+Margin || Head.Z>SafeZ+1)
    { Error=TEXT("Crane left its reserved envelope"); return false; }
    if (bCarrying && (CargoActor->GetAttachParentActor()!=this || CargoActor->GetBody()->IsSimulatingPhysics() ||
        (bSTS?!CargoActor->GetActorTransform().Equals(LockedCargoTransform*Spreader->GetComponentTransform(),1.f):
            !CargoActor->GetActorLocation().Equals(HeadPosition()-FVector(0,0,WorkingCrane::LiftOffset),1.f))))
    { Error=TEXT("Locked cargo is detached or out of alignment"); return false; }
    if (!bCarrying && CargoActor->GetAttachParentActor() &&
        !(Stage<=2 && CargoActor->LocationOwner==ECargoOwner::AGV && !CargoActor->GetBody()->IsSimulatingPhysics()))
    { Error=TEXT("Released cargo remains attached"); return false; }
    return true;
}

void APortWorkingCrane::EndPlay(const EEndPlayReason::Type Reason)
{
    if (!bExternalJobs && IsValid(CargoActor)) CargoActor->Destroy();
    CargoActor=nullptr;
    Super::EndPlay(Reason);
}

bool APortWorkingCrane::AssignCargo(APortContainerActor* Cargo,FVector Source,FVector Destination,bool SourceSupport,bool DestinationSupport,APortAGVActor* HandoverVehicle)
{
    if (!bExternalJobs || bJobActive || !IsValid(Cargo) || !Fault.IsEmpty()) return false;
    if(bSTS)
    {
        if(!STSProfile.bReady) { Stop(TEXT("STS profile unavailable")); return false; }
        if(!FMath::IsFinite(Cargo->MassKg) || Cargo->MassKg<=0 || Cargo->MassKg>STSProfile.RatedPayloadKg)
        { Stop(TEXT("Payload exceeds resolved STS reference capacity")); return false; }
        if(!STSProfile.ContainsTarget(Local(Source)+FVector(0,0,WorkingCrane::LiftOffset)) ||
            !STSProfile.ContainsTarget(Local(Destination)+FVector(0,0,WorkingCrane::LiftOffset)))
        { Stop(TEXT("Source/destination outside reference STS working envelope")); return false; }
    }
    ClearSTSState(); HandoverAGV=HandoverVehicle;
    CargoActor=Cargo; Slots[0]=Source; Slots[1]=Destination; SourceSlot=Stage=0;
    bDestinationReady=true;
    bJobActive=true; bCarrying=false; Speed=StageTime=SettleTime=0; JobStartHead=Head;
    for (int32 I=0;I<2;++I)
    {
        const bool Support=I==0?SourceSupport:DestinationSupport;
        Pads[I]->SetWorldLocationAndRotation(Slots[I]-FVector(0,0,139.5f),Orientation);
        Pads[I]->SetCollisionEnabled(Support?ECollisionEnabled::QueryAndPhysics:ECollisionEnabled::NoCollision);
        Pads[I]->SetVisibility(Support);
    }
    if(bSTS) SampleSTS(true);
    return true;
}
