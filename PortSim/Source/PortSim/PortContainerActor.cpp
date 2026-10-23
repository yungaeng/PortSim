#include "PortContainerActor.h"
#include "PortAGVActor.h"
#include "PortEnvironmentComponent.h"
#include "PortAGVActor.h"
#include "PortWorkingCrane.h"
#include "AGVReference.h"
#include "TerminalLayout.h"
#include "Components/StaticMeshComponent.h"
#include "Components/TextRenderComponent.h"
#include "Engine/StaticMesh.h"
#include "UObject/ConstructorHelpers.h"

APortContainerActor::APortContainerActor()
{
    PrimaryActorTick.bCanEverTick=false;
    static ConstructorHelpers::FObjectFinder<UStaticMesh> Cube(TEXT("/Engine/BasicShapes/Cube.Cube"));
    Body=CreateDefaultSubobject<UStaticMeshComponent>(TEXT("ContainerBody"));
    SetRootComponent(Body);
    Body->SetStaticMesh(Cube.Object);
    Body->SetWorldScale3D(ExternalDimensionsCm/100.f);
    Body->SetMobility(EComponentMobility::Movable);
    Body->SetCollisionProfileName(TEXT("BlockAllDynamic"));
    // Sensors use explicit traces/overlap queries, never overlap-event callbacks.
    // Moving every secured box otherwise refreshes event pairs at every substep.
    Body->SetGenerateOverlapEvents(false);
    Body->SetSimulatePhysics(true);
    Body->SetLinearDamping(.4f);
    Body->SetAngularDamping(2.f);
    Body->BodyInstance.bUseCCD=true;
    Body->BodyInstance.PositionSolverIterationCount=16;
    Body->BodyInstance.VelocitySolverIterationCount=8;
    Visual=CreateDefaultSubobject<UStaticMeshComponent>(TEXT("ContainerVisual"));
    Visual->SetupAttachment(Body);
    Visual->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    Visual->SetGenerateOverlapEvents(false);
    Visual->SetAbsolute(false,false,true);
    Tags.Add(TEXT("PortSim.Container"));
}

void APortContainerActor::BeginPlay()
{
    Super::BeginPlay();
    SetPhysicalParameters(MassKg,CoGOffsetCm);
    ApplyContainerAppearance();
}

void APortContainerActor::SetPhysicalParameters(float Mass,FVector CoGOffset)
{
    MassKg=Mass; CargoMassKg=FMath::Max(0.f,MassKg-TareMassKg); CoGOffsetCm=CoGOffset;
    Body->SetMassOverrideInKg(NAME_None,MassKg);
    // Chaos scales COMNudge by the body scale. CoGOffsetCm is already a physical
    // centimetre offset; compensate so a 40ft mesh does not multiply it again.
    Body->SetCenterOfMass(CoGOffsetCm/Body->GetComponentScale());
}

void APortContainerActor::ApplyWind(FVector WindVelocityMetersPerSecond,float DragCoefficient)
{
    if(!Body->IsSimulatingPhysics()) return;
    const FVector DimensionsMeters=ExternalDimensionsCm*.01;
    const FVector ForceNewtons=FPortWindAerodynamics::DragForceNewtons(WindVelocityMetersPerSecond,
        Body->GetPhysicsLinearVelocity(),Body->GetComponentQuat(),DimensionsMeters,DragCoefficient);
    // Chaos uses centimetres, so one SI newton is 100 kg*cm/s^2.
    Body->AddForce(ForceNewtons*100.f);
}

void APortContainerActor::SetSecuredVesselMotion(FVector Position,FQuat Rotation,FVector VelocityCentimetersPerSecond,
    FVector AccelerationCentimetersPerSecondSquared,FVector JerkCentimetersPerSecondCubed,double FrameDurationSeconds,bool ApplyPose)
{
    const bool WasSimulating=Body->IsSimulatingPhysics();
    if(WasSimulating) Body->SetSimulatePhysics(false);
    const FTransform Next(Rotation.GetNormalized(),Position);
    if(!bHasSecuredMotionFrame || WasSimulating)
    {
        PreviousSecuredTransform=Next;
        PreviousSecuredVelocity=VelocityCentimetersPerSecond;
        PreviousSecuredAcceleration=AccelerationCentimetersPerSecondSquared;
        PreviousSecuredJerk=JerkCentimetersPerSecondCubed;
    }
    else
    {
        PreviousSecuredTransform=CurrentSecuredTransform;
        PreviousSecuredVelocity=CurrentSecuredVelocity;
        PreviousSecuredAcceleration=CurrentSecuredAcceleration;
        PreviousSecuredJerk=CurrentSecuredJerk;
    }
    CurrentSecuredTransform=Next;
    CurrentSecuredVelocity=VelocityCentimetersPerSecond;
    CurrentSecuredAcceleration=AccelerationCentimetersPerSecondSquared;
    CurrentSecuredJerk=JerkCentimetersPerSecondCubed;
    SecuredFrameDurationSeconds=bHasSecuredMotionFrame && !WasSimulating && FMath::IsFinite(FrameDurationSeconds)?
        FMath::Max(0.,FrameDurationSeconds):0.;
    bHasSecuredMotionFrame=true;
    SecuredWindowStart=0; SecuredWindowEnd=1;
    // Hermite position is a cubic Bezier curve. Its four control points bound
    // the whole path, including overshoot between endpoints. A rotation chord
    // bounds every corner through slerp without an oversized spherical box.
    SecuredSweptBounds=FBox(ForceInit);
    SecuredSweptBounds+=PreviousSecuredTransform.GetLocation();
    SecuredSweptBounds+=PreviousSecuredTransform.GetLocation()+PreviousSecuredVelocity*SecuredFrameDurationSeconds/3.;
    SecuredSweptBounds+=CurrentSecuredTransform.GetLocation()-CurrentSecuredVelocity*SecuredFrameDurationSeconds/3.;
    SecuredSweptBounds+=CurrentSecuredTransform.GetLocation();
    const FVector Half=ExternalDimensionsCm*.5;
    const FVector InitialExtent=FBox(-Half,Half).TransformBy(FTransform(PreviousSecuredTransform.GetRotation())).GetExtent();
    const double Angle=PreviousSecuredTransform.GetRotation().AngularDistance(CurrentSecuredTransform.GetRotation());
    const double RotationMargin=2.*Half.Size()*FMath::Sin(Angle*.5)+2.;
    SecuredSweptBounds=SecuredSweptBounds.ExpandBy(InitialExtent+FVector(RotationMargin));
    if(ApplyPose) ApplySecuredMotionFraction(1.);
}

void APortContainerActor::SetSecuredMotionWindow(double Start,double End)
{
    SecuredWindowStart=FMath::Clamp(Start,0.,1.);
    SecuredWindowEnd=FMath::Clamp(End,SecuredWindowStart,1.);
}

void APortContainerActor::BeginVisualFrame()
{
    if(DeferredVisualParent.IsValid() || !Visual->GetAttachParent()) return;
    check(Visual->GetCollisionEnabled()==ECollisionEnabled::NoCollision);
    DeferredVisualParent=Visual->GetAttachParent();
    DeferredVisualRelative=Visual->GetRelativeTransform();
    // Only the non-colliding artwork is deferred. Body remains the root and
    // updates its physics/query pose at every original control substep.
    Visual->DetachFromComponent(FDetachmentTransformRules::KeepWorldTransform);
}

void APortContainerActor::EndVisualFrame()
{
    auto* Parent=DeferredVisualParent.Get(); if(!Parent) return;
    Visual->SetRelativeTransform(DeferredVisualRelative);
    Visual->AttachToComponent(Parent,FAttachmentTransformRules::KeepRelativeTransform);
    DeferredVisualParent.Reset();
}

void APortContainerActor::ApplySecuredMotionFraction(double Fraction)
{
    if(!bHasSecuredMotionFrame || Body->IsSimulatingPhysics()) return;
    const double Alpha=FMath::Lerp(SecuredWindowStart,SecuredWindowEnd,FMath::Clamp(Fraction,0.,1.));
    FVector Position,Velocity;
    if(SecuredFrameDurationSeconds>UE_SMALL_NUMBER)
    {
        // Cubic Hermite interpolation keeps the position derivative consistent
        // with the analytic vessel velocity. Linear position interpolation plus
        // an unrelated velocity sample destabilizes pickup at high playback.
        const double T=Alpha,T2=T*T,T3=T2*T,H=SecuredFrameDurationSeconds;
        const FVector P0=PreviousSecuredTransform.GetLocation(),P1=CurrentSecuredTransform.GetLocation();
        Position=(2*T3-3*T2+1)*P0+(T3-2*T2+T)*H*PreviousSecuredVelocity+
            (-2*T3+3*T2)*P1+(T3-T2)*H*CurrentSecuredVelocity;
        Velocity=((6*T2-6*T)*P0+(3*T2-4*T+1)*H*PreviousSecuredVelocity+
            (-6*T2+6*T)*P1+(3*T2-2*T)*H*CurrentSecuredVelocity)/H;
    }
    else
    {
        Position=FMath::Lerp(PreviousSecuredTransform.GetLocation(),CurrentSecuredTransform.GetLocation(),Alpha);
        Velocity=FMath::Lerp(PreviousSecuredVelocity,CurrentSecuredVelocity,Alpha);
    }
    const FQuat Rotation=FQuat::Slerp(PreviousSecuredTransform.GetRotation(),CurrentSecuredTransform.GetRotation(),Alpha).GetNormalized();
    SetActorLocationAndRotation(Position,Rotation,false,nullptr,ETeleportType::TeleportPhysics);
    KinematicVelocityCentimetersPerSecond=Velocity;
    KinematicAccelerationCentimetersPerSecondSquared=FMath::Lerp(PreviousSecuredAcceleration,CurrentSecuredAcceleration,Alpha);
    KinematicJerkCentimetersPerSecondCubed=FMath::Lerp(PreviousSecuredJerk,CurrentSecuredJerk,Alpha);
}

FVector APortContainerActor::GetMotionVelocity() const
{
    if(Body->IsSimulatingPhysics()) return Body->GetPhysicsLinearVelocity();
    // Cached vessel motion only applies while the cargo is secured aboard.
    if(LocationOwner==ECargoOwner::Ship && !GetAttachParentActor()) return KinematicVelocityCentimetersPerSecond;
    if(const auto* Vehicle=Cast<APortAGVActor>(GetAttachParentActor()))
    {
        const auto& D=Vehicle->Dynamics;
        const FVector CoG=Vehicle->GetActorLocation()+Vehicle->GetActorRightVector()*D.CombinedCoGM.X*100-
            Vehicle->GetActorForwardVector()*D.CombinedCoGM.Y*100;
        const FVector Arm=GetActorLocation()-CoG;
        return Vehicle->GetActorRightVector()*D.U*100-Vehicle->GetActorForwardVector()*D.V*100+
            FVector(-D.YawRate*Arm.Y,D.YawRate*Arm.X,0);
    }
    if(const auto* Crane=Cast<APortWorkingCrane>(GetAttachParentActor())) return Crane->Observation.SpreaderVelocity;
    return FVector::ZeroVector;
}

FVector APortContainerActor::GetMotionAcceleration() const
{
    return !Body->IsSimulatingPhysics() && LocationOwner==ECargoOwner::Ship && !GetAttachParentActor()?
        KinematicAccelerationCentimetersPerSecondSquared:FVector::ZeroVector;
}

FVector APortContainerActor::GetMotionJerk() const
{
    return !Body->IsSimulatingPhysics() && LocationOwner==ECargoOwner::Ship && !GetAttachParentActor()?
        KinematicJerkCentimetersPerSecondCubed:FVector::ZeroVector;
}

void APortContainerActor::InitializeContainer(int32 Number)
{
    ContainerID=FName(*FString::Printf(TEXT("C%02d"),Number));
#if WITH_EDITOR
    SetActorLabel(FString::Printf(TEXT("Container_%s"),*ContainerID.ToString()));
    SetFolderPath(TEXT("PortSim/Containers"));
#endif
}

void APortContainerActor::ConfigureSpecification(const FPortContainerSpecification& Specification,FVector CoGOffset)
{
    SizeClass=Specification.Size; LengthFt=Specification.LengthFt;
    ExternalDimensionsCm=FVector(Specification.WidthCm,Specification.LengthCm,Specification.HeightCm);
    TareMassKg=Specification.TareMassKg; CargoMassKg=Specification.CargoMassKg;
    Body->SetWorldScale3D(ExternalDimensionsCm/100.f);
    SetPhysicalParameters(Specification.GrossMassKg(),CoGOffset);
    ApplyContainerAppearance();
}

void APortContainerActor::ResetCargo(FVector Position)
{
    bHasSecuredMotionFrame=false; SecuredFrameDurationSeconds=0;
    Body->SetSimulatePhysics(false);
    SetActorLocationAndRotation(Position,FRotator::ZeroRotator,false,nullptr,ETeleportType::TeleportPhysics);
    Body->SetSimulatePhysics(true);
    Body->SetPhysicsLinearVelocity(FVector::ZeroVector);
    Body->SetPhysicsAngularVelocityInDegrees(FVector::ZeroVector);
    Body->WakeAllRigidBodies();
    LocationOwner=ECargoOwner::Ship;
}

void APortContainerActor::ApplyContainerAppearance()
{
    auto* Model=LoadObject<UStaticMesh>(nullptr,TEXT("/Game/PortSim/Assets/Models/Container_Quaternius/Container_Quaternius/StaticMeshes/Container_Quaternius.Container_Quaternius"));
    if (!Model) return;
    Visual->SetStaticMesh(Model);
    const FBoxSphereBounds Bounds=Model->GetBounds();
    const FVector Size=Bounds.BoxExtent*2.f;
    // The source asset's long axis is X; rotate it onto the collision body's Y axis.
    const FVector Scale(ExternalDimensionsCm.Y/Size.X,ExternalDimensionsCm.X/Size.Y,ExternalDimensionsCm.Z/Size.Z);
    const FRotator Rotation(0,90,0);
    Visual->SetRelativeRotation(Rotation);
    Visual->SetRelativeScale3D(Scale);
    Visual->SetRelativeLocation(-Rotation.RotateVector(Bounds.Origin*Scale)/Body->GetComponentScale());
    Body->SetVisibility(false,false);
}

