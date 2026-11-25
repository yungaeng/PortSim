#include "PortSupportVehicle.h"
#include "PortContainerActor.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/World.h"

// Equivalent six-contact tire model and three hydraulic head actuators.
// SI units; parameters are simulation assumptions, not an OEM load chart.
bool APortSupportVehicle::AdvanceRecoveryDrive(FVector Target,float Dt,bool Clear,bool Final,
    const FPortSurfaceState& Surface,const FPortFrictionSettings& Friction)
{
    if(bFreePhysics || bPhysicsPaused || Dt<=0 || Dt>1.f/60.f+1.e-6f) return false;
    TireSettings.AxleCount=3; // Preserve this vehicle's existing six contacts.
    TireSettings.Sanitize();
    const FVector P=GetActorLocation(),Forward=GetActorForwardVector(),Left=GetActorRightVector();
    const FVector Delta=Target-P;const double Distance=Delta.Size2D()*.01;
    const double Angle=FMath::Atan2(FVector::DotProduct(Delta,Left),FVector::DotProduct(Delta,Forward));
    FAGVDynamicsInput Input;Input.EmptyMassKg=FMath::Max(10000.f,EmptyMassKg);Input.PayloadKg=PayloadKg;Input.CargoCoGM=CargoCoGM;
    Input.StaticMu=Friction.Tire.Static(Surface);Input.KineticMu=Friction.Tire.Kinetic(Surface);
    Input.RollingResistance=Friction.TireRollingResistance;Input.DrivenFraction=.5;
    Input.AccelerationLimitMps2=.4;Input.BrakeDemandMps2=1.;
    Input.CurvaturePerM=FMath::Clamp(2*FMath::Sin(Angle)/FMath::Max(3.,Distance),-1./5.8,1./5.8);
    Input.TargetSpeedMps=Clear?FMath::Min(2.,FMath::Sqrt(2.*FMath::Max(0.,Distance-(Final?.03:.4)))):0.;
    Input.TargetSpeedMps*=FMath::Max(.1,FMath::Cos(Angle));
    if(Distance<(Final?.06:.8)) Input.TargetSpeedMps=0;
    TireDynamics.PositionM=FVector::ZeroVector;TireDynamics.Heading=0;
    TireDynamics.Step(TireSettings,Input,Dt);
    if(!TireDynamics.Finite() || !TireDynamics.Fault.IsEmpty())
    {PhysicsFault=TEXT("R/S tire dynamics invalid");TireDynamics.ConstrainStop();return false;}
    // Collision admission remains a hard constraint, separate from measured
    // tire braking. Never advance through a blocked swept chassis envelope.
    if(!Clear) TireDynamics.ConstrainStop();
    SetActorLocationAndRotation(P+(Forward*TireDynamics.PositionM.X+Left*TireDynamics.PositionM.Y)*(Clear?100.:0.),
        FRotator(FMath::RadiansToDegrees(-TireDynamics.Pitch),GetActorRotation().Yaw+
            FMath::RadiansToDegrees(Clear?TireDynamics.Heading:0.),FMath::RadiansToDegrees(TireDynamics.Roll)));
    TInlineComponentArray<UStaticMeshComponent*> Parts(this);
    for(auto* Part:Parts) for(int32 Side=0;Side<2;++Side) for(int32 Axle=0;Axle<3;++Axle)
        if(Part->GetName()==FString::Printf(TEXT("Wheel_%d_%d"),Side?1:-1,Axle-1))
        {
            const auto& W=TireDynamics.Wheels[Axle*2+Side];
            Part->SetRelativeRotation(FRotator(FMath::RadiansToDegrees(W.Angle),FMath::RadiansToDegrees(W.Steer),0));
        }
    if(TireDynamics.OverturnThreshold) {EnterFreePhysics();return false;}
    return Distance<(Final?.06:.8) && (!Final || FMath::Abs(TireDynamics.U)<.03);
}

FVector APortSupportVehicle::RecoveryHeadPosition() const
{ return RecoveryGrip?RecoveryGrip->GetComponentLocation():GetActorTransform().TransformPosition((HydraulicPositionM+FVector(0,SideShiftM,0))*100.); }

bool APortSupportVehicle::LockRecoveryCargo(APortContainerActor* Cargo)
{
    if(!Cargo || !RecoveryGrip || GrippedCargo.IsValid() || bFreePhysics || Cargo->MassKg>RatedCapacityKg ||
        (Cargo->LengthFt!=20 && Cargo->LengthFt!=40) || Cargo->GetBody()->IsSimulatingPhysics() ||
        FMath::Abs(TireDynamics.U)>.03 || HydraulicVelocityMps.Size()>.02) return false;
    const FVector Top=Cargo->GetActorLocation()+Cargo->GetActorUpVector()*Cargo->ExternalDimensionsCm.Z*.5;
    if(!RecoveryHeadPosition().Equals(Top,1.) || RecoveryGrip->GetComponentQuat().AngularDistance(Cargo->GetActorQuat())>FMath::DegreesToRadians(.2)) return false;
    if(!Cargo->AttachToComponent(RecoveryGrip,FAttachmentTransformRules::KeepWorldTransform)) return false;
    GrippedCargo=Cargo;return true;
}

bool APortSupportVehicle::ReleaseRecoveryCargo(APortContainerActor* Cargo)
{
    if(!Cargo || GrippedCargo.Get()!=Cargo || bFreePhysics || HydraulicVelocityMps.Size()>.02) return false;
    Cargo->DetachFromActor(FDetachmentTransformRules::KeepWorldTransform);GrippedCargo.Reset();return true;
}

bool APortSupportVehicle::AdvanceRecoveryHydraulics(FVector Target,FRotator Rotation,float LengthCm,float LoadKg,float Dt)
{
    if(bFreePhysics || bPhysicsPaused || Dt<=0 || Dt>1.f/60.f+1.e-6f) return false;
    FVector Goal=GetActorTransform().InverseTransformPosition(Target)*.01;
    bHydraulicLimited=Goal.Size2D()>13. || Goal.Z<2.5 || Goal.Z>12.;
    if(bHydraulicLimited) return false;
    const double ShiftLimit=FMath::Clamp(double(MaximumSideShiftM),0.,.8);
    const double ShiftGoal=FMath::Clamp(Goal.Y,-ShiftLimit,ShiftLimit);
    SideShiftM=FMath::FInterpConstantTo(SideShiftM,ShiftGoal,Dt,.1f);
    Goal.Y-=SideShiftM;
    const double Area=FMath::Clamp(double(CylinderAreaM2),.005,.1);
    const double Relief=FMath::Clamp(double(ReliefPressureMPa),1.,40.)*1.e6;
    const double MaxFlow=FMath::Clamp(double(PumpLitresPerMinute),1.,1500.)/60000.;
    const double Mass=FMath::Max(2000.,double(LoadKg)+2000.);
    constexpr double Bulk=2.e8,Volume=.08,Leak=1.e-12,Damping=40000.;
    HydraulicFlowM3ps=0;
    // Four bounded numerical subdivisions inside ONE 1/60 control step.
    for(int32 Sub=0;Sub<4;++Sub)
    {
        const double H=Dt/4.;
        FVector Demand=FVector::ZeroVector;
        for(int32 Axis=0;Axis<3;++Axis)
        {
            const double Force=Axis==2?Mass*9.80665:0.;
            const double Velocity=FMath::Clamp((Goal[Axis]-HydraulicPositionM[Axis])*1.5,-.4,.4);
            const double DesiredPressure=FMath::Clamp((Force+Damping*Velocity+Mass*(Velocity-HydraulicVelocityMps[Axis])*8)/Area,-Relief,Relief);
            Demand[Axis]=Area*HydraulicVelocityMps[Axis]+Leak*HydraulicPressurePa[Axis]+
                (DesiredPressure-HydraulicPressurePa[Axis])*Volume/(Bulk*.05);
        }
        const double Total=FMath::Abs(Demand.X)+FMath::Abs(Demand.Y)+FMath::Abs(Demand.Z);
        if(Total>MaxFlow) Demand*=MaxFlow/Total;
        HydraulicFlowM3ps=Demand.GetAbs().X+Demand.GetAbs().Y+Demand.GetAbs().Z;
        for(int32 Axis=0;Axis<3;++Axis)
        {
            const double Load=Axis==2?Mass*9.80665:0.;
            const double V=HydraulicVelocityMps[Axis];
            const double Pressure=(HydraulicPressurePa[Axis]+H*Bulk/Volume*(Demand[Axis]-Area*V))/(1+H*Bulk/Volume*Leak);
            bHydraulicLimited|=FMath::Abs(Pressure)>=Relief;
            HydraulicPressurePa[Axis]=FMath::Clamp(Pressure,-Relief,Relief);
            // Pilot-operated check valves hold supported loads when the pump
            // cannot overcome gravity. They do not increase available pressure.
            if(Axis==2 && HydraulicPressurePa[Axis]*Area<Load && V<=0 && Goal.Z>=HydraulicPositionM.Z)
                HydraulicVelocityMps[Axis]=0;
            else HydraulicVelocityMps[Axis]=(V+H*(HydraulicPressurePa[Axis]*Area-Load)/Mass)/(1+H*Damping/Mass);
            HydraulicPositionM[Axis]+=HydraulicVelocityMps[Axis]*H;
        }
    }
    GripRotation=FMath::QInterpConstantTo(GripRotation,Rotation.Quaternion(),Dt,.2f);
    const FVector Head=GetActorTransform().TransformPosition((HydraulicPositionM+FVector(0,SideShiftM,0))*100.);
    PositionRecoverySpreader(Head,GripRotation.Rotator(),LengthCm);
    return FVector::Dist(Head,Target)<.5 && HydraulicVelocityMps.Size()<.02 &&
        GripRotation.AngularDistance(Rotation.Quaternion())<FMath::DegreesToRadians(.1);
}

void APortSupportVehicle::RefreshRecoveryLoad(float LoadKg,FVector WorldCoG,float Dt)
{
    PayloadKg=FMath::Max(0.f,LoadKg);
    CargoCoGM=GetActorTransform().InverseTransformPosition(WorldCoG)*.01;
    if(bFreePhysics || bPhysicsPaused) return;
    TireSettings.Sanitize();
    FAGVDynamicsInput Input;Input.EmptyMassKg=FMath::Max(10000.f,EmptyMassKg);Input.PayloadKg=PayloadKg;Input.CargoCoGM=CargoCoGM;
    TireDynamics.RefreshLoads(TireSettings,Input);
    if(TireDynamics.OverturnThreshold) EnterFreePhysics();
}

bool APortSupportVehicle::OwnsRecoveryPhysicsBody(const UPrimitiveComponent* Part) const
{return bFreePhysics && Part && Part->GetOwner()==this && Part->IsA<UStaticMeshComponent>();}

void APortSupportVehicle::EnterFreePhysics()
{
    if(bFreePhysics || !RecoveryBody || !RecoverySceneRoot) return;
    PhysicsFault=TEXT("R/S 전도 · 자유 강체 물리 · 초기화 필요");
    const FVector Velocity=GetActorForwardVector()*TireDynamics.U*100+GetActorRightVector()*TireDynamics.V*100;
    TArray<AActor*> Loads;GetAttachedActors(Loads);
    RecoveryBody->DetachFromComponent(FDetachmentTransformRules::KeepWorldTransform);
    SetRootComponent(RecoveryBody);
    if(RecoveryGrip) RecoveryGrip->AttachToComponent(RecoveryBody,FAttachmentTransformRules::KeepWorldTransform);
    TInlineComponentArray<UStaticMeshComponent*> Parts(this);
    for(auto* Part:Parts) if(Part!=RecoveryBody)
        Part->AttachToComponent(RecoveryBody,FAttachmentTransformRules(EAttachmentRule::KeepWorld,true));
    for(auto* Load:Loads) Load->AttachToComponent(RecoveryGrip?RecoveryGrip.Get():RecoveryBody.Get(),FAttachmentTransformRules::KeepWorldTransform);
    RecoveryBody->SetMassOverrideInKg(NAME_None,FMath::Max(10000.f,EmptyMassKg)+PayloadKg,true);
    RecoveryBody->SetCenterOfMass(TireDynamics.CombinedCoGM*100-RecoveryBodyRelative.GetLocation());
    RecoveryBody->SetSimulatePhysics(true);
    RecoveryBody->SetPhysicsLinearVelocity(Velocity);
    RecoveryBody->SetPhysicsAngularVelocityInRadians(FVector(TireDynamics.RollRate,TireDynamics.PitchRate,TireDynamics.YawRate));
    bFreePhysics=true;
}

void APortSupportVehicle::SetRecoveryPhysicsPaused(bool Paused)
{
    if(!bFreePhysics || bPhysicsPaused==Paused || !RecoveryBody) return;
    bPhysicsPaused=Paused;
    if(Paused)
    {
        PausedLinear=RecoveryBody->GetPhysicsLinearVelocity();PausedAngular=RecoveryBody->GetPhysicsAngularVelocityInRadians();
        RecoveryBody->SetSimulatePhysics(false);
    }
    else
    {
        RecoveryBody->SetSimulatePhysics(true);RecoveryBody->SetPhysicsLinearVelocity(PausedLinear);
        RecoveryBody->SetPhysicsAngularVelocityInRadians(PausedAngular);
    }
}

void APortSupportVehicle::ResetRecoveryPhysics()
{
    if(bFreePhysics && RecoveryBody && RecoverySceneRoot)
    {
        RecoveryBody->SetSimulatePhysics(false);
        const FTransform Pose=GetActorTransform();SetRootComponent(RecoverySceneRoot);SetActorTransform(Pose);
        TInlineComponentArray<UStaticMeshComponent*> Parts(this);
        for(auto* Part:Parts) if(Part!=RecoveryBody)
        {Part->UnWeldFromParent();Part->AttachToComponent(RecoverySceneRoot,FAttachmentTransformRules::KeepWorldTransform);}
        RecoveryBody->AttachToComponent(RecoverySceneRoot,FAttachmentTransformRules::KeepWorldTransform);
        RecoveryBody->SetRelativeTransform(RecoveryBodyRelative);
    }
    bFreePhysics=bPhysicsPaused=false;PhysicsFault.Empty();PayloadKg=0;
    GrippedCargo.Reset();SideShiftM=0;GripRotation=FQuat::Identity;
    if(RecoveryGrip) RecoveryGrip->AttachToComponent(RecoverySceneRoot,FAttachmentTransformRules::KeepWorldTransform);
    HydraulicPositionM=FVector(5.4,0,6.5);HydraulicVelocityMps=HydraulicPressurePa=FVector::ZeroVector;
    TireDynamics.Reset(0,TireSettings);
    PositionRecoverySpreader(GetActorTransform().TransformPosition(HydraulicPositionM*100),GetActorRotation(),1219.2f);
}
