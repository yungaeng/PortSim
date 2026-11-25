#include "UnchangedUpdates.h"
#include "SimulationPerformance.h"
#include "PortAGVActor.h"
#include "AGVReference.h"
#include "PortEnvironmentComponent.h"
#include "Components/StaticMeshComponent.h"

FAGVDynamicsInput APortAGVActor::DynamicsInput() const
{
    FAGVDynamicsInput I;
    I.EmptyMassKg=Friction.AGVEmptyMassKg; I.PayloadKg=PayloadKg;
    I.CargoCoGM=FVector(DeckCoGOffsetCm.Y,-DeckCoGOffsetCm.X,DeckCoGOffsetCm.Z)*.01;
    I.StaticMu=Friction.Tire.Static(Surface); I.KineticMu=Friction.Tire.Kinetic(Surface);
    I.RollingResistance=Friction.TireRollingResistance; I.DrivenFraction=Friction.DrivenWeightFraction;
    if(Emergency.Active)
    {
        FPortSurfaceState TestSurface=Surface;
        if(bEmergencyTestOverrides) { TestSurface.Wetness=ActiveEmergencySettings.Wetness; TestSurface.Ice=ActiveEmergencySettings.Ice; }
        I.StaticMu=Friction.Tire.Static(TestSurface); I.KineticMu=Friction.Tire.Kinetic(TestSurface);
        I.DeckStaticMu=Friction.Steel.Static(TestSurface); I.DeckKineticMu=Friction.Steel.Kinetic(TestSurface);
        I.CargoContactEnabled=true; I.CargoSecured=Emergency.CargoSecured;
        // DeckCoG is rebuilt from actual attached, displaced cargo each frame.
        // The solver adds its relative offset once, never twice.
        I.CargoCoGM-=Dynamics.CargoOffsetM;
    }
    I.AccelerationLimitMps2=AccelerationCmPerSecondSquared()*.01;
    I.BrakeDemandMps2=BrakingCmPerSecondSquared()*.01;
    const FVector Velocity=GetActorRightVector()*Dynamics.U*100-GetActorForwardVector()*Dynamics.V*100;
    const FVector Force=FPortWindAerodynamics::DragForceNewtons(WindVelocity,Velocity,GetActorQuat(),
        FVector(4.2,14.6,PayloadKg>0?4.3:2.),1.2);
    I.WindForceBodyN=FVector(FVector::DotProduct(Force,GetActorRightVector()),
        -FVector::DotProduct(Force,GetActorForwardVector()),0);
    return I;
}

void APortAGVActor::ConstrainDynamicsStop()
{
    if(Emergency.Active && !Emergency.Finished)Emergency.Invalidate();
    Speed=0; Dynamics.ConstrainStop(); Sensors.LateralSlipCm=0;
}

void APortAGVActor::AdvanceDynamics(double TargetSpeedMps,double CurvaturePerM,double Seconds)
{
    PortSimPerformance::FScope Timing(TEXT("agv_dynamics"));
    if(Seconds<=0) return;
    DynamicsSettings.Sanitize();
    // Compatibility for initial speed set by scene fixtures or recovery code.
    if(FMath::Abs(Speed-FMath::Sqrt(Dynamics.U*Dynamics.U+Dynamics.V*Dynamics.V)*100)>.1)
        Dynamics.Reset((Sensors.bReversing?-1:1)*Speed*.01,DynamicsSettings);
    const FVector Previous=GetActorLocation(),Forward=GetActorRightVector(),Left=-GetActorForwardVector();
    Dynamics.PositionM=FVector::ZeroVector; Dynamics.Heading=0;
    const double WheelBefore=Dynamics.WheelDistanceM,SlipBefore=Dynamics.SideSlipDistanceM;
    auto Input=DynamicsInput(); Input.TargetSpeedMps=TargetSpeedMps; Input.CurvaturePerM=CurvaturePerM;
    if(Emergency.Active)
    {
        const FVector Delta=(EmergencyWorkerPosition-Previous)*.01;
        const double Along=FVector::DotProduct(Delta,EmergencyDirection);
        const double Side=FVector::CrossProduct(EmergencyDirection,Delta).Z;
        const double Gap=Along-AGVReference::PhysicalHalfLengthCm*.01-ActiveEmergencySettings.WorkerRadiusM;
        const double Range=Sensors.LiDARDetectionRangeCm>=0?Sensors.LiDARDetectionRangeCm*.01:100;
        Emergency.Step(ActiveEmergencySettings,DynamicsSettings,Input,Dynamics,Gap,Side,Range,Seconds);
    }
    else Dynamics.Step(DynamicsSettings,Input,Seconds);
    SetActorLocationAndRotation(Previous+(Forward*Dynamics.PositionM.X+Left*Dynamics.PositionM.Y)*100,
        GetActorRotation()+FRotator(0,FMath::RadiansToDegrees(Dynamics.Heading),0));
    Speed=FMath::Sqrt(Dynamics.U*Dynamics.U+Dynamics.V*Dynamics.V)*100;
    LastCurvaturePerCm=FMath::Abs(Dynamics.U)>.01?Dynamics.YawRate/FMath::Abs(Dynamics.U)*.01:0;
    Sensors.WheelOdometryCm+=FMath::Abs(Dynamics.WheelDistanceM-WheelBefore)*100;
    Sensors.LateralSlipCm=(Dynamics.SideSlipDistanceM-SlipBefore)*100;
    Sensors.SteeringAngleDegrees=FMath::RadiansToDegrees((Dynamics.Wheels[2].Steer+Dynamics.Wheels[3].Steer)*.5);
    UpdateOdometry(Previous); UpdateWheelVisuals();
}

void APortAGVActor::UpdateWheelVisuals()
{
    // Construction order is side(-/+) then axle(rear/front), tire then hub.
    for(int Side=0;Side<2;++Side) for(int Axle=0;Axle<2;++Axle)
    {
        const auto& W=Dynamics.Wheels[Axle*2+Side];
        const double Steer=FMath::RadiansToDegrees(W.Steer),Spin=FMath::RadiansToDegrees(W.Angle);
        const FQuat Q=FQuat(FVector::UpVector,Steer*PI/180.)*
            FQuat(FVector::ForwardVector,Spin*PI/180.)*FRotator(90,0,0).Quaternion();
        for(int Part=0;Part<2;++Part) if(WheelMeshes.IsValidIndex((Side*2+Axle)*2+Part))
        {
            auto* Mesh=WheelMeshes[(Side*2+Axle)*2+Part].Get();
            if(!PortSimUpdates::SkipUnchanged() || !Mesh->GetRelativeTransform().GetRotation().Equals(Q,0.))
                Mesh->SetRelativeRotation(Q);
        }
    }
}
