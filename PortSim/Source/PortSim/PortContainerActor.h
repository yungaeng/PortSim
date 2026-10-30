#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "ContainerSpecification.h"
#include "SpreaderTelescope.h"
#include "CargoInformation.h"
#include "PortContainerActor.generated.h"

class UStaticMeshComponent;
class UTextRenderComponent;

UENUM(BlueprintType)
enum class ECargoOwner : uint8 { Ship, STS, AGV, RMG, Yard };

UCLASS(Blueprintable)
class PORTSIM_API APortContainerActor : public AActor
{
    GENERATED_BODY()
    friend class FTerminalReplayCheckpointTest;
    friend class FTerminalCheckpoint;
    friend class FTerminalReplay;
public:
    APortContainerActor();
    virtual void BeginPlay() override;
    void InitializeContainer(int32 Number);
    void ConfigureSpecification(const FPortContainerSpecification& Specification, FVector CoGOffset=FVector::ZeroVector);
    void ResetCargo(FVector Position);
    void ApplyContainerAppearance();
    void SetPhysicalParameters(float Mass, FVector CoGOffset);
    void ApplyWind(FVector WindVelocityMetersPerSecond,float DragCoefficient=1.2f);
    void SetSecuredVesselMotion(FVector Position,FQuat Rotation,FVector VelocityCentimetersPerSecond,
        FVector AccelerationCentimetersPerSecondSquared=FVector::ZeroVector,
        FVector JerkCentimetersPerSecondCubed=FVector::ZeroVector,double FrameDurationSeconds=0.,bool ApplyPose=true);
    /** Interpolate the latest vessel-motion frame for fixed-rate crane substeps. */
    void ApplySecuredMotionFraction(double Fraction);
    void SetSecuredMotionWindow(double Start,double End);
    void BeginVisualFrame();
    void EndVisualFrame();
    bool SecuredMotionCouldIntersect(const FBox& Query) const { return SecuredSweptBounds.Intersect(Query); }
    FVector GetMotionVelocity() const;
    FVector GetMotionAcceleration() const;
    FVector GetMotionJerk() const;
    UStaticMeshComponent* GetBody() const { return Body; }

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Container") FName ContainerID;
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Booking") FPortCargoInformation CargoInformation;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Container") EPortContainerSize SizeClass=EPortContainerSize::Ft40;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Container") int32 LengthFt=40;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Container") FVector ExternalDimensionsCm=FVector(243.8f,1219.2f,259.1f);
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Container") float TareMassKg=3800.f;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Container") float CargoMassKg=8200.f;
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Container", meta=(ClampMin="1")) float MassKg=12000.f;
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Container") FVector CoGOffsetCm=FVector::ZeroVector;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Container") ECargoOwner LocationOwner=ECargoOwner::Ship;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Container") TObjectPtr<UStaticMeshComponent> Body;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Container") TObjectPtr<UStaticMeshComponent> Visual;
    float SensorHalfLengthCm() const { return PortSpreaderTelescope::TwistlockHalfLengthCm(ExternalDimensionsCm.Y); }
    float SensorHalfWidthCm() const { return PortSpreaderTelescope::TwistlockHalfWidthCm(ExternalDimensionsCm.X); }
    FVector KinematicVelocityCentimetersPerSecond=FVector::ZeroVector;
    FVector KinematicAccelerationCentimetersPerSecondSquared=FVector::ZeroVector;
    FVector KinematicJerkCentimetersPerSecondCubed=FVector::ZeroVector;
private:
    TWeakObjectPtr<USceneComponent> DeferredVisualParent;
    FTransform DeferredVisualRelative;
    FBox SecuredSweptBounds=FBox(ForceInit);
    FTransform PreviousSecuredTransform=FTransform::Identity, CurrentSecuredTransform=FTransform::Identity;
    FVector PreviousSecuredVelocity=FVector::ZeroVector, CurrentSecuredVelocity=FVector::ZeroVector;
    FVector PreviousSecuredAcceleration=FVector::ZeroVector, CurrentSecuredAcceleration=FVector::ZeroVector;
    FVector PreviousSecuredJerk=FVector::ZeroVector, CurrentSecuredJerk=FVector::ZeroVector;
    double SecuredFrameDurationSeconds=0;
    double SecuredWindowStart=0,SecuredWindowEnd=1;
    bool bHasSecuredMotionFrame=false;
};
