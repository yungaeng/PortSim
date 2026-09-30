#include "PortAGVActor.h"
#include "TerminalLayout.h"
#include "Components/StaticMeshComponent.h"
#include "Components/TextRenderComponent.h"
#include "Engine/StaticMesh.h"
#include "UObject/ConstructorHelpers.h"

APortAGVActor::APortAGVActor()
{
    Box(TEXT("AGV_Deck"),RootComponent,FVector(0,0,170),FVector(300,1380,100),true);
    for (int32 Side:{-1,1}) for (int32 Axle:{-1,0,1})
        Box(*FString::Printf(TEXT("Bogie_AGV_%d_%d"),Side,Axle),RootComponent,FVector(Side*170,Axle*480,75),FVector(75,160,110));
    Box(TEXT("AGV_Sensor"),RootComponent,FVector(0,670,265),FVector(110,45,90));
    NameLabel=Label(TEXT("AGVLabel"),FVector(0,0,230));
    NameLabel->SetText(FText::FromString(TEXT("AGV")));
    Tags.Add(TEXT("PortSim.AGV"));
}

void APortAGVActor::InitializeVehicle(int32 Number)
{
    VehicleID=Number;
    NameLabel->SetText(FText::FromString(FString::Printf(TEXT("AGV %d"),Number)));
#if WITH_EDITOR
    SetActorLabel(FString::Printf(TEXT("AGV_%02d"),Number));
    SetFolderPath(TEXT("PortSim/Equipment"));
#endif
}

void APortAGVActor::ResetVehicle(FVector Position)
{
    Speed=0; CompletedJobs=0;
    SetActorLocationAndRotation(Position,FRotator::ZeroRotator,false,nullptr,ETeleportType::TeleportPhysics);
}

FVector APortAGVActor::CargoPosition() const
{
    return GetActorLocation()+FVector(0,0,349.5f);
}

bool APortAGVActor::MoveToX(float X,float Dt)
{
    FVector P=GetActorLocation();
    const float Distance=FMath::Abs(X-P.X);
    const float Desired=FMath::Min(450.f,FMath::Sqrt(2.f*180.f*Distance));
    Speed=FMath::FInterpConstantTo(Speed,Desired,Dt,180.f);
    P.X+=FMath::Sign(X-P.X)*FMath::Min(Distance,Speed*Dt);
    SetActorLocation(P);
    if (FMath::Abs(X-P.X)<.1f) { Speed=0; return true; }
    return false;
}


bool APortAGVActor::MoveToPosition(FVector Target,float Dt,bool StopAtTarget)
{
    const float Distance=FVector::Dist(GetActorLocation(),Target);
    const float Desired=StopAtTarget?FMath::Min(450.f,FMath::Sqrt(2.f*180.f*Distance)):450.f;
    Speed=FMath::FInterpConstantTo(Speed,Desired,Dt,180.f);
    SetActorLocation(FMath::VInterpConstantTo(GetActorLocation(),Target,Dt,Speed));
    if (FVector::Dist(GetActorLocation(),Target)>.1f) return false;
    SetActorLocation(Target); if (StopAtTarget) Speed=0; return true;
}
