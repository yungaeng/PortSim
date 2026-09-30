#include "QuayCrane.h"
#include "Camera/CameraComponent.h"
#include "GameFramework/SpringArmComponent.h"
#include "GameFramework/PlayerController.h"
#include "InputCoreTypes.h"
#include "PortSiteLogistics.h"
#include "PortSupportVehicle.h"
#include "ProceduralMeshComponent.h"
#include "TerminalLayout.h"
#include "PortWorkingCrane.h"
#include "PortAGVActor.h"

FString AQuayCrane::GetAGVStatus() const
{ return SiteLogistics?SiteLogistics->VehicleStatus():FString(); }

void AQuayCrane::FollowLoadedAGV()
{
    if (!SiteLogistics) return;
    FollowedAGV=SiteLogistics->LoadedVehicle(FollowedAGV.Get());
    if (!FollowedAGV.IsValid()) return;
    bFreeCamera=false;
    CameraArm->SetWorldLocation(FollowedAGV->GetActorLocation()+FVector(0,0,350));
    CameraArm->SetWorldRotation(FRotator(-28,-38,0));
    CameraArm->TargetArmLength=4200;
}

void AQuayCrane::MoveFreeCamera(FVector Input,FVector2D Look,float WallDt,bool Fast)
{
    bFollowAGV=false;
    if (!bFreeCamera)
    {
        const FVector Eye=Camera->GetComponentLocation();
        const FRotator View=Camera->GetComponentRotation();
        CameraArm->TargetArmLength=0;
        CameraArm->SetWorldLocationAndRotation(Eye,View);
        bFreeCamera=true;
    }
    FRotator View=CameraArm->GetComponentRotation();
    View.Yaw+=Look.X*.15f;
    View.Pitch=FMath::Clamp(View.Pitch+Look.Y*.15f,-89.f,89.f);
    View.Roll=0;
    CameraArm->SetWorldRotation(View);
    const FVector Direction=View.Vector()*Input.X+FRotationMatrix(View).GetUnitAxis(EAxis::Y)*Input.Y+FVector::UpVector*Input.Z;
    CameraArm->AddWorldOffset(Direction.GetClampedToMaxSize(1.f)*(Fast?30000.f:6000.f)*WallDt);
}

void AQuayCrane::TickFreeCamera(float WallDt)
{
    auto* PC=Cast<APlayerController>(GetController());
    if (!PC || !bUnifiedTerminal) return;
    const bool Looking=PC->IsInputKeyDown(EKeys::RightMouseButton);
    if (Looking!=bMouseLooking)
    {
        bMouseLooking=Looking; PC->bShowMouseCursor=!Looking;
        if (Looking) { FInputModeGameOnly Mode; PC->SetInputMode(Mode); }
        else { FInputModeGameAndUI Mode; Mode.SetHideCursorDuringCapture(false); PC->SetInputMode(Mode); }
    }
    auto Axis=[PC](FKey Plus,FKey Minus) { return float(PC->IsInputKeyDown(Plus))-float(PC->IsInputKeyDown(Minus)); };
    const FVector Input(Axis(EKeys::W,EKeys::S),Axis(EKeys::D,EKeys::A),Axis(EKeys::E,EKeys::Q));
    FVector2D Look=FVector2D::ZeroVector;
    if (Looking) PC->GetInputMouseDelta(Look.X,Look.Y);
    if (Looking || !Input.IsNearlyZero()) MoveFreeCamera(Input,Look,WallDt,PC->IsInputKeyDown(EKeys::LeftShift));
}

void AQuayCrane::TestEquipmentAndCamera()
{
    FString Error;
    bool Pass=ValidateTerminalActors(Error);
    int32 Counts[5]={};
    for (const auto& Actor:SupportFleet)
        if (const auto* Vehicle=Cast<APortSupportVehicle>(Actor)) ++Counts[static_cast<int32>(Vehicle->EquipmentType)];
    Pass &= Counts[0]==4 && Counts[1]==18 && Counts[2]==2 && Counts[3]==7 && Counts[4]==74;
    int32 CC=0,TC=0;
    for (const auto& Crane:WorkingCranes) { CC+=Crane->bSTS; TC+=!Crane->bSTS; }
    Pass &= CC==9 && TC==36 && SiteLogistics->Vehicles.Num()==60;
    const auto* SitePolygon=FindComponentByClass<UProceduralMeshComponent>();
    Pass &= SitePolygon && SitePolygon->GetNumSections()==1 &&
        TerminalLayout::PlanDepthM>519.f && TerminalLayout::PlanDepthM<521.f && TerminalLayout::ConceptSiteAreaM2()>400000.f;
    const FTransform Saved=CameraArm->GetComponentTransform();
    const float Length=CameraArm->TargetArmLength;
    MoveFreeCamera(FVector::ZeroVector,FVector2D::ZeroVector,0,false);
    const FVector Start=CameraArm->GetComponentLocation();
    MoveFreeCamera(FVector(1,0,0),FVector2D::ZeroVector,1,false);
    Pass &= FMath::IsNearlyEqual(FVector::Distance(Start,CameraArm->GetComponentLocation()),6000.f,1.f);
    const FVector Next=CameraArm->GetComponentLocation();
    MoveFreeCamera(FVector(0,0,1),FVector2D(40,10000),1,true);
    Pass &= CameraArm->GetComponentLocation().Equals(Next+FVector(0,0,30000),1.f);
    Pass &= FMath::IsNearlyEqual(CameraArm->GetComponentRotation().Pitch,89.f,.1f);
    CameraArm->SetWorldTransform(Saved); CameraArm->TargetArmLength=Length; bFreeCamera=false;
    UE_LOG(LogTemp,Display,TEXT("PORTSIM_EQUIPMENT_CAMERA_%s: polygon site %.0f m2; 210 equipment actors; 60 dispatch AGVs; free translation, boost, mouse rotation and pitch clamp; %s"),
        Pass?TEXT("PASS"):TEXT("FAIL"),TerminalLayout::ConceptSiteAreaM2(),*Error);
    FPlatformMisc::RequestExitWithStatus(false,Pass?0:1);
}
