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
#include "EngineUtils.h"

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
        if (const auto* Vehicle=Cast<APortSupportVehicle>(Actor))
        {
            ++Counts[static_cast<int32>(Vehicle->EquipmentType)];
            const FVector P=Vehicle->GetActorLocation()/100.f;
            Pass &= P.X>=TerminalLayout::SiteX(685) && P.X<=TerminalLayout::SiteX(825)
                && P.Y>=0 && P.Y<=185;
        }
    Pass &= Counts[0]==4 && Counts[1]==18 && Counts[2]==2 && Counts[3]==7 && Counts[4]==74;
    int32 CC=0,TC=0;
    for (const auto& Crane:WorkingCranes) { CC+=Crane->bSTS; TC+=!Crane->bSTS; }
    Pass &= CC==9 && TC==2*TerminalLayout::YardBlockCount && SiteLogistics->Vehicles.Num()==60;
    int32 WorldAGVCount=0;
    bool ParkingPass=true;
    double MaxAGVY=-DBL_MAX;
    TArray<FBox> AGVBounds;
    for(TActorIterator<APortAGVActor> It(GetWorld());It;++It)
    {
        ++WorldAGVCount;
        FVector Center,Extent;
        It->GetActorBounds(false,Center,Extent);
        MaxAGVY=FMath::Max(MaxAGVY,Center.Y+Extent.Y);
        ParkingPass &= Center.Y+Extent.Y<23800 && Center.Y-Extent.Y>-52500;
        const FBox Bounds(Center-Extent,Center+Extent);
        for(const FBox& Other:AGVBounds)
            ParkingPass &= Bounds.Max.X<Other.Min.X || Bounds.Min.X>Other.Max.X
                || Bounds.Max.Y<Other.Min.Y || Bounds.Min.Y>Other.Max.Y;
        AGVBounds.Add(Bounds);
    }
    ParkingPass &= WorldAGVCount==60;
    Pass &= ParkingPass;
    UE_LOG(LogTemp,Display,TEXT("AGV_PARKING_%s: actual actors=%d; eastmost vehicle edge=%.2f m; worker-rest apron starts at 238 m; no parked overlap"),
        ParkingPass?TEXT("PASS"):TEXT("FAIL"),WorldAGVCount,MaxAGVY/100.);
    auto* SitePolygon=FindComponentByClass<UProceduralMeshComponent>();
    Pass &= SitePolygon && SitePolygon->GetNumSections()==1 &&
        FMath::Abs(TerminalLayout::ConceptSiteAreaM2()-TerminalLayout::SiteAreaM2)<1.0;
    // Check the generated mesh itself, not only the layout constants.
    double MeshAreaM2=0;
    if(SitePolygon && SitePolygon->GetNumSections()==1)
    {
        const auto* Section=SitePolygon->GetProcMeshSection(0);
        for(int32 I=0;I+2<Section->ProcIndexBuffer.Num();I+=3)
        {
            const FVector A=Section->ProcVertexBuffer[Section->ProcIndexBuffer[I]].Position;
            const FVector B=Section->ProcVertexBuffer[Section->ProcIndexBuffer[I+1]].Position;
            const FVector C=Section->ProcVertexBuffer[Section->ProcIndexBuffer[I+2]].Position;
            MeshAreaM2+=FMath::Abs(FVector::CrossProduct(B-A,C-A).Z)*.5/10000.;
        }
    }
    Pass &= FMath::Abs(MeshAreaM2-TerminalLayout::SiteAreaM2)<1.0;
    UE_LOG(LogTemp,Display,TEXT("SITE_AREA_CHECK: mesh=%.3f target=%.3f quay_m=%.1f max_depth_m=%.3f"),
        MeshAreaM2,TerminalLayout::SiteAreaM2,TerminalLayout::QuayLength/100.,TerminalLayout::PlanDepthM);
    bool PhysicalScalePass=true, HandoverPass=true;
    for(const auto& Crane:WorkingCranes)
        PhysicalScalePass &= Crane->GetActorScale3D().Equals(FVector::OneVector,.0001f);
    for(const auto& Vehicle:SiteLogistics->Vehicles)
        PhysicalScalePass &= Vehicle->GetActorScale3D().Equals(FVector::OneVector,.0001f);
    for(const auto& Actor:SupportFleet)
        PhysicalScalePass &= Actor->GetActorScale3D().Equals(FVector::OneVector,.0001f);
    // Site STSs retain their original physical profile and world X=12 m.
    HandoverPass &= STSProfile.ContainsTarget(FVector(TerminalLayout::SiteCmX(4500)-1200.f,0,504.f));
    Pass &= PhysicalScalePass && HandoverPass;
    UE_LOG(LogTemp,Display,TEXT("SITE_PHYSICAL_CHECK: actor_scales=%s sts_handover=%s"),
        PhysicalScalePass?TEXT("PASS"):TEXT("FAIL"),HandoverPass?TEXT("PASS"):TEXT("FAIL"));
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
    UE_LOG(LogTemp,Display,TEXT("PORTSIM_EQUIPMENT_CAMERA_%s: polygon site %.0f m2; 220 equipment actors; 60 dispatch AGVs; free translation, boost, mouse rotation and pitch clamp; %s"),
        Pass?TEXT("PASS"):TEXT("FAIL"),TerminalLayout::ConceptSiteAreaM2(),*Error);
    FPlatformMisc::RequestExitWithStatus(false,Pass?0:1);
}
