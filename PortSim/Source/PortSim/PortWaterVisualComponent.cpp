#include "PortWaterVisualComponent.h"
#include "PortEnvironmentComponent.h"
#include "ProceduralMeshComponent.h"
#include "Camera/PlayerCameraManager.h"
#include "Engine/GameViewportClient.h"
#include "Engine/World.h"
#include "GameFramework/PlayerController.h"
#include "HAL/IConsoleManager.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Misc/App.h"

namespace
{
    TAutoConsoleVariable<int32> WaterWaves(TEXT("PortSim.WaterWaves"),1,
        TEXT("GPU water displacement and ripples (0=flat, 1=on). Does not change vessel motion."));
    constexpr double SurfaceZ=-250.;
}

void UPortWaterVisualComponent::BeginPlay()
{
    Super::BeginPlay();
    if(!FApp::CanEverRender() || GetWorld()->GetNetMode()==NM_DedicatedServer) return;
    Environment=GetOwner()->FindComponentByClass<UPortEnvironmentComponent>();
    auto* GameViewport=GetWorld()->GetGameViewport();
    if(!Environment || !GameViewport) return;
    BuildSurface();
    Viewport=GameViewport;
    GameViewport->OnBeginDraw().AddUObject(this,&UPortWaterVisualComponent::UpdateForDraw);
    UpdateForDraw();
}

void UPortWaterVisualComponent::BuildSurface()
{
    auto* Parent=LoadObject<UMaterialInterface>(nullptr,
        TEXT("/Game/PortSim/Assets/Materials/M_PortGerstnerWater.M_PortGerstnerWater"));
    if(!Parent)
    {
        UE_LOG(LogTemp,Error,TEXT("Water material missing: run Scripts/create_water_material.py in the editor."));
        return;
    }
    Surface=NewObject<UPortWaterSurfaceComponent>(GetOwner(),TEXT("PortWaterSurface"));
    GetOwner()->AddInstanceComponent(Surface);
    Surface->SetMobility(EComponentMobility::Movable);
    Surface->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    Surface->SetCastShadow(false);
    Surface->SetVisibleInRayTracing(false);
    Surface->SetReceivesDecals(false);
    Surface->RegisterComponent();
    Material=UMaterialInstanceDynamic::Create(Parent,this);
    // Entire-surface LODs avoid cracks between independently displaced tiles.
    // Meshes are built once. Exactly one section is visible, with no vertex uploads per frame.
    for(int32 LOD=0;LOD<3;++LOD)
    {
        const int32 NX=128>>LOD,NY=320>>LOD;
        TArray<FVector> Vertices,Normals;
        TArray<FVector2D> UV;
        TArray<int32> Triangles;
        Vertices.Reserve((NX+1)*(NY+1));
        for(int32 Y=0;Y<=NY;++Y) for(int32 X=0;X<=NX;++X)
        {
            Vertices.Add(FVector(-60700.+60000.*X/NX,-72500.+145000.*Y/NY,SurfaceZ));
            Normals.Add(FVector::UpVector);
            UV.Add(FVector2D(double(X)/NX,double(Y)/NY));
        }
        for(int32 Y=0;Y<NY;++Y) for(int32 X=0;X<NX;++X)
        {
            const int32 A=Y*(NX+1)+X,B=A+1,C=A+NX+1,D=C+1;
            Triangles.Append({A,C,B,B,C,D});
        }
        Surface->CreateMeshSection(LOD,Vertices,Triangles,Normals,UV,TArray<FColor>(),TArray<FProcMeshTangent>(),false);
        Surface->SetMaterial(LOD,Material);
        Surface->SetMeshSectionVisible(LOD,false);
    }
    UE_LOG(LogTemp,Display,TEXT("PORTSIM_WATER_READY: 4 GPU waves; LOD vertices=41409/10465/2673; collision=off"));
}

void UPortWaterVisualComponent::UpdateForDraw()
{
    if(!Material || !Surface) return;
    const auto& Sea=Environment->GetSeaState();
    const float Enabled=Sea.bSeaMotionEnabled && WaterWaves.GetValueOnGameThread()!=0?1.f:0.f;
    Material->SetScalarParameterValue(TEXT("WaveEnabled"),Enabled);
    FLinearColor Phases(0,0,0,0);
    for(int32 I=0;I<4;++I)
    {
        const bool Swell=I>=2;
        const bool Secondary=(I%2)!=0;
        const double Period=(Swell?Sea.SwellPeriodSeconds:Sea.WavePeriodSeconds)*(Secondary?.73:1.);
        const double Omega=2.*PI/FMath::Max(.1,Period);
        const double WaveNumber=Omega*Omega/9.81/100.;
        const double Angle=FMath::DegreesToRadians((Swell?Sea.SwellDirectionDegrees:Sea.WaveDirectionDegrees)+(Secondary?27.:0.));
        const FVector Direction=FPortWindState::FlowDirection(FMath::RadiansToDegrees(Angle));
        const double Amplitude=(Swell?Sea.SwellHeightMeters:Sea.WaveHeightMeters)*50.*(Secondary?.3:.7);
        Material->SetVectorParameterValue(FName(*FString::Printf(TEXT("Wave%d"),I)),
            FLinearColor(Direction.X,Direction.Y,Amplitude,WaveNumber));
        Phases.Component(I)=FMath::Fmod((Swell?Sea.SwellPhaseRadians:Sea.WavePhaseRadians)+Sea.ElapsedSeconds*Omega+I*1.7,2.*PI);
    }
    Material->SetVectorParameterValue(TEXT("WavePhases"),Phases);
    Material->SetScalarParameterValue(TEXT("RipplePhase"),FMath::Fmod(Sea.ElapsedSeconds,2.*PI));
    auto* Player=GetWorld()->GetFirstPlayerController();
    if(!Player || !Player->PlayerCameraManager) return;
    const double Height=FMath::Abs(Player->PlayerCameraManager->GetCameraLocation().Z-SurfaceZ);
    // Hysteresis prevents repeated swaps at an altitude threshold.
    int32 LOD=Height<20000.?0:Height<80000.?1:2;
    if(CurrentLOD==0 && Height<24000.) LOD=0;
    if(CurrentLOD==1 && Height>=16000. && Height<96000.) LOD=1;
    if(CurrentLOD==2 && Height>=64000.) LOD=2;
    if(LOD!=CurrentLOD)
    {
        if(CurrentLOD!=INDEX_NONE) Surface->SetMeshSectionVisible(CurrentLOD,false);
        Surface->SetMeshSectionVisible(LOD,true);
        CurrentLOD=LOD;
    }
}

void UPortWaterVisualComponent::EndPlay(EEndPlayReason::Type Reason)
{
    if(auto* GameViewport=Viewport.Get()) GameViewport->OnBeginDraw().RemoveAll(this);
    Super::EndPlay(Reason);
}
