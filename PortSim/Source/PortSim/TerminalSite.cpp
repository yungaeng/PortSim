#include "QuayCrane.h"
#include "PortWorkingCrane.h"
#include "PortSiteLogistics.h"
#include "TerminalLayout.h"
#include "Components/HierarchicalInstancedStaticMeshComponent.h"
#include "Components/TextRenderComponent.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "Materials/MaterialInterface.h"

// Site dimensions are published totals; internal buildings/equipment are blockouts.
// Coordinates in this function are metres: X inland, Y along the quay, Z up.
void AQuayCrane::BuildTerminalSite()
{
    FActorSpawnParameters Params;
    Params.Owner=this;
    SiteLogistics=GetWorld()->SpawnActor<APortSiteLogistics>(FVector::ZeroVector,FRotator::ZeroRotator,Params);
    SiteLogistics->SetSTSProfile(STSProfile);
    TArray<FSiteYardSlot> YardSlots;
    int32 FixedYard=0;
    SiteActor=GetWorld()->SpawnActor<AActor>(FVector::ZeroVector,FRotator::ZeroRotator,Params);
    auto* Root=NewObject<USceneComponent>(SiteActor,TEXT("SiteRoot"));
    SiteActor->AddInstanceComponent(Root);
    SiteActor->SetRootComponent(Root);
    Root->RegisterComponent();
    SiteActor->Tags.Add(TEXT("PortSim.DGT.SiteBlockout"));
#if WITH_EDITOR
    SiteActor->SetActorLabel(TEXT("DGT_7_Site_1050m_837201m2"));
    SiteActor->SetFolderPath(TEXT("PortSim/Site"));
#endif
    auto* Cube=LoadObject<UStaticMesh>(nullptr,TEXT("/Engine/BasicShapes/Cube.Cube"));
    auto Group=[&](const TCHAR* Name,const TCHAR* Material)
    {
        auto* Mesh=NewObject<UHierarchicalInstancedStaticMeshComponent>(SiteActor,Name);
        SiteActor->AddInstanceComponent(Mesh);
        Mesh->SetupAttachment(Root);
        Mesh->SetStaticMesh(Cube);
        Mesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);
        const FString Path=FString::Printf(TEXT("/Game/PortSim/Assets/Materials/M_%s.M_%s"),Material,Material);
        if (auto* Mat=LoadObject<UMaterialInterface>(nullptr,*Path)) Mesh->SetMaterial(0,Mat);
        Mesh->RegisterComponent();
        return Mesh;
    };
    auto* Roads=Group(TEXT("Site_Roads"),TEXT("SiteRoad"));
    auto* White=Group(TEXT("Site_Markings"),TEXT("SiteWhite"));
    auto* Buildings=Group(TEXT("Site_Buildings"),TEXT("SiteBuilding"));
    auto* Blue=Group(TEXT("Site_Blue"),TEXT("SiteBlue"));
    auto* Yellow=Group(TEXT("Site_Cranes"),TEXT("SiteOrange"));
    auto* Red=Group(TEXT("Site_Red"),TEXT("SiteRed"));
    auto* Green=Group(TEXT("Site_Green"),TEXT("SiteGreen"));
    UHierarchicalInstancedStaticMeshComponent* Containers[]={Blue,Yellow,Red,Green};
    auto Box=[](UHierarchicalInstancedStaticMeshComponent* Mesh,FVector Position,FVector Size)
    {
        return Mesh->AddInstance(FTransform(FQuat::Identity,Position*100.,Size));
    };
    auto Label=[&](const FString& Name,FVector Position,float Size=4.f)
    {
        auto* Text=NewObject<UTextRenderComponent>(SiteActor);
        SiteActor->AddInstanceComponent(Text);
        Text->SetupAttachment(Root);
        Text->SetRelativeLocation(Position*100.);
        Text->SetRelativeRotation(FRotator(90,0,0));
        Text->SetHorizontalAlignment(EHTA_Center);
        Text->SetWorldSize(Size*100.f);
        Text->SetTextRenderColor(FColor::White);
        Text->SetText(FText::FromString(Name));
        Text->RegisterComponent();
    };
    // Perimeter and cross-yard roads, with open access at the inland gate.
    for (float X:{145.f,650.f,775.f})
    {
        Box(Roads,FVector(X,0,.24),FVector(X==145.f?60.f:18.f,1020,.06));
        for (int32 I=0;I<85;++I) Box(White,FVector(X,-500+I*12,.28),FVector(.15,5,.02));
    }
    for (float Y:{-505.f,325.f,505.f})
    {
        Box(Roads,FVector(390,Y,.24),FVector(760,18,.06));
        for (int32 I=0;I<63;++I) Box(White,FVector(16+I*12,Y,.28),FVector(5,.15,.02));
    }
    // Eighteen blocks: 40 ft boxes retain their physical ISO-sized envelope.
    // HISM batches avoid thousands of ticking/physics actors for the context yard.
    for (int32 Block=0;Block<18;++Block)
    {
        const float Y=-460.f+Block*44.f;
        Box(Roads,FVector(405,Y,.24),FVector(430,34,.04));
        for (int32 Rail:{-1,1}) Box(White,FVector(405,Y+Rail*16,.30),FVector(430,.25,.12));
        for (int32 Bay=0;Bay<30;++Bay)
            for (int32 Row=0;Row<7;++Row)
                for (int32 Tier=0;Tier<2+(Bay+Row+Block)%3;++Tier)
                {
                    const bool FormerSource=(Bay==4 || Bay==23) && Row==2;
                    const bool FormerDestination=(Bay==6 || Bay==25) && Row==4;
                    const bool FourZones=bUnifiedTerminal && Block<5;
                    if (FourZones && (Bay==6 || Bay==14 || Bay==21)) continue;
                    if (FormerDestination || (FormerSource && Tier>0)) continue;
                    if (!FormerSource && (Bay*7+Row+Block)%11==0) continue;
                    FSiteYardSlot Slot;
                    Slot.Position=FVector((205+Bay*13)*100,(Y+(Row-3)*3)*100,149.5f+Tier*259);
                    Slot.Block=Block; Slot.Half=Bay>=15?1:0; Slot.Color=(Bay+Row+Block)%4;
                    const int32 Zone=FourZones?(Bay<7?0:Bay<15?1:Bay<22?2:3):Slot.Half;
                    Slot.Crane=WorkingCranes.Num()+Zone;
                    const float DockX=FourZones?(205.f+(Zone==0?0:Zone==1?7:Zone==2?15:22)*13.f):(Slot.Half?420.f:180.f);
                    Slot.Handover=FVector(DockX*100,(Y+13.2f)*100,0);
                    YardSlots.Add(Slot);
                }
        // Two disjoint work reservations per block: no shared gantry travel zone.
        const int32 ZoneCount=bUnifiedTerminal && Block<5?4:2;
        for (int32 Half=0;Half<ZoneCount;++Half)
        {
            const float X=ZoneCount==4?(205.f+(Half==0?0:Half==1?7:Half==2?15:22)*13.f):(Half?517.f:270.f);
            auto* Crane=GetWorld()->SpawnActor<APortWorkingCrane>(FVector(X*100,Y*100,0),FRotator(0,90,0),Params);
            WorkingCranes.Add(Crane);
            Crane->Configure(WorkingCranes.Num(),false,FVector(X*100,(Y-3)*100,149.5f),FVector((X+13)*100,(Y+3)*100,149.5f),false);
        }
        for (int32 Half=0;Half<ZoneCount;++Half)
        {
            const float HX=ZoneCount==4?(205.f+(Half==0?0:Half==1?7:Half==2?15:22)*13.f):(Half?420.f:180.f);
            Box(Roads,FVector(HX,Y+13.2f,.25),FVector(15,3.6,.06));
            for (float Side:{-1.f,1.f})
                Box(White,FVector(HX,Y+13.2f+Side*1.8f,.30),FVector(15,.12,.02));
        }
        Label(FString::Printf(TEXT("%s %02d"),Block<3?TEXT("REEFER"):TEXT("CY"),Block+1),FVector(177,Y,.4),2.5f);
    }
    int32 STSIndex=0;
    const TArray<float> STSPositions=bUnifiedTerminal?
        TArray<float>{-450,-350,-250,-100,0,100,250,350,450}:
        TArray<float>{-450,-350,-250,-100,100,250,350,450};
    for (float Y:STSPositions)
    {
        if (bUnifiedTerminal)
        {
            Box(Roads,FVector(55,Y+28,.25),FVector(4.2,15,.06));
            for (float Side:{-1.f,1.f}) Box(White,FVector(55+Side*2.1f,Y+28,.31),FVector(.12,15,.02));
            Label(TEXT("NEXT AGV"),FVector(55,Y+28,.4),1.f);
        }
        auto* Crane=GetWorld()->SpawnActor<APortWorkingCrane>(FVector(1200,Y*100,0),FRotator::ZeroRotator,Params);
        WorkingCranes.Add(Crane);
    Crane->SetSTSProfile(STSProfile);
        const float ShipDeck=(!bUnifiedTerminal && FMath::Abs(Y)<200.f)?200.f:440.f;
        Crane->Configure(WorkingCranes.Num(),true,FVector(-1600,(Y-8)*100,ShipDeck+129.5f),FVector(4500,(Y+8)*100,149.5f),false);
        const FVector Position(-1600,(Y-8)*100,ShipDeck+129.5f);
        if (!bUnifiedTerminal) SiteLogistics->AddShipCargo(Position,STSIndex);
        ++STSIndex;
    }
    // Identical hull, deck and 11 x 16 x 3 container arrangement at every berth.
    const TArray<float> ShipPositions=bUnifiedTerminal?TArray<float>{-350,0,350}:TArray<float>{-350,350};
    int32 VesselIndex=0;
    for (float Y:ShipPositions)
    {
        Box(Roads,FVector(-32,Y,-1),FVector(45,285,10));
        Box(Blue,FVector(-32,Y,4.2),FVector(43,285,.4));
        Box(Roads,FVector(-32,Y+146,-1),FVector(29,7,10));
        Box(Buildings,FVector(-32,Y-125,14),FVector(38,18,20));
        for (int32 Row=0;Row<11;++Row)
            for (int32 Bay=0;Bay<16;++Bay)
                for (int32 Tier=0;Tier<3;++Tier)
                {
                    const FVector Position(-49+Row*3,Y-99+Bay*13,5.695+Tier*2.59);
                    const int32 Nearest=FMath::Clamp(FMath::RoundToInt((Position.Y-(Y-100))/100.f),0,2)+(bUnifiedTerminal?VesselIndex*3:(Y<0?0:5));
                    SiteLogistics->AddShipCargo(Position*100.,Nearest);
                }
        ++VesselIndex;
    }
    auto Building=[&](const TCHAR* Name,float X,float Y,FVector Size)
    {
        Box(Buildings,FVector(X,Y,.2+Size.Z*.5),Size);
        Box(Blue,FVector(X,Y,.4+Size.Z),FVector(Size.X+2,Size.Y+2,.5));
        Label(Name,FVector(X,Y,Size.Z+1),3.f);
    };
    Building(TEXT("1 OPERATIONS"),714,-190,FVector(45,65,28));
    Building(TEXT("2 WORKER REST"),714,-290,FVector(24,45,7));
    Building(TEXT("3 MAINTENANCE"),210,425,FVector(65,90,14));
    Building(TEXT("5 CIS"),710,20,FVector(40,40,9));
    Building(TEXT("6 SUBSTATION"),710,-400,FVector(30,45,8));
    Building(TEXT("10 WASH / REPAIR"),350,425,FVector(55,80,12));
    Box(Roads,FVector(708,175,.24),FVector(90,160,.06));
    Label(TEXT("8 NON-STANDARD CARGO"),FVector(708,175,.4),3.f);
    for (int32 I=0;I<8;++I) Box(Yellow,FVector(685+(I%2)*35,125+(I/2)*30,2.2),FVector(20,10,4));
    Label(TEXT("9 EMPTY CONTAINERS"),FVector(500,355,.4),3.f);
    for (int32 Row=0;Row<11;++Row)
        for (int32 Bay=0;Bay<8;++Bay)
            for (int32 Tier=0;Tier<3;++Tier)
                { Box(Containers[(Row+Bay)%4],FVector(440+Bay*13,375+Row*3,1.495+Tier*2.59),FVector(12.192,2.438,2.59)); ++FixedYard; }
    // Ten 4 m gate lanes, canopy and booths aligned to the inland access road.
    Box(Blue,FVector(760,420,8),FVector(22,44,1));
    for (int32 Lane=0;Lane<11;++Lane)
    {
        Box(White,FVector(760,400+Lane*4,.3),FVector(45,.15,.1));
        Box(Buildings,FVector(760,400+Lane*4,3.9),FVector(1,1,7.4));
    }
    Label(TEXT("4 GATE"),FVector(760,420,9),4.f);
    Label(TEXT("DGT | BUSAN NEW PORT 7 | 1,050 m"),FVector(72,-200,.4),5.f);
    SiteLogistics->Initialize(WorkingCranes,MoveTemp(YardSlots),{Blue,Yellow,Red,Green},bUnifiedTerminal?0:24,FixedYard);
    SiteLogistics->RegisterBerthVehicles(AGVActors);
    if (bUnifiedTerminal) BuildSupportFleet();
}
