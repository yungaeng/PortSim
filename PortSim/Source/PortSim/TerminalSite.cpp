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
    SiteActor->SetActorLabel(TEXT("DGT_7_Site_1050m_836755m2"));
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
    auto* TreeCanopies=NewObject<UHierarchicalInstancedStaticMeshComponent>(SiteActor,TEXT("Site_TreeCanopies"));
    SiteActor->AddInstanceComponent(TreeCanopies);
    TreeCanopies->SetupAttachment(Root);
    TreeCanopies->SetStaticMesh(LoadObject<UStaticMesh>(nullptr,TEXT("/Engine/BasicShapes/Sphere.Sphere")));
    TreeCanopies->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    if (auto* Mat=LoadObject<UMaterialInterface>(nullptr,TEXT("/Game/PortSim/Assets/Materials/M_SiteGreen.M_SiteGreen")))
        TreeCanopies->SetMaterial(0,Mat);
    TreeCanopies->RegisterComponent();
    UHierarchicalInstancedStaticMeshComponent* Containers[]={Blue,Yellow,Red,Green};
    auto Box=[](UHierarchicalInstancedStaticMeshComponent* Mesh,FVector Position,FVector Size)
    {
        return Mesh->AddInstance(FTransform(FQuat::Identity,Position*100.,Size));
    };
    auto OrientedBox=[](UHierarchicalInstancedStaticMeshComponent* Mesh,FVector Position,FVector Size,float Yaw)
    {
        return Mesh->AddInstance(FTransform(FRotator(0,Yaw,0),Position*100.,Size));
    };
    auto RoadSegment=[&](UHierarchicalInstancedStaticMeshComponent* Mesh,FVector2D A,FVector2D B,float Width)
    {
        const FVector2D Delta=B-A;
        const float Yaw=FMath::RadiansToDegrees(FMath::Atan2(Delta.Y,Delta.X));
        OrientedBox(Mesh,FVector((A.X+B.X)*.5f,(A.Y+B.Y)*.5f,.24f),
            FVector(Delta.Size(),Width,.06f),Yaw);
    };
    auto ZoneOutline=[&](UHierarchicalInstancedStaticMeshComponent* Mesh,FVector2D Center,FVector2D Size)
    {
        constexpr float LineWidth=1.2f;
        constexpr float LineZ=.38f;
        Box(Mesh,FVector(Center.X,Center.Y-Size.Y*.5f,LineZ),FVector(Size.X,LineWidth,.08f));
        Box(Mesh,FVector(Center.X,Center.Y+Size.Y*.5f,LineZ),FVector(Size.X,LineWidth,.08f));
        Box(Mesh,FVector(Center.X-Size.X*.5f,Center.Y,LineZ),FVector(LineWidth,Size.Y,.08f));
        Box(Mesh,FVector(Center.X+Size.X*.5f,Center.Y,LineZ),FVector(LineWidth,Size.Y,.08f));
    };
    auto Tree=[&](float X,float Y,float Scale=1.f)
    {
        Box(Buildings,FVector(X,Y,2.1f*Scale),FVector(.7f*Scale,.7f*Scale,4.2f*Scale));
        Box(TreeCanopies,FVector(X,Y,5.4f*Scale),FVector(5.2f*Scale,5.2f*Scale,5.2f*Scale));
    };
    auto Parking=[&](FVector2D Center,FVector2D Size,int32 Spaces)
    {
        Box(Roads,FVector(Center.X,Center.Y,.22),FVector(Size.X,Size.Y,.04));
        const float Step=Size.Y/Spaces;
        for (int32 I=0;I<=Spaces;++I)
            Box(White,FVector(Center.X,Center.Y-Size.Y*.5f+I*Step,.29),FVector(Size.X*.42f,.12f,.02f));
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
    // The apron road spans the full berth.  The inland yard road ends where
    // the reference plan opens into the curved service/gate wing.
    Box(Roads,FVector(TerminalLayout::YardRoadX,0,.24),FVector(60,1020,.06));
    for (int32 I=0;I<85;++I)
        Box(White,FVector(TerminalLayout::YardRoadX,-500+I*12,.28),FVector(.15,5,.02));
    Box(Roads,FVector(TerminalLayout::MainInlandRoadX,-105,.24),FVector(18,790,.06));
    for (int32 I=0;I<66;++I)
        Box(White,FVector(TerminalLayout::MainInlandRoadX,-494+I*12,.28),FVector(.15,5,.02));
    // Rear perimeter road follows the same segmented curve as the site mesh.
    for (int32 I=0;I<TerminalLayout::SiteBoundaryPointCount-1;++I)
    {
        RoadSegment(Roads,
            FVector2D(TerminalLayout::SiteBoundaryProfile[I][0],TerminalLayout::SiteBoundaryProfile[I][1]),
            FVector2D(TerminalLayout::SiteBoundaryProfile[I+1][0],TerminalLayout::SiteBoundaryProfile[I+1][1]),18.f);
        RoadSegment(Green,
            FVector2D(TerminalLayout::SiteBoundaryProfile[I][0]-10.f,TerminalLayout::SiteBoundaryProfile[I][1]),
            FVector2D(TerminalLayout::SiteBoundaryProfile[I+1][0]-10.f,TerminalLayout::SiteBoundaryProfile[I+1][1]),4.f);
        const FVector2D A(TerminalLayout::SiteBoundaryProfile[I][0],TerminalLayout::SiteBoundaryProfile[I][1]);
        const FVector2D B(TerminalLayout::SiteBoundaryProfile[I+1][0],TerminalLayout::SiteBoundaryProfile[I+1][1]);
        const int32 TreeCount=FMath::FloorToInt((B-A).Size()/28.f);
        for (int32 TreeIndex=0;TreeIndex<TreeCount;++TreeIndex)
        {
            const float T=(TreeIndex+.5f)/TreeCount;
            const FVector2D P=FMath::Lerp(A,B,T);
            Tree(P.X-23.f,P.Y,.9f+(TreeIndex%3)*.08f);
        }
    }
    // The drawing has a second landscaped edge down the gate-side boundary.
    for (int32 I=0;I<9;++I) Tree(660.f+I*25.f,514.f,.92f+(I%2)*.1f);
    // The east wing rises inland and carries CIS, non-standard cargo and gate.
    Box(Roads,FVector(TerminalLayout::WingRoadX,350.f,.24),FVector(20,300,.06));
    for (int32 I=0;I<25;++I)
        Box(White,FVector(TerminalLayout::WingRoadX,206+I*12,.28),FVector(.15,5,.02));
    Box(Roads,FVector(325,-505,.24),FVector(630,18,.06));
    auto CrossRoad=[&](float Y,float EndX)
    {
        Box(Roads,FVector(EndX*.5f,Y,.24),FVector(EndX-20.f,18,.06));
        for (int32 I=0;I<FMath::FloorToInt((EndX-20.f)/12.f);++I)
            Box(White,FVector(15+I*12,Y,.28),FVector(5,.15,.02));
    };
    CrossRoad(TerminalLayout::EastServiceY,880.f);
    CrossRoad(505.f,865.f);
    // Angled gate approach reproduces the bend visible on the reference plan.
    OrientedBox(Roads,FVector(706,382,.24),FVector(150,22,.06),-42.f);
    OrientedBox(Roads,FVector(825,433,.24),FVector(125,22,.06),28.f);
    RoadSegment(Roads,FVector2D(790,441),FVector2D(TerminalLayout::GateX,TerminalLayout::GateY),44.f);
    for (int32 I=0;I<8;++I)
        Box(Green,FVector(720,-470+I*70,.45),FVector(7,30,.5));
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
        const TCHAR* ZoneCode=Block<10?TEXT("RF"):Block>=11?TEXT("MTY"):TEXT("CY");
        Label(FString::Printf(TEXT("%s %02d"),ZoneCode,Block+1),FVector(177,Y,.4),2.5f);
    }
    // Match the original BPA map: reefer storage occupies the inland/western
    // yard blocks, while empty-container storage is integrated into the
    // quay-side central/eastern blocks. These outlines are planning overlays;
    // the underlying operational slots and crane reservations remain shared.
    ZoneOutline(Green,FVector2D(TerminalLayout::ReeferZoneX,TerminalLayout::ReeferZoneY),
        FVector2D(TerminalLayout::ReeferZoneDepth,TerminalLayout::ReeferZoneLength));
    ZoneOutline(Yellow,FVector2D(TerminalLayout::EmptyZoneX,TerminalLayout::EmptyZoneY),
        FVector2D(TerminalLayout::EmptyZoneDepth,TerminalLayout::EmptyZoneLength));
    Label(TEXT("7 REEFER YARD | 430 x 390 m"),
        FVector(TerminalLayout::ReeferZoneX,TerminalLayout::ReeferZoneY,32.f),3.4f);
    Label(TEXT("9 EMPTY CONTAINER YARD | 310 x 390 m"),
        FVector(TerminalLayout::EmptyZoneX,TerminalLayout::EmptyZoneY,32.f),3.4f);
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
            Label(TEXT("AGV APPROACH"),FVector(55,Y+28,.4),1.f);
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
    // Fast-test fleet: 10 rows x 10 bays x 2 tiers = 200 per vessel.
    // Spread bays over the hull so all three STSs on each vessel still participate.
    const int32 ShipRows=bUnifiedTerminal?TerminalLayout::VesselRows:11;
    const int32 ShipBays=bUnifiedTerminal?TerminalLayout::VesselBays:16;
    const int32 ShipTiers=bUnifiedTerminal?TerminalLayout::VesselTiers:3;
    const TArray<float> ShipPositions=bUnifiedTerminal?TArray<float>{-350,0,350}:TArray<float>{-350,350};
    int32 VesselIndex=0;
    for (float Y:ShipPositions)
    {
        Box(Roads,FVector(-32,Y,-1),FVector(45,285,10));
        Box(Blue,FVector(-32,Y,4.2),FVector(43,285,.4));
        Box(Roads,FVector(-32,Y+146,-1),FVector(29,7,10));
        Box(Buildings,FVector(-32,Y-125,14),FVector(38,18,20));
        for (int32 Row=0;Row<ShipRows;++Row)
            for (int32 Bay=0;Bay<ShipBays;++Bay)
                for (int32 Tier=0;Tier<ShipTiers;++Tier)
                {
                    const float BayY=bUnifiedTerminal?-117.f+Bay*26.f:-99.f+Bay*13.f;
                    const FVector Position(-49+Row*3,Y+BayY,5.695+Tier*2.59);
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
    // Facility pads and footprints are scaled from the 1,050 m berth in the
    // concept drawing.  Pads intentionally include the visible parking/service
    // apron instead of inflating the building mesh itself.
    Box(Roads,FVector(650,-55,.22),FVector(92,112,.04));
    Box(Roads,FVector(72,285,.22),FVector(60,108,.04));
    Box(Roads,FVector(155,445,.22),FVector(112,86,.04));
    Box(Roads,FVector(735,235,.22),FVector(92,92,.04));
    Box(Roads,FVector(650,-300,.22),FVector(68,136,.04));
    Box(Roads,FVector(350,365,.22),FVector(132,112,.04));
    Building(TEXT("1 OPERATIONS"),650,-55,FVector(55,70,18));
    Building(TEXT("2 WORKER REST"),72,285,FVector(38,82,8));
    Building(TEXT("3 MAINTENANCE"),155,445,FVector(82,65,16));
    Building(TEXT("5 CIS"),735,235,FVector(62,62,12));
    Building(TEXT("6 SUBSTATION"),650,-300,FVector(44,112,9));
    Building(TEXT("10 WASH / REPAIR"),350,365,FVector(100,72,14));
    // Small structures shown between the yard exit and the main gate.
    Box(Roads,FVector(702,325,.22),FVector(92,54,.04));
    Building(TEXT("GATE CONTROL"),680,325,FVector(18,28,7));
    Building(TEXT("INSPECTION"),724,325,FVector(18,28,7));
    Parking(FVector2D(605,-55),FVector2D(28,72),12);
    Parking(FVector2D(700,235),FVector2D(24,64),10);
    Parking(FVector2D(105,445),FVector2D(24,58),9);
    Parking(FVector2D(292,365),FVector2D(24,72),11);
    // Transformer banks and repair-yard service bays are visible as detached
    // equipment in the source plan rather than as part of the roof footprint.
    for (int32 I=0;I<4;++I)
        Box(Yellow,FVector(672,-340+I*24,2.0),FVector(12,8,3.2));
    for (int32 I=0;I<3;++I)
        Box(Blue,FVector(405,338+I*25,2.5),FVector(14,9,4.2));
    Box(Roads,FVector(825,395,.24),FVector(140,118,.06));
    Label(TEXT("8 NON-STANDARD CARGO"),FVector(825,395,.4),3.f);
    for (int32 I=0;I<8;++I) Box(Yellow,FVector(795+(I%2)*42,355+(I/2)*27,2.2),FVector(24,11,4));
    // Ten 4 m gate lanes, canopy and booths aligned to the inland access road.
    Box(Blue,FVector(TerminalLayout::GateX,TerminalLayout::GateY,10),FVector(30,58,1));
    for (int32 Lane=0;Lane<11;++Lane)
    {
        const float LaneY=TerminalLayout::GateY-24.2f+Lane*4.4f;
        Box(White,FVector(TerminalLayout::GateX,LaneY,.3),FVector(62,.15,.1));
        Box(Buildings,FVector(TerminalLayout::GateX,LaneY,4.5),FVector(1.2,1.2,8.5));
    }
    // Paired weighbridges and guard booths at the lane merge.
    for (float LaneY:{TerminalLayout::GateY-11.f,TerminalLayout::GateY+11.f})
    {
        Box(White,FVector(835,LaneY,.34),FVector(26,3.2,.16));
        Box(Buildings,FVector(848,LaneY+3.3f,2.2),FVector(4.5,3.2,4));
    }
    Label(TEXT("4 GATE"),FVector(TerminalLayout::GateX,TerminalLayout::GateY,11),4.f);
    Label(TEXT("DGT | BUSAN NEW PORT 7 | 1,050 m"),FVector(72,-200,.4),5.f);
    SiteLogistics->Initialize(WorkingCranes,MoveTemp(YardSlots),{Blue,Yellow,Red,Green},bUnifiedTerminal?0:24,FixedYard);
    SiteLogistics->RegisterBerthVehicles(AGVActors);
    if (bUnifiedTerminal) BuildSupportFleet();
}
