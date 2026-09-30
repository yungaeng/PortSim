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
    SiteActor->SetActorLabel(TEXT("DGT_7_PlanBoundary_1050m"));
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
        if(Position.X>0) { Position.X=TerminalLayout::SiteX(Position.X); Size.X*=TerminalLayout::PlanDepthScale; }
        return Mesh->AddInstance(FTransform(FQuat::Identity,Position*100.,Size));
    };
    auto OrientedBox=[](UHierarchicalInstancedStaticMeshComponent* Mesh,FVector Position,FVector Size,float Yaw)
    {
        return Mesh->AddInstance(FTransform(FRotator(0,Yaw,0),Position*100.,Size));
    };
    auto RoadSegment=[&](UHierarchicalInstancedStaticMeshComponent* Mesh,FVector2D A,FVector2D B,float Width)
    {
        A.X=TerminalLayout::SiteX(A.X); B.X=TerminalLayout::SiteX(B.X);
        const FVector2D Delta=B-A;
        const float Yaw=FMath::RadiansToDegrees(FMath::Atan2(Delta.Y,Delta.X));
        Mesh->AddInstance(FTransform(FRotator(0,Yaw,0),FVector((A.X+B.X)*50.,(A.Y+B.Y)*50.,24),
            FVector(Delta.Size(),Width*.75f,.06f)));
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
        Position.X=TerminalLayout::SiteX(Position.X); Text->SetRelativeLocation(Position*100.);
        Text->SetRelativeRotation(FRotator(90,0,0));
        Text->SetHorizontalAlignment(EHTA_Center);
        Text->SetWorldSize(Size*100.f);
        Text->SetTextRenderColor(FColor::White);
        Text->SetText(FText::FromString(Name));
        Text->RegisterComponent();
    };
    // The approved plan has a continuous yard loop and a bent inland road
    // into the gate. No diagonal roads cut across the open cargo hardstand.
    Box(Roads,FVector(145,0,.24),FVector(50,1020,.06));
    for(int32 I=0;I<85;++I)
        Box(White,FVector(145,-500+I*12,.29),FVector(.15,5,.02));
    RoadSegment(Roads,FVector2D(145,-505),FVector2D(635,-505),18);
    RoadSegment(Roads,FVector2D(635,-505),FVector2D(635,185),22);
    RoadSegment(Roads,FVector2D(635,185),FVector2D(665,220),22);
    RoadSegment(Roads,FVector2D(665,220),FVector2D(665,295),22);
    RoadSegment(Roads,FVector2D(665,295),FVector2D(635,430),22);
    RoadSegment(Roads,FVector2D(635,430),FVector2D(635,480),22);
    RoadSegment(Roads,FVector2D(145,480),FVector2D(890,480),22);
    RoadSegment(Roads,FVector2D(145,300),FVector2D(145,480),24);
    // Short, broad apron to the right-hand gate; no long surplus tail.
    RoadSegment(Roads,FVector2D(690,480),FVector2D(890,480),38);
    for(int32 I=0;I<38;++I)
        Box(White,FVector(175+I*18,480,.29),FVector(7,.15,.02));
    for(int32 I=0;I<TerminalLayout::SiteBoundaryPointCount-1;++I)
    {
        const FVector2D A(TerminalLayout::SiteBoundaryProfile[I][0]-30.f,FMath::Clamp(TerminalLayout::SiteBoundaryProfile[I][1],-505.f,505.f));
        const FVector2D B(TerminalLayout::SiteBoundaryProfile[I+1][0]-30.f,FMath::Clamp(TerminalLayout::SiteBoundaryProfile[I+1][1],-505.f,505.f));
        RoadSegment(Roads,A,B,10);
    }
    // Actual ISO boxes retain size; alternate bays leave >=14m pitch at plan scale.
    // HISM batches avoid thousands of ticking/physics actors for the context yard.
    for (int32 Block=0;Block<18;++Block)
    {
        const float Y=TerminalLayout::BlockY(Block);
        Box(Roads,FVector(405,Y,.24),FVector(430,34,.04));
        for (int32 Rail:{-1,1}) Box(White,FVector(405,Y+Rail*16,.30),FVector(430,.25,.12));
        for (int32 Bay=0;Bay<30;Bay+=2)
            for (int32 Row=0;Row<7;++Row)
                for (int32 Tier=0;Tier<2+(Bay+Row+Block)%3;++Tier)
                {
                    if (TerminalLayout::ContainerOverlapsEmptyZone(TerminalLayout::SiteX(205.f+Bay*13.f),Y+(Row-3)*3.f)) continue;
                    const bool FormerSource=(Bay==4 || Bay==23) && Row==2;
                    const bool FormerDestination=(Bay==6 || Bay==25) && Row==4;
                    constexpr bool FourZones=false;
                    if (FourZones && (Bay==6 || Bay==14 || Bay==21)) continue;
                    if (FormerDestination || (FormerSource && Tier>0)) continue;
                    if (!FormerSource && (Bay*7+Row+Block)%11==0) continue;
                    FSiteYardSlot Slot;
                    Slot.Position=FVector(TerminalLayout::SiteX(205+Bay*13)*100,(Y+(Row-3)*3)*100,149.5f+Tier*259);
                    Slot.Block=Block; Slot.Half=Bay>=15?1:0; Slot.Color=(Bay+Row+Block)%4;
                    const int32 Zone=FourZones?(Bay<7?0:Bay<15?1:Bay<22?2:3):Slot.Half;
                    Slot.Crane=WorkingCranes.Num()+Zone;
                    const float DockX=FourZones?(205.f+(Zone==0?0:Zone==1?7:Zone==2?15:22)*13.f):(Slot.Half?420.f:180.f);
                    Slot.Handover=FVector(TerminalLayout::SiteX(DockX)*100,(Y+13.2f)*100,0);
                    YardSlots.Add(Slot);
                }
        // Two disjoint work reservations per block: no shared gantry travel zone.
        constexpr int32 ZoneCount=2;
        for (int32 Half=0;Half<ZoneCount;++Half)
        {
            const float X=ZoneCount==4?(205.f+(Half==0?0:Half==1?7:Half==2?15:22)*13.f):(Half?517.f:270.f);
            auto* Crane=GetWorld()->SpawnActor<APortWorkingCrane>(FVector(TerminalLayout::SiteX(X)*100,Y*100,0),FRotator(0,90,0),Params);
            WorkingCranes.Add(Crane);
            Crane->Configure(WorkingCranes.Num(),false,FVector(TerminalLayout::SiteX(X)*100,(Y-3)*100,149.5f),FVector(TerminalLayout::SiteX(X+13)*100,(Y+3)*100,149.5f),false);
        }
        for (int32 Half=0;Half<ZoneCount;++Half)
        {
            const float HX=ZoneCount==4?(205.f+(Half==0?0:Half==1?7:Half==2?15:22)*13.f):(Half?420.f:180.f);
            Box(Roads,FVector(HX,Y+13.2f,.25),FVector(15,3.6,.06));
            for (float Side:{-1.f,1.f})
                Box(White,FVector(HX,Y+13.2f+Side*1.8f,.30),FVector(15,.12,.02));
        }
        const TCHAR* ZoneCode=Block<10?TEXT("RF"):TEXT("CY");
        Label(FString::Printf(TEXT("%s %02d"),ZoneCode,Block+1),FVector(177,Y,.4),2.5f);
    }
    Label(TEXT("7 REEFER / CONTAINER YARD"),FVector(395,-230,32),3.4f);
    // A vacant hardstand, not a label superimposed on container rows.
    Box(Roads,FVector(TerminalLayout::EmptyZoneX,TerminalLayout::EmptyZoneY,.25),
        FVector(TerminalLayout::EmptyZoneDepth,TerminalLayout::EmptyZoneLength,.06));
    ZoneOutline(White,FVector2D(TerminalLayout::EmptyZoneX,TerminalLayout::EmptyZoneY),
        FVector2D(TerminalLayout::EmptyZoneDepth,TerminalLayout::EmptyZoneLength));
    Label(TEXT("9 EMPTY YARD - KEEP CLEAR"),FVector(400,246,.5),4);
    int32 STSIndex=0;
    const TArray<float> STSPositions=bUnifiedTerminal?
        TArray<float>{TerminalLayout::STSCenterY(0),TerminalLayout::STSCenterY(1),TerminalLayout::STSCenterY(2),TerminalLayout::STSCenterY(3),TerminalLayout::STSCenterY(4),TerminalLayout::STSCenterY(5),TerminalLayout::STSCenterY(6),TerminalLayout::STSCenterY(7),TerminalLayout::STSCenterY(8)}:
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
        Crane->Configure(WorkingCranes.Num(),true,FVector(-1600,(Y-8)*100,ShipDeck+129.5f),FVector(TerminalLayout::SiteCmX(4500),(Y+8)*100,149.5f),false);
        const FVector Position(-1600,(Y-8)*100,ShipDeck+129.5f);
        if (!bUnifiedTerminal) SiteLogistics->AddShipCargo(Position,STSIndex);
        ++STSIndex;
    }
    // Fast-test fleet: 10 rows x 10 bays x 2 tiers = 200 per vessel.
    // Spread bays over the hull so all three STSs on each vessel still participate.
    const int32 ShipRows=bUnifiedTerminal?TerminalLayout::VesselRows:11;
    const int32 ShipBays=bUnifiedTerminal?TerminalLayout::VesselBays:16;
    const int32 ShipTiers=bUnifiedTerminal?TerminalLayout::VesselTiers:3;
    const TArray<float> ShipPositions=bUnifiedTerminal?TArray<float>{TerminalLayout::VesselCenterY(0),TerminalLayout::VesselCenterY(1),TerminalLayout::VesselCenterY(2)}:TArray<float>{-350,350};
    int32 VesselIndex=0;
    for (float Y:ShipPositions)
    {
        Box(Roads,FVector(-32,Y,-1),FVector(45,bUnifiedTerminal?TerminalLayout::VesselLengthM:285.f,10));
        Box(Blue,FVector(-32,Y,4.2),FVector(43,bUnifiedTerminal?TerminalLayout::VesselLengthM:285.f,.4));
        Box(Roads,FVector(-32,Y+(bUnifiedTerminal?108.5f:146.f),-1),FVector(29,7,10));
        Box(Buildings,FVector(-32,Y-(bUnifiedTerminal?91.f:125.f),14),FVector(38,18,20));
        for (int32 Row=0;Row<ShipRows;++Row)
            for (int32 Bay=0;Bay<ShipBays;++Bay)
                for (int32 Tier=0;Tier<ShipTiers;++Tier)
                {
                    const float BayY=bUnifiedTerminal?-81.f+Bay*18.f:-99.f+Bay*13.f;
                    const FVector Position(-49+Row*3,Y+BayY,5.695+Tier*2.59);
                    const float CraneSpacing=bUnifiedTerminal?70.f:100.f;
                    const int32 Nearest=FMath::Clamp(FMath::RoundToInt((Position.Y-(Y-CraneSpacing))/CraneSpacing),0,2)+(bUnifiedTerminal?VesselIndex*3:(Y<0?0:5));
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
    // Footprints and open areas follow the approved top-down plan V3.
    // Published quay length sets scale; the drawing is not a cadastral survey.
    Box(Roads,FVector(675,-55,.22),FVector(64,82,.04));
    Box(White,FVector(675,-55,3.2),FVector(34,58,6));
    Box(Blue,FVector(675,-55,6.5),FVector(36,60,.6));
    Building(TEXT("1 OPERATIONS"),680,-62,FVector(18,20,24));
    Parking(FVector2D(675,-96),FVector2D(36,14),5);
    Box(Roads,FVector(675,-240,.22),FVector(50,104,.04));
    Box(Buildings,FVector(675,-240,3.2),FVector(26,86,6));
    Box(White,FVector(675,-240,6.5),FVector(28,88,.6));
    Label(TEXT("6 SUBSTATION"),FVector(675,-240,8),3);

    // 2 is on the quay-side strip, clear of the operational AGV lanes.
    Box(Roads,FVector(72,290,.22),FVector(40,104,.04));
    Box(Buildings,FVector(72,290,3.2),FVector(24,86,6));
    Box(White,FVector(72,290,6.5),FVector(26,90,.6));
    Label(TEXT("2 WORKER REST"),FVector(72,290,8),3);

    // 3: long workshop inland, smaller office toward the quay.
    Box(Roads,FVector(330,365,.22),FVector(208,104,.04));
    Building(TEXT("3 MAINTENANCE WORKSHOP"),365,365,FVector(164,46,18));
    Building(TEXT("3 MAINTENANCE OFFICE"),219,381,FVector(72,30,11));
    for(int32 I=0;I<7;++I)
        Box(Roads,FVector(295+I*23,341.8,6.5),FVector(15,.2,12));
    Parking(FVector2D(272,325),FVector2D(120,18),7);

    // Explicit vacant reserve boundary between operations (1) and CIS (5).
    // The inland edge follows the curved site profile with a generous setback.
    const FVector2D OperationsCISReserve[]={
        {685,0},{705,0},{705,45},{709,58},{728,76},{754,90},
        {788,108},{816,125},{825,140},{825,185},{685,185}
    };
    constexpr int32 ReservePoints=UE_ARRAY_COUNT(OperationsCISReserve);
    for(int32 I=0;I<ReservePoints;++I)
        RoadSegment(White,OperationsCISReserve[I],OperationsCISReserve[(I+1)%ReservePoints],.9f);
    Label(TEXT("OPEN RESERVE | 1 - 5"),FVector(735,95,.5),3);

    // 5 sits at the right of its large open apron, leaving space to its left.
    Box(Roads,FVector(768,165,.23),FVector(142,180,.04));
    Box(White,FVector(750,230,4.2),FVector(36,42,8));
    Box(Blue,FVector(750,230,8.5),FVector(38,44,.6));
    Label(TEXT("5 CIS"),FVector(750,230,10),3);
    Parking(FVector2D(750,201),FVector2D(40,10),5);

    // 10 has exactly three detached sheds: the first two form a close pair.
    Box(Roads,FVector(555,365,.22),FVector(74,140,.04));
    for(float Y:{327.f,346.f,398.f})
    {
        Building(TEXT(""),550,Y,FVector(36,14,6));
        Box(Roads,FVector(531.8,Y,2.7),FVector(.2,9,5));
        Box(White,FVector(524,Y,.3),FVector(14,12,.06));
    }
    Label(TEXT("10 REPAIR / WASH - 3 SHEDS"),FVector(594,365,.5),3);

    // 8 and the gate share a compact upper wing, as in the approved image.
    Box(Roads,FVector(802,370,.23),FVector(192,156,.04));
    ZoneOutline(White,FVector2D(802,370),FVector2D(192,156));
    Label(TEXT("8 NON-STANDARD CARGO"),FVector(802,370,.5),4);
    Box(Blue,FVector(TerminalLayout::GateX,TerminalLayout::GateY,9),FVector(36,64,1));
    for(int32 Lane=0;Lane<=8;++Lane)
    {
        const float Y=TerminalLayout::GateY-28+Lane*7;
        Box(White,FVector(TerminalLayout::GateX,Y,.3),FVector(102,.15,.04));
        Box(Buildings,FVector(TerminalLayout::GateX,Y,4.3),FVector(.8,.8,8.6));
        if(Lane<8) Box(White,FVector(TerminalLayout::GateX-13,Y+1,1.5),FVector(4,1.5,3));
    }
    Label(TEXT("4 GATE"),FVector(TerminalLayout::GateX,TerminalLayout::GateY,11),4);
    Label(TEXT("DGT | BUSAN NEW PORT 7 | 1,050 m"),FVector(72,-200,.4),5.f);
    SiteLogistics->Initialize(WorkingCranes,MoveTemp(YardSlots),{Blue,Yellow,Red,Green},bUnifiedTerminal?0:24,FixedYard);
    SiteLogistics->RegisterBerthVehicles(AGVActors);
    if (bUnifiedTerminal) BuildSupportFleet();
}
