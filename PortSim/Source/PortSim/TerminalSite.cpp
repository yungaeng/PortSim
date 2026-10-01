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
    auto* PaintYellow=Group(TEXT("Site_YellowMarkings"),TEXT("CraneYellow"));
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
    auto BezierRoad=[&](UHierarchicalInstancedStaticMeshComponent* Mesh,FVector2D P0,FVector2D P1,
        FVector2D P2,FVector2D P3,float Width,int32 Steps)
    {
        FVector2D Previous=P0;
        for(int32 I=1;I<=Steps;++I)
        {
            const float T=float(I)/Steps;
            const float U=1.f-T;
            const FVector2D Point=U*U*U*P0+3.f*U*U*T*P1+3.f*U*T*T*P2+T*T*T*P3;
            RoadSegment(Mesh,Previous,Point,Width);
            Previous=Point;
        }
    };
    auto ZoneOutline=[&](UHierarchicalInstancedStaticMeshComponent* Mesh,FVector2D Center,FVector2D Size)
    {
        constexpr float LineWidth=.20f;
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
    // Rounded quay-side corners join the straight inland service road.
    BezierRoad(Roads,{145,-480},{145,-505},{170,-505},{195,-505},18,10);
    RoadSegment(Roads,{195,-505},{595,-505},18);
    BezierRoad(Roads,{595,-505},{635,-505},{635,-485},{635,-465},18,10);
    // Straight service road beneath the combined CIS/cargo block.
    RoadSegment(Roads,{635,-465},{635,480},22);
    RoadSegment(Roads,{195,480},{1035,480},22);
    BezierRoad(Roads,{145,430},{145,480},{170,480},{195,480},22,12);
    RoadSegment(Roads,{550,480},{655,480},38);
    // Continue the gate axis past the inland boundary to the external road.
    for(int32 I=0;I<48;++I)
        Box(White,FVector(175+I*18,480,.29),FVector(7,.15,.02));
    // No perimeter-road spurs behind operations, CIS or cargo facility 8.
    // Positions follow the area-calibrated plan; ISO boxes and cranes retain size.
    // HISM batches avoid thousands of ticking/physics actors for the context yard.
    for (int32 Block=0;Block<TerminalLayout::YardBlockCount;++Block)
    {
        const float Y=TerminalLayout::BlockY(Block);
        // All operational blocks remain west of the vacant facility 9.
        Box(Roads,FVector(405,Y,.24),FVector(430,34,.04));
        for (int32 Rail:{-1,1})
            OrientedBox(White,FVector((TerminalLayout::RMGRailStartX+TerminalLayout::RMGRailEndX)*.5f,
                Y+Rail*TerminalLayout::RMGHalfGaugeM,.30),
                FVector(TerminalLayout::RMGRailEndX-TerminalLayout::RMGRailStartX,.25,.12),0);
        // White slot grid follows the empty-yard satellite reference.
        for(int32 MarkBay=0;MarkBay<30;++MarkBay)
        {
            const float MX=TerminalLayout::YardBayX(MarkBay);
            for(int32 Edge=0;Edge<=7;++Edge)
                OrientedBox(White,FVector(MX,Y-10.5f+Edge*3.f,.39),FVector(12.2f,.12f,.025f),0);
            for(float Side:{-1.f,1.f})
                OrientedBox(White,FVector(MX+Side*6.1f,Y,.39),FVector(.12f,21.f,.025f),0);
        }
        for(float EndX:{172.f,600.f})
            for(int32 Stripe=0;Stripe<7;++Stripe)
                OrientedBox(PaintYellow,FVector(TerminalLayout::SiteX(EndX),Y-9.f+Stripe*3.f,.40),
                    FVector(9.f,.16f,.025f),25.f);
        for (int32 Bay=0;Bay<30;Bay+=2)
            for (int32 Row=0;Row<7;++Row)
                for (int32 Tier=0;Tier<2+(Bay+Row+Block)%3;++Tier)
                {
                    if (!TerminalLayout::IsOperationalYardBay(Bay)) continue;
                    const bool FormerSource=(Bay==4 || Bay==23) && Row==2;
                    const bool FormerDestination=(Bay==6 || Bay==25) && Row==4;
                    constexpr bool FourZones=false;
                    if (FourZones && (Bay==6 || Bay==14 || Bay==21)) continue;
                    if (FormerDestination || (FormerSource && Tier>0)) continue;
                    if (!FormerSource && (Bay*7+Row+Block)%11==0) continue;
                    FSiteYardSlot Slot;
                    Slot.Position=FVector(TerminalLayout::YardBayX(Bay)*100,(Y+(Row-3)*3)*100,149.5f+Tier*259);
                    Slot.Block=Block; Slot.Half=Bay>=15?1:0; Slot.Color=(Bay+Row+Block)%4;
                    const int32 Zone=FourZones?(Bay<7?0:Bay<15?1:Bay<22?2:3):Slot.Half;
                    Slot.Crane=WorkingCranes.Num()+Zone;
                    const float DockX=FourZones?(205.f+(Zone==0?0:Zone==1?7:Zone==2?15:22)*13.f):(Slot.Half?420.f:180.f);
                    Slot.Handover=FVector((FourZones?TerminalLayout::SiteX(DockX):TerminalLayout::RMGHandoverX(Slot.Half))*100,(Y+13.2f)*100,0);
                    YardSlots.Add(Slot);
                }
        // Two disjoint work reservations per block: no shared gantry travel zone.
        constexpr int32 ZoneCount=2;
        for (int32 Half=0;Half<ZoneCount;++Half)
        {
            const float X=ZoneCount==4?(205.f+(Half==0?0:Half==1?7:Half==2?15:22)*13.f):(Half?517.f:270.f);
            const float HomeX=ZoneCount==2?TerminalLayout::RMGHomeX(Half):TerminalLayout::SiteX(X);
            auto* Crane=GetWorld()->SpawnActor<APortWorkingCrane>(FVector(HomeX*100,Y*100,0),FRotator(0,90,0),Params);
            WorkingCranes.Add(Crane);
            Crane->Configure(WorkingCranes.Num(),false,FVector(HomeX*100,(Y-3)*100,149.5f),FVector(TerminalLayout::SiteX(X+13)*100,(Y+3)*100,149.5f),false);
        }
        for (int32 Half=0;Half<ZoneCount;++Half)
        {
            const float HX=ZoneCount==4?(205.f+(Half==0?0:Half==1?7:Half==2?15:22)*13.f):(Half?420.f:180.f);
            Box(Roads,FVector(HX,Y+13.2f,.25),FVector(15,3.6,.06));
            for (float Side:{-1.f,1.f})
                Box(White,FVector(HX,Y+13.2f+Side*1.8f,.30),FVector(15,.12,.02));
        }
        const TCHAR* ZoneCode=TEXT("RF");
        Label(FString::Printf(TEXT("%s %02d"),ZoneCode,Block+1),FVector(177,Y,.4),2.5f);
    }
    ZoneOutline(White,FVector2D(TerminalLayout::ReeferZoneX,TerminalLayout::ReeferZoneY),
        FVector2D(TerminalLayout::ReeferZoneDepth,TerminalLayout::ReeferZoneLength));
    ZoneOutline(White,{395,-340},{430,314});
    ZoneOutline(White,{395,87.5f},{430,489});
    RoadSegment(Roads,{145,TerminalLayout::CentralRoadY},{635,TerminalLayout::CentralRoadY},32);
    for(float X=185;X<620;X+=12)
        Box(White,FVector(X,TerminalLayout::CentralRoadY,.39),FVector(5,.18,.025));
    Label(TEXT("LEFT YARD 01-09"),FVector(610,-340,.4),3.f);
    Label(TEXT("RIGHT YARD 10-23"),FVector(610,87.5,.4),3.f);
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
        OrientedBox(Buildings,FVector(TerminalLayout::SiteX(X),Y,.2+Size.Z*.5),Size,0);
        OrientedBox(Blue,FVector(TerminalLayout::SiteX(X),Y,.4+Size.Z),FVector(Size.X+1,Size.Y+1,.35),0);
        Label(Name,FVector(X,Y,Size.Z+1),3.f);
    };
    // Footprints and open areas follow the approved top-down plan V3.
    // Published quay length and area set scale; the drawing is not a cadastral survey.
    // Shrink rear edges to X=700, inside the narrowest site boundary (X=705).
    Box(Roads,FVector(665,-55,.22),FVector(70,100,.04));
    // Satellite-proportioned estimates in metres, not surveyed dimensions.
    Building(TEXT("1 OPERATIONS"),665,-55,FVector(24,36,8));
    Building(TEXT("1 CONTROL"),672,-62,FVector(12,14,22));
    Parking(FVector2D(665,-101),FVector2D(54,16),6);
    Building(TEXT("6 SUBSTATION"),667.5,-240,FVector(16,64,6));

    // Replace the two rear road stretches beside the substation with parking.
    // Physical 5 m-deep bays on both sides leave a central circulation aisle.
    auto RearParking=[&](float CenterY,float Length,int32 Bays)
    {
        Box(Roads,FVector(675,CenterY,.24),FVector(50,Length,.06));
        const float StartY=CenterY-Bays*2.f;
        for(float X:{655.f,695.f})
        {
            for(int32 Bay=0;Bay<=Bays;++Bay)
                Box(White,FVector(X,StartY+Bay*4,.31),FVector(9,.15,.03));
            Box(White,FVector(X+(X<675?-4.5f:4.5f),CenterY,.31),FVector(.2,Bays*4.f,.03));
        }
        RoadSegment(Roads,{635,CenterY},{675,CenterY},12);
    };
    RearParking(-404,180,42);
    RearParking(-143,62,14);

    // 2 is on the quay-side strip, clear of the operational AGV lanes.
    Box(Roads,FVector(72,290,.22),FVector(72,104,.04));
    Building(TEXT("2 WORKER REST"),72,465,FVector(14,44,5));

    // Open paved staff parking east of the rest building, below maintenance.
    // Two rows of 5 m-deep bays flank a generous central circulation apron.
    Box(Roads,FVector(72,393,.24),FVector(74,110,.06));
    RoadSegment(Roads,{108,355},{145,355},14);
    RoadSegment(Roads,{108,431},{145,431},14);
    for(float X:{45.f,99.f})
    {
        for(int32 Bay=0;Bay<=16;++Bay)
            Box(White,FVector(X,350+Bay*5,.31),FVector(10,.15,.03));
        Box(White,FVector(X+(X<72?-5:5),390,.31),FVector(.25,80,.03));
    }
    Label(TEXT("STAFF PARKING"),FVector(72,391,.4),2.5f);

    // 3: shorten and inset both buildings; roof overhangs stay inside the apron.
    Box(Roads,FVector(340,442,.22),FVector(295,40,.04));
    Building(TEXT("3 MAINTENANCE"),380,442,FVector(72,28,12));
    Building(TEXT("3 OFFICE"),240,442,FVector(32,16,8));
    Parking(FVector2D(292,442),FVector2D(35,26),10);
    // Photo reference: repeated workshop doors and a two-storey office facade.
    for(int32 I=0;I<6;++I)
        OrientedBox(White,FVector(TerminalLayout::SiteX(350)+I*12,427.8,4.2),FVector(7,.3,8),0);
    for(int32 Floor=0;Floor<2;++Floor)
        for(int32 I=0;I<5;++I)
            OrientedBox(Blue,FVector(TerminalLayout::SiteX(228)+I*6,433.8,2.5+Floor*3.5),FVector(3,.3,1.6),0);
    // Explicit vacant reserve boundary between operations (1) and CIS (5).
    // The inland edge follows the curved site profile with a generous setback.
    const FVector2D OperationsCISReserve[]={
        {685,0},{705,0},{705,45},{709,58},{728,76},{754,90},
        {788,108},{816,125},{825,140},{825,185},{685,185}
    };
    constexpr int32 ReservePoints=UE_ARRAY_COUNT(OperationsCISReserve);
    for(int32 I=0;I<ReservePoints;++I)
        RoadSegment(White,OperationsCISReserve[I],OperationsCISReserve[(I+1)%ReservePoints],.9f);
    Label(TEXT("PARKED SUPPORT EQUIPMENT"),FVector(735,95,.5),3);

    // CIS and unused cargo facility 8 share one continuous rectangular block.
    // Extend the shared block to the inland edge of the 22 m service road.
    // Road width is rendered at 75%; Box scales authored X coordinates.
    constexpr float SharedApronRoadEdgeX=635.f+(22.f*.75f*.5f)/TerminalLayout::PlanDepthScale;
    Box(Roads,FVector((SharedApronRoadEdgeX+850.f)*.5f,318,.23),
        FVector(850.f-SharedApronRoadEdgeX,250,.04));
    Building(TEXT("5 CIS"),750,228,FVector(24,20,7));
    Parking(FVector2D(750,198),FVector2D(48,12),8);

    // 10: expand inland only (X=490..610), leaving clearance to the road.
    // Keep the adjoining repair/service pair and the separate wash building.
    Box(Roads,FVector(550,404,.22),FVector(120,116,.04));
    constexpr float ServiceBuildingX=550.f;
    constexpr float ServiceBuildingY[]={365.f,387.f,425.f};
    const TCHAR* ServiceBuildingNames[]={TEXT("10 REPAIR"),TEXT("10 SERVICE"),TEXT("10 WASH")};
    for(int32 I=0;I<UE_ARRAY_COUNT(ServiceBuildingY);++I)
    {
        Building(ServiceBuildingNames[I],ServiceBuildingX,ServiceBuildingY[I],FVector(20,12,6));
        // Each work pad aligns with its own building, including the close pair.
        ZoneOutline(White,{ServiceBuildingX+20.f,ServiceBuildingY[I]},{16,14});
    }
    // 8 occupies the inland/eastern hardstand; the gate is below it on the
    // side boundary rather than at the rear edge.
    // The shared apron is created with CIS above, without a narrow connector.
    Label(TEXT("8 NON-STANDARD CARGO"),FVector(777.5,373,.5),4);
    OrientedBox(Blue,FVector(TerminalLayout::SiteX(TerminalLayout::GateX),TerminalLayout::GateY,7),FVector(18,60,.6),0);
    for(int32 Lane=0;Lane<=8;++Lane)
    {
        const float Y=TerminalLayout::GateY-28+Lane*7;
        Box(White,FVector(TerminalLayout::GateX,Y,.3),FVector(102,.15,.04));
        Box(Buildings,FVector(TerminalLayout::GateX,Y,3.5),FVector(.6,.6,7));
        if(Lane<8) Box(White,FVector(TerminalLayout::GateX-13,Y+1,1.5),FVector(4,1.5,3));
    }
    Label(TEXT("4 GATE"),FVector(TerminalLayout::GateX,TerminalLayout::GateY,11),4);
    // Asphalt is continuous; thin paint defines circulation and work areas.
    
    ZoneOutline(White,{340,442},{295,40});
    ZoneOutline(White,{550,404},{120,116});
    ZoneOutline(White,{747,318},{202,250});
    auto PaintLine=[&](FVector2D A,FVector2D B)
    {
        A.X=TerminalLayout::SiteX(A.X); B.X=TerminalLayout::SiteX(B.X);
        const FVector2D D=B-A;
        OrientedBox(PaintYellow,FVector((A.X+B.X)*.5,(A.Y+B.Y)*.5,.39),
            FVector(D.Size(),.18,.025),FMath::RadiansToDegrees(FMath::Atan2(D.Y,D.X)));
    };
    // Facility 9 is separate from the 23 RMG yards and their cargo inventory.
    ZoneOutline(White,{TerminalLayout::EmptyZoneX,TerminalLayout::EmptyZoneY},
        {TerminalLayout::EmptyZoneDepth,TerminalLayout::EmptyZoneLength});
    // Open entries through both short edges lead into a 16 m handling aisle.
    for(float X:{195.f,480.f})
        Box(Roads,FVector(X,380,.42),FVector(.6,16,.03));
    for(int Bay=0;Bay<19;++Bay)
        for(float Y:{355.f,360.f,365.f,395.f,400.f,405.f})
        {
            const float X=TerminalLayout::SiteX(210.f)+Bay*14.f;
            for(float Side:{-1.f,1.f})
            {
                OrientedBox(White,FVector(X,Y+Side*1.3f,.40),FVector(12.2,.15,.025),0);
                OrientedBox(White,FVector(X+Side*6.1f,Y,.40),FVector(.15,2.6,.025),0);
            }
        }
    Label(TEXT("9 EMPTY CONTAINER YARD"),FVector(337.5,380,.45),4.f);
    for(float X=200;X<470;X+=10)
        PaintLine({X,412},{X+5,417});
    // Keep-clear strips alongside the yard access road and service aprons.
    for(float Y=-480;Y<330;Y+=8)
        PaintLine({166,Y},{173,Y+7});
    for(float Y=350;Y<450;Y+=8)
        PaintLine({488,Y},{496,Y+7});
    for(float Y=-480;Y<465;Y+=12)
        Box(White,FVector(635,Y,.39),FVector(.18,5,.025));
    for(float X=195;X<600;X+=12)
        Box(White,FVector(X,-505,.39),FVector(5,.18,.025));
    Label(TEXT("DGT | BUSAN NEW PORT 7 | 1,050 m"),FVector(72,-200,.4),5.f);
    SiteLogistics->Initialize(WorkingCranes,MoveTemp(YardSlots),{Blue,Yellow,Red,Green},bUnifiedTerminal?0:24,FixedYard);
    SiteLogistics->RegisterBerthVehicles(AGVActors);
    if (bUnifiedTerminal) BuildSupportFleet();
}
