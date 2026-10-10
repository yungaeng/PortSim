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
    // Paint the same parking-crossing network used by the AGV path planner.
    for(int32 Gap=0;Gap<TerminalLayout::AGVParkBays-1;++Gap)
    {
        const float Y=TerminalLayout::AGVParkCrossingY(Gap)/100.f;
        const float Width=TerminalLayout::AGVParkCrossingWidthM;
        Box(Roads,FVector(100,Y,.25f),FVector(130,Width,.06f));
        for(float Side:{-1.f,1.f})
            Box(PaintYellow,FVector(100,Y+Side*Width*.5f,.30f),FVector(130,.12f,.02f));
        const float Direction=TerminalLayout::AGVParkCrossingToYard(Gap)?1.f:-1.f;
        for(float X:{65.f,105.f,145.f})
        {
            const float WorldX=TerminalLayout::SiteX(X);
            OrientedBox(White,FVector(WorldX,Y,.31f),FVector(4,.18f,.02f),0);
            for(float Side:{-1.f,1.f})
                OrientedBox(White,FVector(WorldX+Direction*1.25f,Y+Side*.65f,.31f),
                    FVector(1.8f,.18f,.02f),-Direction*Side*45.f);
        }
    }
    const float NorthParkingAccessY=(TerminalLayout::AGVParkY(TerminalLayout::AGVParkBays-1)+3000.f)/100.f;
    RoadSegment(Roads,{50,NorthParkingAccessY},{167,NorthParkingAccessY},18);
    for(int32 Vehicle=0;Vehicle<TerminalLayout::AGVParkRows*TerminalLayout::AGVParkBays;++Vehicle)
    {
        const float X=TerminalLayout::SiteCmX(TerminalLayout::AGVParkX(Vehicle))/100.f;
        const float Y=TerminalLayout::AGVParkY(Vehicle)/100.f;
        for(float Side:{-1.f,1.f})
        {
            OrientedBox(White,FVector(X+Side*2.5f,Y,.30f),FVector(.12f,16.f,.02f),0);
            OrientedBox(White,FVector(X,Y+Side*8.f,.30f),FVector(5.f,.12f,.02f),0);
        }
    }
    // Rounded quay-side corners join the straight inland service road.
    BezierRoad(Roads,{145,-480},{145,-505},{170,-505},{195,-505},18,10);
    RoadSegment(Roads,{195,-505},{595,-505},18);
    BezierRoad(Roads,{595,-505},{635,-505},{635,-485},{635,-465},18,10);
    // Straight service road beneath the combined CIS/cargo block.
    RoadSegment(Roads,{635,-465},{635,480},26);
    // RoadSegment displays 75% of its nominal width. These paved junction
    // aprons cover the full swept chassis at the widened inland lane turns.
    for(float CrossY:{TerminalLayout::AGVCentralReturnY/100.f,
        TerminalLayout::AGVNorthCrossY/100.f})
        Box(Roads,FVector(635,CrossY,.24),FVector(32,32,.06));
    RoadSegment(Roads,{195,620},{910,620},22);
    BezierRoad(Roads,{145,550},{145,620},{170,620},{195,620},22,12);
    // Gate approach provides the road surface here; no second overlapping strip.
    // Internal perimeter road ends inside the site; no external road spur.
    for(int32 I=0;I<24;++I)
        Box(White,FVector(175+I*18,620,.29),FVector(7,.15,.02));
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
        for(int32 MarkBay=0;MarkBay<30;MarkBay+=2)
        {
            if (!TerminalLayout::IsOperationalYardBay(MarkBay)) continue;
            const float MX=TerminalLayout::YardBayX(MarkBay);
            for(int32 Edge=0;Edge<=7;++Edge)
                OrientedBox(White,FVector(MX,Y-10.5f+Edge*3.f,.39),FVector(12.2f,.12f,.025f),0);
            for(float Side:{-1.f,1.f})
                OrientedBox(White,FVector(MX+Side*6.1f,Y,.39),FVector(.12f,21.f,.025f),0);
        }
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
                    Slot.Handover=FVector((FourZones?TerminalLayout::SiteX(DockX):TerminalLayout::RMGHandoverX(Slot.Half))*100,TerminalLayout::RMGHandoverY(Block)*100,0);
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
            Box(Roads,FVector(HX,TerminalLayout::RMGHandoverY(Block),.25),FVector(15,3.6,.06));
            for (float Side:{-1.f,1.f})
                Box(White,FVector(HX,TerminalLayout::RMGHandoverY(Block)+Side*1.8f,.30),FVector(15,.12,.02));
        }
        const TCHAR* ZoneCode=TEXT("CY");
        Label(FString::Printf(TEXT("%s %02d"),ZoneCode,Block+1),FVector(177,Y,.4),2.5f);
    }
    // Reefer blocks use the same slot markings; no overlapping zone rectangle.
    ZoneOutline(White,{395,-325},{430,314});
    ZoneOutline(White,{395,92.5f},{430,489});
    RoadSegment(Roads,{145,TerminalLayout::CentralRoadY},{650,TerminalLayout::CentralRoadY},28);
    for(float X=185;X<620;X+=12)
        Box(White,FVector(X,TerminalLayout::CentralRoadY,.39),FVector(5,.18,.025));
    // Northern member of the south/central/north AGV circulation network.
    // Its centre is shared with the FMS graph, immediately above yard 23.
    const float AGVNorthRoadY=TerminalLayout::AGVNorthCrossY/100.f;
    RoadSegment(Roads,{145,AGVNorthRoadY},{650,AGVNorthRoadY},16);
    RoadSegment(Roads,{145,TerminalLayout::AGVNorthReturnY/100.f},
        {650,TerminalLayout::AGVNorthReturnY/100.f},6);
    for(float X=185;X<620;X+=12)
        Box(White,FVector(X,AGVNorthRoadY,.39),FVector(5,.18,.025));
    Label(TEXT("LEFT YARD 01-09"),FVector(610,-325,.4),3.f);
    Label(TEXT("RIGHT YARD 10-23"),FVector(610,92.5,.4),3.f);
    // Facility 7 from the marked satellite: inland ends of five central blocks.
    // Reefer identity is shown by its label and equipment, without lines crossing slots.
    Label(TEXT("7 REEFER CONTAINER AREA"),FVector(515,-65,22),3.5f);
    for(int32 B=9;B<14;++B)
    {
        const float Y=TerminalLayout::BlockY(B)-12.f;
        for(float X=450;X<585;X+=14)
        {
            OrientedBox(White,FVector(TerminalLayout::SiteX(X),Y,1.5),FVector(1.2,.65,2.6),0);
            OrientedBox(Blue,FVector(TerminalLayout::SiteX(X),Y,2.9),FVector(1.3,.7,.2),0);
        }
    }
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
        auto* Roof=(FCString::Strstr(Name,TEXT("OPERATIONS")) || FCString::Strstr(Name,TEXT("CONTROL")) || FCString::Strstr(Name,TEXT("SUBSTATION")))?White:Blue;
        OrientedBox(Roof,FVector(TerminalLayout::SiteX(X),Y,.4+Size.Z),FVector(Size.X+1,Size.Y+1,.35),0);
        Label(Name,FVector(X,Y,Size.Z+1),3.f);
    };
    const FVector2D Operations(TerminalLayout::PhotoAnchorX(30.6f),TerminalLayout::PhotoAnchorY(40.7f));
    const FVector2D Substation(TerminalLayout::PhotoAnchorX(34.3f),TerminalLayout::PhotoAnchorY(64.7f));
    const FVector2D Rest(TerminalLayout::PhotoAnchorX(74.2f),TerminalLayout::PhotoAnchorY(9.8f));
    const FVector2D Maintenance(TerminalLayout::PhotoAnchorX(58.8f),TerminalLayout::PhotoAnchorY(8.3f));
    const FVector2D CIS(TerminalLayout::PhotoAnchorX(23.1f),TerminalLayout::PhotoAnchorY(23.2f));
    const FVector2D RepairWash(TerminalLayout::PhotoAnchorX(37.1f),TerminalLayout::PhotoAnchorY(13.7f));
    // Footprints and open areas follow the approved top-down plan V3.
    // Published quay length and area set scale; the drawing is not a cadastral survey.
    // Shrink rear edges to X=700, inside the narrowest site boundary (X=705).
    ZoneOutline(White,{Operations.X-8,Operations.Y},{38,60});
    Building(TEXT("1 OPERATIONS"),Operations.X,Operations.Y,FVector(24,36,8));
    Building(TEXT("1 CONTROL"),Operations.X+7,Operations.Y-7,FVector(12,14,22));
    Parking({Operations.X,Operations.Y-35},{42,16},6);
    Building(TEXT("6 SUBSTATION"),Substation.X,Substation.Y,FVector(18,88,6));
    ZoneOutline(White,Substation,{44,110});
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
    RearParking(-235,62,14);

    // 2 is on the quay-side strip, clear of the operational AGV lanes.
    ZoneOutline(White,Rest,{40,74});
    Building(TEXT("2 WORKER REST"),Rest.X,Rest.Y,FVector(14,44,5));

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

    // Three distinct blue roofs identified in close-up 2.
    // Shared maintenance apron: retain parking markings without an oversized perimeter box.
    Building(TEXT("3 MAINTENANCE"),Maintenance.X,Maintenance.Y,FVector(72,28,12));
    Building(TEXT("3 QUAY SIDE"),Maintenance.X-85,Maintenance.Y+15,FVector(73.f*127.f/154.f-1.f,29.f*52.f/61.f-1.f,8));
    Building(TEXT("3 GATE SIDE"),Maintenance.X+130,Maintenance.Y+5,FVector(18,16,7));
    // Maintenance apron intentionally has no internal pavement markings.
    for(int I=0;I<6;++I)
        OrientedBox(White,FVector(TerminalLayout::SiteX(Maintenance.X)-30+I*12,Maintenance.Y-14.2,4.2),FVector(7,.3,8),0);
    // Equipment parking shares the apron; no extra outline across the access road.
    Label(TEXT("PARKED SUPPORT EQUIPMENT"),FVector(755,395,.5),3);
    // CIS and unused cargo facility 8 share one continuous rectangular block.
    // Extend the shared block to the inland edge of the 22 m service road.
    // Road width is rendered at 75%; Box scales authored X coordinates.
    constexpr float SharedApronRoadEdgeX=635.f+(22.f*.75f*.5f)/TerminalLayout::PlanDepthScale;
    Box(Roads,FVector((SharedApronRoadEdgeX+850.f)*.5f,435,.23),
        FVector(850.f-SharedApronRoadEdgeX,270,.04));
    Building(TEXT("5 CIS"),CIS.X,CIS.Y,FVector(24,25.f*110.f/62.f-1.f,7));
    Parking({CIS.X,CIS.Y-36},{40,12},8);

    // Exactly two small blue-roof buildings. Adjacent long rectangles in the photo are cargo.
    // Keep one work-pad outline clear of the service-road junction.
    Building(TEXT("10 REPAIR"),RepairWash.X,RepairWash.Y-7,FVector(8,7,4));
    Building(TEXT("10 WASH"),RepairWash.X,RepairWash.Y+7,FVector(8,7,4));
    // Wash-pad paint is drawn below, clear of the empty-yard perimeter.
    Label(TEXT("10 REPAIR / WASH WORK AREA"),FVector(RepairWash.X-22,RepairWash.Y+17,.50),1.6f);
    // 8 occupies the inland/eastern hardstand; the gate is below it on the
    // side boundary rather than at the rear edge.
    // The shared apron is created with CIS above, without a narrow connector.
    Label(TEXT("8 NON-STANDARD CARGO"),FVector(840,500,.5),4);
    // Overview registration: inland entrance, canopy perpendicular to the X approach.
    OrientedBox(Blue,FVector(TerminalLayout::SiteX(TerminalLayout::GateX),TerminalLayout::GateY,7),FVector(10,20,.6),0);
    for(int32 Lane=0;Lane<=4;++Lane)
    {
        const float Y=TerminalLayout::GateY-8+Lane*4;
        // Gate lane paint is generated with its continuous approach markings below.
        OrientedBox(Buildings,FVector(TerminalLayout::SiteX(TerminalLayout::GateX),Y,3.5),FVector(.6,.6,7),0);
        if(Lane<4) OrientedBox(White,FVector(TerminalLayout::SiteX(TerminalLayout::GateX)-8,Y+2,1.2),FVector(3,1.2,2.4),0);
    }

    Box(Roads,FVector(TerminalLayout::GateX,TerminalLayout::GateY,.28),FVector(48,22,.04));
    // Quay apron and access below the worker rest building, within the existing boundary.
    Box(Roads,FVector(8,450,.24),FVector(12,370,.06));
    for(float Y=270;Y<630;Y+=12)
        Box(White,FVector(8,Y,.39),FVector(.18,5,.025));
    Box(White,FVector(-6,0,.39),FVector(.35,1050,.06));
    RoadSegment(Roads,{20,630},{145,630},16);
    Label(TEXT("4 GATE"),FVector(TerminalLayout::GateX,TerminalLayout::GateY,11),4);
    // Asphalt is continuous; thin paint defines circulation and work areas.



    // Continuous apron: avoid a second rectangle crossing the gate approaches.
    auto PaintLine=[&](FVector2D A,FVector2D B)
    {
        A.X=TerminalLayout::SiteX(A.X); B.X=TerminalLayout::SiteX(B.X);
        const FVector2D D=B-A;
        OrientedBox(PaintYellow,FVector((A.X+B.X)*.5,(A.Y+B.Y)*.5,.39),
            FVector(D.Size(),.18,.025),FMath::RadiansToDegrees(FMath::Atan2(D.Y,D.X)));
    };
    // Facility 9 is separate from the 23 RMG yards and their cargo inventory.
    // The former square hardstand outline is replaced by the rounded circulation road below.
    // Open entries through both short edges lead into a 16 m handling aisle.

    // Facility 9 traced by region from the red-circled satellite reference:
    // narrow yard-side strip, wider central strip, open building-side pads.
    // Widths differ intentionally; these are not three identical storage banks.
    for(int Strip=0;Strip<2;++Strip)
    {
        const float Y=Strip==0?367.f:419.f;
        const float HalfWidth=Strip==0?8.f:14.f;
        const float Start=TerminalLayout::SiteX(193.f);
        const float End=TerminalLayout::SiteX(605.f);
        for(float Side:{-1.f,1.f})
            OrientedBox(White,FVector((Start+End)*.5f,Y+Side*HalfWidth,.48f),
                FVector(End-Start,.12f,.025f),0);
        for(float X=Start;X<End;X+=6.f)
        {
            // Small breaks visible in the long central strip.
            if(Strip==1 && X>TerminalLayout::SiteX(420.f) && X<TerminalLayout::SiteX(438.f)) continue;
            OrientedBox(White,FVector(X,Y,.48f),FVector(.12f,HalfWidth*2,.025f),0);
        }
    }
    // Long empty rectangle beside maintenance, then a separate shorter pad.
    // Sea is at smaller X; the reference is rotated 180 degrees from plan view.
    ZoneOutline(White,{309,471},{230,30});
    ZoneOutline(White,{469,471},{58,30});
    Label(TEXT("9 EMPTY CONTAINER YARD"),FVector(395,447,.45),3.f);
    auto RoadPaint=[&](UHierarchicalInstancedStaticMeshComponent* Mesh,FVector2D A,FVector2D B,float Width=.18f)
    {
        A.X=TerminalLayout::SiteX(A.X); B.X=TerminalLayout::SiteX(B.X);
        const FVector2D D=B-A;
        OrientedBox(Mesh,FVector((A.X+B.X)*.5,(A.Y+B.Y)*.5,Mesh==Roads?.32f:.48f),
            FVector(D.Size(),Width,.025),FMath::RadiansToDegrees(FMath::Atan2(D.Y,D.X)));
    };
    const float EmptyMinX=TerminalLayout::EmptyZoneX-TerminalLayout::EmptyZoneDepth*.5f;
    const float EmptyMaxX=TerminalLayout::EmptyZoneX+TerminalLayout::EmptyZoneDepth*.5f;
    const float EmptyY=TerminalLayout::EmptyZoneY;
    // The reference has clear dark handling corridors between the marked strips.
    // Do not add highway centre dashes or yellow hatching over this storage apron.
    // Road geometry and paint share the same sampled curves, not separate guessed paths.
    auto Curve=[&](UHierarchicalInstancedStaticMeshComponent* Mesh,FVector2D A,FVector2D B,
        FVector2D C,FVector2D D,float Width)
    {
        FVector2D Last=A;
        for(int I=1;I<=32;++I)
        {
            const float T=I/32.f,U=1-T;
            const FVector2D P=U*U*U*A+3*U*U*T*B+3*U*T*T*C+T*T*T*D;
            RoadPaint(Mesh,Last,P,Width); Last=P;
        }
    };
    // Rounded perimeter of the empty-container storage banks.
    // Paint stops at the four aisle intersections so it never seals an entrance.
    auto EmptyLoop=[&](UHierarchicalInstancedStaticMeshComponent* Mesh,float Offset,float Width,bool Gaps)
    {
        const float L=174-Offset/TerminalLayout::PlanDepthScale;
        const float R=616+Offset/TerminalLayout::PlanDepthScale;
        const float B=346-Offset,T=494+Offset,Radius=14+Offset;
        RoadPaint(Mesh,{L+Radius,B},{R-Radius,B},Width);
        RoadPaint(Mesh,{L+Radius,T},{R-Radius,T},Width);
        for(float X:{L,R})
        {
            if(!Gaps) RoadPaint(Mesh,{X,B+Radius},{X,T-Radius},Width);
            else
            {
                RoadPaint(Mesh,{X,B+Radius},{X,386},Width);
                RoadPaint(Mesh,{X,404},{X,436},Width);
                RoadPaint(Mesh,{X,454},{X,T-Radius},Width);
            }
        }
        Curve(Mesh,{L+Radius,B},{L+Radius*.45f,B},{L,B+Radius*.45f},{L,B+Radius},Width);
        Curve(Mesh,{R-Radius,B},{R-Radius*.45f,B},{R,B+Radius*.45f},{R,B+Radius},Width);
        Curve(Mesh,{L,T-Radius},{L,T-Radius*.45f},{L+Radius*.45f,T},{L+Radius,T},Width);
        Curve(Mesh,{R,T-Radius},{R,T-Radius*.45f},{R-Radius*.45f,T},{R-Radius,T},Width);
    };
    EmptyLoop(Roads,0,10,false);
    EmptyLoop(White,5,.20f,true);
    EmptyLoop(White,-5,.20f,true);
    // Visible junction throats connect the perimeter to each transverse aisle.
    for(float Y:{395.f,445.f})
    {
        for(float X:{174.f,616.f})
            OrientedBox(Roads,FVector(TerminalLayout::SiteX(X),Y,.51),FVector(12,17,.02),0);
        for(float Side:{-1.f,1.f})
        {
            RoadPaint(White,{145,Y+Side*9},{166,Y+Side*9});
            RoadPaint(White,{624,Y+Side*9},{635,Y+Side*9});
        }
    }
    // Red-circled reference: long inland wrap, lower reverse curve, and a
    // separate service spine. Coordinates fit the existing terminal boundary.
    // Lane offsets are taken from one physical centreline to avoid crossing bends.
    TArray<FVector2D> GatePath;
    auto AddGatePoint=[&](FVector2D P)
    {
        const FVector2D End(TerminalLayout::SiteX(P.X),P.Y);
        if(GatePath.IsEmpty()) { GatePath.Add(End); return; }
        const FVector2D Start=GatePath.Last();
        const int Steps=FMath::Max(1,FMath::CeilToInt((End-Start).Size()/1.5f));
        for(int I=1;I<=Steps;++I) GatePath.Add(FMath::Lerp(Start,End,float(I)/Steps));
    };
    auto AddGateCurve=[&](FVector2D A,FVector2D B,FVector2D C,FVector2D D)
    {
        for(int I=1;I<=40;++I)
        {
            const float T=I/40.f,U=1-T;
            AddGatePoint(U*U*U*A+3*U*U*T*B+3*U*T*T*C+T*T*T*D);
        }
    };
    AddGatePoint({635,300}); AddGatePoint({635,530});
    AddGateCurve({635,530},{635,605},{660,625},{715,625});
    AddGatePoint({880,625});
    AddGateCurve({880,625},{903,625},{910,606},{910,580});
    AddGatePoint({910,375});
    AddGateCurve({910,375},{906,349},{852,334},{815,315});
    AddGateCurve({815,315},{796,299},{817,294},{797,273});
    AddGatePoint({759,245});
    AddGateCurve({759,245},{740,235},{725,225},{705,225});
    AddGatePoint({635,225});
    auto GateStroke=[&](UHierarchicalInstancedStaticMeshComponent* Mesh,float Offset,float Width,bool Dashed)
    {
        float Travel=0;
        for(int I=1;I<GatePath.Num();++I)
        {
            FVector2D A=GatePath[I-1],B=GatePath[I];
            const FVector2D D=(B-A).GetSafeNormal();
            const FVector2D TA=(GatePath[I]-GatePath[FMath::Max(0,I-2)]).GetSafeNormal();
            const FVector2D TB=(GatePath[FMath::Min(GatePath.Num()-1,I+1)]-GatePath[I-1]).GetSafeNormal();
            const FVector2D NA(-TA.Y,TA.X),NB(-TB.Y,TB.X);
            const float Length=(B-A).Size();
            for(float Along=0;Along<Length;)
            {
                const float Phase=FMath::Fmod(Travel+Along,10.f);
                const float Step=FMath::Min(Length-Along,Dashed?(Phase<4?4-Phase:10-Phase):Length);
                const float MidY=(A+D*(Along+Step*.5f)).Y;
                const float Spread=(2.f/3.f)+(1.f/3.f)*FMath::Clamp((MidY-500.f)/55.f,0.f,1.f);
                const bool MergedLane=Dashed && FMath::Fmod(FMath::Abs(Offset),14.f)>6.f && Spread<.99f;
                if((!Dashed || Phase<4) && !MergedLane)
                {
                    const float T0=Along/Length,T1=(Along+Step)/Length;
                    const FVector2D P0=A+D*Along;
                    const FVector2D P1=A+D*(Along+Step);
                    const float S0=(2.f/3.f)+(1.f/3.f)*FMath::Clamp((P0.Y-500.f)/55.f,0.f,1.f);
                    const float S1=(2.f/3.f)+(1.f/3.f)*FMath::Clamp((P1.Y-500.f)/55.f,0.f,1.f);
                    const FVector2D Q0=P0+FMath::Lerp(NA,NB,T0).GetSafeNormal()*(Offset*S0);
                    const FVector2D Q1=P1+FMath::Lerp(NA,NB,T1).GetSafeNormal()*(Offset*S1);
                    const FVector2D Mid=(Q0+Q1)*.5f,Stroke=Q1-Q0;
                    OrientedBox(Mesh,FVector(Mid.X,Mid.Y,Mesh==Roads?.32f:.48f),
                        FVector(Stroke.Size()+.015f,Mesh==Roads?Width*Spread:Width,.025f),FMath::RadiansToDegrees(FMath::Atan2(Stroke.Y,Stroke.X)));
                }
                Along+=FMath::Max(Step,.001f);
            }
            Travel+=Length;
        }
    };
    GateStroke(Roads,0,18,false);
    GateStroke(White,-9,.18f,false); GateStroke(White,9,.18f,false);
    for(int Lane=1;Lane<4;++Lane)
        GateStroke(Lane==2?PaintYellow:White,-8+Lane*4,.16f,Lane!=2);
    const float GX=TerminalLayout::GateX,GY=TerminalLayout::GateY;
    for(int Lane=0;Lane<4;++Lane)
    {
        const float Offset=-8+Lane*4;
        const float StopX=GX+(Lane<2?-12.f:12.f)/TerminalLayout::PlanDepthScale;
        RoadPaint(White,{StopX,GY+Offset+.4f},{StopX,GY+Offset+3.6f},.45f);
    }
    // Open junction mouths on the service spine, with rounded corner returns.
    for(float Y:{395.f,445.f,513.f})
    {
        OrientedBox(Roads,FVector(TerminalLayout::SiteX(635),Y,.54f),FVector(66,22,.025f),0);
        RoadPaint(Roads,{605,Y},{635,Y},20);
        // Leave the junction mouth open; no extra arcs through the work pad.
    }
    // Lower curved entrance connects to the operations-side approach.
    // The entrance pavement and markings now share GatePath through the final connection.

    // Two-way external access between operations and CIS, branching at the bend.
    // The spur is an access road outside the terminal parcel, not extra yard area.
    RoadPaint(Roads,{797,273},{940,273},14.f);
    OrientedBox(Roads,FVector(TerminalLayout::SiteX(802),273,.55f),FVector(23,15,.025f),0);
    for(float Side:{-1.f,1.f})
        RoadPaint(White,{819,273+Side*6.8f},{940,273+Side*6.8f},.18f);
    RoadPaint(PaintYellow,{819,273},{940,273},.18f);
    // Open the operations approach onto the inland service road.
    OrientedBox(Roads,FVector(TerminalLayout::SiteX(635),225,.55f),FVector(20,16,.025f),0);
    // Main service-road edges are interrupted at the gate and empty-yard junctions.
    for(float Y=-465;Y<300;Y+=8)
    {
        if(Y<235 && Y+8>215) continue;
        if(FMath::Abs(Y-(EmptyY-25))<14 || FMath::Abs(Y-(EmptyY+25))<14 || FMath::Abs(Y-(TerminalLayout::GateY-100))<16) continue;
        for(float X:{627.f,643.f}) RoadPaint(White,{X,Y},{X,Y+8});
    }
    // One hatch group per yard end; no overlapping continuous diagonal strip.
    for(float Y=-480;Y<300;Y+=12)
    {
        if(FMath::Abs(Y-(EmptyY-25))<14 || FMath::Abs(Y-(EmptyY+25))<14 || FMath::Abs(Y-(TerminalLayout::GateY-100))<16) continue;
        RoadPaint(White,{635,Y},{635,Y+5});
    }
    for(float X=195;X<600;X+=12)
        RoadPaint(White,{X,-505},{X+5,-505});
    Label(TEXT("DGT | BUSAN NEW PORT 7 | 1,050 m"),FVector(72,-200,.4),5.f);
    SiteLogistics->Initialize(WorkingCranes,MoveTemp(YardSlots),{Blue,Yellow,Red,Green},bUnifiedTerminal?0:24,FixedYard);
    SiteLogistics->RegisterBerthVehicles(AGVActors);
    if (bUnifiedTerminal) BuildSupportFleet();
}
