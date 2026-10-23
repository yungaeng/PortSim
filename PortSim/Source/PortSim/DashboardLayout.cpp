#include "DashboardLayout.h"
#include "TerminalLayout.h"
#include "Components/HierarchicalInstancedStaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "Dom/JsonObject.h"

namespace
{
    TArray<TSharedPtr<FJsonValue>> Numbers(std::initializer_list<double> Values)
    {TArray<TSharedPtr<FJsonValue>> A;for(double V:Values)A.Add(MakeShared<FJsonValueNumber>(V));return A;}
    TSharedRef<FJsonValueArray> Point(double X,double Y)
    {return MakeShared<FJsonValueArray>(Numbers({X,Y}));}
}

TSharedRef<FJsonObject> DashboardLayout::Footprint(const FTransform& T,FVector MeshSize)
{
    auto R=MakeShared<FJsonObject>();const FVector Size=MeshSize*T.GetScale3D().GetAbs()*.01,P=T.GetLocation()*.01;
    R->SetArrayField(TEXT("position_m"),Numbers({P.X,P.Y}));R->SetArrayField(TEXT("size_m"),Numbers({Size.X,Size.Y}));
    R->SetNumberField(TEXT("yaw_deg"),T.Rotator().Yaw);return R;
}

TSharedRef<FJsonObject> DashboardLayout::Footprint(const UStaticMeshComponent* C)
{return Footprint(C->GetComponentTransform(),C->GetStaticMesh()?C->GetStaticMesh()->GetBounds().BoxExtent*2:FVector(100));}

TUniquePtr<FJsonObject> DashboardLayout::Capture(UWorld* World,const FString& Run)
{
    auto Root=MakeUnique<FJsonObject>();Root->SetNumberField(TEXT("schema_version"),1);Root->SetStringField(TEXT("run_id"),Run);
    Root->SetStringField(TEXT("basis"),TEXT("engine_world_metres; actual instanced scenery and TerminalLayout slot markings"));
    Root->SetStringField(TEXT("axes"),TEXT("X inland, Y along quay; equal scale on both axes; not a surveyed georeference"));
    TArray<TSharedPtr<FJsonValue>> Boundary;
    Boundary.Add(Point(TerminalLayout::QuayLeftX*.01,-525));
    for(const auto& P:TerminalLayout::SiteBoundaryProfile)Boundary.Add(Point(TerminalLayout::SiteX(P[0]),P[1]));
    Boundary.Add(Point(TerminalLayout::QuayLeftX*.01,645));Root->SetArrayField(TEXT("boundary_m"),Boundary);
    Root->SetArrayField(TEXT("bounds_m"),Numbers({-75,-540,TerminalLayout::SiteX(950),660}));
    TArray<TSharedPtr<FJsonValue>> Roads,Markings,Buildings;
    if(World)for(TActorIterator<AActor> It(World);It;++It)if(It->ActorHasTag(TEXT("PortSim.DGT.SiteBlockout")))
    {
        TInlineComponentArray<UHierarchicalInstancedStaticMeshComponent*> Meshes(*It);
        for(const auto* Mesh:Meshes)
        {
            const FString Name=Mesh->GetName();auto* Output=Name==TEXT("Site_Roads")?&Roads:
                (Name==TEXT("Site_Markings") || Name==TEXT("Site_YellowMarkings"))?&Markings:Name==TEXT("Site_Buildings")?&Buildings:nullptr;
            if(!Output || !Mesh->GetStaticMesh())continue;
            const FVector Size=Mesh->GetStaticMesh()->GetBounds().BoxExtent*2;
            for(int32 I=0;I<Mesh->GetInstanceCount();++I)
            {
                FTransform T;if(!Mesh->GetInstanceTransform(I,T,true))continue;
                const FVector ActualSize=Size*T.GetScale3D().GetAbs()*.01;
                if(Output==&Buildings && (ActualSize.Z<2 || ActualSize.X*ActualSize.Y<25))continue;
                auto R=Footprint(T,Size);R->SetStringField(TEXT("material"),Name==TEXT("Site_YellowMarkings")?TEXT("yellow"):TEXT("white"));
                Output->Add(MakeShared<FJsonValueObject>(R));
            }
        }
    }
    Root->SetArrayField(TEXT("pavement"),Roads);Root->SetArrayField(TEXT("markings"),Markings);Root->SetArrayField(TEXT("buildings"),Buildings);
    TArray<TSharedPtr<FJsonValue>> Bays,Blocks;
    for(int32 B=0;B<TerminalLayout::YardBlockCount;++B)
    {
        const double Y=TerminalLayout::BlockY(B);auto Block=MakeShared<FJsonObject>();
        Block->SetStringField(TEXT("id"),FString::Printf(TEXT("yard-%d"),B+1));Block->SetNumberField(TEXT("number"),B+1);
        Block->SetArrayField(TEXT("label_position_m"),Numbers({TerminalLayout::SiteX(172),Y}));Blocks.Add(MakeShared<FJsonValueObject>(Block));
        for(int32 Bay=0;Bay<30;Bay+=2)if(TerminalLayout::IsOperationalYardBay(Bay))
        {
            auto R=MakeShared<FJsonObject>();const int32 Ft=TerminalLayout::YardSizeFt(Bay);
            R->SetStringField(TEXT("yard_id"),FString::Printf(TEXT("yard-%d"),B+1));R->SetNumberField(TEXT("bay"),Bay);
            R->SetNumberField(TEXT("length_ft"),Ft);R->SetNumberField(TEXT("rows"),7);R->SetNumberField(TEXT("pads"),TerminalLayout::YardPads(Ft));
            R->SetArrayField(TEXT("position_m"),Numbers({TerminalLayout::YardBayX(Bay),Y}));
            R->SetArrayField(TEXT("size_m"),Numbers({Ft==20?12.192:TerminalLayout::YardLengthCm(Ft)*.01,21}));
            Bays.Add(MakeShared<FJsonValueObject>(R));
        }
    }
    Root->SetArrayField(TEXT("yard_bays"),Bays);Root->SetArrayField(TEXT("yard_blocks"),Blocks);
    Root->SetArrayField(TEXT("central_road_m"),Numbers({TerminalLayout::SiteX(145),TerminalLayout::CentralRoadY,
        TerminalLayout::SiteX(635),TerminalLayout::CentralRoadWidthM}));
    Root->SetArrayField(TEXT("yard_view_m"),Numbers({TerminalLayout::SiteX(165),-515,TerminalLayout::SiteX(660),360}));
    Root->SetArrayField(TEXT("quay_view_m"),Numbers({-65,-525,TerminalLayout::SiteX(178),350}));
    Root->SetArrayField(TEXT("depot_view_m"),Numbers({TerminalLayout::SiteX(30),345,TerminalLayout::SiteX(170),650}));
    Root->SetArrayField(TEXT("vessel_size_m"),Numbers({45,TerminalLayout::VesselLengthM}));
    return Root;
}
