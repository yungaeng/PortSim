#if WITH_DEV_AUTOMATION_TESTS
#include "DashboardLayout.h"
#include "TerminalLayout.h"
#include "Misc/AutomationTest.h"
#include "Engine/World.h"
#include "Engine/StaticMesh.h"
#include "Components/HierarchicalInstancedStaticMeshComponent.h"
#include "Dom/JsonObject.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTerminalMapLayoutTest,"PortSim.Dashboard.TerminalWorldLayout",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FTerminalMapLayoutTest::RunTest(const FString& Parameters)
{
    const auto Init=UWorld::InitializationValues().AllowAudioPlayback(false).CreatePhysicsScene(false).RequiresHitProxies(false).CreateNavigation(false).CreateAISystem(false);
    auto* World=UWorld::CreateWorld(EWorldType::Game,false,NAME_None,nullptr,true,ERHIFeatureLevel::Num,&Init);
    if(!TestNotNull(TEXT("Terminal layout world"),World))return false;
    auto* Site=World->SpawnActor<AActor>();Site->Tags.Add(TEXT("PortSim.DGT.SiteBlockout"));
    auto* Root=NewObject<USceneComponent>(Site);Site->SetRootComponent(Root);Site->AddInstanceComponent(Root);Root->RegisterComponent();
    auto* Roads=NewObject<UHierarchicalInstancedStaticMeshComponent>(Site,TEXT("Site_Roads"));Site->AddInstanceComponent(Roads);
    Roads->SetupAttachment(Root);Roads->SetStaticMesh(LoadObject<UStaticMesh>(nullptr,TEXT("/Engine/BasicShapes/Cube.Cube")));
    Roads->RegisterComponent();Roads->AddInstance(FTransform(FRotator(0,30,0),FVector(10000,-16000,24),FVector(20,16,.06)));
    auto L=DashboardLayout::Capture(World,TEXT("test-run"));
    TestEqual(TEXT("Layout belongs to the current run"),L->GetStringField(TEXT("run_id")),FString(TEXT("test-run")));
    TestEqual(TEXT("One real road instance exported"),L->GetArrayField(TEXT("pavement")).Num(),1);
    const auto Road=L->GetArrayField(TEXT("pavement"))[0]->AsObject();
    TestTrue(TEXT("Pavement uses actual world metres, without applying plan scaling twice"),
        FMath::IsNearlyEqual(Road->GetArrayField(TEXT("position_m"))[0]->AsNumber(),100.) &&
        FMath::IsNearlyEqual(Road->GetArrayField(TEXT("size_m"))[1]->AsNumber(),16.));
    TestTrue(TEXT("Road orientation comes from the actual instance"),FMath::IsNearlyEqual(Road->GetNumberField(TEXT("yaw_deg")),30.,.001));
    TestEqual(TEXT("All twenty-three blocks exported"),L->GetArrayField(TEXT("yard_blocks")).Num(),23);
    const auto Bays=L->GetArrayField(TEXT("yard_bays"));TestEqual(TEXT("Thirteen separated bay columns per yard"),Bays.Num(),23*13);
    int32 Left=0,Right=0;double LastLeft=-1,FirstRight=1.e9;
    for(const auto& Value:Bays)
    {
        const auto B=Value->AsObject();if(B->GetStringField(TEXT("yard_id"))!=TEXT("yard-1"))continue;
        const int32 Bay=B->GetIntegerField(TEXT("bay"));const double X=B->GetArrayField(TEXT("position_m"))[0]->AsNumber();
        TestTrue(TEXT("Reserved middle columns are not painted as yard slots"),Bay!=14 && Bay!=16);
        if(Bay<14){++Left;LastLeft=FMath::Max(LastLeft,X);}else{++Right;FirstRight=FMath::Min(FirstRight,X);}
    }
    TestTrue(TEXT("Seven columns and six columns preserve the large central opening"),Left==7 && Right==6 && FirstRight-LastLeft>50);
    TestTrue(TEXT("Cross-yard central road is exactly the engine's 16 m strip"),FMath::IsNearlyEqual(L->GetArrayField(TEXT("central_road_m"))[3]->AsNumber(),16.));
    Roads->ClearInstances();
    TestEqual(TEXT("Captured layout owns its footprint data after scene mutation"),L->GetArrayField(TEXT("pavement")).Num(),1);
    TestEqual(TEXT("Next capture observes the updated actual scene"),DashboardLayout::Capture(World,TEXT("next"))->GetArrayField(TEXT("pavement")).Num(),0);
    World->DestroyWorld(false);return !HasAnyErrors();
}
#endif
