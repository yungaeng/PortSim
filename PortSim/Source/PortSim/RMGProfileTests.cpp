#if WITH_DEV_AUTOMATION_TESTS
#include "STSOperatingProfile.h"
#include "TerminalLayout.h"
#include "Misc/AutomationTest.h"
#include "Misc/Paths.h"
#include "Misc/FileHelper.h"
#include "HAL/FileManager.h"
#include "Dom/JsonObject.h"
#include "Serialization/JsonSerializer.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRMGProfileTest,"PortSim.RMG.ReferenceAndLimits",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FRMGProfileTest::RunTest(const FString& Parameters)
{
    const FString File=FPaths::ProjectConfigDir()/TEXT("RMG_Simulation.json");
    FSTSOperatingProfile P;
    if(!TestTrue(TEXT("Independent RMG profile loads"),P.LoadRMG(File))){AddError(P.Error);return false;}
    // Both cranes can approach their boundary at full speed. Include both
    // stopping distances, sensor latency and the real (unscaled) bogie bodies.
    const float Stop=P.CollisionMargin+P.GantrySpeed*P.SensorMaxAge+
        P.GantrySpeed*P.GantrySpeed/(2.f*P.GantryAcceleration);
    const float Required=1962.f+2.f*Stop;
    float NearMax=TerminalLayout::SiteX(180.f)*100.f;
    float FarMin=TerminalLayout::SiteX(420.f)*100.f;
    for(int Bay=0;Bay<30;++Bay) if(TerminalLayout::IsOperationalYardBay(Bay))
    {
        if(Bay<15) NearMax=FMath::Max(NearMax,TerminalLayout::YardBayX(Bay)*100.f);
        else FarMin=FMath::Min(FarMin,TerminalLayout::YardBayX(Bay)*100.f);
    }
    TestTrue(TEXT("Scaled RMG zones retain opposing full-speed stopping clearance"),FarMin-NearMax>Required);
    TestTrue(TEXT("Regression: old bay 14 violates clearance at far handover"),
        (420.f-(205.f+14*13.f))*((1050.f*672.f/1358.f-7.f)/925.f)*100.f<Required);
    TestEqual(TEXT("RMG gauge stays physical after area expansion"),P.RailGauge,
        TerminalLayout::RMGHalfGaugeM*200.f);
    for(int Bay=0;Bay<30;++Bay) if(TerminalLayout::IsOperationalYardBay(Bay))
    {
        const int Half=Bay>=15?1:0;
        const float X=TerminalLayout::YardBayX(Bay);
        TestTrue(TEXT("Gantry bogies stay on extended rails"),
            X-TerminalLayout::RMGBogieHalfLengthM>=TerminalLayout::RMGRailStartX-.01f &&
            X+TerminalLayout::RMGBogieHalfLengthM<=TerminalLayout::RMGRailEndX+.01f);
        // Crane local X is along world Y: inland scaling changes gantry travel,
        // not the physical trolley gauge or lift envelope.
        for(int Row=0;Row<7;++Row)
            TestTrue(TEXT("Expanded yard targets stay in physical RMG work envelope"),
                P.ContainsTarget(FVector((Row-3)*300.f,
                    (TerminalLayout::RMGHomeX(Half)-X)*100.f,1081.f)));
        TestTrue(TEXT("AGV handover stays in physical RMG work envelope"),
            P.ContainsTarget(FVector(1320.f,
                (TerminalLayout::RMGHomeX(Half)-TerminalLayout::RMGHandoverX(Half))*100.f,504.f)));
    }
    TestEqual(TEXT("Selected STD capacity in kg"),P.RatedPayloadKg,40000.f);
    TestEqual(TEXT("Loaded hoist uses 30 m/min, not STS speed"),P.HoistLimit(12000,true),50.f);
    TestEqual(TEXT("Empty hoist uses 60 m/min"),P.HoistLimit(0,false),100.f);
    TestEqual(TEXT("Overload rejects motion"),P.HoistLimit(40001,true),0.f);
    TestEqual(TEXT("Selected span in cm"),P.RailGauge,3200.f);
    TestFalse(TEXT("Lift beyond reference height rejected"),P.ContainsTarget(FVector(0,0,1901)));
    TestTrue(TEXT("Stack profile installed"),P.SensorKeys.Contains(TEXT("stack_profile")));
    TestFalse(TEXT("No RTG magnetic steering"),P.SensorKeys.Contains(TEXT("auto_steering")));
    FString Text;FFileHelper::LoadFileToString(Text,*File);TSharedPtr<FJsonObject> Root;
    FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text),Root);
    const FString Dir=FPaths::ProjectSavedDir()/TEXT("Tests/RMG");IFileManager::Get().MakeDirectory(*Dir,true);
    const FString Invalid=Dir/TEXT("invalid.json");
    auto Reject=[&](const TCHAR* Name)
    {
        FString Json;FJsonSerializer::Serialize(Root.ToSharedRef(),TJsonWriterFactory<>::Create(&Json));
        FFileHelper::SaveStringToFile(Json,*Invalid);FSTSOperatingProfile Bad;
        TestFalse(Name,Bad.LoadRMG(Invalid));TestFalse(TEXT("Invalid profile never becomes ready"),Bad.bReady);
    };
    Root->SetNumberField(TEXT("loaded_hoist_m_min"),87);Reject(TEXT("STS loaded speed must not enter RMG config"));
    Root->SetNumberField(TEXT("loaded_hoist_m_min"),30);
    Root->SetNumberField(TEXT("sensor_max_age_s"),.001);Reject(TEXT("Observation expiry shorter than sampling rejected"));
    Root->SetNumberField(TEXT("sensor_max_age_s"),.15);
    auto Mounts=Root->GetArrayField(TEXT("sensor_mounts"));
    Mounts.RemoveAll([](const TSharedPtr<FJsonValue>& V){return V->AsObject()->GetStringField(TEXT("key"))==TEXT("stack_profile");});
    Root->SetArrayField(TEXT("sensor_mounts"),Mounts);Reject(TEXT("Missing stack observation cannot silently fall back"));
    return true;
}
#endif
