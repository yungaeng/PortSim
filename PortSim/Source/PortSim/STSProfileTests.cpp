#if WITH_DEV_AUTOMATION_TESTS
#include "STSOperatingProfile.h"
#include "Misc/AutomationTest.h"
#include "Misc/Paths.h"
#include "Misc/FileHelper.h"
#include "HAL/FileManager.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSTSProfileTest,"PortSim.STS.ReferenceAndInterlocks",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)

bool FSTSProfileTest::RunTest(const FString& Parameters)
{
    FString Ref=FPaths::ConvertRelativePathToFull(FPaths::ProjectDir()/TEXT("../Document/STS/STS_ReferenceData.json"));
    if(!FPaths::FileExists(Ref)) Ref=FPaths::ProjectContentDir()/TEXT("STS/STS_ReferenceData.json");
    const FString Settings=FPaths::ProjectConfigDir()/TEXT("STS_Simulation.json");
    FSTSOperatingProfile P;
    if(!TestTrue(TEXT("Reference and assumptions load"),P.Load(Ref,Settings))) { AddError(P.Error); return false; }
    TestEqual(TEXT("Gauge converted to cm"),P.RailGauge,3048.f);
    TestEqual(TEXT("Trolley speed converted independently"),P.TrolleySpeed,400.f);
    TestEqual(TEXT("Gantry speed converted independently"),P.GantrySpeed,100.f);
    TestTrue(TEXT("Rated loaded curve point"),FMath::IsNearlyEqual(P.HoistLimit(P.RatedPayloadKg,true),53.f*100/60,0.02f));
    TestTrue(TEXT("Low mass does not get empty-spreader speed"),FMath::IsNearlyEqual(P.HoistLimit(12000,true),145.f,0.02f));
    TestTrue(TEXT("Empty-spreader condition"),FMath::IsNearlyEqual(P.HoistLimit(0,false),170.f*100/60,0.02f));
    TestTrue(TEXT("Heavier loads reduce actual hoist command limit"),P.HoistLimit(60000,true)<P.HoistLimit(30000,true));
    TestEqual(TEXT("Overload cannot drive"),P.HoistLimit(P.RatedPayloadKg+1,true),0.f);
    TestEqual(TEXT("Negative mass cannot drive"),P.HoistLimit(-1,true),0.f);
    TestFalse(TEXT("Outreach rejects unreachable target"),P.ContainsTarget(FVector(P.MinTrolley()-1,0,1000)));
    TestFalse(TEXT("Lift envelope rejects high target"),P.ContainsTarget(FVector(0,0,P.LiftAboveRail+1)));
    TestEqual(TEXT("Sensor reference retained"),P.SensorKeys.Num(),14);
    TestTrue(TEXT("Loaded interpolation at 45 LT"),FMath::IsNearlyEqual(P.HoistLimit(45*P.LoadUnitKg,true),64.5f*100/60,0.02f));

    FSTSObservation S;
    TestFalse(TEXT("Default sample is invalid"),S.IsFresh(0,1));
    S.bValid=true; S.Timestamp=2;
    TestTrue(TEXT("Current sample accepted"),S.IsFresh(2.1,0.15));
    TestFalse(TEXT("Stale sample rejected"),S.IsFresh(2.2,0.15));
    TestFalse(TEXT("Future timestamp rejected"),S.IsFresh(1,0.15));
    for(bool& Lock:S.Locked) Lock=true;
    TestTrue(TEXT("Four locks required"),S.AllLocked()); S.Locked[2]=false;
    TestFalse(TEXT("One failed lock rejects loaded hoist"),S.AllLocked());

    FString SettingsText,RefText;
    FFileHelper::LoadFileToString(SettingsText,*Settings); FFileHelper::LoadFileToString(RefText,*Ref);
    const FString Dir=FPaths::ProjectSavedDir()/TEXT("Tests/STS");
    IFileManager::Get().MakeDirectory(*Dir,true);
    const FString Bad=Dir/TEXT("invalid.json");
    auto Write=[&](const FString& Text){return FFileHelper::SaveStringToFile(Text,*Bad);};
    Write(TEXT("{}"));
    TestFalse(TEXT("Missing reference fields fail closed"),P.Load(Bad,Settings));
    TestFalse(TEXT("Failed reload clears ready state"),P.bReady);
    Write(SettingsText.Replace(TEXT("\"sensor_sample_hz\": 20"),TEXT("\"sensor_sample_hz\": 0")));
    TestFalse(TEXT("Zero sample frequency rejected"),P.Load(Ref,Bad));
    Write(SettingsText.Replace(TEXT("\"sensor_max_age_s\": 0.15"),TEXT("\"sensor_max_age_s\": 0.001")));
    TestFalse(TEXT("Inconsistent timing rejected"),P.Load(Ref,Bad));
    Write(RefText.Replace(TEXT("\"key\": \"landed\""),TEXT("\"key\": \"missing_landed\"")));
    TestFalse(TEXT("Missing required sensor rejects automation profile"),P.Load(Bad,Settings));
    TestFalse(TEXT("Missing file rejects profile"),P.Load(Dir/TEXT("missing.json"),Settings));
    return true;
}
#endif
