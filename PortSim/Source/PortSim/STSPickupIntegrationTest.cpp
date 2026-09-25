#include "QuayCrane.h"
#include "PortWorkingCrane.h"
#include "PortContainerActor.h"
#include "Components/StaticMeshComponent.h"
#include "Dom/JsonObject.h"
#include "Engine/World.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"

void AQuayCrane::BeginPickupTest()
{
    bPickupTest=true;PickupTestCase=TEXT("offset");
    FParse::Value(FCommandLine::Get(),TEXT("PortSimPickupCase="),PickupTestCase);
    if(!STSProfile.bReady){UE_LOG(LogTemp,Error,TEXT("PORTSIM_PICKUP_FAIL: invalid profile"));FPlatformMisc::RequestExitWithStatus(false,1);return;}
    auto Profile=STSProfile;
    if(PickupTestCase==TEXT("bias"))Profile.Pickup.PoseBias.X=15;
    const FVector Home(500000,0,0), Source=Home+FVector(-1600,0,1700), Destination=Home+FVector(3000,0,149.5);
    FActorSpawnParameters Spawn;Spawn.Owner=this;Spawn.SpawnCollisionHandlingOverride=ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
    PickupTestCrane=GetWorld()->SpawnActor<APortWorkingCrane>(Home,FRotator::ZeroRotator,Spawn);
    WorkingCranes.Add(PickupTestCrane);PickupTestCrane->SetSTSProfile(Profile);
    PickupTestCrane->Configure(1,true,Source,Destination,false);
    const FRotator Rotation(0,PickupTestCase==TEXT("yaw")?1.5:0,0);
    auto* Box=GetWorld()->SpawnActor<APortContainerActor>(Source+FVector(30,-20,0),Rotation,Spawn);
    Box->InitializeContainer(9000);ContainerActors.Add(Box);
    Box->SetPhysicalParameters(PickupTestCase==TEXT("eccentric")?24000:12000,PickupTestCase==TEXT("eccentric")?FVector(25,-80,0):FVector::ZeroVector);
    Box->GetBody()->SetSimulatePhysics(false);Box->LocationOwner=ECargoOwner::Ship;
    if(!PickupTestCrane->AssignCargo(Box,Source,Destination,false,false))
    {UE_LOG(LogTemp,Error,TEXT("PORTSIM_PICKUP_FAIL: assignment rejected"));FPlatformMisc::RequestExitWithStatus(false,1);return;}
    PickupTestCrane->SetDestinationReady(false);
}

void AQuayCrane::TickPickupTest(float Dt)
{
    if(!PickupTestCrane)return;
    PickupTestSeconds+=Dt;auto* C=PickupTestCrane.Get();
    const bool WasAttached=C->bCarrying;const FVector Before=C->CargoActor->GetActorLocation();
    C->Advance(Dt,false);
    auto Finish=[&](bool Pass,const FString& Why)
    {
        UE_LOG(LogTemp,Display,TEXT("PORTSIM_PICKUP_%s: %s: %s (%.2f simulated seconds)"),Pass?TEXT("PASS"):TEXT("FAIL"),*PickupTestCase,*Why,PickupTestSeconds);
        FPlatformMisc::RequestExitWithStatus(false,Pass?0:1);
    };
    const bool Negative=PickupTestCase!=TEXT("offset")&&PickupTestCase!=TEXT("eccentric");
    if(!WasAttached&&C->bCarrying && (!C->Observation.AllLocked()||!Before.Equals(C->CargoActor->GetActorLocation(),.01)))
    {Finish(false,TEXT("Attachment bypassed feedback or teleported cargo"));return;}
    if(Negative&&C->bCarrying){Finish(false,TEXT("Invalid pickup attached cargo"));return;}
    bPickupSawTrial |= C->bCarrying&&C->Stage==2;
    if(!C->Fault.IsEmpty())
    {
        const FString Expected=PickupTestCase==TEXT("lock")?TEXT("Twist lock alignment failed"):TEXT("Pickup alignment attempts exhausted");
        Finish(Negative&&C->Fault==Expected,C->Fault);return;
    }
    if(C->Stage>=3)
    {
        const auto Pick=C->DashboardState()->GetObjectField(TEXT("pickup"));
        const double Estimated=Pick->GetNumberField(TEXT("estimated_mass_kg"));
        const auto CoG=Pick->GetArrayField(TEXT("estimated_cog_m"));
        const FVector MeasuredCoG(CoG[0]->AsNumber()*100,CoG[1]->AsNumber()*100,0);
        const bool Pass=!Negative&&bPickupSawTrial&&Pick->GetBoolField(TEXT("verified"))&&
            FMath::Abs(Estimated-C->CargoActor->MassKg)<50&&MeasuredCoG.Equals(C->CargoActor->CoGOffsetCm,3);
        Finish(Pass,FString::Printf(TEXT("measured mass %.2f kg, CoG %s; offset corrected and trial lift verified"),Estimated,*MeasuredCoG.ToCompactString()));return;
    }
    if(PickupTestSeconds>350)Finish(false,TEXT("Scenario timed out"));
}
