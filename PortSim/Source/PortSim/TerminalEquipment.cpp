#include "TerminalLayout.h"
#include "QuayCrane.h"
#include "PortSupportVehicle.h"
#include "PortSiteLogistics.h"
#include "PortWorkingCrane.h"
#include "PortAGVActor.h"
#include "Engine/World.h"

void AQuayCrane::BuildSupportFleet()
{
    FActorSpawnParameters Params; Params.Owner=this;
    Params.SpawnCollisionHandlingOverride=ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
    auto Spawn=[&](EPortSupportType Type,int32 Number,FVector Metres)
    {
        Metres.X=TerminalLayout::SiteX(Metres.X);
        auto* Vehicle=GetWorld()->SpawnActor<APortSupportVehicle>(Metres*100,FRotator::ZeroRotator,Params);
        Vehicle->Configure(Type,Number); SupportFleet.Add(Vehicle);
    };
    // Support fleets are parked in the matching plan zones; simulation-owned
    // STS/RMG/AGV actors retain their operational handover coordinates.
    for (int32 I=0;I<74;++I)
        Spawn(EPortSupportType::YardChassis,I+1,FVector(200+(I/15)*18,425+(I%15)*3.2f,0));
    for (int32 I=0;I<18;++I)
        Spawn(EPortSupportType::YardTractor,I+1,FVector(620+(I/6)*14,330+(I%6)*20,0));
    for (int32 I=0;I<4;++I)
        Spawn(EPortSupportType::ReachStacker,I+1,FVector(875,315+I*32,0));
    for (int32 I=0;I<2;++I)
        Spawn(EPortSupportType::EmptyHandler,I+1,FVector(575,90+I*38,0));
    for (int32 I=0;I<7;++I)
        Spawn(EPortSupportType::Forklift,I+1,FVector(470+(I%2)*13,330+(I/2)*18,0));
    UE_LOG(LogTemp,Display,TEXT("EQUIPMENT_INVENTORY: CC=9 (24 rows), TC=36, AGV=60 active, RS=4, YT=18, EH=2, FL=7, YC=74; total=210"));
}
