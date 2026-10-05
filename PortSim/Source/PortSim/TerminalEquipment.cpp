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
        Metres.X=TerminalLayout::SiteX(Metres.X); Metres.Y+=300.f;
        auto* Vehicle=GetWorld()->SpawnActor<APortSupportVehicle>(Metres*100,FRotator::ZeroRotator,Params);
        Vehicle->Configure(Type,Number); SupportFleet.Add(Vehicle);
    };
    // Support fleets are parked in the matching plan zones; simulation-owned
    // STS/RMG/AGV actors retain their operational handover coordinates.
    for (int32 I=0;I<74;++I)
        Spawn(EPortSupportType::YardChassis,I+1,FVector(700+(I/19)*28,115+(I%19)*3.6f,0));
    for (int32 I=0;I<18;++I)
        Spawn(EPortSupportType::YardTractor,I+1,FVector(703+(I/9)*18,78+(I%9)*4.f,0));
    for (int32 I=0;I<4;++I)
        Spawn(EPortSupportType::ReachStacker,I+1,FVector(810,133+I*15,0));
    for (int32 I=0;I<2;++I)
        Spawn(EPortSupportType::EmptyHandler,I+1,FVector(695,20+I*20,0));
    for (int32 I=0;I<7;++I)
        Spawn(EPortSupportType::Forklift,I+1,FVector(695,55+I*4,0));
    UE_LOG(LogTemp,Display,TEXT("EQUIPMENT_INVENTORY: CC=9 (24 rows), TC=46, AGV=60 active, RS=4, YT=18, EH=2, FL=7, YC=74; total=220"));
}
