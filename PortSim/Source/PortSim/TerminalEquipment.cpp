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
        auto* Vehicle=GetWorld()->SpawnActor<APortSupportVehicle>(Metres*100,FRotator::ZeroRotator,Params);
        Vehicle->Configure(Type,Number); SupportFleet.Add(Vehicle);
    };
    // Support fleets are parked in the matching plan zones; simulation-owned
    // STS/RMG/AGV actors retain their operational handover coordinates.
    for (int32 I=0;I<74;++I)
        Spawn(EPortSupportType::YardChassis,I+1,FVector(220+(I/15)*15,335+(I%15)*10.5f,0));
    for (int32 I=0;I<18;++I)
        Spawn(EPortSupportType::YardTractor,I+1,FVector(675+(I/6)*15,350+(I%6)*18,0));
    for (int32 I=0;I<4;++I)
        Spawn(EPortSupportType::ReachStacker,I+1,FVector(775,355+I*27,0));
    for (int32 I=0;I<2;++I)
        Spawn(EPortSupportType::EmptyHandler,I+1,FVector(575,95+I*42,0));
    for (int32 I=0;I<7;++I)
        Spawn(EPortSupportType::Forklift,I+1,FVector(285+(I%2)*13,350+(I/2)*16,0));
    UE_LOG(LogTemp,Display,TEXT("EQUIPMENT_INVENTORY: CC=9 (24 rows), TC=46, AGV=60 active, RS=4, YT=18, EH=2, FL=7, YC=74; total=220"));
}
