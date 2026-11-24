#include "PortShipActor.h"
#include "TerminalLayout.h"
#include "Components/StaticMeshComponent.h"
#include "Components/TextRenderComponent.h"
#include "Engine/StaticMesh.h"
#include "UObject/ConstructorHelpers.h"

APortShipActor::APortShipActor()
{
    Box(TEXT("ShipHull"),RootComponent,FVector(-1400,0,-95),FVector(4500,28200,550),true);
    Box(TEXT("ShipDeck"),RootComponent,FVector(-1400,0,190),FVector(4400,28000,20),true);
    Box(TEXT("ShipBow1"),RootComponent,FVector(-1400,14350,-95),FVector(3400,500,550),true);
    Box(TEXT("ShipBow2"),RootComponent,FVector(-1400,14730,-95),FVector(2200,260,550),true);
    Box(TEXT("ShipBow3"),RootComponent,FVector(-1400,14930,-95),FVector(1000,180,550),true);
    Box(TEXT("ShipStern"),RootComponent,FVector(-1400,-14390,-95),FVector(4000,580,550),true);
    Box(TEXT("ShipBridge"),RootComponent,FVector(-1400,-14350,1230),FVector(3800,500,2100),true);
    Box(TEXT("ShipBridgeWindows"),RootComponent,FVector(-1400,-14350,2200),FVector(3900,550,110),true);
    Box(TEXT("ShipFunnel"),RootComponent,FVector(-1650,-14350,2480),FVector(600,500,700),true);
    Tags.Add(TEXT("PortSim.Ship"));
}
