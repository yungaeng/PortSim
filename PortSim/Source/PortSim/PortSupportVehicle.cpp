#include "PortSupportVehicle.h"
#include "Components/StaticMeshComponent.h"
#include "Components/TextRenderComponent.h"
#include "Materials/MaterialInterface.h"
#include "Engine/StaticMesh.h"

void APortSupportVehicle::Configure(EPortSupportType Type,int32 Number)
{
    EquipmentType=Type; EquipmentNumber=Number;
    const TCHAR* Codes[]={TEXT("RS"),TEXT("YT"),TEXT("EH"),TEXT("FL"),TEXT("YC")};
    const FString Code=Codes[static_cast<int32>(Type)];
    const bool Chassis=Type==EPortSupportType::YardChassis;
    const bool Small=Type==EPortSupportType::Forklift;
    const float Length=Chassis?1250.f:Small?340.f:Type==EPortSupportType::YardTractor?550.f:
        Type==EPortSupportType::EmptyHandler?700.f:900.f;
    const float Width=Small?190.f:Chassis?250.f:Type==EPortSupportType::YardTractor?250.f:
        Type==EPortSupportType::EmptyHandler?350.f:320.f;
    auto* Paint=LoadObject<UMaterialInterface>(nullptr,TEXT("/Game/PortSim/Assets/Materials/M_SiteOrange.M_SiteOrange"));
    auto* Steel=LoadObject<UMaterialInterface>(nullptr,TEXT("/Game/PortSim/Assets/Materials/M_Steel.M_Steel"));
    auto Part=[&](const TCHAR* Name,FVector P,FVector S,bool Dark=false)
    {
        auto* Mesh=NewObject<UStaticMeshComponent>(this,Name);
        AddInstanceComponent(Mesh); Mesh->SetupAttachment(RootComponent);
        Mesh->SetStaticMesh(LoadObject<UStaticMesh>(nullptr,TEXT("/Engine/BasicShapes/Cube.Cube")));
        Mesh->SetRelativeLocation(P); Mesh->SetRelativeScale3D(S/100.f);
        Mesh->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
        Mesh->SetMaterial(0,Dark?Steel:Paint);
        Mesh->RegisterComponent();
        return Mesh;
    };
    Part(TEXT("Frame"),FVector(0,0,110),FVector(Length,Width,Chassis?35:100));
    for (int32 Side:{-1,1}) for (int32 Axle:{-1,1})
        Part(*FString::Printf(TEXT("Wheel_%d_%d"),Side,Axle),FVector(Axle*Length*.32f,Side*Width*.48f,65),FVector(110,60,120),true);
    if (Chassis)
    {
        Part(TEXT("Neck"),FVector(Length*.45f,0,130),FVector(150,90,35));
        for (int32 X:{-1,1}) for (int32 Y:{-1,1})
            Part(*FString::Printf(TEXT("Twistlock_%d_%d"),X,Y),FVector(X*570,Y*110,145),FVector(35,35,40),true);
    }
    else
    {
        Part(TEXT("Cab"),FVector(Length*.15f,0,245),FVector(Small?130:230,Width*.75f,180));
        Part(TEXT("Windscreen"),FVector(Length*.15f+(Small?66:116),0,270),FVector(5,Width*.65f,90),true);
        if (Type==EPortSupportType::ReachStacker)
        {
            auto* Boom=Part(TEXT("TelescopicBoom"),FVector(150,0,470),FVector(900,100,100));
            Boom->SetRelativeRotation(FRotator(25,0,0));
            Part(TEXT("Spreader"),FVector(540,0,650),FVector(240,1220,50),true);
        }
        else if (Type==EPortSupportType::EmptyHandler || Small)
        {
            const float Height=Small?450.f:1100.f;
            for (int32 Side:{-1,1})
                Part(*FString::Printf(TEXT("Mast_%d"),Side),FVector(Length*.48f,Side*70,Height*.5f+100),FVector(55,40,Height),true);
            if (Small)
                for (int32 Side:{-1,1}) Part(*FString::Printf(TEXT("Fork_%d"),Side),FVector(Length*.65f,Side*55,60),FVector(180,25,20),true);
            else Part(TEXT("EmptySpreader"),FVector(Length*.55f,0,620),FVector(130,1220,60));
        }
    }
    auto* Name=NewObject<UTextRenderComponent>(this,TEXT("EquipmentID"));
    AddInstanceComponent(Name); Name->SetupAttachment(RootComponent);
    Name->SetRelativeLocation(FVector(0,0,Chassis?190:360));
    Name->SetRelativeRotation(FRotator(90,0,0));
    Name->SetHorizontalAlignment(EHTA_Center); Name->SetWorldSize(100);
    Name->RegisterComponent();
    Name->SetText(FText::FromString(FString::Printf(TEXT("%s %02d"),*Code,Number)));
    Tags.Add(FName(*FString::Printf(TEXT("PortSim.%s"),*Code)));
#if WITH_EDITOR
    SetActorLabel(FString::Printf(TEXT("%s_%02d"),*Code,Number));
    SetFolderPath(TEXT("PortSim/SupportEquipment"));
#endif
}
