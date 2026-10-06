#pragma once

#include "CoreMinimal.h"
#include "ContainerSpecification.generated.h"

UENUM(BlueprintType)
enum class EPortContainerSize : uint8
{
    Ft20 UMETA(DisplayName="20 ft"),
    Ft40 UMETA(DisplayName="40 ft"),
    Ft45 UMETA(DisplayName="45 ft")
};

struct FPortContainerSpecification
{
    EPortContainerSize Size=EPortContainerSize::Ft40;
    int32 LengthFt=40;
    float LengthCm=1219.2f;
    float WidthCm=243.8f;
    float HeightCm=259.1f;
    float TareMassKg=3800.f;
    float CargoMassKg=12000.f;

    float GrossMassKg() const { return TareMassKg+CargoMassKg; }
};

namespace PortContainerSpecification
{
    // Deterministic mixed manifest: 30% 20 ft, 50% 40 ft and 20% 45 ft.
    // Masses are representative simulation inputs, not a claim that size fixes weight.
    inline FPortContainerSpecification ForSequence(int32 Sequence)
    {
        // Mix adjacent STS streams as well as the full manifest while preserving
        // the exact 30/50/20 distribution in every 600-container site batch.
        const int32 Bucket=FMath::Abs(Sequence+(Sequence/6)*3)%10;
        FPortContainerSpecification Result;
        if(Bucket==0 || Bucket==4 || Bucket==7)
        {
            Result.Size=EPortContainerSize::Ft20; Result.LengthFt=20; Result.LengthCm=605.8f;
            Result.TareMassKg=2300.f; Result.CargoMassKg=7000.f+1000.f*(FMath::Abs(Sequence*7)%9);
        }
        else if(Bucket==3 || Bucket==8)
        {
            Result.Size=EPortContainerSize::Ft45; Result.LengthFt=45; Result.LengthCm=1371.6f;
            Result.TareMassKg=4800.f; Result.CargoMassKg=14000.f+1500.f*(FMath::Abs(Sequence*5)%8);
        }
        else
        {
            Result.Size=EPortContainerSize::Ft40; Result.LengthFt=40; Result.LengthCm=1219.2f;
            Result.TareMassKg=3800.f; Result.CargoMassKg=12000.f+1500.f*(FMath::Abs(Sequence*3)%9);
        }
        return Result;
    }
}
