#pragma once
#include "CoreMinimal.h"
#include "AGVReference.h"

namespace AGVTrafficGeometry
{
    // Connect parallel lanes with two opposite constant-radius arcs. Two
    // independent right-angle fillets overlap when the lane offset is < 2R.
    inline bool LaneChange(FVector A,FVector B,FVector C,FVector D,TArray<FVector>& Points)
    {
        const float R=AGVReference::MinimumInnerTurnRadiusCm;
        const FVector In=(B-A).GetSafeNormal2D(),Side=(C-B).GetSafeNormal2D();
        const FVector Out=(D-C).GetSafeNormal2D();
        const float Width=FVector::Dist2D(B,C);
        if(In.IsNearlyZero() || FVector::DotProduct(In,Out)<.999f ||
            FMath::Abs(FVector::DotProduct(In,Side))>.001f || Width>=2*R || Width<1.f) return false;
        const float Theta=FMath::Acos(1.f-Width/(2*R));
        const float Run=R*FMath::Sin(Theta);
        if(FVector::Dist2D(A,B)<Run+100.f || FVector::Dist2D(C,D)<Run+R+100.f) return false;
        const FVector Start=B-In*Run;
        Points.Add(Start);
        const int32 Steps=FMath::Max(2,FMath::CeilToInt(Theta/FMath::DegreesToRadians(22.5f)));
        for(int32 I=1;I<=Steps;++I)
        {
            const float T=Theta*float(I)/Steps;
            Points.Add(Start+In*(R*FMath::Sin(T))+Side*(R*(1.f-FMath::Cos(T))));
        }
        const FVector Middle=Points.Last();
        for(int32 I=1;I<=Steps;++I)
        {
            const float T=Theta*(1.f-float(I)/Steps);
            Points.Add(Middle+In*(R*(FMath::Sin(Theta)-FMath::Sin(T)))+
                Side*(R*(FMath::Cos(T)-FMath::Cos(Theta))));
        }
        return true;
    }

    // Narrow phase for a rectangular chassis swept along a road chord. AABB
    // queries are only a broad phase: at a turn their empty corners must not
    // block a vehicle in an adjacent parking bay.
    inline bool SweptChassisIntersects(FVector From,FVector To,FVector OtherPosition,
        FVector OtherLongitudinal,float HalfLength=AGVReference::PhysicalHalfLengthCm,
        float HalfWidth=AGVReference::PhysicalHalfWidthCm,float TurnMargin=0.f)
    {
        FVector Long=(To-From).GetSafeNormal2D();
        if(Long.IsNearlyZero()) Long=OtherLongitudinal.GetSafeNormal2D();
        const FVector Side(-Long.Y,Long.X,0);
        const FVector OtherLong=OtherLongitudinal.GetSafeNormal2D();
        const FVector OtherSide(-OtherLong.Y,OtherLong.X,0);
        const FVector Delta=OtherPosition-(From+To)*.5;
        const double SweepHalfLength=FVector::Dist2D(From,To)*.5+HalfLength;
        const FVector Axes[]={Long,Side,OtherLong,OtherSide};
        for(const FVector& Axis:Axes)
        {
            const double Own=FMath::Abs(FVector::DotProduct(Long,Axis))*SweepHalfLength+
                FMath::Abs(FVector::DotProduct(Side,Axis))*(HalfWidth+TurnMargin);
            const double Other=FMath::Abs(FVector::DotProduct(OtherLong,Axis))*HalfLength+
                FMath::Abs(FVector::DotProduct(OtherSide,Axis))*HalfWidth;
            if(FMath::Abs(FVector::DotProduct(Delta,Axis))>Own+Other) return false;
        }
        return true;
    }
}
