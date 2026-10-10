#pragma once
#include "CoreMinimal.h"

namespace TrafficJunctionPolicy
{
    // Reserve 60 m and continue through coupled corners to a straight exit.
    // The exit must have a 20 m tangent beyond the last turn, so stopping at
    // its gate leaves the complete chassis clear of the intersection.
    inline void AppendExitCorridor(TArray<FBox>& Segments,const TArray<FVector>& Route,
        int32 Waypoint,int32 RouteEnd,TFunctionRef<FVector(int32)> ExtentFor)
    {
        if(!Route.IsValidIndex(Waypoint)) return;
        RouteEnd=FMath::Clamp(RouteEnd,Waypoint+1,Route.Num());
        FVector From=Route[Waypoint];
        double Length=0,SinceTurn=0;
        for(int32 Next=Waypoint+1;Next<RouteEnd;++Next)
        {
            const FVector Extent=ExtentFor(Next);
            FBox Exit(From-Extent,From+Extent);
            Exit+=Route[Next]-Extent; Exit+=Route[Next]+Extent;
            Segments.Add(Exit);
            const FVector Direction=(Route[Next]-From).GetSafeNormal2D();
            const double StepLength=FVector::Dist2D(From,Route[Next]);
            Length+=StepLength;
            const FVector Previous=(From-(Next>1?Route[Next-2]:From)).GetSafeNormal2D();
            if(!Previous.IsNearlyZero() && FVector::DotProduct(Previous,Direction)<.995f) SinceTurn=0;
            SinceTurn+=StepLength;
            From=Route[Next];
            if(Next+1<RouteEnd && Length>=6000.f && SinceTurn>=2000.f)
            {
                const FVector After=(Route[Next+1]-From).GetSafeNormal2D();
                if(FVector::DotProduct(Direction,After)>.995f) break;
            }
        }
    }

    inline bool OlderRequestFirst(const double* A,const double* B,int32 IDA,int32 IDB)
    {
        if(bool(A)!=bool(B)) return A!=nullptr;
        if(A && B && *A!=*B) return *A<*B;
        return IDA<IDB;
    }
}
