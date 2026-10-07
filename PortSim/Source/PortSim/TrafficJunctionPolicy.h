#pragma once
#include "CoreMinimal.h"

namespace TrafficJunctionPolicy
{
    // Reserve the exit as well as the approach. A bend must not count towards
    // clearance until the vehicle centre is physically clear of the junction.
    inline void AppendExitCorridor(TArray<FBox>& Segments,const TArray<FVector>& Route,
        int32 Waypoint,int32 RouteEnd,TFunctionRef<FVector(int32)> ExtentFor)
    {
        const FVector Junction=Route[Waypoint];
        FVector From=Junction;
        for(int32 Next=Waypoint+1;Next<RouteEnd;++Next)
        {
            const FVector Extent=ExtentFor(Next);
            FBox Exit(From-Extent,From+Extent);
            Exit+=Route[Next]-Extent; Exit+=Route[Next]+Extent;
            Segments.Add(Exit);
            From=Route[Next];
            if(FVector::Dist2D(Junction,From)>=1600.f) break;
        }
    }

    inline bool OlderRequestFirst(const double* A,const double* B,int32 IDA,int32 IDB)
    {
        if(bool(A)!=bool(B)) return A!=nullptr;
        if(A && B && *A!=*B) return *A<*B;
        return IDA<IDB;
    }
}
