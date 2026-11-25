#pragma once
#include "CoreMinimal.h"

// Container counts, not TEU or summed crane-hours. Simulation seconds only.
struct FPortBerthPerformance
{
    double StartedAt=-1,CompletedAt=-1;
    int32 Handled=0,Planned=0;
    double Seconds(double Now) const
    {return StartedAt<0?0:FMath::Max(0.,(CompletedAt>=0?CompletedAt:Now)-StartedAt);}
};
