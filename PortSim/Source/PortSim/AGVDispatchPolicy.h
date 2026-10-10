#pragma once

#include "CoreMinimal.h"
#include "TerminalLayout.h"

namespace AGVDispatchPolicy
{
    constexpr double CruiseSpeedCmPerSecond=600.;
    constexpr double FairnessSecondsPerExtraJob=12.;

    inline double EstimatedApproachDistanceCm(FVector From,FVector Target,int32 STS)
    {
        const double ParkAisleX=TerminalLayout::SiteCmX(TerminalLayout::AGVParkAisleX(0));
        const double BerthApproachX=TerminalLayout::SiteCmX(TerminalLayout::AGVBerthApproachX);
        const double CrossY=TerminalLayout::AGVInboundCrossY(STS);
        return FMath::Abs(From.X-ParkAisleX)+FMath::Abs(From.Y-CrossY)+
            FMath::Abs(ParkAisleX-BerthApproachX)+FMath::Abs(Target.Y-CrossY)+
            FMath::Abs(Target.X-BerthApproachX);
    }

    inline double Score(FVector From,FVector Target,int32 STS,int32 CompletedJobs,int32 MinimumCompletedJobs)
    {
        const double PickupSeconds=EstimatedApproachDistanceCm(From,Target,STS)/CruiseSpeedCmPerSecond;
        const int32 ExtraJobs=FMath::Max(0,CompletedJobs-MinimumCompletedJobs);
        return PickupSeconds+ExtraJobs*FairnessSecondsPerExtraJob;
    }
}
