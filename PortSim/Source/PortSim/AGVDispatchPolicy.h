#pragma once

#include "CoreMinimal.h"

namespace AGVDispatchPolicy
{
    constexpr double CruiseSpeedCmPerSecond=450.;
    constexpr double FairnessSecondsPerExtraJob=12.;

    inline double EstimatedApproachDistanceCm(FVector From,FVector Target)
    {
        // ScheduleFleet uses the same three-leg approach through the 65 m aisle.
        constexpr double AisleX=6500.;
        return FMath::Abs(From.X-AisleX)+FMath::Abs(From.Y-Target.Y)+FMath::Abs(Target.X-AisleX);
    }

    inline double Score(FVector From,FVector Target,int32 CompletedJobs,int32 MinimumCompletedJobs)
    {
        const double PickupSeconds=EstimatedApproachDistanceCm(From,Target)/CruiseSpeedCmPerSecond;
        const int32 ExtraJobs=FMath::Max(0,CompletedJobs-MinimumCompletedJobs);
        return PickupSeconds+ExtraJobs*FairnessSecondsPerExtraJob;
    }
}
