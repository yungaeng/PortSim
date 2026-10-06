#pragma once

#include "CoreMinimal.h"

/** Shared single-lift 20/40/45 ft spreader geometry.
 *
 * The 30 second full-stroke time follows the public Bromma SSX45/YSX45
 * reference.  The simulation moves the two end beams symmetrically; the
 * central frame does not change size.  Dimensions are centimetres/seconds.
 */
namespace PortSpreaderTelescope
{
    constexpr float Length20Cm=605.8f;
    constexpr float Length40Cm=1219.2f;
    constexpr float Length45Cm=1371.6f;
    constexpr float FullStrokeSeconds=30.f;
    constexpr float FullLengthSpeedCmPerSecond=(Length45Cm-Length20Cm)/FullStrokeSeconds;
    constexpr float PositionToleranceCm=1.f;

    inline float Advance(float CurrentLengthCm,float TargetLengthCm,float DeltaSeconds)
    {
        return FMath::FInterpConstantTo(CurrentLengthCm,TargetLengthCm,DeltaSeconds,FullLengthSpeedCmPerSecond);
    }

    inline bool IsReady(float CurrentLengthCm,float TargetLengthCm)
    {
        return FMath::Abs(CurrentLengthCm-TargetLengthCm)<=PositionToleranceCm;
    }

    // ISO corner fitting centre is approximately 103 mm inboard from each end
    // and 89.5 mm inboard from each side of the external container envelope.
    inline float TwistlockHalfLengthCm(float ExternalLengthCm)
    {
        return FMath::Max(0.f,ExternalLengthCm*.5f-10.3f);
    }

    inline float TwistlockHalfWidthCm(float ExternalWidthCm=243.8f)
    {
        return FMath::Max(0.f,ExternalWidthCm*.5f-8.95f);
    }
}
