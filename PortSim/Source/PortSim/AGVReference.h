#pragma once

#include "CoreMinimal.h"

// Numeric reference values transcribed from Document/AGV/ga-zpmc-qc8-12.pdf.
// Published vehicle maxima are kept separate from the existing PortSim site
// operating limits, which remain authoritative where they are more conservative.
namespace AGVReference
{
    constexpr float VehicleLengthCm=1600.f;
    constexpr float VehicleWidthCm=300.f;
    constexpr float VehicleHeightCm=230.f;
    constexpr float MaximumPayloadKg=65000.f;

    constexpr float ReferenceStraightSpeedCmPerSecond=700.f;
    constexpr float ReferenceCurveSpeedCmPerSecond=250.f;
    // The source PDF lists a crab/S-motion maximum, but PortSim deliberately
    // disables that mode.  Runtime travel is longitudinal forward/reverse only.
    constexpr float SourceCrabSpeedNotAppliedCmPerSecond=200.f;
    constexpr bool LateralMotionEnabled=false;
    constexpr float EmptyAccelerationCmPerSecondSquared=150.f;
    constexpr float LoadedAccelerationCmPerSecondSquared=40.f;
    constexpr float PositionAccuracyCm=2.5f;
    constexpr float DockingAccuracyCm=5.f;
    // A one-degree centreline tolerance becomes roughly 12 cm at a 45 ft
    // container corner.  Keep the heading tight enough for the RMG's
    // independent twistlock corner sensors instead of accepting centre-only alignment.
    constexpr float DockingHeadingToleranceDegrees=.1f;
    constexpr float DockingSpeedCmPerSecond=10.f;
    constexpr float DockingApproachDistanceCm=100.f;
    constexpr float MinimumInnerTurnRadiusCm=580.f;
    constexpr float MinimumOuterTurnRadiusCm=1135.f;
    // Intermediate FMS points define the road centreline.  A car-like vehicle
    // rounds these points inside a bounded corner corridor instead of stopping
    // on a mathematically exact right-angle coordinate.
    constexpr float WaypointAcceptanceCm=800.f;
    // Engineering assumption used only to convert commanded curvature to the
    // steering value shown by the simulation sensors.
    constexpr float EffectiveWheelbaseCm=850.f;

    // Existing PortSim values take priority and are below the supplied reference maximum.
    constexpr float SiteEmptySpeedCmPerSecond=450.f;
    constexpr float SiteLoadedSpeedCmPerSecond=350.f;
    constexpr float PhysicalHalfLengthCm=730.f;
    constexpr float PhysicalHalfWidthCm=210.f;

    inline float PayloadRatio(float PayloadKg)
    { return FMath::Clamp(PayloadKg/MaximumPayloadKg,0.f,1.f); }

    inline float SiteSpeedLimit(float PayloadKg)
    {
        return FMath::Min(ReferenceStraightSpeedCmPerSecond,
            FMath::Lerp(SiteEmptySpeedCmPerSecond,SiteLoadedSpeedCmPerSecond,
                FMath::Clamp(PayloadKg/30000.f,0.f,1.f)));
    }

    inline float AccelerationLimit(float PayloadKg)
    {
        return FMath::Lerp(EmptyAccelerationCmPerSecondSquared,
            LoadedAccelerationCmPerSecondSquared,PayloadRatio(PayloadKg));
    }

    inline float DynamicLiDARSafetyDistance(float SpeedCmPerSecond,float DecelerationCmPerSecondSquared)
    {
        // The supplied document does not publish a LiDAR range.  PortSim uses
        // the exact no-latency braking distance plus localization uncertainty.
        return SpeedCmPerSecond*SpeedCmPerSecond/(2.f*FMath::Max(1.f,DecelerationCmPerSecondSquared))+
            2.f*PositionAccuracyCm;
    }

    inline float AxialHeadingErrorDegrees(float A,float B)
    {
        const float Error=FMath::Abs(FMath::FindDeltaAngleDegrees(A,B));
        // A rectangular AGV can enter a handover in reverse.  A 180 degree
        // heading therefore has the same longitudinal alignment as 0 degrees.
        return FMath::Min(Error,180.f-Error);
    }
}
