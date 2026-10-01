#pragma once

// Centimeters. Expand inland from the quay-side rail; ship/AGV handovers stay fixed.
namespace TerminalLayout
{
    constexpr int VesselCount = 3;
    constexpr int VesselRows = 10, VesselBays = 10, VesselTiers = 2;
    constexpr int ContainersPerVessel = VesselRows * VesselBays * VesselTiers;
    constexpr int TotalVesselContainers = VesselCount * ContainersPerVessel;
    // Three 210 m vessels stop before the worker-rest building (Y=247..333 m).
    constexpr float VesselLengthM=210.f;
    constexpr float VesselCenterY(int Index) { return -390.f+Index*230.f; }
    constexpr float STSCenterY(int Index) { return VesselCenterY(Index/3)+(Index%3-1)*70.f; }
    static_assert(VesselCenterY(2)+VesselLengthM*.5f+7.f<247.f,"Ships must stop before worker rest");
    static_assert(VesselCenterY(0)-VesselLengthM*.5f>-525.f,"Ships must stay inside quay extent");
    constexpr float YardScale = 1.f;
    constexpr float NearRailX = 5500.f;
    constexpr float FarRailX = NearRailX + 4500.f * YardScale;
    constexpr float RailLength = 10000.f * YardScale;
    constexpr float BridgeCenterX = (NearRailX + FarRailX) * 0.5f;
    constexpr float BridgeWidth = FarRailX - NearRailX + 200.f;
    constexpr float QuayLeftX = -700.f;
    // DGT published area / quay length; rectangular equivalent, not a cadastral outline.
    constexpr float SiteAreaM2 = 837000.f;
    constexpr float QuayRightX = QuayLeftX + (SiteAreaM2 / 1050.f) * 100.f;
    constexpr float QuayLength = 105000.f;

    // Approved plan's SITE bounds (excluding sea, labels and legend):
    // 1358 px wide x 672 px deep. Preserve the 1050 m quay, not 800 m depth.
    constexpr float PlanWidthPixels=1358.f, PlanDepthPixels=672.f;
    constexpr float PlanDepthM=1050.f*PlanDepthPixels/PlanWidthPixels;
    constexpr float PlanDepthScale=(PlanDepthM+QuayLeftX/100.f)/925.f;
    // Convert authored inland positions/footprints; physical equipment and ISO
    // containers are never scaled. Seaward ship geometry remains unchanged.
    constexpr float SiteX(float Metres) { return Metres>0?Metres*PlanDepthScale:Metres; }
    constexpr float SiteCmX(float Cm) { return SiteX(Cm/100.f)*100.f; }
    constexpr float SiteBoundaryProfile[][2] = {
        {705.f,-525.f},{705.f,-90.f},{725.f,-90.f},{725.f,50.f},
        {729.f,58.f},{742.f,68.f},{762.f,78.f},{789.f,90.f},
        {816.f,102.f},{840.f,112.f},{852.f,118.f},{855.f,125.f},
        {855.f,250.f},{860.f,265.f},{875.f,287.f},{898.f,312.f},
        {918.f,332.f},{925.f,347.f},{925.f,525.f}
    };
    constexpr int SiteBoundaryPointCount = sizeof(SiteBoundaryProfile)/sizeof(SiteBoundaryProfile[0]);
    constexpr float SiteHalfLengthY = QuayLength * .5f;

    constexpr float ConceptSiteAreaM2()
    {
        float Area=0.f;
        for (int I=0;I<SiteBoundaryPointCount-1;++I)
        {
            const float Span=SiteBoundaryProfile[I+1][1]-SiteBoundaryProfile[I][1];
            const float DepthA=SiteX(SiteBoundaryProfile[I][0])-QuayLeftX/100.f;
            const float DepthB=SiteX(SiteBoundaryProfile[I+1][0])-QuayLeftX/100.f;
            Area+=Span*(DepthA+DepthB)*.5f;
        }
        return Area;
    }
    static_assert(PlanDepthM>519.f && PlanDepthM<521.f && ConceptSiteAreaM2()>400000.f,
        "Approved plan aspect ratio and nondegenerate polygon");

    // Readable metre anchors used by the site blockout.
    constexpr float QuayApronX = 95.f;
    constexpr float YardRoadX = 145.f;
    constexpr float MainInlandRoadX = 620.f;
    constexpr float WingRoadX = 520.f;
    constexpr float EastServiceY = 300.f;
    constexpr float GateX = 600.f;
    constexpr float GateY = 480.f;

    // Shared block coordinates drive scenery, route planning and telemetry.
    constexpr float YardFirstY = -480.f;
    constexpr float YardBlockPitch = 35.f;
    constexpr float BlockY(int Index) { return YardFirstY+Index*YardBlockPitch; }
    constexpr float ReeferZoneX = 395.f;
    constexpr float ReeferZoneY = -185.f;
    constexpr float ReeferZoneDepth = 430.f;
    constexpr float ReeferZoneLength = 630.f;
    // Facility 9 is a vacant hardstand east of all operational yard blocks.
    constexpr float EmptyZoneX = 395.f;
    constexpr float EmptyZoneY = 202.f;
    constexpr float EmptyZoneDepth = 430.f;
    constexpr float EmptyZoneLength = 124.f;

    static_assert(BlockY(17)+17.f < EmptyZoneY-EmptyZoneLength*.5f,
        "Operational yard and cranes must remain west of vacant facility 9");

    constexpr float YardSlotX(int Index)
    {
        return NearRailX + (1700.f + (Index / 4) * 400.f) * YardScale;
    }
    constexpr float YardSlotY(int Index)
    {
        return (-2400.f + (Index % 4) * 1600.f) * YardScale;
    }
}
