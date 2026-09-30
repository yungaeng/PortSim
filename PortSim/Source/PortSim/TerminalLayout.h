#pragma once

// Centimeters. Expand inland from the quay-side rail; ship/AGV handovers stay fixed.
namespace TerminalLayout
{
    constexpr int VesselCount = 3;
    constexpr int VesselRows = 10, VesselBays = 10, VesselTiers = 2;
    constexpr int ContainersPerVessel = VesselRows * VesselBays * VesselTiers;
    constexpr int TotalVesselContainers = VesselCount * ContainersPerVessel;
    constexpr float YardScale = 1.f;
    constexpr float NearRailX = 5500.f;
    constexpr float FarRailX = NearRailX + 4500.f * YardScale;
    constexpr float RailLength = 10000.f * YardScale;
    constexpr float BridgeCenterX = (NearRailX + FarRailX) * 0.5f;
    constexpr float BridgeWidth = FarRailX - NearRailX + 200.f;
    constexpr float QuayLeftX = -700.f;
    // DGT published area / quay length; rectangular equivalent, not a cadastral outline.
    constexpr float SiteAreaM2 = 837201.f;
    constexpr float QuayRightX = QuayLeftX + (SiteAreaM2 / 1050.f) * 100.f;
    constexpr float QuayLength = 105000.f;

    // DGT concept-plan back boundary in metres: {inland X, along-quay Y}.
    // The straight western yard transitions through a multi-segment curve to
    // the deeper gate wing instead of using an axis-aligned L footprint.
    constexpr float SiteBoundaryProfile[][2] = {
        {732.f,-525.f},{732.f,50.f},{739.f,80.f},{759.f,110.f},
        {794.f,140.f},{844.f,170.f},{889.f,200.f},{894.f,280.f},
        {896.f,360.f},{894.f,440.f},{889.f,500.f},{879.f,525.f}
    };
    constexpr int SiteBoundaryPointCount = sizeof(SiteBoundaryProfile)/sizeof(SiteBoundaryProfile[0]);
    constexpr float SiteHalfLengthY = QuayLength * .5f;

    constexpr float ConceptSiteAreaM2()
    {
        float Area=0.f;
        for (int I=0;I<SiteBoundaryPointCount-1;++I)
        {
            const float Span=SiteBoundaryProfile[I+1][1]-SiteBoundaryProfile[I][1];
            const float DepthA=SiteBoundaryProfile[I][0]-QuayLeftX/100.f;
            const float DepthB=SiteBoundaryProfile[I+1][0]-QuayLeftX/100.f;
            Area+=Span*(DepthA+DepthB)*.5f;
        }
        return Area;
    }
    static_assert(ConceptSiteAreaM2()>830000.f && ConceptSiteAreaM2()<840000.f,
        "DGT concept footprint must remain within the published 830-840k m2 range");

    // Readable metre anchors used by the site blockout.
    constexpr float QuayApronX = 95.f;
    constexpr float YardRoadX = 145.f;
    constexpr float MainInlandRoadX = 620.f;
    constexpr float WingRoadX = 820.f;
    constexpr float EastServiceY = 300.f;
    constexpr float GateX = 860.f;
    constexpr float GateY = 480.f;

    // Original BPA yard-map zones, expressed in metres. Both zones are part
    // of the main operational yard; zone 9 is not a detached service pad.
    constexpr float ReeferZoneX = 395.f;
    constexpr float ReeferZoneY = -245.f;
    constexpr float ReeferZoneDepth = 390.f;
    constexpr float ReeferZoneLength = 430.f;
    constexpr float EmptyZoneX = 395.f;
    constexpr float EmptyZoneY = 155.f;
    constexpr float EmptyZoneDepth = 390.f;
    constexpr float EmptyZoneLength = 310.f;

    constexpr float YardSlotX(int Index)
    {
        return NearRailX + (1700.f + (Index / 4) * 400.f) * YardScale;
    }
    constexpr float YardSlotY(int Index)
    {
        return (-2400.f + (Index % 4) * 1600.f) * YardScale;
    }
}
