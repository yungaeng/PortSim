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

    constexpr float YardSlotX(int Index)
    {
        return NearRailX + (1700.f + (Index / 4) * 400.f) * YardScale;
    }
    constexpr float YardSlotY(int Index)
    {
        return (-2400.f + (Index % 4) * 1600.f) * YardScale;
    }
}
