#pragma once

// Preserve the authored boundary profile and 1,050 m quay. Expand only inland
// layout coordinates; physical equipment/container dimensions remain unscaled.
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
    // DGT phase 2-5 area target. The outline remains a concept plan, not a survey.
    constexpr double SiteAreaM2 = 837248.0;
    constexpr float QuayLength = 105000.f;
    constexpr float QuayRightX = QuayLeftX + (SiteAreaM2 / 1050.0) * 100.0;

    // Full-frame reference: berth ends at Y=525; eastern service land continues to Y=645.
    // Rear-profile stations follow the overview, not the cropped close-up edge.
    constexpr float SiteBoundaryProfile[][2] = {
        {705.f,-525.f},{705.f,110.f},{730.f,155.f},{785.f,220.f},
        {840.f,265.f},{895.f,320.f},{925.f,370.f},{925.f,645.f}
    };
    constexpr float SiteAlongshoreLengthM=1170.f;
    constexpr int SiteBoundaryPointCount = sizeof(SiteBoundaryProfile)/sizeof(SiteBoundaryProfile[0]);
    constexpr float SiteHalfLengthY = QuayLength * .5f;

    constexpr double AuthoredInlandAreaM2()
    {
        double Area=0;
        for(int I=0;I<SiteBoundaryPointCount-1;++I)
            Area+=(SiteBoundaryProfile[I+1][1]-SiteBoundaryProfile[I][1])*
                (double(SiteBoundaryProfile[I][0])+SiteBoundaryProfile[I+1][0])*.5;
        return Area;
    }
    // The fixed -7 m quay edge contributes a strip that must not be scaled.
    constexpr double PlanDepthScale=(SiteAreaM2+(QuayLeftX/100.0)*SiteAlongshoreLengthM)/AuthoredInlandAreaM2();
    constexpr float SiteX(float Metres) { return Metres>0?Metres*PlanDepthScale:Metres; }
    constexpr float SiteCmX(float Cm) { return SiteX(Cm/100.f)*100.f; }
    constexpr double PlanDepthM=SiteBoundaryProfile[SiteBoundaryPointCount-1][0]*PlanDepthScale-QuayLeftX/100.0;
    constexpr double ConceptSiteAreaM2()
    {
        return AuthoredInlandAreaM2()*PlanDepthScale-(QuayLeftX/100.0)*SiteAlongshoreLengthM;
    }
    static_assert(ConceptSiteAreaM2()>SiteAreaM2-.01 && ConceptSiteAreaM2()<SiteAreaM2+.01,
        "Boundary integration must match the actual area target");

    // Readable metre anchors used by the site blockout.
    constexpr float QuayApronX = 95.f;
    constexpr float YardRoadX = 145.f;
    constexpr float MainInlandRoadX = 620.f;
    constexpr float WingRoadX = 520.f;
    constexpr float EastServiceY = 300.f;
    constexpr float GateX = 790.f;
    constexpr float GateY = 625.f;

    // User-supplied percent anchors in the 916 x 984 sea-on-right overview.
    // Image quay X=700; berth endpoints Y=125..975. Inland service baseline X=285.
    // These map photo positions to authored layout; SiteX supplies the area calibration.
    constexpr float PhotoAnchorX(float Percent) { return (700.f-916.f*Percent/100.f)*705.f/415.f; }
    constexpr float PhotoAnchorY(float Percent) { return 525.f-(984.f*Percent/100.f-125.f)*1050.f/850.f; }

    // Shared block coordinates drive scenery, route planning and telemetry.
    constexpr int LeftYardCount=9, RightYardCount=14;
    constexpr int YardBlockCount=LeftYardCount+RightYardCount;
    constexpr float CentralRoadY=-160.f;
    constexpr float YardFirstY = -465.f;
    constexpr float YardBlockPitch = 35.f;
    // Keep full-size RMG bogies and stopping envelopes apart after plan scaling.
    // Bay 14 used to leave only 18.3 m to the far crane's AGV handover.
    constexpr bool IsOperationalYardBay(int Bay) { return Bay>=0 && Bay<30 && Bay%2==0 && Bay!=14 && Bay!=16; }
    constexpr float YardBayX(int Bay) { return SiteX(205.f+Bay*13.f); }
    constexpr float RMGHomeX(int Half) { return SiteX(Half?517.f:270.f); }
    constexpr float RMGHandoverX(int Half) { return SiteX(Half?420.f:180.f); }
    constexpr float RMGHalfGaugeM=16.f; // Physical rail gauge: 32 m, never scaled.
    constexpr float RMGBogieHalfLengthM=9.8f;
    // Rails support the outer bogies at the nearest handover and furthest bay.
    constexpr float RMGRailStartX=RMGHandoverX(0)-RMGBogieHalfLengthM;
    constexpr float RMGRailEndX=YardBayX(28)+RMGBogieHalfLengthM;


    constexpr float BlockY(int Index) { return YardFirstY+Index*YardBlockPitch+(Index>=LeftYardCount?15.f:0.f); }
    constexpr float ReeferZoneX = 395.f;
    constexpr float ReeferZoneY = -72.5f;
    constexpr float ReeferZoneDepth = 430.f;
    constexpr float ReeferZoneLength = 819.f;
    // Empty-container hardstand between the 23 operational yards and maintenance.
    constexpr float EmptyZoneX = 395.f;
    constexpr float EmptyZoneY = 420.f;
    constexpr float EmptyZoneDepth = 430.f;
    constexpr float EmptyZoneLength = 4.f*YardBlockPitch;

    static_assert(YardBlockCount==23 && BlockY(8)+17.f<CentralRoadY-7.f &&
        BlockY(9)-17.f>CentralRoadY+7.f,"9 left / 14 right blocks clear central road");

    static_assert(EmptyZoneLength==4.f*YardBlockPitch, "Empty yard must span four yard pitches");
    static_assert(BlockY(22)+25.f<=EmptyZoneY-EmptyZoneLength*.5f, "Yard departure must clear empty hardstand");
    // Unified AGV routes, centimetres; keep physical envelopes inside the site.
    constexpr float AGVNorthCrossY = 35000.f;
    constexpr float AGVSouthLoadedY = -48750.f;
    constexpr float AGVSouthReturnY = -50250.f;
    constexpr float AGVParkFirstY = -46000.f;
    // Three 20-bay rows leave room for a full AGV envelope between parked bays.
    constexpr int AGVParkRows=3, AGVParkBays=20;
    constexpr float AGVParkPitch = 3300.f;
    constexpr float AGVParkCrossingWidthM=18.f;
    constexpr float AGVParkX(int Vehicle)
    { return Vehicle/AGVParkBays==0?8000.f:(Vehicle/AGVParkBays==1?10000.f:11200.f); }
    constexpr float AGVParkY(int Vehicle)
    { return AGVParkFirstY+(Vehicle%AGVParkBays)*AGVParkPitch; }
    constexpr float AGVParkExitX(int Vehicle)
    { return Vehicle/AGVParkBays==0?7200.f:(Vehicle/AGVParkBays==1?9000.f:12500.f); }
    constexpr float AGVParkReturnX(int Vehicle)
    { return Vehicle/AGVParkBays==2?12500.f:9000.f; }
    constexpr float AGVParkCrossingY(int Gap)
    { return AGVParkFirstY+(Gap+.5f)*AGVParkPitch; }
    constexpr bool AGVParkCrossingToYard(int Gap) { return Gap%2==0; }
    static_assert(AGVParkRows*AGVParkBays==60,"All fleet vehicles need a parking bay");
    static_assert(AGVParkPitch*.5f>2*730.f+150.f,"Crossings clear parked and moving AGV envelopes");
    static_assert(AGVParkY(AGVParkBays-1)+730.f<23800.f,"Parking stays below the worker-rest apron");
    inline bool AGVEnvelopeInside(double MinX,double MaxX,double MinY,double MaxY)
    {
        if (MinX<SiteCmX(2000.f) || MaxX>SiteCmX(62000.f) || MinY<-51000.f || MaxY>36000.f)
            return false;
        // Only the quay/spine corridor extends beyond the operational yard.
        return MaxX<=SiteCmX(18000.f) || (MinY>=-49700.f && MaxY<=34800.f);
    }

    constexpr float YardSlotX(int Index)
    {
        return NearRailX + (1700.f + (Index / 4) * 400.f) * YardScale;
    }
    constexpr float YardSlotY(int Index)
    {
        return (-2400.f + (Index % 4) * 1600.f) * YardScale;
    }
}
