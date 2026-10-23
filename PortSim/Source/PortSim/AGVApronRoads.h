#pragma once
#include "AGVRoadNetwork.h"
#include "TerminalLayout.h"
#include "AGVReference.h"

// Shared by the operational planner and regression tests. Never enabled while
// the old parking rows are occupied. It adds roads, not free-space shortcuts.
namespace AGVApronRoads
{
    inline FVector P(float X,float Y)
    { return FVector(TerminalLayout::SiteCmX(X),Y,0); }

    inline float LoadedSpine(float DepartureY,float DestinationY)
    { return DestinationY>=DepartureY?TerminalLayout::AGVApronNorthX:TerminalLayout::AGVApronSouthX; }

    inline float DepartureY(float HandoverY,float DestinationY)
    {
        float Y=TerminalLayout::AGVApronDepartureY(HandoverY);
        const float Clearance=2*AGVReference::MinimumInnerTurnRadiusCm+100.f;
        for(int Pass=0;Pass<4;++Pass)
        {
            if(FMath::Abs(Y-DestinationY)<Clearance)
                Y=DestinationY-1600.f>=TerminalLayout::AGVSouthLoadedY?DestinationY-1600.f:DestinationY+1600.f;
            if(FMath::Abs(Y-HandoverY)<AGVReference::MinimumInnerTurnRadiusCm+100.f)
                Y=HandoverY+1600.f;
            // A turn just beside a cross-terminal road couples its entire
            // exit reservation to an opposing crossing. Leave a full chassis
            // and turn tangent clear of the newly straightened central road.
            if(FMath::Abs(Y-TerminalLayout::AGVCentralReturnY)<1800.f)
                Y=TerminalLayout::AGVCentralReturnY-1800.f;
            if(FMath::Abs(Y-TerminalLayout::AGVCentralInboundY)<Clearance &&
                !FMath::IsNearlyEqual(Y,TerminalLayout::AGVCentralInboundY,.1f))
                Y=TerminalLayout::AGVCentralInboundY-Clearance;
        }
        return Y;
    }

    inline void Add(FAGVRoadNetwork& Network)
    {
        using namespace TerminalLayout;
        Network.RestrictNarrowCrossing(AGVCentralInboundY,AGVCentralReturnY);
        Network.Add(P(AGVApronNorthX,AGVSouthReturnY),P(AGVApronNorthX,AGVNorthInboundY));
        Network.Add(P(AGVApronSouthX,AGVNorthInboundY),P(AGVApronSouthX,AGVSouthReturnY));
        // The old parked-row jog is unnecessary. All three cross-terminal
        // corridors now connect directly across the vacant apron.
        for(float Y:{AGVSouthLoadedY,AGVCentralInboundY,AGVNorthInboundY})
            Network.Add(P(AGVApronNorthX,Y),P(AGVInlandReturnX,Y));
    }

    inline float AddDeparture(FAGVRoadNetwork& Network,FVector Handover,float DestinationY)
    {
        const float Y=DepartureY(Handover.Y,DestinationY);
        const float X=LoadedSpine(Y,DestinationY);
        Network.Add(Handover,FVector(Handover.X,Y,0));
        Network.Add(FVector(Handover.X,Y,0),P(X,Y));
        return X;
    }

    inline float QueueAccessY(float QueueY,float DockY)
    {
        // Two real 90-degree turns need separate tangents; no quantization to
        // the obsolete 33 m parking grid. Clear the RMG exit turn as well.
        constexpr float Clearance=2*AGVReference::MinimumInnerTurnRadiusCm+100.f;
        float Y=QueueY+1600.f;
        if(FMath::Abs(Y-DockY)<Clearance) Y=DockY+Clearance;
        return Y;
    }
}
