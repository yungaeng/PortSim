#pragma once

#include "CoreMinimal.h"
#include "TerminalLayout.h"

namespace AGVDispatchPolicy
{
    constexpr double CruiseSpeedCmPerSecond=600.;
    constexpr double FairnessSecondsPerExtraJob=12.;

    // Oldest physically accessible request wins among executable work. A
    // blocked request never prevents a different ready yard/RMG from working.
    inline bool RequestBefore(double EligibleA,int32 OrderA,double EligibleB,int32 OrderB)
    {
        return EligibleA!=EligibleB?EligibleA<EligibleB:OrderA<OrderB;
    }

    inline bool UseDirectReturn(double ReturnSeconds,double IdlePickupSeconds)
    {
        return ReturnSeconds<=IdlePickupSeconds;
    }

    struct FRequest
    {
        int32 Cargo=INDEX_NONE, Order=INDEX_NONE;
        double EligibleAt=0;
        bool SourceReady=false, YardReady=false;
        int32 CriticalPath=0, Unlocks=0;
        bool DedicatedHazardYard=false;
    };
    inline double ReadyPriority(double EligibleAt,int32 CriticalPath,bool DedicatedHazardYard=false)
    {
        // Each hazard group has only one compatible yard block. Feed its
        // approach lanes while general cargo can still use the other blocks.
        // The fixed lead keeps the sort transitive and lets older work catch up.
        constexpr double DedicatedYardLeadSeconds=2400.;
        return FMath::Clamp(CriticalPath,0,8)*120.+(DedicatedHazardYard?DedicatedYardLeadSeconds:0.)-EligibleAt;
    }
    inline bool ReadyBefore(const FRequest& A,const FRequest& B)
    {
        const double PA=ReadyPriority(A.EligibleAt,A.CriticalPath,A.DedicatedHazardYard);
        const double PB=ReadyPriority(B.EligibleAt,B.CriticalPath,B.DedicatedHazardYard);
        if(PA!=PB) return PA>PB;
        if(A.Unlocks!=B.Unlocks) return A.Unlocks>B.Unlocks;
        return RequestBefore(A.EligibleAt,A.Order,B.EligibleAt,B.Order);
    }
    inline int32 SelectExecutable(const TArray<FRequest>& Requests)
    {
        const FRequest* Best=nullptr;
        for(const auto& R:Requests)
            if(R.SourceReady && R.YardReady && (!Best ||
                ReadyBefore(R,*Best))) Best=&R;
        return Best?Best->Cargo:INDEX_NONE;
    }

    inline double EstimatedApproachDistanceCm(FVector From,FVector Target,int32 STS,int32 DepotBay=INDEX_NONE)
    {
        if(DepotBay>=0 && DepotBay<TerminalLayout::AGVChargeBays && From.Y>36000.)
        {
            // Leave the bay longitudinally through its one-way row aisle,
            // then use the depot's southbound spine before entering the quay.
            // Odd/even rows have different exit ends; the old quay parking
            // estimate understated this detour and favoured distant idle AGVs.
            const double ExitY=TerminalLayout::AGVStandbyExitY(DepotBay);
            const double ExitX=TerminalLayout::SiteCmX(TerminalLayout::AGVDepotOutboundX);
            return FMath::Abs(From.Y-ExitY)+FMath::Abs(From.X-ExitX)+
                FMath::Abs(ExitY-TerminalLayout::AGVDepotAccessY)+FMath::Abs(ExitX-TerminalLayout::SiteCmX(14000.f))+
                FMath::Abs(TerminalLayout::AGVDepotAccessY-Target.Y)+FMath::Abs(TerminalLayout::SiteCmX(14000.f)-Target.X);
        }
        const double ParkAisleX=TerminalLayout::SiteCmX(TerminalLayout::AGVParkAisleX(0));
        const double BerthApproachX=TerminalLayout::SiteCmX(TerminalLayout::AGVBerthApproachX);
        const double CrossY=TerminalLayout::AGVInboundCrossY(STS);
        return FMath::Abs(From.X-ParkAisleX)+FMath::Abs(From.Y-CrossY)+
            FMath::Abs(ParkAisleX-BerthApproachX)+FMath::Abs(Target.Y-CrossY)+
            FMath::Abs(Target.X-BerthApproachX);
    }

    inline double Score(FVector From,FVector Target,int32 STS,int32 CompletedJobs,int32 MinimumCompletedJobs,
        int32 DepotBay=INDEX_NONE)
    {
        const double PickupSeconds=EstimatedApproachDistanceCm(From,Target,STS,DepotBay)/CruiseSpeedCmPerSecond;
        const int32 ExtraJobs=FMath::Clamp(CompletedJobs-MinimumCompletedJobs,0,4);
        return PickupSeconds+ExtraJobs*FairnessSecondsPerExtraJob;
    }
}
