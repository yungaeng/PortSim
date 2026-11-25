#include "PortAGVActor.h"
#include "AGVReference.h"

bool APortAGVActor::IncidentNavigationStop() const
{
    return IncidentFaults.Contains(TEXT("fms_disconnect"))||bIncidentFMSPending||bIncidentRoadHold||
        (IncidentFaults.Contains(TEXT("localization_loss"))&&IncidentPositionUncertaintyCm>=25);
}

void APortAGVActor::AdvanceNavigationIncidents(double Dt)
{
    IncidentNavigationClock+=Dt;
    if(IncidentFaults.Contains(TEXT("localization_loss")))
    {
        Sensors.bTransponderLocked=false;
        // Assumed dead-reckoning uncertainty, not measured OEM sensor accuracy.
        IncidentPositionUncertaintyCm+=2.*Dt;
        IncidentResponses.Add(TEXT("localization_loss"));
    }
    if(IncidentFaults.Contains(TEXT("fms_disconnect")))
    {
        Sensors.bFMSConnected=false;
        IncidentResponses.Add(TEXT("fms_disconnect"));
        return;
    }
    if((Sensors.bFMSConnected && !bIncidentNeedsPositionFix) || bDriveFault || bIncidentSensorFault ||
        !Sensors.bDockingNode || Sensors.FMSNodeIndex<0 || Sensors.FMSNodeIndex>=Sensors.FMSNodeCount ||
        Speed>.1f || !GetActorLocation().Equals(Sensors.FMSNextNode,AGVReference::DockingAccuracyCm)) return;

    // A docked AGV can be waiting for a crane or a yard lease with no new
    // waypoint. Confirm the existing dock command without changing its hold.
    if(!Sensors.bFMSConnected)
        SetFMSCommand(Sensors.FMSNextNode,Sensors.FMSNodeIndex,Sensors.FMSNodeCount,true,FMSAcceptanceCm);
    if(Sensors.bFMSConnected && bIncidentNeedsPositionFix)
        ReacquirePositionAtNode(Sensors.FMSNextNode);
}

bool APortAGVActor::ReacquirePositionAtNode(FVector Node)
{
    if(IncidentFaults.Contains(TEXT("localization_loss"))||Speed>.1f||
        !GetActorLocation().Equals(Node,AGVReference::DockingAccuracyCm+.1f))return false;
    CorrectAtTransponder(GetActorLocation());return Sensors.bTransponderLocked;
}

bool APortAGVActor::LocalizationObservationValid() const
{
    if(!FMath::IsFinite(Sensors.PositionErrorCm)||!FMath::IsFinite(IncidentPositionUncertaintyCm))return false;
    if(IncidentFaults.Contains(TEXT("localization_loss"))||bIncidentNeedsPositionFix)
        return !Sensors.bTransponderLocked&&Sensors.PositionErrorCm<=IncidentPositionUncertaintyCm+5.;
    return Sensors.PositionErrorCm<=AGVReference::PositionAccuracyCm+.01f;
}
