#include "TerminalCheckpoint.h"
#include "TerminalReplay.h"
#include "QuayCrane.h"
#include "PortSiteLogistics.h"
#include "PortWorkingCrane.h"
#include "PortAGVActor.h"
#include "PortContainerActor.h"
#include "PortShipActor.h"
#include "Components/PrimitiveComponent.h"
#include "Components/TextRenderComponent.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "PhysicsEngine/PhysicsSettings.h"
#include "PhysicalMaterials/PhysicalMaterial.h"
#include "Chaos/ChaosEngineInterface.h"
#include "SimulationPerformance.h"

bool FTerminalCheckpoint::CanPredict(const AQuayCrane& Owner)
{
    PortSimPerformance::FScope Timing(TEXT("replay_physics_boundary"));
    if(!Owner.bUnifiedTerminal || !IsValid(Owner.SiteLogistics) || !Owner.SiteLogistics->bReady ||
        UPhysicsSettings::Get()->bTickPhysicsAsync || UPhysicsSettings::Get()->PhysicsPrediction.bEnablePhysicsPrediction) return false;
    if(FTerminalReplay::IsPredicting())
    {
        // No engine ticks or external callbacks run inside a transaction. The
        // unified model can only release its manifest cargo into Chaos. Check
        // those bodies each step; the whole world is checked at entry below.
        for(const auto& Cargo:Owner.SiteLogistics->ShipContainers)
            if(!IsValid(Cargo) || Cargo->GetBody()->IsSimulatingPhysics()) return false;
        return true;
    }
    // Sleeping rigid bodies also carry Chaos history and are not safe to rewind.
    for(TActorIterator<AActor> It(Owner.GetWorld());It;++It)
    {
        TInlineComponentArray<UPrimitiveComponent*> Parts(*It);
        for(const auto* Part:Parts) if(Part->IsAnySimulatingPhysics()) return false;
    }
    return true;
}

void FTerminalCheckpoint::CaptureActor(AActor& Actor)
{
    Objects.Add(&Actor);
    Actors.Add({&Actor,Actor.IsHidden(),Actor.GetActorEnableCollision()});
    TInlineComponentArray<USceneComponent*> Components(&Actor);
    for(auto* Part:Components)
    {
        if(Part->Mobility==EComponentMobility::Static) continue;
        Objects.Add(Part);
        auto* Primitive=Cast<UPrimitiveComponent>(Part);
        Parts.Add({Part,Part->GetAttachParent(),Primitive,Part->GetRelativeTransform(),Part->GetAttachSocketName(),
            uint8(Primitive?Primitive->GetCollisionEnabled():ECollisionEnabled::NoCollision),Part->IsVisible(),Part->bHiddenInGame!=0});
        if(auto* Label=Cast<UTextRenderComponent>(Part)) Labels.Add({Label,Label->Text});
    }
}

TSharedPtr<FTerminalCheckpoint> FTerminalCheckpoint::Capture(AQuayCrane& Owner)
{
    PortSimPerformance::FScope Timing(TEXT("replay_capture"));
    if(!CanPredict(Owner)) return nullptr;
    auto Result=MakeShared<FTerminalCheckpoint>();
    Result->Objects.Add(&Owner);
    Result->Values(Owner.AutoElapsed,Owner.AutoCompleted,Owner.Deliveries,Owner.Status,Owner.bAutoRunning,
        Owner.WeatherHold,Owner.STSSimulationTime,Owner.STSObservation,Owner.VesselBaseTransforms,Owner.VesselBasePivots);
    Result->Capture(*Owner.Environment);
    Result->Capture(*Owner.SiteLogistics);
    for(TActorIterator<AActor> It(Owner.GetWorld());It;++It)
    {
        // Camera/UI remain at the user's current pose; only simulation scene
        // components are restored. The unified controller has no live crane.
        if(!Cast<APortWorkingCrane>(*It) && !Cast<APortAGVActor>(*It) &&
            !Cast<APortContainerActor>(*It) && !Cast<APortShipActor>(*It)) continue;
        Result->CaptureActor(**It);
        if(auto* Crane=Cast<APortWorkingCrane>(*It)) Result->Capture(*Crane);
        else if(auto* Vehicle=Cast<APortAGVActor>(*It)) Result->Capture(*Vehicle);
        else if(auto* Cargo=Cast<APortContainerActor>(*It)) Result->Capture(*Cargo);
    }
    return Result;
}

bool FTerminalCheckpoint::Restore() const
{
    PortSimPerformance::FScope Timing(TEXT("replay_restore"));
    for(const auto& Object:Objects) if(!Object.IsValid()) return false;
    for(const auto& Restore:Restorers) Restore();
    for(const auto& A:Actors)
    {
        if(A.Actor->IsHidden()!=A.Hidden) A.Actor->SetActorHiddenInGame(A.Hidden);
        if(A.Actor->GetActorEnableCollision()!=A.Collision) A.Actor->SetActorEnableCollision(A.Collision);
    }
    for(const auto& P:Parts)
    {
        if(P.Primitive && P.Primitive->IsSimulatingPhysics()) P.Primitive->SetSimulatePhysics(false);
        if(P.Part->GetAttachParent()!=P.Parent || P.Part->GetAttachSocketName()!=P.Socket)
        {
            if(P.Parent) P.Part->AttachToComponent(P.Parent,FAttachmentTransformRules::KeepRelativeTransform,P.Socket);
            else P.Part->DetachFromComponent(FDetachmentTransformRules::KeepRelativeTransform);
        }
        if(!P.Part->GetRelativeTransform().Equals(P.Relative,0.))
            P.Part->SetRelativeTransform(P.Relative,false,nullptr,ETeleportType::TeleportPhysics);
        if(P.Part->IsVisible()!=P.Visible) P.Part->SetVisibility(P.Visible);
        if(P.Part->bHiddenInGame!=P.Hidden) P.Part->SetHiddenInGame(P.Hidden);
        if(P.Primitive && P.Primitive->GetCollisionEnabled()!=P.Collision)
            P.Primitive->SetCollisionEnabled(ECollisionEnabled::Type(P.Collision));
    }
    for(const auto& L:Labels) if(!L.Label->Text.EqualTo(L.Text)) L.Label->SetText(L.Text);
    for(const auto& Restore:SceneRestorers) Restore();
    return true;
}

void FTerminalCheckpoint::Capture(UPortEnvironmentComponent& O)
{
    Objects.Add(&O);
    Values(O.OperatingLimits,O.FrictionSettings,O.Surface,O.bSnowEnabled,O.SnowIntensity,O.bIceEnabled,O.IceIntensity);
    Values(O.MaterialSurface,O.MaterialSettings,O.Accumulation,O.PublishedSurface,O.bSurfaceInitialized,
        O.Wind,O.Sea,O.Fog,O.Rain,O.LastReportedGustCount);
    if(O.ContactMaterial)
    {
        auto* Material=O.ContactMaterial.Get(); Objects.Add(Material);
        Values(Material->Friction,Material->StaticFriction,Material->Restitution);
        SceneRestorers.Add([Material]() { FChaosEngineInterface::UpdateMaterial(Material->GetPhysicsMaterial(),Material); });
    }
}

void FTerminalCheckpoint::Capture(APortContainerActor& O)
{
    Values(O.ContainerID,O.CargoInformation,O.SizeClass,O.LengthFt,O.ExternalDimensionsCm,O.TareMassKg,
        O.CargoMassKg,O.MassKg,O.CoGOffsetCm,O.LocationOwner);
    Values(O.KinematicVelocityCentimetersPerSecond,O.KinematicAccelerationCentimetersPerSecondSquared,
        O.KinematicJerkCentimetersPerSecondCubed,O.DeferredVisualParent,O.DeferredVisualRelative,O.SecuredSweptBounds);
    Values(O.PreviousSecuredTransform,O.CurrentSecuredTransform,O.PreviousSecuredVelocity,O.CurrentSecuredVelocity,
        O.PreviousSecuredAcceleration,O.CurrentSecuredAcceleration,O.PreviousSecuredJerk,O.CurrentSecuredJerk,
        O.SecuredFrameDurationSeconds,O.SecuredWindowStart,O.SecuredWindowEnd,O.bHasSecuredMotionFrame);
}

void FTerminalCheckpoint::Capture(APortAGVActor& O)
{
    Values(O.bDriveFault,O.EmergencySettings,O.ActiveEmergencySettings,O.bEmergencyTestOverrides,O.Emergency,
        O.EmergencyWorkerPosition,O.EmergencyDirection,O.DynamicsSettings,O.Dynamics,O.Surface,O.Friction,O.WindVelocity);
    Values(O.LastCurvaturePerCm,O.BatteryPercent,O.bNeedsCharge,O.bCharging,O.VehicleID,O.Speed,O.CompletedJobs,
        O.PayloadKg,O.ActiveDeckOffsetCm,O.DeckCoGOffsetCm,O.Sensors);
    Values(O.EmergencyCargo,O.OriginalEmergencyCoG,O.AppliedEmergencyCoG,O.DisplayedBattery,O.bDisplayedCharging,
        O.FMSAcceptanceCm,O.FMSApproachSpeedLimit,O.SteeringLookAhead,O.bSteeringLookAhead,O.DockingAxis);
}

void FTerminalCheckpoint::Capture(APortWorkingCrane& O)
{
    Values(O.Surface,O.Friction,O.SeatingFrictionDemandN,O.SeatingFrictionCapacityN,O.bSeatingFrictionOK,
        O.LastJobSeconds,O.LastPausedSeconds,O.JobStageSeconds,O.LastJobStageSeconds,O.CompletedStageSeconds,
        O.JobAGVWaitSeconds,O.CompletedAGVWaitSeconds,O.Observation);
    Values(O.bEnabled,O.bSTS,O.bCarrying,O.CraneID,O.CompletedJobs,O.Stage,O.Fault,O.FaultType,O.bSensorFault,O.bGantryFault);
    Values(O.bAutoSensorRecovery,O.SensorRepairMethod,O.SensorRebootSeconds,O.SensorTechnicianRepairSeconds,
        O.SensorSelfCheckSeconds,O.bAllowSensorFaultEvacuation,O.bIndependentTravelSafetyHealthy,O.bFaultEvacuating,
        O.bFaultEvacuationComplete,O.FaultEvacuationTargetY,O.SensorRepairElapsed,O.SensorRepairPhase,O.RecoveryStatus,O.bRailSafetyHold);
    Values(O.CargoActor,O.bReverseTransfer,O.bDeferRopeVisuals,O.bRopeVisualsDirty,O.ShipControlFraction,O.bDeferParts,
        O.bPartsDirty,O.SuspensionState,O.Pickup,O.LockProgress,O.PhysicalSeating,O.PlantAcceleration,O.LockedCargoTransform,O.SuspendedOffset);
    Values(O.HandoverAGV,O.AxisVelocity,O.WindVelocityMetersPerSecond,O.FogVisibilityMeters,O.RainCameraRangeScale,
        O.RainLiDARRangeScale,O.SimulationTime,O.NextSample,O.JobSeconds,O.PausedSeconds,O.AGVAlignmentWait,O.CornerLocked);
    Values(O.bAppliedSensorFault,O.bFatalSensorTest,O.bAppliedGantryFault,O.RecoveryCoordinator,O.SensorInvalidReason,
        O.LockFault,O.Home,O.Orientation,O.Slots,O.Head,O.PausedVelocity,O.PausedAngularVelocity);
    Values(O.BeamZ,O.SafeZ,O.Speed,O.TelescopeLengthCm,O.TelescopeTargetLengthCm,O.SettleTime,O.StageTime,O.SourceSlot,
        O.bPaused,O.bResumePhysics,O.bConfigured,O.bExternalJobs,O.bJobActive,O.bDestinationReady,O.JobStartHead,O.StageHoldTarget);
}

void FTerminalCheckpoint::Capture(APortSiteLogistics& O)
{
    Values(O.Surface,O.Friction,O.SurfaceActorCount,O.BaselineYard,O.InitialYard,O.ReceivingCapacity,O.QueuedHandoffs,
        O.Delivered,O.DispatchLimit,O.Fault,O.EquipmentFaultEvents,O.STSReassignments,O.YardReassignments);
    Values(O.PeakMovingVehicles,O.PrefetchedJobs,O.PairedTrips,O.PlacedContainers,O.RecoveryEvents,O.RecoverySequence,
        O.SimulationTime,O.bDeferringVisuals,O.KinematicPhysicsPoses,O.ShipWindowStart,O.ShipWindowEnd);
    Values(O.ReservationIndex,O.VehicleIndex,O.PendingRoadIndex,O.bTrafficIndexReady,O.VesselStarted,O.VesselUnloaded,
        O.VesselPlaced,O.ResultsCsv,O.Manifest,O.DeferredYardLeases,O.IsolatedEquipment,O.NextRecoveryAttempt);
    Values(O.ScheduledFailedSTS,O.ScheduledFailedRMG,O.ScheduledFailureAt,O.ScheduledRepairAfter,O.bFailureInjected,
        O.bFailureRepaired,O.bRecoveryRailGuard,O.ExportCargo,O.RetainedCargo,O.SourceAbove,O.ManifestBySTS);
    Values(O.Yard,O.CentralReservation,O.CentralPending,O.Jobs,O.BlocksBusy,O.RMGBusy,O.YardApproachOwners,
        O.PreparedCargo,O.STSOwners,O.NextVehicles,O.PreparedStarted,O.RoadReservations,O.RoadReservationEndWaypoint);
    Values(O.TrafficRecoveryTargets,O.TrafficRecoveryOriginalReverse,O.TrafficRecoveryYieldTo,O.LastTrafficRecoveryAt,
        O.RoadTargets,O.RoadBlockers,O.RoadWaitSince,O.PendingRoadRequests,O.TrafficClock,O.LastCycleRecoveryAt);
    Values(O.NoProgressTime,O.LastProgressDelivered,O.FinishedRoadSegments,O.SlotAssigned,O.CentralCount,O.Dispatched,
        O.NextTripID,O.ReturnRouteCounts,O.CentralRoadTravelCm,O.bReady,O.bWasPaused,O.DispatchPriorityLimit);
    Values(O.WindVelocityMetersPerSecond,O.FrameSeaState,O.FogVisibilityMeters,O.RainState,O.VesselMotions,
        O.VesselBasePivots,O.PendingSimulationSeconds);
}
