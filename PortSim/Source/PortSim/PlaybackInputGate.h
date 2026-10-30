#pragma once
#include "CoreMinimal.h"

// Requested speed survives external control edits. Wall time, rather than
// simulation time, determines when a continuous slider gesture is finished.
struct FPlaybackInputGate
{
    double QuietUntil=0;
    uint64 Revision=0;
    void Submit(double Now) { QuietUntil=Now+2.; ++Revision; }
    bool IsLive(double Now) const { return Now<QuietUntil; }
    float EffectiveRate(float Requested,double Now) const { return IsLive(Now)?1.f:Requested; }
};
