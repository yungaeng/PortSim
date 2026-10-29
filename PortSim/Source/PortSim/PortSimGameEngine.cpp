#include "PortSimGameEngine.h"
#include "PortSimTimeStep.h"
#include "HAL/PlatformTime.h"
#include "HAL/IConsoleManager.h"

static TAutoConsoleVariable<float> CVarPresentationHz(TEXT("PortSim.PresentationHz"),60.f,
    TEXT("Standalone viewport refresh rate in wall-clock Hz (1..120). Physics remains fixed at 60 Hz simulation time."));

void UPortSimGameEngine::RedrawViewports(bool bShouldPresent)
{
    if(Cast<UPortSimTimeStep>(GetCustomTimeStep()))
    {
        const double Now=FPlatformTime::Seconds();
        if(Now<NextPresentationWall) return;
        const float Requested=CVarPresentationHz.GetValueOnGameThread();
        const double Hz=FMath::IsFinite(Requested)?FMath::Clamp(double(Requested),1.,120.):60.;
        NextPresentationWall=FMath::Max(NextPresentationWall+1./Hz,Now);
    }
    else NextPresentationWall=0;
    // Draw all scene elements from the same completed step. In particular,
    // never throttle the rope separately from the spreader or attached load.
    Super::RedrawViewports(bShouldPresent);
}
