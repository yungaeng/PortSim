#if WITH_DEV_AUTOMATION_TESTS
#include "TerminalLayout.h"
#include "AGVReference.h"


#include "Misc/AutomationTest.h"
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAGVBoundsTest,"PortSim.Logistics.AGVTravelBounds",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FAGVBoundsTest::RunTest(const FString& Parameters)
{
    using namespace TerminalLayout;

    auto Check=[&](bool Value,const char* Name) { TestTrue(UTF8_TO_TCHAR(Name),Value); };
    auto Fits=[](float X,float Y,float EX,float EY) { return AGVEnvelopeInside(X-EX,X+EX,Y-EY,Y+EY); };
    Check(!Fits(SiteCmX(16500),51500,210,730),"old right-side route is forbidden");
    Check(!Fits(SiteCmX(14500),-55000,730,210),"old off-map return is forbidden");
    Check(!Fits(SiteCmX(5500),46000,210,730),"worker-rest site is excluded");
    Check(!Fits(SiteCmX(42000),37000,730,210),"unused cargo apron is excluded");
    Check(!Fits(SiteCmX(62000),0,730,210),"vehicle body cannot cross inland boundary");
    Check(!Fits(SiteCmX(14500),-50750,730,300),"body protrusion across quay end rejected");
    for(float X:{5500.f,16500.f}) Check(Fits(SiteCmX(X),AGVNorthCrossY,210,730),"north crossing footprint");
    for(float X:{3500.f,12500.f}) Check(Fits(SiteCmX(X),AGVSouthLoadedY,210,730),"south loaded footprint");
    for(float X:{10000.f,14500.f}) Check(Fits(SiteCmX(X),AGVSouthReturnY,730,210),"return crossing footprint");
    for(int I=0;I<60;++I) Check(Fits(SiteCmX(AGVParkX(I)),AGVParkY(I),210,730),"fleet parking footprint");
    Check(AGVParkY(AGVParkBays-1)+730 < AGVNorthCrossY-730,"north crossing clears parked fleet");
    Check(AGVSouthLoadedY-AGVSouthReturnY>1460,"south crossings clear full vehicle lengths");
    Check(AGVParkFirstY-AGVSouthLoadedY>1460,"south crossing clears idle parking");
    Check(Fits(SiteCmX(9000),AGVSouthReturnY,210,730),"empty return keeps full length inside south boundary");
    Check(SiteCmX(9000)-SiteCmX(8000)>420 && SiteCmX(10000)-SiteCmX(9000)>420,"parking aisle clears both occupied rows");
    for(int Gap=0;Gap<AGVParkBays-1;++Gap)
    {
        const float Y=AGVParkCrossingY(Gap);
        for(int Vehicle=0;Vehicle<60;++Vehicle)
            Check(FMath::Abs(Y-AGVParkY(Vehicle))>1460.f,"crossing clears every parked AGV envelope");
        for(float X:{3500.f,8000.f,10000.f,11200.f,16500.f})
            Check(Fits(SiteCmX(X),Y,210,730),"internal crossing stays inside road bounds");
    }
    Check(FMath::IsNearlyEqual(AGVReference::MaximumPayloadKg,65000.f),"supplied 65 t payload reference");
    Check(FMath::IsNearlyEqual(AGVReference::ReferenceStraightSpeedCmPerSecond,700.f),"supplied 7 m/s straight speed reference");
    Check(FMath::IsNearlyEqual(AGVReference::ReferenceCurveSpeedCmPerSecond,250.f),"supplied 2.5 m/s curve speed reference");
    Check(!AGVReference::LateralMotionEnabled,"lateral crab motion is disabled by project requirement");
    Check(FMath::IsNearlyEqual(AGVReference::PositionAccuracyCm,2.5f),"supplied 25 mm position accuracy reference");
    for(int B=0;B<YardBlockCount;++B) for(float X:{18000.f,42000.f})
    {
        Check(Fits(SiteCmX(X),BlockY(B)*100+1320,730,210),"yard handover footprint");
        Check(Fits(SiteCmX(X),BlockY(B)*100+2050,730,210),"yard entry footprint");
        Check(Fits(SiteCmX(X),BlockY(B)*100+2500,730,210),"yard departure footprint");
    }

    return true;
}


#endif
