#include "calibration/SphereDistanceAngleStudy.h"

#include <chrono>
#include <iostream>
#include <string>

using namespace mls::calibration;

namespace {
int fail(const char* s){ std::cerr << "FAIL: " << s << "\n"; return 1; }

SphereStudyPathOptions makeOptions()
{
    SphereStudyPathOptions o;
    o.sphereRadiusMm=9.525;
    o.measurementDistancesMm={82.0,87.0,92.0,97.0,102.0,107.0};
    o.sectionAnglesDeg={0.0,10.0,20.0,30.0,40.0,50.0};
    o.centerBearingAnglesDeg={-15.0,-7.5,0.0,7.5,15.0};
    o.referenceJoint={0,0,0,0,0};
    o.opticalCenterSensorMm={0,0,0};
    o.xMinMm=-147;o.xMaxMm=114;
    o.yMinMm=-116;o.yMaxMm=134;
    o.zMinMm=-32;o.zMaxMm=53;
    o.cMinDeg=-87;o.cMaxDeg=87;
    o.bMinDeg=-87;o.bMaxDeg=87;
    o.automaticPlaneFamily=true;
    o.planeDirectionsPerGeometry=8;
    o.rotaryCandidateStepDeg=10.0;
    o.maxAutomaticRotaryCandidateCount=2048;
    o.minimumPlaneSeparationDeg=8.0;
    o.spaceFillingSampling=true;
    o.targetPoseCount=120; // match the GUI default used in the reported offline case
    o.sensorXMinMm=-25;o.sensorXMaxMm=25;
    o.sensorZMinMm=70;o.sensorZMaxMm=120;
    o.sensorXSafetyMarginMm=1;o.sensorZSafetyMarginMm=1;
    o.minimumPlaneIntersectionMarginMm=0.5;
    o.minimumPredictedVisibleArcDeg=75;
    o.minimumPredictedVisibleArcLengthMm=6.0;
    o.visibleArcSamples=180;
    o.adaptiveOpticalResampling=false;
    return o;
}
}

int main()
{
    const Vec3d sphereM(0.0,0.0,0.0);
    Physical17Parameters model;

    // Reproduce the AC offline bug input exactly: an unconnected controller
    // leaves AxisParams at +/-999999. The numerical planner must reject this
    // before ever entering a C x B nested loop.
    auto bad=makeOptions();
    bad.cMinDeg=-999996.0; bad.cMaxDeg=999996.0;
    bad.bMinDeg=-999996.0; bad.bMaxDeg=999996.0;
    std::string badErr;
    const auto t0=std::chrono::steady_clock::now();
    const auto badPath=SphereDistanceAngleStudy::generatePath(sphereM,model,bad,badErr);
    const auto ms=std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now()-t0).count();
    if(!badPath.empty()) return fail("placeholder rotary limits should be rejected");
    if(badErr.find("grid is too large") == std::string::npos) return fail("missing oversized-grid diagnostic");
    if(ms > 1000) return fail("oversized-grid guard did not fail fast");

    // The finite Machine5X/V32.9 offline bounds used by AD must remain accepted.
    auto good=makeOptions();
    std::string goodErr;
    const auto goodPath=SphereDistanceAngleStudy::generatePath(
        Vec3d(42.0,-17.5,103.0),model,good,goodErr);
    if(goodPath.empty()) return fail("finite offline bounds should generate a path");
    for(const auto& p:goodPath) if(!p.valid) return fail("finite offline path contains invalid selected pose");

    std::cout << "PASS V27.6.16AD offline planning finite-limit + rotary-grid hang guard\n";
    return 0;
}
