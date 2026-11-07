#include "calibration/HardwareTemporalCalibration.h"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <vector>

using namespace mls::calibration;

namespace {
void require(bool cond, const char* msg) {
    if (!cond) { std::cerr << "FAIL: " << msg << "\n"; std::exit(1); }
}
}

int main()
{
    // Dynamic point clouds may have a common effective radius that is not the
    // physical artefact radius. The free refinement must keep that nuisance
    // geometry instead of forcing the nominal 9.525 mm value.
    const Vec3d trueCenter{18.3, 8.2, -206.2};
    constexpr double effectiveR = 9.545;
    std::vector<Vec3d> sphere;
    for (int ia=0; ia<24; ++ia) {
        const double a = 2.0 * 3.14159265358979323846 * double(ia) / 24.0;
        for (int ib=1; ib<12; ++ib) {
            const double b = 3.14159265358979323846 * double(ib) / 12.0;
            sphere.push_back({trueCenter.x + effectiveR*std::sin(b)*std::cos(a),
                              trueCenter.y + effectiveR*std::sin(b)*std::sin(a),
                              trueCenter.z + effectiveR*std::cos(b)});
        }
    }
    auto freeFit = HardwareTemporalCalibration::refineFreeSphere(
        sphere, {trueCenter.x+0.05,trueCenter.y-0.03,trueCenter.z+0.02}, 9.525, 0.12, 30);
    require(freeFit.success, "free effective-radius fit succeeds");
    require(std::abs(freeFit.radiusMm-effectiveR) < 1e-8, "free fit recovers effective radius");
    require(std::abs(freeFit.centerMachineMm.x-trueCenter.x) < 1e-8, "free fit center x");

    constexpr double trueDelay = 0.000300; // 300 us
    const Vec3d drift{0.00050/60.0, 0.00080/60.0, 0.00140/60.0}; // mm/s = um/min / 1000 / 60
    const Vec3d c0{18.3228, 8.3006, -206.2326};
    std::vector<TemporalTrajectoryObservation> obs;
    std::uint64_t id=1, pair=1, plan=1;
    double tSec=0.0;
    for(int level=0; level<10; ++level){
        const double speed=0.5+0.5*level;
        for(int rep=0; rep<3; ++rep){
            // Long low-speed scans intentionally create a large slow-drift bias
            // in the naive +/- pair estimate.
            const double scanSec=30.0/speed;
            const double tPos=tSec+0.5*scanSec;
            const double tNeg=tSec+scanSec+4.0+0.5*scanSec;
            const double pairR=effectiveR + 0.0008*std::sin(double(pair));
            TemporalTrajectoryObservation pos;
            pos.id=id++;pos.planId=plan++;pos.pairId=pair;pos.directionSign=+1;
            pos.velocityMmPerSec={speed,0,0};
            pos.sphereCenterMachineMm={c0.x+drift.x*tPos-trueDelay*speed,
                                       c0.y+drift.y*tPos,
                                       c0.z+drift.z*tPos};
            pos.sphereRmsMm=0.010;pos.fittedRadiusMm=pairR+0.0002;
            pos.sphereInliers=200000;pos.profileCount=1000;pos.resolutionIndex=0;
            pos.exposureTime10Us=30;pos.idleTime10Us=469;pos.triggerPeriodMs=5;
            pos.captureMidPcUs=1'000'000 + std::int64_t(std::llround(tPos*1e6));

            TemporalTrajectoryObservation neg=pos;
            neg.id=id++;neg.planId=plan++;neg.directionSign=-1;
            neg.velocityMmPerSec={-speed,0,0};
            neg.sphereCenterMachineMm={c0.x+drift.x*tNeg+trueDelay*speed,
                                       c0.y+drift.y*tNeg,
                                       c0.z+drift.z*tNeg};
            neg.fittedRadiusMm=pairR-0.0002;
            neg.captureMidPcUs=1'000'000 + std::int64_t(std::llround(tNeg*1e6));
            obs.push_back(pos);obs.push_back(neg);
            ++pair;tSec=tNeg+0.5*scanSec+4.0;
        }
    }

    auto r=HardwareTemporalCalibration::solveTrajectoryPairs(obs);
    require(r.success, "drift-aware trajectory solve succeeds");
    require(r.driftModelUsed, "capture midpoint timestamps activate drift-aware model");
    require(r.pairCount==30, "0.5..5.0 x3 gives 30 pairs");
    require(r.distinctSpeedLevels==10, "ten speed levels detected");
    require(std::abs(r.delaySec-trueDelay)<2e-9, "drift-aware delay recovers 300 us");
    require(std::abs(r.centerDriftMmPerSec.x-drift.x)<2e-10, "x drift recovered");
    require(std::abs(r.centerDriftMmPerSec.z-drift.z)<2e-10, "z drift recovered");
    require(std::abs(r.naivePairDelaySec-r.delaySec)>1e-5, "naive pair delay is visibly biased by slow drift");
    require(r.maxPairRadiusDifferenceMm<0.001, "forward/reverse effective radii remain common despite nominal bias");
    require(std::abs(r.meanEffectiveRadiusMm-effectiveR)<0.001, "effective radius may differ from nominal without invalidating delay");

    TemporalTrajectoryCriteria c;
    c.minPairs=20;c.minDistinctSpeedLevels=8;c.minRSquared=0.95;
    c.maxDelayStdSec=50e-6;c.maxPairResidualMm=0.01;c.maxPairRadiusDifferenceMm=0.005;
    require(HardwareTemporalCalibration::evaluateTrajectoryQuality(r,c), "V50 drift/radius quality gates pass");

    std::cout << "V27.6.50 drift-aware/free-effective-radius temporal tests PASS\n";
    return 0;
}
