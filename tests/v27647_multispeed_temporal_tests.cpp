#include "calibration/HardwareTemporalCalibration.h"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <vector>

using namespace mls::calibration;

namespace {
void require(bool cond, const char* msg)
{
    if (!cond) {
        std::cerr << "FAIL: " << msg << "\n";
        std::exit(1);
    }
}
}

int main()
{
    constexpr double trueDelay = 0.000300; // 300 us
    const Vec3d center{18.3228, 8.3006, -206.2326};
    std::vector<TemporalTrajectoryObservation> obs;
    std::uint64_t id = 1, pair = 1, plan = 1;

    for (int level = 0; level < 9; ++level) {
        const double speed = 1.0 + 0.5 * level;
        for (int rep = 0; rep < 3; ++rep) {
            // Deterministic sub-micron center perturbations emulate repeat noise
            // without creating a velocity-dependent bias.
            const double n = (rep - 1) * 0.000015; // +/- 0.015 um
            TemporalTrajectoryObservation pos;
            pos.id = id++; pos.planId = plan++; pos.pairId = pair; pos.directionSign = +1;
            pos.velocityMmPerSec = {speed, 0.0, 0.0};
            pos.sphereCenterMachineMm = {center.x - trueDelay * speed + n,
                                         center.y + 0.25*n,
                                         center.z - 0.10*n};
            pos.sphereRmsMm = 0.010; pos.fittedRadiusMm = 9.542;
            pos.sphereInliers = 200000; pos.profileCount = 1000;
            pos.resolutionIndex = 0; pos.exposureTime10Us = 30; pos.idleTime10Us = 469; pos.triggerPeriodMs = 5;

            TemporalTrajectoryObservation neg = pos;
            neg.id = id++; neg.planId = plan++; neg.directionSign = -1;
            neg.velocityMmPerSec = {-speed, 0.0, 0.0};
            // Use the same common-mode repeat perturbation so forward/reverse
            // differ only by the temporal term, as intended by paired scanning.
            neg.sphereCenterMachineMm = {center.x + trueDelay * speed + n,
                                         center.y + 0.25*n,
                                         center.z - 0.10*n};
            obs.push_back(pos); obs.push_back(neg); ++pair;
        }
    }

    auto r = HardwareTemporalCalibration::solveTrajectoryPairs(obs);
    require(r.success, "9-speed x3 solver should succeed");
    require(r.pairCount == 27, "expected 27 forward/reverse pairs");
    require(r.distinctSpeedLevels == 9, "expected 9 distinct measured speed levels");
    require(std::abs(r.delaySec - trueDelay) < 1e-9, "global delay should recover 300 us");
    require(r.rSquared > 0.999999, "ideal repeated experiment should be highly linear");

    TemporalTrajectoryCriteria c;
    c.minPairs = 18;
    c.minDistinctSpeedLevels = 7;
    c.minRSquared = 0.70;
    c.maxDelayStdSec = 0.000150;
    c.maxPairResidualMm = 0.050;
    require(HardwareTemporalCalibration::evaluateTrajectoryQuality(r, c), "recommended V27.6.47 gates should pass ideal experiment");

    for (const auto& pf : r.pairs)
        require(std::abs(pf.pairDelaySec - trueDelay) < 1e-9, "each repeated pair should recover the same delay");

    std::cout << "V27.6.47 9-speed x3 repeated temporal experiment tests PASS\n";
    return 0;
}
