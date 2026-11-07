#pragma once
#include "calibration/SphereExposureSweep.h"
#include <cmath>
#include <array>
#include <cstddef>

namespace mls::calibration {
// Portable quality contract for every imported .scan v3 sphere reference.
// The geometric thresholds are DIAGNOSTIC starting values, not sensor accuracy guarantees.
struct SphereTwoStageQuality {
    double minimumArcSpanDeg = 120.0;
    double maximumArcRmsMm = 0.010;
    std::size_t minimumArcInliers = 80;
    std::size_t minimumAcceptedProfiles = 20;
    double refinedRansacThresholdMm = 0.020;
};
// Machine-center -> sensor X/Z prediction for a *saved* row-major 4x4
// sensorToMachine transform.  Only the center projected to the profile plane
// is used as an arc ROI anchor.  All coordinates are binary64.
inline bool projectSphereCenterToSensorArcXZ(
    const std::array<double,16>& sensorToMachine,
    double machineX, double machineY, double machineZ,
    double& expectedX, double& expectedZ)
{
    const auto& t = sensorToMachine;
    for (double a : t) if (!std::isfinite(a)) return false;
    if (!std::isfinite(machineX) || !std::isfinite(machineY) || !std::isfinite(machineZ))
        return false;
    const double dx=machineX-t[3], dy=machineY-t[7], dz=machineZ-t[11];
    expectedX=t[0]*dx+t[4]*dy+t[8]*dz;
    expectedZ=t[2]*dx+t[6]*dy+t[10]*dz;
    return std::isfinite(expectedX) && std::isfinite(expectedZ);
}
inline bool qualifySphereArc(const SphereExposureArc& a,
                             const SphereTwoStageQuality& q,
                             double expectedRadiusMm)
{
    return a.valid && a.arcSpanDeg >= q.minimumArcSpanDeg &&
           a.rms <= q.maximumArcRmsMm && a.inliers >= q.minimumArcInliers &&
           a.radius > 0.20 * expectedRadiusMm &&
           a.radius <= expectedRadiusMm + 0.25 &&
           std::isfinite(a.radius) && std::isfinite(a.rms);
}
} // namespace mls::calibration
