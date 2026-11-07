#pragma once

#include "runtime/FiveAxis.h"
#include "calibration/SingleSphere17Calibration.h"

#include <array>
#include <cstddef>
#include <string>
#include <vector>

namespace mls::runtime {

/**
 * Software RTCP for the calibrated XYZ-C-B scanner head.
 *
 * The controlled point is expressed in the sensor frame.  For the default
 * {0,0,0} this is the calibrated sensor-frame origin.  A caller can instead
 * select a reference point in the laser plane (for example {0,0,95} mm).
 *
 * Five-axis kinematic DOF are used as: desired controlled-point XYZ in the
 * machine frame + commanded C/B orientation.  Arbitrary 6-DoF sensor poses are
 * intentionally rejected; this machine has only two independent rotary DOF.
 */
struct Rtcp17Options {
    calibration::Vec3d controlledPointSensorMm{0.0, 0.0, 0.0};
    double tcpChordToleranceMm = 0.002;   // max midpoint path error per joint segment
    double maxRotaryStepDeg = 0.10;       // adaptive subdivision limit per joint segment
    double maxLinearJointStepMm = 0.50;   // keeps the controller polyline dense
    int maxSubdivisionDepth = 22;
    std::size_t maxSegments = 200000;
};

struct Rtcp17Waypoint {
    Joint q{};                            // exact inverse solution at this task sample
    calibration::Vec3d targetMachineMm;  // desired controlled point
    double cDeg = 0.0;
    double bDeg = 0.0;
    double tcpDistanceFromPreviousMm = 0.0;
    double nominalSeconds = 0.0;
    // Feed value for the existing joint planner.  It is chosen so that the
    // planner's legacy XYZ-joint metric corresponds to nominalSeconds.
    double controllerFeedPerSecond = 0.0;
};

struct Rtcp17Plan {
    bool success = false;
    std::vector<Rtcp17Waypoint> waypoints;
    double requestedTcpSpeedMmPerSec = 0.0;
    double nominalTcpLengthMm = 0.0;
    double nominalSeconds = 0.0;
    double maxForwardErrorMm = 0.0;
    double maxJointLinearStepMm = 0.0;
    double maxRotaryStepDeg = 0.0;
    std::string message;
};

class Rtcp17Model {
public:
    /** Forward position of the configured controlled point. */
    static calibration::Vec3d controlledPointInMachine(
        const Joint& q,
        const calibration::Physical17Parameters& parameters,
        const calibration::Vec3d& controlledPointSensorMm = {});

    /**
     * Exact inverse for X/Y/Z when C and B are prescribed.
     *
     * The full 17-parameter forward model is affine in the three linear-axis
     * coordinates, so no nominal-machine approximation is needed here.
     */
    static bool inverseControlledPointAtAngles(
        const calibration::Vec3d& targetMachineMm,
        double cDeg,
        double bDeg,
        const calibration::Physical17Parameters& parameters,
        Joint& q,
        std::string& error,
        const calibration::Vec3d& controlledPointSensorMm = {});

    static double forwardErrorMm(
        const Joint& q,
        const calibration::Vec3d& targetMachineMm,
        const calibration::Physical17Parameters& parameters,
        const calibration::Vec3d& controlledPointSensorMm = {});
};

class Rtcp17Planner {
public:
    explicit Rtcp17Planner(Rtcp17Options options = {});

    const Rtcp17Options& options() const { return m_options; }
    void setOptions(const Rtcp17Options& options) { m_options = options; }

    /**
     * Plan one task-space line.  Controlled-point XYZ and C/B are all linear
     * in task parameter s.  The returned joint polyline is adaptively refined
     * until the calibrated forward model stays within tcpChordToleranceMm.
     */
    Rtcp17Plan planLine(
        const Joint& startJoint,
        const calibration::Vec3d& endMachineMm,
        double endCDeg,
        double endBDeg,
        double tcpSpeedMmPerSec,
        const calibration::Physical17Parameters& parameters) const;

    /** Same geometry, but with an explicit total nominal duration.  This is used
     * by G93 and by pure C/B compensation moves whose controlled point is held. */
    Rtcp17Plan planLineTimed(
        const Joint& startJoint,
        const calibration::Vec3d& endMachineMm,
        double endCDeg,
        double endBDeg,
        double nominalSeconds,
        const calibration::Physical17Parameters& parameters) const;

private:
    Rtcp17Options m_options;
};

} // namespace mls::runtime
