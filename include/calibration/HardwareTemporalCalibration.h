#pragma once

#include "calibration/SingleSphere17Calibration.h"
#include "runtime/Acquisition5D.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace mls::calibration {

/**
 * Hardware temporal calibration for the commissioned ECI3808 V32.13 topology:
 *
 *   OP14/PWM0 -> scanCONTROL external trigger
 *   OP14/PWM0 -> ECI IN0 -> REGIST(104) XYZ/C/B MPOS latch
 *
 * Calibrated quantity:
 *   delay = t_sensor_effective - t_eci_latch
 *
 * V27.6.50 keeps the legacy single-axis solver for backward compatibility, but
 * the active workflow is trajectory based and supports slow-drift nuisance estimation.  A calibration pair may follow any
 * straight XYZ direction that keeps the sphere observable.  The two scans are
 * executed in opposite directions and share one global delay.
 */
enum class TemporalAxis : std::uint32_t { X = 0, Y = 1, Z = 2 };

const char* temporalAxisName(TemporalAxis axis);

struct TemporalPoseSample
{
    std::uint64_t ordinal = 0;
    std::array<double,5> joints{}; // XYZ mm, C/B deg; measured MPOS in machine convention
};

struct TemporalMotionEstimate
{
    bool success = false;
    TemporalAxis axis = TemporalAxis::X;
    double signedSpeedMmPerSec = 0.0;
    double speedMadMmPerSec = 0.0;
    double speedMadFraction = 0.0;
    double crossAxisRatio = 0.0;
    double rotaryP80DegPerSec = 0.0;
    std::size_t totalIntervals = 0;
    std::size_t usedIntervals = 0;
    std::string message;
};

/** Robust straight-line XYZ velocity estimate from consecutive REGIST(104) MPOS samples. */
struct TemporalMotionVectorEstimate
{
    bool success = false;
    Vec3d velocityMmPerSec;
    double speedMmPerSec = 0.0;
    double speedMadMmPerSec = 0.0;
    double speedMadFraction = 0.0;
    double crossTrackP80Fraction = 0.0;
    double rotaryP80DegPerSec = 0.0;
    std::size_t totalIntervals = 0;
    std::size_t usedIntervals = 0;
    std::string message;
};

struct KnownSphereCenterFit
{
    bool success = false;
    Vec3d centerMachineMm;
    double knownRadiusMm = 0.0;
    double residualRmsMm = 0.0;
    double residualMaxAbsMm = 0.0;
    std::size_t pointCount = 0;
    std::string message;
};

struct FreeSphereFit
{
    bool success = false;
    Vec3d centerMachineMm;
    double radiusMm = 0.0;
    double residualRmsMm = 0.0;
    double residualMaxAbsMm = 0.0;
    std::size_t pointCount = 0;
    std::string message;
};

// Legacy V27.6.44 observation/solver. Retained to load old JSON and keep the
// numerical regression tests/source compatibility intact.
struct HardwareTemporalObservation
{
    std::uint64_t id = 0;
    TemporalAxis axis = TemporalAxis::X;
    double signedSpeedMmPerSec = 0.0;
    Vec3d sphereCenterMachineMm;
    double sphereRmsMm = 0.0;
    double fittedRadiusMm = 0.0;
    std::size_t sphereInliers = 0;
    std::size_t profileCount = 0;
    unsigned resolutionIndex = 0;
    unsigned exposureTime10Us = 0;
    unsigned idleTime10Us = 0;
    int measuringFieldIndex = 7;
    int triggerFrequencyHz = mls::runtime::Table5D::defaultTriggerFrequencyHz;
    double triggerPeriodMs = 1000.0 / static_cast<double>(mls::runtime::Table5D::defaultTriggerFrequencyHz);
};

struct TemporalAxisFit
{
    TemporalAxis axis = TemporalAxis::X;
    bool valid = false;
    std::size_t observationCount = 0;
    std::size_t positiveCount = 0;
    std::size_t negativeCount = 0;
    std::size_t distinctAbsSpeedLevels = 0;
    double interceptMm = 0.0;
    double slopeSec = 0.0;
    double delaySec = 0.0;
    double delayStdSec = std::numeric_limits<double>::infinity();
    double residualRmsMm = 0.0;
    double rSquared = 0.0;
    double minSignedSpeedMmPerSec = 0.0;
    double maxSignedSpeedMmPerSec = 0.0;
};

struct HardwareTemporalCalibrationResult
{
    bool success = false;
    bool configurationConsistent = false;
    bool qualityPassed = false;
    double delaySec = 0.0;
    double delayStdSec = std::numeric_limits<double>::infinity();
    double axisSpreadSec = std::numeric_limits<double>::infinity();
    double residualRmsMm = 0.0;
    std::array<TemporalAxisFit,3> axes{};
    std::size_t activeAxisCount = 0;
    std::size_t observationCount = 0;
    unsigned resolutionIndex = 0;
    unsigned exposureTime10Us = 0;
    unsigned idleTime10Us = 0;
    int measuringFieldIndex = 7;
    int triggerFrequencyHz = mls::runtime::Table5D::defaultTriggerFrequencyHz;
    double triggerPeriodMs = 1000.0 / static_cast<double>(mls::runtime::Table5D::defaultTriggerFrequencyHz);
    std::string message;
    std::string qualityMessage;
};

struct HardwareTemporalCriteria
{
    std::size_t minAxes = 3;
    std::size_t minObservationsPerAxis = 4;
    double minRSquared = 0.40;
    double maxAxisSpreadSec = 0.00030;
    double maxDelayStdSec = 0.00025;
    double maxDelayFractionOfPeriod = 0.95;
};

/**
 * One completed dynamic scan in the V27.6.47 trajectory workflow.
 * pairId identifies the forward/reverse pair generated from the same path
 * configuration and repeat. directionSign is +1 for start->end and -1 for the
 * exact reverse. velocity is measured from REGIST(104), never from command feed.
 */
struct TemporalTrajectoryObservation
{
    std::uint64_t id = 0;
    std::uint64_t planId = 0;
    std::uint64_t pairId = 0;
    int directionSign = 0;
    Vec3d velocityMmPerSec;
    Vec3d sphereCenterMachineMm;
    double sphereRmsMm = 0.0;
    double fittedRadiusMm = 0.0;
    std::size_t sphereInliers = 0;
    std::size_t profileCount = 0;
    unsigned resolutionIndex = 0;
    unsigned exposureTime10Us = 0;
    unsigned idleTime10Us = 0;
    int measuringFieldIndex = 7;
    int triggerFrequencyHz = mls::runtime::Table5D::defaultTriggerFrequencyHz;
    double triggerPeriodMs = 1000.0 / static_cast<double>(mls::runtime::Table5D::defaultTriggerFrequencyHz);
    std::int64_t captureMidPcUs = 0; // host-clock midpoint; drift nuisance only, never exposure/latch timing truth
};

struct TemporalTrajectoryPairFit
{
    std::uint64_t pairId = 0;
    bool valid = false;
    Vec3d deltaVelocityMmPerSec; // v_forward - v_reverse
    Vec3d deltaCenterMm;         // c_forward - c_reverse
    double meanSpeedMmPerSec = 0.0;
    double pairDelaySec = 0.0;
    Vec3d modelResidualMm;
    double residualNormMm = 0.0;
    double meanEffectiveRadiusMm = 0.0;
    double radiusDifferenceMm = 0.0;
    double pairTimeDeltaSec = 0.0;
};

struct TemporalTrajectoryCalibrationResult
{
    bool success = false;
    bool configurationConsistent = false;
    bool qualityPassed = false;
    double delaySec = 0.0;
    double delayStdSec = std::numeric_limits<double>::infinity();
    double residualRmsMm = std::numeric_limits<double>::infinity();
    double rSquared = 0.0;
    bool driftModelUsed = false;
    Vec3d centerDriftMmPerSec;
    double timeSpanSec = 0.0;
    double naivePairDelaySec = 0.0;
    double maxPairRadiusDifferenceMm = 0.0;
    double meanEffectiveRadiusMm = 0.0;
    std::size_t pairCount = 0;
    std::size_t distinctSpeedLevels = 0;
    std::size_t observationCount = 0;
    std::vector<TemporalTrajectoryPairFit> pairs;
    unsigned resolutionIndex = 0;
    unsigned exposureTime10Us = 0;
    unsigned idleTime10Us = 0;
    int measuringFieldIndex = 7;
    int triggerFrequencyHz = mls::runtime::Table5D::defaultTriggerFrequencyHz;
    double triggerPeriodMs = 1000.0 / static_cast<double>(mls::runtime::Table5D::defaultTriggerFrequencyHz);
    std::string message;
    std::string qualityMessage;
};

struct TemporalTrajectoryCriteria
{
    std::size_t minPairs = 3;
    std::size_t minDistinctSpeedLevels = 2;
    double minRSquared = 0.40;
    double maxDelayStdSec = 0.00025;
    double maxPairResidualMm = 0.050;
    double maxPairRadiusDifferenceMm = 0.050;
    double maxDelayFractionOfPeriod = 0.95;
    // H3 CMM/R1 may latch AFTER effective exposure; H1 remains nonnegative.
    bool allowSignedDelay = false;
};

struct TemporalValidationObservation
{
    std::uint64_t id = 0;
    std::uint64_t pairId = 0;
    int directionSign = 0;
    Vec3d velocityMmPerSec;
    Vec3d rawSphereCenterMachineMm;
    Vec3d correctedSphereCenterMachineMm;
    double rawSphereRmsMm = 0.0;
    double correctedSphereRmsMm = 0.0;
    std::int64_t captureMidPcUs = 0;
};

struct TemporalValidationResult
{
    bool success = false;
    bool passed = false;
    std::size_t pairCount = 0;
    double rawEquivalentDelaySec = 0.0;
    double correctedResidualDelaySec = 0.0;
    double improvementFraction = 0.0;
    double rawPairResidualRmsMm = 0.0;
    double correctedPairResidualRmsMm = 0.0;
    std::string message;
};

struct TemporalValidationCriteria
{
    std::size_t minPairs = 2;
    double minImprovementFraction = 0.50;
    double maxResidualDelaySec = 0.00015;
};

class HardwareTemporalCalibration final
{
public:
    static TemporalMotionEstimate estimateDominantLinearMotion(
        const std::vector<TemporalPoseSample>& poses,
        double triggerPeriodSec);

    static TemporalMotionVectorEstimate estimateLinearMotionVector(
        const std::vector<TemporalPoseSample>& poses,
        double triggerPeriodSec);

    static KnownSphereCenterFit refineKnownSphereCenter(
        const std::vector<Vec3d>& points,
        const Vec3d& initialCenterMachineMm,
        double knownRadiusMm,
        double huberThresholdMm = 0.12,
        int maxIterations = 20);

    // Dynamic scans may be geometrically deformed by residual hand-eye / kinematic
    // errors.  V27.6.50 therefore estimates an effective free radius together with
    // the center; the physical nominal radius remains a reference/diagnostic only.
    static FreeSphereFit refineFreeSphere(
        const std::vector<Vec3d>& points,
        const Vec3d& initialCenterMachineMm,
        double initialRadiusMm,
        double huberThresholdMm = 0.12,
        int maxIterations = 20);

    // Legacy V27.6.44 API.
    static HardwareTemporalCalibrationResult solve(
        const std::vector<HardwareTemporalObservation>& observations);
    static bool evaluateQuality(HardwareTemporalCalibrationResult& result,
                                const HardwareTemporalCriteria& criteria);

    // V27.6.47 trajectory-based pair solver. The physical equation is
    //   (c+ - c-) = -delay * (v+ - v-)
    // using actual velocities from hardware-latched MPOS.
    static TemporalTrajectoryCalibrationResult solveTrajectoryPairs(
        const std::vector<TemporalTrajectoryObservation>& observations);
    static bool evaluateTrajectoryQuality(TemporalTrajectoryCalibrationResult& result,
                                          const TemporalTrajectoryCriteria& criteria);

    // Hold-out validation compares the equivalent delay of raw and offline
    // corrected sphere centers for independent trajectory pairs.
    static TemporalValidationResult evaluateValidationPairs(
        const std::vector<TemporalValidationObservation>& observations,
        const TemporalValidationCriteria& criteria);
};

} // namespace mls::calibration
