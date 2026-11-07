#pragma once

#include "calibration/SingleSphere17Calibration.h"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace mls {
namespace calibration {

/** Robust 3-D sphere extraction from a reconstructed machine-frame point cloud. */
struct SphereCloudRansacOptions
{
    int maxIterations = 1200;
    double inlierThresholdMm = 0.12;
    int minInliers = 200;
    double minInlierRatio = 0.05;
    double expectedRadiusMm = 9.525;
    double maxRadiusDeviationMm = 1.50;
    int refineIterations = 12;
    std::size_t maxRansacWorkingPoints = 50000;
    std::size_t maxRefinePoints = 100000;
    std::uint32_t randomSeed = 0x29505017u;
};

struct SphereCloudFitResult
{
    bool success = false;
    Vec3d centerMachineMm;
    double radiusMm = 0.0;
    double residualMeanMm = 0.0;
    double residualMeanAbsMm = 0.0;
    double residualRmsMm = 0.0;
    double residualMaxAbsMm = 0.0;
    std::size_t candidatePointCount = 0;
    std::size_t inlierPointCount = 0;
    double inlierRatio = 0.0;
    std::vector<std::size_t> inlierIndices;
    std::string message;
};

/**
 * Study geometry introduced in V27.6.16V, extended by V27.6.16Y, and optically gated in V27.6.16Z.
 *
 * MeasurementDistance D:
 *   Euclidean distance from the declared sensor optical reference center to
 *   the fitted 2-D section-circle center in the sensor profile plane.
 *
 * SectionAngle gamma:
 *   Unsigned eccentric section angle of the laser profile plane through the
 *   known sphere.  If h is the perpendicular distance from the known sphere
 *   center to the current profile plane, gamma = asin(h/R), 0..90 deg.
 *
 * CenterBearingAngle eta:
 *   Direction of the fitted section-circle center in the sensor X-Z profile
 *   plane, measured from +Z toward +X:
 *       eta = atan2(Xc-X0, Zc-Z0).
 *   D and eta together determine the full circle-center position relative to
 *   the declared optical reference center.  eta=0 is only the special case in
 *   which the circle center lies on the sensor +Z axis.
 *
 * The path target places the section-circle center at the requested (D,eta)
 * position in the sensor X-Z plane and offsets the sphere center by h along
 * sensor +/-Y.  The Y sign is only a path-continuity choice; it is NOT an
 * experiment variable and is never used as a quality feature.
 */
struct SphereStudyTargetGeometry
{
    double measurementDistanceMm = 0.0;
    double sectionAngleDeg = 0.0;
    double centerBearingAngleDeg = 0.0;
    int sectionSideSign = +1;
    Vec3d targetCircleCenterSensorMm;
    Vec3d targetSphereCenterSensorMm;
    double theoreticalSectionRadiusMm = 0.0;
};

struct SphereStudyRotarySample
{
    int id = 0;
    double cDeg = 0.0;
    double bDeg = 0.0;
    // Sensor +Y expressed in machine coordinates.  This is the normal of the
    // laser profile plane.  In V27.6.16Y the automatic plane-family planner
    // selects rotary samples by spreading this normal over the reachable
    // orientation manifold, rather than forming a blind C x B Cartesian grid.
    Vec3d planeNormalMachine{0.0, 0.0, 0.0};
};

struct SphereStudyPathOptions
{
    std::vector<double> measurementDistancesMm;
    std::vector<double> sectionAnglesDeg;
    std::vector<double> centerBearingAnglesDeg;
    std::vector<SphereStudyRotarySample> rotarySamples;

    // Recommended V27.6.16Y path mode.  When rotarySamples is empty and this
    // flag is true, the planner builds a dense set of C/B candidates inside
    // the configured software limits, keeps candidates that can realize the
    // central D/gamma geometry, then uses maximin angular selection on the
    // laser-plane normal (sensor +Y in machine coordinates).  The result is a
    // small family of physically diverse, reachable section planes.
    bool automaticPlaneFamily = false;
    // Number of diverse reachable plane-orientation candidates retained before
    // the final D/gamma/eta/plane space-filling selection.
    int planeDirectionsPerGeometry = 8;
    double rotaryCandidateStepDeg = 10.0;
    // Hard guard for the automatic C/B search lattice.  This protects the GUI
    // from placeholder/offline limits such as +/-999999 ever turning into an
    // effectively unbounded nested loop.  The MainWindow normally supplies the
    // finite Machine5X offline planning limits, but the numerical core remains
    // fail-fast even when called directly or by a future UI.
    std::size_t maxAutomaticRotaryCandidateCount = 2048;
    double minimumPlaneSeparationDeg = 8.0;
    // V27.6.16Y: build all feasible D/gamma/eta/plane candidates first, then
    // choose a compact maximin subset in normalized study-geometry space.
    bool spaceFillingSampling = true;
    int targetPoseCount = 120;
    double sphereRadiusMm = 9.525;
    Vec3d opticalCenterSensorMm{0.0, 0.0, 0.0};
    int sectionSideSign = +1;

    // Used only as the Newton seed for XYZ.  C/B are taken from each explicit
    // rotary sample, so the study can intentionally cover multiple C/B poses.
    JointPose5d referenceJoint;

    double xMinMm = -1.0e12;
    double xMaxMm = +1.0e12;
    double yMinMm = -1.0e12;
    double yMaxMm = +1.0e12;
    double zMinMm = -1.0e12;
    double zMaxMm = +1.0e12;
    double cMinDeg = -90.0;
    double cMaxDeg = +90.0;
    double bMinDeg = -90.0;
    double bMaxDeg = +90.0;

    // scanCONTROL 2950-50BL working-Z guard used only for target feasibility.
    double sensorZMinMm = 70.0;
    double sensorZMaxMm = 120.0;
    double sensorZSafetyMarginMm = 1.0;
    // scanCONTROL 2950-50BL standard measuring field.  Physically the field is
    // a trapezoid whose SMALL edge is closest to the sensor and whose LARGE
    // edge is farthest from it (42/50/58 mm over the 50 mm depth range).
    //
    // IMPORTANT: the project's calibrated profile convention normally has
    // PROC_FLIP_DISTANCE / FlipZ enabled.  In that processed Sensor frame the
    // numerical Z ordering is opposite to physical optical depth: the
    // near-sensor edge is at the larger reported Z and the far edge is at the
    // smaller reported Z.  Do not bind physical near/far to numeric ZMin/ZMax.
    // sensorXMin/Max retain the mid-range ±25 mm convention for compatibility.
    double sensorXMinMm = -25.0;
    double sensorXMaxMm = +25.0;
    double sensorXSafetyMarginMm = 1.0;
    bool useTrapezoidalSensorField = true;
    // Coordinate convention of the profile data used by the hand-eye model.
    // true => reported Sensor Z is flipped relative to physical optical depth,
    //         therefore near edge = sensorZMaxMm and far edge = sensorZMinMm.
    // false => near edge = sensorZMinMm and far edge = sensorZMaxMm.
    bool sensorZCoordinateFlipped = true;
    // Physical field widths.  These are intentionally named by optical depth,
    // not by numerical Sensor-Z, so FlipZ cannot silently reverse the trapezoid.
    double sensorXSpanNearMm = 42.0;
    double sensorXSpanFarMm = 58.0;

    // Legacy aliases retained so older callers/tests that explicitly populate
    // them still compile.  New code must use sensorXSpanNear/FarMm.  They are
    // ignored by the trapezoid implementation from V27.6.16AF onward.
    double sensorXSpanAtZMinMm = 42.0;
    double sensorXSpanAtZMaxMm = 58.0;

    // Visible-arc side rule.  A geometric section circle contains both the
    // near and far sides of the sphere.  The real line sensor mounted on the
    // five-axis head can only contribute the contour on the sphere hemisphere
    // facing the kinematic end-effector.  Do NOT infer this from machine +Z or
    // from an arbitrary optical reference point; derive the end-effector origin
    // from the calibrated ^E T_S hand-eye transform and keep the hemisphere for
    // which (P-Csphere) dot (E-Csphere) > 0.
    bool requireEndEffectorFacingArc = true;

    // Optical feasibility gate applied AFTER inverse kinematics and a forward
    // simulation of the solved XYZBC pose.  This prevents a mathematically
    // reachable joint pose from entering the autonomous queue when the laser
    // plane misses the known sphere or only a tiny circle arc is predicted to
    // remain inside the configured scanCONTROL X-Z measuring field.
    double minimumPlaneIntersectionMarginMm = 0.50;
    double minimumPredictedVisibleArcDeg = 75.0;
    // Angular span alone is insufficient near a very small section circle.
    // Reject a pose unless the physically visible arc also has enough length
    // to give Circle-RANSAC useful support in the real profile.
    double minimumPredictedVisibleArcLengthMm = 6.0;
    int visibleArcSamples = 720;

    // V27.6.16AB: if the original D/gamma/eta grid cannot provide enough
    // optically useful candidates, insert deterministic mid-points between
    // the requested study values and try again.  Replacement poses always
    // remain inside the user-declared study range and still pass the exact
    // same forward optical gate.
    bool adaptiveOpticalResampling = true;
    int adaptiveOpticalResamplingPasses = 2;
    std::size_t maxAdaptiveCandidateCount = 24000;

    int maxNewtonIterations = 18;
    double xyzFiniteDifferenceMm = 0.01;
    double targetToleranceMm = 1.0e-5;
    double maxNewtonStepMm = 8.0;
    bool serpentine = true;
};

struct SphereStudyPose
{
    bool valid = false;
    int sequence = -1;
    int rotarySampleId = -1;
    SphereStudyTargetGeometry geometry;
    JointPose5d joints;
    Vec3d predictedSphereCenterSensorMm;
    Vec3d predictedCircleCenterSensorMm;
    double predictedSectionRadiusMm = 0.0;
    double predictedVisibleArcDeg = 0.0;
    double predictedVisibleArcLengthMm = 0.0;
    double predictedPlaneIntersectionMarginMm = 0.0;
    double solveResidualMm = 0.0;
    bool adaptivelyRegenerated = false;
    std::string message;
};

/** Forward-simulated optical feasibility of one XYZBC study pose. */
struct SphereStudyOpticalFeasibility
{
    bool valid = false;
    bool planeIntersectsSphere = false;
    bool visibleArcSufficient = false;
    Vec3d sphereCenterSensorMm;
    Vec3d circleCenterSensorMm;
    double sectionRadiusMm = 0.0;
    double planeIntersectionMarginMm = 0.0;
    double visibleArcDeg = 0.0;
    double visibleArcLengthMm = 0.0;
    double visibleChordMm = 0.0;
    double visibleFraction = 0.0;
    // End-effector origin expressed in Sensor coordinates from calibrated
    // ^E T_S.  The accepted contour must lie on the sphere hemisphere facing
    // this point; this makes the rule invariant to machine-axis orientation.
    Vec3d endEffectorOriginSensorMm;
    double minimumEndEffectorSideProjectionMm2 = 0.0;
    std::string message;
};

/** Geometry used by the GUI digital-twin scan demonstration. */
struct SphereStudyScanSimulation
{
    bool valid = false;
    SphereStudyOpticalFeasibility optical;
    // Guarded scanCONTROL X-Z trapezoidal measuring field, expressed in
    // Sensor frame. Four points are ordered around the polygon; Y is zero.
    std::vector<Vec3d> laserFieldCornersSensorMm;
    // Complete geometric sphere/laser-plane section circle when an
    // intersection exists.  The GUI renders this independently from the
    // actually predicted visible arc.
    std::vector<Vec3d> sectionCircleSensorMm;
    // Longest contiguous end-effector-facing arc that lies inside the guarded
    // trapezoidal measuring field.
    std::vector<Vec3d> visibleArcSensorMm;
    std::string message;
};

/** Lightweight real-profile gate used before an autonomous 20-profile capture. */
struct SphereStudyProbeResult
{
    bool success = false;
    int profileCount = 0;
    int detectedCircleCount = 0;
    double detectionRatio = 0.0;
    double medianCenterXmm = 0.0;
    double medianCenterZmm = 0.0;
    double medianRadiusMm = 0.0;
    double medianArcSpanDeg = 0.0;
    std::string message;
};

/** Model-based geometry available before a profile is fitted. */
struct SphereStudyObservedGeometry
{
    bool valid = false;
    double measurementDistanceMm = 0.0;
    double sectionAngleDeg = 0.0;
    double centerBearingAngleDeg = 0.0;
    double sectionPlaneOffsetMm = 0.0;
    double theoreticalSectionRadiusMm = 0.0;
    Vec3d sphereCenterSensorMm;
    Vec3d circleCenterSensorMm;
    std::string message;
};

/** Full raw slot. Invalid coordinates are deliberately retained. */
struct SphereStudyRawSample
{
    std::size_t index = 0;
    bool coordinateValid = false;
    double xMm = std::numeric_limits<double>::quiet_NaN();
    double zMm = std::numeric_limits<double>::quiet_NaN();

    bool hasWidth = false;
    bool hasMaximum = false;
    bool hasThreshold = false;
    bool hasMoment0 = false;
    bool hasMoment1 = false;
    double width = std::numeric_limits<double>::quiet_NaN();
    double maximum = std::numeric_limits<double>::quiet_NaN();
    double threshold = std::numeric_limits<double>::quiet_NaN();
    double moment0 = std::numeric_limits<double>::quiet_NaN();
    double moment1 = std::numeric_limits<double>::quiet_NaN();
};

struct SphereStudyRawProfile
{
    std::uint64_t callbackOrdinal = 0;
    std::uint32_t profileIndex = 0;
    double tOpenSec = 0.0;
    double tCloseSec = 0.0;
    std::vector<SphereStudyRawSample> samples;
};

struct RobustScalarStats
{
    bool valid = false;
    std::size_t count = 0;
    double median = 0.0;
    double mad = 0.0;
    double p10 = 0.0;
    double p90 = 0.0;
};

struct SphereStudyPointMetrics
{
    std::size_t sampleIndex = 0;
    bool inEvaluationRoi = false;
    bool coordinateValid = false;
    bool geometricGood = false;
    bool geometricOutlier = false;
    bool dropout = false;

    double measuredXmm = std::numeric_limits<double>::quiet_NaN();
    double measuredZmm = std::numeric_limits<double>::quiet_NaN();
    double expectedXmm = std::numeric_limits<double>::quiet_NaN();
    double expectedZmm = std::numeric_limits<double>::quiet_NaN();
    double localSurfaceAngleDeg = std::numeric_limits<double>::quiet_NaN();
    double shapeResidualMm = std::numeric_limits<double>::quiet_NaN();
    double absoluteSectionResidualMm = std::numeric_limits<double>::quiet_NaN();

    bool hasWidth = false;
    bool hasMaximum = false;
    bool hasThreshold = false;
    bool hasMoment0 = false;
    bool hasMoment1 = false;
    double width = std::numeric_limits<double>::quiet_NaN();
    double maximum = std::numeric_limits<double>::quiet_NaN();
    double threshold = std::numeric_limits<double>::quiet_NaN();
    double moment0 = std::numeric_limits<double>::quiet_NaN();
    double moment1 = std::numeric_limits<double>::quiet_NaN();
};

struct SphereStudyProfileAnalysisOptions
{
    // First-stage RANSAC threshold.  The second stage shrinks it from residual
    // MAD, but never below adaptiveMinThresholdMm and never above this value.
    SphereArcRansacOptions ransac;
    double adaptiveMinThresholdMm = 0.008;
    double adaptiveMadMultiplier = 4.0;
    double predictedSearchMarginMm = 3.0;
    double evaluationEdgeMarginMm = 0.15;
    int minimumConsensusProfiles = 3;
};

struct SphereStudyProfileMetrics
{
    bool valid = false;
    bool circleDetected = false;
    std::uint64_t callbackOrdinal = 0;
    std::uint32_t profileIndex = 0;

    int evaluationStartIndex = -1;
    int evaluationEndIndex = -1;
    std::size_t evaluationPointCount = 0;
    std::size_t coordinateValidCount = 0;
    std::size_t goodPointCount = 0;
    std::size_t geometricOutlierCount = 0;
    std::size_t dropoutCount = 0;
    double goodPointRatio = 0.0;
    double geometricOutlierRatio = 0.0;
    double dropoutRatio = 0.0;
    int maxDropoutRun = 0;
    int dropoutRunCount = 0;
    int maxOutlierRun = 0;

    double measurementDistanceMm = 0.0;
    double sectionAngleDeg = 0.0;
    double centerBearingAngleDeg = 0.0;
    double sectionPlaneOffsetMm = 0.0;
    double fittedRadiusMm = 0.0;
    double theoreticalRadiusMm = 0.0;
    double radiusBiasMm = 0.0;
    double arcSpanDeg = 0.0;
    double adaptiveInlierThresholdMm = 0.0;

    double circleRmsSensorMm = 0.0;
    double circleMadSensorMm = 0.0;
    double shapeP95Mm = 0.0;
    double shapeMaxAbsMm = 0.0;
    double absoluteSectionMeanAbsMm = 0.0;
    double absoluteSectionRmsMm = 0.0;
    double absoluteSectionP95Mm = 0.0;
    double absoluteSectionMaxAbsMm = 0.0;

    Vec3d fittedCircleCenterSensorMm;
    Vec3d fittedCircleCenterMachineMm;

    RobustScalarStats maximumAllValid;
    RobustScalarStats maximumGood;
    RobustScalarStats widthAllValid;
    RobustScalarStats widthGood;
    RobustScalarStats moment0AllValid;
    RobustScalarStats moment0Good;
    RobustScalarStats moment1AllValid;
    RobustScalarStats moment1Good;
    RobustScalarStats thresholdAllValid;
    RobustScalarStats thresholdGood;

    std::vector<SphereStudyPointMetrics> points;
    std::string message;
};

struct SphereStudyGroupAnalysis
{
    bool valid = false;
    int evaluationStartIndex = -1;
    int evaluationEndIndex = -1;
    double consensusCenterXmm = 0.0;
    double consensusCenterZmm = 0.0;
    double consensusRadiusMm = 0.0;
    int visibleArcBranchSign = +1;
    std::vector<SphereStudyProfileMetrics> profiles;
    std::string message;
};

/** Observation used by the non-parametric D-gamma-eta quality map. */
struct SphereStudyQualityObservation
{
    double measurementDistanceMm = 0.0;
    double sectionAngleDeg = 0.0;
    double centerBearingAngleDeg = 0.0;
    double absoluteP95Mm = 0.0;
    double radiusBiasMm = 0.0;
    double dropoutRatio = 0.0;
    double geometricOutlierRatio = 0.0;
    double goodPointRatio = 0.0;
};

struct SphereStudyQualityPrediction
{
    bool valid = false;
    bool lowSupport = true;
    double queryDistanceMm = 0.0;
    double querySectionAngleDeg = 0.0;
    double queryCenterBearingAngleDeg = 0.0;
    double predictedAbsoluteP95Mm = 0.0;
    double predictedRadiusBiasMm = 0.0;
    double predictedDropoutRatio = 0.0;
    double predictedOutlierRatio = 0.0;
    double predictedGoodPointRatio = 0.0;
    double localSpreadMm = 0.0;
    double effectiveSampleCount = 0.0;
    double nearestNormalizedDistance = 0.0;
};

class SphereDistanceAngleStudy
{
public:
    static SphereCloudFitResult fitMachineSphereRansac(
        const std::vector<Vec3d>& points,
        const SphereCloudRansacOptions& options = SphereCloudRansacOptions());

    static SphereStudyTargetGeometry targetGeometry(
        double sphereRadiusMm,
        double measurementDistanceMm,
        double sectionAngleDeg,
        double centerBearingAngleDeg,
        int sectionSideSign = +1,
        const Vec3d& opticalCenterSensorMm = Vec3d(0.0, 0.0, 0.0));

    /** Backward-compatible eta=0 target constructor. */
    static SphereStudyTargetGeometry targetGeometry(
        double sphereRadiusMm,
        double measurementDistanceMm,
        double sectionAngleDeg,
        int sectionSideSign = +1,
        const Vec3d& opticalCenterSensorMm = Vec3d(0.0, 0.0, 0.0))
    {
        return targetGeometry(sphereRadiusMm, measurementDistanceMm,
                              sectionAngleDeg, 0.0, sectionSideSign,
                              opticalCenterSensorMm);
    }

    static SphereStudyObservedGeometry observeGeometry(
        const JointPose5d& joints,
        const Vec3d& sphereCenterMachineMm,
        double sphereRadiusMm,
        const Physical17Parameters& model,
        const Vec3d& opticalCenterSensorMm = Vec3d(0.0, 0.0, 0.0));

    /**
     * Forward-simulate whether the laser plane really intersects the known
     * sphere and how much of the end-effector-facing section arc lies inside
     * the configured scanCONTROL X-Z trapezoidal measuring field.
     */
    static SphereStudyOpticalFeasibility simulateOpticalFeasibility(
        const JointPose5d& joints,
        const Vec3d& sphereCenterMachineMm,
        double sphereRadiusMm,
        const Physical17Parameters& model,
        const SphereStudyPathOptions& options);

    /**
     * Build the actual sensor X-Z trapezoidal field, sphere/plane section
     * circle and longest end-effector-facing predicted arc for digital-twin
     * visualization.  This
     * uses the same optical rules as simulateOpticalFeasibility().
     */
    static SphereStudyScanSimulation buildScanSimulation(
        const JointPose5d& joints,
        const Vec3d& sphereCenterMachineMm,
        double sphereRadiusMm,
        const Physical17Parameters& model,
        const SphereStudyPathOptions& options,
        int circleSamples = 360);

    /**
     * Fixed-C/B RTCP-equivalent XYZ inverse used by local sphere reacquisition.
     * This intentionally shares the same joint-limit/forward-residual checks as
     * the path planner.
     */
    static bool solvePoseForSphereCenterTarget(
        const Vec3d& sphereCenterMachineMm,
        const Physical17Parameters& model,
        const SphereStudyPathOptions& options,
        const Vec3d& targetSphereCenterSensorMm,
        double targetCDeg, double targetBDeg,
        const JointPose5d& seed,
        JointPose5d& solved,
        Vec3d& predictedSphereCenterSensorMm,
        double& residualMm,
        std::string& error);

    /**
     * Fast multi-frame Circle-RANSAC gate for autonomous acquisition.  It does
     * not delete/drop poor points from the formal experiment; it merely asks
     * whether enough probe frames contain a usable sphere arc before the
     * 20-profile research capture is started.
     */
    static SphereStudyProbeResult probeProfileGroup(
        const std::vector<SphereStudyRawProfile>& rawProfiles,
        const Vec3d& predictedSphereCenterSensorMm,
        double sphereRadiusMm,
        int centerYSign,
        const SphereStudyProfileAnalysisOptions& options,
        int minimumDetectedProfiles = 3);

    static std::vector<SphereStudyPose> generatePath(
        const Vec3d& sphereCenterMachineMm,
        const Physical17Parameters& model,
        const SphereStudyPathOptions& options,
        std::string& error);

    /**
     * Analyze one static visit/group. RANSAC only establishes the circle and
     * EvaluationROI. Every raw slot inside that ROI is retained and classified
     * as good, geometric outlier or dropout.
     */
    static SphereStudyGroupAnalysis analyzeProfileGroup(
        const JointPose5d& actualJoints,
        const std::vector<SphereStudyRawProfile>& rawProfiles,
        const Vec3d& sphereCenterMachineMm,
        double sphereRadiusMm,
        const Physical17Parameters& model,
        const Vec3d& opticalCenterSensorMm,
        const SphereStudyProfileAnalysisOptions& options = SphereStudyProfileAnalysisOptions());

    /** Backward-compatible X/Z-only wrapper used by portable tests/tools. */
    static SphereStudyProfileMetrics analyzeProfile(
        const JointPose5d& actualJoints,
        const std::vector<SensorProfilePoint>& rawProfilePoints,
        const Vec3d& sphereCenterMachineMm,
        double sphereRadiusMm,
        const Physical17Parameters& model,
        const SphereArcRansacOptions& arcOptions = SphereArcRansacOptions(),
        double roiMarginMm = 2.0);

    /** Interpretable local Gaussian-kernel regression for an empirical D-gamma-eta map. */
    static SphereStudyQualityPrediction predictQualityKernel(
        const std::vector<SphereStudyQualityObservation>& observations,
        double queryDistanceMm,
        double querySectionAngleDeg,
        double queryCenterBearingAngleDeg,
        double distanceBandwidthMm,
        double sectionAngleBandwidthDeg,
        double centerBearingBandwidthDeg);

    /** Backward-compatible eta=0 2-D kernel query. */
    static SphereStudyQualityPrediction predictQualityKernel(
        const std::vector<SphereStudyQualityObservation>& observations,
        double queryDistanceMm,
        double querySectionAngleDeg,
        double distanceBandwidthMm,
        double angleBandwidthDeg)
    {
        return predictQualityKernel(observations, queryDistanceMm,
                                    querySectionAngleDeg, 0.0,
                                    distanceBandwidthMm,
                                    angleBandwidthDeg, 15.0);
    }
};

} // namespace calibration
} // namespace mls
