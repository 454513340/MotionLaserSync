#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <iosfwd>
#include <string>
#include <vector>

namespace mls {
namespace calibration {

/**
 * @brief Small double-precision vector used by the portable calibration core.
 *
 * The class deliberately has no Qt/Eigen dependency so the numerical core can
 * be regression-tested on Linux and reused by the Windows/Qt application.
 */
struct Vec3d
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    Vec3d() = default;
    Vec3d(double vx, double vy, double vz) : x(vx), y(vy), z(vz) {}
};

struct JointPose5d
{
    double x = 0.0;   // mm
    double y = 0.0;   // mm
    double z = 0.0;   // mm
    double c = 0.0;   // controller degree
    double b = 0.0;   // controller degree
};

struct SensorProfilePoint
{
    double x = 0.0;       // sensor X, mm
    double z = 0.0;       // sensor Z, mm; sensor Y is the laser plane y=0
    double weight = 1.0;  // optional inverse-variance-like weight
};

struct SphereProfileObservation
{
    std::int64_t profileId = 0;
    // Pose used by the calibration solver.  V27.6.22+ static Physical17 capture
    // stores measured XYZ/C/B MPOS here.
    JointPose5d joints;

    // Optional controller demand pose captured at the same stationary sample.
    // This is deliberately diagnostic-only: the solver continues to consume
    // `joints` (MPOS), while exported CSV can retain both MPOS and DPOS for an
    // exact A/B comparison of the same laser profiles.
    JointPose5d dposJoints;
    bool hasDposJoints = false;
    std::vector<SensorProfilePoint> points;
};

/**
 * @brief Fourteen data-observable parameters for one unknown fixed sphere.
 *
 * Gauge convention:
 *   - the calibration-frame origin is the optical sensor origin at XYZCB=0;
 *   - hand-eye translation is therefore fixed to [0,0,0];
 *   - the four rotary offsets are the C/B average-axis positions relative to
 *     that origin, expressed in each axis' normal-plane basis;
 *   - the three hand-eye rotation-vector components remain observable.
 *
 * This is NOT a unique decomposition into the traditional 11 physical PIGEs
 * plus a physical 6-DoF hand-eye transform. With one unknown sphere and no
 * independent length datum, those 17 physical quantities have a 3-D gauge.
 * The structure below is the unique 14-D composite model supported by data.
 */
struct Gauge14Parameters
{
    enum Index : std::size_t {
        Sxy = 0,
        Sxz,
        Syz,
        COffsetU,
        COffsetV,
        CTiltX,
        CTiltY,
        BOffsetU,
        BOffsetV,
        BTiltX,
        BTiltZ,
        HandEyeRx,
        HandEyeRy,
        HandEyeRz,
        Count
    };

    std::array<double, Count> value{};

    double& operator[](std::size_t i) { return value[i]; }
    const double& operator[](std::size_t i) const { return value[i]; }

    static const std::array<const char*, Count>& names();
    static const std::array<const char*, Count>& units();
};

struct SphereCenterFit
{
    bool success = false;
    Vec3d centerSensorMm;
    double sectionRadiusMm = 0.0;
    double circleRmsMm = 0.0;
    std::size_t candidatePointCount = 0;
    std::size_t inlierPointCount = 0;
    double inlierRatio = 0.0;
    double arcSpanDeg = 0.0;
    std::string message;
};

/**
 * @brief Robust circle-arc extraction settings for the 19.05 mm sphere.
 *
 * The X/Z ROI is only a coarse software search window. RANSAC is responsible
 * for rejecting background, fixture edges and isolated invalid returns before
 * the known-sphere constraint is used to recover the 3-D center.
 */
struct SphereArcRansacOptions
{
    int maxIterations = 350;
    double inlierThresholdMm = 0.050;
    int minInliers = 10;
    double minInlierRatio = 0.20;
    double minArcSpanDeg = 30.0;
    double maxRadiusOvershootMm = 0.080;
    int geometricRefineIterations = 8;
    std::uint32_t randomSeed = 0x295050u;
};

struct SolverOptions
{
    double sphereDiameterMm = 19.05;
    int sphereCenterYSign = +1;

    int alternatingIterations = 8;
    int blockIterations = 18;
    int jointIterations = 90;
    int multiStartCount = 3;

    double huberDeltaMm = 0.015;
    double finiteDifferenceRelativeStep = 1e-5;
    double convergenceStep = 1e-9;
    double convergenceCost = 1e-12;
    double initialLambda = 1e-3;

    // Scales are used to make the Jacobian dimensionless and well-conditioned.
    std::array<double, Gauge14Parameters::Count + 3> parameterScales{};

    SolverOptions();
};

struct ObservabilityReport
{
    int targetRank = 0;
    int expectedTargetRank = static_cast<int>(Gauge14Parameters::Count);
    double conditionNumber = 0.0;
    double smallestSingularValue = 0.0;
    double largestSingularValue = 0.0;
    std::array<double, Gauge14Parameters::Count> targetSingularValues{};
    std::string message;
};

struct SingleSphere14Result
{
    bool success = false;
    Gauge14Parameters parameters;
    Vec3d sphereCenterGaugeMm;

    double residualRmsMm = 0.0;
    double residualMaxAbsMm = 0.0;
    int alternatingIterations = 0;
    int jointIterations = 0;
    std::size_t profileCount = 0;
    std::size_t pointCount = 0;

    ObservabilityReport observability;
    std::string gaugeDescription;
    std::string message;
};

struct PosePlannerConfig
{
    double sphereDiameterMm = 19.05;

    double bMinDeg = -50.0;
    double bMaxDeg = +50.0;
    double bStepDeg = 15.0;
    double cMinDeg = -70.0;
    double cMaxDeg = +70.0;
    double cStepDeg = 20.0;

    // Conservative central region for LLT2950-50/BL.
    double sensorZMinMm = 80.0;
    double sensorZMaxMm = 110.0;
    double sensorXHalfWidthMm = 18.0;

    // Optional machine-coordinate filter for generated XYZ seeds. Configure
    // these from the real software limits before producing an acquisition plan.
    double xMinMm = -1.0e12;
    double xMaxMm = +1.0e12;
    double yMinMm = -1.0e12;
    double yMaxMm = +1.0e12;
    double zMinMm = -1.0e12;
    double zMaxMm = +1.0e12;

    std::vector<double> targetCenterXmm{-5.0, 0.0, 5.0};
    std::vector<double> targetCenterZmm{88.0, 95.0, 102.0};
    double targetCenterYRadiusRatio = 0.40;
    bool useBothLaserPlaneSides = true;

    // V27.6.17: optional sensor-quality-study geometry.  When enabled the
    // Stage-2 planner constructs candidate sphere centers directly from the
    // same D/gamma/eta definitions used by SphereDistanceAngleStudy:
    //   circleCenter = [D sin(eta), 0, D cos(eta)]
    //   sphereCenter.y = +/- R sin(gamma)
    // This lets re-calibration keep the physical-rank/D-optimal objective while
    // restricting acquisition to the empirically high-quality sensor region.
    // It is disabled by default at the portable core API level for backward
    // compatibility; the integrated GUI enables it with D=92..100 mm.
    bool useQualityStudyGeometry = false;
    double qualityDMinMm = 92.0;
    double qualityDMaxMm = 100.0;
    double qualityDStepMm = 4.0;
    double qualityGammaMinDeg = 0.0;
    double qualityGammaMaxDeg = 50.0;
    double qualityGammaStepDeg = 25.0;
    double qualityEtaMinDeg = -7.5;
    double qualityEtaMaxDeg = +7.5;
    double qualityEtaStepDeg = 7.5;

    // Optional Physical17 acquisition geometry.  Legacy callers retain the
    // historical lower arc.  The Stage-2 GUI enables the SAME physical field
    // and end-effector-facing hemisphere used by its optical preflight.
    bool useEndEffectorFacingArc = false;
    bool useTrapezoidalSensorField = false;
    bool sensorZCoordinateFlipped = false;
    double sensorXSpanNearMm = 42.0;
    double sensorXSpanFarMm = 58.0;
    double sensorXSafetyMarginMm = 0.0;
    double sensorZSafetyMarginMm = 0.0;
    bool reorderRecordedPosesForTravel = true;
    bool requireRequestedPoseCount = false; // legacy/hold-out callers may accept a smaller pool

    int requestedRecordedPoses = 42;
    int syntheticPointsPerProfile = 21;
    int maxRecordedPosesPerBCPair = 2;
    double maxTransitAngleStepDeg = 5.0;
    double informationRidge = 1e-6;
};

struct PosePlannerSeed
{
    // A live/known start pose is useful for choosing the first acquisition pose
    // nearest to the machine's present state and for checking the initial swept
    // transition.  Offline planning deliberately sets this false: path generation
    // remains available from a known machine-frame sphere center, while no fictitious
    // zero/current joint pose is treated as a real motion origin.
    bool hasTaughtJoint = true;
    JointPose5d taughtJoint;
    Vec3d taughtSphereCenterSensorMm;

    // After at least one successful physical-17 solve, the fixed sphere center
    // is already available in the same machine/gauge frame as currentEstimate.
    // Supplying it removes the temporary +/-Y ambiguity of the taught 2-D
    // profile from all subsequent pose planning.  Bootstrap planning leaves
    // this false and treats predicted XYZ as an untrusted seed only.
    bool hasSphereCenterMachine = false;
    Vec3d sphereCenterMachineMm;
};

struct CalibrationPoseInstruction
{
    int sequence = 0;
    bool recordProfile = true;
    JointPose5d predictedJoint;
    Vec3d targetSphereCenterSensorMm;
    double informationGain = 0.0;
    std::string group;
    std::string note;
};

class SingleSphere14Model
{
public:
    /** Sensor point -> gauge-fixed calibrated machine frame. */
    static Vec3d mapSensorPoint(const JointPose5d& joints,
                                const Vec3d& sensorPoint,
                                const Gauge14Parameters& parameters);

    /** 4x4 row-major sensor -> gauge-machine homogeneous transform. */
    static std::array<double, 16> sensorToGaugeMachineTransform(
        const JointPose5d& joints,
        const Gauge14Parameters& parameters);

    /** Fixed sphere center expressed in the sensor frame for a pose. */
    static Vec3d sphereCenterInSensor(const JointPose5d& joints,
                                      const Vec3d& sphereCenterGaugeMm,
                                      const Gauge14Parameters& parameters);
};

class SingleSphere14Calibrator
{
public:
    explicit SingleSphere14Calibrator(const SolverOptions& options = SolverOptions());

    void clear();
    void addObservation(const SphereProfileObservation& observation);
    const std::vector<SphereProfileObservation>& observations() const { return m_observations; }

    static SphereCenterFit fitSphereCenterInSensor(
        const std::vector<SensorProfilePoint>& profilePoints,
        double sphereDiameterMm,
        int centerYSign = +1,
        const SphereArcRansacOptions& ransacOptions = SphereArcRansacOptions(),
        std::vector<std::size_t>* inlierIndices = nullptr);

    SingleSphere14Result solve(const Gauge14Parameters& initialParameters,
                               const Vec3d* initialSphereCenterGaugeMm = nullptr) const;

    ObservabilityReport analyzeObservability(
        const Gauge14Parameters& parameters,
        const Vec3d& sphereCenterGaugeMm) const;

    static bool readObservationCsv(std::istream& input,
                                   std::vector<SphereProfileObservation>& observations,
                                   std::string& error);
    static bool writeResultJson(std::ostream& output,
                                const SingleSphere14Result& result,
                                std::string& error);

private:
    SolverOptions m_options;
    std::vector<SphereProfileObservation> m_observations;
};

class SingleSphere14PosePlanner
{
public:
    explicit SingleSphere14PosePlanner(const PosePlannerConfig& config = PosePlannerConfig());

    /**
     * @brief Greedy D-optimal recorded-pose selection.
     *
     * The taught reference profile supplies an initial estimate of the fixed
     * sphere center. Predicted XYZ values are only acquisition seeds: the real
     * machine must approach in small B/C increments and close the loop using
     * fitSphereCenterInSensor() before recording each profile.
     */
    std::vector<CalibrationPoseInstruction> generate(
        const PosePlannerSeed& seed,
        const Gauge14Parameters& currentEstimate,
        std::string& error) const;

    /**
     * @brief Recommend one bounded XYZ correction for closed-loop sphere centering.
     *
     * B/C are kept unchanged. The measured sphere center is converted to the
     * current gauge frame, a local 3x3 XYZ-to-sensor-center Jacobian is built,
     * and one Newton step is returned. Call repeatedly after reacquiring and
     * refitting the sphere profile; never execute an unverified large jump.
     */
    static bool recommendXYZRecentering(
        const JointPose5d& currentJoint,
        const Vec3d& measuredCenterSensorMm,
        const Vec3d& targetCenterSensorMm,
        const Gauge14Parameters& currentEstimate,
        double maxStepMm,
        JointPose5d& suggestedJoint,
        Vec3d& predictedCenterAfterStepMm,
        std::string& error);

    static bool writeCsv(std::ostream& output,
                         const std::vector<CalibrationPoseInstruction>& poses,
                         std::string& error);

private:
    PosePlannerConfig m_config;
};

} // namespace calibration
} // namespace mls
