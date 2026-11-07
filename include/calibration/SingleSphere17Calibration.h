#pragma once

#include "calibration/SingleSphere14Calibration.h"

#include <array>
#include <cstddef>
#include <atomic>
#include <functional>
#include <iosfwd>
#include <string>
#include <vector>

namespace mls {
namespace calibration {

/**
 * @brief Full physical 17-parameter five-axis + hand-eye model.
 *
 * Parameter order:
 *   11 machine geometric parameters + 6-DoF sensor hand-eye transform.
 *
 * IMPORTANT: with one fixed sphere whose machine-frame center is unknown, the
 * 17 physical parameters have only rank 14 after eliminating the three sphere
 * center nuisance variables.  The three missing directions are an exact
 * translation gauge mixing C/B axis-line locations, hand-eye translation and
 * sphere-center translation.  The model therefore retains all 17 quantities,
 * while the solver anchors hand-eye translation to its supplied initial value.
 * That anchor is a gauge choice / external prior, not a data-identified result.
 */
struct Physical17Parameters
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
        HandEyeTx,
        HandEyeTy,
        HandEyeTz,
        Count
    };

    std::array<double, Count> value{};
    double& operator[](std::size_t i) { return value[i]; }
    const double& operator[](std::size_t i) const { return value[i]; }

    static const std::array<const char*, Count>& names();
    static const std::array<const char*, Count>& units();
};

struct Physical17SolverOptions
{
    double sphereDiameterMm = 19.05;
    int alternatingIterations = 8;
    int blockIterations = 18;
    int jointIterations = 90;
    int multiStartCount = 3;

    double huberDeltaMm = 0.015;
    double finiteDifferenceRelativeStep = 1e-5;
    double convergenceStep = 1e-9;
    double convergenceCost = 1e-12;
    double convergenceGradient = 1e-9;
    double initialLambda = 1e-3;

    // 17 physical parameters + 3 unknown sphere-center coordinates.
    std::array<double, Physical17Parameters::Count + 3> parameterScales{};

    Physical17SolverOptions();
};

struct Physical17ObservabilityReport
{
    // Physical parameter block after Schur elimination of unknown sphere center.
    int targetRank = 0;
    int physicalParameterCount = static_cast<int>(Physical17Parameters::Count);
    int expectedTargetRank = 14;
    double conditionNumber = 0.0;
    double smallestSingularValue = 0.0;
    double largestSingularValue = 0.0;
    std::array<double, Physical17Parameters::Count> targetSingularValues{};

    // Full data-only state [physical17, sphere-center3].
    int fullStateRank = 0;
    int fullStateCount = static_cast<int>(Physical17Parameters::Count + 3);
    int expectedFullStateRank = 17;
    std::array<double, Physical17Parameters::Count + 3> fullStateSingularValues{};

    std::string message;
};

/** Per-pose surface-residual statistics used by V27.6.41 progressive trimmed Stage-2 screening. */
struct Physical17PoseResidualSummary
{
    JointPose5d pose;
    std::size_t profileCount = 0;
    std::size_t pointCount = 0;
    double residualMeanAbsMm = 0.0;
    double residualRmsMm = 0.0;
    double residualP95AbsMm = 0.0;
    double residualMaxAbsMm = 0.0;
    double rejectionScore = 0.0;
    bool rejected = false;
};

/**
 * Rank-aware progressive pose-trimming settings.  Screening is intentionally done
 * at the pose level: an entire stationary XYZBC pose is either retained or
 * excluded from the next solve.  Raw imported/captured observations are never
 * destroyed; the filter only chooses the training subset.
 */
struct Physical17PoseScreeningOptions
{
    double poseLinearToleranceMm = 0.03;
    double poseAngularToleranceDeg = 0.03;
    int minRetainedPoseCount = 18;

    // Per-round hard cap.  The V27.6.41 workflow also applies a separate
    // cumulative cap across all rounds.
    double maxRejectedFraction = 0.15;

    // V27.6.41 uses the *stricter* of the relative-quality gate and the
    // median/MAD gate (but never below the absolute floor).  V27.6.40 used
    // max(relative, MAD), which let a broad contaminated tail inflate the MAD
    // threshold and mask several mediocre poses.
    double madSigma = 1.75;
    double minRobustSigmaMm = 0.003;
    double relativeRmsFactor = 1.30;
    double relativeP95Factor = 1.30;
    double absoluteRmsFloorMm = 0.050;
    double absoluteP95FloorMm = 0.080;

    // Progressive trimmed least-squares mode.  If the aggregate training RMS
    // is still above target and the pose-to-pose spread is real, force a small
    // batch from the worst tail into the candidate set even when classical
    // outlier thresholds alone would mark too few groups.  The caller must
    // re-bootstrap/refit and accept the batch only if the penalized fit improves.
    bool enableProgressiveTrim = true;
    double targetTrainingRmsMm = 0.045;
    double progressiveTrimFraction = 0.12;
    int minProgressiveTrimCount = 3;
    int maxProgressiveTrimCount = 6;
    double minProgressiveSpreadMm = 0.010;
    double progressiveTailFactor = 1.02;

    // Preserve XYZ and rotary excitation while removing high-residual poses.
    double minLinearSpanFraction = 0.55;
    double minRotarySpanFraction = 0.65;
    double signMarginDeg = 0.5;
    int minPositiveNegativePoseCount = 2;
};

struct Physical17PoseScreeningReport
{
    bool valid = false;
    bool applied = false;
    std::size_t inputPoseCount = 0;
    std::size_t retainedPoseCount = 0;
    std::size_t rejectedPoseCount = 0;
    std::size_t statisticalCandidateCount = 0;
    std::size_t progressiveCandidateCount = 0;
    double aggregateRmsMm = 0.0;
    double medianRmsMm = 0.0;
    double madRmsMm = 0.0;
    double rmsThresholdMm = 0.0;
    double medianP95AbsMm = 0.0;
    double madP95AbsMm = 0.0;
    double p95ThresholdMm = 0.0;

    // Filled by the V27.6.41 workflow after a candidate subset is re-fit.
    double refitRmsBeforeMm = 0.0;
    double refitRmsAfterMm = 0.0;
    double selectionObjectiveBefore = 0.0;
    double selectionObjectiveAfter = 0.0;

    std::vector<Physical17PoseResidualSummary> poses;
    std::string message;
};

struct SingleSphere17Result
{
    bool success = false;
    // A finite-budget result may pass the existing rank/finite-value checks
    // without meeting a stationarity tolerance; do not conflate the two.
    bool optimizationConverged = false;
    double robustCost = 0.0;
    Physical17Parameters parameters;
    Vec3d sphereCenterMachineMm;

    double residualRmsMm = 0.0;
    double residualMaxAbsMm = 0.0;
    int alternatingIterations = 0;
    int jointIterations = 0;
    std::size_t profileCount = 0;
    std::size_t pointCount = 0;

    // Optional provenance for the seed used by the Physical17 solve.  V27.6.41
    // fills this from a nominal-machine 9-state hand-eye+sphere bootstrap solved
    // from the SAME Stage-2 training observations.  The three hand-eye
    // translations then become the explicit gauge anchor for the rank-14 solve.
    bool hasInitializationBootstrap = false;
    std::string initializationSource;
    Physical17Parameters initializationParameters;
    Vec3d initializationSphereCenterMachineMm;
    double initializationResidualRmsMm = 0.0;
    double initializationResidualMaxAbsMm = 0.0;
    int initializationStateRank = 0;
    double initializationConditionNumber = 0.0;
    std::size_t initializationProfileCount = 0;
    std::size_t initializationPointCount = 0;

    // V27.6.41 progressive trimmed pose-screening provenance.  These counts refer to the
    // Stage-2 training pose groups before/after adaptive hard rejection.
    int poseScreeningRounds = 0;
    std::size_t trainingPoseCountBeforeScreening = 0;
    std::size_t trainingPoseCountAfterScreening = 0;
    double poseScreeningRmsThresholdMm = 0.0;
    double poseScreeningP95ThresholdMm = 0.0;
    std::vector<Physical17PoseResidualSummary> rejectedTrainingPoses;

    Physical17ObservabilityReport observability;
    std::string gaugeDescription;
    std::string message;
};

/** Lightweight, Qt-free progress information for long Stage-2 solves. */
struct Physical17SolveProgress
{
    enum class Phase {
        Preparing,
        MultiStart,
        AlternatingMachine,
        AlternatingHandEye,
        JointRefinement,
        Observability,
        Finished
    };

    Phase phase = Phase::Preparing;
    int percent = 0;
    int startIndex = 0;
    int startCount = 0;
    int outerIteration = 0;
    int outerCount = 0;
    int innerIteration = 0;
    int innerCount = 0;
    double robustCost = 0.0;
    std::string message;
};

using Physical17ProgressCallback = std::function<void(const Physical17SolveProgress&)>;

/**
 * @brief Independent hold-out validation statistics for a fixed Physical17 model.
 *
 * Validation observations must NOT be part of the solve data set.  The report
 * evaluates the already-frozen model against raw sphere-surface points using
 * exactly the same geometric residual as calibration, but it performs no
 * parameter update and no sphere-center refit.
 */
struct Physical17ValidationReport
{
    bool valid = false;
    std::size_t profileCount = 0;
    std::size_t pointCount = 0;
    double residualMeanMm = 0.0;
    double residualMeanAbsMm = 0.0;
    double residualRmsMm = 0.0;
    double residualMaxAbsMm = 0.0;
    std::string message;
};

class SingleSphere17Model
{
public:
    /** Sensor point -> calibrated machine frame. */
    static Vec3d mapSensorPoint(const JointPose5d& joints,
                                const Vec3d& sensorPoint,
                                const Physical17Parameters& parameters);

    /** 4x4 row-major hand-eye transform ^E T_S retained for later planning. */
    static std::array<double, 16> handEyeTransform(
        const Physical17Parameters& parameters);

    /** 4x4 row-major sensor -> machine homogeneous transform. */
    static std::array<double, 16> sensorToMachineTransform(
        const JointPose5d& joints,
        const Physical17Parameters& parameters);

    /** Fixed machine-frame point expressed in the sensor frame for a pose. */
    static Vec3d machinePointInSensor(const JointPose5d& joints,
                                      const Vec3d& machinePointMm,
                                      const Physical17Parameters& parameters);

    /**
     * Exact three-translation gauge transform.  Mapping and sphere center both
     * shift by delta, leaving every point-to-sphere residual unchanged.
     */
    static void applyTranslationGauge(const Physical17Parameters& input,
                                      const Vec3d& sphereCenterMachineMm,
                                      const Vec3d& deltaMm,
                                      Physical17Parameters& shiftedParameters,
                                      Vec3d& shiftedSphereCenterMachineMm);
};


/**
 * @brief Stage-1 bootstrap result with nominal machine geometry fixed.
 *
 * Unknowns are only the 6-DoF hand-eye transform and the fixed sphere center
 * in the nominal controller machine frame.  Because the 11 machine geometry
 * parameters are held at their nominal values, this 9-state problem has no
 * translation gauge when both rotary axes are sufficiently excited.
 *
 * The per-profile +/-Y circle-center branch is NOT an operator input.  It is
 * treated as a latent initialization variable and the final refinement uses
 * the raw sphere-surface residual, which contains no Y-sign term.
 */
struct NominalHandEyeSphereResult
{
    bool success = false;
    Physical17Parameters parameters;
    Vec3d sphereCenterMachineMm;

    double residualRmsMm = 0.0;
    double residualMaxAbsMm = 0.0;
    double centerInitRmsMm = 0.0;
    int stateRank = 0;
    int expectedStateRank = 9;
    double conditionNumber = 0.0;
    std::array<double, 9> singularValues{};
    std::size_t profileCount = 0;
    std::size_t pointCount = 0;
    std::vector<int> inferredCenterYSigns;
    std::string message;
};

class NominalHandEyeSphereCalibrator
{
public:
    explicit NominalHandEyeSphereCalibrator(
        const Physical17SolverOptions& options = Physical17SolverOptions());

    void clear();
    void addObservation(const SphereProfileObservation& observation);
    const std::vector<SphereProfileObservation>& observations() const { return m_observations; }

    NominalHandEyeSphereResult solve() const;

private:
    Physical17SolverOptions m_options;
    std::vector<SphereProfileObservation> m_observations;
};

class SingleSphere17Calibrator
{
public:
    explicit SingleSphere17Calibrator(
        const Physical17SolverOptions& options = Physical17SolverOptions());

    void clear();
    void addObservation(const SphereProfileObservation& observation);
    const std::vector<SphereProfileObservation>& observations() const { return m_observations; }

    // Kept for profile segmentation / closed-loop finding only.  The sign is
    // NOT used by the joint sphere-surface calibration equation.
    static SphereCenterFit fitSphereCenterInSensor(
        const std::vector<SensorProfilePoint>& profilePoints,
        double sphereDiameterMm,
        int centerYSign = +1,
        const SphereArcRansacOptions& ransacOptions = SphereArcRansacOptions(),
        std::vector<std::size_t>* inlierIndices = nullptr);

    /**
     * Solve the nominal-machine 9-state hand-eye+sphere bootstrap from this
     * calibrator's own observations.  This is the V27.6.41 direct-import path:
     * the input CSV itself supplies the hand-eye seed instead of requiring a
     * separate historical Stage-1 result file.
     */
    NominalHandEyeSphereResult solveNominalHandEyeBootstrap() const;

    SingleSphere17Result solve(
        const Physical17Parameters& initialParameters,
        const Vec3d* initialSphereCenterMachineMm = nullptr,
        const Physical17ProgressCallback& progress = Physical17ProgressCallback(),
        const std::atomic_bool* cancelRequested = nullptr) const;

    Physical17ObservabilityReport analyzeObservability(
        const Physical17Parameters& parameters,
        const Vec3d& sphereCenterMachineMm) const;

    static Physical17ValidationReport validateModel(
        const std::vector<SphereProfileObservation>& observations,
        const Physical17Parameters& parameters,
        const Vec3d& sphereCenterMachineMm,
        double sphereDiameterMm);

    /**
     * Evaluate every distinct XYZBC pose against a frozen Physical17 model and
     * select high-residual pose groups for exclusion.  V27.6.41 combines robust
     * median/MAD gates with progressive least-trimmed-tail candidates, an
     * explicit rejection cap, and B/C excitation-preservation guards.  The
     * caller re-fits candidate subsets and decides whether the trim is useful.
     */
    static Physical17PoseScreeningReport screenPoseOutliers(
        const std::vector<SphereProfileObservation>& observations,
        const Physical17Parameters& parameters,
        const Vec3d& sphereCenterMachineMm,
        double sphereDiameterMm,
        const Physical17PoseScreeningOptions& options = Physical17PoseScreeningOptions());

    static bool readObservationCsv(std::istream& input,
                                   std::vector<SphereProfileObservation>& observations,
                                   std::string& error);
    static bool writeResultJson(std::ostream& output,
                                const SingleSphere17Result& result,
                                std::string& error);

private:
    Physical17SolverOptions m_options;
    std::vector<SphereProfileObservation> m_observations;
};

/** Read-only planning gates: never execute a machine move from these callbacks.
 * A rejected transition is reconsidered from another predecessor; a rejected
 * target is cached as infeasible for the duration of this planning call.
 */
struct Physical17PlanningChecks
{
    /**
     * Optional medium-cost shortlist gate (for example, detailed optical-FOV
     * simulation).  Dense planning evaluates this only on the bounded
     * information shortlist, not on every raw Cartesian-grid combination.
     */
    std::function<bool(const JointPose5d&, const Vec3d&)> shortlistAllowed;

    /**
     * Exact target-pose gate (for example, STL/FCL MachineSafety).  When a
     * shortlist gate is supplied this callback is evaluated lazily only for
     * D-optimal candidates that are actually about to be accepted.  For
     * backward compatibility, callers that supply poseAllowed without a
     * shortlistAllowed gate retain the historical eager filtering behaviour.
     */
    std::function<bool(const JointPose5d&, const Vec3d&)> poseAllowed;

    /** Exact swept-transition gate, evaluated lazily in acquisition order. */
    std::function<bool(const JointPose5d&, const JointPose5d&)> transitionAllowed;
};

enum class Physical17PlanningPhase
{
    Preparing,
    FilteringCandidates,
    EvaluatingShortlist,
    SelectingDOptimal,
    Finalizing
};

/** Lightweight progress packet for potentially expensive Stage-2 planning.
 * The callback may be invoked from a worker thread; GUI callers must marshal
 * updates onto their UI thread themselves.
 */
struct Physical17PlanningProgress
{
    Physical17PlanningPhase phase = Physical17PlanningPhase::Preparing;
    std::size_t current = 0;
    std::size_t total = 0;
    std::size_t feasibleCandidates = 0;
    int selectedPoses = 0;
    int requestedPoses = 0;
};

using Physical17PlanningProgressCallback =
    std::function<void(const Physical17PlanningProgress&)>;

/**
 * @brief Pose planner using the full physical transform.
 *
 * D-optimal selection is performed in the 14-D data-observable subspace while
 * the supplied hand-eye translation remains the chosen translation gauge.
 */
class SingleSphere17PosePlanner
{
public:
    explicit SingleSphere17PosePlanner(const PosePlannerConfig& config = PosePlannerConfig());

    std::vector<CalibrationPoseInstruction> generate(
        const PosePlannerSeed& seed,
        const Physical17Parameters& currentEstimate,
        std::string& error,
        const Physical17PlanningChecks& checks = Physical17PlanningChecks(),
        const Physical17PlanningProgressCallback& progress = Physical17PlanningProgressCallback(),
        const std::atomic_bool* cancelRequested = nullptr) const;

    static bool recommendXYZRecentering(
        const JointPose5d& currentJoint,
        const Vec3d& measuredCenterSensorMm,
        const Vec3d& targetCenterSensorMm,
        const Physical17Parameters& currentEstimate,
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
