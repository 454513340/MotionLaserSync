#pragma once
// V28.7 M6: OFFLINE 5-axis continuous scan planning / ray-traced evaluation.
// Plans are NOT controller programs; optical intrinsics, fixture and mechanical
// clearance require independent validation before any physical use.
#include "twin/TwinM5Coverage.h"
#include "twin/VirtualLaser2950.h"
#include "twin/TwinTrajectory.h"
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <array>
#include <string>
#include <vector>

namespace mls::twin {

struct M6AxisBounds {
    Joint5 min{{-200.,-200.,-350.,-90.,-90.}};
    Joint5 max{{200.,200.,100.,90.,90.}};
    Joint5 maxSpeed{{12.,12.,12.,15.,15.}};  // XYZ mm/s, C/B deg/s
    Joint5 maxAccel{{35.,35.,35.,45.,45.}};
    Joint5 maxJerk{{300.,300.,300.,400.,400.}};
};

struct M6Options {
    double scanRateHz=200.;
    double focusSensorZmm=95.;
    double maxCellDistanceMm=0.95;
    std::uint16_t minNewHitsPerCell=2; // consistent with M5 minimum hit count
    double candidateMinGainMm2=0.5;
    double maxDurationSec=240.;
    double maxTotalDurationSec=900.;
    double tiltAmplitudeDeg=22.;
    double yawAmplitudeDeg=65.;
    double pathRadiusScale=0.58;
    unsigned int loopsPerSegment=2;
    unsigned int maxSelectedSegments=9;
    unsigned int candidateEvaluationFrames=36;  // refined optical frames / trajectory
    unsigned int candidateCoarseFrames=8;      // cheap first-pass optical screening
    unsigned int maxRefinedCandidates=40;       // expensive FCL and refined ray tests
    unsigned int maxCandidateCurves=240;        // bounded coarse search; all feature groups remain eligible
    double maxCandidateSearchSeconds=150.;    // bounded total candidate search time
    unsigned int maxFullProfiles=90000;
    unsigned int previewLimit=85000;
    // 1 retains all simulated valid points in the PLY spool. A larger value
    // retains every Nth valid point for export; coverage always uses ALL points.
    unsigned int exportPointStride=1;
    bool alsoExportPly=false; // M6 default: lossless .scan volumes; PLY only on explicit request.
    bool excludeTopIfAlreadyCovered=true;
    bool hasInitialJoints=false;    // offline start pose from recorded M2, NOT live machine
    Joint5 initialJoints{};
    bool returnToStartAfterScan=true;
    // Optional M5.1: reopening an H1 center upper-chamfer bin even if the
    // coarse M5 >=2-hit occupancy rule declared it covered. 72*4 bins.
    std::array<std::uint8_t,72*4> prioritizeSparseChamfer{{}};
    M6AxisBounds bounds{};
};

enum class M6SegmentRole : std::uint8_t { Transfer=0, Scan=1 };
struct M6PathSample {
    double seconds=0;
    Joint5 joints{};
    Transform sensorToMachine;
    M6SegmentRole role=M6SegmentRole::Scan;
};
struct M6SegmentSummary {
    std::uint16_t featureId=0;
    std::string feature;
    unsigned int phaseId=0;
    double amplitudeScale=1.0;
    double startSeconds=0;
    double durationSec=0;
    double predictedNewAreaMm2=0;
    double measuredNewAreaMm2=0;
    std::uint64_t virtualValidPoints=0;
    std::uint64_t emittedProfiles=0;
    double vmaxUsage=0,amaxUsage=0,jmaxUsage=0;
    bool jointLimitsPassed=false;
    bool dynamicsPassed=false;
    bool collisionChecked=false;
    std::string status;
};
struct M6SegmentPreview {
    std::uint16_t featureId=0;
    std::uint64_t observedSamples=0;
    std::vector<Vec3> points;  // bounded deterministic per-segment reservoir
};
struct M6PreviewPoint {
    Vec3 positionMachine;
    bool path=false;
};
struct M6Report {
    M6Options options{};
    std::vector<M6SegmentSummary> segments;
    std::vector<M6PathSample> path;
    std::vector<Vec3> pointPreview; // flattened, retained for old Qt adapters
    std::vector<M6SegmentPreview> segmentPreviews;
    // 0 if not newly covered; 1-based FIRST accepted scan segment otherwise.
    // Original M5 cells are not modified, including re-opened sparse M5.1 cells.
    std::vector<std::uint16_t> firstCoveredSegmentByCell;
    // Original M5-"covered" CAD cells re-opened due to M5.1 angular sparsity.
    // Diagnostic targets only; never convert CAD points into laser returns.
    std::vector<std::uint32_t> reopenedSparseCellIds;
    std::vector<Vec3> pathPreview;
    double reopenedSparseChamferAreaMm2=0;
    double initialObservedAreaMm2=0;
    double targetAreaMm2=0;
    double newAreaMm2=0;
    double finalObservedAreaMm2=0;
    std::uint64_t fullProfiles=0;
    std::uint64_t fullValidPoints=0;
    std::uint64_t exportedValidPoints=0; // point count actually written to PLY spool
    std::string rawFloatXYZPath; // optional disk-spooled full set, float32 XYZ, little-endian
    std::string kinematicsSource;
    std::vector<double> featureNewAreaMm2; // by M5 feature id
    bool collisionModelUsed=false;
    bool returnedToReferencePose=false;
    std::string transferSafetyWarning;
    bool g93PhysicalTimelinePlanned=false; // accel-aware, not field motion permission
    bool canExportForHardware=false; // ALWAYS false in M6: offline only
    bool cancelled=false;
    bool candidateSearchTruncated=false;
    std::size_t candidateCurvesAttempted=0;
    std::size_t candidateCurvesFeasible=0;
    std::size_t candidateCurvesRefined=0;
    std::uint64_t candidateProfilesSimulated=0;
    std::string error;
    bool ok() const {return error.empty()&&!cancelled;}
};

// Live progress is separate from the number of accepted full-resolution exposures.
// Candidate optical profiles are not final scanned points and must never be
// counted as such. All fields can be read by Qt's progress timer without locks.
enum class M6ProgressStage : int {
    Starting=0, Geometry=1, CandidateCoarse=2, CandidateRefine=3,
    FullSimulation=4, Finished=5, Cancelled=6, Failed=7
};
struct M6ProgressState {
    std::atomic<int> stage{int(M6ProgressStage::Starting)};
    std::atomic<std::size_t> featuresTotal{0};
    std::atomic<std::size_t> candidatesTotal{0};
    std::atomic<std::size_t> candidatesDone{0};
    std::atomic<std::size_t> candidatesFeasible{0};
    std::atomic<std::size_t> refinementsTotal{0};
    std::atomic<std::size_t> refinementsDone{0};
    std::atomic<std::uint64_t> candidateProfiles{0};
    std::atomic<std::uint64_t> fullProfiles{0};
    std::atomic<std::size_t> segmentsCompleted{0};
    std::atomic<std::size_t> activeSegmentNumber{0}; // 1-based current segment, 0 before any scan
    std::atomic<std::uint64_t> archiveProfilesSaved{0}; // profiles accepted by an archival sink
    std::atomic<std::size_t> archiveVolumesSaved{0};
    std::atomic<std::uint64_t> activeSegmentProfilesTotal{0};
    std::atomic<std::uint64_t> activeSegmentProfilesDone{0};
};

// Produces a calibrated sensor->machine transform for a five-axis joint pose.
// GUI injects the real Physical17 model via SingleSphere17Model and checks
// agreement with the archived exposure pose before planning. This callback
// MUST be thread safe and must not access any live controller.
using M6ForwardKinematics=std::function<Transform(const Joint5&)>;
// Optional machine FCL checker; false rejects a candidate (fail closed).
using M6CollisionCheck=std::function<bool(const Joint5&,std::string*)>;
// Called only on accepted, laser-on exposure frames, never on candidate previews
// or unexposed transfers. A failed archive write aborts the offline planning run.
// The sink must consume the profile synchronously; it may not retain references.
using M6FullProfileSink=std::function<bool(const VirtualProfile&,std::size_t segmentNumber,std::string* error)>;

// Generates continuous C2-in-time scanning paths and greedily chooses the
// best added surface area using the *same* M3 dual-ray scanner. Forbid use of
// simulation ground truth for the initial M5 observed mask.
class TwinM6ContinuousPlanner final {
public:
    TwinM6ContinuousPlanner(const TwinScene& scene,const M5Report& initialCoverage,
        const Sensor2950Geometry& optics, M6ForwardKinematics fk,
        M6Options options={},M6CollisionCheck machineCollision={});
    M6Report run(const std::atomic<bool>* cancel=nullptr,
        std::atomic<std::size_t>* completedProfiles=nullptr,
        const std::string& spoolXYZPath={},
        M6ProgressState* liveProgress=nullptr,
        const M6FullProfileSink& archiveSink={}) const;
private:
    const TwinScene& scene_;
    const M5Report& coverage_;
    Sensor2950Geometry optics_;
    M6ForwardKinematics fk_;
    M6Options options_;
    M6CollisionCheck collision_;
};

// A geometrically anchored nominal fallback for CLI tests. Does NOT replace
// Physical17 and should never be used for machine-ready path execution.
M6ForwardKinematics makeM6NominalFromRecorded(const Joint5& referenceJoint,
                                               const Transform& recordedSensorToMachine);

} // namespace mls::twin
