#pragma once
// Independent multi-orientation experiment planner for M3/M4 data acquisition.
// NOT M0-M8 coverage completion and NOT a certified machine motion program.
// Portable implementation: no Qt, device SDK or implicit access to real axes.
#include "twin/TwinM6Continuous.h"
#include "runtime/FiveAxis.h"
#include <functional>
#include <string>
#include <vector>

namespace mls::twin {

enum class ExperimentSweepAxis { AutoMachineX, CadX, CadY };
struct ExperimentPose {
    std::string id;                  // e.g. P01, stable measurement/analysis label
    double cDeg=0, bDeg=0;           // physical machine rotary JOINT angles (not Euler Rx/Ry/Rz)
};
struct ExperimentOptions {
    Bounds roiCad;                   // rectangular CAD-local patch, not FOV bounds
    double surfaceZCadMm=0.;         // nominal CAD surface reference, measured in CAD coordinates
    ExperimentSweepAxis direction=ExperimentSweepAxis::AutoMachineX;
    // Extra distance where laser MUST still acquire profiles, before and
    // after the physical part in the actual X+ -> X- scanning direction.
    double leadInMm=5.;
    double leadOutMm=5.;
    double lateralCoverageMarginMm=0.5;
    Sensor2950Geometry opticalGeometry{};
    double lanePitchMm=32.;          // scanner line spans X; lateral lane pitch, NOT profile pitch
    double scanSpeedMmPerSec=10.;
    double transferSpeedMmPerSec=10.;
    double profileRateHz=200.;
    double focusSensorZmm=95.;       // 2950 model sensor-local focus location
    double retractFromSurfaceCadMm=0.; // deprecated: ignored, no Z approach/retraction is generated
    double maximumJointChordMm=1.;  // controller-polyline spacing (not profile spacing)
    unsigned int opticalPreviewProfilesPerPose=24;
    std::size_t maximumProfiles=160000; // aggregate across all B/C experiments
    std::size_t maximumPathPoints=100000;
    M6AxisBounds bounds{};          // replace by machine_config planning limits when available
    // Live-machine NC generation uses the SAME limits/timing as GCodeExecutor.
    // Other legacy optical-only digital-twin studies can still use arbitrary
    // synthetic machine bounds without implicitly claiming V30 feasibility.
    bool enforceHardwareG93=false;
    mls::runtime::Limits hardwareG93Limits{};
    double hardwareVectorAccelCap=25., hardwareVectorDecelCap=25.;
    bool hasArchivedInitialJoints=false; // historical FK context ONLY, never starting motion
    Joint5 archivedInitialJoints{};  // historical .scan state, NOT current MPOS
};
struct ExperimentViewReport {
    std::string id;
    double cDeg=0, bDeg=0;
    std::vector<M6PathSample> path;  // independent XYZCB path; exposure only on Scan edges
    std::vector<VirtualProfilePose> exposures;  // ideal profile clocks, not hardware timestamps
    std::vector<VirtualProfilePose> opticalPreviewPoses;  // bounded subset of exposures
    std::vector<Vec3> focusPathMachine;
    std::size_t scanGroups=0;
    std::size_t plannedProfiles=0;
    ExperimentSweepAxis resolvedAxis=ExperimentSweepAxis::AutoMachineX;
    bool machineXDescending=false;
    double machineXScanAxisProjection=0.;
    double scanStartCad=0.,scanEndCad=0.;
    double projectedLaserScanMinMm=0.,projectedLaserScanMaxMm=0.;
    double projectedLaserLateralMinMm=0.,projectedLaserLateralMaxMm=0.;
    double beforePartMarginMm=0.,afterPartMarginMm=0.;
    double predictedLateralCoverageWidthMm=0.;
    bool planarCoverageEnvelopePassed=false;
    double nominalScanLengthMm=0.;
    double nominalDurationSec=0;
    double predictedProfilePitchMm=0;
    bool collisionCheckerUsed=false;
    std::string warning;
};
struct ExperimentPlan {
    std::vector<ExperimentViewReport> views;
    std::string error;
    bool registrationVerified=false;     // intentionally FALSE: must be certified by a separate registration result
    bool opticsVerifiedAcrossAngles=false; // M3 physical camera/laser model remains provisional
    bool hardwareAuthorized=false;
    bool ok() const {return error.empty() && !views.empty();}
};

// Forward and optional collision checker reuse the same M6/Physical17 contracts.
// The inverse linear xyz solver here evaluates the provided CALIBRATED FK at
// fixed C/B; there is no hardcoded assumption that q.X == machine X.
ExperimentPlan planMultiPoseExperiment(const TwinScene& scene,
    const std::vector<ExperimentPose>& poses,
    const ExperimentOptions& options,
    const M6ForwardKinematics& forward,
    const M6CollisionCheck& collision={});

// A single continuous OFFLINE NC session. Unlike a playlist of independent
// P01..P37 files, all changes between poses have a finite-duration, laser-OFF,
// FCL-tested XYZCB path. No fabricated first G00 approach is included.
struct ConnectedExperimentSession {
    M6Report merged;
    std::vector<double> poseStartSeconds;
    std::vector<double> poseEndSeconds;
    std::string error;
    bool ok() const {return error.empty() && merged.ok() && !poseStartSeconds.empty();}
};
// Fail closed if no collision checker is supplied for multi-pose connections.
// No unvalidated Z-lift or G00 shortcut is introduced.
ConnectedExperimentSession planConnectedExperimentSession(
    const ExperimentPlan& plan,const ExperimentOptions& options,
    const M6ForwardKinematics& forward,const M6CollisionCheck& collision);

// Experiment protocol: B and C = {-10,-6,-2,2,6,10} degrees.
// B-major/C-minor gives P01..P36, plus the separately requested
// (B=0,C=0) reference P37. These 37 poses are independent, offline only.
std::vector<ExperimentPose> standardBCExperimentMatrix();

// Build an offline-only NC preview for ONE orientation by reusing the existing
// V29 M6 G05/G93 exporter. The caller must supply H1(1) or H3(3) explicitly.
M6Report multiPoseAsOfflineM6(const ExperimentViewReport& view,
                              const ExperimentOptions& options);
}
