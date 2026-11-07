#pragma once

#include "core/DataTypes.h"
#include "machine_config.h"
#include "../../machine5x/collision_engine.h"  // Always use the bundled STEP-aware FCL API.
#include "twin/TwinTypes.h"

#include <Eigen/Core>

#include <array>
#include <filesystem>
#include <mutex>
#include <string>
#include <vector>

namespace mls::twin { struct TriangleMesh; struct Transform; }

namespace mls::safety {

struct PoseSafetyResult
{
    bool modelReady = false;
    bool safe = false;
    double minimumClearanceMm = 0.0;
    std::vector<CollisionHit> collisions;
    std::vector<std::string> collisionLinks;
    std::string message;
};

struct TransitionSafetyResult
{
    bool modelReady = false;
    bool safe = false;
    int sampleCount = 0;
    int firstUnsafeSample = -1;
    double firstUnsafeT = -1.0;
    JointPose5D firstUnsafePose;
    PoseSafetyResult unsafePoseResult;
    std::string message;
};

/**
 * Collision/safety facade around the user's validated Machine5X FCL model.
 *
 * The collision meshes and the VTK visual meshes share the same zero-pose MCS.
 * Controller MPOS is used directly:
 *   BASE -> X -> Y -> Z -> C -> B -> END_SENSOR
 *   RC = RotZ(-C), RB = RotY(-B)
 *
 * Transition checking supports both the exact synchronous joint-space segment
 * commanded by moveJointAbs5D() and the per-axis trapezoidal/triangular time
 * path commanded by moveJointAbs5DIndependent().  This is fail-closed: an
 * unloaded model or any sampled collision makes the transition unsafe.
 */
class MachineSafetyModel
{
public:
    bool load(const std::filesystem::path& configPath, std::string* error = nullptr);
    bool isReady() const;
    const MachineConfig& config() const { return config_; }

    // Full imported STEP triangle mesh (CAD-local mm) and reviewed CAD->MCS
    // placement. Only moving links collide with this fixed workpiece; the
    // BASE/contacting fixture is excluded intentionally.
    bool setWorkpiece(const mls::twin::TriangleMesh& mesh,
                      const mls::twin::Transform& cadToMcs,
                      std::string* error = nullptr);
    void clearWorkpiece();

    void setStudySphere(double xMm, double yMm, double zMm, double radiusMm);
    void clearStudySphere();
    bool hasStudySphere() const;

    PoseSafetyResult checkPose(const JointPose5D& q, bool checkStaticClearance = true);
    // When checkStaticClearanceThroughout is true, every sampled pose must also
    // satisfy safety.minimum_static_clearance_mm, not just the endpoints.  This
    // is required while XYZ command compensation is enabled: a nominally clear
    // MPOS path must keep a clearance larger than the compensation envelope.
    TransitionSafetyResult checkTransition(const JointPose5D& from, const JointPose5D& to,
                                           bool checkStaticClearanceThroughout = false,
                                           bool firstSampleAlreadyVerified = false);
    TransitionSafetyResult checkIndependentTransition(
        const JointPose5D& from,
        const JointPose5D& to,
        const std::array<AxisParams, Constants::kAxisParamArraySize>& axisParams,
        double speedScale,
        bool checkStaticClearanceThroughout = false);

    static MachineState toMachineState(const JointPose5D& q);
    static JointPose5D interpolate(const JointPose5D& a, const JointPose5D& b, double t);

private:
    PoseSafetyResult checkPoseLocked(const JointPose5D& q, bool checkStaticClearance);

    mutable std::mutex mutex_;
    MachineConfig config_;
    CollisionEngine engine_;
    bool ready_ = false;

    bool studySphereEnabled_ = false;
    Eigen::Vector3d studySphereCenterMm_ = Eigen::Vector3d::Zero();
    double studySphereRadiusMm_ = 0.0;
};

} // namespace mls::safety
