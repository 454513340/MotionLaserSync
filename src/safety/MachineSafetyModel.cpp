#include "safety/MachineSafetyModel.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <sstream>
#include <type_traits>

// Fail early if a stale Machine5X CollisionEngine header is accidentally used.
// In particular, setWorkpiece/clearWorkpiece/minimumDistanceBelow MUST remain
// declared in the bundled machine5x/collision_engine.h.
static_assert(CollisionEngine::kStepWorkpieceApiVersion >= 2,
              "Incorrect Machine5X collision_engine.h: use the current source tree");
static_assert(std::is_same_v<
    decltype(&CollisionEngine::setWorkpiece),
    bool (CollisionEngine::*)(const mls::twin::TriangleMesh&,
                               const mls::twin::Transform&, std::string*)>,
    "CollisionEngine::setWorkpiece declaration/definition mismatch");
static_assert(std::is_same_v<
    decltype(&CollisionEngine::clearWorkpiece),
    void (CollisionEngine::*)()>,
    "CollisionEngine::clearWorkpiece declaration/definition mismatch");
static_assert(std::is_same_v<
    decltype(&CollisionEngine::minimumDistanceBelow),
    DistanceInfo (CollisionEngine::*)(double) const>,
    "CollisionEngine::minimumDistanceBelow declaration/definition mismatch");

namespace mls::safety {
namespace {

bool finiteJoint(const JointPose5D& q)
{
    return std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z)
        && std::isfinite(q.c) && std::isfinite(q.b);
}

void appendUniqueLink(std::vector<std::string>& links, const std::string& name)
{
    if (std::find(links.begin(), links.end(), name) == links.end()) links.push_back(name);
}

std::string hitSummary(const std::vector<CollisionHit>& hits)
{
    std::ostringstream os;
    for (std::size_t i = 0; i < hits.size(); ++i) {
        if (i) os << "; ";
        os << hits[i].linkA << " <-> " << hits[i].linkB;
        if (hits[i].penetration_mm > 0.0) os << " penetration=" << hits[i].penetration_mm << " mm";
    }
    return os.str();
}

double jointCoordinate(const JointPose5D& q, int axis)
{
    switch (axis) {
    case AxisX: return q.x;
    case AxisY: return q.y;
    case AxisZ: return q.z;
    case AxisC: return q.c;
    case AxisB: return q.b;
    default: return 0.0;
    }
}

void setJointCoordinate(JointPose5D& q, int axis, double value)
{
    switch (axis) {
    case AxisX: q.x=value; break;
    case AxisY: q.y=value; break;
    case AxisZ: q.z=value; break;
    case AxisC: q.c=value; break;
    case AxisB: q.b=value; break;
    default: break;
    }
}

struct IndependentAxisProfile
{
    double start = 0.0;
    double target = 0.0;
    double sign = 1.0;
    double distance = 0.0;
    double accel = 0.0;
    double decel = 0.0;
    double peakSpeed = 0.0;
    double tAccel = 0.0;
    double tCruise = 0.0;
    double tDecel = 0.0;
    double dAccel = 0.0;
    double dCruise = 0.0;
    double totalTime = 0.0;
};

bool buildIndependentAxisProfile(double start, double target,
                                 const AxisParams& params, double speedScale,
                                 IndependentAxisProfile& out)
{
    out = IndependentAxisProfile{};
    out.start = start;
    out.target = target;
    const double delta = target - start;
    out.sign = delta >= 0.0 ? 1.0 : -1.0;
    out.distance = std::abs(delta);
    if (out.distance <= 1e-12) return true;

    const double vmax = static_cast<double>(params.speed) * speedScale;
    const double accel = static_cast<double>(params.accel) * speedScale;
    const double decel = static_cast<double>(params.decel) * speedScale;
    if (!std::isfinite(vmax) || vmax <= 0.0 ||
        !std::isfinite(accel) || accel <= 0.0 ||
        !std::isfinite(decel) || decel <= 0.0) return false;

    out.accel = accel;
    out.decel = decel;
    const double dAccelAtVmax = vmax * vmax / (2.0 * accel);
    const double dDecelAtVmax = vmax * vmax / (2.0 * decel);
    if (dAccelAtVmax + dDecelAtVmax <= out.distance) {
        out.peakSpeed = vmax;
        out.tAccel = vmax / accel;
        out.tDecel = vmax / decel;
        out.dAccel = dAccelAtVmax;
        out.dCruise = out.distance - dAccelAtVmax - dDecelAtVmax;
        out.tCruise = out.dCruise / vmax;
    } else {
        const double peak = std::sqrt((2.0 * out.distance * accel * decel) / (accel + decel));
        out.peakSpeed = peak;
        out.tAccel = peak / accel;
        out.tDecel = peak / decel;
        out.dAccel = peak * peak / (2.0 * accel);
        out.dCruise = 0.0;
        out.tCruise = 0.0;
    }
    out.totalTime = out.tAccel + out.tCruise + out.tDecel;
    return std::isfinite(out.totalTime);
}

double independentAxisPosition(const IndependentAxisProfile& p, double t)
{
    if (p.distance <= 1e-12 || t <= 0.0) return p.start;
    if (t >= p.totalTime) return p.target;

    double traveled = 0.0;
    if (t <= p.tAccel) {
        traveled = 0.5 * p.accel * t * t;
    } else if (t <= p.tAccel + p.tCruise) {
        const double tc = t - p.tAccel;
        traveled = p.dAccel + p.peakSpeed * tc;
    } else {
        const double td = t - p.tAccel - p.tCruise;
        traveled = p.dAccel + p.dCruise + p.peakSpeed * td - 0.5 * p.decel * td * td;
    }
    traveled = std::clamp(traveled, 0.0, p.distance);
    return p.start + p.sign * traveled;
}

} // namespace

bool MachineSafetyModel::load(const std::filesystem::path& configPath, std::string* error)
{
    std::lock_guard<std::mutex> lock(mutex_);
    ready_ = false;
    try {
        config_ = loadMachineConfig(configPath);
        engine_.load(config_);
        engine_.update(MachineState{});
        ready_ = true;
        return true;
    } catch (const std::exception& e) {
        if (error) *error = e.what();
        return false;
    }
}

bool MachineSafetyModel::isReady() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return ready_;
}

bool MachineSafetyModel::setWorkpiece(const mls::twin::TriangleMesh& mesh,
    const mls::twin::Transform& cadToMcs, std::string* error)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if(!ready_){if(error)*error="Machine FCL model not ready";return false;}
    try{return engine_.setWorkpiece(mesh,cadToMcs,error);}
    catch(const std::exception& e){engine_.clearWorkpiece();if(error)*error=e.what();return false;}
}

void MachineSafetyModel::clearWorkpiece()
{
    std::lock_guard<std::mutex> lock(mutex_);
    engine_.clearWorkpiece();
}

void MachineSafetyModel::setStudySphere(double xMm, double yMm, double zMm, double radiusMm)
{
    std::lock_guard<std::mutex> lock(mutex_);
    studySphereCenterMm_ = Eigen::Vector3d(xMm, yMm, zMm);
    studySphereRadiusMm_ = std::max(0.0, radiusMm);
    studySphereEnabled_ = studySphereRadiusMm_ > 0.0;
}

void MachineSafetyModel::clearStudySphere()
{
    std::lock_guard<std::mutex> lock(mutex_);
    studySphereEnabled_ = false;
    studySphereRadiusMm_ = 0.0;
}

bool MachineSafetyModel::hasStudySphere() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return studySphereEnabled_;
}

MachineState MachineSafetyModel::toMachineState(const JointPose5D& q)
{
    MachineState s;
    s.x_mm = q.x;
    s.y_mm = q.y;
    s.z_mm = q.z;
    s.c_deg = q.c;
    s.b_deg = q.b;
    return s;
}

JointPose5D MachineSafetyModel::interpolate(const JointPose5D& a, const JointPose5D& b, double t)
{
    t = std::clamp(t, 0.0, 1.0);
    JointPose5D q;
    q.x = a.x + (b.x-a.x)*t;
    q.y = a.y + (b.y-a.y)*t;
    q.z = a.z + (b.z-a.z)*t;
    q.c = a.c + (b.c-a.c)*t;
    q.b = a.b + (b.b-a.b)*t;
    return q;
}

PoseSafetyResult MachineSafetyModel::checkPose(const JointPose5D& q, bool checkStaticClearance)
{
    std::lock_guard<std::mutex> lock(mutex_);
    return checkPoseLocked(q, checkStaticClearance);
}

PoseSafetyResult MachineSafetyModel::checkPoseLocked(const JointPose5D& q, bool checkStaticClearance)
{
    PoseSafetyResult out;
    out.modelReady = ready_;
    if (!ready_) {
        out.message = "Machine collision model is not loaded.";
        return out;
    }
    if (!finiteJoint(q)) {
        out.message = "Joint pose contains NaN/Inf.";
        return out;
    }

    engine_.update(toMachineState(q));
    out.collisions = engine_.checkCollisions();

    if (studySphereEnabled_) {
        const double inflatedRadius = studySphereRadiusMm_ + std::max(0.0, config_.safety.studySphereClearanceMm);
        auto sphereHits = engine_.checkSphereObstacle(
            studySphereCenterMm_, inflatedRadius,
            config_.safety.studySphereCollisionLinks);
        out.collisions.insert(out.collisions.end(), sphereHits.begin(), sphereHits.end());
    }

    if (!out.collisions.empty()) {
        for (const auto& h : out.collisions) {
            appendUniqueLink(out.collisionLinks, h.linkA);
            appendUniqueLink(out.collisionLinks, h.linkB);
        }
        out.message = "Collision: " + hitSummary(out.collisions);
        return out;
    }

    out.minimumClearanceMm = std::numeric_limits<double>::infinity();
    if (checkStaticClearance && config_.safety.minimumStaticClearanceMm > 0.0) {
        const auto d = engine_.minimumDistanceBelow(config_.safety.minimumStaticClearanceMm);
        out.minimumClearanceMm = d.distance_mm;
        if (std::isfinite(d.distance_mm)
            && d.distance_mm + 1e-9 < config_.safety.minimumStaticClearanceMm) {
            out.collisionLinks = {d.linkA, d.linkB};
            std::ostringstream os;
            os << "Static clearance " << d.distance_mm << " mm between "
               << d.linkA << " and " << d.linkB << " is below required "
               << config_.safety.minimumStaticClearanceMm << " mm.";
            out.message = os.str();
            return out;
        }
    }

    out.safe = true;
    out.message = "OK";
    return out;
}

TransitionSafetyResult MachineSafetyModel::checkTransition(const JointPose5D& from, const JointPose5D& to,
    bool checkStaticClearanceThroughout, bool firstSampleAlreadyVerified)
{
    std::lock_guard<std::mutex> lock(mutex_);
    TransitionSafetyResult out;
    out.modelReady = ready_;
    if (!ready_) {
        out.message = "Machine collision model is not loaded.";
        return out;
    }
    if (!finiteJoint(from) || !finiteJoint(to)) {
        out.message = "Transition endpoint contains NaN/Inf.";
        return out;
    }

    const double linearStep = std::max(0.05, std::abs(config_.safety.transitionLinearStepMm));
    const double rotaryStep = std::max(0.05, std::abs(config_.safety.transitionRotaryStepDeg));
    int segments = 1;
    segments = std::max(segments, static_cast<int>(std::ceil(std::abs(to.x-from.x)/linearStep)));
    segments = std::max(segments, static_cast<int>(std::ceil(std::abs(to.y-from.y)/linearStep)));
    segments = std::max(segments, static_cast<int>(std::ceil(std::abs(to.z-from.z)/linearStep)));
    segments = std::max(segments, static_cast<int>(std::ceil(std::abs(to.c-from.c)/rotaryStep)));
    segments = std::max(segments, static_cast<int>(std::ceil(std::abs(to.b-from.b)/rotaryStep)));
    segments = std::min(segments, std::max(1, config_.safety.transitionMaxSamples));
    out.sampleCount = segments + 1;

    // Adjacent NC segments share the same vertex. The caller may reuse an
    // immediately preceding PASS for this *exact same* end/start point. All
    // interior samples and the final endpoint remain checked with unchanged
    // spacing, collision meshes and clearance requirements.
    for (int i = firstSampleAlreadyVerified ? 1 : 0; i <= segments; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(segments);
        const JointPose5D q = interpolate(from, to, t);
        // Endpoint clearance is checked; interior samples use fast exact mesh
        // collision testing so long autonomous paths remain practical.
        const bool clearance = checkStaticClearanceThroughout || (i == 0 || i == segments);
        const PoseSafetyResult pose = checkPoseLocked(q, clearance);
        if (!pose.safe) {
            out.firstUnsafeSample = i;
            out.firstUnsafeT = t;
            out.firstUnsafePose = q;
            out.unsafePoseResult = pose;
            std::ostringstream os;
            os << "Unsafe transition at t=" << t << " sample " << i << "/" << segments
               << ": " << pose.message;
            out.message = os.str();
            return out;
        }
    }

    out.safe = true;
    out.message = "OK";
    return out;
}

TransitionSafetyResult MachineSafetyModel::checkIndependentTransition(
    const JointPose5D& from,
    const JointPose5D& to,
    const std::array<AxisParams, Constants::kAxisParamArraySize>& axisParams,
    double speedScale,
    bool checkStaticClearanceThroughout)
{
    std::lock_guard<std::mutex> lock(mutex_);
    TransitionSafetyResult out;
    out.modelReady = ready_;
    if (!ready_) {
        out.message = "Machine collision model is not loaded.";
        return out;
    }
    if (!finiteJoint(from) || !finiteJoint(to) ||
        !std::isfinite(speedScale) || speedScale <= 0.0 || speedScale > 1.0) {
        out.message = "Independent transition endpoint or speed scale is invalid.";
        return out;
    }

    std::array<IndependentAxisProfile, Constants::kAxisParamArraySize> profiles{};
    double totalTime = 0.0;
    for (int axis=AxisX; axis<=AxisB; ++axis) {
        if (!buildIndependentAxisProfile(jointCoordinate(from, axis), jointCoordinate(to, axis),
                                         axisParams[axis], speedScale, profiles[axis])) {
            out.message = "Independent transition has invalid per-axis speed/acceleration parameters.";
            return out;
        }
        totalTime = std::max(totalTime, profiles[axis].totalTime);
    }

    if (totalTime <= 1e-12) {
        out.sampleCount = 1;
        const PoseSafetyResult pose = checkPoseLocked(to, true);
        if (!pose.safe) {
            out.firstUnsafeSample = 0;
            out.firstUnsafeT = 1.0;
            out.firstUnsafePose = to;
            out.unsafePoseResult = pose;
            out.message = "Independent transition endpoint is unsafe: " + pose.message;
            return out;
        }
        out.safe = true;
        out.message = "OK";
        return out;
    }

    const double linearStep = std::max(0.05, std::abs(config_.safety.transitionLinearStepMm));
    const double rotaryStep = std::max(0.05, std::abs(config_.safety.transitionRotaryStepDeg));
    int segments = 1;
    for (int axis=AxisX; axis<=AxisB; ++axis) {
        const double step = axis <= AxisZ ? linearStep : rotaryStep;
        const double maxTravelPerSecond = profiles[axis].peakSpeed;
        if (maxTravelPerSecond > 0.0) {
            segments = std::max(segments,
                static_cast<int>(std::ceil(totalTime * maxTravelPerSecond / step)));
        }
    }
    segments = std::min(segments, std::max(1, config_.safety.transitionMaxSamples));
    out.sampleCount = segments + 1;

    for (int i=0; i<=segments; ++i) {
        const double u = static_cast<double>(i) / static_cast<double>(segments);
        const double t = totalTime * u;
        JointPose5D q = from;
        for (int axis=AxisX; axis<=AxisB; ++axis)
            setJointCoordinate(q, axis, independentAxisPosition(profiles[axis], t));

        const bool clearance = checkStaticClearanceThroughout || (i == 0 || i == segments);
        const PoseSafetyResult pose = checkPoseLocked(q, clearance);
        if (!pose.safe) {
            out.firstUnsafeSample = i;
            out.firstUnsafeT = u;
            out.firstUnsafePose = q;
            out.unsafePoseResult = pose;
            std::ostringstream os;
            os << "Unsafe independent transition at normalized t=" << u
               << " sample " << i << "/" << segments << ": " << pose.message;
            out.message = os.str();
            return out;
        }
    }

    out.safe = true;
    out.message = "OK";
    return out;
}

} // namespace mls::safety
