#pragma once
// V28 M3: portable virtual profile scanner. No Qt, no controller/LLT SDK.
// Geometric model ONLY: camera/laser origins are ASSUMED until optical calibration.
#include "twin/TwinScene.h"
#include <array>
#include <atomic>
#include <cstdint>
#include <functional>
#include <limits>
#include <string>
#include <vector>

namespace mls::twin {

enum class VirtualSampleState : std::uint8_t {
    Valid=0, LaserMiss=1, CameraOccluded=2, OutsideField=3,
    BadPose=4, Cancelled=5
};
constexpr std::size_t kVirtualStateCount = 6;

struct Sensor2950Geometry {
    int sampleCount=1280;
    // Reported Sensor-Z from the provided H1 SCAN; FlipZ is ALREADY applied.
    double zNear=70.0, zCenter=95.0, zFar=120.0;
    // Real H1 reference x width decreases with INCREASING reported Sensor-Z.
    double widthAtNear=58.0, widthAtCenter=50.0, widthAtFar=42.0;
    // V2 empirically fitted raw SCAN sample index -> Sensor-X mapping.
    // These correct virtual RAY origins only; NEVER modify recorded Sensor-X.
    double sampleXScale=1.0, sampleXOffsetMm=0.0;
    // Virtual pinhole laser-origin satisfying the measured depth-width slope.
    // This is an effective projective centre, NOT a factory-measured emitter position.
    Vec3 laserOriginSensor{0,0,251.25};
    // H1 single-pose reference: +Sensor-Y predicts the observed counterbore
    // visibility side. +/-75 mm is an EFFECTIVE fitted model, not an actual
    // factory-calibrated camera position; validate other B/C poses separately.
    Vec3 cameraOriginSensor{0,+75,251.25};
    double cameraVisibilityEpsilonMm=0.05;
    double maxTraceDistanceMm=1000;
    bool validate(std::string* error=nullptr) const;
    double widthAtZ(double z) const;
    double sampleXAtZ(int index,double z) const;
};

struct VirtualProfilePose {
    std::uint64_t sourceOrdinal=0;
    double elapsedSeconds=0;
    std::array<double,5> joints{}; // UI only, not reapplied to the recorded matrix
    Transform sensorToMachine;       // exposure-time Frame5D transform, exactly once
};

// Provenance is an optional in-memory M5 diagnostic. It does not change the
// stable M3VPROF1 serialized profile schema.
enum class VirtualHitObject : std::uint8_t {None=0,Workpiece=1,Support=2};
struct VirtualProfileSample {
    float sensorX=std::numeric_limits<float>::quiet_NaN();
    float sensorZ=std::numeric_limits<float>::quiet_NaN();
    VirtualSampleState state=VirtualSampleState::LaserMiss;
    VirtualHitObject hitObject=VirtualHitObject::None;
    // V2 diagnostic-only metadata. No changes to M3VPROF1 binary serialization;
    // not a calibrated intensity or physical camera image measurement.
    std::uint32_t cadFaceId=0;
    float laserNormalCos=std::numeric_limits<float>::quiet_NaN();
    float receiverNormalCos=std::numeric_limits<float>::quiet_NaN();
    // Ideal mirror-reflection direction alignment ONLY; not measured intensity.
    float specularReceiverAlignment=std::numeric_limits<float>::quiet_NaN();
    float rayHitSensorX=std::numeric_limits<float>::quiet_NaN();
    float rayHitSensorZ=std::numeric_limits<float>::quiet_NaN();
    float cameraOcclusionClearanceMm=std::numeric_limits<float>::quiet_NaN();
    Vec3 hitPositionMachine{};
};

struct VirtualProfile {
    VirtualProfilePose pose;
    std::vector<VirtualProfileSample> samples; // always 1280 slots, invalid slots = NaN
};

struct VirtualScanResult {
    std::vector<VirtualProfile> profiles;
    std::array<std::uint64_t,kVirtualStateCount> counts{};
    std::uint64_t totalProfiles=0;
    bool cancelled=false;
    std::string error;
    std::uint64_t validCount() const {return counts[0];}
};

// Scene accelerator has immutable input; instantiate once for a whole scan.
// BVH is built over triangles in local CAD coordinates, support stays in MCS.
class TwinRayBvh final {
public:
    explicit TwinRayBvh(const TwinScene& scene,bool useEstimatedPose=false);
    RayHit firstHit(const Ray& machineRay,double maxDistanceMm) const;
    std::size_t triangleCount() const {return triangles_.size();}
private:
    struct Node {
        Bounds bounds;
        int left=-1,right=-1;
        std::uint32_t start=0,count=0;
    };
    std::vector<Triangle> triangles_;
    std::vector<std::uint32_t> indices_;
    std::vector<Node> nodes_;
    Transform cadToMachine_,machineToCad_;
    bool supportEnabled_=false;
    double supportZ_=0,supportX_=0,supportY_=0,supportW_=0,supportH_=0;
    int buildNode(std::size_t begin,std::size_t end);
};

class VirtualLaser2950 final {
public:
    explicit VirtualLaser2950(const TwinScene& scene,
                              const Sensor2950Geometry& model={},bool useEstimatedPose=false);
    bool ready(std::string* error=nullptr) const;
    VirtualProfile simulate(const VirtualProfilePose& pose) const;
    VirtualScanResult simulateAll(
        const std::vector<VirtualProfilePose>& poses,
        const std::atomic<bool>* cancel=nullptr,
        const std::function<void(std::size_t,std::size_t)>& progress={}) const;
private:
    Sensor2950Geometry geometry_;
    TwinRayBvh tracer_;
};

} // namespace mls::twin
