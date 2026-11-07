#pragma once
// V28.4 / M5: CAD-mesh surface occupancy, NOT CAD reconstruction and NOT
// metrology-grade coverage. The real SCAN stores sparse returns with UNKNOWN
// missing-point causes; estimated CAD pose is the only input transform.
// No Qt, controller, sensor SDK, STEP parser or GUI dependencies.
#include "twin/TwinScene.h"
#include <atomic>
#include <cstdint>
#include <string>
#include <vector>

namespace mls::twin {

enum class M5FeatureKind : std::uint8_t {
    Top=0, CounterboreUpper=1, CounterboreFloor=2,
    CounterboreLower=3, CenterHole=4, ArcSlots=5,
    OuterAndOther=6, BottomExcluded=7
};

struct M5Options {
    double cellPitchMm=0.60;          // approximate triangular side length
    double maxSurfaceDistanceMm=0.65; // measurement-to-CAD association; NOT tolerance inspection
    std::uint32_t minHitsPerCell=2;
    std::uint32_t minDistinctProfilesPerCell=1; // optional stricter real exposure support
    bool excludeBottom=true;
    std::uint32_t maxCells=250000;    // bounded memory and GUI preview
};

struct M5Cell {
    Vec3 cadCenter{},cadNormal{};
    double areaMm2=0;
    std::uint32_t cadFaceId=0;
    std::uint32_t sourceTriangle=0;
    std::uint16_t featureId=0;
    std::uint32_t realHits=0,virtualHits=0;
    std::uint32_t realDistinctProfiles=0;
    bool target=true;
};

struct M5Feature {
    std::uint16_t id=0;
    std::string name;
    M5FeatureKind kind=M5FeatureKind::OuterAndOther;
    bool target=true;
    std::uint64_t totalCells=0, realCells=0,virtualCells=0,bothCells=0;
    double totalAreaMm2=0,realAreaMm2=0,virtualAreaMm2=0;
    double bothAreaMm2=0,missingAreaMm2=0;
    double realCoverage() const {return totalAreaMm2>0?realAreaMm2/totalAreaMm2:0.;}
    double virtualCoverage() const {return totalAreaMm2>0?virtualAreaMm2/totalAreaMm2:0.;}
};

struct M5Report {
    M5Options requestedOptions{};
    double effectiveCellPitchMm=0.;
    bool h1FeatureTemplate=false;
    std::vector<M5Cell> cells;
    std::vector<M5Feature> features;
    std::uint64_t realPointsSeen=0,virtualPointsSeen=0;
    std::uint64_t realAccepted=0,virtualAccepted=0;
    std::uint64_t realNoAssociation=0,virtualNoAssociation=0;
    std::uint64_t realExcluded=0,virtualExcluded=0;
    double targetAreaMm2=0,realAreaMm2=0,virtualAreaMm2=0;
    double bothAreaMm2=0,realOnlyAreaMm2=0,virtualOnlyAreaMm2=0;
    double missingBothAreaMm2=0,excludedAreaMm2=0;
    std::string error;
    bool ok() const {return error.empty()&&!cells.empty()&&targetAreaMm2>0;}
    double realCoverage() const {return targetAreaMm2>0?realAreaMm2/targetAreaMm2:0.;}
    double virtualCoverage() const {return targetAreaMm2>0?virtualAreaMm2/targetAreaMm2:0.;}
    double bothMissingRatio() const {return targetAreaMm2>0?missingBothAreaMm2/targetAreaMm2:0.;}
};

// M5CoverageBuilder owns an immutable approximation of the imported STEP
// tessellation. Each triangle is subdivided into equal-area barycentric cells
// (n*n), then measured and virtual points are independently associated via
// nearest-triangle BVH. Sensor frame coordinates must be transformed using
// Frame5D.sensorToMachine exactly ONCE by the calling layer.
class TwinM5CoverageBuilder final {
public:
    TwinM5CoverageBuilder(const TwinScene& scene, M5Options options={});
    bool ready() const {return error_.empty();}
    const std::string& error() const {return error_;}
    // Points already in machine coordinate frame; uses ESTIMATED CAD pose.
    void addRealMachine(Vec3 point);
    void addRealMachine(Vec3 point, std::uint64_t exposureKey);
    void addVirtualMachine(Vec3 point);
    // Diagnostic helper for tests / offline CAD points, not point synthesis.
    void addRealCad(Vec3 point);
    void addRealCad(Vec3 point, std::uint64_t exposureKey);
    void addVirtualCad(Vec3 point);
    M5Report finish() &&;
private:
    struct TriangleInfo {
        Vec3 a{},b{},c{};
        Bounds bounds;
        Vec3 normal{};
        std::uint32_t faceId=0;
        std::size_t cellStart=0;
        std::uint32_t subdivision=0;
        std::uint16_t featureId=0;
        bool target=true;
    };
    struct Node {
        Bounds bounds;
        int left=-1,right=-1;
        std::size_t begin=0,end=0;
    };
    struct Closest {
        std::size_t triangle=0;
        double distanceSquared=0;
        double baryB=0,baryC=0;
        bool found=false;
    };
    M5Options options_{};
    Transform machineToCad_{};
    M5Report result_{};
    std::vector<TriangleInfo> tris_;
    std::vector<std::uint32_t> order_;
    std::vector<Node> nodes_;
    std::vector<std::uint64_t> lastRealExposure_;
    std::string error_;
    int buildNode(std::size_t begin,std::size_t end);
    Closest nearest(Vec3 p) const;
    std::size_t cellFor(const TriangleInfo& tri, double baryB,double baryC) const;
    void ingest(Vec3 cad,bool simulated,std::uint64_t exposureKey=~std::uint64_t(0));
};
} // namespace mls::twin
