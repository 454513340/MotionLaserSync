#pragma once
// V28.5 / M5.1 - independent, offline diagnostic of the H1 center through-hole.
// This is NOT metrology and NOT an estimate of camera occlusion cause.
// CAD nominal geometry provides a target; only actual/virtual returns count as hits.
#include "twin/TwinScene.h"
#include <array>
#include <atomic>
#include <cstdint>
#include <string>
#include <vector>

namespace mls::twin {
enum class M51Surface : std::uint8_t { TopChamfer=0, Cylinder=1, BottomChamfer=2 };
constexpr int M51AngularBins=72;
constexpr int M51ChamferLayers=4;
constexpr int M51CylinderLayers=20;
constexpr int M51ThresholdCount=3;

struct M51Options {
    // Geometric association sensitivity sweep. These are NOT dimensional tolerances.
    std::array<double,M51ThresholdCount> thresholdsMm{{0.15,0.30,0.65}};
    int primaryThreshold=1; // 0.30mm working diagnostic; strict 0.15 remains sensitivity check
    double sparseRatio=0.30;  // relative to upper-quartile angular reference
    std::uint32_t sparseMinReferenceHits=8;
    // Clamp out-of-surface near-boundary returns; never extrapolate deep into planes.
    double regionMarginMm=0.06;
};

struct M51Bin {
    M51Surface surface=M51Surface::TopChamfer;
    std::uint16_t angle=0,layer=0;
    double areaMm2=0.0;
    std::array<std::uint32_t,M51ThresholdCount> realHits{{0,0,0}};
    std::array<std::uint32_t,M51ThresholdCount> virtualHits{{0,0,0}};
    double realRelativeDensity=0.0;
    double virtualRelativeDensity=0.0;
    bool realSparse=false,virtualSparse=false;
};

struct M51FaceGroup {
    std::string name;
    M51Surface surface=M51Surface::TopChamfer;
    double nominalAreaMm2=0.;
    double meshAreaMm2=0.;
    std::vector<std::uint32_t> cadFaceIds;
};

struct M51Report {
    M51Options options{};
    std::vector<M51Bin> bins;
    std::vector<M51FaceGroup> surfaces;
    std::array<std::uint64_t,M51ThresholdCount> realAssociated{{0,0,0}};
    std::array<std::uint64_t,M51ThresholdCount> virtualAssociated{{0,0,0}};
    std::uint64_t realSeen=0,virtualSeen=0;
    double xCenterCadMm=0.,yCenterCadMm=-38.;
    double nominalOuterRadiusMm=0.,nominalInnerRadiusMm=0.,nominalChamferDepthMm=0.;
    double cadTopZMm=10., cadBottomZMm=0.;
    std::uint32_t realLowDensityAngles=0;
    // Exploratory conical-profile equation residual r + depth - nominalOuterRadius.
    // This is NOT a direct dimension measurement (pose/optics effects mixed).
    double realConeEquationMedianMm=0.;
    double realConeEquationMadMm=0.;
    double virtualConeEquationMedianMm=0.;
    std::uint64_t realConeResidualSamples=0,virtualConeResidualSamples=0;
    bool hasVirtual=false;
    std::string limitation;
    std::string error;
    bool ok() const { return error.empty() && !bins.empty(); }
};

// H1-specific geometry validated against imported STEP triangulation. All
// points are ingested in CAD coordinates (NOT rounded VTK preview points).
// The M2/M3 transformations must be applied exactly once in the Qt adapter.
class TwinM51ChamferAnalyzer final {
public:
    explicit TwinM51ChamferAnalyzer(const TwinScene& scene,M51Options opts={});
    bool ready() const {return report_.error.empty();}
    const std::string& error() const {return report_.error;}
    void addRealCad(Vec3 p);
    void addVirtualCad(Vec3 p);
    M51Report finish() &&;
private:
    M51Report report_{};
    double zTop_=10.,zBottom_=0.;
    double outerRadius_=3.25,innerRadius_=2.75,chamferDepth_=.5;
    std::vector<double> coneResidualReal_, coneResidualVirtual_;
    void add(Vec3 p,bool virt);
};
} // namespace mls::twin
