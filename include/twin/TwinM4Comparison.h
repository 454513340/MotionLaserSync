#pragma once
// V28.3 M4: deterministic in-memory comparison of observed SCAN v3 indexed
// samples with M3's geometric-approximation virtual profiles.
// NO controller, laser SDK, Qt, STEP reader or plotting dependencies.
// IMPORTANT: a missing real sample has UNKNOWN cause; it is NOT automatically
// a camera-occluded sample. ROI mapping uses CAD truth ONLY for diagnostics.
#include "twin/VirtualLaser2950.h"
#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace mls::twin {
struct M4MeasuredSample {
    std::uint32_t index=0;
    double sensorX=0, sensorZ=0;
};
struct M4HoleRoi {
    std::string name;
    double x=0, y=0;  // CAD-local centre
    double radiusMm=4.2; // projected top-plane evaluation ROI
    double floorZ=5.4; // CAD Z of counterbore annular landing
    double annulusInnerMm=2.5, annulusOuterMm=3.78;
};
struct M4Context {
    Transform cadToMachine;
    double cadTopZ=10.0;
    Sensor2950Geometry optical;
    std::vector<M4HoleRoi> holes; // empty => global-only comparison
};
M4Context h1ReferenceM4Context(const TwinScene& scene, const Sensor2950Geometry& optics,bool useEstimatedPose=false);

struct M4Metrics {
    std::uint64_t truePositive=0, falsePositive=0, falseNegative=0, trueNegative=0;
    std::uint64_t pairedSamples=0;
    double sumAbsX=0, sumAbsZ=0, sumSquaredX=0, sumSquaredZ=0;
    // Two separate distributions: virtual predictions for ALL samples, and
    // predictions only where the observed SCAN has no sample. Real missing
    // cause remains UNKNOWN; these are not real occlusion labels.
    std::array<std::uint64_t,kVirtualStateCount> predictedStates{};
    std::array<std::uint64_t,kVirtualStateCount> predictedWhereRealMissing{};
    std::uint64_t realValid() const {return truePositive+falseNegative;}
    std::uint64_t simulatedValid() const {return truePositive+falsePositive;}
    std::uint64_t compared() const {return truePositive+falsePositive+falseNegative+trueNegative;}
    double precision() const;
    double recall() const;
    double f1() const;
    double iou() const;
    double meanAbsX() const;
    double meanAbsZ() const;
    double rmseZ() const;
};
struct M4FloorCoverage {
    // 3 radial bands x 36 angular bins. This is a coarse CAD annulus occupancy
    // diagnostic, not a physical surface-area metrology result.
    static constexpr int kAngularBins=36, kRadialBins=3, kBins=108;
    std::array<std::uint32_t,kBins> realCounts{};
    std::array<std::uint32_t,kBins> simulatedCounts{};
    std::uint64_t realPositiveY=0,realNegativeY=0;
    std::uint64_t simPositiveY=0,simNegativeY=0;
    std::uint64_t realFloorPoints=0,simFloorPoints=0;
    std::uint64_t realOccupied(int minHits=1) const;
    std::uint64_t simulatedOccupied(int minHits=1) const;
    double intersectionOverUnion(int minHits=1) const;
    double realPositiveRatio() const;
    double simulatedPositiveRatio() const;
};
struct M4FrameStats {
    std::uint64_t sourceOrdinal=0;
    M4Metrics global;
    M4Metrics holeRegions;
};
struct M4Report {
    M4Metrics global;
    M4Metrics holeRegions;
    M4Metrics outsideHoles;
    std::vector<M4Metrics> perHole;
    std::vector<M4FloorCoverage> floor;
    std::vector<M4FrameStats> perFrame;
    std::size_t profilesCompared=0;
    std::uint64_t rejectedIndexedPoints=0;
    std::string error;
    bool ok() const {return error.empty()&&profilesCompared>0;}
    // Weighted orientation-dependent *diagnostic* score, not a calibrated likelihood.
    double fitScore() const;
};

class TwinM4Accumulator {
public:
    explicit TwinM4Accumulator(M4Context context);
    // Match profileOrdinal externally, then pass its sparse real samples.
    // Duplicate/out-of-range/nonfinite indices are rejected, never fabricated.
    bool addFrame(const VirtualProfile& simulated,
                  const std::vector<M4MeasuredSample>& real);
    M4Report finish() &&;
private:
    M4Context ctx_;
    Transform machineToCad_;
    M4Report report_;
    bool projectToCadTop(const VirtualProfilePose& pose,int sampleIndex,Vec3* out) const;
};
} // namespace mls::twin
