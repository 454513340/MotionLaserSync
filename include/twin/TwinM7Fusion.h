#pragma once
// V28.8 M7: source-aware offline fusion and coverage verification.
// REAL M2 returns are NEVER overwritten or averaged with SIMULATED M6 returns.
// All colors/voxel outputs are visual/diagnostic, not dimensional metrology.
#include "twin/TwinM5Coverage.h"
#include "twin/TwinM6Continuous.h"
#include <cstdint>
#include <cstddef>
#include <string>
#include <unordered_map>
#include <vector>

namespace mls::twin {

enum class M7Source : std::uint8_t { MeasuredM2=1, SimulatedM6=2 };
enum class M7CellClass : std::uint8_t {
    Excluded=0, Measured=1, SimulatedNew=2, Missing=3
};

struct M7FeatureStats {
    std::uint16_t featureId=0;
    std::string name;
    double targetAreaMm2=0, measuredAreaMm2=0, simulatedNewAreaMm2=0;
    double stillMissingAreaMm2=0, sparseReopenedAreaMm2=0;
    std::uint64_t measuredCells=0, simulatedNewCells=0, missingCells=0;
};
struct M7Coverage {
    std::vector<M7CellClass> classifications;
    std::vector<std::uint16_t> firstSimulatedSegment;
    std::vector<M7FeatureStats> features;
    std::vector<double> addedAreaBySegmentMm2;
    std::vector<double> cumulativeAreaBySegmentMm2;
    double targetAreaMm2=0, measuredAreaMm2=0;
    double simulatedNewAreaMm2=0, remainingAreaMm2=0, reopenedSparseAreaMm2=0;
    double expectedM6NewAreaMm2=0, expectedM6FinalAreaMm2=0;
    double coverageThreshold=0.95;
    bool needsAdditionalView=false, areaConsistent=false;
    std::uint64_t matchedNewCells=0;
    std::string error;
    bool ok() const {return error.empty()&&!classifications.empty();}
    double finalFraction() const {return targetAreaMm2>0?(measuredAreaMm2+simulatedNewAreaMm2)/targetAreaMm2:0.;}
};

// Analytical union of M5 measured CAD cells and actual M6 simulated coverage
// flags. Reopened under-sampled M5.1 cells do not count as fully measured.
// This never re-labels simulated returns as measured returns.
M7Coverage calculateM7Coverage(const M5Report& realM5,const M6Report& simulatedM6,
                               double stopAtFraction=0.95);

struct M7VoxelOptions {
    double voxelPitchMm=0.18;
    std::size_t maxRetainedVoxels=65000;
    std::uint32_t sampledProfileModulo=7;
    std::uint32_t sampledIndexModulo=8;
};
struct M7VoxelPoint {
    Vec3 xyz{}; // mean of actual accepted return positions in a voxel
    M7Source source=M7Source::MeasuredM2;
    std::uint16_t segment=0;
    std::uint32_t hits=0;
};

// Bounded spatial min-hash sample of voxels, stable across the entire scan.
// Key selection depends on position, not arrival order. Repeated points at
// the same voxel are averaged only within the SAME provenance/scan segment.
class M7VoxelSampler final {
public:
    explicit M7VoxelSampler(M7VoxelOptions options={});
    bool valid() const {return error_.empty();}
    const std::string& error()const{return error_;}
    bool observe(Vec3 machinePoint);
    std::vector<M7VoxelPoint> finalize(M7Source source,std::uint16_t segment) const;
    std::size_t retainedVoxelCount()const{return voxels_.size();}
    std::uint64_t eligiblePoints()const{return eligible_;}
private:
    struct Key {
        std::int64_t x=0,y=0,z=0;
        bool operator==(const Key& rhs)const{return x==rhs.x&&y==rhs.y&&z==rhs.z;}
    };
    struct KeyHash {std::size_t operator()(const Key& key) const noexcept;};
    struct Accum {Vec3 sum{};std::uint32_t hits=0;std::uint64_t priority=0;};
    M7VoxelOptions options_{};
    std::unordered_map<Key,Accum,KeyHash> voxels_;
    std::uint64_t eligible_=0;
    std::size_t hashMask_=0; // incremental global hash filter, no early-segment bias
    std::string error_;
};

} // namespace mls::twin
