#pragma once
// Off-line ROI footprint coverage gate, measured using VirtualLaser2950
// optical hits from the EXACT parsed NC exposure sequence. This does not
// measure physical sensor performance and cannot certify hidden surfaces.
#include "twin/VirtualLaser2950.h"
#include <cstddef>
#include <string>
#include <vector>

namespace mls::twin {
struct MultiPoseRoiCoverage {
    std::size_t expectedCells=0;
    std::size_t coveredCells=0;
    std::size_t expectedBoundaryCells=0;
    std::size_t coveredBoundaryCells=0;
    double fraction=0;
    double boundaryFraction=0;
    bool sufficient=false;
    std::string message;
};

class MultiPoseOpticalCoverageGrid {
public:
    MultiPoseOpticalCoverageGrid(const TwinScene& scene,const Bounds& roiCad,
                                int cellsPerAxis=6);
    void ingest(const VirtualProfile& measuredProfile);
    MultiPoseRoiCoverage summarize(double minimumFraction=0.95,
                                   double minimumBoundaryFraction=0.95) const;
private:
    Bounds roi_;
    Transform machineToCad_{};
    int cells_=6;
    std::vector<unsigned char> expected_,visited_;
};
} // namespace mls::twin
