#include "twin/MultiPoseOpticalCoverage.h"
#include <algorithm>
#include <cmath>

namespace mls::twin {
namespace {
// Only count CAD triangles with nondegenerate XY projection. Thus holes in
// the nominal ROI need not be treated as missing coverage.
bool coversXY(double x,double y,const Triangle& t){
    const double ax=t.a.x,ay=t.a.y;
    const double bx=t.b.x-ax,by=t.b.y-ay;
    const double cx=t.c.x-ax,cy=t.c.y-ay;
    const double det=bx*cy-by*cx;
    if(std::abs(det)<1e-9)return false;
    const double px=x-ax,py=y-ay;
    const double u=(px*cy-py*cx)/det;
    const double v=(bx*py-by*px)/det;
    return u>=-1e-6&&v>=-1e-6&&u+v<=1.+1e-6;
}
}
MultiPoseOpticalCoverageGrid::MultiPoseOpticalCoverageGrid(
    const TwinScene& scene,const Bounds& roiCad,int cellsPerAxis)
    :roi_(roiCad),machineToCad_(scene.cadToMachineEstimate().inverseRigid()),
     cells_(std::clamp(cellsPerAxis,2,32)),
     expected_(static_cast<std::size_t>(cells_*cells_),0),
     visited_(static_cast<std::size_t>(cells_*cells_),0){
    if(!roi_.valid()||roi_.size().x<=0||roi_.size().y<=0)return;
    for(int row=0;row<cells_;++row){
        for(int col=0;col<cells_;++col){
            const double x=roi_.min.x+(double(col)+0.5)*roi_.size().x/cells_;
            const double y=roi_.min.y+(double(row)+0.5)*roi_.size().y/cells_;
            for(const Triangle& tri:scene.mesh().triangles){
                if(coversXY(x,y,tri)){
                    expected_[static_cast<std::size_t>(row*cells_+col)]=1;
                    break;
                }
            }
        }
    }
}
void MultiPoseOpticalCoverageGrid::ingest(const VirtualProfile& profile){
    if(roi_.size().x<=0||roi_.size().y<=0)return;
    for(const auto& sample:profile.samples){
        if(sample.state!=VirtualSampleState::Valid ||
           sample.hitObject!=VirtualHitObject::Workpiece)continue;
        const Vec3 cad=machineToCad_.point(sample.hitPositionMachine);
        const double u=(cad.x-roi_.min.x)/roi_.size().x;
        const double v=(cad.y-roi_.min.y)/roi_.size().y;
        if(!(u>=0&&u<=1&&v>=0&&v<=1))continue;
        const int col=std::clamp(static_cast<int>(std::floor(u*cells_)),0,cells_-1);
        const int row=std::clamp(static_cast<int>(std::floor(v*cells_)),0,cells_-1);
        const std::size_t ix=static_cast<std::size_t>(row*cells_+col);
        if(expected_[ix])visited_[ix]=1;
    }
}
MultiPoseRoiCoverage MultiPoseOpticalCoverageGrid::summarize(
    double minimumFraction,double minimumBoundaryFraction) const{
    MultiPoseRoiCoverage r;
    for(int row=0;row<cells_;++row){
        for(int col=0;col<cells_;++col){
            const std::size_t ix=static_cast<std::size_t>(row*cells_+col);
            if(!expected_[ix])continue;
            ++r.expectedCells;
            if(visited_[ix])++r.coveredCells;
            if(row==0||row==cells_-1||col==0||col==cells_-1){
                ++r.expectedBoundaryCells;
                if(visited_[ix])++r.coveredBoundaryCells;
            }
        }
    }
    if(r.expectedCells)
        r.fraction=double(r.coveredCells)/double(r.expectedCells);
    if(r.expectedBoundaryCells)
        r.boundaryFraction=double(r.coveredBoundaryCells)/double(r.expectedBoundaryCells);
    r.sufficient=(r.expectedCells>0&&r.expectedBoundaryCells>0&&
                  r.fraction>=minimumFraction&&
                  r.boundaryFraction>=minimumBoundaryFraction);
    if(!r.expectedCells)r.message="ROI has no CAD XY footprint cells to test";
    else if(!r.sufficient)r.message="Visible optical samples miss ROI cells/boundary; a single line crossing is NOT full inspection";
    else r.message="Coarse ROI optical occupancy passed; occluded geometry and actual hardware still unverified";
    return r;
}
} // namespace mls::twin
