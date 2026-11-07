#include "twin/TwinM7Fusion.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_set>

namespace mls::twin {
namespace {
std::uint64_t mix(std::uint64_t x){
    x+=0x9e3779b97f4a7c15ULL;
    x=(x^(x>>30))*0xbf58476d1ce4e5b9ULL;
    x=(x^(x>>27))*0x94d049bb133111ebULL;
    return x^(x>>31);
}
}
M7Coverage calculateM7Coverage(const M5Report& base,const M6Report& m6,double threshold){
    M7Coverage out;out.coverageThreshold=threshold;
    if(!base.ok()||!m6.ok()||!std::isfinite(threshold)||threshold<=0||threshold>1){
        out.error="M7 requires valid M5/M6 reports and 0 < target coverage <= 1";return out;
    }
    if(m6.firstCoveredSegmentByCell.size()!=base.cells.size()){
        out.error="M6 per-cell coverage map does not match the M5 CAD tessellation";return out;
    }
    if(std::abs(base.targetAreaMm2-m6.targetAreaMm2)>std::max(.1,base.targetAreaMm2*.0001)){
        out.error="M5/M6 CAD target area mismatch";return out;
    }
    out.expectedM6NewAreaMm2=m6.newAreaMm2;
    out.expectedM6FinalAreaMm2=m6.finalObservedAreaMm2;
    out.classifications.resize(base.cells.size(),M7CellClass::Excluded);
    out.firstSimulatedSegment=m6.firstCoveredSegmentByCell;
    out.features.resize(base.features.size());
    out.addedAreaBySegmentMm2.assign(m6.segments.size(),0.);
    for(std::size_t f=0;f<base.features.size();++f){
        out.features[f].featureId=base.features[f].id;
        out.features[f].name=base.features[f].name;
    }
    const std::unordered_set<std::uint32_t> reopened(m6.reopenedSparseCellIds.begin(),m6.reopenedSparseCellIds.end());
    const auto thresholdHits=base.requestedOptions.minHitsPerCell;
    for(std::size_t i=0;i<base.cells.size();++i){
        const auto& c=base.cells[i];
        if(!c.target){
            if(i<out.firstSimulatedSegment.size()&&out.firstSimulatedSegment[i]){
                out.error="M6 marked excluded CAD geometry as newly observed";return out;
            }
            continue;
        }
        if(c.featureId>=out.features.size()||!std::isfinite(c.areaMm2)||c.areaMm2<0){
            out.error="M7 contains an invalid CAD cell or feature id";return out;
        }
        auto& feat=out.features[c.featureId];
        out.targetAreaMm2+=c.areaMm2;
        feat.targetAreaMm2+=c.areaMm2;
        const bool sparseReopened=reopened.count(static_cast<std::uint32_t>(i))!=0;
        if(sparseReopened){
            out.reopenedSparseAreaMm2+=c.areaMm2;
            feat.sparseReopenedAreaMm2+=c.areaMm2;
        }
        // A CAD cell first seen in the REAL scan remains REAL. An M5.1
        // sparse cell is reopened until it receives a genuine virtual hit.
        const bool real=(c.realHits>=thresholdHits&&!sparseReopened);
        const std::uint16_t scan=out.firstSimulatedSegment[i];
        if(scan>m6.segments.size()){
            out.error="M7 simulated segment index is out of range";return out;
        }
        if(real){
            if(scan){out.error="Simulated new cover on already-measured CAD cell";return out;}
            out.classifications[i]=M7CellClass::Measured;
            out.measuredAreaMm2+=c.areaMm2;
            feat.measuredAreaMm2+=c.areaMm2;
            ++feat.measuredCells;
        }else if(scan>0){
            out.addedAreaBySegmentMm2[scan-1]+=c.areaMm2;
            out.classifications[i]=M7CellClass::SimulatedNew;
            out.simulatedNewAreaMm2+=c.areaMm2;
            feat.simulatedNewAreaMm2+=c.areaMm2;
            ++feat.simulatedNewCells;++out.matchedNewCells;
        }else{
            out.classifications[i]=M7CellClass::Missing;
            out.remainingAreaMm2+=c.areaMm2;
            feat.stillMissingAreaMm2+=c.areaMm2;
            ++feat.missingCells;
        }
    }
    if(!(out.targetAreaMm2>0)){out.error="M7 target CAD area is empty";return out;}
    const double epsilon=std::max(.02,out.targetAreaMm2*.0001);
    // Compare against the M6 planner's original exact hit-accounting, but
    // do NOT force consistency if the M5.1 re-open mask changed.
    out.areaConsistent=(std::abs(out.simulatedNewAreaMm2-m6.newAreaMm2)<=epsilon&&
         std::abs(out.measuredAreaMm2+out.simulatedNewAreaMm2-m6.finalObservedAreaMm2)<=epsilon);
    double accumulated=out.measuredAreaMm2;
    for(const double a:out.addedAreaBySegmentMm2){
        accumulated+=a;out.cumulativeAreaBySegmentMm2.push_back(accumulated);
    }
    out.needsAdditionalView=out.finalFraction()+1e-10<threshold;
    return out;
}

std::size_t M7VoxelSampler::KeyHash::operator()(const Key& key)const noexcept{
    std::uint64_t a=mix(static_cast<std::uint64_t>(key.x));
    a=mix(a^mix(static_cast<std::uint64_t>(key.y)));
    return static_cast<std::size_t>(mix(a^mix(static_cast<std::uint64_t>(key.z))));
}
M7VoxelSampler::M7VoxelSampler(M7VoxelOptions opt):options_(opt){
    if(!std::isfinite(opt.voxelPitchMm)||opt.voxelPitchMm<.025||opt.voxelPitchMm>5.||
       opt.maxRetainedVoxels==0||opt.maxRetainedVoxels>1500000||
       opt.sampledIndexModulo==0||opt.sampledProfileModulo==0)
        error_="Invalid M7 voxel sampling options";
    if(valid())voxels_.reserve(std::min<std::size_t>(opt.maxRetainedVoxels,150000));
}
bool M7VoxelSampler::observe(Vec3 xyz){
    if(!valid())return false;
    if(!std::isfinite(xyz.x)||!std::isfinite(xyz.y)||!std::isfinite(xyz.z))return false;
    const auto coord=[this](double v)->std::int64_t{
        return static_cast<std::int64_t>(std::floor(v/options_.voxelPitchMm));
    };
    if(std::abs(xyz.x)>100000||std::abs(xyz.y)>100000||std::abs(xyz.z)>100000)return false;
    ++eligible_;
    Key key{coord(xyz.x),coord(xyz.y),coord(xyz.z)};
    const auto hash=KeyHash{}(key);
    auto it=voxels_.find(key);
    if(it!=voxels_.end()){
        auto& v=it->second;
        if(v.hits<std::numeric_limits<std::uint32_t>::max()){v.sum=v.sum+xyz;++v.hits;}
        return true;
    }
    // Spatially unbiased across segments: when full, double the global hash
    // sparsity and REMOVE previously retained points that no longer qualify.
    // O(K) only when the threshold tightens, not on each of millions of returns.
    if((hash & hashMask_)!=0)return false;
    if(voxels_.size()>=options_.maxRetainedVoxels){
        do {
            hashMask_=(hashMask_<<1)|std::size_t(1);
            for(auto old=voxels_.begin();old!=voxels_.end();){
                if((KeyHash{}(old->first)&hashMask_)!=0)old=voxels_.erase(old);
                else ++old;
            }
        }while(voxels_.size()>=options_.maxRetainedVoxels);
        if((hash & hashMask_)!=0)return false;
    }
    voxels_.emplace(key,Accum{xyz,1u,hash});
    return true;
}
std::vector<M7VoxelPoint> M7VoxelSampler::finalize(M7Source source,std::uint16_t segment)const{
    std::vector<M7VoxelPoint> result;
    result.reserve(voxels_.size());
    for(const auto& kv:voxels_){
        const auto& v=kv.second;
        if(!v.hits)continue;
        result.push_back({v.sum/double(v.hits),source,segment,v.hits});
    }
    std::sort(result.begin(),result.end(),[](const M7VoxelPoint& a,const M7VoxelPoint& b){
        if(a.xyz.x!=b.xyz.x)return a.xyz.x<b.xyz.x;
        if(a.xyz.y!=b.xyz.y)return a.xyz.y<b.xyz.y;
        return a.xyz.z<b.xyz.z;
    });
    return result;
}
} // namespace mls::twin
