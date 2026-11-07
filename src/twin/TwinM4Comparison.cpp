#include "twin/TwinM4Comparison.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace mls::twin {
namespace {
constexpr double kEpsilon=1.e-10;
double ratio(double a,double b) {return b>0.0?a/b:0.0;}
void update(M4Metrics& m,bool observed,const VirtualProfileSample& predicted,
            double realX,double realZ){
    const bool virtualValid=predicted.state==VirtualSampleState::Valid &&
        std::isfinite(predicted.sensorX)&&std::isfinite(predicted.sensorZ);
    const auto state=static_cast<std::size_t>(predicted.state);
    if(state<kVirtualStateCount){
        ++m.predictedStates[state];
        if(!observed)++m.predictedWhereRealMissing[state];
    }
    if(observed&&virtualValid){
        ++m.truePositive;++m.pairedSamples;
        const double dx=realX-predicted.sensorX, dz=realZ-predicted.sensorZ;
        m.sumAbsX+=std::abs(dx);m.sumAbsZ+=std::abs(dz);
        m.sumSquaredX+=dx*dx;m.sumSquaredZ+=dz*dz;
    }else if(observed)++m.falseNegative;
    else if(virtualValid)++m.falsePositive;
    else ++m.trueNegative;
}
int floorBin(Vec3 cad,const M4HoleRoi& hole,double zTolerance){
    if(hole.annulusOuterMm<=hole.annulusInnerMm||
       !std::isfinite(cad.x)||!std::isfinite(cad.y)||!std::isfinite(cad.z)||
       std::abs(cad.z-hole.floorZ)>zTolerance)return -1;
    const double x=cad.x-hole.x,y=cad.y-hole.y;
    const double radius=std::hypot(x,y);
    if(radius<hole.annulusInnerMm||radius>hole.annulusOuterMm)return -1;
    double a=std::atan2(y,x);
    if(a<0)a+=2*kPi;
    const int sector=std::min(35,int(a/(2*kPi)*36.));
    const double u=(radius-hole.annulusInnerMm)/(hole.annulusOuterMm-hole.annulusInnerMm);
    const int ring=std::clamp(int(u*3),0,2);
    return ring*36+sector;
}
void accumulateFloor(M4FloorCoverage& f,const M4HoleRoi& hole,
                     Vec3 cad,bool measured,double zTol){
    const int bin=floorBin(cad,hole,zTol);
    if(bin<0)return;
    if(measured){
        ++f.realCounts[std::size_t(bin)];++f.realFloorPoints;
        if(cad.y>=hole.y)++f.realPositiveY;else ++f.realNegativeY;
    } else {
        ++f.simulatedCounts[std::size_t(bin)];++f.simFloorPoints;
        if(cad.y>=hole.y)++f.simPositiveY;else ++f.simNegativeY;
    }
}
} // namespace

double M4Metrics::precision() const {return ratio(double(truePositive),double(truePositive+falsePositive));}
double M4Metrics::recall() const {return ratio(double(truePositive),double(truePositive+falseNegative));}
double M4Metrics::f1() const {return ratio(2.*truePositive,2.*truePositive+falsePositive+falseNegative);}
double M4Metrics::iou() const {return ratio(double(truePositive),double(truePositive+falsePositive+falseNegative));}
double M4Metrics::meanAbsX() const {return ratio(sumAbsX,double(pairedSamples));}
double M4Metrics::meanAbsZ() const {return ratio(sumAbsZ,double(pairedSamples));}
double M4Metrics::rmseZ() const {return pairedSamples?std::sqrt(sumSquaredZ/double(pairedSamples)):0.;}
std::uint64_t M4FloorCoverage::realOccupied(int minHits) const {
    std::uint64_t n=0;for(auto v:realCounts)if(v>=std::uint32_t(std::max(1,minHits)))++n;return n;
}
std::uint64_t M4FloorCoverage::simulatedOccupied(int minHits) const {
    std::uint64_t n=0;for(auto v:simulatedCounts)if(v>=std::uint32_t(std::max(1,minHits)))++n;return n;
}
double M4FloorCoverage::intersectionOverUnion(int minHits) const {
    std::uint64_t both=0,either=0;minHits=std::max(1,minHits);
    for(std::size_t i=0;i<realCounts.size();++i){
        const bool a=realCounts[i]>=std::uint32_t(minHits),b=simulatedCounts[i]>=std::uint32_t(minHits);
        if(a&&b)++both;
        if(a||b)++either;
    }
    return ratio(double(both),double(either));
}
double M4FloorCoverage::realPositiveRatio() const {
    return ratio(double(realPositiveY),double(realFloorPoints));
}
double M4FloorCoverage::simulatedPositiveRatio() const {
    return ratio(double(simPositiveY),double(simFloorPoints));
}
double M4Report::fitScore() const {
    if(holeRegions.compared()==0||holeRegions.realValid()<10)return -1.;
    double meanIoU=0.;int used=0;
    for(const auto& f:floor){
        if(f.realFloorPoints==0)continue;
        meanIoU+=f.intersectionOverUnion();++used;
    }
    // Include depth agreement only as a soft diagnostic to avoid using exact CAD
    // points as a surrogate for real samples. No fit is 'factory calibration'.
    const double maskScore=holeRegions.f1();
    return used? .65*maskScore+.35*(meanIoU/double(used)):maskScore;
}

M4Context h1ReferenceM4Context(const TwinScene& scene,const Sensor2950Geometry& optics,bool useEstimatedPose){
    M4Context out;
    out.cadToMachine=useEstimatedPose?scene.cadToMachineEstimate():scene.cadToMachineTruth();
    out.optical=optics;
    out.cadTopZ=scene.mesh().bounds.max.z;
    // This H1-specific template comes from the supplied STEP: four identical
    // R=4, top-depth=4.6 counterbores. Never apply silently to unrelated CADs.
    const auto dim=scene.mesh().bounds.size();
    if(std::abs(dim.x-30.)>1.0||std::abs(dim.y-95.)>1.0||std::abs(dim.z-10.)>1.)return out;
    out.holes={
        {"CB1",-7.5,17.5,4.2,5.4,2.5,3.78},
        {"CB2",+7.5,17.5,4.2,5.4,2.5,3.78},
        {"CB3",-7.5,32.5,4.2,5.4,2.5,3.78},
        {"CB4",+7.5,32.5,4.2,5.4,2.5,3.78}
    };
    return out;
}

TwinM4Accumulator::TwinM4Accumulator(M4Context context):
    ctx_(std::move(context)),machineToCad_(ctx_.cadToMachine.inverseRigid()){
    report_.perHole.resize(ctx_.holes.size());
    report_.floor.resize(ctx_.holes.size());
}

bool TwinM4Accumulator::projectToCadTop(const VirtualProfilePose& pose,int index,Vec3* out) const {
    if(!out)return false;
    const auto rayOrigin=pose.sensorToMachine.point(ctx_.optical.laserOriginSensor);
    const Vec3 target{ctx_.optical.sampleXAtZ(index,ctx_.optical.zCenter),0,ctx_.optical.zCenter};
    const Vec3 rayDirection=pose.sensorToMachine.vector(normalized(target-ctx_.optical.laserOriginSensor));
    const Vec3 cadOrigin=machineToCad_.point(rayOrigin);
    const Vec3 cadDirection=machineToCad_.vector(rayDirection);
    if(std::abs(cadDirection.z)<kEpsilon)return false;
    const double t=(ctx_.cadTopZ-cadOrigin.z)/cadDirection.z;
    if(!std::isfinite(t)||t<=0||t>ctx_.optical.maxTraceDistanceMm)return false;
    *out=cadOrigin+cadDirection*t;
    return std::isfinite(out->x)&&std::isfinite(out->y)&&std::isfinite(out->z);
}

bool TwinM4Accumulator::addFrame(const VirtualProfile& simulated,
                                 const std::vector<M4MeasuredSample>& real) {
    const auto count=simulated.samples.size();
    if(count==0 || count!=std::size_t(ctx_.optical.sampleCount)){
        report_.error="M4 virtual profile sampleCount mismatch";return false;
    }
    if(!report_.error.empty())return false;
    std::vector<std::uint8_t> seen(count,0);
    std::vector<double> rx(count,0.0),rz(count,0.0);
    for(const auto& p:real){
        if(p.index>=count || !std::isfinite(p.sensorX)||!std::isfinite(p.sensorZ) ||
           seen[p.index]){
            ++report_.rejectedIndexedPoints;
            report_.error="M4 observed points include invalid, duplicate or out-of-range index";
            return false;
        }
        seen[p.index]=1;
        rx[p.index]=p.sensorX;rz[p.index]=p.sensorZ;
    }
    M4FrameStats row;
    row.sourceOrdinal=simulated.pose.sourceOrdinal;
    const Transform& sensorToMachine=simulated.pose.sensorToMachine;
    for(std::size_t i=0;i<count;++i){
        const bool observed=seen[i]!=0;
        const auto& v=simulated.samples[i];
        update(report_.global,observed,v,rx[i],rz[i]);
        update(row.global,observed,v,rx[i],rz[i]);
        int region=-1;
        Vec3 onTop;
        if(projectToCadTop(simulated.pose,int(i),&onTop)){
            for(std::size_t h=0;h<ctx_.holes.size();++h){
                const auto& hole=ctx_.holes[h];
                if(std::hypot(onTop.x-hole.x,onTop.y-hole.y)<=hole.radiusMm){
                    region=int(h);break;
                }
            }
        }
        if(region>=0){
            update(report_.holeRegions,observed,v,rx[i],rz[i]);
            update(report_.perHole[std::size_t(region)],observed,v,rx[i],rz[i]);
            update(row.holeRegions,observed,v,rx[i],rz[i]);
        }else update(report_.outsideHoles,observed,v,rx[i],rz[i]);

        if(ctx_.holes.empty())continue;
        if(observed){
            const Vec3 posCad=machineToCad_.point(sensorToMachine.point({rx[i],0,rz[i]}));
            for(std::size_t h=0;h<ctx_.holes.size();++h)
                accumulateFloor(report_.floor[h],ctx_.holes[h],posCad,true,0.35);
        }
        if(v.state==VirtualSampleState::Valid){
            const Vec3 posCad=machineToCad_.point(sensorToMachine.point({v.sensorX,0,v.sensorZ}));
            for(std::size_t h=0;h<ctx_.holes.size();++h)
                accumulateFloor(report_.floor[h],ctx_.holes[h],posCad,false,0.35);
        }
    }
    report_.perFrame.push_back(row);
    ++report_.profilesCompared;
    return true;
}
M4Report TwinM4Accumulator::finish()&& {return std::move(report_);}
} // namespace mls::twin
