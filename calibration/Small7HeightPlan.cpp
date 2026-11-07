#include "calibration/Small7HeightPlan.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace mls::calibration::small7 {
namespace {
constexpr double pi = 3.14159265358979323846;
bool finite(double x) { return std::isfinite(x); }
bool fail(const std::string& reason, std::string* error) {
    if (error) *error = reason;
    return false;
}
JointPose5d at(const JointPose5d& base, double x, double z) {
    auto p=base; p.x=x; p.z=z; return p;
}
}

int motionTimeoutMs(double distanceMm, double speedMmS, double accelMmS2,
                    double decelMmS2, int hardMaxMs) {
    if (!finite(distanceMm) || !finite(speedMmS) || !finite(accelMmS2) ||
        !finite(decelMmS2) || speedMmS <= 0 || accelMmS2 <= 0 || decelMmS2 <= 0 ||
        hardMaxMs < 15000) return std::max(15000, hardMaxMs);
    // Conservative trapezoid bound: cruise at <= commanded speed and full
    // acceleration/deceleration time, plus startup/IO/settle margin.
    const double seconds = std::abs(distanceMm)/speedMmS +
          speedMmS/accelMmS2 + speedMmS/decelMmS2;
    return static_cast<int>(std::clamp(std::ceil(1000.0*(seconds*1.8+12.0)),
                         15000.0, double(hardMaxMs)));
}

int referenceMaxProfileHz(int fieldIndex) {
    if (fieldIndex < 0 || fieldIndex >= 128) return 0;
    // Mirrors LaserSensor's Quick Reference for the scanCONTROL 2950.
    static constexpr int table[16][8] = {
        {144,192,192,192,284,284,284,552},
        {181,239,239,239,354,354,354,680},
        {181,239,239,239,354,354,354,680},
        {181,239,239,239,354,354,354,680},
        {241,318,318,318,469,469,469,892},
        {241,318,318,318,469,469,469,892},
        {241,318,318,318,469,469,469,892},
        {241,318,318,318,469,469,469,892},
        {362,476,476,476,694,694,694,1282},
        {362,476,476,476,694,694,694,1282},
        {362,476,476,476,694,694,694,1282},
        {362,476,476,476,694,694,694,1282},
        {552,552,1030,1030,1030,1030,1030,1030},
        {680,680,1265,1265,1265,1265,1265,1265},
        {892,892,1612,1612,1612,1612,1612,1612},
        {1282,1282,2000,2000,2000,2000,2000,2000}
    };
    return table[fieldIndex / 8][fieldIndex % 8];
}

bool calculateScanTiming(int requestedHz, double desiredLineSpacingMm,
                         ScanTiming& out, std::string* error) {
    out={};
    if (requestedHz<25 || requestedHz>2000 || !finite(desiredLineSpacingMm) ||
        desiredLineSpacingMm<0.001 || desiredLineSpacingMm>0.10)
        return fail("Invalid profile rate / adjacent profile spacing",error);
    out.period10us=static_cast<int>(std::ceil(100000.0/requestedHz));
    out.effectiveProfileHz=100000.0/out.period10us;
    out.nominalLineSpacingMm=desiredLineSpacingMm;
    out.scanSpeedMmS=desiredLineSpacingMm*out.effectiveProfileHz;
    if (!finite(out.scanSpeedMmS) || out.scanSpeedMmS<0.1 || out.scanSpeedMmS>5.0)
        return fail("Derived scan speed outside certified 0.1..5.0 mm/s; change rate or line spacing",error);
    return true;
}

bool fullCircleInsideOptics(double cx,double cz,double radius,const OpticalWindow& w){
    if(!w.verified || !finite(cx) || !finite(cz) || !finite(radius) || radius<=0 ||
       !finite(w.marginMm) || w.marginMm<0)return false;
    return cx-radius>=w.xMinMm+w.marginMm && cx+radius<=w.xMaxMm-w.marginMm &&
           cz-radius>=w.zMinMm+w.marginMm && cz+radius<=w.zMaxMm-w.marginMm;
}

bool maxCircleFitArcInsideOptics(double cx,double cz,double radius,const OpticalWindow& w,
                                 double minimumArcDeg){
    return w.verified && finite(cx) && finite(cz) && finite(radius) && radius>0 &&
        finite(minimumArcDeg) && minimumArcDeg>=40 && minimumArcDeg<=360 &&
        visibleArcSpanDeg(cx,cz,radius,w) >= minimumArcDeg;
}

namespace {
bool theoreticalCenterAtZ(double z,const JointPose5d& current,const Vec3d& sphere,
                          const Physical17Parameters& model,double& centerMachineX,
                          double& centerSensorX,double& centerSensorZ,double& gradient){
    const auto sensorAtX=[&](double x){
        return SingleSphere17Model::machinePointInSensor(at(current,x,z),sphere,model);
    };
    const auto ref=sensorAtX(current.x);
    const auto plus=sensorAtX(current.x+0.5);
    const auto minus=sensorAtX(current.x-0.5);
    gradient=plus.y-minus.y;
    if(!finite(ref.y)||!finite(gradient)||std::abs(gradient)<0.15)return false;
    centerMachineX=current.x-ref.y/gradient;
    if(!finite(centerMachineX))return false;
    const auto peak=sensorAtX(centerMachineX);
    centerSensorX=peak.x;centerSensorZ=peak.z;
    return finite(peak.x)&&finite(peak.y)&&finite(peak.z)&&std::abs(peak.y)<0.1;
}
}

std::vector<OpticalZInterval> findOpticalZIntervals(
    const Settings& s,const OpticalWindow& w,const JointPose5d& current,
    const Vec3d& sphere,const Physical17Parameters& model,double samplingStepMm){
    std::vector<OpticalZInterval> intervals;
    if(!w.verified||!finite(s.zBeginMm)||!finite(s.zEndMm)||s.zBeginMm>s.zEndMm||
       !finite(s.knownSphereDiameterMm)||s.knownSphereDiameterMm<=0||
       !finite(samplingStepMm)||samplingStepMm<=0)return intervals;
    const auto span=s.zEndMm-s.zBeginMm;
    const int n=static_cast<int>(std::ceil(span/samplingStepMm));
    if(n>20000)return intervals;
    const auto qualifies=[&](double z){
        double x=0,cx=0,cz=0,grad=0;
        return theoreticalCenterAtZ(z,current,sphere,model,x,cx,cz,grad)&&
            maxCircleFitArcInsideOptics(cx,cz,s.knownSphereDiameterMm/2,w);
    };
    // This is still a sampled feasibility estimate, not formal continuous
    // interval arithmetic. At observed transitions bisect to 0.1 micrometre.
    const auto boundary=[&](double left,double right,bool rightIsValid){
        for(int j=0;j<22 && right-left>0.0001;++j){
            const double mid=(left+right)/2;
            if(qualifies(mid)==rightIsValid)right=mid;
            else left=mid;
        }
        return rightIsValid?right:left;
    };
    bool previousGood=false;
    double previousZ=s.zBeginMm;
    for(int i=0;i<=n;++i){
        const double z=(i==n)?s.zEndMm:s.zBeginMm+span*double(i)/std::max(n,1);
        const bool good=qualifies(z);
        if(good&&!previousGood){
            const double from=(i==0)?z:boundary(previousZ,z,true);
            intervals.push_back({from,z});
        }else if(!good&&previousGood){
            intervals.back().zEndMm=boundary(previousZ,z,false);
        }else if(good){
            intervals.back().zEndMm=z;
        }
        previousGood=good;previousZ=z;
    }
    return intervals;
}

bool findScannedMaximumCircle(const std::vector<ScanPeakSample>& samples,
                              double start,double end,double expectedX,double radius,
                              double diameterTolerance,ScanPeak& out,std::string* error){
    out={};
    if(!finite(start)||!finite(end)||end<=start||!finite(expectedX)||
       !finite(radius)||radius<=1||!finite(diameterTolerance)||diameterTolerance<=0)
        return fail("Invalid scan/known sphere settings",error);
    // Fit radius^2 = a + b*(x-expectedX) + c*(x-expectedX)^2.
    // Quadratic behavior is dictated by the sphere cross-section; aggregate
    // over the scan instead of accepting one spurious large-radius profile.
    double mat[3][4]{};
    int count=0;double lo=end,hi=start;
    for(const auto& p:samples){
        if(!finite(p.machineX)||!finite(p.radiusMm)||p.machineX<start||p.machineX>end||
           p.radiusMm<radius*0.35||p.radiusMm>radius+diameterTolerance/2)continue;
        const double x=p.machineX-expectedX;
        if(std::abs(x)>radius+1)continue;
        const double a[3]={1.0,x,x*x};
        for(int r=0;r<3;++r){
            for(int c=0;c<3;++c)mat[r][c]+=a[r]*a[c];
            mat[r][3]+=a[r]*p.radiusMm*p.radiusMm;
        }
        ++count;lo=std::min(lo,p.machineX);hi=std::max(hi,p.machineX);
    }
    if(count<25||lo>expectedX-1.0||hi<expectedX+1.0)
        return fail("Too few X-labelled valid profiles or incomplete left/right sphere coverage",error);
    for(int col=0;col<3;++col){
        int pivot=col;
        for(int r=col+1;r<3;++r)if(std::abs(mat[r][col])>std::abs(mat[pivot][col]))pivot=r;
        if(std::abs(mat[pivot][col])<1e-9)return fail("Degenerate scan peak fit",error);
        for(int c=col;c<4;++c)std::swap(mat[col][c],mat[pivot][c]);
        const double factor=mat[col][col];
        for(int c=col;c<4;++c)mat[col][c]/=factor;
        for(int r=0;r<3;++r)if(r!=col){
            const double f=mat[r][col];
            for(int c=col;c<4;++c)mat[r][c]-=f*mat[col][c];
        }
    }
    const double a=mat[0][3],b=mat[1][3],c=mat[2][3];
    if(!finite(c)||c>=-0.02||c<-10.0)return fail("Sphere scan is not a concave radius-squared curve",error);
    const double dx=-b/(2*c);
    const double peakR2=a+b*dx+c*dx*dx;
    const double peakX=expectedX+dx;
    if(!finite(peakX)||!finite(peakR2)||peakR2<=0 ||
       peakX<start+0.6||peakX>end-0.6 ||std::abs(dx)>0.8)
        return fail("Measured scan maximum deviates from verified model or scan endpoints",error);
    const double fittedRadius=std::sqrt(peakR2);
    if(std::abs(2*fittedRadius-2*radius)>diameterTolerance)
        return fail("Measured scan maximum diameter disagrees with known sphere",error);
    out={peakX,fittedRadius,count};
    return true;
}

bool validateSettings(const Settings& s, const OpticalWindow& w, std::string* error) {
    if (!w.verified || w.measuringFieldIndex != s.measuringFieldIndex ||
        referenceMaxProfileHz(s.measuringFieldIndex) == 0 || w.verificationId.empty())
        return fail("Selected measuring field has no matching verified optical calibration ID",error);
    if (!finite(w.xMinMm)||!finite(w.xMaxMm)||!finite(w.zMinMm)||!finite(w.zMaxMm)||
        !finite(w.marginMm)||w.marginMm < 0.0 ||
        w.xMaxMm-w.xMinMm <= 2*w.marginMm+0.5 ||
        w.zMaxMm-w.zMinMm <= 2*w.marginMm+0.5)
        return fail("Selected measuring field ROI bounds are invalid",error);
    if (!finite(s.zBeginMm)||!finite(s.zEndMm)||!finite(s.zIntervalMm)||s.zIntervalMm<=0.0)
        return fail("Machine Z begin/end/interval are invalid",error);
    if (s.zBeginMm>s.zEndMm) return fail("Machine Z beginning must be <= ending",error);
    if ((s.zEndMm-s.zBeginMm)/s.zIntervalMm > 49.0000001)
        return fail("Too many Z heights: maximum 50",error);
    if (!finite(s.positioningSpeedMmS)||s.positioningSpeedMmS<1||
        s.positioningSpeedMmS>5||std::abs(s.positioningSpeedMmS-std::round(s.positioningSpeedMmS))>1e-9)
        return fail("Positioning speed must be 1/2/3/4/5 mm/s",error);
    ScanTiming timing;
    if(!calculateScanTiming(s.profileRateHz,s.requestedLineSpacingMm,timing,error))return false;
    if(!finite(s.scanSpeedMmS)||std::abs(s.scanSpeedMmS-timing.scanSpeedMmS)>1e-8)
        return fail("Scan speed must be derived from quantized profile rate and requested line spacing",error);
    if (s.profileRateHz<25 || s.profileRateHz>referenceMaxProfileHz(s.measuringFieldIndex) ||
        !finite(s.maximumFrameSpacingMm)||s.maximumFrameSpacingMm<=0 ||
        timing.nominalLineSpacingMm>s.maximumFrameSpacingMm+1e-10)
        return fail("Scan speed/profile rate violate maximum frame spacing",error);
    if (!finite(s.knownSphereDiameterMm)||s.knownSphereDiameterMm<2.0||
        !finite(s.scanMarginMm)||s.scanMarginMm<0.2||s.scanMarginMm>3.0||
        !finite(s.sevenPointStepMm)||s.sevenPointStepMm<=0||s.sevenPointStepMm>1.0)
        return fail("Sphere radius, scan margin or 7-point step invalid",error);
    return true;
}

double visibleArcSpanDeg(double cx,double cz,double r,const OpticalWindow& w){
    if(!finite(cx)||!finite(cz)||!finite(r)||r<=0) return 0;
    constexpr int n=720;
    int best=0,run=0;
    // Two revolutions make circular arcs crossing angle zero contiguous.
    for(int i=0;i<2*n;++i){
        const double a=2*pi*(i%n)/n;
        const double x=cx+r*std::cos(a),z=cz+r*std::sin(a);
        const bool visible=x>=w.xMinMm+w.marginMm&&x<=w.xMaxMm-w.marginMm&&
                           z>=w.zMinMm+w.marginMm&&z<=w.zMaxMm-w.marginMm;
        run=visible?std::min(n,run+1):0;
        best=std::max(best,run);
    }
    return 360.0*best/n;
}

bool planHeights(const Settings& s,const OpticalWindow& w,
                 const JointPose5d& current, const Vec3d& sphere,
                 const Physical17Parameters& model,std::vector<HeightPlan>& output,
                 std::string* error){
    output.clear();
    if(!validateSettings(s,w,error)) return false;
    const double radius=s.knownSphereDiameterMm*0.5;
    std::vector<double> heights;
    for(int i=0;i<50;++i){
        const double z=s.zBeginMm+i*s.zIntervalMm;
        if(z>s.zEndMm+1e-8)break;
        heights.push_back(std::min(z,s.zEndMm));
    }
    if(heights.empty()||heights.back()<s.zEndMm-1e-8){
        if(heights.size()>=50)return fail("More than 50 heights including end Z",error);
        heights.push_back(s.zEndMm);
    }
    for(std::size_t idx=0;idx<heights.size();++idx){
        const double z=heights[idx];
        const auto sensorAtX=[&](double x){
            return SingleSphere17Model::machinePointInSensor(at(current,x,z),sphere,model);
        };
        double gradient=0,sensorCx=0,sensorCz=0;
        double centerX=0;
        if(!theoreticalCenterAtZ(z,current,sphere,model,centerX,sensorCx,sensorCz,gradient))
            return fail("X machine axis does not span the sensor laser plane at Z height "+std::to_string(idx+1),error);
        HeightPlan h;
        h.machineZMm=z;
        h.centerXMachineMm=centerX;
        if(!finite(h.centerXMachineMm))return fail("Invalid Physical17 scan center X",error);
        const double halfspan=(radius+s.scanMarginMm)/std::abs(gradient);
        if(!finite(halfspan)||halfspan>28.0||halfspan<0.25)
            return fail("Excessive/invalid predicted X scan length at Z height "+std::to_string(idx+1),error);
        h.scanStartXMachineMm=h.centerXMachineMm-halfspan;
        h.scanEndXMachineMm=h.centerXMachineMm+halfspan;
        const auto peak=sensorAtX(h.centerXMachineMm);
        const double sectionR2=radius*radius-peak.y*peak.y;
        if(!finite(peak.x)||!finite(peak.y)||!finite(peak.z)||sectionR2<=0)
            return fail("Sphere misses the selected field laser plane at Z height "+std::to_string(idx+1),error);
        h.predictedSensorCenterX=peak.x;
        h.predictedSensorCenterZ=peak.z;
        h.visibleArcSpanDeg=visibleArcSpanDeg(peak.x,peak.z,std::sqrt(sectionR2),w);
        if(!maxCircleFitArcInsideOptics(peak.x,peak.z,radius,w))
            return fail("Insufficient theoretical maximum-circle arc in the verified optical ROI at Z height "+std::to_string(idx+1),error);
        for(int i=0;i<7;++i){
            h.sevenPointX[static_cast<std::size_t>(i)]=h.centerXMachineMm+(i-3)*s.sevenPointStepMm;
            if(h.sevenPointX[static_cast<std::size_t>(i)]<h.scanStartXMachineMm||
               h.sevenPointX[static_cast<std::size_t>(i)]>h.scanEndXMachineMm)
                return fail("7-point search exceeds planned X scan",error);
        }
        output.push_back(h);
    }
    return true;
}
}
