// Portable regression for Small7 Experiment3 optical limits, Z sweep and motion timing.
#include "calibration/Small7HeightPlan.h"
#include <cstdlib>
#define CHECK(c) do { if(!(c)){std::cerr<<"FAILED at line "<<__LINE__<<": "<<#c<<"\n";std::exit(1);} } while(0)
#include <cmath>
#include <iostream>
#include <string>
#include <vector>
using namespace mls::calibration;
using namespace mls::calibration::small7;
int main(){
    OpticalWindow roi;
    Settings s;
    std::string error;
    CHECK(!validateSettings(s,roi,&error));
    roi.verified=true;roi.verificationId="test-only-synthetic";
    roi.xMinMm=-20;roi.xMaxMm=20;roi.zMinMm=70;roi.zMaxMm=120;
    CHECK(validateSettings(s,roi,&error));
    CHECK(referenceMaxProfileHz(0)==144);
    CHECK(referenceMaxProfileHz(2)==192);
    CHECK(referenceMaxProfileHz(7)==552);
    CHECK(referenceMaxProfileHz(122)==2000);
    CHECK(referenceMaxProfileHz(127)==2000);
    CHECK(referenceMaxProfileHz(128)==0);
    roi.measuringFieldIndex=0;
    CHECK(!validateSettings(s,roi,&error)); // field cannot silently change
    s.measuringFieldIndex=0;
    CHECK(!validateSettings(s,roi,&error)); // 200 Hz > Large 144 Hz
    ScanTiming timing;
    auto setRate=[&](int rate){
        s.profileRateHz=rate;
        CHECK(calculateScanTiming(rate,s.requestedLineSpacingMm,timing,&error));
        s.scanSpeedMmS=timing.scanSpeedMmS;
    };
    setRate(100);
    CHECK(validateSettings(s,roi,&error)); // verified Large works
    setRate(145);
    CHECK(!validateSettings(s,roi,&error));
    setRate(100);
    roi.measuringFieldIndex=2;
    CHECK(!validateSettings(s,roi,&error));
    s.measuringFieldIndex=2;
    CHECK(validateSettings(s,roi,&error));
    roi.measuringFieldIndex=7;
    s.measuringFieldIndex=7;
    setRate(200);
    CHECK(validateSettings(s,roi,&error));
    for(int speed=1;speed<=5;++speed){
        s.positioningSpeedMmS=speed;
        CHECK(validateSettings(s,roi,&error));
    }
    s.positioningSpeedMmS=0.5;
    CHECK(!validateSettings(s,roi,&error));
    s.positioningSpeedMmS=3;
    setRate(200);s.maximumFrameSpacingMm=0.004;
    CHECK(!validateSettings(s,roi,&error));
    s.maximumFrameSpacingMm=0.010;
    CHECK(validateSettings(s,roi,&error));
    // A 333Hz requested frequency quantizes to 301 × 10us; speed uses effective 332.226Hz.
    setRate(333);s.maximumFrameSpacingMm=0.003005;
    CHECK(timing.period10us==301);
    CHECK(std::abs(timing.scanSpeedMmS-0.005*100000.0/301.0)<1e-9);
    CHECK(!validateSettings(s,roi,&error));
    setRate(200);s.maximumFrameSpacingMm=0.010;
    CHECK(visibleArcSpanDeg(0,95,9.525,roi)>359);
    CHECK(fullCircleInsideOptics(0,95,9.525,roi));
    CHECK(!fullCircleInsideOptics(0,70.3,9.525,roi));
    CHECK(!fullCircleInsideOptics(19,95,9.525,roi));
    // A visible large arc can be fitted even when the full 360-degree circle
    // physically exceeds the selected ROI width (important for field selection).
    OpticalWindow narrow=roi;
    narrow.xMinMm=-8;narrow.xMaxMm=8;
    CHECK(!fullCircleInsideOptics(0,95,9.525,narrow));
    CHECK(maxCircleFitArcInsideOptics(0,95,9.525,narrow));
    narrow.xMinMm=-2;narrow.xMaxMm=2;
    CHECK(!maxCircleFitArcInsideOptics(0,95,9.525,narrow));
    s.scanSpeedMmS=1; s.requestedLineSpacingMm=0.01;
    CHECK(!validateSettings(s,roi,&error)); // hard-coded speed forbidden
    setRate(200);
    CHECK(std::abs(s.scanSpeedMmS-2.0)<1e-9);
    CHECK(validateSettings(s,roi,&error));
    s.requestedLineSpacingMm=0.005;setRate(200);
    s.measuringFieldIndex=128;
    CHECK(!validateSettings(s,roi,&error));
    s.measuringFieldIndex=7;
    CHECK(validateSettings(s,roi,&error));
    CHECK(visibleArcSpanDeg(0,140,9.525,roi)<0.1);
    CHECK(visibleArcSpanDeg(0,70.3,9.525,roi)>40);
    CHECK(motionTimeoutMs(0.1,5,25,25)<motionTimeoutMs(30,1,25,25));
    CHECK(motionTimeoutMs(0.1,5,25,25)>=15000);
    CHECK(motionTimeoutMs(100000,1,25,25)==600000);
    // With identity machine-axes alignment, moving machine X need not span
    // sensor Y. A physically degenerate scan must be rejected.
    Physical17Parameters physical;
    JointPose5d at;
    const Vec3d center{0,0,95};
    s.zBeginMm=0;s.zEndMm=2;s.zIntervalMm=2;
    std::vector<HeightPlan> plan;
    CHECK(!planHeights(s,roi,at,center,physical,plan,&error));
    CHECK(plan.empty());
    // Synthetic 90° hand-eye rotation aligns machine X to sensor Y.
    physical[Physical17Parameters::HandEyeRz]=1.5707963267948966;
    CHECK(planHeights(s,roi,at,center,physical,plan,&error));
    CHECK(plan.size()==2);
    CHECK(std::abs(plan[0].machineZMm)<1e-9);
    CHECK(std::abs(plan[1].machineZMm-2.0)<1e-9);
    CHECK(std::abs(plan[0].centerXMachineMm)<1e-5);
    CHECK(plan[0].scanEndXMachineMm>plan[0].scanStartXMachineMm);
    CHECK(plan[0].sevenPointX[3]==plan[0].centerXMachineMm);
    narrow=roi;narrow.xMinMm=-8;narrow.xMaxMm=8;
    CHECK(planHeights(s,narrow,at,center,physical,plan,&error));
    narrow.xMinMm=-2;narrow.xMaxMm=2;
    CHECK(!planHeights(s,narrow,at,center,physical,plan,&error));
    CHECK(planHeights(s,roi,at,center,physical,plan,&error));
    const auto accepted=findOpticalZIntervals(s,roi,at,center,physical);
    CHECK(!accepted.empty()&&accepted.front().zStartMm<0.001&&accepted.front().zEndMm>=1.999);
    s.zEndMm=3.1;
    CHECK(planHeights(s,roi,at,center,physical,plan,&error));
    CHECK(plan.size()==3 && std::abs(plan.back().machineZMm-3.1)<1e-9);
    roi.zMinMm=120.0;roi.zMaxMm=121.0;
    CHECK(!planHeights(s,roi,at,center,physical,plan,&error));
    CHECK(findOpticalZIntervals(s,roi,at,center,physical).empty());
    std::vector<ScanPeakSample> scan;
    for(int i=0;i<=600;++i){
        const double x=-3+0.01*i,delta=x-0.18;
        scan.push_back({x,std::sqrt(9.525*9.525-delta*delta)});
    }
    ScanPeak peak;
    CHECK(findScannedMaximumCircle(scan,-11,11,0,9.525,0.5,peak,&error));
    CHECK(std::abs(peak.machineX-0.18)<1e-4);
    CHECK(std::abs(peak.fittedRadiusMm-9.525)<1e-4);
    CHECK(peak.usedSamples>=600);
    CHECK(!findScannedMaximumCircle({},-11,11,0,9.525,0.5,peak,&error));
    for(auto& v:scan)v.radiusMm*=0.9;
    CHECK(!findScannedMaximumCircle(scan,-11,11,0,9.525,0.5,peak,&error));
    std::cout<<"Experiment3 portable PASS: derived line-speed, conservative fitting arc, "
                 "Z intervals, scanned-peak fitting and failure gates\n";
}
