#include "calibration/HardwareTemporalCalibration.h"
#include "runtime/HardwareTemporalCorrection.h"

#include <cmath>
#include <iostream>
#include <vector>

using namespace mls::calibration;
using namespace mls::runtime;

namespace {
void require(bool v, const char* message) {
    if (!v) { std::cerr << "FAIL: " << message << "\n"; std::exit(2); }
}

double coordinate(const Vec3d& p, TemporalAxis a) {
    return a==TemporalAxis::X?p.x:(a==TemporalAxis::Y?p.y:p.z);
}
}

int main() {
    constexpr double delay = 0.0008;
    std::vector<HardwareTemporalObservation> obs;
    std::uint64_t id=1;
    const double speeds[] = {-5.0,-3.0,3.0,5.0};
    for (int ai=0; ai<3; ++ai) {
        const auto axis=static_cast<TemporalAxis>(ai);
        for(double v:speeds) {
            HardwareTemporalObservation o;
            o.id=id++; o.axis=axis; o.signedSpeedMmPerSec=v;
            o.sphereCenterMachineMm={100.0,200.0,300.0};
            if(axis==TemporalAxis::X)o.sphereCenterMachineMm.x-=v*delay;
            if(axis==TemporalAxis::Y)o.sphereCenterMachineMm.y-=v*delay;
            if(axis==TemporalAxis::Z)o.sphereCenterMachineMm.z-=v*delay;
            o.sphereRmsMm=0.01; o.fittedRadiusMm=9.525; o.sphereInliers=1000; o.profileCount=100;
            o.resolutionIndex=0;o.exposureTime10Us=100;o.idleTime10Us=300;o.triggerPeriodMs=5;
            obs.push_back(o);
        }
    }
    auto solved=HardwareTemporalCalibration::solve(obs);
    require(solved.success,"temporal solve succeeds");
    require(std::abs(solved.delaySec-delay)<1e-10,"common delay recovered");
    HardwareTemporalCriteria criteria;
    criteria.minRSquared=0.99;
    require(HardwareTemporalCalibration::evaluateQuality(solved,criteria),"quality gates pass");

    auto bad=obs; bad.back().idleTime10Us=301;
    auto inconsistent=HardwareTemporalCalibration::solve(bad);
    require(!inconsistent.success && !inconsistent.configurationConsistent,"mixed sensor config rejected");

    std::vector<TemporalPoseSample> poses;
    TemporalPoseSample probe; probe.ordinal=0; poses.push_back(probe);
    for(std::uint64_t k=1;k<=60;++k){TemporalPoseSample p;p.ordinal=k;p.joints[0]=10.0+2.0*0.005*double(k-1);poses.push_back(p);}
    auto motion=HardwareTemporalCalibration::estimateDominantLinearMotion(poses,0.005);
    require(motion.success && motion.axis==TemporalAxis::X,"motion X detected");
    require(std::abs(motion.signedSpeedMmPerSec-2.0)<1e-9,"measured latch velocity recovered");

    Frame5D a,b;
    a.session=b.session=7;a.ordinal=1;b.ordinal=2;
    a.timing=b.timing=PoseTiming::HardwareLatched;
    a.feedbackMask=b.feedbackMask=31;a.latchMask=b.latchMask=31;a.commandMask=b.commandMask=0;
    a.joints={0,0,0,0,0};b.joints={0.025,0,0,0,0};
    a.raw=a.command=a.joints;b.raw=b.command=b.joints;
    a.physicalParameters=b.physicalParameters={};
    HardwareTemporalCorrectionConfig cfg;cfg.enabled=true;cfg.delaySec=0.001;cfg.triggerPeriodSec=0.005;
    auto cor=interpolateHardwareTemporalFrame(a,b,cfg);
    require(cor.success,"runtime temporal interpolation succeeds");
    require(std::abs(cor.frame.joints[0]-0.005)<1e-12,"runtime temporal X interpolation correct");
    require(cor.frame.timing==PoseTiming::HardwareTemporalInterpolated,"temporal provenance timing recorded");
    require(cor.frame.temporalNextOrdinal==2,"look-ahead ordinal recorded");

    // V27.6.46 trajectory-based solver: arbitrary XYZ straight lines share one
    // physical delay.  No Y-only/Z-only complete sphere is required.
    std::vector<TemporalTrajectoryObservation> traj;
    std::uint64_t tid=100;
    std::uint64_t pair=10;
    const Vec3d c0{120.0,80.0,40.0};
    const Vec3d vels[]={{2.0,0.0,0.0},{3.5,0.0,0.0},{5.0,0.0,0.0},
                        {4.0,0.8,0.0},{4.0,0.0,0.8},{4.0,0.8,0.8}};
    for(const auto& vp:vels){
        for(int sign: {+1,-1}){
            TemporalTrajectoryObservation o;
            o.id=tid++;o.planId=o.id;o.pairId=pair;o.directionSign=sign;
            o.velocityMmPerSec={sign*vp.x,sign*vp.y,sign*vp.z};
            o.sphereCenterMachineMm={c0.x-delay*o.velocityMmPerSec.x,
                                     c0.y-delay*o.velocityMmPerSec.y,
                                     c0.z-delay*o.velocityMmPerSec.z};
            o.sphereRmsMm=0.008;o.fittedRadiusMm=9.525;o.sphereInliers=800;o.profileCount=300;
            o.resolutionIndex=0;o.exposureTime10Us=100;o.idleTime10Us=300;o.triggerPeriodMs=5;
            traj.push_back(o);
        }
        ++pair;
    }
    auto trajectorySolved=HardwareTemporalCalibration::solveTrajectoryPairs(traj);
    require(trajectorySolved.success,"V46 arbitrary-XYZ trajectory solve succeeds");
    require(std::abs(trajectorySolved.delaySec-delay)<1e-12,"V46 trajectory common delay recovered");
    TemporalTrajectoryCriteria tc;
    tc.minPairs=3;tc.minDistinctSpeedLevels=2;tc.minRSquared=0.99;
    tc.maxDelayStdSec=1e-6;tc.maxPairResidualMm=1e-8;
    require(HardwareTemporalCalibration::evaluateTrajectoryQuality(trajectorySolved,tc),
            "V46 trajectory quality gates pass");

    std::vector<TemporalPoseSample> diagonalPoses;
    TemporalPoseSample dp0;dp0.ordinal=0;diagonalPoses.push_back(dp0);
    for(std::uint64_t k=1;k<=80;++k){
        TemporalPoseSample p;p.ordinal=k;
        const double t=0.005*double(k-1);
        p.joints[0]=10.0+4.0*t;p.joints[1]=20.0+0.8*t;p.joints[2]=30.0+0.4*t;
        diagonalPoses.push_back(p);
    }
    auto vectorMotion=HardwareTemporalCalibration::estimateLinearMotionVector(diagonalPoses,0.005);
    require(vectorMotion.success,"V46 vector MPOS motion estimate succeeds");
    require(std::abs(vectorMotion.velocityMmPerSec.x-4.0)<1e-9 &&
            std::abs(vectorMotion.velocityMmPerSec.y-0.8)<1e-9 &&
            std::abs(vectorMotion.velocityMmPerSec.z-0.4)<1e-9,
            "V46 actual REGIST XYZ velocity vector recovered");

    // Hold-out pair: raw data contain 800 us equivalent delay, candidate
    // correction leaves only 30 us residual. Validation must report improvement.
    constexpr double residualDelay=0.00003;
    std::vector<TemporalValidationObservation> val;
    std::uint64_t vid=500;pair=100;
    for(const Vec3d vp: {Vec3d{4.0,0.8,0.0},Vec3d{4.0,0.0,0.8},Vec3d{4.0,0.8,0.8}}){
        for(int sign: {+1,-1}){
            TemporalValidationObservation o;o.id=vid++;o.pairId=pair;o.directionSign=sign;
            o.velocityMmPerSec={sign*vp.x,sign*vp.y,sign*vp.z};
            o.rawSphereCenterMachineMm={c0.x-delay*o.velocityMmPerSec.x,
                                        c0.y-delay*o.velocityMmPerSec.y,
                                        c0.z-delay*o.velocityMmPerSec.z};
            o.correctedSphereCenterMachineMm={c0.x-residualDelay*o.velocityMmPerSec.x,
                                              c0.y-residualDelay*o.velocityMmPerSec.y,
                                              c0.z-residualDelay*o.velocityMmPerSec.z};
            o.rawSphereRmsMm=0.010;o.correctedSphereRmsMm=0.007;
            val.push_back(o);
        }
        ++pair;
    }
    TemporalValidationCriteria vc;vc.minPairs=2;vc.minImprovementFraction=0.50;vc.maxResidualDelaySec=0.00015;
    auto validated=HardwareTemporalCalibration::evaluateValidationPairs(val,vc);
    require(validated.success && validated.passed,"V46 independent hold-out validation passes");
    require(std::abs(validated.rawEquivalentDelaySec-delay)<1e-12,"V46 raw hold-out delay recovered");
    require(std::abs(validated.correctedResidualDelaySec-residualDelay)<1e-12,"V46 corrected residual delay recovered");

    std::cout << "V27.6.46 hardware temporal workflow tests PASS\n";
    return 0;
}
