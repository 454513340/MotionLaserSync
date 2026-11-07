#include "calibration/SphereDistanceAngleStudy.h"

#include <cmath>
#include <iostream>
#include <random>
#include <vector>

using namespace mls::calibration;

namespace {
bool near(double a,double b,double t){return std::abs(a-b)<=t;}
int fail(const char* s){std::cerr<<"FAIL: "<<s<<"\n";return 1;}
}

int main()
{
    const double R=9.525;
    const Vec3d sphereM(42.0,-17.5,103.0);
    Physical17Parameters model;

    SphereStudyPathOptions opt;
    opt.sphereRadiusMm=R;
    opt.opticalCenterSensorMm={0,0,0};
    opt.measurementDistancesMm={95.0,100.0};
    opt.sectionAnglesDeg={0.0,30.0};
    opt.centerBearingAnglesDeg={-5.0,0.0,5.0};
    opt.referenceJoint={0,0,0,0,0};
    opt.xMinMm=-300;opt.xMaxMm=300;opt.yMinMm=-300;opt.yMaxMm=300;opt.zMinMm=-300;opt.zMaxMm=300;
    opt.cMinDeg=-90;opt.cMaxDeg=90;opt.bMinDeg=-90;opt.bMaxDeg=90;
    opt.sensorXMinMm=-25;opt.sensorXMaxMm=25;opt.sensorZMinMm=70;opt.sensorZMaxMm=120;
    opt.sensorXSafetyMarginMm=1;opt.sensorZSafetyMarginMm=1;
    opt.minimumPlaneIntersectionMarginMm=0.5;
    opt.minimumPredictedVisibleArcDeg=75;
    opt.visibleArcSamples=720;
    opt.rotarySamples={{0,0,0}};
    opt.spaceFillingSampling=false;

    std::string err;
    auto path=SphereDistanceAngleStudy::generatePath(sphereM,model,opt,err);
    if(path.empty())return fail("optically feasible path empty");
    for(const auto& p:path){
        if(!p.valid)return fail(p.message.c_str());
        if(!(p.predictedVisibleArcDeg>=75.0-1e-6))return fail("visible arc gate");
        if(!(p.predictedPlaneIntersectionMarginMm>0.5))return fail("intersection margin gate");
        const auto sim=SphereDistanceAngleStudy::simulateOpticalFeasibility(p.joints,sphereM,R,model,opt);
        if(!sim.valid||!sim.planeIntersectsSphere||!sim.visibleArcSufficient)return fail("forward optical feasibility");
    }

    // A circle center far to +X may be kinematically reachable, but it must not
    // enter the autonomous path when the actual project FOV is only X=-25..25.
    SphereStudyPathOptions badFov=opt;
    badFov.measurementDistancesMm={100.0};
    badFov.sectionAnglesDeg={20.0};
    badFov.centerBearingAnglesDeg={22.0}; // Xc ~= 37.5 mm
    std::string badErr;
    const auto badPath=SphereDistanceAngleStudy::generatePath(sphereM,model,badFov,badErr);
    int badValid=0;for(const auto& p:badPath)if(p.valid)++badValid;
    if(badValid!=0)return fail("actual X FOV must reject off-field sphere arc");

    // Explicit laser-plane miss: sphere center sensor Y is outside R.
    JointPose5d miss;
    const Vec3d wantedMiss(0.0,R+1.0,100.0);
    miss.x=sphereM.x-wantedMiss.x;
    miss.y=sphereM.y-wantedMiss.y;
    miss.z=sphereM.z-wantedMiss.z;
    const auto missSim=SphereDistanceAngleStudy::simulateOpticalFeasibility(miss,sphereM,R,model,opt);
    if(missSim.valid||missSim.planeIntersectsSphere)return fail("laser plane miss must be rejected");

    // Real-profile Probe: three or more of five frames contain a robust sphere arc.
    const auto tg=SphereDistanceAngleStudy::targetGeometry(R,100.0,20.0,0.0,+1,{0,0,0});
    const double r=tg.theoreticalSectionRadiusMm;
    std::mt19937 rng(7u);std::normal_distribution<double> n(0.0,0.004);
    std::vector<SphereStudyRawProfile> raw(5);
    for(int f=0;f<5;++f){
        raw[f].profileIndex=100+f;raw[f].callbackOrdinal=200+f;raw[f].samples.resize(401);
        for(int i=0;i<401;++i){
            auto& s=raw[f].samples[static_cast<std::size_t>(i)];s.index=i;
            const double x=-20.0+40.0*i/400.0;s.xMm=x;s.zMm=115.0;s.coordinateValid=true;
            if(std::abs(x)<=r)s.zMm=100.0-std::sqrt(std::max(0.0,r*r-x*x))+n(rng);
        }
        if(f==4){ // one deliberately useless frame; group probe should still pass
            for(auto& s:raw[f].samples){s.coordinateValid=false;s.zMm=0.0;}
        }
    }
    SphereStudyProfileAnalysisOptions pa;
    pa.ransac.inlierThresholdMm=0.05;pa.ransac.minInliers=20;pa.ransac.minInlierRatio=0.1;pa.ransac.minArcSpanDeg=30;
    const auto probe=SphereDistanceAngleStudy::probeProfileGroup(raw,tg.targetSphereCenterSensorMm,R,+1,pa,3);
    if(!probe.success||probe.detectedCircleCount<3||!near(probe.medianCenterZmm,100.0,0.08))return fail("real-profile probe Circle RANSAC");

    // A watchdog timeout with fewer frames than the requested quorum must not
    // silently degrade "3 circles required" into a 1/1 success.
    std::vector<SphereStudyRawProfile> partial{raw[0]};
    const auto partialProbe=SphereDistanceAngleStudy::probeProfileGroup(partial,tg.targetSphereCenterSensorMm,R,+1,pa,3);
    if(partialProbe.success)return fail("partial Probe must honor requested quorum");

    // Reacquisition inverse: fixed C/B but shift sphere Y in sensor frame.
    JointPose5d solved;Vec3d predicted;double residual=0;std::string solveErr;
    Vec3d reacqTarget=tg.targetSphereCenterSensorMm;reacqTarget.y+=0.5;
    if(!SphereDistanceAngleStudy::solvePoseForSphereCenterTarget(sphereM,model,opt,reacqTarget,0,0,opt.referenceJoint,solved,predicted,residual,solveErr))
        return fail("reacquisition RTCP-equivalent inverse");
    if(!near(predicted.y,reacqTarget.y,1e-6)||residual>1e-5)return fail("reacquisition forward residual");

    std::cout<<"PASS V27.6.16Z optical intersection + actual FOV + Probe/reacquisition\n";
    return 0;
}
