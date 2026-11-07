#include "calibration/SphereDistanceAngleStudy.h"

#include <cmath>
#include <iostream>
#include <string>

using namespace mls::calibration;

namespace {
int fail(const char* s){std::cerr<<"FAIL: "<<s<<"\n";return 1;}
bool near(double a,double b,double t){return std::abs(a-b)<=t;}

SphereStudyPathOptions baseOptions(double radius)
{
    SphereStudyPathOptions o;
    o.sphereRadiusMm=radius;
    o.opticalCenterSensorMm={0,0,0};
    o.referenceJoint={0,0,0,0,0};
    o.xMinMm=-500;o.xMaxMm=500;o.yMinMm=-500;o.yMaxMm=500;o.zMinMm=-500;o.zMaxMm=500;
    o.cMinDeg=-90;o.cMaxDeg=90;o.bMinDeg=-90;o.bMaxDeg=90;
    o.sensorXMinMm=-25;o.sensorXMaxMm=25;o.sensorZMinMm=70;o.sensorZMaxMm=120;
    o.sensorXSafetyMarginMm=1;o.sensorZSafetyMarginMm=1;
    o.minimumPlaneIntersectionMarginMm=0.01;
    o.minimumPredictedVisibleArcDeg=30;
    o.minimumPredictedVisibleArcLengthMm=5.0;
    o.visibleArcSamples=720;
    o.rotarySamples={{0,0,0}};
    return o;
}
}

int main()
{
    const double R=9.525;
    const Vec3d sphereM(42.0,-17.5,103.0);
    Physical17Parameters model;

    // 1) A nearly tangent section can have a large angular span but still be
    // physically too short for robust Circle RANSAC.  AB rejects it by mm arc.
    auto shortArcOpt=baseOptions(R);
    const double desiredSectionRadius=0.7;
    const double h=std::sqrt(R*R-desiredSectionRadius*desiredSectionRadius);
    const Vec3d shortTarget(0.0,h,100.0);
    JointPose5d shortPose;Vec3d predicted;double residual=0.0;std::string solveErr;
    if(!SphereDistanceAngleStudy::solvePoseForSphereCenterTarget(
        sphereM,model,shortArcOpt,shortTarget,0,0,shortArcOpt.referenceJoint,
        shortPose,predicted,residual,solveErr))return fail("short-arc inverse solve");
    const auto shortSim=SphereDistanceAngleStudy::simulateOpticalFeasibility(
        shortPose,sphereM,R,model,shortArcOpt);
    if(!shortSim.planeIntersectsSphere)return fail("near-tangent plane should geometrically intersect");
    if(!(shortSim.visibleArcDeg>=shortArcOpt.minimumPredictedVisibleArcDeg))return fail("test setup needs adequate angular span");
    if(!(shortSim.visibleArcLengthMm<shortArcOpt.minimumPredictedVisibleArcLengthMm))return fail("test setup needs short physical arc");
    if(shortSim.valid||shortSim.visibleArcSufficient)return fail("physical visible arc length gate");

    // 2) The GUI simulation geometry must use the same gate and expose the
    // guarded XZ rectangle, full section circle, and longest visible arc.
    auto goodOpt=baseOptions(R);
    goodOpt.minimumPlaneIntersectionMarginMm=0.5;
    goodOpt.minimumPredictedVisibleArcDeg=75;
    goodOpt.minimumPredictedVisibleArcLengthMm=6.0;
    const auto tg=SphereDistanceAngleStudy::targetGeometry(R,100.0,20.0,0.0,+1,{0,0,0});
    JointPose5d goodPose;Vec3d goodPred;double goodResidual=0.0;std::string goodErr;
    if(!SphereDistanceAngleStudy::solvePoseForSphereCenterTarget(
        sphereM,model,goodOpt,tg.targetSphereCenterSensorMm,0,0,goodOpt.referenceJoint,
        goodPose,goodPred,goodResidual,goodErr))return fail("good simulation inverse solve");
    const auto demo=SphereDistanceAngleStudy::buildScanSimulation(
        goodPose,sphereM,R,model,goodOpt,360);
    if(!demo.valid||!demo.optical.valid)return fail("good scan simulation validity");
    if(demo.laserFieldCornersSensorMm.size()!=4)return fail("simulation laser XZ field corners");
    if(demo.sectionCircleSensorMm.size()<360)return fail("simulation section circle samples");
    if(demo.visibleArcSensorMm.empty())return fail("simulation visible arc samples");
    if(!(demo.optical.visibleArcLengthMm>=6.0))return fail("simulation visible arc length metric");

    // 3) Original D endpoints both miss a deliberately narrow Z field, while
    // the deterministic D midpoint intersects.  Adaptive replacement should
    // recover D=100 inside the user range and still pass the same optical gate.
    SphereStudyPathOptions adaptive=baseOptions(R);
    adaptive.measurementDistancesMm={80.0,120.0};
    adaptive.sectionAnglesDeg={0.0};
    adaptive.centerBearingAnglesDeg={0.0};
    adaptive.sensorXMinMm=-15;adaptive.sensorXMaxMm=15;
    adaptive.sensorZMinMm=90;adaptive.sensorZMaxMm=110;
    adaptive.sensorXSafetyMarginMm=0;adaptive.sensorZSafetyMarginMm=0;
    adaptive.minimumPlaneIntersectionMarginMm=0.5;
    adaptive.minimumPredictedVisibleArcDeg=40.0;
    adaptive.minimumPredictedVisibleArcLengthMm=5.0;
    adaptive.spaceFillingSampling=true;
    adaptive.targetPoseCount=1;
    adaptive.adaptiveOpticalResampling=true;
    adaptive.adaptiveOpticalResamplingPasses=1;
    adaptive.maxAdaptiveCandidateCount=100;
    std::string pathErr;
    const auto path=SphereDistanceAngleStudy::generatePath(sphereM,model,adaptive,pathErr);
    if(path.size()!=1||!path.front().valid)return fail("adaptive replacement path");
    if(!path.front().adaptivelyRegenerated)return fail("adaptive replacement flag");
    if(!near(path.front().geometry.measurementDistanceMm,100.0,1e-9))return fail("adaptive midpoint D");
    if(!(path.front().predictedVisibleArcLengthMm>=adaptive.minimumPredictedVisibleArcLengthMm))return fail("adaptive replacement optical gate");

    std::cout<<"PASS V27.6.16AB scan simulation + physical arc gate + adaptive replan\n";
    return 0;
}
