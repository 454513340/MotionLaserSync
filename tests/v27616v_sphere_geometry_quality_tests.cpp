#include "calibration/SphereDistanceAngleStudy.h"

#include <cmath>
#include <iostream>
#include <random>
#include <vector>

using namespace mls::calibration;

namespace {
constexpr double pi = 3.14159265358979323846;
bool near(double a,double b,double t){return std::abs(a-b)<=t;}
int fail(const char* s){std::cerr<<"FAIL: "<<s<<"\n";return 1;}
}

int main()
{
    const Vec3d trueCenter(42.0,-17.5,103.0);
    const double R=9.525;
    std::mt19937 rng(12345u);
    std::normal_distribution<double> cloudNoise(0.0,0.015);
    std::vector<Vec3d> cloud;
    for(int i=0;i<1600;++i){
        const double u=(i+0.5)/1600.0,z=1.0-2.0*u,phi=2*pi*std::fmod(i*0.6180339887498949,1.0),s=std::sqrt(std::max(0.0,1-z*z));
        cloud.emplace_back(trueCenter.x+R*s*std::cos(phi)+cloudNoise(rng),trueCenter.y+R*s*std::sin(phi)+cloudNoise(rng),trueCenter.z+R*z+cloudNoise(rng));
    }
    std::uniform_real_distribution<double> out(-40.0,40.0);
    for(int i=0;i<500;++i)cloud.emplace_back(trueCenter.x+out(rng),trueCenter.y+out(rng),trueCenter.z+out(rng));
    SphereCloudRansacOptions ro;ro.maxIterations=900;ro.inlierThresholdMm=0.08;ro.minInliers=900;ro.minInlierRatio=0.35;ro.expectedRadiusMm=R;ro.maxRadiusDeviationMm=0.8;
    const auto sf=SphereDistanceAngleStudy::fitMachineSphereRansac(cloud,ro);
    if(!sf.success||!near(sf.centerMachineMm.x,trueCenter.x,0.04)||!near(sf.centerMachineMm.y,trueCenter.y,0.04)||!near(sf.centerMachineMm.z,trueCenter.z,0.04))return fail("3-D sphere RANSAC");

    // New geometry: D is circle-center to optical-center Euclidean distance;
    // gamma is unsigned section eccentricity. Target circle center is on +Z.
    const auto tg=SphereDistanceAngleStudy::targetGeometry(R,100.0,20.0,+1,{0,0,0});
    const double h=R*std::sin(20*pi/180.0), rRef=R*std::cos(20*pi/180.0);
    if(!near(tg.targetCircleCenterSensorMm.x,0.0,1e-12)||!near(tg.targetCircleCenterSensorMm.z,100.0,1e-12)||
       !near(tg.targetSphereCenterSensorMm.y,h,1e-12)||!near(tg.theoreticalSectionRadiusMm,rRef,1e-12))return fail("D/gamma target geometry");

    Physical17Parameters model; // nominal model
    SphereStudyPathOptions po;po.sphereRadiusMm=R;po.measurementDistancesMm={90.0,100.0};po.sectionAnglesDeg={0.0,20.0};po.spaceFillingSampling=false;
    po.referenceJoint={0,0,0,0,0};po.xMinMm=-300;po.xMaxMm=300;po.yMinMm=-300;po.yMaxMm=300;po.zMinMm=-200;po.zMaxMm=300;
    po.cMinDeg=-90;po.cMaxDeg=90;po.bMinDeg=-90;po.bMaxDeg=90;po.rotarySamples={{0,-20,0},{1,20,15}};po.sensorZSafetyMarginMm=0.0;
    std::string err;const auto path=SphereDistanceAngleStudy::generatePath(trueCenter,model,po,err);
    if(path.size()!=8)return fail("D/gamma/C/B path count");
    bool sawCneg=false,sawCpos=false,sawB=false;
    for(const auto& p:path){if(!p.valid)return fail(p.message.c_str());sawCneg|=near(p.joints.c,-20,1e-9);sawCpos|=near(p.joints.c,20,1e-9);sawB|=near(p.joints.b,15,1e-9);
        const auto g=SphereDistanceAngleStudy::observeGeometry(p.joints,trueCenter,R,model);
        if(!g.valid||!near(g.measurementDistanceMm,p.geometry.measurementDistanceMm,1e-4)||!near(g.sectionAngleDeg,p.geometry.sectionAngleDeg,1e-4))return fail("planned D/gamma observation");}
    if(!sawCneg||!sawCpos||!sawB)return fail("rotary diversity missing");

    // V27.6.16X automatic plane family: do not form a blind CxB product.
    // Select a compact, reachable set of well-separated profile-plane normals
    // and reuse that family over the D/gamma grid.
    SphereStudyPathOptions px=po;
    px.rotarySamples.clear();
    px.automaticPlaneFamily=true;
    px.planeDirectionsPerGeometry=4;
    px.rotaryCandidateStepDeg=30.0;
    px.minimumPlaneSeparationDeg=5.0;
    std::string xerr;
    const auto xpath=SphereDistanceAngleStudy::generatePath(trueCenter,model,px,xerr);
    if(xpath.empty())return fail("automatic plane family empty");
    int maxPlane=-1;
    for(const auto& p:xpath){
        if(!p.valid)return fail(p.message.c_str());
        maxPlane=std::max(maxPlane,p.rotarySampleId);
        const auto g=SphereDistanceAngleStudy::observeGeometry(p.joints,trueCenter,R,model);
        if(!g.valid||!near(g.measurementDistanceMm,p.geometry.measurementDistanceMm,1e-4)||
           !near(g.sectionAngleDeg,p.geometry.sectionAngleDeg,1e-4))return fail("automatic plane-family D/gamma observation");
    }
    const int planeCount=maxPlane+1;
    if(planeCount<2||planeCount>4)return fail("automatic plane-family count");
    if(xpath.size()!=static_cast<std::size_t>(planeCount*4))return fail("automatic plane-family path count");

    // Static visit at D=100, gamma=20. Generate ordered raw slots. Invalid and
    // anomalous slots inside the real circle interval must survive as quality events.
    JointPose5d q;
    q.x=trueCenter.x-tg.targetSphereCenterSensorMm.x;
    q.y=trueCenter.y-tg.targetSphereCenterSensorMm.y;
    q.z=trueCenter.z-tg.targetSphereCenterSensorMm.z;
    std::normal_distribution<double> profileNoise(0.0,0.004);
    std::vector<SphereStudyRawProfile> group;
    for(int f=0;f<6;++f){
        SphereStudyRawProfile raw;raw.callbackOrdinal=100+f;raw.profileIndex=200+f;raw.samples.resize(181);
        for(int i=0;i<181;++i){
            const double x=-12.0+24.0*i/180.0;auto& s=raw.samples[static_cast<std::size_t>(i)];s.index=i;s.xMm=x;s.zMm=118.0;s.coordinateValid=true;
            if(std::abs(x)<=rRef){s.zMm=100.0-std::sqrt(std::max(0.0,rRef*rRef-x*x))+profileNoise(rng);}
            s.hasMaximum=s.hasWidth=s.hasMoment0=s.hasMoment1=s.hasThreshold=true;
            s.maximum=800+f;s.width=10;s.moment0=5000;s.moment1=2000;s.threshold=100;
        }
        // Known dropouts/outliers inside the circle. Keep X and rich channels.
        for(int idx:{70,71,72,110}){raw.samples[idx].zMm=0.0;raw.samples[idx].coordinateValid=false;}
        for(int idx:{82,98}){raw.samples[idx].zMm+=0.45;}
        group.push_back(std::move(raw));
    }
    SphereStudyProfileAnalysisOptions ao;ao.ransac.maxIterations=600;ao.ransac.inlierThresholdMm=0.08;ao.ransac.minInliers=40;ao.ransac.minInlierRatio=0.20;ao.ransac.minArcSpanDeg=60;ao.minimumConsensusProfiles=3;ao.adaptiveMinThresholdMm=0.008;
    const auto ga=SphereDistanceAngleStudy::analyzeProfileGroup(q,group,trueCenter,R,model,{0,0,0},ao);
    if(!ga.valid||ga.profiles.size()!=group.size())return fail(ga.message.c_str());
    const auto& m=ga.profiles.front();
    if(!m.valid||!m.circleDetected)return fail(m.message.c_str());
    if(!near(m.measurementDistanceMm,100.0,0.08)||!near(m.sectionAngleDeg,20.0,0.08)||!near(m.theoreticalRadiusMm,rRef,0.02))return fail("measured D/gamma definition");
    if(m.dropoutCount<4||m.geometricOutlierCount<2||m.evaluationPointCount<=m.goodPointCount)return fail("ROI must retain dropout/outlier slots");
    if(!(m.dropoutRatio>0.0)||!(m.geometricOutlierRatio>0.0)||!(m.goodPointRatio<1.0))return fail("quality ratios");
    bool invalidHasTheta=false;
    for(const auto& p:m.points)if(p.dropout&&std::isfinite(p.localSurfaceAngleDeg)){invalidHasTheta=true;break;}
    if(!invalidHasTheta)return fail("dropout expected local theta");
    if(std::abs(m.radiusBiasMm)>0.05||m.circleRmsSensorMm>0.03||m.absoluteSectionP95Mm<=0.0)return fail("circle/absolute error metrics");
    if(!m.maximumAllValid.valid||!m.maximumGood.valid||m.maximumAllValid.count<=m.maximumGood.count)return fail("rich all-valid vs good statistics");

    std::vector<SphereStudyQualityObservation> obs;
    obs.push_back({98,10,0.0,0.010,0.002,0.01,0.01,0.98});
    obs.push_back({100,20,0.0,0.015,0.003,0.02,0.01,0.97});
    obs.push_back({102,30,0.0,0.025,0.006,0.04,0.03,0.93});
    obs.push_back({104,40,0.0,0.040,0.010,0.08,0.05,0.87});
    const auto pred=SphereDistanceAngleStudy::predictQualityKernel(obs,100.0,20.0,4.0,15.0);
    if(!pred.valid||pred.predictedAbsoluteP95Mm<=0.0||pred.predictedGoodPointRatio<=0.0||pred.effectiveSampleCount<=1.0)return fail("kernel quality map");

    std::cout<<"PASS V27.6.16X sphere plane-family + D/gamma/theta + lossless ROI quality\n";
    return 0;
}
