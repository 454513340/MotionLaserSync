#include "calibration/SphereDistanceAngleStudy.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <set>
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
    const Vec3d optical(0.0,0.0,0.0);

    // D + eta must determine the complete X-Z circle-center position.
    const auto g=SphereDistanceAngleStudy::targetGeometry(R,100.0,20.0,15.0,+1,optical);
    const double eta=15.0*3.14159265358979323846/180.0;
    if(!near(g.targetCircleCenterSensorMm.x,100.0*std::sin(eta),1e-9) ||
       !near(g.targetCircleCenterSensorMm.z,100.0*std::cos(eta),1e-9) ||
       !near(g.centerBearingAngleDeg,15.0,1e-12)) return fail("D/eta target circle center");

    Physical17Parameters model;
    SphereStudyPathOptions one;
    one.sphereRadiusMm=R;
    one.measurementDistancesMm={100.0};
    one.sectionAnglesDeg={20.0};
    one.centerBearingAnglesDeg={15.0};
    one.spaceFillingSampling=false;
    one.referenceJoint={0,0,0,0,0};
    one.xMinMm=-300;one.xMaxMm=300;one.yMinMm=-300;one.yMaxMm=300;one.zMinMm=-300;one.zMaxMm=300;
    one.cMinDeg=-90;one.cMaxDeg=90;one.bMinDeg=-90;one.bMaxDeg=90;
    one.rotarySamples={{0,0,0}};
    one.sensorZSafetyMarginMm=0.0;one.sensorXSafetyMarginMm=0.0;
    std::string err;
    const auto p1=SphereDistanceAngleStudy::generatePath(sphereM,model,one,err);
    if(p1.size()!=1 || !p1.front().valid) return fail("single eta path");
    const auto obs=SphereDistanceAngleStudy::observeGeometry(p1.front().joints,sphereM,R,model,optical);
    if(!obs.valid || !near(obs.measurementDistanceMm,100.0,1e-5) ||
       !near(obs.sectionAngleDeg,20.0,1e-5) || !near(obs.centerBearingAngleDeg,15.0,1e-5))
        return fail("observed D/gamma/eta");

    // Y planner: create a larger feasible candidate pool, then retain only a
    // maximin space-filling subset rather than the full Cartesian product.
    SphereStudyPathOptions sf=one;
    sf.measurementDistancesMm={90.0,100.0};
    sf.sectionAnglesDeg={0.0,20.0};
    sf.centerBearingAnglesDeg={-10.0,0.0,10.0};
    sf.rotarySamples.clear();
    sf.automaticPlaneFamily=true;
    sf.planeDirectionsPerGeometry=4;
    sf.rotaryCandidateStepDeg=30.0;
    sf.minimumPlaneSeparationDeg=5.0;
    sf.spaceFillingSampling=true;
    sf.targetPoseCount=12;
    std::string sfErr;
    const auto path=SphereDistanceAngleStudy::generatePath(sphereM,model,sf,sfErr);
    if(path.size()!=12) return fail("space filling target count");

    double minD=1e9,maxD=-1e9,minG=1e9,maxG=-1e9,minE=1e9,maxE=-1e9;
    std::set<int> planes;
    for(const auto& p:path){
        if(!p.valid) return fail("space filling invalid pose");
        minD=std::min(minD,p.geometry.measurementDistanceMm);maxD=std::max(maxD,p.geometry.measurementDistanceMm);
        minG=std::min(minG,p.geometry.sectionAngleDeg);maxG=std::max(maxG,p.geometry.sectionAngleDeg);
        minE=std::min(minE,p.geometry.centerBearingAngleDeg);maxE=std::max(maxE,p.geometry.centerBearingAngleDeg);
        planes.insert(p.rotarySampleId);
        const auto o=SphereDistanceAngleStudy::observeGeometry(p.joints,sphereM,R,model,optical);
        if(!o.valid || !near(o.measurementDistanceMm,p.geometry.measurementDistanceMm,1e-4) ||
           !near(o.sectionAngleDeg,p.geometry.sectionAngleDeg,1e-4) ||
           !near(o.centerBearingAngleDeg,p.geometry.centerBearingAngleDeg,1e-4))
            return fail("space filling forward geometry");
    }
    if(!(minD<95.0 && maxD>95.0 && minG<5.0 && maxG>15.0 && minE<-5.0 && maxE>5.0))
        return fail("space filling geometry coverage");
    if(planes.size()<2) return fail("space filling plane diversity");

    // 3-D quality kernel must distinguish eta rather than silently collapsing
    // opposite sides of the sensor field.
    std::vector<SphereStudyQualityObservation> qobs;
    qobs.push_back({100,20,-15,0.030,0.010,0.05,0.04,0.90});
    qobs.push_back({100,20,  0,0.010,0.001,0.01,0.01,0.98});
    qobs.push_back({100,20, 15,0.040,0.015,0.08,0.06,0.86});
    const auto q0=SphereDistanceAngleStudy::predictQualityKernel(qobs,100,20,0,4,10,5);
    const auto qp=SphereDistanceAngleStudy::predictQualityKernel(qobs,100,20,15,4,10,5);
    if(!q0.valid||!qp.valid||!(q0.predictedAbsoluteP95Mm < qp.predictedAbsoluteP95Mm))
        return fail("eta-aware quality kernel");

    std::cout<<"PASS V27.6.16Y center-bearing eta + D/gamma/eta/plane space filling\n";
    return 0;
}
