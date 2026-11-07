#include "calibration/SphereDistanceAngleStudy.h"

#include <cmath>
#include <iostream>

using namespace mls::calibration;

namespace {
int fail(const char* s){ std::cerr << "FAIL: " << s << "\n"; return 1; }
bool near(double a,double b,double tol=1e-9){ return std::abs(a-b)<=tol; }
double width(const Vec3d& a,const Vec3d& b){ return std::abs(b.x-a.x); }
double dot3(const Vec3d& a,const Vec3d& b){ return a.x*b.x+a.y*b.y+a.z*b.z; }
Vec3d sub3(const Vec3d& a,const Vec3d& b){ return {a.x-b.x,a.y-b.y,a.z-b.z}; }
}

int main()
{
    const double R=9.525;
    Physical17Parameters model;
    // Put the five-axis end-effector origin at Sensor Z=+200 mm.  This mimics
    // the saved FlipZ=true project convention: inside the reported 70..120 mm
    // profile range, Z=120 is physically nearer the head than Z=70.
    model[Physical17Parameters::HandEyeTz] = -200.0;
    const JointPose5d q{0,0,0,0,0};
    const Vec3d sphereM(0.0,0.0,-105.0); // -> sphere center S=(0,0,95)

    SphereStudyPathOptions o;
    o.sphereRadiusMm=R;
    o.opticalCenterSensorMm={0,0,0};
    o.sensorZMinMm=70.0;
    o.sensorZMaxMm=120.0;
    o.sensorXMinMm=-25.0;
    o.sensorXMaxMm=25.0;
    o.sensorXSafetyMarginMm=0.0;
    o.sensorZSafetyMarginMm=0.0;
    o.useTrapezoidalSensorField=true;
    o.sensorZCoordinateFlipped=true;
    o.sensorXSpanNearMm=42.0;
    o.sensorXSpanFarMm=58.0;
    o.requireEndEffectorFacingArc=true;
    o.minimumPlaneIntersectionMarginMm=0.01;
    o.minimumPredictedVisibleArcDeg=1.0;
    o.minimumPredictedVisibleArcLengthMm=0.01;
    o.visibleArcSamples=1440;

    const auto sim=SphereDistanceAngleStudy::buildScanSimulation(q,sphereM,R,model,o,1440);
    if(!sim.valid || !sim.optical.valid) return fail("FlipZ simulation should be valid");
    if(sim.laserFieldCornersSensorMm.size()!=4) return fail("trapezoid requires four corners");

    const auto& c=sim.laserFieldCornersSensorMm;
    // AF contract: corners are ordered by PHYSICAL depth.  With FlipZ=true the
    // near/small edge is at reported ZMax=120, and the far/large edge is at
    // reported ZMin=70.  This is the exact reversal missing in AE.
    if(!near(c[0].z,120.0)||!near(c[1].z,120.0)||
       !near(c[2].z,70.0)||!near(c[3].z,70.0))
        return fail("FlipZ near/far reported-Z mapping");
    if(!near(width(c[0],c[1]),42.0)||!near(width(c[3],c[2]),58.0))
        return fail("physical small-near / large-far trapezoid widths");

    const Vec3d endS=sim.optical.endEffectorOriginSensorMm;
    if(!near(endS.z,200.0)) return fail("end-effector origin in Sensor frame");
    const double nearDistance=std::abs(endS.z-c[0].z);
    const double farDistance=std::abs(endS.z-c[2].z);
    if(!(nearDistance < farDistance)) return fail("small edge must be closer to end/sensor side");

    if(sim.visibleArcSensorMm.empty()) return fail("visible end-effector-side arc missing");
    const Vec3d sphereS=sim.optical.sphereCenterSensorMm;
    const Vec3d towardEnd=sub3(endS,sphereS);
    double meanZ=0.0;
    for(const auto& p:sim.visibleArcSensorMm){
        if(!(dot3(sub3(p,sphereS),towardEnd)>0.0))
            return fail("visible arc escaped end-effector-facing hemisphere");

        // Physical depth fraction is reversed with FlipZ: Z=120 -> depth 0
        // (42 mm span), Z=70 -> depth 1 (58 mm span).
        const double tDepth=(120.0-p.z)/50.0;
        const double half=0.5*(42.0+tDepth*(58.0-42.0));
        if(p.z<70.0-1e-8||p.z>120.0+1e-8||p.x<-half-1e-8||p.x>half+1e-8)
            return fail("visible arc escaped FlipZ-aware physical trapezoid");
        meanZ += p.z;
    }
    meanZ/=static_cast<double>(sim.visibleArcSensorMm.size());
    if(!(meanZ > sphereS.z+0.2)) return fail("end-effector-side contour should face +reported-Z in this setup");

    // Also keep the non-flipped branch explicit: near=ZMin, far=ZMax.
    o.sensorZCoordinateFlipped=false;
    const auto nonFlip=SphereDistanceAngleStudy::buildScanSimulation(q,sphereM,R,model,o,720);
    if(nonFlip.laserFieldCornersSensorMm.size()!=4) return fail("non-flip trapezoid corners");
    const auto& n=nonFlip.laserFieldCornersSensorMm;
    if(!near(n[0].z,70.0)||!near(n[1].z,70.0)||
       !near(n[2].z,120.0)||!near(n[3].z,120.0))
        return fail("non-flip near/far mapping");
    if(!near(width(n[0],n[1]),42.0)||!near(width(n[3],n[2]),58.0))
        return fail("non-flip physical trapezoid widths");

    std::cout << "PASS V27.6.16AF FlipZ-aware physical trapezoid orientation\n";
    return 0;
}
