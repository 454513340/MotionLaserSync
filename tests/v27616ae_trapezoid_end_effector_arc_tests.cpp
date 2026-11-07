#include "calibration/SphereDistanceAngleStudy.h"

#include <cmath>
#include <iostream>
#include <string>

using namespace mls::calibration;

namespace {
int fail(const char* s){ std::cerr << "FAIL: " << s << "\n"; return 1; }
bool near(double a,double b,double tol){ return std::abs(a-b)<=tol; }
double dist2(const Vec3d& a,const Vec3d& b){
    const double dx=a.x-b.x,dy=a.y-b.y,dz=a.z-b.z;
    return dx*dx+dy*dy+dz*dz;
}
double dot3(const Vec3d& a,const Vec3d& b){ return a.x*b.x+a.y*b.y+a.z*b.z; }
Vec3d sub3(const Vec3d& a,const Vec3d& b){ return {a.x-b.x,a.y-b.y,a.z-b.z}; }
}

int main()
{
    const double R=9.525;
    Physical17Parameters model;
    // ^E T_S translation x=-200 mm => E origin is +200 mm on Sensor X.
    // This non-zero hand-eye case verifies that AE really uses the calibrated
    // kinematic end frame rather than assuming Sensor origin or machine Z.
    model[Physical17Parameters::HandEyeTx] = -200.0;
    const JointPose5d q{0,0,0,0,0};
    const Vec3d sphereM(-200.0,0.0,95.0); // maps to Sensor center (0,0,95)

    SphereStudyPathOptions o;
    o.sphereRadiusMm=R;
    // Deliberately place the geometry/reference point on the *opposite* side of
    // the sphere.  AE visibility must ignore this for side selection and use E.
    o.opticalCenterSensorMm={0.0,0.0,300.0};
    o.sensorZMinMm=70.0; o.sensorZMaxMm=120.0;
    o.sensorXMinMm=-25.0; o.sensorXMaxMm=25.0;
    o.useTrapezoidalSensorField=true;
    // AE behavior is retained as the explicit non-flipped convention branch.
    // AF adds a separate regression for the project's real FlipZ=true setup.
    o.sensorZCoordinateFlipped=false;
    o.sensorXSpanNearMm=42.0;
    o.sensorXSpanFarMm=58.0;
    o.sensorXSafetyMarginMm=0.0;
    o.sensorZSafetyMarginMm=0.0;
    o.requireEndEffectorFacingArc=true;
    o.minimumPlaneIntersectionMarginMm=0.01;
    o.minimumPredictedVisibleArcDeg=1.0;
    o.minimumPredictedVisibleArcLengthMm=0.01;
    o.visibleArcSamples=720;

    const auto sim=SphereDistanceAngleStudy::buildScanSimulation(q,sphereM,R,model,o,720);
    if(!sim.valid || !sim.optical.valid) return fail("AE simulation should be optically valid");
    if(sim.laserFieldCornersSensorMm.size()!=4) return fail("trapezoid needs four corners");

    // Non-flipped convention: physical near=reported Z=70, far=reported Z=120.
    const auto& c=sim.laserFieldCornersSensorMm;
    if(!near(c[0].z,70.0,1e-9)||!near(c[1].z,70.0,1e-9)||
       !near(c[2].z,120.0,1e-9)||!near(c[3].z,120.0,1e-9))
        return fail("trapezoid Z corners");
    if(!near(c[0].x,-21.0,1e-9)||!near(c[1].x,21.0,1e-9)||
       !near(c[2].x,29.0,1e-9)||!near(c[3].x,-29.0,1e-9))
        return fail("trapezoid X spans 42/58 mm");

    if(sim.visibleArcSensorMm.empty()) return fail("end-effector-side arc missing");
    const Vec3d sphereS=sim.optical.sphereCenterSensorMm;
    const Vec3d endS=sim.optical.endEffectorOriginSensorMm;
    if(!near(endS.x,200.0,1e-9)||!near(endS.y,0.0,1e-9)||!near(endS.z,0.0,1e-9))
        return fail("end-effector origin from inverse hand-eye");
    const Vec3d towardEnd=sub3(endS,sphereS);
    double meanZ=0.0;
    for(const auto& p:sim.visibleArcSensorMm){
        // All synthetic acquisition points must be on the hemisphere facing E.
        if(!(dot3(sub3(p,sphereS),towardEnd)>0.0))
            return fail("visible arc contains a far-side sphere point");
        meanZ += p.z;
        // And every point must stay inside the trapezoid at its own Z.
        const double t=(p.z-70.0)/50.0;
        const double half=0.5*(42.0+t*(58.0-42.0));
        if(p.x < -half-1e-8 || p.x > half+1e-8 || p.z<70.0-1e-8 || p.z>120.0+1e-8)
            return fail("visible arc escaped trapezoidal field");
    }
    meanZ/=static_cast<double>(sim.visibleArcSensorMm.size());
    // E has both +X and -Z direction relative to the sphere.  A negative mean
    // Z shift confirms the optical reference at Z=300 did not choose the
    // opposite/far contour; the dot-product assertion above checks full E side.
    if(!(meanZ < sphereS.z-0.2)) return fail("arc followed optical reference instead of end-effector side");

    // Near-side points must be closer to E than the sphere center itself on
    // average for this symmetric great-circle setup.
    double meanD2=0.0;
    for(const auto& p:sim.visibleArcSensorMm) meanD2 += dist2(p,endS);
    meanD2/=static_cast<double>(sim.visibleArcSensorMm.size());
    if(!(meanD2 < dist2(sphereS,endS))) return fail("selected contour is not the near end-effector side");

    std::cout << "PASS V27.6.16AE trapezoid FOV + end-effector-side sphere contour\n";
    return 0;
}
