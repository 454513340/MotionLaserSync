#include "calibration/SphereArcQualityGate.h"
#include "calibration/SphereDistanceAngleStudy.h"
#include <cmath>
#include <array>
#include <limits>
#include <cstddef>
#include <iostream>
#include <vector>

int main() {
    using namespace mls::calibration;
    const double pi=3.14159265358979323846;
    // The same machine-to-sensor ROI-anchor helper is used by imported .scan
    // and live H1/H3 dynamic/hold-out profiles; exercise rotation + translation.
    const std::array<double,16> sensorToMachine={
        0.0,0.0,1.0,10.0,
        0.0,1.0,0.0,20.0,
        -1.0,0.0,0.0,30.0,
        0.0,0.0,0.0,1.0};
    double ex=0,ez=0;
    // Sensor position (2, -3, 85) -> machine position (95, 17, 28).
    if(!projectSphereCenterToSensorArcXZ(sensorToMachine,95.0,17.0,28.0,ex,ez)
       ||std::abs(ex-2.0)>1e-12||std::abs(ez-85.0)>1e-12){
        std::cerr << "FAIL machine-to-sensor projection: " << ex << "," << ez << "\n";
        return 6;
    }
    if(projectSphereCenterToSensorArcXZ(sensorToMachine,
                                        std::numeric_limits<double>::infinity(),17.0,28.0,ex,ez)){
        std::cerr << "FAIL accepted nonfinite sphere center\n";return 7;
    }
    std::vector<SphereExposurePoint> clean;
    for(int i=0;i<180;++i) {
        const double a=(-55.0+150.0*i/179.0)*pi/180;
        const double r=9.2 + 0.0012*std::sin(13.0*i);
        clean.push_back({0.2+r*std::cos(a),88.0+r*std::sin(a),i});
    }
    const auto fit=SphereExposureSweep::fitArc(clean,0.2,88.0,9.525);
    if(!qualifySphereArc(fit,SphereTwoStageQuality{},9.525)) {
        std::cerr<<"FAIL valid arc; span="<<fit.arcSpanDeg<<" rms="<<fit.rms<<"\n"; return 1;
    }
    std::vector<SphereExposurePoint> shortArc;
    for(int i=0;i<180;++i) {
        const double a=(0.0+55.0*i/179.0)*pi/180;
        shortArc.push_back({0.2+9.2*std::cos(a),88.0+9.2*std::sin(a),i});
    }
    if(qualifySphereArc(SphereExposureSweep::fitArc(shortArc,0.2,88.0,9.525),
                        SphereTwoStageQuality{},9.525)) {
        std::cerr<<"FAIL insufficient arc span accepted\n"; return 2;
    }
    auto bad=fit;bad.rms=0.019;
    if(qualifySphereArc(bad,SphereTwoStageQuality{},9.525)) {
        std::cerr<<"FAIL high RMS arc accepted\n"; return 3;
    }
    bad=fit;bad.inliers=10;
    if(qualifySphereArc(bad,SphereTwoStageQuality{},9.525)) {
        std::cerr<<"FAIL low-inlier arc accepted\n";return 4;
    }
    std::vector<Vec3d> cloud;
    const Vec3d center{10,20,-195};
    for(int i=0;i<120;++i) for(int j=0;j<90;++j){
        double th=pi*(i+0.5)/120.0, ph=2*pi*j/90.0;
        double rad=9.525+0.002*std::sin(i*19+j*7);
        cloud.push_back({center.x+rad*std::sin(th)*std::cos(ph),
                         center.y+rad*std::sin(th)*std::sin(ph),
                         center.z+rad*std::cos(th)});
    }
    SphereCloudRansacOptions opt;
    opt.expectedRadiusMm=9.525;opt.inlierThresholdMm=0.020;
    const auto sphere=SphereDistanceAngleStudy::fitMachineSphereRansac(cloud,opt);
    if(!sphere.success || std::abs(2*sphere.radiusMm-19.05)>0.003) {
        std::cerr<<"FAIL 3D sphere after profile quality filter\n";return 5;
    }
    std::cout<<"PASS circle quality gates + independent 3D RANSAC, diameter="
             <<(2*sphere.radiusMm)<<", accepted arc span="<<fit.arcSpanDeg<<"\n";
    return 0;
}
