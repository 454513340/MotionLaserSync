// Deterministic synthetic H1 geometry; top plane and chamfer offsets must be
// identifiable separately and holdout bins must NOT take part in fit selection.
#include "twin/TwinM52Geometry.h"
#include <cassert>
#include <cmath>
#include <iostream>
using namespace mls::twin;
int main(int argc,char** argv){
    assert(argc==2);
    TwinScene scene;std::string error;
    assert(scene.loadObjMesh(argv[1],&error));
    TwinM52GeometryAnalyzer estimator(scene);
    if(!estimator.ready())std::cerr<<estimator.error()<<"\n";
    assert(estimator.ready());
    constexpr double pi=3.14159265358979323846;
    auto plane=[](double x,double y){return .20+.0006*x-.0003*y;};
    // Uniform candidate grid; analyzer samples only the safe imported top face.
    for(double y=-54.;y<40.;y+=.25)for(double x=-14.5;x<14.6;x+=.25){
        estimator.addRealCad({x,y,10.+plane(x,y)+.004*std::sin(3*x+y)});
        estimator.addVirtualCad({x,y,10.});
    }
    for(int angle=0;angle<360;angle+=5){
        if(angle>=240&&angle<295)continue; // incomplete sampling must not bias fitted center excessively
        const double a=(angle+2.5)*pi/180.;
        for(int layer=0;layer<4;++layer)for(int n=0;n<8;++n){
            const double u=(layer+(n+.5)/8.)/4.;
            const double r=3.25-.5*u+.045;
            const double x=.11+r*std::cos(a), y=-38.-.07+r*std::sin(a);
            const double z=10.-.5*u+plane(x,y)+.002*std::sin(13*a+u);
            estimator.addRealCad({x,y,z});
            // model itself matches nominal chamfer
            estimator.addVirtualCad({(3.25-.5*u)*std::cos(a),-38.+(3.25-.5*u)*std::sin(a),10.-.5*u});
        }
    }
    auto report=std::move(estimator).finish();
    if(!report.ok())std::cerr<<report.error<<"\n";
    assert(report.ok());
    const auto& r=report.real;
    assert(r.topCellsUsed>500);
    assert(r.conePointsUsed>250);
    assert(r.holdoutEvaluated);
    assert(std::abs(r.topInterceptMm-.20)<.014);
    assert(std::abs(r.topSlopeX-.0006)<.00035);
    assert(std::abs(r.topSlopeY+.0003)<.00035);
    assert(std::abs(r.candidateCenterShiftXmm-.11)<.045);
    assert(std::abs(r.candidateCenterShiftYmm+.07)<.045);
    assert(std::abs(r.coneInterceptMm-.045)<.055);
    assert(r.coneFitValidationMadMm<.06);
    assert(!r.canApplyAutomatically);
    assert(report.virtualFit.ok);
    assert(std::abs(report.virtualFit.topOffsetAtHoleMm)<.01);
    assert(std::abs(report.virtualFit.coneInterceptMm)<.02);
    std::cout<<"M5.2 synthetic H1 top/center/residual/holdout OK: z="<<r.topOffsetAtHoleMm
             <<" dx="<<r.candidateCenterShiftXmm<<" dy="<<r.candidateCenterShiftYmm
             <<" intercept="<<r.coneInterceptMm<<" holdoutMAD="<<r.coneFitValidationMadMm<<"\n";
}
