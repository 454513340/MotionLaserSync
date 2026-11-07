// V28.4 portable regression: geometric cell area, projection, bottom filtering,
// two separate observation channels, and independent rigid placement.
#include "twin/TwinM5Coverage.h"
#include <cassert>
#include <cmath>
#include <iostream>
#include <string>
#include <utility>
using namespace mls::twin;

TriangleMesh boxMesh(){
    TriangleMesh m;
    const Vec3 a{0,0,0},b{10,0,0},c{10,10,0},d{0,10,0};
    const Vec3 A{0,0,2},B{10,0,2},C{10,10,2},D{0,10,2};
    const auto push=[&](Vec3 x,Vec3 y,Vec3 z){m.triangles.push_back({x,y,z,static_cast<std::uint32_t>(m.triangles.size()+1)});};
    push(A,B,C);push(A,C,D);      // top normal +Z
    push(a,c,b);push(a,d,c);      // bottom normal -Z
    push(a,b,B);push(a,B,A);push(b,c,C);push(b,C,B);
    push(c,d,D);push(c,D,C);push(d,a,A);push(d,A,D);
    m.rebuildBounds();return m;
}
int main(){
    TwinScene scene;std::string error;
    assert(scene.setWorkpiece(boxMesh(),&error));
    const Pose6 pose{41,7,-225,0,0,65};
    scene.setTruePose(pose);scene.setEstimatedPose(pose);
    M5Options options;options.cellPitchMm=1.;options.maxSurfaceDistanceMm=.2;
    options.minHitsPerCell=2;options.excludeBottom=true;
    TwinM5CoverageBuilder g(scene,options);
    assert(g.ready());
    const Vec3 topA{3.4,2.7,2.};
    auto measuredMachine=scene.cadToMachineEstimate().point(topA);
    g.addRealMachine(measuredMachine);
    g.addRealMachine(measuredMachine);  // now 2 hits
    g.addVirtualMachine(measuredMachine);
    g.addVirtualMachine(measuredMachine);
    g.addRealCad({4.1,3.7,-0.0}); // underside must not become target
    g.addVirtualCad({900,900,2}); // out of tolerance
    g.addRealCad({4,6,50}); // out of tolerance
    auto r=std::move(g).finish();
    assert(r.ok());
    assert(r.realAccepted==2&&r.virtualAccepted==2);
    assert(r.realExcluded==1&&r.virtualNoAssociation==1&&r.realNoAssociation==1);
    assert(std::abs(r.targetAreaMm2-180.)<1e-6); // box surface 280 minus bottom 100
    assert(std::abs(r.excludedAreaMm2-100.)<1e-6);
    assert(r.realAreaMm2>0&&r.bothAreaMm2>0);
    assert(std::abs(r.realAreaMm2-r.bothAreaMm2)<1e-12);
    assert(std::abs(r.realAreaMm2+r.missingBothAreaMm2-r.targetAreaMm2)<1e-6);
    assert(std::abs(r.realAreaMm2-r.virtualAreaMm2)<1e-6);
    bool foundTop=false,foundBottom=false;
    for(const auto& f:r.features){
        if(f.kind==M5FeatureKind::Top){foundTop=true;assert(f.target);assert(f.realCoverage()>0);}
        if(f.kind==M5FeatureKind::BottomExcluded){foundBottom=true;assert(!f.target);}
    }
    assert(foundTop&&foundBottom);
    // Dynamic coarsening must bound allocation but conserve CAD surface area.
    options.cellPitchMm=.02;options.maxCells=2000;
    TwinM5CoverageBuilder small(scene,options);assert(small.ready());
    auto compact=std::move(small).finish();
    assert(compact.ok()&&compact.cells.size()<=2000);
    assert(compact.effectiveCellPitchMm>options.cellPitchMm);
    assert(std::abs(compact.targetAreaMm2-180.)<1e-5);
    // Every CAD cell centroid must map back to its own subdivided triangle.
    // This detects lost/downward sub-triangle cells and boundary off-by-ones.
    TwinM5CoverageBuilder density(scene,options);
    for(const auto& cell:compact.cells){
        if(!cell.target)continue;
        density.addRealCad(cell.cadCenter);
        density.addRealCad(cell.cadCenter);
    }
    const auto full=std::move(density).finish();
    assert(full.ok());
    assert(std::abs(full.realCoverage()-1.)<1e-9);
    assert(full.realNoAssociation==0);
    // No CAD input and invalid options must be rejected, never report 100%.
    TwinScene empty;
    TwinM5CoverageBuilder bad(empty,options);assert(!bad.ready());
    options.minHitsPerCell=0;
    TwinM5CoverageBuilder badOptions(scene,options);assert(!badOptions.ready());
    std::cout<<"M5 portable surface coverage tests PASS\n";
}
