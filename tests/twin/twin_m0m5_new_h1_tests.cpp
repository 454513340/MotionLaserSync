#include "twin/TwinH1Geometry.h"
#include <algorithm>
#include "twin/TwinM5Coverage.h"
#include "twin/TwinM51Chamfer.h"
#include "twin/TwinM52Geometry.h"
#include "twin/TwinM4Comparison.h"
#include <cassert>
#include <cmath>
#include <iostream>
using namespace mls::twin;
int main(int argc,char** argv){
    assert(argc==3);
    TwinScene old,now;std::string error;
    assert(old.loadObjMesh(argv[1],&error));
    assert(now.loadObjMesh(argv[2],&error));
    const auto go=inferH1Geometry(old);
    const auto gn=inferH1Geometry(now);
    assert(go.valid && gn.valid);
    assert(std::abs(go.outerRadius-3.25)<0.02);
    assert(std::abs(go.chamferDepth-0.50)<0.02);
    assert(std::abs(gn.outerRadius-3.35)<0.02);
    assert(std::abs(gn.innerRadius-2.75)<0.02);
    assert(std::abs(gn.chamferDepth-0.60)<0.02);
    TwinM51ChamferAnalyzer chamfer(now);
    if(!chamfer.ready())std::cerr<<"new STEP chamfer: "<<chamfer.error()<<'\n';
    assert(chamfer.ready());
    const auto report=std::move(chamfer).finish();
    assert(std::abs(report.nominalOuterRadiusMm-3.35)<.02);
    assert(std::abs(report.nominalChamferDepthMm-.60)<.02);
    TwinM52GeometryAnalyzer diagnostic(now);
    if(!diagnostic.ready())std::cerr<<"new STEP M5.2: "<<diagnostic.error()<<'\n';
    assert(diagnostic.ready());
    // Different real measured points in ONE exposure are not independent exposures.
    M5Options opt;opt.cellPitchMm=.65;opt.minHitsPerCell=2;
    opt.minDistinctProfilesPerCell=2;
    TwinM5CoverageBuilder empty(now,opt);
    assert(empty.ready());
    const auto cells=std::move(empty).finish().cells;
    const auto it=std::find_if(cells.begin(),cells.end(),[](const M5Cell& c){return c.target;});
    assert(it!=cells.end());
    const auto p=it->cadCenter;
    TwinM5CoverageBuilder same(now,opt);
    same.addRealCad(p,100);same.addRealCad(p,100);
    auto s=std::move(same).finish();assert(s.ok());
    assert(s.realAreaMm2==0);
    TwinM5CoverageBuilder independent(now,opt);
    independent.addRealCad(p,100);independent.addRealCad(p,101);
    auto ok=std::move(independent).finish();assert(ok.ok());
    assert(ok.realAreaMm2>0);
    assert(ok.realAccepted==2);
    // M3/M4 must be allowed to use estimated CAD pose rather than inaccessible
    // digital-twin ground truth in physical data comparisons.
    now.setTruePose({999,999,999,0,0,0});
    now.setEstimatedPose({6.87,8.95,-235.26,0,0,-89.4});
    Sensor2950Geometry model;
    const auto estimate=h1ReferenceM4Context(now,model,true);
    const auto truth=h1ReferenceM4Context(now,model,false);
    assert(std::abs(estimate.cadToMachine.m[3]-6.87)<1e-9);
    assert(std::abs(truth.cadToMachine.m[3]-999)<1e-9);
    std::cout<<"new STEP geometry + distinct exposure + estimated pose tests passed\n";
}
