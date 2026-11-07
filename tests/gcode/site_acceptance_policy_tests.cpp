#include "safety/SiteAcceptancePolicy.h"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

using namespace mls;

static twin::TriangleMesh makeMesh() {
    twin::TriangleMesh mesh;
    mesh.triangles.push_back({{0,0,0},{120,0,0},{0,80,0},1});
    mesh.triangles.push_back({{0,0,50},{120,0,50},{0,80,50},2});
    mesh.rebuildBounds();
    return mesh;
}

int main() {
    auto mesh=makeMesh();
    twin::Pose6 expected{25.0,10.0,50.0,0.0,0.0,7.0};
    safety::SiteAcceptanceInput accepted;
    accepted.independentlyMeasuredCadToMcs=expected;
    accepted.measuredMinimumFixtureClearanceMm=3.0;
    accepted.independentMeasurementConfirmed=true;
    accepted.partAndFixturesSecured=true;
    accepted.entireSweptEnvelopeInspected=true;
    accepted.guardingAndEStopVerified=true;
    accepted.measurementMethod="CMM, 3 datum points, file CMM42";
    accepted.operatorIdentification="operator01";
    const auto ok=safety::evaluateSiteAcceptance(accepted,mesh,expected,1.0);
    assert(ok.accepted);
    assert(ok.worstCornerPositionErrorMm<1.e-8);
    assert(ok.allowedPositionErrorMm==0.5);
    assert(ok.minimumRequiredFixtureClearanceMm==2.0);

    auto input=accepted;
    input.independentlyMeasuredCadToMcs.x+=0.8;
    assert(!safety::evaluateSiteAcceptance(input,mesh,expected,1.0).accepted);
    input=accepted;
    input.independentlyMeasuredCadToMcs.rzDeg+=1.0;
    assert(!safety::evaluateSiteAcceptance(input,mesh,expected,1.0).accepted);
    input=accepted;
    input.measuredMinimumFixtureClearanceMm=1.9;
    assert(!safety::evaluateSiteAcceptance(input,mesh,expected,1.0).accepted);
    input=accepted;
    input.independentMeasurementConfirmed=false;
    assert(!safety::evaluateSiteAcceptance(input,mesh,expected,1.0).accepted);
    input=accepted;
    input.entireSweptEnvelopeInspected=false;
    assert(!safety::evaluateSiteAcceptance(input,mesh,expected,1.0).accepted);
    input=accepted;
    input.operatorIdentification.clear();
    assert(!safety::evaluateSiteAcceptance(input,mesh,expected,1.0).accepted);
    input=accepted;
    input.measurementMethod="CMM";
    assert(!safety::evaluateSiteAcceptance(input,mesh,expected,1.0).accepted);
    input=accepted;
    input.independentlyMeasuredCadToMcs.x=std::numeric_limits<double>::quiet_NaN();
    assert(!safety::evaluateSiteAcceptance(input,mesh,expected,1.0).accepted);
    assert(!safety::evaluateSiteAcceptance(accepted,mesh,expected,0.01).accepted);
    std::cout<<"Site acceptance: real pass path and all rejection scenarios passed\n";
}
