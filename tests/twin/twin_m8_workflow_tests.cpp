#include "twin/TwinM8Workflow.h"
#include <cassert>
#include <cmath>
#include <iostream>
using namespace mls::twin;
static void planTest(){
 M8WorkflowOptions o;
 auto p=makeM8Plan(o);assert(p.ok()&&p.stages.size()==7);
 assert(p.stages.front()==M8Stage::VirtualM3);
 assert(p.stages.back()==M8Stage::FusionM7);
 M8InputState s;auto no=inspectM8Preflight(p,o,s);assert(!no.permitted);
 s.cadPlaced=true;s.realM2Loaded=true;
 assert(inspectM8Preflight(p,o,s).permitted);
 s.m2SourceIsSimulated=true;assert(!inspectM8Preflight(p,o,s).permitted);
 s.m2SourceIsSimulated=false;s.h1SpecificGeometry=false;
 assert(!inspectM8Preflight(p,o,s).permitted);
 o.inspectChamferM51=o.inspectGeometryM52=false;
 p=makeM8Plan(o);assert(inspectM8Preflight(p,o,s).permitted);
 o.simulateM3=false;assert(!inspectM8Preflight(makeM8Plan(o),o,s).permitted);
 s.hasM3=true;assert(inspectM8Preflight(makeM8Plan(o),o,s).permitted);
 o.coverageGoal=1.01;assert(!makeM8Plan(o).ok());
}
static void acceptanceTest(){
 M5Report m5;m5.requestedOptions.minHitsPerCell=2;
 m5.features.resize(2);m5.features[0].name="Top";m5.features[1].name="Hole";
 for(unsigned int i=0;i<4;++i){
    M5Cell c;c.areaMm2=10;c.target=true;c.featureId=(i<2?0:1);
    c.realHits=(i==0?5:0);m5.cells.push_back(c);
 }
 m5.targetAreaMm2=40.;
 M6Report m6;m6.firstCoveredSegmentByCell={0,1,1,0};
 m6.segments.resize(1);m6.fullProfiles=100;m6.targetAreaMm2=40;
 m6.initialObservedAreaMm2=10;m6.newAreaMm2=20;m6.finalObservedAreaMm2=30;
 auto m7=calculateM7Coverage(m5,m6,.95);
 assert(m7.ok()&&m7.areaConsistent&&std::abs(m7.remainingAreaMm2-10)<1e-8);
 auto a=evaluateM8Acceptance(m5,m6,m7,.95);
 assert(a.ok()&&!a.predictedTargetReached&&a.requiresAdditionalView);
 assert(a.priorities.size()==1&&a.priorities[0].feature=="Hole");
 assert(!a.hardwareAuthorized&&!a.opticalMultiAngleValidated);
 assert(std::abs(a.predictedFraction-.75)<1e-10);
 assert(evaluateM8Acceptance(m5,m6,m7,.70).predictedTargetReached);
 m6.newAreaMm2=19;assert(!evaluateM8Acceptance(m5,m6,m7,.95).ok());
}
int main(){planTest();acceptanceTest();std::cout<<"M8 workflow/acceptance PASS\n";}
