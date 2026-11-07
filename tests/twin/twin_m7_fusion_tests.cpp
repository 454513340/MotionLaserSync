#include "twin/TwinM7Fusion.h"
#include <cassert>
#include <cmath>
#include <iostream>
using namespace mls::twin;
static bool close(double a,double b){return std::abs(a-b)<1e-7;}
static void coverageTest(){
 M5Report m5;m5.requestedOptions.minHitsPerCell=2;
 m5.features.resize(2);m5.features[0].name="top";m5.features[1].name="wall";
 const std::uint32_t real[]={3,0,3,0,2,0};
 const std::uint16_t fid[]={0,0,0,1,1,1};
 for(int i=0;i<6;++i){M5Cell c;c.target=(i!=5);c.areaMm2=double(i+1);c.featureId=fid[i];c.realHits=real[i];m5.cells.push_back(c);}
 m5.targetAreaMm2=15.;
 M6Report m6;m6.segments.resize(2);m6.firstCoveredSegmentByCell={0,1,2,0,0,0};
 m6.reopenedSparseCellIds={2,4};m6.targetAreaMm2=15.;
 m6.newAreaMm2=5.;m6.initialObservedAreaMm2=1.;m6.finalObservedAreaMm2=6.;
 auto r=calculateM7Coverage(m5,m6,.95);
 assert(r.ok()&&r.areaConsistent);
 assert(close(r.measuredAreaMm2,1.));assert(close(r.simulatedNewAreaMm2,5.));
 assert(close(r.remainingAreaMm2,9.));assert(close(r.reopenedSparseAreaMm2,8.));
 assert(r.needsAdditionalView&&r.classifications[0]==M7CellClass::Measured);
 assert(r.classifications[2]==M7CellClass::SimulatedNew);
 assert(r.classifications[4]==M7CellClass::Missing);
 m6.firstCoveredSegmentByCell[1]=3; assert(!calculateM7Coverage(m5,m6).ok());
}
static void voxelTest(){
 M7VoxelSampler f({.25,1024,1,1});assert(f.valid());
 f.observe({1.01,2.02,3.01});f.observe({1.02,2.01,3.03});
 auto pts=f.finalize(M7Source::MeasuredM2,0);
 assert(pts.size()==1 && pts[0].hits==2 && close(pts[0].xyz.x,1.015));
 M7VoxelSampler bounded({.15,300,1,1});
 for(int seg=1;seg<=4;++seg)for(int i=0;i<10000;++i)
     bounded.observe({double(seg)*100+double(i)*.20,double(seg)*2,1});
 assert(bounded.retainedVoxelCount()<=300);
 assert(bounded.finalize(M7Source::SimulatedM6,4).size()<=300);
}
int main(){coverageTest();voxelTest();std::cout<<"M7 fusion tests PASS\n";}
