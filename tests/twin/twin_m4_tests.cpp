#include "twin/TwinM4Comparison.h"
#include "twin/TwinM4Calibration.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
using namespace mls::twin;

static void testSparseMaskAndResidual(){
    M4Context ctx;ctx.optical.sampleCount=8;
    ctx.optical.cameraOriginSensor.y=75;
    TwinM4Accumulator accumulator(ctx);
    VirtualProfile v;
    v.pose.sourceOrdinal=100;
    v.samples.resize(8);
    v.samples[0]={1.0f,100.0f,VirtualSampleState::Valid};
    v.samples[1]={2.0f,101.0f,VirtualSampleState::Valid};
    v.samples[2].state=VirtualSampleState::CameraOccluded;
    v.samples[3].state=VirtualSampleState::LaserMiss;
    std::vector<M4MeasuredSample> real{{0,1.125,100.5},{2,5,100}};
    assert(accumulator.addFrame(v,real));
    auto report=std::move(accumulator).finish();
    assert(report.ok());
    assert(report.global.truePositive==1);
    assert(report.global.falsePositive==1);
    assert(report.global.falseNegative==1);
    assert(report.global.trueNegative==5);
    assert(report.global.pairedSamples==1);
    assert(std::abs(report.global.meanAbsX()-.125)<1e-7);
    assert(std::abs(report.global.meanAbsZ()-.5)<1e-7);
    assert(std::abs(report.global.iou()-1./3.)<1e-9);
    assert(report.global.predictedStates[std::size_t(VirtualSampleState::CameraOccluded)]==1);
    assert(report.global.predictedWhereRealMissing[std::size_t(VirtualSampleState::LaserMiss)]==5);
    assert(report.global.predictedWhereRealMissing[std::size_t(VirtualSampleState::CameraOccluded)]==0);
    assert(report.perFrame[0].sourceOrdinal==100);
}
static void testDuplicateDetection(){
    M4Context ctx;ctx.optical.sampleCount=8;
    TwinM4Accumulator acc(ctx);
    VirtualProfile v;v.samples.resize(8);
    assert(!acc.addFrame(v,{{1,1,90},{1,2,90}}));
    auto r=std::move(acc).finish();
    assert(!r.ok() && r.rejectedIndexedPoints==1);
}
static void testFloorBins(){
    M4FloorCoverage floor;
    floor.realCounts[0]=10;floor.simulatedCounts[0]=2;
    floor.realCounts[1]=7;floor.simulatedCounts[2]=3;
    assert(floor.realOccupied()==2);
    assert(floor.simulatedOccupied()==2);
    assert(std::abs(floor.intersectionOverUnion()-1./3.)<1e-9);
    floor.realPositiveY=5;floor.realNegativeY=3;floor.realFloorPoints=8;
    assert(std::abs(floor.realPositiveRatio()-.625)<1e-10);
}
static void testH1BaselineFit(const std::string& mesh,const std::string& poses){
    TwinScene scene;std::string error;
    assert(scene.loadObjMesh(mesh,&error));
    scene.setTruePose({44.3195,10.2369,-225.05,0,0,-89.2265});
    scene.setSupportPlane(true,-225.05,42,10,140,90);
    std::ifstream in(poses);assert(in);
    std::string line;
    std::vector<M4CalibrationFrame> inputs;
    Sensor2950Geometry correct;
    correct.cameraOriginSensor.y=+75;
    VirtualLaser2950 scanner(scene,correct);
    while(std::getline(in,line)){
        if(line.empty()||line[0]=='#')continue;
        std::replace(line.begin(),line.end(),',',' ');
        std::istringstream iss(line);
        M4CalibrationFrame f;
        if(!(iss>>f.pose.sourceOrdinal>>f.pose.elapsedSeconds))break;
        for(double& e:f.pose.sensorToMachine.m)assert(bool(iss>>e));
        auto truth=scanner.simulate(f.pose);
        for(std::size_t i=0;i<truth.samples.size();++i){
            const auto& s=truth.samples[i];
            if(s.state==VirtualSampleState::Valid)
                f.observed.push_back({std::uint32_t(i),s.sensorX,s.sensorZ});
        }
        inputs.push_back(std::move(f));
    }
    assert(inputs.size()>=50);
    M4FitOptions options;options.minSensorY=-75;options.maxSensorY=75;
    options.stepSensorY=75;options.maxFrames=25;
    const auto report=TwinM4Calibration::fit(scene,correct,inputs,options);
    std::cout<<"M4 train="<<report.trainingFrames<<" holdout="<<report.validationFrames
             <<" chosen="<<report.bestBaselineY<<"\n";
    if(!report.ok())std::cerr<<report.error<<"\n";
    assert(report.ok());
    assert(report.bestBaselineY==75);
    assert(report.candidates.size()==3);
    assert(report.candidates.back().training.holeRegions.f1()>.99);
    assert(report.candidates.back().validation.holeRegions.f1()>.99);
    assert(report.candidates.back().trainingScore>report.candidates.front().trainingScore);
    assert(report.candidates.back().validationScore>report.candidates.front().validationScore);
    // Input with unknown generic geometry must not silently apply H1 ROI.
    TwinScene generic;
    TriangleMesh tri;
    tri.triangles.push_back({{0,0,0},{10,0,0},{0,10,0},1});
    assert(generic.setWorkpiece(std::move(tri)));
    const auto invalid=TwinM4Calibration::fit(generic,correct,inputs,options);
    assert(!invalid.ok());
}
int main(int argc,char** argv){
    testSparseMaskAndResidual();testDuplicateDetection();testFloorBins();
    if(argc==3)testH1BaselineFit(argv[1],argv[2]);
    std::cout<<"M4 comparison/calibration tests passed\n";
}
