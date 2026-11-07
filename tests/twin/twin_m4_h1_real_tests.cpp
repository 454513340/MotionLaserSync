// Genuine H1 subset regression. Inputs were extracted WITHOUT interpolation or
// synthesis from user-provided SCAN v3 using the documented Archive v5 layout.
// Reference only: 100 low-rate frames are not a full optical calibration.
#include "twin/TwinM4Comparison.h"
#include "twin/TwinM4Calibration.h"
#include <cassert>
#include <algorithm>
#include <cstring>
#include <stdexcept>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
using namespace mls::twin;
namespace {
std::uint64_t readUnsigned(std::istream& in,int n){
    std::uint64_t u=0;
    for(int i=0;i<n;++i){const int b=in.get();if(b==EOF)throw std::runtime_error("Fixture truncated");u|=std::uint64_t(b)<<(8*i);}
    return u;
}
double readDouble(std::istream& in){
    const std::uint64_t b=readUnsigned(in,8);double x=0;
    static_assert(sizeof(double)==sizeof(b));std::memcpy(&x,&b,8);return x;
}
}
int main(int argc,char** argv){
    if(argc!=4)return 2;
    std::ifstream posefile(argv[2]);assert(posefile);
    std::map<std::uint64_t,VirtualProfilePose> poses;
    std::string line;
    while(std::getline(posefile,line)){
        if(line.empty()||line[0]=='#')continue;
        std::replace(line.begin(),line.end(),',',' ');
        std::istringstream ss(line);VirtualProfilePose pose;
        assert(bool(ss>>pose.sourceOrdinal>>pose.elapsedSeconds));
        for(auto& d:pose.sensorToMachine.m)assert(bool(ss>>d));
        poses.emplace(pose.sourceOrdinal,pose);
    }
    std::ifstream bin(argv[3],std::ios::binary);assert(bin);
    char magic[8]={};bin.read(magic,8);
    assert(std::string(magic,8)=="H1M4RAW1");
    const auto number=readUnsigned(bin,4);assert(number==poses.size());
    std::vector<M4CalibrationFrame> frames;
    for(std::uint64_t i=0;i<number;++i){
        const std::uint64_t ordinal=readUnsigned(bin,8);
        const auto n=readUnsigned(bin,4);
        M4CalibrationFrame frame;frame.pose=poses.at(ordinal);
        assert(n<=1280);
        frame.observed.reserve(std::size_t(n));
        for(std::uint64_t j=0;j<n;++j){
            M4MeasuredSample sample;
            sample.index=std::uint32_t(readUnsigned(bin,4));
            sample.sensorX=readDouble(bin);sample.sensorZ=readDouble(bin);
            frame.observed.push_back(sample);
        }
        frames.push_back(std::move(frame));
    }
    assert(frames.size()==100);
    TwinScene scene;std::string error;
    assert(scene.loadObjMesh(argv[1],&error));
    scene.setTruePose({44.3195,10.2369,-225.05,0,0,-89.2265});
    scene.setSupportPlane(true,-225.05,42,10,140,90);
    Sensor2950Geometry optics;
    const auto context=h1ReferenceM4Context(scene,optics);assert(context.holes.size()==4);
    for(double baseline:{-75.0,75.0}){
        optics.cameraOriginSensor.y=baseline;
        VirtualLaser2950 laser(scene,optics);
        auto ctx=h1ReferenceM4Context(scene,optics);
        TwinM4Accumulator acc(ctx);
        for(const auto& f:frames)
            assert(acc.addFrame(laser.simulate(f.pose),f.observed));
        const auto report=std::move(acc).finish();
        assert(report.ok()&&report.profilesCompared==100);
        std::uint64_t realFloors=0,simFloors=0;
        double simPositive=0.;int found=0;
        for(const auto& f:report.floor){
            realFloors+=f.realFloorPoints;simFloors+=f.simFloorPoints;
            if(f.simFloorPoints){simPositive+=f.simulatedPositiveRatio();++found;}
        }
        std::cout<<"real H1 baseline="<<baseline<<" hole F1="<<report.holeRegions.f1()
                 <<" floorBinIoU mean="<<report.fitScore()
                 <<" realFloor="<<realFloors<<" virtualFloor="<<simFloors
                 <<" predicted+Y="<<(found?simPositive/found:0.0)<<"\n";
        assert(report.holeRegions.compared()>100);
        assert(realFloors>0&&simFloors>0);
        assert((baseline<0?(simPositive/found<0.25):(simPositive/found>0.75)));
    }
    M4FitOptions fitOptions;
    fitOptions.minSensorY=35;fitOptions.maxSensorY=95;
    fitOptions.stepSensorY=10;fitOptions.maxFrames=60;
    const auto fit=TwinM4Calibration::fit(scene,optics,frames,fitOptions);
    if(!fit.ok())std::cerr<<fit.error<<"\n";
    assert(fit.ok());
    assert(fit.validationFrames>=2);
    if(!fit.holdoutSupportsBest)
        std::cout<<"M4 guard: training optimum is NOT accepted for automatic use: "
                 <<fit.validationNote<<"\n";
    std::cout<<"REAL H1 M4 fit: training="<<fit.trainingFrames
             <<" holdout="<<fit.validationFrames
             <<" bestTrainingY="<<fit.bestBaselineY<<"\n";
    for(const auto& c:fit.candidates)
        std::cout<<"  Y="<<c.cameraSensorY<<" train="<<c.trainingScore
                 <<" holdout="<<c.validationScore<<"\n";
    std::cout<<"REAL H1 100-profile M4 geometry/index smoke test PASSED\n";
}
