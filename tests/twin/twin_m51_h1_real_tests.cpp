// V28.5 H1 authentic SCAN 100-profile subset -> 0.30mm nominal chamfer
// diagnostic. No invented points, no STEP truth transform leak to planner.
#include "twin/TwinM51Chamfer.h"
#include "twin/VirtualLaser2950.h"
#include <cassert>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <algorithm>
#include <stdexcept>
using namespace mls::twin;
namespace {
std::uint64_t readU(std::istream& in,int n){
    std::uint64_t u=0;
    for(int i=0;i<n;++i){int c=in.get();if(c==EOF)throw std::runtime_error("H1 fixture truncated");u|=std::uint64_t(c)<<(8*i);}
    return u;
}
double readD(std::istream& in){auto u=readU(in,8);double d=0;std::memcpy(&d,&u,8);return d;}
}
int main(int argc,char** argv){
    assert(argc==4);
    TwinScene scene;std::string err;
    assert(scene.loadObjMesh(argv[1],&err));
    const Pose6 cadPose{44.3195,10.2369,-225.05,0,0,-89.2265};
    scene.setTruePose(cadPose);scene.setEstimatedPose(cadPose);
    TwinM51ChamferAnalyzer analyzer(scene);
    assert(analyzer.ready());
    const auto machineToCad=scene.cadToMachineEstimate().inverseRigid();
    std::ifstream posesFile(argv[2]);assert(posesFile);
    std::map<std::uint64_t,VirtualProfilePose> poses;
    for(std::string line;std::getline(posesFile,line);){
        if(line.empty()||line[0]=='#')continue;
        std::replace(line.begin(),line.end(),',',' ');
        std::istringstream stream(line);VirtualProfilePose p;
        assert(bool(stream>>p.sourceOrdinal>>p.elapsedSeconds));
        for(auto& v:p.sensorToMachine.m)assert(bool(stream>>v));
        poses.emplace(p.sourceOrdinal,p);
    }
    std::ifstream bin(argv[3],std::ios::binary);assert(bin);
    char magic[8]{};bin.read(magic,8);assert(std::string(magic,8)=="H1M4RAW1");
    const auto frames=readU(bin,4);assert(frames==100);
    Sensor2950Geometry optic;optic.cameraOriginSensor.y=75;
    VirtualLaser2950 virtualScanner(scene,optic);
    std::uint64_t realPoints=0,virtualPoints=0;
    for(std::uint64_t f=0;f<frames;++f){
        const auto ordinal=readU(bin,8);
        const auto n=readU(bin,4);
        const auto& pose=poses.at(ordinal);
        for(std::uint64_t j=0;j<n;++j){
            assert(readU(bin,4)<1280);
            const double x=readD(bin),z=readD(bin);
            analyzer.addRealCad(machineToCad.point(pose.sensorToMachine.point({x,0,z})));
            ++realPoints;
        }
        const auto sim=virtualScanner.simulate(pose);
        for(const auto& p:sim.samples){
            if(p.state!=VirtualSampleState::Valid||p.hitObject!=VirtualHitObject::Workpiece)continue;
            analyzer.addVirtualCad(machineToCad.point(pose.sensorToMachine.point({p.sensorX,0,p.sensorZ})));
            ++virtualPoints;
        }
    }
    const auto result=std::move(analyzer).finish();
    assert(result.ok());
    assert(result.realSeen==realPoints&&result.virtualSeen==virtualPoints);
    assert(result.realSeen>100000);
    assert(result.realAssociated[0]<=result.realAssociated[1]);
    assert(result.realAssociated[1]<=result.realAssociated[2]);
    assert(result.virtualAssociated[0]<=result.virtualAssociated[1]);
    for(const auto& s:result.surfaces)assert(s.meshAreaMm2>5.);
    std::cout<<"REAL H1 M5.1 100 frames; valid="<<realPoints
             <<" realAssociated(0.15,0.30,0.65)="<<result.realAssociated[0]
             <<","<<result.realAssociated[1]<<","<<result.realAssociated[2]
             <<" top residual median="<<result.realConeEquationMedianMm<<"\n";
}
