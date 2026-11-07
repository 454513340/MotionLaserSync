// V28.4: true H1 indexed-profile fixture exercises CAD association against
// the user's original 100 (unaltered) SCAN v3 frames, plus M3 replay optics.
#include "twin/TwinM5Coverage.h"
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
std::uint64_t readUnsigned(std::istream& in,int n){
    std::uint64_t value=0;
    for(int i=0;i<n;++i){const int c=in.get();if(c==EOF)throw std::runtime_error("Truncated H1 fixture");value|=std::uint64_t(c)<<(8*i);}
    return value;
}
double readDouble(std::istream& in){
    const auto raw=readUnsigned(in,8);double v=0;std::memcpy(&v,&raw,sizeof(v));return v;
}
}
int main(int argc,char** argv){
    if(argc!=4)return 2;
    TwinScene scene;std::string error;
    assert(scene.loadObjMesh(argv[1],&error));
    scene.setTruePose({44.3195,10.2369,-225.05,0,0,-89.2265});
    scene.setEstimatedPose(scene.truePose());
    scene.setSupportPlane(true,-225.05,42,10,140,90);
    M5Options opt;opt.cellPitchMm=.65;opt.maxSurfaceDistanceMm=.70;
    TwinM5CoverageBuilder builder(scene,opt);
    if(!builder.ready())std::cerr<<builder.error()<<"\n";
    assert(builder.ready());
    std::ifstream in(argv[2]);assert(in);
    std::map<std::uint64_t,VirtualProfilePose> poses;
    std::string line;
    while(std::getline(in,line)){
        if(line.empty()||line[0]=='#')continue;
        std::replace(line.begin(),line.end(),',',' ');
        std::istringstream ss(line);
        VirtualProfilePose p;
        assert(bool(ss>>p.sourceOrdinal>>p.elapsedSeconds));
        for(auto& x:p.sensorToMachine.m)assert(bool(ss>>x));
        poses.emplace(p.sourceOrdinal,p);
    }
    assert(poses.size()==100);
    std::ifstream binary(argv[3],std::ios::binary);assert(binary);
    char magic[8]{};binary.read(magic,8);assert(std::string(magic,8)=="H1M4RAW1");
    const auto frames=readUnsigned(binary,4);assert(frames==100);
    Sensor2950Geometry optics;optics.cameraOriginSensor.y=75;
    VirtualLaser2950 sensor(scene,optics);
    std::uint64_t realSamples=0,virtualSamples=0;
    for(std::uint64_t f=0;f<frames;++f){
        const auto ordinal=readUnsigned(binary,8);
        const auto points=readUnsigned(binary,4);
        const auto& pose=poses.at(ordinal);
        for(std::uint64_t i=0;i<points;++i){
            const auto idx=readUnsigned(binary,4);
            const auto x=readDouble(binary);
            const auto z=readDouble(binary);
            assert(idx<1280);
            builder.addRealMachine(pose.sensorToMachine.point({x,0,z}));
            ++realSamples;
        }
        const auto predicted=sensor.simulate(pose);
        for(const auto& p:predicted.samples){
            if(p.state!=VirtualSampleState::Valid || p.hitObject!=VirtualHitObject::Workpiece)continue;
            builder.addVirtualMachine(pose.sensorToMachine.point({p.sensorX,0,p.sensorZ}));
            ++virtualSamples;
        }
    }
    auto r=std::move(builder).finish();
    assert(r.ok() &&r.h1FeatureTemplate);
    assert(r.cells.size()<=opt.maxCells);
    assert(r.realPointsSeen==realSamples);
    assert(r.virtualPointsSeen==virtualSamples);
    assert(realSamples>=100000&&virtualSamples>10000);
    assert(r.targetAreaMm2>1000);
    assert(r.realCoverage()>0 && r.realCoverage()<1);
    assert(r.virtualCoverage()>0 && r.virtualCoverage()<1);
    bool foundLanding=false;
    for(const auto& f:r.features){
        if(f.kind==M5FeatureKind::CounterboreFloor&&f.target&&f.totalAreaMm2>0)
            foundLanding=true;
    }
    assert(foundLanding);
    int landingCount=0;
    for(const auto& feature:r.features){
        if(feature.kind==M5FeatureKind::CounterboreFloor&&feature.totalAreaMm2>0)++landingCount;
        std::cout<<"  "<<feature.name<<" area="<<feature.totalAreaMm2
                 <<" realCover="<<feature.realCoverage()*100<<"%\n";
    }
    assert(landingCount==4);
    const double conserved=r.bothAreaMm2+r.realOnlyAreaMm2+
                           r.virtualOnlyAreaMm2+r.missingBothAreaMm2;
    assert(std::abs(conserved-r.targetAreaMm2)<1.e-4);
    std::cout<<"M5 real H1 profiles="<<frames<<" real="<<realSamples
             <<" virtual="<<virtualSamples<<" cells="<<r.cells.size()
             <<" realCoverage="<<r.realCoverage()*100
             <<" virtualCoverage="<<r.virtualCoverage()*100<<"%\n";
}
