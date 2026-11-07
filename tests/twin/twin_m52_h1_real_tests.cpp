// Regression: authentic H1 100-Profile fixture + exact hardware-latched poses.
// No fabricated measurements; the nominal placement is only an initial estimate.
#include "twin/TwinM52Geometry.h"
#include "twin/VirtualLaser2950.h"
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
using namespace mls::twin;
namespace {
std::uint64_t readUnsigned(std::istream& f,int n){
    std::uint64_t v=0;
    for(int i=0;i<n;++i){int c=f.get();if(c==EOF)throw std::runtime_error("Truncated H1 fixture");v|=std::uint64_t(c)<<(i*8);}
    return v;
}
double readDouble(std::istream& f){auto bits=readUnsigned(f,8);double v;std::memcpy(&v,&bits,8);return v;}
}
int main(int argc,char** argv){
    assert(argc==4);
    TwinScene scene;std::string err;
    assert(scene.loadObjMesh(argv[1],&err));
    const Pose6 placement{44.3195,10.2369,-225.05,0,0,-89.2265};
    scene.setEstimatedPose(placement);scene.setTruePose(placement);
    TwinM52GeometryAnalyzer analyzer(scene);
    if(!analyzer.ready())std::cerr<<analyzer.error()<<"\n";
    assert(analyzer.ready());
    const auto toCad=scene.cadToMachineEstimate().inverseRigid();
    std::ifstream posesFile(argv[2]);assert(posesFile);
    std::map<std::uint64_t,VirtualProfilePose> poses;
    for(std::string line;std::getline(posesFile,line);){
        if(line.empty()||line[0]=='#')continue;
        std::replace(line.begin(),line.end(),',',' ');
        std::istringstream in(line);VirtualProfilePose pose;
        assert(bool(in>>pose.sourceOrdinal>>pose.elapsedSeconds));
        for(double& v:pose.sensorToMachine.m)assert(bool(in>>v));
        poses.emplace(pose.sourceOrdinal,pose);
    }
    Sensor2950Geometry geometry;geometry.cameraOriginSensor.y=75.;
    VirtualLaser2950 scanner(scene,geometry);
    std::ifstream f(argv[3],std::ios::binary);assert(f);
    char magic[8]{};f.read(magic,8);assert(std::string(magic,8)=="H1M4RAW1");
    const auto count=readUnsigned(f,4);assert(count==100);
    for(std::uint64_t i=0;i<count;++i){
        const auto ordinal=readUnsigned(f,8);
        const auto n=readUnsigned(f,4);
        const auto& pose=poses.at(ordinal);
        for(std::uint64_t j=0;j<n;++j){
            assert(readUnsigned(f,4)<1280);
            const double x=readDouble(f),z=readDouble(f);
            analyzer.addRealCad(toCad.point(pose.sensorToMachine.point({x,0.,z})));
        }
        const auto sim=scanner.simulate(pose);
        for(const auto& p:sim.samples){
            if(p.state==VirtualSampleState::Valid&&p.hitObject==VirtualHitObject::Workpiece)
                analyzer.addVirtualCad(toCad.point(pose.sensorToMachine.point({p.sensorX,0.,p.sensorZ})));
        }
    }
    auto report=std::move(analyzer).finish();
    // Small subsample may not resolve a 5.5mm hole at every angle. A clean
    // structured "insufficient angular support" outcome is also valid here.
    assert(report.real.pointsSeen>100000);
    assert(report.real.topCellsUsed>350);
    assert(report.virtualFit.pointsSeen>10000);
    // Intentional undercoverage fixture: 100 sampled frames do not span all
    // 72 chamfer sectors. The solver MUST refuse an extrapolated center shift.
    assert(!report.ok());
    assert(!report.real.ok);
    assert(report.real.error.find("insufficient angular coverage")!=std::string::npos);
    assert(!report.real.canApplyAutomatically);
    std::cout<<"M5.2 H1-100 authentic profiles: real/top="<<report.real.pointsSeen<<"/"<<report.real.topCellsUsed
             <<" cone="<<report.real.conePointsUsed<<" sectors="<<report.real.angularBinsUsed
             <<" status="<<(report.ok()?"OK":"insufficient coverage")
             <<" note="<<report.error<<"\n";
}
