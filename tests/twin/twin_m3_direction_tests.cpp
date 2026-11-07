// V28.2 regression for M3 scan direction vs measured H1 orientation.
// This uses only the bundled low-rate H1 pose subset and the reference OBJ;
// no hardware calls, STEP/Qt/VTK requirement, or unrecorded fabrication of points.
#include "twin/VirtualLaser2950.h"
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
using namespace mls::twin;

struct FloorObservation {
    std::uint64_t positiveY=0;
    std::uint64_t negativeY=0;
    std::uint64_t blocked=0;
    std::uint64_t floor() const {return positiveY+negativeY;}
    double fractionPositive() const {
        return floor() ? double(positiveY)/double(floor()) : 0.0;
    }
};

static FloorObservation countCounterboreFloor(
    const TwinScene& scene,
    const std::vector<VirtualProfilePose>& poses,
    double cameraSensorY) {
    Sensor2950Geometry optics;
    optics.cameraOriginSensor.y=cameraSensorY;
    assert(optics.validate());
    VirtualLaser2950 scanner(scene,optics);
    const Transform machineToCad=scene.cadToMachineTruth().inverseRigid();
    // The four stepped counterbores in the supplied STEP reference geometry.
    constexpr std::array<Vec3,4> holes{{
        {-7.5,17.5,5.4},{7.5,17.5,5.4},
        {-7.5,32.5,5.4},{7.5,32.5,5.4}
    }};
    FloorObservation out;
    for(const auto& pose:poses){
        const VirtualProfile profile=scanner.simulate(pose);
        assert(profile.samples.size()==1280);
        for(const auto& sample:profile.samples){
            if(sample.state==VirtualSampleState::CameraOccluded){
                ++out.blocked;
                continue;
            }
            if(sample.state!=VirtualSampleState::Valid)continue;
            const Vec3 pMachine=pose.sensorToMachine.point({sample.sensorX,0,sample.sensorZ});
            const Vec3 pCad=machineToCad.point(pMachine);
            if(std::abs(pCad.z-5.4)>0.35)continue;
            for(const auto& hole:holes){
                const double r=std::hypot(pCad.x-hole.x,pCad.y-hole.y);
                if(r>2.5 && r<3.78){
                    if(pCad.y>hole.y)++out.positiveY;
                    else ++out.negativeY;
                    break;
                }
            }
        }
    }
    return out;
}

int main(int argc,char** argv){
    if(argc!=3){std::cerr<<"usage: twin_m3_direction_tests OBJ H1_SUBSET_CSV\n";return 2;}
    TwinScene scene;std::string error;
    if(!scene.loadObjMesh(argv[1],&error)){std::cerr<<error<<'\n';return 3;}
    scene.setTruePose({44.3195,10.2369,-225.05,0,0,-89.2265});
    scene.setSupportPlane(true,-225.05,42,10,140,90);
    std::ifstream in(argv[2]);
    if(!in){std::cerr<<"Unable to open H1 test poses\n";return 4;}
    std::vector<VirtualProfilePose> poses;
    std::string line;
    while(std::getline(in,line)){
        if(line.empty()||line[0]=='#')continue;
        std::replace(line.begin(),line.end(),',',' ');
        std::istringstream ss(line);
        VirtualProfilePose p;
        if(!(ss>>p.sourceOrdinal>>p.elapsedSeconds))return 5;
        for(auto& v:p.sensorToMachine.m)if(!(ss>>v))return 6;
        poses.push_back(p);
    }
    assert(poses.size()>=50);
    // H1 sampling index convention is opposite to increasing Sensor-X.
    Sensor2950Geometry optics;
    assert(optics.cameraOriginSensor.y>0);
    assert(optics.sampleXAtZ(0,105.342)>optics.sampleXAtZ(1279,105.342));
    const auto correct=countCounterboreFloor(scene,poses,+75.0);
    const auto reversed=countCounterboreFloor(scene,poses,-75.0);
    std::cout<<"H1 poses="<<poses.size()
             <<" +75 floor="<<correct.floor()
             <<" positiveFraction="<<correct.fractionPositive()
             <<" -75 floor="<<reversed.floor()
             <<" positiveFraction="<<reversed.fractionPositive()<<'\n';
    // These wide tolerances test the SIDE, not calibrated numerical accuracy.
    assert(correct.floor()>50 && reversed.floor()>50);
    assert(correct.fractionPositive()>0.80);
    assert(reversed.fractionPositive()<0.20);
    std::cout<<"M3 H1 direction/index regression PASSED\n";
}
