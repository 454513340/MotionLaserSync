// Minimal portable smoke test for M3: synthetic X-only scan across a placed OBJ.
// Does not read SCAN (M2 GUI has that adapter), call any device API or plan G-code.
#include "twin/VirtualLaser2950.h"
#include <iomanip>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
using namespace mls::twin;
int main(int argc,char** argv){
    if(argc!=2 && argc!=3){std::cerr<<"usage: digital_twin_m3_cli <reference-mesh.obj> [measured-poses.csv]\n";return 2;}
    TwinScene scene;std::string error;
    if(!scene.loadObjMesh(argv[1],&error)){std::cerr<<error<<'\n';return 3;}
    // Placement matched to the initial H1 reference estimate.
    scene.setTruePose({44.3195,10.2369,-225.05,0,0,-89.2265});
    scene.setSupportPlane(true,-225.05,42,10,140,90);
    VirtualLaser2950 sensor(scene);
    std::vector<VirtualProfilePose> poses;
    for(int i=0;i<=40;++i){
        VirtualProfilePose p;
        p.sourceOrdinal=std::uint64_t(i+1);
        p.elapsedSeconds=i*.005;
        const double machineX=45.+double(i)*.75;
        // sensor X points to +MCS Y, sensor Y points to -MCS X.
        p.sensorToMachine.m={{0,-1,0,machineX,1,0,0,10,0,0,1,-315,0,0,0,1}};
        p.joints={{machineX,0,-315,0,0}};
        poses.push_back(p);
    }
    if(argc==3){
        poses.clear();std::ifstream csv(argv[2]);std::string line;
        if(!csv){std::cerr<<"Cannot open measured poses CSV\n";return 6;}
        while(std::getline(csv,line)){
            if(line.empty()||line[0]=='#')continue;
            for(char& c:line)if(c==',')c=' ';
            std::istringstream ss(line);VirtualProfilePose p;
            if(!(ss>>p.sourceOrdinal>>p.elapsedSeconds))return 7;
            for(double& v:p.sensorToMachine.m)if(!(ss>>v))return 8;
            poses.push_back(p);
        }
    }
    const auto result=sensor.simulateAll(poses);
    if(!result.error.empty()){std::cerr<<result.error<<'\n';return 4;}
    std::cout<<"Virtual sensor 2950 geometric X-scan test: frames="<<result.totalProfiles
             <<" valid="<<result.counts[0]<<" laserMiss="<<result.counts[1]
             <<" cameraOccluded="<<result.counts[2]<<" outsideField="<<result.counts[3]<<'\n';
    if(result.validCount()==0){std::cerr<<"Unexpected: no points on reference workpiece\n";return 5;}
    return 0;
}
