// Offline-only replay/verification for a given machine-joint NC file.
// It deliberately does not connect to the real controller and cannot perform
// FCL workpiece collision checks without a separately registered STEP scene.
#include "twin/MultiPoseNcSimulation.h"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <string>

int main(int argc,char** argv){
    if(argc!=2){
        std::cerr<<"Usage: diagnose_nc_offline <file.nc>\n";
        return 2;
    }
    std::ifstream input(argv[1],std::ios::binary);
    if(!input){std::cerr<<"Unable to read input file\n";return 3;}
    const std::string content((std::istreambuf_iterator<char>(input)),
                              std::istreambuf_iterator<char>());
    const auto sim=mls::twin::simulateMultiPoseNc(content,200000);
    if(!sim.ok()){
        std::cerr<<"NC interpretation BLOCK: "<<sim.error<<'\n';return 4;
    }
    std::cout<<std::fixed<<std::setprecision(5);
    std::cout<<"OFFLINE_NC_INTERPRETER_PASS\n"
        <<"H"<<sim.topology<<" frequency="<<sim.profileRateHz<<"Hz edge="<<sim.h3Edge<<'\n'
        <<"joint_vertices="<<sim.vertices.size()<<" segments="<<sim.vertices.size()-1
        <<" scan_groups="<<sim.scanGroups<<" saved_groups="<<sim.saveGroups
        <<" simulated_profiles="<<sim.profiles.size()<<'\n'
        <<"nominal_duration_sec="<<sim.durationSeconds<<'\n';
    if(!sim.vertices.empty()){
        std::cout<<"initial_G00_reference_XYZCB=";
        for(int i=0;i<5;++i)std::cout<<(i?",":"")<<sim.vertices.front().joints[i];
        std::cout<<'\n';
    }
    std::cout<<"SAFETY_NOT_CLEARED: first G00 approach is not anchored to live MPOS; "
        "STEP placement and real machine collision are not assessed by this tool\n";
}
