// OFFLINE engineering utility. It NEVER communicates with a controller,
// changes machine parameters, or approves NC execution.
#include "twin/NcG93MotionAudit.h"
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
int main(int argc,char** argv) {
    if(argc!=3 || std::string(argv[1])==std::string(argv[2])) {
        std::cerr<<"Usage: nc_g93_retime_candidate_cli <original.nc> <new_candidate.nc>\n"
                    "Writes a new OFFLINE-ONLY NC using bundled software limits X/Y/Z=5 mm/s "
                    "C/B=2 deg/s, accel=decel=25.\n"
                    "Requires a complete STEP, fixture and on-machine preflight again.\n";
        return 2;
    }
    std::ifstream f(argv[1],std::ios::binary);
    if(!f){std::cerr<<"Could not open input NC\n";return 3;}
    const std::string source(std::istreambuf_iterator<char>(f),{});
    const auto parsed=mls::twin::simulateMultiPoseNc(source,200000);
    if(!parsed.ok()){std::cerr<<parsed.error<<'\n';return 4;}
    const mls::runtime::Limits softwareLimits{};
    const auto result=mls::twin::retimeNcG93Conservative(source,parsed,softwareLimits,25,25);
    if(!result.ok()){std::cerr<<result.error<<'\n';return 5;}
    std::ofstream out(argv[2],std::ios::binary|std::ios::trunc);
    if(!out || !(out<<result.text)){std::cerr<<"Cannot write output candidate\n";return 6;}
    std::cout<<"OFFLINE RETIME CANDIDATE ONLY: no machine authorization.\n"
             <<"Changed G93 motion points: "<<result.changedMotionPoints<<'\n'
             <<"Original NC duration: "<<result.originalSeconds<<" seconds\n"
             <<"Retimed NC duration: "<<result.retimedSeconds<<" seconds\n"
             <<"M50/M51, XYZCB, S0/S1 geometry unchanged; 1-4/4 must be repeated.\n";
    return 0;
}
