#include "twin/TwinM6NcPreview.h"
#include "runtime/GCode5D.h"
#include <cstdlib>
#define REQUIRE(x) do { if (!(x)) { std::cerr << "FAIL line " << __LINE__ << ": " << #x << "\n"; return 1; } } while(false)
#include <iostream>
#include <sstream>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>
using namespace mls::twin;
int main(int argc, char** argv){
    M6Report r;
    r.options.hasInitialJoints=true;r.options.initialJoints={{0,0,0,0,0}};
    r.path={
       {0.00,{{0,0,0,0,0}},{},M6SegmentRole::Transfer},
       {1.00,{{1,0,0,0,0}},{},M6SegmentRole::Transfer},
       {1.00,{{1,0,0,0,0}},{},M6SegmentRole::Scan},
       {1.02,{{1.1,0,0,2,1}},{},M6SegmentRole::Scan},
       {1.04,{{1.2,0,0,4,2}},{},M6SegmentRole::Scan},
       {1.04,{{1.2,0,0,4,2}},{},M6SegmentRole::Transfer},
       {2.04,{{0,0,0,0,0}},{},M6SegmentRole::Transfer}
    };
    auto h1=makeM6NcPreview(r,{1,114,"",""});
    auto h3=makeM6NcPreview(r,{3,115,"",""});
    REQUIRE(h1.ok()&&h3.ok());
    REQUIRE(h1.scanningGroups==1&&h1.transferGroups==2);
    REQUIRE(h1.text.find("; OfflineOnly: true")!=std::string::npos);
    REQUIRE(h1.text.find("; HardwareTopology: H1")!=std::string::npos);
    REQUIRE(h1.text.find("; RegistrationVerified: false")!=std::string::npos);
    REQUIRE(h1.text.find("; InitialJointAnchored: true")!=std::string::npos);
    REQUIRE(h3.text.find("; HardwareTopology: H3")!=std::string::npos);
    REQUIRE(h3.text.find("; CmmR1Edge: 115")!=std::string::npos);
    REQUIRE(h1.text.find("M50\n")!=std::string::npos);
    REQUIRE(h1.text.find("G05 S1")!=std::string::npos);
    REQUIRE(h1.text.find("M51\nM60")!=std::string::npos);
    // Feed in G93 is inverse minutes; for a 20-ms segment F = 3000.
    REQUIRE(h1.text.find("F3000")!=std::string::npos);
    mls::runtime::NcState parser;
    std::istringstream in(h1.text);std::string line,err;
    int motions=0,points=0;bool inG5=false;
    while(std::getline(in,line)){
        mls::runtime::NcBlock b;
        if(!mls::runtime::parseNcBlock(line,parser,b,err)){
            std::cerr<<"generated invalid NC: "<<line<<" : "<<err<<'\n';return 1;
        }
        if(b.kind==mls::runtime::NcBlock::Kind::Motion){
            ++motions;if(b.motion==5)inG5=true;
            if(inG5 && b.motion==5 && b.axisMask==31)++points;
        }else if(b.kind==mls::runtime::NcBlock::Kind::MCode)inG5=false;
    }
    REQUIRE(motions>3&&points==4);
    REQUIRE(makeM6NcPreview(r,{2}).error.find("retired")!=std::string::npos);
    REQUIRE(!makeM6NcPreview(r,{}).ok());
    REQUIRE(makeM6NcPreview(r,{3,113}).error.find("H3")!=std::string::npos);
    r.canExportForHardware=true;
    REQUIRE(!makeM6NcPreview(r).ok());
    r.canExportForHardware=false;
    r.path[4].seconds=0.8;
    REQUIRE(!makeM6NcPreview(r).ok());
    if(argc==3 && std::string(argv[1])=="--write-examples"){
        std::filesystem::create_directories(argv[2]);
        const std::string folder=argv[2];
        std::ofstream a(folder+"/M6_SYNTHETIC_OFFLINE_ONLY_H1.nc",std::ios::binary);
        std::ofstream b(folder+"/M6_SYNTHETIC_OFFLINE_ONLY_H3.nc",std::ios::binary);
        REQUIRE(bool(a)&&bool(b));
        a<<h1.text;b<<h3.text;
        a.flush();b.flush();REQUIRE(bool(a)&&bool(b));
    }
    std::cout<<"M6 offline H1/H3 NC preview contract PASS\n";
}
