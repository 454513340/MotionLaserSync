#include "twin/TwinM51Chamfer.h"
#include <cassert>
#include <cmath>
#include <iostream>
using namespace mls::twin;
static Vec3 chamfer(double angle,double u){
    const double a=angle*3.14159265358979323846/180.;
    const double radius=3.25-.5*u;
    return {radius*std::cos(a),-38.+radius*std::sin(a),10.-.5*u};
}
int main(int argc,char**argv){
    assert(argc>=2);
    TwinScene scene;
    std::string err;
    assert(scene.loadObjMesh(argv[1],&err));
    TwinM51ChamferAnalyzer analyzer(scene);
    if(!analyzer.ready())std::cerr<<analyzer.error()<<"\n";
    assert(analyzer.ready());
    for(int angle=0;angle<360;angle+=5){
        if(angle>=210&&angle<300)continue; // known synthetic sparse sector
        for(int layer=0;layer<4;++layer){
            for(int k=0;k<12;++k){
                const double u=(layer+(k+.5)/12.)/4.;
                analyzer.addRealCad(chamfer(angle+.2,u));
            }
        }
    }
    for(int angle=0;angle<360;angle+=5){
        for(int layer=0;layer<4;++layer){
            for(int k=0;k<12;++k){
                const double u=(layer+(k+.5)/12.)/4.;
                analyzer.addVirtualCad(chamfer(angle+.2,u));
            }
        }
    }
    // Reject a top-plane point 1.5mm outside the nominal chamfer,
    // including for the loose .65mm diagnostic threshold.
    analyzer.addRealCad({4.75,-38.,10.});
    auto r=std::move(analyzer).finish();
    assert(r.ok());
    assert(r.surfaces.size()==3);
    assert(std::abs(r.surfaces[0].nominalAreaMm2-13.33)<.05);
    assert(std::abs(r.surfaces[1].nominalAreaMm2-155.51)<.3);
    assert(r.realLowDensityAngles>=16 && r.realLowDensityAngles<=20);
    assert(r.realAssociated[0]==(72-18)*4*12);
    assert(r.virtualAssociated[0]==72*4*12);
    assert(r.bins.size()==72*(4+20+4));
    // Top chamfer no-return region is sparse in real but not virtual.
    const auto& empty=r.bins[static_cast<std::size_t>(2)*72+45];
    assert(empty.realHits[0]==0&&empty.virtualHits[0]>0&&empty.realSparse);
    // Ensure a lower chamfer and cylinder return go to independent surface groups.
    TwinM51ChamferAnalyzer geometryAnalyzer(scene);
    assert(geometryAnalyzer.ready());
    const auto toRad=[](double d){return d*3.14159265358979323846/180.;};
    const double a=toRad(45.);
    geometryAnalyzer.addRealCad({3.0*std::cos(a),-38.+3.0*std::sin(a),.25});
    geometryAnalyzer.addRealCad({2.75*std::cos(a),-38.+2.75*std::sin(a),5.0});
    auto geometry=std::move(geometryAnalyzer).finish();
    std::uint64_t cylinder=0,lower=0;
    for(const auto& b:geometry.bins){
        if(b.surface==M51Surface::Cylinder)cylinder+=b.realHits[1];
        if(b.surface==M51Surface::BottomChamfer)lower+=b.realHits[1];
    }
    assert(cylinder==1&&lower==1);
    // A 30x95x10 outer box WITHOUT a center hole is not a valid H1 target.
    TwinScene flatScene;
    TriangleMesh plain;
    plain.triangles.push_back({{-15,-55,10},{15,-55,10},{15,40,10},1});
    plain.triangles.push_back({{-15,-55,10},{15,40,10},{-15,40,10},1});
    plain.triangles.push_back({{-15,-55,0},{15,-55,0},{15,40,0},2});
    assert(flatScene.setWorkpiece(std::move(plain)));
    TwinM51ChamferAnalyzer shouldFail(flatScene);
    assert(!shouldFail.ready());
    std::cout<<"M5.1 H1 CAD/sector/area/geometry tests OK\n";
}
