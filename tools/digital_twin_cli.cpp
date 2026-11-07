#include "twin/TwinScene.h"
#include "twin/CadGeometryAdapter.h"
#include <iostream>
#include <iomanip>
#include <string>
int main(int argc,char**argv){
    using namespace mls::twin;
    if(argc!=3||(std::string(argv[1])!="--step"&&std::string(argv[1])!="--obj")){
        std::cerr<<"Usage: digital_twin_cli --step file.step | --obj file.obj\n";
        std::cerr<<"Open CASCADE STEP importer: "<<(CadGeometryAdapter::stepSupported()?"yes":"no")<<"\n";
        return 2;
    }
    TwinScene scene;std::string error;
    bool ok=false;
    if(std::string(argv[1])=="--step"){
        auto r=CadGeometryAdapter::loadStep(argv[2]);ok=r.ok;error=r.message;
        if(ok)ok=scene.setWorkpiece(std::move(r.mesh),&error);
    }else ok=scene.loadObjMesh(argv[2],&error);
    if(!ok){std::cerr<<"Import error: "<<error<<"\n";return 1;}
    const auto b=scene.mesh().bounds;
    std::cout<<std::fixed<<std::setprecision(4)<<"triangle_count="<<scene.mesh().triangles.size()<<"\n"
       <<"cad_min_mm="<<b.min.x<<","<<b.min.y<<","<<b.min.z<<"\n"
       <<"cad_max_mm="<<b.max.x<<","<<b.max.y<<","<<b.max.z<<"\n";
    scene.setTruePose({44.3195,10.2369,-225.05,0,0,-89.2265});
    const auto w=scene.workpieceBoundsMachine();
    std::cout<<"mcs_min_mm="<<w.min.x<<","<<w.min.y<<","<<w.min.z<<"\n"
       <<"mcs_max_mm="<<w.max.x<<","<<w.max.y<<","<<w.max.z<<"\n";
    return 0;
}
