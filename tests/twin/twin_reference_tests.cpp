#include "twin/TwinScene.h"
#include <cassert>
#include <cmath>
#include <iostream>
using namespace mls::twin;
int main(int argc,char** argv){
    if(argc!=2){std::cerr<<"usage: twin_reference_tests example.obj\n";return 2;}
    TwinScene scene;std::string err;
    if(!scene.loadObjMesh(argv[1],&err)){std::cerr<<err<<"\n";return 1;}
    const auto& b=scene.mesh().bounds;
    assert(!scene.mesh().empty());
    assert(std::abs(b.size().x-30.0)<0.2);
    assert(std::abs(b.size().y-95.0)<0.2);
    assert(std::abs(b.size().z-10.0)<0.02);
    scene.setTruePose({44.3195,10.2369,-225.05,0,0,-89.2265});
    const auto world=scene.workpieceBoundsMachine();
    assert(world.valid()&&world.min.z < -225.04&&world.max.z > -215.06);
    // CAD-local planar face away from holes, ray in MCS must hit part top.
    const auto T=scene.cadToMachineTruth();
    const Vec3 p=T.point({11,-15,25});
    const auto hit=scene.firstHit({p,T.vector({0,0,-1})});
    assert(hit.hit&&hit.objectName=="workpiece");
    assert(std::abs(hit.positionMachine.z+215.05)<0.01);
    // No filling in genuine empty space: far-away ray remains unobserved.
    assert(!scene.firstHit({{999,999,0},{0,0,-1}}).hit);
    std::cout<<"V28 real CAD reference mesh test PASSED: "<<scene.mesh().triangles.size()<<" triangles\n";
}
