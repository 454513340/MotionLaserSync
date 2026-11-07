#include "twin/TwinScene.h"
#include "twin/TwinTrajectory.h"
#include <cassert>
#include <cmath>
#include <iostream>
using namespace mls::twin;
bool near(double a,double b,double tol=1.e-8){return std::abs(a-b)<tol;}
int main(){
    for(Pose6 pose: {Pose6{},Pose6{4,-2,1,20,-35,70},Pose6{44.32,10.24,-225.05,0,0,-89.226}}){
        const auto T=Transform::fromPose(pose);const auto inv=T.inverseRigid();
        const Vec3 p{4,5,-12},p1=inv.point(T.point(p));
        assert(near(p1.x,p.x)&&near(p1.y,p.y)&&near(p1.z,p.z));
        auto identity=T*inv;
        assert(near(identity.m[0],1)&&near(identity.m[5],1)&&near(identity.m[10],1));
        assert(near(identity.m[3],0)&&near(identity.m[7],0)&&near(identity.m[11],0));
    }
    TwinScene scene;
    TriangleMesh mesh;
    mesh.triangles.push_back({{-10,-10,0},{10,-10,0},{10,10,0},1});
    mesh.triangles.push_back({{-10,-10,0},{10,10,0},{-10,10,0},2});
    assert(scene.setWorkpiece(std::move(mesh)));
    scene.setTruePose({30,40,-200,0,0,90});
    scene.setEstimatedPose({0,0,0,0,0,0});
    const auto hit=scene.firstHit({{30,40,-190},{0,0,-1}});
    assert(hit.hit&&hit.objectName=="workpiece"&&near(hit.positionMachine.z,-200));
    assert(!scene.firstHit({{130,140,-190},{0,0,-1}}).hit);
    const auto dims=scene.workpieceBoundsMachine();
    assert(dims.valid()&&near(dims.center().x,30)&&near(dims.center().y,40));
    scene.setSupportPlane(true,-210,30,40,100,100);
    const auto ground=scene.firstHit({{60,60,-190},{0,0,-1}});
    assert(ground.hit&&ground.objectName=="support"&&near(ground.distanceMm,20));
    // The scene truth must NOT be silently changed by estimated placement.
    assert(near(scene.truePose().x,30)&&near(scene.estimatedPose().x,0));
    TwinTrajectory trajectory;std::string err;
    assert(!trajectory.setSamples({{1,{}},{1,{}}},&err));
    assert(trajectory.setSamples({{0,Joint5{{0,0,0,0,0}}},{1,Joint5{{10,20,30,40,50}}}}));
    Joint5 q{};assert(trajectory.interpolate(.5,&q));
    for(std::size_t i=0;i<5;++i)assert(near(q[i],5.*static_cast<double>(i+1)));
    std::cout<<"V28 M0/M1 portable geometry tests PASSED\n";
}
