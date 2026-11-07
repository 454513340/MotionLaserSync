#include "twin/MultiPoseRegistrationAssist.h"
#include "twin/TwinScene.h"
#include <cmath>
#include <iostream>
#include <vector>
using namespace mls::twin;
#define REQUIRE(c) do{if(!(c)){std::cerr<<"failed at "<<__LINE__<<" : "<<#c<<"\n";return 1;}}while(false)
int main(){
    TriangleMesh cad;
    // A known planar component in CAD XY and a second lower parallel plane.
    // Registration assistance must NEVER claim a complete 6DoF certification.
    for(double z:{0.,10.}){
        const Vec3 a{-15,-55,z},b{15,-55,z},c{15,40,z},d{-15,40,z};
        cad.triangles.push_back({a,b,c,1});cad.triangles.push_back({a,c,d,1});
    }
    cad.rebuildBounds();
    const Pose6 truth{45.,10.,-225.,0.,0.,-90.};
    const Transform T=Transform::fromPose(truth);
    std::vector<Vec3> obs;obs.reserve(9000);
    for(int i=0;i<=95;++i)for(int j=0;j<=30;++j){
        obs.push_back(T.point({-15.+double(j),-55.+double(i),10.}));
        if((i+j)%2==0)obs.push_back(T.point({-15.+double(j),-55.+double(i),0.}));
    }
    for(int i=0;i<250;++i) obs.push_back({300.+i,-100.,-250.});
    auto r=proposeTopSurfaceRegistration(cad,obs,1000);
    std::cerr<<"proposed xyz="<<r.proposed.x<<","<<r.proposed.y<<","<<r.proposed.z
             <<" yaw="<<r.proposed.rzDeg<<" inliers="<<r.fractionWithinOneMm
             <<" median="<<r.medianSurfaceDistanceMm<<"\n";
    REQUIRE(r.proposedAvailable);
    REQUIRE(r.observedSamples>=150);
    REQUIRE(std::abs(r.proposed.z-truth.z)<.7);
    REQUIRE(r.fractionWithinOneMm>.55);
    REQUIRE(!r.message.empty());
    REQUIRE(!proposeTopSurfaceRegistration(cad,{{0,0,0}},500).proposedAvailable);
    std::cout<<"Coarse registration proposal / non-certified limits / sparse failure PASS\n";
}
