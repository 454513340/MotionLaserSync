#include "twin/TwinM34ReturnModel.h"
#include <cassert>
#include <cmath>
#include <fstream>
#include <iostream>
using namespace mls::twin;
static void square(TriangleMesh& mesh,Vec3 a,Vec3 b,Vec3 c,Vec3 d){
  mesh.triangles.push_back({a,b,c,1});mesh.triangles.push_back({a,c,d,1});
}
int main(){
    Sensor2950Geometry geometry;M34ReturnParams trained;
    trained.equivalentReceiverSensor=geometry.cameraOriginSensor;
    assert(trained.validate());
    TwinScene scene;TriangleMesh mesh;
    square(mesh,{-45,-45,100},{45,-45,100},{45,45,100},{-45,45,100});
    assert(scene.setWorkpiece(std::move(mesh)));
    VirtualLaser2950 laser(scene,geometry);
    VirtualProfilePose pose;
    auto profile=laser.simulate(pose);
    TwinM34ReturnModel model(trained,geometry);
    assert(profile.samples.size()==1280);
    double sum=0.;
    for(std::size_t i=0;i<profile.samples.size();++i){
        auto& point=profile.samples[i];
        assert(point.hitObject==VirtualHitObject::Workpiece);
        assert(point.cadFaceId==1);
        assert(std::abs(point.rayHitSensorZ-100)<.001);
        assert(std::isfinite(point.receiverNormalCos)&&point.receiverNormalCos>=0&&point.receiverNormalCos<=1);
        assert(std::isfinite(point.laserNormalCos)&&point.laserNormalCos>=0&&point.laserNormalCos<=1);
        const double p=model.probability(profile,i,false);
        assert(p>0&&p<1);
        assert(model.probability(profile,i,true)<=p);
        sum+=p;
    }
    assert(sum>500);
    // Regression: first-hit identity and incidence are retained when blocked.
    TriangleMesh blockedMesh;
    square(blockedMesh,{-45,-45,100},{45,-45,100},{45,45,100},{-45,45,100});
    square(blockedMesh,{-45,20,125},{45,20,125},{45,20,165},{-45,20,165});
    TwinScene blockedScene;assert(blockedScene.setWorkpiece(std::move(blockedMesh)));
    VirtualLaser2950 blockedScanner(blockedScene,geometry);
    auto blocked=blockedScanner.simulate(pose);
    std::size_t n=0;
    for(std::size_t i=0;i<blocked.samples.size();++i){
        const auto& v=blocked.samples[i];
        if(v.state==VirtualSampleState::CameraOccluded){
            assert(v.hitObject==VirtualHitObject::Workpiece);
            assert(std::isfinite(v.laserNormalCos));
            assert(std::isfinite(v.cameraOcclusionClearanceMm));
            assert(model.probability(blocked,i,false)==0.);
            ++n;
        }
    }
    assert(n>1000);
    M34ReturnParams unsafe=trained;
    unsafe.readyForM6=true;assert(!unsafe.validate());
    unsafe=trained;unsafe.weights[2]=1.;assert(!unsafe.validate());
    const std::string file="m34_v2_tmp_parameters.cfg";
    {
      std::ofstream f(file);
      f<<"M34P2\n0 90 251.25\n4 -3 -3 -0.25 -0.2 -0.7 -1\n1 0\n";
    }
    M34ReturnParams parsed;
    assert(TwinM34ReturnModel::readResearchParameters(file,parsed));
    assert(std::abs(parsed.equivalentReceiverSensor.y-90)<1e-8);
    std::remove(file.c_str());
    std::cout<<"M3/M4 V2 geometric features, gated probabilities, strict parameter loading PASSED\n";
}
