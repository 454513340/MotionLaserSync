#include "twin/VirtualLaser2950.h"
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
using namespace mls::twin;
static void square(TriangleMesh& m, Vec3 a,Vec3 b,Vec3 c,Vec3 d){
    const std::uint32_t id=std::uint32_t(m.triangles.size()/2+1);
    m.triangles.push_back({a,b,c,id});m.triangles.push_back({a,c,d,id});
}
int main(){
    Sensor2950Geometry model;
    assert(model.validate());
    assert(std::abs(model.cameraOriginSensor.y-75.0)<1.e-12); // same baseline as GUI
    // Raw H1 samples decrease in Sensor-X as raw index increases.
    assert(model.sampleXAtZ(0,105.34)>0.0);
    assert(model.sampleXAtZ(1279,105.34)<0.0);
    const double sampleDx=model.sampleXAtZ(1,105.34)-model.sampleXAtZ(0,105.34);
    assert(std::abs(sampleDx+model.widthAtZ(105.34)/1280.0)<1.e-10);
    {
        Sensor2950Geometry bad=model;
        bad.widthAtCenter=std::numeric_limits<double>::quiet_NaN();
        assert(!bad.validate());
        bad=model;bad.maxTraceDistanceMm=std::numeric_limits<double>::infinity();
        assert(!bad.validate());
        bad=model;bad.cameraVisibilityEpsilonMm=std::numeric_limits<double>::quiet_NaN();
        assert(!bad.validate());
        bad=model;bad.sampleCount=-1;
        assert(!bad.validate());
        TwinScene harmless;
        VirtualLaser2950 invalidLaser(harmless,bad);
        assert(invalidLaser.simulate(VirtualProfilePose{}).samples.empty());
    }
    assert(std::abs(model.widthAtZ(95)-50)<1.e-12);
    assert(std::abs(model.widthAtZ(105)-46.8)<1.e-8);
    VirtualProfilePose pose;pose.sensorToMachine=Transform{};
    {
        TriangleMesh m;
        square(m,{-40,-40,100},{40,-40,100},{40,40,100},{-40,40,100});
        TwinScene scene;assert(scene.setWorkpiece(std::move(m)));
        VirtualLaser2950 scanner(scene,model);
        auto p=scanner.simulate(pose);
        assert(p.samples.size()==1280);
        std::uint64_t good=0;
        for(const auto& s:p.samples)if(s.state==VirtualSampleState::Valid){
            ++good;assert(std::abs(s.sensorZ-100)<1.e-3);
        }
        assert(good>1200);
        VirtualProfilePose nonRigid=pose;
        nonRigid.sensorToMachine.m[0]=1.15; // finite but not a rigid pose
        const auto invalid=scanner.simulate(nonRigid);
        for(const auto& point:invalid.samples)
            assert(point.state==VirtualSampleState::BadPose);
    }
    {
        TriangleMesh m;
        square(m,{-40,-40,100},{40,-40,100},{40,40,100},{-40,40,100});
        // Positive camera Y: wall at +20 blocks camera rays but not laser plane Y=0.
        square(m,{-45,+20,125},{45,+20,125},{45,+20,165},{-45,+20,165});
        TwinScene scene;assert(scene.setWorkpiece(std::move(m)));
        VirtualLaser2950 scanner(scene,model);
        auto p=scanner.simulate(pose);
        std::uint64_t blocked=0;
        for(const auto& s:p.samples)if(s.state==VirtualSampleState::CameraOccluded)++blocked;
        assert(blocked>1000);
    }
    {
        TriangleMesh m;
        square(m,{-40,-40,125},{40,-40,125},{40,40,125},{-40,40,125});
        TwinScene scene;assert(scene.setWorkpiece(std::move(m)));
        VirtualLaser2950 scanner(scene,model);
        auto p=scanner.simulate(pose);
        std::uint64_t out=0;
        for(const auto& s:p.samples)if(s.state==VirtualSampleState::OutsideField)++out;
        assert(out>1000);
    }
    {
        TriangleMesh m;
        square(m,{-40,-40,100},{40,-40,100},{40,40,100},{-40,40,100});
        TwinScene scene;assert(scene.setWorkpiece(std::move(m)));
        const TwinRayBvh accel(scene);
        for(int i=0;i<80;++i){
            Ray ray{{-50.+i*1.25,-10,155},{0,0,-1}};
            auto a=scene.firstHit(ray);
        auto b=accel.firstHit(ray,1000);
            assert(a.hit==b.hit);
            if(a.hit)assert(std::abs(a.distanceMm-b.distanceMm)<1.e-7);
        }
        VirtualLaser2950 scanner(scene,model);
        std::vector<VirtualProfilePose> v(3,pose);
        std::atomic<bool> cancelled{true};
        auto r=scanner.simulateAll(v,&cancelled);
        assert(r.cancelled&&r.totalProfiles==0);
        cancelled=false;r=scanner.simulateAll(v,&cancelled);
        assert(r.totalProfiles==3&&r.validCount()>3600);
    }
    std::cout<<"V28 M3 geometric ray, occlusion, field, BVH and cancellation tests PASSED\n";
}
