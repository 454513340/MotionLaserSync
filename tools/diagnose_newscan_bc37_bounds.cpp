// Reproduce the user-supplied 6061_Alumina_Scan_H2_New_01.scan first-frame
// calibration as an offline 37-case joint-limit diagnostic (NO controller use).
// SCAN v3 has hardware-latched positions, but its name does not prove H2 topology.
#include "twin/MultiPoseExperiment.h"
#include "calibration/SingleSphere17Calibration.h"
#include <array>
#include <algorithm>
#include <iomanip>
#include <iostream>
#include <string>
using namespace mls::twin;
using namespace mls::calibration;
int main(){
    // First profile snapshot (X,Y,Z,C,B), read from the uploaded .scan.
    const Joint5 archived{{-0.001500000013038516,-0.0060000000521540642,
                           -5.8555002212524414,-0.0005664825439453125,
                           5.8364868164062489e-05}};
    const double physical[]={
        0.0020145012222724402,0.0014825560673554501,-0.0031379475844700599,
        1.6502535339522599,0.182794747646847,0.00016249655477456899,
        -0.00021668825973842601,2.7838744120589198,0.143752270364912,
        0.0038821378454866099,-0.00026147067294919298,0.0131773706992971,
        0.010026316982626401,1.57523368889529,31.1852777770888,
        11.253508753988299,-292.70878630012402};
    const std::array<double,16> recorded{{
        -0.0044639666620056775,-0.99988101475164282,0.014765816633886656,31.175274000597827,
        0.99998796031455028,-0.0044935596479845066,-0.0019715850565085258,11.266174165882452,
        0.0020377015447664922,0.014756837768135416,0.99988903559920062,-298.56422230564397,
        0,0,0,1}};
    Physical17Parameters params{};
    for(int i=0;i<17;++i)params[i]=physical[i];
    const auto toPhysical=[](const Joint5& q)->JointPose5d{
        return {q[0],q[1],q[2],q[3],q[4]};
    };
    const Transform anchor=Transform{recorded}*
        Transform{SingleSphere17Model::sensorToMachineTransform(toPhysical(archived),params)}.inverseRigid();
    const M6ForwardKinematics fk=[&](const Joint5& q)->Transform{
        return anchor*Transform{SingleSphere17Model::sensorToMachineTransform(toPhysical(q),params)};
    };
    TwinScene scene;
    std::string error;
    if(!scene.loadObjMesh("examples/digital_twin/6061_sensor_connector_NEW_2026.obj",&error)){
        std::cerr<<"Fixture CAD mesh: "<<error<<'\n';return 1;
    }
    scene.setEstimatedPose({-9.769,7.347,-217.681,0,0,-90});
    const auto bb=scene.mesh().bounds;
    ExperimentOptions o;
    o.roiCad.min={bb.min.x,bb.min.y,10};
    o.roiCad.max={bb.max.x,bb.max.y,10};
    o.surfaceZCadMm=10.;
    o.focusSensorZmm=95.;
    o.profileRateHz=200.;
    o.scanSpeedMmPerSec=10.;
    o.transferSpeedMmPerSec=10.;
    o.lanePitchMm=32.;
    o.leadInMm=5.;o.leadOutMm=5.;
    o.maximumProfiles=160000;
    o.bounds.min={{-147,-116,-32,-87,-87}};
    o.bounds.max={{114,134,53,87,87}};
    const auto poses=standardBCExperimentMatrix();
    if(poses.size()!=37 || poses.back().bDeg!=0 || poses.back().cDeg!=0)
        return 2;
    std::cout<<std::fixed<<std::setprecision(4);
    std::cout<<"pose,B_deg,C_deg,result,minX,maxX,minY,maxY,minZ,maxZ,profiles,reason\n";
    unsigned passed=0;
    std::size_t aggregateProfiles=0;
    for(const auto& pose:poses){
        const auto plan=planMultiPoseExperiment(scene,{pose},o,fk);
        if(!plan.ok()){
            std::cout<<pose.id<<','<<pose.bDeg<<','<<pose.cDeg
                     <<",FAIL,,,,,,,0,\""<<plan.error<<"\"\n";
            continue;
        }
        ++passed;
        const auto& v=plan.views.front();
        aggregateProfiles+=v.plannedProfiles;
        std::array<double,5> mn{{1e9,1e9,1e9,1e9,1e9}};
        std::array<double,5> mx{{-1e9,-1e9,-1e9,-1e9,-1e9}};
        for(const auto& p:v.path){
            for(int axis=0;axis<5;++axis){
                mn[axis]=std::min(mn[axis],p.joints[axis]);
                mx[axis]=std::max(mx[axis],p.joints[axis]);
            }
        }
        std::cout<<pose.id<<','<<pose.bDeg<<','<<pose.cDeg<<",PASS,"
                 <<mn[0]<<','<<mx[0]<<','<<mn[1]<<','<<mx[1]<<','
                 <<mn[2]<<','<<mx[2]<<','<<v.plannedProfiles<<",\n";
    }
    std::cerr<<"BASIC_FK_AXIS_PASS "<<passed<<"/"<<poses.size()
             <<"; planned_profiles="<<aggregateProfiles<<"/"<<o.maximumProfiles
             <<"; excludes FCL/optical coverage/physical machine execution.\n";
    if(passed!=poses.size() || aggregateProfiles>o.maximumProfiles)return 3;
    const auto full=planMultiPoseExperiment(scene,poses,o,fk);
    if(!full.ok()){
        std::cerr<<"AGGREGATE_PLAN_FAIL "<<full.error<<'\n';return 4;
    }
    if(full.views.size()!=poses.size())return 5;
    return 0;
}
