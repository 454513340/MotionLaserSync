// Offline regression: uses the reference STEP-matched OBJ and explicitly
// supplied recorded SCAN v3 Physical17 snapshot. No hardware instructions.
#include "twin/MultiPoseExperiment.h"
#include "calibration/SingleSphere17Calibration.h"
#include <iostream>
#include <iomanip>
#include <sstream>
using namespace mls::twin;
using namespace mls::calibration;
int main(){
    const double initial[]={51.993499755859375,-.0044999998062849045,-27.672500610351562,-.0004978179931640625,1.201629638671875e-05};
    const double physical[]={.00201450122227244,.00148255606735545,-.00313794758447006,1.65025353395226,.182794747646847,.000162496554774569,-.000216688259738426,2.78387441205892,.143752270364912,.00388213784548661,-.000261470672949193,.0131773706992971,.0100263169826264,1.57523368889529,31.1852777770888,11.2535087539883,-292.708786300124};
    const std::array<double,16> stored={{
    -.00446276975,-.999881008,.0147666233,83.1377083,
    .999987966,-.00449236451,-.00197160317,11.3360994,
    .00203770562,.0147576468,.999889024,-320.381114,
    0,0,0,1}};
    Physical17Parameters params{};for(int i=0;i<17;++i)params[i]=physical[i];
    Joint5 origin{{initial[0],initial[1],initial[2],initial[3],initial[4]}};
    auto toPhysical=[](const Joint5&q)->JointPose5d{return {q[0],q[1],q[2],q[3],q[4]};};
    const Transform anchor = Transform{stored} * Transform{SingleSphere17Model::sensorToMachineTransform(toPhysical(origin),params)}.inverseRigid();
    const M6ForwardKinematics fk=[&](const Joint5&q)->Transform{
        return anchor*Transform{SingleSphere17Model::sensorToMachineTransform(toPhysical(q),params)};
    };
    TwinScene scene;
    std::string err;
    if(!scene.loadObjMesh("examples/digital_twin/6061_sensor_connector_NEW_2026.obj",&err)){
      std::cerr<<"OBJ: "<<err<<"\n";return 1;}
    const auto bb=scene.mesh().bounds;
    std::cout<<"Mesh bounds "<<bb.min.x<<","<<bb.max.x<<" / "<<bb.min.y<<","<<bb.max.y<<" / "<<bb.min.z<<","<<bb.max.z<<"\n";
    scene.setEstimatedPose({43.713,10.208,-225.045,0,0,-90});
    ExperimentOptions o;
    o.roiCad.min={bb.min.x,bb.min.y,10};o.roiCad.max={bb.max.x,bb.max.y,10};o.surfaceZCadMm=10.;
    o.focusSensorZmm=95;o.scanSpeedMmPerSec=10;o.transferSpeedMmPerSec=10;
    o.profileRateHz=200;o.lanePitchMm=32;o.leadInMm=5;o.leadOutMm=5;
    o.maximumProfiles=160000;o.hasArchivedInitialJoints=true;o.archivedInitialJoints=origin;
    o.bounds.min={{-147,-116,-32,-87,-87}};o.bounds.max={{114,134,53,87,87}};
    std::cout<<std::setprecision(7);
    for(double retract:{10.0,0.0})for(double lead:{5.0,0.0}){
       o.retractFromSurfaceCadMm=retract;o.leadInMm=lead;o.leadOutMm=lead;
       auto plan=planMultiPoseExperiment(scene,{{"P01",-10,-20}},o,fk);
       std::cout<<"retract="<<retract<<" extension="<<lead<<" : "<< (plan.ok()?"SUCCESS":plan.error)<<"\n";
       if(plan.ok()){
         for(size_t i=0;i<plan.views[0].path.size();i+=std::max<size_t>(1,plan.views[0].path.size()/4)){
           const auto &q=plan.views[0].path[i].joints;
           std::cout<<"  joint=";for(double v:q)std::cout<<v<<" ";std::cout<<"\n";}
       }
    }
    o.retractFromSurfaceCadMm=0.;o.leadInMm=5.;o.leadOutMm=5.;
    int ok=0;
    std::cout<<"\n=== historical 45-pose offline kinematic check (without FCL) ===\n";
    // Historical V29 fixture (B=-20..20 / C=-10..10 / 5-degree step).
    // Kept independent from the current 37-case experimental default to
    // prevent silently changing previously reported 34/45 regression.
    std::vector<ExperimentPose> historical;
    for(int b=-20;b<=20;b+=5){
        for(int c=-10;c<=10;c+=5){
            const auto n=historical.size()+1;
            const std::string id=std::string("P")+(n<10?"0":"")+std::to_string(n);
            historical.push_back({id,double(c),double(b)});
        }
    }
    for(const auto& pose:historical){
        auto plan=planMultiPoseExperiment(scene,{pose},o,fk);
        if(plan.ok())++ok;
        std::cout<<pose.id<<" B="<<pose.bDeg<<" C="<<pose.cDeg<<" "<<(plan.ok()?"BASIC_KINEMATIC_PASS":plan.error)<<"\n";
    }
    std::cout<<"BASIC_KINEMATIC_PASS_COUNT "<<ok<<" / 45\n";
    if(ok!=34){std::cerr<<"REAL H3 snapshot axis-limit regression changed\n";return 2;}
    return 0;
}
