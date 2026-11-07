#include "twin/MultiPoseExperiment.h"
#include "twin/TwinM6NcPreview.h"
#include "twin/MultiPoseNcSimulation.h"
#include "twin/MultiPosePlayback.h"
#include "twin/VirtualLaser2950.h"
#include "twin/MultiPoseOpticalCoverage.h"
#include "runtime/GCode5D.h"
#include <cmath>
#include <iostream>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <set>
#include <vector>
#define CHECK(x) do{if(!(x)){std::cerr<<"FAIL "<<__LINE__<<" "<<#x<<"\n";return 1;}}while(false)
using namespace mls::twin;
static M6ForwardKinematics testForward(){
    return [](const Joint5& q){
        const double C=q[3]*kPi/180.,B=q[4]*kPi/180.;
        const double c=std::cos(C),s=std::sin(C),b=std::cos(B),a=std::sin(B);
        // Physical laser stripe is transverse to machine X (along machine Y).
        // The Rz(+90) sensor mount is essential: machine X scanning only
        // provides 2D coverage when the stripe has lateral extent.
        Transform t;t.m={{-s,-c*b,c*a,q[0], c,-s*b,s*a,q[1], 0,a,b,q[2], 0,0,0,1}};
        return t;
    };
}
int main(int argc,char** argv){
    TwinScene scene;
    TriangleMesh mesh;
    mesh.triangles={{{-10,-15,0},{10,-15,0},{10,15,0},1},{{-10,-15,0},{10,15,0},{-10,15,0},1}};
    mesh.rebuildBounds();std::string err;CHECK(scene.setWorkpiece(mesh,&err));
    scene.setEstimatedPose({0,0,0,0,0,0});
    ExperimentOptions options;
    options.roiCad.min={-5,-10,0};options.roiCad.max={5,10,0};
    options.surfaceZCadMm=0;options.scanSpeedMmPerSec=20;options.profileRateHz=200;
    options.transferSpeedMmPerSec=20;options.bounds.maxSpeed={{40,40,40,45,45}};options.lanePitchMm=20;options.opticalPreviewProfilesPerPose=9;
    const std::vector<ExperimentPose> poses={{"P01",0,0},{"P02",20,10},{"P03",-15,-12}};
    const auto plan=planMultiPoseExperiment(scene,poses,options,testForward());
    if(!plan.ok())std::cerr<<plan.error<<"\n";
    CHECK(plan.ok());CHECK(plan.views.size()==3);
    CHECK(!plan.registrationVerified&&!plan.hardwareAuthorized&&!plan.opticsVerifiedAcrossAngles);
    for(std::size_t n=0;n<plan.views.size();++n){
        const auto& v=plan.views[n];
        CHECK(v.id==poses[n].id);
        CHECK(v.plannedProfiles>=201); // explicit lead-in / lead-out
        CHECK(v.planarCoverageEnvelopePassed&&v.machineXDescending);
        CHECK(v.scanStartCad>options.roiCad.max.x);
        CHECK(v.scanEndCad<options.roiCad.min.x);
        CHECK(v.beforePartMarginMm>=5.&&v.afterPartMarginMm>=5.);
        CHECK(v.scanGroups==1);CHECK(v.opticalPreviewPoses.size()==9);
        CHECK(std::abs(v.predictedProfilePitchMm-.1)<1.e-12);
        int scanPoints=0,transferPoints=0;
        for(const auto& p:v.path){
            CHECK(std::abs(p.joints[3]-poses[n].cDeg)<1e-8);
            CHECK(std::abs(p.joints[4]-poses[n].bDeg)<1e-8);
            if(p.role==M6SegmentRole::Scan)++scanPoints; else ++transferPoints;
        }
        CHECK(scanPoints>0&&transferPoints>0);
        // This experiment intentionally has one optical lane. There must be
        // one UNEXPOSED start reference, NO Z lift before or after the scan.
        CHECK(transferPoints==1);
        CHECK(v.path.front().role==M6SegmentRole::Transfer);
        CHECK(v.path.back().role==M6SegmentRole::Scan);
        const auto m6=multiPoseAsOfflineM6(v,options);
        CHECK(!m6.options.hasInitialJoints); // historical SCAN != live MPOS
        M6NcPreviewOptions g;
        g.proposedTopologyId=3;g.h3R1Edge=115;g.studyId="same_cad_different_orientation";
        g.scanGroupLabel="MP_"+v.id;
        const auto nc=makeM6NcPreview(m6,g);
        if(!nc.ok())std::cerr<<nc.error<<"\n";
        CHECK(nc.ok());CHECK(nc.scanningGroups==1);
        CHECK(nc.text.find("M60 L\"MP_"+v.id+"_1\"")!=std::string::npos);
        CHECK(nc.text.find("; OfflineOnly: true")!=std::string::npos);
        CHECK(nc.text.find("HardwareTopology: H3")!=std::string::npos);
        CHECK(nc.text.find("RegistrationVerified: false")!=std::string::npos);
        CHECK(nc.text.find("ExperimentStudyId: same_cad_different_orientation")!=std::string::npos);
        // End-to-end: drive M3 from REINTERPRETED NC trigger poses, not the
        // ideal planner exposures. Same CAD ROI / changing C/B, each isolated.
        const auto executed=simulateMultiPoseNc(nc.text,options.maximumProfiles);
        if(!executed.ok())std::cerr<<executed.error<<"\n";
        CHECK(executed.ok());
        CHECK(verifyMultiPoseNcAgainstPlan(executed,m6).empty());
        CHECK(executed.profiles.size()>=v.plannedProfiles-1);
        CHECK(executed.profiles.size()<=v.plannedProfiles+1);
        Sensor2950Geometry geom;
        VirtualLaser2950 byNc(scene,geom,true);
        CHECK(byNc.ready(&err));
        const auto& centerProfile=executed.profiles[executed.profiles.size()/2];
        VirtualProfilePose sensorPose;
        sensorPose.elapsedSeconds=centerProfile.seconds;
        sensorPose.joints=centerProfile.joints;
        sensorPose.sensorToMachine=testForward()(centerProfile.joints);
        const auto synthesized=byNc.simulate(sensorPose);
        unsigned ncHits=0;
        for(const auto& sample:synthesized.samples)
            if(sample.state==VirtualSampleState::Valid && sample.hitObject==VirtualHitObject::Workpiece)++ncHits;
        CHECK(ncHits>0);
        mls::runtime::NcState state; std::istringstream input(nc.text);
        std::string line,parseError;
        while(std::getline(input,line)){
            mls::runtime::NcBlock block;
            CHECK(mls::runtime::parseNcBlock(line,state,block,parseError));
        }
        if(n==0){
            Sensor2950Geometry opt;VirtualLaser2950 virt(scene,opt,true);
            CHECK(virt.ready(&err));
            const auto frame=virt.simulate(v.opticalPreviewPoses[4]);
            unsigned valid=0;
            for(const auto& p:frame.samples)if(p.state==VirtualSampleState::Valid)++valid;
            CHECK(valid>0);
        }
    }
    // Regression: connecting separate pose NC files must NEVER teleport.
    // The merged experiment requires explicit collision-checked G05 S0
    // travel between each pair, with laser capture switched off.
    const auto allowPose=[](const Joint5&,std::string*){return true;};
    const auto connected=planConnectedExperimentSession(plan,options,testForward(),allowPose);
    if(!connected.ok())std::cerr<<connected.error<<"\n";
    CHECK(connected.ok());
    CHECK(connected.poseStartSeconds.size()==poses.size());
    CHECK(connected.poseEndSeconds.size()==poses.size());
    CHECK(connected.poseStartSeconds[0]==0.);
    CHECK(connected.poseStartSeconds[1]>connected.poseEndSeconds[0]);
    CHECK(connected.poseStartSeconds[2]>connected.poseEndSeconds[1]);
    CHECK(!planConnectedExperimentSession(plan,options,testForward(),{}).ok());
    const auto forbidTransfer=[](const Joint5& q,std::string* error){
        if(q[3]>0.1 && q[3]<19.9){if(error)*error="synthetic fixture collision";return false;}
        return true;
    };
    const auto blocked=planConnectedExperimentSession(plan,options,testForward(),forbidTransfer);
    CHECK(!blocked.ok());CHECK(blocked.error.find("laser-OFF inter-pose")!=std::string::npos);
    for(const int topo:{1,3}){
        M6NcPreviewOptions settings;settings.proposedTopologyId=topo;settings.h3R1Edge=115;
        settings.studyId="CONNECTED_EXPERIMENT_OFFLINE_ONLY";
        const auto sessionText=makeM6NcPreview(connected.merged,settings);
        if(!sessionText.ok())std::cerr<<sessionText.error<<"\n";
        CHECK(sessionText.ok());
        CHECK(sessionText.transferGroups>=2);
        const auto sessionNc=simulateMultiPoseNc(sessionText.text,options.maximumProfiles);
        if(!sessionNc.ok())std::cerr<<sessionNc.error<<"\n";
        CHECK(sessionNc.ok());
        CHECK(sessionNc.scanGroups==3&&sessionNc.saveGroups==3);
        CHECK(verifyMultiPoseNcAgainstPlan(sessionNc,connected.merged).empty());
        const auto timeline=makeConnectedNcPlaybackTimeline(connected,sessionNc);
        CHECK(timeline.ok());
        const double transferMid=(connected.poseEndSeconds[0]+connected.poseStartSeconds[1])/2.;
        const auto sample=sampleConnectedNcPlayback(sessionNc,timeline,transferMid,testForward());
        CHECK(sample.valid&&sample.betweenPoses&&!sample.scanning);
        CHECK(sample.poseIndex==1&&sample.previousPoseIndex==0);
        // NC and digital twin must show a genuine intermediate C/B pose.
        CHECK(sample.joints[3]>0&&sample.joints[3]<20);
        CHECK(sample.joints[4]>0&&sample.joints[4]<10);
        const auto next=sampleConnectedNcPlayback(sessionNc,timeline,
            connected.poseStartSeconds[1]+0.01,testForward());
        CHECK(next.valid&&next.poseIndex==1&&next.scanning&&!next.betweenPoses);
        // Deliberately corrupt the transfer NC: the composite-path verifier
        // must reject it, rather than replaying the original planner path.
        auto tampered=sessionText.text;
        const auto moveLine=tampered.find("G05 S0 ");CHECK(moveLine!=std::string::npos);
        const auto x=tampered.find(" X",moveLine);CHECK(x!=std::string::npos);
        const auto space=tampered.find(' ',x+2);CHECK(space!=std::string::npos);
        tampered.replace(x+2,space-x-2,"999");
        const auto manipulated=simulateMultiPoseNc(tampered,options.maximumProfiles);
        CHECK(manipulated.ok());
        CHECK(!verifyMultiPoseNcAgainstPlan(manipulated,connected.merged).empty());
    }
    const auto matrix=standardBCExperimentMatrix();
    CHECK(matrix.size()==37);
    const int angles[]={-10,-6,-2,2,6,10};
    std::set<std::pair<int,int>> uniqueAngles;
    std::set<std::string> uniqueIds;
    for(std::size_t i=0;i<matrix.size();++i){
        CHECK(matrix[i].id==std::string("P")+(i+1<10?"0":"")+std::to_string(i+1));
        CHECK(uniqueIds.insert(matrix[i].id).second);
        CHECK(uniqueAngles.insert({int(matrix[i].bDeg),int(matrix[i].cDeg)}).second);
        if(i<36){
            CHECK(matrix[i].bDeg==angles[i/6]);
            CHECK(matrix[i].cDeg==angles[i%6]);
            CHECK(matrix[i].bDeg!=0 && matrix[i].cDeg!=0);
        }else{
            CHECK(matrix[i].id=="P37"&&matrix[i].bDeg==0&&matrix[i].cDeg==0);
        }
    }
    CHECK(uniqueAngles.size()==37);
    auto many=options;many.maximumProfiles=160000;
    const auto matrixPlan=planMultiPoseExperiment(scene,matrix,many,testForward());
    if(!matrixPlan.ok())std::cerr<<"matrix plan: "<<matrixPlan.error<<"\n";
    CHECK(matrixPlan.ok());
    CHECK(matrixPlan.views.size()==37);
    for(const auto& view:matrixPlan.views){
        CHECK(view.planarCoverageEnvelopePassed&&view.machineXDescending);
        for(std::size_t k=1;k<view.path.size();++k){
            if(view.path[k].role==M6SegmentRole::Scan){
                const auto& prev=view.path[k-1];
                CHECK(view.focusPathMachine[k].x < view.focusPathMachine[k-1].x+1e-7);
            }
        }
    }
    // Full 37-pose session: no pose boundary may be silently spliced.
    const auto matrixJoined=planConnectedExperimentSession(matrixPlan,many,testForward(),allowPose);
    if(!matrixJoined.ok())std::cerr<<"joined BC37: "<<matrixJoined.error<<"\n";
    CHECK(matrixJoined.ok());
    CHECK(matrixJoined.poseStartSeconds.size()==37);
    CHECK(matrixJoined.poseEndSeconds.size()==37);
    for(std::size_t k=1;k<37;++k)
        CHECK(matrixJoined.poseStartSeconds[k]>matrixJoined.poseEndSeconds[k-1]);
    M6NcPreviewOptions matrixOpts;matrixOpts.proposedTopologyId=3;matrixOpts.h3R1Edge=115;
    matrixOpts.studyId="BC37_CONTINUOUS_NO_TELEPORT_TEST";
    const auto matrixNc=makeM6NcPreview(matrixJoined.merged,matrixOpts);
    CHECK(matrixNc.ok());
    const auto matrixSim=simulateMultiPoseNc(matrixNc.text,many.maximumProfiles);
    if(!matrixSim.ok())std::cerr<<"joined BC37 NC: "<<matrixSim.error<<"\n";
    CHECK(matrixSim.ok());
    CHECK(matrixSim.scanGroups==37&&matrixSim.saveGroups==37);
    CHECK(verifyMultiPoseNcAgainstPlan(matrixSim,matrixJoined.merged).empty());
    const auto matrixTimeline=makeConnectedNcPlaybackTimeline(matrixJoined,matrixSim);
    CHECK(matrixTimeline.ok());
    const double beforeLast=(matrixJoined.poseEndSeconds[35]+matrixJoined.poseStartSeconds[36])/2.;
    const auto lastTransition=sampleConnectedNcPlayback(matrixSim,matrixTimeline,beforeLast,testForward());
    CHECK(lastTransition.valid&&lastTransition.betweenPoses&&!lastTransition.scanning);
    CHECK(lastTransition.poseIndex==36&&lastTransition.previousPoseIndex==35);
    // Changing the legacy retract knob MUST NOT create a Z approach/exit.
    auto ignoredRetract=options;
    ignoredRetract.retractFromSurfaceCadMm=100.;
    const auto noRetractPlan=planMultiPoseExperiment(scene,{{"P01",0,0}},options,testForward());
    const auto oldRetractPlan=planMultiPoseExperiment(scene,{{"P01",0,0}},ignoredRetract,testForward());
    CHECK(noRetractPlan.ok()&&oldRetractPlan.ok());
    CHECK(noRetractPlan.views.front().path.size()==oldRetractPlan.views.front().path.size());
    for(std::size_t i=0;i<noRetractPlan.views.front().path.size();++i){
        const auto& a=noRetractPlan.views.front().path[i];
        const auto& b=oldRetractPlan.views.front().path[i];
        CHECK(a.role==b.role);
        for(int ax=0;ax<5;++ax)CHECK(std::abs(a.joints[ax]-b.joints[ax])<1.e-9);
    }
    // Multiple optical lanes must use laser-off direct transfers between
    // scans, never automatic Z lifting or a final Z-axis retraction.
    TwinScene wide;
    TriangleMesh wideMesh;
    wideMesh.triangles={{{-10,-60,0},{10,-60,0},{10,60,0},1},
                        {{-10,-60,0},{10,60,0},{-10,60,0},1}};
    wideMesh.rebuildBounds();CHECK(wide.setWorkpiece(wideMesh,&err));
    wide.setEstimatedPose({0,0,0,0,0,0});
    auto manyLanes=options;
    manyLanes.roiCad.min={-5,-55,0};manyLanes.roiCad.max={5,55,0};
    manyLanes.lanePitchMm=20.;
    const auto lanePlan=planMultiPoseExperiment(wide,{{"P01",0,0}},manyLanes,testForward());
    if(!lanePlan.ok())std::cerr<<lanePlan.error<<"\n";
    CHECK(lanePlan.ok());CHECK(lanePlan.views[0].scanGroups>=2);
    const auto& lane=lanePlan.views.front();
    CHECK(lane.path.front().role==M6SegmentRole::Transfer);
    CHECK(lane.path.back().role==M6SegmentRole::Scan);
    std::size_t firstSegments=0,laterTransfers=0;
    const double expectedZ=lane.path.front().joints[2];
    for(const auto& p:lane.path){
        CHECK(std::abs(p.joints[2]-expectedZ)<1.e-8);
        if(p.role==M6SegmentRole::Transfer)++laterTransfers;
        else ++firstSegments;
    }
    CHECK(laterTransfers>1&&firstSegments>1);
    M6NcPreviewOptions laneSettings;
    laneSettings.proposedTopologyId=3;laneSettings.h3R1Edge=115;
    const auto laneNc=makeM6NcPreview(multiPoseAsOfflineM6(lane,manyLanes),laneSettings);
    if(!laneNc.ok())std::cerr<<laneNc.error<<"\n";
    CHECK(laneNc.ok());
    const auto laneReplay=simulateMultiPoseNc(laneNc.text,manyLanes.maximumProfiles);
    CHECK(laneReplay.ok());
    CHECK(laneReplay.scanGroups==lane.scanGroups&&laneReplay.saveGroups==lane.scanGroups);
    // Intra-pose strip return must also be a real parsed, laser-OFF NC move.
    const auto firstM51=laneNc.text.find("M51\n");
    CHECK(firstM51!=std::string::npos);
    const auto offG05=laneNc.text.find("G05 S0 ",firstM51);
    CHECK(offG05!=std::string::npos);
    const auto nextM50=laneNc.text.find("M50\n",offG05);
    CHECK(nextM50!=std::string::npos&&firstM51<offG05&&offG05<nextM50);
    bool detectedStripMove=false;
    for(std::size_t j=1;j<laneReplay.vertices.size();++j){
        const auto& from=laneReplay.vertices[j-1];
        const auto& to=laneReplay.vertices[j];
        if(!to.scanning && to.seconds>from.seconds){
            detectedStripMove=true;
            CHECK(to.seconds-from.seconds>0.);
        }
    }
    CHECK(detectedStripMove);
    CHECK(verifyMultiPoseNcAgainstPlan(laneReplay,multiPoseAsOfflineM6(lane,manyLanes)).empty());
    // Validate M3 optical ROI coverage, not just one laser-line hit at an edge.
    // The planar full-ROI case must pass at tilted end angles and 0/0.
    VirtualLaser2950 rays(scene,Sensor2950Geometry{},true);
    CHECK(rays.ready(&err));
    for(const std::size_t index:{std::size_t(0),std::size_t(18),std::size_t(36)}){
        const auto& view=matrixPlan.views.at(index);
        MultiPoseOpticalCoverageGrid grid(scene,many.roiCad,6);
        const auto m6=multiPoseAsOfflineM6(view,many);
        M6NcPreviewOptions ncSettings;
        ncSettings.proposedTopologyId=3;
        ncSettings.h3R1Edge=115;
        const auto preview=makeM6NcPreview(m6,ncSettings);
        CHECK(preview.ok());
        const auto parsed=simulateMultiPoseNc(preview.text,many.maximumProfiles);
        CHECK(parsed.ok());
        CHECK(verifyMultiPoseNcAgainstPlan(parsed,m6).empty());
        const auto count=std::min<std::size_t>(36,parsed.profiles.size());
        for(std::size_t j=0;j<count;++j){
            const auto& profile=parsed.profiles[j*(parsed.profiles.size()-1)/(count-1)];
            VirtualProfilePose sample;
            sample.joints=profile.joints;
            sample.elapsedSeconds=profile.seconds;
            sample.sensorToMachine=testForward()(profile.joints);
            grid.ingest(rays.simulate(sample));
        }
        const auto coverage=grid.summarize();
        if(!coverage.sufficient)
            std::cerr<<"optical "<<view.id<<" ROI="<<coverage.fraction
                     <<" boundary="<<coverage.boundaryFraction<<"\n";
        CHECK(coverage.sufficient);
    }
    // Real deployment frequently has CAD Rz=-90deg. In that case CAD Y
    // increases with machine X, and must be scanned in reverse (Ymax->Ymin).
    TwinScene turned=scene;
    turned.setEstimatedPose({44.,10.,-225.,0.,0.,-90.});
    const auto changedCad=planMultiPoseExperiment(
        turned,{{"P01",0,0},{"P02",10,20}},options,testForward());
    if(!changedCad.ok())std::cerr<<"rotated CAD: "<<changedCad.error<<"\n";
    CHECK(changedCad.ok());
    for(const auto& view:changedCad.views){
        CHECK(view.resolvedAxis==ExperimentSweepAxis::CadY);
        CHECK(view.machineXDescending);
        CHECK(view.scanStartCad>options.roiCad.max.y);
        CHECK(view.scanEndCad<options.roiCad.min.y);
        bool sawScan=false;
        for(std::size_t k=1;k<view.path.size();++k){
            if(view.path[k].role!=M6SegmentRole::Scan)continue;
            sawScan=true;
            CHECK(view.focusPathMachine[k].x<=view.focusPathMachine[k-1].x+1e-7);
        }
        CHECK(sawScan);
    }
    // Degenerate optics guard: machine-X scanner with a parallel laser stripe
    // cannot claim 2-D coverage using a single 32-mm-pitch lane.
    CHECK(!planMultiPoseExperiment(scene,{{"P01",0,0}},options,
        [](const Joint5& q){Transform t;t.m[3]=q[0];t.m[7]=q[1];t.m[11]=q[2];return t;}).ok());
    auto bad=options;bad.roiCad.min.x=-1000;
    CHECK(!planMultiPoseExperiment(scene,poses,bad,testForward()).ok());
    bad=options;bad.profileRateHz=0;
    CHECK(!planMultiPoseExperiment(scene,poses,bad,testForward()).ok());
    bad=options;bad.maximumProfiles=15;
    CHECK(!planMultiPoseExperiment(scene,poses,bad,testForward()).ok());
    CHECK(!planMultiPoseExperiment(scene,{{"P01",0,0},{"P01",10,0}},options,testForward()).ok());
    CHECK(!planMultiPoseExperiment(scene,{{"P01",0,0}},options,testForward(),
        [](const Joint5&,std::string* e){if(e)*e="test obstacle";return false;}).ok());
    auto b=options;b.bounds.max[4]=5;
    CHECK(!planMultiPoseExperiment(scene,poses,b,testForward()).ok());
    if(argc==3&&std::string(argv[1])=="--write-examples"){
        std::filesystem::create_directories(argv[2]);
        std::ofstream summary(std::string(argv[2])+"/synthetic_experiment_pose_summary.csv");
        summary<<"pose_id,C_deg,B_deg,expected_profiles,scan_groups,profile_pitch_mm,hardware_authorized\n";
        for(const auto& v:plan.views){
            summary<<v.id<<","<<v.cDeg<<","<<v.bDeg<<","<<v.plannedProfiles<<","<<v.scanGroups<<","<<v.predictedProfilePitchMm<<",false\n";
            for(const int topo:{1,3}){
                M6NcPreviewOptions n;
                n.proposedTopologyId=topo;n.h3R1Edge=115;
                n.studyId="SYNTHETIC_TEST_DO_NOT_EXECUTE";n.scanGroupLabel="MP_"+v.id;
                const auto file=makeM6NcPreview(multiPoseAsOfflineM6(v,options),n);
                CHECK(file.ok());
                const auto out=std::string(argv[2])+"/SYNTHETIC_"+v.id+(topo==1?"_H1.nc":"_H3.nc");
                std::ofstream writer(out,std::ios::binary);writer<<file.text;writer.flush();CHECK(bool(writer));
            }
        }
        CHECK(bool(summary));
        M6NcPreviewOptions connectedSettings;
        connectedSettings.proposedTopologyId=3;
        connectedSettings.h3R1Edge=115;
        connectedSettings.studyId="SYNTHETIC_CONNECTED_DO_NOT_EXECUTE";
        connectedSettings.scanGroupLabel="SYNTHETIC_ALL";
        const auto joinedExample=makeM6NcPreview(connected.merged,connectedSettings);
        CHECK(joinedExample.ok());
        std::ofstream connectedOut(std::string(argv[2])+"/SYNTHETIC_CONNECTED_P01_P03_OFFLINE_ONLY_H3.nc",std::ios::binary);
        connectedOut<<joinedExample.text;
        connectedOut.flush();CHECK(bool(connectedOut));
    }
    std::cout<<"MultiPose offline experiment: 3 poses / NC H3 parse / virtual optics / guards PASS\n";
}
