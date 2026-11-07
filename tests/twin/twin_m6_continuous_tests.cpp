#include "twin/TwinM6Continuous.h"
#include "twin/TwinM7Fusion.h"
#include <cassert>
#include <cmath>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <numeric>
#include <string>
using namespace mls::twin;
int main(int argc,char** argv){
    assert(argc>=2);
    TwinScene scene;std::string err;
    assert(scene.loadObjMesh(argv[1],&err));
    const Pose6 place{44.3195,10.2369,-225.05,0,0,-89.2265};
    scene.setTruePose(place);scene.setEstimatedPose(place);
    auto refJoint=Joint5{{0,0,0,0,0}};
    Transform ref;ref.m[3]=0;ref.m[7]=0;ref.m[11]=0;
    auto fk=makeM6NominalFromRecorded(refJoint,ref);
    const auto ident=fk(refJoint);
    for(int i=0;i<16;++i)assert(std::abs(ident.m[std::size_t(i)]-ref.m[std::size_t(i)])<1e-9);
    const auto later=fk({{4.,5.,6.,12.,-15.}});
    assert(std::abs(later.m[3]-4.)<10.); // coupled orientation is finite
    M5Options covOpt;covOpt.cellPitchMm=1.4;covOpt.maxCells=35000;
    TwinM5CoverageBuilder builder(scene,covOpt);
    assert(builder.ready());
    auto coverage=std::move(builder).finish();assert(coverage.ok());
    M6Options opt;opt.scanRateHz=25;opt.candidateEvaluationFrames=8;
    opt.maxSelectedSegments=1;opt.maxFullProfiles=2000;
    opt.maxDurationSec=150;opt.maxTotalDurationSec=155;
    opt.previewLimit=1000;opt.tiltAmplitudeDeg=15;opt.yawAmplitudeDeg=16;
    opt.exportPointStride=8;
    auto scan=TwinM6ContinuousPlanner(scene,coverage,Sensor2950Geometry{},fk,opt);
    const auto rawPath=std::filesystem::temp_directory_path()/"mls_m6_continuous_test_xyzfloat.bin";
    M6ProgressState progress;
    std::uint64_t archivedProfiles=0, archivedValid=0;
    std::size_t lastSegment=0;
    M6FullProfileSink sink=[&](const VirtualProfile& p,std::size_t segment,std::string* error){
        (void)error;
        assert(segment>=1 && segment>=lastSegment);
        lastSegment=segment;
        assert(p.samples.size()==1280);
        ++archivedProfiles;
        for(const auto& s:p.samples)if(s.state==VirtualSampleState::Valid)++archivedValid;
        return true;
    };
    const auto result=scan.run(nullptr,nullptr,rawPath.string(),&progress,sink);
    std::cout<<"M6 continuous offline: good="<<result.ok()<<" error="<<result.error
             <<" segments="<<result.segments.size()<<" profiles="<<result.fullProfiles
             <<" preview="<<result.pointPreview.size()<<" addedArea="<<result.newAreaMm2<<"\n";
    assert(result.ok());
    assert(progress.stage.load()==int(M6ProgressStage::Finished));
    assert(progress.candidatesDone.load()>0);
    assert(progress.candidatesTotal.load()>=progress.candidatesDone.load());
    assert(progress.refinementsDone.load()>0);
    assert(progress.candidateProfiles.load()>0);
    assert(result.candidateCurvesAttempted>0);
    assert(result.candidateProfilesSimulated>0);
    assert(progress.fullProfiles.load()==result.fullProfiles);
    assert(!result.segments.empty());
    assert(result.fullProfiles>10);
    assert(archivedProfiles==result.fullProfiles);
    assert(archivedValid>=result.fullValidPoints);
    assert(progress.archiveProfilesSaved.load()==result.fullProfiles);
    assert(progress.activeSegmentNumber.load()>0);
    assert(result.fullValidPoints>100);
    assert(result.exportedValidPoints==(result.fullValidPoints+opt.exportPointStride-1)/opt.exportPointStride);
    assert(std::filesystem::file_size(rawPath)==result.exportedValidPoints*3*sizeof(float));
    assert(progress.activeSegmentProfilesTotal.load()>0);
    assert(progress.activeSegmentProfilesDone.load()==progress.activeSegmentProfilesTotal.load());
    assert(!result.pathPreview.empty());
    const auto toCad=scene.cadToMachineEstimate().inverseRigid();
    bool focusNearWorkpiece=false;
    for(const auto& p:result.pathPreview){
        if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z))continue;
        const auto local=toCad.point(p);
        if(local.z>=-1.0&&local.z<=11.0){focusNearWorkpiece=true;break;}
    }
    assert(focusNearWorkpiece);
    std::filesystem::remove(rawPath);
    assert(result.newAreaMm2>0.);
    const auto sumFeatures=std::accumulate(result.featureNewAreaMm2.begin(),result.featureNewAreaMm2.end(),0.);
    assert(std::abs(sumFeatures-result.newAreaMm2)<1e-5);
    for(const auto& seg:result.segments){
        assert(seg.amplitudeScale>=0.35 && seg.amplitudeScale<=1.0);
        assert(seg.vmaxUsage<=1.0+1e-6);
        assert(seg.amaxUsage<=1.0+1e-6);
        assert(seg.jmaxUsage<=1.0+1e-6);
    }
    assert(!result.path.empty());
    assert(result.pointPreview.size()<=opt.previewLimit);
    assert(result.segmentPreviews.size()==result.segments.size());
    assert(result.firstCoveredSegmentByCell.size()==coverage.cells.size());
    double reopenedArea=0.;
    for(const auto id:result.reopenedSparseCellIds){
        assert(id<coverage.cells.size());
        reopenedArea+=coverage.cells[id].areaMm2;
    }
    assert(std::abs(reopenedArea-result.reopenedSparseChamferAreaMm2)<1e-5);
    std::size_t previewTotal=0;
    for(const auto& part:result.segmentPreviews){
        assert(part.observedSamples>0);
        assert(part.points.size()<=opt.previewLimit/opt.maxSelectedSegments);
        previewTotal+=part.points.size();
    }
    assert(previewTotal==result.pointPreview.size());
    double newlyCovered=0.;
    std::vector<double> areaBySegment(result.segments.size(),0.);
    for(std::size_t i=0;i<result.firstCoveredSegmentByCell.size();++i){
        const auto first=result.firstCoveredSegmentByCell[i];
        if(!first)continue;
        assert(first<=result.segments.size());
        newlyCovered+=coverage.cells[i].areaMm2;
        areaBySegment[first-1]+=coverage.cells[i].areaMm2;
    }
    assert(std::abs(newlyCovered-result.newAreaMm2)<1e-5);
    // M7 acceptance consumes the exact CAD cells produced by the true M6
    // continuous ray simulation, not an invented synthetic coverage mask.
    auto fusion=calculateM7Coverage(coverage,result,.95);
    assert(fusion.ok()&&fusion.areaConsistent);
    assert(std::abs(fusion.simulatedNewAreaMm2-result.newAreaMm2)<1e-4);
    assert(fusion.addedAreaBySegmentMm2.size()==result.segments.size());

    for(std::size_t i=0;i<result.segments.size();++i)
        assert(std::abs(areaBySegment[i]-result.segments[i].measuredNewAreaMm2)<1e-5);
    assert(!result.canExportForHardware);
    bool simultaneous=false;
    for(std::size_t k=1;k<result.path.size();++k){
        const auto& a=result.path[k-1];const auto& b=result.path[k];
        if(a.role!=M6SegmentRole::Scan||b.role!=M6SegmentRole::Scan)continue;
        const auto& p=a.joints;const auto& q=b.joints;
        if((std::abs(q[0]-p[0])+std::abs(q[1]-p[1])+std::abs(q[2]-p[2]))>1e-5 &&
            std::abs(q[3]-p[3])>1e-5 &&std::abs(q[4]-p[4])>1e-5){simultaneous=true;break;}
    }
    assert(simultaneous);
    // No collision checker must NEVER yield hardware-authorized trajectory.
    assert(!result.collisionModelUsed);
    auto reject=[](const Joint5&,std::string*){return false;};
    const auto denied=TwinM6ContinuousPlanner(scene,coverage,Sensor2950Geometry{},fk,opt,reject).run();
    assert(!denied.ok()||denied.segments.empty());
    std::atomic<bool> preCancelled{true};
    M6ProgressState cancelProgress;
    const auto stopped=scan.run(&preCancelled,nullptr,{},&cancelProgress);
    assert(stopped.cancelled && !stopped.ok());
    assert(cancelProgress.stage.load()==int(M6ProgressStage::Cancelled));
    M6Options tinyBudget=opt;
    tinyBudget.maxCandidateSearchSeconds=0.000000001;
    M6ProgressState budgetProgress;
    const auto timed=TwinM6ContinuousPlanner(scene,coverage,Sensor2950Geometry{},fk,tinyBudget)
        .run(nullptr,nullptr,{},&budgetProgress);
    assert(!timed.ok() && timed.candidateSearchTruncated);
    assert(budgetProgress.stage.load()==int(M6ProgressStage::Failed));
    std::cout<<"M6 bounded search, candidate progress, cancellation and continuous 5-axis PASS\n";
}
