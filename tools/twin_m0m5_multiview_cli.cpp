// M0/M1/M2/M5 offline source-aware real coverage. Does not call M3/M6/Qt.
// Reads only strictly verified M5S1 intermediate files produced by
// tools/m0m5_prepare_scans.py (CRC+per-exposure transform already checked).
#include "twin/TwinM5Coverage.h"
#include "twin/TwinH1Geometry.h"
#include "twin/TwinM51Chamfer.h"
#include "twin/TwinM52Geometry.h"
#include <algorithm>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>
using namespace mls::twin;
template<typename T> static bool fetch(std::istream& f,T& v){return bool(f.read(reinterpret_cast<char*>(&v),sizeof(T)));}
static void writeNum(std::ostream& s,double n){s<<std::setprecision(11)<<n;}
int main(int argc,char** argv){
    if(argc<11||std::string(argv[1])!="--obj"||std::string(argv[3])!="--pose"||std::string(argv[10])!="--sources"){
        std::cerr<<"usage: twin_m0m5_multiview_cli --obj NEW.obj --pose x y z rx ry rz --sources scan1.m5s1 ... --output prefix [--distance mm] [--pitch mm] [--frames N]\n";return 2;
    }
    TwinScene scene;std::string error;
    if(!scene.loadObjMesh(argv[2],&error)){std::cerr<<error<<'\n';return 3;}
    Pose6 pose;
    try{pose={std::stod(argv[4]),std::stod(argv[5]),std::stod(argv[6]),std::stod(argv[7]),std::stod(argv[8]),std::stod(argv[9])};}
    catch(...){std::cerr<<"bad pose\n";return 4;}
    scene.setEstimatedPose(pose);
    M5Options opt;opt.minDistinctProfilesPerCell=2;
    std::vector<std::string> sources;std::string prefix;
    for(int i=11;i<argc;++i){const std::string val=argv[i];
        if(val=="--output"&&i+1<argc)prefix=argv[++i];
        else if(val=="--distance"&&i+1<argc)opt.maxSurfaceDistanceMm=std::stod(argv[++i]);
        else if(val=="--pitch"&&i+1<argc)opt.cellPitchMm=std::stod(argv[++i]);
        else if(val=="--frames"&&i+1<argc)opt.minDistinctProfilesPerCell=std::stoul(argv[++i]);
        else if(val.rfind("--",0)==0){std::cerr<<"unknown option "<<val<<"\n";return 2;}
        else sources.push_back(val);
    }
    if(sources.empty()||prefix.empty()||sources.size()>16){std::cerr<<"bad sources/output\n";return 2;}
    std::vector<M5Report> reports;
    std::vector<M51Report> centerChamferReports;
    std::vector<M52Report> geometryReports;
    const Transform machineToCad=scene.cadToMachineEstimate().inverseRigid();
    for(const auto& filename:sources){
        std::ifstream input(filename,std::ios::binary);if(!input){std::cerr<<"cannot open "<<filename<<'\n';return 5;}
        char magic[4]{};std::uint64_t nframes=0,npoints=0;
        if(!input.read(magic,4)||std::string(magic,4)!="M5S1"||!fetch(input,nframes)||!fetch(input,npoints)||nframes==0||npoints>100000000ull){std::cerr<<"bad M5S1 header";return 5;}
        TwinM5CoverageBuilder builder(scene,opt);
        if(!builder.ready()){std::cerr<<builder.error()<<'\n';return 5;}
        TwinM51ChamferAnalyzer chamfer(scene);
        TwinM52GeometryAnalyzer geometry(scene);
        // H1 geometry-specific analysis is optional for generic imported parts.
        const bool hasChamfer=chamfer.ready(),hasGeometry=geometry.ready();
        for(std::uint64_t j=0;j<npoints;++j){
            std::uint32_t frame=0;double x=0,y=0,z=0;
            if(!fetch(input,frame)||!fetch(input,x)||!fetch(input,y)||!fetch(input,z)||!frame||frame>nframes){std::cerr<<"bad real sample";return 6;}
            const Vec3 point{x,y,z};
            builder.addRealMachine(point,frame);
            if(hasChamfer||hasGeometry){
                const Vec3 cad=machineToCad.point(point);
                if(hasChamfer)chamfer.addRealCad(cad);
                if(hasGeometry)geometry.addRealCad(cad);
            }
        }
        if(input.peek()!=std::char_traits<char>::eof()){std::cerr<<"unexpected trailing bytes";return 6;}
        auto report=std::move(builder).finish();
        if(!report.ok()){std::cerr<<"coverage error: "<<report.error;return 7;}
        std::cerr<<filename<<" points="<<npoints<<" accepted="<<report.realAccepted<<" noCAD="<<report.realNoAssociation<<" covered="<<report.realCoverage()*100<<"%\n";
        reports.push_back(std::move(report));
        centerChamferReports.push_back(std::move(chamfer).finish());
        geometryReports.push_back(std::move(geometry).finish());
    }
    const auto& base=reports.front();
    std::ofstream csv(prefix+"_cells.csv"),js(prefix+"_report.json");
    if(!csv||!js){std::cerr<<"cannot write result files\n";return 8;}
    csv<<"cell_id,cad_x,cad_y,cad_z,nx,ny,nz,area_mm2,feature_id,target,state,observed_sessions,total_hits,total_profiles";
    for(std::size_t s=0;s<sources.size();++s)csv<<",hits_"<<s<<",profiles_"<<s;
    csv<<'\n';
    double unionSessionArea=0,unionPooledArea=0,sparseArea=0,missingArea=0,excluded=0;
    std::vector<double> perFeature(base.features.size(),0.0), missingFeature(base.features.size(),0.0);
    std::uint64_t cellCovered=0;
    for(std::size_t i=0;i<base.cells.size();++i){
        const auto& cell=base.cells[i];
        unsigned nSessions=0;std::uint64_t allHits=0,allFrames=0;
        for(auto& r:reports){
            if(r.cells.size()!=base.cells.size()||r.cells[i].sourceTriangle!=cell.sourceTriangle){std::cerr<<"inconsistent CAD cells";return 9;}
            const auto& c=r.cells[i];allHits+=c.realHits;allFrames+=c.realDistinctProfiles;
            if(c.realHits>=opt.minHitsPerCell&&c.realDistinctProfiles>=opt.minDistinctProfilesPerCell)++nSessions;
        }
        int state=0; // 0 missing; 1 measured but insufficient; 2 covered; 3 excluded
        if(!cell.target){state=3;excluded+=cell.areaMm2;}
        else{
            perFeature[cell.featureId]+=cell.areaMm2;
            if(nSessions)unionSessionArea+=cell.areaMm2;
            if(allHits>=opt.minHitsPerCell&&allFrames>=opt.minDistinctProfilesPerCell){
                state=2;unionPooledArea+=cell.areaMm2;++cellCovered;
            }else if(allHits){state=1;sparseArea+=cell.areaMm2;missingFeature[cell.featureId]+=cell.areaMm2;}
            else {state=0;missingArea+=cell.areaMm2;missingFeature[cell.featureId]+=cell.areaMm2;}
        }
        csv<<i<<','<<cell.cadCenter.x<<','<<cell.cadCenter.y<<','<<cell.cadCenter.z<<','<<cell.cadNormal.x<<','<<cell.cadNormal.y<<','<<cell.cadNormal.z<<','<<cell.areaMm2<<','<<cell.featureId<<','<<int(cell.target)<<','<<state<<','<<nSessions<<','<<allHits<<','<<allFrames;
        for(auto& r:reports)csv<<','<<r.cells[i].realHits<<','<<r.cells[i].realDistinctProfiles;
        csv<<'\n';
    }
    js<<std::fixed<<std::setprecision(9)<<"{\n\"warning\":\"Geometric CAD association ONLY; not certified completeness. Missing points have unknown cause.\",\n";
    js<<"\"cell_pitch_mm\":"<<base.effectiveCellPitchMm<<",\"association_mm\":"<<opt.maxSurfaceDistanceMm<<",\"min_hits\":"<<opt.minHitsPerCell<<",\"min_distinct_exposures\":"<<opt.minDistinctProfilesPerCell<<",\n";
    js<<"\"cad_pose_xyz_rxryrz\":["<<pose.x<<','<<pose.y<<','<<pose.z<<','<<pose.rxDeg<<','<<pose.ryDeg<<','<<pose.rzDeg<<"],\n";
    const auto geom=inferH1Geometry(scene);
    js<<"\"h1_geometry_verified\":"<<(geom.valid?"true":"false")<<",\"center_hole_outer_radius_mm\":"<<geom.outerRadius<<",\"center_hole_inner_radius_mm\":"<<geom.innerRadius<<",\"center_hole_chamfer_depth_mm\":"<<geom.chamferDepth<<",\n";
    js<<"\"target_area_mm2\":"<<base.targetAreaMm2<<",\"observed_area_all_sessions_mm2\":"<<unionSessionArea<<",\"observed_area_pooled_mm2\":"<<unionPooledArea<<",\"sparse_area_mm2\":"<<sparseArea<<",\"no_hits_area_mm2\":"<<missingArea<<",\"excluded_area_mm2\":"<<excluded<<",\"pooled_coverage\":"<<unionPooledArea/base.targetAreaMm2<<",\n";
    js<<"\"sessions\":[\n";
    for(std::size_t i=0;i<reports.size();++i){const auto& r=reports[i];
       if(i)js<<",\n";
       js<<"{\"source\":\""<<sources[i]<<"\",\"points_seen\":"<<r.realPointsSeen<<",\"points_accepted\":"<<r.realAccepted<<",\"no_cad_association\":"<<r.realNoAssociation<<",\"excluded_points\":"<<r.realExcluded<<",\"real_coverage\":"<<r.realCoverage()<<",\"real_area_mm2\":"<<r.realAreaMm2;
       const auto& h=centerChamferReports[i];
       if(h.ok())js<<",\"m51_chamfer_geometry_outer_radius_mm\":"<<h.nominalOuterRadiusMm
                    <<",\"m51_chamfer_real_sparse_angles\":"<<h.realLowDensityAngles
                    <<",\"m51_upper_cone_residual_median_mm\":"<<h.realConeEquationMedianMm
                    <<",\"m51_upper_cone_residual_samples\":"<<h.realConeResidualSamples;
       const auto& g=geometryReports[i];
       js<<",\"m52_geometry_fit_ready\":"<<(g.real.ok?"true":"false");
       if(g.real.ok)js<<",\"m52_top_at_center_hole_mm\":"<<g.real.topOffsetAtHoleMm
                       <<",\"m52_top_plane_mad_mm\":"<<g.real.topPlaneMadMm
                       <<",\"m52_upper_cone_after_top_median_mm\":"<<g.real.coneAfterTopMedianMm
                       <<",\"m52_unobserved_arc_deg\":"<<g.real.maxUnobservedArcDeg;
       js<<"}";
    }
    js<<"],\n\"features\":[\n";
    for(std::size_t i=0;i<base.features.size();++i){if(i)js<<",\n";js<<"{\"id\":"<<i<<",\"name\":\""<<base.features[i].name<<"\",\"target_area_mm2\":"<<perFeature[i]<<",\"uncovered_or_sparse_area_mm2\":"<<missingFeature[i]<<"}";}
    js<<"]\n}\n";
    std::cout<<"M0-M5 multiview real coverage "<<sources.size()<<" scans, "<<base.cells.size()<<" CAD cells, pooled="<<unionPooledArea/base.targetAreaMm2*100<<"%, missing="<<missingArea<<" mm2, sparse="<<sparseArea<<" mm2\n";
}
