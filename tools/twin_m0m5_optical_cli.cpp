// Multi-posture M3/M4 optical diagnostic on recorded SCAN v3 indexed profiles.
// Candidate search is only one equivalent camera-Y parameter; trained on
// scan 02/04, externally checked on 05/06. Never authorizes hardware.
#include "twin/TwinM4Calibration.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>
using namespace mls::twin;
template<typename T> static bool rd(std::istream& f,T& x){return bool(f.read(reinterpret_cast<char*>(&x),sizeof x));}
struct Session {std::string filename;std::vector<M4CalibrationFrame> frames;};
static bool load(const std::string& p,Session& session){
    session.filename=p;std::ifstream f(p,std::ios::binary);
    char magic[4]{};std::uint64_t frames=0;
    if(!f.read(magic,4)||std::string(magic,4)!="M4S1"||!rd(f,frames)||frames==0||frames>1000)return false;
    for(std::uint64_t i=0;i<frames;++i){
        M4CalibrationFrame frame;std::uint32_t n=0;
        if(!rd(f,frame.pose.sourceOrdinal)||!rd(f,frame.pose.elapsedSeconds))return false;
        for(double& val:frame.pose.sensorToMachine.m)if(!rd(f,val))return false;
        if(!rd(f,n)||n>1280)return false;
        frame.observed.reserve(n);
        for(std::uint32_t j=0;j<n;++j){M4MeasuredSample a;
            if(!rd(f,a.index)||!rd(f,a.sensorX)||!rd(f,a.sensorZ)||a.index>=1280||!std::isfinite(a.sensorX)||!std::isfinite(a.sensorZ))return false;
            frame.observed.push_back(a);
        }
        session.frames.push_back(std::move(frame));
    }
    return f.peek()==std::char_traits<char>::eof();
}
static void metrics(std::ostream& j,const M4Metrics& r){
    j<<"{\"f1\":"<<r.f1()<<",\"iou\":"<<r.iou()<<",\"mean_abs_sensor_z_mm\":"<<r.meanAbsZ()<<",\"paired\":"<<r.pairedSamples<<",\"real_valid\":"<<r.realValid()<<",\"simulated_valid\":"<<r.simulatedValid()<<"}";
}
int main(int argc,char** argv){
    if(argc<16||std::string(argv[1])!="--obj"||std::string(argv[3])!="--pose"||std::string(argv[10])!="--train"){
        std::cerr<<"usage: twin_m0m5_optical_cli --obj NEW.obj --pose x y z rx ry rz --train 02.m4s1 04.m4s1 --holdout 05.m4s1 06.m4s1 --output file.json\n";return 2;
    }
    std::vector<Session> train,holdout;
    std::string output;enum class Group{Train,Holdout,None};Group group=Group::Train;
    for(int k=11;k<argc;++k){const std::string x=argv[k];
        if(x=="--holdout"){group=Group::Holdout;continue;}
        if(x=="--output"&&k+1<argc){output=argv[++k];group=Group::None;continue;}
        if(x.rfind("--",0)==0){std::cerr<<"unknown CLI option";return 2;}
        Session s;if(!load(x,s)){std::cerr<<"bad M4 source: "<<x<<'\n';return 3;}
        if(group==Group::Train)train.push_back(std::move(s));
        else if(group==Group::Holdout)holdout.push_back(std::move(s));
    }
    if(train.size()<2||holdout.empty()||output.empty()){std::cerr<<"need independent training+holdout SCAN sessions";return 2;}
    TwinScene scene;std::string err;
    if(!scene.loadObjMesh(argv[2],&err)){std::cerr<<err<<'\n';return 4;}
    Pose6 pose{};
    try{pose={std::stod(argv[4]),std::stod(argv[5]),std::stod(argv[6]),std::stod(argv[7]),std::stod(argv[8]),std::stod(argv[9])};}
    catch(...){return 2;}
    scene.setEstimatedPose(pose); // M3/M4 must NOT access scene's unobserved truePose
    const Sensor2950Geometry defaultModel;
    auto all=train;all.insert(all.end(),holdout.begin(),holdout.end());
    const std::array<double,9> candidates{{-75,-50,-25,0,25,50,75,90,100}};
    struct Candidate{double Y=0,score=0,hold=0;std::vector<M4Report> reports;};
    std::vector<Candidate> results;
    for(double baseline:candidates){
        Sensor2950Geometry model=defaultModel;model.cameraOriginSensor.y=baseline;
        VirtualLaser2950 scanner(scene,model,true);
        if(!scanner.ready(&err)){std::cerr<<err<<'\n';return 5;}
        Candidate c;c.Y=baseline;
        for(std::size_t si=0;si<all.size();++si){
            auto ctx=h1ReferenceM4Context(scene,model,true);
            TwinM4Accumulator acc(ctx);
            for(const auto& f:all[si].frames){
                const auto v=scanner.simulate(f.pose);
                if(!acc.addFrame(v,f.observed)){std::cerr<<"M4 indexing error";return 6;}
            }
            auto result=std::move(acc).finish();
            if(!result.ok()){std::cerr<<"M4 compare failed: "<<result.error;return 7;}
            const double roiScore=result.holeRegions.realValid()>=20?result.holeRegions.f1():result.global.f1();
            const double score=.65*roiScore+.35*result.global.f1();
            if(si<train.size())c.score+=score/double(train.size());
            else c.hold+=score/double(holdout.size());
            c.reports.push_back(std::move(result));
        }
        std::cerr<<"cameraY="<<baseline<<" training="<<c.score<<" holdout="<<c.hold<<'\n';
        results.push_back(std::move(c));
    }
    const auto bestTrain=std::max_element(results.begin(),results.end(),[](const Candidate& a,const Candidate& b){return a.score<b.score;});
    const auto bestHoldout=std::max_element(results.begin(),results.end(),[](const Candidate& a,const Candidate& b){return a.hold<b.hold;});
    bool holdoutQuality=true;
    for(std::size_t k=train.size();k<all.size();++k){
        const auto& r=bestTrain->reports[k];
        if(r.global.f1()<0.85||r.holeRegions.f1()<0.80||
           r.holeRegions.meanAbsZ()>0.20)holdoutQuality=false;
    }
    std::ofstream o(output);if(!o){std::cerr<<"cannot output";return 8;}
    o<<std::fixed<<std::setprecision(7)<<"{\"model\":\"M3 geometric dual-ray approximate camera Sensor-Y only\",\"training_sessions\":"<<train.size()<<",\"holdout_sessions\":"<<holdout.size()<<",\"optical_physical_calibration_certified\":false,\"holdout_quality_gate_pass\":"<<(holdoutQuality?"true":"false")<<",\"ready_for_m6_optical_model\":false,";
    o<<"\"candidate_selected_train_camera_y_mm\":"<<bestTrain->Y<<",\"candidate_best_holdout_camera_y_mm\":"<<bestHoldout->Y<<",\"train_holdout_agreement\":"<<(bestTrain->Y==bestHoldout->Y?"true":"false")<<",\"candidates\":[";
    for(std::size_t i=0;i<results.size();++i){const auto& c=results[i];if(i)o<<',';
        o<<"{\"camera_y_mm\":"<<c.Y<<",\"training_score\":"<<c.score<<",\"holdout_score\":"<<c.hold<<",\"sessions\":[";
        for(std::size_t k=0;k<all.size();++k){if(k)o<<',';
            o<<"{\"path\":\""<<all[k].filename<<"\",\"split\":\""<<(k<train.size()?"train":"holdout")<<"\",\"profiles\":"<<c.reports[k].profilesCompared<<",\"global\":";
            metrics(o,c.reports[k].global);o<<",\"hole_region\":";metrics(o,c.reports[k].holeRegions);o<<'}';
        }
        o<<"]}";
    }
    o<<"]}\n";
    std::cout<<"M3/M4 multi-posture validation: training best Y="<<bestTrain->Y<<"mm; independent holdout best Y="<<bestHoldout->Y<<"mm; no physical calibration certified\n";
}
