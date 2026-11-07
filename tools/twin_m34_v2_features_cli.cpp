// M3/M4 V2 OFFLINE optical geometry diagnostic.
// Extends (does not replace) VirtualLaser2950; NO machine control or hardware claims.
// M4L2 is an approximate CAD-proximity sidecar, NOT a ground-truth fixture segmentation.
#include "twin/VirtualLaser2950.h"
#include "twin/TwinM34ReturnModel.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <limits>
#include <vector>
using namespace mls::twin;
template<class T> static bool read(std::istream& f,T& x){return bool(f.read(reinterpret_cast<char*>(&x),sizeof(x)));}
struct Frame {VirtualProfilePose pose;std::vector<std::uint8_t> labels;std::vector<double> rx,rz;};
struct Session {std::string file;std::vector<Frame> frames;};
// Frozen-model whole-session streaming evaluator: avoids multi-GB intermediate
// CSV when validating every profile. Same CAD-relative evaluation as Python.
struct ScoreAcc {
  std::uint64_t n=0,tp=0,fp=0,fn=0,tn=0,paired=0;
  double brier=0,absZ=0;
  void add(bool real,double probability,bool hasZ,double realZ,double predZ){
    ++n; probability=std::max(0.0,std::min(1.0,probability));
    const bool pred=probability>=0.5;
    if(pred&&real)++tp;else if(pred)++fp;else if(real)++fn;else ++tn;
    brier+=(probability-double(real))*(probability-double(real));
    if(real&&hasZ&&std::isfinite(realZ)&&std::isfinite(predZ)){
      ++paired;absZ+=std::abs(realZ-predZ);
    }
  }
};
static bool load(const std::string& name,Session& session){
  std::ifstream in(name,std::ios::binary),mask(name.substr(0,name.find_last_of('.'))+".m4l2",std::ios::binary);
  if(!in||!mask)return false;
  char m4[4]{},l2[4]{};std::uint64_t frames=0,labels=0;
  if(!in.read(m4,4)||!mask.read(l2,4)||std::string(m4,4)!="M4S1"||std::string(l2,4)!="M4L2"||!read(in,frames)||!read(mask,labels)||frames!=labels||frames>5000)return false;
  session.file=name;
  for(std::uint64_t k=0;k<frames;++k){
    Frame row;std::uint32_t n=0;std::uint64_t o=0;
    if(!read(in,row.pose.sourceOrdinal)||!read(in,row.pose.elapsedSeconds))return false;
    for(double& v:row.pose.sensorToMachine.m)if(!read(in,v))return false;
    if(!read(in,n)||n>1280||!read(mask,o)||o!=row.pose.sourceOrdinal)return false;
    row.labels.resize(1280);row.rx.assign(1280,0);row.rz.assign(1280,0);
    if(!mask.read(reinterpret_cast<char*>(row.labels.data()),1280))return false;
    std::vector<std::uint8_t> seen(1280,0);
    for(std::uint32_t i=0;i<n;++i){std::uint32_t idx=0;double x=0,z=0;
      if(!read(in,idx)||!read(in,x)||!read(in,z)||idx>=1280||seen[idx]++||!std::isfinite(x)||!std::isfinite(z)||row.labels[idx]==0)return false;
      row.rx[idx]=x;row.rz[idx]=z;
    }
    session.frames.push_back(std::move(row));
  }
  return in.peek()==std::char_traits<char>::eof()&&mask.peek()==std::char_traits<char>::eof();
}
static bool roiHole(const VirtualProfilePose& pose,int idx,const Sensor2950Geometry& g,const Transform& invCad,double topZ){
    const Vec3 o=invCad.point(pose.sensorToMachine.point(g.laserOriginSensor));
    const Vec3 t=pose.sensorToMachine.vector(normalized(Vec3{g.sampleXAtZ(idx,g.zCenter),0,g.zCenter}-g.laserOriginSensor));
    const Vec3 d=invCad.vector(t);
    if(std::abs(d.z)<1e-10)return false;
    const double lambda=(topZ-o.z)/d.z;
    if(lambda<=0||lambda>g.maxTraceDistanceMm)return false;
    const Vec3 p=o+d*lambda;
    for(double x:{-7.5,7.5})for(double y:{17.5,32.5})if(std::hypot(p.x-x,p.y-y)<=4.2)return true;
    return false;
}
int main(int argc,char** argv){
    std::string obj,out,parameterFile,summaryOut;std::array<double,6> pose{};
    double cameraX=0,cameraY=90,cameraZ=251.25,fanScale=1.,fanOffset=0.;std::vector<std::string> sources;
    try{
      for(int i=1;i<argc;++i){std::string arg=argv[i];
        if(arg=="--obj"&&i+1<argc)obj=argv[++i];
        else if(arg=="--pose"&&i+6<argc)for(auto& v:pose)v=std::stod(argv[++i]);
        else if(arg=="--camera-x"&&i+1<argc)cameraX=std::stod(argv[++i]);
        else if(arg=="--camera-y"&&i+1<argc)cameraY=std::stod(argv[++i]);
        else if(arg=="--camera-z"&&i+1<argc)cameraZ=std::stod(argv[++i]);
        else if(arg=="--fan-scale"&&i+1<argc)fanScale=std::stod(argv[++i]);
        else if(arg=="--fan-offset"&&i+1<argc)fanOffset=std::stod(argv[++i]);
        else if(arg=="--source"&&i+1<argc)sources.push_back(argv[++i]);
        else if(arg=="--output"&&i+1<argc)out=argv[++i];
        else if(arg=="--params"&&i+1<argc)parameterFile=argv[++i];
        else if(arg=="--summary-output"&&i+1<argc)summaryOut=argv[++i];
        else {std::cerr<<"Invalid option: "<<arg<<'\n';return 2;}
      }
    }catch(...){std::cerr<<"Invalid numeric argument\n";return 2;}
    if(obj.empty()||(out.empty()&&summaryOut.empty())||sources.empty()||!std::isfinite(cameraY)||!std::isfinite(cameraZ)||!std::isfinite(cameraX))return 2;
    TwinScene scene;std::string err;
    if(!scene.loadObjMesh(obj,&err)){std::cerr<<err;return 3;}
    scene.setEstimatedPose({pose[0],pose[1],pose[2],pose[3],pose[4],pose[5]});
    Sensor2950Geometry geom;
    geom.cameraOriginSensor={cameraX,cameraY,cameraZ};
    geom.sampleXScale=fanScale;geom.sampleXOffsetMm=fanOffset;
    VirtualLaser2950 laser(scene,geom,true);
    if(!laser.ready(&err)){std::cerr<<err;return 4;}
    M34ReturnParams fit;
    if(!parameterFile.empty()){
        if(!TwinM34ReturnModel::readResearchParameters(parameterFile,fit,&err)){
            std::cerr<<"Invalid M34P2 parameters: "<<err;return 8;
        }
        if(std::abs(fit.equivalentReceiverSensor.x-cameraX)>1e-6||
           std::abs(fit.equivalentReceiverSensor.y-cameraY)>1e-6||
           std::abs(fit.equivalentReceiverSensor.z-cameraZ)>1e-6||
           std::abs(fit.fanXScale-fanScale)>1e-6||
           std::abs(fit.fanXOffsetMm-fanOffset)>1e-6){
            std::cerr<<"M34 parameter receiver mismatch";return 9;
        }
    }
    const TwinM34ReturnModel returnModel(fit,geom);
    const auto cadInverse=scene.cadToMachineEstimate().inverseRigid();
    const bool summaryOnly=!summaryOut.empty();
    std::ofstream f;
    if(!summaryOnly){
      f.open(out);
      if(!f){std::cerr<<"Unable to write features";return 5;}
      f<<"session,frame,index,label,hole,geometry_valid,hit_workpiece,hit_support,state,real_z,pred_z,cos_laser,cos_receiver,depth_risk,field_risk,boundary_risk,hit_cad_x,hit_cad_y,hit_cad_z,face_id,v2_probability,specular_alignment\n";
      f<<std::fixed<<std::setprecision(6);
    }
    std::array<ScoreAcc,4> summaryStats{}; // overall geometry / model, hole geometry / model
    std::uint64_t cadFarCount=0,ambiguousCount=0;
    std::size_t frameCount=0;
    for(std::size_t si=0;si<sources.size();++si){
      Session session;if(!load(sources[si],session)){std::cerr<<"invalid M4S1/M4L2 pair "<<sources[si]<<'\n';return 6;}
      for(const auto& frame:session.frames){
        const auto sim=laser.simulate(frame.pose);
        if(sim.samples.size()!=1280){std::cerr<<"Invalid simulated profile";return 7;}
        std::array<double,1280> boundary{};
        // Near a transition the finite-width laser line can mix foreground and
        // background. This is an uncalibrated geometric proxy, not a spot PSF.
        for(int i=0;i<1280;++i){
          int nearest=99;
          for(int j=std::max(0,i-7);j<=std::min(1279,i+7);++j){
            if(sim.samples[std::size_t(j)].state!=VirtualSampleState::Valid){nearest=std::min(nearest,std::abs(i-j));}
          }
          boundary[std::size_t(i)]=nearest<=7?std::exp(-nearest/2.):0.;
        }
        for(int i=0;i<1280;++i){
          const auto& s=sim.samples[std::size_t(i)];
          const double z=s.rayHitSensorZ;
          const double x=s.rayHitSensorX;
          const double depth=std::isfinite(z)?std::abs(z-geom.zCenter)/(geom.zFar-geom.zCenter):0.;
          const double edge=std::isfinite(z)&&std::isfinite(x)?std::abs(x)/(geom.widthAtZ(z)*.5):0.;
          const bool isHole=roiHole(frame.pose,i,geom,cadInverse,scene.mesh().bounds.max.z);
          const int observed=frame.labels[std::size_t(i)];
          if(observed==2)++cadFarCount;
          if(observed==3)++ambiguousCount;
          if(summaryOnly){
            const bool workpieceHit=s.hitObject==VirtualHitObject::Workpiece;
            if((observed==0||observed==1)&&(workpieceHit||observed==1)){
              const bool valid=s.state==VirtualSampleState::Valid && workpieceHit;
              const double pr=valid?(parameterFile.empty()?1.:returnModel.probability(sim,std::size_t(i),isHole)):0.;
              summaryStats[0].add(observed==1,valid?1.:0.,valid,frame.rz[std::size_t(i)],s.sensorZ);
              summaryStats[1].add(observed==1,pr,valid,frame.rz[std::size_t(i)],s.sensorZ);
              if(isHole){
                summaryStats[2].add(observed==1,valid?1.:0.,valid,frame.rz[std::size_t(i)],s.sensorZ);
                summaryStats[3].add(observed==1,pr,valid,frame.rz[std::size_t(i)],s.sensorZ);
              }
            }
            continue;
          }
          f<<si<<','<<frame.pose.sourceOrdinal<<','<<i<<','<<int(frame.labels[std::size_t(i)])<<','
           <<int(roiHole(frame.pose,i,geom,cadInverse,scene.mesh().bounds.max.z))<<','
           <<int(s.state==VirtualSampleState::Valid)<<','
           <<int(s.hitObject==VirtualHitObject::Workpiece)<<','
           <<int(s.hitObject==VirtualHitObject::Support)<<','<<int(s.state)<<',';
          if(frame.labels[std::size_t(i)]!=0)f<<frame.rz[std::size_t(i)];else f<<"nan";
          f<<',';
          if(s.state==VirtualSampleState::Valid)f<<s.sensorZ;else f<<"nan";
          f<<','<<s.laserNormalCos<<','<<s.receiverNormalCos<<','<<depth<<','<<edge<<','<<boundary[std::size_t(i)];
          if(s.hitObject!=VirtualHitObject::None){const auto pCad=cadInverse.point(s.hitPositionMachine);
            f<<','<<pCad.x<<','<<pCad.y<<','<<pCad.z<<','<<s.cadFaceId;
          }else f<<",nan,nan,nan,0";
          if(parameterFile.empty())f<<",nan";
          else f<<','<<returnModel.probability(sim,std::size_t(i),
                    roiHole(frame.pose,i,geom,cadInverse,scene.mesh().bounds.max.z));
          f<<','<<s.specularReceiverAlignment<<'\n';
        }
        ++frameCount;
      }
    }
    if(summaryOnly){
      std::ofstream so(summaryOut);
      if(!so){std::cerr<<"Unable to write summary";return 10;}
      so<<"scope,model,samples,TP,FP,FN,TN,F1,Brier,ZPaired,MAE_Z_mm,CADFarExcluded,AmbiguousExcluded\n";
      so<<std::fixed<<std::setprecision(10);
      const std::array<const char*,4> scopes={"workpiece","workpiece","holes","holes"};
      const std::array<const char*,4> modes={"geometry","probability","geometry","probability"};
      for(std::size_t i=0;i<4;++i){const auto& v=summaryStats[i];
        const double denom=double(2*v.tp+v.fp+v.fn);
        so<<scopes[i]<<','<<modes[i]<<','<<v.n<<','<<v.tp<<','<<v.fp<<','<<v.fn<<','<<v.tn<<','
          <<(denom>0?2*v.tp/denom:0.)<<','<<(v.n?v.brier/double(v.n):0.)<<','<<v.paired<<','
          <<(v.paired?v.absZ/double(v.paired):0.)<<','<<cadFarCount<<','<<ambiguousCount<<'\n';
      }
    }
    std::cerr<<"M34 V2 geometry features: "<<frameCount<<" real profiles, "<<frameCount*1280
       <<" indexed samples, "<<(summaryOnly?"summary=":"output=")<<(summaryOnly?summaryOut:out)<<'\n';
    return 0;
}
