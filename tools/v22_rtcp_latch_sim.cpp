#include "runtime/Rtcp17.h"
#include "runtime/Acquisition5D.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

using namespace mls::runtime;
using namespace mls::calibration;
namespace {
Physical17Parameters truth(){
    Physical17Parameters p;
    p[Physical17Parameters::Sxy]=1.2e-4;p[Physical17Parameters::Sxz]=-1.8e-4;p[Physical17Parameters::Syz]=0.9e-4;
    p[Physical17Parameters::COffsetU]=1.8;p[Physical17Parameters::COffsetV]=-1.1;p[Physical17Parameters::CTiltX]=2.2e-4;p[Physical17Parameters::CTiltY]=-1.4e-4;
    p[Physical17Parameters::BOffsetU]=-0.7;p[Physical17Parameters::BOffsetV]=2.3;p[Physical17Parameters::BTiltX]=-1.5e-4;p[Physical17Parameters::BTiltZ]=1.9e-4;
    p[Physical17Parameters::HandEyeRx]=0.012;p[Physical17Parameters::HandEyeRy]=-0.018;p[Physical17Parameters::HandEyeRz]=0.007;
    p[Physical17Parameters::HandEyeTx]=12.0;p[Physical17Parameters::HandEyeTy]=-8.0;p[Physical17Parameters::HandEyeTz]=96.0;
    return p;
}
double d(const Vec3d&a,const Vec3d&b){return std::hypot(std::hypot(a.x-b.x,a.y-b.y),a.z-b.z);}
Joint lerp(const Joint&a,const Joint&b,double u){Joint q{};for(int i=0;i<5;++i)q[i]=a[i]+u*(b[i]-a[i]);return q;}
void require(bool ok,const std::string& msg){if(!ok)throw std::runtime_error(msg);}
}
int main(int argc,char**argv){try{
    const auto p=truth();Joint start{20,-15,5,-25,-18};const auto p0=Rtcp17Model::controlledPointInMachine(start,p);
    Rtcp17Options opt;opt.tcpChordToleranceMm=0.001;opt.maxRotaryStepDeg=0.15;opt.maxLinearJointStepMm=0.4;
    Rtcp17Planner planner(opt);Vec3d end{p0.x+40,p0.y+25,p0.z-12};auto line=planner.planLine(start,end,35,22,12,p);require(line.success,line.message);
    Joint prev=start;Vec3d prevP=p0;double lineSampleErr=0;
    for(const auto&w:line.waypoints){for(double u:{.125,.25,.5,.75,.875}){auto q=lerp(prev,w.q,u);Vec3d target{prevP.x+u*(w.targetMachineMm.x-prevP.x),prevP.y+u*(w.targetMachineMm.y-prevP.y),prevP.z+u*(w.targetMachineMm.z-prevP.z)};lineSampleErr=std::max(lineSampleErr,d(Rtcp17Model::controlledPointInMachine(q,p),target));}prev=w.q;prevP=w.targetMachineMm;}
    Joint lineEnd=line.waypoints.back().q;double naiveLine=0;for(int k=0;k<=1000;++k){double u=k/1000.;auto q=lerp(start,lineEnd,u);Vec3d target{p0.x+u*(end.x-p0.x),p0.y+u*(end.y-p0.y),p0.z+u*(end.z-p0.z)};naiveLine=std::max(naiveLine,d(Rtcp17Model::controlledPointInMachine(q,p),target));}

    auto orient=planner.planLineTimed(start,p0,32,24,3.0,p);require(orient.success,orient.message);prev=start;double orientErr=0;
    for(const auto&w:orient.waypoints){for(double u:{.125,.25,.5,.75,.875})orientErr=std::max(orientErr,d(Rtcp17Model::controlledPointInMachine(lerp(prev,w.q,u),p),p0));prev=w.q;}
    const auto oq=orient.waypoints.back().q;double comp=std::hypot(std::hypot(oq[0]-start[0],oq[1]-start[1]),oq[2]-start[2]);
    double naiveOrient=0;for(int k=0;k<=1000;++k){double u=k/1000.;Joint q=start;q[3]=start[3]+u*(32-start[3]);q[4]=start[4]+u*(24-start[4]);naiveOrient=std::max(naiveOrient,d(Rtcp17Model::controlledPointInMachine(q,p),p0));}

    // Latch model: all five coordinates are sampled at t0.  The "sequential"
    // comparison below uses arbitrary 20..100 us offsets only to show sensitivity;
    // they are NOT measured ECI3808 timing numbers.
    const Joint qEdge{40.1234567,10.2345678,2.3456789,12.3456789,-8.7654321};const Joint velocity{10,-4,2,30,-20};const std::array<double,2> offsets{3,-5};
    float rec[Table5D::dimension]={42,float(qEdge[0]),float(qEdge[1]),float(qEdge[2]),float(qEdge[3]-offsets[0]),float(qEdge[4]-offsets[1])};
    Frame5D latched;std::string err;require(decodeRecord(rec,42,7,offsets,latched,err),err);require(completeFrame(latched,p.value,err),err);
    double latchJointQuant=0;for(int j=0;j<5;++j)latchJointQuant=std::max(latchJointQuant,std::abs(latched.joints[j]-qEdge[j]));
    const auto exactTcp=Rtcp17Model::controlledPointInMachine(qEdge,p);const auto latchTcp=Rtcp17Model::controlledPointInMachine(latched.joints,p);double latchTcpErr=d(exactTcp,latchTcp);
    const double delays[5]={20e-6,40e-6,60e-6,80e-6,100e-6};Joint mixed=qEdge;for(int j=0;j<5;++j)mixed[j]+=velocity[j]*delays[j];double hypotheticalSequentialTcpErr=d(exactTcp,Rtcp17Model::controlledPointInMachine(mixed,p));

    std::ostringstream o;o<<std::setprecision(17)
      <<"{\n"
      <<"  \"test\": \"V22 physical17 RTCP + common-edge latch numerical simulation\",\n"
      <<"  \"hardware_tested\": false,\n"
      <<"  \"zdevelop_compiled\": false,\n"
      <<"  \"latch_input\": "<<Table5D::latchInput<<",\n"
      <<"  \"trigger_output\": "<<Table5D::triggerOutput<<",\n"
      <<"  \"latch_mode\": "<<Table5D::latchMode<<",\n"
      <<"  \"rtcp_line_segments\": "<<line.waypoints.size()<<",\n"
      <<"  \"rtcp_line_sampled_max_error_mm\": "<<lineSampleErr<<",\n"
      <<"  \"naive_joint_line_max_error_mm\": "<<naiveLine<<",\n"
      <<"  \"pure_orientation_segments\": "<<orient.waypoints.size()<<",\n"
      <<"  \"pure_orientation_rtcp_max_error_mm\": "<<orientErr<<",\n"
      <<"  \"pure_orientation_naive_max_drift_mm\": "<<naiveOrient<<",\n"
      <<"  \"pure_orientation_xyz_compensation_mm\": "<<comp<<",\n"
      <<"  \"latch_binary32_max_joint_quantization\": "<<latchJointQuant<<",\n"
      <<"  \"latch_binary32_tcp_error_mm\": "<<latchTcpErr<<",\n"
      <<"  \"hypothetical_sequential_delays_us\": [20,40,60,80,100],\n"
      <<"  \"hypothetical_sequential_tcp_error_mm\": "<<hypotheticalSequentialTcpErr<<",\n"
      <<"  \"note\": \"Sequential delays are illustrative only; real OUT14-IN0-laser timing requires oscilloscope measurement.\"\n"
      <<"}\n";
    std::cout<<o.str();if(argc>1){std::ofstream f(argv[1]);require(bool(f),"cannot open summary output");f<<o.str();require(bool(f),"cannot write summary output");}
    require(lineSampleErr<.0025,"RTCP line tolerance failed");require(orientErr<.0025,"RTCP orientation-hold tolerance failed");require(naiveLine>.1&&naiveOrient>1,"simulation did not distinguish RTCP");return 0;
}catch(const std::exception&e){std::cerr<<"ERROR: "<<e.what()<<"\n";return 1;}}
