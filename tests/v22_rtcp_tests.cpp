#include "runtime/Rtcp17.h"
#include "TestSupport.h"
#include <cmath>
#include <iostream>

using namespace mls::runtime;
using namespace mls::calibration;

namespace {
Physical17Parameters truth(){
    Physical17Parameters p;
    p[Physical17Parameters::Sxy]=1.2e-4;
    p[Physical17Parameters::Sxz]=-1.8e-4;
    p[Physical17Parameters::Syz]=0.9e-4;
    p[Physical17Parameters::COffsetU]=1.8;
    p[Physical17Parameters::COffsetV]=-1.1;
    p[Physical17Parameters::CTiltX]=2.2e-4;
    p[Physical17Parameters::CTiltY]=-1.4e-4;
    p[Physical17Parameters::BOffsetU]=-0.7;
    p[Physical17Parameters::BOffsetV]=2.3;
    p[Physical17Parameters::BTiltX]=-1.5e-4;
    p[Physical17Parameters::BTiltZ]=1.9e-4;
    p[Physical17Parameters::HandEyeRx]=0.012;
    p[Physical17Parameters::HandEyeRy]=-0.018;
    p[Physical17Parameters::HandEyeRz]=0.007;
    p[Physical17Parameters::HandEyeTx]=12.0;
    p[Physical17Parameters::HandEyeTy]=-8.0;
    p[Physical17Parameters::HandEyeTz]=96.0;
    return p;
}
double dist(const Vec3d&a,const Vec3d&b){return std::hypot(std::hypot(a.x-b.x,a.y-b.y),a.z-b.z);}
}

int main(){
    int fails=0;
    auto check=[&](bool ok,const char* name){if(!ok){std::cerr<<"FAIL "<<name<<"\n";++fails;}else std::cout<<"PASS "<<name<<"\n";};
    const auto p=truth();
    Joint start{20.0,-15.0,5.0,-25.0,-18.0};
    const Vec3d startTcp=Rtcp17Model::controlledPointInMachine(start,p);
    Vec3d target{startTcp.x+40.0,startTcp.y+25.0,startTcp.z-12.0};
    Joint q;
    std::string err;
    check(Rtcp17Model::inverseControlledPointAtAngles(target,35.0,22.0,p,q,err),"inverse at prescribed C/B");
    check(Rtcp17Model::forwardErrorMm(q,target,p)<1e-9,"inverse-forward residual");

    Rtcp17Options opt;opt.tcpChordToleranceMm=0.001;opt.maxRotaryStepDeg=0.15;opt.maxLinearJointStepMm=0.4;
    Rtcp17Planner planner(opt);
    auto plan=planner.planLine(start,target,35.0,22.0,12.0,p);
    check(plan.success,"RTCP plan success");
    check(plan.waypoints.size()>10,"adaptive RTCP subdivision");
    check(plan.maxForwardErrorMm<=opt.tcpChordToleranceMm*1.0001,"midpoint chord error bound");
    check(std::abs(plan.nominalSeconds-plan.nominalTcpLengthMm/12.0)<1e-10,"TCP speed timing");

    // Re-sample each controller joint-linear segment at quarter points and
    // compare with the desired task-space line.  Adaptive midpoint recursion
    // should keep the full segment well below a few microns for this smooth case.
    Joint prev=start;Vec3d prevP=startTcp;double traveled=0,maxErr=0;
    for(const auto& w:plan.waypoints){
        const Vec3d endP=w.targetMachineMm;
        for(double u: {0.25,0.5,0.75}){
            Joint qi{};for(int j=0;j<5;++j)qi[j]=prev[j]+u*(w.q[j]-prev[j]);
            Vec3d wanted{prevP.x+u*(endP.x-prevP.x),prevP.y+u*(endP.y-prevP.y),prevP.z+u*(endP.z-prevP.z)};
            maxErr=std::max(maxErr,dist(Rtcp17Model::controlledPointInMachine(qi,p),wanted));
        }
        traveled+=w.tcpDistanceFromPreviousMm;prev=w.q;prevP=endP;
    }
    check(maxErr<0.0025,"quarter-point RTCP path accuracy");
    check(std::abs(traveled-plan.nominalTcpLengthMm)<1e-9,"task-space line length accounting");

    // Naive joint interpolation between the same endpoints is intentionally not RTCP.
    double naiveMax=0;
    for(int k=0;k<=100;++k){double u=k/100.0;Joint qi{};for(int j=0;j<5;++j)qi[j]=start[j]+u*(q[j]-start[j]);
        Vec3d wanted{startTcp.x+u*(target.x-startTcp.x),startTcp.y+u*(target.y-startTcp.y),startTcp.z+u*(target.z-startTcp.z)};
        naiveMax=std::max(naiveMax,dist(Rtcp17Model::controlledPointInMachine(qi,p),wanted));}
    std::cout<<"naive_joint_midpath_error_mm="<<naiveMax<<"\n";
    check(naiveMax>0.1,"simulation distinguishes joint interpolation from RTCP");

    // Pure C/B orientation change while the calibrated sensor origin remains
    // fixed in machine space.  This is the defining compensation behavior that
    // ordinary five-axis joint interpolation does not provide.
    auto orient=planner.planLineTimed(start,startTcp,32.0,24.0,3.0,p);
    check(orient.success,"pure-orientation RTCP plan success");
    check(!orient.waypoints.empty(),"pure-orientation RTCP emits compensation joints");
    double pureMax=0;Joint pp=start;Vec3d pureTarget=startTcp;
    for(const auto& w:orient.waypoints){
        for(double u:{0.25,0.5,0.75}){Joint qi{};for(int j=0;j<5;++j)qi[j]=pp[j]+u*(w.q[j]-pp[j]);pureMax=std::max(pureMax,dist(Rtcp17Model::controlledPointInMachine(qi,p),pureTarget));}
        pp=w.q;
    }
    check(pureMax<0.0025,"pure-orientation TCP remains fixed between controller vertices");
    check(std::abs(orient.nominalSeconds-3.0)<1e-10,"pure-orientation explicit timing preserved");
    const auto& oq=orient.waypoints.back().q;
    check(std::hypot(std::hypot(oq[0]-start[0],oq[1]-start[1]),oq[2]-start[2])>1.0,"C/B rotation generates XYZ RTCP compensation");
    double naiveOrient=0;
    for(int k=0;k<=100;++k){double u=k/100.0;Joint qi=start;qi[3]=start[3]+u*(32.0-start[3]);qi[4]=start[4]+u*(24.0-start[4]);naiveOrient=std::max(naiveOrient,dist(Rtcp17Model::controlledPointInMachine(qi,p),startTcp));}
    std::cout<<"pure_orientation_rtcp_max_error_mm="<<pureMax<<"\n";
    std::cout<<"pure_orientation_naive_max_drift_mm="<<naiveOrient<<"\n";
    std::cout<<"pure_orientation_xyz_compensation_mm="<<std::hypot(std::hypot(oq[0]-start[0],oq[1]-start[1]),oq[2]-start[2])<<"\n";
    check(naiveOrient>1.0,"pure-orientation simulation exposes uncompensated TCP drift");

    // A nonzero point in the sensor frame can be selected as the controlled
    // reference for future laser-plane trajectory planning.
    Rtcp17Options planeOpt=opt;planeOpt.controlledPointSensorMm={8.0,0.0,35.0};Rtcp17Planner planePlanner(planeOpt);
    const auto planeStart=Rtcp17Model::controlledPointInMachine(start,p,planeOpt.controlledPointSensorMm);
    auto planePlan=planePlanner.planLine(start,{planeStart.x+12,planeStart.y-6,planeStart.z+4},10,-5,8,p);
    check(planePlan.success,"nonzero sensor-frame controlled point RTCP");

    std::cout<<"rtcp_line_max_sampled_error_mm="<<maxErr<<"\n";
    std::cout<<"rtcp_line_segments="<<plan.waypoints.size()<<"\n";
    std::cout<<"rtcp_pure_orientation_segments="<<orient.waypoints.size()<<"\n";
    return fails?1:0;
}
