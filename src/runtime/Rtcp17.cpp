#include "runtime/Rtcp17.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <functional>

namespace mls::runtime {
namespace {

using calibration::JointPose5d;
using calibration::Physical17Parameters;
using calibration::Vec3d;

bool finite(double v) { return std::isfinite(v); }
bool finite(const Vec3d& v) { return finite(v.x) && finite(v.y) && finite(v.z); }
bool finite(const Joint& q) { for (double v : q) if (!finite(v)) return false; return true; }

Vec3d add(const Vec3d& a, const Vec3d& b) { return {a.x+b.x,a.y+b.y,a.z+b.z}; }
Vec3d sub(const Vec3d& a, const Vec3d& b) { return {a.x-b.x,a.y-b.y,a.z-b.z}; }
Vec3d mul(const Vec3d& a, double s) { return {a.x*s,a.y*s,a.z*s}; }
double norm(const Vec3d& a) { return std::hypot(std::hypot(a.x,a.y),a.z); }
Vec3d lerp(const Vec3d& a,const Vec3d& b,double s){return add(a,mul(sub(b,a),s));}

JointPose5d toCalibration(const Joint& q) {
    JointPose5d out; out.x=q[0];out.y=q[1];out.z=q[2];out.c=q[3];out.b=q[4]; return out;
}

bool solve3x3(const Vec3d& col0, const Vec3d& col1, const Vec3d& col2,
              const Vec3d& rhs, Vec3d& x) {
    const double a00=col0.x,a01=col1.x,a02=col2.x;
    const double a10=col0.y,a11=col1.y,a12=col2.y;
    const double a20=col0.z,a21=col1.z,a22=col2.z;
    const double det = a00*(a11*a22-a12*a21)-a01*(a10*a22-a12*a20)+a02*(a10*a21-a11*a20);
    if (!finite(det) || std::abs(det) < 1e-10) return false;
    const double dx = rhs.x*(a11*a22-a12*a21)-a01*(rhs.y*a22-a12*rhs.z)+a02*(rhs.y*a21-a11*rhs.z);
    const double dy = a00*(rhs.y*a22-a12*rhs.z)-rhs.x*(a10*a22-a12*a20)+a02*(a10*rhs.z-rhs.y*a20);
    const double dz = a00*(a11*rhs.z-rhs.y*a21)-a01*(a10*rhs.z-rhs.y*a20)+rhs.x*(a10*a21-a11*a20);
    x={dx/det,dy/det,dz/det};
    return finite(x);
}

double maxLinearJointDelta(const Joint& a,const Joint& b) {
    return std::max({std::abs(a[0]-b[0]),std::abs(a[1]-b[1]),std::abs(a[2]-b[2])});
}
double maxRotaryDelta(const Joint& a,const Joint& b) {
    return std::max(std::abs(a[3]-b[3]),std::abs(a[4]-b[4]));
}
Joint midpoint(const Joint& a,const Joint& b){Joint q{};for(int i=0;i<5;++i)q[i]=0.5*(a[i]+b[i]);return q;}

double jointFeedMeasure(const Joint& a,const Joint& b) {
    const double xyz=std::hypot(std::hypot(b[0]-a[0],b[1]-a[1]),b[2]-a[2]);
    if (xyz > 1e-12) return xyz;
    return std::hypot(b[3]-a[3],b[4]-a[4]);
}

struct Sample {
    double s=0.0;
    Joint q{};
    Vec3d p{};
};

} // namespace

Vec3d Rtcp17Model::controlledPointInMachine(const Joint& q,
                                             const Physical17Parameters& parameters,
                                             const Vec3d& controlledPointSensorMm) {
    return calibration::SingleSphere17Model::mapSensorPoint(
        toCalibration(q), controlledPointSensorMm, parameters);
}

bool Rtcp17Model::inverseControlledPointAtAngles(const Vec3d& targetMachineMm,
                                                  double cDeg,
                                                  double bDeg,
                                                  const Physical17Parameters& parameters,
                                                  Joint& q,
                                                  std::string& error,
                                                  const Vec3d& controlledPointSensorMm) {
    error.clear(); q={};
    if (!finite(targetMachineMm) || !finite(cDeg) || !finite(bDeg) || !finite(controlledPointSensorMm)) {
        error="RTCP target/angles/control point contain NaN or Inf"; return false;
    }
    for (double v : parameters.value) if (!finite(v)) { error="RTCP physical17 model contains NaN or Inf"; return false; }

    Joint base{0.0,0.0,0.0,cDeg,bDeg};
    const Vec3d p0=controlledPointInMachine(base,parameters,controlledPointSensorMm);
    Joint qx=base,qy=base,qz=base;qx[0]=1.0;qy[1]=1.0;qz[2]=1.0;
    const Vec3d ux=sub(controlledPointInMachine(qx,parameters,controlledPointSensorMm),p0);
    const Vec3d uy=sub(controlledPointInMachine(qy,parameters,controlledPointSensorMm),p0);
    const Vec3d uz=sub(controlledPointInMachine(qz,parameters,controlledPointSensorMm),p0);
    Vec3d xyz;
    if(!solve3x3(ux,uy,uz,sub(targetMachineMm,p0),xyz)){
        error="RTCP linear-axis basis is singular/ill-conditioned";return false;
    }
    q={xyz.x,xyz.y,xyz.z,cDeg,bDeg};
    const double err=forwardErrorMm(q,targetMachineMm,parameters,controlledPointSensorMm);
    if(!finite(err) || err>1e-8){
        error="RTCP inverse/forward consistency residual exceeds 1e-8 mm";return false;
    }
    return true;
}

double Rtcp17Model::forwardErrorMm(const Joint& q,const Vec3d& targetMachineMm,
                                    const Physical17Parameters& parameters,
                                    const Vec3d& controlledPointSensorMm) {
    return norm(sub(controlledPointInMachine(q,parameters,controlledPointSensorMm),targetMachineMm));
}

Rtcp17Planner::Rtcp17Planner(Rtcp17Options options):m_options(options){}

Rtcp17Plan Rtcp17Planner::planLine(const Joint& startJoint,
                                    const Vec3d& endMachineMm,
                                    double endCDeg,
                                    double endBDeg,
                                    double tcpSpeedMmPerSec,
                                    const Physical17Parameters& parameters) const {
    if(!finite(tcpSpeedMmPerSec) || tcpSpeedMmPerSec<=0){Rtcp17Plan x;x.message="Invalid RTCP TCP speed";return x;}
    const Vec3d startP=Rtcp17Model::controlledPointInMachine(startJoint,parameters,m_options.controlledPointSensorMm);
    if(!finite(startP)){Rtcp17Plan x;x.message="Invalid RTCP forward start";return x;}
    const double length=norm(sub(endMachineMm,startP));
    if(length<=1e-12){Rtcp17Plan x;x.message="Use planLineTimed for pure-orientation RTCP motion";return x;}
    auto out=planLineTimed(startJoint,endMachineMm,endCDeg,endBDeg,length/tcpSpeedMmPerSec,parameters);
    out.requestedTcpSpeedMmPerSec=tcpSpeedMmPerSec;
    return out;
}

Rtcp17Plan Rtcp17Planner::planLineTimed(const Joint& startJoint,
                                         const Vec3d& endMachineMm,
                                         double endCDeg,
                                         double endBDeg,
                                         double nominalSeconds,
                                         const Physical17Parameters& parameters) const {
    Rtcp17Plan out;out.nominalSeconds=nominalSeconds;
    if(!finite(startJoint)||!finite(endMachineMm)||!finite(endCDeg)||!finite(endBDeg)||
       !finite(nominalSeconds)||nominalSeconds<=0 ||
       !finite(m_options.tcpChordToleranceMm)||m_options.tcpChordToleranceMm<=0 ||
       !finite(m_options.maxRotaryStepDeg)||m_options.maxRotaryStepDeg<=0 ||
       !finite(m_options.maxLinearJointStepMm)||m_options.maxLinearJointStepMm<=0 ||
       m_options.maxSubdivisionDepth<1||m_options.maxSubdivisionDepth>40||m_options.maxSegments<1){
        out.message="Invalid RTCP start/target/duration/options";return out;
    }
    const Vec3d startP=Rtcp17Model::controlledPointInMachine(startJoint,parameters,m_options.controlledPointSensorMm);
    if(!finite(startP)){out.message="Invalid RTCP forward start";return out;}
    out.nominalTcpLengthMm=norm(sub(endMachineMm,startP));
    const double dc=endCDeg-startJoint[3], db=endBDeg-startJoint[4];
    const double rotaryMagnitude=std::max(std::abs(dc),std::abs(db));
    if(out.nominalTcpLengthMm<=1e-12 && rotaryMagnitude<=1e-12){
        out.success=true;out.nominalSeconds=0;out.message="No RTCP motion";return out;
    }
    if(out.nominalTcpLengthMm>1e-12)out.requestedTcpSpeedMmPerSec=out.nominalTcpLengthMm/nominalSeconds;

    Joint endQ;std::string detail;
    if(!Rtcp17Model::inverseControlledPointAtAngles(endMachineMm,endCDeg,endBDeg,parameters,endQ,detail,m_options.controlledPointSensorMm)){
        out.message=detail;return out;
    }
    std::vector<Sample> samples;samples.reserve(1024);
    samples.push_back({0.0,startJoint,startP});

    std::function<bool(const Sample&,const Sample&,int)> refine;
    refine=[&](const Sample& a,const Sample& b,int depth)->bool{
        if(samples.size()>m_options.maxSegments){out.message="RTCP segment budget exceeded";return false;}
        const double sm=0.5*(a.s+b.s);
        const Vec3d pm=lerp(startP,endMachineMm,sm);
        Joint qm;std::string e;
        if(!Rtcp17Model::inverseControlledPointAtAngles(pm,startJoint[3]+dc*sm,startJoint[4]+db*sm,
               parameters,qm,e,m_options.controlledPointSensorMm)){out.message=e;return false;}
        const Joint qlin=midpoint(a.q,b.q);
        const double chordErr=Rtcp17Model::forwardErrorMm(qlin,pm,parameters,m_options.controlledPointSensorMm);
        const double linearStep=maxLinearJointDelta(a.q,b.q);
        const double rotaryStep=maxRotaryDelta(a.q,b.q);
        const bool split=chordErr>m_options.tcpChordToleranceMm ||
                         linearStep>m_options.maxLinearJointStepMm ||
                         rotaryStep>m_options.maxRotaryStepDeg;
        if(split){
            if(depth>=m_options.maxSubdivisionDepth){out.message="RTCP subdivision depth exceeded";return false;}
            Sample mid{sm,qm,pm};
            if(!refine(a,mid,depth+1))return false;
            if(!refine(mid,b,depth+1))return false;
        }else{
            samples.push_back(b);
            out.maxForwardErrorMm=std::max(out.maxForwardErrorMm,chordErr);
            out.maxJointLinearStepMm=std::max(out.maxJointLinearStepMm,linearStep);
            out.maxRotaryStepDeg=std::max(out.maxRotaryStepDeg,rotaryStep);
        }
        return true;
    };
    Sample end{1.0,endQ,endMachineMm};
    if(!refine(samples.front(),end,0))return out;
    if(samples.size()<2){out.message="RTCP refinement produced no segment";return out;}

    out.waypoints.reserve(samples.size()-1);
    double assigned=0;
    for(std::size_t i=1;i<samples.size();++i){
        const auto& prev=samples[i-1];const auto& cur=samples[i];
        Rtcp17Waypoint w;w.q=cur.q;w.targetMachineMm=cur.p;w.cDeg=cur.q[3];w.bDeg=cur.q[4];
        w.tcpDistanceFromPreviousMm=norm(sub(cur.p,prev.p));
        // Time follows task parameter, not joint length.  This also handles a
        // pure orientation move that holds the controlled point exactly fixed.
        const double fraction=cur.s-prev.s;
        w.nominalSeconds=nominalSeconds*fraction;
        const double measure=jointFeedMeasure(prev.q,cur.q);
        if(!finite(w.nominalSeconds)||w.nominalSeconds<=0||!finite(measure)||measure<=1e-15){
            out.message="RTCP produced an invalid segment duration/metric";out.waypoints.clear();return out;
        }
        w.controllerFeedPerSecond=measure/w.nominalSeconds;
        assigned+=w.nominalSeconds;
        out.waypoints.push_back(w);
    }
    out.nominalSeconds=assigned;
    out.success=true;
    out.message="RTCP task-space path converted to calibrated XYZCB joint polyline";
    return out;
}

} // namespace mls::runtime
