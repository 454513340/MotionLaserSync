#include "twin/MultiPoseExperiment.h"
#include "runtime/FiveAxis.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

#include <utility>

namespace mls::twin {
namespace {
bool finite(double v){return std::isfinite(v);}
bool finite(Vec3 v){return finite(v.x)&&finite(v.y)&&finite(v.z);}
bool validJoint(const Joint5& q){for(double v:q)if(!finite(v))return false;return true;}
bool inside(const Joint5& q,const M6AxisBounds& b){
    for(int i=0;i<5;++i)if(q[i]<b.min[i]-1.e-8||q[i]>b.max[i]+1.e-8)return false;
    return true;
}
Vec3 mix(Vec3 a,Vec3 b,double u){return a+(b-a)*u;}
Joint5 interpolate(const Joint5& a,const Joint5& b,double u){
    Joint5 q{};for(int i=0;i<5;++i)q[i]=a[i]+(b[i]-a[i])*u;return q;
}
bool solve3(Vec3 a,Vec3 b,Vec3 c,Vec3 rhs,Vec3& out){
    const double d=dot(a,cross(b,c));
    if(!finite(d)||std::abs(d)<1e-10)return false;
    out={dot(rhs,cross(b,c))/d,dot(a,cross(rhs,c))/d,dot(a,cross(b,rhs))/d};
    return finite(out);
}
// Exact for affine-in-XYZ Physical17 FK. Validate residual to fail closed.
bool inverseFocus(const M6ForwardKinematics& fk,Vec3 target,double C,double B,
                  double focusZ,Joint5& q,std::string& error){
    Joint5 origin{{0,0,0,C,B}};
    const Vec3 focus{0,0,focusZ};
    const Vec3 p0=fk(origin).point(focus);
    if(!finite(p0)){error="FK returned nonfinite sensor focus";return false;}
    Joint5 x=origin,y=origin,z=origin;x[0]=1.;y[1]=1.;z[2]=1.;
    const Vec3 ux=fk(x).point(focus)-p0;
    const Vec3 uy=fk(y).point(focus)-p0;
    const Vec3 uz=fk(z).point(focus)-p0;
    Vec3 v;
    if(!solve3(ux,uy,uz,target-p0,v)){error="Physical17 XYZ basis singular";return false;}
    q={{v.x,v.y,v.z,C,B}};
    if(length(fk(q).point(focus)-target)>1e-5){
        error="FK/IK mismatch >1e-5 mm, model cannot anchor a fixed-angle ROI";return false;
    }
    return true;
}
std::string badPath(const Joint5& q,const ExperimentOptions& o,
                    const M6CollisionCheck& check){
    if(!validJoint(q))return "nonfinite joint";
    const char* labels[]={"X", "Y", "Z", "C", "B"};
    for(int axis=0; axis<5; ++axis){
        if(q[axis]<o.bounds.min[axis]-1.e-8 || q[axis]>o.bounds.max[axis]+1.e-8){
            std::ostringstream os;
            os.setf(std::ios::fixed);
            os.precision(4);
            os<<"joint outside configured planning bounds: axis "<<labels[axis]
              <<" requested="<<q[axis]<<" allowed=["
              <<o.bounds.min[axis]<<", "<<o.bounds.max[axis]<<"]"
              <<"; XYZCB=("<<q[0]<<","<<q[1]<<","<<q[2]<<","<<q[3]<<","<<q[4]<<")";
            return os.str();
        }
    }
    if(check){std::string error;if(!check(q,&error))return "machine collision/clearance check failed: "+error;}
    return {};
}
struct WayPoint {Vec3 focus;double C=0,B=0;};
struct FanEnvelope {
    double minAlong=std::numeric_limits<double>::infinity();
    double maxAlong=-std::numeric_limits<double>::infinity();
    double minAcross=std::numeric_limits<double>::infinity();
    double maxAcross=-std::numeric_limits<double>::infinity();
    std::size_t samples=0;
};
// Project the actual M3 virtual laser fan to the CAD reference plane for
// THIS B/C orientation. A focus point at the CAD boundary is not a scan
// coverage criterion: the projected fan may be strongly skewed or shifted.
// This is a conservative, planar envelope, not proof of optical visibility
// for curved faces, holes or occlusions.
bool projectedFan(const M6ForwardKinematics& fk,const Transform& cadToMachine,
                  const Sensor2950Geometry& optics,double C,double B,
                  double focusZ,Vec3 centreCad,ExperimentSweepAxis sweep,
                  FanEnvelope& out,std::string& error){
    Joint5 q{};
    const Vec3 centreMachine=cadToMachine.point(centreCad);
    if(!inverseFocus(fk,centreMachine,C,B,focusZ,q,error))return false;
    const Transform sensorToMachine=fk(q);
    const Transform machineToSensor=sensorToMachine.inverseRigid();
    const Transform machineToCad=cadToMachine.inverseRigid();
    const Vec3 planeNormal=normalized(cadToMachine.vector({0,0,1}));
    const Vec3 laserOrigin=sensorToMachine.point(optics.laserOriginSensor);
    const double zRef=optics.zCenter;
    for(int j=0;j<optics.sampleCount;++j){
        const Vec3 rayTarget{optics.sampleXAtZ(j,zRef),0,zRef};
        const Vec3 rayDirection=normalized(sensorToMachine.vector(
            normalized(rayTarget-optics.laserOriginSensor)));
        const double den=dot(planeNormal,rayDirection);
        if(std::abs(den)<1.e-8)continue;
        const double distance=dot(planeNormal,centreMachine-laserOrigin)/den;
        if(!finite(distance)||distance<=0||distance>optics.maxTraceDistanceMm)continue;
        const Vec3 hitMachine=laserOrigin+rayDirection*distance;
        const Vec3 pSensor=machineToSensor.point(hitMachine);
        if(pSensor.z<optics.zNear||pSensor.z>optics.zFar||
           std::abs(pSensor.x)>optics.widthAtZ(pSensor.z)*0.5)continue;
        const Vec3 relative=machineToCad.point(hitMachine)-centreCad;
        const double along=sweep==ExperimentSweepAxis::CadY?relative.y:relative.x;
        const double across=sweep==ExperimentSweepAxis::CadY?relative.x:relative.y;
        out.minAlong=std::min(out.minAlong,along);
        out.maxAlong=std::max(out.maxAlong,along);
        out.minAcross=std::min(out.minAcross,across);
        out.maxAcross=std::max(out.maxAcross,across);
        ++out.samples;
    }
    if(out.samples<32 || out.maxAcross-out.minAcross<1.){
        error="B/C laser fan has insufficient projected lateral width on CAD reference plane";
        return false;
    }
    return true;
}

}

std::vector<ExperimentPose> standardBCExperimentMatrix(){
    // Controlled M3/M4 protocol: 6 B values x 6 C values, plus one
    // separately requested normal-incidence reference (B=0, C=0).
    // A 4-degree step from -10 to +10 does NOT contain zero.
    std::vector<ExperimentPose> poses;
    poses.reserve(37);
    for(int B=-10;B<=10;B+=4){
        for(int C=-10;C<=10;C+=4){
            std::ostringstream id;
            id<<"P";
            if(poses.size()+1<10)id<<"0";
            id<<poses.size()+1;
            poses.push_back({id.str(),double(C),double(B)});
        }
    }
    poses.push_back({"P37",0.0,0.0});
    return poses;
}

ExperimentPlan planMultiPoseExperiment(const TwinScene& scene,
    const std::vector<ExperimentPose>& poses,const ExperimentOptions& o,
    const M6ForwardKinematics& fk,const M6CollisionCheck& collision){
    ExperimentPlan out;
    auto fail=[&](const std::string& msg){out.views.clear();out.error=msg;return out;};
    if(!scene.hasWorkpiece())return fail("STEP/CAD scene missing");
    if(!fk)return fail("Physical17/recorded FK callback missing (no dummy motion)");
    if(poses.empty()||poses.size()>81)return fail("Require 1..81 independent C/B poses (37 in the standard matrix)");
    if(!o.roiCad.valid()||!finite(o.surfaceZCadMm)||!finite(o.scanSpeedMmPerSec)||
       !finite(o.transferSpeedMmPerSec)||!finite(o.profileRateHz)||
       !finite(o.focusSensorZmm)||!finite(o.retractFromSurfaceCadMm)||
       !finite(o.lanePitchMm)||!finite(o.maximumJointChordMm)||
       !finite(o.leadInMm)||!finite(o.leadOutMm)||!finite(o.lateralCoverageMarginMm))
        return fail("Nonfinite experiment parameters/ROI");
    if(o.scanSpeedMmPerSec<=0||o.transferSpeedMmPerSec<=0||o.profileRateHz<1||o.profileRateHz>2000||
       o.focusSensorZmm<=0||o.retractFromSurfaceCadMm<0||o.maximumJointChordMm<=0||o.lanePitchMm<=0||
       o.leadInMm<0||o.leadOutMm<0||o.lateralCoverageMarginMm<0)
        return fail("Invalid speed/rate/focus/retract/lane pitch");
    const Bounds cad=scene.mesh().bounds;
    if(o.roiCad.min.x<cad.min.x-1e-3||o.roiCad.max.x>cad.max.x+1e-3||
       o.roiCad.min.y<cad.min.y-1e-3||o.roiCad.max.y>cad.max.y+1e-3||
       o.surfaceZCadMm<cad.min.z-1e-3||o.surfaceZCadMm>cad.max.z+1e-3)
        return fail("CAD-local ROI/reference surface lies outside CAD bounding box");
    if(o.maximumProfiles<2 || o.maximumProfiles>1000000)
        return fail("Invalid aggregate profile budget");
    std::string opticalError;
    if(!o.opticalGeometry.validate(&opticalError))
        return fail("Invalid M3 optical geometry: "+opticalError);
    std::size_t aggregateProfiles=0;
    if(o.hasArchivedInitialJoints && !validJoint(o.archivedInitialJoints))
        return fail("Recorded historical start joints are invalid");
    const Transform T=scene.cadToMachineEstimate();
    for(double n:T.m)if(!finite(n))return fail("CAD->MCS placement contains NaN/Inf");
    std::string error;
    for(const auto& spec:poses){
        if(spec.id.empty()||spec.id.size()>40||spec.id.find_first_not_of("ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789_-")!=std::string::npos)
            return fail("Pose id must be 1..40 ASCII letters/digits/_/-");
        for(const auto& v:out.views)if(v.id==spec.id)return fail("Pose ids must be unique");
        if(!finite(spec.cDeg)||!finite(spec.bDeg))return fail("Nonfinite rotary pose");
        ExperimentViewReport view;view.id=spec.id;view.cDeg=spec.cDeg;view.bDeg=spec.bDeg;
        view.collisionCheckerUsed=bool(collision);
        // Resolve the CAD sweep axis from the actual CAD->MCS transform.
        // The machine X projection, NOT the CAD name, establishes the
        // demanded physical X+ to X- scan direction.
        const double xProjectionCadX=T.vector({1,0,0}).x;
        const double xProjectionCadY=T.vector({0,1,0}).x;
        ExperimentSweepAxis sweep=o.direction;
        if(sweep==ExperimentSweepAxis::AutoMachineX){
            sweep=std::abs(xProjectionCadX)>=std::abs(xProjectionCadY)
                ?ExperimentSweepAxis::CadX:ExperimentSweepAxis::CadY;
        }
        const double machineXProjection=sweep==ExperimentSweepAxis::CadY
            ?xProjectionCadY:xProjectionCadX;
        if(std::abs(machineXProjection)<0.25)
            return fail(view.id+": chosen CAD axis is not aligned with machine X; use AutoMachineX");
        const bool scanFromHighToLow=machineXProjection>0.;
        view.resolvedAxis=sweep;
        view.machineXScanAxisProjection=machineXProjection;
        view.machineXDescending=true;
        const double scanLo=sweep==ExperimentSweepAxis::CadY?o.roiCad.min.y:o.roiCad.min.x;
        const double scanHi=sweep==ExperimentSweepAxis::CadY?o.roiCad.max.y:o.roiCad.max.x;
        const double lateralLo=sweep==ExperimentSweepAxis::CadY?o.roiCad.min.x:o.roiCad.min.y;
        const double lateralHi=sweep==ExperimentSweepAxis::CadY?o.roiCad.max.x:o.roiCad.max.y;
        const double scanSpan=scanHi-scanLo;
        const double lateralSpan=lateralHi-lateralLo;
        if(scanSpan<0.1||lateralSpan<0)
            return fail("ROI scan dimension must be at least 0.1 mm");
        // Project laser fan onto the CAD reference plane at the requested B/C.
        // This accounts for the skewed footprint and lateral shift that
        // previously caused the last diagonal stripe merely to touch one edge.
        const Vec3 centreCad{o.roiCad.center().x,o.roiCad.center().y,o.surfaceZCadMm};
        FanEnvelope fan;
        if(!projectedFan(fk,T,o.opticalGeometry,spec.cDeg,spec.bDeg,
                         o.focusSensorZmm,centreCad,sweep,fan,error))
            return fail(view.id+": projected optical fan: "+error);
        view.projectedLaserScanMinMm=fan.minAlong;
        view.projectedLaserScanMaxMm=fan.maxAlong;
        view.projectedLaserLateralMinMm=fan.minAcross;
        view.projectedLaserLateralMaxMm=fan.maxAcross;
        const double fanWidth=fan.maxAcross-fan.minAcross;
        view.predictedLateralCoverageWidthMm=fanWidth;
        // Conservatively overrun both ends for EVERY projected laser sample.
        // Laser remains ON throughout the lead-in and lead-out portions.
        const double extendedHi=scanHi-fan.minAlong+o.leadInMm;
        const double extendedLo=scanLo-fan.maxAlong-o.leadOutMm;
        if(!(extendedHi>extendedLo) || !finite(extendedHi) || !finite(extendedLo))
            return fail(view.id+": invalid skew compensated start/end envelope");
        view.scanStartCad=scanFromHighToLow?extendedHi:extendedLo;
        view.scanEndCad=scanFromHighToLow?extendedLo:extendedHi;
        view.beforePartMarginMm=o.leadInMm;
        view.afterPartMarginMm=o.leadOutMm;
        const double extendedSpan=extendedHi-extendedLo;
        // Use overlapping laser-footprint projections, never the nominal 50mm
        // sensor width alone. Offsetting the first lane centres covers BOTH
        // lateral ROI boundaries, including slanted B/C projections.
        const double pitch=std::min(o.lanePitchMm,0.8*fanWidth);
        if(!(pitch>0))return fail(view.id+": invalid lateral fan pitch");
        const double requestedSpan=lateralSpan+2*o.lateralCoverageMarginMm;
        const std::size_t lanes=std::max<std::size_t>(1,
            static_cast<std::size_t>(std::ceil(std::max(0.,requestedSpan-fanWidth)/pitch))+1);
        if(lanes>16)return fail(view.id+": tilted laser footprint needs >16 lanes; adjust ROI/pose");
        const double laneFirst=(lateralLo+lateralHi-
            fan.minAcross-fan.maxAcross-double(lanes-1)*pitch)*0.5;
        const double totalMin=laneFirst+fan.minAcross;
        const double totalMax=laneFirst+double(lanes-1)*pitch+fan.maxAcross;
        if(totalMin>lateralLo-o.lateralCoverageMarginMm+1e-6||
           totalMax<lateralHi+o.lateralCoverageMarginMm-1e-6)
            return fail(view.id+": projected fan cannot span both lateral ROI boundaries");
        view.planarCoverageEnvelopePassed=true;
        const std::size_t profilesPerLane=static_cast<std::size_t>(
            std::ceil(extendedSpan*o.profileRateHz/o.scanSpeedMmPerSec))+1;
        if(profilesPerLane<2 || profilesPerLane>o.maximumProfiles ||
           lanes>o.maximumProfiles/profilesPerLane ||
           aggregateProfiles>o.maximumProfiles-lanes*profilesPerLane)
            return fail(view.id+": aggregate profile limit exceeded; raise experiment profile budget for 37 poses");
        aggregateProfiles+=lanes*profilesPerLane;
        const Vec3 focusLocal{0,0,o.focusSensorZmm};
        double t=0;
        auto cadPoint=[&](double lateral,double progress,double lift)->Vec3{
            const Vec3 p=sweep==ExperimentSweepAxis::CadY
                ?Vec3{lateral,progress,o.surfaceZCadMm+lift}
                :Vec3{progress,lateral,o.surfaceZCadMm+lift};
            return T.point(p);
        };
        // Every view begins with an independent program. Never link them with
        // an implicit, untested C/B turn while the laser is on.
        auto add=[&](double time,const Joint5& q,M6SegmentRole role)->bool{
            if(!finite(time)||time<0){error="Invalid experiment timeline";return false;}
            if(!view.path.empty()&&time<view.path.back().seconds-1e-8){error="Timeline backwards";return false;}
            if(auto bad=badPath(q,o,collision);!bad.empty()){
                const char* stage=view.path.empty()
                    ?": first scan START reference (no approach motion): "
                    :(role==M6SegmentRole::Scan?": SCAN: ":": laser-OFF lane transfer: ");
                error=view.id+stage+bad;
                return false;
            }
            M6PathSample p;p.seconds=time;p.joints=q;p.sensorToMachine=fk(q);p.role=role;
            view.path.push_back(p);view.focusPathMachine.push_back(p.sensorToMachine.point(focusLocal));
            if(view.path.size()>o.maximumPathPoints){error="Too many NC path points";return false;}
            return true;
        };
        // No invented approach/retraction: each independent NC begins at its
        // first validated scan start. A historical .scan joint is ONLY used
        // to anchor Physical17 FK, never as a current machine state or as a
        // commanded move. The NC exporter labels its initial G00 as UNKNOWN.
        // Subsequent lane changes are laser-OFF transfers at the same nominal
        // focal height; they still require collision and travel validation.
        Joint5 current{};
        bool hasCurrent=false;
        for(std::size_t lane=0;lane<lanes;++lane){
            const double lateral=laneFirst+double(lane)*pitch;
            // Every lane sweeps machine X+ -> X-. Non-acquiring lateral
            // transfers happen after the previous scan has stopped.
            const double begin=view.scanStartCad;
            const double end=view.scanEndCad;
            const Vec3 targetStart=cadPoint(lateral,begin,0);
            const Vec3 targetEnd=cadPoint(lateral,end,0);
            Joint5 qStart{},qEnd{};
            if(!inverseFocus(fk,targetStart,spec.cDeg,spec.bDeg,o.focusSensorZmm,qStart,error)||
               !inverseFocus(fk,targetEnd,spec.cDeg,spec.bDeg,o.focusSensorZmm,qEnd,error))
                return fail(view.id+": "+error);
            auto transferTo=[&](const Joint5& to)->bool{
                if(!hasCurrent){current=to;hasCurrent=true;return add(t,current,M6SegmentRole::Transfer);}
                const double distance=length(fk(to).point(focusLocal)-fk(current).point(focusLocal));
                double duration=std::max(distance/o.transferSpeedMmPerSec,1.e-3);
                // Simultaneous machine axis rates limit transfer time too.
                for(int i=0;i<5;++i){
                    const double v=o.bounds.maxSpeed[i];
                    if(!finite(v)||v<=0){error="Invalid joint maximum speed";return false;}
                    duration=std::max(duration,1.05*std::abs(to[i]-current[i])/v);
                }
                const auto n=std::max<std::size_t>(1,std::size_t(std::ceil(
                    std::max(distance/o.maximumJointChordMm, std::max(std::abs(to[3]-current[3]),std::abs(to[4]-current[4]))/0.5))));
                if(n>100000){error="Transfer interpolation step budget exceeded";return false;}
                const Joint5 old=current;const double startT=t;
                for(std::size_t j=1;j<=n;++j){
                    const double u=double(j)/double(n);const auto q=interpolate(old,to,u);
                    if(!add(startT+duration*u,q,M6SegmentRole::Transfer))return false;
                }
                t=startT+duration;current=to;return true;
            };
            // Stop M50 capture at the previous scan's end (M51), traverse
            // to the next scan start with S0, then re-arm M50 at the start.
            // No Z lift, no extra non-acquiring approach/exit segment.
            if(!transferTo(qStart))return fail(error);
            const double duration=length(targetEnd-targetStart)/o.scanSpeedMmPerSec;
            const std::size_t nPath=std::max<std::size_t>(1,std::size_t(std::ceil(extendedSpan/o.maximumJointChordMm)));
            const double scanStartTime=t;
            // Analytically commanded CAD straight line, independently inverse-
            // mapped at the chosen C/B. Preview profiles are sampled on an
            // ideal clock; actual REGIST hardware sampling may differ.
            for(std::size_t j=1;j<=nPath;++j){
                const double u=double(j)/double(nPath);
                Joint5 q{};
                if(!inverseFocus(fk,mix(targetStart,targetEnd,u),spec.cDeg,spec.bDeg,o.focusSensorZmm,q,error))
                    return fail(view.id+": "+error);
                if(!add(scanStartTime+duration*u,q,M6SegmentRole::Scan))return fail(error);
            }
            t=scanStartTime+duration;current=qEnd;
            ++view.scanGroups;view.nominalScanLengthMm+=extendedSpan;
            // Uniform ideal profile rate for M3 optical simulation. Ignore
            // speed ramp and edge exposure here, explicitly reported as limit.
            const std::size_t n=profilesPerLane;
            for(std::size_t j=0;j<n;++j){
                const double u=double(j)/double(n-1);
                Joint5 q{};
                if(!inverseFocus(fk,mix(targetStart,targetEnd,u),spec.cDeg,spec.bDeg,o.focusSensorZmm,q,error))
                    return fail(view.id+": "+error);
                VirtualProfilePose pose;pose.elapsedSeconds=scanStartTime+duration*u;
                pose.sourceOrdinal=static_cast<std::uint64_t>(view.exposures.size());
                pose.joints=q;pose.sensorToMachine=fk(q);
                view.exposures.push_back(pose);
            }
            view.plannedProfiles=view.exposures.size();
            // At the end of the last scan point the NC exporter emits M51,
            // then M60/M61; no retract or other unrequested motion is added.
        }
        const std::size_t sampleCap=std::max(1u,o.opticalPreviewProfilesPerPose);
        for(std::size_t j=0;j<std::min(sampleCap,view.exposures.size());++j){
            const std::size_t idx=(j*(view.exposures.size()-1))/std::max<std::size_t>(1,std::min(sampleCap,view.exposures.size())-1);
            view.opticalPreviewPoses.push_back(view.exposures[idx]);
        }
        // Check the ideal joint-space rate on every NC segment, including
        // tilted B/C orientations. Controller acceleration, jerk and actual
        // trigger times still need independent runtime qualification.
        for(std::size_t j=1;j<view.path.size();++j){
            const double dt=view.path[j].seconds-view.path[j-1].seconds;
            if(dt<=1.e-9)return fail(view.id+": zero-time NC motion segment");
            for(int axis=0;axis<5;++axis){
                const double speed=std::abs(view.path[j].joints[axis]-view.path[j-1].joints[axis])/dt;
                if(speed>o.bounds.maxSpeed[axis]+1.e-7)
                    return fail(view.id+": planned NC edge exceeds offline joint speed limit");
            }
        }
        view.nominalDurationSec=t;
        view.predictedProfilePitchMm=o.scanSpeedMmPerSec/o.profileRateHz;
        view.warning="OFFLINE: projected fan envelope and X+->X- with pre/post scanning coverage margins on nominal CAD plane; "
                     "this does not prove optical visibility at recessed/occluded surfaces. "
                     "Manually placed CAD, provisional multi-angle 2950 camera geometry; "
                     "first G00 start is unknown, direct laser-OFF lane transfers require collision verification, "
                     "trigger latency and motor acceleration not certified";
        out.views.push_back(std::move(view));
    }
    return out;
}

// Enforce the actual five-axis G93 trajectory envelope *at generation time*.
// Retiming occurs before NC export and BEFORE the NC/planner comparison; the
// geometry, exposure roles and all STEP/FCL decisions remain unchanged.
static bool retimeReportG93(M6Report& report,const ExperimentOptions& opts,std::string& error){
    error.clear();
    auto& samples=report.path;
    if(samples.size()<2){error="G93 trajectory needs at least two XYZCB positions";return false;}
    const auto originals=samples;
    mls::runtime::Limits limits=opts.hardwareG93Limits;
    // Use the commissioned five-axis SP software speed caps, not the older
    // M6 inverse-kinematics / sample-spacing limits (which default to 12/15).
    // Any lower limits read from the device are still checked again by stage 1.
    double currentTime=0;
    samples.front().seconds=0;
    std::size_t from=1;
    while(from<originals.size()){
        const auto role=originals[from].role;
        std::size_t to=from;
        std::vector<mls::runtime::Waypoint> points;
        for(;to<originals.size() && originals[to].role==role;++to){
            const double duration=originals[to].seconds-originals[to-1].seconds;
            if(!finite(duration)||duration<=1e-10){
                error="G93 trajectory has zero/negative point duration";return false;
            }
            points.push_back({originals[to].joints,60./duration,
                              mls::runtime::FeedMode::InverseMinute,true});
        }
        std::vector<double> actualSeconds;
        std::string why;
        if(!mls::runtime::retimeG93Polyline(originals[from-1].joints,
                points,limits,opts.hardwareVectorAccelCap,
                opts.hardwareVectorDecelCap,actualSeconds,why)){
            error="Continuous G93 path cannot be dynamically timed at step "+
                std::to_string(from)+": "+why;
            return false;
        }
        if(actualSeconds.size()!=points.size()){
            error="G93 planner changed the number of NC knots";return false;
        }
        for(std::size_t i=0;i<actualSeconds.size();++i){
            currentTime+=actualSeconds[i];
            samples[from+i].seconds=currentTime;
        }
        from=to;
    }
    return true;
}

ConnectedExperimentSession planConnectedExperimentSession(
    const ExperimentPlan& plan,const ExperimentOptions& options,
    const M6ForwardKinematics& fk,const M6CollisionCheck& collision){
    ConnectedExperimentSession out;
    auto fail=[&](const std::string& reason){out.error=reason;out.merged.path.clear();return out;};
    if(!plan.ok()||plan.views.empty()||!fk)
        return fail("Cannot join an invalid or empty multi-pose plan");
    if(plan.views.size()>1&&!collision)
        return fail("Continuous all-pose NC requires an active machine collision checker; "
                    "separate pose programs must not be played as a continuous machine move");
    if(!finite(options.transferSpeedMmPerSec)||options.transferSpeedMmPerSec<=0||
       !finite(options.maximumJointChordMm)||options.maximumJointChordMm<=0)
        return fail("Invalid inter-pose transfer speed/chord setting");
    out.merged=multiPoseAsOfflineM6(plan.views.front(),options);
    out.merged.path.clear();
    out.merged.collisionModelUsed=bool(collision);
    out.merged.canExportForHardware=false;
    out.merged.options.hasInitialJoints=false;
    for(std::size_t poseIndex=0;poseIndex<plan.views.size();++poseIndex){
        const auto& view=plan.views[poseIndex];
        if(view.path.size()<2)return fail(view.id+": missing five-axis path");
        const auto& start=view.path.front();
        if(poseIndex==0){
            for(const auto& step:view.path){
                if(auto bad=badPath(step.joints,options,collision);!bad.empty())
                    return fail(view.id+": "+bad);
            }
            out.merged.path=view.path;
            out.poseStartSeconds.push_back(0.);
            out.poseEndSeconds.push_back(view.path.back().seconds-view.path.front().seconds);
            // Shift the first pose only if an external caller gave nonzero initial time.
            for(auto& step:out.merged.path)step.seconds-=view.path.front().seconds;
            continue;
        }
        // Each independent NC starts with an UNKNOWN first G00. A continuous
        // session cannot jump to that pose. Compute a physically finite,
        // collision-checked laser-OFF connector from the preceding NC endpoint.
        const auto& previous=out.merged.path.back();
        const Joint5 from=previous.joints,to=start.joints;
        const Vec3 focus{0.,0.,options.focusSensorZmm};
        const Vec3 a=fk(from).point(focus), b=fk(to).point(focus);
        if(!finite(a)||!finite(b))return fail(view.id+": invalid transfer FK");
        const double distance=length(b-a);
        double duration=std::max(distance/options.transferSpeedMmPerSec,1.e-3);
        double maxCartesianAxis=0.,maxAngular=0.;
        for(int axis=0;axis<5;++axis){
            const double diff=std::abs(to[axis]-from[axis]);
            const double vmax=options.bounds.maxSpeed[axis];
            if(!finite(vmax)||vmax<=0)return fail(view.id+": invalid axis maximum speed");
            duration=std::max(duration,1.05*diff/vmax);
            if(axis<3)maxCartesianAxis=std::max(maxCartesianAxis,diff);
            else maxAngular=std::max(maxAngular,diff);
        }
        // Check the entire actual joint interpolation, not just both endpoints.
        const double subdivisions=std::max({1.,distance/options.maximumJointChordMm,
            maxCartesianAxis/options.maximumJointChordMm,maxAngular/0.5});
        if(!finite(subdivisions)||subdivisions>100000)
            return fail(view.id+": inter-pose connection exceeds interpolation budget");
        const auto steps=std::size_t(std::ceil(subdivisions));
        const double base=previous.seconds;
        for(std::size_t k=1;k<=steps;++k){
            const double u=double(k)/double(steps);
            const auto joints=interpolate(from,to,u);
            const auto why=badPath(joints,options,collision);
            if(!why.empty()){
                std::ostringstream message;
                message<<view.id<<": laser-OFF inter-pose connection fails at step "
                       <<k<<"/"<<steps<<": "<<why;
                return fail(message.str());
            }
            M6PathSample edge;
            edge.seconds=base+duration*u;
            edge.joints=joints;edge.sensorToMachine=fk(joints);
            edge.role=M6SegmentRole::Transfer;
            out.merged.path.push_back(edge);
        }
        const double joinedAt=base+duration;
        out.poseStartSeconds.push_back(joinedAt);
        // The first point is already the end of the connecting S0 move.
        // Remaining samples of the independent pose keep their exact time/role.
        for(std::size_t k=1;k<view.path.size();++k){
            auto step=view.path[k];
            const double local=step.seconds-start.seconds;
            if(!finite(local)||local<=0.)return fail(view.id+": non-positive local NC step");
            step.seconds=joinedAt+local;
            out.merged.path.push_back(std::move(step));
        }
        out.poseEndSeconds.push_back(out.merged.path.back().seconds);
    }
    if(out.poseEndSeconds.size()!=plan.views.size())
        return fail("Continuous NC session pose-count mismatch");
    // Save source indices for every pose before changing any timestamps.
    std::vector<std::size_t> starts,ends;
    for(std::size_t i=0;i<out.poseStartSeconds.size();++i){
        auto findIndex=[&](double seconds){
            const auto it=std::lower_bound(out.merged.path.begin(),out.merged.path.end(),seconds,
                [](const M6PathSample& p,double t){return p.seconds<t-1e-8;});
            return std::min(std::size_t(it-out.merged.path.begin()),out.merged.path.size()-1);
        };
        starts.push_back(findIndex(out.poseStartSeconds[i]));
        ends.push_back(findIndex(out.poseEndSeconds[i]));
    }
    if(options.enforceHardwareG93){
        std::string timingIssue;
        if(!retimeReportG93(out.merged,options,timingIssue))
            return fail("Connected G93 physical-time planner: "+timingIssue);
        out.merged.g93PhysicalTimelinePlanned=true;
        for(std::size_t i=0;i<starts.size();++i){
            out.poseStartSeconds[i]=out.merged.path[starts[i]].seconds;
            out.poseEndSeconds[i]=out.merged.path[ends[i]].seconds;
        }
    }
    out.merged.transferSafetyWarning="OFFLINE: laser-OFF inter-pose five-axis transfers were "
        "joint-interpolated and checked at discrete collision samples. First G00 approach and "
        "controller acceleration/jerk remain unverified. Never authorize hardware execution.";
    return out;
}

M6Report multiPoseAsOfflineM6(const ExperimentViewReport& view,
                              const ExperimentOptions& opts){
    M6Report m6;
    m6.options.scanRateHz=opts.profileRateHz;
    m6.options.focusSensorZmm=opts.focusSensorZmm;
    m6.options.bounds=opts.bounds;
    // Archived SCAN joints are historical; they are not a verified live NC
    // start pose and must not be recorded as an anchored initial G00 state.
    m6.options.hasInitialJoints=false;
    m6.options.initialJoints={};
    m6.path=view.path;
    // Use the same continuous physical SP planner as real G05 execution.
    if(opts.enforceHardwareG93){
        std::string timingIssue;
        if(!retimeReportG93(m6,opts,timingIssue))m6.error=timingIssue;
        else m6.g93PhysicalTimelinePlanned=true;
    }
    m6.kinematicsSource="MultiPoseExperiment : FK from recorded Physical17 / explicit callback";
    // The source M6NcPreview must remain unable to grant hardware authorization.
    m6.canExportForHardware=false;
    return m6;
}
} // namespace mls::twin
