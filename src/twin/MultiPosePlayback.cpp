#include "twin/MultiPosePlayback.h"
#include <algorithm>
#include <cmath>
namespace mls::twin {
MultiPosePlaybackTimeline makeMultiPosePlaybackTimeline(const ExperimentPlan& plan,double pause){
    MultiPosePlaybackTimeline out;
    if(!plan.ok()){out.error="Planning result is empty or invalid";return out;}
    if(!std::isfinite(pause)||pause<0||pause>30){out.error="Invalid session separator";return out;}
    double offset=0;
    for(std::size_t k=0;k<plan.views.size();++k){
        const auto& path=plan.views[k].path;
        if(path.empty()){out.error="Pose has no timeline";out.segments.clear();return out;}
        const double base=path.front().seconds,duration=path.back().seconds-base;
        if(!std::isfinite(base)||!std::isfinite(duration)||duration<0){
            out.error="Invalid NC motion time";out.segments.clear();return out;
        }
        for(std::size_t j=1;j<path.size();++j){
            if(!std::isfinite(path[j].seconds)||path[j].seconds<path[j-1].seconds){
                out.error="Nonmonotonic NC path";out.segments.clear();return out;
            }
        }
        if(k>0)offset+=pause;
        out.segments.push_back({k,offset,offset+duration,duration});
        offset+=duration;
    }
    out.totalSeconds=offset;
    return out;
}
MultiPosePlaybackTimeline makeNcDrivenPlaybackTimeline(const std::vector<NcSimResult>& runs,double pause){
    MultiPosePlaybackTimeline out;
    if(runs.empty()){out.error="No NC files available";return out;}
    if(!std::isfinite(pause)||pause<0||pause>30){out.error="Invalid NC session separator";return out;}
    double offset=0;
    for(std::size_t k=0;k<runs.size();++k){
        const auto& nc=runs[k];
        if(!nc.ok()){out.error="Pose "+std::to_string(k)+": "+nc.error;out.segments.clear();return out;}
        if(k>0)offset+=pause;
        out.segments.push_back({k,offset,offset+nc.durationSeconds,nc.durationSeconds});
        offset+=nc.durationSeconds;
    }
    out.totalSeconds=offset;return out;
}
MultiPosePlaybackState sampleNcDrivenPlayback(const std::vector<NcSimResult>& runs,
    const MultiPosePlaybackTimeline& timeline,double time,const M6ForwardKinematics& fk){
    MultiPosePlaybackState out;
    if(!timeline.ok()||!fk||!std::isfinite(time))return out;
    out.globalSeconds=std::clamp(time,0.,timeline.totalSeconds);
    const MultiPosePlaybackSegment* seg=&timeline.segments.front();
    for(const auto& candidate:timeline.segments){
        if(out.globalSeconds<candidate.globalStart)break;
        seg=&candidate;
        if(out.globalSeconds<=candidate.globalEnd)break;
    }
    if(seg->poseIndex>=runs.size())return out;
    out.poseIndex=seg->poseIndex;
    out.localSeconds=std::clamp(out.globalSeconds-seg->globalStart,0.,seg->localEnd);
    const auto nc=sampleMultiPoseNc(runs[seg->poseIndex],out.localSeconds);
    if(!nc.valid)return out;
    out.joints=nc.joints;out.scanning=nc.scanning;
    out.pathIndex=nc.nextVertex;out.exposureCount=nc.passedProfiles;
    out.sensorToMachine=fk(out.joints);
    for(const double v:out.sensorToMachine.m)if(!std::isfinite(v))return out;
    out.valid=true;return out;
}
MultiPosePlaybackTimeline makeConnectedNcPlaybackTimeline(
    const ConnectedExperimentSession& connected,const NcSimResult& nc){
    MultiPosePlaybackTimeline out;
    if(!connected.ok()||!nc.ok()||connected.poseStartSeconds.size()!=connected.poseEndSeconds.size()){
        out.error="Cannot make time axis from unverified connected NC";return out;
    }
    const auto n=connected.poseStartSeconds.size();
    for(std::size_t k=0;k<n;++k){
        const double begin=connected.poseStartSeconds[k];
        const double end=connected.poseEndSeconds[k];
        if(!std::isfinite(begin)||!std::isfinite(end)||begin<0||end<=begin||
           (k>0&&begin<=connected.poseEndSeconds[k-1]+1.e-8)){
            out.error="Connected NC is missing a positive-time inter-pose move";
            out.segments.clear();return out;
        }
        out.segments.push_back({k,begin,end,end-begin});
    }
    if(std::abs(nc.durationSeconds-out.segments.back().globalEnd)>1.e-3){
        out.error="Connected NC duration disagrees with session plan";
        out.segments.clear();return out;
    }
    out.totalSeconds=nc.durationSeconds;
    return out;
}
MultiPosePlaybackState sampleConnectedNcPlayback(const NcSimResult& nc,
    const MultiPosePlaybackTimeline& timeline,double time,
    const M6ForwardKinematics& fk){
    MultiPosePlaybackState out;
    if(!timeline.ok()||!nc.ok()||!fk||!std::isfinite(time))return out;
    const double at=std::clamp(time,0.,timeline.totalSeconds);
    const auto sampled=sampleMultiPoseNc(nc,at);
    if(!sampled.valid)return out;
    std::size_t index=timeline.segments.size()-1;
    bool movingBetween=false;
    for(std::size_t i=0;i<timeline.segments.size();++i){
        if(at<timeline.segments[i].globalStart-1.e-9){
            index=i;movingBetween=i>0;break;
        }
        if(at<=timeline.segments[i].globalEnd+1.e-9){index=i;break;}
    }
    const auto& seg=timeline.segments[index];
    out.globalSeconds=at;out.poseIndex=index;
    out.localSeconds=movingBetween?0.:
        std::clamp(at-seg.globalStart,0.,seg.localEnd);
    out.betweenPoses=movingBetween;
    out.previousPoseIndex=index>0?index-1:0;
    out.joints=sampled.joints;
    out.scanning=sampled.scanning&&!movingBetween;
    out.pathIndex=sampled.nextVertex;
    if(!movingBetween){
        const auto begin=std::lower_bound(nc.profiles.begin(),nc.profiles.end(),
            seg.globalStart-1.e-8,[](const NcSimProfile& p,double t){return p.seconds<t;});
        const auto end=std::upper_bound(begin,nc.profiles.end(),at+1.e-8,
            [](double t,const NcSimProfile& p){return t<p.seconds;});
        out.exposureCount=std::size_t(end-begin);
    }
    out.sensorToMachine=fk(out.joints);
    for(double v:out.sensorToMachine.m)if(!std::isfinite(v))return {};
    out.valid=true;
    return out;
}
MultiPosePlaybackState sampleMultiPosePlayback(const ExperimentPlan& plan,
    const MultiPosePlaybackTimeline& timeline,double t,const M6ForwardKinematics& fk){
    MultiPosePlaybackState out;
    if(!timeline.ok()||!fk||!std::isfinite(t))return out;
    out.globalSeconds=std::clamp(t,0.,timeline.totalSeconds);
    // The visual gap is parked on the PRIOR session's last known pose;
    // it is NOT interpreted as an unplanned C/B motion between sessions.
    const MultiPosePlaybackSegment* seg=&timeline.segments.front();
    for(const auto& candidate:timeline.segments){
        if(out.globalSeconds<candidate.globalStart)break;
        seg=&candidate;
        if(out.globalSeconds<=candidate.globalEnd)break;
    }
    out.poseIndex=seg->poseIndex;
    const auto& view=plan.views[seg->poseIndex];
    const auto& path=view.path;
    const double local=std::clamp(out.globalSeconds-seg->globalStart,0.,seg->localEnd);
    out.localSeconds=local;
    const double target=path.front().seconds+local;
    auto it=std::lower_bound(path.begin(),path.end(),target,
        [](const M6PathSample& p,double time){return p.seconds<time;});
    std::size_t index=std::size_t(it-path.begin());
    if(index>=path.size())index=path.size()-1;
    out.pathIndex=index;
    if(index==0){out.joints=path.front().joints;out.scanning=false;}
    else{
        const auto& a=path[index-1],&b=path[index];
        const double dt=b.seconds-a.seconds;
        const double u=dt>0?std::clamp((target-a.seconds)/dt,0.,1.):1.;
        for(int axis=0;axis<5;++axis)out.joints[axis]=a.joints[axis]+(b.joints[axis]-a.joints[axis])*u;
        // A path sample's role denotes the INCOMING segment, not the
        // previous vertex's outgoing role.
        out.scanning=b.role==M6SegmentRole::Scan&&u>0.;
    }
    out.sensorToMachine=fk(out.joints);
    for(double v:out.sensorToMachine.m)if(!std::isfinite(v))return out;
    auto ex=std::upper_bound(view.exposures.begin(),view.exposures.end(),target,
        [](double time,const VirtualProfilePose& p){return time<p.elapsedSeconds;});
    out.exposureCount=std::size_t(ex-view.exposures.begin());
    out.valid=true;
    return out;
}
}
