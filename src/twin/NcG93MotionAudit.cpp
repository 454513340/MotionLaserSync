#include "twin/NcG93MotionAudit.h"
#include <cmath>
#include <string>
#include <vector>
#include <algorithm>
#include <iomanip>
#include <locale>
#include <sstream>

namespace mls::twin {
NcG93MotionAudit auditNcG93Motion(const NcSimResult& nc,
                                  const mls::runtime::Limits& limits,
                                  double accelerationCap,
                                  double decelerationCap) {
    NcG93MotionAudit out;
    if (!nc.ok()) {
        out.error = "NC must first pass exact offline parsing: " + nc.error;
        return out;
    }
    if (!std::isfinite(accelerationCap) || !std::isfinite(decelerationCap) ||
        accelerationCap<=0 || decelerationCap<=0) {
        out.error = "Invalid configured acceleration/deceleration";
        return out;
    }
    // A G05 S0 or S1 group is sent to prepareMergePath as ONE plan, then
    // controller buffering. Its first and final points must not rely on
    // velocity from an earlier M50/M51, G00, or M60 event.
    std::size_t from=1;
    while (from<nc.vertices.size()) {
        const bool scanning=nc.vertices[from].scanning;
        std::size_t to=from;
        std::vector<mls::runtime::Waypoint> points;
        while (to<nc.vertices.size() && nc.vertices[to].scanning==scanning) {
            const auto& previous=nc.vertices[to-1];
            const auto& point=nc.vertices[to];
            const double dt=point.seconds-previous.seconds;
            if (!std::isfinite(dt)||dt<=0||!std::isfinite(60.0/dt)) {
                out.error="Invalid exact G93 duration on NC line " + std::to_string(point.sourceLine);
                out.failedNcLine=point.sourceLine;
                return out;
            }
            // Match live GCodeExecutor::startMotion(): G05 knots may preserve
            // velocity only when their five-axis directions are collinear.
            // Exactly match GCodeExecutor: a group starts from live feedback,
            // not the ideal simulated previous NC point. Force one exact-stop
            // anchor at the first G05 knot before accelerating into blending.
            points.push_back({point.joints,60.0/dt,
                mls::runtime::FeedMode::InverseMinute,!points.empty()});
            ++to;
        }
        mls::runtime::Plan planned;
        std::string detail;
        if (!mls::runtime::plan(nc.vertices[from-1].joints,points,limits,
                                accelerationCap,decelerationCap,planned,detail)) {
            // Detailed planner errors carry the first failing segment's
            // index. Map it back to an ORIGINAL NC source line, not a new
            // pseudo-line emitted by this audit.
            std::size_t bad=0;
            const std::string marker=" at segment ";
            const auto pos=detail.find(marker);
            if (pos!=std::string::npos) {
                try {
                    const auto value=std::stoull(detail.substr(pos+marker.size()));
                    if(value>=1 && value<=points.size())bad=static_cast<std::size_t>(value-1);
                }catch(...){}
            }
            const auto line=nc.vertices[from+bad].sourceLine;
            out.failedNcLine=line;
            out.error="NC line " + std::to_string(line) + " (G05 " +
                (scanning?"S1 scan":"S0 transfer") + "): " + detail +
                ". Regenerate the NC with feasible G93 times and repeat all on-site checks; "
                "do not increase configured mechanical limits to silence this error.";
            return out;
        }
        // V4 regression: offline feasibility at exactly the nominal previous
        // point did NOT guarantee feasibility at the live DPOS/MPOS returned
        // after an accepted 0.05mm/0.05deg convergence. Check all 32 signed
        // endpoint extremes at each G05 group boundary, with the very SAME
        // planner that prepareMergePath uses. No controller writes here.
        constexpr mls::runtime::Joint convergence{0.05,0.05,0.05,0.05,0.05};
        for(unsigned mask=0;mask<32;++mask){
            auto actual=nc.vertices[from-1].joints;
            for(int j=0;j<5;++j){
                actual[j]+=mask&(1u<<j)?convergence[j]:-convergence[j];
                actual[j]=std::clamp(actual[j],limits.minimum[j],limits.maximum[j]);
            }
            mls::runtime::Plan driftPlan;
            if(!mls::runtime::plan(actual,points,limits,
                                    accelerationCap,decelerationCap,driftPlan,detail)){
                out.failedNcLine=nc.vertices[from].sourceLine;
                out.error="NC line "+std::to_string(out.failedNcLine)+
                    " (G05 live-start tolerance): G93 only worked at the exact "
                    "nominal NC point, not at a permitted 0.05mm/0.05deg "
                    "XYZCB DPOS/MPOS residual (corner mask="+std::to_string(mask)+") : "+
                    detail+". Regenerate the NC with V5 robust first-knot timing "
                    "before any M50 or machine motion.";
                return out;
            }
        }
        out.segmentsChecked+=points.size();
        ++out.groupsChecked;
        out.plannedSeconds+=planned.seconds;
        from=to;
    }
    out.nominalSeconds=nc.durationSeconds;
    out.passed=true;
    return out;
}
} // namespace mls::twin

namespace mls::twin {
NcG93RetimeCandidate retimeNcG93Continuous(const std::string& originalText,
                                            const NcSimResult& parsed,
                                            const mls::runtime::Limits& limits,
                                            double accelerationCap,
                                            double decelerationCap){
    NcG93RetimeCandidate out;
    if(!parsed.ok()){
        out.error="NC must be strictly parsed before G93 continuous retiming";
        return out;
    }
    std::vector<std::string> lines;
    std::size_t offset=0;
    while(offset<originalText.size()){
        const auto newline=originalText.find('\n',offset);
        lines.push_back(originalText.substr(offset,newline==std::string::npos?
            originalText.size()-offset:newline-offset));
        if(newline==std::string::npos)break;
        offset=newline+1;
    }
    if(lines.empty()){out.error="Empty NC text";return out;}
    std::size_t from=1;
    while(from<parsed.vertices.size()){
        const bool scan=parsed.vertices[from].scanning;
        std::size_t to=from;
        std::vector<mls::runtime::Waypoint> points;
        while(to<parsed.vertices.size() && parsed.vertices[to].scanning==scan){
            const double dt=parsed.vertices[to].seconds-parsed.vertices[to-1].seconds;
            if(!std::isfinite(dt)||dt<=0.){
                out.error="Invalid G93 interval in input NC";return out;
            }
            points.push_back({parsed.vertices[to].joints,60./dt,
                              mls::runtime::FeedMode::InverseMinute,true});
            ++to;
        }
        std::vector<double> physicalTimes;
        std::string why;
        if(!mls::runtime::retimeG93Polyline(parsed.vertices[from-1].joints,
            points,limits,accelerationCap,decelerationCap,physicalTimes,why)){
            out.error="G93 continuous group starting at NC line "+
                      std::to_string(parsed.vertices[from].sourceLine)+": "+why;
            return out;
        }
        for(std::size_t i=0;i<points.size();++i){
            const double oldTime=60./points[i].feed;
            const double newTime=physicalTimes[i];
            if(newTime+1e-9<oldTime){
                out.error="Continuous SP retime unexpectedly shortened original G93 duration";
                return out;
            }
            if(newTime<=oldTime+1e-8)continue;
            const std::size_t lineIndex=parsed.vertices[from+i].sourceLine;
            if(lineIndex==0||lineIndex>lines.size()){
                out.error="G93 retime NC source-line mapping failed";return out;
            }
            auto& body=lines[lineIndex-1];
            const auto f=body.find(" F");
            if(f==std::string::npos){out.error="G05 NC point lacks F";return out;}
            const std::size_t first=f+2;
            const auto end=body.find_first_of(" \r\t;",first);
            if(first==(end==std::string::npos?body.size():end)){
                out.error="Malformed F word";return out;
            }
            std::ostringstream token;
            token.imbue(std::locale::classic());
            token<<std::setprecision(16)<<(60.0/newTime);
            body.replace(first,(end==std::string::npos?body.size():end)-first,
                         token.str());
            ++out.changedMotionPoints;
        }
        from=to;
    }
    bool haveOffline=false;
    for(auto& line:lines){
        if(line.find("; OfflineOnly:")==0){
            line="; OfflineOnly: true";
            haveOffline=true;
            break;
        }
    }
    if(!haveOffline){out.error="OfflineOnly declaration is required for retimed NC";return out;}
    std::ostringstream saved;
    saved.imbue(std::locale::classic());
    for(const auto& line:lines){
        saved<<line<<'\n';
        if(line.find("; OfflineOnly:")==0)
            saved<<"; G93Planning: CONTINUOUS_SP_WITH_ROBUST_GROUP_START_V2\n"
                    "; G93PlanningStatus: ENGINEERING_CANDIDATE_NOT_MACHINE_AUTHORIZATION\n"
                    "; G93PlanningNotice: Changed NC requires full preflight and independent controller SP time verification\n";
    }
    out.text=saved.str();
    const auto replay=simulateMultiPoseNc(out.text,250000);
    if(!replay.ok()){
        out.error="Re-timed NC was rejected by the real NC interpreter: "+replay.error;
        out.text.clear();return out;
    }
    if(replay.vertices.size()!=parsed.vertices.size() ||
       replay.scanGroups!=parsed.scanGroups || replay.events.size()!=parsed.events.size()){
        out.error="Re-timing changed the NC geometry or acquisition topology";
        out.text.clear();return out;
    }
    for(std::size_t i=0;i<parsed.vertices.size();++i){
        if(parsed.vertices[i].joints!=replay.vertices[i].joints ||
           parsed.vertices[i].scanning!=replay.vertices[i].scanning){
            out.error="Re-timing modified XYZCB geometry or S0/S1 scan status";
            out.text.clear();return out;
        }
    }
    const auto audit=auditNcG93Motion(replay,limits,accelerationCap,decelerationCap);
    if(!audit.passed){
        out.error="Re-timed NC failed continuous physical G93 planning: "+audit.error;
        out.text.clear();return out;
    }
    out.originalSeconds=parsed.durationSeconds;
    out.retimedSeconds=replay.durationSeconds;
    return out;
}

NcG93RetimeCandidate retimeNcG93Conservative(const std::string& originalText,
                                              const NcSimResult& parsed,
                                              const mls::runtime::Limits& limits,
                                              double accelerationCap,
                                              double decelerationCap,
                                              double margin) {
    NcG93RetimeCandidate out;
    if (!parsed.ok()) {out.error="Source NC does not pass strict offline parsing";return out;}
    if (!std::isfinite(margin) || margin<0.01 || margin>0.5) {
        out.error="A 1%-50% explicit planning margin is required";return out;
    }
    std::vector<std::string> lines;
    std::size_t off=0;
    while (off<originalText.size()) {
        const auto next=originalText.find('\n',off);
        lines.push_back(originalText.substr(off,next==std::string::npos?
            originalText.size()-off:next-off));
        if(next==std::string::npos)break;
        off=next+1;
    }
    if(lines.empty()){out.error="Empty NC";return out;}
    std::size_t from=1;
    while(from<parsed.vertices.size()){
        const bool scanning=parsed.vertices[from].scanning;
        std::size_t to=from;
        std::vector<mls::runtime::Waypoint> unconstrainedTime;
        while(to<parsed.vertices.size() && parsed.vertices[to].scanning==scanning){
            // Large legacy feed value saturates at each configured axis speed,
            // without ever exceeding limits. False = independent exact stops.
            unconstrainedTime.push_back({parsed.vertices[to].joints,1.e7,
                mls::runtime::FeedMode::LegacyPerSecond,false});
            ++to;
        }
        mls::runtime::Plan minimum;
        std::string detail;
        if(!mls::runtime::plan(parsed.vertices[from-1].joints,unconstrainedTime,
            limits,accelerationCap,decelerationCap,minimum,detail) ||
           minimum.segments.size()!=unconstrainedTime.size()){
            out.error="Cannot produce conservative G93 times: " + detail;
            return out;
        }
        for(std::size_t k=0;k<unconstrainedTime.size();++k){
            const auto& current=parsed.vertices[from+k];
            const auto& previous=parsed.vertices[from+k-1];
            const double oldSeconds=current.seconds-previous.seconds;
            // G93 is currently exact-stop at EVERY point in the live SP
            // planner, including adjacent collinear points. The legacy
            // fastest-path planner blends collinear segments; do not reuse its
            // per-segment seconds as though G93 did the same. Re-plan each
            // point independently, with the entire group's conservative
            // projected acceleration/deceleration caps.
            mls::runtime::Plan exactStop;
            if(!mls::runtime::plan(previous.joints,{unconstrainedTime[k]},
                    limits,minimum.accel,minimum.decel,exactStop,detail) ||
               exactStop.segments.size()!=1){
                out.error="Cannot compute one-point G93 exact-stop time: " + detail;
                return out;
            }
            const double newSeconds=std::max(oldSeconds,exactStop.segments[0].seconds*(1+margin));
            if(!std::isfinite(newSeconds) || newSeconds<=0){
                out.error="Invalid retimed segment duration";return out;
            }
            if(newSeconds>oldSeconds+1.e-8){
                const auto row=current.sourceLine;
                if(row==0 || row>lines.size()){
                    out.error="NC source line mapping failed";return out;
                }
                auto& body=lines[row-1];
                // The strict NC parser ensures each G05 coordinate line has
                // one explicit F; do NOT change positions or S0/S1 semantics.
                const auto f=body.find(" F");
                if(f==std::string::npos){out.error="Missing F on G05 point";return out;}
                const auto begin=f+2;
                const auto end=body.find_first_of(" \r\t;",begin);
                if(end==begin){out.error="Empty G93 F";return out;}
                std::ostringstream replacement;
                replacement.imbue(std::locale::classic());
                replacement<<std::setprecision(15)<<(60.0/newSeconds);
                body.replace(begin,(end==std::string::npos?body.size():end)-begin,
                    replacement.str());
                ++out.changedMotionPoints;
            }
        }
        from=to;
    }
    bool haveOfflineHeader=false;
    for(auto& line:lines){
        if(line.find("; OfflineOnly:")==0){
            line="; OfflineOnly: true";
            haveOfflineHeader=true;
            break;
        }
    }
    if(!haveOfflineHeader){out.error="Missing OfflineOnly source header";return out;}
    std::ostringstream text;
    text.imbue(std::locale::classic());
    for(std::size_t i=0;i<lines.size();++i){
        text<<lines[i]<<'\n';
        if(lines[i].find("; OfflineOnly:")==0){
            text<<"; G93Retiming: CONSERVATIVE_EXACT_STOP_CANDIDATE_NOT_REAL_RUN_PERMIT\n"
                   "; G93RetimingNotice: 5-axis G93 feed was altered; repeat entire real machine 1-4/4 verification\n";
        }
    }
    out.text=text.str();
    const auto replay=simulateMultiPoseNc(out.text,200000);
    if(!replay.ok()){
        out.error="Retimed NC is not parsable/within offline profile budget: " + replay.error;
        out.text.clear();return out;
    }
    if(replay.vertices.size()!=parsed.vertices.size() ||
       replay.scanGroups!=parsed.scanGroups || replay.events.size()!=parsed.events.size()){
        out.error="Retiming changed motion/event count";out.text.clear();return out;
    }
    for(std::size_t i=0;i<replay.vertices.size();++i)
        if(replay.vertices[i].joints!=parsed.vertices[i].joints ||
           replay.vertices[i].scanning!=parsed.vertices[i].scanning){
            out.error="Retiming modified a joint target/scan role";
            out.text.clear();return out;
        }
    const auto verified=auditNcG93Motion(replay,limits,accelerationCap,decelerationCap);
    if(!verified.passed){
        out.error="Retimed NC dynamics audit failed: " + verified.error;
        out.text.clear();return out;
    }
    out.originalSeconds=parsed.durationSeconds;
    out.retimedSeconds=replay.durationSeconds;
    return out;
}
} // namespace mls::twin
