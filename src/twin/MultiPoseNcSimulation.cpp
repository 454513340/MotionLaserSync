#include "twin/MultiPoseNcSimulation.h"
#include "runtime/GCode5D.h"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <utility>

namespace mls::twin {
namespace {
bool finiteJoints(const Joint5& joints){
    for(double axis:joints)if(!std::isfinite(axis))return false;
    return true;
}
std::string trim(std::string s){
    const auto first=s.find_first_not_of(" \t\r\n");
    if(first==std::string::npos)return {};
    const auto last=s.find_last_not_of(" \t\r\n");
    return s.substr(first,last-first+1);
}
std::string up(std::string s){
    for(char& c:s)if(c>='a'&&c<='z')c=char(c-'a'+'A');
    return s;
}
std::string lineError(std::size_t line,const std::string& message){
    return "NC line "+std::to_string(line)+": "+message;
}
std::string axisError(std::size_t k,int axis,double wanted,double got){
    std::ostringstream s;s.precision(12);
    s<<"NC/planner mismatch at waypoint "<<k<<" axis "<<"XYZCB"[axis]
     <<" (planner="<<wanted<<", parsed NC="<<got<<")";
    return s.str();
}
}
NcSimResult simulateMultiPoseNc(const std::string& text,std::size_t maxProfiles){
    NcSimResult out;
    auto fail=[&](std::size_t row,const std::string& cause)->NcSimResult{
        out.error=lineError(row,cause);return out;
    };
    if(maxProfiles==0||maxProfiles>1000000)return fail(0,"invalid offline simulation profile budget");
    if(text.size()>64*1024*1024)return fail(0,"NC exceeds 64MiB offline input limit");
    mls::runtime::NcState modal;
    bool hdrOffline=false,hdrFrame=false,hdrHardware=false,hdrFrequency=false,hdrTopology=false;
    std::string sensorTriggerMode; // optional in historical files
    std::string legacyTriggerType; // H1-only external-edge metadata
    bool haveInitial=false,haveGroup=false,scanOn=false,groupExposure=false;
    bool hadM50=false,haveM63=false,havePendingSave=false;
    bool haveM30=false,bodyStarted=false;
    double time=0.,nextTrigger=0.;
    const double epsilon=1.e-8;
    std::istringstream source(text);
    std::string line;
    std::size_t row=0;
    // ParseNcBlock is the actual lexical / modal engine used elsewhere by V29.
    while(std::getline(source,line)){
        ++row;
        const auto cleaned=trim(line);
        if(cleaned.empty())continue;
        if(cleaned.front()==';' || cleaned.front()=='('){
            auto comment=cleaned;
            if(comment.front()==';')comment=trim(comment.substr(1));
            else if(comment.back()==')')comment=trim(comment.substr(1,comment.size()-2));
            const auto colon=comment.find(':');
            if(colon==std::string::npos)continue;
            const auto key=up(trim(comment.substr(0,colon)));
            const auto value=up(trim(comment.substr(colon+1)));
            const bool critical=key=="OFFLINEONLY"||key=="SYNCMODE"||key=="PROGRAMFRAME"||
                key=="HARDWARETOPOLOGY"||key=="HARDWAREFREQUENCYHZ"||key=="CMMR1EDGE"||
                key=="SENSORTRIGGERMODE"||key=="HARDWARETRIGGERTYPE";
            if(critical && bodyStarted)return fail(row,"critical NC metadata must appear before executable instructions");
            if(key=="OFFLINEONLY"){
                // This text replay is not a run permit. Treat true and false
                // identically for NC geometry/timing checking: V30.0.1 run
                // authorization is always independent of this editable marker.
                if(value!="TRUE" && value!="FALSE")
                    return fail(row,"OfflineOnly must be explicitly true or false");
                hdrOffline=true;
            }else if(key=="SYNCMODE"){
                if(value!="HARDWARE")return fail(row,"generated multi-pose NC must declare Hardware mode");
                hdrHardware=true;
            }else if(key=="PROGRAMFRAME"){
                if(value!="MACHINE_JOINTS_XYZCB")return fail(row,"only MACHINE_JOINTS_XYZCB is implemented; never reinterpret RTCP");
                hdrFrame=true;
            }else if(key=="HARDWARETOPOLOGY"){
                if(value=="H1")out.topology=1;
                else if(value=="H3")out.topology=3;
                else return fail(row,"H1 or H3 required; H2 retired");
                hdrTopology=true;
            }else if(key=="HARDWAREFREQUENCYHZ"){
                try{
                    std::size_t consumed=0;
                    const auto f=std::stod(value,&consumed);
                    if(consumed!=value.size()||!std::isfinite(f)||f<1||f>2000||std::floor(f)!=f)
                        return fail(row,"invalid exact nominal profile frequency");
                    out.profileRateHz=int(f);hdrFrequency=true;
                }catch(...){return fail(row,"invalid frequency number");}
            }else if(key=="SENSORTRIGGERMODE"){
                if(value!="HARDWAREEXTERNAL" && value!="SOFTWAREINTERNAL")
                    return fail(row,"SensorTriggerMode must be HardwareExternal or SoftwareInternal");
                sensorTriggerMode=value;
            }else if(key=="HARDWARETRIGGERTYPE"){
                if(value!="EDGERISING")return fail(row,"legacy HardwareTriggerType only supports EdgeRising");
                legacyTriggerType=value;
            }else if(key=="CMMR1EDGE"){
                if(value!="114" && value!="115")return fail(row,"H3 CmmR1Edge must be 114/115");
                out.h3Edge=std::stoi(value);
            }
            continue;
        }
        if(haveM30)return fail(row,"command following M30 is not allowed");
        // A strict front gate prevents stray unvalidated motions before headers.
        if(!hdrOffline||!hdrHardware||!hdrFrame||!hdrFrequency||!hdrTopology)
            return fail(row,"missing required offline NC header before executable blocks");
        bodyStarted=true;
        mls::runtime::NcBlock block;std::string parserError;
        if(!mls::runtime::parseNcBlock(line,modal,block,parserError))
            return fail(row,parserError);
        if(block.rtcp)return fail(row,"G43.4 RTCP is invalid in MACHINE_JOINTS_XYZCB NC");
        if(!block.absolute)return fail(row,"G91 is unsupported for this absolute XYZCB NC format");
        if(block.kind==mls::runtime::NcBlock::Kind::SetWorkPosition)
            return fail(row,"G92 work offset changes are not implemented in this offline interpreter");
        if(block.kind==mls::runtime::NcBlock::Kind::Dwell)
            return fail(row,"dwell semantics are not validated for scan/replay; reject instead of skipping");
        if(block.kind==mls::runtime::NcBlock::Kind::MCode){
            const int code=block.mcode;
            out.events.push_back({"M"+std::to_string(code),time,row});
            if(code==50){
                if(scanOn||!haveInitial||havePendingSave||haveM63)return fail(row,"M50 invalid capture transition");
                scanOn=true;groupExposure=false;hadM50=true;nextTrigger=time;
            }else if(code==51){
                if(!scanOn||!groupExposure)return fail(row,"M51 before a valid exposed G05 scan");
                scanOn=false;haveGroup=false;havePendingSave=true;
            }else if(code==60){
                if(scanOn||!havePendingSave)return fail(row,"M60 must follow M51, exactly one save per scan group");
                ++out.saveGroups;
            }else if(code==61){
                if(scanOn||!havePendingSave||out.saveGroups!=out.scanGroups)
                    return fail(row,"M61 requires saved, stopped capture group");
                havePendingSave=false;
            }else if(code==63){
                if(scanOn||havePendingSave||out.saveGroups!=out.scanGroups)
                    return fail(row,"M63 requires fully closed and saved capture groups");
                haveM63=true;
            }else if(code==30){
                if(scanOn||havePendingSave||!haveM63||!hadM50||out.saveGroups!=out.scanGroups)
                    return fail(row,"M30 requires M50/M51/M60/M61 and M63 completion");
                haveM30=true;
            }else return fail(row,"unsupported M code in NC simulation (M0/M2 are not silently skipped)");
            continue;
        }
        if(block.kind==mls::runtime::NcBlock::Kind::Empty || block.kind==mls::runtime::NcBlock::Kind::Modal)continue;
        if(block.kind!=mls::runtime::NcBlock::Kind::Motion)
            return fail(row,"unsupported NC block kind");
        if(block.motion==0){
            // Existing generator's initial G00 cannot be proven safe: no live
            // machine state is available. Park the virtual machine at that
            // reference pose; NO fictitious approach motion is animated.
            if(haveInitial||scanOn||block.axisMask!=31||haveM63)
                return fail(row,"only a single full-axis initial G00 reference is supported");
            if(!finiteJoints(block.words))return fail(row,"initial XYZCB contains NaN/Inf");
            out.vertices.push_back({block.words,0.,false,row});haveInitial=true;
            continue;
        }
        if(block.motion!=5)return fail(row,"NC simulator supports only generated G05 motion; G01/G02/G03 must not be approximated");
        if(!haveInitial||haveM63)return fail(row,"G05 outside anchored offline trace");
        if(block.feedMode!=mls::runtime::FeedMode::InverseMinute)
            return fail(row,"G05 requires G93 inverse-time F per interpolated point");
        if(block.axisMask==0){
            // Header G05 S0/S1, no motion yet. Ensure S was explicitly set;
            // parseNcBlock defaults missing S to true for modal axis lines.
            std::vector<mls::runtime::Word> words;std::string lexError;
            if(!mls::runtime::lex(line,words,lexError))return fail(row,lexError);
            bool hasS=false;
            for(const auto& w:words)if(w.letter=='S')hasS=true;
            if(!hasS)return fail(row,"G05 group needs explicit S0 or S1");
            if(scanOn!=block.capture)return fail(row,"G05 S0/S1 disagrees with M50/M51 state");
            haveGroup=true;
            if(scanOn)++out.scanGroups;
            continue;
        }
        if(!haveGroup||block.axisMask!=31)
            return fail(row,"G05 points require active header and explicit XYZCB");
        if(!block.feed||!std::isfinite(*block.feed)||*block.feed<=0)
            return fail(row,"each G93 XYZCB motion requires its own positive F");
        const double dt=60.0/(*block.feed);
        if(!std::isfinite(dt)||dt<1.e-8||dt>3600.)return fail(row,"invalid G93 point duration (0 or excessive)");
        Joint5 q=block.words;
        if(!finiteJoints(q))return fail(row,"nonfinite XYZCB point");
        if(out.vertices.size()>=500000)return fail(row,"NC motion point safety budget exceeded");
        const auto& start=out.vertices.back();
        const double t0=time;
        const double t1=time+dt;
        if(!std::isfinite(t1)||t1>24*3600.)return fail(row,"NC exceeds 24h offline simulation budget");
        if(scanOn){
            groupExposure=true;
            const double step=1.0/double(out.profileRateHz);
            while(nextTrigger<=t1+epsilon){
                if(nextTrigger<t0-epsilon){
                    return fail(row,"invalid scan trigger phase");
                }
                if(out.profiles.size()>=maxProfiles)return fail(row,"simulation profile budget exceeded");
                const double u=std::clamp((nextTrigger-t0)/dt,0.,1.);
                Joint5 sampled{};
                for(int j=0;j<5;++j)sampled[j]=start.joints[j]+(q[j]-start.joints[j])*u;
                out.profiles.push_back({sampled,nextTrigger,out.scanGroups,row});
                nextTrigger+=step;
            }
        }
        time=t1;
        out.vertices.push_back({q,time,scanOn,row});
    }
    if(!hdrOffline||!hdrHardware||!hdrFrame||!hdrFrequency||!hdrTopology)return fail(row,"mandatory header missing");
    if(out.topology==3 && out.h3Edge!=114 && out.h3Edge!=115)return fail(row,"H3 requires explicit CmmR1Edge:114/115");
    if(!sensorTriggerMode.empty() && sensorTriggerMode !=
        (out.topology==1?"HARDWAREEXTERNAL":"SOFTWAREINTERNAL"))
        return fail(row,"SensorTriggerMode contradicts H1/H3 topology");
    // Legacy EdgeRising in H3 never overrides its internal sensor clock.
    (void)legacyTriggerType;
    if(out.topology==1 && out.h3Edge!=0)return fail(row,"H1 must not contain CmmR1Edge");
    if(!haveM30)return fail(row,"missing mandatory M30");
    out.reachedM30=haveM30;out.sawM63=haveM63;out.durationSeconds=time;
    out.warning="OFFLINE ideal NC interpolation: first G00's approach is UNKNOWN; G93 linear joint segments, "
                "nominal periodic H1/H3 exposures only. Hardware acceleration, latency and collision clearance NOT proven.";
    if(out.vertices.size()<2||out.profiles.empty())return fail(row,"empty NC scan or no valid trigger time");
    return out;
}

std::string verifyMultiPoseNcAgainstPlan(const NcSimResult& nc,const M6Report& plan,
                                         double jt,double tt){
    if(!nc.ok())return nc.error.empty()?"NC replay not complete":nc.error;
    if(!plan.ok()||plan.path.empty())return "planner reference is invalid";
    if(!std::isfinite(jt)||!std::isfinite(tt)||jt<=0||tt<=0)return "invalid NC/planner comparison tolerance";
    std::size_t i=0;
    for(std::size_t k=0;k<plan.path.size();++k){
        const auto& p=plan.path[k];
        if(k && p.seconds-plan.path[k-1].seconds<1e-8){
            bool same=true;
            for(int j=0;j<5;++j)same=same&&std::abs(p.joints[j]-plan.path[k-1].joints[j])<=jt;
            if(!same)return "planner contains nonzero joint jump at zero dt";
            continue;
        }
        if(i>=nc.vertices.size())return "NC is missing planner waypoints";
        const auto& n=nc.vertices[i];
        for(int j=0;j<5;++j)if(std::abs(n.joints[j]-p.joints[j])>jt)
            return axisError(k,j,p.joints[j],n.joints[j]);
        const double elapsed=p.seconds-plan.path.front().seconds;
        if(std::abs(n.seconds-elapsed)>tt)
            return "NC G93 timing mismatch at planner waypoint "+std::to_string(k);
        if(i>0 && n.scanning!=(p.role==M6SegmentRole::Scan))
            return "NC exposure/transfer segment mismatch at planner waypoint "+std::to_string(k);
        ++i;
    }
    if(i!=nc.vertices.size())return "NC contains unplanned extra five-axis motion waypoints";
    if(nc.scanGroups==0||nc.scanGroups!=nc.saveGroups)return "NC scan/save group mismatch";
    if(nc.profileRateHz!=int(std::lround(plan.options.scanRateHz)))return "NC frequency changed from planner";
    return {};
}

NcSimSample sampleMultiPoseNc(const NcSimResult& nc,double at){
    NcSimSample out;
    if(!nc.ok()||!std::isfinite(at))return out;
    const double t=std::clamp(at,0.,nc.durationSeconds);
    auto found=std::lower_bound(nc.vertices.begin(),nc.vertices.end(),t,
        [](const NcSimVertex& v,double s){return v.seconds<s;});
    const std::size_t index=std::min(std::size_t(found-nc.vertices.begin()),nc.vertices.size()-1);
    const auto& b=nc.vertices[index];
    out.joints=b.joints;
    if(index>0){
        const auto& a=nc.vertices[index-1];
        const double d=b.seconds-a.seconds;
        const double u=d>0?std::clamp((t-a.seconds)/d,0.,1.):1.;
        for(int j=0;j<5;++j)out.joints[j]=a.joints[j]+(b.joints[j]-a.joints[j])*u;
        out.scanning=b.scanning && u>0.;
    }
    auto exposure=std::upper_bound(nc.profiles.begin(),nc.profiles.end(),t+1.e-8,
       [](double s,const NcSimProfile& e){return s<e.seconds;});
    out.passedProfiles=std::size_t(exposure-nc.profiles.begin());
    out.nextVertex=index;out.seconds=t;out.valid=true;
    return out;
}
}
