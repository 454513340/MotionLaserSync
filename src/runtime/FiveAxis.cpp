#include "runtime/FiveAxis.h"
#include <algorithm>
#include <cmath>
#include <cctype>
#include <charconv>
#include <system_error>
#include <cerrno>
#include <limits>
#include <iomanip>
#include <sstream>

namespace mls::runtime {
namespace {
constexpr double pi=3.1415926535897932384626433832795;
bool finite(const Joint& q) { for(double v:q) if(!std::isfinite(v)) return false; return true; }
double norm(const Joint& q) { double r=0; for(double v:q) r=std::hypot(r,v); return r; }
double duration(double l,double v,double v0,double v1,double a,double d) {
    const double peak=std::sqrt((2*l*a*d+d*v0*v0+a*v1*v1)/(a+d));
    v=std::min(v,peak);
    const double sa=std::max(0.0,(v*v-v0*v0)/(2*a));
    const double sd=std::max(0.0,(v*v-v1*v1)/(2*d));
    return std::max(0.0,(v-v0)/a)+std::max(0.0,(v-v1)/d)+std::max(0.0,l-sa-sd)/v;
}
// A preceding G00/M51 is qualified by the executor's MPOS tolerance, NOT
// guaranteed to finish at the infinitely precise NC coordinate. XYZ DPOS
// and B/C MPOS are also independently measured. For the first motion in each
// G05 group we do not assume the next NC segment stays perfectly collinear
// with the actual start pose. Bound the cold-start exact-stop time against
// all 2^5 extremes of the allowed five-axis starting-position residuals.
// Units follow Joint: XYZ in mm, C/B in degrees.
// This only ADDS TIME to G93; it never raises the machine's speed/accel caps.
double worstCaseFirstKnotSeconds(const Joint& nominalStart, const Joint& firstTarget,
                                 const Limits& limits, double vectorAccelCap,
                                 double vectorDecelCap) {
    constexpr Joint initialUncertainty{0.05,0.05,0.05,0.05,0.05};
    double worst=0.0;
    for(unsigned mask=0;mask<32;++mask){
        Joint source=nominalStart;
        for(int j=0;j<5;++j){
            const double variation=(mask&(1u<<j))?initialUncertainty[j]:-initialUncertainty[j];
            source[j]+=variation;
            // Physical position cannot be outside hard software axis bounds.
            source[j]=std::clamp(source[j],limits.minimum[j],limits.maximum[j]);
        }
        Joint delta{};
        for(int j=0;j<5;++j)delta[j]=firstTarget[j]-source[j];
        const double length=norm(delta);
        if(length<=1e-12)continue;
        double vcap=std::numeric_limits<double>::max();
        double acap=vectorAccelCap,dcap=vectorDecelCap;
        for(int j=0;j<5;++j){
            const double ratio=std::abs(delta[j])/length;
            if(ratio<=1e-14)continue;
            vcap=std::min(vcap,limits.speed[j]/ratio);
            acap=std::min(acap,limits.accel[j]/ratio);
            dcap=std::min(dcap,limits.decel[j]/ratio);
        }
        worst=std::max(worst,duration(length,vcap,0.,0.,acap,dcap));
    }
    return worst;
}
double sweep(double a,double b,bool cw) {
    double x=b-a;
    if(cw) { while(x>=-1e-12) x-=2*pi; }
    else { while(x<=1e-12) x+=2*pi; }
    return x;
}
}
bool lex(const std::string& line,std::vector<Word>& words,std::string& error) {
    words.clear(); error.clear(); std::size_t i=0;
    while(i<line.size()) {
        unsigned char ch=static_cast<unsigned char>(line[i]);
        if(std::isspace(ch)) {++i;continue;}
        if(ch==';') break;
        if(ch=='(') {
            auto end=line.find(')',i+1);
            if(end==std::string::npos) {error="Unclosed comment";return false;}
            i=end+1;continue;
        }
        if(ch=='%' && words.empty()) { ++i; continue; }
        if(!std::isalpha(ch)) {error="Unexpected character at column "+std::to_string(i+1);return false;}
        Word w; w.letter=static_cast<char>(std::toupper(ch)); ++i;
        while(i<line.size() && std::isspace(static_cast<unsigned char>(line[i]))) ++i;
        if(i<line.size() && line[i]=='"') {
            auto end=line.find('"',i+1);
            if(end==std::string::npos) {error="Unclosed string";return false;}
            w.text=true; w.string=line.substr(i+1,end-i-1);i=end+1;
        } else {
            const std::size_t begin=i;
            if(i<line.size() && (line[i]=='+'||line[i]=='-'))++i;
            bool digits=false;
            while(i<line.size() && std::isdigit(static_cast<unsigned char>(line[i]))){++i;digits=true;}
            if(i<line.size()&&line[i]=='.'){++i;while(i<line.size()&&std::isdigit(static_cast<unsigned char>(line[i]))){++i;digits=true;}}
            if(i<line.size()&&(line[i]=='e'||line[i]=='E')){
                ++i;if(i<line.size()&&(line[i]=='+'||line[i]=='-'))++i;
                const auto exponent=i;while(i<line.size()&&std::isdigit(static_cast<unsigned char>(line[i])))++i;
                if(exponent==i){error="Invalid decimal exponent";return false;}
            }
            if(!digits){error="Numeric word requires decimal digits";return false;}
            const char* first=line.data()+begin;if(*first=='+')++first;
            auto parsed=std::from_chars(first,line.data()+i,w.number,std::chars_format::general);
            if(parsed.ec!=std::errc{}||parsed.ptr!=line.data()+i||!std::isfinite(w.number)){
                error="Invalid finite decimal word "+std::string(1,w.letter);return false;
            }
        }
        words.push_back(w);
    }
    return true;
}
bool resolve(const Joint& current,const Joint& words,std::uint8_t mask,bool absolute,
             Joint& target,std::string& error) {
    if(!finite(current) || !finite(words) || (mask&~31u)) {error="Invalid joint/axis mask";return false;}
    target=current;
    for(std::size_t i=0;i<5;++i) if(mask&(1u<<i)) target[i]=absolute?words[i]:current[i]+words[i];
    if(!finite(target)) {error="Joint arithmetic overflow";return false;} return true;
}
bool plan(const Joint& start,const std::vector<Waypoint>& path,const Limits& limits,
          double acap,double dcap,Plan& result,std::string& error) {
    result={}; error.clear();
    if(!finite(start) || path.empty() || path.size()>200000 || !std::isfinite(acap) ||
       !std::isfinite(dcap) || acap<=0 || dcap<=0) {error="Invalid start/path/acceleration";return false;}
    for(int j=0;j<5;++j) if(!std::isfinite(limits.speed[j]) || limits.speed[j]<=0 ||
       !std::isfinite(limits.accel[j]) || limits.accel[j]<=0 ||
       !std::isfinite(limits.decel[j]) || limits.decel[j]<=0 ||
       !std::isfinite(limits.minimum[j]) || !std::isfinite(limits.maximum[j]) ||
       limits.minimum[j]>limits.maximum[j] || start[j]<limits.minimum[j]-1e-6 ||
       start[j]>limits.maximum[j]+1e-6) {error="Invalid per-axis limits or start outside limits";return false;}
    Plan staged; staged.accel=acap;staged.decel=dcap; Joint previous=start;
    std::vector<double> inverseTimes;
    std::vector<bool> continuous;
    for(const auto& p:path) {
        if(!finite(p.q) || !std::isfinite(p.feed) || p.feed<=0) {error="Non-finite target or nonpositive F";return false;}
        Segment s; s.target=p.q; Joint delta{};
        for(int j=0;j<5;++j) {
            if(p.q[j]<limits.minimum[j]-1e-6 || p.q[j]>limits.maximum[j]+1e-6) {
                error="Target exceeds axis "+std::to_string(j+1)+" soft limit";return false;
            }
            delta[j]=p.q[j]-previous[j];
        }
        previous=p.q; s.length=norm(delta);
        if(s.length<=1e-12) continue;
        double maxv=std::numeric_limits<double>::max();
        for(int j=0;j<5;++j) {
            s.direction[j]=delta[j]/s.length; double r=std::abs(s.direction[j]);
            if(r>1e-14) {
                maxv=std::min(maxv,limits.speed[j]/r);
                staged.accel=std::min(staged.accel,limits.accel[j]/r);
                staged.decel=std::min(staged.decel,limits.decel[j]/r);
            }
        }
        double t=0;
        if(p.mode==FeedMode::InverseMinute) t=60.0/p.feed;
        else {
            double measure=std::hypot(std::hypot(delta[0],delta[1]),delta[2]);
            // Pure rotary blocks use deg/s (legacy) or deg/min (G94), explicitly.
            if(measure<=1e-12) measure=std::hypot(delta[3],delta[4]);
            t=measure/(p.feed/(p.mode==FeedMode::PerMinute?60.0:1.0));
        }
        if(!std::isfinite(t) || t<=0) {error="Invalid block duration";return false;}
        s.speed=p.mode==FeedMode::InverseMinute?maxv:std::min(maxv,s.length/t);
        staged.segments.push_back(s);
        inverseTimes.push_back(p.mode==FeedMode::InverseMinute?t:0.0);
        continuous.push_back(p.continuous);
    }
    auto& seg=staged.segments;
    // The NC G05 points are interpolation knots, not stop commands.  Preserve
    // velocity only for essentially collinear five-axis motion; a non-collinear
    // corner is an exact stop so the STEP-tested polyline is not rounded away.
    // A G93 point is allowed to blend ONLY when explicitly marked continuous.
    std::vector<double> junctionMax(seg.size()+1,0.0);
    for(std::size_t i=1;i<seg.size();++i) {
        double difference=0,dot=0;
        for(int j=0;j<5;++j){
            difference=std::hypot(difference,seg[i-1].direction[j]-seg[i].direction[j]);
            dot+=seg[i-1].direction[j]*seg[i].direction[j];
        }
        const bool timed=(inverseTimes[i-1]>0 || inverseTimes[i]>0);
        // A change of continuous-mode is an explicit junction boundary even
        // in the legacy speed-envelope pass used for G93 retiming. Previous
        // code only honored this for G93 and accidentally let the nominal
        // retimer accelerate through the required first-knot anchor.
        if(continuous[i-1]!=continuous[i])continue;
        if(timed && (!continuous[i-1] || !continuous[i]))continue;
        if(difference<1e-7){
            junctionMax[i]=std::min(seg[i-1].speed,seg[i].speed);
        }else if(!timed && continuous[i-1] && continuous[i]){
            dot=std::clamp(dot,-1.0,1.0);
            const double angle=std::acos(dot);
            if(angle<20.0*pi/180.0)
                junctionMax[i]=std::max(0.10,std::cos(angle)) *
                    std::min(seg[i-1].speed,seg[i].speed);
        }
    }
    auto reachable=[&](double factor) {
        std::vector<double> j=junctionMax;
        for(std::size_t i=1;i+1<j.size();++i)j[i]*=factor;
        // The previous version initialized ALL junctions to 0 and then used
        // min(0,reachableSpeed), so the look-ahead passes could never blend.
        for(std::size_t i=0;i<seg.size();++i)
            j[i+1]=std::min(j[i+1],std::sqrt(j[i]*j[i]+2*staged.accel*seg[i].length));
        for(std::size_t i=seg.size();i>0;--i)
            j[i-1]=std::min(j[i-1],std::sqrt(j[i]*j[i]+2*staged.decel*seg[i-1].length));
        return j;
    };
    std::vector<double> junction=reachable(1.0);
    const bool timed=std::any_of(inverseTimes.begin(),inverseTimes.end(),
                                 [](double t){return t>0;});
    if(timed){
        // Find ONE globally consistent knot-velocity scale for the G05 group.
        // A time-constrained segment can need LESS than the maximum possible
        // junction speed. Do not insert implicit dwells or silently change F.
        // The NC exporter constructs its timestamps using the same SP envelope.
        bool found=false;
        for(int k=0;k<=1000;++k){
            auto trial=reachable(1.0-0.001*double(k));
            bool valid=true;
            for(std::size_t i=0;i<seg.size();++i){
                if(inverseTimes[i]<=0)continue;
                const double v0=trial[i],v1=trial[i+1];
                const double vmin=std::max(v0,v1);
                if(vmin>seg[i].speed+1e-8){valid=false;break;}
                const double fastest=duration(seg[i].length,seg[i].speed,
                                              v0,v1,staged.accel,staged.decel);
                if(fastest>inverseTimes[i]+1e-8){valid=false;break;}
                if(vmin>1e-10){
                    const double slowest=duration(seg[i].length,vmin,
                                                  v0,v1,staged.accel,staged.decel);
                    if(slowest+1e-8<inverseTimes[i]){valid=false;break;}
                }
            }
            if(valid){junction=std::move(trial);found=true;break;}
        }
        if(!found){
            // Give the FIRST failing NC knot, its original F and the minimum
            // exact-stop time as actionable diagnostics; never issue motion.
            for(std::size_t i=0;i<seg.size();++i){
                if(inverseTimes[i]<=0)continue;
                const auto& s=seg[i];
                const double slowest=duration(s.length,s.speed,0,0,
                                               staged.accel,staged.decel);
                if(inverseTimes[i]+1e-8>=slowest)continue;
                std::ostringstream diagnostic;
                diagnostic.imbue(std::locale::classic());
                diagnostic<<std::fixed<<std::setprecision(6)
                    <<"G93 time infeasible under axis velocity/acceleration limits"
                    <<" at segment "<<(i+1)
                    <<": requested="<<inverseTimes[i]<<"s"
                    <<", exact-stop minimum="<<slowest<<"s"
                    <<". Straight G05 knots support continuous SP velocity;"
                    <<" first/last ramps and corners need feasible G93 times.";
                error=diagnostic.str();return false;
            }
            error="G93 knot times have no shared, physically attainable SP "
                  "entry/exit-velocity schedule within current axis limits "
                  "(request is either too short or too long for adjacent knots)";
            return false;
        }
    }
    for(std::size_t i=0;i<seg.size();++i){
        auto& s=seg[i];s.entry=junction[i];s.exit=junction[i+1];
        if(inverseTimes[i]>0){
            const double targetTime=inverseTimes[i];
            const double minimum=duration(s.length,s.speed,s.entry,s.exit,
                                          staged.accel,staged.decel);
            if(minimum>targetTime+1e-8){
                error="G93 segment time is shorter than planned acceleration envelope";
                return false;
            }
            // The scalar FORCE_SPEED is the trapezoid peak, NOT its average.
            // Bracket the peak between the entry/exit velocities and axis cap.
            double lo=std::max({s.entry,s.exit,1e-12}),hi=s.speed;
            for(int k=0;k<70;++k){
                const double peak=(lo+hi)*0.5;
                if(duration(s.length,peak,s.entry,s.exit,
                            staged.accel,staged.decel)>targetTime)lo=peak;
                else hi=peak;
            }
            s.speed=(lo+hi)*0.5;
        }
        s.seconds=duration(s.length,s.speed,s.entry,s.exit,
                           staged.accel,staged.decel);
        if(!std::isfinite(s.seconds) || s.seconds<=0){error="Invalid planned duration";return false;}
        if(inverseTimes[i]>0 && std::abs(s.seconds-inverseTimes[i])>2e-7){
            error="G93 SP timing solver did not converge to requested duration";return false;
        }
        staged.seconds+=s.seconds;
    }
    result=std::move(staged);return true;
}
bool retimeG93Polyline(const Joint& start,
                       const std::vector<Waypoint>& original,
                       const Limits& limits,double vectorAccelCap,
                       double vectorDecelCap,std::vector<double>& seconds,
                       std::string& error,double speedReserve) {
    seconds.clear();error.clear();
    if(original.empty() || original.size()>200000 ||
       !std::isfinite(speedReserve) || speedReserve<=0.0 || speedReserve>=1.0){
        error="Invalid G93 retiming inputs";return false;
    }
    // Rebuild the requested geometric feed as a continuously moving, physical
    // trapezoidal SP envelope. Reserve velocity headroom rather than reducing
    // physical limits or inserting an exact stop at every NC interpolation knot.
    // Smaller reserves are fallback choices if adjacent slow/corner knots need
    // more room for an exact, realizable inverse-time schedule.
    for(double factor:speedReserve>=0.95
                       ?std::vector<double>{speedReserve,0.93,0.90,0.85,0.80,0.70,0.60,0.50}
                       :std::vector<double>{speedReserve,speedReserve*0.90,
                                           speedReserve*0.80,speedReserve*0.70}) {
        Limits reserved=limits;
        for(double& v:reserved.speed)v*=factor;
        std::vector<Waypoint> natural;
        natural.reserve(original.size());
        Joint previous=start;
        bool valid=true;
        for(const auto& p:original){
            const double dt=p.mode==FeedMode::InverseMinute?60.0/p.feed:0.0;
            if(!finite(p.q) || !std::isfinite(dt) || dt<=0.0){
                error="G93 polyline requires explicit positive F and finite XYZCB";
                return false;
            }
            Joint delta{};
            for(int j=0;j<5;++j)delta[j]=p.q[j]-previous[j];
            const double step=norm(delta);
            if(step<=1e-12){error="G93 retime cannot preserve zero-motion timed point";return false;}
            double measure=std::hypot(std::hypot(delta[0],delta[1]),delta[2]);
            if(measure<=1e-12)measure=std::hypot(delta[3],delta[4]);
            if(measure<=1e-12){valid=false;break;}
            // First G05 knot must be a standstill anchor irrespective of the
            // actual post-G00/post-M51 feedback residual. Later collinear
            // knots may carry nonzero STARTMOVE_SPEED and ENDMOVE_SPEED.
            natural.push_back({p.q,measure/dt,FeedMode::LegacyPerSecond,
                               !natural.empty()});
            previous=p.q;
        }
        if(!valid)continue;
        Plan envelope;
        std::string why;
        if(!plan(start,natural,reserved,vectorAccelCap,vectorDecelCap,envelope,why)){
            error="G93 acceleration-aware lookahead failed: "+why;return false;
        }
        if(envelope.segments.size()!=original.size()){
            error="G93 retime changed the number of interpolation points";return false;
        }
        std::vector<Waypoint> proposed;
        proposed.reserve(original.size());
        std::vector<double> elapsed;
        elapsed.reserve(original.size());
        const double firstRobustSeconds=worstCaseFirstKnotSeconds(
            start,original.front().q,limits,vectorAccelCap,vectorDecelCap);
        if(!std::isfinite(firstRobustSeconds) || firstRobustSeconds<=0){
            error="Invalid first-knot acceleration time under live MPOS tolerance";
            return false;
        }
        for(std::size_t i=0;i<original.size();++i){
            const double requested=60.0/original[i].feed;
            // Legacy path has capped vector speed and acceleration overhead.
            // The first knot must additionally work for a LIVE XYZ/C/B start
            // within the executor's 0.05mm/0.05deg convergence tolerance.
            // 2% reserve covers rounding of nominal NC and SDK float demands.
            const double dt=std::max({requested,envelope.segments[i].seconds,
                             i==0?firstRobustSeconds*1.02:0.0});
            if(!std::isfinite(dt) || dt<=0){valid=false;break;}
            proposed.push_back({original[i].q,60.0/dt,
                                FeedMode::InverseMinute,i>0});
            elapsed.push_back(dt);
        }
        if(!valid)continue;
        Plan verified;
        if(plan(start,proposed,limits,vectorAccelCap,vectorDecelCap,verified,why) &&
           verified.segments.size()==proposed.size()){
            seconds=std::move(elapsed);
            return true;
        }
        error="No controller-consistent continuous G93 velocity schedule: "+why;
    }
    return false;
}

bool arcWaypoints(const Joint& start,const Joint& end,const Arc& arc,std::vector<Joint>& points,std::string& error) {
    points.clear();error.clear();
    if(!finite(start)||!finite(end)||!finite(arc.centerOffset)||!std::isfinite(arc.radius)||
       !std::isfinite(arc.chordToleranceMm)||arc.chordToleranceMm<=0||
       !std::isfinite(arc.rotaryStepDeg)||arc.rotaryStepDeg<=0) {error="Invalid arc";return false;}
    int u=0,v=1; if(arc.plane==Plane::ZX){u=2;v=0;} else if(arc.plane==Plane::YZ){u=1;v=2;}
    double cu=start[u]+arc.centerOffset[u], cv=start[v]+arc.centerOffset[v];
    if(arc.useRadius) {
        double dx=end[u]-start[u],dy=end[v]-start[v],c=std::hypot(dx,dy),r=std::abs(arc.radius);
        if(c<1e-12||r<=0||c>2*r+1e-9) {error="Invalid R arc (full circles require IJK)";return false;}
        double h=std::sqrt(std::max(0.0,r*r-c*c/4));
        bool selected=false;
        for(double sign:{1.0,-1.0}) {
            double x=(start[u]+end[u])/2-sign*h*dy/c,y=(start[v]+end[v])/2+sign*h*dx/c;
            double a=std::atan2(start[v]-y,start[u]-x),b=std::atan2(end[v]-y,end[u]-x);
            double angle=std::abs(sweep(a,b,arc.clockwise));
            if((arc.radius>=0 && angle<=pi+1e-10)||(arc.radius<0 && angle>=pi-1e-10)) {cu=x;cv=y;selected=true;break;}
        }
        if(!selected){error="Cannot select R arc";return false;}
    }
    double r=std::hypot(start[u]-cu,start[v]-cv),rend=std::hypot(end[u]-cu,end[v]-cv);
    if(r<=1e-10||std::abs(r-rend)>std::max(1e-5,r*1e-6)) {error="Arc endpoints have inconsistent radii";return false;}
    double a=std::atan2(start[v]-cv,start[u]-cu),b=std::atan2(end[v]-cv,end[u]-cu);
    double angle=sweep(a,b,arc.clockwise);
    double maxAngle=2*std::acos(std::clamp(1-arc.chordToleranceMm/r,-1.0,1.0));
    maxAngle=std::min(maxAngle,pi/18); // also <=10deg
    if(maxAngle<=1e-12){error="Arc tolerance too small";return false;}
    double n=std::max(std::ceil(std::abs(angle)/maxAngle),std::ceil(std::max(std::abs(end[3]-start[3]),std::abs(end[4]-start[4]))/arc.rotaryStepDeg));
    if(!std::isfinite(n)||n>200000){error="Arc exceeds segment budget";return false;}
    int count=std::max(1,static_cast<int>(n));points.reserve(count);
    for(int k=1;k<=count;++k) {
        double f=static_cast<double>(k)/count;Joint q;
        for(int j=0;j<5;++j)q[j]=start[j]+f*(end[j]-start[j]);
        q[u]=cu+r*std::cos(a+f*angle);q[v]=cv+r*std::sin(a+f*angle);
        if(k==count) q=end;
        points.push_back(q);
    }
    return true;
}
bool threePointArc(const Joint& start,const Joint& mid,const Joint& end,bool cw,Arc& arc,std::string& error) {
    if(!finite(start)||!finite(mid)||!finite(end)){error="Invalid three-point arc";return false;}
    double ax=mid[0]-start[0],ay=mid[1]-start[1],bx=end[0]-start[0],by=end[1]-start[1];
    double d=2*(ax*by-ay*bx);
    if(std::abs(d)<1e-12){error="Collinear three-point arc";return false;}
    arc={};arc.clockwise=cw;
    arc.centerOffset[0]=((ax*ax+ay*ay)*by-(bx*bx+by*by)*ay)/d;
    arc.centerOffset[1]=((bx*bx+by*by)*ax-(ax*ax+ay*ay)*bx)/d;
    double cu=start[0]+arc.centerOffset[0],cv=start[1]+arc.centerOffset[1];
    double s=std::atan2(start[1]-cv,start[0]-cu),m=std::atan2(mid[1]-cv,mid[0]-cu),e=std::atan2(end[1]-cv,end[0]-cu);
    if(std::abs(sweep(s,m,cw))>=std::abs(sweep(s,e,cw))+1e-10){error="Midpoint conflicts with G02/G03 direction";return false;}
    return true;
}
} // namespace mls::runtime
