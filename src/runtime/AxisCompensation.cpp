#include "runtime/AxisCompensation.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <locale>
#include <sstream>
#include <unordered_map>

namespace mls::axiscomp {
namespace {
std::string trim(std::string s) {
    const auto first=s.find_first_not_of(" \t\r\n");
    return first==std::string::npos ? std::string{} : s.substr(first,s.find_last_not_of(" \t\r\n")-first+1);
}
bool fields(const std::string& line,std::vector<std::string>& out) {
    out.clear(); std::string value; bool quoted=false,closed=false;
    for(std::size_t i=0;i<line.size();++i) {
        const char c=line[i];
        if(quoted) {
            if(c=='"') { if(i+1<line.size() && line[i+1]=='"'){value+='"';++i;} else {quoted=false;closed=true;} }
            else value+=c;
        } else if(c==',') {out.push_back(trim(value));value.clear();closed=false;}
        else if(c=='"') {if(!trim(value).empty() || closed)return false;value.clear();quoted=true;}
        else {if(closed && c!=' ' && c!='\t' && c!='\r')return false;value+=c;}
    }
    if(quoted)return false;
    out.push_back(trim(value));return true;
}
bool number(const std::string& s,double& n) {
    std::istringstream in(s);in.imbue(std::locale::classic());
    if(!(in>>n) || !std::isfinite(n))return false;
    in>>std::ws;return in.eof();
}
std::string label(const Table& t) {
    return t.axisIndex>=0 && t.axisIndex<3 ? std::string(1,"XYZ"[t.axisIndex]) : std::string("?");
}
int sign(double delta,double tolerance) {return delta>tolerance?1:(delta<-tolerance?-1:0);}
bool finite(const runtime::Joint& q) {for(double v:q)if(!std::isfinite(v))return false;return true;}
double feedMeasure(const runtime::Joint& a,const runtime::Joint& b) {
    const double linear=std::hypot(std::hypot(b[0]-a[0],b[1]-a[1]),b[2]-a[2]);
    return linear>1e-12?linear:std::hypot(b[3]-a[3],b[4]-a[4]);
}
bool methodAllowed(const Table& t,const Options& o,std::string& error) {
    if(t.method==SamplingMethod::PointToPointPreload && !o.allowLegacyPointTableOnPlannedMotion) {
        error=label(t)+": table was acquired with per-point preload; production history is not qualified. "
              "Explicitly acknowledge/revalidate planned-path use; no hidden preload move is inserted.";
        return false;
    }
    return true;
}
}
bool validateTable(const Table& t,double cap,std::string& error) {
    error.clear();
    if(t.axisIndex<0 || t.axisIndex>2 || !std::isfinite(cap) || cap<=0 || cap>1.0 ||
       t.settleMs<0 || t.settleMs>60000 || !std::isfinite(t.preloadMm) || t.preloadMm<=0 ||
       !std::isfinite(t.precisionSpeedMmS) || t.precisionSpeedMmS<0) {
        error="Invalid table metadata / safety correction bound";return false;
    }
    for(const auto& branch:t.directions) {
        if(branch.size()<2 || branch.size()>100000) {error=label(t)+": both directions require 2..100000 nodes";return false;}
        for(std::size_t i=0;i<branch.size();++i) {
            const auto& n=branch[i];
            if(!std::isfinite(n.desiredMpos) || !std::isfinite(n.requiredDpos) ||
               std::abs(n.requiredDpos-n.desiredMpos)>cap) {
                error=label(t)+": non-finite node or correction exceeds configured bound";return false;
            }
            if(i && (!(n.desiredMpos>branch[i-1].desiredMpos) || !(n.requiredDpos>branch[i-1].requiredDpos))) {
                error=label(t)+": MPOS and DPOS must both be strictly increasing in each direction";return false;
            }
        }
    }
    const auto& p=t.directions[0];const auto& n=t.directions[1];
    if((std::max)(p.front().desiredMpos,n.front().desiredMpos)>=(std::min)(p.back().desiredMpos,n.back().desiredMpos)) {
        error=label(t)+": positive/negative ranges do not overlap";return false;
    }
    return true;
}
bool validateTables(const Tables& tables,const Options& o,std::string& error) {
    if(o.pointSettleMs<0 || o.pointSettleMs>60000 || !std::isfinite(o.holdToleranceMm) ||
       o.holdToleranceMm<0 || o.holdToleranceMm>0.001 || !std::isfinite(o.commandDirectionToleranceMm) ||
       o.commandDirectionToleranceMm<0 || o.commandDirectionToleranceMm>0.001 ||
       o.maxPathPoints<2 || o.maxPathPoints>200000) {
        error="Invalid runtime compensation options";return false;
    }
    for(int i=0;i<3;++i) {
        if(tables[i].axisIndex!=i){error="XYZ bundle axis order/identity mismatch";return false;}
        if(!validateTable(tables[i],o.maxCorrectionMm,error))return false;
        if(o.pointSettleMs<tables[i].settleMs){error="Point settle time cannot be shorter than table acquisition settle time";return false;}
    }
    return true;
}
bool readCandidateCsv(std::istream& input,int expectedAxis,Table& output,std::string& error) {
    error.clear();Table t;t.axisIndex=expectedAxis;
    if(expectedAxis<0 || expectedAxis>2){error="Expected XYZ axis index";return false;}
    std::string line;std::vector<std::string> header,row;
    if(!std::getline(input,line)){error="Empty candidate CSV";return false;}
    if(line.compare(0,3,"\xEF\xBB\xBF")==0)line.erase(0,3);
    if(!fields(line,header)){error="Malformed CSV header";return false;}
    std::unordered_map<std::string,std::size_t> cols;
    for(std::size_t i=0;i<header.size();++i)if(!cols.emplace(header[i],i).second){error="Duplicate CSV header";return false;}
    for(const char* name:{"axis","direction","desired_mpos_mm","required_dpos_mm"})
        if(!cols.count(name)){error=std::string("Missing CSV column: ")+name;return false;}
    std::size_t lineNo=1;
    while(std::getline(input,line)) {
        ++lineNo;if(trim(line).empty())continue;
        const auto fail=[&](const std::string& why){error="CSV line "+std::to_string(lineNo)+": "+why;return false;};
        if(line.size()>65536 || !fields(line,row) || row.size()!=header.size())return fail("Malformed row / field count");
        if(row[cols.at("axis")]!=std::string(1,"XYZ"[expectedAxis]))return fail("Mixed/wrong axis identity");
        const auto& d=row[cols.at("direction")];if(d!="+" && d!="-")return fail("Direction must be + or -");
        Node n;if(!number(row[cols.at("desired_mpos_mm")],n.desiredMpos) ||
                  !number(row[cols.at("required_dpos_mm")],n.requiredDpos))return fail("Invalid MPOS/DPOS number");
        const auto ci=cols.find("correction_required_dpos_minus_desired_mpos_mm");
        if(ci!=cols.end()) {
            double c=0;if(!number(row[ci->second],c) || std::abs((n.requiredDpos-n.desiredMpos)-c)>5e-8)
                return fail("Correction column disagrees with MPOS -> DPOS mapping");
        }
        const auto si=cols.find("samples");
        if(si!=cols.end()){double count=0;if(!number(row[si->second],count) || count<1 || count!=std::floor(count))return fail("Invalid sample count");}
        auto& branch=t.directions[d=="+"?0:1];branch.push_back(n);
        if(branch.size()>100000)return fail("Node count bound exceeded");
    }
    if(input.bad()){error="CSV read error";return false;}
    for(auto& branch:t.directions)std::sort(branch.begin(),branch.end(),[](const Node&a,const Node&b){return a.desiredMpos<b.desiredMpos;});
    if(!validateTable(t,0.10,error))return false;
    output=std::move(t);return true;
}
bool interpolate(const Table& t,int direction,double desired,double& command,std::string& error) {
    if((direction!=1 && direction!=-1) || !std::isfinite(desired)){error="Invalid interpolation direction/target";return false;}
    const auto& nodes=t.directions[direction>0?0:1];
    if(nodes.size()<2 || desired<nodes.front().desiredMpos || desired>nodes.back().desiredMpos) {
        error=label(t)+": desired MPOS "+std::to_string(desired)+" is outside calibrated "+(direction>0?"+":"-")+" range; extrapolation is forbidden";return false;
    }
    auto hi=std::lower_bound(nodes.begin(),nodes.end(),desired,[](const Node& n,double v){return n.desiredMpos<v;});
    if(hi==nodes.begin() || (hi!=nodes.end() && hi->desiredMpos==desired)){command=hi->requiredDpos;return true;}
    if(hi==nodes.end()){command=nodes.back().requiredDpos;return true;}
    const auto& lo=*(hi-1);const double u=(desired-lo.desiredMpos)/(hi->desiredMpos-lo.desiredMpos);
    command=lo.requiredDpos+u*(hi->requiredDpos-lo.requiredDpos);
    if(!std::isfinite(command)){error="Interpolation overflow";return false;}return true;
}
bool mapPoint(const Table& t,const Options& o,double start,double currentD,double desired,
              PointResult& result,std::string& error) {
    error.clear();result={};
    if(!std::isfinite(start)||!std::isfinite(currentD)||!std::isfinite(desired)){error="Non-finite point coordinate";return false;}
    const int direction=sign(desired-start,o.holdToleranceMm);
    if(!direction){result.commandDpos=currentD;result.held=true;return true;}
    if(!methodAllowed(t,o,error))return false;
    double command=0;if(!interpolate(t,direction,desired,command,error))return false;
    if(std::abs(command-desired)>o.maxCorrectionMm){error="Interpolated correction exceeds configured bound";return false;}
    if((command-currentD)*direction < -o.commandDirectionToleranceMm){
        error=label(t)+": branch change would reverse the demanded motion. Re-establish approach direction; no automatic backlash jump is permitted";return false;
    }
    result.commandDpos=command;result.direction=direction;return true;
}
bool mapPath(const Tables& tables,const Options& o,const runtime::Joint& initialNominal,
             const runtime::Joint& initialCommand,const std::vector<runtime::Waypoint>& input,
             std::vector<runtime::Waypoint>& output,std::string& error) {
    output.clear();error.clear();
    if(!validateTables(tables,o,error))return false;
    bool movesXYZ=false;auto probe=initialNominal;
    for(const auto& w:input){for(int j=0;j<3;++j)if(std::abs(w.q[j]-probe[j])>o.holdToleranceMm)movesXYZ=true;probe=w.q;}
    if(movesXYZ && !o.allowContinuous){error="Continuous/G-code/RTCP compensation requires explicit production-condition qualification acknowledgement";return false;}
    if(!finite(initialNominal)||!finite(initialCommand)||input.empty()||input.size()>o.maxPathPoints){error="Invalid compensated path start/size";return false;}
    auto previousNominal=initialNominal;
    auto previousCommand=initialCommand;
    std::vector<runtime::Waypoint> staged;
    std::array<int,3> precedingDirection{};
    for(const auto& w:input) {
        if(!finite(w.q)||!std::isfinite(w.feed)||w.feed<=0){error="Invalid nominal waypoint/feed";return false;}
        std::array<int,3> dirs{};
        std::vector<double> cuts{1.0};
        for(int j=0;j<3;++j) {
            dirs[j]=sign(w.q[j]-previousNominal[j],o.holdToleranceMm);
            if(!dirs[j])continue;
            if(!methodAllowed(tables[j],o,error))return false;
            double unused=0;
            if(!interpolate(tables[j],dirs[j],previousNominal[j],unused,error) ||
               !interpolate(tables[j],dirs[j],w.q[j],unused,error))return false;
            const auto& nodes=tables[j].directions[dirs[j]>0?0:1];
            const double low=(std::min)(previousNominal[j],w.q[j]), high=(std::max)(previousNominal[j],w.q[j]);
            auto it=std::upper_bound(nodes.begin(),nodes.end(),low,[](double v,const Node& n){return v<n.desiredMpos;});
            for(;it!=nodes.end() && it->desiredMpos<high;++it) {
                cuts.push_back((it->desiredMpos-previousNominal[j])/(w.q[j]-previousNominal[j]));
                if(cuts.size()>o.maxPathPoints){error="Compensation subdivision limit exceeded";return false;}
            }
        }
        std::sort(cuts.begin(),cuts.end());
        cuts.erase(std::unique(cuts.begin(),cuts.end(),[](double a,double b){return std::abs(a-b)<1e-12;}),cuts.end());
        auto nominalPieceStart=previousNominal;double previousT=0;
        bool directionChange=false;
        for(int j=0;j<3;++j)if(dirs[j] && precedingDirection[j] && dirs[j]!=precedingDirection[j])directionChange=true;
        if(directionChange && !staged.empty())staged.back().continuous=false;
        for(double t:cuts) {
            if(t<=previousT+1e-13)continue;
            runtime::Waypoint c=w;runtime::Joint nominal{};
            for(int j=0;j<5;++j)nominal[j]=t==1?w.q[j]:previousNominal[j]+t*(w.q[j]-previousNominal[j]);
            c.q=nominal;
            for(int j=0;j<3;++j) {
                if(!dirs[j]){c.q[j]=previousCommand[j];continue;}
                if(!interpolate(tables[j],dirs[j],nominal[j],c.q[j],error))return false;
                if(std::abs(c.q[j]-nominal[j])>o.maxCorrectionMm ||
                   (c.q[j]-previousCommand[j])*dirs[j]<-o.commandDirectionToleranceMm) {
                    error=label(tables[j])+": path correction/branch transition violates command direction; shorten history mismatch or requalify approach";return false;
                }
            }
            const double nominalMeasure=feedMeasure(nominalPieceStart,nominal);
            const double commandMeasure=feedMeasure(previousCommand,c.q);
            if(commandMeasure<=1e-12 && nominalMeasure<=1e-12){previousT=t;nominalPieceStart=nominal;continue;}
            if(nominalMeasure<=1e-12 || commandMeasure<=1e-12){error="Degenerate compensated path segment";return false;}
            if(w.mode==runtime::FeedMode::InverseMinute)c.feed=w.feed/(t-previousT);
            else c.feed=w.feed*commandMeasure/nominalMeasure;
            // Compensation knots are artificial subdivisions of one line. The
            // velocity planner bounds their small-angle junctions; real reversals stop.
            c.continuous=directionChange?false:(w.continuous || t<1.0 || previousT>0);
            if(!std::isfinite(c.feed)||c.feed<=0){error="Compensated feed overflow";return false;}
            staged.push_back(c);
            if(staged.size()>o.maxPathPoints){error="Compensated path exceeds controller host-plan bound";return false;}
            previousCommand=c.q;nominalPieceStart=nominal;previousT=t;
        }
        previousNominal=w.q;
        for(int j=0;j<3;++j)if(dirs[j])precedingDirection[j]=dirs[j];
    }
    output=std::move(staged);return true;
}
} // namespace mls::axiscomp
