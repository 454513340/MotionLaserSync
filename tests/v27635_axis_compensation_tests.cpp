#include "TestSupport.h"
#include "runtime/AxisCompensation.h"
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
using namespace mls;
namespace {
axiscomp::Tables synthetic() {
    axiscomp::Tables tables;
    for(int j=0;j<3;++j){auto& t=tables[j];t.axisIndex=j;t.method=axiscomp::SamplingMethod::MonotonicSweep;t.settleMs=0;
        for(int d=0;d<2;++d)for(double x:{-10.,-1.,0.,0.5,1.,2.,10.}){
            double c=0.01+(d?0.005:0.)+0.001*x;
            t.directions[d].push_back({x,x+c});
        }
    }
    return tables;
}
axiscomp::Options options(){axiscomp::Options o;o.pointSettleMs=0;o.allowContinuous=true;o.allowLegacyPointTableOnPlannedMotion=true;return o;}
axiscomp::Tables supplied(){axiscomp::Tables ts;std::string error;
    for(int j=0;j<3;++j){const std::string a(1,"XYZ"[j]);std::ifstream in(std::string(MLS_COMPENSATION_DATA_ROOT)+"/"+a+"/"+a+"_compensation_candidate.csv");REQUIRE(in.good());REQUIRE(axiscomp::readCandidateCsv(in,j,ts[j],error));}
    return ts;
}
void close(double a,double b,double eps=1e-10){REQUIRE(std::abs(a-b)<=eps);}
}
int main(){return runTests({
 {"all 48 recorded Y AB compensated demands reproduced",[]{auto tables=supplied();
    std::ifstream in(std::string(MLS_COMPENSATION_DATA_ROOT)+"/validation_evidence/Y_previous_conversation/axis_compensation_ab_validation.csv");
    REQUIRE(in.good());std::string line,error;std::getline(in,line);int count=0;
    while(std::getline(in,line)){std::vector<std::string> values;std::istringstream row(line);std::string v;
        while(std::getline(row,v,',')){if(v.size()>=2&&v.front()=='"'&&v.back()=='"')v=v.substr(1,v.size()-2);values.push_back(v);}
        REQUIRE(values.size()>=10);if(values[5]!="COMPENSATED")continue;double out=0;
        REQUIRE(axiscomp::interpolate(tables[1],values[2]=="+"?1:-1,std::stod(values[8]),out,error));
        close(out,std::stod(values[9]),3e-9);++count;
    }REQUIRE(count==48);
 }},
 {"exact supplied XYZ tables load with both directional node counts",[]{auto t=supplied();REQUIRE(t[0].directions[0].size()==519);REQUIRE(t[0].directions[1].size()==519);REQUIRE(t[1].directions[0].size()==327);REQUIRE(t[1].directions[1].size()==327);REQUIRE(t[2].directions[0].size()==173);REQUIRE(t[2].directions[1].size()==173);}},
 {"all 2038 supplied nodes round-trip exactly",[]{auto t=supplied();std::string e;std::size_t count=0;for(auto& a:t)for(int d=0;d<2;++d)for(auto& n:a.directions[d]){double out=0;REQUIRE(axiscomp::interpolate(a,d? -1:1,n.desiredMpos,out,e));close(out,n.requiredDpos);++count;}REQUIRE(count==2038);}},
 {"all 2032 supplied off-grid midpoints interpolate adjacent demands",[]{auto t=supplied();std::string e;std::size_t count=0;for(auto& a:t)for(int d=0;d<2;++d)for(std::size_t i=1;i<a.directions[d].size();++i){const auto& p=a.directions[d][i-1];const auto& n=a.directions[d][i];double out=0;REQUIRE(axiscomp::interpolate(a,d? -1:1,(p.desiredMpos+n.desiredMpos)/2,out,e));close(out,(p.requiredDpos+n.requiredDpos)/2,1e-10);++count;}REQUIRE(count==2032);}},
 {"supplied Y positive AB target matches validation command",[]{auto t=supplied();double out=0;std::string e;REQUIRE(axiscomp::interpolate(t[1],1,-59.259749094,out,e));close(out,-59.25,3e-9);}},
 {"supplied Y negative AB target matches validation command",[]{auto t=supplied();double out=0;std::string e;REQUIRE(axiscomp::interpolate(t[1],-1,83.242248535,out,e));close(out,83.245746321,3e-9);}},
 {"direction selects different branch",[]{auto ts=synthetic();std::string e;double a,b;REQUIRE(axiscomp::interpolate(ts[0],1,1,a,e));REQUIRE(axiscomp::interpolate(ts[0],-1,1,b,e));close(b-a,.005);}},
 {"out of range / invalid direction / NaN rejected without extrapolation",[]{auto ts=synthetic();std::string e;double c=77;REQUIRE(!axiscomp::interpolate(ts[0],1,10.000001,c,e));REQUIRE(!axiscomp::interpolate(ts[0],-1,-10.000001,c,e));REQUIRE(!axiscomp::interpolate(ts[0],0,1,c,e));REQUIRE(!axiscomp::interpolate(ts[0],1,std::numeric_limits<double>::quiet_NaN(),c,e));}},
 {"both MPOS and DPOS must be strictly monotonic",[]{auto t=synthetic()[0];std::string e;t.directions[0][2].requiredDpos=t.directions[0][1].requiredDpos;REQUIRE(!axiscomp::validateTable(t,.1,e));}},
 {"duplicate MPOS and missing direction rejected",[]{auto t=synthetic()[0];std::string e;t.directions[0][2].desiredMpos=t.directions[0][1].desiredMpos;REQUIRE(!axiscomp::validateTable(t,.1,e));t=synthetic()[0];t.directions[1].clear();REQUIRE(!axiscomp::validateTable(t,.1,e));}},
 {"excessive and non-finite corrections rejected",[]{auto t=synthetic()[0];std::string e;t.directions[0][2].requiredDpos+=.5;REQUIRE(!axiscomp::validateTable(t,.1,e));t=synthetic()[0];t.directions[0][2].requiredDpos=std::numeric_limits<double>::infinity();REQUIRE(!axiscomp::validateTable(t,.1,e));}},
 {"wrong CSV axis rejected atomically",[]{std::istringstream in("axis,direction,desired_mpos_mm,required_dpos_mm\nY,+,0,.01\n");axiscomp::Table t;t.axisIndex=2;std::string e;REQUIRE(!axiscomp::readCandidateCsv(in,0,t,e));REQUIRE(t.axisIndex==2);}},
 {"CSV garbage/NaN/correction mismatch never skipped",[]{for(auto row:{"X,+,0,nan,0.01","X,+,0,0.01junk,0.01","X,+,0,.01,.02"}){std::istringstream in(std::string("axis,direction,desired_mpos_mm,required_dpos_mm,correction_required_dpos_minus_desired_mpos_mm\n")+row+"\n");axiscomp::Table t;std::string e;REQUIRE(!axiscomp::readCandidateCsv(in,0,t,e));}}},
 {"BOM/quoted CSV valid; blank line supported",[]{std::istringstream in("\xEF\xBB\xBF\"axis\",direction,desired_mpos_mm,required_dpos_mm\r\n\"X\",\"+\",0,.01\nX,+,1,1.01\n\nX,-,0,.02\nX,-,1,1.02\n");axiscomp::Table t;std::string e;REQUIRE(axiscomp::readCandidateCsv(in,0,t,e));}},
 {"held axis preserves demand, not a re-compensated MPOS",[]{auto ts=synthetic();axiscomp::PointResult r;std::string e;REQUIRE(axiscomp::mapPoint(ts[0],options(),1,1.073,1,r,e));close(r.commandDpos,1.073);REQUIRE(r.held&&r.direction==0);}},
 {"tiny reversal that would demand wrong direction rejected",[]{auto ts=synthetic();axiscomp::PointResult r;std::string e;REQUIRE(!axiscomp::mapPoint(ts[0],options(),1,1.011,0.9999,r,e));}},
 {"legacy Z history requires explicit acknowledgement",[]{auto t=synthetic()[2];t.method=axiscomp::SamplingMethod::PointToPointPreload;auto o=options();o.allowLegacyPointTableOnPlannedMotion=false;std::string e;axiscomp::PointResult r;REQUIRE(!axiscomp::mapPoint(t,o,0,.01,1,r,e));o.allowLegacyPointTableOnPlannedMotion=true;REQUIRE(axiscomp::mapPoint(t,o,0,.01,1,r,e));}},
 {"point settle cannot be shorter than acquisition",[]{auto ts=synthetic();ts[2].settleMs=2000;std::string e;auto o=options();REQUIRE(!axiscomp::validateTables(ts,o,e));o.pointSettleMs=2000;REQUIRE(axiscomp::validateTables(ts,o,e));}},
 {"path subdivides all crossed XYZ calibration knots",[]{auto ts=synthetic();auto o=options();runtime::Joint n{0,0,0,0,0},d{.01,.013,.017,0,0};std::vector<runtime::Waypoint> w{{{2,0,0,0,0},2,runtime::FeedMode::LegacyPerSecond,true}},out;std::string e;REQUIRE(axiscomp::mapPath(ts,o,n,d,w,out,e));REQUIRE(out.size()==3);close(out[0].q[0],.5105);close(out[1].q[0],1.011);close(out[2].q[0],2.012);for(auto& p:out){close(p.q[1],.013);close(p.q[2],.017);}}},
 {"feed transformed for physical segment timing; G94 preserved",[]{auto ts=synthetic();auto o=options();runtime::Joint n{},d{.01,.01,.01,0,0};std::vector<runtime::Waypoint> w{{{2,0,0,0,0},120,runtime::FeedMode::PerMinute,true}},out;std::string e;REQUIRE(axiscomp::mapPath(ts,o,n,d,w,out,e));for(auto& p:out)close(p.feed,120.12,1e-8);}},
 {"G93 subdivision nominal times sum to original block time",[]{auto ts=synthetic();auto o=options();runtime::Joint n{},d{.01,.01,.01,0,0};std::vector<runtime::Waypoint> w{{{2,0,0,0,0},6,runtime::FeedMode::InverseMinute,false}},out;std::string e;REQUIRE(axiscomp::mapPath(ts,o,n,d,w,out,e));double total=0;for(auto& p:out)total+=60/p.feed;close(total,10);}},
 {"continuous gating cannot silently return RAW",[]{auto ts=synthetic();auto o=options();o.allowContinuous=false;runtime::Joint n{},d{.01,.01,.01,0,0};std::vector<runtime::Waypoint> w{{{2,0,0,0,0},2}},out;std::string e;REQUIRE(!axiscomp::mapPath(ts,o,n,d,w,out,e));REQUIRE(out.empty());}},
 {"rotary-only path preserves XYZ even with continuous compensation blocked",[]{auto ts=synthetic();auto o=options();o.allowContinuous=false;runtime::Joint n{},d{.01,.02,.03,0,0};std::vector<runtime::Waypoint> w{{{0,0,0,1,2},2}},out;std::string e;REQUIRE(axiscomp::mapPath(ts,o,n,d,w,out,e));REQUIRE(out.size()==1);close(out[0].q[0],.01);close(out[0].q[1],.02);close(out[0].q[2],.03);close(out[0].q[3],1);close(out[0].q[4],2);}},
 {"late invalid waypoint leaves entire mapped output empty",[]{auto ts=synthetic();auto o=options();runtime::Joint n{},d{.01,.01,.01,0,0};std::vector<runtime::Waypoint> w{{{2,0,0,0,0},2},{{20,0,0,0,0},2}},out;std::string e;REQUIRE(!axiscomp::mapPath(ts,o,n,d,w,out,e));REQUIRE(out.empty());}},
 {"bounded subdivisions reject oversize output",[]{auto ts=synthetic();auto o=options();o.maxPathPoints=2;runtime::Joint n{},d{.01,.01,.01,0,0};std::vector<runtime::Waypoint> w{{{2,0,0,0,0},2}},out;std::string e;REQUIRE(!axiscomp::mapPath(ts,o,n,d,w,out,e));REQUIRE(out.empty());}}
});}
