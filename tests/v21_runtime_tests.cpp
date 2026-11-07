#include "TestSupport.h"
#include "runtime/FiveAxis.h"
#include "runtime/GCode5D.h"
#include "runtime/ScanArchive5D.h"
#include <algorithm>
#include <future>
#include <limits>
#include <random>
using namespace mls::runtime;
namespace {
constexpr double pi=3.14159265358979323846;
bool near(double a,double b,double t=1e-10){return std::abs(a-b)<=t;}
ArchiveProfile profile() {
    ArchiveProfile p;p.frame.joints={1,2,3,23,-17};p.frame.raw={1,2,3,13,3};
    p.frame.command=p.frame.joints;p.frame.rotaryOffsets={10,-20};
    p.frame.session=99;p.frame.ordinal=7;p.profileIndex=0xffffff;
    p.frame.hostEstimateUs=1234567890123LL;p.timeUs=1234567890111LL;p.sensorTimeSec=12.3456789;
    std::array<double,17> model{};model[3]=0.1;model[7]=0.2;model[11]=0.01;model[14]=13;
    std::string e;REQUIRE(completeFrame(p.frame,model,e));
    p.points={{0,-0.123456789012345,700.1234567890123},{1279,45.123456789,2.2}};return p;
}
std::array<float,22> header(){std::array<float,22> h{};h[0]=96;h[7]=270909;h[8]=6;h[9]=5;h[10]=h[11]=9;h[12]=1;h[13]=31;h[14]=4;h[15]=31;h[16]=0;h[17]=0;h[18]=14;h[19]=104;h[20]=14;h[21]=1;return h;}
}
int main(){return runTests({
{"G-code lower case, compact words, comments and full XYZCB",[]{NcState s;NcBlock b;std::string e;REQUIRE(parseNcBlock("n1g90g94g1x1.5y-2z3c45b-30f120s0 (ok)",s,b,e));REQUIRE(b.axisMask==31&&b.words[3]==45&&b.words[4]==-30&&!b.capture&&b.feed==120);}},
{"Omitted axes hold current, incremental applies only given words",[]{Joint q;std::string e;REQUIRE(resolve({1,2,3,40,-20},{9,0,0,0,0},1,true,q,e));REQUIRE((q==Joint{9,2,3,40,-20}));REQUIRE(resolve(q,{0,0,0,5,0},8,false,q,e));REQUIRE(q[3]==45&&q[4]==-20);}},
{"Invalid NC leaves prior modal state unchanged",[]{NcState s;NcBlock b;std::string e;REQUIRE(!parseNcBlock("G91 G43 X1",s,b,e));REQUIRE(s.absolute&&s.modalMotion==-1);}},
{"G43.4 and G49 are explicit RTCP modal states",[]{NcState s;NcBlock b;std::string e;REQUIRE(parseNcBlock("G43.4",s,b,e)&&s.rtcp&&b.rtcp);REQUIRE(parseNcBlock("G1 X1 C20 F10",s,b,e)&&b.rtcp);REQUIRE(parseNcBlock("G49",s,b,e)&&!s.rtcp&&!b.rtcp);}},
{"Unsupported work offsets and unknown codes fail closed",[]{for(const auto& text:{"G43.5 X1","G54","G20","M3 S100","G1 A2","G1 X1 garbage"}){NcState s;NcBlock b;std::string e;REQUIRE(!parseNcBlock(text,s,b,e));}}},
{"NaN infinity and overflow are rejected",[]{for(const auto& text:{"G1 Xnan","G1 Xinf","G1 X1e999","G1 X1 F0","G1 X1 F-1","G1 X1 Fnan"}){NcState s;NcBlock b;std::string e;REQUIRE(!parseNcBlock(text,s,b,e));}}},
{"Duplicate modal groups and words are errors",[]{for(const auto& text:{"G90 G91 G1 X1","G1 G0 X1","G1 X1 X2","G1 X1 M50","G1 X1 F2 F3"}){NcState s;NcBlock b;std::string e;REQUIRE(!parseNcBlock(text,s,b,e));}}},
{"Axis-only line requires established motion mode",[]{NcState s;NcBlock b;std::string e;REQUIRE(!parseNcBlock("X1",s,b,e));REQUIRE(parseNcBlock("G1 X1 F2",s,b,e));REQUIRE(parseNcBlock("C2 B3",s,b,e));REQUIRE(b.motion==1&&b.axisMask==24);}},
{"G92 is a program-coordinate operation, not physical zero",[]{NcState s;NcBlock b;std::string e;REQUIRE(parseNcBlock("G92 X0 C90",s,b,e));REQUIRE(b.kind==NcBlock::Kind::SetWorkPosition&&b.axisMask==9);}},
{"G93 requires block-local F except rapid positioning",[]{NcState s;NcBlock b;std::string e;REQUIRE(parseNcBlock("G93",s,b,e));REQUIRE(!parseNcBlock("G1 X1",s,b,e));REQUIRE(parseNcBlock("G1 X1 F1",s,b,e));REQUIRE(!parseNcBlock("X2",s,b,e));REQUIRE(parseNcBlock("G0 X0",s,b,e));}},
{"Dwell uses explicit integer milliseconds in this dialect",[]{NcState s;NcBlock b;std::string e;REQUIRE(parseNcBlock("G4 P500",s,b,e)&&b.dwellMs==500);REQUIRE(!parseNcBlock("G4 P0.5",s,b,e));}},
{"Quoted M62 path is kept intact",[]{NcState s;NcBlock b;std::string e;REQUIRE(parseNcBlock("M62 P\"D:/scan data\" Q\"part\" T\"scan_zlib\"",s,b,e));REQUIRE(b.strings.at('P')=="D:/scan data");}},
{"Arc definitions cannot be mixed or missing",[]{for(const auto& text:{"G2 X1","G2 X1 I0 R2","G2 X1 P2","G2 X1 K1 I1","G2 X1 R0"}){NcState s;NcBlock b;std::string e;REQUIRE(!parseNcBlock(text,s,b,e));}}},
{"Zero I/J are legitimate explicit center words",[]{NcState s;NcBlock b;std::string e;REQUIRE(parseNcBlock("G3 X1 Y1 I0 J1",s,b,e));REQUIRE(b.arcDefinition==NcBlock::ArcDefinition::Center);}},
{"Five-axis simultaneous feed respects every axis speed and acceleration",[]{Plan p;std::string e;Limits l;REQUIRE(plan({},{{{10,1,0,30,-10},300,FeedMode::PerMinute}},l,100,100,p,e));for(const auto& s:p.segments)for(int j=0;j<5;++j){REQUIRE(std::abs(s.direction[j]*s.speed)<=l.speed[j]+1e-12);REQUIRE(std::abs(s.direction[j]*p.accel)<=l.accel[j]+1e-12);}}},
{"All targets validate before a partial plan can escape",[]{Plan p;std::string e;REQUIRE(!plan({},{{{1,0,0,0,0},1},{{1000,0,0,0,0},1}},Limits{},25,25,p,e));REQUIRE(p.segments.empty());}},
{"Pure rotary G94 uses degrees per minute and never divides by XYZ zero",[]{Plan p;std::string e;REQUIRE(plan({},{{{0,0,0,10,0},60,FeedMode::PerMinute}},Limits{},25,25,p,e));REQUIRE(near(p.segments[0].speed,1));}},
{"G94 linear F60 and legacy F1 give the same plan",[]{Plan a,b;std::string e;REQUIRE(plan({},{{{10,0,0,0,0},60,FeedMode::PerMinute}},Limits{},25,25,a,e));REQUIRE(plan({},{{{10,0,0,0,0},1,FeedMode::LegacyPerSecond}},Limits{},25,25,b,e));REQUIRE(near(a.seconds,b.seconds));}},
{"G93 feasible time includes acceleration and deceleration",[]{Plan p;std::string e;REQUIRE(plan({},{{{1,0,0,0,0},6,FeedMode::InverseMinute}},Limits{},25,25,p,e));REQUIRE(near(p.seconds,10,1e-8));}},
{"G93 infeasible time is not silently stretched",[]{Plan p;std::string e;REQUIRE(!plan({},{{{10,0,0,0,0},600,FeedMode::InverseMinute}},Limits{},25,25,p,e));}},
{"Collinear forward joints stay continuous with segment-local speed",[]{Plan p;std::string e;REQUIRE(plan({},{{{1,0,0,1,0},1},{{3,0,0,3,0},2}},Limits{},25,25,p,e));REQUIRE(p.segments[0].exit>0&&near(p.segments[0].exit,p.segments[1].entry));}},
{"Direction changes and reversal are exact stops, not unbounded velocity jumps",[]{Plan p;std::string e;REQUIRE(plan({},{{{1,0,0,0,0},1},{{1,1,0,0,0},1},{{1,0,0,0,0},1}},Limits{},25,25,p,e));REQUIRE(p.segments[0].exit==0&&p.segments[1].exit==0);}},
{"Identical target is a valid zero-motion plan",[]{Plan p;std::string e;REQUIRE(plan({},{{{},1}},Limits{},25,25,p,e));REQUIRE(p.segments.empty()&&p.seconds==0);}},
{"XY arc preserves full CB endpoint and point-order",[]{Arc a;a.centerOffset[0]=-1;std::vector<Joint> q;std::string e;REQUIRE(arcWaypoints({1,0,0,0,0},{0,1,2,12,-6},a,q,e));REQUIRE((q.back()==Joint{0,1,2,12,-6}));for(const auto& p:q)REQUIRE(near(std::hypot(p[0],p[1]),1));}},
{"G18 ZX and G19 YZ are right-handed planes",[]{for(auto plane:{Plane::ZX,Plane::YZ}){Arc a;a.plane=plane;Joint s{},end{};int u=plane==Plane::ZX?2:1,v=plane==Plane::ZX?0:2;s[u]=1;end[v]=1;a.centerOffset[u]=-1;std::vector<Joint> q;std::string e;REQUIRE(arcWaypoints(s,end,a,q,e));REQUIRE(q.front()[v]>0);}}},
{"Signed R chooses minor or major arc",[]{std::vector<Joint>a,b;std::string e;Arc r;r.useRadius=true;r.radius=1;REQUIRE(arcWaypoints({1,0,0,0,0},{0,1,0,0,0},r,a,e));r.radius=-1;REQUIRE(arcWaypoints({1,0,0,0,0},{0,1,0,0,0},r,b,e));REQUIRE(b.size()>a.size()*2);}},
{"IJK full circle supported; ambiguous R full circle refused",[]{Arc a;a.centerOffset[0]=-1;std::vector<Joint>q;std::string e;Joint s{1,0,0,0,0};REQUIRE(arcWaypoints(s,s,a,q,e)&&q.size()>=36);a.useRadius=true;a.radius=1;REQUIRE(!arcWaypoints(s,s,a,q,e));}},
{"Arc chord tolerance bound",[]{Arc a;a.centerOffset[0]=-10;a.chordToleranceMm=.003;Joint prev{10,0,0,0,0};std::vector<Joint>q;std::string e;REQUIRE(arcWaypoints(prev,{0,10,0,0,0},a,q,e));for(const auto& p:q){double mx=(p[0]+prev[0])/2,my=(p[1]+prev[1])/2;REQUIRE(10-std::hypot(mx,my)<=.0030000001);prev=p;}}},
{"Three-point arc checks collinearity and direction",[]{Arc a;std::string e;REQUIRE(threePointArc({1,0,0,0,0},{0,1,0,0,0},{-1,0,0,0,0},false,a,e));REQUIRE(!threePointArc({1,0,0,0,0},{0,1,0,0,0},{-1,0,0,0,0},true,a,e));REQUIRE(!threePointArc({},{1,0,0,0,0},{2,0,0,0,0},false,a,e));}},
{"V32 Buffer-B schema fits actual 1600-float space",[]{auto h=header();std::string e;REQUIRE(Table5D::validate(h.data(),9,e));REQUIRE(Table5D::data+Table5D::capacity*Table5D::dimension==946);REQUIRE(1138<=Table5D::tableSize);}},
{"Old three-axis BASIC cannot pass V32 handshake",[]{auto h=header();h[8]=3;std::string e;REQUIRE(!Table5D::validate(h.data(),9,e));}},
{"TABLE session, nonintegral indexes and incomplete provenance rejected",[]{std::string e;auto h=header();REQUIRE(!Table5D::validate(h.data(),10,e));h[1]=1.5f;REQUIRE(!Table5D::validate(h.data(),9,e));h=header();h[13]=0;REQUIRE(!Table5D::validate(h.data(),9,e));}},
{"V32.9 header accepts full five-axis MPOS provenance",[]{auto h=header();std::string e;REQUIRE(Table5D::validate(h.data(),9,e));h[13]=24;h[14]=1;h[15]=24;h[16]=7;REQUIRE(!Table5D::validate(h.data(),9,e));}},
{"Wrong REGIST metadata is rejected by V32.9",[]{auto h=header();std::string e;h[19]=103;REQUIRE(!Table5D::validate(h.data(),9,e));REQUIRE(e.find("REGIST=104")!=std::string::npos);}},
{"Software sample convenience record is full measured MPOS after XYZ grating upgrade",[]{float v[6]={7,1,2,3,4,5};Frame5D f;std::string e;REQUIRE(decodeRecord(v,7,99,{10,-20},f,e));REQUIRE(f.joints[3]==14&&f.joints[4]==-15&&f.raw[3]==4&&f.raw[4]==5);REQUIRE(f.feedbackMask==31&&f.latchMask==0&&f.commandMask==0&&f.timing==PoseTiming::SoftwareInterpolated);}},
{"TABLE gap and NaN never produce valid pose",[]{float v[6]={7,1,2,3,4,5};Frame5D f;std::string e;REQUIRE(!decodeRecord(v,8,99,{0,0},f,e));v[4]=std::numeric_limits<float>::quiet_NaN();REQUIRE(!decodeRecord(v,7,99,{0,0},f,e));}},
{"Nominal FK uses negative C then negative B and full hand-eye",[]{Frame5D f;std::array<double,17> p{};std::string e;f.joints={1,2,3,90,0};REQUIRE(completeFrame(f,p,e));auto q=mapProfilePoint(f,2,5);REQUIRE(near(q[0],1)&&near(q[1],0)&&near(q[2],8));f.joints={0,0,0,0,90};REQUIRE(completeFrame(f,p,e));q=mapProfilePoint(f,0,2);REQUIRE(near(q[0],-2)&&near(q[2],0));}},
{"Missing or mirrored transform is invalid",[]{std::array<double,16> t{};REQUIRE(!validTransform(t));t={-1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};REQUIRE(!validTransform(t));}},
{"Archive preserves doubles, CB, hand-eye, exact timestamps and sparse point indices",[]{auto p=profile();std::string e;std::vector<std::uint8_t>b;std::vector<ArchiveProfile> q;REQUIRE(encodeArchive({p},b,e));REQUIRE(decodeArchive(b.data(),b.size(),q,e));REQUIRE(q.size()==1&&q[0].frame.joints==p.frame.joints&&q[0].frame.sensorToMachine==p.frame.sensorToMachine&&q[0].frame.physicalParameters==p.frame.physicalParameters);REQUIRE(q[0].timeUs==p.timeUs&&q[0].sensorTimeSec==p.sensorTimeSec&&q[0].points[1].index==1279&&q[0].points[0].z==p.points[0].z);}},
{"Archive point reconstruction is exact before/after reload",[]{auto p=profile();std::string e;std::vector<std::uint8_t>b;std::vector<ArchiveProfile> q;REQUIRE(encodeArchive({p},b,e)&&decodeArchive(b.data(),b.size(),q,e));REQUIRE(mapProfilePoint(p.frame,p.points[0].x,p.points[0].z)==mapProfilePoint(q[0].frame,q[0].points[0].x,q[0].points[0].z));}},
{"Empty profile keeps its pose, not a deleted ordinal",[]{auto p=profile();p.points.clear();std::string e;std::vector<std::uint8_t>b;std::vector<ArchiveProfile>q;REQUIRE(encodeArchive({p},b,e)&&decodeArchive(b.data(),b.size(),q,e));REQUIRE(q.size()==1&&q[0].frame.ordinal==7&&q[0].points.empty());}},
{"CRC rejects changed bytes and all truncated lengths",[]{std::vector<std::uint8_t>b;std::vector<ArchiveProfile>q;std::string e;REQUIRE(encodeArchive({profile()},b,e));for(std::size_t n=0;n<b.size();++n)REQUIRE(!decodeArchive(b.data(),n,q,e));b[40]^=1;REQUIRE(!decodeArchive(b.data(),b.size(),q,e));}},
{"Valid CRC does not permit malicious allocation count",[]{std::vector<std::uint8_t>b;std::vector<ArchiveProfile>q;std::string e;REQUIRE(encodeArchive({profile()},b,e));for(int i=8;i<16;++i)b[i]=255;auto c=archiveCrc32(b.data(),b.size()-4);for(int j=0;j<4;++j)b[b.size()-4+j]=std::uint8_t(c>>(8*j));REQUIRE(!decodeArchive(b.data(),b.size(),q,e));}},
{"Archive rejects duplicate point indices and stale calibration transforms",[]{auto p=profile();p.points[1].index=0;std::vector<std::uint8_t>b;std::string e;REQUIRE(!encodeArchive({p},b,e));p=profile();p.frame.physicalParameters[14]+=1;REQUIRE(!encodeArchive({p},b,e));}},
{"CRC table initialization is safe under concurrent calls",[]{std::vector<std::future<std::uint32_t>>fs;for(int i=0;i<16;++i)fs.push_back(std::async(std::launch::async,[]{return archiveCrc32("123456789",9);}));for(auto&f:fs)REQUIRE(f.get()==0xcbf43926u);}},
{"Seeded random five-axis plans satisfy projected kinematic limits",[]{std::mt19937 gen(210517);std::uniform_real_distribution<double>u(-10,10);for(int k=0;k<1000;++k){Joint q{u(gen),u(gen),u(gen),u(gen),u(gen)};Plan p;Limits l;std::string e;REQUIRE(plan({},{{q,300,FeedMode::PerMinute}},l,25,25,p,e));for(auto&s:p.segments)for(int j=0;j<5;++j)REQUIRE(std::abs(s.direction[j]*s.speed)<=l.speed[j]+1e-10);}}},
{"Bounded random input decoder never accepts malformed bytes",[]{std::mt19937 gen(5);std::vector<ArchiveProfile>q;std::string e;for(int k=0;k<1000;++k){std::vector<std::uint8_t>b(20+gen()%512);for(auto&x:b)x=std::uint8_t(gen());REQUIRE(!decodeArchive(b.data(),b.size(),q,e));}}}
});}
