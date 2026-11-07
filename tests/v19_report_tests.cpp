#include "TestSupport.h"
#include "core/HardwareSyncReport.h"
#include <cmath>
#include <fstream>
#include <limits>
using namespace syncdiag;
namespace {
bool near(double a,double b){return std::abs(a-b)<1e-6;}
HardwareReportInput synthetic6468() {
    HardwareReportInput in; in.poses=in.received=in.aligned=in.triggers=6468;
    in.validAligned=5698;in.emptyAligned=770;in.synchronized=true;in.triggerCountValid=true;
    in.exposureMs=1;in.idleMs=3;in.cloudPoints=5446272;
    for(std::uint32_t id=10950;id<=17417;++id){
        ProfileRecord r; r.index=id;r.shutterOpenSec=10.0+(id-10950)*0.005;
        // Synthetic regression data: match only the two supplied gap examples,
        // NOT a reconstruction of the user's unavailable raw exposure timestamps.
        if(id==16647)r.shutterOpenSec-=0.000010;
        if(id==16999)r.shutterOpenSec-=0.000140;
        r.callbackPcUs=1000000+(id-10950)*5000;r.pairing=PairingState::Aligned;
        r.validPoints=(id<=16643 || (id>=16647&&id<=16649) || id==16999)?1:0;
        in.frames.push_back(r);in.poseEstimateUs.push_back(1000000+(id-10950)*5000);
    }
    in.poseEstimateUs.back()+=5180;return in;
}
bool contains(const std::string& s,const std::string& text){return s.find(text)!=std::string::npos;}
}
int main(int argc,char** argv) {
 if(argc==3 && std::string(argv[1])=="--sample-report"){
    std::ofstream out(argv[2]);out<<"合成回归数据示例；不是新增实机试验，也不是用户原始时间戳重建。\n\n"<<hardwareSyncReport(synthetic6468());return out?0:1;
 }
 return runTests({
 {"6468-frame fixture retains 770 paired empty profiles",[]{auto in=synthetic6468();std::size_t valid=0;for(auto&r:in.frames)valid+=r.validPoints>0;REQUIRE(in.frames.size()==6468&&valid==5698&&in.frames.size()-valid==770);}},
 {"valid-only 16643->16647 gap uses four trigger periods",[]{auto g=effectiveGaps(synthetic6468().frames,5);REQUIRE(g.size()==2);auto r=g[0].timing;REQUIRE(r.deltaIndex==4&&near(r.expectedMs,20)&&near(r.actualMs,19.99)&&near(r.residualMs,-.01)&&!r.residualWarning&&g[0].confirmedEmpty==3);}},
 {"valid-only 16649->16999 gap uses 350 periods",[]{auto g=effectiveGaps(synthetic6468().frames,5);auto r=g[1].timing;REQUIRE(r.deltaIndex==350&&near(r.expectedMs,1750)&&near(r.actualMs,1749.86)&&near(r.residualMs,-.14)&&!r.residualWarning&&g[1].confirmedEmpty==349);}},
 {"all-frame sensor timing analyses all 6467 intervals",[]{auto t=analyzeReceivedTiming(synthetic6468().frames,5,true);REQUIRE(t.actual.count==6467&&t.warningCount==0&&t.invalidTime==0&&t.crossIndexIntervals==0);}},
 {"last 418 empty frames are retained after last valid frame",[]{auto in=synthetic6468();std::size_t tail=0;for(auto i=in.frames.rbegin();i!=in.frames.rend()&&i->validPoints==0;++i)++tail;REQUIRE(tail==418);}},
 {"formatter gives units and no false lost-trigger label",[]{auto text=hardwareSyncReport(synthetic6468());REQUIRE(contains(text,"其中有有效点=5698，无有效点=770")&&contains(text,"曝光时间残差超阈值=0")&&contains(text,"主机位置估计超阈值=1")&&contains(text,"5446272")&&!contains(text,"检测到 2 个触发异常"));}},
 {"report timing uses callback time not pose estimate",[]{auto in=synthetic6468();in.frames[1].callbackPcUs+=3000;auto t=analyzeReceivedTiming(in.frames,5,false);auto sensor=analyzeReceivedTiming(in.frames,5,true);REQUIRE(t.warningCount==2&&sensor.warningCount==0);}},
 {"real adjacent exposure delay is still warned",[]{auto r=interval(100,101,10.18,5);REQUIRE(r.residualWarning&&near(r.residualMs,5.18));}},
 {"forward missing index is not hidden by expected-time normalization",[]{std::vector<ProfileRecord> f{{100,1.0,0},{103,1.015,15000}};auto t=analyzeReceivedTiming(f,5,true);REQUIRE(t.crossIndexIntervals==1&&t.warningCount==0);HardwareReportInput in;in.missing=2;in.frames=f;REQUIRE(!contains(hardwareSyncReport(in),"计数与接收索引检查通过"));}},
 {"24-bit profile index wrap is valid",[]{auto r=interval(0xfffffe,1,15,5);REQUIRE(r.indexValid&&r.deltaIndex==3&&near(r.expectedMs,15));}},
 {"duplicate and backward index intervals not interpreted as elapsed triggers",[]{REQUIRE(!interval(5,5,5,5).indexValid&&!interval(5,4,5,5).indexValid);}},
 {"invalid and reset exposure timestamp not silently unwrapped",[]{REQUIRE(!interval(1,2,-1,5).timeValid&&!interval(1,2,std::numeric_limits<double>::quiet_NaN(),5).timeValid);}},
 {"pending or dropped frame is not classified as paired empty",[]{std::vector<ProfileRecord> f{{10,1,0,PairingState::Aligned,1},{11,1.005,0,PairingState::Dropped,0},{12,1.010,0,PairingState::Aligned,1}};auto g=effectiveGaps(f,5);REQUIRE(g.size()==1&&g[0].confirmedEmpty==0&&g[0].otherReceived==1);}},
 {"all-empty capture has exposure timing but no effective gaps",[]{std::vector<ProfileRecord> f{{10,1,0,PairingState::Aligned,0},{11,1.005,0,PairingState::Aligned,0}};REQUIRE(effectiveGaps(f,5).empty()&&analyzeReceivedTiming(f,5,true).actual.count==1);}},
 {"no data or trigger snapshot never claims success",[]{HardwareReportInput in;auto text=hardwareSyncReport(in);REQUIRE(!contains(text,"计数与接收索引检查通过")&&contains(text,"尚无有效停止后快照"));}},
 {"running mixed snapshot explicitly provisional",[]{auto in=synthetic6468();in.running=true;auto t=hardwareSyncReport(in);REQUIRE(contains(t,"当前仍在采集中")&&!contains(t,"计数与接收索引检查通过"));}},
 {"latched fault cannot be masked by matching counts",[]{auto in=synthetic6468();in.fault=true;in.faultReason="injected ACK ambiguity";auto t=hardwareSyncReport(in);REQUIRE(contains(t,in.faultReason)&&!contains(t,"计数与接收索引检查通过"));}},
 {"BASIC explanation matches V32.13 single-OP14 full-MPOS cold-prime protocol",[]{auto t=hardwareSyncReport(synthetic6468());REQUIRE(contains(t,"V32.13 ECI3808 SINGLE-OP14")&&contains(t,"REGIST(104)")&&contains(t,"EdgeRising")&&contains(t,"enable=3预热")&&contains(t,"至少8个可转换Profile")&&contains(t,"五轴均为ATYPE=4")&&contains(t,"machine=rawMPOS+offset")&&contains(t,"曝光相位"));}},
 {"V32.13 formal-session sensor callback diagnostics are reported",[]{auto in=synthetic6468();in.sensorRawCallbacks=3;in.sensorConvertedProfiles=2;in.sensorConvertFailures=1;in.sensorIngressDrops=1;in.sensorRawRingHighWater=7;in.sensorRawRingCapacity=128;in.sensorMaxCallbackServiceUs=55;in.sensorMaxDecoderQueueUs=800;in.sensorMaxDecodeServiceUs=1400;in.sensorTriggerReadbackValid=true;in.sensorTriggerRequested=0x3200000;in.sensorTriggerReadback=0x3200000;auto t=hardwareSyncReport(in);REQUIRE(contains(t,"原始SDK回调=3")&&contains(t,"转换成功=2")&&contains(t,"转换失败=1")&&contains(t,"raw-ring丢弃=1")&&contains(t,"7/128")&&contains(t,"decoder最大排队=800")&&contains(t,"readbackValid=1"));}},
 {"profile gap forensic localizes pre-callback gap when callback ordinals remain consecutive",[]{HardwareReportInput in;in.received=2;in.sensorIngressDrops=0;in.sensorConvertFailures=0;ProfileRecord a,b;a.index=797;a.shutterOpenSec=1.000;a.callbackPcUs=1000000;a.callbackOrdinal=100;a.ingressDepth=1;b.index=799;b.shutterOpenSec=1.010;b.callbackPcUs=1010000;b.callbackOrdinal=101;b.ingressDepth=2;in.frames={a,b};auto t=hardwareSyncReport(in);REQUIRE(contains(t,"GAP Index 797→799")&&contains(t,"ΔCallback=1")&&contains(t,"缺失索引未形成额外SDK callback")&&contains(t,"传感器/以太网/LLT callback之前"));}},
 {"profile gap forensic identifies host ingress participation evidence",[]{HardwareReportInput in;in.received=2;in.sensorIngressDrops=1;ProfileRecord a,b;a.index=100;a.callbackOrdinal=10;b.index=102;b.callbackOrdinal=12;in.frames={a,b};auto t=hardwareSyncReport(in);REQUIRE(contains(t,"GAP Index 100→102")&&contains(t,"ΔCallback=2")&&contains(t,"主机接收/解码链可能参与"));}},
 {"scheduler stats format includes queue and execution maxima separately",[]{eci::EciIoScheduler::Stats st;st.inFlight=true;st.inFlightName="blockedSDK";st.inFlightMs=123;auto t=schedulerReport(st);REQUIRE(contains(t,"blockedSDK")&&contains(t,"最大排队")&&contains(t,"最大事务执行")&&contains(t,"不能抢占"));}}
 });
}
