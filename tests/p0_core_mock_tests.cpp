#include "TestSupport.h"
#include "MockZaux.h"
#include "core/ZmcController.h"
#include "core/SyncManager.h"
#include "core/LaserSensor.h"
#include <limits>
#include <future>
#include <chrono>
#include "utils/GlobalTimeSync.h"
using namespace std::chrono_literals;
void ZmcController::errorOccurred(const QString&) {}
void ISyncStrategy::errorOccurred(const QString&) {}
void ISyncStrategy::alignedDataReady(const QVector<AlignedPoint>&) {}
void SyncManager::errorOccurred(const QString&) {}
void SyncManager::alignedDataReady(const QVector<AlignedPoint>&) {}
void SyncManager::runningChanged(bool) {}

struct P0ControllerTestAccess {
    static void ready(ZmcController& c) {
        p0::IoLock l(c.m_ioMutex);
        for(int axis:{AxisX,AxisY,AxisZ}) c.m_linearMposReferenced[axis]=true;
        for(int axis:{AxisC,AxisB}) {
            c.m_rotaryAbsReady[axis]=true; c.m_rotaryServoEnabled[axis]=true; c.m_rotaryHomeCompleted[axis]=true;
            c.m_rotaryMachineSoftLimitNeg[axis]=-1000;
            c.m_rotaryMachineSoftLimitPos[axis]=1000;
        }
        c.m_hwOffsets={10,-20};
        c.m_rotaryMachineOffset[AxisC]=10;
        c.m_rotaryMachineOffset[AxisB]=-20;
        std::lock_guard<std::mutex> m(mock.mutex);
        mock.dpos[AxisC]=mock.mpos[AxisC]=-10;
        mock.dpos[AxisB]=mock.mpos[AxisB]=20;
        mock.table[357]=31; // mirror synthetic XYZ/C/B power-session Home state
        // Test fixture models a prior successful 200Hz configuration from the host.
        mock.table[359]=1;mock.table[360]=1;
    }
    static void clearCHome(ZmcController& c) { p0::IoLock l(c.m_ioMutex); c.m_rotaryHomeCompleted[AxisC]=false; }
    static void rotaryAbsoluteReady(ZmcController& c) {
        p0::IoLock l(c.m_ioMutex);
        c.m_rotaryAbsReady[AxisC]=true; c.m_rotaryAbsReady[AxisB]=true;
        c.m_rotaryMachineOffset[AxisC]=10; c.m_rotaryMachineOffset[AxisB]=-20;
    }
    static void rotaryMachineLimits(ZmcController& c,int axis,double neg,double pos) {
        p0::IoLock l(c.m_ioMutex);
        c.m_rotaryMachineSoftLimitNeg[axis]=neg;
        c.m_rotaryMachineSoftLimitPos[axis]=pos;
    }
    static void jointBusy(ZmcController& c,bool busy) {
        // Enter through the same scheduler boundary as production; never hold its state lock while dispatching.
        if(busy) REQUIRE(c.tryStartJointTask5D(nullptr)); else c.finishJointTask5D();
    }
    static void completedMerge(ZmcController& c) {
        p0::IoLock l(c.m_ioMutex); c.m_mergePath={{1,2,3}};c.m_mergeIndex=1;
    }
};
struct P0SyncTestAccess {
    static void arm(HardwareSyncStrategy& s) {
        std::array<double,17> p{};p[13]=3.14159265358979323846/2; // explicit HE maps sensor X to machine Y
        REQUIRE(s.setKinematicModel(p,false));s.m_probeStartedUs=GlobalTimeSync::instance().timestampUs();
        s.m_running=true;s.m_syncEstablished=true;s.m_ingressSession.begin(); // queue-only tests, post-probe state
    }
    static void receive(HardwareSyncStrategy& s,quint32 id,bool empty=false) {
        LaserProfile p; p.index=id;p.tOpen=1;p.tPcUs=1;
        if(!empty){p.x={1};p.z={2};} s.onProfileReceived(p);
    }
    static void poses(HardwareSyncStrategy& s,int n) {
        QMutexLocker l(&s.m_poseMutex);
        for(int i=0;i<n;++i){SamplePoint p;p.x=double(s.m_allPoseSamples.size());p.tUs=5000*s.m_allPoseSamples.size();p.hasRotary=true;p.frame.ordinal=s.m_allPoseSamples.size();s.m_allPoseSamples.append(p);}
        s.m_poseSampleCount=s.m_allPoseSamples.size();
    }
    static void align(HardwareSyncStrategy& s) {s.onAlignTimer();}
    static void fault(HardwareSyncStrategy& s){s.invalidateAlignment("injected ambiguity");s.reportAlignmentFault();}
};
struct Fixture {
    ZmcController controller;
    Fixture(){mock.reset();REQUIRE(controller.connectTcp("127.0.0.1"));P0ControllerTestAccess::ready(controller);}
};
MockCall lastCall(const std::string& name) {
    std::lock_guard<std::mutex> l(mock.mutex);
    for(auto i=mock.calls.rbegin();i!=mock.calls.rend();++i) if(i->name==name)return *i;
    throw std::runtime_error("No mock call: "+name);
}
int countCalls(const std::string& name) {
    std::lock_guard<std::mutex> l(mock.mutex);
    int n=0; for(const auto& c:mock.calls) if(c.name==name) ++n; return n;
}
bool waitMotion() {
    for(int i=0;i<500;++i) {
        {std::lock_guard<std::mutex> l(mock.mutex); for(int a=1;a<=5;++a)if(mock.idle[a]==0)return true;}
        std::this_thread::sleep_for(1ms);
    }
    return false;
}
void ring(int read,int write) {
    std::lock_guard<std::mutex> l(mock.mutex);mock.table[332]=float(read);mock.table[331]=float(write);
    constexpr int cap=mls::runtime::Table5D::capacity,dim=mls::runtime::Table5D::dimension;
    int count=(write-read+cap)%cap;
    for(int i=0;i<count;++i){int slot=(read+i)%cap;float* p=mock.table.data()+mls::runtime::Table5D::data+dim*slot;
        p[0]=float(i);p[1]=float(slot);p[2]=10;p[3]=20;p[4]=-10;p[5]=20;}
    mock.table[336]=float(count);
}
JointMotionPlan5D plan() {
    JointMotionPlan5D p;p.valid=true;p.hasMotion=true;p.baseAxis=AxisC;
    p.target={1,2,3,4,5};p.vectorSpeed=1;p.vectorAccel=2;p.vectorDecel=2;
    return p;
}
bool waitQueued(ZmcController& c,std::size_t n=1) {
    for(int i=0;i<1000;++i) {if(c.ioStatistics().pending>=n)return true;std::this_thread::sleep_for(1ms);}return false;
}
// V27.6.35: transport assertions use synthetic tables, not a mechanics simulator.
void enableCompensation(Fixture& f,bool continuous=true,bool legacy=true,bool legacyZ=false,int settle=0,double precisionSpeed=0.0) {
    mls::axiscomp::Tables tables;
    for(int j=0;j<3;++j){auto& t=tables[j];t.axisIndex=j;t.settleMs=settle;
        t.method=(legacyZ&&j==2)?mls::axiscomp::SamplingMethod::PointToPointPreload:mls::axiscomp::SamplingMethod::MonotonicSweep;
        t.precisionSpeedMmS=precisionSpeed;
        for(auto& branch:t.directions)for(double x:{-200.,0.,200.})branch.push_back({x,x+0.01*(j+1)});
    }
    mls::axiscomp::Options options;options.pointSettleMs=settle;options.allowContinuous=continuous;
    options.allowLegacyPointTableOnPlannedMotion=legacy;
    QString error;REQUIRE(f.controller.installAxisCompensation(tables,options,&error));
    REQUIRE(!f.controller.axisCompensationEnabled());REQUIRE(f.controller.setAxisCompensationEnabled(true,&error));
}
bool nearValue(double a,double b,double tolerance=2e-6){return std::abs(a-b)<tolerance;}
int main(){return runTests({
    {"H1/H3 topology validity: retired ID=2 rejected",[]{
        using T=mls::runtime::Table5D;
        REQUIRE(T::validTopologyEdge(1,104));
        REQUIRE(T::validTopologyEdge(3,114) && T::validTopologyEdge(3,115));
        REQUIRE(!T::validTopologyEdge(1,114) && !T::validTopologyEdge(2,104));
        REQUIRE(!T::validTopologyEdge(2,114) && !T::validTopologyEdge(2,115));
        REQUIRE(!T::validTopologyEdge(4,114) && !T::validTopologyEdge(3,116));
        float retiredAck[4]={2,114,2,114};
        REQUIRE(!T::modeReady(retiredAck,2,114));
    }},
    {"Retired topology ID=2 rejected BEFORE any controller write",[]{
        Fixture f;QString err;using Mode=mls::runtime::HardwareSyncTopology;
        const int writes=countCalls("ZAux_Direct_SetTable");
        REQUIRE(!f.controller.setHardwareAcquisitionConfig(200,static_cast<Mode>(2),114,&err));
        REQUIRE(countCalls("ZAux_Direct_SetTable")==writes);
        REQUIRE(mock.table[365]==1 && mock.table[366]==104);
    }},
    {"Old BASIC runtime capability is rejected before any config write",[]{
        Fixture f;QString err;using Mode=mls::runtime::HardwareSyncTopology;
        { std::lock_guard<std::mutex> l(mock.mutex); mock.table[354]=321057; }
        const int writes=countCalls("ZAux_Direct_SetTable");
        REQUIRE(!f.controller.setHardwareAcquisitionConfig(200,Mode::EciPwmShared,104,&err));
        REQUIRE(countCalls("ZAux_Direct_SetTable")==writes);
    }},
    {"H3 internal CMM R1 continuous probe accepts multiple frames",[]{
        Fixture f;QString err;using Mode=mls::runtime::HardwareSyncTopology;
        REQUIRE(f.controller.setHardwareAcquisitionConfig(144,Mode::SensorInternalCmm,115,&err));
        REQUIRE(mock.table[349]==115 && mock.table[347]==1 && mock.table[350]==0);
        REQUIRE(f.controller.hwTableSampleStart(&err));
        REQUIRE(mock.table[369]==1 && mock.table[334]==2);
        { std::lock_guard<std::mutex> l(mock.mutex);mock.table[335]=12;mock.table[336]=12; }
        REQUIRE(f.controller.hwTableContinueAfterProbe(&err));
        REQUIRE(mock.table[334]==1);
        REQUIRE(f.controller.hwTableSampleStop(&err));
    }},
    {"H3 rejects unacknowledged topology or altered R1 edge",[]{
        Fixture f;QString err;using Mode=mls::runtime::HardwareSyncTopology;
        REQUIRE(f.controller.setHardwareAcquisitionConfig(200,Mode::SensorInternalCmm,114,&err));
        { std::lock_guard<std::mutex> l(mock.mutex);mock.table[368]=115; }
        REQUIRE(!f.controller.hwTableSampleStart(&err));
        REQUIRE(mock.table[334]==0);
    }},

    {"V55 reject NaN BASIC state before frequency config",[]{
        Fixture f;QString err;
        {std::lock_guard<std::mutex> l(mock.mutex);mock.table[358]=std::numeric_limits<float>::quiet_NaN();}
        const int writes=countCalls("ZAux_Direct_SetTable");
        REQUIRE(!f.controller.setHardwareTriggerFrequencyHz(333,&err));
        REQUIRE(countCalls("ZAux_Direct_SetTable")==writes);
    }},
    {"V55 reject fractional frequency ACK sequence",[]{
        Fixture f;QString err;
        {std::lock_guard<std::mutex> l(mock.mutex);mock.table[360]=1.5f;}
        const int writes=countCalls("ZAux_Direct_SetTable");
        REQUIRE(!f.controller.setHardwareTriggerFrequencyHz(333,&err));
        REQUIRE(countCalls("ZAux_Direct_SetTable")==writes);
    }},
    {"V55 reject NaN stop status rather than casting",[]{
        Fixture f;QString err;
        {std::lock_guard<std::mutex> l(mock.mutex);mock.table[334]=std::numeric_limits<float>::quiet_NaN();}
        REQUIRE(!f.controller.hwTableWaitIdleReady(50,&err));
    }},
    {"V55 refuse sensor ColdPrime while C HOME unconfirmed",[]{
        Fixture f;QString err;P0ControllerTestAccess::clearCHome(f.controller);
        const int writes=countCalls("ZAux_Direct_SetTable");
        REQUIRE(!f.controller.hwSensorColdPrimeStart(&err));
        REQUIRE(countCalls("ZAux_Direct_SetTable")==writes);
    }},
    {"AUDIT reject fractional persisted HOME mask",[]{
        Fixture f; AxisParams motion;
        {std::lock_guard<std::mutex> l(mock.mutex); mock.table[357]=30.5f;}
        const int moves=countCalls("ZAux_Direct_Single_MoveAbs");
        REQUIRE(!f.controller.homeRotaryAxisToZero(AxisC,motion));
        REQUIRE(countCalls("ZAux_Direct_Single_MoveAbs")==moves);
    }},
    {"AUDIT reject preflight and sample when C home unconfirmed",[]{
        Fixture f; P0ControllerTestAccess::clearCHome(f.controller);
        QString summary,err; bool auth=false;
        REQUIRE(!f.controller.hwTableCommissioningPreflight(summary,&err,&auth));
        REQUIRE(!f.controller.hwTableSampleStart(&err));
        REQUIRE(mock.table[334]==0);
    }},
    {"AUDIT reject fractional requested or ACK Hz",[]{
        Fixture f; QString err; int req=0,ack=0;
        {std::lock_guard<std::mutex> l(mock.mutex); mock.table[356]=200.75f;}
        REQUIRE(!f.controller.readHardwareTriggerFrequencyHz(req,ack,&err));
    }},
    {"AUDIT fail closed UNITS after home state read failure",[]{
        Fixture f; AxisParams p; p.units=200.0f; QString err;
        const int old=countCalls("ZAux_Direct_SetUnits");
        {std::lock_guard<std::mutex> l(mock.mutex);mock.failAllTableReads=true;}
        REQUIRE(!f.controller.writeAxisParams(AxisX,p,&err));
        REQUIRE(countCalls("ZAux_Direct_SetUnits")==old);
    }},
    {"AUDIT no C motion on missing actual persisted HOME clear",[]{
        Fixture f; AxisParams motion;
        {std::lock_guard<std::mutex> l(mock.mutex); mock.ignoreHomeMaskWrites=true;}
        const int moves=countCalls("ZAux_Direct_Single_MoveAbs");
        REQUIRE(!f.controller.homeRotaryAxisToZero(AxisC,motion));
        REQUIRE(countCalls("ZAux_Direct_Single_MoveAbs")==moves);
    }},
    {"AUDIT stale HOME restore rejects fake success on mask correction",[]{
        Fixture f; f.controller.disconnect();
        {std::lock_guard<std::mutex> l(mock.mutex);
         mock.table[357]=31; mock.homeStatus.fill(0); mock.homeStatus[AxisX]=1;
         mock.ignoreHomeMaskWrites=true;}
        REQUIRE(f.controller.connectTcp("127.0.0.1"));
        P0ControllerTestAccess::rotaryAbsoluteReady(f.controller);
        REQUIRE(!f.controller.isAxisHomed(AxisC));
        REQUIRE(!f.controller.isAxisHomed(AxisB));
    }},

    {"V27636 MachineSafety uses actual compensated independent-axis speeds and envelope",[]{Fixture f;enableCompensation(f,true,true,false,0,2.0);
        std::array<AxisParams,Constants::kAxisParamArraySize> requested{},effective{};
        for(auto& p:requested){p.speed=10.0f;p.accel=25.0f;p.decel=25.0f;}
        QString error;REQUIRE(f.controller.effectiveIndependentAxisParams5D(requested,0.5,effective,&error));
        for(int axis=AxisX;axis<=AxisZ;++axis)REQUIRE(nearValue(effective[axis].speed,4.0));
        REQUIRE(nearValue(effective[AxisC].speed,10.0)&&nearValue(effective[AxisB].speed,10.0));
        double envelope=0.0;REQUIRE(f.controller.axisCompensationSafetyEnvelopeMm(envelope,&error));
        REQUIRE(nearValue(envelope,0.06,1e-8));
    }},
    {"V27635 XYZ absolute maps once and raw calibration bypasses",[]{Fixture f;enableCompensation(f);
        REQUIRE(f.controller.moveAbs(AxisX,10));REQUIRE(nearValue(lastCall("ZAux_Direct_Single_MoveAbs").target[0],10.01));
        REQUIRE(f.controller.moveAbsRawDpos(AxisX,12));REQUIRE(nearValue(lastCall("ZAux_Direct_Single_MoveAbs").target[0],12));
        REQUIRE(f.controller.axisCompensationEnabled());
    }},
    {"V27635 relative physical motion starts at MPOS not DPOS",[]{Fixture f;enableCompensation(f);
        {std::lock_guard<std::mutex> l(mock.mutex);mock.dpos[AxisX]=10.01f;mock.mpos[AxisX]=9.995f;}
        REQUIRE(f.controller.moveRel(AxisX,1));REQUIRE(nearValue(lastCall("ZAux_Direct_Single_MoveAbs").target[0],11.005));
        REQUIRE(countCalls("ZAux_Direct_Single_Move")==0);
        REQUIRE(f.controller.moveRelRawDpos(AxisX,1));REQUIRE(countCalls("ZAux_Direct_Single_Move")==1);
    }},
    {"V27635 repeated nominal target preserves demand with residual feedback",[]{Fixture f;enableCompensation(f);
        REQUIRE(f.controller.moveAbs(AxisX,10));
        {std::lock_guard<std::mutex> l(mock.mutex);mock.mpos[AxisX]=10.002f;}
        REQUIRE(f.controller.moveAbs(AxisX,10));REQUIRE(nearValue(lastCall("ZAux_Direct_Single_MoveAbs").target[0],10.01));
    }},
    {"V27635 five-axis synchronous maps XYZ leaves rotary offsets unchanged",[]{Fixture f;enableCompensation(f);
        JointMoveResult5D result;REQUIRE(f.controller.moveJointAbs5D(plan(),result));
        const auto call=lastCall("ZAux_Direct_MoveAbs");REQUIRE(call.axes==std::vector<int>({4,1,2,3,5}));
        REQUIRE(nearValue(call.target[0],-6)&&nearValue(call.target[1],1.01)&&nearValue(call.target[2],2.02)&&nearValue(call.target[3],3.03)&&nearValue(call.target[4],25));
        REQUIRE(result.completed&&result.maxLinearDposErrorMm<2e-6);
    }},
    {"V27635 independent Stage2 measured XYZ hold does not chase residual",[]{Fixture f;enableCompensation(f);
        REQUIRE(f.controller.moveAbs(AxisX,10));
        {std::lock_guard<std::mutex> l(mock.mutex);mock.mpos[AxisX]=10.002f;}
        JointState5D state;REQUIRE(f.controller.readJointState5D(state));auto target=state.mpos;target.c=5;
        std::array<AxisParams,Constants::kAxisParamArraySize> params{};JointMoveResult5D result;
        const int before=countCalls("ZAux_Direct_Single_MoveAbs");
        REQUIRE(f.controller.moveJointAbs5DIndependent(target,params,1,result));
        REQUIRE(countCalls("ZAux_Direct_Single_MoveAbs")==before+1);
        REQUIRE(lastCall("ZAux_Direct_Single_MoveAbs").axes==std::vector<int>({AxisC}));
        REQUIRE(nearValue(mock.dpos[AxisX],10.01)&&result.maxLinearDposErrorMm<2e-6);
    }},
    {"V27635 merged demand path maps once and preserves held Z",[]{Fixture f;enableCompensation(f);
        AxisParams p;PathPoint target;target.x=1;target.y=2;target.axisMask=3;
        REQUIRE(f.controller.prepareMergePath({target},p));bool finished=false;REQUIRE(f.controller.pumpMergeBuffer(finished)&&finished);
        const auto c=lastCall("ZAux_Direct_MoveAbsSp");REQUIRE(nearValue(c.target[0],1.01)&&nearValue(c.target[1],2.02)&&nearValue(c.target[2],0));
    }},
    {"V27635 compensated queue splits at interior calibration knot",[]{Fixture f;enableCompensation(f);
        {std::lock_guard<std::mutex> l(mock.mutex);mock.dpos[AxisX]=-1;mock.mpos[AxisX]=-1;}
        AxisParams p;PathPoint target;target.x=1;target.axisMask=1;
        REQUIRE(f.controller.prepareMergePath({target},p));bool finished=false;REQUIRE(f.controller.pumpMergeBuffer(finished)&&finished);
        REQUIRE(countCalls("ZAux_Direct_MoveAbsSp")==2);REQUIRE(nearValue(lastCall("ZAux_Direct_MoveAbsSp").target[0],1.01));
    }},
    {"V27635 RTCP measured start preserves XYZ through rotary-only path",[]{Fixture f;enableCompensation(f);
        REQUIRE(f.controller.moveAbs(AxisX,10));{std::lock_guard<std::mutex> l(mock.mutex);mock.mpos[AxisX]=10.002f;}
        JointState5D state;REQUIRE(f.controller.readJointState5D(state));
        PathPoint p;p.x=state.mpos.x;p.y=state.mpos.y;p.z=state.mpos.z;p.c=2;p.b=0;p.axisMask=31;
        AxisParams motion;REQUIRE(f.controller.prepareMergePath({p},motion,nullptr,ZmcController::PathStartReference::MeasuredMpos));
        bool done=false;REQUIRE(f.controller.pumpMergeBuffer(done)&&done);const auto call=lastCall("ZAux_Direct_MoveAbsSp");REQUIRE(nearValue(call.target[0],10.01));
    }},
    {"V27635 continuous and legacy Z permissions fail closed",[]{Fixture f;enableCompensation(f,false,false,true);
        AxisParams p;PathPoint target;target.x=1;target.axisMask=1;
        REQUIRE(!f.controller.prepareMergePath({target},p));REQUIRE(!f.controller.moveAbs(AxisZ,1));
        REQUIRE(countCalls("ZAux_Direct_MoveAbsSp")==0&&countCalls("ZAux_Direct_Single_MoveAbs")==0);
        REQUIRE(f.controller.moveAbs(AxisX,1));
    }},
    {"V27635 rotary-only motion does not require continuous XYZ permission",[]{Fixture f;enableCompensation(f,false,false,true);
        AxisParams p;PathPoint target;target.c=2;target.axisMask=8;
        REQUIRE(f.controller.prepareMergePath({target},p));bool done=false;REQUIRE(f.controller.pumpMergeBuffer(done));
        REQUIRE(nearValue(lastCall("ZAux_Direct_MoveAbsSp").target[2],0));
    }},
    {"V27635 out of range never falls back to raw",[]{Fixture f;enableCompensation(f);
        REQUIRE(!f.controller.moveAbs(AxisX,201));REQUIRE(countCalls("ZAux_Direct_Single_MoveAbs")==0);
        AxisParams p;PathPoint a;a.x=1;a.axisMask=1;PathPoint b=a;b.x=201;
        REQUIRE(!f.controller.prepareMergePath({a,b},p));REQUIRE(countCalls("ZAux_Direct_MoveAbsSp")==0);
    }},
    {"V27635 compensation cannot be reconfigured while queue is pending",[]{Fixture f;enableCompensation(f);
        AxisParams p;PathPoint target;target.x=1;target.axisMask=1;REQUIRE(f.controller.prepareMergePath({target},p));
        const int before=mock.callCount;REQUIRE(!f.controller.setAxisCompensationEnabled(false));REQUIRE(mock.callCount==before);
        REQUIRE(f.controller.stopAxis(AxisX,false));REQUIRE(f.controller.setAxisCompensationEnabled(false));
    }},
    {"V27635 feedback access is never compensated",[]{Fixture f;enableCompensation(f);
        {std::lock_guard<std::mutex> l(mock.mutex);mock.dpos[AxisX]=3;mock.mpos[AxisX]=2.99f;mock.encoder[AxisX]=123;}
        JointState5D state;REQUIRE(f.controller.readJointState5D(state));
        REQUIRE(nearValue(state.dpos.x,3)&&nearValue(state.mpos.x,2.99));
    }},
    {"V27635 compensated arcs use checked SP polyline not raw SDK circle",[]{Fixture f;enableCompensation(f);AxisParams p;
        REQUIRE(f.controller.moveArcCenterAbs(1,0,2,0,true,p));
        REQUIRE(countCalls("ZAux_Direct_MoveCirc2Abs")==0&&countCalls("ZAux_Direct_MoveAbsSp")>0);
    }},
    {"V27635 demand send failure cancels queue and cannot refill stale path",[]{Fixture f;enableCompensation(f);AxisParams p;PathPoint q;q.x=1;q.axisMask=1;
        REQUIRE(f.controller.prepareMergePath({q},p));mock.failSp=true;bool done=false;REQUIRE(!f.controller.pumpMergeBuffer(done));
        REQUIRE(countCalls("ZAux_Direct_CancelAxisList")>0);mock.failSp=false;const int before=countCalls("ZAux_Direct_MoveAbsSp");
        REQUIRE(f.controller.pumpMergeBuffer(done)&&done);REQUIRE(countCalls("ZAux_Direct_MoveAbsSp")==before);
    }},
    {"V27635 stop interrupts compensated settle without owning IO worker",[]{Fixture f;enableCompensation(f,true,true,false,500);
        auto task=std::async(std::launch::async,[&]{return f.controller.moveAbs(AxisX,1);});
        bool issued=false;for(int i=0;i<300;++i){if(countCalls("ZAux_Direct_Single_MoveAbs")>0){issued=true;break;}std::this_thread::sleep_for(1ms);}
        const bool stopped=f.controller.stopAxis(AxisX,false);const bool result=task.get();REQUIRE(issued&&stopped&&!result&&mock.maxInFlight==1);
    }},
    {"V27635 reconnect revokes runtime enable",[]{Fixture f;enableCompensation(f);f.controller.disconnect();
        REQUIRE(!f.controller.axisCompensationEnabled());REQUIRE(f.controller.connectTcp("127.0.0.1"));
        REQUIRE(!f.controller.axisCompensationEnabled()&&!f.controller.setAxisCompensationEnabled(true));
    }},

    {"absolute XY arc uses physical axes 1,2",[]{Fixture f;AxisParams p;REQUIRE(f.controller.moveArc2Abs(1,1,2,0,true,p));REQUIRE(lastCall("ZAux_Direct_MoveCirc2Abs").axes==std::vector<int>({1,2}));}},
    {"relative XY arc uses physical axes 1,2",[]{Fixture f;AxisParams p;REQUIRE(f.controller.moveArc2Rel(1,1,2,0,false,p));REQUIRE(lastCall("ZAux_Direct_MoveCirc2").axes==std::vector<int>({1,2}));}},
    {"center XY arc uses physical axes 1,2",[]{Fixture f;AxisParams p;REQUIRE(f.controller.moveArcCenterAbs(1,0,2,0,true,p));REQUIRE(lastCall("ZAux_Direct_MoveCirc2Abs").axes==std::vector<int>({1,2}));}},
    {"parallel actual core calls never overlap SDK",[]{
        Fixture f;mock.delayUs=150;std::atomic<bool> ok{true};std::vector<std::thread> ts;
        for(int i=0;i<4;++i)ts.emplace_back([&]{for(int k=0;k<20;++k){double x,y,z;if(!f.controller.readAxisPositions(x,y,z))ok=false;}});
        for(auto& t:ts)t.join();REQUIRE(ok.load() && mock.maxInFlight==1);
    }},
    {"OpenEth remains on object GUI thread",[]{Fixture f;int old=mock.opens;bool result=true;std::thread t([&]{result=f.controller.connectTcp("127.0.0.2");});t.join();REQUIRE(!result && mock.opens==old);}},
    {"actual motion wait releases lock for stop and reads",[]{
        Fixture f;mock.holdMotion=true;
        auto task=std::async(std::launch::async,[&]{return f.controller.moveAbs(AxisX,10);});
        const bool moving=waitMotion();double x,y,z;const bool read=f.controller.readAxisPositions(x,y,z);
        const bool stopped=f.controller.stopAxis(AxisX,false);auto result=task.get();
        REQUIRE(moving && read && stopped && !result && mock.maxInFlight==1);
    }},
    {"disconnect/reconnect rejected while worker retains handle",[]{
        Fixture f;mock.holdMotion=true;auto task=std::async(std::launch::async,[&]{return f.controller.moveAbs(AxisX,10);});
        const bool moving=waitMotion();const int closes=mock.closes;
        f.controller.disconnect();const bool connected=f.controller.isConnected();const bool reconnected=f.controller.connectTcp("127.0.0.2");
        const int duringCloses=mock.closes;f.controller.stopAxis(AxisX,false);task.get();f.controller.disconnect();
        REQUIRE(moving && connected && !reconnected && duringCloses==closes && !f.controller.isConnected());
    }},
    {"five-axis plan preserves BASE order and mechanical offsets",[]{
        Fixture f;P0ControllerTestAccess::ready(f.controller);JointMoveResult5D result;
        REQUIRE(f.controller.moveJointAbs5D(plan(),result));
        const auto c=lastCall("ZAux_Direct_MoveAbs");REQUIRE(c.axes==std::vector<int>({4,1,2,3,5}));
        REQUIRE(c.target==std::vector<float>({-6,1,2,3,25}));REQUIRE(result.completed && result.maxRotaryMposErrorDeg<1e-6);
    }},
    {"single rotary stop routes through active five-axis group",[]{
        Fixture f;P0ControllerTestAccess::ready(f.controller);mock.holdMotion=true;
        auto task=std::async(std::launch::async,[&]{JointMoveResult5D r;return f.controller.moveJointAbs5D(plan(),r);});
        const bool moving=waitMotion();const bool stopped=f.controller.stopAxis(AxisB,false);const bool done=task.get();
        const auto c=lastCall("ZAux_Direct_CancelAxisList");REQUIRE(moving && stopped && !done && c.mode==2 && c.axes==std::vector<int>({1,2,3,4,5}));
    }},
    {"active five-axis task rejects legacy writes before SDK",[]{
        Fixture f;P0ControllerTestAccess::jointBusy(f.controller,true);const int old=mock.callCount;AxisParams p;
        const bool a=f.controller.moveLineAbs({1,2,3},p);const bool b=f.controller.writeMotionParams(AxisX,p);
        const bool c=f.controller.hwTableSampleStart();const bool d=f.controller.moveArc2Abs(1,1,2,0,true,p);
        const int count=mock.callCount;P0ControllerTestAccess::jointBusy(f.controller,false);
        REQUIRE(!a && !b && !c && !d && old==count);
    }},
    {"completed merge timer is harmless during five-axis task",[]{
        Fixture f;P0ControllerTestAccess::completedMerge(f.controller);P0ControllerTestAccess::jointBusy(f.controller,true);
        const int old=mock.callCount;bool finished=false;const bool ok=f.controller.pumpMergeBuffer(finished);
        const int count=mock.callCount;P0ControllerTestAccess::jointBusy(f.controller,false);REQUIRE(ok && finished && old==count);
    }},
    {"stop clears pending host merge to prevent refeeding",[]{
        Fixture f;AxisParams p;REQUIRE(f.controller.prepareMergePath({{1,2,3},{4,5,6}},p));
        REQUIRE(f.controller.stopAxis(AxisX,false));const int old=mock.callCount;bool finished=false;
        REQUIRE(f.controller.pumpMergeBuffer(finished) && finished && mock.callCount==old);
    }},
    {"actual wrapped TABLE failure does not publish or advance",[]{
        Fixture f;ring(94,2);mock.failDataRead=2;QVector<SamplePoint> out;bool overflow=false;
        REQUIRE(!f.controller.hwTableSampleReadOnce(out,overflow));REQUIRE(out.isEmpty() && f.controller.getHwSampleCount()==0 && mock.table[332]==94);
        mock.failDataRead=0;REQUIRE(f.controller.hwTableSampleReadOnce(out,overflow));
        REQUIRE(out.size()==4 && out[0].x==94 && out[2].x==0 && f.controller.getHwSampleCount()==4 && mock.table[332]==2);
    }},
    {"actual TABLE lost ACK latches and forbids blind retry",[]{
        Fixture f;ring(0,2);mock.failAck=true;QVector<SamplePoint> out;bool overflow=false;
        REQUIRE(!f.controller.hwTableSampleReadOnce(out,overflow));REQUIRE(out.isEmpty() && f.controller.getHwSampleCount()==0 && f.controller.hwTableReadFaulted());
        const int old=mock.callCount;mock.failAck=false;REQUIRE(!f.controller.hwTableSampleReadOnce(out,overflow) && mock.callCount==old);
        REQUIRE(f.controller.hwTableSampleStart() && !f.controller.hwTableReadFaulted());f.controller.hwTableSampleStop();
    }},
    {"actual TABLE sticky overflow publishes nothing",[]{
        Fixture f;ring(0,2);mock.table[333]=1;QVector<SamplePoint> out;bool overflow=false;
        REQUIRE(f.controller.hwTableSampleReadOnce(out,overflow) && overflow && out.isEmpty() && f.controller.hwTableReadFaulted());
        REQUIRE(mock.table[332]==0 && f.controller.getHwSampleCount()==0);
    }},
    {"V32.2 hardware TABLE converts raw incremental C/B MPOS through frozen RS485 absolute offsets",[]{
        Fixture f;ring(0,1);QVector<SamplePoint> out;bool overflow=false;
        REQUIRE(f.controller.hwTableSampleReadOnce(out,overflow));
        REQUIRE(!overflow && out.size()==1);
        // ring() publishes raw C=-10,B=20 while Fixture freezes offsets +10,-20.
        // Machine coordinates must therefore be C=0,B=0, not the raw encoder coordinates.
        REQUIRE(std::abs(out[0].c)<1e-12 && std::abs(out[0].b)<1e-12);
        REQUIRE(std::abs(out[0].frame.raw[3]+10)<1e-12 && std::abs(out[0].frame.raw[4]-20)<1e-12);
        REQUIRE(std::abs(out[0].frame.rotaryOffsets[0]-10)<1e-12 && std::abs(out[0].frame.rotaryOffsets[1]+20)<1e-12);
    }},
    {"hardware previews cannot contaminate inactive strategy",[]{
        Fixture f;LaserSensor laser;HardwareSyncStrategy s(&laser,&f.controller);P0SyncTestAccess::receive(s,100);REQUIRE(s.getAlignStats().totalProfiles==0);
    }},
    {"actual hardware queue preserves pending FIFO order",[]{
        Fixture f;LaserSensor laser;HardwareSyncStrategy s(&laser,&f.controller);P0SyncTestAccess::arm(s);
        P0SyncTestAccess::receive(s,10);P0SyncTestAccess::receive(s,11);P0SyncTestAccess::align(s);REQUIRE(s.getAlignStats().pendingProfiles==2);
        P0SyncTestAccess::poses(s,2);P0SyncTestAccess::align(s);auto p=s.getAllAlignedPoints();
        REQUIRE(p.size()==2 && p[0].profileIndex==10 && p[1].profileIndex==11 && std::abs(p[0].machineX)<1e-12 && std::abs(p[1].machineX-1)<1e-12);
    }},
    {"actual missing profile keeps its empty pose ordinal",[]{
        Fixture f;LaserSensor laser;HardwareSyncStrategy s(&laser,&f.controller);P0SyncTestAccess::arm(s);P0SyncTestAccess::poses(s,3);
        P0SyncTestAccess::receive(s,100);P0SyncTestAccess::receive(s,102);P0SyncTestAccess::align(s);
        auto p=s.getAllAlignedPoints();REQUIRE(p.size()==2 && p[1].machineX==2 && s.getAlignStats().missingProfileIndices==1);
    }},
    {"actual profile counter wrap aligns monotonically",[]{
        Fixture f;LaserSensor laser;HardwareSyncStrategy s(&laser,&f.controller);P0SyncTestAccess::arm(s);P0SyncTestAccess::poses(s,3);
        P0SyncTestAccess::receive(s,0xfffffe);P0SyncTestAccess::receive(s,0xffffff);P0SyncTestAccess::receive(s,0);P0SyncTestAccess::align(s);
        auto p=s.getAllAlignedPoints();REQUIRE(p.size()==3 && p[2].machineX==2 && !s.getAlignStats().alignmentFault);
    }},
    {"empty profile still occupies exactly one pose",[]{
        Fixture f;LaserSensor laser;HardwareSyncStrategy s(&laser,&f.controller);P0SyncTestAccess::arm(s);P0SyncTestAccess::poses(s,2);
        P0SyncTestAccess::receive(s,100,true);P0SyncTestAccess::receive(s,101);P0SyncTestAccess::align(s);
        auto p=s.getAllAlignedPoints();REQUIRE(p.size()==1 && p[0].machineX==1 && s.getAlignStats().alignedProfiles==2);
    }},
    {"duplicate profile drops without shifting pose",[]{
        Fixture f;LaserSensor laser;HardwareSyncStrategy s(&laser,&f.controller);P0SyncTestAccess::arm(s);P0SyncTestAccess::poses(s,2);
        P0SyncTestAccess::receive(s,100);P0SyncTestAccess::receive(s,100);P0SyncTestAccess::receive(s,101);P0SyncTestAccess::align(s);
        auto p=s.getAllAlignedPoints();REQUIRE(p.size()==2 && p[1].machineX==1 && s.getAlignStats().duplicateProfiles==1);
    }},
    {"index reset faults instead of reanchoring pose zero",[]{
        Fixture f;LaserSensor laser;HardwareSyncStrategy s(&laser,&f.controller);P0SyncTestAccess::arm(s);P0SyncTestAccess::poses(s,3);
        P0SyncTestAccess::receive(s,100);P0SyncTestAccess::align(s);P0SyncTestAccess::receive(s,0);P0SyncTestAccess::receive(s,1);P0SyncTestAccess::align(s);
        auto st=s.getAlignStats();REQUIRE(st.alignmentFault && !st.syncEstablished && st.outOfOrderProfiles==1 && s.getAllAlignedPoints().size()==1);
    }},
    {"stats getter never calls controller SDK",[]{
        Fixture f;LaserSensor laser;HardwareSyncStrategy s(&laser,&f.controller);const int old=mock.callCount;
        for(int i=0;i<20;++i){auto stats=s.getAlignStats();REQUIRE(!stats.zmcTriggerCountValid);}REQUIRE(mock.callCount==old);
    }},
    {"real table reader latches overflow once and stops trigger",[]{
        Fixture f;LaserSensor laser;HardwareSyncStrategy s(&laser,&f.controller);REQUIRE(s.setKinematicModel({},false));REQUIRE(s.start());
        {std::lock_guard<std::mutex> l(mock.mutex);mock.table[333]=1;}
        bool fault=false;for(int i=0;i<500;++i){if(s.getAlignStats().alignmentFault){fault=true;break;}std::this_thread::sleep_for(1ms);}
        s.stop();const auto st=s.getAlignStats();REQUIRE(fault && st.tableOverflows==1 && mock.table[334]==0);
    }},
    {"V19 connected SDK calls share one worker; OpenEth is GUI lifecycle exception",[]{
        Fixture f;const auto gui=std::this_thread::get_id();double x,y,z;
        REQUIRE(f.controller.readAxisPositions(x,y,z));AxisParams params;REQUIRE(f.controller.readAxisParams(AxisX,params));
        f.controller.disconnect();std::thread::id owner;
        std::lock_guard<std::mutex> lock(mock.mutex);
        for(const auto& c:mock.calls) {
            if(c.name=="ZAux_OpenEth")REQUIRE(c.threadId==gui);
            else {if(owner==std::thread::id{})owner=c.threadId;REQUIRE(c.threadId==owner&&owner!=gui);}
        }
    }},
    {"V19 GUI STOP queues while SDK is blocked and cancels not-started movement",[]{
        Fixture f;mock.armBlock("ZAux_Direct_GetMpos");
        auto read=std::async(std::launch::async,[&]{float d,m;bool idle;return f.controller.getAxisPositionPair(AxisX,d,m,idle);});
        const bool blocked=mock.waitBlocked();
        auto move=std::async(std::launch::async,[&]{return f.controller.moveAbs(AxisX,10);});
        const bool queued=waitQueued(f.controller);
        f.controller.requestStopAllAxes(false); // must return while blocked SDK still in flight
        const bool stillBlocked=f.controller.ioStatistics().inFlight;
        mock.releaseBlock();const bool readOK=read.get();const bool moveOK=move.get();
        float d,m;bool idle;f.controller.getAxisPositionPair(AxisX,d,m,idle); // flush after Emergency
        bool moved=false;{std::lock_guard<std::mutex> lock(mock.mutex);for(const auto& c:mock.calls)moved|=c.name=="ZAux_Direct_Single_MoveAbs";}
        REQUIRE(blocked&&queued&&stillBlocked&&readOK&&!moveOK&&!moved&&mock.maxInFlight==1);
    }},
    {"V19 stop during acquisition handshake prevents delayed enable",[]{
        Fixture f;mock.armBlock("ZAux_Direct_SetTable");
        auto start=std::async(std::launch::async,[&]{return f.controller.hwTableSampleStart();});
        const bool blocked=mock.waitBlocked();
        auto stop=std::async(std::launch::async,[&]{return f.controller.hwTableSampleStop();});
        const bool queued=waitQueued(f.controller);mock.releaseBlock();
        const bool stopped=stop.get();const bool started=start.get();bool enabled=false;
        {std::lock_guard<std::mutex> lock(mock.mutex);for(const auto& c:mock.calls)
            enabled|=c.name=="ZAux_Direct_SetTable"&&c.axes==std::vector<int>{334}&&c.target==std::vector<float>{1};}
        REQUIRE(blocked&&queued&&stopped&&!started&&!enabled&&mock.table[334]==0);
    }},
    {"V21 high water bounds XYZCB prefill; batches are at most eight moves",[]{
        Fixture f;AxisParams params;std::vector<PathPoint> path;for(int i=0;i<120;++i)path.push_back({float(i)*0.5f,0,0});
        mock.accountBuffered=true;REQUIRE(f.controller.prepareMergePath(path,params));bool finished=false;
        REQUIRE(f.controller.pumpMergeBuffer(finished)&&!finished&&mock.buffered==48);
        int batch=0,maximum=0;{std::lock_guard<std::mutex> lock(mock.mutex);for(const auto& c:mock.calls) {
            if(c.name=="ZAux_Direct_GetMovesBuffered"){maximum=(std::max)(maximum,batch);batch=0;}
            if(c.name=="ZAux_Direct_MoveAbsSp")++batch;
        }}maximum=(std::max)(maximum,batch);REQUIRE(maximum==8);
        const int before=mock.callCount;REQUIRE(f.controller.pumpMergeBuffer(finished)&&mock.buffered==48&&mock.callCount==before+2);
        {std::lock_guard<std::mutex> lock(mock.mutex);mock.buffered=16;}
        REQUIRE(f.controller.pumpMergeBuffer(finished)&&mock.buffered==48);
    }},
    {"V19 stop cancels subsequent refill batches, no old producer restarts",[]{
        Fixture f;AxisParams params;std::vector<PathPoint> path;for(int i=0;i<120;++i)path.push_back({float(i)*0.5f,0,0});
        REQUIRE(f.controller.prepareMergePath(path,params));mock.armBlock("ZAux_Direct_MoveAbsSp");
        auto pump=std::async(std::launch::async,[&]{bool finished;return f.controller.pumpMergeBuffer(finished);});
        const bool blocked=mock.waitBlocked();f.controller.requestStopAllAxes(false);mock.releaseBlock();(void)pump.get();
        float d,m;bool idle;f.controller.getAxisPositionPair(AxisX,d,m,idle);int moves=0;
        {std::lock_guard<std::mutex> lock(mock.mutex);for(const auto& c:mock.calls)moves+=c.name=="ZAux_Direct_MoveAbsSp";}
        bool finished=false;const bool drained=f.controller.pumpMergeBuffer(finished);
        REQUIRE(blocked&&drained&&finished&&moves<=8);
    }},
    {"V19 table and UI diagnostic requests leave different scheduling counters",[]{
        Fixture f;float d,m;bool idle;REQUIRE(f.controller.readAxisUiPosition(AxisX,d,m,idle));
        ring(0,2);QVector<SamplePoint> out;bool overflow=false;REQUIRE(f.controller.hwTableSampleReadOnce(out,overflow));
        auto st=f.controller.ioStatistics();REQUIRE(st.classes[4].completed>0&&st.classes[2].completed>0);
        const int before=mock.callCount;for(int i=0;i<50;++i)(void)f.controller.ioStatistics();REQUIRE(mock.callCount==before);
    }},
    {"V19 metadata explicitly distinguishes paired valid and paired empty",[]{
        Fixture f;LaserSensor laser;HardwareSyncStrategy s(&laser,&f.controller);P0SyncTestAccess::arm(s);P0SyncTestAccess::poses(s,2);
        P0SyncTestAccess::receive(s,100,true);P0SyncTestAccess::receive(s,101);P0SyncTestAccess::align(s);
        auto st=s.getAlignStats();auto rows=s.getProfileRecords();
        REQUIRE(st.validAlignedProfiles==1&&st.emptyAlignedProfiles==1&&st.alignedProfiles==2&&rows.size()==2);
        REQUIRE(rows[0].pairing==syncdiag::PairingState::Aligned&&rows[0].validPoints==0&&rows[1].validPoints==1);
    }},
    {"V19 metadata contains dropped duplicates rather than calling them empty",[]{
        Fixture f;LaserSensor laser;HardwareSyncStrategy s(&laser,&f.controller);P0SyncTestAccess::arm(s);P0SyncTestAccess::poses(s,1);
        P0SyncTestAccess::receive(s,100);P0SyncTestAccess::receive(s,100,true);P0SyncTestAccess::align(s);auto rows=s.getProfileRecords();
        REQUIRE(rows.size()==2&&rows[0].pairing==syncdiag::PairingState::Aligned&&rows[1].pairing==syncdiag::PairingState::Dropped&&s.getAlignStats().emptyAlignedProfiles==0);
    }},
    {"V19 metadata reset does not retain prior capture",[]{
        Fixture f;LaserSensor laser;HardwareSyncStrategy s(&laser,&f.controller);P0SyncTestAccess::arm(s);P0SyncTestAccess::poses(s,1);
        P0SyncTestAccess::receive(s,100);P0SyncTestAccess::align(s);s.stop();s.clearData();
        REQUIRE(s.getProfileRecords().empty()&&s.getAlignStats().validAlignedProfiles==0&&s.getAlignStats().emptyAlignedProfiles==0);
    }},
    {"V32.13 C/B home-to-zero commits five-axis session bit only after verified motion",[]{
        Fixture f; AxisParams motion;
        REQUIRE(f.controller.homeRotaryAxisToZero(AxisC,motion));
        REQUIRE(f.controller.isAxisHomed(AxisC));
        REQUIRE(static_cast<int>(mock.table[357])==31);
        const auto move=lastCall("ZAux_Direct_Single_MoveAbs");
        REQUIRE(move.axes==std::vector<int>{AxisC});
        REQUIRE(nearValue(move.target[0],-10.0)); // machine 0deg -> raw controller -offset
        bool clearBeforeMove=false, setAfterMove=false, sawMove=false;
        {std::lock_guard<std::mutex> lock(mock.mutex); for(const auto& call:mock.calls){
            if(call.name=="ZAux_Direct_SetTable" && call.axes==std::vector<int>{357}
               && call.target==std::vector<float>{23.0f} && !sawMove) clearBeforeMove=true;
            if(call.name=="ZAux_Direct_Single_MoveAbs" && call.axes==std::vector<int>{AxisC}) sawMove=true;
            if(sawMove && call.name=="ZAux_Direct_SetTable" && call.axes==std::vector<int>{357}
               && call.target==std::vector<float>{31.0f}) setAfterMove=true;
        }}
        REQUIRE(clearBeforeMove&&sawMove&&setAfterMove);
    }},
    {"V32.13 failed C/B home leaves persisted Home bit cleared",[]{
        Fixture f; AxisParams motion;
        P0ControllerTestAccess::rotaryMachineLimits(f.controller,AxisC,1.0,1000.0); // excludes 0deg
        REQUIRE(!f.controller.homeRotaryAxisToZero(AxisC,motion));
        REQUIRE(!f.controller.isAxisHomed(AxisC));
        REQUIRE(static_cast<int>(mock.table[357])==23);
    }},
    {"V32.13 home failure keeps controller-resident X reference revoked",[]{
        Fixture f;mock.failHomeRestore=true;HomeParams hp;hp.mode=4;AxisParams motion;
        REQUIRE(!f.controller.homeAxis(AxisX,hp,motion));
        REQUIRE(static_cast<int>(mock.table[357])==30);
        REQUIRE(!f.controller.isLinearMposReferenced(AxisX));
    }},
    {"V27.6.16S XYZ home establishes grating MPOS machine zero only after HOME_STATUS success",[]{
        Fixture f;HomeParams hp;hp.mode=4;AxisParams motion;
        REQUIRE(f.controller.homeAxis(AxisX,hp,motion));
        REQUIRE(std::abs(mock.dpos[AxisX])<1.0e-6f);
        REQUIRE(std::abs(mock.mpos[AxisX])<1.0e-6f);
        REQUIRE(f.controller.isLinearMposReferenced(AxisX));
        bool sawHomeStatus=false,sawSetMpos=false;
        for(const auto& call:mock.calls){
            sawHomeStatus = sawHomeStatus || call.name=="ZAux_Direct_GetHomeStatus";
            sawSetMpos = sawSetMpos || call.name=="ZAux_Direct_SetMpos";
        }
        REQUIRE(sawHomeStatus&&sawSetMpos);
    }},
    {"V27.6.27 SetMpos generation changes only on home and encoder stays separately readable",[]{
        Fixture f; HomeParams hp; hp.mode=4; AxisParams motion;
        REQUIRE(f.controller.linearMposSetGeneration(AxisZ)==0);
        mock.encoder[AxisZ]=12.345f; float encoder=0.0f;
        REQUIRE(f.controller.readAxisEncoder(AxisZ,encoder));
        REQUIRE(std::abs(encoder-12.345f)<1.0e-5f);
        REQUIRE(f.controller.homeAxis(AxisZ,hp,motion));
        REQUIRE(f.controller.linearMposSetGeneration(AxisZ)==1);
        REQUIRE(f.controller.moveAbs(AxisZ,4.5f,&motion));
        REQUIRE(f.controller.linearMposSetGeneration(AxisZ)==1);
    }},
    {"V21 STOP during prepare cannot launder XYZCB path into new generation",[]{
        Fixture f;AxisParams params;mock.armBlock("ZAux_Direct_SetAccel");
        auto prep=std::async(std::launch::async,[&]{return f.controller.prepareMergePath({{1,2,3},{2,3,4}},params);});
        const bool blocked=mock.waitBlocked();f.controller.requestStopAxis(AxisC,false);mock.releaseBlock();
        const bool prepared=prep.get();float d,m;bool idle;f.controller.getAxisPositionPair(AxisX,d,m,idle);
        bool finished=false;const bool drained=f.controller.pumpMergeBuffer(finished);bool moved=false;
        {std::lock_guard<std::mutex> lock(mock.mutex);for(const auto& c:mock.calls)moved|=c.name=="ZAux_Direct_MoveAbsSp";}
        REQUIRE(blocked&&prepared&&drained&&finished&&!moved);
    }}

,
    {"V21 five-axis SP packet retains axis order and rotary native offsets",[]{
        Fixture f;AxisParams m;PathPoint p;p.x=1;p.y=2;p.z=3;p.c=4;p.b=5;p.feed=1;p.axisMask=31;
        REQUIRE(f.controller.prepareMergePath({p},m));bool done=false;REQUIRE(f.controller.pumpMergeBuffer(done)&&done);
        auto call=lastCall("ZAux_Direct_MoveAbsSp");REQUIRE(call.axes==std::vector<int>({1,2,3,4,5}));REQUIRE(call.target==std::vector<float>({1,2,3,-6,25}));
    }},
    {"V21 legacy XYZ aggregate holds nonzero C/B rather than zeroing",[]{
        Fixture f;mock.dpos[4]=mock.mpos[4]=20;mock.dpos[5]=mock.mpos[5]=30;
        AxisParams m;REQUIRE(f.controller.moveLineAbs({1,2,3},m));auto c=lastCall("ZAux_Direct_MoveAbsSp");REQUIRE(c.target[3]==20&&c.target[4]==30);
    }},
    {"V21 queued feed uses SP speeds and never changes global SPEED per point",[]{
        Fixture f;AxisParams m;REQUIRE(f.controller.prepareMergePath({{1,0,0,1},{2,0,0,2}},m));
        int baseline=mock.callCount;bool done=false;REQUIRE(f.controller.pumpMergeBuffer(done));bool sawForce=false,sawGlobal=false;
        {std::lock_guard<std::mutex> l(mock.mutex);for(std::size_t i=baseline;i<mock.calls.size();++i){sawForce|=mock.calls[i].name=="ZAux_Direct_SetForceSpeed";sawGlobal|=mock.calls[i].name=="ZAux_Direct_SetSpeed";}}
        REQUIRE(sawForce&&!sawGlobal);
    }},
    {"G93 continuous collinear G05 sends nonzero SP knot speeds",[]{
        Fixture f;AxisParams motion;motion.accel=25;motion.decel=25;
        std::vector<PathPoint> points;
        for(int i=1;i<=20;++i){
            PathPoint p;p.x=float(i);p.axisMask=31;p.continuous=true;
            p.feedMode=mls::runtime::FeedMode::InverseMinute;
            p.feed=60.0/((i==1||i==20)?0.305526315789:0.210526315789);
            points.push_back(p);
        }
        REQUIRE(f.controller.prepareMergePath(points,motion));
        bool done=false;
        for(int i=0;i<30&&!done;++i)REQUIRE(f.controller.pumpMergeBuffer(done));
        REQUIRE(done);
        int nonzeroKnots=0,stops=0,moves=0;
        {std::lock_guard<std::mutex> lk(mock.mutex);
         for(const auto& c:mock.calls){
             if(c.name=="ZAux_Direct_MoveAbsSp")++moves;
             if(c.name=="ZAux_Direct_SetEndMoveSpeed"&&!c.target.empty()){
                 if(c.target[0]>4.0f)++nonzeroKnots;
                 else if(c.target[0]<1e-5f)++stops;
             }
         }
        }
        REQUIRE(moves==20 && nonzeroKnots>=18 && stops>=1);
    }},
    {"G93 live-start rotary MPOS residual preserves robust first stop and later SP blend",[]{
        Fixture f; AxisParams motion; motion.accel=25; motion.decel=25;
        // Simulate a normal post-G00 settle that is well inside the existing
        // 0.05mm/0.05deg completion tolerance. V4 would reject the first
        // 0.3054s G93 knot because this start is not mathematically collinear.
        {std::lock_guard<std::mutex> lk(mock.mutex);
         mock.dpos[AxisX]=0.005f;
         mock.mpos[AxisC]=-9.996f; // machine C = raw MPOS + 10 = +0.004
         mock.mpos[AxisB]=19.998f; // machine B = raw MPOS - 20 = -0.002
        }
        mls::runtime::Limits limits{};
        std::vector<mls::runtime::Waypoint> input;
        for(int i=1;i<=20;++i)
            input.push_back({{double(i),0,0,0,0},300,
                mls::runtime::FeedMode::InverseMinute,true});
        std::vector<double> seconds;std::string problem;
        REQUIRE(mls::runtime::retimeG93Polyline({},input,limits,25,25,seconds,problem));
        std::vector<PathPoint> physical;
        for(int i=1;i<=20;++i){
            PathPoint p;p.x=float(i);p.axisMask=31;
            p.continuous=i>1;
            p.feedMode=mls::runtime::FeedMode::InverseMinute;
            p.feed=60.0/seconds[std::size_t(i-1)];
            physical.push_back(p);
        }
        QString err;
        REQUIRE(f.controller.prepareMergePath(physical,motion,&err));
        bool done=false;
        for(int i=0;i<30&&!done;++i)REQUIRE(f.controller.pumpMergeBuffer(done,&err));
        REQUIRE(done);
        int ends=0,nonzero=0,firstZero=0;
        {std::lock_guard<std::mutex> lk(mock.mutex);
         for(const auto& call:mock.calls)
             if(call.name=="ZAux_Direct_SetEndMoveSpeed"&&!call.target.empty()){
                 if(ends==0 && call.target[0]<1.e-6f)++firstZero;
                 if(ends>1 && call.target[0]>4.0f)++nonzero;
                 ++ends;
             }
        }
        REQUIRE(ends==20 && firstZero==1 && nonzero>=15);
    }},
    {"V21 motion parameters cannot overwrite commissioned UNITS",[]{
        Fixture f;AxisParams m;m.units=4000;const int n=mock.callCount;REQUIRE(f.controller.writeMotionParams(AxisC,m));
        bool wroteUnits=false;{std::lock_guard<std::mutex> l(mock.mutex);for(std::size_t i=n;i<mock.calls.size();++i)wroteUnits|=mock.calls[i].name=="ZAux_Direct_SetUnits";}REQUIRE(!wroteUnits);
    }},
    {"V21 pending trajectory excludes manual motion and parameter changes",[]{
        Fixture f;AxisParams m;REQUIRE(f.controller.prepareMergePath({{1,2,3},{2,3,4}},m));const int before=mock.callCount;
        REQUIRE(!f.controller.writeMotionParams(AxisC,m));REQUIRE(!f.controller.moveAbs(AxisX,1));REQUIRE(before==mock.callCount);
    }},
    {"V21 C/B stop cancels all five axes of buffered trajectory",[]{
        Fixture f;AxisParams m;REQUIRE(f.controller.prepareMergePath({{1,2,3}},m));REQUIRE(f.controller.stopAxis(AxisB,false));
        auto c=lastCall("ZAux_Direct_CancelAxisList");REQUIRE(c.axes==std::vector<int>({1,2,3,4,5})&&c.mode==2);
    }},
    {"V21 out-of-range path is rejected before any interpolation packet",[]{
        Fixture f;AxisParams m;REQUIRE(!f.controller.prepareMergePath({{1,2,3},{1000,0,0}},m));
        bool moved=false;{std::lock_guard<std::mutex> l(mock.mutex);for(const auto&c:mock.calls)moved|=c.name=="ZAux_Direct_MoveAbsSp";}REQUIRE(!moved);
    }},
    {"V21 unsupported interpolation setting refuses motion",[]{Fixture f;mock.failInterp=true;AxisParams m;REQUIRE(!f.controller.prepareMergePath({{1,0,0}},m));}},
    {"V21 SP transport failure cancels all axes and queued commands",[]{
        Fixture f;AxisParams m;REQUIRE(f.controller.prepareMergePath({{1,0,0}},m));mock.failSp=true;bool done=false;REQUIRE(!f.controller.pumpMergeBuffer(done));REQUIRE(lastCall("ZAux_Direct_CancelAxisList").mode==2);
    }},
    {"V21 pause/resume SDK failures propagate to caller",[]{Fixture f;mock.failPause=true;REQUIRE(!f.controller.pauseTrajectory5D());mock.failPause=false;REQUIRE(f.controller.pauseTrajectory5D());mock.failResume=true;REQUIRE(!f.controller.resumeTrajectory5D());}},
    {"V21 B-axis fault prevents five-axis motion",[]{Fixture f;mock.status[AxisB]=4194304;REQUIRE(!f.controller.checkMotionHealth5D());AxisParams m;REQUIRE(!f.controller.prepareMergePath({{1,0,0}},m));}},
    {"V32.13 single-OP14 full-MPOS read-only TABLE preflight performs no SetTable and returns execution authorization",[]{Fixture f;QString summary,err;bool authorized=false;const int before=countCalls("ZAux_Direct_SetTable");REQUIRE(f.controller.hwTableCommissioningPreflight(summary,&err,&authorized));REQUIRE(authorized);REQUIRE(countCalls("ZAux_Direct_SetTable")==before);REQUIRE(!summary.isEmpty());}},
    {"V32.13 read-only preflight accepts compatible auth0 header but keeps execution unauthorized",[]{Fixture f;{std::lock_guard<std::mutex> l(mock.mutex);mock.table[342]=0;}QString summary,err;bool authorized=true;const int before=countCalls("ZAux_Direct_SetTable");REQUIRE(f.controller.hwTableCommissioningPreflight(summary,&err,&authorized));REQUIRE(!authorized);REQUIRE(countCalls("ZAux_Direct_SetTable")==before);REQUIRE(summary.str().find("hardwareExecutionAuthorization=0")!=std::string::npos);}},
    {"V32.13 hardware start rejects auth0 before any TABLE write",[]{Fixture f;{std::lock_guard<std::mutex> l(mock.mutex);mock.table[342]=0;}const int before=countCalls("ZAux_Direct_SetTable");QString err;REQUIRE(!f.controller.hwTableSampleStart(&err));REQUIRE(countCalls("ZAux_Direct_SetTable")==before);REQUIRE(err.str().find("TABLE[342]=0")!=std::string::npos);}},
    {"V32.13 preflight rejects REGIST metadata 103 instead of 104",[]{Fixture f;{std::lock_guard<std::mutex> l(mock.mutex);mock.table[349]=103;}QString summary;REQUIRE(!f.controller.hwTableCommissioningPreflight(summary));REQUIRE(mock.table[334]==0);}},
    {"V32.13 preflight identifies the exact mismatched TABLE field",[]{Fixture f;{std::lock_guard<std::mutex> l(mock.mutex);mock.table[349]=103;}QString summary,err;const int before=countCalls("ZAux_Direct_SetTable");REQUIRE(!f.controller.hwTableCommissioningPreflight(summary,&err));const auto text=err.str();REQUIRE(text.find("V32.16")!=std::string::npos&&text.find("104")!=std::string::npos&&text.find("TABLE[330..351]")!=std::string::npos);REQUIRE(countCalls("ZAux_Direct_SetTable")==before);}},
    {"V32.13 five-axis full-MPOS read-only preflight is accepted",[]{Fixture f;QString summary,err;REQUIRE(f.controller.hwTableCommissioningPreflight(summary,&err));REQUIRE(summary.str().find("FullMPOSLatched")!=std::string::npos&&summary.str().find("coldPrimeCapability=1")!=std::string::npos);}},
    {"V32.13 rejects legacy mixed-DPOS header because hardware TABLE must be full MPOS",[]{Fixture f;{std::lock_guard<std::mutex> l(mock.mutex);mock.table[343]=24;mock.table[344]=1;mock.table[345]=24;mock.table[346]=7;}QString summary,err;REQUIRE(!f.controller.hwTableCommissioningPreflight(summary,&err));REQUIRE(err.str().find("V32.16")!=std::string::npos);}},
    {"V32.13 cold-prime mode uses same OP14 before formal session and does not publish poses",[]{Fixture f;const auto w=mock.table[331];const auto p=mock.table[336];REQUIRE(f.controller.hwSensorColdPrimeStart());REQUIRE(mock.table[334]==3&&mock.table[331]==w&&mock.table[336]==p);REQUIRE(f.controller.hwSensorColdPrimeStop());REQUIRE(mock.table[334]==0);}},
    {"V32.13 cold-prime runtime counter is readable",[]{Fixture f;{std::lock_guard<std::mutex> l(mock.mutex);mock.table[352]=9;}qint64 n=-1;REQUIRE(f.controller.hwSensorColdPrimeCount(n)&&n==9);}},
    {"V32 old three-axis TABLE firmware fails before PWM",[]{Fixture f;mock.table[338]=3;REQUIRE(!f.controller.hwTableSampleStart());REQUIRE(mock.table[334]==0);}},
    {"V32 one-shot probe stays armed until PC continues",[]{Fixture f;REQUIRE(f.controller.hwTableSampleStart());REQUIRE(mock.table[336]==1&&mock.table[334]==2);REQUIRE(f.controller.hwTableContinueAfterProbe());REQUIRE(mock.table[334]==1);f.controller.hwTableSampleStop();}},
    {"V32 ambiguous two-probe startup cannot continue",[]{Fixture f;REQUIRE(f.controller.hwTableSampleStart());mock.table[336]=2;REQUIRE(!f.controller.hwTableContinueAfterProbe());REQUIRE(mock.table[334]==2);f.controller.hwTableSampleStop();}},
    {"V32 PC-owned read cursor mismatch fails closed",[]{Fixture f;REQUIRE(f.controller.hwTableSampleStart());
        {std::lock_guard<std::mutex> l(mock.mutex);mock.table[332]=1;}
        QVector<SamplePoint> out;bool overflow=false;QString err;
        REQUIRE(!f.controller.hwTableSampleReadOnce(out,overflow,&err));
        REQUIRE(f.controller.hwTableReadFaulted() && out.isEmpty());
        f.controller.hwTableSampleStop();}},
    {"V32.13 runtime PWM frequency request is ACKed by controller protocol",[]{
        Fixture f;QString err;REQUIRE(f.controller.setHardwareTriggerFrequencyHz(333,&err));
        int requested=0,applied=0;REQUIRE(f.controller.readHardwareTriggerFrequencyHz(requested,applied,&err));
        REQUIRE(requested==333&&applied==333&&static_cast<int>(mock.table[355])==333&&static_cast<int>(mock.table[356])==333);
        REQUIRE(mock.table[359]==2 && mock.table[360]==2);
        REQUIRE(!f.controller.setHardwareTriggerFrequencyHz(2001,&err));
    }},
    {"V32.13 rejects mismatched Host PWM Hz even when controller request equals ACK",[]{
        Fixture f;QString err;
        {std::lock_guard<std::mutex> l(mock.mutex);mock.table[355]=333;mock.table[356]=333;mock.table[359]=7;mock.table[360]=7;}
        REQUIRE(!f.controller.hwTableSampleStart(&err));
        REQUIRE(mock.table[334]==0);
        REQUIRE(f.controller.setHardwareTriggerFrequencyHz(333,&err));
        REQUIRE(f.controller.hwTableSampleStart(&err));
        REQUIRE(f.controller.hwTableSampleStop(&err));
    }},
    {"V32.13 requires BASIC actual Idle state to modify frequency",[]{
        Fixture f;QString err;
        {std::lock_guard<std::mutex> l(mock.mutex);mock.table[358]=4;}
        REQUIRE(!f.controller.setHardwareTriggerFrequencyHz(333,&err));
        {std::lock_guard<std::mutex> l(mock.mutex);mock.table[358]=0;}
        REQUIRE(f.controller.setHardwareTriggerFrequencyHz(333,&err));
        REQUIRE(f.controller.hwTableWaitIdleReady(100,&err));
    }},
    {"V32.13 refuses to enable RUN if a foreign writer changes frequency during Probe",[]{
        Fixture f;QString err;
        REQUIRE(f.controller.setHardwareTriggerFrequencyHz(200,&err));
        REQUIRE(f.controller.hwTableSampleStart(&err));
        {std::lock_guard<std::mutex> l(mock.mutex);mock.table[355]=300;}
        REQUIRE(!f.controller.hwTableContinueAfterProbe(&err));
        REQUIRE(mock.table[334]==2);
        REQUIRE(f.controller.hwTableSampleStop(&err));
    }},
    {"V32.13 missing config ACK cannot claim success",[]{
        Fixture f;QString err;
        {std::lock_guard<std::mutex> l(mock.mutex);mock.suppressPwmConfigAck=true;}
        REQUIRE(!f.controller.setHardwareTriggerFrequencyHz(333,&err));
        REQUIRE(mock.table[356]==200 && mock.table[359]==2 && mock.table[360]==1);
        REQUIRE(!f.controller.hwTableSampleStart(&err));
    }},
    {"V32.13 frequency ACK waiting leaves emergency STOP operable",[]{
        Fixture f;QString configError,stopError;
        {std::lock_guard<std::mutex> l(mock.mutex);mock.suppressPwmConfigAck=true;}
        auto future=std::async(std::launch::async,[&]{return f.controller.setHardwareTriggerFrequencyHz(333,&configError);});
        bool pending=false;
        for(int i=0;i<150;++i){
            {std::lock_guard<std::mutex> l(mock.mutex);pending=mock.table[359]==2;}
            if(pending)break;
            std::this_thread::sleep_for(2ms);
        }
        const auto t0=std::chrono::steady_clock::now();
        const bool stopped=f.controller.hwTableSampleStop(&stopError);
        const auto ms=std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now()-t0).count();
        const bool acked=future.get();
        REQUIRE(pending && stopped && !acked && ms<500);
    }},
    {"V32.13 config-error recovery does not assert hardware RUN",[]{
        Fixture f;QString err;
        {std::lock_guard<std::mutex> l(mock.mutex);mock.table[355]=5000;mock.table[359]=1;mock.table[361]=10;mock.table[358]=5;}
        REQUIRE(f.controller.setHardwareTriggerFrequencyHz(144,&err));
        REQUIRE(mock.table[356]==144 && mock.table[360]==2 && mock.table[361]==0);
        REQUIRE(f.controller.hwTableWaitIdleReady(100,&err));
    }},
    {"V32.13 five-axis Home mask survives PC disconnect reconnect while controller stays powered",[]{
        Fixture f;HomeParams hp;hp.mode=4;AxisParams motion;
        REQUIRE(f.controller.homeAxis(AxisX,hp,motion));REQUIRE(f.controller.homeAxis(AxisY,hp,motion));REQUIRE(f.controller.homeAxis(AxisZ,hp,motion));
        REQUIRE(f.controller.homeRotaryAxisToZero(AxisC,motion));REQUIRE(f.controller.homeRotaryAxisToZero(AxisB,motion));
        REQUIRE(static_cast<int>(mock.table[357])==31);
        REQUIRE(f.controller.areAllAxesHomed());
        f.controller.disconnect();REQUIRE(!f.controller.areAllAxesHomed());
        REQUIRE(f.controller.connectTcp("127.0.0.1"));
        REQUIRE(f.controller.areLinearMposReferenced());
        REQUIRE(!f.controller.isAxisHomed(AxisC)&&!f.controller.isAxisHomed(AxisB));
        P0ControllerTestAccess::rotaryAbsoluteReady(f.controller);
        REQUIRE(f.controller.areAllAxesHomed());
    }},
    {"V32.13 stale XYZ bits are downgraded while C/B Home-session bits are preserved",[]{
        Fixture f;f.controller.disconnect();
        {std::lock_guard<std::mutex> l(mock.mutex);mock.table[357]=31;mock.homeStatus.fill(0);mock.homeStatus[AxisX]=1;}
        REQUIRE(f.controller.connectTcp("127.0.0.1"));
        REQUIRE(f.controller.isLinearMposReferenced(AxisX));
        REQUIRE(!f.controller.isLinearMposReferenced(AxisY)&&!f.controller.isLinearMposReferenced(AxisZ));
        REQUIRE(static_cast<int>(mock.table[357])==25);
        P0ControllerTestAccess::rotaryAbsoluteReady(f.controller);
        REQUIRE(f.controller.isAxisHomed(AxisC)&&f.controller.isAxisHomed(AxisB));
    }},
    {"V32.13 controller BASIC restart power-session mask zero forces five-axis re-home",[]{
        Fixture f;f.controller.disconnect();
        {std::lock_guard<std::mutex> l(mock.mutex);mock.table[357]=0;mock.homeStatus[AxisX]=mock.homeStatus[AxisY]=mock.homeStatus[AxisZ]=1;}
        REQUIRE(f.controller.connectTcp("127.0.0.1"));
        P0ControllerTestAccess::rotaryAbsoluteReady(f.controller);
        REQUIRE(!f.controller.areAllAxesHomed());
    }},
    {"V21 DPOS zero is refused without writing the controller",[]{Fixture f;const int before=mock.callCount;REQUIRE(!f.controller.zeroAllDpos());REQUIRE(before==mock.callCount);}},
    {"V21 five-axis scanner refuses unset calibration",[]{Fixture f;LaserSensor laser;HardwareSyncStrategy s(&laser,&f.controller);REQUIRE(!s.start());}},
    {"V27.6.38 XYZ speed-only full parameter write preserves DATUM",[]{
        Fixture f; AxisParams p; p.units=100.0f; p.speed=2.5f; p.accel=12.0f; p.decel=12.0f;
        REQUIRE(f.controller.areLinearMposReferenced());
        const int unitsBefore=countCalls("ZAux_Direct_SetUnits");
        QString err; REQUIRE(f.controller.writeAxisParams(AxisX,p,&err));
        REQUIRE(f.controller.areLinearMposReferenced());
        REQUIRE(countCalls("ZAux_Direct_SetUnits")==unitsBefore);
    }},
    {"V27.6.38 BC speed-only full parameter write preserves absolute-ready",[]{
        Fixture f; AxisParams p; p.units=100.0f; p.softLimitNeg=-1000.0f; p.softLimitPos=1000.0f;
        p.speed=3.0f; p.accel=15.0f; p.decel=15.0f;
        REQUIRE(f.controller.isRotaryAbsoluteReady(AxisC));
        REQUIRE(f.controller.isRotaryServoEnabled(AxisC));
        const int unitsBefore=countCalls("ZAux_Direct_SetUnits");
        QString err; REQUIRE(f.controller.writeAxisParams(AxisC,p,&err));
        REQUIRE(f.controller.isRotaryAbsoluteReady(AxisC));
        REQUIRE(f.controller.isRotaryServoEnabled(AxisC));
        REQUIRE(f.controller.isRotaryAbsoluteReady(AxisB));
        REQUIRE(f.controller.isRotaryServoEnabled(AxisB));
        REQUIRE(countCalls("ZAux_Direct_SetUnits")==unitsBefore);
    }},
    {"V27.6.38 XYZ UNITS change still invalidates DATUM",[]{
        Fixture f; AxisParams p; p.units=200.0f;
        REQUIRE(f.controller.areLinearMposReferenced());
        QString err; REQUIRE(f.controller.writeAxisParams(AxisX,p,&err));
        REQUIRE(!f.controller.areLinearMposReferenced());
        REQUIRE(countCalls("ZAux_Direct_SetUnits")>0);
    }}

});}
