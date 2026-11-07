#include "core/ZmcController.h"
#include "utils/LogManager.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>
#include <thread>

bool ZmcController::compensationConfigurationIdle(QString* err) const
{
    if (!m_handle) {if(err)*err=QStringLiteral("请先连接控制器，再加载/启用 XYZ 补偿");return false;}
    if (m_hwSampling || m_mergeIndex<m_mergePath.size() || m_activeBlockingCalls.load()>0) {
        if(err)*err=QStringLiteral("运动/采样/阻塞任务运行中，禁止改变 XYZ 补偿");return false;
    }
    for (int axis=AxisX;axis<=AxisB;++axis) {
        if(isAxisBusy(axis)){if(err)*err=QStringLiteral("轴任务运行中，禁止改变 XYZ 补偿");return false;}
        int idle=0,buffered=0;
        int32 r=ZAux_Direct_GetIfIdle(m_handle,axis,&idle);
        if(r==ERR_OK)r=ZAux_Direct_GetMovesBuffered(m_handle,axis,&buffered);
        if(r!=ERR_OK){if(err)*err=errorTextFromCode(r,"GetIdle/Buffered(compensation)");return false;}
        if(idle==0 || buffered!=0){if(err)*err=QStringLiteral("所有轴必须空闲且缓冲清空后才能改变 XYZ 补偿");return false;}
    }
    return true;
}

bool ZmcController::installAxisCompensation(const mls::axiscomp::Tables& tables,
                                            const mls::axiscomp::Options& options,QString* err)
{
    if(!m_io.isWorkerThread())return runIo(IoPriority::Control,"axiscomp.install",[&]{
        return installAxisCompensation(tables,options,err);
    },err);
    p0::IoLock lock(m_ioMutex);
    if(!compensationConfigurationIdle(err))return false;
    std::string detail;
    if(!mls::axiscomp::validateTables(tables,options,detail)){
        if(err)*err=QString::fromStdString(detail);return false;
    }
    // Validate completely before replacing the previously working set.
    m_axisCompTables=tables;m_axisCompOptions=options;m_axisCompLoaded=true;
    m_axisCompEnabled.store(false);invalidateLinearTarget();
    LOG_INFO(LogCategory::Controller,QStringLiteral("XYZ compensation tables installed; runtime remains OFF until explicit enable"));
    return true;
}

bool ZmcController::setAxisCompensationEnabled(bool enabled,QString* err)
{
    if(!m_io.isWorkerThread())return runIo(IoPriority::Control,"axiscomp.enable",[&]{
        return setAxisCompensationEnabled(enabled,err);
    },err);
    p0::IoLock lock(m_ioMutex);
    if(!compensationConfigurationIdle(err))return false;
    if(enabled && (!m_axisCompLoaded || !areLinearMposReferenced())){
        if(err)*err=QStringLiteral("启用前必须成功加载 XYZ 三轴表，并在本连接中完成 XYZ 正规 DATUM 回零");return false;
    }
    if(enabled){std::string detail;if(!mls::axiscomp::validateTables(m_axisCompTables,m_axisCompOptions,detail)){
        if(err)*err=QString::fromStdString(detail);return false;
    }}
    invalidateLinearTarget();m_axisCompEnabled.store(enabled);
    LOG_INFO(LogCategory::Controller,enabled?QStringLiteral("XYZ physical-target compensation ENABLED; feedback/encoder/homing remain RAW"):
                                          QStringLiteral("XYZ physical-target compensation DISABLED"));
    return true;
}

QString ZmcController::axisCompensationStatus(QString* err) const
{
    if(!isConnected())return QStringLiteral("XYZ 补偿：未连接 / 禁用；连接后加载数据，正规回零后人工启用");
    if(!m_io.isWorkerThread()){
        QString text;
        if(!runIo(IoPriority::Diagnostic,"axiscomp.status",[&]{text=axisCompensationStatus(err);return true;},err))
            return QStringLiteral("XYZ 补偿状态读取失败");
        return text;
    }
    p0::IoLock lock(m_ioMutex);
    if(!m_axisCompLoaded)return QStringLiteral("XYZ 补偿：未加载数据 / 禁用");
    std::ostringstream out;
    out<<"XYZ runtime: "<<(m_axisCompEnabled.load()?"ON":"OFF")
       <<"; point settle="<<m_axisCompOptions.pointSettleMs<<"ms; continuous="
       <<(m_axisCompOptions.allowContinuous?"acknowledged":"blocked")<<"; Z legacy history="
       <<(m_axisCompOptions.allowLegacyPointTableOnPlannedMotion?"acknowledged":"blocked");
    for(int j=0;j<3;++j){
        const auto& t=m_axisCompTables[j];
        out<<"\n"<<"XYZ"[j]<<": "<<(t.method==mls::axiscomp::SamplingMethod::MonotonicSweep?"MONOTONIC_SWEEP":"POINT_TO_POINT_PRELOAD")
           <<"; nodes="<<t.directions[0].size()<<"/"<<t.directions[1].size();
        for(int d=0;d<2;++d)out<<"; "<<(d==0?"+":"-")<<" MPOS=["<<t.directions[d].front().desiredMpos<<","<<t.directions[d].back().desiredMpos<<"]mm";
        out<<"; SHA256="<<t.sourceId;
    }
    return QString::fromStdString(out.str());
}

void ZmcController::invalidateLinearTarget(int axis)
{
    if(axis<0){for(auto& c:m_axisCompTargets)c.valid=false;return;}
    if(axis>=AxisX && axis<=AxisZ)m_axisCompTargets[axis-AxisX].valid=false;
}
void ZmcController::rememberLinearTarget(int axis,double nominal,double command)
{
    if(axis<AxisX || axis>AxisZ)return;
    m_axisCompTargets[axis-AxisX]={true,nominal,command};
    LOG_INFO(LogCategory::Controller,
             QStringLiteral("AXIS_COMP axis=%1 desiredMPOS=%2 commandedDPOS=%3 correction=%4 mm")
             .arg(axis).arg(nominal,0,'f',9).arg(command,0,'f',9).arg(command-nominal,0,'f',9));
}
double ZmcController::compensatedNominalStart(int axis,double dpos,double mpos) const
{
    if(axis<AxisX || axis>AxisZ || !m_axisCompEnabled.load())return dpos;
    const auto& c=m_axisCompTargets[axis-AxisX];
    // Repeating an accepted absolute target must not swap direction/chase its
    // residual MPOS error. Unexpected demand/large external movement invalidates
    // the cache logically and starts again from actual feedback.
    if(c.valid && std::abs(dpos-c.command)<=0.00020 &&
       std::abs(mpos-c.nominal)<=m_axisCompOptions.maxCorrectionMm)return c.nominal;
    return mpos;
}

bool ZmcController::axisCompensationSafetyEnvelopeMm(double& envelopeMm, QString* err) const
{
    if(!m_io.isWorkerThread()) return runIo(IoPriority::Control, "axisComp.safetyEnvelope", [&]{
        return axisCompensationSafetyEnvelopeMm(envelopeMm,err);
    },err,eci::EciIoScheduler::None);
    p0::IoLock lock(m_ioMutex);
    envelopeMm=0.0;
    if(!m_axisCompEnabled.load()) return true;
    if(!m_axisCompLoaded){if(err)*err=QStringLiteral("XYZ compensation tables are not loaded");return false;}
    for(int j=0;j<3;++j){
        double axisMax=0.0;
        for(const auto& branch:m_axisCompTables[j].directions) for(const auto& node:branch){
            if(!std::isfinite(node.desiredMpos)||!std::isfinite(node.requiredDpos)){
                if(err)*err=QStringLiteral("XYZ compensation table contains NaN/Inf");return false;
            }
            axisMax=(std::max)(axisMax,std::abs(node.requiredDpos-node.desiredMpos));
        }
        envelopeMm+=axisMax;
    }
    if(!std::isfinite(envelopeMm)){if(err)*err=QStringLiteral("XYZ compensation safety envelope overflow");return false;}
    return true;
}

bool ZmcController::effectiveIndependentAxisParams5D(
    const std::array<AxisParams, Constants::kAxisParamArraySize>& requested,
    double speedScale,
    std::array<AxisParams, Constants::kAxisParamArraySize>& effective,
    QString* err) const
{
    if(!m_io.isWorkerThread()) return runIo(IoPriority::Control, "axisComp.independentSafetyParams", [&]{
        return effectiveIndependentAxisParams5D(requested,speedScale,effective,err);
    },err,eci::EciIoScheduler::None);
    p0::IoLock lock(m_ioMutex);
    if(!std::isfinite(speedScale)||speedScale<=0.0||speedScale>1.0){
        if(err)*err=QStringLiteral("Independent move speed scale is invalid");return false;
    }
    effective=requested;
    if(!m_axisCompEnabled.load()) return true;
    if(!m_axisCompLoaded){if(err)*err=QStringLiteral("XYZ compensation tables are not loaded");return false;}
    for(int axis=AxisX;axis<=AxisZ;++axis){
        const double precision=m_axisCompTables[axis-AxisX].precisionSpeedMmS;
        if(!(precision>0.0)) continue;
        const double requestedEffective=static_cast<double>(requested[axis].speed)*speedScale;
        if(!std::isfinite(requestedEffective)||requestedEffective<=0.0){
            if(err)*err=QStringLiteral("Independent XYZ speed is invalid");return false;
        }
        const double capped=(std::min)(requestedEffective,precision);
        effective[axis].speed=static_cast<float>(capped/speedScale);
    }
    return true;
}

bool ZmcController::readMotionStartPose5D(JointPose5D& nominal,QString* err)
{
    if(!m_io.isWorkerThread())return runIo(IoPriority::Control,"axiscomp.motionStart",[&]{
        return readMotionStartPose5D(nominal,err);
    },err);
    p0::IoLock lock(m_ioMutex);
    JointState5D state;if(!readJointState5D(state,err))return false;
    if(!state.allIdle){if(err)*err=QStringLiteral("规划起点必须从所有轴 IDLE 状态读取");return false;}
    nominal=state.mpos;
    nominal.x=compensatedNominalStart(AxisX,state.dpos.x,state.mpos.x);
    nominal.y=compensatedNominalStart(AxisY,state.dpos.y,state.mpos.y);
    nominal.z=compensatedNominalStart(AxisZ,state.dpos.z,state.mpos.z);
    return true;
}

bool ZmcController::mapLinearTarget(int axis,double desired,double currentD,double currentM,
                                   double& command,QString* err,bool relativeFromFeedback) const
{
    if(axis<AxisX || axis>AxisZ){if(err)*err=QStringLiteral("XYZ 补偿收到非直线轴编号");return false;}
    if(!m_axisCompLoaded || !isLinearMposReferenced(axis)){
        if(err)*err=QStringLiteral("XYZ 补偿没有有效数据/本连接 DATUM 基准");return false;
    }
    // A measured XYZ component carried through a BC-only/Stage2 target is HOLD,
    // even when the previous nominal command differs by a settling residual.
    if(std::isfinite(desired) && std::abs(desired-currentM)<=m_axisCompOptions.holdToleranceMm){
        command=currentD;return true;
    }
    const double nominalStart=relativeFromFeedback?currentM:compensatedNominalStart(axis,currentD,currentM);
    mls::axiscomp::PointResult r;std::string detail;
    if(!mls::axiscomp::mapPoint(m_axisCompTables[axis-AxisX],m_axisCompOptions,nominalStart,currentD,desired,r,detail)){
        if(err)*err=QString::fromStdString(detail);return false;
    }
    command=r.commandDpos;return true;
}

bool ZmcController::validateLinearCommand(int axis,double desired,double command,QString* err) const
{
    if(axis<AxisX || axis>AxisZ || !std::isfinite(desired)||!std::isfinite(command)){
        if(err)*err=QStringLiteral("非法 XYZ 目标或补偿溢出");return false;
    }
    float low=0,high=0;int32 r=ZAux_Direct_GetRsLimit(m_handle,axis,&low);
    if(r==ERR_OK)r=ZAux_Direct_GetFsLimit(m_handle,axis,&high);
    if(r!=ERR_OK){if(err)*err=errorTextFromCode(r,"GetLimits(axiscomp)");return false;}
    if(!std::isfinite(low)||!std::isfinite(high)||low>high || desired<low || desired>high || command<low || command>high){
        if(err)*err=QStringLiteral("轴%1 名义目标%2 / 补偿命令%3 超出控制器限位[%4,%5]；没有截断或外推")
            .arg(axis).arg(desired,0,'f',9).arg(command,0,'f',9).arg(low).arg(high);
        return false;
    }
    return true;
}

bool ZmcController::waitCompensationSettle(int milliseconds,int axis,QString* err)
{
    if(milliseconds<=0)return true;
    if(m_io.isWorkerThread()){if(err)*err=QStringLiteral("Settle wait must not occupy ECI I/O worker");return false;}
    const auto until=std::chrono::steady_clock::now()+std::chrono::milliseconds(milliseconds);
    while(std::chrono::steady_clock::now()<until){
        bool stopped=!m_connected.load();
        if(axis>=AxisX && axis<=AxisB)stopped=stopped||m_stopRequested[axis].load();
        else for(int a=AxisX;a<=AxisB;++a)stopped=stopped||m_stopRequested[a].load();
        if(stopped){if(err)*err=QStringLiteral("补偿定位的静置等待已被停止/断开取消");return false;}
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return true;
}

bool ZmcController::compensatedArc(const mls::runtime::Joint& end,const mls::runtime::Arc& arc,
                                   const AxisParams& motion,QString* err)
{
    // Called on the scheduler. Geometry is physical, then the existing complete
    // five-axis queue path maps/densifies/replans once. Never compensate I/J centers.
    JointPose5D q;if(!readMotionStartPose5D(q,err))return false;
    mls::runtime::Joint start{q.x,q.y,q.z,q.c,q.b};
    std::vector<mls::runtime::Joint> vertices;std::string detail;
    if(!mls::runtime::arcWaypoints(start,end,arc,vertices,detail)){
        if(err)*err=QString::fromStdString(detail);return false;
    }
    std::vector<PathPoint> path;path.reserve(vertices.size());
    for(const auto& v:vertices){PathPoint p;p.x=v[0];p.y=v[1];p.z=v[2];p.c=v[3];p.b=v[4];p.axisMask=31;p.feed=motion.speed;p.continuous=true;path.push_back(p);}
    if(!prepareMergePath(path,motion,err))return false;
    bool finished=false;return pumpMergeBuffer(finished,err);
}
