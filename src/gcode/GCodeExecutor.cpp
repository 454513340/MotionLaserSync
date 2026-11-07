#include <QStringList>
#include "gcode/GCodeExecutor.h"
#include "gcode/GCodeParser.h"
#include "gcode/GCodeSyncMetadata.h"
#include "gcode/MotionCompletionPolicy.h"
#include "core/ZmcController.h"
#include "core/LaserSensor.h"
#include "core/SyncManager.h"
#include "core/Physical17Identity.h"
#include "core/PointCloudBuilder.h"
#include "utils/AsyncSaveManager.h"
#include "utils/CompactScanFormat.h"
#include "utils/GlobalTimeSync.h"
#include "utils/LogManager.h"

#include <QDebug>
#include <QFileInfo>
#include <QFile>
#include <QCryptographicHash>
#include <array>
#include <QDir>
#include <QThread>
#include <QCoreApplication>
#include "runtime/FiveAxis.h"
#include "twin/MultiPoseNcSimulation.h"
#include "twin/NcG93MotionAudit.h"
#include <algorithm>
#include <cmath>

GCodeExecutor::GCodeExecutor(ZmcController* zmc,
                             LaserSensor* laser,
                             SyncManager* syncMgr,
                             PointCloudBuilder* cloudBuilder,
                             QObject* parent)
    : QObject(parent)
    , m_zmc(zmc)
    , m_laser(laser)
    , m_syncMgr(syncMgr)
    , m_cloudBuilder(cloudBuilder)
{
    // 初始化定时器
    m_stateMachineTimer.setInterval(10);
    connect(&m_stateMachineTimer, &QTimer::timeout, 
            this, &GCodeExecutor::onStateMachineTimer);
    
    // 50ms remains responsive but avoids starving the 10ms motion-buffer
    // feeder with 5-axis controller status queries on the same ECI I/O bus.
    m_motionCheckTimer.setInterval(50);
    connect(&m_motionCheckTimer, &QTimer::timeout,
            this, &GCodeExecutor::onMotionCheckTimer);
    
    m_saveCheckTimer.setInterval(100);
    connect(&m_saveCheckTimer, &QTimer::timeout,
            this, &GCodeExecutor::onSaveCheckTimer);
    
    m_dwellTimer.setSingleShot(true);
    connect(&m_dwellTimer, &QTimer::timeout,
            this, &GCodeExecutor::onDwellTimer);
    
    m_mergeTimer.setInterval(10);
    connect(&m_mergeTimer, &QTimer::timeout, this, [this]() {
        if (!m_zmc || !m_zmc->isConnected()) {
            m_mergeTimer.stop();
            handleError(tr("轨迹发送中控制器断开"));
            return;
        }
        
        bool finished = false;
        QString err;
        if (!m_zmc->pumpMergeBuffer(finished, &err)) {
            m_mergeTimer.stop();
            handleError(tr("轨迹填充失败: %1").arg(err));
            return;
        }
        
        if (finished) {
            m_mergeTimer.stop();
            m_trajectoryDataSent = true;
            LOG_DEBUG(LogCategory::GCode, tr("轨迹数据已全部发送"));
        }
    });
    
    connect(m_syncMgr,&SyncManager::errorOccurred,this,[this](const QString& message){
        if(isRunning() || m_state==ExecutorState::Paused)handleError(message);
    });
    // 默认保存参数
    m_currentSaveParams.baseDir = "./scan_data";
    m_currentSaveParams.prefix = "scan";
    m_currentSaveParams.format = "csv";
}

GCodeExecutor::~GCodeExecutor()
{
    stop();
}

// ==================== 程序管理 ====================

bool GCodeExecutor::loadProgram(const GCodeProgram& program, QString* err)
{
    if (isRunning() || m_state == ExecutorState::Paused) {
        if (err) *err = tr("正在执行或暂停中，无法加载新程序");
        return false;
    }
    
    GCodeParser verifier;
    QList<ParseError> problems;
    if(!verifier.validateProgram(program,&problems)){
        if(err)*err=problems.isEmpty()?tr("G代码预检失败"):problems.first().errorMessage;
        return false;
    }
    m_program = program;
    m_currentBlockIndex = 0;
    m_currentSaveParams = program.defaultSaveParams;
    
    if (m_currentSaveParams.baseDir.isEmpty()) {
        m_currentSaveParams.baseDir = "./scan_data";
    }
    if (m_currentSaveParams.prefix.isEmpty()) {
        m_currentSaveParams.prefix = "scan";
    }
    
    transitionTo(ExecutorState::Ready);
    
    emit progressMessage(tr("程序已加载: %1 (%2 个程序段)")
        .arg(program.name)
        .arg(program.blocks.size()));
    
    return true;
}

bool GCodeExecutor::loadFromFile(const QString& filePath, QString* err)
{
    GCodeParser parser;
    GCodeProgram program;
    QList<ParseError> errors;
    
    if (!parser.parseFile(filePath, program, &errors)) {
        if (err) {
            QString errorMsg;
            for (const auto& e : errors) {
                errorMsg += tr("行 %1: %2\n").arg(e.lineNumber).arg(e.errorMessage);
            }
            *err = errorMsg;
        }
        return false;
    }
    
    // 显示警告但继续
    for (const auto& e : errors) {
        emit warningOccurred(tr("行 %1: %2").arg(e.lineNumber).arg(e.errorMessage));
    }
    
    return loadProgram(program, err);
}

bool GCodeExecutor::loadFromText(const QString& text, QString* err)
{
    GCodeParser parser;
    GCodeProgram program;
    QList<ParseError> errors;
    
    if (!parser.parseText(text, program, &errors)) {
        if (err) {
            QString errorMsg;
            for (const auto& e : errors) {
                errorMsg += tr("行 %1: %2\n").arg(e.lineNumber).arg(e.errorMessage);
            }
            *err = errorMsg;
        }
        return false;
    }
    
    for (const auto& e : errors) {
        emit warningOccurred(tr("行 %1: %2").arg(e.lineNumber).arg(e.errorMessage));
    }
    
    return loadProgram(program, err);
}

bool GCodeExecutor::setRtcpKinematicModel(const std::array<double,17>& parameters,
                                              bool absoluteGaugeKnown,
                                              QString* err)
{
    if(isRunning() || m_state==ExecutorState::Paused){
        if(err)*err=tr("G代码运行或暂停时禁止替换RTCP标定模型");
        return false;
    }
    for(double v:parameters)if(!std::isfinite(v)){if(err)*err=tr("RTCP 17参数包含NaN/Inf");return false;}
    mls::calibration::Physical17Parameters p;p.value=parameters;
    const mls::runtime::Joint q{0,0,0,0,0};
    const auto tcp=mls::runtime::Rtcp17Model::controlledPointInMachine(q,p,m_rtcpOptions.controlledPointSensorMm);
    if(!std::isfinite(tcp.x)||!std::isfinite(tcp.y)||!std::isfinite(tcp.z)){if(err)*err=tr("RTCP 17参数正运动学无效");return false;}
    m_rtcpParameters=parameters;m_rtcpAbsoluteGaugeKnown=absoluteGaugeKnown;m_rtcpModelConfigured=true;
    if(err)err->clear();
    return true;
}

bool GCodeExecutor::clearRtcpKinematicModel(QString* err)
{
    if(isRunning() || m_state==ExecutorState::Paused){
        if(err)*err=tr("G代码运行或暂停时禁止清除RTCP标定模型");
        return false;
    }
    m_rtcpParameters.fill(0.0);
    m_rtcpModelConfigured=false;
    m_rtcpAbsoluteGaugeKnown=false;
    if(err)err->clear();
    return true;
}

void GCodeExecutor::unloadProgram()
{
    if (isRunning()) {
        stop();
    }
    
    m_program = GCodeProgram();
    m_currentBlockIndex = 0;
    transitionTo(ExecutorState::Idle);
}

// This check intentionally has no command/control side effects. In particular,
// do NOT call SyncManager::start(), ZmcController::prepareMergePath(), or any
// arm/motion method here. Do not convert this report to a machine run permit.
GCodeExecutor::RealRunPreflightReport GCodeExecutor::inspectRealRunReadOnly(
    const GCodeProgram& p, const QString& ncText, const QString& originalPath,
    const HardwareSyncUiIntent& uiIntent)
{
    RealRunPreflightReport r;
    if (!originalPath.isEmpty()) {
        QFile source(originalPath);
        if (!source.open(QIODevice::ReadOnly)) {
            r.blockers << tr("无法读取原始NC文件，无法锚定磁盘版本：%1").arg(source.errorString());
        } else {
            const QByteArray bytes = source.readAll();
            QString diskText=QString::fromUtf8(bytes);
            diskText.replace(QStringLiteral("\r\n"),QStringLiteral("\n"));
            QString editorText=ncText;
            editorText.replace(QStringLiteral("\r\n"),QStringLiteral("\n"));
            if (diskText != editorText)
                r.blockers << tr("磁盘NC与编辑器当前解析内容不一致；重新从磁盘载入并核查");
            r.ncSha256 = QString::fromLatin1(QCryptographicHash::hash(
                bytes, QCryptographicHash::Sha256).toHex());
            r.observations << tr("原始文件：%1").arg(originalPath);
        }
    } else {
        r.ncSha256 = QString::fromLatin1(QCryptographicHash::hash(
            ncText.toUtf8(), QCryptographicHash::Sha256).toHex());
        r.blockers << tr("NC尚未关联保存后的原始磁盘文件；编辑器内容的SHA256不构成已核验机床程序");
    }
    r.observations << tr("SHA-256：%1").arg(r.ncSha256);

    // Never interpret free-form comments as independently signed approvals.
    auto headerValue = [&](const QString& requested)->QString {
        const auto lines = ncText.split(QLatin1Char('\n'));
        for (const auto& raw : lines) {
            const QString line=raw.trimmed();
            if (!line.startsWith(QLatin1Char(';'))) continue;
            const QString part=line.mid(1).trimmed();
            const qsizetype colon=part.indexOf(QLatin1Char(':'));
            if (colon>0 && part.left(colon).trimmed().compare(requested,Qt::CaseInsensitive)==0)
                return part.mid(colon+1).trimmed();
        }
        return QString();
    };
    // Free-text NC headers are NOT evidence of actual registration/FCL/optics.
    // The active CAD scene is independently ray-traced in the dedicated geometry audit.
    for (const QString key : {QStringLiteral("RegistrationVerified"),
                              QStringLiteral("SimulationVerified"),
                              QStringLiteral("InitialJointAnchored")}) {
        const QString state=headerValue(key);
        r.observations << tr("未采用NC注释 %1=%2 作为现场许可或阻断")
            .arg(key,state.isEmpty()?tr("未声明"):state);
    }
    if(p.offlineOnly)
        r.observations << tr("OfflineOnly为数字孪生候选来源标识；只有独立现场发布验证器在全部证据齐备时可条件放行；当前只读检查本身不授权");
    r.requiredActions << tr("V30综合诊断：NC逐段重放、按NC执行真实硬件试采、整条NC FCL和STEP/ROI光学重算；不采信注释");
    r.observations << tr("程序字节SHA仅用于精确区分检查时的NC版本；不作为17参数标定有效性的判断依据");
    if (p.programFrame != QStringLiteral("MACHINE_JOINTS_XYZCB"))
        r.blockers << tr("坐标语义不是已实现的 MACHINE_JOINTS_XYZCB");
    // The NC header cannot attest a machine enclosure, fixtures, guarding,
    // emergency stop, electrical clock topology, or a collision-safe approach.
    r.observations << tr("夹具、整机FCL及硬件同步的现场认证不从NC注释推导");

    bool captures=false;
    const GCodeBlock* initial=nullptr;
    int m50=0,m51=0;
    for (const auto& b : p.blocks) {
        if (b.type==BlockType::M50_SyncOn) {++m50;captures=true;}
        if (b.type==BlockType::M51_SyncOff) ++m51;
        if (b.syncCapture) captures=true;
        if (!initial && GCodeUtils::isMotionBlock(b.type)) initial=&b;
    }
    r.observations << tr("采集组：M50=%1，M51=%2；完整运动/运动限位/FCL需要独立的整条NC复核")
        .arg(m50).arg(m51);
    if (captures && m50!=m51)
        r.blockers << tr("M50/M51数量不匹配，可能发生采集持续开启");
    if (captures && p.syncMode==SyncMode::Hardware) {
        std::array<double,17> active{};
        if(!m_syncMgr || !m_syncMgr->currentKinematicModel(&active)){
            r.blockers << tr("当前同步重建模块尚未应用17参数标定；请检查启动标定恢复是否成功");
        } else {
            bool finite=true;
            for(const double v:active)finite=finite&&std::isfinite(v);
            if(!finite)r.blockers << tr("当前生效17参数存在非有限数值");
            else r.observations << tr("已读取当前SyncManager生效17参数数值；不要求其哈希与NC注释相等；光学复核必须使用当前数值重新计算");
        }
    }
    if (!initial || initial->type!=BlockType::G00_RapidMove ||
        !(initial->hasX&&initial->hasY&&initial->hasZ&&initial->hasC&&initial->hasB)) {
        r.blockers << tr("未找到完整XYZCB第一条G00初始目标；无法验证实际进场路径");
    }
    // V30 diagnostics must enforce the same modal-coordinate constraint as
    // GCodeExecutor::start(). Do not declare a relative/G92 approach safe if
    // Start would later interpret the first G00 as a machine absolute target.
    if(captures && p.syncMode==SyncMode::Hardware){
        for(const auto& b:p.blocks){
            if(&b==initial)break;
            if(b.type==BlockType::G91_IncrementalMode || b.type==BlockType::G92_SetPosition){
                r.blockers << tr("首G00之前含G91/G92，实机准入不允许未经复核的相对坐标/工件偏置");
                break;
            }
        }
        if(initial && !initial->absoluteModeSnapshot.value_or(true))
            r.blockers << tr("首G00不是G90绝对机床坐标，不能认证进场路径");
    }

    if (captures && p.syncMode == SyncMode::Hardware) {
        if (!m_syncMgr) {
            r.blockers << tr("硬件同步管理器缺失");
        } else {
            r.observations << tr("NC申请H%1/%2Hz/R1=%3。UI和SyncManager内存参数不再被用作硬件现场判据")
                .arg(p.hardwareTopologyId).arg(p.hardwareFrequencyHz).arg(p.cmmR1Edge);
            r.requiredActions << tr("综合诊断自动按NC试采并验证真实轮廓/锁存/丢帧/停止结果；空采集不算成功");
            if(!m_syncMgr->hasKinematicModel())
                r.blockers << tr("同步管理器尚未加载有效的五轴运动学标定");
            QString triggerProblem;
            if(!mls::gcode::validateSensorTriggerMetadata(p,&triggerProblem))
                r.blockers << tr("NC传感器触发方式与H1/H3冲突：%1").arg(triggerProblem);
            else r.observations << (p.hardwareTopologyId==1
                ? tr("H1传感器OP14外部上升沿；控制器IN0/R0锁存104，软件已明确选择对应拓扑")
                : tr("H3传感器内部周期；控制器IN1/R1锁存沿由CmmR1Edge选择"));
        }
        if(!m_laser || !m_laser->isConnected())
            r.blockers << tr("线激光传感器当前未连接");
    }

    if (!m_zmc || !m_zmc->isConnected()) {
        r.blockers << tr("机床控制器当前未连接：无法读取真实MPOS/回零/空闲状态");
    } else {
        if(captures && p.syncMode==SyncMode::Hardware){
            QString tableReport,tableErr;bool tableAuthorized=false;
            if(!m_zmc->hwTableCommissioningPreflight(tableReport,&tableErr,&tableAuthorized) || !tableAuthorized)
                r.blockers << tr("实际控制器TABLE状态未满足要求：%1").arg(tableErr);
            r.observations << tr("实时控制器TABLE预检：%1").arg(tableReport);
            r.observations << tr("注意：TABLE读回不证明已经产生真实硬件触发；必须另做主动试采");
        }
        if(!m_zmc->areAllAxesHomed())
            r.blockers << tr("五轴并非全部完成本上电会话的真实回零和绝对就绪");
        QString readError;
        JointPose5D live{};
        if(!m_zmc->readJointPose5D(live,&readError)) {
            r.blockers << tr("读取实际五轴MPOS失败：%1").arg(readError);
        } else {
            r.observations << tr("检测到当前五轴MPOS：X%1 Y%2 Z%3 C%4 B%5")
                .arg(live.x,0,'f',4).arg(live.y,0,'f',4).arg(live.z,0,'f',4)
                .arg(live.c,0,'f',4).arg(live.b,0,'f',4);
            r.observations << tr("第一条G00是开始前实际定位命令；不以当前位置等于起点为准入条件");
            r.requiredActions << tr("起点路径碰撞验收应包含最新MPOS到第一G00、所有G05扫描段和非采集转场；现场移动前必须更新检查");
        }
        QString health;
        if(!m_zmc->checkMotionHealth5D(&health))
            r.blockers << tr("五轴运动健康检查失败：%1").arg(health);
        QString idleError;
        if(!m_zmc->areAllAxesIdle(&idleError))
            r.blockers << tr("五轴不是全部空闲：%1").arg(idleError);
    }
    // This uses the same mls::runtime::plan() as the live five-axis SP
    // controller path. The earlier ideal XYZCB NC replay does NOT prove that
    // 60/F gives enough time to accelerate and decelerate every G05 point.
    if(captures && p.syncMode==SyncMode::Hardware){
        QString timingError,timingSummary;
        if(!validateG93DynamicsReadOnly(p,ncText,&timingError,&timingSummary))
            r.blockers << tr("G93五轴动力学预检失败（在硬件试采/碰撞复核之前必须解决）：%1").arg(timingError);
        else if(!timingSummary.isEmpty())r.observations << timingSummary;
    }
    r.observations << tr("说明：以上全部是只读核查；不调用控制器运动、PWM、采集或安全闸门解锁指令");
    return r;
}

bool GCodeExecutor::validateG93DynamicsReadOnly(const GCodeProgram& program,
                                                 const QString& ncText,
                                                 QString* error,
                                                 QString* summary) const
{
    if(summary)summary->clear();
    const bool hasG05=std::any_of(program.blocks.begin(),program.blocks.end(),
        [](const GCodeBlock& b){return b.type==BlockType::G05_ContinuousPath;});
    if(!hasG05)return true; // Other G01/G02 paths have independent live checks.
    if(!m_zmc || !m_zmc->isConnected()){
        if(error)*error=tr("读取五轴限速/加速度前控制器已断开");return false;
    }
    mls::runtime::Limits limits;
    QString limitsError;
    if(!m_zmc->readTrajectoryLimits5D(limits,&limitsError)){
        if(error)*error=tr("读取当前五轴SP限速设置失败：%1").arg(limitsError);return false;
    }
    const QByteArray utf8=ncText.toUtf8();
    const std::string source(utf8.constData(),std::size_t(utf8.size()));
    const auto sim=mls::twin::simulateMultiPoseNc(source,200000);
    if(!sim.ok()){
        if(error)*error=tr("无法按严格G05/G93 NC语义预演：%1")
            .arg(QString::fromStdString(sim.error));return false;
    }
    const auto audit=mls::twin::auditNcG93Motion(sim,limits,
        program.motionParams.accel,program.motionParams.decel);
    if(!audit.passed){
        if(error)*error=QString::fromStdString(audit.error);
        return false;
    }
    if(summary)*summary=tr("已用实机五轴SP规划器进行整条G93限速/加减速预演：%1运动段、%2组、名义%3秒，规划%4秒。"
                            "实际控制器的拐角融合、加加速度限制和到位误差仍需现场核验。")
        .arg(static_cast<qulonglong>(audit.segmentsChecked))
        .arg(static_cast<qulonglong>(audit.groupsChecked))
        .arg(audit.nominalSeconds,0,'f',3)
        .arg(audit.plannedSeconds,0,'f',3);
    return true;
}

// ==================== 执行控制 ====================

bool GCodeExecutor::start(QString* err)
{
    // A previous attempt's live-entry approval must never be reused.
    m_firstMoveEntryQualified=false;
    if (!hasProgram()) {
        if (err) *err = tr("没有加载程序");
        return false;
    }
    // OfflineOnly marks a digital-twin candidate, NOT a permanent ban.
    // The only candidate eligible for commissioned release is an explicit
    // hardware-synchronized MACHINE_JOINTS_XYZCB scan program.  Its final
    // permission is granted later by the independent SiteHardwareNcVerifier,
    // after fresh on-machine checks; there is never a comment-based bypass.
    bool needsCapture=false, needsRtcp=false;
    for(const auto& b:m_program.blocks){
        needsCapture=needsCapture||b.syncCapture||b.type==BlockType::M50_SyncOn;
        needsRtcp=needsRtcp||b.rtcpModeSnapshot.value_or(false);
    }
    if(m_program.offlineOnly){
        const bool validCandidate = needsCapture &&
            m_program.syncMode==SyncMode::Hardware &&
            m_program.programFrame==QStringLiteral("MACHINE_JOINTS_XYZCB") &&
            (m_program.hardwareTopologyId==1 || m_program.hardwareTopologyId==3) &&
            mls::gcode::validateSensorTriggerMetadata(m_program);
        if(!validCandidate){
            if(err)*err=tr("OfflineOnly仅可作为明确H1/H3硬件同步、机床关节坐标的多姿态NC候选；本文件不满足条件");
            return false;
        }
        // No execution is permitted by this branch.  The site verifier must
        // still produce a POSITIVE verdict for this exact disk program.
    }
    if(m_program.syncMode==SyncMode::Hardware){
        if(!m_syncMgr){if(err)*err=tr("硬件同步管理器未初始化");return false;}
        if(m_program.hardwareTopologyId!=static_cast<int>(m_syncMgr->hardwareTopology()) ||
           m_program.hardwareFrequencyHz!=m_syncMgr->hardwareTriggerFrequencyHz() ||
           (m_program.hardwareTopologyId==3 && m_program.cmmR1Edge!=m_syncMgr->currentCmmR1Edge())){
            if(err)*err=tr("硬件同步G代码的H1/H3、频率或H3锁存沿与现场配置不一致，禁止任何运动");
            return false;
        }
    }
    
    if (isRunning() || m_state == ExecutorState::Paused) {
        if (err) *err = tr("程序已在运行或暂停中；暂停后应使用恢复，不得重新从头开始");
        return false;
    }
    
    if (!m_zmc || !m_zmc->isConnected()) {
        if (err) *err = tr("控制器未连接");
        return false;
    }
    
    if(needsCapture && (!m_laser || !m_laser->isConnected() || !m_syncMgr || !m_syncMgr->hasKinematicModel())){
        if(err)*err=tr("采集程序需要连接传感器并应用17参数五轴标定");return false;
    }
    // Conditional hardware-G-code execution path. The historical unconditional
    // rejection is replaced with positive on-site commissioning requirements.
    // Neither a changed OfflineOnly comment nor one in-app acknowledgement can
    // satisfy this trust boundary. All checks happen BEFORE ANY axis motion.
    if(needsCapture && m_program.syncMode==SyncMode::Hardware){
        // A source label, even a content-addressed one, cannot substitute for
        // the exact already-applied Physical17 numerical model.
        // Calibration is the active machine model, not a string from the NC comments.
        std::array<double,17> active{};
        if(!m_syncMgr->currentKinematicModel(&active)){
            if(err)*err=tr("当前未加载生效的Physical17标定；检查自动标定恢复状态");
            return false;
        }
        for(double v:active)if(!std::isfinite(v)){
            if(err)*err=tr("当前生效Physical17标定包含非法数值");return false;
        }
        QString triggerProblem;
        if(!mls::gcode::validateSensorTriggerMetadata(m_program,&triggerProblem)){
            if(err)*err=tr("NC传感器触发方式与H1/H3拓扑不符：%1").arg(triggerProblem);
            return false;
        }
        if(!m_siteHardwareNcVerifier){
            if(err)*err=tr("现场独立NC发布验证器尚未接入。不能仅靠NC注释/检查按钮授权硬件采集");
            return false;
        }
        if(!m_zmc->areAllAxesHomed()){
            if(err)*err=tr("五轴尚未完成本次上电会话的回零和绝对位置确认");
            return false;
        }
        const GCodeBlock* first=nullptr;
        for(const auto& b:m_program.blocks){
            if(GCodeUtils::isMotionBlock(b.type)){first=&b;break;}
        }
        // The original live-entry checker incorrectly treated G91/G92 words as
        // absolute machine coordinates. Generated hardware NC uses G90 and no G92.
        // Reject ambiguous machine-coordinate transforms before motion.
        for(const auto& b:m_program.blocks){
            if(&b==first)break;
            if(b.type==BlockType::G91_IncrementalMode || b.type==BlockType::G92_SetPosition){
                if(err)*err=tr("硬件扫描首G00之前不允许G91/G92；需采用已验证的G90绝对机床坐标");
                return false;
            }
        }
        if(first && !first->absoluteModeSnapshot.value_or(true)){
            if(err)*err=tr("硬件采集首G00不是G90绝对机床坐标，拒绝错误的路径比对");return false;
        }
        if(!first || first->type!=BlockType::G00_RapidMove ||
           !(first->hasX&&first->hasY&&first->hasZ&&first->hasC&&first->hasB)){
            if(err)*err=tr("硬件扫描NC须有完整五轴首G00定位参考");
            return false;
        }
        // The NC's first G00 is a REAL positioning block, not a requirement
        // that the machine had already been positioned at file-import time.
        // Re-read LIVE MPOS immediately before starting; never use a stale
        // read-only report as the run-time start state.
        JointPose5D measured{};
        if(!m_zmc->readJointPose5D(measured,err))return false;
        const std::array<double,5> liveValues{measured.x,measured.y,measured.z,measured.c,measured.b};
        const std::array<double,5> targetValues{first->target.x,first->target.y,
            first->target.z,first->target.c,first->target.b};
        const auto entry=mls::gcode::preflight::analyzeFirstG00(liveValues,targetValues);
        if(!entry.valid){
            if(err)*err=tr("首G00与实时MPOS包含无效轴坐标");
            return false;
        }
        if(m_program.filePath.isEmpty()){
            if(err)*err=tr("硬件扫描程序尚未绑定实际待执行的磁盘NC原件");
            return false;
        }
        QFile nc(m_program.filePath);
        if(!nc.open(QIODevice::ReadOnly)){
            if(err)*err=tr("无法读取待执行的NC原件：%1").arg(nc.errorString());
            return false;
        }
        // Check the file against the exact text that created the in-memory
        // program.  Comparing only a path's SHA without checking the decoded
        // instructions permits a caller other than the GUI to load A, point
        // filePath to B and pass B's digest to a verifier while executing A.
        const QByteArray sourceBytes=nc.readAll();
        // Revalidate the exact disk NC and current SP limits at the final gate.
        // This must happen BEFORE the initial G00; otherwise an impossible
        // first G05 would start the sensor and then fail at the first point.
        QString timingError;
        if(!validateG93DynamicsReadOnly(m_program,QString::fromUtf8(sourceBytes),&timingError)){
            if(err)*err=tr("NC G93运动时间不可实现，已在首G00前拦截：%1").arg(timingError);
            return false;
        }
        const QByteArray digest=QCryptographicHash::hash(sourceBytes,QCryptographicHash::Sha256);
        GCodeParser diskParser;
        GCodeProgram diskProgram;
        QList<ParseError> diskErrors;
        if(!diskParser.parseText(QString::fromUtf8(sourceBytes),diskProgram,&diskErrors) ||
           !diskParser.validateProgram(diskProgram,&diskErrors) ||
           diskProgram.blocks.size()!=m_program.blocks.size() ||
           diskProgram.syncMode!=m_program.syncMode ||
           diskProgram.offlineOnly!=m_program.offlineOnly ||
           diskProgram.programFrame!=m_program.programFrame ||
           diskProgram.hardwareTopologyId!=m_program.hardwareTopologyId ||
           diskProgram.hardwareFrequencyHz!=m_program.hardwareFrequencyHz ||
           diskProgram.cmmR1Edge!=m_program.cmmR1Edge ||
           diskProgram.hardwareTriggerType!=m_program.hardwareTriggerType ||
           diskProgram.sensorTriggerMode!=m_program.sensorTriggerMode ||
           diskProgram.motionParams.feedSpeed!=m_program.motionParams.feedSpeed ||
           diskProgram.motionParams.rapidSpeed!=m_program.motionParams.rapidSpeed ||
           diskProgram.motionParams.accel!=m_program.motionParams.accel ||
           diskProgram.motionParams.decel!=m_program.motionParams.decel){
            if(err)*err=tr("磁盘原始NC与内存中将执行的程序/硬件或运动参数不同，拒绝复用旧准入结果");
            return false;
        }
        for(std::size_t i=0;i<m_program.blocks.size();++i){
            if(diskProgram.blocks[i].rawLine!=m_program.blocks[i].rawLine ||
               diskProgram.blocks[i].type!=m_program.blocks[i].type){
                if(err)*err=tr("磁盘NC第%1段与即将执行的内存程序不同").arg(i+1);
                return false;
            }
        }
        QString approvalError;
        if(!m_siteHardwareNcVerifier(m_program,digest,&approvalError)){
            if(err)*err=tr("本次NC完整轨迹/实测同步/现场安全准入未通过：%1").arg(approvalError);
            return false;
        }
        {
            // No tolerance-based bypass: even a sub-0.05mm first G00 is a
            // physical motion request. No pre-position requirement; validate
            // live MPOS -> target when actually starting execution.
            if(!m_siteEntryMoveVerifier){
                if(err)*err=tr("首G00需实时路径FCL/夹具核查，现场起点路径验证器尚未接入；无需事先人工到位");
                return false;
            }
            const JointPose5D firstTarget{first->target.x,first->target.y,
                first->target.z,first->target.c,first->target.b};
            QString entryError;
            if(!m_siteEntryMoveVerifier(measured,firstTarget,digest,&entryError)){
                if(err)*err=tr("现场首G00进场路径未通过核查：%1").arg(entryError);
                return false;
            }
        }
        // The controller's TABLE[342], actual hardware configuration and
        // trigger-source readbacks are rechecked inside SyncManager::start().
        QString tableReport,tableError;bool tableAuthorized=false;
        if(!m_zmc->hwTableCommissioningPreflight(tableReport,&tableError,&tableAuthorized)
           || !tableAuthorized){
            if(err)*err=tr("控制器H1/H3硬件TABLE未获现场授权：%1").arg(tableError);
            return false;
        }
        // Explicitly bound to THIS NC after all checks, no stale GUI default.
        m_firstMoveEntryLivePose=measured;
        m_firstMoveApprovedNcDigest=digest;
        m_firstMoveEntryQualified=true;
        if(m_program.hardwareTopologyId==1)
            m_syncMgr->setTriggerType(HardwareTriggerType::EdgeRising);
        else m_syncMgr->clearHardwareTriggerTypeConfiguration();
    }
    if(needsRtcp && !m_rtcpModelConfigured){if(err)*err=tr("程序包含G43.4 RTCP，但尚未应用17参数/手眼运动学模型");return false;}
    if(!m_zmc->checkMotionHealth5D(err) || !m_zmc->areAllAxesIdle(err))return false;
    m_singleStep=false;m_finishing=false;
    m_workOffsets={};m_rtcpWorkOffsets={};m_rtcpWorkOffsetMask=0;m_lastMotionTarget={};m_batchBlocks=1;m_controllerHeld=false;m_pauseCompletesBlock=false;
    // 重置状态
    m_currentBlockIndex = 0;
    m_syncActive = false;
    m_absoluteMode = true;
    m_idleCheckCount = 0;
    m_trajectoryDataSent = false;
    m_idleOutOfToleranceSinceMs=-1;
    m_motionDiagnosticPhase=-1;
    m_pendingSaveTasks.clear();
    m_currentSaveParams = m_program.defaultSaveParams;
    m_currentSaveParams.autoIndex = 0;
    
    // 初始化模态进给速度为程序默认值
    m_modalFeedSpeed = m_program.motionParams.feedSpeed;
    
    LOG_INFO(LogCategory::GCode, tr("初始化运动参数 - 默认进给=%1, 快速=%2, 加速=%3")
             .arg(m_program.motionParams.feedSpeed, 0, 'f', 2)
             .arg(m_program.motionParams.rapidSpeed, 0, 'f', 2)
             .arg(m_program.motionParams.accel, 0, 'f', 2));
    
    // 配置同步管理器。G-code 当前没有独立保存/解析硬件触发极性，
    // 因此绝不能继承同一进程里上一次手动采集留下的触发类型。
    m_syncMgr->setSyncMode(m_program.syncMode);
    m_syncMgr->setSensorParams(m_program.sensorParams);
    m_syncMgr->setMountOffset(m_program.mountOffset);
    // For trusted hardware NC, trigger type was explicitly set from the
    // approved NC above. Clearing it now would make M50 fail every time.

    // 启动计时
    m_elapsedTimer.start();
    
    // 启动状态机
    transitionTo(ExecutorState::Running);
    m_stateMachineTimer.start();
    
    emit progressMessage(tr("程序开始执行: %1").arg(m_program.name));
    
    return true;
}

bool GCodeExecutor::resume(QString* err)
{
    m_singleStep=false;
    if(m_state!=ExecutorState::Paused){if(err)*err=tr("程序未暂停");return false;}
    if(m_controllerHeld){
        if(!m_zmc->checkMotionHealth5D(err) || !m_zmc->resumeTrajectory5D(err)){
            handleError(err?*err:tr("控制器恢复失败"));return false;
        }
        m_controllerHeld=false;m_motionWatchdog.restart();
        m_idleOutOfToleranceSinceMs=-1;
    }
    if(m_pauseCompletesBlock){m_pauseCompletesBlock=false;advanceToNextBlock();}
    else {
        transitionTo(m_resumeState);
        if(m_state==ExecutorState::WaitingMotionComplete){m_motionCheckTimer.start();if(!m_trajectoryDataSent)m_mergeTimer.start();}
        if(m_state==ExecutorState::Dwelling)m_dwellTimer.start((std::max)(0,m_dwellRemainingMs));
        if(m_state==ExecutorState::WaitingSaveComplete)m_saveCheckTimer.start();
    }
    m_stateMachineTimer.start();emit progressMessage(tr("恢复原执行状态；不重发当前运动段"));return true;
}


void GCodeExecutor::pause()
{
    if(!isRunning())return;
    m_resumeState=m_state;m_pauseCompletesBlock=false;
    if(m_state==ExecutorState::WaitingMotionComplete){
        QString err;if(!m_zmc->pauseTrajectory5D(&err)){handleError(tr("控制器暂停失败：%1").arg(err));return;}
        m_controllerHeld=true;m_motionBudgetMs=(std::max)(qint64(1000),m_motionBudgetMs-m_motionWatchdog.elapsed());
    }
    if(m_state==ExecutorState::Dwelling)m_dwellRemainingMs=m_dwellTimer.remainingTime();
    m_stateMachineTimer.stop();m_motionCheckTimer.stop();m_mergeTimer.stop();m_dwellTimer.stop();m_saveCheckTimer.stop();
    m_pauseReason=tr("用户暂停：运动缓冲保持，采集可继续记录静止轮廓");transitionTo(ExecutorState::Paused);
    emit programPaused(m_pauseReason);
}


void GCodeExecutor::stop()
{
    if (m_state == ExecutorState::Idle) return;
    
    m_controllerHeld=false;m_pauseCompletesBlock=false;m_batchBlocks=1;
    transitionTo(ExecutorState::Stopping);
    
    // 停止所有定时器
    m_stateMachineTimer.stop();
    m_motionCheckTimer.stop();
    m_saveCheckTimer.stop();
    m_dwellTimer.stop();
    m_mergeTimer.stop();
    
    // 停止运动
    if (m_zmc && m_zmc->isConnected()) {
        m_zmc->requestStopAllAxes(false);
    }
    
    // 停止采集
    if (m_syncActive) {
        m_syncMgr->stop();
        m_syncActive = false;
    }
    
    // 取消所有保存任务
    AsyncSaveManager::instance().cancelAllTasks();
    m_pendingSaveTasks.clear();
    
    transitionTo(ExecutorState::Idle);
    
    emit programStopped();
    emit progressMessage(tr("程序已停止"));
}

bool GCodeExecutor::stepOnce(QString* err)
{
    if (m_state == ExecutorState::Paused) {
        if (m_controllerHeld || (m_resumeState != ExecutorState::Running && !m_pauseCompletesBlock)) {
            if (err) *err = tr("当前暂停发生在段内，请先恢复完成该段，不允许重复下发单步"); return false;
        }
        if (!resume(err)) return false;
    } else if (m_state == ExecutorState::Ready || m_state == ExecutorState::Completed) {
        if (!start(err)) return false;
    } else {
        if (err) *err = tr("单步只能从就绪或段间暂停状态开始"); return false;
    }
    m_singleStep=true;
    return true;
}

// ==================== 状态查询 ====================

const GCodeBlock* GCodeExecutor::currentBlock() const
{
    if (m_currentBlockIndex >= 0 && 
        m_currentBlockIndex < static_cast<int>(m_program.blocks.size())) {
        return &m_program.blocks[m_currentBlockIndex];
    }
    return nullptr;
}

bool GCodeExecutor::isRunning() const
{
    return m_state == ExecutorState::Running ||
           m_state == ExecutorState::ExecutingBlock ||
           m_state == ExecutorState::WaitingMotionComplete ||
           m_state == ExecutorState::WaitingSyncStart ||
           m_state == ExecutorState::WaitingSyncStop ||
           m_state == ExecutorState::WaitingSaveComplete ||
           m_state == ExecutorState::Dwelling;
}

ExecutorProgress GCodeExecutor::getProgress() const
{
    m_progress.currentBlock = m_currentBlockIndex;
    m_progress.totalBlocks = static_cast<int>(m_program.blocks.size());
    m_progress.completionPercent = m_progress.totalBlocks > 0 ?
        (m_currentBlockIndex * 100.0 / m_progress.totalBlocks) : 0;
    m_progress.elapsedTimeMs = m_elapsedTimer.isValid() ? m_elapsedTimer.elapsed() : 0;
    m_progress.state = m_state;
    
    // 获取采集统计
    if (m_syncMgr) {
        int profileCount = 0, poseCount = 0;
        m_syncMgr->getStats(profileCount, poseCount);
        m_progress.profileCount = profileCount;
        m_progress.poseCount = poseCount;
    }
    
    if (m_cloudBuilder) {
        m_progress.pointCloudCount = m_cloudBuilder->pointCount();
    }
    
    // 保存任务统计
    m_progress.saveTasksPending = AsyncSaveManager::instance().runningTaskCount();
    m_progress.saveTasksCompleted = static_cast<int>(
        AsyncSaveManager::instance().getStatistics().totalTasksCompleted);
    
    // 当前程序段信息
    if (const auto* block = currentBlock()) {
        m_progress.currentBlockInfo = QString("N%1 %2")
            .arg(block->lineNumber)
            .arg(GCodeUtils::blockTypeName(block->type));
        if (!block->comment.isEmpty()) {
            m_progress.currentBlockInfo += " (" + block->comment + ")";
        }
    }
    
    return m_progress;
}

void GCodeExecutor::setStateMachineInterval(int ms)
{
    m_stateMachineTimer.setInterval(ms);
}

// ==================== 定时器槽函数 ====================

void GCodeExecutor::onStateMachineTimer()
{
    processStateMachine();
    updateProgress();
}

void GCodeExecutor::onMotionCheckTimer()
{
    if (m_state == ExecutorState::WaitingMotionComplete) {
        if (checkMotionComplete()) {
            m_motionCheckTimer.stop();
            
            // 运动完成后，检查是否需要关闭采集
            const auto* block = currentBlock();
            if (block && m_syncActive) {
                // 检查下一个程序段是否还需要采集
                int nextIndex = m_currentBlockIndex + 1;
                bool nextNeedsSync = false;
                
                if (nextIndex < static_cast<int>(m_program.blocks.size())) {
                    const auto& nextBlock = m_program.blocks[nextIndex];
                    nextNeedsSync = nextBlock.syncCapture ||
                                   nextBlock.type == BlockType::M50_SyncOn;
                }
                
                // 如果下一段不需要采集，关闭采集
                if (!nextNeedsSync && !block->syncCapture) {
                    // 当前段不需要采集了，关闭
                }
            }
            
            advanceToNextBlock();
        }
    }
}

void GCodeExecutor::onSaveCheckTimer()
{
    auto& manager=AsyncSaveManager::instance();
    for (const auto& id : m_pendingSaveTasks) {
        const auto info=manager.getTaskInfo(id);
        if (info.status==SaveTaskStatus::Failed || info.status==SaveTaskStatus::Cancelled) {
            m_saveCheckTimer.stop();handleError(tr("保存未成功：%1：%2").arg(info.filePath).arg(info.errorMessage));return;
        }
        if (info.status==SaveTaskStatus::Pending || info.status==SaveTaskStatus::Running) return;
    }
    m_saveCheckTimer.stop();m_pendingSaveTasks.clear();
    if (m_state==ExecutorState::WaitingSaveComplete) {
        if (m_finishing) finishProgram(); else advanceToNextBlock();
    }
}

void GCodeExecutor::onDwellTimer()
{
    if (m_state == ExecutorState::Dwelling) {
        advanceToNextBlock();
    }
}

// ==================== 状态机处理 ====================

void GCodeExecutor::processStateMachine()
{
    switch (m_state) {
    case ExecutorState::Running:
        if (m_currentBlockIndex < static_cast<int>(m_program.blocks.size())) {
            transitionTo(ExecutorState::ExecutingBlock);
        } else {
            finishProgram();
        }
        break;
        
    case ExecutorState::ExecutingBlock:
        executeCurrentBlock();
        break;
        
    case ExecutorState::WaitingSyncStart:
        if (m_syncMgr->isRunning()) {
            if(m_syncMgr->syncMode()==SyncMode::Hardware){
                const auto stats=m_syncMgr->getHardwareAlignStats();
                if(stats.alignmentFault){handleError(stats.faultReason);return;}
                if(!stats.syncEstablished){if(m_blockTimer.elapsed()>2500)handleError(tr("采集启动探针握手超时"));return;}
            }
            m_syncActive = true;
            emit syncStateChanged(true);
            emit progressMessage(tr("同步采集已启动"));
            
            // 继续执行当前程序段的运动
            const auto* block = currentBlock();
            if (block && GCodeUtils::isMotionBlock(block->type)) {
                QString err;
                if (!startMotion(*block, &err)) {
                    handleError(err);
                } else {
                    transitionTo(ExecutorState::WaitingMotionComplete);
                    m_motionCheckTimer.start();
                }
            } else {
                advanceToNextBlock();
            }
        }
        break;
        
    case ExecutorState::WaitingSyncStop:
        if (!m_syncMgr->isRunning()) {
            m_syncActive = false;
            emit syncStateChanged(false);
            emit progressMessage(tr("同步采集已停止"));
            advanceToNextBlock();
        }
        break;
        
    default:
        break;
    }
}

void GCodeExecutor::transitionTo(ExecutorState newState)
{
    if (m_state != newState) {
        m_previousState = m_state;
        m_state = newState;
        emit stateChanged(newState, m_previousState);
        
        LOG_DEBUG(LogCategory::GCode, tr("状态转换: %1 -> %2")
                 .arg(GCodeUtils::stateName(m_previousState))
                 .arg(GCodeUtils::stateName(newState)));
    }
}

// ==================== 程序段执行 ====================

void GCodeExecutor::executeCurrentBlock()
{
    if (m_currentBlockIndex >= static_cast<int>(m_program.blocks.size())) {
        finishProgram();
        return;
    }
    
    const auto& block = m_program.blocks[m_currentBlockIndex];
    
    m_blockTimer.start();m_batchBlocks=1;
    if(block.absoluteModeSnapshot)m_absoluteMode=*block.absoluteModeSnapshot;
    if(block.type==BlockType::Modal && block.feedRate)m_modalFeedSpeed=*block.feedRate;
    emit blockStarted(m_currentBlockIndex, block);
    
    LOG_DEBUG(LogCategory::GCode, tr("执行程序段 %1: %2 (行号: %3)")
             .arg(m_currentBlockIndex)
             .arg(GCodeUtils::blockTypeName(block.type))
             .arg(block.lineNumber));
    
    switch (block.type) {
    case BlockType::G00_RapidMove:
        executeG00RapidMove(block);
        break;
        
    case BlockType::G01_LinearInterp:
        executeG01Linear(block);
        break;
        
    case BlockType::G02_ArcCW:
        executeG02G03Arc(block, true);
        break;
        
    case BlockType::G03_ArcCCW:
        executeG02G03Arc(block, false);
        break;
        
    case BlockType::G04_Dwell:
        executeG04Dwell(block);
        break;
        
    case BlockType::G05_ContinuousPath:
        executeG05ContinuousPath(block);
        break;
        
    case BlockType::G90_AbsoluteMode:
    case BlockType::G91_IncrementalMode:
        executeG90G91Mode(block);
        break;
        
    case BlockType::G92_SetPosition:
        executeG92SetPosition(block);
        break;
        
    case BlockType::M50_SyncOn:
        executeM50SyncOn();
        break;
        
    case BlockType::M51_SyncOff:
        executeM51SyncOff();
        break;
        
    case BlockType::M60_SavePointCloud:
        executeM60SavePointCloud(block);
        break;
        
    case BlockType::M61_ClearPointCloud:
        executeM61ClearPointCloud();
        break;
        
    case BlockType::M62_SetSavePath:
        executeM62SetSavePath(block);
        break;
        
    case BlockType::M63_WaitSaveComplete:
        executeM63WaitSave();
        break;
        
    case BlockType::M00_ProgramStop:
        executeM00ProgramStop(block);
        break;
        
    case BlockType::M30_ProgramEnd:
        executeM30ProgramEnd();
        break;
        
    case BlockType::Modal:
    case BlockType::Comment:
    case BlockType::Empty:
        // 跳过注释和空行
        advanceToNextBlock();
        break;
        
    default:
        handleError(tr("未知程序段类型，拒绝跳过执行"));
        break;
    }
}

void GCodeExecutor::executeG00RapidMove(const GCodeBlock& block)
{
    // The first G00 is a real move. Avoid a time-of-check/time-of-use
    // gap between on-site authorization and controller dispatch.
    if(m_firstMoveEntryQualified){
        JointPose5D liveNow{};
        QString readError;
        if(!m_zmc || !m_zmc->readJointPose5D(liveNow,&readError)){
            handleError(tr("首G00执行前再次读取MPOS失败：%1").arg(readError));
            return;
        }
        // Recalculate the actual live->G00 FCL path at dispatch time,
        // not a 0.05mm equality test against a stale MPOS snapshot.
        const JointPose5D actualTarget{block.target.x,block.target.y,block.target.z,
                                        block.target.c,block.target.b};
        QString entryError;
        if(!m_siteEntryMoveVerifier ||
           !m_siteEntryMoveVerifier(liveNow,actualTarget,m_firstMoveApprovedNcDigest,&entryError)){
            handleError(tr("首G00正式下发前最新MPOS路径FCL校验失败：%1").arg(entryError));
            return;
        }
        m_firstMoveEntryQualified=false;
    }
    // G00 非采集运动；当前程序在M50之前先到达第一条扫描起点。
    if(block.syncCapture && !m_syncActive){QString e;if(!startSyncCapture(&e)){handleError(e);return;}
        transitionTo(ExecutorState::WaitingSyncStart);return;}
    QString err;
    if (!startMotion(block, &err)) {
        handleError(tr("G00 运动失败: %1").arg(err));
        return;
    }
    
    transitionTo(ExecutorState::WaitingMotionComplete);
    m_motionCheckTimer.start();
}

void GCodeExecutor::executeG01Linear(const GCodeBlock& block)
{
    // G01 直线插补
    // 检查是否需要开启采集
    if (block.syncCapture && !m_syncActive) {
        QString err;
        if (!startSyncCapture(&err)) {
            handleError(tr("启动采集失败: %1").arg(err));
            return;
        }
        transitionTo(ExecutorState::WaitingSyncStart);
        return;  // 等待采集启动后再开始运动
    }
    
    QString err;
    if (!startMotion(block, &err)) {
        handleError(tr("G01 运动失败: %1").arg(err));
        return;
    }
    
    transitionTo(ExecutorState::WaitingMotionComplete);
    m_motionCheckTimer.start();
}

void GCodeExecutor::executeG02G03Arc(const GCodeBlock& block, bool clockwise)
{
    Q_UNUSED(clockwise)
    
    // 检查是否需要开启采集
    if (block.syncCapture && !m_syncActive) {
        QString err;
        if (!startSyncCapture(&err)) {
            handleError(tr("启动采集失败: %1").arg(err));
            return;
        }
        transitionTo(ExecutorState::WaitingSyncStart);
        return;
    }
    
    // 执行圆弧运动
    QString err;
    if (!startMotion(block, &err)) {
        handleError(tr("圆弧运动失败: %1").arg(err));
        return;
    }
    
    transitionTo(ExecutorState::WaitingMotionComplete);
    m_motionCheckTimer.start();
}

void GCodeExecutor::executeG04Dwell(const GCodeBlock& block)
{
    int dwellMs = block.dwellTimeMs;
    if(dwellMs<0){handleError(tr("负的停留时间"));return;}
    
    emit progressMessage(tr("暂停 %1 ms").arg(dwellMs));
    
    transitionTo(ExecutorState::Dwelling);
    m_dwellTimer.start(dwellMs);
}

void GCodeExecutor::executeG05ContinuousPath(const GCodeBlock& block)
{
    if(block.syncCapture && !m_syncActive){QString e;if(!startSyncCapture(&e)){handleError(e);return;}
        transitionTo(ExecutorState::WaitingSyncStart);return;}
    QString err;if(!startMotion(block,&err)){handleError(err);return;}
    transitionTo(ExecutorState::WaitingMotionComplete);m_motionCheckTimer.start();
}


void GCodeExecutor::executeG90G91Mode(const GCodeBlock& block)
{
    m_absoluteMode = (block.type == BlockType::G90_AbsoluteMode);
    emit progressMessage(tr("坐标模式: %1")
        .arg(m_absoluteMode ? tr("绝对") : tr("相对")));
    advanceToNextBlock();
}

void GCodeExecutor::executeG92SetPosition(const GCodeBlock& block)
{
    JointState5D state;QString err;
    if(!m_zmc->readJointState5D(state,&err) || !state.allIdle){handleError(tr("G92需要五轴停止且位置有效：%1").arg(err));return;}
    const bool rtcp=block.rtcpModeSnapshot.value_or(false);
    const double desired[5]={block.target.x,block.target.y,block.target.z,block.target.c,block.target.b};
    const bool specified[5]={block.hasX,block.hasY,block.hasZ,block.hasC,block.hasB};
    if(rtcp){
        if(!m_rtcpModelConfigured){handleError(tr("G92处于G43.4模式，但RTCP模型未配置"));return;}
        mls::calibration::Physical17Parameters p;p.value=m_rtcpParameters;
        const mls::runtime::Joint q{state.mpos.x,state.mpos.y,state.mpos.z,state.mpos.c,state.mpos.b};
        const auto tcp=mls::runtime::Rtcp17Model::controlledPointInMachine(q,p,m_rtcpOptions.controlledPointSensorMm);
        const double machine[5]={tcp.x,tcp.y,tcp.z,q[3],q[4]};
        for(int j=0;j<5;++j)if(specified[j]){m_rtcpWorkOffsets[j]=machine[j]-desired[j];m_rtcpWorkOffsetMask|=std::uint8_t(1u<<j);}
        emit progressMessage(tr("G92更新RTCP任务坐标偏置；未修改控制器DPOS/MPOS。XYZ偏置可作为未知平移规范的局部锚点。"));
    }else{
        const mls::runtime::Joint q{state.mpos.x,state.mpos.y,state.mpos.z,state.mpos.c,state.mpos.b};
        for(int j=0;j<5;++j)if(specified[j])m_workOffsets[j]=q[j]-desired[j];
        emit progressMessage(tr("G92仅更新程序工件偏置；C/B使用编码器MPOS，不把C/B DPOS作为测量姿态"));
    }
    advanceToNextBlock();
}

void GCodeExecutor::executeM50SyncOn()
{
    if (m_syncActive) {
        emit progressMessage(tr("采集已在运行中"));
        advanceToNextBlock();
        return;
    }
    
    QString err;
    if (!startSyncCapture(&err)) {
        handleError(tr("启动采集失败: %1").arg(err));
        return;
    }
    
    transitionTo(ExecutorState::WaitingSyncStart);
}

void GCodeExecutor::executeM51SyncOff()
{
    if (!m_syncActive) {
        emit progressMessage(tr("采集未在运行"));
        advanceToNextBlock();
        return;
    }
    
    QString err;
    if (!stopSyncCapture(&err)) {
        handleError(tr("停止采集/完整性校验失败: %1").arg(err));return;
    }
    if(m_state==ExecutorState::Error)return;
    transitionTo(ExecutorState::WaitingSyncStop);
}

void GCodeExecutor::executeM60SavePointCloud(const GCodeBlock& block)
{
    if(m_syncMgr->hasCaptureFault()){handleError(m_syncMgr->captureFaultReason());return;}
    if(m_syncMgr->isRunning()){handleError(tr("M60前必须执行M51完成尾帧排空与完整性验证"));return;}
    if (m_syncMgr->syncMode() == SyncMode::Hardware) {
        const auto stats = m_syncMgr->getHardwareAlignStats();
        if (stats.alignmentFault) {
            handleError(tr("M60拒绝自动保存无效扫描：%1。诊断导出请在主界面确认。").arg(stats.faultReason));
            return;
        }
    }
    // 生成文件路径
    QString filePath = generateSaveFilePath(block);
    
    // 获取数据
    const auto& alignedPoints = m_syncMgr->getHardwareAlignedPoints();
    
    if (alignedPoints.isEmpty()) {
        handleError(tr("M60没有有效点云；拒绝把空扫描报告为保存成功。可使用诊断导出查看轮廓关节记录。"));
        return;
    }
    
    // SCAN v3 stores exact time in each profile; the header interval is nominal.
    // Previous code hard-coded 5000 us even when H1/H3 ran at another frequency.
    const uint32_t nominalPeriodUs=(m_program.syncMode==SyncMode::Hardware &&
            m_program.hardwareFrequencyHz>0)
        ?static_cast<uint32_t>(std::lround(1000000.0/m_program.hardwareFrequencyHz))
        :0u; // Software: irregular/unknown, not a fabricated 200Hz period.
    // 提交异步保存任务
    auto& saveMgr = AsyncSaveManager::instance();
    QString taskId;
    QString format = m_currentSaveParams.format.toLower();
    
    if (format == "csv") {
        // CSV格式
        taskId = saveMgr.submitSaveAlignedPoints(
            filePath, alignedPoints, block.name, block.lineNumber);
    } 
    else if (format == "ply") {
        // PLY ASCII格式 - 需要先转换为点云
        QVector<QVector3D> points;
        points.reserve(alignedPoints.size());
        for (const auto& ap : alignedPoints) {
            points.append(QVector3D(
                static_cast<float>(ap.machineX),
                static_cast<float>(ap.machineY),
                static_cast<float>(ap.machineZ)));
        }
        taskId = saveMgr.submitSavePointCloudPLY(
            filePath, points, block.name, block.lineNumber);
    }
    else if (format == "ply_bin") {
        // PLY Binary格式
        QVector<QVector3D> points;
        points.reserve(alignedPoints.size());
        for (const auto& ap : alignedPoints) {
            points.append(QVector3D(
                static_cast<float>(ap.machineX),
                static_cast<float>(ap.machineY),
                static_cast<float>(ap.machineZ)));
        }
        taskId = saveMgr.submitSavePointCloudPLYBinary(
            filePath, points, block.name, block.lineNumber);
    }
    else if (format == "scan") {
        // 紧凑扫描格式（无压缩）
        taskId = saveMgr.submitSaveCompactScan(
            filePath, alignedPoints, CompressionType::None,
            1, 0, nominalPeriodUs, QMatrix4x4(),
            block.name, block.name, block.lineNumber);
    }
    else if (format == "scan_zlib" || format == "scan_lz4") {
        // Qt zlib; historical format name is accepted as a compatibility alias.
        taskId = saveMgr.submitSaveCompactScan(
            filePath, alignedPoints, CompressionType::QtZlib,
            1, 0, nominalPeriodUs, QMatrix4x4(),
            block.name, block.name, block.lineNumber);
    }
    else if (format == "scan_zstd") {
        // Historical alias; the on-disk codec is honestly tagged Qt zlib.
        taskId = saveMgr.submitSaveCompactScan(
            filePath, alignedPoints, CompressionType::QtZlib,
            1, 0, nominalPeriodUs, QMatrix4x4(),
            block.name, block.name, block.lineNumber);
    }
    else {
        handleError(tr("未知保存格式 '%1'，拒绝写入与扩展名不一致的文件").arg(format));
        return;
    }
    
    m_pendingSaveTasks.append(taskId);
    // PLY alone cannot retain joints; the companion also retains empty-profile poses.
    const auto poses=m_syncMgr->getHardwarePoseSamples();
    if (!poses.isEmpty()) m_pendingSaveTasks.append(saveMgr.submitSavePoseSamples(
        filePath + ".poses.csv", poses, block.name, block.lineNumber));
    m_currentSaveParams.autoIndex++;
    
    emit progressMessage(tr("点云保存已提交 [%1]: %2 (%3 点)")
        .arg(format.toUpper())
        .arg(QFileInfo(filePath).fileName())
        .arg(alignedPoints.size()));
    
    // 不等待保存完成，直接继续
    advanceToNextBlock();
}

void GCodeExecutor::executeM61ClearPointCloud()
{
    if (m_syncActive || m_syncMgr->isRunning()) {
        handleError(tr("M61前必须停止采集；不能在扫描期间清空关联数据"));return;
    }
    // 清空点云数据
    if (m_cloudBuilder) {
        m_cloudBuilder->clear();
    }
    
    // 清空同步管理器数据
    if (!m_syncActive) {
        m_syncMgr->clearData();
    }
    
    emit progressMessage(tr("点云数据已清空"));
    advanceToNextBlock();
}

void GCodeExecutor::executeM62SetSavePath(const GCodeBlock& block)
{
    if (!block.saveParams.baseDir.isEmpty()) {
        m_currentSaveParams.baseDir = block.saveParams.baseDir;
    }
    if (!block.saveParams.prefix.isEmpty()) {
        m_currentSaveParams.prefix = block.saveParams.prefix;
    }
    if (!block.saveParams.suffix.isEmpty()) {
        m_currentSaveParams.suffix = block.saveParams.suffix;
    }
    if (!block.saveParams.format.isEmpty()) {
        const QString format=block.saveParams.format.toLower();
        if (!QStringList{"csv","ply","ply_bin","scan","scan_zlib","scan_lz4","scan_zstd"}.contains(format)) {
            handleError(tr("M62不支持保存格式 %1").arg(format));return;
        }
        m_currentSaveParams.format = format;
    }
    
    // 确保目录存在
    QDir dir(m_currentSaveParams.baseDir);
    if (!dir.exists() && !dir.mkpath(".")) {
        handleError(tr("无法创建保存目录 %1").arg(dir.path()));return;
    }
    
    emit progressMessage(tr("保存路径设置: %1/%2_*.%3")
        .arg(m_currentSaveParams.baseDir)
        .arg(m_currentSaveParams.prefix)
        .arg(m_currentSaveParams.getExtension()));
    
    advanceToNextBlock();
}

void GCodeExecutor::executeM63WaitSave()
{
    m_finishing=false;
    transitionTo(ExecutorState::WaitingSaveComplete);
    m_saveCheckTimer.start();onSaveCheckTimer();
}

void GCodeExecutor::executeM00ProgramStop(const GCodeBlock& block)
{
    m_pauseReason=block.comment.isEmpty()?tr("程序暂停 (M00)"):block.comment;
    m_pauseCompletesBlock=true;m_controllerHeld=false;
    transitionTo(ExecutorState::Paused);m_stateMachineTimer.stop();emit programPaused(m_pauseReason);
}


void GCodeExecutor::executeM30ProgramEnd()
{
    finishProgram(); // asynchronous save barrier; never block the GUI event loop
}

// ==================== 同步采集控制 ====================

bool GCodeExecutor::startSyncCapture(QString* err)
{
    if (!m_syncMgr) {
        if (err) *err = tr("同步管理器未初始化");
        return false;
    }
    
    if (m_syncMgr->isRunning()) {
        return true;  // 已经在运行
    }
    
    // 配置同步管理器
    m_syncMgr->setSyncMode(m_program.syncMode);
    m_syncMgr->setSensorParams(m_program.sensorParams);
    
    // 启动全局时间同步
    GlobalTimeSync::instance().start();
    
    QString startErr;
    if (!m_syncMgr->start(&startErr)) {
        if (err) *err = startErr;
        return false;
    }
    
    return true;
}

bool GCodeExecutor::stopSyncCapture(QString* err)
{
    if(!m_syncMgr)return false;
    m_syncMgr->stop();m_syncActive=false;
    if(m_syncMgr->hasCaptureFault()){if(err)*err=m_syncMgr->captureFaultReason();return false;}
    if(m_syncMgr->syncMode()==SyncMode::Hardware){auto stats=m_syncMgr->getHardwareAlignStats();
        if(stats.alignmentFault){if(err)*err=stats.faultReason;return false;}}
    return true;
}


// ==================== 运动控制 ====================

bool GCodeExecutor::startMotion(const GCodeBlock& block,QString* err)
{
    using namespace mls::runtime;
    if(block.rtcpModeSnapshot.value_or(false)) return startRtcpMotion(block,err);
    if(!m_zmc || !m_zmc->isConnected()){if(err)*err=tr("控制器未连接");return false;}
    JointState5D state;if(!m_zmc->readJointState5D(state,err))return false;
    if(!state.allIdle){if(err)*err=tr("前一个五轴运动未结束");return false;}
    JointPose5D motionStart;
    if(!m_zmc->readMotionStartPose5D(motionStart,err))return false;
    Joint previous{motionStart.x,motionStart.y,motionStart.z,motionStart.c,motionStart.b};
    std::vector<PathPoint> path;std::string detail;m_batchBlocks=1;
    auto target=[&](const PathPoint& words,std::uint8_t mask,bool absolute,Joint& q){
        Joint v{words.x,words.y,words.z,words.c,words.b};
        if(absolute)for(int j=0;j<5;++j)if(mask&(1u<<j))v[j]+=m_workOffsets[j];
        return resolve(previous,v,mask,absolute,q,detail);
    };
    auto add=[&](const Joint& q,double feed,FeedMode mode){PathPoint p;p.x=q[0];p.y=q[1];p.z=q[2];p.c=q[3];p.b=q[4];p.feed=feed;p.feedMode=mode;p.axisMask=31;path.push_back(p);previous=q;};
    auto mask=[](const GCodeBlock& b)->std::uint8_t{return (b.hasX?1:0)|(b.hasY?2:0)|(b.hasZ?4:0)|(b.hasC?8:0)|(b.hasB?16:0);};
    auto line=[&](const GCodeBlock& b){
        if(b.feedRate)m_modalFeedSpeed=*b.feedRate;
        Joint q;if(!target(b.target,mask(b),b.absoluteModeSnapshot.value_or(m_absoluteMode),q))return false;
        const bool rapid=b.type==BlockType::G00_RapidMove;
        add(q,rapid?m_program.motionParams.rapidSpeed:m_modalFeedSpeed,rapid?FeedMode::LegacyPerSecond:b.feedMode);return true;
    };
    if(block.type==BlockType::G00_RapidMove || block.type==BlockType::G01_LinearInterp){
        if(!line(block)){if(err)*err=QString::fromStdString(detail);return false;}
        // Consecutive G1 blocks share one validated five-axis queue. Do not batch over M-code boundaries.
        if(block.type==BlockType::G01_LinearInterp && !m_singleStep){
            for(int k=m_currentBlockIndex+1;k<int(m_program.blocks.size()) && m_batchBlocks<200000;++k){
                const auto& next=m_program.blocks[k];if(next.type!=BlockType::G01_LinearInterp ||
                   next.syncCapture!=block.syncCapture ||
                   next.rtcpModeSnapshot.value_or(false)!=block.rtcpModeSnapshot.value_or(false) ||
                   next.feedMode!=block.feedMode)break;
                if(!line(next)){if(err)*err=QString::fromStdString(detail);return false;}++m_batchBlocks;
            }
        }
    }else if(block.type==BlockType::G05_ContinuousPath){
        if(block.feedRate)m_modalFeedSpeed=*block.feedRate;
        for(const auto& p:block.continuousPath.points){
            Joint q;if(!target(p,p.axisMask,block.absoluteModeSnapshot.value_or(m_absoluteMode),q)){if(err)*err=QString::fromStdString(detail);return false;}
            if(p.feed>0)m_modalFeedSpeed=p.feed;add(q,m_modalFeedSpeed,p.feedMode);
            // The FIRST knot after an M50/M51/G05 boundary is an exact-stop
            // anchor. Live controller start comes from XYZ DPOS and B/C MPOS,
            // which may differ from the NC nominal endpoint by an allowed
            // sub-0.05mm/deg settling error. Assuming perfect collinearity at
            // this junction falsely promises a 0.30s start which can require
            // 0.40s on hardware. From the second knot onward, preserve SP
            // look-ahead only at genuinely collinear XYZCB junctions.
            path.back().continuous=path.size()>1;
        }
    }else if(block.type==BlockType::G02_ArcCW || block.type==BlockType::G03_ArcCCW){
        if(block.feedRate)m_modalFeedSpeed=*block.feedRate;
        Joint end;if(!target(block.target,mask(block),block.absoluteModeSnapshot.value_or(m_absoluteMode),end)){if(err)*err=QString::fromStdString(detail);return false;}
        mls::runtime::Arc arc;arc.clockwise=block.type==BlockType::G02_ArcCW;arc.plane=block.arc.plane==ArcPlane::XY?mls::runtime::Plane::XY:(block.arc.plane==ArcPlane::XZ?mls::runtime::Plane::ZX:mls::runtime::Plane::YZ);
        arc.centerOffset={block.arc.centerI,block.arc.centerJ,block.arc.centerK,0,0};
        arc.useRadius=block.arc.defineMode==ArcDefineMode::RadiusEnd;arc.radius=block.arc.radius;
        if(block.arc.defineMode==ArcDefineMode::ThreePoint){
            Joint mid=previous;
            const bool absolute=block.absoluteModeSnapshot.value_or(m_absoluteMode);
            mid[0]=block.arc.midX+(absolute?m_workOffsets[0]:previous[0]);mid[1]=block.arc.midY+(absolute?m_workOffsets[1]:previous[1]);
            if(!threePointArc(previous,mid,end,arc.clockwise,arc,detail)){if(err)*err=QString::fromStdString(detail);return false;}
        }
        std::vector<Joint> points;if(!arcWaypoints(previous,end,arc,points,detail)){if(err)*err=QString::fromStdString(detail);return false;}
        const double feed=block.feedMode==FeedMode::InverseMinute?m_modalFeedSpeed*points.size():m_modalFeedSpeed;
        for(const auto& q:points)add(q,feed,block.feedMode);
    }else{if(err)*err=tr("不支持的五轴运动类型");return false;}
    if(path.empty()){if(err)*err=tr("五轴轨迹为空");return false;}
    AxisParams motion=m_program.motionParams.toAxisParams();
    if(!m_zmc->prepareMergePath(path,motion,err))return false;
    m_lastMotionTarget=previous;m_idleCheckCount=0;
    m_plannedMotionSeconds=m_zmc->trajectoryDurationSec();
    m_motionBudgetMs=mls::gcode::motionWatchdogBudgetMs(m_plannedMotionSeconds);
    if(m_motionBudgetMs<=0){if(err)*err=tr("五轴控制器返回非法/溢出的规划运动时间");return false;}
    m_motionWatchdog.restart();m_idleOutOfToleranceSinceMs=-1;m_motionDiagnosticPhase=-1;
    bool sent=false;if(!m_zmc->pumpMergeBuffer(sent,err))return false;
    m_trajectoryDataSent=sent;if(!sent)m_mergeTimer.start();else m_mergeTimer.stop();
    emit progressMessage(tr("五轴SP队列：%1段，控制器计划%2秒，监控上限%3秒；"
        "XYZ/C/B采用实际MPOS，到位容差0.05mm/0.05°")
        .arg(path.size()).arg(m_plannedMotionSeconds,0,'f',3).arg(m_motionBudgetMs/1000.0,0,'f',2));
    return true;
}


bool GCodeExecutor::startRtcpMotion(const GCodeBlock& block,QString* err)
{
    using namespace mls::runtime;
    using mls::calibration::Vec3d;
    if(!m_zmc || !m_zmc->isConnected()){if(err)*err=tr("控制器未连接");return false;}
    if(!m_rtcpModelConfigured){if(err)*err=tr("G43.4 RTCP需要已应用的17参数/手眼模型");return false;}
    JointState5D state;if(!m_zmc->readJointState5D(state,err))return false;
    if(!state.allIdle){if(err)*err=tr("前一个五轴运动未结束");return false;}

    mls::calibration::Physical17Parameters model;model.value=m_rtcpParameters;
    Rtcp17Planner rtcp(m_rtcpOptions);
    // RTCP starts from measured five-axis MPOS. Non-RTCP uses the nominal target
    // cache when XYZ compensation is enabled, and the original DPOS otherwise.
    // Both paths are converted exactly once by the controller demand adapter.
    Joint previous{state.mpos.x,state.mpos.y,state.mpos.z,state.mpos.c,state.mpos.b};
    const auto startTcp=Rtcp17Model::controlledPointInMachine(previous,model,m_rtcpOptions.controlledPointSensorMm);
    Joint task{startTcp.x,startTcp.y,startTcp.z,previous[3],previous[4]};
    std::vector<PathPoint> path;std::string detail;m_batchBlocks=1;
    double nominalTaskSeconds=0,maxModelChordError=0;

    auto mask=[](const GCodeBlock& b)->std::uint8_t{return (b.hasX?1:0)|(b.hasY?2:0)|(b.hasZ?4:0)|(b.hasC?8:0)|(b.hasB?16:0);};
    auto resolveTask=[&](const PathPoint& words,std::uint8_t axisMask,bool absolute,const Joint& current,Joint& out)->bool{
        out=current;const double v[5]={words.x,words.y,words.z,words.c,words.b};
        for(int j=0;j<5;++j)if(axisMask&(1u<<j)){
            if(absolute){
                if(j<3 && !m_rtcpAbsoluteGaugeKnown && !(m_rtcpWorkOffsetMask&(1u<<j))){
                    detail="RTCP absolute XYZ requires an external translation gauge or G92 anchor for every addressed XYZ axis";return false;
                }
                const double off=(m_rtcpWorkOffsetMask&(1u<<j))?m_rtcpWorkOffsets[j]:0.0;
                out[j]=v[j]+off;
            }else out[j]=current[j]+v[j];
        }
        for(double x:out)if(!std::isfinite(x)){detail="RTCP task coordinate overflow";return false;}
        return true;
    };
    auto measure=[&](const Joint& a,const Joint& b){
        const double linear=std::hypot(std::hypot(b[0]-a[0],b[1]-a[1]),b[2]-a[2]);
        if(linear>1e-12)return linear;
        return std::max(std::abs(b[3]-a[3]),std::abs(b[4]-a[4]));
    };
    auto durationFor=[&](const Joint& a,const Joint& b,double feed,FeedMode mode,bool rapid,double& seconds)->bool{
        const double m=measure(a,b);if(m<=1e-12){seconds=0;return true;}
        if(!std::isfinite(feed)||feed<=0){detail="RTCP feed must be positive";return false;}
        if(!rapid && mode==FeedMode::InverseMinute){seconds=60.0/feed;return std::isfinite(seconds)&&seconds>0;}
        const double rate=feed/(mode==FeedMode::PerMinute?60.0:1.0);
        seconds=m/rate;
        if(!std::isfinite(seconds)||seconds<=0){detail="Invalid RTCP task duration";return false;}
        return true;
    };
    auto appendSegment=[&](const Joint& target,double seconds)->bool{
        if(seconds<=0){task=target;return true;}
        const Vec3d p{target[0],target[1],target[2]};
        auto plan=rtcp.planLineTimed(previous,p,target[3],target[4],seconds,model);
        if(!plan.success){detail=plan.message;return false;}
        if(path.size()+plan.waypoints.size()>200000){detail="RTCP generated more than 200000 controller segments";return false;}
        for(const auto& w:plan.waypoints){PathPoint q;q.x=w.q[0];q.y=w.q[1];q.z=w.q[2];q.c=w.q[3];q.b=w.q[4];q.axisMask=31;
            q.feed=w.controllerFeedPerSecond;q.feedMode=FeedMode::LegacyPerSecond;q.continuous=true;path.push_back(q);previous=w.q;}
        task=target;nominalTaskSeconds+=plan.nominalSeconds;maxModelChordError=std::max(maxModelChordError,plan.maxForwardErrorMm);
        return true;
    };
    auto lineBlock=[&](const GCodeBlock& b)->bool{
        if(b.feedRate)m_modalFeedSpeed=*b.feedRate;
        Joint target;if(!resolveTask(b.target,mask(b),b.absoluteModeSnapshot.value_or(m_absoluteMode),task,target))return false;
        const bool rapid=b.type==BlockType::G00_RapidMove;
        const double feed=rapid?m_program.motionParams.rapidSpeed:m_modalFeedSpeed;
        double seconds=0;if(!durationFor(task,target,feed,rapid?FeedMode::LegacyPerSecond:b.feedMode,rapid,seconds))return false;
        return appendSegment(target,seconds);
    };
    auto appendArc=[&](const GCodeBlock& b)->bool{
        if(b.feedRate)m_modalFeedSpeed=*b.feedRate;
        Joint end;if(!resolveTask(b.target,mask(b),b.absoluteModeSnapshot.value_or(m_absoluteMode),task,end))return false;
        mls::runtime::Arc arc;arc.clockwise=b.type==BlockType::G02_ArcCW;arc.plane=b.arc.plane==ArcPlane::XY?mls::runtime::Plane::XY:(b.arc.plane==ArcPlane::XZ?mls::runtime::Plane::ZX:mls::runtime::Plane::YZ);
        arc.centerOffset={b.arc.centerI,b.arc.centerJ,b.arc.centerK,0,0};arc.useRadius=b.arc.defineMode==ArcDefineMode::RadiusEnd;arc.radius=b.arc.radius;
        if(b.arc.defineMode==ArcDefineMode::ThreePoint){Joint mid=task;const bool absolute=b.absoluteModeSnapshot.value_or(m_absoluteMode);
            mid[0]=b.arc.midX+(absolute?((m_rtcpWorkOffsetMask&1)?m_rtcpWorkOffsets[0]:0.0):task[0]);
            mid[1]=b.arc.midY+(absolute?((m_rtcpWorkOffsetMask&2)?m_rtcpWorkOffsets[1]:0.0):task[1]);
            if(!threePointArc(task,mid,end,arc.clockwise,arc,detail))return false;
        }
        std::vector<Joint> points;if(!arcWaypoints(task,end,arc,points,detail))return false;
        if(points.empty()){detail="RTCP arc generated no task points";return false;}
        double totalMeasure=0;Joint a=task;for(const auto& q:points){totalMeasure+=measure(a,q);a=q;}
        if(totalMeasure<=1e-12){task=end;return true;}
        double totalSeconds=0;
        if(b.feedMode==FeedMode::InverseMinute){if(!std::isfinite(m_modalFeedSpeed)||m_modalFeedSpeed<=0){detail="G93 RTCP arc requires F";return false;}totalSeconds=60.0/m_modalFeedSpeed;}
        const double rate=(b.feedMode==FeedMode::PerMinute?m_modalFeedSpeed/60.0:m_modalFeedSpeed);
        a=task;
        for(const auto& q:points){double seconds=0;const double m=measure(a,q);
            if(b.feedMode==FeedMode::InverseMinute)seconds=totalSeconds*m/totalMeasure;
            else {if(!std::isfinite(rate)||rate<=0){detail="RTCP arc feed must be positive";return false;}seconds=m/rate;}
            if(!appendSegment(q,seconds))return false;a=q;
        }
        return true;
    };

    if(block.type==BlockType::G00_RapidMove || block.type==BlockType::G01_LinearInterp){
        if(!lineBlock(block)){if(err)*err=QString::fromStdString(detail);return false;}
        if(block.type==BlockType::G01_LinearInterp && !m_singleStep){
            for(int k=m_currentBlockIndex+1;k<int(m_program.blocks.size()) && m_batchBlocks<200000;++k){
                const auto& next=m_program.blocks[k];
                if(next.type!=BlockType::G01_LinearInterp || next.syncCapture!=block.syncCapture || !next.rtcpModeSnapshot.value_or(false))break;
                if(!lineBlock(next)){if(err)*err=QString::fromStdString(detail);return false;}++m_batchBlocks;
            }
        }
    }else if(block.type==BlockType::G05_ContinuousPath){
        if(block.feedRate)m_modalFeedSpeed=*block.feedRate;
        for(const auto& point:block.continuousPath.points){
            Joint target;if(!resolveTask(point,point.axisMask,block.absoluteModeSnapshot.value_or(m_absoluteMode),task,target)){if(err)*err=QString::fromStdString(detail);return false;}
            if(point.feed>0)m_modalFeedSpeed=point.feed;
            double seconds=0;if(!durationFor(task,target,m_modalFeedSpeed,point.feedMode,false,seconds)){if(err)*err=QString::fromStdString(detail);return false;}
            if(!appendSegment(target,seconds)){if(err)*err=QString::fromStdString(detail);return false;}
        }
    }else if(block.type==BlockType::G02_ArcCW || block.type==BlockType::G03_ArcCCW){
        if(!appendArc(block)){if(err)*err=QString::fromStdString(detail);return false;}
    }else{if(err)*err=tr("G43.4下不支持的运动类型");return false;}

    if(path.empty()){if(err)*err=tr("RTCP轨迹为空");return false;}
    AxisParams motion=m_program.motionParams.toAxisParams();
    if(!m_zmc->prepareMergePath(path,motion,err,ZmcController::PathStartReference::MeasuredMpos))return false;
    m_lastMotionTarget=previous;m_idleCheckCount=0;
    m_plannedMotionSeconds=m_zmc->trajectoryDurationSec();
    m_motionBudgetMs=mls::gcode::motionWatchdogBudgetMs(m_plannedMotionSeconds);
    if(m_motionBudgetMs<=0){if(err)*err=tr("RTCP控制器返回非法/溢出的规划时间");return false;}
    m_motionWatchdog.restart();m_idleOutOfToleranceSinceMs=-1;m_motionDiagnosticPhase=-1;
    bool sent=false;if(!m_zmc->pumpMergeBuffer(sent,err))return false;
    m_trajectoryDataSent=sent;if(!sent)m_mergeTimer.start();else m_mergeTimer.stop();
    emit progressMessage(tr("17参数RTCP：%1个XYZCB SP段；任务空间名义时间%2 s，控制器规划%3 s，模型弦误差上界%4 mm")
        .arg(path.size()).arg(nominalTaskSeconds,0,'f',3).arg(m_zmc->trajectoryDurationSec(),0,'f',3).arg(maxModelChordError,0,'g',6));
    return true;
}

bool GCodeExecutor::checkMotionComplete()
{
    QString err;
    if(!m_zmc || !m_zmc->checkMotionHealth5D(&err)){
        handleError(tr("五轴运动期间控制器报警/连接异常：%1").arg(err));return false;
    }
    const qint64 elapsed=m_motionWatchdog.elapsed();
    auto phaseLog=[this](int phase,const QString& status){
        if(m_motionDiagnosticPhase!=phase){
            m_motionDiagnosticPhase=phase;
            LOG_INFO(LogCategory::GCode,tr("五轴运动阶段：%1").arg(status));
        }
    };
    if(!m_trajectoryDataSent){
        m_idleCheckCount=0;
        m_idleOutOfToleranceSinceMs=-1;
        phaseLog(0,tr("控制器运动缓冲仍在填充"));
        if(elapsed>m_motionBudgetMs)
            handleError(tr("五轴缓冲填充超时：已用%1ms，预算%2ms，规划%3s；"
                           "需排查GetRemain_Buffer/MoveAbsSp及以太网/ECI调度，不是MPOS精度问题")
                .arg(elapsed).arg(m_motionBudgetMs).arg(m_plannedMotionSeconds,0,'f',3));
        return false;
    }
    JointState5D state;
    if(!m_zmc->readJointState5D(state,&err)){
        handleError(tr("五轴运动反馈读取失败：%1").arg(err));return false;
    }
    const std::array<double,5> actual{{state.mpos.x,state.mpos.y,state.mpos.z,state.mpos.c,state.mpos.b}};
    const std::array<double,5> target{{m_lastMotionTarget[0],m_lastMotionTarget[1],
                                        m_lastMotionTarget[2],m_lastMotionTarget[3],m_lastMotionTarget[4]}};
    const auto arrival=mls::gcode::observeArrival(actual,target);
    if(!arrival.valid){handleError(tr("五轴MPOS或目标反馈存在NaN/Inf"));return false;}
    const auto axes=[&](const std::array<double,5>& v){
        return tr("X=%1 Y=%2 Z=%3 C=%4 B=%5")
            .arg(v[0],0,'f',4).arg(v[1],0,'f',4).arg(v[2],0,'f',4)
            .arg(v[3],0,'f',4).arg(v[4],0,'f',4);
    };
    const QString detail=tr("目标[%1]；MPOS[%2]；DPOS[%3]；误差[%4]；allIdle=%5；"
                            "缓冲全部发送=%6；已用%7/%8ms；计划%9s")
        .arg(axes(target)).arg(axes(actual))
        .arg(axes({{state.dpos.x,state.dpos.y,state.dpos.z,state.dpos.c,state.dpos.b}}))
        .arg(axes(arrival.error)).arg(int(state.allIdle)).arg(int(m_trajectoryDataSent))
        .arg(elapsed).arg(m_motionBudgetMs).arg(m_plannedMotionSeconds,0,'f',3);
    if(!state.allIdle){
        phaseLog(1,tr("控制器报告仍在运动"));
        m_idleCheckCount=0;m_idleOutOfToleranceSinceMs=-1;
    }else if(!arrival.withinTolerance){
        phaseLog(2,tr("控制器已空闲但实际MPOS与目标不一致（停止后不应盲等）"));
        m_idleCheckCount=0;
        if(m_idleOutOfToleranceSinceMs<0)m_idleOutOfToleranceSinceMs=elapsed;
        if(elapsed-m_idleOutOfToleranceSinceMs>=mls::gcode::idleFeedbackSettleBudgetMs){
            handleError(tr("五轴已空闲但MPOS连续3秒未收敛，按机床0.05mm/0.05°到位要求拒绝继续：%1")
                .arg(detail));
            return false;
        }
    }else{
        phaseLog(3,tr("MPOS已收敛，正在确认连续稳定反馈"));
        m_idleOutOfToleranceSinceMs=-1;
        if(++m_idleCheckCount>=m_idleCheckThreshold)return true;
    }
    if(elapsed>m_motionBudgetMs){
        handleError(tr("五轴运动超时（%1）：%2")
            .arg(state.allIdle?tr("反馈未稳定"):tr("控制器仍未停止"),detail));
    }
    return false;
}


bool GCodeExecutor::checkAllAxesIdle()
{
    QString err;const bool idle=m_zmc && m_zmc->areAllAxesIdle(&err);
    if(!err.isEmpty())handleError(err);return idle;
}


// ==================== 程序控制 ====================

void GCodeExecutor::advanceToNextBlock()
{
    for(int k=0;k<m_batchBlocks;++k)emit blockCompleted(m_currentBlockIndex+k);
    m_currentBlockIndex+=m_batchBlocks;m_batchBlocks=1;
    m_idleCheckCount = 0;
    
    if (m_currentBlockIndex >= static_cast<int>(m_program.blocks.size())) {
        finishProgram();
    } else if (m_singleStep) {
        m_singleStep=false;m_resumeState=ExecutorState::Running;m_pauseCompletesBlock=false;
        transitionTo(ExecutorState::Paused);m_stateMachineTimer.stop();
        emit programPaused(tr("单步段已完整执行；下一次从下一段开始"));
    } else {
        transitionTo(ExecutorState::Running);
    }
}

void GCodeExecutor::finishProgram()
{
    // 停止所有定时器
    m_stateMachineTimer.stop();
    m_motionCheckTimer.stop();
    m_mergeTimer.stop();
    
    // 确保采集已停止
    if (m_syncActive) {
        QString error;
        if (!stopSyncCapture(&error)) { handleError(error); return; }
        emit syncStateChanged(false);
    }
    if(m_state==ExecutorState::Error)return;
    if (!m_pendingSaveTasks.isEmpty()) {
        m_finishing=true;transitionTo(ExecutorState::WaitingSaveComplete);
        m_saveCheckTimer.start();onSaveCheckTimer();return;
    }
    m_finishing=false;
    transitionTo(ExecutorState::Completed);
    
    auto stats = AsyncSaveManager::instance().getStatistics();
    emit progressMessage(tr("程序执行完成\n"
                           "耗时: %1 秒\n"
                           "保存文件: %2 个")
        .arg(m_elapsedTimer.elapsed() / 1000.0, 0, 'f', 1)
        .arg(stats.totalTasksCompleted));
    
    emit programCompleted();
}

void GCodeExecutor::handleError(const QString& msg)
{
    if(m_handlingError)return;m_handlingError=true;
    LOG_ERROR(LogCategory::GCode,tr("执行错误: %1").arg(msg));
    transitionTo(ExecutorState::Error);
    m_stateMachineTimer.stop();m_motionCheckTimer.stop();m_mergeTimer.stop();m_dwellTimer.stop();m_saveCheckTimer.stop();
    if(m_zmc && m_zmc->isConnected())m_zmc->requestStopAllAxes(false);
    if(m_syncMgr && m_syncMgr->isRunning())m_syncMgr->stop();
    m_syncActive=false;m_controllerHeld=false;emit syncStateChanged(false);emit errorOccurred(msg);m_handlingError=false;
}


// ==================== 保存相关 ====================

QString GCodeExecutor::generateSaveFilePath(const GCodeBlock& block)
{
    QString blockName = block.name;
    if (blockName.isEmpty()) {
        blockName = QString("Block%1").arg(block.lineNumber);
    }
    
    return m_currentSaveParams.generateFilePath(blockName);
}

void GCodeExecutor::updateProgress()
{
    emit progressUpdated(getProgress());
}
