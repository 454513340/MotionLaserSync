#pragma once

#include <QObject>
#include <QTimer>
#include <QElapsedTimer>
#include <QStringList>
#include <atomic>
#include <memory>
#include <functional>
#include <utility>
#include <QByteArray>

#include "gcode/GCodeTypes.h"
#include "gcode/HardwarePreflightLogic.h"
#include "core/DataTypes.h"
#include "runtime/Rtcp17.h"

// 前向声明
class ZmcController;
class LaserSensor;
class SyncManager;
class PointCloudBuilder;

/**
 * @file GCodeExecutor.h
 * @brief G代码执行引擎
 * 
 * 状态机驱动的G代码执行器，实现：
 * - 顺序执行程序段
 * - 自动运动完成检测
 * - 同步采集的自动开启/关闭
 * - 点云数据的异步保存
 */

class GCodeExecutor : public QObject
{
    Q_OBJECT

public:
    explicit GCodeExecutor(ZmcController* zmc,
                           LaserSensor* laser,
                           SyncManager* syncMgr,
                           PointCloudBuilder* cloudBuilder,
                           QObject* parent = nullptr);
    ~GCodeExecutor() override;

    // ==================== 程序管理 ====================
    
    /**
     * @brief 加载G代码程序
     */
    bool loadProgram(const GCodeProgram& program, QString* err = nullptr);
    
    /**
     * @brief 从文件加载程序
     */
    bool loadFromFile(const QString& filePath, QString* err = nullptr);
    
    /**
     * @brief 从文本加载程序
     */
    bool loadFromText(const QString& text, QString* err = nullptr);
    
    /**
     * @brief 卸载程序
     */
    void unloadProgram();
    
    /**
     * @brief 是否有程序加载
     */
    bool hasProgram() const { return !m_program.blocks.empty(); }
    
    /**
     * @brief 获取当前程序
     */
    const GCodeProgram& program() const { return m_program; }

    // ==================== 执行控制 ====================
    
    /**
     * @brief 开始执行（从头）
     */
    /**
     * Read-only on-site eligibility review. It cannot enable any offline-only
     * program or provide an execution permit. No axis, IO, PWM or laser commands.
     * Reports current machine pose only through existing low-frequency readers.
     */
    struct RealRunPreflightReport {
        QStringList blockers;
        QStringList requiredActions;
        QStringList observations;
        QString ncSha256;
        bool allCheckedConditionsClear() const { return blockers.isEmpty(); }
    };
    // Values shown in the Parameters panel; never call these a physical read-back.
    struct HardwareSyncUiIntent {
        bool available = false;
        mls::gcode::preflight::HardwareSetting value;
        bool manualCaptureHardwareSelected = false; // separate from an NC's SyncMode
    };
    RealRunPreflightReport inspectRealRunReadOnly(
        const GCodeProgram& program,
        const QString& ncText,
        const QString& originalPath,
        const HardwareSyncUiIntent& uiIntent);
    RealRunPreflightReport inspectRealRunReadOnly(
        const GCodeProgram& program,
        const QString& ncText,
        const QString& originalPath = QString()) {
        return inspectRealRunReadOnly(program,ncText,originalPath,HardwareSyncUiIntent{});
    }

    // A TRUSTED on-site component must verify measured synchronization,
    // a complete Machine5X FCL audit of the EXACT NC, current active model,
    // machine health, independent physical interlocks and the live part fixture.
    // Default unset = FAIL CLOSED. Never return true from NC comments, stale
    // cache/GUI values, or a single operator confirmation checkbox.
    using SiteHardwareNcVerifier = std::function<bool(
        const GCodeProgram&, const QByteArray& ncSha256,
        QString* failure)>;
    void setSiteHardwareNcVerifier(SiteHardwareNcVerifier verifier) {
        m_siteHardwareNcVerifier = std::move(verifier);
    }
    // V30 integrated diagnostics may evaluate the EXACT same commissioned
    // site evidence before Start without motion. Start ALWAYS rechecks live
    // MPOS, PLC, TABLE and the source program itself; this is not a cached permit.
    bool inspectSiteExecutionQualificationReadOnly(
        const GCodeProgram& program,const QByteArray& rawDiskDigest,QString* failure) const {
        if(!m_siteHardwareNcVerifier){
            if(failure)*failure=tr("尚未安装独立现场安全与夹具状态验证器");
            return false;
        }
        return m_siteHardwareNcVerifier(program,rawDiskDigest,failure);
    }

    // An initial G00 is a movement command, not an assertion that the machine
    // was ALREADY at its destination when the NC was imported. If a physical
    // move is needed, a trusted independent on-site validator must approve the
    // exact LIVE-MPOS -> G00 path for the signed NC. No UI toggle can do this.
    using SiteEntryMoveVerifier = std::function<bool(
        const JointPose5D& live,
        const JointPose5D& firstG00Target,
        const QByteArray& ncSha256,
        QString* failure)>;
    void setSiteEntryMoveVerifier(SiteEntryMoveVerifier verifier) {
        m_siteEntryMoveVerifier = std::move(verifier);
    }

    bool start(QString* err = nullptr);
    
    /**
     * @brief 从暂停处继续
     */
    bool resume(QString* err = nullptr);
    
    /**
     * @brief 暂停执行
     */
    void pause();
    
    /**
     * @brief 停止执行
     */
    void stop();
    
    /**
     * @brief 单步执行
     */
    bool stepOnce(QString* err = nullptr);

    // ==================== 状态查询 ====================
    
    ExecutorState state() const { return m_state; }
    QString stateText() const { return GCodeUtils::stateName(m_state); }
    
    int currentBlockIndex() const { return m_currentBlockIndex; }
    int totalBlocks() const { return static_cast<int>(m_program.blocks.size()); }
    
    const GCodeBlock* currentBlock() const;
    
    bool isRunning() const;
    bool isPaused() const { return m_state == ExecutorState::Paused; }
    bool isSyncActive() const { return m_syncActive; }
    
    /**
     * @brief 获取执行进度
     */
    ExecutorProgress getProgress() const;

    // ==================== 设置 ====================
    
    /** Configure the exact same physical17/hand-eye model used by scan reconstruction. */
    bool setRtcpKinematicModel(const std::array<double,17>& parameters,
                               bool absoluteGaugeKnown=false,
                               QString* err=nullptr);
    bool hasRtcpKinematicModel() const { return m_rtcpModelConfigured; }
    bool clearRtcpKinematicModel(QString* err=nullptr);
    void setRtcpOptions(const mls::runtime::Rtcp17Options& options) { m_rtcpOptions=options; }

    /**
     * @brief 设置运动完成检测的稳定次数
     */
    void setIdleCheckThreshold(int count) { m_idleCheckThreshold = count; }
    
    /**
     * @brief 设置状态机定时器间隔
     */
    void setStateMachineInterval(int ms);

signals:
    /**
     * @brief 状态改变
     */
    void stateChanged(ExecutorState newState, ExecutorState oldState);
    
    /**
     * @brief 程序段开始执行
     */
    void blockStarted(int index, const GCodeBlock& block);
    
    /**
     * @brief 程序段执行完成
     */
    void blockCompleted(int index);
    
    /**
     * @brief 采集状态改变
     */
    void syncStateChanged(bool active);
    
    /**
     * @brief 程序执行完成
     */
    void programCompleted();
    
    /**
     * @brief 程序被停止
     */
    void programStopped();
    
    /**
     * @brief 程序暂停（M00或用户暂停）
     */
    void programPaused(const QString& reason);
    
    /**
     * @brief 错误发生
     */
    void errorOccurred(const QString& message);
    
    /**
     * @brief 警告消息
     */
    void warningOccurred(const QString& message);
    
    /**
     * @brief 进度消息
     */
    void progressMessage(const QString& message);
    
    /**
     * @brief 进度更新
     */
    void progressUpdated(const ExecutorProgress& progress);

private slots:
    void onStateMachineTimer();
    void onMotionCheckTimer();
    void onSaveCheckTimer();
    void onDwellTimer();

private:
    // 状态机处理
    void processStateMachine();
    void transitionTo(ExecutorState newState);
    
    // 程序段执行
    void executeCurrentBlock();
    void executeG00RapidMove(const GCodeBlock& block);
    void executeG01Linear(const GCodeBlock& block);
    void executeG02G03Arc(const GCodeBlock& block, bool clockwise);
    void executeG04Dwell(const GCodeBlock& block);
    void executeG05ContinuousPath(const GCodeBlock& block);
    void executeG90G91Mode(const GCodeBlock& block);
    void executeG92SetPosition(const GCodeBlock& block);
    void executeM50SyncOn();
    void executeM51SyncOff();
    void executeM60SavePointCloud(const GCodeBlock& block);
    void executeM61ClearPointCloud();
    void executeM62SetSavePath(const GCodeBlock& block);
    void executeM63WaitSave();
    void executeM00ProgramStop(const GCodeBlock& block);
    void executeM30ProgramEnd();
    
    // 同步采集控制
    bool startSyncCapture(QString* err = nullptr);
    bool stopSyncCapture(QString* err = nullptr);
    void waitSyncReady();
    
    // 运动控制
    // Read-only dry run of the real five-axis G93 SP planner against current
    // configured axis bounds. Called both in diagnostic step 1 and again
    // before the first physical G00; never issues SDK motion commands.
    bool validateG93DynamicsReadOnly(const GCodeProgram& program,
                                     const QString& ncText,
                                     QString* error,
                                     QString* summary = nullptr) const;
    bool startMotion(const GCodeBlock& block, QString* err = nullptr);
    bool startRtcpMotion(const GCodeBlock& block, QString* err = nullptr);
    bool checkMotionComplete();
    bool checkAllAxesIdle();
    
    // 程序控制
    void advanceToNextBlock();
    void finishProgram();
    void handleError(const QString& msg);
    
    // 保存相关
    QString generateSaveFilePath(const GCodeBlock& block);
    void updateProgress();

private:
    // 外部组件引用
    ZmcController* m_zmc = nullptr;
    LaserSensor* m_laser = nullptr;
    SyncManager* m_syncMgr = nullptr;
    SiteHardwareNcVerifier m_siteHardwareNcVerifier;
    SiteEntryMoveVerifier m_siteEntryMoveVerifier;
    // Recheck the first live G00 against the MPOS for which entry was approved.
    bool m_firstMoveEntryQualified = false;
    JointPose5D m_firstMoveEntryLivePose{};
    QByteArray m_firstMoveApprovedNcDigest;
    PointCloudBuilder* m_cloudBuilder = nullptr;

    // 程序数据
    GCodeProgram m_program;
    int m_currentBlockIndex = 0;
    
    // 执行状态
    ExecutorState m_state = ExecutorState::Idle;
    ExecutorState m_previousState = ExecutorState::Idle;
    bool m_syncActive = false;
    bool m_absoluteMode = true;
    
    // 模态参数 (Modal parameters - 一旦设置保持有效直到被覆盖)
    double m_modalFeedSpeed = 0.0;   // 当前模态进给速度（F参数），0表示使用默认值

    mls::runtime::Joint m_workOffsets{};
    mls::runtime::Joint m_rtcpWorkOffsets{}; // XYZ = controlled-point work frame; C/B = mechanical rotary work offsets
    std::uint8_t m_rtcpWorkOffsetMask=0;
    std::array<double,17> m_rtcpParameters{};
    mls::runtime::Rtcp17Options m_rtcpOptions{};
    bool m_rtcpModelConfigured=false;
    bool m_rtcpAbsoluteGaugeKnown=false;
    mls::runtime::Joint m_lastMotionTarget{};
    int m_batchBlocks=1;
    bool m_singleStep=false, m_finishing=false;
    bool m_pauseCompletesBlock=false, m_controllerHeld=false, m_handlingError=false;
    ExecutorState m_resumeState=ExecutorState::Ready;
    QElapsedTimer m_motionWatchdog;
    qint64 m_motionBudgetMs=0;
    double m_plannedMotionSeconds=0.0;
    qint64 m_idleOutOfToleranceSinceMs=-1;
    int m_motionDiagnosticPhase=-1;

    // 保存状态
    SaveParams m_currentSaveParams;
    QStringList m_pendingSaveTasks;
    
    // 运动完成检测
    int m_idleCheckCount = 0;
    int m_idleCheckThreshold = 3;
    bool m_trajectoryDataSent = false;
    
    // 暂停/延时
    int m_dwellRemainingMs = 0;
    QString m_pauseReason;
    
    // 定时器
    QTimer m_stateMachineTimer;     // 状态机驱动 (10ms)
    QTimer m_motionCheckTimer;      // 运动检测 (20ms)
    QTimer m_saveCheckTimer;        // 保存检测 (100ms)
    QTimer m_dwellTimer;            // 延时定时器 (单次)
    
    // 计时
    QElapsedTimer m_elapsedTimer;
    QElapsedTimer m_blockTimer;
    
    // 连续插补控制
    QTimer m_mergeTimer;            // 连续插补填充定时器
    
    // 进度统计
    mutable ExecutorProgress m_progress;
};
