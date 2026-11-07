#pragma once

#include <QObject>
#include <QString>
#include <QVector>
#include <QMutex>
#include <array>
#include <atomic>
#include <cstdint>
#include <cmath>
#include <limits>
#include <vector>

#include "core/DataTypes.h"
#include "core/ControllerIoGate.h"
#include "core/EciIoScheduler.h"
#include "runtime/AxisCompensation.h"

// ZAux 厂家SDK头文件（类名保留历史名称，硬件为ECI3808）
#include "zauxdll2.h"

/**
 * @brief ECI3808 PC SDK 封装（保留 V17 类名与 SDK）
 * 
 * 封装 ECI3808 控制卡的连接、轴参数、单轴运动、多轴插补和位置采样功能。
 * V19: 连接后的 SDK 事务由单一 EciIoScheduler 线程执行，支持优先级/软截止期/背压/代际取消。
 * 已验证的 OpenEth 留在 GUI 线程，只在关闭请求入口、排空旧连接后执行；其余 I/O 在调度线程。
 * 运动等待、15ms 启动握手和 RS485 留在调用线程，禁止占用 I/O 线程。
 * 软实时且事务不可抢占，不能替代硬件急停，也不保证 DLL 响应上界。
 */
class ZmcController : public QObject
{
    Q_OBJECT

public:
    // 轴任务类型
    enum AxisTask {
        AxisTaskNone = 0,
        AxisTaskHome = 1,
        AxisTaskMoveAbs = 2,
        AxisTaskMoveRel = 3,
        AxisTaskJointMove5D = 4,
    };

    explicit ZmcController(QObject* parent = nullptr);
    ~ZmcController() override;

    // ==================== 连接管理 ====================
    
    /**
     * @brief 通过 TCP 连接到控制器
     * @param ip 控制器 IP 地址
     * @param port 端口号（默认 777）
     * @param err 错误信息输出
     * @return 成功返回 true
     */
    bool connectTcp(const QString& ip, int port = 777, QString* err = nullptr);
    
    /**
     * @brief 断开连接
     */
    void disconnect();
    
    /**
     * @brief 检查是否已连接
     */
    bool isConnected() const { return m_connected.load(); }
    
    // V19: no public raw handle; all connected SDK I/O goes through the scheduler.
    eci::EciIoScheduler::Stats ioStatistics() const { return m_io.stats(); }
    bool requestStopAxis(int axis, bool emergency = false);
    bool requestStopJointMove5D(bool emergency = false);
    bool requestStopAllAxes(bool emergency = false);

    // ==================== 轴状态查询 ====================
    
    /**
     * @brief 查询某轴是否正在回零
     */
    bool isAxisHoming(int axis) const;
    
    /**
     * @brief 查询某轴是否有长任务在执行
     */
    bool isAxisBusy(int axis) const;

    // ==================== 轴参数读写 ====================
    
    bool readAxisParams(int axis, AxisParams& out, QString* err = nullptr);
    bool writeAxisParams(int axis, const AxisParams& in, QString* err = nullptr);
    bool writeMotionParams(int axis, const AxisParams& in, QString* err = nullptr);  // 只写运动参数，不含软限位
    bool readAllAxisParams(AxisParams (&params)[Constants::kAxisParamArraySize], QString* err = nullptr);
    bool writeAllAxisParams(const AxisParams (&params)[Constants::kAxisParamArraySize], QString* err = nullptr);

    // ==================== 位置读取 ====================
    
    /**
     * @brief 读取三轴光栅反馈当前位置（MPOS）
     * @note XYZ 为增量光栅；正规 DATUM 回零成功后 homeAxis() 会用 SetMpos(0)
     *       建立机床零点。回零前 MPOS 数值不应解释为有效机床绝对坐标。
     *       此函数标记为 const，可在采样线程中安全调用。
     */
    bool readAxisPositions(double& x, double& y, double& z, QString* errMsg = nullptr) const;

    /**
     * @brief 读取当前五轴机械关节实际坐标，用于运动学/标定等低频场景。
     * @note XYZ 读取增量光栅 MPOS；B/C 读取 absolute-ready 后的软件机械 MPOS。
     *       此函数会产生多次控制器通信，不得用于在线扫描高频轮询。
     */
    bool readJointPose5D(JointPose5D& pose, QString* errMsg = nullptr);

    /**
     * @brief 低频读取五轴 DPOS/MPOS 机械坐标与整体空闲状态。
     * @note P4 五轴同步定位前/后使用。在线扫描期间禁止高频调用。
     */
    bool readJointState5D(JointState5D& state, QString* errMsg = nullptr);
    
    /**
     * @brief 获取单轴状态（位置和空闲状态）
     */
    bool getAxisStatus(int axis, float& pos, bool& idle, QString* err = nullptr);

    /**
     * @brief 同时读取轴的 DPOS、MPOS 与空闲状态。
     * @note XYZ 返回控制器原始坐标；B/C 在 absolute-ready 后返回上位机机械坐标
     *       （controller + software-frame offset），不会修改控制器内部坐标。
     */
    bool readAxisUiPosition(int axis, float& dpos, float& mpos, bool& idle, QString* err = nullptr);
    bool readAxisType(int axis, int& atype, QString* err = nullptr);
    bool getAxisPositionPair(int axis, float& dpos, float& mpos, bool& idle,
                             QString* err = nullptr);

    /**
     * @brief 读取控制器内部编码器值。
     * @note ZMotion SDK 将 ENCODER 定义为内部编码器值；用于诊断 MPOS 坐标重基准与
     *       实际反馈计数是否同时发生跳变。该接口只读，不修改控制器状态。
     */
    bool readAxisEncoder(int axis, float& encoder, QString* err = nullptr) const;

    /**
     * @brief 当前连接会话中，上位机对 XYZ 成功调用 SetMpos(home-zero) 的次数。
     * @note 仅统计本程序发起的 SetMpos；用于证明普通过零运动期间上位机是否再次改写 MPOS。
     */
    std::uint64_t linearMposSetGeneration(int axis) const;

    /**
     * @brief 读取某轴当前 DATUM_IN 配置以及该数字输入的原始电平。
     * @note 仅用于低频诊断/实验记录；不会修改控制器参数。
     */
    bool readAxisDatumInputState(int axis, int& inputNumber, int& rawValue,
                                 QString* err = nullptr) const;

    /**
     * @brief 读取 AXISSTATUS 32 位状态字。
     * @note 用于持续监视硬限位、软限位、伺服报警、跟随误差等状态。
     */
    bool getAxisStatusWord(int axis, int& statusWord, QString* err = nullptr) const;

    /**
     * @brief 将 AXISSTATUS 状态字转换为可读文本。
     */
    static QString describeAxisStatusWord(int statusWord);

    /**
     * @brief 判断 AXISSTATUS 中是否包含需要报警提示的状态位。
     * @note 找原点、HOLD、CANCEL、暂停等运行状态不计为“报警”，但仍会显示。
     */
    static bool axisStatusHasAlarm(int statusWord);

    // ==================== B/C 绝对位置与人工使能 ====================

    /**
     * @brief 通过 RS485 读取禾川绝对值伺服位置，并建立 C/B 的上位机机械绝对坐标系。
     * @note V15/V16 不再写控制器 MPOS/DPOS/OFFPOS。实机已经证明这些坐标写操作在控制器不断电、
     *       仅上位机断开/重连时会累计坐标偏移。现在只读取 P21.32/P21.34，按已标定的
     *       23bit + C10:1/B100:1 计算机械角度，然后保存 machine = controller + offset 的映射。
     *       B/C 单轴绝对定位、相对定位、回 0°、位置显示和软限位均经过该映射转换。
     *       如果连接时发现控制器 DPOS-MPOS 已被旧版本污染为较大差值，则拒绝 ready/使能，
     *       要求先重新上电 ZMotion 控制器一次清除旧坐标偏移，绝不再自动叠加坐标写操作。
     */
    bool initializeRotaryAxesFromAbsolute(double& cAxisDeg, double& bAxisDeg,
                                          QString* err = nullptr);

    /**
     * @brief 人工设置 B/C 伺服使能。B=OP(24)，C=OP(26)。
     * @note enable=true 时必须已经完成绝对位置同步，否则拒绝操作。
     */
    bool setRotaryServoEnable(int axis, bool enable, QString* err = nullptr);

    bool isRotaryAbsoluteReady(int axis) const;
    bool isRotaryServoEnabled(int axis) const;

    /**
     * @brief XYZ增量光栅是否已建立正规 DATUM/MPOS 机床基准。
     * @note V32.13 会从控制器 TABLE[357] 的本次上电会话 bit 恢复；
     *       控制器/BASIC 重启会清零，只有 homeAxis() 完整成功才重新置位。
     */
    bool isLinearMposReferenced(int axis) const;
    bool areLinearMposReferenced() const;

    /**
     * @brief 五轴“本次控制器上电会话已完成回零”状态。
     * @note X/Y/Z 对应 DATUM+MPOS 零点；C/B 对应在绝对机械坐标就绪后成功回到 0°。
     *       C/B 的 Home bit 不能替代绝对编码器重读；isAxisHomed(C/B) 只有在
     *       absolute-ready 重新建立后才返回 true。
     */
    bool isAxisHomed(int axis) const;
    bool areAllAxesHomed() const;

    /**
     * @brief 获取 B/C 上位机机械坐标偏移 machine = controller + offset。
     */
    double rotaryMachineOffset(int axis) const;

    // V27.6.36: XYZ physical-target -> controller-demand compensation with MachineSafety coupling.
    // Loading is atomic and leaves compensation disabled. Enabling requires
    // this connection's XYZ DATUM, idle axes and no hardware/trajectory/task owner.
    bool installAxisCompensation(const mls::axiscomp::Tables& tables,
                                 const mls::axiscomp::Options& options, QString* err = nullptr);
    bool setAxisCompensationEnabled(bool enabled, QString* err = nullptr);
    bool axisCompensationEnabled() const { return m_axisCompEnabled.load(); }
    QString axisCompensationStatus(QString* err = nullptr) const;
    // Conservative L1 envelope of the loaded XYZ demand corrections.  Used by
    // MachineSafety to prove that its full-path clearance margin dominates the
    // extra geometric uncertainty introduced by command-space compensation.
    bool axisCompensationSafetyEnvelopeMm(double& envelopeMm, QString* err = nullptr) const;
    // Return the exact per-axis motion parameters that moveJointAbs5DIndependent()
    // will realize after the compensation acquisition-speed cap is applied.
    // The collision predictor must use these parameters, otherwise asynchronous
    // XYZ/C/B timing can diverge from the path checked by MachineSafety.
    bool effectiveIndependentAxisParams5D(
        const std::array<AxisParams, Constants::kAxisParamArraySize>& requested,
        double speedScale,
        std::array<AxisParams, Constants::kAxisParamArraySize>& effective,
        QString* err = nullptr) const;
    bool readMotionStartPose5D(JointPose5D& nominal, QString* err = nullptr);
    // Explicit RAW command-space APIs: calibration, A/B validation and DPOS diagnostics ONLY.
    // Do not use these for production physical targets or an already compensated value
    // with moveAbs(). Raw movement invalidates the nominal-target cache.
    bool moveAbsRawDpos(int axis, float dpos, const AxisParams* motion = nullptr, QString* err = nullptr);
    bool moveRelRawDpos(int axis, float delta, const AxisParams* motion = nullptr, QString* err = nullptr);

    // ==================== 单轴运动 ====================
    
    /**
     * @brief 单轴回零（阻塞函数，应在工作线程调用）
     */
    bool homeAxis(int axis, const HomeParams& hp, const AxisParams& motion, QString* err = nullptr);

    /**
     * @brief B/C 回零到机械绝对 0°，并维护控制器 TABLE[357] 的 C/B Home bit。
     * @note 开始运动前先清旧 bit；只有 moveAbs(0°) 成功并通过位置回读后才重新置位。
     */
    bool homeRotaryAxisToZero(int axis, const AxisParams& motion, QString* err = nullptr);
    
    /**
     * @brief 绝对定位（阻塞函数，工作线程调用）
     * XYZ runtime ON: pos is desired physical MPOS; OFF: legacy DPOS command.
     * B/C retain software mechanical absolute coordinates in both modes.
     */
    bool moveAbs(int axis, float pos, const AxisParams* motion = nullptr, QString* err = nullptr);

    /**
     * Absolute X/Y/Z move specified in measured machine MPOS (not raw DPOS).
     * Executes ONE axis only. With XYZ compensation ON, reuse the calibrated
     * mapping; with it OFF, map the requested MPOS displacement to current DPOS.
     * The caller must independently check the whole motion with MachineSafety.
     * Blocking operation; call from a worker, never the UI or ECI IO worker.
     */
    bool moveLinearMposAbs(int axis, float machineMpos, const AxisParams* motion = nullptr,
                           QString* err = nullptr);
    
    /**
     * @brief 相对定位（阻塞函数，工作线程调用）
     * XYZ runtime ON: desired = current measured MPOS + delta, map once.
     * OFF: legacy controller-demand relative move.
     */
    bool moveRel(int axis, float delta, const AxisParams* motion = nullptr, QString* err = nullptr);
    
    /**
     * @brief 点动
     * @param velocityMode true 为原始持续速度点动；false 为补偿相对定位（阻塞，工作线程调用）
     */
    bool jog(int axis, bool positive, const AxisParams& motion, 
             bool velocityMode, float step, QString* err = nullptr);
    
    /**
     * @brief 停止轴运动
     * @param emergency true 为急停，false 为减速停止
     */
    bool stopAxis(int axis, bool emergency, QString* err = nullptr);
    
    /**
     * @brief 历史接口：禁止任意清零 X/Y/Z DPOS。正规 XYZ 回零由 homeAxis()
     *        在 DATUM 成功后同时建立 DPOS=0 与光栅 MPOS=0 的机床坐标基准。
     */
    bool zeroAllDpos(QString* err = nullptr);

    // ==================== P4 五轴关节空间同步定位 ====================

    /**
     * @brief 执行已经规划/校验过的一段五轴关节空间绝对同步定位。
     *
     * target 使用上位机机械坐标；B/C 在函数内部自动转换为控制器原始坐标。
     * 函数会再次检查 B/C absoluteReady/人工使能、五轴空闲、AXISSTATUS、
     * 控制器实际软限位，并使用 ZAux_Direct_MoveAbs 一次发送五轴目标。
     *
     * @note 阻塞函数，应在工作线程调用；扫描过程中不应调用。
     */
    bool moveJointAbs5D(const JointMotionPlan5D& plan,
                        JointMoveResult5D& result,
                        QString* err = nullptr);

    /**
     * @brief 标定姿态之间的五轴独立绝对定位（非插补）。
     *
     * X/Y/Z/C/B 各轴分别使用自身 AxisParams 的 SPEED/ACCEL/DECEL * speedScale，
     * 并在同一控制器事务中分别下发 Single_MoveAbs。各轴允许先后到达目标；函数仅在
     * 五轴全部停稳后返回。为使独立轨迹可预测，定位期间 LSPEED=0、SRAMP=0。
     *
     * 该接口专用于静态标定姿态切换；连续扫描/P4人工同步定位仍使用 moveJointAbs5D()。
     */
    bool moveJointAbs5DIndependent(
        const JointPose5D& target,
        const std::array<AxisParams, Constants::kAxisParamArraySize>& axisParams,
        double speedScale,
        JointMoveResult5D& result,
        QString* err = nullptr);

    /**
     * @brief 停止当前五轴定位（同步插补或标定独立轴定位）。
     * @param emergency false=按 FASTDEC 取消当前/缓冲运动；true=立即中断脉冲。
     */
    bool stopJointMove5D(bool emergency, QString* err = nullptr);

    /**
     * @brief 判断 X/Y/Z/C/B 五个关节轴是否全部停止。
     * @note 仅做事件式检查，不用于周期轮询。
     */
    bool areFiveJointAxesIdle(QString* err = nullptr) const;

    // ==================== 原三轴多轴插补（P4 暂不改动） ====================
    
    /**
     * @brief 三轴直线绝对定位
     */
    bool moveLineAbs(const PathPoint& target, const AxisParams& motion, QString* err = nullptr);
    // Read-only snapshot of the software trajectory bounds actually used by
    // prepareMergePath(). These are distinct from the controller soft limits;
    // the live SDK still checks those again before any motion is sent.
    bool readTrajectoryLimits5D(mls::runtime::Limits& out, QString* err = nullptr);
    bool setTrajectoryLimits5D(const mls::runtime::Limits& limits, QString* err = nullptr);
    bool pauseTrajectory5D(QString* err = nullptr);
    double trajectoryDurationSec() const { return m_trajectorySeconds.load(); }
    bool checkMotionHealth5D(QString* err = nullptr);
    bool resumeTrajectory5D(QString* err = nullptr);
    bool readJointSample5D(SamplePoint& sample, QString* err = nullptr);
    // Startup probe is produced after all five continuous REGIST channels are armed.
    // This call switches BASIC from one-shot probe to the currently configured runtime PWM frequency.
    bool hwTableContinueAfterProbe(QString* err = nullptr);
    
    /**
     * @brief 准备连续插补轨迹
     */
    enum class PathStartReference { NominalTarget, MeasuredMpos };
    bool prepareMergePath(const std::vector<PathPoint>& path,
                          const AxisParams& motion, QString* err = nullptr,
                          PathStartReference startReference = PathStartReference::NominalTarget);
    
    /**
     * @brief 定时调用，向控制器缓冲区填充轨迹点
     * @param finished 输出参数，表示轨迹是否已全部发送
     */
    bool pumpMergeBuffer(bool& finished, QString* err = nullptr);

    // ==================== 软件位置采样 ====================
    
    /**
     * @brief 采集位置流（阻塞函数，用于测试）
     * @param periodUs 采样周期（微秒）
     * @param sampleCount 采样数量
     * @param out 输出采样点
     */
    bool capturePositionStream(int periodUs, int sampleCount,
                               QVector<SamplePoint>& out, QString* errMsg = nullptr) const;

    // ==================== 硬件 TABLE 采样 ====================
    
    /**
     * @brief 启动硬件 TABLE 位置采样
     * 需要控制器内运行相应的 ZBasic 程序
     */
    bool hwTableSampleStart(QString* errMsg = nullptr);
    bool hwTableWaitCmmArmed(int timeoutMs=1200,QString* errMsg=nullptr);

    /**
     * @brief 设置硬件同步PWM频率。仅允许BASIC空闲时修改；BASIC回写applied值后才返回成功。
     * @note 上位机还会按scanCONTROL当前Measuring Field进一步限制频率；控制器侧只做25~2000Hz协议保护。
     */
    bool setHardwareTriggerFrequencyHz(int frequencyHz, QString* errMsg = nullptr);
    bool setHardwareAcquisitionConfig(int frequencyHz, mls::runtime::HardwareSyncTopology topology,
                                      int cmmR1Edge, QString* errMsg = nullptr);
    bool readHardwareTriggerFrequencyHz(int& requestedHz, int& appliedHz, QString* errMsg = nullptr) const;
    /** Wait for BASIC to finish the real PWM stop-tail and FIFO drain (TABLE[358]=IdleReady).
     * Do not use while the acquisition reader is still expecting tail records; call after drain.
     */
    bool hwTableWaitIdleReady(int timeoutMs = 1500, QString* errMsg = nullptr);


    /**
     * @brief 冷启动传感器数据链预热：BASIC enable=3，使用当前运行时PWM频率触发scanCONTROL；正式session随后清空/重建FIFO。
     * 不产生正式REGIST位姿、不写Buffer-B。仅在正式session之前使用。
     */
    bool hwSensorColdPrimeStart(QString* errMsg = nullptr);
    bool hwSensorColdPrimeStop(QString* errMsg = nullptr);
    bool hwSensorColdPrimeCount(qint64& pulseCount, QString* errMsg = nullptr);

    /**
     * @brief 只读检查当前ECI3808 BASIC硬件同步协议头。
     * 不写TABLE、不启动PWM、不产生触发，也不下发运动。
     */
    bool hwTableCommissioningPreflight(QString& summary, QString* errMsg = nullptr, bool* executionAuthorized = nullptr);
    
    /**
     * @brief 停止硬件 TABLE 位置采样
     */
    bool hwTableSampleStop(QString* errMsg = nullptr);
    
    /**
     * @brief 历史接口名：当前同时停止位置采样与触发输出
     * 实机 BASIC 共用 TABLE[334] enable；本接口等价于 hwTableSampleStop()。
     * 停止后可继续从已有 TABLE 记录和传感器接收队列中收尾，不产生新位置记录。
     */
    bool hwTriggerOnlyStop(QString* errMsg = nullptr);
    
    /**
     * @brief 读取 ECI3808 BASIC 维护的已确认锁存事件计数
     * V32.13计数来自已发布的五轴MPOS锁存FIFO推进量，而不是PC发送命令次数；
     * 用于与传感器收到的 Profile 数量做最终完整性校验。
     */
    bool hwGetTriggerCount(qint64& triggerCount, QString* errMsg = nullptr);

    struct HwStopTailDiag {
        int phase = -1;
        int state = -1, enable = -1, basicFault = -1;
        int sessionAck = -1, expectedSession = -1;
        int rawHead[5] = {-1,-1,-1,-1,-1};
        int stopRawHead = -1, pumpedHead = -1;
        int ringWrite = -1, ringRead = -1;
        double pwmDuty = -1.0;
        int op14 = -1; // SDK output-command readback, NOT independent electrical measurement
        qint64 published = -1;
        bool rawFifoDrained() const {
            if (phase != mls::runtime::Table5D::TailDrained ||
                state != mls::runtime::Table5D::IdleReady || enable != 0 ||
                basicFault != 0 || sessionAck != expectedSession ||
                !std::isfinite(pwmDuty) || pwmDuty < 0.0 || pwmDuty > 0.0001 || op14 != 0 ||
                pumpedHead != stopRawHead) return false;
            for (int h : rawHead) if (h != pumpedHead) return false;
            return true;
        }
    };
    /** Read-only, diagnostic snapshot of controller stopped RAW latch FIFO and Buffer-B. */
    bool hwReadStopTailDiagnostics(HwStopTailDiag& diag, QString* errMsg = nullptr);

    
    /**
     * @brief 历史接口；V32.13不允许单独清零。新 session 会原子复位双缓冲、序号和计数。
     */
    bool hwResetTriggerCount(QString* errMsg = nullptr);
    
    /**
     * @brief 从 TABLE 读取新的采样点
     * @param outNewSamples 新读取的采样点
     * @param overflow 是否发生过溢出
     */
    bool hwTableSampleReadOnce(QVector<SamplePoint>& outNewSamples,
                               bool& overflow, QString* errMsg = nullptr);
    
    /**
     * @brief 获取已读取的采样总数
     */
    qint64 getHwSampleCount() const { return m_hwSampleIndex.load(); }

    // ACK 结果不明/已发现溢出后禁止盲目续读；仅新一轮 hwTableSampleStart 清除。
    bool hwTableReadFaulted() const { return m_hwReadFault.load(); }

    // ==================== 圆弧插补 ====================
    
    /**
     * @brief XY平面三点圆弧（绝对坐标）
     * @param midX, midY 圆弧中间点
     * @param endX, endY 圆弧终点
     * @param clockwise 兼容参数；三点圆弧方向实际由起点/中间点/终点决定。
     * @param motion 运动参数
     */
    bool moveArc2Abs(float midX, float midY, 
                     float endX, float endY,
                     bool clockwise,
                     const AxisParams& motion,
                     QString* err = nullptr);
    
    /**
     * @brief XY平面三点圆弧（相对坐标）
     */
    bool moveArc2Rel(float midX, float midY,
                     float endX, float endY,
                     bool clockwise,
                     const AxisParams& motion,
                     QString* err = nullptr);
    
    /**
     * @brief XY平面圆心+终点圆弧（绝对坐标）
     * @param centerI, centerJ 圆心相对于起点的偏移
     * @param endX, endY 终点
     * @param clockwise true=顺时针, false=逆时针
     */
    bool moveArcCenterAbs(float centerI, float centerJ,
                          float endX, float endY,
                          bool clockwise,
                          const AxisParams& motion,
                          QString* err = nullptr);

    // ==================== 轴状态检查 ====================
    
    /**
     * @brief 检查所有插补轴是否空闲
     * @return 所有轴都空闲返回true
     */
    bool areAllAxesIdle(QString* err = nullptr) const;
    
    /**
     * @brief 检查轨迹是否执行完成
     */
    bool isTrajectoryComplete() const;

signals:
    /**
     * @brief 发生错误时发出
     */
    void errorOccurred(const QString& msg);

private:
#ifdef MLS_P0_TESTING
    friend struct P0ControllerTestAccess;
#endif
    using IoPriority = eci::EciIoScheduler::Priority;
    bool runIo(IoPriority priority, const char* name, std::function<bool()> fn,
               QString* err, unsigned domain = eci::EciIoScheduler::None,
               const eci::EciIoScheduler::Token* operationToken = nullptr,
               unsigned invalidateDomains = eci::EciIoScheduler::None) const;
    bool runLifecycle(const char* name, std::function<bool()> fn, QString* err);
    bool postStop(const char* name, std::function<bool()> fn);
    QString errorTextFromCode(int32 code, const char* func) const;
    bool checkNoJointTask5D(QString* err) const;
    bool checkNoBufferedTrajectory5D(QString* err);
    bool tryStartAxisTask(int axis, AxisTask task, QString* err);
    void finishAxisTask(int axis);
    bool tryStartJointTask5D(QString* err);
    void finishJointTask5D();
    bool waitAxisIdle(int axis, int timeoutMs, QString* err,
                      double expectedMpos = std::numeric_limits<double>::quiet_NaN(),
                      double effectiveSpeed = 0.0);
    bool waitJointMoveIdle5D(int baseAxis, int timeoutMs, QString* err);
    bool validateJointTargetAgainstControllerLimits5D(const JointPose5D& machineTarget,
                                                       JointPose5D& controllerTarget,
                                                       QString* err) const;

    bool moveAbsImpl(int axis, float pos, const AxisParams* motion, QString* err, bool rawDpos, bool targetIsMpos = false);
    bool moveRelImpl(int axis, float delta, const AxisParams* motion, QString* err, bool rawDpos);
    bool compensationConfigurationIdle(QString* err) const;
    double compensatedNominalStart(int axis, double dpos, double mpos) const;
    bool mapLinearTarget(int axis, double desired, double currentDpos, double currentMpos,
                         double& command, QString* err, bool relativeFromFeedback = false) const;
    bool validateLinearCommand(int axis, double desired, double command, QString* err) const;
    void rememberLinearTarget(int axis, double desired, double command);
    void invalidateLinearTarget(int axis = -1);
    bool waitCompensationSettle(int milliseconds, int axis, QString* err);
    bool compensatedArc(const mls::runtime::Joint& end, const mls::runtime::Arc& arc,
                        const AxisParams& motion, QString* err);

    bool checkRotaryMotionAllowed(int axis, QString* err) const;
    bool resetRotarySafetyState(QString* err = nullptr);
    int rotaryEnableOutputForAxis(int axis) const;
    bool getAxisPositionPairRaw(int axis, float& dpos, float& mpos, bool& idle,
                                QString* err = nullptr) const;
    float controllerToMachinePosition(int axis, float controllerPos) const;
    float machineToControllerPosition(int axis, float machinePos) const;
    bool applyRotaryControllerSoftLimits(int axis, QString* err = nullptr);
    bool restoreMachineHomeStateFromController(QString* err = nullptr);
    bool updateControllerHomeMaskBit(int axis, bool homed, QString* err = nullptr);

private:
    static constexpr int kMaxAxisCount = 8;

    // Worker-owned state guard retained for recursive helpers/test access. External
    // callers must dispatch BEFORE acquiring this lock; it is not the scheduler.
    mutable p0::IoMutex m_ioMutex;
    mutable std::atomic_int m_activeBlockingCalls{0};
    struct BlockingCallGuard {
        std::atomic_int& count;
        explicit BlockingCallGuard(std::atomic_int& c, std::mutex& lifecycle) : count(c) {
            std::lock_guard<std::mutex> lock(lifecycle); ++count;
        }
        ~BlockingCallGuard() { --count; }
        BlockingCallGuard(const BlockingCallGuard&) = delete;
        BlockingCallGuard& operator=(const BlockingCallGuard&) = delete;
    };

    ZMC_HANDLE m_handle = nullptr;
    std::atomic_bool m_connected{false};
    QString m_lastIp;

    // 各轴当前任务类型
    std::array<std::atomic_int, kMaxAxisCount> m_axisTask{};
    
    // 各轴停止请求标志
    std::array<std::atomic_bool, kMaxAxisCount> m_stopRequested{};

    // XYZ增量光栅的机床坐标基准有效状态。V32.13可从控制器RAM中的本次上电Home mask恢复；控制器重启/BASIC重启会清零。
    std::array<std::atomic_bool, kMaxAxisCount> m_linearMposReferenced{};

    // 当前连接会话中由上位机成功发出的 SetMpos(home-zero) 计数。
    // 普通过零运动绝不能改变该计数；诊断 CSV 会保存起止 generation。
    std::array<std::atomic_uint64_t, kMaxAxisCount> m_linearMposSetGeneration{};

    // B/C 旋转轴绝对位置/人工使能状态。
    std::array<std::atomic_bool, kMaxAxisCount> m_rotaryAbsReady{};
    std::array<std::atomic_bool, kMaxAxisCount> m_rotaryServoEnabled{};

    // V32.13: C/B 本次控制器上电会话是否已经执行过“回零到机械 0°”。
    // 该状态持久化在 TABLE[357] bits 3/4，但不能替代每次上位机连接后的 RS485 absolute-ready 重建。
    std::array<std::atomic_bool, kMaxAxisCount> m_rotaryHomeCompleted{};

    // V15/V16: 不再修改控制器 MPOS/DPOS。机械坐标由上位机维护：
    // machinePosition = controllerPosition + m_rotaryMachineOffset[axis]
    std::array<std::atomic<double>, kMaxAxisCount> m_rotaryMachineOffset{};
    std::array<std::atomic<double>, kMaxAxisCount> m_rotaryMachineSoftLimitPos{};
    std::array<std::atomic<double>, kMaxAxisCount> m_rotaryMachineSoftLimitNeg{};

    // All table/cache/options access is serialized by EciIoScheduler; only
    // enabled is atomic for UI readback. Never change a map during a queued path.
    mls::axiscomp::Tables m_axisCompTables;
    mls::axiscomp::Options m_axisCompOptions;
    bool m_axisCompLoaded = false;
    std::atomic_bool m_axisCompEnabled{false};
    struct NominalTargetCache { bool valid = false; double nominal = 0, command = 0; };
    std::array<NominalTargetCache, 3> m_axisCompTargets{};
    mls::runtime::Joint m_mergeFinalNominal{};
    bool m_mergeUsesAxisCompensation = false;

    // 连续插补数据
    std::vector<PathPoint> m_mergePath;
    mls::runtime::Plan m_mergePlan;
    mls::runtime::Limits m_trajectoryLimits;
    std::array<double,2> m_mergeOffsets{};
    bool m_mergePaused = false;
    std::atomic<double> m_trajectorySeconds{0};
    size_t m_mergeIndex = 0;
    bool m_mergeRefilling = false;
    int m_mergeLastBatchSent = 0;
    eci::EciIoScheduler::Token m_mergeToken;
    std::uint64_t m_mergeRevision = 0;

    // 硬件 TABLE 采样状态
    int m_hwBufSize = 0;
    int m_hwReadIndex = 0;
    bool m_hwReadCursorOwned = false; // valid after V32.13 session handshake; remains true during tail drain
    std::atomic<qint64> m_hwSampleIndex{0};
    std::atomic_bool m_hwReadFault{false};
    QString m_hwReadFaultReason;
    bool m_hwSampling = false;
    int m_hwSession = 0;
    std::atomic_int m_hwTriggerFrequencyHz{mls::runtime::Table5D::defaultTriggerFrequencyHz};
    std::atomic_int m_hwTopology{1};
    std::atomic_int m_hwCmmEdge{104};
    std::uint32_t m_hwFeedbackMask = mls::runtime::Table5D::defaultFeedbackMask;
    std::uint32_t m_hwLatchMask = mls::runtime::Table5D::defaultLatchMask;
    std::uint32_t m_hwCommandMask = mls::runtime::Table5D::defaultCommandMask;
    mls::runtime::PoseTiming m_hwTiming = mls::runtime::PoseTiming::HardwareLatched;
    mls::runtime::TablePoseVariant m_hwVariant = mls::runtime::TablePoseVariant::FullMposLatched;
    std::array<double,2> m_hwOffsets{};
    mutable std::mutex m_lifecycleMutex;
    std::atomic_bool m_lifecycleChanging{false};
    mutable eci::EciIoScheduler m_io; // destroyed/joined before captured member state

};
