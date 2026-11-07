#pragma once

#include <QObject>
#include <QVector>
#include <QMutex>
#include <QTimer>
#include <QThread>
#include <memory>
#include <atomic>
#include <deque>

#include "core/DataTypes.h"
#include "core/ProfileCounter24.h"
#include "core/ProfileSessionEpoch.h"
#include "core/ProfileTimingAnalysis.h"
#include "runtime/HardwareTemporalCorrection.h"
#include "runtime/Acquisition5D.h"

class LaserSensor;
class ZmcController;

/**
 * @brief 同步策略接口
 */
class ISyncStrategy : public QObject
{
    Q_OBJECT

public:
    explicit ISyncStrategy(QObject* parent = nullptr) : QObject(parent) {}
    virtual ~ISyncStrategy() = default;

    virtual bool start(QString* errMsg = nullptr) = 0;
    virtual void stop() = 0;
    virtual bool isRunning() const = 0;
    virtual void getStats(int& profileCount, int& poseCount) const = 0;
    virtual void setMountOffset(const MountOffset& offset) = 0;
    bool setKinematicModel(const std::array<double,17>& p, bool absoluteGaugeKnown, QString* err=nullptr);
    bool clearKinematicModel(QString* err=nullptr);

protected:
    std::shared_ptr<const mls::runtime::Frame5D> calibratedFrame(const SamplePoint& pose, QString& err) const;
    std::array<double,17> m_physicalParameters{};
    bool m_modelConfigured=false;
    bool m_absoluteGaugeKnown=false;


signals:
    void alignedDataReady(const QVector<AlignedPoint>& points);
    void errorOccurred(const QString& msg);
};

/**
 * @brief 软件同步策略
 */
class SoftwareSyncStrategy : public ISyncStrategy
{
    Q_OBJECT

public:
    SoftwareSyncStrategy(LaserSensor* laser, ZmcController* zmc, QObject* parent = nullptr);
    ~SoftwareSyncStrategy() override;

    bool start(QString* errMsg = nullptr) override;
    void stop() override;
    bool isRunning() const override { return m_running; }
    void getStats(int& profileCount, int& poseCount) const override;
    void setMountOffset(const MountOffset& offset) override { m_mountOffset = offset; }
    QVector<AlignedPoint> getAllAlignedPoints() const { QMutexLocker lock(&m_savedMutex);return m_savedAligned; }
    QVector<SamplePoint> getAllPoseSamples() const { QMutexLocker lock(&m_savedMutex);return m_savedPoses; }
    void clearData() { if(!m_running){QMutexLocker lock(&m_savedMutex);m_savedAligned.clear();m_savedPoses.clear();} }


private slots:
    void onProfileReceived(const LaserProfile& profile);
    void onProcessTimer();

private:
    bool interpolatePoseAtTime(qint64 tUs, SamplePoint& pose);
    void startPoseThread();
    void stopPoseThread();

private:
    LaserSensor* m_laser = nullptr;
    ZmcController* m_zmc = nullptr;

    bool m_running = false;
    MountOffset m_mountOffset;

    mutable QMutex m_savedMutex;
    QVector<AlignedPoint> m_savedAligned;
    QVector<SamplePoint> m_savedPoses;
    std::uint64_t m_captureSession=0, m_captureOrdinal=0;
    QVector<LaserProfile> m_profileQueue;
    mutable QMutex m_profileMutex;
    qint64 m_profileCount = 0;

    // 位置环形缓冲区（软件同步用于插值，不需要无限存储）
    static constexpr int kSoftwarePoseRingSize = 30000;  // 30 秒 @ 1kHz
    QVector<SamplePoint> m_poseRing;
    int m_poseRingHead = 0;
    int m_poseRingCount = 0;
    mutable QMutex m_poseRingMutex;

    class PoseSampleThread;
    PoseSampleThread* m_poseThread = nullptr;
    QTimer m_processTimer;
    QMetaObject::Connection m_profileConnection;
};

/**
 * @brief ECI3808 V32.13 SINGLE-OP14 / FULL-MPOS-LATCH + ColdPrime TABLE同步策略。
 *
 * 现场只使用OP14：同一物理输出同时连接scanCONTROL触发输入和ECI IN0。
 * scanCONTROL使用EdgeRising；ECI使用REGIST(104)锁R0逻辑下降沿，两者均对应
 * 已实测的OP14关闭/线路回高时刻。BASIC按最终光栅尺架构工作：XYZ/C/B五轴
 * 均为ATYPE=4，并由REGIST(104)保存raw MPOS。XYZ增量光栅在DATUM成功后由
 * homeAxis()建立MPOS机床零点；普通状态、软件同步和标定也统一使用五轴MPOS。
 * DPOS仍保留为控制器命令/需求位置以及跟随误差诊断量。
 *
 * enable=3通过同一OP14进行ColdPrime。预热发生在正式session/REGIST重建之前；
 * 预热Profile全部抑制并丢弃，随后新session清空并重新Arm五轴REGIST FIFO，
 * 再执行正式1 Profile + 1 Pose Probe。
 */
class HardwareSyncStrategy : public ISyncStrategy
{
    Q_OBJECT

public:
    HardwareSyncStrategy(LaserSensor* laser, ZmcController* zmc, QObject* parent = nullptr);
    void setHardwareTopology(mls::runtime::HardwareSyncTopology topology) { m_hardwareTopology=topology; }
    void restartProbeTimeout();
    ~HardwareSyncStrategy() override;

    bool start(QString* errMsg = nullptr) override;
    void stop() override;
    bool isRunning() const override { return m_running.load(); }
    void getStats(int& profileCount, int& poseCount) const override;
    void setMountOffset(const MountOffset& offset) override;

    /**
     * @brief 完成最后的对齐处理（stop 后调用）
     */
    void finalizeAlignment();

    /**
     * @brief 对齐统计信息
     */
    struct AlignStats {
        qint64 totalPoseSamples = 0;    // 已读取的位置采样数
        qint64 totalProfiles = 0;       // 已接收的 Profile 数
        qint64 alignedProfiles = 0;     // 成功对齐的 Profile 数
        qint64 validAlignedProfiles = 0; // paired frames containing at least one valid point
        qint64 emptyAlignedProfiles = 0; // paired, zero valid points; NOT necessarily empty travel
        qint64 temporalBoundarySkippedProfiles = 0; // Probe/final tail paired but not reconstructed under +delay look-ahead
        qint64 pendingProfiles = 0;     // 等待对齐的 Profile 数
        qint64 droppedProfiles = 0;     // 丢弃的 Profile 数（位置数据已丢失）
        qint64 tableOverflows = 0;      // TABLE 溢出次数
        qint64 zmcTriggerCount = 0;     // ECI3808 BASIC已发布锁存ordinal计数；字段名仅保留ABI兼容，不是独立物理触发计数
        
        quint32 firstProfileIndex = 0;  // 第一个 Profile 的 index
        bool syncEstablished = false;
        bool alignmentFault = false;   // true: whole scan invalid; earlier cloud is diagnostic only
        QString faultReason;
        qint64 missingProfileIndices = 0; // observed forward gaps, not proof of lost triggers
        qint64 duplicateProfiles = 0;
        qint64 outOfOrderProfiles = 0;
        qint64 invalidProfileIndices = 0;
        qint64 tableReadErrors = 0;
        bool zmcTriggerCountValid = false; // cached published-latch count at finalize; not an independent edge counter
    };
    AlignStats getAlignStats() const;

    /**
     * @brief 获取所有已对齐的数据（用于导出）
     */
    QVector<AlignedPoint> getAllAlignedPoints() const;
    
    /**
     * @brief 获取所有位置数据（用于导出）
     */
    QVector<SamplePoint> getAllPoseSamples() const;

    /**
     * V27.6.48: transfer ownership of the completed hardware capture out of
     * SyncManager without duplicating the potentially very large aligned-point
     * array.  This is valid only after stop()/finalizeAlignment().
     *
     * The destination vectors receive the backing storage by swap; the strategy
     * is left with empty vectors so the next calibration line cannot accumulate
     * the previous line in RAM.
     */
    bool takeCompletedCapture(QVector<AlignedPoint>& aligned,
                              QVector<SamplePoint>& poses,
                              QString* error = nullptr);

    /**
     * Hardware-temporal automatic scans do not need live 3-D GUI accumulation.
     * Suppressing alignedDataReady during those scans prevents PointCloudBuilder
     * and queued GUI events from retaining a second full copy of the scan.
     */
    void setAlignedDataEmissionEnabled(bool enabled) { m_emitAlignedData.store(enabled); }
    bool alignedDataEmissionEnabled() const { return m_emitAlignedData.load(); }

    /**
     * V27.6.48: dedicated memory-safe hardware-temporal capture mode.
     *
     * In this mode processProfiles() proves the Profile<->REGIST ordinal pairing
     * and stores the decoded LaserProfile once, but deliberately does NOT create
     * one AlignedPoint object per valid sensor point on the GUI/owner thread.
     * The completed raw profiles and hardware-latched poses are transferred to a
     * background archive/analysis worker after Stop/Drain.
     */
    void setRawProfileCaptureEnabled(bool enabled) { m_rawProfileCaptureEnabled.store(enabled); }
    bool rawProfileCaptureEnabled() const { return m_rawProfileCaptureEnabled.load(); }
    bool takeCompletedRawCapture(QVector<LaserProfile>& profiles,
                                 QVector<SamplePoint>& poses,
                                 QString* error = nullptr);

    /**
     * @brief 复制当前硬件同步 TABLE 已读取的最新一条五轴锁存位姿。
     *
     * 仅访问 m_allPoseSamples 内存，不产生任何 ZAux/ECI I/O。返回的 C/B 已经
     * 应用本次硬件 session 冻结的 software-frame offset，属于机械绝对角。
     * 样本只有在 Physical17 completeFrame 校验通过后才会进入该数组。
     */
    bool latestPoseSample(SamplePoint& sample) const;
    
    /**
     * @brief 获取所有接收到的 Profile Index（包括空Profile）
     * 用于分析报告区分"真正丢包"和"空Profile过滤"
     */
    QVector<quint32> getAllReceivedProfileIndices() const;
    std::vector<syncdiag::ProfileRecord> getProfileRecords() const;
    
    /**
     * @brief 清除所有数据（仅在非运行状态调用）
     */
    void clearData();

private slots:
    void onProfileReceived(const LaserProfile& profile);
    void onAlignTimer();

private:
    class TableReadThread;
    friend class TableReadThread;
#ifdef MLS_P0_TESTING
    friend struct P0SyncTestAccess;
#endif
    friend class SyncManager;
    void invalidateAlignment(const QString& reason);
    void reportAlignmentFault();

    /**
     * @brief 尝试建立同步
     */
    bool tryEstablishSync();

    /**
     * @brief 处理待对齐的 Profiles
     */
    void processProfiles(bool finalPass = false);

private:
    LaserSensor* m_laser = nullptr;
    ZmcController* m_zmc = nullptr;

    std::atomic<bool> m_running{false};
    MountOffset m_mountOffset;
    mutable QMutex m_mountOffsetMutex;

    // ===== Profile 数据（无限存储）=====
    struct PendingProfile {
        LaserProfile profile;
        std::int64_t poseOrdinal = 0;
        std::size_t metadataIndex = 0;
    };
    p0::ProfileCounter24 m_profileCounter; // protected by m_profileMutex
    std::deque<PendingProfile> m_pendingProfiles;  // 待对齐的 Profiles
    mutable QMutex m_profileMutex;
    std::atomic<qint64> m_totalProfileCount{0};
    // H3: scanCONTROL timestamp carries a 14-bit generated-CMM-impulse count.
    // These variables are protected by m_profileMutex (NOT a controller IN1 edge counter).
    bool m_cmmCounterSeen = false;
    quint32 m_lastCmmCounter = 0;
    qint64 m_contiguousCmmProfiles = 0;
    
    // ===== 所有接收到的 Profile Index（用于分析连续性）=====
    QVector<quint32> m_allReceivedProfileIndices;
    std::vector<syncdiag::ProfileRecord> m_profileRecords;
    mutable QMutex m_receivedIndicesMutex;

    // ===== 位置数据（无限存储）=====
    QVector<SamplePoint> m_allPoseSamples;
    mutable QMutex m_poseMutex;
    std::atomic<qint64> m_poseSampleCount{0};

    // ===== 对齐结果（无限存储）=====
    QVector<AlignedPoint> m_allAlignedPoints;
    mutable QMutex m_alignedMutex;
    std::atomic_bool m_emitAlignedData{true};

    // Memory-safe temporal-calibration mode: keep one decoded profile record per
    // trigger instead of expanding every point into a heavyweight AlignedPoint.
    QVector<LaserProfile> m_rawCapturedProfiles;
    mutable QMutex m_rawCaptureMutex;
    std::atomic_bool m_rawProfileCaptureEnabled{false};

    // ===== 同步状态 =====
    std::atomic<bool> m_syncEstablished{false};
    quint32 m_firstProfileIndex = 0;
    p0::ProfileSessionEpoch m_ingressSession; // callback-time epoch, checked again in Qt queue
    std::atomic_bool m_alignmentFault{false};
    mutable QMutex m_faultMutex;
    QString m_faultReason;
    bool m_faultReported = false;
    std::atomic<qint64> m_missingIndexCount{0};
    std::atomic<qint64> m_duplicateCount{0};
    std::atomic<qint64> m_outOfOrderCount{0};
    std::atomic<qint64> m_invalidIndexCount{0};
    std::atomic<qint64> m_tableReadErrors{0};
    std::atomic<qint64> m_cachedTriggerCount{0};
    std::atomic_bool m_cachedTriggerCountValid{false};
    
    // ===== 统计 =====
    std::atomic<qint64> m_alignedCount{0};
    std::atomic<qint64> m_validAlignedCount{0}, m_emptyAlignedCount{0};
    std::atomic<qint64> m_temporalBoundarySkippedCount{0};

    mls::runtime::HardwareSyncTopology m_hardwareTopology=mls::runtime::HardwareSyncTopology::EciPwmShared;
    // Frozen at session start, read-only while TableReadThread is alive.
    int m_captureTriggerFrequencyHz=mls::runtime::Table5D::defaultTriggerFrequencyHz;
    // V27.6.44 applies only to the commissioned SINGLE-OP14 -> sensor + IN0/REGIST(104) topology.
    bool m_temporalCorrectionEnabled = false;
    double m_temporalDelaySec = 0.0;
    double m_temporalTriggerPeriodSec = 1.0 / static_cast<double>(mls::runtime::Table5D::defaultTriggerFrequencyHz);
    std::atomic<qint64> m_droppedCount{0};
    std::atomic<qint64> m_overflowCount{0};

    // ===== 线程和定时器 =====
    TableReadThread* m_tableThread = nullptr;
    QTimer m_alignTimer;
    QMetaObject::Connection m_profileConnection;
    qint64 m_probeStartedUs=0;
};


/** V27.6.44 hardware temporal calibration/correction bound to the current
 * ECI3808 V32.13 SINGLE-OP14 + IN0/REGIST(104) acquisition topology. */
struct HardwareTemporalCorrectionSettings {
    bool enabled = false;
    double delaySec = 0.0;
    double delayStdSec = 0.0;
    QString calibrationId;
    SensorParams sensorParams;
    int triggerFrequencyHz = mls::runtime::Table5D::defaultTriggerFrequencyHz;
    double triggerPeriodMs = 1000.0 / static_cast<double>(mls::runtime::Table5D::defaultTriggerFrequencyHz);
    mls::runtime::HardwareSyncTopology topology = mls::runtime::HardwareSyncTopology::EciPwmShared;
    int latchEdge = mls::runtime::Table5D::h1R0Falling; // H1=104; H3=114/115
    // H3 CMM configuration in LaserSensor::configureCmmOutput; fail-closed if changed.
    int cmmDivider = 0; // H1=0, H3=1
    int cmmSkewHalfUs = 0;
    int cmmRatio = 1;
};

/**
 * @brief 同步管理器
 */
class SyncManager : public QObject
{
    Q_OBJECT

public:
    SyncManager(LaserSensor* laser, ZmcController* zmc, QObject* parent = nullptr);
    ~SyncManager() override;

    bool setKinematicModel(const std::array<double,17>& parameters, bool absoluteGaugeKnown=false, QString* err=nullptr);
    bool clearKinematicModel(QString* err=nullptr);
    bool hasKinematicModel() const { return m_modelConfigured; }
    // Read-only snapshot of the model ACTUALLY applied to both acquisition
    // strategies. Does not assert its independent metrological acceptance.
    bool currentKinematicModel(std::array<double,17>* out) const {
        if (!out || !m_modelConfigured) return false;
        *out=m_activePhysical17;
        return true;
    }
    bool hasCaptureFault() const { return m_captureFault; }
    QString captureFaultReason() const { return m_captureFaultReason; }
    void setSyncMode(SyncMode mode);
    SyncMode syncMode() const { return m_syncMode; }

    void setMountOffset(const MountOffset& offset);
    void setSensorParams(const SensorParams& params) { m_sensorParams = params; }
    void setHardwareTriggerFrequencyHz(int frequencyHz) { m_hwTriggerFrequencyHz = frequencyHz; }
    int hardwareTriggerFrequencyHz() const { return m_hwTriggerFrequencyHz; }
    int currentCmmR1Edge() const { return m_cmmR1Edge; }
    int lastCaptureCmmR1Edge() const { return m_lastCaptureCmmR1Edge; }
    mls::runtime::HardwareSyncTopology lastCaptureTopology() const { return m_lastCaptureTopology; }
    void setHardwareTopology(mls::runtime::HardwareSyncTopology mode,int cmmEdge=114) {
        m_hardwareTopology=mode; m_cmmR1Edge=cmmEdge;
    }
    mls::runtime::HardwareSyncTopology hardwareTopology() const { return m_hardwareTopology; }

    // Hardware trigger semantics must be supplied explicitly by the initiating
    // workflow.  A default enum value is not authorization to configure hardware.
    void setTriggerType(HardwareTriggerType type) { m_triggerType = type; m_triggerTypeConfigured = true; }
    void clearHardwareTriggerTypeConfiguration() { m_triggerTypeConfigured = false; }
    bool hasExplicitHardwareTriggerType() const { return m_triggerTypeConfigured; }

    bool start(QString* errMsg = nullptr);
    void stop();
    bool isRunning() const;
    
    /**
     * @brief 清除所有采集数据（仅在非运行状态调用）
     */
    void clearData();

    void getStats(int& profileCount, int& poseCount) const;
    HardwareSyncStrategy::AlignStats getHardwareAlignStats() const;
    
    QVector<AlignedPoint> getHardwareAlignedPoints() const;
    QVector<SamplePoint> getHardwarePoseSamples() const;

    // V27.6.48 memory-safe handoff used by the automatic hardware-temporal
    // workflow.  Unlike getHardwareAlignedPoints(), this moves the completed
    // capture out of SyncManager instead of creating/retaining another owner.
    bool takeHardwareCaptureData(QVector<AlignedPoint>& aligned,
                                 QVector<SamplePoint>& poses,
                                 QString* error = nullptr);

    // Disable only the live alignedDataReady fan-out.  Formal hardware capture
    // and m_allAlignedPoints remain lossless and are handed off after Stop/Drain.
    void setHardwareAlignedDataEmissionEnabled(bool enabled);
    bool hardwareAlignedDataEmissionEnabled() const;

    // V27.6.48 automatic temporal-calibration capture: keep decoded profiles +
    // REGIST poses only during the motion, then move them to a worker after Stop.
    void setHardwareRawProfileCaptureEnabled(bool enabled);
    bool hardwareRawProfileCaptureEnabled() const;
    bool takeHardwareRawCaptureData(QVector<LaserProfile>& profiles,
                                    QVector<SamplePoint>& poses,
                                    QString* error = nullptr);

    /**
     * @brief 硬件同步运行期间取得最新 TABLE 锁存位姿，仅供只读 UI/诊断。
     *
     * 不访问控制器，不参与标定采样，不改变 Physical17/RTCP 模型。
     */
    bool latestHardwarePoseSample(SamplePoint& sample) const;
    
    /**
     * @brief 获取所有接收到的 Profile Index（包括空Profile）
     * 用于分析报告区分"真正丢包"和"空Profile过滤"
     */
    QVector<quint32> getHardwareReceivedProfileIndices() const;
    std::vector<syncdiag::ProfileRecord> getHardwareProfileRecords() const;
    SensorParams lastCaptureSensorParams() const { return m_lastCaptureSensorParams; }
    int lastCaptureTriggerFrequencyHz() const { return m_lastCaptureTriggerFrequencyHz; }
    double lastCaptureTriggerPeriodMs() const { return m_lastCaptureTriggerPeriodMs; }

    // V27.6.44 temporal correction is a PC-side reconstruction step.  It never
    // changes the ECI BASIC program, OP14 edge, IN0 wiring or REGIST(104) mode.
    bool setHardwareTemporalCorrection(const HardwareTemporalCorrectionSettings& settings, QString* errMsg = nullptr);
    void disableHardwareTemporalCorrection();
    HardwareTemporalCorrectionSettings hardwareTemporalCorrection() const { return m_temporalCorrection; }
    HardwareTemporalCorrectionSettings lastCaptureHardwareTemporalCorrection() const { return m_lastCaptureTemporalCorrection; }

signals:
    void alignedDataReady(const QVector<AlignedPoint>& points);
    void errorOccurred(const QString& msg);
    void runningChanged(bool running);

private:
    bool validateSensorParams(QString* errMsg) const;

private:
    LaserSensor* m_laser = nullptr;
    ZmcController* m_zmc = nullptr;

    bool m_modelConfigured = false;
    std::array<double,17> m_activePhysical17{};
    bool m_captureFault = false;
    QString m_captureFaultReason;
    bool m_stopping = false; // stop() processes queued Qt events; prevent reentry
    SyncMode m_syncMode = SyncMode::Software;
    MountOffset m_mountOffset;
    SensorParams m_sensorParams;
    SensorParams m_lastCaptureSensorParams;
    int m_lastCaptureTriggerFrequencyHz = mls::runtime::Table5D::defaultTriggerFrequencyHz;
    double m_lastCaptureTriggerPeriodMs = 1000.0 / static_cast<double>(mls::runtime::Table5D::defaultTriggerFrequencyHz);
    int m_hwTriggerFrequencyHz = mls::runtime::Table5D::defaultTriggerFrequencyHz;
    mls::runtime::HardwareSyncTopology m_hardwareTopology=mls::runtime::HardwareSyncTopology::EciPwmShared;
    int m_cmmR1Edge=114;
    int m_lastCaptureCmmR1Edge = mls::runtime::Table5D::cmmR1Rising;
    mls::runtime::HardwareSyncTopology m_lastCaptureTopology=mls::runtime::HardwareSyncTopology::EciPwmShared;
    HardwareTriggerType m_triggerType = HardwareTriggerType::EdgeRising; // V32.13 single OP14: sensor rising line edge + ECI REGIST(104) R0 falling
    bool m_triggerTypeConfigured = false;
    HardwareTemporalCorrectionSettings m_temporalCorrection;
    HardwareTemporalCorrectionSettings m_lastCaptureTemporalCorrection;

    std::unique_ptr<SoftwareSyncStrategy> m_softwareStrategy;
    std::unique_ptr<HardwareSyncStrategy> m_hardwareStrategy;
};
