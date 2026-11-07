#include "core/SyncManager.h"
#include "core/LaserSensor.h"
#include "core/ZmcController.h"
#include "utils/GlobalTimeSync.h"
#include "utils/LogManager.h"

#include <QThread>
#include <QDebug>
#include <QElapsedTimer>
#include <QCoreApplication>
#include <QEvent>
#include <QMetaObject>
#include <QEventLoop>
#include <cmath>
#include <atomic>
namespace {
    std::atomic<std::uint64_t> softwareSessionCounter{ 1 };

}

#ifdef Q_OS_WIN
#include <windows.h>
#include <mmsystem.h>
#endif

bool ISyncStrategy::setKinematicModel(const std::array<double, 17>& p, bool absoluteGaugeKnown, QString* err)
{
    if (isRunning()) { if (err)*err = tr("采集中禁止更换标定模型"); return false; }
    mls::runtime::Frame5D check; std::string detail;
    if (!mls::runtime::completeFrame(check, p, detail)) { if (err)*err = QString::fromStdString(detail); return false; }
    m_physicalParameters = p; m_absoluteGaugeKnown = absoluteGaugeKnown; m_modelConfigured = true; return true;
}
bool ISyncStrategy::clearKinematicModel(QString* err)
{
    if (isRunning()) { if (err)*err = tr("采集中禁止清除标定模型"); return false; }
    m_physicalParameters.fill(0.0);
    m_absoluteGaugeKnown = false;
    m_modelConfigured = false;
    if (err) err->clear();
    return true;
}
std::shared_ptr<const mls::runtime::Frame5D> ISyncStrategy::calibratedFrame(const SamplePoint& pose, QString& err) const
{
    if (!m_modelConfigured || !pose.hasRotary) { err = tr("缺少完整17参数标定或逐轮廓C/B关节信息"); return {}; }
    auto f = std::make_shared<mls::runtime::Frame5D>(pose.frame);
    f->joints = { pose.x,pose.y,pose.z,pose.c,pose.b };
    f->hostEstimateUs = pose.tUs; f->absoluteGaugeKnown = m_absoluteGaugeKnown;
    std::string detail;
    if (!mls::runtime::completeFrame(*f, m_physicalParameters, detail)) { err = QString::fromStdString(detail); return {}; }
    return f;
}

// ============================================================================
// 软件同步 - 位置采样线程
// ============================================================================

class SoftwareSyncStrategy::PoseSampleThread : public QThread
{
public:
    PoseSampleThread(ZmcController* controller,
        QVector<SamplePoint>* ring,
        int* head, int* count,
        QMutex* mutex,
        QObject* parent = nullptr)
        : QThread(parent)
        , m_controller(controller)
        , m_ring(ring)
        , m_head(head)
        , m_count(count)
        , m_mutex(mutex)
    {
    }

    void requestStop() { m_stop.store(true); }

protected:
    void run() override
    {
#ifdef Q_OS_WIN
        timeBeginPeriod(1);
#endif
        setPriority(QThread::HighestPriority);

        QElapsedTimer timer;
        timer.start();
        qint64 nextWakeUs = 10000;

        while (!m_stop.load()) {
            qint64 nowUs = timer.nsecsElapsed() / 1000;
            if (nowUs < nextWakeUs) {
                qint64 sleepUs = nextWakeUs - nowUs;
                if (sleepUs > 500) {
                    QThread::usleep(static_cast<unsigned long>(sleepUs - 200));
                }
                while (timer.nsecsElapsed() / 1000 < nextWakeUs) { /* busy wait */ }
            }
            nextWakeUs = timer.nsecsElapsed() / 1000 + 10000;

            SamplePoint sp;
            if (m_controller->readJointSample5D(sp, nullptr)) {
                QMutexLocker locker(m_mutex);
                const int capacity = m_ring->size();
                if (capacity > 0) {
                    (*m_ring)[*m_head] = sp;
                    *m_head = (*m_head + 1) % capacity;
                    if (*m_count < capacity) ++(*m_count);
                }
            }
        }

#ifdef Q_OS_WIN
        timeEndPeriod(1);
#endif
    }

private:
    ZmcController* m_controller;
    QVector<SamplePoint>* m_ring;
    int* m_head;
    int* m_count;
    QMutex* m_mutex;
    std::atomic<bool> m_stop{ false };
};

// ============================================================================
// 软件同步策略
// ============================================================================

SoftwareSyncStrategy::SoftwareSyncStrategy(LaserSensor* laser, ZmcController* zmc, QObject* parent)
    : ISyncStrategy(parent), m_laser(laser), m_zmc(zmc)
{
    // Profile subscription is armed only while software sync is active. This avoids
    // queuing per-profile no-op events during hardware acquisition.
    m_processTimer.setInterval(Constants::kProfileUpdateIntervalMs);
    connect(&m_processTimer, &QTimer::timeout, this, &SoftwareSyncStrategy::onProcessTimer);
}

SoftwareSyncStrategy::~SoftwareSyncStrategy() { stop(); }

bool SoftwareSyncStrategy::start(QString* errMsg)
{
    if (m_running) {
        if (errMsg) *errMsg = tr("软件同步已在运行");
        return false;
    }

    if (!m_modelConfigured) { if (errMsg)*errMsg = tr("请先应用17参数扫描标定"); return false; }
    m_poseRing.resize(kSoftwarePoseRingSize);
    m_poseRingHead = 0;
    m_poseRingCount = 0;
    m_profileCount = 0;
    { QMutexLocker lock(&m_savedMutex); m_savedAligned.clear(); m_savedPoses.clear(); }
    m_captureSession = softwareSessionCounter.fetch_add(1); m_captureOrdinal = 0;

    { QMutexLocker locker(&m_profileMutex); m_profileQueue.clear(); }

    SamplePoint initial;
    if (!m_zmc->readJointSample5D(initial, errMsg))return false;
    m_poseRing[0] = initial; m_poseRingHead = 1; m_poseRingCount = 1;
    startPoseThread();
    m_profileConnection = connect(m_laser, &LaserSensor::profileReceived,
        this, &SoftwareSyncStrategy::onProfileReceived,
        Qt::QueuedConnection);
    m_processTimer.start();
    m_running = true;
    return true;
}

void SoftwareSyncStrategy::stop()
{
    if (!m_running) return;
    m_processTimer.stop();
    // Sensor delivery is stopped by SyncManager first. Keep the pose worker alive
    // long enough to bracket the final received profile; never drop the tail silently.
    QElapsedTimer drain; drain.start();
    while (drain.elapsed() < 200) {
        onProcessTimer();
        bool empty; { QMutexLocker lock(&m_profileMutex); empty = m_profileQueue.isEmpty(); }
        if (empty) break;
        QThread::msleep(10);
    }
    bool missing; { QMutexLocker lock(&m_profileMutex); missing = !m_profileQueue.isEmpty(); }
    stopPoseThread();
    if (m_profileConnection) { disconnect(m_profileConnection); m_profileConnection = {}; }
    m_running = false;
    if (missing) emit errorOccurred(tr("软件同步尾帧缺少五轴包围采样，拒绝将该次扫描标记为完整"));
}

void SoftwareSyncStrategy::getStats(int& profileCount, int& poseCount) const
{
    profileCount = static_cast<int>(m_profileCount);
    QMutexLocker locker(&m_poseRingMutex);
    poseCount = m_poseRingCount;
}

void SoftwareSyncStrategy::startPoseThread()
{
    if (m_poseThread) return;
    m_poseThread = new PoseSampleThread(m_zmc, &m_poseRing, &m_poseRingHead, &m_poseRingCount, &m_poseRingMutex, this);
    m_poseThread->start();
}

void SoftwareSyncStrategy::stopPoseThread()
{
    if (!m_poseThread) return;
    m_poseThread->requestStop();
    if (!m_poseThread->wait(3000)) {
        LOG_WARNING(LogCategory::Sync, tr("等待位置采样线程退出超过3秒；继续等待，禁止删除运行中的QThread"));
        m_poseThread->wait();
    }
    delete m_poseThread;
    m_poseThread = nullptr;
}

void SoftwareSyncStrategy::onProfileReceived(const LaserProfile& profile)
{
    if (!m_running) return;
    QMutexLocker locker(&m_profileMutex);
    m_profileQueue.append(profile);
    ++m_profileCount;
}

void SoftwareSyncStrategy::onProcessTimer()
{
    QVector<LaserProfile> profiles;
    { QMutexLocker lock(&m_profileMutex); if (m_profileQueue.isEmpty())return; profiles.swap(m_profileQueue); }
    QVector<AlignedPoint> points;
    for (const auto& profile : profiles) {
        SamplePoint pose;
        if (!interpolatePoseAtTime(profile.tPcUs, pose)) {
            // Wait for the bracketing sample, never extrapolate or substitute a current pose.
            if (GlobalTimeSync::instance().timestampUs() - profile.tPcUs < 100000) {
                QMutexLocker lock(&m_profileMutex); m_profileQueue.append(profile); continue;
            }
            m_processTimer.stop(); m_zmc->requestStopAllAxes(false);
            emit errorOccurred(tr("软件同步缺少100ms内的五轴包围采样；已停止运动，禁止用当前姿态补帧")); return;
        }
        pose.frame.session = m_captureSession; pose.frame.ordinal = m_captureOrdinal++;
        QString err; const auto frame = calibratedFrame(pose, err);
        if (!frame) { m_processTimer.stop(); m_zmc->requestStopAllAxes(false); emit errorOccurred(err); return; }
        pose.frame = *frame;
        { QMutexLocker lock(&m_savedMutex); m_savedPoses.append(pose); }
        for (int i = 0; i < qMin(profile.x.size(), profile.z.size()); ++i) {
            if (!SensorUtils::isValidPoint(profile.x[i], profile.z[i]))continue;
            AlignedPoint ap; ap.tUs = pose.tUs; ap.tSensorSec = profile.tOpen; ap.profileIndex = profile.index; ap.pointIndex = i;
            ap.sensorX = profile.x[i]; ap.sensorZ = profile.z[i]; ap.frame = frame;
            const auto p = mls::runtime::mapProfilePoint(*frame, ap.sensorX, ap.sensorZ);
            ap.machineX = p[0]; ap.machineY = p[1]; ap.machineZ = p[2]; points.append(ap);
        }
    }
    if (!points.isEmpty()) {
        { QMutexLocker lock(&m_savedMutex); m_savedAligned.append(points); }
        emit alignedDataReady(points);
    }
}


bool SoftwareSyncStrategy::interpolatePoseAtTime(qint64 tUs, SamplePoint& pose)
{
    QMutexLocker lock(&m_poseRingMutex);
    const int n = m_poseRingCount, cap = m_poseRing.size(); if (n < 2 || cap == 0)return false;
    const int first = (m_poseRingHead - n + cap) % cap;
    SamplePoint prev = m_poseRing[first];
    if (tUs < prev.tUs)return false;
    for (int i = 1; i < n; ++i) {
        const auto& next = m_poseRing[(first + i) % cap];
        if (next.tUs >= tUs) {
            const qint64 dt = next.tUs - prev.tUs;
            if (dt <= 0 || dt > 50000 || !prev.hasRotary || !next.hasRotary)return false;
            const double u = double(tUs - prev.tUs) / dt;
            pose = prev; pose.tUs = tUs;
            pose.x += u * (next.x - prev.x); pose.y += u * (next.y - prev.y); pose.z += u * (next.z - prev.z);
            // Mechanical angles are continuous (no shortest-path +/-180 rewriting).
            pose.c += u * (next.c - prev.c); pose.b += u * (next.b - prev.b);
            for (int j = 0; j < 5; ++j) { pose.frame.command[j] += u * (next.frame.command[j] - prev.frame.command[j]); pose.frame.raw[j] += u * (next.frame.raw[j] - prev.frame.raw[j]); }
            pose.frame.timing = mls::runtime::PoseTiming::SoftwareInterpolated; pose.frame.hostEstimateUs = tUs; pose.frame.latchMask = 0; pose.frame.commandMask = mls::runtime::Table5D::defaultCommandMask;
            return true;
        }
        prev = next;
    }
    return false;
}


// ============================================================================
// 硬件同步 - TABLE 读取线程
// ============================================================================
// 
// ECI3808 TABLE读取周期与通信负载：保留本项目已验证的10ms目标周期。
// 既往项目记录：2ms轮询曾少30~40个Profile，5ms曾少5~8个，8~10ms数量匹配。
// 这些是特定工况现象，不证明卡CPU架构、优先级或“通信必然延迟中断”的因果。
// V19：读取线程向单一I/O工作线程提交Measurement请求；不再自己进入ZAux。
// 每个硬件触发周期生成一条记录；主机/网络延迟后可一次补读更多记录。
// 环形缓冲缓解短时延迟，但不能保证无限吞吐或无条件不溢出；溢出仍锁存故障。
// ============================================================================

class HardwareSyncStrategy::TableReadThread : public QThread
{
public:
    explicit TableReadThread(HardwareSyncStrategy* owner)
        : QThread(owner), m_owner(owner) {
    }
    void requestStop() { m_stop.store(true); }
protected:
    void run() override
    {
#ifdef Q_OS_WIN
        timeBeginPeriod(1);
#endif
        setPriority(QThread::HighPriority); // V27.6.16M: do not outrank the LLT decode/receive path
        QElapsedTimer timer;
        timer.start();
        constexpr qint64 kReadPeriodUs = Constants::kHwTableReadIntervalMs * 1000LL;
        qint64 nextWakeUs = kReadPeriodUs;
        int consecutiveErrors = 0;
        // A conservative host-side watchdog. This does not replace a true
        // monotonic hardware counter for each ECI IN1 edge (BASIC has none).
        // If the TABLE reader stops making progress for HALF the 128-latch
        // buffer residence window, the scan can no longer claim completeness.
        const qint64 fifoGuardUs =
            (128LL * 1000000LL)
            / (2 * std::max(1, m_owner->m_captureTriggerFrequencyHz));
        qint64 lastCompletedReadUs = 0;
        while (!m_stop.load() && m_owner->m_running.load()
            && !m_owner->m_alignmentFault.load()) {
            const qint64 remaining = nextWakeUs - timer.nsecsElapsed() / 1000;
            if (remaining > 0) QThread::usleep(static_cast<unsigned long>(remaining));
            if (m_stop.load() || m_owner->m_alignmentFault.load()) break;

            QVector<SamplePoint> newSamples;
            bool overflow = false;
            QString err;
            const bool ok = m_owner->m_zmc->hwTableSampleReadOnce(newSamples, overflow, &err);
            const qint64 completedUs = timer.nsecsElapsed() / 1000;
            if (lastCompletedReadUs > 0 &&
                completedUs - lastCompletedReadUs >= fifoGuardUs &&
                (m_owner->m_totalProfileCount.load() > 0 ||
                    m_owner->m_poseSampleCount.load() > 0)) {
                m_owner->invalidateAlignment(
                    tr("ECI TABLE读取停顿%1us，达到128点原始FIFO保守半周期阈值%2us；"
                        "即使读指针表面连续，也不能排除整圈回绕，本次扫描无效")
                    .arg(completedUs - lastCompletedReadUs).arg(fifoGuardUs));
                break;
            }
            if (ok) lastCompletedReadUs = completedUs;
            if (overflow) {
                m_owner->m_overflowCount.fetch_add(1); // sticky card flag: count once/session
                m_owner->invalidateAlignment(tr("TABLE 溢出：本扫描段失效；禁止重新映射到 pose[0]"));
                break;
            }
            if (!ok) {
                m_owner->m_tableReadErrors.fetch_add(1);
                ++consecutiveErrors;
                if (consecutiveErrors == 1) {
                    LOG_WARNING(LogCategory::Sync, tr("TABLE 读取失败：%1").arg(err));
                }
                if (m_owner->m_zmc->hwTableReadFaulted() || consecutiveErrors >= 10) {
                    m_owner->invalidateAlignment(tr("TABLE 读取无法安全继续（连续失败%1次）：%2")
                        .arg(consecutiveErrors).arg(err));
                    break;
                }
            }
            else {
                if (consecutiveErrors > 0) {
                    LOG_INFO(LogCategory::Sync, tr("TABLE 读取恢复；之前连续失败%1次，未发布半批次数据")
                        .arg(consecutiveErrors));
                }
                consecutiveErrors = 0;
                if (!newSamples.isEmpty() && !m_owner->m_alignmentFault.load()) {
                    bool valid = true;
                    for (auto& sample : newSamples) {
                        QString detail; auto frame = m_owner->calibratedFrame(sample, detail);
                        if (!frame) { m_owner->invalidateAlignment(detail); valid = false; break; }sample.frame = *frame;
                    }
                    if (!valid)break;
                    QMutexLocker locker(&m_owner->m_poseMutex);
                    m_owner->m_allPoseSamples.append(newSamples);
                    m_owner->m_poseSampleCount.store(m_owner->m_allPoseSamples.size());
                }
                // TWO independent sources can fall behind: sensor callback
                // delivery and ECI-published poses. Large live disagreement
                // is unsafe, even when the controller's 128-slot raw pointer
                // happens to equal a previous pointer after a full rotation.
                const qint64 profiles = m_owner->m_totalProfileCount.load();
                const qint64 poses = m_owner->m_poseSampleCount.load();
                const qint64 skew = profiles >= poses ? profiles - poses : poses - profiles;
                if (skew >= 96 && !m_owner->m_alignmentFault.load()) {
                    m_owner->invalidateAlignment(
                        tr("硬件采集Profile/Pose实时积压%1条(>=96)，"
                            "128点FIFO发生不可证实的覆盖风险；禁止继续扫描")
                        .arg(skew));
                    break;
                }
            }
            nextWakeUs += kReadPeriodUs;
            const qint64 now = timer.nsecsElapsed() / 1000;
            // A slow SDK call must not produce a burst of catch-up TABLE requests.
            if (nextWakeUs < now) nextWakeUs = now + kReadPeriodUs;
        }
#ifdef Q_OS_WIN
        timeEndPeriod(1);
#endif
    }
private:
    HardwareSyncStrategy* m_owner;
    std::atomic_bool m_stop{ false };
};

// ============================================================================
// 硬件同步策略
// ============================================================================

HardwareSyncStrategy::HardwareSyncStrategy(LaserSensor* laser, ZmcController* zmc, QObject* parent)
    : ISyncStrategy(parent), m_laser(laser), m_zmc(zmc)
{
    // V27.6.16M arms the profile connection only for an active hardware session.
    // The decoded-profile ingress runs directly in ProfileDecodeThread and only
    // performs mutex/atomic bookkeeping; heavy point transformation remains on
    // the 30 ms owner-thread timer. Thus GUI stalls cannot delay index accounting.

    // 30ms 对齐处理周期（不需要太频繁，对齐计算在主线程）
    m_alignTimer.setInterval(30);
    connect(&m_alignTimer, &QTimer::timeout, this, &HardwareSyncStrategy::onAlignTimer);
}

HardwareSyncStrategy::~HardwareSyncStrategy() { stop(); }

void HardwareSyncStrategy::restartProbeTimeout() { m_probeStartedUs = GlobalTimeSync::instance().timestampUs(); }

bool HardwareSyncStrategy::start(QString* errMsg)
{
    if (m_running.load() || m_tableThread) {
        if (errMsg) *errMsg = tr("硬件同步/读取线程尚未停止");
        return false;
    }
    if (!m_modelConfigured) { if (errMsg)*errMsg = tr("请先应用完整17参数扫描标定"); return false; }
    m_syncEstablished.store(false);
    m_firstProfileIndex = 0;
    m_ingressSession.end();
    m_faultReported = false;
    { QMutexLocker lock(&m_faultMutex); m_faultReason.clear(); m_alignmentFault.store(false); }
    m_totalProfileCount.store(0);
    m_poseSampleCount.store(0);
    m_alignedCount.store(0);
    m_validAlignedCount.store(0);
    m_emptyAlignedCount.store(0);
    m_temporalBoundarySkippedCount.store(0);
    m_droppedCount.store(0);
    m_overflowCount.store(0);
    m_missingIndexCount.store(0);
    m_duplicateCount.store(0);
    m_outOfOrderCount.store(0);
    m_invalidIndexCount.store(0);
    m_tableReadErrors.store(0);
    m_cachedTriggerCount.store(0);
    m_cachedTriggerCountValid.store(false);
    {
        QMutexLocker lock(&m_profileMutex);
        m_pendingProfiles.clear(); m_profileCounter.reset();
        m_cmmCounterSeen = false; m_lastCmmCounter = 0; m_contiguousCmmProfiles = 0;
    }
    { QMutexLocker lock(&m_poseMutex); QVector<SamplePoint>().swap(m_allPoseSamples); m_allPoseSamples.reserve(Constants::kInitialPoseReserve); }
    { QMutexLocker lock(&m_alignedMutex); QVector<AlignedPoint>().swap(m_allAlignedPoints); m_allAlignedPoints.reserve(Constants::kInitialAlignedReserve); }
    { QMutexLocker lock(&m_rawCaptureMutex); QVector<LaserProfile>().swap(m_rawCapturedProfiles); }
    { QMutexLocker lock(&m_receivedIndicesMutex); m_allReceivedProfileIndices.clear(); m_profileRecords.clear(); }

    m_running.store(true);
    m_ingressSession.begin(); // arm reception before the card can send its first pulse
    m_profileConnection = connect(m_laser, &LaserSensor::profileReceived, this,
        [this](const LaserProfile& profile) {
            const auto token = m_ingressSession.current();
            if (m_ingressSession.accepts(token)) onProfileReceived(profile);
        }, Qt::DirectConnection);
    QString err;
    if (!m_zmc->hwTableSampleStart(&err)) {
        m_ingressSession.end();
        if (m_profileConnection) { disconnect(m_profileConnection); m_profileConnection = {}; }
        m_running.store(false);
        if (errMsg) *errMsg = tr("启动 TABLE 采样失败：%1").arg(err);
        return false;
    }
    // Probe timeout starts only after the controller has completed the new-session
    // handshake and has actually entered probe mode. Do not charge handshake time
    // against the one-profile synchronization proof.
    m_probeStartedUs = GlobalTimeSync::instance().timestampUs();
    m_tableThread = new TableReadThread(this);
    m_tableThread->start();
    m_alignTimer.start();
    return true;
}

void HardwareSyncStrategy::stop()
{
    if (!m_running.load() && !m_tableThread) return;
    // Trigger shutdown and final sensor queue draining are done by SyncManager.
    m_alignTimer.stop();
    if (m_tableThread) {
        m_tableThread->requestStop();
        if (!m_tableThread->wait(3000)) {
            LOG_WARNING(LogCategory::Sync, tr("TABLE 线程退出超过3秒；继续等待，禁止删除运行中的QThread"));
            m_tableThread->wait();
        }
        delete m_tableThread;
        m_tableThread = nullptr;
    }
    m_ingressSession.end();
    if (m_profileConnection) { disconnect(m_profileConnection); m_profileConnection = {}; }
    m_running.store(false);
}

void HardwareSyncStrategy::clearData()
{
    if (m_running.load() || m_tableThread) {
        LOG_WARNING(LogCategory::Sync, tr("无法在运行时清除数据"));
        return;
    }
    m_syncEstablished.store(false);
    m_firstProfileIndex = 0;
    m_ingressSession.end();
    m_faultReported = false;
    { QMutexLocker lock(&m_faultMutex); m_faultReason.clear(); m_alignmentFault.store(false); }
    m_totalProfileCount.store(0);
    m_poseSampleCount.store(0);
    m_alignedCount.store(0);
    m_validAlignedCount.store(0);
    m_emptyAlignedCount.store(0);
    m_temporalBoundarySkippedCount.store(0);
    m_droppedCount.store(0);
    m_overflowCount.store(0);
    m_missingIndexCount.store(0);
    m_duplicateCount.store(0);
    m_outOfOrderCount.store(0);
    m_invalidIndexCount.store(0);
    m_tableReadErrors.store(0);
    m_cachedTriggerCount.store(0);
    m_cachedTriggerCountValid.store(false);
    {
        QMutexLocker lock(&m_profileMutex);
        m_pendingProfiles.clear(); m_profileCounter.reset();
        m_cmmCounterSeen = false; m_lastCmmCounter = 0; m_contiguousCmmProfiles = 0;
    }
    { QMutexLocker lock(&m_poseMutex); QVector<SamplePoint>().swap(m_allPoseSamples); m_allPoseSamples.reserve(Constants::kInitialPoseReserve); }
    { QMutexLocker lock(&m_alignedMutex); QVector<AlignedPoint>().swap(m_allAlignedPoints); m_allAlignedPoints.reserve(Constants::kInitialAlignedReserve); }
    { QMutexLocker lock(&m_rawCaptureMutex); QVector<LaserProfile>().swap(m_rawCapturedProfiles); }
    { QMutexLocker lock(&m_receivedIndicesMutex); m_allReceivedProfileIndices.clear(); m_profileRecords.clear(); }
    // No controller reset/write here; V17 resets the counter only on Start.
    LOG_DEBUG(LogCategory::Sync, tr("硬件同步数据已清除"));
}

void HardwareSyncStrategy::finalizeAlignment()
{
    if (!m_alignmentFault.load()) processProfiles(true); // final pass may pair-but-skip the last look-ahead boundary
    qint64 count = 0;
    const bool counterRead = m_zmc->hwGetTriggerCount(count, nullptr);
    m_cachedTriggerCount.store(count);
    m_cachedTriggerCountValid.store(counterRead);
    reportAlignmentFault();
    const auto stats = getAlignStats();
    if (stats.alignmentFault) {
        LOG_WARNING(LogCategory::Sync, tr("本次扫描无效，现有点云仅供诊断：%1").arg(stats.faultReason));
        return;
    }
    const bool indicesContinuous = stats.missingProfileIndices == 0
        && stats.duplicateProfiles == 0 && stats.outOfOrderProfiles == 0
        && stats.invalidProfileIndices == 0;
    const bool countsMatch = stats.totalProfiles > 0
        && stats.totalPoseSamples == stats.totalProfiles
        && stats.alignedProfiles == stats.totalProfiles && stats.pendingProfiles == 0
        && stats.droppedProfiles == 0 && stats.zmcTriggerCountValid
        && stats.zmcTriggerCount == stats.totalProfiles;
    if (indicesContinuous && countsMatch) {
        LOG_INFO(LogCategory::Sync, m_hardwareTopology == mls::runtime::HardwareSyncTopology::SensorInternalCmm
                ? tr("H3五轴MPOS Probe与索引检查通过；CMM/IN1/R1锁存物理相位仍需示波器验证")
                : tr("H1五轴MPOS Probe与索引检查通过；OP14/IN0/R0锁存物理相位仍需示波器验证"));
    }
    else {
        invalidateAlignment(tr("最终同步完整性失败：Pose=%1 Profile=%2 Aligned=%3 Pending=%4；禁止保存为有效扫描")
            .arg(stats.totalPoseSamples).arg(stats.totalProfiles).arg(stats.alignedProfiles).arg(stats.pendingProfiles));
        reportAlignmentFault();
    }
}

void HardwareSyncStrategy::getStats(int& profileCount, int& poseCount) const
{
    profileCount = static_cast<int>(m_totalProfileCount.load());
    poseCount = static_cast<int>(m_poseSampleCount.load());
}

void HardwareSyncStrategy::setMountOffset(const MountOffset& offset)
{
    QMutexLocker locker(&m_mountOffsetMutex);
    m_mountOffset = offset;
}

HardwareSyncStrategy::AlignStats HardwareSyncStrategy::getAlignStats() const
{
    AlignStats stats;
    stats.totalPoseSamples = m_poseSampleCount.load();
    stats.totalProfiles = m_totalProfileCount.load();
    stats.alignedProfiles = m_alignedCount.load();
    stats.validAlignedProfiles = m_validAlignedCount.load();
    stats.emptyAlignedProfiles = m_emptyAlignedCount.load();
    stats.temporalBoundarySkippedProfiles = m_temporalBoundarySkippedCount.load();
    stats.droppedProfiles = m_droppedCount.load();
    stats.tableOverflows = m_overflowCount.load();
    stats.syncEstablished = m_syncEstablished.load();
    stats.missingProfileIndices = m_missingIndexCount.load();
    stats.duplicateProfiles = m_duplicateCount.load();
    stats.outOfOrderProfiles = m_outOfOrderCount.load();
    stats.invalidProfileIndices = m_invalidIndexCount.load();
    stats.tableReadErrors = m_tableReadErrors.load();
    stats.zmcTriggerCount = m_cachedTriggerCount.load();
    stats.zmcTriggerCountValid = m_cachedTriggerCountValid.load();
    {
        QMutexLocker lock(&m_faultMutex);
        stats.alignmentFault = m_alignmentFault.load();
        stats.faultReason = m_faultReason;
    }
    {
        QMutexLocker lock(&m_profileMutex);
        stats.firstProfileIndex = m_firstProfileIndex;
        stats.pendingProfiles = static_cast<qint64>(m_pendingProfiles.size());
    }
    return stats; // no I/O, and no profile mutex held across a controller call
}

QVector<quint32> HardwareSyncStrategy::getAllReceivedProfileIndices() const
{
    QMutexLocker locker(&m_receivedIndicesMutex);
    return m_allReceivedProfileIndices;  // 返回副本
}

void HardwareSyncStrategy::onProfileReceived(const LaserProfile& profile)
{
    if (!m_running.load()) return; // preview / another strategy must not fill this queue
    std::size_t metadataIndex = 0;
    {
        QMutexLocker lock(&m_receivedIndicesMutex);
        metadataIndex = m_profileRecords.size();
        syncdiag::ProfileRecord record;
        record.index = profile.index;
        record.shutterOpenSec = profile.tOpen;
        record.callbackPcUs = profile.tPcUs;
        record.pairing = syncdiag::PairingState::Pending;
        record.callbackOrdinal = profile.callbackOrdinal;
        record.rawBytes = profile.rawBytes;
        record.ingressDepth = profile.ingressDepth;
        record.callbackServiceUs = profile.callbackServiceUs;
        record.decoderQueueUs = profile.decoderQueueUs;
        record.decodeServiceUs = profile.decodeServiceUs;
        record.decodeDonePcUs = profile.decodeDonePcUs;
        m_profileRecords.push_back(record);
        m_allReceivedProfileIndices.append(profile.index); // keep arrival order, including empty profiles
    }
    // H3 CMM first-frame diagnostics.
    // This does NOT relax the fail-closed acquisition-integrity guard.
    const qint64 sessionProfileNumber = m_totalProfileCount.fetch_add(1) + 1;
    const bool usesCmm = m_hardwareTopology ==
        mls::runtime::HardwareSyncTopology::SensorInternalCmm;
    const bool cmmMetadataOk = profile.cmmValid && profile.cmmActive;
    if (usesCmm && !m_alignmentFault.load() &&
        (sessionProfileNumber <= 3 || !cmmMetadataOk)) {
        const QString topologyName = QStringLiteral("H3");
        const QString diagnostic =
            tr("CMM帧诊断 topology=%1 sessionProfile=%2 valid=%3 active=%4 "
                "trigger=%5 count14=%6 profileIndex=%7 callbackOrdinal=%8 "
                "rawBytes=%9 poseCount=%10")
            .arg(topologyName)
            .arg(sessionProfileNumber)
            .arg(profile.cmmValid ? 1 : 0)
            .arg(profile.cmmActive ? 1 : 0)
            .arg(profile.cmmTrigger ? 1 : 0)
            .arg(profile.cmmCount)
            .arg(profile.index)
            .arg(static_cast<qulonglong>(profile.callbackOrdinal))
            .arg(static_cast<qulonglong>(profile.rawBytes))
            .arg(m_poseSampleCount.load());
        if (cmmMetadataOk) LOG_INFO(LogCategory::Sync, diagnostic);
        else LOG_WARNING(LogCategory::Sync, diagnostic);
    }
    if (usesCmm && !cmmMetadataOk) {
        invalidateAlignment(tr("CMM同步帧缺少有效CMM timestamp / active标志；拒绝盲目ordinal配对"));
        return;
    }
    auto markDropped = [&] {
        QMutexLocker lock(&m_receivedIndicesMutex);
        m_profileRecords[metadataIndex].pairing = syncdiag::PairingState::Dropped;
        };
    if (m_alignmentFault.load()) { m_droppedCount.fetch_add(1); markDropped(); return; }

    QString fault;
    {
        QMutexLocker lock(&m_profileMutex);
        const auto observed = m_profileCounter.observe(profile.index);
        using Kind = p0::ProfileCounter24::Kind;
        const bool useCmm = m_hardwareTopology == mls::runtime::HardwareSyncTopology::SensorInternalCmm;
        if (observed.accepted()) {
            if (observed.kind == Kind::First) m_firstProfileIndex = profile.index;
            m_missingIndexCount.fetch_add(observed.missing);
            // The LLT CMM impulse counter is 14-bit. A real one-per-profile
            // CMM output at divider=1 must advance by one modulo 16384.
            // This detects skipped/repeated sensor CMM events, NOT an
            // absolute pose-ordinal hardware anchor on the ECI IN1 input.
            if (useCmm) {
                if (observed.kind == Kind::Gap) {
                    fault = tr("H3 Profile Index出现缺帧%1，禁止继续按顺序号配对")
                        .arg(observed.missing);
                }
                else if (profile.cmmCount >= 16384u) {
                    fault = tr("传感器CMM 14位脉冲计数越界: %1").arg(profile.cmmCount);
                }
                else if (m_cmmCounterSeen &&
                    ((profile.cmmCount - m_lastCmmCounter) & 0x3fffu) != 1u) {
                    fault = tr("CMM脉冲计数不连续: 前=%1 本次=%2；本次采集无效")
                        .arg(m_lastCmmCounter).arg(profile.cmmCount);
                }
                if (fault.isEmpty()) {
                    m_cmmCounterSeen = true;
                    m_lastCmmCounter = profile.cmmCount;
                    ++m_contiguousCmmProfiles;
                }
            }
            if (fault.isEmpty()) m_pendingProfiles.push_back({ profile, observed.ordinal, metadataIndex });
            else { m_droppedCount.fetch_add(1); markDropped(); }
        }
        else {
            m_droppedCount.fetch_add(1);
            markDropped();
            if (observed.kind == Kind::Duplicate) {
                m_duplicateCount.fetch_add(1);
                if (useCmm) fault = tr("H3重复Profile Index，无法证明CMM/ECI对齐");
            }
            else if (observed.kind == Kind::Invalid) {
                m_invalidIndexCount.fetch_add(1);
                fault = tr("Profile 索引超出V17约定的24bit范围，禁止继续对齐");
            }
            else {
                m_outOfOrderCount.fetch_add(1);
                fault = tr("Profile 索引倒退/重置或大跳变，不能判定对应关系；本段失效");
            }
        }
    }

    if (metadataIndex > 0) {
        syncdiag::ProfileRecord previous;
        syncdiag::ProfileRecord current;
        {
            QMutexLocker lock(&m_receivedIndicesMutex);
            previous = m_profileRecords[metadataIndex - 1];
            current = m_profileRecords[metadataIndex];
        }
        const quint32 deltaIndex = (current.index - previous.index) & p0::ProfileCounter24::kMask;
        if (deltaIndex > 1 && deltaIndex < p0::ProfileCounter24::kModulus / 2) {
            const quint64 deltaCallback = current.callbackOrdinal >= previous.callbackOrdinal
                ? current.callbackOrdinal - previous.callbackOrdinal : 0;
            LOG_WARNING(LogCategory::Sync,
                tr("Profile gap forensic: Index %1→%2 ΔIndex=%3; callbackOrdinal %4→%5 ΔCallback=%6; "
                    "callbackΔ=%7us; sensorΔ=%8ms; ingressDepth=%9→%10; callbackService=%11/%12us; "
                    "decoderQueue=%13/%14us; decodeService=%15/%16us")
                .arg(previous.index).arg(current.index).arg(deltaIndex)
                .arg(previous.callbackOrdinal).arg(current.callbackOrdinal).arg(deltaCallback)
                .arg(current.callbackPcUs - previous.callbackPcUs)
                .arg((current.shutterOpenSec - previous.shutterOpenSec) * 1000.0, 0, 'f', 3)
                .arg(previous.ingressDepth).arg(current.ingressDepth)
                .arg(previous.callbackServiceUs).arg(current.callbackServiceUs)
                .arg(previous.decoderQueueUs).arg(current.decoderQueueUs)
                .arg(previous.decodeServiceUs).arg(current.decodeServiceUs));
        }
    }
    // Invalid/reset indices are already a fatal acquisition condition. Keep the normal
    // profile path allocation-light and non-blocking; on this exceptional path we fail
    // closed immediately so no later frame can be accepted as if the ordinal were valid.
    if (!fault.isEmpty()) invalidateAlignment(fault);
}

void HardwareSyncStrategy::onAlignTimer()
{
    if (!m_running.load()) return;
    if (m_alignmentFault.load()) { reportAlignmentFault(); return; }
    if (!m_syncEstablished.load() && GlobalTimeSync::instance().timestampUs() - m_probeStartedUs > (m_hardwareTopology == mls::runtime::HardwareSyncTopology::SensorInternalCmm ? 2500000 : 1000000)) {
        const auto cb = m_laser->profileCallbackDiagnostics();
        const auto hw = m_laser->hardwareTriggerDiagnostics();
        invalidateAlignment(
            tr("V32.16 H1/H3正式Probe等待超过1秒：Pose=%1，sensorRawCallback=%2，convertedProfile=%3，convertFailure=%4，lastConvertRet=%5，lastBytes=%6，resolution=%7；TRIGGER req=%8 read=%9，DIGITAL_IO req=%10 read=%11，readbackValid=%12。")
            .arg(m_poseSampleCount.load())
            .arg(cb.rawCallbacks).arg(cb.convertedProfiles).arg(cb.conversionFailures)
            .arg(cb.lastConvertResult).arg(cb.lastCallbackBytes).arg(cb.lastResolution)
            .arg(hw.triggerRequested).arg(hw.triggerReadback)
            .arg(hw.digitalIoRequested).arg(hw.digitalIoReadback)
            .arg(hw.readbackValid ? 1 : 0));
        reportAlignmentFault();
        return;
    }
    processProfiles();
}

bool HardwareSyncStrategy::tryEstablishSync()
{
    if (m_alignmentFault.load())return false;
    { QMutexLocker lock(&m_profileMutex); if (m_pendingProfiles.empty() || m_poseSampleCount.load() < 1)return false; }
    const bool continuous = (m_hardwareTopology == mls::runtime::HardwareSyncTopology::SensorInternalCmm);
    if (continuous) {
        // H3 never pauses its internal 200 Hz clock: wait for a stable sample
        // window. All records still share ordinal zero established after CMM
        // gating by StartTransmissionAndCmmTrigger().
        const auto profiles = m_totalProfileCount.load();
        const auto poses = m_poseSampleCount.load();
        if (profiles < 8 || poses < 8) return false;
        // Unlike the old >=8 && >=8 check, do not authorize H3 Run
        // while the two session streams have different observed lengths.
        // The CMM 14-bit counter continuity above supplies an independent
        // sensor-side check; it is NOT an independent ECI edge ordinal.
        if (profiles != poses) return false;
        {
            QMutexLocker lock(&m_profileMutex);
            if (!m_cmmCounterSeen || m_contiguousCmmProfiles != profiles) return false;
        }
        if (m_missingIndexCount.load() != 0 || m_duplicateCount.load() != 0 ||
            m_outOfOrderCount.load() != 0 || m_invalidIndexCount.load() != 0) {
            invalidateAlignment(tr("H3连续Profile索引未连续，禁止建立ordinal配对")); return false;
        }
    }
    else if (m_totalProfileCount.load() != 1 || m_poseSampleCount.load() != 1) {
        invalidateAlignment(tr("H1正式Probe未得到恰好1条轮廓及1条五轴锁存位姿")); return false;
    }
    QString err;
    if (m_running.load() && !m_zmc->hwTableContinueAfterProbe(&err)) {
        invalidateAlignment(tr("启动探针握手失败：%1").arg(err)); return false;
    }
    m_syncEstablished.store(true);
    LOG_INFO(LogCategory::Sync, continuous ? tr("H3 Sensor-master CMM连续帧窗口验证通过，ECI IN1五轴R1锁存流继续运行") : tr("H1单帧Probe已通过，开始正式采集"));
    return true;
}


void HardwareSyncStrategy::processProfiles(bool finalPass)
{
    if (m_alignmentFault.load()) return;
    if (!m_syncEstablished.load() && !tryEstablishSync()) return;
    QVector<AlignedPoint> newAligned;
    QVector<LaserProfile> newRawProfiles;
    const bool rawProfileMode = m_rawProfileCaptureEnabled.load();
    qint64 completedProfiles = 0;
    struct CompletedMeta { std::size_t index = 0; std::uint32_t validPoints = 0; bool temporalBoundarySkipped = false; };
    std::vector<CompletedMeta> completedMetadata;

    while (!m_alignmentFault.load()) {
        PendingProfile pending;
        SamplePoint pose;
        SamplePoint nextPose;
        bool boundarySkip = false;
        {
            // Consistent lock order; no device I/O while either data lock is held.
            QMutexLocker profileLock(&m_profileMutex);
            QMutexLocker poseLock(&m_poseMutex);
            if (m_pendingProfiles.empty()) break;
            const auto ordinal = m_pendingProfiles.front().poseOrdinal;
            if (ordinal < 0 || ordinal >= m_allPoseSamples.size()) break;

            if (m_temporalCorrectionEnabled) {
                // Ordinal 0 is the isolated Probe, not a regular 5ms neighbor.
                // Positive tau uses next latch; negative tau uses previous latch.
                if (m_temporalDelaySec >= 0.0) {
                    if (ordinal == 0) boundarySkip = true;
                    else if (ordinal + 1 >= m_allPoseSamples.size()) {
                        if (!finalPass) break; // wait for next continuous-stage latch
                        boundarySkip = true;   // stop tail has no future latch
                    }
                } else if (ordinal < 2) {
                    boundarySkip = true; // H3 negative tau: do not interpolate with Probe
                }
            }

            pending = std::move(m_pendingProfiles.front());
            m_pendingProfiles.pop_front();
            pose = m_allPoseSamples[static_cast<qsizetype>(ordinal)];
            if (m_temporalCorrectionEnabled && !boundarySkip) {
                const auto adjacent = m_temporalDelaySec >= 0.0 ? ordinal + 1 : ordinal - 1;
                nextPose = m_allPoseSamples[static_cast<qsizetype>(adjacent)];
            }
        }

        if (pose.frame.ordinal != static_cast<std::uint64_t>(pending.poseOrdinal)) {
            invalidateAlignment(tr("轮廓与控制器publishedLatchOrdinal不一致")); break;
        }

        // V27.6.48 temporal-calibration raw mode.  The hardware pairing proof is
        // completed here, but the millions of Machine-frame points are not
        // materialized on this QObject/GUI thread.  Store exactly one decoded
        // LaserProfile per trigger; the worker maps a bounded sample after Stop.
        if (rawProfileMode) {
            if (m_temporalCorrectionEnabled) {
                invalidateAlignment(tr("RAW时域标定采集禁止同时启用运行时τ补偿"));
                break;
            }
            const auto& profile = pending.profile;
            const int pointCount = static_cast<int>(qMin(profile.x.size(), profile.z.size()));
            std::uint32_t validPoints = 0;
            for (int k = 0; k < pointCount; ++k)
                if (SensorUtils::isValidPoint(profile.x[k], profile.z[k])) ++validPoints;
            newRawProfiles.push_back(std::move(pending.profile));
            completedMetadata.push_back({ pending.metadataIndex, validPoints, false });
            ++completedProfiles;
            continue;
        }

        if (boundarySkip) {
            completedMetadata.push_back({ pending.metadataIndex, 0u, true });
            ++completedProfiles;
            continue;
        }

        SamplePoint reconstructionPose = pose;
        if (m_temporalCorrectionEnabled) {
            mls::runtime::HardwareTemporalCorrectionConfig cfg;
            cfg.enabled = true;
            cfg.delaySec = m_temporalDelaySec;
            cfg.triggerPeriodSec = m_temporalTriggerPeriodSec;
            const auto corrected = mls::runtime::interpolateHardwareTemporalFrame(
                pose.frame, nextPose.frame, cfg);
            if (!corrected.success) {
                invalidateAlignment(tr("硬件同步时域补偿失败：%1")
                    .arg(QString::fromStdString(corrected.message)));
                break;
            }
            reconstructionPose.frame = corrected.frame;
            reconstructionPose.x = corrected.frame.joints[0];
            reconstructionPose.y = corrected.frame.joints[1];
            reconstructionPose.z = corrected.frame.joints[2];
            reconstructionPose.c = corrected.frame.joints[3];
            reconstructionPose.b = corrected.frame.joints[4];
            reconstructionPose.hasRotary = true;
        }

        QString transformError;
        const auto frame = calibratedFrame(reconstructionPose, transformError);
        if (!frame) { invalidateAlignment(transformError); break; }

        const auto& profile = pending.profile;
        const int pointCount = static_cast<int>(qMin(profile.x.size(), profile.z.size()));
        std::uint32_t validPoints = 0;
        for (int k = 0; k < pointCount; ++k) {
            const double sx = profile.x[k];
            const double sz = profile.z[k];
            if (!SensorUtils::isValidPoint(sx, sz)) continue;
            AlignedPoint ap;
            // tUs remains the host estimate of the same-ordinal raw latch.  It is not
            // promoted to hardware time; calibrated temporal provenance lives in Frame5D.
            ap.tUs = pose.tUs;
            ap.tSensorSec = profile.tOpen;
            ap.profileIndex = profile.index;
            ap.pointIndex = static_cast<quint32>(k);
            ap.sensorX = sx;
            ap.sensorZ = sz;
            ap.frame = frame;
            const auto mapped = mls::runtime::mapProfilePoint(*frame, sx, sz);
            ap.machineX = mapped[0]; ap.machineY = mapped[1]; ap.machineZ = mapped[2];
            newAligned.append(ap);
            ++validPoints;
        }
        completedMetadata.push_back({ pending.metadataIndex, validPoints, false });
        ++completedProfiles; // empty profiles still occupy a trigger/pose index
    }

    if (m_alignmentFault.load()) {
        {
            QMutexLocker lock(&m_receivedIndicesMutex);
            for (const auto& item : completedMetadata) m_profileRecords[item.index].pairing = syncdiag::PairingState::Dropped;
        }
        m_droppedCount.fetch_add(completedProfiles);
        return;
    }
    {
        QMutexLocker lock(&m_receivedIndicesMutex);
        for (const auto& item : completedMetadata) {
            auto& record = m_profileRecords[item.index];
            record.pairing = syncdiag::PairingState::Aligned;
            record.validPoints = item.validPoints;
            record.temporalBoundarySkipped = item.temporalBoundarySkipped;
            if (item.temporalBoundarySkipped) {
                m_temporalBoundarySkippedCount.fetch_add(1);
            }
            else if (item.validPoints) {
                m_validAlignedCount.fetch_add(1);
            }
            else {
                m_emptyAlignedCount.fetch_add(1);
            }
        }
    }
    m_alignedCount.fetch_add(completedProfiles);
    if (!newRawProfiles.isEmpty()) {
        QMutexLocker lock(&m_rawCaptureMutex);
        for (auto& profile : newRawProfiles) m_rawCapturedProfiles.push_back(std::move(profile));
    }
    if (!newAligned.isEmpty()) {
        { QMutexLocker lock(&m_alignedMutex); m_allAlignedPoints.append(newAligned); }
        // V27.6.48: automatic temporal-calibration scans suppress this fan-out.
        // The authoritative capture remains lossless in m_allAlignedPoints and
        // is transferred after Stop/Drain.  This avoids PointCloudBuilder + GUI
        // queued-event duplication of million-point captures.
        if (m_emitAlignedData.load()) emit alignedDataReady(newAligned);
    }
}

void HardwareSyncStrategy::invalidateAlignment(const QString& reason)
{
    {
        QMutexLocker lock(&m_faultMutex);
        if (m_alignmentFault.load()) return;
        m_faultReason = reason;
        m_alignmentFault.store(true); // permanent until a deliberate new session
        m_syncEstablished.store(false);
    }
    m_zmc->requestStopAllAxes(false);
    QString stopError;
    if (!m_zmc->hwTableSampleStop(&stopError)) {
        QMutexLocker lock(&m_faultMutex);
        m_faultReason += tr("；停止触发命令失败：%1。请保持设备停用状态并由合格人员处置。").arg(stopError);
    }
    // Keep running/invalid until operator Stop drains and joins. Do not enable
    // unrelated UI motion controls while the reader is exiting on this fault.
}

void HardwareSyncStrategy::reportAlignmentFault()
{
    if (!m_alignmentFault.load() || m_faultReported) return;
    m_faultReported = true;
    {
        QMutexLocker lock(&m_profileMutex);
        m_droppedCount.fetch_add(static_cast<qint64>(m_pendingProfiles.size()));
        {
            QMutexLocker recordsLock(&m_receivedIndicesMutex);
            for (const auto& pending : m_pendingProfiles)
                m_profileRecords[pending.metadataIndex].pairing = syncdiag::PairingState::Dropped;
        }
        m_pendingProfiles.clear();
    }
    QString reason;
    { QMutexLocker lock(&m_faultMutex); reason = m_faultReason; }
    LOG_WARNING(LogCategory::Sync, tr("采集完整性故障已锁存：%1").arg(reason));
    emit errorOccurred(tr("采集无效：%1。请停止本次采集并重新开始；原有点云仅供诊断。").arg(reason));
}

QVector<AlignedPoint> HardwareSyncStrategy::getAllAlignedPoints() const
{
    QMutexLocker lock(&m_alignedMutex);
    return m_allAlignedPoints;
}

QVector<SamplePoint> HardwareSyncStrategy::getAllPoseSamples() const
{
    QMutexLocker lock(&m_poseMutex);
    return m_allPoseSamples;
}

bool HardwareSyncStrategy::takeCompletedCapture(QVector<AlignedPoint>& aligned,
    QVector<SamplePoint>& poses,
    QString* error)
{
    if (m_running.load() || m_tableThread) {
        if (error) *error = tr("硬件采集仍在运行，不能转移完成数据");
        return false;
    }

    // The hardware session is stopped, therefore no producer can append while
    // these swaps are performed.  We still take the normal data locks to keep
    // the API race-proof against diagnostics/export callers.
    {
        QMutexLocker lock(&m_alignedMutex);
        aligned.clear();
        aligned.swap(m_allAlignedPoints);
    }
    {
        QMutexLocker lock(&m_poseMutex);
        poses.clear();
        poses.swap(m_allPoseSamples);
    }

    if (error) error->clear();
    return true;
}

bool HardwareSyncStrategy::takeCompletedRawCapture(QVector<LaserProfile>& profiles,
    QVector<SamplePoint>& poses,
    QString* error)
{
    if (m_running.load() || m_tableThread) {
        if (error) *error = tr("硬件采集仍在运行，不能转移RAW Profile数据");
        return false;
    }
    {
        QMutexLocker lock(&m_rawCaptureMutex);
        profiles.clear();
        profiles.swap(m_rawCapturedProfiles);
    }
    {
        QMutexLocker lock(&m_poseMutex);
        poses.clear();
        poses.swap(m_allPoseSamples);
    }
    if (profiles.size() != poses.size()) {
        if (error) *error = tr("RAW Profile/REGIST pose数量不一致：Profile=%1 Pose=%2")
            .arg(profiles.size()).arg(poses.size());
        return false;
    }
    if (error) error->clear();
    return true;
}

bool HardwareSyncStrategy::latestPoseSample(SamplePoint& sample) const
{
    QMutexLocker lock(&m_poseMutex);
    if (m_allPoseSamples.isEmpty()) return false;
    sample = m_allPoseSamples.last();
    return sample.hasRotary;
}

// ============================================================================
// 同步管理器
// ============================================================================

SyncManager::SyncManager(LaserSensor* laser, ZmcController* zmc, QObject* parent)
    : QObject(parent), m_laser(laser), m_zmc(zmc)
{
    m_softwareStrategy = std::make_unique<SoftwareSyncStrategy>(m_laser, m_zmc, this);
    m_hardwareStrategy = std::make_unique<HardwareSyncStrategy>(m_laser, m_zmc, this);

    connect(m_softwareStrategy.get(), &ISyncStrategy::alignedDataReady, this, &SyncManager::alignedDataReady);

    connect(m_hardwareStrategy.get(), &ISyncStrategy::alignedDataReady, this, &SyncManager::alignedDataReady);
    auto captureError = [this](const QString& message) {
        m_captureFault = true; m_captureFaultReason = message;
        m_zmc->requestStopAllAxes(false);
        emit errorOccurred(message);
        QTimer::singleShot(0, this, [this]() {if (isRunning())stop(); });
        };
    connect(m_softwareStrategy.get(), &ISyncStrategy::errorOccurred, this, captureError);
    connect(m_hardwareStrategy.get(), &ISyncStrategy::errorOccurred, this, captureError);
    connect(m_laser, &LaserSensor::profileIngressFault, this, [this](const QString& reason) {
        if (m_syncMode == SyncMode::Hardware && m_hardwareStrategy->isRunning()) {
            m_hardwareStrategy->invalidateAlignment(reason);
            m_hardwareStrategy->reportAlignmentFault();
        }
        else if (m_syncMode == SyncMode::Software && m_softwareStrategy->isRunning()) {
            emit errorOccurred(reason);
            QTimer::singleShot(0, this, [this]() { if (isRunning()) stop(); });
        }
        }, Qt::QueuedConnection);
}

SyncManager::~SyncManager() { stop(); }

void SyncManager::setSyncMode(SyncMode mode)
{
    if (isRunning()) { LOG_WARNING(LogCategory::Sync, tr("无法在运行时更改同步模式")); return; }
    m_syncMode = mode;
}

bool SyncManager::setHardwareTemporalCorrection(const HardwareTemporalCorrectionSettings& settings, QString* errMsg)
{
    if (isRunning() || m_stopping) {
        if (errMsg) *errMsg = tr("采集运行/停止收尾期间禁止切换硬件同步时域补偿");
        return false;
    }
    const bool hasCalibrationPayload = settings.enabled || !settings.calibrationId.isEmpty()
        || settings.delaySec != 0.0 || settings.delayStdSec != 0.0;
    if (hasCalibrationPayload) {
        const double periodSec = settings.triggerFrequencyHz > 0
            ? 1.0 / static_cast<double>(settings.triggerFrequencyHz)
            : settings.triggerPeriodMs / 1000.0;
        if (settings.triggerFrequencyHz < mls::runtime::Table5D::minTriggerFrequencyHz ||
            settings.triggerFrequencyHz > mls::runtime::Table5D::maxTriggerFrequencyHz ||
            !std::isfinite(settings.triggerPeriodMs) || settings.triggerPeriodMs <= 0.0 ||
            !std::isfinite(settings.delaySec) || std::abs(settings.delaySec) >= periodSec ||
            !std::isfinite(settings.delayStdSec) || settings.delayStdSec < 0.0 ||
            !mls::runtime::Table5D::validTopologyEdge(
                static_cast<int>(settings.topology), settings.latchEdge) ||
            (settings.topology == mls::runtime::HardwareSyncTopology::EciPwmShared && settings.delaySec < 0.0) ||
            (settings.topology == mls::runtime::HardwareSyncTopology::SensorInternalCmm &&
                (settings.cmmDivider != 1 || settings.cmmSkewHalfUs != 0 || settings.cmmRatio != 1)) ||
            (settings.topology == mls::runtime::HardwareSyncTopology::EciPwmShared && settings.cmmDivider != 0)) {
            if (errMsg) *errMsg = tr("硬件同步标定参数无效：H1要求0<=τ<T，H3要求|τ|<T且CMM配置/锁存边沿匹配");
            return false;
        }
        if (settings.calibrationId.isEmpty()) {
            if (errMsg) *errMsg = tr("硬件同步时域标定缺少可追溯Calibration ID");
            return false;
        }
    }
    // V27.6.53: keep the loaded/calibrated delay even when runtime correction
    // is disabled.  `enabled` controls whether q(t+tau) is used; the remaining
    // fields are the persistent, inspectable active-calibration payload shown in
    // the Parameters page and reused when the operator turns compensation back on.
    m_temporalCorrection = settings;
    if (errMsg) errMsg->clear();
    return true;
}

void SyncManager::disableHardwareTemporalCorrection()
{
    if (isRunning() || m_stopping) return;
    // Do not erase tau / sigma / calibration ID.  OFF means RAW same-ordinal
    // REGIST(104) reconstruction, not "forget the active calibration".
    m_temporalCorrection.enabled = false;
}

bool SyncManager::setKinematicModel(const std::array<double, 17>& p, bool absoluteGaugeKnown, QString* err)
{
    if (isRunning() || m_stopping) { if (err)*err = tr("采集中不能替换17参数/手眼模型"); return false; }
    QString softwareError;
    if (!m_softwareStrategy->setKinematicModel(p, absoluteGaugeKnown, &softwareError)) {
        if (err)*err = softwareError;
        return false;
    }
    QString hardwareError;
    if (!m_hardwareStrategy->setKinematicModel(p, absoluteGaugeKnown, &hardwareError)) {
        QString rollbackError;
        m_softwareStrategy->clearKinematicModel(&rollbackError);
        if (err)*err = hardwareError;
        return false;
    }
    m_activePhysical17=p;
    m_modelConfigured = true;
    if (err)err->clear();
    return true;
}

bool SyncManager::clearKinematicModel(QString* err)
{
    if (isRunning() || m_stopping) { if (err)*err = tr("采集运行或停止收尾时禁止清除17参数/手眼模型"); return false; }
    QString firstError;
    if (!m_softwareStrategy->clearKinematicModel(&firstError)) {
        if (err)*err = firstError;
        return false;
    }
    QString secondError;
    if (!m_hardwareStrategy->clearKinematicModel(&secondError)) {
        if (err)*err = secondError;
        return false;
    }
    m_modelConfigured = false;
    m_activePhysical17.fill(0.0);
    if (err)err->clear();
    return true;
}

void SyncManager::setMountOffset(const MountOffset& offset)
{
    if (isRunning())return; // legacy-only setting; 17-parameter model already includes hand-eye
    m_mountOffset = offset;
    m_softwareStrategy->setMountOffset(offset);
    m_hardwareStrategy->setMountOffset(offset);
}

bool SyncManager::validateSensorParams(QString* errMsg) const
{
    if (m_syncMode != SyncMode::Hardware) return true;

    using T = mls::runtime::Table5D;
    if (m_hwTriggerFrequencyHz < T::minTriggerFrequencyHz ||
        m_hwTriggerFrequencyHz > T::maxTriggerFrequencyHz) {
        if (errMsg) *errMsg = tr("硬件同步频率%1 Hz超出控制器协议范围%2~%3 Hz")
            .arg(m_hwTriggerFrequencyHz).arg(T::minTriggerFrequencyHz).arg(T::maxTriggerFrequencyHz);
        return false;
    }

    const int fieldMaxHz = LaserSensor::getMaxFrequencyForIndex(m_sensorParams.measuringFieldIndex);
    if (fieldMaxHz <= 0) {
        if (errMsg) *errMsg = tr("Measuring Field索引%1没有项目Quick Reference最大频率，拒绝硬件同步启动")
            .arg(m_sensorParams.measuringFieldIndex);
        return false;
    }
    if (m_hwTriggerFrequencyHz > fieldMaxHz) {
        if (errMsg) *errMsg = tr("硬件同步频率%1 Hz超过当前Measuring Field索引%2的保守上限%3 Hz；请降低频率或选择更小测量区域")
            .arg(m_hwTriggerFrequencyHz).arg(m_sensorParams.measuringFieldIndex).arg(fieldMaxHz);
        return false;
    }

    // Sensor exposure/idle units are 10 us.  Use the exact frequency-derived
    // period instead of an integer millisecond constant so rates such as 144 Hz
    // are validated correctly.
    const double totalTimeUs = static_cast<double>(m_sensorParams.exposureTime + m_sensorParams.idleTime) * 10.0;
    const double triggerPeriodUs = 1.0e6 / static_cast<double>(m_hwTriggerFrequencyHz);
    if (m_hardwareTopology == mls::runtime::HardwareSyncTopology::SensorInternalCmm) {
        const int units = static_cast<int>(std::lround(100000.0 / m_hwTriggerFrequencyHz));
        if (units < 2 || static_cast<int>(m_sensorParams.exposureTime) >= units) {
            if (errMsg)*errMsg = tr("H3内部周期%1个10us单位不足以容纳曝光%2单位")
                .arg(units).arg(m_sensorParams.exposureTime);
            return false;
        }
        const double actualHz = 100000.0 / units;
        if (std::abs(actualHz - m_hwTriggerFrequencyHz) / m_hwTriggerFrequencyHz > 0.005) {
            if (errMsg)*errMsg = tr("H3内部频率量化误差超过0.5%：请求%1，实际%2")
                .arg(m_hwTriggerFrequencyHz).arg(actualHz, 0, 'f', 3);
            return false;
        }
        return true;
    }
    if (totalTimeUs >= triggerPeriodUs) {
        if (errMsg) {
            *errMsg = tr("传感器参数不匹配硬件触发频率！\n"
                "曝光时间(%1 us) + 空闲时间(%2 us) = %3 us\n"
                "必须小于%4 Hz对应的触发周期 %5 us (%6 ms)")
                .arg(m_sensorParams.exposureTime * 10)
                .arg(m_sensorParams.idleTime * 10)
                .arg(totalTimeUs, 0, 'f', 1)
                .arg(m_hwTriggerFrequencyHz)
                .arg(triggerPeriodUs, 0, 'f', 3)
                .arg(triggerPeriodUs / 1000.0, 0, 'f', 6);
        }
        return false;
    }
    return true;
}

bool SyncManager::start(QString* errMsg)
{
    if (m_stopping || isRunning()) {
        if (errMsg) *errMsg = tr("采集正在运行或停止收尾中，禁止重复启动");
        return false;
    }
    if (!m_modelConfigured) { if (errMsg)*errMsg = tr("先在标定面板加载/求解17参数结果，然后选择应用到五轴扫描"); return false; }

    // H2 (protocol ID 2) is retired. Reject stale/programmatic selections
    // before touching the controller or starting a sensor/ECI clock.
    if (m_syncMode == SyncMode::Hardware &&
        !mls::runtime::Table5D::validTopologyEdge(
            static_cast<int>(m_hardwareTopology),
            m_hardwareTopology == mls::runtime::HardwareSyncTopology::EciPwmShared
                ? mls::runtime::Table5D::h1R0Falling : m_cmmR1Edge)) {
        if (errMsg) *errMsg = tr("不支持的硬件同步拓扑：仅允许H1(1)或H3(3)，旧拓扑编号2已停用");
        return false;
    }
    // H3 uses TRIG_INTERNAL; the external sensor edge has no meaning for H3.
    const bool h1External = m_syncMode == SyncMode::Hardware &&
        m_hardwareTopology == mls::runtime::HardwareSyncTopology::EciPwmShared;
    if (h1External && !m_triggerTypeConfigured) {
        if (errMsg)*errMsg = tr("硬件同步触发类型尚未由当前启动流程显式配置；拒绝使用隐藏默认值。");
        return false;
    }
    if (h1External && m_triggerType != HardwareTriggerType::EdgeRising) {
        if (errMsg)*errMsg = tr("H1的scanCONTROL外部输入必须为EdgeRising；H3走Internal+CMM且不使用外部触发。当前界面不支持另选边沿/脉冲。");
        return false;
    }

    if (m_syncMode == SyncMode::Hardware && m_temporalCorrection.enabled) {
        const auto& c = m_temporalCorrection;
        const int currentEdge = m_hardwareTopology == mls::runtime::HardwareSyncTopology::EciPwmShared
            ? mls::runtime::Table5D::h1R0Falling : m_cmmR1Edge;
        if (c.topology != m_hardwareTopology || c.latchEdge != currentEdge ||
            (m_hardwareTopology == mls::runtime::HardwareSyncTopology::SensorInternalCmm &&
                (c.cmmDivider != 1 || c.cmmSkewHalfUs != 0 || c.cmmRatio != 1))) {
            if (errMsg)*errMsg = tr("时域标定拓扑/锁存边沿/CMM参数不匹配：标定H%1 edge=%2；当前H%3 edge=%4；必须使用同拓扑独立标定")
                .arg(static_cast<int>(c.topology)).arg(c.latchEdge)
                .arg(static_cast<int>(m_hardwareTopology)).arg(currentEdge);
            return false;
        }
        const int effectiveIdle = m_hardwareTopology == mls::runtime::HardwareSyncTopology::SensorInternalCmm
            ? int(std::lround(100000.0 / m_hwTriggerFrequencyHz)) - int(m_sensorParams.exposureTime)
            : int(m_sensorParams.idleTime);
        const bool sensorMatch = c.sensorParams.resolutionIndex == m_sensorParams.resolutionIndex
            && c.sensorParams.exposureTime == m_sensorParams.exposureTime
            && int(c.sensorParams.idleTime) == effectiveIdle
            && c.sensorParams.measuringFieldIndex == m_sensorParams.measuringFieldIndex;
        if (c.triggerFrequencyHz != m_hwTriggerFrequencyHz || !sensorMatch) {
            if (errMsg) *errMsg = tr("已加载的硬件同步时域标定与当前正式测量配置不一致：要求 field=%1 resolutionIndex=%2 exposure=%3 idle=%4 freq=%5Hz；当前=%6/%7/%8/%9/%10Hz。请重新标定或禁用时域补偿。")
                .arg(c.sensorParams.measuringFieldIndex).arg(c.sensorParams.resolutionIndex).arg(c.sensorParams.exposureTime).arg(c.sensorParams.idleTime).arg(c.triggerFrequencyHz)
                .arg(m_sensorParams.measuringFieldIndex).arg(m_sensorParams.resolutionIndex).arg(m_sensorParams.exposureTime).arg(m_sensorParams.idleTime).arg(m_hwTriggerFrequencyHz);
            return false;
        }
    }

    if (m_syncMode == SyncMode::Hardware) {
        if (!m_zmc || !m_zmc->isConnected()) {
            if (errMsg)*errMsg = tr("控制器未连接，无法执行硬件同步启动前复核");
            return false;
        }
        QString preflight, preflightErr;
        bool executionAuthorized = false;
        if (!m_zmc->hwTableCommissioningPreflight(preflight, &preflightErr, &executionAuthorized)) {
            if (errMsg)*errMsg = tr("硬件同步启动前只读复核失败：%1").arg(preflightErr);
            return false;
        }
        if (!executionAuthorized) {
            if (errMsg)*errMsg = tr("协议头匹配，但TABLE[342]=0。请下载V32.16 H1/H3 ONLY FULL-MPOS BASIC；修复后断开重连控制器或执行B/C绝对坐标重同步，软件会自动重新预检。");
            return false;
        }
        LOG_INFO(LogCategory::Sync,
            tr("V32.16 H1/H3启动前只读复核通过；BASIC/TABLE严格保存XYZ/C/B五轴MPOS。H1在正式session前通过同一OP14执行scanCONTROL冷启动预热；H3使用传感器内部时钟与CMM。"));
    }
    if (!validateSensorParams(errMsg)) return false;
    if (m_syncMode == SyncMode::Hardware) {
        QString frequencyError;
        if (!m_zmc->setHardwareAcquisitionConfig(m_hwTriggerFrequencyHz, m_hardwareTopology,
            m_hardwareTopology == mls::runtime::HardwareSyncTopology::EciPwmShared ? 104 : m_cmmR1Edge,
            &frequencyError)) {
            if (errMsg) *errMsg = tr("设置控制器硬件同步PWM频率失败：%1").arg(frequencyError);
            return false;
        }
    }

    const bool h3 = m_syncMode == SyncMode::Hardware &&
        m_hardwareTopology == mls::runtime::HardwareSyncTopology::SensorInternalCmm;
    const bool useCmm = h3;
    if (h3) {
        const int units = static_cast<int>(std::lround(100000.0 / m_hwTriggerFrequencyHz));
        m_sensorParams.idleTime = static_cast<unsigned int>(units - static_cast<int>(m_sensorParams.exposureTime));
        LOG_INFO(LogCategory::Sync, tr("H3传感器内部周期=%1×10us，曝光=%2，Idle=%3，实际频率=%4Hz")
            .arg(units).arg(m_sensorParams.exposureTime).arg(m_sensorParams.idleTime)
            .arg(100000.0 / units, 0, 'f', 4));
    }
    SensorTriggerMode triggerMode = (m_syncMode == SyncMode::Hardware && !h3)
        ? SensorTriggerMode::HardwareExternal
        : SensorTriggerMode::SoftwareInternal;

    QString err;
    m_captureFault = false; m_captureFaultReason.clear();
    m_lastCaptureSensorParams = m_sensorParams;
    m_lastCaptureTriggerFrequencyHz = m_hwTriggerFrequencyHz;
    m_lastCaptureTriggerPeriodMs = h3
        ? (m_sensorParams.exposureTime + m_sensorParams.idleTime) * 0.01
        : 1000.0 / static_cast<double>(m_hwTriggerFrequencyHz);
    m_lastCaptureTopology = m_hardwareTopology;
    m_lastCaptureCmmR1Edge = m_hardwareTopology == mls::runtime::HardwareSyncTopology::EciPwmShared
        ? mls::runtime::Table5D::h1R0Falling : m_cmmR1Edge;
    m_hardwareStrategy->setHardwareTopology(m_hardwareTopology);
    m_hardwareStrategy->m_captureTriggerFrequencyHz = m_hwTriggerFrequencyHz;
    m_lastCaptureTemporalCorrection = m_temporalCorrection;
    m_hardwareStrategy->m_temporalCorrectionEnabled = (m_syncMode == SyncMode::Hardware) && m_temporalCorrection.enabled && m_temporalCorrection.delaySec != 0.0;
    m_hardwareStrategy->m_temporalDelaySec = m_hardwareStrategy->m_temporalCorrectionEnabled ? m_temporalCorrection.delaySec : 0.0;
    m_hardwareStrategy->m_temporalTriggerPeriodSec = m_lastCaptureTriggerPeriodMs / 1000.0;

    if (m_syncMode == SyncMode::Hardware) {
        LOG_WARNING("SyncManager",
            "V32.16 H1/H3硬件同步：TABLE正式数据为五轴硬件锁存raw MPOS；H1采用REGIST(104)/IN0，H3采用REGIST(114/115)/IN1；XYZ增量光栅必须先完成DATUM并建立MPOS机床零点。普通状态/软件同步/标定同样采用实际MPOS，DPOS仅作命令与诊断。");
    }
    if (!m_laser->configure(triggerMode, m_sensorParams, true, m_triggerType, &err)) {
        if (errMsg) *errMsg = tr("配置传感器失败：%1").arg(err);
        return false;
    }
    if (m_syncMode == SyncMode::Hardware) {
        const auto hw = m_laser->hardwareTriggerDiagnostics();
        LOG_INFO(LogCategory::Sync,
            tr("V32.16 H1/H3传感器触发回读缓存: valid=%1, DIGITAL_IO req=%2 read=%3, TRIGGER req=%4 read=%5")
            .arg(hw.readbackValid ? 1 : 0)
            .arg(hw.digitalIoRequested).arg(hw.digitalIoReadback)
            .arg(hw.triggerRequested).arg(hw.triggerReadback));
    }

    if (useCmm) {
        if (h3 && !m_laser->setMeasuringFieldIndex(m_sensorParams.measuringFieldIndex, false, &err)) {
            if (errMsg)*errMsg = tr("H3设置Measuring Field失败: %1").arg(err);
            return false;
        }
        if (!m_laser->configureCmmOutput(&err)) {
            if (errMsg)*errMsg = tr("配置RS422 CMM失败: %1").arg(err);
            return false;
        }
    }
    // H1/software use ordinary profile streaming; H3 uses the LLT CMM SDK
    // after the controller's five R1 FIFOs have been armed.
    if (!h3 && !m_laser->startContinuous(&err)) {
        if (errMsg) *errMsg = tr("启动传感器失败：%1").arg(err);
        return false;
    }

    if (m_syncMode == SyncMode::Hardware && !h3) {
        // H1 only. Warm the external-trigger Profile/SDK/UDP data plane using
        // the SAME OP14 line, before the formal session clears/rearms the R0 FIFOs.
        QThread::msleep(120);
        m_laser->setProfileEmissionSuppressed(true);
        m_laser->resetProfileCallbackDiagnostics();
        QString primeErr;
        if (!m_zmc->hwSensorColdPrimeStart(&primeErr)) {
            m_laser->setProfileEmissionSuppressed(false);
            m_laser->stopContinuous();
            if (errMsg)*errMsg = tr("启动scanCONTROL冷启动预热失败：%1").arg(primeErr);
            return false;
        }

        QElapsedTimer primeTimer; primeTimer.start();
        constexpr quint64 kPrimeProfilesRequired = 8;
        bool primed = false;
        LaserSensor::ProfileCallbackDiagnostics primeDiag;
        while (primeTimer.elapsed() < 1500) {
            QThread::msleep(20);
            primeDiag = m_laser->profileCallbackDiagnostics();
            if (primeDiag.convertedProfiles >= kPrimeProfilesRequired) {
                primed = true;
                break;
            }
        }
        QString stopPrimeErr;
        const bool primeStopOk = m_zmc->hwSensorColdPrimeStop(&stopPrimeErr);
        if (!primeStopOk) {
            // Fail closed if the normal PWM stop does not reach IdleReady.
            QString fallbackErr;
            m_zmc->hwTableSampleStop(&fallbackErr);
            LOG_WARNING(LogCategory::Sync,
                tr("H1 ColdPrime停止未确认: %1；备用EN=0=%2，禁止扫描")
                .arg(stopPrimeErr, fallbackErr));
        }
        // Drain suppressed UDP/decoder tail before establishing ordinal zero.
        quint64 lastRaw = m_laser->profileCallbackDiagnostics().rawCallbacks;
        int quiet = 0;
        for (int i = 0; i < 20 && quiet < 3; ++i) {
            QThread::msleep(20);
            const auto d = m_laser->profileCallbackDiagnostics();
            if (d.rawCallbacks == lastRaw && d.rawRingDepth == 0) ++quiet;
            else { quiet = 0; lastRaw = d.rawCallbacks; }
        }
        primeDiag = m_laser->profileCallbackDiagnostics();
        qint64 primePulses = 0; QString primeCountErr;
        const bool primeCountOk = m_zmc->hwSensorColdPrimeCount(primePulses, &primeCountErr);
        LOG_INFO(LogCategory::Sync,
            tr("H1 cold-prime结束: converted=%1 raw=%2 convertFail=%3 controllerPrimePulses=%4%5")
            .arg(primeDiag.convertedProfiles).arg(primeDiag.rawCallbacks)
            .arg(primeDiag.conversionFailures)
            .arg(primeCountOk ? tr("%1").arg(primePulses) : QStringLiteral("读取失败"))
            .arg(primeCountOk ? QString() : QStringLiteral("(%1)").arg(primeCountErr)));
        if (!primeStopOk || !primed) {
            m_laser->setProfileEmissionSuppressed(false);
            m_laser->stopContinuous();
            if (errMsg) *errMsg = tr("H1传感器预热失败：converted=%1 raw=%2 pulses=%3；%4")
                .arg(primeDiag.convertedProfiles).arg(primeDiag.rawCallbacks)
                .arg(primeCountOk ? tr("%1").arg(primePulses) : QStringLiteral("未知"))
                .arg(!primeStopOk ? stopPrimeErr : QStringLiteral("Profile未连续达到8帧"));
            return false;
        }
        m_laser->resetProfileCallbackDiagnostics();
        m_laser->setProfileEmissionSuppressed(false);
        LOG_INFO(LogCategory::Sync,
            tr("H1 cold-prime通过；同一OP14预热帧已丢弃，新的正式session将清空并重新Arm五轴R0 REGIST FIFO。"));
    }
    else {
        QThread::msleep(50);
    }

    bool ok = (m_syncMode == SyncMode::Software)
        ? m_softwareStrategy->start(&err)
        : m_hardwareStrategy->start(&err);
    if (ok && h3) {
        // Crucial: five IN1/R1 latch FIFOs are armed before LLT enables CMM.
        ok = m_laser->startCmmGated(m_hwTriggerFrequencyHz, &err);
        m_hardwareStrategy->restartProbeTimeout();
    }
    if (!ok) {
        if (m_syncMode == SyncMode::Hardware) {
            if (h3) m_zmc->hwTableSampleStop(nullptr);
            m_hardwareStrategy->stop();
        }
        m_laser->setProfileEmissionSuppressed(false);
        m_laser->stopContinuous();
        if (errMsg) *errMsg = err;
        return false;
    }

    emit runningChanged(true);
    return true;
}

void SyncManager::stop()
{
    if (m_stopping || !isRunning()) return;
    m_stopping = true;
    struct StopGuard { bool& flag; ~StopGuard() { flag = false; } } guard{ m_stopping };
    if (m_syncMode == SyncMode::Hardware) {
        // H3 is sensor-master: stop CMM FIRST while ECI IN1 is still armed.
        // LLT's special stop waits for the first CMM-inactive frame and
        // suppresses its callback, leaving no unmatched new pose.
        if (m_lastCaptureTopology == mls::runtime::HardwareSyncTopology::SensorInternalCmm) {
            QString cmmStopError;
            if (!m_laser->stopCmmGated(&cmmStopError))
                m_hardwareStrategy->invalidateAlignment(tr("H3停止CMM输出/传输失败: %1").arg(cmmStopError));
        }
        QString stopError;
        const bool triggerStopAccepted = m_zmc->hwTableSampleStop(&stopError);
        if (!triggerStopAccepted) {
            m_hardwareStrategy->invalidateAlignment(tr("停止触发失败：%1").arg(stopError));
        }

        // STOP-DRAIN 1316: count convergence alone does not prove raw REGIST
        // FIFOs were pumped. BASIC must publish TailDrained and an equal five-axis
        // raw head/gr snapshot for the same session BEFORE the host declares success.
        const double triggerPeriodMs = 1000.0 / static_cast<double>(std::max(1, m_lastCaptureTriggerFrequencyHz));
        const int waitTimeMs = std::max(1500, static_cast<int>(std::ceil(triggerPeriodMs * 6.0 + 800.0)));
        QElapsedTimer timer;
        timer.start();
        qint64 lastTrigger = -1, lastPose = -1, lastProfiles = -1;
        int stableCompleteCount = 0, diagPoll = 0;
        bool tailSettled = false, tailDiagOk = false;
        QString counterError, tailDiagError;
        ZmcController::HwStopTailDiag tailDiag;
        auto describeTail = [&]() -> QString {
            if (!tailDiagOk) return tr("STOP诊断读取失败：%1").arg(tailDiagError);
            return tr("phase=%1 state=%2 enable=%3 BASICfault=%4 sessionExpected=%5 sessionACK=%6 "
                "rawHeads=[%7,%8,%9,%10,%11] pumpHead=%12 stopRawHead=%13 "
                "published=%14 ringRead=%15 ringWrite=%16 PWM0_Duty=%17 OP14_logic=%18")
                .arg(tailDiag.phase).arg(tailDiag.state).arg(tailDiag.enable).arg(tailDiag.basicFault)
                .arg(tailDiag.expectedSession).arg(tailDiag.sessionAck)
                .arg(tailDiag.rawHead[0]).arg(tailDiag.rawHead[1]).arg(tailDiag.rawHead[2])
                .arg(tailDiag.rawHead[3]).arg(tailDiag.rawHead[4])
                .arg(tailDiag.pumpedHead).arg(tailDiag.stopRawHead)
                .arg(tailDiag.published).arg(tailDiag.ringRead).arg(tailDiag.ringWrite)
                .arg(tailDiag.pwmDuty, 0, 'f', 6).arg(tailDiag.op14);
            };

        while (timer.elapsed() < waitTimeMs) {
            QThread::msleep(20);
            QCoreApplication::processEvents(QEventLoop::ExcludeUserInputEvents);
            QCoreApplication::sendPostedEvents(m_hardwareStrategy.get(), QEvent::MetaCall);
            m_hardwareStrategy->processProfiles();

            qint64 triggerCount = 0;
            const bool counterOk = m_zmc->hwGetTriggerCount(triggerCount, &counterError);
            const auto stats = m_hardwareStrategy->getAlignStats();
            const bool temporalLookAhead = m_temporalCorrection.enabled && m_temporalCorrection.delaySec > 0.0;
            const bool alignmentComplete = temporalLookAhead
                ? (stats.alignedProfiles + stats.pendingProfiles == stats.totalProfiles && stats.pendingProfiles <= 1)
                : (stats.alignedProfiles == stats.totalProfiles && stats.pendingProfiles == 0);
            const bool countsComplete = counterOk
                && stats.totalPoseSamples == triggerCount
                && stats.totalProfiles == triggerCount
                && alignmentComplete;
            const bool countsStable = counterOk
                && triggerCount == lastTrigger
                && stats.totalPoseSamples == lastPose
                && stats.totalProfiles == lastProfiles;

            // Query the richer STOP telemetry only every fifth cycle or when
            // counts converge, to avoid starving the high-priority TABLE reader.
            if (++diagPoll % 5 == 0 || (countsComplete && countsStable)) {
                tailDiagError.clear();
                tailDiagOk = m_zmc->hwReadStopTailDiagnostics(tailDiag, &tailDiagError);
            }
            if (tailDiagOk && (tailDiag.phase == mls::runtime::Table5D::TailFailure ||
                tailDiag.basicFault != 0 || tailDiag.state == mls::runtime::Table5D::Fault)) {
                LOG_WARNING(LogCategory::Sync, tr("STOP BASIC故障：%1").arg(describeTail()));
                break;
            }

            const bool basicDrained = tailDiagOk && tailDiag.rawFifoDrained()
                && tailDiag.published == triggerCount
                && tailDiag.ringWrite == tailDiag.ringRead;
            if (countsComplete && countsStable && basicDrained) {
                ++stableCompleteCount;
                if (stableCompleteCount >= 5) {
                    tailSettled = true;
                    break;
                }
            }
            else {
                stableCompleteCount = 0;
            }
            lastTrigger = counterOk ? triggerCount : -1;
            lastPose = stats.totalPoseSamples;
            lastProfiles = stats.totalProfiles;
        }

        // Capture one last raw-FIFO snapshot before ending the sensor stream.
        tailDiagError.clear();
        tailDiagOk = m_zmc->hwReadStopTailDiagnostics(tailDiag, &tailDiagError);
        LOG_INFO(LogCategory::Sync, tr("STOP原始FIFO/Buffer-B诊断：%1").arg(describeTail()));
        if (!tailSettled || !tailDiagOk || !tailDiag.rawFifoDrained() ||
            tailDiag.ringWrite != tailDiag.ringRead) {
            const auto stats = m_hardwareStrategy->getAlignStats();
            qint64 triggerCount = 0;
            QString finalCounterError;
            const bool counterOk = m_zmc->hwGetTriggerCount(triggerCount, &finalCounterError);
            m_hardwareStrategy->invalidateAlignment(
                tr("STOP闭环不完整：BASIC=%1；PublishedLatch=%2%3 Pose=%4 Profile=%5 "
                    "Aligned=%6 Pending=%7；本次数据禁止判为有效。")
                .arg(describeTail())
                .arg(counterOk ? tr("%1").arg(triggerCount) : tr("读取失败"))
                .arg(counterOk ? QString() : tr("(%1)").arg(finalCounterError))
                .arg(stats.totalPoseSamples).arg(stats.totalProfiles)
                .arg(stats.alignedProfiles).arg(stats.pendingProfiles));
        }

        QString actualIdleError;
        if (!m_zmc->hwTableWaitIdleReady(1500, &actualIdleError)) {
            m_hardwareStrategy->invalidateAlignment(
                tr("BASIC实际停止状态握手失败（不是IdleReady）：%1；本次同步数据无效。")
                .arg(actualIdleError));
        }
        m_laser->stopContinuous();
        // Profile ingress is direct from ProfileDecodeThread in V27.6.16M;
        // stopContinuous() has already drained the raw ring/decoder. Perform one
        // final alignment pass before closing the session.
        m_hardwareStrategy->processProfiles(true);
        const auto finalCounts = m_hardwareStrategy->getAlignStats();
        qint64 finalPublished = 0;
        QString finalReadError;
        const bool finalPublishedOk = m_zmc->hwGetTriggerCount(finalPublished, &finalReadError);
        ZmcController::HwStopTailDiag afterStop;
        QString afterStopError;
        const bool finalTailOk = m_zmc->hwReadStopTailDiagnostics(afterStop, &afterStopError);
        if (!finalTailOk || !afterStop.rawFifoDrained() ||
            afterStop.ringWrite != afterStop.ringRead || !finalPublishedOk ||
            finalPublished != afterStop.published ||
            finalCounts.totalPoseSamples != finalPublished ||
            finalCounts.totalProfiles != finalPublished) {
            m_hardwareStrategy->invalidateAlignment(
                tr("传感器解码队列停止后的最终复核失败：tail=%1 published=%2 pose=%3 "
                    "profile=%4 diag=%5；禁止将多余尾帧静默丢弃后判成功")
                .arg(finalTailOk ? afterStop.phase : -1)
                .arg(finalPublishedOk ? tr("%1").arg(finalPublished) : finalReadError)
                .arg(finalCounts.totalPoseSamples).arg(finalCounts.totalProfiles)
                .arg(finalTailOk ? QStringLiteral("读到BASIC诊断") : afterStopError));
        }
        m_hardwareStrategy->stop();
        m_hardwareStrategy->finalizeAlignment();
        const auto stats = m_hardwareStrategy->getAlignStats();
        LOG_INFO(LogCategory::Sync,
            tr("最终统计：Pose=%1 Profile=%2 Aligned=%3 Pending=%4 Dropped=%5 Fault=%6")
            .arg(stats.totalPoseSamples).arg(stats.totalProfiles).arg(stats.alignedProfiles)
            .arg(stats.pendingProfiles).arg(stats.droppedProfiles).arg(stats.alignmentFault));
    }
    else {
        m_laser->stopContinuous();
        QCoreApplication::sendPostedEvents(m_softwareStrategy.get(), QEvent::MetaCall);
        m_softwareStrategy->stop();
    }
    emit runningChanged(false);
}

bool SyncManager::isRunning() const
{
    return m_softwareStrategy->isRunning() || m_hardwareStrategy->isRunning();
}

void SyncManager::getStats(int& profileCount, int& poseCount) const
{
    if (m_syncMode == SyncMode::Software) {
        m_softwareStrategy->getStats(profileCount, poseCount);
    }
    else {
        m_hardwareStrategy->getStats(profileCount, poseCount);
    }
}

HardwareSyncStrategy::AlignStats SyncManager::getHardwareAlignStats() const
{
    return m_hardwareStrategy->getAlignStats();
}

QVector<AlignedPoint> SyncManager::getHardwareAlignedPoints() const
{
    return m_syncMode == SyncMode::Software ? m_softwareStrategy->getAllAlignedPoints() : m_hardwareStrategy->getAllAlignedPoints();
}

QVector<SamplePoint> SyncManager::getHardwarePoseSamples() const
{
    return m_syncMode == SyncMode::Software ? m_softwareStrategy->getAllPoseSamples() : m_hardwareStrategy->getAllPoseSamples();
}

bool SyncManager::takeHardwareCaptureData(QVector<AlignedPoint>& aligned,
    QVector<SamplePoint>& poses,
    QString* error)
{
    if (m_syncMode != SyncMode::Hardware) {
        if (error) *error = tr("只有Hardware模式支持完成采集数据转移");
        return false;
    }
    return m_hardwareStrategy->takeCompletedCapture(aligned, poses, error);
}

void SyncManager::setHardwareAlignedDataEmissionEnabled(bool enabled)
{
    m_hardwareStrategy->setAlignedDataEmissionEnabled(enabled);
}

bool SyncManager::hardwareAlignedDataEmissionEnabled() const
{
    return m_hardwareStrategy->alignedDataEmissionEnabled();
}

void SyncManager::setHardwareRawProfileCaptureEnabled(bool enabled)
{
    m_hardwareStrategy->setRawProfileCaptureEnabled(enabled);
}

bool SyncManager::hardwareRawProfileCaptureEnabled() const
{
    return m_hardwareStrategy->rawProfileCaptureEnabled();
}

bool SyncManager::takeHardwareRawCaptureData(QVector<LaserProfile>& profiles,
    QVector<SamplePoint>& poses,
    QString* error)
{
    if (m_syncMode != SyncMode::Hardware) {
        if (error) *error = tr("只有Hardware模式支持RAW Profile数据转移");
        return false;
    }
    return m_hardwareStrategy->takeCompletedRawCapture(profiles, poses, error);
}

bool SyncManager::latestHardwarePoseSample(SamplePoint& sample) const
{
    // UI/diagnostic accessor only. Never substitute this for the calibration-source
    // measured-pose contract (XYZ grating MPOS + C/B machine MPOS) or for ordinary control sampling.
    if (m_syncMode != SyncMode::Hardware || !m_hardwareStrategy->isRunning()) return false;
    return m_hardwareStrategy->latestPoseSample(sample);
}

QVector<quint32> SyncManager::getHardwareReceivedProfileIndices() const
{
    return m_hardwareStrategy->getAllReceivedProfileIndices();
}

void SyncManager::clearData()
{
    if (isRunning()) {
        LOG_WARNING(LogCategory::Sync, tr("无法在运行时清除数据"));
        return;
    }

    m_hardwareStrategy->clearData();
    m_softwareStrategy->clearData();

    LOG_DEBUG(LogCategory::Sync, tr("SyncManager: 数据已清除"));
}

std::vector<syncdiag::ProfileRecord> HardwareSyncStrategy::getProfileRecords() const
{
    QMutexLocker lock(&m_receivedIndicesMutex);
    return m_profileRecords;
}
std::vector<syncdiag::ProfileRecord> SyncManager::getHardwareProfileRecords() const
{
    return m_hardwareStrategy->getProfileRecords();
}
