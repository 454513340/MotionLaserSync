#include "core/LaserSensor.h"
#include "core/SensorTriggerReadbackLogic.h"
#include "calibration/SensorCalibrationTiming.h"
#include "utils/GlobalTimeSync.h"
#include "utils/LogManager.h"

#include <QDebug>
#include <QThread>
#include <vector>
#include <cstring>
#include <algorithm>
#include <limits>
#include <type_traits>
#include <utility>

static constexpr unsigned int MAX_INTERFACE_COUNT = 5;
static constexpr unsigned int MAX_RESOLUTIONS = 6;
// Compile-time check that our portable interpretation stays in sync with the
// bundled, actual vendor scanCONTROL headers.
static_assert(TRIG_EXT_ACTIVE == mls::sensor::trigger::kExternalActiveBit);
static_assert(MULTI_RS422_CMM == mls::sensor::trigger::kCmmRs422Port);


// The bundled LLT 4.0 C++ wrapper explicitly returns void here.
// Keep the real vendor API declaration checked in the GUI build (the portable
// core-mock target deliberately does not compile LaserSensor.cpp).
static_assert(std::is_same_v<
    decltype(std::declval<CInterfaceLLT&>().Timestamp2CmmTriggerAndInCounter(
        nullptr, nullptr, nullptr, nullptr, nullptr)), void>,
    "Bundled LLT Timestamp2CmmTriggerAndInCounter API must return void");

namespace {
    template <class T>
    void atomicMax(std::atomic<T>& target, T value)
    {
        T current = target.load(std::memory_order_relaxed);
        while (current < value &&
            !target.compare_exchange_weak(current, value,
                std::memory_order_relaxed,
                std::memory_order_relaxed)) {
        }
    }
}

// V27.6.16M: SDK callback only enqueues raw bytes. Conversion/timestamp parsing
// runs here at elevated (but not TimeCritical) priority so GUI stalls cannot
// directly block LLT callback service.
class ProfileDecodeThread : public QThread
{
public:
    explicit ProfileDecodeThread(LaserSensor* sensor) : m_sensor(sensor) {}
    void requestStop() { m_stop.store(true); }
protected:
    void run() override
    {
        while (m_sensor && (!m_stop.load(std::memory_order_acquire) ||
            m_sensor->m_profileIngressReadSeq.load(std::memory_order_acquire) <
            m_sensor->m_profileIngressWriteSeq.load(std::memory_order_acquire))) {
            if (!m_sensor->decodeOneQueuedProfile()) QThread::usleep(200);
        }
    }
private:
    LaserSensor* m_sensor = nullptr;
    std::atomic_bool m_stop{ false };
};

// 前向声明
static int getMaxFrequencyForROIIndex_Internal(int index);

// ============================================================================
// VideoStreamThread 实现
// ============================================================================

VideoStreamThread::VideoStreamThread(LaserSensor* sensor, CInterfaceLLT* llt,
    int width, int height, QObject* parent)
    : QThread(parent)
    , m_sensor(sensor)
    , m_llt(llt)
    , m_width(width)
    , m_height(height)
{
}

VideoStreamThread::~VideoStreamThread()
{
    requestStop();
    wait(3000);
}

void VideoStreamThread::run()
{
    if (!m_llt || !m_sensor || m_width <= 0 || m_height <= 0) return;

    const int bufferSize = m_width * m_height;
    QVector<unsigned char> buffer(bufferSize);

    // 获取传感器参数计算帧间隔
    const auto& params = m_sensor->params();
    int frameIntervalMs = static_cast<int>((params.exposureTime + params.idleTime) * 0.01 + 0.5);
    if (frameIntervalMs < 10) frameIntervalMs = 10;

    LOG_DEBUG(LogCategory::Sensor, QString("VideoStreamThread 启动, 帧间隔: %1ms").arg(frameIntervalMs));

    while (!m_stop.load()) {
        // 轮询获取 Video 帧
        int ret = m_llt->GetActualProfile(buffer.data(), bufferSize, VIDEO_IMAGE, nullptr);

        if (ret == ERROR_PROFTRANS_NO_NEW_PROFILE) {
            // 没有新帧，等待一会再试
            QThread::msleep(frameIntervalMs / 2);
            continue;
        }
        else if (ret < 0) {
            // 发生错误
            LOG_WARNING(LogCategory::Sensor, QString("VideoStreamThread: GetActualProfile 错误: %1").arg(ret));
            QThread::msleep(100);
            continue;
        }
        else if (ret == bufferSize) {
            // 成功获取一帧
            m_sensor->processVideoFrame(buffer);
        }

        // 稍微延时避免 CPU 占用过高
        QThread::msleep(1);
    }

    LOG_DEBUG(LogCategory::Sensor, "VideoStreamThread 停止");
}

// ============================================================================
// LaserSensor 实现
// ============================================================================

LaserSensor::LaserSensor(QObject* parent)
    : QObject(parent)
{
    qRegisterMetaType<LaserProfile>("LaserProfile");
    qRegisterMetaType<QImage>("QImage");
}

LaserSensor::~LaserSensor()
{
    stopAll();
    disconnectSensor();
}

void LaserSensor::clearInternal()
{
    m_resolutions.clear();
    m_resolution = 0;
    m_connected = false;
    m_videoWidth = 0;
    m_videoHeight = 0;
    m_cmmConfigured.store(false);
    m_cmmGated.store(false);
    m_hwTriggerReadbackValid.store(false);
    m_cmmPortReadbackValid.store(false);
    m_hwTriggerRequested.store(0);
    m_hwTriggerReadback.store(0);
    m_hwDigitalIoRequested.store(0);
    m_hwDigitalIoReadback.store(0);
    resetProfileCallbackDiagnostics();
    m_suppressProfileEmission.store(false);
    m_sensorCalibrationRichProfileEnabled.store(false);
    {
        QMutexLocker lock(&m_sensorCalibrationProfileMutex);
        m_hasLatestSensorCalibrationProfile = false;
        m_latestSensorCalibrationProfile = SensorCalibrationProfileFrame{};
    }
}

LaserSensor::ProfileCallbackDiagnostics LaserSensor::profileCallbackDiagnostics() const
{
    ProfileCallbackDiagnostics d;
    d.rawCallbacks = m_rawProfileCallbackCount.load();
    d.convertedProfiles = m_convertedProfileCount.load();
    d.conversionFailures = m_profileConvertFailureCount.load();
    d.ingressDrops = m_profileIngressDropCount.load();
    const quint64 writeSeq = m_profileIngressWriteSeq.load(std::memory_order_acquire);
    const quint64 readSeq = m_profileIngressReadSeq.load(std::memory_order_acquire);
    d.rawRingDepth = writeSeq >= readSeq ? writeSeq - readSeq : 0;
    d.rawRingHighWater = m_profileIngressHighWater.load();
    d.rawRingCapacity = kProfileIngressSlotCount;
    d.lastCallbackOrdinal = m_lastProfileCallbackOrdinal.load();
    d.maxCallbackServiceUs = m_maxProfileCallbackServiceUs.load();
    d.maxDecoderQueueUs = m_maxProfileDecoderQueueUs.load();
    d.maxDecodeServiceUs = m_maxProfileDecodeServiceUs.load();
    d.lastConvertResult = m_lastProfileConvertResult.load();
    d.lastCallbackBytes = m_lastProfileCallbackBytes.load();
    d.lastResolution = m_lastProfileResolution.load();
    return d;
}

LaserSensor::HardwareTriggerDiagnostics LaserSensor::hardwareTriggerDiagnostics() const
{
    HardwareTriggerDiagnostics d;
    d.readbackValid = m_hwTriggerReadbackValid.load();
    d.cmmPortReadbackValid = m_cmmPortReadbackValid.load();
    d.triggerRequested = m_hwTriggerRequested.load();
    d.triggerReadback = m_hwTriggerReadback.load();
    d.digitalIoRequested = m_hwDigitalIoRequested.load();
    d.digitalIoReadback = m_hwDigitalIoReadback.load();
    return d;
}

void LaserSensor::resetProfileCallbackDiagnostics()
{
    m_rawProfileCallbackCount.store(0);
    m_convertedProfileCount.store(0);
    m_profileConvertFailureCount.store(0);
    m_profileIngressDropCount.store(0);
    m_profileIngressHighWater.store(0);
    m_maxProfileCallbackServiceUs.store(0);
    m_maxProfileDecoderQueueUs.store(0);
    m_maxProfileDecodeServiceUs.store(0);
    m_lastProfileCallbackOrdinal.store(0);
    m_lastProfileConvertResult.store(0);
    m_lastProfileCallbackBytes.store(0);
    m_lastProfileResolution.store(0);
    m_profileIngressFaultEmitted.store(false);
}

bool LaserSensor::latestProfile(LaserProfile& profile) const
{
    QMutexLocker lock(&m_latestProfileMutex);
    if (!m_hasLatestDecodedProfile) return false;
    profile = m_latestDecodedProfile;
    return true;
}

void LaserSensor::setSensorCalibrationRichProfileEnabled(bool enabled)
{
    m_sensorCalibrationRichProfileEnabled.store(enabled, std::memory_order_release);
    if (!enabled) return;

    QMutexLocker lock(&m_sensorCalibrationProfileMutex);
    m_hasLatestSensorCalibrationProfile = false;
    m_latestSensorCalibrationProfile = SensorCalibrationProfileFrame{};
}

bool LaserSensor::latestSensorCalibrationProfile(SensorCalibrationProfileFrame& frame) const
{
    QMutexLocker lock(&m_sensorCalibrationProfileMutex);
    if (!m_hasLatestSensorCalibrationProfile) return false;
    frame = m_latestSensorCalibrationProfile;
    return true;
}

bool LaserSensor::readDeviceIdentity(QString& deviceName, QString& serialNumber, QString* errMsg)
{
    deviceName.clear();
    serialNumber.clear();

    QMutexLocker locker(&m_mutex);
    if (!m_llt || !m_connected) {
        if (errMsg) *errMsg = tr("传感器未连接");
        return false;
    }

    char devName[256] = {};
    char vendorName[256] = {};
    char serial[256] = {};

    const int nameRet = m_llt->GetDeviceName(devName, sizeof(devName), vendorName, sizeof(vendorName));
    const int serialRet = m_llt->GetSerialNumber(serial, sizeof(serial));

    if (nameRet >= GENERAL_FUNCTION_OK) {
        const QString vendor = QString::fromLatin1(vendorName).trimmed();
        const QString model = QString::fromLatin1(devName).trimmed();
        deviceName = vendor.isEmpty() ? model : (vendor + QStringLiteral(" ") + model).trimmed();
    }
    if (serialRet >= GENERAL_FUNCTION_OK) {
        serialNumber = QString::fromLatin1(serial).trimmed();
    }

    // Some LLT/firmware combinations do not expose both strings. The experiment
    // may continue: sensorTypeName() and the connected IP are still recorded.
    if (nameRet < GENERAL_FUNCTION_OK && serialRet < GENERAL_FUNCTION_OK && errMsg) {
        *errMsg = tr("设备名称/序列号均无法读取（name=%1, serial=%2）；将保存类型名和IP")
            .arg(nameRet).arg(serialRet);
    }
    return true;
}

QString LaserSensor::sensorTypeName() const
{
    if (m_scannerType >= scanCONTROL30xx_25 && m_scannerType <= scanCONTROL30xx_xxx)
        return "scanCONTROL 30xx";
    else if (m_scannerType >= scanCONTROL29xx_25 && m_scannerType <= scanCONTROL29xx_xxx)
        return "scanCONTROL 29xx";
    else if (m_scannerType >= scanCONTROL27xx_25 && m_scannerType <= scanCONTROL27xx_xxx)
        return "scanCONTROL 27xx";
    else if (m_scannerType >= scanCONTROL26xx_25 && m_scannerType <= scanCONTROL26xx_xxx)
        return "scanCONTROL 26xx";
    else if (m_scannerType >= scanCONTROL25xx_25 && m_scannerType <= scanCONTROL25xx_xxx)
        return "scanCONTROL 25xx";
    else
        return "Unknown";
}

bool LaserSensor::connectSensor(const QString& dllPath, QString* errMsg)
{
    disconnectSensor();

    bool loadError = false;
    const std::string dllName = dllPath.isEmpty()
        ? std::string("LLT.dll")
        : dllPath.toStdString();

    CInterfaceLLT* llt = new (std::nothrow) CInterfaceLLT(dllName.c_str(), &loadError);
    if (!llt || loadError) {
        if (errMsg) {
            *errMsg = tr("加载 LLT.dll 失败（路径：%1）").arg(QString::fromStdString(dllName));
        }
        delete llt;
        return false;
    }

    bool ok = true;
    int ret = 0;

    // 创建以太网设备句柄
    ret = llt->CreateLLTDevice(INTF_TYPE_ETHERNET);
    if (ret < GENERAL_FUNCTION_OK) {
        ok = false;
        if (errMsg) *errMsg = tr("CreateLLTDevice 失败，错误码 %1").arg(ret);
    }

    // 搜索可用接口
    std::vector<unsigned int> interfaces(MAX_INTERFACE_COUNT);
    unsigned int interfaceCount = 0;

    if (ok) {
        ret = llt->GetDeviceInterfacesFast(interfaces.data(),
            static_cast<unsigned int>(interfaces.size()));

        if (ret == ERROR_GETDEVINTERFACES_REQUEST_COUNT) {
            interfaceCount = static_cast<unsigned int>(interfaces.size());
        }
        else if (ret < 0) {
            ok = false;
            if (errMsg) *errMsg = tr("GetDeviceInterfacesFast 失败，错误码 %1").arg(ret);
        }
        else {
            interfaceCount = static_cast<unsigned int>(ret);
        }

        if (ok && interfaceCount == 0) {
            ok = false;
            if (errMsg) *errMsg = tr("未找到任何 scanCONTROL 设备");
        }
    }

    // 选择第一个接口
    if (ok) {
        ret = llt->SetDeviceInterface(interfaces[0], 0);
        if (ret < GENERAL_FUNCTION_OK) {
            ok = false;
            if (errMsg) *errMsg = tr("SetDeviceInterface 失败，错误码 %1").arg(ret);
        }
    }

    // 连接传感器
    if (ok) {
        ret = llt->Connect();
        if (ret < GENERAL_FUNCTION_OK) {
            ok = false;
            if (errMsg) *errMsg = tr("Connect 失败，错误码 %1").arg(ret);
        }
    }

    // 获取扫描器类型
    TScannerType type = scanCONTROL2xxx;
    if (ok) {
        ret = llt->GetLLTType(&type);
        if (ret < GENERAL_FUNCTION_OK) {
            ok = false;
            if (errMsg) *errMsg = tr("GetLLTType 失败，错误码 %1").arg(ret);
        }
    }

    // 获取可用分辨率
    QVector<unsigned int> resolutions;
    unsigned int activeRes = 0;

    if (ok) {
        std::vector<DWORD> resBuf(MAX_RESOLUTIONS);
        ret = llt->GetResolutions(resBuf.data(),
            static_cast<unsigned int>(resBuf.size()));

        if (ret < GENERAL_FUNCTION_OK) {
            ok = false;
            if (errMsg) *errMsg = tr("GetResolutions 失败，错误码 %1").arg(ret);
        }
        else {
            for (DWORD v : resBuf) {
                if (v != 0) {
                    resolutions.append(static_cast<unsigned int>(v));
                }
            }

            if (resolutions.isEmpty()) {
                ok = false;
                if (errMsg) *errMsg = tr("没有有效分辨率");
            }
            else {
                activeRes = resolutions.first();
            }
        }
    }

    if (!ok) {
        llt->Disconnect();
        delete llt;
        return false;
    }

    // 成功，更新成员变量
    {
        QMutexLocker locker(&m_mutex);
        m_llt = llt;
        m_connected = true;
        m_scannerType = type;
        m_resolutions = resolutions;
        m_resolution = activeRes;
        m_workMode = SensorWorkMode::Profile;  // 默认 Profile 模式
    }

    LOG_INFO(LogCategory::Sensor, QString("传感器已连接: %1, 可用分辨率: %2")
        .arg(sensorTypeName())
        .arg(resolutions.size()));

    // 注意：不在连接时自动修改传感器参数
    // Peak Filter等参数只在用户明确点击"写入传感器"时才设置
    // 这样可以保持传感器的原有状态，不影响硬件触发等功能

    return true;
}

void LaserSensor::disconnectSensor()
{
    stopAll();

    CInterfaceLLT* llt = nullptr;

    {
        QMutexLocker locker(&m_mutex);

        if (!m_llt) {
            clearInternal();
            return;
        }

        llt = m_llt;
        m_llt = nullptr;
        m_connected = false;
        clearInternal();
    }

    if (!llt) return;

    // DLL 调用在锁外执行，避免死锁
    llt->TransferProfiles(NORMAL_TRANSFER, false);
    llt->TransferVideoStream(VIDEO_MODE_1, false, nullptr, nullptr);
    llt->RegisterCallback(STD_CALL, nullptr, nullptr);
    llt->Disconnect();
    delete llt;
}

// ============================================================================
// Profile 模式配置
// ============================================================================

bool LaserSensor::configure(SensorTriggerMode mode,
    const SensorParams& params,
    bool highLevel24V,
    HardwareTriggerType triggerType,
    QString* errMsg)
{
    if (!m_llt || !m_connected) {
        if (errMsg) *errMsg = tr("传感器未连接");
        return false;
    }

    // 如果正在运行，先停止
    if (isRunning()) {
        stopAll();
    }
    // CMM metadata decoding is only valid for the current CMM session.
    // H3 switching back to H1 or software must not retain old CMM flags.
    m_cmmConfigured.store(false);

    bool ok = false;
    if (mode == SensorTriggerMode::SoftwareInternal) {
        ok = configureSoftwareInternal(params, errMsg);
    }
    else {
        ok = configureHardwareExternal(params, highLevel24V, triggerType, errMsg);
    }

    if (ok) {
        m_triggerMode = mode;
        m_params = params;
        m_workMode = SensorWorkMode::Profile;
    }
    return ok;
}

bool LaserSensor::configureSoftwareInternal(const SensorParams& params, QString* errMsg)
{
    QMutexLocker locker(&m_mutex);
    m_hwTriggerReadbackValid.store(false);
    m_cmmPortReadbackValid.store(false);
    m_hwTriggerRequested.store(0);
    m_hwTriggerReadback.store(0);
    m_hwDigitalIoRequested.store(0);
    m_hwDigitalIoReadback.store(0);

    if (m_resolutions.isEmpty()) {
        if (errMsg) *errMsg = tr("还没有读取到可用分辨率");
        return false;
    }

    unsigned int idx = qMin(params.resolutionIndex,
        static_cast<unsigned int>(m_resolutions.size() - 1));
    unsigned int res = m_resolutions[idx];

    int ret = m_llt->SetResolution(res);
    if (ret < GENERAL_FUNCTION_OK) {
        if (errMsg) *errMsg = tr("SetResolution(%1) 失败，错误码 %2").arg(res).arg(ret);
        return false;
    }
    m_resolution = res;

    // ========== 内部触发 ==========
    const DWORD trig = TRIG_INTERNAL;
    m_hwTriggerRequested.store(static_cast<unsigned int>(trig));
    ret = m_llt->SetFeature(FEATURE_FUNCTION_TRIGGER, trig);
    if (ret < GENERAL_FUNCTION_OK) {
        if (errMsg) *errMsg = tr("设置内部触发失败，错误码 %1").arg(ret);
        return false;
    }
    // H3: TRIG_INTERNAL==0 clears the external-enable bit. The SDK readback
    // can retain unrelated feature bits; never require all bits to equal zero.
    DWORD internalBack = 0;
    ret = m_llt->GetFeature(FEATURE_FUNCTION_TRIGGER, &internalBack);
    if (ret < GENERAL_FUNCTION_OK) {
        if (errMsg) *errMsg = tr("H3内部触发FEATURE_FUNCTION_TRIGGER读取失败，错误码 %1").arg(ret);
        return false;
    }
    m_hwTriggerReadback.store(static_cast<unsigned int>(internalBack));
    if (!mls::sensor::trigger::sensorTriggerMatches(3, trig, internalBack)) {
        if (errMsg) *errMsg = tr("内部触发配置回读错误：仍启用外部触发，寄存器=0x%1")
            .arg(static_cast<qulonglong>(internalBack), 8, 16, QChar('0'));
        return false;
    }
    // Retain the actual readback, but publish a valid configuration only after
    // all relevant sensor feature writes have succeeded.

    // ========== Profile 类型 ==========
    ret = m_llt->SetProfileConfig(PROFILE);
    if (ret < GENERAL_FUNCTION_OK) {
        if (errMsg) *errMsg = tr("SetProfileConfig(PROFILE) 失败，错误码 %1").arg(ret);
        return false;
    }

    // ========== 曝光和 idle 时间 ==========
    ret = m_llt->SetFeature(FEATURE_FUNCTION_EXPOSURE_TIME, params.exposureTime);
    if (ret < GENERAL_FUNCTION_OK) {
        if (errMsg) *errMsg = tr("设置曝光时间失败，错误码 %1").arg(ret);
        return false;
    }
    LOG_DEBUG(LogCategory::Sensor, QString("曝光时间: %1 (10µs) = %2 ms")
        .arg(params.exposureTime).arg(params.exposureTime / 100.0));

    ret = m_llt->SetFeature(FEATURE_FUNCTION_IDLE_TIME, params.idleTime);
    if (ret < GENERAL_FUNCTION_OK) {
        if (errMsg) *errMsg = tr("设置 idle 时间失败，错误码 %1").arg(ret);
        return false;
    }
    LOG_DEBUG(LogCategory::Sensor, QString("空闲时间: %1 (10µs) = %2 ms")
        .arg(params.idleTime).arg(params.idleTime / 100.0));

    m_hwTriggerReadbackValid.store(true);
    LOG_INFO(LogCategory::Sensor, tr("软件内部触发模式读回通过并完成Profile时序配置"));
    return true;
}

bool LaserSensor::configureHardwareExternal(const SensorParams& params,
    bool highLevel24V,
    HardwareTriggerType triggerType,
    QString* errMsg)
{
    QMutexLocker locker(&m_mutex);

    if (m_resolutions.isEmpty()) {
        if (errMsg) *errMsg = tr("还没有读取到可用分辨率");
        return false;
    }

    if (params.measuringFieldIndex < 0 || params.measuringFieldIndex > 127) {
        if (errMsg) *errMsg = tr("Measuring Field索引非法: %1").arg(params.measuringFieldIndex);
        return false;
    }
    int ret = m_llt->SetFeature(FEATURE_FUNCTION_MEASURINGFIELD, static_cast<DWORD>(params.measuringFieldIndex));
    if (ret < GENERAL_FUNCTION_OK) {
        if (errMsg) *errMsg = tr("设置Measuring Field(%1)失败，错误码 %2").arg(params.measuringFieldIndex).arg(ret);
        return false;
    }
    DWORD fieldReadback = 0;
    ret = m_llt->GetFeature(FEATURE_FUNCTION_MEASURINGFIELD, &fieldReadback);
    if (ret < GENERAL_FUNCTION_OK || static_cast<int>(fieldReadback) != params.measuringFieldIndex) {
        if (errMsg) *errMsg = tr("Measuring Field回读不一致：请求%1，回读%2，错误码%3")
            .arg(params.measuringFieldIndex).arg(static_cast<int>(fieldReadback)).arg(ret);
        return false;
    }

    unsigned int idx = qMin(params.resolutionIndex,
        static_cast<unsigned int>(m_resolutions.size() - 1));
    unsigned int res = m_resolutions[idx];

    ret = m_llt->SetResolution(res);
    if (ret < GENERAL_FUNCTION_OK) {
        if (errMsg) *errMsg = tr("SetResolution(%1) 失败，错误码 %2").arg(res).arg(ret);
        return false;
    }
    m_resolution = res;

    // ========== Profile 类型 ==========
    ret = m_llt->SetProfileConfig(PROFILE);
    if (ret < GENERAL_FUNCTION_OK) {
        if (errMsg) *errMsg = tr("SetProfileConfig(PROFILE) 失败，错误码 %1").arg(ret);
        return false;
    }

    // ========== 曝光和 idle 时间 ==========
    ret = m_llt->SetFeature(FEATURE_FUNCTION_EXPOSURE_TIME, params.exposureTime);
    if (ret < GENERAL_FUNCTION_OK) {
        if (errMsg) *errMsg = tr("设置曝光时间失败，错误码 %1").arg(ret);
        return false;
    }
    LOG_DEBUG(LogCategory::Sensor, QString("曝光时间: %1 (10µs) = %2 ms")
        .arg(params.exposureTime).arg(params.exposureTime / 100.0));

    ret = m_llt->SetFeature(FEATURE_FUNCTION_IDLE_TIME, params.idleTime);
    if (ret < GENERAL_FUNCTION_OK) {
        if (errMsg) *errMsg = tr("设置 idle 时间失败，错误码 %1").arg(ret);
        return false;
    }
    LOG_DEBUG(LogCategory::Sensor, QString("空闲时间: %1 (10µs) = %2 ms")
        .arg(params.idleTime).arg(params.idleTime / 100.0));

    // ========== V32.11 外部触发配置 ==========
    // 现场只接OP14：OP14同时驱动scanCONTROL触发输入与ECI IN0。
    // 已实测OP14 OFF时ECI IN0从1->0；因此ECI使用REGIST(104)，
    // scanCONTROL使用线路回高对应的EdgeRising，使Profile与C/B锁存对齐在同一关闭边沿。
    DWORD Trigger = 0;
    QString triggerModeName;

    switch (triggerType) {
    case HardwareTriggerType::EdgeRising:
        Trigger |= TRIG_MODE_EDGE;
        Trigger |= TRIG_POLARITY_HIGH;
        triggerModeName = "Edge Rising";
        break;

    case HardwareTriggerType::EdgeFalling:
        Trigger |= TRIG_MODE_EDGE;
        Trigger |= TRIG_POLARITY_LOW;
        triggerModeName = "Edge Falling";
        break;

    case HardwareTriggerType::PulseHigh:
        Trigger |= TRIG_MODE_PULSE;
        Trigger |= TRIG_POLARITY_HIGH;
        triggerModeName = "Pulse High";
        break;

    case HardwareTriggerType::PulseLow:
        Trigger |= TRIG_MODE_PULSE;
        Trigger |= TRIG_POLARITY_LOW;
        triggerModeName = "Pulse Low";
        break;
    }

    Trigger |= TRIG_INPUT_DIGIN | TRIG_EXT_ACTIVE;
    const DWORD MultiPort = MULTI_DIGIN_TRIG_ONLY
        | MULTI_INPUT_PULLUP
        | (highLevel24V ? MULTI_LEVEL_24V : MULTI_LEVEL_5V);

    m_hwTriggerReadbackValid.store(false);
    m_cmmPortReadbackValid.store(false);
    m_hwTriggerRequested.store(static_cast<unsigned int>(Trigger));
    m_hwDigitalIoRequested.store(static_cast<unsigned int>(MultiPort));
    m_hwTriggerReadback.store(0);
    m_hwDigitalIoReadback.store(0);

    // IMPORTANT: configure the digital input first. Some devices may re-apply
    // input-port state when DIGITAL_IO changes, so TRIGGER is written last.
    ret = m_llt->SetFeature(FEATURE_FUNCTION_DIGITAL_IO, MultiPort);
    if (ret < GENERAL_FUNCTION_OK) {
        if (errMsg) *errMsg = tr("配置数字 IO 失败，错误码 %1").arg(ret);
        return false;
    }

    DWORD ioReadback = 0;
    ret = m_llt->GetFeature(FEATURE_FUNCTION_DIGITAL_IO, &ioReadback);
    if (ret < GENERAL_FUNCTION_OK) {
        if (errMsg) *errMsg = tr("读取数字 IO 配置失败，错误码 %1").arg(ret);
        return false;
    }
    m_hwDigitalIoReadback.store(static_cast<unsigned int>(ioReadback));
    if (!mls::sensor::trigger::digitalIoMatches(1, MultiPort, ioReadback)) {
        if (errMsg) {
            *errMsg = tr("数字 IO 配置回读不一致：requested=0x%1 readback=0x%2")
                .arg(static_cast<qulonglong>(MultiPort), 8, 16, QChar('0'))
                .arg(static_cast<qulonglong>(ioReadback), 8, 16, QChar('0'));
        }
        return false;
    }

    ret = m_llt->SetFeature(FEATURE_FUNCTION_TRIGGER, Trigger);
    if (ret < GENERAL_FUNCTION_OK) {
        if (errMsg) *errMsg = tr("配置外部触发失败，错误码 %1").arg(ret);
        return false;
    }

    DWORD triggerReadback = 0;
    ret = m_llt->GetFeature(FEATURE_FUNCTION_TRIGGER, &triggerReadback);
    if (ret < GENERAL_FUNCTION_OK) {
        if (errMsg) *errMsg = tr("读取外部触发配置失败，错误码 %1").arg(ret);
        return false;
    }
    m_hwTriggerReadback.store(static_cast<unsigned int>(triggerReadback));
    if (!mls::sensor::trigger::sensorTriggerMatches(1, Trigger, triggerReadback)) {
        if (errMsg) {
            *errMsg = tr("外部触发配置回读不一致：requested=0x%1 readback=0x%2")
                .arg(static_cast<qulonglong>(Trigger), 8, 16, QChar('0'))
                .arg(static_cast<qulonglong>(triggerReadback), 8, 16, QChar('0'));
        }
        return false;
    }

    m_hwTriggerReadbackValid.store(true);
    LOG_DEBUG(LogCategory::Sensor, QString("触发模式: %1").arg(triggerModeName));
    LOG_INFO(LogCategory::Sensor,
        tr("V32.11外部触发配置回读通过: DIGITAL_IO req=0x%1 read=0x%2; TRIGGER req=0x%3 read=0x%4")
        .arg(MultiPort, 8, 16, QChar('0'))
        .arg(ioReadback, 8, 16, QChar('0'))
        .arg(Trigger, 8, 16, QChar('0'))
        .arg(triggerReadback, 8, 16, QChar('0')));
    return true;
}

// ============================================================================
// 高级参数应用
// ============================================================================

bool LaserSensor::applyAdvancedParams(const SensorAdvancedParams& advParams, QString* errMsg)
{
    if (!m_llt || !m_connected) {
        if (errMsg) *errMsg = tr("传感器未连接");
        return false;
    }

    QMutexLocker locker(&m_mutex);
    int ret = 0;

    LOG_INFO(LogCategory::Sensor, tr("开始应用参数设置..."));

    // ========== 0. 确保Profile配置和分辨率已设置 ==========
    ret = m_llt->SetProfileConfig(PROFILE);
    if (ret < GENERAL_FUNCTION_OK) {
        LOG_DEBUG(LogCategory::Sensor, QString("设置Profile配置失败，错误码 %1").arg(ret));
    }

    if (advParams.resolution > 0) {
        ret = m_llt->SetResolution(advParams.resolution);
        if (ret < GENERAL_FUNCTION_OK) {
            LOG_DEBUG(LogCategory::Sensor, QString("设置分辨率失败，错误码 %1").arg(ret));
        }
    }
    else if (m_resolution > 0) {
        ret = m_llt->SetResolution(m_resolution);
        if (ret < GENERAL_FUNCTION_OK) {
            LOG_DEBUG(LogCategory::Sensor, QString("设置分辨率失败，错误码 %1").arg(ret));
        }
    }

    // ========== 1. 设置测量区域索引 ==========
    int maxFreq = getMaxFrequencyForROIIndex_Internal(advParams.measuringFieldIndex);
    LOG_DEBUG(LogCategory::Sensor, QString("设置测量区域索引: %1 (最大频率: %2 Hz)")
        .arg(advParams.measuringFieldIndex).arg(maxFreq > 0 ? QString::number(maxFreq) : "未知"));

    ret = m_llt->SetFeature(FEATURE_FUNCTION_MEASURINGFIELD, static_cast<DWORD>(advParams.measuringFieldIndex));
    if (ret < GENERAL_FUNCTION_OK) {
        LOG_WARNING(LogCategory::Sensor, QString("设置测量区域索引失败，错误码 %1").arg(ret));
    }
    else {
        // 等待传感器处理
        QThread::msleep(100);
    }

    // ========== 2. Profile Processing ==========
    DWORD processing = 0;

    // 高分辨率
    if (advParams.highResolution) {
        processing |= PROC_HIGH_RESOLUTION;
    }

    // 校准
    if (advParams.calibration) {
        processing |= PROC_CALIBRATION;
    }

    // 反射选择
    switch (advParams.reflectionSelect) {
    case 0: processing |= PROC_MULTIREFL_ALL; break;
    case 1: processing |= PROC_MULITREFL_FIRST; break;
    case 2: processing |= PROC_MULITREFL_LAST; break;
    case 3: processing |= PROC_MULITREFL_LARGESTAREA; break;
    case 4: processing |= PROC_MULITREFL_MAXINTENS; break;
    default: processing |= PROC_MULITREFL_LARGESTAREA; break;
    }

    // X轴翻转
    if (advParams.flipX) {
        processing |= PROC_FLIP_POSITION;
    }

    // Z轴翻转
    if (advParams.flipZ) {
        processing |= PROC_FLIP_DISTANCE;
    }

    // 快门对齐
    switch (advParams.shutterAlign) {
    case 0: processing |= PROC_SHUTTERALIGN_CENTER; break;
    case 1: processing |= PROC_SHUTTERALIGN_RIGHT; break;
    case 2: processing |= PROC_SHUTTERALIGN_LEFT; break;
    case 3: processing |= PROC_SHUTTERALIGN_OFF; break;
    default: processing |= PROC_SHUTTERALIGN_RIGHT; break;
    }

    ret = m_llt->SetFeature(FEATURE_FUNCTION_PROFILE_PROCESSING, processing);
    if (ret < GENERAL_FUNCTION_OK) {
        if (errMsg) *errMsg = tr("设置 Profile Processing 失败，错误码 %1").arg(ret);
        return false;
    }
    LOG_DEBUG(LogCategory::Sensor, QString("Profile Processing: 0x%1").arg(processing, 8, 16, QChar('0')));

    // ========== 3. 阈值与滤波 ==========
    DWORD threshold = static_cast<DWORD>(advParams.thresholdPercent * 10.0);  // 转换为 0.1% 单位

    if (advParams.ambientLightSuppression) {
        threshold |= THRESHOLD_AMBIENT_LIGHT_SUPPRESSION;
    }
    if (advParams.videoFilter) {
        threshold |= THRESHOLD_VIDEO_FILTER;
    }

    ret = m_llt->SetFeature(FEATURE_FUNCTION_THRESHOLD, threshold);
    if (ret < GENERAL_FUNCTION_OK) {
        if (errMsg) *errMsg = tr("设置阈值失败，错误码 %1").arg(ret);
        return false;
    }
    LOG_DEBUG(LogCategory::Sensor, QString("阈值: 0x%1").arg(threshold, 8, 16, QChar('0')));

    // ========== 4. 激光功率 ==========
    ret = m_llt->SetFeature(FEATURE_FUNCTION_LASER, advParams.laserPower);
    if (ret < GENERAL_FUNCTION_OK) {
        if (errMsg) *errMsg = tr("设置激光功率失败，错误码 %1").arg(ret);
        return false;
    }

    // ========== 5. 有效反射参数 (Peak Filter) - 仅写入 ==========
    // 首先尝试直接寄存器方式（固件版本 >= v43）
    bool peakFilterSuccess = false;

    // WIDTH 寄存器: 高16位=最小宽度, 低16位=最大宽度
    DWORD peakWidth = ((advParams.peakWidthMin & 0xFFFF) << 16) | (advParams.peakWidthMax & 0xFFFF);
    ret = m_llt->SetFeature(FEATURE_FUNCTION_PEAKFILTER_WIDTH, peakWidth);
    if (ret >= GENERAL_FUNCTION_OK) {
        LOG_DEBUG(LogCategory::Sensor, QString("反射宽度: %1 ~ %2")
            .arg(advParams.peakWidthMin).arg(advParams.peakWidthMax));

        // HEIGHT 寄存器: 高16位=最小强度, 低16位=最大强度
        DWORD peakHeight = ((advParams.peakIntensityMin & 0xFFFF) << 16) | (advParams.peakIntensityMax & 0xFFFF);
        ret = m_llt->SetFeature(FEATURE_FUNCTION_PEAKFILTER_HEIGHT, peakHeight);
        if (ret >= GENERAL_FUNCTION_OK) {
            LOG_DEBUG(LogCategory::Sensor, QString("反射强度: %1 ~ %2")
                .arg(advParams.peakIntensityMin).arg(advParams.peakIntensityMax));

            // 激活 Peak Filter
            ret = m_llt->SetFeature(FEATURE_FUNCTION_EXTRA_PARAMETER, 0);
            if (ret >= GENERAL_FUNCTION_OK) {
                LOG_DEBUG(LogCategory::Sensor, QString("Peak Filter已激活（直接寄存器方式）"));
                peakFilterSuccess = true;
            }
        }
    }

    // 如果直接寄存器方式失败，使用命令序列方式（固件版本 < v43）
    if (!peakFilterSuccess) {
        LOG_DEBUG(LogCategory::Sensor, QString("尝试命令序列方式设置Peak Filter"));
        peakFilterSuccess = setPeakFilterViaSharpness(
            static_cast<unsigned short>(advParams.peakWidthMin),
            static_cast<unsigned short>(advParams.peakWidthMax),
            static_cast<unsigned short>(advParams.peakIntensityMin),
            static_cast<unsigned short>(advParams.peakIntensityMax)
        );
        if (peakFilterSuccess) {
            LOG_DEBUG(LogCategory::Sensor, QString("Peak Filter设置成功（命令序列方式）"));
        }
        else {
            LOG_WARNING(LogCategory::Sensor, QString("Peak Filter设置失败"));
        }
    }

    // ========== 6. 自动曝光 ==========
    if (advParams.autoExposure) {
        // 启用自动曝光 (在曝光时间寄存器设置 bit 24)
        DWORD exposure = EXPOSURE_AUTOMATIC;
        exposure |= ((advParams.autoExpMode & 0x03) << 25);  // 模式位
        ret = m_llt->SetFeature(FEATURE_FUNCTION_EXPOSURE_TIME, exposure);
        if (ret < GENERAL_FUNCTION_OK) {
            LOG_WARNING(LogCategory::Sensor, QString("启用自动曝光失败，错误码 %1").arg(ret));
        }
        LOG_DEBUG(LogCategory::Sensor, QString("自动曝光: 启用, 模式=%1, 快门对齐=%2")
            .arg(advParams.autoExpMode).arg(advParams.shutterAlign));
    }
    else {
        LOG_DEBUG(LogCategory::Sensor, QString("自动曝光: 关闭（使用手动曝光）"));
    }

    LOG_INFO(LogCategory::Sensor, tr("参数设置完成"));
    return true;
}

bool LaserSensor::readAdvancedParams(SensorAdvancedParams& advParams, QString* errMsg)
{
    if (!m_llt || !m_connected) {
        if (errMsg) *errMsg = tr("传感器未连接");
        return false;
    }

    QMutexLocker locker(&m_mutex);
    int ret = 0;
    DWORD value = 0;

    LOG_INFO(LogCategory::Sensor, tr("开始读取传感器参数..."));

    // ========== 0. 确保Profile配置和分辨率已设置 ==========
    ret = m_llt->SetProfileConfig(PROFILE);
    if (ret < GENERAL_FUNCTION_OK) {
        LOG_DEBUG(LogCategory::Sensor, QString("设置Profile配置失败，错误码 %1").arg(ret));
    }

    if (m_resolution > 0) {
        ret = m_llt->SetResolution(m_resolution);
        if (ret < GENERAL_FUNCTION_OK) {
            LOG_DEBUG(LogCategory::Sensor, QString("设置分辨率失败，错误码 %1").arg(ret));
        }
    }

    // ========== 1. 读取测量区域索引 ==========
    ret = m_llt->GetFeature(FEATURE_FUNCTION_MEASURINGFIELD, &value);
    if (ret >= GENERAL_FUNCTION_OK) {
        advParams.measuringFieldIndex = static_cast<int>(value);
        int maxFreq = getMaxFrequencyForROIIndex_Internal(value);
        LOG_DEBUG(LogCategory::Sensor, QString("测量区域索引: %1 (最大频率: %2 Hz)")
            .arg(value).arg(maxFreq > 0 ? QString::number(maxFreq) : "未知"));
    }
    else {
        LOG_WARNING(LogCategory::Sensor, QString("读取测量区域索引失败，错误码 %1").arg(ret));
    }

    // ========== 2. 读取分辨率 ==========
    DWORD resolution = 0;
    ret = m_llt->GetResolution(&resolution);
    if (ret >= GENERAL_FUNCTION_OK) {
        advParams.resolution = resolution;
        LOG_DEBUG(LogCategory::Sensor, QString("分辨率: %1").arg(resolution));
    }
    else {
        LOG_WARNING(LogCategory::Sensor, QString("读取分辨率失败，错误码 %1").arg(ret));
    }

    // ========== 3. 读取 Profile Processing ==========
    ret = m_llt->GetFeature(FEATURE_FUNCTION_PROFILE_PROCESSING, &value);
    if (ret >= GENERAL_FUNCTION_OK) {
        // 解析处理标志
        advParams.highResolution = (value & PROC_HIGH_RESOLUTION) != 0;
        advParams.calibration = (value & PROC_CALIBRATION) != 0;
        advParams.flipX = (value & PROC_FLIP_POSITION) != 0;
        advParams.flipZ = (value & PROC_FLIP_DISTANCE) != 0;

        // 反射选择
        int reflectBits = (value >> 2) & 0x07;
        switch (reflectBits) {
        case 0: advParams.reflectionSelect = 0; break;  // 全部
        case 1: advParams.reflectionSelect = 1; break;  // 第一个
        case 2: advParams.reflectionSelect = 2; break;  // 最后一个
        case 3: advParams.reflectionSelect = 3; break;  // 最大面积
        case 4: advParams.reflectionSelect = 4; break;  // 最大强度
        default: advParams.reflectionSelect = 3; break;
        }

        // 快门对齐 (bits 9-10)
        int shutterBits = (value >> 9) & 0x03;
        advParams.shutterAlign = shutterBits;  // 0=中, 1=右, 2=左, 3=关闭

        LOG_DEBUG(LogCategory::Sensor, QString("Profile Processing: 0x%1").arg(value, 8, 16, QChar('0')));
        LOG_DEBUG(LogCategory::Sensor, QString("  高分辨率: %1, 校准: %2, FlipX: %3, FlipZ: %4, 反射选择: %5, 快门对齐: %6")
            .arg(advParams.highResolution).arg(advParams.calibration)
            .arg(advParams.flipX).arg(advParams.flipZ).arg(advParams.reflectionSelect).arg(advParams.shutterAlign));
    }
    else {
        LOG_WARNING(LogCategory::Sensor, QString("读取 Profile Processing 失败，错误码 %1").arg(ret));
    }

    // ========== 4. 读取阈值 ==========
    ret = m_llt->GetFeature(FEATURE_FUNCTION_THRESHOLD, &value);
    if (ret >= GENERAL_FUNCTION_OK) {
        // 阈值百分比（低10位，单位0.1%）
        double thresholdRaw = static_cast<double>(value & 0x3FF);
        advParams.thresholdPercent = thresholdRaw / 10.0;

        // 滤波选项
        advParams.ambientLightSuppression = (value & THRESHOLD_AMBIENT_LIGHT_SUPPRESSION) != 0;
        advParams.videoFilter = (value & THRESHOLD_VIDEO_FILTER) != 0;

        LOG_DEBUG(LogCategory::Sensor, QString("阈值: 0x%1 => %2%, 背光抑制: %3, 视频滤波: %4")
            .arg(value, 8, 16, QChar('0')).arg(advParams.thresholdPercent)
            .arg(advParams.ambientLightSuppression).arg(advParams.videoFilter));
    }
    else {
        LOG_WARNING(LogCategory::Sensor, QString("读取阈值失败，错误码 %1").arg(ret));
    }

    // ========== 5. 读取激光功率 ==========
    ret = m_llt->GetFeature(FEATURE_FUNCTION_LASER, &value);
    if (ret >= GENERAL_FUNCTION_OK) {
        advParams.laserPower = static_cast<int>(value & 0x03);
        LOG_DEBUG(LogCategory::Sensor, QString("激光功率: %1").arg(advParams.laserPower));
    }
    else {
        LOG_WARNING(LogCategory::Sensor, QString("读取激光功率失败，错误码 %1").arg(ret));
    }

    // ========== 6. 读取曝光时间（检查是否自动曝光） ==========
    ret = m_llt->GetFeature(FEATURE_FUNCTION_EXPOSURE_TIME, &value);
    if (ret >= GENERAL_FUNCTION_OK) {
        advParams.autoExposure = (value & EXPOSURE_AUTOMATIC) != 0;
        // 读取自动曝光模式 (bits 25-26)
        advParams.autoExpMode = static_cast<int>((value >> 25) & 0x03);
        LOG_DEBUG(LogCategory::Sensor, QString("曝光时间寄存器: 0x%1, 自动曝光: %2, 模式: %3")
            .arg(value, 8, 16, QChar('0'))
            .arg(advParams.autoExposure ? "启用" : "关闭")
            .arg(advParams.autoExpMode));
    }
    else {
        LOG_WARNING(LogCategory::Sensor, QString("读取曝光时间失败，错误码 %1").arg(ret));
    }

    LOG_INFO(LogCategory::Sensor, tr("传感器参数读取完成"));
    return true;
}

// ============================================================================
// scanCONTROL 2950 Measuring Field preset 索引与项目参考最大频率表
// 来源于项目既有 Quick Reference 表；用于保守约束/提示，不伪装成设备实时回读的Max Frequency
// ============================================================================

static int getMaxFrequencyForROIIndex_Internal(int index)
{
    // 将索引分解为行和列
    int row = (index / 8) * 8;  // 0, 8, 16, 24, 32, 40, 48, 56, 64, 72, 80, 88, 96, 104, 112, 120
    int col = index % 8;        // 0-7

    // 最大频率表 (行 x 列) - 来自官方Quick Reference
    // 行: 0, 8, 16, 24, 32, 40, 48, 56, 64, 72, 80, 88, 96, 104, 112, 120
    // 列: +0, +1, +2, +3, +4, +5, +6, +7
    static const int freqTable[16][8] = {
        // +0    +1    +2    +3    +4    +5    +6    +7
        {144,  192,  192,  192,  284,  284,  284,  552},   // 0   (Large=0, Standard=2, Small=7)
        {181,  239,  239,  239,  354,  354,  354,  680},   // 8
        {181,  239,  239,  239,  354,  354,  354,  680},   // 16
        {181,  239,  239,  239,  354,  354,  354,  680},   // 24
        {241,  318,  318,  318,  469,  469,  469,  892},   // 32
        {241,  318,  318,  318,  469,  469,  469,  892},   // 40
        {241,  318,  318,  318,  469,  469,  469,  892},   // 48
        {241,  318,  318,  318,  469,  469,  469,  892},   // 56
        {362,  476,  476,  476,  694,  694,  694, 1282},   // 64
        {362,  476,  476,  476,  694,  694,  694, 1282},   // 72
        {362,  476,  476,  476,  694,  694,  694, 1282},   // 80
        {362,  476,  476,  476,  694,  694,  694, 1282},   // 88
        {552,  552, 1030, 1030, 1030, 1030, 1030, 1030},   // 96
        {680,  680, 1265, 1265, 1265, 1265, 1265, 1265},   // 104
        {892,  892, 1612, 1612, 1612, 1612, 1612, 1612},   // 112
        {1282, 1282, 2000, 2000, 2000, 2000, 2000, 2000},  // 120
    };

    int rowIndex = row / 8;
    if (rowIndex < 0 || rowIndex >= 16 || col < 0 || col >= 8) {
        return -1;  // 无效索引
    }

    return freqTable[rowIndex][col];
}

// ============================================================================
// 恢复传感器默认设置（修复硬件触发问题）
// ============================================================================

bool LaserSensor::restoreDefaultSettings(QString* errMsg)
{
    if (!m_llt || !m_connected) {
        if (errMsg) *errMsg = tr("传感器未连接");
        return false;
    }

    LOG_INFO(LogCategory::Sensor, tr("========== 恢复传感器设置 =========="));

    int ret = 0;
    DWORD value = 0;

    // 1. 读取当前 ROI1_PRESET 值
    ret = m_llt->GetFeature(FEATURE_FUNCTION_MEASURINGFIELD, &value);
    int currentMaxFreq = getMaxFrequencyForROIIndex_Internal(value);
    LOG_INFO(LogCategory::Sensor, QString("当前 ROI1_PRESET: %1 (最大频率: %2 Hz)")
        .arg(value).arg(currentMaxFreq > 0 ? QString::number(currentMaxFreq) : "未知"));

    // 2. 读取当前分辨率
    DWORD resolution = 0;
    ret = m_llt->GetResolution(&resolution);
    if (ret >= GENERAL_FUNCTION_OK) {
        LOG_INFO(LogCategory::Sensor, QString("当前分辨率: %1 点/轮廓").arg(resolution));
    }

    // 显示频率对照表供参考
    LOG_INFO(LogCategory::Sensor, tr(""));
    LOG_INFO(LogCategory::Sensor, tr("【测量区域索引与最大频率对照表】(来自官方Quick Reference)"));
    LOG_INFO(LogCategory::Sensor, tr("  索引  0 (Large)    -> 最大 144 Hz"));
    LOG_INFO(LogCategory::Sensor, tr("  索引  2 (Standard) -> 最大 192 Hz (50mm传感器推荐)"));
    LOG_INFO(LogCategory::Sensor, tr("  索引  7 (Small)    -> 最大 552 Hz"));
    LOG_INFO(LogCategory::Sensor, tr("  索引 32            -> 最大 241 Hz"));
    LOG_INFO(LogCategory::Sensor, tr("  索引 64            -> 最大 362 Hz"));
    LOG_INFO(LogCategory::Sensor, tr("  索引 96            -> 最大 552 Hz"));
    LOG_INFO(LogCategory::Sensor, tr("  索引 122           -> 最大 2000 Hz"));
    LOG_INFO(LogCategory::Sensor, tr(""));

    // 3. 方法1: 先尝试加载用户模式0（出厂默认）
    LOG_INFO(LogCategory::Sensor, tr("--- 方法1: 加载用户模式0（出厂默认） ---"));
    ret = m_llt->ReadWriteUserModes(0, 0);  // nWrite=0 (load), nUserMode=0 (factory)
    LOG_INFO(LogCategory::Sensor, QString("ReadWriteUserModes(0,0) 返回值: %1").arg(ret));

    // 等待传感器处理
    QThread::msleep(200);

    // 读取加载后的值
    ret = m_llt->GetFeature(FEATURE_FUNCTION_MEASURINGFIELD, &value);
    int loadedMaxFreq = getMaxFrequencyForROIIndex_Internal(value);
    LOG_INFO(LogCategory::Sensor, QString("加载用户模式0后 ROI1_PRESET: %1 (最大频率: %2 Hz)")
        .arg(value).arg(loadedMaxFreq > 0 ? QString::number(loadedMaxFreq) : "未知"));

    // 4. 方法2: 尝试设置 Standard 预设（索引2，50mm传感器推荐）
    LOG_INFO(LogCategory::Sensor, tr("--- 方法2: 尝试设置 ROI1_PRESET = 2 (Standard) ---"));

    ret = m_llt->SetFeature(FEATURE_FUNCTION_MEASURINGFIELD, 2);
    LOG_INFO(LogCategory::Sensor, QString("SetFeature(2) 返回值: %1").arg(ret));

    bool presetSuccess = (ret >= GENERAL_FUNCTION_OK);

    QThread::msleep(100);
    ret = m_llt->GetFeature(FEATURE_FUNCTION_MEASURINGFIELD, &value);
    int newMaxFreq = getMaxFrequencyForROIIndex_Internal(value);
    LOG_INFO(LogCategory::Sensor, QString("设置后 ROI1_PRESET: %1 (最大频率: %2 Hz)")
        .arg(value).arg(newMaxFreq > 0 ? QString::number(newMaxFreq) : "未知"));

    // 5. 重置 IDLE_TIME 为 0
    LOG_INFO(LogCategory::Sensor, tr("--- 重置 IDLE_TIME ---"));
    ret = m_llt->SetFeature(FEATURE_FUNCTION_IDLE_TIME, 0);
    LOG_INFO(LogCategory::Sensor, QString("IDLE_TIME = 0, 返回值: %1").arg(ret));

    // 6. 重置触发为内部触发
    LOG_INFO(LogCategory::Sensor, tr("--- 重置触发模式 ---"));
    ret = m_llt->SetFeature(FEATURE_FUNCTION_TRIGGER, TRIG_INTERNAL);
    LOG_INFO(LogCategory::Sensor, QString("TRIGGER = INTERNAL, 返回值: %1").arg(ret));

    // 7. 尝试保存当前设置到用户模式1（这样可以覆盖之前错误的设置）
    LOG_INFO(LogCategory::Sensor, tr("--- 保存当前设置到用户模式1 ---"));
    ret = m_llt->ReadWriteUserModes(1, 1);  // nWrite=1 (save), nUserMode=1
    LOG_INFO(LogCategory::Sensor, QString("ReadWriteUserModes(1,1) 返回值: %1").arg(ret));
    if (ret >= GENERAL_FUNCTION_OK) {
        LOG_INFO(LogCategory::Sensor, tr("  -> 设置已保存到用户模式1"));
    }

    // 8. 最终状态
    LOG_INFO(LogCategory::Sensor, tr("--- 最终状态 ---"));
    ret = m_llt->GetFeature(FEATURE_FUNCTION_MEASURINGFIELD, &value);
    int finalMaxFreq = getMaxFrequencyForROIIndex_Internal(value);
    LOG_INFO(LogCategory::Sensor, QString("最终 ROI1_PRESET: %1 (最大频率: %2 Hz)")
        .arg(value).arg(finalMaxFreq > 0 ? QString::number(finalMaxFreq) : "未知"));

    ret = m_llt->GetFeature(FEATURE_FUNCTION_IDLE_TIME, &value);
    LOG_INFO(LogCategory::Sensor, QString("最终 IDLE_TIME: %1").arg(value));

    ret = m_llt->GetResolution(&resolution);
    LOG_INFO(LogCategory::Sensor, QString("最终分辨率: %1 点/轮廓").arg(resolution));

    LOG_INFO(LogCategory::Sensor, tr("========== 恢复完成 =========="));
    LOG_INFO(LogCategory::Sensor, tr(""));
    LOG_INFO(LogCategory::Sensor, tr("⚠️ 重要提示："));
    LOG_INFO(LogCategory::Sensor, tr("当前设置是运行时参数，断电后可能丢失！"));
    LOG_INFO(LogCategory::Sensor, tr(""));
    LOG_INFO(LogCategory::Sensor, tr("如需永久保存或更高频率，请在官方软件中："));
    LOG_INFO(LogCategory::Sensor, tr("1. 打开 scanCONTROL Configuration Tools"));
    LOG_INFO(LogCategory::Sensor, tr("2. Advanced -> Measuring field 选择测量区域"));
    LOG_INFO(LogCategory::Sensor, tr("3. Parameters -> Store permanently on scanCONTROL"));

    return presetSuccess;
}

// ============================================================================
// 获取指定索引对应的最大采样频率（公共接口）
// ============================================================================

int LaserSensor::getMaxFrequencyForIndex(int index)
{
    return getMaxFrequencyForROIIndex_Internal(index);
}

bool LaserSensor::readTimingAndMeasuringField(unsigned int& exposureUnits,
    unsigned int& idleUnits,
    int& measuringFieldIndex,
    QString* errMsg)
{
    if (!m_llt || !m_connected) {
        if (errMsg) *errMsg = tr("传感器未连接");
        return false;
    }

    QMutexLocker locker(&m_mutex);
    DWORD field = 0;
    DWORD exposure = 0;
    DWORD idle = 0;

    int ret = m_llt->GetFeature(FEATURE_FUNCTION_MEASURINGFIELD, &field);
    if (ret < GENERAL_FUNCTION_OK) {
        if (errMsg) *errMsg = tr("读取MEASURINGFIELD失败，错误码 %1").arg(ret);
        return false;
    }
    ret = m_llt->GetFeature(FEATURE_FUNCTION_EXPOSURE_TIME, &exposure);
    if (ret < GENERAL_FUNCTION_OK) {
        if (errMsg) *errMsg = tr("读取Exposure失败，错误码 %1").arg(ret);
        return false;
    }
    ret = m_llt->GetFeature(FEATURE_FUNCTION_IDLE_TIME, &idle);
    if (ret < GENERAL_FUNCTION_OK) {
        if (errMsg) *errMsg = tr("读取Idle失败，错误码 %1").arg(ret);
        return false;
    }

    if ((exposure & EXPOSURE_AUTOMATIC) != 0) {
        if (errMsg) *errMsg = tr("传感标定要求手动曝光，但设备回读为自动曝光状态");
        return false;
    }

    measuringFieldIndex = static_cast<int>(field);
    exposureUnits = static_cast<unsigned int>(exposure);
    idleUnits = static_cast<unsigned int>(idle);
    return true;
}

// ============================================================================
// 设置测量区域索引（ROI1_PRESET）
// ============================================================================

bool LaserSensor::setMeasuringFieldIndex(int index, bool saveToUserMode, QString* errMsg)
{
    if (!m_llt || !m_connected) {
        if (errMsg) *errMsg = tr("传感器未连接");
        return false;
    }

    // 验证索引范围
    if (index < 0 || index > 127) {
        if (errMsg) *errMsg = tr("索引值超出范围 (0-127)");
        return false;
    }

    int maxFreq = getMaxFrequencyForROIIndex_Internal(index);
    LOG_INFO(LogCategory::Sensor, QString("设置测量区域索引: %1 (最大频率: %2 Hz)")
        .arg(index).arg(maxFreq > 0 ? QString::number(maxFreq) : "未知"));

    // 读取当前值
    DWORD currentValue = 0;
    int ret = m_llt->GetFeature(FEATURE_FUNCTION_MEASURINGFIELD, &currentValue);
    if (ret >= GENERAL_FUNCTION_OK) {
        int currentFreq = getMaxFrequencyForROIIndex_Internal(currentValue);
        LOG_INFO(LogCategory::Sensor, QString("当前索引: %1 (最大频率: %2 Hz)")
            .arg(currentValue).arg(currentFreq > 0 ? QString::number(currentFreq) : "未知"));
    }

    // 设置新值
    ret = m_llt->SetFeature(FEATURE_FUNCTION_MEASURINGFIELD, static_cast<DWORD>(index));
    if (ret < GENERAL_FUNCTION_OK) {
        if (errMsg) *errMsg = tr("SetFeature 失败，错误码: %1").arg(ret);
        LOG_ERROR(LogCategory::Sensor, QString("设置 ROI1_PRESET 失败，错误码: %1").arg(ret));
        return false;
    }

    // 等待传感器处理
    QThread::msleep(100);

    // 验证设置
    DWORD newValue = 0;
    ret = m_llt->GetFeature(FEATURE_FUNCTION_MEASURINGFIELD, &newValue);
    if (ret < GENERAL_FUNCTION_OK) {
        if (errMsg) *errMsg = tr("写入测量场后回读失败，错误码: %1").arg(ret);
        LOG_ERROR(LogCategory::Sensor, tr("测量场配置未通过设备回读，拒绝继续采集"));
        return false;
    }
    const int newFreq = getMaxFrequencyForROIIndex_Internal(newValue);
    LOG_INFO(LogCategory::Sensor, QString("设置后索引: %1 (最大频率: %2 Hz)")
        .arg(newValue).arg(newFreq > 0 ? QString::number(newFreq) : "未知"));
    if (static_cast<int>(newValue) != index) {
        if (errMsg) *errMsg = tr("测量场设备回读不一致：期望index=%1，实际index=%2")
            .arg(index).arg(newValue);
        LOG_ERROR(LogCategory::Sensor, tr("测量场配置未生效，拒绝继续采集"));
        return false;
    }

    // 可选：保存到用户模式1
    if (saveToUserMode) {
        LOG_INFO(LogCategory::Sensor, tr("保存设置到用户模式1..."));
        ret = m_llt->ReadWriteUserModes(1, 1);  // nWrite=1 (save), nUserMode=1
        if (ret >= GENERAL_FUNCTION_OK) {
            LOG_INFO(LogCategory::Sensor, tr("  -> 已保存到用户模式1"));
        }
        else {
            LOG_WARNING(LogCategory::Sensor, QString("保存到用户模式1失败，错误码: %1").arg(ret));
        }
    }

    LOG_INFO(LogCategory::Sensor, tr(""));
    if (saveToUserMode) {
        LOG_INFO(LogCategory::Sensor, tr("注意: 此测量区域已请求保存到用户模式1，后续启动可能继续使用该区域。"));
    }
    else {
        LOG_INFO(LogCategory::Sensor, tr("标定/临时模式: 测量区域仅运行时设置，不写入用户模式。"));
    }
    LOG_INFO(LogCategory::Sensor, tr("如需永久修改设备默认值，可使用官方软件保存。"));

    return true;
}

void LaserSensor::printDiagnostics()
{
    if (!m_llt || !m_connected) {
        LOG_WARNING(LogCategory::Sensor, tr("传感器未连接，无法获取诊断信息"));
        return;
    }

    LOG_INFO(LogCategory::Sensor, tr("========== 传感器诊断信息 =========="));

    DWORD value = 0;
    int ret = 0;

    // 0. 用户模式
    unsigned int actualUserMode = 0;
    unsigned int userModeCount = 0;
    ret = m_llt->GetActualUserMode(&actualUserMode, &userModeCount);
    if (ret >= GENERAL_FUNCTION_OK) {
        LOG_INFO(LogCategory::Sensor, QString("用户模式: %1 (可用: %2个)").arg(actualUserMode).arg(userModeCount));
        if (actualUserMode == 0) {
            LOG_INFO(LogCategory::Sensor, tr("  -> 出厂默认模式"));
        }
    }
    else {
        LOG_DEBUG(LogCategory::Sensor, QString("获取用户模式失败，错误码 %1").arg(ret));
    }

    // 0.5 分辨率（点数/轮廓）
    DWORD resolution = 0;
    ret = m_llt->GetResolution(&resolution);
    if (ret >= GENERAL_FUNCTION_OK) {
        LOG_INFO(LogCategory::Sensor, QString("分辨率: %1 点/轮廓").arg(resolution));
        if (resolution >= 1280) {
            LOG_INFO(LogCategory::Sensor, tr("  -> 全分辨率模式"));
        }
        else if (resolution >= 640) {
            LOG_INFO(LogCategory::Sensor, tr("  -> 中等分辨率"));
        }
        else {
            LOG_INFO(LogCategory::Sensor, tr("  -> 低分辨率，可支持高速采样"));
        }
    }

    // 1. ROI1_PRESET / MEASURINGFIELD (0xf0f00880)
    LOG_INFO(LogCategory::Sensor, tr("--- ROI (测量区域) 相关寄存器 ---"));
    ret = m_llt->GetFeature(FEATURE_FUNCTION_MEASURINGFIELD, &value);
    if (ret >= GENERAL_FUNCTION_OK) {
        int maxFreq = getMaxFrequencyForROIIndex_Internal(value);
        LOG_INFO(LogCategory::Sensor, QString("ROI1_PRESET (0xf0f00880): %1").arg(value));

        if (maxFreq > 0) {
            LOG_INFO(LogCategory::Sensor, QString("  -> 最大采样频率: %1 Hz").arg(maxFreq));
        }

        // 解析预设名称
        if (value == 0) {
            LOG_INFO(LogCategory::Sensor, tr("  -> Large 预设 (覆盖完整测量范围，频率最低)"));
        }
        else if (value == 2) {
            LOG_INFO(LogCategory::Sensor, tr("  -> Standard 预设 (50mm传感器推荐值)"));
        }
        else if (value == 4) {
            LOG_INFO(LogCategory::Sensor, tr("  -> Standard 预设 (100mm传感器推荐值)"));
        }
        else if (value == 7) {
            LOG_INFO(LogCategory::Sensor, tr("  -> Small 预设 (较小区域，较高频率)"));
        }
        else if (value > 7) {
            int row = (value / 8) * 8;
            int col = value % 8;
            LOG_INFO(LogCategory::Sensor, QString("  -> 自定义区域: 行=%1, 列=%2").arg(row).arg(col));
        }

        // V32.11 no longer treats 200 Hz as universal. The Parameters page
        // clamps the operator-selected hardware-sync rate to this field limit.
        if (maxFreq > 0) {
            LOG_INFO(LogCategory::Sensor,
                tr("  -> V32.11硬件同步频率上限将按该测量区域限制为 %1 Hz").arg(maxFreq));
        }
    }
    else {
        LOG_WARNING(LogCategory::Sensor, QString("读取 ROI1_PRESET 失败，错误码 %1").arg(ret));
    }

    // 2. ROI1_POSITION (0xf0b0200c) - X轴起始点和大小
    ret = m_llt->GetFeature(FEATURE_FUNCTION_ROI1_POSITION, &value);
    if (ret >= GENERAL_FUNCTION_OK) {
        unsigned short startX = value & 0xFFFF;
        unsigned short sizeX = (value >> 16) & 0xFFFF;
        LOG_INFO(LogCategory::Sensor, QString("ROI1_POSITION (0xf0b0200c): 0x%1 (startX=%2, sizeX=%3)")
            .arg(value, 8, 16, QChar('0')).arg(startX).arg(sizeX));
    }
    else {
        LOG_DEBUG(LogCategory::Sensor, QString("ROI1_POSITION 读取失败 (可能固件 < v43)"));
    }

    // 3. ROI1_DISTANCE (0xf0b02008) - Z轴起始点和大小
    ret = m_llt->GetFeature(FEATURE_FUNCTION_ROI1_DISTANCE, &value);
    if (ret >= GENERAL_FUNCTION_OK) {
        unsigned short startZ = value & 0xFFFF;
        unsigned short sizeZ = (value >> 16) & 0xFFFF;
        LOG_INFO(LogCategory::Sensor, QString("ROI1_DISTANCE (0xf0b02008): 0x%1 (startZ=%2, sizeZ=%3)")
            .arg(value, 8, 16, QChar('0')).arg(startZ).arg(sizeZ));
    }
    else {
        LOG_DEBUG(LogCategory::Sensor, QString("ROI1_DISTANCE 读取失败 (可能固件 < v43)"));
    }

    // 4. TRIGGER
    LOG_INFO(LogCategory::Sensor, tr("--- 触发和时序相关 ---"));
    ret = m_llt->GetFeature(FEATURE_FUNCTION_TRIGGER, &value);
    if (ret >= GENERAL_FUNCTION_OK) {
        LOG_INFO(LogCategory::Sensor, QString("TRIGGER: 0x%1").arg(value, 8, 16, QChar('0')));
        if (value & TRIG_EXT_ACTIVE) {
            LOG_INFO(LogCategory::Sensor, tr("  -> 外部触发已激活"));
        }
        else {
            LOG_INFO(LogCategory::Sensor, tr("  -> 内部触发"));
        }
    }

    // 5. EXPOSURE_TIME
    DWORD exposureTime = 0;
    ret = m_llt->GetFeature(FEATURE_FUNCTION_EXPOSURE_TIME, &exposureTime);
    if (ret >= GENERAL_FUNCTION_OK) {
        LOG_INFO(LogCategory::Sensor, QString("EXPOSURE_TIME: %1 (10µs) = %2 ms").arg(exposureTime).arg(exposureTime / 100.0));
    }

    // 6. IDLE_TIME
    DWORD idleTime = 0;
    ret = m_llt->GetFeature(FEATURE_FUNCTION_IDLE_TIME, &idleTime);
    if (ret >= GENERAL_FUNCTION_OK) {
        LOG_INFO(LogCategory::Sensor, QString("IDLE_TIME: %1 (10µs) = %2 ms").arg(idleTime).arg(idleTime / 100.0));

        // 计算理论最大采样频率
        if (exposureTime > 0 || idleTime > 0) {
            double totalTime = (exposureTime + idleTime) / 100.0;  // 转换为毫秒
            if (totalTime > 0) {
                double maxFreq = 1000.0 / totalTime;
                LOG_INFO(LogCategory::Sensor, QString("  -> 基于时序的理论最大频率: %.1f Hz").arg(maxFreq));
            }
        }
    }

    // 7. PROFILE_PROCESSING
    ret = m_llt->GetFeature(FEATURE_FUNCTION_PROFILE_PROCESSING, &value);
    if (ret >= GENERAL_FUNCTION_OK) {
        LOG_INFO(LogCategory::Sensor, QString("PROFILE_PROCESSING: 0x%1").arg(value, 8, 16, QChar('0')));
    }

    // 8. SHARPNESS / EXTRA_PARAMETER
    LOG_INFO(LogCategory::Sensor, tr("--- Peak Filter 相关 ---"));
    ret = m_llt->GetFeature(FEATURE_FUNCTION_SHARPNESS, &value);
    if (ret >= GENERAL_FUNCTION_OK) {
        LOG_INFO(LogCategory::Sensor, QString("SHARPNESS/EXTRA_PARAMETER: 0x%1").arg(value, 8, 16, QChar('0')));
    }

    // 9. PEAKFILTER_WIDTH (固件 v43+)
    ret = m_llt->GetFeature(FEATURE_FUNCTION_PEAKFILTER_WIDTH, &value);
    if (ret >= GENERAL_FUNCTION_OK) {
        unsigned short minW = value & 0xFFFF;
        unsigned short maxW = (value >> 16) & 0xFFFF;
        LOG_INFO(LogCategory::Sensor, QString("PEAKFILTER_WIDTH: 0x%1 (min=%2, max=%3)")
            .arg(value, 8, 16, QChar('0')).arg(minW).arg(maxW));
    }
    else {
        LOG_DEBUG(LogCategory::Sensor, QString("PEAKFILTER_WIDTH 不支持 (固件 < v43)"));
    }

    // 10. PEAKFILTER_HEIGHT (固件 v43+)
    ret = m_llt->GetFeature(FEATURE_FUNCTION_PEAKFILTER_HEIGHT, &value);
    if (ret >= GENERAL_FUNCTION_OK) {
        unsigned short minH = value & 0xFFFF;
        unsigned short maxH = (value >> 16) & 0xFFFF;
        LOG_INFO(LogCategory::Sensor, QString("PEAKFILTER_HEIGHT: 0x%1 (min=%2, max=%3)")
            .arg(value, 8, 16, QChar('0')).arg(minH).arg(maxH));
    }
    else {
        LOG_DEBUG(LogCategory::Sensor, QString("PEAKFILTER_HEIGHT 不支持 (固件 < v43)"));
    }

    // 11. DIGITAL_IO
    LOG_INFO(LogCategory::Sensor, tr("--- IO 相关 ---"));
    ret = m_llt->GetFeature(FEATURE_FUNCTION_DIGITAL_IO, &value);
    if (ret >= GENERAL_FUNCTION_OK) {
        LOG_INFO(LogCategory::Sensor, QString("DIGITAL_IO: 0x%1").arg(value, 8, 16, QChar('0')));
    }

    LOG_INFO(LogCategory::Sensor, tr("========== 诊断信息结束 =========="));
    LOG_INFO(LogCategory::Sensor, tr(""));
    LOG_INFO(LogCategory::Sensor, tr("提示：如果 ROI1_PRESET 值异常，请在官方软件中："));
    LOG_INFO(LogCategory::Sensor, tr("1. 打开 scanCONTROL Configuration Tools"));
    LOG_INFO(LogCategory::Sensor, tr("2. 点击 Advanced -> Measuring field 选项卡"));
    LOG_INFO(LogCategory::Sensor, tr("3. 选择合适的测量区域预设"));
    LOG_INFO(LogCategory::Sensor, tr("4. 点击 'Store parameters permanently on scanCONTROL'"));
}

// ============================================================================
// Video 模式配置
// ============================================================================

bool LaserSensor::configureVideo(const SensorParams& params, QString* errMsg)
{
    if (!m_llt || !m_connected) {
        if (errMsg) *errMsg = tr("传感器未连接");
        return false;
    }

    // Always enforce the LLT 29xx Video upper limit at the device-facing
    // boundary. This also protects callers outside the calibration UI.
    const auto videoPlan = mls::calibration::SensorCalibrationTiming::videoPlan(
        static_cast<int>(params.exposureTime));
    if (!videoPlan.valid) {
        if (errMsg) *errMsg = tr("Video仅允许25Hz内部时钟，曝光%1 (10µs)无法满足40ms周期：%2")
            .arg(params.exposureTime).arg(QString::fromLatin1(videoPlan.reason));
        return false;
    }
    SensorParams videoParams = params;
    videoParams.idleTime = static_cast<unsigned int>(videoPlan.idle10us);
    if (videoParams.idleTime != params.idleTime) {
        LOG_WARNING(LogCategory::Sensor,
            tr("Video时序与Profile时序不同：忽略传入Idle=%1，Video固定25Hz并设置Idle=%2 (10µs)")
                .arg(params.idleTime).arg(videoParams.idleTime));
    }

    // 如果正在运行，先停止
    if (isRunning()) {
        stopAll();
    }

    QMutexLocker locker(&m_mutex);

    int ret = 0;

    // 设置 Profile 配置为 VIDEO_IMAGE
    ret = m_llt->SetProfileConfig(VIDEO_IMAGE);
    if (ret < GENERAL_FUNCTION_OK) {
        if (errMsg) *errMsg = tr("SetProfileConfig(VIDEO_IMAGE) 失败，错误码 %1").arg(ret);
        return false;
    }

    // 设置内部触发（Video 模式只支持内部触发）
    ret = m_llt->SetFeature(FEATURE_FUNCTION_TRIGGER, TRIG_INTERNAL);
    if (ret < GENERAL_FUNCTION_OK) {
        if (errMsg) *errMsg = tr("设置内部触发失败，错误码 %1").arg(ret);
        return false;
    }

    // 设置曝光时间
    ret = m_llt->SetFeature(FEATURE_FUNCTION_EXPOSURE_TIME, params.exposureTime);
    if (ret < GENERAL_FUNCTION_OK) {
        if (errMsg) *errMsg = tr("设置曝光时间失败，错误码 %1").arg(ret);
        return false;
    }

    // Raw Video never uses the Profile timing. Set the 25 Hz video Idle.
    ret = m_llt->SetFeature(FEATURE_FUNCTION_IDLE_TIME, videoParams.idleTime);
    if (ret < GENERAL_FUNCTION_OK) {
        if (errMsg) *errMsg = tr("设置 idle 时间失败，错误码 %1").arg(ret);
        return false;
    }

    // 设置 Packet Size
    // 30xx 系列需要更大的包大小（Jumbo Frames）
    unsigned int packetSize = 1024;
    if (m_scannerType >= scanCONTROL30xx_25 && m_scannerType <= scanCONTROL30xx_xxx) {
        packetSize = 8192;  // 30xx 需要 Jumbo Frames
    }

    ret = m_llt->SetPacketSize(packetSize);
    if (ret < GENERAL_FUNCTION_OK) {
        if (errMsg) *errMsg = tr("SetPacketSize 失败，错误码 %1").arg(ret);
        return false;
    }

    m_params = videoParams;
    m_workMode = SensorWorkMode::Video;

    LOG_INFO(LogCategory::Sensor,
        tr("Video模式配置：固定%1Hz，曝光=%2、Idle=%3 (均为10µs)，包大小=%4")
            .arg(videoPlan.theoreticalRateHz, 0, 'f', 2)
            .arg(videoParams.exposureTime).arg(videoParams.idleTime).arg(packetSize));

    return true;
}

// ============================================================================
// V27.6.16M Profile raw-ingress / decoder
// ============================================================================

void LaserSensor::prepareProfileIngress(unsigned int resolution)
{
    // scanCONTROL PROFILE callback is resolution*64 bytes on the tested 29xx.
    // Keep at least 128 KiB/slot so supported larger resolutions do not force
    // callback-time reallocations. 128 slots provide ~640 ms at 200 Hz.
    const std::size_t slotBytes = std::max<std::size_t>(
        static_cast<std::size_t>(resolution) * 64u, 128u * 1024u);
    for (auto& slot : m_profileIngressSlots) {
        slot.bytes.resize(slotBytes);
        slot.size = 0;
        slot.callbackPcUs = 0;
        slot.callbackDonePcUs = 0;
        slot.callbackOrdinal = 0;
        slot.ingressDepth = 0;
    }
    m_profileIngressWriteSeq.store(0, std::memory_order_release);
    m_profileIngressReadSeq.store(0, std::memory_order_release);
    {
        QMutexLocker lock(&m_latestProfileMutex);
        m_hasLatestDecodedProfile = false;
        m_latestDecodedProfile = LaserProfile{};
    }
    {
        QMutexLocker lock(&m_sensorCalibrationProfileMutex);
        m_hasLatestSensorCalibrationProfile = false;
        m_latestSensorCalibrationProfile = SensorCalibrationProfileFrame{};
    }
}

void LaserSensor::stopProfileDecoder()
{
    if (!m_profileDecodeThread) return;
    m_profileDecodeThread->requestStop();
    if (!m_profileDecodeThread->wait(3000)) {
        LOG_WARNING(LogCategory::Sensor, tr("ProfileDecodeThread退出超过3秒；继续等待，禁止释放LLT对象"));
        m_profileDecodeThread->wait();
    }
    delete m_profileDecodeThread;
    m_profileDecodeThread = nullptr;
}

bool LaserSensor::decodeOneQueuedProfile()
{
    const quint64 readSeq = m_profileIngressReadSeq.load(std::memory_order_relaxed);
    const quint64 writeSeq = m_profileIngressWriteSeq.load(std::memory_order_acquire);
    if (readSeq >= writeSeq) return false;

    RawProfileSlot& slot = m_profileIngressSlots[readSeq % kProfileIngressSlotCount];
    const qint64 decodeStartUs = GlobalTimeSync::instance().timestampUs();
    const qint64 queueUs = std::max<qint64>(0, decodeStartUs - slot.callbackDonePcUs);
    atomicMax(m_maxProfileDecoderQueueUs, queueUs);

    CInterfaceLLT* localLlt = nullptr;
    unsigned int resolution = 0;
    TScannerType type;
    {
        QMutexLocker locker(&m_mutex);
        localLlt = m_llt;
        resolution = m_resolution;
        type = m_scannerType;
    }

    if (!localLlt || resolution == 0 || slot.size == 0) {
        m_profileIngressReadSeq.store(readSeq + 1, std::memory_order_release);
        return true;
    }

    QVector<double> vx(resolution);
    QVector<double> vz(resolution);

    // Rich auxiliary conversion is strictly opt-in for the sensor calibration
    // workflow. Hardware-sync never enables this flag, so its 200 Hz path keeps
    // the same X/Z-only allocations and conversion request as V27.6.16P.
    const bool richCalibration = m_sensorCalibrationRichProfileEnabled.load(std::memory_order_acquire);
    QVector<unsigned short> width;
    QVector<unsigned short> maximum;
    QVector<unsigned short> threshold;
    QVector<unsigned int> moment0;
    QVector<unsigned int> moment1;
    if (richCalibration) {
        width.resize(static_cast<int>(resolution));
        maximum.resize(static_cast<int>(resolution));
        threshold.resize(static_cast<int>(resolution));
        moment0.resize(static_cast<int>(resolution));
        moment1.resize(static_cast<int>(resolution));
    }

    const int ret = localLlt->ConvertProfile2Values(
        slot.bytes.data(),
        resolution,
        PROFILE,
        type,
        0, 1,
        richCalibration ? width.data() : nullptr,
        richCalibration ? maximum.data() : nullptr,
        richCalibration ? threshold.data() : nullptr,
        vx.data(), vz.data(),
        richCalibration ? moment0.data() : nullptr,
        richCalibration ? moment1.data() : nullptr);
    m_lastProfileConvertResult.store(ret);
    const qint64 decodeDoneUs = GlobalTimeSync::instance().timestampUs();
    const qint64 decodeServiceUs = std::max<qint64>(0, decodeDoneUs - decodeStartUs);
    atomicMax(m_maxProfileDecodeServiceUs, decodeServiceUs);

    if (!(ret & CONVERT_X) || !(ret & CONVERT_Z)) {
        const quint64 failureNo = m_profileConvertFailureCount.fetch_add(1) + 1;
        if (failureNo <= 3 || (failureNo % 100) == 0) {
            LOG_WARNING(LogCategory::Sensor,
                tr("Profile raw ingress已接收但DecodeThread转换失败: ret=%1, bytes=%2, resolution=%3, callbackOrdinal=%4, failureNo=%5")
                .arg(ret).arg(slot.size).arg(resolution).arg(slot.callbackOrdinal).arg(failureNo));
        }
        if (!m_suppressProfileEmission.load() && !m_profileIngressFaultEmitted.exchange(true)) {
            emit profileIngressFault(
                tr("正式采集Profile转换失败：callbackOrdinal=%1 ret=%2；本段禁止判为完整")
                .arg(slot.callbackOrdinal).arg(ret));
        }
        m_profileIngressReadSeq.store(readSeq + 1, std::memory_order_release);
        return true;
    }

    m_convertedProfileCount.fetch_add(1);

    double tOpen = 0.0, tClose = 0.0;
    unsigned int idx = 0;
    unsigned int cmmEdgeCount = 0, cmmCount = 0;
    int cmmTrig = 0, cmmActive = 0;
    bool cmmTimestampDecoded = false;
    if (slot.size >= static_cast<std::size_t>(resolution) * 64u) {
        unsigned char* ts = slot.bytes.data() + (static_cast<std::size_t>(resolution) * 64u - 16u);
        localLlt->Timestamp2TimeAndCount(ts, &tOpen, &tClose, &idx);
        if (m_cmmConfigured.load(std::memory_order_acquire)) {
            // The scanCONTROL SDK function returns void, NOT int. It has no
            // success/error code. Mark only that the optional timestamp was
            // decoded; SyncManager separately requires the CMM-active flag.
            localLlt->Timestamp2CmmTriggerAndInCounter(
                ts, &cmmEdgeCount, &cmmTrig, &cmmActive, &cmmCount);
            cmmTimestampDecoded = true;
        }
    }

    LaserProfile profile;
    profile.x = std::move(vx);
    profile.z = std::move(vz);
    profile.tOpen = tOpen;
    profile.tClose = tClose;
    profile.index = idx;
    profile.cmmValid = cmmTimestampDecoded;
    profile.cmmActive = (cmmActive != 0);
    profile.cmmTrigger = (cmmTrig != 0);
    profile.cmmCount = cmmCount;
    profile.tPcUs = slot.callbackPcUs;
    profile.callbackOrdinal = slot.callbackOrdinal;
    profile.rawBytes = slot.size;
    profile.ingressDepth = slot.ingressDepth;
    profile.callbackServiceUs = std::max<qint64>(0, slot.callbackDonePcUs - slot.callbackPcUs);
    profile.decoderQueueUs = queueUs;
    profile.decodeServiceUs = decodeServiceUs;
    profile.decodeDonePcUs = decodeDoneUs;

    {
        QMutexLocker lock(&m_latestProfileMutex);
        m_latestDecodedProfile = profile;
        m_hasLatestDecodedProfile = true;
    }

    if (richCalibration) {
        SensorCalibrationProfileFrame richFrame;
        richFrame.profile = profile;
        richFrame.convertFlags = ret;
        if (ret & CONVERT_WIDTH) richFrame.width = std::move(width);
        if (ret & CONVERT_MAXIMUM) richFrame.maximum = std::move(maximum);
        if (ret & CONVERT_THRESHOLD) richFrame.threshold = std::move(threshold);
        if (ret & CONVERT_M0) richFrame.moment0 = std::move(moment0);
        if (ret & CONVERT_M1) richFrame.moment1 = std::move(moment1);

        QMutexLocker lock(&m_sensorCalibrationProfileMutex);
        m_latestSensorCalibrationProfile = std::move(richFrame);
        m_hasLatestSensorCalibrationProfile = true;
    }

    if (!m_suppressProfileEmission.load(std::memory_order_acquire))
        emit profileReceived(profile);
    m_profileIngressReadSeq.store(readSeq + 1, std::memory_order_release);
    return true;
}

// ============================================================================
// 采集控制 - Profile 模式
// ============================================================================

bool LaserSensor::configureCmmOutput(QString* errMsg)
{
    QMutexLocker lock(&m_mutex);
    if (!m_llt || !m_connected || m_profileRunning.load()) {
        if (errMsg)*errMsg = tr("CMM配置要求传感器已连接且传输已停止");
        return false;
    }
    m_cmmConfigured = false;
    m_cmmPortReadbackValid.store(false);
    // Preserve the current digital input bits while configuring H3 RS422/CMM.
    DWORD port = 0;
    int rc = m_llt->GetFeature(FEATURE_FUNCTION_DIGITAL_IO, &port);
    if (rc < GENERAL_FUNCTION_OK) { if (errMsg)*errMsg = tr("CMM读取DIGITAL_IO失败: %1").arg(rc); return false; }
    port = (port & ~static_cast<DWORD>(0x0fu)) |
        static_cast<DWORD>(MULTI_RS422_CMM | MULTI_RS422_TERM_OFF);
    rc = m_llt->SetFeature(FEATURE_FUNCTION_DIGITAL_IO, port);
    if (rc < GENERAL_FUNCTION_OK) { if (errMsg)*errMsg = tr("RS422设为CMM输出失败: %1").arg(rc); return false; }
    DWORD back = 0;
    rc = m_llt->GetFeature(FEATURE_FUNCTION_DIGITAL_IO, &back);
    m_hwDigitalIoRequested.store(static_cast<unsigned int>(port));
    m_hwDigitalIoReadback.store(static_cast<unsigned int>(back));
    if (rc < GENERAL_FUNCTION_OK || !mls::sensor::trigger::digitalIoMatches(3, port, back)) {
        if (errMsg)*errMsg = tr("CMM RS422端口回读失败: %1 / 0x%2").arg(rc).arg(back, 8, 16, QChar('0'));
        return false;
    }
    // SDK v4.0 29xx Section 6.16: configure ratio/skew/port BEFORE divisor.
    for (DWORD cmd : {DWORD(0x00000401u), DWORD(0x00000800u), DWORD(0x00000c00u), DWORD(0u)}) {
        rc = m_llt->SetFeature(FEATURE_FUNCTION_CMM_TRIGGER, cmd);
        if (rc < GENERAL_FUNCTION_OK) {
            if (errMsg)*errMsg = tr("CMM配置指令0x%1失败: %2").arg(cmd, 8, 16, QChar('0')).arg(rc);
            return false;
        }
    }
    // DIGITAL_IO/CMM commands can change other device feature bits. Verify
    // the internal trigger *again* after all CMM configuration, without
    // conflating this with an external-edge readback.
    DWORD triggerBack = 0;
    m_hwTriggerReadbackValid.store(false);
    rc = m_llt->GetFeature(FEATURE_FUNCTION_TRIGGER, &triggerBack);
    if (rc < GENERAL_FUNCTION_OK) {
        if (errMsg)*errMsg = tr("H3配置CMM后内部触发读回失败: %1").arg(rc);
        return false;
    }
    m_hwTriggerRequested.store(static_cast<unsigned int>(TRIG_INTERNAL));
    m_hwTriggerReadback.store(static_cast<unsigned int>(triggerBack));
    if (!mls::sensor::trigger::sensorTriggerMatches(3, TRIG_INTERNAL, triggerBack)) {
        if (errMsg)*errMsg = tr("H3配置CMM后设备意外处于外部触发，TRIGGER=0x%1")
            .arg(static_cast<qulonglong>(triggerBack), 8, 16, QChar('0'));
        return false;
    }
    m_hwTriggerReadbackValid.store(true);
    m_cmmPortReadbackValid.store(true);
    m_cmmConfigured = true;
    LOG_INFO(LogCategory::Sensor, tr("H3内部触发及CMM端口读回通过；divider=0(预备)，skew=0，ratio=1:1，RS422终端关闭"));
    return true;
}

bool LaserSensor::startCmmGated(int nominalHz, QString* errMsg)
{
    if (!m_connected || !m_llt || !m_cmmConfigured.load() || m_profileRunning.load() || m_videoRunning.load()) {
        if (errMsg)*errMsg = tr("H3启动条件不满足（连接、CMM配置或互斥传输）"); return false;
    }
    resetProfileCallbackDiagnostics();
    prepareProfileIngress(m_resolution);
    m_profileDecodeThread = new ProfileDecodeThread(this);
    m_profileDecodeThread->start(QThread::HighestPriority);
    int rc = m_llt->RegisterCallback(STD_CALL, reinterpret_cast<void*>(profileCallback), this);
    if (rc < GENERAL_FUNCTION_OK) { stopProfileDecoder(); if (errMsg)*errMsg = tr("CMM RegisterCallback失败: %1").arg(rc); return false; }
    const unsigned int forerun = static_cast<unsigned int>(std::max(8, nominalHz / 4));
    // H3 only: ECI R1 FIFOs must already be armed; sensor provides internal clock.
    rc = m_llt->StartTransmissionAndCmmTrigger(1, NORMAL_TRANSFER, forerun, nullptr, AVI, 3000);
    if (rc < GENERAL_FUNCTION_OK) {
        // Capture diagnostics BEFORE unregistering/tearing down SDK transfer.
        // Counters observed after cleanup cannot prove what happened during
        // StartTransmissionAndCmmTrigger's blocking timeout interval.
        const auto before = profileCallbackDiagnostics();
        LOG_WARNING(LogCategory::Sensor,
            tr("H3 SDK CMM启动失败: rc=%1 raw=%2 converted=%3")
            .arg(rc).arg(before.rawCallbacks).arg(before.convertedProfiles));
        // A failed SDK special start may have already activated the divisor.
        // Always return the RS422 CMM output to its safe disabled state.
        m_llt->SetFeature(FEATURE_FUNCTION_CMM_TRIGGER, DWORD(0));
        m_llt->TransferProfiles(NORMAL_TRANSFER, false);
        m_llt->RegisterCallback(STD_CALL, nullptr, nullptr);
        stopProfileDecoder();
        if (errMsg) {
            const QString sdkStage = rc == ERROR_CMMTRIGGER_NO_DIVISOR
                ? QStringLiteral("NO_DIVISOR") :
                rc == ERROR_CMMTRIGGER_TIMEOUT_AFTER_TRANSFERPROFILES
                ? QStringLiteral("TIMEOUT_AFTER_TRANSFERPROFILES") :
                rc == ERROR_CMMTRIGGER_TIMEOUT_AFTER_SETCMMTRIGGER
                ? QStringLiteral("TIMEOUT_AFTER_SETCMMTRIGGER") :
                QStringLiteral("OTHER_SDK_ERROR");
            *errMsg = tr("StartTransmissionAndCmmTrigger失败: %1 (%2)；已尝试关闭CMM。"
                "注意：SDK超时阶段并不单独证明电气输出不存在")
                .arg(rc).arg(sdkStage);
        }
        return false;
    }
    m_cmmGated = true;
    m_profileRunning = true;
    LOG_INFO(LogCategory::Sensor, tr("H3 LLT特殊CMM传输已启动，forerun=%1帧，divider=1").arg(forerun));
    return true;
}

bool LaserSensor::stopCmmGated(QString* errMsg)
{
    if (!m_cmmGated.load()) {
        if (errMsg)*errMsg = tr("当前并非CMM特殊传输"); return false;
    }
    const int rc = m_llt->StopTransmissionAndCmmTrigger(0, 500);
    if (rc < GENERAL_FUNCTION_OK) {
        // Controller must remain armed until this failure is reported and stopped.
        if (errMsg)*errMsg = tr("StopTransmissionAndCmmTrigger失败: %1（本次数据应判无效）").arg(rc);
        m_llt->SetFeature(FEATURE_FUNCTION_CMM_TRIGGER, 0);
        m_llt->TransferProfiles(NORMAL_TRANSFER, false);
    }
    m_llt->RegisterCallback(STD_CALL, nullptr, nullptr);
    m_cmmGated = false;
    m_profileRunning = false;
    stopProfileDecoder();
    return rc >= GENERAL_FUNCTION_OK;
}

bool LaserSensor::startContinuous(QString* errMsg)
{
    resetProfileCallbackDiagnostics();
    CInterfaceLLT* llt = nullptr;
    bool connected = false;

    {
        QMutexLocker locker(&m_mutex);
        llt = m_llt;
        connected = m_connected;
    }

    if (!llt || !connected) {
        if (errMsg) *errMsg = tr("传感器未连接");
        return false;
    }

    if (m_profileRunning) {
        if (errMsg) *errMsg = tr("Profile 传输已在运行");
        return false;
    }

    if (m_videoRunning) {
        if (errMsg) *errMsg = tr("Video 模式正在运行，请先停止");
        return false;
    }

    prepareProfileIngress(m_resolution);
    m_profileDecodeThread = new ProfileDecodeThread(this);
    m_profileDecodeThread->start(QThread::HighestPriority);

    // 注册回调。此后callback只做timestamp + memcpy到预分配ring。
    int ret = llt->RegisterCallback(STD_CALL,
        reinterpret_cast<void*>(profileCallback),
        this);
    if (ret < GENERAL_FUNCTION_OK) {
        stopProfileDecoder();
        if (errMsg) *errMsg = tr("RegisterCallback 失败，错误码 %1").arg(ret);
        return false;
    }

    // 启动连续传输
    ret = llt->TransferProfiles(NORMAL_TRANSFER, true);
    if (ret < GENERAL_FUNCTION_OK) {
        if (errMsg) *errMsg = tr("TransferProfiles 失败，错误码 %1").arg(ret);
        llt->RegisterCallback(STD_CALL, nullptr, nullptr);
        stopProfileDecoder();
        return false;
    }

    m_profileRunning.store(true);
    LOG_INFO(LogCategory::Sensor,
        tr("轮廓流采集已启动：LLT callback轻量raw-ring=%1槽，DecodeThread=HighestPriority")
        .arg(kProfileIngressSlotCount));

    return true;
}

void LaserSensor::stopContinuous()
{
    if (m_cmmGated.load()) {
        QString e;
        if (!stopCmmGated(&e)) LOG_WARNING(LogCategory::Sensor, e);
        return;
    }
    if (!m_profileRunning.load()) return;

    CInterfaceLLT* llt = nullptr;
    bool connected = false;

    {
        QMutexLocker locker(&m_mutex);
        llt = m_llt;
        connected = m_connected;
    }

    if (!llt || !connected) return;

    // 先阻止新的SDK callback，再让DecodeThread排空已进入raw ring的尾帧。
    llt->TransferProfiles(NORMAL_TRANSFER, false);
    llt->RegisterCallback(STD_CALL, nullptr, nullptr);
    m_profileRunning.store(false);
    stopProfileDecoder();
    const auto diag = profileCallbackDiagnostics();
    LOG_INFO(LogCategory::Sensor,
        tr("轮廓流采集已停止：raw=%1 converted=%2 ingressDrop=%3 rawRingHighWater=%4/%5 maxCallback=%6us maxDecoderQueue=%7us maxDecode=%8us")
        .arg(diag.rawCallbacks).arg(diag.convertedProfiles).arg(diag.ingressDrops)
        .arg(diag.rawRingHighWater).arg(diag.rawRingCapacity)
        .arg(diag.maxCallbackServiceUs).arg(diag.maxDecoderQueueUs).arg(diag.maxDecodeServiceUs));
}

// ============================================================================
// 采集控制 - Video 模式
// ============================================================================

bool LaserSensor::startVideoStream(QString* errMsg)
{
    CInterfaceLLT* llt = nullptr;
    bool connected = false;

    {
        QMutexLocker locker(&m_mutex);
        llt = m_llt;
        connected = m_connected;
    }

    if (!llt || !connected) {
        if (errMsg) *errMsg = tr("传感器未连接");
        return false;
    }

    if (m_videoRunning) {
        if (errMsg) *errMsg = tr("Video 流已在运行");
        return false;
    }

    if (m_profileRunning) {
        if (errMsg) *errMsg = tr("Profile 模式正在运行，请先停止");
        return false;
    }

    // 等待参数设置生效
    QThread::msleep(120);

    // 选择 Video 模式
    // 30xx 系列使用 VIDEO_MODE_0（降采样）节省带宽
    TTransferVideoType videoType = VIDEO_MODE_1;
    if (m_scannerType >= scanCONTROL30xx_25 && m_scannerType <= scanCONTROL30xx_xxx) {
        videoType = VIDEO_MODE_0;
    }

    unsigned int width = 0, height = 0;
    int ret = llt->TransferVideoStream(videoType, true, &width, &height);
    if (ret < GENERAL_FUNCTION_OK) {
        if (errMsg) *errMsg = tr("TransferVideoStream 失败，错误码 %1").arg(ret);
        return false;
    }

    m_videoWidth = static_cast<int>(width);
    m_videoHeight = static_cast<int>(height);

    LOG_INFO(LogCategory::Sensor, QString("视频流已启动, 尺寸: %1x%2").arg(m_videoWidth).arg(m_videoHeight));

    // 等待传输预热
    QThread::msleep(500);

    // 启动读取线程
    m_videoThread = new VideoStreamThread(this, llt, m_videoWidth, m_videoHeight, this);
    m_videoThread->start();

    m_videoRunning.store(true);

    return true;
}

void LaserSensor::stopVideoStream()
{
    if (!m_videoRunning.load() && !m_videoThread) return;

    CInterfaceLLT* llt = nullptr;
    bool connected = false;
    TTransferVideoType videoType = VIDEO_MODE_1;

    {
        QMutexLocker locker(&m_mutex);
        llt = m_llt;
        connected = m_connected.load();
        if (m_scannerType >= scanCONTROL30xx_25 && m_scannerType <= scanCONTROL30xx_xxx) {
            videoType = VIDEO_MODE_0;
        }
    }

    // First request the reader thread to leave, then stop the vendor transfer.
    // Turning the stream off before wait() helps unblock GetActualProfile() when
    // the sensor/network disappears, which was one cause of slow or stuck exit.
    if (m_videoThread) {
        m_videoThread->requestStop();
    }
    if (llt && connected) {
        llt->TransferVideoStream(videoType, false, nullptr, nullptr);
    }

    if (m_videoThread) {
        if (!m_videoThread->wait(3000)) {
            LOG_WARNING(LogCategory::Sensor, tr("Video 读取线程 3 秒内未退出，继续等待释放"));
            m_videoThread->wait();
        }
        delete m_videoThread;
        m_videoThread = nullptr;
    }

    m_videoRunning.store(false);
    m_videoWidth = 0;
    m_videoHeight = 0;

    LOG_INFO(LogCategory::Sensor, tr("视频流已停止"));
}

void LaserSensor::stopAll()
{
    stopContinuous();
    stopVideoStream();
}

// ============================================================================
// Video 帧处理
// ============================================================================

void LaserSensor::processVideoFrame(const QVector<unsigned char>& buffer)
{
    if (buffer.isEmpty() || m_videoWidth <= 0 || m_videoHeight <= 0) return;

    // 创建灰度图像
    QImage image(m_videoWidth, m_videoHeight, QImage::Format_Grayscale8);

    // 复制数据
    for (int y = 0; y < m_videoHeight; ++y) {
        uchar* line = image.scanLine(y);
        const int srcOffset = y * m_videoWidth;
        std::memcpy(line, buffer.data() + srcOffset, m_videoWidth);
    }

    emit videoFrameReceived(image);
}

// ============================================================================
// Profile 回调
// ============================================================================

void __stdcall LaserSensor::profileCallback(const unsigned char* data,
    unsigned int size,
    void* user)
{
    if (!user || !data || !size) return;

    LaserSensor* self = static_cast<LaserSensor*>(user);
    self->handleProfileCallback(data, size);
}

void LaserSensor::handleProfileCallback(const unsigned char* data, unsigned int size)
{
    const qint64 callbackPcUs = GlobalTimeSync::instance().timestampUs();
    const quint64 ordinal = m_rawProfileCallbackCount.fetch_add(1) + 1;
    m_lastProfileCallbackOrdinal.store(ordinal);
    m_lastProfileCallbackBytes.store(size);
    m_lastProfileResolution.store(m_resolution);

    bool dropped = false;
    QString dropReason;
    {
        // Defensive producer serialization in case a vendor DLL version invokes
        // callbacks re-entrantly. The critical section contains no allocation or SDK call.
        std::lock_guard<std::mutex> producerLock(m_profileProducerMutex);
        const quint64 writeSeq = m_profileIngressWriteSeq.load(std::memory_order_relaxed);
        const quint64 readSeq = m_profileIngressReadSeq.load(std::memory_order_acquire);
        const quint64 depth = writeSeq >= readSeq ? writeSeq - readSeq : 0;
        if (depth >= kProfileIngressSlotCount) {
            dropped = true;
            dropReason = tr("Profile raw ring溢出：depth=%1 capacity=%2 callbackOrdinal=%3")
                .arg(depth).arg(kProfileIngressSlotCount).arg(ordinal);
        }
        else {
            RawProfileSlot& slot = m_profileIngressSlots[writeSeq % kProfileIngressSlotCount];
            if (size > slot.bytes.size()) {
                dropped = true;
                dropReason = tr("Profile callback字节数超过预分配slot：bytes=%1 capacity=%2 callbackOrdinal=%3")
                    .arg(size).arg(slot.bytes.size()).arg(ordinal);
            }
            else {
                std::memcpy(slot.bytes.data(), data, size);
                slot.size = size;
                slot.callbackPcUs = callbackPcUs;
                slot.callbackOrdinal = ordinal;
                slot.ingressDepth = static_cast<quint32>(depth + 1);
                slot.callbackDonePcUs = GlobalTimeSync::instance().timestampUs();
                m_profileIngressWriteSeq.store(writeSeq + 1, std::memory_order_release);
                atomicMax(m_profileIngressHighWater, depth + 1);
                atomicMax(m_maxProfileCallbackServiceUs,
                    std::max<qint64>(0, slot.callbackDonePcUs - callbackPcUs));
            }
        }
    }

    if (dropped) {
        m_profileIngressDropCount.fetch_add(1);
        const qint64 doneUs = GlobalTimeSync::instance().timestampUs();
        atomicMax(m_maxProfileCallbackServiceUs, std::max<qint64>(0, doneUs - callbackPcUs));
        if (!m_suppressProfileEmission.load() && !m_profileIngressFaultEmitted.exchange(true))
            emit profileIngressFault(dropReason + tr("；正式session立即判无效，禁止静默覆盖"));
    }
}

// ============================================================================
// Peak Filter 命令序列实现（用于固件版本 < v43）
// 参考 SDK 示例: advanced/LLTPeakFilter
// ============================================================================

void LaserSensor::peakFilterWriteCommand(int command, int data)
{
    // 命令格式: (command << 9) + (toggle << 8) + data
    DWORD value = (command << 9) | (m_peakFilterToggle << 8) | (data & 0xFF);
    m_llt->SetFeature(FEATURE_FUNCTION_SHARPNESS, value);
    m_peakFilterToggle = !m_peakFilterToggle;
}

void LaserSensor::peakFilterWriteValue(unsigned short value)
{
    // 写入16位值，分两次写入（高8位和低8位）
    peakFilterWriteCommand(1, value / 256);   // 高8位
    peakFilterWriteCommand(1, value % 256);   // 低8位
}

void LaserSensor::peakFilterResetCommandList()
{
    // 重置命令列表
    peakFilterWriteCommand(0, 0);
    peakFilterWriteCommand(0, 0);
}

void LaserSensor::peakFilterEndCommandList()
{
    // 结束命令列表
    peakFilterWriteCommand(0, 0);
}

bool LaserSensor::setPeakFilterViaSharpness(unsigned short minWidth, unsigned short maxWidth,
    unsigned short minIntensity, unsigned short maxIntensity)
{
    if (!m_llt) {
        LOG_WARNING(LogCategory::Sensor, QString("setPeakFilterViaSharpness: m_llt is null!"));
        return false;
    }

    // 使用命令序列方式设置 Peak Filter（适用于固件版本 < v43）
    // 参考 SDK: LLTPeakFilter.cpp 中的 SetPeakValues 函数

    LOG_DEBUG(LogCategory::Sensor, QString("设置Peak Filter（命令序列方式）: 宽度 %1~%2, 强度 %3~%4")
        .arg(minWidth).arg(maxWidth).arg(minIntensity).arg(maxIntensity));

    m_peakFilterToggle = 0;  // 重置toggle状态

    // 按照SDK示例的顺序：maxWidth, minWidth, maxIntensity, minIntensity
    peakFilterResetCommandList();
    peakFilterWriteValue(maxWidth);
    peakFilterWriteValue(minWidth);
    peakFilterWriteValue(maxIntensity);
    peakFilterWriteValue(minIntensity);
    peakFilterEndCommandList();

    LOG_DEBUG(LogCategory::Sensor, QString("Peak Filter命令序列发送完成"));
    return true;
}

bool LaserSensor::disablePeakFilter()
{
    if (!m_llt) {
        LOG_WARNING(LogCategory::Sensor, QString("disablePeakFilter: m_llt is null!"));
        return false;
    }

    LOG_INFO(LogCategory::Sensor, tr("========== 尝试禁用 Peak Filter =========="));

    int ret = 0;
    DWORD currentWidth = 0, currentHeight = 0;

    // 首先读取当前 Peak Filter 值
    LOG_INFO(LogCategory::Sensor, tr("--- 读取当前 Peak Filter 值 ---"));

    ret = m_llt->GetFeature(FEATURE_FUNCTION_PEAKFILTER_WIDTH, &currentWidth);
    if (ret >= GENERAL_FUNCTION_OK) {
        unsigned short minW = currentWidth & 0xFFFF;
        unsigned short maxW = (currentWidth >> 16) & 0xFFFF;
        LOG_INFO(LogCategory::Sensor, QString("PEAKFILTER_WIDTH (0xf0b02000): 0x%1 (min=%2, max=%3)")
            .arg(currentWidth, 8, 16, QChar('0')).arg(minW).arg(maxW));
    }
    else {
        LOG_WARNING(LogCategory::Sensor, QString("读取 PEAKFILTER_WIDTH 失败，错误码: %1 (可能固件版本 < v43)").arg(ret));
    }

    ret = m_llt->GetFeature(FEATURE_FUNCTION_PEAKFILTER_HEIGHT, &currentHeight);
    if (ret >= GENERAL_FUNCTION_OK) {
        unsigned short minH = currentHeight & 0xFFFF;
        unsigned short maxH = (currentHeight >> 16) & 0xFFFF;
        LOG_INFO(LogCategory::Sensor, QString("PEAKFILTER_HEIGHT (0xf0b02004): 0x%1 (min=%2, max=%3)")
            .arg(currentHeight, 8, 16, QChar('0')).arg(minH).arg(maxH));
    }
    else {
        LOG_WARNING(LogCategory::Sensor, QString("读取 PEAKFILTER_HEIGHT 失败，错误码: %1 (可能固件版本 < v43)").arg(ret));
    }

    DWORD sharpnessValue = 0;
    ret = m_llt->GetFeature(FEATURE_FUNCTION_SHARPNESS, &sharpnessValue);
    LOG_INFO(LogCategory::Sensor, QString("当前 SHARPNESS: 0x%1").arg(sharpnessValue, 8, 16, QChar('0')));

    // ========== 方法1: 直接写入 PEAKFILTER 寄存器 (固件 v43+) ==========
    LOG_INFO(LogCategory::Sensor, tr("--- 方法1: 直接写入 PEAKFILTER 寄存器 (固件 v43+) ---"));

    // 设置为最大范围: min=0, max=1023
    // 格式假设: 低16位=min, 高16位=max
    DWORD widthValue = (1023 << 16) | 0;    // max=1023, min=0
    DWORD heightValue = (1023 << 16) | 0;   // max=1023, min=0

    ret = m_llt->SetFeature(FEATURE_FUNCTION_PEAKFILTER_WIDTH, widthValue);
    LOG_INFO(LogCategory::Sensor, QString("设置 PEAKFILTER_WIDTH = 0x%1, 返回值: %2")
        .arg(widthValue, 8, 16, QChar('0')).arg(ret));

    ret = m_llt->SetFeature(FEATURE_FUNCTION_PEAKFILTER_HEIGHT, heightValue);
    LOG_INFO(LogCategory::Sensor, QString("设置 PEAKFILTER_HEIGHT = 0x%1, 返回值: %2")
        .arg(heightValue, 8, 16, QChar('0')).arg(ret));

    // ========== 方法2: 多次写入0到 SHARPNESS 清除命令缓冲 ==========
    LOG_INFO(LogCategory::Sensor, tr("--- 方法2: 多次写入0到 SHARPNESS 清除命令缓冲 ---"));

    for (int i = 0; i < 10; i++) {
        ret = m_llt->SetFeature(FEATURE_FUNCTION_SHARPNESS, 0);
    }
    LOG_INFO(LogCategory::Sensor, QString("SHARPNESS = 0 (写入10次), 最后返回值: %1").arg(ret));

    // ========== 方法3: 发送 reset 命令序列 ==========
    LOG_INFO(LogCategory::Sensor, tr("--- 方法3: 发送多次 reset 命令序列 ---"));

    m_peakFilterToggle = 0;
    // 发送多次 reset (command=0, data=0)
    for (int i = 0; i < 5; i++) {
        peakFilterWriteCommand(0, 0);  // reset command
    }
    LOG_INFO(LogCategory::Sensor, tr("发送5次 reset 命令"));

    // ========== 方法4: 使用 SHARPNESS 命令序列设置全范围 ==========
    LOG_INFO(LogCategory::Sensor, tr("--- 方法4: 使用 SHARPNESS 命令序列设置全范围 (0~1023) ---"));

    m_peakFilterToggle = 0;
    peakFilterResetCommandList();
    peakFilterWriteValue(1023);  // maxWidth = 1023
    peakFilterWriteValue(0);     // minWidth = 0
    peakFilterWriteValue(1023);  // maxIntensity = 1023
    peakFilterWriteValue(0);     // minIntensity = 0
    peakFilterEndCommandList();
    LOG_INFO(LogCategory::Sensor, tr("SHARPNESS 命令序列已发送: width(0~1023), intensity(0~1023)"));

    // ========== 方法5: 写入0到 EXTRA_PARAMETER ==========
    LOG_INFO(LogCategory::Sensor, tr("--- 方法5: 写入0到 EXTRA_PARAMETER ---"));

    ret = m_llt->SetFeature(FEATURE_FUNCTION_EXTRA_PARAMETER, 0);
    LOG_INFO(LogCategory::Sensor, QString("EXTRA_PARAMETER = 0, 返回值: %1").arg(ret));

    // ========== 最终验证 ==========
    LOG_INFO(LogCategory::Sensor, tr("--- 最终验证 ---"));

    ret = m_llt->GetFeature(FEATURE_FUNCTION_SHARPNESS, &sharpnessValue);
    LOG_INFO(LogCategory::Sensor, QString("最终 SHARPNESS: 0x%1").arg(sharpnessValue, 8, 16, QChar('0')));

    ret = m_llt->GetFeature(FEATURE_FUNCTION_PEAKFILTER_WIDTH, &currentWidth);
    if (ret >= GENERAL_FUNCTION_OK) {
        unsigned short minW = currentWidth & 0xFFFF;
        unsigned short maxW = (currentWidth >> 16) & 0xFFFF;
        LOG_INFO(LogCategory::Sensor, QString("最终 PEAKFILTER_WIDTH: 0x%1 (min=%2, max=%3)")
            .arg(currentWidth, 8, 16, QChar('0')).arg(minW).arg(maxW));
    }

    ret = m_llt->GetFeature(FEATURE_FUNCTION_PEAKFILTER_HEIGHT, &currentHeight);
    if (ret >= GENERAL_FUNCTION_OK) {
        unsigned short minH = currentHeight & 0xFFFF;
        unsigned short maxH = (currentHeight >> 16) & 0xFFFF;
        LOG_INFO(LogCategory::Sensor, QString("最终 PEAKFILTER_HEIGHT: 0x%1 (min=%2, max=%3)")
            .arg(currentHeight, 8, 16, QChar('0')).arg(minH).arg(maxH));
    }

    LOG_INFO(LogCategory::Sensor, tr("========== Peak Filter 禁用尝试完成 =========="));
    LOG_INFO(LogCategory::Sensor, tr("请重新配置硬件触发并测试采样频率"));

    return true;
}
