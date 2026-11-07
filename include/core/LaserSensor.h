#pragma once

#include <QObject>
#include <QMutex>
#include <QVector>
#include <QString>
#include <QImage>
#include <QThread>
#include <atomic>
#include <array>
#include <vector>
#include <mutex>

#include "core/DataTypes.h"

// Micro-Epsilon C++ 接口头文件
#include "InterfaceLLT_2.h"

// 前向声明
class VideoStreamThread;
class ProfileDecodeThread;

/**
 * @brief 线激光传感器封装类
 *
 * 封装 Micro-Epsilon scanCONTROL 系列传感器的连接、配置和数据采集功能。
 * 支持两种工作模式：
 *   - Profile 模式：获取轮廓数据 (X, Z 坐标)
 *   - Video 模式：获取原始 CMOS 图像
 *
 * 线程安全：回调在 DLL 线程中执行，通过 Qt 信号队列传递到主线程。
 */
class LaserSensor : public QObject
{
    Q_OBJECT

public:
    explicit LaserSensor(QObject* parent = nullptr);
    ~LaserSensor() override;

    // ==================== 连接管理 ====================

    /**
     * @brief 连接传感器
     * @param dllPath LLT.dll 的路径
     * @param errMsg 错误信息输出
     * @return 成功返回 true
     */
    bool connectSensor(const QString& dllPath, QString* errMsg = nullptr);

    /**
     * @brief 断开传感器连接
     */
    void disconnectSensor();

    /**
     * @brief 检查是否已连接
     */
    bool isConnected() const { return m_connected.load(); }

    // ==================== Profile 模式配置 ====================

    /**
     * @brief 配置 Profile 模式
     * @param mode 触发模式（软件/硬件）
     * @param params 传感器参数
     * @param highLevel24V 硬件模式时是否使用 24V 电平
     * @param triggerType 硬件触发类型（仅硬件模式有效）
     * @param errMsg 错误信息输出
     */
    bool configure(SensorTriggerMode mode,
        const SensorParams& params,
        bool highLevel24V,
        HardwareTriggerType triggerType = HardwareTriggerType::EdgeRising,
        QString* errMsg = nullptr);

    /**
     * @brief 应用高级参数设置
     * @param advParams 高级参数结构
     * @param errMsg 错误信息输出
     * @return 成功返回 true
     */
    bool applyAdvancedParams(const SensorAdvancedParams& advParams, QString* errMsg = nullptr);

    /**
     * @brief 从传感器读取高级参数
     * @param advParams 输出参数结构
     * @param errMsg 错误信息输出
     * @return 成功返回 true
     */
    bool readAdvancedParams(SensorAdvancedParams& advParams, QString* errMsg = nullptr);

    /**
     * @brief 恢复传感器默认设置（修复硬件触发问题）
     * @param errMsg 错误信息输出
     * @return 成功返回 true
     */
    bool restoreDefaultSettings(QString* errMsg = nullptr);

    /**
     * @brief 设置Measuring Field预设索引（MEASURINGFIELD / ROI1_PRESET）
     * 项目Quick Reference参考值:
     *   0 = Large/全测量场预设 (ref 144Hz), 2 = Standard (ref 192Hz), 7 = Small (ref 552Hz)
     * 这些频率用于保守约束/提示，不等同于设备实时回读的最大Profile频率。
     * @param index 测量区域索引 (0-127)
     * @param saveToUserMode 是否保存到用户模式1
     * @param errMsg 错误信息输出
     * @return 成功返回 true
     */
    bool setMeasuringFieldIndex(int index, bool saveToUserMode = true, QString* errMsg = nullptr);

    /**
     * @brief 获取指定索引对应的最大采样频率
     * @param index 测量区域索引
     * @return 最大频率(Hz)，无效索引返回-1
     */
    static int getMaxFrequencyForIndex(int index);

    /**
     * @brief 回读当前运行时的曝光、Idle与Measuring Field。
     * 用于传感标定启动前后的真实设备约束校验；不读取2950上无法可靠回读的有效反射参数。
     */
    bool readTimingAndMeasuringField(unsigned int& exposureUnits,
        unsigned int& idleUnits,
        int& measuringFieldIndex,
        QString* errMsg = nullptr);

    /**
     * @brief 禁用 Peak Filter（尝试恢复硬件触发功能）
     * @return 成功返回 true
     */
    bool disablePeakFilter();

    /**
     * @brief 读取并显示传感器诊断信息
     */
    void printDiagnostics();

    // ==================== Video 模式配置 ====================

    /**
     * @brief 配置 Video 模式
     * @param params 传感器参数（曝光、idle 时间）
     * @param errMsg 错误信息输出
     */
    bool configureVideo(const SensorParams& params, QString* errMsg = nullptr);

    // ==================== 采集控制 ====================

    /**
     * @brief 启动连续轮廓传输（Profile 模式）
     */
     // CMM RS422 output (pin 12+/11-) and output gating. CMM configuration is
     // volatile on 29xx; configure after every connect/trigger-mode switch.
    bool configureCmmOutput(QString* errMsg = nullptr);
    // SDK-owned transfer + CMM gating: H3 only, after five IN1/R1 FIFOs are
    // armed. Do not also startContinuous().
    bool startCmmGated(int nominalHz, QString* errMsg = nullptr);
    bool stopCmmGated(QString* errMsg = nullptr); // H3: needs internally generated CMM-inactive frame
    bool startContinuous(QString* errMsg = nullptr);

    /**
     * @brief 停止连续轮廓传输（Profile 模式）
     */
    void stopContinuous();

    /**
     * @brief 启动 Video 流
     * @param errMsg 错误信息输出
     * @return 成功返回 true
     */
    bool startVideoStream(QString* errMsg = nullptr);

    /**
     * @brief 停止 Video 流
     */
    void stopVideoStream();

    /**
     * @brief 停止所有传输（Profile 和 Video）
     */
    void stopAll();

    // ==================== 状态查询 ====================

    SensorWorkMode workMode() const { return m_workMode; }
    SensorTriggerMode triggerMode() const { return m_triggerMode; }
    const SensorParams& params() const { return m_params; }
    const QVector<unsigned int>& availableResolutions() const { return m_resolutions; }
    unsigned int activeResolution() const { return m_resolution; }

    /**
     * @brief 获取 Video 图像尺寸
     */
    int videoWidth() const { return m_videoWidth; }
    int videoHeight() const { return m_videoHeight; }

    /**
     * @brief 检查是否正在运行
     */
    bool isRunning() const { return m_profileRunning || m_videoRunning; }
    bool isProfileRunning() const { return m_profileRunning; }
    bool isVideoRunning() const { return m_videoRunning; }

    /**
     * @brief 获取传感器类型名称
     */
    QString sensorTypeName() const;

    struct ProfileCallbackDiagnostics {
        quint64 rawCallbacks = 0;
        quint64 convertedProfiles = 0;
        quint64 conversionFailures = 0;
        quint64 ingressDrops = 0;
        quint64 rawRingDepth = 0;
        quint64 rawRingHighWater = 0;
        quint64 rawRingCapacity = 0;
        quint64 lastCallbackOrdinal = 0;
        qint64 maxCallbackServiceUs = 0;
        qint64 maxDecoderQueueUs = 0;
        qint64 maxDecodeServiceUs = 0;
        int lastConvertResult = 0;
        unsigned int lastCallbackBytes = 0;
        unsigned int lastResolution = 0;
    };

    struct HardwareTriggerDiagnostics {
        bool readbackValid = false; // FEATURE_FUNCTION_TRIGGER was read successfully and verified
        bool cmmPortReadbackValid = false; // H3 only: DIGITAL_IO lower-nibble CMM port verified
        unsigned int triggerRequested = 0;
        unsigned int triggerReadback = 0;
        unsigned int digitalIoRequested = 0;
        unsigned int digitalIoReadback = 0;
    };

    ProfileCallbackDiagnostics profileCallbackDiagnostics() const;
    HardwareTriggerDiagnostics hardwareTriggerDiagnostics() const;
    void resetProfileCallbackDiagnostics();
    void setProfileEmissionSuppressed(bool suppressed) { m_suppressProfileEmission.store(suppressed); }

    // Latest decoded profile snapshot for low-rate GUI/calibration preview.
    // This is an in-memory copy only; it does not touch the LLT SDK or controller.
    bool latestProfile(LaserProfile& profile) const;

    // V27.6.16Q sensor-calibration-only rich profile snapshot.
    // IMPORTANT: this path is disabled by default and is never enabled by the
    // hardware-sync acquisition strategy.  When enabled, DecodeThread asks the
    // LLT SDK for the auxiliary values exposed by ConvertProfile2Values().
    struct SensorCalibrationProfileFrame {
        LaserProfile profile;
        QVector<unsigned short> width;       // SDK pWidth
        QVector<unsigned short> maximum;     // SDK pMaximum (peak maximum / intensity-like)
        QVector<unsigned short> threshold;   // SDK pThreshold
        QVector<unsigned int> moment0;       // SDK pM0
        QVector<unsigned int> moment1;       // SDK pM1
        int convertFlags = 0;

        bool hasWidth() const { return (convertFlags & CONVERT_WIDTH) != 0; }
        bool hasMaximum() const { return (convertFlags & CONVERT_MAXIMUM) != 0; }
        bool hasThreshold() const { return (convertFlags & CONVERT_THRESHOLD) != 0; }
        bool hasMoment0() const { return (convertFlags & CONVERT_M0) != 0; }
        bool hasMoment1() const { return (convertFlags & CONVERT_M1) != 0; }
    };

    void setSensorCalibrationRichProfileEnabled(bool enabled);
    bool latestSensorCalibrationProfile(SensorCalibrationProfileFrame& frame) const;

    // Read-only identity query used once when a sensor-calibration experiment starts.
    // Failure of name/serial sub-queries is reported as empty fields rather than
    // changing the acquisition path.
    bool readDeviceIdentity(QString& deviceName, QString& serialNumber, QString* errMsg = nullptr);

signals:
    /**
     * @brief 收到新的轮廓数据时发出（Profile 模式）
     */
    void profileReceived(const LaserProfile& profile);

    // Fail-closed host ingress fault. Emitted only for formal-session conditions
    // such as a raw-ring overflow or conversion failure while emission is enabled.
    void profileIngressFault(const QString& reason);

    /**
     * @brief 收到新的视频帧时发出（Video 模式）
     */
    void videoFrameReceived(const QImage& frame);

private:
    friend class VideoStreamThread;
    friend class ProfileDecodeThread;

    // DLL 回调函数（静态）
    static void __stdcall profileCallback(const unsigned char* pucData,
        unsigned int uiSize,
        void* pUserData);

    // 实例回调处理：V27.6.16M仅做时间戳+预分配ring memcpy，禁止SDK转换/GUI工作。
    void handleProfileCallback(const unsigned char* pucData, unsigned int uiSize);
    bool decodeOneQueuedProfile();
    void prepareProfileIngress(unsigned int resolution);
    void stopProfileDecoder();

    // 配置实现
    bool configureSoftwareInternal(const SensorParams& params, QString* errMsg);
    bool configureHardwareExternal(const SensorParams& params,
        bool highLevel24V,
        HardwareTriggerType triggerType,
        QString* errMsg);

    // 清理内部状态
    void clearInternal();

    // Video 帧处理（由 VideoStreamThread 调用）
    void processVideoFrame(const QVector<unsigned char>& buffer);

private:
    CInterfaceLLT* m_llt = nullptr;
    std::atomic_bool m_connected{ false };

    QVector<unsigned int> m_resolutions;
    unsigned int m_resolution = 0;
    TScannerType m_scannerType = scanCONTROL29xx_50;

    // 当前工作模式
    SensorWorkMode m_workMode = SensorWorkMode::Profile;
    SensorTriggerMode m_triggerMode = SensorTriggerMode::SoftwareInternal;
    SensorParams m_params;

    // 运行状态
    std::atomic<bool> m_profileRunning{ false };
    std::atomic_bool m_cmmConfigured{ false };
    std::atomic_bool m_cmmGated{ false };
    std::atomic<bool> m_videoRunning{ false };

    // V32.11 hardware-trigger diagnostics. These counters distinguish
    // "no physical callback" from "callback arrived but SDK conversion failed".
    std::atomic<quint64> m_rawProfileCallbackCount{ 0 };
    std::atomic<quint64> m_convertedProfileCount{ 0 };
    std::atomic<quint64> m_profileConvertFailureCount{ 0 };
    std::atomic<int> m_lastProfileConvertResult{ 0 };
    std::atomic<unsigned int> m_lastProfileCallbackBytes{ 0 };
    std::atomic<unsigned int> m_lastProfileResolution{ 0 };
    std::atomic<bool> m_suppressProfileEmission{ false }; // V32.11 cold-prime frames are diagnostic-only

    // V27.6.16M: callback/decoder decoupling. LLT callback is the single producer;
    // ProfileDecodeThread is the single consumer. Slots are preallocated before
    // TransferProfiles(true), so the callback does not allocate memory.
    static constexpr std::size_t kProfileIngressSlotCount = 128;
    struct RawProfileSlot {
        std::vector<unsigned char> bytes;
        unsigned int size = 0;
        qint64 callbackPcUs = 0;
        qint64 callbackDonePcUs = 0;
        quint64 callbackOrdinal = 0;
        quint32 ingressDepth = 0;
    };
    std::array<RawProfileSlot, kProfileIngressSlotCount> m_profileIngressSlots;
    std::atomic<quint64> m_profileIngressWriteSeq{ 0 };
    std::atomic<quint64> m_profileIngressReadSeq{ 0 };
    std::mutex m_profileProducerMutex;
    ProfileDecodeThread* m_profileDecodeThread = nullptr;

    std::atomic<quint64> m_profileIngressDropCount{ 0 };
    std::atomic<quint64> m_profileIngressHighWater{ 0 };
    std::atomic<qint64> m_maxProfileCallbackServiceUs{ 0 };
    std::atomic<qint64> m_maxProfileDecoderQueueUs{ 0 };
    std::atomic<qint64> m_maxProfileDecodeServiceUs{ 0 };
    std::atomic<quint64> m_lastProfileCallbackOrdinal{ 0 };
    std::atomic_bool m_profileIngressFaultEmitted{ false };

    mutable QMutex m_latestProfileMutex;
    LaserProfile m_latestDecodedProfile;
    bool m_hasLatestDecodedProfile = false;

    // Sensor-calibration rich decode is opt-in. Keeping the default false avoids
    // any extra point arrays or ConvertProfile2Values outputs in hardware-sync.
    std::atomic<bool> m_sensorCalibrationRichProfileEnabled{ false };
    mutable QMutex m_sensorCalibrationProfileMutex;
    SensorCalibrationProfileFrame m_latestSensorCalibrationProfile;
    bool m_hasLatestSensorCalibrationProfile = false;

    // Cached immediately after SetFeature/GetFeature verification. No LLT I/O is
    // performed from the Probe timeout path.
    std::atomic<bool> m_hwTriggerReadbackValid{ false };
    std::atomic<bool> m_cmmPortReadbackValid{ false };
    std::atomic<unsigned int> m_hwTriggerRequested{ 0 };
    std::atomic<unsigned int> m_hwTriggerReadback{ 0 };
    std::atomic<unsigned int> m_hwDigitalIoRequested{ 0 };
    std::atomic<unsigned int> m_hwDigitalIoReadback{ 0 };

    // Video 模式相关
    int m_videoWidth = 0;
    int m_videoHeight = 0;
    VideoStreamThread* m_videoThread = nullptr;

    // Peak Filter 命令序列相关（用于固件版本 < v43）
    int m_peakFilterToggle = 0;
    void peakFilterWriteCommand(int command, int data);
    void peakFilterWriteValue(unsigned short value);
    void peakFilterResetCommandList();
    void peakFilterEndCommandList();
    bool setPeakFilterViaSharpness(unsigned short minWidth, unsigned short maxWidth,
        unsigned short minIntensity, unsigned short maxIntensity);

    mutable QMutex m_mutex;
};

/**
 * @brief Video 流读取线程
 *
 * 在独立线程中轮询获取 Video 帧，避免阻塞主线程
 */
class VideoStreamThread : public QThread
{
    Q_OBJECT

public:
    VideoStreamThread(LaserSensor* sensor, CInterfaceLLT* llt,
        int width, int height, QObject* parent = nullptr);
    ~VideoStreamThread() override;

    void requestStop() { m_stop.store(true); }

protected:
    void run() override;

private:
    LaserSensor* m_sensor;
    CInterfaceLLT* m_llt;
    int m_width;
    int m_height;
    std::atomic<bool> m_stop{ false };
};
