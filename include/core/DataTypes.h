#pragma once

#include <QVector>
#include <QVector3D>
#include <QMatrix4x4>
#include <QString>
#include <QMetaType>
#include <cstdint>
#include <cmath>
#include <memory>
#include "runtime/Acquisition5D.h"

// ==================== 轴定义 ====================
enum AxisId {
    AxisX = 1,
    AxisY = 2,
    AxisZ = 3,
    AxisC = 4,
    AxisB = 5
};

// ==================== 五轴统一数据结构（P1） ====================
/**
 * @brief 五轴关节坐标，统一采用上位机/控制器机械坐标约定。
 * X/Y/Z: mm；C/B: deg。
 *
 * 这是后续运动学、五轴轨迹、同步采样、GCode 扩展共同使用的基础类型，
 * 避免各模块继续用散乱的 x/y/z/c/b 参数。
 */
struct JointPose5D {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double c = 0.0;
    double b = 0.0;

    bool isFinite() const {
        return std::isfinite(x) && std::isfinite(y) && std::isfinite(z)
            && std::isfinite(c) && std::isfinite(b);
    }
};

/**
 * @brief 五轴正运动学输出。
 * 当前末端参考坐标系原点定义在 B/C 两旋转轴线交点，暂不包含传感器外参。
 */
struct EndPose5D {
    JointPose5D joints;          // 产生该位姿的控制器五轴坐标
    QVector3D position;          // 末端参考点在机床坐标系的位置 (mm)
    QMatrix4x4 transform;        // ^M T_E 齐次变换矩阵
    double mathCdeg = 0.0;       // 数学右手定义的 C 角（当前 = -controller C）
    double mathBdeg = 0.0;       // 数学右手定义的 B 角（当前 = -controller B）
};

/**
 * @brief 五轴解析逆运动学结果。
 */
struct InverseKinematics5DResult {
    bool success = false;
    JointPose5D joints;
    double rotationResidual = 0.0;
    double positionResidual = 0.0;
    QString error;
};

// ==================== 五轴关节运动/轨迹数据结构（P4） ====================
/**
 * @brief 同一时刻五轴的需求位置(DPOS)与反馈位置(MPOS)。
 *
 * 对外全部采用机床机械坐标：XYZ 为 mm，C/B 为 deg。
 * B/C 的 software-frame 偏移由 ZmcController 内部处理，高层不接触控制器原始坐标。
 */
struct JointState5D {
    JointPose5D dpos;
    JointPose5D mpos;
    bool allIdle = false;
};

/**
 * @brief 五轴轨迹中的一个关节点。
 * timeFromStartSec 是从整条轨迹起点累计的时间。
 * P4 先用于点到点同步定位；P9 的 TCP 轨迹会复用同一结构。
 */
struct JointTrajectoryPoint5D {
    JointPose5D position;
    double timeFromStartSec = 0.0;
    bool scanEnabled = false;
};

/**
 * @brief 五轴关节轨迹容器。
 */
struct JointTrajectory5D {
    QVector<JointTrajectoryPoint5D> points;

    bool isEmpty() const { return points.isEmpty(); }
    int size() const { return points.size(); }
};

/**
 * @brief P4 点到点五轴同步定位规划结果。
 *
 * controllerMetricLength / vectorSpeed 等量是 ZMotion 多轴直线插补使用的
 * 数值关节空间度量（XYZ:mm, C/B:deg 的数值组合），不是 TCP 空间长度/速度。
 */
struct JointMotionPlan5D {
    bool valid = false;
    bool hasMotion = false;

    JointPose5D start;
    JointPose5D target;
    JointTrajectory5D trajectory;

    int baseAxis = AxisX;
    double speedScale = 0.20;
    double controllerMetricLength = 0.0;
    double vectorSpeed = 0.0;
    double vectorAccel = 0.0;
    double vectorDecel = 0.0;
    double srampMs = 0.0;
    double estimatedDurationSec = 0.0;

    QString error;
};

/**
 * @brief 五轴同步定位结束后的低频验证结果。
 */
struct JointMoveResult5D {
    bool completed = false;
    JointState5D finalState;

    double maxLinearDposErrorMm = 0.0;
    double maxLinearMposErrorMm = 0.0;
    double maxRotaryMposErrorDeg = 0.0;

    QString message;
};

// ==================== 轴参数 ====================
struct AxisParams {
    float units = 4000.0f;       // UNITS 脉冲当量
    float lspeed = 0.50f;        // LSPEED 起始速度
    float speed = 5.0f;          // SPEED 运行速度
    float accel = 25.0f;         // ACCEL 加速度
    float decel = 25.0f;         // DECEL 减速度
    float sramp = 50.0f;         // SRAMP S曲线时间
    float creep = 100.0f;        // CREEP 二次回零爬行速度
    float softLimitPos = 999999.0f;   // FS_LIMIT 正软限位（默认大值，不限制）
    float softLimitNeg = -999999.0f;  // RS_LIMIT 负软限位（默认大负值，不限制）
};

// ==================== 回零参数 ====================
struct HomeParams {
    int mode = 13;               // 回零模式：默认 DATUM(13)
    int datumIn = 0;             // 原点输入口号
    bool invertIn = true;        // 是否反向原点输入
};

// ==================== 轨迹点 ====================
struct PathPoint {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double feed = 0.0;           // 进给速度
    double c = 0.0;              // C mechanical degrees
    double b = 0.0;              // B mechanical degrees
    std::uint8_t axisMask = 7;  // Legacy XYZ aggregate holds C/B; full 5D writers explicitly set 31
    mls::runtime::FeedMode feedMode = mls::runtime::FeedMode::LegacyPerSecond;
    bool continuous = false;       // dense RTCP/polyline junction may keep non-zero SP speed

    JointPose5D jointPose() const {
        JointPose5D q;
        q.x = x; q.y = y; q.z = z; q.c = c; q.b = b;
        return q;
    }
};

// ==================== 位置采样点 ====================
struct SamplePoint {
    qint64 tUs = 0;              // 时间戳（微秒）
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double c = 0.0;              // C轴机械角（deg）；P5 同步扩展后由控制器 TABLE 填充
    double b = 0.0;              // B轴机械角（deg）；P5 同步扩展后由控制器 TABLE 填充
    bool hasRotary = false;      // Missing C/B is invalid for five-axis reconstruction.
    mls::runtime::Frame5D frame; // acquisition provenance; transform filled after calibration

    JointPose5D jointPose() const {
        JointPose5D q;
        q.x = x; q.y = y; q.z = z; q.c = c; q.b = b;
        return q;
    }
};

// ==================== 激光轮廓数据 ====================
struct LaserProfile {
    QVector<double> x;           // 传感器 X 坐标 (mm)
    QVector<double> z;           // 传感器 Z 坐标 (mm)
    double tOpen = 0.0;          // 光窗打开时间（秒）
    double tClose = 0.0;         // 光窗关闭时间（秒）
    qint64 tPcUs = 0;            // LLT原始callback进入时的PC全局时间戳（µs）
    quint32 index = 0;           // profile 计数/硬件索引
    bool cmmValid = false;       // optional 29xx CMM timestamp parsed successfully
    bool cmmActive = false;
    bool cmmTrigger = false;
    quint32 cmmCount = 0;

    // V27.6.16M ingress forensic metadata. These fields are diagnostic only and
    // are not used as pose/exposure timing truth. They allow a missing sensor
    // index to be localized to before callback, raw-ingress buffering, SDK
    // conversion, or later host processing.
    quint64 callbackOrdinal = 0;
    quint32 rawBytes = 0;
    quint32 ingressDepth = 0;
    qint64 callbackServiceUs = 0;
    qint64 decoderQueueUs = 0;
    qint64 decodeServiceUs = 0;
    qint64 decodeDonePcUs = 0;
};

// ==================== 对齐后的点 ====================
struct AlignedPoint {
    qint64 tUs = 0;              // 时间戳（µs）- 来自位置数据
    double tSensorSec = 0.0;     // 传感器硬件时间戳（秒）- 来自 tOpen
    quint32 profileIndex = 0;    // 轮廓序号
    quint32 pointIndex = 0;      // 轮廓中的点索引

    double sensorX = 0.0;        // 传感器坐标系 X
    double sensorZ = 0.0;        // 传感器坐标系 Z

    double machineX = 0.0;       // 机床坐标系 X
    double machineY = 0.0;       // 机床坐标系 Y
    double machineZ = 0.0;       // 机床坐标系 Z
    std::shared_ptr<const mls::runtime::Frame5D> frame; // immutable, shared once per profile
};

// ==================== 同步模式 ====================
enum class SyncMode {
    Software,                    // 软件同步（PC 时间戳插值）
    Hardware                     // 硬件同步（TABLE + 数字触发）
};

// ==================== 传感器参数 ====================
struct SensorParams {
    unsigned int resolutionIndex = 0;   // 分辨率索引
    unsigned int exposureTime = 100;    // 曝光时间（10µs 单位）
    unsigned int idleTime = 300;        // Idle 时间（10µs 单位）= 3ms
    int measuringFieldIndex = 7;        // 正式采集 Measuring Field 预设；硬件触发频率上限由该预设约束
};

// ==================== 传感器高级参数 ====================
/**
 * @brief 传感器高级参数设置
 * 
 * 与 Micro-Epsilon scanCONTROL Configuration Tools 参数对应
 */
struct SensorAdvancedParams {
    // ===== 测量区域 =====
    int measuringFieldIndex = 2;         // Measuring Field预设索引 (0-127)
                                         // 常用: 0=Large/全测量场预设, 2=Standard, 7=Small
                                         // 频率映射为项目Quick Reference参考值，不是设备实时Max Frequency回读
    
    // ===== 轮廓设置 =====
    unsigned int resolution = 1280;         // 每个轮廓的点数
    bool flipX = false;                     // 沿 X 轴方向反转
    bool flipZ = true;                      // 沿 Z 轴方向反转（官方默认启用）
    
    // ===== 传感器设置 =====
    int laserPower = 2;                     // 激光功率: 0=关闭, 1=降低, 2=标准
    double thresholdPercent = 12.5;         // 阈值百分比 (0-100)
    int reflectionSelect = 3;               // 反射选择: 0=全部, 1=第一, 2=最后, 3=最大面积, 4=最大强度
    
    // ===== 自动曝光 =====
    bool autoExposure = false;              // 自动曝光开关（默认关闭）
    int shutterAlign = 1;                   // 快门对齐: 0=中, 1=右, 2=左, 3=关闭
    int autoExpMode = 1;                    // 模式: 0=原始数据, 1=轮廓, 2=过滤的轮廓
    
    // ===== 有效反射（仅支持写入） =====
    int peakWidthMin = 2;                   // 最小宽度 (值<2会增加噪声)
    int peakWidthMax = 256;                 // 最大宽度 (官方默认值)
    int peakIntensityMin = 0;               // 最小强度
    int peakIntensityMax = 1023;            // 最大强度 (SDK最大值1023)
    
    // ===== 处理选项 =====
    bool ambientLightSuppression = true;    // 抑制背光
    bool videoFilter = true;                // 视频过滤器
    bool highResolution = true;             // 高分辨率
    bool calibration = true;                // 校准
};

// ==================== 传感器触发模式 ====================
enum class SensorTriggerMode {
    SoftwareInternal,            // 内部触发
    HardwareExternal             // 外部数字触发
};

// ==================== 传感器工作模式 ====================
enum class SensorWorkMode {
    Profile,                     // Profile 模式 - 获取轮廓数据 (X, Z 坐标)
    Video                        // Video 模式 - 获取原始 CMOS 图像
};

// ==================== 硬件触发类型 ====================
enum class HardwareTriggerType {
    EdgeRising,     // 上升沿触发；硬件流程必须显式选择，不能依赖默认值
    EdgeFalling,    // 下降沿触发
    PulseHigh,      // 高电平脉冲触发
    PulseLow        // 低电平脉冲触发
};

// ==================== 坐标变换参数 ====================
struct MountOffset {
    double x = 0.0;              // 传感器安装 X 偏移
    double y = 0.0;              // 传感器安装 Y 偏移
    double z = 0.0;              // 传感器安装 Z 偏移
};

// ==================== 常量定义 ====================
namespace Constants {
    // =============== Micro-Epsilon 2950-50BL 传感器量程 ===============
    // Z 轴（高度方向）：测量范围 70 ~ 120 mm
    constexpr double kSensorZMin = 70.0;   // Z 最小有效值 (mm)
    constexpr double kSensorZMax = 120.0;  // Z 最大有效值 (mm)
    constexpr double kSensorZRange = 50.0; // Z 测量范围 (mm)
    
    // X 轴（宽度方向）：scanCONTROL 29xx-50 标准测量场是梯形，而不是恒定 ±25 mm 的矩形。
    // 物理含义必须按“离传感器的光学深度”理解：近端小口 42 mm，中间 50 mm，
    // 远端大口 58 mm。项目默认开启 FlipZ，因此处理后的 Sensor-Z 数值顺序与物理
    // 近/远顺序相反；仿真代码不得把“小口/大口”直接绑定到 ZMin/ZMax。
    // kSensorXMin/Max 继续表示中量程处的 ±25 mm，供旧显示/接口兼容。
    constexpr double kSensorXMin = -25.0;  // 中量程 X 最小值 (mm)
    constexpr double kSensorXMax = 25.0;   // 中量程 X 最大值 (mm)
    constexpr double kSensorXRange = 50.0; // 中量程宽度 (mm)
    constexpr double kSensorXSpanNear = 42.0; // 物理近端/靠近传感器的小口 (mm)
    constexpr double kSensorXSpanMid  = 50.0; // 物理中间深度 (mm)
    constexpr double kSensorXSpanFar  = 58.0; // 物理远端/远离传感器的大口 (mm)

    // 旧名称仅保留源码兼容。当前项目默认 FlipZ=true，因此若旧代码仍按
    // “数值 ZMin/ZMax”取宽度，默认约定下应是 ZMin=物理远端(58 mm)、
    // ZMax=物理近端(42 mm)。新代码禁止依赖这些旧名称，必须使用 Near/Far
    // 并通过保存的 FlipZ 显式完成“物理深度 <-> reported Sensor-Z”映射。
    constexpr double kSensorXSpanAtZMin = kSensorXSpanFar;   // only for default FlipZ=true compatibility
    constexpr double kSensorXSpanAtZMid = kSensorXSpanMid;
    constexpr double kSensorXSpanAtZMax = kSensorXSpanNear;  // only for default FlipZ=true compatibility
    
    // =============== 定时器间隔 ===============
    constexpr int kStatusTimerIntervalMs = 100;
    constexpr int kPosTimerIntervalMs = 50;
    constexpr int kProfileUpdateIntervalMs = 50;
    constexpr int kHwTableReadIntervalMs = 10;

    // 软件同步位置环形缓冲区（仅用于插值，不需要无限存储）
    constexpr int kSoftwarePoseRingMs = 30000;    // 30 秒的位置缓存（软件同步用）

    // V27 TABLE 采样配置。控制器内部先由连续 REGIST 写五轴 raw FIFO，
    // 再由普通 BASIC 任务搬到 PC 可见的 interleaved ring。PC 只访问下列环形区。
    // Header 330..349 保持 V22 前20字段位置：capacity/write/read/fault/enable/...
    // Data 370..1137，每条: ordinal,X_latched,Y_latched,Z_latched,C_latched,B_latched。
    constexpr int kTableBase = mls::runtime::Table5D::base;
    constexpr int kPointDim = mls::runtime::Table5D::dimension;
    constexpr int kDataBase = mls::runtime::Table5D::data;

    // 旋转轴伺服使能输出。由上位机人工按钮直接控制，不使用 Basic/TABLE 握手。
    // B=OP(24)，C=OP(26)。
    constexpr int kBServoEnableOutput = 24;
    constexpr int kCServoEnableOutput = 26;

    // 轴参数按轴号直接索引，0 号保留不用，因此需要 0..5 共 6 个元素。
    constexpr int kAxisParamArraySize = 6;

    // 运动超时
    constexpr int kHomeTimeoutMs = 120000;         // 回零超时 2 分钟
    constexpr int kMoveTimeoutMs = 600000;         // 运动超时 10 分钟
    
    // 初始预分配大小（不是限制，仅用于减少扩容次数）
    constexpr int kInitialPoseReserve = 100000;       // 初始预分配位置数据
    constexpr int kInitialAlignedReserve = 1000000;   // 初始预分配对齐点
    constexpr int kInitialCloudReserve = 1000000;     // 初始预分配点云点
}

// ==================== 元类型注册 ====================
// 注意：对于模板类型，需要使用 typedef 避免宏解析问题
Q_DECLARE_METATYPE(LaserProfile)
Q_DECLARE_METATYPE(SamplePoint)
Q_DECLARE_METATYPE(AlignedPoint)
Q_DECLARE_METATYPE(AxisParams)
Q_DECLARE_METATYPE(JointPose5D)
Q_DECLARE_METATYPE(EndPose5D)
Q_DECLARE_METATYPE(InverseKinematics5DResult)
Q_DECLARE_METATYPE(JointState5D)
Q_DECLARE_METATYPE(JointTrajectoryPoint5D)
Q_DECLARE_METATYPE(JointTrajectory5D)
Q_DECLARE_METATYPE(JointMotionPlan5D)
Q_DECLARE_METATYPE(JointMoveResult5D)

// 模板类型使用 typedef 包装
typedef QVector<AlignedPoint> AlignedPointVector;
typedef QVector<QVector3D> QVector3DVector;
Q_DECLARE_METATYPE(AlignedPointVector)
Q_DECLARE_METATYPE(QVector3DVector)

// ==================== 传感器数据有效性判断 ====================
namespace SensorUtils {
    /**
     * @brief 判断传感器 Z 值是否有效
     * 
     * 2950-50BL 传感器返回 z=0 表示无效测量点
     * 传感器内部光斑提取算法已保证有效值在量程范围内
     */
    inline bool isValidZ(double z) {
        if (!std::isfinite(z)) return false;
        return std::abs(z) > 1e-9;  // z=0 是无效值
    }
    
    /**
     * @brief 判断传感器数据点是否有效
     * 只需检查 z 值，x 值由传感器保证有效
     */
    inline bool isValidPoint(double x, double z) {
        return std::isfinite(x) && std::isfinite(z) && isValidZ(z);
    }
}
