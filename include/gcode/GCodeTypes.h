#pragma once

#include <QObject>
#include <QString>
#include <QVector>
#include <QVector3D>
#include <QDateTime>
#include <optional>
#include <vector>

#include "core/DataTypes.h"

/**
 * @file GCodeTypes.h
 * @brief G代码系统数据类型定义
 * 
 * 定义G代码程序、程序段、参数等数据结构
 */

// ==================== 程序段类型 ====================
enum class BlockType {
    // ===== 运动指令 (G代码) =====
    G00_RapidMove,          // G00 快速定位（点到点，默认不采集）
    G01_LinearInterp,       // G01 直线插补（可选采集）
    G02_ArcCW,              // G02 顺时针圆弧（可选采集）
    G03_ArcCCW,             // G03 逆时针圆弧（可选采集）
    G04_Dwell,              // G04 暂停/延时
    
    // ===== 连续轨迹指令 =====
    G05_ContinuousPath,     // G05 连续轨迹（多点连续插补）
    
    // ===== 坐标系指令 =====
    G90_AbsoluteMode,       // G90 切换到绝对坐标模式
    G91_IncrementalMode,    // G91 切换到相对坐标模式
    G92_SetPosition,        // G92 设置当前位置（清零）
    
    // ===== 同步采集控制 (M代码) =====
    M50_SyncOn,             // M50 开启同步采集
    M51_SyncOff,            // M51 关闭同步采集
    
    // ===== 数据管理指令 =====
    M60_SavePointCloud,     // M60 保存当前点云数据（异步）
    M61_ClearPointCloud,    // M61 清空点云数据
    M62_SetSavePath,        // M62 设置保存路径和文件名前缀
    M63_WaitSaveComplete,   // M63 等待所有保存任务完成
    
    // ===== 程序控制 =====
    M00_ProgramStop,        // M00 程序暂停（等待用户继续）
    M30_ProgramEnd,         // M30 程序结束
    
    // ===== 辅助 =====
    Comment,                // 注释行
    Empty,                  // 空行
    Modal                   // non-motion G17/G18/G19/G21/G43.4/G49/G93/G94/F
};

// ==================== 圆弧平面选择 ====================
enum class ArcPlane {
    XY,     // XY平面（默认）
    XZ,     // XZ平面
    YZ      // YZ平面
};

// ==================== 圆弧定义方式 ====================
enum class ArcDefineMode {
    ThreePoint,     // 三点定义（起点、中间点、终点）
    CenterEnd,      // 圆心+终点定义
    RadiusEnd       // 半径+终点定义
};

// ==================== 圆弧参数 ====================
struct ArcParams {
    ArcDefineMode defineMode = ArcDefineMode::ThreePoint;
    ArcPlane plane = ArcPlane::XY;
    
    // 三点定义（起点为当前位置）
    double midX = 0.0, midY = 0.0, midZ = 0.0;    // 中间点
    double endX = 0.0, endY = 0.0, endZ = 0.0;    // 终点
    
    // 圆心定义（I, J, K 相对于起点的偏移）
    double centerI = 0.0, centerJ = 0.0, centerK = 0.0;
    
    // 半径定义
    double radius = 0.0;
    
    // 螺旋参数（可选）
    float helixPitch = 0.0f;    // 螺旋升程
};

// ==================== 连续轨迹参数 ====================
struct ContinuousPathParams {
    std::vector<PathPoint> points;  // 轨迹点列表
    QString name;                   // 轨迹名称（用于保存文件命名）
    bool closed = false;            // 是否闭合轨迹
};

// ==================== G代码运动参数 ====================
/**
 * @brief G代码运动参数
 * 
 * 注意：默认值参考 AxisParams 的实际控制器参数
 * - 速度单位：mm/s（取决于脉冲当量设置）
 * - 加速度单位：mm/s²
 * 
 * 实际使用时应从控制器读取当前参数或由用户在界面设置
 */
struct GCodeMotionParams {
    // 快速定位速度 (G00) - 空行程移动，可以稍快
    float rapidSpeed = 10.0f;       // G00 快速定位速度 (units/s)
    
    // 进给速度 (G01, G02, G03, G05) - 扫描时使用的速度
    float feedSpeed = 5.0f;         // 默认进给速度 (units/s)，与AxisParams.speed一致
    
    // 加减速参数（与AxisParams默认值一致）
    float accel = 25.0f;            // 加速度 (units/s²)
    float decel = 25.0f;            // 减速度 (units/s²)
    
    // S曲线参数
    float sramp = 50.0f;            // S曲线时间比例 (0-100)
    
    // 起始速度（与AxisParams默认值一致）
    float lspeed = 0.5f;            // 起始速度 (units/s)
    
    /**
     * @brief 从AxisParams复制参数（用于从控制器读取后同步）
     */
    void fromAxisParams(const AxisParams& params) {
        feedSpeed = params.speed;
        rapidSpeed = params.speed * 2.0f;  // 快速速度默认为进给速度的2倍
        accel = params.accel;
        decel = params.decel;
        sramp = params.sramp;
        lspeed = params.lspeed;
    }
    
    /**
     * @brief 转换为AxisParams用于运动控制
     * @param useRapid true使用快速速度(G00)，false使用进给速度(G01等)
     * @param overrideSpeed 如果>0，则使用此速度覆盖默认速度（F参数）
     */
    AxisParams toAxisParams(bool useRapid = false, float overrideSpeed = 0.0f) const {
        AxisParams params;
        if (overrideSpeed > 0.0f) {
            params.speed = overrideSpeed;
        } else {
            params.speed = useRapid ? rapidSpeed : feedSpeed;
        }
        params.accel = accel;
        params.decel = decel;
        params.sramp = sramp;
        params.lspeed = lspeed;
        return params;
    }
};

// ==================== 保存参数 ====================
struct SaveParams {
    QString baseDir;                // 基础目录
    QString prefix;                 // 文件前缀
    QString suffix;                 // 文件后缀
    int autoIndex = 0;              // 自动递增编号
    bool useBlockName = true;       // 使用程序段名称作为文件名一部分
    QString format = "scan_zlib";    // 保存格式: csv, ply, ply_bin, scan, scan_lz4, scan_zstd
    
    /**
     * @brief 获取格式对应的文件扩展名
     */
    QString getExtension() const {
        if (format == "csv") return "csv";
        if (format == "ply" || format == "ply_bin") return "ply";
        if (format.startsWith("scan")) return "scan";
        return format;  // 默认使用format作为扩展名
    }
    
    /**
     * @brief 生成完整文件路径
     */
    QString generateFilePath(const QString& blockName = QString()) const {
        QString fileName = prefix;
        if (useBlockName && !blockName.isEmpty()) {
            if (!fileName.isEmpty()) fileName += "_";
            fileName += blockName;
        }
        fileName += QString("_%1").arg(autoIndex, 3, 10, QChar('0'));
        fileName += "." + getExtension();
        
        QString path = baseDir;
        if (!path.endsWith('/') && !path.endsWith('\\')) {
            path += '/';
        }
        return path + fileName;
    }
};

// ==================== 程序段定义 ====================
struct GCodeBlock {
    BlockType type = BlockType::Empty;
    
    // 标识信息
    int lineNumber = 0;             // 行号 (N10, N20...)
    QString name;                   // 程序段名称（L参数）
    QString comment;                // 注释
    QString rawLine;                // 原始文本行
    
    // 目标位置（用于 G00, G01, G92）
    PathPoint target;
    bool hasX = false, hasY = false, hasZ = false, hasC = false, hasB = false;  // 标记哪些坐标被指定
    
    std::optional<bool> absoluteModeSnapshot;
    std::optional<bool> rtcpModeSnapshot;      // G43.4=true, G49=false
    mls::runtime::FeedMode feedMode = mls::runtime::FeedMode::LegacyPerSecond;
    // 圆弧参数（用于 G02, G03）
    ArcParams arc;
    
    // 连续轨迹参数（用于 G05）
    ContinuousPathParams continuousPath;
    
    // 暂停参数（用于 G04, M00）
    int dwellTimeMs = 0;
    
    // 采集控制
    bool syncCapture = false;       // 本段是否需要同步采集（用于G01等）
    
    // 保存参数（用于 M60, M62）
    QString saveFileName;           // 直接指定的文件名
    SaveParams saveParams;          // 保存参数
    
    // 运动参数覆盖（可选）
    std::optional<double> feedRate;  // F 进给速度
    std::optional<float> accel;     // 加速度
    std::optional<float> decel;     // 减速度
};

// ==================== G代码程序定义 ====================
struct GCodeProgram {
    // 程序信息
    QString name;                   // 程序名称
    QString description;            // 程序描述
    QString author;                 // 作者
    QString version;                // 版本
    QDateTime createTime;           // 创建时间
    QString filePath;               // 文件路径（如果从文件加载）
    
    // 程序内容
    std::vector<GCodeBlock> blocks; // 程序段列表
    
    // 运动参数（G代码专用）
    GCodeMotionParams motionParams; // 运动参数
    // Header provenance: a numeric value equal to its default can still be explicitly specified.
    bool headerFeedSpecified=false, headerRapidSpecified=false;
    bool headerAccelSpecified=false, headerDecelSpecified=false;
    
    // 传感器参数
    SensorParams sensorParams;      // 传感器参数
    SyncMode syncMode = SyncMode::Software;
    bool syncModeSpecified = false; // never silently override a G-code header with stale GUI state
    // V32.16: 0 unspecified, 1 H1 OP14/IN0/R0, 3 H3 CMM/IN1/R1.
    // ID 2 is retired and MUST NOT be treated as H3.
    int hardwareTopologyId = 0;
    int hardwareFrequencyHz = 0;
    QString hardwareTriggerType; // legacy H1 sensor external edge; optional, ignored for H3
    QString sensorTriggerMode; // HardwareExternal (H1), SoftwareInternal (H3); optional in old NC
    int cmmR1Edge = 0; // H3: explicit 114 or 115
    bool offlineOnly = false; // digital-twin provenance; conditional run permission is checked independently
    QString programFrame;     // MACHINE_JOINTS_XYZCB or RTCP_TCP_XYZCB
    QString partPoseId;       // provenance, not a pose-estimation claim
    QString calibrationId;    // provenance, not an authorization signature
    QString physical17Sha256; // exact model content; compare to active SyncManager model
    MountOffset mountOffset;        // 传感器安装偏移
    
    // 默认保存设置
    SaveParams defaultSaveParams;
    
    // 坐标模式（运行时状态）
    bool absoluteMode = true;       // true=G90绝对, false=G91相对
    
    // 统计信息
    int totalLines() const { return static_cast<int>(blocks.size()); }
    int motionBlocks() const;       // 统计运动指令数量
    int captureBlocks() const;      // 统计采集指令数量
};

// ==================== 执行器状态 ====================
enum class ExecutorState {
    Idle,                   // 空闲，等待程序
    Ready,                  // 程序已加载，等待启动
    
    // 运行状态
    Running,                // 正在执行
    ExecutingBlock,         // 正在执行程序段
    WaitingMotionComplete,  // 等待运动完成
    WaitingSyncStart,       // 等待同步采集启动
    WaitingSyncStop,        // 等待同步采集停止
    WaitingSaveComplete,    // 等待保存完成
    Dwelling,               // 暂停延时中
    
    // 停止状态
    Paused,                 // 用户暂停
    Stopping,               // 正在停止
    Completed,              // 程序执行完成
    Error                   // 发生错误
};

// ==================== 执行进度信息 ====================
struct ExecutorProgress {
    int currentBlock = 0;           // 当前程序段索引
    int totalBlocks = 0;            // 总程序段数
    double completionPercent = 0.0; // 完成百分比
    
    qint64 elapsedTimeMs = 0;       // 已用时间
    qint64 estimatedRemainingMs = 0;// 预估剩余时间
    
    int profileCount = 0;           // 采集的轮廓数
    int poseCount = 0;              // 位置采样数
    int pointCloudCount = 0;        // 点云点数
    
    int saveTasksPending = 0;       // 待保存任务数
    int saveTasksCompleted = 0;     // 已完成保存数
    
    QString currentBlockInfo;       // 当前程序段信息
    ExecutorState state = ExecutorState::Idle;
};

// ==================== 辅助函数 ====================
namespace GCodeUtils {
    /**
     * @brief 获取程序段类型名称
     */
    inline QString blockTypeName(BlockType type) {
        switch (type) {
        case BlockType::G00_RapidMove:       return "G00";
        case BlockType::G01_LinearInterp:    return "G01";
        case BlockType::G02_ArcCW:           return "G02";
        case BlockType::G03_ArcCCW:          return "G03";
        case BlockType::G04_Dwell:           return "G04";
        case BlockType::G05_ContinuousPath:  return "G05";
        case BlockType::G90_AbsoluteMode:    return "G90";
        case BlockType::G91_IncrementalMode: return "G91";
        case BlockType::G92_SetPosition:     return "G92";
        case BlockType::Modal: return "Modal";
        case BlockType::M50_SyncOn:          return "M50";
        case BlockType::M51_SyncOff:         return "M51";
        case BlockType::M60_SavePointCloud:  return "M60";
        case BlockType::M61_ClearPointCloud: return "M61";
        case BlockType::M62_SetSavePath:     return "M62";
        case BlockType::M63_WaitSaveComplete:return "M63";
        case BlockType::M00_ProgramStop:     return "M00";
        case BlockType::M30_ProgramEnd:      return "M30";
        case BlockType::Comment:             return ";";
        case BlockType::Empty:               return "";
        default:                             return "???";
        }
    }
    
    /**
     * @brief 获取执行状态名称
     */
    inline QString stateName(ExecutorState state) {
        switch (state) {
        case ExecutorState::Idle:                return QObject::tr("空闲");
        case ExecutorState::Ready:               return QObject::tr("就绪");
        case ExecutorState::Running:             return QObject::tr("运行中");
        case ExecutorState::ExecutingBlock:      return QObject::tr("执行中");
        case ExecutorState::WaitingMotionComplete:return QObject::tr("等待运动完成");
        case ExecutorState::WaitingSyncStart:    return QObject::tr("等待采集启动");
        case ExecutorState::WaitingSyncStop:     return QObject::tr("等待采集停止");
        case ExecutorState::WaitingSaveComplete: return QObject::tr("等待保存完成");
        case ExecutorState::Dwelling:            return QObject::tr("暂停中");
        case ExecutorState::Paused:              return QObject::tr("已暂停");
        case ExecutorState::Stopping:            return QObject::tr("停止中");
        case ExecutorState::Completed:           return QObject::tr("已完成");
        case ExecutorState::Error:               return QObject::tr("错误");
        default:                                 return QObject::tr("未知");
        }
    }
    
    /**
     * @brief 判断是否是运动指令
     */
    inline bool isMotionBlock(BlockType type) {
        return type == BlockType::G00_RapidMove ||
               type == BlockType::G01_LinearInterp ||
               type == BlockType::G02_ArcCW ||
               type == BlockType::G03_ArcCCW ||
               type == BlockType::G05_ContinuousPath;
    }
    
    /**
     * @brief 判断指令是否默认需要采集
     */
    inline bool defaultSyncCapture(BlockType type) {
        // G00快速定位默认不采集，其他运动指令默认采集
        return type == BlockType::G01_LinearInterp ||
               type == BlockType::G02_ArcCW ||
               type == BlockType::G03_ArcCCW ||
               type == BlockType::G05_ContinuousPath;
    }
}

// 实现 GCodeProgram 的成员函数
inline int GCodeProgram::motionBlocks() const {
    int count = 0;
    for (const auto& block : blocks) {
        if (GCodeUtils::isMotionBlock(block.type)) {
            count++;
        }
    }
    return count;
}

inline int GCodeProgram::captureBlocks() const {
    int count = 0;
    for (const auto& block : blocks) {
        if (block.syncCapture) {
            count++;
        }
    }
    return count;
}

// Qt 元类型注册
Q_DECLARE_METATYPE(BlockType)
Q_DECLARE_METATYPE(ExecutorState)
Q_DECLARE_METATYPE(GCodeBlock)
Q_DECLARE_METATYPE(GCodeProgram)
Q_DECLARE_METATYPE(ExecutorProgress)
