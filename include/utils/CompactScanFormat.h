#pragma once

/** V21 .scan v3: full XYZCB + immutable 17-parameter/hand-eye transform per
 * profile; original point indices and binary64 XZ, variable profile lengths.
 * v1/v2 are READ-ONLY legacy XYZ archives and cannot recover missing C/B.
 */
#include <QVector>
#include <QVector3D>
#include <QMatrix4x4>
#include <QString>
#include <QByteArray>
#include <QObject>
#include <cstdint>
#include "runtime/ScanArchive5D.h"

// ==================== 压缩算法类型 ====================
enum class CompressionType : uint8_t {
    None = 0,           // 无压缩
    LZ4 = 1,            // historical v2 value: actually Qt zlib; migrated explicitly on write
    ZSTD = 2,           // historical v2 value: actually Qt zlib; migrated explicitly on write
    QtZlib = 3          // accurately named qCompress envelope + zlib
};

// ==================== 存储格式版本 ====================
constexpr uint32_t SCAN_FORMAT_VERSION = 3;
constexpr char SCAN_FORMAT_MAGIC[4] = {'S', 'C', 'A', 'N'};

// ==================== 文件头结构（256字节，固定大小）====================
#pragma pack(push, 1)
struct ScanFileHeader {
    char magic[4] = {'S', 'C', 'A', 'N'};   // 文件标识 (4)
    uint32_t version = SCAN_FORMAT_VERSION;  // 格式版本 (4)
    
    // 扫描信息
    uint64_t profileCount = 0;           // 轮廓总数 (8)
    uint64_t startTimeUs = 0;                // 起始时间戳（微秒）(8)
    uint32_t sampleIntervalUs = 5000;        // 名义周期；V3每条轮廓另存实际时间戳 (4)
    
    // 传感器参数
    uint16_t pointsPerProfile = 1280;        // 每条轮廓的点数 (2)
    uint16_t reserved1 = 0;                  // 对齐填充 (2)
    float xScale = 0.001f;                   // int16 → mm 缩放因子 (4)，精度0.001mm
    float zScale = 0.001f;                   // int16 → mm 缩放因子 (4)，精度0.001mm
    float xOffset = 0.0f;                    // X偏移量 (mm)，存储时减去 (4)
    float zOffset = 95.0f;                   // Z偏移量 (mm)，默认95mm居中 (4)
    
    // 手眼矩阵（传感器相对于机床坐标系的变换）(64)
    float handEyeMatrix[16] = {
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1
    };
    
    // 压缩信息
    CompressionType compression = CompressionType::None;  // (1)
    uint8_t compressionLevel = 1;            // 压缩级别 (1-9) (1)
    uint16_t reserved2 = 0;                  // 对齐填充 (2)
    uint64_t uncompressedSize = 0;           // 未压缩数据大小 (8)
    uint64_t compressedSize = 0;             // 压缩后数据大小 (8)
    
    // 扫描元数据
    char scanName[64] = {0};                 // 扫描名称 (64)
    char blockName[32] = {0};                // 关联的程序段名称 (32)
    int32_t blockLineNumber = 0;             // 关联的行号 (4)
    
    // 预留空间（确保头部为256字节）
    uint8_t reserved[20] = {0};              // (20)
    
    // 校验和
    uint32_t headerChecksum = 0;             // 头部校验和 (4)
};
#pragma pack(pop)

static_assert(sizeof(ScanFileHeader) == 256, "ScanFileHeader must be 256 bytes");

// ==================== 紧凑轮廓数据（每条轮廓）====================
#pragma pack(push, 1)
struct CompactProfileData {
    // 机床坐标（每条轮廓存储一次）
    float machineX = 0.0f;                   // 4字节
    float machineY = 0.0f;                   // 4字节
    float machineZ = 0.0f;                   // 4字节
    // 传感器数据（动态大小，通常是1280点）
    // int16_t sensorX[pointsPerProfile];    // 2560字节 @ 1280点
    // int16_t sensorZ[pointsPerProfile];    // 2560字节 @ 1280点
    // 总计：12 + 2560 + 2560 = 5132字节/轮廓
};
#pragma pack(pop)

// 计算每条轮廓的字节大小
inline size_t calculateProfileDataSize(uint16_t pointsPerProfile) {
    return sizeof(CompactProfileData) + pointsPerProfile * 2 * sizeof(int16_t);
}

// ==================== 内存中的轮廓数据 ====================
struct ProfileData {
    bool hasFiveAxis = false;
    mls::runtime::ArchiveProfile fiveAxis; // authoritative v3 record; legacy fields below are read-only
    float machineX = 0.0f;
    float machineY = 0.0f;
    float machineZ = 0.0f;
    QVector<int16_t> sensorX;                // 传感器X坐标（缩放后的int16）
    QVector<int16_t> sensorZ;                // 传感器Z坐标（缩放后的int16）
};

// ==================== 扫描数据（完整内存表示）====================
struct ScanData {
    ScanFileHeader header;
    QVector<ProfileData> profiles;
    QString validationError;
    
    // 计算存储统计
    struct Statistics {
        uint64_t totalPoints = 0;            // 总点数
        uint64_t rawSizeBytes = 0;           // 原始大小
        uint64_t compactSizeBytes = 0;       // 紧凑格式大小
        uint64_t compressedSizeBytes = 0;    // 压缩后大小
        double compressionRatio = 1.0;       // 压缩比
        double savingsPercent = 0.0;         // 节省百分比
    };
    
    Statistics calculateStatistics() const;
};

// ==================== 紧凑扫描文件读写器 ====================
class CompactScanIO {
public:
    // 写入选项
    struct WriteOptions {
        CompressionType compression;
        int compressionLevel;            // 1=快速, 9=最高压缩
        QString scanName;
        QString blockName;
        int blockLineNumber;
        WriteOptions() : compression(CompressionType::QtZlib), compressionLevel(1), blockLineNumber(0) {}
    };
    
    // 读取选项
    struct ReadOptions {
        bool loadAllProfiles;         // 是否加载所有轮廓
        uint64_t startProfile;           // 起始轮廓索引
        uint64_t profileCount;           // 读取轮廓数（0=全部）
        ReadOptions() : loadAllProfiles(true), startProfile(0), profileCount(0) {}
    };

    /**
     * @brief 保存扫描数据到文件
     * @param filePath 文件路径
     * @param data 扫描数据
     * @param options 写入选项
     * @param errorMsg 错误信息输出
     * @return 成功返回true
     */
    static bool save(const QString& filePath, 
                     const ScanData& data,
                     const WriteOptions& options = WriteOptions(),
                     QString* errorMsg = nullptr);

    /**
     * @brief 从文件加载扫描数据
     * @param filePath 文件路径
     * @param data 扫描数据输出
     * @param options 读取选项
     * @param errorMsg 错误信息输出
     * @return 成功返回true
     */
    static bool load(const QString& filePath,
                     ScanData& data,
                     const ReadOptions& options = ReadOptions(),
                     QString* errorMsg = nullptr);

    /**
     * @brief 仅读取文件头
     * @param filePath 文件路径
     * @param header 头部输出
     * @param errorMsg 错误信息输出
     * @return 成功返回true
     */
    static bool readHeader(const QString& filePath,
                           ScanFileHeader& header,
                           QString* errorMsg = nullptr);

    /**
     * @brief 从AlignedPoint数据创建ScanData
     * @param alignedPoints 对齐点数据
     * @param startTimeUs 起始时间戳
     * @param sampleIntervalUs 采样间隔
     * @param handEyeMatrix 手眼矩阵（可选）
     * @return ScanData
     */
    static ScanData fromAlignedPoints(const QVector<struct AlignedPoint>& alignedPoints,
                                      uint64_t startTimeUs = 0,
                                      uint32_t sampleIntervalUs = 5000,
                                      const QMatrix4x4& handEyeMatrix = QMatrix4x4());

    /**
     * @brief 从LaserProfile和SamplePoint数据创建ScanData
     * @param profiles 轮廓数据
     * @param poses 位置采样数据
     * @param startTimeUs 起始时间戳
     * @param sampleIntervalUs 采样间隔
     * @param handEyeMatrix 手眼矩阵
     * @return ScanData
     */
    static ScanData fromProfilesAndPoses(const QVector<struct LaserProfile>& profiles,
                                         const QVector<struct SamplePoint>& poses,
                                         uint64_t startTimeUs = 0,
                                         uint32_t sampleIntervalUs = 5000,
                                         const QMatrix4x4& handEyeMatrix = QMatrix4x4());

private:
    static QByteArray compress(const QByteArray& data, CompressionType type, int level);
    static QByteArray decompress(const QByteArray& data, CompressionType type, size_t expectedSize);
    static uint32_t calculateChecksum(const void* data, size_t size);
};

// ==================== 点云重建器 ====================
class PointCloudReconstructor {
public:
    /**
     * @brief 从ScanData重建完整点云
     * @param data 扫描数据
     * @return 世界坐标系中的点云
     */
    static QVector<QVector3D> reconstruct(const ScanData& data);

    /**
     * @brief 重建指定范围的点云
     * @param data 扫描数据
     * @param startProfile 起始轮廓索引
     * @param profileCount 轮廓数量（0=到末尾）
     * @return 世界坐标系中的点云
     */
    static QVector<QVector3D> reconstructRange(const ScanData& data,
                                                uint64_t startProfile,
                                                uint64_t profileCount = 0);

    /**
     * @brief 重建单条轮廓的点云
     * @param header 文件头（包含手眼矩阵和缩放参数）
     * @param profile 轮廓数据
     * @return 世界坐标系中的点
     */
    static QVector<QVector3D> reconstructProfile(const ScanFileHeader& header,
                                                  const ProfileData& profile);

    /**
     * @brief 获取重建后的时间戳
     * @param header 文件头
     * @param profileIndex 轮廓索引
     * @return 时间戳（微秒）
     */
    static uint64_t getTimestamp(const ScanFileHeader& header, uint64_t profileIndex);
};

// ==================== 保存格式枚举（用于UI选择）====================
enum class PointCloudSaveFormat {
    CSV,                    // CSV 文本格式（最大兼容性）
    PLY_ASCII,              // PLY ASCII 格式
    PLY_BINARY,             // PLY 二进制格式
    SCAN_UNCOMPRESSED,      // .scan 紧凑格式（无压缩）
    SCAN_LZ4,               // .scan 紧凑格式 + LZ4压缩（推荐）
    SCAN_ZSTD               // .scan 紧凑格式 + ZSTD压缩（最高压缩比）
};

/**
 * @brief 获取格式的文件扩展名
 */
inline QString getFormatExtension(PointCloudSaveFormat format) {
    switch (format) {
        case PointCloudSaveFormat::CSV: return ".csv";
        case PointCloudSaveFormat::PLY_ASCII:
        case PointCloudSaveFormat::PLY_BINARY: return ".ply";
        case PointCloudSaveFormat::SCAN_UNCOMPRESSED:
        case PointCloudSaveFormat::SCAN_LZ4:
        case PointCloudSaveFormat::SCAN_ZSTD: return ".scan";
        default: return ".dat";
    }
}

/**
 * @brief 获取格式的描述
 */
inline QString getFormatDescription(PointCloudSaveFormat format) {
    switch (format) {
        case PointCloudSaveFormat::CSV: 
            return QObject::tr("CSV 文本格式 (*.csv) - 派生XYZ点；不保存可重建的五轴Physical17快照");
        case PointCloudSaveFormat::PLY_ASCII: 
            return QObject::tr("PLY ASCII 格式 (*.ply) - 派生XYZ点；不保存XYZCB/Physical17来源");
        case PointCloudSaveFormat::PLY_BINARY: 
            return QObject::tr("PLY 二进制格式 (*.ply) - 派生XYZ点；不保存XYZCB/Physical17来源");
        case PointCloudSaveFormat::SCAN_UNCOMPRESSED: 
            return QObject::tr("五轴源扫描格式 (*.scan) - 无压缩；保存每轮廓XYZCB、17参数、变换及原始传感器XZ");
        case PointCloudSaveFormat::SCAN_LZ4: 
            return QObject::tr("五轴源扫描格式 + Qt zlib (*.scan) - 无损；保存XYZCB、Physical17、每轮廓变换和原始XZ，可重新重建点云");
        case PointCloudSaveFormat::SCAN_ZSTD: 
            return QObject::tr("五轴扫描格式 + Qt zlib (*.scan) - 兼容旧选项；并非ZSTD");
        default: 
            return QObject::tr("未知格式");
    }
}

/**
 * @brief 估算存储大小
 * @param pointCount 点数
 * @param format 格式
 * @return 估算字节数
 */
inline uint64_t estimateStorageSize(uint64_t pointCount, PointCloudSaveFormat format) {
    switch (format) {
        case PointCloudSaveFormat::CSV:
            return pointCount * 1400;  // Full double-precision provenance CSV, rough estimate
        case PointCloudSaveFormat::PLY_ASCII:
            return pointCount * 35;  // 约35字节/点
        case PointCloudSaveFormat::PLY_BINARY:
            return pointCount * 12 + 200;  // 12字节/点 + 头部
        case PointCloudSaveFormat::SCAN_UNCOMPRESSED:
        case PointCloudSaveFormat::SCAN_LZ4:
        case PointCloudSaveFormat::SCAN_ZSTD:
            // Uncompressed estimate only. Do not promise a data-dependent ratio.
            return pointCount * 20 + ((pointCount + 1279) / 1280) * 460 + 276;
        default:
            return pointCount * 40;
    }
}
