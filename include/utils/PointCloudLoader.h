#pragma once

#include <QString>
#include <QVector>
#include <QVector3D>
#include "utils/CompactScanFormat.h"

/**
 * @brief 点云文件格式枚举
 */
enum class PointCloudFileFormat {
    Unknown,
    SCAN,       // .scan 紧凑扫描格式
    PLY,        // .ply 格式
    CSV         // .csv 格式
};

/**
 * @brief 点云文件加载器
 * 
 * 支持加载多种格式的点云文件：
 * - .scan (紧凑扫描格式)
 * - .ply (PLY格式)
 * - .csv (CSV格式)
 */
class PointCloudLoader {
public:
    /**
     * @brief 检测文件格式
     * @param filePath 文件路径
     * @return 文件格式枚举
     */
    static PointCloudFileFormat detectFormat(const QString& filePath);
    
    /**
     * @brief 加载点云文件
     * @param filePath 文件路径
     * @param points 输出点云数据
     * @param errorMsg 错误信息
     * @return 成功返回true
     */
    static bool load(const QString& filePath,
                     QVector<QVector3D>& points,
                     QString* errorMsg = nullptr);
    
    /**
     * @brief 加载紧凑扫描文件
     * @param filePath 文件路径
     * @param scanData 输出扫描数据（包含原始轮廓信息）
     * @param errorMsg 错误信息
     * @return 成功返回true
     */
    static bool loadScan(const QString& filePath,
                         ScanData& scanData,
                         QString* errorMsg = nullptr);
    
    /**
     * Reconstruct .scan XYZ AND its original organized sample topology.
     * rowOffsets contains profileCount + 1 offsets (including empty profiles);
     * sampleColumns[i] is the unfiltered original laser sample number of
     * XYZ point i. This prevents holes from shifting subsequent columns.
     * This is a read-only preview helper; no calibration/provenance is changed.
     */
    static bool reconstructOrganizedScan(const ScanData& scan,
                                         QVector<QVector3D>& points,
                                         QVector<qsizetype>& rowOffsets,
                                         QVector<quint32>& sampleColumns,
                                         QString* errorMsg = nullptr);

    /**
     * @brief 获取文件信息（不加载全部数据）
     */
    struct FileInfo {
        PointCloudFileFormat format = PointCloudFileFormat::Unknown;
        QString formatName;           // 格式名称
        uint64_t pointCount = 0;      // 点数
        uint64_t profileCount = 0;    // 轮廓数（仅.scan格式）
        uint64_t fileSize = 0;        // 文件大小
        QString compression;          // 压缩类型
        bool isValid = false;
    };
    
    static FileInfo getFileInfo(const QString& filePath);
    
private:
    static bool loadPLY(const QString& filePath, QVector<QVector3D>& points, QString* errorMsg);
    static bool loadCSV(const QString& filePath, QVector<QVector3D>& points, QString* errorMsg);
    static bool loadScanFile(const QString& filePath, QVector<QVector3D>& points, QString* errorMsg);
};
