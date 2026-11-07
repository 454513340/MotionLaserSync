#include "utils/PointCloudLoader.h"
#include "core/DataTypes.h"
#include "utils/LogManager.h"

#include <QFile>
#include <QFileInfo>
#include <QTextStream>
#include <QRegularExpression>
#include <QDebug>

PointCloudFileFormat PointCloudLoader::detectFormat(const QString& filePath) {
    QFileInfo fileInfo(filePath);
    if (!fileInfo.exists()) {
        return PointCloudFileFormat::Unknown;
    }
    
    QString suffix = fileInfo.suffix().toLower();
    
    if (suffix == "scan") {
        // 进一步验证文件头
        QFile file(filePath);
        if (file.open(QIODevice::ReadOnly)) {
            char magic[4];
            if (file.read(magic, 4) == 4) {
                if (magic[0] == 'S' && magic[1] == 'C' && 
                    magic[2] == 'A' && magic[3] == 'N') {
                    return PointCloudFileFormat::SCAN;
                }
            }
        }
        return PointCloudFileFormat::Unknown;
    } else if (suffix == "ply") {
        return PointCloudFileFormat::PLY;
    } else if (suffix == "csv" || suffix == "txt") {
        return PointCloudFileFormat::CSV;
    }
    
    return PointCloudFileFormat::Unknown;
}

bool PointCloudLoader::load(const QString& filePath,
                            QVector<QVector3D>& points,
                            QString* errorMsg) {
    QFileInfo fileInfo(filePath);
    if (!fileInfo.exists()) {
        if (errorMsg) *errorMsg = QObject::tr("文件不存在: %1").arg(filePath);
        return false;
    }
    
    PointCloudFileFormat format = detectFormat(filePath);
    
    switch (format) {
        case PointCloudFileFormat::SCAN:
            return loadScanFile(filePath, points, errorMsg);
        case PointCloudFileFormat::PLY:
            return loadPLY(filePath, points, errorMsg);
        case PointCloudFileFormat::CSV:
            return loadCSV(filePath, points, errorMsg);
        default:
            if (errorMsg) *errorMsg = QObject::tr("不支持的文件格式");
            return false;
    }
}

bool PointCloudLoader::loadScan(const QString& filePath,
                                ScanData& scanData,
                                QString* errorMsg) {
    return CompactScanIO::load(filePath, scanData, CompactScanIO::ReadOptions(), errorMsg);
}

bool PointCloudLoader::reconstructOrganizedScan(const ScanData& scan,
                                                 QVector<QVector3D>& points,
                                                 QVector<qsizetype>& rowOffsets,
                                                 QVector<quint32>& sampleColumns,
                                                 QString* errorMsg)
{
    // Keep exactly the same reconstruction mathematics as the existing .scan
    // reader. Only restore the archived original column ordinal for each XYZ.
    points.clear();
    rowOffsets.clear();
    sampleColumns.clear();
    rowOffsets.reserve(scan.profiles.size() + 1);
    rowOffsets.push_back(0);
    for (const auto& profile : scan.profiles) {
        const auto xyz = PointCloudReconstructor::reconstructProfile(scan.header, profile);
        if (profile.hasFiveAxis) {
            if (xyz.size() != qsizetype(profile.fiveAxis.points.size())) {
                if (errorMsg) *errorMsg = QObject::tr("SCAN轮廓变换无效，无法恢复原始采样索引");
                return false;
            }
            for (const auto& sample : profile.fiveAxis.points)
                sampleColumns.push_back(sample.index);
        } else {
            // Legacy v1/v2 reconstructProfile discards 0/0 and invalid Z;
            // reproduce the exact predicate to preserve sample ordinal.
            const auto n = std::min(profile.sensorX.size(), profile.sensorZ.size());
            for (qsizetype i = 0; i < n; ++i) {
                if (profile.sensorX[i] == 0 && profile.sensorZ[i] == 0) continue;
                const double x = profile.sensorX[i] * scan.header.xScale + scan.header.xOffset;
                const double z = profile.sensorZ[i] * scan.header.zScale + scan.header.zOffset;
                if (SensorUtils::isValidPoint(x, z)) sampleColumns.push_back(quint32(i));
            }
        }
        points.append(xyz);
        rowOffsets.push_back(points.size());
        if (points.size() != sampleColumns.size()) {
            if (errorMsg) *errorMsg = QObject::tr("SCAN点云与原始轮廓索引数量不一致");
            return false;
        }
    }
    return true;
}

PointCloudLoader::FileInfo PointCloudLoader::getFileInfo(const QString& filePath) {
    FileInfo info;
    
    QFileInfo fileInfo(filePath);
    if (!fileInfo.exists()) {
        return info;
    }
    
    info.fileSize = fileInfo.size();
    info.format = detectFormat(filePath);
    
    switch (info.format) {
        case PointCloudFileFormat::SCAN: {
            ScanFileHeader header;
            QString errorMsg;
            if (CompactScanIO::readHeader(filePath, header, &errorMsg)) {
                info.isValid = true;
                info.formatName = QObject::tr("紧凑扫描格式 (SCAN)");
                info.profileCount = header.profileCount;
                info.pointCount = header.profileCount * header.pointsPerProfile;
                
                switch (header.compression) {
                    case CompressionType::None:
                        info.compression = QObject::tr("无");
                        break;
                    case CompressionType::LZ4:
                        info.compression = "LZ4";
                        break;
                    case CompressionType::ZSTD:
                        info.compression = "ZSTD";
                        break;
                }
            }
            break;
        }
        
        case PointCloudFileFormat::PLY: {
            QFile file(filePath);
            if (file.open(QIODevice::ReadOnly | QIODevice::Text)) {
                QTextStream in(&file);
                QString line;
                bool headerEnded = false;
                
                while (!in.atEnd() && !headerEnded) {
                    line = in.readLine().trimmed();
                    if (line.startsWith("element vertex")) {
                        QStringList parts = line.split(' ', Qt::SkipEmptyParts);
                        if (parts.size() >= 3) {
                            info.pointCount = parts[2].toULongLong();
                        }
                    } else if (line.startsWith("format")) {
                        if (line.contains("binary")) {
                            info.formatName = QObject::tr("PLY 二进制");
                        } else {
                            info.formatName = QObject::tr("PLY ASCII");
                        }
                    } else if (line == "end_header") {
                        headerEnded = true;
                    }
                }
                info.isValid = true;
                info.compression = QObject::tr("无");
            }
            break;
        }
        
        case PointCloudFileFormat::CSV: {
            QFile file(filePath);
            if (file.open(QIODevice::ReadOnly | QIODevice::Text)) {
                QTextStream in(&file);
                uint64_t lineCount = 0;
                while (!in.atEnd()) {
                    in.readLine();
                    lineCount++;
                }
                // 减去表头行
                info.pointCount = lineCount > 0 ? lineCount - 1 : 0;
                info.formatName = "CSV";
                info.isValid = true;
                info.compression = QObject::tr("无");
            }
            break;
        }
        
        default:
            break;
    }
    
    return info;
}

bool PointCloudLoader::loadPLY(const QString& filePath,
                               QVector<QVector3D>& points,
                               QString* errorMsg) {
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly)) {
        if (errorMsg) *errorMsg = QObject::tr("无法打开文件: %1").arg(file.errorString());
        return false;
    }
    
    QTextStream in(&file);
    QString line;
    uint64_t vertexCount = 0;
    bool isBinary = false;
    bool headerEnded = false;
    
    // 解析头部
    while (!in.atEnd() && !headerEnded) {
        line = in.readLine().trimmed();
        
        if (line.startsWith("element vertex")) {
            QStringList parts = line.split(' ', Qt::SkipEmptyParts);
            if (parts.size() >= 3) {
                vertexCount = parts[2].toULongLong();
            }
        } else if (line.startsWith("format binary")) {
            isBinary = true;
        } else if (line == "end_header") {
            headerEnded = true;
        }
    }
    
    if (vertexCount == 0) {
        if (errorMsg) *errorMsg = QObject::tr("无效的PLY文件：未找到顶点数据");
        return false;
    }
    
    points.clear();
    points.reserve(static_cast<int>(vertexCount));
    
    if (isBinary) {
        // 二进制PLY需要重新打开文件并跳过头部
        file.close();
        if (!file.open(QIODevice::ReadOnly)) {
            if (errorMsg) *errorMsg = QObject::tr("无法重新打开文件");
            return false;
        }
        
        // 跳过头部
        while (!file.atEnd()) {
            QByteArray lineBytes = file.readLine();
            if (lineBytes.trimmed() == "end_header") {
                break;
            }
        }
        
        // 读取二进制数据
        for (uint64_t i = 0; i < vertexCount; ++i) {
            float x, y, z;
            if (file.read(reinterpret_cast<char*>(&x), sizeof(float)) != sizeof(float) ||
                file.read(reinterpret_cast<char*>(&y), sizeof(float)) != sizeof(float) ||
                file.read(reinterpret_cast<char*>(&z), sizeof(float)) != sizeof(float)) {
                if (errorMsg) *errorMsg = QObject::tr("读取二进制数据失败");
                return false;
            }
            points.append(QVector3D(x, y, z));
        }
    } else {
        // ASCII PLY
        while (!in.atEnd() && points.size() < static_cast<int>(vertexCount)) {
            line = in.readLine().trimmed();
            if (line.isEmpty()) continue;
            
            QStringList parts = line.split(QRegularExpression("\\s+"), Qt::SkipEmptyParts);
            if (parts.size() >= 3) {
                float x = parts[0].toFloat();
                float y = parts[1].toFloat();
                float z = parts[2].toFloat();
                points.append(QVector3D(x, y, z));
            }
        }
    }
    
    LOG_INFO(LogCategory::File, QString("加载PLY文件: %1, 点数: %2")
             .arg(filePath).arg(points.size()));
    
    return true;
}

bool PointCloudLoader::loadCSV(const QString& filePath,
                               QVector<QVector3D>& points,
                               QString* errorMsg) {
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        if (errorMsg) *errorMsg = QObject::tr("无法打开文件: %1").arg(file.errorString());
        return false;
    }
    
    QTextStream in(&file);
    points.clear();
    
    // 跳过表头
    QString header = in.readLine();
    
    // 确定分隔符
    QChar delimiter = header.contains('\t') ? '\t' : ',';
    
    // 确定X, Y, Z列的索引
    QStringList headerParts = header.split(delimiter, Qt::SkipEmptyParts);
    int xCol = -1, yCol = -1, zCol = -1;
    
    for (int i = 0; i < headerParts.size(); ++i) {
        QString col = headerParts[i].trimmed().toUpper();
        if (col == "X" || col == "MACHINE_X") xCol = i;
        else if (col == "Y" || col == "MACHINE_Y") yCol = i;
        else if (col == "Z" || col == "MACHINE_Z") zCol = i;
    }
    
    // 如果没有找到列名，假设前三列是X, Y, Z
    if (xCol < 0) xCol = 0;
    if (yCol < 0) yCol = 1;
    if (zCol < 0) zCol = 2;
    
    int maxCol = qMax(xCol, qMax(yCol, zCol));
    
    while (!in.atEnd()) {
        QString line = in.readLine().trimmed();
        if (line.isEmpty()) continue;
        
        QStringList parts = line.split(delimiter, Qt::SkipEmptyParts);
        if (parts.size() > maxCol) {
            float x = parts[xCol].toFloat();
            float y = parts[yCol].toFloat();
            float z = parts[zCol].toFloat();
            points.append(QVector3D(x, y, z));
        }
    }
    
    LOG_INFO(LogCategory::File, QString("加载CSV文件: %1, 点数: %2")
             .arg(filePath).arg(points.size()));
    
    return true;
}

bool PointCloudLoader::loadScanFile(const QString& filePath,
                                    QVector<QVector3D>& points,
                                    QString* errorMsg) {
    ScanData scanData;
    if (!CompactScanIO::load(filePath, scanData, CompactScanIO::ReadOptions(), errorMsg)) {
        return false;
    }
    
    // 重建点云
    points = PointCloudReconstructor::reconstruct(scanData);
    
    LOG_INFO(LogCategory::File, QString("加载SCAN文件: %1, 轮廓数: %2, 重建点数: %3")
             .arg(filePath).arg(scanData.profiles.size()).arg(points.size()));
    
    return true;
}
