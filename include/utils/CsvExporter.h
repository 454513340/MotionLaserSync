#pragma once

#include <QString>
#include <QVector>

#include "core/DataTypes.h"

/**
 * @brief CSV 数据导出工具
 */
class QTextStream;
namespace CsvExporter {
void writeAlignmentHeader(QTextStream& out);
void writeAlignmentRow(QTextStream& out, const AlignedPoint& point);
void writePoseHeader(QTextStream& out);
void writePoseRow(QTextStream& out, const SamplePoint& pose);

/**
 * @brief 导出对齐点云数据
 */
bool exportAlignmentData(const QString& fileName,
                         const QVector<AlignedPoint>& data,
                         QString* errMsg = nullptr);

/**
 * @brief 导出位置采样流
 */
bool exportPoseStream(const QString& fileName,
                      const QVector<SamplePoint>& data,
                      QString* errMsg = nullptr);

/**
 * @brief 导出 3D 点云（PLY 格式）
 */
bool exportPointCloudPly(const QString& fileName,
                         const QVector<QVector3D>& points,
                         QString* errMsg = nullptr);

/**
 * @brief 导出 3D 点云（CSV 格式）
 */
bool exportPointCloudCsv(const QString& fileName,
                         const QVector<QVector3D>& points,
                         QString* errMsg = nullptr);

} // namespace CsvExporter
