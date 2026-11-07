#include "core/PointCloudBuilder.h"

PointCloudBuilder::PointCloudBuilder(QObject* parent)
    : QObject(parent)
{
    // 预分配空间以减少扩容次数
    m_points.reserve(Constants::kInitialCloudReserve);
    m_alignmentLog.reserve(Constants::kInitialAlignedReserve);
}

PointCloudBuilder::~PointCloudBuilder() = default;

QVector<QVector3D> PointCloudBuilder::getPoints() const
{
    QReadLocker locker(&m_rwLock);
    return m_points;
}

QVector<AlignedPoint> PointCloudBuilder::getAlignmentLog() const
{
    QReadLocker locker(&m_rwLock);
    return m_alignmentLog;
}

void PointCloudBuilder::clear()
{
    {
        QWriteLocker locker(&m_rwLock);
        // V27.6.48: swap with empty containers instead of clear-only so a
        // previous million-point scan cannot leave its peak capacity resident.
        QVector<QVector3D>().swap(m_points);
        QVector<AlignedPoint>().swap(m_alignmentLog);
        m_points.reserve(Constants::kInitialCloudReserve);
        m_alignmentLog.reserve(Constants::kInitialAlignedReserve);
    }
    
    m_pointCount.store(0);
    
    emit cleared();
}

void PointCloudBuilder::onAlignedDataReady(const QVector<AlignedPoint>& points)
{
    if (points.isEmpty()) return;

    int addedCount = 0;
    int totalCount = 0;

    {
        QWriteLocker locker(&m_rwLock);

        // 转换并添加点
        for (const auto& ap : points) {
            m_points.append(QVector3D(
                static_cast<float>(ap.machineX),
                static_cast<float>(ap.machineY),
                static_cast<float>(ap.machineZ)
            ));
        }

        // 添加到对齐日志（无限存储）
        m_alignmentLog += points;
        
        addedCount = points.size();
        totalCount = m_points.size();
    }

    m_pointCount.store(totalCount);

    // 【关键修改】发送增量通知，不传递数据！
    emit pointsAdded(addedCount, totalCount);
}
