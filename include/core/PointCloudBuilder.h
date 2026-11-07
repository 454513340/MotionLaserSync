#pragma once

#include <QObject>
#include <QVector>
#include <QVector3D>
#include <QMutex>
#include <QReadWriteLock>
#include <atomic>

#include "core/DataTypes.h"

/**
 * @brief 点云构建器（重构版）
 * 
 * 接收对齐后的点数据，构建和管理 3D 点云。
 * 无存储容量限制，支持无限数据采集。
 * 
 * 重构改进：
 * 1. 使用读写锁支持并发访问
 * 2. 增量更新通知，避免全量拷贝
 */
class PointCloudBuilder : public QObject
{
    Q_OBJECT

public:
    explicit PointCloudBuilder(QObject* parent = nullptr);
    ~PointCloudBuilder() override;

    /**
     * @brief 获取当前点云数量（无锁，原子操作）
     */
    int pointCount() const { return m_pointCount.load(); }

    /**
     * @brief 获取点云数据副本（线程安全）
     */
    QVector<QVector3D> getPoints() const;

    /**
     * @brief 获取对齐日志副本
     */
    QVector<AlignedPoint> getAlignmentLog() const;
    
    /**
     * @brief 获取点云数据的常量引用（需配合读锁使用）
     */
    const QVector<QVector3D>& getPointsRef() const { return m_points; }
    
    /**
     * @brief 获取读写锁引用
     */
    QReadWriteLock& rwLock() { return m_rwLock; }

    /**
     * @brief 清空点云
     */
    void clear();

public slots:
    /**
     * @brief 接收对齐数据
     */
    void onAlignedDataReady(const QVector<AlignedPoint>& points);

signals:
    /**
     * @brief 【新信号】增量更新通知（不传递数据）
     */
    void pointsAdded(int addedCount, int totalCount);
    
    /**
     * @brief 点云已清空
     */
    void cleared();

private:
    QVector<QVector3D> m_points;
    QVector<AlignedPoint> m_alignmentLog;
    mutable QReadWriteLock m_rwLock;
    std::atomic<int> m_pointCount{0};
};
