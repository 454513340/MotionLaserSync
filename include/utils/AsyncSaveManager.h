#pragma once

#include <QObject>
#include <QThreadPool>
#include <QRunnable>
#include <QMutex>
#include <QWaitCondition>
#include <QHash>
#include <QUuid>
#include <QElapsedTimer>
#include <QDateTime>
#include <QMatrix4x4>
#include <atomic>

#include "core/DataTypes.h"
#include "utils/CompactScanFormat.h"

/**
 * @file AsyncSaveManager.h
 * @brief 异步文件保存管理器
 * 
 * 管理点云和其他数据的异步保存任务。
 * 支持多文件并行保存，不阻塞主线程。
 * 
 * 支持的格式：
 * - CSV: 文本格式，最大兼容性
 * - PLY: 通用点云格式（ASCII/Binary）
 * - SCAN: 紧凑二进制格式，支持LZ4/ZSTD压缩
 */

// ==================== 保存任务类型 ====================
enum class SaveTaskType {
    AlignedPoints,      // 对齐点数据 (CSV)
    PointCloudCSV,      // 点云 CSV
    PointCloudPLY,      // 点云 PLY (ASCII)
    PointCloudPLYBinary,// 点云 PLY (Binary)
    PoseSamples,        // 位置采样数据
    RawProfiles,        // 原始轮廓数据
    CompactScan,        // 紧凑扫描格式 (.scan)
    CompactScanLZ4,     // 紧凑扫描格式 + LZ4 压缩
    CompactScanZSTD     // 紧凑扫描格式 + ZSTD 压缩
};

// ==================== 保存任务状态 ====================
enum class SaveTaskStatus {
    Pending,            // 等待执行
    Running,            // 执行中
    Completed,          // 完成
    Failed,             // 失败
    Cancelled           // 已取消
};

// ==================== 保存任务信息 ====================
struct SaveTaskInfo {
    QString taskId;                 // 唯一任务ID
    SaveTaskType type;              // 任务类型
    QString filePath;               // 目标文件路径
    SaveTaskStatus status = SaveTaskStatus::Pending;
    QString errorMessage;           // 错误信息
    
    qint64 dataSize = 0;            // 数据大小（点数）
    qint64 bytesWritten = 0;        // 已写入字节数
    double progress = 0.0;          // 进度 0.0 ~ 1.0
    
    qint64 submitTimeMs = 0;        // 提交时间
    qint64 startTimeMs = 0;         // 开始时间
    qint64 endTimeMs = 0;           // 结束时间
    
    QString blockName;              // 关联的程序段名称
    int blockLineNumber = 0;        // 关联的行号
};

/**
 * @brief 保存任务（QRunnable 实现）
 */
class SaveTask : public QObject, public QRunnable
{
    Q_OBJECT

public:
    SaveTask(const QString& taskId, 
             SaveTaskType type,
             const QString& filePath,
             QObject* parent = nullptr);
    ~SaveTask() override = default;

    // 设置数据（深拷贝）
    void setAlignedPoints(const QVector<AlignedPoint>& points);
    void setPointCloud(const QVector<QVector3D>& points);
    void setPoseSamples(const QVector<SamplePoint>& samples);
    void setLaserProfiles(const QVector<LaserProfile>& profiles);
    
    // 设置紧凑格式参数
    void setCompactScanParams(uint64_t startTimeUs, 
                              uint32_t sampleIntervalUs,
                              const QMatrix4x4& handEyeMatrix);
    void setCompressionLevel(int level) { m_compressionLevel = level; }
    void setScanName(const QString& name) { m_scanName = name; }
    void setBlockInfo(const QString& blockName, int lineNumber) {
        m_blockName = blockName;
        m_blockLineNumber = lineNumber;
    }
    
    // QRunnable 接口
    void run() override;
    
    // 取消任务
    void cancel() { m_cancelled.store(true); }
    bool isCancelled() const { return m_cancelled.load(); }
    
    QString taskId() const { return m_taskId; }
    QString filePath() const { return m_filePath; }

signals:
    void started(const QString& taskId);
    void progressUpdated(const QString& taskId, double progress, qint64 bytesWritten);
    void completed(const QString& taskId, const QString& filePath, qint64 totalBytes);
    void failed(const QString& taskId, const QString& filePath, const QString& errorMessage);

private:
    bool saveAlignedPointsCSV();
    bool savePointCloudCSV();
    bool savePointCloudPLY();
    bool savePointCloudPLYBinary();
    bool savePoseSamplesCSV();
    bool saveCompactScan(CompressionType compression);

private:
    QString m_taskId;
    SaveTaskType m_type;
    QString m_filePath;
    
    // 数据（深拷贝）
    QVector<AlignedPoint> m_alignedPoints;
    QVector<QVector3D> m_cloudPoints;
    QVector<SamplePoint> m_poseSamples;
    QVector<LaserProfile> m_laserProfiles;
    
    // 紧凑格式参数
    uint64_t m_startTimeUs = 0;
    uint32_t m_sampleIntervalUs = 1000;
    QMatrix4x4 m_handEyeMatrix;
    int m_compressionLevel = 1;
    QString m_scanName;
    QString m_blockName;
    int m_blockLineNumber = 0;
    
    std::atomic<bool> m_cancelled{false};
    
    static constexpr int kProgressUpdateIntervalMs = 100;
};

/**
 * @brief 异步保存管理器（单例）
 * 
 * 使用方式：
 *   1. 获取单例: AsyncSaveManager::instance()
 *   2. 提交任务: submitSaveAlignedPoints(...)
 *   3. 监听信号: taskCompleted, taskFailed
 *   4. 可选等待: waitForTask(taskId) 或 waitForAll()
 */
class AsyncSaveManager : public QObject
{
    Q_OBJECT

public:
    /**
     * @brief 获取单例
     */
    static AsyncSaveManager& instance();

    /**
     * @brief 设置最大并发任务数
     */
    void setMaxConcurrent(int maxConcurrent);
    int maxConcurrent() const { return m_maxConcurrent; }

    // ==================== 提交保存任务 ====================
    
    /**
     * @brief 提交对齐点数据保存任务
     * @return 任务ID
     */
    QString submitSaveAlignedPoints(const QString& filePath,
                                    const QVector<AlignedPoint>& points,
                                    const QString& blockName = QString(),
                                    int blockLineNumber = 0);

    /**
     * @brief 提交点云保存任务（CSV格式）
     */
    QString submitSavePointCloudCSV(const QString& filePath,
                                    const QVector<QVector3D>& points,
                                    const QString& blockName = QString(),
                                    int blockLineNumber = 0);

    /**
     * @brief 提交点云保存任务（PLY格式）
     */
    QString submitSavePointCloudPLY(const QString& filePath,
                                    const QVector<QVector3D>& points,
                                    const QString& blockName = QString(),
                                    int blockLineNumber = 0);

    /**
     * @brief 提交位置采样数据保存任务
     */
    QString submitSavePoseSamples(const QString& filePath,
                                  const QVector<SamplePoint>& samples,
                                  const QString& blockName = QString(),
                                  int blockLineNumber = 0);

    /**
     * @brief 提交紧凑扫描格式保存任务
     * @param filePath 文件路径
     * @param alignedPoints 对齐点数据
     * @param compression 压缩类型
     * @param startTimeUs 起始时间戳
     * @param sampleIntervalUs 采样间隔（微秒）
     * @param handEyeMatrix 手眼变换矩阵
     * @param scanName 扫描名称
     * @param blockName 程序段名称
     * @param blockLineNumber 行号
     * @return 任务ID
     */
    QString submitSaveCompactScan(const QString& filePath,
                                  const QVector<AlignedPoint>& alignedPoints,
                                  CompressionType compression = CompressionType::LZ4,
                                  int compressionLevel = 1,
                                  uint64_t startTimeUs = 0,
                                  uint32_t sampleIntervalUs = 1000,
                                  const QMatrix4x4& handEyeMatrix = QMatrix4x4(),
                                  const QString& scanName = QString(),
                                  const QString& blockName = QString(),
                                  int blockLineNumber = 0);

    /**
     * @brief 提交紧凑扫描格式保存任务（从Profile和Pose数据）
     */
    QString submitSaveCompactScanFromProfiles(const QString& filePath,
                                              const QVector<LaserProfile>& profiles,
                                              const QVector<SamplePoint>& poses,
                                              CompressionType compression = CompressionType::LZ4,
                                              int compressionLevel = 1,
                                              uint64_t startTimeUs = 0,
                                              uint32_t sampleIntervalUs = 1000,
                                              const QMatrix4x4& handEyeMatrix = QMatrix4x4(),
                                              const QString& scanName = QString(),
                                              const QString& blockName = QString(),
                                              int blockLineNumber = 0);

    /**
     * @brief 提交点云保存任务（PLY二进制格式）
     */
    QString submitSavePointCloudPLYBinary(const QString& filePath,
                                          const QVector<QVector3D>& points,
                                          const QString& blockName = QString(),
                                          int blockLineNumber = 0);

    // ==================== 任务管理 ====================

    /**
     * @brief 获取任务信息
     */
    SaveTaskInfo getTaskInfo(const QString& taskId) const;

    /**
     * @brief 获取所有活动任务
     */
    QList<SaveTaskInfo> getActiveTasks() const;

    /**
     * @brief 获取待执行任务数量
     */
    int pendingTaskCount() const;

    /**
     * @brief 获取正在执行的任务数量
     */
    int runningTaskCount() const;

    /**
     * @brief 取消任务
     */
    bool cancelTask(const QString& taskId);

    /**
     * @brief 取消所有任务
     */
    void cancelAllTasks();

    /**
     * @brief 等待特定任务完成（阻塞）
     */
    bool waitForTask(const QString& taskId, int timeoutMs = -1);

    /**
     * @brief 等待所有任务完成（阻塞）
     */
    bool waitForAll(int timeoutMs = -1);

    /**
     * @brief 是否有任务正在执行
     */
    bool isBusy() const;

    // ==================== 统计信息 ====================
    
    struct Statistics {
        qint64 totalTasksSubmitted = 0;
        qint64 totalTasksCompleted = 0;
        qint64 totalTasksFailed = 0;
        qint64 totalTasksCancelled = 0;
        qint64 totalBytesWritten = 0;
        qint64 totalPointsSaved = 0;
    };
    
    Statistics getStatistics() const;
    void resetStatistics();

signals:
    void taskStarted(const QString& taskId, const QString& filePath);
    void taskProgress(const QString& taskId, double progress);
    void taskCompleted(const QString& taskId, const QString& filePath, qint64 bytesWritten);
    void taskFailed(const QString& taskId, const QString& filePath, const QString& error);
    void allTasksCompleted();

private slots:
    void onTaskStarted(const QString& taskId);
    void onTaskProgressUpdated(const QString& taskId, double progress, qint64 bytesWritten);
    void onTaskCompleted(const QString& taskId, const QString& filePath, qint64 totalBytes);
    void onTaskFailed(const QString& taskId, const QString& filePath, const QString& errorMessage);

private:
    AsyncSaveManager(QObject* parent = nullptr);
    ~AsyncSaveManager() override;
    
    // 禁止拷贝
    AsyncSaveManager(const AsyncSaveManager&) = delete;
    AsyncSaveManager& operator=(const AsyncSaveManager&) = delete;

    QString generateTaskId();
    QString submitTask(SaveTask* task, const QString& blockName, int blockLineNumber);

private:
    QThreadPool* m_threadPool = nullptr;
    int m_maxConcurrent = 4;
    
    // 任务追踪
    mutable QMutex m_mutex;
    QHash<QString, SaveTaskInfo> m_taskInfoMap;
    QHash<QString, SaveTask*> m_activeTasks;
    
    // 统计
    Statistics m_stats;
    
    // 等待机制
    QWaitCondition m_waitCondition;
};
