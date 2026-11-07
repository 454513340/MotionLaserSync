#include "utils/CsvExporter.h"
#include <QSaveFile>
#include "utils/AsyncSaveManager.h"
#include <exception>
#include <algorithm>
#include "utils/LogManager.h"
#include "utils/CompactScanFormat.h"

#include <QFile>
#include <QTextStream>
#include <QDataStream>
#include <QDir>
#include <QFileInfo>
#include <QThread>
#include <QCoreApplication>
#include <QDebug>

// ============================================================================
// SaveTask 实现
// ============================================================================

SaveTask::SaveTask(const QString& taskId, 
                   SaveTaskType type,
                   const QString& filePath,
                   QObject* parent)
    : QObject(parent)
    , m_taskId(taskId)
    , m_type(type)
    , m_filePath(filePath)
{
    setAutoDelete(false);  // 由 AsyncSaveManager 管理生命周期
}

void SaveTask::setAlignedPoints(const QVector<AlignedPoint>& points)
{
    m_alignedPoints = points;  // 深拷贝
}

void SaveTask::setPointCloud(const QVector<QVector3D>& points)
{
    m_cloudPoints = points;  // 深拷贝
}

void SaveTask::setPoseSamples(const QVector<SamplePoint>& samples)
{
    m_poseSamples = samples;  // 深拷贝
}

void SaveTask::setLaserProfiles(const QVector<LaserProfile>& profiles)
{
    m_laserProfiles = profiles;  // 深拷贝
}

void SaveTask::setCompactScanParams(uint64_t startTimeUs, 
                                    uint32_t sampleIntervalUs,
                                    const QMatrix4x4& handEyeMatrix)
{
    m_startTimeUs = startTimeUs;
    m_sampleIntervalUs = sampleIntervalUs;
    m_handEyeMatrix = handEyeMatrix;
}

void SaveTask::run()
{
    emit started(m_taskId);
    
    if (m_cancelled.load()) {
        emit failed(m_taskId, m_filePath, tr("任务已取消"));
        return;
    }

    // 确保目录存在
    QDir dir(QFileInfo(m_filePath).absolutePath());
    if (!dir.exists()) {
        if (!dir.mkpath(".")) {
            emit failed(m_taskId, m_filePath, 
                       tr("无法创建目录: %1").arg(dir.absolutePath()));
            return;
        }
    }

    bool success = false;
    
    try {
    switch (m_type) {
    case SaveTaskType::AlignedPoints:
        success = saveAlignedPointsCSV();
        break;
    case SaveTaskType::PointCloudCSV:
        success = savePointCloudCSV();
        break;
    case SaveTaskType::PointCloudPLY:
        success = savePointCloudPLY();
        break;
    case SaveTaskType::PointCloudPLYBinary:
        success = savePointCloudPLYBinary();
        break;
    case SaveTaskType::PoseSamples:
        success = savePoseSamplesCSV();
        break;
    case SaveTaskType::CompactScan:
        success = saveCompactScan(CompressionType::None);
        break;
    case SaveTaskType::CompactScanLZ4:
        success = saveCompactScan(CompressionType::LZ4);
        break;
    case SaveTaskType::CompactScanZSTD:
        success = saveCompactScan(CompressionType::ZSTD);
        break;
    default:
        emit failed(m_taskId, m_filePath, tr("未知的保存类型"));
        return;
    }

    } catch (const std::exception& error) {
        emit failed(m_taskId,m_filePath,QString::fromStdString(error.what()));
    }
    Q_UNUSED(success)  // 结果已在具体函数中处理
}

bool SaveTask::saveAlignedPointsCSV()
{
    QSaveFile file(m_filePath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        emit failed(m_taskId, m_filePath, 
                   tr("无法打开文件: %1").arg(file.errorString()));
        return false;
    }

    QTextStream out(&file);
    out.setRealNumberPrecision(17);
    
    // 写入表头
    CsvExporter::writeAlignmentHeader(out);
    
    const int total = m_alignedPoints.size();
    qint64 bytesWritten = 0;
    QElapsedTimer progressTimer;
    progressTimer.start();
    
    for (int i = 0; i < total; ++i) {
        if (m_cancelled.load()) {
            file.cancelWriting();
            emit failed(m_taskId, m_filePath, tr("任务已取消"));
            return false;
        }
        
        const auto& p = m_alignedPoints[i];
        CsvExporter::writeAlignmentRow(out, p);
        
        if (progressTimer.elapsed() >= kProgressUpdateIntervalMs) {
            bytesWritten = file.pos();
            double progress = static_cast<double>(i + 1) / total;
            emit progressUpdated(m_taskId, progress, bytesWritten);
            progressTimer.restart();
        }
    }
    
    out.flush();
    bytesWritten = file.size();
    if(out.status()!=QTextStream::Ok || !file.commit()){emit failed(m_taskId,m_filePath,file.errorString());return false;}
    
    emit completed(m_taskId, m_filePath, bytesWritten);
    return true;
}

bool SaveTask::savePointCloudCSV()
{
    QFile file(m_filePath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        emit failed(m_taskId, m_filePath,
                   tr("无法打开文件: %1").arg(file.errorString()));
        return false;
    }

    QTextStream out(&file);
    out.setRealNumberPrecision(6);
    
    out << "X,Y,Z\n";
    
    const int total = m_cloudPoints.size();
    qint64 bytesWritten = 0;
    QElapsedTimer progressTimer;
    progressTimer.start();
    
    for (int i = 0; i < total; ++i) {
        if (m_cancelled.load()) {
            file.close();
            QFile::remove(m_filePath);
            emit failed(m_taskId, m_filePath, tr("任务已取消"));
            return false;
        }
        
        const auto& p = m_cloudPoints[i];
        out << p.x() << ',' << p.y() << ',' << p.z() << '\n';
        
        if (progressTimer.elapsed() >= kProgressUpdateIntervalMs) {
            bytesWritten = file.pos();
            double progress = static_cast<double>(i + 1) / total;
            emit progressUpdated(m_taskId, progress, bytesWritten);
            progressTimer.restart();
        }
    }
    
    file.flush();
    bytesWritten = file.size();
    file.close();
    
    emit completed(m_taskId, m_filePath, bytesWritten);
    return true;
}

bool SaveTask::savePointCloudPLY()
{
    QFile file(m_filePath);
    if (!file.open(QIODevice::WriteOnly)) {
        emit failed(m_taskId, m_filePath,
                   tr("无法打开文件: %1").arg(file.errorString()));
        return false;
    }

    const int total = m_cloudPoints.size();
    
    // 写入 PLY 头
    QTextStream headerOut(&file);
    headerOut << "ply\n";
    headerOut << "format ascii 1.0\n";
    headerOut << "element vertex " << total << "\n";
    headerOut << "property float x\n";
    headerOut << "property float y\n";
    headerOut << "property float z\n";
    headerOut << "end_header\n";
    headerOut.flush();
    
    // 写入数据
    QTextStream out(&file);
    out.setRealNumberPrecision(6);
    
    qint64 bytesWritten = 0;
    QElapsedTimer progressTimer;
    progressTimer.start();
    
    for (int i = 0; i < total; ++i) {
        if (m_cancelled.load()) {
            file.close();
            QFile::remove(m_filePath);
            emit failed(m_taskId, m_filePath, tr("任务已取消"));
            return false;
        }
        
        const auto& p = m_cloudPoints[i];
        out << p.x() << ' ' << p.y() << ' ' << p.z() << '\n';
        
        if (progressTimer.elapsed() >= kProgressUpdateIntervalMs) {
            bytesWritten = file.pos();
            double progress = static_cast<double>(i + 1) / total;
            emit progressUpdated(m_taskId, progress, bytesWritten);
            progressTimer.restart();
        }
    }
    
    file.flush();
    bytesWritten = file.size();
    file.close();
    
    emit completed(m_taskId, m_filePath, bytesWritten);
    return true;
}

bool SaveTask::savePoseSamplesCSV()
{
    QSaveFile file(m_filePath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        emit failed(m_taskId, m_filePath,
                   tr("无法打开文件: %1").arg(file.errorString()));
        return false;
    }

    QTextStream out(&file);
    out.setRealNumberPrecision(17);
    
    CsvExporter::writePoseHeader(out);
    
    const int total = m_poseSamples.size();
    qint64 bytesWritten = 0;
    QElapsedTimer progressTimer;
    progressTimer.start();
    
    for (int i = 0; i < total; ++i) {
        if (m_cancelled.load()) {
            file.cancelWriting();
            emit failed(m_taskId, m_filePath, tr("任务已取消"));
            return false;
        }
        
        const auto& s = m_poseSamples[i];
        CsvExporter::writePoseRow(out, s);
        
        if (progressTimer.elapsed() >= kProgressUpdateIntervalMs) {
            bytesWritten = file.pos();
            double progress = static_cast<double>(i + 1) / total;
            emit progressUpdated(m_taskId, progress, bytesWritten);
            progressTimer.restart();
        }
    }
    
    out.flush();
    bytesWritten = file.size();
    if(out.status()!=QTextStream::Ok || !file.commit()){emit failed(m_taskId,m_filePath,file.errorString());return false;}
    
    emit completed(m_taskId, m_filePath, bytesWritten);
    return true;
}

bool SaveTask::savePointCloudPLYBinary()
{
    QFile file(m_filePath);
    if (!file.open(QIODevice::WriteOnly)) {
        emit failed(m_taskId, m_filePath,
                   tr("无法打开文件: %1").arg(file.errorString()));
        return false;
    }

    const int total = m_cloudPoints.size();
    
    // 写入 PLY 头（ASCII）
    QTextStream headerOut(&file);
    headerOut << "ply\n";
    headerOut << "format binary_little_endian 1.0\n";
    headerOut << "element vertex " << total << "\n";
    headerOut << "property float x\n";
    headerOut << "property float y\n";
    headerOut << "property float z\n";
    headerOut << "end_header\n";
    headerOut.flush();
    
    // 写入二进制数据
    QDataStream dataOut(&file);
    dataOut.setByteOrder(QDataStream::LittleEndian);
    dataOut.setFloatingPointPrecision(QDataStream::SinglePrecision);
    
    qint64 bytesWritten = 0;
    QElapsedTimer progressTimer;
    progressTimer.start();
    
    for (int i = 0; i < total; ++i) {
        if (m_cancelled.load()) {
            file.close();
            QFile::remove(m_filePath);
            emit failed(m_taskId, m_filePath, tr("任务已取消"));
            return false;
        }
        
        const auto& p = m_cloudPoints[i];
        dataOut << p.x() << p.y() << p.z();
        
        if (progressTimer.elapsed() >= kProgressUpdateIntervalMs) {
            bytesWritten = file.pos();
            double progress = static_cast<double>(i + 1) / total;
            emit progressUpdated(m_taskId, progress, bytesWritten);
            progressTimer.restart();
        }
    }
    
    file.flush();
    bytesWritten = file.size();
    file.close();
    
    emit completed(m_taskId, m_filePath, bytesWritten);
    return true;
}

bool SaveTask::saveCompactScan(CompressionType compression)
{
    // 根据数据来源创建ScanData
    ScanData scanData;
    
    if (!m_alignedPoints.isEmpty()) {
        // 从AlignedPoint数据创建
        scanData = CompactScanIO::fromAlignedPoints(
            m_alignedPoints,
            m_startTimeUs,
            m_sampleIntervalUs,
            m_handEyeMatrix
        );
    } else if (!m_laserProfiles.isEmpty()) {
        // 从Profile和Pose数据创建
        scanData = CompactScanIO::fromProfilesAndPoses(
            m_laserProfiles,
            m_poseSamples,
            m_startTimeUs,
            m_sampleIntervalUs,
            m_handEyeMatrix
        );
    } else {
        emit failed(m_taskId, m_filePath, tr("没有可保存的数据"));
        return false;
    }
    
    if (m_cancelled.load()) {
        emit failed(m_taskId, m_filePath, tr("任务已取消"));
        return false;
    }
    
    // 发送进度更新（数据转换完成 50%）
    emit progressUpdated(m_taskId, 0.5, 0);
    
    // 设置写入选项
    CompactScanIO::WriteOptions options;
    options.compression = compression;
    options.compressionLevel = m_compressionLevel;
    options.scanName = m_scanName;
    options.blockName = m_blockName;
    options.blockLineNumber = m_blockLineNumber;
    
    // 保存文件
    QString errorMsg;
    if (!CompactScanIO::save(m_filePath, scanData, options, &errorMsg)) {
        emit failed(m_taskId, m_filePath, errorMsg);
        return false;
    }
    
    // 获取文件大小
    QFileInfo fileInfo(m_filePath);
    qint64 bytesWritten = fileInfo.size();
    
    // 计算统计信息并记录日志
    auto stats = scanData.calculateStatistics();
    LOG_INFO(LogCategory::File, QString("紧凑扫描保存完成: %1\n"
                                        "  - 总点数: %2\n"
                                        "  - 原始大小: %3 MB\n"
                                        "  - 压缩后: %4 MB\n"
                                        "  - 压缩比: %5:1\n"
                                        "  - 节省: %6%")
             .arg(m_filePath)
             .arg(stats.totalPoints)
             .arg(stats.rawSizeBytes / (1024.0 * 1024.0), 0, 'f', 2)
             .arg(bytesWritten / (1024.0 * 1024.0), 0, 'f', 2)
             .arg(stats.rawSizeBytes > 0 ? static_cast<double>(stats.rawSizeBytes) / bytesWritten : 1.0, 0, 'f', 1)
             .arg(stats.savingsPercent, 0, 'f', 1));
    
    emit completed(m_taskId, m_filePath, bytesWritten);
    return true;
}

// ============================================================================
// AsyncSaveManager 实现
// ============================================================================

AsyncSaveManager& AsyncSaveManager::instance()
{
    static AsyncSaveManager s_instance;
    return s_instance;
}

AsyncSaveManager::AsyncSaveManager(QObject* parent)
    : QObject(parent)
{
    m_threadPool = new QThreadPool(this);
    
    // 默认使用 CPU 核心数，最少2个，最多8个
    int idealCount = QThread::idealThreadCount();
    m_maxConcurrent = qBound(2, idealCount, 8);
    m_threadPool->setMaxThreadCount(m_maxConcurrent);
    
    LOG_DEBUG(LogCategory::File, QString("AsyncSaveManager 初始化，最大并发数: %1").arg(m_maxConcurrent));
}

AsyncSaveManager::~AsyncSaveManager()
{
    cancelAllTasks();
    m_threadPool->waitForDone(5000);
    
    QMutexLocker locker(&m_mutex);
    qDeleteAll(m_activeTasks);
    m_activeTasks.clear();
}

void AsyncSaveManager::setMaxConcurrent(int maxConcurrent)
{
    m_maxConcurrent = qBound(1, maxConcurrent, 16);
    m_threadPool->setMaxThreadCount(m_maxConcurrent);
}

QString AsyncSaveManager::generateTaskId()
{
    return QUuid::createUuid().toString(QUuid::WithoutBraces).left(8);
}

QString AsyncSaveManager::submitTask(SaveTask* task, const QString& blockName, int blockLineNumber)
{
    QString taskId = task->taskId();
    
    // 连接信号
    connect(task, &SaveTask::started,
            this, &AsyncSaveManager::onTaskStarted, Qt::QueuedConnection);
    connect(task, &SaveTask::progressUpdated, 
            this, &AsyncSaveManager::onTaskProgressUpdated, Qt::QueuedConnection);
    connect(task, &SaveTask::completed, 
            this, &AsyncSaveManager::onTaskCompleted, Qt::QueuedConnection);
    connect(task, &SaveTask::failed, 
            this, &AsyncSaveManager::onTaskFailed, Qt::QueuedConnection);
    
    // 记录任务信息
    {
        QMutexLocker locker(&m_mutex);
        
        SaveTaskInfo info;
        info.taskId = taskId;
        info.filePath = task->filePath();
        info.status = SaveTaskStatus::Pending;
        info.submitTimeMs = QDateTime::currentMSecsSinceEpoch();
        info.blockName = blockName;
        info.blockLineNumber = blockLineNumber;
        
        m_taskInfoMap[taskId] = info;
        m_activeTasks[taskId] = task;
        
        m_stats.totalTasksSubmitted++;
    }
    
    // 提交到线程池
    m_threadPool->start(task);
    
    LOG_DEBUG(LogCategory::File, QString("提交保存任务: %1 文件: %2").arg(taskId).arg(task->filePath()));
    
    return taskId;
}

QString AsyncSaveManager::submitSaveAlignedPoints(const QString& filePath,
                                                   const QVector<AlignedPoint>& points,
                                                   const QString& blockName,
                                                   int blockLineNumber)
{
    QString taskId = generateTaskId();
    
    SaveTask* task = new SaveTask(taskId, SaveTaskType::AlignedPoints, filePath);
    task->setAlignedPoints(points);
    
    {
        QMutexLocker locker(&m_mutex);
        SaveTaskInfo& info = m_taskInfoMap[taskId];
        info.type = SaveTaskType::AlignedPoints;
        info.dataSize = points.size();
    }
    
    return submitTask(task, blockName, blockLineNumber);
}

QString AsyncSaveManager::submitSavePointCloudCSV(const QString& filePath,
                                                   const QVector<QVector3D>& points,
                                                   const QString& blockName,
                                                   int blockLineNumber)
{
    QString taskId = generateTaskId();
    
    SaveTask* task = new SaveTask(taskId, SaveTaskType::PointCloudCSV, filePath);
    task->setPointCloud(points);
    
    {
        QMutexLocker locker(&m_mutex);
        SaveTaskInfo& info = m_taskInfoMap[taskId];
        info.type = SaveTaskType::PointCloudCSV;
        info.dataSize = points.size();
    }
    
    return submitTask(task, blockName, blockLineNumber);
}

QString AsyncSaveManager::submitSavePointCloudPLY(const QString& filePath,
                                                   const QVector<QVector3D>& points,
                                                   const QString& blockName,
                                                   int blockLineNumber)
{
    QString taskId = generateTaskId();
    
    SaveTask* task = new SaveTask(taskId, SaveTaskType::PointCloudPLY, filePath);
    task->setPointCloud(points);
    
    {
        QMutexLocker locker(&m_mutex);
        SaveTaskInfo& info = m_taskInfoMap[taskId];
        info.type = SaveTaskType::PointCloudPLY;
        info.dataSize = points.size();
    }
    
    return submitTask(task, blockName, blockLineNumber);
}

QString AsyncSaveManager::submitSavePoseSamples(const QString& filePath,
                                                 const QVector<SamplePoint>& samples,
                                                 const QString& blockName,
                                                 int blockLineNumber)
{
    QString taskId = generateTaskId();
    
    SaveTask* task = new SaveTask(taskId, SaveTaskType::PoseSamples, filePath);
    task->setPoseSamples(samples);
    
    {
        QMutexLocker locker(&m_mutex);
        SaveTaskInfo& info = m_taskInfoMap[taskId];
        info.type = SaveTaskType::PoseSamples;
        info.dataSize = samples.size();
    }
    
    return submitTask(task, blockName, blockLineNumber);
}

QString AsyncSaveManager::submitSaveCompactScan(const QString& filePath,
                                                const QVector<AlignedPoint>& alignedPoints,
                                                CompressionType compression,
                                                int compressionLevel,
                                                uint64_t startTimeUs,
                                                uint32_t sampleIntervalUs,
                                                const QMatrix4x4& handEyeMatrix,
                                                const QString& scanName,
                                                const QString& blockName,
                                                int blockLineNumber)
{
    QString taskId = generateTaskId();
    
    SaveTaskType taskType;
    switch (compression) {
        case CompressionType::QtZlib:
        case CompressionType::LZ4:
            taskType = SaveTaskType::CompactScanLZ4;
            break;
        case CompressionType::ZSTD:
            taskType = SaveTaskType::CompactScanZSTD;
            break;
        default:
            taskType = SaveTaskType::CompactScan;
            break;
    }
    
    SaveTask* task = new SaveTask(taskId, taskType, filePath);
    task->setAlignedPoints(alignedPoints);
    task->setCompressionLevel(std::clamp(compressionLevel, 1, 9));
    task->setCompactScanParams(startTimeUs, sampleIntervalUs, handEyeMatrix);
    task->setScanName(scanName);
    task->setBlockInfo(blockName, blockLineNumber);
    
    {
        QMutexLocker locker(&m_mutex);
        SaveTaskInfo& info = m_taskInfoMap[taskId];
        info.type = taskType;
        info.dataSize = alignedPoints.size();
    }
    
    return submitTask(task, blockName, blockLineNumber);
}

QString AsyncSaveManager::submitSaveCompactScanFromProfiles(const QString& filePath,
                                                            const QVector<LaserProfile>& profiles,
                                                            const QVector<SamplePoint>& poses,
                                                            CompressionType compression,
                                                            int compressionLevel,
                                                            uint64_t startTimeUs,
                                                            uint32_t sampleIntervalUs,
                                                            const QMatrix4x4& handEyeMatrix,
                                                            const QString& scanName,
                                                            const QString& blockName,
                                                            int blockLineNumber)
{
    QString taskId = generateTaskId();
    
    SaveTaskType taskType;
    switch (compression) {
        case CompressionType::QtZlib:
        case CompressionType::LZ4:
            taskType = SaveTaskType::CompactScanLZ4;
            break;
        case CompressionType::ZSTD:
            taskType = SaveTaskType::CompactScanZSTD;
            break;
        default:
            taskType = SaveTaskType::CompactScan;
            break;
    }
    
    SaveTask* task = new SaveTask(taskId, taskType, filePath);
    task->setLaserProfiles(profiles);
    task->setPoseSamples(poses);
    task->setCompressionLevel(std::clamp(compressionLevel, 1, 9));
    task->setCompactScanParams(startTimeUs, sampleIntervalUs, handEyeMatrix);
    task->setScanName(scanName);
    task->setBlockInfo(blockName, blockLineNumber);
    
    {
        QMutexLocker locker(&m_mutex);
        SaveTaskInfo& info = m_taskInfoMap[taskId];
        info.type = taskType;
        info.dataSize = profiles.size();
    }
    
    return submitTask(task, blockName, blockLineNumber);
}

QString AsyncSaveManager::submitSavePointCloudPLYBinary(const QString& filePath,
                                                        const QVector<QVector3D>& points,
                                                        const QString& blockName,
                                                        int blockLineNumber)
{
    QString taskId = generateTaskId();
    
    SaveTask* task = new SaveTask(taskId, SaveTaskType::PointCloudPLYBinary, filePath);
    task->setPointCloud(points);
    
    {
        QMutexLocker locker(&m_mutex);
        SaveTaskInfo& info = m_taskInfoMap[taskId];
        info.type = SaveTaskType::PointCloudPLYBinary;
        info.dataSize = points.size();
    }
    
    return submitTask(task, blockName, blockLineNumber);
}

void AsyncSaveManager::onTaskStarted(const QString& taskId)
{
    QString filePath;
    {
        QMutexLocker locker(&m_mutex);
        if (m_taskInfoMap.contains(taskId)) {
            m_taskInfoMap[taskId].status = SaveTaskStatus::Running;
            m_taskInfoMap[taskId].startTimeMs = QDateTime::currentMSecsSinceEpoch();
            filePath = m_taskInfoMap[taskId].filePath;
        }
    }
    
    emit taskStarted(taskId, filePath);
}

void AsyncSaveManager::onTaskProgressUpdated(const QString& taskId, 
                                              double progress, 
                                              qint64 bytesWritten)
{
    {
        QMutexLocker locker(&m_mutex);
        if (m_taskInfoMap.contains(taskId)) {
            m_taskInfoMap[taskId].progress = progress;
            m_taskInfoMap[taskId].bytesWritten = bytesWritten;
        }
    }
    
    emit taskProgress(taskId, progress);
}

void AsyncSaveManager::onTaskCompleted(const QString& taskId, 
                                        const QString& filePath, 
                                        qint64 totalBytes)
{
    SaveTask* task = nullptr;
    qint64 dataSize = 0;
    
    {
        QMutexLocker locker(&m_mutex);
        
        if (m_taskInfoMap.contains(taskId)) {
            m_taskInfoMap[taskId].status = SaveTaskStatus::Completed;
            m_taskInfoMap[taskId].bytesWritten = totalBytes;
            m_taskInfoMap[taskId].progress = 1.0;
            m_taskInfoMap[taskId].endTimeMs = QDateTime::currentMSecsSinceEpoch();
            dataSize = m_taskInfoMap[taskId].dataSize;
        }
        
        if (m_activeTasks.contains(taskId)) {
            task = m_activeTasks.take(taskId);
        }
        
        m_stats.totalTasksCompleted++;
        m_stats.totalBytesWritten += totalBytes;
        m_stats.totalPointsSaved += dataSize;
        
        m_waitCondition.wakeAll();
    }
    
    if (task) {
        task->deleteLater();
    }
    
    LOG_INFO(LogCategory::File, QString("保存任务完成: %1 文件: %2 大小: %3 bytes").arg(taskId).arg(filePath).arg(totalBytes));
    
    emit taskCompleted(taskId, filePath, totalBytes);
    
    if (!isBusy()) {
        emit allTasksCompleted();
    }
}

void AsyncSaveManager::onTaskFailed(const QString& taskId, 
                                     const QString& filePath,
                                     const QString& errorMessage)
{
    SaveTask* task = nullptr;
    
    {
        QMutexLocker locker(&m_mutex);
        
        if (m_taskInfoMap.contains(taskId)) {
            m_taskInfoMap[taskId].status = SaveTaskStatus::Failed;
            m_taskInfoMap[taskId].errorMessage = errorMessage;
            m_taskInfoMap[taskId].endTimeMs = QDateTime::currentMSecsSinceEpoch();
        }
        
        if (m_activeTasks.contains(taskId)) {
            task = m_activeTasks.take(taskId);
        }
        
        m_stats.totalTasksFailed++;
        
        m_waitCondition.wakeAll();
    }
    
    if (task) {
        task->deleteLater();
    }
    
    LOG_ERROR(LogCategory::File, QString("保存任务失败: %1 错误: %2").arg(taskId).arg(errorMessage));
    
    emit taskFailed(taskId, filePath, errorMessage);
    
    if (!isBusy()) {
        emit allTasksCompleted();
    }
}

SaveTaskInfo AsyncSaveManager::getTaskInfo(const QString& taskId) const
{
    QMutexLocker locker(&m_mutex);
    return m_taskInfoMap.value(taskId);
}

QList<SaveTaskInfo> AsyncSaveManager::getActiveTasks() const
{
    QMutexLocker locker(&m_mutex);
    QList<SaveTaskInfo> result;
    for (const auto& taskId : m_activeTasks.keys()) {
        if (m_taskInfoMap.contains(taskId)) {
            result.append(m_taskInfoMap[taskId]);
        }
    }
    return result;
}

int AsyncSaveManager::pendingTaskCount() const
{
    QMutexLocker locker(&m_mutex);
    int count = 0;
    for (const auto& info : m_taskInfoMap) {
        if (info.status == SaveTaskStatus::Pending) {
            count++;
        }
    }
    return count;
}

int AsyncSaveManager::runningTaskCount() const
{
    QMutexLocker locker(&m_mutex);
    return m_activeTasks.size();
}

bool AsyncSaveManager::cancelTask(const QString& taskId)
{
    QMutexLocker locker(&m_mutex);
    
    if (m_activeTasks.contains(taskId)) {
        m_activeTasks[taskId]->cancel();
        if (m_taskInfoMap.contains(taskId)) {
            m_taskInfoMap[taskId].status = SaveTaskStatus::Cancelled;
        }
        m_stats.totalTasksCancelled++;
        return true;
    }
    return false;
}

void AsyncSaveManager::cancelAllTasks()
{
    QMutexLocker locker(&m_mutex);
    
    for (auto* task : m_activeTasks) {
        task->cancel();
    }
    m_stats.totalTasksCancelled += m_activeTasks.size();
}

bool AsyncSaveManager::waitForTask(const QString& taskId, int timeoutMs)
{
    QMutexLocker locker(&m_mutex);
    
    QElapsedTimer timer;
    timer.start();
    
    while (m_activeTasks.contains(taskId)) {
        if (timeoutMs >= 0 && timer.elapsed() >= timeoutMs) {
            return false;
        }
        
        int waitTime = timeoutMs < 0 ? 100 : qMin(100, timeoutMs - static_cast<int>(timer.elapsed()));
        if (waitTime > 0) {
            m_waitCondition.wait(&m_mutex, static_cast<unsigned long>(waitTime));
        }
    }
    
    if (m_taskInfoMap.contains(taskId)) {
        return m_taskInfoMap[taskId].status == SaveTaskStatus::Completed;
    }
    return false;
}

bool AsyncSaveManager::waitForAll(int timeoutMs)
{
    QMutexLocker locker(&m_mutex);
    
    QElapsedTimer timer;
    timer.start();
    
    while (!m_activeTasks.isEmpty()) {
        if (timeoutMs >= 0 && timer.elapsed() >= timeoutMs) {
            return false;
        }
        
        int waitTime = timeoutMs < 0 ? 100 : qMin(100, timeoutMs - static_cast<int>(timer.elapsed()));
        if (waitTime > 0) {
            m_waitCondition.wait(&m_mutex, static_cast<unsigned long>(waitTime));
        }
    }
    
    return true;
}

bool AsyncSaveManager::isBusy() const
{
    QMutexLocker locker(&m_mutex);
    return !m_activeTasks.isEmpty();
}

AsyncSaveManager::Statistics AsyncSaveManager::getStatistics() const
{
    QMutexLocker locker(&m_mutex);
    return m_stats;
}

void AsyncSaveManager::resetStatistics()
{
    QMutexLocker locker(&m_mutex);
    m_stats = Statistics();
}
