#include "utils/LogManager.h"

#include <QDir>
#include <QFileInfo>
#include <QDebug>
#include <QCoreApplication>
#include <iostream>

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
#include <QStringConverter>
#endif

// ==================== LogEntry 实现 ====================

QString LogEntry::levelName(LogLevel level)
{
    switch (level) {
    case LogLevel::Debug:   return "DEBUG";
    case LogLevel::Info:    return "INFO";
    case LogLevel::Warning: return "WARN";
    case LogLevel::Error:   return "ERROR";
    case LogLevel::Fatal:   return "FATAL";
    default:                return "???";
    }
}

QString LogEntry::levelColor(LogLevel level)
{
    switch (level) {
    case LogLevel::Debug:   return "#888888";  // 灰色
    case LogLevel::Info:    return "#000000";  // 黑色
    case LogLevel::Warning: return "#FF8C00";  // 橙色
    case LogLevel::Error:   return "#FF0000";  // 红色
    case LogLevel::Fatal:   return "#8B0000";  // 深红色
    default:                return "#000000";
    }
}

QString LogEntry::levelIcon(LogLevel level)
{
    switch (level) {
    case LogLevel::Debug:   return "[D]";
    case LogLevel::Info:    return "[I]";
    case LogLevel::Warning: return "[W]";
    case LogLevel::Error:   return "[E]";
    case LogLevel::Fatal:   return "[F]";
    default:                return "[?]";
    }
}

QString LogEntry::toString(bool includeSource) const
{
    QString result = QString("[%1] [%2] [%3] %4")
        .arg(timestamp.toString("yyyy-MM-dd hh:mm:ss.zzz"))
        .arg(levelName(level), -5)
        .arg(category, -12)
        .arg(message);
    
    if (includeSource && !file.isEmpty()) {
        QFileInfo fi(file);
        result += QString(" (%1:%2 %3)")
            .arg(fi.fileName())
            .arg(line)
            .arg(function);
    }
    
    return result;
}

QString LogEntry::toHtml() const
{
    QString color = levelColor(level);
    QString levelStr = levelName(level);
    QString timeStr = timestamp.toString("hh:mm:ss.zzz");
    
    // 转义HTML特殊字符
    QString escapedMsg = message;
    escapedMsg.replace("&", "&amp;");
    escapedMsg.replace("<", "&lt;");
    escapedMsg.replace(">", "&gt;");
    escapedMsg.replace("\n", "<br>");
    
    QString html = QString(
        "<span style='color:#666666;'>[%1]</span> "
        "<span style='color:%2; font-weight:bold;'>[%3]</span> "
        "<span style='color:#0066CC;'>[%4]</span> "
        "<span style='color:#333333;'>%5</span>")
        .arg(timeStr)
        .arg(color)
        .arg(levelStr, -5)
        .arg(category, -12)
        .arg(escapedMsg);
    
    return html;
}

// ==================== LogManager 实现 ====================

LogManager& LogManager::instance()
{
    static LogManager s_instance;
    return s_instance;
}

LogManager::LogManager()
    : QObject(nullptr)
{
}

LogManager::~LogManager()
{
    shutdown();
}

void LogManager::initialize(const QString& logDir,
                           const QString& logFilePrefix,
                           qint64 maxFileSize,
                           int maxFileCount)
{
    QMutexLocker locker(&m_mutex);
    
    if (m_initialized) {
        return;
    }
    
    m_logDir = logDir;
    m_logFilePrefix = logFilePrefix;
    m_maxFileSize = maxFileSize;
    m_maxFileCount = maxFileCount;
    
    // 创建日志目录
    QDir dir(m_logDir);
    if (!dir.exists()) {
        dir.mkpath(".");
    }
    
    // 创建日志文件
    if (m_fileOutput) {
        m_currentLogFileName = generateLogFileName();
        m_logFile = std::make_unique<QFile>(m_currentLogFileName);
        if (m_logFile->open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text)) {
            m_logStream = std::make_unique<QTextStream>(m_logFile.get());
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
            m_logStream->setEncoding(QStringConverter::Utf8);
#else
            m_logStream->setCodec("UTF-8");
#endif
        } else {
            m_logFile.reset();
            qWarning() << "LogManager: Failed to open log file:" << m_currentLogFileName;
        }
    }
    
    // 清理旧日志文件
    cleanOldLogFiles();
    
    m_initialized = true;
    
    // 不在初始化过程中记录日志，避免信号问题
    // 日志系统启动信息可以在MainWindow完全初始化后再记录
}

void LogManager::shutdown()
{
    QMutexLocker locker(&m_mutex);
    
    if (!m_initialized) {
        return;
    }
    
    // 记录关闭日志
    if (m_logStream) {
        LogEntry entry;
        entry.timestamp = QDateTime::currentDateTime();
        entry.level = LogLevel::Info;
        entry.category = LogCategory::System;
        entry.message = "日志系统关闭";
        
        *m_logStream << entry.toString() << "\n";
        m_logStream->flush();
    }
    
    m_logStream.reset();
    m_logFile.reset();
    m_initialized = false;
}

void LogManager::log(LogLevel level, const QString& category, const QString& message,
                    const QString& file, int line, const QString& function)
{
    // 级别过滤
    if (level < m_minLevel) {
        return;
    }
    
    // 分类过滤
    if (!m_categoryFilter.isEmpty() && !m_categoryFilter.contains(category)) {
        return;
    }
    
    LogEntry entry;
    entry.timestamp = QDateTime::currentDateTime();
    entry.level = level;
    entry.category = category;
    entry.message = message;
    entry.file = file;
    entry.line = line;
    entry.function = function;
    
    {
        QMutexLocker locker(&m_mutex);
        
        // 添加到缓存
        m_logCache.append(entry);
        while (m_logCache.size() > m_maxCacheSize) {
            m_logCache.removeFirst();
        }
        
        // 输出到文件
        if (m_fileOutput && m_logStream) {
            writeToFile(entry);
        }
        
        // 输出到控制台
        if (m_consoleOutput) {
            writeToConsole(entry);
        }
    }
    
    // 发送信号（在锁外发送）
    emit logAdded(entry);
}

void LogManager::debug(const QString& category, const QString& message)
{
    log(LogLevel::Debug, category, message);
}

void LogManager::info(const QString& category, const QString& message)
{
    log(LogLevel::Info, category, message);
}

void LogManager::warning(const QString& category, const QString& message)
{
    log(LogLevel::Warning, category, message);
}

void LogManager::error(const QString& category, const QString& message)
{
    log(LogLevel::Error, category, message);
}

void LogManager::fatal(const QString& category, const QString& message)
{
    log(LogLevel::Fatal, category, message);
}

void LogManager::setMinLevel(LogLevel level)
{
    QMutexLocker locker(&m_mutex);
    m_minLevel = level;
}

void LogManager::setConsoleOutput(bool enable)
{
    QMutexLocker locker(&m_mutex);
    m_consoleOutput = enable;
}

void LogManager::setFileOutput(bool enable)
{
    QMutexLocker locker(&m_mutex);
    m_fileOutput = enable;
    
    if (enable && !m_logFile && m_initialized) {
        m_currentLogFileName = generateLogFileName();
        m_logFile = std::make_unique<QFile>(m_currentLogFileName);
        if (m_logFile->open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text)) {
            m_logStream = std::make_unique<QTextStream>(m_logFile.get());
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
            m_logStream->setEncoding(QStringConverter::Utf8);
#else
            m_logStream->setCodec("UTF-8");
#endif
        }
    } else if (!enable) {
        m_logStream.reset();
        m_logFile.reset();
    }
}

void LogManager::setCategoryFilter(const QStringList& categories)
{
    QMutexLocker locker(&m_mutex);
    m_categoryFilter = categories;
}

QList<LogEntry> LogManager::recentLogs(int count, LogLevel level) const
{
    QMutexLocker locker(&m_mutex);
    
    QList<LogEntry> result;
    
    for (int i = m_logCache.size() - 1; i >= 0; --i) {
        if (m_logCache[i].level >= level) {
            result.prepend(m_logCache[i]);
            if (count > 0 && result.size() >= count) {
                break;
            }
        }
    }
    
    return result;
}

QString LogManager::currentLogFile() const
{
    QMutexLocker locker(&m_mutex);
    return m_currentLogFileName;
}

QStringList LogManager::logFiles() const
{
    QMutexLocker locker(&m_mutex);
    
    QDir dir(m_logDir);
    QStringList filters;
    filters << QString("%1_*.log").arg(m_logFilePrefix);
    
    QStringList files = dir.entryList(filters, QDir::Files, QDir::Name | QDir::Reversed);
    
    QStringList result;
    for (const QString& f : files) {
        result.append(dir.absoluteFilePath(f));
    }
    
    return result;
}

void LogManager::clearMemoryLogs()
{
    QMutexLocker locker(&m_mutex);
    m_logCache.clear();
    
    emit logsCleared();
}

bool LogManager::exportLogs(const QString& filePath, LogLevel minLevel,
                           const QDateTime& startTime, const QDateTime& endTime) const
{
    QMutexLocker locker(&m_mutex);
    
    QFile file(filePath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        return false;
    }
    
    QTextStream out(&file);
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    out.setEncoding(QStringConverter::Utf8);
#else
    out.setCodec("UTF-8");
#endif
    
    // 写入头部
    out << "# 日志导出\n";
    out << "# 导出时间: " << QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss") << "\n";
    out << "# 最小级别: " << LogEntry::levelName(minLevel) << "\n";
    if (startTime.isValid()) {
        out << "# 开始时间: " << startTime.toString("yyyy-MM-dd hh:mm:ss") << "\n";
    }
    if (endTime.isValid()) {
        out << "# 结束时间: " << endTime.toString("yyyy-MM-dd hh:mm:ss") << "\n";
    }
    out << "#\n\n";
    
    // 写入日志
    for (const LogEntry& entry : m_logCache) {
        if (entry.level < minLevel) continue;
        if (startTime.isValid() && entry.timestamp < startTime) continue;
        if (endTime.isValid() && entry.timestamp > endTime) continue;
        
        out << entry.toString(true) << "\n";
    }
    
    file.close();
    return true;
}

void LogManager::writeToFile(const LogEntry& entry)
{
    if (!m_logStream) return;
    
    *m_logStream << entry.toString(true) << "\n";
    m_logStream->flush();
    
    // 检查文件大小，需要滚动
    if (m_logFile && m_logFile->size() > m_maxFileSize) {
        rotateLogFile();
    }
}

void LogManager::writeToConsole(const LogEntry& entry)
{
    // 使用 qDebug 输出，可以被 Qt 的消息处理器捕获
    QString msg = entry.toString();
    
    switch (entry.level) {
    case LogLevel::Debug:
        qDebug().noquote() << msg;
        break;
    case LogLevel::Info:
        qInfo().noquote() << msg;
        break;
    case LogLevel::Warning:
        qWarning().noquote() << msg;
        break;
    case LogLevel::Error:
    case LogLevel::Fatal:
        qCritical().noquote() << msg;
        break;
    }
}

void LogManager::rotateLogFile()
{
    if (!m_logFile) return;
    
    m_logStream.reset();
    m_logFile->close();
    
    // 生成新文件名
    m_currentLogFileName = generateLogFileName();
    m_logFile = std::make_unique<QFile>(m_currentLogFileName);
    
    if (m_logFile->open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text)) {
        m_logStream = std::make_unique<QTextStream>(m_logFile.get());
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
        m_logStream->setEncoding(QStringConverter::Utf8);
#else
        m_logStream->setCodec("UTF-8");
#endif
    }
    
    // 清理旧文件
    cleanOldLogFiles();
}

void LogManager::cleanOldLogFiles()
{
    QDir dir(m_logDir);
    QStringList filters;
    filters << QString("%1_*.log").arg(m_logFilePrefix);
    
    QStringList files = dir.entryList(filters, QDir::Files, QDir::Time);
    
    // 删除超出数量的旧文件
    while (files.size() > m_maxFileCount) {
        QString oldest = files.takeLast();
        QFile::remove(dir.absoluteFilePath(oldest));
    }
}

QString LogManager::generateLogFileName() const
{
    QString timestamp = QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss");
    return QString("%1/%2_%3.log")
        .arg(m_logDir)
        .arg(m_logFilePrefix)
        .arg(timestamp);
}
