#pragma once

#include <QObject>
#include <QString>
#include <QDateTime>
#include <QMutex>
#include <QFile>
#include <QTextStream>
#include <QQueue>
#include <memory>

/**
 * @file LogManager.h
 * @brief 日志管理系统
 * 
 * 提供统一的日志记录、显示和管理功能。
 * 支持多级别日志、文件输出、界面显示、日志过滤等功能。
 */

// ==================== 日志级别 ====================
enum class LogLevel {
    Debug = 0,      // 调试信息
    Info = 1,       // 一般信息
    Warning = 2,    // 警告
    Error = 3,      // 错误
    Fatal = 4       // 致命错误
};

// ==================== 日志条目 ====================
struct LogEntry {
    QDateTime timestamp;                    // 时间戳
    LogLevel level = LogLevel::Info;        // 日志级别（默认Info）
    QString category;                       // 分类（模块名）
    QString message;                        // 日志内容
    QString file;                           // 源文件（可选）
    int line = 0;                           // 行号（可选）
    QString function;                       // 函数名（可选）
    
    // 默认构造函数
    LogEntry() = default;
    
    // 带参数构造函数
    LogEntry(LogLevel lvl, const QString& cat, const QString& msg)
        : timestamp(QDateTime::currentDateTime())
        , level(lvl)
        , category(cat)
        , message(msg)
    {}
    
    // 格式化输出
    QString toString(bool includeSource = false) const;
    QString toHtml() const;
    
    // 获取级别名称
    static QString levelName(LogLevel level);
    static QString levelColor(LogLevel level);
    static QString levelIcon(LogLevel level);
};

// ==================== 日志管理器（单例） ====================
class LogManager : public QObject
{
    Q_OBJECT

public:
    /**
     * @brief 获取单例实例
     */
    static LogManager& instance();
    
    /**
     * @brief 初始化日志系统
     * @param logDir 日志文件目录
     * @param logFilePrefix 日志文件前缀
     * @param maxFileSize 单个日志文件最大大小（字节）
     * @param maxFileCount 保留的日志文件数量
     */
    void initialize(const QString& logDir = "./logs",
                   const QString& logFilePrefix = "app",
                   qint64 maxFileSize = 10 * 1024 * 1024,  // 10MB
                   int maxFileCount = 10);
    
    /**
     * @brief 关闭日志系统
     */
    void shutdown();
    
    // ===== 日志记录 =====
    
    /**
     * @brief 记录日志
     */
    void log(LogLevel level, const QString& category, const QString& message,
             const QString& file = QString(), int line = 0, 
             const QString& function = QString());
    
    // 便捷方法
    void debug(const QString& category, const QString& message);
    void info(const QString& category, const QString& message);
    void warning(const QString& category, const QString& message);
    void error(const QString& category, const QString& message);
    void fatal(const QString& category, const QString& message);
    
    // ===== 配置 =====
    
    /**
     * @brief 设置最小日志级别（低于此级别的日志不记录）
     */
    void setMinLevel(LogLevel level);
    LogLevel minLevel() const { return m_minLevel; }
    
    /**
     * @brief 设置是否输出到控制台
     */
    void setConsoleOutput(bool enable);
    bool consoleOutput() const { return m_consoleOutput; }
    
    /**
     * @brief 设置是否输出到文件
     */
    void setFileOutput(bool enable);
    bool fileOutput() const { return m_fileOutput; }
    
    /**
     * @brief 设置分类过滤（空列表表示不过滤）
     */
    void setCategoryFilter(const QStringList& categories);
    QStringList categoryFilter() const { return m_categoryFilter; }
    
    // ===== 日志查询 =====
    
    /**
     * @brief 获取最近的日志条目
     * @param count 数量（0表示全部）
     * @param level 最小级别
     */
    QList<LogEntry> recentLogs(int count = 0, LogLevel level = LogLevel::Debug) const;
    
    /**
     * @brief 获取日志文件路径
     */
    QString currentLogFile() const;
    
    /**
     * @brief 获取所有日志文件
     */
    QStringList logFiles() const;
    
    /**
     * @brief 清空内存中的日志
     */
    void clearMemoryLogs();
    
    /**
     * @brief 导出日志到文件
     */
    bool exportLogs(const QString& filePath, LogLevel minLevel = LogLevel::Debug,
                   const QDateTime& startTime = QDateTime(),
                   const QDateTime& endTime = QDateTime()) const;

signals:
    /**
     * @brief 新日志条目信号
     */
    void logAdded(const LogEntry& entry);
    
    /**
     * @brief 日志清空信号
     */
    void logsCleared();

private:
    LogManager();
    ~LogManager();
    LogManager(const LogManager&) = delete;
    LogManager& operator=(const LogManager&) = delete;
    
    void writeToFile(const LogEntry& entry);
    void writeToConsole(const LogEntry& entry);
    void rotateLogFile();
    void cleanOldLogFiles();
    QString generateLogFileName() const;
    
private:
    mutable QMutex m_mutex;
    
    // 配置
    QString m_logDir;
    QString m_logFilePrefix;
    qint64 m_maxFileSize = 10 * 1024 * 1024;
    int m_maxFileCount = 10;
    LogLevel m_minLevel = LogLevel::Debug;
    bool m_consoleOutput = true;
    bool m_fileOutput = true;
    QStringList m_categoryFilter;
    
    // 文件输出
    std::unique_ptr<QFile> m_logFile;
    std::unique_ptr<QTextStream> m_logStream;
    QString m_currentLogFileName;
    
    // 内存缓存
    QList<LogEntry> m_logCache;
    int m_maxCacheSize = 10000;
    
    // 状态
    bool m_initialized = false;
};

// ==================== 便捷宏 ====================

// 带源文件信息的日志宏
#define LOG_DEBUG(category, message) \
    LogManager::instance().log(LogLevel::Debug, category, message, __FILE__, __LINE__, __FUNCTION__)

#define LOG_INFO(category, message) \
    LogManager::instance().log(LogLevel::Info, category, message, __FILE__, __LINE__, __FUNCTION__)

#define LOG_WARNING(category, message) \
    LogManager::instance().log(LogLevel::Warning, category, message, __FILE__, __LINE__, __FUNCTION__)

#define LOG_ERROR(category, message) \
    LogManager::instance().log(LogLevel::Error, category, message, __FILE__, __LINE__, __FUNCTION__)

#define LOG_FATAL(category, message) \
    LogManager::instance().log(LogLevel::Fatal, category, message, __FILE__, __LINE__, __FUNCTION__)

// 简化宏（不带源文件信息）
#define LOG_D(category, message) LogManager::instance().debug(category, message)
#define LOG_I(category, message) LogManager::instance().info(category, message)
#define LOG_W(category, message) LogManager::instance().warning(category, message)
#define LOG_E(category, message) LogManager::instance().error(category, message)
#define LOG_F(category, message) LogManager::instance().fatal(category, message)

// 常用分类定义
namespace LogCategory {
    constexpr const char* System = "System";
    constexpr const char* Controller = "Controller";
    constexpr const char* Sensor = "Sensor";
    constexpr const char* Sync = "Sync";
    constexpr const char* GCode = "GCode";
    constexpr const char* PointCloud = "PointCloud";
    constexpr const char* Calibration = "Calibration";
    constexpr const char* UI = "UI";
    constexpr const char* File = "File";
}

// 注册元类型（用于信号槽传递）
Q_DECLARE_METATYPE(LogEntry)
Q_DECLARE_METATYPE(LogLevel)
