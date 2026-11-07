#pragma once

/**
 * @file IAlgorithm.h
 * @brief 算法接口基类
 * 
 * 定义所有算法模块的通用接口，便于统一管理和扩展
 */

#include <QObject>
#include <QString>
#include <functional>
#include "AlgorithmTypes.h"

/**
 * @brief 算法接口基类
 * 
 * 所有算法模块继承此接口，实现统一的执行流程
 */
class IAlgorithm : public QObject
{
    Q_OBJECT

public:
    explicit IAlgorithm(QObject* parent = nullptr) : QObject(parent) {}
    virtual ~IAlgorithm() = default;

    /**
     * @brief 获取算法名称
     */
    virtual QString name() const = 0;

    /**
     * @brief 获取算法描述
     */
    virtual QString description() const = 0;

    /**
     * @brief 获取算法版本
     */
    virtual QString version() const { return "1.0.0"; }

    /**
     * @brief 初始化算法
     * @return 是否成功
     */
    virtual bool initialize() = 0;

    /**
     * @brief 执行算法
     * @return 是否成功
     */
    virtual bool execute() = 0;

    /**
     * @brief 取消执行
     */
    virtual void cancel() { m_cancelled = true; }

    /**
     * @brief 是否已取消
     */
    bool isCancelled() const { return m_cancelled; }

    /**
     * @brief 是否正在运行
     */
    bool isRunning() const { return m_running; }

    /**
     * @brief 获取最后的错误信息
     */
    QString lastError() const { return m_lastError; }

signals:
    /**
     * @brief 进度更新信号
     */
    void progressUpdated(const AlgorithmProgress& progress);

    /**
     * @brief 算法完成信号
     */
    void completed(bool success);

    /**
     * @brief 错误发生信号
     */
    void errorOccurred(const QString& message);

    /**
     * @brief 警告信号
     */
    void warningOccurred(const QString& message);

    /**
     * @brief 日志消息信号
     */
    void logMessage(const QString& message);

protected:
    /**
     * @brief 报告进度
     */
    void reportProgress(int current, int total, const QString& message = QString()) {
        AlgorithmProgress progress;
        progress.current = current;
        progress.total = total;
        progress.message = message;
        emit progressUpdated(progress);
    }

    /**
     * @brief 设置错误信息
     */
    void setError(const QString& error) {
        m_lastError = error;
        emit errorOccurred(error);
    }

    /**
     * @brief 发送日志
     */
    void log(const QString& message) {
        emit logMessage(message);
    }

protected:
    bool m_cancelled = false;
    bool m_running = false;
    QString m_lastError;
};

/**
 * @brief 算法工厂基类
 * 
 * 用于创建和管理算法实例
 */
class AlgorithmFactory
{
public:
    using Creator = std::function<IAlgorithm*(QObject*)>;
    
    static AlgorithmFactory& instance() {
        static AlgorithmFactory factory;
        return factory;
    }

    /**
     * @brief 注册算法
     */
    void registerAlgorithm(const QString& id, Creator creator) {
        m_creators[id] = creator;
    }

    /**
     * @brief 创建算法实例
     */
    IAlgorithm* create(const QString& id, QObject* parent = nullptr) {
        if (m_creators.contains(id)) {
            return m_creators[id](parent);
        }
        return nullptr;
    }

    /**
     * @brief 获取所有已注册算法ID
     */
    QStringList registeredAlgorithms() const {
        return m_creators.keys();
    }

private:
    AlgorithmFactory() = default;
    QMap<QString, Creator> m_creators;
};

/**
 * @brief 算法注册辅助宏
 */
#define REGISTER_ALGORITHM(AlgorithmClass) \
    namespace { \
        static bool _registered_##AlgorithmClass = []() { \
            AlgorithmFactory::instance().registerAlgorithm( \
                #AlgorithmClass, \
                [](QObject* parent) -> IAlgorithm* { return new AlgorithmClass(parent); } \
            ); \
            return true; \
        }(); \
    }
