#pragma once

#include <QElapsedTimer>
#include <atomic>
#include <cstdint>

/**
 * @brief 全局时间同步单例
 * 
 * 提供统一的时间基准，用于激光轮廓和机床位置的时间对齐。
 * 使用高精度计时器，返回微秒级时间戳。
 */
class GlobalTimeSync {
public:
    /**
     * @brief 获取单例实例
     */
    static GlobalTimeSync& instance();

    /**
     * @brief 启动计时器（只在第一次调用时生效）
     */
    void start();

    /**
     * @brief 获取当前时间戳（微秒）
     * @return 如果计时器未启动返回 0，否则返回从启动到现在的微秒数
     */
    qint64 timestampUs() const;

    /**
     * @brief 检查计时器是否已启动
     */
    bool isStarted() const;

    /**
     * @brief 重置计时器（用于测试或重新开始采集）
     */
    void reset();

private:
    GlobalTimeSync() = default;
    ~GlobalTimeSync() = default;

    // 禁止拷贝和移动
    GlobalTimeSync(const GlobalTimeSync&) = delete;
    GlobalTimeSync& operator=(const GlobalTimeSync&) = delete;
    GlobalTimeSync(GlobalTimeSync&&) = delete;
    GlobalTimeSync& operator=(GlobalTimeSync&&) = delete;

private:
    QElapsedTimer m_timer;
    std::atomic<bool> m_started{false};
    mutable std::atomic<bool> m_resetPending{false};
};

// 便捷函数（兼容旧代码）
inline void Zmc_GlobalSyncTimerStart() {
    GlobalTimeSync::instance().start();
}

inline qint64 Zmc_GlobalSyncTimestampUs() {
    return GlobalTimeSync::instance().timestampUs();
}
