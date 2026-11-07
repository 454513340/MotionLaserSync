#include "utils/GlobalTimeSync.h"

GlobalTimeSync& GlobalTimeSync::instance()
{
    static GlobalTimeSync s_instance;
    return s_instance;
}

void GlobalTimeSync::start()
{
    bool expected = false;
    if (m_started.compare_exchange_strong(expected, true)) {
        m_timer.start();
    }
}

qint64 GlobalTimeSync::timestampUs() const
{
    if (!m_started.load(std::memory_order_acquire)) {
        return 0;
    }
    return m_timer.nsecsElapsed() / 1000;  // ns -> µs
}

bool GlobalTimeSync::isStarted() const
{
    return m_started.load(std::memory_order_acquire);
}

void GlobalTimeSync::reset()
{
    m_started.store(false, std::memory_order_release);
}
