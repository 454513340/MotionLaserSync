#pragma once

#include <mutex>

// Historical V18 recursive state guard, kept for nested controller helpers and
// regression tests. V19 dispatches connected SDK transactions to EciIoScheduler
// BEFORE taking this lock. This mutex itself does not provide scheduling.
// ScopedIoRelease remains for the V18 unit tests/compatibility; production V19
// waiting is split into caller-side start/poll/finish rather than releasing an
// outer SDK lock while sleeping.
namespace p0 {
using IoMutex = std::recursive_mutex;
using IoLock = std::unique_lock<IoMutex>;

class ScopedIoRelease final {
public:
    explicit ScopedIoRelease(IoLock& lock) : lock_(lock) { lock_.unlock(); }
    ~ScopedIoRelease() { lock_.lock(); }
    ScopedIoRelease(const ScopedIoRelease&) = delete;
    ScopedIoRelease& operator=(const ScopedIoRelease&) = delete;
private:
    IoLock& lock_;
};
} // namespace p0
