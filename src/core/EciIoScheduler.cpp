#include "core/EciIoScheduler.h"
#include <algorithm>
#include <exception>
#include <iterator>
#include <stdexcept>
#include <utility>
#include <vector>

namespace eci {
thread_local const EciIoScheduler* EciIoScheduler::current_ = nullptr;
thread_local EciIoScheduler::Token EciIoScheduler::currentToken_{};
namespace {
double milliseconds(EciIoScheduler::Clock::duration d) {
    return std::chrono::duration<double, std::milli>(d).count();
}
}
EciIoScheduler::EciIoScheduler(std::size_t capacity, std::size_t reserve)
    : normalCapacity_((std::max)(std::size_t{1}, capacity)),
      emergencyReserve_((std::max)(std::size_t{1}, reserve)),
      worker_([this] { run(); }) {}
EciIoScheduler::~EciIoScheduler() { shutdown(); }
bool EciIoScheduler::isWorkerThread() const noexcept { return current_ == this; }
EciIoScheduler::Token EciIoScheduler::token() const {
    std::lock_guard<std::mutex> lock(mutex_); return generation_;
}
EciIoScheduler::Token EciIoScheduler::transactionToken() const {
    return isWorkerThread() ? currentToken_ : token();
}
EciIoScheduler::Stats EciIoScheduler::stats() const {
    std::lock_guard<std::mutex> lock(mutex_);
    Stats s; s.classes = classes_; s.pending = queue_.size(); s.highWater = highWater_;
    s.accepting = accepting_; s.inFlight = inFlight_; s.inFlightName = inFlightName_;
    s.inFlightMs = inFlight_ ? milliseconds(Clock::now() - inFlightSince_) : 0;
    s.generation = generation_; return s;
}
EciIoScheduler::Options EciIoScheduler::options(Priority p, const char* name, unsigned domain) {
    Options o; o.priority = p; o.name = name; o.domain = domain;
    switch (p) {
    case Priority::Emergency: o.deadline = std::chrono::milliseconds(5); break;
    case Priority::Motion: o.deadline = std::chrono::milliseconds(10); break;
    case Priority::Measurement: o.deadline = std::chrono::milliseconds(10); break;
    case Priority::Control: o.deadline = std::chrono::milliseconds(50); break;
    default: o.deadline = std::chrono::milliseconds(200); break;
    }
    // Diagnostics are not dropped by default: existing bool APIs must not
    // quietly return cached/stale values. Callers can explicitly opt in.
    return o;
}
bool EciIoScheduler::staleLocked(const Job& j) const {
    return (j.options.sessionBound && j.token.session != generation_.session)
        || ((j.options.domain & Motion) && j.token.motion != generation_.motion)
        || ((j.options.domain & Acquisition) && j.token.acquisition != generation_.acquisition);
}
void EciIoScheduler::fail(const std::shared_ptr<Job>& j, const std::string& why) {
    j->promise.set_exception(std::make_exception_ptr(std::runtime_error(why)));
    if (j->onError) { try { j->onError(why); } catch (...) {} }
}
void EciIoScheduler::cancelStale() {
    std::vector<std::shared_ptr<Job>> cancelled;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        for (auto i = queue_.begin(); i != queue_.end();) {
            if (staleLocked(**i)) {
                ++classes_[index((*i)->options.priority)].cancelled;
                cancelled.push_back(*i); i = queue_.erase(i);
            } else ++i;
        }
    }
    for (const auto& j : cancelled) fail(j, "ECI request cancelled by STOP or connection generation change: " + j->options.name);
    wake_.notify_one();
}
void EciIoScheduler::invalidate(unsigned domains) {
    { std::lock_guard<std::mutex> lock(mutex_);
      if (domains & Motion) ++generation_.motion;
      if (domains & Acquisition) ++generation_.acquisition; }
    cancelStale();
}
void EciIoScheduler::newSession() {
    { std::lock_guard<std::mutex> lock(mutex_); ++generation_.session; ++generation_.motion; ++generation_.acquisition; }
    cancelStale();
}
bool EciIoScheduler::enqueue(const std::shared_ptr<Job>& j) {
    std::string rejection;
    std::vector<std::shared_ptr<Job>> cancelled;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto& s = classes_[index(j->options.priority)]; ++s.submitted;
        if (!accepting_) { ++s.rejected; rejection = "ECI scheduler is shutting down"; }
        else if (staleLocked(*j)) { ++s.cancelled; rejection = "ECI stale request generation"; }
        else {
            if (j->options.invalidateDomains) {
                // No gap in which a new motion can jump ahead of STOP admission.
                if (j->options.invalidateDomains & Motion) ++generation_.motion;
                if (j->options.invalidateDomains & Acquisition) ++generation_.acquisition;
                j->token = generation_;
                for (auto i = queue_.begin(); i != queue_.end();) {
                    if (staleLocked(**i)) {
                        ++classes_[index((*i)->options.priority)].cancelled;
                        cancelled.push_back(*i); i = queue_.erase(i);
                    } else ++i;
                }
            }
            const bool emergency = j->options.priority == Priority::Emergency;
            const auto emergencies = static_cast<std::size_t>(std::count_if(queue_.begin(), queue_.end(),
                [](const auto& q) { return q->options.priority == Priority::Emergency; }));
            if ((emergency && emergencies >= emergencyReserve_)
                || (!emergency && queue_.size() - emergencies >= normalCapacity_)) {
                ++s.rejected; rejection = "ECI request queue full (backpressure; request NOT sent)";
            } else {
                queue_.push_back(j); highWater_ = (std::max)(highWater_, queue_.size());
            }
        }
    }
    for (const auto& old : cancelled) fail(old, "ECI request cancelled by STOP: " + old->options.name);
    if (!rejection.empty()) fail(j, rejection + ": " + j->options.name);
    else wake_.notify_one();
    return rejection.empty();
}
std::future<bool> EciIoScheduler::submit(Options o, Token t, std::function<bool()> fn) {
    if (index(o.priority) >= index(Priority::Count)) throw std::invalid_argument("Invalid I/O priority");
    auto j = std::make_shared<Job>(); j->options = std::move(o); j->token = t;
    j->fn = std::move(fn); j->enqueued = Clock::now(); auto future = j->promise.get_future();
    enqueue(j); return future;
}
bool EciIoScheduler::invoke(Options o, Token t, std::function<bool()> fn) {
    // Nested guarded helpers belong to the current atomic transaction. Do not
    // queue behind ourselves (deadlock) or interleave BASE/params/MOVE.
    if (isWorkerThread()) return fn();
    return submit(std::move(o), t, std::move(fn)).get();
}
bool EciIoScheduler::post(Options o, Token t, std::function<bool()> fn,
                          std::function<void(const std::string&)> onError) {
    if (index(o.priority) >= index(Priority::Count)) throw std::invalid_argument("Invalid I/O priority");
    auto j = std::make_shared<Job>(); j->options = std::move(o); j->token = t;
    j->fn = std::move(fn); j->onError = std::move(onError); j->enqueued = Clock::now(); return enqueue(j);
}
std::shared_ptr<EciIoScheduler::Job> EciIoScheduler::takeNextLocked(Clock::time_point now) {
    auto best = queue_.begin();
    // Emergency always wins. Otherwise overdue requests use earliest deadline;
    // normal requests use priority/FIFO. This prevents continuous refills from
    // starving TABLE. A deadline miss is counted, NOT a hard-real-time promise.
    auto better = [now](const auto& a, const auto& b) {
        const bool ae = a->options.priority == Priority::Emergency, be = b->options.priority == Priority::Emergency;
        if (ae != be) return ae;
        if (ae) return a->enqueued < b->enqueued;
        const auto ad = a->enqueued + a->options.deadline;
        const auto bd = b->enqueued + b->options.deadline;
        const bool al = ad <= now, bl = bd <= now;
        if (al != bl) return al;
        if (al && ad != bd) return ad < bd;
        if (a->options.priority != b->options.priority) return a->options.priority < b->options.priority;
        return a->enqueued < b->enqueued;
    };
    for (auto i = std::next(queue_.begin()); i != queue_.end(); ++i) if (better(*i, *best)) best = i;
    auto j = *best; queue_.erase(best); return j;
}
void EciIoScheduler::run() {
    current_ = this;
    for (;;) {
        std::shared_ptr<Job> j; std::string rejection;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            wake_.wait(lock, [this] { return stopping_ || !queue_.empty(); });
            if (stopping_ && queue_.empty()) break;
            const auto now = Clock::now(); j = takeNextLocked(now);
            auto& s = classes_[index(j->options.priority)];
            s.maxQueueMs = (std::max)(s.maxQueueMs, milliseconds(now - j->enqueued));
            const bool late = now > j->enqueued + j->options.deadline;
            if (staleLocked(*j)) { ++s.cancelled; rejection = "ECI stale generation before execution"; }
            else if (late && j->options.expireBeforeStart && j->options.priority != Priority::Emergency) {
                ++s.expired; rejection = "ECI read request expired before execution";
            } else {
                if (late) ++s.late;
                inFlight_ = true; inFlightSince_ = now; inFlightName_ = j->options.name;
            }
        }
        if (!rejection.empty()) { fail(j, rejection + ": " + j->options.name); continue; }
        currentToken_ = j->token; // STOP may advance global generation while fn runs
        bool result = false; std::exception_ptr exception;
        try { result = j->fn(); } catch (...) { exception = std::current_exception(); }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            auto& s = classes_[index(j->options.priority)]; ++s.completed;
            if (exception || !result) ++s.failed;
            s.maxExecutionMs = (std::max)(s.maxExecutionMs, milliseconds(Clock::now() - inFlightSince_));
            inFlight_ = false; inFlightName_.clear();
        }
        if (exception) {
            j->promise.set_exception(exception);
            if (j->onError) { try { j->onError("ECI I/O transaction threw: " + j->options.name); } catch (...) {} }
        } else {
            j->promise.set_value(result);
            if (!result && j->onError) { try { j->onError("ECI I/O transaction failed: " + j->options.name); } catch (...) {} }
        }
    }
    current_ = nullptr;
}
void EciIoScheduler::shutdown() {
    // Destruction must be performed by owner after its external callers exit.
    if (isWorkerThread()) std::terminate();
    std::lock_guard<std::mutex> joinLock(shutdownMutex_);
    std::deque<std::shared_ptr<Job>> cancelled;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        accepting_ = false; stopping_ = true; cancelled.swap(queue_);
        for (const auto& j : cancelled) ++classes_[index(j->options.priority)].cancelled;
    }
    for (const auto& j : cancelled) fail(j, "ECI scheduler shutdown: request NOT executed");
    wake_.notify_all();
    if (worker_.joinable()) worker_.join();
}
} // namespace eci
