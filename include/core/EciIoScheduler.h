#pragma once

#include <array>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

// V19: one worker for connected ECI3808 I/O. Scheduling is non-preemptive at
// TRANSACTION boundaries, not at SDK-call boundaries. No DLL timeout/abort is
// invented here. Callbacks MUST NOT sleep, wait for motion, pump GUI events or
// perform RS485 operations. The synchronous facade waits on the CALLER thread.
namespace eci {
class EciIoScheduler final {
public:
    using Clock = std::chrono::steady_clock;
    enum class Priority : unsigned { Emergency, Motion, Measurement, Control, Diagnostic, Count };
    enum Domain : unsigned { None = 0, Motion = 1, Acquisition = 2 };
    struct Token { std::uint64_t session = 0, motion = 0, acquisition = 0; };
    struct Options {
        Priority priority = Priority::Control;
        unsigned domain = None;
        unsigned invalidateDomains = None; // STOP: invalidate and enqueue atomically
        std::chrono::milliseconds deadline{50}; // soft queue-delay budget, not a guarantee
        bool expireBeforeStart = false;        // only for discardable read requests
        bool sessionBound = true;
        std::string name;
    };
    struct ClassStats {
        std::uint64_t submitted = 0, completed = 0, failed = 0;
        std::uint64_t cancelled = 0, rejected = 0, expired = 0, late = 0;
        double maxQueueMs = 0, maxExecutionMs = 0;
    };
    struct Stats {
        std::array<ClassStats, static_cast<unsigned>(Priority::Count)> classes{};
        std::size_t pending = 0, highWater = 0;
        bool accepting = true, inFlight = false;
        std::string inFlightName;
        double inFlightMs = 0;
        Token generation;
    };

    // Regular requests have a bounded queue; stop/cleanup get a separate reserve.
    explicit EciIoScheduler(std::size_t normalCapacity = 256, std::size_t emergencyReserve = 32);
    ~EciIoScheduler();
    EciIoScheduler(const EciIoScheduler&) = delete;
    EciIoScheduler& operator=(const EciIoScheduler&) = delete;

    bool isWorkerThread() const noexcept;
    Token token() const;
    Token transactionToken() const; // immutable admission token of the running transaction
    Stats stats() const; // memory snapshot only, NEVER communicates with the card
    void invalidate(unsigned domains); // cancel queued old start/refill/enable jobs
    void newSession();                // invalidate queued work from the old connection

    std::future<bool> submit(Options options, Token token, std::function<bool()> fn);
    bool invoke(Options options, Token token, std::function<bool()> fn);
    // Fire-and-report, used for GUI STOP. Returns admission status, NOT completion.
    // onError runs on worker (or submitting
    // thread if rejected immediately); it must not access widgets directly.
    bool post(Options options, Token token, std::function<bool()> fn,
              std::function<void(const std::string&)> onError);
    void shutdown(); // cancels queued jobs; joins in-flight transaction, NEVER detach/kill

    static Options options(Priority priority, const char* name, unsigned domain = None);
private:
    struct Job {
        Options options;
        Token token;
        Clock::time_point enqueued;
        std::function<bool()> fn;
        std::promise<bool> promise;
        std::function<void(const std::string&)> onError;
    };
    static thread_local const EciIoScheduler* current_;
    static thread_local Token currentToken_;
    static unsigned index(Priority p) { return static_cast<unsigned>(p); }
    bool staleLocked(const Job& job) const;
    std::shared_ptr<Job> takeNextLocked(Clock::time_point now);
    bool enqueue(const std::shared_ptr<Job>& job);
    static void fail(const std::shared_ptr<Job>& job, const std::string& reason);
    void cancelStale();
    void run();

    mutable std::mutex mutex_;
    std::condition_variable wake_;
    std::deque<std::shared_ptr<Job>> queue_;
    std::array<ClassStats, static_cast<unsigned>(Priority::Count)> classes_{};
    Token generation_{};
    std::size_t normalCapacity_, emergencyReserve_, highWater_ = 0;
    bool accepting_ = true, stopping_ = false;
    bool inFlight_ = false;
    Clock::time_point inFlightSince_{};
    std::string inFlightName_;
    std::mutex shutdownMutex_;
    std::thread worker_;
};
} // namespace eci
