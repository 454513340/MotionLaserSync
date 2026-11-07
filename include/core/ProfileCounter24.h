#pragma once

#include <cstdint>
#include <deque>
#include <utility>

namespace p0 {
// The V17 diagnostic report already defines this counter as 24-bit. Unwrap by
// successive accepted indices, not by subtracting the first raw index forever.
// Assumption: forward progress between observations is less than half a cycle.
// A larger jump / reset is ambiguous and must NOT silently re-anchor a scan.
class ProfileCounter24 final {
public:
    static constexpr std::uint32_t kModulus = 1U << 24;
    static constexpr std::uint32_t kMask = kModulus - 1U;
    enum class Kind { First, Next, Gap, Duplicate, BackwardOrReset, Invalid };
    struct Observation {
        Kind kind = Kind::Invalid;
        std::int64_t ordinal = 0;
        std::uint32_t missing = 0;
        bool accepted() const {
            return kind == Kind::First || kind == Kind::Next || kind == Kind::Gap;
        }
    };

    Observation observe(std::uint32_t index) {
        if (index > kMask) return {Kind::Invalid, ordinal_, 0};
        if (!started_) {
            started_ = true;
            first_ = last_ = index;
            return {Kind::First, 0, 0};
        }
        const std::uint32_t delta = (index - last_) & kMask;
        if (delta == 0) return {Kind::Duplicate, ordinal_, 0};
        if (delta >= kModulus / 2U) return {Kind::BackwardOrReset, ordinal_, 0};
        ordinal_ += delta;
        last_ = index;
        return {delta == 1 ? Kind::Next : Kind::Gap, ordinal_, delta - 1U};
    }

    void reset() { *this = ProfileCounter24{}; }
    bool started() const { return started_; }
    std::uint32_t first() const { return first_; }
private:
    bool started_ = false;
    std::uint32_t first_ = 0;
    std::uint32_t last_ = 0;
    std::int64_t ordinal_ = 0;
};
// Called under the owner's queue/pose locks. If the front is not ready, leave
// the entire FIFO untouched. Never push_front several waiting items in a loop.
template<class Pending>
bool takeReadyFront(std::deque<Pending>& queue, std::int64_t availablePoses,
                    Pending& out) {
    if (queue.empty() || queue.front().poseOrdinal < 0
        || queue.front().poseOrdinal >= availablePoses) return false;
    out = std::move(queue.front());
    queue.pop_front();
    return true;
}
} // namespace p0
