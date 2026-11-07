#pragma once
#include <cmath>
#include <cstdint>
#include <vector>
#include "core/ProfileCounter24.h"

namespace syncdiag {
enum class PairingState : unsigned { Pending, Aligned, Dropped };
// Small per-received-frame metadata. Unlike AlignedPoint, EMPTY profiles are
// represented too. Does not change LaserProfile, TABLE or the .scan v2 format.
struct ProfileRecord {
    std::uint32_t index = 0;
    double shutterOpenSec = 0;
    std::int64_t callbackPcUs = 0;
    PairingState pairing = PairingState::Pending;
    std::uint32_t validPoints = 0; // interpreted only after successful pairing
    // V27.6.44: profile/REGIST pairing is valid, but no point cloud is emitted
    // when look-ahead temporal interpolation is impossible (formal Probe or final tail).
    bool temporalBoundarySkipped = false;

    // Host-ingress forensic metadata copied from LaserProfile.
    std::uint64_t callbackOrdinal = 0;
    std::uint32_t rawBytes = 0;
    std::uint32_t ingressDepth = 0;
    std::int64_t callbackServiceUs = 0;
    std::int64_t decoderQueueUs = 0;
    std::int64_t decodeServiceUs = 0;
    std::int64_t decodeDonePcUs = 0;
};
struct Interval {
    std::uint32_t first = 0, last = 0, deltaIndex = 0;
    double actualMs = 0, expectedMs = 0, residualMs = 0, perIndexMs = 0;
    bool indexValid = false, timeValid = false, residualWarning = false;
};
inline Interval interval(std::uint32_t first, std::uint32_t last,
                         double actualMs, double nominalMs) {
    Interval r; r.first = first; r.last = last; r.actualMs = actualMs;
    if (first > p0::ProfileCounter24::kMask || last > p0::ProfileCounter24::kMask) return r;
    r.deltaIndex = (last - first) & p0::ProfileCounter24::kMask;
    r.indexValid = r.deltaIndex > 0 && r.deltaIndex < p0::ProfileCounter24::kModulus / 2;
    if (!r.indexValid || !std::isfinite(nominalMs) || nominalMs <= 0) return r;
    r.expectedMs = r.deltaIndex * nominalMs;
    r.timeValid = std::isfinite(actualMs) && actualMs > 0;
    if (!r.timeValid) return r; // do not silently unwrap/reset an undocumented sensor clock
    r.residualMs = actualMs - r.expectedMs;
    r.perIndexMs = actualMs / r.deltaIndex;
    r.residualWarning = std::abs(r.residualMs) > nominalMs * 0.5;
    return r;
}
struct Distribution {
    std::uint64_t count = 0;
    double mean = 0, m2 = 0, min = 0, max = 0;
    void add(double v) {
        if (!std::isfinite(v)) return;
        if (count++ == 0) { mean = min = max = v; return; }
        if (v < min) min = v;
        if (v > max) max = v;
        const auto d = v - mean; mean += d / static_cast<double>(count);
        m2 += d * (v - mean);
    }
    double stddev() const { return count ? std::sqrt(m2 / static_cast<double>(count)) : 0; }
};
struct TimingSummary {
    Distribution actual, residual, normalized;
    std::uint64_t skippedIndex = 0, invalidTime = 0, warningCount = 0, crossIndexIntervals = 0;
    std::vector<Interval> warnings; // at most ten printed examples; count remains exact
};
inline TimingSummary analyzeReceivedTiming(const std::vector<ProfileRecord>& frames,
                                          double periodMs, bool sensorClock) {
    TimingSummary out;
    for (std::size_t i = 1; i < frames.size(); ++i) {
        const auto& a = frames[i - 1]; const auto& b = frames[i];
        const double actual = sensorClock ? (b.shutterOpenSec - a.shutterOpenSec) * 1000.0
                                         : (b.callbackPcUs - a.callbackPcUs) / 1000.0;
        auto r = interval(a.index, b.index, actual, periodMs);
        if (!r.indexValid) { ++out.skippedIndex; continue; }
        if (!r.timeValid) { ++out.invalidTime; continue; }
        out.actual.add(r.actualMs); out.residual.add(r.residualMs); out.normalized.add(r.perIndexMs);
        if (r.deltaIndex > 1) ++out.crossIndexIntervals;
        if (r.residualWarning) {
            ++out.warningCount; if (out.warnings.size() < 10) out.warnings.push_back(r);
        }
    }
    return out;
}
struct EffectiveGap {
    Interval timing;
    std::uint64_t confirmedEmpty = 0, otherReceived = 0;
};
inline std::vector<EffectiveGap> effectiveGaps(const std::vector<ProfileRecord>& frames,
                                              double periodMs) {
    std::vector<EffectiveGap> out;
    std::size_t previous = frames.size();
    std::uint64_t empty = 0, other = 0;
    for (std::size_t i = 0; i < frames.size(); ++i) {
        const auto& b = frames[i];
        if (b.pairing != PairingState::Aligned || b.validPoints == 0) {
            if (previous != frames.size()) {
                if (b.pairing == PairingState::Aligned && b.validPoints == 0 && !b.temporalBoundarySkipped) ++empty;
                else ++other;
            }
            continue;
        }
        if (previous != frames.size()) {
            const auto& a = frames[previous];
            const auto r = interval(a.index, b.index, (b.shutterOpenSec - a.shutterOpenSec) * 1000.0, periodMs);
            if (!r.indexValid || r.deltaIndex != 1) out.push_back({r, empty, other});
        }
        previous = i; empty = other = 0;
    }
    return out;
}
} // namespace syncdiag
