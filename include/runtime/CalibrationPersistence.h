#pragma once

#include <cstdint>

namespace mls::runtime {

enum class FeedbackMaskEncoding {
    CompactXYZCB,
    LegacyControllerAxisOneBased
};

struct FeedbackMaskDecodeResult {
    bool ok = false;
    std::uint8_t normalized = 0;
    bool migratedLegacyOneBased = false;
};

// Persisted/runtime feedback masks use compact XYZCB bits 0..4. Historical
// Stage-1 files and the V27.6.12 schema-v1 active bundle used controller AxisId
// 1..5 as bit positions, so they must be decoded by schema rather than by value.
constexpr FeedbackMaskDecodeResult decodeFeedbackMask(
    int rawMask, FeedbackMaskEncoding encoding) noexcept
{
    if (encoding == FeedbackMaskEncoding::CompactXYZCB) {
        if (rawMask < 0 || rawMask > 0x1F) return {};
        return {true, static_cast<std::uint8_t>(rawMask), false};
    }

    if (rawMask < 0 || rawMask > 0x3E || (rawMask & 0x01) != 0) return {};
    const int shifted = rawMask >> 1;
    if (shifted < 0 || shifted > 0x1F) return {};
    return {true, static_cast<std::uint8_t>(shifted), true};
}

} // namespace mls::runtime
