#pragma once
// V30.0.3: Interpret only documented scanCONTROL feature bits. Raw feature
// register equality is NOT a valid H3 check (internal mode is bit 25 cleared).
// Pure functions allow regression tests without the sensor, Qt or the SDK DLL.
#include <cstdint>
namespace mls::sensor::trigger {
inline constexpr std::uint32_t kExternalActiveBit = 1u << 25;
inline constexpr std::uint32_t kH1TriggerMask = (3u << 16) | (1u << 21) | (1u << 24) | kExternalActiveBit;
inline constexpr std::uint32_t kH1DigitalIoMask = (1u << 11) | (1u << 8) | (0x0fu << 4);
inline constexpr std::uint32_t kRs422PortMask = 0x0fu;
inline constexpr std::uint32_t kCmmRs422Port = 7u;

inline constexpr bool sensorTriggerMatches(int topologyId, std::uint32_t requested, std::uint32_t readback) {
    switch (topologyId) {
    case 1: // H1: actual external trigger, input, edge, polarity
        return (requested & kExternalActiveBit) != 0u &&
               (readback & kH1TriggerMask) == (requested & kH1TriggerMask);
    case 3: // H3: internally clocked sensor; non-mode bits need not be zero
        return (readback & kExternalActiveBit) == 0u;
    default:
        return false;
    }
}
inline constexpr bool digitalIoMatches(int topologyId, std::uint32_t requested, std::uint32_t readback) {
    switch (topologyId) {
    case 1: return (readback & kH1DigitalIoMask) == (requested & kH1DigitalIoMask);
    case 3: return (readback & kRs422PortMask) == kCmmRs422Port;
    default: return false;
    }
}
} // namespace mls::sensor::trigger
