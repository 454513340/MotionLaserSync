#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cmath>

// Pure, Qt-free logic shared by the diagnostic UI and the execution entry check.
// A UI selection is an operator INTENT. SyncManager's value is only a staged
// in-process value; neither is an independent physical controller read-back.
namespace mls::gcode::preflight {
struct HardwareSetting {
    int topology = 0;   // 1: H1, 3: H3
    int frequencyHz = 0;
    int r1Edge = 0;     // H3: 114/115; ignored for H1
};

enum class HardwareSettingState {
    Matches,
    UiMatchesButRuntimeCacheStale,
    UiDisagreesWithNc,
    NoUiIntentRuntimeMatches,
    NoUiIntentRuntimeDisagrees
};

inline bool matchesNc(const HardwareSetting& observed, const HardwareSetting& nc) {
    return observed.topology == nc.topology &&
           observed.frequencyHz == nc.frequencyHz &&
           (nc.topology != 3 || observed.r1Edge == nc.r1Edge);
}

inline HardwareSettingState classifyHardwareSettings(
    const HardwareSetting& nc,
    const HardwareSetting& stagedManagerValue,
    bool haveUiSelection,
    const HardwareSetting& uiSelection)
{
    if (!haveUiSelection) {
        return matchesNc(stagedManagerValue, nc)
            ? HardwareSettingState::NoUiIntentRuntimeMatches
            : HardwareSettingState::NoUiIntentRuntimeDisagrees;
    }
    if (!matchesNc(uiSelection, nc))
        return HardwareSettingState::UiDisagreesWithNc;
    return matchesNc(stagedManagerValue, nc)
        ? HardwareSettingState::Matches
        : HardwareSettingState::UiMatchesButRuntimeCacheStale;
}

struct InitialMoveDifference {
    bool valid = false;
    bool needsMotion = false;
    std::array<double, 5> delta{};
    // Linear and rotary axes use separate units and tolerances.
    double maxLinearMm = 0.0;
    double maxRotaryDeg = 0.0;
};
inline InitialMoveDifference analyzeFirstG00(
    const std::array<double, 5>& live,
    const std::array<double, 5>& destination,
    double linearToleranceMm = 0.05,
    double rotaryToleranceDeg = 0.05)
{
    InitialMoveDifference r;
    r.valid = linearToleranceMm >= 0 && rotaryToleranceDeg >= 0;
    for (int i = 0; i < 5; ++i) {
        if (!std::isfinite(live[i]) || !std::isfinite(destination[i])) {
            r.valid = false;
            return r;
        }
        r.delta[static_cast<std::size_t>(i)] = destination[i] - live[i];
        const double absDelta = std::abs(r.delta[static_cast<std::size_t>(i)]);
        if (i < 3) r.maxLinearMm = (std::max)(r.maxLinearMm, absDelta);
        else       r.maxRotaryDeg = (std::max)(r.maxRotaryDeg, absDelta);
    }
    r.needsMotion = r.maxLinearMm > linearToleranceMm ||
                    r.maxRotaryDeg > rotaryToleranceDeg;
    return r;
}
} // namespace mls::gcode::preflight
