#pragma once

#include "runtime/Acquisition5D.h"
#include <string>

namespace mls::runtime {

/**
 * Signed time correction for H1 (R0/IN0) and H3 (CMM/R1/IN1).
 * tau = t_sensor,effective - t_eci,latch.  A positive tau uses the following
 * hardware-latched pose; a negative tau uses the preceding pose.  Both require
 * strictly consecutive ordinals in the same session (the Probe is excluded).
 */
struct HardwareTemporalCorrectionConfig {
    bool enabled = false;
    double delaySec = 0.0;
    double triggerPeriodSec = 1.0 / static_cast<double>(Table5D::defaultTriggerFrequencyHz);
};

struct HardwareTemporalCorrectionResult {
    bool success = false;
    double alpha = 0.0;
    Frame5D frame;
    std::string message;
};

HardwareTemporalCorrectionResult interpolateHardwareTemporalFrame(
    const Frame5D& current,
    const Frame5D& neighbor,
    const HardwareTemporalCorrectionConfig& config);

} // namespace mls::runtime
