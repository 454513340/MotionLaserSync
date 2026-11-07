#pragma once

#include "runtime/Acquisition5D.h"

#include <string>

namespace mls::calibration {

struct ExposureTimingDecision {
    bool valid = false;
    int exposure10us = 0;
    int idle10us = 0;
    int period10us = 0;
    double resultingRateHz = 0.0;
    bool hardwareAdjusted = false;
    std::string explanation;
};

class ExposureTimingPolicy {
public:
    // Software/internal-trigger mode reuses the calibrated reference idle/rate.
    // Hardware mode never copies a slow calibration idle into the external-trigger
    // cycle; it derives a safe internal period from the configured PWM frequency
    // and keeps Exposure+Idle strictly shorter than that period.
    static ExposureTimingDecision decide(int recommendedExposure10us,
                                         int referenceIdle10us,
                                         int referenceProfileRateHz,
                                         bool hardwareSync,
                                         int hardwareTriggerFrequencyHz = mls::runtime::Table5D::defaultTriggerFrequencyHz);
};

} // namespace mls::calibration
