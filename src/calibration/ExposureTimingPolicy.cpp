#include "calibration/ExposureTimingPolicy.h"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace mls::calibration {

ExposureTimingDecision ExposureTimingPolicy::decide(int recommendedExposure10us,
                                                    int referenceIdle10us,
                                                    int referenceProfileRateHz,
                                                    bool hardwareSync,
                                                    int hardwareTriggerFrequencyHz)
{
    ExposureTimingDecision d;
    d.exposure10us = recommendedExposure10us;
    if (recommendedExposure10us <= 0) {
        d.explanation = "recommended exposure must be positive";
        return d;
    }

    if (hardwareSync) {
        const int hz = std::max(1, hardwareTriggerFrequencyHz);
        const double triggerPeriod10us = 100000.0 / static_cast<double>(hz);
        // Keep at least one 10us unit of guard below the exact external period.
        // floor() is important for frequencies such as 144 Hz where the period is
        // not an integer number of 10us sensor timing units.
        const int guardedPeriod10us = std::max(1, static_cast<int>(std::floor(triggerPeriod10us)) - 1);
        if (recommendedExposure10us >= guardedPeriod10us) {
            std::ostringstream msg;
            msg << "recommended exposure " << recommendedExposure10us
                << " x10us cannot fit inside hardware trigger frequency "
                << hz << " Hz (period " << triggerPeriod10us << " x10us)";
            d.explanation = msg.str();
            return d;
        }
        d.period10us = guardedPeriod10us;
        d.idle10us = d.period10us - recommendedExposure10us;
        d.resultingRateHz = 100000.0 / static_cast<double>(d.period10us);
        d.hardwareAdjusted = true;
        d.valid = true;
        d.explanation = "hardware sync: idle derived from runtime PWM frequency with 10us guard";
        return d;
    }

    if (referenceIdle10us >= 0) {
        d.idle10us = referenceIdle10us;
        d.period10us = recommendedExposure10us + referenceIdle10us;
    } else if (referenceProfileRateHz > 0) {
        d.period10us = std::max(recommendedExposure10us,
            static_cast<int>(std::lround(100000.0 / static_cast<double>(referenceProfileRateHz))));
        d.idle10us = std::max(0, d.period10us - recommendedExposure10us);
    } else {
        d.explanation = "material profile has neither reference idle nor reference profile rate";
        return d;
    }

    if (d.period10us <= 0) {
        d.explanation = "invalid material reference period";
        return d;
    }
    d.resultingRateHz = 100000.0 / static_cast<double>(d.period10us);
    d.valid = true;
    d.explanation = "software/internal trigger: calibrated reference exposure and idle restored";
    return d;
}

} // namespace mls::calibration
