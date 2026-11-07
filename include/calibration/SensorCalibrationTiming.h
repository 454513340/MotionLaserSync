#pragma once

// Portable policy for calibration *internal-trigger* timing only.  These
// equations compute SDK settings; they do not certify the actual frame rate.
// Do not apply them to H1 external PWM or H3 sensor/CMM synchronisation.
namespace mls::calibration {

struct SensorCalibrationTimingPlan {
    bool valid = false;
    int requestedHz = 0;
    int exposure10us = 0;
    int idle10us = 0;
    int period10us = 0;
    double theoreticalRateHz = 0.0;
    const char* reason = "unknown error";
};

class SensorCalibrationTiming {
public:
    static constexpr int kUnitsPerSecond = 100000; // 10 us per unit
    static constexpr int kMinHz = 25;
    static constexpr int kMaxHz = 2000;
    // LLT.dll C++ Interface Documentation v1.3 (2019), section 3.1:
    // raw Video Mode MUST NOT exceed 25 Hz, independently of Profile rate.
    // scanCONTROL 29xx transmits 1280x1024 8-bit sensor images at ~262 Mbps.
    static constexpr int kVideoRateHz = 25;
    static constexpr int kMinExposure10us = 1;
    static constexpr int kMaxExposure10us = 4000;

    // The 29xx Video acquisition uses its own 25 Hz clock. Never pass a
    // measuring-field/profile maximum (144/552/...) to this path.
    static SensorCalibrationTimingPlan videoPlan(int exposure10us) noexcept
    {
        return plan(kVideoRateHz, exposure10us, kVideoRateHz);
    }

    static SensorCalibrationTimingPlan plan(int requestedHz,
                                            int exposure10us,
                                            int fieldReferenceMaxHz) noexcept
    {
        SensorCalibrationTimingPlan p;
        p.requestedHz = requestedHz;
        p.exposure10us = exposure10us;
        if (requestedHz < kMinHz || requestedHz > kMaxHz) {
            p.reason = "rate must be between 25 and 2000 Hz";
            return p;
        }
        if (fieldReferenceMaxHz < kMinHz || requestedHz > fieldReferenceMaxHz) {
            p.reason = "rate exceeds the selected measuring-field reference limit";
            return p;
        }
        if (exposure10us < kMinExposure10us || exposure10us > kMaxExposure10us) {
            p.reason = "exposure must be between 1 and 4000 units (10 us each)";
            return p;
        }
        // Exact integer ceil(100000/rate), no floating point rounding drift.
        p.period10us = (kUnitsPerSecond + requestedHz - 1) / requestedHz;
        if (exposure10us > p.period10us) {
            p.reason = "exposure is longer than the requested profile period";
            return p;
        }
        p.idle10us = p.period10us - exposure10us;
        p.theoreticalRateHz = static_cast<double>(kUnitsPerSecond) / p.period10us;
        p.valid = true;
        p.reason = "ok";
        return p;
    }
};

} // namespace mls::calibration
