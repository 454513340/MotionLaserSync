#include "runtime/HardwareTemporalCorrection.h"

#include <cmath>

namespace mls::runtime {

HardwareTemporalCorrectionResult interpolateHardwareTemporalFrame(
    const Frame5D& current,
    const Frame5D& neighbor,
    const HardwareTemporalCorrectionConfig& config)
{
    HardwareTemporalCorrectionResult out;
    if (!config.enabled) {
        out.success = true;
        out.frame = current;
        out.message = "Temporal correction disabled; raw hardware-latched frame retained.";
        return out;
    }
    if (!std::isfinite(config.delaySec) || !std::isfinite(config.triggerPeriodSec) ||
        config.triggerPeriodSec <= 0.0 ||
        std::abs(config.delaySec) >= config.triggerPeriodSec) {
        out.message = "Signed temporal delay must satisfy abs(delay) < trigger period.";
        return out;
    }
    if (current.timing != PoseTiming::HardwareLatched || neighbor.timing != PoseTiming::HardwareLatched ||
        current.latchMask != Table5D::allAxesMask || neighbor.latchMask != Table5D::allAxesMask ||
        current.feedbackMask != Table5D::allAxesMask || neighbor.feedbackMask != Table5D::allAxesMask ||
        current.commandMask != 0 || neighbor.commandMask != 0) {
        out.message = "Temporal correction requires two consecutive full-MPOS hardware-latched frames.";
        return out;
    }
    const bool forward = config.delaySec >= 0.0;
    const bool consecutive = forward
        ? (current.ordinal >= 1 && neighbor.ordinal == current.ordinal + 1)
        : (current.ordinal >= 2 && neighbor.ordinal >= 1 && neighbor.ordinal + 1 == current.ordinal);
    if (current.session == 0 || current.session != neighbor.session || !consecutive) {
        out.message = "Temporal correction requires adjacent continuous-stage ordinals in one session (Probe ordinal 0 is excluded).";
        return out;
    }
    if (current.rotaryOffsets != neighbor.rotaryOffsets ||
        current.physicalParameters != neighbor.physicalParameters ||
        current.absoluteGaugeKnown != neighbor.absoluteGaugeKnown) {
        out.message = "Temporal correction look-ahead frames have inconsistent calibration provenance.";
        return out;
    }
    if (!finiteJoint(current.joints) || !finiteJoint(neighbor.joints)) {
        out.message = "Temporal correction received non-finite joints.";
        return out;
    }

    const double alpha = config.delaySec / config.triggerPeriodSec;
    Frame5D f = current;
    for (std::size_t i = 0; i < f.joints.size(); ++i) {
        // XYZ/C/B are controller machine coordinates.  Do not shortest-path wrap C/B:
        // the controller already publishes the continuous mechanical coordinate used by Physical17.
        const double delta = forward ? (neighbor.joints[i] - current.joints[i])
                                     : (current.joints[i] - neighbor.joints[i]);
        f.joints[i] = current.joints[i] + alpha * delta;
    }
    f.timing = PoseTiming::HardwareTemporalInterpolated;
    f.temporalOffsetSec = config.delaySec;
    f.temporalAlpha = alpha;
    // The persisted field records the adjacent interpolation ordinal. For
    // negative tau it is the PREVIOUS ordinal (kept for schema compatibility).
    f.temporalNextOrdinal = neighbor.ordinal;
    // sensorToMachine must be recomputed from interpolated joints by completeFrame().
    f.sensorToMachine = {};

    out.success = true;
    out.alpha = alpha;
    out.frame = f;
    out.message = "OK";
    return out;
}

} // namespace mls::runtime
