#include "calibration/ExposureTimingPolicy.h"
#include "runtime/Acquisition5D.h"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

namespace {
void require(bool condition, const char* message)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(1);
    }
}
}

int main()
{
    using mls::calibration::ExposureTimingPolicy;
    using mls::runtime::Table5D;

    require(Table5D::defaultTriggerFrequencyHz == 200, "default hardware trigger frequency");
    require(Table5D::minTriggerFrequencyHz == 25, "minimum hardware trigger frequency");
    require(Table5D::maxTriggerFrequencyHz == 2000, "controller protocol maximum trigger frequency");
    require(Table5D::runtimeCapabilityOffset == 24, "runtime capability TABLE offset");
    require(Table5D::requestedFrequencyHzOffset == 25, "requested frequency TABLE offset");
    require(Table5D::appliedFrequencyHzOffset == 26, "applied frequency TABLE offset");
    require(Table5D::machineHomeMaskOffset == 27, "five-axis home mask TABLE offset");
    require(Table5D::runtimeCapabilityMagic == 321055, "runtime protocol capability magic");

    // Hardware timing must follow the selected runtime PWM rate, not a fixed 5 ms period.
    const auto hz200 = ExposureTimingPolicy::decide(100, 300, 0, true, 200);
    require(hz200.valid, "200 Hz timing should be valid");
    require(hz200.period10us == 499, "200 Hz guarded period should be 499 x 10us");
    require(hz200.idle10us == 399, "200 Hz idle should be derived from selected rate");

    const auto hz144 = ExposureTimingPolicy::decide(100, 300, 0, true, 144);
    require(hz144.valid, "144 Hz timing should be valid");
    require(hz144.period10us == 693, "144 Hz guarded period should floor the non-integer period then reserve one unit");
    require(hz144.idle10us == 593, "144 Hz idle should follow the 144 Hz runtime period");

    const auto hz2000 = ExposureTimingPolicy::decide(20, 300, 0, true, 2000);
    require(hz2000.valid, "2000 Hz timing with short exposure should be valid");
    require(hz2000.period10us == 49, "2000 Hz guarded period should be 49 x 10us");
    require(hz2000.idle10us == 29, "2000 Hz idle should be derived from selected rate");

    const auto tooLong = ExposureTimingPolicy::decide(49, 300, 0, true, 2000);
    require(!tooLong.valid, "exposure that does not fit inside the selected hardware period must fail closed");

    // Software/internal mode remains driven by its material/calibration timing and is independent
    // of the hardware PWM setting.
    const auto software = ExposureTimingPolicy::decide(100, 300, 0, false, 333);
    require(software.valid, "software timing should remain valid");
    require(software.period10us == 400 && software.idle10us == 300,
            "software timing should keep its calibrated exposure+idle period");

    std::cout << "PASS V27.6.53 runtime PWM/timing protocol tests\n";
    return 0;
}
