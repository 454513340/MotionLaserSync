#pragma once
// Portable completion policy for all five measured MPOS axes. This is *not*
// a software replacement for the controller alarm/idle/E-stop checks.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

namespace mls::gcode {
constexpr double arrivalLinearToleranceMm=0.05;  // Machine5X safety config
constexpr double arrivalRotaryToleranceDeg=0.05;
constexpr std::int64_t idleFeedbackSettleBudgetMs=3000;

struct ArrivalObservation {
    bool valid=false;
    bool withinTolerance=false;
    std::array<double,5> error{}; // actual MPOS minus target, XYZ(mm), C/B(deg)
};
inline ArrivalObservation observeArrival(const std::array<double,5>& mpos,
                                         const std::array<double,5>& target){
    ArrivalObservation out;
    out.valid=true;
    out.withinTolerance=true;
    for(int j=0;j<5;++j){
        if(!std::isfinite(mpos[j]) || !std::isfinite(target[j])){
            out.valid=false;out.withinTolerance=false;return out;
        }
        out.error[j]=mpos[j]-target[j];
        const double tol=j<3?arrivalLinearToleranceMm:arrivalRotaryToleranceDeg;
        if(std::abs(out.error[j])>tol)out.withinTolerance=false;
    }
    return out;
}
// Original policy: 3x the actual controller planner's duration + 5s for
// refill/convergence, min 5s. Do not blindly remove the watchdog to hide stalls.
inline std::int64_t motionWatchdogBudgetMs(double plannedSeconds){
    if(!std::isfinite(plannedSeconds) || plannedSeconds<0)return 0;
    const double proposed=std::max(5000.0,plannedSeconds*3000.0+5000.0);
    if(proposed>double((std::numeric_limits<std::int64_t>::max)()/4))return 0;
    return static_cast<std::int64_t>(std::ceil(proposed));
}
} // namespace mls::gcode
