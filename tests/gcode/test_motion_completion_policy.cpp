// Controller-independent regression for the G00/G05 five-axis arrival policy.
// Neither this test nor the policy starts any real machine movement.
#include "gcode/MotionCompletionPolicy.h"
#include <array>
#include <cmath>
#include <iostream>
#include <limits>

#define TEST(cond) do { if (!(cond)) { \
    std::cerr << "FAIL " << __LINE__ << ": " << #cond << '\n'; return 1; \
} } while (false)

int main(){
    using namespace mls::gcode;
    const std::array<double,5> target{{43.653397934,-4.46764841327,-7.82286460302,-10.,-10.}};
    auto mpos=target;
    auto observed=observeArrival(mpos,target);
    TEST(observed.valid && observed.withinTolerance);
    for(int axis=0;axis<5;++axis){
        mpos=target;
        mpos[axis]+=(axis<3?arrivalLinearToleranceMm:arrivalRotaryToleranceDeg)*0.9;
        observed=observeArrival(mpos,target);
        TEST(observed.valid && observed.withinTolerance);
        mpos[axis]+=(axis<3?arrivalLinearToleranceMm:arrivalRotaryToleranceDeg)*0.2;
        observed=observeArrival(mpos,target);
        TEST(observed.valid && !observed.withinTolerance);
    }
    // Regression: a legitimate 0.02 mm linear encoder residual was previously
    // rejected by the hardcoded 0.005 mm finish gate (machine config = 0.05).
    mpos=target;
    mpos[0]+=0.02;
    TEST(observeArrival(mpos,target).withinTolerance);
    mpos[0]=std::numeric_limits<double>::infinity();
    TEST(!observeArrival(mpos,target).valid);
    mpos=target;
    mpos[4]=std::numeric_limits<double>::quiet_NaN();
    TEST(!observeArrival(mpos,target).valid);
    TEST(motionWatchdogBudgetMs(0.0)==5000);
    TEST(motionWatchdogBudgetMs(10.0)==35000);
    TEST(motionWatchdogBudgetMs(10.125)==35375);
    TEST(motionWatchdogBudgetMs(-1.0)==0);
    TEST(motionWatchdogBudgetMs(std::numeric_limits<double>::infinity())==0);
    TEST(motionWatchdogBudgetMs(std::numeric_limits<double>::quiet_NaN())==0);
    TEST(motionWatchdogBudgetMs(1.0e100)==0);
    TEST(idleFeedbackSettleBudgetMs>=2000);
    std::cout << "Motion completion arrival tolerance, finite-state safety, and watchdog PASS\n";
    return 0;
}
