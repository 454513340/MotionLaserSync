#include "gcode/HardwarePreflightLogic.h"
#include <cassert>
#include <cmath>
#include <iostream>
using namespace mls::gcode::preflight;
int main(){
    const HardwareSetting ncH3{3,100,114};
    const HardwareSetting staleH1{1,200,104};
    const HardwareSetting uiH3{3,100,114};
    const HardwareSetting wrongEdge{3,100,115};
    assert(classifyHardwareSettings(ncH3,staleH1,true,uiH3)==
           HardwareSettingState::UiMatchesButRuntimeCacheStale);
    assert(classifyHardwareSettings(ncH3,uiH3,true,uiH3)==HardwareSettingState::Matches);
    assert(classifyHardwareSettings(ncH3,uiH3,true,wrongEdge)==HardwareSettingState::UiDisagreesWithNc);
    assert(classifyHardwareSettings(ncH3,staleH1,false,{})==HardwareSettingState::NoUiIntentRuntimeDisagrees);
    // H1 has no R1 latch-edge contract.
    assert(matchesNc(HardwareSetting{1,200,104},HardwareSetting{1,200,0}));
    assert(!matchesNc(HardwareSetting{3,100,115},ncH3));

    // The user's logged live entry is already within the 0.05 mm/deg window.
    const std::array<double,5> current{43.6435,-4.4725,-7.8425,-9.9995,-10.0015};
    const std::array<double,5> target{43.6534,-4.4676,-7.8229,-10.0,-10.0};
    const auto near=analyzeFirstG00(current,target);
    assert(near.valid && !near.needsMotion);
    assert(std::abs(near.maxLinearMm-0.0196)<1.e-8);
    assert(near.maxRotaryDeg<0.002);
    const auto away=analyzeFirstG00(current,std::array<double,5>{70.,0.,0.,5.,10.});
    assert(away.valid && away.needsMotion);
    const auto bad=analyzeFirstG00(current,std::array<double,5>{NAN,0.,0.,0.,0.});
    assert(!bad.valid);
    std::cout<<"PASS: UI H3/100 vs stale H1/200, independent cache, dynamic first G00 analysis\n";
}
