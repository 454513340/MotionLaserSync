#pragma once
// Controller/Sensor session evidence, not a software settings comparison.
// This pure policy deliberately never authorizes NC axis movement.
#include <cstdint>
#include <cmath>
#include <string>
#include <vector>

namespace mls::gcode::probe {
struct Evidence {
    int topologyId=0; // 1=H1 sensor external / 3=H3 sensor internal+CMM
    bool stopCompleted=false;
    bool synchronized=false;
    bool alignmentFault=false;
    bool publishedTriggerCountValid=false;
    bool sensorTriggerReadbackValid=false;
    bool sensorTriggerConfigMatches=false;
    bool sensorCmmPortReadbackValid=false; // H3 requires its own RS422/CMM readback
    std::int64_t poses=0, profiles=0, publishedTriggers=0, aligned=0;
    std::int64_t validProfiles=0, emptyProfiles=0, boundarySkipped=0;
    std::int64_t pending=0, dropped=0, overflows=0;
    std::int64_t missing=0, duplicates=0, outOfOrder=0, invalidIndices=0, tableReadErrors=0;
    std::uint64_t rawCallbacks=0, conversionFailures=0, ingressDrops=0;
    int configuredHz=0;
    std::int64_t measuredSteadyIntervals=0;
    double measuredMedianPeriodMs=0;
};
struct Verdict {
    bool sessionValid=false;
    std::vector<std::string> failures;
};
inline Verdict assess(const Evidence& e, std::int64_t minimumProfiles=3) {
    Verdict r;
    const auto reject=[&](const char* why){r.failures.emplace_back(why);};
    if(!e.stopCompleted) reject("stop/drain not completed");
    if(e.alignmentFault) reject("hardware alignment fault latched");
    if(!e.synchronized) reject("hardware sync not established");
    if(!e.publishedTriggerCountValid) reject("controller published latch count unavailable");
    if(e.profiles < minimumProfiles) reject("insufficient real profiles to prove capture");
    if(e.poses != e.profiles || e.aligned != e.profiles ||
       e.publishedTriggers != e.profiles) reject("pose/profile/align/published-latch count mismatch");
    if(e.pending || e.dropped || e.overflows || e.missing || e.duplicates ||
       e.outOfOrder || e.invalidIndices || e.tableReadErrors)
        reject("missing/duplicate/out-of-order/drop/overflow/TABLE failure");
    if(e.validProfiles + e.emptyProfiles + e.boundarySkipped != e.aligned)
        reject("profile classification/count mismatch");
    // Static surfaces may yield blank profiles; those are not a sync fault.
    // Never evaluate H3 as if it were H1's external rising-edge input.
    if(e.topologyId == 1) {
        if(!e.sensorTriggerReadbackValid || !e.sensorTriggerConfigMatches)
            reject("H1 external trigger / digital input configuration not verified");
    } else if(e.topologyId == 3) {
        if(!e.sensorTriggerReadbackValid || !e.sensorTriggerConfigMatches)
            reject("H3 internal sensor trigger configuration not verified");
        if(!e.sensorCmmPortReadbackValid)
            reject("H3 RS422 CMM output configuration not read back or mismatched");
    } else {
        reject("unknown H1/H3 topology: cannot establish sensor trigger contract");
    }
    if(e.rawCallbacks < static_cast<std::uint64_t>(e.profiles) || e.conversionFailures || e.ingressDrops)
        reject("sensor callback, decoder or host-ingress failure");
    if(e.configuredHz<=0 || e.measuredSteadyIntervals<2 ||
       !std::isfinite(e.measuredMedianPeriodMs) || e.measuredMedianPeriodMs<=0.0){
        reject("cannot establish NC frequency from real sensor timestamps");
    }else if(std::abs(e.measuredMedianPeriodMs-1000.0/e.configuredHz)>0.10*(1000.0/e.configuredHz)){
        reject("sensor measured steady-state rate differs from requested NC rate by more than 10 percent");
    }
    r.sessionValid=r.failures.empty();
    return r;
}
}
