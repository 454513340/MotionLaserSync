#include "calibration/HardwareTemporalCalibration.h"
#include "runtime/HardwareTemporalCorrection.h"
#include <cmath>
#include <cstdint>
#include <iostream>
#include <vector>
#include <cstdlib>

using namespace mls::calibration;
using namespace mls::runtime;

static void test(bool condition, const char* name)
{
    if (!condition) { std::cerr << "FAIL " << name << "\n"; std::exit(1); }
}

int main()
{
    // A physically plausible H3: sensor exposure precedes CMM-to-R1 latch.
    constexpr double tau = -0.000275; // -275 us
    std::vector<TemporalTrajectoryObservation> obs;
    std::uint64_t id=1, pairId=1;
    for(int speed : {2,4,6,8}) for(int repeat=0;repeat<4;++repeat) {
        for(int dir : {1,-1}) {
            TemporalTrajectoryObservation o;
            o.id=id++;o.pairId=pairId;o.planId=o.id;o.directionSign=dir;
            o.velocityMmPerSec={dir*double(speed),0,0};
            o.captureMidPcUs=1'000'000+std::int64_t(o.id)*3'000'000;
            const double t=(o.captureMidPcUs-1'000'000)*1e-6;
            o.sphereCenterMachineMm={10.0 + 0.000001*t-tau*o.velocityMmPerSec.x,
                                    20.0+0.0000015*t,30.0-0.000002*t};
            o.sphereRmsMm=0.01;o.fittedRadiusMm=9.535;o.sphereInliers=10000;
            o.profileCount=1000;o.resolutionIndex=0;
            o.exposureTime10Us=15;o.idleTime10Us=485;o.measuringFieldIndex=7;
            o.triggerFrequencyHz=200;o.triggerPeriodMs=5.0;
            obs.push_back(o);
        }
        ++pairId;
    }
    auto r=HardwareTemporalCalibration::solveTrajectoryPairs(obs);
    test(r.success,"H3 negative delay solved");
    test(r.pairCount==16,"16 H3 pairs");
    test(std::abs(r.delaySec-tau)<1.0e-7,"signed tau recovered");
    TemporalTrajectoryCriteria c;
    c.minPairs=12;c.minDistinctSpeedLevels=3;c.minRSquared=0.8;
    c.maxPairResidualMm=0.01;c.maxPairRadiusDifferenceMm=0.01;
    c.maxDelayStdSec=150e-6;
    test(!HardwareTemporalCalibration::evaluateTrajectoryQuality(r,c),"H1 rejects negative tau");
    c.allowSignedDelay=true;
    test(HardwareTemporalCalibration::evaluateTrajectoryQuality(r,c),"H3 accepts measured negative tau");

    Frame5D prev,now,nxt;
    prev.session=now.session=nxt.session=7;
    prev.ordinal=2;now.ordinal=3;nxt.ordinal=4;
    prev.joints={10.0,0,0,0,0};now.joints={10.025,0,0,0,0};nxt.joints={10.050,0,0,0,0};
    for (auto* f : {&prev,&now,&nxt}) {
        f->timing=PoseTiming::HardwareLatched;
        f->latchMask=f->feedbackMask=31;f->commandMask=0;
    }
    HardwareTemporalCorrectionConfig config;
    config.enabled=true;config.delaySec=-0.001;config.triggerPeriodSec=0.005;
    const auto back=interpolateHardwareTemporalFrame(now,prev,config);
    test(back.success,"negative-tau interpolation succeeds");
    test(std::abs(back.frame.joints[0]-10.020)<1e-10,"negative-tau interpolation uses previous");
    test(back.frame.temporalNextOrdinal==2 && back.frame.temporalAlpha==-0.2,"signed provenance records previous ordinal");
    auto completeNegative=back.frame;
    std::string transformError;
    const std::array<double,17> physical17{};
    test(completeFrame(completeNegative,physical17,transformError),"completeFrame accepts H3 negative provenance");
    test(validTransform(completeNegative.sensorToMachine),"completeFrame rebuilds valid H3 transform");
    auto forgedNegative=back.frame;
    forgedNegative.temporalNextOrdinal=forgedNegative.ordinal+1;
    test(!completeFrame(forgedNegative,physical17,transformError),"completeFrame rejects forged H3 forward neighbor");
    test(!interpolateHardwareTemporalFrame(now,nxt,config).success,"negative-tau rejects next as neighbor");
    auto probe=prev;probe.ordinal=0;
    auto ordinalOne=now;ordinalOne.ordinal=1;
    test(!interpolateHardwareTemporalFrame(ordinalOne,probe,config).success,"negative-tau rejects Probe reference");
    auto differentSession=prev;differentSession.session=8;
    test(!interpolateHardwareTemporalFrame(now,differentSession,config).success,"cross-session rejected");
    config.delaySec=-0.005;
    test(!interpolateHardwareTemporalFrame(now,prev,config).success,"one whole-period negative tau rejected");
    config.delaySec=0.001;
    auto positive=interpolateHardwareTemporalFrame(now,nxt,config);
    test(positive.success && std::abs(positive.frame.joints[0]-10.030)<1e-10,"H1 positive interpolation preserved");
    std::cout << "H3 signed temporal tests PASS\n";
}
