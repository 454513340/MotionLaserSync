// Portable test: simulated exposure matrices survive Archive v5 unchanged;
// hardware-origin strict Physical17 validation must remain fail closed.
#include "runtime/ScanArchive5D.h"
#include <cassert>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

using namespace mls::runtime;
int main(){
    ArchiveProfile profile;
    auto& f=profile.frame;
    f.session=42;f.ordinal=1;
    f.joints={10.,20.,30.,5.,-7.};
    f.command=f.joints;f.raw=f.joints;
    f.timing=PoseTiming::Simulated;
    f.feedbackMask=0;f.latchMask=0;f.commandMask=0;
    f.absoluteGaugeKnown=false;
    f.hostEstimateUs=5000;
    f.sensorToMachine={1.,0.,0.,13.,0.,1.,0.,24.,0.,0.,1.,35.,0.,0.,0.,1.};
    profile.timeUs=5000;profile.sensorTimeSec=0.005;
    profile.profileIndex=0;
    profile.points={{0,1.5,95.},{1279,-2.5,112.}};
    std::vector<std::uint8_t> packed;
    std::string error;
    assert(encodeArchive({profile},packed,error));
    std::vector<ArchiveProfile> decoded;
    assert(decodeArchive(packed.data(),packed.size(),decoded,error));
    assert(decoded.size()==1);
    assert(decoded[0].frame.timing==PoseTiming::Simulated);
    assert(decoded[0].frame.feedbackMask==0 && decoded[0].frame.latchMask==0);
    assert(decoded[0].frame.sensorToMachine==f.sensorToMachine);
    assert(decoded[0].points[1].index==1279);
    assert(decoded[0].points[1].z==112.);
    // Provenance cannot be faked by changing timing while claiming a latch.
    profile.frame.latchMask=31;
    assert(!encodeArchive({profile},packed,error));
    profile.frame.latchMask=0;
    profile.frame.feedbackMask=31;
    assert(!encodeArchive({profile},packed,error));
    profile.frame.feedbackMask=0;
    // Real hardware capture: an arbitrary simulated transform is NOT allowed.
    profile.frame.timing=PoseTiming::HardwareLatched;
    profile.frame.feedbackMask=31;profile.frame.latchMask=31;
    assert(!encodeArchive({profile},packed,error));
    // A genuinely consistent hardware profile continues to round-trip under
    // the original strict Physical17 rules (we never weakened that path).
    {
        ArchiveProfile real=profile;
        real.frame.timing=PoseTiming::HardwareLatched;
        real.frame.feedbackMask=31;real.frame.latchMask=31;
        std::string why;
        assert(completeFrame(real.frame,real.frame.physicalParameters,why));
        assert(encodeArchive({real},packed,error));
        assert(decodeArchive(packed.data(),packed.size(),decoded,error));
        assert(decoded[0].frame.timing==PoseTiming::HardwareLatched);
        assert(decoded[0].frame.sensorToMachine==real.frame.sensorToMachine);
    }
    // CRC corruption must still be detected for simulation.
    profile.frame.timing=PoseTiming::Simulated;
    profile.frame.feedbackMask=0;profile.frame.latchMask=0;
    assert(encodeArchive({profile},packed,error));
    packed[24]^=0x30;
    assert(!decodeArchive(packed.data(),packed.size(),decoded,error));
    std::cout<<"M6 simulation archive provenance, exact pose, indices, CRC PASS\n";
}
