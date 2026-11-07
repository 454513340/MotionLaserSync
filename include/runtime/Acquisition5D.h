#pragma once
#include "runtime/FiveAxis.h"
#include "calibration/SingleSphere17Calibration.h"
#include <array>
#include <cmath>
#include <cstdint>
#include <string>

namespace mls::runtime {

// Supported topology IDs: H1=1 (ECI PWM/IN0 R0), H3=3 (sensor internal CMM/IN1 R1).
// ID 2 is retired and intentionally INVALID; do not renumber H3 (TABLE protocol).
enum class HardwareSyncTopology : std::uint32_t {
    EciPwmShared = 1, SensorInternalCmm = 3
};

enum class PoseTiming : std::uint32_t {
    ControllerSequential = 1,
    SoftwareInterpolated = 2,
    Simulated = 3,
    HardwareLatched = 4,
    HardwareTemporalInterpolated = 5
};

enum class TablePoseVariant : std::uint32_t {
    FullMposLatched = 4
};

struct TableHeaderProfile {
    TablePoseVariant variant=TablePoseVariant::FullMposLatched;
    PoseTiming timing=PoseTiming::HardwareLatched;
    std::uint32_t feedbackMask=31;
    std::uint32_t latchMask=31;
    std::uint32_t commandMask=0;
};

// ECI3808 TABLE contract used by V32.13 SINGLE-OP14 / runtime-PWM / FULL-MPOS-LATCH.
//
// Physical wiring remains exactly the commissioned single-output topology:
//   OP14/PWM0 -> scanCONTROL external trigger AND -> ECI IN0 latch input.
//   scanCONTROL uses EdgeRising; ECI REGIST(104) captures R0 logical falling.
//   With the verified OP14->IN0 polarity, both correspond to the OP14 OFF edge.
//
// Controller/BASIC acquisition is designed exactly as the final grating topology:
// XYZ/C/B are ATYPE=4 and all five TABLE coordinates are REGIST(104) MPOS samples.
// During commissioning before XYZ gratings are connected, X/Y/Z MPOS may remain 0;
// this is intentional and still allows the OP14 -> sensor + five-axis latch chain to
// be verified using moving C/B axes.
//
// IMPORTANT separation of responsibilities:
// - Hardware-sync TABLE records are five-axis MPOS and are never relabelled as DPOS.
// - Ordinary measured pose / software-sync sampling also uses XYZ/C/B MPOS.
// - DPOS is still retained separately as the controller demand/command coordinate for
//   trajectory generation and following-error diagnostics; it is not a measured pose.
//
// Cold-start priming uses enable=3 and pulses the SAME OP14 line before the formal
// acquisition session. Prime profiles are discarded. A new session then clears and
// re-arms all five REGIST FIFOs before the official 1 Profile + 1 Pose probe.
struct Table5D {
    static constexpr int base=330, headerSize=22, data=370, dimension=6;
    static constexpr int capacity=96, tableSize=1600, magic=270909, legacyDefaultPeriodMs=5;
    static constexpr int defaultTriggerFrequencyHz=200, minTriggerFrequencyHz=25, maxTriggerFrequencyHz=2000;
    static constexpr int maxOrdinal=16777214;
    static constexpr std::uint32_t allAxesMask=31;

    // Hardware TABLE profile: all five values are measured feedback and latched.
    static constexpr std::uint32_t defaultFeedbackMask=31;
    static constexpr std::uint32_t defaultCommandMask=0;
    static constexpr std::uint32_t defaultLatchMask=31;

    // Independent upper-PC software sampling profile. All five axes are feedback
    // MPOS. Software sampling is sequential/interpolated rather than edge-latched,
    // therefore latchMask remains 0 even though the position source is MPOS.
    static constexpr std::uint32_t softwareSampleFeedbackMask=31;
    static constexpr std::uint32_t softwareSampleLatchMask=0;
    static constexpr std::uint32_t softwareSampleCommandMask=0;

    static constexpr int latchInput=0;
    static constexpr int triggerOutput=14;
    static constexpr int latchBridgeOutput=14; // same physical OP14 line
    static constexpr int latchMode=104; // continuous REGIST(100+4): R0 falling edge
    static constexpr int executionAuthorization=1;
    static constexpr int coldPrimeCapability=1; // TABLE[351]=1
    static constexpr int primeCounterOffset=22; // TABLE[352], runtime only
    static constexpr int fifoWrapCounterOffset=23; // TABLE[353], runtime only
    // V32.13 runtime control/status.  These cells live in the reserved gap before Buffer-B.
    static constexpr int runtimeCapabilityOffset=24;       // TABLE[354]
    static constexpr int requestedFrequencyHzOffset=25;    // TABLE[355], host -> BASIC
    static constexpr int appliedFrequencyHzOffset=26;      // TABLE[356], BASIC -> host ACK
    static constexpr int machineHomeMaskOffset=27;         // TABLE[357], bits X/Y/Z/C/B; cleared by BASIC startup/power cycle
    static constexpr int linearHomeMaskOffset=machineHomeMaskOffset; // compatibility alias for older call sites/tests
    static constexpr int controllerStateOffset=28;         // TABLE[358] BASIC -> host actual state
    static constexpr int frequencyRequestSequenceOffset=29; // TABLE[359] host -> BASIC commit-last
    static constexpr int frequencyAckSequenceOffset=30;     // TABLE[360] BASIC -> host
    static constexpr int frequencyConfigErrorOffset=31;     // TABLE[361] BASIC -> host
    static constexpr int runtimeStatusWords=8;              // TABLE[354..361]
    // RAM-LITE STOP extension; host reads five live FIFO heads itself.
    // 362=1316 marker, 363=phase(0 reset/1 draining/2 done/3 fault),
    // 364=processed head (gr); host queries all five RAW heads separately.
    static constexpr int stopTailMarkerOffset=32;  // TABLE[362]
    static constexpr int stopTailWords=3;          // TABLE[362..364]
    static constexpr int stopTailMarker=1316;
    enum StopTailPhase { TailReset=0, TailDraining=1, TailDrained=2, TailFailure=3 };

    static constexpr int maxRequestSequence=16777214;
    static constexpr int runtimeCapabilityMagic=321058; // V32.16 two-topology handshake
    static constexpr int topologyRequestIndex=365;
    static constexpr int edgeRequestIndex=366;
    static constexpr int topologyAppliedIndex=367;
    static constexpr int edgeAppliedIndex=368;
    static constexpr int cmmR1Rising=114;
    static constexpr int cmmR1Falling=115;
    static constexpr int h1R0Falling=104;
    static bool validTopologyEdge(int mode,int edge) {
        return (mode==1 && edge==h1R0Falling) ||
               (mode==3 && (edge==cmmR1Rising || edge==cmmR1Falling));
    }
    static bool modeReady(const float* mode, int wantedTopology, int wantedEdge) {
        return mode && validTopologyEdge(wantedTopology,wantedEdge) &&
            integer(mode[0],1,3) && integer(mode[1],104,115) &&
            integer(mode[2],1,3) && integer(mode[3],104,115) &&
            mode[0]==wantedTopology && mode[1]==wantedEdge &&
            mode[2]==wantedTopology && mode[3]==wantedEdge;
    }
    enum RuntimeState { IdleReady=0, Running=1, Probe=2, ColdPrime=3,
                        Stopping=4, ConfigError=5, Fault=6 };

    // TABLE[354..361], with a controller-validated status and ACK sequence.
    // The host must never treat TABLE[334]=0 alone as evidence that PWM is off.
    static bool runtimeReady(const float* v, int expectedHz, int expectedState) {
        if (!v || !integer(v[0],runtimeCapabilityMagic,runtimeCapabilityMagic) ||
            !integer(v[1],minTriggerFrequencyHz,maxTriggerFrequencyHz) ||
            !integer(v[2],minTriggerFrequencyHz,maxTriggerFrequencyHz) ||
            !integer(v[3],0,static_cast<int>(allAxesMask)) ||
            !integer(v[4],IdleReady,Fault) ||
            !integer(v[5],1,maxRequestSequence) || !integer(v[6],1,maxRequestSequence) ||
            !integer(v[7],0,0)) return false;
        return v[1]==expectedHz && v[2]==expectedHz &&
            v[4]==expectedState && v[5]==v[6];
    }

    // Header layout:
    // [0] capacity   [1] writeIndex [2] readIndex  [3] fault
    // [4] enable: 0=idle,1=continuous,2=one-shot probe,3=sensor cold-prime
    // [5] published  [6] publishedLatchCount [7] magic=270909
    // [8] recordWords [9] legacy default period marker (=5 ms; NOT the active runtime rate) [10] sessionReq [11] sessionAck
    // [12] hardware-sync execution authorization=1
    // [13] feedbackMask=31 [14] HardwareLatched=4 [15] latchMask=31
    // [16] commandMask=0  [17] latchInput=0 [18] sensorTriggerOutput=14
    // [19] REGIST mode=104 [20] latchBridgeOutput=14 (same physical line)
    // [21] cold-prime capability=1. TABLE[352] is a runtime prime-pulse counter.
    static bool integer(float v, int lo, int hi) {
        return std::isfinite(v) && v>=lo && v<=hi && std::floor(v)==v;
    }
    static bool executionAuthorized(const float* h) {
        return h && h[12]==static_cast<float>(executionAuthorization);
    }
    static bool provenance(float feedback, float command) {
        if(!integer(feedback,0,31) || !integer(command,0,31)) return false;
        const auto f=static_cast<std::uint32_t>(feedback);
        const auto c=static_cast<std::uint32_t>(command);
        return (f&c)==0 && (f|c)==allAxesMask;
    }
    static bool decodeProfile(const float* h, TableHeaderProfile& p, std::string& e) {
        if(!h || !provenance(h[13],h[16])) { e="invalid TABLE XYZCB provenance masks"; return false; }
        const auto f=static_cast<std::uint32_t>(h[13]);
        const auto c=static_cast<std::uint32_t>(h[16]);
        const int mode = static_cast<int>(h[19]);
        const bool validR0 = mode==h1R0Falling && h[17]==0 && h[20]==14;
        const bool validR1 = (mode==cmmR1Rising || mode==cmmR1Falling) && h[17]==1 && h[20]==0;
        if(f!=31 || c!=0 || h[14]!=4 || h[15]!=31 || (!validR0 && !validR1)) {
            e="V32.16 H1/H3 requires five-axis MPOS latch; R0=104/IN0 or R1=114/115/IN1";
            return false;
        }
        p.variant=TablePoseVariant::FullMposLatched;
        p.timing=PoseTiming::HardwareLatched;
        p.feedbackMask=f;p.latchMask=defaultLatchMask;p.commandMask=c;
        return true;
    }
    static bool validate(const float* h, int session, std::string& e) {
        if (!h || h[0]!=capacity || h[7]!=magic || h[8]!=dimension || h[9]!=legacyDefaultPeriodMs ||
            h[18]!=triggerOutput ||
            h[21]!=coldPrimeCapability) {
            e="TABLE V32.16 H1/H3 full-MPOS/cold-prime base schema mismatch";return false;
        }
        TableHeaderProfile profile;
        if(!decodeProfile(h,profile,e)) return false;
        for(int k: {1,2}) if(!integer(h[k],0,capacity-1)){e="invalid TABLE ring index";return false;}
        for(int k: {3,4,5,6,10,11}) if(!integer(h[k],0,maxOrdinal)){e="non-integral TABLE header";return false;}
        if(!integer(h[12],0,1)){e="invalid TABLE hardware execution authorization";return false;}
        if(h[4]>3){e="invalid acquisition enable mode";return false;}
        if(session>=0 && (h[10]!=session || h[11]!=session)){e="TABLE session changed/unacknowledged";return false;}
        return true;
    }
};
static_assert(Table5D::data + Table5D::capacity * Table5D::dimension <= Table5D::tableSize);
// Measured-pose provenance is now consistent across software and hardware paths.
// Software samples are not hardware-latched, so only latchMask differs.
static_assert(Table5D::softwareSampleFeedbackMask == 31); // software XYZ/C/B MPOS
static_assert(Table5D::softwareSampleCommandMask == 0);
static_assert(Table5D::defaultFeedbackMask == 31);        // hardware TABLE XYZ/C/B MPOS
static_assert(Table5D::defaultCommandMask == 0);

struct Frame5D {
    Joint joints{}, command{}, raw{};
    std::array<double,2> rotaryOffsets{};
    std::array<double,16> sensorToMachine{};
    std::array<double,17> physicalParameters{};
    std::uint64_t session=0, ordinal=0;
    std::int64_t hostEstimateUs=0;
    PoseTiming timing=PoseTiming::HardwareLatched;
    std::uint32_t feedbackMask=Table5D::defaultFeedbackMask;
    std::uint32_t latchMask=Table5D::defaultLatchMask;
    std::uint32_t commandMask=Table5D::defaultCommandMask;
    bool absoluteGaugeKnown=false;

    // Signed H1/H3 temporal calibration provenance. Zero values mean RAW.
    // Positive offset: exposure effective AFTER latch; adjacent ordinal is next.
    // Negative offset: exposure effective BEFORE latch; adjacent ordinal is previous.
    // temporalNextOrdinal keeps its binary name for archive compatibility; for
    // negative delays this is the PREVIOUS ordinal, never the Probe ordinal 0.
    double temporalOffsetSec=0.0;
    double temporalAlpha=0.0;
    std::uint64_t temporalNextOrdinal=0;
};

inline bool finiteJoint(const Joint& q) { for(double v:q) if(!std::isfinite(v)) return false;return true; }
inline bool validTransform(const std::array<double,16>& t) {
    for(double v:t) if(!std::isfinite(v)) return false;
    if(std::abs(t[12])+std::abs(t[13])+std::abs(t[14])+std::abs(t[15]-1)>1e-10) return false;
    for(int r=0;r<3;++r)for(int c=0;c<3;++c){double v=0;for(int k=0;k<3;++k)v+=t[r*4+k]*t[c*4+k];if(std::abs(v-(r==c?1.:0.))>1e-8)return false;}
    double d=t[0]*(t[5]*t[10]-t[6]*t[9])-t[1]*(t[4]*t[10]-t[6]*t[8])+t[2]*(t[4]*t[9]-t[5]*t[8]);
    return std::abs(d-1)<1e-8;
}
inline std::array<double,3> mapProfilePoint(const Frame5D& f, double x, double z) {
    const auto& t=f.sensorToMachine;
    return {t[0]*x+t[2]*z+t[3], t[4]*x+t[6]*z+t[7], t[8]*x+t[10]*z+t[11]};
}
bool completeFrame(Frame5D& frame, const std::array<double,17>& parameters, std::string& error);
bool decodeRecord(const float* values, std::uint64_t expectedOrdinal, std::uint64_t session,
                  const std::array<double,2>& offsets, std::uint32_t feedbackMask,
                  std::uint32_t latchMask, std::uint32_t commandMask, PoseTiming timing,
                  Frame5D& frame, std::string& error);

// Convenience overload used by legacy/current software-synchronization reads.
inline bool decodeRecord(const float* values, std::uint64_t expectedOrdinal, std::uint64_t session,
                         const std::array<double,2>& offsets, Frame5D& frame, std::string& error) {
    return decodeRecord(values,expectedOrdinal,session,offsets,
        Table5D::softwareSampleFeedbackMask,Table5D::softwareSampleLatchMask,Table5D::softwareSampleCommandMask,
        PoseTiming::SoftwareInterpolated,frame,error);
}
} // namespace mls::runtime
