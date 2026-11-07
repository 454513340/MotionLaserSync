#include "twin/NcG93MotionAudit.h"
#include <cmath>
#include <iostream>
#include <string>
#define REQUIRE(x) do { if(!(x)) { std::cerr << "FAIL " << __LINE__ << ": " << #x << "\n"; return 1; } } while(0)

static std::string nc(double f) {
    const std::string feed=std::to_string(f);
    return "; OfflineOnly: true\n"
           "; SyncMode: Hardware\n"
           "; HardwareTopology: H3\n"
           "; HardwareFrequencyHz: 100\n"
           "; ProgramFrame: MACHINE_JOINTS_XYZCB\n"
           "; CmmR1Edge: 114\n"
           "G21\nG90\nG93\n"
           "G00 X0 Y0 Z0 C0 B0\n"
           "M50\n"
           "G05 S1 F" + feed + "\n"
           "X1 Y0 Z0 C0 B0 F" + feed + "\n"
           "X2 Y0 Z0 C0 B0 F" + feed + "\n"
           "M51\nM60 L\"a\"\nM61\nM63\nM30\n";
}
int main() {
    const mls::runtime::Limits l{};
    const auto fast=mls::twin::simulateMultiPoseNc(nc(300.174469661));
    REQUIRE(fast.ok());
    const auto reject=mls::twin::auditNcG93Motion(fast,l,25,25);
    REQUIRE(!reject.passed && reject.failedNcLine==13);
    REQUIRE(reject.error.find("G93 time infeasible")!=std::string::npos);
    REQUIRE(reject.error.find("minimum=0.400000")!=std::string::npos);
    const auto candidate=mls::twin::retimeNcG93Conservative(nc(300.174469661),fast,l,25,25);
    REQUIRE(candidate.ok() && candidate.changedMotionPoints==2);
    REQUIRE(candidate.text.find("; OfflineOnly: true")!=std::string::npos);
    REQUIRE(candidate.text.find("CONSERVATIVE_EXACT_STOP_CANDIDATE_NOT_REAL_RUN_PERMIT")!=std::string::npos);
    const auto reSim=mls::twin::simulateMultiPoseNc(candidate.text);
    REQUIRE(reSim.ok());
    REQUIRE(reSim.vertices.size()==fast.vertices.size());
    REQUIRE(reSim.vertices.back().joints==fast.vertices.back().joints);
    REQUIRE(mls::twin::auditNcG93Motion(reSim,l,25,25).passed);
    REQUIRE(candidate.retimedSeconds>candidate.originalSeconds);
    REQUIRE(!mls::twin::retimeNcG93Conservative(nc(300.174469661),fast,l,25,25,0.0).ok());
    const auto slow=mls::twin::simulateMultiPoseNc(nc(120));
    REQUIRE(slow.ok());
    const auto accepted=mls::twin::auditNcG93Motion(slow,l,25,25);
    REQUIRE(accepted.passed && accepted.segmentsChecked==2 && accepted.groupsChecked==1);
    REQUIRE(std::abs(accepted.nominalSeconds-1.0)<1.e-9);
    // Software-configured limits must be honored; a different controller
    // acceleration cap cannot be bypassed by cached NC preflight results.
    auto lower=l;lower.accel[0]=5;lower.decel[0]=5;
    const auto rejectLowAccel=mls::twin::auditNcG93Motion(slow,lower,25,25);
    REQUIRE(!rejectLowAccel.passed);
    std::cout << "G93 early dynamics preflight: unsafe fast segment blocked, feasible slow path accepted\n";
}
