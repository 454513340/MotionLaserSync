// Standalone cross-platform contract regression for the H1/H3-only controller ABI.
// No physical controller is accessed. Build: c++ -std=c++17 -I../include h1_h3_only_contract_tests.cpp -o h1_h3_only_contract_tests
#include "runtime/Acquisition5D.h"
#include <array>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {
void require(bool cond, const char* description) {
    if (!cond) throw std::runtime_error(description);
}
}

int main() {
    try {
        using T = mls::runtime::Table5D;
        using Topology = mls::runtime::HardwareSyncTopology;
        static_assert(static_cast<int>(Topology::EciPwmShared) == 1);
        static_assert(static_cast<int>(Topology::SensorInternalCmm) == 3);
        static_assert(T::runtimeCapabilityMagic == 321058);

        require(T::validTopologyEdge(1, 104), "H1 must accept R0 falling edge 104");
        require(T::validTopologyEdge(3, 114), "H3 must accept R1 rising edge 114");
        require(T::validTopologyEdge(3, 115), "H3 must accept R1 falling edge 115");
        require(!T::validTopologyEdge(2, 104), "retired topology 2 must reject edge 104");
        require(!T::validTopologyEdge(2, 114), "retired topology 2 must reject edge 114");
        require(!T::validTopologyEdge(2, 115), "retired topology 2 must reject edge 115");
        require(!T::validTopologyEdge(1, 114), "H1 must reject H3 edge");
        require(!T::validTopologyEdge(3, 104), "H3 must reject H1 edge");

        const std::array<float,4> ackH1{1, 104, 1, 104};
        const std::array<float,4> ackH3{3, 114, 3, 114};
        const std::array<float,4> staleH2{2, 114, 2, 114};
        require(T::modeReady(ackH1.data(), 1, 104), "H1 topology ACK must pass");
        require(T::modeReady(ackH3.data(), 3, 114), "H3 topology ACK must pass");
        require(!T::modeReady(staleH2.data(), 2, 114), "H2 ACK must never pass");
        require(!T::modeReady(staleH2.data(), 3, 114), "stale H2 ACK must not be accepted as H3");

        std::array<float,8> runtime{321058, 200, 200, 31, 0, 1, 1, 0};
        require(T::runtimeReady(runtime.data(), 200, T::IdleReady), "V32.16 runtime handshake must pass");
        runtime[0] = 321057;
        require(!T::runtimeReady(runtime.data(), 200, T::IdleReady), "old V32.15 runtime must be rejected");

        std::array<float, T::headerSize> h{};
        h[0] = T::capacity;
        h[7] = T::magic;
        h[8] = T::dimension;
        h[9] = T::legacyDefaultPeriodMs;
        h[12] = T::executionAuthorization;
        h[13] = T::defaultFeedbackMask;
        h[14] = static_cast<int>(mls::runtime::PoseTiming::HardwareLatched);
        h[15] = T::defaultLatchMask;
        h[16] = T::defaultCommandMask;
        h[17] = 0;
        h[18] = 14;
        h[19] = T::h1R0Falling;
        h[20] = 14;
        h[21] = T::coldPrimeCapability;
        std::string reason;
        require(T::validate(h.data(), 0, reason), "H1 Full-MPOS TABLE header must remain valid");
        h[17] = 1;
        h[19] = T::cmmR1Rising;
        h[20] = 0;
        reason.clear();
        require(T::validate(h.data(), 0, reason), "H3 Full-MPOS TABLE header must remain valid");
        h[19] = T::cmmR1Falling;
        reason.clear();
        require(T::validate(h.data(), 0, reason), "H3 alternate R1 edge header must remain valid");

        std::cout << "PASS: H1/H3 IDs and R0/R1 edges; H2 IDs rejected; "
                     "V32.16 handshake and old BASIC rejection; H1/H3 TABLE schemas\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
