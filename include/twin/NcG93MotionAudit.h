#pragma once
// Offline dynamics audit for an already-parsed M6 G05/G93 XYZCB NC.
// Uses the SAME mls::runtime::plan() as ZmcController::prepareMergePath,
// without sending SDK commands or changing the user NC.
// IMPORTANT: This proves only nominal trajectory limits/acceleration under the
// configured planner. Actual controller jerk, servo limits and collision
// clearance must still be independently verified before real execution.
#include "twin/MultiPoseNcSimulation.h"
#include "runtime/FiveAxis.h"
#include <cstddef>
#include <string>

namespace mls::twin {
struct NcG93MotionAudit {
    bool passed = false;
    std::size_t segmentsChecked = 0;
    std::size_t groupsChecked = 0;
    std::size_t failedNcLine = 0;
    double nominalSeconds = 0.0;
    double plannedSeconds = 0.0;
    std::string error;
};

NcG93MotionAudit auditNcG93Motion(const NcSimResult& nc,
                                  const mls::runtime::Limits& limits,
                                  double accelerationCap,
                                  double decelerationCap);

struct NcG93RetimeCandidate {
    std::string text;
    std::string error;
    std::size_t changedMotionPoints=0;
    double originalSeconds=0.0;
    double retimedSeconds=0.0;
    bool ok() const {return error.empty() && !text.empty();}
};

// Create a NEW OFFLINE-ONLY candidate file. Coordinates, M50/M51, topology,
// scanning roles, and step geometry remain byte-for-byte except for the G93
// F terms and explicit candidate labels. This is intentionally conservative:
// every G05 point gets an independent 0-to-0 SP acceleration envelope. A
// new file hash requires all four live preflight/fixture checks to be redone.
// NO AUTOMATIC EXECUTION APPROVAL is granted by this transformation.
NcG93RetimeCandidate retimeNcG93Conservative(const std::string& originalText,
                                              const NcSimResult& parsed,
                                              const mls::runtime::Limits& limits,
                                              double accelerationCap,
                                              double decelerationCap,
                                              double margin=0.02);
// Continuous-SP engineering candidate: preserves the long straight scan speed
// and accounts for actual first/last acceleration, non-collinear corners and
// 5-axis axis-speed caps. Always a NEW file requiring full site re-validation.
NcG93RetimeCandidate retimeNcG93Continuous(const std::string& originalText,
                                            const NcSimResult& parsed,
                                            const mls::runtime::Limits& limits,
                                            double accelerationCap,
                                            double decelerationCap);
}
