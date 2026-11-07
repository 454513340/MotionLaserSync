#pragma once
// Offline M6 five-axis NC preview. This is never a machine authorization.
// The generated file is explicitly marked OfflineOnly and must be rejected by
// GCodeExecutor::start(), even if a user copies it into the G-code editor.
#include "twin/TwinM6Continuous.h"
#include <string>

namespace mls::twin {
struct M6NcPreviewOptions {
    // An explicit H1/H3 proposal is required to serialize NC;
    // ID 2 is retired and 0 means no proposal (rejected).
    int proposedTopologyId = 0;
    int h3R1Edge = 114;     // 114 or 115, relevant only to H3
    std::string sourcePartPoseId;
    std::string calibrationId;
    std::string physical17Sha256;
    // Source is not itself a certified part pose or a calibration identifier.
    std::string sourceCadPath;
    std::string kinematicsSource;
    std::string studyId;          // optional traceability-only experiment identifier
    std::string scanGroupLabel;   // ASCII basename for M60, never a controller path
};
struct M6NcPreviewResult {
    std::string text;
    std::string error;
    unsigned int scanningGroups=0;
    unsigned int transferGroups=0;
    std::size_t encodedMotionPoints=0;
    bool ok() const { return error.empty() && !text.empty(); }
};
// Convert the SAME accepted M6Report.path into an inspectable, parsable NC
// preview. Preserve every 5-axis sample and scan/transfer role, with nominal
// per-segment durations expressed by G93 inverse-minute F (NOT a guarantee of
// controller timing or metrological profile spacing).
M6NcPreviewResult makeM6NcPreview(const M6Report& report,
                                  const M6NcPreviewOptions& options={});
} // namespace mls::twin
