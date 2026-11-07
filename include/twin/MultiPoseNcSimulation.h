#pragma once
// OFFLINE, fail-closed interpreter for the exact G05/G93 five-axis NC dialect
// produced by makeM6NcPreview(). It uses the shared runtime parseNcBlock()
// lexer/interpreter grammar, NEVER the live GCodeExecutor or a controller.
// An unanchored first G00 is treated as an unknown approach, NOT a proven move.
#include "twin/TwinM6Continuous.h"
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace mls::twin {
struct NcSimVertex {
    Joint5 joints{};
    double seconds=0.;
    bool scanning=false;                 // incoming movement segment is exposed
    std::size_t sourceLine=0;
};
struct NcSimProfile {
    Joint5 joints{};
    double seconds=0.;                  // ideal trigger instant in NC time
    std::size_t scanGroup=0;
    std::size_t sourceLine=0;
};
struct NcSimEvent {
    std::string code;
    double seconds=0.;
    std::size_t sourceLine=0;
};
struct NcSimResult {
    std::string error;
    std::string warning;
    int topology=0;
    int profileRateHz=0;
    int h3Edge=0;
    bool initialG00Unverified=true;
    bool reachedM30=false;
    bool sawM63=false;
    double durationSeconds=0.;
    std::size_t scanGroups=0;
    std::size_t saveGroups=0;
    std::vector<NcSimVertex> vertices;
    std::vector<NcSimProfile> profiles;
    std::vector<NcSimEvent> events;
    bool ok() const {return error.empty()&&reachedM30&&vertices.size()>1&&!profiles.empty();}
};

// Parses HEADER, modal G codes, every five-axis point and M50/M51/M60/M61/M63/M30.
// For generated G93 NC: duration of EACH G05 point is 60/F seconds. M50 opens
// an ideal periodic profile clock, M51 closes it; no real H1/H3 latency is claimed.
NcSimResult simulateMultiPoseNc(const std::string& ncText,
                                std::size_t maxProfiles=200000);

// Compare reinterpreted NC point-by-point with the unmodified planning trace.
// A mismatch blocks *simulation*, not only eventual hardware export.
std::string verifyMultiPoseNcAgainstPlan(const NcSimResult& nc,
                                         const M6Report& plan,
                                         double jointTolerance=1.e-6,
                                         double timeToleranceSeconds=1.e-4);

// Deterministic NC playback interpolation, NOT interpolation of the original plan.
struct NcSimSample {
    Joint5 joints{};
    double seconds=0.;
    bool valid=false,scanning=false;
    std::size_t nextVertex=0;
    std::size_t passedProfiles=0;
};
NcSimSample sampleMultiPoseNc(const NcSimResult& nc,double seconds);
}
