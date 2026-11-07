#pragma once
// V28.9 M8: OFFLINE workflow orchestration and predicted coverage acceptance.
// Never converts simulated M6 returns into measured M2 data or authorizes motion.
#include "twin/TwinM7Fusion.h"
#include <cstdint>
#include <string>
#include <vector>

namespace mls::twin {

enum class M8Stage : std::uint8_t {
    VirtualM3=3, ComparisonM4=4, CoverageM5=5,
    ChamferM51=51, GeometryM52=52, ContinuousM6=6, FusionM7=7
};

const char* m8StageName(M8Stage stage) noexcept;

struct M8WorkflowOptions {
    bool simulateM3=true;
    bool compareM4=true;
    bool inspectChamferM51=true;   // H1-specific: disable on other CAD models
    bool inspectGeometryM52=true;  // H1-specific: diagnostic only, never auto-apply offsets
    bool reuseValidResults=true;
    double coverageGoal=0.95;
};

struct M8WorkflowPlan {
    std::vector<M8Stage> stages;
    std::string error;
    bool ok() const noexcept {return error.empty()&&!stages.empty();}
};

// M0/M1 placement and the REAL M2 file are mandatory. This deliberately
// refuses to use synthetic M6 .scan volumes as a replacement for M2.
struct M8InputState {
    bool cadPlaced=false;
    bool realM2Loaded=false;
    bool m2SourceIsSimulated=false;
    bool otherTaskBusy=false;
    bool hasM3=false;
    bool h1SpecificGeometry=true;
};
struct M8Preflight {
    bool permitted=false;
    std::vector<std::string> blockers;
    std::vector<std::string> warnings;
};

M8WorkflowPlan makeM8Plan(const M8WorkflowOptions& options);
M8Preflight inspectM8Preflight(const M8WorkflowPlan& plan,
                               const M8WorkflowOptions& options,
                               const M8InputState& input);

struct M8FeaturePriority {
    std::uint16_t id=0;
    std::string feature;
    double targetAreaMm2=0;
    double stillMissingAreaMm2=0;
    double simulatedNewAreaMm2=0;
    double fractionMissing=0;
};
struct M8Acceptance {
    double targetAreaMm2=0;
    double realAreaMm2=0;
    double simulatedNewAreaMm2=0;
    double stillMissingAreaMm2=0;
    double predictedFraction=0;
    double targetFraction=0;
    bool coverageBookkeepingValid=false;
    bool predictedTargetReached=false;
    bool opticalMultiAngleValidated=false;
    bool fullFixtureCollisionCertified=false;
    bool hardwareAuthorized=false; // always false, by construction
    bool requiresAdditionalView=false;
    std::vector<M8FeaturePriority> priorities;
    std::string error;
    bool ok() const noexcept {return error.empty()&&coverageBookkeepingValid;}
};

M8Acceptance evaluateM8Acceptance(const M5Report& m5,
                                   const M6Report& m6,
                                   const M7Coverage& m7,
                                   double goalFraction);
} // namespace mls::twin
