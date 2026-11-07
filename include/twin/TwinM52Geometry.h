#pragma once
// V28.6 / M5.2: OFFLINE H1 geometric consistency diagnosis only.
// Does NOT change CAD, machine pose, calibration, or controller commands.
// Model: stable CAD top plane + angularly balanced center chamfer sections.
#include "twin/TwinScene.h"
#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace mls::twin {
constexpr int M52AngularBins = 72;
struct M52Options {
    double topGridPitchMm = 0.30;
    double topGuardMm = 1.20;
    double topZGateMm = 1.0;
    double coneGateMm = 0.65;
    unsigned minTopCells = 350;
    unsigned minConePoints = 180;
    unsigned minFitAngularBins = 24;
};
struct M52AngleRecord {
    int angleStartDeg=0;
    std::uint32_t hits=0;
    bool training=false;
    double medianRawMm=0.;
    double medianTopCorrectedMm=0.;
    double medianAfterXYMm=0.;
    double medianAfterFullFitMm=0.; // diagnostic geometry intercept is removed here
};
struct M52Fit {
    bool ok=false;
    std::string error;
    std::uint64_t pointsSeen=0;
    std::uint32_t topCellsUsed=0;
    std::uint32_t conePointsUsed=0;
    std::uint32_t angularBinsUsed=0;
    std::uint32_t trainBins=0;
    std::uint32_t maxUnobservedArcDeg=360;
    std::uint32_t validationBins=0;
    // In CAD axes, fitted raw top surface: z_observed = cadTopZ + ax*x + by*y + c.
    double topSlopeX=0.,topSlopeY=0.,topInterceptMm=0.;
    double topOffsetAtHoleMm=0.;
    double topPlaneMadMm=0.;
    // Conical section AFTER top-plane correction: e = r + depth - R.
    // Fit e(theta)=dx*cos(theta)+dy*sin(theta)+intercept (mm).
    // dx/dy are exploratory candidate center shifts, NOT certified pose corrections.
    double candidateCenterShiftXmm=0., candidateCenterShiftYmm=0.;
    double coneInterceptMm=0.;
    double coneRawMedianMm=0.,coneAfterTopMedianMm=0.;
    double coneAfterXYMedianMm=0.,coneAfterXYMadMm=0.;
    double coneFitTrainMadMm=0.,coneFitValidationMadMm=0.;
    bool holdoutEvaluated=false;
    // Dimension/geometric consistency can be indeterminate even with a valid fit.
    bool enoughAngularSupport=false;
    bool canApplyAutomatically=false; // ALWAYS false for one fixed scan
    std::array<M52AngleRecord,M52AngularBins> angular{};
};
struct M52Report {
    M52Options options{};
    M52Fit real, virtualFit;
    bool hasVirtual=false;
    double nominalOuterRadiusMm=3.25;
    double nominalChamferDepthMm=.5;
    double nominalHoleXmm=0., nominalHoleYmm=-38.;
    double nominalTopZmm=10.;
    std::string limitation;
    std::string error;
    bool ok() const {return error.empty()&&real.ok;}
};

class TwinM52GeometryAnalyzer final {
public:
    explicit TwinM52GeometryAnalyzer(const TwinScene& scene,M52Options options={});
    bool ready() const {return report_.error.empty();}
    const std::string& error() const {return report_.error;}
    void addRealCad(Vec3 point);
    void addVirtualCad(Vec3 point);
    M52Report finish() &&;
private:
    struct Dataset {
        std::uint64_t seen=0;
        std::vector<Vec3> top; // spatially stratified, max one per safe top-grid cell
        std::vector<Vec3> cone; // <= 1200 samples per 5-degree interval
        std::vector<std::uint8_t> topUsed;
        std::array<unsigned,M52AngularBins> coneCounts{};
    };
    M52Report report_{};
    double topZ_=10.,bottomZ_=0.;
    double gridMinX_=0.,gridMinY_=0.;
    int gridW_=0,gridH_=0;
    std::vector<std::uint8_t> safeTop_;
    Dataset real_,virtual_;
    void add(Vec3 p,Dataset& data);
    M52Fit fit(const Dataset& data) const;
};
} // namespace mls::twin
