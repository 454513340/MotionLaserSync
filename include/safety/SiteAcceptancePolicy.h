#pragma once

#include "twin/TwinTypes.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

namespace mls::safety {

// These values are observations entered and explicitly attested by a qualified
// on-site operator. They are not inferred from the NC header or CAD preview.
// This policy checks mathematical consistency and required evidence; the
// operator remains responsible for the authenticity of physical measurements.
struct SiteAcceptanceInput {
    mls::twin::Pose6 independentlyMeasuredCadToMcs{};
    double measuredMinimumFixtureClearanceMm = std::numeric_limits<double>::quiet_NaN();
    bool independentMeasurementConfirmed = false;
    bool partAndFixturesSecured = false;
    bool entireSweptEnvelopeInspected = false;
    bool guardingAndEStopVerified = false;
    std::string measurementMethod;
    std::string operatorIdentification;
};

struct SiteAcceptanceVerdict {
    bool accepted = false;
    double worstCornerPositionErrorMm = std::numeric_limits<double>::infinity();
    double allowedPositionErrorMm = 0.0;
    double minimumRequiredFixtureClearanceMm = 0.0;
    std::string reason;
};

inline SiteAcceptanceVerdict evaluateSiteAcceptance(
    const SiteAcceptanceInput& evidence,
    const mls::twin::TriangleMesh& mesh,
    const mls::twin::Pose6& checkedCadPose,
    double requiredStaticClearanceMm)
{
    SiteAcceptanceVerdict out;
    const auto finite=[](double d) {return std::isfinite(d);};
    out.allowedPositionErrorMm = std::min(0.5, requiredStaticClearanceMm * 0.5);
    out.minimumRequiredFixtureClearanceMm = std::max(2.0, requiredStaticClearanceMm * 2.0);
    if(mesh.empty() || !mesh.bounds.valid() || !finite(requiredStaticClearanceMm) ||
       requiredStaticClearanceMm <= 0.0 || out.allowedPositionErrorMm < 0.05) {
        out.reason = "Invalid mesh or insufficient collision clearance configuration";
        return out;
    }
    if(evidence.operatorIdentification.empty() || evidence.measurementMethod.size() < 8) {
        out.reason = "Site operator ID and independent measurement method are required";
        return out;
    }
    if(!evidence.independentMeasurementConfirmed || !evidence.partAndFixturesSecured ||
       !evidence.entireSweptEnvelopeInspected || !evidence.guardingAndEStopVerified) {
        out.reason = "Independent placement, workholding, whole-path fixtures and guarding/E-stop require confirmation";
        return out;
    }
    const auto& measured=evidence.independentlyMeasuredCadToMcs;
    for(double v:{checkedCadPose.x,checkedCadPose.y,checkedCadPose.z,
                  checkedCadPose.rxDeg,checkedCadPose.ryDeg,checkedCadPose.rzDeg,
                  measured.x,measured.y,measured.z,
                  measured.rxDeg,measured.ryDeg,measured.rzDeg}){
        if(!finite(v)) {out.reason="Non-finite CAD or independently measured pose";return out;}
    }
    if(!finite(evidence.measuredMinimumFixtureClearanceMm) ||
       evidence.measuredMinimumFixtureClearanceMm < out.minimumRequiredFixtureClearanceMm) {
        out.reason = "The measured clearance to unmodelled fixtures is below the site acceptance requirement";
        return out;
    }
    const auto model=mls::twin::Transform::fromPose(checkedCadPose);
    const auto actual=mls::twin::Transform::fromPose(measured);
    const auto a=mesh.bounds.min,b=mesh.bounds.max;
    double worst=0.0;
    for(double x:{a.x,b.x})for(double y:{a.y,b.y})for(double z:{a.z,b.z}){
        const auto p=model.point({x,y,z});
        const auto q=actual.point({x,y,z});
        const double error=mls::twin::length(p-q);
        if(!finite(error)) {out.reason="Invalid transformed workpiece bounds";return out;}
        worst=std::max(worst,error);
    }
    out.worstCornerPositionErrorMm=worst;
    if(worst>out.allowedPositionErrorMm+1.0e-9){
        out.reason="Measured CAD-to-machine registration disagrees with the STEP collision scene";
        return out;
    }
    out.accepted=true;
    out.reason="Measured placement and documented site inspection conform to the collision-check configuration";
    return out;
}

} // namespace mls::safety
