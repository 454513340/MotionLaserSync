#pragma once
// Offline initial CAD-to-machine registration PROPOSAL for a mostly horizontal
// part. It is deliberately NOT a certified 6DoF registration or a motion grant.
// The candidate is scored against an indexed CAD surface, and rejected if the
// scanned evidence is insufficient. Real sample provenance remains elsewhere.
#include "twin/TwinTypes.h"
#include <string>
#include <vector>

namespace mls::twin {
struct RegistrationAssistReport {
    Pose6 proposed{};
    std::size_t candidateSamples=0;
    std::size_t observedSamples=0;
    std::size_t inliers=0;
    double medianSurfaceDistanceMm=0;
    double fractionWithinOneMm=0;
    double orientationMargin=0; // best-vs-second score, NOT a probability
    bool proposedAvailable=false;
    bool orientationAmbiguous=true;
    std::string message;
};
// Assumptions: scanned top surface approximately parallel to machine XY;
// nominal CAD top surface along +CAD Z, coarse C/B zero or nearly zero.
// Searches four horizontal yaw hypotheses, NOT an arbitrary six-axis pose.
// If those assumptions do not hold, use a separately validated 6DoF alignment.
RegistrationAssistReport proposeTopSurfaceRegistration(const TriangleMesh& cad,
                  const std::vector<Vec3>& observedMachine,
                  std::size_t maximumSamples=2000);
}
