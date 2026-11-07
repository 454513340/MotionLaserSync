#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace mls::calibration {

// Portable, sensor-frame-only geometry. No timing calibration, hand-eye transform,
// ECI FIFO, or production material library is modified by this evaluator.
struct SphereExposurePoint {
    double x = 0.0;
    double z = 0.0;
    int index = 0;
};

struct SphereExposureArc {
    bool valid = false;
    double centerX = 0.0;
    double centerZ = 0.0;
    double radius = 0.0;
    double rms = 0.0;
    double arcSpanDeg = 0.0;
    std::size_t inliers = 0;
};

struct SphereExposureFrame {
    unsigned int index = 0;
    double pcUs = 0.0;
    SphereExposureArc arc;
    // Optional LLT rich channels; sampled only when ordinal matches this Profile.
    bool signalAvailable = false;
    double maximumMean = 0.0;
    double widthMedian = 0.0;
    double maximumPlateauFraction = 0.0;
};

struct SphereExposurePass {
    int exposure10us = 0;
    int rateHz = 0;
    int repeat = 0;
    int direction = 1;
    double speedMmPerSec = 0.0;
    double nominalSpacingMm = 0.0;
    double knownDiameterMm = 19.05;
    std::size_t frames = 0;
    std::size_t acceptedFrames = 0;
    double observedMaxDiameterMm = 0.0;
    double fittedPeakDiameterMm = 0.0;
    double medianArcRmsMm = 0.0;
    double medianArcSpanDeg = 0.0;
    double validFraction = 0.0;
    std::size_t signalFrames = 0;
    double medianMaximum = 0.0;
    double medianWidth = 0.0;
    double medianMaximumPlateauFraction = 0.0;
    bool peakInterpolated = false;
    bool complete = false;
    std::string note;
};

struct SphereExposureRank {
    int exposure10us = 0;
    int passes = 0;
    bool eligible = false;
    double medianDiameterMm = 0.0;
    double diameterErrorMm = 0.0;
    double diameterRepeatabilityMm = 0.0;
    double medianArcRmsMm = 0.0;
    double coverageFraction = 0.0;
    double signalCoverageFraction = 0.0;
    double medianMaximumPlateauFraction = 0.0;
    double medianWidth = 0.0;
    double score = 0.0; // lower is better; not a manufacturer accuracy certificate
    std::string note;
};

class SphereExposureSweep {
public:
    static SphereExposureArc fitArc(const std::vector<SphereExposurePoint>& points,
                                    double expectedCx, double expectedCz,
                                    double sphereRadiusMm);
    static SphereExposurePass summarize(const std::vector<SphereExposureFrame>& frames,
                                         int exposure10us, int rateHz, int repeat,
                                         int direction, double speedMmPerSec,
                                         double knownDiameterMm = 19.05);
    static std::vector<SphereExposureRank> rank(const std::vector<SphereExposurePass>& passes,
                                                double knownDiameterMm = 19.05);
};

} // namespace mls::calibration
