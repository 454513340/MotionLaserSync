#pragma once

#include <cstddef>
#include <filesystem>
#include <string>
#include <vector>

namespace mls::calibration {

struct ExposureAnalysisConfig {
    // Invalid scanCONTROL profile points are represented by Z==0 in the
    // sensor-calibration CSVs. Valid points are split into contiguous surface
    // segments so bright side/background surfaces cannot dominate the target.
    double segmentZJumpMm = 5.0;
    std::size_t minimumSegmentPoints = 8;

    // A clipped stripe produces many identical Maximum samples at the frame's
    // top value. The threshold is intentionally inferred from the sweep rather
    // than hard-coding the UI PeakFilter maximum (1023).
    double saturationPlateauTolerance = 1.0;
    double saturationPlateauFractionThreshold = 0.05;

    // Knee selection: enough geometric coverage and enough signal while still
    // remaining before the first clipped/saturated exposure.
    double preferredCoverageFraction = 0.95;
    double preferredSignalFraction = 0.85;
    double safeCoverageFraction = 0.90;
    double safeSignalFraction = 0.70;
};

struct ExposureFrameMetrics {
    int exposure10us = 0;
    int frameIndex = 0;
    int targetPointCount = 0;
    double targetXSpanMm = 0.0;
    double maximumMean = 0.0;
    double maximumMedian = 0.0;
    double maximumP10 = 0.0;
    double maximumP90 = 0.0;
    double maximumMax = 0.0;
    double maxPlateauFraction = 0.0;
    double widthMedian = 0.0;
    double widthP90 = 0.0;
};

struct ExposureLevelMetrics {
    int exposure10us = 0;
    int profileFrames = 0;
    double targetPointsMean = 0.0;
    double targetPointsStd = 0.0;
    double targetXSpanMm = 0.0;
    double maximumMean = 0.0;
    double maximumMedian = 0.0;
    double maximumP10 = 0.0;
    double maximumP90 = 0.0;
    double maximumMax = 0.0;
    double maxPlateauFraction = 0.0;
    double widthMedian = 0.0;
    double widthP90 = 0.0;
    double coverageVsSessionMax = 0.0;
    double signalVsSessionMax = 0.0;
};

struct ExposureAnalysisResult {
    bool ok = false;
    std::string error;
    std::string sourceDirectory;
    int profileFilesAnalyzed = 0;
    int recommendedExposure10us = 0;
    int saturationOnsetExposure10us = 0;
    int safeCandidateMinExposure10us = 0;
    int safeCandidateMaxExposure10us = 0;
    double inferredSaturationCeiling = 0.0;
    std::string confidence;
    std::string rationale;
    std::vector<ExposureFrameMetrics> frames;
    std::vector<ExposureLevelMetrics> levels;
};

class ExposureAnalysisEngine {
public:
    static ExposureAnalysisResult analyzeDirectory(
        const std::filesystem::path& directory,
        const ExposureAnalysisConfig& config = ExposureAnalysisConfig{});

    static bool writeSummaryCsv(const ExposureAnalysisResult& result,
                                const std::filesystem::path& filePath,
                                std::string* error = nullptr);
    static bool writeResultJson(const ExposureAnalysisResult& result,
                                const std::filesystem::path& filePath,
                                std::string* error = nullptr);
};

} // namespace mls::calibration
