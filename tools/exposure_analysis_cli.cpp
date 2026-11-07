#include "calibration/ExposureAnalysisEngine.h"

#include <filesystem>
#include <iostream>
#include <string>

int main(int argc, char** argv)
{
    if (argc < 2 || argc > 3) {
        std::cerr << "Usage: exposure_analysis_cli <session-directory> [output-directory]\n";
        return 2;
    }

    const std::filesystem::path sessionDir = std::filesystem::u8path(argv[1]);
    const std::filesystem::path outputDir = argc >= 3 ? std::filesystem::u8path(argv[2]) : sessionDir;
    const auto result = mls::calibration::ExposureAnalysisEngine::analyzeDirectory(sessionDir);
    if (!result.ok) {
        std::cerr << "Analysis failed: " << result.error << '\n';
        return 1;
    }

    std::filesystem::create_directories(outputDir);
    std::string error;
    const auto csv = outputDir / "exposure_analysis.csv";
    const auto json = outputDir / "exposure_analysis.json";
    if (!mls::calibration::ExposureAnalysisEngine::writeSummaryCsv(result, csv, &error)) {
        std::cerr << error << '\n';
        return 1;
    }
    if (!mls::calibration::ExposureAnalysisEngine::writeResultJson(result, json, &error)) {
        std::cerr << error << '\n';
        return 1;
    }

    std::cout << "Profile files analyzed: " << result.profileFilesAnalyzed << '\n';
    std::cout << "Recommended exposure: " << result.recommendedExposure10us
              << " x10us (" << result.recommendedExposure10us * 0.01 << " ms)\n";
    std::cout << "Saturation onset: " << result.saturationOnsetExposure10us << " x10us\n";
    std::cout << "Safe candidate band: " << result.safeCandidateMinExposure10us
              << ".." << result.safeCandidateMaxExposure10us << " x10us\n";
    std::cout << "Inferred Maximum ceiling: " << result.inferredSaturationCeiling << '\n';
    std::cout << "Confidence: " << result.confidence << '\n';
    std::cout << "Rationale: " << result.rationale << '\n';
    std::cout << "CSV: " << csv.u8string() << '\n';
    std::cout << "JSON: " << json.u8string() << '\n';
    return 0;
}
