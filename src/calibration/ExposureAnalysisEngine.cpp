#include "calibration/ExposureAnalysisEngine.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <numeric>
#include <regex>
#include <sstream>
#include <utility>

namespace fs = std::filesystem;

namespace mls::calibration {
namespace {

struct ProfilePoint {
    int pointIndex = 0;
    double xMm = 0.0;
    double zMm = 0.0;
    double width = std::numeric_limits<double>::quiet_NaN();
    double maximum = std::numeric_limits<double>::quiet_NaN();
};

std::vector<std::string> splitCsvSimple(const std::string& line)
{
    // Sensor calibration CSVs are numeric and unquoted. Keeping this parser
    // deliberately small makes the portable CLI independent of Qt/vendor SDKs.
    std::vector<std::string> out;
    std::string current;
    for (const char ch : line) {
        if (ch == ',') {
            out.push_back(current);
            current.clear();
        } else {
            current.push_back(ch);
        }
    }
    out.push_back(current);
    return out;
}

bool parseDouble(const std::string& s, double& v)
{
    if (s.empty()) return false;
    try {
        std::size_t used = 0;
        v = std::stod(s, &used);
        return used == s.size() && std::isfinite(v);
    } catch (...) {
        return false;
    }
}

bool parseInt(const std::string& s, int& v)
{
    if (s.empty()) return false;
    try {
        std::size_t used = 0;
        const long value = std::stol(s, &used, 10);
        if (used != s.size()) return false;
        v = static_cast<int>(value);
        return true;
    } catch (...) {
        return false;
    }
}

double quantile(std::vector<double> values, double q)
{
    if (values.empty()) return 0.0;
    std::sort(values.begin(), values.end());
    if (q <= 0.0) return values.front();
    if (q >= 1.0) return values.back();
    const double pos = q * static_cast<double>(values.size() - 1);
    const std::size_t lo = static_cast<std::size_t>(std::floor(pos));
    const std::size_t hi = static_cast<std::size_t>(std::ceil(pos));
    const double t = pos - static_cast<double>(lo);
    return values[lo] * (1.0 - t) + values[hi] * t;
}

double mean(const std::vector<double>& values)
{
    if (values.empty()) return 0.0;
    return std::accumulate(values.begin(), values.end(), 0.0)
        / static_cast<double>(values.size());
}

double sampleStd(const std::vector<double>& values)
{
    if (values.size() < 2) return 0.0;
    const double m = mean(values);
    double ss = 0.0;
    for (const double v : values) {
        const double d = v - m;
        ss += d * d;
    }
    return std::sqrt(ss / static_cast<double>(values.size() - 1));
}

bool readProfileCsv(const fs::path& path,
                    int exposure10us,
                    int frameIndex,
                    const ExposureAnalysisConfig& config,
                    ExposureFrameMetrics& metrics,
                    std::string& error)
{
    std::ifstream in(path);
    if (!in) {
        error = "cannot open profile CSV: " + path.u8string();
        return false;
    }

    std::string line;
    std::vector<std::string> header;
    int idxPoint = -1, idxX = -1, idxZ = -1, idxWidth = -1, idxMaximum = -1;
    std::vector<ProfilePoint> points;

    while (std::getline(in, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line.empty() || line[0] == '#') continue;
        if (header.empty()) {
            header = splitCsvSimple(line);
            for (int i = 0; i < static_cast<int>(header.size()); ++i) {
                if (header[i] == "PointIndex") idxPoint = i;
                else if (header[i] == "X_mm") idxX = i;
                else if (header[i] == "Z_mm") idxZ = i;
                else if (header[i] == "Width") idxWidth = i;
                else if (header[i] == "Maximum") idxMaximum = i;
            }
            if (idxPoint < 0 || idxX < 0 || idxZ < 0 || idxMaximum < 0) {
                error = "profile CSV lacks PointIndex/X_mm/Z_mm/Maximum: " + path.u8string();
                return false;
            }
            continue;
        }

        const auto fields = splitCsvSimple(line);
        const auto get = [&fields](int idx) -> const std::string* {
            return (idx >= 0 && idx < static_cast<int>(fields.size())) ? &fields[idx] : nullptr;
        };
        const std::string* pPoint = get(idxPoint);
        const std::string* pX = get(idxX);
        const std::string* pZ = get(idxZ);
        const std::string* pMaximum = get(idxMaximum);
        if (!pPoint || !pX || !pZ || !pMaximum) continue;

        ProfilePoint p;
        if (!parseInt(*pPoint, p.pointIndex)
            || !parseDouble(*pX, p.xMm)
            || !parseDouble(*pZ, p.zMm)
            || !parseDouble(*pMaximum, p.maximum)) {
            continue;
        }
        if (idxWidth >= 0) {
            if (const std::string* pWidth = get(idxWidth)) parseDouble(*pWidth, p.width);
        }
        points.push_back(p);
    }

    if (points.empty()) {
        error = "profile CSV contains no usable numeric rows: " + path.u8string();
        return false;
    }

    std::vector<std::vector<ProfilePoint>> segments;
    std::vector<ProfilePoint> current;
    ProfilePoint previous;
    bool havePrevious = false;
    auto flush = [&]() {
        if (current.size() >= config.minimumSegmentPoints) segments.push_back(current);
        current.clear();
    };

    for (const auto& p : points) {
        // Current scanCONTROL calibration exports encode invalid/no-return Z as 0.
        if (!std::isfinite(p.xMm) || !std::isfinite(p.zMm) || p.zMm == 0.0
            || !std::isfinite(p.maximum)) {
            if (!current.empty()) flush();
            havePrevious = false;
            continue;
        }
        const bool discontinuity = havePrevious
            && ((p.pointIndex - previous.pointIndex) > 1
                || std::abs(p.zMm - previous.zMm) > config.segmentZJumpMm);
        if (discontinuity && !current.empty()) flush();
        current.push_back(p);
        previous = p;
        havePrevious = true;
    }
    if (!current.empty()) flush();

    if (segments.empty()) {
        error = "no contiguous valid target segment found: " + path.u8string();
        return false;
    }

    // V27.6.43 default ROI policy: choose the continuous valid surface segment
    // whose mean X is nearest the optical X=0 axis. This is what prevents the
    // bright left/right fixtures in the Natural_Sunny data from biasing the
    // material exposure decision. Future UI work can add explicit/manual ROI.
    const std::vector<ProfilePoint>* target = nullptr;
    double bestScore = std::numeric_limits<double>::infinity();
    for (const auto& segment : segments) {
        double sx = 0.0;
        for (const auto& p : segment) sx += p.xMm;
        const double meanX = sx / static_cast<double>(segment.size());
        const double score = std::abs(meanX);
        if (!target || score < bestScore - 1.0e-12
            || (std::abs(score - bestScore) < 1.0e-12 && segment.size() > target->size())) {
            target = &segment;
            bestScore = score;
        }
    }

    if (!target || target->empty()) {
        error = "target ROI selection failed: " + path.u8string();
        return false;
    }

    std::vector<double> maximums;
    std::vector<double> widths;
    maximums.reserve(target->size());
    widths.reserve(target->size());
    double minX = std::numeric_limits<double>::infinity();
    double maxX = -std::numeric_limits<double>::infinity();
    for (const auto& p : *target) {
        maximums.push_back(p.maximum);
        if (std::isfinite(p.width)) widths.push_back(p.width);
        minX = std::min(minX, p.xMm);
        maxX = std::max(maxX, p.xMm);
    }

    const double vmax = *std::max_element(maximums.begin(), maximums.end());
    int plateauCount = 0;
    for (const double v : maximums) {
        if (v >= vmax - config.saturationPlateauTolerance) ++plateauCount;
    }

    metrics.exposure10us = exposure10us;
    metrics.frameIndex = frameIndex;
    metrics.targetPointCount = static_cast<int>(target->size());
    metrics.targetXSpanMm = maxX - minX;
    metrics.maximumMean = mean(maximums);
    metrics.maximumMedian = quantile(maximums, 0.50);
    metrics.maximumP10 = quantile(maximums, 0.10);
    metrics.maximumP90 = quantile(maximums, 0.90);
    metrics.maximumMax = vmax;
    metrics.maxPlateauFraction = static_cast<double>(plateauCount)
        / static_cast<double>(maximums.size());
    metrics.widthMedian = widths.empty() ? 0.0 : quantile(widths, 0.50);
    metrics.widthP90 = widths.empty() ? 0.0 : quantile(widths, 0.90);
    return true;
}

template <typename Getter>
double aggregateMean(const std::vector<const ExposureFrameMetrics*>& frames, Getter getter)
{
    if (frames.empty()) return 0.0;
    double sum = 0.0;
    for (const auto* f : frames) sum += getter(*f);
    return sum / static_cast<double>(frames.size());
}

std::string jsonEscape(const std::string& input)
{
    std::ostringstream out;
    for (const unsigned char ch : input) {
        switch (ch) {
        case '"': out << "\\\""; break;
        case '\\': out << "\\\\"; break;
        case '\b': out << "\\b"; break;
        case '\f': out << "\\f"; break;
        case '\n': out << "\\n"; break;
        case '\r': out << "\\r"; break;
        case '\t': out << "\\t"; break;
        default:
            if (ch < 0x20) {
                out << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                    << static_cast<int>(ch) << std::dec << std::setfill(' ');
            } else {
                out << static_cast<char>(ch);
            }
        }
    }
    return out.str();
}

} // namespace

ExposureAnalysisResult ExposureAnalysisEngine::analyzeDirectory(
    const std::filesystem::path& directory,
    const ExposureAnalysisConfig& config)
{
    ExposureAnalysisResult result;
    result.sourceDirectory = directory.u8string();

    std::error_code ec;
    const fs::path root(directory);
    if (!fs::exists(root, ec) || !fs::is_directory(root, ec)) {
        result.error = "exposure session directory does not exist: " + result.sourceDirectory;
        return result;
    }

    const std::regex filenamePattern(
        R"(.*_exp([0-9]+)_profile_f([0-9]+)\.csv$)",
        std::regex::icase);
    struct FileRecord { fs::path path; int exposure = 0; int frame = 0; };
    std::vector<FileRecord> files;

    for (const auto& entry : fs::directory_iterator(root, ec)) {
        if (ec) break;
        if (!entry.is_regular_file()) continue;
        std::smatch match;
        const std::string name = entry.path().filename().u8string();
        if (!std::regex_match(name, match, filenamePattern) || match.size() < 3) continue;
        try {
            FileRecord r;
            r.path = entry.path();
            r.exposure = std::stoi(match[1].str());
            r.frame = std::stoi(match[2].str());
            files.push_back(std::move(r));
        } catch (...) {
        }
    }

    if (files.empty()) {
        result.error = "no *_expNNNN_profile_fNNN.csv files found in: " + result.sourceDirectory;
        return result;
    }

    std::sort(files.begin(), files.end(), [](const FileRecord& a, const FileRecord& b) {
        if (a.exposure != b.exposure) return a.exposure < b.exposure;
        return a.frame < b.frame;
    });

    for (const auto& f : files) {
        ExposureFrameMetrics metrics;
        std::string frameError;
        if (readProfileCsv(f.path, f.exposure, f.frame, config, metrics, frameError)) {
            result.frames.push_back(metrics);
        }
    }

    result.profileFilesAnalyzed = static_cast<int>(result.frames.size());
    if (result.frames.empty()) {
        result.error = "profile files were found, but none contained analyzable rich profile data";
        return result;
    }

    std::map<int, std::vector<const ExposureFrameMetrics*>> grouped;
    for (const auto& f : result.frames) grouped[f.exposure10us].push_back(&f);

    double sessionMaxCoverage = 0.0;
    double sessionMaxSignal = 0.0;
    for (const auto& [exposure, framesAtLevel] : grouped) {
        ExposureLevelMetrics level;
        level.exposure10us = exposure;
        level.profileFrames = static_cast<int>(framesAtLevel.size());

        std::vector<double> pointCounts;
        pointCounts.reserve(framesAtLevel.size());
        for (const auto* f : framesAtLevel) pointCounts.push_back(static_cast<double>(f->targetPointCount));
        level.targetPointsMean = mean(pointCounts);
        level.targetPointsStd = sampleStd(pointCounts);
        level.targetXSpanMm = aggregateMean(framesAtLevel, [](const auto& f) { return f.targetXSpanMm; });
        level.maximumMean = aggregateMean(framesAtLevel, [](const auto& f) { return f.maximumMean; });
        level.maximumMedian = aggregateMean(framesAtLevel, [](const auto& f) { return f.maximumMedian; });
        level.maximumP10 = aggregateMean(framesAtLevel, [](const auto& f) { return f.maximumP10; });
        level.maximumP90 = aggregateMean(framesAtLevel, [](const auto& f) { return f.maximumP90; });
        level.maximumMax = aggregateMean(framesAtLevel, [](const auto& f) { return f.maximumMax; });
        level.maxPlateauFraction = aggregateMean(framesAtLevel, [](const auto& f) { return f.maxPlateauFraction; });
        level.widthMedian = aggregateMean(framesAtLevel, [](const auto& f) { return f.widthMedian; });
        level.widthP90 = aggregateMean(framesAtLevel, [](const auto& f) { return f.widthP90; });
        sessionMaxCoverage = std::max(sessionMaxCoverage, level.targetPointsMean);
        sessionMaxSignal = std::max(sessionMaxSignal, level.maximumP90);
        result.levels.push_back(level);
    }

    for (auto& level : result.levels) {
        level.coverageVsSessionMax = sessionMaxCoverage > 0.0
            ? level.targetPointsMean / sessionMaxCoverage : 0.0;
        level.signalVsSessionMax = sessionMaxSignal > 0.0
            ? level.maximumP90 / sessionMaxSignal : 0.0;
    }

    for (const auto& level : result.levels) {
        if (level.maxPlateauFraction >= config.saturationPlateauFractionThreshold) {
            result.saturationOnsetExposure10us = level.exposure10us;
            break;
        }
    }

    std::vector<double> clippedCeilings;
    if (result.saturationOnsetExposure10us > 0) {
        for (const auto& level : result.levels) {
            if (level.exposure10us >= result.saturationOnsetExposure10us)
                clippedCeilings.push_back(level.maximumMax);
        }
    }
    if (!clippedCeilings.empty()) {
        result.inferredSaturationCeiling = quantile(clippedCeilings, 0.50);
    } else {
        for (const auto& level : result.levels) clippedCeilings.push_back(level.maximumMax);
        result.inferredSaturationCeiling = quantile(clippedCeilings, 1.0);
    }

    const auto beforeSaturation = [&result](const ExposureLevelMetrics& level) {
        return result.saturationOnsetExposure10us <= 0
            || level.exposure10us < result.saturationOnsetExposure10us;
    };

    std::vector<const ExposureLevelMetrics*> safeCandidates;
    for (const auto& level : result.levels) {
        if (!beforeSaturation(level)) continue;
        if (level.maxPlateauFraction >= config.saturationPlateauFractionThreshold) continue;
        if (level.coverageVsSessionMax + 1.0e-12 < config.safeCoverageFraction) continue;
        if (level.signalVsSessionMax + 1.0e-12 < config.safeSignalFraction) continue;
        safeCandidates.push_back(&level);
    }
    if (!safeCandidates.empty()) {
        result.safeCandidateMinExposure10us = safeCandidates.front()->exposure10us;
        result.safeCandidateMaxExposure10us = safeCandidates.back()->exposure10us;
    }

    const ExposureLevelMetrics* recommended = nullptr;
    for (const auto& level : result.levels) {
        if (!beforeSaturation(level)) continue;
        if (level.maxPlateauFraction >= config.saturationPlateauFractionThreshold) continue;
        if (level.coverageVsSessionMax + 1.0e-12 >= config.preferredCoverageFraction
            && level.signalVsSessionMax + 1.0e-12 >= config.preferredSignalFraction) {
            // Select the first knee meeting both coverage and signal targets;
            // there is no benefit in adding exposure after the useful plateau.
            recommended = &level;
            break;
        }
    }

    if (!recommended && result.saturationOnsetExposure10us > 0) {
        for (const auto& level : result.levels) {
            if (level.exposure10us < result.saturationOnsetExposure10us)
                recommended = &level;
        }
    }
    if (!recommended && !result.levels.empty()) {
        // No clipping was observed. Prefer the lowest level with near-maximal
        // coverage; otherwise fall back to the highest tested exposure.
        for (const auto& level : result.levels) {
            if (level.coverageVsSessionMax + 1.0e-12 >= config.preferredCoverageFraction) {
                recommended = &level;
                break;
            }
        }
        if (!recommended) recommended = &result.levels.back();
    }

    if (!recommended) {
        result.error = "unable to select a recommended exposure";
        return result;
    }

    result.recommendedExposure10us = recommended->exposure10us;
    const bool strongKnee = recommended->coverageVsSessionMax >= config.preferredCoverageFraction
        && recommended->signalVsSessionMax >= config.preferredSignalFraction
        && recommended->maxPlateauFraction < 0.01;
    const bool boundedBySaturation = result.saturationOnsetExposure10us > recommended->exposure10us;
    result.confidence = (strongKnee && boundedBySaturation) ? "high"
        : (strongKnee || boundedBySaturation) ? "medium" : "low";

    std::ostringstream rationale;
    rationale << "center-segment ROI; coverage=" << std::fixed << std::setprecision(3)
              << recommended->coverageVsSessionMax
              << ", signal=" << recommended->signalVsSessionMax
              << ", plateau=" << recommended->maxPlateauFraction;
    if (result.saturationOnsetExposure10us > 0) {
        rationale << "; first saturation/clipping onset at "
                  << result.saturationOnsetExposure10us << " x10us";
    } else {
        rationale << "; no saturation onset observed in tested sweep";
    }
    result.rationale = rationale.str();
    result.ok = true;
    return result;
}

bool ExposureAnalysisEngine::writeSummaryCsv(const ExposureAnalysisResult& result,
                                             const std::filesystem::path& filePath,
                                             std::string* error)
{
    std::ofstream out(filePath, std::ios::trunc);
    if (!out) {
        if (error) *error = "cannot create CSV: " + filePath.u8string();
        return false;
    }
    out << "exposure_10us,exposure_ms,profile_frames,target_points_mean,target_points_std,"
           "target_x_span_mm,maximum_mean,maximum_median,maximum_p10,maximum_p90,maximum_max,"
           "max_plateau_fraction,width_median,width_p90,coverage_vs_session_max,signal_vs_session_max\n";
    out << std::setprecision(10);
    for (const auto& l : result.levels) {
        out << l.exposure10us << ',' << (l.exposure10us * 0.01) << ',' << l.profileFrames << ','
            << l.targetPointsMean << ',' << l.targetPointsStd << ',' << l.targetXSpanMm << ','
            << l.maximumMean << ',' << l.maximumMedian << ',' << l.maximumP10 << ',' << l.maximumP90 << ','
            << l.maximumMax << ',' << l.maxPlateauFraction << ',' << l.widthMedian << ',' << l.widthP90 << ','
            << l.coverageVsSessionMax << ',' << l.signalVsSessionMax << '\n';
    }
    return true;
}

bool ExposureAnalysisEngine::writeResultJson(const ExposureAnalysisResult& result,
                                             const std::filesystem::path& filePath,
                                             std::string* error)
{
    std::ofstream out(filePath, std::ios::trunc);
    if (!out) {
        if (error) *error = "cannot create JSON: " + filePath.u8string();
        return false;
    }
    out << std::setprecision(10);
    out << "{\n";
    out << "  \"schema\": \"MotionLaserSync.ExposureAnalysis.v1\",\n";
    out << "  \"algorithmVersion\": \"V27.6.43-center-segment-knee-v1\",\n";
    out << "  \"sourceDirectory\": \"" << jsonEscape(result.sourceDirectory) << "\",\n";
    out << "  \"profileFilesAnalyzed\": " << result.profileFilesAnalyzed << ",\n";
    out << "  \"recommendedExposure_10us\": " << result.recommendedExposure10us << ",\n";
    out << "  \"recommendedExposure_ms\": " << (result.recommendedExposure10us * 0.01) << ",\n";
    out << "  \"saturationOnset_10us\": " << result.saturationOnsetExposure10us << ",\n";
    out << "  \"safeCandidateBand_10us\": [" << result.safeCandidateMinExposure10us
        << ", " << result.safeCandidateMaxExposure10us << "],\n";
    out << "  \"inferredSaturationCeiling\": " << result.inferredSaturationCeiling << ",\n";
    out << "  \"confidence\": \"" << jsonEscape(result.confidence) << "\",\n";
    out << "  \"rationale\": \"" << jsonEscape(result.rationale) << "\",\n";
    out << "  \"roiPolicy\": \"contiguous valid segment nearest optical X=0; split on invalid point/index gap/Z jump\",\n";
    out << "  \"important\": \"Maximum saturation is inferred from sweep plateaus; UI maxIntensity=1023 is not used as a clipping threshold.\",\n";
    out << "  \"levels\": [\n";
    for (std::size_t i = 0; i < result.levels.size(); ++i) {
        const auto& l = result.levels[i];
        out << "    {\"exposure_10us\":" << l.exposure10us
            << ",\"profileFrames\":" << l.profileFrames
            << ",\"targetPointsMean\":" << l.targetPointsMean
            << ",\"targetPointsStd\":" << l.targetPointsStd
            << ",\"targetXSpanMm\":" << l.targetXSpanMm
            << ",\"maximumMean\":" << l.maximumMean
            << ",\"maximumMedian\":" << l.maximumMedian
            << ",\"maximumP10\":" << l.maximumP10
            << ",\"maximumP90\":" << l.maximumP90
            << ",\"maximumMax\":" << l.maximumMax
            << ",\"maxPlateauFraction\":" << l.maxPlateauFraction
            << ",\"widthMedian\":" << l.widthMedian
            << ",\"widthP90\":" << l.widthP90
            << ",\"coverageVsSessionMax\":" << l.coverageVsSessionMax
            << ",\"signalVsSessionMax\":" << l.signalVsSessionMax << '}';
        if (i + 1 < result.levels.size()) out << ',';
        out << '\n';
    }
    out << "  ]\n";
    out << "}\n";
    return true;
}

} // namespace mls::calibration
