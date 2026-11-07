#pragma once

#include "runtime/FiveAxis.h"
#include <array>
#include <cstddef>
#include <istream>
#include <string>
#include <vector>

namespace mls::axiscomp {
// This is a demand-coordinate transform, NOT a feedback filter or coordinate reset.
// Physical controller axis IDs 1/2/3 are converted to array indices 0/1/2 by the adapter.
enum class SamplingMethod { MonotonicSweep, PointToPointPreload };
struct Node { double desiredMpos = 0, requiredDpos = 0; };
struct Table {
    int axisIndex = -1;
    SamplingMethod method = SamplingMethod::PointToPointPreload;
    std::array<std::vector<Node>, 2> directions; // [0] positive, [1] negative; sorted by MPOS
    int settleMs = 2000;
    double preloadMm = 2;
    double precisionSpeedMmS = 0; // 0 means not declared, NOT an inferred qualification
    std::string sourceId; // file SHA-256 supplied by the bundle loader
};
using Tables = std::array<Table, 3>;
struct Options {
    // Different production histories are never silently called validated.
    bool allowLegacyPointTableOnPlannedMotion = false;
    bool allowContinuous = false;
    int pointSettleMs = 2000;
    double maxCorrectionMm = 0.10;
    double holdToleranceMm = 0.000001;
    double commandDirectionToleranceMm = 0.00015; // float/command quantization only
    std::size_t maxPathPoints = 200000;
};
struct PointResult { double commandDpos = 0; int direction = 0; bool held = false; };

// Strict readers: quoted CSV, BOM, named columns, finite numbers, axis identity,
// both monotonic branches, correction-column consistency. Never skip a bad row.
bool readCandidateCsv(std::istream& input, int expectedAxis, Table& output, std::string& error);
bool validateTable(const Table& table, double maxCorrectionMm, std::string& error);
bool validateTables(const Tables& tables, const Options& options, std::string& error);
// Pure interpolation; no clamping/extrapolation. direction must be +1 or -1.
bool interpolate(const Table& table, int direction, double desired, double& command,
                 std::string& error);

// nominalStart is the preceding accepted nominal target when that demand is still
// current, otherwise the measured MPOS. Holding an axis preserves its DPOS exactly.
bool mapPoint(const Table& table, const Options& options, double nominalStart,
              double currentDpos, double desired, PointResult& result, std::string& error);

// Split each nominal line at the union of XYZ calibration interpolation knots.
// Held axes retain their already-commanded coordinates. Directions follow the
// planned nominal sequence, never asynchronously polled feedback.
// The caller must validate/replan the ENTIRE returned command path before any SDK move.
bool mapPath(const Tables& tables, const Options& options,
             const runtime::Joint& nominalStart, const runtime::Joint& commandStart,
             const std::vector<runtime::Waypoint>& nominalPath,
             std::vector<runtime::Waypoint>& commandPath, std::string& error);
} // namespace mls::axiscomp
