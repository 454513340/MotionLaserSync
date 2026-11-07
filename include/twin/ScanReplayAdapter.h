#pragma once
// V28 M2: offline-only recorded SCAN-v3 replay. Not a sensor simulator.
// No device controller / SDK calls. All spatial samples use the saved
// exposure-time sensorToMachine matrix exactly once.
#include "utils/CompactScanFormat.h"
#include <QVector3D>
#include <QVector>
#include <QString>
#include <array>
#include <cstdint>
#include <memory>

namespace mls::twin {
struct ScanReplayFrame {
    std::uint32_t profileIndex = 0;
    std::uint64_t ordinal = 0;
    double elapsedSeconds = 0.0; // monotone: sensor acquisition time, guarded fallback
    std::array<double,5> joints{{}}; // XYZCB recorded physical axes (MPOS)
    std::uint32_t rawPoints = 0;
    qsizetype previewEnd = 0;  // cumulative exclusive end in previewPoints
};

struct ScanReplayData {
    ScanData scan;  // preserves every source sample index & calibration provenance
    QVector<ScanReplayFrame> frames;
    QVector<QVector3D> previewPoints; // bounded, VISUAL ONLY (not analysis source)
    std::uint64_t totalRawPoints = 0;
    QVector3D previewBoundsMin, previewBoundsMax;
    bool hasPreviewBounds = false;
    std::uint64_t invalidTransforms = 0;
    std::uint64_t invalidPointCoords = 0;
    QString sourcePath;
    QString error;
    bool ok() const { return error.isEmpty() && !frames.isEmpty(); }
};

class ScanReplayAdapter final {
public:
    // Load and CRC-validate archive on a worker thread; only SCAN v3 is accepted.
    // A bounded sample is generated for VTK, while ALL original data is retained.
    static std::shared_ptr<ScanReplayData> load(const QString& file, int previewLimit=250000);
    // Full-resolution little-endian binary PLY (no visual decimation).
    // Empty error string on success. Intended to run on a worker thread.
    static QString exportFullPly(const ScanReplayData& data, const QString& filename);
};
} // namespace mls::twin
