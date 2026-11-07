#pragma once
// M6-only streaming bridge to the existing SCAN v3 + Archive v5 codec.
// Not a controller API.  Only synthetic exposures with PoseTiming::Simulated
// and ZERO hardware provenance flags may be written by this bridge.
#include "twin/VirtualLaser2950.h"
#include "utils/CompactScanFormat.h"
#include <QString>
#include <atomic>
#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace mls::twin {
struct M6ScanVolume {
    QString temporaryPath;
    std::size_t scanSegmentNumber=0;
    std::size_t partNumber=0;
    std::uint64_t firstOrdinal=0, lastOrdinal=0;
    std::uint64_t profiles=0, validPoints=0;
    std::uint64_t storedBytes=0;
};
struct M6ExportProgress {
    std::atomic<std::uint64_t> bytesDone{0}, bytesTotal{0};
    std::atomic<std::size_t> filesDone{0}, filesTotal{0};
};

class M6ScanVolumeWriter final {
public:
    M6ScanVolumeWriter(const QString& scratchDirectory, double scanRateHz,
        const std::array<double,17>& sourcePhysical17, std::uint64_t simulationSession,
        std::atomic<std::size_t>* volumesReady=nullptr);
    // Consumes a complete 1280-slot M3 profile synchronously; does NOT
    // interpolate, resample or invent sensor returns.
    bool append(const VirtualProfile& virtualProfile, std::size_t oneBasedScanSegment,
                std::string* error);
    bool finish(std::string* error);
    const std::vector<M6ScanVolume>& volumes() const {return volumes_;}
    std::uint64_t profileCount() const {return ordinal_;}
    std::uint64_t validPointCount() const {return totalValid_;}
private:
    bool flush(std::string* error);
    QString directory_;
    double scanRateHz_=200;
    std::array<double,17> physical_{};
    std::uint64_t session_=0, ordinal_=0, totalValid_=0;
    std::size_t activeSegment_=0, part_=0;
    std::uint64_t rawEstimate_=16;
    ScanData chunk_;
    std::vector<M6ScanVolume> volumes_;
    std::atomic<std::size_t>* volumesReady_=nullptr;
    static constexpr std::uint64_t kSafeUncompressedBudget=48ULL*1024*1024;
};
} // namespace mls::twin
