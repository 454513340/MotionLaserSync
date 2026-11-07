#pragma once
// M4 Qt bridge: real archived sparse indexed profiles -> portable comparison.
// Comparison/fit are offline operations, no hardware or live-point-cloud calls.
#include "twin/ScanReplayAdapter.h"
#include "twin/TwinSimulationAdapter.h"
#include "twin/TwinM4Calibration.h"
#include <QString>
#include <atomic>
#include <memory>

namespace mls::twin {
struct TwinM4ComparisonData {
    M4Report result;
    QString realScanPath;
    Sensor2950Geometry model;
    QString message;
    bool ok() const {return message.isEmpty() && result.ok();}
};
struct TwinM4FitData {
    M4FitResult result;
    QString realScanPath;
    QString message;
    bool ok() const {return message.isEmpty() && result.ok();}
};
class TwinM4Adapter final {
public:
    static std::shared_ptr<TwinM4ComparisonData> compare(
        const ScanReplayData& measured,
        const TwinSimulationData& simulated,
        const TwinScene& scene,
        const std::atomic<bool>* cancel=nullptr,
        std::atomic<std::size_t>* progress=nullptr);
    static std::shared_ptr<TwinM4FitData> fitEquivalentCameraY(
        const ScanReplayData& measured,
        const TwinScene& scene,
        const Sensor2950Geometry& baseOptics,
        const M4FitOptions& options,
        const std::atomic<bool>* cancel=nullptr,
        std::atomic<std::size_t>* progress=nullptr);
    static QString exportComparison(const TwinM4ComparisonData& data,
                                    const QString& destinationJson);
    static QString exportFit(const TwinM4FitData& data,
                             const QString& destinationJson);
};
} // namespace mls::twin
