#pragma once
// V28.4 M5 Qt bridge: SCAN v3 archived samples + M3 virtual samples ->
// portable area-weighted CAD occupancy. Read-only/offline, no device actions.
#include "twin/ScanReplayAdapter.h"
#include "twin/TwinSimulationAdapter.h"
#include "twin/TwinM5Coverage.h"
#include <QVector3D>
#include <QVector>
#include <QString>
#include <atomic>
#include <memory>

namespace mls::twin {
struct TwinM5Data {
    M5Report report;
    QString sourceScanPath;
    QString sourceCadPath;
    bool containsVirtualProfiles=false;
    QVector<QVector3D> realObservedPreview;
    QVector<QVector3D> realMissingPreview;
    QVector<QVector3D> virtualOnlyPreview;
    QString message;
    bool ok() const {return message.isEmpty()&&report.ok();}
};
class TwinM5Adapter final {
public:
    static std::shared_ptr<TwinM5Data> analyze(
        const ScanReplayData& measured,
        const TwinSimulationData* simulated,
        const TwinScene& scene,
        const M5Options& options,
        const std::atomic<bool>* cancel=nullptr,
        std::atomic<std::size_t>* progress=nullptr);
    // JSON summary, full per-cell CSV (machine/CAD coordinates, area, hits),
    // face-group CSV. All output stays offline and preserves source inputs.
    static QString exportReport(const TwinM5Data& data, const QString& filenameJson);
};
} // namespace mls::twin
