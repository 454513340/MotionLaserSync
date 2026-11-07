#pragma once
// Qt bridge for the H1-specific M5.1 chamfer-density diagnostic.
#include "twin/ScanReplayAdapter.h"
#include "twin/TwinSimulationAdapter.h"
#include "twin/TwinM51Chamfer.h"
#include <QString>
#include <atomic>
#include <memory>

namespace mls::twin {
struct TwinM51Data {
    M51Report report;
    QString sourceScanPath,sourceCadPath,message;
    bool ok() const {return message.isEmpty()&&report.ok();}
};
class TwinM51Adapter final {
public:
    static std::shared_ptr<TwinM51Data> analyze(
        const ScanReplayData& real,const TwinSimulationData* virt,
        const TwinScene& scene,const M51Options& options,
        const std::atomic<bool>* cancelled=nullptr,
        std::atomic<std::size_t>* progress=nullptr);
    static QString exportReport(const TwinM51Data& data,const QString& outputJson);
};
} // namespace mls::twin
