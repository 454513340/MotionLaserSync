#pragma once
// Qt bridge: M2/M3 original Profile -> CAD frame -> portable M5.2 diagnosis.
#include "twin/ScanReplayAdapter.h"
#include "twin/TwinSimulationAdapter.h"
#include "twin/TwinM52Geometry.h"
#include <QString>
#include <atomic>
#include <memory>
namespace mls::twin {
struct TwinM52Data {
    M52Report report;
    QString sourceScanPath,sourceCadPath,message;
    bool ok() const {return message.isEmpty()&&report.ok();}
};
class TwinM52Adapter final {
public:
    static std::shared_ptr<TwinM52Data> analyze(
        const ScanReplayData& real,const TwinSimulationData* virt,
        const TwinScene& scene,const M52Options& options,
        const std::atomic<bool>* cancelled=nullptr,
        std::atomic<std::size_t>* progress=nullptr);
    static QString exportReport(const TwinM52Data& data,const QString& outputJson);
};
} // namespace mls::twin
