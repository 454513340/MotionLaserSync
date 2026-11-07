#pragma once
// V28 M3: Qt/SCAN-v3 bridge for the portable optical geometry simulator.
#include "twin/ScanReplayAdapter.h"
#include "twin/VirtualLaser2950.h"
#include <QVector3D>
#include <QVector>
#include <QString>
#include <atomic>
#include <memory>

namespace mls::twin {
struct TwinSimulationData {
    VirtualScanResult scan;
    Sensor2950Geometry opticalModel;
    QString sourceScanPath;
    QVector<QVector3D> previewPoints; // downsampled display only
    QString error;
    bool ok() const {return error.isEmpty()&&!scan.cancelled&&scan.totalProfiles>0;}
};
class TwinSimulationAdapter final {
public:
    static std::shared_ptr<TwinSimulationData> run(
        const TwinScene& scene,const Sensor2950Geometry& optics,
        const ScanReplayData& recorded,
        const std::atomic<bool>* cancel=nullptr,
        std::atomic<std::size_t>* finishedProfiles=nullptr,
        int previewLimit=250000);
    static QString exportFullPly(const TwinSimulationData& data,const QString& path);
    static QString exportMaskBinary(const TwinSimulationData& data,const QString& path);
    static QString exportMetadataJson(const TwinSimulationData& data,const QString& path);
};
} // namespace mls::twin
