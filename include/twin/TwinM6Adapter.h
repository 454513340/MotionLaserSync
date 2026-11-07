#pragma once
// V28.7 M6 Qt bridge, no hardware commands. Consumes existing M2/M5/M3
// snapshots and injects recorded Physical17 kinematics and optional FCL checks.
#include "twin/ScanReplayAdapter.h"
#include "twin/TwinM5Adapter.h"
#include "twin/TwinM6Continuous.h"
#include "twin/TwinM6ScanArchive.h"
#include <QVector>
#include <QVector3D>
#include <QString>
#include <atomic>
#include <memory>

namespace mls::twin {
struct TwinM6Data {
    M6Report report;
    QVector<QVector3D> preview;
    QVector<QVector3D> pathPreview;
    QVector<QVector<QVector3D>> segmentPointPreviews;
    QVector<QVector<QVector3D>> segmentLaserPaths;
    QVector<QVector3D> transferLaserPath;
    QString status,sourceCadPath,sourceScanPath;
    QString binarySpoolPath;
    QString temporaryArchiveDirectory;
    std::vector<M6ScanVolume> scanVolumes;
    std::uint64_t archivedValidPoints=0;
    bool fullPhysical17Validated=false; // H1 replay consistency only
    bool bcVariedInRecordedScan=false;
    bool machineFclLoaded=false;
    bool ok()const{return status.isEmpty()&&report.ok();}
    ~TwinM6Data();
};
class TwinM6Adapter final {
public:
    static std::shared_ptr<TwinM6Data> run(
        const TwinScene& scene,const M5Report& baseline,
        const ScanReplayData& measured,const Sensor2950Geometry& model,
        M6Options options, const QString& machineSafetyConfigPath,
        const std::atomic<bool>* cancel=nullptr,
        std::atomic<std::size_t>* finishedProfiles=nullptr,
        M6ProgressState* liveProgress=nullptr);
    static QString exportReport(const TwinM6Data& data,const QString& jsonPath,
        const std::atomic<bool>* cancel=nullptr, M6ExportProgress* exportProgress=nullptr);
};
} // namespace mls::twin
