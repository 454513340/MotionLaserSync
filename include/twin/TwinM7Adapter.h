#pragma once
// V28.8 M7 Qt bridge: M2 measured points + M6 simulated multi-view returns.
// This is read-only/offline and never claims synthetic samples are hardware data.
#include "twin/TwinM7Fusion.h"
#include "twin/TwinM5Adapter.h"
#include "twin/TwinM6Adapter.h"
#include <QVector>
#include <QVector3D>
#include <QString>
#include <atomic>
#include <memory>

namespace mls::twin {
struct M7Progress {
    std::atomic<int> stage{0}; // 1: coverage, 2: M2 preview, 3: M6 preview, 4: complete
    std::atomic<std::uint64_t> done{0}, total{0};
};
struct M7RemainingTarget {
    std::uint32_t cellIndex=0;
    std::uint16_t featureId=0;
    Vec3 cadPosition{},machinePosition{},machineNormal{};
    double areaMm2=0;
    std::uint32_t measuredHits=0;
    bool sparseReopened=false;
};
struct TwinM7Data {
    M7Coverage coverage;
    std::vector<M7RemainingTarget> remainingTargets;
    QVector<QVector3D> realPoints;       // bounded preview, REAL M2 only
    QVector<QVector3D> simulatedPoints;  // bounded preview, ALL M6 virtual ray returns
    QVector<QVector3D> simulatedOnlyPoints; // only voxels without any real M2 point
    QVector<QVector3D> overlapPoints;    // spatial overlap, REAL position wins
    QVector<QVector3D> fusedPreview;     // bounded union; not full-resolution CAD reconstruction
    QVector<QVector3D> newlyCoveredCad;  // CAD DIAGNOSTIC markers, not measurements
    QVector<QVector3D> stillMissingCad;  // CAD DIAGNOSTIC markers, not measurements
    QVector<QVector<QVector3D>> segmentPreview;
    double overlapMeanMm=0,overlapRmsMm=0;
    std::uint64_t previewM2Points=0,previewM6Points=0;
    QString sourceM2,sourceCad;
    QString message;
    bool ok() const{return message.isEmpty()&&coverage.ok()&&coverage.areaConsistent;}
};
class TwinM7Adapter final {
public:
    static std::shared_ptr<TwinM7Data> fuse(
        const TwinScene& scene,const ScanReplayData& real,
        const TwinM5Data& m5,const TwinM6Data& m6,
        const M7VoxelOptions& voxels,double coverageTarget,
        const std::atomic<bool>* cancel=nullptr,M7Progress* progress=nullptr);
    static QString exportReport(const TwinM7Data& data,const TwinM6Data& m6,
                                const QString& jsonPath,const std::atomic<bool>* cancel=nullptr);
};
} // namespace mls::twin
