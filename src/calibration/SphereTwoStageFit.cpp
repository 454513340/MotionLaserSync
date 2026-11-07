#include "calibration/SphereTwoStageFit.h"
#include "calibration/SphereExposureSweep.h"
#include "utils/CompactScanFormat.h"

#include <QFileInfo>
#include <QObject>
#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace mls::calibration {
namespace {
struct IndexedPoint { Vec3d point; std::size_t originalIndex = 0; };

void mapInliers(SphereTwoStageResult& out,
                const std::vector<IndexedPoint>& selected,
                const std::vector<Vec3d>& originalCloud)
{
    out.acceptedInlierMachinePoints.reserve(out.accepted.inlierIndices.size());
    // Preserve the source-cloud indexing contract for UI inlier display / CSV.
    // This is only correct if the original imported cloud has the same order.
    const bool aligned = out.totalProfiles > 0 && !originalCloud.empty() &&
        out.original.candidatePointCount == originalCloud.size();
    std::vector<std::size_t> sourceIndices;
    sourceIndices.reserve(out.accepted.inlierIndices.size());
    for (const std::size_t i : out.accepted.inlierIndices) {
        if (i >= selected.size()) continue;
        out.acceptedInlierMachinePoints.push_back(selected[i].point);
        if (aligned) sourceIndices.push_back(selected[i].originalIndex);
    }
    // If the view cloud has another order, don't pretend filtered inlier indices
    // index it. The fit itself remains valid; only highlight is unavailable.
    out.accepted.inlierIndices = std::move(sourceIndices);
}

QString describe(const SphereCloudFitResult& fit) {
    if (!fit.success) return QObject::tr("失败(%1)").arg(QString::fromStdString(fit.message));
    return QObject::tr("直径=%1mm，RMS=%2mm，内点=%3/%4")
        .arg(2.0 * fit.radiusMm, 0, 'f', 6)
        .arg(fit.residualRmsMm, 0, 'f', 6)
        .arg(qulonglong(fit.inlierPointCount))
        .arg(qulonglong(fit.candidatePointCount));
}
}

SphereTwoStageResult SphereTwoStageFit::fitImportedSphere(
    const QString& sourcePath, const std::vector<Vec3d>& importedPoints,
    const SphereCloudRansacOptions& originalOptions,
    const SphereTwoStageQuality& quality)
{
    SphereTwoStageResult out;
    const bool isScan = QFileInfo(sourcePath).suffix().compare(
        QStringLiteral("scan"), Qt::CaseInsensitive) == 0;
    ScanData archive;
    std::vector<Vec3d> all;

    if (isScan) {
        QString error;
        if (!CompactScanIO::load(sourcePath, archive, CompactScanIO::ReadOptions(), &error)) {
            out.note = QObject::tr(".scan 重新解码失败：%1；禁止静默改用另一个点云").arg(error);
            return out;
        }
        bool v3 = !archive.profiles.isEmpty();
        for (const auto& p : archive.profiles)
            if (!p.hasFiveAxis) {v3 = false; break;}
        if (v3) {
            out.scanWithRawProfiles = true;
            out.totalProfiles = std::size_t(archive.profiles.size());
            std::size_t reservePoints = 0;
            for (const auto& p : archive.profiles) reservePoints += p.fiveAxis.points.size();
            all.reserve(reservePoints);
            for (const auto& p : archive.profiles)
                for (const auto& q : p.fiveAxis.points) {
                    const auto m = mls::runtime::mapProfilePoint(p.fiveAxis.frame, q.x, q.z);
                    all.push_back(Vec3d{m[0], m[1], m[2]});
                }
        }
    }

    // Non-v3 scan, CSV, PLY have no reliable per-frame sensor geometry.
    // Show a transparent LEGACY result instead of synthesizing arc coverage.
    if (!out.scanWithRawProfiles) {
        out.original = SphereDistanceAngleStudy::fitMachineSphereRansac(importedPoints, originalOptions);
        out.accepted = out.original;
        out.acceptedIsLegacy = out.original.success;
        out.note = isScan
            ? QObject::tr("旧版.scan没有可验证的逐帧五轴XZ，保留原三维RANSAC作为对照/兼容；未执行圆弧筛选")
            : QObject::tr("PLY/CSV没有逐帧原始XZ，保留原三维RANSAC；未执行圆弧筛选");
        if (out.original.success)
            for (auto idx:out.original.inlierIndices)
                if(idx<importedPoints.size())out.acceptedInlierMachinePoints.push_back(importedPoints[idx]);
        return out;
    }

    out.original = SphereDistanceAngleStudy::fitMachineSphereRansac(all, originalOptions);
    if (!out.original.success) {
        out.note = QObject::tr("传统全点RANSAC未找到可靠球心，不能用它推测球面XZ圆心。%1")
            .arg(QString::fromStdString(out.original.message));
        return out;
    }
    const double R = originalOptions.expectedRadiusMm;
    if (!(R > 0.0) || !std::isfinite(R)) {
        out.note = QObject::tr("理论半径无效，停止两阶段拟合");
        return out;
    }

    std::vector<IndexedPoint> selected;
    selected.reserve(all.size() / 2);
    std::size_t cloudOffset = 0;
    for (const auto& profile: archive.profiles) {
        const auto& frame = profile.fiveAxis.frame;
        const auto& t = frame.sensorToMachine;
        const std::size_t n = profile.fiveAxis.points.size();
        const Vec3d center = out.original.centerMachineMm;
        double expectedX=0.0, expectedZ=0.0;
        // Inverse of saved rigid Physical17 transform: R^T*(C_machine - t).
        if(!projectSphereCenterToSensorArcXZ(t,center.x,center.y,center.z,expectedX,expectedZ)) {
            cloudOffset += n;
            continue;
        }
        std::vector<SphereExposurePoint> arcPoints;
        arcPoints.reserve(n);
        for (const auto& p:profile.fiveAxis.points) {
            if (std::isfinite(p.x) && std::isfinite(p.z))
                arcPoints.push_back({p.x,p.z,int(p.index)});
        }
        const auto arc = SphereExposureSweep::fitArc(arcPoints,expectedX,expectedZ,R);
        if (qualifySphereArc(arc, quality, R)) {
            ++out.qualifiedProfiles;
            for(std::size_t i=0;i<n;++i)
                selected.push_back({all[cloudOffset+i],cloudOffset+i});
        }
        cloudOffset += n;
    }
    out.qualifiedPoints = selected.size();
    if(out.qualifiedProfiles < quality.minimumAcceptedProfiles ||
       selected.size() < std::size_t(std::max(100, originalOptions.minInliers))) {
        out.note = QObject::tr("单帧圆弧质量不足：合格%1/%2条、%3点。保留原RANSAC作诊断，禁止直接用于自动路径或标定")
            .arg(qulonglong(out.qualifiedProfiles)).arg(qulonglong(out.totalProfiles))
            .arg(qulonglong(out.qualifiedPoints));
        return out;
    }
    std::vector<Vec3d> fitPoints;
    fitPoints.reserve(selected.size());
    for(const auto& p:selected)fitPoints.push_back(p.point);
    SphereCloudRansacOptions refined = originalOptions;
    refined.inlierThresholdMm = quality.refinedRansacThresholdMm;
    // Do not enlarge the original physical radius tolerance.
    out.filtered = SphereDistanceAngleStudy::fitMachineSphereRansac(fitPoints,refined);
    if (!out.filtered.success) {
        out.note = QObject::tr("已完成圆弧筛选，但三维RANSAC失败：%1；仍保留传统结果对照")
            .arg(QString::fromStdString(out.filtered.message));
        return out;
    }
    // The legacy fit only anchors ROI selection. A wildly displaced second
    // candidate likely indicates another object rather than a better sphere.
    const Vec3d& c0=out.original.centerMachineMm;
    const Vec3d& c1=out.filtered.centerMachineMm;
    const double centerDistance=std::sqrt((c0.x-c1.x)*(c0.x-c1.x)
         +(c0.y-c1.y)*(c0.y-c1.y)+(c0.z-c1.z)*(c0.z-c1.z));
    if (!std::isfinite(centerDistance) || centerDistance > 1.0) {
        out.note=QObject::tr("两阶段球心相差%1mm，疑似背景/坐标约定不一致；拒绝自动采用")
            .arg(centerDistance,0,'f',4);
        return out;
    }
    out.twoStageAccepted = true;
    out.accepted = out.filtered;
    mapInliers(out,selected,importedPoints);
    out.note = QObject::tr("两阶段完成：筛选合格%1/%2帧、保留%3点；旧阈值%4mm，新阈值%5mm。")
        .arg(qulonglong(out.qualifiedProfiles)).arg(qulonglong(out.totalProfiles))
        .arg(qulonglong(out.qualifiedPoints))
        .arg(originalOptions.inlierThresholdMm,0,'f',3)
        .arg(refined.inlierThresholdMm,0,'f',3);
    return out;
}

QString SphereTwoStageFit::comparisonText(const SphereTwoStageResult& r)
{
    QString result=QObject::tr("原始全点3D RANSAC：%1；\n圆弧筛选后3D RANSAC：%2。\n%3")
        .arg(describe(r.original))
        .arg(!r.scanWithRawProfiles ? QObject::tr("不适用") :
             (r.filtered.candidatePointCount==0 ? QObject::tr("尚未执行或质量不合格") : describe(r.filtered)))
        .arg(r.note);
    return result;
}
} // namespace mls::calibration
