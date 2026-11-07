#pragma once

#include "calibration/SphereDistanceAngleStudy.h"
#include "calibration/SphereArcQualityGate.h"
#include <QString>
#include <vector>
#include <cstddef>

namespace mls::calibration {

// Unified fit for every IMPORTED reference sphere (.scan, PLY/CSV).
// .scan v3 has binary64 per-profile sensor X/Z and frozen Physical17 transforms;
// other file types MUST use the unchanged 3D fit, without invented arc metadata.

struct SphereTwoStageResult {
    SphereCloudFitResult original;          // Unchanged 3-D RANSAC comparison
    SphereCloudFitResult filtered;          // Arc-qualified profile 3-D RANSAC
    SphereCloudFitResult accepted;          // Only this may enter planning/calibration
    std::vector<Vec3d> acceptedInlierMachinePoints; // authoritative doubles for further refinement
    bool scanWithRawProfiles = false;
    bool twoStageAccepted = false;
    bool acceptedIsLegacy = false;
    std::size_t totalProfiles = 0;
    std::size_t qualifiedProfiles = 0;
    std::size_t qualifiedPoints = 0;
    QString note;
};

class SphereTwoStageFit {
public:
    // sourcePath is the actual imported file; importedPoints is the original
    // visible XYZ cloud. For .scan v3 the authoritative double archive is
    // reloaded rather than fitting display-only float QVector3D coordinates.
    static SphereTwoStageResult fitImportedSphere(
        const QString& sourcePath, const std::vector<Vec3d>& importedPoints,
        const SphereCloudRansacOptions& originalOptions,
        const SphereTwoStageQuality& quality = SphereTwoStageQuality());

    static QString comparisonText(const SphereTwoStageResult& result);
};

} // namespace mls::calibration
