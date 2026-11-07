#pragma once

#include "machine_config.h"
#include "machine_kinematics.h"
#include "twin/TwinTypes.h"

#include <Eigen/Core>
#include <fcl/fcl.h>

#include <filesystem>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

struct CollisionHit {
    std::string linkA;
    std::string linkB;
    Eigen::Vector3d contact_mm = Eigen::Vector3d::Zero();
    double penetration_mm = 0.0;
};

struct DistanceInfo {
    double distance_mm = 0.0;
    std::string linkA;
    std::string linkB;
};

class CollisionEngine {
public:
    // Bundled STEP collision API: MachineSafetyModel must compile against this
    // exact header, not a system/vcpkg or stale build-directory header.
    static constexpr unsigned kStepWorkpieceApiVersion = 2;
    void load(const MachineConfig& config);
    void update(const MachineState& state);

    // Imported STEP is triangulated once and kept as a static CAD->MCS FCL
    // obstacle.  It must NOT inherit the moving head's X/Y/Z/C/B transform.
    // The caller supplies an independently reviewed CAD placement in MCS.
    bool setWorkpiece(const mls::twin::TriangleMesh& mesh,
                      const mls::twin::Transform& cadToMcs,
                      std::string* error = nullptr);
    void clearWorkpiece();
    bool hasWorkpiece() const { return bool(workpiece_); }

    std::vector<CollisionHit> checkCollisions(std::size_t maxHits = 32) const;
    DistanceInfo minimumDistance() const;
    // Only the "below threshold" decision is needed by sampled clearance
    // gates. AABB lower bounds avoid expensive FCL distance on distant pairs.
    DistanceInfo minimumDistanceBelow(double thresholdMm) const;

    // Dynamic study obstacle: the RANSAC sphere is expressed directly in MCS.
    // Only selected moving links are tested against it so intended mounting or
    // base geometry does not create false positives.
    std::vector<CollisionHit> checkSphereObstacle(
        const Eigen::Vector3d& centerMm,
        double radiusMm,
        const std::vector<std::string>& linkNames,
        std::size_t maxHits = 16) const;
    DistanceInfo minimumDistanceToSphereObstacle(
        const Eigen::Vector3d& centerMm,
        double radiusMm,
        const std::vector<std::string>& linkNames) const;

    std::size_t totalTriangles() const { return totalTriangles_; }
    std::size_t linkCount() const { return links_.size(); }

private:
    using Model = fcl::BVHModel<fcl::OBBRSSd>;

    struct CollisionLink {
        std::string name;
        std::filesystem::path sourcePath;
        std::size_t triangleCount = 0;
        std::shared_ptr<Model> model;
        std::unique_ptr<fcl::CollisionObjectd> object;
    };

    bool shouldCheck(const std::string& a, const std::string& b) const;

    std::unordered_map<std::string, CollisionLink> links_;
    std::unique_ptr<CollisionLink> workpiece_;
    std::set<std::pair<std::string, std::string>> ignoredPairs_;
    std::size_t totalTriangles_ = 0;
};
