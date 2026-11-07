#pragma once
#include "twin/TwinTypes.h"
#include <string>
namespace mls::twin {
// Ground truth is confined to rendering/ray tracing. Planners must use estimatedPose().
// This distinction is deliberate for later CAD registration simulation.
class TwinScene {
public:
    bool setWorkpiece(TriangleMesh mesh, std::string* error=nullptr);
    bool loadObjMesh(const std::string& fileName, std::string* error=nullptr);
    void clear();
    bool hasWorkpiece() const {return !mesh_.empty();}
    const TriangleMesh& mesh() const {return mesh_;}
    void setTruePose(const Pose6& p) {truth_=p;}
    void setEstimatedPose(const Pose6& p) {estimate_=p;}
    Pose6 truePose() const {return truth_;}
    Pose6 estimatedPose() const {return estimate_;}
    Transform cadToMachineTruth() const {return Transform::fromPose(truth_);}
    Transform cadToMachineEstimate() const {return Transform::fromPose(estimate_);}
    Bounds workpieceBoundsMachine() const;
    // Optional horizontal support plane. Limited rectangle in machine XY.
    void setSupportPlane(bool enabled, double zMm, double centerX, double centerY,
                         double widthMm, double heightMm);
    bool supportEnabled() const {return supportEnabled_;}
    double supportZ() const {return supportZ_;}
    Vec3 supportCenter() const {return {supportX_,supportY_,supportZ_};}
    Vec3 supportSize() const {return {supportW_,supportH_,0};}
    // First-hit diagnostic ray trace, millimetres in MCS. M1 brute force; M3 replaces with BVH.
    RayHit firstHit(const Ray& ray, double maxDistanceMm=10000.) const;
private:
    TriangleMesh mesh_;
    Pose6 truth_{}, estimate_{};
    bool supportEnabled_=false;
    double supportZ_=0,supportX_=0,supportY_=0,supportW_=200,supportH_=200;
};
} // namespace mls::twin
