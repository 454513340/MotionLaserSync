#include "collision_engine.h"
#include "stl_mesh.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
// For axis-aligned boxes, this is a conservative lower bound on mesh distance
// in mm. Never use it to declare two overlapping boxes collision-free.
double aabbLowerBound(const fcl::AABBd& a, const fcl::AABBd& b) {
    double square=0.0;
    for(int k=0;k<3;++k){
        const double gap=std::max({0.0,a.min_[k]-b.max_[k],b.min_[k]-a.max_[k]});
        square+=gap*gap;
    }
    return std::sqrt(square);
}
bool workpieceMovingLink(const std::string& name){
    // BASE can be the fixture/support; its contact with the part is intended.
    // All moving head/axis envelopes must remain clear of the part.
    return name=="02_X_AXIS" || name=="03_Y_AXIS" || name=="04_Z_AXIS" ||
           name=="05_C_AXIS" || name=="06_B_AXIS" || name=="07_END_SENSOR";
}
}

void CollisionEngine::load(const MachineConfig& config) {
    links_.clear();
    workpiece_.reset();
    ignoredPairs_ = config.ignoredPairs;
    totalTriangles_ = 0;

    for (const auto& linkCfg : config.links) {
        if (!std::filesystem::exists(linkCfg.collisionPath)) {
            throw std::runtime_error("Collision STL not found: " + linkCfg.collisionPath.string());
        }

        const auto triangles = loadStlTriangles(linkCfg.collisionPath);
        if (triangles.empty()) {
            throw std::runtime_error("Collision STL is empty: " + linkCfg.collisionPath.string());
        }

        auto model = std::make_shared<Model>();
        model->beginModel(
            static_cast<int>(triangles.size()),
            static_cast<int>(triangles.size() * 3));

        for (const auto& t : triangles) model->addTriangle(t.a, t.b, t.c);
        model->endModel();

        CollisionLink link;
        link.name = linkCfg.name;
        link.sourcePath = linkCfg.collisionPath;
        link.triangleCount = triangles.size();
        link.model = std::move(model);
        link.object = std::make_unique<fcl::CollisionObjectd>(
            link.model, fcl::Transform3d::Identity());
        link.object->computeAABB();

        totalTriangles_ += link.triangleCount;
        links_.emplace(link.name, std::move(link));
    }
}

bool CollisionEngine::setWorkpiece(const mls::twin::TriangleMesh& mesh,
                                  const mls::twin::Transform& cadToMcs,
                                  std::string* error) {
    workpiece_.reset();
    if(mesh.triangles.empty()){
        if(error)*error="STEP workpiece contains no triangles";
        return false;
    }
    for(double x:cadToMcs.m)if(!std::isfinite(x)){
        if(error)*error="CAD->MCS transform contains non-finite values";
        return false;
    }
    const auto& a=cadToMcs.m;
    Eigen::Matrix3d rotation;
    for(int r=0;r<3;++r)for(int c=0;c<3;++c)rotation(r,c)=a[4*r+c];
    if((rotation.transpose()*rotation-Eigen::Matrix3d::Identity()).norm()>1e-5 ||
       std::abs(rotation.determinant()-1.0)>1e-5 ||
       std::abs(a[12])+std::abs(a[13])+std::abs(a[14])+std::abs(a[15]-1)>1e-6){
        if(error)*error="CAD placement must be a rigid, right-handed mm transform";
        return false;
    }
    auto link=std::make_unique<CollisionLink>();
    link->name="WORKPIECE_STEP";
    link->triangleCount=mesh.triangles.size();
    link->model=std::make_shared<Model>();
    link->model->beginModel(static_cast<int>(mesh.triangles.size()),
                            static_cast<int>(mesh.triangles.size()*3));
    for(const auto& tri:mesh.triangles){
        const auto valid=[](const mls::twin::Vec3& p){
            return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z);
        };
        if(!valid(tri.a)||!valid(tri.b)||!valid(tri.c)){
            if(error)*error="STEP workpiece contains invalid vertices";
            return false;
        }
        link->model->addTriangle(Eigen::Vector3d(tri.a.x,tri.a.y,tri.a.z),
                                 Eigen::Vector3d(tri.b.x,tri.b.y,tri.b.z),
                                 Eigen::Vector3d(tri.c.x,tri.c.y,tri.c.z));
    }
    link->model->endModel();
    fcl::Transform3d tf=fcl::Transform3d::Identity();
    tf.linear()=rotation;
    tf.translation()=Eigen::Vector3d(a[3],a[7],a[11]);
    link->object=std::make_unique<fcl::CollisionObjectd>(link->model,tf);
    link->object->computeAABB();
    workpiece_=std::move(link);
    return true;
}

void CollisionEngine::clearWorkpiece(){workpiece_.reset();}

void CollisionEngine::update(const MachineState& state) {
    for (auto& [name, link] : links_) {
        const Eigen::Isometry3d T = linkDelta(name, state);
        link.object->setTransform(T);
        link.object->computeAABB();
    }
}

bool CollisionEngine::shouldCheck(const std::string& a, const std::string& b) const {
    return ignoredPairs_.find(canonicalPair(a, b)) == ignoredPairs_.end();
}

std::vector<CollisionHit> CollisionEngine::checkCollisions(std::size_t maxHits) const {
    std::vector<const CollisionLink*> list;
    list.reserve(links_.size());
    for (const auto& [name, link] : links_) list.push_back(&link);
    std::sort(list.begin(), list.end(), [](const auto* a, const auto* b) {
        return a->name < b->name;
    });

    std::vector<CollisionHit> hits;
    // Append STEP as a separate obstacle. This keeps original machine ignored
    // pairs intact and does not apply linkDelta() to the static workpiece.
    if(workpiece_)list.push_back(workpiece_.get());
    for (std::size_t i = 0; i < list.size(); ++i) {
        for (std::size_t j = i + 1; j < list.size(); ++j) {
            const auto& A = *list[i];
            const auto& B = *list[j];
            const bool part=A.name=="WORKPIECE_STEP" || B.name=="WORKPIECE_STEP";
            if(part ? !workpieceMovingLink(part && A.name=="WORKPIECE_STEP"?B.name:A.name)
                    : !shouldCheck(A.name, B.name)) continue;
            if(!A.object->getAABB().overlap(B.object->getAABB()))continue;

            fcl::CollisionRequestd request;
            request.num_max_contacts = 1;
            request.enable_contact = true;
            fcl::CollisionResultd result;
            const std::size_t n = fcl::collide(A.object.get(), B.object.get(), request, result);
            if (n == 0 || !result.isCollision()) continue;

            CollisionHit hit;
            hit.linkA = A.name;
            hit.linkB = B.name;
            std::vector<fcl::Contactd> contacts;
            result.getContacts(contacts);
            if (!contacts.empty()) {
                hit.contact_mm = contacts.front().pos;
                hit.penetration_mm = contacts.front().penetration_depth;
            }
            hits.emplace_back(std::move(hit));
            if (hits.size() >= maxHits) return hits;
        }
    }
    return hits;
}

DistanceInfo CollisionEngine::minimumDistance() const {
    std::vector<const CollisionLink*> list;
    list.reserve(links_.size());
    for (const auto& [name, link] : links_) list.push_back(&link);

    DistanceInfo best;
    best.distance_mm = std::numeric_limits<double>::infinity();
    if(workpiece_)list.push_back(workpiece_.get());
    for (std::size_t i = 0; i < list.size(); ++i) {
        for (std::size_t j = i + 1; j < list.size(); ++j) {
            const auto& A = *list[i];
            const auto& B = *list[j];
            const bool part=A.name=="WORKPIECE_STEP" || B.name=="WORKPIECE_STEP";
            if(part ? !workpieceMovingLink(A.name=="WORKPIECE_STEP"?B.name:A.name)
                    : !shouldCheck(A.name, B.name)) continue;
            if(aabbLowerBound(A.object->getAABB(),B.object->getAABB())>=best.distance_mm)continue;

            fcl::DistanceRequestd request;
            request.enable_nearest_points = false;
            fcl::DistanceResultd result;
            const double d = fcl::distance(A.object.get(), B.object.get(), request, result);
            if (d < best.distance_mm) {
                best.distance_mm = d;
                best.linkA = A.name;
                best.linkB = B.name;
            }
        }
    }
    return best;
}

DistanceInfo CollisionEngine::minimumDistanceBelow(double thresholdMm) const {
    DistanceInfo result;
    result.distance_mm=std::numeric_limits<double>::infinity();
    if(!(thresholdMm>0.0)||!std::isfinite(thresholdMm))return result;
    std::vector<const CollisionLink*> list;
    list.reserve(links_.size()+1);
    for(const auto& kv:links_)list.push_back(&kv.second);
    if(workpiece_)list.push_back(workpiece_.get());
    for(std::size_t i=0;i<list.size();++i)for(std::size_t j=i+1;j<list.size();++j){
        const auto& A=*list[i];const auto& B=*list[j];
        const bool part=A.name=="WORKPIECE_STEP" || B.name=="WORKPIECE_STEP";
        if(part ? !workpieceMovingLink(A.name=="WORKPIECE_STEP"?B.name:A.name)
                : !shouldCheck(A.name,B.name))continue;
        if(aabbLowerBound(A.object->getAABB(),B.object->getAABB()) >=
           std::min(thresholdMm,result.distance_mm))continue;
        fcl::DistanceRequestd request;
        request.enable_nearest_points=false;
        fcl::DistanceResultd exact;
        const double distance=fcl::distance(A.object.get(),B.object.get(),request,exact);
        if(distance<result.distance_mm){
            result.distance_mm=distance;result.linkA=A.name;result.linkB=B.name;
        }
    }
    return result;
}

std::vector<CollisionHit> CollisionEngine::checkSphereObstacle(
    const Eigen::Vector3d& centerMm,
    double radiusMm,
    const std::vector<std::string>& linkNames,
    std::size_t maxHits) const
{
    std::vector<CollisionHit> hits;
    if (!(radiusMm > 0.0)) return hits;

    auto sphereGeometry = std::make_shared<fcl::Sphered>(radiusMm);
    fcl::Transform3d sphereTf = fcl::Transform3d::Identity();
    sphereTf.translation() = centerMm;
    fcl::CollisionObjectd sphere(sphereGeometry, sphereTf);
    sphere.computeAABB();

    for (const auto& name : linkNames) {
        const auto it = links_.find(name);
        if (it == links_.end()) continue;

        fcl::CollisionRequestd request;
        request.num_max_contacts = 1;
        request.enable_contact = true;
        fcl::CollisionResultd result;
        const std::size_t n = fcl::collide(it->second.object.get(), &sphere, request, result);
        if (n == 0 || !result.isCollision()) continue;

        CollisionHit hit;
        hit.linkA = name;
        hit.linkB = "STUDY_SPHERE";
        std::vector<fcl::Contactd> contacts;
        result.getContacts(contacts);
        if (!contacts.empty()) {
            hit.contact_mm = contacts.front().pos;
            hit.penetration_mm = contacts.front().penetration_depth;
        }
        hits.emplace_back(std::move(hit));
        if (hits.size() >= maxHits) break;
    }
    return hits;
}

DistanceInfo CollisionEngine::minimumDistanceToSphereObstacle(
    const Eigen::Vector3d& centerMm,
    double radiusMm,
    const std::vector<std::string>& linkNames) const
{
    DistanceInfo best;
    best.distance_mm = std::numeric_limits<double>::infinity();
    if (!(radiusMm > 0.0)) return best;

    auto sphereGeometry = std::make_shared<fcl::Sphered>(radiusMm);
    fcl::Transform3d sphereTf = fcl::Transform3d::Identity();
    sphereTf.translation() = centerMm;
    fcl::CollisionObjectd sphere(sphereGeometry, sphereTf);
    sphere.computeAABB();

    for (const auto& name : linkNames) {
        const auto it = links_.find(name);
        if (it == links_.end()) continue;
        fcl::DistanceRequestd request;
        request.enable_nearest_points = false;
        fcl::DistanceResultd result;
        const double d = fcl::distance(it->second.object.get(), &sphere, request, result);
        if (d < best.distance_mm) {
            best.distance_mm = d;
            best.linkA = name;
            best.linkB = "STUDY_SPHERE";
        }
    }
    return best;
}
