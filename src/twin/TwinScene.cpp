#include "twin/TwinScene.h"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <utility>
#include <stdexcept>
#include <cctype>
namespace mls::twin {
namespace {
bool triangleHit(const Ray& ray, const Triangle& tri, double* t) {
    constexpr double eps=1e-10;
    const Vec3 e1=tri.b-tri.a, e2=tri.c-tri.a;
    const Vec3 h=cross(ray.direction,e2);
    const double determinant=dot(e1,h);
    if(std::abs(determinant)<eps) return false;
    const double inv=1./determinant;
    const Vec3 s=ray.origin-tri.a;
    const double u=dot(s,h)*inv;
    if(u < -eps || u > 1.+eps) return false;
    const double v=dot(ray.direction,cross(s,e1))*inv;
    if(v < -eps || u+v > 1.+eps) return false;
    const double d=dot(e2,cross(s,e1))*inv;
    if(d<=eps) return false;
    *t=d;return true;
}
}
bool TwinScene::setWorkpiece(TriangleMesh mesh, std::string* error) {
    if(mesh.empty()) {if(error)*error="Empty CAD triangle mesh";return false;}
    for(const auto& t:mesh.triangles) {
        const auto good=[](Vec3 p){return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z);};
        if(!good(t.a)||!good(t.b)||!good(t.c)) {if(error)*error="Mesh contains non-finite vertices";return false;}
    }
    mesh.rebuildBounds();
    if(!mesh.bounds.valid()) {if(error)*error="Invalid CAD mesh bounds";return false;}
    mesh_=std::move(mesh);return true;
}
void TwinScene::clear(){mesh_=TriangleMesh{};truth_=Pose6{};estimate_=Pose6{};supportEnabled_=false;}
bool TwinScene::loadObjMesh(const std::string& path, std::string* error) {
    std::ifstream is(path);
    if(!is){if(error)*error="Cannot open OBJ: "+path;return false;}
    std::vector<Vec3> v;
    TriangleMesh result;result.source=path;
    std::string line;int lineNo=0;
    while(std::getline(is,line)){
        ++lineNo;std::istringstream ss(line);
        std::string token;ss>>token;
        if(token=="v"){
            Vec3 p;if(!(ss>>p.x>>p.y>>p.z)) {if(error)*error="Invalid OBJ vertex at line "+std::to_string(lineNo);return false;}
            v.push_back(p);
        }else if(token=="f"){
            std::vector<std::size_t> inds;std::string part;
            while(ss>>part){
                const auto slash=part.find('/'); if(slash!=std::string::npos)part.resize(slash);
                try{
                    const auto index=std::stoll(part);
                    if(index<=0||static_cast<std::size_t>(index)>v.size())throw std::out_of_range("bad index");
                    inds.push_back(static_cast<std::size_t>(index-1));
                }catch(...){if(error)*error="Invalid OBJ face at line "+std::to_string(lineNo);return false;}
            }
            if(inds.size()<3){if(error)*error="Invalid OBJ polygon at line "+std::to_string(lineNo);return false;}
            const std::uint32_t faceId=static_cast<std::uint32_t>(result.triangles.size()+1);
            for(std::size_t i=1;i+1<inds.size();++i) result.triangles.push_back({v[inds[0]],v[inds[i]],v[inds[i+1]],faceId});
        }
    }
    return setWorkpiece(std::move(result),error);
}
Bounds TwinScene::workpieceBoundsMachine() const {
    Bounds bounds;
    if(mesh_.empty()||!mesh_.bounds.valid())return bounds;
    const auto m=cadToMachineTruth();
    const auto a=mesh_.bounds.min;
    const auto b=mesh_.bounds.max;
    for(double x:{a.x,b.x})for(double y:{a.y,b.y})for(double z:{a.z,b.z})bounds.add(m.point({x,y,z}));
    return bounds;
}
void TwinScene::setSupportPlane(bool enabled,double z,double cx,double cy,double w,double h){
    supportEnabled_=enabled;supportZ_=z;supportX_=cx;supportY_=cy;
    supportW_=std::max(0.,w);supportH_=std::max(0.,h);
}
RayHit TwinScene::firstHit(const Ray& input, double maxDist) const {
    RayHit result;
    const auto norm=length(input.direction);
    if(norm<=1e-12||maxDist<=0)return result;
    const Ray ray{input.origin,input.direction/norm};
    if(!mesh_.empty()){
        const Transform T=cadToMachineTruth();const Transform inv=T.inverseRigid();
        const Ray local{inv.point(ray.origin),inv.vector(ray.direction)};
        for(const auto& tri:mesh_.triangles){
            double d=0;
            if(triangleHit(local,tri,&d)&&d<result.distanceMm&&d<=maxDist){
                result.hit=true;result.distanceMm=d;result.positionMachine=ray.origin+ray.direction*d;
                result.normalMachine=normalized(T.vector(cross(tri.b-tri.a,tri.c-tri.a)));
                result.faceId=tri.faceId;result.objectName="workpiece";
            }
        }
    }
    if(supportEnabled_&&std::abs(ray.direction.z)>1.e-10){
        const double d=(supportZ_-ray.origin.z)/ray.direction.z;
        if(d>1.e-10&&d<result.distanceMm&&d<=maxDist){
            const auto p=ray.origin+ray.direction*d;
            if(std::abs(p.x-supportX_)<=supportW_*0.5&&std::abs(p.y-supportY_)<=supportH_*0.5){
                result.hit=true;result.distanceMm=d;result.positionMachine=p;
                result.normalMachine={0,0,1};result.faceId=0;result.objectName="support";
            }
        }
    }
    return result;
}
} // namespace mls::twin
