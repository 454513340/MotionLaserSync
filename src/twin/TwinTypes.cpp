#include "twin/TwinTypes.h"
#include <algorithm>
namespace mls::twin {
Transform Transform::fromPose(const Pose6& p) {
    const double a=p.rxDeg*kPi/180., b=p.ryDeg*kPi/180., c=p.rzDeg*kPi/180.;
    const double sa=std::sin(a),ca=std::cos(a),sb=std::sin(b),cb=std::cos(b),sc=std::sin(c),cc=std::cos(c);
    Transform t;
    t.m = {{ cc*cb, cc*sb*sa-sc*ca, cc*sb*ca+sc*sa,p.x,
             sc*cb, sc*sb*sa+cc*ca, sc*sb*ca-cc*sa,p.y,
             -sb,   cb*sa,          cb*ca,         p.z,
             0,0,0,1 }};
    return t;
}
Vec3 Transform::point(Vec3 p) const {
    return {m[0]*p.x+m[1]*p.y+m[2]*p.z+m[3],
            m[4]*p.x+m[5]*p.y+m[6]*p.z+m[7],
            m[8]*p.x+m[9]*p.y+m[10]*p.z+m[11]};
}
Vec3 Transform::vector(Vec3 p) const {
    return {m[0]*p.x+m[1]*p.y+m[2]*p.z,
            m[4]*p.x+m[5]*p.y+m[6]*p.z,
            m[8]*p.x+m[9]*p.y+m[10]*p.z};
}
Transform Transform::inverseRigid() const {
    Transform out;
    for(int i=0;i<3;++i) for(int j=0;j<3;++j)out.m[i*4+j]=m[j*4+i];
    const Vec3 t{m[3],m[7],m[11]};
    const Vec3 neg=out.vector(t)*(-1.);
    out.m[3]=neg.x;out.m[7]=neg.y;out.m[11]=neg.z;
    return out;
}
Transform Transform::operator*(const Transform& rhs) const {
    Transform result;
    for(int i=0;i<4;++i) for(int j=0;j<4;++j) {
        double sum=0.; for(int k=0;k<4;++k) sum+=m[i*4+k]*rhs.m[k*4+j];
        result.m[i*4+j]=sum;
    }
    return result;
}
void Bounds::add(Vec3 p) {
    min.x=std::min(min.x,p.x);min.y=std::min(min.y,p.y);min.z=std::min(min.z,p.z);
    max.x=std::max(max.x,p.x);max.y=std::max(max.y,p.y);max.z=std::max(max.z,p.z);
}
bool Bounds::valid() const {return std::isfinite(min.x)&&std::isfinite(min.y)&&std::isfinite(min.z)&&
    std::isfinite(max.x)&&std::isfinite(max.y)&&std::isfinite(max.z)&&
    min.x<=max.x&&min.y<=max.y&&min.z<=max.z;}
void TriangleMesh::rebuildBounds(){bounds=Bounds{}; for(const auto& tri:triangles){bounds.add(tri.a);bounds.add(tri.b);bounds.add(tri.c);}}
}
