#pragma once
// V28 M0: portable, millimetre-based MCS/CAD geometry types. No Qt or device SDK.
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace mls::twin {
constexpr double kPi = 3.14159265358979323846;
struct Vec3 {
    double x = 0, y = 0, z = 0;
    Vec3 operator+(Vec3 b) const { return {x+b.x,y+b.y,z+b.z}; }
    Vec3 operator-(Vec3 b) const { return {x-b.x,y-b.y,z-b.z}; }
    Vec3 operator*(double t) const { return {x*t,y*t,z*t}; }
    Vec3 operator/(double t) const { return {x/t,y/t,z/t}; }
};
inline double dot(Vec3 a, Vec3 b) { return a.x*b.x+a.y*b.y+a.z*b.z; }
inline Vec3 cross(Vec3 a, Vec3 b) {
    return {a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x};
}
inline double length(Vec3 a) { return std::sqrt(dot(a,a)); }
inline Vec3 normalized(Vec3 a) { auto d=length(a); return d>1.e-15 ? a/d : Vec3{}; }

// CAD-to-MCS rigid placement. Euler convention: Rz(rz) * Ry(ry) * Rx(rx), angles degrees.
// NOT the machine's B/C joint convention; this is a separately specified workpiece placement.
struct Pose6 {
    double x=0,y=0,z=0, rxDeg=0,ryDeg=0,rzDeg=0;
};
struct Transform {
    // Row-major affine homogeneous matrix. p_M = cadToMachine * p_CAD.
    std::array<double,16> m{{1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1}};
    static Transform fromPose(const Pose6& pose);
    Vec3 point(Vec3 p) const;
    Vec3 vector(Vec3 v) const;
    Transform inverseRigid() const;
    Transform operator*(const Transform& rhs) const;
};
struct Bounds {
    Vec3 min{std::numeric_limits<double>::infinity(),std::numeric_limits<double>::infinity(),std::numeric_limits<double>::infinity()};
    Vec3 max{-std::numeric_limits<double>::infinity(),-std::numeric_limits<double>::infinity(),-std::numeric_limits<double>::infinity()};
    void add(Vec3 p);
    bool valid() const;
    Vec3 size() const { return max-min; }
    Vec3 center() const { return (min+max)*0.5; }
};
struct Triangle {
    Vec3 a,b,c;
    std::uint32_t faceId=0; // Stable within one import; no persistent CAD topology guarantee.
};
struct TriangleMesh {
    std::vector<Triangle> triangles;
    Bounds bounds;
    std::string source;
    void rebuildBounds();
    bool empty() const { return triangles.empty(); }
};
struct Ray {
    Vec3 origin{}, direction{0,0,1}; // Normalized for distances in mm.
};
struct RayHit {
    bool hit=false;
    double distanceMm=std::numeric_limits<double>::infinity();
    Vec3 positionMachine{}, normalMachine{};
    std::uint32_t faceId=0;
    std::string objectName;
};
} // namespace mls::twin
