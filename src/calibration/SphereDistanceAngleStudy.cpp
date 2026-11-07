#include "calibration/SphereDistanceAngleStudy.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>
#include <random>
#include <set>
#include <sstream>

namespace mls {
namespace calibration {
namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;

bool finite(double v) { return std::isfinite(v); }
bool finiteVec(const Vec3d& p) { return finite(p.x) && finite(p.y) && finite(p.z); }

Vec3d sub(const Vec3d& a, const Vec3d& b)
{
    return Vec3d(a.x - b.x, a.y - b.y, a.z - b.z);
}

double norm(const Vec3d& v)
{
    return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

double distance(const Vec3d& a, const Vec3d& b)
{
    return norm(sub(a, b));
}

bool solve3x3(double a[3][3], double b[3], double x[3])
{
    for (int col = 0; col < 3; ++col) {
        int pivot = col;
        double best = std::abs(a[col][col]);
        for (int r = col + 1; r < 3; ++r) {
            const double v = std::abs(a[r][col]);
            if (v > best) { best = v; pivot = r; }
        }
        if (!(best > 1e-14) || !finite(best)) return false;
        if (pivot != col) {
            for (int c = col; c < 3; ++c) std::swap(a[pivot][c], a[col][c]);
            std::swap(b[pivot], b[col]);
        }
        const double d = a[col][col];
        for (int c = col; c < 3; ++c) a[col][c] /= d;
        b[col] /= d;
        for (int r = 0; r < 3; ++r) {
            if (r == col) continue;
            const double f = a[r][col];
            if (f == 0.0) continue;
            for (int c = col; c < 3; ++c) a[r][c] -= f * a[col][c];
            b[r] -= f * b[col];
        }
    }
    for (int i = 0; i < 3; ++i) x[i] = b[i];
    return finite(x[0]) && finite(x[1]) && finite(x[2]);
}

bool solve4x4(double a[4][4], double b[4], double x[4])
{
    for (int col = 0; col < 4; ++col) {
        int pivot = col;
        double best = std::abs(a[col][col]);
        for (int r = col + 1; r < 4; ++r) {
            const double v = std::abs(a[r][col]);
            if (v > best) { best = v; pivot = r; }
        }
        if (!(best > 1e-16) || !finite(best)) return false;
        if (pivot != col) {
            for (int c = col; c < 4; ++c) std::swap(a[pivot][c], a[col][c]);
            std::swap(b[pivot], b[col]);
        }
        const double d = a[col][col];
        for (int c = col; c < 4; ++c) a[col][c] /= d;
        b[col] /= d;
        for (int r = 0; r < 4; ++r) {
            if (r == col) continue;
            const double f = a[r][col];
            if (f == 0.0) continue;
            for (int c = col; c < 4; ++c) a[r][c] -= f * a[col][c];
            b[r] -= f * b[col];
        }
    }
    for (int i = 0; i < 4; ++i) x[i] = b[i];
    return finite(x[0]) && finite(x[1]) && finite(x[2]) && finite(x[3]);
}

bool sphereFromFourPoints(const Vec3d& p0, const Vec3d& p1,
                          const Vec3d& p2, const Vec3d& p3,
                          Vec3d& center, double& radius)
{
    const Vec3d p[4] = {p0, p1, p2, p3};
    double a[3][3]{};
    double b[3]{};
    const double n0 = p0.x*p0.x + p0.y*p0.y + p0.z*p0.z;
    for (int i = 1; i < 4; ++i) {
        a[i-1][0] = 2.0 * (p[i].x - p0.x);
        a[i-1][1] = 2.0 * (p[i].y - p0.y);
        a[i-1][2] = 2.0 * (p[i].z - p0.z);
        b[i-1] = p[i].x*p[i].x + p[i].y*p[i].y + p[i].z*p[i].z - n0;
    }
    double x[3]{};
    if (!solve3x3(a, b, x)) return false;
    center = Vec3d(x[0], x[1], x[2]);
    radius = distance(center, p0);
    return finiteVec(center) && finite(radius) && radius > 1e-9;
}

std::vector<std::size_t> makeWorkingIndices(std::size_t count, std::size_t cap)
{
    std::vector<std::size_t> idx;
    if (count == 0) return idx;
    const std::size_t use = (cap == 0) ? count : std::min(count, cap);
    idx.reserve(use);
    if (use == count) {
        for (std::size_t i = 0; i < count; ++i) idx.push_back(i);
        return idx;
    }
    const double step = static_cast<double>(count) / static_cast<double>(use);
    for (std::size_t k = 0; k < use; ++k) {
        const std::size_t i = std::min(count - 1,
            static_cast<std::size_t>(std::floor((static_cast<double>(k) + 0.5) * step)));
        idx.push_back(i);
    }
    return idx;
}

bool radiusAllowed(double r, const SphereCloudRansacOptions& o)
{
    if (!(r > 0.0) || !finite(r)) return false;
    if (o.expectedRadiusMm > 0.0 && o.maxRadiusDeviationMm >= 0.0) {
        return std::abs(r - o.expectedRadiusMm) <= o.maxRadiusDeviationMm;
    }
    return true;
}

struct Score
{
    std::size_t count = 0;
    double rms = std::numeric_limits<double>::infinity();
};

Score scoreSphere(const std::vector<Vec3d>& points,
                  const std::vector<std::size_t>& indices,
                  const Vec3d& center, double radius, double threshold)
{
    Score s;
    double ss = 0.0;
    for (std::size_t i : indices) {
        const Vec3d& p = points[i];
        if (!finiteVec(p)) continue;
        const double e = distance(p, center) - radius;
        if (std::abs(e) <= threshold) {
            ++s.count;
            ss += e * e;
        }
    }
    if (s.count > 0) s.rms = std::sqrt(ss / static_cast<double>(s.count));
    return s;
}

std::vector<std::size_t> classifySphere(const std::vector<Vec3d>& points,
                                        const Vec3d& center, double radius,
                                        double threshold)
{
    std::vector<std::size_t> inliers;
    inliers.reserve(points.size() / 4 + 16);
    for (std::size_t i = 0; i < points.size(); ++i) {
        if (!finiteVec(points[i])) continue;
        if (std::abs(distance(points[i], center) - radius) <= threshold) inliers.push_back(i);
    }
    return inliers;
}

bool refineSphere(const std::vector<Vec3d>& points,
                  const std::vector<std::size_t>& allInliers,
                  const SphereCloudRansacOptions& options,
                  Vec3d& center, double& radius)
{
    if (allInliers.size() < 4) return false;
    std::vector<std::size_t> inliers = allInliers;
    if (options.maxRefinePoints > 0 && inliers.size() > options.maxRefinePoints) {
        std::vector<std::size_t> reduced;
        reduced.reserve(options.maxRefinePoints);
        const double step = static_cast<double>(inliers.size()) /
                            static_cast<double>(options.maxRefinePoints);
        for (std::size_t k = 0; k < options.maxRefinePoints; ++k) {
            reduced.push_back(inliers[std::min(inliers.size() - 1,
                static_cast<std::size_t>(std::floor((k + 0.5) * step)))]);
        }
        inliers.swap(reduced);
    }

    const double huber = std::max(options.inlierThresholdMm, 1e-6);
    for (int iteration = 0; iteration < std::max(1, options.refineIterations); ++iteration) {
        double n[4][4]{};
        double rhs[4]{};
        std::size_t used = 0;
        for (std::size_t i : inliers) {
            const Vec3d& p = points[i];
            const double dx = center.x - p.x;
            const double dy = center.y - p.y;
            const double dz = center.z - p.z;
            const double d = std::sqrt(dx*dx + dy*dy + dz*dz);
            if (!(d > 1e-12) || !finite(d)) continue;
            const double residual = d - radius;
            const double ar = std::abs(residual);
            const double w = (ar <= huber || ar <= 1e-15) ? 1.0 : huber / ar;
            const double j[4] = { dx/d, dy/d, dz/d, -1.0 };
            for (int r = 0; r < 4; ++r) {
                rhs[r] += -w * j[r] * residual;
                for (int c = 0; c < 4; ++c) n[r][c] += w * j[r] * j[c];
            }
            ++used;
        }
        if (used < 4) return false;
        for (int i = 0; i < 4; ++i) n[i][i] += 1e-12;
        double delta[4]{};
        if (!solve4x4(n, rhs, delta)) return false;
        center.x += delta[0]; center.y += delta[1]; center.z += delta[2]; radius += delta[3];
        if (!finiteVec(center) || !finite(radius) || radius <= 0.0) return false;
        const double step = std::sqrt(delta[0]*delta[0] + delta[1]*delta[1] +
                                      delta[2]*delta[2] + delta[3]*delta[3]);
        if (step < 1e-10) break;
    }
    return true;
}

bool inLinearLimits(const JointPose5d& q, const SphereStudyPathOptions& o)
{
    return q.x >= o.xMinMm && q.x <= o.xMaxMm &&
           q.y >= o.yMinMm && q.y <= o.yMaxMm &&
           q.z >= o.zMinMm && q.z <= o.zMaxMm;
}

bool inRotaryLimits(double c, double b, const SphereStudyPathOptions& o)
{
    return finite(c) && finite(b) &&
           c >= o.cMinDeg && c <= o.cMaxDeg &&
           b >= o.bMinDeg && b <= o.bMaxDeg;
}

Vec3d add(const Vec3d& a, const Vec3d& b)
{
    return Vec3d(a.x + b.x, a.y + b.y, a.z + b.z);
}

Vec3d mul(const Vec3d& a, double s)
{
    return Vec3d(a.x*s, a.y*s, a.z*s);
}

double dot(const Vec3d& a, const Vec3d& b)
{
    return a.x*b.x + a.y*b.y + a.z*b.z;
}

Vec3d normalized(const Vec3d& v)
{
    const double n = norm(v);
    return (n > 1e-15 && finite(n)) ? mul(v, 1.0/n) : Vec3d();
}

double clampUnit(double v)
{
    return std::max(-1.0, std::min(1.0, v));
}

double radToDeg(double v) { return v * 180.0 / kPi; }
double degToRad(double v) { return v * kPi / 180.0; }

// V27.6.16AF: exact standard measuring-field geometry used by the sphere scan
// digital twin, with the physical optical depth kept separate from the
// processed Sensor-Z coordinate convention.
//
// The real scanCONTROL 29xx-50 field is narrow at the sensor side and widens
// away from the sensor (42 -> 58 mm).  This project's calibrated profiles use
// FlipZ/PROC_FLIP_DISTANCE, so the *reported* Z ordering is reversed relative
// to physical depth.  In that convention the near/small edge is at numerical
// ZMax and the far/large edge is at numerical ZMin.  The old AE code attached
// 42 mm to ZMin and 58 mm to ZMax unconditionally, which drew and clipped the
// trapezoid backwards in the hand-eye/machine scene.
struct GuardedSensorField
{
    bool valid = false;
    bool trapezoidal = false;
    bool zCoordinateFlipped = false;
    double rawZMin = 0.0;
    double rawZMax = 0.0;
    double zLo = 0.0;
    double zHi = 0.0;
    double nearZ = 0.0;
    double farZ = 0.0;
    double xCenter = 0.0;
    double rectangularXLo = 0.0;
    double rectangularXHi = 0.0;
    double spanNear = 0.0;
    double spanFar = 0.0;
    double xMargin = 0.0;
    std::array<Vec3d,4> corners{};

    bool xBoundsAt(double z, double& xLo, double& xHi) const
    {
        if (!valid || z < zLo - 1e-9 || z > zHi + 1e-9) return false;
        if (!trapezoidal) {
            xLo = rectangularXLo;
            xHi = rectangularXHi;
            return xLo < xHi;
        }
        if (!(rawZMin < rawZMax)) return false;
        // Physical optical-depth fraction: 0 at the sensor-side/small edge,
        // 1 at the far/large edge.  FlipZ reverses only the numerical Z order;
        // it must never reverse the physical trapezoid.
        const double tNumeric = std::max(0.0, std::min(1.0,
            (z - rawZMin) / (rawZMax - rawZMin)));
        const double tDepth = zCoordinateFlipped ? (1.0 - tNumeric) : tNumeric;
        const double span = spanNear + tDepth * (spanFar - spanNear);
        const double guardedHalfSpan = 0.5 * span - xMargin;
        if (!(guardedHalfSpan > 0.0) || !finite(guardedHalfSpan)) return false;
        xLo = xCenter - guardedHalfSpan;
        xHi = xCenter + guardedHalfSpan;
        return xLo < xHi;
    }

    bool contains(const Vec3d& p) const
    {
        if (!valid || p.z < zLo - 1e-9 || p.z > zHi + 1e-9) return false;
        double xLo=0.0, xHi=0.0;
        return xBoundsAt(p.z, xLo, xHi) && p.x >= xLo - 1e-9 && p.x <= xHi + 1e-9;
    }
};

GuardedSensorField makeGuardedSensorField(const SphereStudyPathOptions& options)
{
    GuardedSensorField f;
    f.rawZMin = options.sensorZMinMm;
    f.rawZMax = options.sensorZMaxMm;
    f.zLo = options.sensorZMinMm + std::max(0.0, options.sensorZSafetyMarginMm);
    f.zHi = options.sensorZMaxMm - std::max(0.0, options.sensorZSafetyMarginMm);
    f.xMargin = std::max(0.0, options.sensorXSafetyMarginMm);
    f.xCenter = 0.5 * (options.sensorXMinMm + options.sensorXMaxMm);
    f.trapezoidal = options.useTrapezoidalSensorField;
    f.zCoordinateFlipped = options.sensorZCoordinateFlipped;

    if (!(f.rawZMin < f.rawZMax) || !(f.zLo < f.zHi)) return f;

    if (f.trapezoidal) {
        f.spanNear = options.sensorXSpanNearMm;
        f.spanFar = options.sensorXSpanFarMm;
        if (!(f.spanNear > 2.0*f.xMargin) ||
            !(f.spanFar > 2.0*f.xMargin) ||
            !finite(f.spanNear) || !finite(f.spanFar)) return f;
    } else {
        f.rectangularXLo = options.sensorXMinMm + f.xMargin;
        f.rectangularXHi = options.sensorXMaxMm - f.xMargin;
        if (!(f.rectangularXLo < f.rectangularXHi)) return f;
    }

    f.valid = true;
    f.nearZ = f.zCoordinateFlipped ? f.zHi : f.zLo;
    f.farZ  = f.zCoordinateFlipped ? f.zLo : f.zHi;
    double nearLo=0.0, nearHi=0.0, farLo=0.0, farHi=0.0;
    if (!f.xBoundsAt(f.nearZ, nearLo, nearHi) || !f.xBoundsAt(f.farZ, farLo, farHi)) {
        f.valid = false;
        return f;
    }
    // Keep polygon order tied to PHYSICAL depth, not numeric Sensor-Z: the
    // first edge is always the small edge at the sensor, the second is the
    // large edge away from the sensor.  This makes the VTK visualization and
    // the clipping predicate describe the same physical field.
    f.corners = {{
        Vec3d(nearLo, 0.0, f.nearZ), Vec3d(nearHi, 0.0, f.nearZ),
        Vec3d(farHi, 0.0, f.farZ), Vec3d(farLo, 0.0, f.farZ)
    }};
    return f;
}

// ^E T_S maps a Sensor-frame point into the five-axis end-effector frame E.
// Therefore E's origin expressed in Sensor coordinates is -R_ES^T * t_ES.
// This is the side reference requested for the sphere contour: it follows the
// actual calibrated end-effector/sensor mounting instead of assuming machine +Z
// or using the study's arbitrary optical reference center.
Vec3d endEffectorOriginSensorMm(const Physical17Parameters& model)
{
    const auto h = SingleSphere17Model::handEyeTransform(model);
    const Vec3d t(h[3], h[7], h[11]);
    return Vec3d(
        -(h[0]*t.x + h[4]*t.y + h[8]*t.z),
        -(h[1]*t.x + h[5]*t.y + h[9]*t.z),
        -(h[2]*t.x + h[6]*t.y + h[10]*t.z));
}

bool endEffectorFacingSphereSide(const Vec3d& p,
                                 const Vec3d& sphereCenterSensorMm,
                                 const Vec3d& endEffectorSensorMm,
                                 bool required,
                                 double* projectionMm2=nullptr)
{
    const Vec3d radial = sub(p, sphereCenterSensorMm);
    const Vec3d towardEnd = sub(endEffectorSensorMm, sphereCenterSensorMm);
    const double projection = dot(radial, towardEnd);
    if (projectionMm2) *projectionMm2 = projection;
    if (!required) return true;
    return norm(radial) > 1e-9 && norm(towardEnd) > 1e-9 && projection > 0.0;
}

double quantileSorted(const std::vector<double>& sorted, double q)
{
    if (sorted.empty()) return std::numeric_limits<double>::quiet_NaN();
    if (sorted.size() == 1) return sorted.front();
    q = std::max(0.0, std::min(1.0, q));
    const double pos = q * static_cast<double>(sorted.size() - 1);
    const std::size_t lo = static_cast<std::size_t>(std::floor(pos));
    const std::size_t hi = static_cast<std::size_t>(std::ceil(pos));
    const double t = pos - static_cast<double>(lo);
    return sorted[lo] * (1.0 - t) + sorted[hi] * t;
}

double median(std::vector<double> values)
{
    values.erase(std::remove_if(values.begin(), values.end(), [](double v){ return !finite(v); }), values.end());
    if (values.empty()) return std::numeric_limits<double>::quiet_NaN();
    std::sort(values.begin(), values.end());
    return quantileSorted(values, 0.5);
}

int medianInt(std::vector<int> values)
{
    if (values.empty()) return -1;
    std::sort(values.begin(), values.end());
    return values[values.size()/2];
}

RobustScalarStats robustStats(std::vector<double> values)
{
    RobustScalarStats out;
    values.erase(std::remove_if(values.begin(), values.end(), [](double v){ return !finite(v); }), values.end());
    if (values.empty()) return out;
    std::sort(values.begin(), values.end());
    out.valid = true;
    out.count = values.size();
    out.median = quantileSorted(values, 0.5);
    out.p10 = quantileSorted(values, 0.10);
    out.p90 = quantileSorted(values, 0.90);
    std::vector<double> deviations;
    deviations.reserve(values.size());
    for (double v : values) deviations.push_back(std::abs(v - out.median));
    std::sort(deviations.begin(), deviations.end());
    out.mad = quantileSorted(deviations, 0.5);
    return out;
}

bool solveXyzForTarget(const Vec3d& sphereCenterMachineMm,
                       const Physical17Parameters& model,
                       const SphereStudyPathOptions& options,
                       const Vec3d& targetSphereCenterSensorMm,
                       double targetC, double targetB,
                       JointPose5d seed,
                       JointPose5d& solved,
                       Vec3d& predicted,
                       double& residual,
                       std::string& error)
{
    (void)seed; // C/B select the orientation; XYZ have an exact affine inverse.
    if (!inRotaryLimits(targetC, targetB, options)) {
        error = "Requested C/B sample is outside configured rotary limits.";
        return false;
    }
    if (!finiteVec(targetSphereCenterSensorMm) || !finiteVec(sphereCenterMachineMm)) {
        error = "Non-finite sphere target for RTCP-equivalent XYZ inverse.";
        return false;
    }

    // For prescribed C/B the calibrated sensor rotation is fixed.  If the
    // known machine-frame sphere center O_M must appear at S_target, then the
    // required sensor-origin position is t_target = O_M - R*S_target.
    JointPose5d base;
    base.x = base.y = base.z = 0.0; base.c = targetC; base.b = targetB;
    const auto tf = SingleSphere17Model::sensorToMachineTransform(base, model);
    const Vec3d ex(tf[0], tf[4], tf[8]);
    const Vec3d ey(tf[1], tf[5], tf[9]);
    const Vec3d ez(tf[2], tf[6], tf[10]);
    const Vec3d rTarget = add(add(mul(ex, targetSphereCenterSensorMm.x),
                                 mul(ey, targetSphereCenterSensorMm.y)),
                                 mul(ez, targetSphereCenterSensorMm.z));
    const Vec3d targetOriginMachine = sub(sphereCenterMachineMm, rTarget);

    const Vec3d p0 = SingleSphere17Model::mapSensorPoint(base, Vec3d(0.0,0.0,0.0), model);
    JointPose5d qx=base,qy=base,qz=base;
    qx.x=1.0; qy.y=1.0; qz.z=1.0;
    const Vec3d ux=sub(SingleSphere17Model::mapSensorPoint(qx, Vec3d(0.0,0.0,0.0), model),p0);
    const Vec3d uy=sub(SingleSphere17Model::mapSensorPoint(qy, Vec3d(0.0,0.0,0.0), model),p0);
    const Vec3d uz=sub(SingleSphere17Model::mapSensorPoint(qz, Vec3d(0.0,0.0,0.0), model),p0);

    double a[3][3] = {{ux.x,uy.x,uz.x},{ux.y,uy.y,uz.y},{ux.z,uy.z,uz.z}};
    const Vec3d rhs=sub(targetOriginMachine,p0);
    double b[3] = {rhs.x,rhs.y,rhs.z};
    double xyz[3]{};
    if (!solve3x3(a,b,xyz)) {
        error = "RTCP-equivalent XYZ linear basis is singular/ill-conditioned at this C/B pose.";
        return false;
    }
    solved.x=xyz[0]; solved.y=xyz[1]; solved.z=xyz[2]; solved.c=targetC; solved.b=targetB;
    if (!inLinearLimits(solved, options)) {
        error = "RTCP-equivalent XYZ solution leaves configured machine software limits.";
        return false;
    }
    predicted = SingleSphere17Model::machinePointInSensor(solved, sphereCenterMachineMm, model);
    if (!finiteVec(predicted)) {
        error = "Physical17 produced a non-finite sensor-frame center after inverse.";
        return false;
    }
    residual = distance(predicted, targetSphereCenterSensorMm);
    if (!finite(residual) || residual > std::max(1e-8, options.targetToleranceMm)) {
        std::ostringstream os;
        os << "RTCP-equivalent inverse/forward residual=" << residual << " mm at C="
           << targetC << " B=" << targetB << ".";
        error = os.str();
        return false;
    }
    return true;
}

Vec3d profilePlaneNormalMachine(double cDeg, double bDeg,
                                const Physical17Parameters& model)
{
    JointPose5d q;
    q.x = 0.0; q.y = 0.0; q.z = 0.0; q.c = cDeg; q.b = bDeg;
    const auto t = SingleSphere17Model::sensorToMachineTransform(q, model);
    // Column 1 of the row-major transform is sensor +Y in machine frame;
    // sensor Y=0 is the laser profile plane.
    return normalized(Vec3d(t[1], t[5], t[9]));
}

double unsignedPlaneAngleDeg(const Vec3d& a, const Vec3d& b)
{
    const Vec3d na = normalized(a), nb = normalized(b);
    if (!(norm(na) > 0.5) || !(norm(nb) > 0.5)) return 0.0;
    // n and -n describe the same geometric plane.  Use |dot| so the maximin
    // selector spreads actual section-plane orientations instead of spending
    // samples on duplicated normal signs.
    return radToDeg(std::acos(clampUnit(std::abs(dot(na, nb)))));
}

SphereStudyOpticalFeasibility simulateOpticalFeasibilityImpl(
    const JointPose5d& joints,
    const Vec3d& sphereCenterMachineMm,
    double sphereRadiusMm,
    const Physical17Parameters& model,
    const SphereStudyPathOptions& options)
{
    SphereStudyOpticalFeasibility out;
    if (!(sphereRadiusMm > 0.0) || !finiteVec(sphereCenterMachineMm) ||
        !finiteVec(options.opticalCenterSensorMm)) {
        out.message = "Invalid sphere/FOV simulation inputs.";
        return out;
    }

    out.sphereCenterSensorMm = SingleSphere17Model::machinePointInSensor(
        joints, sphereCenterMachineMm, model);
    if (!finiteVec(out.sphereCenterSensorMm)) {
        out.message = "Forward model produced a non-finite sphere center in sensor coordinates.";
        return out;
    }

    const double h = std::abs(out.sphereCenterSensorMm.y);
    out.planeIntersectionMarginMm = sphereRadiusMm - h;
    // Keep the pure geometric fact separate from the engineering margin gate.
    // This matters for the GUI digital twin: a near-tangent section should be
    // rendered as a tiny circle and marked rejected, not shown as if the plane
    // did not intersect the sphere at all.
    out.planeIntersectsSphere = h < sphereRadiusMm;
    if (out.planeIntersectsSphere) {
        out.sectionRadiusMm = std::sqrt(std::max(0.0, sphereRadiusMm*sphereRadiusMm - h*h));
        out.circleCenterSensorMm = Vec3d(out.sphereCenterSensorMm.x, 0.0, out.sphereCenterSensorMm.z);
    }
    if (!(out.planeIntersectionMarginMm > std::max(0.0, options.minimumPlaneIntersectionMarginMm))) {
        std::ostringstream os;
        os << "Laser plane misses/is too close to tangency: |Ysphere|=" << h
           << " mm, R=" << sphereRadiusMm << " mm, intersection margin="
           << out.planeIntersectionMarginMm << " mm (required > "
           << std::max(0.0, options.minimumPlaneIntersectionMarginMm) << " mm).";
        out.message = os.str();
        return out;
    }
    if (!(out.sectionRadiusMm > 1e-9) || !finiteVec(out.circleCenterSensorMm)) {
        out.message = "Predicted sphere-plane section is degenerate.";
        return out;
    }

    const GuardedSensorField field = makeGuardedSensorField(options);
    if (!field.valid) {
        out.message = "Configured scanCONTROL guarded X-Z trapezoidal field is empty/invalid.";
        return out;
    }

    out.endEffectorOriginSensorMm = endEffectorOriginSensorMm(model);
    if (!finiteVec(out.endEffectorOriginSensorMm)) {
        out.message = "Calibrated hand-eye produced a non-finite end-effector origin in Sensor coordinates.";
        return out;
    }
    if (options.requireEndEffectorFacingArc &&
        !(norm(sub(out.endEffectorOriginSensorMm, out.sphereCenterSensorMm)) > 1e-6)) {
        out.message = "End-effector origin is degenerate with the sphere center; cannot select the near-side contour.";
        return out;
    }

    const int samples = std::max(72, options.visibleArcSamples);
    std::vector<unsigned char> good(static_cast<std::size_t>(samples), 0);
    int goodCount = 0;
    double minSideProjection = std::numeric_limits<double>::infinity();
    for (int i = 0; i < samples; ++i) {
        const double a = 2.0*kPi*static_cast<double>(i)/static_cast<double>(samples);
        const Vec3d p(out.circleCenterSensorMm.x + out.sectionRadiusMm*std::cos(a),
                      0.0,
                      out.circleCenterSensorMm.z + out.sectionRadiusMm*std::sin(a));

        // The measurement is not "circle intersection only".  A simulated
        // profile sample must be inside the real trapezoidal X-Z measuring
        // field AND on the sphere hemisphere facing the current five-axis
        // kinematic end-effector.  The second condition is coordinate-system
        // invariant and avoids the old lower/upper-surface sign assumption.
        if (!field.contains(p)) continue;
        double sideProjection=0.0;
        if (!endEffectorFacingSphereSide(p, out.sphereCenterSensorMm,
                                         out.endEffectorOriginSensorMm,
                                         options.requireEndEffectorFacingArc,
                                         &sideProjection)) continue;
        good[static_cast<std::size_t>(i)] = 1;
        ++goodCount;
        minSideProjection = std::min(minSideProjection, sideProjection);
    }
    out.minimumEndEffectorSideProjectionMm2 =
        finite(minSideProjection) ? minSideProjection : 0.0;

    // Longest circular run that simultaneously lies inside the actual
    // trapezoidal measuring field and on the sphere side facing the five-axis
    // end-effector.  This is the only synthetic contour accumulated as a
    // simulated Profile for Circle-RANSAC.
    int longest = 0, run = 0;
    for (int k = 0; k < samples*2; ++k) {
        if (good[static_cast<std::size_t>(k % samples)]) {
            run = std::min(run + 1, samples);
            longest = std::max(longest, run);
        } else {
            run = 0;
        }
    }
    longest = std::min(longest, samples);
    out.visibleArcDeg = 360.0 * static_cast<double>(longest) / static_cast<double>(samples);
    out.visibleArcLengthMm = out.sectionRadiusMm * degToRad(out.visibleArcDeg);
    out.visibleChordMm = 2.0 * out.sectionRadiusMm *
        std::sin(0.5 * degToRad(std::min(180.0, out.visibleArcDeg)));
    out.visibleFraction = static_cast<double>(goodCount) / static_cast<double>(samples);
    const double minArcDeg = std::max(0.0, options.minimumPredictedVisibleArcDeg);
    const double minArcLengthMm = std::max(0.0, options.minimumPredictedVisibleArcLengthMm);
    out.visibleArcSufficient = out.visibleArcDeg + 1e-9 >= minArcDeg &&
        out.visibleArcLengthMm + 1e-9 >= minArcLengthMm;
    if (!out.visibleArcSufficient) {
        std::ostringstream os;
        os << "Predicted sphere section intersects the laser plane, but only "
           << out.visibleArcDeg << " deg / " << out.visibleArcLengthMm
           << " mm of the end-effector-facing contour lies inside the guarded "
           << (field.trapezoidal ? "trapezoidal" : "rectangular")
           << " scanCONTROL field; required >= " << minArcDeg
           << " deg and >= " << minArcLengthMm << " mm.";
        out.message = os.str();
        return out;
    }

    out.valid = true;
    out.message = "OK";
    return out;
}

std::vector<SphereStudyRotarySample> selectAutomaticPlaneFamily(
    const Vec3d& sphereCenterMachineMm,
    const Physical17Parameters& model,
    const SphereStudyPathOptions& options,
    std::string* error)
{
    if (error) error->clear();
    std::vector<SphereStudyRotarySample> feasible;
    const int wanted = std::max(1, options.planeDirectionsPerGeometry);
    if (options.measurementDistancesMm.empty() || options.sectionAnglesDeg.empty()) return feasible;

    const double dMid = options.measurementDistancesMm[options.measurementDistancesMm.size()/2];
    const double gMid = options.sectionAnglesDeg[options.sectionAnglesDeg.size()/2];
    const double eMid = options.centerBearingAnglesDeg.empty()
        ? 0.0 : options.centerBearingAnglesDeg[options.centerBearingAnglesDeg.size()/2];
    const auto tg = SphereDistanceAngleStudy::targetGeometry(
        options.sphereRadiusMm, dMid, gMid, eMid, options.sectionSideSign,
        options.opticalCenterSensorMm);
    if (!(tg.measurementDistanceMm > 0.0) || !finiteVec(tg.targetSphereCenterSensorMm)) return feasible;

    struct Candidate { SphereStudyRotarySample r; double seedDistance = 0.0; };
    std::vector<Candidate> candidates;
    auto tryCandidate = [&](double cDeg, double bDeg) {
        if (!inRotaryLimits(cDeg, bDeg, options)) return;
        for (const auto& e : candidates) {
            if (std::abs(e.r.cDeg-cDeg) < 1e-8 && std::abs(e.r.bDeg-bDeg) < 1e-8) return;
        }
        JointPose5d solved; Vec3d predicted; double residual = 0.0; std::string err;
        if (!solveXyzForTarget(sphereCenterMachineMm, model, options,
                               tg.targetSphereCenterSensorMm, cDeg, bDeg,
                               options.referenceJoint, solved, predicted, residual, err)) return;
        SphereStudyRotarySample r;
        r.cDeg = cDeg; r.bDeg = bDeg;
        r.planeNormalMachine = profilePlaneNormalMachine(cDeg, bDeg, model);
        if (!(norm(r.planeNormalMachine) > 0.5)) return;
        Candidate c; c.r = r;
        c.seedDistance = std::hypot(cDeg-options.referenceJoint.c,
                                    bDeg-options.referenceJoint.b);
        candidates.push_back(c);
    };

    const double step = std::max(2.0, std::abs(options.rotaryCandidateStepDeg));
    if (!finite(options.cMinDeg) || !finite(options.cMaxDeg) ||
        !finite(options.bMinDeg) || !finite(options.bMaxDeg) ||
        !(options.cMinDeg < options.cMaxDeg) || !(options.bMinDeg < options.bMaxDeg)) {
        if (error) *error = "Automatic plane-family C/B limits are non-finite or invalid.";
        return feasible;
    }

    // Integer-indexed traversal guarantees termination and lets us reject an
    // accidental placeholder range before entering the C x B nested loop.
    const auto gridCount = [step](double lo, double hi) -> std::size_t {
        const long double span = static_cast<long double>(hi) - static_cast<long double>(lo);
        if (!(span >= 0.0L) || !std::isfinite(static_cast<double>(span))) return 0;
        const long double intervals = span / static_cast<long double>(step);
        const long double rounded = std::round(intervals);
        const bool exact = std::abs(intervals-rounded) <= 1.0e-12L;
        const long double n = (exact ? rounded : std::floor(intervals) + 1.0L) + 1.0L;
        if (!(n >= 1.0L) || n > static_cast<long double>((std::numeric_limits<std::size_t>::max)())) return 0;
        return static_cast<std::size_t>(n);
    };
    const std::size_t cCount = gridCount(options.cMinDeg, options.cMaxDeg);
    const std::size_t bCount = gridCount(options.bMinDeg, options.bMaxDeg);
    const std::size_t cap = std::max<std::size_t>(1, options.maxAutomaticRotaryCandidateCount);
    if (cCount == 0 || bCount == 0 || cCount > cap || bCount > cap || cCount > cap / bCount) {
        if (error) {
            std::ostringstream os;
            os << "Automatic C/B search grid is too large/invalid: C=[" << options.cMinDeg
               << "," << options.cMaxDeg << "], B=[" << options.bMinDeg << ","
               << options.bMaxDeg << "], step=" << step << " deg, grid="
               << cCount << "x" << bCount << ", hard cap=" << cap
               << ". Refuse to enumerate; check offline/controller software limits.";
            *error = os.str();
        }
        return feasible;
    }
    // Always test the current orientation first after the range/candidate
    // guard has proved that the automatic lattice itself is finite.
    tryCandidate(options.referenceJoint.c, options.referenceJoint.b);

    for (std::size_t bi = 0; bi < bCount; ++bi) {
        const double b = (bi + 1 == bCount) ? options.bMaxDeg
            : std::min(options.bMaxDeg, options.bMinDeg + step*static_cast<double>(bi));
        for (std::size_t ci = 0; ci < cCount; ++ci) {
            const double c = (ci + 1 == cCount) ? options.cMaxDeg
                : std::min(options.cMaxDeg, options.cMinDeg + step*static_cast<double>(ci));
            tryCandidate(c, b);
        }
    }

    if (candidates.empty()) return feasible;

    // First direction: closest feasible orientation to the current machine pose.
    std::size_t first = 0;
    for (std::size_t i=1; i<candidates.size(); ++i)
        if (candidates[i].seedDistance < candidates[first].seedDistance) first = i;
    feasible.push_back(candidates[first].r);
    std::vector<bool> used(candidates.size(), false); used[first] = true;

    // Remaining directions: farthest-point / maximin selection in plane-normal
    // space.  This creates a compact but diverse family of sphere section
    // planes and avoids the old blind C x B Cartesian product.
    while (static_cast<int>(feasible.size()) < wanted) {
        std::size_t best = candidates.size();
        double bestScore = -1.0;
        for (std::size_t i=0; i<candidates.size(); ++i) {
            if (used[i]) continue;
            double minAngle = 1.0e9;
            for (const auto& s : feasible)
                minAngle = std::min(minAngle, unsignedPlaneAngleDeg(
                    candidates[i].r.planeNormalMachine, s.planeNormalMachine));
            if (minAngle > bestScore + 1e-12 ||
                (std::abs(minAngle-bestScore) <= 1e-12 &&
                 (best==candidates.size() || candidates[i].seedDistance < candidates[best].seedDistance))) {
                bestScore = minAngle; best = i;
            }
        }
        if (best == candidates.size()) break;
        if (bestScore + 1e-9 < std::max(0.0, options.minimumPlaneSeparationDeg)) break;
        used[best] = true;
        feasible.push_back(candidates[best].r);
    }

    for (std::size_t i=0; i<feasible.size(); ++i) feasible[i].id = static_cast<int>(i);
    return feasible;
}

struct PlannedCandidate
{
    SphereStudyPose pose;
    Vec3d planeNormalMachine;
};

double normalizedDelta(double a, double b, double lo, double hi)
{
    const double span = std::max(1e-9, hi - lo);
    return (a - b) / span;
}

double geometryFeatureDistance(const PlannedCandidate& a,
                               const PlannedCandidate& b,
                               double dLo, double dHi,
                               double gLo, double gHi,
                               double eLo, double eHi)
{
    const double dd = normalizedDelta(a.pose.geometry.measurementDistanceMm,
                                      b.pose.geometry.measurementDistanceMm,
                                      dLo, dHi);
    const double dg = normalizedDelta(a.pose.geometry.sectionAngleDeg,
                                      b.pose.geometry.sectionAngleDeg,
                                      gLo, gHi);
    const double de = normalizedDelta(a.pose.geometry.centerBearingAngleDeg,
                                      b.pose.geometry.centerBearingAngleDeg,
                                      eLo, eHi);
    // Plane normals are unoriented: n and -n represent the same section plane.
    const double dp = unsignedPlaneAngleDeg(a.planeNormalMachine,
                                            b.planeNormalMachine) / 90.0;
    return std::sqrt(dd*dd + dg*dg + de*de + dp*dp);
}

double jointMotionCost(const JointPose5d& a, const JointPose5d& b)
{
    const double dx=(a.x-b.x)/50.0, dy=(a.y-b.y)/50.0, dz=(a.z-b.z)/50.0;
    const double dc=(a.c-b.c)/30.0, db=(a.b-b.b)/30.0;
    return dx*dx+dy*dy+dz*dz+dc*dc+db*db;
}

std::vector<SphereStudyPose> selectSpaceFillingPath(
    const std::vector<PlannedCandidate>& candidates,
    const SphereStudyPathOptions& options)
{
    std::vector<SphereStudyPose> out;
    if (candidates.empty()) return out;
    const int requested = std::max(1, options.targetPoseCount);
    const std::size_t wanted = std::min<std::size_t>(candidates.size(),
                                                     static_cast<std::size_t>(requested));

    auto range = [](const std::vector<double>& v) {
        const auto mm = std::minmax_element(v.begin(), v.end());
        return std::pair<double,double>(*mm.first,*mm.second);
    };
    const auto dr = range(options.measurementDistancesMm);
    const auto gr = range(options.sectionAnglesDeg);
    const std::vector<double> etaValues = options.centerBearingAnglesDeg.empty()
        ? std::vector<double>{0.0} : options.centerBearingAnglesDeg;
    const auto er = range(etaValues);

    // Start close to the current machine state to keep the first autonomous
    // move low-risk, then greedily maximize the minimum normalized geometry
    // distance to all already-selected samples.
    std::vector<std::size_t> selected;
    std::vector<bool> used(candidates.size(), false);
    std::size_t first = 0;
    double firstCost = jointMotionCost(candidates.front().pose.joints,
                                       options.referenceJoint);
    for (std::size_t i=1;i<candidates.size();++i) {
        const double c = jointMotionCost(candidates[i].pose.joints, options.referenceJoint);
        if (c < firstCost) { firstCost=c; first=i; }
    }
    selected.push_back(first); used[first]=true;
    while (selected.size() < wanted) {
        std::size_t best = candidates.size();
        double bestScore = -1.0;
        for (std::size_t i=0;i<candidates.size();++i) {
            if (used[i]) continue;
            double minDist = std::numeric_limits<double>::infinity();
            for (std::size_t s : selected) {
                minDist = std::min(minDist, geometryFeatureDistance(
                    candidates[i], candidates[s],
                    dr.first,dr.second,gr.first,gr.second,er.first,er.second));
            }
            if (minDist > bestScore) { bestScore=minDist; best=i; }
        }
        if (best==candidates.size()) break;
        used[best]=true; selected.push_back(best);
    }

    // Geometry selection and execution ordering are separate concerns.  Keep
    // the maximin sample set, then order it by a deterministic nearest-neighbor
    // joint-space route to reduce unnecessary XYZBC travel.
    std::vector<bool> routeUsed(selected.size(), false);
    JointPose5d current = options.referenceJoint;
    out.reserve(selected.size());
    for (std::size_t step=0; step<selected.size(); ++step) {
        std::size_t bestLocal = selected.size();
        double bestCost = std::numeric_limits<double>::infinity();
        for (std::size_t k=0;k<selected.size();++k) {
            if (routeUsed[k]) continue;
            const double c = jointMotionCost(candidates[selected[k]].pose.joints, current);
            if (c < bestCost) { bestCost=c; bestLocal=k; }
        }
        if (bestLocal==selected.size()) break;
        routeUsed[bestLocal]=true;
        SphereStudyPose p = candidates[selected[bestLocal]].pose;
        p.sequence = static_cast<int>(out.size());
        current = p.joints;
        out.push_back(std::move(p));
    }
    return out;
}

struct CircleExtraction
{
    SphereCenterFit fit;
    std::vector<std::size_t> inliers;
    double thresholdMm = 0.0;
};

std::vector<SensorProfilePoint> makeRansacPoints(const SphereStudyRawProfile& raw,
                                                  const Vec3d& predictedCenter,
                                                  double sphereRadius,
                                                  double margin,
                                                  bool usePredictedRoi)
{
    std::vector<SensorProfilePoint> points(raw.samples.size());
    const double half = sphereRadius + std::max(0.0, margin);
    for (std::size_t i = 0; i < raw.samples.size(); ++i) {
        const auto& s = raw.samples[i];
        SensorProfilePoint p;
        p.x = s.xMm;
        p.z = s.zMm;
        p.weight = 0.0;
        if (s.coordinateValid && finite(s.xMm) && finite(s.zMm)) {
            if (!usePredictedRoi ||
                (std::abs(s.xMm - predictedCenter.x) <= half &&
                 std::abs(s.zMm - predictedCenter.z) <= half)) {
                p.weight = 1.0;
            }
        }
        points[i] = p;
    }
    return points;
}

CircleExtraction extractCircle(const SphereStudyRawProfile& raw,
                               const Vec3d& predictedCenter,
                               double sphereRadius,
                               int centerYSign,
                               const SphereStudyProfileAnalysisOptions& options)
{
    CircleExtraction out;
    out.thresholdMm = options.ransac.inlierThresholdMm;
    auto points = makeRansacPoints(raw, predictedCenter, sphereRadius,
                                   options.predictedSearchMarginMm, true);
    int weighted = 0;
    for (const auto& p : points) if (p.weight > 0.0) ++weighted;
    if (weighted < 5) points = makeRansacPoints(raw, predictedCenter, sphereRadius,
                                                options.predictedSearchMarginMm, false);

    SphereArcRansacOptions first = options.ransac;
    out.fit = SingleSphere17Calibrator::fitSphereCenterInSensor(
        points, 2.0*sphereRadius, centerYSign, first, &out.inliers);

    // The existing Physical17/machine model is only an approximate guide for
    // finding the ball.  A stale calibration must never make the real sphere
    // disappear from the study.  If the predicted search window cannot form a
    // stable circle, retry once against every coordinate-valid sample in the
    // profile.  Rich signal channels are deliberately NOT used as a gate.
    if ((!out.fit.success || out.inliers.size() < 5) && weighted >= 5) {
        points = makeRansacPoints(raw, predictedCenter, sphereRadius,
                                  options.predictedSearchMarginMm, false);
        out.inliers.clear();
        out.fit = SingleSphere17Calibrator::fitSphereCenterInSensor(
            points, 2.0*sphereRadius, centerYSign, first, &out.inliers);
    }
    if (!out.fit.success || out.inliers.size() < 5) return out;

    std::vector<double> residuals;
    residuals.reserve(out.inliers.size());
    for (std::size_t idx : out.inliers) {
        if (idx >= points.size()) continue;
        const auto& p = points[idx];
        if (!(p.weight > 0.0)) continue;
        residuals.push_back(std::hypot(p.x - out.fit.centerSensorMm.x,
                                       p.z - out.fit.centerSensorMm.z)
                            - out.fit.sectionRadiusMm);
    }
    const double med = median(residuals);
    std::vector<double> absDev;
    absDev.reserve(residuals.size());
    for (double r : residuals) absDev.push_back(std::abs(r - med));
    const double mad = median(absDev);
    const double sigma = finite(mad) ? 1.4826 * mad : 0.0;
    const double adaptive = std::max(options.adaptiveMinThresholdMm,
        std::min(options.ransac.inlierThresholdMm,
                 std::max(options.adaptiveMinThresholdMm,
                          options.adaptiveMadMultiplier * sigma)));
    out.thresholdMm = adaptive;

    if (adaptive < first.inlierThresholdMm * 0.98) {
        SphereArcRansacOptions second = first;
        second.inlierThresholdMm = adaptive;
        std::vector<std::size_t> refinedInliers;
        const auto refined = SingleSphere17Calibrator::fitSphereCenterInSensor(
            points, 2.0*sphereRadius, centerYSign, second, &refinedInliers);
        if (refined.success && refinedInliers.size() >= 5) {
            out.fit = refined;
            out.inliers = std::move(refinedInliers);
        }
    }
    return out;
}

struct SectionGeometry
{
    bool valid = false;
    double distanceMm = 0.0;
    double hMm = 0.0;
    double gammaDeg = 0.0;
    double etaDeg = 0.0;
    double theoreticalRadiusMm = 0.0;
    Vec3d circleCenterMachine;
};

SectionGeometry sectionGeometryFromCircle(const JointPose5d& joints,
                                          const Vec3d& circleCenterSensor,
                                          const Vec3d& sphereCenterMachine,
                                          double sphereRadius,
                                          const Physical17Parameters& model,
                                          const Vec3d& opticalCenterSensor)
{
    SectionGeometry out;
    if (!(sphereRadius > 0.0) || !finiteVec(circleCenterSensor) || !finiteVec(opticalCenterSensor)) return out;
    out.distanceMm = distance(circleCenterSensor, opticalCenterSensor);
    const double dxOpt = circleCenterSensor.x - opticalCenterSensor.x;
    const double dzOpt = circleCenterSensor.z - opticalCenterSensor.z;
    out.etaDeg = radToDeg(std::atan2(dxOpt, dzOpt));
    out.circleCenterMachine = SingleSphere17Model::mapSensorPoint(joints, circleCenterSensor, model);
    if (!finiteVec(out.circleCenterMachine)) return out;
    const auto t = SingleSphere17Model::sensorToMachineTransform(joints, model);
    Vec3d normal(t[1], t[5], t[9]); // R * sensor +Y, row-major transform.
    normal = normalized(normal);
    if (!(norm(normal) > 0.5)) return out;
    out.hMm = std::abs(dot(normal, sub(sphereCenterMachine, out.circleCenterMachine)));
    if (!finite(out.hMm) || out.hMm > sphereRadius + 1e-5) return out;
    out.hMm = std::min(sphereRadius, out.hMm);
    out.gammaDeg = radToDeg(std::asin(clampUnit(out.hMm / sphereRadius)));
    out.theoreticalRadiusMm = std::sqrt(std::max(0.0, sphereRadius*sphereRadius - out.hMm*out.hMm));
    out.valid = finite(out.distanceMm) && finite(out.gammaDeg) && finite(out.etaDeg) &&
                finite(out.theoreticalRadiusMm);
    return out;
}

int branchSignFromCircle(const SphereStudyRawProfile& raw,
                         const CircleExtraction& extraction)
{
    std::vector<double> signs;
    signs.reserve(extraction.inliers.size());
    for (std::size_t idx : extraction.inliers) {
        if (idx >= raw.samples.size()) continue;
        const auto& s = raw.samples[idx];
        if (!s.coordinateValid || !finite(s.zMm)) continue;
        signs.push_back(s.zMm - extraction.fit.centerSensorMm.z);
    }
    const double m = median(signs);
    return (!finite(m) || m >= 0.0) ? +1 : -1;
}

int longestRun(const std::vector<bool>& flags, int& runCount)
{
    int longest = 0, current = 0;
    runCount = 0;
    bool inRun = false;
    for (bool f : flags) {
        if (f) {
            ++current;
            if (!inRun) { ++runCount; inRun = true; }
            longest = std::max(longest, current);
        } else {
            current = 0;
            inRun = false;
        }
    }
    return longest;
}

void copyRich(const SphereStudyRawSample& s, SphereStudyPointMetrics& p)
{
    p.hasWidth = s.hasWidth; p.width = s.width;
    p.hasMaximum = s.hasMaximum; p.maximum = s.maximum;
    p.hasThreshold = s.hasThreshold; p.threshold = s.threshold;
    p.hasMoment0 = s.hasMoment0; p.moment0 = s.moment0;
    p.hasMoment1 = s.hasMoment1; p.moment1 = s.moment1;
}

void appendIf(bool has, double value, std::vector<double>& v)
{
    if (has && finite(value)) v.push_back(value);
}

} // namespace

SphereCloudFitResult SphereDistanceAngleStudy::fitMachineSphereRansac(
    const std::vector<Vec3d>& points, const SphereCloudRansacOptions& options)
{
    SphereCloudFitResult result;
    result.candidatePointCount = points.size();
    if (points.size() < 4 || options.maxIterations <= 0 ||
        !(options.inlierThresholdMm > 0.0) || options.minInliers < 4 ||
        options.minInlierRatio < 0.0 || options.minInlierRatio > 1.0) {
        result.message = "Invalid sphere-cloud RANSAC input/options.";
        return result;
    }

    std::vector<std::size_t> working = makeWorkingIndices(points.size(), options.maxRansacWorkingPoints);
    working.erase(std::remove_if(working.begin(), working.end(), [&](std::size_t i) {
        return !finiteVec(points[i]);
    }), working.end());
    if (working.size() < 4) { result.message = "Too few finite point-cloud samples."; return result; }

    std::mt19937 rng(options.randomSeed ^ static_cast<std::uint32_t>(working.size()));
    std::uniform_int_distribution<std::size_t> pick(0, working.size() - 1);
    Vec3d bestCenter;
    double bestRadius = 0.0;
    Score best;

    for (int it = 0; it < options.maxIterations; ++it) {
        std::array<std::size_t,4> k{};
        bool unique = false;
        for (int retry = 0; retry < 12 && !unique; ++retry) {
            for (auto& v : k) v = pick(rng);
            unique = (k[0] != k[1] && k[0] != k[2] && k[0] != k[3] &&
                      k[1] != k[2] && k[1] != k[3] && k[2] != k[3]);
        }
        if (!unique) continue;
        Vec3d c;
        double r = 0.0;
        if (!sphereFromFourPoints(points[working[k[0]]], points[working[k[1]]],
                                  points[working[k[2]]], points[working[k[3]]], c, r)) continue;
        if (!radiusAllowed(r, options)) continue;
        const Score s = scoreSphere(points, working, c, r, options.inlierThresholdMm);
        if (s.count > best.count || (s.count == best.count && s.rms < best.rms)) {
            best = s; bestCenter = c; bestRadius = r;
        }
    }

    const std::size_t minWork = std::max<std::size_t>(
        static_cast<std::size_t>(options.minInliers),
        static_cast<std::size_t>(std::ceil(options.minInlierRatio * static_cast<double>(working.size()))));
    if (best.count < std::min(minWork, working.size())) {
        std::ostringstream os;
        os << "RANSAC found only " << best.count << "/" << working.size()
           << " working-set inliers.";
        result.message = os.str();
        return result;
    }

    auto inliers = classifySphere(points, bestCenter, bestRadius, options.inlierThresholdMm);
    const std::size_t requiredAll = std::max<std::size_t>(
        static_cast<std::size_t>(options.minInliers),
        static_cast<std::size_t>(std::ceil(options.minInlierRatio * static_cast<double>(points.size()))));
    if (inliers.size() < std::min(requiredAll, points.size())) {
        result.message = "Full-cloud sphere classification retained too few inliers.";
        return result;
    }

    if (!refineSphere(points, inliers, options, bestCenter, bestRadius) || !radiusAllowed(bestRadius, options)) {
        result.message = "Sphere inlier refinement failed or radius left the accepted range.";
        return result;
    }
    inliers = classifySphere(points, bestCenter, bestRadius, options.inlierThresholdMm);
    if (inliers.size() < 4) { result.message = "Sphere refinement lost all inliers."; return result; }

    double sum = 0.0, sumAbs = 0.0, sumSq = 0.0, maxAbs = 0.0;
    for (std::size_t i : inliers) {
        const double e = distance(points[i], bestCenter) - bestRadius;
        sum += e; sumAbs += std::abs(e); sumSq += e*e; maxAbs = std::max(maxAbs, std::abs(e));
    }
    const double n = static_cast<double>(inliers.size());
    result.success = true;
    result.centerMachineMm = bestCenter;
    result.radiusMm = bestRadius;
    result.residualMeanMm = sum / n;
    result.residualMeanAbsMm = sumAbs / n;
    result.residualRmsMm = std::sqrt(sumSq / n);
    result.residualMaxAbsMm = maxAbs;
    result.inlierPointCount = inliers.size();
    result.inlierRatio = static_cast<double>(inliers.size()) / static_cast<double>(points.size());
    result.inlierIndices = std::move(inliers);
    std::ostringstream os;
    os << "OK: center=[" << bestCenter.x << "," << bestCenter.y << "," << bestCenter.z
       << "] mm, R=" << bestRadius << " mm, inliers=" << result.inlierPointCount
       << "/" << points.size() << ", RMS=" << result.residualRmsMm << " mm.";
    result.message = os.str();
    return result;
}

SphereStudyProbeResult SphereDistanceAngleStudy::probeProfileGroup(
    const std::vector<SphereStudyRawProfile>& rawProfiles,
    const Vec3d& predictedSphereCenterSensorMm,
    double sphereRadiusMm, int centerYSign,
    const SphereStudyProfileAnalysisOptions& options,
    int minimumDetectedProfiles)
{
    SphereStudyProbeResult out;
    out.profileCount = static_cast<int>(rawProfiles.size());
    if (!(sphereRadiusMm > 0.0) || rawProfiles.empty() || !finiteVec(predictedSphereCenterSensorMm)) {
        out.message = "Invalid probe inputs.";
        return out;
    }

    std::vector<double> cx, cz, rr, arc;
    cx.reserve(rawProfiles.size()); cz.reserve(rawProfiles.size());
    rr.reserve(rawProfiles.size()); arc.reserve(rawProfiles.size());
    for (const auto& raw : rawProfiles) {
        const auto ex = extractCircle(raw, predictedSphereCenterSensorMm, sphereRadiusMm,
                                      centerYSign, options);
        if (!ex.fit.success || ex.inliers.size() < 5) continue;
        cx.push_back(ex.fit.centerSensorMm.x);
        cz.push_back(ex.fit.centerSensorMm.z);
        rr.push_back(ex.fit.sectionRadiusMm);
        arc.push_back(ex.fit.arcSpanDeg);
    }
    out.detectedCircleCount = static_cast<int>(cx.size());
    out.detectionRatio = rawProfiles.empty() ? 0.0
        : static_cast<double>(out.detectedCircleCount) / static_cast<double>(rawProfiles.size());
    // Do not silently relax the requested Probe quorum on timeout/partial input.
    // If the automatic gate asks for 3 circles out of 5 but only one frame
    // arrives before the watchdog, that is a Probe failure, not a 1/1 success.
    const int required = std::max(1, minimumDetectedProfiles);
    if (out.detectedCircleCount < required) {
        std::ostringstream os;
        os << "Probe Circle-RANSAC detected sphere arc in " << out.detectedCircleCount
           << "/" << out.profileCount << " frames; required " << required << ".";
        out.message = os.str();
        return out;
    }

    out.medianCenterXmm = median(cx);
    out.medianCenterZmm = median(cz);
    out.medianRadiusMm = median(rr);
    out.medianArcSpanDeg = median(arc);
    out.success = finite(out.medianCenterXmm) && finite(out.medianCenterZmm) &&
                  finite(out.medianRadiusMm) && out.medianRadiusMm > 0.0;
    if (out.success) {
        std::ostringstream os;
        os << "OK: probe circle " << out.detectedCircleCount << "/" << out.profileCount
           << ", center=[" << out.medianCenterXmm << "," << out.medianCenterZmm
           << "] mm, r=" << out.medianRadiusMm << " mm, median arc="
           << out.medianArcSpanDeg << " deg.";
        out.message = os.str();
    } else {
        out.message = "Probe circle statistics are non-finite.";
    }
    return out;
}

SphereStudyTargetGeometry SphereDistanceAngleStudy::targetGeometry(
    double sphereRadiusMm, double measurementDistanceMm, double sectionAngleDeg,
    double centerBearingAngleDeg, int sectionSideSign,
    const Vec3d& opticalCenterSensorMm)
{
    SphereStudyTargetGeometry g;
    g.measurementDistanceMm = measurementDistanceMm;
    g.sectionAngleDeg = sectionAngleDeg;
    g.centerBearingAngleDeg = centerBearingAngleDeg;
    g.sectionSideSign = sectionSideSign < 0 ? -1 : +1;
    if (!(sphereRadiusMm > 0.0) || !(measurementDistanceMm > 0.0) ||
        sectionAngleDeg < 0.0 || sectionAngleDeg >= 90.0 ||
        !finite(centerBearingAngleDeg) || std::abs(centerBearingAngleDeg) >= 89.0 ||
        !finiteVec(opticalCenterSensorMm) || std::abs(opticalCenterSensorMm.y) > 1e-6) {
        return g;
    }
    const double gamma = degToRad(sectionAngleDeg);
    const double eta = degToRad(centerBearingAngleDeg);
    const double h = sphereRadiusMm * std::sin(gamma);
    g.theoreticalSectionRadiusMm = sphereRadiusMm * std::cos(gamma);
    // D and eta jointly determine the full circle-center location in the
    // sensor X-Z plane.  eta is measured from +Z toward +X; eta=0 reproduces
    // the older special case in which the circle center lies on the optical
    // +Z axis.
    g.targetCircleCenterSensorMm = Vec3d(
        opticalCenterSensorMm.x + measurementDistanceMm * std::sin(eta),
        0.0,
        opticalCenterSensorMm.z + measurementDistanceMm * std::cos(eta));
    g.targetSphereCenterSensorMm = add(
        g.targetCircleCenterSensorMm,
        Vec3d(0.0, static_cast<double>(g.sectionSideSign) * h, 0.0));
    return g;
}

SphereStudyObservedGeometry SphereDistanceAngleStudy::observeGeometry(
    const JointPose5d& joints, const Vec3d& sphereCenterMachineMm,
    double sphereRadiusMm, const Physical17Parameters& model,
    const Vec3d& opticalCenterSensorMm)
{
    SphereStudyObservedGeometry out;
    if (!(sphereRadiusMm > 0.0) || !finiteVec(opticalCenterSensorMm)) {
        out.message = "Invalid sphere radius or optical center.";
        return out;
    }
    out.sphereCenterSensorMm = SingleSphere17Model::machinePointInSensor(
        joints, sphereCenterMachineMm, model);
    if (!finiteVec(out.sphereCenterSensorMm)) {
        out.message = "Non-finite sphere center in sensor frame.";
        return out;
    }
    out.circleCenterSensorMm = Vec3d(out.sphereCenterSensorMm.x, 0.0, out.sphereCenterSensorMm.z);
    const auto geometry = sectionGeometryFromCircle(
        joints, out.circleCenterSensorMm, sphereCenterMachineMm,
        sphereRadiusMm, model, opticalCenterSensorMm);
    if (!geometry.valid) {
        out.message = "Current profile plane does not form a physically consistent section of the known sphere.";
        return out;
    }
    out.measurementDistanceMm = geometry.distanceMm;
    out.sectionPlaneOffsetMm = geometry.hMm;
    out.sectionAngleDeg = geometry.gammaDeg;
    out.centerBearingAngleDeg = geometry.etaDeg;
    out.theoreticalSectionRadiusMm = geometry.theoreticalRadiusMm;
    out.valid = true;
    out.message = "OK";
    return out;
}

SphereStudyOpticalFeasibility SphereDistanceAngleStudy::simulateOpticalFeasibility(
    const JointPose5d& joints, const Vec3d& sphereCenterMachineMm,
    double sphereRadiusMm, const Physical17Parameters& model,
    const SphereStudyPathOptions& options)
{
    return simulateOpticalFeasibilityImpl(joints, sphereCenterMachineMm,
                                          sphereRadiusMm, model, options);
}

SphereStudyScanSimulation SphereDistanceAngleStudy::buildScanSimulation(
    const JointPose5d& joints, const Vec3d& sphereCenterMachineMm,
    double sphereRadiusMm, const Physical17Parameters& model,
    const SphereStudyPathOptions& options, int circleSamples)
{
    SphereStudyScanSimulation out;
    out.optical = simulateOpticalFeasibilityImpl(
        joints, sphereCenterMachineMm, sphereRadiusMm, model, options);

    const GuardedSensorField field = makeGuardedSensorField(options);
    if (field.valid) {
        out.laserFieldCornersSensorMm.assign(field.corners.begin(), field.corners.end());
    }

    if (!(sphereRadiusMm > 0.0) || !finiteVec(out.optical.sphereCenterSensorMm)) {
        out.message = out.optical.message;
        return out;
    }

    // For visualization retain even a near-tangent geometric section that the
    // feasibility gate rejects.  This makes a "bad" simulated pose visible to
    // the operator instead of silently showing an empty scene.
    const double h = std::abs(out.optical.sphereCenterSensorMm.y);
    if (!(h < sphereRadiusMm) || !field.valid) {
        out.message = out.optical.message;
        return out;
    }

    const Vec3d center(out.optical.sphereCenterSensorMm.x, 0.0,
                       out.optical.sphereCenterSensorMm.z);
    const double radius = std::sqrt(std::max(0.0, sphereRadiusMm*sphereRadiusMm - h*h));
    const Vec3d endSensor = finiteVec(out.optical.endEffectorOriginSensorMm)
        ? out.optical.endEffectorOriginSensorMm : endEffectorOriginSensorMm(model);
    const int samples = std::max(72, circleSamples);
    std::vector<unsigned char> good(static_cast<std::size_t>(samples), 0);
    out.sectionCircleSensorMm.reserve(static_cast<std::size_t>(samples + 1));
    for (int i = 0; i < samples; ++i) {
        const double a = 2.0*kPi*static_cast<double>(i)/static_cast<double>(samples);
        const Vec3d p(center.x + radius*std::cos(a), 0.0,
                      center.z + radius*std::sin(a));
        out.sectionCircleSensorMm.push_back(p);
        if (!field.contains(p)) continue;
        if (endEffectorFacingSphereSide(p, out.optical.sphereCenterSensorMm, endSensor,
                                        options.requireEndEffectorFacingArc))
            good[static_cast<std::size_t>(i)] = 1;
    }
    if (!out.sectionCircleSensorMm.empty())
        out.sectionCircleSensorMm.push_back(out.sectionCircleSensorMm.front());

    int bestLen = 0, bestEnd = -1, run = 0;
    for (int k = 0; k < samples*2; ++k) {
        if (good[static_cast<std::size_t>(k % samples)]) {
            run = std::min(run + 1, samples);
            if (run > bestLen) { bestLen = run; bestEnd = k; }
        } else {
            run = 0;
        }
    }
    if (bestLen > 0 && bestEnd >= 0) {
        const int start = bestEnd - bestLen + 1;
        out.visibleArcSensorMm.reserve(static_cast<std::size_t>(bestLen));
        for (int k = 0; k < bestLen; ++k) {
            const int i = (start + k) % samples;
            const double a = 2.0*kPi*static_cast<double>(i)/static_cast<double>(samples);
            out.visibleArcSensorMm.emplace_back(center.x + radius*std::cos(a), 0.0,
                                                center.z + radius*std::sin(a));
        }
    }

    out.valid = out.optical.valid;
    out.message = out.optical.message;
    return out;
}

bool SphereDistanceAngleStudy::solvePoseForSphereCenterTarget(
    const Vec3d& sphereCenterMachineMm, const Physical17Parameters& model,
    const SphereStudyPathOptions& options, const Vec3d& targetSphereCenterSensorMm,
    double targetCDeg, double targetBDeg, const JointPose5d& seed,
    JointPose5d& solved, Vec3d& predictedSphereCenterSensorMm,
    double& residualMm, std::string& error)
{
    return solveXyzForTarget(sphereCenterMachineMm, model, options,
                             targetSphereCenterSensorMm, targetCDeg, targetBDeg,
                             seed, solved, predictedSphereCenterSensorMm,
                             residualMm, error);
}

std::vector<SphereStudyPose> SphereDistanceAngleStudy::generatePath(
    const Vec3d& sphereCenterMachineMm, const Physical17Parameters& model,
    const SphereStudyPathOptions& options, std::string& error)
{
    error.clear();
    std::vector<SphereStudyPose> result;
    if (!finiteVec(sphereCenterMachineMm) || !(options.sphereRadiusMm > 0.0) ||
        options.measurementDistancesMm.empty() || options.sectionAnglesDeg.empty() ||
        !finite(options.referenceJoint.x) || !finite(options.referenceJoint.y) ||
        !finite(options.referenceJoint.z) || !finite(options.referenceJoint.c) ||
        !finite(options.referenceJoint.b) || !finiteVec(options.opticalCenterSensorMm)) {
        error = "Invalid sphere center, radius, optical center, reference joint, or empty D/gamma grid.";
        return result;
    }
    if (std::abs(options.opticalCenterSensorMm.y) > 1e-6) {
        error = "The configured sensor optical center must lie in the X-Z profile plane (Y=0).";
        return result;
    }
    const std::vector<double> etaValues = options.centerBearingAnglesDeg.empty()
        ? std::vector<double>{0.0} : options.centerBearingAnglesDeg;
    for (double eta : etaValues) {
        if (!finite(eta) || std::abs(eta) >= 89.0) {
            error = "Center-bearing eta must be finite and inside (-89,+89) deg.";
            return result;
        }
    }

    std::vector<SphereStudyRotarySample> rotary = options.rotarySamples;
    if (rotary.empty() && options.automaticPlaneFamily) {
        std::string planeFamilyError;
        rotary = selectAutomaticPlaneFamily(sphereCenterMachineMm, model, options, &planeFamilyError);
        if (rotary.empty() && !planeFamilyError.empty()) {
            error = planeFamilyError;
            return result;
        }
    }
    if (rotary.empty()) {
        SphereStudyRotarySample r;
        r.id = 0; r.cDeg = options.referenceJoint.c; r.bDeg = options.referenceJoint.b;
        r.planeNormalMachine = profilePlaneNormalMachine(r.cDeg, r.bDeg, model);
        rotary.push_back(r);
    }

    std::vector<PlannedCandidate> validCandidates;
    std::vector<SphereStudyPose> legacyRows;
    JointPose5d seed = options.referenceJoint;
    int sequence = 0;
    std::size_t attemptedCandidateCount = 0;
    int opticalRejectedCount = 0;
    int adaptiveAcceptedCount = 0;
    // Deduplicate by the physical rotary orientation + study geometry, not by
    // rotarySample.id.  External callers are allowed to leave sample ids at
    // their default value, so using only id here could accidentally collapse
    // distinct C/B planes during adaptive re-sampling.
    std::set<std::array<long long,5>> attempted;

    auto quantize = [](double v) -> long long {
        return static_cast<long long>(std::llround(v * 1000000.0));
    };
    auto densify = [](std::vector<double> values) {
        std::sort(values.begin(), values.end());
        values.erase(std::unique(values.begin(), values.end(), [](double a, double b) {
            return std::abs(a-b) < 1e-10;
        }), values.end());
        if (values.size() < 2) return values;
        std::vector<double> out;
        out.reserve(values.size()*2-1);
        for (std::size_t i=0; i+1<values.size(); ++i) {
            out.push_back(values[i]);
            out.push_back(0.5*(values[i]+values[i+1]));
        }
        out.push_back(values.back());
        return out;
    };

    auto tryCandidate = [&](const SphereStudyRotarySample& rs,
                            double d, double gamma, double eta,
                            bool adaptive) {
        if (adaptive && attemptedCandidateCount >= options.maxAdaptiveCandidateCount) return;
        const std::array<long long,5> key{{quantize(rs.cDeg),quantize(rs.bDeg),
            quantize(d),quantize(gamma),quantize(eta)}};
        if (!attempted.insert(key).second) return;
        ++attemptedCandidateCount;

        SphereStudyPose pose;
        pose.sequence = sequence++;
        pose.rotarySampleId = rs.id;
        pose.adaptivelyRegenerated = adaptive;
        pose.geometry = targetGeometry(options.sphereRadiusMm, d, gamma, eta,
                                       options.sectionSideSign,
                                       options.opticalCenterSensorMm);
        if (!(pose.geometry.measurementDistanceMm > 0.0) ||
            !finiteVec(pose.geometry.targetSphereCenterSensorMm) ||
            !(pose.geometry.theoreticalSectionRadiusMm > 0.0)) {
            pose.message = "Invalid target D/gamma/eta geometry.";
            if (!options.spaceFillingSampling) legacyRows.push_back(pose);
            return;
        }

        std::string solveError;
        if (!solveXyzForTarget(sphereCenterMachineMm, model, options,
                               pose.geometry.targetSphereCenterSensorMm,
                               rs.cDeg, rs.bDeg, seed,
                               pose.joints, pose.predictedSphereCenterSensorMm,
                               pose.solveResidualMm, solveError)) {
            pose.message = solveError;
            if (!options.spaceFillingSampling) legacyRows.push_back(pose);
            return;
        }

        // V27.6.16AB: every solved pose is forward simulated against the real
        // configured sensor X-Z field.  A no-intersection pose, a near-tangent
        // pose, or an intersection that leaves only a tiny physical arc is not
        // allowed into the autonomous queue.
        const auto optical = simulateOpticalFeasibilityImpl(
            pose.joints, sphereCenterMachineMm, options.sphereRadiusMm,
            model, options);
        pose.predictedCircleCenterSensorMm = optical.circleCenterSensorMm;
        pose.predictedSectionRadiusMm = optical.sectionRadiusMm;
        pose.predictedVisibleArcDeg = optical.visibleArcDeg;
        pose.predictedVisibleArcLengthMm = optical.visibleArcLengthMm;
        pose.predictedPlaneIntersectionMarginMm = optical.planeIntersectionMarginMm;
        if (!optical.valid) {
            ++opticalRejectedCount;
            pose.message = optical.message;
            if (!options.spaceFillingSampling) legacyRows.push_back(pose);
            return;
        }

        pose.valid = true;
        std::ostringstream ok;
        ok << (adaptive ? "OK; adaptive replacement; " : "OK; ")
           << "predicted visible sphere arc=" << optical.visibleArcDeg
           << " deg / " << optical.visibleArcLengthMm
           << " mm, plane-intersection margin="
           << optical.planeIntersectionMarginMm << " mm.";
        pose.message = ok.str();
        seed = pose.joints;
        PlannedCandidate c;
        c.pose = pose;
        c.planeNormalMachine = rs.planeNormalMachine;
        if (!(norm(c.planeNormalMachine)>0.5))
            c.planeNormalMachine = profilePlaneNormalMachine(rs.cDeg, rs.bDeg, model);
        validCandidates.push_back(c);
        if (adaptive) ++adaptiveAcceptedCount;
        if (!options.spaceFillingSampling) legacyRows.push_back(pose);
    };

    auto enumerateGrid = [&](const std::vector<double>& dValues,
                             const std::vector<double>& gammaValues,
                             const std::vector<double>& eValues,
                             bool adaptive) {
        for (std::size_t ri = 0; ri < rotary.size(); ++ri) {
            if (adaptive && attemptedCandidateCount >= options.maxAdaptiveCandidateCount) return;
            const auto& rs = rotary[ri];
            if (!inRotaryLimits(rs.cDeg, rs.bDeg, options)) continue;
            for (std::size_t di = 0; di < dValues.size(); ++di) {
                if (adaptive && attemptedCandidateCount >= options.maxAdaptiveCandidateCount) return;
                std::vector<double> angles = gammaValues;
                if (!options.spaceFillingSampling && options.serpentine && ((di + ri) % 2u) == 1u)
                    std::reverse(angles.begin(), angles.end());
                for (double gamma : angles) {
                    for (double eta : eValues) {
                        tryCandidate(rs, dValues[di], gamma, eta, adaptive);
                        if (adaptive && attemptedCandidateCount >= options.maxAdaptiveCandidateCount) return;
                    }
                }
            }
        }
    };

    enumerateGrid(options.measurementDistancesMm, options.sectionAnglesDeg, etaValues, false);

    // If optical/FOV gating discarded too many original grid points, create
    // replacement poses by inserting mid-points inside the user's declared
    // D/gamma/eta ranges.  The replacements are not trusted merely because
    // they were generated: they must pass the exact same forward simulation.
    if (options.spaceFillingSampling && options.adaptiveOpticalResampling &&
        validCandidates.size() < static_cast<std::size_t>(std::max(1, options.targetPoseCount))) {
        std::vector<double> dDense = options.measurementDistancesMm;
        std::vector<double> gDense = options.sectionAnglesDeg;
        std::vector<double> eDense = etaValues;
        const int passes = std::max(0, options.adaptiveOpticalResamplingPasses);
        for (int pass=0; pass<passes; ++pass) {
            dDense = densify(std::move(dDense));
            gDense = densify(std::move(gDense));
            eDense = densify(std::move(eDense));
            enumerateGrid(dDense, gDense, eDense, true);
            if (validCandidates.size() >= static_cast<std::size_t>(std::max(1, options.targetPoseCount)) ||
                attemptedCandidateCount >= options.maxAdaptiveCandidateCount) break;
        }
    }

    if (options.spaceFillingSampling) {
        result = selectSpaceFillingPath(validCandidates, options);
        if (result.empty()) {
            std::ostringstream os;
            os << "No D/gamma/eta/plane-family candidate survived planning. attempted="
               << attemptedCandidateCount << ", optical_rejected=" << opticalRejectedCount
               << ", adaptive_accepted=" << adaptiveAcceptedCount << ".";
            error = os.str();
            return result;
        }
        if (static_cast<int>(result.size()) < std::max(1, options.targetPoseCount)) {
            std::ostringstream os;
            os << "Only " << result.size() << " feasible space-filling poses were available from "
               << validCandidates.size() << " valid candidates; requested "
               << std::max(1, options.targetPoseCount) << ". attempted="
               << attemptedCandidateCount << ", optical_rejected=" << opticalRejectedCount
               << ", adaptive_accepted=" << adaptiveAcceptedCount << ".";
            error = os.str();
        }
        return result;
    }

    result = std::move(legacyRows);
    const auto validCount = std::count_if(result.begin(), result.end(),
        [](const SphereStudyPose& p){ return p.valid; });
    if (validCount == 0)
        error = "No D/gamma/eta/plane-family target could be solved inside configured limits.";
    return result;
}

SphereStudyGroupAnalysis SphereDistanceAngleStudy::analyzeProfileGroup(
    const JointPose5d& actualJoints,
    const std::vector<SphereStudyRawProfile>& rawProfiles,
    const Vec3d& sphereCenterMachineMm,
    double sphereRadiusMm,
    const Physical17Parameters& model,
    const Vec3d& opticalCenterSensorMm,
    const SphereStudyProfileAnalysisOptions& options)
{
    SphereStudyGroupAnalysis group;
    if (!(sphereRadiusMm > 0.0) || rawProfiles.empty() || !finiteVec(opticalCenterSensorMm)) {
        group.message = "Invalid sphere radius, optical center or empty profile group.";
        return group;
    }
    const Vec3d predictedSphereCenter = SingleSphere17Model::machinePointInSensor(
        actualJoints, sphereCenterMachineMm, model);
    if (!finiteVec(predictedSphereCenter)) {
        group.message = "Physical17 produced a non-finite sensor-frame sphere center.";
        return group;
    }
    const int centerYSign = predictedSphereCenter.y < 0.0 ? -1 : +1;

    std::vector<CircleExtraction> extracted(rawProfiles.size());
    std::vector<double> centersX, centersZ, radii, thresholds;
    std::vector<int> starts, ends;
    std::vector<int> branchVotes;
    std::size_t maxSamples = 0;
    for (std::size_t i = 0; i < rawProfiles.size(); ++i) {
        maxSamples = std::max(maxSamples, rawProfiles[i].samples.size());
        extracted[i] = extractCircle(rawProfiles[i], predictedSphereCenter,
                                     sphereRadiusMm, centerYSign, options);
        if (!extracted[i].fit.success || extracted[i].inliers.empty()) continue;
        centersX.push_back(extracted[i].fit.centerSensorMm.x);
        centersZ.push_back(extracted[i].fit.centerSensorMm.z);
        radii.push_back(extracted[i].fit.sectionRadiusMm);
        thresholds.push_back(extracted[i].thresholdMm);
        std::size_t lo = *std::min_element(extracted[i].inliers.begin(), extracted[i].inliers.end());
        std::size_t hi = *std::max_element(extracted[i].inliers.begin(), extracted[i].inliers.end());
        starts.push_back(static_cast<int>(lo));
        ends.push_back(static_cast<int>(hi));
        branchVotes.push_back(branchSignFromCircle(rawProfiles[i], extracted[i]));
    }
    const std::size_t required = std::max<std::size_t>(1,
        std::min<std::size_t>(rawProfiles.size(), static_cast<std::size_t>(std::max(1, options.minimumConsensusProfiles))));
    if (centersX.size() < required) {
        std::ostringstream os;
        os << "Only " << centersX.size() << "/" << rawProfiles.size()
           << " profiles produced a circle; need " << required << " for group ROI consensus.";
        group.message = os.str();
        return group;
    }
    group.consensusCenterXmm = median(centersX);
    group.consensusCenterZmm = median(centersZ);
    group.consensusRadiusMm = median(radii);
    const double groupThreshold = std::max(options.adaptiveMinThresholdMm, median(thresholds));
    const int branchSum = std::accumulate(branchVotes.begin(), branchVotes.end(), 0);
    group.visibleArcBranchSign = branchSum < 0 ? -1 : +1;

    const Vec3d consensusCircle(group.consensusCenterXmm, 0.0, group.consensusCenterZmm);
    const auto section = sectionGeometryFromCircle(actualJoints, consensusCircle,
                                                    sphereCenterMachineMm, sphereRadiusMm,
                                                    model, opticalCenterSensorMm);
    if (!section.valid || !(section.theoreticalRadiusMm > 0.0)) {
        group.message = "Known sphere / profile-plane geometry is inconsistent at the captured pose.";
        return group;
    }

    // Median X by original sample index.  X normally remains defined even when Z
    // is invalid, which lets a dropout keep its theoretical local angle.
    std::vector<std::vector<double>> xByIndex(maxSamples);
    for (const auto& raw : rawProfiles) {
        for (std::size_t i = 0; i < raw.samples.size(); ++i) {
            if (finite(raw.samples[i].xMm)) xByIndex[i].push_back(raw.samples[i].xMm);
        }
    }
    std::vector<double> referenceX(maxSamples, std::numeric_limits<double>::quiet_NaN());
    for (std::size_t i = 0; i < maxSamples; ++i) referenceX[i] = median(xByIndex[i]);

    int start = medianInt(starts);
    int end = medianInt(ends);
    if (start < 0 || end < start || end >= static_cast<int>(maxSamples)) {
        group.message = "RANSAC inlier support could not define a stable evaluation ROI.";
        return group;
    }
    const double allowedDx = section.theoreticalRadiusMm + std::max(0.0, options.evaluationEdgeMarginMm);
    while (start > 0) {
        const double x = referenceX[static_cast<std::size_t>(start - 1)];
        if (!finite(x) || std::abs(x - group.consensusCenterXmm) > allowedDx) break;
        --start;
    }
    while (end + 1 < static_cast<int>(maxSamples)) {
        const double x = referenceX[static_cast<std::size_t>(end + 1)];
        if (!finite(x) || std::abs(x - group.consensusCenterXmm) > allowedDx) break;
        ++end;
    }
    group.evaluationStartIndex = start;
    group.evaluationEndIndex = end;

    group.profiles.reserve(rawProfiles.size());
    for (std::size_t pi = 0; pi < rawProfiles.size(); ++pi) {
        const auto& raw = rawProfiles[pi];
        const auto& ex = extracted[pi];
        SphereStudyProfileMetrics m;
        m.callbackOrdinal = raw.callbackOrdinal;
        m.profileIndex = raw.profileIndex;
        m.evaluationStartIndex = start;
        m.evaluationEndIndex = end;
        m.evaluationPointCount = static_cast<std::size_t>(end - start + 1);
        m.circleDetected = ex.fit.success;
        m.adaptiveInlierThresholdMm = ex.fit.success ? ex.thresholdMm : groupThreshold;

        const double cx = ex.fit.success ? ex.fit.centerSensorMm.x : group.consensusCenterXmm;
        const double cz = ex.fit.success ? ex.fit.centerSensorMm.z : group.consensusCenterZmm;
        const double rFit = ex.fit.success ? ex.fit.sectionRadiusMm : group.consensusRadiusMm;
        m.fittedCircleCenterSensorMm = Vec3d(cx, 0.0, cz);
        m.fittedRadiusMm = rFit;
        m.fittedCircleCenterMachineMm = SingleSphere17Model::mapSensorPoint(
            actualJoints, m.fittedCircleCenterSensorMm, model);
        const auto geom = sectionGeometryFromCircle(actualJoints, m.fittedCircleCenterSensorMm,
                                                    sphereCenterMachineMm, sphereRadiusMm,
                                                    model, opticalCenterSensorMm);
        if (!geom.valid || !(geom.theoreticalRadiusMm > 0.0)) {
            m.message = "Profile section geometry is inconsistent with the known sphere.";
            group.profiles.push_back(std::move(m));
            continue;
        }
        m.measurementDistanceMm = geom.distanceMm;
        m.sectionPlaneOffsetMm = geom.hMm;
        m.sectionAngleDeg = geom.gammaDeg;
        m.centerBearingAngleDeg = geom.etaDeg;
        m.theoreticalRadiusMm = geom.theoreticalRadiusMm;
        m.radiusBiasMm = rFit - geom.theoreticalRadiusMm;
        m.arcSpanDeg = ex.fit.success ? ex.fit.arcSpanDeg : 0.0;

        int branch = group.visibleArcBranchSign;
        if (ex.fit.success) branch = branchSignFromCircle(raw, ex);
        const Vec3d sphereSensor = SingleSphere17Model::machinePointInSensor(
            actualJoints, sphereCenterMachineMm, model);
        const int sideSign = sphereSensor.y < 0.0 ? -1 : +1;
        const Vec3d localSphereCenter(cx, static_cast<double>(sideSign) * geom.hMm, cz);

        std::vector<double> shapeGood, absAll;
        std::vector<double> maxAll, maxGood, widthAll, widthGood, m0All, m0Good,
                            m1All, m1Good, thAll, thGood;
        std::vector<bool> dropoutFlags, outlierFlags;
        m.points.reserve(m.evaluationPointCount);

        for (int idx = start; idx <= end; ++idx) {
            SphereStudyPointMetrics pm;
            pm.sampleIndex = static_cast<std::size_t>(idx);
            pm.inEvaluationRoi = true;
            const SphereStudyRawSample* rs = (idx >= 0 && idx < static_cast<int>(raw.samples.size()))
                                           ? &raw.samples[static_cast<std::size_t>(idx)] : nullptr;
            if (rs) {
                pm.coordinateValid = rs->coordinateValid && finite(rs->xMm) && finite(rs->zMm);
                pm.measuredXmm = rs->xMm;
                pm.measuredZmm = rs->zMm;
                copyRich(*rs, pm);
            }
            pm.dropout = !pm.coordinateValid;
            if (pm.coordinateValid) ++m.coordinateValidCount;
            else ++m.dropoutCount;

            double expectedX = rs && finite(rs->xMm) ? rs->xMm
                : (idx >= 0 && idx < static_cast<int>(referenceX.size())
                   ? referenceX[static_cast<std::size_t>(idx)]
                   : std::numeric_limits<double>::quiet_NaN());
            pm.expectedXmm = expectedX;
            if (finite(expectedX)) {
                const double dx = expectedX - cx;
                const double rr = geom.theoreticalRadiusMm * geom.theoreticalRadiusMm - dx*dx;
                if (rr >= 0.0) {
                    pm.expectedZmm = cz + static_cast<double>(branch) * std::sqrt(std::max(0.0, rr));
                    const Vec3d expectedPoint(expectedX, 0.0, pm.expectedZmm);
                    const Vec3d n = normalized(sub(expectedPoint, localSphereCenter));
                    const Vec3d view = normalized(sub(opticalCenterSensorMm, expectedPoint));
                    if (norm(n) > 0.5 && norm(view) > 0.5) {
                        pm.localSurfaceAngleDeg = radToDeg(std::acos(clampUnit(std::abs(dot(n, view)))));
                    }
                }
            }

            if (pm.coordinateValid) {
                const double rho = std::hypot(pm.measuredXmm - cx, pm.measuredZmm - cz);
                pm.shapeResidualMm = rho - rFit;
                pm.absoluteSectionResidualMm = rho - geom.theoreticalRadiusMm;
                pm.geometricGood = std::abs(pm.shapeResidualMm) <= m.adaptiveInlierThresholdMm;
                pm.geometricOutlier = !pm.geometricGood;
                if (pm.geometricGood) {
                    ++m.goodPointCount;
                    shapeGood.push_back(pm.shapeResidualMm);
                } else {
                    ++m.geometricOutlierCount;
                }
                absAll.push_back(pm.absoluteSectionResidualMm);
                appendIf(pm.hasMaximum, pm.maximum, maxAll);
                appendIf(pm.hasWidth, pm.width, widthAll);
                appendIf(pm.hasMoment0, pm.moment0, m0All);
                appendIf(pm.hasMoment1, pm.moment1, m1All);
                appendIf(pm.hasThreshold, pm.threshold, thAll);
                if (pm.geometricGood) {
                    appendIf(pm.hasMaximum, pm.maximum, maxGood);
                    appendIf(pm.hasWidth, pm.width, widthGood);
                    appendIf(pm.hasMoment0, pm.moment0, m0Good);
                    appendIf(pm.hasMoment1, pm.moment1, m1Good);
                    appendIf(pm.hasThreshold, pm.threshold, thGood);
                }
            }
            dropoutFlags.push_back(pm.dropout);
            outlierFlags.push_back(pm.geometricOutlier);
            m.points.push_back(std::move(pm));
        }

        if (m.evaluationPointCount > 0) {
            const double denom = static_cast<double>(m.evaluationPointCount);
            m.goodPointRatio = static_cast<double>(m.goodPointCount) / denom;
            m.geometricOutlierRatio = static_cast<double>(m.geometricOutlierCount) / denom;
            m.dropoutRatio = static_cast<double>(m.dropoutCount) / denom;
        }
        m.maxDropoutRun = longestRun(dropoutFlags, m.dropoutRunCount);
        int outlierRuns = 0;
        m.maxOutlierRun = longestRun(outlierFlags, outlierRuns);

        if (!shapeGood.empty()) {
            double ss = 0.0, maxAbs = 0.0;
            for (double e : shapeGood) { ss += e*e; maxAbs = std::max(maxAbs, std::abs(e)); }
            m.circleRmsSensorMm = std::sqrt(ss / static_cast<double>(shapeGood.size()));
            std::vector<double> absShape; absShape.reserve(shapeGood.size());
            for (double e : shapeGood) absShape.push_back(std::abs(e));
            std::sort(absShape.begin(), absShape.end());
            m.circleMadSensorMm = robustStats(shapeGood).mad;
            m.shapeP95Mm = quantileSorted(absShape, 0.95);
            m.shapeMaxAbsMm = maxAbs;
        }
        if (!absAll.empty()) {
            double sumAbs = 0.0, ss = 0.0, maxAbs = 0.0;
            std::vector<double> aa; aa.reserve(absAll.size());
            for (double e : absAll) {
                const double a = std::abs(e); sumAbs += a; ss += e*e; maxAbs = std::max(maxAbs, a); aa.push_back(a);
            }
            std::sort(aa.begin(), aa.end());
            m.absoluteSectionMeanAbsMm = sumAbs / static_cast<double>(absAll.size());
            m.absoluteSectionRmsMm = std::sqrt(ss / static_cast<double>(absAll.size()));
            m.absoluteSectionP95Mm = quantileSorted(aa, 0.95);
            m.absoluteSectionMaxAbsMm = maxAbs;
        }
        m.maximumAllValid = robustStats(maxAll); m.maximumGood = robustStats(maxGood);
        m.widthAllValid = robustStats(widthAll); m.widthGood = robustStats(widthGood);
        m.moment0AllValid = robustStats(m0All); m.moment0Good = robustStats(m0Good);
        m.moment1AllValid = robustStats(m1All); m.moment1Good = robustStats(m1Good);
        m.thresholdAllValid = robustStats(thAll); m.thresholdGood = robustStats(thGood);
        m.valid = m.evaluationPointCount > 0 && finite(m.measurementDistanceMm) &&
                  finite(m.sectionAngleDeg) && finite(m.centerBearingAngleDeg);
        if (m.circleDetected) m.message = "OK";
        else m.message = "Circle not independently detected in this frame; group consensus retained it as a quality-failure sample.";
        group.profiles.push_back(std::move(m));
    }

    group.valid = !group.profiles.empty();
    group.message = group.valid ? "OK" : "No profile metrics could be produced.";
    return group;
}

SphereStudyProfileMetrics SphereDistanceAngleStudy::analyzeProfile(
    const JointPose5d& actualJoints,
    const std::vector<SensorProfilePoint>& rawProfilePoints,
    const Vec3d& sphereCenterMachineMm,
    double sphereRadiusMm,
    const Physical17Parameters& model,
    const SphereArcRansacOptions& arcOptions,
    double roiMarginMm)
{
    SphereStudyRawProfile raw;
    raw.samples.reserve(rawProfilePoints.size());
    for (std::size_t i = 0; i < rawProfilePoints.size(); ++i) {
        SphereStudyRawSample s;
        s.index = i;
        s.xMm = rawProfilePoints[i].x;
        s.zMm = rawProfilePoints[i].z;
        s.coordinateValid = finite(s.xMm) && finite(s.zMm) && rawProfilePoints[i].weight > 0.0;
        raw.samples.push_back(s);
    }
    SphereStudyProfileAnalysisOptions o;
    o.ransac = arcOptions;
    o.predictedSearchMarginMm = roiMarginMm;
    o.minimumConsensusProfiles = 1;
    const auto group = analyzeProfileGroup(actualJoints, {raw}, sphereCenterMachineMm,
                                           sphereRadiusMm, model, Vec3d(0.0,0.0,0.0), o);
    if (!group.profiles.empty()) return group.profiles.front();
    SphereStudyProfileMetrics out;
    out.message = group.message;
    return out;
}

SphereStudyQualityPrediction SphereDistanceAngleStudy::predictQualityKernel(
    const std::vector<SphereStudyQualityObservation>& observations,
    double queryDistanceMm, double querySectionAngleDeg,
    double queryCenterBearingAngleDeg,
    double distanceBandwidthMm, double sectionAngleBandwidthDeg,
    double centerBearingBandwidthDeg)
{
    SphereStudyQualityPrediction out;
    out.queryDistanceMm = queryDistanceMm;
    out.querySectionAngleDeg = querySectionAngleDeg;
    out.queryCenterBearingAngleDeg = queryCenterBearingAngleDeg;
    if (observations.empty() || !(distanceBandwidthMm > 0.0) ||
        !(sectionAngleBandwidthDeg > 0.0) || !(centerBearingBandwidthDeg > 0.0) ||
        !finite(queryDistanceMm) || !finite(querySectionAngleDeg) ||
        !finite(queryCenterBearingAngleDeg)) return out;

    double sw=0.0, sw2=0.0, p95=0.0, bias=0.0, drop=0.0, outlier=0.0, good=0.0;
    double nearest = std::numeric_limits<double>::infinity();
    struct WeightedValue { double w; double y; };
    std::vector<WeightedValue> p95Values;
    for (const auto& o : observations) {
        if (!finite(o.measurementDistanceMm) || !finite(o.sectionAngleDeg) ||
            !finite(o.centerBearingAngleDeg) || !finite(o.absoluteP95Mm) ||
            !finite(o.radiusBiasMm) || !finite(o.dropoutRatio) ||
            !finite(o.geometricOutlierRatio) || !finite(o.goodPointRatio)) continue;
        const double nd = (o.measurementDistanceMm - queryDistanceMm) / distanceBandwidthMm;
        const double ng = (o.sectionAngleDeg - querySectionAngleDeg) / sectionAngleBandwidthDeg;
        const double ne = (o.centerBearingAngleDeg - queryCenterBearingAngleDeg) / centerBearingBandwidthDeg;
        const double d2 = nd*nd + ng*ng + ne*ne;
        nearest = std::min(nearest, std::sqrt(d2));
        if (d2 > 36.0) continue;
        const double w = std::exp(-0.5*d2);
        sw += w; sw2 += w*w;
        p95 += w*o.absoluteP95Mm;
        bias += w*o.radiusBiasMm;
        drop += w*o.dropoutRatio;
        outlier += w*o.geometricOutlierRatio;
        good += w*o.goodPointRatio;
        p95Values.push_back({w,o.absoluteP95Mm});
    }
    if (!(sw > 1e-12)) return out;
    out.predictedAbsoluteP95Mm = p95/sw;
    out.predictedRadiusBiasMm = bias/sw;
    out.predictedDropoutRatio = drop/sw;
    out.predictedOutlierRatio = outlier/sw;
    out.predictedGoodPointRatio = good/sw;
    double var = 0.0;
    for (const auto& v : p95Values) {
        const double e = v.y - out.predictedAbsoluteP95Mm;
        var += v.w*e*e;
    }
    out.localSpreadMm = std::sqrt(std::max(0.0, var/sw));
    out.effectiveSampleCount = sw2 > 0.0 ? (sw*sw/sw2) : 0.0;
    out.nearestNormalizedDistance = finite(nearest) ? nearest : 1e9;
    out.lowSupport = out.effectiveSampleCount < 3.0 || out.nearestNormalizedDistance > 2.0;
    out.valid = true;
    return out;
}

} // namespace calibration
} // namespace mls
