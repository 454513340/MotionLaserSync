#include "calibration/SingleSphere17Calibration.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <limits>
#include <map>
#include <numeric>
#include <queue>
#include <random>
#include <cstdint>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace mls {
namespace calibration {
namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kControllerCSign = -1.0;
constexpr double kControllerBSign = -1.0;
constexpr std::size_t kTargetCount = Physical17Parameters::Count;
constexpr std::size_t kStateCount = Physical17Parameters::Count + 3;
constexpr std::size_t kIdentifiableCount = 14;
constexpr std::size_t kObservableStateCount = kIdentifiableCount + 3;

struct Mat3d
{
    double v[3][3]{};
};

struct AxisLine
{
    Vec3d point;
    Vec3d direction;
    Vec3d basisU;
    Vec3d basisV;
};

struct DenseMatrix
{
    int rows = 0;
    int cols = 0;
    std::vector<double> data;

    DenseMatrix() = default;
    DenseMatrix(int r, int c, double initial = 0.0)
        : rows(r), cols(c), data(static_cast<std::size_t>(r * c), initial)
    {
    }

    double& operator()(int r, int c)
    {
        return data[static_cast<std::size_t>(r * cols + c)];
    }

    double operator()(int r, int c) const
    {
        return data[static_cast<std::size_t>(r * cols + c)];
    }
};

struct CandidatePose
{
    JointPose5d joint;
    Vec3d targetCenterSensor;
    SphereProfileObservation syntheticObservation;
    DenseMatrix information;
    double informationGain = 0.0;
};

struct LightweightCandidate
{
    JointPose5d joint;
    Vec3d targetCenterSensor;
    std::uint64_t priority = 0;
    std::size_t ordinal = 0;
    bool startAnchor = false;
};

std::uint64_t stableMix64(std::uint64_t x)
{
    // SplitMix64 finalizer: deterministic, fast and independent of STL hash
    // implementation details.  Dense Stage-2 shortlisting uses this only to
    // obtain an unbiased, reproducible sample of the Cartesian candidate grid.
    x += 0x9e3779b97f4a7c15ULL;
    x = (x ^ (x >> 30U)) * 0xbf58476d1ce4e5b9ULL;
    x = (x ^ (x >> 27U)) * 0x94d049bb133111ebULL;
    return x ^ (x >> 31U);
}

Vec3d add(const Vec3d& a, const Vec3d& b)
{
    return Vec3d(a.x + b.x, a.y + b.y, a.z + b.z);
}

Vec3d sub(const Vec3d& a, const Vec3d& b)
{
    return Vec3d(a.x - b.x, a.y - b.y, a.z - b.z);
}

Vec3d mul(const Vec3d& a, double s)
{
    return Vec3d(a.x * s, a.y * s, a.z * s);
}

double dot(const Vec3d& a, const Vec3d& b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vec3d cross(const Vec3d& a, const Vec3d& b)
{
    return Vec3d(a.y * b.z - a.z * b.y,
                 a.z * b.x - a.x * b.z,
                 a.x * b.y - a.y * b.x);
}

double norm(const Vec3d& a)
{
    return std::sqrt(dot(a, a));
}

Vec3d normalized(const Vec3d& a)
{
    const double n = norm(a);
    if (n < 1e-15) {
        return Vec3d();
    }
    return mul(a, 1.0 / n);
}

bool finiteVec(const Vec3d& a)
{
    return std::isfinite(a.x) && std::isfinite(a.y) && std::isfinite(a.z);
}

Mat3d identity3()
{
    Mat3d m;
    m.v[0][0] = 1.0;
    m.v[1][1] = 1.0;
    m.v[2][2] = 1.0;
    return m;
}

Mat3d transpose(const Mat3d& a)
{
    Mat3d out;
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            out.v[r][c] = a.v[c][r];
        }
    }
    return out;
}

Mat3d multiply(const Mat3d& a, const Mat3d& b)
{
    Mat3d out;
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            double sum = 0.0;
            for (int k = 0; k < 3; ++k) {
                sum += a.v[r][k] * b.v[k][c];
            }
            out.v[r][c] = sum;
        }
    }
    return out;
}

Vec3d multiply(const Mat3d& a, const Vec3d& p)
{
    return Vec3d(
        a.v[0][0] * p.x + a.v[0][1] * p.y + a.v[0][2] * p.z,
        a.v[1][0] * p.x + a.v[1][1] * p.y + a.v[1][2] * p.z,
        a.v[2][0] * p.x + a.v[2][1] * p.y + a.v[2][2] * p.z);
}

Mat3d skew(const Vec3d& v)
{
    Mat3d out;
    out.v[0][1] = -v.z;
    out.v[0][2] = +v.y;
    out.v[1][0] = +v.z;
    out.v[1][2] = -v.x;
    out.v[2][0] = -v.y;
    out.v[2][1] = +v.x;
    return out;
}

Mat3d addMatrix(const Mat3d& a, const Mat3d& b)
{
    Mat3d out;
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            out.v[r][c] = a.v[r][c] + b.v[r][c];
        }
    }
    return out;
}

Mat3d scaleMatrix(const Mat3d& a, double s)
{
    Mat3d out;
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            out.v[r][c] = a.v[r][c] * s;
        }
    }
    return out;
}

Mat3d rotationFromVector(const Vec3d& rotationVector)
{
    const double angle = norm(rotationVector);
    const Mat3d I = identity3();
    if (angle < 1e-14) {
        return addMatrix(I, skew(rotationVector));
    }

    const Vec3d axis = mul(rotationVector, 1.0 / angle);
    const Mat3d K = skew(axis);
    const Mat3d K2 = multiply(K, K);
    return addMatrix(addMatrix(I, scaleMatrix(K, std::sin(angle))),
                     scaleMatrix(K2, 1.0 - std::cos(angle)));
}

Vec3d rotateAroundLine(const Vec3d& p,
                       const Vec3d& linePoint,
                       const Vec3d& lineDirection,
                       double angleRad)
{
    const Mat3d r = rotationFromVector(mul(lineDirection, angleRad));
    return add(linePoint, multiply(r, sub(p, linePoint)));
}

void linearAxisDirections(const Physical17Parameters& p,
                          Vec3d& ux,
                          Vec3d& uy,
                          Vec3d& uz)
{
    ux = Vec3d(1.0, 0.0, 0.0);
    uy = normalized(Vec3d(std::tan(p[Physical17Parameters::Sxy]), 1.0, 0.0));
    uz = normalized(Vec3d(std::tan(p[Physical17Parameters::Sxz]),
                          std::tan(p[Physical17Parameters::Syz]),
                          1.0));
}

AxisLine cAxisLine(const Physical17Parameters& p)
{
    AxisLine line;
    const Mat3d tilt = rotationFromVector(
        Vec3d(p[Physical17Parameters::CTiltX],
              p[Physical17Parameters::CTiltY],
              0.0));
    line.direction = normalized(multiply(tilt, Vec3d(0.0, 0.0, 1.0)));

    Vec3d u = sub(Vec3d(1.0, 0.0, 0.0),
                  mul(line.direction, dot(line.direction, Vec3d(1.0, 0.0, 0.0))));
    if (norm(u) < 1e-10) {
        u = sub(Vec3d(0.0, 1.0, 0.0),
                mul(line.direction, dot(line.direction, Vec3d(0.0, 1.0, 0.0))));
    }
    line.basisU = normalized(u);
    line.basisV = normalized(cross(line.direction, line.basisU));
    line.point = add(mul(line.basisU, p[Physical17Parameters::COffsetU]),
                     mul(line.basisV, p[Physical17Parameters::COffsetV]));
    return line;
}

AxisLine bAxisLine(const Physical17Parameters& p)
{
    AxisLine line;
    const Mat3d tilt = rotationFromVector(
        Vec3d(p[Physical17Parameters::BTiltX],
              0.0,
              p[Physical17Parameters::BTiltZ]));
    line.direction = normalized(multiply(tilt, Vec3d(0.0, 1.0, 0.0)));

    Vec3d u = sub(Vec3d(1.0, 0.0, 0.0),
                  mul(line.direction, dot(line.direction, Vec3d(1.0, 0.0, 0.0))));
    if (norm(u) < 1e-10) {
        u = sub(Vec3d(0.0, 0.0, 1.0),
                mul(line.direction, dot(line.direction, Vec3d(0.0, 0.0, 1.0))));
    }
    line.basisU = normalized(u);
    line.basisV = normalized(cross(line.basisU, line.direction));
    line.point = add(mul(line.basisU, p[Physical17Parameters::BOffsetU]),
                     mul(line.basisV, p[Physical17Parameters::BOffsetV]));
    return line;
}

Vec3d handEyeRotate(const Physical17Parameters& p, const Vec3d& sensorPoint)
{
    const Mat3d r = rotationFromVector(
        Vec3d(p[Physical17Parameters::HandEyeRx],
              p[Physical17Parameters::HandEyeRy],
              p[Physical17Parameters::HandEyeRz]));
    return multiply(r, sensorPoint);
}

std::array<double, kStateCount> packState(const Physical17Parameters& p,
                                           const Vec3d& sphereCenter)
{
    std::array<double, kStateCount> state{};
    for (std::size_t i = 0; i < kTargetCount; ++i) {
        state[i] = p[i];
    }
    state[kTargetCount + 0] = sphereCenter.x;
    state[kTargetCount + 1] = sphereCenter.y;
    state[kTargetCount + 2] = sphereCenter.z;
    return state;
}

void unpackState(const std::array<double, kStateCount>& state,
                 Physical17Parameters& p,
                 Vec3d& sphereCenter)
{
    for (std::size_t i = 0; i < kTargetCount; ++i) {
        p[i] = state[i];
    }
    sphereCenter = Vec3d(state[kTargetCount + 0],
                         state[kTargetCount + 1],
                         state[kTargetCount + 2]);
}

// A profile has one rigid sensor-to-machine transform.  Build it once, not
// once for every XZ point.  The linear-axis nonorthogonality is a translation
// for a fixed pose and therefore does not invalidate distance invariance.
struct PreparedPhysical17
{
    Vec3d ux, uy, uz;
    AxisLine cLine, bLine;
    Mat3d handEyeRotation;
    Vec3d handEyeTranslation;

    explicit PreparedPhysical17(const Physical17Parameters& p)
        : cLine(cAxisLine(p)), bLine(bAxisLine(p)),
          handEyeRotation(rotationFromVector(Vec3d(p[11], p[12], p[13]))),
          handEyeTranslation(p[14], p[15], p[16])
    {
        linearAxisDirections(p, ux, uy, uz);
    }
};

struct ProfileTransform
{
    Mat3d rotation;
    Vec3d translation;
};

ProfileTransform profileTransform(const PreparedPhysical17& p, const JointPose5d& q)
{
    const Mat3d rb = rotationFromVector(mul(p.bLine.direction, kControllerBSign*q.b*kDegToRad));
    const Mat3d rc = rotationFromVector(mul(p.cLine.direction, kControllerCSign*q.c*kDegToRad));
    ProfileTransform t;
    t.rotation = multiply(multiply(rc, rb), p.handEyeRotation);
    const Vec3d bOrigin = add(p.bLine.point, multiply(rb, sub(p.handEyeTranslation, p.bLine.point)));
    t.translation = add(add(add(mul(p.ux, q.x), mul(p.uy, q.y)), mul(p.uz, q.z)),
                        add(p.cLine.point, multiply(rc, sub(bOrigin, p.cLine.point))));
    return t;
}

Vec3d centerInSensor(const ProfileTransform& t, const Vec3d& center)
{
    return multiply(transpose(t.rotation), sub(center, t.translation));
}

bool finitePose(const JointPose5d& q)
{
    return std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z)
        && std::isfinite(q.b) && std::isfinite(q.c);
}

bool finiteParameters(const Physical17Parameters& p)
{
    return std::all_of(p.value.begin(), p.value.end(), [](double v) { return std::isfinite(v); });
}

std::vector<double> basePointWeights(const std::vector<SphereProfileObservation>& observations)
{
    std::vector<double> out;
    std::size_t total = 0;
    for (const auto& obs : observations) {
        total += obs.points.size();
    }
    out.reserve(total);
    for (const auto& obs : observations) {
        for (const auto& point : obs.points) {
            out.push_back(std::max(0.0, point.weight));
        }
    }
    return out;
}

std::vector<double> evaluateResiduals(
    const std::vector<SphereProfileObservation>& observations,
    const std::array<double, kStateCount>& state,
    double sphereRadiusMm)
{
    Physical17Parameters parameters;
    Vec3d sphereCenter;
    unpackState(state, parameters, sphereCenter);

    std::vector<double> residuals;
    std::size_t total = 0;
    for (const auto& obs : observations) {
        total += obs.points.size();
    }
    residuals.reserve(total);

    const PreparedPhysical17 prepared(parameters);
    for (const auto& obs : observations) {
        const Vec3d center = centerInSensor(profileTransform(prepared, obs.joints), sphereCenter);
        for (const auto& point : obs.points) {
            residuals.push_back(norm(Vec3d(point.x-center.x, -center.y, point.z-center.z)) - sphereRadiusMm);
        }
    }
    return residuals;
}

double huberCost(double residual, double delta)
{
    const double a = std::abs(residual);
    if (delta <= 0.0 || a <= delta) {
        return 0.5 * residual * residual;
    }
    return delta * (a - 0.5 * delta);
}

double totalRobustCost(const std::vector<double>& residuals,
                       const std::vector<double>& baseWeights,
                       double huberDelta)
{
    double cost = 0.0;
    for (std::size_t i = 0; i < residuals.size(); ++i) {
        const double w = (i < baseWeights.size()) ? baseWeights[i] : 1.0;
        cost += w * huberCost(residuals[i], huberDelta);
    }
    return cost;
}

double huberIrlsWeight(double residual, double delta)
{
    const double a = std::abs(residual);
    if (delta <= 0.0 || a <= delta || a < 1e-18) {
        return 1.0;
    }
    return delta / a;
}

// Differentiate only the 3-D sensor-frame sphere center once per profile.
// Machine/hand-eye derivatives use central differences; the 3 nuisance-center
// derivatives and the point-to-sphere distance derivative are analytic.
// Derivatives are with respect to normalized state increments, as in the old LM.
struct ProfileLinearizer
{
    PreparedPhysical17 base;
    Vec3d sphereCenter;
    std::vector<PreparedPhysical17> plus, minus;
    const std::array<double, kStateCount>& scales;
    const std::vector<int>& active;
    double hNorm;

    ProfileLinearizer(const std::array<double, kStateCount>& state,
                      const std::array<double, kStateCount>& scaleValues,
                      double relativeStep, const std::vector<int>& activeIndices)
        : base([&]() { Physical17Parameters p; Vec3d c; unpackState(state, p, c); return p; }()),
          sphereCenter(state[17], state[18], state[19]), scales(scaleValues),
          active(activeIndices), hNorm(std::max(1e-8, relativeStep))
    {
        plus.reserve(active.size()); minus.reserve(active.size());
        Physical17Parameters p; Vec3d unused; unpackState(state, p, unused);
        for (int index : active) {
            Physical17Parameters pp = p, pm = p;
            if (index < static_cast<int>(kTargetCount)) {
                const double h = hNorm * scales[static_cast<std::size_t>(index)];
                pp[index] += h; pm[index] -= h;
            }
            plus.emplace_back(pp); minus.emplace_back(pm);
        }
    }

    Vec3d evaluate(const JointPose5d& q, std::array<Vec3d, kStateCount>& dc) const
    {
        const ProfileTransform t = profileTransform(base, q);
        const Vec3d center = centerInSensor(t, sphereCenter);
        for (std::size_t j=0; j<active.size(); ++j) {
            const int index = active[j];
            if (index >= static_cast<int>(kTargetCount)) {
                const int axis = index-static_cast<int>(kTargetCount);
                dc[j] = mul(Vec3d(t.rotation.v[axis][0], t.rotation.v[axis][1],
                                  t.rotation.v[axis][2]), scales[index]);
            } else {
                const Vec3d cp = centerInSensor(profileTransform(plus[j], q), sphereCenter);
                const Vec3d cm = centerInSensor(profileTransform(minus[j], q), sphereCenter);
                dc[j] = mul(sub(cp, cm), 0.5/hNorm);
            }
        }
        return center;
    }
};

DenseMatrix numericalJacobian(
    const std::vector<SphereProfileObservation>& observations,
    const std::array<double, kStateCount>& state,
    const std::array<double, kStateCount>& scales,
    double /*sphereRadiusMm*/, double relativeStep, const std::vector<int>& active)
{
    std::size_t count=0;
    for (const auto& obs : observations) count += obs.points.size();
    DenseMatrix jac(static_cast<int>(count), static_cast<int>(active.size()));
    const ProfileLinearizer linearizer(state, scales, relativeStep, active);
    int row=0;
    for (const auto& obs : observations) {
        std::array<Vec3d, kStateCount> dc{};
        const Vec3d c = linearizer.evaluate(obs.joints, dc);
        for (const auto& point : obs.points) {
            const Vec3d v(c.x-point.x, c.y, c.z-point.z);
            const double length=norm(v);
            // The norm is nondifferentiable at zero.  A zero row matches the
            // symmetric-difference convention; such a point is not on a sphere.
            const Vec3d n = length>1e-14 ? mul(v, 1.0/length) : Vec3d();
            for (std::size_t j=0; j<active.size(); ++j)
                jac(row, static_cast<int>(j)) = dot(n, dc[j]);
            ++row;
        }
    }
    return jac;
}

struct ProfileNormalEquations
{
    DenseMatrix h;
    std::vector<double> gradient;
    bool cancelled=false;
};

// Exact IRLS normal-equation factorization for the current linearization:
// J_i = A_i B_i, B_i=dc_sensor/d(normalized_state), A_i=d(residual)/dc_sensor.
// Accumulate only A_i^T W A_i (3x3) and A_i^T W r (3x1) per profile, then
// scatter B_i^T (...) B_i.  No N-by-20 allocation and no per-point 20x20 loop.
ProfileNormalEquations profileNormalEquations(
    const std::vector<SphereProfileObservation>& observations,
    const std::array<double, kStateCount>& state,
    const Physical17SolverOptions& options, const std::vector<int>& active,
    const std::atomic_bool* cancelRequested=nullptr)
{
    const int n=static_cast<int>(active.size());
    ProfileNormalEquations out{DenseMatrix(n,n), std::vector<double>(active.size(),0.0), false};
    const ProfileLinearizer linearizer(state, options.parameterScales,
                                       options.finiteDifferenceRelativeStep, active);
    const double radius=0.5*options.sphereDiameterMm;
    for (const auto& obs : observations) {
        if (cancelRequested && cancelRequested->load(std::memory_order_relaxed)) {
            out.cancelled=true; return out;
        }
        std::array<Vec3d, kStateCount> dc{};
        const Vec3d c=linearizer.evaluate(obs.joints, dc);
        Mat3d a;
        Vec3d g;
        for (const auto& point : obs.points) {
            if (point.weight<=0.0) continue;
            const Vec3d v(c.x-point.x, c.y, c.z-point.z);
            const double length=norm(v);
            if (!(length>1e-14)) continue;
            const double r=length-radius;
            const double w=point.weight*huberIrlsWeight(r,options.huberDeltaMm);
            const Vec3d u=mul(v,1.0/length);
            const double nx[3]={u.x,u.y,u.z};
            g=add(g,mul(u,w*r));
            for (int i=0;i<3;++i)
                for (int j=i;j<3;++j) a.v[i][j]+=w*nx[i]*nx[j];
        }
        for (int i=0;i<3;++i) for (int j=0;j<i;++j) a.v[i][j]=a.v[j][i];
        std::array<Vec3d,kStateCount> adc{};
        for (int j=0;j<n;++j) adc[j]=multiply(a,dc[j]);
        for (int i=0;i<n;++i) {
            out.gradient[i]+=dot(dc[i],g);
            for (int j=i;j<n;++j) out.h(i,j)+=dot(dc[i],adc[j]);
        }
    }
    for (int i=0;i<n;++i) for (int j=0;j<i;++j) out.h(i,j)=out.h(j,i);
    return out;
}

bool solveDenseLinear(DenseMatrix a, std::vector<double> b, std::vector<double>& x)
{
    if (a.rows != a.cols || a.rows != static_cast<int>(b.size())) {
        return false;
    }
    const int n = a.rows;
    x.assign(static_cast<std::size_t>(n), 0.0);

    for (int k = 0; k < n; ++k) {
        int pivot = k;
        double pivotAbs = std::abs(a(k, k));
        for (int r = k + 1; r < n; ++r) {
            const double candidate = std::abs(a(r, k));
            if (candidate > pivotAbs) {
                pivotAbs = candidate;
                pivot = r;
            }
        }
        if (pivotAbs < 1e-18 || !std::isfinite(pivotAbs)) {
            return false;
        }
        if (pivot != k) {
            for (int c = k; c < n; ++c) {
                std::swap(a(k, c), a(pivot, c));
            }
            std::swap(b[static_cast<std::size_t>(k)], b[static_cast<std::size_t>(pivot)]);
        }

        const double diag = a(k, k);
        for (int r = k + 1; r < n; ++r) {
            const double factor = a(r, k) / diag;
            if (std::abs(factor) < 1e-30) {
                continue;
            }
            a(r, k) = 0.0;
            for (int c = k + 1; c < n; ++c) {
                a(r, c) -= factor * a(k, c);
            }
            b[static_cast<std::size_t>(r)] -= factor * b[static_cast<std::size_t>(k)];
        }
    }

    for (int r = n - 1; r >= 0; --r) {
        double rhs = b[static_cast<std::size_t>(r)];
        for (int c = r + 1; c < n; ++c) {
            rhs -= a(r, c) * x[static_cast<std::size_t>(c)];
        }
        if (std::abs(a(r, r)) < 1e-18) {
            return false;
        }
        x[static_cast<std::size_t>(r)] = rhs / a(r, r);
    }
    return true;
}

bool invert3(const DenseMatrix& input, DenseMatrix& inverse)
{
    if (input.rows != 3 || input.cols != 3) {
        return false;
    }
    inverse = DenseMatrix(3, 3);
    for (int col = 0; col < 3; ++col) {
        std::vector<double> rhs(3, 0.0);
        rhs[static_cast<std::size_t>(col)] = 1.0;
        std::vector<double> solution;
        if (!solveDenseLinear(input, rhs, solution)) {
            return false;
        }
        for (int row = 0; row < 3; ++row) {
            inverse(row, col) = solution[static_cast<std::size_t>(row)];
        }
    }
    return true;
}

DenseMatrix normalInformation(const DenseMatrix& jac)
{
    DenseMatrix h(jac.cols, jac.cols, 0.0);
    for (int row = 0; row < jac.rows; ++row) {
        for (int a = 0; a < jac.cols; ++a) {
            const double ja = jac(row, a);
            for (int b = a; b < jac.cols; ++b) {
                h(a, b) += ja * jac(row, b);
            }
        }
    }
    for (int a = 0; a < jac.cols; ++a) {
        for (int b = 0; b < a; ++b) {
            h(a, b) = h(b, a);
        }
    }
    return h;
}

DenseMatrix addDense(const DenseMatrix& a, const DenseMatrix& b)
{
    DenseMatrix out(a.rows, a.cols, 0.0);
    if (a.rows != b.rows || a.cols != b.cols) {
        return out;
    }
    for (std::size_t i = 0; i < a.data.size(); ++i) {
        out.data[i] = a.data[i] + b.data[i];
    }
    return out;
}

DenseMatrix targetSchurInformation(const DenseMatrix& full, int targetCount)
{
    const int nuisanceCount = 3;
    if (targetCount <= 0 || full.rows != targetCount + nuisanceCount ||
        full.cols != targetCount + nuisanceCount) {
        return DenseMatrix();
    }
    DenseMatrix target(targetCount, targetCount, 0.0);
    DenseMatrix crossBlock(targetCount, nuisanceCount, 0.0);
    DenseMatrix nuisance(nuisanceCount, nuisanceCount, 0.0);
    for (int r = 0; r < targetCount; ++r) {
        for (int c = 0; c < targetCount; ++c) {
            target(r, c) = full(r, c);
        }
        for (int c = 0; c < nuisanceCount; ++c) {
            crossBlock(r, c) = full(r, targetCount + c);
        }
    }
    for (int r = 0; r < nuisanceCount; ++r) {
        for (int c = 0; c < nuisanceCount; ++c) {
            nuisance(r, c) = full(targetCount + r, targetCount + c);
        }
    }
    DenseMatrix nuisanceInverse;
    if (!invert3(nuisance, nuisanceInverse)) {
        return DenseMatrix();
    }
    for (int r = 0; r < targetCount; ++r) {
        for (int c = 0; c < targetCount; ++c) {
            double correction = 0.0;
            for (int i = 0; i < nuisanceCount; ++i) {
                for (int j = 0; j < nuisanceCount; ++j) {
                    correction += crossBlock(r, i) * nuisanceInverse(i, j) *
                                  crossBlock(c, j);
                }
            }
            target(r, c) -= correction;
        }
    }
    return target;
}

double logDetPositiveDefinite(const DenseMatrix& input)
{
    if (input.rows != input.cols || input.rows == 0) {
        return -std::numeric_limits<double>::infinity();
    }
    const int n = input.rows;
    DenseMatrix l(n, n, 0.0);
    double logDet = 0.0;
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j <= i; ++j) {
            double sum = input(i, j);
            for (int k = 0; k < j; ++k) {
                sum -= l(i, k) * l(j, k);
            }
            if (i == j) {
                if (!(sum > 1e-24) || !std::isfinite(sum)) {
                    return -std::numeric_limits<double>::infinity();
                }
                l(i, j) = std::sqrt(sum);
                logDet += 2.0 * std::log(l(i, j));
            } else {
                l(i, j) = sum / l(j, j);
            }
        }
    }
    return logDet;
}

std::vector<double> jacobiEigenvalues(DenseMatrix a)
{
    if (a.rows != a.cols) {
        return {};
    }
    const int n = a.rows;
    const int maxSweeps = std::max(50, 20 * n * n);
    for (int sweep = 0; sweep < maxSweeps; ++sweep) {
        int p = 0;
        int q = 1;
        double maximum = 0.0;
        for (int r = 0; r < n; ++r) {
            for (int c = r + 1; c < n; ++c) {
                const double value = std::abs(a(r, c));
                if (value > maximum) {
                    maximum = value;
                    p = r;
                    q = c;
                }
            }
        }
        if (maximum < 1e-14) {
            break;
        }

        const double app = a(p, p);
        const double aqq = a(q, q);
        const double apq = a(p, q);
        const double phi = 0.5 * std::atan2(2.0 * apq, aqq - app);
        const double c = std::cos(phi);
        const double s = std::sin(phi);

        for (int k = 0; k < n; ++k) {
            if (k == p || k == q) {
                continue;
            }
            const double akp = a(k, p);
            const double akq = a(k, q);
            a(k, p) = a(p, k) = c * akp - s * akq;
            a(k, q) = a(q, k) = s * akp + c * akq;
        }
        a(p, p) = c * c * app - 2.0 * s * c * apq + s * s * aqq;
        a(q, q) = s * s * app + 2.0 * s * c * apq + c * c * aqq;
        a(p, q) = a(q, p) = 0.0;
    }

    std::vector<double> eigenvalues(static_cast<std::size_t>(n), 0.0);
    for (int i = 0; i < n; ++i) {
        eigenvalues[static_cast<std::size_t>(i)] = a(i, i);
    }
    std::sort(eigenvalues.begin(), eigenvalues.end(), std::greater<double>());
    return eigenvalues;
}

// Remove the KNOWN exact translation gauge analytically before diagonalizing
// information matrices.  This is not clamping an observed rank to 14: Q spans
// the orthogonal complement of the three explicit gauge directions, so true
// additional degeneracies are still measured and rejected.
DenseMatrix translationGaugeComplementInformation(
    const DenseMatrix& information, const Physical17Parameters& p,
    const std::array<double,kStateCount>& scales)
{
    const int n=information.rows;
    const AxisLine c=cAxisLine(p), b=bAxisLine(p);
    std::vector<std::vector<double>> basis;
    auto appendOrthogonal=[&](std::vector<double> v) {
        for (int pass=0;pass<2;++pass)
            for (const auto& q:basis) {
                double d=0.0; for (int i=0;i<n;++i) d+=q[i]*v[i];
                for (int i=0;i<n;++i) v[i]-=d*q[i];
            }
        double length=0.0; for (double x:v) length+=x*x;
        if (length<1e-22) return false;
        length=std::sqrt(length);
        for (double& x:v) x/=length;
        basis.push_back(std::move(v));
        return true;
    };
    for (int axis=0;axis<3;++axis) {
        Vec3d d(axis==0 ? 1.0:0.0,axis==1 ? 1.0:0.0,axis==2 ? 1.0:0.0);
        std::vector<double> g(static_cast<std::size_t>(n),0.0);
        g[3]=dot(c.basisU,d)/scales[3]; g[4]=dot(c.basisV,d)/scales[4];
        g[7]=dot(b.basisU,d)/scales[7]; g[8]=dot(b.basisV,d)/scales[8];
        g[14+axis]=1.0/scales[14+axis];
        if (n==static_cast<int>(kStateCount)) g[17+axis]=1.0/scales[17+axis];
        if (!appendOrthogonal(std::move(g))) return DenseMatrix();
    }
    for (int axis=0;axis<n && static_cast<int>(basis.size())<n;++axis) {
        std::vector<double> v(static_cast<std::size_t>(n),0.0); v[axis]=1.0;
        appendOrthogonal(std::move(v));
    }
    if (static_cast<int>(basis.size())!=n) return DenseMatrix();
    DenseMatrix out(n-3,n-3);
    for (int i=0;i<n-3;++i)
        for (int j=i;j<n-3;++j) {
            double v=0.0;
            for (int r=0;r<n;++r)
                for (int col=0;col<n;++col) v+=basis[i+3][r]*information(r,col)*basis[j+3][col];
            out(i,j)=out(j,i)=v;
        }
    return out;
}

struct OptimizeResult
{
    bool success = false;
    bool cancelled = false;
    bool converged = false;
    int iterations = 0;
    double cost = std::numeric_limits<double>::infinity();
};

OptimizeResult optimizeSubset(
    const std::vector<SphereProfileObservation>& observations,
    const Physical17SolverOptions& options,
    const std::vector<int>& active,
    int maxIterations,
    std::array<double, kStateCount>& state,
    const std::function<bool(int, int, double)>& iterationCallback = {},
    const std::atomic_bool* cancelRequested = nullptr)
{
    OptimizeResult result;
    if (observations.empty() || active.empty()) {
        return result;
    }

    const double radius = 0.5 * options.sphereDiameterMm;
    const std::vector<double> baseWeights = basePointWeights(observations);
    std::vector<double> residuals = evaluateResiduals(observations, state, radius);
    double cost = totalRobustCost(residuals, baseWeights, options.huberDeltaMm);
    double lambda = std::max(1e-12, options.initialLambda);
    if (!std::isfinite(cost)) return result;

    ProfileNormalEquations normal;
    bool needLinearization = true;
    for (int iteration = 0; iteration < maxIterations; ++iteration) {
        if (iterationCallback && !iterationCallback(iteration, maxIterations, cost)) {
            result.cancelled = true;
            break;
        }
        if (needLinearization) {
            normal = profileNormalEquations(observations, state, options, active, cancelRequested);
            if (normal.cancelled) { result.cancelled=true; break; }
            needLinearization=false;
        }
        const DenseMatrix& h=normal.h;
        const std::vector<double>& gradient=normal.gradient;
        if (!std::all_of(h.data.begin(),h.data.end(),[](double v){return std::isfinite(v);}) ||
            !std::all_of(gradient.begin(),gradient.end(),[](double v){return std::isfinite(v);})) break;
        double scaledGradient=0.0;
        for (int i=0;i<h.rows;++i)
            scaledGradient=std::max(scaledGradient,std::abs(gradient[i])/std::sqrt(std::max(1e-30,h(i,i))));
        if (scaledGradient <= options.convergenceGradient * std::max(1.0,std::sqrt(2.0*std::max(0.0,cost)))) {
            result.success=true;
            result.converged=true;
            break;
        }
        result.iterations=iteration+1;

        DenseMatrix damped = h;
        for (int i = 0; i < damped.rows; ++i) {
            damped(i, i) += lambda * std::max(1.0, h(i, i));
        }

        std::vector<double> rhs(gradient.size(), 0.0);
        for (std::size_t i = 0; i < gradient.size(); ++i) {
            rhs[i] = -gradient[i];
        }
        std::vector<double> deltaNormalized;
        if (!solveDenseLinear(damped, rhs, deltaNormalized)) {
            lambda *= 10.0;
            if (lambda > 1e16) {
                break;
            }
            continue;
        }

        double stepNorm = 0.0;
        std::array<double, kStateCount> trial = state;
        for (std::size_t j = 0; j < active.size(); ++j) {
            const int index = active[j];
            const double normalizedStep = deltaNormalized[j];
            stepNorm += normalizedStep * normalizedStep;
            trial[static_cast<std::size_t>(index)] +=
                options.parameterScales[static_cast<std::size_t>(index)] * normalizedStep;
        }
        stepNorm = std::sqrt(stepNorm);

        const std::vector<double> trialResiduals = evaluateResiduals(observations, trial, radius);
        const double trialCost = totalRobustCost(
            trialResiduals, baseWeights, options.huberDeltaMm);

        if (std::isfinite(trialCost) && trialCost < cost) {
            const double relativeImprovement = (cost - trialCost) / std::max(1.0, cost);
            state = trial;
            residuals = trialResiduals;
            cost = trialCost;
            lambda = std::max(1e-12, lambda * 0.3);
            needLinearization = true;
            if (stepNorm < options.convergenceStep ||
                relativeImprovement < options.convergenceCost) {
                result.success = true;
                result.converged = true;
                break;
            }
        } else {
            if (stepNorm < options.convergenceStep && lambda <= 1.0 && std::isfinite(trialCost)) {
                result.converged=true;
                break;
            }
            lambda *= 10.0;
            if (lambda > 1e16) {
                break;
            }
        }
    }

    result.cost = cost;
    if (!result.success) {
        result.success = std::isfinite(cost);
    }
    return result;
}

std::vector<int> machineAndCenterIndices()
{
    std::vector<int> indices;
    for (int i = 0; i <= static_cast<int>(Physical17Parameters::BTiltZ); ++i) {
        indices.push_back(i);
    }
    indices.push_back(static_cast<int>(kTargetCount + 0));
    indices.push_back(static_cast<int>(kTargetCount + 1));
    indices.push_back(static_cast<int>(kTargetCount + 2));
    return indices;
}

std::vector<int> handEyeAndCenterIndices()
{
    return {
        static_cast<int>(Physical17Parameters::HandEyeRx),
        static_cast<int>(Physical17Parameters::HandEyeRy),
        static_cast<int>(Physical17Parameters::HandEyeRz),
        static_cast<int>(kTargetCount + 0),
        static_cast<int>(kTargetCount + 1),
        static_cast<int>(kTargetCount + 2)
    };
}

// Constrained solve coordinates: the physical state remains 17-D, but the
// three hand-eye translations are equality-constrained to the chosen gauge
// anchor.  The data-active block is therefore 14 physical directions + the
// three unknown sphere-center nuisance coordinates.
std::vector<int> gaugeFixedObservableAndCenterIndices()
{
    std::vector<int> indices;
    indices.reserve(kObservableStateCount);
    for (int i = 0; i < static_cast<int>(kIdentifiableCount); ++i) {
        indices.push_back(i);
    }
    indices.push_back(static_cast<int>(kTargetCount + 0));
    indices.push_back(static_cast<int>(kTargetCount + 1));
    indices.push_back(static_cast<int>(kTargetCount + 2));
    return indices;
}

std::vector<int> fullIndices()
{
    std::vector<int> indices(kStateCount);
    std::iota(indices.begin(), indices.end(), 0);
    return indices;
}

std::array<double, kStateCount> deterministicStart(
    const std::array<double, kStateCount>& base,
    const Physical17SolverOptions& options,
    int startIndex)
{
    std::array<double, kStateCount> out = base;
    if (startIndex == 0) {
        return out;
    }
    std::minstd_rand generator(static_cast<unsigned int>(0x5A17u + startIndex * 7919u));
    std::uniform_real_distribution<double> unit(-1.0, 1.0);
    for (std::size_t i = 0; i < kStateCount; ++i) {
        double amplitude = 0.0;
        if (i <= Physical17Parameters::Syz ||
            (i >= Physical17Parameters::CTiltX && i <= Physical17Parameters::CTiltY) ||
            (i >= Physical17Parameters::BTiltX && i <= Physical17Parameters::BTiltZ)) {
            amplitude = 2.0;
        } else if (i == Physical17Parameters::COffsetU ||
                   i == Physical17Parameters::COffsetV ||
                   i == Physical17Parameters::BOffsetU ||
                   i == Physical17Parameters::BOffsetV) {
            amplitude = 0.8;
        } else if (i >= Physical17Parameters::HandEyeRx &&
                   i <= Physical17Parameters::HandEyeRz) {
            amplitude = 1.5;
        } else if (i >= Physical17Parameters::HandEyeTx &&
                   i <= Physical17Parameters::HandEyeTz) {
            amplitude = 0.0; // exact translation gauge anchor
        } else {
            amplitude = 0.05;
        }
        out[i] += amplitude * options.parameterScales[i] * unit(generator);
    }
    return out;
}

std::string trim(const std::string& text)
{
    const std::size_t first = text.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
        return std::string();
    }
    const std::size_t last = text.find_last_not_of(" \t\r\n");
    return text.substr(first, last - first + 1);
}

std::vector<std::string> splitCsvLine(const std::string& line)
{
    std::vector<std::string> fields;
    std::string current;
    bool quoted = false;
    for (std::size_t i = 0; i < line.size(); ++i) {
        const char ch = line[i];
        if (ch == '"') {
            if (quoted && i + 1 < line.size() && line[i + 1] == '"') {
                current.push_back('"');
                ++i;
            } else {
                quoted = !quoted;
            }
        } else if (ch == ',' && !quoted) {
            fields.push_back(trim(current));
            current.clear();
        } else {
            current.push_back(ch);
        }
    }
    fields.push_back(trim(current));
    return fields;
}

bool parseDouble(const std::string& value, double& output)
{
    char* end = nullptr;
    output = std::strtod(value.c_str(), &end);
    return end != value.c_str() && *end == '\0' && std::isfinite(output);
}

bool parseInt64(const std::string& value, std::int64_t& output)
{
    char* end = nullptr;
    const long long parsed = std::strtoll(value.c_str(), &end, 10);
    if (end == value.c_str() || *end != '\0') {
        return false;
    }
    output = static_cast<std::int64_t>(parsed);
    return true;
}

std::string jsonEscape(const std::string& value)
{
    std::ostringstream out;
    for (char ch : value) {
        switch (ch) {
        case '\\': out << "\\\\"; break;
        case '"': out << "\\\""; break;
        case '\n': out << "\\n"; break;
        case '\r': out << "\\r"; break;
        case '\t': out << "\\t"; break;
        default: out << ch; break;
        }
    }
    return out.str();
}

bool solve3x3(const Mat3d& a, const Vec3d& b, Vec3d& solution)
{
    DenseMatrix dense(3, 3);
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            dense(r, c) = a.v[r][c];
        }
    }
    std::vector<double> rhs{b.x, b.y, b.z};
    std::vector<double> x;
    if (!solveDenseLinear(dense, rhs, x)) {
        return false;
    }
    solution = Vec3d(x[0], x[1], x[2]);
    return finiteVec(solution);
}

bool circleFromThreePoints(const SensorProfilePoint& p1,
                           const SensorProfilePoint& p2,
                           const SensorProfilePoint& p3,
                           double& centerX,
                           double& centerZ,
                           double& radius)
{
    const double x1 = p1.x, z1 = p1.z;
    const double x2 = p2.x, z2 = p2.z;
    const double x3 = p3.x, z3 = p3.z;
    const double d = 2.0 * (x1 * (z2 - z3) + x2 * (z3 - z1) + x3 * (z1 - z2));
    if (std::abs(d) < 1e-12) {
        return false;
    }
    const double q1 = x1 * x1 + z1 * z1;
    const double q2 = x2 * x2 + z2 * z2;
    const double q3 = x3 * x3 + z3 * z3;
    centerX = (q1 * (z2 - z3) + q2 * (z3 - z1) + q3 * (z1 - z2)) / d;
    centerZ = (q1 * (x3 - x2) + q2 * (x1 - x3) + q3 * (x2 - x1)) / d;
    radius = std::hypot(x1 - centerX, z1 - centerZ);
    return std::isfinite(centerX) && std::isfinite(centerZ)
        && std::isfinite(radius) && radius > 1e-9;
}

bool algebraicCircleFit(const std::vector<SensorProfilePoint>& points,
                        const std::vector<std::size_t>& indices,
                        double& centerX,
                        double& centerZ,
                        double& radius)
{
    Mat3d normal;
    Vec3d rhs;
    double weightSum = 0.0;
    for (std::size_t index : indices) {
        if (index >= points.size()) continue;
        const auto& point = points[index];
        if (!std::isfinite(point.x) || !std::isfinite(point.z) || point.weight <= 0.0) continue;
        const double w = point.weight;
        const double row[3] = {point.x, point.z, 1.0};
        const double target = -(point.x * point.x + point.z * point.z);
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) normal.v[r][c] += w * row[r] * row[c];
        }
        rhs.x += w * row[0] * target;
        rhs.y += w * row[1] * target;
        rhs.z += w * row[2] * target;
        weightSum += w;
    }
    Vec3d abc;
    if (weightSum <= 0.0 || !solve3x3(normal, rhs, abc)) return false;
    centerX = -0.5 * abc.x;
    centerZ = -0.5 * abc.y;
    const double radiusSquared = centerX * centerX + centerZ * centerZ - abc.z;
    if (!(radiusSquared > 0.0)) return false;
    radius = std::sqrt(radiusSquared);
    return std::isfinite(centerX) && std::isfinite(centerZ) && std::isfinite(radius);
}

void refineCircleGeometric(const std::vector<SensorProfilePoint>& points,
                           const std::vector<std::size_t>& indices,
                           int iterations,
                           double& centerX,
                           double& centerZ,
                           double& radius)
{
    for (int iter = 0; iter < std::max(0, iterations); ++iter) {
        Mat3d normal;
        Vec3d rhs;
        int used = 0;
        for (std::size_t index : indices) {
            if (index >= points.size()) continue;
            const auto& point = points[index];
            if (!std::isfinite(point.x) || !std::isfinite(point.z) || point.weight <= 0.0) continue;
            const double dx = centerX - point.x;
            const double dz = centerZ - point.z;
            const double distance = std::hypot(dx, dz);
            if (distance < 1e-12) continue;
            const double residual = distance - radius;
            const double jac[3] = {dx / distance, dz / distance, -1.0};
            const double w = point.weight;
            for (int r = 0; r < 3; ++r) {
                for (int c = 0; c < 3; ++c) normal.v[r][c] += w * jac[r] * jac[c];
            }
            rhs.x += -w * jac[0] * residual;
            rhs.y += -w * jac[1] * residual;
            rhs.z += -w * jac[2] * residual;
            ++used;
        }
        Vec3d delta;
        if (used < 3 || !solve3x3(normal, rhs, delta)) break;
        centerX += delta.x;
        centerZ += delta.y;
        radius += delta.z;
        if (!std::isfinite(centerX) || !std::isfinite(centerZ) || !std::isfinite(radius) || radius <= 0.0) break;
        if (std::sqrt(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z) < 1e-10) break;
    }
}

std::vector<std::size_t> circleInliers(const std::vector<SensorProfilePoint>& points,
                                       const std::vector<std::size_t>& validIndices,
                                       double centerX,
                                       double centerZ,
                                       double radius,
                                       double thresholdMm,
                                       double* rms = nullptr)
{
    std::vector<std::size_t> inliers;
    double squared = 0.0;
    for (std::size_t index : validIndices) {
        const auto& point = points[index];
        const double residual = std::abs(std::hypot(point.x - centerX, point.z - centerZ) - radius);
        if (residual <= thresholdMm) {
            inliers.push_back(index);
            squared += residual * residual;
        }
    }
    if (rms) *rms = inliers.empty() ? std::numeric_limits<double>::infinity()
                                     : std::sqrt(squared / static_cast<double>(inliers.size()));
    return inliers;
}

double circleArcSpanDeg(const std::vector<SensorProfilePoint>& points,
                        const std::vector<std::size_t>& indices,
                        double centerX,
                        double centerZ)
{
    if (indices.size() < 2) return 0.0;
    std::vector<double> angles;
    angles.reserve(indices.size());
    for (std::size_t index : indices) {
        const auto& point = points[index];
        double angle = std::atan2(point.z - centerZ, point.x - centerX);
        if (angle < 0.0) angle += 2.0 * kPi;
        angles.push_back(angle);
    }
    std::sort(angles.begin(), angles.end());
    double largestGap = 0.0;
    for (std::size_t i = 1; i < angles.size(); ++i) {
        largestGap = std::max(largestGap, angles[i] - angles[i - 1]);
    }
    largestGap = std::max(largestGap, 2.0 * kPi - angles.back() + angles.front());
    return (2.0 * kPi - largestGap) / kDegToRad;
}

std::vector<double> inclusiveRange(double minimum, double maximum, double step)
{
    std::vector<double> values;
    if (!std::isfinite(minimum) || !std::isfinite(maximum) || !std::isfinite(step) ||
        !(step > 0.0) || maximum < minimum ||
        !std::isfinite((maximum-minimum)/step) || (maximum-minimum)/step > 10000.0) {
        return values;
    }
    for (double value = minimum; value <= maximum + 0.5 * step; value += step) {
        values.push_back(std::min(value, maximum));
        if (values.size() > 10000) return {};
    }
    if (values.empty() || std::abs(values.back() - maximum) > 1e-9) {
        values.push_back(maximum);
    }
    // Calibration needs an explicit all-zero rotary reference whenever the
    // configured range straddles zero, even when the arithmetic step misses it.
    if (minimum < 0.0 && maximum > 0.0) {
        values.push_back(0.0);
    }
    std::sort(values.begin(), values.end());
    values.erase(std::unique(values.begin(), values.end(),
                             [](double a, double b) {
                                 return std::abs(a - b) < 1e-10;
                             }),
                 values.end());
    return values;
}

SphereProfileObservation syntheticProfile(const JointPose5d& joint,
                                           const Vec3d& centerSensor,
                                           double sphereRadius,
                                           int pointCount)
{
    SphereProfileObservation observation;
    observation.joints = joint;
    const double crossSectionSquared = sphereRadius * sphereRadius -
                                       centerSensor.y * centerSensor.y;
    if (!(crossSectionSquared > 0.0)) {
        return observation;
    }
    const double rho = std::sqrt(crossSectionSquared);
    const int count = std::max(7, pointCount);
    observation.points.reserve(static_cast<std::size_t>(count));
    for (int i = 0; i < count; ++i) {
        const double t = (count == 1) ? 0.0 : static_cast<double>(i) / (count - 1);
        const double x = centerSensor.x + (-0.90 + 1.80 * t) * rho;
        const double dx = x - centerSensor.x;
        const double z = centerSensor.z - std::sqrt(std::max(0.0, rho * rho - dx * dx));
        SensorProfilePoint point;
        point.x = x;
        point.z = z;
        point.weight = 1.0;
        observation.points.push_back(point);
    }
    return observation;
}

// Use the predicted visible contour, not an unconditional z=center-sqrt(...)
// lower arc.  In a flipped-Z installation the visible contour can be above
// the center; hemisphere selection comes from inverse hand-eye, not a sign guess.
SphereProfileObservation syntheticAcquisitionProfile(
    const JointPose5d& q, const Vec3d& center, double radius,
    const Physical17Parameters& model, const PosePlannerConfig& cfg)
{
    if (!cfg.useEndEffectorFacingArc && !cfg.useTrapezoidalSensorField)
        return syntheticProfile(q,center,radius,cfg.syntheticPointsPerProfile);
    SphereProfileObservation out; out.joints=q;
    const double rho2=radius*radius-center.y*center.y;
    if (!(rho2>0.0)) return out;
    const double rho=std::sqrt(rho2);
    const PreparedPhysical17 prepared(model);
    const Vec3d end=mul(multiply(transpose(prepared.handEyeRotation),prepared.handEyeTranslation),-1.0);
    const Vec3d towardEnd=sub(end,center);
    const int samples=std::max(144,std::min(1440,12*std::max(7,cfg.syntheticPointsPerProfile)));
    std::vector<Vec3d> circle; circle.reserve(samples);
    std::vector<bool> good(static_cast<std::size_t>(samples),false);
    for (int i=0;i<samples;++i) {
        const double angle=2.0*kPi*i/samples;
        Vec3d point(center.x+rho*std::cos(angle),0.0,center.z+rho*std::sin(angle));
        circle.push_back(point);
        if (point.z<cfg.sensorZMinMm+cfg.sensorZSafetyMarginMm ||
            point.z>cfg.sensorZMaxMm-cfg.sensorZSafetyMarginMm) continue;
        double halfWidth=cfg.sensorXHalfWidthMm;
        if (cfg.useTrapezoidalSensorField) {
            const double depth=cfg.sensorZCoordinateFlipped
                ? (cfg.sensorZMaxMm-point.z)/(cfg.sensorZMaxMm-cfg.sensorZMinMm)
                : (point.z-cfg.sensorZMinMm)/(cfg.sensorZMaxMm-cfg.sensorZMinMm);
            halfWidth=0.5*(cfg.sensorXSpanNearMm+depth*(cfg.sensorXSpanFarMm-cfg.sensorXSpanNearMm));
        }
        if (std::abs(point.x)>halfWidth-cfg.sensorXSafetyMarginMm) continue;
        if (cfg.useEndEffectorFacingArc && dot(sub(point,center),towardEnd)<=0.0) continue;
        good[i]=true;
    }
    int run=0,bestLength=0,bestEnd=-1;
    for (int i=0;i<2*samples;++i) {
        run=good[i%samples] ? std::min(samples,run+1):0;
        if (run>bestLength) {bestLength=run;bestEnd=i;}
    }
    if (bestLength<5) return out;
    const int count=std::min(bestLength,std::max(7,cfg.syntheticPointsPerProfile));
    for (int i=0;i<count;++i) {
        const int at=bestEnd-bestLength+1+static_cast<int>(std::lround(double(i)*(bestLength-1)/(count-1)));
        const Vec3d& point=circle[static_cast<std::size_t>(at%samples)];
        SensorProfilePoint p; p.x=point.x;p.z=point.z;p.weight=1.0;
        out.points.push_back(p);
    }
    return out;
}

DenseMatrix candidateInformation(const CandidatePose& candidate,
                                 const Physical17Parameters& estimate,
                                 const Vec3d& sphereCenter,
                                 double sphereRadius,
                                 double relativeStep)
{
    std::vector<SphereProfileObservation> observations{candidate.syntheticObservation};
    std::array<double, kStateCount> state = packState(estimate, sphereCenter);
    Physical17SolverOptions options;
    const std::vector<int> active = gaugeFixedObservableAndCenterIndices();
    options.sphereDiameterMm=2.0*sphereRadius;
    options.finiteDifferenceRelativeStep=relativeStep;
    options.huberDeltaMm=0.0; // ideal noiseless design, not observed robust weights
    return profileNormalEquations(observations,state,options,active).h;
}

double acquisitionDistance(const CandidatePose& a, const CandidatePose& b)
{
    const double db = (a.joint.b - b.joint.b) / 15.0;
    const double dc = (a.joint.c - b.joint.c) / 20.0;
    const Vec3d ds = sub(a.targetCenterSensor, b.targetCenterSensor);
    return db * db + dc * dc +
           0.05 * ((ds.x / 5.0) * (ds.x / 5.0) +
                   (ds.y / 5.0) * (ds.y / 5.0) +
                   (ds.z / 10.0) * (ds.z / 10.0));
}


struct BootstrapCircle
{
    JointPose5d joints;
    Vec3d plusCenterSensor;
    double absY = 0.0;
};

std::vector<int> nominalHandEyeSphereIndices()
{
    return {
        static_cast<int>(Physical17Parameters::HandEyeRx),
        static_cast<int>(Physical17Parameters::HandEyeRy),
        static_cast<int>(Physical17Parameters::HandEyeRz),
        static_cast<int>(Physical17Parameters::HandEyeTx),
        static_cast<int>(Physical17Parameters::HandEyeTy),
        static_cast<int>(Physical17Parameters::HandEyeTz),
        static_cast<int>(kTargetCount + 0),
        static_cast<int>(kTargetCount + 1),
        static_cast<int>(kTargetCount + 2)
    };
}

bool solveBootstrapLinearTranslation(
    const std::vector<BootstrapCircle>& circles,
    const std::vector<int>& signs,
    const Vec3d& handEyeRotation,
    Vec3d& handEyeTranslation,
    Vec3d& sphereCenter)
{
    if (circles.size() < 3 || signs.size() != circles.size()) {
        return false;
    }

    DenseMatrix ata(6, 6, 0.0);
    std::vector<double> atb(6, 0.0);

    Physical17Parameters base;
    base[Physical17Parameters::HandEyeRx] = handEyeRotation.x;
    base[Physical17Parameters::HandEyeRy] = handEyeRotation.y;
    base[Physical17Parameters::HandEyeRz] = handEyeRotation.z;

    for (std::size_t i = 0; i < circles.size(); ++i) {
        Vec3d center = circles[i].plusCenterSensor;
        center.y = (signs[i] < 0 ? -1.0 : 1.0) * circles[i].absY;

        const Vec3d p0 = SingleSphere17Model::mapSensorPoint(
            circles[i].joints, center, base);

        Vec3d columns[3];
        for (int axis = 0; axis < 3; ++axis) {
            Physical17Parameters shifted = base;
            shifted[static_cast<std::size_t>(Physical17Parameters::HandEyeTx + axis)] = 1.0;
            columns[axis] = sub(
                SingleSphere17Model::mapSensorPoint(circles[i].joints, center, shifted),
                p0);
        }

        const double rhs[3] = {p0.x, p0.y, p0.z};
        for (int row = 0; row < 3; ++row) {
            // c_M - R_i * t_HE = p0
            double arow[6] = {0,0,0,0,0,0};
            arow[row] = 1.0;
            arow[3] = -(row == 0 ? columns[0].x : row == 1 ? columns[0].y : columns[0].z);
            arow[4] = -(row == 0 ? columns[1].x : row == 1 ? columns[1].y : columns[1].z);
            arow[5] = -(row == 0 ? columns[2].x : row == 1 ? columns[2].y : columns[2].z);
            for (int r = 0; r < 6; ++r) {
                atb[static_cast<std::size_t>(r)] += arow[r] * rhs[row];
                for (int c = r; c < 6; ++c) {
                    ata(r, c) += arow[r] * arow[c];
                }
            }
        }
    }
    for (int r = 0; r < 6; ++r) {
        for (int c = 0; c < r; ++c) ata(r, c) = ata(c, r);
    }

    std::vector<double> x;
    if (!solveDenseLinear(ata, atb, x) || x.size() != 6) return false;
    sphereCenter = Vec3d(x[0], x[1], x[2]);
    handEyeTranslation = Vec3d(x[3], x[4], x[5]);
    return finiteVec(sphereCenter) && finiteVec(handEyeTranslation);
}

double bootstrapCenterRms(const std::vector<BootstrapCircle>& circles,
                          const std::vector<int>& signs,
                          const Physical17Parameters& parameters,
                          const Vec3d& sphereCenter)
{
    if (circles.empty() || signs.size() != circles.size()) return 0.0;
    double sum = 0.0;
    for (std::size_t i = 0; i < circles.size(); ++i) {
        Vec3d center = circles[i].plusCenterSensor;
        center.y = (signs[i] < 0 ? -1.0 : 1.0) * circles[i].absY;
        const Vec3d mapped = SingleSphere17Model::mapSensorPoint(
            circles[i].joints, center, parameters);
        const Vec3d d = sub(mapped, sphereCenter);
        sum += dot(d, d);
    }
    return std::sqrt(sum / static_cast<double>(circles.size()));
}

std::vector<int> inferBootstrapSigns(const std::vector<BootstrapCircle>& circles,
                                     const Physical17Parameters& parameters,
                                     const Vec3d& sphereCenter)
{
    std::vector<int> signs(circles.size(), +1);
    for (std::size_t i = 0; i < circles.size(); ++i) {
        Vec3d plus = circles[i].plusCenterSensor;
        plus.y = circles[i].absY;
        Vec3d minus = plus;
        minus.y = -circles[i].absY;
        const double ep = norm(sub(SingleSphere17Model::mapSensorPoint(
            circles[i].joints, plus, parameters), sphereCenter));
        const double em = norm(sub(SingleSphere17Model::mapSensorPoint(
            circles[i].joints, minus, parameters), sphereCenter));
        signs[i] = ep <= em ? +1 : -1;
    }
    return signs;
}

void bootstrapRankAndCondition(
    const std::vector<SphereProfileObservation>& observations,
    const Physical17SolverOptions& options,
    const Physical17Parameters& parameters,
    const Vec3d& sphereCenter,
    int& rank,
    double& condition,
    std::array<double, 9>& singularValues)
{
    rank = 0;
    condition = std::numeric_limits<double>::infinity();
    singularValues.fill(0.0);
    if (observations.empty()) return;

    const auto active = nominalHandEyeSphereIndices();
    const auto state = packState(parameters, sphereCenter);
    DenseMatrix jac = numericalJacobian(observations, state, options.parameterScales,
                                        0.5 * options.sphereDiameterMm,
                                        options.finiteDifferenceRelativeStep, active);
    const auto residuals = evaluateResiduals(observations, state, 0.5 * options.sphereDiameterMm);
    const auto baseWeights = basePointWeights(observations);
    for (int row = 0; row < jac.rows; ++row) {
        const double base = static_cast<std::size_t>(row) < baseWeights.size()
            ? baseWeights[static_cast<std::size_t>(row)] : 1.0;
        const double robust = huberIrlsWeight(residuals[static_cast<std::size_t>(row)], options.huberDeltaMm);
        const double sw = std::sqrt(std::max(0.0, base * robust));
        for (int col = 0; col < jac.cols; ++col) jac(row, col) *= sw;
    }
    const auto eig = jacobiEigenvalues(normalInformation(jac));
    if (eig.size() != 9) return;
    double largest = 0.0;
    for (std::size_t i = 0; i < 9; ++i) {
        singularValues[i] = std::sqrt(std::max(0.0, eig[i]));
        largest = std::max(largest, singularValues[i]);
    }
    const double threshold = std::max(1e-10, largest * 1e-7);
    double smallest = std::numeric_limits<double>::infinity();
    for (double s : singularValues) {
        if (s > threshold) {
            ++rank;
            smallest = std::min(smallest, s);
        }
    }
    if (std::isfinite(smallest) && smallest > 0.0) condition = largest / smallest;
}

} // namespace

const std::array<const char*, Physical17Parameters::Count>& Physical17Parameters::names()
{
    static const std::array<const char*, Count> kNames{{
        "Sxy", "Sxz", "Syz",
        "C_offset_u", "C_offset_v",
        "C_tilt_x", "C_tilt_y",
        "B_offset_u", "B_offset_v",
        "B_tilt_x", "B_tilt_z",
        "handeye_rx", "handeye_ry", "handeye_rz",
        "handeye_tx", "handeye_ty", "handeye_tz"
    }};
    return kNames;
}

const std::array<const char*, Physical17Parameters::Count>& Physical17Parameters::units()
{
    static const std::array<const char*, Count> kUnits{{
        "rad", "rad", "rad",
        "mm", "mm", "rad", "rad",
        "mm", "mm", "rad", "rad",
        "rad", "rad", "rad",
        "mm", "mm", "mm"
    }};
    return kUnits;
}

Physical17SolverOptions::Physical17SolverOptions()
{
    parameterScales = {{
        1e-4, 1e-4, 1e-4,
        10.0, 10.0,
        1e-4, 1e-4,
        10.0, 10.0,
        1e-4, 1e-4,
        0.02, 0.02, 0.02,
        100.0, 100.0, 100.0,
        100.0, 100.0, 100.0
    }};
}


NominalHandEyeSphereCalibrator::NominalHandEyeSphereCalibrator(
    const Physical17SolverOptions& options)
    : m_options(options)
{
}

void NominalHandEyeSphereCalibrator::clear()
{
    m_observations.clear();
}

void NominalHandEyeSphereCalibrator::addObservation(
    const SphereProfileObservation& observation)
{
    if (!observation.points.empty()) m_observations.push_back(observation);
}

NominalHandEyeSphereResult NominalHandEyeSphereCalibrator::solve() const
{
    NominalHandEyeSphereResult result;
    result.profileCount = m_observations.size();
    for (const auto& obs : m_observations) result.pointCount += obs.points.size();

    if (m_observations.size() < 8) {
        result.message = "Stage-1 bootstrap needs at least eight diverse static XYZCB poses; 12-20 are recommended.";
        return result;
    }

    std::vector<BootstrapCircle> circles;
    circles.reserve(m_observations.size());
    for (const auto& obs : m_observations) {
        const auto fit = SingleSphere17Calibrator::fitSphereCenterInSensor(
            obs.points, m_options.sphereDiameterMm, +1);
        if (!fit.success) continue;
        BootstrapCircle c;
        c.joints = obs.joints;
        c.plusCenterSensor = fit.centerSensorMm;
        c.absY = std::abs(fit.centerSensorMm.y);
        circles.push_back(c);
    }
    if (circles.size() < 8) {
        result.message = "Too few stage-1 profiles have a valid sphere circle fit.";
        return result;
    }

    const std::array<Vec3d, 13> rotationStarts{{
        Vec3d(0.0, 0.0, 0.0),
        Vec3d(+0.10, 0.0, 0.0), Vec3d(-0.10, 0.0, 0.0),
        Vec3d(0.0, +0.10, 0.0), Vec3d(0.0, -0.10, 0.0),
        Vec3d(0.0, 0.0, +0.10), Vec3d(0.0, 0.0, -0.10),
        Vec3d(+0.08, +0.08, 0.0), Vec3d(+0.08, -0.08, 0.0),
        Vec3d(-0.08, +0.08, 0.0), Vec3d(-0.08, -0.08, 0.0),
        Vec3d(+0.06, 0.0, +0.06), Vec3d(-0.06, 0.0, -0.06)
    }};

    double bestCost = std::numeric_limits<double>::infinity();
    std::array<double, kStateCount> bestState{};
    std::vector<int> bestSigns;

    for (const Vec3d& startRotation : rotationStarts) {
        std::vector<int> signs(circles.size(), +1);
        Physical17Parameters p;
        p[Physical17Parameters::HandEyeRx] = startRotation.x;
        p[Physical17Parameters::HandEyeRy] = startRotation.y;
        p[Physical17Parameters::HandEyeRz] = startRotation.z;
        Vec3d center;

        bool initialized = false;
        for (int outer = 0; outer < 4; ++outer) {
            Vec3d t;
            Vec3d r(p[Physical17Parameters::HandEyeRx],
                    p[Physical17Parameters::HandEyeRy],
                    p[Physical17Parameters::HandEyeRz]);
            if (!solveBootstrapLinearTranslation(circles, signs, r, t, center)) break;
            p[Physical17Parameters::HandEyeTx] = t.x;
            p[Physical17Parameters::HandEyeTy] = t.y;
            p[Physical17Parameters::HandEyeTz] = t.z;
            initialized = true;

            auto state = packState(p, center);
            optimizeSubset(m_observations, m_options, nominalHandEyeSphereIndices(), 22, state);
            unpackState(state, p, center);

            const auto inferred = inferBootstrapSigns(circles, p, center);
            if (inferred == signs) {
                signs = inferred;
                break;
            }
            signs = inferred;
        }
        if (!initialized) continue;

        // Rank the multi-start candidates after a short coarse refinement.  The old
        // implementation ran a full 90-iteration numerical-Jacobian refinement for
        // every one of 13 starts; that dominated Stage-1 wall time while rarely changing
        // which basin won.  We now select the basin cheaply, then refine only the winner.
        auto state = packState(p, center);
        const auto residuals = evaluateResiduals(
            m_observations, state, 0.5 * m_options.sphereDiameterMm);
        const double cost = totalRobustCost(
            residuals, basePointWeights(m_observations), m_options.huberDeltaMm);
        if (std::isfinite(cost) && cost < bestCost) {
            bestCost = cost;
            bestState = state;
            Physical17Parameters bp;
            Vec3d bc;
            unpackState(bestState, bp, bc);
            bestSigns = inferBootstrapSigns(circles, bp, bc);
        }
    }

    if (!std::isfinite(bestCost) || bestSigns.empty()) {
        result.message = "Stage-1 nominal hand-eye/sphere optimization did not converge.";
        return result;
    }

    // One accurate refinement of the selected basin.  Capped at 70 iterations because
    // Stage-1 is a navigation/bootstrap solve; the full Physical17 solve remains the
    // final metrology estimate in Stage-2.
    optimizeSubset(m_observations, m_options, nominalHandEyeSphereIndices(),
                   std::max(45, std::min(70, m_options.jointIterations)), bestState);
    {
        Physical17Parameters bp;
        Vec3d bc;
        unpackState(bestState, bp, bc);
        bestSigns = inferBootstrapSigns(circles, bp, bc);
    }

    unpackState(bestState, result.parameters, result.sphereCenterMachineMm);
    // Enforce the intended stage-1 model contract explicitly: all machine
    // geometry parameters remain exactly nominal; only the 6 hand-eye states
    // and the sphere center are estimated.
    for (std::size_t i = 0; i <= Physical17Parameters::BTiltZ; ++i) {
        result.parameters[i] = 0.0;
    }
    result.inferredCenterYSigns = bestSigns;
    result.centerInitRmsMm = bootstrapCenterRms(
        circles, bestSigns, result.parameters, result.sphereCenterMachineMm);

    const auto residuals = evaluateResiduals(
        m_observations, packState(result.parameters, result.sphereCenterMachineMm),
        0.5 * m_options.sphereDiameterMm);
    double sum = 0.0;
    double maxAbs = 0.0;
    for (double r : residuals) {
        sum += r * r;
        maxAbs = std::max(maxAbs, std::abs(r));
    }
    result.residualRmsMm = residuals.empty() ? 0.0
        : std::sqrt(sum / static_cast<double>(residuals.size()));
    result.residualMaxAbsMm = maxAbs;

    bootstrapRankAndCondition(m_observations, m_options,
                              result.parameters, result.sphereCenterMachineMm,
                              result.stateRank, result.conditionNumber,
                              result.singularValues);

    result.success = result.stateRank == result.expectedStateRank
        && std::isfinite(result.residualRmsMm)
        && std::isfinite(result.conditionNumber);
    if (result.success) {
        result.message = "Stage-1 nominal machine solve reached rank 9/9. Hand-eye 6-DoF and fixed sphere center are now a self-consistent bootstrap model for stage-2 planning; per-profile +/-Y was inferred internally and is not an operator input.";
    } else {
        result.message = "Stage-1 solve finished but did not reach rank 9/9. Add poses that independently vary both B and C; XYZ-only motion or one rotary axis alone is insufficient.";
    }
    return result;
}

Vec3d SingleSphere17Model::mapSensorPoint(const JointPose5d& joints,
                                          const Vec3d& sensorPoint,
                                          const Physical17Parameters& parameters)
{
    Vec3d ux;
    Vec3d uy;
    Vec3d uz;
    linearAxisDirections(parameters, ux, uy, uz);
    const AxisLine cLine = cAxisLine(parameters);
    const AxisLine bLine = bAxisLine(parameters);

    Vec3d p = handEyeRotate(parameters, sensorPoint);
    p = add(p, Vec3d(parameters[Physical17Parameters::HandEyeTx],
                     parameters[Physical17Parameters::HandEyeTy],
                     parameters[Physical17Parameters::HandEyeTz]));
    p = rotateAroundLine(p,
                         bLine.point,
                         bLine.direction,
                         kControllerBSign * joints.b * kDegToRad);
    p = rotateAroundLine(p,
                         cLine.point,
                         cLine.direction,
                         kControllerCSign * joints.c * kDegToRad);
    p = add(p, add(add(mul(ux, joints.x), mul(uy, joints.y)), mul(uz, joints.z)));
    return p;
}

std::array<double, 16> SingleSphere17Model::handEyeTransform(
    const Physical17Parameters& parameters)
{
    const Mat3d r = rotationFromVector(
        Vec3d(parameters[Physical17Parameters::HandEyeRx],
              parameters[Physical17Parameters::HandEyeRy],
              parameters[Physical17Parameters::HandEyeRz]));
    const Vec3d t(parameters[Physical17Parameters::HandEyeTx],
                  parameters[Physical17Parameters::HandEyeTy],
                  parameters[Physical17Parameters::HandEyeTz]);
    return {{
        r.v[0][0], r.v[0][1], r.v[0][2], t.x,
        r.v[1][0], r.v[1][1], r.v[1][2], t.y,
        r.v[2][0], r.v[2][1], r.v[2][2], t.z,
        0.0,       0.0,       0.0,       1.0
    }};
}

std::array<double, 16> SingleSphere17Model::sensorToMachineTransform(
    const JointPose5d& joints,
    const Physical17Parameters& parameters)
{
    const ProfileTransform t=profileTransform(PreparedPhysical17(parameters),joints);
    const Mat3d& r=t.rotation;
    const Vec3d& v=t.translation;
    return {{
        r.v[0][0], r.v[0][1], r.v[0][2], v.x,
        r.v[1][0], r.v[1][1], r.v[1][2], v.y,
        r.v[2][0], r.v[2][1], r.v[2][2], v.z,
        0.0, 0.0, 0.0, 1.0
    }};
}

Vec3d SingleSphere17Model::machinePointInSensor(const JointPose5d& joints,
                                                const Vec3d& sphereCenterMachineMm,
                                                const Physical17Parameters& parameters)
{
    const std::array<double, 16> t = sensorToMachineTransform(joints, parameters);
    Mat3d r;
    r.v[0][0] = t[0]; r.v[0][1] = t[1]; r.v[0][2] = t[2];
    r.v[1][0] = t[4]; r.v[1][1] = t[5]; r.v[1][2] = t[6];
    r.v[2][0] = t[8]; r.v[2][1] = t[9]; r.v[2][2] = t[10];
    const Vec3d origin(t[3], t[7], t[11]);
    return multiply(transpose(r), sub(sphereCenterMachineMm, origin));
}

void SingleSphere17Model::applyTranslationGauge(
    const Physical17Parameters& input,
    const Vec3d& sphereCenterMachineMm,
    const Vec3d& deltaMm,
    Physical17Parameters& shiftedParameters,
    Vec3d& shiftedSphereCenterMachineMm)
{
    shiftedParameters = input;
    shiftedSphereCenterMachineMm = add(sphereCenterMachineMm, deltaMm);
    shiftedParameters[Physical17Parameters::HandEyeTx] += deltaMm.x;
    shiftedParameters[Physical17Parameters::HandEyeTy] += deltaMm.y;
    shiftedParameters[Physical17Parameters::HandEyeTz] += deltaMm.z;

    const AxisLine cLine = cAxisLine(input);
    const AxisLine bLine = bAxisLine(input);
    const Vec3d cShifted = add(cLine.point, deltaMm);
    const Vec3d bShifted = add(bLine.point, deltaMm);
    shiftedParameters[Physical17Parameters::COffsetU] = dot(cShifted, cLine.basisU);
    shiftedParameters[Physical17Parameters::COffsetV] = dot(cShifted, cLine.basisV);
    shiftedParameters[Physical17Parameters::BOffsetU] = dot(bShifted, bLine.basisU);
    shiftedParameters[Physical17Parameters::BOffsetV] = dot(bShifted, bLine.basisV);
}

SingleSphere17Calibrator::SingleSphere17Calibrator(const Physical17SolverOptions& options)
    : m_options(options)
{
}

void SingleSphere17Calibrator::clear()
{
    m_observations.clear();
}

void SingleSphere17Calibrator::addObservation(const SphereProfileObservation& observation)
{
    if (!observation.points.empty()) {
        m_observations.push_back(observation);
    }
}

SphereCenterFit SingleSphere17Calibrator::fitSphereCenterInSensor(
    const std::vector<SensorProfilePoint>& profilePoints,
    double sphereDiameterMm,
    int centerYSign,
    const SphereArcRansacOptions& ransacOptions,
    std::vector<std::size_t>* inlierIndices)
{
    SphereCenterFit fit;
    if (inlierIndices) inlierIndices->clear();
    if (!(sphereDiameterMm > 0.0)) {
        fit.message = "Sphere diameter must be positive.";
        return fit;
    }
    if (!(ransacOptions.inlierThresholdMm > 0.0) || ransacOptions.maxIterations <= 0) {
        fit.message = "RANSAC settings are invalid.";
        return fit;
    }

    std::vector<std::size_t> validIndices;
    validIndices.reserve(profilePoints.size());
    for (std::size_t i = 0; i < profilePoints.size(); ++i) {
        const auto& point = profilePoints[i];
        if (std::isfinite(point.x) && std::isfinite(point.z) && point.weight > 0.0) {
            validIndices.push_back(i);
        }
    }
    fit.candidatePointCount = validIndices.size();
    if (validIndices.size() < 5) {
        fit.message = "A sphere ROI requires at least five valid candidate points.";
        return fit;
    }

    const double sphereRadius = 0.5 * sphereDiameterMm;
    const int requiredInliers = std::max(
        std::max(5, ransacOptions.minInliers),
        static_cast<int>(std::ceil(ransacOptions.minInlierRatio * static_cast<double>(validIndices.size()))));

    std::mt19937 rng(ransacOptions.randomSeed ^ static_cast<std::uint32_t>(validIndices.size() * 2654435761u));
    std::uniform_int_distribution<std::size_t> pick(0, validIndices.size() - 1);
    std::vector<std::size_t> bestInliers;
    double bestCenterX = 0.0, bestCenterZ = 0.0, bestRadius = 0.0;
    double bestRms = std::numeric_limits<double>::infinity();

    for (int iteration = 0; iteration < ransacOptions.maxIterations; ++iteration) {
        std::size_t a = pick(rng), b = pick(rng), c = pick(rng);
        if (a == b || a == c || b == c) continue;
        double centerX = 0.0, centerZ = 0.0, radius = 0.0;
        if (!circleFromThreePoints(profilePoints[validIndices[a]],
                                   profilePoints[validIndices[b]],
                                   profilePoints[validIndices[c]],
                                   centerX, centerZ, radius)) {
            continue;
        }
        if (radius > sphereRadius + ransacOptions.maxRadiusOvershootMm) continue;
        double rms = 0.0;
        auto inliers = circleInliers(profilePoints, validIndices, centerX, centerZ, radius,
                                     ransacOptions.inlierThresholdMm, &rms);
        if (inliers.size() > bestInliers.size()
            || (inliers.size() == bestInliers.size() && rms < bestRms)) {
            bestInliers = std::move(inliers);
            bestCenterX = centerX;
            bestCenterZ = centerZ;
            bestRadius = radius;
            bestRms = rms;
        }
    }

    if (static_cast<int>(bestInliers.size()) < requiredInliers) {
        std::ostringstream msg;
        msg << "RANSAC did not find enough circle-arc inliers ("
            << bestInliers.size() << "/" << validIndices.size()
            << ", required " << requiredInliers << ").";
        fit.message = msg.str();
        return fit;
    }

    if (!algebraicCircleFit(profilePoints, bestInliers, bestCenterX, bestCenterZ, bestRadius)) {
        fit.message = "RANSAC inlier circle refinement is singular.";
        return fit;
    }
    refineCircleGeometric(profilePoints, bestInliers, ransacOptions.geometricRefineIterations,
                          bestCenterX, bestCenterZ, bestRadius);

    // Reclassify after refinement and refine once more. This keeps the exported
    // observation limited to the actual spherical arc rather than the ROI.
    bestInliers = circleInliers(profilePoints, validIndices, bestCenterX, bestCenterZ, bestRadius,
                                ransacOptions.inlierThresholdMm, &bestRms);
    if (static_cast<int>(bestInliers.size()) >= requiredInliers
        && algebraicCircleFit(profilePoints, bestInliers, bestCenterX, bestCenterZ, bestRadius)) {
        refineCircleGeometric(profilePoints, bestInliers, ransacOptions.geometricRefineIterations,
                              bestCenterX, bestCenterZ, bestRadius);
        bestInliers = circleInliers(profilePoints, validIndices, bestCenterX, bestCenterZ, bestRadius,
                                    ransacOptions.inlierThresholdMm, &bestRms);
    }

    fit.inlierPointCount = bestInliers.size();
    fit.inlierRatio = validIndices.empty() ? 0.0
                                           : static_cast<double>(bestInliers.size()) / static_cast<double>(validIndices.size());
    fit.arcSpanDeg = circleArcSpanDeg(profilePoints, bestInliers, bestCenterX, bestCenterZ);
    if (static_cast<int>(bestInliers.size()) < requiredInliers) {
        fit.message = "Circle refinement lost too many RANSAC inliers.";
        return fit;
    }
    if (fit.arcSpanDeg < ransacOptions.minArcSpanDeg) {
        std::ostringstream msg;
        msg << "Extracted circle arc is too short (" << fit.arcSpanDeg
            << " deg, required " << ransacOptions.minArcSpanDeg << " deg).";
        fit.message = msg.str();
        return fit;
    }
    if (!(bestRadius > 0.0) || bestRadius > sphereRadius + ransacOptions.maxRadiusOvershootMm) {
        fit.message = "RANSAC circle radius is incompatible with the known sphere.";
        return fit;
    }

    const double ySquared = sphereRadius * sphereRadius - bestRadius * bestRadius;
    if (ySquared < -ransacOptions.maxRadiusOvershootMm * sphereDiameterMm) {
        fit.message = "Fitted profile circle is larger than the known sphere.";
        return fit;
    }

    double squaredError = 0.0;
    for (std::size_t index : bestInliers) {
        const auto& point = profilePoints[index];
        const double error = std::hypot(point.x - bestCenterX, point.z - bestCenterZ) - bestRadius;
        squaredError += error * error;
    }
    fit.circleRmsMm = bestInliers.empty() ? 0.0
        : std::sqrt(squaredError / static_cast<double>(bestInliers.size()));
    const double sign = (centerYSign < 0) ? -1.0 : 1.0;
    fit.centerSensorMm = Vec3d(bestCenterX,
                               sign * std::sqrt(std::max(0.0, ySquared)),
                               bestCenterZ);
    fit.sectionRadiusMm = bestRadius;
    fit.success = finiteVec(fit.centerSensorMm);
    if (inlierIndices) *inlierIndices = bestInliers;
    if (fit.success) {
        std::ostringstream msg;
        msg << "OK (RANSAC " << fit.inlierPointCount << "/" << fit.candidatePointCount
            << ", arc " << fit.arcSpanDeg << " deg)";
        fit.message = msg.str();
    } else {
        fit.message = "Non-finite fitted sphere center.";
    }
    return fit;
}

NominalHandEyeSphereResult SingleSphere17Calibrator::solveNominalHandEyeBootstrap() const
{
    NominalHandEyeSphereCalibrator bootstrap(m_options);
    for (const auto& observation : m_observations) {
        bootstrap.addObservation(observation);
    }
    return bootstrap.solve();
}

SingleSphere17Result SingleSphere17Calibrator::solve(
    const Physical17Parameters& initialParameters,
    const Vec3d* initialSphereCenterMachineMm,
    const Physical17ProgressCallback& progress,
    const std::atomic_bool* cancelRequested) const
{
    SingleSphere17Result result;
    result.gaugeDescription =
        "Full 17-parameter physical model. Single-sphere data identify only 14 "
        "independent physical combinations after eliminating the unknown sphere "
        "center. The supplied hand-eye translation is retained as the 3-D "
        "translation-gauge anchor; it is not claimed as data-identified. B/C "
        "axis-line offsets are estimated relative to that anchor.";

    const auto isCancelled = [&]() {
        return cancelRequested && cancelRequested->load(std::memory_order_relaxed);
    };
    const auto notify = [&](Physical17SolveProgress::Phase phase, int percent,
                            const std::string& message, int startIndex = 0,
                            int startCount = 0, int outerIteration = 0,
                            int outerCount = 0, int innerIteration = 0,
                            int innerCount = 0, double cost = 0.0) {
        if (!progress) return;
        Physical17SolveProgress p;
        p.phase = phase;
        p.percent = std::max(0, std::min(100, percent));
        p.startIndex = startIndex;
        p.startCount = startCount;
        p.outerIteration = outerIteration;
        p.outerCount = outerCount;
        p.innerIteration = innerIteration;
        p.innerCount = innerCount;
        p.robustCost = cost;
        p.message = message;
        progress(p);
    };
    notify(Physical17SolveProgress::Phase::Preparing, 0,
           "Checking Stage-2 observations and preparing the joint state.");

    if (!finiteParameters(initialParameters) ||
        (initialSphereCenterMachineMm && !finiteVec(*initialSphereCenterMachineMm))) {
        result.message="Non-finite initial parameters or sphere center.";
        return result;
    }
    const bool scalesValid=std::all_of(m_options.parameterScales.begin(),m_options.parameterScales.end(),
                                      [](double v) { return std::isfinite(v) && v>0.0; });
    if (!scalesValid || !std::isfinite(m_options.finiteDifferenceRelativeStep) ||
        m_options.finiteDifferenceRelativeStep<=0.0 || !std::isfinite(m_options.huberDeltaMm) ||
        !std::isfinite(m_options.initialLambda) || m_options.initialLambda<=0.0 ||
        !std::isfinite(m_options.convergenceStep) || m_options.convergenceStep<0.0 ||
        !std::isfinite(m_options.convergenceCost) || m_options.convergenceCost<0.0 ||
        !std::isfinite(m_options.convergenceGradient) || m_options.convergenceGradient<0.0 ||
        m_options.multiStartCount<1 || m_options.alternatingIterations<0 ||
        m_options.blockIterations<1 || m_options.jointIterations<1) {
        result.message="Invalid solver scales, tolerances, damping or iteration limits.";
        return result;
    }
    std::size_t positivePoints=0, positiveProfiles=0;
    for (const auto& obs:m_observations) {
        if (!finitePose(obs.joints)) { result.message="Non-finite observation joint pose."; return result; }
        std::size_t used=0;
        for (const auto& point:obs.points) {
            if (!std::isfinite(point.x) || !std::isfinite(point.z) ||
                !std::isfinite(point.weight) || point.weight<0.0) {
                result.message="Non-finite point or invalid negative/non-finite weight.";
                return result;
            }
            if (point.weight>0.0) ++used;
        }
        positivePoints+=used;
        if (used>0) ++positiveProfiles;
    }
    if (positiveProfiles<8 || positivePoints<80) {
        result.message="At least eight profiles and 80 sphere points with positive weight are required.";
        return result;
    }
    if (!std::isfinite(m_options.sphereDiameterMm) || !(m_options.sphereDiameterMm > 0.0)) {
        result.message = "Sphere diameter must be positive.";
        return result;
    }
    if (m_observations.size() < 8) {
        result.message = "Too few profiles. Use at least eight diverse XYZCB poses; 30-50 are recommended.";
        return result;
    }
    std::size_t pointCount = 0;
    for (const auto& obs : m_observations) {
        pointCount += obs.points.size();
    }
    if (pointCount < 80) {
        result.message = "Too few sphere surface points. At least 80 segmented points are required.";
        return result;
    }

    result.profileCount = m_observations.size();
    result.pointCount = pointCount;
    if (isCancelled()) {
        result.message = "Stage-2 calibration cancelled before optimization.";
        return result;
    }

    Vec3d initialCenter;
    if (initialSphereCenterMachineMm && finiteVec(*initialSphereCenterMachineMm)) {
        initialCenter = *initialSphereCenterMachineMm;
    } else {
        // The joint calibration residual has no sphere-side sign.  A single XZ
        // circle fit has a temporary +/-Y ambiguity only for initialization, so
        // evaluate both branches against the complete data set and keep the
        // lower-cost branch automatically.
        const SphereCenterFit fitPlus = fitSphereCenterInSensor(
            m_observations.front().points, m_options.sphereDiameterMm, +1);
        const SphereCenterFit fitMinus = fitSphereCenterInSensor(
            m_observations.front().points, m_options.sphereDiameterMm, -1);
        if (!fitPlus.success && !fitMinus.success) {
            result.message = std::string("Cannot initialize sphere center: ") + fitPlus.message;
            return result;
        }
        std::vector<Vec3d> candidates;
        if (fitPlus.success) {
            candidates.push_back(SingleSphere17Model::mapSensorPoint(
                m_observations.front().joints, fitPlus.centerSensorMm, initialParameters));
        }
        if (fitMinus.success) {
            candidates.push_back(SingleSphere17Model::mapSensorPoint(
                m_observations.front().joints, fitMinus.centerSensorMm, initialParameters));
        }
        double bestInitialCost = std::numeric_limits<double>::infinity();
        for (const Vec3d& candidate : candidates) {
            const auto candidateState = packState(initialParameters, candidate);
            const auto candidateResiduals = evaluateResiduals(
                m_observations, candidateState, 0.5 * m_options.sphereDiameterMm);
            const double candidateCost = totalRobustCost(
                candidateResiduals, basePointWeights(m_observations), m_options.huberDeltaMm);
            if (candidateCost < bestInitialCost) {
                bestInitialCost = candidateCost;
                initialCenter = candidate;
            }
        }
    }

    const std::array<double, kStateCount> base = packState(initialParameters, initialCenter);
    std::array<double, kStateCount> bestState = base;
    double bestCost = std::numeric_limits<double>::infinity();
    int bestAlternatingIterations = 0;
    int bestJointIterations = 0;
    bool bestConverged = false;

    const int starts = std::max(1, m_options.multiStartCount);
    const int outerCount = std::max(0, m_options.alternatingIterations);
    const int blockIterations = std::max(1, m_options.blockIterations);
    const int jointIterations = std::max(1, m_options.jointIterations);
    const int workPerStart = std::max(1, outerCount * 2 * blockIterations + jointIterations);
    const int totalWork = std::max(1, starts * workPerStart);
    int completedWork = 0;

    for (int start = 0; start < starts; ++start) {
        if (isCancelled()) {
            result.message = "Stage-2 calibration cancelled by the operator.";
            return result;
        }
        notify(Physical17SolveProgress::Phase::MultiStart,
               std::min(94, (completedWork * 94) / totalWork),
               "Starting deterministic multi-start candidate.",
               start + 1, starts);
        std::array<double, kStateCount> state = deterministicStart(base, m_options, start);
        int alternatingPerformed = 0;
        for (int outer = 0; outer < outerCount; ++outer) {
            const int machineBase = completedWork;
            const OptimizeResult machine = optimizeSubset(
                m_observations, m_options, machineAndCenterIndices(),
                blockIterations, state,
                [&](int iter, int maxIter, double cost) {
                    const int done = machineBase + iter;
                    notify(Physical17SolveProgress::Phase::AlternatingMachine,
                           std::min(94, (done * 94) / totalWork),
                           "Alternating block: machine geometry + sphere center.",
                           start + 1, starts, outer + 1, outerCount,
                           iter + 1, maxIter, cost);
                    return !isCancelled();
                }, cancelRequested);
            completedWork += blockIterations;
            if (machine.cancelled || isCancelled()) {
                result.message = "Stage-2 calibration cancelled during machine-geometry refinement.";
                return result;
            }

            const int handEyeBase = completedWork;
            const OptimizeResult handEye = optimizeSubset(
                m_observations, m_options, handEyeAndCenterIndices(),
                blockIterations, state,
                [&](int iter, int maxIter, double cost) {
                    const int done = handEyeBase + iter;
                    notify(Physical17SolveProgress::Phase::AlternatingHandEye,
                           std::min(94, (done * 94) / totalWork),
                           "Alternating block: hand-eye rotation + sphere center.",
                           start + 1, starts, outer + 1, outerCount,
                           iter + 1, maxIter, cost);
                    return !isCancelled();
                }, cancelRequested);
            completedWork += blockIterations;
            if (handEye.cancelled || isCancelled()) {
                result.message = "Stage-2 calibration cancelled during hand-eye refinement.";
                return result;
            }
            ++alternatingPerformed;
            if (machine.iterations <= 1 && handEye.iterations <= 1) break;
        }

        const int jointBase = completedWork;
        const OptimizeResult joint = optimizeSubset(
            m_observations, m_options, gaugeFixedObservableAndCenterIndices(),
            jointIterations, state,
            [&](int iter, int maxIter, double cost) {
                const int done = jointBase + iter;
                notify(Physical17SolveProgress::Phase::JointRefinement,
                       std::min(94, (done * 94) / totalWork),
                       "Joint refinement: 14 observable physical combinations + sphere center.",
                       start + 1, starts, 0, 0, iter + 1, maxIter, cost);
                return !isCancelled();
            }, cancelRequested);
        completedWork += jointIterations;
        if (joint.cancelled || isCancelled()) {
            result.message = "Stage-2 calibration cancelled during joint refinement.";
            return result;
        }
        if (joint.cost < bestCost) {
            bestCost = joint.cost;
            bestState = state;
            bestAlternatingIterations = alternatingPerformed;
            bestJointIterations = joint.iterations;
            bestConverged = joint.converged;
        }
    }

    unpackState(bestState, result.parameters, result.sphereCenterMachineMm);
    const std::vector<double> residuals = evaluateResiduals(
        m_observations, bestState, 0.5 * m_options.sphereDiameterMm);
    double squared = 0.0;
    double maximum = 0.0;
    for (double value : residuals) {
        squared += value * value;
        maximum = std::max(maximum, std::abs(value));
    }
    result.residualRmsMm = residuals.empty()
                               ? 0.0
                               : std::sqrt(squared / static_cast<double>(residuals.size()));
    result.residualMaxAbsMm = maximum;
    result.alternatingIterations = bestAlternatingIterations;
    result.jointIterations = bestJointIterations;
    result.optimizationConverged = bestConverged;
    result.robustCost = bestCost;
    result.profileCount = m_observations.size();
    result.pointCount = pointCount;
    notify(Physical17SolveProgress::Phase::Observability, 96,
           "Optimization finished; computing numerical rank and condition number.");
    if (isCancelled()) {
        result.message = "Stage-2 calibration cancelled before observability analysis.";
        return result;
    }
    result.observability = analyzeObservability(result.parameters,
                                                result.sphereCenterMachineMm);
    result.success = result.observability.targetRank >=
                         result.observability.expectedTargetRank &&
                     result.observability.fullStateRank >=
                         result.observability.expectedFullStateRank &&
                     std::isfinite(result.residualRmsMm);
    if (result.success) {
        if (result.observability.targetRank > result.observability.expectedTargetRank) {
            result.message = "Optimization completed with sufficient rank. Numerical finite-difference leakage made the reported physical rank exceed the theoretical single-sphere rank 14; the 3-D translation gauge remains externally anchored.";
        } else {
            result.message = "17 physical parameters are retained; the sphere data provide rank 14/17. Hand-eye translation is the explicit 3-D gauge anchor, while the remaining 14 independent combinations are data-identified.";
        }
    } else {
        result.message = "Optimization completed, but the data do not reach at least physical rank 14/17 and full-state rank 17/20.";
    }
    if (!bestConverged) {
        result.message += " Joint iteration budget/damping limit reached; stationarity was NOT confirmed. Inspect independent validation or increase jointIterations.";
    }
    notify(Physical17SolveProgress::Phase::Finished, 100, result.message);
    return result;
}

Physical17ObservabilityReport SingleSphere17Calibrator::analyzeObservability(
    const Physical17Parameters& parameters,
    const Vec3d& sphereCenterMachineMm) const
{
    Physical17ObservabilityReport report;
    if (m_observations.empty()) {
        report.message = "No observations.";
        return report;
    }

    if (!finiteParameters(parameters) || !finiteVec(sphereCenterMachineMm) ||
        !std::isfinite(m_options.sphereDiameterMm) || m_options.sphereDiameterMm<=0.0 ||
        !std::isfinite(m_options.huberDeltaMm) ||
        !std::isfinite(m_options.finiteDifferenceRelativeStep) || m_options.finiteDifferenceRelativeStep<=0.0 ||
        !std::all_of(m_options.parameterScales.begin(),m_options.parameterScales.end(),
                     [](double v) { return std::isfinite(v) && v>0.0; })) {
        report.message="Invalid model or solver options for observability analysis.";
        return report;
    }
    for (const auto& obs:m_observations) {
        if (!finitePose(obs.joints)) { report.message="Non-finite observation pose."; return report; }
        for (const auto& point:obs.points)
            if (!std::isfinite(point.x) || !std::isfinite(point.z) || !std::isfinite(point.weight) || point.weight<0.0) {
                report.message="Invalid point/weight for observability analysis."; return report;
            }
    }
    const std::array<double, kStateCount> state = packState(parameters, sphereCenterMachineMm);
    const DenseMatrix fullInformation=profileNormalEquations(
        m_observations, state, m_options, fullIndices()).h;
    DenseMatrix htt(static_cast<int>(kTargetCount), static_cast<int>(kTargetCount));
    DenseMatrix htn(static_cast<int>(kTargetCount), 3);
    DenseMatrix hnn(3, 3);
    for (int i=0;i<static_cast<int>(kTargetCount);++i) {
        for (int j=0;j<static_cast<int>(kTargetCount);++j) htt(i,j)=fullInformation(i,j);
        for (int j=0;j<3;++j) htn(i,j)=fullInformation(i,static_cast<int>(kTargetCount)+j);
    }
    for (int i=0;i<3;++i)
        for (int j=0;j<3;++j) hnn(i,j)=fullInformation(static_cast<int>(kTargetCount)+i,static_cast<int>(kTargetCount)+j);

    DenseMatrix hnnInverse;
    if (!invert3(hnn, hnnInverse)) {
        report.message = "Sphere-center nuisance block is singular.";
        return report;
    }

    DenseMatrix effective = htt;
    for (int r = 0; r < static_cast<int>(kTargetCount); ++r) {
        for (int c = 0; c < static_cast<int>(kTargetCount); ++c) {
            double correction = 0.0;
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) {
                    correction += htn(r, i) * hnnInverse(i, j) * htn(c, j);
                }
            }
            effective(r, c) -= correction;
        }
    }

    const std::vector<double> eigenvalues = jacobiEigenvalues(translationGaugeComplementInformation(effective,parameters,m_options.parameterScales));
    if (eigenvalues.size() != kIdentifiableCount) {
        report.message = "Eigenvalue computation failed.";
        return report;
    }
    double largest = 0.0;
    for (std::size_t i = 0; i < kIdentifiableCount; ++i) {
        const double singular = std::sqrt(std::max(0.0, eigenvalues[i]));
        report.targetSingularValues[i] = singular;
        largest = std::max(largest, singular);
    }
    const double threshold = std::max(1e-10, largest * 1e-7);
    int rank = 0;
    double smallestNonzero = std::numeric_limits<double>::infinity();
    for (double singular : report.targetSingularValues) {
        if (singular > threshold) {
            ++rank;
            smallestNonzero = std::min(smallestNonzero, singular);
        }
    }
    report.targetRank = rank;
    report.largestSingularValue = largest;
    report.smallestSingularValue = std::isfinite(smallestNonzero) ? smallestNonzero : 0.0;
    report.conditionNumber = (report.smallestSingularValue > 0.0)
                                 ? report.largestSingularValue / report.smallestSingularValue
                                 : std::numeric_limits<double>::infinity();

    const std::vector<double> fullEigenvalues = jacobiEigenvalues(translationGaugeComplementInformation(fullInformation,parameters,m_options.parameterScales));
    if (fullEigenvalues.size() == kObservableStateCount) {
        double fullLargest = 0.0;
        for (std::size_t i = 0; i < kObservableStateCount; ++i) {
            const double singular = std::sqrt(std::max(0.0, fullEigenvalues[i]));
            report.fullStateSingularValues[i] = singular;
            fullLargest = std::max(fullLargest, singular);
        }
        const double fullThreshold = std::max(1e-10, fullLargest * 1e-7);
        for (double singular : report.fullStateSingularValues) {
            if (singular > fullThreshold) ++report.fullStateRank;
        }
    }

    std::ostringstream message;
    message << "Physical target rank " << rank << "/" << kTargetCount
            << " (expected data-identifiable rank " << report.expectedTargetRank << ")"
            << ", full data-state rank " << report.fullStateRank << "/" << kStateCount
            << " (expected " << report.expectedFullStateRank << ")"
            << "; exact translation gauge projected out, nonzero condition number " << report.conditionNumber << ".";
    report.message = message.str();
    return report;
}

Physical17ValidationReport SingleSphere17Calibrator::validateModel(
    const std::vector<SphereProfileObservation>& observations,
    const Physical17Parameters& parameters,
    const Vec3d& sphereCenterMachineMm,
    double sphereDiameterMm)
{
    Physical17ValidationReport report;
    if (observations.empty()) {
        report.message = "No independent validation observations.";
        return report;
    }
    if (!std::isfinite(sphereDiameterMm) || sphereDiameterMm <= 0.0) {
        report.message = "Invalid sphere diameter.";
        return report;
    }

    if (!finiteParameters(parameters) || !finiteVec(sphereCenterMachineMm)) {
        report.message="Non-finite validation model or sphere center.";
        return report;
    }
    const double radius = 0.5 * sphereDiameterMm;
    double sum = 0.0;
    double sumAbs = 0.0;
    double sumSq = 0.0;
    double maxAbsResidual = 0.0;
    std::size_t pointCount = 0;

    const PreparedPhysical17 prepared(parameters);
    for (const auto& observation : observations) {
        if (observation.points.empty()) continue;
        ++report.profileCount;
        const Vec3d center=centerInSensor(profileTransform(prepared,observation.joints),sphereCenterMachineMm);
        for (const auto& point : observation.points) {
            const double dx=point.x-center.x;
            const double dy=-center.y;
            const double dz=point.z-center.z;
            const double residual = std::sqrt(dx*dx + dy*dy + dz*dz) - radius;
            if (!std::isfinite(residual)) {
                report.message = "Non-finite validation residual.";
                return report;
            }
            sum += residual;
            sumAbs += std::abs(residual);
            sumSq += residual * residual;
            maxAbsResidual = std::max(maxAbsResidual, std::abs(residual));
            ++pointCount;
        }
    }

    report.pointCount = pointCount;
    if (pointCount == 0) {
        report.message = "Independent validation set contains no sphere-surface points.";
        return report;
    }

    const double n = static_cast<double>(pointCount);
    report.residualMeanMm = sum / n;
    report.residualMeanAbsMm = sumAbs / n;
    report.residualRmsMm = std::sqrt(sumSq / n);
    report.residualMaxAbsMm = maxAbsResidual;
    report.valid = std::isfinite(report.residualMeanMm)
        && std::isfinite(report.residualMeanAbsMm)
        && std::isfinite(report.residualRmsMm)
        && std::isfinite(report.residualMaxAbsMm);

    std::ostringstream message;
    message << "Independent validation: profiles=" << report.profileCount
            << ", points=" << report.pointCount
            << ", mean=" << report.residualMeanMm << " mm"
            << ", MAE=" << report.residualMeanAbsMm << " mm"
            << ", RMS=" << report.residualRmsMm << " mm"
            << ", max=" << report.residualMaxAbsMm << " mm.";
    report.message = message.str();
    return report;
}


Physical17PoseScreeningReport SingleSphere17Calibrator::screenPoseOutliers(
    const std::vector<SphereProfileObservation>& observations,
    const Physical17Parameters& parameters,
    const Vec3d& sphereCenterMachineMm,
    double sphereDiameterMm,
    const Physical17PoseScreeningOptions& options)
{
    Physical17PoseScreeningReport report;
    if (observations.empty()) {
        report.message = "No Stage-2 observations are available for pose screening.";
        return report;
    }
    if (!finiteParameters(parameters) || !finiteVec(sphereCenterMachineMm) ||
        !std::isfinite(sphereDiameterMm) || sphereDiameterMm <= 0.0 ||
        !std::isfinite(options.poseLinearToleranceMm) || options.poseLinearToleranceMm <= 0.0 ||
        !std::isfinite(options.poseAngularToleranceDeg) || options.poseAngularToleranceDeg <= 0.0 ||
        !std::isfinite(options.maxRejectedFraction) || options.maxRejectedFraction < 0.0 ||
        options.maxRejectedFraction >= 1.0 ||
        !std::isfinite(options.madSigma) || options.madSigma <= 0.0 ||
        !std::isfinite(options.minRobustSigmaMm) || options.minRobustSigmaMm <= 0.0 ||
        !std::isfinite(options.targetTrainingRmsMm) || options.targetTrainingRmsMm <= 0.0 ||
        !std::isfinite(options.progressiveTrimFraction) || options.progressiveTrimFraction < 0.0 ||
        options.progressiveTrimFraction >= 1.0 ||
        options.minProgressiveTrimCount < 0 || options.maxProgressiveTrimCount < 1 ||
        options.minProgressiveTrimCount > options.maxProgressiveTrimCount ||
        !std::isfinite(options.minProgressiveSpreadMm) || options.minProgressiveSpreadMm < 0.0 ||
        !std::isfinite(options.progressiveTailFactor) || options.progressiveTailFactor < 1.0 ||
        !std::isfinite(options.minLinearSpanFraction) || options.minLinearSpanFraction < 0.0 ||
        options.minLinearSpanFraction > 1.0 ||
        !std::isfinite(options.minRotarySpanFraction) || options.minRotarySpanFraction < 0.0 ||
        options.minRotarySpanFraction > 1.0) {
        report.message = "Invalid Physical17 pose-screening model/options.";
        return report;
    }

    struct PoseGroup {
        JointPose5d pose;
        std::vector<const SphereProfileObservation*> observations;
    };
    const auto samePose = [&](const JointPose5d& a, const JointPose5d& b) {
        return std::abs(a.x-b.x) <= options.poseLinearToleranceMm &&
               std::abs(a.y-b.y) <= options.poseLinearToleranceMm &&
               std::abs(a.z-b.z) <= options.poseLinearToleranceMm &&
               std::abs(a.c-b.c) <= options.poseAngularToleranceDeg &&
               std::abs(a.b-b.b) <= options.poseAngularToleranceDeg;
    };

    std::vector<PoseGroup> groups;
    for (const auto& observation : observations) {
        if (observation.points.empty()) continue;
        bool found = false;
        for (auto& group : groups) {
            if (samePose(observation.joints, group.pose)) {
                group.observations.push_back(&observation);
                found = true;
                break;
            }
        }
        if (!found) groups.push_back(PoseGroup{observation.joints, {&observation}});
    }
    report.inputPoseCount = groups.size();
    if (groups.empty()) {
        report.message = "Stage-2 observations contain no non-empty pose groups.";
        return report;
    }

    const double radius = 0.5 * sphereDiameterMm;
    const PreparedPhysical17 prepared(parameters);
    double aggregateSumSq = 0.0;
    std::size_t aggregateCount = 0;
    report.poses.reserve(groups.size());
    for (const auto& group : groups) {
        Physical17PoseResidualSummary summary;
        summary.pose = group.pose;
        summary.profileCount = group.observations.size();
        std::vector<double> absoluteResiduals;
        double sumAbs = 0.0;
        double sumSq = 0.0;
        double maxAbs = 0.0;
        for (const auto* observation : group.observations) {
            const Vec3d center = centerInSensor(
                profileTransform(prepared, observation->joints), sphereCenterMachineMm);
            for (const auto& point : observation->points) {
                if (!std::isfinite(point.x) || !std::isfinite(point.z) ||
                    !std::isfinite(point.weight) || point.weight <= 0.0) continue;
                const double dx = point.x - center.x;
                const double dy = -center.y;
                const double dz = point.z - center.z;
                const double residual = std::sqrt(dx*dx + dy*dy + dz*dz) - radius;
                if (!std::isfinite(residual)) continue;
                const double a = std::abs(residual);
                sumAbs += a;
                sumSq += residual * residual;
                aggregateSumSq += residual * residual;
                ++aggregateCount;
                maxAbs = std::max(maxAbs, a);
                absoluteResiduals.push_back(a);
            }
        }
        summary.pointCount = absoluteResiduals.size();
        if (!absoluteResiduals.empty()) {
            const double n = static_cast<double>(absoluteResiduals.size());
            summary.residualMeanAbsMm = sumAbs / n;
            summary.residualRmsMm = std::sqrt(sumSq / n);
            summary.residualMaxAbsMm = maxAbs;
            std::sort(absoluteResiduals.begin(), absoluteResiduals.end());
            const double p = 0.95 * static_cast<double>(absoluteResiduals.size()-1);
            const std::size_t lo = static_cast<std::size_t>(std::floor(p));
            const std::size_t hi = static_cast<std::size_t>(std::ceil(p));
            const double t = p - static_cast<double>(lo);
            summary.residualP95AbsMm = absoluteResiduals[lo] * (1.0-t) + absoluteResiduals[hi] * t;
        } else {
            summary.residualRmsMm = std::numeric_limits<double>::infinity();
            summary.residualP95AbsMm = std::numeric_limits<double>::infinity();
            summary.residualMaxAbsMm = std::numeric_limits<double>::infinity();
        }
        report.poses.push_back(summary);
    }
    report.aggregateRmsMm = aggregateCount > 0
        ? std::sqrt(aggregateSumSq / static_cast<double>(aggregateCount))
        : std::numeric_limits<double>::infinity();

    const auto medianOf = [](std::vector<double> values) {
        values.erase(std::remove_if(values.begin(), values.end(),
                    [](double v){ return !std::isfinite(v); }), values.end());
        if (values.empty()) return std::numeric_limits<double>::infinity();
        std::sort(values.begin(), values.end());
        const std::size_t mid = values.size()/2;
        return (values.size()%2) ? values[mid] : 0.5*(values[mid-1]+values[mid]);
    };
    std::vector<double> rmsValues, p95Values;
    rmsValues.reserve(report.poses.size());
    p95Values.reserve(report.poses.size());
    for (const auto& pose : report.poses) {
        rmsValues.push_back(pose.residualRmsMm);
        p95Values.push_back(pose.residualP95AbsMm);
    }
    report.medianRmsMm = medianOf(rmsValues);
    report.medianP95AbsMm = medianOf(p95Values);
    if (!std::isfinite(report.medianRmsMm) || !std::isfinite(report.medianP95AbsMm)) {
        report.message = "Pose screening could not compute finite residual statistics.";
        return report;
    }
    std::vector<double> rmsDeviation, p95Deviation;
    rmsDeviation.reserve(report.poses.size());
    p95Deviation.reserve(report.poses.size());
    for (const auto& pose : report.poses) {
        if (std::isfinite(pose.residualRmsMm))
            rmsDeviation.push_back(std::abs(pose.residualRmsMm-report.medianRmsMm));
        if (std::isfinite(pose.residualP95AbsMm))
            p95Deviation.push_back(std::abs(pose.residualP95AbsMm-report.medianP95AbsMm));
    }
    report.madRmsMm = medianOf(rmsDeviation);
    report.madP95AbsMm = medianOf(p95Deviation);
    const double sigmaRms = std::max(options.minRobustSigmaMm, 1.4826 * report.madRmsMm);
    const double sigmaP95 = std::max(options.minRobustSigmaMm, 1.4826 * report.madP95AbsMm);

    // V27.6.41: use the stricter of a relative quality gate and a robust
    // median/MAD gate.  In V27.6.40 the maximum of these gates was used, so a
    // contaminated high-residual tail enlarged MAD and could make the final
    // threshold very permissive (classic outlier masking).
    const double rmsRobustGate = report.medianRmsMm + options.madSigma * sigmaRms;
    const double p95RobustGate = report.medianP95AbsMm + options.madSigma * sigmaP95;
    const double rmsRelativeGate = options.relativeRmsFactor * report.medianRmsMm;
    const double p95RelativeGate = options.relativeP95Factor * report.medianP95AbsMm;
    report.rmsThresholdMm = std::max(options.absoluteRmsFloorMm,
                                     std::min(rmsRobustGate, rmsRelativeGate));
    report.p95ThresholdMm = std::max(options.absoluteP95FloorMm,
                                     std::min(p95RobustGate, p95RelativeGate));

    struct Coverage {
        double minX = std::numeric_limits<double>::infinity();
        double maxX = -std::numeric_limits<double>::infinity();
        double minY = std::numeric_limits<double>::infinity();
        double maxY = -std::numeric_limits<double>::infinity();
        double minZ = std::numeric_limits<double>::infinity();
        double maxZ = -std::numeric_limits<double>::infinity();
        double minB = std::numeric_limits<double>::infinity();
        double maxB = -std::numeric_limits<double>::infinity();
        double minC = std::numeric_limits<double>::infinity();
        double maxC = -std::numeric_limits<double>::infinity();
        int bPos = 0, bNeg = 0, cPos = 0, cNeg = 0, count = 0;
    };
    const auto coverage = [&](const std::vector<bool>& rejected) {
        Coverage c;
        for (std::size_t i=0; i<report.poses.size(); ++i) {
            if (i < rejected.size() && rejected[i]) continue;
            const auto& q = report.poses[i].pose;
            c.minX = std::min(c.minX, q.x); c.maxX = std::max(c.maxX, q.x);
            c.minY = std::min(c.minY, q.y); c.maxY = std::max(c.maxY, q.y);
            c.minZ = std::min(c.minZ, q.z); c.maxZ = std::max(c.maxZ, q.z);
            c.minB = std::min(c.minB, q.b); c.maxB = std::max(c.maxB, q.b);
            c.minC = std::min(c.minC, q.c); c.maxC = std::max(c.maxC, q.c);
            if (q.b > options.signMarginDeg) ++c.bPos;
            if (q.b < -options.signMarginDeg) ++c.bNeg;
            if (q.c > options.signMarginDeg) ++c.cPos;
            if (q.c < -options.signMarginDeg) ++c.cNeg;
            ++c.count;
        }
        return c;
    };
    const std::vector<bool> noneRejected(report.poses.size(), false);
    const Coverage original = coverage(noneRejected);
    const double originalXSpan = original.count > 0 ? original.maxX-original.minX : 0.0;
    const double originalYSpan = original.count > 0 ? original.maxY-original.minY : 0.0;
    const double originalZSpan = original.count > 0 ? original.maxZ-original.minZ : 0.0;
    const double originalBSpan = original.count > 0 ? original.maxB-original.minB : 0.0;
    const double originalCSpan = original.count > 0 ? original.maxC-original.minC : 0.0;
    const auto preservesCoverage = [&](const std::vector<bool>& rejected) {
        const Coverage c = coverage(rejected);
        if (c.count < std::max(8, options.minRetainedPoseCount)) return false;
        if (originalXSpan > 1.0 && (c.maxX-c.minX) + 1e-12 < options.minLinearSpanFraction*originalXSpan)
            return false;
        if (originalYSpan > 1.0 && (c.maxY-c.minY) + 1e-12 < options.minLinearSpanFraction*originalYSpan)
            return false;
        if (originalZSpan > 1.0 && (c.maxZ-c.minZ) + 1e-12 < options.minLinearSpanFraction*originalZSpan)
            return false;
        if (originalBSpan > 1.0 && (c.maxB-c.minB) + 1e-12 < options.minRotarySpanFraction*originalBSpan)
            return false;
        if (originalCSpan > 1.0 && (c.maxC-c.minC) + 1e-12 < options.minRotarySpanFraction*originalCSpan)
            return false;
        const int need = std::max(1, options.minPositiveNegativePoseCount);
        if (original.bPos >= need && original.bNeg >= need && (c.bPos < need || c.bNeg < need)) return false;
        if (original.cPos >= need && original.cNeg >= need && (c.cPos < need || c.cNeg < need)) return false;
        return true;
    };

    std::vector<std::size_t> candidates;
    std::vector<std::size_t> ranked(report.poses.size());
    std::iota(ranked.begin(), ranked.end(), std::size_t{0});
    for (std::size_t i=0; i<report.poses.size(); ++i) {
        auto& pose = report.poses[i];
        const bool rmsBad = !std::isfinite(pose.residualRmsMm) ||
                            pose.residualRmsMm > report.rmsThresholdMm;
        const bool p95Bad = !std::isfinite(pose.residualP95AbsMm) ||
                            pose.residualP95AbsMm > report.p95ThresholdMm;
        const double rmsRatio = pose.residualRmsMm / std::max(1e-12, report.rmsThresholdMm);
        const double p95Ratio = pose.residualP95AbsMm / std::max(1e-12, report.p95ThresholdMm);
        const double rmsTail = std::max(0.0,
            (pose.residualRmsMm-report.medianRmsMm) / std::max(1e-12, sigmaRms));
        const double p95Tail = std::max(0.0,
            (pose.residualP95AbsMm-report.medianP95AbsMm) / std::max(1e-12, sigmaP95));
        pose.rejectionScore = 0.55*std::max(rmsRatio, p95Ratio) +
                              0.30*rmsTail + 0.15*p95Tail;
        if (rmsBad || p95Bad) candidates.push_back(i);
    }
    report.statisticalCandidateCount = candidates.size();

    const auto worsePose = [&](std::size_t a, std::size_t b) {
        if (report.poses[a].rejectionScore != report.poses[b].rejectionScore)
            return report.poses[a].rejectionScore > report.poses[b].rejectionScore;
        if (report.poses[a].residualRmsMm != report.poses[b].residualRmsMm)
            return report.poses[a].residualRmsMm > report.poses[b].residualRmsMm;
        return report.poses[a].residualP95AbsMm > report.poses[b].residualP95AbsMm;
    };
    std::sort(ranked.begin(), ranked.end(), worsePose);
    std::sort(candidates.begin(), candidates.end(), worsePose);

    // Progressive least-trimmed tail: when the aggregate model is still above
    // the target and the pose RMS distribution has enough spread to indicate
    // heterogeneous pose quality, ensure that a small worst-tail batch is
    // actually tested.  This solves the V27.6.40 failure mode where only one
    // pose exceeded an inflated MAD threshold.
    if (options.enableProgressiveTrim &&
        std::isfinite(report.aggregateRmsMm) &&
        report.aggregateRmsMm > options.targetTrainingRmsMm &&
        sigmaRms >= options.minProgressiveSpreadMm &&
        !ranked.empty()) {
        const std::size_t desired = std::min<std::size_t>(
            static_cast<std::size_t>(std::max(1, options.maxProgressiveTrimCount)),
            std::max<std::size_t>(
                static_cast<std::size_t>(std::max(1, options.minProgressiveTrimCount)),
                static_cast<std::size_t>(std::ceil(
                    options.progressiveTrimFraction * static_cast<double>(report.poses.size())))));
        std::vector<bool> already(report.poses.size(), false);
        for (std::size_t i : candidates) if (i < already.size()) already[i] = true;
        const double rmsTailFloor = std::max(options.absoluteRmsFloorMm,
                                             options.progressiveTailFactor*report.medianRmsMm);
        const double p95TailFloor = std::max(options.absoluteP95FloorMm,
                                             options.progressiveTailFactor*report.medianP95AbsMm);
        for (std::size_t i : ranked) {
            if (candidates.size() >= desired) break;
            if (already[i]) continue;
            const auto& pose = report.poses[i];
            if (pose.residualRmsMm < rmsTailFloor &&
                pose.residualP95AbsMm < p95TailFloor) continue;
            candidates.push_back(i);
            already[i] = true;
            ++report.progressiveCandidateCount;
        }
        std::sort(candidates.begin(), candidates.end(), worsePose);
    }

    const std::size_t minimumRetained = std::min<std::size_t>(
        report.poses.size(), static_cast<std::size_t>(std::max(8, options.minRetainedPoseCount)));
    const std::size_t fractionLimit = static_cast<std::size_t>(std::floor(
        options.maxRejectedFraction * static_cast<double>(report.poses.size()) + 1e-12));
    const std::size_t countLimit = report.poses.size() > minimumRetained
        ? report.poses.size()-minimumRetained : 0;
    const std::size_t maxReject = std::min(fractionLimit, countLimit);

    std::vector<bool> rejected(report.poses.size(), false);
    std::size_t rejectedCount = 0;
    for (std::size_t index : candidates) {
        if (rejectedCount >= maxReject) break;
        rejected[index] = true;
        if (!preservesCoverage(rejected)) {
            rejected[index] = false;
            continue;
        }
        report.poses[index].rejected = true;
        ++rejectedCount;
    }

    report.rejectedPoseCount = rejectedCount;
    report.retainedPoseCount = report.poses.size()-rejectedCount;
    report.valid = true;
    std::ostringstream message;
    message << "Pose screening: input=" << report.inputPoseCount
            << ", retained=" << report.retainedPoseCount
            << ", rejected=" << report.rejectedPoseCount
            << "; aggregate RMS=" << report.aggregateRmsMm
            << " mm; RMS median=" << report.medianRmsMm
            << " mm, MAD=" << report.madRmsMm
            << " mm, threshold=" << report.rmsThresholdMm
            << " mm; P95 median=" << report.medianP95AbsMm
            << " mm, threshold=" << report.p95ThresholdMm
            << " mm; statistical candidates=" << report.statisticalCandidateCount
            << ", progressive additions=" << report.progressiveCandidateCount
            << ". Per-round rejection is capped at " << (100.0*options.maxRejectedFraction)
            << "% and preserves XYZ span plus B/C sign/span excitation.";
    report.message = message.str();
    return report;
}

bool SingleSphere17Calibrator::readObservationCsv(
    std::istream& input,
    std::vector<SphereProfileObservation>& observations,
    std::string& error)
{
    observations.clear();
    std::string headerLine;
    if (!std::getline(input, headerLine)) {
        error = "CSV is empty.";
        return false;
    }
    const std::vector<std::string> headers = splitCsvLine(headerLine);
    std::unordered_map<std::string, int> column;
    for (std::size_t i = 0; i < headers.size(); ++i) {
        column[headers[i]] = static_cast<int>(i);
    }
    const char* required[] = {
        "profile_id", "sensor_x_mm", "sensor_z_mm"
    };
    for (const char* name : required) {
        if (column.find(name) == column.end()) {
            error = std::string("Missing required CSV column: ") + name;
            return false;
        }
    }
    const bool hasLegacyPose =
        column.find("x_mm") != column.end() && column.find("y_mm") != column.end() &&
        column.find("z_mm") != column.end() && column.find("c_deg") != column.end() &&
        column.find("b_deg") != column.end();
    const bool hasExplicitMpos =
        column.find("mpos_x_mm") != column.end() && column.find("mpos_y_mm") != column.end() &&
        column.find("mpos_z_mm") != column.end() && column.find("mpos_c_deg") != column.end() &&
        column.find("mpos_b_deg") != column.end();
    if (!hasLegacyPose && !hasExplicitMpos) {
        error = "CSV must contain either legacy x_mm/y_mm/z_mm/c_deg/b_deg columns or explicit mpos_* pose columns.";
        return false;
    }

    const bool anyDpos =
        column.find("dpos_x_mm") != column.end() || column.find("dpos_y_mm") != column.end() ||
        column.find("dpos_z_mm") != column.end() || column.find("dpos_c_deg") != column.end() ||
        column.find("dpos_b_deg") != column.end();
    const bool hasDpos =
        column.find("dpos_x_mm") != column.end() && column.find("dpos_y_mm") != column.end() &&
        column.find("dpos_z_mm") != column.end() && column.find("dpos_c_deg") != column.end() &&
        column.find("dpos_b_deg") != column.end();
    if (anyDpos && !hasDpos) {
        error = "DPOS CSV columns are incomplete; dpos_x/y/z/c/b must be present together.";
        return false;
    }
    const int weightColumn = (column.find("weight") != column.end())
                                 ? column["weight"]
                                 : -1;

    std::map<std::int64_t, std::size_t> lookup;
    std::string line;
    int lineNumber = 1;
    while (std::getline(input, line)) {
        ++lineNumber;
        if (trim(line).empty() || trim(line)[0] == '#') {
            continue;
        }
        const std::vector<std::string> fields = splitCsvLine(line);
        auto field = [&](const char* name) -> std::string {
            const auto it = column.find(name);
            if (it == column.end()) return std::string();
            const int index = it->second;
            return (index >= 0 && index < static_cast<int>(fields.size()))
                       ? fields[static_cast<std::size_t>(index)]
                       : std::string();
        };

        std::int64_t profileId = 0;
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double c = 0.0;
        double b = 0.0;
        double sensorX = 0.0;
        double sensorZ = 0.0;
        const char* xName = hasExplicitMpos ? "mpos_x_mm" : "x_mm";
        const char* yName = hasExplicitMpos ? "mpos_y_mm" : "y_mm";
        const char* zName = hasExplicitMpos ? "mpos_z_mm" : "z_mm";
        const char* cName = hasExplicitMpos ? "mpos_c_deg" : "c_deg";
        const char* bName = hasExplicitMpos ? "mpos_b_deg" : "b_deg";
        if (!parseInt64(field("profile_id"), profileId) ||
            !parseDouble(field(xName), x) ||
            !parseDouble(field(yName), y) ||
            !parseDouble(field(zName), z) ||
            !parseDouble(field(cName), c) ||
            !parseDouble(field(bName), b) ||
            !parseDouble(field("sensor_x_mm"), sensorX) ||
            !parseDouble(field("sensor_z_mm"), sensorZ)) {
            std::ostringstream message;
            message << "Invalid numeric field at CSV line " << lineNumber << ".";
            error = message.str();
            return false;
        }
        if (hasLegacyPose && hasExplicitMpos) {
            double legacyX=0.0, legacyY=0.0, legacyZ=0.0, legacyC=0.0, legacyB=0.0;
            if (!parseDouble(field("x_mm"), legacyX) || !parseDouble(field("y_mm"), legacyY) ||
                !parseDouble(field("z_mm"), legacyZ) || !parseDouble(field("c_deg"), legacyC) ||
                !parseDouble(field("b_deg"), legacyB) ||
                std::abs(legacyX-x)>1e-8 || std::abs(legacyY-y)>1e-8 ||
                std::abs(legacyZ-z)>1e-8 || std::abs(legacyC-c)>1e-8 || std::abs(legacyB-b)>1e-8) {
                error = "Legacy pose columns and explicit MPOS columns disagree at CSV line " + std::to_string(lineNumber) + ".";
                return false;
            }
        }
        JointPose5d dpos;
        bool rowHasDpos = false;
        if (hasDpos) {
            const std::string dx = trim(field("dpos_x_mm"));
            const std::string dy = trim(field("dpos_y_mm"));
            const std::string dz = trim(field("dpos_z_mm"));
            const std::string dc = trim(field("dpos_c_deg"));
            const std::string db = trim(field("dpos_b_deg"));
            const int populated = static_cast<int>(!dx.empty()) + static_cast<int>(!dy.empty()) +
                                  static_cast<int>(!dz.empty()) + static_cast<int>(!dc.empty()) +
                                  static_cast<int>(!db.empty());
            if (populated != 0 && populated != 5) {
                error = "DPOS values are partially populated at CSV line " + std::to_string(lineNumber) + ".";
                return false;
            }
            rowHasDpos = populated == 5;
            if (rowHasDpos &&
                (!parseDouble(dx, dpos.x) || !parseDouble(dy, dpos.y) ||
                 !parseDouble(dz, dpos.z) || !parseDouble(dc, dpos.c) ||
                 !parseDouble(db, dpos.b))) {
                error = "Invalid DPOS numeric field at CSV line " + std::to_string(lineNumber) + ".";
                return false;
            }
        }
        double weight = 1.0;
        if (weightColumn >= 0 && weightColumn < static_cast<int>(fields.size()) &&
            !fields[static_cast<std::size_t>(weightColumn)].empty()) {
            if (!parseDouble(fields[static_cast<std::size_t>(weightColumn)], weight)) {
                std::ostringstream message;
                message << "Invalid weight at CSV line " << lineNumber << ".";
                error = message.str();
                return false;
            }
        }

        if (weight<0.0) {
            error="Negative weight at CSV line "+std::to_string(lineNumber)+".";
            return false;
        }
        std::size_t obsIndex = 0;
        const auto found = lookup.find(profileId);
        if (found == lookup.end()) {
            SphereProfileObservation obs;
            obs.profileId = profileId;
            obs.joints.x = x;
            obs.joints.y = y;
            obs.joints.z = z;
            obs.joints.c = c;
            obs.joints.b = b;
            obs.dposJoints = dpos;
            obs.hasDposJoints = rowHasDpos;
            obsIndex = observations.size();
            observations.push_back(obs);
            lookup[profileId] = obsIndex;
        } else {
            obsIndex = found->second;
            const JointPose5d& q = observations[obsIndex].joints;
            if (std::abs(q.x - x) > 1e-8 || std::abs(q.y - y) > 1e-8 ||
                std::abs(q.z - z) > 1e-8 || std::abs(q.c - c) > 1e-8 ||
                std::abs(q.b - b) > 1e-8) {
                std::ostringstream message;
                message << "Inconsistent joint values for profile " << profileId
                        << " at line " << lineNumber << ".";
                error = message.str();
                return false;
            }
            if (observations[obsIndex].hasDposJoints != rowHasDpos) {
                error = "DPOS availability changes within profile " + std::to_string(profileId)
                      + " at line " + std::to_string(lineNumber) + ".";
                return false;
            }
            if (rowHasDpos) {
                const SphereProfileObservation& obs = observations[obsIndex];
                if (std::abs(obs.dposJoints.x-dpos.x)>1e-8 || std::abs(obs.dposJoints.y-dpos.y)>1e-8 ||
                    std::abs(obs.dposJoints.z-dpos.z)>1e-8 || std::abs(obs.dposJoints.c-dpos.c)>1e-8 ||
                    std::abs(obs.dposJoints.b-dpos.b)>1e-8) {
                    error = "Inconsistent DPOS joint values for profile " + std::to_string(profileId)
                          + " at line " + std::to_string(lineNumber) + ".";
                    return false;
                }
            }
        }
        SensorProfilePoint point;
        point.x = sensorX;
        point.z = sensorZ;
        point.weight = std::max(0.0, weight);
        observations[obsIndex].points.push_back(point);
    }

    observations.erase(
        std::remove_if(observations.begin(), observations.end(),
                       [](const SphereProfileObservation& obs) {
                           return obs.points.size() < 5;
                       }),
        observations.end());
    if (observations.empty()) {
        error = "CSV contains no profile with at least five valid sphere points.";
        return false;
    }
    error.clear();
    return true;
}

bool SingleSphere17Calibrator::writeResultJson(std::ostream& output,
                                               const SingleSphere17Result& result,
                                               std::string& error)
{
    if (!output.good()) {
        error = "Output stream is not writable.";
        return false;
    }
    if (result.success && (!finiteParameters(result.parameters) || !finiteVec(result.sphereCenterMachineMm) ||
        !std::isfinite(result.residualRmsMm) || !std::isfinite(result.residualMaxAbsMm) ||
        !std::isfinite(result.observability.conditionNumber))) {
        error="Cannot serialize a successful calibration containing non-finite values.";
        return false;
    }
    if (result.hasInitializationBootstrap &&
        (!finiteParameters(result.initializationParameters) ||
         !finiteVec(result.initializationSphereCenterMachineMm) ||
         !std::isfinite(result.initializationResidualRmsMm) ||
         !std::isfinite(result.initializationResidualMaxAbsMm) ||
         !std::isfinite(result.initializationConditionNumber))) {
        error="Cannot serialize non-finite initialization bootstrap provenance.";
        return false;
    }
    const auto jsonNumber=[](double value) {
        if (!std::isfinite(value)) return std::string("null");
        std::ostringstream text; text<<std::setprecision(17)<<value; return text.str();
    };
    output << std::setprecision(17);
    output << "{\n";
    output << "  \"version\": \"20.2-single-sphere17-rank14-v27.6.41\",\n";
    output << "  \"success\": " << (result.success ? "true" : "false") << ",\n";
    output << "  \"optimization_converged\": " << (result.optimizationConverged ? "true" : "false") << ",\n";
    output << "  \"robust_cost\": " << jsonNumber(result.robustCost) << ",\n";
    output << "  \"model\": \"single_sphere_physical17_rank14\",\n";
    output << "  \"gauge_description\": \""
           << jsonEscape(result.gaugeDescription) << "\",\n";
    output << "  \"message\": \"" << jsonEscape(result.message) << "\",\n";
    output << "  \"parameters\": {\n";
    const auto& names = Physical17Parameters::names();
    const auto& units = Physical17Parameters::units();
    for (std::size_t i = 0; i < kTargetCount; ++i) {
        output << "    \"" << names[i] << "\": {\"value\": "
               << jsonNumber(result.parameters[i]) << ", \"unit\": \"" << units[i] << "\"}";
        output << ((i + 1 < kTargetCount) ? ",\n" : "\n");
    }
    output << "  },\n";
    output << "  \"handeye_translation_gauge_anchor_mm\": ["
           << jsonNumber(result.parameters[Physical17Parameters::HandEyeTx]) << ", "
           << jsonNumber(result.parameters[Physical17Parameters::HandEyeTy]) << ", "
           << jsonNumber(result.parameters[Physical17Parameters::HandEyeTz]) << "],\n";
    output << "  \"sphere_center_machine_mm\": ["
           << jsonNumber(result.sphereCenterMachineMm.x) << ", "
           << jsonNumber(result.sphereCenterMachineMm.y) << ", "
           << jsonNumber(result.sphereCenterMachineMm.z) << "],\n";
    if (result.hasInitializationBootstrap) {
        output << "  \"initialization_bootstrap\": {\n";
        output << "    \"source\": \"" << jsonEscape(result.initializationSource) << "\",\n";
        output << "    \"state_rank\": " << result.initializationStateRank << ",\n";
        output << "    \"residual_rms_mm\": " << jsonNumber(result.initializationResidualRmsMm) << ",\n";
        output << "    \"residual_max_abs_mm\": " << jsonNumber(result.initializationResidualMaxAbsMm) << ",\n";
        output << "    \"condition_number\": " << jsonNumber(result.initializationConditionNumber) << ",\n";
        output << "    \"profile_count\": " << result.initializationProfileCount << ",\n";
        output << "    \"point_count\": " << result.initializationPointCount << ",\n";
        output << "    \"hand_eye\": {"
               << "\"rx_rad\":" << jsonNumber(result.initializationParameters[Physical17Parameters::HandEyeRx]) << ","
               << "\"ry_rad\":" << jsonNumber(result.initializationParameters[Physical17Parameters::HandEyeRy]) << ","
               << "\"rz_rad\":" << jsonNumber(result.initializationParameters[Physical17Parameters::HandEyeRz]) << ","
               << "\"tx_mm\":" << jsonNumber(result.initializationParameters[Physical17Parameters::HandEyeTx]) << ","
               << "\"ty_mm\":" << jsonNumber(result.initializationParameters[Physical17Parameters::HandEyeTy]) << ","
               << "\"tz_mm\":" << jsonNumber(result.initializationParameters[Physical17Parameters::HandEyeTz]) << "},\n";
        output << "    \"sphere_center_machine_mm\": ["
               << jsonNumber(result.initializationSphereCenterMachineMm.x) << ", "
               << jsonNumber(result.initializationSphereCenterMachineMm.y) << ", "
               << jsonNumber(result.initializationSphereCenterMachineMm.z) << "]\n";
        output << "  },\n";
    }
    output << "  \"pose_screening\": {\n";
    output << "    \"rounds_applied\": " << result.poseScreeningRounds << ",\n";
    output << "    \"input_pose_count\": " << result.trainingPoseCountBeforeScreening << ",\n";
    output << "    \"retained_pose_count\": " << result.trainingPoseCountAfterScreening << ",\n";
    output << "    \"rejected_pose_count\": " << result.rejectedTrainingPoses.size() << ",\n";
    output << "    \"last_rms_threshold_mm\": " << jsonNumber(result.poseScreeningRmsThresholdMm) << ",\n";
    output << "    \"last_p95_threshold_mm\": " << jsonNumber(result.poseScreeningP95ThresholdMm) << ",\n";
    output << "    \"rejected_poses\": [";
    for (std::size_t i=0; i<result.rejectedTrainingPoses.size(); ++i) {
        const auto& pose=result.rejectedTrainingPoses[i];
        if (i>0) output << ",";
        output << "\n      {\"x_mm\":" << jsonNumber(pose.pose.x)
               << ",\"y_mm\":" << jsonNumber(pose.pose.y)
               << ",\"z_mm\":" << jsonNumber(pose.pose.z)
               << ",\"c_deg\":" << jsonNumber(pose.pose.c)
               << ",\"b_deg\":" << jsonNumber(pose.pose.b)
               << ",\"profile_count\":" << pose.profileCount
               << ",\"point_count\":" << pose.pointCount
               << ",\"rms_mm\":" << jsonNumber(pose.residualRmsMm)
               << ",\"p95_abs_mm\":" << jsonNumber(pose.residualP95AbsMm)
               << ",\"max_abs_mm\":" << jsonNumber(pose.residualMaxAbsMm)
               << ",\"score\":" << jsonNumber(pose.rejectionScore) << "}";
    }
    if (!result.rejectedTrainingPoses.empty()) output << "\n    ";
    output << "]\n";
    output << "  },\n";
    output << "  \"residual_rms_mm\": " << jsonNumber(result.residualRmsMm) << ",\n";
    output << "  \"residual_max_abs_mm\": " << jsonNumber(result.residualMaxAbsMm) << ",\n";
    output << "  \"profile_count\": " << result.profileCount << ",\n";
    output << "  \"point_count\": " << result.pointCount << ",\n";
    output << "  \"observability\": {\n";
    output << "    \"physical_parameter_count\": "
           << result.observability.physicalParameterCount << ",\n";
    output << "    \"rank_after_sphere_center_elimination\": "
           << result.observability.targetRank << ",\n";
    output << "    \"expected_rank_after_sphere_center_elimination\": "
           << result.observability.expectedTargetRank << ",\n";
    output << "    \"full_state_count\": "
           << result.observability.fullStateCount << ",\n";
    output << "    \"full_state_rank\": "
           << result.observability.fullStateRank << ",\n";
    output << "    \"expected_full_state_rank\": "
           << result.observability.expectedFullStateRank << ",\n";
    output << "    \"nonzero_condition_number\": "
           << jsonNumber(result.observability.conditionNumber) << ",\n";
    output << "    \"physical_singular_values\": [";
    for (std::size_t i = 0; i < kTargetCount; ++i) {
        if (i > 0) output << ", ";
        output << jsonNumber(result.observability.targetSingularValues[i]);
    }
    output << "],\n";
    output << "    \"full_state_singular_values\": [";
    for (std::size_t i = 0; i < kStateCount; ++i) {
        if (i > 0) output << ", ";
        output << jsonNumber(result.observability.fullStateSingularValues[i]);
    }
    output << "]\n";
    output << "  }\n";
    output << "}\n";
    if (!output.good()) {
        error = "Failed while writing JSON.";
        return false;
    }
    error.clear();
    return true;
}

SingleSphere17PosePlanner::SingleSphere17PosePlanner(const PosePlannerConfig& config)
    : m_config(config)
{
}

bool SingleSphere17PosePlanner::recommendXYZRecentering(
    const JointPose5d& currentJoint,
    const Vec3d& measuredCenterSensorMm,
    const Vec3d& targetCenterSensorMm,
    const Physical17Parameters& currentEstimate,
    double maxStepMm,
    JointPose5d& suggestedJoint,
    Vec3d& predictedCenterAfterStepMm,
    std::string& error)
{
    if (!finiteVec(measuredCenterSensorMm) || !finiteVec(targetCenterSensorMm) ||
        !(maxStepMm > 0.0) || !std::isfinite(maxStepMm)) {
        error = "Invalid sphere-center input or maximum XYZ step.";
        return false;
    }

    // Treat the currently measured center as the fixed workspace point for
    // this local feedback step. This avoids requiring its absolute position.
    const Vec3d centerMachine = SingleSphere17Model::mapSensorPoint(
        currentJoint, measuredCenterSensorMm, currentEstimate);

    constexpr double h = 0.05; // mm; only used to obtain the local Jacobian.
    Vec3d columns[3];
    for (int axis = 0; axis < 3; ++axis) {
        JointPose5d plus = currentJoint;
        JointPose5d minus = currentJoint;
        if (axis == 0) { plus.x += h; minus.x -= h; }
        if (axis == 1) { plus.y += h; minus.y -= h; }
        if (axis == 2) { plus.z += h; minus.z -= h; }
        const Vec3d cp = SingleSphere17Model::machinePointInSensor(
            plus, centerMachine, currentEstimate);
        const Vec3d cm = SingleSphere17Model::machinePointInSensor(
            minus, centerMachine, currentEstimate);
        columns[axis] = mul(sub(cp, cm), 1.0 / (2.0 * h));
    }

    Mat3d jacobian;
    jacobian.v[0][0] = columns[0].x;
    jacobian.v[1][0] = columns[0].y;
    jacobian.v[2][0] = columns[0].z;
    jacobian.v[0][1] = columns[1].x;
    jacobian.v[1][1] = columns[1].y;
    jacobian.v[2][1] = columns[1].z;
    jacobian.v[0][2] = columns[2].x;
    jacobian.v[1][2] = columns[2].y;
    jacobian.v[2][2] = columns[2].z;

    Vec3d delta;
    if (!solve3x3(jacobian, sub(targetCenterSensorMm, measuredCenterSensorMm), delta)) {
        error = "The local XYZ recentering Jacobian is singular.";
        return false;
    }
    const double deltaNorm = norm(delta);
    if (deltaNorm > maxStepMm) {
        delta = mul(delta, maxStepMm / deltaNorm);
    }

    suggestedJoint = currentJoint;
    suggestedJoint.x += delta.x;
    suggestedJoint.y += delta.y;
    suggestedJoint.z += delta.z;
    predictedCenterAfterStepMm = SingleSphere17Model::machinePointInSensor(
        suggestedJoint, centerMachine, currentEstimate);
    error.clear();
    return true;
}

std::vector<CalibrationPoseInstruction> SingleSphere17PosePlanner::generate(
    const PosePlannerSeed& seed,
    const Physical17Parameters& currentEstimate,
    std::string& error,
    const Physical17PlanningChecks& checks,
    const Physical17PlanningProgressCallback& progress,
    const std::atomic_bool* cancelRequested) const
{
    std::vector<CalibrationPoseInstruction> output;
    const auto cancelled=[&]() {
        return cancelRequested && cancelRequested->load(std::memory_order_relaxed);
    };
    const auto report=[&](Physical17PlanningPhase phase,
                          std::size_t current,
                          std::size_t total,
                          std::size_t feasibleCandidates,
                          int selectedPoses) {
        if (!progress) return;
        Physical17PlanningProgress p;
        p.phase=phase;
        p.current=current;
        p.total=total;
        p.feasibleCandidates=feasibleCandidates;
        p.selectedPoses=selectedPoses;
        p.requestedPoses=m_config.requestedRecordedPoses;
        progress(p);
    };
    report(Physical17PlanningPhase::Preparing,0,1,0,0);
    if (cancelled()) {
        error="Pose planning cancelled by operator.";
        return output;
    }
    const double radius = 0.5 * m_config.sphereDiameterMm;
    if (!std::isfinite(radius) || !(radius > 0.0) || m_config.requestedRecordedPoses < 1) {
        error = "Invalid sphere diameter or requested pose count.";
        return output;
    }
    if (!seed.hasTaughtJoint && !seed.hasSphereCenterMachine) {
        error = "Offline planning without a known start pose requires a machine-frame sphere center.";
        return output;
    }
    if (!seed.hasSphereCenterMachine && std::abs(seed.taughtSphereCenterSensorMm.y) >= radius) {
        error = "The taught laser plane does not intersect the configured sphere.";
        return output;
    }

    if (!finiteParameters(currentEstimate) ||
        (seed.hasTaughtJoint && !finitePose(seed.taughtJoint)) ||
        (seed.hasSphereCenterMachine ? !finiteVec(seed.sphereCenterMachineMm) : !finiteVec(seed.taughtSphereCenterSensorMm))) {
        error="Non-finite model, reference pose or sphere center."; return output;
    }
    const auto orderedFinite=[](double lo,double hi) { return std::isfinite(lo)&&std::isfinite(hi)&&lo<=hi; };
    if (!orderedFinite(m_config.xMinMm,m_config.xMaxMm) || !orderedFinite(m_config.yMinMm,m_config.yMaxMm) ||
        !orderedFinite(m_config.zMinMm,m_config.zMaxMm) || !orderedFinite(m_config.sensorZMinMm,m_config.sensorZMaxMm) ||
        m_config.sensorZMinMm==m_config.sensorZMaxMm || !std::isfinite(m_config.sensorXHalfWidthMm) || m_config.sensorXHalfWidthMm<=0.0 ||
        !std::isfinite(m_config.informationRidge) || m_config.informationRidge<=0.0 ||
        m_config.syntheticPointsPerProfile<5 || m_config.syntheticPointsPerProfile>10000) {
        error="Invalid XYZ/sensor limits, information ridge or synthetic sample count."; return output;
    }
    if (!std::isfinite(m_config.sensorXSafetyMarginMm) || m_config.sensorXSafetyMarginMm<0.0 ||
        !std::isfinite(m_config.sensorZSafetyMarginMm) || m_config.sensorZSafetyMarginMm<0.0 ||
        2.0*m_config.sensorZSafetyMarginMm>=m_config.sensorZMaxMm-m_config.sensorZMinMm ||
        (m_config.useTrapezoidalSensorField &&
         (!std::isfinite(m_config.sensorXSpanNearMm) || !std::isfinite(m_config.sensorXSpanFarMm) ||
          std::min(m_config.sensorXSpanNearMm,m_config.sensorXSpanFarMm)<=2.0*m_config.sensorXSafetyMarginMm))) {
        error="Invalid trapezoid dimensions or field safety margins."; return output;
    }

    const Vec3d sphereCenterMachine = seed.hasSphereCenterMachine
        ? seed.sphereCenterMachineMm
        : SingleSphere17Model::mapSensorPoint(
              seed.taughtJoint,
              seed.taughtSphereCenterSensorMm,
              currentEstimate);

    Vec3d ux;
    Vec3d uy;
    Vec3d uz;
    linearAxisDirections(currentEstimate, ux, uy, uz);
    Mat3d axisMatrix;
    axisMatrix.v[0][0] = ux.x; axisMatrix.v[0][1] = uy.x; axisMatrix.v[0][2] = uz.x;
    axisMatrix.v[1][0] = ux.y; axisMatrix.v[1][1] = uy.y; axisMatrix.v[1][2] = uz.y;
    axisMatrix.v[2][0] = ux.z; axisMatrix.v[2][1] = uy.z; axisMatrix.v[2][2] = uz.z;

    std::vector<double> bValues = inclusiveRange(
        m_config.bMinDeg, m_config.bMaxDeg, m_config.bStepDeg);
    std::vector<double> cValues = inclusiveRange(
        m_config.cMinDeg, m_config.cMaxDeg, m_config.cStepDeg);

    if (bValues.empty() || cValues.empty()) {
        error="Invalid/empty B/C range or excessive angular grid (limit 10000 intervals).";
        return output;
    }

    // Build the desired sphere-center locations in Sensor coordinates first.
    // Legacy callers keep the historical X/Z lattice.  The integrated Stage-2
    // GUI enables the D/gamma/eta branch so pose quality and kinematic rank are
    // handled as two separate requirements: sensor-quality geometry is a hard
    // candidate gate; D-optimality still decides which reachable poses are kept.
    std::vector<Vec3d> targetCenters;
    if (m_config.useQualityStudyGeometry) {
        if (!(m_config.qualityDMinMm>0.0) || m_config.qualityGammaMinDeg<0.0 ||
            m_config.qualityGammaMaxDeg>=89.0 || m_config.qualityEtaMinDeg<=-89.0 || m_config.qualityEtaMaxDeg>=89.0) {
            error="Invalid D/gamma/eta geometry bounds."; return output;
        }
        const auto dValues = inclusiveRange(
            m_config.qualityDMinMm, m_config.qualityDMaxMm, m_config.qualityDStepMm);
        const auto gammaValues = inclusiveRange(
            m_config.qualityGammaMinDeg, m_config.qualityGammaMaxDeg, m_config.qualityGammaStepDeg);
        const auto etaValues = inclusiveRange(
            m_config.qualityEtaMinDeg, m_config.qualityEtaMaxDeg, m_config.qualityEtaStepDeg);
        if (dValues.empty() || gammaValues.empty() || etaValues.empty()) {
            error = "Invalid D/gamma/eta quality-study candidate range.";
            return output;
        }
        // Do not reject on the theoretical Cartesian-product size here.  Many
        // combinations are removed immediately by XYZ reachability, optical-FOV
        // and static MachineSafety gates.  V27.6.23 therefore applies the hard
        // memory/complexity budget to the *feasible* candidate pool below rather
        // than to this conservative raw count.
        for (double d : dValues) {
            if (!(d > 0.0)) continue;
            for (double gammaDeg : gammaValues) {
                if (!std::isfinite(gammaDeg) || gammaDeg < 0.0 || gammaDeg >= 89.0) continue;
                const double h = radius * std::sin(gammaDeg * kDegToRad);
                for (double etaDeg : etaValues) {
                    if (!std::isfinite(etaDeg) || std::abs(etaDeg) >= 89.0) continue;
                    const double eta = etaDeg * kDegToRad;
                    const double targetX = d * std::sin(eta);
                    const double targetZ = d * std::cos(eta);
                    targetCenters.emplace_back(targetX, +h, targetZ);
                    if (m_config.useBothLaserPlaneSides && std::abs(h) > 1e-12) {
                        targetCenters.emplace_back(targetX, -h, targetZ);
                    }
                }
            }
        }
    } else {
        std::vector<double> yValues{radius * m_config.targetCenterYRadiusRatio};
        if (m_config.useBothLaserPlaneSides) {
            yValues.push_back(-radius * m_config.targetCenterYRadiusRatio);
        }
        for (double targetX : m_config.targetCenterXmm) {
            for (double targetY : yValues) {
                for (double targetZ : m_config.targetCenterZmm) {
                    targetCenters.emplace_back(targetX, targetY, targetZ);
                }
            }
        }
    }

    const long double gridSize=static_cast<long double>(bValues.size())*cValues.size()*targetCenters.size();
    // V27.6.25: raw dense grids are cheap geometry.  Do not spend one exact
    // STL/FCL MachineSafety query and one 17x17 information build on every raw
    // Cartesian combination.  Keep only a deterministic bounded reservoir,
    // evaluate the detailed optical gate + information on a shortlist, and
    // defer exact pose/transition safety until a candidate is about to enter
    // the D-optimal acquisition sequence.
    constexpr long double kMaxRawCandidateCombinations = 5000000.0L;
    constexpr std::size_t kDenseActivationThreshold = 20000u;
    constexpr std::size_t kDenseReservoirCapacity = 32768u;
    constexpr std::size_t kOnlineStartAnchorCapacity = 256u;
    if (gridSize>kMaxRawCandidateCombinations) {
        error="Raw candidate grid exceeds 5000000 combinations; increase B/C/D/gamma/eta steps.";
        return output;
    }

    const std::size_t desiredInformationCandidates=std::min<std::size_t>(
        8192u,
        std::max<std::size_t>(2048u,
            static_cast<std::size_t>(std::max(1,m_config.requestedRecordedPoses))*96u));

    // Supplying only poseAllowed preserves the historical eager-gate contract
    // for portable/core callers.  The integrated GUI supplies shortlistAllowed
    // (optical) and poseAllowed (exact MachineSafety) separately, enabling the
    // new lazy safety path without changing older call sites.
    const bool eagerLegacyPoseGate=static_cast<bool>(checks.poseAllowed) &&
                                   !static_cast<bool>(checks.shortlistAllowed);

    auto reservoirLess=[](const LightweightCandidate& a,const LightweightCandidate& b) {
        return a.priority < b.priority; // max-heap: worst retained hash on top
    };
    std::priority_queue<LightweightCandidate,
                        std::vector<LightweightCandidate>,
                        decltype(reservoirLess)> reservoir(reservoirLess);

    struct StartCandidate {
        double distance = 0.0;
        LightweightCandidate candidate;
    };
    auto startLess=[](const StartCandidate& a,const StartCandidate& b) {
        return a.distance < b.distance; // max-heap: farthest retained start anchor
    };
    std::priority_queue<StartCandidate,
                        std::vector<StartCandidate>,
                        decltype(startLess)> startAnchors(startLess);

    const auto considerReservoir=[&](LightweightCandidate candidate) {
        if (reservoir.size()<kDenseReservoirCapacity) {
            reservoir.push(std::move(candidate));
            return;
        }
        if (candidate.priority<reservoir.top().priority) {
            reservoir.pop();
            reservoir.push(std::move(candidate));
        }
    };

    const auto startDistance=[&](const JointPose5d& q) {
        if (!seed.hasTaughtJoint) return 0.0;
        const double dx=(q.x-seed.taughtJoint.x)/50.0;
        const double dy=(q.y-seed.taughtJoint.y)/50.0;
        const double dz=(q.z-seed.taughtJoint.z)/50.0;
        const double db=(q.b-seed.taughtJoint.b)/15.0;
        const double dc=(q.c-seed.taughtJoint.c)/20.0;
        return dx*dx+dy*dy+dz*dz+db*db+dc*dc;
    };

    const std::size_t rawGridCount=static_cast<std::size_t>(gridSize);
    const std::size_t candidateProgressStride=std::max<std::size_t>(1,rawGridCount/500u);
    std::size_t rawProcessed=0;
    std::size_t geometricallyFeasible=0;
    std::size_t reservoirEligible=0;
    report(Physical17PlanningPhase::FilteringCandidates,0,rawGridCount,0,0);
    for (double b : bValues) {
        for (double c : cValues) {
            for (const Vec3d& target : targetCenters) {
                ++rawProcessed;
                if ((rawProcessed%64u)==0u && cancelled()) {
                    error="Pose planning cancelled by operator.";
                    return output;
                }
                if ((rawProcessed%candidateProgressStride)==0u || rawProcessed==rawGridCount) {
                    report(Physical17PlanningPhase::FilteringCandidates,
                           rawProcessed,rawGridCount,geometricallyFeasible,0);
                }
                const double rhoSquared = radius * radius - target.y * target.y;
                if (!(rhoSquared > 0.0)) continue;
                const double rho = std::sqrt(rhoSquared);
                if (!finiteVec(target)) continue;
                if (m_config.useEndEffectorFacingArc || m_config.useTrapezoidalSensorField) {
                    // Cheap disjoint-bounds gate only.  Detailed visible-arc
                    // simulation is intentionally postponed to the shortlist.
                    const double maxHalf=m_config.useTrapezoidalSensorField
                        ? 0.5*std::max(m_config.sensorXSpanNearMm,m_config.sensorXSpanFarMm)
                        : m_config.sensorXHalfWidthMm;
                    if (std::abs(target.x)-rho>maxHalf || target.z+rho<m_config.sensorZMinMm ||
                        target.z-rho>m_config.sensorZMaxMm) continue;
                } else if (std::abs(target.x)+0.90*rho>m_config.sensorXHalfWidthMm ||
                           target.z-rho<m_config.sensorZMinMm || target.z>m_config.sensorZMaxMm) {
                    continue;
                }

                JointPose5d orientationOnly;
                orientationOnly.b = b;
                orientationOnly.c = c;
                const Vec3d rotated = SingleSphere17Model::mapSensorPoint(
                    orientationOnly, target, currentEstimate);
                Vec3d xyz;
                if (!solve3x3(axisMatrix, sub(sphereCenterMachine, rotated), xyz)) continue;
                if (xyz.x < m_config.xMinMm || xyz.x > m_config.xMaxMm ||
                    xyz.y < m_config.yMinMm || xyz.y > m_config.yMaxMm ||
                    xyz.z < m_config.zMinMm || xyz.z > m_config.zMaxMm) continue;

                LightweightCandidate light;
                light.joint.x=xyz.x;
                light.joint.y=xyz.y;
                light.joint.z=xyz.z;
                light.joint.b=b;
                light.joint.c=c;
                light.targetCenterSensor=target;
                light.ordinal=rawProcessed-1u;
                light.priority=stableMix64(static_cast<std::uint64_t>(light.ordinal));
                ++geometricallyFeasible;

                if (eagerLegacyPoseGate &&
                    !checks.poseAllowed(light.joint,light.targetCenterSensor)) {
                    continue;
                }
                ++reservoirEligible;
                considerReservoir(light);

                if (seed.hasTaughtJoint) {
                    StartCandidate start;
                    start.distance=startDistance(light.joint);
                    start.candidate=light;
                    start.candidate.startAnchor=true;
                    if (startAnchors.size()<kOnlineStartAnchorCapacity) {
                        startAnchors.push(std::move(start));
                    } else if (start.distance<startAnchors.top().distance) {
                        startAnchors.pop();
                        startAnchors.push(std::move(start));
                    }
                }
            }
        }
    }

    if (cancelled()) {
        error="Pose planning cancelled by operator.";
        return output;
    }
    if (reservoirEligible==0 || reservoir.empty()) {
        error = "No feasible candidate pose inside the configured sensor field.";
        return output;
    }

    const bool denseMode=reservoirEligible>kDenseActivationThreshold;
    std::vector<LightweightCandidate> pool;
    pool.reserve(reservoir.size()+startAnchors.size());
    while (!reservoir.empty()) {
        pool.push_back(reservoir.top());
        reservoir.pop();
    }
    while (!startAnchors.empty()) {
        pool.push_back(startAnchors.top().candidate);
        startAnchors.pop();
    }

    // Merge start anchors with the deterministic reservoir.  For small grids the
    // reservoir contains every eligible pose and ordinal ordering preserves the
    // historical candidate/D-optimal ordering exactly.  Dense grids evaluate
    // online start anchors first, then the stable hash sample.
    std::sort(pool.begin(),pool.end(),[](const LightweightCandidate& a,const LightweightCandidate& b) {
        return a.ordinal<b.ordinal;
    });
    std::vector<LightweightCandidate> merged;
    merged.reserve(pool.size());
    for (const auto& item:pool) {
        if (!merged.empty() && merged.back().ordinal==item.ordinal) {
            merged.back().startAnchor=merged.back().startAnchor||item.startAnchor;
            continue;
        }
        merged.push_back(item);
    }
    pool.swap(merged);
    if (denseMode) {
        std::sort(pool.begin(),pool.end(),[](const LightweightCandidate& a,const LightweightCandidate& b) {
            if (a.startAnchor!=b.startAnchor) return a.startAnchor>b.startAnchor;
            if (a.priority!=b.priority) return a.priority<b.priority;
            return a.ordinal<b.ordinal;
        });
    }

    const std::size_t informationTarget=denseMode
        ? std::min(desiredInformationCandidates,pool.size())
        : pool.size();
    std::vector<CandidatePose> candidates;
    candidates.reserve(informationTarget);
    const std::size_t shortlistProgressStride=std::max<std::size_t>(1,pool.size()/300u);
    const std::size_t shortlistProgressTotal=denseMode ? informationTarget : pool.size();
    std::size_t shortlistProcessed=0;
    report(Physical17PlanningPhase::EvaluatingShortlist,0,shortlistProgressTotal,0,0);
    for (const LightweightCandidate& light:pool) {
        ++shortlistProcessed;
        if ((shortlistProcessed%32u)==0u && cancelled()) {
            error="Pose planning cancelled by operator.";
            return output;
        }
        if ((shortlistProcessed%shortlistProgressStride)==0u || shortlistProcessed==pool.size()) {
            const std::size_t progressCurrent=denseMode
                ? std::min(candidates.size(),shortlistProgressTotal)
                : std::min(shortlistProcessed,shortlistProgressTotal);
            report(Physical17PlanningPhase::EvaluatingShortlist,
                   progressCurrent,shortlistProgressTotal,candidates.size(),0);
        }
        if (checks.shortlistAllowed &&
            !checks.shortlistAllowed(light.joint,light.targetCenterSensor)) {
            continue;
        }

        CandidatePose candidate;
        candidate.joint=light.joint;
        candidate.targetCenterSensor=light.targetCenterSensor;
        candidate.syntheticObservation=syntheticAcquisitionProfile(
            candidate.joint,candidate.targetCenterSensor,radius,currentEstimate,m_config);
        if (candidate.syntheticObservation.points.size()<5) continue;
        candidate.information=candidateInformation(
            candidate,currentEstimate,sphereCenterMachine,radius,1e-5);
        candidates.push_back(std::move(candidate));

        if (denseMode && candidates.size()>=informationTarget) {
            report(Physical17PlanningPhase::EvaluatingShortlist,
                   informationTarget,informationTarget,candidates.size(),0);
            break;
        }
    }

    if (cancelled()) {
        error="Pose planning cancelled by operator.";
        return output;
    }
    if (candidates.empty()) {
        error = "No candidate survived shortlist optical/profile evaluation.";
        return output;
    }
    if (static_cast<int>(candidates.size())<m_config.requestedRecordedPoses &&
        m_config.requireRequestedPoseCount) {
        error="Only "+std::to_string(candidates.size())+" information candidates survived the hierarchical shortlist; widen the geometry or increase its sampling density.";
        return output;
    }

    DenseMatrix accumulated(static_cast<int>(kObservableStateCount),
                            static_cast<int>(kObservableStateCount), 0.0);
    for (int i = 0; i < accumulated.rows; ++i) {
        accumulated(i, i) = std::max(1e-12, m_config.informationRidge);
    }

    std::vector<int> selected;
    std::vector<bool> used(candidates.size(), false);
    std::map<std::pair<int, int>, int> bcCounts;

    // V27.6.25 lazy exact-safety gate.  In the integrated GUI the expensive
    // STL/FCL target check is intentionally *not* run on the full dense grid.
    // It is cached the first time a shortlisted candidate reaches the top of a
    // D-optimal/nearest ranking.  Transition safety remains predecessor-specific
    // and is therefore checked each time a candidate is considered from a new
    // acquisition predecessor.
    std::vector<int> feasible(candidates.size(),eagerLegacyPoseGate ? 1 : 0);
    auto poseAllowedOnly=[&](std::size_t i) {
        if (i>=candidates.size()) return false;
        if (feasible[i]>0) return true;
        if (feasible[i]<0) return false;
        if (checks.poseAllowed &&
            !checks.poseAllowed(candidates[i].joint,candidates[i].targetCenterSensor)) {
            feasible[i]=-1;
            return false;
        }
        feasible[i]=1;
        return true;
    };
    auto allowedTransition=[&](std::size_t i,const JointPose5d& from) {
        return poseAllowedOnly(i) &&
            (!checks.transitionAllowed || checks.transitionAllowed(from,candidates[i].joint));
    };

    int referenceIndex=-1;
    const std::size_t candidateCount=candidates.size();
    const std::size_t selectionTotal=std::max<std::size_t>(
        1u,candidateCount*static_cast<std::size_t>(std::max(1,m_config.requestedRecordedPoses)));
    const std::size_t selectionProgressStride=std::max<std::size_t>(1,candidateCount/250u);
    std::size_t selectionWork=0;
    report(Physical17PlanningPhase::SelectingDOptimal,0,selectionTotal,candidateCount,0);
    if (seed.hasTaughtJoint) {
        const Vec3d referenceCenterSensor = seed.hasSphereCenterMachine
            ? SingleSphere17Model::machinePointInSensor(
                  seed.taughtJoint, sphereCenterMachine, currentEstimate)
            : seed.taughtSphereCenterSensorMm;
        std::vector<std::pair<double,int>> nearest;
        for (std::size_t i=0;i<candidates.size();++i) {
            ++selectionWork;
            if ((selectionWork%256u)==0u && cancelled()) {
                error="Pose planning cancelled by operator.";
                return output;
            }
            if ((selectionWork%selectionProgressStride)==0u) {
                report(Physical17PlanningPhase::SelectingDOptimal,
                       std::min(selectionWork,selectionTotal),selectionTotal,candidateCount,0);
            }
            const double db=candidates[i].joint.b-seed.taughtJoint.b;
            const double dc=candidates[i].joint.c-seed.taughtJoint.c;
            const Vec3d ds=sub(candidates[i].targetCenterSensor,referenceCenterSensor);
            nearest.emplace_back(db*db+dc*dc+dot(ds,ds),static_cast<int>(i));
        }
        std::sort(nearest.begin(),nearest.end());
        for (const auto& entry:nearest) {
            if (allowedTransition(static_cast<std::size_t>(entry.second),seed.taughtJoint)) {
                referenceIndex=entry.second;
                break;
            }
        }
        if (referenceIndex<0) {
            error="No candidate passes optical/target/initial-transition preflight.";
            return output;
        }
    } else {
        // No live motion origin exists offline.  Use information gain to pick the
        // first safe pose and intentionally do not fabricate a transition from
        // XYZCB=(0,0,0,0,0) or from an old historical pose.
        const double baseLogDet=logDetPositiveDefinite(
            targetSchurInformation(accumulated,static_cast<int>(kIdentifiableCount)));
        std::vector<std::pair<double,int>> rankedFirst;
        for (std::size_t i=0;i<candidates.size();++i) {
            ++selectionWork;
            if ((selectionWork%256u)==0u && cancelled()) {
                error="Pose planning cancelled by operator.";
                return output;
            }
            if ((selectionWork%selectionProgressStride)==0u) {
                report(Physical17PlanningPhase::SelectingDOptimal,
                       std::min(selectionWork,selectionTotal),selectionTotal,candidateCount,0);
            }
            if (feasible[i]<0) continue;
            const double score=logDetPositiveDefinite(targetSchurInformation(
                addDense(accumulated,candidates[i].information),static_cast<int>(kIdentifiableCount)));
            if (std::isfinite(score)) rankedFirst.emplace_back(score,static_cast<int>(i));
        }
        std::sort(rankedFirst.begin(),rankedFirst.end(),[](const auto& x,const auto& y) {
            return x.first!=y.first ? x.first>y.first : x.second<y.second;
        });
        for (const auto& entry:rankedFirst) {
            if (poseAllowedOnly(static_cast<std::size_t>(entry.second))) {
                referenceIndex=entry.second;
                candidates[referenceIndex].informationGain=entry.first-baseLogDet;
                break;
            }
        }
        if (referenceIndex<0) {
            error="No candidate passes offline optical/target-pose preflight.";
            return output;
        }
    }

    selected.push_back(referenceIndex);
    used[static_cast<std::size_t>(referenceIndex)]=true;
    if (seed.hasTaughtJoint) {
        candidates[referenceIndex].informationGain=logDetPositiveDefinite(targetSchurInformation(
            addDense(accumulated,candidates[referenceIndex].information),static_cast<int>(kIdentifiableCount)))
            -logDetPositiveDefinite(targetSchurInformation(accumulated,static_cast<int>(kIdentifiableCount)));
    }
    accumulated=addDense(accumulated,candidates[referenceIndex].information);
    auto bcKey=[&](std::size_t i) {return std::make_pair(
        static_cast<int>(std::lround(candidates[i].joint.b*1000.0)),
        static_cast<int>(std::lround(candidates[i].joint.c*1000.0)));};
    ++bcCounts[bcKey(static_cast<std::size_t>(referenceIndex))];
    const int targetCount=m_config.requestedRecordedPoses;
    report(Physical17PlanningPhase::SelectingDOptimal,
           std::min(selectionWork,selectionTotal),selectionTotal,candidateCount,
           static_cast<int>(selected.size()));
    while (static_cast<int>(selected.size())<targetCount) {
        if (cancelled()) {
            error="Pose planning cancelled by operator.";
            return output;
        }
        const double previousLogDet=logDetPositiveDefinite(targetSchurInformation(accumulated,static_cast<int>(kIdentifiableCount)));
        std::vector<std::pair<double,int>> ranked;
        for (std::size_t i=0;i<candidates.size();++i) {
            ++selectionWork;
            if ((selectionWork%256u)==0u && cancelled()) {
                error="Pose planning cancelled by operator.";
                return output;
            }
            if ((selectionWork%selectionProgressStride)==0u) {
                report(Physical17PlanningPhase::SelectingDOptimal,
                       std::min(selectionWork,selectionTotal),selectionTotal,candidateCount,
                       static_cast<int>(selected.size()));
            }
            if (used[i] || feasible[i]<0 || bcCounts[bcKey(i)]>=std::max(1,m_config.maxRecordedPosesPerBCPair)) continue;
            const double score=logDetPositiveDefinite(targetSchurInformation(addDense(accumulated,candidates[i].information),static_cast<int>(kIdentifiableCount)));
            if (std::isfinite(score)) ranked.emplace_back(score,static_cast<int>(i));
        }
        std::sort(ranked.begin(),ranked.end(),[](const auto& x,const auto& y) {
            return x.first!=y.first ? x.first>y.first : x.second<y.second;
        });
        int best=-1;
        for (const auto& entry:ranked) {
            if (allowedTransition(static_cast<std::size_t>(entry.second),candidates[selected.back()].joint)) {
                best=entry.second;
                candidates[best].informationGain=entry.first-previousLogDet;
                break;
            }
        }
        if (best<0) break;
        selected.push_back(best);used[best]=true;
        accumulated=addDense(accumulated,candidates[best].information);
        ++bcCounts[bcKey(static_cast<std::size_t>(best))];
        report(Physical17PlanningPhase::SelectingDOptimal,
               std::min(selectionWork,selectionTotal),selectionTotal,candidateCount,
               static_cast<int>(selected.size()));
    }
    if (static_cast<int>(selected.size())<targetCount && m_config.requireRequestedPoseCount) {
        error="Only "+std::to_string(selected.size())+"/"+std::to_string(targetCount)+
            " poses pass D-optimal selection, per-B/C limits and optical/transition constraints.";
        return output;
    }

    report(Physical17PlanningPhase::Finalizing,0,1,candidateCount,
           static_cast<int>(selected.size()));
    if (cancelled()) {
        error="Pose planning cancelled by operator.";
        return output;
    }

    std::vector<int> ordered;
    if (!m_config.reorderRecordedPosesForTravel || checks.transitionAllowed) {
        // Reordering would create different, unchecked edges.  Keep the exact
        // checked acquisition order when transitions constrain the design.
        ordered=selected;
    } else {
        ordered.push_back(selected.front());selected.erase(selected.begin());
        while (!selected.empty()) {
            const CandidatePose& current=candidates[ordered.back()];
            auto bestIt=selected.begin();double bestDistance=std::numeric_limits<double>::infinity();
            for (auto it=selected.begin();it!=selected.end();++it) {
                const double distance=acquisitionDistance(current,candidates[*it]);
                if (distance<bestDistance) {bestDistance=distance;bestIt=it;}
            }
            ordered.push_back(*bestIt);selected.erase(bestIt);
        }
    }

    output.reserve(ordered.size());
    for (std::size_t sequence = 0; sequence < ordered.size(); ++sequence) {
        const CandidatePose& candidate = candidates[static_cast<std::size_t>(ordered[sequence])];
        CalibrationPoseInstruction instruction;
        instruction.sequence = static_cast<int>(sequence + 1);
        instruction.recordProfile = true;
        instruction.predictedJoint = candidate.joint;
        instruction.targetSphereCenterSensorMm = candidate.targetCenterSensor;
        instruction.informationGain = candidate.informationGain;
        instruction.group = (std::abs(candidate.joint.b) < 1e-9 ||
                             std::abs(candidate.joint.c) < 1e-9)
                                ? "single_rotary_excitation"
                                : "coupled_BC_excitation";
        std::ostringstream note;
        note << "Predicted XYZ is only a seed. Move B/C in increments <= "
             << m_config.maxTransitAngleStepDeg
             << " deg, fit the sphere after every increment, and close the XYZ loop "
                "until the fitted center reaches the requested sensor coordinate.";
        instruction.note = note.str();
        output.push_back(instruction);
    }

    report(Physical17PlanningPhase::Finalizing,1,1,candidateCount,
           static_cast<int>(output.size()));
    error.clear();
    return output;
}

bool SingleSphere17PosePlanner::writeCsv(
    std::ostream& output,
    const std::vector<CalibrationPoseInstruction>& poses,
    std::string& error)
{
    if (!output.good()) {
        error = "Output stream is not writable.";
        return false;
    }
    output << "sequence,record_profile,x_mm,y_mm,z_mm,c_deg,b_deg,"
              "target_sensor_x_mm,target_sensor_y_mm,target_sensor_z_mm,"
              "information_gain,group,note\n";
    output << std::setprecision(12);
    for (const auto& pose : poses) {
        std::string escapedNote = pose.note;
        std::size_t pos = 0;
        while ((pos = escapedNote.find('"', pos)) != std::string::npos) {
            escapedNote.insert(pos, 1, '"');
            pos += 2;
        }
        output << pose.sequence << ','
               << (pose.recordProfile ? 1 : 0) << ','
               << pose.predictedJoint.x << ','
               << pose.predictedJoint.y << ','
               << pose.predictedJoint.z << ','
               << pose.predictedJoint.c << ','
               << pose.predictedJoint.b << ','
               << pose.targetSphereCenterSensorMm.x << ','
               << pose.targetSphereCenterSensorMm.y << ','
               << pose.targetSphereCenterSensorMm.z << ','
               << pose.informationGain << ','
               << pose.group << ",\"" << escapedNote << "\"\n";
    }
    if (!output.good()) {
        error = "Failed while writing pose CSV.";
        return false;
    }
    error.clear();
    return true;
}

} // namespace calibration
} // namespace mls
