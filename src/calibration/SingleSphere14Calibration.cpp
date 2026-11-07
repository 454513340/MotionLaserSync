#include "calibration/SingleSphere14Calibration.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <limits>
#include <map>
#include <numeric>
#include <random>
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
constexpr std::size_t kTargetCount = Gauge14Parameters::Count;
constexpr std::size_t kStateCount = Gauge14Parameters::Count + 3;

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

void linearAxisDirections(const Gauge14Parameters& p,
                          Vec3d& ux,
                          Vec3d& uy,
                          Vec3d& uz)
{
    ux = Vec3d(1.0, 0.0, 0.0);
    uy = normalized(Vec3d(std::tan(p[Gauge14Parameters::Sxy]), 1.0, 0.0));
    uz = normalized(Vec3d(std::tan(p[Gauge14Parameters::Sxz]),
                          std::tan(p[Gauge14Parameters::Syz]),
                          1.0));
}

AxisLine cAxisLine(const Gauge14Parameters& p)
{
    AxisLine line;
    const Mat3d tilt = rotationFromVector(
        Vec3d(p[Gauge14Parameters::CTiltX],
              p[Gauge14Parameters::CTiltY],
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
    line.point = add(mul(line.basisU, p[Gauge14Parameters::COffsetU]),
                     mul(line.basisV, p[Gauge14Parameters::COffsetV]));
    return line;
}

AxisLine bAxisLine(const Gauge14Parameters& p)
{
    AxisLine line;
    const Mat3d tilt = rotationFromVector(
        Vec3d(p[Gauge14Parameters::BTiltX],
              0.0,
              p[Gauge14Parameters::BTiltZ]));
    line.direction = normalized(multiply(tilt, Vec3d(0.0, 1.0, 0.0)));

    Vec3d u = sub(Vec3d(1.0, 0.0, 0.0),
                  mul(line.direction, dot(line.direction, Vec3d(1.0, 0.0, 0.0))));
    if (norm(u) < 1e-10) {
        u = sub(Vec3d(0.0, 0.0, 1.0),
                mul(line.direction, dot(line.direction, Vec3d(0.0, 0.0, 1.0))));
    }
    line.basisU = normalized(u);
    line.basisV = normalized(cross(line.basisU, line.direction));
    line.point = add(mul(line.basisU, p[Gauge14Parameters::BOffsetU]),
                     mul(line.basisV, p[Gauge14Parameters::BOffsetV]));
    return line;
}

Vec3d handEyeRotate(const Gauge14Parameters& p, const Vec3d& sensorPoint)
{
    const Mat3d r = rotationFromVector(
        Vec3d(p[Gauge14Parameters::HandEyeRx],
              p[Gauge14Parameters::HandEyeRy],
              p[Gauge14Parameters::HandEyeRz]));
    return multiply(r, sensorPoint);
}

std::array<double, kStateCount> packState(const Gauge14Parameters& p,
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
                 Gauge14Parameters& p,
                 Vec3d& sphereCenter)
{
    for (std::size_t i = 0; i < kTargetCount; ++i) {
        p[i] = state[i];
    }
    sphereCenter = Vec3d(state[kTargetCount + 0],
                         state[kTargetCount + 1],
                         state[kTargetCount + 2]);
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
    Gauge14Parameters parameters;
    Vec3d sphereCenter;
    unpackState(state, parameters, sphereCenter);

    std::vector<double> residuals;
    std::size_t total = 0;
    for (const auto& obs : observations) {
        total += obs.points.size();
    }
    residuals.reserve(total);

    for (const auto& obs : observations) {
        for (const auto& point : obs.points) {
            const Vec3d sensor(point.x, 0.0, point.z);
            const Vec3d machine = SingleSphere14Model::mapSensorPoint(
                obs.joints, sensor, parameters);
            residuals.push_back(norm(sub(machine, sphereCenter)) - sphereRadiusMm);
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

DenseMatrix numericalJacobian(
    const std::vector<SphereProfileObservation>& observations,
    const std::array<double, kStateCount>& state,
    const std::array<double, kStateCount>& scales,
    double sphereRadiusMm,
    double relativeStep,
    const std::vector<int>& active)
{
    const std::vector<double> r0 = evaluateResiduals(observations, state, sphereRadiusMm);
    DenseMatrix jac(static_cast<int>(r0.size()), static_cast<int>(active.size()));
    const double hNorm = std::max(1e-8, relativeStep);

    for (std::size_t j = 0; j < active.size(); ++j) {
        const int index = active[j];
        std::array<double, kStateCount> plus = state;
        std::array<double, kStateCount> minus = state;
        const double h = hNorm * std::max(1e-12, scales[static_cast<std::size_t>(index)]);
        plus[static_cast<std::size_t>(index)] += h;
        minus[static_cast<std::size_t>(index)] -= h;
        const std::vector<double> rp = evaluateResiduals(observations, plus, sphereRadiusMm);
        const std::vector<double> rm = evaluateResiduals(observations, minus, sphereRadiusMm);
        for (std::size_t row = 0; row < r0.size(); ++row) {
            // Derivative with respect to normalized parameter delta.
            jac(static_cast<int>(row), static_cast<int>(j)) =
                (rp[row] - rm[row]) / (2.0 * hNorm);
        }
    }
    return jac;
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

DenseMatrix targetSchurInformation(const DenseMatrix& full)
{
    if (full.rows != static_cast<int>(kStateCount) ||
        full.cols != static_cast<int>(kStateCount)) {
        return DenseMatrix();
    }
    DenseMatrix target(static_cast<int>(kTargetCount),
                       static_cast<int>(kTargetCount), 0.0);
    DenseMatrix crossBlock(static_cast<int>(kTargetCount), 3, 0.0);
    DenseMatrix nuisance(3, 3, 0.0);
    for (int r = 0; r < static_cast<int>(kTargetCount); ++r) {
        for (int c = 0; c < static_cast<int>(kTargetCount); ++c) {
            target(r, c) = full(r, c);
        }
        for (int c = 0; c < 3; ++c) {
            crossBlock(r, c) = full(r, static_cast<int>(kTargetCount) + c);
        }
    }
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            nuisance(r, c) = full(static_cast<int>(kTargetCount) + r,
                                  static_cast<int>(kTargetCount) + c);
        }
    }
    DenseMatrix nuisanceInverse;
    if (!invert3(nuisance, nuisanceInverse)) {
        return DenseMatrix();
    }
    for (int r = 0; r < static_cast<int>(kTargetCount); ++r) {
        for (int c = 0; c < static_cast<int>(kTargetCount); ++c) {
            double correction = 0.0;
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) {
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

struct OptimizeResult
{
    bool success = false;
    int iterations = 0;
    double cost = std::numeric_limits<double>::infinity();
};

OptimizeResult optimizeSubset(
    const std::vector<SphereProfileObservation>& observations,
    const SolverOptions& options,
    const std::vector<int>& active,
    int maxIterations,
    std::array<double, kStateCount>& state)
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

    for (int iteration = 0; iteration < maxIterations; ++iteration) {
        DenseMatrix jac = numericalJacobian(observations,
                                            state,
                                            options.parameterScales,
                                            radius,
                                            options.finiteDifferenceRelativeStep,
                                            active);

        DenseMatrix h(static_cast<int>(active.size()), static_cast<int>(active.size()), 0.0);
        std::vector<double> gradient(active.size(), 0.0);

        for (int row = 0; row < jac.rows; ++row) {
            const double base = (static_cast<std::size_t>(row) < baseWeights.size())
                                    ? baseWeights[static_cast<std::size_t>(row)]
                                    : 1.0;
            const double robust = huberIrlsWeight(
                residuals[static_cast<std::size_t>(row)], options.huberDeltaMm);
            const double sqrtWeight = std::sqrt(std::max(0.0, base * robust));
            const double rw = sqrtWeight * residuals[static_cast<std::size_t>(row)];
            for (int a = 0; a < jac.cols; ++a) {
                const double ja = sqrtWeight * jac(row, a);
                gradient[static_cast<std::size_t>(a)] += ja * rw;
                for (int b = a; b < jac.cols; ++b) {
                    h(a, b) += ja * (sqrtWeight * jac(row, b));
                }
            }
        }
        for (int a = 0; a < h.rows; ++a) {
            for (int b = 0; b < a; ++b) {
                h(a, b) = h(b, a);
            }
        }

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
            result.iterations = iteration + 1;
            if (stepNorm < options.convergenceStep ||
                relativeImprovement < options.convergenceCost) {
                result.success = true;
                break;
            }
        } else {
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
    for (int i = 0; i <= static_cast<int>(Gauge14Parameters::BTiltZ); ++i) {
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
        static_cast<int>(Gauge14Parameters::HandEyeRx),
        static_cast<int>(Gauge14Parameters::HandEyeRy),
        static_cast<int>(Gauge14Parameters::HandEyeRz),
        static_cast<int>(kTargetCount + 0),
        static_cast<int>(kTargetCount + 1),
        static_cast<int>(kTargetCount + 2)
    };
}

std::vector<int> allIndices()
{
    std::vector<int> indices(kStateCount);
    std::iota(indices.begin(), indices.end(), 0);
    return indices;
}

std::array<double, kStateCount> deterministicStart(
    const std::array<double, kStateCount>& base,
    const SolverOptions& options,
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
        if (i <= Gauge14Parameters::Syz ||
            (i >= Gauge14Parameters::CTiltX && i <= Gauge14Parameters::CTiltY) ||
            (i >= Gauge14Parameters::BTiltX && i <= Gauge14Parameters::BTiltZ)) {
            amplitude = 2.0;
        } else if (i == Gauge14Parameters::COffsetU ||
                   i == Gauge14Parameters::COffsetV ||
                   i == Gauge14Parameters::BOffsetU ||
                   i == Gauge14Parameters::BOffsetV) {
            amplitude = 0.8;
        } else if (i >= Gauge14Parameters::HandEyeRx &&
                   i <= Gauge14Parameters::HandEyeRz) {
            amplitude = 1.5;
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
    if (!(step > 0.0) || maximum < minimum) {
        return values;
    }
    for (double value = minimum; value <= maximum + 0.5 * step; value += step) {
        values.push_back(std::min(value, maximum));
        if (values.size() > 10000) {
            break;
        }
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

DenseMatrix candidateInformation(const CandidatePose& candidate,
                                 const Gauge14Parameters& estimate,
                                 const Vec3d& sphereCenter,
                                 double sphereRadius,
                                 double relativeStep)
{
    std::vector<SphereProfileObservation> observations{candidate.syntheticObservation};
    std::array<double, kStateCount> state = packState(estimate, sphereCenter);
    SolverOptions options;
    const std::vector<int> active = allIndices();
    DenseMatrix jac = numericalJacobian(observations,
                                        state,
                                        options.parameterScales,
                                        sphereRadius,
                                        relativeStep,
                                        active);
    return normalInformation(jac);
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

} // namespace

const std::array<const char*, Gauge14Parameters::Count>& Gauge14Parameters::names()
{
    static const std::array<const char*, Count> kNames{{
        "Sxy", "Sxz", "Syz",
        "C_offset_u_effective", "C_offset_v_effective",
        "C_tilt_x", "C_tilt_y",
        "B_offset_u_effective", "B_offset_v_effective",
        "B_tilt_x", "B_tilt_z",
        "handeye_rx", "handeye_ry", "handeye_rz"
    }};
    return kNames;
}

const std::array<const char*, Gauge14Parameters::Count>& Gauge14Parameters::units()
{
    static const std::array<const char*, Count> kUnits{{
        "rad", "rad", "rad",
        "mm", "mm", "rad", "rad",
        "mm", "mm", "rad", "rad",
        "rad", "rad", "rad"
    }};
    return kUnits;
}

SolverOptions::SolverOptions()
{
    parameterScales = {{
        1e-4, 1e-4, 1e-4,
        10.0, 10.0,
        1e-4, 1e-4,
        10.0, 10.0,
        1e-4, 1e-4,
        0.02, 0.02, 0.02,
        100.0, 100.0, 100.0
    }};
}

Vec3d SingleSphere14Model::mapSensorPoint(const JointPose5d& joints,
                                          const Vec3d& sensorPoint,
                                          const Gauge14Parameters& parameters)
{
    Vec3d ux;
    Vec3d uy;
    Vec3d uz;
    linearAxisDirections(parameters, ux, uy, uz);
    const AxisLine cLine = cAxisLine(parameters);
    const AxisLine bLine = bAxisLine(parameters);

    Vec3d p = handEyeRotate(parameters, sensorPoint);
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

std::array<double, 16> SingleSphere14Model::sensorToGaugeMachineTransform(
    const JointPose5d& joints,
    const Gauge14Parameters& parameters)
{
    const Vec3d origin = mapSensorPoint(joints, Vec3d(0.0, 0.0, 0.0), parameters);
    const Vec3d ex = sub(mapSensorPoint(joints, Vec3d(1.0, 0.0, 0.0), parameters), origin);
    const Vec3d ey = sub(mapSensorPoint(joints, Vec3d(0.0, 1.0, 0.0), parameters), origin);
    const Vec3d ez = sub(mapSensorPoint(joints, Vec3d(0.0, 0.0, 1.0), parameters), origin);

    return {{
        ex.x, ey.x, ez.x, origin.x,
        ex.y, ey.y, ez.y, origin.y,
        ex.z, ey.z, ez.z, origin.z,
        0.0,  0.0,  0.0,  1.0
    }};
}

Vec3d SingleSphere14Model::sphereCenterInSensor(const JointPose5d& joints,
                                                const Vec3d& sphereCenterGaugeMm,
                                                const Gauge14Parameters& parameters)
{
    const std::array<double, 16> t = sensorToGaugeMachineTransform(joints, parameters);
    Mat3d r;
    r.v[0][0] = t[0]; r.v[0][1] = t[1]; r.v[0][2] = t[2];
    r.v[1][0] = t[4]; r.v[1][1] = t[5]; r.v[1][2] = t[6];
    r.v[2][0] = t[8]; r.v[2][1] = t[9]; r.v[2][2] = t[10];
    const Vec3d origin(t[3], t[7], t[11]);
    return multiply(transpose(r), sub(sphereCenterGaugeMm, origin));
}

SingleSphere14Calibrator::SingleSphere14Calibrator(const SolverOptions& options)
    : m_options(options)
{
}

void SingleSphere14Calibrator::clear()
{
    m_observations.clear();
}

void SingleSphere14Calibrator::addObservation(const SphereProfileObservation& observation)
{
    if (!observation.points.empty()) {
        m_observations.push_back(observation);
    }
}

SphereCenterFit SingleSphere14Calibrator::fitSphereCenterInSensor(
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

SingleSphere14Result SingleSphere14Calibrator::solve(
    const Gauge14Parameters& initialParameters,
    const Vec3d* initialSphereCenterGaugeMm) const
{
    SingleSphere14Result result;
    result.gaugeDescription =
        "14-D sensor-origin gauge: hand-eye translation is fixed to zero; "
        "C/B offsets are effective average-axis locations relative to the "
        "sensor optical origin at XYZCB=0. The four offsets must not be "
        "reported as the traditional physical PIGEs without an independent "
        "3-D translation datum.";

    if (!(m_options.sphereDiameterMm > 0.0)) {
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

    Vec3d initialCenter;
    if (initialSphereCenterGaugeMm && finiteVec(*initialSphereCenterGaugeMm)) {
        initialCenter = *initialSphereCenterGaugeMm;
    } else {
        const SphereCenterFit fit = fitSphereCenterInSensor(
            m_observations.front().points,
            m_options.sphereDiameterMm,
            m_options.sphereCenterYSign);
        if (!fit.success) {
            result.message = std::string("Cannot initialize sphere center: ") + fit.message;
            return result;
        }
        initialCenter = SingleSphere14Model::mapSensorPoint(
            m_observations.front().joints,
            fit.centerSensorMm,
            initialParameters);
    }

    const std::array<double, kStateCount> base = packState(initialParameters, initialCenter);
    std::array<double, kStateCount> bestState = base;
    double bestCost = std::numeric_limits<double>::infinity();
    int bestAlternatingIterations = 0;
    int bestJointIterations = 0;

    const int starts = std::max(1, m_options.multiStartCount);
    for (int start = 0; start < starts; ++start) {
        std::array<double, kStateCount> state = deterministicStart(base, m_options, start);
        int alternatingPerformed = 0;
        for (int outer = 0; outer < std::max(0, m_options.alternatingIterations); ++outer) {
            const OptimizeResult machine = optimizeSubset(
                m_observations,
                m_options,
                machineAndCenterIndices(),
                std::max(1, m_options.blockIterations),
                state);
            const OptimizeResult handEye = optimizeSubset(
                m_observations,
                m_options,
                handEyeAndCenterIndices(),
                std::max(1, m_options.blockIterations),
                state);
            ++alternatingPerformed;
            if (machine.iterations <= 1 && handEye.iterations <= 1) {
                break;
            }
        }

        const OptimizeResult joint = optimizeSubset(
            m_observations,
            m_options,
            allIndices(),
            std::max(1, m_options.jointIterations),
            state);
        if (joint.cost < bestCost) {
            bestCost = joint.cost;
            bestState = state;
            bestAlternatingIterations = alternatingPerformed;
            bestJointIterations = joint.iterations;
        }
    }

    unpackState(bestState, result.parameters, result.sphereCenterGaugeMm);
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
    result.profileCount = m_observations.size();
    result.pointCount = pointCount;
    result.observability = analyzeObservability(result.parameters,
                                                result.sphereCenterGaugeMm);
    result.success = result.observability.targetRank ==
                     result.observability.expectedTargetRank &&
                     std::isfinite(result.residualRmsMm);
    if (result.success) {
        result.message = "The 14 observable parameters were identified. Physical hand-eye translation and the traditional four rotary location PIGEs remain inseparable without an external 3-D translation datum.";
    } else {
        result.message = "Optimization completed, but the selected poses do not provide full rank for the 14-D target model.";
    }
    return result;
}

ObservabilityReport SingleSphere14Calibrator::analyzeObservability(
    const Gauge14Parameters& parameters,
    const Vec3d& sphereCenterGaugeMm) const
{
    ObservabilityReport report;
    if (m_observations.empty()) {
        report.message = "No observations.";
        return report;
    }

    const std::array<double, kStateCount> state = packState(parameters, sphereCenterGaugeMm);
    DenseMatrix jac = numericalJacobian(m_observations,
                                        state,
                                        m_options.parameterScales,
                                        0.5 * m_options.sphereDiameterMm,
                                        m_options.finiteDifferenceRelativeStep,
                                        allIndices());
    const std::vector<double> residuals = evaluateResiduals(
        m_observations, state, 0.5 * m_options.sphereDiameterMm);
    const std::vector<double> baseWeights = basePointWeights(m_observations);
    for (int row = 0; row < jac.rows; ++row) {
        const double base = (static_cast<std::size_t>(row) < baseWeights.size())
                                ? baseWeights[static_cast<std::size_t>(row)]
                                : 1.0;
        const double robust = huberIrlsWeight(
            residuals[static_cast<std::size_t>(row)], m_options.huberDeltaMm);
        const double sqrtWeight = std::sqrt(std::max(0.0, base * robust));
        for (int col = 0; col < jac.cols; ++col) {
            jac(row, col) *= sqrtWeight;
        }
    }

    DenseMatrix htt(static_cast<int>(kTargetCount), static_cast<int>(kTargetCount), 0.0);
    DenseMatrix htn(static_cast<int>(kTargetCount), 3, 0.0);
    DenseMatrix hnn(3, 3, 0.0);
    for (int row = 0; row < jac.rows; ++row) {
        for (int a = 0; a < static_cast<int>(kTargetCount); ++a) {
            for (int b = a; b < static_cast<int>(kTargetCount); ++b) {
                htt(a, b) += jac(row, a) * jac(row, b);
            }
            for (int n = 0; n < 3; ++n) {
                htn(a, n) += jac(row, a) * jac(row, static_cast<int>(kTargetCount) + n);
            }
        }
        for (int a = 0; a < 3; ++a) {
            for (int b = a; b < 3; ++b) {
                hnn(a, b) += jac(row, static_cast<int>(kTargetCount) + a) *
                             jac(row, static_cast<int>(kTargetCount) + b);
            }
        }
    }
    for (int a = 0; a < static_cast<int>(kTargetCount); ++a) {
        for (int b = 0; b < a; ++b) {
            htt(a, b) = htt(b, a);
        }
    }
    for (int a = 0; a < 3; ++a) {
        for (int b = 0; b < a; ++b) {
            hnn(a, b) = hnn(b, a);
        }
    }

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

    const std::vector<double> eigenvalues = jacobiEigenvalues(effective);
    if (eigenvalues.size() != kTargetCount) {
        report.message = "Eigenvalue computation failed.";
        return report;
    }
    double largest = 0.0;
    for (std::size_t i = 0; i < kTargetCount; ++i) {
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
    std::ostringstream message;
    message << "Effective target rank " << rank << "/" << kTargetCount
            << ", condition number " << report.conditionNumber << ".";
    report.message = message.str();
    return report;
}

bool SingleSphere14Calibrator::readObservationCsv(
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

bool SingleSphere14Calibrator::writeResultJson(std::ostream& output,
                                               const SingleSphere14Result& result,
                                               std::string& error)
{
    if (!output.good()) {
        error = "Output stream is not writable.";
        return false;
    }
    output << std::setprecision(15);
    output << "{\n";
    output << "  \"version\": \"20.0-single-sphere14\",\n";
    output << "  \"success\": " << (result.success ? "true" : "false") << ",\n";
    output << "  \"model\": \"single_sphere_gauge14\",\n";
    output << "  \"gauge_description\": \""
           << jsonEscape(result.gaugeDescription) << "\",\n";
    output << "  \"message\": \"" << jsonEscape(result.message) << "\",\n";
    output << "  \"parameters\": {\n";
    const auto& names = Gauge14Parameters::names();
    const auto& units = Gauge14Parameters::units();
    for (std::size_t i = 0; i < kTargetCount; ++i) {
        output << "    \"" << names[i] << "\": {\"value\": "
               << result.parameters[i] << ", \"unit\": \"" << units[i] << "\"}";
        output << ((i + 1 < kTargetCount) ? ",\n" : "\n");
    }
    output << "  },\n";
    output << "  \"gauge_handeye_translation_mm\": [0.0, 0.0, 0.0],\n";
    output << "  \"sphere_center_gauge_mm\": ["
           << result.sphereCenterGaugeMm.x << ", "
           << result.sphereCenterGaugeMm.y << ", "
           << result.sphereCenterGaugeMm.z << "],\n";
    output << "  \"residual_rms_mm\": " << result.residualRmsMm << ",\n";
    output << "  \"residual_max_abs_mm\": " << result.residualMaxAbsMm << ",\n";
    output << "  \"profile_count\": " << result.profileCount << ",\n";
    output << "  \"point_count\": " << result.pointCount << ",\n";
    output << "  \"observability\": {\n";
    output << "    \"rank\": " << result.observability.targetRank << ",\n";
    output << "    \"expected_rank\": " << result.observability.expectedTargetRank << ",\n";
    output << "    \"condition_number\": " << result.observability.conditionNumber << ",\n";
    output << "    \"singular_values\": [";
    for (std::size_t i = 0; i < kTargetCount; ++i) {
        if (i > 0) output << ", ";
        output << result.observability.targetSingularValues[i];
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

SingleSphere14PosePlanner::SingleSphere14PosePlanner(const PosePlannerConfig& config)
    : m_config(config)
{
}

bool SingleSphere14PosePlanner::recommendXYZRecentering(
    const JointPose5d& currentJoint,
    const Vec3d& measuredCenterSensorMm,
    const Vec3d& targetCenterSensorMm,
    const Gauge14Parameters& currentEstimate,
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
    const Vec3d centerGauge = SingleSphere14Model::mapSensorPoint(
        currentJoint, measuredCenterSensorMm, currentEstimate);

    constexpr double h = 0.05; // mm; only used to obtain the local Jacobian.
    Vec3d columns[3];
    for (int axis = 0; axis < 3; ++axis) {
        JointPose5d plus = currentJoint;
        JointPose5d minus = currentJoint;
        if (axis == 0) { plus.x += h; minus.x -= h; }
        if (axis == 1) { plus.y += h; minus.y -= h; }
        if (axis == 2) { plus.z += h; minus.z -= h; }
        const Vec3d cp = SingleSphere14Model::sphereCenterInSensor(
            plus, centerGauge, currentEstimate);
        const Vec3d cm = SingleSphere14Model::sphereCenterInSensor(
            minus, centerGauge, currentEstimate);
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
    predictedCenterAfterStepMm = SingleSphere14Model::sphereCenterInSensor(
        suggestedJoint, centerGauge, currentEstimate);
    error.clear();
    return true;
}

std::vector<CalibrationPoseInstruction> SingleSphere14PosePlanner::generate(
    const PosePlannerSeed& seed,
    const Gauge14Parameters& currentEstimate,
    std::string& error) const
{
    std::vector<CalibrationPoseInstruction> output;
    const double radius = 0.5 * m_config.sphereDiameterMm;
    if (!(radius > 0.0) || m_config.requestedRecordedPoses < 1) {
        error = "Invalid sphere diameter or requested pose count.";
        return output;
    }
    if (std::abs(seed.taughtSphereCenterSensorMm.y) >= radius) {
        error = "The taught laser plane does not intersect the 19.05 mm sphere.";
        return output;
    }

    const Vec3d sphereCenterGauge = SingleSphere14Model::mapSensorPoint(
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
    std::vector<double> yValues{radius * m_config.targetCenterYRadiusRatio};
    if (m_config.useBothLaserPlaneSides) {
        yValues.push_back(-radius * m_config.targetCenterYRadiusRatio);
    }

    std::vector<CandidatePose> candidates;
    for (double b : bValues) {
        for (double c : cValues) {
            for (double targetX : m_config.targetCenterXmm) {
                for (double targetY : yValues) {
                    for (double targetZ : m_config.targetCenterZmm) {
                        const Vec3d target(targetX, targetY, targetZ);
                        const double rhoSquared = radius * radius - targetY * targetY;
                        if (!(rhoSquared > 0.0)) {
                            continue;
                        }
                        const double rho = std::sqrt(rhoSquared);
                        if (std::abs(targetX) + 0.90 * rho > m_config.sensorXHalfWidthMm ||
                            targetZ - rho < m_config.sensorZMinMm ||
                            targetZ > m_config.sensorZMaxMm) {
                            continue;
                        }

                        JointPose5d orientationOnly;
                        orientationOnly.b = b;
                        orientationOnly.c = c;
                        const Vec3d rotated = SingleSphere14Model::mapSensorPoint(
                            orientationOnly, target, currentEstimate);
                        Vec3d xyz;
                        if (!solve3x3(axisMatrix, sub(sphereCenterGauge, rotated), xyz)) {
                            continue;
                        }
                        if (xyz.x < m_config.xMinMm || xyz.x > m_config.xMaxMm ||
                            xyz.y < m_config.yMinMm || xyz.y > m_config.yMaxMm ||
                            xyz.z < m_config.zMinMm || xyz.z > m_config.zMaxMm) {
                            continue;
                        }
                        CandidatePose candidate;
                        candidate.joint.x = xyz.x;
                        candidate.joint.y = xyz.y;
                        candidate.joint.z = xyz.z;
                        candidate.joint.b = b;
                        candidate.joint.c = c;
                        candidate.targetCenterSensor = target;
                        candidate.syntheticObservation = syntheticProfile(
                            candidate.joint,
                            target,
                            radius,
                            m_config.syntheticPointsPerProfile);
                        if (candidate.syntheticObservation.points.size() < 5) {
                            continue;
                        }
                        candidate.information = candidateInformation(
                            candidate,
                            currentEstimate,
                            sphereCenterGauge,
                            radius,
                            1e-5);
                        candidates.push_back(candidate);
                    }
                }
            }
        }
    }

    if (candidates.empty()) {
        error = "No feasible candidate pose inside the configured sensor field.";
        return output;
    }

    DenseMatrix accumulated(static_cast<int>(kStateCount),
                            static_cast<int>(kStateCount), 0.0);
    for (int i = 0; i < accumulated.rows; ++i) {
        accumulated(i, i) = std::max(1e-12, m_config.informationRidge);
    }

    std::vector<int> selected;
    std::vector<bool> used(candidates.size(), false);
    std::map<std::pair<int, int>, int> bcCounts;

    // Add the candidate nearest to the taught reference first.
    int referenceIndex = 0;
    double referenceDistance = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < candidates.size(); ++i) {
        const double db = candidates[i].joint.b - seed.taughtJoint.b;
        const double dc = candidates[i].joint.c - seed.taughtJoint.c;
        const Vec3d ds = sub(candidates[i].targetCenterSensor,
                             seed.taughtSphereCenterSensorMm);
        const double distance = db * db + dc * dc + dot(ds, ds);
        if (distance < referenceDistance) {
            referenceDistance = distance;
            referenceIndex = static_cast<int>(i);
        }
    }
    selected.push_back(referenceIndex);
    used[static_cast<std::size_t>(referenceIndex)] = true;
    accumulated = addDense(accumulated, candidates[static_cast<std::size_t>(referenceIndex)].information);
    bcCounts[{static_cast<int>(std::lround(candidates[static_cast<std::size_t>(referenceIndex)].joint.b * 1000.0)),
              static_cast<int>(std::lround(candidates[static_cast<std::size_t>(referenceIndex)].joint.c * 1000.0))}] = 1;

    const int targetCount = std::min(m_config.requestedRecordedPoses,
                                     static_cast<int>(candidates.size()));
    while (static_cast<int>(selected.size()) < targetCount) {
        int best = -1;
        double bestScore = -std::numeric_limits<double>::infinity();
        const double previousLogDet = logDetPositiveDefinite(
            targetSchurInformation(accumulated));
        for (std::size_t i = 0; i < candidates.size(); ++i) {
            if (used[i]) {
                continue;
            }
            const auto key = std::make_pair(
                static_cast<int>(std::lround(candidates[i].joint.b * 1000.0)),
                static_cast<int>(std::lround(candidates[i].joint.c * 1000.0)));
            if (bcCounts[key] >= std::max(1, m_config.maxRecordedPosesPerBCPair)) {
                continue;
            }
            const DenseMatrix trial = addDense(accumulated, candidates[i].information);
            const double score = logDetPositiveDefinite(targetSchurInformation(trial));
            if (score > bestScore) {
                bestScore = score;
                best = static_cast<int>(i);
            }
        }
        if (best < 0) {
            break;
        }
        candidates[static_cast<std::size_t>(best)].informationGain =
            bestScore - previousLogDet;
        selected.push_back(best);
        used[static_cast<std::size_t>(best)] = true;
        accumulated = addDense(accumulated,
                               candidates[static_cast<std::size_t>(best)].information);
        const auto key = std::make_pair(
            static_cast<int>(std::lround(candidates[static_cast<std::size_t>(best)].joint.b * 1000.0)),
            static_cast<int>(std::lround(candidates[static_cast<std::size_t>(best)].joint.c * 1000.0)));
        ++bcCounts[key];
    }

    // Reorder selected recorded poses to reduce B/C travel. D-optimal membership is unchanged.
    std::vector<int> ordered;
    ordered.push_back(selected.front());
    selected.erase(selected.begin());
    while (!selected.empty()) {
        const CandidatePose& current = candidates[static_cast<std::size_t>(ordered.back())];
        auto bestIt = selected.begin();
        double bestDistance = std::numeric_limits<double>::infinity();
        for (auto it = selected.begin(); it != selected.end(); ++it) {
            const double distance = acquisitionDistance(
                current, candidates[static_cast<std::size_t>(*it)]);
            if (distance < bestDistance) {
                bestDistance = distance;
                bestIt = it;
            }
        }
        ordered.push_back(*bestIt);
        selected.erase(bestIt);
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

    error.clear();
    return output;
}

bool SingleSphere14PosePlanner::writeCsv(
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
