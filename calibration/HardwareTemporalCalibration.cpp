#include "calibration/HardwareTemporalCalibration.h"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <sstream>

namespace mls::calibration {
namespace {

bool finite(double v) { return std::isfinite(v); }
bool finiteVec(const Vec3d& v) { return finite(v.x) && finite(v.y) && finite(v.z); }

double coord(const Vec3d& p, TemporalAxis axis)
{
    switch (axis) {
    case TemporalAxis::X: return p.x;
    case TemporalAxis::Y: return p.y;
    case TemporalAxis::Z: return p.z;
    }
    return p.x;
}

double median(std::vector<double> v)
{
    if (v.empty()) return 0.0;
    const auto mid = v.begin() + static_cast<std::ptrdiff_t>(v.size() / 2);
    std::nth_element(v.begin(), mid, v.end());
    double m = *mid;
    if ((v.size() & 1u) == 0u) {
        const auto lower = std::max_element(v.begin(), mid);
        m = 0.5 * (m + *lower);
    }
    return m;
}

double percentile(std::vector<double> v, double p)
{
    if (v.empty()) return 0.0;
    p = std::clamp(p, 0.0, 1.0);
    const double pos = p * static_cast<double>(v.size() - 1);
    const std::size_t lo = static_cast<std::size_t>(std::floor(pos));
    const std::size_t hi = static_cast<std::size_t>(std::ceil(pos));
    std::nth_element(v.begin(), v.begin() + static_cast<std::ptrdiff_t>(lo), v.end());
    const double a = v[lo];
    if (hi == lo) return a;
    std::nth_element(v.begin(), v.begin() + static_cast<std::ptrdiff_t>(hi), v.end());
    return a + (v[hi] - a) * (pos - static_cast<double>(lo));
}

bool solve3x3(double a[3][3], double b[3], double x[3])
{
    double m[3][4]{};
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) m[r][c] = a[r][c];
        m[r][3] = b[r];
    }
    for (int c = 0; c < 3; ++c) {
        int pivot = c;
        for (int r = c + 1; r < 3; ++r)
            if (std::abs(m[r][c]) > std::abs(m[pivot][c])) pivot = r;
        if (std::abs(m[pivot][c]) < 1e-14) return false;
        if (pivot != c) for (int k = c; k < 4; ++k) std::swap(m[c][k], m[pivot][k]);
        const double d = m[c][c];
        for (int k = c; k < 4; ++k) m[c][k] /= d;
        for (int r = 0; r < 3; ++r) {
            if (r == c) continue;
            const double f = m[r][c];
            for (int k = c; k < 4; ++k) m[r][k] -= f * m[c][k];
        }
    }
    for (int i = 0; i < 3; ++i) x[i] = m[i][3];
    return true;
}


bool solveDense(std::vector<std::vector<double>> a, std::vector<double> b,
                std::vector<double>& x)
{
    const std::size_t n=a.size();
    if(n==0||b.size()!=n)return false;
    for(const auto& row:a)if(row.size()!=n)return false;
    for(std::size_t c=0;c<n;++c){
        std::size_t pivot=c;
        for(std::size_t r=c+1;r<n;++r)if(std::abs(a[r][c])>std::abs(a[pivot][c]))pivot=r;
        if(std::abs(a[pivot][c])<1e-16)return false;
        if(pivot!=c){std::swap(a[pivot],a[c]);std::swap(b[pivot],b[c]);}
        const double d=a[c][c];
        for(std::size_t k=c;k<n;++k)a[c][k]/=d;b[c]/=d;
        for(std::size_t r=0;r<n;++r){
            if(r==c)continue;
            const double f=a[r][c];
            if(std::abs(f)<1e-30)continue;
            for(std::size_t k=c;k<n;++k)a[r][k]-=f*a[c][k];
            b[r]-=f*b[c];
        }
    }
    x=b;return true;
}

struct DriftInput { Vec3d center; Vec3d velocity; std::int64_t timeUs=0; };
struct DriftFit {
    bool success=false;
    double delaySec=0.0;
    double delayStdSec=std::numeric_limits<double>::infinity();
    Vec3d driftMmPerSec;
    double residualRmsMm=std::numeric_limits<double>::infinity();
    double rSquared=0.0;
    double timeSpanSec=0.0;
};

DriftFit fitCenterDriftAndDelay(const std::vector<DriftInput>& in)
{
    DriftFit out;
    if(in.size()<4)return out;
    std::int64_t minUs=std::numeric_limits<std::int64_t>::max();
    std::int64_t maxUs=std::numeric_limits<std::int64_t>::min();
    long double sumUs=0.0L;
    for(const auto& o:in){
        if(o.timeUs<=0||!finiteVec(o.center)||!finiteVec(o.velocity))return out;
        minUs=std::min(minUs,o.timeUs);maxUs=std::max(maxUs,o.timeUs);sumUs+=o.timeUs;
    }
    const double meanUs=static_cast<double>(sumUs/static_cast<long double>(in.size()));
    out.timeSpanSec=double(maxUs-minUs)*1e-6;
    if(!(out.timeSpanSec>0.5))return out;

    constexpr std::size_t P=7; // bx by bz gx gy gz tau
    std::vector<std::vector<double>> h(P,std::vector<double>(P,0.0));
    std::vector<double> rhs(P,0.0);
    Vec3d meanCenter{};
    for(const auto&o:in){meanCenter.x+=o.center.x;meanCenter.y+=o.center.y;meanCenter.z+=o.center.z;}
    meanCenter.x/=double(in.size());meanCenter.y/=double(in.size());meanCenter.z/=double(in.size());
    auto accumulate=[&](int axis,double center,double vel,double t){
        double row[P]{};row[axis]=1.0;row[3+axis]=t;row[6]=-vel;
        for(std::size_t a=0;a<P;++a){rhs[a]+=row[a]*center;for(std::size_t b=0;b<P;++b)h[a][b]+=row[a]*row[b];}
    };
    for(const auto&o:in){
        const double t=(double(o.timeUs)-meanUs)*1e-6;
        accumulate(0,o.center.x,o.velocity.x,t);
        accumulate(1,o.center.y,o.velocity.y,t);
        accumulate(2,o.center.z,o.velocity.z,t);
    }
    std::vector<double> beta;
    if(!solveDense(h,rhs,beta)||beta.size()!=P)return out;
    out.delaySec=beta[6];out.driftMmPerSec={beta[3],beta[4],beta[5]};
    double sse=0.0,sst=0.0;
    for(const auto&o:in){
        const double t=(double(o.timeUs)-meanUs)*1e-6;
        const Vec3d pred{beta[0]+beta[3]*t-beta[6]*o.velocity.x,
                         beta[1]+beta[4]*t-beta[6]*o.velocity.y,
                         beta[2]+beta[5]*t-beta[6]*o.velocity.z};
        const double ex=o.center.x-pred.x,ey=o.center.y-pred.y,ez=o.center.z-pred.z;
        sse+=ex*ex+ey*ey+ez*ez;
        const double dx=o.center.x-meanCenter.x,dy=o.center.y-meanCenter.y,dz=o.center.z-meanCenter.z;
        sst+=dx*dx+dy*dy+dz*dz;
    }
    out.residualRmsMm=std::sqrt(sse/std::max<std::size_t>(1,in.size()));
    out.rSquared=sst>1e-18?std::max(-1.0,1.0-sse/sst):0.0;
    const std::ptrdiff_t dof=static_cast<std::ptrdiff_t>(3*in.size())-static_cast<std::ptrdiff_t>(P);
    if(dof>0){
        std::vector<double> e(P,0.0),col;e[6]=1.0;
        if(solveDense(h,e,col)&&col.size()==P&&col[6]>=0.0){
            const double sigma2=sse/static_cast<double>(dof);
            out.delayStdSec=std::sqrt(std::max(0.0,sigma2*col[6]));
        }
    }
    out.success=finite(out.delaySec)&&finite(out.residualRmsMm)&&finiteVec(out.driftMmPerSec);
    return out;
}

TemporalAxisFit fitAxis(const std::vector<HardwareTemporalObservation>& observations,
                        TemporalAxis axis)
{
    TemporalAxisFit out;
    out.axis = axis;
    std::vector<std::pair<double,double>> xy;
    for (const auto& o : observations) {
        if (o.axis != axis || !finite(o.signedSpeedMmPerSec) || !finiteVec(o.sphereCenterMachineMm)) continue;
        xy.emplace_back(o.signedSpeedMmPerSec, coord(o.sphereCenterMachineMm, axis));
        if (o.signedSpeedMmPerSec > 0.0) ++out.positiveCount;
        if (o.signedSpeedMmPerSec < 0.0) ++out.negativeCount;
    }
    out.observationCount = xy.size();
    if (xy.size() < 2 || out.positiveCount == 0 || out.negativeCount == 0) return out;
    {
        std::vector<double> levels;
        for (const auto& p : xy) {
            const double a = std::abs(p.first);
            bool seen = false;
            for (double b : levels) {
                const double scale = std::max({a, b, 1e-9});
                if (std::abs(a-b) <= 0.03*scale) { seen = true; break; }
            }
            if (!seen) levels.push_back(a);
        }
        out.distinctAbsSpeedLevels = levels.size();
    }

    out.minSignedSpeedMmPerSec = xy.front().first;
    out.maxSignedSpeedMmPerSec = xy.front().first;
    double mv = 0.0, my = 0.0;
    for (const auto& p : xy) {
        mv += p.first; my += p.second;
        out.minSignedSpeedMmPerSec = std::min(out.minSignedSpeedMmPerSec, p.first);
        out.maxSignedSpeedMmPerSec = std::max(out.maxSignedSpeedMmPerSec, p.first);
    }
    mv /= static_cast<double>(xy.size());
    my /= static_cast<double>(xy.size());
    double sxx = 0.0, sxy = 0.0, syy = 0.0;
    for (const auto& p : xy) {
        const double dv = p.first - mv;
        const double dy = p.second - my;
        sxx += dv * dv; sxy += dv * dy; syy += dy * dy;
    }
    if (sxx <= 1e-12) return out;
    out.slopeSec = sxy / sxx;
    out.delaySec = -out.slopeSec;
    out.interceptMm = my - out.slopeSec * mv;
    double sse = 0.0;
    for (const auto& p : xy) {
        const double e = p.second - (out.interceptMm + out.slopeSec * p.first);
        sse += e * e;
    }
    out.residualRmsMm = std::sqrt(sse / static_cast<double>(xy.size()));
    out.rSquared = syy > 1e-18 ? std::max(-1.0, 1.0 - sse / syy) : 0.0;
    if (xy.size() > 2) {
        const double variance = sse / static_cast<double>(xy.size() - 2);
        out.delayStdSec = std::sqrt(std::max(0.0, variance / sxx));
    }
    out.valid = finite(out.delaySec) && finite(out.interceptMm) && finite(out.residualRmsMm);
    return out;
}

} // namespace

const char* temporalAxisName(TemporalAxis axis)
{
    switch (axis) {
    case TemporalAxis::X: return "X";
    case TemporalAxis::Y: return "Y";
    case TemporalAxis::Z: return "Z";
    }
    return "?";
}

TemporalMotionEstimate HardwareTemporalCalibration::estimateDominantLinearMotion(
    const std::vector<TemporalPoseSample>& poses, double triggerPeriodSec)
{
    TemporalMotionEstimate out;
    if (poses.size() < 10 || !finite(triggerPeriodSec) || triggerPeriodSec <= 0.0) {
        out.message = "Too few pose samples or invalid trigger period.";
        return out;
    }

    struct V { double x=0,y=0,z=0,c=0,b=0; };
    std::vector<V> vel;
    vel.reserve(poses.size());
    for (std::size_t i = 1; i < poses.size(); ++i) {
        const auto& a = poses[i-1];
        const auto& b = poses[i];
        // ordinal 0 is the one-shot Probe; the 0->1 handshake gap is NOT 5 ms.
        if (a.ordinal < 1 || b.ordinal != a.ordinal + 1) continue;
        bool ok = true;
        for (double q : a.joints) ok = ok && finite(q);
        for (double q : b.joints) ok = ok && finite(q);
        if (!ok) continue;
        V v;
        v.x = (b.joints[0] - a.joints[0]) / triggerPeriodSec;
        v.y = (b.joints[1] - a.joints[1]) / triggerPeriodSec;
        v.z = (b.joints[2] - a.joints[2]) / triggerPeriodSec;
        v.c = (b.joints[3] - a.joints[3]) / triggerPeriodSec;
        v.b = (b.joints[4] - a.joints[4]) / triggerPeriodSec;
        vel.push_back(v);
    }
    out.totalIntervals = vel.size();
    if (vel.size() < 8) { out.message = "Too few consecutive continuous-stage latch intervals."; return out; }

    std::array<std::vector<double>,3> absLinear;
    std::vector<double> absRotary;
    for (const auto& v : vel) {
        absLinear[0].push_back(std::abs(v.x));
        absLinear[1].push_back(std::abs(v.y));
        absLinear[2].push_back(std::abs(v.z));
        absRotary.push_back(std::max(std::abs(v.c), std::abs(v.b)));
    }
    std::array<double,3> p80{};
    for (int k = 0; k < 3; ++k) p80[k] = percentile(absLinear[k], 0.80);
    int dominant = 0;
    if (p80[1] > p80[dominant]) dominant = 1;
    if (p80[2] > p80[dominant]) dominant = 2;
    const double scale = p80[dominant];
    const double second = std::max(p80[(dominant+1)%3], p80[(dominant+2)%3]);
    out.axis = static_cast<TemporalAxis>(dominant);
    out.crossAxisRatio = scale > 1e-12 ? second / scale : std::numeric_limits<double>::infinity();
    out.rotaryP80DegPerSec = percentile(absRotary, 0.80);
    if (!(scale >= 0.05)) { out.message = "No sufficiently fast single-axis linear motion found."; return out; }

    auto dominantV = [dominant](const V& v) {
        return dominant == 0 ? v.x : (dominant == 1 ? v.y : v.z);
    };
    auto crossV = [dominant](const V& v) {
        if (dominant == 0) return std::max(std::abs(v.y), std::abs(v.z));
        if (dominant == 1) return std::max(std::abs(v.x), std::abs(v.z));
        return std::max(std::abs(v.x), std::abs(v.y));
    };

    std::vector<double> firstPass;
    for (const auto& v : vel) {
        const double d = dominantV(v);
        if (std::abs(d) < 0.50 * scale) continue;
        if (crossV(v) > 0.25 * std::abs(d)) continue;
        if (std::max(std::abs(v.c), std::abs(v.b)) > 0.20) continue;
        firstPass.push_back(d);
    }
    if (firstPass.size() < 8) { out.message = "Constant-speed single-axis segment is too short/impure."; return out; }
    const double seed = median(firstPass);
    if (std::abs(seed) < 0.05) { out.message = "Robust signed speed is too small."; return out; }

    std::vector<double> accepted;
    const double tolerance = std::max(0.02, 0.25 * std::abs(seed));
    for (double v : firstPass) {
        if ((v > 0.0) != (seed > 0.0)) continue;
        if (std::abs(v - seed) <= tolerance) accepted.push_back(v);
    }
    if (accepted.size() < 8) { out.message = "Too few intervals remain after constant-speed filtering."; return out; }
    const double speed = median(accepted);
    std::vector<double> deviations; deviations.reserve(accepted.size());
    for (double v : accepted) deviations.push_back(std::abs(v - speed));
    const double mad = median(deviations);

    out.signedSpeedMmPerSec = speed;
    out.speedMadMmPerSec = mad;
    out.speedMadFraction = mad / std::max(std::abs(speed), 1e-12);
    out.usedIntervals = accepted.size();

    if (out.crossAxisRatio > 0.20) {
        out.message = "Scan is not single-axis enough (cross-axis velocity too large).";
        return out;
    }
    if (out.rotaryP80DegPerSec > 0.20) {
        out.message = "B/C moved during the linear temporal-calibration scan.";
        return out;
    }
    if (out.speedMadFraction > 0.15) {
        out.message = "Selected motion segment is not sufficiently constant-speed.";
        return out;
    }

    out.success = true;
    std::ostringstream os;
    os << "OK: axis=" << temporalAxisName(out.axis)
       << ", v=" << out.signedSpeedMmPerSec << " mm/s, intervals="
       << out.usedIntervals << "/" << out.totalIntervals;
    out.message = os.str();
    return out;
}

KnownSphereCenterFit HardwareTemporalCalibration::refineKnownSphereCenter(
    const std::vector<Vec3d>& points, const Vec3d& initialCenterMachineMm,
    double knownRadiusMm, double huberThresholdMm, int maxIterations)
{
    KnownSphereCenterFit out;
    out.knownRadiusMm = knownRadiusMm;
    out.pointCount = points.size();
    if (points.size() < 4 || !finiteVec(initialCenterMachineMm) ||
        !finite(knownRadiusMm) || knownRadiusMm <= 0.0 ||
        !finite(huberThresholdMm) || huberThresholdMm <= 0.0 || maxIterations < 1) {
        out.message = "Invalid known-sphere refinement input.";
        return out;
    }

    Vec3d c = initialCenterMachineMm;
    for (int it = 0; it < maxIterations; ++it) {
        double h[3][3]{};
        double g[3]{};
        std::size_t used = 0;
        for (const auto& p : points) {
            if (!finiteVec(p)) continue;
            const double dx = c.x - p.x, dy = c.y - p.y, dz = c.z - p.z;
            const double d = std::sqrt(dx*dx + dy*dy + dz*dz);
            if (!(d > 1e-12) || !finite(d)) continue;
            const double r = d - knownRadiusMm;
            const double ar = std::abs(r);
            const double w = ar <= huberThresholdMm ? 1.0 : huberThresholdMm / ar;
            const double j[3] = {dx/d, dy/d, dz/d};
            for (int a = 0; a < 3; ++a) {
                g[a] += w * j[a] * r;
                for (int b = 0; b < 3; ++b) h[a][b] += w * j[a] * j[b];
            }
            ++used;
        }
        if (used < 4) { out.message = "Too few finite sphere inliers."; return out; }
        double rhs[3] = {-g[0], -g[1], -g[2]};
        double step[3]{};
        if (!solve3x3(h, rhs, step)) { out.message = "Known-sphere center normal matrix is singular."; return out; }
        c.x += step[0]; c.y += step[1]; c.z += step[2];
        if (std::sqrt(step[0]*step[0] + step[1]*step[1] + step[2]*step[2]) < 1e-10) break;
    }

    double sse = 0.0, maxAbs = 0.0;
    std::size_t n = 0;
    for (const auto& p : points) {
        if (!finiteVec(p)) continue;
        const double dx = p.x-c.x, dy=p.y-c.y, dz=p.z-c.z;
        const double e = std::sqrt(dx*dx+dy*dy+dz*dz) - knownRadiusMm;
        if (!finite(e)) continue;
        sse += e*e; maxAbs = std::max(maxAbs, std::abs(e)); ++n;
    }
    if (n < 4) { out.message = "Known-sphere residual evaluation failed."; return out; }
    out.success = true;
    out.centerMachineMm = c;
    out.residualRmsMm = std::sqrt(sse/static_cast<double>(n));
    out.residualMaxAbsMm = maxAbs;
    std::ostringstream os;
    os << "OK: fixed R=" << knownRadiusMm << " mm, center=[" << c.x << "," << c.y << "," << c.z
       << "], RMS=" << out.residualRmsMm << " mm, N=" << n;
    out.message = os.str();
    return out;
}


FreeSphereFit HardwareTemporalCalibration::refineFreeSphere(
    const std::vector<Vec3d>& points, const Vec3d& initialCenterMachineMm,
    double initialRadiusMm, double huberThresholdMm, int maxIterations)
{
    FreeSphereFit out;
    out.pointCount=points.size();
    if(points.size()<4||!finiteVec(initialCenterMachineMm)||!finite(initialRadiusMm)||initialRadiusMm<=0.0||
       !finite(huberThresholdMm)||huberThresholdMm<=0.0||maxIterations<1){
        out.message="Invalid free-sphere refinement input.";return out;
    }
    Vec3d c=initialCenterMachineMm;double radius=initialRadiusMm;
    for(int it=0;it<maxIterations;++it){
        std::vector<std::vector<double>> h(4,std::vector<double>(4,0.0));
        std::vector<double> g(4,0.0);std::size_t used=0;
        for(const auto& p:points){
            if(!finiteVec(p))continue;
            const double dx=c.x-p.x,dy=c.y-p.y,dz=c.z-p.z;
            const double d=std::sqrt(dx*dx+dy*dy+dz*dz);
            if(!(d>1e-12)||!finite(d))continue;
            const double r=d-radius,ar=std::abs(r);
            const double w=ar<=huberThresholdMm?1.0:huberThresholdMm/ar;
            const double j[4]={dx/d,dy/d,dz/d,-1.0};
            for(int a=0;a<4;++a){g[a]+=w*j[a]*r;for(int b=0;b<4;++b)h[a][b]+=w*j[a]*j[b];}
            ++used;
        }
        if(used<4){out.message="Too few finite free-sphere inliers.";return out;}
        std::vector<double> rhs(4),step;for(int i=0;i<4;++i)rhs[std::size_t(i)]=-g[std::size_t(i)];
        if(!solveDense(h,rhs,step)||step.size()!=4){out.message="Free-sphere normal matrix is singular.";return out;}
        c.x+=step[0];c.y+=step[1];c.z+=step[2];radius+=step[3];
        if(!(radius>0.0)||!finite(radius)){out.message="Free-sphere radius became invalid.";return out;}
        const double dn=std::sqrt(step[0]*step[0]+step[1]*step[1]+step[2]*step[2]+step[3]*step[3]);
        if(dn<1e-10)break;
    }
    double sse=0.0,maxAbs=0.0;std::size_t n=0;
    for(const auto& p:points){
        if(!finiteVec(p))continue;
        const double dx=p.x-c.x,dy=p.y-c.y,dz=p.z-c.z;
        const double e=std::sqrt(dx*dx+dy*dy+dz*dz)-radius;
        if(!finite(e))continue;sse+=e*e;maxAbs=std::max(maxAbs,std::abs(e));++n;
    }
    if(n<4){out.message="Free-sphere residual evaluation failed.";return out;}
    out.success=true;out.centerMachineMm=c;out.radiusMm=radius;
    out.residualRmsMm=std::sqrt(sse/static_cast<double>(n));out.residualMaxAbsMm=maxAbs;
    std::ostringstream os;os<<"OK: free R="<<radius<<" mm, center=["<<c.x<<","<<c.y<<","<<c.z
        <<"], RMS="<<out.residualRmsMm<<" mm, N="<<n;out.message=os.str();return out;
}

HardwareTemporalCalibrationResult HardwareTemporalCalibration::solve(
    const std::vector<HardwareTemporalObservation>& observations)
{
    HardwareTemporalCalibrationResult out;
    out.observationCount = observations.size();
    if (observations.empty()) { out.message = "No temporal-calibration observations."; return out; }

    const auto& first = observations.front();
    out.resolutionIndex = first.resolutionIndex;
    out.exposureTime10Us = first.exposureTime10Us;
    out.idleTime10Us = first.idleTime10Us;
    out.measuringFieldIndex = first.measuringFieldIndex;
    out.triggerFrequencyHz = first.triggerFrequencyHz;
    out.triggerPeriodMs = first.triggerPeriodMs;
    out.configurationConsistent = first.triggerFrequencyHz > 0 && first.triggerPeriodMs > 0.0;
    for (const auto& o : observations) {
        const bool finiteObs = finite(o.signedSpeedMmPerSec) && std::abs(o.signedSpeedMmPerSec) > 1e-12
            && finiteVec(o.sphereCenterMachineMm) && finite(o.sphereRmsMm) && o.sphereRmsMm >= 0.0;
        if (!finiteObs || o.resolutionIndex != out.resolutionIndex ||
            o.exposureTime10Us != out.exposureTime10Us || o.idleTime10Us != out.idleTime10Us ||
            o.measuringFieldIndex != out.measuringFieldIndex || o.triggerFrequencyHz != out.triggerFrequencyHz ||
            std::abs(o.triggerPeriodMs - out.triggerPeriodMs) > 1e-9) {
            out.configurationConsistent = false;
        }
    }
    if (!out.configurationConsistent) {
        out.message = "Observations mix invalid or different sensor/trigger configurations.";
        return out;
    }

    out.axes[0] = fitAxis(observations, TemporalAxis::X);
    out.axes[1] = fitAxis(observations, TemporalAxis::Y);
    out.axes[2] = fitAxis(observations, TemporalAxis::Z);

    // Common-slope regression with one nuisance intercept per active axis.
    struct Group { TemporalAxis axis; std::vector<std::pair<double,double>> xy; double mv=0,my=0; };
    std::vector<Group> groups;
    for (const auto& af : out.axes) {
        if (!af.valid) continue;
        Group g; g.axis = af.axis;
        for (const auto& o : observations) {
            if (o.axis != af.axis) continue;
            g.xy.emplace_back(o.signedSpeedMmPerSec, coord(o.sphereCenterMachineMm, o.axis));
            g.mv += o.signedSpeedMmPerSec; g.my += coord(o.sphereCenterMachineMm, o.axis);
        }
        if (g.xy.empty()) continue;
        g.mv /= static_cast<double>(g.xy.size());
        g.my /= static_cast<double>(g.xy.size());
        groups.push_back(std::move(g));
    }
    out.activeAxisCount = groups.size();
    if (groups.empty()) { out.message = "No axis has both positive and negative usable scans."; return out; }

    double sxx = 0.0, sxy = 0.0;
    std::size_t n = 0;
    for (const auto& g : groups) for (const auto& p : g.xy) {
        const double dv = p.first - g.mv;
        const double dy = p.second - g.my;
        sxx += dv*dv; sxy += dv*dy; ++n;
    }
    if (sxx <= 1e-12) { out.message = "Insufficient signed-speed span for temporal regression."; return out; }
    const double slope = sxy / sxx;
    out.delaySec = -slope;

    double sse = 0.0;
    for (const auto& g : groups) {
        const double intercept = g.my - slope * g.mv;
        for (const auto& p : g.xy) {
            const double e = p.second - (intercept + slope*p.first);
            sse += e*e;
        }
    }
    out.residualRmsMm = std::sqrt(sse / static_cast<double>(std::max<std::size_t>(1,n)));
    const std::ptrdiff_t dof = static_cast<std::ptrdiff_t>(n) - static_cast<std::ptrdiff_t>(groups.size()) - 1;
    if (dof > 0) out.delayStdSec = std::sqrt(std::max(0.0, (sse/static_cast<double>(dof))/sxx));

    double minDelay = std::numeric_limits<double>::infinity();
    double maxDelay = -std::numeric_limits<double>::infinity();
    for (const auto& af : out.axes) if (af.valid) {
        minDelay = std::min(minDelay, af.delaySec);
        maxDelay = std::max(maxDelay, af.delaySec);
    }
    out.axisSpreadSec = maxDelay - minDelay;
    out.success = finite(out.delaySec) && finite(out.residualRmsMm);
    std::ostringstream os;
    os << "OK: common delay=" << out.delaySec*1e6 << " us, axes=" << out.activeAxisCount
       << ", observations=" << n << ", residualRMS=" << out.residualRmsMm << " mm.";
    out.message = os.str();
    return out;
}

bool HardwareTemporalCalibration::evaluateQuality(HardwareTemporalCalibrationResult& r,
                                                    const HardwareTemporalCriteria& c)
{
    r.qualityPassed = false;
    if (!r.success || !r.configurationConsistent) {
        r.qualityMessage = "Solver/configuration is not valid.";
        return false;
    }
    if (r.activeAxisCount < c.minAxes) {
        r.qualityMessage = "Not enough independently scanned XYZ axes.";
        return false;
    }
    for (const auto& a : r.axes) {
        if (!a.valid) continue;
        if (a.observationCount < c.minObservationsPerAxis || a.positiveCount == 0 || a.negativeCount == 0) {
            r.qualityMessage = std::string("Axis ") + temporalAxisName(a.axis) + " has too few +/- observations.";
            return false;
        }
        if (a.distinctAbsSpeedLevels < 2) {
            r.qualityMessage = std::string("Axis ") + temporalAxisName(a.axis) + " needs at least two distinct absolute speed levels.";
            return false;
        }
        if (a.rSquared < c.minRSquared) {
            r.qualityMessage = std::string("Axis ") + temporalAxisName(a.axis) + " speed/center regression R^2 is too low.";
            return false;
        }
    }
    const double periodSec = r.triggerFrequencyHz > 0
        ? 1.0 / static_cast<double>(r.triggerFrequencyHz)
        : r.triggerPeriodMs / 1000.0;
    if (!(r.delaySec >= 0.0) || !(r.delaySec < periodSec*c.maxDelayFractionOfPeriod)) {
        r.qualityMessage = "Delay is outside the supported current-topology interval [0, trigger period).";
        return false;
    }
    if (!finite(r.axisSpreadSec) || r.axisSpreadSec > c.maxAxisSpreadSec) {
        r.qualityMessage = "XYZ axis delay estimates disagree beyond the configured gate.";
        return false;
    }
    if (!finite(r.delayStdSec) || r.delayStdSec > c.maxDelayStdSec) {
        r.qualityMessage = "Common delay statistical uncertainty exceeds the configured gate.";
        return false;
    }
    r.qualityPassed = true;
    r.qualityMessage = "PASS: fixed-delay model is consistent across XYZ for the current sensor/trigger configuration.";
    return true;
}


TemporalMotionVectorEstimate HardwareTemporalCalibration::estimateLinearMotionVector(
    const std::vector<TemporalPoseSample>& poses, double triggerPeriodSec)
{
    TemporalMotionVectorEstimate out;
    if (poses.size() < 10 || !finite(triggerPeriodSec) || triggerPeriodSec <= 0.0) {
        out.message = "Too few pose samples or invalid trigger period.";
        return out;
    }
    struct V { double x=0,y=0,z=0,c=0,b=0; };
    std::vector<V> vel;
    vel.reserve(poses.size());
    for (std::size_t i=1;i<poses.size();++i) {
        const auto& a=poses[i-1]; const auto& b=poses[i];
        if (a.ordinal < 1 || b.ordinal != a.ordinal + 1) continue; // Probe gap is not 5 ms.
        bool ok=true; for(double q:a.joints) ok=ok&&finite(q); for(double q:b.joints) ok=ok&&finite(q);
        if(!ok) continue;
        vel.push_back({(b.joints[0]-a.joints[0])/triggerPeriodSec,
                       (b.joints[1]-a.joints[1])/triggerPeriodSec,
                       (b.joints[2]-a.joints[2])/triggerPeriodSec,
                       (b.joints[3]-a.joints[3])/triggerPeriodSec,
                       (b.joints[4]-a.joints[4])/triggerPeriodSec});
    }
    out.totalIntervals=vel.size();
    if(vel.size()<8){out.message="Too few consecutive continuous-stage latch intervals.";return out;}

    std::vector<double> xs,ys,zs,rot;
    xs.reserve(vel.size());ys.reserve(vel.size());zs.reserve(vel.size());rot.reserve(vel.size());
    for(const auto& v:vel){xs.push_back(v.x);ys.push_back(v.y);zs.push_back(v.z);rot.push_back(std::max(std::abs(v.c),std::abs(v.b)));}
    Vec3d seed{median(xs),median(ys),median(zs)};
    const double seedNorm=std::sqrt(seed.x*seed.x+seed.y*seed.y+seed.z*seed.z);
    out.rotaryP80DegPerSec=percentile(rot,0.80);
    if(!(seedNorm>=0.05)){out.message="No sufficiently fast XYZ straight-line motion found.";return out;}
    const Vec3d u{seed.x/seedNorm,seed.y/seedNorm,seed.z/seedNorm};

    std::vector<V> accepted;
    std::vector<double> along;
    std::vector<double> lateralFractions;
    for(const auto& v:vel){
        const double proj=v.x*u.x+v.y*u.y+v.z*u.z;
        if(proj < 0.50*seedNorm) continue; // same direction and sufficiently inside constant-speed plateau
        const double lx=v.x-proj*u.x, ly=v.y-proj*u.y, lz=v.z-proj*u.z;
        const double lateral=std::sqrt(lx*lx+ly*ly+lz*lz);
        if(lateral > 0.25*std::max(std::abs(proj),1e-12)) continue;
        if(std::max(std::abs(v.c),std::abs(v.b))>0.20) continue;
        accepted.push_back(v);along.push_back(proj);lateralFractions.push_back(lateral/std::max(std::abs(proj),1e-12));
    }
    if(accepted.size()<8){out.message="Constant-speed XYZ line segment is too short/impure.";return out;}
    const double alongMed=median(along);
    std::vector<double> dev;dev.reserve(along.size());for(double a:along)dev.push_back(std::abs(a-alongMed));
    const double mad=median(dev);
    out.speedMmPerSec=alongMed;
    out.speedMadMmPerSec=mad;
    out.speedMadFraction=mad/std::max(std::abs(alongMed),1e-12);
    out.crossTrackP80Fraction=percentile(lateralFractions,0.80);
    out.usedIntervals=accepted.size();
    if(out.rotaryP80DegPerSec>0.20){out.message="B/C moved during XYZ temporal-calibration scan.";return out;}
    if(out.speedMadFraction>0.15){out.message="Selected XYZ segment is not sufficiently constant-speed.";return out;}
    if(out.crossTrackP80Fraction>0.15){out.message="Selected XYZ segment is not sufficiently straight.";return out;}

    std::vector<double> ax,ay,az; ax.reserve(accepted.size());ay.reserve(accepted.size());az.reserve(accepted.size());
    for(const auto& v:accepted){ax.push_back(v.x);ay.push_back(v.y);az.push_back(v.z);}
    out.velocityMmPerSec={median(ax),median(ay),median(az)};
    out.speedMmPerSec=std::sqrt(out.velocityMmPerSec.x*out.velocityMmPerSec.x+
                                out.velocityMmPerSec.y*out.velocityMmPerSec.y+
                                out.velocityMmPerSec.z*out.velocityMmPerSec.z);
    out.success=finiteVec(out.velocityMmPerSec)&&out.speedMmPerSec>=0.05;
    std::ostringstream os;os<<"OK: v=["<<out.velocityMmPerSec.x<<","<<out.velocityMmPerSec.y<<","<<out.velocityMmPerSec.z
        <<"] mm/s, speed="<<out.speedMmPerSec<<", intervals="<<out.usedIntervals<<"/"<<out.totalIntervals;
    out.message=os.str();
    return out;
}

TemporalTrajectoryCalibrationResult HardwareTemporalCalibration::solveTrajectoryPairs(
    const std::vector<TemporalTrajectoryObservation>& observations)
{
    TemporalTrajectoryCalibrationResult out;
    out.observationCount=observations.size();
    if(observations.empty()){out.message="No trajectory temporal-calibration observations.";return out;}
    const auto& first=observations.front();
    out.resolutionIndex=first.resolutionIndex;out.exposureTime10Us=first.exposureTime10Us;
    out.idleTime10Us=first.idleTime10Us;out.measuringFieldIndex=first.measuringFieldIndex;
    out.triggerFrequencyHz=first.triggerFrequencyHz;out.triggerPeriodMs=first.triggerPeriodMs;
    out.configurationConsistent=first.triggerFrequencyHz>0 && first.triggerPeriodMs>0.0;
    for(const auto& o:observations){
        if(o.directionSign==0||!finiteVec(o.velocityMmPerSec)||!finiteVec(o.sphereCenterMachineMm)||
           !finite(o.sphereRmsMm)||o.sphereRmsMm<0.0||!finite(o.fittedRadiusMm)||o.fittedRadiusMm<=0.0||
           o.resolutionIndex!=out.resolutionIndex||o.exposureTime10Us!=out.exposureTime10Us||
           o.idleTime10Us!=out.idleTime10Us||o.measuringFieldIndex!=out.measuringFieldIndex||
           o.triggerFrequencyHz!=out.triggerFrequencyHz||std::abs(o.triggerPeriodMs-out.triggerPeriodMs)>1e-9)
            out.configurationConsistent=false;
    }
    if(!out.configurationConsistent){out.message="Trajectory observations mix invalid/different sensor or trigger configurations.";return out;}

    std::vector<std::uint64_t> pairIds;
    for(const auto& o:observations)if(std::find(pairIds.begin(),pairIds.end(),o.pairId)==pairIds.end())pairIds.push_back(o.pairId);
    double denom=0.0,numer=0.0,radiusSum=0.0;std::size_t radiusN=0;
    std::vector<double> speedLevels;
    for(auto pid:pairIds){
        const TemporalTrajectoryObservation *pos=nullptr,*neg=nullptr;
        for(const auto& o:observations)if(o.pairId==pid){if(o.directionSign>0)pos=&o;else if(o.directionSign<0)neg=&o;}
        if(!pos||!neg)continue;
        TemporalTrajectoryPairFit pf;pf.pairId=pid;
        pf.deltaVelocityMmPerSec={pos->velocityMmPerSec.x-neg->velocityMmPerSec.x,
                                  pos->velocityMmPerSec.y-neg->velocityMmPerSec.y,
                                  pos->velocityMmPerSec.z-neg->velocityMmPerSec.z};
        pf.deltaCenterMm={pos->sphereCenterMachineMm.x-neg->sphereCenterMachineMm.x,
                          pos->sphereCenterMachineMm.y-neg->sphereCenterMachineMm.y,
                          pos->sphereCenterMachineMm.z-neg->sphereCenterMachineMm.z};
        const double dv2=pf.deltaVelocityMmPerSec.x*pf.deltaVelocityMmPerSec.x+
                         pf.deltaVelocityMmPerSec.y*pf.deltaVelocityMmPerSec.y+
                         pf.deltaVelocityMmPerSec.z*pf.deltaVelocityMmPerSec.z;
        if(dv2<=1e-8)continue;
        const double dot=pf.deltaVelocityMmPerSec.x*pf.deltaCenterMm.x+
                         pf.deltaVelocityMmPerSec.y*pf.deltaCenterMm.y+
                         pf.deltaVelocityMmPerSec.z*pf.deltaCenterMm.z;
        pf.pairDelaySec=-dot/dv2; // overwritten below when drift timestamps are available
        const double sp=std::sqrt(pos->velocityMmPerSec.x*pos->velocityMmPerSec.x+pos->velocityMmPerSec.y*pos->velocityMmPerSec.y+pos->velocityMmPerSec.z*pos->velocityMmPerSec.z);
        const double sn=std::sqrt(neg->velocityMmPerSec.x*neg->velocityMmPerSec.x+neg->velocityMmPerSec.y*neg->velocityMmPerSec.y+neg->velocityMmPerSec.z*neg->velocityMmPerSec.z);
        pf.meanSpeedMmPerSec=0.5*(sp+sn);
        pf.meanEffectiveRadiusMm=0.5*(pos->fittedRadiusMm+neg->fittedRadiusMm);
        pf.radiusDifferenceMm=pos->fittedRadiusMm-neg->fittedRadiusMm;
        if(pos->captureMidPcUs>0&&neg->captureMidPcUs>0)pf.pairTimeDeltaSec=double(pos->captureMidPcUs-neg->captureMidPcUs)*1e-6;
        pf.valid=true;out.pairs.push_back(pf);denom+=dv2;numer+=dot;
        radiusSum+=pos->fittedRadiusMm+neg->fittedRadiusMm;radiusN+=2;
        out.maxPairRadiusDifferenceMm=std::max(out.maxPairRadiusDifferenceMm,std::abs(pf.radiusDifferenceMm));
        bool seen=false;for(double q:speedLevels){const double scale=std::max({q,pf.meanSpeedMmPerSec,1e-9});if(std::abs(q-pf.meanSpeedMmPerSec)<=0.05*scale){seen=true;break;}}
        if(!seen)speedLevels.push_back(pf.meanSpeedMmPerSec);
    }
    out.pairCount=out.pairs.size();out.distinctSpeedLevels=speedLevels.size();
    if(out.pairs.empty()||denom<=1e-12){out.message="No complete forward/reverse trajectory pair with usable measured velocity span.";return out;}
    out.naivePairDelaySec=-numer/denom;
    out.meanEffectiveRadiusMm=radiusN?radiusSum/double(radiusN):0.0;

    // V27.6.50 primary model: c_i = c0 + g*(t_i-t0) - tau*v_i.
    // t_i is a host-clock scan midpoint used ONLY to absorb slow thermal/geometric
    // drift; it is not used as exposure/latch timing truth.
    std::vector<DriftInput> driftInput;driftInput.reserve(observations.size());
    for(const auto&o:observations)if(o.captureMidPcUs>0)
        driftInput.push_back({o.sphereCenterMachineMm,o.velocityMmPerSec,o.captureMidPcUs});
    const auto driftFit=fitCenterDriftAndDelay(driftInput);
    if(driftFit.success&&driftInput.size()==observations.size()){
        out.driftModelUsed=true;out.delaySec=driftFit.delaySec;out.delayStdSec=driftFit.delayStdSec;
        out.centerDriftMmPerSec=driftFit.driftMmPerSec;out.timeSpanSec=driftFit.timeSpanSec;
        out.residualRmsMm=driftFit.residualRmsMm;out.rSquared=driftFit.rSquared;
    }else{
        out.driftModelUsed=false;out.delaySec=out.naivePairDelaySec;
        double sse=0.0,sst=0.0;
        for(auto& p:out.pairs){
            p.modelResidualMm={p.deltaCenterMm.x+out.delaySec*p.deltaVelocityMmPerSec.x,
                               p.deltaCenterMm.y+out.delaySec*p.deltaVelocityMmPerSec.y,
                               p.deltaCenterMm.z+out.delaySec*p.deltaVelocityMmPerSec.z};
            p.residualNormMm=std::sqrt(p.modelResidualMm.x*p.modelResidualMm.x+p.modelResidualMm.y*p.modelResidualMm.y+p.modelResidualMm.z*p.modelResidualMm.z);
            sse+=p.residualNormMm*p.residualNormMm;
            sst+=p.deltaCenterMm.x*p.deltaCenterMm.x+p.deltaCenterMm.y*p.deltaCenterMm.y+p.deltaCenterMm.z*p.deltaCenterMm.z;
        }
        out.residualRmsMm=std::sqrt(sse/std::max<std::size_t>(1,out.pairs.size()));
        out.rSquared=sst>1e-18?std::max(-1.0,1.0-sse/sst):0.0;
        const std::ptrdiff_t dof=static_cast<std::ptrdiff_t>(3*out.pairs.size())-1;
        if(dof>0)out.delayStdSec=std::sqrt(std::max(0.0,(sse/static_cast<double>(dof))/denom));
    }

    // Pair diagnostics use drift-corrected center differences when possible.
    double pairSse=0.0,pairSst=0.0;
    for(auto& p:out.pairs){
        Vec3d dc=p.deltaCenterMm;
        if(out.driftModelUsed){
            dc.x-=out.centerDriftMmPerSec.x*p.pairTimeDeltaSec;
            dc.y-=out.centerDriftMmPerSec.y*p.pairTimeDeltaSec;
            dc.z-=out.centerDriftMmPerSec.z*p.pairTimeDeltaSec;
        }
        const double dv2=p.deltaVelocityMmPerSec.x*p.deltaVelocityMmPerSec.x+
                         p.deltaVelocityMmPerSec.y*p.deltaVelocityMmPerSec.y+
                         p.deltaVelocityMmPerSec.z*p.deltaVelocityMmPerSec.z;
        const double dot=p.deltaVelocityMmPerSec.x*dc.x+p.deltaVelocityMmPerSec.y*dc.y+p.deltaVelocityMmPerSec.z*dc.z;
        if(dv2>1e-12)p.pairDelaySec=-dot/dv2;
        p.modelResidualMm={dc.x+out.delaySec*p.deltaVelocityMmPerSec.x,
                           dc.y+out.delaySec*p.deltaVelocityMmPerSec.y,
                           dc.z+out.delaySec*p.deltaVelocityMmPerSec.z};
        p.residualNormMm=std::sqrt(p.modelResidualMm.x*p.modelResidualMm.x+p.modelResidualMm.y*p.modelResidualMm.y+p.modelResidualMm.z*p.modelResidualMm.z);
        pairSse+=p.residualNormMm*p.residualNormMm;pairSst+=dc.x*dc.x+dc.y*dc.y+dc.z*dc.z;
    }
    if(!out.driftModelUsed){
        out.rSquared=pairSst>1e-18?std::max(-1.0,1.0-pairSse/pairSst):0.0;
        out.residualRmsMm=std::sqrt(pairSse/std::max<std::size_t>(1,out.pairs.size()));
    }
    out.success=finite(out.delaySec)&&finite(out.residualRmsMm)&&finite(out.delayStdSec);
    std::ostringstream os;
    os<<"OK: "<<(out.driftModelUsed?"drift-aware":"pair-only")<<" trajectory delay="<<out.delaySec*1e6
      <<" us, naivePair="<<out.naivePairDelaySec*1e6<<" us, pairs="<<out.pairCount
      <<", speedLevels="<<out.distinctSpeedLevels<<", R2="<<out.rSquared
      <<", residualRMS="<<out.residualRmsMm<<" mm";
    if(out.driftModelUsed)os<<", drift(um/min)=["<<out.centerDriftMmPerSec.x*60000.0<<","<<out.centerDriftMmPerSec.y*60000.0<<","<<out.centerDriftMmPerSec.z*60000.0<<"]";
    os<<", meanEffectiveR="<<out.meanEffectiveRadiusMm<<" mm, maxPairDeltaR="<<out.maxPairRadiusDifferenceMm<<" mm.";
    out.message=os.str();return out;
}

bool HardwareTemporalCalibration::evaluateTrajectoryQuality(TemporalTrajectoryCalibrationResult& r,
                                                              const TemporalTrajectoryCriteria& c)
{
    r.qualityPassed=false;
    if(!r.success||!r.configurationConsistent){r.qualityMessage="Trajectory solver/configuration is not valid.";return false;}
    if(r.pairCount<c.minPairs){r.qualityMessage="Too few complete forward/reverse calibration pairs.";return false;}
    if(r.distinctSpeedLevels<c.minDistinctSpeedLevels){r.qualityMessage="Calibration needs at least two distinct measured speed levels.";return false;}
    if(r.rSquared<c.minRSquared){r.qualityMessage="Trajectory center-difference model R^2 is too low.";return false;}
    if(!finite(r.delayStdSec)||r.delayStdSec>c.maxDelayStdSec){r.qualityMessage="Trajectory delay uncertainty exceeds the configured gate.";return false;}
    if(!finite(r.maxPairRadiusDifferenceMm)||r.maxPairRadiusDifferenceMm>c.maxPairRadiusDifferenceMm){r.qualityMessage="Forward/reverse effective fitted radii disagree beyond the common-deformation gate.";return false;}
    const double periodSec=r.triggerFrequencyHz>0 ? 1.0/static_cast<double>(r.triggerFrequencyHz) : r.triggerPeriodMs/1000.0;
    const bool allowed = c.allowSignedDelay
        ? (std::isfinite(r.delaySec) && std::abs(r.delaySec) < periodSec*c.maxDelayFractionOfPeriod)
        : (std::isfinite(r.delaySec) && r.delaySec >= 0.0 && r.delaySec < periodSec*c.maxDelayFractionOfPeriod);
    if(!allowed){r.qualityMessage=c.allowSignedDelay
        ? "Delay is outside the signed (-period,+period) interpolation interval."
        : "Delay is outside the H1 [0, trigger period) look-ahead interval.";return false;}
    for(const auto& p:r.pairs)if(p.valid&&p.residualNormMm>c.maxPairResidualMm){r.qualityMessage="At least one forward/reverse trajectory pair has excessive 3-D model residual.";return false;}
    r.qualityPassed=true;
    r.qualityMessage=r.driftModelUsed
        ? "PASS: drift-aware fixed-delay model explains the trajectories; forward/reverse effective-radius consistency also passed."
        : "PASS: fixed-delay pair model explains the trajectories; forward/reverse effective-radius consistency also passed.";
    return true;
}

TemporalValidationResult HardwareTemporalCalibration::evaluateValidationPairs(
    const std::vector<TemporalValidationObservation>& observations,
    const TemporalValidationCriteria& criteria)
{
    TemporalValidationResult out;
    if(observations.empty()){out.message="No hold-out validation observations.";return out;}

    // Prefer the same slow-drift nuisance model used by calibration whenever all
    // validation scans carry capture midpoint timestamps. This prevents a long
    // hold-out sequence from making residual tau look worse merely because the
    // machine/sensor geometry drifts slowly with time.
    std::vector<DriftInput> rawIn,corrIn;rawIn.reserve(observations.size());corrIn.reserve(observations.size());
    for(const auto&o:observations){
        if(o.captureMidPcUs>0){
            rawIn.push_back({o.rawSphereCenterMachineMm,o.velocityMmPerSec,o.captureMidPcUs});
            corrIn.push_back({o.correctedSphereCenterMachineMm,o.velocityMmPerSec,o.captureMidPcUs});
        }
    }
    const auto rawDrift=fitCenterDriftAndDelay(rawIn);
    const auto corrDrift=fitCenterDriftAndDelay(corrIn);

    std::vector<std::uint64_t> ids;for(const auto&o:observations)if(std::find(ids.begin(),ids.end(),o.pairId)==ids.end())ids.push_back(o.pairId);
    double den=0.0,numRaw=0.0,numCorr=0.0,sseRaw=0.0,sseCorr=0.0;std::size_t n=0;
    struct Tmp{Vec3d dv,dr,dc;double dt=0.0;};std::vector<Tmp> pairs;
    for(auto pid:ids){
        const TemporalValidationObservation *pos=nullptr,*neg=nullptr;
        for(const auto&o:observations)if(o.pairId==pid){if(o.directionSign>0)pos=&o;else if(o.directionSign<0)neg=&o;}
        if(!pos||!neg)continue;
        Tmp t;
        t.dv={pos->velocityMmPerSec.x-neg->velocityMmPerSec.x,pos->velocityMmPerSec.y-neg->velocityMmPerSec.y,pos->velocityMmPerSec.z-neg->velocityMmPerSec.z};
        t.dr={pos->rawSphereCenterMachineMm.x-neg->rawSphereCenterMachineMm.x,pos->rawSphereCenterMachineMm.y-neg->rawSphereCenterMachineMm.y,pos->rawSphereCenterMachineMm.z-neg->rawSphereCenterMachineMm.z};
        t.dc={pos->correctedSphereCenterMachineMm.x-neg->correctedSphereCenterMachineMm.x,pos->correctedSphereCenterMachineMm.y-neg->correctedSphereCenterMachineMm.y,pos->correctedSphereCenterMachineMm.z-neg->correctedSphereCenterMachineMm.z};
        if(pos->captureMidPcUs>0&&neg->captureMidPcUs>0)t.dt=double(pos->captureMidPcUs-neg->captureMidPcUs)*1e-6;
        const double d2=t.dv.x*t.dv.x+t.dv.y*t.dv.y+t.dv.z*t.dv.z;if(d2<=1e-8)continue;
        den+=d2;numRaw+=t.dv.x*t.dr.x+t.dv.y*t.dr.y+t.dv.z*t.dr.z;numCorr+=t.dv.x*t.dc.x+t.dv.y*t.dc.y+t.dv.z*t.dc.z;pairs.push_back(t);++n;
    }
    if(n==0||den<=1e-12){out.message="No complete hold-out forward/reverse pair.";return out;}
    out.rawEquivalentDelaySec=(rawDrift.success&&rawIn.size()==observations.size())?rawDrift.delaySec:-numRaw/den;
    out.correctedResidualDelaySec=(corrDrift.success&&corrIn.size()==observations.size())?corrDrift.delaySec:-numCorr/den;
    out.pairCount=n;
    for(const auto&t0:pairs){
        Vec3d dr=t0.dr,dc=t0.dc;
        if(rawDrift.success&&rawIn.size()==observations.size()){
            dr.x-=rawDrift.driftMmPerSec.x*t0.dt;dr.y-=rawDrift.driftMmPerSec.y*t0.dt;dr.z-=rawDrift.driftMmPerSec.z*t0.dt;
        }
        if(corrDrift.success&&corrIn.size()==observations.size()){
            dc.x-=corrDrift.driftMmPerSec.x*t0.dt;dc.y-=corrDrift.driftMmPerSec.y*t0.dt;dc.z-=corrDrift.driftMmPerSec.z*t0.dt;
        }
        Vec3d rr{dr.x+out.rawEquivalentDelaySec*t0.dv.x,dr.y+out.rawEquivalentDelaySec*t0.dv.y,dr.z+out.rawEquivalentDelaySec*t0.dv.z};
        Vec3d rc{dc.x+out.correctedResidualDelaySec*t0.dv.x,dc.y+out.correctedResidualDelaySec*t0.dv.y,dc.z+out.correctedResidualDelaySec*t0.dv.z};
        sseRaw+=rr.x*rr.x+rr.y*rr.y+rr.z*rr.z;sseCorr+=rc.x*rc.x+rc.y*rc.y+rc.z*rc.z;
    }
    out.rawPairResidualRmsMm=std::sqrt(sseRaw/static_cast<double>(n));out.correctedPairResidualRmsMm=std::sqrt(sseCorr/static_cast<double>(n));
    const double base=std::max(std::abs(out.rawEquivalentDelaySec),1e-12);
    out.improvementFraction=1.0-std::abs(out.correctedResidualDelaySec)/base;
    out.success=true;
    out.passed=out.pairCount>=criteria.minPairs&&out.improvementFraction>=criteria.minImprovementFraction&&
               std::abs(out.correctedResidualDelaySec)<=criteria.maxResidualDelaySec;
    std::ostringstream os;os<<(out.passed?"PASS":"FAIL")<<": hold-out pairs="<<out.pairCount
        <<", rawEquivalentDelay="<<out.rawEquivalentDelaySec*1e6<<" us, residual="<<out.correctedResidualDelaySec*1e6
        <<" us, improvement="<<out.improvementFraction*100.0<<"%"
        <<((rawDrift.success&&corrDrift.success)?", drift-aware":"")<<".";out.message=os.str();
    return out;
}

} // namespace mls::calibration
