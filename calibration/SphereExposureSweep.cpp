#include "calibration/SphereExposureSweep.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <numeric>

namespace mls::calibration {
namespace {
constexpr double pi = 3.14159265358979323846;

double median(std::vector<double> v) {
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    const std::size_t mid = v.size() / 2;
    return (v.size() % 2) ? v[mid] : (v[mid - 1] + v[mid]) / 2.0;
}

// Gaussian elimination with partial pivoting. All quantities are in mm.
bool solve3(double a[3][4], double out[3]) {
    for (int c = 0; c < 3; ++c) {
        int pivot = c;
        for (int i = c + 1; i < 3; ++i)
            if (std::abs(a[i][c]) > std::abs(a[pivot][c])) pivot = i;
        if (std::abs(a[pivot][c]) < 1e-11) return false;
        if (pivot != c) for (int j = 0; j < 4; ++j) std::swap(a[pivot][j], a[c][j]);
        const double d = a[c][c];
        for (int j = c; j < 4; ++j) a[c][j] /= d;
        for (int i = 0; i < 3; ++i) if (i != c) {
            const double f = a[i][c];
            for (int j = c; j < 4; ++j) a[i][j] -= f * a[c][j];
        }
    }
    for (int i = 0; i < 3; ++i) out[i] = a[i][3];
    return true;
}

bool leastSquaresCircle(const std::vector<SphereExposurePoint>& pts,
                        const std::vector<std::size_t>& selection,
                        double& cx, double& cz, double& r) {
    if (selection.size() < 12) return false;
    double mx = 0.0, mz = 0.0;
    for (const auto idx : selection) { mx += pts[idx].x; mz += pts[idx].z; }
    mx /= selection.size(); mz /= selection.size();
    double xx=0, zz=0, xz=0, xq=0, zq=0, sq=0;
    for (const auto idx : selection) {
        const double x=pts[idx].x-mx, z=pts[idx].z-mz, q=x*x+z*z;
        xx+=x*x; zz+=z*z; xz+=x*z; xq+=x*q; zq+=z*q; sq+=q;
    }
    // -2a*x -2b*z + (r²-a²-b²) = -(x²+z²)
    double m[3][4]={{xx,xz,0.0,0.5*xq}, {xz,zz,0.0,0.5*zq}, {0,0,static_cast<double>(selection.size()),sq}};
    double u[3];
    if (!solve3(m,u)) return false;
    cx=mx+u[0]; cz=mz+u[1];
    const double r2=u[2] + u[0]*u[0]+u[1]*u[1];
    r=(r2>0)?std::sqrt(r2):0.0;
    return std::isfinite(cx)&&std::isfinite(cz)&&std::isfinite(r)&&r>0.0;
}

bool parabolaPeak(const std::vector<std::pair<double,double>>& samples, double& peakR) {
    if (samples.size() < 8) return false;
    auto top=std::max_element(samples.begin(), samples.end(),
        [](const auto& a,const auto& b){return a.second<b.second;});
    const double maxR=top->second;
    double sx=0,sx2=0,sx3=0,sx4=0,sy=0,sxy=0,sx2y=0;
    int n=0;
    // Narrow central window controls arc-fit radius curvature while rejecting edge arcs.
    for (const auto& v:samples) {
        if (v.second<maxR*0.975) continue;
        const double x=v.first-top->first, y=v.second*v.second;
        sx+=x; sx2+=x*x; sx3+=x*x*x; sx4+=x*x*x*x;
        sy+=y; sxy+=x*y; sx2y+=x*x*y; ++n;
    }
    if (n<8) return false;
    double m[3][4]={{static_cast<double>(n),sx,sx2,sy},
                    {sx,sx2,sx3,sxy},{sx2,sx3,sx4,sx2y}};
    double p[3]; if(!solve3(m,p)||p[2]>=-1e-13)return false;
    const double x0=-p[1]/(2*p[2]);
    // Never extrapolate more than two sensor sampling intervals beyond data peak.
    double xmin=std::numeric_limits<double>::max(),xmax=-xmin;
    for(const auto& v:samples)if(v.second>=maxR*0.975){
        xmin=std::min(xmin,v.first-top->first);
        xmax=std::max(xmax,v.first-top->first);
    }
    if(x0<xmin||x0>xmax)return false;
    const double pr2=p[0]+p[1]*x0+p[2]*x0*x0;
    if(pr2<=0||!std::isfinite(pr2))return false;
    peakR=std::sqrt(pr2);
    // A gross bump is not a believable sub-frame interpolation.
    return peakR>=maxR*0.995 && peakR<=maxR+0.10;
}
}

SphereExposureArc SphereExposureSweep::fitArc(const std::vector<SphereExposurePoint>& points,
      double expectedCx, double expectedCz, double R) {
    SphereExposureArc out;
    if (!std::isfinite(expectedCx)||!std::isfinite(expectedCz)||!(R>1)) return out;
    // The model-predicted sensor X/Z center is the ROI anchor, not a fit result.
    // Choose the longest contiguous stripe to avoid background surfaces.
    std::vector<SphereExposurePoint> best, run;
    auto flush=[&]{ if(run.size()>best.size())best.swap(run); run.clear(); };
    int previous=-10;
    for(const auto& p:points) {
        const double d=std::hypot(p.x-expectedCx,p.z-expectedCz);
        if(!std::isfinite(p.x)||!std::isfinite(p.z)||p.z==0.0
           ||d<1.4||d>R+0.8 || p.index>previous+3) {
            flush();
            if(!std::isfinite(d)||d<1.4||d>R+0.8||p.z==0.0){previous=p.index;continue;}
        }
        if(!run.empty() && std::hypot(p.x-run.back().x,p.z-run.back().z)>2.0)flush();
        run.push_back(p);previous=p.index;
    }
    flush();
    if(best.size()<40)return out;
    std::vector<std::size_t> selected;
    selected.reserve(best.size());
    for(std::size_t i=0;i<best.size();++i)selected.push_back(i);
    double cx=0,cz=0,r=0;
    for(int iter=0;iter<4;++iter) {
        if(!leastSquaresCircle(best,selected,cx,cz,r))return out;
        std::vector<double> absErr;absErr.reserve(selected.size());
        for(auto i:selected)absErr.push_back(std::abs(std::hypot(best[i].x-cx,best[i].z-cz)-r));
        const double threshold=std::max(0.012,2.7*median(absErr));
        std::vector<std::size_t> next;
        for(auto i:selected)if(std::abs(std::hypot(best[i].x-cx,best[i].z-cz)-r)<=threshold)next.push_back(i);
        if(next.size()<40||next.size()<best.size()*0.65)return out;
        if(next.size()==selected.size())break;
        selected.swap(next);
    }
    if(!leastSquaresCircle(best,selected,cx,cz,r)||r<R*0.18||r>R+0.45
       ||std::hypot(cx-expectedCx,cz-expectedCz)>1.8)return out;
    double err2=0.0;
    std::vector<double> angles; angles.reserve(selected.size());
    for(const auto i:selected) {
        const double e=std::hypot(best[i].x-cx,best[i].z-cz)-r;
        err2+=e*e;
        angles.push_back(std::atan2(best[i].z-cz,best[i].x-cx));
    }
    const double rms=std::sqrt(err2/selected.size());
    std::sort(angles.begin(),angles.end());
    double largestGap=0.0;
    for(std::size_t i=1;i<angles.size();++i)largestGap=std::max(largestGap,angles[i]-angles[i-1]);
    largestGap=std::max(largestGap,angles.front()+2*pi-angles.back());
    const double span=(2*pi-largestGap)*180.0/pi;
    if(rms>0.080||span<40.0)return out;
    out.valid=true;out.centerX=cx;out.centerZ=cz;out.radius=r;
    out.rms=rms;out.arcSpanDeg=span;out.inliers=selected.size();
    return out;
}

SphereExposurePass SphereExposureSweep::summarize(const std::vector<SphereExposureFrame>& frames,
      int exposure10us,int rateHz,int repeat,int direction,double speedMmPerSec,double knownDiameterMm) {
    SphereExposurePass out;
    out.exposure10us=exposure10us;out.rateHz=rateHz;out.repeat=repeat;
    out.direction=direction;out.speedMmPerSec=speedMmPerSec;
    out.nominalSpacingMm=speedMmPerSec/std::max(1,rateHz);
    out.knownDiameterMm=knownDiameterMm;out.frames=frames.size();
    std::vector<double> radii,rms,spans,maximums,widths,plateaus;
    std::vector<std::pair<double,double>> samples;
    for(std::size_t i=0;i<frames.size();++i) {
        const auto& a=frames[i].arc;
        if(!a.valid)continue;
        ++out.acceptedFrames;
        if(frames[i].signalAvailable){
            ++out.signalFrames;
            maximums.push_back(frames[i].maximumMean);
            widths.push_back(frames[i].widthMedian);
            plateaus.push_back(frames[i].maximumPlateauFraction);
        }
        radii.push_back(a.radius);rms.push_back(a.rms);spans.push_back(a.arcSpanDeg);
        samples.emplace_back(static_cast<double>(i),a.radius);
    }
    out.validFraction=frames.empty()?0.0:double(out.acceptedFrames)/frames.size();
    out.medianArcRmsMm=median(rms);out.medianArcSpanDeg=median(spans);
    out.medianMaximum=median(maximums);
    out.medianWidth=median(widths);
    out.medianMaximumPlateauFraction=median(plateaus);
    if(!radii.empty()) {
        const double maxR=*std::max_element(radii.begin(),radii.end());
        out.observedMaxDiameterMm=2.0*maxR;
        double fittedR=maxR;
        out.peakInterpolated=parabolaPeak(samples,fittedR);
        out.fittedPeakDiameterMm=2.0*fittedR;
    }
    out.complete=frames.size()>=20 && out.acceptedFrames>=12 && out.validFraction>=0.18
        && out.nominalSpacingMm<=0.080+1e-9;
    out.note=out.complete ? "eligible for repeatability comparison" : "insufficient arc coverage or coarse spatial sampling";
    return out;
}

std::vector<SphereExposureRank> SphereExposureSweep::rank(
    const std::vector<SphereExposurePass>& passes,double knownDiameterMm) {
    std::map<int,std::vector<const SphereExposurePass*>> byExposure;
    for(const auto& pass:passes)byExposure[pass.exposure10us].push_back(&pass);
    std::vector<SphereExposureRank> ranks;
    for(const auto& kv:byExposure){
        SphereExposureRank r;r.exposure10us=kv.first;r.passes=kv.second.size();
        std::vector<double> diameter,rms,coverage,signalCoverage,plateaus,widths;
        bool allComplete=true;
        for(const auto* p:kv.second){
            allComplete &= p->complete;
            if(p->complete){diameter.push_back(p->fittedPeakDiameterMm);
                rms.push_back(p->medianArcRmsMm);coverage.push_back(p->validFraction);
                signalCoverage.push_back(p->frames>0 ? double(p->signalFrames)/p->frames : 0.0);
                if(p->signalFrames>0) {plateaus.push_back(p->medianMaximumPlateauFraction);
                    widths.push_back(p->medianWidth);}
            }
        }
        r.eligible=allComplete && diameter.size()>=2;
        if(!diameter.empty()){
            r.medianDiameterMm=median(diameter);
            r.diameterErrorMm=std::abs(r.medianDiameterMm-knownDiameterMm);
            const auto range=std::minmax_element(diameter.begin(),diameter.end());
            r.diameterRepeatabilityMm=*range.second-*range.first;
            r.medianArcRmsMm=median(rms);r.coverageFraction=median(coverage);
            r.signalCoverageFraction=median(signalCoverage);
            r.medianMaximumPlateauFraction=median(plateaus);
            r.medianWidth=median(widths);
            // Lower better. In mm: penalize systematic diameter deviation,
            // repeatability, and arc noise; coverage is a weak supporting term.
            r.score=3.0*r.diameterErrorMm+2.0*r.diameterRepeatabilityMm
                  +1.5*r.medianArcRmsMm+0.020*(1.0-r.coverageFraction);
            // Weak optical-signal penalty, only if auxiliary LLT values were observed.
            // The plateau of a frame is a diagnostic, NOT absolute saturation.
            if(r.signalCoverageFraction>=0.5 && !plateaus.empty())
                r.score+=0.008*r.medianMaximumPlateauFraction;
        }
        r.note=r.eligible ? "diagnostic ranking only, require optical/geometry validation" :
            "not eligible: need >=2 successful complete passes";
        ranks.push_back(r);
    }
    std::stable_sort(ranks.begin(),ranks.end(),[](const auto& a,const auto& b){
        if(a.eligible!=b.eligible)return a.eligible;
        return a.score<b.score;
    });
    return ranks;
}
} // namespace mls::calibration
