#include "twin/TwinM52Geometry.h"
#include "twin/TwinM51Chamfer.h"
#include "twin/TwinH1Geometry.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>
#include <vector>

namespace mls::twin {
namespace {
constexpr double kCx=0., kCy=-38.;
constexpr int kMaxPerAngle=1200;
bool finite(Vec3 v){return std::isfinite(v.x)&&std::isfinite(v.y)&&std::isfinite(v.z);}
double median(std::vector<double> v){
    if(v.empty())return std::numeric_limits<double>::quiet_NaN();
    const auto n=v.size();
    const auto k=n/2;
    std::nth_element(v.begin(),v.begin()+std::ptrdiff_t(k),v.end());
    const double hi=v[k];
    if(n%2)return hi;
    std::nth_element(v.begin(),v.begin()+std::ptrdiff_t(k-1),v.end());
    return .5*(hi+v[k-1]);
}
double mad(const std::vector<double>& x,double center){
    std::vector<double> a;a.reserve(x.size());
    for(double v:x)if(std::isfinite(v))a.push_back(std::abs(v-center));
    return median(std::move(a));
}
bool solve3(double a[3][3],double b[3],double out[3]){
    for(int j=0;j<3;++j){
        int pivot=j;
        for(int i=j+1;i<3;++i)if(std::abs(a[i][j])>std::abs(a[pivot][j]))pivot=i;
        if(std::abs(a[pivot][j])<1e-10)return false;
        for(int k=j;k<3;++k)std::swap(a[pivot][k],a[j][k]);
        std::swap(b[pivot],b[j]);
        const double v=a[j][j];
        for(int k=j;k<3;++k)a[j][k]/=v;
        b[j]/=v;
        for(int i=0;i<3;++i){
            if(i==j)continue;
            const double f=a[i][j];
            for(int k=j;k<3;++k)a[i][k]-=f*a[j][k];
            b[i]-=f*b[j];
        }
    }
    for(int i=0;i<3;++i)out[i]=b[i];
    return true;
}
// Each row is (u,v,1,y) and has base weight. IRLS with Huber losses.
struct Row { double u,v,y,base=1.; };
bool robustFit(const std::vector<Row>& rows,std::array<double,3>& coefficients){
    if(rows.size()<3)return false;
    double params[3]={coefficients[0],coefficients[1],coefficients[2]};
    for(int iteration=0;iteration<10;++iteration){
        std::vector<double> residuals;residuals.reserve(rows.size());
        for(const auto& row:rows)residuals.push_back(row.y-row.u*params[0]-row.v*params[1]-params[2]);
        const double offset=median(residuals);
        const double scale=std::max(0.005,1.4826*mad(residuals,offset));
        double a[3][3]{},b[3]{};
        for(std::size_t j=0;j<rows.size();++j){
            const auto& r=rows[j];
            const double absres=std::abs(residuals[j]-offset);
            const double w=r.base*(absres<=2.*scale?1.:(2.*scale/absres));
            const double f[3]={r.u,r.v,1.};
            for(int h=0;h<3;++h){
                b[h]+=w*f[h]*r.y;
                for(int k=0;k<3;++k)a[h][k]+=w*f[h]*f[k];
            }
        }
        double next[3]{};
        if(!solve3(a,b,next))return false;
        const double shift=std::abs(params[0]-next[0])+std::abs(params[1]-next[1])+std::abs(params[2]-next[2]);
        std::copy(next,next+3,params);
        if(shift<1e-10)break;
    }
    for(int i=0;i<3;++i)coefficients[std::size_t(i)]=params[i];
    return true;
}
int sector(Vec3 p){
    double phi=std::atan2(p.y-kCy,p.x-kCx);
    if(phi<0)phi+=2*kPi;
    return std::clamp(int(std::floor(phi*M52AngularBins/(2*kPi))),0,M52AngularBins-1);
}
bool containsXY(const Triangle& t,double x,double y){
    const double d=(t.b.y-t.c.y)*(t.a.x-t.c.x)+(t.c.x-t.b.x)*(t.a.y-t.c.y);
    if(std::abs(d)<1e-12)return false;
    const double w1=((t.b.y-t.c.y)*(x-t.c.x)+(t.c.x-t.b.x)*(y-t.c.y))/d;
    const double w2=((t.c.y-t.a.y)*(x-t.c.x)+(t.a.x-t.c.x)*(y-t.c.y))/d;
    return w1>=-1e-9 && w2>=-1e-9 && w1+w2<=1.+1e-9;
}
struct ConeObservation {int bin=0;double angle=0,raw=0,corrected=0;};
}
TwinM52GeometryAnalyzer::TwinM52GeometryAnalyzer(const TwinScene& scene,M52Options options){
    report_.options=options;
    const auto& o=options;
    if(!std::isfinite(o.topGridPitchMm)||o.topGridPitchMm<.1||o.topGridPitchMm>1.||
       !std::isfinite(o.topGuardMm)||o.topGuardMm<.4||o.topGuardMm>3.||
       !std::isfinite(o.topZGateMm)||o.topZGateMm<.3||o.topZGateMm>2.||
       !std::isfinite(o.coneGateMm)||o.coneGateMm<.2||o.coneGateMm>1.2||
       o.minTopCells<10||o.minConePoints<10||o.minFitAngularBins<8){
        report_.error="M5.2: invalid diagnostic options";return;
    }
    TwinM51ChamferAnalyzer geometry(scene);
    if(!geometry.ready()){
        report_.error="M5.2: "+geometry.error();return;
    }
    const auto g=inferH1Geometry(scene);
    report_.nominalOuterRadiusMm=g.outerRadius;
    report_.nominalChamferDepthMm=g.chamferDepth;
    const auto& m=scene.mesh();
    topZ_=m.bounds.max.z;bottomZ_=m.bounds.min.z;
    report_.nominalTopZmm=topZ_;
    gridMinX_=m.bounds.min.x;gridMinY_=m.bounds.min.y;
    gridW_=int(std::ceil((m.bounds.max.x-gridMinX_)/o.topGridPitchMm));
    gridH_=int(std::ceil((m.bounds.max.y-gridMinY_)/o.topGridPitchMm));
    if(gridW_<5||gridH_<5||gridW_>2000||gridH_>2000){
        report_.error="M5.2: CAD top grid resolution out of bounds";return;
    }
    const auto w=std::size_t(gridW_);
    const auto h=std::size_t(gridH_);
    std::vector<std::uint8_t> raw(w*h,0);
    // Tessellated B-Rep top triangles, no bounding-box shortcut across holes.
    for(const auto& t:m.triangles){
        if(std::abs(t.a.z-topZ_)>.02||std::abs(t.b.z-topZ_)>.02||std::abs(t.c.z-topZ_)>.02)continue;
        const double minX=std::min({t.a.x,t.b.x,t.c.x}),maxX=std::max({t.a.x,t.b.x,t.c.x});
        const double minY=std::min({t.a.y,t.b.y,t.c.y}),maxY=std::max({t.a.y,t.b.y,t.c.y});
        const int i0=std::clamp(int(std::floor((minX-gridMinX_)/o.topGridPitchMm)),0,gridW_-1);
        const int i1=std::clamp(int(std::floor((maxX-gridMinX_)/o.topGridPitchMm)),0,gridW_-1);
        const int j0=std::clamp(int(std::floor((minY-gridMinY_)/o.topGridPitchMm)),0,gridH_-1);
        const int j1=std::clamp(int(std::floor((maxY-gridMinY_)/o.topGridPitchMm)),0,gridH_-1);
        for(int j=j0;j<=j1;++j){
            const double y=gridMinY_+(j+.5)*o.topGridPitchMm;
            for(int i=i0;i<=i1;++i){
                const double x=gridMinX_+(i+.5)*o.topGridPitchMm;
                if(containsXY(t,x,y))raw[std::size_t(j)*w+std::size_t(i)]=1;
            }
        }
    }
    // Erode 2D top mask by guard distance to exclude slots, holes and edges.
    const int radius=int(std::ceil(o.topGuardMm/o.topGridPitchMm));
    std::vector<std::uint32_t> integral((w+1)*(h+1),0);
    for(std::size_t j=0;j<h;++j)for(std::size_t i=0;i<w;++i)
        integral[(j+1)*(w+1)+i+1]=raw[j*w+i]+integral[j*(w+1)+i+1]+integral[(j+1)*(w+1)+i]-integral[j*(w+1)+i];
    safeTop_.assign(w*h,0);
    std::size_t valid=0;
    for(int j=radius;j<gridH_-radius;++j)for(int i=radius;i<gridW_-radius;++i){
        const std::size_t x0=std::size_t(i-radius),x1=std::size_t(i+radius+1);
        const std::size_t y0=std::size_t(j-radius),y1=std::size_t(j+radius+1);
        const std::uint32_t sum=integral[y1*(w+1)+x1]-integral[y0*(w+1)+x1]
                                -integral[y1*(w+1)+x0]+integral[y0*(w+1)+x0];
        if(sum==std::uint32_t((2*radius+1)*(2*radius+1))){
            safeTop_[std::size_t(j)*w+std::size_t(i)]=1;++valid;
        }
    }
    if(valid<std::size_t(o.minTopCells)){
        report_.error="M5.2: insufficient stable top-plane CAD grid cells";return;
    }
    real_.topUsed.assign(w*h,0);virtual_.topUsed.assign(w*h,0);
    report_.limitation="M5.2 is an independent diagnostic. Only measured returns are used; missing point causes remain UNKNOWN. "
        "A single fixed optical view cannot separate pose, scale, edge mixing and manufacturing deviation; "
        "candidate shifts are NEVER written to CAD, calibration or machine commands. "
        "Relative cone offsets are not certified dimensional errors.";
}
void TwinM52GeometryAnalyzer::add(Vec3 p,Dataset& data){
    ++data.seen;
    if(!ready()||!finite(p))return;
    const auto& o=report_.options;
    if(std::abs(p.z-topZ_)<=o.topZGateMm){
        const int i=int(std::floor((p.x-gridMinX_)/o.topGridPitchMm));
        const int j=int(std::floor((p.y-gridMinY_)/o.topGridPitchMm));
        if(i>=0&&i<gridW_&&j>=0&&j<gridH_){
            const auto id=std::size_t(j)*std::size_t(gridW_)+std::size_t(i);
            if(safeTop_[id]&&!data.topUsed[id]){
                data.topUsed[id]=1;data.top.push_back(p);
            }
        }
    }
    const double r=std::hypot(p.x-kCx,p.y-kCy),d=topZ_-p.z;
    if(r>=report_.nominalOuterRadiusMm-report_.nominalChamferDepthMm-.4 &&
       r<=report_.nominalOuterRadiusMm+.4 &&d>=-o.coneGateMm&&
       d<=report_.nominalChamferDepthMm+o.coneGateMm){
        const int a=sector(p);
        if(data.coneCounts[std::size_t(a)]<kMaxPerAngle){
            ++data.coneCounts[std::size_t(a)];data.cone.push_back(p);
        }
    }
}
void TwinM52GeometryAnalyzer::addRealCad(Vec3 p){add(p,real_);}
void TwinM52GeometryAnalyzer::addVirtualCad(Vec3 p){report_.hasVirtual=true;add(p,virtual_);}
M52Fit TwinM52GeometryAnalyzer::fit(const Dataset& data) const {
    M52Fit out;out.pointsSeen=data.seen;
    out.topCellsUsed=std::uint32_t(data.top.size());
    if(data.top.size()<report_.options.minTopCells){out.error="insufficient measured CAD-safe top-plane returns";return out;}
    const double y0=-7.5; // center the numerical solve, convert intercept back below
    std::vector<Row> planeRows;planeRows.reserve(data.top.size());
    for(const auto& p:data.top)planeRows.push_back({p.x,p.y-y0,p.z-topZ_,1.});
    std::vector<double> zObserved;
    zObserved.reserve(planeRows.size());
    for(const auto& row:planeRows)zObserved.push_back(row.y);
    std::array<double,3> plane{{0.,0.,median(std::move(zObserved))}};
    if(!robustFit(planeRows,plane)){out.error="degenerate top-plane fit";return out;}
    out.topSlopeX=plane[0];out.topSlopeY=plane[1];out.topInterceptMm=plane[2]-plane[1]*y0;
    out.topOffsetAtHoleMm=out.topSlopeX*kCx+out.topSlopeY*kCy+out.topInterceptMm;
    std::vector<double> planeRes;planeRes.reserve(data.top.size());
    for(const auto& p:data.top)planeRes.push_back(p.z-topZ_-(out.topSlopeX*p.x+out.topSlopeY*p.y+out.topInterceptMm));
    const double pm=median(planeRes);out.topPlaneMadMm=mad(planeRes,pm);
    // Geometry interior selection after independent top-plane stabilization.
    std::array<std::vector<double>,M52AngularBins> binRaw{},binAdjusted{};
    for(const auto& p:data.cone){
        const double radius=std::hypot(p.x-kCx,p.y-kCy);
        const double topCorrection=out.topSlopeX*p.x+out.topSlopeY*p.y+out.topInterceptMm;
        const double rawD=topZ_-p.z;
        const double d=rawD+topCorrection; // subtract plane offset from measured z
        const double u=(report_.nominalOuterRadiusMm-radius+d)/(2.*report_.nominalChamferDepthMm); // nearest interior parameter, 0<=u<=1 for 0.5x45
        const double dist=std::abs(radius+d-report_.nominalOuterRadiusMm)/std::sqrt(2.);
        if(d<-.12||d>report_.nominalChamferDepthMm+.15||u<.10||u>.90||dist>.48)continue;
        const int a=sector(p);
        binRaw[std::size_t(a)].push_back(radius+rawD-report_.nominalOuterRadiusMm);
        binAdjusted[std::size_t(a)].push_back(radius+d-report_.nominalOuterRadiusMm);
        ++out.conePointsUsed;
    }
    if(out.conePointsUsed<report_.options.minConePoints){out.error="insufficient nominal upper-chamfer samples";return out;}
    std::vector<Row> train,validation,all;
    std::vector<double> rawMedians,topMedians;
    for(int a=0;a<M52AngularBins;++a){
        auto& rec=out.angular[std::size_t(a)];rec.angleStartDeg=a*5;
        rec.hits=std::uint32_t(binAdjusted[std::size_t(a)].size());
        if(rec.hits==0)continue;
        rec.medianRawMm=median(binRaw[std::size_t(a)]);
        rec.medianTopCorrectedMm=median(binAdjusted[std::size_t(a)]);
        // Use angular MEDIANS, not raw points, so dense sectors do not dominate.
        if(rec.hits<3)continue;
        ++out.angularBinsUsed;
        rawMedians.push_back(rec.medianRawMm);
        topMedians.push_back(rec.medianTopCorrectedMm);
        const double angle=(a+.5)*5*kPi/180.;
        const Row r{std::cos(angle),std::sin(angle),rec.medianTopCorrectedMm,1.};
        all.push_back(r);
        rec.training=(a%2==0);
        if(rec.training)train.push_back(r);
        else validation.push_back(r);
    }
    // Number of populated bins alone is not enough: 24 adjacent bins form
    // only one 120-degree arc and can produce a falsely confident circle fit.
    int widestEmpty=0,currentEmpty=0;
    int occupiedQuadrants=0;
    for(int q=0;q<4;++q){
        bool found=false;
        for(int a=q*18;a<(q+1)*18;++a)
            if(out.angular[std::size_t(a)].hits>=3){found=true;break;}
        if(found)++occupiedQuadrants;
    }
    for(int j=0;j<2*M52AngularBins;++j){
        if(out.angular[std::size_t(j%M52AngularBins)].hits<3)
            currentEmpty=std::min(currentEmpty+1,M52AngularBins);
        else currentEmpty=0;
        widestEmpty=std::max(widestEmpty,currentEmpty);
    }
    out.maxUnobservedArcDeg=std::uint32_t(widestEmpty*5);
    out.enoughAngularSupport=(out.angularBinsUsed>=report_.options.minFitAngularBins &&
                               occupiedQuadrants>=3 && out.maxUnobservedArcDeg<=140);
    if(!out.enoughAngularSupport||train.size()<12){
        out.error="insufficient angular coverage: require >=3 quadrants and maximum empty arc <=140 deg";
        return out;
    }
    out.trainBins=std::uint32_t(train.size());out.validationBins=std::uint32_t(validation.size());
    out.coneRawMedianMm=median(rawMedians);
    out.coneAfterTopMedianMm=median(topMedians);
    std::array<double,3> params{{0.,0.,out.coneAfterTopMedianMm}};
    if(!robustFit(train,params)){out.error="degenerate chamfer circle/offset fit";return out;}
    auto residualMAD=[&](const std::vector<Row>& group){
        std::vector<double> r;r.reserve(group.size());
        for(const auto& v:group)r.push_back(v.y-v.u*params[0]-v.v*params[1]-params[2]);
        return mad(r,median(r));
    };
    out.coneFitTrainMadMm=residualMAD(train);
    out.coneFitValidationMadMm=validation.empty()?std::numeric_limits<double>::quiet_NaN():residualMAD(validation);
    out.holdoutEvaluated=!validation.empty();
    // Final fit uses both groups. Holdout performance above remains independent.
    if(!robustFit(all,params)){out.error="degenerate final fit";return out;}
    out.candidateCenterShiftXmm=params[0];out.candidateCenterShiftYmm=params[1];
    out.coneInterceptMm=params[2];
    std::vector<double> afterXY,afterFit;
    for(int a=0;a<M52AngularBins;++a){
        auto& rec=out.angular[std::size_t(a)];
        if(rec.hits<3)continue;
        const double theta=(a+.5)*5*kPi/180.;
        const double harmonic=std::cos(theta)*params[0]+std::sin(theta)*params[1];
        rec.medianAfterXYMm=rec.medianTopCorrectedMm-harmonic;
        rec.medianAfterFullFitMm=rec.medianAfterXYMm-params[2];
        afterXY.push_back(rec.medianAfterXYMm);
        afterFit.push_back(rec.medianAfterFullFitMm);
    }
    out.coneAfterXYMedianMm=median(afterXY);
    out.coneAfterXYMadMm=mad(afterXY,out.coneAfterXYMedianMm);
    // No automatic pose updates, even if fit residual is small.
    out.canApplyAutomatically=false;
    out.ok=true;
    return out;
}
M52Report TwinM52GeometryAnalyzer::finish() && {
    if(!ready())return std::move(report_);
    report_.real=fit(real_);
    if(!report_.real.ok)report_.error="M5.2 real data: "+report_.real.error;
    if(report_.hasVirtual)report_.virtualFit=fit(virtual_);
    return std::move(report_);
}
} // namespace mls::twin
