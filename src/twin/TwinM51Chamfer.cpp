#include "twin/TwinM51Chamfer.h"
#include "twin/TwinH1Geometry.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <set>

namespace mls::twin {
namespace {
constexpr double PI=3.1415926535897932384626433832795;
constexpr double centerX=0.,centerY=-38.;
constexpr double holeDepth=10.;
constexpr double rootTwo=1.4142135623730950488;
bool finite(Vec3 v){return std::isfinite(v.x)&&std::isfinite(v.y)&&std::isfinite(v.z);}
double triangleArea(const Triangle& t){return .5*length(cross(t.b-t.a,t.c-t.a));}
int angularBin(double x,double y){
    const double a=std::atan2(y-centerY,x-centerX);
    const double normalized=a<0?a+2.*PI:a;
    return std::min(M51AngularBins-1,static_cast<int>(std::floor(normalized*M51AngularBins/(2.*PI))));
}
int layerCount(M51Surface s){return s==M51Surface::Cylinder?M51CylinderLayers:M51ChamferLayers;}
std::size_t offset(M51Surface s){
    if(s==M51Surface::TopChamfer)return 0;
    if(s==M51Surface::Cylinder)return M51AngularBins*M51ChamferLayers;
    return M51AngularBins*(M51ChamferLayers+M51CylinderLayers);
}
std::size_t index(M51Surface s,int angle,int layer){return offset(s)+std::size_t(layer)*M51AngularBins+std::size_t(angle);}
double surfaceArea(M51Surface s,int layer,double outerRadius,double innerRadius,double chamferDepth){
    if(s==M51Surface::Cylinder) return 2.*PI*innerRadius*(holeDepth-2.*chamferDepth)/(M51AngularBins*M51CylinderLayers);
    const double du=1./M51ChamferLayers;
    const double r0=outerRadius-chamferDepth*layer*du;
    const double r1=outerRadius-chamferDepth*(layer+1)*du;
    return PI*(r0+r1)*(rootTwo*chamferDepth*du)/M51AngularBins;
}
double median(std::vector<double> x){
    if(x.empty())return 0.;
    const std::size_t k=x.size()/2;
    std::nth_element(x.begin(),x.begin()+std::ptrdiff_t(k),x.end());
    const double upper=x[k];
    if(x.size()%2)return upper;
    std::nth_element(x.begin(),x.begin()+std::ptrdiff_t(k-1),x.begin()+std::ptrdiff_t(k));
    return (upper+x[k-1])*.5;
}
std::uint32_t quantileUpperQuartile(const M51Report& report,M51Surface s,int layer,bool virtualData){
    std::array<std::uint32_t,M51AngularBins> hits{};
    for(int a=0;a<M51AngularBins;++a){
        const auto& b=report.bins[index(s,a,layer)];
        hits[std::size_t(a)]=virtualData?b.virtualHits[report.options.primaryThreshold]:b.realHits[report.options.primaryThreshold];
    }
    std::sort(hits.begin(),hits.end());
    return hits[static_cast<std::size_t>(M51AngularBins*3/4)];
}
}
TwinM51ChamferAnalyzer::TwinM51ChamferAnalyzer(const TwinScene& scene,M51Options opts){
    report_.options=opts;
    auto& r=report_;
    if(!scene.hasWorkpiece()){r.error="M5.1 requires a loaded STEP/OBJ CAD workpiece";return;}
    const auto& bounds=scene.mesh().bounds;
    const auto span=bounds.size();
    if(!(std::abs(span.x-30.)<.75 && std::abs(span.y-95.)<.75 && std::abs(span.z-10.)<.5)){
        r.error="M5.1 center-hole analysis is currently specific to the validated H1 30x95x10 CAD geometry";return;
    }
    for(const double d:opts.thresholdsMm)if(!std::isfinite(d)||d<=0||d>2.){
        r.error="Invalid association thresholds";return;
    }
    if(opts.primaryThreshold<0||opts.primaryThreshold>=M51ThresholdCount||
       !std::isfinite(opts.sparseRatio)||opts.sparseRatio<=0||opts.sparseRatio>=1||
       opts.regionMarginMm<0||opts.regionMarginMm>.15){
        r.error="Invalid M5.1 option";return;
    }
    const auto geometry=inferH1Geometry(scene);
    if(!geometry.valid){r.error="M5.1: imported CAD centre-hole profile is unrecognized";return;}
    outerRadius_=geometry.outerRadius;innerRadius_=geometry.innerRadius;
    chamferDepth_=geometry.chamferDepth;
    r.nominalOuterRadiusMm=outerRadius_;r.nominalInnerRadiusMm=innerRadius_;
    r.nominalChamferDepthMm=chamferDepth_;
    const double outerRadius=outerRadius_,innerRadius=innerRadius_,chamferDepth=chamferDepth_;
    zTop_=bounds.max.z;zBottom_=bounds.min.z;
    r.cadTopZMm=zTop_;r.cadBottomZMm=zBottom_;
    r.xCenterCadMm=centerX;r.yCenterCadMm=centerY;
    r.surfaces={
        {"Center upper CAD-derived 45deg chamfer",M51Surface::TopChamfer,PI*(outerRadius+innerRadius)*rootTwo*chamferDepth},
        {"Center cylindrical wall",M51Surface::Cylinder,2.*PI*innerRadius*(holeDepth-2.*chamferDepth)},
        {"Center lower CAD-derived 45deg chamfer",M51Surface::BottomChamfer,PI*(outerRadius+innerRadius)*rootTwo*chamferDepth}
    };
    // Identify contributing STEP triangulation face IDs (does not rely on
    // serial face numbers being stable across STEP importer versions).
    std::array<std::set<std::uint32_t>,3> faces;
    for(const auto& tri:scene.mesh().triangles){
        const Vec3 c=(tri.a+tri.b+tri.c)/3.;
        const double radial=std::hypot(c.x-centerX,c.y-centerY);
        if(radial<innerRadius-.15||radial>outerRadius+.15)continue;
        const double zFromTop=zTop_-c.z;
        const double zFromBottom=c.z-zBottom_;
        const auto normal=normalized(cross(tri.b-tri.a,tri.c-tri.a));
        const double area=triangleArea(tri);
        if(area<1e-12)continue;
        int surface=-1;
        if(zFromTop>=-.01&&zFromTop<=chamferDepth+.01&&std::abs(radial-(outerRadius-zFromTop))<.08&&
           std::abs(normal.z)>.25)surface=0;
        else if(zFromBottom>=-.01&&zFromBottom<=chamferDepth+.01&&std::abs(radial-(outerRadius-zFromBottom))<.08&&
           std::abs(normal.z)>.25)surface=2;
        else if(zFromTop>chamferDepth&&zFromBottom>chamferDepth&&std::abs(radial-innerRadius)<.07&&
                std::abs(normal.z)<.25)surface=1;
        if(surface>=0){
            r.surfaces[std::size_t(surface)].meshAreaMm2+=area;
            faces[std::size_t(surface)].insert(tri.faceId);
        }
    }
    for(int j=0;j<3;++j)r.surfaces[std::size_t(j)].cadFaceIds.assign(faces[std::size_t(j)].begin(),faces[std::size_t(j)].end());
    // A mesh with the expected envelope but no actual hole/chamfer must fail
    // rather than invent a valid "covered" CAD target.
    if(r.surfaces[0].meshAreaMm2<5.||r.surfaces[1].meshAreaMm2<100.||r.surfaces[2].meshAreaMm2<5.){
        r.error="H1 STEP geometry check failed: upper/lower chamfer or cylinder faces not found";return;
    }
    for(int surf=0;surf<3;++surf){
        const auto s=static_cast<M51Surface>(surf);
        for(int layer=0;layer<layerCount(s);++layer)
            for(int a=0;a<M51AngularBins;++a){
                M51Bin b;b.surface=s;b.layer=static_cast<std::uint16_t>(layer);
                b.angle=static_cast<std::uint16_t>(a);b.areaMm2=surfaceArea(s,layer,outerRadius,innerRadius,chamferDepth);
                r.bins.push_back(b);
            }
    }
    r.limitation="H1-specific nominal CAD surfaces; input SCAN missing samples have unknown cause. "
        "Density and surface association are not dimensional accuracy or certified camera visibility.";
}
void TwinM51ChamferAnalyzer::add(Vec3 p,bool virt){
    if(virt)++report_.virtualSeen;else ++report_.realSeen;
    if(!ready()||!finite(p))return;
    const double outerRadius=outerRadius_,innerRadius=innerRadius_,chamferDepth=chamferDepth_;
    const double radius=std::hypot(p.x-centerX,p.y-centerY);
    const double dTop=zTop_-p.z,dBottom=p.z-zBottom_;
    const double m=report_.options.regionMarginMm;
    if(radius<innerRadius-m||radius>outerRadius+m)return;
    const int angle=angularBin(p.x,p.y);
    struct Candidate {M51Surface type;double dist,u;};
    Candidate best{M51Surface::TopChamfer,std::numeric_limits<double>::infinity(),0.};
    // Bounded conical radial-height section. Region gates independent of
    // association threshold prevent a loose 0.65mm sweep from swallowing
    // large areas of neighboring top/underside planes.
    if(dTop>=-m&&dTop<=chamferDepth+m){
        const double u=std::clamp((outerRadius-radius+dTop)/(2.*chamferDepth),0.,1.);
        const double targetR=outerRadius-chamferDepth*u, targetD=chamferDepth*u;
        const double dist=std::hypot(radius-targetR,dTop-targetD);
        best={M51Surface::TopChamfer,dist,u};
        // Use only interior of nominal top chamfer for exploratory profile
        // residuals; never use this correction to move the CAD automatically.
        if(u>=.1&&u<=.9&&dist<=.4){
            auto& residuals=virt?coneResidualVirtual_:coneResidualReal_;
            residuals.push_back(radius+dTop-outerRadius);
        }
    }
    if(dBottom>=-m&&dBottom<=chamferDepth+m){
        const double u=std::clamp((outerRadius-radius+dBottom)/(2.*chamferDepth),0.,1.);
        const double targetR=outerRadius-chamferDepth*u,targetD=chamferDepth*u;
        const double dist=std::hypot(radius-targetR,dBottom-targetD);
        if(dist<best.dist)best={M51Surface::BottomChamfer,dist,u};
    }
    if(dTop>=chamferDepth-m&&dBottom>=chamferDepth-m){
        const double dist=std::abs(radius-innerRadius);
        if(dist<best.dist){
            const double u=std::clamp((dTop-chamferDepth)/(holeDepth-2.*chamferDepth),0.,1.);
            best={M51Surface::Cylinder,dist,u};
        }
    }
    if(!std::isfinite(best.dist))return;
    const int layers=layerCount(best.type);
    const int layer=std::min(layers-1,std::max(0,int(best.u*layers)));
    auto& bin=report_.bins[index(best.type,angle,layer)];
    for(int k=0;k<M51ThresholdCount;++k){
        if(best.dist>report_.options.thresholdsMm[std::size_t(k)])continue;
        auto& hits=virt?bin.virtualHits[std::size_t(k)]:bin.realHits[std::size_t(k)];
        if(hits<std::numeric_limits<std::uint32_t>::max())++hits;
        if(virt)++report_.virtualAssociated[std::size_t(k)];
        else ++report_.realAssociated[std::size_t(k)];
    }
}
void TwinM51ChamferAnalyzer::addRealCad(Vec3 p){add(p,false);}
void TwinM51ChamferAnalyzer::addVirtualCad(Vec3 p){report_.hasVirtual=true;add(p,true);}
M51Report TwinM51ChamferAnalyzer::finish() && {
    if(!ready())return std::move(report_);
    const auto& opt=report_.options;
    std::array<std::uint32_t,M51AngularBins> realAngle{},virtualAngle{};
    for(int surf=0;surf<3;++surf){
        const auto s=static_cast<M51Surface>(surf);
        for(int layer=0;layer<layerCount(s);++layer){
            const auto rRef=quantileUpperQuartile(report_,s,layer,false);
            const auto vRef=quantileUpperQuartile(report_,s,layer,true);
            for(int a=0;a<M51AngularBins;++a){
                auto& b=report_.bins[index(s,a,layer)];
                const std::uint32_t rh=b.realHits[opt.primaryThreshold],vh=b.virtualHits[opt.primaryThreshold];
                b.realRelativeDensity=rRef?double(rh)/rRef:0.;
                b.virtualRelativeDensity=vRef?double(vh)/vRef:0.;
                b.realSparse=rRef>=opt.sparseMinReferenceHits&&b.realRelativeDensity<opt.sparseRatio;
                b.virtualSparse=vRef>=opt.sparseMinReferenceHits&&b.virtualRelativeDensity<opt.sparseRatio;
                if(s==M51Surface::TopChamfer){
                    realAngle[std::size_t(a)]+=rh;virtualAngle[std::size_t(a)]+=vh;
                }
            }
        }
    }
    report_.realConeResidualSamples=coneResidualReal_.size();
    report_.virtualConeResidualSamples=coneResidualVirtual_.size();
    report_.realConeEquationMedianMm=median(coneResidualReal_);
    report_.virtualConeEquationMedianMm=median(coneResidualVirtual_);
    for(auto& x:coneResidualReal_)x=std::abs(x-report_.realConeEquationMedianMm);
    report_.realConeEquationMadMm=median(coneResidualReal_);
    auto ref=realAngle;
    std::sort(ref.begin(),ref.end());
    const std::uint32_t reference=ref[3*M51AngularBins/4];
    for(auto hits:realAngle)if(reference>=opt.sparseMinReferenceHits&&
        double(hits)<double(reference)*opt.sparseRatio)++report_.realLowDensityAngles;
    return std::move(report_);
}
} // namespace mls::twin
