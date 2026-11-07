#include "twin/TwinM5Coverage.h"
#include "twin/TwinH1Geometry.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <utility>

namespace mls::twin {
namespace {
constexpr double kTiny=1e-12;
bool finite(Vec3 p){return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z);}
double get(Vec3 v,int axis){return axis==0?v.x:axis==1?v.y:v.z;}
Vec3 center(const Bounds& b){return b.center();}
double triangleArea(const Triangle& t){return 0.5*length(cross(t.b-t.a,t.c-t.a));}
double boxDistanceSquared(const Bounds& b,Vec3 p){
    double d=0.;
    for(int i=0;i<3;++i){
        const double x=get(p,i),a=get(b.min,i),z=get(b.max,i);
        const double delta=x<a?a-x:x>z?x-z:0.;d+=delta*delta;
    }
    return d;
}
// Closest-point-on-triangle algorithm (Ericson), with vertex barycentric
// coefficients returned for the same original CAD facet. No projection to an
// infinitely extended triangle plane, especially near sharp hole edges.
struct Projection {Vec3 nearest{};double b=0,c=0;};
Projection closestOnTriangle(Vec3 p,Vec3 a,Vec3 b,Vec3 c){
    const Vec3 ab=b-a,ac=c-a,ap=p-a;
    const double d1=dot(ab,ap),d2=dot(ac,ap);
    if(d1<=0&&d2<=0)return {a,0,0};
    const Vec3 bp=p-b;const double d3=dot(ab,bp),d4=dot(ac,bp);
    if(d3>=0&&d4<=d3)return {b,1,0};
    const double vc=d1*d4-d3*d2;
    if(vc<=0&&d1>=0&&d3<=0){const double t=d1/(d1-d3);return {a+ab*t,t,0};}
    const Vec3 cp=p-c;const double d5=dot(ab,cp),d6=dot(ac,cp);
    if(d6>=0&&d5<=d6)return {c,0,1};
    const double vb=d5*d2-d1*d6;
    if(vb<=0&&d2>=0&&d6<=0){const double t=d2/(d2-d6);return {a+ac*t,0,t};}
    const double va=d3*d6-d5*d4;
    if(va<=0&&(d4-d3)>=0&&(d5-d6)>=0){
        const double t=(d4-d3)/((d4-d3)+(d5-d6));return {b+(c-b)*t,1-t,t};
    }
    const double denom=1./(va+vb+vc);
    const double u=vb*denom,v=vc*denom;
    return {a+ab*u+ac*v,u,v};
}

bool matchesH1(const Bounds& bounds){
    if(!bounds.valid())return false;
    const auto s=bounds.size();
    return std::abs(s.x-30.)<0.75&&std::abs(s.y-95.)<0.75&&std::abs(s.z-10.)<0.5;
}
// H1 only: named features based on the *known* 6061 STEP geometry.
// Generic STEP imports retain top / underside / other CAD geometry grouping.
std::pair<M5FeatureKind,std::string> classify(const Triangle& t,const Bounds& bounds,bool h1){
    const Vec3 mid=(t.a+t.b+t.c)/3.;
    const Vec3 n=normalized(cross(t.b-t.a,t.c-t.a));
    const double top=bounds.max.z,bottom=bounds.min.z;
    if(std::abs(mid.z-bottom)<.12&&n.z<-.7)
        return {M5FeatureKind::BottomExcluded,"CAD underside (default excluded)"};
    if(std::abs(mid.z-top)<.12&&n.z>.7)
        return {M5FeatureKind::Top,"Top plane"};
    if(h1){
        const double xs[]{-7.5,7.5,-7.5,7.5};
        const double ys[]{17.5,17.5,32.5,32.5};
        for(int i=0;i<4;++i){
            const double r=std::hypot(mid.x-xs[i],mid.y-ys[i]);
            if(r>4.35)continue;
            const std::string name="CB"+std::to_string(i+1)+" ";
            if(std::abs(mid.z-5.4)<.20&&n.z>.6&&r>2.0)
                return {M5FeatureKind::CounterboreFloor,name+"landing"};
            if(mid.z>=5.4-.15&&std::abs(n.z)<.85)
                return {M5FeatureKind::CounterboreUpper,name+"upper wall"};
            if(mid.z<5.4+.15&&r<2.85)
                return {M5FeatureKind::CounterboreLower,name+"lower wall"};
        }
        const double r0=std::hypot(mid.x,mid.y+38.);
        if(r0<3.6)return {M5FeatureKind::CenterHole,"Centre through-hole"};
        if(r0>5.5&&r0<15.5&&mid.y<-20.)
            return {M5FeatureKind::ArcSlots,"Curved slots"};
    }
    return {M5FeatureKind::OuterAndOther,"Other CAD side / geometry"};
}
}

TwinM5CoverageBuilder::TwinM5CoverageBuilder(const TwinScene& scene,M5Options options)
  :options_(options),machineToCad_(scene.cadToMachineEstimate().inverseRigid()){
    const auto fail=[&](const char* why){error_=why;result_.error=error_;};
    if(!scene.hasWorkpiece()){fail("M5 requires a loaded CAD mesh");return;}
    if(!std::isfinite(options.cellPitchMm)||options.cellPitchMm<=0.0||
       !std::isfinite(options.maxSurfaceDistanceMm)||options.maxSurfaceDistanceMm<=0.0||
       options.minHitsPerCell==0||options.minDistinctProfilesPerCell==0||options.maxCells<32||options.maxCells>2000000){
        fail("Invalid M5 resolution, distance, hit count or cell limit");return;
    }
    result_.requestedOptions=options;
    const auto& triangles=scene.mesh().triangles;
    const bool h1=matchesH1(scene.mesh().bounds)&&inferH1Geometry(scene).valid;
    result_.h1FeatureTemplate=h1;
    std::vector<std::pair<M5FeatureKind,std::string>> labels;
    labels.reserve(triangles.size());
    std::vector<double> areas(triangles.size(),0.);
    std::vector<double> scale(triangles.size(),0.);
    double areaTotal=0;
    for(std::size_t k=0;k<triangles.size();++k){
        const auto& t=triangles[k];
        labels.push_back(classify(t,scene.mesh().bounds,h1));
        const double area=triangleArea(t);
        if(!std::isfinite(area)) {fail("Nonfinite triangle area");return;}
        areas[k]=area;
        if(area<kTiny)continue;
        // Long/thin triangles need more samples along the long axis, but are
        // bounded to avoid an unbounded n^2 grid from pathological imports.
        const double longest=std::max({length(t.b-t.a),length(t.c-t.a),length(t.b-t.c)});
        scale[k]=std::max(std::sqrt(area)*1.12,longest*.45);
        areaTotal+=area;
    }
    if(!(areaTotal>0)){fail("CAD mesh has no nondegenerate surface");return;}
    double pitch=options.cellPitchMm;
    std::uint64_t nTotal=0;
    for(int attempt=0;attempt<25;++attempt){
        nTotal=0;
        for(std::size_t k=0;k<triangles.size();++k){
            if(areas[k]<kTiny)continue;
            const auto n=std::max(1,static_cast<int>(std::ceil(scale[k]/pitch)));
            if(n>1024){nTotal=std::uint64_t(options.maxCells)+1;break;}
            nTotal+=std::uint64_t(n)*std::uint64_t(n);
        }
        if(nTotal<=options.maxCells)break;
        pitch*=std::max(1.05,std::sqrt(double(nTotal)/double(options.maxCells))*1.03);
    }
    if(nTotal>options.maxCells){fail("CAD surface subdivision exceeded configured cell limit");return;}
    result_.effectiveCellPitchMm=pitch;
    result_.cells.reserve(std::size_t(nTotal));
    tris_.reserve(triangles.size());
    for(std::size_t k=0;k<triangles.size();++k){
        const auto& t=triangles[k];
        if(areas[k]<kTiny)continue;
        const auto& label=labels[k];
        std::uint16_t group=0;
        auto found=std::find_if(result_.features.begin(),result_.features.end(),[&](const M5Feature& f){return f.name==label.second;});
        if(found==result_.features.end()){
            group=static_cast<std::uint16_t>(result_.features.size());
            M5Feature f;f.id=group;f.name=label.second;f.kind=label.first;
            f.target=!(options.excludeBottom&&label.first==M5FeatureKind::BottomExcluded);
            result_.features.push_back(f);
        } else group=found->id;
        const bool target=result_.features[group].target;
        const auto n=static_cast<std::uint32_t>(std::max(1,static_cast<int>(std::ceil(scale[k]/pitch))));
        TriangleInfo info;
        info.a=t.a;info.b=t.b;info.c=t.c;
        info.bounds.add(t.a);info.bounds.add(t.b);info.bounds.add(t.c);
        info.faceId=t.faceId;info.normal=normalized(cross(t.b-t.a,t.c-t.a));
        info.cellStart=result_.cells.size();info.subdivision=n;
        info.featureId=group;info.target=target;
        tris_.push_back(info);
        const double piece=areas[k]/double(n*n);
        for(std::uint32_t i=0;i<n;++i){
            for(std::uint32_t j=0;j<n-i;++j){
                const auto makeCell=[&](double b,double c){
                    M5Cell cell;cell.cadCenter=t.a+(t.b-t.a)*b+(t.c-t.a)*c;
                    cell.cadNormal=info.normal;cell.areaMm2=piece;cell.cadFaceId=t.faceId;
                    cell.sourceTriangle=static_cast<std::uint32_t>(k);
                    cell.featureId=group;cell.target=target;
                    result_.cells.push_back(cell);
                };
                makeCell((double(i)+1./3.)/n,(double(j)+1./3.)/n);
                if(i+j+1<n)makeCell((double(i)+2./3.)/n,(double(j)+2./3.)/n);
            }
        }
    }
    lastRealExposure_.assign(result_.cells.size(),0);
    order_.resize(tris_.size());
    std::iota(order_.begin(),order_.end(),std::uint32_t(0));
    if(!order_.empty())buildNode(0,order_.size());
}

int TwinM5CoverageBuilder::buildNode(std::size_t begin,std::size_t end){
    const int current=static_cast<int>(nodes_.size());nodes_.emplace_back();
    Bounds bounds,centroidBounds;
    for(std::size_t i=begin;i<end;++i){
        const auto& t=tris_[order_[i]];
        bounds.add(t.bounds.min);bounds.add(t.bounds.max);
        centroidBounds.add(center(t.bounds));
    }
    nodes_[current].bounds=bounds;
    if(end-begin<=8){nodes_[current].begin=begin;nodes_[current].end=end;return current;}
    const Vec3 span=centroidBounds.size();
    const int axis=(span.y>=span.x&&span.y>=span.z)?1:(span.z>=span.x&&span.z>=span.y)?2:0;
    const std::size_t middle=begin+(end-begin)/2;
    std::nth_element(order_.begin()+std::ptrdiff_t(begin),order_.begin()+std::ptrdiff_t(middle),
                     order_.begin()+std::ptrdiff_t(end),[&](std::uint32_t a,std::uint32_t b){
        return get(center(tris_[a].bounds),axis)<get(center(tris_[b].bounds),axis);
    });
    const int left=buildNode(begin,middle);
    const int right=buildNode(middle,end);
    nodes_[current].left=left;nodes_[current].right=right;
    return current;
}

TwinM5CoverageBuilder::Closest TwinM5CoverageBuilder::nearest(Vec3 p) const{
    Closest best;
    best.distanceSquared=options_.maxSurfaceDistanceMm*options_.maxSurfaceDistanceMm;
    if(nodes_.empty())return best;
    std::vector<int> stack;stack.reserve(48);stack.push_back(0);
    while(!stack.empty()){
        const auto n=stack.back();stack.pop_back();
        const auto& node=nodes_[std::size_t(n)];
        if(boxDistanceSquared(node.bounds,p)>best.distanceSquared)continue;
        if(node.left>=0){
            const double dLeft=boxDistanceSquared(nodes_[std::size_t(node.left)].bounds,p);
            const double dRight=boxDistanceSquared(nodes_[std::size_t(node.right)].bounds,p);
            if(dLeft<dRight){
                if(dRight<=best.distanceSquared)stack.push_back(node.right);
                if(dLeft<=best.distanceSquared)stack.push_back(node.left);
            }else{
                if(dLeft<=best.distanceSquared)stack.push_back(node.left);
                if(dRight<=best.distanceSquared)stack.push_back(node.right);
            }
            continue;
        }
        for(std::size_t i=node.begin;i<node.end;++i){
            const auto idx=order_[i];const auto& t=tris_[idx];
            const auto projection=closestOnTriangle(p,t.a,t.b,t.c);
            const double d=dot(p-projection.nearest,p-projection.nearest);
            if(d<=best.distanceSquared){
                best={std::size_t(idx),d,projection.b,projection.c,true};
            }
        }
    }
    return best;
}

std::size_t TwinM5CoverageBuilder::cellFor(const TriangleInfo& t,double baryB,double baryC) const{
    const std::uint32_t n=t.subdivision;
    // Clamp off-triangle floating error without changing the closest facet.
    const double u=std::max(0.,std::min(1.,baryB));
    const double v=std::max(0.,std::min(1.-u,baryC));
    const double b=std::min(double(n)-1e-10,double(n)*u);
    const double c=std::min(double(n)-1e-10,double(n)*v);
    std::uint32_t i=std::min(n-1,static_cast<std::uint32_t>(b));
    std::uint32_t j=std::min(n-1,static_cast<std::uint32_t>(c));
    if(i+j>=n)j=n-i-1;
    bool inverted=((b-i)+(c-j)>1.)&&(i+j+1<n);
    const std::size_t offset=std::size_t(2)*n*i-std::size_t(i)*i+std::size_t(2)*j+std::size_t(inverted);
    return t.cellStart+offset;
}

void TwinM5CoverageBuilder::ingest(Vec3 p,bool simulated,std::uint64_t exposureKey){
    auto& seen=simulated?result_.virtualPointsSeen:result_.realPointsSeen;
    auto& accepted=simulated?result_.virtualAccepted:result_.realAccepted;
    auto& excluded=simulated?result_.virtualExcluded:result_.realExcluded;
    auto& unassigned=simulated?result_.virtualNoAssociation:result_.realNoAssociation;
    ++seen;
    if(!ready()||!finite(p)){++unassigned;return;}
    const auto hit=nearest(p);
    if(!hit.found){++unassigned;return;}
    const auto& t=tris_[hit.triangle];
    if(!t.target){++excluded;return;}
    const auto id=cellFor(t,hit.baryB,hit.baryC);
    if(id>=result_.cells.size()){++unassigned;return;}
    auto& h=simulated?result_.cells[id].virtualHits:result_.cells[id].realHits;
    if(h!=std::numeric_limits<std::uint32_t>::max())++h;
    if(!simulated && lastRealExposure_[id]!=exposureKey){
        lastRealExposure_[id]=exposureKey;
        if(result_.cells[id].realDistinctProfiles!=std::numeric_limits<std::uint32_t>::max())
            ++result_.cells[id].realDistinctProfiles;
    }
    ++accepted;
}
void TwinM5CoverageBuilder::addRealMachine(Vec3 p){ingest(machineToCad_.point(p),false);}
void TwinM5CoverageBuilder::addRealMachine(Vec3 p,std::uint64_t key){ingest(machineToCad_.point(p),false,key);}
void TwinM5CoverageBuilder::addVirtualMachine(Vec3 p){ingest(machineToCad_.point(p),true);}
void TwinM5CoverageBuilder::addRealCad(Vec3 p){ingest(p,false);}
void TwinM5CoverageBuilder::addRealCad(Vec3 p,std::uint64_t key){ingest(p,false,key);}
void TwinM5CoverageBuilder::addVirtualCad(Vec3 p){ingest(p,true);}

M5Report TwinM5CoverageBuilder::finish() && {
    if(!ready())return std::move(result_);
    const std::uint32_t minHits=options_.minHitsPerCell;
    for(const auto& c:result_.cells){
        auto& f=result_.features[c.featureId];
        const bool r=c.realHits>=minHits &&
            c.realDistinctProfiles>=options_.minDistinctProfilesPerCell;
        const bool v=c.virtualHits>=minHits;
        if(!c.target){result_.excludedAreaMm2+=c.areaMm2;continue;}
        ++f.totalCells;f.totalAreaMm2+=c.areaMm2;
        result_.targetAreaMm2+=c.areaMm2;
        if(r){++f.realCells;f.realAreaMm2+=c.areaMm2;result_.realAreaMm2+=c.areaMm2;}
        if(v){++f.virtualCells;f.virtualAreaMm2+=c.areaMm2;result_.virtualAreaMm2+=c.areaMm2;}
        if(r&&v){++f.bothCells;f.bothAreaMm2+=c.areaMm2;result_.bothAreaMm2+=c.areaMm2;}
        else if(r)result_.realOnlyAreaMm2+=c.areaMm2;
        else if(v)result_.virtualOnlyAreaMm2+=c.areaMm2;
        else {f.missingAreaMm2+=c.areaMm2;result_.missingBothAreaMm2+=c.areaMm2;}
    }
    return std::move(result_);
}
} // namespace mls::twin
