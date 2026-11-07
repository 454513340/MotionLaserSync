#include "twin/MultiPoseRegistrationAssist.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>
#include <vector>

namespace mls::twin {
namespace {
bool finite(Vec3 p) { return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z); }
std::vector<double> finiteValues(const std::vector<Vec3>& pts,int axis){
    std::vector<double> r;r.reserve(pts.size());
    for(const auto& p:pts) if(finite(p)) r.push_back(axis==0?p.x:axis==1?p.y:p.z);
    return r;
}
double quantile(std::vector<double>& v,double f){
    if(v.empty())return std::numeric_limits<double>::quiet_NaN();
    const std::size_t i=std::min(v.size()-1,static_cast<std::size_t>(f*double(v.size()-1)));
    std::nth_element(v.begin(),v.begin()+std::ptrdiff_t(i),v.end());return v[i];
}
struct Kd {
    std::vector<Vec3> pts;
    struct Node {int id=-1,left=-1,right=-1,axis=0;};
    std::vector<Node> nodes;
    std::vector<int> order;
    static double get(Vec3 p,int k){return k==0?p.x:k==1?p.y:p.z;}
    int build(int a,int b,int depth){
        if(a>=b)return -1;
        const int axis=depth%3,mid=a+(b-a)/2;
        std::nth_element(order.begin()+a,order.begin()+mid,order.begin()+b,
            [&](int x,int y){return get(pts[std::size_t(x)],axis)<get(pts[std::size_t(y)],axis);});
        const int n=int(nodes.size());nodes.push_back({order[mid],-1,-1,axis});
        const int l=build(a,mid,depth+1),r=build(mid+1,b,depth+1);
        nodes[std::size_t(n)].left=l;nodes[std::size_t(n)].right=r;return n;
    }
    explicit Kd(std::vector<Vec3> p):pts(std::move(p)){
        order.resize(pts.size());std::iota(order.begin(),order.end(),0);
        nodes.reserve(pts.size());build(0,int(order.size()),0);
    }
    double nearest2(Vec3 q,int n=0,double best=std::numeric_limits<double>::infinity()) const {
        if(n<0||nodes.empty())return best;
        const auto& node=nodes[std::size_t(n)];const Vec3 p=pts[std::size_t(node.id)];
        const Vec3 d=p-q;best=std::min(best,dot(d,d));
        const double side=get(q,node.axis)-get(p,node.axis);
        const int primary=side<0?node.left:node.right;
        const int other=side<0?node.right:node.left;
        if(primary>=0)best=nearest2(q,primary,best);
        if(other>=0&&side*side<best)best=nearest2(q,other,best);
        return best;
    }
};
std::vector<Vec3> sampleSurface(const TriangleMesh& cad){
    double totalArea=0;
    for(const auto& t:cad.triangles)totalArea+=0.5*length(cross(t.b-t.a,t.c-t.a));
    const std::size_t target=22000,cap=65000;
    std::vector<Vec3> points;points.reserve(target+cad.triangles.size());
    if(totalArea<=0)return points;
    for(std::size_t idx=0;idx<cad.triangles.size();++idx){
        const auto& t=cad.triangles[idx];const double area=0.5*length(cross(t.b-t.a,t.c-t.a));
        if(area<1.e-12)continue;
        const std::size_t count=std::max<std::size_t>(1,std::min<std::size_t>(2400,
            std::size_t(std::ceil(double(target)*area/totalArea))));
        for(std::size_t j=0;j<count;++j){
            // Deterministic low-discrepancy barycentric coverage of large CAD faces.
            const double a=std::fmod((double(j)+.5)*.6180339887498949,1.);
            const double b=std::fmod((double(j)+double(idx)*.29+.5)*.4142135623730951,1.);
            const double s=std::sqrt(a),u=1.-s,v=s*(1.-b),w=s*b;
            points.push_back(t.a*u+t.b*v+t.c*w);
            if(points.size()>=cap)return points;
        }
    }
    return points;
}
// Stable machine-Z upper surface band: use a quantile rather than absolute max
// (max may be one optical outlier). Points near this band may still include a
// fixture or other part, hence no assertion that a registration is verified.
struct Band {std::vector<Vec3> pts;double z=0,centerX=0,centerY=0;};
Band selectTopBand(const std::vector<Vec3>& input,std::size_t maxCount){
    Band b;auto zs=finiteValues(input,2);if(zs.size()<250)return b;
    b.z=quantile(zs,.82);
    std::vector<Vec3> filtered;filtered.reserve(input.size()/3);
    for(const auto& p:input)if(finite(p)&&std::abs(p.z-b.z)<.55)filtered.push_back(p);
    if(filtered.size()<150)return b;
    auto xs=finiteValues(filtered,0);
    auto ys=finiteValues(filtered,1);
    b.centerX=.5*(quantile(xs,.035)+quantile(xs,.965));
    b.centerY=.5*(quantile(ys,.035)+quantile(ys,.965));
    const std::size_t stride=std::max<std::size_t>(1,filtered.size()/std::max<std::size_t>(1,maxCount));
    for(std::size_t i=0;i<filtered.size()&&b.pts.size()<maxCount;i+=stride)b.pts.push_back(filtered[i]);
    return b;
}
struct Score {double loss=1e6,inlier=0,median=1e6;std::size_t n=0,hit=0;};
Score quality(const Kd& kd,const std::vector<Vec3>& measured,const Pose6& pose,bool detail=false){
    Score s;const auto inv=Transform::fromPose(pose).inverseRigid();
    std::vector<double> ds;ds.reserve(measured.size());
    // Avoid letting fixture points outside the nominal part volume dominate.
    // Closest CAD surface is still a surrogate, not true inspection residual.
    double sum=0;
    for(const auto& p:measured){
        Vec3 x=inv.point(p);
        const double d=std::sqrt(kd.nearest2(x));
        const double c=std::min(d,2.5);
        sum+=c;
        if(d<1.0)++s.hit;
        if(detail)ds.push_back(d);
        ++s.n;
    }
    if(s.n==0)return s;
    s.inlier=double(s.hit)/double(s.n);
    s.loss=sum/double(s.n);
    if(detail){s.median=quantile(ds,.5);}
    return s;
}
}
RegistrationAssistReport proposeTopSurfaceRegistration(const TriangleMesh& cad,
    const std::vector<Vec3>& measured,std::size_t maximumSamples){
    RegistrationAssistReport r;
    if(cad.empty()||!cad.bounds.valid()){r.message="No usable CAD triangles";return r;}
    if(measured.size()<250){r.message="Not enough real scan points";return r;}
    const auto band=selectTopBand(measured,std::max<std::size_t>(300,std::min<std::size_t>(maximumSamples,3000)));
    r.observedSamples=band.pts.size();
    if(band.pts.size()<150){r.message="No sufficiently populated horizontal top surface; automatic rough placement unsupported";return r;}
    auto sampled=sampleSurface(cad);r.candidateSamples=sampled.size();
    if(sampled.size()<50){r.message="CAD mesh is too sparse for registration proposal";return r;}
    const Kd kd(std::move(sampled));
    const auto c=cad.bounds.center();
    double best=1e9,second=1e9;Pose6 bestPose{};
    for(double rz:{0.,90.,-90.,180.}){
        Pose6 p;p.rzDeg=rz;p.z=band.z-cad.bounds.max.z;
        const Vec3 rot=Transform::fromPose(p).point(c);
        p.x=band.centerX-rot.x;p.y=band.centerY-rot.y;
        // A small deterministic XY translation grid makes the heuristic less
        // brittle to holes, missing edges and partial real coverage.
        double yawBest=1e9;Pose6 yawPose=p;
        for(double dx:{-2.,0.,2.})for(double dy:{-2.,0.,2.}){
            Pose6 candidate=p;candidate.x+=dx;candidate.y+=dy;
            const auto s=quality(kd,band.pts,candidate);
            if(s.loss<yawBest){yawBest=s.loss;yawPose=candidate;}
        }
        // Polish the best local XY offset and Z; no invented arbitrary tilt.
        for(double step:{1.,.3,.1}){
            bool improved=true;int iters=0;
            while(improved&&++iters<12){improved=false;
                for(int ax=0;ax<3;++ax)for(double dir:{-1.,1.}){
                    Pose6 cand=yawPose;
                    if(ax==0)cand.x+=dir*step;
                    if(ax==1)cand.y+=dir*step;
                    if(ax==2)cand.z+=dir*step*.25;
                    const auto loss=quality(kd,band.pts,cand).loss;
                    if(loss+1e-7<yawBest){yawBest=loss;yawPose=cand;improved=true;}
                }
            }
        }
        if(yawBest<best){second=best;best=yawBest;bestPose=yawPose;}
        else second=std::min(second,yawBest);
    }
    const auto s=quality(kd,band.pts,bestPose,true);
    r.proposed=bestPose;r.inliers=s.hit;r.fractionWithinOneMm=s.inlier;
    r.medianSurfaceDistanceMm=s.median;r.orientationMargin=second-best;
    r.orientationAmbiguous=(second-best<.12);
    if(s.inlier<.15||s.hit<100){
        r.message="Rough registration REJECTED: very few measured top-band points agree with the CAD. "
                  "May be a fixture, wrong part, tilted part or partial overlap. "
                  "Provide geometric landmarks or a validated initial pose.";return r;
    }
    r.proposedAvailable=true;
    r.message="Unverified horizontal TOP-SURFACE coarse placement ONLY. "
              "Review CAD/SCAN overlay and verify separate geometric landmarks; "
              "do not use this pose for real machine motion.";
    return r;
}
} // namespace mls::twin
