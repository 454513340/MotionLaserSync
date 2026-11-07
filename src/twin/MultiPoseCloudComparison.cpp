#include "twin/MultiPoseCloudComparison.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
namespace mls::twin {
namespace {
using P=Vec3;
double v(const P& p,int axis){return axis==0?p.x:axis==1?p.y:p.z;}
double sq(const P& a,const P& b){return (a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y)+(a.z-b.z)*(a.z-b.z);}
bool finite(const P& p){return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z);}
struct Tree {
    struct Node {int pt=-1,left=-1,right=-1,axis=0;};
    std::vector<P> points;
    std::vector<Node> nodes;
    std::vector<int> indices;
    int root=-1;
    explicit Tree(const std::vector<P>& src):points(src){
        indices.resize(points.size());std::iota(indices.begin(),indices.end(),0);
        nodes.reserve(points.size());root=build(0,indices.size(),0);
    }
    int build(std::size_t b,std::size_t e,int depth){
        if(b>=e)return -1;
        const int axis=depth%3;
        const auto mid=b+(e-b)/2;
        std::nth_element(indices.begin()+b,indices.begin()+mid,indices.begin()+e,
            [&](int a,int c){return v(points[std::size_t(a)],axis)<v(points[std::size_t(c)],axis);});
        const int index=int(nodes.size());
        nodes.push_back({indices[mid],-1,-1,axis});
        const int left=build(b,mid,depth+1),right=build(mid+1,e,depth+1);
        nodes[std::size_t(index)].left=left;nodes[std::size_t(index)].right=right;
        return index;
    }
    void search(int n,const P& p,double& best)const{
        if(n<0)return;
        const auto& node=nodes[std::size_t(n)];
        const P& candidate=points[std::size_t(node.pt)];
        best=std::min(best,sq(p,candidate));
        const double d=v(p,node.axis)-v(candidate,node.axis);
        const int near=d<0?node.left:node.right;
        const int far=d<0?node.right:node.left;
        search(near,p,best);
        if(d*d<=best)search(far,p,best);
    }
    float distance(const P& p)const{
        double best=std::numeric_limits<double>::infinity();search(root,p,best);
        return float(std::sqrt(best));
    }
};
std::vector<P> bounded(const std::vector<P>& input,std::size_t maximum){
    std::vector<P> out;out.reserve(std::min(input.size(),maximum));
    const std::size_t step=std::max<std::size_t>(1,(input.size()+maximum-1)/maximum);
    for(std::size_t i=0;i<input.size();i+=step)if(finite(input[i]))out.push_back(input[i]);
    return out;
}
double percentile(std::vector<float> distances,double p){
    if(distances.empty())return 0;
    const std::size_t k=std::min(distances.size()-1,std::size_t(p*double(distances.size()-1)));
    std::nth_element(distances.begin(),distances.begin()+k,distances.end());
    return distances[k];
}
}
MultiPoseCloudMetrics compareMultiPoseClouds(const std::vector<Vec3>& actual,
     const std::vector<Vec3>& virtualPoints,double tolerance,std::size_t maxPoints){
    MultiPoseCloudMetrics result;
    if(!(std::isfinite(tolerance)&&tolerance>0&&maxPoints>=100)){
        result.error="Invalid comparison tolerance or point budget";return result;
    }
    const auto a=bounded(actual,maxPoints);
    const auto b=bounded(virtualPoints,maxPoints);
    if(a.empty()||b.empty()){result.error="Real or synthetic cloud is empty";return result;}
    result.realCount=a.size();result.syntheticCount=b.size();result.toleranceMm=tolerance;
    Tree treeB(b);
    result.realDistancesMm.reserve(a.size());
    std::size_t matched=0;
    for(const auto& pt:a){const float d=treeB.distance(pt);
        result.realDistancesMm.push_back(d);if(d<=tolerance)++matched;
    }
    result.realToSimMedianMm=percentile(result.realDistancesMm,.50);
    result.realToSimP95Mm=percentile(result.realDistancesMm,.95);
    result.realWithinTolerance=double(matched)/double(a.size());
    Tree treeA(a);std::vector<float> reverse;reverse.reserve(b.size());matched=0;
    for(const auto& pt:b){const float d=treeA.distance(pt);
        reverse.push_back(d);if(d<=tolerance)++matched;
    }
    result.simToRealMedianMm=percentile(reverse,.50);
    result.simToRealP95Mm=percentile(reverse,.95);
    result.simWithinTolerance=double(matched)/double(b.size());
    result.valid=true;
    return result;
}
}
