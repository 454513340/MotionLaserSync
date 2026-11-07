#pragma once
// H1 nominal centre-hole profile from imported CAD tessellation, never from
// a previously fitted scan. Geometry is *checked*, not inferred from a bounding
// box alone. Fixed centre only describes the documented H1 family.
#include "twin/TwinScene.h"
#include <algorithm>
#include <cmath>
#include <vector>
namespace mls::twin {
struct TwinH1Geometry {
    bool valid=false;
    double cx=0.,cy=-38.,innerRadius=0.,outerRadius=0.,chamferDepth=0.;
};
inline TwinH1Geometry inferH1Geometry(const TwinScene& scene) {
    TwinH1Geometry g;
    if(!scene.hasWorkpiece())return g;
    const auto b=scene.mesh().bounds; const auto s=b.size();
    if(std::abs(s.x-30.)>.75||std::abs(s.y-95.)>.75||std::abs(s.z-10.)>.5)return g;
    std::vector<double> outer,inner;
    for(const auto& t:scene.mesh().triangles)for(const auto& p:{t.a,t.b,t.c}){
        const double r=std::hypot(p.x-g.cx,p.y-g.cy);
        if(std::abs(p.z-b.max.z)<.015 && r>3.0 && r<3.8)outer.push_back(r);
        if(p.z>b.min.z+.25 && p.z<b.max.z-.25 && r>2.45 && r<2.95)inner.push_back(r);
    }
    if(outer.size()<40||inner.size()<40)return g;
    std::sort(outer.begin(),outer.end());std::sort(inner.begin(),inner.end());
    g.outerRadius=outer[outer.size()/2];g.innerRadius=inner[inner.size()/2];
    g.chamferDepth=g.outerRadius-g.innerRadius;
    if(g.outerRadius<3.0||g.outerRadius>3.65||g.innerRadius<2.5||g.innerRadius>2.9||
       g.chamferDepth<.35||g.chamferDepth>.8)return g;
    // Confirm the imported CAD has true upper/lower conical facets of the
    // inferred 45-degree section. A similarly-sized block is NOT an H1 part.
    std::size_t upper=0,lower=0,wall=0;
    for(const auto& t:scene.mesh().triangles){
        const auto c=(t.a+t.b+t.c)/3.;
        const auto n=normalized(cross(t.b-t.a,t.c-t.a));
        const double r=std::hypot(c.x-g.cx,c.y-g.cy);
        const double up=b.max.z-c.z,down=c.z-b.min.z;
        if(up>0&&up<g.chamferDepth && std::abs(r+up-g.outerRadius)<.10&&std::abs(n.z)>.2)++upper;
        if(down>0&&down<g.chamferDepth && std::abs(r+down-g.outerRadius)<.10&&std::abs(n.z)>.2)++lower;
        if(up>g.chamferDepth&&down>g.chamferDepth&&std::abs(r-g.innerRadius)<.10&&std::abs(n.z)<.25)++wall;
    }
    g.valid=upper>=8&&lower>=8&&wall>=8;
    return g;
}
} // namespace mls::twin
