#include "twin/TwinM6Continuous.h"
#include <algorithm>
#include <cmath>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <numeric>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace mls::twin {
namespace {
constexpr double pi=kPi;
bool finite(double a){return std::isfinite(a);}
bool finite(Vec3 a){return finite(a.x)&&finite(a.y)&&finite(a.z);}
int clampBin(double d){return int(std::floor(d));}
struct GridKey {int x,y,z;bool operator==(const GridKey& o)const{return x==o.x&&y==o.y&&z==o.z;}};
struct KeyHash {std::size_t operator()(GridKey k)const noexcept {
    std::uint64_t h=1469598103934665603ULL;
    for(int v:{k.x,k.y,k.z}){h^=std::uint32_t(v);h*=1099511628211ULL;}
    return std::size_t(h);
}};
struct CellIndex {
    explicit CellIndex(const M5Report& r,double gate):r_(r),pitch_(std::max(0.1,gate)) {
        for(std::uint32_t i=0;i<r.cells.size();++i){
            const auto& c=r.cells[i];if(c.target&&c.areaMm2>0&&finite(c.cadCenter))
                buckets_[key(c.cadCenter)].push_back(i);
        }
    }
    int nearest(Vec3 p)const {
        if(!finite(p))return -1;
        const auto b=key(p);int idx=-1;double ds=pitch_*pitch_;
        for(int x=-1;x<=1;++x)for(int y=-1;y<=1;++y)for(int z=-1;z<=1;++z){
            const auto it=buckets_.find({b.x+x,b.y+y,b.z+z});
            if(it==buckets_.end())continue;
            for(const auto j:it->second){
                const auto& c=r_.cells[j];const auto d=p-c.cadCenter;
                const double dist=dot(d,d);
                // A nearby unrelated CAD face (e.g. at a sharp step edge)
                // must not steal the hit simply because its cell centroid
                // happens to be closer. Normal-plane residual prevents such
                // cross-face matching while permitting tangential offsets.
                const double normalResidual=std::abs(dot(d,c.cadNormal));
                if(normalResidual>std::min(0.20,pitch_*0.35))continue;
                if(dist<ds){ds=dist;idx=int(j);}
            }
        }
        return idx;
    }
    GridKey key(Vec3 p)const{return {clampBin(p.x/pitch_),clampBin(p.y/pitch_),clampBin(p.z/pitch_)};}
    const M5Report& r_;double pitch_;
    std::unordered_map<GridKey,std::vector<std::uint32_t>,KeyHash> buckets_;
};

// Complete translation plus coupled B/C orientation. The injected FK may be
// full Physical17, not necessarily the nominal orthogonal X/Y/Z axes. Solve
// the 3x3 linear-axis Jacobian rather than assuming qXYZ == tool XYZ.
bool inverse3(const double a[3][3],Vec3 b,Vec3& x) {
    double v[3][4]={{a[0][0],a[0][1],a[0][2],b.x},
                    {a[1][0],a[1][1],a[1][2],b.y},
                    {a[2][0],a[2][1],a[2][2],b.z}};
    for(int k=0;k<3;++k){
        int p=k;for(int i=k+1;i<3;++i)if(std::abs(v[i][k])>std::abs(v[p][k]))p=i;
        if(std::abs(v[p][k])<1e-8)return false;
        for(int j=k;j<4;++j)std::swap(v[k][j],v[p][j]);
        const double denom=v[k][k];for(int j=k;j<4;++j)v[k][j]/=denom;
        for(int i=0;i<3;++i){if(i==k)continue;const double m=v[i][k];for(int j=k;j<4;++j)v[i][j]-=m*v[k][j];}
    }
    x={v[0][3],v[1][3],v[2][3]};return finite(x);
}
bool findJoints(const M6ForwardKinematics& fk,Vec3 desiredMcs,double focusZ,
                double cDeg,double bDeg,Joint5* output){
    if(!output||!fk||!finite(desiredMcs))return false;
    Joint5 q{{0,0,0,cDeg,bDeg}};
    const Vec3 focus{0,0,focusZ};
    const Vec3 base=fk(q).point(focus);
    double J[3][3]{};
    for(int j=0;j<3;++j){auto v=q;v[std::size_t(j)]=1.;
        const auto d=fk(v).point(focus)-base;
        J[0][j]=d.x;J[1][j]=d.y;J[2][j]=d.z;
    }
    Vec3 linear; if(!inverse3(J,desiredMcs-base,linear))return false;
    q[0]=linear.x;q[1]=linear.y;q[2]=linear.z;
    if(length(fk(q).point(focus)-desiredMcs)>0.02)return false;
    *output=q;return true;
}
struct FeatureGroup {
    std::uint16_t id=0;
    std::string name;
    M5FeatureKind kind=M5FeatureKind::OuterAndOther;
    Vec3 center{},min{},max{};
    double radius=1.5, missingAreaMm2=0;
    std::vector<Vec3> routeKnots; // periodic C2 B-spline through target CAD samples
    unsigned int count=0;
};
std::vector<FeatureGroup> groupsFrom(const M5Report& cov,const M6Options& opt){
    std::vector<FeatureGroup> g;
    for(const auto& f:cov.features){
        if(!f.target||f.totalAreaMm2<0.1)continue;
        // 97% complete broad top surface provides little marginal information.
        if(opt.excludeTopIfAlreadyCovered&&f.kind==M5FeatureKind::Top&&f.realCoverage()>0.93)continue;
        FeatureGroup group;group.id=f.id;group.name=f.name;group.kind=f.kind;
        group.min={1e6,1e6,1e6};group.max={-1e6,-1e6,-1e6};
        for(const auto& c:cov.cells){if(!c.target||c.featureId!=f.id)continue;
            group.center=group.center+c.cadCenter;
            group.min.x=std::min(group.min.x,c.cadCenter.x);group.min.y=std::min(group.min.y,c.cadCenter.y);group.min.z=std::min(group.min.z,c.cadCenter.z);
            group.max.x=std::max(group.max.x,c.cadCenter.x);group.max.y=std::max(group.max.y,c.cadCenter.y);group.max.z=std::max(group.max.z,c.cadCenter.z);
            ++group.count;
            if(c.realHits<cov.requestedOptions.minHitsPerCell)group.missingAreaMm2+=c.areaMm2;
        }
        if(!group.count||group.missingAreaMm2<opt.candidateMinGainMm2)continue;
        group.center=group.center/double(group.count);
        const double rx=(group.max.x-group.min.x)*.5;
        const double ry=(group.max.y-group.min.y)*.5;
        group.radius=std::max(0.6,std::min(18.,std::sqrt(rx*rx+ry*ry)*opt.pathRadiusScale));
        // Avoid circling outside 5.5/8mm apertures: the laser stripe itself spans 50mm.
        if(group.kind==M5FeatureKind::CenterHole)group.radius=std::min(group.radius,1.8);
        if(group.kind==M5FeatureKind::CounterboreFloor ||
           group.kind==M5FeatureKind::CounterboreUpper ||
           group.kind==M5FeatureKind::CounterboreLower)group.radius=std::min(group.radius,2.0);
        if(group.kind==M5FeatureKind::OuterAndOther)group.radius=std::min(group.radius,12.);
        // Follow the actual missing CAD geometry instead of assuming a perfect
        // circular workpiece feature. Build an angular, area-weighted ribbon of
        // missing cells; fill gaps from available feature cells. C2 periodic
        // cubic spline interpolation avoids joint-jerk discontinuities.
        constexpr int knots=48;
        std::array<Vec3,knots> missingSum{},allSum{};
        std::array<double,knots> missingWeight{},allWeight{};
        for(const auto& cell:cov.cells){
            if(!cell.target||cell.featureId!=f.id)continue;
            double theta=std::atan2(cell.cadCenter.y-group.center.y,cell.cadCenter.x-group.center.x);
            if(theta<0.)theta+=2.*pi;
            const int bin=std::clamp(int(theta*knots/(2.*pi)),0,knots-1);
            const double area=std::max(1e-7,cell.areaMm2);
            allSum[std::size_t(bin)]=allSum[std::size_t(bin)]+cell.cadCenter*area;
            allWeight[std::size_t(bin)]+=area;
            if(cell.realHits<cov.requestedOptions.minHitsPerCell){
                missingSum[std::size_t(bin)]=missingSum[std::size_t(bin)]+cell.cadCenter*area;
                missingWeight[std::size_t(bin)]+=area;
            }
        }
        group.routeKnots.reserve(knots);
        for(int k=0;k<knots;++k){
            const double a=2.*pi*(k+.5)/knots;
            const Vec3 fallback{group.center.x+group.radius*std::cos(a),
                                group.center.y+group.radius*std::sin(a),group.center.z};
            Vec3 p=fallback;
            if(missingWeight[std::size_t(k)]>1e-6)
                p=missingSum[std::size_t(k)]/missingWeight[std::size_t(k)];
            else if(allWeight[std::size_t(k)]>1e-6)
                p=allSum[std::size_t(k)]/allWeight[std::size_t(k)];
            group.routeKnots.push_back(p);
        }
        g.push_back(group);
    }
    std::stable_sort(g.begin(),g.end(),[](const auto& a,const auto& b){return a.missingAreaMm2>b.missingAreaMm2;});
    return g;
}
double smooth5(double u){return u*u*u*(10.+u*(-15.+6.*u));}
struct Candidate {FeatureGroup feature;unsigned phase=0;double amplitudeScale=1.;double duration=0;double predictedGain=0;double timePenalty=0;double vmax=0,amax=0,jmax=0;
    std::vector<std::uint32_t> expectedCells;std::string error;};
struct M6Curve {
    FeatureGroup g;unsigned phase=0;double amplitudeScale=1.;const M6Options* opt=nullptr;
    const TwinScene* scene=nullptr;const M6ForwardKinematics* fk=nullptr;
    bool at(double u,Joint5& q)const {
        u=std::clamp(u,0.,1.);
        const double s=smooth5(u);
        const double theta=2.*pi*(double(opt->loopsPerSegment)*s+double(phase)*.25);
        const double phaseRad=double(phase)*pi*.5;
        double r=g.radius;
        if(g.kind==M5FeatureKind::Top)r=std::max(1.,g.radius);
        const double breath=1.+.15*std::sin(2.*theta+phaseRad);
        Vec3 cad{g.center.x+r*breath*std::cos(theta),
                 g.center.y+r*breath*std::sin(theta),g.center.z};
        if(g.routeKnots.size()>=4){
            const int sz=int(g.routeKnots.size());
            const double cycles=theta/(2.*pi);
            const double value=(cycles-std::floor(cycles))*sz;
            const int i=int(std::floor(value));const double t=value-i;
            const auto& p0=g.routeKnots[std::size_t((i-1+sz)%sz)];
            const auto& p1=g.routeKnots[std::size_t(i%sz)];
            const auto& p2=g.routeKnots[std::size_t((i+1)%sz)];
            const auto& p3=g.routeKnots[std::size_t((i+2)%sz)];
            const double b0=std::pow(1.-t,3)/6.;
            const double b1=(3.*t*t*t-6.*t*t+4.)/6.;
            const double b2=(-3.*t*t*t+3.*t*t+3.*t+1.)/6.;
            const double b3=t*t*t/6.;
            const Vec3 spline=p0*b0+p1*b1+p2*b2+p3*b3;
            cad.x=spline.x;cad.y=spline.y;
        }
        // Keep the target within a region's extents, not inside an empty void.
        cad.x=std::clamp(cad.x,g.min.x-1.0,g.max.x+1.0);
        cad.y=std::clamp(cad.y,g.min.y-1.0,g.max.y+1.0);
        // Aim near the top entry of a deep void, leaving downward laser rays
        // to intersect the internal walls by genuine geometry, not injected CAD.
        if(g.kind==M5FeatureKind::CenterHole ||
           g.kind==M5FeatureKind::CounterboreLower ||
           g.kind==M5FeatureKind::CounterboreUpper ||
           g.kind==M5FeatureKind::CounterboreFloor ||
           g.kind==M5FeatureKind::ArcSlots) cad.z=std::min(10.,g.max.z);
        // Signed B tilts and wide C sweeps explore BOTH sides of the optical
        // baseline. The physical axes remain limited by the machine model.
        const double tilt=opt->tiltAmplitudeDeg*amplitudeScale*std::cos(theta+phaseRad);
        const double yaw=opt->yawAmplitudeDeg*amplitudeScale*std::sin(theta+phaseRad);
        // Real C/B mechanism only, no imaginary roll axis.
        return findJoints(*fk,scene->cadToMachineEstimate().point(cad),
                          opt->focusSensorZmm,yaw,tilt,&q);
    }
};
bool inBounds(const Joint5& q,const M6AxisBounds& b){
    for(std::size_t i=0;i<5;++i)
        if(!finite(q[i])||q[i]<b.min[i]+1e-7||q[i]>b.max[i]-1e-7)return false;
    return true;
}
struct Dynamics {double duration=0,v=0,a=0,j=0;bool valid=false;};
Dynamics timeParameterize(const M6Curve& curve,const M6Options& options,
                         const M6CollisionCheck& collision,
                         const std::atomic<bool>* cancel=nullptr,
                         const std::chrono::steady_clock::time_point* deadline=nullptr){
    constexpr int n=240;
    std::array<Joint5,n+1> qs{};
    for(int i=0;i<=n;++i){
        if(cancel && cancel->load())return {};
        if(deadline && std::chrono::steady_clock::now()>=*deadline)return {};
        if(!curve.at(double(i)/n,qs[std::size_t(i)])||!inBounds(qs[std::size_t(i)],options.bounds))return {};
        if(collision){
            std::string why;if(!collision(qs[std::size_t(i)],&why))return {};
        }
    }
    double v=0,a=0,j=0;
    for(int i=1;i<=n;++i)for(int ax=0;ax<5;++ax){
        const auto p=std::size_t(i);
        const auto k=std::size_t(ax);
        const double diff=std::abs(qs[p][k]-qs[p-1][k]);
        v=std::max(v,diff*n/options.bounds.maxSpeed[k]);
        if(i>=2){const double d2=std::abs(qs[p][k]-2*qs[p-1][k]+qs[p-2][k]);
            a=std::max(a,d2*n*n/options.bounds.maxAccel[k]);}
        if(i>=3){const double d3=std::abs(qs[p][k]-3*qs[p-1][k]+3*qs[p-2][k]-qs[p-3][k]);
            j=std::max(j,d3*n*n*n/options.bounds.maxJerk[k]);}
    }
    const double time=std::max({1.,v,std::sqrt(a),std::cbrt(j)})*1.20;
    if(!finite(time)||time>options.maxDurationSec)return {};
    return {time,v/time,a/(time*time),j/(time*time*time),true};
}
void accumulateCells(const VirtualProfile& profile,const Transform& toCad,
    const CellIndex& ix,std::unordered_set<std::uint32_t>& result){
    for(const auto& s:profile.samples){
        if(s.state!=VirtualSampleState::Valid||s.hitObject!=VirtualHitObject::Workpiece)continue;
        const Vec3 p=toCad.point(profile.pose.sensorToMachine.point({s.sensorX,0,s.sensorZ}));
        const int id=ix.nearest(p);if(id>=0)result.insert(std::uint32_t(id));
    }
}
std::uint64_t addVirtualData(const VirtualProfile& profile,const Transform& toCad,
    const CellIndex& ix,std::vector<unsigned char>& seen,std::vector<Vec3>& preview,
    std::size_t previewCap,std::uint64_t& previewSeen,std::ofstream* rawOutput,std::uint64_t& totalValid,
    std::uint64_t& exportedValid, unsigned int exportStride,
    double& newArea,const M5Report& cov,
    std::vector<std::uint16_t>& candidateHits,std::vector<double>& byFeature,
    std::uint16_t minimumHits,std::vector<std::uint16_t>* firstCovered=nullptr,
    std::uint16_t segmentNumber=0){
    std::uint64_t hits=0;
    for(const auto& s:profile.samples){
        if(s.state!=VirtualSampleState::Valid||s.hitObject!=VirtualHitObject::Workpiece)continue;
        const auto xyz=profile.pose.sensorToMachine.point({s.sensorX,0,s.sensorZ});
        const auto local=toCad.point(xyz);const int id=ix.nearest(local);
        if(id>=0&&!seen[std::size_t(id)]){
            auto& hits=candidateHits[std::size_t(id)];
            if(hits<minimumHits)++hits;
            if(hits>=minimumHits){
                seen[std::size_t(id)]=1;
                if(firstCovered && std::size_t(id)<firstCovered->size())
                    (*firstCovered)[std::size_t(id)]=segmentNumber;
                const auto& cell=cov.cells[std::size_t(id)];
                newArea+=cell.areaMm2;
                if(cell.featureId<byFeature.size())byFeature[cell.featureId]+=cell.areaMm2;
            }
        }
        // Deterministic bounded per-segment reservoir: late frames have the
        // same chance to appear as early frames, unlike the old first-85k cap.
        ++previewSeen;
        if(previewCap){
            if(preview.size()<previewCap)preview.push_back(xyz);
            else{
                std::uint64_t h=previewSeen+0x9e3779b97f4a7c15ULL;
                h=(h^(h>>30))*0xbf58476d1ce4e5b9ULL;
                h=(h^(h>>27))*0x94d049bb133111ebULL;
                h^=(h>>31);
                const auto slot=h%previewSeen;
                if(slot<previewCap)preview[std::size_t(slot)]=xyz;
            }
        }
        if(rawOutput && (totalValid%std::max(1U,exportStride)==0)){
            const float floats[3]{float(xyz.x),float(xyz.y),float(xyz.z)};
            rawOutput->write(reinterpret_cast<const char*>(floats),sizeof(floats));
            ++exportedValid;
        }
        ++hits;++totalValid;
    }
    return hits;
}
std::vector<M6PathSample> makeTransfer(const Joint5& first,const Joint5& next,
    const M6ForwardKinematics& fk,const M6Options& opt,double baseSeconds){
    double duration=1.;
    for(std::size_t ax=0;ax<5;++ax){
        const auto dist=std::abs(first[ax]-next[ax]);
        duration=std::max(duration,2.4*dist/opt.bounds.maxSpeed[ax]);
        duration=std::max(duration,std::sqrt(8.*dist/opt.bounds.maxAccel[ax]));
        duration=std::max(duration,std::cbrt(65.*dist/opt.bounds.maxJerk[ax]));
    }
    const int n=std::max(4,int(std::ceil(duration*20.)));
    std::vector<M6PathSample> path;path.reserve(std::size_t(n+1));
    for(int i=0;i<=n;++i){
        const double u=double(i)/n,w=smooth5(u);
        Joint5 q{};for(int k=0;k<5;++k)q[std::size_t(k)]=first[std::size_t(k)]*(1.-w)+next[std::size_t(k)]*w;
        path.push_back({baseSeconds+duration*u,q,fk(q),M6SegmentRole::Transfer});
    }
    return path;
}
// RotZ(-C)*RotY(-B), matching Machine5X nominal XYZ-C-B chain.
Transform nominalRotation(double c,double b){
    const double C=-c*pi/180.,B=-b*pi/180.;
    const double cs=std::cos(C),sn=std::sin(C),cb=std::cos(B),sb=std::sin(B);
    Transform T;
    T.m={cs*cb,-sn,cs*sb,0, sn*cb,cs,sn*sb,0,
         -sb,0,cb,0, 0,0,0,1};
    return T;
}
} // namespace

M6ForwardKinematics makeM6NominalFromRecorded(const Joint5& ref,const Transform& reference){
    const Transform R0=nominalRotation(ref[3],ref[4]);
    const Transform fixed=R0.inverseRigid()*reference;
    const Vec3 xyz0{ref[0],ref[1],ref[2]};
    // Anchored tool translation relative to XYZ-C-B rotational intersection.
    const Vec3 pivot=R0.inverseRigid().vector(reference.point({0,0,0})-xyz0);
    Transform mount=fixed;mount.m[3]=pivot.x;mount.m[7]=pivot.y;mount.m[11]=pivot.z;
    return [mount](const Joint5& q){
        Transform translation;translation.m[3]=q[0];translation.m[7]=q[1];translation.m[11]=q[2];
        return translation*nominalRotation(q[3],q[4])*mount;
    };
}

TwinM6ContinuousPlanner::TwinM6ContinuousPlanner(const TwinScene& scene,const M5Report& coverage,
   const Sensor2950Geometry& optics,M6ForwardKinematics fk,M6Options options,M6CollisionCheck collision)
   :scene_(scene),coverage_(coverage),optics_(optics),fk_(std::move(fk)),options_(options),collision_(std::move(collision)){}

M6Report TwinM6ContinuousPlanner::run(const std::atomic<bool>* cancel,
   std::atomic<std::size_t>* finished,const std::string& spoolXYZPath,
   M6ProgressState* liveProgress,
   const M6FullProfileSink& archiveSink) const {
    M6Report report;report.options=options_;report.collisionModelUsed=bool(collision_);
    const auto setStage=[&](M6ProgressStage value){if(liveProgress)liveProgress->stage.store(int(value));};
    setStage(M6ProgressStage::Starting);
    if(finished)finished->store(0);
    report.kinematicsSource="CALLER_SUPPLIED_FK; logged in adapter; offline only";
    if(!scene_.hasWorkpiece()||!coverage_.ok()||!fk_){report.error="M6 needs scene, M5 coverage and calibrated forward kinematics";return report;}
    std::string reason;
    if(!optics_.validate(&reason)){report.error="M6 optics: "+reason;return report;}
    if(options_.scanRateHz<20||options_.scanRateHz>2000||options_.candidateEvaluationFrames<6||
       options_.loopsPerSegment<1||options_.loopsPerSegment>6||options_.maxSelectedSegments==0||
       options_.maxCellDistanceMm<=0||options_.maxDurationSec<1||
       options_.exportPointStride<1||options_.exportPointStride>10000||
       options_.minNewHitsPerCell<1||options_.minNewHitsPerCell>200||
       options_.candidateCoarseFrames<6||options_.maxRefinedCandidates==0||
       options_.maxCandidateCurves==0||options_.maxCandidateSearchSeconds<=0){
        report.error="Invalid M6 options";return report;}
    for(std::size_t i=0;i<5;++i){
        if(options_.bounds.min[i]>=options_.bounds.max[i]||options_.bounds.maxSpeed[i]<=0||
           options_.bounds.maxAccel[i]<=0||options_.bounds.maxJerk[i]<=0){report.error="Invalid axis limits/dynamics";return report;}
    }
    report.targetAreaMm2=coverage_.targetAreaMm2;
    report.featureNewAreaMm2.assign(coverage_.features.size(),0.);
    std::vector<std::uint16_t> newHits(coverage_.cells.size(),0);
    report.firstCoveredSegmentByCell.assign(coverage_.cells.size(),0);
    std::vector<unsigned char> seen(coverage_.cells.size(),0);
    for(std::size_t i=0;i<coverage_.cells.size();++i){
        const auto& c=coverage_.cells[i];
        if(!c.target||c.realHits<coverage_.requestedOptions.minHitsPerCell)continue;
        bool reopen=false;
        // M5.1 flags local density failures which coarse M5 can hide.
        // This is a simulation priority, NOT claiming a real surface is absent.
        if(c.featureId<coverage_.features.size() &&
           coverage_.features[c.featureId].kind==M5FeatureKind::CenterHole &&
           c.cadCenter.z>=9.45 && c.cadCenter.z<=10.05){
            const double radius=std::hypot(c.cadCenter.x,c.cadCenter.y+38.);
            if(radius>=2.74&&radius<=3.26){
                double angle=std::atan2(c.cadCenter.y+38.,c.cadCenter.x);
                if(angle<0.)angle+=2.*pi;
                const int a=std::clamp(int(angle*72./(2.*pi)),0,71);
                const int layer=std::clamp(int((3.25-radius)*8.),0,3);
                reopen=options_.prioritizeSparseChamfer[std::size_t(layer*72+a)]!=0;
            }
        }
        if(reopen){
            report.reopenedSparseChamferAreaMm2+=c.areaMm2;
            report.reopenedSparseCellIds.push_back(static_cast<std::uint32_t>(i));
            continue;
        }
        seen[i]=1;report.initialObservedAreaMm2+=c.areaMm2;
    }
    setStage(M6ProgressStage::Geometry);
    const auto groups=groupsFrom(coverage_,options_);
    if(liveProgress)liveProgress->featuresTotal.store(groups.size());
    if(groups.empty()){report.finalObservedAreaMm2=report.initialObservedAreaMm2;return report;}
    VirtualLaser2950 laser(scene_,optics_);
    if(!laser.ready(&reason)){report.error="Cannot initialize virtual laser: "+reason;return report;}
    const auto toCad=scene_.cadToMachineEstimate().inverseRigid();
    CellIndex index(coverage_,options_.maxCellDistanceMm);
    // The V28.7 version ray-traced every candidate at 36 x 1280 samples
    // BEFORE incrementing the full-profile progress counter. With 15-16 H1
    // CAD groups this can mean millions of rays and several minutes of "0".
    // Use a bounded two-level search: sparse prescreen of all eligible
    // continuous five-axis curves, then full FCL and optical refinement for
    // the highest-value, geographically diverse shortlist.
    std::vector<Candidate> coarse;
    const auto started=std::chrono::steady_clock::now();
    const auto deadline=started+std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(options_.maxCandidateSearchSeconds));
    auto expired=[&](){return std::chrono::steady_clock::now()>=deadline;};
    const std::size_t estimatedCandidates=std::min<std::size_t>(
        groups.size()*std::size_t(12),options_.maxCandidateCurves);
    if(liveProgress)liveProgress->candidatesTotal.store(estimatedCandidates);
    std::size_t attempted=0,rejectedDynamics=0,rejectedCoarseCollision=0;
    bool exhaustedBudget=false;
    setStage(M6ProgressStage::CandidateCoarse);
    for(const auto& group:groups){
        if(exhaustedBudget)break;
        for(unsigned phase=0;phase<4&&!exhaustedBudget;++phase){
            for(const double amplitudeScale:{1.0,0.65,0.35}){
                if(cancel&&cancel->load()){report.cancelled=true;setStage(M6ProgressStage::Cancelled);return report;}
                if(attempted>=options_.maxCandidateCurves || expired()){
                    exhaustedBudget=true;break;
                }
                M6Curve curve{group,phase,amplitudeScale,&options_,&scene_,&fk_};
                ++attempted;
                if(liveProgress)liveProgress->candidatesDone.store(attempted);
                // Inexpensive joint kinematics and motion dynamics first.
                const auto d=timeParameterize(curve,options_,{},cancel,&deadline);
                if(!d.valid){++rejectedDynamics;continue;}
                // Sparse fail-fast FCL, followed by a dense check only when
                // promoted to the refinement shortlist. Never claim this
                // sparse preflight is a collision certificate.
                bool coarseSafe=true;
                if(collision_){
                    for(int k=0;k<=16;++k){
                        if(cancel&&cancel->load()){
                            report.cancelled=true;setStage(M6ProgressStage::Cancelled);return report;
                        }
                        Joint5 q{};
                        if(!curve.at(double(k)/16.,q)||!collision_(q,nullptr)){
                            coarseSafe=false;break;
                        }
                    }
                }
                if(!coarseSafe){++rejectedCoarseCollision;continue;}
                Candidate c;c.feature=group;c.phase=phase;c.amplitudeScale=amplitudeScale;
                c.duration=d.duration;c.vmax=d.v;c.amax=d.a;c.jmax=d.j;
                std::unordered_set<std::uint32_t> ids;
                for(unsigned k=0;k<options_.candidateCoarseFrames;++k){
                    if(cancel&&cancel->load()){
                        report.cancelled=true;setStage(M6ProgressStage::Cancelled);return report;
                    }
                    Joint5 q{};
                    const double u=double(k)/double(options_.candidateCoarseFrames-1);
                    if(!curve.at(u,q))break;
                    VirtualProfilePose pose;
                    pose.elapsedSeconds=d.duration*u;
                    pose.joints=q;pose.sensorToMachine=fk_(q);
                    accumulateCells(laser.simulate(pose),toCad,index,ids);
                    ++report.candidateProfilesSimulated;
                    if(liveProgress)liveProgress->candidateProfiles.fetch_add(1);
                }
                for(const auto id:ids)if(!seen[id])c.predictedGain+=coverage_.cells[id].areaMm2;
                if(c.predictedGain>=std::max(0.05,options_.candidateMinGainMm2*.15)){
                    coarse.push_back(std::move(c));
                    if(liveProgress)liveProgress->candidatesFeasible.store(coarse.size());
                }
            }
        }
    }
    report.candidateCurvesAttempted=attempted;
    report.candidateCurvesFeasible=coarse.size();
    report.candidateSearchTruncated=exhaustedBudget;
    if(cancel&&cancel->load()){report.cancelled=true;setStage(M6ProgressStage::Cancelled);return report;}
    if(coarse.empty()){
        report.error="No useful continuous candidate after " +std::to_string(attempted)+
            " coarse attempts (rejected dynamics="+std::to_string(rejectedDynamics)+
            ", FCL="+std::to_string(rejectedCoarseCollision)+
            "). Check B/C travel, Physical17, CAD pose and laser field.";
        setStage(M6ProgressStage::Failed);return report;
    }
    // Prefer good gain/second, but preserve coverage of different features.
    std::vector<std::size_t> ranking(coarse.size());
    std::iota(ranking.begin(),ranking.end(),std::size_t(0));
    std::stable_sort(ranking.begin(),ranking.end(),[&](std::size_t a,std::size_t b){
        return coarse[a].predictedGain/(coarse[a].duration+3.)>
               coarse[b].predictedGain/(coarse[b].duration+3.);
    });
    std::vector<std::size_t> chosen;
    chosen.reserve(std::min<std::size_t>(options_.maxRefinedCandidates,coarse.size()));
    std::unordered_set<std::size_t> selected;
    std::unordered_map<std::uint16_t,int> featureCount;
    for(int quota=0;quota<2;++quota){
        for(const auto idx:ranking){
            if(chosen.size()>=options_.maxRefinedCandidates)break;
            if(featureCount[coarse[idx].feature.id]>quota)continue;
            if(selected.insert(idx).second){
                chosen.push_back(idx);++featureCount[coarse[idx].feature.id];
            }
        }
    }
    for(const auto idx:ranking){
        if(chosen.size()>=options_.maxRefinedCandidates)break;
        if(selected.insert(idx).second)chosen.push_back(idx);
    }
    if(liveProgress)liveProgress->refinementsTotal.store(chosen.size());
    setStage(M6ProgressStage::CandidateRefine);
    std::vector<Candidate> candidates;
    candidates.reserve(chosen.size());
    for(const auto idx:chosen){
        if(cancel&&cancel->load()){report.cancelled=true;setStage(M6ProgressStage::Cancelled);return report;}
        if(expired()){report.candidateSearchTruncated=true;break;}
        const auto& rough=coarse[idx];
        M6Curve curve{rough.feature,rough.phase,rough.amplitudeScale,&options_,&scene_,&fk_};
        // This dense FCL check is mandatory for EVERY accepted path candidate.
        const auto d=timeParameterize(curve,options_,collision_,cancel,&deadline);
        if(!d.valid){
            if(liveProgress)liveProgress->refinementsDone.fetch_add(1);
            continue;
        }
        Candidate c=rough;c.expectedCells.clear();c.predictedGain=0;
        c.duration=d.duration;c.vmax=d.v;c.amax=d.a;c.jmax=d.j;
        std::unordered_set<std::uint32_t> ids;
        for(unsigned k=0;k<options_.candidateEvaluationFrames;++k){
            if(cancel&&cancel->load()){
                report.cancelled=true;setStage(M6ProgressStage::Cancelled);return report;
            }
            Joint5 q{};
            const double u=double(k)/double(options_.candidateEvaluationFrames-1);
            if(!curve.at(u,q))break;
            VirtualProfilePose pose;
            pose.elapsedSeconds=d.duration*u;
            pose.joints=q;pose.sensorToMachine=fk_(q);
            accumulateCells(laser.simulate(pose),toCad,index,ids);
            ++report.candidateProfilesSimulated;
            if(liveProgress)liveProgress->candidateProfiles.fetch_add(1);
        }
        c.expectedCells.assign(ids.begin(),ids.end());
        for(const auto id:ids)if(!seen[id])c.predictedGain+=coverage_.cells[id].areaMm2;
        if(c.predictedGain>=options_.candidateMinGainMm2)candidates.push_back(std::move(c));
        if(liveProgress)liveProgress->refinementsDone.fetch_add(1);
    }
    report.candidateCurvesRefined=candidates.size();
    if(expired())report.candidateSearchTruncated=true;
    if(candidates.empty()){
        report.error="No candidate passed dense IK/dynamics/FCL and optical refinement "
            "(coarse viable="+std::to_string(coarse.size())+
            ", refined attempts="+std::to_string(chosen.size())+
            "). Try checking the machine limits, safety model and optical pose.";
        setStage(M6ProgressStage::Failed);return report;
    }
    setStage(M6ProgressStage::FullSimulation);
    std::ofstream spool;
    if(!spoolXYZPath.empty()){
        spool.open(spoolXYZPath,std::ios::binary|std::ios::trunc);
        if(!spool){report.error="Unable to open M6 full point spool";return report;}
        report.rawFloatXYZPath=spoolXYZPath;
    }
    double timeline=0.;bool haveEnd=options_.hasInitialJoints;
    Joint5 previous=options_.initialJoints;
    std::vector<unsigned char> used(candidates.size(),0);
    for(unsigned iter=0;iter<options_.maxSelectedSegments;++iter){
        if(cancel&&cancel->load()){report.cancelled=true;break;}
        int best=-1;double bestScore=0,bestArea=0;
        for(std::size_t cidx=0;cidx<candidates.size();++cidx){
            if(used[cidx])continue;
            const auto& c=candidates[cidx];double area=0;
            for(const auto idx:c.expectedCells)if(!seen[idx])area+=coverage_.cells[idx].areaMm2;
            const double score=area/(c.duration+3.);
            if(area>=options_.candidateMinGainMm2&&score>bestScore){
                best=int(cidx);bestScore=score;bestArea=area;
            }
        }
        if(best<0)break;
        auto& c=candidates[std::size_t(best)];used[std::size_t(best)]=1;
        M6Curve curve{c.feature,c.phase,c.amplitudeScale,&options_,&scene_,&fk_};
        Joint5 qStart{};if(!curve.at(0.,qStart)){continue;}
        const std::uint64_t n=std::uint64_t(std::ceil(c.duration*options_.scanRateHz))+1;
        // Do not mutate report/path for a rejected over-budget segment.
        if(n>options_.maxFullProfiles || n+report.fullProfiles>options_.maxFullProfiles ||
           timeline+c.duration>options_.maxTotalDurationSec)continue;
        if(haveEnd){
            auto transfer=makeTransfer(previous,qStart,fk_,options_,timeline);
            bool allowed=true;
            for(const auto& sample:transfer){
                if(!inBounds(sample.joints,options_.bounds) || (collision_ && !collision_(sample.joints,nullptr))){allowed=false;break;}
            }
            if(!allowed)continue;
            if(timeline+(transfer.back().seconds-timeline)+c.duration>options_.maxTotalDurationSec)continue;
            timeline=transfer.back().seconds;
            // TRANSFER joints remain in report.path and the CSV, but do not
            // draw them as laser-on scanning paths.
            report.path.insert(report.path.end(),transfer.begin(),transfer.end());
        }
        M6SegmentSummary summary;
        summary.featureId=c.feature.id;summary.feature=c.feature.name;summary.phaseId=c.phase;
        summary.amplitudeScale=c.amplitudeScale;
        summary.startSeconds=timeline;summary.durationSec=c.duration;
        summary.predictedNewAreaMm2=bestArea;
        summary.vmaxUsage=c.vmax;summary.amaxUsage=c.amax;summary.jmaxUsage=c.jmax;
        summary.jointLimitsPassed=true;summary.dynamicsPassed=true;
        summary.collisionChecked=bool(collision_);
        summary.status=collision_?"SIMULATED_WITH_MACHINE_FCL_CHECKS; WORKPIECE_HOUSING_NOT_CERTIFIED":
                                  "SIMULATED_KINEMATICS_ONLY; COLLISION_UNVERIFIED";
        const double before=report.newAreaMm2;
        M6SegmentPreview segmentPreview;
        segmentPreview.featureId=c.feature.id;
        const std::size_t segmentPreviewQuota=std::max<std::size_t>(1,
            options_.previewLimit/std::max(1U,options_.maxSelectedSegments));
        segmentPreview.points.reserve(segmentPreviewQuota);
        if(liveProgress)liveProgress->activeSegmentNumber.store(report.segments.size()+1);
        // Each scan segment is a separate polyline. Never connect its end
        // across an unexposed transfer to the next region.
        const double nan=std::numeric_limits<double>::quiet_NaN();
        report.pathPreview.push_back({nan,nan,nan});
        if(liveProgress){
            liveProgress->activeSegmentProfilesTotal.store(n);
            liveProgress->activeSegmentProfilesDone.store(0);
        }
        for(std::uint64_t i=0;i<n;++i){
            if(cancel&&cancel->load()){report.cancelled=true;break;}
            const double u=double(i)/double(n-1);
            Joint5 q{};if(!curve.at(u,q)||!inBounds(q,options_.bounds)){report.error="Continuous path pose invalid";break;}
            VirtualProfilePose pose;
            pose.elapsedSeconds=timeline+u*c.duration;
            pose.joints=q;pose.sensorToMachine=fk_(q);
            const auto p=laser.simulate(pose);
            if(archiveSink){
                std::string archiveError;
                if(!archiveSink(p,report.segments.size()+1,&archiveError)){
                    report.error="M6 simulated SCAN archive failed: "+archiveError;
                    break;
                }
                if(liveProgress)liveProgress->archiveProfilesSaved.fetch_add(1);
            }
            summary.virtualValidPoints+=addVirtualData(p,toCad,index,seen,
                segmentPreview.points,segmentPreviewQuota,segmentPreview.observedSamples,
                spool?&spool:nullptr,report.fullValidPoints,report.exportedValidPoints,
                options_.exportPointStride,report.newAreaMm2,coverage_,newHits,
                report.featureNewAreaMm2,options_.minNewHitsPerCell,
                &report.firstCoveredSegmentByCell,
                static_cast<std::uint16_t>(report.segments.size()+1));
            ++summary.emittedProfiles;++report.fullProfiles;
            // Preserve every exposure-time XYZCB joint and Physical17 matrix.
            // Preview is decimated separately, never the exported trajectory.
            report.path.push_back({pose.elapsedSeconds,q,pose.sensorToMachine,M6SegmentRole::Scan});
            if((i%std::max<std::uint64_t>(1,n/160))==0||i+1==n){
                // Yellow is the nominal laser focus on/near the workpiece,
                // NOT the sensor coordinate origin 95 mm away.
                report.pathPreview.push_back(
                    pose.sensorToMachine.point({0,0,options_.focusSensorZmm}));
            }
            if(finished && (i%24==0||i+1==n))finished->store(std::size_t(report.fullProfiles));
            if(liveProgress && (i%24==0||i+1==n)){
                liveProgress->fullProfiles.store(report.fullProfiles);
                liveProgress->activeSegmentProfilesDone.store(i+1);
            }
            if(!report.rawFloatXYZPath.empty() && spool.fail()){
                report.error="M6 binary XYZ spool write failure";break;
            }
        }
        if(report.cancelled||!report.error.empty())break;
        summary.measuredNewAreaMm2=report.newAreaMm2-before;
        report.segments.push_back(summary);
        report.segmentPreviews.push_back(std::move(segmentPreview));
        if(liveProgress)liveProgress->segmentsCompleted.store(report.segments.size());
        timeline+=c.duration;
        curve.at(1.,previous);haveEnd=true;
    }
    if(!report.cancelled && report.error.empty() && options_.hasInitialJoints &&
       options_.returnToStartAfterScan && haveEnd && !report.segments.empty()){
        auto home=makeTransfer(previous,options_.initialJoints,fk_,options_,timeline);
        bool safe=home.back().seconds<=options_.maxTotalDurationSec;
        for(const auto& point:home){
            if(!inBounds(point.joints,options_.bounds)||
               (collision_&&!collision_(point.joints,nullptr))){safe=false;break;}
        }
        if(safe){
            // The return-home TRANSFER is recorded but is not laser-on.
            report.path.insert(report.path.end(),home.begin(),home.end());
            report.returnedToReferencePose=true;
        }else{
            report.transferSafetyWarning="Return-to-recorded-reference trajectory was not validated; do not execute";
        }
    }
    if(spool){spool.flush();if(!spool&&report.error.empty())report.error="M6 point file flush failed";spool.close();}
    // Flatten without pulling extra CAD points into the measured cloud;
    // the full measured returns remain in lossless simulated SCAN volumes.
    report.pointPreview.clear();
    for(const auto& part:report.segmentPreviews)
        report.pointPreview.insert(report.pointPreview.end(),part.points.begin(),part.points.end());
    report.finalObservedAreaMm2=report.initialObservedAreaMm2+report.newAreaMm2;
    if(!report.cancelled && report.error.empty() && report.segments.empty()){
        report.error="Candidate planning succeeded, but no segment met scan time/profile/transition limits. "
            "Reduce duration or examine trajectory safety limits.";
    }
    setStage(report.cancelled?M6ProgressStage::Cancelled:
             !report.error.empty()?M6ProgressStage::Failed:M6ProgressStage::Finished);
    return report;
}
} // namespace mls::twin
