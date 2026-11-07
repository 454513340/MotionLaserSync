#include "twin/VirtualLaser2950.h"
#include <algorithm>
#include <cmath>
#include <numeric>
#include <utility>

namespace mls::twin {
namespace {
constexpr double eps=1e-9;
bool finite(Vec3 p){return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z);}
bool finiteTransform(const Transform& T){
    for(double d:T.m)if(!std::isfinite(d))return false;
    if(std::abs(T.m[12])>1e-6||std::abs(T.m[13])>1e-6||
       std::abs(T.m[14])>1e-6||std::abs(T.m[15]-1)>1e-6)return false;
    // inverseRigid() assumes a true rotation. A finite but scaled/sheared
    // exposure matrix otherwise silently creates incorrect camera rays/points.
    const Vec3 r0{T.m[0],T.m[1],T.m[2]};
    const Vec3 r1{T.m[4],T.m[5],T.m[6]};
    const Vec3 r2{T.m[8],T.m[9],T.m[10]};
    constexpr double tol=1e-3;
    return std::abs(dot(r0,r0)-1)<tol&&std::abs(dot(r1,r1)-1)<tol&&
           std::abs(dot(r2,r2)-1)<tol&&std::abs(dot(r0,r1))<tol&&
           std::abs(dot(r0,r2))<tol&&std::abs(dot(r1,r2))<tol&&
           std::abs(dot(r0,cross(r1,r2))-1)<tol;
}
bool hitTri(const Ray& r,const Triangle& t,double* dist) {
    const Vec3 e1=t.b-t.a,e2=t.c-t.a,h=cross(r.direction,e2);
    const double det=dot(e1,h);
    if(std::abs(det)<eps)return false;
    const double inv=1./det;
    const Vec3 s=r.origin-t.a;
    const double u=dot(s,h)*inv;
    if(u<0||u>1)return false;
    const Vec3 q=cross(s,e1);
    const double v=dot(r.direction,q)*inv;
    if(v<0||u+v>1)return false;
    const double d=dot(e2,q)*inv;
    if(d<=eps)return false;
    *dist=d;return true;
}
Vec3 centroid(const Triangle& t){return (t.a+t.b+t.c)/3.;}
double comp(Vec3 p,int axis){return axis==0?p.x:axis==1?p.y:p.z;}
bool hitAabb(const Ray& r,const Bounds& box,double limit){
    double nearT=0,farT=limit;
    for(int i=0;i<3;++i){
        const double v=comp(r.direction,i),o=comp(r.origin,i),lo=comp(box.min,i),hi=comp(box.max,i);
        if(std::abs(v)<1e-14){if(o<lo||o>hi)return false;continue;}
        double t0=(lo-o)/v,t1=(hi-o)/v;
        if(t0>t1)std::swap(t0,t1);
        nearT=std::max(nearT,t0);farT=std::min(farT,t1);
        if(farT<nearT)return false;
    }
    return true;
}
}

bool Sensor2950Geometry::validate(std::string* error) const {
    const auto fail=[&](const char* msg){if(error)*error=msg;return false;};
    if(sampleCount<=0||sampleCount>8192)return fail("Invalid profile sampleCount");
    if(!std::isfinite(zNear)||!std::isfinite(zCenter)||!std::isfinite(zFar)||
       !(zNear<zCenter&&zCenter<zFar)||zNear<=0)return fail("Invalid Sensor-Z limits");
    if(!std::isfinite(widthAtNear)||!std::isfinite(widthAtCenter)||
       !std::isfinite(widthAtFar)||
       widthAtNear<=0||widthAtCenter<=0||widthAtFar<=0)return fail("Invalid scan width");
    if(!std::isfinite(sampleXScale)||!std::isfinite(sampleXOffsetMm)||
       sampleXScale<0.8||sampleXScale>1.2||std::abs(sampleXOffsetMm)>5.0)
        return fail("Invalid virtual scan index-X correction");
    if(!finite(laserOriginSensor)||!finite(cameraOriginSensor))return fail("Nonfinite optical centre");
    if(laserOriginSensor.z<=zFar+eps)return fail("Laser virtual origin must be behind highest reported depth");
    if(!std::isfinite(cameraVisibilityEpsilonMm)||!std::isfinite(maxTraceDistanceMm)||
       cameraVisibilityEpsilonMm<0||maxTraceDistanceMm<zFar)
       return fail("Invalid optical tolerances");
    return true;
}
double Sensor2950Geometry::widthAtZ(double z) const {
    if(z<=zCenter){
        const double a=(z-zNear)/(zCenter-zNear);
        return widthAtNear+(widthAtCenter-widthAtNear)*a;
    }
    const double a=(z-zCenter)/(zFar-zCenter);
    return widthAtCenter+(widthAtFar-widthAtCenter)*a;
}
double Sensor2950Geometry::sampleXAtZ(int index,double z) const {
    // H1 SCAN raw profile index increases in the OPPOSITE direction to
    // calibrated Sensor-X: d(Sensor-X)/d(index) < 0 at the measured depth.
    // Use the same convention in GUI playback, batch simulation and export.
    const double t=(double(index)+0.5)/double(sampleCount)-0.5;
    return -t*widthAtZ(z)*sampleXScale+sampleXOffsetMm;
}

TwinRayBvh::TwinRayBvh(const TwinScene& scene,bool useEstimatedPose)
  : triangles_(scene.mesh().triangles),cadToMachine_(useEstimatedPose?scene.cadToMachineEstimate():scene.cadToMachineTruth()),
    machineToCad_(cadToMachine_.inverseRigid()),
    supportEnabled_(scene.supportEnabled()),supportZ_(scene.supportZ()),
    supportX_(scene.supportCenter().x),supportY_(scene.supportCenter().y),
    supportW_(scene.supportSize().x),supportH_(scene.supportSize().y) {
    indices_.resize(triangles_.size());
    std::iota(indices_.begin(),indices_.end(),std::uint32_t(0));
    if(!indices_.empty())buildNode(0,indices_.size());
}
int TwinRayBvh::buildNode(std::size_t begin,std::size_t end){
    const int here=static_cast<int>(nodes_.size());nodes_.emplace_back();
    Bounds b,cent;
    for(std::size_t i=begin;i<end;++i){
        const auto t=triangles_[indices_[i]];
        b.add(t.a);b.add(t.b);b.add(t.c);cent.add(centroid(t));
    }
    nodes_[here].bounds=b;
    const auto size=end-begin;
    if(size<=8){nodes_[here].start=std::uint32_t(begin);nodes_[here].count=std::uint32_t(size);return here;}
    const Vec3 span=cent.size();
    const int axis=span.y>span.x && span.y>=span.z?1:span.z>span.x && span.z>span.y?2:0;
    const auto mid=begin+size/2;
    std::nth_element(indices_.begin()+std::ptrdiff_t(begin),indices_.begin()+std::ptrdiff_t(mid),
                     indices_.begin()+std::ptrdiff_t(end),[&](std::uint32_t a,std::uint32_t b){
        return comp(centroid(triangles_[a]),axis)<comp(centroid(triangles_[b]),axis);
    });
    const int left=buildNode(begin,mid),right=buildNode(mid,end);
    nodes_[here].left=left;nodes_[here].right=right;
    return here;
}
RayHit TwinRayBvh::firstHit(const Ray& input,double maxDist) const {
    RayHit best;
    if(maxDist<=0||!finite(input.origin)||!finite(input.direction))return best;
    const auto L=length(input.direction);
    if(L<=eps)return best;
    const Ray r{input.origin,input.direction/L};
    const Ray cad{machineToCad_.point(r.origin),machineToCad_.vector(r.direction)};
    // Rigid transforms preserve normalized ray length.
    if(!nodes_.empty()){
        int stack[128];int top=0;stack[top++]=0;
        while(top>0){
            const int n=stack[--top];
            const auto& node=nodes_[std::size_t(n)];
            if(!hitAabb(cad,node.bounds,std::min(best.distanceMm,maxDist)))continue;
            if(node.left>=0){stack[top++]=node.left;stack[top++]=node.right;continue;}
            for(std::uint32_t k=0;k<node.count;++k){
                const auto& tri=triangles_[indices_[node.start+k]];
                double t=0;
                if(hitTri(cad,tri,&t)&&t<best.distanceMm&&t<=maxDist){
                    best.hit=true;best.distanceMm=t;
                    best.positionMachine=r.origin+r.direction*t;
                    best.normalMachine=normalized(cadToMachine_.vector(cross(tri.b-tri.a,tri.c-tri.a)));
                    best.faceId=tri.faceId;best.objectName="workpiece";
                }
            }
        }
    }
    if(supportEnabled_&&std::abs(r.direction.z)>1e-12){
        const double d=(supportZ_-r.origin.z)/r.direction.z;
        if(d>1e-9&&d<best.distanceMm&&d<=maxDist){
            const auto p=r.origin+r.direction*d;
            if(std::abs(p.x-supportX_)<=supportW_*.5 &&std::abs(p.y-supportY_)<=supportH_*.5){
                best.hit=true;best.distanceMm=d;best.positionMachine=p;
                best.normalMachine={0,0,1};best.faceId=0;best.objectName="support";
            }
        }
    }
    return best;
}

VirtualLaser2950::VirtualLaser2950(const TwinScene& scene,const Sensor2950Geometry& model,bool useEstimatedPose)
 :geometry_(model),tracer_(scene,useEstimatedPose){}
bool VirtualLaser2950::ready(std::string* error) const {
    if(!geometry_.validate(error))return false;
    if(!tracer_.triangleCount()){
        if(error)*error="No workpiece mesh loaded";
        return false;
    }
    return true;
}
VirtualProfile VirtualLaser2950::simulate(const VirtualProfilePose& pose) const {
    VirtualProfile output;output.pose=pose;
    // simulate() is public; do not allocate using a negative/huge sampleCount
    // if a caller bypasses simulateAll()/ready().
    if(!geometry_.validate())return output;
    output.samples.resize(std::size_t(geometry_.sampleCount));
    if(!finiteTransform(pose.sensorToMachine)){
        for(auto& s:output.samples)s.state=VirtualSampleState::BadPose;
        return output;
    }
    const Transform T=pose.sensorToMachine;
    const Transform inv=T.inverseRigid();
    const Vec3 laserOrigin=T.point(geometry_.laserOriginSensor);
    const Vec3 cameraOrigin=T.point(geometry_.cameraOriginSensor);
    const double zref=geometry_.zCenter;
    for(int i=0;i<geometry_.sampleCount;++i){
        auto& s=output.samples[std::size_t(i)];
        // A ray per index through the laser fan at reference depth. Unlike
        // sampling the CAD, the return is the FIRST intersection on the ray.
        const Vec3 targetSensor{geometry_.sampleXAtZ(i,zref),0,zref};
        const Vec3 direction=T.vector(normalized(targetSensor-geometry_.laserOriginSensor));
        const auto hit=tracer_.firstHit({laserOrigin,direction},geometry_.maxTraceDistanceMm);
        if(!hit.hit){s.state=VirtualSampleState::LaserMiss;continue;}
        // Preserve the first illuminated object's provenance even if the
        // receiver is blocked or the hit is outside the reported field.
        s.hitObject=hit.objectName=="workpiece" ? VirtualHitObject::Workpiece : VirtualHitObject::Support;
        s.cadFaceId=hit.faceId;
        s.hitPositionMachine=hit.positionMachine;
        s.laserNormalCos=static_cast<float>(std::clamp(std::abs(dot(hit.normalMachine,direction)),0.,1.));
        const Vec3 pS=inv.point(hit.positionMachine);
        s.rayHitSensorX=static_cast<float>(pS.x);
        s.rayHitSensorZ=static_cast<float>(pS.z);
        if(pS.z<geometry_.zNear||pS.z>geometry_.zFar ||
           std::abs(pS.x)>geometry_.widthAtZ(pS.z)*.5){
            s.state=VirtualSampleState::OutsideField;continue;
        }
        const Vec3 cameraVec=hit.positionMachine-cameraOrigin;
        const double d=length(cameraVec);
        if(d<1.e-8){s.state=VirtualSampleState::BadPose;continue;}
        s.receiverNormalCos=static_cast<float>(std::clamp(std::abs(dot(hit.normalMachine,cameraVec/d)),0.,1.));
        const Vec3 mirrorDir=normalized(direction-hit.normalMachine*(2.*dot(direction,hit.normalMachine)));
        s.specularReceiverAlignment=static_cast<float>(std::clamp(dot(mirrorDir,cameraVec/(-d)),0.,1.));
        const double clearance=geometry_.cameraVisibilityEpsilonMm;
        if(d>clearance){
            const auto obstruction=tracer_.firstHit({cameraOrigin,cameraVec/d},d-clearance);
            if(obstruction.hit){
                s.cameraOcclusionClearanceMm=static_cast<float>(d-obstruction.distanceMm);
                s.state=VirtualSampleState::CameraOccluded;continue;
            }
        }
        s.sensorX=static_cast<float>(pS.x);
        s.sensorZ=static_cast<float>(pS.z);
        s.state=VirtualSampleState::Valid;
    }
    return output;
}
VirtualScanResult VirtualLaser2950::simulateAll(
    const std::vector<VirtualProfilePose>& poses,const std::atomic<bool>* cancel,
    const std::function<void(std::size_t,std::size_t)>& progress) const {
    VirtualScanResult result;
    if(!ready(&result.error))return result;
    result.profiles.reserve(poses.size());
    for(std::size_t i=0;i<poses.size();++i){
        if(cancel&&cancel->load()){result.cancelled=true;break;}
        auto p=simulate(poses[i]);
        for(const auto& s:p.samples)++result.counts[std::size_t(s.state)];
        result.profiles.push_back(std::move(p));
        if(progress&&(i%32==0||i+1==poses.size()))progress(i+1,poses.size());
    }
    result.totalProfiles=result.profiles.size();
    return result;
}
} // namespace mls::twin
