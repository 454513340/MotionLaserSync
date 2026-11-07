#include "calibration/SingleSphere14Calibration.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace mls::calibration;

namespace {

constexpr double kPi = 3.14159265358979323846;

void require(bool condition, const std::string& message)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << std::endl;
        std::exit(1);
    }
}

Vec3d add(const Vec3d& a, const Vec3d& b)
{
    return Vec3d(a.x + b.x, a.y + b.y, a.z + b.z);
}

Vec3d sub(const Vec3d& a, const Vec3d& b)
{
    return Vec3d(a.x - b.x, a.y - b.y, a.z - b.z);
}

Vec3d mul(const Vec3d& a, double s)
{
    return Vec3d(a.x * s, a.y * s, a.z * s);
}

double norm(const Vec3d& a)
{
    return std::sqrt(a.x*a.x + a.y*a.y + a.z*a.z);
}

bool solve3(const Vec3d& c0, const Vec3d& c1, const Vec3d& c2,
            const Vec3d& rhs, Vec3d& x)
{
    double a[3][4] = {
        {c0.x, c1.x, c2.x, rhs.x},
        {c0.y, c1.y, c2.y, rhs.y},
        {c0.z, c1.z, c2.z, rhs.z}
    };
    for (int k=0;k<3;++k) {
        int p=k;
        for (int r=k+1;r<3;++r) if (std::abs(a[r][k])>std::abs(a[p][k])) p=r;
        if (std::abs(a[p][k])<1e-14) return false;
        if (p!=k) for(int c=k;c<4;++c) std::swap(a[p][c],a[k][c]);
        for(int r=k+1;r<3;++r) {
            const double f=a[r][k]/a[k][k];
            for(int c=k;c<4;++c) a[r][c]-=f*a[k][c];
        }
    }
    double out[3]{};
    for(int r=2;r>=0;--r) {
        double value=a[r][3];
        for(int c=r+1;c<3;++c) value-=a[r][c]*out[c];
        out[r]=value/a[r][r];
    }
    x=Vec3d(out[0],out[1],out[2]);
    return true;
}

JointPose5d poseForTarget(double bDeg, double cDeg,
                          const Vec3d& targetCenterSensor,
                          const Vec3d& fixedSphereCenter,
                          const Gauge14Parameters& p)
{
    JointPose5d q0;
    q0.b=bDeg;
    q0.c=cDeg;
    const Vec3d origin=SingleSphere14Model::mapSensorPoint(q0,targetCenterSensor,p);

    JointPose5d qx=q0; qx.x=1.0;
    JointPose5d qy=q0; qy.y=1.0;
    JointPose5d qz=q0; qz.z=1.0;
    const Vec3d cx=sub(SingleSphere14Model::mapSensorPoint(qx,targetCenterSensor,p),origin);
    const Vec3d cy=sub(SingleSphere14Model::mapSensorPoint(qy,targetCenterSensor,p),origin);
    const Vec3d cz=sub(SingleSphere14Model::mapSensorPoint(qz,targetCenterSensor,p),origin);
    Vec3d xyz;
    require(solve3(cx,cy,cz,sub(fixedSphereCenter,origin),xyz),"solve XYZ target");
    q0.x=xyz.x; q0.y=xyz.y; q0.z=xyz.z;
    return q0;
}

SphereProfileObservation makeProfile(std::int64_t id,
                                     const JointPose5d& q,
                                     const Vec3d& centerSensor,
                                     double diameter,
                                     int count=25)
{
    SphereProfileObservation obs;
    obs.profileId=id;
    obs.joints=q;
    const double r=diameter*0.5;
    const double rho=std::sqrt(r*r-centerSensor.y*centerSensor.y);
    for(int i=0;i<count;++i) {
        const double t=static_cast<double>(i)/(count-1);
        const double x=centerSensor.x+(-0.9+1.8*t)*rho;
        const double dx=x-centerSensor.x;
        SensorProfilePoint point;
        point.x=x;
        point.z=centerSensor.z-std::sqrt(std::max(0.0,rho*rho-dx*dx));
        obs.points.push_back(point);
    }
    return obs;
}

Gauge14Parameters truthParameters()
{
    Gauge14Parameters p;
    p[Gauge14Parameters::Sxy]=90e-6;
    p[Gauge14Parameters::Sxz]=-70e-6;
    p[Gauge14Parameters::Syz]=55e-6;
    p[Gauge14Parameters::COffsetU]=-34.8882;
    p[Gauge14Parameters::COffsetV]=11.89934;
    p[Gauge14Parameters::CTiltX]=130e-6;
    p[Gauge14Parameters::CTiltY]=-100e-6;
    p[Gauge14Parameters::BOffsetU]=-35.158318;
    p[Gauge14Parameters::BOffsetV]=-81.88868;
    p[Gauge14Parameters::BTiltX]=-110e-6;
    p[Gauge14Parameters::BTiltZ]=140e-6;
    p[Gauge14Parameters::HandEyeRx]=15e-3;
    p[Gauge14Parameters::HandEyeRy]=-22e-3;
    p[Gauge14Parameters::HandEyeRz]=10e-3;
    return p;
}

void testSphereCenterFit()
{
    const double diameter=19.05;
    const Vec3d center(2.0,0.40*(diameter*0.5),95.0);
    JointPose5d q;
    const SphereProfileObservation obs=makeProfile(1,q,center,diameter,31);
    const SphereCenterFit fit=SingleSphere14Calibrator::fitSphereCenterInSensor(
        obs.points,diameter,+1);
    require(fit.success,"sphere center fit succeeded");
    require(norm(sub(fit.centerSensorMm,center))<1e-7,"sphere center fit accuracy");
}

void testSphereCenterRansacRejectsBackground()
{
    const double diameter=19.05;
    const Vec3d center(-1.5,0.35*(diameter*0.5),96.0);
    JointPose5d q;
    SphereProfileObservation obs=makeProfile(2,q,center,diameter,35);

    // Simulate a fixture/background edge and isolated boundary returns inside
    // the coarse software ROI. These points must not reach the 14-D solver.
    for (int i=0;i<45;++i) {
        SensorProfilePoint p;
        p.x=-15.0+30.0*static_cast<double>(i)/44.0;
        p.z=84.0+0.02*p.x;
        obs.points.push_back(p);
    }
    for (int i=0;i<12;++i) {
        SensorProfilePoint p;
        p.x=-12.0+2.0*i;
        p.z=104.0-(i%4)*3.5;
        obs.points.push_back(p);
    }

    SphereArcRansacOptions options;
    options.maxIterations=600;
    options.inlierThresholdMm=0.025;
    options.minInlierRatio=0.20;
    options.minArcSpanDeg=40.0;
    std::vector<std::size_t> inliers;
    const SphereCenterFit fit=SingleSphere14Calibrator::fitSphereCenterInSensor(
        obs.points,diameter,+1,options,&inliers);
    require(fit.success,"RANSAC sphere fit succeeded: "+fit.message);
    require(norm(sub(fit.centerSensorMm,center))<1e-5,"RANSAC recovers sphere center with background");
    require(inliers.size()>=30 && inliers.size()<obs.points.size(),"RANSAC exports only sphere-arc inliers");
    require(fit.arcSpanDeg>40.0,"RANSAC validates useful arc span");
}

std::vector<SphereProfileObservation> makeDiverseData(const Gauge14Parameters& truth,
                                                       const Vec3d& sphereCenter,
                                                       double diameter)
{
    const double bValues[]={-50,-35,-20,0,20,35,50};
    const double cValues[]={-70,-45,-20,0,25,50,70};
    const double xTargets[]={-5,0,5};
    const double zTargets[]={88,95,102};
    std::vector<SphereProfileObservation> observations;
    std::int64_t id=1;
    int selector=0;
    for(double b:bValues) {
        for(double c:cValues) {
            if ((selector++ % 2)!=0 && std::abs(b)>1e-9 && std::abs(c)>1e-9) continue;
            const double y=((selector%2)==0?+1.0:-1.0)*0.40*(diameter*0.5);
            const Vec3d target(xTargets[selector%3],y,zTargets[(selector/3)%3]);
            const JointPose5d q=poseForTarget(b,c,target,sphereCenter,truth);
            observations.push_back(makeProfile(id++,q,target,diameter,25));
        }
    }
    return observations;
}

void testNoiselessCalibration()
{
    const double diameter=19.05;
    const Gauge14Parameters truth=truthParameters();
    const Vec3d sphereCenter(385.0,272.0,98.0);
    const auto observations=makeDiverseData(truth,sphereCenter,diameter);
    require(observations.size()>=25,"diverse profiles generated");

    Gauge14Parameters initial=truth;
    initial[Gauge14Parameters::Sxy]+=30e-6;
    initial[Gauge14Parameters::Sxz]-=25e-6;
    initial[Gauge14Parameters::Syz]+=20e-6;
    initial[Gauge14Parameters::COffsetU]+=2.0;
    initial[Gauge14Parameters::COffsetV]-=1.5;
    initial[Gauge14Parameters::BOffsetU]-=2.5;
    initial[Gauge14Parameters::BOffsetV]+=2.0;
    initial[Gauge14Parameters::CTiltX]+=25e-6;
    initial[Gauge14Parameters::CTiltY]-=20e-6;
    initial[Gauge14Parameters::BTiltX]+=30e-6;
    initial[Gauge14Parameters::BTiltZ]-=25e-6;
    initial[Gauge14Parameters::HandEyeRx]+=3e-3;
    initial[Gauge14Parameters::HandEyeRy]-=2e-3;
    initial[Gauge14Parameters::HandEyeRz]+=2e-3;
    Vec3d centerInitial=add(sphereCenter,Vec3d(2,-2,1));

    SolverOptions options;
    options.sphereDiameterMm=diameter;
    options.multiStartCount=1;
    options.alternatingIterations=8;
    options.blockIterations=20;
    options.jointIterations=100;
    options.huberDeltaMm=0.01;
    SingleSphere14Calibrator calibrator(options);
    for(const auto& obs:observations) calibrator.addObservation(obs);
    const SingleSphere14Result result=calibrator.solve(initial,&centerInitial);
    require(result.success,"noiseless 14-D solve success: "+result.message);
    require(result.observability.targetRank==14,"14-D target rank");
    require(result.residualRmsMm<1e-7,"noiseless residual");

    for(std::size_t i=0;i<Gauge14Parameters::Count;++i) {
        const double tolerance=(i==Gauge14Parameters::COffsetU ||
                                i==Gauge14Parameters::COffsetV ||
                                i==Gauge14Parameters::BOffsetU ||
                                i==Gauge14Parameters::BOffsetV) ? 2e-4 : 3e-7;
        require(std::abs(result.parameters[i]-truth[i])<tolerance,
                std::string("parameter recovery: ")+Gauge14Parameters::names()[i]);
    }
}

void testDegeneratePosesRejected()
{
    const double diameter=19.05;
    const Gauge14Parameters truth=truthParameters();
    const Vec3d sphereCenter(385.0,272.0,98.0);
    SingleSphere14Calibrator calibrator;
    for(int i=0;i<12;++i) {
        const Vec3d target(-4.0+0.7*i,0.4*(diameter*0.5),90.0+0.5*i);
        const JointPose5d q=poseForTarget(0.0,0.0,target,sphereCenter,truth);
        calibrator.addObservation(makeProfile(i+1,q,target,diameter,21));
    }
    const ObservabilityReport report=calibrator.analyzeObservability(truth,sphereCenter);
    require(report.targetRank<14,"degenerate B=C=0 data is rank deficient");
}

void testRecenteringStep()
{
    const Gauge14Parameters truth=truthParameters();
    const Vec3d sphereCenter(385.0,272.0,98.0);
    const Vec3d target(0.0,0.40*(19.05*0.5),95.0);
    JointPose5d desired=poseForTarget(30.0,-40.0,target,sphereCenter,truth);
    JointPose5d current=desired;
    current.x+=0.8;
    current.y-=0.5;
    current.z+=0.6;
    const Vec3d measured=SingleSphere14Model::sphereCenterInSensor(
        current,sphereCenter,truth);
    JointPose5d suggested;
    Vec3d predicted;
    std::string error;
    require(SingleSphere14PosePlanner::recommendXYZRecentering(
                current,measured,target,truth,2.0,suggested,predicted,error),
            "recentering step: "+error);
    require(std::abs(suggested.b-current.b)<1e-12 &&
            std::abs(suggested.c-current.c)<1e-12,
            "recentering keeps B/C unchanged");
    require(norm(sub(predicted,target))<norm(sub(measured,target))*0.02,
            "recentering strongly reduces center error");
}

void testPlanner()
{
    Gauge14Parameters truth=truthParameters();
    const Vec3d sphereCenter(385.0,272.0,98.0);
    const Vec3d target(0.0,0.4*(19.05*0.5),95.0);
    PosePlannerSeed seed;
    seed.taughtJoint=poseForTarget(0.0,0.0,target,sphereCenter,truth);
    seed.taughtSphereCenterSensorMm=target;
    PosePlannerConfig config;
    config.requestedRecordedPoses=24;
    SingleSphere14PosePlanner planner(config);
    std::string error;
    const auto poses=planner.generate(seed,truth,error);
    require(error.empty(),"planner error: "+error);
    require(poses.size()==24,"planner pose count");
    require(std::abs(poses.front().predictedJoint.b-seed.taughtJoint.b)<1e-9 &&
            std::abs(poses.front().predictedJoint.c-seed.taughtJoint.c)<1e-9,
            "planner starts from the taught zero-rotary reference");
    bool hasB=false,hasC=false,hasCoupled=false;
    for(const auto& pose:poses) {
        hasB=hasB || std::abs(pose.predictedJoint.b)>10;
        hasC=hasC || std::abs(pose.predictedJoint.c)>10;
        hasCoupled=hasCoupled || (std::abs(pose.predictedJoint.b)>10 && std::abs(pose.predictedJoint.c)>10);
        require(pose.targetSphereCenterSensorMm.z>=config.sensorZMinMm,"planner z lower");
        require(pose.targetSphereCenterSensorMm.z<=config.sensorZMaxMm,"planner z upper");
    }
    require(hasB && hasC && hasCoupled,"planner excites both rotary axes");
}

} // namespace

int main()
{
    testSphereCenterFit();
    testSphereCenterRansacRejectsBackground();
    testNoiselessCalibration();
    testDegeneratePosesRejected();
    testRecenteringStep();
    testPlanner();
    std::cout << "v20 single-sphere 14-D calibration tests passed\n";
    return 0;
}
