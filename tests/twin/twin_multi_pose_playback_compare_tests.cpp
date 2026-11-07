#include "twin/MultiPosePlayback.h"
#include "twin/MultiPoseCloudComparison.h"
#include <cmath>
#include <iostream>
#include <limits>
using namespace mls::twin;
#define CHECK(c) do{if(!(c)){std::cerr<<"FAILED "<<__LINE__<<" : "<<#c<<"\n";return 1;}}while(false)
static Transform fakeFk(const Joint5& q){
    Transform r;r.m[3]=q[0];r.m[7]=q[1];r.m[11]=q[2];
    return r;
}
static M6PathSample sample(double t,double x,M6SegmentRole role){
    M6PathSample p;p.seconds=t;p.joints={{x,5,10,0,0}};
    p.sensorToMachine=fakeFk(p.joints);p.role=role;
    return p;
}
int main(){
    ExperimentPlan plan;
    ExperimentViewReport a;a.id="P01";
    a.path={sample(0,0,M6SegmentRole::Transfer),
            sample(1,10,M6SegmentRole::Transfer),
            sample(2,20,M6SegmentRole::Scan),
            sample(3,30,M6SegmentRole::Scan),
            sample(4,35,M6SegmentRole::Transfer)};
    a.exposures.resize(4);
    a.exposures[0].elapsedSeconds=1.5;
    a.exposures[1].elapsedSeconds=2.0;
    a.exposures[2].elapsedSeconds=2.5;
    a.exposures[3].elapsedSeconds=3.0;
    ExperimentViewReport b;b.id="P02";
    b.path={sample(0,90,M6SegmentRole::Transfer),sample(1,100,M6SegmentRole::Scan)};
    plan.views={a,b};
    auto timeline=makeMultiPosePlaybackTimeline(plan,.5);
    CHECK(timeline.ok());
    CHECK(std::abs(timeline.totalSeconds-5.5)<1.e-8);
    const auto fk=[](const Joint5& q){return fakeFk(q);};
    auto state=sampleMultiPosePlayback(plan,timeline,.5,fk);
    CHECK(state.valid&&!state.scanning&&state.poseIndex==0&&std::abs(state.joints[0]-5)<1.e-8);
    state=sampleMultiPosePlayback(plan,timeline,1.5,fk);
    CHECK(state.scanning&&state.poseIndex==0&&std::abs(state.joints[0]-15)<1.e-8);
    CHECK(state.exposureCount==1);
    state=sampleMultiPosePlayback(plan,timeline,3.5,fk);
    CHECK(!state.scanning);
    state=sampleMultiPosePlayback(plan,timeline,4.25,fk);
    CHECK(state.valid&&!state.scanning); // gap must not be laser-active
    state=sampleMultiPosePlayback(plan,timeline,4.75,fk);
    CHECK(state.poseIndex==1&&state.scanning&&std::abs(state.joints[0]-92.5)<1.e-8);
    CHECK(!sampleMultiPosePlayback(plan,timeline,std::numeric_limits<double>::quiet_NaN(),fk).valid);
    auto bad=plan;bad.views[0].path[2].seconds=-1;
    CHECK(!makeMultiPosePlaybackTimeline(bad).ok());
    std::vector<Vec3> real,virtualCloud;
    for(int i=0;i<1000;++i){
        const double x=double(i)*.01;
        real.push_back({x,0,0});
        virtualCloud.push_back({x,0,.02});
    }
    const auto good=compareMultiPoseClouds(real,virtualCloud,.1);
    CHECK(good.valid&&good.realCount==1000&&good.syntheticCount==1000);
    CHECK(good.realToSimMedianMm>.019&&good.realToSimMedianMm<.021);
    CHECK(good.realToSimP95Mm<.021&&good.realWithinTolerance>.999);
    CHECK(good.simToRealMedianMm>.019&&good.simToRealMedianMm<.021);
    for(auto& p:virtualCloud)p.z=1.;
    const auto badMatch=compareMultiPoseClouds(real,virtualCloud,.1);
    CHECK(badMatch.valid&&badMatch.realWithinTolerance==0&&badMatch.simWithinTolerance==0);
    CHECK(!compareMultiPoseClouds({},virtualCloud).valid);
    CHECK(!compareMultiPoseClouds(real,virtualCloud,-1.).valid);
    std::cout<<"Portable multi-pose replay: independent sessions, XYZCB interpolation, laser-off and geometry compare PASS\n";
}
