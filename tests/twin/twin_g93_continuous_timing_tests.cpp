#include "runtime/FiveAxis.h"
#include "twin/MultiPoseExperiment.h"
#include "twin/TwinM6NcPreview.h"
#include "twin/MultiPoseNcSimulation.h"
#include "twin/NcG93MotionAudit.h"
#include <iostream>
#include <cmath>
#define CHECK(expr) do { if(!(expr)) { std::cerr<<"FAIL line "<<__LINE__<<": "<<#expr<<"\n";return 1; } } while(false)
int main() {
    using namespace mls;
    const runtime::Limits limits{};
    std::vector<runtime::Waypoint> requested;
    for(int i=1;i<=100;i++)
        requested.push_back({{double(i),0,0,0,0},300,
                            runtime::FeedMode::InverseMinute,true});
    runtime::Plan impossible;std::string why;
    // The original 100 * 200ms scan cannot start and stop at 5mm/s.
    CHECK(!runtime::plan({},requested,limits,25,25,impossible,why));
    std::vector<double> duration;
    CHECK(runtime::retimeG93Polyline({},requested,limits,25,25,duration,why));
    CHECK(duration.size()==100);
    CHECK(duration.front()>0.40 && duration.front()<0.70);
    CHECK(duration[1]>0.28 && duration[1]<0.40);
    CHECK(duration.back()>0.29 && duration.back()<0.35);
    CHECK(duration[50]>0.20 && duration[50]<0.22);
    std::vector<runtime::Waypoint> feasible=requested;
    for(std::size_t i=0;i<feasible.size();++i) {
        feasible[i].feed=60/duration[i];
        feasible[i].continuous=(i>0); // same rule as live executor and audit
    }
    runtime::Plan planned;
    CHECK(runtime::plan({},feasible,limits,25,25,planned,why));
    CHECK(planned.segments.size()==100);
    CHECK(planned.segments[0].entry==0);
    CHECK(planned.segments[0].exit==0); // live-start anchor, no ideal-start blend
    CHECK(planned.segments[1].entry==0);
    CHECK(planned.segments[1].exit>4.0);
    CHECK(planned.segments[50].entry>4.0);
    CHECK(planned.segments[99].exit==0);
    for(std::size_t i=0;i<planned.segments.size();++i)
        CHECK(std::abs(planned.segments[i].seconds-duration[i])<1e-7);
    // V4 regression: a tiny live post-G00/B-C MPOS residual breaks the
    // extremely strict collinearity predicate (1e-7). Previously the NC
    // allocated only ~0.305s for a start which now requires an exact stop.
    std::vector<runtime::Waypoint> v4;
    for(int i=1;i<=10;++i)
        v4.push_back({{double(i),0,0,0,0},
            i==1 ? 60.0/0.305404 : 60.0/0.210404,
            runtime::FeedMode::InverseMinute,true});
    runtime::Joint driftedStart{0.005,-0.003,0.001,0.004,-0.002};
    runtime::Plan driftPlan;
    CHECK(!runtime::plan(driftedStart,v4,limits,25,25,driftPlan,why));
    // V5 remains feasible for starts near the NC reference, INCLUDING
    // independently measured rotary MPOS and the full 0.05mm/0.05deg
    // allowed convergence envelope. Continuous speed is restored *after*
    // the first exact-stop anchor.
    for(unsigned mask=0;mask<32;++mask){
        runtime::Joint live{};
        for(int axis=0;axis<5;++axis)
            live[axis]=(mask&(1u<<axis))?0.05:-0.05;
        CHECK(runtime::plan(live,feasible,limits,25,25,driftPlan,why));
        CHECK(driftPlan.segments.size()==feasible.size());
        CHECK(driftPlan.segments.front().exit==0);
        CHECK(driftPlan.segments[1].exit>4.0);
    }
    // An actual 90-degree corner must retain a zero junction speed.
    requested.clear();
    requested.push_back({{1,0,0,0,0},300,runtime::FeedMode::InverseMinute,true});
    requested.push_back({{1,1,0,0,0},300,runtime::FeedMode::InverseMinute,true});
    CHECK(runtime::retimeG93Polyline({},requested,limits,25,25,duration,why));
    CHECK(duration[0]>.35 && duration[1]>.35);
    for(std::size_t i=0;i<requested.size();++i){
        requested[i].feed=60/duration[i];
        requested[i].continuous=(i>0);
    }
    CHECK(runtime::plan({},requested,limits,25,25,planned,why));
    CHECK(planned.segments[0].exit==0 && planned.segments[1].entry==0);
    // The actual multi-pose NC exporter must inherit the new physical time,
    // not independently reintroduce distance/speed G93 values.
    twin::ExperimentViewReport view;
    view.id="P01";
    view.path.push_back({0,{0,0,0,0,0},{},twin::M6SegmentRole::Transfer});
    for(int i=1;i<=50;++i)
        view.path.push_back({0.2*i,{double(i),0,0,0,0},{},twin::M6SegmentRole::Scan});
    twin::ExperimentOptions recipe;
    recipe.enforceHardwareG93=true;
    recipe.profileRateHz=100;
    auto report=twin::multiPoseAsOfflineM6(view,recipe);
    CHECK(report.ok());
    CHECK(report.path.back().seconds>10 && report.path.back().seconds<13);
    twin::M6NcPreviewOptions preview;
    preview.proposedTopologyId=3;preview.h3R1Edge=114;
    const auto saved=twin::makeM6NcPreview(report,preview);
    CHECK(saved.ok());
    const auto replay=twin::simulateMultiPoseNc(saved.text);
    CHECK(replay.ok());
    CHECK(twin::verifyMultiPoseNcAgainstPlan(replay,report).empty());
    CHECK(twin::auditNcG93Motion(replay,limits,25,25).passed);
    // Two connected scan poses must preserve the physical timeline across
    // S1 -> M51 -> S0 transfer -> M50 -> S1, including pose playback indexes.
    twin::ExperimentPlan experiment;
    auto buildView=[](const char* name,double start) {
        twin::ExperimentViewReport v;v.id=name;
        v.path.push_back({0,{start,0,0,0,0},{},twin::M6SegmentRole::Transfer});
        for(int i=1;i<=20;++i)
            v.path.push_back({0.2*i,{start+i,0,0,0,0},{},twin::M6SegmentRole::Scan});
        return v;
    };
    experiment.views={buildView("P01",0),buildView("P02",40)};
    const auto forward=[](const twin::Joint5& q) {
        twin::Transform t;t.m[3]=q[0];t.m[7]=q[1];t.m[11]=q[2];return t;
    };
    const auto allow=[](const twin::Joint5&,std::string*){return true;};
    recipe.transferSpeedMmPerSec=5.;
    const auto session=twin::planConnectedExperimentSession(experiment,recipe,forward,allow);
    if(!session.ok())std::cerr<<"connected timing: "<<session.error<<"\n";
    CHECK(session.ok());
    CHECK(session.merged.g93PhysicalTimelinePlanned);
    CHECK(session.poseStartSeconds.size()==2 && session.poseEndSeconds.size()==2);
    CHECK(session.poseStartSeconds[1]>session.poseEndSeconds[0]);
    CHECK(session.poseEndSeconds[1]>session.poseStartSeconds[1]);
    const auto joined=twin::makeM6NcPreview(session.merged,preview);
    CHECK(joined.ok());
    const auto joinedRun=twin::simulateMultiPoseNc(joined.text);
    CHECK(joinedRun.ok());
    CHECK(joinedRun.scanGroups==2 && joinedRun.saveGroups==2);
    CHECK(twin::auditNcG93Motion(joinedRun,limits,25,25).passed);
    CHECK(twin::verifyMultiPoseNcAgainstPlan(joinedRun,session.merged).empty());
    std::cout<<"Continuous G93 physical timeline, corner stop, generator and NC replay PASS\n";
}
