#pragma once
// Offline-only deterministic replay over EXACT saved XYZCB NC path samples.
// Independent orientations are separate sessions, NEVER interpolated together.
// No controller, sensor, Qt, or motor access.
#include "twin/MultiPoseExperiment.h"
#include "twin/MultiPoseNcSimulation.h"
#include <cstddef>
#include <string>
#include <vector>

namespace mls::twin {
struct MultiPosePlaybackSegment {
    std::size_t poseIndex=0;
    double globalStart=0.,globalEnd=0.,localEnd=0.;
};
struct MultiPosePlaybackTimeline {
    std::vector<MultiPosePlaybackSegment> segments;
    double totalSeconds=0.;
    std::string error;
    bool ok() const {return error.empty()&&!segments.empty();}
};
struct MultiPosePlaybackState {
    bool valid=false;
    bool scanning=false;
    std::size_t poseIndex=0;
    double globalSeconds=0.,localSeconds=0.;
    bool betweenPoses=false; // laser-OFF NC connector to the next independent scan
    std::size_t previousPoseIndex=0;
    Joint5 joints{};
    Transform sensorToMachine{};
    std::size_t pathIndex=0;
    std::size_t exposureCount=0;
};
// pauseSeconds is only a visual session separator: no machine motion is implied.
MultiPosePlaybackTimeline makeMultiPosePlaybackTimeline(const ExperimentPlan& plan,
                                                       double pauseSeconds=0.35);
// Interpolate JOINTS, then recalculate full calibrated sensor pose using FK.
// Never interpolate rotation matrices, never infer a link between independent poses.
// Time axis reconstructed from the PARSED NC, not the planner path.
MultiPosePlaybackTimeline makeNcDrivenPlaybackTimeline(const std::vector<NcSimResult>& runs,
                                                     double pauseSeconds=0.35);
MultiPosePlaybackState sampleNcDrivenPlayback(const std::vector<NcSimResult>& runs,
    const MultiPosePlaybackTimeline& timeline,double globalSeconds,
    const M6ForwardKinematics& forward);

// Timeline and sampling for ONE explicitly connected and re-parsed NC file.
// The time between consecutive scan sessions is *real parsed G05 S0 time*,
// never a cosmetic pause or interpolation of two separate G00 starts.
MultiPosePlaybackTimeline makeConnectedNcPlaybackTimeline(
    const ConnectedExperimentSession& connected,const NcSimResult& nc);
MultiPosePlaybackState sampleConnectedNcPlayback(const NcSimResult& nc,
    const MultiPosePlaybackTimeline& timeline,double globalSeconds,
    const M6ForwardKinematics& forward);

MultiPosePlaybackState sampleMultiPosePlayback(const ExperimentPlan& plan,
     const MultiPosePlaybackTimeline& timeline,double globalSeconds,
     const M6ForwardKinematics& forward);
}
