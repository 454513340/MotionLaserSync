#pragma once
// Non-registration metric diagnostic: fixed-MCS nearest neighbours between
// real exposure-supported points and synthetic CAD hits. Never fit alignment.
#include "twin/TwinTypes.h"
#include <cstddef>
#include <string>
#include <vector>
namespace mls::twin {
struct MultiPoseCloudMetrics {
    bool valid=false;
    std::size_t realCount=0,syntheticCount=0;
    double realToSimMedianMm=0,realToSimP95Mm=0;
    double simToRealMedianMm=0,simToRealP95Mm=0;
    double realWithinTolerance=0,simWithinTolerance=0;
    double toleranceMm=0.;
    std::string error;
    std::vector<float> realDistancesMm; // for per-point heatmap, same order as bounded input
};
MultiPoseCloudMetrics compareMultiPoseClouds(const std::vector<Vec3>& real,
     const std::vector<Vec3>& synthetic,double toleranceMm=0.25,
     std::size_t maxPointsEach=120000);
}
