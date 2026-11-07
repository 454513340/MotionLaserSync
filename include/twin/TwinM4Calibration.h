#pragma once
// V28.3 M4: opt-in SINGLE-PARAMETER equivalent-camera-baseline search.
// A lone H1 sweep cannot identify separate emitter and camera 3D centres.
#include "twin/TwinM4Comparison.h"
#include <atomic>
#include <functional>

namespace mls::twin {
struct M4CalibrationFrame {
    VirtualProfilePose pose;
    std::vector<M4MeasuredSample> observed;
};
struct M4FitOptions {
    double minSensorY=35.0,maxSensorY=95.0,stepSensorY=10.0;
    std::size_t maxFrames=180;
    bool useEstimatedPose=false; // true in physical M0-M5 runs; default preserves synthetic tests
};
struct M4FitCandidate {
    double cameraSensorY=0;
    M4Report training,validation;
    double trainingScore=-1,validationScore=-1;
};
struct M4FitResult {
    std::vector<M4FitCandidate> candidates;
    std::size_t selectedFrames=0,trainingFrames=0,validationFrames=0;
    double bestBaselineY=0;
    double bestHoldoutBaselineY=0;
    bool holdoutSupportsBest=false;
    std::string validationNote;
    bool cancelled=false;
    std::string error;
    bool ok() const {return error.empty()&&!cancelled&&!candidates.empty();}
};

class TwinM4Calibration final {
public:
    // A deterministic ROI-only split, 4:1 training/validation by frame index.
    // No random train/validation leakage, no hardware calls.
    static M4FitResult fit(const TwinScene& scene,
                           const Sensor2950Geometry& sourceModel,
                           const std::vector<M4CalibrationFrame>& frames,
                           const M4FitOptions& options,
                           const std::atomic<bool>* cancel=nullptr,
                           const std::function<void(std::size_t,std::size_t)>& progress={});
};
} // namespace mls::twin
