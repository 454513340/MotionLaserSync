#include "twin/TwinM4Calibration.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace mls::twin {
namespace {
bool insideReferenceHole(const M4CalibrationFrame& frame,
                        const Transform& machineToCad,const M4Context& ctx){
    for(std::size_t i=0;i<frame.observed.size();i+=8){
        const auto& s=frame.observed[i];
        const Vec3 p=machineToCad.point(frame.pose.sensorToMachine.point({s.sensorX,0,s.sensorZ}));
        if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z) ||
           p.z<0 || p.z>ctx.cadTopZ+1.0)continue;
        for(const auto& h:ctx.holes)
            if(std::hypot(p.x-h.x,p.y-h.y)<=h.radiusMm+1.0)return true;
    }
    return false;
}
}
M4FitResult TwinM4Calibration::fit(const TwinScene& scene,
        const Sensor2950Geometry& sourceModel,
        const std::vector<M4CalibrationFrame>& frames,
        const M4FitOptions& options,
        const std::atomic<bool>* cancel,
        const std::function<void(std::size_t,std::size_t)>& progress){
    M4FitResult result;
    if(frames.empty()||options.maxFrames<10||
       !std::isfinite(options.minSensorY)||!std::isfinite(options.maxSensorY)||
       !std::isfinite(options.stepSensorY)||options.minSensorY>options.maxSensorY||
       options.stepSensorY<=0 || options.minSensorY < -100||options.maxSensorY>100){
        result.error="Invalid baseline range or M4 frame cap";return result;
    }
    const double span=options.maxSensorY-options.minSensorY;
    const std::size_t count=std::size_t(std::floor(span/options.stepSensorY+1e-7))+1;
    if(count<2||count>41){result.error="M4 baseline candidate count outside [2,41]";return result;}
    const M4Context ctx=h1ReferenceM4Context(scene,sourceModel,options.useEstimatedPose);
    if(ctx.holes.size()!=4){result.error="M4 H1 counterbore ROI does not match this CAD geometry";return result;}
    const Transform machineToCad=ctx.cadToMachine.inverseRigid();
    std::vector<std::size_t> useful;
    for(std::size_t i=0;i<frames.size();++i){
        if(cancel&&cancel->load()){result.cancelled=true;return result;}
        if(insideReferenceHole(frames[i],machineToCad,ctx))useful.push_back(i);
    }
    // Fail rather than claim a fit based solely on featureless plate pixels.
    if(useful.size()<10){result.error="Insufficient real counterbore frames for M4 baseline fitting";return result;}
    std::vector<std::size_t> selected;
    const std::size_t limit=std::min(options.maxFrames,useful.size());
    selected.reserve(limit);
    for(std::size_t k=0;k<limit;++k)selected.push_back(useful[k*useful.size()/limit]);
    result.selectedFrames=selected.size();
    for(std::size_t k=0;k<selected.size();++k)
        if(k%5==4)++result.validationFrames;
        else ++result.trainingFrames;
    if(result.validationFrames<2||result.trainingFrames<8){
        result.error="Insufficient training/holdout split";return result;
    }
    double best=-std::numeric_limits<double>::infinity();
    for(std::size_t ci=0;ci<count;++ci){
        if(cancel&&cancel->load()){result.cancelled=true;return result;}
        M4FitCandidate candidate;
        candidate.cameraSensorY=options.minSensorY+double(ci)*options.stepSensorY;
        Sensor2950Geometry model=sourceModel;
        model.cameraOriginSensor.y=candidate.cameraSensorY;
        std::string reason;
        if(!model.validate(&reason)){result.error=reason;return result;}
        VirtualLaser2950 laser(scene,model,options.useEstimatedPose);
        if(!laser.ready(&reason)){result.error=reason;return result;}
        M4Context sampleCtx=ctx;sampleCtx.optical=model;
        TwinM4Accumulator train(sampleCtx),validation(sampleCtx);
        for(std::size_t k=0;k<selected.size();++k){
            if(cancel&&cancel->load()){result.cancelled=true;return result;}
            const auto& input=frames[selected[k]];
            auto profile=laser.simulate(input.pose);
            if(k%5==4){
                if(!validation.addFrame(profile,input.observed)){
                    candidate.validation=std::move(validation).finish();
                    result.error=candidate.validation.error;return result;
                }
            }else if(!train.addFrame(profile,input.observed)){
                candidate.training=std::move(train).finish();
                result.error=candidate.training.error;return result;
            }
            if(progress)progress(ci*selected.size()+k+1,count*selected.size());
        }
        candidate.training=std::move(train).finish();
        candidate.validation=std::move(validation).finish();
        candidate.trainingScore=candidate.training.fitScore();
        candidate.validationScore=candidate.validation.fitScore();
        if(candidate.trainingScore>best){best=candidate.trainingScore;result.bestBaselineY=candidate.cameraSensorY;}
        result.candidates.push_back(std::move(candidate));
    }
    if(best<0){result.error="Counterbore score cannot be computed from selected frames";return result;}
    double topHoldout=-std::numeric_limits<double>::infinity();
    double selectedHoldout=-std::numeric_limits<double>::infinity();
    for(const auto& c:result.candidates){
        if(c.validationScore>topHoldout){
            topHoldout=c.validationScore;result.bestHoldoutBaselineY=c.cameraSensorY;
        }
        if(std::abs(c.cameraSensorY-result.bestBaselineY)<1e-8)
            selectedHoldout=c.validationScore;
    }
    // HOLDOUT is only a same-sweep frame subset, NOT new optical calibration
    // evidence. Refuse automatic apply if it contradicts training or is too small.
    result.holdoutSupportsBest=(result.validationFrames>=10 &&
        topHoldout>=0 && selectedHoldout>=topHoldout-0.05);
    if(result.validationFrames<10)
        result.validationNote="Too few independent held-out frames; do not auto-apply fit";
    else if(!result.holdoutSupportsBest)
        result.validationNote="Holdout prefers another baseline; do not auto-apply fit";
    else result.validationNote="Same-sweep holdout is consistent; still not physical optical calibration";
    return result;
}
} // namespace mls::twin
