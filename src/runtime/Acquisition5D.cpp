#include "runtime/Acquisition5D.h"
namespace mls::runtime {
bool completeFrame(Frame5D& f, const std::array<double,17>& p, std::string& e) {
    if(!finiteJoint(f.joints) || !finiteJoint(f.command) || !finiteJoint(f.raw)) {e="nonfinite joint pose";return false;}
    if((f.feedbackMask|f.latchMask|f.commandMask)>31 || (f.feedbackMask&f.commandMask)!=0 ||
       (f.feedbackMask|f.commandMask)!=Table5D::allAxesMask) {e="invalid five-axis provenance masks";return false;}
    if((f.timing==PoseTiming::HardwareLatched || f.timing==PoseTiming::HardwareTemporalInterpolated) &&
       f.latchMask!=Table5D::allAxesMask) {e="hardware-derived frame does not contain all XYZCB latch bits";return false;}
    if(f.timing==PoseTiming::HardwareTemporalInterpolated) {
        const bool forward = f.temporalOffsetSec >= 0.0 &&
            f.temporalAlpha >= 0.0 && f.temporalAlpha < 1.0 &&
            f.ordinal >= 1 && f.temporalNextOrdinal == f.ordinal+1;
        const bool backward = f.temporalOffsetSec < 0.0 &&
            f.temporalAlpha < 0.0 && f.temporalAlpha > -1.0 &&
            f.ordinal >= 2 && f.temporalNextOrdinal >= 1 &&
            f.temporalNextOrdinal+1 == f.ordinal;
        if(!std::isfinite(f.temporalOffsetSec) || !std::isfinite(f.temporalAlpha) ||
           !(forward || backward)) {
            e="invalid signed H1/H3 hardware temporal interpolation provenance"; return false;
        }
    } else if(f.temporalOffsetSec!=0.0 || f.temporalAlpha!=0.0 || f.temporalNextOrdinal!=0) {
        e="temporal metadata present on a non-temporal frame"; return false;
    }
    if(f.timing==PoseTiming::SoftwareInterpolated && f.latchMask!=0) {e="software-interpolated frame must not claim hardware latch bits";return false;}
    for(double v:p) if(!std::isfinite(v)){e="nonfinite calibration parameter";return false;}
    calibration::Physical17Parameters model; model.value=p;
    calibration::JointPose5d q; q.x=f.joints[0];q.y=f.joints[1];q.z=f.joints[2];q.c=f.joints[3];q.b=f.joints[4];
    f.sensorToMachine=calibration::SingleSphere17Model::sensorToMachineTransform(q,model);
    f.physicalParameters=p;
    if(!validTransform(f.sensorToMachine)){e="invalid calibrated sensor-to-machine transform";return false;}
    return true;
}
bool decodeRecord(const float* v, std::uint64_t expected, std::uint64_t session,
                  const std::array<double,2>& offsets, std::uint32_t feedbackMask,
                  std::uint32_t latchMask, std::uint32_t commandMask, PoseTiming timing,
                  Frame5D& f, std::string& e) {
    if(!v || expected>Table5D::maxOrdinal || !Table5D::integer(v[0],0,Table5D::maxOrdinal) ||
       static_cast<std::uint64_t>(v[0])!=expected){e="TABLE trigger ordinal discontinuity";return false;}
    for(int k=1;k<Table5D::dimension;++k) if(!std::isfinite(v[k])){e="nonfinite TABLE latched joint";return false;}
    for(double o:offsets) if(!std::isfinite(o)){e="nonfinite rotary frame offset";return false;}
    if((feedbackMask|latchMask|commandMask)>31 || (feedbackMask&commandMask)!=0 ||
       (feedbackMask|commandMask)!=Table5D::allAxesMask){e="invalid TABLE pose provenance";return false;}
    if(timing==PoseTiming::HardwareLatched && latchMask!=Table5D::allAxesMask){e="incomplete hardware latch mask";return false;}
    f=Frame5D{}; f.session=session;f.ordinal=expected;f.rotaryOffsets=offsets;
    f.raw={v[1],v[2],v[3],v[4],v[5]}; f.joints=f.raw;
    f.joints[3]+=offsets[0];f.joints[4]+=offsets[1];
    // command[] is retained for archive compatibility.  commandMask is the
    // authoritative indication of which coordinates originate from controller
    // command position rather than encoder feedback.
    f.command=f.joints;
    f.feedbackMask=feedbackMask;f.latchMask=latchMask;f.commandMask=commandMask;
    f.timing=timing;
    return true;
}
}
