#include "twin/TwinM6NcPreview.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>

namespace mls::twin {
namespace {
bool finiteJoint(const Joint5& q){for(double v:q)if(!std::isfinite(v))return false;return true;}
bool equalJoint(const Joint5& a,const Joint5& b){
    for(int j=0;j<5;++j)if(std::abs(a[j]-b[j])>1e-10)return false;
    return true;
}
void xyzcb(std::ostringstream& os,const Joint5& q){
    static constexpr char axes[]="XYZCB";
    for(int j=0;j<5;++j)os << ' ' << axes[j] << std::setprecision(12) << q[j];
}
std::string safeScanLabel(const std::string& s){
    if(s.empty())return "simulated_strip";
    if(s.size()>40)return {};
    for(const unsigned char c:s){
        if(!((c>='a'&&c<='z')||(c>='A'&&c<='Z')||
             (c>='0'&&c<='9')||c=='_'||c=='-'))return {};
    }
    return s;
}
std::string cleanComment(std::string s){
    for(char& c:s)if(c=='\r'||c=='\n'||c==';'||c=='('||c==')')c='_';
    return s;
}
}
M6NcPreviewResult makeM6NcPreview(const M6Report& r,const M6NcPreviewOptions& opt){
    M6NcPreviewResult result;
    auto reject=[&](const std::string& reason){result.error=reason;return result;};
    if(!r.ok())return reject("M6 report has an error or is cancelled");
    const std::string scanLabel=safeScanLabel(opt.scanGroupLabel);
    if(scanLabel.empty())return reject("Invalid M60 scan group label");
    if(r.canExportForHardware)return reject("Unexpected hardware authorization flag in an offline M6 report");
    if(r.path.size()<2)return reject("M6 path must contain at least two timed joints");
    if(opt.proposedTopologyId!=1 && opt.proposedTopologyId!=3)
        return reject("Explicit H1(1) or H3(3) required; H2(2) was retired");
    if(opt.proposedTopologyId==3 && opt.h3R1Edge!=114 && opt.h3R1Edge!=115)
        return reject("H3 only permits commissioned R1 latch edge 114 or 115");
    if(!std::isfinite(r.options.scanRateHz)||r.options.scanRateHz<1.0||r.options.scanRateHz>2000.0 ||
       std::abs(r.options.scanRateHz-std::round(r.options.scanRateHz))>1e-9)
        return reject("Invalid/non-integer M6 nominal profile frequency for V32.16 NC metadata");
    // The legacy M6 planner may have no explicitly anchored initial machine
    // joint. Keep the trace inspectable, but mark its first G00 as an UNKNOWN
    // approach. Never imply this is a collision-certified start move.
    if(r.options.hasInitialJoints&&!finiteJoint(r.options.initialJoints))
        return reject("M6 recorded initial joint pose is invalid");
    for(std::size_t i=0;i<r.path.size();++i){
        const auto& p=r.path[i];
        if(!std::isfinite(p.seconds)||p.seconds<0||!finiteJoint(p.joints))
            return reject("Invalid time or XYZCB joint coordinate in M6 path");
        if(i && p.seconds+1e-9<r.path[i-1].seconds)
            return reject("M6 timeline moves backwards");
        for(int j=0;j<5;++j)
            if(p.joints[j]<r.options.bounds.min[j]-1e-7||p.joints[j]>r.options.bounds.max[j]+1e-7)
                return reject("M6 joint is outside the recorded offline planning bounds");
        if(i && p.seconds-r.path[i-1].seconds<1e-8 && !equalJoint(p.joints,r.path[i-1].joints))
            return reject("M6 contains a nonzero five-axis jump with zero elapsed time");
    }
    std::ostringstream nc; nc.imbue(std::locale::classic());
    nc << "; Program: M6 offline five-axis trajectory - NOT FOR EXECUTION\n"
       << "; OfflineOnly: true\n"
       << "; SimulationVerified: false\n"
       << "; SyncMode: Hardware\n"
       << "; HardwareTopology: " << (opt.proposedTopologyId==1?"H1":"H3") << "\n"
       << "; HardwareFrequencyHz: " << std::setprecision(12) << r.options.scanRateHz << "\n"
       << "; SensorTriggerMode: " << (opt.proposedTopologyId==1?"HardwareExternal":"SoftwareInternal") << "\n"
       << "; ProgramFrame: MACHINE_JOINTS_XYZCB\n"
       << "; CoordinateOrder: X Y Z C B\n"
       << "; LinearUnit: mm\n; RotaryUnit: deg\n"
       << "; PartPoseId: " << cleanComment(opt.sourcePartPoseId.empty()?"UNVERIFIED":opt.sourcePartPoseId) << "\n"
       << "; CalibrationId: " << cleanComment(opt.calibrationId.empty()?"UNVERIFIED":opt.calibrationId) << "\n";
    if (!opt.physical17Sha256.empty())
        nc << "; Physical17Sha256: " << cleanComment(opt.physical17Sha256) << "\n";
    nc << "; SourceCad: " << cleanComment(opt.sourceCadPath) << "\n"
       << "; KinematicsSource: " << cleanComment(opt.kinematicsSource) << "\n"
       << "; ExperimentStudyId: " << cleanComment(opt.studyId) << "\n"
       << "; RegistrationVerified: false\n"
       << "; InitialJointAnchored: " << (r.options.hasInitialJoints?"true":"false") << "\n"
       << "; InitialJointSource: " << (opt.studyId=="MULTIPOSE_ORIENTATION_STUDY"
           ?"HISTORICAL_SCAN_ARCHIVE_NOT_LIVE_MPOS":"UNVERIFIED_PLANNING_REFERENCE") << "\n"
       << "; Warning: First G00 is a position reference; approach from LIVE MPOS is NOT planned/verified\n"
       << "; Warning: Scan edges have M50 ON; M51 OFF is emitted before any lane transfer. No Z retract is commanded by MultiPose\n"
       << "; Warning: M6 timestamps become nominal G93 segment times; controller jerk/accel planning may alter exposure positions\n"
       << "; Warning: 2950 multi-angle optics / full housing and fixture collisions are NOT certified\n";
    if(r.g93PhysicalTimelinePlanned)
        nc << "; G93PhysicalTiming: SP_CONTINUOUS_WITH_LIVE_START_ANCHOR_V2\n"
           << "; G93PhysicalTimingScope: controller-software-planner-only; actual ECI SP timing unverified\n";
    if(opt.proposedTopologyId==1)nc << "; HardwareTriggerType: EdgeRising\n"; // legacy H1 external edge only
    if(opt.proposedTopologyId==3)nc << "; CmmR1Edge: " << opt.h3R1Edge << "\n";
    nc << "G21\nG90\nG93\n";
    // Initial unexposed positioning is a preview, NOT a proven safe travel.
    // A physical program requires a separate collision-certified initial move.
    nc << "; Initial machine state must be inspected before any real movement\n";
    nc << "G00";xyzcb(nc,r.path.front().joints);nc << "\n";

    bool laser=false, groupOpen=false;
    const auto fileLabel=[&](){return scanLabel + (opt.scanGroupLabel.empty()?std::string{}:"_"+std::to_string(result.scanningGroups));};
    auto closeGroup=[&](){if(groupOpen){groupOpen=false;nc << "; end M6 G05 segment\n";}};
    for(std::size_t i=1;i<r.path.size();++i){
        const auto& before=r.path[i-1];const auto& now=r.path[i];
        const bool scan=now.role==M6SegmentRole::Scan;
        const double dt=now.seconds-before.seconds;
        if(dt<1e-8){
            // The first point of a new scan can duplicate the end of the
            // already-generated transfer. It adds no motion and is not emitted.
            if(!equalJoint(before.joints,now.joints))return reject("Invalid zero-time segment");
            continue;
        }
        const double inverseMinute=60.0/dt;
        if(!std::isfinite(inverseMinute)||inverseMinute<=0)return reject("Invalid G93 feed");
        if(scan!=laser){
            closeGroup();
            if(laser){nc << "M51\nM60 L\"" << fileLabel() << "\"\nM61\n";}
            if(scan){nc << "M50\n";}
            laser=scan;
        }
        if(!groupOpen){
            nc << "; M6 " << (scan?"SCAN":"TRANSFER") << " — offline sampled XYZCB polyline\n";
            nc << "G05 S" << (scan?1:0) << " F" << std::setprecision(12) << inverseMinute << "\n";
            groupOpen=true;
            if(scan)++result.scanningGroups;else ++result.transferGroups;
        }
        xyzcb(nc,now.joints);nc << " F" << std::setprecision(12) << inverseMinute << "\n";
        ++result.encodedMotionPoints;
    }
    closeGroup();
    if(laser)nc << "M51\nM60 L\"" << fileLabel() << "\"\nM61\n";
    if(!result.scanningGroups)return reject("M6 has no scan segments to export");
    nc << "M63\nM30\n";
    result.text=nc.str();return result;
}
} // namespace mls::twin
