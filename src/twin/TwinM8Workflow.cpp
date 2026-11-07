#include "twin/TwinM8Workflow.h"
#include <algorithm>
#include <cmath>

namespace mls::twin {
const char* m8StageName(M8Stage s) noexcept {
    switch(s){
    case M8Stage::VirtualM3: return "M3 Virtual scan";
    case M8Stage::ComparisonM4: return "M4 Real/virtual comparison";
    case M8Stage::CoverageM5: return "M5 CAD occupancy";
    case M8Stage::ChamferM51: return "M5.1 Center chamfer density";
    case M8Stage::GeometryM52: return "M5.2 Geometry consistency";
    case M8Stage::ContinuousM6: return "M6 Continuous five-axis simulation";
    case M8Stage::FusionM7: return "M7 Provenance-aware acceptance";
    }
    return "Unknown";
}
M8WorkflowPlan makeM8Plan(const M8WorkflowOptions& o){
    M8WorkflowPlan p;
    if(!std::isfinite(o.coverageGoal)||o.coverageGoal<=0.||o.coverageGoal>1.){
        p.error="M8 coverage goal must be finite and within (0,1]";
        return p;
    }
    if(o.simulateM3) p.stages.push_back(M8Stage::VirtualM3);
    if(o.compareM4) p.stages.push_back(M8Stage::ComparisonM4);
    p.stages.push_back(M8Stage::CoverageM5);
    if(o.inspectChamferM51) p.stages.push_back(M8Stage::ChamferM51);
    if(o.inspectGeometryM52) p.stages.push_back(M8Stage::GeometryM52);
    p.stages.push_back(M8Stage::ContinuousM6);
    p.stages.push_back(M8Stage::FusionM7);
    return p;
}
M8Preflight inspectM8Preflight(const M8WorkflowPlan& p,
                               const M8WorkflowOptions& o,
                               const M8InputState& input){
    M8Preflight check;
    if(!p.ok())check.blockers.push_back(p.error.empty()?"Empty workflow":p.error);
    if(!input.cadPlaced)check.blockers.push_back("M0/M1 CAD workpiece was not placed");
    if(!input.realM2Loaded)check.blockers.push_back("M2 real scan was not loaded");
    if(input.m2SourceIsSimulated)check.blockers.push_back("M2 source is simulated, not a real measurement");
    if(input.otherTaskBusy)check.blockers.push_back("Another twin background job is still active");
    if(o.compareM4 && !o.simulateM3 && !input.hasM3)
        check.blockers.push_back("M4 requires an existing M3 simulation or selecting M3");
    if((o.inspectChamferM51||o.inspectGeometryM52)&&!input.h1SpecificGeometry)
        check.blockers.push_back("M5.1/5.2 are H1-specific: disable them on other STEP models");
    check.warnings.push_back("M6 multi-angle optical model has not been independently validated");
    check.warnings.push_back("M5.2 proposed CAD offsets are diagnostic-only and are NEVER applied to machine pose");
    check.warnings.push_back("M8 is an OFFLINE predicted coverage assessment, not hardware authorization");
    check.permitted=check.blockers.empty();
    return check;
}
M8Acceptance evaluateM8Acceptance(const M5Report& m5,const M6Report& m6,
                                   const M7Coverage& m7,double goal){
    M8Acceptance out;out.targetFraction=goal;
    if(!std::isfinite(goal)||goal<=0.||goal>1.){
        out.error="Invalid M8 coverage goal";return out;
    }
    if(!m5.ok()||!m6.ok()||!m7.ok()||!m7.areaConsistent){
        out.error="M5/M6/M7 data missing, inconsistent or incomplete";return out;
    }
    const double tol=std::max(0.05,1e-4*m7.targetAreaMm2);
    if(std::abs(m5.targetAreaMm2-m7.targetAreaMm2)>tol ||
       std::abs(m6.targetAreaMm2-m7.targetAreaMm2)>tol ||
       std::abs(m6.newAreaMm2-m7.simulatedNewAreaMm2)>tol ||
       std::abs(m6.finalObservedAreaMm2-(m7.measuredAreaMm2+m7.simulatedNewAreaMm2))>tol){
        out.error="CAD area or simulated covering-cell provenance does not match M5/M6/M7";
        return out;
    }
    if(m6.fullProfiles==0||m6.segments.empty()||m6.firstCoveredSegmentByCell.size()!=m5.cells.size()){
        out.error="M6 has no valid continuous-scan exposure data or mismatched CAD-cell index";return out;
    }
    out.targetAreaMm2=m7.targetAreaMm2;
    out.realAreaMm2=m7.measuredAreaMm2;
    out.simulatedNewAreaMm2=m7.simulatedNewAreaMm2;
    out.stillMissingAreaMm2=m7.remainingAreaMm2;
    out.predictedFraction=m7.finalFraction();
    out.predictedTargetReached=out.predictedFraction+1e-10>=goal;
    out.requiresAdditionalView=!out.predictedTargetReached;
    out.coverageBookkeepingValid=true;
    // Never infer true multi-view calibration from one H1 fixed-B/C recording.
    out.opticalMultiAngleValidated=false;
    out.fullFixtureCollisionCertified=false;
    out.hardwareAuthorized=false;
    for(const auto& f:m7.features){
        if(f.targetAreaMm2<=0)continue;
        if(f.stillMissingAreaMm2<=0)continue;
        M8FeaturePriority priority;
        priority.id=f.featureId;
        priority.feature=f.name;
        priority.targetAreaMm2=f.targetAreaMm2;
        priority.stillMissingAreaMm2=f.stillMissingAreaMm2;
        priority.simulatedNewAreaMm2=f.simulatedNewAreaMm2;
        priority.fractionMissing=f.stillMissingAreaMm2/f.targetAreaMm2;
        out.priorities.push_back(std::move(priority));
    }
    std::sort(out.priorities.begin(),out.priorities.end(),
      [](const M8FeaturePriority& a,const M8FeaturePriority& b){
        if(a.stillMissingAreaMm2!=b.stillMissingAreaMm2)
            return a.stillMissingAreaMm2>b.stillMissingAreaMm2;
        return a.id<b.id;
      });
    return out;
}
} // namespace mls::twin
