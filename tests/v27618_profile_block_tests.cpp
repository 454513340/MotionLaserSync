// White-box numerical equivalence tests.  The implementation is deliberately
// included only in this portable test executable, never twice in a target.
#define main original_physical17_regression_main
#include "v20_2_single_sphere17_tests.cpp"
#undef main
#include "../src/calibration/SingleSphere17Calibration.cpp"
#include <limits>
#include <random>

namespace fast_tests {
using namespace mls::calibration;
std::vector<double> referenceResiduals(const std::vector<SphereProfileObservation>& obs,
    const std::array<double,20>& state,double radius)
{
    Physical17Parameters p; Vec3d c; unpackState(state,p,c);
    std::vector<double> r;
    for(const auto& o:obs) for(const auto& point:o.points) {
        const auto m=SingleSphere17Model::mapSensorPoint(o.joints,{point.x,0,point.z},p);
        r.push_back(norm(sub(m,c))-radius);
    }
    return r;
}
void equivalence()
{
    auto p=truthParameters();const Vec3d c{245,-105,165};
    auto obs=makeObservations(p,c);
    p[0]+=45e-6;p[3]+=0.08;p[11]+=0.0015;
    for(std::size_t i=0;i<obs.size();++i) for(std::size_t j=0;j<obs[i].points.size();++j) {
        obs[i].points[j].weight=(j%11==0)?0.0:((j%3==0)?0.4:1.0);
        obs[i].points[j].z+=0.003*std::sin(double(i*7+j));
    }
    Physical17SolverOptions options;
    const auto state=packState(p,c);
    const auto active=fullIndices();
    const auto r=evaluateResiduals(obs,state,9.525);
    const auto rr=referenceResiduals(obs,state,9.525);
    double maxResidualError=0.0;
    for(std::size_t i=0;i<r.size();++i) maxResidualError=std::max(maxResidualError,std::abs(r[i]-rr[i]));
    require(maxResidualError<5e-11,"rigid sensor-center residual must match original machine residual");
    const auto jac=numericalJacobian(obs,state,options.parameterScales,9.525,1e-5,active);
    double maxJacError=0.0;
    for(int col=0;col<20;++col) {
        auto plus=state;
        auto minus=state;
        const double h=1e-5*options.parameterScales[col];plus[col]+=h;minus[col]-=h;
        const auto rp=referenceResiduals(obs,plus,9.525);
        const auto rm=referenceResiduals(obs,minus,9.525);
        double largest=0.0,error=0.0;
        for(int row=0;row<jac.rows;++row) {
            const double v=(rp[row]-rm[row])/(2e-5);
            largest=std::max(largest,std::abs(v));error=std::max(error,std::abs(v-jac(row,col)));
        }
        maxJacError=std::max(maxJacError,error);
        require(error<2e-8+2e-6*largest,"hybrid normalized Jacobian column vs original central differences");
    }
    const auto normal=profileNormalEquations(obs,state,options,active);
    DenseMatrix h(20,20);std::vector<double> g(20,0.0);
    const auto weights=basePointWeights(obs);
    for(int row=0;row<jac.rows;++row) {
        const double w=weights[row]*huberIrlsWeight(r[row],options.huberDeltaMm);
        for(int i=0;i<20;++i) {
            g[i]+=w*jac(row,i)*r[row];
            for(int j=0;j<20;++j) h(i,j)+=w*jac(row,i)*jac(row,j);
        }
    }
    double hError=0,hScale=0,gError=0,gScale=0;
    for(std::size_t i=0;i<h.data.size();++i){hError=std::max(hError,std::abs(h.data[i]-normal.h.data[i]));hScale=std::max(hScale,std::abs(h.data[i]));}
    for(int i=0;i<20;++i){gError=std::max(gError,std::abs(g[i]-normal.gradient[i]));gScale=std::max(gScale,std::abs(g[i]));}
    require(hError/std::max(1.0,hScale)<1e-11,"3x3 factored information equals explicit JTWJ");
    require(gError/std::max(1.0,gScale)<1e-11,"3x3 factored gradient equals explicit JTWr");
    // Verify the objective derivative, including its Huber and base weights.
    for(int col=0;col<20;++col) {
        auto plus=state;
        auto minus=state;const double hn=1e-6;
        plus[col]+=hn*options.parameterScales[col];minus[col]-=hn*options.parameterScales[col];
        const double cp=totalRobustCost(evaluateResiduals(obs,plus,9.525),weights,options.huberDeltaMm);
        const double cm=totalRobustCost(evaluateResiduals(obs,minus,9.525),weights,options.huberDeltaMm);
        const double fd=(cp-cm)/(2*hn);
        require(std::abs(fd-g[col])<2e-6+2e-4*std::abs(fd),"Huber objective derivative");
    }
    std::cout<<"equivalence: residual_max="<<maxResidualError<<", Jacobian_max="<<maxJacError
             <<", relative_H="<<hError/hScale<<", relative_g="<<gError/gScale<<'\n';
}
void finiteInputsAndCancellation()
{
    const auto p=truthParameters();const Vec3d c{245,-105,165};const auto obs=makeObservations(p,c);
    auto invalid=obs;invalid[0].points[0].z=std::numeric_limits<double>::quiet_NaN();
    SingleSphere17Calibrator bad;for(const auto& o:invalid)bad.addObservation(o);
    require(!bad.solve(p,&c).success,"nonfinite point rejected");
    require(bad.analyzeObservability(p,c).targetRank==0,"invalid observability rejected");
    auto zero=obs;for(auto& o:zero)for(auto& x:o.points)x.weight=0;
    SingleSphere17Calibrator z;for(const auto& o:zero)z.addObservation(o);
    require(!z.solve(p,&c).success,"all-zero weights rejected");
    Physical17SolverOptions options;options.parameterScales[4]=0;
    SingleSphere17Calibrator invalidOptions(options);for(const auto& o:obs)invalidOptions.addObservation(o);
    require(!invalidOptions.solve(p,&c).success,"zero scale rejected");
    SingleSphere17Calibrator cal;for(const auto& o:obs)cal.addObservation(o);
    std::atomic_bool cancelled(false);
    auto progress=[&](const Physical17SolveProgress& update) {
        if(update.phase==Physical17SolveProgress::Phase::AlternatingMachine)cancelled.store(true);
    };
    auto initial=p;initial[3]+=0.1;
    const auto result=cal.solve(initial,&c,progress,&cancelled);
    require(!result.success && result.message.find("cancelled")!=std::string::npos,"mid-solve cancellation");
    const auto normal=profileNormalEquations(obs,packState(p,c),Physical17SolverOptions{},fullIndices(),&cancelled);
    require(normal.cancelled,"profile-level cancellation");
    const auto invalidRank=cal.analyzeObservability(p,{std::numeric_limits<double>::infinity(),0,0});
    require(invalidRank.targetRank==0,"infinite center rejected");
    std::vector<SphereProfileObservation> decoded;std::string error;
    std::istringstream csv("profile_id,x_mm,y_mm,z_mm,c_deg,b_deg,sensor_x_mm,sensor_z_mm,weight\n1,0,0,0,0,0,0,95,-1\n");
    require(!SingleSphere17Calibrator::readObservationCsv(csv,decoded,error),"negative CSV weight rejected");
    SingleSphere17Result failed;
    failed.residualRmsMm=std::numeric_limits<double>::infinity();
    failed.parameters[0]=std::numeric_limits<double>::quiet_NaN();
    std::ostringstream failureJson;
    require(SingleSphere17Calibrator::writeResultJson(failureJson,failed,error),"failed result can be serialized");
    require(failureJson.str().find("\"residual_rms_mm\": null")!=std::string::npos,
            "nonfinite failed diagnostics use JSON null");
    failed.success=true;
    std::ostringstream invalidSuccess;
    require(!SingleSphere17Calibrator::writeResultJson(invalidSuccess,failed,error),
            "successful nonfinite result cannot be serialized");
}
PosePlannerConfig config()
{
    PosePlannerConfig cfg;cfg.bMinDeg=-30;cfg.bMaxDeg=30;cfg.bStepDeg=15;
    cfg.cMinDeg=-45;cfg.cMaxDeg=45;cfg.cStepDeg=15;
    cfg.sensorXHalfWidthMm=100;cfg.sensorZMinMm=0;cfg.sensorZMaxMm=200;
    cfg.requestedRecordedPoses=12;cfg.maxRecordedPosesPerBCPair=3;
    cfg.reorderRecordedPosesForTravel=false;cfg.requireRequestedPoseCount=true;
    return cfg;
}
void plannerConstraints()
{
    const auto p=truthParameters();const Vec3d c{245,-105,165};
    PosePlannerSeed seed;seed.hasSphereCenterMachine=true;seed.sphereCenterMachineMm=c;
    seed.taughtJoint=poseForTarget(0,0,{0,0,95},c,p);
    // This stale sensor center must not reject planning with a known machine center.
    seed.taughtSphereCenterSensorMm={0,25,95};
    auto cfg=config();std::string error;
    Physical17PlanningChecks checks;
    checks.poseAllowed=[](const JointPose5d& q,const Vec3d&){return q.c>=0;};
    checks.transitionAllowed=[](const JointPose5d& a,const JointPose5d& b){return std::abs(a.b-b.b)<=30;};
    const auto plan=SingleSphere17PosePlanner(cfg).generate(seed,p,error,checks);
    require(plan.size()==12,"known center can plan from nonintersecting/stale reference");
    JointPose5d cursor=seed.taughtJoint;
    for(const auto& pose:plan) {
        require(pose.predictedJoint.c>=0,"hard target filter honored during selection");
        require(std::abs(cursor.b-pose.predictedJoint.b)<=30,"all executed-order transitions checked");
        const auto center=SingleSphere17Model::machinePointInSensor(pose.predictedJoint,c,p);
        require(maxAbs(center,pose.targetSphereCenterSensorMm)<1e-8,"exact inverse/forward target consistency");
        cursor=pose.predictedJoint;
    }
    cfg.requestedRecordedPoses=18;
    const auto larger=SingleSphere17PosePlanner(cfg).generate(seed,p,error,checks);
    require(larger.size()==18,"larger feasible plan exists");
    for(std::size_t i=0;i<plan.size();++i) {
        require(plan[i].predictedJoint.b==larger[i].predictedJoint.b && plan[i].predictedJoint.c==larger[i].predictedJoint.c &&
                maxAbs(plan[i].targetSphereCenterSensorMm,larger[i].targetSphereCenterSensorMm)<1e-12,"D-optimal prefix not changed by reserve size/travel sorting");
    }
    cfg.bStepDeg=std::numeric_limits<double>::quiet_NaN();
    require(SingleSphere17PosePlanner(cfg).generate(seed,p,error).empty()&&!error.empty(),"NaN range rejected");
    cfg=config();cfg.bMinDeg=cfg.bMaxDeg=cfg.cMinDeg=cfg.cMaxDeg=0;cfg.maxRecordedPosesPerBCPair=1;
    require(SingleSphere17PosePlanner(cfg).generate(seed,p,error).empty()&&!error.empty(),"insufficient feasible count explicitly rejected");

    // V27.6.20: offline Stage-2 planning has a known machine-space sphere center
    // but deliberately has no truthful current joint pose.  The planner must not
    // reject that mode or fabricate a transition from zero/NaN to the first pose.
    cfg=config();
    PosePlannerSeed offlineSeed=seed;
    offlineSeed.hasTaughtJoint=false;
    offlineSeed.taughtJoint.x=std::numeric_limits<double>::quiet_NaN();
    bool sawNonfiniteTransition=false;
    Physical17PlanningChecks offlineChecks;
    offlineChecks.poseAllowed=[](const JointPose5d& q,const Vec3d&){return q.c>=0;};
    offlineChecks.transitionAllowed=[&](const JointPose5d& a,const JointPose5d& b){
        const bool finite=std::isfinite(a.x)&&std::isfinite(a.y)&&std::isfinite(a.z)&&std::isfinite(a.c)&&std::isfinite(a.b)&&
                          std::isfinite(b.x)&&std::isfinite(b.y)&&std::isfinite(b.z)&&std::isfinite(b.c)&&std::isfinite(b.b);
        if(!finite)sawNonfiniteTransition=true;
        return finite && std::abs(a.b-b.b)<=30;
    };
    const auto offlinePlan=SingleSphere17PosePlanner(cfg).generate(offlineSeed,p,error,offlineChecks);
    require(offlinePlan.size()==12,"offline known-center plan should not require a live taught pose");
    require(!sawNonfiniteTransition,"offline planner must not use fake start pose in transition gate");
    for(std::size_t i=1;i<offlinePlan.size();++i)
        require(std::abs(offlinePlan[i-1].predictedJoint.b-offlinePlan[i].predictedJoint.b)<=30,
                "offline plan internal transitions remain checked");

    // V27.6.23: a dense GUI grid may have >200k theoretical combinations even
    // though XYZ/FOV/static-safety filtering leaves only a small useful pool.
    // The raw Cartesian-product size must therefore not reject the plan before
    // those real feasibility gates run.  These values reproduce the reported
    // 266760 conservative combinations from D=86..104/4, gamma=0..40/5,
    // eta=-5.5..5.5/1.5, B=-30..30/5, C=-45..45/5, both plane sides.
    auto denseCfg=config();
    denseCfg.useQualityStudyGeometry=true;
    denseCfg.qualityDMinMm=86;denseCfg.qualityDMaxMm=104;denseCfg.qualityDStepMm=4;
    denseCfg.qualityGammaMinDeg=0;denseCfg.qualityGammaMaxDeg=40;denseCfg.qualityGammaStepDeg=5;
    denseCfg.qualityEtaMinDeg=-5.5;denseCfg.qualityEtaMaxDeg=5.5;denseCfg.qualityEtaStepDeg=1.5;
    denseCfg.bMinDeg=-30;denseCfg.bMaxDeg=30;denseCfg.bStepDeg=5;
    denseCfg.cMinDeg=-45;denseCfg.cMaxDeg=45;denseCfg.cStepDeg=5;
    denseCfg.requestedRecordedPoses=8;denseCfg.maxRecordedPosesPerBCPair=20;
    PosePlannerSeed denseSeed=seed;
    denseSeed.hasSphereCenterMachine=true;denseSeed.sphereCenterMachineMm=c;
    denseSeed.hasTaughtJoint=false;
    Physical17PlanningChecks denseChecks;
    denseChecks.poseAllowed=[](const JointPose5d& q,const Vec3d& target){
        return std::abs(q.b)<1e-12 && std::abs(q.c)<1e-12 && std::abs(target.x)<1e-10;
    };
    const auto densePlan=SingleSphere17PosePlanner(denseCfg).generate(denseSeed,p,error,denseChecks);
    require(densePlan.size()==8,"dense raw grid must be pruned by feasibility instead of rejected at 200k");

    // V27.6.25: integrated dense planning splits the medium-cost shortlist
    // gate from exact target/transition safety.  Exact MachineSafety-equivalent
    // callbacks must be lazy rather than O(raw-grid).
    std::size_t shortlistChecks=0,exactPoseChecks=0,transitionChecks=0;
    Physical17PlanningChecks hierarchicalChecks;
    hierarchicalChecks.shortlistAllowed=[&](const JointPose5d&,const Vec3d&){
        ++shortlistChecks; return true;
    };
    hierarchicalChecks.poseAllowed=[&](const JointPose5d&,const Vec3d&){
        ++exactPoseChecks; return true;
    };
    hierarchicalChecks.transitionAllowed=[&](const JointPose5d&,const JointPose5d&){
        ++transitionChecks; return true;
    };
    const auto hierarchicalPlan=SingleSphere17PosePlanner(denseCfg).generate(
        denseSeed,p,error,hierarchicalChecks);
    require(hierarchicalPlan.size()==8,"hierarchical dense planner returns requested pose count");
    require(shortlistChecks<=32768,"dense optical/detail gate must be bounded by shortlist reservoir");
    require(exactPoseChecks<=32,"exact target safety must be lazy, not raw-grid sized");
    require(transitionChecks<=32,"exact transition safety must be lazy, not raw-grid sized");

    // V27.6.24: planner progress/cancellation is the contract used by the Qt
    // worker-thread UI.  It must report the expensive filtering/selection
    // phases without changing the generated design, and cancellation must stop
    // inside the candidate loop rather than waiting for the entire grid.
    auto progressCfg=config();
    progressCfg.requestedRecordedPoses=8;
    PosePlannerSeed progressSeed=seed;
    bool sawFiltering=false,sawShortlist=false,sawSelection=false,sawFinalizing=false;
    auto progressCallback=[&](const Physical17PlanningProgress& update) {
        if(update.phase==Physical17PlanningPhase::FilteringCandidates)sawFiltering=true;
        if(update.phase==Physical17PlanningPhase::EvaluatingShortlist)sawShortlist=true;
        if(update.phase==Physical17PlanningPhase::SelectingDOptimal)sawSelection=true;
        if(update.phase==Physical17PlanningPhase::Finalizing)sawFinalizing=true;
    };
    const auto progressPlan=SingleSphere17PosePlanner(progressCfg).generate(
        progressSeed,p,error,Physical17PlanningChecks{},progressCallback,nullptr);
    require(progressPlan.size()==8 && sawFiltering && sawShortlist && sawSelection && sawFinalizing,
            "planner emits worker-thread progress through all hierarchical phases");

    std::atomic_bool cancelPlanning(false);
    auto cancelProgress=[&](const Physical17PlanningProgress& update) {
        if(update.phase==Physical17PlanningPhase::FilteringCandidates && update.current>0)
            cancelPlanning.store(true);
    };
    error.clear();
    const auto cancelledPlan=SingleSphere17PosePlanner(progressCfg).generate(
        progressSeed,p,error,Physical17PlanningChecks{},cancelProgress,&cancelPlanning);
    require(cancelledPlan.empty() && error.find("cancelled")!=std::string::npos,
            "planner cancellation interrupts candidate generation");

    cfg=config();seed.hasSphereCenterMachine=false;
    require(SingleSphere17PosePlanner(cfg).generate(seed,p,error).empty(),"unknown center still requires intersecting teach profile");
}
void upperArcAndGauge()
{
    Physical17Parameters p;p[16]=-200;PosePlannerConfig cfg=config();
    cfg.useEndEffectorFacingArc=true;cfg.useTrapezoidalSensorField=true;cfg.sensorZCoordinateFlipped=true;
    cfg.sensorZMinMm=70;cfg.sensorZMaxMm=120;cfg.sensorXHalfWidthMm=29;
    cfg.sensorXSafetyMarginMm=cfg.sensorZSafetyMarginMm=1;
    const Vec3d centerS{0,0,95};const auto arc=syntheticAcquisitionProfile({},centerS,9.525,p,cfg);
    require(arc.points.size()>=7,"upper visible arc has points");double sum=0;
    for(const auto& point:arc.points) {
        const Vec3d v{point.x,0,point.z};sum+=point.z;
        require(dot(sub(v,centerS),sub(Vec3d{0,0,200},centerS))>0,"information contour faces actual end effector");
        require(std::abs(norm(sub(v,centerS))-9.525)<1e-10,"synthetic upper contour lies on sphere");
        const double half=0.5*(42+(120-point.z)/50*16)-1;
        require(std::abs(point.x)<=half+1e-10,"FlipZ physical trapezoid containment");
    }
    require(sum/arc.points.size()>95,"information no longer assumes lower-Z arc");
    const auto truth=truthParameters();const Vec3d centerM{245,-105,165};
    const auto obs=makeObservations(truth,centerM);SingleSphere17Calibrator cal;
    for(const auto& o:obs)cal.addObservation(o);
    const auto report=cal.analyzeObservability(truth,centerM);
    require(report.targetRank==14 && report.fullStateRank==17,"analytically projected gauge ranks");
    for(int i=14;i<17;++i)require(report.targetSingularValues[i]==0,"physical gauge singular values exactly zero");
    for(int i=17;i<20;++i)require(report.fullStateSingularValues[i]==0,"full-state gauge singular values exactly zero");
    SingleSphere17Calibrator repeated;
    for(int i=0;i<12;++i)repeated.addObservation(obs.front());
    require(repeated.analyzeObservability(truth,centerM).targetRank<14,"projection must not fake rank for repeated poses");
}
}
int main()
{
    fast_tests::equivalence();
    fast_tests::finiteInputsAndCancellation();
    fast_tests::plannerConstraints();
    fast_tests::upperArcAndGauge();
    std::cout<<"PASS V27.6.18 profile-block equivalence, invalid data, cancellation, gauge, pose gates and upper arc\n";
}
