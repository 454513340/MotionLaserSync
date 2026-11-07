#include "calibration/SingleSphere17Calibration.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace mls::calibration;

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;
constexpr double kRadius = 19.05 / 2.0;

void require(bool condition, const std::string& message)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(1);
    }
}

double maxAbs(const Vec3d& a, const Vec3d& b)
{
    return std::max({std::abs(a.x-b.x), std::abs(a.y-b.y), std::abs(a.z-b.z)});
}

Vec3d addv(const Vec3d& a, const Vec3d& b) { return {a.x+b.x,a.y+b.y,a.z+b.z}; }
Vec3d subv(const Vec3d& a, const Vec3d& b) { return {a.x-b.x,a.y-b.y,a.z-b.z}; }
Vec3d mulv(const Vec3d& a,double s) { return {a.x*s,a.y*s,a.z*s}; }

double dotv(const Vec3d& a,const Vec3d& b) { return a.x*b.x+a.y*b.y+a.z*b.z; }
Vec3d crossv(const Vec3d& a,const Vec3d& b) { return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x}; }
Vec3d normv(const Vec3d& a) { const double n=std::sqrt(dotv(a,a)); return mulv(a,1.0/n); }

Vec3d rotateAxis(const Vec3d& p, Vec3d axis, double angle)
{
    axis=normv(axis);
    return addv(addv(mulv(p,std::cos(angle)), mulv(crossv(axis,p),std::sin(angle))),
                mulv(axis,dotv(axis,p)*(1.0-std::cos(angle))));
}

Vec3d rotateVector(const Vec3d& p, const Vec3d& r)
{
    const double a=std::sqrt(dotv(r,r));
    if(a<1e-14) return p;
    return rotateAxis(p,mulv(r,1.0/a),a);
}

bool solve3(const double A[3][3], const Vec3d& b, Vec3d& x)
{
    double m[3][4]={{A[0][0],A[0][1],A[0][2],b.x},
                    {A[1][0],A[1][1],A[1][2],b.y},
                    {A[2][0],A[2][1],A[2][2],b.z}};
    for(int k=0;k<3;++k){
        int piv=k;
        for(int r=k+1;r<3;++r) if(std::abs(m[r][k])>std::abs(m[piv][k])) piv=r;
        if(std::abs(m[piv][k])<1e-14) return false;
        if(piv!=k) for(int c=k;c<4;++c) std::swap(m[piv][c],m[k][c]);
        for(int r=k+1;r<3;++r){
            double f=m[r][k]/m[k][k];
            for(int c=k;c<4;++c)m[r][c]-=f*m[k][c];
        }
    }
    double v[3]{};
    for(int r=2;r>=0;--r){
        double s=m[r][3]; for(int c=r+1;c<3;++c)s-=m[r][c]*v[c];
        v[r]=s/m[r][r];
    }
    x={v[0],v[1],v[2]}; return true;
}

Physical17Parameters truthParameters()
{
    Physical17Parameters p;
    p[Physical17Parameters::Sxy]=+120e-6;
    p[Physical17Parameters::Sxz]=-90e-6;
    p[Physical17Parameters::Syz]=+70e-6;
    p[Physical17Parameters::COffsetU]=+0.45;
    p[Physical17Parameters::COffsetV]=-0.32;
    p[Physical17Parameters::CTiltX]=+180e-6;
    p[Physical17Parameters::CTiltY]=-130e-6;
    p[Physical17Parameters::BOffsetU]=-0.38;
    p[Physical17Parameters::BOffsetV]=+0.27;
    p[Physical17Parameters::BTiltX]=+160e-6;
    p[Physical17Parameters::BTiltZ]=-110e-6;
    p[Physical17Parameters::HandEyeRx]=+0.012;
    p[Physical17Parameters::HandEyeRy]=-0.018;
    p[Physical17Parameters::HandEyeRz]=+0.007;
    p[Physical17Parameters::HandEyeTx]=+32.0;
    p[Physical17Parameters::HandEyeTy]=-18.0;
    p[Physical17Parameters::HandEyeTz]=+54.0;
    return p;
}

JointPose5d poseForTarget(double b, double c, const Vec3d& centerS,
                          const Vec3d& centerM, const Physical17Parameters& p)
{
    JointPose5d q0; q0.b=b; q0.c=c;
    const Vec3d base=SingleSphere17Model::mapSensorPoint(q0,centerS,p);
    JointPose5d qx=q0,qy=q0,qz=q0; qx.x=1; qy.y=1; qz.z=1;
    const Vec3d ux=subv(SingleSphere17Model::mapSensorPoint(qx,centerS,p),base);
    const Vec3d uy=subv(SingleSphere17Model::mapSensorPoint(qy,centerS,p),base);
    const Vec3d uz=subv(SingleSphere17Model::mapSensorPoint(qz,centerS,p),base);
    const double A[3][3]={{ux.x,uy.x,uz.x},{ux.y,uy.y,uz.y},{ux.z,uy.z,uz.z}};
    Vec3d xyz; require(solve3(A,subv(centerM,base),xyz),"XYZ solve");
    JointPose5d q=q0; q.x=xyz.x; q.y=xyz.y; q.z=xyz.z; return q;
}

std::vector<SphereProfileObservation> makeObservations(const Physical17Parameters& truth,
                                                        const Vec3d& centerM)
{
    const double bs[]={-48,-32,-16,0,16,32,48};
    const double cs[]={-66,-44,-22,0,22,44,66};
    const double xs[]={-4.5,0,4.5};
    const double ys[]={3.2,-3.2,5.0,-5.0};
    const double zs[]={90,97,104};
    std::vector<SphereProfileObservation> out;
    for(int k=0;k<42;++k){
        const double b=bs[k%7];
        const double c=cs[(3*k+k/7)%7];
        const Vec3d centerS(xs[k%3],ys[(k/3)%4],zs[(k/5)%3]);
        SphereProfileObservation obs;
        obs.profileId=k+1;
        obs.joints=poseForTarget(b,c,centerS,centerM,truth);
        const double rho=std::sqrt(kRadius*kRadius-centerS.y*centerS.y);
        for(int i=0;i<21;++i){
            const double dx=(-0.88+1.76*static_cast<double>(i)/20.0)*rho;
            SensorProfilePoint sp;
            sp.x=centerS.x+dx;
            sp.z=centerS.z-std::sqrt(std::max(0.0,rho*rho-dx*dx));
            obs.points.push_back(sp);
            const Vec3d pm=SingleSphere17Model::mapSensorPoint(obs.joints,{sp.x,0.0,sp.z},truth);
            require(std::abs(std::sqrt(dotv(subv(pm,centerM),subv(pm,centerM)))-kRadius)<2e-10,
                    "synthetic point must lie on sphere");
        }
        out.push_back(obs);
    }
    return out;
}

void testNominalConsistency()
{
    Physical17Parameters p;
    p[Physical17Parameters::HandEyeRx]=0.014;
    p[Physical17Parameters::HandEyeRy]=-0.009;
    p[Physical17Parameters::HandEyeRz]=0.006;
    p[Physical17Parameters::HandEyeTx]=31.0;
    p[Physical17Parameters::HandEyeTy]=-17.0;
    p[Physical17Parameters::HandEyeTz]=52.0;
    JointPose5d q; q.x=112.3;q.y=-47.2;q.z=23.5;q.c=37.0;q.b=-28.0;
    Vec3d s{4.2,-1.1,96.0};
    Vec3d e=rotateVector(s,{p[11],p[12],p[13]});
    e=addv(e,{p[14],p[15],p[16]});
    e=rotateAxis(e,{0,1,0},-q.b*kDeg);
    e=rotateAxis(e,{0,0,1},-q.c*kDeg);
    e=addv(e,{q.x,q.y,q.z});
    const Vec3d actual=SingleSphere17Model::mapSensorPoint(q,s,p);
    require(maxAbs(actual,e)<1e-10,
            "zero-error physical17 must equal Trans(XYZ)*Rz(-C)*Ry(-B)*T_ES");

    const auto he=SingleSphere17Model::handEyeTransform(p);
    require(std::abs(he[3]-p[Physical17Parameters::HandEyeTx])<1e-14 &&
            std::abs(he[7]-p[Physical17Parameters::HandEyeTy])<1e-14 &&
            std::abs(he[11]-p[Physical17Parameters::HandEyeTz])<1e-14,
            "handEyeTransform must retain full hand-eye translation");
    const Vec3d eFromMatrix{
        he[0]*s.x+he[1]*s.y+he[2]*s.z+he[3],
        he[4]*s.x+he[5]*s.y+he[6]*s.z+he[7],
        he[8]*s.x+he[9]*s.y+he[10]*s.z+he[11]};
    const Vec3d eExpected=addv(rotateVector(s,{p[11],p[12],p[13]}),{p[14],p[15],p[16]});
    require(maxAbs(eFromMatrix,eExpected)<1e-12,
            "handEyeTransform must match the hand-eye part of mapSensorPoint");

    const Vec3d recovered=SingleSphere17Model::machinePointInSensor(q,actual,p);
    require(maxAbs(recovered,s)<2e-10,
            "sensorToMachineTransform inverse must recover the original sensor point");
}


std::vector<SphereProfileObservation> observationsFromPlan(
    const std::vector<CalibrationPoseInstruction>& plan,
    const Physical17Parameters& truth,
    const Vec3d& centerM)
{
    std::vector<SphereProfileObservation> out;
    int profileId = 1;
    for (const auto& instruction : plan) {
        SphereProfileObservation obs;
        obs.profileId = profileId++;
        obs.joints = instruction.predictedJoint;
        const Vec3d centerS = instruction.targetSphereCenterSensorMm;
        const double rho2 = kRadius*kRadius - centerS.y*centerS.y;
        require(rho2 > 0.0, "planned laser plane must intersect sphere");
        const double rho = std::sqrt(rho2);
        for (int i=0;i<21;++i) {
            const double dx=(-0.88+1.76*static_cast<double>(i)/20.0)*rho;
            SensorProfilePoint sp;
            sp.x=centerS.x+dx;
            sp.z=centerS.z-std::sqrt(std::max(0.0,rho*rho-dx*dx));
            obs.points.push_back(sp);
            const Vec3d pm=SingleSphere17Model::mapSensorPoint(obs.joints,{sp.x,0.0,sp.z},truth);
            require(std::abs(std::sqrt(dotv(subv(pm,centerM),subv(pm,centerM)))-kRadius)<2e-8,
                    "planned synthetic point must lie on fixed sphere");
        }
        out.push_back(std::move(obs));
    }
    return out;
}

void testGuidedBootstrapPlanHasRank14()
{
    const auto truth = truthParameters();
    const Vec3d centerM{245.0,-105.0,165.0};
    const Vec3d referenceCenterS{0.0,0.0,95.0};

    PosePlannerSeed seed;
    seed.taughtJoint = poseForTarget(0.0,0.0,referenceCenterS,centerM,truth);
    seed.taughtSphereCenterSensorMm = referenceCenterS;

    PosePlannerConfig cfg;
    cfg.bMinDeg=-20.0; cfg.bMaxDeg=20.0; cfg.bStepDeg=5.0;
    cfg.cMinDeg=-30.0; cfg.cMaxDeg=30.0; cfg.cStepDeg=5.0;
    cfg.requestedRecordedPoses=24;
    cfg.targetCenterYRadiusRatio=0.40;
    cfg.useBothLaserPlaneSides=false;
    cfg.maxRecordedPosesPerBCPair=2;
    cfg.xMinMm=cfg.yMinMm=cfg.zMinMm=-1e12;
    cfg.xMaxMm=cfg.yMaxMm=cfg.zMaxMm=+1e12;

    // The first plan is intentionally generated before geometry calibration.
    // Use a nominal/zero physical model here; only B/C and requested sensor
    // field locations are trusted. Closed-loop acquisition later finds the
    // real XYZ using measurements.
    Physical17Parameters nominal;
    SingleSphere17PosePlanner planner(cfg);
    std::string error;
    const auto plan=planner.generate(seed,nominal,error);
    require(plan.size()==24,"guided bootstrap planner should produce 24 poses from nominal geometry");
    for (const auto& p:plan) {
        require(p.targetSphereCenterSensorMm.y>0.0,
                "bootstrap plan should stay on one signed Y side");
    }

    // Emulate the guided closed loop: ignore planner XYZ seed and put the
    // fixed sphere at each requested sensor target using the real geometry.
    std::vector<CalibrationPoseInstruction> acquiredPlan = plan;
    for (auto& instruction : acquiredPlan) {
        instruction.predictedJoint = poseForTarget(
            instruction.predictedJoint.b, instruction.predictedJoint.c,
            instruction.targetSphereCenterSensorMm, centerM, truth);
    }
    Physical17SolverOptions options;
    SingleSphere17Calibrator cal(options);
    for (const auto& obs:observationsFromPlan(acquiredPlan,truth,centerM)) cal.addObservation(obs);
    const auto report=cal.analyzeObservability(truth,centerM);
    require(report.targetRank==14,
            "one-sided guided bootstrap with varied X/Z/B/C must reach physical rank14");
    require(report.fullStateRank==17,
            "one-sided guided bootstrap must reach full rank17");
}

void testSolvedSphereCenterRemovesTeachYBranchFromPlan()
{
    const auto truth = truthParameters();
    const Vec3d centerM{245.0,-105.0,165.0};
    const Vec3d referenceCenterS{0.0,0.0,95.0};

    PosePlannerConfig cfg;
    cfg.bMinDeg=-20.0; cfg.bMaxDeg=20.0; cfg.bStepDeg=10.0;
    cfg.cMinDeg=-30.0; cfg.cMaxDeg=30.0; cfg.cStepDeg=15.0;
    cfg.requestedRecordedPoses=10;
    cfg.useBothLaserPlaneSides=true;
    cfg.xMinMm=cfg.yMinMm=cfg.zMinMm=-1e12;
    cfg.xMaxMm=cfg.yMaxMm=cfg.zMaxMm=+1e12;

    PosePlannerSeed plus;
    plus.taughtJoint=poseForTarget(0.0,0.0,referenceCenterS,centerM,truth);
    plus.taughtSphereCenterSensorMm={0.0,+3.0,95.0};
    plus.hasSphereCenterMachine=true;
    plus.sphereCenterMachineMm=centerM;
    PosePlannerSeed minus=plus;
    minus.taughtSphereCenterSensorMm.y=-3.0;

    SingleSphere17PosePlanner planner(cfg);
    std::string e1,e2;
    const auto p1=planner.generate(plus,truth,e1);
    const auto p2=planner.generate(minus,truth,e2);
    require(!p1.empty() && p1.size()==p2.size(),
            "solved-center refinement plans should both generate");
    for (std::size_t i=0;i<p1.size();++i) {
        require(maxAbs(p1[i].targetSphereCenterSensorMm,p2[i].targetSphereCenterSensorMm)<1e-12,
                "known sphere center must remove taught +/-Y branch from target ordering");
        require(std::abs(p1[i].predictedJoint.x-p2[i].predictedJoint.x)<1e-10 &&
                std::abs(p1[i].predictedJoint.y-p2[i].predictedJoint.y)<1e-10 &&
                std::abs(p1[i].predictedJoint.z-p2[i].predictedJoint.z)<1e-10 &&
                std::abs(p1[i].predictedJoint.b-p2[i].predictedJoint.b)<1e-12 &&
                std::abs(p1[i].predictedJoint.c-p2[i].predictedJoint.c)<1e-12,
                "known sphere center must make predicted refinement plan independent of taught Y sign");
    }
}



Physical17Parameters nominalHandEyeTruth()
{
    Physical17Parameters p;
    p[Physical17Parameters::HandEyeRx]=+0.035;
    p[Physical17Parameters::HandEyeRy]=-0.022;
    p[Physical17Parameters::HandEyeRz]=+0.017;
    p[Physical17Parameters::HandEyeTx]=+31.5;
    p[Physical17Parameters::HandEyeTy]=-18.2;
    p[Physical17Parameters::HandEyeTz]=+53.7;
    return p;
}

std::vector<SphereProfileObservation> makeBootstrapObservations(
    const Physical17Parameters& truth,
    const Vec3d& centerM,
    bool varyB,
    bool varyC)
{
    const double bs[]={-24,-16,-8,0,8,16,24,12,-12,20,-20,4,-4,18,-18,0};
    const double cs[]={-36,-18,0,18,36,27,-27,9,-9,33,-33,15,-15,24,-24,30};
    const double xs[]={-3.5,0.0,3.5,1.5,-1.5};
    const double ys[]={+3.0,-3.8,+4.5,-2.5,+1.8,-4.8};
    const double zs[]={90.0,95.0,100.0,103.0,92.0};
    std::vector<SphereProfileObservation> out;
    for(int k=0;k<16;++k){
        const double b=varyB?bs[k]:0.0;
        const double c=varyC?cs[k]:0.0;
        const Vec3d centerS(xs[k%5],ys[k%6],zs[(2*k+1)%5]);
        SphereProfileObservation obs;
        obs.profileId=k+1;
        obs.joints=poseForTarget(b,c,centerS,centerM,truth);
        const double rho=std::sqrt(kRadius*kRadius-centerS.y*centerS.y);
        for(int i=0;i<25;++i){
            const double dx=(-0.90+1.80*static_cast<double>(i)/24.0)*rho;
            SensorProfilePoint sp;
            sp.x=centerS.x+dx;
            sp.z=centerS.z-std::sqrt(std::max(0.0,rho*rho-dx*dx));
            obs.points.push_back(sp);
        }
        out.push_back(std::move(obs));
    }
    return out;
}

void testNominalHandEyeSphereBootstrap()
{
    const auto truth=nominalHandEyeTruth();
    const Vec3d centerM{238.0,-103.0,168.0};

    Physical17SolverOptions options;
    options.multiStartCount=1;
    options.alternatingIterations=4;
    options.blockIterations=10;
    options.jointIterations=60;

    NominalHandEyeSphereCalibrator bootstrap(options);
    for(const auto& obs:makeBootstrapObservations(truth,centerM,true,true))
        bootstrap.addObservation(obs);
    const auto result=bootstrap.solve();
    require(result.success,"stage1 nominal hand-eye+sphere solve should succeed");
    require(result.stateRank==9,"stage1 must reach rank 9/9 with independent B and C excitation");
    require(result.residualRmsMm<1e-6,"stage1 noiseless raw sphere residual");
    for(std::size_t i=0;i<=Physical17Parameters::BTiltZ;++i)
        require(std::abs(result.parameters[i])<1e-14,"stage1 machine geometry must remain nominal");
    require(std::abs(result.parameters[Physical17Parameters::HandEyeRx]-truth[Physical17Parameters::HandEyeRx])<2e-5,
            "stage1 hand-eye Rx recovery");
    require(std::abs(result.parameters[Physical17Parameters::HandEyeRy]-truth[Physical17Parameters::HandEyeRy])<2e-5,
            "stage1 hand-eye Ry recovery");
    require(std::abs(result.parameters[Physical17Parameters::HandEyeRz]-truth[Physical17Parameters::HandEyeRz])<2e-5,
            "stage1 hand-eye Rz recovery");
    require(std::abs(result.parameters[Physical17Parameters::HandEyeTx]-truth[Physical17Parameters::HandEyeTx])<2e-3,
            "stage1 hand-eye Tx recovery");
    require(std::abs(result.parameters[Physical17Parameters::HandEyeTy]-truth[Physical17Parameters::HandEyeTy])<2e-3,
            "stage1 hand-eye Ty recovery");
    require(std::abs(result.parameters[Physical17Parameters::HandEyeTz]-truth[Physical17Parameters::HandEyeTz])<2e-3,
            "stage1 hand-eye Tz recovery");
    require(maxAbs(result.sphereCenterMachineMm,centerM)<2e-3,
            "stage1 fixed machine-frame sphere center recovery");

    NominalHandEyeSphereCalibrator noRot(options);
    for(const auto& obs:makeBootstrapObservations(truth,centerM,false,false)) noRot.addObservation(obs);
    const auto noRotResult=noRot.solve();
    require(noRotResult.stateRank<9,
            "XYZ-only stage1 must expose insufficient observability instead of pretending rank9");

    NominalHandEyeSphereCalibrator onlyB(options);
    for(const auto& obs:makeBootstrapObservations(truth,centerM,true,false)) onlyB.addObservation(obs);
    const auto onlyBResult=onlyB.solve();
    require(onlyBResult.stateRank<9,
            "single-rotary stage1 must expose insufficient observability");
}



void testStage1BootstrapsStage2Visibility()
{
    // Here the real machine contains all 11 Physical17 geometric errors, while
    // stage1 deliberately fixes those errors to zero.  The purpose of stage1
    // is therefore only a navigation-quality model, not the final geometry.
    const auto truth=truthParameters();
    const Vec3d centerM{245.0,-105.0,165.0};
    Physical17SolverOptions options;
    options.jointIterations=70;

    NominalHandEyeSphereCalibrator bootstrap(options);
    const auto manual=makeBootstrapObservations(truth,centerM,true,true);
    for(const auto& obs:manual) bootstrap.addObservation(obs);
    const auto stage1=bootstrap.solve();
    require(stage1.success && stage1.stateRank==9,
            "stage1 must remain solvable when small real machine-geometry errors are unmodelled");
    require(stage1.residualRmsMm<0.20,
            "stage1 nominal-machine residual should remain navigation quality in the synthetic case");

    PosePlannerSeed seed;
    seed.taughtJoint=manual.front().joints;
    seed.hasSphereCenterMachine=true;
    seed.sphereCenterMachineMm=stage1.sphereCenterMachineMm;
    seed.taughtSphereCenterSensorMm=SingleSphere17Model::machinePointInSensor(
        seed.taughtJoint,seed.sphereCenterMachineMm,stage1.parameters);

    PosePlannerConfig cfg;
    cfg.bMinDeg=-30.0; cfg.bMaxDeg=30.0; cfg.bStepDeg=10.0;
    cfg.cMinDeg=-45.0; cfg.cMaxDeg=45.0; cfg.cStepDeg=15.0;
    cfg.requestedRecordedPoses=30;
    cfg.useBothLaserPlaneSides=true;
    cfg.targetCenterYRadiusRatio=0.40;
    cfg.xMinMm=cfg.yMinMm=cfg.zMinMm=-1e12;
    cfg.xMaxMm=cfg.yMaxMm=cfg.zMaxMm=+1e12;

    SingleSphere17PosePlanner planner(cfg);
    std::string error;
    const auto plan=planner.generate(seed,stage1.parameters,error);
    require(plan.size()==30,"stage2 planner should produce the conservative 30-pose plan from stage1");

    double maxCenterError=0.0;
    for(const auto& instruction:plan){
        const Vec3d actual=SingleSphere17Model::machinePointInSensor(
            instruction.predictedJoint,centerM,truth);
        const Vec3d d=subv(actual,instruction.targetSphereCenterSensorMm);
        maxCenterError=std::max(maxCenterError,std::sqrt(dotv(d,d)));
        require(std::abs(actual.x)<18.0 && actual.z>80.0 && actual.z<110.0 && std::abs(actual.y)<kRadius,
                "stage1-planned stage2 pose should keep the real sphere in the conservative synthetic field");
    }
    require(maxCenterError<1.0,
            "stage1 model should predict stage2 sphere center to within 1 mm in the synthetic geometry-error case");
}

void testGaugeAndRanksAndSolve()
{
    const auto truth=truthParameters();
    const Vec3d centerM{245.0,-105.0,165.0};
    const auto observations=makeObservations(truth,centerM);

    // Exact 3-D translation gauge.
    Physical17Parameters shifted;
    Vec3d shiftedCenter;
    SingleSphere17Model::applyTranslationGauge(truth,centerM,{7.0,-4.0,3.0},shifted,shiftedCenter);
    for(std::size_t i=0;i<observations.size();i+=5){
        const auto& o=observations[i];
        for(std::size_t j=0;j<o.points.size();j+=5){
            Vec3d s{o.points[j].x,0.0,o.points[j].z};
            Vec3d a=subv(SingleSphere17Model::mapSensorPoint(o.joints,s,truth),centerM);
            Vec3d b=subv(SingleSphere17Model::mapSensorPoint(o.joints,s,shifted),shiftedCenter);
            require(maxAbs(a,b)<3e-10,"translation gauge must leave relative sphere vector invariant");
        }
    }

    Physical17SolverOptions options;
    options.multiStartCount=1;
    options.alternatingIterations=5;
    options.blockIterations=12;
    options.jointIterations=70;
    SingleSphere17Calibrator cal(options);
    for(const auto& o:observations)cal.addObservation(o);
    const auto report=cal.analyzeObservability(truth,centerM);
    require(report.targetRank==14,"physical rank after unknown-center elimination must be 14/17");
    require(report.fullStateRank==17,"full [physical17,center3] rank must be 17/20");

    Physical17Parameters initial;
    initial[Physical17Parameters::HandEyeRx]=0.003;
    initial[Physical17Parameters::HandEyeRy]=-0.004;
    initial[Physical17Parameters::HandEyeRz]=0.002;
    initial[Physical17Parameters::HandEyeTx]=truth[Physical17Parameters::HandEyeTx];
    initial[Physical17Parameters::HandEyeTy]=truth[Physical17Parameters::HandEyeTy];
    initial[Physical17Parameters::HandEyeTz]=truth[Physical17Parameters::HandEyeTz];
    const auto result=cal.solve(initial,nullptr); // exercises automatic +/- initializer
    require(result.success,"noiseless physical17 rank14 solve should succeed");
    require(result.observability.targetRank==14,"solve rank 14/17");
    require(result.observability.fullStateRank==17,"solve full rank 17/20");
    require(result.residualRmsMm<1e-7,"noiseless solve residual");
    require(std::abs(result.parameters[Physical17Parameters::HandEyeTx]-truth[Physical17Parameters::HandEyeTx])<1e-14,
            "hand-eye Tx gauge anchor must remain fixed");
    require(std::abs(result.parameters[Physical17Parameters::HandEyeTy]-truth[Physical17Parameters::HandEyeTy])<1e-14,
            "hand-eye Ty gauge anchor must remain fixed");
    require(std::abs(result.parameters[Physical17Parameters::HandEyeTz]-truth[Physical17Parameters::HandEyeTz])<1e-14,
            "hand-eye Tz gauge anchor must remain fixed");
    for(std::size_t i=0;i<14;++i){
        const double tol=(i==Physical17Parameters::COffsetU || i==Physical17Parameters::COffsetV ||
                          i==Physical17Parameters::BOffsetU || i==Physical17Parameters::BOffsetV) ? 2e-5 : 2e-7;
        require(std::abs(result.parameters[i]-truth[i])<tol,
                std::string("parameter recovery: ")+Physical17Parameters::names()[i]);
    }

    std::ostringstream json;
    std::string error;
    require(SingleSphere17Calibrator::writeResultJson(json,result,error),"write V20.2 JSON");
    const std::string text=json.str();
    require(text.find("single_sphere_physical17_rank14")!=std::string::npos,"JSON model id");
    require(text.find("handeye_translation_gauge_anchor_mm")!=std::string::npos,"JSON gauge anchor");
    require(text.find("full_state_rank")!=std::string::npos,"JSON full-state rank");
}

void testQualityStudyPlannerTargetsStayInsideDgammaEtaWindow()
{
    const auto truth = truthParameters();
    const Vec3d centerM{245.0,-105.0,165.0};
    const Vec3d referenceCenterS{0.0,0.0,96.0};

    PosePlannerSeed seed;
    seed.taughtJoint = poseForTarget(0.0,0.0,referenceCenterS,centerM,truth);
    seed.taughtSphereCenterSensorMm = referenceCenterS;
    seed.hasSphereCenterMachine = true;
    seed.sphereCenterMachineMm = centerM;

    PosePlannerConfig cfg;
    cfg.bMinDeg=-20.0; cfg.bMaxDeg=20.0; cfg.bStepDeg=10.0;
    cfg.cMinDeg=-30.0; cfg.cMaxDeg=30.0; cfg.cStepDeg=15.0;
    cfg.requestedRecordedPoses=18;
    cfg.maxRecordedPosesPerBCPair=2;
    cfg.useBothLaserPlaneSides=true;
    cfg.useQualityStudyGeometry=true;
    cfg.qualityDMinMm=92.0; cfg.qualityDMaxMm=100.0; cfg.qualityDStepMm=4.0;
    cfg.qualityGammaMinDeg=0.0; cfg.qualityGammaMaxDeg=50.0; cfg.qualityGammaStepDeg=25.0;
    cfg.qualityEtaMinDeg=-7.5; cfg.qualityEtaMaxDeg=7.5; cfg.qualityEtaStepDeg=7.5;
    // Do not let the legacy conservative optical screen obscure this unit test:
    // exact optical feasibility is checked by the integrated GUI preflight.
    cfg.sensorXHalfWidthMm=100.0;
    cfg.sensorZMinMm=0.0;
    cfg.sensorZMaxMm=200.0;
    cfg.xMinMm=cfg.yMinMm=cfg.zMinMm=-1e12;
    cfg.xMaxMm=cfg.yMaxMm=cfg.zMaxMm=+1e12;

    SingleSphere17PosePlanner planner(cfg);
    std::string error;
    const auto plan=planner.generate(seed,truth,error);
    require(plan.size()==18,"quality-study planner should produce the requested D/gamma/eta constrained plan");

    constexpr double radToDeg = 180.0 / 3.14159265358979323846;
    const double radius=0.5*cfg.sphereDiameterMm;
    for(const auto& instruction:plan){
        const Vec3d& t=instruction.targetSphereCenterSensorMm;
        const double d=std::hypot(t.x,t.z);
        const double ratio=std::clamp(std::abs(t.y)/radius,0.0,1.0);
        const double gamma=std::asin(ratio)*radToDeg;
        const double eta=std::atan2(t.x,t.z)*radToDeg;
        require(d>=cfg.qualityDMinMm-1e-9 && d<=cfg.qualityDMaxMm+1e-9,
                "quality-study target D must stay inside requested range");
        require(gamma>=cfg.qualityGammaMinDeg-1e-9 && gamma<=cfg.qualityGammaMaxDeg+1e-9,
                "quality-study target gamma must stay inside requested range");
        require(eta>=cfg.qualityEtaMinDeg-1e-9 && eta<=cfg.qualityEtaMaxDeg+1e-9,
                "quality-study target eta must stay inside requested range");
    }
}

void testQualityStudyPlannerStillProvidesPhysicalRank14()
{
    const auto truth = truthParameters();
    const Vec3d centerM{245.0,-105.0,165.0};
    const Vec3d referenceCenterS{0.0,0.0,96.0};

    PosePlannerSeed seed;
    seed.taughtJoint = poseForTarget(0.0,0.0,referenceCenterS,centerM,truth);
    seed.taughtSphereCenterSensorMm = referenceCenterS;
    seed.hasSphereCenterMachine = true;
    seed.sphereCenterMachineMm = centerM;

    PosePlannerConfig cfg;
    cfg.bMinDeg=-30.0; cfg.bMaxDeg=30.0; cfg.bStepDeg=10.0;
    cfg.cMinDeg=-45.0; cfg.cMaxDeg=45.0; cfg.cStepDeg=15.0;
    cfg.requestedRecordedPoses=30;
    cfg.maxRecordedPosesPerBCPair=3;
    cfg.useBothLaserPlaneSides=true;
    cfg.useQualityStudyGeometry=true;
    cfg.qualityDMinMm=92.0; cfg.qualityDMaxMm=100.0; cfg.qualityDStepMm=4.0;
    cfg.qualityGammaMinDeg=0.0; cfg.qualityGammaMaxDeg=50.0; cfg.qualityGammaStepDeg=25.0;
    cfg.qualityEtaMinDeg=-7.5; cfg.qualityEtaMaxDeg=7.5; cfg.qualityEtaStepDeg=7.5;
    cfg.sensorXHalfWidthMm=100.0;
    cfg.sensorZMinMm=0.0; cfg.sensorZMaxMm=200.0;
    cfg.xMinMm=cfg.yMinMm=cfg.zMinMm=-1e12;
    cfg.xMaxMm=cfg.yMaxMm=cfg.zMaxMm=+1e12;

    SingleSphere17PosePlanner planner(cfg);
    std::string error;
    const auto plan=planner.generate(seed,truth,error);
    require(plan.size()==30,"quality-study D-optimal planner should produce 30 poses");

    Physical17SolverOptions options;
    SingleSphere17Calibrator cal(options);
    for (const auto& obs:observationsFromPlan(plan,truth,centerM)) cal.addObservation(obs);
    const auto report=cal.analyzeObservability(truth,centerM);
    require(report.targetRank>=14,"quality-constrained D/gamma/eta plan must preserve physical rank >=14");
    require(report.fullStateRank>=17,"quality-constrained D/gamma/eta plan must preserve full-state rank >=17");
}

void testStage2DataSelfBootstrapAnchorsTranslation()
{
    const auto truth=truthParameters();
    const Vec3d centerM{245.0,-105.0,165.0};
    const auto observations=makeObservations(truth,centerM);

    Physical17SolverOptions options;
    options.multiStartCount=1;
    options.alternatingIterations=5;
    options.blockIterations=12;
    options.jointIterations=80;

    SingleSphere17Calibrator cal(options);
    for(const auto& obs:observations) cal.addObservation(obs);

    // V27.6.41 direct-import path: the Stage-2 observations themselves first
    // solve the nominal-machine 9-state hand-eye+sphere problem.
    const auto bootstrap=cal.solveNominalHandEyeBootstrap();
    require(bootstrap.success,"Stage-2 data self-bootstrap should succeed");
    require(bootstrap.stateRank==9,"Stage-2 data self-bootstrap must reach rank 9/9");
    require(std::isfinite(bootstrap.residualRmsMm),"Stage-2 bootstrap RMS must be finite");

    // The full Physical17 solve starts from that result.  Its three hand-eye
    // translations are an explicit gauge anchor and must never move.
    auto result=cal.solve(bootstrap.parameters,&bootstrap.sphereCenterMachineMm);
    require(result.success,"self-bootstrapped Physical17 solve should succeed");
    require(result.observability.targetRank>=14,"self-bootstrapped physical rank must be >=14");
    require(result.observability.fullStateRank>=17,"self-bootstrapped full-state rank must be >=17");
    require(result.residualRmsMm<1e-7,"self-bootstrapped noiseless Physical17 residual");
    require(std::abs(result.parameters[Physical17Parameters::HandEyeTx]
                     -bootstrap.parameters[Physical17Parameters::HandEyeTx])<1e-14,
            "self-bootstrap Tx must remain the exact gauge anchor");
    require(std::abs(result.parameters[Physical17Parameters::HandEyeTy]
                     -bootstrap.parameters[Physical17Parameters::HandEyeTy])<1e-14,
            "self-bootstrap Ty must remain the exact gauge anchor");
    require(std::abs(result.parameters[Physical17Parameters::HandEyeTz]
                     -bootstrap.parameters[Physical17Parameters::HandEyeTz])<1e-14,
            "self-bootstrap Tz must remain the exact gauge anchor");

    result.hasInitializationBootstrap=true;
    result.initializationSource="stage2_training_data_nominal_machine_handeye";
    result.initializationParameters=bootstrap.parameters;
    result.initializationSphereCenterMachineMm=bootstrap.sphereCenterMachineMm;
    result.initializationResidualRmsMm=bootstrap.residualRmsMm;
    result.initializationResidualMaxAbsMm=bootstrap.residualMaxAbsMm;
    result.initializationStateRank=bootstrap.stateRank;
    result.initializationConditionNumber=bootstrap.conditionNumber;
    result.initializationProfileCount=bootstrap.profileCount;
    result.initializationPointCount=bootstrap.pointCount;

    std::ostringstream json;
    std::string error;
    require(SingleSphere17Calibrator::writeResultJson(json,result,error),
            "write self-bootstrap Physical17 JSON");
    const std::string text=json.str();
    require(text.find("initialization_bootstrap")!=std::string::npos,
            "JSON must preserve Stage-2 self-bootstrap provenance");
    require(text.find("stage2_training_data_nominal_machine_handeye")!=std::string::npos,
            "JSON must preserve self-bootstrap source id");
}


void testRobustPoseScreeningRejectsBadPoseGroups()
{
    const auto truth=truthParameters();
    const Vec3d centerM{245.0,-105.0,165.0};
    auto observations=makeObservations(truth,centerM);
    const std::vector<std::size_t> badIndices{8,9,15,16,22,23};
    for (std::size_t index : badIndices) {
        require(index<observations.size(),"bad synthetic pose index");
        for (auto& point : observations[index].points) {
            point.z += 0.28; // pose-level acquisition/registration fault
        }
    }

    Physical17PoseScreeningOptions options;
    options.minRetainedPoseCount=12;
    options.maxRejectedFraction=0.25;
    options.absoluteRmsFloorMm=0.080;
    options.absoluteP95FloorMm=0.150;
    const auto report=SingleSphere17Calibrator::screenPoseOutliers(
        observations,truth,centerM,19.05,options);
    require(report.valid,"pose screening report must be valid");
    require(report.inputPoseCount==observations.size(),"one synthetic profile per pose");
    require(report.rejectedPoseCount==badIndices.size(),"all injected bad poses should be rejected");
    require(report.retainedPoseCount+report.rejectedPoseCount==report.inputPoseCount,
            "pose screening counts must balance");

    std::vector<SphereProfileObservation> kept;
    for (const auto& observation : observations) {
        bool rejected=false;
        for (const auto& summary : report.poses) {
            if (!summary.rejected) continue;
            const auto& a=observation.joints; const auto& b=summary.pose;
            if (std::abs(a.x-b.x)<=options.poseLinearToleranceMm &&
                std::abs(a.y-b.y)<=options.poseLinearToleranceMm &&
                std::abs(a.z-b.z)<=options.poseLinearToleranceMm &&
                std::abs(a.c-b.c)<=options.poseAngularToleranceDeg &&
                std::abs(a.b-b.b)<=options.poseAngularToleranceDeg) {
                rejected=true; break;
            }
        }
        if (!rejected) kept.push_back(observation);
    }
    const auto clean=SingleSphere17Calibrator::validateModel(kept,truth,centerM,19.05);
    require(clean.valid,"screened synthetic data must validate");
    require(clean.residualRmsMm<1e-9,"screening must retain the noiseless good poses");
}


void testProgressivePoseScreeningBreaksOutlierMasking()
{
    const auto truth=truthParameters();
    const Vec3d centerM{245.0,-105.0,165.0};
    auto observations=makeObservations(truth,centerM);

    // A broad tail of moderately degraded pose groups plus one very poor pose.
    // This emulates the V27.6.40 field case where MAD was inflated enough that
    // only the single worst pose crossed the classical threshold.
    const std::vector<std::pair<std::size_t,double>> faults{
        {4,0.070},{7,0.075},{10,0.080},{13,0.085},{16,0.090},
        {19,0.100},{22,0.110},{25,0.120},{28,0.135},{31,0.200}
    };
    for (const auto& fault:faults) {
        require(fault.first<observations.size(),"masked-tail synthetic pose index");
        for (auto& point:observations[fault.first].points) point.z += fault.second;
    }

    Physical17PoseScreeningOptions options;
    options.minRetainedPoseCount=18;
    options.maxRejectedFraction=0.15;
    options.madSigma=1.75;
    options.relativeRmsFactor=1.30;
    options.relativeP95Factor=1.30;
    options.absoluteRmsFloorMm=0.050;
    options.absoluteP95FloorMm=0.080;
    options.enableProgressiveTrim=true;
    options.targetTrainingRmsMm=0.045;
    options.progressiveTrimFraction=0.12;
    options.minProgressiveTrimCount=3;
    options.maxProgressiveTrimCount=6;
    options.minProgressiveSpreadMm=0.010;
    options.progressiveTailFactor=1.02;

    const auto report=SingleSphere17Calibrator::screenPoseOutliers(
        observations,truth,centerM,19.05,options);
    require(report.valid,"progressive pose screening report must be valid");
    require(report.rejectedPoseCount>=3,
            "progressive screening must test a batch, not only the single worst pose");
    require(report.rejectedPoseCount<=6,
            "per-round progressive screening must remain bounded");
    require(report.statisticalCandidateCount+report.progressiveCandidateCount>=report.rejectedPoseCount,
            "reported candidate accounting must cover selected poses");
}

void testIndependentValidationUsesFrozenModel()
{
    const auto truth=truthParameters();
    const Vec3d centerM{245.0,-105.0,165.0};
    auto observations=makeObservations(truth,centerM);
    const auto exact=SingleSphere17Calibrator::validateModel(
        observations,truth,centerM,19.05);
    require(exact.valid,"exact hold-out validation report must be valid");
    require(exact.profileCount==observations.size(),"validation profile count");
    require(exact.pointCount==observations.size()*observations.front().points.size(),"validation point count");
    require(exact.residualRmsMm<1e-9,"exact hold-out RMS must be essentially zero");

    Physical17Parameters wrong=truth;
    wrong[Physical17Parameters::HandEyeRx]+=0.010;
    const auto bad=SingleSphere17Calibrator::validateModel(
        observations,wrong,centerM,19.05);
    require(bad.valid,"perturbed hold-out validation report must still be computable");
    require(bad.residualRmsMm>1e-2,"frozen wrong model must be rejected by nonzero hold-out residual");
}
}

int main()
{
    testNominalConsistency();
    testNominalHandEyeSphereBootstrap();
    testStage1BootstrapsStage2Visibility();
    testGaugeAndRanksAndSolve();
    testGuidedBootstrapPlanHasRank14();
    testSolvedSphereCenterRemovesTeachYBranchFromPlan();
    testQualityStudyPlannerTargetsStayInsideDgammaEtaWindow();
    testQualityStudyPlannerStillProvidesPhysicalRank14();
    testStage2DataSelfBootstrapAnchorsTranslation();
    testRobustPoseScreeningRejectsBadPoseGroups();
    testProgressivePoseScreeningBreaksOutlierMasking();
    testIndependentValidationUsesFrozenModel();
    std::cout << "V20.2 physical17/rank14 + two-stage calibration tests passed\n";
    return 0;
}
