#include "twin/MultiPoseExperimentQtAdapter.h"
#include "twin/TwinM6NcPreview.h"
#include "twin/MultiPoseOpticalCoverage.h"
#include "twin/TwinM6ScanArchive.h"
#include "calibration/SingleSphere17Calibration.h"
#include "core/Physical17Identity.h"
#include "safety/MachineSafetyModel.h"
#include "runtime/GCode5D.h"
#include "gcode/GCodeParser.h"
#include <QCryptographicHash>
#include <QDateTime>
#include <QDataStream>
#include <QTemporaryFile>
#include <QTextStream>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSaveFile>
#include <QStringList>
#include <cmath>
#include <filesystem>
#include <sstream>
#include <algorithm>
#include <limits>
#include <numeric>

namespace mls::twin {
namespace {
Vec3 at(const Transform& x){return x.point({0,0,0});}
Joint5 qFrom(const std::array<double,5>& a){return {{a[0],a[1],a[2],a[3],a[4]}};}
mls::calibration::JointPose5d qForPhysical(const Joint5& q){return {q[0],q[1],q[2],q[3],q[4]};}
Transform physFk(const Joint5& q,const mls::calibration::Physical17Parameters& p){
    return {mls::calibration::SingleSphere17Model::sensorToMachineTransform(qForPhysical(q),p)};
}
bool finiteMatrix(const Transform& x){for(double v:x.m)if(!std::isfinite(v))return false;return true;}
bool writeAtomic(const QString& path,const QByteArray& bytes,QString* err){
    QSaveFile f(path);
    if(!f.open(QIODevice::WriteOnly)){if(err)*err=QStringLiteral("无法写入 ")+path+QStringLiteral(": ")+f.errorString();return false;}
    if(f.write(bytes)!=bytes.size()||!f.commit()){
        if(err)*err=QStringLiteral("写入失败 ")+path+QStringLiteral(": ")+f.errorString();return false;
    }
    return true;
}
QByteArray fileSha256(const QString& path){
    QFile f(path);if(!f.open(QIODevice::ReadOnly))return {};
    QCryptographicHash hash(QCryptographicHash::Sha256);
    while(!f.atEnd())hash.addData(f.read(1024*1024));
    return hash.result().toHex();
}
// Enforce the SAME grammar + program-validation contract as GCodeWidget.
// Passing this check authorizes import/parse ONLY, never physical execution.
QString verifyGCodeImportSyntax(const QByteArray& nc,const QString& label){
    GCodeParser parser;
    GCodeProgram program;
    QList<ParseError> errors;
    if(!parser.parseText(QString::fromUtf8(nc),program,&errors)){
        const ParseError first = errors.isEmpty() ? ParseError{} : errors.first();
        return QStringLiteral("G代码模块导入校验失败 [%1] 行%2：%3")
            .arg(label).arg(first.lineNumber)
            .arg(first.errorMessage.isEmpty()?QStringLiteral("未知解析错误"):first.errorMessage);
    }
    if(!program.offlineOnly || program.syncMode!=SyncMode::Hardware ||
       program.programFrame!=QStringLiteral("MACHINE_JOINTS_XYZCB")){
        return QStringLiteral("导出文件头与M6离线XYZCB协议不一致：")+label;
    }
    return {};
}
QJsonArray jsonJoint(const Joint5& q){QJsonArray a;for(double v:q)a.append(v);return a;}
QJsonArray jsonPose(const Pose6& p){return {p.x,p.y,p.z,p.rxDeg,p.ryDeg,p.rzDeg};}
QJsonArray jsonMatrix(const Transform& m){QJsonArray a;for(double v:m.m)a.append(v);return a;}
QJsonObject metadata(const MultiPoseQtReport& r){
    QJsonObject root;
    root.insert("schema","MLS_MULTIPOSE_EXPERIMENT_V1_OFFLINE");
    root.insert("purpose","Same physical CAD region, independent C/B angle measurement experiment for M3/M4");
    root.insert("sourceCad",r.cadPath);root.insert("sourceScan",r.scanPath);
    root.insert("cadSha256",r.cadSha256);
    root.insert("realScanSha256",r.realScanSha256);
    root.insert("partPoseId",r.partPoseId);
    root.insert("calibrationId",r.calibrationId);
    root.insert("physical17Sha256",r.physical17Sha256);
    QJsonArray physical17Snapshot;
    for(const double value:r.physicalParameters)physical17Snapshot.append(value);
    root.insert("importedPhysical17Parameters",physical17Snapshot);
    root.insert("provenanceMeaning","IMPORTED_CANDIDATE_IDENTITIES_NOT_CERTIFICATIONS");
    root.insert("cadToMachine",jsonMatrix(Transform::fromPose(r.cadPose)));
    root.insert("cadPlacementPose6",jsonPose(r.cadPose));
    root.insert("registrationVerified",false);
    root.insert("registrationProcedure","ROUGH_HORIZONTAL_INITIALIZATION_OR_USER_INITIAL_POSE_ONLY");
    root.insert("fullSixDofRegistrationImplemented",false);
    root.insert("opticalHitRequirement","Each pose: projected fan envelope + per-ROI optical coverage; preview is not a metrology guarantee");
    root.insert("opticsAcrossAnglesVerified",false);
    root.insert("hardwareAuthorized",false);
    root.insert("initialStateIsHistoricalNotLive",true);
    root.insert("fullPhysical17ReplayConsistent",r.physical17ReplayConsistent);
    root.insert("BCVariedInRecordedScan",r.rotaryReplayExcited);
    root.insert("rawScanNonMonotonicTimestampCount",static_cast<qint64>(r.timeOrderAnomalies));
    root.insert("machineCollisionModelLoaded",r.machineCollisionModelLoaded);
    root.insert("opticalPreviewUsable",r.opticalPreviewUsable);
    root.insert("kinematics",r.kinematicsSummary);
    root.insert("warning",r.warning);
    root.insert("rateHz",r.options.profileRateHz);
    root.insert("scanSpeedMmPerSec",r.options.scanSpeedMmPerSec);
    root.insert("nominalProfilePitchMm",r.options.scanSpeedMmPerSec/r.options.profileRateHz);
    root.insert("focusSensorZmm",r.options.focusSensorZmm);
    root.insert("roiMinCad",QJsonArray{r.options.roiCad.min.x,r.options.roiCad.min.y,r.options.roiCad.min.z});
    root.insert("roiMaxCad",QJsonArray{r.options.roiCad.max.x,r.options.roiCad.max.y,r.options.roiCad.max.z});
    root.insert("referenceSurfaceZCadMm",r.options.surfaceZCadMm);
    root.insert("sweepAxis",r.options.direction==ExperimentSweepAxis::AutoMachineX
        ?"AUTO_MACHINE_X_POS_TO_NEG"
        :(r.options.direction==ExperimentSweepAxis::CadY?"CAD_Y":"CAD_X"));
    root.insert("scanDirectionMachineX", "X_PLUS_TO_X_MINUS");
    root.insert("leadInBeforeWorkpieceMm",r.options.leadInMm);
    root.insert("leadOutAfterWorkpieceMm",r.options.leadOutMm);
    root.insert("tiltAwareLaserFanProjection",true);
    root.insert("lanePitchMm",r.options.lanePitchMm);
    root.insert("retractCadZMm",0.0); // deprecated, explicitly disabled
    root.insert("scanEntryExitPolicy","NO_AUTO_RETRACT; NO_HISTORICAL_START_MOVE; LASER_OFF_LANE_TRANSFERS");
    root.insert("angularOpticsModel","UNVERIFIED: virtual pinhole geometry only");
    return root;
}
}
std::shared_ptr<MultiPoseQtReport> MultiPoseExperimentQtAdapter::planAndSimulatePreview(
    const TwinScene& scene,const ScanReplayData& real,
    const std::vector<ExperimentPose>& poses,const ExperimentOptions& options,
    const Sensor2950Geometry& optical,const QString& machineConfigPath,
    const std::atomic<bool>* cancel){
    auto d=std::make_shared<MultiPoseQtReport>();
    d->options=options;
    d->cadPose=scene.estimatedPose();
    d->cadPath=QString::fromLocal8Bit(scene.mesh().source.c_str());
    d->scanPath=real.sourcePath;
    if(!scene.hasWorkpiece()||!real.ok()||real.scan.profiles.isEmpty()){
        d->status=QStringLiteral("需要独立导入 STEP + 实测 SCAN v3");return d;
    }
    for(const auto& profile:real.scan.profiles){
        if(profile.fiveAxis.frame.timing==mls::runtime::PoseTiming::Simulated){
            d->status=QStringLiteral("不接受离线合成 SCAN 充当真实前期校准数据");return d;
        }
    }
    const auto& start=real.scan.profiles.first().fiveAxis.frame;
    for(qsizetype j=1;j<real.scan.profiles.size();++j){
        if(real.scan.profiles[j].fiveAxis.timeUs <= real.scan.profiles[j-1].fiveAxis.timeUs)
            ++d->timeOrderAnomalies;
    }
    const Joint5 origin=qFrom(start.joints);
    const Transform recorded{start.sensorToMachine};
    if(!finiteMatrix(recorded)){d->status=QStringLiteral("首帧 XYZCB 变换无效");return d;}
    mls::calibration::Physical17Parameters p;p.value=start.physicalParameters;
    for(double x:p.value)if(!std::isfinite(x)){
        d->status=QStringLiteral("SCAN 中 Physical17 参数非有限值，无法建立五轴姿态模型");return d;
    }
    d->physicalParameters=p.value;
    d->cadSha256=QString::fromLatin1(fileSha256(d->cadPath));
    d->realScanSha256=QString::fromLatin1(fileSha256(d->scanPath));
    d->physical17Sha256=mls::identity::physical17Sha256(p.value);
    d->calibrationId=mls::identity::candidateCalibrationId(p.value);
    d->partPoseId=mls::identity::candidatePoseId(d->cadSha256.toLatin1(),
                                                   Transform::fromPose(d->cadPose).m);
    if(d->cadSha256.isEmpty()||d->realScanSha256.isEmpty()||
       d->physical17Sha256.isEmpty()||d->calibrationId.isEmpty()||d->partPoseId.isEmpty()){
        d->status=QStringLiteral("STEP/SCAN/Physical17 来源哈希无法建立，拒绝发布NC");return d;
    }
    const Transform predicted=physFk(origin,p);
    if(!finiteMatrix(predicted)){
        d->status=QStringLiteral("Physical17 FK 计算得到无效矩阵");return d;
    }
    const Transform anchor=recorded*predicted.inverseRigid();
    d->physical17Anchor=anchor;
    const M6ForwardKinematics fk=[p,anchor](const Joint5& q){return anchor*physFk(q,p);};
    double maxPos=0,maxRot=0;
    const qsizetype sz=real.scan.profiles.size();
    for(const qsizetype i:{qsizetype(0),sz/4,sz/2,(3*sz)/4,sz-1}){
        if(cancel&&cancel->load()){d->status=QStringLiteral("用户取消");return d;}
        const auto& a=real.scan.profiles[i].fiveAxis.frame;
        if(std::abs(a.joints[3]-origin[3])>0.05||std::abs(a.joints[4]-origin[4])>0.05)
            d->rotaryReplayExcited=true;
        const Transform calc=fk(qFrom(a.joints));
        const Transform stored{a.sensorToMachine};
        maxPos=std::max(maxPos,length(at(calc)-at(stored)));
        for(int row=0;row<3;++row)for(int col=0;col<3;++col)
            maxRot=std::max(maxRot,std::abs(calc.m[std::size_t(row*4+col)]-stored.m[std::size_t(row*4+col)]));
    }
    if(maxPos>=0.30||maxRot>=.003){
        d->status=QStringLiteral("Physical17 与存储变换重放不一致：位置最大差=%1 mm，旋转项差=%2；禁止生成五轴运动程序")
           .arg(maxPos,0,'g',6).arg(maxRot,0,'g',6);return d;
    }
    d->physical17ReplayConsistent=true;
    d->kinematicsSummary=QStringLiteral("M2 Physical17锚定重放 5帧 | 最大位置误差 %1mm，旋转项 %2 | BC激励:%3")
        .arg(maxPos,0,'g',6).arg(maxRot,0,'g',6).arg(d->rotaryReplayExcited?"YES":"NO");
    ExperimentOptions local=options;
    local.opticalGeometry=optical;
    if(!local.hasArchivedInitialJoints){local.archivedInitialJoints=origin;local.hasArchivedInitialJoints=true;}
    std::shared_ptr<mls::safety::MachineSafetyModel> model;
    if(!machineConfigPath.isEmpty()&&QFileInfo::exists(machineConfigPath)){
        auto candidate=std::make_shared<mls::safety::MachineSafetyModel>();
        std::string why;
        if(candidate->load(std::filesystem::path(machineConfigPath.toStdWString()),&why)&&candidate->isReady()){
            model=candidate;
            const auto& cfg=model->config().planningLimits;
            if(cfg.configured){
                const auto& guard=model->config().safety;
                const double gap=guard.linearJointMarginMm,ang=guard.rotaryJointMarginDeg;
                local.bounds.min={{cfg.xMinMm+gap,cfg.yMinMm+gap,cfg.zMinMm+gap,cfg.cMinDeg+ang,cfg.bMinDeg+ang}};
                local.bounds.max={{cfg.xMaxMm-gap,cfg.yMaxMm-gap,cfg.zMaxMm-gap,cfg.cMaxDeg-ang,cfg.bMaxDeg-ang}};
            }
        }else{
            d->status=QStringLiteral("已指定机床碰撞模型但加载失败：")+QString::fromStdString(why);
            return d;
        }
    }
    d->machineCollisionModelLoaded=bool(model);
    M6CollisionCheck collision;
    if(model){collision=[model](const Joint5& q,std::string* why){
        const auto r=model->checkPose(JointPose5D{q[0],q[1],q[2],q[3],q[4]},true);
        if(!r.safe&&why)*why=r.message;
        return r.modelReady&&r.safe;
    };}
    d->options=local;
    d->plan=planMultiPoseExperiment(scene,poses,local,fk,collision);
    if(!d->plan.ok()){
        d->status=QString::fromStdString(d->plan.error);
        // If a complete experiment stops at the first axis-bound failure,
        // diagnose every requested pose separately so P01 does not hide P41.
        // This fast pass intentionally omits FCL; it does NOT approve any NC.
        if(poses.size()>1 && d->plan.error.find("joint outside configured planning bounds")!=std::string::npos){
            int kinematicPass=0;
            int kinematicFail=0;
            QStringList failures;
            for(const auto& pose:poses){
                if(cancel&&cancel->load()){
                    d->status=QStringLiteral("用户取消");return d;
                }
                const auto single=planMultiPoseExperiment(scene,{pose},local,fk);
                if(single.ok())++kinematicPass;
                else{
                    ++kinematicFail;
                    if(failures.size()<24)
                        failures.push_back(QString::fromStdString(single.error));
                }
            }
            d->status+=QStringLiteral("\n基础单姿态离线检查（不含 FCL、完整光学覆盖、所有姿态合计 Profile 预算）：%1/%2 可行，%3 不可行。")
                .arg(kinematicPass).arg(poses.size()).arg(kinematicFail);
            for(const auto& reason:failures)d->status+=QStringLiteral("\n")+reason;
            if(kinematicFail>failures.size())
                d->status+=QStringLiteral("\n其他失败姿态数量：%1").arg(kinematicFail-failures.size());
            d->status+=QStringLiteral("\n本次不发布 NC：请核查机床限位配置的现场有效性、工件装夹位置及所需光学工作距离，禁止自动删除失败姿态或放宽限位。");
        }
        return d;
    }
    if(cancel&&cancel->load()){d->status=QStringLiteral("用户取消");return d;}
    // Plan connected laser-OFF G05 S0 motion on the background worker.
    // Failure here does not erase independently valid per-pose NC results,
    // but strictly disables the misleading all-pose continuous playback.
    d->connectedSession=planConnectedExperimentSession(d->plan,local,fk,collision);
    if(!d->connectedSession.ok()){
        d->continuousSessionIssue=QString::fromStdString(d->connectedSession.error);
        d->warning+=QStringLiteral("\n连续姿态转场未验证：")+d->continuousSessionIssue
            +QStringLiteral("；不能连续播放不同姿态，仍可分别验证独立 NC。");
    }
    VirtualLaser2950 virtualLaser(scene,optical,true);
    std::string error;
    if(!virtualLaser.ready(&error)){
        d->status=QStringLiteral("M3 光学模型初始化失败（五轴轨迹已生成，光学未验收）：")
                  +QString::fromStdString(error);return d;
    }
    for(std::size_t viewIndex=0;viewIndex<d->plan.views.size();++viewIndex){
        const auto& view=d->plan.views[viewIndex];
        // Previews are also generated from the exact G05/G93 NC program, so the
        // planned optical test cannot hide an incorrect/changed NC command.
        const auto executed=executeNcForPose(*d,viewIndex,3,115);
        if(!executed.ok()){
            d->status=QStringLiteral("M3 NC 指令级仿真失败：")+executed.error;
            return d;
        }
        const auto& nc=executed.execution;
        QVector<QVector3D> pts;
        QVector<QVector<QVector3D>> replayFrames;
        QVector<double> replayTimes;
        QVector<QVector3D> line;
        int workpieceFrames=0;
        MultiPoseOpticalCoverageGrid roiCoverage(scene,local.roiCad,6);
        const float nan=std::numeric_limits<float>::quiet_NaN();
        d->transferFocusPath.append({nan,nan,nan});
        for(const auto& p:nc.vertices){
            const Vec3 x=fk(p.joints).point({0,0,local.focusSensorZmm});
            const QVector3D pt{float(x.x),float(x.y),float(x.z)};
            if(p.scanning)line.append(pt);
            else d->transferFocusPath.append(pt);
        }
        const std::size_t samples=std::min<std::size_t>(
            std::max(1u,local.opticalPreviewProfilesPerPose),nc.profiles.size());
        for(std::size_t preview=0;preview<samples;++preview){
            const std::size_t profileIndex=preview*(nc.profiles.size()-1)/std::max<std::size_t>(1,samples-1);
            const auto& src=nc.profiles[profileIndex];
            VirtualProfilePose pose;
            pose.elapsedSeconds=src.seconds;pose.joints=src.joints;
            pose.sourceOrdinal=static_cast<std::uint64_t>(profileIndex);
            pose.sensorToMachine=fk(src.joints);
            if(cancel&&cancel->load()){d->status=QStringLiteral("用户取消");return d;}
            const auto vf=virtualLaser.simulate(pose);
            roiCoverage.ingest(vf);
            bool frameHasHit=false;
            QVector<QVector3D> perFrame;
            for(std::size_t j=0;j<vf.samples.size();++j){
                const auto& sample=vf.samples[j];
                if(sample.state!=VirtualSampleState::Valid||sample.hitObject!=VirtualHitObject::Workpiece)continue;
                frameHasHit=true;
                if(j%3==0&&pts.size()<9000){
                    const Vec3 x=sample.hitPositionMachine;
                    const QVector3D point{float(x.x),float(x.y),float(x.z)};
                    pts.append(point);
                    perFrame.append(point);
                }
            }
            if(frameHasHit)++workpieceFrames;
            replayTimes.append(pose.elapsedSeconds);
            replayFrames.append(std::move(perFrame));
        }
        const auto coverage=roiCoverage.summarize();
        d->previewRoiCoverageFractions.append(coverage.fraction);
        d->previewBoundaryCoverageFractions.append(coverage.boundaryFraction);
        d->previewCoverageWarnings.append(QString::fromStdString(coverage.message));
        d->previewProfilesWithWorkpieceHit.append(workpieceFrames);
        d->virtualPointPreviews.append(std::move(pts));
        d->opticalPlaybackTimes.append(std::move(replayTimes));
        d->opticalPlaybackFrames.append(std::move(replayFrames));
        d->scanFocusPaths.append(std::move(line));
    }
    d->opticalPreviewUsable=!d->previewProfilesWithWorkpieceHit.isEmpty();
    for(qsizetype i=0;i<d->previewProfilesWithWorkpieceHit.size();++i){
        const int validFrames=d->previewProfilesWithWorkpieceHit[i];
        const auto total=std::size_t(d->opticalPlaybackTimes.at(i).size());
        // A single edge hit is not an adequate planned scan of a full ROI.
        const int required=std::max(1,int((total+1)/2));
        if(validFrames<required||!d->plan.views[static_cast<std::size_t>(i)].planarCoverageEnvelopePassed||
           d->previewRoiCoverageFractions[i]<0.95||
           d->previewBoundaryCoverageFractions[i]<0.95)
            d->opticalPreviewUsable=false;
    }
    if(!d->opticalPreviewUsable)
        d->warning+=QStringLiteral("至少一个姿态的 ROI 网格/边界光学覆盖不足或命中率低于 50%，已禁止发布扫描 NC/模拟 SCAN。 ");
    if(d->timeOrderAnomalies)
        d->warning+=QStringLiteral("原始 SCAN 存在非单调时序，必须按原始 Profile/硬件锁存核查，不得简单使用名义间隔。 ");
    d->warning+=QStringLiteral("多姿态实验专用：不执行 M5/M6 的增量覆盖优化；手工 CAD 放置未经自动配准。"
        "光学模型在新 B/C 下未完成真实验证；SCAN 首帧姿态只是历史记录，不等于机床当前轴位。"
        "机器 FCL 部分覆盖不代表夹具/传感器外壳已全部完成碰撞认证。");
    if(!d->rotaryReplayExcited)
        d->warning+=QStringLiteral("本次 SCAN 的 BC 基本不动，无法验证物理模型的大角度回转预测。");
    return d;
}

MultiPoseOfflineNcRun MultiPoseExperimentQtAdapter::executeNcForPose(
    const MultiPoseQtReport& r,std::size_t index,int topology,int h3Edge,
    const std::string* exactNcText){
    MultiPoseOfflineNcRun result;
    if(!r.ok()||index>=r.plan.views.size()){
        result.error=QStringLiteral("没有有效的姿态规划");return result;
    }
    if(topology!=1&&topology!=3){result.error=QStringLiteral("NC 仿真仅支持 H1/H3");return result;}
    const auto& view=r.plan.views[index];
    if(exactNcText){result.text=*exactNcText;}
    else{
        M6NcPreviewOptions opt;
        opt.proposedTopologyId=topology;opt.h3R1Edge=h3Edge;
        opt.studyId="MULTIPOSE_ORIENTATION_STUDY";
        opt.scanGroupLabel="MP_"+view.id;
        opt.sourcePartPoseId=r.partPoseId.toStdString();
        opt.calibrationId=r.calibrationId.toStdString();
        opt.physical17Sha256=r.physical17Sha256.toStdString();
        opt.sourceCadPath=r.cadPath.toLocal8Bit().toStdString();
        opt.kinematicsSource=r.kinematicsSummary.toLocal8Bit().toStdString();
        const auto file=makeM6NcPreview(multiPoseAsOfflineM6(view,r.options),opt);
        if(!file.ok()){result.error=QString::fromStdString(file.error);return result;}
        result.text=file.text;
    }
    result.execution=simulateMultiPoseNc(result.text,r.options.maximumProfiles);
    if(!result.execution.ok()){
        result.error=QString::fromStdString(result.execution.error);return result;
    }
    if(result.execution.topology!=topology ||
       (topology==3&&result.execution.h3Edge!=h3Edge)){
        result.error=QStringLiteral("NC 硬件同步拓扑/锁存沿与当前选项不一致");return result;
    }
    const auto difference=verifyMultiPoseNcAgainstPlan(result.execution,multiPoseAsOfflineM6(view,r.options));
    if(!difference.empty()){
        result.error=QStringLiteral("G代码逐段仿真与规划不一致：")+QString::fromStdString(difference);
    }
    return result;
}
MultiPoseOfflineNcRun MultiPoseExperimentQtAdapter::executeConnectedNc(
    const MultiPoseQtReport& r,int topology,int h3Edge,
    const std::string* exactNcText){
    MultiPoseOfflineNcRun result;
    if(!r.ok()||!r.connectedSession.ok()){
        result.error=QStringLiteral("独立姿态没有通过连续转场检查：")+r.continuousSessionIssue;
        return result;
    }
    if(topology!=1&&topology!=3){result.error=QStringLiteral("连续仿真只支持 H1/H3");return result;}
    M6NcPreviewOptions opts;
    opts.proposedTopologyId=topology;opts.h3R1Edge=h3Edge;
    opts.studyId="MULTIPOSE_CONNECTED_OFFLINE_ONLY";
    opts.scanGroupLabel="MP_SESSION";
    opts.sourcePartPoseId=r.partPoseId.toStdString();
    opts.calibrationId=r.calibrationId.toStdString();
    opts.physical17Sha256=r.physical17Sha256.toStdString();
    opts.sourceCadPath=r.cadPath.toLocal8Bit().toStdString();
    opts.kinematicsSource=r.kinematicsSummary.toLocal8Bit().toStdString();
    const auto generated=makeM6NcPreview(r.connectedSession.merged,opts);
    if(!generated.ok()){
        result.error=QStringLiteral("连接 G代码生成失败：")+QString::fromStdString(generated.error);
        return result;
    }
    result.text=exactNcText?*exactNcText:generated.text;
    result.execution=simulateMultiPoseNc(result.text,r.options.maximumProfiles);
    if(!result.execution.ok()){
        result.error=QStringLiteral("连接 G代码重新解析失败：")+
                     QString::fromStdString(result.execution.error);
        return result;
    }
    const auto& nc=result.execution;
    if(nc.topology!=topology||(topology==3&&nc.h3Edge!=h3Edge)){
        result.error=QStringLiteral("连接 G代码 H1/H3 或 R1 沿与选项不一致");return result;
    }
    const auto mismatch=verifyMultiPoseNcAgainstPlan(nc,r.connectedSession.merged);
    if(!mismatch.empty()){
        result.error=QStringLiteral("连接 NC 的 XYZCB / G93 / M50/M51 与规划不一致：")+
                     QString::fromStdString(mismatch);
        return result;
    }
    const std::size_t plannedGroups=std::accumulate(r.plan.views.begin(),r.plan.views.end(),
        std::size_t(0),[](std::size_t n,const ExperimentViewReport& view){return n+view.scanGroups;});
    if(nc.scanGroups!=plannedGroups){
        result.error=QStringLiteral("连接 NC 遗失一条扫描带的采集组");return result;
    }
    const auto timeline=makeConnectedNcPlaybackTimeline(r.connectedSession,nc);
    if(!timeline.ok()){
        result.error=QStringLiteral("连接 NC 时间轴验证失败：")+QString::fromStdString(timeline.error);
    }
    return result;
}
MultiPosePlaybackState MultiPoseExperimentQtAdapter::replayConnectedNcAt(
    const MultiPoseQtReport& report,const NcSimResult& nc,
    const MultiPosePlaybackTimeline& timeline,double time){
    mls::calibration::Physical17Parameters p;p.value=report.physicalParameters;
    const Transform anchor=report.physical17Anchor;
    return sampleConnectedNcPlayback(nc,timeline,time,
        [p,anchor](const Joint5& q){return anchor*physFk(q,p);});
}
MultiPosePlaybackState MultiPoseExperimentQtAdapter::replayStateAt(
    const MultiPoseQtReport& report,const std::vector<NcSimResult>& executedNc,
    const MultiPosePlaybackTimeline& timeline,double time){
    mls::calibration::Physical17Parameters p;p.value=report.physicalParameters;
    const Transform anchor=report.physical17Anchor;
    return sampleNcDrivenPlayback(executedNc,timeline,time,
        [p,anchor](const Joint5& q){return anchor*physFk(q,p);});
}

QString MultiPoseExperimentQtAdapter::exportOfflineNcAndManifest(
    const MultiPoseQtReport& r,const QString& directory,int h3Edge,
    const std::vector<std::string>* selectedNc,int selectedTopology,
    const std::string* verifiedConnectedNc){
    if(!r.ok())return QStringLiteral("没有有效的离线多姿态规划");
    if(!r.opticalPreviewUsable)return QStringLiteral("至少一个姿态 ROI/边界覆盖未通过，禁止发布扫描 NC");
    if(h3Edge!=114&&h3Edge!=115)return QStringLiteral("H3 R1 锁存沿只允许 114/115");
    if(!QDir().mkpath(directory))return QStringLiteral("无法创建目录：")+directory;
    auto root=metadata(r);
    QJsonArray tasks;
    for(std::size_t i=0;i<r.plan.views.size();++i){
        const auto& view=r.plan.views[i];
        QJsonObject rec;
        rec.insert("poseId",QString::fromStdString(view.id));
        rec.insert("C_deg",view.cDeg);
        rec.insert("B_deg",view.bDeg);
        rec.insert("machineXDescending",view.machineXDescending);
        rec.insert("geometricCoverageEnvelopePassed",view.planarCoverageEnvelopePassed);
        rec.insert("resolvedSweepAxis",view.resolvedAxis==ExperimentSweepAxis::CadX?"CAD_X":"CAD_Y");
        rec.insert("scanStartCad",view.scanStartCad);
        rec.insert("scanEndCad",view.scanEndCad);
        rec.insert("laserFanMinAlongCadMm",view.projectedLaserScanMinMm);
        rec.insert("laserFanMaxAlongCadMm",view.projectedLaserScanMaxMm);
        rec.insert("laserFanLateralWidthMm",view.predictedLateralCoverageWidthMm);
        rec.insert("leadInMm",view.beforePartMarginMm);
        rec.insert("leadOutMm",view.afterPartMarginMm);
        rec.insert("nominalProfiles",static_cast<qint64>(view.plannedProfiles));
        rec.insert("scanGroups",static_cast<qint64>(view.scanGroups));
        rec.insert("nominalDurationSeconds",view.nominalDurationSec);
        rec.insert("nominalProfilePitchMm",view.predictedProfilePitchMm);
        rec.insert("previewWorkpieceHitProfiles",r.previewProfilesWithWorkpieceHit.at(qsizetype(i)));
        rec.insert("previewRoiCoverageFraction",r.previewRoiCoverageFractions.at(qsizetype(i)));
        rec.insert("previewBoundaryCoverageFraction",r.previewBoundaryCoverageFractions.at(qsizetype(i)));
        rec.insert("previewCoverageWarning",r.previewCoverageWarnings.at(qsizetype(i)));
        QJsonArray ncFiles;
        for(int topo:{1,3}){
            M6NcPreviewOptions opt;
            opt.proposedTopologyId=topo;opt.h3R1Edge=h3Edge;
            opt.studyId="MULTIPOSE_ORIENTATION_STUDY";
            opt.scanGroupLabel="MP_"+view.id;
            opt.sourcePartPoseId=r.partPoseId.toStdString();
            opt.calibrationId=r.calibrationId.toStdString();
            opt.physical17Sha256=r.physical17Sha256.toStdString();
            opt.sourceCadPath=r.cadPath.toLocal8Bit().toStdString();
            opt.kinematicsSource=r.kinematicsSummary.toLocal8Bit().toStdString();
            const auto file=makeM6NcPreview(multiPoseAsOfflineM6(view,r.options),opt);
            if(!file.ok())return QStringLiteral("NC 生成失败：")+QString::fromStdString(file.error);
            // Preserve the SAME already-verified NC bytes used for playback
            // when the user explicitly imports a semantically equivalent file.
            const std::string* sourceText=&file.text;
            if(topo==selectedTopology && selectedNc && selectedNc->size()==r.plan.views.size())
                sourceText=&selectedNc->at(i);
            // Actually execute the generated NC text offline and compare EVERY
            // XYZCB / G93 time / S0-S1 segment against the planner. A syntactic
            // parse alone would not reveal a changed coordinate or missing M50.
            const auto ncRun=executeNcForPose(r,i,topo,h3Edge,sourceText);
            if(!ncRun.ok())return QStringLiteral("导出前 NC 数字孪生执行校验失败：")+ncRun.error;
            const QString name=QStringLiteral("MP_%1_OFFLINE_ONLY_%2.nc")
                .arg(QString::fromStdString(view.id)).arg(topo==1?"H1":"H3");
            QString why;
            const auto ncPath=QDir(directory).filePath(name);
            const QByteArray plannedBytes=QByteArray::fromStdString(*sourceText);
            if(const auto parseError=verifyGCodeImportSyntax(plannedBytes,name);!parseError.isEmpty())return parseError;
            if(!writeAtomic(ncPath,plannedBytes,&why))return why;
            QFile actualNc(ncPath);
            if(!actualNc.open(QIODevice::ReadOnly))return QStringLiteral("NC 回读失败：")+ncPath;
            const auto actualBytes=actualNc.readAll();
            if(const auto parseError=verifyGCodeImportSyntax(actualBytes,name);!parseError.isEmpty())return parseError;
            const std::string saved(actualBytes.constData(),std::size_t(actualBytes.size()));
            const auto savedRun=executeNcForPose(r,i,topo,h3Edge,&saved);
            if(!savedRun.ok())return QStringLiteral("已保存 NC 回读仿真失败：")+savedRun.error;
            QJsonObject f;f.insert("name",name);f.insert("H",topo);f.insert("sha256",QString::fromLatin1(fileSha256(ncPath)));
            f.insert("offlineNcSimulator", "MLS_NC_TEXT_DRIVEN_V1");
            f.insert("ncGeometryAndTimingComparedToPlanner",true);
            f.insert("nominalSimulatedProfiles",static_cast<qint64>(savedRun.execution.profiles.size()));
            f.insert("ncSimSeconds",savedRun.execution.durationSeconds);
            f.insert("ncSimScanGroups",static_cast<qint64>(savedRun.execution.scanGroups));
            f.insert("unknownInitialG00Approach",savedRun.execution.initialG00Unverified);
            f.insert("offlineOnly",true);ncFiles.append(f);
        }
        rec.insert("ncFiles",ncFiles);
        tasks.append(rec);
    }
    root.insert("poses",tasks);
    // Export BOTH real hardware topology alternatives as independently checked
    // offline sessions. Do NOT convert H3 into H1 by editing header comments.
    // Rate/timing still originate from the same explicitly configured experiment.
    // Live hardware H1/200 and NC H3/100 are NOT interchangeable; user must
    // re-plan at the approved rate before either output is considered for field use.
    if(verifiedConnectedNc){
        if(!r.connectedSession.ok())
            return QStringLiteral("连续实验 NC 尚未通过独立转场碰撞检查");
        QJsonArray connectedSessions;
        for(int topo:{1,3}){
            const std::string* selectedText=topo==selectedTopology?verifiedConnectedNc:nullptr;
            const auto run=executeConnectedNc(r,topo,h3Edge,selectedText);
            if(!run.ok())
                return QStringLiteral("H%1 连续 NC 重新生成/解释失败：%2")
                    .arg(topo).arg(run.error);
            const QString sessionName=QStringLiteral("MP_ALL_POSES_CONNECTED_OFFLINE_ONLY_H%1.nc")
                .arg(topo);
            const QString sessionPath=QDir(directory).filePath(sessionName);
            const QByteArray plannedBytes=QByteArray::fromStdString(run.text);
            if(const auto parseError=verifyGCodeImportSyntax(plannedBytes,sessionName);!parseError.isEmpty())return parseError;
            QString why;
            if(!writeAtomic(sessionPath,plannedBytes,&why))return why;
            QFile input(sessionPath);
            if(!input.open(QIODevice::ReadOnly))
                return QStringLiteral("连续 H%1 NC 保存后读取失败").arg(topo);
            const QByteArray bytes=input.readAll();
            if(const auto parseError=verifyGCodeImportSyntax(bytes,sessionName);!parseError.isEmpty())return parseError;
            const std::string saved(bytes.constData(),std::size_t(bytes.size()));
            const auto savedRun=executeConnectedNc(r,topo,h3Edge,&saved);
            if(!savedRun.ok())
                return QStringLiteral("连续 H%1 NC 保存后重新执行验证失败：%2")
                    .arg(topo).arg(savedRun.error);
            const auto& replay=savedRun.execution;
            QJsonObject joined;
            joined.insert("filename",sessionName);
            joined.insert("sha256",QString::fromLatin1(fileSha256(sessionPath)));
            joined.insert("hardwareTopology",topo);
            joined.insert("hardwareFrequencyHz",replay.profileRateHz);
            joined.insert("hardwareCompatibleOnlyWhenSiteMatches",true);
            joined.insert("laserOffPoseConnections",static_cast<int>(r.plan.views.size()-1));
            joined.insert("numberOfScanGroups",static_cast<qint64>(replay.scanGroups));
            joined.insert("simulatedDurationSeconds",replay.durationSeconds);
            QJsonArray poseClock;
            QJsonArray connectors;
            for(std::size_t k=0;k<r.plan.views.size();++k){
                QJsonObject sec;
                sec.insert("poseId",QString::fromStdString(r.plan.views[k].id));
                sec.insert("ncStartSeconds",r.connectedSession.poseStartSeconds[k]);
                sec.insert("ncEndSeconds",r.connectedSession.poseEndSeconds[k]);
                poseClock.append(sec);
                if(k>0){
                    QJsonObject link;
                    link.insert("fromPose",QString::fromStdString(r.plan.views[k-1].id));
                    link.insert("toPose",QString::fromStdString(r.plan.views[k].id));
                    link.insert("laserOn",false);
                    link.insert("ncStartSeconds",r.connectedSession.poseEndSeconds[k-1]);
                    link.insert("ncEndSeconds",r.connectedSession.poseStartSeconds[k]);
                    connectors.append(link);
                }
            }
            joined.insert("poseSections",poseClock);
            joined.insert("verifiedLaserOffConnections",connectors);
            joined.insert("offlineOnly",true);
            joined.insert("firstG00StartUnverified",true);
            joined.insert("interPoseTransfersFclSamplesValidated",true);
            connectedSessions.append(joined);
            if(topo==selectedTopology)
                root.insert("continuousNcSession",joined); // legacy readers
        }
        root.insert("continuousNcSessions",connectedSessions);
    }else{
        root.insert("continuousNcSession",QStringLiteral("NOT_GENERATED: do not concatenate independent NC sessions"));
    }
    root.insert("gcodeContract","H1=ECI OP14/REGIST104; H3=sensor CMM RS422/REGIST114-115; H2 retired");
    root.insert("verificationScope","NC text re-parsed, ideal G93 joint motion and M50/M51 timing replayed, compared to planner; NOT actual controller interpolation");
    QJsonObject automatic;
    automatic.insert("physical17ArchivedFkReplayConsistent",r.physical17ReplayConsistent);
    automatic.insert("connectedNcOfflineReplayChecked",r.connectedSession.ok());
    automatic.insert("importedModelFingerprintBoundToNc",true);
    automatic.insert("independentSixDofRegistrationAccepted",false);
    automatic.insert("onsiteHardwareAndCollisionQualified",false);
    root.insert("automaticVerification",automatic);
    root.insert("M50M51Meaning","acquisition lifetime; NEVER sufficient to authorize electrical triggers");
    root.insert("note","NC exports are offline previews for M3/M4 experiment design, not approved for machine execution");
    QString error;
    if(!writeAtomic(QDir(directory).filePath(QStringLiteral("multi_pose_experiment_manifest.json")),
        QJsonDocument(root).toJson(QJsonDocument::Indented),&error))return error;
    return {};
}

QString MultiPoseExperimentQtAdapter::exportSimulatedProfilesForM3M4(
    const MultiPoseQtReport& r,const TwinScene& scene,const Sensor2950Geometry& optics,
    const QString& directory,const std::vector<std::string>& exactNcTexts,
    int topology,const std::atomic<bool>* cancel,
    std::atomic<std::size_t>* progress,MultiPoseSimArtifacts* artifacts){
    if(!r.ok())return QStringLiteral("没有有效的多姿态实验规划");
    if(!r.opticalPreviewUsable)return QStringLiteral("至少一个姿态 ROI/边界覆盖未通过，禁止发布模拟 SCAN");
    if(exactNcTexts.size()!=r.plan.views.size())return QStringLiteral("没有对应每个姿态的 G 代码文本");
    if(topology!=1&&topology!=3)return QStringLiteral("模拟只支持 H1/H3 同步拓扑");
    if(!QDir().mkpath(directory))return QStringLiteral("无法创建仿真目录");
    if(artifacts){artifacts->poses.clear();artifacts->directory=directory;artifacts->manifestPath.clear();}
    VirtualLaser2950 simulator(scene,optics,true);
    std::string why;
    if(!simulator.ready(&why))return QString::fromStdString(why);
    QJsonArray poses;
    for(std::size_t k=0;k<r.plan.views.size();++k){
        const auto& view=r.plan.views[k];
        // The virtual scanner MUST consume the NC interpreter's trigger poses,
        // never a second idealized planning exposure array.
        const auto& exactNc=exactNcTexts[k];
        const auto sourceNc=simulateMultiPoseNc(exactNc,r.options.maximumProfiles);
        if(!sourceNc.ok())return QStringLiteral("NC 模拟执行失败：")+QString::fromStdString(sourceNc.error);
        if(sourceNc.topology!=topology)return QStringLiteral("所选 NC 的 H1/H3 拓扑不匹配");
        const auto discrepancy=verifyMultiPoseNcAgainstPlan(sourceNc,multiPoseAsOfflineM6(view,r.options));
        if(!discrepancy.empty())return QStringLiteral("G代码与规划不同，不生成虚拟点云：")+QString::fromStdString(discrepancy);
        if(cancel&&cancel->load())return QStringLiteral("已取消；未生成完成标记，已有中间文件不可作为实验数据");
        const QString sub=QDir(directory).filePath(QString::fromStdString(view.id));
        if(!QDir().mkpath(sub))return QStringLiteral("无法创建仿真子目录");
        const QString ncFile=QDir(sub).filePath(QStringLiteral("SIMULATION_INPUT_H%1_OFFLINE_ONLY.nc").arg(topology));
        QString ncSaveError;
        if(!writeAtomic(ncFile,QByteArray::fromStdString(exactNc),&ncSaveError))return ncSaveError;
        const std::uint64_t session=std::uint64_t(QDateTime::currentMSecsSinceEpoch())+k+1;
        M6ScanVolumeWriter archive(sub,r.options.profileRateHz,r.physicalParameters,session);
        MultiPoseSimPoseArtifacts poseOut;
        poseOut.poseId=QString::fromStdString(view.id);
        poseOut.ncFilePath=ncFile;
        poseOut.ncSha256=QString::fromLatin1(fileSha256(ncFile));
        const QString plyPath=QDir(sub).filePath("SIMULATION_ONLY_workpiece.ply");
        QTemporaryFile raw(QDir(sub).filePath("sim_cloud_XXXXXX.tmp"));
        if(!raw.open())return QStringLiteral("无法创建仿真 PLY 中间文件：")+raw.errorString();
        QDataStream binary(&raw);
        binary.setByteOrder(QDataStream::LittleEndian);
        binary.setFloatingPointPrecision(QDataStream::DoublePrecision);
        const std::size_t replayStride=std::max<std::size_t>(1,(sourceNc.profiles.size()+239)/240);
        std::uint64_t workpieceCount=0;
        for(std::size_t frame=0;frame<sourceNc.profiles.size();++frame){
            if(cancel&&cancel->load())return QStringLiteral("模拟采集已取消；不发布完整 manifest");
            const auto& exposure=sourceNc.profiles[frame];
            VirtualProfilePose pose;
            pose.elapsedSeconds=exposure.seconds;
            pose.sourceOrdinal=static_cast<std::uint64_t>(frame);
            pose.joints=exposure.joints;
            mls::calibration::Physical17Parameters physical;physical.value=r.physicalParameters;
            pose.sensorToMachine=r.physical17Anchor*physFk(exposure.joints,physical);
            const auto profile=simulator.simulate(pose);
            if(!archive.append(profile,1,&why))return QString::fromStdString(why);
            const bool replayFrame=(frame%replayStride==0||frame+1==sourceNc.profiles.size());
            QVector<QVector3D> captured;
            for(std::size_t j=0;j<profile.samples.size();++j){
                const auto& sample=profile.samples[j];
                if(sample.state!=VirtualSampleState::Valid||sample.hitObject!=VirtualHitObject::Workpiece)continue;
                const Vec3 x=sample.hitPositionMachine;
                binary<<double(x.x)<<double(x.y)<<double(x.z);
                ++workpieceCount;
                if(replayFrame&&j%8==0){
                    const QVector3D pt{float(x.x),float(x.y),float(x.z)};
                    captured.append(pt);
                    poseOut.comparisonPreview.append(pt);
                }
            }
            if(replayFrame){
                poseOut.replayTimes.append(pose.elapsedSeconds);
                poseOut.replayFrames.append(std::move(captured));
            }
            if(binary.status()!=QDataStream::Ok)return QStringLiteral("模拟 PLY 写入失败");
            if(progress)++(*progress);
        }
        if(!archive.finish(&why))return QString::fromStdString(why);
        if(!raw.flush()||!raw.seek(0))return QStringLiteral("模拟 PLY 缓存不可读");
        QSaveFile ply(plyPath);
        if(!ply.open(QIODevice::WriteOnly))return QStringLiteral("无法创建仿真 PLY：")+ply.errorString();
        const QByteArray header=QByteArray("ply\nformat binary_little_endian 1.0\n")
          +"comment GENERATED_BY_MLS_OFFLINE_M3; NOT REAL DATA; NOT METROLOGY GROUND TRUTH\n"
          +"element vertex "+QByteArray::number(workpieceCount)+"\n"
          +"property double x\nproperty double y\nproperty double z\nend_header\n";
        if(ply.write(header)!=header.size())return QStringLiteral("PLY 文件头写入失败");
        while(!raw.atEnd()){
            const QByteArray chunk=raw.read(1024*1024);
            if(chunk.isEmpty()&&raw.error()!=QFileDevice::NoError)return QStringLiteral("PLY 数据读取失败");
            if(ply.write(chunk)!=chunk.size())return QStringLiteral("PLY 数据写入失败");
        }
        if(!ply.commit())return QStringLiteral("PLY 保存失败：")+ply.errorString();
        poseOut.plyPath=plyPath;
        poseOut.validWorkpiecePoints=workpieceCount;
        poseOut.profileCount=archive.profileCount();
        QJsonObject v;v.insert("poseId",QString::fromStdString(view.id));
        v.insert("simulationOnly",true);v.insert("C_B_deg",QJsonArray{view.cDeg,view.bDeg});
        v.insert("profiles",static_cast<qint64>(archive.profileCount()));
        v.insert("ncSourceFile",QDir(directory).relativeFilePath(ncFile));
        v.insert("ncSourceSha256",poseOut.ncSha256);
        v.insert("ncTextInterpreter", "MLS_NC_TEXT_DRIVEN_V1");
        v.insert("ncNominalExecutionSeconds",sourceNc.durationSeconds);
        v.insert("ncM50M51Groups",static_cast<qint64>(sourceNc.scanGroups));
        v.insert("initialG00UnknownPhysicalApproach",true);
        v.insert("validWorkpieceSamples",static_cast<qint64>(workpieceCount));
        v.insert("plyFile",QDir(directory).relativeFilePath(plyPath));
        v.insert("plySha256",QString::fromLatin1(fileSha256(plyPath)));
        v.insert("replayPreviewStrideProfiles",static_cast<qint64>(replayStride));
        v.insert("replayPreviewPointStride",8);
        QJsonArray files;
        for(const auto& volume:archive.volumes()){
            QJsonObject f;f.insert("file",QDir(directory).relativeFilePath(volume.temporaryPath));
            f.insert("sha256",QString::fromLatin1(fileSha256(volume.temporaryPath)));
            f.insert("profiles",static_cast<qint64>(volume.profiles));
            f.insert("validSamples",static_cast<qint64>(volume.validPoints));
            files.append(f);
        }
        v.insert("scanV3Volumes",files);poses.append(v);
        if(artifacts)artifacts->poses.append(std::move(poseOut));
    }
    // Concatenate independent pose PLY vertices without resampling or merging.
    // Their separate per-pose archives and ID map remain authoritative.
    const QString allPly=QDir(directory).filePath("ALL_POSES_SIMULATION_ONLY_UNFUSED.ply");
    std::uint64_t totalPoints=0;
    if(artifacts)for(const auto& v:artifacts->poses)totalPoints+=v.validWorkpiecePoints;
    // Full export without a result sink remains supported: sum the archive
    // JSON pose records already constructed in this transaction.
    else for(const auto& j:poses)totalPoints+=std::uint64_t(j.toObject().value("validWorkpieceSamples").toVariant().toULongLong());
    QSaveFile all(allPly);
    if(!all.open(QIODevice::WriteOnly))return QStringLiteral("全姿态 PLY 无法写入");
    const QByteArray allHeader=QByteArray("ply\nformat binary_little_endian 1.0\n")
      +"comment SIMULATION_ONLY: unfused concatenation of independent poses; NOT real measurements\n"
      +"element vertex "+QByteArray::number(totalPoints)+"\n"
      +"property double x\nproperty double y\nproperty double z\nend_header\n";
    if(all.write(allHeader)!=allHeader.size())return QStringLiteral("全姿态 PLY 头部写入失败");
    for(const auto& entry:poses){
        const QString src=QDir(directory).filePath(entry.toObject().value("plyFile").toString());
        QFile f(src);
        if(!f.open(QIODevice::ReadOnly))return QStringLiteral("无法合并姿态 PLY：")+src;
        bool headerEnd=false;
        while(!f.atEnd()){
            const QByteArray line=f.readLine(4096);
            if(line.trimmed()=="end_header"){headerEnd=true;break;}
        }
        if(!headerEnd)return QStringLiteral("姿态 PLY 缺少 end_header");
        while(!f.atEnd()){
            if(cancel&&cancel->load())return QStringLiteral("全姿态 PLY 合并已取消");
            const QByteArray bytes=f.read(1024*1024);
            if(all.write(bytes)!=bytes.size())return QStringLiteral("全姿态 PLY 写入失败");
        }
    }
    if(!all.commit())return QStringLiteral("全姿态 PLY 保存失败");
    QJsonObject obj=metadata(r);
    obj.insert("allPosesUnfusedPly",QDir(directory).relativeFilePath(allPly));
    obj.insert("allPosesUnfusedPlySha256",QString::fromLatin1(fileSha256(allPly)));
    obj.insert("allPosesUnfusedPlyVertices",static_cast<qint64>(totalPoints));
    if(artifacts)artifacts->allPosesPlyPath=allPly;
    obj.insert("artifactType","M3_M4_SIMULATED_PROFILES_AND_WORKPIECE_PLY_NOT_REAL");
    obj.insert("simulationDrivenBy", "EXACT_SAVED_OFFLINE_GCODE_TEXT_REPARSED_AS_G93_XYZCB");
    obj.insert("hardwareTopologyNominal",topology);
    obj.insert("hardwareTriggerLatencyModeled",false);
    obj.insert("PoseTiming","Simulated");
    obj.insert("laserModel","M3 geometric ray-trace; multi-angle optics NOT independently calibrated");
    obj.insert("pointsAreCADPredictions",true);
    obj.insert("poses",poses);
    QString err;
    const QString manifest=QDir(directory).filePath("M3M4_SIMULATED_ONLY_manifest.json");
    if(!writeAtomic(manifest,QJsonDocument(obj).toJson(QJsonDocument::Indented),&err))return err;
    if(artifacts)artifacts->manifestPath=manifest;
    return {};
}

MultiPoseCompareArtifacts MultiPoseExperimentQtAdapter::compareRealScanForPose(
    const MultiPoseQtReport& plan,const MultiPoseSimArtifacts& sim,
    std::size_t poseIndex,const QString& realFile,double toleranceMm){
    MultiPoseCompareArtifacts result;
    if(poseIndex>=std::size_t(sim.poses.size())||poseIndex>=plan.plan.views.size()){
        result.error=QStringLiteral("未找到对应姿态的完整仿真点云，请先生成虚拟 SCAN/PLY");return result;
    }
    const auto& pose=plan.plan.views[poseIndex];
    result.poseId=QString::fromStdString(pose.id);result.realScanPath=realFile;
    auto cloud=ScanReplayAdapter::load(realFile,90000);
    if(!cloud||!cloud->ok()){result.error=cloud?cloud->error:QStringLiteral("真实 SCAN 读取失败");return result;}
    // Rejected simulation archive must NEVER be scored as independent real evidence.
    for(const auto& profile:cloud->scan.profiles){
        if(profile.fiveAxis.frame.timing==mls::runtime::PoseTiming::Simulated){
            result.error=QStringLiteral("此文件是模拟 SCAN，不能冒充实测数据进行独立对比");return result;
        }
    }
    const auto& q0=cloud->scan.profiles.first().fiveAxis.frame.joints;
    bool orientationStable=true;
    for(const auto& prof:cloud->scan.profiles){
        const auto& recorded=prof.fiveAxis.frame.joints;
        if(!std::isfinite(recorded[3])||!std::isfinite(recorded[4])||
           std::abs(recorded[3]-pose.cDeg)>.5||std::abs(recorded[4]-pose.bDeg)>.5){
            orientationStable=false;break;
        }
    }
    if(!orientationStable){
        result.error=QStringLiteral("实测 SCAN 历史 C/B=(%1,%2)，与所选姿态 %3=(%4,%5) 不符；禁止错位姿对比")
           .arg(q0[3],0,'f',3).arg(q0[4],0,'f',3).arg(result.poseId)
           .arg(pose.cDeg,0,'f',3).arg(pose.bDeg,0,'f',3);
        return result;
    }
    const auto inverse=Transform::fromPose(plan.cadPose).inverseRigid();
    const auto& roi=plan.options.roiCad;
    std::vector<Vec3> actual,synth;
    actual.reserve(std::size_t(cloud->previewPoints.size()));
    // Common CAD ROI is essential: unrelated fixture and out-of-area real
    // observations must not dominate the nearest-neighbour distribution.
    for(const auto& p:cloud->previewPoints){
        const Vec3 x{p.x(),p.y(),p.z()};
        const Vec3 cad=inverse.point(x);
        if(cad.x<roi.min.x-2||cad.x>roi.max.x+2||
           cad.y<roi.min.y-2||cad.y>roi.max.y+2||
           std::abs(cad.z-plan.options.surfaceZCadMm)>6)continue;
        actual.push_back(x);result.realPoints.append(p);
    }
    for(const auto& p:sim.poses[qsizetype(poseIndex)].comparisonPreview)
        synth.push_back({p.x(),p.y(),p.z()});
    result.metrics=compareMultiPoseClouds(actual,synth,toleranceMm,120000);
    if(!result.metrics.valid){result.error=QString::fromStdString(result.metrics.error);return result;}
    result.realDistancesMm.reserve(qsizetype(result.metrics.realDistancesMm.size()));
    for(float x:result.metrics.realDistancesMm)result.realDistancesMm.append(x);
    result.description=QStringLiteral(
        "%1 实测→仿真中位=%2 mm, P95=%3 mm, <%4 mm比例=%5%；"
        "仿真→实测中位=%6 mm, P95=%7 mm, <%4 mm比例=%8%。\n"
        "注意：只评价共用机床坐标下可见几何的近邻差异；未重新配准，"
        "不代表设备真实测量精度；CAD 位姿仍 registrationVerified=false。")
        .arg(result.poseId)
        .arg(result.metrics.realToSimMedianMm,0,'f',4)
        .arg(result.metrics.realToSimP95Mm,0,'f',4)
        .arg(toleranceMm,0,'f',3)
        .arg(result.metrics.realWithinTolerance*100,0,'f',1)
        .arg(result.metrics.simToRealMedianMm,0,'f',4)
        .arg(result.metrics.simToRealP95Mm,0,'f',4)
        .arg(result.metrics.simWithinTolerance*100,0,'f',1);
    QJsonObject manifest;
    manifest.insert("schema","MLS_MULTIPOSE_REAL_SIM_COMPARE_V1_DIAGNOSTIC_ONLY");
    manifest.insert("poseId",result.poseId);
    manifest.insert("realScanPath",realFile);
    manifest.insert("realScanSha256",QString::fromLatin1(fileSha256(realFile)));
    manifest.insert("simPlyPath",sim.poses[qsizetype(poseIndex)].plyPath);
    manifest.insert("simPlySha256",QString::fromLatin1(fileSha256(sim.poses[qsizetype(poseIndex)].plyPath)));
    manifest.insert("simulationNcPath",sim.poses[qsizetype(poseIndex)].ncFilePath);
    manifest.insert("simulationNcSha256",sim.poses[qsizetype(poseIndex)].ncSha256);
    manifest.insert("realSimComparisonUsesOriginalMachineFrame",true);
    manifest.insert("registrationVerified",false);
    manifest.insert("realPreviewPointsWithinCadRoi",static_cast<qint64>(result.metrics.realCount));
    manifest.insert("simPreviewPoints",static_cast<qint64>(result.metrics.syntheticCount));
    manifest.insert("distanceToleranceMm",toleranceMm);
    manifest.insert("realToSimMedianMm",result.metrics.realToSimMedianMm);
    manifest.insert("realToSimP95Mm",result.metrics.realToSimP95Mm);
    manifest.insert("simToRealMedianMm",result.metrics.simToRealMedianMm);
    manifest.insert("simToRealP95Mm",result.metrics.simToRealP95Mm);
    manifest.insert("realWithinTolerance",result.metrics.realWithinTolerance);
    manifest.insert("simWithinTolerance",result.metrics.simWithinTolerance);
    manifest.insert("warning","Offline fixed-coordinate nearest-neighbour analysis; not metrology accuracy or independent registration proof");
    QString err;
    if(!writeAtomic(QDir(sim.directory).filePath("comparison_"+result.poseId+".json"),
       QJsonDocument(manifest).toJson(QJsonDocument::Indented),&err))
        result.error=err;
    return result;
}
} // namespace mls::twin
