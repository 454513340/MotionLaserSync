#include "twin/TwinM52Adapter.h"
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSaveFile>
#include <QLocale>
#include <cmath>
#include <exception>

namespace mls::twin {
namespace {
bool samePose(const VirtualProfilePose& v,const mls::runtime::ArchiveProfile& real){
    if(v.sourceOrdinal!=real.frame.ordinal)return false;
    for(std::size_t j=0;j<16;++j)
        if(std::abs(v.sensorToMachine.m[j]-real.frame.sensorToMachine[j])>1e-8)return false;
    return true;
}
QJsonValue number(double d){return std::isfinite(d)?QJsonValue(d):QJsonValue(QJsonValue::Null);}
QString fixed(double d,int digits=7){return std::isfinite(d)?QLocale::c().toString(d,'f',digits):QString();}
QJsonObject jsonFit(const M52Fit& r){
    return QJsonObject{
        {"ok",r.ok},{"error",QString::fromStdString(r.error)},
        {"pointsSeen",double(r.pointsSeen)},{"topCellsUsed",int(r.topCellsUsed)},
        {"conePointsUsed",int(r.conePointsUsed)},
        {"angularBinsUsed",int(r.angularBinsUsed)},
        {"trainBins",int(r.trainBins)},{"validationBins",int(r.validationBins)},
        {"maxUnobservedArcDeg",int(r.maxUnobservedArcDeg)},
        {"topSlopeX",number(r.topSlopeX)},{"topSlopeY",number(r.topSlopeY)},
        {"topInterceptMm",number(r.topInterceptMm)},
        {"topOffsetAtHoleMm",number(r.topOffsetAtHoleMm)},
        {"topPlaneMadMm",number(r.topPlaneMadMm)},
        {"candidateCenterShiftXmm",number(r.candidateCenterShiftXmm)},
        {"candidateCenterShiftYmm",number(r.candidateCenterShiftYmm)},
        {"coneRawMedianMm",number(r.coneRawMedianMm)},
        {"coneAfterTopMedianMm",number(r.coneAfterTopMedianMm)},
        {"coneAfterXYMedianMm",number(r.coneAfterXYMedianMm)},
        {"coneAfterXYMadMm",number(r.coneAfterXYMadMm)},
        {"coneInterceptMm",number(r.coneInterceptMm)},
        {"coneFitTrainMadMm",number(r.coneFitTrainMadMm)},
        {"coneFitValidationMadMm",number(r.coneFitValidationMadMm)},
        {"holdoutEvaluated",r.holdoutEvaluated},
        {"enoughAngularSupport",r.enoughAngularSupport},
        {"canApplyAutomatically",false}
    };
}
QString writeAtomic(const QString& path,const QByteArray& bytes){
    QSaveFile out(path);
    if(!out.open(QIODevice::WriteOnly))return out.errorString();
    if(out.write(bytes)!=bytes.size()){
        out.cancelWriting();return QStringLiteral("M5.2 写入不完整：")+path;
    }
    if(!out.commit())return out.errorString();
    return {};
}
}
std::shared_ptr<TwinM52Data> TwinM52Adapter::analyze(
    const ScanReplayData& real,const TwinSimulationData* virt,
    const TwinScene& scene,const M52Options& options,
    const std::atomic<bool>* cancelled,std::atomic<std::size_t>* progress){
    auto out=std::make_shared<TwinM52Data>();
    out->sourceScanPath=real.sourcePath;
    out->sourceCadPath=QString::fromLocal8Bit(scene.mesh().source.c_str());
    if(!real.ok()||!scene.hasWorkpiece()){
        out->message=QStringLiteral("M5.2 需要完整 H1 STEP 和 M2 实测 SCAN");return out;
    }
    if(virt&&!virt->ok()){
        out->message=QStringLiteral("M5.2 M3 虚拟数据无效");return out;
    }
    const auto& profiles=real.scan.profiles;
    const auto* virtualProfiles=virt?&virt->scan.profiles:nullptr;
    if(virtualProfiles&&virtualProfiles->size()!=std::size_t(profiles.size())){
        out->message=QStringLiteral("M5.2 M2/M3 帧数不一致");return out;
    }
    TwinM52GeometryAnalyzer analyzer(scene,options);
    if(!analyzer.ready()){
        out->message=QString::fromStdString(analyzer.error());return out;
    }
    const auto m2cad=scene.cadToMachineEstimate().inverseRigid();
    for(qsizetype i=0;i<profiles.size();++i){
        if(cancelled&&cancelled->load(std::memory_order_relaxed)){
            out->message=QStringLiteral("用户取消了 M5.2 分析");return out;
        }
        const auto& prof=profiles[i].fiveAxis;
        const Transform sensorToMachine{prof.frame.sensorToMachine};
        for(const auto& p:prof.points){
            if(std::isfinite(p.x)&&std::isfinite(p.z))
                analyzer.addRealCad(m2cad.point(sensorToMachine.point({p.x,0.,p.z})));
        }
        if(virtualProfiles){
            const auto& virtualFrame=(*virtualProfiles)[std::size_t(i)];
            if(!samePose(virtualFrame.pose,prof)){
                out->message=QStringLiteral("M5.2 M2/M3 曝光位姿不一致，第 %1 帧").arg(i);return out;
            }
            for(const auto& p:virtualFrame.samples){
                if(p.state==VirtualSampleState::Valid&&
                   p.hitObject==VirtualHitObject::Workpiece&&
                   std::isfinite(p.sensorX)&&std::isfinite(p.sensorZ))
                    analyzer.addVirtualCad(m2cad.point(sensorToMachine.point({p.sensorX,0.,p.sensorZ})));
            }
        }
        if(progress&&(i%32==0||i+1==profiles.size()))
            progress->store(std::size_t(i+1),std::memory_order_relaxed);
    }
    out->report=std::move(analyzer).finish();
    if(!out->report.ok())out->message=QString::fromStdString(out->report.error);
    return out;
}
QString TwinM52Adapter::exportReport(const TwinM52Data& data,const QString& outputJson){
    if(!data.ok())return QStringLiteral("M5.2 分析尚未完成");
    const auto& report=data.report;
    const auto& opt=report.options;
    QJsonArray residualAngles;
    for(int a=0;a<M52AngularBins;++a){
        const auto& r=report.real.angular[std::size_t(a)];
        const auto& v=report.virtualFit.angular[std::size_t(a)];
        residualAngles.append(QJsonObject{
            {"startDeg",r.angleStartDeg},{"endDeg",r.angleStartDeg+5},
            {"realHits",int(r.hits)},{"virtualHits",int(v.hits)},
            {"isFitTrainingBin",r.training},
            {"realRawMedianMm",number(r.medianRawMm)},
            {"realAfterTopMedianMm",number(r.medianTopCorrectedMm)},
            {"realAfterXYMedianMm",number(r.medianAfterXYMm)},
            {"realAfterFullFitMedianMm",number(r.medianAfterFullFitMm)},
            {"virtualRawMedianMm",number(v.medianRawMm)},
            {"virtualAfterTopMedianMm",number(v.medianTopCorrectedMm)},
            {"virtualAfterXYMedianMm",number(v.medianAfterXYMm)},
            {"virtualAfterFullFitMedianMm",number(v.medianAfterFullFitMm)}
        });
    }
    const QJsonObject root{
        {"schema","MotionLaserSync.M52GeometricConsistency"},{"schemaVersion",1},
        {"sourceScanFile",data.sourceScanPath},{"sourceCadFile",data.sourceCadPath},
        {"nominalChamferRadiusMm",report.nominalOuterRadiusMm},
        {"nominalChamferDepthMm",report.nominalChamferDepthMm},
        {"nominalHoleXmm",report.nominalHoleXmm},
        {"nominalHoleYmm",report.nominalHoleYmm},
        {"nominalTopZmm",report.nominalTopZmm},
        {"topGridPitchMm",opt.topGridPitchMm},
        {"topGuardMm",opt.topGuardMm},
        {"topZGateMm",opt.topZGateMm},
        {"coneGateMm",opt.coneGateMm},
        {"real",jsonFit(report.real)},
        {"virtual",report.hasVirtual?QJsonValue(jsonFit(report.virtualFit)):QJsonValue(QJsonValue::Null)},
        {"angular",residualAngles},
        {"autoApplyEnabled",false},
        {"limitation",QString::fromStdString(report.limitation)}
    };
    auto error=writeAtomic(outputJson,QJsonDocument(root).toJson(QJsonDocument::Indented));
    if(!error.isEmpty())return error;
    QString base=outputJson;
    if(base.endsWith(QStringLiteral(".json"),Qt::CaseInsensitive))base.chop(5);
    const QString csvPath=base+QStringLiteral("_angular.csv");
    QByteArray csv("angle_start_deg,angle_end_deg,real_hits,virtual_hits,is_training,real_raw_mm,real_after_top_mm,real_after_xy_mm,real_after_full_fit_mm,virtual_raw_mm,virtual_after_top_mm,virtual_after_xy_mm,virtual_after_full_fit_mm\n");
    for(int a=0;a<M52AngularBins;++a){
        const auto& r=report.real.angular[std::size_t(a)];
        const auto& v=report.virtualFit.angular[std::size_t(a)];
        const QString line=QStringLiteral("%1,%2,%3,%4,%5,%6,%7,%8,%9,%10,%11,%12,%13\n")
            .arg(a*5).arg((a+1)*5).arg(r.hits).arg(v.hits).arg(int(r.training))
            .arg(fixed(r.medianRawMm)).arg(fixed(r.medianTopCorrectedMm))
            .arg(fixed(r.medianAfterXYMm)).arg(fixed(r.medianAfterFullFitMm))
            .arg(fixed(v.medianRawMm)).arg(fixed(v.medianTopCorrectedMm))
            .arg(fixed(v.medianAfterXYMm)).arg(fixed(v.medianAfterFullFitMm));
        csv.append(line.toUtf8());
    }
    return writeAtomic(csvPath,csv);
}
} // namespace mls::twin
