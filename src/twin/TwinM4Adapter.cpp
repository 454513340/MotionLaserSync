#include "twin/TwinM4Adapter.h"
#include <QFileInfo>
#include <QSaveFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLocale>
#include <QStringList>
#include <cmath>
#include <algorithm>
#include <vector>

namespace mls::twin {
namespace {
std::vector<M4MeasuredSample> sparseSamples(const mls::runtime::ArchiveProfile& a){
    std::vector<M4MeasuredSample> samples;
    samples.reserve(a.points.size());
    for(const auto& p:a.points)samples.push_back({p.index,p.x,p.z});
    return samples;
}
VirtualProfilePose sourcePose(const mls::runtime::ArchiveProfile& a,double elapsed){
    VirtualProfilePose p;
    p.sourceOrdinal=a.frame.ordinal;
    p.elapsedSeconds=elapsed;
    p.joints=a.frame.joints;
    p.sensorToMachine.m=a.frame.sensorToMachine;
    return p;
}
bool matchingTransform(const VirtualProfilePose& v, const mls::runtime::ArchiveProfile& real){
    if(v.sourceOrdinal!=real.frame.ordinal)return false;
    for(std::size_t i=0;i<v.sensorToMachine.m.size();++i)
        if(std::abs(v.sensorToMachine.m[i]-real.frame.sensorToMachine[i])>1e-8)return false;
    return true;
}
QJsonObject toJson(const M4Metrics& m){
    QJsonArray predictions, unknownRealMissingPredictions;
    for(auto v:m.predictedStates)predictions.append(double(v));
    for(auto v:m.predictedWhereRealMissing)unknownRealMissingPredictions.append(double(v));
    return { {"TP",double(m.truePositive)}, {"FP",double(m.falsePositive)},
        {"FN",double(m.falseNegative)}, {"TN",double(m.trueNegative)},
        {"observedValid",double(m.realValid())},
        {"virtualValid",double(m.simulatedValid())},
        {"precision",m.precision()},{"recall",m.recall()},
        {"F1",m.f1()},{"IoU",m.iou()},
        {"matchedBothValid",double(m.pairedSamples)},
        {"virtualStatusCounts",predictions},
        {"virtualStatusAtUnknownRealMissing",unknownRealMissingPredictions},
        {"meanAbsSensorXmm",m.meanAbsX()},
        {"meanAbsSensorZmm",m.meanAbsZ()},{"rmseSensorZmm",m.rmseZ()}
    };
}
QJsonObject toJson(const M4FloorCoverage& f){
    QJsonArray observed,predicted;
    for(auto x:f.realCounts)observed.append(int(x));
    for(auto x:f.simulatedCounts)predicted.append(int(x));
    return {{"realFloorSamples",double(f.realFloorPoints)},
            {"virtualFloorSamples",double(f.simFloorPoints)},
            {"realOccupiedBins",double(f.realOccupied())},
            {"virtualOccupiedBins",double(f.simulatedOccupied())},
            {"binIoU",f.intersectionOverUnion()},
            {"realPositiveYRatio",f.realPositiveRatio()},
            {"virtualPositiveYRatio",f.simulatedPositiveRatio()},
            {"realBinCounts",observed},{"virtualBinCounts",predicted}};
}
QJsonObject toJson(const M4Report& report){
    QJsonArray holes;
    for(std::size_t i=0;i<report.perHole.size();++i){
        holes.append(QJsonObject{{"name",QStringLiteral("CB%1").arg(static_cast<qulonglong>(i+1))},
                                 {"projectedRoi",toJson(report.perHole[i])},
                                 {"floorAnnulus",toJson(report.floor[i])}});
    }
    return {{"profilesCompared",double(report.profilesCompared)},
            {"rejectedIndexedPoints",double(report.rejectedIndexedPoints)},
            {"all",toJson(report.global)},
            {"counterboreRois",toJson(report.holeRegions)},
            {"outsideRois",toJson(report.outsideHoles)},
            {"counterbores",holes},{"equivalentFitScore",report.fitScore()}};
}
QJsonObject fileHeader(const QString& type,const QString& scanPath){
    return {{"schema",type},{"schemaVersion",1},
      {"referenceScan",scanPath},{"units","mm"},
      {"dataOrigin","OBSERVED_SCAN_v3_vs_VIRTUAL_GEOMETRIC_SIMULATION"},
      {"realMissingPointMeaning","unknown_cause_not_camera_occlusion"},
      {"opticalCalibrationState","UNCALIBRATED_EFFECTIVE_CAMERA_MODEL"},
      {"referenceROI","H1_four_counterbores_template_only_if_CAD_dimensions_match"},
      {"ROIProjection","laser_ray_intersection_with_nominal_CAD_top_plane"},
      {"floorCoverage","36_angle_x_3_radius_bins_in_annular_STEP_floor_not_CAD_surface_coverage"},
      {"limitation","single_view_cannot_identify_independent_true_laser_and_camera_intrinsics"}};
}
QString csvMetrics(const M4Metrics& m){
    return QStringLiteral("%1,%2,%3,%4,%5,%6,%7,%8,%9,%10")
        .arg(static_cast<qulonglong>(m.truePositive))
        .arg(static_cast<qulonglong>(m.falsePositive))
        .arg(static_cast<qulonglong>(m.falseNegative))
        .arg(static_cast<qulonglong>(m.trueNegative))
        .arg(QLocale::c().toString(m.f1(),'f',7))
        .arg(QLocale::c().toString(m.iou(),'f',7))
        .arg(QLocale::c().toString(m.meanAbsX(),'f',6))
        .arg(QLocale::c().toString(m.meanAbsZ(),'f',6))
        .arg(QLocale::c().toString(m.rmseZ(),'f',6))
        .arg(static_cast<qulonglong>(m.pairedSamples));
}
QString writeFile(const QString& filename,const QByteArray& bytes){
    QSaveFile out(filename);
    if(!out.open(QIODevice::WriteOnly))return out.errorString();
    if(out.write(bytes)!=bytes.size()){out.cancelWriting();return QStringLiteral("写入失败：")+filename;}
    if(!out.commit())return out.errorString();
    return {};
}
}
std::shared_ptr<TwinM4ComparisonData> TwinM4Adapter::compare(
    const ScanReplayData& measured,const TwinSimulationData& simulated,
    const TwinScene& scene,const std::atomic<bool>* cancel,
    std::atomic<std::size_t>* progress){
    auto out=std::make_shared<TwinM4ComparisonData>();
    out->realScanPath=measured.sourcePath;
    out->model=simulated.opticalModel;
    if(!measured.ok()||!simulated.ok()){
        out->message=QStringLiteral("M4 需要有效的 M2 和已完成的 M3 点云");return out;
    }
    const auto& rp=measured.scan.profiles;
    const auto& vp=simulated.scan.profiles;
    if(rp.size()!=qsizetype(vp.size())){
        out->message=QStringLiteral("M2/M3 Profile 数量不一致，请重新运行 M3");return out;
    }
    M4Context context=h1ReferenceM4Context(scene,simulated.opticalModel,true);
    TwinM4Accumulator acc(std::move(context));
    for(qsizetype i=0;i<rp.size();++i){
        if(cancel&&cancel->load()){
            out->message=QStringLiteral("用户取消 M4 对比");return out;
        }
        const auto& archive=rp[i].fiveAxis;
        if(!matchingTransform(vp[std::size_t(i)].pose,archive)){
            out->message=QStringLiteral("M2/M3 帧序号或曝光变换不同，不能逐索引比较：帧 %1").arg(i);
            return out;
        }
        if(!acc.addFrame(vp[std::size_t(i)],sparseSamples(archive))){
            out->result=std::move(acc).finish();
            out->message=QStringLiteral("M4 采样索引错误：")+QString::fromStdString(out->result.error);
            return out;
        }
        if(progress)progress->store(std::size_t(i)+1,std::memory_order_relaxed);
    }
    out->result=std::move(acc).finish();
    if(!out->result.ok())out->message=QString::fromStdString(out->result.error);
    return out;
}
std::shared_ptr<TwinM4FitData> TwinM4Adapter::fitEquivalentCameraY(
    const ScanReplayData& measured,const TwinScene& scene,
    const Sensor2950Geometry& model,const M4FitOptions& options,
    const std::atomic<bool>* cancel,std::atomic<std::size_t>* progress){
    auto out=std::make_shared<TwinM4FitData>();
    out->realScanPath=measured.sourcePath;
    if(!measured.ok()||!scene.hasWorkpiece()){
        out->message=QStringLiteral("M4 拟合需要有效的 M2 和 STEP 工件");return out;
    }
    const auto ctx=h1ReferenceM4Context(scene,model,true);
    if(ctx.holes.empty()){
        out->message=QStringLiteral("M4 当前仅有参考 H1 四沉孔 ROI 模板；其他工件需先建立 CAD 特征提取模型");return out;
    }
    const auto count=std::size_t(measured.scan.profiles.size());
    const std::size_t retain=std::max<std::size_t>(options.maxFrames*5,200);
    const std::size_t stride=std::max<std::size_t>(1,count/retain);
    std::vector<M4CalibrationFrame> frames;
    frames.reserve(std::min(count,retain+stride));
    for(std::size_t i=0;i<count;i+=stride){
        if(cancel&&cancel->load()){out->message=QStringLiteral("用户取消 M4 拟合");return out;}
        const auto& a=measured.scan.profiles[int(i)].fiveAxis;
        M4CalibrationFrame frame;
        frame.pose=sourcePose(a,measured.frames[int(i)].elapsedSeconds);
        frame.observed=sparseSamples(a);
        frames.push_back(std::move(frame));
    }
    M4FitOptions physicalOptions=options;physicalOptions.useEstimatedPose=true;
    out->result=TwinM4Calibration::fit(scene,model,frames,physicalOptions,cancel,
        [progress](std::size_t done,std::size_t){
            if(progress)progress->store(done,std::memory_order_relaxed);
        });
    if(out->result.cancelled)out->message=QStringLiteral("用户取消 M4 拟合");
    else if(!out->result.ok())out->message=QString::fromStdString(out->result.error);
    return out;
}
QString TwinM4Adapter::exportComparison(const TwinM4ComparisonData& data,
                                        const QString& destinationJson){
    if(!data.ok())return QStringLiteral("M4 报告无效");
    QJsonObject root=fileHeader("MotionLaserSync.TwinM4Comparison",data.realScanPath);
    root.insert("metrics",toJson(data.result));
    root.insert("cameraBaselineY",data.model.cameraOriginSensor.y);
    auto err=writeFile(destinationJson,QJsonDocument(root).toJson(QJsonDocument::Indented));
    if(!err.isEmpty())return err;
    const QString csv=QFileInfo(destinationJson).absolutePath()+"/"+
        QFileInfo(destinationJson).completeBaseName()+"_profiles.csv";
    QByteArray text("sourceOrdinal,TP,FP,FN,TN,F1,IoU,meanAbsXmm,meanAbsZmm,rmseZmm,pairedSamples,roiTP,roiFP,roiFN\n");
    for(const auto& row:data.result.perFrame){
        const QString line=QString::number(static_cast<qulonglong>(row.sourceOrdinal))+
            ","+csvMetrics(row.global)+
            ","+QString::number(static_cast<qulonglong>(row.holeRegions.truePositive))+
            ","+QString::number(static_cast<qulonglong>(row.holeRegions.falsePositive))+
            ","+QString::number(static_cast<qulonglong>(row.holeRegions.falseNegative))+"\n";
        text+=line.toUtf8();
    }
    return writeFile(csv,text);
}
QString TwinM4Adapter::exportFit(const TwinM4FitData& data,
                                 const QString& destinationJson){
    if(!data.ok())return QStringLiteral("M4 拟合报告无效");
    QJsonObject root=fileHeader("MotionLaserSync.TwinM4EffectiveBaselineFit",data.realScanPath);
    root.insert("bestBaselineY",data.result.bestBaselineY);
    root.insert("bestHoldoutBaselineY",data.result.bestHoldoutBaselineY);
    root.insert("holdoutSupportsBest",data.result.holdoutSupportsBest);
    root.insert("validationNote",QString::fromStdString(data.result.validationNote));
    root.insert("selectedFrames",double(data.result.selectedFrames));
    root.insert("trainingFrames",double(data.result.trainingFrames));
    root.insert("validationFrames",double(data.result.validationFrames));
    root.insert("notPhysicalOpticalCentre",true);
    QJsonArray candidates;
    QByteArray csv("cameraYmm,trainScore,holdoutScore,trainF1,holdoutF1,trainFloorBinIoU,holdoutFloorBinIoU\n");
    for(const auto& c:data.result.candidates){
        candidates.append(QJsonObject{{"candidateSensorY",c.cameraSensorY},
           {"trainingScore",c.trainingScore},{"holdoutScore",c.validationScore},
           {"training",toJson(c.training)},{"holdout",toJson(c.validation)}});
        const auto floorMean=[](const M4Report& r){
            double sum=0;int n=0;
            for(const auto& f:r.floor)if(f.realFloorPoints){sum+=f.intersectionOverUnion();++n;}
            return n?sum/n:0.;
        };
        const auto num=[](double n){return QLocale::c().toString(n,'f',6);};
        csv+=QStringLiteral("%1,%2,%3,%4,%5,%6,%7\n")
            .arg(num(c.cameraSensorY)).arg(num(c.trainingScore))
            .arg(num(c.validationScore)).arg(num(c.training.holeRegions.f1()))
            .arg(num(c.validation.holeRegions.f1())).arg(num(floorMean(c.training)))
            .arg(num(floorMean(c.validation))).toUtf8();
    }
    root.insert("candidates",candidates);
    auto err=writeFile(destinationJson,QJsonDocument(root).toJson(QJsonDocument::Indented));
    if(!err.isEmpty())return err;
    const QString outCsv=QFileInfo(destinationJson).absolutePath()+"/"+
        QFileInfo(destinationJson).completeBaseName()+"_candidates.csv";
    return writeFile(outCsv,csv);
}
} // namespace mls::twin
