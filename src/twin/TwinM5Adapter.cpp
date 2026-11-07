#include "twin/TwinM5Adapter.h"
#include <QFileInfo>
#include <QSaveFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLocale>
#include <QtGlobal>
#include <cmath>
#include <algorithm>
#include <limits>

namespace mls::twin {
namespace {
QString writeAtomic(const QString& fileName,const QByteArray& bytes){
    QSaveFile f(fileName);
    if(!f.open(QIODevice::WriteOnly))return f.errorString();
    if(f.write(bytes)!=bytes.size()){f.cancelWriting();return QStringLiteral("文件写入不完整：")+fileName;}
    if(!f.commit())return f.errorString();
    return {};
}
QString number(double v,int prec=7){return QLocale::c().toString(v,'f',prec);}
void appendCsv(QByteArray& bytes,const QString& line){bytes.append(line.toUtf8());}
QString appendFileChunk(QSaveFile& f,QByteArray& bytes){
    if(bytes.isEmpty())return {};
    if(f.write(bytes)!=bytes.size())return QStringLiteral("CSV 文件写入失败：")+f.fileName();
    bytes.clear();return {};
}
bool equalTransform(const VirtualProfilePose& pose,const mls::runtime::ArchiveProfile& real){
    if(pose.sourceOrdinal!=real.frame.ordinal)return false;
    for(std::size_t i=0;i<pose.sensorToMachine.m.size();++i)
        if(std::abs(pose.sensorToMachine.m[i]-real.frame.sensorToMachine[i])>1.e-8)return false;
    return true;
}
}
std::shared_ptr<TwinM5Data> TwinM5Adapter::analyze(
    const ScanReplayData& measured,const TwinSimulationData* simulated,
    const TwinScene& scene,const M5Options& options,
    const std::atomic<bool>* cancel,std::atomic<std::size_t>* progress){
    auto out=std::make_shared<TwinM5Data>();
    out->sourceScanPath=measured.sourcePath;
    out->sourceCadPath=QString::fromLocal8Bit(scene.mesh().source.c_str());
    if(!measured.ok()||!scene.hasWorkpiece()){
        out->message=QStringLiteral("M5 需要导入 CAD 工件及有效 M2 SCAN v3");return out;
    }
    const bool useVirtual=simulated!=nullptr;
    if(useVirtual && !simulated->ok()){
        out->message=QStringLiteral("当前 M3 虚拟数据无效，请清空后重试");return out;
    }
    out->containsVirtualProfiles=useVirtual;
    const auto& realProfiles=measured.scan.profiles;
    const std::vector<VirtualProfile>* virtualProfiles=
        useVirtual?&simulated->scan.profiles:nullptr;
    if(useVirtual&&std::size_t(realProfiles.size())!=virtualProfiles->size()){
        out->message=QStringLiteral("M5 检测到 M2/M3 帧数不同，请重新运行 M3");return out;
    }
    TwinM5CoverageBuilder accumulator(scene,options);
    if(!accumulator.ready()){
        out->message=QString::fromStdString(accumulator.error());return out;
    }
    for(qsizetype i=0;i<realProfiles.size();++i){
        if(cancel&&cancel->load(std::memory_order_relaxed)){
            out->message=QStringLiteral("用户取消 M5 CAD 覆盖分析");return out;
        }
        const auto& rp=realProfiles[i].fiveAxis;
        const auto T=Transform{rp.frame.sensorToMachine};
        // SCAN v3 contains valid, sparse returns only. A missing index does
        // NOT identify geometric occlusion, noise or exposure failure.
        for(const auto& p:rp.points){
            if(std::isfinite(p.x)&&std::isfinite(p.z))
                accumulator.addRealMachine(T.point({p.x,0,p.z}), static_cast<std::uint64_t>(i)+1);
        }
        if(virtualProfiles){
            const auto& vp=(*virtualProfiles)[std::size_t(i)];
            if(!equalTransform(vp.pose,rp)){
                out->message=QStringLiteral("M2/M3 曝光位姿不一致，取消 M5：帧 %1").arg(i);return out;
            }
            for(const auto& p:vp.samples){
                // Do not count virtual support-plane echoes as CAD coverage.
                if(p.state!=VirtualSampleState::Valid ||
                   p.hitObject!=VirtualHitObject::Workpiece ||
                   !std::isfinite(p.sensorX)||!std::isfinite(p.sensorZ))continue;
                accumulator.addVirtualMachine(T.point({p.sensorX,0,p.sensorZ}));
            }
        }
        if(progress&&(i%16==0||i+1==realProfiles.size()))
            progress->store(std::size_t(i)+1,std::memory_order_relaxed);
    }
    out->report=std::move(accumulator).finish();
    if(!out->report.ok()){
        out->message=QString::fromStdString(out->report.error);return out;
    }
    // Bounded VTK preview only. The full-area statistics and export keep all
    // original cells, regardless of display decimation.
    constexpr std::size_t kPreviewMax=85000;
    const std::size_t stride=std::max<std::size_t>(1,(out->report.cells.size()+kPreviewMax-1)/kPreviewMax);
    const auto estimated=scene.cadToMachineEstimate();
    const auto hits=options.minHitsPerCell;
    const auto frames=options.minDistinctProfilesPerCell;
    for(std::size_t i=0;i<out->report.cells.size();i+=stride){
        const auto& cell=out->report.cells[i];
        if(!cell.target)continue;
        const Vec3 q=estimated.point(cell.cadCenter);
        const QVector3D point{static_cast<float>(q.x),static_cast<float>(q.y),static_cast<float>(q.z)};
        const bool realCovered=cell.realHits>=hits && cell.realDistinctProfiles>=frames;
        if(realCovered)out->realObservedPreview.append(point);
        else out->realMissingPreview.append(point);
        if(out->containsVirtualProfiles && !realCovered && cell.virtualHits>=hits)
            out->virtualOnlyPreview.append(point);
    }
    return out;
}

QString TwinM5Adapter::exportReport(const TwinM5Data& data,const QString& destinationJson){
    if(!data.ok())return QStringLiteral("M5 分析结果不存在，不能导出");
    const auto& r=data.report;
    QJsonArray groups;
    for(const auto& f:r.features){
        groups.append(QJsonObject{
            {"name",QString::fromStdString(f.name)},
            {"featureId",int(f.id)},{"kind",int(f.kind)},{"target",f.target},
            {"targetAreaMm2",f.totalAreaMm2},
            {"realObservedAreaMm2",f.realAreaMm2},
            {"virtualObservedAreaMm2",f.virtualAreaMm2},
            {"bothObservedAreaMm2",f.bothAreaMm2},
            {"realCoverage",f.realCoverage()},{"virtualCoverage",f.virtualCoverage()},
            {"missingBothAreaMm2",f.missingAreaMm2},
            {"realMissingAreaMm2",f.totalAreaMm2-f.realAreaMm2},
            {"cells",double(f.totalCells)},
            {"realObservedCells",double(f.realCells)},
            {"virtualObservedCells",double(f.virtualCells)}
        });
    }
    const auto& o=r.requestedOptions;
    QJsonObject root{
        {"schema","MotionLaserSync.TwinM5SurfaceCoverage"},
        {"schemaVersion",1},{"units","mm"},
        {"sourceCadFile",data.sourceCadPath},
        {"sourceScanFile",data.sourceScanPath},
        {"hasVirtualM3",data.containsVirtualProfiles},
        {"h1FeatureTemplateApplied",r.h1FeatureTemplate},
        {"surfaceRepresentation","STEP_TRIANGULATED_PROXY_or_OBJ_NOT_ANALYTIC_BREP"},
        {"coverageDefinition","area_weighted_cells_with_minHits_and_minDistinctExposureFrames"},
        {"realMissingCause","UNKNOWN_NOT_AUTOMATICALLY_CAMERA_OCCLUSION"},
        {"registration","estimated_cadToMachine_no_truth_leak_into_measured_assignment"},
        {"notMetrologyTolerance",true},
        {"opticalModelState","M3_UNCALIBRATED_GEOMETRIC_APPROXIMATION"},
        {"cellPitchRequestedMm",o.cellPitchMm},
        {"cellPitchEffectiveMm",r.effectiveCellPitchMm},
        {"associationDistanceMm",o.maxSurfaceDistanceMm},
        {"minHitsPerCell",int(o.minHitsPerCell)},
        {"minDistinctProfilesPerCell",int(o.minDistinctProfilesPerCell)},
        {"excludeBottom",o.excludeBottom},
        {"maxCells",double(o.maxCells)},
        {"totalCells",double(r.cells.size())},
        {"cadTargetAreaMm2",r.targetAreaMm2},
        {"excludedAreaMm2",r.excludedAreaMm2},
        {"realAreaMm2",r.realAreaMm2},
        {"virtualAreaMm2",r.virtualAreaMm2},
        {"bothAreaMm2",r.bothAreaMm2},
        {"realOnlyAreaMm2",r.realOnlyAreaMm2},
        {"virtualOnlyAreaMm2",r.virtualOnlyAreaMm2},
        {"missingBothAreaMm2",r.missingBothAreaMm2},
        {"realMissingAreaMm2",r.targetAreaMm2-r.realAreaMm2},
        {"realAreaCoverage",r.realCoverage()},
        {"virtualAreaCoverage",r.virtualCoverage()},
        {"realPointsSeen",double(r.realPointsSeen)},
        {"virtualPointsSeen",double(r.virtualPointsSeen)},
        {"realPointsAssociated",double(r.realAccepted)},
        {"virtualPointsAssociated",double(r.virtualAccepted)},
        {"realPointsExcluded",double(r.realExcluded)},
        {"virtualPointsExcluded",double(r.virtualExcluded)},
        {"realPointsNotMatched",double(r.realNoAssociation)},
        {"virtualPointsNotMatched",double(r.virtualNoAssociation)},
        {"features",groups},
        {"limitation","Surface cells are discretized approximate geometry; one viewpoint cannot identify unreachable regions or true laser-camera intrinsics"}
    };
    auto error=writeAtomic(destinationJson,QJsonDocument(root).toJson(QJsonDocument::Indented));
    if(!error.isEmpty())return error;
    QString base=destinationJson;
    if(base.endsWith(QStringLiteral(".json"),Qt::CaseInsensitive))base.chop(5);
    QSaveFile csv(base+QStringLiteral("_cells.csv"));
    if(!csv.open(QIODevice::WriteOnly))return csv.errorString();
    QByteArray dataChunk;
    appendCsv(dataChunk,QStringLiteral("cell_id,feature_id,face_id,source_triangle,cad_x_mm,cad_y_mm,cad_z_mm,nx,ny,nz,area_mm2,target,real_hits,virtual_hits,real_covered,virtual_covered,distinct_real_exposures\n"));
    const auto minHits=o.minHitsPerCell;
    for(std::size_t i=0;i<r.cells.size();++i){
        const auto& cell=r.cells[i];
        const QString line=QStringLiteral("%1,%2,%3,%4,%5,%6,%7,%8,%9,%10,%11,%12,%13,%14,%15,%16,%17\n")
            .arg(static_cast<qulonglong>(i))
            .arg(cell.featureId).arg(cell.cadFaceId).arg(cell.sourceTriangle)
            .arg(number(cell.cadCenter.x)).arg(number(cell.cadCenter.y)).arg(number(cell.cadCenter.z))
            .arg(number(cell.cadNormal.x)).arg(number(cell.cadNormal.y)).arg(number(cell.cadNormal.z))
            .arg(number(cell.areaMm2,9)).arg(int(cell.target))
            .arg(cell.realHits).arg(cell.virtualHits)
            .arg(int(cell.target && cell.realHits>=minHits &&
                     cell.realDistinctProfiles>=o.minDistinctProfilesPerCell))
            .arg(int(cell.target && cell.virtualHits>=minHits))
            .arg(cell.realDistinctProfiles);
        appendCsv(dataChunk,line);
        if(dataChunk.size()>1024*1024){error=appendFileChunk(csv,dataChunk);if(!error.isEmpty()){csv.cancelWriting();return error;}}
    }
    error=appendFileChunk(csv,dataChunk);
    if(!error.isEmpty()){csv.cancelWriting();return error;}
    if(!csv.commit())return csv.errorString();
    QSaveFile summary(base+QStringLiteral("_features.csv"));
    if(!summary.open(QIODevice::WriteOnly))return summary.errorString();
    QByteArray featuresCsv;
    appendCsv(featuresCsv,QStringLiteral("feature_id,name,target,cells,area_mm2,real_area_mm2,virtual_area_mm2,missing_both_mm2,real_coverage,virtual_coverage\n"));
    for(const auto& f:r.features){
        QString safe=QString::fromStdString(f.name);safe.replace(',',QChar(' '));
        appendCsv(featuresCsv,QStringLiteral("%1,%2,%3,%4,%5,%6,%7,%8,%9,%10\n")
            .arg(f.id).arg(safe).arg(int(f.target))
            .arg(static_cast<qulonglong>(f.totalCells))
            .arg(number(f.totalAreaMm2,6)).arg(number(f.realAreaMm2,6))
            .arg(number(f.virtualAreaMm2,6)).arg(number(f.missingAreaMm2,6))
            .arg(number(f.realCoverage(),7)).arg(number(f.virtualCoverage(),7)));
    }
    if(summary.write(featuresCsv)!=featuresCsv.size()){summary.cancelWriting();return QStringLiteral("覆盖分组 CSV 写入失败");}
    if(!summary.commit())return summary.errorString();
    return {};
}
} // namespace mls::twin
