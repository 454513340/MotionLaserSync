#include "twin/TwinM51Adapter.h"
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLocale>
#include <QSaveFile>
#include <algorithm>
#include <cmath>

namespace mls::twin {
namespace {
bool samePose(const VirtualProfilePose& a,const mls::runtime::ArchiveProfile& b){
    if(a.sourceOrdinal!=b.frame.ordinal)return false;
    for(std::size_t k=0;k<a.sensorToMachine.m.size();++k)
        if(std::abs(a.sensorToMachine.m[k]-b.frame.sensorToMachine[k])>1e-8)return false;
    return true;
}
QString format(double x,int digits=8){return QLocale::c().toString(x,'f',digits);}
QString writeFile(const QString& path,const QByteArray& bytes){
    QSaveFile f(path);
    if(!f.open(QIODevice::WriteOnly))return f.errorString();
    if(f.write(bytes)!=bytes.size()){f.cancelWriting();return QStringLiteral("写入不完整：")+path;}
    if(!f.commit())return f.errorString();
    return {};
}
const char* surfName(M51Surface s){
    switch(s){case M51Surface::TopChamfer:return "top_chamfer";
    case M51Surface::Cylinder:return "cylinder";
    case M51Surface::BottomChamfer:return "bottom_chamfer";}
    return "unknown";
}
}
std::shared_ptr<TwinM51Data> TwinM51Adapter::analyze(
    const ScanReplayData& real,const TwinSimulationData* virt,
    const TwinScene& scene,const M51Options& options,
    const std::atomic<bool>* cancelled,std::atomic<std::size_t>* progress){
    auto result=std::make_shared<TwinM51Data>();
    result->sourceScanPath=real.sourcePath;
    result->sourceCadPath=QString::fromLocal8Bit(scene.mesh().source.c_str());
    if(!real.ok()||!scene.hasWorkpiece()){
        result->message=QStringLiteral("M5.1 需要 STEP 工件与有效 M2 SCAN");return result;
    }
    if(virt&&!virt->ok()){
        result->message=QStringLiteral("M3 数据无效，请清理后重试");return result;
    }
    const auto& realProfiles=real.scan.profiles;
    const auto* vProfiles=virt?&virt->scan.profiles:nullptr;
    if(vProfiles&&vProfiles->size()!=std::size_t(realProfiles.size())){
        result->message=QStringLiteral("M2/M3 Profile 数量不匹配");return result;
    }
    TwinM51ChamferAnalyzer analyzer(scene,options);
    if(!analyzer.ready()){
        result->message=QString::fromStdString(analyzer.error());return result;
    }
    const auto machineToCad=scene.cadToMachineEstimate().inverseRigid();
    for(qsizetype i=0;i<realProfiles.size();++i){
        if(cancelled&&cancelled->load(std::memory_order_relaxed)){
            result->message=QStringLiteral("用户取消 M5.1 分析");return result;
        }
        const auto& prof=realProfiles[i].fiveAxis;
        const Transform sToMachine{prof.frame.sensorToMachine};
        // M2 raw point positions are in SENSOR x/z. Use frame pose only once.
        for(const auto& p:prof.points)
            if(std::isfinite(p.x)&&std::isfinite(p.z))
                analyzer.addRealCad(machineToCad.point(sToMachine.point({p.x,0,p.z})));
        if(vProfiles){
            const auto& v=(*vProfiles)[std::size_t(i)];
            if(!samePose(v.pose,prof)){
                result->message=QStringLiteral("M2/M3 曝光位姿不一致：帧 %1").arg(i);return result;
            }
            for(const auto& p:v.samples)
                if(p.state==VirtualSampleState::Valid&&p.hitObject==VirtualHitObject::Workpiece&&
                   std::isfinite(p.sensorX)&&std::isfinite(p.sensorZ))
                    analyzer.addVirtualCad(machineToCad.point(sToMachine.point({p.sensorX,0,p.sensorZ})));
        }
        if(progress&&(i%32==0||i+1==realProfiles.size()))
            progress->store(std::size_t(i)+1,std::memory_order_relaxed);
    }
    result->report=std::move(analyzer).finish();
    if(!result->report.ok())result->message=QString::fromStdString(result->report.error);
    return result;
}

QString TwinM51Adapter::exportReport(const TwinM51Data& data,const QString& outputJson){
    if(!data.ok())return QStringLiteral("M5.1 分析尚未完成");
    const auto& r=data.report;
    const auto& opts=r.options;
    QJsonArray geometry,thresholds;
    for(const auto d:opts.thresholdsMm)thresholds.append(d);
    for(const auto& g:r.surfaces){
        QJsonArray ids;
        for(auto id:g.cadFaceIds)ids.append(int(id));
        geometry.append(QJsonObject{
            {"name",QString::fromStdString(g.name)},
            {"kind",QString::fromLatin1(surfName(g.surface))},
            {"cadFaceIds",ids},
            {"nominalAreaMm2",g.nominalAreaMm2},
            {"triangulatedMeshAreaMm2",g.meshAreaMm2}
        });
    }
    QJsonArray perAngle;
    const auto primary=opts.primaryThreshold;
    for(int a=0;a<M51AngularBins;++a){
        std::uint64_t real=0,virtualCount=0;
        int lowReal=0,lowVirtual=0;
        for(int j=0;j<M51ChamferLayers;++j){
            const auto& b=r.bins[std::size_t(j*M51AngularBins+a)];
            real+=b.realHits[primary];virtualCount+=b.virtualHits[primary];
            if(b.realSparse)++lowReal;
            if(b.virtualSparse)++lowVirtual;
        }
        perAngle.append(QJsonObject{{"angleStartDeg",a*5},{"angleEndDeg",(a+1)*5},
            {"realHits",double(real)},{"virtualHits",double(virtualCount)},
            {"realSparseRadialLayers",lowReal},{"virtualSparseRadialLayers",lowVirtual}});
    }
    QJsonArray sensitivity;
    for(int k=0;k<M51ThresholdCount;++k){
        sensitivity.append(QJsonObject{{"thresholdMm",opts.thresholdsMm[std::size_t(k)]},
            {"realAssociated",double(r.realAssociated[std::size_t(k)])},
            {"virtualAssociated",double(r.virtualAssociated[std::size_t(k)])}});
    }
    QJsonObject root{
        {"schema","MotionLaserSync.M51H1CenterHole"},{"schemaVersion",1},
        {"sourceCadFile",data.sourceCadPath},{"sourceScanFile",data.sourceScanPath},
        {"hasM3Virtual",r.hasVirtual},{"centerCadXmm",r.xCenterCadMm},
        {"centerCadYmm",r.yCenterCadMm}, {"cadTopZmm",r.cadTopZMm},
        {"angularBins",M51AngularBins},{"chamferRadialLayers",M51ChamferLayers},
        {"cylinderAxialLayers",M51CylinderLayers},
        {"thresholdsMm",thresholds},{"primaryThresholdIndex",primary},
        {"relativeSparseThreshold",opts.sparseRatio},
        {"regionMarginMm",opts.regionMarginMm},
        {"realTopChamferSparseAngles",int(r.realLowDensityAngles)},
        {"realConeEquationMedianMm",r.realConeEquationMedianMm},
        {"realConeEquationMADmm",r.realConeEquationMadMm},
        {"virtualConeEquationMedianMm",r.virtualConeEquationMedianMm},
        {"realConeResidualSamples",double(r.realConeResidualSamples)},
        {"virtualConeResidualSamples",double(r.virtualConeResidualSamples)},
        {"realPointsSeen",double(r.realSeen)},
        {"virtualPointsSeen",double(r.virtualSeen)},
        {"features",geometry},{"associationSensitivity",sensitivity},
        {"topChamferAngularDiagnostic",perAngle},
        {"limitation",QString::fromStdString(r.limitation)},
        {"note","No-return cause unknown; no interpolated CAD points are treated as measurements. Mesh surface classification is H1-specific."}
    };
    auto error=writeFile(outputJson,QJsonDocument(root).toJson(QJsonDocument::Indented));
    if(!error.isEmpty())return error;
    QString base=outputJson;
    if(base.endsWith(QStringLiteral(".json"),Qt::CaseInsensitive))base.chop(5);
    QSaveFile csv(base+QStringLiteral("_bins.csv"));
    if(!csv.open(QIODevice::WriteOnly))return csv.errorString();
    QByteArray chunk("surface,angle_start_deg,angle_end_deg,layer,area_mm2,real_hits_015,virtual_hits_015,real_hits_030,virtual_hits_030,real_hits_065,virtual_hits_065,real_relative_density,virtual_relative_density,real_sparse,virtual_sparse\n");
    for(const auto& b:r.bins){
        const QString line=QStringLiteral("%1,%2,%3,%4,%5,%6,%7,%8,%9,%10,%11,%12,%13,%14,%15\n")
            .arg(QString::fromLatin1(surfName(b.surface)))
            .arg(int(b.angle)*5).arg((int(b.angle)+1)*5).arg(b.layer)
            .arg(format(b.areaMm2,9))
            .arg(b.realHits[0]).arg(b.virtualHits[0])
            .arg(b.realHits[1]).arg(b.virtualHits[1])
            .arg(b.realHits[2]).arg(b.virtualHits[2])
            .arg(format(b.realRelativeDensity,5))
            .arg(format(b.virtualRelativeDensity,5))
            .arg(int(b.realSparse)).arg(int(b.virtualSparse));
        chunk.append(line.toUtf8());
        if(chunk.size()>65536){
            if(csv.write(chunk)!=chunk.size()){csv.cancelWriting();return QStringLiteral("M5.1 CSV 写入失败");}
            chunk.clear();
        }
    }
    if(!chunk.isEmpty()&&csv.write(chunk)!=chunk.size()){csv.cancelWriting();return QStringLiteral("M5.1 CSV 写入不完整");}
    if(!csv.commit())return csv.errorString();
    return {};
}
} // namespace mls::twin
