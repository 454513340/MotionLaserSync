#include "twin/TwinM7Adapter.h"
#include <QSaveFile>
#include <QFile>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QByteArray>
#include <QtGlobal>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <limits>

namespace mls::twin {
namespace {
QVector3D toQt(Vec3 p){return QVector3D{float(p.x),float(p.y),float(p.z)};}
Vec3 toVec(QVector3D p){return {double(p.x()),double(p.y()),double(p.z())};}
struct Key {std::int64_t x,y,z;
    bool operator==(const Key& rhs)const {return x==rhs.x&&y==rhs.y&&z==rhs.z;}};
struct Hash {std::size_t operator()(Key k)const noexcept{
    std::uint64_t h=1469598103934665603ULL;
    for(auto x:{k.x,k.y,k.z}){h^=static_cast<std::uint64_t>(x);h*=1099511628211ULL;}
    return std::size_t(h);
}};
Key keyFor(Vec3 p,double pitch){return {std::int64_t(std::floor(p.x/pitch)),
    std::int64_t(std::floor(p.y/pitch)),std::int64_t(std::floor(p.z/pitch))};}
QJsonValue jn(double v){return std::isfinite(v)?QJsonValue(v):QJsonValue(QJsonValue::Null);}
QString writeAtomic(const QString& path,const QByteArray& data){
    QSaveFile f(path);
    if(!f.open(QIODevice::WriteOnly))return f.errorString();
    if(f.write(data)!=data.size()){f.cancelWriting();return QStringLiteral("写入失败：")+path;}
    if(!f.commit())return f.errorString();return {};
}
}
std::shared_ptr<TwinM7Data> TwinM7Adapter::fuse(
    const TwinScene& scene,const ScanReplayData& real,
    const TwinM5Data& m5,const TwinM6Data& m6,
    const M7VoxelOptions& voxels,double target,
    const std::atomic<bool>* cancel,M7Progress* progress){
    auto d=std::make_shared<TwinM7Data>();
    d->sourceM2=real.sourcePath;
    d->sourceCad=QString::fromLocal8Bit(scene.mesh().source.c_str());
    const auto cancelled=[&]{return cancel&&cancel->load(std::memory_order_relaxed);};
    if(!real.ok()||!m5.ok()||!m6.ok()||!scene.hasWorkpiece()){
        d->message=QStringLiteral("M7 需要当前 STEP、实际 M2、M5 和完整 M6 模拟结果");return d;
    }
    if(!(target>0&&target<=1&&std::isfinite(target))){
        d->message=QStringLiteral("M7 验收目标应为 0%～100%");return d;
    }
    for(const auto& p:real.scan.profiles){
        if(p.fiveAxis.frame.timing==mls::runtime::PoseTiming::Simulated){
            d->message=QStringLiteral("M7 不允许将模拟 SCAN 当作真实 M2 初扫");return d;
        }
    }
    std::uint64_t archiveProfiles=0,archiveValid=0;
    for(const auto& v:m6.scanVolumes){archiveProfiles+=v.profiles;archiveValid+=v.validPoints;}
    if(archiveProfiles!=m6.report.fullProfiles || archiveValid!=m6.archivedValidPoints){
        d->message=QStringLiteral("M6 分卷索引 Profile/采样数与报告不一致");return d;
    }
    if(progress){progress->stage.store(1);progress->total.store(m5.report.cells.size());progress->done.store(0);}
    d->coverage=calculateM7Coverage(m5.report,m6.report,target);
    if(!d->coverage.ok()){
        d->message=QStringLiteral("M7 CAD 覆盖计算失败：")+QString::fromStdString(d->coverage.error);return d;
    }
    if(!d->coverage.areaConsistent){
        d->message=QStringLiteral("M5/M6 覆盖面积不守恒，请确认未重新放置 CAD 或修改 M5 单元");return d;
    }
    if(cancelled()){d->message=QStringLiteral("M7 已取消");return d;}
    const auto T=scene.cadToMachineEstimate();
    d->newlyCoveredCad.reserve(45000);d->stillMissingCad.reserve(45000);
    d->remainingTargets.reserve(m5.report.cells.size()/2);
    const std::unordered_set<std::uint32_t> reopened(m6.report.reopenedSparseCellIds.begin(),
                                                       m6.report.reopenedSparseCellIds.end());
    const std::size_t markerStep=std::max<std::size_t>(1,m5.report.cells.size()/85000);
    for(std::size_t i=0;i<m5.report.cells.size();++i){
        const auto status=d->coverage.classifications[i];
        if(status!=M7CellClass::SimulatedNew&&status!=M7CellClass::Missing)continue;
        const auto& c=m5.report.cells[i];
        if(status==M7CellClass::Missing){
            M7RemainingTarget t;
            t.cellIndex=static_cast<std::uint32_t>(i);t.featureId=c.featureId;
            t.cadPosition=c.cadCenter;t.machinePosition=T.point(c.cadCenter);
            t.machineNormal=normalized(T.vector(c.cadNormal));
            t.areaMm2=c.areaMm2;t.measuredHits=c.realHits;
            t.sparseReopened=reopened.count(static_cast<std::uint32_t>(i))!=0;
            d->remainingTargets.push_back(t);
        }
        if(i%markerStep==0){
            const auto p=T.point(c.cadCenter+c.cadNormal*.12);
            if(status==M7CellClass::SimulatedNew)d->newlyCoveredCad.append(toQt(p));
            else d->stillMissingCad.append(toQt(p));
        }
        if(cancelled()){d->message=QStringLiteral("M7 已取消");return d;}
    }
    if(progress){progress->done.store(m5.report.cells.size());progress->stage.store(2);
        progress->total.store(std::uint64_t(real.previewPoints.size())+
            std::uint64_t(m6.report.pointPreview.size()));progress->done.store(0);}
    M7VoxelSampler samplerReal(voxels),samplerM6(voxels);
    if(!samplerReal.valid()||!samplerM6.valid()){
        d->message=QStringLiteral("M7 体素预览参数无效");return d;
    }
    const auto bounds=scene.mesh().bounds;
    const auto inv=T.inverseRigid();
    const auto nearCad=[&](Vec3 p){
        const auto c=inv.point(p);
        const double margin=.7;
        return c.x>=bounds.min.x-margin && c.x<=bounds.max.x+margin &&
            c.y>=bounds.min.y-margin && c.y<=bounds.max.y+margin &&
            c.z>=bounds.min.z-margin && c.z<=bounds.max.z+margin;
    };
    for(qsizetype i=0;i<real.previewPoints.size();++i){
        if(cancelled()){d->message=QStringLiteral("M7 已取消");return d;}
        const auto p=toVec(real.previewPoints[i]);
        if(nearCad(p))samplerReal.observe(p);
        if(progress&&i%2048==0)progress->done.store(std::uint64_t(i));
    }
    d->segmentPreview.reserve(qsizetype(m6.segmentPointPreviews.size()));
    const std::uint64_t m2Size=std::uint64_t(real.previewPoints.size());
    std::uint64_t scanned=0;
    if(progress)progress->stage.store(3);
    for(qsizetype s=0;s<m6.segmentPointPreviews.size();++s){
        QVector<QVector3D> preview;
        const auto& input=m6.segmentPointPreviews[s];
        M7VoxelOptions segOpt=voxels;
        segOpt.maxRetainedVoxels=std::max<std::size_t>(1,voxels.maxRetainedVoxels/
            std::max<std::size_t>(1,std::size_t(m6.segmentPointPreviews.size())));
        M7VoxelSampler perSegment(segOpt);
        for(const auto& p:input){
            if(cancelled()){d->message=QStringLiteral("M7 已取消");return d;}
            const auto pt=toVec(p);
            if(nearCad(pt)){samplerM6.observe(pt);perSegment.observe(pt);}
            ++scanned;
            if(progress&&(scanned%2048==0))progress->done.store(m2Size+scanned);
        }
        for(const auto& item:perSegment.finalize(M7Source::SimulatedM6,std::uint16_t(s+1)))
            preview.append(toQt(item.xyz));
        d->segmentPreview.append(std::move(preview));
    }
    const auto rpoints=samplerReal.finalize(M7Source::MeasuredM2,0);
    const auto spoints=samplerM6.finalize(M7Source::SimulatedM6,0);
    d->previewM2Points=std::uint64_t(rpoints.size());
    d->previewM6Points=std::uint64_t(spoints.size());
    d->realPoints.reserve(qsizetype(rpoints.size()));
    d->simulatedPoints.reserve(qsizetype(spoints.size()));
    d->fusedPreview.reserve(qsizetype(rpoints.size()+spoints.size()));
    std::unordered_map<Key,Vec3,Hash> rvox;
    rvox.reserve(rpoints.size());
    for(const auto& p:rpoints){
        d->realPoints.append(toQt(p.xyz));d->fusedPreview.append(toQt(p.xyz));
        rvox.emplace(keyFor(p.xyz,voxels.voxelPitchMm),p.xyz);
    }
    double sum=0,sumSq=0;std::uint64_t both=0;
    for(const auto& p:spoints){
        d->simulatedPoints.append(toQt(p.xyz));
        const auto it=rvox.find(keyFor(p.xyz,voxels.voxelPitchMm));
        if(it!=rvox.end()){
            ++both;const auto diff=length(p.xyz-it->second);
            sum+=diff;sumSq+=diff*diff;
            // real position wins, synthetic point is NOT averaged into M2
            d->overlapPoints.append(toQt(it->second));
        }else{d->fusedPreview.append(toQt(p.xyz));d->simulatedOnlyPoints.append(toQt(p.xyz));}
    }
    if(both){d->overlapMeanMm=sum/double(both);d->overlapRmsMm=std::sqrt(sumSq/double(both));}
    if(progress){progress->stage.store(4);progress->done.store(progress->total.load());}
    return d;
}

QString TwinM7Adapter::exportReport(const TwinM7Data& d,const TwinM6Data& m6,
                                   const QString& path,const std::atomic<bool>* cancel){
    if(!d.ok())return QStringLiteral("M7 没有有效融合结果");
    const auto stop=[&]{return cancel&&cancel->load();};
    const QFileInfo fi(path);
    const QString stem=fi.absolutePath()+"/"+fi.completeBaseName();
    const QString byFeature=stem+"_features.csv",bySegment=stem+"_segments.csv";
    const QString pointPreview=stem+"_fusion_preview.ply";
    const QString missingCsv=stem+"_remaining_targets.csv";
    // Never replace an existing user's report or data.
    for(const QString& p:{path,byFeature,bySegment,pointPreview,missingCsv})
        if(QFileInfo::exists(p))return QStringLiteral("M7 文件已存在，请另选报告名称：")+p;
    QJsonObject root;
    root["schema"]="MotionLaserSync.M7OfflineFusionReview";
    root["version"]=1;
    root["offlineOnly"]=true;
    root["machineAuthorized"]=false;
    root["m2Source"]=d.sourceM2;
    root["cadSource"]=d.sourceCad;
    root["m6LosslessScanVolumes"]=int(m6.scanVolumes.size());
    root["m6ProfileCount"]=double(m6.report.fullProfiles);
    root["m6ValidWorkpiecePoints"]=double(m6.report.fullValidPoints);
    root["m6ArchivedReturnSamples"]=double(m6.archivedValidPoints);
    root["previewOnlyPly"]=true;
    root["fusionPolicy"]="REAL M2 position takes priority; M6 returns are SIMULATED; no CAD interpolation";
    root["coverageCaveat"]="M6 predicted optical visibility at new B/C angles is not experimentally calibrated";
    const auto& c=d.coverage;
    root["targetAreaMm2"]=jn(c.targetAreaMm2);
    root["measuredAreaMm2"]=jn(c.measuredAreaMm2);
    root["simulatedNewAreaMm2"]=jn(c.simulatedNewAreaMm2);
    root["finalPredictedAreaMm2"]=jn(c.measuredAreaMm2+c.simulatedNewAreaMm2);
    root["remainingAreaMm2"]=jn(c.remainingAreaMm2);
    root["coverageFraction"]=jn(c.finalFraction());
    root["needsFurtherView"]=c.needsAdditionalView;
    root["remainingCadCells"]=double(d.remainingTargets.size());
    root["m51ReopenedAreaMm2"]=jn(c.reopenedSparseAreaMm2);
    root["m2PreviewVoxels"]=double(d.previewM2Points);
    root["m6PreviewVoxels"]=double(d.previewM6Points);
    root["fusedUniquePreviewPoints"]=double(d.fusedPreview.size());
    root["m6NonOverlapPreviewPoints"]=double(d.simulatedOnlyPoints.size());
    root["overlapVoxelMeanDistanceMm"]=jn(d.overlapMeanMm);
    root["overlapVoxelRmsDistanceMm"]=jn(d.overlapRmsMm);
    QJsonArray segments,features;
    QByteArray segmentCsv("segment,feature,profiles,validSimulatedWorkpiecePoints,durationSec,expectedNewAreaMm2,simulatedNewAreaMm2\n");
    QByteArray featureCsv("featureId,feature,targetAreaMm2,measuredAreaMm2,simulatedNewAreaMm2,remainingAreaMm2,reopenedSparseAreaMm2\n");
    for(std::size_t i=0;i<m6.report.segments.size();++i){
        const auto& s=m6.report.segments[i];
        QJsonObject j; j["segment"]=int(i+1);
        j["feature"]=QString::fromStdString(s.feature);
        j["profiles"]=double(s.emittedProfiles);j["durationSec"]=jn(s.durationSec);
        j["expectedNewAreaMm2"]=jn(s.predictedNewAreaMm2);
        j["simulatedNewAreaMm2"]=jn(c.addedAreaBySegmentMm2[i]);
        j["cumulativeAreaMm2"]=jn(c.cumulativeAreaBySegmentMm2[i]);
        segments.append(j);
        segmentCsv+=QByteArray::number(i+1)+",\""+QByteArray::fromStdString(s.feature)+"\","+
            QByteArray::number(s.emittedProfiles)+","+QByteArray::number(s.virtualValidPoints)+","+
            QByteArray::number(s.durationSec,'f',5)+","+QByteArray::number(s.predictedNewAreaMm2,'f',6)+","+
            QByteArray::number(c.addedAreaBySegmentMm2[i],'f',6)+"\n";
    }
    for(const auto& f:c.features){
        QJsonObject j; j["id"]=int(f.featureId);j["name"]=QString::fromStdString(f.name);
        j["targetAreaMm2"]=jn(f.targetAreaMm2);j["measuredAreaMm2"]=jn(f.measuredAreaMm2);
        j["simulatedNewAreaMm2"]=jn(f.simulatedNewAreaMm2);
        j["stillMissingAreaMm2"]=jn(f.stillMissingAreaMm2);
        j["sparseReopenedAreaMm2"]=jn(f.sparseReopenedAreaMm2);
        features.append(j);
        featureCsv+=QByteArray::number(f.featureId)+",\""+QByteArray::fromStdString(f.name)+"\","+
            QByteArray::number(f.targetAreaMm2,'f',6)+","+
            QByteArray::number(f.measuredAreaMm2,'f',6)+","+
            QByteArray::number(f.simulatedNewAreaMm2,'f',6)+","+
            QByteArray::number(f.stillMissingAreaMm2,'f',6)+","+
            QByteArray::number(f.sparseReopenedAreaMm2,'f',6)+"\n";
    }
    root["segments"]=segments;root["features"]=features;
    if(stop())return QStringLiteral("M7 已取消导出");
    const auto json=QJsonDocument(root).toJson(QJsonDocument::Indented);
    const auto e=writeAtomic(path,json);if(!e.isEmpty())return e;
    if(stop()){QFile::remove(path);return QStringLiteral("M7 已取消导出");}
    const auto e1=writeAtomic(byFeature,featureCsv);
    if(!e1.isEmpty()){QFile::remove(path);return e1;}
    const auto e2=writeAtomic(bySegment,segmentCsv);
    if(!e2.isEmpty()){QFile::remove(path);QFile::remove(byFeature);return e2;}
    QSaveFile missing(missingCsv);
    if(!missing.open(QIODevice::WriteOnly))return missing.errorString();
    QByteArray csv="cadCellIndex,featureId,cadX,cadY,cadZ,machineX,machineY,machineZ,"
        "normalX,normalY,normalZ,areaMm2,m2HitCount,m51SparseReopened,category\n";
    for(const auto& t:d.remainingTargets){
        if(stop()){missing.cancelWriting();return QStringLiteral("M7 已取消导出");}
        csv+=QByteArray::number(t.cellIndex)+","+QByteArray::number(t.featureId)+","+
            QByteArray::number(t.cadPosition.x,'f',7)+","+
            QByteArray::number(t.cadPosition.y,'f',7)+","+
            QByteArray::number(t.cadPosition.z,'f',7)+","+
            QByteArray::number(t.machinePosition.x,'f',7)+","+
            QByteArray::number(t.machinePosition.y,'f',7)+","+
            QByteArray::number(t.machinePosition.z,'f',7)+","+
            QByteArray::number(t.machineNormal.x,'f',7)+","+
            QByteArray::number(t.machineNormal.y,'f',7)+","+
            QByteArray::number(t.machineNormal.z,'f',7)+","+
            QByteArray::number(t.areaMm2,'f',7)+","+
            QByteArray::number(t.measuredHits)+","+
            QByteArray::number(int(t.sparseReopened))+",UNKNOWN_MISSING_CAUSE\n";
        if(csv.size()>=(1<<18)){
            if(missing.write(csv)!=csv.size()){
                missing.cancelWriting();return QStringLiteral("M7 待补扫 CSV 写入失败");
            }
            csv.clear();
        }
    }
    if(!csv.isEmpty()&&missing.write(csv)!=csv.size()){
        missing.cancelWriting();return QStringLiteral("M7 待补扫 CSV 写入失败");
    }
    if(!missing.commit())return missing.errorString();
    // Binary little-endian PLY is a PREVIEW, real and synthetic points retain
    // separate source labels. The full raw sources remain untouched in SCAN.
    QSaveFile ply(pointPreview);
    if(!ply.open(QIODevice::WriteOnly))return ply.errorString();
    const quint64 nr=quint64(d.realPoints.size()),ns=quint64(d.simulatedOnlyPoints.size());
    QByteArray head="ply\nformat binary_little_endian 1.0\ncomment M7 preview only; full M2 and M6 SCAN archives are source of truth\n";
    head+="element vertex "+QByteArray::number(nr+ns)+"\nproperty float x\nproperty float y\nproperty float z\n";
    head+="property uchar source\nend_header\n";
    if(ply.write(head)!=head.size()){ply.cancelWriting();return QStringLiteral("M7 PLY 头写入失败");}
    auto writePoints=[&](const QVector<QVector3D>& pts,char provenance)->bool{
        QByteArray buffer;buffer.reserve(1<<19);
        for(qsizetype i=0;i<pts.size();++i){
            if(stop())return false;
            const auto& p=pts[i];
            const float xyz[3]={p.x(),p.y(),p.z()};
            buffer.append(reinterpret_cast<const char*>(xyz),sizeof(xyz));
            buffer.append(provenance);
            if(buffer.size()>=(1<<18)){
                if(ply.write(buffer)!=buffer.size())return false;
                buffer.clear();
            }
        }
        return buffer.isEmpty()||ply.write(buffer)==buffer.size();
    };
    if(!writePoints(d.realPoints,char(1))||!writePoints(d.simulatedOnlyPoints,char(2))){
        ply.cancelWriting();QFile::remove(path);QFile::remove(byFeature);QFile::remove(bySegment);QFile::remove(missingCsv);
        return QStringLiteral("M7 PLY 写入失败或取消");
    }
    if(!ply.commit())return ply.errorString();
    return {};
}
} // namespace mls::twin
