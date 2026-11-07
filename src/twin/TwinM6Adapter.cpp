#include "twin/TwinM6Adapter.h"
#include "twin/TwinM6NcPreview.h"
#include "calibration/SingleSphere17Calibration.h"
#include "safety/MachineSafetyModel.h"
#include <QDir>
#include <QDateTime>
#include <QCryptographicHash>
#include <QVector>
#include <QStorageInfo>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QStringList>
#include <QSaveFile>
#include <QUuid>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>

namespace mls::twin {
namespace {
Joint5 qFrom(const std::array<double,5>& a){return {{a[0],a[1],a[2],a[3],a[4]}};}
mls::calibration::JointPose5d qToPhysical(const Joint5& q){
    return {q[0],q[1],q[2],q[3],q[4]};
}
Transform physical17Forward(const Joint5& q,const mls::calibration::Physical17Parameters& p){
    return {mls::calibration::SingleSphere17Model::sensorToMachineTransform(qToPhysical(q),p)};
}
Vec3 xyz(Transform t){return t.point({0,0,0});}
bool matrixFinite(const Transform& T){for(const double v:T.m)if(!std::isfinite(v))return false;return true;}
QJsonValue safeNumber(double v){return std::isfinite(v)?QJsonValue(v):QJsonValue(QJsonValue::Null);}
QString writeTo(QSaveFile& f,const QByteArray& b){
    if(f.write(b)!=b.size())return QStringLiteral("写入不完整：")+f.fileName();
    return {};
}
} // namespace
TwinM6Data::~TwinM6Data(){
    if(!binarySpoolPath.isEmpty())QFile::remove(binarySpoolPath);
    if(!temporaryArchiveDirectory.isEmpty())QDir(temporaryArchiveDirectory).removeRecursively();
}
std::shared_ptr<TwinM6Data> TwinM6Adapter::run(
    const TwinScene& scene,const M5Report& baseline,
    const ScanReplayData& measured,const Sensor2950Geometry& model,
    M6Options options,const QString& machineSafetyConfigPath,
    const std::atomic<bool>* cancel,std::atomic<std::size_t>* progress,
    M6ProgressState* liveProgress){
    auto d=std::make_shared<TwinM6Data>();
    d->sourceCadPath=QString::fromLocal8Bit(scene.mesh().source.c_str());
    d->sourceScanPath=measured.sourcePath;
    if(!scene.hasWorkpiece()||!measured.ok()||!baseline.ok()){
        d->status=QStringLiteral("M6 需要 STEP、M2 回放和 M5 CAD 覆盖结果");return d;
    }
    // A simulated SCAN may be viewed by M2, but never treated as an
    // independent measured baseline for M6 planning.
    for(const auto& p:measured.scan.profiles){
        if(p.fiveAxis.frame.timing==mls::runtime::PoseTiming::Simulated){
            d->status=QStringLiteral("M6 输入必须是实际 H1/H3 扫描，不接受 M6 Simulated SCAN 作为初扫真值");
            return d;
        }
    }
    if(measured.scan.profiles.isEmpty()){
        d->status=QStringLiteral("真实 SCAN 没有有效位姿");return d;
    }
    const auto& f0=measured.scan.profiles[0].fiveAxis.frame;
    const auto qRef=qFrom(f0.joints);
    for(const auto& profile:measured.scan.profiles){
        const auto& joints=profile.fiveAxis.frame.joints;
        if(std::abs(joints[3]-qRef[3])>0.05 || std::abs(joints[4]-qRef[4])>0.05){
            d->bcVariedInRecordedScan=true;break;
        }
    }
    const Transform recorded{f0.sensorToMachine};
    if(!matrixFinite(recorded)){
        d->status=QStringLiteral("M2 参考传感器位姿无效");return d;
    }
    mls::calibration::Physical17Parameters physical;
    physical.value=f0.physicalParameters;
    bool allFinite=true;
    for(const double x:physical.value)if(!std::isfinite(x))allFinite=false;
    M6ForwardKinematics fk;
    QString kineSource;
    if(allFinite){
        const auto calculated=physical17Forward(qRef,physical);
        if(matrixFinite(calculated)){
            const Transform anchor=recorded*calculated.inverseRigid();
            const auto full=[physical,anchor](const Joint5& q){
                return anchor*physical17Forward(q,physical);
            };
            // Verify on several independent recorded joints, not only anchor frame.
            double worstPosition=0,worstRotation=0;
            const qsizetype size=measured.scan.profiles.size();
            for(const qsizetype pos:{qsizetype(0),size/4,size/2,(size*3)/4,size-1}){
                const auto& frame=measured.scan.profiles[pos].fiveAxis.frame;
                const auto approx=full(qFrom(frame.joints));
                const Transform truth{frame.sensorToMachine};
                worstPosition=std::max(worstPosition,length(xyz(approx)-xyz(truth)));
                for(int row=0;row<3;++row)for(int col=0;col<3;++col)
                    worstRotation=std::max(worstRotation,std::abs(approx.m[std::size_t(row*4+col)]-truth.m[std::size_t(row*4+col)]));
            }
            if(worstPosition<0.30 && worstRotation<0.003){
                fk=full;d->fullPhysical17Validated=true;
                kineSource=d->bcVariedInRecordedScan
                    ?QStringLiteral("Physical17 anchored to M2; 5-sample XYZBC replay check <=0.30mm/0.003")
                    :QStringLiteral("Physical17 anchored to M2; H1 scan has fixed B/C, angular response NOT independently validated");
            }
        }
    }
    if(!fk){
        // This fallback remains an OFFLINE preview. Never machine-authorized.
        fk=makeM6NominalFromRecorded(qRef,recorded);
        kineSource=QStringLiteral("Nominal XYZCB anchored to M2; Physical17 replay mismatch; NOT CALIBRATED");
    }
    std::shared_ptr<mls::safety::MachineSafetyModel> machine;
    if(!machineSafetyConfigPath.isEmpty()&&QFileInfo::exists(machineSafetyConfigPath)){
        auto check=std::make_shared<mls::safety::MachineSafetyModel>();
        std::string error;
        if(check->load(std::filesystem::path(machineSafetyConfigPath.toStdWString()),&error) && check->isReady()){
            machine=std::move(check);
            const auto& cfg=machine->config().planningLimits;
            if(cfg.configured){
                const auto& guard=machine->config().safety;
                const double g=guard.linearJointMarginMm,a=guard.rotaryJointMarginDeg;
                options.bounds.min={{cfg.xMinMm+g,cfg.yMinMm+g,cfg.zMinMm+g,cfg.cMinDeg+a,cfg.bMinDeg+a}};
                options.bounds.max={{cfg.xMaxMm-g,cfg.yMaxMm-g,cfg.zMaxMm-g,cfg.cMaxDeg-a,cfg.bMaxDeg-a}};
            }
        }
    }
    d->machineFclLoaded=bool(machine);
    M6CollisionCheck collision;
    if(machine){
        collision=[machine](const Joint5& q,std::string* why){
            const JointPose5D joint{q[0],q[1],q[2],q[3],q[4]};
            const auto r=machine->checkPose(joint,true);
            if(!r.safe&&why)*why=r.message;
            return r.modelReady&&r.safe;
        };
    }
    const QUuid archiveId=QUuid::createUuid();
    d->temporaryArchiveDirectory=QDir(QDir::tempPath()).filePath(
        QStringLiteral("MotionLaserSync_M6_SIM_%1").arg(archiveId.toString(QUuid::WithoutBraces)));
    if(!QDir().mkpath(d->temporaryArchiveDirectory)){
        d->status=QStringLiteral("无法创建 M6 离线 SCAN 暂存目录");return d;
    }
    std::uint64_t session=14695981039346656037ULL;
    const QByteArray unique=archiveId.toRfc4122();
    for(const unsigned char c:unique){session^=c;session*=1099511628211ULL;}
    if(session==0)session=1;
    M6ScanVolumeWriter writer(d->temporaryArchiveDirectory,options.scanRateHz,
        f0.physicalParameters,session,
        liveProgress?&liveProgress->archiveVolumesSaved:nullptr);
    if(options.alsoExportPly)
        d->binarySpoolPath=QDir(QDir::tempPath()).filePath(QStringLiteral("MotionLaserSync_M6_%1.xyzfloat")
            .arg(QUuid::createUuid().toString(QUuid::WithoutBraces)));
    TwinM6ContinuousPlanner planner(scene,baseline,model,std::move(fk),options,std::move(collision));
    const M6FullProfileSink sink=[&writer](const VirtualProfile& p,std::size_t segment,std::string* error){
        return writer.append(p,segment,error);
    };
    d->report=planner.run(cancel,progress,d->binarySpoolPath.toLocal8Bit().toStdString(),liveProgress,sink);
    if(d->report.ok()){
        std::string error;
        if(!writer.finish(&error))d->report.error="M6 SCAN archive verification: "+error;
        else if(writer.profileCount()!=d->report.fullProfiles)
            d->report.error="M6 simulated SCAN profile count mismatch";
        else{
            d->scanVolumes=writer.volumes();
            d->archivedValidPoints=writer.validPointCount();
        }
    }
    d->report.kinematicsSource=kineSource.toStdString();
    if(!d->report.ok()){
        d->status=d->report.cancelled?QStringLiteral("M6 用户取消"):
            QString::fromStdString(d->report.error);
        return d;
    }
    d->preview.reserve(qsizetype(d->report.pointPreview.size()));
    for(const auto& p:d->report.pointPreview)
        d->preview.append(QVector3D{float(p.x),float(p.y),float(p.z)});
    d->pathPreview.reserve(qsizetype(d->report.pathPreview.size()));
    for(const auto& p:d->report.pathPreview)
        d->pathPreview.append(QVector3D{float(p.x),float(p.y),float(p.z)});
    // V28.7.4: retain every selected scan segment as a separate display
    // layer. Preview samples are spread over each complete segment; laser-off
    // transfer positions have their own polyline and never join exposure paths.
    auto asQt=[](Vec3 pt){return QVector3D{float(pt.x),float(pt.y),float(pt.z)};};
    d->segmentPointPreviews.reserve(qsizetype(d->report.segmentPreviews.size()));
    for(const auto& seg:d->report.segmentPreviews){
        QVector<QVector3D> samples;
        samples.reserve(qsizetype(seg.points.size()));
        for(const auto& pt:seg.points)samples.push_back(asQt(pt));
        d->segmentPointPreviews.push_back(std::move(samples));
    }
    d->segmentLaserPaths.resize(qsizetype(d->report.segments.size()));
    // Decimate TIME samples only, not the complete lossless SCAN archive.
    // Each path contains its own start/end, and the XYZCB pose timeline above
    // remains full-resolution for pose inspection.
    std::size_t curSegment=0;
    bool previousScan=false,previousTransfer=false;
    std::size_t segmentSamples=0,transferSamples=0;
    const Vec3 focus{0.,0.,d->report.options.focusSensorZmm};
    const auto invalid=QVector3D{std::numeric_limits<float>::quiet_NaN(),
        std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::quiet_NaN()};
    for(std::size_t index=0;index<d->report.path.size();++index){
        const auto& sample=d->report.path[index];
        const bool laserOn=sample.role==M6SegmentRole::Scan;
        const auto world=asQt(sample.sensorToMachine.point(focus));
        if(laserOn){
            if(!previousScan){
                if(previousScan==false && segmentSamples>0)++curSegment;
                segmentSamples=0;
            }
            if(curSegment<d->report.segments.size()){
                auto& path=d->segmentLaserPaths[qsizetype(curSegment)];
                const std::size_t count=std::max<std::uint64_t>(1,d->report.segments[curSegment].emittedProfiles);
                const std::size_t step=std::max<std::size_t>(1,count/400);
                if(segmentSamples==0 || segmentSamples%step==0 ||
                    segmentSamples+1==count)path.append(world);
            }
            ++segmentSamples;
            previousTransfer=false;
        }else{
            if(!previousTransfer && !d->transferLaserPath.isEmpty())d->transferLaserPath.append(invalid);
            if(transferSamples%6==0)d->transferLaserPath.append(world);
            ++transferSamples;
            previousTransfer=true;
        }
        previousScan=laserOn;
    }
    return d;
}

// Multi-volume export uses the EXISTING CompactScanIO writer and preserves
// simulation source provenance. The manifest is written last; incomplete
// output never masquerades as a finished capture session.
QString TwinM6Adapter::exportReport(const TwinM6Data& data,const QString& path,
    const std::atomic<bool>* cancel,M6ExportProgress* progress){
    if(!data.ok()||data.scanVolumes.empty())return QStringLiteral("没有可导出的 M6 模拟 SCAN 分卷");
    const auto& r=data.report;
    const QFileInfo targetInfo(path);
    const QString stem=targetInfo.absolutePath()+"/"+targetInfo.completeBaseName();
    const QString trajectoryPath=stem+"_trajectory_offline.csv";
    const QString manifestPath=stem+"_scan_manifest.json";
    // Two independent previews expose the H1/H3 electrical contract, but are
    // unconditionally OfflineOnly and NOT machine-executable.
    const QString ncH1Path=stem+"_OFFLINE_ONLY_H1.nc";
    const QString ncH3Path=stem+"_OFFLINE_ONLY_H3.nc";
    M6NcPreviewOptions baseOpt;
    baseOpt.proposedTopologyId=1;
    baseOpt.sourceCadPath=data.sourceCadPath.toStdString();
    baseOpt.kinematicsSource=r.kinematicsSource;
    // No verified CAD-to-machine 6-DoF part pose or uniquely identified
    // commissioning calibration is available from the current M6 report.
    // Never disguise source file names as a PartPoseId or CalibrationId.

    const auto h1=makeM6NcPreview(r,baseOpt);
    M6NcPreviewOptions h3Opt=baseOpt;h3Opt.proposedTopologyId=3;
    const auto h3=makeM6NcPreview(r,h3Opt);
    if(!h1.ok()||!h3.ok())
        return QStringLiteral("M6 NC 离线轨迹无法安全序列化：")+
            QString::fromStdString(!h1.ok()?h1.error:h3.error);
    const QString plyPath=stem+(r.options.exportPointStride==1 ? "_simulated_full.ply":"_simulated_sampled.ply");
    struct VolumePath {M6ScanVolume volume;QString dst;};
    std::vector<VolumePath> output;
    output.reserve(data.scanVolumes.size());
    for(std::size_t i=0;i<data.scanVolumes.size();++i){
        const auto& v=data.scanVolumes[i];
        const QString dst=stem+QStringLiteral("_seg%1_part%2.scan")
            .arg(static_cast<qulonglong>(v.scanSegmentNumber),2,10,QChar('0'))
            .arg(static_cast<qulonglong>(v.partNumber),4,10,QChar('0'));
        output.push_back({v,dst});
    }
    QStringList allPaths{path,trajectoryPath,manifestPath,ncH1Path,ncH3Path};
    if(r.options.alsoExportPly)allPaths.push_back(plyPath);
    for(const auto& v:output)allPaths.push_back(v.dst);
    // Never accidentally overwrite an earlier valid multi-volume data set.
    for(const QString& p:allPaths){
        if(QFileInfo::exists(p))return QStringLiteral("目标文件已存在，请改用新文件名：")+p;
    }
    const auto cancelled=[&](){return cancel&&cancel->load();};
    QStringList created;
    const auto failAndRollback=[&](const QString& msg){
        for(const QString& f:created)QFile::remove(f);
        return msg;
    };
    if(progress){
        std::uint64_t total=0;
        for(const auto& p:output)total+=p.volume.storedBytes;
        if(r.options.alsoExportPly)total+=r.exportedValidPoints*3*sizeof(float);
        progress->bytesTotal.store(total);progress->bytesDone.store(0);
        progress->filesTotal.store(output.size()+5+(r.options.alsoExportPly?1:0));
        progress->filesDone.store(0);
    }
    QJsonArray volumeItems;
    std::uint64_t totalProfiles=0,totalSamples=0,totalStored=0;
    for(const auto& item:output){
        if(cancelled())return failAndRollback(QStringLiteral("导出已取消"));
        QFile src(item.volume.temporaryPath);
        if(!src.open(QIODevice::ReadOnly))return failAndRollback(src.errorString());
        if(static_cast<std::uint64_t>(src.size())!=item.volume.storedBytes)
            return failAndRollback(QStringLiteral("M6 临时 SCAN 分卷长度已变化"));
        QSaveFile dst(item.dst);
        if(!dst.open(QIODevice::WriteOnly))return failAndRollback(dst.errorString());
        QCryptographicHash sha(QCryptographicHash::Sha256);
        while(!src.atEnd()){
            if(cancelled()){dst.cancelWriting();return failAndRollback(QStringLiteral("导出已取消"));}
            const QByteArray block=src.read(1024*1024);
            if(block.isEmpty()&&!src.atEnd()){
                dst.cancelWriting();return failAndRollback(QStringLiteral("读取 M6 模拟 SCAN 暂存文件失败"));
            }
            if(dst.write(block)!=block.size()){
                dst.cancelWriting();return failAndRollback(dst.errorString());
            }
            sha.addData(block);
            if(progress)progress->bytesDone.fetch_add(static_cast<std::uint64_t>(block.size()));
        }
        if(!dst.commit())return failAndRollback(dst.errorString());
        created.push_back(item.dst);
        if(progress)progress->filesDone.fetch_add(1);
        totalProfiles+=item.volume.profiles;totalSamples+=item.volume.validPoints;
        totalStored+=item.volume.storedBytes;
        volumeItems.append(QJsonObject{
            {"file",QFileInfo(item.dst).fileName()},
            {"scanSegment",static_cast<int>(item.volume.scanSegmentNumber)},
            {"partNumber",static_cast<int>(item.volume.partNumber)},
            {"firstOrdinal",static_cast<double>(item.volume.firstOrdinal)},
            {"lastOrdinal",static_cast<double>(item.volume.lastOrdinal)},
            {"profileCount",static_cast<double>(item.volume.profiles)},
            {"validSamples",static_cast<double>(item.volume.validPoints)},
            {"bytes",static_cast<double>(item.volume.storedBytes)},
            {"sha256",QString::fromLatin1(sha.result().toHex())}
        });
    }
    if(totalProfiles!=r.fullProfiles||totalSamples!=data.archivedValidPoints)
        return failAndRollback(QStringLiteral("M6 SCAN 分卷与仿真汇总计数不一致"));
    // The offline joint/transform CSV includes unexposed transfers and
    // exposes them explicitly as TRANSFER, NOT as laser-on samples.
    {
        QSaveFile csv(trajectoryPath);
        if(!csv.open(QIODevice::WriteOnly))return failAndRollback(csv.errorString());
        QByteArray b("time_sec,mode,X_mm,Y_mm,Z_mm,C_deg,B_deg,sensor_m00,sensor_m01,sensor_m02,sensor_tx,sensor_m10,sensor_m11,sensor_m12,sensor_ty,sensor_m20,sensor_m21,sensor_m22,sensor_tz,hardware_authorized\n");
        for(const auto& item:r.path){
            if(cancelled()){csv.cancelWriting();return failAndRollback(QStringLiteral("导出已取消"));}
            b.append(QByteArray::number(item.seconds,'f',8));
            b.append(item.role==M6SegmentRole::Scan?",SCAN":",TRANSFER");
            for(const auto q:item.joints){b.append(',');b.append(QByteArray::number(q,'f',7));}
            const auto& m=item.sensorToMachine.m;
            for(int row=0;row<3;++row)for(int col=0;col<4;++col){
                b.append(',');b.append(QByteArray::number(m[std::size_t(row*4+col)],'f',9));
            }
            b.append(",0\n");
            if(b.size()>65536){const QString err=writeTo(csv,b);if(!err.isEmpty()){
                csv.cancelWriting();return failAndRollback(err);}b.clear();}
        }
        const QString err=writeTo(csv,b);
        if(!err.isEmpty()){csv.cancelWriting();return failAndRollback(err);}
        if(!csv.commit())return failAndRollback(csv.errorString());
        created.push_back(trajectoryPath);
        if(progress)progress->filesDone.fetch_add(1);
    }
    // Atomic G-code previews: every accepted M6 five-axis joint is retained,
    // scan/transfer groups separated; no real controller/Sensor API is called.
    for(const auto& pair:std::array<std::pair<QString,const M6NcPreviewResult*>,2>{{
            {ncH1Path,&h1},{ncH3Path,&h3}}}){
        if(cancelled())return failAndRollback(QStringLiteral("导出已取消"));
        QSaveFile nc(pair.first);
        if(!nc.open(QIODevice::WriteOnly))return failAndRollback(nc.errorString());
        const QByteArray content=QByteArray::fromStdString(pair.second->text);
        const auto writeError=writeTo(nc,content);
        if(!writeError.isEmpty()){nc.cancelWriting();return failAndRollback(writeError);}
        if(!nc.commit())return failAndRollback(nc.errorString());
        created.push_back(pair.first);
        if(progress)progress->filesDone.fetch_add(1);
    }
    // Optional visual-only PLY. The lossless SCAN volumes are always produced.
    if(r.options.alsoExportPly){
        QFile raw(data.binarySpoolPath);
        if(!raw.open(QIODevice::ReadOnly) ||
           raw.size()!=qint64(r.exportedValidPoints)*qint64(3*sizeof(float)))
            return failAndRollback(QStringLiteral("可选 PLY 原始缓存缺失或长度不一致"));
        QSaveFile ply(plyPath);
        if(!ply.open(QIODevice::WriteOnly))return failAndRollback(ply.errorString());
        const QByteArray hdr=QStringLiteral("ply\nformat binary_little_endian 1.0\ncomment M6 simulated, NOT hardware data\nelement vertex %1\nproperty float x\nproperty float y\nproperty float z\nend_header\n")
            .arg(static_cast<qulonglong>(r.exportedValidPoints)).toLatin1();
        if(ply.write(hdr)!=hdr.size()){
            ply.cancelWriting();return failAndRollback(QStringLiteral("PLY 文件头写入失败"));}
        while(!raw.atEnd()){
            if(cancelled()){ply.cancelWriting();return failAndRollback(QStringLiteral("导出已取消"));}
            const QByteArray part=raw.read(1024*1024);
            if(part.isEmpty()&&!raw.atEnd()){
                ply.cancelWriting();return failAndRollback(QStringLiteral("读取 PLY 缓存失败"));}
            if(ply.write(part)!=part.size()){
                ply.cancelWriting();return failAndRollback(QStringLiteral("PLY 写入失败"));}
            if(progress)progress->bytesDone.fetch_add(static_cast<std::uint64_t>(part.size()));
        }
        if(!ply.commit())return failAndRollback(ply.errorString());
        created.push_back(plyPath);
        if(progress)progress->filesDone.fetch_add(1);
    }
    const QJsonObject manifest{
        {"schema","MotionLaserSync.M6SimulatedScanVolumes"},{"version",1},
        {"source","SIMULATED_NOT_HARDWARE"},{"scanVersion",3},
        {"archiveVersion",5},{"compression","QtZlib_qCompress"},
        {"profileIndexSpace","GLOBAL_0_BASED"},
        {"poseTiming","PoseTiming::Simulated (NO HARDWARE FEEDBACK/LATCH CLAIMS)"},
        {"profileTime","SYNTHETIC_RELATIVE_MICROSECONDS"},
        {"validSamplesNote","all valid simulated workpiece/support returns; invalid sample slots omitted, original indices retained"},
        {"scanRateHz",r.options.scanRateHz},
        {"ncH1Preview",QFileInfo(ncH1Path).fileName()},
        {"ncH3Preview",QFileInfo(ncH3Path).fileName()},
        {"ncExecutePermission","OFFLINE_ONLY; NEVER MACHINE_AUTHORIZED"},
        {"totalProfiles",static_cast<double>(totalProfiles)},
        {"totalValidSamples",static_cast<double>(totalSamples)},
        {"compressedBytes",static_cast<double>(totalStored)},
        {"opticsMultiAngleValidated",false},{"machineAuthorized",false},
        {"files",volumeItems}
    };
    const auto atomicJson=[&](const QString& file,const QJsonObject& object)->QString{
        QSaveFile f(file);
        if(!f.open(QIODevice::WriteOnly))return f.errorString();
        const QString err=writeTo(f,QJsonDocument(object).toJson(QJsonDocument::Indented));
        if(!err.isEmpty()){f.cancelWriting();return err;}
        if(!f.commit())return f.errorString();
        created.push_back(file);
        if(progress)progress->filesDone.fetch_add(1);
        return {};
    };
    QString err=atomicJson(manifestPath,manifest);
    if(!err.isEmpty())return failAndRollback(err);

    QJsonArray pieces;
    for(const auto& s:r.segments)pieces.append(QJsonObject{
        {"feature",QString::fromStdString(s.feature)},{"featureId",int(s.featureId)},
        {"phase",int(s.phaseId)},{"amplitudeScale",s.amplitudeScale},
        {"startSeconds",s.startSeconds},{"durationSeconds",s.durationSec},
        {"expectedNewAreaMm2",s.predictedNewAreaMm2},
        {"measuredNewAreaMm2",s.measuredNewAreaMm2},
        {"profileCount",static_cast<double>(s.emittedProfiles)},
        {"validVirtualPoints",static_cast<double>(s.virtualValidPoints)},
        {"jointLimitsPassed",s.jointLimitsPassed},{"dynamicLimitsPassed",s.dynamicsPassed},
        {"machineFclChecked",s.collisionChecked},
        {"peakRelativeSpeed",s.vmaxUsage},{"peakRelativeAccel",s.amaxUsage},
        {"peakRelativeJerk",s.jmaxUsage},{"status",QString::fromStdString(s.status)}
    });
    QJsonArray byFeature;
    for(std::size_t i=0;i<r.featureNewAreaMm2.size();++i)
        byFeature.append(QJsonObject{{"featureId",int(i)},
            {"simulatedNewAreaMm2",r.featureNewAreaMm2[i]}});
    QJsonObject root{
        {"schema","MotionLaserSync.M6Continuous5AxisOffline"},{"version",2},
        {"sourceCad",data.sourceCadPath},{"sourceScan",data.sourceScanPath},
        {"kinematics",QString::fromStdString(r.kinematicsSource)},
        {"fullPhysical17Validated",data.fullPhysical17Validated},
        {"recordedBCVaried",data.bcVariedInRecordedScan},
        {"multiAngleOpticsValidated",false},{"machineFclLoaded",data.machineFclLoaded},
        {"returnedToRecordedReferencePose",r.returnedToReferencePose},
        {"transferSafetyWarning",QString::fromStdString(r.transferSafetyWarning)},
        {"fullDynamicFixtureCollisionCertified",false},{"canExportForHardware",false},
        {"warnings","SIMULATED ONLY: M3 multi-angle optics and housing/fixture safety not validated"},
        {"m51SparseReopenedAreaMm2",r.reopenedSparseChamferAreaMm2},
        {"initialAreaMm2",r.initialObservedAreaMm2},{"newAreaMm2",r.newAreaMm2},
        {"finalAreaMm2",r.finalObservedAreaMm2},{"targetAreaMm2",r.targetAreaMm2},
        {"scanRateHz",r.options.scanRateHz},
        {"candidateFrameCount",static_cast<int>(r.options.candidateEvaluationFrames)},
        {"candidateCoarseFrameCount",static_cast<int>(r.options.candidateCoarseFrames)},
        {"candidateAttempted",static_cast<double>(r.candidateCurvesAttempted)},
        {"candidateCoarseFeasible",static_cast<double>(r.candidateCurvesFeasible)},
        {"candidateRefined",static_cast<double>(r.candidateCurvesRefined)},
        {"candidateOpticalProfiles",static_cast<double>(r.candidateProfilesSimulated)},
        {"candidateSearchTruncated",r.candidateSearchTruncated},
        {"exportPointStride",static_cast<int>(r.options.exportPointStride)},
        {"continuousScanSegments",int(r.segments.size())},
        {"virtualProfiles",static_cast<double>(r.fullProfiles)},
        {"virtualValidWorkpiecePoints",static_cast<double>(r.fullValidPoints)},
        {"archivedValidSamples",static_cast<double>(data.archivedValidPoints)},
        {"scanArchiveManifest",QFileInfo(manifestPath).fileName()},
        {"gcodePreviewH1",QFileInfo(ncH1Path).fileName()},
        {"gcodePreviewH3",QFileInfo(ncH3Path).fileName()},
        {"gcodePreviewHardwareAuthorized",false},
        {"scanVolumeCount",int(data.scanVolumes.size())},
        {"alsoExportedPly",r.options.alsoExportPly},
        {"exportedPlyPoints",static_cast<double>(r.exportedValidPoints)},
        {"segments",pieces},{"newAreaByFeature",byFeature},
        {"minNewHitsPerCell",int(r.options.minNewHitsPerCell)}
    };
    if(cancelled())return failAndRollback(QStringLiteral("导出已取消"));
    err=atomicJson(path,root);
    if(!err.isEmpty())return failAndRollback(err);
    return {};
}
} // namespace mls::twin
