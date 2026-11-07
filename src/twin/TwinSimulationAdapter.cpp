#include "twin/TwinSimulationAdapter.h"
#include <QFile>
#include <QSaveFile>
#include <QDataStream>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QtEndian>
#include <cmath>
#include <cstring>
#include <limits>
#include <algorithm>

namespace mls::twin {
std::shared_ptr<TwinSimulationData> TwinSimulationAdapter::run(
    const TwinScene& scene,const Sensor2950Geometry& optics,
    const ScanReplayData& recorded,const std::atomic<bool>* cancel,
    std::atomic<std::size_t>* finished,int previewLimit) {
    auto out=std::make_shared<TwinSimulationData>();
    out->opticalModel=optics;
    out->sourceScanPath=recorded.sourcePath;
    if(!recorded.ok()){out->error=QStringLiteral("必须先导入有效的 M2 SCAN v3 回放数据");return out;}
    std::vector<VirtualProfilePose> poses;
    poses.reserve(std::size_t(recorded.scan.profiles.size()));
    for(const auto& source:recorded.scan.profiles){
        const auto& a=source.fiveAxis;
        VirtualProfilePose p;
        p.sourceOrdinal=a.frame.ordinal;
        p.elapsedSeconds=poses.size()<std::size_t(recorded.frames.size()) ?
            recorded.frames[int(poses.size())].elapsedSeconds : 0.;
        p.joints=a.frame.joints;
        p.sensorToMachine.m=a.frame.sensorToMachine;
        poses.emplace_back(p);
    }
    VirtualLaser2950 scanner(scene,optics,true);
    out->scan=scanner.simulateAll(poses,cancel,[&](std::size_t done,std::size_t){
        if(finished)finished->store(done,std::memory_order_relaxed);
    });
    if(out->scan.cancelled){out->error=QStringLiteral("用户取消 M3 虚拟扫描");return out;}
    if(!out->scan.error.empty()){
        out->error=QString::fromStdString(out->scan.error);return out;
    }
    const std::uint64_t totalSamples=std::uint64_t(out->scan.profiles.size())*
          std::uint64_t(optics.sampleCount);
    const auto step=std::max<std::uint64_t>(1,
        (totalSamples+std::uint64_t(std::max(1,previewLimit))-1)/
        std::uint64_t(std::max(1,previewLimit)));
    out->previewPoints.reserve(qsizetype(std::min<std::uint64_t>(
         out->scan.validCount(),std::uint64_t(std::max(1,previewLimit)))));
    std::uint64_t sampleOrdinal=0;
    for(const auto& profile:out->scan.profiles){
        for(const auto& s:profile.samples){
            if((sampleOrdinal++%step)!=0 || s.state!=VirtualSampleState::Valid)continue;
            const auto q=profile.pose.sensorToMachine.point({s.sensorX,0,s.sensorZ});
            if(!std::isfinite(q.x)||!std::isfinite(q.y)||!std::isfinite(q.z))continue;
            out->previewPoints.append(QVector3D{
               static_cast<float>(q.x),static_cast<float>(q.y),static_cast<float>(q.z)});
        }
    }
    return out;
}

QString TwinSimulationAdapter::exportFullPly(const TwinSimulationData& data,const QString& path){
    if(!data.ok())return QStringLiteral("M3 数据不存在或已经取消");
    QSaveFile out(path);
    if(!out.open(QIODevice::WriteOnly))return out.errorString();
    const QByteArray hdr=QStringLiteral("ply\nformat binary_little_endian 1.0\ncomment MotionLaserSync V28 M3 GEOMETRIC APPROXIMATION (uncalibrated optical model)\nelement vertex %1\nproperty float x\nproperty float y\nproperty float z\nend_header\n")
        .arg(static_cast<qulonglong>(data.scan.validCount())).toLatin1();
    if(out.write(hdr)!=hdr.size()){out.cancelWriting();return QStringLiteral("PLY 头写入失败");}
    QByteArray block;block.reserve(12*4096);
    auto flush=[&]()->bool{
        if(block.isEmpty())return true;
        const auto n=out.write(block);
        const bool success=n==block.size();block.clear();return success;
    };
    for(const auto& p:data.scan.profiles){
        for(const auto& sample:p.samples){
            if(sample.state!=VirtualSampleState::Valid)continue;
            const auto pt=p.pose.sensorToMachine.point({sample.sensorX,0,sample.sensorZ});
            for(double coord:{pt.x,pt.y,pt.z}){
                float f=static_cast<float>(coord);quint32 bits=0;
                std::memcpy(&bits,&f,sizeof(float));bits=qToLittleEndian(bits);
                block.append(reinterpret_cast<const char*>(&bits),sizeof(bits));
            }
            if(block.size()>12*4096&&!flush()){
                out.cancelWriting();return QStringLiteral("PLY 写入失败");
            }
        }
    }
    if(!flush()){out.cancelWriting();return QStringLiteral("PLY 写入失败");}
    if(!out.commit())return out.errorString();
    return {};
}

QString TwinSimulationAdapter::exportMaskBinary(const TwinSimulationData& data,const QString& path){
    if(!data.ok())return QStringLiteral("M3 数据不存在或已经取消");
    // Versioned full-profile data: magic 'M3VPROF1', uint32 frames/samples,
    // per frame uint64 ordinal + float64 time + 16 float64 transform,
    // followed by 1280 tuples (uint8 state, float32 sensorX, float32 sensorZ).
    // Invalid samples retain NaN X/Z. All numbers little-endian.
    // It is NOT a real .scan archive, and must never be relabelled as one.
    QSaveFile out(path);
    if(!out.open(QIODevice::WriteOnly))return out.errorString();
    if(out.write("M3VPROF1",8)!=8){out.cancelWriting();return QStringLiteral("掩码头写入失败");}
    QDataStream ds(&out);
    ds.setByteOrder(QDataStream::LittleEndian);
    ds.setFloatingPointPrecision(QDataStream::DoublePrecision);
    ds<<quint32(data.scan.profiles.size());
    ds<<quint32(data.scan.profiles.empty()?0:data.scan.profiles[0].samples.size());
    for(const auto& p:data.scan.profiles){
        ds<<quint64(p.pose.sourceOrdinal)<<double(p.pose.elapsedSeconds);
        for(double v:p.pose.sensorToMachine.m)ds<<v;
        QByteArray raw;raw.reserve(qsizetype(p.samples.size())*9);
        for(const auto& s:p.samples){
            raw.append(char(s.state));
            for(float v:{s.sensorX,s.sensorZ}){
                quint32 bits=0;std::memcpy(&bits,&v,sizeof(bits));
                bits=qToLittleEndian(bits);
                raw.append(reinterpret_cast<const char*>(&bits),4);
            }
        }
        if(out.write(raw)!=raw.size()){
            out.cancelWriting();return QStringLiteral("虚拟 Profile 写入失败");
        }
    }
    if(ds.status()!=QDataStream::Ok||!out.commit())return QStringLiteral("掩码文件提交失败");
    return {};
}
} // namespace mls::twin

namespace mls::twin {
QString TwinSimulationAdapter::exportMetadataJson(const TwinSimulationData& data,const QString& path){
    if(!data.ok())return QStringLiteral("M3 模拟结果无效，不能导出模型参数");
    const auto& m=data.opticalModel;
    const auto vec=[](Vec3 v){return QJsonArray{v.x,v.y,v.z};};
    QJsonArray counts;
    for(auto v:data.scan.counts)counts.append(double(v));
    QJsonObject json{
        {"schema","MotionLaserSync.VirtualLaser2950.M3"},
        {"version",1},
        {"assumedOpticalCentres",true},
        {"cameraBaselineConvention","H1_effective_positive_SensorY_uncalibrated"},
        {"profileIndexConvention","H1_index_increase_means_SensorX_decrease"},
        {"sourceScanPath",data.sourceScanPath},
        {"units","mm"},
        {"sampleCount",m.sampleCount},
        {"sensorZLimits",QJsonArray{m.zNear,m.zCenter,m.zFar}},
        {"widthsAtSensorZ",QJsonArray{m.widthAtNear,m.widthAtCenter,m.widthAtFar}},
        {"laserOriginSensor",vec(m.laserOriginSensor)},
        {"cameraOriginSensor",vec(m.cameraOriginSensor)},
        {"cameraClearanceMm",m.cameraVisibilityEpsilonMm},
        {"maxRayLengthMm",m.maxTraceDistanceMm},
        {"frameCount",double(data.scan.totalProfiles)},
        {"sampleStatusCounts",counts},
        {"statusOrder","Valid,LaserMiss,CameraOccluded,OutsideField,BadPose,Cancelled"},
        {"calibrationStatus","GEOMETRIC_APPROXIMATION_ONLY_NOT_HARDWARE_CALIBRATED"}
    };
    QSaveFile out(path);
    if(!out.open(QIODevice::WriteOnly))return out.errorString();
    const QByteArray bytes=QJsonDocument(json).toJson(QJsonDocument::Indented);
    if(out.write(bytes)!=bytes.size()||!out.commit())return QStringLiteral("M3 配置 JSON 保存失败");
    return {};
}
} // namespace mls::twin
