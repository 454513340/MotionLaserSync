#include "twin/ScanReplayAdapter.h"
#include "runtime/Acquisition5D.h"
#include <QFileInfo>
#include <QSaveFile>
#include <QtEndian>
#include <cmath>
#include <algorithm>
#include <limits>
#include <cstring>

namespace mls::twin {
std::shared_ptr<ScanReplayData> ScanReplayAdapter::load(const QString& path,int previewLimit) {
    auto out=std::make_shared<ScanReplayData>();
    out->sourcePath=QFileInfo(path).absoluteFilePath();
    QString error;
    if (!CompactScanIO::load(path,out->scan,{},&error)) {
        out->error=QStringLiteral("SCAN 加载/CRC 失败: ")+error; return out;
    }
    if(out->scan.header.version!=3 || out->scan.profiles.isEmpty()) {
        out->error=QStringLiteral("M2 仅允许携带逐帧 XYZCB + 标定变换的 SCAN v3; 旧版不可推断 C/B。");
        return out;
    }
    if(static_cast<std::uint64_t>(out->scan.profiles.size())!=out->scan.header.profileCount) {
        out->error=QStringLiteral("SCAN header/profileCount 不匹配"); return out;
    }
    for(const auto& p : out->scan.profiles) {
        if(!p.hasFiveAxis) {out->error=QStringLiteral("发现无五轴曝光位姿的轮廓");return out;}
        out->totalRawPoints+=static_cast<std::uint64_t>(p.fiveAxis.points.size());
    }
    // Preview is deliberately capped; full-resolution export always uses scan.profiles.
    const std::uint64_t step=std::max<std::uint64_t>(1,
        (out->totalRawPoints+static_cast<std::uint64_t>(std::max(1,previewLimit))-1)
        /static_cast<std::uint64_t>(std::max(1,previewLimit)));
    out->previewPoints.reserve(static_cast<qsizetype>(std::min<std::uint64_t>(out->totalRawPoints,static_cast<std::uint64_t>(std::max(1,previewLimit)))));
    out->frames.reserve(out->scan.profiles.size());
    double elapsed=0.0;
    double lastSensor=std::numeric_limits<double>::quiet_NaN();
    std::uint64_t pointOrdinal=0;
    std::uint64_t lastOrdinal=0,firstSession=0;
    bool haveFirst=false;
    for(const auto& p : out->scan.profiles) {
        const auto& a=p.fiveAxis;
        ScanReplayFrame f;
        f.profileIndex=a.profileIndex;
        f.ordinal=a.frame.ordinal;
        f.joints=a.frame.joints;
        f.rawPoints=static_cast<std::uint32_t>(a.points.size());
        if(haveFirst) {
            if(a.frame.session!=firstSession || a.frame.ordinal<=lastOrdinal){
                out->error=QStringLiteral("SCAN 存在跨 session 或非递增五轴锁存 ordinal，拒绝回放");return out;
            }
            // Sensor clock is authoritative; host receive time may jump.
            double dt=a.sensorTimeSec-lastSensor;
            const double nominal=std::max(0.000001,out->scan.header.sampleIntervalUs*1e-6);
            if(!std::isfinite(dt)||dt<=0||dt>1.0)dt=nominal;
            elapsed+=dt;
        } else {firstSession=a.frame.session;haveFirst=true;}
        lastSensor=a.sensorTimeSec;lastOrdinal=a.frame.ordinal;
        f.elapsedSeconds=elapsed;
        const bool valid=mls::runtime::validTransform(a.frame.sensorToMachine) &&
             mls::runtime::finiteJoint(a.frame.joints);
        if(!valid) ++out->invalidTransforms;
        for(const auto& s:a.points) {
            if(pointOrdinal++%step!=0)continue;
            if(!valid || !std::isfinite(s.x)||!std::isfinite(s.z)){
                ++out->invalidPointCoords;continue;
            }
            const auto q=mls::runtime::mapProfilePoint(a.frame,s.x,s.z);
            if(!std::isfinite(q[0])||!std::isfinite(q[1])||!std::isfinite(q[2])){
                ++out->invalidPointCoords;continue;
            }
            // MSVC C++17: use brace initialization; parentheses with `float(q[i])`
            // are parsed as a function declaration (most vexing parse).
            const QVector3D xyz{
                static_cast<float>(q[0]),
                static_cast<float>(q[1]),
                static_cast<float>(q[2])
            };
            if(!out->hasPreviewBounds){
                out->previewBoundsMin=xyz;out->previewBoundsMax=xyz;out->hasPreviewBounds=true;
            }else{
                out->previewBoundsMin.setX(std::min(out->previewBoundsMin.x(),xyz.x()));
                out->previewBoundsMin.setY(std::min(out->previewBoundsMin.y(),xyz.y()));
                out->previewBoundsMin.setZ(std::min(out->previewBoundsMin.z(),xyz.z()));
                out->previewBoundsMax.setX(std::max(out->previewBoundsMax.x(),xyz.x()));
                out->previewBoundsMax.setY(std::max(out->previewBoundsMax.y(),xyz.y()));
                out->previewBoundsMax.setZ(std::max(out->previewBoundsMax.z(),xyz.z()));
            }
            out->previewPoints.append(xyz);
        }
        f.previewEnd=out->previewPoints.size();
        out->frames.append(f);
    }
    if(out->invalidTransforms) {
        out->error=QStringLiteral("SCAN 存在 %1 条无效的五轴变换/姿态，不允许将其解释为有效曝光位置")
                   .arg(static_cast<qulonglong>(out->invalidTransforms));
    }
    return out;
}

QString ScanReplayAdapter::exportFullPly(const ScanReplayData& data,const QString& filename) {
    if(!data.ok())return QStringLiteral("没有有效的 SCAN v3 数据");
    std::uint64_t count=0;
    for(const auto& p:data.scan.profiles){
        for(const auto& s:p.fiveAxis.points){
            if(!std::isfinite(s.x)||!std::isfinite(s.z))continue;
            const auto q=mls::runtime::mapProfilePoint(p.fiveAxis.frame,s.x,s.z);
            if(std::isfinite(q[0])&&std::isfinite(q[1])&&std::isfinite(q[2]))++count;
        }
    }
    QSaveFile out(filename);
    if(!out.open(QIODevice::WriteOnly))return out.errorString();
    const QByteArray hdr=QStringLiteral("ply\nformat binary_little_endian 1.0\ncomment SCAN v3 Frame5D exposure transform, full valid raw measurement\nelement vertex %1\nproperty float x\nproperty float y\nproperty float z\nend_header\n")
       .arg(static_cast<qulonglong>(count)).toLatin1();
    if(out.write(hdr)!=hdr.size()){out.cancelWriting();return QStringLiteral("PLY 文件头写入失败");}
    QByteArray block;block.reserve(12*4096);
    auto flush=[&]()->bool{if(block.isEmpty())return true;const qint64 n=out.write(block);bool ok=n==block.size();block.clear();return ok;};
    for(const auto& p:data.scan.profiles){
        for(const auto& s:p.fiveAxis.points){
            if(!std::isfinite(s.x)||!std::isfinite(s.z))continue;
            const auto xyz=mls::runtime::mapProfilePoint(p.fiveAxis.frame,s.x,s.z);
            if(!std::isfinite(xyz[0])||!std::isfinite(xyz[1])||!std::isfinite(xyz[2]))continue;
            for(double value:xyz){
                const float fv=float(value);
                static_assert(sizeof(float)==sizeof(quint32),"PLY float32 required");
                quint32 bits=0;std::memcpy(&bits,&fv,sizeof(bits));
                bits=qToLittleEndian(bits);
                block.append(reinterpret_cast<const char*>(&bits),sizeof(bits));
            }
            if(block.size()>=12*4096 && !flush()){
                out.cancelWriting();return QStringLiteral("PLY 数据写入失败");
            }
        }
    }
    if(!flush()){out.cancelWriting();return QStringLiteral("PLY 数据写入失败");}
    if(!out.commit())return out.errorString();
    return {};
}
} // namespace mls::twin
