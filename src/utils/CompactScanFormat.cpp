#include "utils/CompactScanFormat.h"
#include "core/DataTypes.h"
#include <QFile>
#include <QSaveFile>
#include <QtEndian>
#include <map>
#include <tuple>
#include <cstring>
#include <algorithm>
#include <limits>

using namespace mls::runtime;
namespace {
bool fail(QString* e,const QString& s){if(e)*e=s;return false;}
bool sameScanProvenance(const Frame5D& a,const Frame5D& b){
    return a.session==b.session && a.rotaryOffsets==b.rotaryOffsets &&
           a.physicalParameters==b.physicalParameters && a.timing==b.timing &&
           a.feedbackMask==b.feedbackMask && a.latchMask==b.latchMask &&
           a.commandMask==b.commandMask && a.absoluteGaugeKnown==b.absoluteGaugeKnown &&
           a.temporalOffsetSec==b.temporalOffsetSec && a.temporalAlpha==b.temporalAlpha;
}
bool headerValid(const ScanFileHeader& h,QString* e){
#if Q_BYTE_ORDER != Q_LITTLE_ENDIAN
    return fail(e,QStringLiteral("SCAN legacy outer header requires a little-endian platform"));
#endif
    if(std::memcmp(h.magic,SCAN_FORMAT_MAGIC,4)||h.version<1||h.version>3)return fail(e,QStringLiteral("Unsupported SCAN header/version"));
    if(archiveCrc32(&h,sizeof(h)-4)!=h.headerChecksum)return fail(e,QStringLiteral("SCAN header checksum mismatch"));
    if(h.profileCount>1000000||h.uncompressedSize>ArchiveMaxBytes||h.compressedSize>ArchiveMaxBytes)return fail(e,QStringLiteral("SCAN size exceeds configured 512 MiB / 1M-profile bound"));
    auto codec=static_cast<unsigned>(h.compression);
    if(codec>3 || (h.version>=3 && codec!=0 && codec!=3))return fail(e,QStringLiteral("Unsupported/mislabeled SCAN codec"));
    return true;
}
}
ScanData::Statistics ScanData::calculateStatistics() const {
    Statistics s;
    for(const auto& p:profiles){auto n=p.hasFiveAxis?p.fiveAxis.points.size():std::size_t(p.sensorX.size());s.totalPoints+=n;s.compactSizeBytes+=p.hasFiveAxis?460+n*20:calculateProfileDataSize(header.pointsPerProfile);}
    s.compactSizeBytes+=256+(header.version>=3?20:0);s.rawSizeBytes=s.totalPoints*sizeof(AlignedPoint);
    s.compressedSizeBytes=header.compressedSize?256+header.compressedSize:s.compactSizeBytes;
    if(s.compressedSizeBytes)s.compressionRatio=double(s.rawSizeBytes)/s.compressedSizeBytes;
    if(s.rawSizeBytes)s.savingsPercent=100*(1-double(s.compressedSizeBytes)/s.rawSizeBytes);
    return s;
}
QByteArray CompactScanIO::compress(const QByteArray& v,CompressionType t,int level){return t==CompressionType::None?v:qCompress(v,std::clamp(level,1,9));}
QByteArray CompactScanIO::decompress(const QByteArray& v,CompressionType t,size_t expected){
    if(expected>ArchiveMaxBytes)return {};
    if(t==CompressionType::None)return v.size()==qsizetype(expected)?v:QByteArray{};
    // Check the declared size. Qt inflation is not a hardened hostile-input decoder.
    if(v.size()<4||qFromBigEndian<quint32>(reinterpret_cast<const uchar*>(v.constData()))!=expected)return {};
    QByteArray out=qUncompress(v);return out.size()==qsizetype(expected)?out:QByteArray{};
}
uint32_t CompactScanIO::calculateChecksum(const void* p,size_t n){return archiveCrc32(p,n);}
bool CompactScanIO::save(const QString& path,const ScanData& data,const WriteOptions& o,QString* e){
    if(!data.validationError.isEmpty())return fail(e,data.validationError);
    if(data.profiles.isEmpty())return fail(e,QStringLiteral("No profiles to save"));
    std::vector<ArchiveProfile> profiles;profiles.reserve(data.profiles.size());
    for(const auto& p:data.profiles){if(!p.hasFiveAxis)return fail(e,QStringLiteral("Cannot write XYZ-only data as a five-axis scan; C/B cannot be inferred"));profiles.push_back(p.fiveAxis);}
    std::vector<std::uint8_t> bytes;std::string err;if(!encodeArchive(profiles,bytes,err))return fail(e,QString::fromStdString(err));
    QByteArray raw(reinterpret_cast<const char*>(bytes.data()),qsizetype(bytes.size()));
    ScanFileHeader h=data.header;h.version=3;h.profileCount=profiles.size();h.uncompressedSize=raw.size();
    // Old UI/API LZ4/ZSTD names are backward-compatible aliases, NOT fake codecs.
    h.compression=o.compression==CompressionType::None?CompressionType::None:CompressionType::QtZlib;
    h.compressionLevel=std::uint8_t(std::clamp(o.compressionLevel,1,9));
    auto body=compress(raw,h.compression,h.compressionLevel);if(body.isEmpty())return fail(e,QStringLiteral("Compression failed"));
    if(body.size()>=raw.size()){body=raw;h.compression=CompressionType::None;}
    h.compressedSize=h.compression==CompressionType::None?0:std::uint64_t(body.size());h.blockLineNumber=o.blockLineNumber;
    if(!o.scanName.isEmpty()){std::memset(h.scanName,0,sizeof(h.scanName));auto b=o.scanName.toUtf8().left(63);std::memcpy(h.scanName,b.data(),b.size());}
    if(!o.blockName.isEmpty()){std::memset(h.blockName,0,sizeof(h.blockName));auto b=o.blockName.toUtf8().left(31);std::memcpy(h.blockName,b.data(),b.size());}
    h.headerChecksum=archiveCrc32(&h,sizeof(h)-4);
    QSaveFile file(path);if(!file.open(QIODevice::WriteOnly))return fail(e,file.errorString());
    if(file.write(reinterpret_cast<const char*>(&h),sizeof(h))!=sizeof(h)||file.write(body)!=body.size()||!file.commit())return fail(e,file.errorString());
    return true;
}
bool CompactScanIO::readHeader(const QString& path,ScanFileHeader& h,QString* e){
    QFile f(path);if(!f.open(QIODevice::ReadOnly))return fail(e,f.errorString());
    if(f.read(reinterpret_cast<char*>(&h),sizeof(h))!=sizeof(h))return fail(e,QStringLiteral("Truncated SCAN header"));
    return headerValid(h,e);
}
bool CompactScanIO::load(const QString& path,ScanData& data,const ReadOptions& o,QString* e){
    QFile f(path);if(!f.open(QIODevice::ReadOnly))return fail(e,f.errorString());ScanData tmp;
    if(f.read(reinterpret_cast<char*>(&tmp.header),256)!=256||!headerValid(tmp.header,e))return false;
    const auto& h=tmp.header;
    const auto stored=h.compression==CompressionType::None?h.uncompressedSize:h.compressedSize;
    if(stored==0||f.size()!=qint64(256+stored))return fail(e,QStringLiteral("SCAN body size/header mismatch"));
    if(!o.loadAllProfiles&&o.startProfile>h.profileCount)return fail(e,QStringLiteral("Profile range starts past end of scan"));
    if(!o.loadAllProfiles&&o.profileCount==0){data=std::move(tmp);return true;} // legacy read-options contract: header-only
    QByteArray raw=decompress(f.read(qint64(stored)),h.compression,std::size_t(h.uncompressedSize));
    if(raw.isEmpty())return fail(e,QStringLiteral("SCAN decompression/size check failed"));
    const auto begin=o.loadAllProfiles?0:o.startProfile;
    const auto count=o.loadAllProfiles?h.profileCount:std::min(o.profileCount,h.profileCount-begin);
    if(h.version>=3){std::vector<ArchiveProfile> pp;std::string err;
        if(!decodeArchive(reinterpret_cast<const std::uint8_t*>(raw.data()),std::size_t(raw.size()),pp,err))return fail(e,QString::fromStdString(err));
        if(pp.size()!=h.profileCount)return fail(e,QStringLiteral("SCAN profile count mismatch"));
        for(auto i=begin;i<begin+count;++i){ProfileData p;p.hasFiveAxis=true;p.fiveAxis=std::move(pp[std::size_t(i)]);tmp.profiles.push_back(std::move(p));}
    }else{
        const auto n=calculateProfileDataSize(h.pointsPerProfile);
        if(!h.pointsPerProfile||h.profileCount>ArchiveMaxBytes/n||h.profileCount*n!=std::uint64_t(raw.size()))return fail(e,QStringLiteral("Invalid legacy XYZ payload size"));
        for(auto i=begin;i<begin+count;++i){const char* p=raw.constData()+i*n;ProfileData v;
            std::memcpy(&v.machineX,p,4);std::memcpy(&v.machineY,p+4,4);std::memcpy(&v.machineZ,p+8,4);p+=12;
            v.sensorX.resize(h.pointsPerProfile);v.sensorZ.resize(h.pointsPerProfile);
            std::memcpy(v.sensorX.data(),p,2*h.pointsPerProfile);std::memcpy(v.sensorZ.data(),p+2*h.pointsPerProfile,2*h.pointsPerProfile);tmp.profiles.push_back(std::move(v));
        }
    }
    data=std::move(tmp);return true;
}
ScanData CompactScanIO::fromAlignedPoints(const QVector<AlignedPoint>& pts,uint64_t start,uint32_t interval,const QMatrix4x4& he){
    Q_UNUSED(he);ScanData d;d.header.sampleIntervalUs=interval;d.header.startTimeUs=start?start:(pts.isEmpty()?0:pts.first().tUs);
    // A .scan file is one authoritative five-axis capture. Never mix sessions or
    // different Physical17/source-mask provenance into a single archive.
    bool haveBaseline=false; Frame5D baseline;
    // Group by capture session AND ordinal. Profile indexes can repeat after a sensor reset.
    std::map<std::tuple<std::uint64_t,std::uint64_t,std::uint32_t>,std::size_t> groups;
    for(const auto& p:pts){
        if(!p.frame){d.validationError=QStringLiteral("Aligned point lacks synchronized XYZCB/17-parameter provenance; refusing lossy reconstruction");return d;}
        if(!haveBaseline){baseline=*p.frame;haveBaseline=true;}
        else if(!sameScanProvenance(baseline,*p.frame)){d.validationError=QStringLiteral("Five-axis scan mixes sessions, Physical17 parameters, rotary offsets, timing or source masks");return d;}
        auto key=std::make_tuple(p.frame->session,p.frame->ordinal,p.profileIndex);auto it=groups.find(key);
        if(it==groups.end()){ProfileData v;v.hasFiveAxis=true;v.fiveAxis.frame=*p.frame;v.fiveAxis.timeUs=p.tUs;v.fiveAxis.sensorTimeSec=p.tSensorSec;v.fiveAxis.profileIndex=p.profileIndex;auto index=std::size_t(d.profiles.size());d.profiles.push_back(v);it=groups.emplace(key,index).first;}
        auto& rec=d.profiles[qsizetype(it->second)].fiveAxis;
        if(rec.frame.joints!=p.frame->joints||rec.frame.sensorToMachine!=p.frame->sensorToMachine){d.validationError=QStringLiteral("One profile contains conflicting pose/transform snapshots");return d;}
        auto xyz=mapProfilePoint(*p.frame,p.sensorX,p.sensorZ);
        if(!std::isfinite(p.machineX)||!std::isfinite(p.machineY)||!std::isfinite(p.machineZ)||std::abs(xyz[0]-p.machineX)>1e-8||std::abs(xyz[1]-p.machineY)>1e-8||std::abs(xyz[2]-p.machineZ)>1e-8){d.validationError=QStringLiteral("Point coordinate/provenance mismatch");return d;}
        rec.points.push_back({p.pointIndex,p.sensorX,p.sensorZ});
        d.header.pointsPerProfile=std::uint16_t(std::min<std::size_t>(65535,std::max<std::size_t>(d.header.pointsPerProfile,rec.points.size())));
    }
    d.header.profileCount=d.profiles.size();return d;
}
ScanData CompactScanIO::fromProfilesAndPoses(const QVector<LaserProfile>& profiles,const QVector<SamplePoint>& poses,uint64_t start,uint32_t interval,const QMatrix4x4& he){
    Q_UNUSED(he);ScanData d;d.header.startTimeUs=start;d.header.sampleIntervalUs=interval;
    if(profiles.size()!=poses.size()){d.validationError=QStringLiteral("Profile/pose counts differ");return d;}
    bool haveBaseline=false; Frame5D baseline;
    for(qsizetype i=0;i<profiles.size();++i){
        if(!poses[i].hasRotary||!validTransform(poses[i].frame.sensorToMachine)){d.validationError=QStringLiteral("Profiles require calibrated five-axis frame snapshots, not raw polling poses");return d;}
        if(!haveBaseline){baseline=poses[i].frame;haveBaseline=true;}
        else if(!sameScanProvenance(baseline,poses[i].frame)){d.validationError=QStringLiteral("Five-axis scan mixes sessions, Physical17 parameters, rotary offsets, timing or source masks");return d;}
        const auto& p=profiles[i];if(p.x.size()!=p.z.size()){d.validationError=QStringLiteral("Profile XZ lengths differ");return d;}
        ProfileData v;v.hasFiveAxis=true;v.fiveAxis.frame=poses[i].frame;v.fiveAxis.profileIndex=p.index;v.fiveAxis.timeUs=p.tPcUs;v.fiveAxis.sensorTimeSec=p.tOpen;
        for(qsizetype k=0;k<p.x.size();++k)if(SensorUtils::isValidPoint(p.x[k],p.z[k]))v.fiveAxis.points.push_back({std::uint32_t(k),p.x[k],p.z[k]});
        d.profiles.push_back(std::move(v));
    }
    d.header.profileCount=d.profiles.size();return d;
}
QVector<QVector3D> PointCloudReconstructor::reconstruct(const ScanData& d){return reconstructRange(d,0,0);}
QVector<QVector3D> PointCloudReconstructor::reconstructRange(const ScanData& d,uint64_t start,uint64_t count){
    QVector<QVector3D> out;auto n=std::uint64_t(d.profiles.size());if(start>=n)return out;auto end=start+(count?std::min(count,n-start):n-start);
    for(auto i=start;i<end;++i)out.append(reconstructProfile(d.header,d.profiles[qsizetype(i)]));return out;
}
QVector<QVector3D> PointCloudReconstructor::reconstructProfile(const ScanFileHeader& h,const ProfileData& p){
    QVector<QVector3D> out;
    if(p.hasFiveAxis){if(!validTransform(p.fiveAxis.frame.sensorToMachine))return out;out.reserve(qsizetype(p.fiveAxis.points.size()));for(const auto& q:p.fiveAxis.points){auto x=mapProfilePoint(p.fiveAxis.frame,q.x,q.z);out.push_back(QVector3D(float(x[0]),float(x[1]),float(x[2])));}return out;}
    // Explicit read-only legacy XYZ convention; no C/B or recoverable hand-eye existed in v2.
    const auto n=std::min(p.sensorX.size(),p.sensorZ.size());for(qsizetype i=0;i<n;++i){if(p.sensorX[i]==0&&p.sensorZ[i]==0)continue;
        double x=p.sensorX[i]*h.xScale+h.xOffset,z=p.sensorZ[i]*h.zScale+h.zOffset;
        if(SensorUtils::isValidPoint(x,z))out.push_back(QVector3D(p.machineX,float(p.machineY+x),float(p.machineZ+z)));
    }return out;
}
uint64_t PointCloudReconstructor::getTimestamp(const ScanFileHeader& h,uint64_t index){return h.startTimeUs+index*h.sampleIntervalUs;} // v3 exact time is ProfileData::fiveAxis.timeUs
