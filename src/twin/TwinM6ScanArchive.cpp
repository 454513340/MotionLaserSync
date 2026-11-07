#include "twin/TwinM6ScanArchive.h"
#include "runtime/Acquisition5D.h"
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <cmath>
#include <limits>

namespace mls::twin {
namespace {
bool fail(std::string* error,const QString& message){if(error)*error=message.toStdString();return false;}
}
M6ScanVolumeWriter::M6ScanVolumeWriter(const QString& folder,double hz,
    const std::array<double,17>& p,std::uint64_t session,
    std::atomic<std::size_t>* ready)
    :directory_(folder),scanRateHz_(hz),physical_(p),session_(session),volumesReady_(ready) {
    chunk_.header.pointsPerProfile=1280;
    chunk_.header.sampleIntervalUs=static_cast<std::uint32_t>(std::llround(1e6/hz));
}

bool M6ScanVolumeWriter::append(const VirtualProfile& p,std::size_t segment,std::string* error){
    if(segment==0 || p.samples.size()!=1280 || !std::isfinite(p.pose.elapsedSeconds)||
       p.pose.elapsedSeconds<0 || !mls::runtime::validTransform(p.pose.sensorToMachine.m))
        return fail(error,QStringLiteral("M6 archive: malformed simulated profile pose/samples"));
    for(double v:p.pose.joints)if(!std::isfinite(v))
        return fail(error,QStringLiteral("M6 archive: non-finite simulated XYZCB"));
    if(ordinal_>=1000000)
        return fail(error,QStringLiteral("M6 archive: simulated ordinal exceeded archive limit"));
    std::uint64_t nvalid=0;
    for(const auto& point:p.samples){
        if(point.state!=VirtualSampleState::Valid)continue;
        if(!std::isfinite(point.sensorX)||!std::isfinite(point.sensorZ))
            return fail(error,QStringLiteral("M6 archive: non-finite valid sensor sample"));
        ++nvalid;
    }
    const auto bytesNeeded=static_cast<std::uint64_t>(550+20*nvalid);
    if(!chunk_.profiles.isEmpty() &&
       (segment!=activeSegment_ || rawEstimate_+bytesNeeded>kSafeUncompressedBudget)){
        if(!flush(error))return false;
    }
    if(chunk_.profiles.isEmpty()){
        activeSegment_=segment;
        ++part_;
    }
    ProfileData rec;
    rec.hasFiveAxis=true;
    auto& a=rec.fiveAxis;
    auto& f=a.frame;
    f.session=session_;
    f.ordinal=++ordinal_;
    f.joints=p.pose.joints;
    f.command=f.joints;f.raw=f.joints;
    f.timing=mls::runtime::PoseTiming::Simulated;
    f.feedbackMask=0;f.latchMask=0;f.commandMask=0;
    f.absoluteGaugeKnown=false;
    f.sensorToMachine=p.pose.sensorToMachine.m;
    f.physicalParameters=physical_; // diagnostic parameters, NOT hardware provenance
    // Simulated monotonic time. Never claim a hardware timestamp.
    const double us=std::llround(p.pose.elapsedSeconds*1e6);
    if(us<0 || us>double(std::numeric_limits<std::int64_t>::max()))
        return fail(error,QStringLiteral("M6 archive: time overflow"));
    a.timeUs=static_cast<std::int64_t>(us);
    f.hostEstimateUs=a.timeUs;
    a.sensorTimeSec=p.pose.elapsedSeconds;
    a.profileIndex=static_cast<std::uint32_t>(f.ordinal-1);
    a.points.reserve(static_cast<std::size_t>(nvalid));
    for(std::size_t index=0;index<p.samples.size();++index){
        const auto& sample=p.samples[index];
        if(sample.state!=VirtualSampleState::Valid)continue;
        a.points.push_back({static_cast<std::uint32_t>(index),sample.sensorX,sample.sensorZ});
    }
    if(chunk_.profiles.isEmpty())chunk_.header.startTimeUs=static_cast<std::uint64_t>(a.timeUs);
    chunk_.profiles.push_back(std::move(rec));
    rawEstimate_+=bytesNeeded;
    totalValid_+=nvalid;
    return true;
}

bool M6ScanVolumeWriter::flush(std::string* error){
    if(chunk_.profiles.isEmpty())return true;
    if(!QDir().mkpath(directory_))
        return fail(error,QStringLiteral("M6 archive: unable to create temporary directory"));
    const QString filename=QStringLiteral("seg%1_part%2.scan")
        .arg(static_cast<qulonglong>(activeSegment_),2,10,QChar('0'))
        .arg(static_cast<qulonglong>(part_),4,10,QChar('0'));
    const QString path=QDir(directory_).filePath(filename);
    chunk_.header.profileCount=static_cast<std::uint64_t>(chunk_.profiles.size());
    CompactScanIO::WriteOptions options;
    options.compression=CompressionType::QtZlib;
    options.compressionLevel=6;
    options.scanName=QStringLiteral("M6_SIMULATED_OFFLINE");
    options.blockName=QStringLiteral("M6_SIMULATED");
    options.blockLineNumber=static_cast<int>(activeSegment_);
    QString why;
    if(!CompactScanIO::save(path,chunk_,options,&why))
        return fail(error,QStringLiteral("M6 archive write: ")+why);
    ScanFileHeader check;
    if(!CompactScanIO::readHeader(path,check,&why) ||
       check.version!=3 || check.profileCount!=static_cast<std::uint64_t>(chunk_.profiles.size()))
        return fail(error,QStringLiteral("M6 archive header verification: ")+why);
    M6ScanVolume volume;
    volume.temporaryPath=path;volume.scanSegmentNumber=activeSegment_;
    volume.partNumber=part_;
    volume.firstOrdinal=chunk_.profiles.first().fiveAxis.frame.ordinal;
    volume.lastOrdinal=chunk_.profiles.last().fiveAxis.frame.ordinal;
    volume.profiles=static_cast<std::uint64_t>(chunk_.profiles.size());
    for(const auto& p:chunk_.profiles)volume.validPoints+=p.fiveAxis.points.size();
    volume.storedBytes=static_cast<std::uint64_t>(QFileInfo(path).size());
    volumes_.push_back(std::move(volume));
    if(volumesReady_)volumesReady_->store(volumes_.size());
    chunk_.profiles.clear(); // release entire chunk buffer after each volume
    chunk_.profiles.squeeze();
    rawEstimate_=16;
    return true;
}

bool M6ScanVolumeWriter::finish(std::string* error){
    if(!flush(error))return false;
    if(ordinal_==0 || volumes_.empty())
        return fail(error,QStringLiteral("M6 archive: no recorded simulated profiles"));
    // Verify first and last volume can actually be decoded by the existing
    // production SCAN v3 reader (not just by the writer/header CRC).
    for(const std::size_t i:{std::size_t(0),volumes_.size()-1}){
        ScanData back;QString why;
        if(!CompactScanIO::load(volumes_[i].temporaryPath,back,{},&why))
            return fail(error,QStringLiteral("M6 archive round-trip: ")+why);
        if(static_cast<std::uint64_t>(back.profiles.size())!=volumes_[i].profiles ||
           back.profiles.first().fiveAxis.frame.timing!=mls::runtime::PoseTiming::Simulated ||
           back.profiles.first().fiveAxis.frame.feedbackMask!=0)
            return fail(error,QStringLiteral("M6 archive round-trip metadata mismatch"));
    }
    return true;
}
} // namespace mls::twin
