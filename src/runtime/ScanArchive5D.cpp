#include "runtime/ScanArchive5D.h"
#include <cstring>
#include <limits>
#include <set>
#include <stdexcept>
namespace mls::runtime {
std::uint32_t archiveCrc32(const void* data,std::size_t size) {
    // Function-local static initialization is thread-safe in C++11 and later.
    static const auto table=[] {std::array<std::uint32_t,256> t{};for(unsigned i=0;i<256;++i){auto c=i;for(int j=0;j<8;++j)c=(c>>1)^((c&1)?0xedb88320u:0);t[i]=c;}return t;}();
    auto p=static_cast<const std::uint8_t*>(data);std::uint32_t c=~0u;
    for(std::size_t i=0;i<size;++i)c=table[(c^p[i])&255]^(c>>8);
    return ~c;
}
namespace {
static_assert(sizeof(double)==8 && std::numeric_limits<double>::is_iec559,"IEEE754 binary64 required");
struct Writer {
    std::vector<std::uint8_t> v;
    void u(std::uint64_t x,int n){if(v.size()+n>ArchiveMaxBytes)throw std::runtime_error("archive exceeds 512 MiB bound; split the scan");for(int i=0;i<n;++i)v.push_back(std::uint8_t(x>>(8*i)));}
    void d(double x){if(!std::isfinite(x))throw std::runtime_error("non-finite archive value");std::uint64_t b;std::memcpy(&b,&x,8);u(b,8);}
};
struct Reader {
    const std::uint8_t* p;std::size_t n,k=0;
    std::uint64_t u(int bytes){if(std::size_t(bytes)>n-k)throw std::runtime_error("truncated archive");std::uint64_t x=0;for(int i=0;i<bytes;++i)x|=std::uint64_t(p[k++])<<(8*i);return x;}
    double d(){auto b=u(8);double x;std::memcpy(&x,&b,8);if(!std::isfinite(x))throw std::runtime_error("non-finite archive value");return x;}
};
void validate(const ArchiveProfile& p){
    if(!finiteJoint(p.frame.joints)||!finiteJoint(p.frame.raw)||!finiteJoint(p.frame.command)||!validTransform(p.frame.sensorToMachine))throw std::runtime_error("missing/invalid five-axis profile transform");
    auto t=static_cast<unsigned>(p.frame.timing);
    if(t<1||t>5||p.frame.feedbackMask>31||p.frame.latchMask>31||p.frame.commandMask>31||p.points.size()>1000000)throw std::runtime_error("invalid five-axis profile metadata");
    // Simulated records intentionally carry an explicitly supplied virtual
    // exposure transform. This may be nominal/anchored and must NOT claim
    // hardware feedback, latch, absolute gauge or temporal calibration.
    // Crucially, NO relaxation applies to any hardware/software measurement.
    if(p.frame.timing==PoseTiming::Simulated){
        if(p.frame.feedbackMask!=0 || p.frame.latchMask!=0 || p.frame.commandMask!=0 ||
           p.frame.absoluteGaugeKnown || p.frame.temporalOffsetSec!=0 ||
           p.frame.temporalAlpha!=0 || p.frame.temporalNextOrdinal!=0 ||
           p.frame.rotaryOffsets!=std::array<double,2>{0.,0.})
            throw std::runtime_error("simulated frame improperly claims hardware provenance");
        for(double x:p.frame.physicalParameters)
            if(!std::isfinite(x))throw std::runtime_error("non-finite simulation parameter");
    }else{
        // Hardware/software captures retain the original strict Physical17
        // consistency validation: NO tolerance or source-mask changes.
        Frame5D expected=p.frame;std::string e;
        if(!completeFrame(expected,p.frame.physicalParameters,e))throw std::runtime_error(e);
        for(int i=0;i<16;++i)
            if(std::abs(expected.sensorToMachine[i]-p.frame.sensorToMachine[i])>1e-8)
                throw std::runtime_error("profile transform differs from recorded 17-parameter model");
    }
    if(p.frame.timing==PoseTiming::Simulated){
        // Fixed 1280-element profile: O(1) index check, avoiding millions of
        // costly tree allocations during streaming simulated capture.
        std::array<bool,1280> seen{};
        for(const auto& q:p.points){
            if(!std::isfinite(q.x)||!std::isfinite(q.z)||q.index>=seen.size()||seen[q.index])
                throw std::runtime_error("invalid/duplicate simulated sample index");
            seen[q.index]=true;
        }
    }else{
        std::set<std::uint32_t> seen;
        for(const auto& q:p.points)
            if(!std::isfinite(q.x)||!std::isfinite(q.z)||!seen.insert(q.index).second)
                throw std::runtime_error("invalid/duplicate sensor point index");
    }
}
}
bool encodeArchive(const std::vector<ArchiveProfile>& profiles,std::vector<std::uint8_t>& out,std::string& error){
    out.clear();error.clear();try{
        if(profiles.size()>1000000)throw std::runtime_error("too many profiles");
        Writer w;w.u(0x34443541u,4);w.u(5,4);w.u(profiles.size(),8);
        for(const auto& p:profiles){validate(p);const auto& f=p.frame;
            w.u(f.session,8);w.u(f.ordinal,8);w.u(std::uint64_t(f.hostEstimateUs),8);w.u(std::uint64_t(p.timeUs),8);
            w.u(p.profileIndex,4);w.u(static_cast<unsigned>(f.timing),4);w.u(f.feedbackMask,4);w.u(f.latchMask,4);w.u(f.commandMask,4);w.u(f.absoluteGaugeKnown?1:0,4);w.u(p.points.size(),4);w.d(p.sensorTimeSec);
            w.d(f.temporalOffsetSec);w.d(f.temporalAlpha);w.u(f.temporalNextOrdinal,8);
            for(const auto* a:{&f.joints,&f.command,&f.raw})for(double x:*a)w.d(x);
            for(double x:f.rotaryOffsets)w.d(x);
            for(double x:f.sensorToMachine)w.d(x);
            for(double x:f.physicalParameters)w.d(x);
            for(const auto& q:p.points){w.u(q.index,4);w.d(q.x);w.d(q.z);}
        }
        auto crc=archiveCrc32(w.v.data(),w.v.size());w.u(crc,4);out=std::move(w.v);return true;
    }catch(const std::exception& e){error=e.what();return false;}
}
bool decodeArchive(const std::uint8_t* bytes,std::size_t size,std::vector<ArchiveProfile>& out,std::string& error){
    out.clear();error.clear();try{
        if(!bytes||size<20||size>ArchiveMaxBytes)throw std::runtime_error("invalid archive size");
        Reader tail{bytes+size-4,4};if(tail.u(4)!=archiveCrc32(bytes,size-4))throw std::runtime_error("archive CRC mismatch");
        Reader r{bytes,size-4};
        if(r.u(4)!=0x34443541u)throw std::runtime_error("unknown archive magic");
        const auto payloadVersion=r.u(4);
        if(payloadVersion!=4 && payloadVersion!=5)throw std::runtime_error("unknown archive payload version");
        const auto count=r.u(8);
        const std::size_t fixedProfile=payloadVersion>=5 ? 492u : 468u;
        if(count>1000000||count>(r.n-r.k)/fixedProfile)throw std::runtime_error("impossible archive profile count");
        std::vector<ArchiveProfile> result;result.reserve(std::size_t(count));
        for(std::uint64_t i=0;i<count;++i){ArchiveProfile p;auto& f=p.frame;
            f.session=r.u(8);f.ordinal=r.u(8);f.hostEstimateUs=static_cast<std::int64_t>(r.u(8));p.timeUs=static_cast<std::int64_t>(r.u(8));
            p.profileIndex=std::uint32_t(r.u(4));f.timing=static_cast<PoseTiming>(r.u(4));f.feedbackMask=std::uint32_t(r.u(4));f.latchMask=std::uint32_t(r.u(4));f.commandMask=std::uint32_t(r.u(4));auto gauge=r.u(4);if(gauge>1)throw std::runtime_error("invalid gauge flag");f.absoluteGaugeKnown=gauge!=0;auto n=r.u(4);p.sensorTimeSec=r.d();
            if(payloadVersion>=5){f.temporalOffsetSec=r.d();f.temporalAlpha=r.d();f.temporalNextOrdinal=r.u(8);}
            if(n>1000000||n>(r.n-r.k)/20)throw std::runtime_error("impossible sensor point count");
            for(auto* a:{&f.joints,&f.command,&f.raw})for(double& x:*a)x=r.d();
            for(double& x:f.rotaryOffsets)x=r.d();
            for(double& x:f.sensorToMachine)x=r.d();
            for(double& x:f.physicalParameters)x=r.d();
            p.points.reserve(std::size_t(n));for(std::uint64_t j=0;j<n;++j){ArchivePoint q;q.index=std::uint32_t(r.u(4));q.x=r.d();q.z=r.d();p.points.push_back(q);}
            validate(p);result.push_back(std::move(p));
        }
        if(r.k!=r.n)throw std::runtime_error("unexpected trailing archive bytes");
        out=std::move(result);return true;
    }catch(const std::exception& e){error=e.what();return false;}
}
} // namespace mls::runtime
