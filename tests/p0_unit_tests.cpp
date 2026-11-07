#include "TestSupport.h"
#include "core/ProfileCounter24.h"
#include "core/ProfileSessionEpoch.h"
#include "core/TableRingRead.h"
#include "core/ControllerIoGate.h"
#include "runtime/CalibrationPersistence.h"
#include <future>
#include <limits>

using Counter = p0::ProfileCounter24;
using Kind = Counter::Kind;
using Status = p0::TableReadStatus;
struct Pending { int payload; std::int64_t poseOrdinal; };

int main() {
    return runTests({
        {"counter starts at zero", [] { Counter c; auto a=c.observe(0); REQUIRE(a.kind==Kind::First && a.ordinal==0); REQUIRE(c.observe(1).ordinal==1); }},
        {"counter starts at arbitrary index", [] { Counter c; c.observe(123); REQUIRE(c.observe(124).ordinal==1); }},
        {"counter preserves forward gap", [] { Counter c; c.observe(100); auto a=c.observe(103); REQUIRE(a.kind==Kind::Gap && a.missing==2 && a.ordinal==3); }},
        {"counter drops duplicate without advancing", [] { Counter c; c.observe(5); REQUIRE(c.observe(5).kind==Kind::Duplicate); REQUIRE(c.observe(6).ordinal==1); }},
        {"counter rejects backward/reset", [] { Counter c; c.observe(1000); REQUIRE(c.observe(0).kind==Kind::BackwardOrReset); REQUIRE(c.observe(1001).ordinal==1); }},
        {"counter rejects invalid width", [] { Counter c; REQUIRE(c.observe(1U<<24).kind==Kind::Invalid); REQUIRE(!c.started()); }},
        {"counter wraps at 24 bits", [] { Counter c; c.observe(Counter::kMask-1); c.observe(Counter::kMask); REQUIRE(c.observe(0).ordinal==2); REQUIRE(c.observe(1).ordinal==3); }},
        {"counter rejects half-cycle ambiguity", [] { Counter c; c.observe(0); REQUIRE(c.observe(Counter::kModulus/2).kind==Kind::BackwardOrReset); }},
        {"counter supports successive complete cycles", [] { Counter c; c.observe(0); Counter::Observation a; for (int i=1;i<=12;++i) a=c.observe((i*(1U<<22)) & Counter::kMask); REQUIRE(a.ordinal==3LL*Counter::kModulus); }},
        {"counter resets only explicitly", [] { Counter c; c.observe(10); c.observe(20); c.reset(); REQUIRE(c.observe(20).ordinal==0); }},
        {"FIFO retains several not-ready profiles", [] { std::deque<Pending> q{{10,0},{11,1}}; Pending p{}; REQUIRE(!p0::takeReadyFront(q,0,p)); REQUIRE(q[0].payload==10 && q[1].payload==11); REQUIRE(p0::takeReadyFront(q,2,p) && p.payload==10); REQUIRE(p0::takeReadyFront(q,2,p) && p.payload==11); }},
        {"FIFO waits at front without skipping", [] { std::deque<Pending> q{{12,2},{13,3}}; Pending p{}; REQUIRE(!p0::takeReadyFront(q,2,p)); REQUIRE(q.size()==2); REQUIRE(p0::takeReadyFront(q,3,p) && p.poseOrdinal==2); REQUIRE(!p0::takeReadyFront(q,3,p)); }},
        {"session ignores inactive callbacks", [] { p0::ProfileSessionEpoch s; REQUIRE(!s.accepts(s.current())); }},
        {"session rejects old queued callbacks after restart", [] { p0::ProfileSessionEpoch s; auto old=s.begin(); REQUIRE(s.accepts(old)); s.end(); auto now=s.begin(); REQUIRE(now!=old && !s.accepts(old) && s.accepts(now)); }},
        {"session stop is idempotent", [] { p0::ProfileSessionEpoch s; s.begin(); s.end(); auto x=s.current(); s.end(); REQUIRE(x==s.current()); }},
        {"valid V17 ring header", [] { float v[4]={420,2,418,0}; p0::TableRingHeader h; REQUIRE(p0::parseTableRingHeader(v,420,h)); REQUIRE(h.capacity==420 && h.read==418); }},
        {"invalid ring header rejected before casts", [] { p0::TableRingHeader h; for(float bad:{-1.f,2.5f,421.f,(std::numeric_limits<float>::infinity)(),(std::numeric_limits<float>::quiet_NaN)()}) { float v[4]={bad,0,0,0}; REQUIRE(!p0::parseTableRingHeader(v,420,h)); } }},
        {"ring rejects out-of-range index", [] { float v[4]={420,420,0,0}; p0::TableRingHeader h; REQUIRE(!p0::parseTableRingHeader(v,420,h)); }},
        {"empty ring performs no I/O", [] { int calls=0; auto b=p0::readTableRingBatch({8,2,2,false},3,[&](int,int,float*){++calls;return true;},[&](int){++calls;return true;}); REQUIRE(b.status==Status::Ok && calls==0 && b.records==0); }},
        {"contiguous ring commit", [] { int ack=-1; auto b=p0::readTableRingBatch({8,5,2,false},3,[](int s,int n,float* p){for(int i=0;i<n*3;++i)p[i]=float(s*3+i);return true;},[&](int r){ack=r;return true;}); REQUIRE(b.status==Status::Ok && b.records==3 && ack==5); REQUIRE(b.values.front()==6 && b.values.back()==14); }},
        {"wrapped ring reads tail then head", [] { std::vector<int> starts; auto b=p0::readTableRingBatch({8,2,6,false},3,[&](int s,int n,float* p){starts.push_back(s);for(int i=0;i<n*3;++i)p[i]=float(s*3+i);return true;},[](int r){return r==2;}); REQUIRE(b.records==4 && starts==std::vector<int>({6,0})); REQUIRE(b.values[0]==18 && b.values[6]==0); }},
        {"first data read failure publishes nothing", [] { int ack=0; auto b=p0::readTableRingBatch({8,2,6,false},3,[](int,int,float*){return false;},[&](int){++ack;return true;}); REQUIRE(b.status==Status::ReadFailed && b.values.empty() && b.records==0 && ack==0); }},
        {"second wrap read failure publishes nothing", [] { int reads=0,acks=0; auto b=p0::readTableRingBatch({8,2,6,false},3,[&](int,int,float*){return ++reads!=2;},[&](int){++acks;return true;}); REQUIRE(b.status==Status::ReadFailed && b.values.empty() && b.records==0 && acks==0); }},
        {"ACK ambiguity is distinct from safe read retry", [] { auto b=p0::readTableRingBatch({8,1,0,false},3,[](int,int,float*){return true;},[](int){return false;}); REQUIRE(b.status==Status::AckUncertain && b.values.empty()); }},
        {"sticky overflow never reads or acknowledges", [] { int calls=0; auto b=p0::readTableRingBatch({8,2,1,true},3,[&](int,int,float*){++calls;return true;},[&](int){++calls;return true;}); REQUIRE(b.status==Status::Overflow && calls==0); }},
        {"I/O nested helpers use recursive lock", [] { p0::IoMutex m; p0::IoLock a(m); {p0::IoLock b(m);} REQUIRE(a.owns_lock()); }},
        {"I/O wait release admits stop worker", [] { p0::IoMutex m; p0::IoLock a(m); std::promise<void> entered; auto f=entered.get_future(); std::thread t([&]{p0::IoLock l(m);entered.set_value();}); bool admitted; {p0::ScopedIoRelease yield(a); admitted=f.wait_for(std::chrono::seconds(1))==std::future_status::ready;} a.unlock();t.join(); REQUIRE(admitted); }},
        {"I/O lock reacquired after exception", [] { p0::IoMutex m; p0::IoLock a(m); try {p0::ScopedIoRelease yield(a);throw 1;} catch(int) {} REQUIRE(a.owns_lock()); }},
        {"V27.6.14 compact feedback masks stay unchanged", [] {
            using namespace mls::runtime;
            for (int raw : {0, 8, 16, 24, 31}) {
                const auto d=decodeFeedbackMask(raw,FeedbackMaskEncoding::CompactXYZCB);
                REQUIRE(d.ok && d.normalized==raw && !d.migratedLegacyOneBased);
            }
        }},
        {"V27.6.14 legacy one-based masks migrate by schema even below 32", [] {
            using namespace mls::runtime;
            const auto cOnly=decodeFeedbackMask(16,FeedbackMaskEncoding::LegacyControllerAxisOneBased);
            const auto cb=decodeFeedbackMask(48,FeedbackMaskEncoding::LegacyControllerAxisOneBased);
            REQUIRE(cOnly.ok && cOnly.normalized==8 && cOnly.migratedLegacyOneBased);
            REQUIRE(cb.ok && cb.normalized==24 && cb.migratedLegacyOneBased);
        }},
        {"V27.6.14 malformed legacy masks are rejected", [] {
            using namespace mls::runtime;
            REQUIRE(!decodeFeedbackMask(17,FeedbackMaskEncoding::LegacyControllerAxisOneBased).ok);
            REQUIRE(!decodeFeedbackMask(63,FeedbackMaskEncoding::LegacyControllerAxisOneBased).ok);
            REQUIRE(!decodeFeedbackMask(32,FeedbackMaskEncoding::CompactXYZCB).ok);
        }}
    });
}
