#include "TestSupport.h"
#include "core/EciIoScheduler.h"
#include <atomic>
#include <chrono>
#include <future>
#include <thread>
#include <vector>
using S = eci::EciIoScheduler;
using P = S::Priority;
using namespace std::chrono_literals;
namespace {
S::Options opt(P p, const char* name, unsigned domain = S::None) {
    auto o = S::options(p, name, domain); o.deadline = 10s; return o;
}
bool fails(std::future<bool>& f) { try { (void)f.get(); return false; } catch (const std::exception&) { return true; } }
struct Barrier {
    struct State { std::promise<void> entered, release; std::shared_future<void> ready = release.get_future().share(); };
    std::shared_ptr<State> state = std::make_shared<State>();
    std::future<bool> task; bool released = false;
    explicit Barrier(S& s) {
        auto start = state->entered.get_future();
        task = s.submit(opt(P::Control,"test.blockedTransaction"),s.token(),[v=state] {
            v->entered.set_value(); v->ready.wait(); return true;
        });
        if (start.wait_for(2s) != std::future_status::ready) { open(); throw std::runtime_error("test barrier did not start"); }
    }
    void open() { if (!released) { state->release.set_value(); released = true; } }
    ~Barrier() { open(); if (task.valid()) task.wait(); }
};
}
int main() { return runTests({
 {"one owner thread handles concurrent callers", [] {
    S s; std::thread::id owner; std::atomic<int> active{0},maximum{0};
    std::vector<std::future<bool>> jobs;
    for (int i=0;i<100;++i) jobs.push_back(s.submit(opt(P::Control,"read"),s.token(),[&] {
        const auto id=std::this_thread::get_id(); if(owner==std::thread::id{})owner=id;
        if(owner!=id)return false; const int a=++active; if(a>maximum)maximum=a; --active; return true;
    }));
    for(auto& j:jobs) REQUIRE(j.get()); REQUIRE(maximum==1 && owner!=std::this_thread::get_id());
 }},
 {"emergency wins next but cannot preempt in-flight DLL transaction", [] {
    S s; Barrier b(s); std::vector<int> order;
    auto ui=s.submit(opt(P::Diagnostic,"ui"),s.token(),[&]{order.push_back(4);return true;});
    auto table=s.submit(opt(P::Measurement,"table"),s.token(),[&]{order.push_back(2);return true;});
    auto motion=s.submit(opt(P::Motion,"motion"),s.token(),[&]{order.push_back(1);return true;});
    auto stop=s.submit(opt(P::Emergency,"stop"),s.token(),[&]{order.push_back(0);return true;});
    REQUIRE(stop.wait_for(5ms)==std::future_status::timeout);
    b.open(); REQUIRE(stop.get() && motion.get() && table.get() && ui.get());
    REQUIRE(order==std::vector<int>({0,1,2,4}));
 }},
 {"STOP admission atomically cancels old motion and acquisition", [] {
    S s; Barrier b(s); const auto old=s.token(); std::atomic<int> ran{0};
    auto move=s.submit(opt(P::Motion,"old.move",S::Motion),old,[&]{++ran;return true;});
    auto enable=s.submit(opt(P::Measurement,"old.enable",S::Acquisition),old,[&]{++ran;return true;});
    auto o=opt(P::Emergency,"STOP");o.invalidateDomains=S::Motion|S::Acquisition;
    auto stop=s.submit(o,old,[]{return true;});
    REQUIRE(fails(move) && fails(enable)); b.open(); REQUIRE(stop.get() && ran==0);
 }},
 {"late continuation cannot restart motion after STOP", [] {
    S s; const auto old=s.token(); s.invalidate(S::Motion);
    auto j=s.submit(opt(P::Motion,"home.secondPhase",S::Motion),old,[]{return true;}); REQUIRE(fails(j));
 }},
 {"new explicit command uses new generation after STOP", [] {
    S s; s.invalidate(S::Motion); REQUIRE(s.invoke(opt(P::Motion,"new.manual",S::Motion),s.token(),[]{return true;}));
 }},
 {"acquisition-only cancellation does not drop independent motion", [] {
    S s; Barrier b(s); auto old=s.token();
    auto a=s.submit(opt(P::Measurement,"enable",S::Acquisition),old,[]{return true;});
    auto m=s.submit(opt(P::Motion,"move",S::Motion),old,[]{return true;});
    s.invalidate(S::Acquisition); REQUIRE(fails(a)); b.open(); REQUIRE(m.get());
 }},
 {"connection generation rejects all stale session reads", [] {
    S s; Barrier b(s);auto old=s.token();auto read=s.submit(opt(P::Diagnostic,"old.read"),old,[]{return true;});
    s.newSession(); REQUIRE(fails(read)); b.open(); REQUIRE(s.invoke(opt(P::Control,"new.read"),s.token(),[]{return true;}));
 }},
 {"lifecycle barrier is not bound to discarded session", [] {
    S s; Barrier b(s);auto o=opt(P::Emergency,"close");o.sessionBound=false;
    auto j=s.submit(o,s.token(),[]{return true;}); s.newSession();b.open(); REQUIRE(j.get());
 }},
 {"normal queue bounded and reserved STOP still admitted", [] {
    S s(2,1); Barrier b(s);
    auto a=s.submit(opt(P::Control,"a"),s.token(),[]{return true;});
    auto c=s.submit(opt(P::Control,"b"),s.token(),[]{return true;});
    auto rejected=s.submit(opt(P::Control,"full"),s.token(),[]{return true;}); REQUIRE(fails(rejected));
    auto stop=s.submit(opt(P::Emergency,"stop"),s.token(),[]{return true;});
    auto rejectedStop=s.submit(opt(P::Emergency,"fullStopReserve"),s.token(),[]{return true;}); REQUIRE(fails(rejectedStop));
    b.open(); REQUIRE(stop.get() && a.get() && c.get());
 }},
 {"overdue TABLE is not starved by fresh trajectory refills", [] {
    S s; Barrier b(s);std::vector<int> order;
    auto o=opt(P::Measurement,"overdue.TABLE");o.deadline=0ms;
    auto a=s.submit(o,s.token(),[&]{order.push_back(2);return true;});
    auto m=s.submit(opt(P::Motion,"fresh.refill"),s.token(),[&]{order.push_back(1);return true;});
    b.open(); REQUIRE(a.get()&&m.get()); REQUIRE(order==std::vector<int>({2,1}));
 }},
 {"overdue emergency still wins over overdue TABLE", [] {
    S s;Barrier b(s);std::vector<int> order;auto o=opt(P::Measurement,"TABLE");o.deadline=0ms;
    auto a=s.submit(o,s.token(),[&]{order.push_back(2);return true;});
    auto e=s.submit(opt(P::Emergency,"STOP"),s.token(),[&]{order.push_back(0);return true;});
    b.open();REQUIRE(a.get()&&e.get());REQUIRE(order==std::vector<int>({0,2}));
 }},
 {"equal priority keeps FIFO", [] {
    S s;Barrier b(s);std::vector<int> order;std::vector<std::future<bool>> jobs;
    for(int i=0;i<12;++i) jobs.push_back(s.submit(opt(P::Control,"fifo"),s.token(),[&,i]{order.push_back(i);return true;}));
    b.open();for(auto& j:jobs)REQUIRE(j.get());for(int i=0;i<12;++i)REQUIRE(order[i]==i);
 }},
 {"discardable read can expire before execution without side effect", [] {
    S s;Barrier b(s);std::atomic<bool> ran{false};auto o=opt(P::Diagnostic,"old.ui");o.deadline=0ms;o.expireBeforeStart=true;
    auto j=s.submit(o,s.token(),[&]{ran=true;return true;});b.open();REQUIRE(fails(j)&&!ran);
    REQUIRE(s.stats().classes[4].expired==1);
 }},
 {"soft deadline does not silently expire motion", [] {
    S s;auto o=opt(P::Motion,"move");o.deadline=0ms;REQUIRE(s.invoke(o,s.token(),[]{return true;}));REQUIRE(s.stats().classes[1].late==1);
 }},
 {"in-flight deadline does not return early or abandon an SDK write", [] {
    S s;Barrier b(s);REQUIRE(b.task.wait_for(5ms)==std::future_status::timeout);REQUIRE(s.stats().inFlight);b.open();REQUIRE(b.task.get());
 }},
 {"nested transaction executes inline without worker self-deadlock", [] {
    S s;REQUIRE(s.invoke(opt(P::Motion,"BASE+MOVE"),s.token(),[&]{
        return s.isWorkerThread() && s.invoke(opt(P::Control,"nested"),s.token(),[&]{return s.isWorkerThread();});
    }));REQUIRE(s.stats().classes[1].completed==1 && s.stats().classes[3].submitted==0);
 }},
 {"transaction exceptions propagate and worker remains usable", [] {
    S s;auto j=s.submit(opt(P::Control,"throw"),s.token(),[]{throw std::runtime_error("injected");return true;});
    REQUIRE(fails(j));REQUIRE(s.invoke(opt(P::Control,"after"),s.token(),[]{return true;}));REQUIRE(s.stats().classes[3].failed==1);
 }},
 {"false result is observable and counted", [] {
    S s;REQUIRE(!s.invoke(opt(P::Measurement,"failed.read"),s.token(),[]{return false;}));REQUIRE(s.stats().classes[2].failed==1);
 }},
 {"fire-and-report does not wait for blocked SDK", [] {
    S s;Barrier b(s);std::promise<std::string> error;auto f=error.get_future();
    s.post(opt(P::Emergency,"async.stop"),s.token(),[]{return false;},[&](const std::string& e){error.set_value(e);});
    REQUIRE(f.wait_for(5ms)==std::future_status::timeout);b.open();REQUIRE(!f.get().empty());
 }},
 {"callback exceptions do not kill worker", [] {
    S s;s.post(opt(P::Emergency,"bad.callback"),s.token(),[]{return false;},[](const std::string&){throw std::runtime_error("callback");});
    REQUIRE(s.invoke(opt(P::Control,"still.alive"),s.token(),[]{return true;}));
 }},
 {"stats snapshot is responsive during blocked transaction", [] {
    S s;Barrier b(s);for(int i=0;i<50;++i){auto st=s.stats();REQUIRE(st.inFlight&&st.inFlightName=="test.blockedTransaction");}
 }},
 {"shutdown cancels queued requests and waits for in-flight completion", [] {
    S s;Barrier b(s);std::atomic<int> ran{0};auto j=s.submit(opt(P::Control,"queued"),s.token(),[&]{++ran;return true;});
    auto close=std::async(std::launch::async,[&]{s.shutdown();});
    const bool cancelled=j.wait_for(1s)==std::future_status::ready;
    const bool waiting=close.wait_for(5ms)==std::future_status::timeout;
    b.open();close.get();REQUIRE(cancelled&&waiting&&fails(j)&&ran==0&&!s.stats().accepting);
 }},
 {"no new request accepted after shutdown", [] {
    S s;s.shutdown();auto j=s.submit(opt(P::Control,"late"),s.token(),[]{return true;});REQUIRE(fails(j));s.shutdown();
 }},
 {"invalid priority fails before corrupting stats", [] {
    S s;auto o=opt(P::Control,"bad");o.priority=P::Count;bool caught=false;
    try{(void)s.submit(o,s.token(),[]{return true;});}catch(const std::invalid_argument&){caught=true;}REQUIRE(caught);
 }},
 {"in-flight producer keeps admission token after generation changes", [] {
    S s; const auto admitted=s.token();
    REQUIRE(s.invoke(opt(P::Motion,"producer",S::Motion),admitted,[&]{
        s.invalidate(S::Motion);
        return s.transactionToken().motion==admitted.motion && s.token().motion!=admitted.motion;
    }));
 }}
,
 {"post reports admission failure, not a false queued acknowledgement", [] {
    S s(1,1);Barrier b(s);std::atomic<int> errors{0};
    const bool admitted=s.post(opt(P::Emergency,"first"),s.token(),[]{return true;},[&](const auto&){++errors;});
    const bool rejected=s.post(opt(P::Emergency,"full"),s.token(),[]{return true;},[&](const auto&){++errors;});
    b.open();REQUIRE(admitted&&!rejected&&errors==1);
 }}

}); }
