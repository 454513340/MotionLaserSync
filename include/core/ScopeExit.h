#pragma once
#include <utility>
namespace eci {
template<class F> class ScopeExit final {
public:
    explicit ScopeExit(F f) : fn_(std::move(f)) {}
    ~ScopeExit() noexcept { fn_(); }
    ScopeExit(const ScopeExit&) = delete;
    ScopeExit& operator=(const ScopeExit&) = delete;
private:
    F fn_;
};
template<class F> ScopeExit<F> scopeExit(F f) { return ScopeExit<F>(std::move(f)); }
}
