#pragma once
#include <atomic>
#include <cstdint>

namespace p0 {
// Start/Stop are called by the owning Qt thread. The sensor callback only reads.
// Odd tokens denote an active capture, even tokens denote a closed capture.
// A delayed Qt callback keeps its old token and cannot enter a new session.
class ProfileSessionEpoch final {
public:
    using Token = std::uint64_t;
    Token begin() {
        Token token = token_.load();
        if ((token & 1U) == 0) token_.store(++token);
        return token;
    }
    void end() {
        Token token = token_.load();
        if ((token & 1U) != 0) token_.store(token + 1);
    }
    Token current() const { return token_.load(); }
    bool accepts(Token token) const { return (token & 1U) != 0 && current() == token; }
private:
    std::atomic<Token> token_{0};
};
} // namespace p0
