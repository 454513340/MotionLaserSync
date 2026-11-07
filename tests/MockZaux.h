#pragma once
// TEST ONLY: controllable transport, not a controller dynamics simulator.
#include "zauxdll2.h"
#include <array>
#include <atomic>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <chrono>
#include <string>
#include <vector>
struct MockCall { std::string name; std::vector<int> axes; std::vector<float> target; int mode=0; std::thread::id threadId; };
struct MockZaux {
    std::mutex mutex;
    std::atomic<int> inFlight{0}, maxInFlight{0}, callCount{0}, delayUs{0};
    std::array<float,8> dpos{},mpos{},encoder{};
    std::array<int,8> idle{}, status{};
    std::array<uint32,8> homeStatus{};
    std::array<unsigned int,64> op{};
    std::array<float,1600> table{};
    std::vector<MockCall> calls;
    bool holdMotion=false, failAck=false, failAllTableReads=false, suppressPwmConfigAck=false, ignoreHomeMaskWrites=false;
    bool failSp=false, failPause=false, failResume=false, failInterp=false;
    int dataReads=0,failDataRead=0,opens=0,closes=0;
    // Test-only deterministic stall at an SDK boundary. No real hardware is used.
    std::mutex hookMutex;
    std::condition_variable hookWake;
    std::string blockName;
    bool blockEntered=false, releaseBlocked=false;
    int buffered=0;
    bool accountBuffered=false, failHomeRestore=false;
    void armBlock(const char* name);
    bool waitBlocked();
    void releaseBlock();
    void beforeCall(const char* name);
    void reset();
};
extern MockZaux mock;
