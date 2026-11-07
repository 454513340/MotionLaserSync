# V18-P0 自动测试结果与覆盖边界

## 实际执行

本记录来自当前交付源码的自动测试运行，没有 ECI3808、伺服、线激光或 Windows 运行环境参与。

| 配置 | 基础逻辑 | 实际核心源码 + 模拟传输 | 结果 |
|---|---:|---:|---|
| GCC 14.2.0 / C++17 | 28/28 | 24/24 | 通过 |
| Clang 17.0.0 / C++17 / Debug | 28/28 | 24/24 | 通过 |
| Clang / AddressSanitizer + UndefinedBehaviorSanitizer | 28/28 | 24/24 | 通过，本轮未报告相关运行错误 |

这是 52 个测试案例在三种配置下重复运行，不是 156 个不同案例。CTest 中有两个测试目标。

模拟测试直接编译交付的 `ZmcController.cpp`、`SyncManager.cpp` 与未修改的 `GlobalTimeSync.cpp`。ZAux 函数签名来自随 V17 打包的原厂 `zauxdll2.h`，未更改 SDK 文件；实现使用 `tests/MockZaux.cpp`，不链接实际 DLL。

测试 Qt 替身在 `tests/shims`，不实现真正 Qt moc、信号投递、事件循环、隐式共享或 GUI。会话 token 的状态机经过单元测试，但真实 Qt 队列时序没有在此环境验证。模拟传输的五轴命令只检查轴列表、参数转换、互斥和停止路由，不模拟 ECI 的运动插补、驱动动力学或实际缓冲消耗。

未执行：完整 Windows/MSVC + Qt6/VTK 应用构建/运行、真实 DLL 线程行为、Windows RS485/SEH 分支、传感器回调时序、卡端定时中断、原点/限位硬件行为、五轴实际速度/加速度/同步精度/急停延迟。主窗口及 GCode 导出提示为源码检查，未作 GUI 自动化测试。

## 源码边界审计

交付控制器源码使用的 ZAux 函数名集合与 V17 相同，均为 43 种，无新增/删除的 SDK 函数种类。此检查不代表每个调用点完全不变；圆弧轴列表、互斥保护、TABLE 提交与故障逻辑正是本版修改内容。

BASIC、全部 third_party 文件、`MachineCalibration.h`、FK/IK、P4 规划器、`DataTypes.h`、真实传感器驱动、`.scan` 序列化代码逐字节保持基线。文件散列与具体变更列表见根目录清单。

## 重复执行

```sh
cmake -S . -B build-p0-tests -DP0_TESTS_ONLY=ON
cmake --build build-p0-tests --config Debug
ctest --test-dir build-p0-tests -C Debug --output-on-failure
python tools/verify_v18_p0.py
```

Linux/Clang 可另外使用 `-fsanitize=address,undefined -fno-omit-frame-pointer` 编译，并添加相应链接选项。所有测试都不需要、也不会连接硬件。

## GCC 案例输出

```text
PASS counter starts at zero
PASS counter starts at arbitrary index
PASS counter preserves forward gap
PASS counter drops duplicate without advancing
PASS counter rejects backward/reset
PASS counter rejects invalid width
PASS counter wraps at 24 bits
PASS counter rejects half-cycle ambiguity
PASS counter supports successive complete cycles
PASS counter resets only explicitly
PASS FIFO retains several not-ready profiles
PASS FIFO waits at front without skipping
PASS session ignores inactive callbacks
PASS session rejects old queued callbacks after restart
PASS session stop is idempotent
PASS valid V17 ring header
PASS invalid ring header rejected before casts
PASS ring rejects out-of-range index
PASS empty ring performs no I/O
PASS contiguous ring commit
PASS wrapped ring reads tail then head
PASS first data read failure publishes nothing
PASS second wrap read failure publishes nothing
PASS ACK ambiguity is distinct from safe read retry
PASS sticky overflow never reads or acknowledges
PASS I/O nested helpers use recursive lock
PASS I/O wait release admits stop worker
PASS I/O lock reacquired after exception
28/28 passed
PASS absolute XY arc uses physical axes 1,2
PASS relative XY arc uses physical axes 1,2
PASS center XY arc uses physical axes 1,2
PASS parallel actual core calls never overlap SDK
PASS OpenEth remains on object GUI thread
PASS actual motion wait releases lock for stop and reads
PASS disconnect/reconnect rejected while worker retains handle
PASS five-axis plan preserves BASE order and mechanical offsets
PASS single rotary stop routes through active five-axis group
PASS active five-axis task rejects legacy writes before SDK
PASS completed merge timer is harmless during five-axis task
PASS stop clears pending host merge to prevent refeeding
PASS actual wrapped TABLE failure does not publish or advance
PASS actual TABLE lost ACK latches and forbids blind retry
PASS actual TABLE sticky overflow publishes nothing
PASS hardware previews cannot contaminate inactive strategy
PASS actual hardware queue preserves pending FIFO order
PASS actual missing profile keeps its empty pose ordinal
PASS actual profile counter wrap aligns monotonically
PASS empty profile still occupies exactly one pose
PASS duplicate profile drops without shifting pose
PASS index reset faults instead of reanchoring pose zero
PASS stats getter never calls controller SDK
PASS real table reader latches overflow once and stops trigger
24/24 passed
```

## 编译器版本

```text
g++ (Debian 14.2.0-19) 14.2.0
clang version 17.0.0 (https://github.com/swiftlang/llvm-project.git 10999b6d034fe318f3d56c83bddb6572593a8bb0)
```
