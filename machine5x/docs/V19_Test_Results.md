# V19 离线测试结果与限制

## 实际测试环境

Linux x86-64 容器，CMake 3.31.6，GCC 14.2.0，Clang 17.0.0，Python 3.13.5。未连接 ECI3808、禾川驱动或线激光；未加载原厂 Windows DLL。

## 测试组成

| CTest目标 | 内部案例数 | 范围 |
|---|---:|---|
| `p0_portable_unit` | 28 | 保留V18独立计数器、TABLE读取、FIFO/代际及互斥基础测试。 |
| `p0_real_core_mock_transport` | 35 | 原V18的24项加11项V19集成测试；编译真实 `ZmcController.cpp`、`SyncManager.cpp` 和 `EciIoScheduler.cpp`，使用测试Qt替身/模拟ZAux传输。 |
| `v19_scheduler` | 26 | 单线程归属、优先级与不可抢占、STOP原子取消、旧代际失效、防TABLE饥饿、背压、停止预留、截止期、异常、嵌套调用、关闭收尾及投递返回值。 |
| `v19_sync_report` | 19 | 全6468帧合成样本、770无有效点帧、两个有效帧跨度、时钟/索引异常、单位、故障和运行中报告。 |
| `v19_source_contract` | 13项源码检查 | BASIC语句等价、保护文件、实际CMake/GUI接入、SDK名称集合等。需要Python3.9+；不属于C++动态案例。 |

合计 **108个C++案例 + 13项源码检查**。同一组案例在不同编译配置中重复执行，不计算为多套不同案例。

## 执行结果

| 配置 | 结果 |
|---|---|
| GCC / C++17 / Release | 全部5个CTest目标通过。 |
| Clang / C++17 / Release | 全部5个CTest目标通过。 |
| Clang / Debug / AddressSanitizer + UndefinedBehaviorSanitizer | 全部5个CTest目标通过；该轮未报告地址/未定义行为错误。 |
| GCC重复运行 | 核心模拟传输和调度器目标各重复30轮，未出现失败。不是实时性能基准。 |

实际输出在 `docs/test_logs/`。编译时原厂头文件有已有的嵌套注释警告，本次没有为消除警告去修改厂商头文件。

## 关键结果的具体含义

模拟传输记录了每次 SDK 调用的线程ID。OpenEth 使用测试对象的“GUI线程”；连接后的操作和 Close 使用同一个工作线程，并发访问最大值为1。

人为阻塞的是测试替身函数，不是真机网络。测试验证：正在执行的事务未被打断；STOP可以排队，随后优先获得执行；旧的待开始运动被取消；15ms采样握手第二阶段不能在STOP之后重新enable；最多8段的补给事务之后可以服务停止，旧生产者不续发。

报告测试是**合成数据**。包含用户已经给出的总量/索引模式及两个间隔，但没有用户完整原始tOpen文件，不能据此宣称重新测出了用户每一帧的真实触发周期。相邻有效轮廓的20ms/1750ms跨度不再误判为两次丢触发。

## 尚未完成的验证

没有完整编译或运行 Windows/MSVC + 真实 Qt6/VTK 应用；`MainWindow.cpp` 的真实 GUI 集成、MOC、真实Qt事件派发、Windows COM3/SEH分支和厂商DLL兼容性不在这些模拟测试覆盖内。

Qt测试替身不是Qt实现：不模拟隐式共享、事件循环、所有QueuedConnection时序或真实设备动态。ASan/UBSan不是线程竞态证明，也不是硬实时或机械安全认证。

没有测得ECI3808真实吞吐、某个请求的最坏网络时间、真实停稳时间、五轴轨迹连续性或测量精度。软件调度优先级不替代硬件急停。

因此本交付不包含 EXE，必须在用户原Windows工程环境中重新构建并按现场检查表验证。

## 复现命令

```powershell
cmake -S . -B out/v19-tests -DV19_TESTS_ONLY=ON
cmake --build out/v19-tests --config Release
ctest --test-dir out/v19-tests -C Release --output-on-failure
python tools/verify_v19.py
```

Linux上的消毒器构建示例：

```sh
cmake -S . -B out/v19-asan -DV19_TESTS_ONLY=ON -DCMAKE_CXX_COMPILER=clang++ \
  -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_CXX_FLAGS="-fsanitize=address,undefined -fno-omit-frame-pointer" \
  -DCMAKE_EXE_LINKER_FLAGS="-fsanitize=address,undefined"
cmake --build out/v19-asan -j4
ASAN_OPTIONS=detect_leaks=1:halt_on_error=1 UBSAN_OPTIONS=halt_on_error=1 \
  ctest --test-dir out/v19-asan --output-on-failure
```
