# V19 逐文件变更与边界

## 基线

软件基线为 V18-P0 FIX1；卡端程序基线为用户随后提供的实机确认版 BASIC。二者不能混淆。BASIC 相对旧源码包确有差异，但 V19 不改变用户当前这份程序的执行语句。

## 主要实现

| 文件 | 变更 |
|---|---|
| `include/core/EciIoScheduler.h` / `src/core/EciIoScheduler.cpp` | 新增实际运行的单工作线程调度器：5类优先级、软排队预算、EDF防饥饿、有界队列和停止预留、排队取消、会话/运动/采样代际、入队不可变令牌、异步投递及入队返回值、统计快照、无强杀销毁。 |
| `include/core/ScopeExit.h` | 新增小型 C++17 RAII 收尾工具，保证失败分支释放轴任务/执行必要收尾。 |
| `include/core/ZmcController.h` / `src/core/ZmcController.cpp` | 连接后 SDK 事务统一迁移到调度线程。保留 GUI OpenEth 例外并加入口屏障。阻塞回零/移动/五轴定位/采样握手拆成启动、调用者等待、查询、收尾。新增主界面异步停止接口，返回的是是否入队。旧 XYZ 补缓冲改为16/48水位、最多8段/事务。 |
| `include/core/ProfileTimingAnalysis.h` | 新增逐接收帧记录与纯时间分析：ΔIndex×名义周期、残差、无效时间、24bit索引、已配对无有效点/待配对/丢弃区别。 |
| `include/core/HardwareSyncReport.h` / `src/core/HardwareSyncReport.cpp` | 报告从 GUI 提取为可独立测试的纯函数，读取全部帧的时间元数据，位置估计/曝光时间/PC回调时间分开，附调度统计与真实 BASIC 时序说明。 |
| `include/core/SyncManager.h` / `src/core/SyncManager.cpp` | 保留接收帧元数据和有效/无效点轮廓分类计数；冻结启动配置供报告使用；清除“单核导致丢触发”等未被证明的因果注释。TABLE正常路径和配对几何算法保留。 |
| `include/app/MainWindow.h` / `src/app/MainWindow.cpp` | V19标题、内存调度状态菜单、报告适配；周期位置查询/补缓冲移到后台并各限一个待办；回调按会话代际过滤；关闭等待后台请求；主要停止按钮直接异步入队。 |
| `include/core/DataTypes.h` | 只纠正管理区 isrCount/triggerCount 注释，字段和已验证的硬件常数不改。 |
| `include/core/ControllerIoGate.h` | 只更新注释：在 V19 中是内部状态锁，不是调度器本身。旧测试工具仍保留。 |
| `controller/ZMC_5Axis_Controller.bas` | 换为用户实机确认的执行语句，修正触发顺序、isrCount和溢出注释。 |
| `CMakeLists.txt` / `tests/CMakeLists.txt` | 将新实现链接进正式程序；增加V19测试目标、Threads依赖、测试-only开关；测试目标MSVC UTF-8选项；Python源码校验可选。 |
| `tests/MockZaux.*` / `tests/p0_core_mock_tests.cpp` | 增加线程归属、可控SDK阻塞、缓冲计数和错误注入，测试真实修改后的两份核心CPP，而不连接设备。 |
| `tests/v19_scheduler_tests.cpp` / `tests/v19_report_tests.cpp` | 新增调度边界与全帧报告回归。 |
| `tools/verify_v19.py` | 检查源码归档、保护文件、BASIC语句和实际接入位置，支持完整交付哈希核对。 |

## 特别修复的两个并发边界

STOP 不能先更新代际、释放锁、再排队；否则新的运动可能趁空隙抢先开始。V19 将取消旧排队任务与 STOP 入队作为同一个原子操作。

运动准备事务也不能在收尾时读取“此刻最新的代际”作为自己所属版本。STOP 可能在事务执行中到来；因此保留事务入队时的不可变 token。测试覆盖“XYZ路径准备中收到 C 轴停止，旧XYZ生产者不能重新获得新代际并继续补发”。

## 不变项目

原 SDK 文件、`MachineCalibration.h`、FK/IK、P4轨迹规划器、LaserSensor、PointCloudBuilder、CompactScanFormat、ProfileCounter24、ProfileSessionEpoch 和 TableRingRead 均按 FIX1 原字节保留。受保护文件共26个。GCode执行器/解析器本次没有重写；它们通过现有驱动门面获得调度，不是另造一套轨迹后端。

## 本版本不是以下功能

不是控制卡新增固件、不是控制卡能力探测、不是五轴轨迹密集点流、不是硬实时伺服控制、不是 `.scan` v3、不是 XYZCB+TriggerID 同步、不是真实B/C几何标定，也不是可量化的五轴测量精度验收。

SDK函数名称集合没有超出 FIX1 中已经使用的43种名称；这不替代线程迁移后的实机兼容性验证。
