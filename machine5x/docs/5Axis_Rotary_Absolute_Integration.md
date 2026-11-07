# V16 当前状态（请优先看本节）

V16 以 V15 已验证的“软件机械坐标框架 + 绝对方向修正”为底座，**不再使用 V13 的 SetMpos/SetDpos 坐标重建方案**。后文 V8~V13 内容均为历史演进记录。

最新千分表机械零位已经集中到 `include/core/MachineCalibration.h`：

```cpp
constexpr int64_t kZeroCountsC = 8163418;
constexpr int64_t kZeroCountsB = 9200405;
constexpr int kAbsoluteCountToControllerSignC = -1;
constexpr int kAbsoluteCountToControllerSignB = -1;
```

标定来源：

```text
C: P21.32=0, P21.34≈8,163,418 -> total=8,163,418
B: P21.32=1, P21.34≈811,797  -> total=9,200,405
```

重新校准后只改 `MachineCalibration.h` 的两个 `kZeroCounts`。

V16 同时新增 P1/P2/P3 五轴运动学基础，详见 `docs/5Axis_Kinematics_P1_P3_V16.md`。

---

# V13 二次连接修正：取消 OFFPOS，分别建立 MPOS / DPOS

现场 V12 二次连接时出现：

```text
target=12.398355
DPOS=12.396600
MPOS=0.001800
```

这组数据说明 V12 的 `SetOffpos(delta)` 在当前 ATYPE=4 实机上并没有把真实编码器反馈坐标 MPOS 一起建立到绝对坐标；它主要把规划坐标 DPOS 推到了目标附近。此前“OFFPOS 会把 DPOS/MPOS 一起重基准”的假设与现场结果不符，因此 V13 完全撤销该方案。

V13 初始化原则：

1. OP24 / OP26 先强制 OFF，B/C 必须停稳；
2. 读取 P21.32 + P21.34，减去已标定 `ZERO_COUNTS` 得到机械绝对角；
3. 不再要求初始化前 DPOS≈MPOS。旧坐标不一致本来就是绝对初始化需要修复的状态；
4. `ZAux_Direct_SetMpos(axis, target)`，短延时后立即回读 MPOS；
5. `ZAux_Direct_SetDpos(axis, target)`，短延时后回读 DPOS/MPOS；
6. 如果某一个坐标仍偏离目标，只对错误的坐标做最多两轮有限纠正；
7. 禁止使用累计 OFFPOS，也禁止无限修正循环；
8. 最后做 3 次、40 ms 间隔的短期稳定性复核。DPOS/MPOS 都稳定在目标 ±0.05° 内才置 absolute-ready；
9. 这些读取只发生在连接/手动重同步事件中，不增加扫描运行时的长期控制器轮询负担。

V13 保留机械零点标定：

```text
C ZERO_COUNTS = 7,498,994，减速比 10:1
B ZERO_COUNTS = 1,510,987，减速比 100:1
```

因此 RS485 机械角仍为：

```text
totalCounts = P21.32 * 8388608 + P21.34
C_deg = (totalCounts - 7498994) * 360 / 8388608 / 10
B_deg = (totalCounts - 1510987) * 360 / 8388608 / 100
```

> V11/V12 的 OFFPOS 重基准方案已被 V13 废弃。下面历史章节仅用于说明版本演进，不应再作为当前实现依据。

---

# V12 机械零点标定：写入 B/C ZERO_COUNTS

现场已在 B/C 机械 0°位置读取 P21.32 + P21.34，并完成零点计数标定。

- C：P21.32=0，P21.34=7,498,994，故 `kZeroCountsC=7498994`。
- B：P21.32=0，P21.34 连续读取 1,510,983 / 1,510,991 / 1,510,987 / 1,510,987；平均值恰为 1,510,987，故 `kZeroCountsB=1510987`。

23bit 编码器单圈计数为 8,388,608。机械角度统一按：

```text
totalCounts = P21.32 * 8388608 + P21.34
C_deg = (totalCounts - 7498994) * 360 / 8388608 / 10
B_deg = (totalCounts - 1510987) * 360 / 8388608 / 100
```

不使用 P21.07 参与零点换算。P21.32/P21.34 仍是唯一的绝对位置源。

在机械零点附近，C 应得到约 0°；B 的 4 count 抖动折算到 100:1 减速后的机械角仅约 0.0000017°，可忽略。

V12 当时保留了 V11 的 OFFPOS 重基准逻辑；该结论已被后续 V13 现场数据推翻，当前版本不再使用 OFFPOS。

---

# V11 重要修正：旋转轴绝对坐标禁止连续 SetMpos + SetDpos

现场发现：C 轴绝对读取约 32°，回到控制器 0° 后断开再连接，MPOS 变成约 64°。
该现象与 V8-V10 中连续调用 `SetMpos(target)`、`SetDpos(target)` 的坐标偏移叠加一致。

V11 已改为单次坐标重基准：

```text
读取当前 DPOS / MPOS
要求轴停止且 DPOS≈MPOS
delta = 禾川机械绝对角度 - 当前 MPOS
ZAux_Direct_SetOffpos(axis, delta)   // 只调用一次
回读 DPOS / MPOS，二者都必须等于目标
```

不再在 B/C 绝对位置初始化中调用 `SetMpos` 或 `SetDpos`。重复连接时若物理位置未变，`delta≈0`，因此不会再次叠加。

另外，`P20.06=7` 并不能保证 `P21.34` 单圈位置清零。V12 已写入现场机械零点偏置：`kZeroCountsC=7498994`、`kZeroCountsB=1510987`。本段关于 V11 OFFPOS 的处理仅为历史记录，V13 已不再采用。

---

# 五轴项目 V8：B/C 单轴运动、DPOS/MPOS 与连接卡顿修正版

## 本版范围

本版只处理 B/C 两个旋转轴的：

- 禾川 RS485 绝对位置初始化；
- DPOS / MPOS 绝对坐标同步；
- 人工使能 OP24 / OP26；
- 单轴点动；
- 单轴绝对定位；
- 单轴相对定位；
- ±90° 软限位检查；
- 控制器未连接时的界面阻塞问题。

**本版没有增加或修改 X/Y/Z/C/B 五轴联动插补。** 原 XYZ 扫描轨迹、GCode 和 TABLE(330...) 采样逻辑保持原结构。

## 1. 控制器连接：不再在不可达 IP 上直接阻塞 ZAux_OpenEth

V7 的手动“连接”按钮会直接在 GUI 线程调用 `ZAux_OpenEth()`。现场已经确认，当 PC 网卡/IP 未配置或控制器不存在时，该 DLL 调用会长时间同步阻塞，因此整个 Qt 界面卡住。

V8 改为：

1. 操作员点击“连接”；
2. Qt 使用 `QProcess` 异步执行一次系统 `ping`；
3. IP 不响应：直接提示失败，**不调用 `ZAux_OpenEth`**，界面不阻塞；
4. IP 响应：才在原 GUI/控制器线程调用一次 `ZAux_OpenEth`；
5. 不自动重试。

这样避免了 V6 将厂商 DLL 放进 QtConcurrent 后出现的进程级闪退，同时避免 V7 在控制器不存在时卡死 GUI。

> 如果现场控制器网络明确能用 ZAux 连接但禁止 ICMP/ping，则本预检查会产生假阴性。出现这种情况时应再改为独立辅助进程探测，而不要把 `ZAux_OpenEth` 放回 QtConcurrent 线程。

## 2. B/C 绝对位置：现在显式同步 DPOS 与 MPOS

现场 V7 已确认：调用 `ZAux_Direct_SetMpos()` 后 MPOS 更新，但 DPOS 仍保持旧值。

因此 V8 初始化流程是：

1. 强制 `OP24=OFF`、`OP26=OFF`；
2. COM3，9600，8N2；
3. C 从站 2 读取 P21.32 + P21.34；
4. B 从站 1 读取 P21.32 + P21.34；
5. 23 bit：8,388,608 count/rev；
6. C 减速比 10:1，B 减速比 100:1；
7. 检查计算得到的机械绝对角度是否位于控制器 FS_LIMIT / RS_LIMIT 内；
8. 对 C(4)、B(5) 分别写 `MPOS=绝对角度`；
9. 再写 `DPOS=绝对角度`；
10. 同时回读 DPOS / MPOS；
11. 如 SetDpos 导致 MPOS 偏移，再补写一次 MPOS 并回读；
12. DPOS、MPOS 与目标误差均 <= 0.02° 才设置 absoluteReady=true；
13. OP24 / OP26 仍保持 OFF，必须人工使能。

因此本版不再假定“SetMpos 一定会自动让 DPOS 一致”，而是以现场控制器的实际回读值为准。

## 3. B/C 单轴绝对定位、相对定位

V8 明确不做五轴联动。

B/C 绝对定位改为单轴接口：

```cpp
ZAux_Direct_Single_MoveAbs(handle, axis, target);
```

相对定位继续使用：

```cpp
ZAux_Direct_Single_Move(handle, axis, delta);
```

在发送 B/C 绝对/相对运动前，上位机先读取 FS_LIMIT / RS_LIMIT，并按当前 DPOS 计算目标位置。如果目标超过 ±90°（或控制器当前实际限位值），直接拒绝并显示明确错误，不向控制器发送运动。

## 4. B/C 点动方向修正

V7 的连续点动错误地使用：

```cpp
positive ? 1 : 0
```

而项目所带 `zauxdll2.h` 明确规定 `idir=1` 为正向、`idir=-1` 为负向。

V8 已改为：

```cpp
ZAux_Direct_Single_Vmove(handle, axis, positive ? 1 : -1);
```

## 5. 人工使能增加 OP 回读

B=OP24，C=OP26。

点击人工使能后，V8 不再只相信 `SetOp()` 的返回码，而会立即 `GetOp()` 回读实际输出状态。只有回读和请求一致时，软件内部 `servoEnabled=true`，随后点动/绝对定位/相对定位按钮才允许运动。

日志会输出例如：

```text
B servo enable output OP(24)=1 readback OK
C servo enable output OP(26)=1 readback OK
```

## 6. 界面同时显示 DPOS / MPOS

B/C 行现在显示：

```text
DPOS：12.345   MPOS：12.346 (停止)
```

便于现场直接观察：

- DPOS：控制器规划/指令坐标；
- MPOS：编码器反馈坐标。

绝对位置初始化后，这两个值应该基本一致；运动时允许存在动态跟随误差，但方向必须一致。

## 7. 控制器 Basic 没有新增握手

控制器程序仍使用现场当前版本：

- Axis4: `ATYPE=4`, `UNITS=1112`, `ENCODER_RATIO(1,-1)`, FS/RS=±90°；
- Axis5: `ATYPE=4`, `UNITS=5560`, `ENCODER_RATIO(1,-1)`, FS/RS=±90°；
- TABLE 采样仍从 330 开始；
- 没有 TABLE(320~322)；
- 没有新增 B/C 绝对位置握手逻辑。

## 首次 V8 实机测试建议

1. 控制器断开状态启动软件，确认界面可以自由切换、退出；
2. 点击“连接”，控制器 IP 不可达时应在约 1.5 秒内失败，且 GUI 不冻结；
3. 正确配置网络后连接；
4. 查看 B/C 是否显示 DPOS 和 MPOS 两个值，初始化后应基本相同；
5. 查看人工使能日志是否出现 OP24/OP26 回读 OK；
6. 先测试 C +1° / -1° 点动，再测试 B +1° / -1°；
7. 测试相对定位 +1°；
8. 测试绝对定位到一个当前附近且位于 ±90° 内的角度；
9. 暂时不要做任何五轴联动测试。

---

# V9 增补：AXISSTATUS 报警监视与 B/C 回零到 0°

## 8. 五轴 AXISSTATUS 周期监视

V8 只读取位置和 IDLE，没有周期读取 `ZAux_Direct_GetAxisStatus()`，因此控制器已有的硬限位、软限位、伺服报警等状态不会被主动解码写入日志。

V9 增加：

- `ZAux_OpenEth` 成功后立即检查 X/Y/Z/C/B 五轴 AXISSTATUS；
- 之后约每 500 ms 检查一次；
- 状态不变时不重复写日志，避免刷屏；
- 状态变化、产生报警、报警清除时才记录；
- 每个单轴面板增加“轴状态”显示。

重点状态包括：

- 0x10 正向硬限位；
- 0x20 反向硬限位；
- 0x100 随动误差超限出错；
- 0x200 正向软限位；
- 0x400 负向软限位；
- 0x1000 脉冲频率超过 MAX_SPEED；
- 0x400000 伺服/告警输入；
- 以及控制器手册定义的其它 AXISSTATUS 位。

典型日志：

```text
[WARNING] [Controller] B轴 AXISSTATUS=0x200：超过正向软限位
[INFO   ] [Controller] B轴报警/状态已清除，AXISSTATUS=0
```

## 9. B/C 单轴回零

B/C 没有原点开关，因此 V9 不对 B/C 调用 DATUM。

当绝对编码器已经完成 DPOS/MPOS 同步、并且操作员已经人工使能后：

```text
B/C “回零(0°)” = 单轴绝对定位到 0°
```

内部直接复用已经验证过的单轴绝对定位流程：

```cpp
moveAbs(axis, 0.0f, ...)
```

所以仍然受以下保护：

- absoluteReady；
- 人工使能；
- FS_LIMIT / RS_LIMIT；
- 单轴忙状态；
- 控制器 AXISSTATUS。

## 10. 五轴全部回零

全局按钮改为“五轴全部回零”。

- X/Y/Z：保持原来的 DATUM 回零；
- C/B：绝对定位到 0°；
- 如果 B/C 任意一个没有完成绝对位置同步或没有人工使能，则整次“五轴全部回零”不启动，避免实际上只回 XYZ 却让操作员误认为五轴都已回零。

本版仍然没有增加五轴联动插补。

---

# V10 增补：扫描期间停止周期状态/位置轮询

## 11. AXISSTATUS 改为事件式读取

V9 的 500 ms 五轴 AXISSTATUS 周期轮询在在线激光同步场景中没有必要，会增加 PC 与控制器之间的额外通信。
V10 删除该周期轮询，改为以下事件点读取：

- 控制器连接成功：五轴各读取一次；
- B/C 绝对坐标同步完成：只读取 C/B 各一次；
- 单轴绝对定位、相对定位、回零结束：只读取本轴一次；
- 点动命令失败：只读取本轴一次；
- 点动/运动停止后：延时约 150 ms，只读取本轴一次；
- B/C 人工使能/断使能后：只读取本轴一次；
- 激光同步采集停止后：五轴各读取一次。

在线激光同步采集期间，`pollAxisAlarmState()` 直接返回，完全不读取 AXISSTATUS。
轴状态面板显示的是“最近一次事件检查结果”，不是实时 500 ms 刷新的状态。

## 12. 扫描期间暂停仅用于 UI 的位置轮询

V10 在 `SyncManager::isRunning()` 为 true 时停止 `onPosTimer()` 对 DPOS/MPOS/IDLE 的界面轮询。
硬件同步所需的 TABLE 位置读取和触发链路不受影响，并优先使用控制器通信。
采集停止后，位置面板自动恢复刷新。
