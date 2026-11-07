# V17 / P4 五轴关节空间同步定位与轨迹数据结构

## 1. 本阶段边界

V17 在 V16.1 已完成 P1/P2/P3（统一五轴姿态、FK、IK）的基础上实现 P4：

- XYZCB 五轴关节空间点到点同步绝对定位；
- 统一的五轴关节状态、轨迹点、轨迹容器、运动计划和运动结果数据结构；
- 依据五个轴各自 SPEED/ACCEL/DECEL 的速度倍率规划保守的同步段；
- B/C software-frame 机械坐标到 ZMotion 控制器坐标的统一转换；
- 五轴联动前的 absoluteReady、人工使能、空闲、AXISSTATUS、缓冲、软限位复核；
- 使用 ZAux_Direct_Base + ZAux_Direct_MoveAbs 一次发送五轴目标；
- 五轴整体停止，并等待 FASTDEC 减速后的五轴真实停稳再释放运动锁；
- 运动结束后读取 XYZCB 并用 FK 做位置/姿态回读校验。

本阶段**不修改**：

- 现有三轴扫描轨迹；
- 控制器 TABLE(330...) 三轴硬件同步；
- G Code；
- 线激光传感器外参；
- TCP 恒速五轴轨迹。

因此 V17 的“五轴同步定位”是 **joint-space PTP**，不是最终 TCP 直线扫描。

## 2. 新增数据结构

`include/core/DataTypes.h` 新增：

- `JointState5D`：机械坐标 DPOS / MPOS + allIdle；
- `JointTrajectoryPoint5D`：`JointPose5D + timeFromStartSec + scanEnabled`；
- `JointTrajectory5D`：多点五轴关节轨迹容器；
- `JointMotionPlan5D`：单段 PTP 的 baseAxis、数值关节空间长度、V/A/D、预计时间；
- `JointMoveResult5D`：运动结束后的 DPOS/MPOS 和直线/旋转关节误差。

这些类型是后续 P5 同步、P9 TCP 轨迹和五轴 GCode 的共同基础。

## 3. 五轴 PTP 规划

新增 header-only：

`include/trajectory/JointTrajectoryPlanner5Axis.h`

输入：

- 当前机械 `JointPose5D`；
- 目标机械 `JointPose5D`；
- 五轴当前 `AxisParams`；
- `speedScale`（1%~100%）。

规划器不把 mm 与 degree 解释成真实空间长度，而仅建立 ZMotion 多轴插补的**数值关节空间度量**：

`L = sqrt(dX^2+dY^2+dZ^2+dC^2+dB^2)`

对每个活动轴 i，限制：

`vectorSpeed <= vmax_i * L / |dq_i|`

因此对应分速度：

`v_i = vectorSpeed * |dq_i| / L`

不会超过 `AxisParams.speed * speedScale`。

加速度和减速度按同样关系限制。该速度只用于 P4 关节同步定位，**不能称为 TCP mm/s**。

## 4. ZmcController 五轴执行链

新增接口：

- `readJointState5D()`；
- `moveJointAbs5D()`；
- `stopJointMove5D()`；
- `areFiveJointAxesIdle()`。

`moveJointAbs5D()` 执行前：

1. 控制器必须连接；
2. C/B 必须 absoluteReady；
3. C/B 必须人工使能；
4. X/Y/Z/C/B 五轴必须停稳；
5. B/C 静止 DPOS-MPOS 差必须 <= 0.20°；
6. 计划起点与当前 DPOS 不得发生明显变化；
7. 五轴各读一次 AXISSTATUS，有报警则拒绝；
8. 五轴各读一次 MOVES_BUFFERED，必须为 0；
9. 目标机械 C/B 转换为控制器原始 C/B；
10. 再读取控制器实际 FS_LIMIT/RS_LIMIT 复核目标。

运动执行：

- 选择归一化耗时最长的轴作为 BASE 列表第一轴；
- 设置该主轴保存的矢量 SPEED/ACCEL/DECEL/SRAMP；
- 明确 `MERGE=0`；
- `ZAux_Direct_Base(...,5,axisList)`；
- `ZAux_Direct_MoveAbs(...,5,axisList,dest)`；
- 只周期查询插补主轴 IDLE，完成后再一次性复核五轴 IDLE，避免运动期间进行五轴高频轮询。

## 5. B/C software frame

高层规划始终使用千分表标定后的机械角度：

- C 机械零位绝对计数：8,163,418；
- B 机械零位绝对计数：9,200,405；
- V15 已验证绝对方向；
- 控制器正方向与运动学数学右手 C/B 方向相反的约定仍保持。

P4 不写 `MPOS/DPOS/OFFPOS`。

`moveJointAbs5D()` 内部使用现有：

`controller = machine - softwareFrameOffset`

转换 B/C 目标，从而不会重新引入 V13 的累计坐标偏移问题。

## 6. 主界面

“五轴运动学”页新增：

- 同步速度倍率；
- “规划五轴定位”；
- “执行五轴同步定位”；
- “停止五轴”；
- 规划摘要和执行回读结果。

执行前会二次确认，并明确提示“不是 TCP 直线”。

运动期间：

- 五轴单轴运动控件禁用，但各轴 Stop 仍保留；
- 三轴 Merge 启动、扫描启动、GCode 控件禁用；
- 控制器断开操作被拒绝，直到运动线程结束。

## 7. 运动后 FK 校验

运动完成后：

- XYZ 使用最终 DPOS；
- C/B 使用最终机械 MPOS；
- 用 `MachineKinematics5Axis::forward()` 分别计算目标和实际 `^M T_E`；
- 输出关节误差；
- 输出 FK 位置误差和姿态角误差。

当前 UI 对 0.10 mm / 0.10° 以上结果给出“完成但校验超差”警告。这个阈值属于 P4 调试阈值，不是最终机床计量精度指标。

## 8. 第一次实机测试建议

先从低倍率 10%~20% 开始，并保持离机械极限较远：

1. XYZ 改变，B=C=0；
2. 仅 C：0 -> +2 -> 0；
3. 仅 B：0 -> +2 -> 0；
4. C/B 同时：C=+2, B=-2；
5. XYZ + C/B 同时做小位移；
6. 验证“停止五轴”；
7. 验证超软限位目标在运动前被拒绝；
8. 验证 B/C 未人工使能时运动被拒绝。

P4 通过后再进入 P5：控制器 TABLE 从 XYZ 扩展到 XYZCB + TriggerID。
