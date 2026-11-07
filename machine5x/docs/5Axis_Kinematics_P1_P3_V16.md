# V16 五轴运动学基础（P1 / P2 / P3）

## 1. 本版本范围

V16 在 V15 已稳定的 B/C 单轴控制基础上，只完成五轴系统的基础层：

- P1：统一五轴数据结构；
- P2：五轴机床本体正运动学 FK；
- P3：五轴机床本体解析逆运动学 IK；
- 所有运动学功能直接集成在原 V15 主项目中，不包含独立 `tests/` 测试工程或第二个 `main()`；
- 增加一个“**五轴运动学**”UI 页用于人工验证 FK 和 FK→IK 回环；
- 将 B/C 绝对编码器零点集中到 `include/core/MachineCalibration.h`，以后重新校准无需再进入 `ZmcController.cpp` 找常量。

本版本**没有**实现五轴联动轨迹、五轴 GCode、五轴 TABLE 同步、传感器外参或五轴点云变换。这些属于 P4 之后。

## 2. 已确认的机床几何约定

机械拓扑：

```text
X -> Y -> Z -> C -> B
```

- X/Y/Z 控制器正方向 = 机床数学 +X/+Y/+Z；
- C 轴绕机床 Z 轴；
- C=0 时 B 轴绕机床 Y 轴；
- B/C 旋转轴线相交；
- 当前末端参考坐标系 `{E}` 原点就定义在 B/C 轴线交点；
- 暂不包含线激光传感器安装矩阵；
- 从 +Z 看向原点时，数学右手 C+ 对应控制器 C-；
- 从 +Y 看向原点时，数学右手 B+ 对应控制器 B-。

因此：

```text
thetaC_math = -C_controller
thetaB_math = -B_controller
```

## 3. 最新机械零位标定

文件：`include/core/MachineCalibration.h`

当前千分表校准后的零位：

```cpp
constexpr int64_t kZeroCountsC = 8163418;
constexpr int64_t kZeroCountsB = 9200405;
```

其中：

```text
C: P21.32=0, P21.34≈8,163,418
B: P21.32=1, P21.34≈811,797
   1*8,388,608 + 811,797 = 9,200,405
```

以后重新标定，只需要修改这两个值：

```text
ZERO_COUNTS = P21.32 * 8,388,608 + P21.34
```

不要修改减速比和方向，除非机械/控制器配置真的发生变化。

## 4. P1：数据结构是什么

以前项目很多接口只有散乱的 `x/y/z` 或 `PathPoint{x,y,z,feed}`。五轴以后如果继续到处增加独立的 `b/c` 参数，运动学、轨迹、同步、GCode 很容易出现轴顺序和单位错误。

V16 新增统一类型：

```cpp
struct JointPose5D {
    double x, y, z; // mm
    double c, b;    // deg，控制器/上位机机械坐标方向
};
```

这是后续所有五轴模块共享的“关节状态”。轴顺序固定为：

```text
[X, Y, Z, C, B]
```

另外新增：

```cpp
struct EndPose5D {
    JointPose5D joints;
    QVector3D position;
    QMatrix4x4 transform; // ^M T_E
    double mathCdeg;
    double mathBdeg;
};
```

和：

```cpp
struct InverseKinematics5DResult {
    bool success;
    JointPose5D joints;
    double rotationResidual;
    double positionResidual;
    QString error;
};
```

`PathPoint`、`SamplePoint` 和算法层的 `ScanPathPoint` 也预留了 C/B 字段/有效标志，但 V16 **尚未把旧三轴联动和 TABLE 采样改为五轴**，因此这些字段只是 P4/P5 的数据结构准备。

## 5. P2：正运动学

核心文件：

```text
include/kinematics/MachineKinematics5Axis.h
```

当前机床本体模型：

```text
^M T_E = Trans(X,Y,Z) * Rz(-C) * Ry(-B)
```

令：

```text
c = -C
b = -B
```

则：

```text
R = Rz(c) Ry(b)

    [ cos(c)cos(b)  -sin(c)  cos(c)sin(b) ]
    [ sin(c)cos(b)   cos(c)  sin(c)sin(b) ]
    [    -sin(b)        0        cos(b)    ]
```

完整矩阵：

```text
[ R00 R01 R02 X ]
[ R10 R11 R12 Y ]
[ R20 R21 R22 Z ]
[  0   0   0  1 ]
```

由于末端原点暂时就是 B/C 交点，所以 B/C 旋转只改变姿态，不改变该点的位置。这是当前模型简单的原因。

## 6. P3：解析逆运动学

对于满足本机床可达姿态形式的目标旋转矩阵 `R`：

```text
thetaC = atan2(-R01, R11)
thetaB = atan2(-R20, R22)
```

由于控制器角度与数学角方向相反：

```text
C_controller = -thetaC
B_controller = -thetaB
```

位置直接为：

```text
X = Tx
Y = Ty
Z = Tz
```

IK 不会盲目接受任意 6DOF 旋转矩阵。它会：

1. 检查矩阵是否有限；
2. 检查旋转矩阵正交性和 determinant≈1；
3. 求 B/C；
4. 检查 B/C 是否在当前 ±90° 限位内；
5. 用求得的关节值重新 FK；
6. 如果重建旋转残差过大，则明确返回“该姿态不属于当前五轴 Rz(C)Ry(B) 可达姿态”。

这很重要，因为本机床只有 5DOF，不能表示任意 6DOF 刚体姿态。

## 7. 与 ZmcController 的连接

新增：

```cpp
bool ZmcController::readJointPose5D(JointPose5D& pose, QString* err);
```

规则：

- XYZ：读取 DPOS；
- C/B：读取 absolute-ready 后的软件机械 MPOS，因为 Axis4/5 是 ATYPE=4，有真实编码器反馈；
- 该函数只用于运动学、标定、人工检查等**低频**场景；
- 明确禁止把它用于在线激光扫描高频轮询。

## 8. UI 验证

右侧增加：

```text
五轴运动学
```

页面。

功能：

- 输入 X/Y/Z/C/B；
- “读取当前五轴”：从控制器读取当前五轴机械姿态；
- “正解 FK”：显示 `^M T_E` 4×4 矩阵；
- “FK→IK 自检”：先 FK，再解析 IK，显示恢复的 X/Y/Z/C/B 和残差。

这个页面**不发送任何五轴联动运动指令**，只是数学模型验证工具。

建议现场验证点：

```text
(0,0,0,0,0)
(10,20,30,0,0)
(10,20,30,10,0)
(10,20,30,0,10)
(10,20,30,30,-20)
(10,20,30,-45,45)
```

所有 FK→IK 自检应返回输入值，残差应接近浮点数精度。


## 8.1 VS2022 工程说明

本版本不再提供独立测试源文件。只需要像原 V15 一样编译/运行主上位机工程。新增的运动学核心是 `include/kinematics/MachineKinematics5Axis.h`，由主程序直接包含使用；不需要建立第二个 Console 工程，也不会出现第二个 `main()`。

## 9. 现在不要做的事情

V16 没有把以下部分贸然改成五轴：

- `moveLineAbs()` 仍是 XYZ 三轴插补；
- `prepareMergePath()` / `pumpMergeBuffer()` 仍是 XYZ 三轴；
- 控制器 TABLE 仍是 XYZ 三维；
- `SamplePoint.c/b` 目前硬件采样不会填入；
- GCode Parser/Executor 仍是 XYZ；
- 点云仍未使用 `FK(q) * T_end_sensor`。

这是刻意的。先让 P1/P2/P3 独立稳定，再做 P4 五轴轨迹和 P5 五轴同步，避免一次改动过大破坏 V15 已稳定的底层控制。

## 10. 下一阶段建议

P1/P2/P3 实机确认后，下一步是 P4：

1. 新增 `JointPathPoint5D` / 时间参数；
2. 增加五轴关节空间同步定位 API；
3. 增加基于 FK/IK 的任务空间离散；
4. 处理 B/C 限位和解连续性；
5. 再扩控制器 TABLE 为 XYZCB，用于激光硬件同步。
