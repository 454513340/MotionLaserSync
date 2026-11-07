# V20.2 五轴机床 + 机床基传感器单球标定复核：17 参数物理模型 / rank 14

## 1. 结论先行

V20.1 的**名义五轴运动链与单球标定核心的旋转顺序、控制器角度符号是一致的**：二者都对应

\[
{}^M T_E(q)=\operatorname{Trans}(X,Y,Z)R_z(-C)R_y(-B).
\]

V20.1 的主要问题不在 C/B 顺序，而在参数表达：它把 6DoF 手眼中的平移固定为 0，并把这 3 个平移规范自由度吸收到 B/C 轴线位置的“effective offsets”中，形成 14 维 gauge 模型。这种模型可用于拟合，但不适合作为后续运动学规划要长期保存的物理接口，因为完整的手眼矩阵已经丢失。

V20.2 改为：

- **定义并保存完整 17 个物理参数**：11 个机床几何参数 + 6DoF 手眼；
- 固定球半径已知，但**球心在机床坐标系中仍是 3 个未知 nuisance variables**；
- 数据状态为 `[physical17, sphereCenter3]`，共 20 维；
- 单个未知固定球对完整数据状态的结构秩为 **17/20**；
- 消去未知球心后，17 个物理参数的有效秩为 **14/17**；
- 缺失的 3 个方向是严格的**平移规范自由度**，它混合了 B/C 轴线位置、手眼平移和未知球心平移；
- 求解时保留 17 参数结果结构，并把手眼平移作为显式外部/规范锚点。球数据只宣称辨识剩余 14 个独立组合，绝不把手眼平移误报成由球数据辨识得到。

## 2. 坐标系和名义运动学一致性

项目现有 `MachineKinematics5Axis::forward()` 定义：

- 拓扑：`X -> Y -> Z -> C -> B`；
- C 轴名义绕机床 Z；
- C=0 时 B 轴名义绕 Y；
- 控制器角度与数学右手方向相反：`thetaC=-Ccontroller`，`thetaB=-Bcontroller`；
- 名义末端原点在 B/C 交点。

因此：

\[
{}^M T_E=\operatorname{Trans}(X,Y,Z)R_z(-C)R_y(-B).
\]

V20.2 的物理标定模型在所有机床几何误差置零时定义

\[
{}^E T_S=\begin{bmatrix}R_{HE}&t_{HE}\\0&1\end{bmatrix},
\]

所以

\[
{}^M T_S(q)={}^{M}T_E(q){}^E T_S
=\operatorname{Trans}(X,Y,Z)R_z(-C)R_y(-B){}^E T_S.
\]

这与原五轴名义运动学在**坐标方向、C/B 顺序、控制器角度符号**上完全一致。新 C++ 回归测试直接对这两个表达式逐点比较。

## 3. 17 个物理参数的定义

### 3.1 三个直线轴垂直度误差

定义实际直线运动基方向

\[
u_X=[1,0,0]^T,
\]

\[
u_Y=\operatorname{normalize}([\tan S_{xy},1,0]^T),
\]

\[
u_Z=\operatorname{normalize}([\tan S_{xz},\tan S_{yz},1]^T).
\]

XYZ 运动产生的平移为

\[
t_{XYZ}=Xu_X+Yu_Y+Zu_Z.
\]

对应 3 参数：`Sxy, Sxz, Syz`。

### 3.2 C 轴实际轴线

C 名义方向为 `e_z`。用 2 个倾斜参数得到实际方向

\[
d_C=\exp([C_{tx},C_{ty},0]^\wedge)e_z.
\]

由于“沿自身轴线移动轴线上的参考点”不改变这条几何直线，轴线位置只需要法平面内 2 个参数：

\[
q_C=C_u u_C+C_v v_C,
\]

其中 `u_C,v_C` 是与 `d_C` 正交的局部基。

C 轴共 4 参数：`C_offset_u, C_offset_v, C_tilt_x, C_tilt_y`。

### 3.3 B 轴实际轴线

同理，B 名义方向为 `e_y`：

\[
d_B=\exp([B_{tx},0,B_{tz}]^\wedge)e_y,
\]

\[
q_B=B_u u_B+B_v v_B.
\]

B 轴共 4 参数：`B_offset_u, B_offset_v, B_tilt_x, B_tilt_z`。

因此机床几何共 `3 + 4 + 4 = 11` 参数。

### 3.4 完整 6DoF 手眼

传感器到零位末端坐标系：

\[
{}^E p = R_{HE}{}^S p+t_{HE},
\]

其中旋转使用 3 维 rotation vector，平移 3 维，因此 6 参数：

`handeye_rx, handeye_ry, handeye_rz, handeye_tx, handeye_ty, handeye_tz`。

合计 17 参数。

## 4. 带误差的完整前向运动学

对传感器点 `pS=[x,0,z]^T`：

1. 手眼：

\[
p_0=R_{HE}p_S+t_{HE}.
\]

2. 绕实际 B 轴线旋转控制器角 `-B`：

\[
p_1=q_B+R(d_B,-B)(p_0-q_B).
\]

3. 绕实际 C 轴线旋转控制器角 `-C`：

\[
p_2=q_C+R(d_C,-C)(p_1-q_C).
\]

4. 加 XYZ 实际平移：

\[
p_M=p_2+Xu_X+Yu_Y+Zu_Z.
\]

这就是 `SingleSphere17Model::mapSensorPoint()`。

## 5. 单球标定方程：没有“球正负号”

球半径 `R` 已知，球心在机床坐标系中未知：

\[
c_M=[c_x,c_y,c_z]^T.
\]

每一个原始激光轮廓点直接建立球面距离残差：

\[
r_i(\theta,c_M)=\left\|p_M(q_i,p^S_i;\theta)-c_M\right\|-R.
\]

联合优化：

\[
\min_{\theta,c_M}\sum_i \rho\left(r_i^2\right),
\]

其中 `theta` 是 17 个物理参数，`rho` 使用 Huber robust loss。

**该方程没有任何 ± 号。**

### 为什么旧代码出现 `centerYSign`？

如果先把一条二维轮廓 `x,z` 拟合为圆：

\[
(x-c_x)^2+(z-c_z)^2=\rho^2,
\]

再利用已知球半径恢复该时刻三维球心，则只能得到

\[
|c_y|=\sqrt{R^2-\rho^2},
\]

所以**单条二维轮廓恢复 3D 球心**天然有 `+Y/-Y` 两支。

这只是：

- 找球；
- 参考球示教；
- XYZ 闭环重定位；
- 或联合优化的初始化；

需要的临时分支信息，不是球面标定方程的先验。

V20.2 中联合标定在没有用户提供机床球心初值时会自动构造 `+Y`、`-Y` 两个初始化，并用**整套观测的球面残差**自动选较优分支。GUI 中的 Y 侧选择仍保留，但明确标记为“仅找球/示教/闭环，不进入联合标定”。

## 6. 为什么 17 个物理参数只能得到 rank 14

设任意三维向量 `delta`。同时执行：

\[
t_{HE}'=t_{HE}+\delta,
\]

\[
L_B'=L_B+\delta,
\]

\[
L_C'=L_C+\delta,
\]

\[
c_M'=c_M+\delta.
\]

对“绕一条直线旋转”有严格恒等式

\[
\operatorname{RotLine}(q+\delta,d,\alpha)(p+\delta)
=
\operatorname{RotLine}(q,d,\alpha)(p)+\delta.
\]

因此完整运动链输出满足

\[
p_M'=p_M+\delta,
\]

而球心也变为 `c_M+delta`，于是

\[
\|p_M'-c_M'\|=\|p_M-c_M\|.
\]

残差**完全不变**。这不是“小奇异值”或采样不足，而是严格结构零空间。

注意 B/C 轴线位置在代码中用各自法平面的 2 个坐标表达；`delta` 在轴向上的分量只会把轴线上参考点沿同一条轴线移动，因此不改变几何轴线，投影回 2D offset 后仍保持上述等价关系。

所以：

- 完整数据状态 20 维（17 物理 + 3 球心）；
- 严格缺 3 个自由度；
- `rank(J_full)=17/20`；
- Schur 消去球心 3 维后，`rank(J_physical|center eliminated)=14/17`。

这正是“BC 轴平移与手眼平移混合，不能全部独立辨识”的数学形式。

## 7. 求解策略为什么采用“17 参数结构 + 3D gauge anchor”

如果直接让 17 个物理参数和球心 3 个变量无约束迭代，正规方程必然有 3 维零空间，求得的某一组数值只由数值正则化/初值决定，不能解释成真实物理分解。

V20.2 的策略：

1. **参数接口始终是 17 维**，不会再丢失完整手眼矩阵；
2. 对完整 20 状态计算数据 Jacobian，显式验证预期 rank 17/20；
3. 消去球心后对 17 物理列验证预期 rank 14/17；
4. 把 `handeye_tx/ty/tz` 作为 3D gauge anchor 固定在用户输入/外部先验值；
5. 优化剩余 14 个数据支持的物理方向 + 未知球心 3 维；
6. JSON 中仍输出完整 17 参数，并清楚标注手眼平移是 `gauge_anchor`，不是 sphere-data identified。

从约束优化角度看，这等价于“20 维状态 + 3 个规范约束”，而不是退回 14 维物理模型。

### 对后续运动学规划的重要限制

如果后续规划只使用同一 gauge 下的相对几何，17 维模型可以完整传递。

但如果后续规划要求**真实机床绝对坐标中的物理手眼平移**，那么单个未知球心绝对不够。必须再引入至少一个能锁定全局 3D 平移的独立信息，例如：

- 已知球心在机床坐标系中的三维坐标；
- 外部测得的手眼平移；
- 具有已知机床坐标位置的三维基准/工装特征。

只增加多个“位置同样未知”的球不会消除全局 3D 平移 gauge。

## 8. 仿真方法（严格按扫描物理生成，而非直接造圆弧）

脚本：`tools/simulate_single_sphere17_scanlines.py`

真值模型包含：

- 三个 XYZ 垂直度误差；
- C/B 实际轴线偏置与倾斜；
- 给定完整手眼旋转和平移；
- 固定 19.05 mm 标准球，球心在机床坐标系中有真值，但**不提供给标定器**。

每个姿态：

1. 建立完整 `^M T_S`；
2. 在传感器 `XZ` 面 `y=0` 中取一族平行直线
   `pS(lambda)=[x_i,0,lambda]^T`；
3. 把每条直线的原点和方向变换到机床坐标系；
4. 在机床系中显式求直线与球的二次方程交点；
5. 取靠近传感器的一支表面交点；
6. 使用 `(^M T_S)^-1` 把机床交点逆回传感器坐标；
7. 只保留 `x,z`，形成模拟 LLT 二维轮廓；
8. 标定器只拿到 `[XYZCB, sensor_x, sensor_z]` 和球直径，不拿球心真值。

默认仿真：42 个姿态 × 25 条平行扫描线 = 1050 点。

## 9. 仿真和 C++ 复算结果

### Python 几何仿真

- 传感器 → 机床 → 传感器逆变换最大误差：约 `2.84e-14 mm`；
- 施加任意 3D gauge 平移后，球面残差最大变化：约 `4.26e-14 mm`；
- 完整 `[physical17, center3]` 数据秩：`17/20`；
- 消去球心后的物理秩：`14/17`；
- 无噪声优化 RMS：约 `7.12e-14 mm`；
- 加入 `sensor x sigma=0.001 mm`、`z sigma=0.003 mm` 噪声后 RMS：约 `0.00231036 mm`。

### 项目 C++ 求解器对同一 CSV 独立复算

- 无噪声：RMS `1.63532e-10 mm`，物理 rank `14/17`，完整 rank `17/20`；
- 有噪声：RMS `0.00231036 mm`，物理 rank `14/17`，完整 rank `17/20`。

仿真产物位于：

`simulation/v20_2_physical17_rank14/`

包含无噪声/有噪声 profile CSV、Python summary、C++ 输出 JSON。

## 10. 项目代码改动

新增：

- `include/calibration/SingleSphere17Calibration.h`
- `src/calibration/SingleSphere17Calibration.cpp`
- `include/calibration/SingleSphere17QtAdapter.h`（含完整 `handEyeQMatrix()` 与 `sensorToMachineQMatrix()`）
- `tools/single_sphere17_cli.cpp`
- `tools/simulate_single_sphere17_scanlines.py`
- `tests/v20_2_single_sphere17_tests.cpp`
- 本文档和仿真结果目录。

GUI 仍保留历史类名 `SingleSphere14CalibrationWidget` 以避免破坏 MainWindow/现有 source-contract 接线，但内部已经改用 `SingleSphere17*` 核心，并在界面显示“17 参数模型 / rank 14”。新增手眼平移规范锚点 Tx/Ty/Tz 输入。

## 11. 仍然存在的项目级集成缺口：在线点云同步路径

本次复核还发现一个与标定数学无关、但对最终项目非常重要的问题：

`SyncManager.cpp` 当前在线点云坐标变换仍保留旧三轴形式，大意是

- `machineX = X + mountOffset.x`
- `machineY = Y + mountOffset.y + sensorX`
- `machineZ = Z + mountOffset.z + sensorZ`

它没有使用：

- C/B 旋转；
- 17 参数标定后的实际 B/C 轴线；
- 完整手眼矩阵。

此外当前控制器 BASIC 的 TABLE 协议明确仍然只锁存 XYZ，没有 C/B/TriggerID；`SamplePoint` 虽已有 `c,b,hasRotary` 字段，但当前硬件同步数据没有真实的每帧 B/C。

因此：

- **标定、离线变换、找球和后续姿态规划**已经可以使用 V20.2 17 参数模型；
- **动态五轴扫描的在线点云**目前还不能安全地直接切到该模型，因为每个 profile 缺少同步 B/C；
- 下一阶段应先扩展 P5/TABLE 或提供等价的可靠五轴时间同步姿态，再把 `SingleSphere17Model::sensorToMachineTransform()` 接入在线点云构建。

为了不破坏 V19 已归档/实机确认的同步基线，本版本没有偷偷修改这条受保护的硬件协议。

## 12. 回归验证

新增 `v20_2_single_sphere17_tests` 覆盖：

- 名义 `Trans*Rz(-C)*Ry(-B)*T_ES` 一致性；
- 3D translation gauge 严格不变性；
- 物理 rank 14/17；
- 完整状态 rank 17/20；
- 无噪声参数恢复；
- 手眼平移 gauge anchor 不被球数据漂移；
- 新 JSON 的 17 参数 / gauge / full-rank 字段。

现有 `v20_single_sphere14` legacy 测试也保留并通过，便于回溯旧规范模型。

`v20_gui_source_contract` 通过。

`v19_source_contract` 仍有 V20.1 原包已存在的 1 项 manifest/FIX1 打包一致性失败（其余 12 项通过）；我另外从用户提供的**未修改原始 V20.1 ZIP**重新解压并执行同一 `verify_v19.py --structure-only`，得到完全相同的 12/13 结果，因此该失败不是本次 17 参数算法改动引入的。本次也没有修改受保护的 V19 BASIC/同步执行逻辑。

此外新增接口/回归检查保证：

- `SingleSphere17Model::handEyeTransform()` 直接输出完整 `^E T_S`，供后续运动学规划复用；
- `SingleSphere17Model::sensorToMachineTransform()` 与 `mapSensorPoint()` 一致；
- `machinePointInSensor()` 对上述变换做逆映射，回归测试可恢复原传感器点。
