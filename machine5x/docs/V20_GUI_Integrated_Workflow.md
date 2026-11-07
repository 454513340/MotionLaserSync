# V20 单球 14 维标定 GUI 集成说明

本版本从原始 `5Axis_project_v19` 直接集成 V20 单球 14 维标定核心。
目标硬件条件：

- 五轴结构：XYZ-C-B；
- 线激光：scanCONTROL LLT2950-50/BL；
- 标准球直径：19.05 mm；
- 标定球固定在工作空间；
- 线激光安装在机床末端；
- 没有独立三维平移基准，因此求解 14 个数据可观测的 gauge 参数，而不是唯一物理 17 参数。

## 1. 与 V19 原接口的连接

V20 新页面位于：

`系统标定 -> 单球五轴14维标定`

它不另写控制器或传感器驱动，复用 V19：

- `MainWindow::onStartProfilePreview()` / `onStopProfilePreview()`；
- `LaserSensor::profileReceived()`；
- `ZmcController::readJointState5D()`；
- `JointTrajectoryPlanner5Axis::planPointToPoint()`；
- `ZmcController::moveJointAbs5D()`；
- `ZmcController::stopJointMove5D()`。

五轴移动仍经过 V19 的 B/C absolute-ready、人工使能、软限位和五轴空闲检查。

## 2. 实际使用流程

### 2.1 上电

1. 连接控制器；
2. 连接 2950；
3. B/C 绝对位置同步；
4. 人工使能 B/C；
5. 执行 V19 的“五轴全部回零”；
6. 确认 X/Y/Z/C/B 都 Idle，且没有限位/伺服报警。

标定会话期间不要再点击“XYZ 清零”，否则数据不在同一坐标框架。

### 2.2 进入单球页面并启动轮廓

点击“启动轮廓预览”。请求会转发给原 MainWindow 的软件内部触发轮廓预览逻辑。

页面内的二维轮廓控件直接监听 `LaserSensor::profileReceived`，所以与原传感器调试页看到的是同一条实时轮廓流。

### 2.3 手动把球移入视场

使用 V19 原单轴 XYZ/C/B 控件，或五轴运动学页面，手动将标准球移入 2950 视场。

建议初始位置：

- Z_sensor 约 95 mm；
- 球面截线位于 X 中央区域；
- 激光平面不要恰好切球赤道；
- 页面默认 ROI：X=[-15,15] mm，Z=[80,110] mm。

### 2.4 拟合球截面

点击“拟合当前轮廓”。

软件只使用 ROI 内有效点，拟合 X-Z 平面截面圆，并利用已知球半径 9.525 mm 恢复球心 Y：

`|Yc| = sqrt(R^2-rho^2)`。

Y 正负由“球心 Y 分支”选择。页面显示：

- 球心 S；
- 截面半径；
- 圆拟合 RMS；
- ROI 点和拟合圆覆盖层。

### 2.5 参考球示教

在一个质量合格的轮廓上点击“设为参考球/零位示教”。

软件：

- 读取真实 `JointState5D`；
- 要求所有五轴 Idle；
- 使用 MPOS 作为静态标定姿态；
- 保存当前球心和 XYZCB；
- 建立姿态规划初值。

### 2.6 生成姿态

设置 B/C 范围和记录姿态数，点击“生成 D-optimal 姿态”。

规划器：

- 从当前参考球和当前参数估计生成候选姿态；
- 按 Schur 消元后的 14 维信息量选姿态；
- 从控制器读取 XYZ/B/C 软限位后过滤；
- 给出每个姿态的预测 XYZCB 和目标传感器球心。

注意：预测 XYZ 是找球初值，不包含真实夹具碰撞模型。

### 2.7 移动到姿态

选择姿态行，点击“移动到选中姿态”。

请求被转发到 MainWindow，并使用 V19：

`JointTrajectoryPlanner5Axis -> ZmcController::moveJointAbs5D`。

每次移动必须人工确认。若 B/C 变化超过 5°，软件明确警告当前没有碰撞和线缆模型；空间不确定时应通过原手动控制界面分段靠近。

### 2.8 XYZ 找球

姿态改变后若球偏离目标区域：

1. 保持轮廓预览；
2. 点击“拟合当前轮廓”；
3. 点击“根据当前球心建议 XYZ 找球”；
4. 软件计算一次限幅 XYZ Newton 步；
5. 人工确认后复用五轴移动接口执行；
6. 重复直到球位于目标区域。

B/C 在该步骤保持不变。

### 2.9 静态采集

点击“采集当前姿态”。

软件不会拿 `m_lastProfile` 直接绑定一个随后读取的位置，而是：

1. `readJointState5D()` 并检查五轴 Idle；
2. 等待配置的停稳时间；
3. 只收集等待结束后到达的 N 条 fresh profiles；
4. 再次 `readJointState5D()`；
5. 检查采集前后 MPOS 漂移；
6. 对每条 fresh profile 做 ROI 和球截面拟合；
7. 至少一半 profile 通过质量阈值才接受；
8. 每条通过的轮廓与该静态姿态绑定并加入标定器。

### 2.10 可观测性

每接受一组采样后，GUI 计算 Schur 消元后的 14 维可观测性：

- rank；
- 最小奇异值；
- condition number。

只有 rank=14 时“开始 14 维联合标定”按钮才启用。

### 2.11 求解

求解在 `QtConcurrent` 数值线程中执行，不访问硬件。

GUI 额外验收：

- rank=14；
- residual RMS <= 0.05 mm；
- condition number <= 1e9。

结果显示 14 个 gauge 参数、球心、RMS、最大残差和奇异值。

### 2.12 保存/恢复

支持：

- 保存结果 JSON；
- 加载结果 JSON；
- 导出已用于标定的 profile-point CSV；
- 导入上述 CSV 重新离线求解。

## 3. 重要限制

1. **14 维不是唯一物理 17 维。** 四个 rotary effective offsets 吸收了不可分开的 hand-eye translation gauge。
2. V19 在线 TABLE 同步链仍是 XYZ 三轴，因此当前 14 维结果尚未自动用于动态五轴扫描点云；本次集成完成的是静态单球标定工作流。
3. 姿态规划器没有夹具碰撞、球座碰撞、线缆扭转和 CAD 机头模型；五轴大姿态移动必须人工确认。
4. 球面分割目前使用 X/Z 数值 ROI，不是图形鼠标多边形 ROI；真实数据应避免球座和背景进入 ROI。
5. 单截面恢复球心 Y 有正负分支，必须在整个会话中保持一致。
6. 当前 Linux 验证环境没有 Qt6/VTK/ZMotion/LLT Windows SDK，因此 GUI target 未在该环境完成链接；portable calibration core 和 V19 source-contract 已验证。

## 4. 新增/修改文件

新增：

- `include/widgets/SingleSphere14CalibrationWidget.h`
- `src/widgets/SingleSphere14CalibrationWidget.cpp`
- `include/calibration/SingleSphere14Calibration.h`
- `include/calibration/SingleSphere14QtAdapter.h`
- `src/calibration/SingleSphere14Calibration.cpp`
- `tools/single_sphere14_cli.cpp`
- `tools/verify_v20_single_sphere_gui_integration.py`

修改：

- `include/widgets/CalibrationWidget.h`
- `src/widgets/CalibrationWidget.cpp`
- `include/widgets/Profile2DWidget.h`
- `src/widgets/Profile2DWidget.cpp`
- `src/app/MainWindow.cpp`
- `CMakeLists.txt`

## V20.1 实机采集修订：RANSAC 球弧 + 软件触发全量程

### 轮廓提取

单球标定中的 X/Z ROI 现在仅作为**软件候选搜索窗口**，不会修改 scanCONTROL 的硬件测量区域。ROI 内的有效点先经过 RANSAC 圆弧提取：

1. 剔除 SDK 无效点、NaN/Inf 以及 ROI 外点；
2. 三点圆模型 RANSAC；
3. 用已知球半径 9.525 mm 排除不可能的截面圆；
4. 按径向残差阈值提取内点；
5. 仅使用内点做代数圆初值 + 几何距离迭代精修；
6. 检查内点数量、内点比例、圆弧覆盖角、截面半径和内点 RMS；
7. **只有 RANSAC 内点进入 `SphereProfileObservation` 和后续 14 维标定器**。

GUI 默认参数：RANSAC 350 次、内点阈值 0.050 mm、最小内点比例 0.20、最小圆弧覆盖 30°。这些值应根据真实球面反射和曝光情况调整；ROI 应尽量包含完整球弧，但无需精确贴合球轮廓。

### 2950 采集模式

单球静态标定固定采用：

- `SensorTriggerMode::SoftwareInternal`（内部/软件触发连续 Profile）；
- 全量程 `Large / ROI1_PRESET = 0`；
- 全量程设置只在运行时生效，调用 `setMeasuringFieldIndex(0, false, ...)`，不会写入用户模式；
- 如果之前使用高频小测量区域导致曝光+idle 对全量程过快，软件会依据该 full-field preset 的最大频率自动增加 idle 时间后再启动预览。

因此，硬件同步扫描阶段曾使用的小测量区域不会被单球标定继承。标定窗口中的 X/Z ROI 仅影响 RANSAC 搜索，不影响传感器物理量程，也不再影响 D-optimal 姿态规划器采用的 80--110 mm / ±18 mm 中央安全目标区域。
