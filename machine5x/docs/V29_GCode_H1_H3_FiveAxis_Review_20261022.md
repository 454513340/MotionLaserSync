# MotionLaserSync V29：STEP—预扫描—五轴扫描—G代码接口审查与离线修订

**审查日期：2026-10-22**　**依据：用户上传的 `MotionLaserSync_V29.rar`（776 个归档条目）**  
**输出类别：源码审查 + 离线补丁 + 可解析的 G 代码预览；不是实机调试或运行许可。**

## 0. 总结及范围

V29 并非缺少五轴运动、同步采集、数字孪生这些底层模块。真正缺口在于：**零件位姿、M6 规划轨迹、数字孪生仿真、G 代码文件和 H1/H3 硬件时序尚未由同一个经过验证的执行合同贯穿**。本次将 M6 的同一条离线 XYZCB 轨迹序列化为带 H1/H3 同步声明的 NC 预览，增加解析、状态检查、数据来源字段与拒绝执行机制；不改变既有 H1/H3 硬件引脚映射，不绕过 G 代码硬件同步安全闸门。

**仍未完成的用户目标：** 用户这次没有上传目标零件的实际 STEP、预扫描原始 `.scan`/有序点云、独立 CAD→机床配准结果、当前机床位姿、夹具/刀具/机架碰撞模型和现场硬件时序验收记录，因此无法算出具体工件 `T_machine_from_CAD`，也无法认证路径或实际下发/执行扫描 G 代码。源码中的 M5.2/H1 验证和 M6 离线规划不能代替对任意 STEP+自测点云的通用配准验证。

## 1. 已确认调用关系和主要原始缺口

| 环节 | 现有源码入口 | 已确认事实 | 本次处置/后续必做 |
|---|---|---|---|
| CAD 模型 | `src/widgets/DigitalTwinScanWidget.cpp`；`src/twin/CadGeometryAdapter.cpp` | M0/M1 用 OCCT 可导入 STEP；CLI 可不带 OCCT | 必须检验 STEP 单位、坐标系及工件基准，不把 CAD 文件路径冒充配准变换 |
| 实测数据 | 数字孪生 M2；`src/utils/CompactScanFormat.cpp` | 可导入压缩 `.scan`；存 profile、时间/五轴位姿等信息 | 目标工件真实预扫描文件未提供，无法输出实测 `T_machine_from_CAD` |
| 配准/定位 | M4 对比、M5.2 位置几何检查 | 现有诊断依赖特定工况；不应将所有输出默认视为 6DoF 全局绝对配准 | 后续建立 `RegistrationResult`，包含 `T_machine_from_CAD(4×4)`、残差、重叠支持、单位、版本、来源与人工批准 |
| 规划 | `src/twin/TwinM6Continuous.cpp` | M6 已规划具有时刻、XYZCB 和 Scan/Transfer 角色的轨迹 | 新增 `TwinM6NcPreview`，从同一个 `M6Report.path` 导出 NC 离线预览 |
| 仿真 | M3/M4/M5/M6/M8 | 可以计算覆盖、部分 FCL/动力学检查；不是完整多姿态光学或整机碰撞认证 | 不自动把离线检查视为硬件运行授权；需实机碰撞模型、RTCP/初始姿态/制动和扫描密度验证 |
| 运行时 NC | `src/runtime/GCode5D.cpp`; `src/gcode/GCodeParser.cpp` | 已有 `G00/G01/G02/G03/G05`、`G43.4/G49`、`G93/G94`、五轴 XYZCB | 修正 G05 后续轴点的模态；扩充 H1/H3 文件协议及强校验 |
| 控制执行 | `src/gcode/GCodeExecutor.cpp`; `src/core/ZmcController.cpp` | 已有五轴 Merge/SP 插补，及 17 参数 RTCP 计算；原代码在硬件采集 G 代码执行前设置安全闸门 | 加载前校验、拒绝 OfflineOnly、核对 H1/H3/频率/触发沿；**保留原硬件闸门** |
| 同步采集 | `include/core/SyncManager.h`; `controller/ECI3808_V32_16_H1_H3_ONLY.bas` | H1/H3 有明确电子线路；H2 已退役 | 禁止 H2；不得用 M50/M51 替代实际触发/锁存时序的验证 |
| 点云归档 | `src/utils/CompactScanFormat.cpp`; 异步保存管理器 | `.scan` 包含有序记录及关联五轴信息；`scan_lz4`/`scan_zstd` **别名实际上调用 QtZlib** | M60 使用声明频率记录名义间隔；所有真实 profile 时间戳仍为准确重建优先来源 |

## 2. 两种硬件同步的区别（不能用相同 NC 假设全部等价）

**H1，拓扑号 `1`**：以 ECI3808 `OP14/PWM` 为硬件触发源，外触发的激光轮廓与 `IN0/R0` 相关联，五轴锁存命令 `REGIST(104)`。文件必须写 `; HardwareTopology: H1` 与 `; HardwareFrequencyHz: ...`；核对现场接线、电平极性、脉冲时序、硬件频率、激光触发模式及位置锁存协议。

**H3，拓扑号 `3`**：以传感器内部轮廓时钟为采集源，经 CMM/RS422 接 `IN1/R1`，使用 `REGIST(114)` 或 `REGIST(115)` 采样五轴位置。文件须额外记录 `; CmmR1Edge: 114` 或 `115`；115/114 必须与现场已验证的实际信号沿相符。

**禁止 H2(2)**。`M50` 是“开始扫描生命周期”、`M51` 是“停止并完成尾帧收尾”，不是直接切换电气输出的硬编码 `M` 指令。真正的硬件配置通过 `SyncManager` 与控制器 BASIC 完成，需一致且独立验收。

## 3. 本次新定义的 G 代码离线协议

### 3.1 强制保留的头字段

```nc
; Program: M6 offline five-axis trajectory - NOT FOR EXECUTION
; OfflineOnly: true
; SimulationVerified: false
; RegistrationVerified: false
; SyncMode: Hardware
; HardwareTopology: H1
; HardwareFrequencyHz: 200
; ProgramFrame: MACHINE_JOINTS_XYZCB
; LinearUnit: mm
; RotaryUnit: deg
; PartPoseId: UNVERIFIED
; CalibrationId: UNVERIFIED
```

`H3` 则将 `HardwareTopology` 写为 `H3`，并写入 `CmmR1Edge: 114` 或 `115`。上述 200 Hz 只是**合成测试例子**，不是对用户当前设备的建议值。`PartPoseId` 和 `CalibrationId` 必须来自独立、可审计的变换/标定工件，不能拿 CAD 路径或“physical17”字样伪装成验收通过的版本号。

### 3.2 指令合同与安全顺序

| 指令/字段 | 本项目解释 | 重点校验 |
|---|---|---|
| `G21`, `G90` | 长度毫米、绝对坐标 | 五轴 C/B 仍以角度定义，不是全轴“mm” |
| `ProgramFrame: MACHINE_JOINTS_XYZCB` + `G49`/不启用 `G43.4` | XYZ 为机床**关节**坐标，C/B 为关节转角 | 必须与离线 M6 输出保持一致 |
| `ProgramFrame: RTCP_TCP_XYZCB` + `G43.4` | XYZ 为控制点/TCP 任务坐标，实际轴由物理 17 参数 RTCP 求解 | 不是上一类数据简单加一条 G43.4 就能互换；需任务坐标标定、绝对平移规约与轴限位 |
| `G93` + `F=60/Δt` | 每段逆时间进给（F 单位 min^-1） | 仅保留名义段时长，不能保证机床按该时间经过，也不能替代采集密度验证 |
| `G05 S0` | 五轴连续非采集 transfer 路段 | 不得在 `M50` 已开启期间穿插 `S0` 路段 |
| `M50` -> `G05 S1` -> `M51` | 一个明确的扫描路段生命周期 | 不得在其间 `G00`、保存或模糊切换采集状态 |
| `M60 L"..."` -> `M61` | 保存当前扫描段并清空 | M51 后，先确认保存已提交；最终 M63 保证所有任务完成 |
| `M63` -> `M30` | 保存屏障及完成 | 不得用 `M30` 自动表示现场运行已通过 |

**不能把 M6 轨迹上任意相邻点的几何间距等同于真实 2950 的触发 profile 间隔。**对直线匀速近似 `Δs≈v/f`；实际五轴应由机床运动学、传感器照射/接触轨迹及**实际触发时刻与锁存位姿**，求每条轮廓的表面法向于激光线方向上的有效间隔，再验证采样支持和遮挡。

### 3.3 M6 导出内容

`TwinM6Adapter::exportReport` 在原 CSV/JSON/压缩模拟 `.scan` 分卷旁新增：

- `<stem>_OFFLINE_ONLY_H1.nc`
- `<stem>_OFFLINE_ONLY_H3.nc`

两者采用同一个 `M6Report.path`、相同 XYZCB/扫描分组，只改变明确标注的拟采用同步拓扑。导出清单会包含预览文件名与 `gcodePreviewHardwareAuthorized=false`。同步保留生成器起始关节锚定状态、CAD 来源和运动学来源；若 M6 没有现场认证的起始运动位姿，第一条 `G00` **仅为非机动预览**。

## 4. 修改清单（实际已改代码）

| 文件 | 修改作用 |
|---|---|
| `include/twin/TwinM6NcPreview.h`、`src/twin/TwinM6NcPreview.cpp` | 新增独立于 Qt 的 M6 离线 G 代码生成、检查时间单调/有限坐标/规划边界/频率/角色和 H1/H3；安全头标记 |
| `src/twin/TwinM6Adapter.cpp` | 将 NC 预览集成原有 M6 归档、原子写入及 JSON manifest，不假造姿态标定 ID |
| `src/runtime/GCode5D.cpp` | 使 `G05` 连续轨迹保持正确模态，其后轴点不再误落入之前的 G00 模态 |
| `include/gcode/GCodeTypes.h` | 增加同步文件声明、拓扑、频率、H3 锁存沿、OfflineOnly、坐标语义、姿态/标定来源字段 |
| `include/gcode/GCodeParser.h`、`src/gcode/GCodeParser.cpp` | 严格解析 H1/H3 头、程序级校验、S0/S1 与 M50/M51 配对、拒绝危险跨段操作、G05 五轴完整性、保留头字段导出 |
| `src/gcode/GCodeExecutor.cpp` | 加载时重新验证；拒绝 OfflineOnly；核对程序现场 H1/H3/频率/锁存沿；G01 批处理不跨 RTCP 或进给模式；M60 名义采样间隔按文件频率计算 |
| `src/widgets/GCodeWidget.cpp` | 文件中显式声明的同步模式不得由 GUI 选项静默覆盖 |
| `CMakeLists.txt`、`tests/twin/twin_m6_nc_preview_tests.cpp` | 纳入离线 NC 生成与真实运行的断言测试；缺失的上游传感器计时测试源码仅警告跳过 |

**保持原有安全拒绝机制：**`GCodeExecutor::start()` 仍会在采集/硬件同步场景拒绝直接执行，因为现阶段仍缺“从文件到电气触发和实测位姿”的独立经过批准的闭环认证；现在新增的文件头**不能**替代这种认证。

## 5. 核心缺口——STEP 到 G 代码仍需哪些模块

1. **通用输入验收：**确定 STEP 单位与 CAD 部件坐标、预扫描点云的 `sensor/Machine/part` 来源、profile 索引与有效掩码；只有 PCD XYZ 而无采集姿态时，需说明这不是可追溯五轴计量原始数据。
2. **零件定位结果：**先用 fixture/特征先验及局部/全局配准，求 `T_M_CAD`（四阶齐次变换，`p_M=T_M_CAD*p_CAD`），另存配准代价、重叠率、残差分布、退化方向、几何重复性、初始猜测、人工复核。不能把算法自己生成的 CAD 重建结果当唯一验收基准。
3. **坐标链闭合：**分别给出 `T_M_S(q)`、`T_S_L` 和 `T_M_CAD`，在每个实际触发 `t_i` 重建 `p_M(t_i)=T_M_S(q_i) p_S(t_i)`；明确 `XYZCB` 的 `C/B` 旋转顺序、回转中心、角度单位和 17 参数物理模型版本。`G43.4` 时必须把 TCP 目标转为机床关节空间，不允许关节空间 M6 路点直接被误读为 TCP。
4. **扫描路径合同：**确定条带覆盖/FOV、入射与遮挡、离焦、法向姿态约束、各轴行程、速度/加速度/jerk、相邻 profile 在工件表面上的实际间距、碰撞/切入/撤离与首段从测量 MPOS 到轨迹的安全入口。
5. **仿真与执行一致性：**以相同轨迹比较规划 XYZCB、控制器最终插补/轴限制、M50/M51 以及“触发—锁存—轮廓”一一对应；保存时间戳/锁存位姿和延迟误差统计。需要软件干跑、模拟器/控制器空运行、PLC/急停试验和现场独立批准。
6. **真实输出和尺寸计量：**归档不可覆盖的原始 `.scan`、所有实测 `XYZCB`/时间戳、标定/工件姿态版本、条带/采集片段 ID、孔边/工件边界等几何真值，提供可重放的 PC/STEP 比较与几何量值不确定度；**不应**将可视化融合/补全点当作真实几何尺寸数据。

## 6. 离线测试与验证限制

离线 Linux 构建（不带 Qt、OpenCASCADE、激光驱动或控制器 SDK）：

```bash
cmake -S . -B build_offline -G Ninja \
  -DDIGITAL_TWIN_CLI_ONLY=ON -DTWIN_ENABLE_OCCT=OFF -DCMAKE_BUILD_TYPE=Release
cmake --build build_offline -j4
ctest --test-dir build_offline --output-on-failure
```

**本次实测：23/23 项存在且被构建的离线测试通过**（包含本次新增的 G05/H1/H3 NC 检查）。原项目 CMake 引用了本次 RAR 中不存在的 `tests/twin/sensor_calibration_timing_tests.cpp`，现仅显式警告并跳过该测试，**不能**把此项当通过；缺少 Windows Qt6/VTK/OCCT/ECI/scanCONTROL 实际环境，不宣称 GUI 完整编译、传感器/控制器现场闭环或实机已通过。

**另一个已明确的局限：**生成器输出的是 XYZCB 关节空间 G05 的离线可检查预览，`G93 F` 是时间近似；“同步触发恰好发生在某个 G05 路点”没有得到认证。后续将此类文件投入运行前，必须先验证实机插补的时域误差及采样时刻位姿、五轴动态与夹具/探头碰撞、RTCP 和姿态初值，并由负责设备安全的工程人员审批。

## 7. 推荐下一轮真实工件验收数据

请提供：**实际 STEP + 同一零件预扫描 `.scan`（优于无序 PCD）+ 17 参数标定版本/坐标系说明 + H1/H3 对应线缆及极性验收记录 + 现场轴行程/速度/加速度/jerk/碰撞模型 + 机床当前 XYZCB**。收到后才可能计算真实工件位姿、输出具体扫描区域/速度/频率以及逐条可核验的 NC。任何高危险运动/硬件触发均需在现场控制系统安全机制下验证。
