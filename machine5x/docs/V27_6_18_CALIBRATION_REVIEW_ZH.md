# MotionLaserSync V27.6.18：标定代码审查与阶段二加速说明

## 0. 基准、交付范围和验证边界

本次修改基于用户最新上传的 `MotionLaserSync_v27_6_17_KINEMATIC_RECALIBRATION_QUALITY_FIX(1).rar`，解包后有326个文件，不是此前缺少examples目录的旧ZIP。该包已经包含examples、controller、config和models资源。

交付的是**源码修订版，不是已经完成VS2022和实机验收的发行版**。已在Linux x86_64、g++14.2.0、C++17、Release下编译并运行可移植数值/规划/运行时测试；没有在Windows上编译Qt/VTK GUI，也没有连接ECI/ZMC控制器、scanCONTROL或真实五轴机床。已有旧版本仿真结果不是本次实测数据。性能样本由确定性合成生成器产生，不能视为用户机器上的耗时承诺或实机精度证明。

保留原工程根目录CMakeLists.txt、CMakeSettings.json、Qt/VTK/FCL配置、供应商SDK、控制器程序和运行时资源。修改5个现有源码/构建文件，新增2个测试/测速源码；未引入Ceres、额外Eigen依赖或新第三方库。历史版本的MANIFEST/SHA256文件保留为历史记录；本修订的文件校验以 `verification/v27618/revision_manifest.json` 为准。

## 1. 已确认的问题

下面的“原文件行号”均指用户RAR解包后的原始文件，而非修订后的行号。

| 问题 | 原文件位置 | 影响和本次修正 |
|---|---|---|
| 每点重复整套正运动学，数值求导重复遍历点云 | `src/calibration/SingleSphere17Calibration.cpp`，evaluateResiduals约328–353行、numericalJacobian约385–413行、optimizeSubset约653–766行 | 同一profile的每个点共享同一刚体变换，但原版反复计算旋转、轴线等。改为profile级中心/导数和3×3信息累加；LM拒绝步复用原线性化。 |
| 阶段二只优化代表点，完整训练集只验算 | `src/widgets/SingleSphere14CalibrationWidget.cpp`，onSolve约5257–5484行 | 原来每姿态最多3个profile、每profile最多180点；之后validateModel和analyzeObservability并不更新模型。现增加真正的全训练数据联合精修。 |
| D-optimal选集在GUI截断时被改变 | 同一Widget约2951–2957行、preflightAndFilterGeneratedPlan约2839–2916行；核心约2745–2764行 | 原版先请求超额候选，核心按行程重排，再由GUI取前N个安全姿态；该前缀不再是原贪心D-optimal的前N个。现按实际N选择，选入前检查光学、目标和前一姿态到当前姿态的过渡；不再重排后截断。 |
| 信息评价使用固定“下半圆”，与真实可见弧不一定一致 | 核心syntheticProfile约1112–1138行 | 原式固定z=zc−sqrt(rho²−dx²)，不考虑手眼决定的末端朝向、FlipZ及梯形视场。GUI启用实际朝向/梯形裁剪的合成可见弧，并用同一轮廓级信息模型评价。 |
| 已知机床球心仍被旧示教截面否决 | 核心generate约2525–2549行 | 无论hasSphereCenterMachine为何值，都检查旧centerSensor.y是否与球相交。已知固定机床球心时无需要求当前参考姿态仍能看见球；未知球心时保留原相交要求。 |
| 留出验证的独立X/Z/Y设置被质量模式覆盖 | Widget，plannerConfigFromUi约2708行；onGenerateValidationPlan约3032行之后 | 质量模式开启后，验证函数设置的legacy X/Z/Y网格没有被使用。本次在独立验证分支明确关闭质量模式，使X/Z/Y独立目标生效，仍保留B/C半步错开及与训练姿态的距离筛选；生成后增加完整安全预检。原来B/C隔离已存在，不能把原验证笼统说成完全不独立。 |
| 有限代价、迭代用尽与满足停止条件未区分 | 核心optimizeSubset及solve；Widget onSolve | 新增optimizationConverged；阶段二全量精修未达到停止条件时GUI不接受。核心success保留兼容语义，外部CLI调用应同时检查该标志和质量/秩。 |
| 无效输入、姿态数量和失败JSON边界 | 核心solve/analyzeObservability/CSV/JSON；Widget onSolve | 拒绝NaN/Inf、负权、无正权点、无效尺度等；GUI实际检查至少8个不同姿态；失败诊断中的非有限数写null，成功模型禁止含非有限基本数值。 |

### 1.1 未作为错误改动的内容

本次没有把D、γ、η的既有定义改成另一个几何约定。原候选生成采用：

```
h = R * sin(gamma)
x = D * sin(eta)
z = D * cos(eta)
y = +h 或 -h
```

这一分支用于生成传感器坐标中的目标球心。B/C给定后，通过当前模型和线性轴方向矩阵求XYZ，再受行程、光学和过渡约束筛选。现有坐标顺序、旋转正负号、FlipX/FlipZ、DPOS/MPOS会话约定没有被重新定义。新的逆/正向一致性测试检查目标球心可恢复；这不等于已验证真实机床的轴方向配置。

现有GUI已经使用QtConcurrent、进度和取消；慢的主要来源是数值核，不是“没有开线程”。新版本沿用该框架，不增加嵌套多线程。

## 2. 17项参数到底辨识了什么

项目的Physical17Parameters包括11项机床几何参数和6项手眼参数。球心未知时，状态还包括3个固定球心坐标，共20个量。原头文件和applyTranslationGauge已明确给出3维平移规范自由度：同步改变C/B轴线位置、手眼平移和球心位置，可以保持全部球面距离残差不变。

因此，在当前“单个固定球、未知机床球心”的模型中，充分激励时：

```
保留的物理参数：17
消去球心后的独立物理方向：14
完整状态：17物理 + 3球心 = 20
完整数据Jacobian的期望秩：17
求解坐标：14个独立物理方向 + 3球心
手眼tx/ty/tz：固定为传入初值中的规范锚点
```

本次保留了原有锚定方式。输出的17个数不意味着阶段二单靠球面数据独立测出了全部17个物理量。阶段一手眼平移作为锚点，可以选定一个规范，但它本身也带有阶段一名义机床假设及误差。

可观测性计算在特征值分解前投影掉这3个**显式已知**规范方向，避免有限差分/正规矩阵舍入泄漏伪造第15–17个物理非零奇异值。这不是强行将任何数据的秩写成14；重复同一姿态的数据仍会被测出额外退化。测试包含这一反例。

秩通过只是局部可观测性条件，不等于高精度、全局最优或全部参数都有可信绝对物理值。要独立确定这些规范相关的绝对量，需要能真正打破该自由度的外部几何基准/已知坐标约束，不能仅靠提高迭代次数。

## 3. 加速原理：保留原始点和Huber目标，不靠降精度

原残差为：

```
p_M = R_i(theta) * s_ij + t_i(theta)
r_ij = ||p_M - c_M|| - sphereRadius
```

对一个固定profile，其传感器到机床的旋转R_i是正交的；线性轴不正交参数影响该姿态的平移，不破坏旋转的正交性。因此可以等价写为：

```
c_Si = R_i^T * (c_M - t_i)
r_ij = ||s_ij - c_Si|| - sphereRadius
```

一个profile的所有点只共享一个c_Si。新代码只对该3维中心的运动学依赖做中心差分；点到球面距离对中心的导数，以及未知机床球心到传感器中心的导数用解析式。这是**混合求导**，不是声称完成了全模型解析Jacobian或自动微分。

记B_i=dc_Si/dx（对归一化状态求导），A_i为逐点距离对c_Si的导数，则：

```
J_i = A_i * B_i
H_i = B_i^T * (A_i^T W_i A_i) * B_i
g_i = B_i^T * (A_i^T W_i r_i)
```

逐点只累加3×3矩阵与3维向量，再合并到17维或20维信息矩阵。W包含原始点权重和原有Huber IRLS权重。所有全量精修点都保留；没有把profile拟合成一个圆心后丢掉原始点，也没有更换代价函数。

计算复杂度的主要变化是：昂贵运动学/运动学差分从“每点”移到“每profile”，稠密正规矩阵累加从“每点d²”变为“每点常量工作 + 每profile d²”。仍使用原LM体系，拒绝步时只增加阻尼、重解小系统，不重复构建同一Jacobian。没有使用fast-math、放宽RMS门槛、减少最终训练点或取消质量约束。

### 3.1 阶段二GUI新流程

```
阶段一/当前模型初值
   -> 代表集初始化（3 profiles/姿态，最多180点/profile）
      2个确定性起点，2轮块交替，每块最多6步，联合最多60步
   -> 全训练集精修（全部有效profile、全部已保留球面点）
      1个起点，0轮块交替，联合最多40步
   -> 全量RMS/max、规范投影后的rank/condition
   -> 停止条件与原GUI质量门槛
   -> 冻结模型，独立hold-out验证
```

这里“全部点”指进入stage2TrainingObservations的训练点，不包括之前被采集/球面提取判定无效的原始激光背景点，也不包括阶段一或hold-out点。没有将阶段一数据回灌阶段二优化。

保留GUI原RMS≤0.050mm、condition≤1e9、足够秩等门槛。**这些是原项目阈值的延续，不是本次重新论证过的计量指标。** 进度分为代表集约0–55%和全量约55–100%；轮廓信息累加检查取消，已有采样不会被删除。非常大的单profile、残差验算或最终诊断仍不是硬实时可中断代码。

## 4. 姿态生成的修正及边界

训练计划在贪心信息选择时调用现有的光学、MachineSafety目标和过渡检查器。静态不合格候选缓存排除；因当前前驱导致过渡不合格的候选，可以在前驱变化后重新考虑。保留已经检查的采集顺序，不再对超额选集旅行重排后截断。选中第一项仍优先靠近参考姿态，其后才是受约束的贪心信息增益选择；这不是全局最优姿态/路径联合优化的证明。

合成弧现在与末端朝向的半球一致，并裁剪实际FlipZ方向的梯形视场。测试包含handeye_tz=-200mm、球心sensor_z=95mm的例子：末端位于sensor_z=200mm，可见弧应朝较大Z方向，不能固定使用较小Z的半弧。

保留光学预检中的75°最小可见弧、6mm最小弧长、0.5mm平面相交余量及既有视场/安全边界。MachineSafety检查器存在时才可调用，因此“可生成”不表示可以绕过设备端限位、急停、夹具和实际安装检查。此次没有放宽碰撞判断，也没有修改运动执行程序。

增加有限数/有序范围/候选网格大小保护：每维最多10000个间隔，笛卡尔候选规模上限200000。训练GUI要求精确达到请求数量；不够就明确失败，而不是静默输出少量姿态。为兼容独立验证候选池，通用接口默认仍允许可行的较少输出，训练GUI通过requireRequestedPoseCount显式收紧。

独立验证使用错开的B/C和明确的X/Z/Y目标，仍检查与训练姿态的角度距离。验证只评估冻结参数和冻结球心，不重新拟合来掩盖误差。重新求解会使旧验证结论失效；已有hold-out采样可以保留，但必须重新计算对新模型的验证结果。

## 5. 数值正确性与性能

### 5.1 等价性测试

新测试在同一真实定义的正运动学函数mapSensorPoint上独立重算原公式，与优化后的传感器球心公式对照：

| 检查 | 本次结果 |
|---|---:|
| 残差最大绝对差 | 8.52651e-14 mm |
| 混合Jacobian与原逐点中心差分最大绝对差 | 2.18557e-7（归一化列，按各列尺度检查容差） |
| 3×3分解后的H与显式JᵀWJ最大元素相对差 | 1.00758e-15 |
| 分解后梯度与显式JᵀWr最大元素相对差 | 7.53086e-16 |

H/g的比较使用同一混合Jacobian；另有独立原逐点差分和加权Huber目标差分检查，不能把这几种误差混为同一个指标。测试覆盖零权、部分降权、非零残差和稳健权重。

### 5.2 测速定义和结果

Linux x86_64容器，g++14.2.0，`-O3 -DNDEBUG -std=c++17`，各3次取中位数。合成数据：42个不同XYZCB姿态，每姿态8个profile，每profile512点，共336个profile、172032点；独立x/z高斯噪声标准差0.003mm，稀疏0.06mm离群点及部分0.5权重。代表集与原GUI上限相同：126个profile、22680点。固定种子20260921，真值与初值来自原工程物理17测试夹具。

| 任务 | 原版中位数 | 改进版中位数 | 说明 |
|---|---:|---:|---|
| 相同代表集、相同求解配置 | 12.403099s | 0.039837s | 两者均2起点/2轮交替/块6/联合18；约311.3倍数值核加速 |
| 172032点完整验算+可观测性分析 | 2.707548s | 0.004639s | 固定同一真值模型，不包含优化 |
| 代表集+全量精修 | 未运行原版此流程 | 0.069080s | 代表联合18，再全量联合40 |
| 按修改后GUI迭代预算的完整数值流程 | 原GUI无对应全量精修 | 0.067530s | 代表联合60，再全量联合40；不含GUI绘制、CSV读取或采集 |

同配置A/B代表集结果：RMS约0.003597861433mm，差约1.084e-13mm；17个输出值的最大直接差约1.40e-10（不同参数有不同单位），球心坐标最大差约1.80e-10mm。只用于验证相同问题的解一致，不是声称真实参数有该级精度。

全量精修后，完整训练集Huber代价从0.834800374725587降至0.834466615746541；完整集RMS为0.00358257869255963mm，达到停止条件，物理秩14、完整秩17。没有拿代表集RMS和完整集RMS的差直接宣称泛化精度改善。

上述加速倍数针对这份代码及此合成样本的热点，不是一般LM算法的普遍速度提升；实机噪声、不同点数/姿态数、初值偏差、CPU及编译器会改变耗时和收敛结果。未运行用户的真实采集CSV。

### 5.3 测试覆盖

最终选定的26项可移植CTest全部通过，包括原单球14/物理17与两阶段、球面质量几何、离线规划保护、可见弧与FlipZ、RTCP/运行时、NC样例解析和新增轮廓信息测试。完整日志在verification/v27618/ctest_portable.log。

工程登记48项测试，**不是48项全通过**。其余22项未纳入通过统计：

- 1项p0_real_core_mock_transport依赖的Qt替身入口头文件（例如QObject）在原RAR中缺失，完整构建尝试因此失败；当前环境也没有真实Qt开发包。
- 21项Python contract测试引用的verify脚本不在原RAR中。未伪造脚本、未删除这些已有测试声明来制造全绿结果。

没有完整编译Windows GUI；Widget调用流程做了静态对照，核心与共享头文件已经过上述可移植编译。没有做长期采集、设备断连、异步销毁、线程竞态、所有控制器流程或全部工程逐行形式化验证。本次重点是用户提出的运动学标定、姿态生成和阶段二数值求解。

## 6. 文件与使用方法

### 6.1 修改文件

```
include/calibration/SingleSphere14Calibration.h
include/calibration/SingleSphere17Calibration.h
src/calibration/SingleSphere17Calibration.cpp
src/widgets/SingleSphere14CalibrationWidget.cpp
tests/CMakeLists.txt
```

新增：

```
tests/v27618_profile_block_tests.cpp
tools/v27618_calibration_benchmark.cpp
```

完整ZIP保留全部原文件。PATCH_FILES.zip可覆盖到**本次RAR对应的版本**工程根目录；不要混覆到结构不同的旧版本。补丁ZIP内还有说明、测试日志和统一diff。

### 6.2 VS2022建议操作

先复制当前可运行工程作为回退，并备份阶段一JSON、阶段二训练CSV、hold-out CSV和最终标定JSON。解压完整修订版到新目录，或把补丁ZIP按相对目录覆盖到工程根。

保持现有Qt/VTK/FCL/SDK路径不变。VS2022重新打开CMake工程，删除缓存并重新配置，选择x64-Release后全部生成。未提供Windows可执行文件，不能把离线CTest通过当作VS2022编译通过。

先做离线数据回归：加载与采样时相同坐标约定的阶段一初值和阶段二CSV，执行最终求解，检查完整profile/点数、停止状态、RMS/max、rank14/17、full-rank17/20、condition和独立hold-out。训练解通过后再重新执行留出验证，未通过之前不将结果用于正式RTCP/扫描。

有实机运动前仍应按既有流程核对轴方向、参考坐标、行程限位、球与夹具、数字孪生模型、碰撞预检和低速单步动作。规划输出和合成测试不构成运动安全保证。

### 6.3 可复现实验命令

只构建标定数值测试，避开原包中缺失的Qt替身与contract脚本：

```sh
cmake -S . -B build-calibration -G Ninja -DV21_TESTS_ONLY=ON -DCMAKE_BUILD_TYPE=Release
cmake --build build-calibration --target v20_2_single_sphere17_tests v27618_profile_block_tests v27618_calibration_benchmark
ctest --test-dir build-calibration --output-on-failure -R '^(v20_2_single_sphere17|v27618_profile_block_equivalence)$'
./build-calibration/tests/v27618_calibration_benchmark ui_schedule
```

Windows Developer Command Prompt使用对应.exe路径。原版对比需要原RAR解压目录；同一benchmark源以原include和原SingleSphere17Calibration.cpp编译即可（不定义MLS_BENCH_FAST）。改进版的benchmark目标已定义MLS_BENCH_FAST。

本报告中的测量记录、原/新源码SHA256和统一diff用于复核。性能提升不依赖修改测量设备配置，也不依赖降低最终采样点数。
