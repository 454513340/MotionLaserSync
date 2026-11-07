# 算法模块开发指南

## 概述

本文档详细描述了MotionLaserSync系统新增的三个算法模块的架构设计、接口规范和扩展方法。

---

## 1. 模块架构

### 1.1 目录结构

```
include/algorithms/
├── AlgorithmTypes.h              # 通用数据类型定义
├── IAlgorithm.h                  # 算法接口基类
├── CalibrationAlgorithms.h       # 标定算法声明
├── PathPlanningAlgorithms.h      # 路径规划算法声明
└── PointCloudProcessingAlgorithms.h  # 点云处理算法声明

src/algorithms/
├── CalibrationAlgorithms.cpp     # 标定算法实现
├── PathPlanningAlgorithms.cpp    # 路径规划算法实现
└── PointCloudProcessingAlgorithms.cpp  # 点云处理算法实现

include/widgets/
├── CalibrationWidget.h           # 标定UI
├── PathPlanningWidget.h          # 路径规划UI
└── PointCloudProcessWidget.h     # 点云处理UI

src/widgets/
├── CalibrationWidget.cpp
├── PathPlanningWidget.cpp
└── PointCloudProcessWidget.cpp
```

### 1.2 类图

```
                    IAlgorithm (基类)
                         │
         ┌───────────────┼───────────────┐
         │               │               │
 标定算法模块      路径规划模块      点云处理模块
         │               │               │
    ├─ UncertaintyCalibration   ├─ PathPlanner        ├─ PointCloudPreprocessor
    ├─ KinematicCalibration     ├─ GCodeGenerator     ├─ PointCloudSegmenter
    └─ HandEyeCalibration       ├─ PathOptimizer      ├─ DeepLearningDetector
                                └─ AdaptivePathPlanner└─ PointCloudRegistration
```

---

## 2. 算法基类接口 (IAlgorithm)

### 2.1 接口定义

```cpp
class IAlgorithm : public QObject
{
    Q_OBJECT
public:
    // 必须实现的纯虚函数
    virtual QString name() const = 0;           // 算法名称
    virtual QString description() const = 0;    // 算法描述
    virtual bool initialize() = 0;              // 初始化
    virtual bool execute() = 0;                 // 执行算法
    
    // 可选重写
    virtual QString version() const;            // 版本号，默认"1.0.0"
    virtual void cancel();                      // 取消执行
    
    // 状态查询
    bool isCancelled() const;
    bool isRunning() const;
    QString lastError() const;

signals:
    void progressUpdated(const AlgorithmProgress& progress);  // 进度更新
    void completed(bool success);                             // 完成信号
    void errorOccurred(const QString& message);               // 错误信号
    void logMessage(const QString& message);                  // 日志信号

protected:
    void reportProgress(int current, int total, const QString& message);
    void setError(const QString& error);
    void log(const QString& message);
    
    bool m_cancelled = false;
    bool m_running = false;
    QString m_lastError;
};
```

### 2.2 使用示例

```cpp
// 创建算法实例
auto calib = std::make_unique<UncertaintyCalibration>();

// 连接信号
connect(calib.get(), &IAlgorithm::progressUpdated, this, [](const AlgorithmProgress& p) {
    qDebug() << "Progress:" << p.current << "/" << p.total << p.message;
});
connect(calib.get(), &IAlgorithm::completed, this, [](bool success) {
    qDebug() << "Completed:" << success;
});

// 配置并执行
calib->setResolution(0.001);
calib->addRepeatabilityData(measurements, reference);
calib->initialize();
calib->execute();
```

### 2.3 算法注册机制

在每个算法实现文件末尾使用宏注册：

```cpp
REGISTER_ALGORITHM(UncertaintyCalibration)
REGISTER_ALGORITHM(PathPlanner)
```

通过工厂创建：

```cpp
IAlgorithm* algo = AlgorithmFactory::instance().create("PathPlanner");
```

---

## 3. 系统标定模块

### 3.1 测量不确定度标定 (UncertaintyCalibration)

#### 功能描述
按照GUM (Guide to the Expression of Uncertainty in Measurement) 标准评估传感器测量不确定度。

#### 核心接口

```cpp
class UncertaintyCalibration : public IAlgorithm
{
public:
    // 数据输入
    void addRepeatabilityData(const QVector<double>& measurements, double referenceValue);
    void addLinearityData(const QVector<double>& measuredValues, const QVector<double>& referenceValues);
    
    // 参数设置
    void setResolution(double resolution);           // 仪器分辨率 (mm)
    void setSpecifiedAccuracy(double accuracy);      // 规格精度 (mm)
    void setTemperatureVariation(double deltaT);     // 温度变化 (°C)
    void setThermalCoefficient(double coeff);        // 热膨胀系数 (1/°C)
    
    // 结果获取
    const UncertaintyResult& result() const;
    QString generateReport() const;
    void clearData();
};
```

#### 结果数据结构

```cpp
struct UncertaintyResult {
    double repeatability;        // 重复性不确定度
    double linearity;            // 线性度误差
    double resolution;           // 分辨率不确定度
    double combinedUncertainty;  // 合成标准不确定度
    double expandedUncertainty;  // 扩展不确定度 (k=2)
    double coverageFactor;       // 包含因子
    QString report;              // 详细报告
    bool isValid;
};
```

#### 算法流程

1. 计算A类不确定度（重复性测量统计）
2. 计算B类不确定度（分辨率、温度、仪器精度）
3. 合成标准不确定度：`uc = sqrt(u1² + u2² + ...)`
4. 扩展不确定度：`U = k × uc` (k=2, 95%置信度)

#### 扩展建议

- [ ] 添加再现性测量支持
- [ ] 支持自定义不确定度分量
- [ ] 添加协方差考虑
- [ ] 支持不同分布类型（正态、均匀、三角）

---

### 3.2 运动学参数标定 (KinematicCalibration)

#### 功能描述
标定机床运动学参数误差，生成误差补偿表。

#### 核心接口

```cpp
class KinematicCalibration : public IAlgorithm
{
public:
    // 数据输入
    void addCalibPoint(const QVector3D& commandPos, const QVector3D& measuredPos, int axis = -1);
    void addCalibPoints(const QVector<QVector3D>& commandPositions, const QVector<QVector3D>& measuredPositions);
    
    // 参数设置
    void setReferenceFrame(const QMatrix4x4& frame);
    void setMaxIterations(int maxIter);
    void setConvergenceThreshold(double threshold);
    
    // 结果获取
    const KinematicCalibResult& result() const;
    QVector3D correctPosition(const QVector3D& commandPos) const;
    bool exportCompensationTable(const QString& filePath) const;
};
```

#### 结果数据结构

```cpp
struct KinematicCalibResult {
    QVector<double> deltaA, deltaAlpha, deltaD, deltaTheta;  // DH参数偏差
    QMatrix4x4 correctionMatrix;                              // 校正矩阵
    double rmse;                                              // 均方根误差
    double maxError;                                          // 最大误差
    int iterations;                                           // 迭代次数
    bool isValid;
};
```

#### 扩展建议

- [ ] 实现完整的DH参数优化
- [ ] 支持6自由度误差模型
- [ ] 添加激光干涉仪数据导入
- [ ] 支持分段线性补偿

---

### 3.3 手眼标定 (HandEyeCalibration)

#### 功能描述
标定传感器相对于机床法兰的位姿关系。

#### 支持的方法

```cpp
enum class Method {
    Tsai,        // Tsai-Lenz 方法
    Park,        // Park 方法
    Horaud,      // Horaud 方法
    Daniilidis,  // 双四元数方法
    PointCloud   // 点云配准方法
};
```

#### 核心接口

```cpp
class HandEyeCalibration : public IAlgorithm
{
public:
    // 数据输入
    void addPosePair(const QMatrix4x4& robotPose, const QMatrix4x4& targetPose);
    void addPointCloudData(const QMatrix4x4& robotPose, const QVector<QVector3D>& sensorPoints);
    
    // 参数设置
    void setMethod(Method method);
    void setInitialGuess(const QMatrix4x4& guess);
    
    // 结果获取
    const HandEyeCalibResult& result() const;
    QVector3D transformToMachine(const QVector3D& sensorPoint, const QMatrix4x4& robotPose) const;
    QVector<QVector3D> transformPointCloud(const QVector<QVector3D>& sensorPoints, const QMatrix4x4& robotPose) const;
    
    // 持久化
    bool saveCalibration(const QString& filePath) const;
    bool loadCalibration(const QString& filePath);
};
```

#### 扩展建议

- [ ] 实现完整的Tsai-Lenz算法
- [ ] 添加RANSAC鲁棒估计
- [ ] 支持在线标定更新
- [ ] 添加误差分析和可视化

---

## 4. 路径规划模块

### 4.1 基础路径规划器 (PathPlanner)

#### 支持的路径类型

```cpp
enum class ScanPathType {
    Raster,     // 栅格扫描（之字形）
    Spiral,     // 螺旋扫描
    Contour,    // 轮廓跟踪
    Adaptive,   // 自适应扫描
    Custom      // 自定义路径
};
```

#### 核心接口

```cpp
class PathPlanner : public IAlgorithm
{
public:
    // 输入设置
    void setPointCloud(const QVector<QVector3D>& points);
    void setBoundary(const QVector3D& min, const QVector3D& max);
    void setParameters(const PathPlanParams& params);
    
    // 参数快捷设置
    void setPathType(ScanPathType type);
    void setLineSpacing(double spacing);
    void setPointSpacing(double spacing);
    void setScanSpeed(double speed);
    void setRapidSpeed(double speed);
    void setSafeHeight(double height);
    
    // 结果获取
    const ScanPath& path() const;
    const QVector<ScanPathPoint>& pathPoints() const;
    QVector<QVector3D> previewPath() const;
};
```

#### 参数数据结构

```cpp
struct PathPlanParams {
    ScanPathType pathType = ScanPathType::Raster;
    ScanDirection direction = ScanDirection::XPositive;
    double lineSpacing = 1.0;      // 行间距 (mm)
    double pointSpacing = 0.5;     // 点间距 (mm)
    double scanSpeed = 10.0;       // 扫描速度 (mm/s)
    double rapidSpeed = 50.0;      // 快移速度 (mm/s)
    double safeHeight = 10.0;      // 安全高度 (mm)
    double nominalHeight = 95.0;   // 标称测量高度 (mm)
    bool adaptiveHeight = false;   // 自适应高度
};
```

#### 扩展建议

- [ ] 实现真正的轮廓跟踪算法
- [ ] 添加障碍物避让
- [ ] 支持多区域扫描
- [ ] 添加路径平滑处理

---

### 4.2 G代码生成器 (GCodeGenerator)

#### 支持的风格

```cpp
enum class GCodeStyle {
    Standard,  // 标准G代码
    Fanuc,     // FANUC风格
    Siemens,   // SIEMENS风格
    ZMC        // 正运动控制器风格
};
```

#### 核心接口

```cpp
class GCodeGenerator : public IAlgorithm
{
public:
    void setPath(const ScanPath& path);
    void setStyle(GCodeStyle style);
    void setProgramNumber(int num);
    void setWorkCoordinate(int coord);
    void setGenerateScanCommands(bool enable);
    void setExposureTime(int exposureUs);
    
    QString gcode() const;
    bool saveToFile(const QString& filePath) const;
    double estimatedTime() const;
    int lineCount() const;
};
```

#### 扩展建议

- [ ] 添加更多控制器风格支持
- [ ] 支持宏程序生成
- [ ] 添加刀具补偿指令
- [ ] 支持子程序调用

---

### 4.3 路径优化器 (PathOptimizer)

#### 优化目标

```cpp
enum class OptimizationGoal {
    MinimizeTime,      // 最小化时间
    MinimizeDistance,  // 最小化距离
    MinimizeTurns,     // 最小化转弯
    Balanced           // 平衡优化
};
```

#### 扩展建议

- [ ] 实现TSP求解器
- [ ] 添加遗传算法优化
- [ ] 支持约束优化

---

## 5. 点云处理模块

### 5.1 点云预处理器 (PointCloudPreprocessor)

#### 支持的滤波器

```cpp
enum class FilterType {
    StatisticalOutlier,  // 统计离群点滤波
    RadiusOutlier,       // 半径离群点滤波
    PassThrough,         // 直通滤波
    VoxelGrid            // 体素网格降采样
};
```

#### 核心接口

```cpp
class PointCloudPreprocessor : public IAlgorithm
{
public:
    void setInputCloud(const QVector<QVector3D>& points);
    void setFilterType(FilterType type);
    void setStatisticalParams(int k, double stdMul);
    void setRadiusParams(double radius, int minNeighbors);
    void setVoxelSize(double size);
    
    const QVector<QVector3D>& outputCloud() const;
    const QVector<QVector3D>& normals() const;
    int removedPointCount() const;
};
```

#### 扩展建议

- [ ] 添加双边滤波
- [ ] 支持条件滤波
- [ ] 实现移动最小二乘平滑
- [ ] 添加法向量优化

---

### 5.2 点云分割器 (PointCloudSegmenter)

#### 支持的方法

```cpp
enum class SegmentMethod {
    EuclideanCluster,  // 欧氏聚类
    RegionGrowing,     // 区域生长
    RANSAC,            // RANSAC平面分割
    ColorBased         // 颜色分割
};
```

#### 核心接口

```cpp
class PointCloudSegmenter : public IAlgorithm
{
public:
    void setInputCloud(const QVector<QVector3D>& points);
    void setMethod(SegmentMethod method);
    void setClusterTolerance(double tol);
    void setMinClusterSize(int size);
    void setMaxClusterSize(int size);
    
    const SegmentationResult& result() const;
};
```

#### 扩展建议

- [ ] 添加DBSCAN聚类
- [ ] 支持多平面分割
- [ ] 实现超体素分割

---

### 5.3 深度学习检测器 (DeepLearningDetector)

#### 支持的网络

```cpp
enum class NetworkType {
    PointNet,
    PointNetPlusPlus,
    DGCNN,
    PointPillars,
    Custom
};
```

#### 核心接口

```cpp
class DeepLearningDetector : public IAlgorithm
{
public:
    bool loadModel(const QString& modelPath);
    void setNetworkType(NetworkType type);
    void setConfidenceThreshold(float threshold);
    void setClassNames(const QStringList& names);
    void setInputCloud(const QVector<QVector3D>& points);
    
    const InferenceResult& result() const;
    const NeuralNetworkInfo& modelInfo() const;
    bool isModelLoaded() const;
    bool runTestInference();  // 测试用
};
```

#### 扩展建议

- [ ] 集成ONNX Runtime
- [ ] 支持TensorRT加速
- [ ] 添加模型量化支持
- [ ] 实现批量推理

---

## 6. UI Widget集成

### 6.1 信号连接模式

```cpp
// 算法 -> UI 连接
connect(m_algorithm.get(), &IAlgorithm::progressUpdated, this, 
    [this](const AlgorithmProgress& p) {
        m_progressBar->setValue(p.current);
        m_statusLabel->setText(p.message);
    });

connect(m_algorithm.get(), &IAlgorithm::completed, this, 
    &MyWidget::onAlgorithmCompleted);

connect(m_algorithm.get(), &IAlgorithm::logMessage, this, 
    &MyWidget::appendLog);
```

### 6.2 状态管理

```cpp
void MyWidget::onStartAlgorithm()
{
    m_isProcessing = true;
    updateUIState();  // 禁用相关按钮
    
    m_algorithm->initialize();
    m_algorithm->execute();
}

void MyWidget::onAlgorithmCompleted(bool success)
{
    m_isProcessing = false;
    updateUIState();  // 恢复按钮状态
    
    if (success) {
        // 显示结果
    } else {
        // 显示错误
    }
}
```

---

## 7. 添加新算法指南

### 步骤1：定义数据类型

在 `AlgorithmTypes.h` 中添加：

```cpp
struct MyAlgorithmResult {
    // 结果字段
    bool isValid = false;
};
```

### 步骤2：创建算法类

在对应的头文件中：

```cpp
class MyAlgorithm : public IAlgorithm
{
    Q_OBJECT
public:
    explicit MyAlgorithm(QObject* parent = nullptr);
    
    QString name() const override { return tr("我的算法"); }
    QString description() const override { return tr("算法描述"); }
    
    bool initialize() override;
    bool execute() override;
    
    // 自定义接口...
    
private:
    MyAlgorithmResult m_result;
};
```

### 步骤3：实现算法

```cpp
bool MyAlgorithm::execute()
{
    m_running = true;
    reportProgress(0, 100, tr("开始"));
    
    // 算法实现...
    
    if (isCancelled()) {
        m_running = false;
        emit completed(false);
        return false;
    }
    
    m_result.isValid = true;
    reportProgress(100, 100, tr("完成"));
    m_running = false;
    emit completed(true);
    return true;
}

// 注册
REGISTER_ALGORITHM(MyAlgorithm)
```

### 步骤4：添加UI

在Widget中添加对应的控件和槽函数。

### 步骤5：更新CMakeLists.txt

确保新文件被添加到构建系统。

---

## 8. 依赖说明

### 必需依赖
- Qt6 (Core, Gui, Widgets)

### 可选依赖
- **Eigen3**: 矩阵运算（手眼标定优化）
- **ONNX Runtime**: 深度学习推理
- **PCL**: 点云处理算法加速
- **OpenCV**: 图像处理辅助

### 当前状态
所有算法当前使用简化实现，不依赖外部库。可以作为框架直接使用，后续可逐步集成专业库以提升性能。

---

## 9. 测试说明

每个模块都包含测试按钮，可在无硬件情况下验证功能：

| 模块 | 测试函数 | 功能 |
|------|----------|------|
| 测量不确定度 | `onTestUncertainty()` | 使用预设数据计算不确定度 |
| 运动学标定 | `onTestKinematic()` | 使用模拟标定点测试 |
| 手眼标定 | `onTestHandEye()` | 使用模拟姿态对测试 |
| 路径规划 | `onTestPathPlanning()` | 生成50x50mm测试路径 |
| 预处理 | `onTestPreprocess()` | 处理带噪声的测试点云 |
| 分割 | `onTestSegmentation()` | 分割多聚类测试点云 |
| 深度学习 | `onTestDetection()` | 模拟推理过程 |

---

## 10. 版本历史

| 版本 | 日期 | 变更 |
|------|------|------|
| 1.0.0 | 2024-XX-XX | 初始版本，包含基础框架和简化实现 |

---

## 11. 待完成事项

### 高优先级
- [ ] 实现完整的Tsai-Lenz手眼标定算法
- [ ] 集成ONNX Runtime进行真实推理
- [ ] 添加点云可视化预览

### 中优先级
- [ ] 实现DH参数优化
- [ ] 添加路径优化TSP求解
- [ ] 支持多种点云文件格式导入

### 低优先级
- [ ] 添加算法参数保存/加载
- [ ] 实现算法链式调用
- [ ] 添加性能基准测试
