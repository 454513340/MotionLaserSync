# 算法模块扩展架构说明

## 1. 总体架构

项目新增了三个独立的算法模块，与原有的"参数设置"和"控制"Tab并列：

```
右侧控制面板 Tab
├── 参数设置     (原有)
├── 控制         (原有)
├── 系统标定     (新增) - CalibrationWidget
├── 路径规划     (新增) - PathPlanningWidget
└── 点云处理     (新增) - PointCloudProcessWidget
```

## 2. 目录结构

```
include/
├── algorithms/                    # 算法模块头文件
│   ├── AlgorithmTypes.h          # 通用数据类型定义
│   ├── IAlgorithm.h              # 算法接口基类
│   ├── CalibrationAlgorithms.h   # 标定算法
│   ├── PathPlanningAlgorithms.h  # 路径规划算法
│   └── PointCloudProcessingAlgorithms.h  # 点云处理算法
├── widgets/
│   ├── CalibrationWidget.h       # 标定UI
│   ├── PathPlanningWidget.h      # 路径规划UI
│   └── PointCloudProcessWidget.h # 点云处理UI

src/
├── algorithms/                    # 算法实现
│   ├── CalibrationAlgorithms.cpp
│   ├── PathPlanningAlgorithms.cpp
│   └── PointCloudProcessingAlgorithms.cpp
├── widgets/
│   ├── CalibrationWidget.cpp
│   ├── PathPlanningWidget.cpp
│   └── PointCloudProcessWidget.cpp
```

## 3. 算法接口设计 (IAlgorithm)

所有算法继承统一的基类接口，便于扩展：

```cpp
class IAlgorithm : public QObject
{
    Q_OBJECT
public:
    virtual QString name() const = 0;
    virtual QString description() const = 0;
    virtual bool initialize() = 0;
    virtual bool execute() = 0;
    virtual void cancel();
    
signals:
    void progressUpdated(const AlgorithmProgress& progress);
    void completed(bool success);
    void errorOccurred(const QString& message);
    void logMessage(const QString& message);
};
```

### 添加新算法的步骤：

1. 创建继承 `IAlgorithm` 的新类
2. 实现 `name()`, `description()`, `initialize()`, `execute()` 方法
3. 在源文件末尾使用 `REGISTER_ALGORITHM(YourAlgorithm)` 注册
4. 在对应的Widget中添加UI和调用逻辑

## 4. 系统标定模块 (CalibrationWidget)

### 4.1 测量不确定度标定 (UncertaintyCalibration)
- 按照GUM标准评估传感器测量不确定度
- A类不确定度（统计方法）
- B类不确定度（非统计方法）
- 合成标准不确定度和扩展不确定度

### 4.2 运动学参数标定 (KinematicCalibration)
- 标定机床运动学参数误差
- 最小二乘优化
- 导出误差补偿表

### 4.3 手眼标定 (HandEyeCalibration)
- 支持多种方法：Tsai-Lenz, Park, Horaud, Daniilidis
- 标定传感器与机床末端位姿关系
- 保存/加载标定结果

## 5. 路径规划模块 (PathPlanningWidget)

### 5.1 路径规划器 (PathPlanner)
- 栅格扫描（Raster）
- 螺旋扫描（Spiral）
- 轮廓跟踪（Contour）
- 自适应扫描（Adaptive）

### 5.2 G代码生成器 (GCodeGenerator)
- 支持多种G代码风格
- 自动生成扫描控制指令
- 导出G代码文件

### 5.3 路径优化器 (PathOptimizer)
- 最小化时间
- 最小化距离
- 路径重排序

## 6. 点云处理模块 (PointCloudProcessWidget)

### 6.1 预处理 (PointCloudPreprocessor)
- 统计离群点滤波
- 半径离群点滤波
- 直通滤波
- 体素网格降采样
- 法向量估计

### 6.2 分割 (PointCloudSegmenter)
- 欧氏聚类
- 区域生长
- RANSAC平面分割

### 6.3 深度学习检测 (DeepLearningDetector)
- 支持PointNet/PointNet++/DGCNN网络
- ONNX模型加载
- 目标检测和识别
- 置信度过滤

## 7. 测试功能

每个模块都包含"测试"按钮，可以在没有实际硬件连接的情况下测试算法功能：

- **测量不确定度测试**: 使用预设测试数据运行标定
- **运动学标定测试**: 使用模拟标定点数据
- **手眼标定测试**: 使用模拟姿态对数据
- **路径规划测试**: 生成50x50mm测试区域路径
- **预处理测试**: 生成带离群点的测试点云
- **分割测试**: 生成多聚类测试点云
- **深度学习测试**: 模拟推理过程

## 8. 数据类型定义 (AlgorithmTypes.h)

定义了所有算法模块使用的数据结构：

```cpp
// 标定相关
struct UncertaintyResult;
struct KinematicCalibResult;
struct HandEyeCalibResult;
struct CalibPose;
struct CalibDataPoint;

// 路径规划相关
enum class ScanPathType;
enum class ScanDirection;
struct PathPlanParams;
struct ScanPathPoint;
struct ScanPath;
struct BoundingBox;

// 点云处理相关
struct DetectionResult;
struct SegmentationResult;
struct PointCloudFeatures;
struct NeuralNetworkInfo;
struct InferenceResult;

// 通用
struct AlgorithmProgress;
struct AlgorithmResult;
```

## 9. 后续扩展建议

### 9.1 添加新的标定算法
1. 在 `CalibrationAlgorithms.h` 中添加新类
2. 在 `CalibrationWidget` 中添加对应的Tab和UI

### 9.2 添加新的路径类型
1. 在 `ScanPathType` 枚举中添加新类型
2. 在 `PathPlanner::execute()` 中添加新的生成方法

### 9.3 集成真实的深度学习框架
1. 添加ONNX Runtime或TensorRT依赖
2. 在 `DeepLearningDetector::runInference()` 中实现实际推理逻辑
3. 替换 `simulateInference()` 的模拟实现

### 9.4 添加点云配准算法
1. 使用 `PointCloudRegistration` 类
2. 实现完整的ICP或NDT算法
3. 集成PCL库获得更好的性能

## 10. 依赖说明

新模块需要以下额外依赖（可选）：

- **Eigen**: 矩阵运算（手眼标定）
- **ONNX Runtime**: 深度学习推理
- **PCL**: 点云处理算法优化

当前实现提供了简化版本，不依赖这些外部库，可以作为框架直接使用。
