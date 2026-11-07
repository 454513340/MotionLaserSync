#pragma once

/**
 * @file AlgorithmTypes.h
 * @brief 算法模块通用数据类型定义
 * 
 * 定义系统标定、路径规划、点云处理等算法模块所需的数据结构
 */

#include <QVector>
#include <QVector3D>
#include <QMatrix4x4>
#include <QString>
#include <QMetaType>

// ==================== 标定相关数据类型 ====================

/**
 * @brief 测量不确定度评估结果
 */
struct UncertaintyResult {
    double repeatability = 0.0;      // 重复性 (mm)
    double reproducibility = 0.0;    // 再现性 (mm)
    double linearity = 0.0;          // 线性度误差 (mm)
    double resolution = 0.0;         // 分辨率 (mm)
    double combinedUncertainty = 0.0;// 合成不确定度 (mm)
    double expandedUncertainty = 0.0;// 扩展不确定度 k=2 (mm)
    double coverageFactor = 2.0;     // 包含因子
    QString report;                  // 详细报告
    bool isValid = false;
};

/**
 * @brief 运动学参数标定结果
 */
struct KinematicCalibResult {
    // DH 参数偏差
    QVector<double> deltaA;      // 连杆长度偏差
    QVector<double> deltaAlpha;  // 扭转角偏差
    QVector<double> deltaD;      // 偏置偏差
    QVector<double> deltaTheta;  // 关节角偏差
    
    // 校正后的变换矩阵
    QMatrix4x4 correctionMatrix;
    
    // 统计信息
    double rmse = 0.0;           // 均方根误差 (mm)
    double maxError = 0.0;       // 最大误差 (mm)
    int iterations = 0;          // 迭代次数
    QString report;
    bool isValid = false;
};

/**
 * @brief 手眼标定结果
 */
struct HandEyeCalibResult {
    // 传感器到法兰的变换 (Eye-to-Hand)
    QMatrix4x4 sensorToFlange;
    
    // 标定精度
    double rotationError = 0.0;   // 旋转误差 (度)
    double translationError = 0.0;// 平移误差 (mm)
    double reprojectionError = 0.0;
    
    // 标定点对数量
    int poseCount = 0;
    QString report;
    bool isValid = false;
};

/**
 * @brief 标定板姿态
 */
struct CalibPose {
    QVector3D position;          // 位置 (mm)
    QVector3D rotation;          // 旋转欧拉角 RPY (度)
    QMatrix4x4 transformMatrix;  // 4x4变换矩阵
    qint64 timestamp = 0;        // 时间戳 (us)
};

/**
 * @brief 标定数据点
 */
struct CalibDataPoint {
    CalibPose robotPose;         // 机器人/机床末端姿态
    QVector<QVector3D> sensorPoints;  // 传感器观测点
    double temperature = 20.0;   // 环境温度 (°C)
    bool isValid = true;
};

// ==================== 路径规划相关数据类型 ====================

/**
 * @brief 扫描路径类型
 */
enum class ScanPathType {
    Raster,         // 栅格扫描 (之字形)
    Spiral,         // 螺旋扫描
    Contour,        // 轮廓跟踪
    Adaptive,       // 自适应扫描
    Custom          // 自定义路径
};

/**
 * @brief 扫描方向
 */
enum class ScanDirection {
    XPositive,
    XNegative,
    YPositive,
    YNegative,
    Bidirectional
};

/**
 * @brief 路径规划参数
 */
struct PathPlanParams {
    ScanPathType pathType = ScanPathType::Raster;
    ScanDirection direction = ScanDirection::XPositive;
    
    double lineSpacing = 1.0;    // 行间距 (mm)
    double pointSpacing = 0.5;   // 点间距 (mm)
    double scanSpeed = 10.0;     // 扫描速度 (mm/s)
    double rapidSpeed = 50.0;    // 快速移动速度 (mm/s)
    double safeHeight = 10.0;    // 安全高度 (mm)
    double overlapRatio = 0.1;   // 重叠率 (0-1)
    
    // 边界设置
    double marginX = 5.0;        // X方向边距 (mm)
    double marginY = 5.0;        // Y方向边距 (mm)
    
    // 高度自适应
    bool adaptiveHeight = false; // 是否启用自适应高度
    double nominalHeight = 95.0; // 标称高度 (mm) - 传感器量程中心
    double heightTolerance = 20.0;// 高度容差 (mm)
};

/**
 * @brief 路径点
 */
struct ScanPathPoint {
    QVector3D position;          // 目标位置 (mm)
    double c = 0.0;              // C轴目标角度 (deg)，P4 五轴轨迹正式启用
    double b = 0.0;              // B轴目标角度 (deg)，P4 五轴轨迹正式启用
    bool hasRotaryPose = false;  // false 表示当前仍是旧三轴路径点
    double feedRate = 10.0;      // 进给速度 (mm/s)
    bool rapidMove = false;      // 是否快速移动
    bool laserOn = true;         // 是否开启扫描
    int lineIndex = 0;           // 所在行号
    QString comment;             // 注释
};

/**
 * @brief 扫描路径
 */
struct ScanPath {
    QVector<ScanPathPoint> points;
    PathPlanParams params;
    
    // 统计信息
    double totalLength = 0.0;    // 总路径长度 (mm)
    double scanLength = 0.0;     // 扫描长度 (mm)
    double rapidLength = 0.0;    // 快移长度 (mm)
    double estimatedTime = 0.0;  // 预估时间 (s)
    int lineCount = 0;           // 扫描行数
    
    QString gcode;               // 生成的G代码
    bool isValid = false;
};

/**
 * @brief 点云边界框
 */
struct BoundingBox {
    QVector3D min;               // 最小点
    QVector3D max;               // 最大点
    QVector3D center;            // 中心点
    QVector3D size;              // 尺寸
    
    void compute(const QVector<QVector3D>& points);
};

// ==================== 点云处理相关数据类型 ====================

/**
 * @brief 目标检测结果
 */
struct DetectionResult {
    int classId = -1;            // 类别ID
    QString className;           // 类别名称
    float confidence = 0.0f;     // 置信度 (0-1)
    BoundingBox bbox;            // 3D边界框
    QVector<int> pointIndices;   // 属于该目标的点索引
    QVector3D centroid;          // 质心
    QString additionalInfo;      // 附加信息
};

/**
 * @brief 点云分割结果
 */
struct SegmentationResult {
    QVector<int> labels;         // 每个点的标签
    int segmentCount = 0;        // 分割数量
    QVector<QVector<int>> segments; // 每个分割包含的点索引
    QVector<QVector3D> centroids;   // 每个分割的质心
    bool isValid = false;
};

/**
 * @brief 点云特征
 */
struct PointCloudFeatures {
    // 全局特征
    QVector3D centroid;          // 质心
    double volume = 0.0;         // 体积估算
    double surfaceArea = 0.0;    // 表面积估算
    
    // 局部特征
    QVector<double> normals;     // 法向量 (Nx3 展平)
    QVector<double> curvatures;  // 曲率
    QVector<double> roughness;   // 粗糙度
    
    // 统计特征
    double density = 0.0;        // 点密度
    int pointCount = 0;
    bool isValid = false;
};

/**
 * @brief 神经网络模型信息
 */
struct NeuralNetworkInfo {
    QString modelPath;           // 模型路径
    QString modelName;           // 模型名称
    QString framework;           // 框架 (ONNX, TensorRT, etc.)
    QStringList classNames;      // 类别名称列表
    int inputSize = 1024;        // 输入点数
    bool isLoaded = false;
};

/**
 * @brief 推理结果
 */
struct InferenceResult {
    QVector<DetectionResult> detections;  // 检测结果
    SegmentationResult segmentation;      // 分割结果
    double inferenceTime = 0.0;           // 推理时间 (ms)
    bool isValid = false;
};

// ==================== 算法基类接口 ====================

/**
 * @brief 算法进度回调
 */
struct AlgorithmProgress {
    int current = 0;             // 当前步骤
    int total = 100;             // 总步骤
    QString message;             // 进度消息
    double percentage() const { return total > 0 ? (current * 100.0 / total) : 0.0; }
};

/**
 * @brief 算法结果基类
 */
struct AlgorithmResult {
    bool success = false;
    QString errorMessage;
    double computeTime = 0.0;    // 计算时间 (ms)
};

// ==================== 元类型注册 ====================
Q_DECLARE_METATYPE(UncertaintyResult)
Q_DECLARE_METATYPE(KinematicCalibResult)
Q_DECLARE_METATYPE(HandEyeCalibResult)
Q_DECLARE_METATYPE(CalibPose)
Q_DECLARE_METATYPE(CalibDataPoint)
Q_DECLARE_METATYPE(PathPlanParams)
Q_DECLARE_METATYPE(ScanPathPoint)
Q_DECLARE_METATYPE(ScanPath)
Q_DECLARE_METATYPE(DetectionResult)
Q_DECLARE_METATYPE(SegmentationResult)
Q_DECLARE_METATYPE(InferenceResult)
Q_DECLARE_METATYPE(AlgorithmProgress)
