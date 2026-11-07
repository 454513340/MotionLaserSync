#pragma once

/**
 * @file PointCloudProcessingAlgorithms.h
 * @brief 点云处理算法模块
 * 
 * 包含以下功能：
 * 1. 点云预处理（滤波、降采样、法向量估计）
 * 2. 点云特征提取
 * 3. 基于深度学习的目标检测和识别
 */

#include "IAlgorithm.h"
#include "AlgorithmTypes.h"
#include <QVector>
#include <QVector3D>

/**
 * @brief 点云预处理器
 */
class PointCloudPreprocessor : public IAlgorithm
{
    Q_OBJECT

public:
    enum class FilterType {
        StatisticalOutlier,  // 统计离群点滤波
        RadiusOutlier,       // 半径离群点滤波
        PassThrough,         // 直通滤波
        VoxelGrid            // 体素网格降采样
    };
    Q_ENUM(FilterType)

    explicit PointCloudPreprocessor(QObject* parent = nullptr);
    ~PointCloudPreprocessor() override = default;

    QString name() const override { return tr("点云预处理器"); }
    QString description() const override { return tr("点云滤波、降采样、法向量估计"); }

    bool initialize() override;
    bool execute() override;

    // 输入设置
    void setInputCloud(const QVector<QVector3D>& points);
    void setFilterType(FilterType type) { m_filterType = type; }
    
    // 滤波参数
    void setStatisticalParams(int k, double stdMul) { m_statK = k; m_statStdMul = stdMul; }
    void setRadiusParams(double radius, int minNeighbors) { m_radius = radius; m_minNeighbors = minNeighbors; }
    void setPassThroughParams(const QString& axis, double min, double max);
    void setVoxelSize(double size) { m_voxelSize = size; }

    // 结果获取
    const QVector<QVector3D>& outputCloud() const { return m_outputCloud; }
    const QVector<QVector3D>& normals() const { return m_normals; }
    int removedPointCount() const { return m_inputCloud.size() - m_outputCloud.size(); }

private:
    bool applyStatisticalFilter();
    bool applyRadiusFilter();
    bool applyPassThroughFilter();
    bool applyVoxelFilter();
    bool estimateNormals();

private:
    QVector<QVector3D> m_inputCloud;
    QVector<QVector3D> m_outputCloud;
    QVector<QVector3D> m_normals;
    
    FilterType m_filterType = FilterType::StatisticalOutlier;
    int m_statK = 50;
    double m_statStdMul = 1.0;
    double m_radius = 1.0;
    int m_minNeighbors = 5;
    QString m_passAxis = "Z";
    double m_passMin = 0, m_passMax = 100;
    double m_voxelSize = 1.0;
};

/**
 * @brief 点云特征提取器
 */
class PointCloudFeatureExtractor : public IAlgorithm
{
    Q_OBJECT

public:
    explicit PointCloudFeatureExtractor(QObject* parent = nullptr);
    ~PointCloudFeatureExtractor() override = default;

    QString name() const override { return tr("特征提取器"); }
    QString description() const override { return tr("提取点云几何和统计特征"); }

    bool initialize() override;
    bool execute() override;

    void setInputCloud(const QVector<QVector3D>& points);
    void setNormals(const QVector<QVector3D>& normals);
    
    const PointCloudFeatures& features() const { return m_features; }

private:
    void computeGlobalFeatures();
    void computeLocalFeatures();

private:
    QVector<QVector3D> m_inputCloud;
    QVector<QVector3D> m_normals;
    PointCloudFeatures m_features;
};

/**
 * @brief 点云分割器
 */
class PointCloudSegmenter : public IAlgorithm
{
    Q_OBJECT

public:
    enum class SegmentMethod {
        EuclideanCluster,    // 欧氏聚类
        RegionGrowing,       // 区域生长
        RANSAC,              // RANSAC平面分割
        ColorBased           // 基于颜色的分割
    };
    Q_ENUM(SegmentMethod)

    explicit PointCloudSegmenter(QObject* parent = nullptr);
    ~PointCloudSegmenter() override = default;

    QString name() const override { return tr("点云分割器"); }
    QString description() const override { return tr("点云聚类和分割"); }

    bool initialize() override;
    bool execute() override;

    void setInputCloud(const QVector<QVector3D>& points);
    void setMethod(SegmentMethod method) { m_method = method; }
    void setClusterTolerance(double tol) { m_clusterTolerance = tol; }
    void setMinClusterSize(int size) { m_minClusterSize = size; }
    void setMaxClusterSize(int size) { m_maxClusterSize = size; }

    const SegmentationResult& result() const { return m_result; }

private:
    bool euclideanClustering();
    bool regionGrowing();
    bool ransacSegmentation();

private:
    QVector<QVector3D> m_inputCloud;
    SegmentMethod m_method = SegmentMethod::EuclideanCluster;
    double m_clusterTolerance = 2.0;
    int m_minClusterSize = 100;
    int m_maxClusterSize = 100000;
    SegmentationResult m_result;
};

/**
 * @brief 深度学习点云检测器
 * 
 * 支持的网络架构：
 * - PointNet / PointNet++
 * - DGCNN
 * - PointPillars
 */
class DeepLearningDetector : public IAlgorithm
{
    Q_OBJECT

public:
    enum class NetworkType {
        PointNet,
        PointNetPlusPlus,
        DGCNN,
        PointPillars,
        Custom
    };
    Q_ENUM(NetworkType)

    explicit DeepLearningDetector(QObject* parent = nullptr);
    ~DeepLearningDetector() override = default;

    QString name() const override { return tr("深度学习检测器"); }
    QString description() const override { return tr("基于神经网络的点云目标检测"); }

    bool initialize() override;
    bool execute() override;

    // 模型设置
    bool loadModel(const QString& modelPath);
    void setNetworkType(NetworkType type) { m_networkType = type; }
    void setConfidenceThreshold(float threshold) { m_confidenceThreshold = threshold; }
    void setClassNames(const QStringList& names) { m_classNames = names; }

    // 输入设置
    void setInputCloud(const QVector<QVector3D>& points);
    void setInputFeatures(const QVector<float>& features);

    // 结果获取
    const InferenceResult& result() const { return m_result; }
    const NeuralNetworkInfo& modelInfo() const { return m_modelInfo; }
    bool isModelLoaded() const { return m_modelInfo.isLoaded; }

    // 测试函数
    bool runTestInference();

private:
    bool preprocessInput();
    bool runInference();
    bool postprocessOutput();
    
    // 模拟推理（用于测试）
    bool simulateInference();

private:
    QVector<QVector3D> m_inputCloud;
    QVector<float> m_inputFeatures;
    QVector<float> m_preprocessedData;
    
    NetworkType m_networkType = NetworkType::PointNet;
    NeuralNetworkInfo m_modelInfo;
    float m_confidenceThreshold = 0.5f;
    QStringList m_classNames;
    
    InferenceResult m_result;
};

/**
 * @brief 点云配准器
 */
class PointCloudRegistration : public IAlgorithm
{
    Q_OBJECT

public:
    enum class Method {
        ICP,             // 迭代最近点
        NDT,             // 正态分布变换
        FeatureBased,    // 基于特征的配准
        GlobalRegistration
    };
    Q_ENUM(Method)

    explicit PointCloudRegistration(QObject* parent = nullptr);
    ~PointCloudRegistration() override = default;

    QString name() const override { return tr("点云配准"); }
    QString description() const override { return tr("点云对齐和融合"); }

    bool initialize() override;
    bool execute() override;

    void setSourceCloud(const QVector<QVector3D>& points);
    void setTargetCloud(const QVector<QVector3D>& points);
    void setMethod(Method method) { m_method = method; }
    void setMaxIterations(int maxIter) { m_maxIterations = maxIter; }
    void setTransformationEpsilon(double eps) { m_transformEpsilon = eps; }

    QMatrix4x4 transformationMatrix() const { return m_transformation; }
    QVector<QVector3D> alignedCloud() const;
    double fitnessScore() const { return m_fitnessScore; }

private:
    bool runICP();
    bool runNDT();

private:
    QVector<QVector3D> m_sourceCloud;
    QVector<QVector3D> m_targetCloud;
    Method m_method = Method::ICP;
    int m_maxIterations = 50;
    double m_transformEpsilon = 1e-6;
    
    QMatrix4x4 m_transformation;
    double m_fitnessScore = 0.0;
};
