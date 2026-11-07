#include "algorithms/PointCloudProcessingAlgorithms.h"
#include <QtMath>
#include <algorithm>
#include <random>
#include <QDateTime>
#include <QElapsedTimer>
#include <QCoreApplication>

/**
 * @file PointCloudProcessingAlgorithms.cpp
 * @brief 点云处理算法实现
 */

// ==================== PointCloudPreprocessor ====================

PointCloudPreprocessor::PointCloudPreprocessor(QObject* parent) : IAlgorithm(parent) {}

bool PointCloudPreprocessor::initialize()
{
    m_cancelled = false;
    m_running = false;
    m_outputCloud.clear();
    m_normals.clear();
    return true;
}

bool PointCloudPreprocessor::execute()
{
    m_running = true;
    log(tr("开始点云预处理..."));
    reportProgress(0, 100, tr("初始化"));
    
    if (m_inputCloud.isEmpty()) {
        setError(tr("输入点云为空"));
        m_running = false;
        emit completed(false);
        return false;
    }
    
    bool success = false;
    reportProgress(20, 100, tr("应用滤波器"));
    
    switch (m_filterType) {
    case FilterType::StatisticalOutlier:
        success = applyStatisticalFilter();
        break;
    case FilterType::RadiusOutlier:
        success = applyRadiusFilter();
        break;
    case FilterType::PassThrough:
        success = applyPassThroughFilter();
        break;
    case FilterType::VoxelGrid:
        success = applyVoxelFilter();
        break;
    }
    
    if (!success) {
        m_running = false;
        emit completed(false);
        return false;
    }
    
    reportProgress(70, 100, tr("估计法向量"));
    estimateNormals();
    
    log(tr("预处理完成: 输入 %1 点, 输出 %2 点").arg(m_inputCloud.size()).arg(m_outputCloud.size()));
    reportProgress(100, 100, tr("完成"));
    m_running = false;
    emit completed(true);
    return true;
}

void PointCloudPreprocessor::setInputCloud(const QVector<QVector3D>& points)
{
    m_inputCloud = points;
}

void PointCloudPreprocessor::setPassThroughParams(const QString& axis, double min, double max)
{
    m_passAxis = axis;
    m_passMin = min;
    m_passMax = max;
}

bool PointCloudPreprocessor::applyStatisticalFilter()
{
    // 统计离群点滤波简化实现
    m_outputCloud.clear();
    
    for (int i = 0; i < m_inputCloud.size(); ++i) {
        if (isCancelled()) return false;
        
        const QVector3D& pt = m_inputCloud[i];
        
        // 计算到K近邻的平均距离
        QVector<double> distances;
        for (int j = 0; j < m_inputCloud.size(); ++j) {
            if (i != j) {
                distances.append((pt - m_inputCloud[j]).length());
            }
        }
        std::sort(distances.begin(), distances.end());
        
        double meanDist = 0;
        int k = qMin(m_statK, distances.size());
        for (int n = 0; n < k; ++n) {
            meanDist += distances[n];
        }
        meanDist /= k;
        
        // 简化：使用固定阈值
        if (meanDist < m_statStdMul * 10.0) {  // 阈值 = stdMul * 10mm
            m_outputCloud.append(pt);
        }
        
        if (i % 1000 == 0) {
            reportProgress(20 + 50 * i / m_inputCloud.size(), 100, tr("滤波处理中..."));
        }
    }
    
    return true;
}

bool PointCloudPreprocessor::applyRadiusFilter()
{
    m_outputCloud.clear();
    
    for (int i = 0; i < m_inputCloud.size(); ++i) {
        if (isCancelled()) return false;
        
        const QVector3D& pt = m_inputCloud[i];
        int neighborCount = 0;
        
        for (const auto& other : m_inputCloud) {
            if ((pt - other).length() < m_radius) {
                neighborCount++;
            }
        }
        
        if (neighborCount >= m_minNeighbors) {
            m_outputCloud.append(pt);
        }
    }
    
    return true;
}

bool PointCloudPreprocessor::applyPassThroughFilter()
{
    m_outputCloud.clear();
    
    for (const auto& pt : m_inputCloud) {
        double value;
        if (m_passAxis == "X") value = pt.x();
        else if (m_passAxis == "Y") value = pt.y();
        else value = pt.z();
        
        if (value >= m_passMin && value <= m_passMax) {
            m_outputCloud.append(pt);
        }
    }
    
    return true;
}

bool PointCloudPreprocessor::applyVoxelFilter()
{
    // 体素网格降采样
    m_outputCloud.clear();
    
    QMap<QString, QVector3D> voxelMap;
    QMap<QString, int> countMap;
    
    for (const auto& pt : m_inputCloud) {
        int vx = static_cast<int>(pt.x() / m_voxelSize);
        int vy = static_cast<int>(pt.y() / m_voxelSize);
        int vz = static_cast<int>(pt.z() / m_voxelSize);
        QString key = QString("%1_%2_%3").arg(vx).arg(vy).arg(vz);
        
        if (voxelMap.contains(key)) {
            voxelMap[key] += pt;
            countMap[key]++;
        } else {
            voxelMap[key] = pt;
            countMap[key] = 1;
        }
    }
    
    for (auto it = voxelMap.begin(); it != voxelMap.end(); ++it) {
        m_outputCloud.append(it.value() / countMap[it.key()]);
    }
    
    return true;
}

bool PointCloudPreprocessor::estimateNormals()
{
    // 简化的法向量估计
    m_normals.clear();
    m_normals.reserve(m_outputCloud.size());
    
    for (int i = 0; i < m_outputCloud.size(); ++i) {
        // 简化：假设法向量向上
        m_normals.append(QVector3D(0, 0, 1));
    }
    
    return true;
}

// ==================== PointCloudFeatureExtractor ====================

PointCloudFeatureExtractor::PointCloudFeatureExtractor(QObject* parent) : IAlgorithm(parent) {}

bool PointCloudFeatureExtractor::initialize()
{
    m_cancelled = false;
    m_running = false;
    m_features = PointCloudFeatures();
    return true;
}

bool PointCloudFeatureExtractor::execute()
{
    m_running = true;
    log(tr("开始特征提取..."));
    
    if (m_inputCloud.isEmpty()) {
        setError(tr("输入点云为空"));
        m_running = false;
        emit completed(false);
        return false;
    }
    
    reportProgress(30, 100, tr("计算全局特征"));
    computeGlobalFeatures();
    
    reportProgress(70, 100, tr("计算局部特征"));
    computeLocalFeatures();
    
    m_features.isValid = true;
    m_features.pointCount = m_inputCloud.size();
    
    log(tr("特征提取完成"));
    reportProgress(100, 100, tr("完成"));
    m_running = false;
    emit completed(true);
    return true;
}

void PointCloudFeatureExtractor::setInputCloud(const QVector<QVector3D>& points)
{
    m_inputCloud = points;
}

void PointCloudFeatureExtractor::setNormals(const QVector<QVector3D>& normals)
{
    m_normals = normals;
}

void PointCloudFeatureExtractor::computeGlobalFeatures()
{
    // 计算质心
    QVector3D sum(0, 0, 0);
    for (const auto& pt : m_inputCloud) {
        sum += pt;
    }
    m_features.centroid = sum / m_inputCloud.size();
    
    // 计算边界框体积
    BoundingBox bbox;
    bbox.compute(m_inputCloud);
    m_features.volume = bbox.size.x() * bbox.size.y() * bbox.size.z();
    
    // 估算点密度
    m_features.density = m_inputCloud.size() / m_features.volume;
}

void PointCloudFeatureExtractor::computeLocalFeatures()
{
    // 简化实现
    m_features.curvatures.resize(m_inputCloud.size());
    m_features.roughness.resize(m_inputCloud.size());
    
    for (int i = 0; i < m_inputCloud.size(); ++i) {
        m_features.curvatures[i] = 0.0;
        m_features.roughness[i] = 0.0;
    }
}

// ==================== PointCloudSegmenter ====================

PointCloudSegmenter::PointCloudSegmenter(QObject* parent) : IAlgorithm(parent) {}

bool PointCloudSegmenter::initialize()
{
    m_cancelled = false;
    m_running = false;
    m_result = SegmentationResult();
    return true;
}

bool PointCloudSegmenter::execute()
{
    m_running = true;
    log(tr("开始点云分割..."));
    
    if (m_inputCloud.isEmpty()) {
        setError(tr("输入点云为空"));
        m_running = false;
        emit completed(false);
        return false;
    }
    
    bool success = false;
    switch (m_method) {
    case SegmentMethod::EuclideanCluster:
        success = euclideanClustering();
        break;
    case SegmentMethod::RegionGrowing:
        success = regionGrowing();
        break;
    case SegmentMethod::RANSAC:
        success = ransacSegmentation();
        break;
    default:
        success = euclideanClustering();
        break;
    }
    
    log(tr("分割完成: %1 个聚类").arg(m_result.segmentCount));
    m_running = false;
    emit completed(success);
    return success;
}

void PointCloudSegmenter::setInputCloud(const QVector<QVector3D>& points)
{
    m_inputCloud = points;
}

bool PointCloudSegmenter::euclideanClustering()
{
    // 简化的欧氏聚类实现
    m_result.labels.resize(m_inputCloud.size());
    m_result.labels.fill(-1);
    
    int currentLabel = 0;
    
    for (int i = 0; i < m_inputCloud.size(); ++i) {
        if (m_result.labels[i] >= 0) continue;
        
        // 开始新聚类
        QVector<int> cluster;
        QVector<int> queue;
        queue.append(i);
        
        while (!queue.isEmpty()) {
            int idx = queue.takeFirst();
            if (m_result.labels[idx] >= 0) continue;
            
            m_result.labels[idx] = currentLabel;
            cluster.append(idx);
            
            // 查找邻居
            for (int j = 0; j < m_inputCloud.size(); ++j) {
                if (m_result.labels[j] < 0) {
                    double dist = (m_inputCloud[idx] - m_inputCloud[j]).length();
                    if (dist < m_clusterTolerance) {
                        queue.append(j);
                    }
                }
            }
        }
        
        // 检查聚类大小
        if (cluster.size() >= m_minClusterSize && cluster.size() <= m_maxClusterSize) {
            m_result.segments.append(cluster);
            
            // 计算质心
            QVector3D centroid(0, 0, 0);
            for (int idx : cluster) {
                centroid += m_inputCloud[idx];
            }
            m_result.centroids.append(centroid / cluster.size());
            
            currentLabel++;
        }
    }
    
    m_result.segmentCount = currentLabel;
    m_result.isValid = true;
    return true;
}

bool PointCloudSegmenter::regionGrowing()
{
    // 区域生长分割简化实现
    return euclideanClustering();
}

bool PointCloudSegmenter::ransacSegmentation()
{
    // RANSAC平面分割简化实现
    m_result.labels.resize(m_inputCloud.size());
    m_result.labels.fill(0);
    m_result.segmentCount = 1;
    m_result.isValid = true;
    return true;
}

// ==================== DeepLearningDetector ====================

DeepLearningDetector::DeepLearningDetector(QObject* parent) : IAlgorithm(parent)
{
    m_classNames << "Object" << "Plane" << "Cylinder" << "Sphere" << "Unknown";
}

bool DeepLearningDetector::initialize()
{
    m_cancelled = false;
    m_running = false;
    m_result = InferenceResult();
    m_preprocessedData.clear();
    return true;
}

bool DeepLearningDetector::execute()
{
    m_running = true;
    log(tr("开始深度学习推理..."));
    reportProgress(0, 100, tr("初始化"));
    
    if (m_inputCloud.isEmpty()) {
        setError(tr("输入点云为空"));
        m_running = false;
        emit completed(false);
        return false;
    }
    
    reportProgress(20, 100, tr("预处理输入"));
    if (!preprocessInput()) {
        m_running = false;
        emit completed(false);
        return false;
    }
    
    reportProgress(50, 100, tr("执行推理"));
    if (!runInference()) {
        m_running = false;
        emit completed(false);
        return false;
    }
    
    reportProgress(80, 100, tr("后处理输出"));
    if (!postprocessOutput()) {
        m_running = false;
        emit completed(false);
        return false;
    }
    
    log(tr("推理完成: 检测到 %1 个目标, 耗时 %2 ms")
        .arg(m_result.detections.size()).arg(m_result.inferenceTime, 0, 'f', 2));
    
    reportProgress(100, 100, tr("完成"));
    m_running = false;
    emit completed(true);
    return true;
}

bool DeepLearningDetector::loadModel(const QString& modelPath)
{
    // 模拟模型加载
    log(tr("加载模型: %1").arg(modelPath));
    
    m_modelInfo.modelPath = modelPath;
    m_modelInfo.modelName = "PointNet_v1";
    m_modelInfo.framework = "ONNX";
    m_modelInfo.classNames = m_classNames;
    m_modelInfo.inputSize = 1024;
    m_modelInfo.isLoaded = true;
    
    log(tr("模型加载成功"));
    return true;
}

void DeepLearningDetector::setInputCloud(const QVector<QVector3D>& points)
{
    m_inputCloud = points;
}

void DeepLearningDetector::setInputFeatures(const QVector<float>& features)
{
    m_inputFeatures = features;
}

bool DeepLearningDetector::preprocessInput()
{
    // 预处理：归一化、采样到固定大小
    int targetSize = m_modelInfo.inputSize;
    m_preprocessedData.clear();
    m_preprocessedData.reserve(targetSize * 3);
    
    // 计算边界框用于归一化
    BoundingBox bbox;
    bbox.compute(m_inputCloud);
    
    // 采样或填充到目标大小
    QVector<QVector3D> sampledPoints;
    if (m_inputCloud.size() >= targetSize) {
        // 随机下采样
        QVector<int> indices(m_inputCloud.size());
        for (int i = 0; i < m_inputCloud.size(); ++i) indices[i] = i;
        std::shuffle(indices.begin(), indices.end(), std::default_random_engine(42));
        for (int i = 0; i < targetSize; ++i) {
            sampledPoints.append(m_inputCloud[indices[i]]);
        }
    } else {
        // 重复采样
        sampledPoints = m_inputCloud;
        while (sampledPoints.size() < targetSize) {
            sampledPoints.append(m_inputCloud[sampledPoints.size() % m_inputCloud.size()]);
        }
    }
    
    // 归一化到 [-1, 1]
    for (const auto& pt : sampledPoints) {
        QVector3D normalized = (pt - bbox.center);
        float scale = qMax(qMax(bbox.size.x(), bbox.size.y()), bbox.size.z()) / 2.0f;
        if (scale > 0) normalized /= scale;
        
        m_preprocessedData.append(normalized.x());
        m_preprocessedData.append(normalized.y());
        m_preprocessedData.append(normalized.z());
    }
    
    return true;
}

bool DeepLearningDetector::runInference()
{
    // 实际应用中这里会调用 ONNX Runtime 或其他推理框架
    // 这里使用模拟推理
    return simulateInference();
}

bool DeepLearningDetector::simulateInference()
{
    // 模拟推理过程
    QDateTime startTime = QDateTime::currentDateTime();
    
    // 模拟计算延迟 (使用非阻塞方式)
    QElapsedTimer timer;
    timer.start();
    while (timer.elapsed() < 50) {
        QCoreApplication::processEvents();
    }
    
    // 生成模拟检测结果
    m_result.detections.clear();
    
    // 模拟检测到一些目标
    std::default_random_engine gen(42);
    std::uniform_real_distribution<float> confDist(0.3f, 0.99f);
    std::uniform_int_distribution<int> classDist(0, m_classNames.size() - 1);
    
    int numDetections = 2 + (m_inputCloud.size() / 10000);
    numDetections = qMin(numDetections, 10);
    
    BoundingBox inputBbox;
    inputBbox.compute(m_inputCloud);
    
    for (int i = 0; i < numDetections; ++i) {
        DetectionResult det;
        det.classId = classDist(gen);
        det.className = m_classNames[det.classId];
        det.confidence = confDist(gen);
        
        if (det.confidence < m_confidenceThreshold) continue;
        
        // 生成随机边界框
        float cx = inputBbox.center.x() + (i - numDetections/2) * inputBbox.size.x() / numDetections;
        float cy = inputBbox.center.y() + (i - numDetections/2) * inputBbox.size.y() / numDetections;
        float cz = inputBbox.center.z();
        
        det.bbox.center = QVector3D(cx, cy, cz);
        det.bbox.size = inputBbox.size / 3.0f;
        det.bbox.min = det.bbox.center - det.bbox.size / 2.0f;
        det.bbox.max = det.bbox.center + det.bbox.size / 2.0f;
        det.centroid = det.bbox.center;
        
        m_result.detections.append(det);
    }
    
    QDateTime endTime = QDateTime::currentDateTime();
    m_result.inferenceTime = startTime.msecsTo(endTime);
    
    return true;
}

bool DeepLearningDetector::postprocessOutput()
{
    // 后处理：NMS、过滤低置信度结果
    QVector<DetectionResult> filtered;
    
    for (const auto& det : m_result.detections) {
        if (det.confidence >= m_confidenceThreshold) {
            filtered.append(det);
        }
    }
    
    // 按置信度排序
    std::sort(filtered.begin(), filtered.end(), 
              [](const DetectionResult& a, const DetectionResult& b) {
                  return a.confidence > b.confidence;
              });
    
    m_result.detections = filtered;
    m_result.isValid = true;
    return true;
}

bool DeepLearningDetector::runTestInference()
{
    // 生成测试点云
    m_inputCloud.clear();
    std::default_random_engine gen(42);
    std::normal_distribution<float> dist(0, 10);
    
    for (int i = 0; i < 1000; ++i) {
        m_inputCloud.append(QVector3D(dist(gen), dist(gen), dist(gen)));
    }
    
    log(tr("运行测试推理: %1 个测试点").arg(m_inputCloud.size()));
    return execute();
}

// ==================== PointCloudRegistration ====================

PointCloudRegistration::PointCloudRegistration(QObject* parent) : IAlgorithm(parent)
{
    m_transformation.setToIdentity();
}

bool PointCloudRegistration::initialize()
{
    m_cancelled = false;
    m_running = false;
    m_transformation.setToIdentity();
    m_fitnessScore = 0;
    return true;
}

bool PointCloudRegistration::execute()
{
    m_running = true;
    log(tr("开始点云配准..."));
    
    if (m_sourceCloud.isEmpty() || m_targetCloud.isEmpty()) {
        setError(tr("源点云或目标点云为空"));
        m_running = false;
        emit completed(false);
        return false;
    }
    
    bool success = false;
    switch (m_method) {
    case Method::ICP:
        success = runICP();
        break;
    case Method::NDT:
        success = runNDT();
        break;
    default:
        success = runICP();
        break;
    }
    
    log(tr("配准完成: 适应度分数 = %1").arg(m_fitnessScore, 0, 'f', 4));
    m_running = false;
    emit completed(success);
    return success;
}

void PointCloudRegistration::setSourceCloud(const QVector<QVector3D>& points)
{
    m_sourceCloud = points;
}

void PointCloudRegistration::setTargetCloud(const QVector<QVector3D>& points)
{
    m_targetCloud = points;
}

bool PointCloudRegistration::runICP()
{
    // 简化的ICP实现
    m_transformation.setToIdentity();
    
    // 计算质心
    QVector3D srcCentroid(0, 0, 0), tgtCentroid(0, 0, 0);
    for (const auto& pt : m_sourceCloud) srcCentroid += pt;
    for (const auto& pt : m_targetCloud) tgtCentroid += pt;
    srcCentroid /= m_sourceCloud.size();
    tgtCentroid /= m_targetCloud.size();
    
    // 简化：仅计算平移
    QVector3D translation = tgtCentroid - srcCentroid;
    m_transformation.translate(translation);
    
    // 计算适应度分数
    double totalDist = 0;
    for (const auto& srcPt : m_sourceCloud) {
        QVector3D transformed = m_transformation.map(srcPt);
        double minDist = std::numeric_limits<double>::max();
        for (const auto& tgtPt : m_targetCloud) {
            double dist = (transformed - tgtPt).length();
            minDist = qMin(minDist, dist);
        }
        totalDist += minDist;
    }
    m_fitnessScore = totalDist / m_sourceCloud.size();
    
    return true;
}

bool PointCloudRegistration::runNDT()
{
    // NDT简化实现
    return runICP();
}

QVector<QVector3D> PointCloudRegistration::alignedCloud() const
{
    QVector<QVector3D> result;
    result.reserve(m_sourceCloud.size());
    for (const auto& pt : m_sourceCloud) {
        result.append(m_transformation.map(pt));
    }
    return result;
}

// 注册算法
REGISTER_ALGORITHM(PointCloudPreprocessor)
REGISTER_ALGORITHM(PointCloudFeatureExtractor)
REGISTER_ALGORITHM(PointCloudSegmenter)
REGISTER_ALGORITHM(DeepLearningDetector)
REGISTER_ALGORITHM(PointCloudRegistration)
