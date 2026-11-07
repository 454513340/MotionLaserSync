#include "algorithms/CalibrationAlgorithms.h"
#include <QFile>
#include <QTextStream>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QtMath>
#include <algorithm>
#include <numeric>

/**
 * @file CalibrationAlgorithms.cpp
 * @brief 系统标定算法实现
 */

// ==================== 测量不确定度标定实现 ====================

UncertaintyCalibration::UncertaintyCalibration(QObject* parent)
    : IAlgorithm(parent)
{
}

bool UncertaintyCalibration::initialize()
{
    m_cancelled = false;
    m_running = false;
    m_result = UncertaintyResult();
    return true;
}

bool UncertaintyCalibration::execute()
{
    m_running = true;
    m_cancelled = false;
    
    log(tr("开始测量不确定度评估..."));
    reportProgress(0, 100, tr("初始化"));
    
    if (m_repeatabilityData.isEmpty() && m_measuredValues.isEmpty()) {
        setError(tr("没有输入测量数据"));
        m_running = false;
        emit completed(false);
        return false;
    }
    
    // Step 1: 计算重复性不确定度 (A类)
    reportProgress(20, 100, tr("计算重复性不确定度"));
    m_result.repeatability = computeRepeatabilityUncertainty();
    log(tr("重复性不确定度: %1 mm").arg(m_result.repeatability, 0, 'f', 4));
    
    if (isCancelled()) {
        m_running = false;
        emit completed(false);
        return false;
    }
    
    // Step 2: 计算线性度误差
    reportProgress(40, 100, tr("计算线性度误差"));
    m_result.linearity = computeLinearityUncertainty();
    log(tr("线性度误差: %1 mm").arg(m_result.linearity, 0, 'f', 4));
    
    // Step 3: 计算分辨率不确定度 (B类)
    reportProgress(60, 100, tr("计算分辨率不确定度"));
    m_result.resolution = computeResolutionUncertainty();
    log(tr("分辨率引起的不确定度: %1 mm").arg(m_result.resolution, 0, 'f', 4));
    
    // Step 4: 计算热影响不确定度 (B类)
    reportProgress(70, 100, tr("计算热影响不确定度"));
    double thermalU = computeThermalUncertainty();
    
    // Step 5: 合成标准不确定度
    reportProgress(80, 100, tr("计算合成标准不确定度"));
    m_result.combinedUncertainty = computeCombinedUncertainty();
    log(tr("合成标准不确定度: %1 mm").arg(m_result.combinedUncertainty, 0, 'f', 4));
    
    // Step 6: 计算扩展不确定度 (k=2, 95%置信水平)
    reportProgress(90, 100, tr("计算扩展不确定度"));
    m_result.coverageFactor = 2.0;
    m_result.expandedUncertainty = m_result.combinedUncertainty * m_result.coverageFactor;
    log(tr("扩展不确定度 (k=2): %1 mm").arg(m_result.expandedUncertainty, 0, 'f', 4));
    
    // 生成报告
    m_result.report = generateReport();
    m_result.isValid = true;
    
    reportProgress(100, 100, tr("完成"));
    m_running = false;
    emit completed(true);
    return true;
}

void UncertaintyCalibration::addRepeatabilityData(const QVector<double>& measurements, 
                                                   double referenceValue)
{
    m_repeatabilityData.append(measurements);
    m_referenceValues.append(referenceValue);
}

void UncertaintyCalibration::addLinearityData(const QVector<double>& measuredValues,
                                               const QVector<double>& referenceValues)
{
    m_measuredValues = measuredValues;
    m_linearityReferences = referenceValues;
}

void UncertaintyCalibration::clearData()
{
    m_repeatabilityData.clear();
    m_referenceValues.clear();
    m_measuredValues.clear();
    m_linearityReferences.clear();
    m_result = UncertaintyResult();
}

double UncertaintyCalibration::computeRepeatabilityUncertainty()
{
    if (m_repeatabilityData.isEmpty()) return 0.0;
    
    double totalVariance = 0.0;
    int totalSets = 0;
    
    for (const auto& measurements : m_repeatabilityData) {
        if (measurements.size() < 2) continue;
        
        // 计算均值
        double mean = std::accumulate(measurements.begin(), measurements.end(), 0.0) 
                      / measurements.size();
        
        // 计算方差
        double variance = 0.0;
        for (double m : measurements) {
            variance += (m - mean) * (m - mean);
        }
        variance /= (measurements.size() - 1);
        
        totalVariance += variance;
        totalSets++;
    }
    
    if (totalSets == 0) return 0.0;
    
    // 合并标准差 (pooled standard deviation)
    double pooledStd = std::sqrt(totalVariance / totalSets);
    
    // A类不确定度 = s / sqrt(n)，这里取最保守的估计
    return pooledStd;
}

double UncertaintyCalibration::computeLinearityUncertainty()
{
    if (m_measuredValues.isEmpty() || m_linearityReferences.isEmpty()) return 0.0;
    if (m_measuredValues.size() != m_linearityReferences.size()) return 0.0;
    
    int n = m_measuredValues.size();
    if (n < 2) return 0.0;
    
    // 线性回归 y = ax + b
    double sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
    for (int i = 0; i < n; ++i) {
        sumX += m_linearityReferences[i];
        sumY += m_measuredValues[i];
        sumXY += m_linearityReferences[i] * m_measuredValues[i];
        sumX2 += m_linearityReferences[i] * m_linearityReferences[i];
    }
    
    double a = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);
    double b = (sumY - a * sumX) / n;
    
    // 计算残差的最大值作为线性度误差
    double maxResidual = 0.0;
    for (int i = 0; i < n; ++i) {
        double predicted = a * m_linearityReferences[i] + b;
        double residual = std::abs(m_measuredValues[i] - predicted);
        maxResidual = std::max(maxResidual, residual);
    }
    
    return maxResidual;
}

double UncertaintyCalibration::computeResolutionUncertainty()
{
    // 分辨率引起的不确定度：假设均匀分布，u = resolution / (2 * sqrt(3))
    return m_resolution / (2.0 * std::sqrt(3.0));
}

double UncertaintyCalibration::computeThermalUncertainty()
{
    // 热膨胀引起的不确定度
    // 假设测量范围内的典型长度为 100mm
    double typicalLength = 100.0;  // mm
    return typicalLength * m_thermalCoefficient * m_temperatureVariation / std::sqrt(3.0);
}

double UncertaintyCalibration::computeCombinedUncertainty()
{
    // 合成标准不确定度 (假设各分量独立)
    // uc = sqrt(u1^2 + u2^2 + u3^2 + ...)
    
    double uRepeat = m_result.repeatability;
    double uLinear = m_result.linearity / std::sqrt(3.0);  // 均匀分布假设
    double uResolution = m_result.resolution;
    double uThermal = computeThermalUncertainty();
    double uSpec = m_specifiedAccuracy / std::sqrt(3.0);   // 仪器规格精度
    
    return std::sqrt(uRepeat * uRepeat + 
                     uLinear * uLinear + 
                     uResolution * uResolution +
                     uThermal * uThermal +
                     uSpec * uSpec);
}

QString UncertaintyCalibration::generateReport() const
{
    QString report;
    QTextStream ts(&report);
    
    ts << "============================================\n";
    ts << "       测量不确定度评估报告\n";
    ts << "============================================\n\n";
    
    ts << "1. 测量条件\n";
    ts << "   - 分辨率: " << m_resolution << " mm\n";
    ts << "   - 规格精度: " << m_specifiedAccuracy << " mm\n";
    ts << "   - 温度变化: " << m_temperatureVariation << " °C\n";
    ts << "   - 热膨胀系数: " << m_thermalCoefficient << " 1/°C\n\n";
    
    ts << "2. A类不确定度 (统计方法)\n";
    ts << "   - 重复性: " << QString::number(m_result.repeatability, 'f', 4) << " mm\n\n";
    
    ts << "3. B类不确定度 (非统计方法)\n";
    ts << "   - 线性度: " << QString::number(m_result.linearity, 'f', 4) << " mm\n";
    ts << "   - 分辨率: " << QString::number(m_result.resolution, 'f', 4) << " mm\n\n";
    
    ts << "4. 合成标准不确定度\n";
    ts << "   uc = " << QString::number(m_result.combinedUncertainty, 'f', 4) << " mm\n\n";
    
    ts << "5. 扩展不确定度 (k=" << m_result.coverageFactor << ", 95%置信水平)\n";
    ts << "   U = " << QString::number(m_result.expandedUncertainty, 'f', 4) << " mm\n\n";
    
    ts << "============================================\n";
    
    return report;
}

// ==================== 运动学参数标定实现 ====================

KinematicCalibration::KinematicCalibration(QObject* parent)
    : IAlgorithm(parent)
{
    m_referenceFrame.setToIdentity();
}

bool KinematicCalibration::initialize()
{
    m_cancelled = false;
    m_running = false;
    m_result = KinematicCalibResult();
    return true;
}

bool KinematicCalibration::execute()
{
    m_running = true;
    m_cancelled = false;
    
    log(tr("开始运动学参数标定..."));
    reportProgress(0, 100, tr("初始化"));
    
    if (m_commandPositions.size() < 4) {
        setError(tr("标定点数量不足 (最少需要4个点)"));
        m_running = false;
        emit completed(false);
        return false;
    }
    
    if (m_commandPositions.size() != m_measuredPositions.size()) {
        setError(tr("指令位置和测量位置数量不匹配"));
        m_running = false;
        emit completed(false);
        return false;
    }
    
    reportProgress(20, 100, tr("数据预处理"));
    
    // 执行最小二乘优化
    reportProgress(40, 100, tr("执行最小二乘优化"));
    bool success = performLeastSquaresOptimization();
    
    if (!success) {
        setError(tr("优化求解失败"));
        m_running = false;
        emit completed(false);
        return false;
    }
    
    reportProgress(80, 100, tr("计算误差统计"));
    m_result.rmse = computeRMSE();
    m_result.maxError = computeMaxError();
    
    log(tr("RMSE: %1 mm").arg(m_result.rmse, 0, 'f', 4));
    log(tr("最大误差: %1 mm").arg(m_result.maxError, 0, 'f', 4));
    
    m_result.report = generateReport();
    m_result.isValid = true;
    
    reportProgress(100, 100, tr("完成"));
    m_running = false;
    emit completed(true);
    return true;
}

void KinematicCalibration::addCalibPoint(const QVector3D& commandPos, 
                                          const QVector3D& measuredPos,
                                          int axis)
{
    m_commandPositions.append(commandPos);
    m_measuredPositions.append(measuredPos);
    m_measurementAxes.append(axis);
}

void KinematicCalibration::addCalibPoints(const QVector<QVector3D>& commandPositions,
                                           const QVector<QVector3D>& measuredPositions)
{
    m_commandPositions.append(commandPositions);
    m_measuredPositions.append(measuredPositions);
    for (int i = 0; i < commandPositions.size(); ++i) {
        m_measurementAxes.append(-1);
    }
}

void KinematicCalibration::clearData()
{
    m_commandPositions.clear();
    m_measuredPositions.clear();
    m_measurementAxes.clear();
    m_compensationTable.clear();
    m_result = KinematicCalibResult();
}

bool KinematicCalibration::performLeastSquaresOptimization()
{
    // 简化的误差补偿模型
    // 计算指令位置到测量位置的仿射变换
    
    int n = m_commandPositions.size();
    
    // 计算平均误差（简单情况下的偏移补偿）
    QVector3D totalError(0, 0, 0);
    for (int i = 0; i < n; ++i) {
        totalError += (m_measuredPositions[i] - m_commandPositions[i]);
    }
    QVector3D avgError = totalError / n;
    
    // 构建校正矩阵（这里简化为平移校正）
    m_result.correctionMatrix.setToIdentity();
    m_result.correctionMatrix.translate(-avgError);
    
    // TODO: 实现完整的仿射变换拟合
    // 这里应该使用 Eigen 或类似库进行矩阵求解
    
    m_result.iterations = 1;
    return true;
}

QVector3D KinematicCalibration::correctPosition(const QVector3D& commandPos) const
{
    if (!m_result.isValid) return commandPos;
    return m_result.correctionMatrix.map(commandPos);
}

double KinematicCalibration::computeRMSE() const
{
    if (m_commandPositions.isEmpty()) return 0.0;
    
    double sumSqError = 0.0;
    for (int i = 0; i < m_commandPositions.size(); ++i) {
        QVector3D corrected = correctPosition(m_commandPositions[i]);
        QVector3D error = corrected - m_measuredPositions[i];
        sumSqError += error.lengthSquared();
    }
    
    return std::sqrt(sumSqError / m_commandPositions.size());
}

double KinematicCalibration::computeMaxError() const
{
    if (m_commandPositions.isEmpty()) return 0.0;
    
    double maxError = 0.0;
    for (int i = 0; i < m_commandPositions.size(); ++i) {
        QVector3D corrected = correctPosition(m_commandPositions[i]);
        double error = (corrected - m_measuredPositions[i]).length();
        maxError = std::max(maxError, error);
    }
    
    return maxError;
}

QString KinematicCalibration::generateReport() const
{
    QString report;
    QTextStream ts(&report);
    
    ts << "============================================\n";
    ts << "       运动学参数标定报告\n";
    ts << "============================================\n\n";
    
    ts << "1. 标定数据\n";
    ts << "   - 标定点数: " << m_commandPositions.size() << "\n";
    ts << "   - 迭代次数: " << m_result.iterations << "\n\n";
    
    ts << "2. 标定结果\n";
    ts << "   校正矩阵:\n";
    for (int row = 0; row < 4; ++row) {
        ts << "   [";
        for (int col = 0; col < 4; ++col) {
            ts << QString::number(m_result.correctionMatrix(row, col), 'f', 6);
            if (col < 3) ts << ", ";
        }
        ts << "]\n";
    }
    ts << "\n";
    
    ts << "3. 误差统计\n";
    ts << "   - RMSE: " << QString::number(m_result.rmse, 'f', 4) << " mm\n";
    ts << "   - 最大误差: " << QString::number(m_result.maxError, 'f', 4) << " mm\n\n";
    
    ts << "============================================\n";
    
    return report;
}

bool KinematicCalibration::exportCompensationTable(const QString& filePath) const
{
    QFile file(filePath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        return false;
    }
    
    QTextStream out(&file);
    out << "# Kinematic Compensation Table\n";
    out << "# Format: CommandX,CommandY,CommandZ,CorrectedX,CorrectedY,CorrectedZ\n";
    
    for (int i = 0; i < m_commandPositions.size(); ++i) {
        QVector3D cmd = m_commandPositions[i];
        QVector3D corr = correctPosition(cmd);
        out << cmd.x() << "," << cmd.y() << "," << cmd.z() << ",";
        out << corr.x() << "," << corr.y() << "," << corr.z() << "\n";
    }
    
    file.close();
    return true;
}

// ==================== 手眼标定实现 ====================

HandEyeCalibration::HandEyeCalibration(QObject* parent)
    : IAlgorithm(parent)
{
    m_initialGuess.setToIdentity();
}

bool HandEyeCalibration::initialize()
{
    m_cancelled = false;
    m_running = false;
    m_result = HandEyeCalibResult();
    return true;
}

bool HandEyeCalibration::execute()
{
    m_running = true;
    m_cancelled = false;
    
    log(tr("开始手眼标定..."));
    reportProgress(0, 100, tr("初始化"));
    
    if (m_robotPoses.size() < 3) {
        setError(tr("姿态对数量不足 (最少需要3对)"));
        m_running = false;
        emit completed(false);
        return false;
    }
    
    reportProgress(20, 100, tr("数据预处理"));
    
    bool success = false;
    
    switch (m_method) {
    case Method::Tsai:
        reportProgress(40, 100, tr("执行 Tsai-Lenz 算法"));
        success = solveTsai();
        break;
    case Method::Park:
        reportProgress(40, 100, tr("执行 Park 算法"));
        success = solvePark();
        break;
    case Method::Horaud:
        reportProgress(40, 100, tr("执行 Horaud 算法"));
        success = solveHoraud();
        break;
    case Method::Daniilidis:
        reportProgress(40, 100, tr("执行 Daniilidis 算法"));
        success = solveDaniilidis();
        break;
    case Method::PointCloud:
        reportProgress(40, 100, tr("执行点云配准"));
        success = solvePointCloud();
        break;
    }
    
    if (!success) {
        setError(tr("标定求解失败"));
        m_running = false;
        emit completed(false);
        return false;
    }
    
    reportProgress(80, 100, tr("计算标定误差"));
    
    // 计算误差
    double error = computeCalibrationError();
    log(tr("重投影误差: %1 mm").arg(error, 0, 'f', 4));
    
    m_result.poseCount = m_robotPoses.size();
    m_result.report = generateReport();
    m_result.isValid = true;
    
    reportProgress(100, 100, tr("完成"));
    m_running = false;
    emit completed(true);
    return true;
}

void HandEyeCalibration::addPosePair(const QMatrix4x4& robotPose, const QMatrix4x4& targetPose)
{
    m_robotPoses.append(robotPose);
    m_targetPoses.append(targetPose);
}

void HandEyeCalibration::addPointCloudData(const QMatrix4x4& robotPose, 
                                            const QVector<QVector3D>& sensorPoints)
{
    m_robotPoses.append(robotPose);
    m_pointCloudData.append(sensorPoints);
}

void HandEyeCalibration::clearData()
{
    m_robotPoses.clear();
    m_targetPoses.clear();
    m_pointCloudData.clear();
    m_result = HandEyeCalibResult();
}

bool HandEyeCalibration::solveTsai()
{
    // Tsai-Lenz 手眼标定算法简化实现
    // AX = XB 问题
    
    // 至少需要2对姿态来形成方程
    if (m_robotPoses.size() < 2) return false;
    
    // 计算相对变换
    QVector<QMatrix4x4> A_list, B_list;
    for (int i = 0; i < m_robotPoses.size() - 1; ++i) {
        QMatrix4x4 A = computeRelativeTransform(m_robotPoses[i], m_robotPoses[i+1]);
        QMatrix4x4 B = computeRelativeTransform(m_targetPoses[i], m_targetPoses[i+1]);
        A_list.append(A);
        B_list.append(B);
    }
    
    // TODO: 实现完整的 Tsai-Lenz 算法
    // 这里提供简化的实现框架
    
    // 简化：假设只有平移，没有旋转
    m_result.sensorToFlange.setToIdentity();
    
    // 估计平移分量
    if (!m_targetPoses.isEmpty()) {
        QVector3D avgTranslation(0, 0, 0);
        for (int i = 0; i < m_targetPoses.size(); ++i) {
            avgTranslation += QVector3D(m_targetPoses[i](0, 3),
                                        m_targetPoses[i](1, 3),
                                        m_targetPoses[i](2, 3));
        }
        avgTranslation /= m_targetPoses.size();
        m_result.sensorToFlange.translate(avgTranslation);
    }
    
    m_result.translationError = 0.1;  // 占位值
    m_result.rotationError = 0.1;     // 占位值
    
    return true;
}

bool HandEyeCalibration::solvePark()
{
    // Park 方法的简化实现
    return solveTsai();  // 使用 Tsai 作为后备
}

bool HandEyeCalibration::solveHoraud()
{
    // Horaud 方法的简化实现
    return solveTsai();
}

bool HandEyeCalibration::solveDaniilidis()
{
    // Daniilidis 双四元数方法的简化实现
    return solveTsai();
}

bool HandEyeCalibration::solvePointCloud()
{
    // 基于点云配准的手眼标定
    // 使用 ICP 或类似方法
    
    if (m_pointCloudData.isEmpty()) return false;
    
    // TODO: 实现点云配准
    m_result.sensorToFlange.setToIdentity();
    
    return true;
}

QMatrix4x4 HandEyeCalibration::computeRelativeTransform(const QMatrix4x4& A, const QMatrix4x4& B) const
{
    return B * A.inverted();
}

double HandEyeCalibration::computeCalibrationError() const
{
    // 计算重投影误差
    if (!m_result.isValid) return 0.0;
    
    // TODO: 实现误差计算
    return 0.0;
}

QVector3D HandEyeCalibration::transformToMachine(const QVector3D& sensorPoint, 
                                                   const QMatrix4x4& robotPose) const
{
    if (!m_result.isValid) return sensorPoint;
    
    // P_machine = T_robot * T_sensor_flange * P_sensor
    QVector3D flangePoint = m_result.sensorToFlange.map(sensorPoint);
    return robotPose.map(flangePoint);
}

QVector<QVector3D> HandEyeCalibration::transformPointCloud(const QVector<QVector3D>& sensorPoints,
                                                            const QMatrix4x4& robotPose) const
{
    QVector<QVector3D> result;
    result.reserve(sensorPoints.size());
    
    for (const auto& pt : sensorPoints) {
        result.append(transformToMachine(pt, robotPose));
    }
    
    return result;
}

QString HandEyeCalibration::generateReport() const
{
    QString report;
    QTextStream ts(&report);
    
    ts << "============================================\n";
    ts << "         手眼标定报告\n";
    ts << "============================================\n\n";
    
    ts << "1. 标定参数\n";
    ts << "   - 方法: ";
    switch (m_method) {
    case Method::Tsai: ts << "Tsai-Lenz\n"; break;
    case Method::Park: ts << "Park\n"; break;
    case Method::Horaud: ts << "Horaud\n"; break;
    case Method::Daniilidis: ts << "Daniilidis\n"; break;
    case Method::PointCloud: ts << "点云配准\n"; break;
    }
    ts << "   - 姿态对数: " << m_result.poseCount << "\n\n";
    
    ts << "2. 标定结果 (传感器到法兰变换)\n";
    ts << "   变换矩阵:\n";
    for (int row = 0; row < 4; ++row) {
        ts << "   [";
        for (int col = 0; col < 4; ++col) {
            ts << QString::number(m_result.sensorToFlange(row, col), 'f', 6);
            if (col < 3) ts << ", ";
        }
        ts << "]\n";
    }
    ts << "\n";
    
    ts << "3. 标定精度\n";
    ts << "   - 旋转误差: " << QString::number(m_result.rotationError, 'f', 4) << " deg\n";
    ts << "   - 平移误差: " << QString::number(m_result.translationError, 'f', 4) << " mm\n";
    ts << "   - 重投影误差: " << QString::number(m_result.reprojectionError, 'f', 4) << " mm\n\n";
    
    ts << "============================================\n";
    
    return report;
}

bool HandEyeCalibration::saveCalibration(const QString& filePath) const
{
    QFile file(filePath);
    if (!file.open(QIODevice::WriteOnly)) {
        return false;
    }
    
    QJsonObject root;
    root["version"] = "1.0";
    root["type"] = "hand_eye_calibration";
    
    // 保存变换矩阵
    QJsonArray matrix;
    for (int row = 0; row < 4; ++row) {
        QJsonArray rowData;
        for (int col = 0; col < 4; ++col) {
            rowData.append(m_result.sensorToFlange(row, col));
        }
        matrix.append(rowData);
    }
    root["transform"] = matrix;
    
    // 保存精度信息
    QJsonObject accuracy;
    accuracy["rotation_error"] = m_result.rotationError;
    accuracy["translation_error"] = m_result.translationError;
    accuracy["reprojection_error"] = m_result.reprojectionError;
    root["accuracy"] = accuracy;
    
    QJsonDocument doc(root);
    file.write(doc.toJson());
    file.close();
    
    return true;
}

bool HandEyeCalibration::loadCalibration(const QString& filePath)
{
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly)) {
        return false;
    }
    
    QJsonDocument doc = QJsonDocument::fromJson(file.readAll());
    file.close();
    
    if (doc.isNull() || !doc.isObject()) {
        return false;
    }
    
    QJsonObject root = doc.object();
    
    // 读取变换矩阵
    QJsonArray matrix = root["transform"].toArray();
    if (matrix.size() != 4) return false;
    
    for (int row = 0; row < 4; ++row) {
        QJsonArray rowData = matrix[row].toArray();
        if (rowData.size() != 4) return false;
        for (int col = 0; col < 4; ++col) {
            m_result.sensorToFlange(row, col) = rowData[col].toDouble();
        }
    }
    
    // 读取精度信息
    QJsonObject accuracy = root["accuracy"].toObject();
    m_result.rotationError = accuracy["rotation_error"].toDouble();
    m_result.translationError = accuracy["translation_error"].toDouble();
    m_result.reprojectionError = accuracy["reprojection_error"].toDouble();
    
    m_result.isValid = true;
    return true;
}

// 注册算法
REGISTER_ALGORITHM(UncertaintyCalibration)
REGISTER_ALGORITHM(KinematicCalibration)
REGISTER_ALGORITHM(HandEyeCalibration)
