#pragma once

/**
 * @file CalibrationAlgorithms.h
 * @brief 系统标定算法模块
 * 
 * 包含以下标定功能：
 * 1. 传感器测量不确定度标定
 * 2. 机床运动学参数标定
 * 3. 手眼关系标定
 */

#include "IAlgorithm.h"
#include "AlgorithmTypes.h"
#include <QVector>
#include <QVector3D>
#include <QMatrix4x4>

// ==================== 测量不确定度标定 ====================

/**
 * @brief 测量不确定度标定算法
 * 
 * 按照 GUM (Guide to the Expression of Uncertainty in Measurement) 标准
 * 评估传感器的测量不确定度
 * 
 * 评估内容：
 * - A类不确定度：统计方法评估 (重复性测量)
 * - B类不确定度：非统计方法评估 (仪器精度、环境影响等)
 * - 合成标准不确定度
 * - 扩展不确定度 (k=2, 95%置信水平)
 */
class UncertaintyCalibration : public IAlgorithm
{
    Q_OBJECT

public:
    explicit UncertaintyCalibration(QObject* parent = nullptr);
    ~UncertaintyCalibration() override = default;

    QString name() const override { return tr("测量不确定度标定"); }
    QString description() const override { 
        return tr("按照GUM标准评估传感器测量不确定度"); 
    }

    bool initialize() override;
    bool execute() override;

    // ===== 数据输入 =====
    
    /**
     * @brief 添加重复性测量数据
     * @param measurements 多次测量同一位置的数据
     * @param referenceValue 参考值（如有）
     */
    void addRepeatabilityData(const QVector<double>& measurements, 
                               double referenceValue = 0.0);

    /**
     * @brief 添加线性度测量数据
     * @param measuredValues 测量值
     * @param referenceValues 对应的参考值
     */
    void addLinearityData(const QVector<double>& measuredValues,
                          const QVector<double>& referenceValues);

    /**
     * @brief 设置仪器分辨率
     */
    void setResolution(double resolution) { m_resolution = resolution; }

    /**
     * @brief 设置仪器规格精度
     */
    void setSpecifiedAccuracy(double accuracy) { m_specifiedAccuracy = accuracy; }

    /**
     * @brief 设置环境温度变化
     */
    void setTemperatureVariation(double deltaT) { m_temperatureVariation = deltaT; }

    /**
     * @brief 设置热膨胀系数
     */
    void setThermalCoefficient(double coeff) { m_thermalCoefficient = coeff; }

    // ===== 结果获取 =====
    
    /**
     * @brief 获取标定结果
     */
    const UncertaintyResult& result() const { return m_result; }

    /**
     * @brief 生成标定报告
     */
    QString generateReport() const;

    /**
     * @brief 清空所有数据
     */
    void clearData();

private:
    // 计算各项不确定度分量
    double computeRepeatabilityUncertainty();
    double computeLinearityUncertainty();
    double computeResolutionUncertainty();
    double computeThermalUncertainty();
    double computeCombinedUncertainty();

private:
    // 输入数据
    QVector<QVector<double>> m_repeatabilityData;
    QVector<double> m_referenceValues;
    QVector<double> m_measuredValues;
    QVector<double> m_linearityReferences;
    
    // 仪器参数
    double m_resolution = 0.001;           // 分辨率 (mm)
    double m_specifiedAccuracy = 0.01;     // 规格精度 (mm)
    double m_temperatureVariation = 5.0;   // 温度变化 (°C)
    double m_thermalCoefficient = 11.7e-6; // 热膨胀系数 (1/°C)
    
    // 计算结果
    UncertaintyResult m_result;
};

// ==================== 运动学参数标定 ====================

/**
 * @brief 机床运动学参数标定算法
 * 
 * 基于激光位移传感器测量数据，标定机床的运动学参数误差
 * 
 * 标定内容：
 * - 轴向定位误差
 * - 直线度误差
 * - 垂直度误差
 * - 俯仰/偏摆角度误差
 */
class KinematicCalibration : public IAlgorithm
{
    Q_OBJECT

public:
    explicit KinematicCalibration(QObject* parent = nullptr);
    ~KinematicCalibration() override = default;

    QString name() const override { return tr("运动学参数标定"); }
    QString description() const override { 
        return tr("标定机床运动学参数误差"); 
    }

    bool initialize() override;
    bool execute() override;

    // ===== 数据输入 =====
    
    /**
     * @brief 添加标定数据点
     * @param commandPos 指令位置 (mm)
     * @param measuredPos 测量位置 (mm)
     * @param axis 测量轴 (0=X, 1=Y, 2=Z)
     */
    void addCalibPoint(const QVector3D& commandPos, 
                       const QVector3D& measuredPos,
                       int axis = -1);

    /**
     * @brief 添加批量标定数据
     */
    void addCalibPoints(const QVector<QVector3D>& commandPositions,
                        const QVector<QVector3D>& measuredPositions);

    /**
     * @brief 设置参考坐标系
     */
    void setReferenceFrame(const QMatrix4x4& frame) { m_referenceFrame = frame; }

    // ===== 参数设置 =====
    
    /**
     * @brief 设置最大迭代次数
     */
    void setMaxIterations(int maxIter) { m_maxIterations = maxIter; }

    /**
     * @brief 设置收敛阈值
     */
    void setConvergenceThreshold(double threshold) { m_convergenceThreshold = threshold; }

    // ===== 结果获取 =====
    
    /**
     * @brief 获取标定结果
     */
    const KinematicCalibResult& result() const { return m_result; }

    /**
     * @brief 获取校正后的位置
     */
    QVector3D correctPosition(const QVector3D& commandPos) const;

    /**
     * @brief 生成标定报告
     */
    QString generateReport() const;

    /**
     * @brief 导出误差补偿表
     */
    bool exportCompensationTable(const QString& filePath) const;

    /**
     * @brief 清空数据
     */
    void clearData();

private:
    // 最小二乘优化
    bool performLeastSquaresOptimization();
    
    // 计算误差
    double computeRMSE() const;
    double computeMaxError() const;

private:
    // 输入数据
    QVector<QVector3D> m_commandPositions;
    QVector<QVector3D> m_measuredPositions;
    QVector<int> m_measurementAxes;
    
    // 参考坐标系
    QMatrix4x4 m_referenceFrame;
    
    // 优化参数
    int m_maxIterations = 100;
    double m_convergenceThreshold = 1e-6;
    
    // 结果
    KinematicCalibResult m_result;
    
    // 补偿查找表
    QVector<QVector3D> m_compensationTable;
};

// ==================== 手眼标定 ====================

/**
 * @brief 手眼标定算法
 * 
 * 标定传感器相对于机床法兰（末端）的位姿关系
 * 支持两种标定方法：
 * 1. AX=XB 问题求解（经典手眼标定）
 * 2. 基于点云配准的标定
 */
class HandEyeCalibration : public IAlgorithm
{
    Q_OBJECT

public:
    /**
     * @brief 标定方法枚举
     */
    enum class Method {
        Tsai,           // Tsai-Lenz 方法
        Park,           // Park 方法
        Horaud,         // Horaud 方法
        Daniilidis,     // Daniilidis 方法 (同时估计旋转和平移)
        PointCloud      // 基于点云配准
    };
    Q_ENUM(Method)

    explicit HandEyeCalibration(QObject* parent = nullptr);
    ~HandEyeCalibration() override = default;

    QString name() const override { return tr("手眼关系标定"); }
    QString description() const override { 
        return tr("标定传感器与机床末端的位姿关系"); 
    }

    bool initialize() override;
    bool execute() override;

    // ===== 数据输入 =====
    
    /**
     * @brief 添加标定姿态对
     * @param robotPose 机器人末端姿态 (4x4变换矩阵)
     * @param targetPose 目标在传感器坐标系下的姿态
     */
    void addPosePair(const QMatrix4x4& robotPose, const QMatrix4x4& targetPose);

    /**
     * @brief 添加点云数据
     * @param robotPose 采集时的机器人姿态
     * @param sensorPoints 传感器测量的点云
     */
    void addPointCloudData(const QMatrix4x4& robotPose, 
                           const QVector<QVector3D>& sensorPoints);

    // ===== 参数设置 =====
    
    /**
     * @brief 设置标定方法
     */
    void setMethod(Method method) { m_method = method; }
    Method method() const { return m_method; }

    /**
     * @brief 设置初始估计（可选，用于迭代优化）
     */
    void setInitialGuess(const QMatrix4x4& guess) { m_initialGuess = guess; m_hasInitialGuess = true; }

    // ===== 结果获取 =====
    
    /**
     * @brief 获取标定结果
     */
    const HandEyeCalibResult& result() const { return m_result; }

    /**
     * @brief 将传感器坐标变换到机床坐标系
     */
    QVector3D transformToMachine(const QVector3D& sensorPoint, 
                                  const QMatrix4x4& robotPose) const;

    /**
     * @brief 批量变换点云
     */
    QVector<QVector3D> transformPointCloud(const QVector<QVector3D>& sensorPoints,
                                            const QMatrix4x4& robotPose) const;

    /**
     * @brief 生成标定报告
     */
    QString generateReport() const;

    /**
     * @brief 保存标定结果
     */
    bool saveCalibration(const QString& filePath) const;

    /**
     * @brief 加载标定结果
     */
    bool loadCalibration(const QString& filePath);

    /**
     * @brief 清空数据
     */
    void clearData();

    /**
     * @brief 获取姿态对数量
     */
    int poseCount() const { return m_robotPoses.size(); }

private:
    // 不同方法的求解
    bool solveTsai();
    bool solvePark();
    bool solveHoraud();
    bool solveDaniilidis();
    bool solvePointCloud();
    
    // 辅助函数
    QMatrix4x4 computeRelativeTransform(const QMatrix4x4& A, const QMatrix4x4& B) const;
    double computeCalibrationError() const;

private:
    // 标定方法
    Method m_method = Method::Tsai;
    
    // 输入数据
    QVector<QMatrix4x4> m_robotPoses;
    QVector<QMatrix4x4> m_targetPoses;
    QVector<QVector<QVector3D>> m_pointCloudData;
    
    // 初始估计
    QMatrix4x4 m_initialGuess;
    bool m_hasInitialGuess = false;
    
    // 结果
    HandEyeCalibResult m_result;
};

// ==================== 注册算法 ====================
// 在cpp文件中使用 REGISTER_ALGORITHM 宏注册
