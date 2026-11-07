#pragma once

/**
 * @file PathPlanningAlgorithms.h
 * @brief 路径规划算法模块
 * 
 * 包含以下路径规划功能：
 * 1. 基于点云的扫描路径规划
 * 2. G代码生成
 * 3. 路径优化
 */

#include "IAlgorithm.h"
#include "AlgorithmTypes.h"
#include <QVector>
#include <QVector3D>

/**
 * @brief 基础路径规划器
 * 
 * 根据点云或边界生成扫描路径
 */
class PathPlanner : public IAlgorithm
{
    Q_OBJECT

public:
    explicit PathPlanner(QObject* parent = nullptr);
    ~PathPlanner() override = default;

    QString name() const override { return tr("路径规划器"); }
    QString description() const override { 
        return tr("根据点云或边界生成扫描路径"); 
    }

    bool initialize() override;
    bool execute() override;

    // ===== 输入设置 =====
    
    /**
     * @brief 设置点云数据
     * @param points 输入点云
     */
    void setPointCloud(const QVector<QVector3D>& points);

    /**
     * @brief 设置扫描边界
     * @param min 最小边界点
     * @param max 最大边界点
     */
    void setBoundary(const QVector3D& min, const QVector3D& max);

    /**
     * @brief 从点云自动计算边界
     */
    void computeBoundaryFromPointCloud();

    /**
     * @brief 设置规划参数
     */
    void setParameters(const PathPlanParams& params) { m_params = params; }
    const PathPlanParams& parameters() const { return m_params; }

    // ===== 路径类型设置 =====
    
    /**
     * @brief 设置扫描路径类型
     */
    void setPathType(ScanPathType type) { m_params.pathType = type; }

    /**
     * @brief 设置行间距
     */
    void setLineSpacing(double spacing) { m_params.lineSpacing = spacing; }

    /**
     * @brief 设置点间距
     */
    void setPointSpacing(double spacing) { m_params.pointSpacing = spacing; }

    /**
     * @brief 设置扫描速度
     */
    void setScanSpeed(double speed) { m_params.scanSpeed = speed; }

    /**
     * @brief 设置快速移动速度
     */
    void setRapidSpeed(double speed) { m_params.rapidSpeed = speed; }

    /**
     * @brief 设置安全高度
     */
    void setSafeHeight(double height) { m_params.safeHeight = height; }

    // ===== 结果获取 =====
    
    /**
     * @brief 获取生成的路径
     */
    const ScanPath& path() const { return m_path; }

    /**
     * @brief 获取路径点
     */
    const QVector<ScanPathPoint>& pathPoints() const { return m_path.points; }

    /**
     * @brief 获取边界框
     */
    const BoundingBox& boundingBox() const { return m_boundingBox; }

    /**
     * @brief 预览路径（返回轨迹点用于可视化）
     */
    QVector<QVector3D> previewPath() const;

private:
    // 不同路径类型的生成方法
    bool generateRasterPath();
    bool generateSpiralPath();
    bool generateContourPath();
    bool generateAdaptivePath();
    
    // 辅助函数
    void computePathStatistics();
    void addPathPoint(const QVector3D& pos, bool rapid = false, bool laserOn = true);

private:
    // 输入数据
    QVector<QVector3D> m_pointCloud;
    BoundingBox m_boundingBox;
    bool m_hasBoundary = false;
    
    // 参数
    PathPlanParams m_params;
    
    // 输出
    ScanPath m_path;
};

/**
 * @brief G代码生成器
 * 
 * 将扫描路径转换为G代码
 */
class GCodeGenerator : public IAlgorithm
{
    Q_OBJECT

public:
    /**
     * @brief G代码风格
     */
    enum class GCodeStyle {
        Standard,       // 标准G代码
        Fanuc,          // FANUC 风格
        Siemens,        // SIEMENS 风格
        ZMC             // 正运动控制器风格
    };
    Q_ENUM(GCodeStyle)

    explicit GCodeGenerator(QObject* parent = nullptr);
    ~GCodeGenerator() override = default;

    QString name() const override { return tr("G代码生成器"); }
    QString description() const override { 
        return tr("将扫描路径转换为G代码"); 
    }

    bool initialize() override;
    bool execute() override;

    // ===== 输入设置 =====
    
    /**
     * @brief 设置扫描路径
     */
    void setPath(const ScanPath& path) { m_path = path; }

    /**
     * @brief 设置路径点
     */
    void setPathPoints(const QVector<ScanPathPoint>& points);

    /**
     * @brief 设置G代码风格
     */
    void setStyle(GCodeStyle style) { m_style = style; }

    /**
     * @brief 设置程序号（FANUC风格）
     */
    void setProgramNumber(int num) { m_programNumber = num; }

    /**
     * @brief 设置工件坐标系
     */
    void setWorkCoordinate(int coord) { m_workCoordinate = coord; }

    /**
     * @brief 设置是否生成扫描控制指令
     */
    void setGenerateScanCommands(bool enable) { m_generateScanCommands = enable; }

    /**
     * @brief 设置曝光时间（用于M指令）
     */
    void setExposureTime(int exposureUs) { m_exposureTime = exposureUs; }

    // ===== 结果获取 =====
    
    /**
     * @brief 获取生成的G代码
     */
    QString gcode() const { return m_gcode; }

    /**
     * @brief 保存到文件
     */
    bool saveToFile(const QString& filePath) const;

    /**
     * @brief 获取预估加工时间
     */
    double estimatedTime() const { return m_estimatedTime; }

    /**
     * @brief 获取总行数
     */
    int lineCount() const { return m_lineCount; }

private:
    // 生成各部分代码
    QString generateHeader();
    QString generateBody();
    QString generateFooter();
    QString generateScanStart();
    QString generateScanStop();
    
    // 生成单个运动指令
    QString formatMove(const ScanPathPoint& point, bool isFirst = false);

private:
    // 输入
    ScanPath m_path;
    
    // 设置
    GCodeStyle m_style = GCodeStyle::Standard;
    int m_programNumber = 1;
    int m_workCoordinate = 54;  // G54
    bool m_generateScanCommands = true;
    int m_exposureTime = 1000;  // us
    
    // 输出
    QString m_gcode;
    double m_estimatedTime = 0.0;
    int m_lineCount = 0;
};

/**
 * @brief 路径优化器
 * 
 * 优化扫描路径以减少扫描时间
 */
class PathOptimizer : public IAlgorithm
{
    Q_OBJECT

public:
    /**
     * @brief 优化目标
     */
    enum class OptimizationGoal {
        MinimizeTime,       // 最小化时间
        MinimizeDistance,   // 最小化距离
        MinimizeTurns,      // 最小化转弯次数
        Balanced            // 平衡优化
    };
    Q_ENUM(OptimizationGoal)

    explicit PathOptimizer(QObject* parent = nullptr);
    ~PathOptimizer() override = default;

    QString name() const override { return tr("路径优化器"); }
    QString description() const override { 
        return tr("优化扫描路径以减少扫描时间"); 
    }

    bool initialize() override;
    bool execute() override;

    // ===== 输入设置 =====
    
    /**
     * @brief 设置原始路径
     */
    void setPath(const ScanPath& path) { m_originalPath = path; }

    /**
     * @brief 设置优化目标
     */
    void setOptimizationGoal(OptimizationGoal goal) { m_goal = goal; }

    /**
     * @brief 设置最大迭代次数
     */
    void setMaxIterations(int maxIter) { m_maxIterations = maxIter; }

    // ===== 结果获取 =====
    
    /**
     * @brief 获取优化后的路径
     */
    const ScanPath& optimizedPath() const { return m_optimizedPath; }

    /**
     * @brief 获取优化改进百分比
     */
    double improvementPercentage() const;

private:
    // 优化算法
    bool optimizeForTime();
    bool optimizeForDistance();
    bool reorderSegments();
    bool smoothPath();

private:
    // 输入
    ScanPath m_originalPath;
    
    // 设置
    OptimizationGoal m_goal = OptimizationGoal::MinimizeTime;
    int m_maxIterations = 100;
    
    // 输出
    ScanPath m_optimizedPath;
};

/**
 * @brief 自适应路径规划器
 * 
 * 根据点云密度和曲率自动调整路径参数
 */
class AdaptivePathPlanner : public IAlgorithm
{
    Q_OBJECT

public:
    explicit AdaptivePathPlanner(QObject* parent = nullptr);
    ~AdaptivePathPlanner() override = default;

    QString name() const override { return tr("自适应路径规划器"); }
    QString description() const override { 
        return tr("根据点云特征自动调整路径参数"); 
    }

    bool initialize() override;
    bool execute() override;

    // ===== 输入设置 =====
    
    /**
     * @brief 设置参考点云
     */
    void setReferencePointCloud(const QVector<QVector3D>& points);

    /**
     * @brief 设置基础参数
     */
    void setBaseParameters(const PathPlanParams& params) { m_baseParams = params; }

    /**
     * @brief 设置密度自适应参数
     */
    void setDensityAdaptation(double minDensity, double maxDensity) {
        m_minDensity = minDensity;
        m_maxDensity = maxDensity;
    }

    // ===== 结果获取 =====
    
    /**
     * @brief 获取生成的路径
     */
    const ScanPath& path() const { return m_path; }

    /**
     * @brief 获取局部参数映射
     */
    const QVector<PathPlanParams>& localParameters() const { return m_localParams; }

private:
    // 分析点云特征
    void analyzePointCloud();
    
    // 计算局部密度
    double computeLocalDensity(const QVector3D& point, double radius) const;
    
    // 计算局部曲率
    double computeLocalCurvature(const QVector3D& point) const;

private:
    // 输入
    QVector<QVector3D> m_referencePoints;
    PathPlanParams m_baseParams;
    
    // 自适应参数
    double m_minDensity = 10.0;   // 最小点密度 (points/mm²)
    double m_maxDensity = 100.0;  // 最大点密度
    
    // 输出
    ScanPath m_path;
    QVector<PathPlanParams> m_localParams;
};

// ==================== 注册算法 ====================
// 在cpp文件中使用 REGISTER_ALGORITHM 宏注册
