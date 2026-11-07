#include "algorithms/PathPlanningAlgorithms.h"
#include <QFile>
#include <QTextStream>
#include <QtMath>
#include <algorithm>

/**
 * @file PathPlanningAlgorithms.cpp
 * @brief 路径规划算法实现
 */

// ==================== BoundingBox 实现 ====================

void BoundingBox::compute(const QVector<QVector3D>& points)
{
    if (points.isEmpty()) return;
    
    min = max = points[0];
    for (const auto& pt : points) {
        min.setX(qMin(min.x(), pt.x()));
        min.setY(qMin(min.y(), pt.y()));
        min.setZ(qMin(min.z(), pt.z()));
        max.setX(qMax(max.x(), pt.x()));
        max.setY(qMax(max.y(), pt.y()));
        max.setZ(qMax(max.z(), pt.z()));
    }
    center = (min + max) / 2.0f;
    size = max - min;
}

// ==================== PathPlanner 实现 ====================

PathPlanner::PathPlanner(QObject* parent) : IAlgorithm(parent)
{
    m_params.pathType = ScanPathType::Raster;
    m_params.lineSpacing = 1.0;
    m_params.pointSpacing = 0.5;
    m_params.scanSpeed = 10.0;
    m_params.rapidSpeed = 50.0;
    m_params.safeHeight = 10.0;
}

bool PathPlanner::initialize()
{
    m_cancelled = false;
    m_running = false;
    m_path = ScanPath();
    m_path.params = m_params;
    return true;
}

bool PathPlanner::execute()
{
    m_running = true;
    m_cancelled = false;
    
    log(tr("开始路径规划..."));
    reportProgress(0, 100, tr("初始化"));
    
    if (!m_hasBoundary) {
        if (!m_pointCloud.isEmpty()) {
            computeBoundaryFromPointCloud();
        } else {
            setError(tr("未设置扫描边界"));
            m_running = false;
            emit completed(false);
            return false;
        }
    }
    
    reportProgress(20, 100, tr("生成路径"));
    
    bool success = false;
    switch (m_params.pathType) {
    case ScanPathType::Raster:
        success = generateRasterPath();
        break;
    case ScanPathType::Spiral:
        success = generateSpiralPath();
        break;
    case ScanPathType::Contour:
        success = generateContourPath();
        break;
    case ScanPathType::Adaptive:
        success = generateAdaptivePath();
        break;
    default:
        success = generateRasterPath();
        break;
    }
    
    if (!success) {
        m_running = false;
        emit completed(false);
        return false;
    }
    
    reportProgress(80, 100, tr("计算统计信息"));
    computePathStatistics();
    
    m_path.isValid = true;
    m_path.params = m_params;
    
    log(tr("路径规划完成: %1 个点, %2 条扫描线").arg(m_path.points.size()).arg(m_path.lineCount));
    
    reportProgress(100, 100, tr("完成"));
    m_running = false;
    emit completed(true);
    return true;
}

void PathPlanner::setPointCloud(const QVector<QVector3D>& points)
{
    m_pointCloud = points;
}

void PathPlanner::setBoundary(const QVector3D& min, const QVector3D& max)
{
    m_boundingBox.min = min;
    m_boundingBox.max = max;
    m_boundingBox.center = (min + max) / 2.0f;
    m_boundingBox.size = max - min;
    m_hasBoundary = true;
}

void PathPlanner::computeBoundaryFromPointCloud()
{
    if (m_pointCloud.isEmpty()) return;
    m_boundingBox.compute(m_pointCloud);
    QVector3D margin(m_params.marginX, m_params.marginY, 0);
    m_boundingBox.min -= margin;
    m_boundingBox.max += margin;
    m_boundingBox.size = m_boundingBox.max - m_boundingBox.min;
    m_hasBoundary = true;
}

void PathPlanner::addPathPoint(const QVector3D& pos, bool rapid, bool laserOn)
{
    ScanPathPoint pt;
    pt.position = pos;
    pt.feedRate = rapid ? m_params.rapidSpeed : m_params.scanSpeed;
    pt.rapidMove = rapid;
    pt.laserOn = laserOn;
    m_path.points.append(pt);
}

bool PathPlanner::generateRasterPath()
{
    m_path.points.clear();
    
    float xMin = m_boundingBox.min.x();
    float xMax = m_boundingBox.max.x();
    float yMin = m_boundingBox.min.y();
    float yMax = m_boundingBox.max.y();
    float zScan = m_params.nominalHeight;
    float zSafe = m_boundingBox.max.z() + m_params.safeHeight;
    
    int lineCount = 0;
    bool forward = true;
    
    addPathPoint(QVector3D(xMin, yMin, zSafe), true, false);
    addPathPoint(QVector3D(xMin, yMin, zScan), true, false);
    
    for (float y = yMin; y <= yMax; y += m_params.lineSpacing) {
        if (isCancelled()) return false;
        lineCount++;
        
        if (forward) {
            for (float x = xMin; x <= xMax; x += m_params.pointSpacing) {
                ScanPathPoint pt;
                pt.position = QVector3D(x, y, zScan);
                pt.feedRate = m_params.scanSpeed;
                pt.laserOn = true;
                pt.lineIndex = lineCount;
                m_path.points.append(pt);
            }
        } else {
            for (float x = xMax; x >= xMin; x -= m_params.pointSpacing) {
                ScanPathPoint pt;
                pt.position = QVector3D(x, y, zScan);
                pt.feedRate = m_params.scanSpeed;
                pt.laserOn = true;
                pt.lineIndex = lineCount;
                m_path.points.append(pt);
            }
        }
        forward = !forward;
    }
    
    addPathPoint(QVector3D(m_path.points.last().position.x(), 
                           m_path.points.last().position.y(), zSafe), true, false);
    m_path.lineCount = lineCount;
    return true;
}

bool PathPlanner::generateSpiralPath()
{
    m_path.points.clear();
    QVector3D center = m_boundingBox.center;
    float maxRadius = qMin(m_boundingBox.size.x(), m_boundingBox.size.y()) / 2.0f;
    float zScan = m_params.nominalHeight;
    float zSafe = m_boundingBox.max.z() + m_params.safeHeight;
    
    addPathPoint(QVector3D(center.x(), center.y(), zSafe), true, false);
    addPathPoint(QVector3D(center.x(), center.y(), zScan), true, false);
    
    double radius = 0, angle = 0;
    double angleStep = m_params.lineSpacing / (2 * M_PI);
    
    while (radius < maxRadius) {
        if (isCancelled()) return false;
        float x = center.x() + radius * cos(angle);
        float y = center.y() + radius * sin(angle);
        
        ScanPathPoint pt;
        pt.position = QVector3D(x, y, zScan);
        pt.feedRate = m_params.scanSpeed;
        pt.laserOn = true;
        m_path.points.append(pt);
        
        angle += angleStep;
        radius = angle * m_params.lineSpacing / (2 * M_PI);
    }
    
    addPathPoint(QVector3D(m_path.points.last().position.x(),
                           m_path.points.last().position.y(), zSafe), true, false);
    return true;
}

bool PathPlanner::generateContourPath()
{
    // 轮廓跟踪 - 简化实现
    return generateRasterPath();
}

bool PathPlanner::generateAdaptivePath()
{
    // 自适应路径 - 简化实现
    return generateRasterPath();
}

void PathPlanner::computePathStatistics()
{
    m_path.totalLength = 0;
    m_path.scanLength = 0;
    m_path.rapidLength = 0;
    
    for (int i = 1; i < m_path.points.size(); ++i) {
        float dist = (m_path.points[i].position - m_path.points[i-1].position).length();
        m_path.totalLength += dist;
        if (m_path.points[i].rapidMove) {
            m_path.rapidLength += dist;
        } else {
            m_path.scanLength += dist;
        }
    }
    
    m_path.estimatedTime = m_path.scanLength / m_params.scanSpeed + 
                           m_path.rapidLength / m_params.rapidSpeed;
}

QVector<QVector3D> PathPlanner::previewPath() const
{
    QVector<QVector3D> result;
    result.reserve(m_path.points.size());
    for (const auto& pt : m_path.points) {
        result.append(pt.position);
    }
    return result;
}

// ==================== GCodeGenerator 实现 ====================

GCodeGenerator::GCodeGenerator(QObject* parent) : IAlgorithm(parent) {}

bool GCodeGenerator::initialize()
{
    m_cancelled = false;
    m_running = false;
    m_gcode.clear();
    m_lineCount = 0;
    return true;
}

bool GCodeGenerator::execute()
{
    m_running = true;
    log(tr("开始生成G代码..."));
    reportProgress(0, 100, tr("初始化"));
    
    if (m_path.points.isEmpty()) {
        setError(tr("路径为空"));
        m_running = false;
        emit completed(false);
        return false;
    }
    
    reportProgress(20, 100, tr("生成程序头"));
    m_gcode = generateHeader();
    
    reportProgress(40, 100, tr("生成运动指令"));
    m_gcode += generateBody();
    
    reportProgress(80, 100, tr("生成程序尾"));
    m_gcode += generateFooter();
    
    m_lineCount = m_gcode.count('\n');
    m_estimatedTime = m_path.estimatedTime;
    
    log(tr("G代码生成完成: %1 行").arg(m_lineCount));
    reportProgress(100, 100, tr("完成"));
    m_running = false;
    emit completed(true);
    return true;
}

void GCodeGenerator::setPathPoints(const QVector<ScanPathPoint>& points)
{
    m_path.points = points;
}

QString GCodeGenerator::generateHeader()
{
    QString header;
    QTextStream ts(&header);
    
    ts << "; Generated by MotionLaserSync Path Planner\n";
    ts << "; Style: " << static_cast<int>(m_style) << "\n";
    ts << ";\n";
    
    if (m_style == GCodeStyle::Fanuc) {
        ts << "O" << QString::number(m_programNumber).rightJustified(4, '0') << "\n";
    }
    
    ts << "G90 ; 绝对坐标\n";
    ts << "G" << m_workCoordinate << " ; 工件坐标系\n";
    ts << "G21 ; 单位mm\n";
    ts << "\n";
    
    return header;
}

QString GCodeGenerator::generateBody()
{
    QString body;
    QTextStream ts(&body);
    
    bool scanActive = false;
    
    for (int i = 0; i < m_path.points.size(); ++i) {
        const auto& pt = m_path.points[i];
        
        // 扫描控制
        if (m_generateScanCommands) {
            if (pt.laserOn && !scanActive) {
                ts << generateScanStart();
                scanActive = true;
            } else if (!pt.laserOn && scanActive) {
                ts << generateScanStop();
                scanActive = false;
            }
        }
        
        ts << formatMove(pt, i == 0);
    }
    
    if (scanActive && m_generateScanCommands) {
        ts << generateScanStop();
    }
    
    return body;
}

QString GCodeGenerator::generateFooter()
{
    QString footer;
    QTextStream ts(&footer);
    
    ts << "\n";
    ts << "M30 ; 程序结束\n";
    
    if (m_style == GCodeStyle::Fanuc) {
        ts << "%\n";
    }
    
    return footer;
}

QString GCodeGenerator::generateScanStart()
{
    return QString("M100 P%1 ; 开始扫描\n").arg(m_exposureTime);
}

QString GCodeGenerator::generateScanStop()
{
    return "M101 ; 停止扫描\n";
}

QString GCodeGenerator::formatMove(const ScanPathPoint& point, bool isFirst)
{
    QString cmd;
    QString moveCode = point.rapidMove ? "G0" : "G1";
    
    cmd = QString("%1 X%2 Y%3 Z%4")
        .arg(moveCode)
        .arg(point.position.x(), 0, 'f', 3)
        .arg(point.position.y(), 0, 'f', 3)
        .arg(point.position.z(), 0, 'f', 3);
    
    if (!point.rapidMove) {
        cmd += QString(" F%1").arg(point.feedRate * 60, 0, 'f', 1); // mm/min
    }
    
    if (!point.comment.isEmpty()) {
        cmd += " ; " + point.comment;
    }
    
    return cmd + "\n";
}

bool GCodeGenerator::saveToFile(const QString& filePath) const
{
    QFile file(filePath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        return false;
    }
    QTextStream out(&file);
    out << m_gcode;
    file.close();
    return true;
}

// ==================== PathOptimizer 实现 ====================

PathOptimizer::PathOptimizer(QObject* parent) : IAlgorithm(parent) {}

bool PathOptimizer::initialize()
{
    m_cancelled = false;
    m_running = false;
    m_optimizedPath = ScanPath();
    return true;
}

bool PathOptimizer::execute()
{
    m_running = true;
    log(tr("开始路径优化..."));
    reportProgress(0, 100, tr("初始化"));
    
    m_optimizedPath = m_originalPath;
    
    bool success = false;
    switch (m_goal) {
    case OptimizationGoal::MinimizeTime:
        success = optimizeForTime();
        break;
    case OptimizationGoal::MinimizeDistance:
        success = optimizeForDistance();
        break;
    default:
        success = optimizeForTime();
        break;
    }
    
    log(tr("路径优化完成, 改进: %1%").arg(improvementPercentage(), 0, 'f', 1));
    reportProgress(100, 100, tr("完成"));
    m_running = false;
    emit completed(success);
    return success;
}

bool PathOptimizer::optimizeForTime()
{
    // 简化实现：重排序段
    return reorderSegments();
}

bool PathOptimizer::optimizeForDistance()
{
    return reorderSegments();
}

bool PathOptimizer::reorderSegments()
{
    // 简化实现
    return true;
}

bool PathOptimizer::smoothPath()
{
    return true;
}

double PathOptimizer::improvementPercentage() const
{
    if (m_originalPath.totalLength <= 0) return 0;
    return (m_originalPath.totalLength - m_optimizedPath.totalLength) 
           / m_originalPath.totalLength * 100.0;
}

// ==================== AdaptivePathPlanner 实现 ====================

AdaptivePathPlanner::AdaptivePathPlanner(QObject* parent) : IAlgorithm(parent) {}

bool AdaptivePathPlanner::initialize()
{
    m_cancelled = false;
    m_running = false;
    m_path = ScanPath();
    return true;
}

bool AdaptivePathPlanner::execute()
{
    m_running = true;
    log(tr("开始自适应路径规划..."));
    
    analyzePointCloud();
    
    // 使用基础路径规划器
    PathPlanner planner;
    planner.setParameters(m_baseParams);
    planner.setPointCloud(m_referencePoints);
    
    if (!planner.execute()) {
        setError(planner.lastError());
        m_running = false;
        emit completed(false);
        return false;
    }
    
    m_path = planner.path();
    m_running = false;
    emit completed(true);
    return true;
}

void AdaptivePathPlanner::setReferencePointCloud(const QVector<QVector3D>& points)
{
    m_referencePoints = points;
}

void AdaptivePathPlanner::analyzePointCloud()
{
    // 点云特征分析
}

double AdaptivePathPlanner::computeLocalDensity(const QVector3D& point, double radius) const
{
    int count = 0;
    for (const auto& pt : m_referencePoints) {
        if ((pt - point).length() < radius) count++;
    }
    return count / (M_PI * radius * radius);
}

double AdaptivePathPlanner::computeLocalCurvature(const QVector3D& point) const
{
    return 0.0;
}

// 注册算法
REGISTER_ALGORITHM(PathPlanner)
REGISTER_ALGORITHM(GCodeGenerator)
REGISTER_ALGORITHM(PathOptimizer)
REGISTER_ALGORITHM(AdaptivePathPlanner)
