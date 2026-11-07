#include "widgets/Profile2DWidget.h"
#include "core/DataTypes.h"

#include <QPainter>
#include <QPainterPath>
#include <QWheelEvent>
#include <QMouseEvent>
#include <cmath>
#include <algorithm>

// 刷新间隔（毫秒）
static constexpr int kRefreshIntervalMs = 100;  // 10 FPS

Profile2DWidget::Profile2DWidget(QWidget* parent)
    : QWidget(parent)
{
    setMinimumHeight(200);
    setMouseTracking(true);
    
    // 设置刷新定时器
    m_refreshTimer.setInterval(kRefreshIntervalMs);
    connect(&m_refreshTimer, &QTimer::timeout, this, &Profile2DWidget::onRefreshTimer);
    m_refreshTimer.start();
    
    // 初始化视图范围为传感器固定量程
    resetView();
}

void Profile2DWidget::setProfile(const QVector<double>& x, const QVector<double>& z)
{
    // 只保存数据，不立即刷新
    QMutexLocker locker(&m_pendingMutex);
    m_pendingX = x;
    m_pendingZ = z;
    m_hasPendingData.store(true);
}

void Profile2DWidget::onRefreshTimer()
{
    if (!m_hasPendingData.load()) return;
    
    // 获取待更新数据
    {
        QMutexLocker locker(&m_pendingMutex);
        m_x = m_pendingX;
        m_z = m_pendingZ;
        m_hasPendingData.store(false);
    }
    
    update();
}

void Profile2DWidget::clear()
{
    m_x.clear();
    m_z.clear();
    m_calibrationOverlayPoints.clear();
    m_hasCalibrationCircle = false;
    m_viewInitialized = false;
    resetView();
    update();
}

void Profile2DWidget::setCalibrationOverlay(const QVector<QPointF>& points,
                                            double centerX, double centerZ, double radius)
{
    m_calibrationOverlayPoints = points;
    m_calibrationCenterX = centerX;
    m_calibrationCenterZ = centerZ;
    m_calibrationRadius = radius;
    m_hasCalibrationCircle = std::isfinite(centerX) && std::isfinite(centerZ)
                          && std::isfinite(radius) && radius > 0.0;
    update();
}

void Profile2DWidget::clearCalibrationOverlay()
{
    m_calibrationOverlayPoints.clear();
    m_hasCalibrationCircle = false;
    update();
}

void Profile2DWidget::resetView()
{
    // 直接使用传感器固定量程作为基础视图范围
    // Z 轴：70 ~ 120 mm
    // X 轴：-25 ~ 25 mm
    m_viewXMin = Constants::kSensorXMin;
    m_viewXMax = Constants::kSensorXMax;
    m_viewZMin = Constants::kSensorZMin;
    m_viewZMax = Constants::kSensorZMax;
    m_viewInitialized = true;
    
    update();
}

void Profile2DWidget::updateDataRange()
{
    // 不再动态计算数据范围，使用固定的传感器量程
    // 此函数保留以兼容原有调用，但不做任何操作
}

QRectF Profile2DWidget::plotRect() const
{
    return rect().adjusted(kLeftMargin, kTopMargin, -kRightMargin, -kBottomMargin);
}

void Profile2DWidget::paintEvent(QPaintEvent*)
{
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);

    // 背景
    p.fillRect(rect(), QColor(20, 20, 20));

    QRectF pr = plotRect();
    if (!pr.isValid() || pr.width() <= 0 || pr.height() <= 0) return;

    // 计算显示范围
    // 基础范围使用传感器固定量程
    double minX = m_viewXMin;
    double maxX = m_viewXMax;
    double minZ = m_viewZMin;
    double maxZ = m_viewZMax;
    
    // 根据界面宽高比调整 X 轴范围
    // 传感器量程：X=50mm, Z=50mm，比例为 1:1
    // 如果界面宽度大于高度，扩展 X 轴范围使显示不变形
    double dataAspect = (maxX - minX) / (maxZ - minZ);  // 数据宽高比 = 1.0
    double viewAspect = pr.width() / pr.height();       // 界面宽高比
    
    if (viewAspect > dataAspect) {
        // 界面比数据更宽，扩展 X 轴范围
        double xCenter = (minX + maxX) / 2.0;
        double zRange = maxZ - minZ;
        double newXRange = zRange * viewAspect;  // 按界面比例计算新的X范围
        minX = xCenter - newXRange / 2.0;
        maxX = xCenter + newXRange / 2.0;
    } else if (viewAspect < dataAspect) {
        // 界面比数据更窄，扩展 Z 轴范围（保持数据居中）
        double zCenter = (minZ + maxZ) / 2.0;
        double xRange = maxX - minX;
        double newZRange = xRange / viewAspect;
        minZ = zCenter - newZRange / 2.0;
        maxZ = zCenter + newZRange / 2.0;
    }

    // 网格
    p.setPen(QPen(QColor(60, 60, 60), 1, Qt::DotLine));
    const int gridCount = 5;
    for (int i = 0; i <= gridCount; ++i) {
        double t = i / double(gridCount);
        double y = pr.top() + t * pr.height();
        double x = pr.left() + t * pr.width();
        p.drawLine(QPointF(pr.left(), y), QPointF(pr.right(), y));
        p.drawLine(QPointF(x, pr.top()), QPointF(x, pr.bottom()));
    }

    // 坐标轴
    p.setPen(QPen(Qt::gray, 1));
    p.drawLine(pr.bottomLeft(), pr.bottomRight());
    p.drawLine(pr.bottomLeft(), pr.topLeft());

    // 刻度
    QFont f = p.font();
    f.setPointSizeF(f.pointSizeF() * 0.9);
    p.setFont(f);
    p.setPen(QPen(Qt::lightGray, 1));

    for (int i = 0; i <= gridCount; ++i) {
        double t = i / double(gridCount);

        // X 轴刻度
        double xPix = pr.left() + t * pr.width();
        double xVal = minX + t * (maxX - minX);
        p.drawLine(QPointF(xPix, pr.bottom()), QPointF(xPix, pr.bottom() + 5));
        QString xStr = QString::number(xVal, 'f', 1);
        QRectF xRect(xPix - 30, pr.bottom() + 5, 60, 14);
        p.drawText(xRect, Qt::AlignHCenter | Qt::AlignTop, xStr);

        // Z 轴刻度（垂直翻转：顶部是maxZ，底部是minZ）
        double yPix = pr.top() + t * pr.height();
        double zVal = maxZ - t * (maxZ - minZ);
        p.drawLine(QPointF(pr.left() - 5, yPix), QPointF(pr.left(), yPix));
        QString zStr = QString::number(zVal, 'f', 1);
        QRectF zRect(0, yPix - 8, kLeftMargin - 7, 16);
        p.drawText(zRect, Qt::AlignRight | Qt::AlignVCenter, zStr);
    }

    // 轴标签
    p.setPen(Qt::lightGray);
    p.drawText(QRectF(pr.left(), rect().bottom() - kBottomMargin, pr.width(), kBottomMargin),
               Qt::AlignCenter, tr("X / mm"));

    p.save();
    p.translate(0, rect().center().y());
    p.rotate(-90);
    p.drawText(QRectF(-rect().height() * 0.5, 0, rect().height(), kLeftMargin),
               Qt::AlignCenter, tr("Z / mm"));
    p.restore();

    // 绘制轮廓线
    if (m_x.isEmpty() || m_z.isEmpty()) {
        p.setPen(Qt::white);
        p.drawText(pr, Qt::AlignCenter, tr("无轮廓数据"));
    } else {
        // 轮廓线 - 跳过无效点（z=0）
        QPainterPath path;
        bool pathStarted = false;
        
        double xRange = maxX - minX;
        double zRange = maxZ - minZ;
        
        for (int i = 0; i < m_x.size() && i < m_z.size(); ++i) {
            double zVal = m_z[i];
            double xVal = m_x[i];
            
            // 只检查 z=0 无效值
            if (!SensorUtils::isValidZ(zVal)) continue;
            
            double xNorm = (xVal - minX) / xRange;
            double zNorm = (zVal - minZ) / zRange;

            double px = pr.left() + xNorm * pr.width();
            // 正常显示：Z值越大越靠上（屏幕Y值越小）
            double py = pr.bottom() - zNorm * pr.height();

            if (!pathStarted) {
                path.moveTo(px, py);
                pathStarted = true;
            } else {
                path.lineTo(px, py);
            }
        }

        p.setPen(QPen(QColor(0, 220, 120), 2));
        p.drawPath(path);
        
        // 显示有效点数统计
        int validCount = 0;
        for (int i = 0; i < m_z.size(); ++i) {
            if (SensorUtils::isValidZ(m_z[i])) validCount++;
        }
        p.setPen(QColor(150, 150, 150));
        QString info = tr("有效点: %1 / %2").arg(validCount).arg(m_z.size());
        p.drawText(QRectF(pr.right() - 120, pr.top() + 5, 115, 20), 
                   Qt::AlignRight | Qt::AlignTop, info);
    }

    // 单球标定覆盖层：ROI 点与拟合截面圆。
    const double xRangeOverlay = maxX - minX;
    const double zRangeOverlay = maxZ - minZ;
    if (!m_calibrationOverlayPoints.isEmpty() && xRangeOverlay > 0.0 && zRangeOverlay > 0.0) {
        p.setPen(QPen(QColor(255, 190, 40), 1));
        p.setBrush(QColor(255, 190, 40));
        for (const QPointF& point : m_calibrationOverlayPoints) {
            const double px = pr.left() + (point.x() - minX) / xRangeOverlay * pr.width();
            const double py = pr.bottom() - (point.y() - minZ) / zRangeOverlay * pr.height();
            if (pr.adjusted(-2,-2,2,2).contains(QPointF(px,py))) p.drawEllipse(QPointF(px,py), 1.7, 1.7);
        }
    }
    if (m_hasCalibrationCircle && xRangeOverlay > 0.0 && zRangeOverlay > 0.0) {
        const double left = pr.left() + (m_calibrationCenterX - m_calibrationRadius - minX) / xRangeOverlay * pr.width();
        const double right = pr.left() + (m_calibrationCenterX + m_calibrationRadius - minX) / xRangeOverlay * pr.width();
        const double top = pr.bottom() - (m_calibrationCenterZ + m_calibrationRadius - minZ) / zRangeOverlay * pr.height();
        const double bottom = pr.bottom() - (m_calibrationCenterZ - m_calibrationRadius - minZ) / zRangeOverlay * pr.height();
        p.setBrush(Qt::NoBrush);
        p.setPen(QPen(QColor(255, 80, 80), 1.5, Qt::DashLine));
        p.drawEllipse(QRectF(QPointF(left,top), QPointF(right,bottom)).normalized());
    }

    // 外框
    p.setPen(QPen(QColor(120, 120, 120), 1));
    p.drawRect(pr);
}

void Profile2DWidget::wheelEvent(QWheelEvent* ev)
{
    QPoint numDeg = ev->angleDelta() / 8;
    if (numDeg.isNull()) {
        ev->ignore();
        return;
    }

    double steps = numDeg.y() / 15.0;
    double factor = std::pow(1.1, -steps);

    QRectF pr = plotRect();
    if (!pr.isValid()) {
        ev->ignore();
        return;
    }

    QPointF pos = ev->position();
    if (!pr.contains(pos)) {
        pos = pr.center();
    }

    double xData = m_viewXMin + (pos.x() - pr.left()) / pr.width() * (m_viewXMax - m_viewXMin);
    double zData = m_viewZMax - (pos.y() - pr.top()) / pr.height() * (m_viewZMax - m_viewZMin);

    double w = m_viewXMax - m_viewXMin;
    double h = m_viewZMax - m_viewZMin;
    double nw = w * factor;
    double nh = h * factor;

    if (nw <= 0 || nh <= 0) {
        ev->ignore();
        return;
    }

    double xRatio = (xData - m_viewXMin) / w;
    double zRatio = (zData - m_viewZMin) / h;

    m_viewXMin = xData - xRatio * nw;
    m_viewXMax = m_viewXMin + nw;
    m_viewZMin = zData - zRatio * nh;
    m_viewZMax = m_viewZMin + nh;

    update();
    ev->accept();
}

void Profile2DWidget::mousePressEvent(QMouseEvent* ev)
{
    if (ev->button() == Qt::LeftButton) {
        m_panning = true;
        m_lastPos = ev->pos();
        setCursor(Qt::ClosedHandCursor);
    } else if (ev->button() == Qt::RightButton) {
        // 右键重置视图
        resetView();
    }
    QWidget::mousePressEvent(ev);
}

void Profile2DWidget::mouseMoveEvent(QMouseEvent* ev)
{
    if (!m_panning) {
        QWidget::mouseMoveEvent(ev);
        return;
    }

    QRectF pr = plotRect();
    if (!pr.isValid()) {
        QWidget::mouseMoveEvent(ev);
        return;
    }

    QPointF delta = ev->pos() - m_lastPos;
    m_lastPos = ev->pos();

    double w = m_viewXMax - m_viewXMin;
    double h = m_viewZMax - m_viewZMin;

    double dx = -delta.x() / pr.width() * w;
    // 正常显示：鼠标向下拖动时，视图向下移动（Z值减小）
    double dz = delta.y() / pr.height() * h;

    m_viewXMin += dx;
    m_viewXMax += dx;
    m_viewZMin += dz;
    m_viewZMax += dz;

    update();
    QWidget::mouseMoveEvent(ev);
}

void Profile2DWidget::mouseReleaseEvent(QMouseEvent* ev)
{
    if (ev->button() == Qt::LeftButton) {
        m_panning = false;
        unsetCursor();
    }
    QWidget::mouseReleaseEvent(ev);
}
