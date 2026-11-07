#include "widgets/ProfileImageWidget.h"
#include "core/DataTypes.h"

#include <QPainter>
#include <cmath>

// 刷新间隔（毫秒）
static constexpr int kRefreshIntervalMs = 100;  // 10 FPS

ProfileImageWidget::ProfileImageWidget(QWidget* parent)
    : QWidget(parent)
{
    setMinimumHeight(200);
    
    // 设置刷新定时器
    m_refreshTimer.setInterval(kRefreshIntervalMs);
    connect(&m_refreshTimer, &QTimer::timeout, this, &ProfileImageWidget::onRefreshTimer);
    m_refreshTimer.start();
}

void ProfileImageWidget::setImageHeight(int height)
{
    m_imageHeight = height;
    clear();
}

void ProfileImageWidget::clear()
{
    m_img = QImage();
    m_nextLine = 0;
    m_hasRange = false;
    m_minZ = 0.0;
    m_maxZ = 0.0;
    
    {
        QMutexLocker locker(&m_pendingMutex);
        m_pendingLines.clear();
        m_hasPendingData.store(false);
    }
    
    update();
}

void ProfileImageWidget::pushProfileLine(const QVector<double>& z)
{
    if (z.isEmpty()) return;
    
    // 只保存数据，不立即处理
    QMutexLocker locker(&m_pendingMutex);
    
    // 限制队列大小，避免内存增长
    if (m_pendingLines.size() > 100) {
        m_pendingLines.removeFirst();
    }
    
    m_pendingLines.append(z);
    m_hasPendingData.store(true);
}

void ProfileImageWidget::onRefreshTimer()
{
    if (!m_hasPendingData.load()) {
        if (m_needsRepaint.load()) {
            m_needsRepaint.store(false);
            update();
        }
        return;
    }
    
    // 获取待处理数据
    QVector<QVector<double>> linesToProcess;
    {
        QMutexLocker locker(&m_pendingMutex);
        linesToProcess = m_pendingLines;
        m_pendingLines.clear();
        m_hasPendingData.store(false);
    }
    
    // 处理所有待处理的行
    for (const auto& z : linesToProcess) {
        processLine(z);
    }
    
    update();
}

void ProfileImageWidget::processLine(const QVector<double>& z)
{
    if (z.isEmpty()) return;

    // 创建或重建图像
    if (m_img.isNull() || m_img.width() != z.size() || m_img.height() != m_imageHeight) {
        m_img = QImage(z.size(), m_imageHeight, QImage::Format_Indexed8);
        QVector<QRgb> table(256);
        for (int i = 0; i < 256; ++i) {
            table[i] = qRgb(i, i, i);
        }
        m_img.setColorTable(table);
        m_img.fill(0);
        m_nextLine = 0;
        // 使用 2950-50BL 传感器的固定量程范围
        m_minZ = Constants::kSensorZMin;
        m_maxZ = Constants::kSensorZMax;
        m_hasRange = true;  // 直接使用固定范围
    }

    // 使用传感器固定量程，不动态更新范围
    double range = m_maxZ - m_minZ;

    // 写入当前行
    uchar* line = m_img.scanLine(m_nextLine);
    for (int i = 0; i < m_img.width(); ++i) {
        double v = z[i];
        
        // 无效值（z=0）显示为黑色
        if (i >= z.size() || !SensorUtils::isValidZ(v)) {
            line[i] = 0;  // 黑色表示无效
            continue;
        }
        
        int gray = static_cast<int>((v - m_minZ) / range * 255.0);
        gray = qBound(0, gray, 255);
        line[i] = static_cast<uchar>(gray);
    }

    m_nextLine = (m_nextLine + 1) % m_img.height();
}

void ProfileImageWidget::paintEvent(QPaintEvent*)
{
    QPainter p(this);
    p.fillRect(rect(), Qt::black);

    if (m_img.isNull()) {
        p.setPen(Qt::white);
        p.drawText(rect(), Qt::AlignCenter, tr("无图像数据"));
        return;
    }

    const int h = m_img.height();
    if (h <= 0) return;

    QRect widgetRect = rect();

    // 将循环缓冲区分成两段绘制
    int firstRows = h - m_nextLine;

    if (firstRows > 0) {
        QRect src1(0, m_nextLine, m_img.width(), firstRows);
        QRectF dst1(widgetRect.left(),
                    widgetRect.top(),
                    widgetRect.width(),
                    widgetRect.height() * (double(firstRows) / h));
        p.drawImage(dst1, m_img, src1);
    }

    if (m_nextLine > 0) {
        QRect src2(0, 0, m_img.width(), m_nextLine);
        QRectF dst2(widgetRect.left(),
                    widgetRect.top() + widgetRect.height() * (double(firstRows) / h),
                    widgetRect.width(),
                    widgetRect.height() * (double(m_nextLine) / h));
        p.drawImage(dst2, m_img, src2);
    }

    // 边框
    p.setPen(QColor(100, 100, 100));
    p.drawRect(widgetRect.adjusted(0, 0, -1, -1));
}
