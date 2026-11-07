#include "widgets/VideoWidget.h"

#include <QPainter>
#include <QResizeEvent>

// 刷新间隔（毫秒）- 约 30 FPS
static constexpr int kRefreshIntervalMs = 33;
// FPS 统计间隔
static constexpr int kFpsUpdateIntervalMs = 1000;

VideoWidget::VideoWidget(QWidget* parent)
    : QWidget(parent)
{
    setMinimumSize(320, 240);
    setMouseTracking(true);
    
    // 设置刷新定时器
    m_refreshTimer.setInterval(kRefreshIntervalMs);
    connect(&m_refreshTimer, &QTimer::timeout, this, &VideoWidget::onRefreshTimer);
    m_refreshTimer.start();
    
    // 设置 FPS 统计定时器
    m_fpsTimer.setInterval(kFpsUpdateIntervalMs);
    connect(&m_fpsTimer, &QTimer::timeout, this, &VideoWidget::updateFps);
    m_fpsTimer.start();
}

void VideoWidget::setFrame(const QImage& frame)
{
    if (frame.isNull()) return;
    
    // 只保存待更新帧，不立即刷新
    QMutexLocker locker(&m_pendingMutex);
    m_pendingFrame = frame;
    m_hasPendingFrame.store(true);
}

void VideoWidget::clear()
{
    {
        QMutexLocker locker(&m_frameMutex);
        m_currentFrame = QImage();
        m_scaledFrame = QImage();
    }
    
    {
        QMutexLocker locker(&m_pendingMutex);
        m_pendingFrame = QImage();
        m_hasPendingFrame.store(false);
    }
    
    m_frameCount = 0;
    m_fps = 0.0;
    m_needsRescale = true;
    
    update();
}

void VideoWidget::onRefreshTimer()
{
    if (!m_hasPendingFrame.load()) return;
    
    // 获取待更新帧
    QImage newFrame;
    {
        QMutexLocker locker(&m_pendingMutex);
        newFrame = m_pendingFrame;
        m_hasPendingFrame.store(false);
    }
    
    if (newFrame.isNull()) return;
    
    // 直接使用原始图像，不做翻转处理
    
    // 更新当前帧
    {
        QMutexLocker locker(&m_frameMutex);
        
        // 检查尺寸是否变化
        if (m_currentFrame.isNull() || 
            m_currentFrame.size() != newFrame.size()) {
            m_needsRescale = true;
        }
        
        m_currentFrame = newFrame;
    }
    
    // 更新帧计数
    m_frameCount++;
    
    // 如果需要重新缩放
    if (m_needsRescale) {
        updateScaledImage();
    } else {
        // 只更新缩放后的图像
        QMutexLocker locker(&m_frameMutex);
        if (!m_currentFrame.isNull() && m_imageRect.isValid()) {
            m_scaledFrame = m_currentFrame.scaled(
                m_imageRect.size(),
                Qt::KeepAspectRatio,
                Qt::FastTransformation
            );
        }
    }
    
    update();
}

void VideoWidget::updateFps()
{
    m_fps = m_frameCount;
    m_frameCount = 0;
}

QImage VideoWidget::currentFrame() const
{
    QMutexLocker locker(const_cast<QMutex*>(&m_frameMutex));
    return m_currentFrame.copy();
}

void VideoWidget::updateScaledImage()
{
    QMutexLocker locker(&m_frameMutex);
    
    if (m_currentFrame.isNull()) {
        m_imageRect = QRect();
        m_scaledFrame = QImage();
        return;
    }
    
    // 计算保持宽高比的图像位置和大小
    QSize frameSize = m_currentFrame.size();
    QSize widgetSize = size();
    
    // 计算缩放比例
    double scaleX = static_cast<double>(widgetSize.width()) / frameSize.width();
    double scaleY = static_cast<double>(widgetSize.height()) / frameSize.height();
    double scale = qMin(scaleX, scaleY);
    
    // 计算缩放后的大小
    int scaledWidth = static_cast<int>(frameSize.width() * scale);
    int scaledHeight = static_cast<int>(frameSize.height() * scale);
    
    // 居中显示
    int x = (widgetSize.width() - scaledWidth) / 2;
    int y = (widgetSize.height() - scaledHeight) / 2;
    
    m_imageRect = QRect(x, y, scaledWidth, scaledHeight);
    
    // 缩放图像
    m_scaledFrame = m_currentFrame.scaled(
        m_imageRect.size(),
        Qt::KeepAspectRatio,
        Qt::FastTransformation
    );
    
    m_needsRescale = false;
}

void VideoWidget::resizeEvent(QResizeEvent* event)
{
    QWidget::resizeEvent(event);
    m_needsRescale = true;
}

void VideoWidget::paintEvent(QPaintEvent*)
{
    QPainter p(this);
    p.setRenderHint(QPainter::SmoothPixmapTransform, false);
    
    // 背景
    p.fillRect(rect(), QColor(30, 30, 30));
    
    QMutexLocker locker(&m_frameMutex);
    
    if (m_currentFrame.isNull() || m_scaledFrame.isNull()) {
        // 无图像时显示提示
        p.setPen(Qt::white);
        QFont font = p.font();
        font.setPointSize(14);
        p.setFont(font);
        p.drawText(rect(), Qt::AlignCenter, tr("无 Video 数据\n\n请启动 Video 预览"));
        return;
    }
    
    // 绘制图像
    if (m_imageRect.isValid()) {
        p.drawImage(m_imageRect.topLeft(), m_scaledFrame);
        
        // 绘制边框
        p.setPen(QPen(QColor(100, 100, 100), 1));
        p.drawRect(m_imageRect.adjusted(0, 0, -1, -1));
    }
    
    // 绘制信息叠加层
    p.setPen(QColor(0, 255, 0));
    QFont font = p.font();
    font.setPointSize(10);
    p.setFont(font);
    
    // 左上角：图像尺寸
    QString sizeInfo = QString("%1 × %2").arg(m_currentFrame.width()).arg(m_currentFrame.height());
    p.drawText(10, 20, sizeInfo);
    
    // 右上角：帧率
    QString fpsInfo = QString("FPS: %1").arg(m_fps, 0, 'f', 1);
    QFontMetrics fm(font);
    int fpsWidth = fm.horizontalAdvance(fpsInfo);
    p.drawText(width() - fpsWidth - 10, 20, fpsInfo);
    
    // 底部：提示信息
    p.setPen(QColor(150, 150, 150));
    QString hint = tr("Video 模式 - 用于检查激光线质量和曝光设置");
    p.drawText(10, height() - 10, hint);
}
