#pragma once

#include <QWidget>
#include <QImage>
#include <QMutex>
#include <QTimer>
#include <atomic>

/**
 * @brief Video 预览控件
 * 
 * 显示激光传感器的原始 CMOS 图像，用于调试和检查激光线质量。
 * 支持：
 *   - 实时显示 Video 帧
 *   - 自动缩放适应窗口
 *   - 保持宽高比
 *   - 显示帧率和图像信息
 */
class VideoWidget : public QWidget
{
    Q_OBJECT

public:
    explicit VideoWidget(QWidget* parent = nullptr);
    ~VideoWidget() override = default;

public slots:
    /**
     * @brief 更新视频帧
     * @param frame 灰度图像
     */
    void setFrame(const QImage& frame);
    
    /**
     * @brief 清除显示
     */
    void clear();
    
    /**
     * @brief 获取当前帧
     * @return 当前显示的图像副本
     */
    QImage currentFrame() const;

protected:
    void paintEvent(QPaintEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;

private slots:
    void onRefreshTimer();
    void updateFps();

private:
    void updateScaledImage();

private:
    // 当前帧
    QImage m_currentFrame;
    QImage m_scaledFrame;
    QMutex m_frameMutex;
    
    // 待更新的帧（节流）
    QImage m_pendingFrame;
    QMutex m_pendingMutex;
    std::atomic<bool> m_hasPendingFrame{false};
    
    // 刷新定时器
    QTimer m_refreshTimer;
    
    // 帧率统计
    QTimer m_fpsTimer;
    int m_frameCount = 0;
    double m_fps = 0.0;
    
    // 缩放状态
    bool m_needsRescale = true;
    QRect m_imageRect;  // 图像在控件中的位置和大小
};
