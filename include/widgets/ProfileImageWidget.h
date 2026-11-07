#pragma once

#include <QWidget>
#include <QImage>
#include <QVector>
#include <QTimer>
#include <QMutex>
#include <atomic>

/**
 * @brief 轮廓图像视图（带节流）
 * 
 * 以灰度图形式显示连续的轮廓数据，形成类似瀑布图的效果。
 */
class ProfileImageWidget : public QWidget
{
    Q_OBJECT

public:
    explicit ProfileImageWidget(QWidget* parent = nullptr);
    ~ProfileImageWidget() override = default;

    /**
     * @brief 推入一条新的轮廓线（节流，不立即刷新）
     */
    void pushProfileLine(const QVector<double>& z);

    /**
     * @brief 清空图像
     */
    void clear();

    /**
     * @brief 设置图像高度（行数）
     */
    void setImageHeight(int height);

protected:
    void paintEvent(QPaintEvent* event) override;

private slots:
    void onRefreshTimer();

private:
    void processLine(const QVector<double>& z);

private:
    QImage m_img;
    int m_nextLine = 0;
    int m_imageHeight = 512;

    double m_minZ = 0.0;
    double m_maxZ = 0.0;
    bool m_hasRange = false;
    
    // 待处理的行数据队列
    QVector<QVector<double>> m_pendingLines;
    QMutex m_pendingMutex;
    std::atomic<bool> m_hasPendingData{false};
    
    // 刷新定时器
    QTimer m_refreshTimer;
    std::atomic<bool> m_needsRepaint{false};
};
