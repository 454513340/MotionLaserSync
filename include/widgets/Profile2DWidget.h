#pragma once

#include <QWidget>
#include <QVector>
#include <QPointF>
#include <QTimer>
#include <QMutex>
#include <atomic>

/**
 * @brief 二维轮廓视图（带节流）
 * 
 * 显示激光轮廓的 X-Z 坐标图，支持缩放和平移。
 */
class Profile2DWidget : public QWidget
{
    Q_OBJECT

public:
    explicit Profile2DWidget(QWidget* parent = nullptr);
    ~Profile2DWidget() override = default;

    /**
     * @brief 设置轮廓数据（节流，不会立即刷新）
     */
    void setProfile(const QVector<double>& x, const QVector<double>& z);

    /**
     * @brief 清空数据
     */
    void clear();

    /**
     * @brief 重置视图（自动缩放）
     */
    void resetView();

    /**
     * @brief Optional calibration overlay used by the single-sphere workflow.
     * The selected points and fitted circle are expressed in sensor X-Z mm.
     */
    void setCalibrationOverlay(const QVector<QPointF>& points,
                               double centerX, double centerZ, double radius);
    void clearCalibrationOverlay();

protected:
    void paintEvent(QPaintEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;

private slots:
    void onRefreshTimer();

private:
    void updateDataRange();
    QRectF plotRect() const;

private:
    // 当前显示的数据
    QVector<double> m_x;
    QVector<double> m_z;
    
    // 待更新的数据（由 setProfile 写入）
    QVector<double> m_pendingX;
    QVector<double> m_pendingZ;
    QMutex m_pendingMutex;
    std::atomic<bool> m_hasPendingData{false};
    
    // 刷新定时器
    QTimer m_refreshTimer;

    // 数据范围
    bool m_hasDataRange = false;
    double m_dataXMin = 0.0, m_dataXMax = 1.0;
    double m_dataZMin = 0.0, m_dataZMax = 1.0;

    // 视图范围
    bool m_viewInitialized = false;
    double m_viewXMin = -1.0, m_viewXMax = 1.0;
    double m_viewZMin = -1.0, m_viewZMax = 1.0;

    // 可选标定覆盖层
    QVector<QPointF> m_calibrationOverlayPoints;
    bool m_hasCalibrationCircle = false;
    double m_calibrationCenterX = 0.0;
    double m_calibrationCenterZ = 0.0;
    double m_calibrationRadius = 0.0;

    // 平移状态
    bool m_panning = false;
    QPoint m_lastPos;

    // 边距
    static constexpr int kLeftMargin = 50;
    static constexpr int kRightMargin = 20;
    static constexpr int kTopMargin = 15;
    static constexpr int kBottomMargin = 40;
};
