#pragma once

#include <QWidget>
#include <QLabel>
#include <QDoubleSpinBox>
#include <QPushButton>

#include "core/DataTypes.h"

class ZmcController;

/**
 * @brief 单轴控制面板
 * 
 * 提供单个轴的位置显示和运动控制功能。
 */
class AxisControlWidget : public QWidget
{
    Q_OBJECT

public:
    explicit AxisControlWidget(int axisId, const QString& axisName, QWidget* parent = nullptr);
    ~AxisControlWidget() override = default;

    /**
     * @brief 设置控制器
     */
    void setController(ZmcController* controller);

    /**
     * @brief 设置轴参数
     */
    void setAxisParams(const AxisParams& params);

    /**
     * @brief 更新当前位置显示
     */
    void updatePosition(float pos, bool idle);

    /**
     * @brief 同时显示 DPOS 与 MPOS，主要用于 B/C 绝对坐标校验。
     */
    void updatePositionPair(float dpos, float mpos, bool idle);

    /**
     * @brief 硬件同步采集时显示来自控制器 TABLE 的机械 MPOS。
     *
     * 该值已经由硬件同步解码层应用 B/C software-frame offset，
     * 不把 TABLE MPOS 伪装成 DPOS，也不会改变标定/RTCP 的坐标源约定。
     */
    void updateHardwareLatchedPosition(float mpos);

    /**
     * @brief 更新控制器 AXISSTATUS 显示。
     * @param statusWord AXISSTATUS 原始状态字；小于 0 表示未连接/未知。
     * @param statusText 解码后的可读状态。
     * @param alarm 是否包含报警/限位类状态。
     */
    void updateAxisStatusWord(int statusWord, const QString& statusText, bool alarm);

    /**
     * @brief 设置控件启用状态
     */
    void setControlsEnabled(bool enabled);

    /**
     * @brief 对没有原点开关的旋转轴启用“绝对位置同步 + 人工伺服使能”模式。
     */
    void configureManualServoEnable(bool required);

    /**
     * @brief 更新旋转轴绝对位置/伺服使能状态。
     */
    void setManualServoState(bool absoluteReady, bool servoEnabled,
                             const QString& statusText = QString());

    /**
     * @brief 获取当前轴参数
     */
    AxisParams axisParams() const { return m_params; }

    /**
     * @brief 获取轴 ID
     */
    int axisId() const { return m_axisId; }

signals:
    /**
     * @brief 请求开始运动
     */
    void homeRequested(int axisId, const AxisParams& params);
    void absMoveRequested(int axisId, float pos, const AxisParams& params);
    void relMoveRequested(int axisId, float delta, const AxisParams& params);
    void jogRequested(int axisId, bool positive, const AxisParams& params);
    void stopRequested(int axisId);
    void servoEnableRequested(int axisId, bool enable);

    /**
     * @brief 发生错误
     */
    void errorOccurred(const QString& msg);

private slots:
    void onHomeClicked();
    void onAbsMoveClicked();
    void onRelMoveClicked();
    void onJogPosPressed();
    void onJogNegPressed();
    void onJogReleased();
    void onStopClicked();
    void onServoEnableClicked();

private:
    void createUi();
    void refreshControlState();

private:
    int m_axisId;
    QString m_axisName;
    ZmcController* m_controller = nullptr;
    AxisParams m_params;

    bool m_controlsEnabled = true;
    bool m_requiresManualServoEnable = false;
    bool m_absoluteReady = false;
    bool m_servoEnabled = false;

    // UI 控件
    QLabel* m_lblPosition = nullptr;
    QLabel* m_lblAxisStatus = nullptr;
    QLabel* m_lblServoState = nullptr;
    QDoubleSpinBox* m_spinAbsPos = nullptr;
    QDoubleSpinBox* m_spinRelStep = nullptr;
    QDoubleSpinBox* m_spinJogSpeed = nullptr;

    QPushButton* m_btnHome = nullptr;
    QPushButton* m_btnAbsMove = nullptr;
    QPushButton* m_btnRelMove = nullptr;
    QPushButton* m_btnJogPos = nullptr;
    QPushButton* m_btnJogNeg = nullptr;
    QPushButton* m_btnStop = nullptr;
    QPushButton* m_btnServoEnable = nullptr;
};
