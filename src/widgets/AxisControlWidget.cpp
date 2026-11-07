#include "widgets/AxisControlWidget.h"
#include "core/ZmcController.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QLabel>

AxisControlWidget::AxisControlWidget(int axisId, const QString& axisName, QWidget* parent)
    : QWidget(parent)
    , m_axisId(axisId)
    , m_axisName(axisName)
{
    createUi();
}

void AxisControlWidget::setController(ZmcController* controller)
{
    m_controller = controller;
}

void AxisControlWidget::setAxisParams(const AxisParams& params)
{
    m_params = params;
    m_spinJogSpeed->setValue(params.speed);
}

void AxisControlWidget::updatePosition(float pos, bool idle)
{
    QString status = idle ? tr("停止") : tr("运动中");
    m_lblPosition->setText(tr("当前位置：%1 (%2)")
                           .arg(pos, 0, 'f', 3)
                           .arg(status));
}

void AxisControlWidget::updatePositionPair(float dpos, float mpos, bool idle)
{
    const QString status = idle ? tr("停止") : tr("运动中");
    m_lblPosition->setText(tr("机械DPOS：%1   机械MPOS：%2 (%3)")
                           .arg(dpos, 0, 'f', 3)
                           .arg(mpos, 0, 'f', 3)
                           .arg(status));
}

void AxisControlWidget::updateHardwareLatchedPosition(float mpos)
{
    m_lblPosition->setText(tr("机械MPOS(TABLE锁存)：%1 (硬件同步)")
                           .arg(mpos, 0, 'f', 3));
}


void AxisControlWidget::updateAxisStatusWord(int statusWord, const QString& statusText, bool alarm)
{
    if (!m_lblAxisStatus) return;

    if (statusWord < 0) {
        m_lblAxisStatus->setText(tr("轴状态：%1").arg(statusText.isEmpty() ? tr("未连接") : statusText));
        m_lblAxisStatus->setStyleSheet("QLabel { color: #666666; font-weight: bold; }");
        return;
    }

    const QString hex = QStringLiteral("0x%1")
                            .arg(static_cast<quint32>(statusWord), 0, 16)
                            .toUpper();
    if (statusWord == 0) {
        m_lblAxisStatus->setText(tr("轴状态：正常"));
        m_lblAxisStatus->setStyleSheet("QLabel { color: green; font-weight: bold; }");
    } else {
        m_lblAxisStatus->setText(tr("轴状态：%1 (%2)").arg(statusText, hex));
        if (alarm) {
            m_lblAxisStatus->setStyleSheet("QLabel { color: red; font-weight: bold; }");
        } else {
            m_lblAxisStatus->setStyleSheet("QLabel { color: #8a5a00; font-weight: bold; }");
        }
    }
}

void AxisControlWidget::setControlsEnabled(bool enabled)
{
    m_controlsEnabled = enabled;
    refreshControlState();
}

void AxisControlWidget::configureManualServoEnable(bool required)
{
    m_requiresManualServoEnable = required;
    if (m_btnServoEnable) m_btnServoEnable->setVisible(required);
    if (m_lblServoState) m_lblServoState->setVisible(required);

    // B/C 没有原点开关，但在绝对位置同步完成后，“回零”定义为单轴绝对定位到 0°，
    // 不执行 DATUM/原点开关搜索。
    if (m_btnHome) {
        m_btnHome->setVisible(true);
        m_btnHome->setText(required ? tr("回零(0°)") : tr("回零"));
        m_btnHome->setToolTip(required
            ? tr("B/C 回零只执行绝对定位到 0°；需要先完成绝对位置同步并人工使能")
            : QString());
    }

    if (required) {
        m_absoluteReady = false;
        m_servoEnabled = false;
        if (m_lblServoState) m_lblServoState->setText(tr("绝对位置未同步，禁止使能"));
    }
    refreshControlState();
}

void AxisControlWidget::setManualServoState(bool absoluteReady, bool servoEnabled,
                                             const QString& statusText)
{
    m_absoluteReady = absoluteReady;
    m_servoEnabled = absoluteReady && servoEnabled;

    if (m_lblServoState) {
        if (!statusText.isEmpty()) {
            m_lblServoState->setText(statusText);
        } else if (!m_absoluteReady) {
            m_lblServoState->setText(tr("绝对位置未同步，禁止使能"));
        } else if (m_servoEnabled) {
            m_lblServoState->setText(tr("绝对位置已同步 / 伺服已使能"));
        } else {
            m_lblServoState->setText(tr("绝对位置已同步 / 等待人工使能"));
        }
    }
    refreshControlState();
}

void AxisControlWidget::refreshControlState()
{
    const bool rotaryMotionAllowed = !m_requiresManualServoEnable
                                  || (m_absoluteReady && m_servoEnabled);
    const bool motionEnabled = m_controlsEnabled && rotaryMotionAllowed;

    m_spinAbsPos->setEnabled(motionEnabled);
    m_spinRelStep->setEnabled(motionEnabled);
    m_spinJogSpeed->setEnabled(motionEnabled);

    // XYZ 回零使用 DATUM；B/C 回零是绝对定位到 0°，因此和其它旋转轴运动一样
    // 必须先 absoluteReady + 人工使能。
    m_btnHome->setEnabled(m_controlsEnabled
                          && (!m_requiresManualServoEnable || rotaryMotionAllowed));
    m_btnAbsMove->setEnabled(motionEnabled);
    m_btnRelMove->setEnabled(motionEnabled);
    m_btnJogPos->setEnabled(motionEnabled);
    m_btnJogNeg->setEnabled(motionEnabled);

    // Stop 始终保留；旋转轴使能按钮只有绝对位置同步成功后才允许点击。
    m_btnStop->setEnabled(true);
    if (m_btnServoEnable) {
        m_btnServoEnable->setEnabled(m_requiresManualServoEnable && m_absoluteReady && m_controlsEnabled);
        m_btnServoEnable->setText(m_servoEnabled ? tr("断使能") : tr("人工使能"));
    }
}

void AxisControlWidget::createUi()
{
    auto* groupBox = new QGroupBox(tr("%1 轴").arg(m_axisName), this);
    auto* vLayout = new QVBoxLayout(groupBox);

    // 第一行：位置设置
    auto* line1 = new QHBoxLayout;

    line1->addWidget(new QLabel(tr("绝对位置:"), this));
    m_spinAbsPos = new QDoubleSpinBox(this);
    m_spinAbsPos->setRange(-1e6, 1e6);
    m_spinAbsPos->setDecimals(3);
    line1->addWidget(m_spinAbsPos);

    line1->addWidget(new QLabel(tr("相对步长:"), this));
    m_spinRelStep = new QDoubleSpinBox(this);
    m_spinRelStep->setRange(-1e6, 1e6);
    m_spinRelStep->setDecimals(3);
    m_spinRelStep->setValue(10.0);
    line1->addWidget(m_spinRelStep);

    line1->addWidget(new QLabel(tr("点动速度:"), this));
    m_spinJogSpeed = new QDoubleSpinBox(this);
    m_spinJogSpeed->setRange(0, 1e6);
    m_spinJogSpeed->setDecimals(3);
    m_spinJogSpeed->setValue(5.0);
    line1->addWidget(m_spinJogSpeed);

    line1->addStretch();
    vLayout->addLayout(line1);

    // 第二行：当前位置显示
    auto* line2 = new QHBoxLayout;
    m_lblPosition = new QLabel(tr("当前位置：0.000 (停止)"), this);
    line2->addWidget(m_lblPosition);

    m_lblServoState = new QLabel(tr("绝对位置未同步，禁止使能"), this);
    m_lblServoState->setStyleSheet("QLabel { color: #b36b00; font-weight: bold; }");
    m_lblServoState->setVisible(false);
    line2->addWidget(m_lblServoState);

    line2->addStretch();
    vLayout->addLayout(line2);

    // 第三行：控制器轴状态（AXISSTATUS）
    auto* lineStatus = new QHBoxLayout;
    m_lblAxisStatus = new QLabel(tr("轴状态：未连接"), this);
    m_lblAxisStatus->setStyleSheet("QLabel { color: #666666; font-weight: bold; }");
    lineStatus->addWidget(m_lblAxisStatus);
    lineStatus->addStretch();
    vLayout->addLayout(lineStatus);

    // 第四行：按钮
    auto* line3 = new QHBoxLayout;

    m_btnServoEnable = new QPushButton(tr("人工使能"), this);
    m_btnServoEnable->setVisible(false);
    m_btnServoEnable->setToolTip(tr("只有成功读取绝对位置并建立 B/C 机械坐标框架后才允许人工使能"));

    m_btnHome = new QPushButton(tr("回零"), this);
    m_btnAbsMove = new QPushButton(tr("绝对定位"), this);
    m_btnRelMove = new QPushButton(tr("相对定位"), this);
    m_btnJogNeg = new QPushButton(tr("点动 -"), this);
    m_btnJogPos = new QPushButton(tr("点动 +"), this);
    m_btnStop = new QPushButton(tr("停止"), this);

    m_btnStop->setStyleSheet("QPushButton { background-color: #ff6666; }");

    line3->addWidget(m_btnServoEnable);
    line3->addWidget(m_btnHome);
    line3->addWidget(m_btnAbsMove);
    line3->addWidget(m_btnRelMove);
    line3->addWidget(m_btnJogNeg);
    line3->addWidget(m_btnJogPos);
    line3->addWidget(m_btnStop);
    line3->addStretch();

    vLayout->addLayout(line3);

    // 主布局
    auto* mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(0, 0, 0, 0);
    mainLayout->addWidget(groupBox);

    // 连接信号
    connect(m_btnHome, &QPushButton::clicked, this, &AxisControlWidget::onHomeClicked);
    connect(m_btnAbsMove, &QPushButton::clicked, this, &AxisControlWidget::onAbsMoveClicked);
    connect(m_btnRelMove, &QPushButton::clicked, this, &AxisControlWidget::onRelMoveClicked);
    connect(m_btnJogPos, &QPushButton::pressed, this, &AxisControlWidget::onJogPosPressed);
    connect(m_btnJogNeg, &QPushButton::pressed, this, &AxisControlWidget::onJogNegPressed);
    connect(m_btnJogPos, &QPushButton::released, this, &AxisControlWidget::onJogReleased);
    connect(m_btnJogNeg, &QPushButton::released, this, &AxisControlWidget::onJogReleased);
    connect(m_btnStop, &QPushButton::clicked, this, &AxisControlWidget::onStopClicked);
    connect(m_btnServoEnable, &QPushButton::clicked, this, &AxisControlWidget::onServoEnableClicked);

    refreshControlState();
}

void AxisControlWidget::onHomeClicked()
{
    m_params.speed = static_cast<float>(m_spinJogSpeed->value());
    emit homeRequested(m_axisId, m_params);
}

void AxisControlWidget::onAbsMoveClicked()
{
    float pos = static_cast<float>(m_spinAbsPos->value());
    m_params.speed = static_cast<float>(m_spinJogSpeed->value());
    emit absMoveRequested(m_axisId, pos, m_params);
}

void AxisControlWidget::onRelMoveClicked()
{
    float delta = static_cast<float>(m_spinRelStep->value());
    m_params.speed = static_cast<float>(m_spinJogSpeed->value());
    emit relMoveRequested(m_axisId, delta, m_params);
}

void AxisControlWidget::onJogPosPressed()
{
    m_params.speed = static_cast<float>(m_spinJogSpeed->value());
    emit jogRequested(m_axisId, true, m_params);
}

void AxisControlWidget::onJogNegPressed()
{
    m_params.speed = static_cast<float>(m_spinJogSpeed->value());
    emit jogRequested(m_axisId, false, m_params);
}

void AxisControlWidget::onJogReleased()
{
    emit stopRequested(m_axisId);
}

void AxisControlWidget::onStopClicked()
{
    emit stopRequested(m_axisId);
}

void AxisControlWidget::onServoEnableClicked()
{
    if (!m_requiresManualServoEnable || !m_absoluteReady) return;
    emit servoEnableRequested(m_axisId, !m_servoEnabled);
}
