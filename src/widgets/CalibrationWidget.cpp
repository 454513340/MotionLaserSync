#include "widgets/CalibrationWidget.h"

#include "algorithms/CalibrationAlgorithms.h"
#include "widgets/SingleSphere14CalibrationWidget.h"
#include "widgets/HardwareSyncCalibrationWidget.h"
#include "utils/LogManager.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QFileDialog>
#include <QFile>
#include <QTextStream>
#include <QTabBar>

#include <utility>

CalibrationWidget::CalibrationWidget(QWidget* parent)
    : QWidget(parent)
    , m_uncertaintyCalib(std::make_unique<UncertaintyCalibration>())
{
    setupUI();
    connectSignals();
    updateUIState();
}

CalibrationWidget::~CalibrationWidget() = default;

void CalibrationWidget::setupUI()
{
    auto* mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(5, 5, 5, 5);

    m_tabWidget = new QTabWidget(this);
    m_tabWidget->tabBar()->setUsesScrollButtons(true);
    m_tabWidget->tabBar()->setElideMode(Qt::ElideRight);

    // V27.6.16Q: sensor calibration/debug is the first System Calibration tab.
    // MainWindow owns the existing scanCONTROL controls and reparents them here.
    m_sensorCalibrationPage = new QWidget(m_tabWidget);
    m_sensorCalibrationTabIndex = m_tabWidget->addTab(m_sensorCalibrationPage, tr("传感标定"));

    // 单球17参数标定仍由同一个对象持有全部阶段状态，只把它的两个页面
    // 拆到外层两个Tab。这样阶段1的手眼/球心结果无需保存/重载就直接供阶段2使用。
    m_singleSphere14Widget = new SingleSphere14CalibrationWidget(this);
    m_singleSphere14Widget->hide(); // logic/state owner only; its two pages are reparented into tabs below.
    QWidget* handEyePage = m_singleSphere14Widget->stage1Page();
    QWidget* kinematicPage = m_singleSphere14Widget->stage2Page();

    m_handEyeTabIndex = m_tabWidget->addTab(handEyePage, tr("初步手眼标定"));
    m_kinematicTabIndex = m_tabWidget->addTab(kinematicPage, tr("运动学标定"));

    // V27.6.44: fourth calibration layer for the existing V32.11 SINGLE-OP14
    // hardware path.  It calibrates timing only; Physical17 remains untouched.
    m_hardwareSyncCalibrationWidget = new HardwareSyncCalibrationWidget(m_tabWidget);
    m_hardwareSyncTabIndex = m_tabWidget->addTab(m_hardwareSyncCalibrationWidget, tr("硬件同步标定"));
    connect(m_hardwareSyncCalibrationWidget, &HardwareSyncCalibrationWidget::logMessage,
            this, &CalibrationWidget::appendLog);
    connect(m_hardwareSyncCalibrationWidget, &HardwareSyncCalibrationWidget::requestPlannedHardwareScan,
            this, &CalibrationWidget::hardwareTemporalPlannedScanRequested);
    connect(m_hardwareSyncCalibrationWidget, &HardwareSyncCalibrationWidget::requestAbortPlannedHardwareScan,
            this, &CalibrationWidget::hardwareTemporalAbortRequested);
    connect(m_hardwareSyncCalibrationWidget, &HardwareSyncCalibrationWidget::referenceSphereChanged,
            this, &CalibrationWidget::hardwareTemporalReferenceSphereRequested);
    connect(m_hardwareSyncCalibrationWidget, &HardwareSyncCalibrationWidget::referenceCloudViewRequested,
            this, &CalibrationWidget::hardwareTemporalReferenceCloudViewRequested);
    connect(m_hardwareSyncCalibrationWidget, &HardwareSyncCalibrationWidget::runtimeCorrectionChanged,
            this, [this](bool enabled, double delayUs){
                emit hardwareTemporalRuntimeCorrectionChanged(enabled, delayUs);
                emit hardwareTemporalRuntimeStateChanged();
            });
    connect(m_hardwareSyncCalibrationWidget, &HardwareSyncCalibrationWidget::activeCalibrationChanged,
            this, [this](){ emit hardwareTemporalRuntimeStateChanged(); });

    auto* uncertaintyTab = new QWidget(m_tabWidget);
    setupUncertaintyTab(uncertaintyTab);
    m_uncertaintyTabIndex = m_tabWidget->addTab(uncertaintyTab, tr("测量不确定度"));

    mainLayout->addWidget(m_tabWidget, 1);

    // 旧算法状态栏仅供“测量不确定度”页使用；前两页有自己的实时状态。
    m_algorithmStatusPanel = new QWidget(this);
    auto* statusLayout = new QHBoxLayout(m_algorithmStatusPanel);
    statusLayout->setContentsMargins(0, 0, 0, 0);
    m_statusLabel = new QLabel(tr("就绪"), m_algorithmStatusPanel);
    m_progressBar = new QProgressBar(m_algorithmStatusPanel);
    m_progressBar->setRange(0, 100);
    statusLayout->addWidget(m_statusLabel);
    statusLayout->addWidget(m_progressBar, 1);
    m_algorithmStatusPanel->setVisible(false);
    mainLayout->addWidget(m_algorithmStatusPanel);

    connectSingleSphereWorkflow();

    connect(m_tabWidget, &QTabWidget::currentChanged, this, [this](int index) {
        if (m_algorithmStatusPanel) {
            m_algorithmStatusPanel->setVisible(index == m_uncertaintyTabIndex);
        }
        // Stage-1 is profile-centric. Stage-2 must not steal the operator's
        // active 3-D/other view; overlays still update in the background.
        if (index == m_handEyeTabIndex) {
            emit singleSphereProfileViewRequested();
        }
    });
}

void CalibrationWidget::connectSingleSphereWorkflow()
{
    connect(m_singleSphere14Widget, &SingleSphere14CalibrationWidget::logMessage,
            this, &CalibrationWidget::appendLog);
    connect(m_singleSphere14Widget, &SingleSphere14CalibrationWidget::requestProfilePreviewStart,
            this, &CalibrationWidget::singleSphereProfilePreviewStartRequested);
    connect(m_singleSphere14Widget, &SingleSphere14CalibrationWidget::requestProfilePreviewStop,
            this, &CalibrationWidget::singleSphereProfilePreviewStopRequested);
    connect(m_singleSphere14Widget, &SingleSphere14CalibrationWidget::requestProfileView,
            this, &CalibrationWidget::singleSphereProfileViewRequested);
    connect(m_singleSphere14Widget, &SingleSphere14CalibrationWidget::requestProfileOverlay,
            this, &CalibrationWidget::singleSphereProfileOverlayRequested);
    connect(m_singleSphere14Widget, &SingleSphere14CalibrationWidget::requestProfileOverlayClear,
            this, &CalibrationWidget::singleSphereProfileOverlayClearRequested);
    connect(m_singleSphere14Widget, &SingleSphere14CalibrationWidget::requestJointMove,
            this, &CalibrationWidget::singleSphereJointMoveRequested);
    connect(m_singleSphere14Widget, &SingleSphere14CalibrationWidget::requestIndependentJointMove,
            this, &CalibrationWidget::singleSphereIndependentJointMoveRequested);
    connect(m_singleSphere14Widget, &SingleSphere14CalibrationWidget::requestJointStop,
            this, &CalibrationWidget::singleSphereJointStopRequested);
    connect(m_singleSphere14Widget, &SingleSphere14CalibrationWidget::requestMachineSafetySphere,
            this, &CalibrationWidget::singleSphereMachineSafetySphereRequested);
    connect(m_singleSphere14Widget, &SingleSphere14CalibrationWidget::requestStage2SimulationFrame,
            this, &CalibrationWidget::singleSphereStage2SimulationFrameRequested);
    connect(m_singleSphere14Widget, &SingleSphere14CalibrationWidget::requestStage2SimulationClear,
            this, &CalibrationWidget::singleSphereStage2SimulationClearRequested);
    connect(m_singleSphere14Widget, &SingleSphere14CalibrationWidget::requestStage2SimulationView,
            this, &CalibrationWidget::singleSphereStage2SimulationViewRequested);
    connect(m_singleSphere14Widget, &SingleSphere14CalibrationWidget::requestMotionControlView,
            this, &CalibrationWidget::singleSphereMotionControlViewRequested);

    connect(m_singleSphere14Widget, &SingleSphere14CalibrationWidget::runtimeCalibrationUsableChanged,
            this, &CalibrationWidget::singleSphereRuntimeCalibrationUsableChanged);

    // V27.6.10: 阶段1通过后停留在当前页，让操作者先查看/保存初步手眼结果。
    // 阶段2已经解锁，但不再强制跳页；操作者确认结果后手动切到“运动学标定”。
    connect(m_singleSphere14Widget, &SingleSphere14CalibrationWidget::stage1BootstrapCompleted,
            this, [this](bool success) {
        if (!success) return;
        emit singleSphereProfileViewRequested();
        appendLog(tr("阶段1初步手眼标定通过：结果已固定显示在当前页，可先保存阶段1 JSON；运动学标定Tab已解锁，请确认后手动进入阶段2。"));
    });
}

void CalibrationWidget::setupUncertaintyTab(QWidget* parent)
{
    auto* layout = new QVBoxLayout(parent);

    auto* paramGroup = new QGroupBox(tr("标定参数"), parent);
    auto* paramLayout = new QGridLayout(paramGroup);

    paramLayout->addWidget(new QLabel(tr("分辨率(mm):")), 0, 0);
    m_spinResolution = new QDoubleSpinBox(paramGroup);
    m_spinResolution->setRange(0.0001, 1.0);
    m_spinResolution->setDecimals(4);
    m_spinResolution->setValue(0.001);
    paramLayout->addWidget(m_spinResolution, 0, 1);

    paramLayout->addWidget(new QLabel(tr("规格精度(mm):")), 0, 2);
    m_spinAccuracy = new QDoubleSpinBox(paramGroup);
    m_spinAccuracy->setRange(0.001, 1.0);
    m_spinAccuracy->setValue(0.01);
    paramLayout->addWidget(m_spinAccuracy, 0, 3);

    paramLayout->addWidget(new QLabel(tr("温度变化(°C):")), 1, 0);
    m_spinTempVar = new QDoubleSpinBox(paramGroup);
    m_spinTempVar->setRange(0, 50);
    m_spinTempVar->setValue(5);
    paramLayout->addWidget(m_spinTempVar, 1, 1);

    paramLayout->addWidget(new QLabel(tr("重复测量次数:")), 1, 2);
    m_spinRepeatCount = new QSpinBox(paramGroup);
    m_spinRepeatCount->setRange(3, 100);
    m_spinRepeatCount->setValue(10);
    paramLayout->addWidget(m_spinRepeatCount, 1, 3);
    layout->addWidget(paramGroup);

    auto* dataGroup = new QGroupBox(tr("测量数据"), parent);
    auto* dataLayout = new QVBoxLayout(dataGroup);
    m_uncertaintyTable = new QTableWidget(0, 3, dataGroup);
    m_uncertaintyTable->setHorizontalHeaderLabels({tr("组号"), tr("测量值"), tr("参考值")});
    m_uncertaintyTable->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    m_uncertaintyTable->setMaximumHeight(120);
    dataLayout->addWidget(m_uncertaintyTable);

    auto* dataBtnLayout = new QHBoxLayout;
    m_btnAddMeasurement = new QPushButton(tr("添加测量"), dataGroup);
    m_btnClearUncertainty = new QPushButton(tr("清空数据"), dataGroup);
    dataBtnLayout->addWidget(m_btnAddMeasurement);
    dataBtnLayout->addWidget(m_btnClearUncertainty);
    dataBtnLayout->addStretch();
    dataLayout->addLayout(dataBtnLayout);
    layout->addWidget(dataGroup);

    auto* ctrlLayout = new QHBoxLayout;
    m_btnStartUncertainty = new QPushButton(tr("开始标定"), parent);
    m_btnStartUncertainty->setStyleSheet("background-color: #4CAF50; color: white;");
    m_btnStopUncertainty = new QPushButton(tr("停止"), parent);
    m_btnStopUncertainty->setEnabled(false);
    m_btnExportUncertainty = new QPushButton(tr("导出报告"), parent);
    m_btnTestUncertainty = new QPushButton(tr("测试"), parent);
    m_btnTestUncertainty->setStyleSheet("background-color: #2196F3; color: white;");

    ctrlLayout->addWidget(m_btnStartUncertainty);
    ctrlLayout->addWidget(m_btnStopUncertainty);
    ctrlLayout->addWidget(m_btnExportUncertainty);
    ctrlLayout->addStretch();
    ctrlLayout->addWidget(m_btnTestUncertainty);
    layout->addLayout(ctrlLayout);

    m_lblUncertaintyResult = new QLabel(tr("扩展不确定度 U = --- mm (k=2)"), parent);
    m_lblUncertaintyResult->setStyleSheet("font-weight: bold; font-size: 14px;");
    layout->addWidget(m_lblUncertaintyResult);
    layout->addStretch();
}

void CalibrationWidget::setController(ZmcController* controller)
{
    m_controller = controller;
    if (m_singleSphere14Widget) m_singleSphere14Widget->setController(controller);
    if (m_hardwareSyncCalibrationWidget) m_hardwareSyncCalibrationWidget->setController(controller);
}

void CalibrationWidget::setSensor(LaserSensor* sensor)
{
    m_sensor = sensor;
    if (m_singleSphere14Widget) m_singleSphere14Widget->setSensor(sensor);
}

void CalibrationWidget::setPointCloudBuilder(PointCloudBuilder* builder)
{
    m_cloudBuilder = builder;
    if (m_singleSphere14Widget) m_singleSphere14Widget->setPointCloudBuilder(builder);
}

void CalibrationWidget::setSyncManager(SyncManager* manager)
{
    if (m_hardwareSyncCalibrationWidget) m_hardwareSyncCalibrationWidget->setSyncManager(manager);
}

void CalibrationWidget::notifyHardwareTemporalScanFinished(quint64 requestId, bool success,
                                                           const QString& message)
{
    if (m_hardwareSyncCalibrationWidget)
        m_hardwareSyncCalibrationWidget->notifyPlannedScanFinished(requestId, success, message);
}

int CalibrationWidget::hardwareTemporalCalibrationTopologyId() const
{
    return m_hardwareSyncCalibrationWidget
        ? m_hardwareSyncCalibrationWidget->selectedTemporalTopologyId() : 1;
}

int CalibrationWidget::hardwareTemporalExpectedR1Edge() const
{
    return m_hardwareSyncCalibrationWidget
        ? m_hardwareSyncCalibrationWidget->expectedCmmR1Edge() : 104;
}

bool CalibrationWidget::setHardwareTemporalRuntimeEnabled(bool enabled, QString* error)
{
    if (!m_hardwareSyncCalibrationWidget) {
        if (error) *error = tr("硬件同步标定组件未初始化");
        return false;
    }
    return m_hardwareSyncCalibrationWidget->setRuntimeCorrectionEnabled(enabled, error);
}

void CalibrationWidget::setSingleSphereSafetyCheckers(
    std::function<bool(const JointPose5D&, QString&)> poseChecker,
    std::function<bool(const JointPose5D&, const JointPose5D&, QString&)> transitionChecker,
    std::function<bool(mls::calibration::PosePlannerConfig&, QString&, QString&)> planningBoundsResolver)
{
    if (m_singleSphere14Widget)
        m_singleSphere14Widget->setStage2SafetyCheckers(
            std::move(poseChecker), std::move(transitionChecker), std::move(planningBoundsResolver));
}

void CalibrationWidget::refreshSingleSphereDeviceState()
{
    if (m_singleSphere14Widget) m_singleSphere14Widget->refreshDeviceState();
}

void CalibrationWidget::shutdownBackgroundWork()
{
    if (m_singleSphere14Widget) m_singleSphere14Widget->shutdownBackgroundWork();
}

void CalibrationWidget::notifySingleSphereJointMoveFinished(bool ok, const QString& message,
                                                             const JointState5D& finalState)
{
    if (m_singleSphere14Widget) {
        m_singleSphere14Widget->notifyJointMoveFinished(ok, message, finalState);
    }
}

void CalibrationWidget::connectSignals()
{
    connect(m_btnAddMeasurement, &QPushButton::clicked,
            this, &CalibrationWidget::onAddUncertaintyMeasurement);
    connect(m_btnStartUncertainty, &QPushButton::clicked,
            this, &CalibrationWidget::onStartUncertaintyCalib);
    connect(m_btnStopUncertainty, &QPushButton::clicked,
            this, &CalibrationWidget::onStopUncertaintyCalib);
    connect(m_btnClearUncertainty, &QPushButton::clicked,
            this, &CalibrationWidget::onClearUncertaintyData);
    connect(m_btnExportUncertainty, &QPushButton::clicked,
            this, &CalibrationWidget::onExportUncertaintyReport);
    connect(m_btnTestUncertainty, &QPushButton::clicked,
            this, &CalibrationWidget::onTestUncertainty);

    connect(m_uncertaintyCalib.get(), &IAlgorithm::progressUpdated, this,
            [this](const AlgorithmProgress& p) {
        onProgressUpdated(p.current, p.total, p.message);
    });
    connect(m_uncertaintyCalib.get(), &IAlgorithm::completed,
            this, &CalibrationWidget::onCalibrationCompleted);
    connect(m_uncertaintyCalib.get(), &IAlgorithm::logMessage,
            this, &CalibrationWidget::appendLog);
}

void CalibrationWidget::updateUIState()
{
    if (!m_btnStartUncertainty || !m_btnStopUncertainty) return;
    m_btnStartUncertainty->setEnabled(!m_isCalibrating);
    m_btnStopUncertainty->setEnabled(m_isCalibrating && m_currentCalibType == "uncertainty");
}

void CalibrationWidget::appendLog(const QString& message)
{
    // Path/Session planning failures are operationally meaningful diagnostics:
    // show them as WARN in the normal system log and exported log files.
    if (message.contains(QStringLiteral("[PATH-REJECT]")) ||
        message.contains(QStringLiteral("[TEMPLATE-REJECT]")) ||
        message.contains(QStringLiteral("[SESSION-RESET-REJECT]")) ||
        message.contains(QStringLiteral("[CENTER-REJECT]"))) {
        LogManager::instance().warning(LogCategory::Calibration, message);
    } else {
        LogManager::instance().info(LogCategory::Calibration, message);
    }
    emit logMessage(message);
}

void CalibrationWidget::onProgressUpdated(int current, int total, const QString& message)
{
    if (m_progressBar) {
        m_progressBar->setMaximum(total);
        m_progressBar->setValue(current);
    }
    if (m_statusLabel) m_statusLabel->setText(message);
}

void CalibrationWidget::onCalibrationCompleted(bool success)
{
    m_isCalibrating = false;
    updateUIState();

    if (m_currentCalibType == "uncertainty" && success) {
        const auto& result = m_uncertaintyCalib->result();
        m_lblUncertaintyResult->setText(
            tr("扩展不确定度 U = %1 mm (k=2)").arg(result.expandedUncertainty, 0, 'f', 4));
    }

    appendLog(tr("%1标定%2").arg(m_currentCalibType, success ? tr("完成") : tr("失败")));
    emit calibrationCompleted(m_currentCalibType, success);
}

void CalibrationWidget::onStartUncertaintyCalib()
{
    m_uncertaintyCalib->initialize();
    m_uncertaintyCalib->setResolution(m_spinResolution->value());
    m_uncertaintyCalib->setSpecifiedAccuracy(m_spinAccuracy->value());
    m_uncertaintyCalib->setTemperatureVariation(m_spinTempVar->value());

    for (int row = 0; row < m_uncertaintyTable->rowCount(); ++row) {
        QVector<double> measurements;
        measurements.append(m_uncertaintyTable->item(row, 1)->text().toDouble());
        const double refValue = m_uncertaintyTable->item(row, 2)->text().toDouble();
        m_uncertaintyCalib->addRepeatabilityData(measurements, refValue);
    }

    m_isCalibrating = true;
    m_currentCalibType = "uncertainty";
    updateUIState();
    appendLog(tr("开始测量不确定度标定..."));
    m_uncertaintyCalib->execute();
}

void CalibrationWidget::onStopUncertaintyCalib()
{
    m_uncertaintyCalib->cancel();
    m_isCalibrating = false;
    updateUIState();
}

void CalibrationWidget::onAddUncertaintyMeasurement()
{
    const int row = m_uncertaintyTable->rowCount();
    m_uncertaintyTable->insertRow(row);
    m_uncertaintyTable->setItem(row, 0, new QTableWidgetItem(QString::number(row + 1)));
    m_uncertaintyTable->setItem(row, 1, new QTableWidgetItem("0.000"));
    m_uncertaintyTable->setItem(row, 2, new QTableWidgetItem("0.000"));
}

void CalibrationWidget::onClearUncertaintyData()
{
    m_uncertaintyTable->setRowCount(0);
    m_uncertaintyCalib->clearData();
}

void CalibrationWidget::onExportUncertaintyReport()
{
    const QString fileName = QFileDialog::getSaveFileName(
        this, tr("导出报告"), QString(), tr("文本文件 (*.txt)"));
    if (fileName.isEmpty()) return;

    QFile file(fileName);
    if (file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QTextStream out(&file);
        out << m_uncertaintyCalib->generateReport();
    }
}

void CalibrationWidget::onTestUncertainty()
{
    appendLog(tr("=== 测试测量不确定度标定 ==="));
    m_uncertaintyCalib->clearData();
    const QVector<double> testData = {
        10.001, 10.002, 9.999, 10.000, 10.001,
        9.998, 10.002, 10.000, 9.999, 10.001
    };
    m_uncertaintyCalib->addRepeatabilityData(testData, 10.0);
    m_isCalibrating = true;
    m_currentCalibType = "uncertainty";
    updateUIState();
    m_uncertaintyCalib->initialize();
    m_uncertaintyCalib->execute();
}

bool CalibrationWidget::scanCalibration(std::array<double,17>& parameters) const
{
    return m_singleSphere14Widget && m_singleSphere14Widget->scanCalibration(parameters);
}

bool CalibrationWidget::sphereStudyPlanningCalibration(
    std::array<double,17>& parameters, QString& source, bool& independentlyValidated) const
{
    if (!m_singleSphere14Widget) {
        source = tr("无Physical17规划模型");
        independentlyValidated = false;
        return false;
    }
    return m_singleSphere14Widget->sphereStudyPlanningCalibration(
        parameters, source, independentlyValidated);
}

bool CalibrationWidget::sphereStudySensorCoordinateConvention(
    bool& flipX, bool& flipZ, QString& source) const
{
    if (!m_singleSphere14Widget) {
        flipX = false;
        flipZ = true;
        source = tr("无单球标定组件，无法读取传感器坐标约定");
        return false;
    }
    return m_singleSphere14Widget->sphereStudySensorCoordinateConvention(
        flipX, flipZ, source);
}

// V27.6.35: do not switch the physical-target transform between automated poses.
bool CalibrationWidget::runtimeCompensationChangeBlocked() const
{
    return m_singleSphere14Widget && m_singleSphere14Widget->runtimeCompensationChangeBlocked();
}
