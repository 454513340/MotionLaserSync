#include "widgets/PathPlanningWidget.h"
#include "widgets/DigitalTwinScanWidget.h"
#include "widgets/MultiPoseExperimentWidget.h"
#include "widgets/PointCloudWidget.h"
#include "algorithms/PathPlanningAlgorithms.h"
#include "core/PointCloudBuilder.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QFileDialog>
#include <QDateTime>
#include <QDialog>
#include <QScrollArea>

PathPlanningWidget::PathPlanningWidget(QWidget* parent)
    : QWidget(parent)
    , m_pathPlanner(std::make_unique<PathPlanner>())
    , m_gcodeGenerator(std::make_unique<GCodeGenerator>())
    , m_pathOptimizer(std::make_unique<PathOptimizer>())
{
    setupUI();
    connectSignals();
    updateUIState();
}

PathPlanningWidget::~PathPlanningWidget() = default;

void PathPlanningWidget::setMainPointCloudView(PointCloudWidget* view) {
    m_mainCloudView=view;
    if(m_digitalTwinWindow)m_digitalTwinWindow->setMainPointCloudView(view);
    if(m_multiPoseExperimentWindow)m_multiPoseExperimentWindow->setMainPointCloudView(view);
}


bool PathPlanningWidget::snapshotOpticalForExactNc(const QString& text,
    mls::twin::TwinScene& scene,std::shared_ptr<const mls::twin::MultiPoseQtReport>& report,
    QString* error) const
{
    if(!m_multiPoseExperimentWindow){
        if(error)*error=tr("请先打开多姿态实验并导入真实STEP/SCAN，生成与G代码文件一致的NC计划");
        return false;
    }
    return m_multiPoseExperimentWindow->snapshotOpticalForExactNc(text,scene,report,error);
}

bool PathPlanningWidget::snapshotCollisionWorkpiece(mls::twin::TwinScene& scene,
    QByteArray& identity, bool& offlinePoseReviewed, QString* error) const
{
    if(!m_multiPoseExperimentWindow){
        if(error)*error=tr("请先打开路径规划→多姿态实验扫描，导入扫描零件STEP并设置正确的CAD位姿");
        return false;
    }
    return m_multiPoseExperimentWindow->snapshotCollisionWorkpiece(
        scene,identity,offlinePoseReviewed,error);
}

void PathPlanningWidget::setupUI()
{
    auto* mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(5, 5, 5, 5);

    // 路径参数
    auto* paramGroup = new QGroupBox(tr("路径参数"));
    auto* paramLayout = new QGridLayout(paramGroup);
    
    paramLayout->addWidget(new QLabel(tr("路径类型:")), 0, 0);
    m_comboPathType = new QComboBox;
    m_comboPathType->addItem(tr("栅格扫描"), 0);
    m_comboPathType->addItem(tr("螺旋扫描"), 1);
    m_comboPathType->addItem(tr("轮廓跟踪"), 2);
    m_comboPathType->addItem(tr("自适应"), 3);
    paramLayout->addWidget(m_comboPathType, 0, 1);
    
    paramLayout->addWidget(new QLabel(tr("扫描方向:")), 0, 2);
    m_comboDirection = new QComboBox;
    m_comboDirection->addItem(tr("X正向"), 0);
    m_comboDirection->addItem(tr("X负向"), 1);
    m_comboDirection->addItem(tr("Y正向"), 2);
    m_comboDirection->addItem(tr("双向"), 4);
    paramLayout->addWidget(m_comboDirection, 0, 3);
    
    paramLayout->addWidget(new QLabel(tr("行间距(mm):")), 1, 0);
    m_spinLineSpacing = new QDoubleSpinBox;
    m_spinLineSpacing->setRange(0.1, 100);
    m_spinLineSpacing->setValue(1.0);
    paramLayout->addWidget(m_spinLineSpacing, 1, 1);
    
    paramLayout->addWidget(new QLabel(tr("点间距(mm):")), 1, 2);
    m_spinPointSpacing = new QDoubleSpinBox;
    m_spinPointSpacing->setRange(0.1, 100);
    m_spinPointSpacing->setValue(0.5);
    paramLayout->addWidget(m_spinPointSpacing, 1, 3);
    
    paramLayout->addWidget(new QLabel(tr("扫描速度(mm/s):")), 2, 0);
    m_spinScanSpeed = new QDoubleSpinBox;
    m_spinScanSpeed->setRange(0.1, 1000);
    m_spinScanSpeed->setValue(10.0);
    paramLayout->addWidget(m_spinScanSpeed, 2, 1);
    
    paramLayout->addWidget(new QLabel(tr("快移速度(mm/s):")), 2, 2);
    m_spinRapidSpeed = new QDoubleSpinBox;
    m_spinRapidSpeed->setRange(1, 5000);
    m_spinRapidSpeed->setValue(50.0);
    paramLayout->addWidget(m_spinRapidSpeed, 2, 3);
    
    paramLayout->addWidget(new QLabel(tr("安全高度(mm):")), 3, 0);
    m_spinSafeHeight = new QDoubleSpinBox;
    m_spinSafeHeight->setRange(1, 100);
    m_spinSafeHeight->setValue(10.0);
    paramLayout->addWidget(m_spinSafeHeight, 3, 1);
    
    paramLayout->addWidget(new QLabel(tr("标称高度(mm):")), 3, 2);
    m_spinNominalHeight = new QDoubleSpinBox;
    m_spinNominalHeight->setRange(50, 150);
    m_spinNominalHeight->setValue(95.0);
    paramLayout->addWidget(m_spinNominalHeight, 3, 3);
    
    m_chkAdaptiveHeight = new QCheckBox(tr("自适应高度"));
    paramLayout->addWidget(m_chkAdaptiveHeight, 4, 0, 1, 2);
    
    mainLayout->addWidget(paramGroup);

    // 边界设置
    auto* boundGroup = new QGroupBox(tr("扫描边界"));
    auto* boundLayout = new QGridLayout(boundGroup);
    
    boundLayout->addWidget(new QLabel(tr("X最小:")), 0, 0);
    m_spinBoundMinX = new QDoubleSpinBox;
    m_spinBoundMinX->setRange(-1000, 1000);
    m_spinBoundMinX->setValue(0);
    boundLayout->addWidget(m_spinBoundMinX, 0, 1);
    
    boundLayout->addWidget(new QLabel(tr("X最大:")), 0, 2);
    m_spinBoundMaxX = new QDoubleSpinBox;
    m_spinBoundMaxX->setRange(-1000, 1000);
    m_spinBoundMaxX->setValue(100);
    boundLayout->addWidget(m_spinBoundMaxX, 0, 3);
    
    boundLayout->addWidget(new QLabel(tr("Y最小:")), 1, 0);
    m_spinBoundMinY = new QDoubleSpinBox;
    m_spinBoundMinY->setRange(-1000, 1000);
    m_spinBoundMinY->setValue(0);
    boundLayout->addWidget(m_spinBoundMinY, 1, 1);
    
    boundLayout->addWidget(new QLabel(tr("Y最大:")), 1, 2);
    m_spinBoundMaxY = new QDoubleSpinBox;
    m_spinBoundMaxY->setRange(-1000, 1000);
    m_spinBoundMaxY->setValue(100);
    boundLayout->addWidget(m_spinBoundMaxY, 1, 3);
    
    m_btnAutobound = new QPushButton(tr("从点云计算"));
    m_btnLoadCloud = new QPushButton(tr("加载点云"));
    boundLayout->addWidget(m_btnAutobound, 2, 0, 1, 2);
    boundLayout->addWidget(m_btnLoadCloud, 2, 2, 1, 2);
    
    mainLayout->addWidget(boundGroup);

    // 控制按钮
    auto* ctrlLayout = new QHBoxLayout;
    m_btnGeneratePath = new QPushButton(tr("生成路径"));
    m_btnGeneratePath->setStyleSheet("background-color: #4CAF50; color: white;");
    m_btnOptimize = new QPushButton(tr("优化路径"));
    m_btnGenerateGCode = new QPushButton(tr("生成G代码"));
    m_btnExportGCode = new QPushButton(tr("导出G代码"));
    m_btnPreview = new QPushButton(tr("预览"));
    m_btnTest = new QPushButton(tr("测试"));
    m_btnTest->setStyleSheet("background-color: #2196F3; color: white;");
    
    ctrlLayout->addWidget(m_btnGeneratePath);
    ctrlLayout->addWidget(m_btnOptimize);
    ctrlLayout->addWidget(m_btnGenerateGCode);
    ctrlLayout->addWidget(m_btnExportGCode);
    ctrlLayout->addWidget(m_btnPreview);
    ctrlLayout->addStretch();
    ctrlLayout->addWidget(m_btnTest);
    mainLayout->addLayout(ctrlLayout);

    // Experimental multi-pose tool.  Place this in the existing Path Planning
    // page directly ABOVE the M0-M8 entry, as requested.  Dialog contents are
    // constructed on demand, never in the MainWindow startup path.
    m_btnMultiPoseExperiment = new QPushButton(tr("多姿态实验扫描 / 规划、仿真与离线 G代码…"), this);
    m_btnMultiPoseExperiment->setObjectName(QStringLiteral("btnOpenMultiPoseExperiment"));
    m_btnMultiPoseExperiment->setToolTip(tr("独立多姿态实验工作流：STEP + 真实 SCAN、指定 C/B、光学仿真、H1/H3 离线 NC；不连接或驱动硬件。"));
    mainLayout->addWidget(m_btnMultiPoseExperiment);

    // Original M0-M8 digital-twin scene editor remains unchanged below.
    m_btnDigitalTwin = new QPushButton(tr("数字孪生 / STEP 工件放置…"),this);
    m_btnDigitalTwin->setToolTip(tr("离线导入 STEP、设置工件位置、M2 实测回放与 M3 虚拟光学扫描；不发送硬件指令。"));
    mainLayout->addWidget(m_btnDigitalTwin);

    // 统计信息
    m_lblStatistics = new QLabel(tr("路径点: 0 | 扫描线: 0 | 总长度: 0mm | 预估时间: 0s"));
    m_lblStatistics->setStyleSheet("font-weight: bold;");
    mainLayout->addWidget(m_lblStatistics);

    // G代码预览
    auto* gcodeGroup = new QGroupBox(tr("G代码预览"));
    auto* gcodeLayout = new QVBoxLayout(gcodeGroup);
    m_gcodePreview = new QTextEdit;
    m_gcodePreview->setReadOnly(true);
    m_gcodePreview->setFontFamily("Courier");
    m_gcodePreview->setMaximumHeight(120);
    gcodeLayout->addWidget(m_gcodePreview);
    mainLayout->addWidget(gcodeGroup);

    // 进度和日志
    auto* statusLayout = new QHBoxLayout;
    m_statusLabel = new QLabel(tr("就绪"));
    m_progressBar = new QProgressBar;
    m_progressBar->setRange(0, 100);
    statusLayout->addWidget(m_statusLabel);
    statusLayout->addWidget(m_progressBar, 1);
    mainLayout->addLayout(statusLayout);

    auto* logGroup = new QGroupBox(tr("日志"));
    auto* logLayout = new QVBoxLayout(logGroup);
    m_logView = new QTextEdit;
    m_logView->setReadOnly(true);
    m_logView->setMaximumHeight(80);
    logLayout->addWidget(m_logView);
    mainLayout->addWidget(logGroup);
}

void PathPlanningWidget::connectSignals()
{
    connect(m_btnGeneratePath, &QPushButton::clicked, this, &PathPlanningWidget::onGeneratePath);
    connect(m_btnOptimize, &QPushButton::clicked, this, &PathPlanningWidget::onOptimizePath);
    connect(m_btnGenerateGCode, &QPushButton::clicked, this, &PathPlanningWidget::onGenerateGCode);
    connect(m_btnExportGCode, &QPushButton::clicked, this, &PathPlanningWidget::onExportGCode);
    connect(m_btnPreview, &QPushButton::clicked, this, &PathPlanningWidget::onPreviewPath);
    connect(m_btnLoadCloud, &QPushButton::clicked, this, &PathPlanningWidget::onLoadPointCloud);
    connect(m_btnTest, &QPushButton::clicked, this, &PathPlanningWidget::onTestPathPlanning);
    connect(m_btnDigitalTwin, &QPushButton::clicked, this, &PathPlanningWidget::onOpenDigitalTwin);
    connect(m_btnMultiPoseExperiment, &QPushButton::clicked,
            this, &PathPlanningWidget::onOpenMultiPoseExperiment);

    connect(m_pathPlanner.get(), &IAlgorithm::progressUpdated, this,
            [this](const AlgorithmProgress& p) { onProgressUpdated(p.current, p.total, p.message); });
    connect(m_pathPlanner.get(), &IAlgorithm::completed, this, &PathPlanningWidget::onPlanningCompleted);
    connect(m_pathPlanner.get(), &IAlgorithm::logMessage, this, &PathPlanningWidget::appendLog);
}

void PathPlanningWidget::updateUIState()
{
    m_btnGeneratePath->setEnabled(!m_isProcessing);
    m_btnOptimize->setEnabled(!m_isProcessing && m_pathPlanner->path().isValid);
    m_btnGenerateGCode->setEnabled(!m_isProcessing && m_pathPlanner->path().isValid);
    m_btnExportGCode->setEnabled(!m_isProcessing && !m_gcodeGenerator->gcode().isEmpty());
}

void PathPlanningWidget::appendLog(const QString& message)
{
    QString timestamp = QDateTime::currentDateTime().toString("hh:mm:ss");
    m_logView->append(QString("[%1] %2").arg(timestamp, message));
    emit logMessage(message);
}

void PathPlanningWidget::updatePathStatistics()
{
    const auto& path = m_pathPlanner->path();
    m_lblStatistics->setText(tr("路径点: %1 | 扫描线: %2 | 总长度: %3mm | 预估时间: %4s")
        .arg(path.points.size())
        .arg(path.lineCount)
        .arg(path.totalLength, 0, 'f', 1)
        .arg(path.estimatedTime, 0, 'f', 1));
}

void PathPlanningWidget::onProgressUpdated(int current, int total, const QString& message)
{
    m_progressBar->setMaximum(total);
    m_progressBar->setValue(current);
    m_statusLabel->setText(message);
}

void PathPlanningWidget::onPlanningCompleted(bool success)
{
    m_isProcessing = false;
    updateUIState();
    
    if (success) {
        updatePathStatistics();
        appendLog(tr("路径生成完成"));
    } else {
        appendLog(tr("路径生成失败: %1").arg(m_pathPlanner->lastError()));
    }
}

void PathPlanningWidget::onGeneratePath()
{
    PathPlanParams params;
    params.pathType = static_cast<ScanPathType>(m_comboPathType->currentIndex());
    params.direction = static_cast<ScanDirection>(m_comboDirection->currentData().toInt());
    params.lineSpacing = m_spinLineSpacing->value();
    params.pointSpacing = m_spinPointSpacing->value();
    params.scanSpeed = m_spinScanSpeed->value();
    params.rapidSpeed = m_spinRapidSpeed->value();
    params.safeHeight = m_spinSafeHeight->value();
    params.nominalHeight = m_spinNominalHeight->value();
    params.adaptiveHeight = m_chkAdaptiveHeight->isChecked();
    
    m_pathPlanner->setParameters(params);
    m_pathPlanner->setBoundary(
        QVector3D(m_spinBoundMinX->value(), m_spinBoundMinY->value(), 0),
        QVector3D(m_spinBoundMaxX->value(), m_spinBoundMaxY->value(), 100)
    );
    
    m_isProcessing = true;
    updateUIState();
    appendLog(tr("开始生成路径..."));
    
    m_pathPlanner->initialize();
    m_pathPlanner->execute();
}

void PathPlanningWidget::onOptimizePath()
{
    m_pathOptimizer->setPath(m_pathPlanner->path());
    m_pathOptimizer->initialize();
    
    appendLog(tr("开始优化路径..."));
    if (m_pathOptimizer->execute()) {
        appendLog(tr("优化完成, 改进: %1%").arg(m_pathOptimizer->improvementPercentage(), 0, 'f', 1));
    }
}

void PathPlanningWidget::onGenerateGCode()
{
    m_gcodeGenerator->setPath(m_pathPlanner->path());
    m_gcodeGenerator->setGenerateScanCommands(true);
    m_gcodeGenerator->initialize();
    
    appendLog(tr("生成G代码..."));
    if (m_gcodeGenerator->execute()) {
        m_gcodePreview->setText(m_gcodeGenerator->gcode());
        appendLog(tr("G代码生成完成: %1 行").arg(m_gcodeGenerator->lineCount()));
        emit pathGenerated(m_gcodeGenerator->gcode());
    }
    updateUIState();
}

void PathPlanningWidget::onExportGCode()
{
    QString fileName = QFileDialog::getSaveFileName(this, tr("导出G代码"), 
        QString(), tr("G代码文件 (*.nc *.gcode *.txt)"));
    if (!fileName.isEmpty()) {
        if (m_gcodeGenerator->saveToFile(fileName)) {
            appendLog(tr("G代码已导出: %1").arg(fileName));
        }
    }
}

void PathPlanningWidget::onPreviewPath()
{
    appendLog(tr("路径预览功能待实现"));
}

void PathPlanningWidget::onLoadPointCloud()
{
    appendLog(tr("加载点云功能待实现"));
}

void PathPlanningWidget::onTestPathPlanning()
{
    appendLog(tr("=== 测试路径规划 ==="));
    
    m_spinBoundMinX->setValue(0);
    m_spinBoundMinY->setValue(0);
    m_spinBoundMaxX->setValue(50);
    m_spinBoundMaxY->setValue(50);
    m_spinLineSpacing->setValue(2.0);
    m_spinPointSpacing->setValue(1.0);
    
    onGeneratePath();
}

void PathPlanningWidget::onOpenDigitalTwin()
{
    if(m_digitalTwinWindow) {
        m_digitalTwinWindow->show();
        m_digitalTwinWindow->raise();
        m_digitalTwinWindow->activateWindow();
        return;
    }
    auto* window=new DigitalTwinScanWidget(this);
    // Keep the scene editor alive when hidden: placement, active CAD model and
    // replay survive until PathPlanningWidget/MainWindow is destroyed.
    window->setMainPointCloudView(m_mainCloudView);
    m_digitalTwinWindow=window;
    window->show();
    appendLog(tr("已打开离线数字孪生工作台（V28 M0+M1 修复 + M2 回放 + M3 虚拟扫描）；主视图与工件位姿同步。"));
}

void PathPlanningWidget::onOpenMultiPoseExperiment()
{
    // The long-lived, non-modal dialog is lazy-created on the first click.
    // Opening the main application no longer constructs MultiPoseExperimentWidget.
    if (m_multiPoseExperimentDialog) {
        m_multiPoseExperimentDialog->show();
        m_multiPoseExperimentDialog->raise();
        m_multiPoseExperimentDialog->activateWindow();
        return;
    }

    auto* window = new QDialog(this);
    window->setObjectName(QStringLiteral("multiPoseExperimentWindow"));
    window->setWindowTitle(tr("多姿态实验扫描 - M3/M4 受控实验（仅离线）"));
    window->setModal(false);
    window->resize(940, 780);
    window->setMinimumSize(680, 480);

    auto* layout = new QVBoxLayout(window);
    layout->setContentsMargins(4, 4, 4, 4);
    auto* scroll = new QScrollArea(window);
    scroll->setWidgetResizable(true);
    scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    auto* experiment = new MultiPoseExperimentWidget(scroll);
    scroll->setWidget(experiment);
    layout->addWidget(scroll);

    experiment->setMainPointCloudView(m_mainCloudView);
    m_multiPoseExperimentWindow = experiment;
    m_multiPoseExperimentDialog = window;
    window->show();
    appendLog(tr("已打开多姿态实验扫描（独立离线工具）；原 M0–M8 完整覆盖流程未受影响。"));
}
