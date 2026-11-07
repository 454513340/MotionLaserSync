#include <QStringList>
#include <algorithm>
#include "widgets/GCodeWidget.h"
#include "gcode/GCodeExecutor.h"
#include "gcode/GCodeParser.h"
#include "gcode/GCodeSyncMetadata.h"
#include "twin/MultiPoseNcSimulation.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QSplitter>
#include <QGroupBox>
#include <QFileDialog>
#include <QMessageBox>
#include <QMenu>
#include <QTextBlock>
#include <QScrollBar>
#include <QHeaderView>
#include <QDateTime>
#include <QTextCharFormat>
#include <QFrame>
#include <QDoubleSpinBox>
#include <QDebug>
#include <QCoreApplication>
#include <QFile>
#include <QFileInfo>
#include <QCryptographicHash>
#include <QJsonObject>
#include <QJsonArray>
#include <QJsonDocument>
#include <QTextDocument>
#include <QStandardPaths>
#include <QDir>
#include <QProgressDialog>
#include <QSaveFile>

GCodeWidget::GCodeWidget(QWidget* parent)
    : QWidget(parent)
{
    m_parser = new GCodeParser(this);
    
    setupUI();
    setupConnections();
    
    m_updateTimer.setInterval(200);
    connect(&m_updateTimer, &QTimer::timeout, this, &GCodeWidget::onUpdateTimer);
}

GCodeWidget::~GCodeWidget()
{
}

void GCodeWidget::setupUI()
{
    auto* mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(5, 5, 5, 5);
    mainLayout->setSpacing(5);
    
    // ===== 工具栏 =====
    auto* toolbarLayout = new QHBoxLayout();
    
    m_btnLoad = new QPushButton(tr("加载文件"));
    m_btnSave = new QPushButton(tr("保存文件"));
    m_btnParse = new QPushButton(tr("解析验证"));
    // V30: one existing preflight entry now orchestrates the three real checks.
    // It triggers actual sensor/controller acquisition, so do not label it read-only.
    m_btnRealPreflight = new QPushButton(tr("实机状态诊断（含硬件试采）"));
    m_btnRealPreflight->setToolTip(tr("一次完成NC逐段验证、硬件同步试采、整条NC+STEP碰撞复核及现场测量验收；诊断本身不执行NC轴运动"));
    m_btnRetrySiteAcceptance=new QPushButton(tr("第4步：现场测量验收"));
    m_btnRetrySiteAcceptance->setToolTip(tr("第1~3步已通过时，补交实测工件位姿和夹具/防护验收；无需再次进行耗时的整条FCL检查"));
    m_btnInsertTemplate = new QPushButton(tr("插入模板"));
    m_btnClear = new QPushButton(tr("清空"));
    
    toolbarLayout->addWidget(m_btnLoad);
    toolbarLayout->addWidget(m_btnSave);
    toolbarLayout->addWidget(m_btnParse);
    toolbarLayout->addWidget(m_btnRealPreflight);
    toolbarLayout->addWidget(m_btnRetrySiteAcceptance);
    toolbarLayout->addWidget(m_btnInsertTemplate);
    toolbarLayout->addWidget(m_btnClear);
    toolbarLayout->addStretch();
    
    m_lineCountLabel = new QLabel(tr("行数: 0"));
    m_statusLabel = new QLabel(tr("状态: 未解析"));
    toolbarLayout->addWidget(m_lineCountLabel);
    toolbarLayout->addWidget(m_statusLabel);
    
    mainLayout->addLayout(toolbarLayout);
    
    // ===== 主分割器 =====
    auto* mainSplitter = new QSplitter(Qt::Vertical);
    
    // ----- 上半部分：编辑器和程序段表 -----
    auto* topSplitter = new QSplitter(Qt::Horizontal);
    
    // 编辑器
    auto* editorGroup = new QGroupBox(tr("G代码编辑器"));
    auto* editorLayout = new QVBoxLayout(editorGroup);
    editorLayout->setContentsMargins(5, 5, 5, 5);
    
    m_editor = new QPlainTextEdit();
    m_editor->setFont(QFont("Consolas", 10));
    m_editor->setLineWrapMode(QPlainTextEdit::NoWrap);
    m_editor->setTabStopDistance(40);
    m_editor->setPlaceholderText(tr(
        "; 在此输入G代码程序\n"
        "; 或点击\"加载文件\"导入\n"
        "; 或点击\"插入模板\"生成示例\n"
        ";\n"
        "; 基本指令:\n"
        ";   G00 X Y Z C B - 五轴关节定位\n"
        ";   G01 X Y Z C B F - 五轴关节插补\n"
        ";   M50          - 开始采集\n"
        ";   M51          - 停止采集\n"
        ";   M60 L\"name\" - 保存点云\n"
        ";   M61          - 清空数据\n"
        ";   M30          - 程序结束\n"
    ));
    
    editorLayout->addWidget(m_editor);
    topSplitter->addWidget(editorGroup);
    
    // 程序段表格
    auto* tableGroup = new QGroupBox(tr("程序段列表"));
    auto* tableLayout = new QVBoxLayout(tableGroup);
    tableLayout->setContentsMargins(5, 5, 5, 5);
    
    m_blockTable = new QTableWidget();
    m_blockTable->setColumnCount(5);
    m_blockTable->setHorizontalHeaderLabels({tr("行号"), tr("指令"), tr("参数"), tr("采集"), tr("注释")});
    m_blockTable->horizontalHeader()->setStretchLastSection(true);
    m_blockTable->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_blockTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_blockTable->setAlternatingRowColors(true);
    m_blockTable->verticalHeader()->setDefaultSectionSize(24);
    
    tableLayout->addWidget(m_blockTable);
    topSplitter->addWidget(tableGroup);
    
    topSplitter->setSizes({400, 300});
    mainSplitter->addWidget(topSplitter);
    
    // ----- 下半部分：控制和日志 -----
    auto* bottomWidget = new QWidget();
    auto* bottomLayout = new QHBoxLayout(bottomWidget);
    bottomLayout->setContentsMargins(0, 0, 0, 0);
    
    // 控制面板
    auto* controlGroup = new QGroupBox(tr("执行控制"));
    auto* controlLayout = new QVBoxLayout(controlGroup);
    controlLayout->setSpacing(8);
    
    // ===== 运动参数设置 =====
    // 注意：默认值参考 AxisParams 的实际控制器参数
    auto* motionParamsLayout = new QGridLayout();
    motionParamsLayout->setSpacing(5);
    
    // 进给速度（与AxisParams.speed一致）
    motionParamsLayout->addWidget(new QLabel(tr("进给速度(F):")), 0, 0);
    m_spinFeedSpeed = new QDoubleSpinBox();
    m_spinFeedSpeed->setRange(0.1, 100.0);
    m_spinFeedSpeed->setValue(5.0);          // 与AxisParams.speed一致
    m_spinFeedSpeed->setSuffix(" mm/s");
    m_spinFeedSpeed->setDecimals(2);
    m_spinFeedSpeed->setToolTip(tr("G01/G02/G03/G05 默认进给速度"));
    motionParamsLayout->addWidget(m_spinFeedSpeed, 0, 1);
    
    // 快速速度（空行程移动）
    motionParamsLayout->addWidget(new QLabel(tr("快速速度:")), 0, 2);
    m_spinRapidSpeed = new QDoubleSpinBox();
    m_spinRapidSpeed->setRange(0.1, 200.0);
    m_spinRapidSpeed->setValue(10.0);        // 快速速度，进给速度的2倍
    m_spinRapidSpeed->setSuffix(" mm/s");
    m_spinRapidSpeed->setDecimals(2);
    m_spinRapidSpeed->setToolTip(tr("G00 快速定位速度"));
    motionParamsLayout->addWidget(m_spinRapidSpeed, 0, 3);
    
    // 加速度（与AxisParams.accel一致）
    motionParamsLayout->addWidget(new QLabel(tr("加速度:")), 1, 0);
    m_spinAccel = new QDoubleSpinBox();
    m_spinAccel->setRange(1.0, 1000.0);
    m_spinAccel->setValue(25.0);             // 与AxisParams.accel一致
    m_spinAccel->setSuffix(" mm/s²");
    m_spinAccel->setDecimals(1);
    motionParamsLayout->addWidget(m_spinAccel, 1, 1);
    
    // 减速度（与AxisParams.decel一致）
    motionParamsLayout->addWidget(new QLabel(tr("减速度:")), 1, 2);
    m_spinDecel = new QDoubleSpinBox();
    m_spinDecel->setRange(1.0, 1000.0);
    m_spinDecel->setValue(25.0);             // 与AxisParams.decel一致
    m_spinDecel->setSuffix(" mm/s²");
    m_spinDecel->setDecimals(1);
    motionParamsLayout->addWidget(m_spinDecel, 1, 3);
    
    controlLayout->addLayout(motionParamsLayout);
    
    // 分隔线
    auto* line1 = new QFrame();
    line1->setFrameShape(QFrame::HLine);
    line1->setFrameShadow(QFrame::Sunken);
    controlLayout->addWidget(line1);
    
    // 保存设置
    auto* saveSettingsLayout = new QGridLayout();
    saveSettingsLayout->addWidget(new QLabel(tr("保存目录:")), 0, 0);
    m_editSaveDir = new QLineEdit("./scan_data");
    m_btnBrowseSaveDir = new QPushButton("...");
    m_btnBrowseSaveDir->setFixedWidth(30);
    saveSettingsLayout->addWidget(m_editSaveDir, 0, 1);
    saveSettingsLayout->addWidget(m_btnBrowseSaveDir, 0, 2);
    
    saveSettingsLayout->addWidget(new QLabel(tr("文件前缀:")), 1, 0);
    m_editSavePrefix = new QLineEdit("scan");
    saveSettingsLayout->addWidget(m_editSavePrefix, 1, 1, 1, 2);
    
    controlLayout->addLayout(saveSettingsLayout);
    
    // 控制按钮
    auto* btnLayout = new QHBoxLayout();
    m_btnStart = new QPushButton(tr("开始执行"));
    m_btnPauseResume = new QPushButton(tr("暂停"));
    m_btnStop = new QPushButton(tr("停止"));
    m_btnStep = new QPushButton(tr("单步"));
    
    m_btnStart->setMinimumHeight(35);
    m_btnPauseResume->setMinimumHeight(35);
    m_btnStop->setMinimumHeight(35);
    m_btnStep->setMinimumHeight(35);
    
    m_btnStart->setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-weight: bold; }");
    m_btnStop->setStyleSheet("QPushButton { background-color: #f44336; color: white; }");
    
    btnLayout->addWidget(m_btnStart);
    btnLayout->addWidget(m_btnPauseResume);
    btnLayout->addWidget(m_btnStop);
    btnLayout->addWidget(m_btnStep);
    controlLayout->addLayout(btnLayout);
    
    // 进度显示
    auto* progressLayout = new QGridLayout();
    
    progressLayout->addWidget(new QLabel(tr("进度:")), 0, 0);
    m_progressBar = new QProgressBar();
    m_progressBar->setRange(0, 100);
    progressLayout->addWidget(m_progressBar, 0, 1, 1, 3);
    
    progressLayout->addWidget(new QLabel(tr("状态:")), 1, 0);
    m_lblState = new QLabel(tr("空闲"));
    m_lblState->setStyleSheet("font-weight: bold;");
    progressLayout->addWidget(m_lblState, 1, 1);
    
    progressLayout->addWidget(new QLabel(tr("当前段:")), 1, 2);
    m_lblCurrentBlock = new QLabel("-");
    progressLayout->addWidget(m_lblCurrentBlock, 1, 3);
    
    progressLayout->addWidget(new QLabel(tr("耗时:")), 2, 0);
    m_lblElapsedTime = new QLabel("00:00:00");
    progressLayout->addWidget(m_lblElapsedTime, 2, 1);
    
    progressLayout->addWidget(new QLabel(tr("轮廓数:")), 2, 2);
    m_lblProfileCount = new QLabel("0");
    progressLayout->addWidget(m_lblProfileCount, 2, 3);
    
    progressLayout->addWidget(new QLabel(tr("点云数:")), 3, 0);
    m_lblPointCount = new QLabel("0");
    progressLayout->addWidget(m_lblPointCount, 3, 1);
    
    controlLayout->addLayout(progressLayout);
    controlLayout->addStretch();
    
    bottomLayout->addWidget(controlGroup);
    
    // 日志面板
    auto* logGroup = new QGroupBox(tr("执行日志"));
    auto* logLayout = new QVBoxLayout(logGroup);
    logLayout->setContentsMargins(5, 5, 5, 5);
    
    m_logView = new QPlainTextEdit();
    m_logView->setReadOnly(true);
    m_logView->setFont(QFont("Consolas", 9));
    m_logView->setMaximumBlockCount(1000);
    
    logLayout->addWidget(m_logView);
    bottomLayout->addWidget(logGroup);
    
    bottomLayout->setStretch(0, 1);
    bottomLayout->setStretch(1, 1);
    
    mainSplitter->addWidget(bottomWidget);
    mainSplitter->setSizes({300, 200});
    
    mainLayout->addWidget(mainSplitter);
    
    // 初始按钮状态
    updateButtonStates();
}

void GCodeWidget::setupConnections()
{
    // 工具栏按钮
    connect(m_btnLoad, &QPushButton::clicked, this, &GCodeWidget::onLoadFileClicked);
    connect(m_btnSave, &QPushButton::clicked, this, &GCodeWidget::onSaveFileClicked);
    connect(m_btnParse, &QPushButton::clicked, this, &GCodeWidget::onParseClicked);
    connect(m_btnRealPreflight, &QPushButton::clicked, this, &GCodeWidget::onRealRunPreflightClicked);
    connect(m_btnInsertTemplate, &QPushButton::clicked, this, &GCodeWidget::onInsertTemplateClicked);
    connect(m_btnClear, &QPushButton::clicked, this, &GCodeWidget::onClearClicked);
    
    // 控制按钮
    connect(m_btnStart, &QPushButton::clicked, this, &GCodeWidget::onStartClicked);
    connect(m_btnPauseResume, &QPushButton::clicked, this, &GCodeWidget::onPauseResumeClicked);
    connect(m_btnStop, &QPushButton::clicked, this, &GCodeWidget::onStopClicked);
    connect(m_btnStep, &QPushButton::clicked, this, &GCodeWidget::onStepClicked);
    connect(m_btnRetrySiteAcceptance,&QPushButton::clicked,
        this,&GCodeWidget::onRetrySiteAcceptanceClicked);
    
    // 浏览目录
    connect(m_btnBrowseSaveDir, &QPushButton::clicked, this, [this]() {
        QString dir = QFileDialog::getExistingDirectory(this, tr("选择保存目录"),
                                                         m_editSaveDir->text());
        if (!dir.isEmpty()) {
            m_editSaveDir->setText(dir);
        }
    });
    
    // 编辑器文本改变
    connect(m_editor, &QPlainTextEdit::textChanged, this, &GCodeWidget::onEditorTextChanged);
    
    // 表格点击同步到编辑器
    connect(m_blockTable, &QTableWidget::cellClicked, this, [this](int row, int) {
        if (row >= 0 && row < static_cast<int>(m_program.blocks.size())) {
            int lineNum = m_program.blocks[row].lineNumber;
            if (lineNum > 0) {
                highlightCurrentLine(lineNum);
            }
        }
    });
}

void GCodeWidget::setExecutor(GCodeExecutor* executor)
{
    // 断开旧连接
    if (m_executor) {
        disconnect(m_executor, nullptr, this, nullptr);
    }
    
    m_executor = executor;
    
    if (m_executor) {
        connect(m_executor, &GCodeExecutor::stateChanged,
                this, &GCodeWidget::onExecutorStateChanged);
        connect(m_executor, &GCodeExecutor::blockStarted,
                this, &GCodeWidget::onBlockStarted);
        connect(m_executor, &GCodeExecutor::blockCompleted,
                this, &GCodeWidget::onBlockCompleted);
        connect(m_executor, &GCodeExecutor::progressUpdated,
                this, &GCodeWidget::onProgressUpdated);
        connect(m_executor, &GCodeExecutor::programCompleted,
                this, &GCodeWidget::onProgramCompleted);
        connect(m_executor, &GCodeExecutor::programStopped,
                this, &GCodeWidget::onProgramStopped);
        connect(m_executor, &GCodeExecutor::errorOccurred,
                this, &GCodeWidget::onErrorOccurred);
        connect(m_executor, &GCodeExecutor::warningOccurred,
                this, &GCodeWidget::onWarningOccurred);
        connect(m_executor, &GCodeExecutor::progressMessage,
                this, &GCodeWidget::onProgressMessage);
    }
    
    updateButtonStates();
}

void GCodeWidget::setSensorParams(const SensorParams& params)
{
    m_sensorParams = params;
}

void GCodeWidget::setSyncMode(SyncMode mode)
{
    m_syncMode = mode;
}

void GCodeWidget::setHardwareSyncUiIntent(int topology,int frequencyHz,int cmmEdge,
                                           bool manualCaptureHardwareSelected)
{
    m_hardwareUiAvailable = topology == 1 || topology == 3;
    m_hardwareUiSetting = {topology,frequencyHz,cmmEdge};
    m_manualHardwareSelected = manualCaptureHardwareSelected;
}

void GCodeWidget::loadGCode(const QString& text)
{
    m_lastFilePath.clear();
    m_editor->setPlainText(text);
    m_programValid = false;
    m_statusLabel->setText(tr("状态: 未解析"));
}

bool GCodeWidget::loadFromFile(const QString& filePath)
{
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        QMessageBox::warning(this, tr("加载失败"),
                            tr("无法打开文件: %1").arg(file.errorString()));
        return false;
    }
    
    QString text = QString::fromUtf8(file.readAll());
    file.close();
    
    m_editor->setPlainText(text);
    m_lastFilePath = filePath;
    m_programValid = false;
    
    appendLog(tr("已加载文件: %1").arg(filePath), "blue");
    
    // 自动解析
    parseAndValidate();
    
    return true;
}

QString GCodeWidget::getGCodeText() const
{
    return m_editor->toPlainText();
}

void GCodeWidget::clearEditor()
{
    m_lastFilePath.clear();
    m_editor->clear();
    m_blockTable->setRowCount(0);
    m_program = GCodeProgram();
    m_programValid = false;
    m_statusLabel->setText(tr("状态: 未解析"));
    m_lineCountLabel->setText(tr("行数: 0"));
}

void GCodeWidget::setReadOnly(bool readOnly)
{
    m_editor->setReadOnly(readOnly);
    m_btnLoad->setEnabled(!readOnly);
    m_btnInsertTemplate->setEnabled(!readOnly);
    m_btnClear->setEnabled(!readOnly);
}

bool GCodeWidget::isExecuting() const
{
    return m_executing;
}

// ==================== 槽函数 ====================

void GCodeWidget::onLoadFileClicked()
{
    QString filePath = QFileDialog::getOpenFileName(
        this, tr("加载G代码文件"),
        m_lastFilePath.isEmpty() ? QString() : QFileInfo(m_lastFilePath).absolutePath(),
        tr("G代码文件 (*.nc *.gcode *.txt *.csv);;所有文件 (*)")
    );
    
    if (!filePath.isEmpty()) {
        loadFromFile(filePath);
    }
}

void GCodeWidget::onSaveFileClicked()
{
    QString filePath = QFileDialog::getSaveFileName(
        this, tr("保存G代码文件"),
        m_lastFilePath.isEmpty() ? "program.nc" : m_lastFilePath,
        tr("G代码文件 (*.nc *.gcode *.txt);;CSV格式 (*.csv);;所有文件 (*)")
    );
    
    if (filePath.isEmpty()) return;
    
    QFile file(filePath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QMessageBox::warning(this, tr("保存失败"),
                            tr("无法写入文件: %1").arg(file.errorString()));
        return;
    }
    
    QString content;
    if (filePath.endsWith(".csv", Qt::CaseInsensitive) && m_programValid) {
        content = m_parser->exportToCSV(m_program);
    } else {
        content = m_editor->toPlainText();
    }
    
    file.write(content.toUtf8());
    file.close();
    
    m_lastFilePath = filePath;
    if(!filePath.endsWith(".csv",Qt::CaseInsensitive))
        m_editor->document()->setModified(false);
    appendLog(tr("已保存文件: %1").arg(filePath), "blue");
}

void GCodeWidget::onParseClicked()
{
    parseAndValidate();
}

void GCodeWidget::onInsertTemplateClicked()
{
    QMenu menu(this);
    menu.addAction(tr("基础扫描模板"), this, [this]() {
        m_lastFilePath.clear();
        m_editor->setPlainText(generateTemplate("basic"));
        parseAndValidate();
    });
    menu.addAction(tr("多区域扫描模板"), this, [this]() {
        m_lastFilePath.clear();
        m_editor->setPlainText(generateTemplate("multiregion"));
        parseAndValidate();
    });
    menu.addAction(tr("栅格扫描模板"), this, [this]() {
        m_lastFilePath.clear();
        m_editor->setPlainText(generateTemplate("raster"));
        parseAndValidate();
    });
    menu.addAction(tr("连续轨迹模板"), this, [this]() {
        m_lastFilePath.clear();
        m_editor->setPlainText(generateTemplate("continuous"));
        parseAndValidate();
    });
    
    menu.exec(m_btnInsertTemplate->mapToGlobal(QPoint(0, m_btnInsertTemplate->height())));
}

void GCodeWidget::onClearClicked()
{
    if (m_editor->toPlainText().isEmpty()) return;
    
    int ret = QMessageBox::question(this, tr("确认清空"),
                                   tr("确定要清空编辑器内容吗？"),
                                   QMessageBox::Yes | QMessageBox::No);
    if (ret == QMessageBox::Yes) {
        clearEditor();
    }
}

// V30: a single asynchronous, fail-closed preflight pipeline. Only the probe
// starts sensor PWM/triggers. Nothing in this pipeline issues NC axis movement.
// All three checks operate on the same saved file/parsed NC identity.
bool GCodeWidget::preflightSourceUnchanged(QString* failure) const
{
    if(m_preflightSourcePath.isEmpty() || m_preflightNcText.isEmpty()){
        if(failure)*failure=tr("没有可验证的原始NC文件");
        return false;
    }
    if(m_lastFilePath!=m_preflightSourcePath ||
       m_editor->document()->isModified() ||
       m_editor->toPlainText()!=m_preflightNcText){
        if(failure)*failure=tr("验证期间编辑器的NC内容或源路径已改变");
        return false;
    }
    QFile file(m_preflightSourcePath);
    if(!file.open(QIODevice::ReadOnly)){
        if(failure)*failure=tr("NC源文件无法重新读取：%1").arg(file.errorString());
        return false;
    }
    const QByteArray bytes=file.readAll();
    if(QCryptographicHash::hash(bytes,QCryptographicHash::Sha256)!=m_preflightSourceDigest){
        if(failure)*failure=tr("验证期间磁盘NC文件字节发生变化");
        return false;
    }
    QString decoded=QString::fromUtf8(bytes);
    decoded.replace(QStringLiteral("\r\n"),QStringLiteral("\n"));
    if(decoded!=m_preflightNcText){
        if(failure)*failure=tr("磁盘NC与参与验证的解析文本不一致");
        return false;
    }
    return true;
}

void GCodeWidget::onRealRunPreflightClicked()
{
    if(m_preflightActive || m_executing || m_probeRunning)return;
    if(!m_executor){
        QMessageBox::warning(this,tr("实机状态诊断"),tr("G代码执行器未初始化"));return;
    }
    // A new diagnosis invalidates ALL previous site evidence, even if this
    // attempt fails at parsing or the first stage. Never reuse old PASS.
    m_preflightQualified=false;
    m_preflightStages123Passed=false;
    emit requestInvalidateNcPreflightEvidence();
    parseAndValidate();
    if(!m_programValid){
        QMessageBox::warning(this,tr("实机状态诊断"),tr("NC解析验证失败；具体错误见日志"));return;
    }
    if(m_lastFilePath.isEmpty() || m_editor->document()->isModified()){
        QMessageBox::warning(this,tr("实机状态诊断"),
            tr("请保存原始NC并重新从文件载入。三个阶段必须验证同一份实际磁盘NC。"));return;
    }
    const QString text=m_editor->toPlainText();
    QFile file(m_lastFilePath);
    if(!file.open(QIODevice::ReadOnly)){
        QMessageBox::warning(this,tr("实机状态诊断"),file.errorString());return;
    }
    const QByteArray diskBytes=file.readAll();
    QString diskText=QString::fromUtf8(diskBytes);
    diskText.replace(QStringLiteral("\r\n"),QStringLiteral("\n"));
    if(diskText!=text){
        QMessageBox::warning(this,tr("实机状态诊断"),tr("磁盘NC内容已变化，请重新载入后再诊断"));return;
    }
    // V30.0.2: H1 is an external sensor rising edge; H3 is an internal
    // sensor clock with independent CMM/R1 latch edge. Topology, not a stale
    // or misleading legacy header, defines the actual sensor trigger source.
    QStringList metadataProblems;
    if(m_program.syncMode!=SyncMode::Hardware)
        metadataProblems << tr("同步模式：需要 ; SyncMode: Hardware");
    if(m_program.hardwareTopologyId!=1 && m_program.hardwareTopologyId!=3)
        metadataProblems << tr("同步拓扑：需要 ; HardwareTopology: H1 或 H3");
    if(m_program.hardwareFrequencyHz<1 || m_program.hardwareFrequencyHz>2000)
        metadataProblems << tr("触发频率：需要 ; HardwareFrequencyHz: <实际频率Hz>（1..2000）");
    if(m_program.programFrame!=QStringLiteral("MACHINE_JOINTS_XYZCB"))
        metadataProblems << tr("机床坐标系：需要 ; ProgramFrame: MACHINE_JOINTS_XYZCB");
    QString triggerProblem;
    if(!mls::gcode::validateSensorTriggerMetadata(m_program,&triggerProblem))
        metadataProblems << tr("传感器触发方式与拓扑冲突：%1").arg(triggerProblem);
    // H3 CMM latch edge is separate from an H1 external scanCONTROL edge.
    if(m_program.hardwareTopologyId==3 && (m_program.cmmR1Edge!=114 && m_program.cmmR1Edge!=115))
        metadataProblems << tr("H3锁存沿：需要 ; CmmR1Edge: 114 或 115");
    if(!metadataProblems.isEmpty()){
        const QString detail=tr("此NC缺少或不符合以下声明：\n%1\n\n"
           "H1采用传感器外触发EdgeRising和IN0/R0；H3采用传感器内部周期及IN1/R1。"
           "旧NC没有SensorTriggerMode时按明确的HardwareTopology确定传感器触发方式，"
           "仍然必须经过实际同步试采及FCL复核；不能靠修改OfflineOnly标记放行。")
           .arg(metadataProblems.join(QStringLiteral("\n")));
        appendLog(tr("[V30.0.2 综合诊断] NC头部校验未通过：%1")
            .arg(metadataProblems.join(QStringLiteral("；"))),"red");
        QMessageBox::warning(this,tr("实机状态诊断：NC头部参数"),detail);
        return;
    }

    if(m_program.hardwareTopologyId==3 && !m_program.hardwareTriggerType.isEmpty())
        appendLog(tr("[V30.0.2] 旧H3 NC的HardwareTriggerType字段已废弃；H3实际是传感器内部周期，锁存沿使用CmmR1Edge"),"orange");

    // Stage 1: the exact parser and the same offline NC interpreter as before.
    const QByteArray utf8=text.toUtf8();
    const std::string source(utf8.constData(),std::size_t(utf8.size()));
    const auto simulated=mls::twin::simulateMultiPoseNc(source);
    if(!simulated.ok() || simulated.vertices.size()<2 || simulated.profiles.empty() ||
       simulated.scanGroups==0 || simulated.topology!=m_program.hardwareTopologyId ||
       simulated.profileRateHz!=m_program.hardwareFrequencyHz){
        const QString why=!simulated.ok()?QString::fromStdString(simulated.error):
            tr("离线解释结果缺少轨迹/有效轮廓，或H1/H3与频率不匹配");
        appendLog(tr("[V30实机诊断 1/4] NC逐段验证失败：%1").arg(why),"red");
        QMessageBox::warning(this,tr("实机状态诊断：第一阶段未通过"),why);
        return;
    }

    // Read actual controller state before activating hardware. No UI/cache
    // equality checks and no NC-header 'SimulationVerified' shortcuts.
    m_program.filePath=m_lastFilePath;
    auto live=m_executor->inspectRealRunReadOnly(m_program,text,m_lastFilePath);
    if(!live.blockers.isEmpty()){
        appendLog(tr("[V30实机诊断] 设备状态前置检查未通过：%1").arg(live.blockers.join("; ")),"red");
        QMessageBox::warning(this,tr("实机状态诊断：设备状态未通过"),live.blockers.join("\n"));
        return;
    }

    // A live sync trial will energize the laser/trigger electronics. Explicit
    // operator confirmation here is for the active test, not a run permit.
    const auto agree=QMessageBox::warning(this,tr("实机状态诊断：即将开展硬件试采"),
        tr("逐段NC验证和设备只读状态检查已通过。下一步将按NC的H%1/%2Hz配置真实控制器与传感器，"
           "启动触发约3秒后停止并分析，再自动开展完整NC FCL碰撞复核。\n"
           "试采不执行NC五轴运动，但会激活硬件同步和传感器/激光触发；静止H3可能无法产生有效轮廓。\n"
           "请确认防护、现场区域及急停可用。是否执行这一次完整诊断？")
            .arg(m_program.hardwareTopologyId).arg(m_program.hardwareFrequencyHz),
        QMessageBox::Yes|QMessageBox::Cancel,QMessageBox::Cancel);
    if(agree!=QMessageBox::Yes)return;

    m_preflightActive=true;
    m_preflightCancelled=false;
    m_preflightStage=PreflightStage::HardwareProbe;
    m_preflightNcText=text;
    m_preflightSourcePath=m_lastFilePath;
    m_preflightSourceDigest=QCryptographicHash::hash(diskBytes,QCryptographicHash::Sha256);
    m_preflightProbeReportPath.clear();
    m_preflightTrace.clear();
    m_preflightTrace<<tr("1/4 NC解析和离线逐段模拟：通过。H%1/%2Hz，扫描组%3，段数%4，模拟轮廓%5")
        .arg(simulated.topology).arg(simulated.profileRateHz)
        .arg(static_cast<qulonglong>(simulated.scanGroups))
        .arg(static_cast<qulonglong>(simulated.vertices.size()))
        .arg(static_cast<qulonglong>(simulated.profiles.size()));
    for(const auto& observation:live.observations)m_preflightTrace<<observation;
    m_preflightTrace<<tr("2/4 按NC执行真实硬件同步试采：进行中");
    m_probeRunning=true;
    setReadOnly(true);
    m_preflightProgress=new QProgressDialog(tr("第2/4步：正在按NC进行真实硬件同步试采…"),
        tr("取消诊断"),0,4,this);
    m_preflightProgress->setWindowTitle(tr("V30 实机状态诊断"));
    m_preflightProgress->setMinimumDuration(0);
    m_preflightProgress->setWindowModality(Qt::ApplicationModal);
    m_preflightProgress->setAutoReset(false);
    m_preflightProgress->setAutoClose(false);
    m_preflightProgress->setValue(1);
    connect(m_preflightProgress,&QProgressDialog::canceled,this,[this]() {
        if(!m_preflightActive || m_preflightCancelled)return;
        m_preflightCancelled=true;
        appendLog(tr("[V30实机诊断] 用户取消，正在安全停止当前检测"),"orange");
        if(m_preflightStage==PreflightStage::HardwareProbe)
            emit requestStopHardwareNcProbe();
        else if(m_preflightStage==PreflightStage::GeometryAudit)
            emit requestCancelFullNcGeometryAudit();
    });
    updateButtonStates();
    appendLog(tr("[V30实机诊断 1/4] NC逐段验证通过，准备进行H%1/%2Hz真实试采")
        .arg(m_program.hardwareTopologyId).arg(m_program.hardwareFrequencyHz),"green");
    // Re-read the original file immediately before activating laser hardware,
    // not only after the 3-second trial.
    QString beforeProbeError;
    if(!preflightSourceUnchanged(&beforeProbeError)){
        completeRealRunPreflight(false,beforeProbeError);
        return;
    }
    // Take a snapshot of the sensor settings actually selected for this trial.
    // The NC itself, not these UI values, determines H1/H3 topology and rate.
    emit requestSensorParams();
    m_program.sensorParams=m_sensorParams;
    emit requestHardwareNcProbe(m_program,m_preflightNcText);
}

void GCodeWidget::showHardwareNcProbeResult(bool passed,const QString& report,const QString& path)
{
    if(!m_preflightActive || m_preflightStage!=PreflightStage::HardwareProbe){
        appendLog(tr("忽略不属于当前实机诊断的硬件试采回调"),"orange");return;
    }
    m_probeRunning=false;
    m_preflightTrace<<tr("2/4 真实H1/H3同步试采结果：%1；证据=%2")
        .arg(passed?tr("通过"):tr("失败或没有有效数据"),path);
    m_preflightTrace<<report;
    appendLog(tr("[V30实机诊断 2/4] 硬件试采%1").arg(passed?tr("通过"):tr("未通过")),
        passed?"green":"red");
    if(!passed || m_preflightCancelled){
        completeRealRunPreflight(false,m_preflightCancelled?tr("用户取消硬件试采"):tr("实测同步采集未通过"));
        return;
    }
    QString changed;
    if(!preflightSourceUnchanged(&changed)){
        completeRealRunPreflight(false,changed);return;
    }
    m_preflightProbeReportPath=path;
    m_preflightStage=PreflightStage::GeometryAudit;
    m_preflightTrace<<tr("3/4 整条NC机床+STEP工件FCL碰撞复核：进行中");
    if(m_preflightProgress){
        m_preflightProgress->setValue(2);
        m_preflightProgress->setLabelText(tr("第3/4步：正在复核整条NC的机床+STEP工件FCL碰撞与运动限位…"));
    }
    updateButtonStates();
    // MainWindow's asynchronous machine FCL sweep. No repeat optical ray tracing.
    emit requestFullNcGeometryAudit(m_preflightNcText);
}

void GCodeWidget::showFullNcGeometryAuditProgress(quint64 checked,quint64 total)
{
    if(!m_preflightActive || m_preflightStage!=PreflightStage::GeometryAudit ||
       !m_preflightProgress || total==0)return;
    const int percent=static_cast<int>((checked*100)/total);
    m_preflightProgress->setLabelText(tr("第3/4步：机床+STEP碰撞复核 %1/%2 段（%3%），可取消")
        .arg(static_cast<qulonglong>(checked))
        .arg(static_cast<qulonglong>(total)).arg(percent));
}

void GCodeWidget::showFullNcGeometryAuditResult(bool passed,const QString& report)
{
    if(!m_preflightActive || m_preflightStage!=PreflightStage::GeometryAudit){
        appendLog(tr("忽略不属于当前实机诊断的几何复核回调"),"orange");return;
    }
    m_preflightTrace<<tr("3/4 全轨迹机床FCL碰撞：%1")
        .arg(passed?tr("通过"):tr("失败或机床FCL模型不可用"));
    m_preflightTrace<<report;
    appendLog(tr("[V30实机诊断 3/4] FCL碰撞复核%1")
        .arg(passed?tr("通过"):tr("未通过")),passed?"green":"red");
    if(!passed || m_preflightCancelled){
        completeRealRunPreflight(false,m_preflightCancelled?tr("用户取消完整NC几何复核"):tr("整条NC的FCL碰撞复核未通过"));
        return;
    }
    // Finite stage 4: the user now supplies independently measured placement
    // and documented physical fixture/guarding inspections; a mathematical
    // consistency gate in MainWindow binds that evidence to this STEP + NC.
    m_preflightStages123Passed=true;
    m_preflightStage=PreflightStage::SiteAcceptance;
    m_preflightTrace<<tr("4/4 独立测量工件位姿、夹具与现场防护验收：等待现场记录");
    if(m_preflightProgress){
        m_preflightProgress->setValue(3);
        m_preflightProgress->hide(); // Do not mask the on-site modal dialog.
    }
    emit requestSiteExecutionAcceptance(m_program,m_preflightSourceDigest);
}

void GCodeWidget::showSiteExecutionAcceptanceResult(bool passed,const QString& report)
{
    if(!m_preflightActive || m_preflightStage!=PreflightStage::SiteAcceptance){
        appendLog(tr("忽略不属于当前第四步验收的回调"),"orange");return;
    }
    m_preflightTrace<<tr("4/4 实测现场验收：%1").arg(passed?tr("通过"):tr("未通过"));
    m_preflightTrace<<report;
    completeRealRunPreflight(passed,passed?QString():report);
}

void GCodeWidget::onRetrySiteAcceptanceClicked()
{
    if(m_preflightActive || m_executing || !m_executor || !m_preflightStages123Passed)return;
    QString changed;
    if(!preflightSourceUnchanged(&changed)){
        m_preflightStages123Passed=false;
        m_preflightQualified=false;
        updateButtonStates();
        QMessageBox::warning(this,tr("第四步验收不能复用"),changed+tr("；请重新运行综合诊断"));
        return;
    }
    m_preflightActive=true;
    m_preflightCancelled=false;
    m_preflightStage=PreflightStage::SiteAcceptance;
    m_program.filePath=m_preflightSourcePath;
    updateButtonStates();
    emit requestSiteExecutionAcceptance(m_program,m_preflightSourceDigest);
}

void GCodeWidget::completeRealRunPreflight(bool stagesPassed,const QString& failureReason)
{
    if(!m_preflightActive)return;
    bool qualified=stagesPassed && !m_preflightCancelled;
    QStringList blockers;
    if(!failureReason.isEmpty())blockers<<failureReason;
    if(m_preflightCancelled && failureReason.isEmpty())blockers<<tr("用户取消诊断");
    QString changed;
    if(!preflightSourceUnchanged(&changed)){qualified=false;blockers<<changed;}


    m_preflightTrace<<tr("4/4 准入状态复核：%1").arg(qualified?tr("通过，允许申请开始执行（启动时实时复核）"):tr("未通过"));
    if(!blockers.isEmpty())m_preflightTrace<<tr("阻断详情：")+blockers.join(QStringLiteral("; "));
    QString directory=QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);
    if(directory.isEmpty())directory=QDir::tempPath()+QStringLiteral("/MotionLaserSync");
    directory+=QStringLiteral("/gcode_preflight");
    QDir().mkpath(directory);
    const QString output=directory+QStringLiteral("/V30_integrated_preflight_")+
        QDateTime::currentDateTimeUtc().toString(QStringLiteral("yyyyMMdd_HHmmss_zzz"))+QStringLiteral(".json");
    QJsonObject json;
    json.insert(QStringLiteral("schema"),QStringLiteral("V30-Integrated-NC-Preflight-v1"));
    json.insert(QStringLiteral("createdUtc"),QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs));
    json.insert(QStringLiteral("sourceNcFile"),m_preflightSourcePath);
    json.insert(QStringLiteral("sourceNcSha256"),QString::fromLatin1(m_preflightSourceDigest.toHex()));
    json.insert(QStringLiteral("activeHardwareTrial"),true);
    json.insert(QStringLiteral("axisMotionDuringDiagnosis"),false);
    json.insert(QStringLiteral("autoRunStarted"),false);
    json.insert(QStringLiteral("preflightPassed"),qualified);
    json.insert(QStringLiteral("hardwareProbeEvidencePath"),m_preflightProbeReportPath);
    json.insert(QStringLiteral("blockers"),QJsonArray::fromStringList(blockers));
    json.insert(QStringLiteral("steps"),QJsonArray::fromStringList(m_preflightTrace));
    QSaveFile outputFile(output);
    if(!outputFile.open(QIODevice::WriteOnly) ||
       outputFile.write(QJsonDocument(json).toJson(QJsonDocument::Indented))<0 ||
       !outputFile.commit()){
        qualified=false;
        blockers<<tr("无法写入V30综合诊断报告；请检查AppData写入权限");
    }
    const QString details=m_preflightTrace.join(QStringLiteral("\n"));
    appendLog(tr("[V30实机状态诊断] %1；详细证据：%2")
        .arg(qualified?tr("四阶段通过"):tr("阻断"),output),qualified?"green":"red");
    for(const auto& b:blockers)appendLog(tr("[阻断] %1").arg(b),"red");
    m_preflightQualified=qualified;
    m_preflightActive=false;
    m_preflightStage=PreflightStage::Idle;
    m_probeRunning=false;
    m_preflightCancelled=false;
    if(m_preflightProgress){
        m_preflightProgress->close();
        m_preflightProgress->deleteLater();
        m_preflightProgress=nullptr;
    }
    setReadOnly(false);
    updateButtonStates();
    QMessageBox result(this);
    result.setWindowTitle(tr("V30 实机状态诊断结果"));
    result.setIcon(qualified?QMessageBox::Information:QMessageBox::Warning);
    result.setText(qualified?
        tr("NC逐段解释、实际硬件同步采集、整条机床FCL碰撞复核及现场状态检查均通过。\n"
           "可以申请开始执行；正式启动仍会重新读取安全回路和最新机床位置。"):
        tr("综合诊断未通过，当前NC不能据此申请实机执行。"));
    result.setInformativeText(tr("综合报告：%1\n%2").arg(output,blockers.join(QStringLiteral("\n"))));
    result.setDetailedText(details);
    result.exec();
}

void GCodeWidget::onStartClicked()
{
    if (!m_executor) {
        QMessageBox::warning(this, tr("错误"), tr("执行器未初始化"));
        return;
    }
    
    // 先解析验证
    if (!m_programValid) {
        parseAndValidate();
        if (!m_programValid) {
            QMessageBox::warning(this, tr("无法执行"),
                               tr("G代码解析失败，请检查错误"));
            return;
        }
    }
    
    // For hardware G-code, signed commissioning is bound to the original
    // disk bytes. An unsaved or edited NC cannot reuse the last signed hash.
    if(m_program.syncMode==SyncMode::Hardware){
        if(m_lastFilePath.isEmpty()||m_editor->document()->isModified()){
            QMessageBox::warning(this,tr("现场NC版本未固定"),
                tr("硬件采集NC必须先保存为独立文件并重新载入，用于绑定本次几何/试采证据与真实磁盘NC"));
            return;
        }
        QFile original(m_lastFilePath);
        if(!original.open(QIODevice::ReadOnly)){
            QMessageBox::warning(this,tr("NC源文件读取失败"),original.errorString());return;
        }
        QString disk=QString::fromUtf8(original.readAll());
        disk.replace(QStringLiteral("\r\n"),QStringLiteral("\n"));
        if(disk!=m_editor->toPlainText()){
            QMessageBox::warning(this,tr("现场NC已改动"),
                tr("当前编辑器和磁盘原件不同；不能复用旧NC的几何或硬件试采证据"));return;
        }
        m_program.filePath=m_lastFilePath;
    }
    // 更新保存设置
    m_program.defaultSaveParams.baseDir = m_editSaveDir->text();
    m_program.defaultSaveParams.prefix = m_editSavePrefix->text();
    
    // ===== 运动参数优先级 =====
    // 优先级：G代码头部定义 > 界面设置 > 默认值
    // 注意：G代码解析时已经读取头部参数到 m_program.motionParams
    // 这里只需要用界面值填充那些G代码头部没有设置的参数
    
    // 获取界面设置的值
    float uiFeed = static_cast<float>(m_spinFeedSpeed->value());
    float uiRapid = static_cast<float>(m_spinRapidSpeed->value());
    float uiAccel = static_cast<float>(m_spinAccel->value());
    float uiDecel = static_cast<float>(m_spinDecel->value());
    
    // Never infer a header's presence from the numeric value. An explicit
    // Feed: 5 is different from an omitted Feed even though both equal defaults.
    const bool gcodeHasFeed=m_program.headerFeedSpecified;
    const bool gcodeHasRapid=m_program.headerRapidSpecified;
    const bool gcodeHasAccel=m_program.headerAccelSpecified;
    const bool gcodeHasDecel=m_program.headerDecelSpecified;

    // 如果G代码头部没有设置，使用界面值
    if (!gcodeHasFeed && m_program.syncMode != SyncMode::Hardware) {
        m_program.motionParams.feedSpeed = uiFeed;
    }
    if (!gcodeHasRapid && m_program.syncMode != SyncMode::Hardware) {
        m_program.motionParams.rapidSpeed = uiRapid;
    }
    if (!gcodeHasAccel && m_program.syncMode != SyncMode::Hardware) {
        m_program.motionParams.accel = uiAccel;
    }
    if (!gcodeHasDecel && m_program.syncMode != SyncMode::Hardware) {
        m_program.motionParams.decel = uiDecel;
    }
    
    // 记录使用的参数及其来源
    appendLog(tr("运动参数:"), "blue");
    appendLog(tr("  进给速度: %1 u/s %2")
        .arg(m_program.motionParams.feedSpeed, 0, 'f', 1)
        .arg(gcodeHasFeed ? "(G代码头)" : (m_program.syncMode==SyncMode::Hardware ? "(程序缺省)" : "(界面)")), "blue");
    appendLog(tr("  快速速度: %1 u/s %2")
        .arg(m_program.motionParams.rapidSpeed, 0, 'f', 1)
        .arg(gcodeHasRapid ? "(G代码头)" : (m_program.syncMode==SyncMode::Hardware ? "(程序缺省)" : "(界面)")), "blue");
    appendLog(tr("  加速度: %1 u/s² %2")
        .arg(m_program.motionParams.accel, 0, 'f', 0)
        .arg(gcodeHasAccel ? "(G代码头)" : (m_program.syncMode==SyncMode::Hardware ? "(程序缺省)" : "(界面)")), "blue");
    
    // 请求MainWindow更新传感器参数
    // 这个信号使用DirectConnection，MainWindow会立即设置m_sensorParams和m_syncMode
    emit requestSensorParams();
    
    // 设置传感器参数和同步模式（从主界面参数设置获取）
    m_program.sensorParams = m_sensorParams;
    // An explicit NC Hardware mode is authoritative. The Parameters page's
    // manual-capture Software/Hardware radio controls another workflow and
    // must not silently turn a Hardware NC into Software or reject it.
    if(!m_program.syncModeSpecified)m_program.syncMode=m_syncMode;
    if(m_program.syncMode==SyncMode::Hardware){
        const mls::gcode::preflight::HardwareSetting nc{
            m_program.hardwareTopologyId,m_program.hardwareFrequencyHz,m_program.cmmR1Edge};
        // NC is the selected capture contract. The UI setting belongs to a
        // separate manual-capture workflow and is never a machine readback.
        emit requestStageHardwareSettings(m_program);
        appendLog(tr("将按NC申请H%1/%2Hz/R1=%3；由启动时硬件读回最终确认")
            .arg(nc.topology).arg(nc.frequencyHz).arg(nc.r1Edge), "blue");
    }
    
    appendLog(tr("采集参数:"), "blue");
    appendLog(tr("  同步模式: %1").arg(m_program.syncMode == SyncMode::Hardware ? "硬件同步" : "软件同步"), "blue");
    appendLog(tr("  曝光时间: %1 us").arg(m_sensorParams.exposureTime), "blue");
    appendLog(tr("  空闲时间: %1 us").arg(m_sensorParams.idleTime), "blue");
    appendLog(tr("  分辨率索引: %1").arg(m_sensorParams.resolutionIndex), "blue");
    
    // 加载程序
    QString err;
    if (!m_executor->loadProgram(m_program, &err)) {
        QMessageBox::warning(this, tr("加载失败"), err);
        return;
    }
    
    // 开始执行
    if (!m_executor->start(&err)) {
        QMessageBox::warning(this, tr("启动失败"), err);
        return;
    }
    
    m_executing = true;
    m_paused = false;
    
    setReadOnly(true);
    updateButtonStates();
    
    m_logView->clear();
    appendLog(tr("程序开始执行"), "green");
    
    m_updateTimer.start();
    
    emit executionStateChanged(true);
    emit requestDisableControls(true);
}

void GCodeWidget::onPauseResumeClicked()
{
    if (!m_executor) return;
    
    if (m_paused) {
        QString err;
        if (m_executor->resume(&err)) {
            m_paused = false;
            appendLog(tr("程序继续执行"), "blue");
        } else {
            QMessageBox::warning(this, tr("继续失败"), err);
        }
    } else {
        m_executor->pause();
        m_paused = true;
        appendLog(tr("程序已暂停"), "orange");
    }
    
    updateButtonStates();
}

void GCodeWidget::onStopClicked()
{
    if (!m_executor) return;
    
    int ret = QMessageBox::question(this, tr("确认停止"),
                                   tr("确定要停止程序执行吗？"),
                                   QMessageBox::Yes | QMessageBox::No);
    if (ret == QMessageBox::Yes) {
        m_executor->stop();
    }
}

void GCodeWidget::onStepClicked()
{
    if (!m_executor) return;
    
    QString err;
    if (!m_executor->stepOnce(&err)) {
        appendLog(tr("单步执行: %1").arg(err), "red");
    }
}

void GCodeWidget::startExecution()
{
    onStartClicked();
}

void GCodeWidget::pauseExecution()
{
    if (m_executor && !m_paused) {
        m_executor->pause();
        m_paused = true;
    }
}

void GCodeWidget::resumeExecution()
{
    if (m_executor && m_paused) {
        QString err;
        m_executor->resume(&err);
        m_paused = false;
    }
}

void GCodeWidget::stopExecution()
{
    if (m_executor) {
        m_executor->stop();
    }
}

// ==================== 执行器信号处理 ====================

void GCodeWidget::onExecutorStateChanged(ExecutorState newState, ExecutorState oldState)
{
    Q_UNUSED(oldState)
    
    m_lblState->setText(GCodeUtils::stateName(newState));
    
    // 更新状态颜色
    QString color;
    switch (newState) {
    case ExecutorState::Running:
    case ExecutorState::ExecutingBlock:
        color = "green";
        break;
    case ExecutorState::Paused:
        color = "orange";
        break;
    case ExecutorState::Error:
        color = "red";
        break;
    case ExecutorState::Completed:
        color = "blue";
        break;
    default:
        color = "black";
        break;
    }
    m_lblState->setStyleSheet(QString("font-weight: bold; color: %1;").arg(color));
    
    updateButtonStates();
}

void GCodeWidget::onBlockStarted(int index, const GCodeBlock& block)
{
    m_lblCurrentBlock->setText(QString("N%1 %2")
        .arg(block.lineNumber)
        .arg(GCodeUtils::blockTypeName(block.type)));
    
    // 高亮表格行
    m_blockTable->selectRow(index);
    
    // 高亮编辑器行
    if (block.lineNumber > 0) {
        highlightCurrentLine(block.lineNumber);
    }
}

void GCodeWidget::onBlockCompleted(int index)
{
    Q_UNUSED(index)
    // 可以在这里更新表格状态
}

void GCodeWidget::onProgressUpdated(const ExecutorProgress& progress)
{
    updateProgressDisplay(progress);
}

void GCodeWidget::onProgramCompleted()
{
    m_executing = false;
    m_paused = false;
    
    m_updateTimer.stop();
    setReadOnly(false);
    updateButtonStates();
    
    appendLog(tr("程序执行完成"), "green");
    
    emit executionStateChanged(false);
    emit requestDisableControls(false);
    
    QMessageBox::information(this, tr("完成"), tr("G代码程序执行完成"));
}

void GCodeWidget::onProgramStopped()
{
    m_executing = false;
    m_paused = false;
    
    m_updateTimer.stop();
    setReadOnly(false);
    updateButtonStates();
    
    appendLog(tr("程序已停止"), "orange");
    
    emit executionStateChanged(false);
    emit requestDisableControls(false);
}

void GCodeWidget::onErrorOccurred(const QString& message)
{
    appendLog(tr("错误: %1").arg(message), "red");
    
    m_executing = false;
    m_paused = false;
    
    m_updateTimer.stop();
    setReadOnly(false);
    updateButtonStates();
    
    emit executionStateChanged(false);
    emit requestDisableControls(false);
    
    QMessageBox::critical(this, tr("执行错误"), message);
}

void GCodeWidget::onWarningOccurred(const QString& message)
{
    appendLog(tr("警告: %1").arg(message), "orange");
}

void GCodeWidget::onProgressMessage(const QString& message)
{
    appendLog(message);
}

void GCodeWidget::onEditorTextChanged()
{
    int lineCount = m_editor->blockCount();
    m_lineCountLabel->setText(tr("行数: %1").arg(lineCount));
    m_programValid = false;
    m_preflightQualified=false;
    m_preflightStages123Passed=false;
    m_statusLabel->setText(tr("状态: 已修改"));
    updateButtonStates();
}

void GCodeWidget::onUpdateTimer()
{
    if (m_executor) {
        auto progress = m_executor->getProgress();
        updateProgressDisplay(progress);
    }
}

// ==================== 辅助函数 ====================

void GCodeWidget::updateButtonStates()
{
    bool hasExecutor = (m_executor != nullptr);
    // Let an OfflineOnly digital-twin hardware-NC candidate request the
    // full real-run gate.  Merely enabling this button does NOT authorize
    // motion: GCodeExecutor::start() still checks actual hardware, FCL entry
    // and the independently commissioned, exact-NC site verifier.
    const bool hasHardwareCapture=m_program.syncMode==SyncMode::Hardware &&
        std::any_of(m_program.blocks.begin(),m_program.blocks.end(),
            [](const GCodeBlock& block) {
                return block.syncCapture || block.type==BlockType::M50_SyncOn;
            });
    const bool commissionedCandidate =
        m_program.syncMode==SyncMode::Hardware &&
        m_program.programFrame==QStringLiteral("MACHINE_JOINTS_XYZCB") &&
        (m_program.hardwareTopologyId==1 || m_program.hardwareTopologyId==3) &&
        mls::gcode::validateSensorTriggerMetadata(m_program);
    // Requiring the integrated check for hardware capture must NOT depend on
    // whether the editable NC header says OfflineOnly: true or false.
    const bool needsIntegratedPreflight = m_program.offlineOnly || hasHardwareCapture;
    bool canStart = hasExecutor && !m_executing && !m_preflightActive &&
        m_programValid && (!needsIntegratedPreflight ||
            (commissionedCandidate && m_preflightQualified));
    bool canPauseResume = hasExecutor && m_executing;
    bool canStop = hasExecutor && m_executing;
    // Keep single-step of a candidate disabled until a proper run permit
    // is issued; it is not a preflight workflow.
    bool canStep = hasExecutor && !m_executing && !m_preflightActive &&
        m_programValid && !needsIntegratedPreflight;
    
    m_btnRealPreflight->setEnabled(hasExecutor && !m_executing && !m_preflightActive);
    m_btnRetrySiteAcceptance->setEnabled(hasExecutor && !m_executing &&
        !m_preflightActive && m_preflightStages123Passed && m_programValid);
    m_btnParse->setEnabled(!m_preflightActive && !m_executing);
    m_btnSave->setEnabled(!m_preflightActive && !m_executing);
    m_btnStart->setToolTip(needsIntegratedPreflight ?
        tr("任何硬件采集NC（OfflineOnly为true或false）都必须完成综合实机诊断；启动时仍重新核查机床状态") : QString());
    m_btnStart->setEnabled(canStart);
    m_btnPauseResume->setEnabled(canPauseResume);
    m_btnStop->setEnabled(canStop);
    m_btnStep->setEnabled(canStep);
    
    m_btnPauseResume->setText(m_paused ? tr("继续") : tr("暂停"));
}

void GCodeWidget::updateProgressDisplay(const ExecutorProgress& progress)
{
    // 进度条
    int percent = progress.totalBlocks > 0 ?
        (progress.currentBlock * 100 / progress.totalBlocks) : 0;
    m_progressBar->setValue(percent);
    
    // 耗时
    qint64 ms = progress.elapsedTimeMs;
    int hours = ms / 3600000;
    int mins = (ms % 3600000) / 60000;
    int secs = (ms % 60000) / 1000;
    m_lblElapsedTime->setText(QString("%1:%2:%3")
        .arg(hours, 2, 10, QChar('0'))
        .arg(mins, 2, 10, QChar('0'))
        .arg(secs, 2, 10, QChar('0')));
    
    // 统计
    m_lblProfileCount->setText(QString::number(progress.profileCount));
    m_lblPointCount->setText(QString::number(progress.pointCloudCount));
}

void GCodeWidget::highlightCurrentLine(int lineNumber)
{
    QTextCursor cursor(m_editor->document()->findBlockByLineNumber(lineNumber - 1));
    m_editor->setTextCursor(cursor);
    m_editor->centerCursor();
    
    // 高亮整行
    QList<QTextEdit::ExtraSelection> selections;
    QTextEdit::ExtraSelection selection;
    selection.format.setBackground(QColor(255, 255, 0, 100));
    selection.format.setProperty(QTextFormat::FullWidthSelection, true);
    selection.cursor = cursor;
    selections.append(selection);
    m_editor->setExtraSelections(selections);
}

void GCodeWidget::appendLog(const QString& message, const QString& color)
{
    QString timestamp = QDateTime::currentDateTime().toString("hh:mm:ss");
    QString html;
    
    if (color.isEmpty()) {
        html = QString("[%1] %2").arg(timestamp, message);
    } else {
        html = QString("[%1] <span style=\"color:%2;\">%3</span>")
               .arg(timestamp, color, message);
    }
    
    m_logView->appendHtml(html);
    m_logView->verticalScrollBar()->setValue(m_logView->verticalScrollBar()->maximum());
    
    emit logMessage(message);
}

void GCodeWidget::parseAndValidate()
{
    m_preflightQualified=false; // a fresh parse requires a fresh integrated check
    m_preflightStages123Passed=false;
    QString text = m_editor->toPlainText();
    if (text.trimmed().isEmpty()) {
        m_statusLabel->setText(tr("状态: 空程序"));
        m_programValid = false;
        m_blockTable->setRowCount(0);
        return;
    }
    
    QList<ParseError> errors;
    bool success = m_parser->parseText(text, m_program, &errors);
    
    // 显示错误/警告
    for (const auto& e : errors) {
        QString msg = e.lineNumber > 0 ?
            tr("行 %1: %2").arg(e.lineNumber).arg(e.errorMessage) :
            e.errorMessage;
        appendLog(msg, "red");
    }
    
    if (success || !m_program.blocks.empty()) {
        // 验证程序
        m_parser->validateProgram(m_program, &errors);
        
        // 更新表格
        m_blockTable->setRowCount(static_cast<int>(m_program.blocks.size()));
        
        for (size_t i = 0; i < m_program.blocks.size(); ++i) {
            const auto& block = m_program.blocks[i];
            int row = static_cast<int>(i);
            
            // 行号
            m_blockTable->setItem(row, 0, new QTableWidgetItem(
                block.lineNumber > 0 ? QString::number(block.lineNumber) : "-"));
            
            // 指令
            m_blockTable->setItem(row, 1, new QTableWidgetItem(
                GCodeUtils::blockTypeName(block.type)));
            
            // 参数
            QString params;
            if (block.hasX) params += QString("X%1 ").arg(block.target.x, 0, 'f', 2);
            if (block.hasY) params += QString("Y%1 ").arg(block.target.y, 0, 'f', 2);
            if (block.hasZ) params += QString("Z%1 ").arg(block.target.z, 0, 'f', 2);
            if (block.hasC) params += QString("C%1deg ").arg(block.target.c, 0, 'f', 4);
            if (block.hasB) params += QString("B%1deg ").arg(block.target.b, 0, 'f', 4);
            if (block.feedRate.has_value()) params += QString("F%1").arg(block.feedRate.value(), 0, 'f', 0);
            m_blockTable->setItem(row, 2, new QTableWidgetItem(params.trimmed()));
            
            // 采集
            QString syncText = block.syncCapture ? tr("是") : tr("否");
            auto* syncItem = new QTableWidgetItem(syncText);
            if (block.syncCapture) {
                syncItem->setBackground(QColor(200, 255, 200));
            }
            m_blockTable->setItem(row, 3, syncItem);
            
            // 注释
            m_blockTable->setItem(row, 4, new QTableWidgetItem(block.comment));
        }
        
        m_blockTable->resizeColumnsToContents();
        
        m_programValid = success;
        m_statusLabel->setText(success ? 
            tr("状态: 有效 (%1 段)").arg(m_program.blocks.size()) :
            tr("状态: 有错误"));
        m_statusLabel->setStyleSheet(success ? "color: green;" : "color: red;");
        
        appendLog(tr("解析完成: %1 个程序段, %2 个运动段, %3 个采集段")
            .arg(m_program.blocks.size())
            .arg(m_program.motionBlocks())
            .arg(m_program.captureBlocks()), 
            success ? "green" : "orange");
    } else {
        m_programValid = false;
        m_statusLabel->setText(tr("状态: 解析失败"));
        m_statusLabel->setStyleSheet("color: red;");
        m_blockTable->setRowCount(0);
    }
    
    updateButtonStates();
}

QString GCodeWidget::generateTemplate(const QString& templateName)
{
    // The same NC files are deployed next to the executable and checked offline.
    if (!QStringList{"basic","multiregion","raster","continuous"}.contains(templateName))
        return QString();
    QFile file(QCoreApplication::applicationDirPath()+"/examples/v21_template_"+templateName+".nc");
    if (!file.open(QIODevice::ReadOnly)) {
        QMessageBox::warning(this,tr("模板不可用"),tr("请把工程examples目录部署到exe目录：%1").arg(file.errorString()));
        return QString();
    }
    const QString dir=m_editSaveDir->text(), prefix=m_editSavePrefix->text();
    for(const auto& value: {dir,prefix}) {
        if(value.contains('"')||value.contains('\n')||value.contains('\r')) {
            QMessageBox::warning(this,tr("路径无效"),tr("路径及文件前缀不能含双引号或换行"));return QString();
        }
    }
    return QString::fromUtf8(file.readAll()).replace("@SAVE_DIR@",dir).replace("@SAVE_PREFIX@",prefix);
}
