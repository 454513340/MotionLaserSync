#pragma once

#include <QWidget>
#include <QPlainTextEdit>
#include <QTableWidget>
#include <QPushButton>
#include <QLabel>
#include <QProgressBar>
#include <QComboBox>
#include <QLineEdit>
#include <QGroupBox>
#include <QTimer>
#include <QSplitter>
#include <QPointer>
#include <QDoubleSpinBox>

#include "gcode/GCodeTypes.h"
#include "gcode/HardwarePreflightLogic.h"

// 前向声明
class GCodeExecutor;
class GCodeParser;
class QProgressDialog;

/**
 * @file GCodeWidget.h
 * @brief G代码控制面板
 * 
 * 提供G代码的输入、编辑、加载、执行控制界面
 */
class GCodeWidget : public QWidget
{
    Q_OBJECT

public:
    explicit GCodeWidget(QWidget* parent = nullptr);
    ~GCodeWidget() override;

    /**
     * @brief 设置执行器
     */
    void setExecutor(GCodeExecutor* executor);

    /**
     * @brief 设置传感器参数（从主界面参数设置获取）
     */
    void setSensorParams(const SensorParams& params);
    
    /**
     * @brief 设置同步模式
     */
    void setSyncMode(SyncMode mode);
    void setHardwareSyncUiIntent(int topology, int frequencyHz, int cmmEdge,
                                 bool manualCaptureHardwareSelected);

    /**
     * @brief 加载G代码文本
     */
    void loadGCode(const QString& text);

    /**
     * @brief 从文件加载
     */
    bool loadFromFile(const QString& filePath);

    /**
     * @brief 获取当前G代码文本
     */
    QString getGCodeText() const;

    /**
     * @brief 清空编辑器
     */
    void clearEditor();

    /**
     * @brief 设置只读模式（执行时禁用编辑）
     */
    void setReadOnly(bool readOnly);

    /**
     * @brief 是否正在执行
     */
    bool isExecuting() const;
    void showHardwareNcProbeResult(bool passed,const QString& report,const QString& reportPath);
    void showFullNcGeometryAuditResult(bool passed,const QString& report);
    void showFullNcGeometryAuditProgress(quint64 checked,quint64 total);
    void showSiteExecutionAcceptanceResult(bool passed, const QString& report);


signals:
    /**
     * @brief 执行状态改变
     */
    void executionStateChanged(bool running);

    /**
     * @brief 请求禁用其他控制
     */
    void requestDisableControls(bool disable);
    
    /**
     * @brief 请求更新传感器参数（在执行开始前发出）
     * MainWindow收到后应立即调用setSensorParams和setSyncMode
     */
    void requestSensorParams();
    // Local, in-memory staging only, invoked on explicit Start; no hardware IO.
    void requestStageHardwareSettings(const GCodeProgram& nc);
    // Explicit powered test: hardware PWM/laser is active, no NC axis movement.
    void requestHardwareNcProbe(const GCodeProgram& nc,const QString& originalNcText);
    void requestStopHardwareNcProbe();
    void requestFullNcGeometryAudit(const QString& exactNcText);
    void requestCancelFullNcGeometryAudit();
    void requestSiteExecutionAcceptance(const GCodeProgram& nc, const QByteArray& rawDiskDigest);
    void requestInvalidateNcPreflightEvidence();


    /**
     * @brief 日志消息
     */
    void logMessage(const QString& message);

public slots:
    /**
     * @brief 开始执行
     */
    void startExecution();

    /**
     * @brief 暂停执行
     */
    void pauseExecution();

    /**
     * @brief 继续执行
     */
    void resumeExecution();

    /**
     * @brief 停止执行
     */
    void stopExecution();

private slots:
    void onLoadFileClicked();
    void onSaveFileClicked();
    void onParseClicked();
    void onRealRunPreflightClicked();
    void onRetrySiteAcceptanceClicked();

    void onInsertTemplateClicked();
    void onClearClicked();
    
    void onStartClicked();
    void onPauseResumeClicked();
    void onStopClicked();
    void onStepClicked();
    
    void onExecutorStateChanged(ExecutorState newState, ExecutorState oldState);
    void onBlockStarted(int index, const GCodeBlock& block);
    void onBlockCompleted(int index);
    void onProgressUpdated(const ExecutorProgress& progress);
    void onProgramCompleted();
    void onProgramStopped();
    void onErrorOccurred(const QString& message);
    void onWarningOccurred(const QString& message);
    void onProgressMessage(const QString& message);
    
    void onEditorTextChanged();
    void onUpdateTimer();

private:
    void setupUI();
    void setupConnections();
    void updateButtonStates();
    void updateProgressDisplay(const ExecutorProgress& progress);
    void highlightCurrentLine(int lineNumber);
    void appendLog(const QString& message, const QString& color = QString());
    void parseAndValidate();
    bool preflightSourceUnchanged(QString* failure) const;
    void completeRealRunPreflight(bool stagesPassed, const QString& failureReason = QString());
    QString generateTemplate(const QString& templateName);

private:
    // 执行器
    GCodeExecutor* m_executor = nullptr;
    GCodeParser* m_parser = nullptr;
    
    // 当前程序
    GCodeProgram m_program;
    bool m_programValid = false;
    
    // UI - 编辑区
    QPlainTextEdit* m_editor = nullptr;
    QLabel* m_lineCountLabel = nullptr;
    QLabel* m_statusLabel = nullptr;
    
    // UI - 程序信息
    QTableWidget* m_blockTable = nullptr;
    
    // UI - 控制按钮
    QPushButton* m_btnLoad = nullptr;
    QPushButton* m_btnSave = nullptr;
    QPushButton* m_btnParse = nullptr;
    QPushButton* m_btnRealPreflight = nullptr;
    // V30 single-button asynchronous preflight state (no separate test buttons).
    enum class PreflightStage { Idle, HardwareProbe, GeometryAudit, SiteAcceptance };
    PreflightStage m_preflightStage=PreflightStage::Idle;
    bool m_probeRunning=false;
    bool m_preflightActive=false;
    bool m_preflightQualified=false;
    bool m_preflightStages123Passed=false;
    QPushButton* m_btnRetrySiteAcceptance=nullptr;
    bool m_preflightCancelled=false;
    QPointer<QProgressDialog> m_preflightProgress;
    QString m_preflightNcText;
    QString m_preflightSourcePath;
    QByteArray m_preflightSourceDigest;
    QString m_preflightProbeReportPath;
    QStringList m_preflightTrace;

    QPushButton* m_btnInsertTemplate = nullptr;
    QPushButton* m_btnClear = nullptr;
    
    QPushButton* m_btnStart = nullptr;
    QPushButton* m_btnPauseResume = nullptr;
    QPushButton* m_btnStop = nullptr;
    QPushButton* m_btnStep = nullptr;
    
    // UI - 进度显示
    QProgressBar* m_progressBar = nullptr;
    QLabel* m_lblCurrentBlock = nullptr;
    QLabel* m_lblElapsedTime = nullptr;
    QLabel* m_lblState = nullptr;
    QLabel* m_lblProfileCount = nullptr;
    QLabel* m_lblPointCount = nullptr;
    
    // UI - 日志
    QPlainTextEdit* m_logView = nullptr;
    
    // UI - 保存设置
    QLineEdit* m_editSaveDir = nullptr;
    QLineEdit* m_editSavePrefix = nullptr;
    QPushButton* m_btnBrowseSaveDir = nullptr;
    
    // UI - 运动参数设置
    QDoubleSpinBox* m_spinFeedSpeed = nullptr;
    QDoubleSpinBox* m_spinRapidSpeed = nullptr;
    QDoubleSpinBox* m_spinAccel = nullptr;
    QDoubleSpinBox* m_spinDecel = nullptr;
    
    // 定时器
    QTimer m_updateTimer;
    
    // 状态
    bool m_executing = false;
    bool m_paused = false;
    QString m_lastFilePath;
    
    // 传感器参数（从主界面参数设置获取）
    SensorParams m_sensorParams;
    SyncMode m_syncMode = SyncMode::Software;
    bool m_hardwareUiAvailable = false;
    mls::gcode::preflight::HardwareSetting m_hardwareUiSetting{};
    bool m_manualHardwareSelected = false;
};
