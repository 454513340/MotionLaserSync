#pragma once

#include <QMainWindow>
#include <QTimer>
#include <QFutureWatcher>
#include <memory>
#include <array>
#include <atomic>
#include <cstdint>
#include <chrono>
#include <functional>
#include <utility>
#include <QByteArray>
#include <limits>
#include <QVector>
#include <QString>

#include "core/DataTypes.h"
#include "calibration/SphereDistanceAngleStudy.h"
#include "calibration/SphereExposureSweep.h"
#include "calibration/Small7HeightPlan.h"
#include <QMetaObject>
#include <vector>

// 前向声明
class QLineEdit;
class QPushButton;
class QTableWidget;
class QTabWidget;
class QComboBox;
class QSpinBox;
class QDoubleSpinBox;
class QRadioButton;
class QCheckBox;
class QLabel;
class QTextEdit;
class QSplitter;
class QGroupBox;
class QStackedWidget;
class QCloseEvent;
class QProcess;
class QProgressBar;
class QFile;

class ZmcController;
class LaserSensor;
class SyncManager;
class PointCloudBuilder;
namespace mls {
namespace safety { class MachineSafetyModel; struct TransitionSafetyResult; struct PoseSafetyResult; }
namespace twin { struct MultiPoseQtReport; }
namespace calibration { class MaterialExposureLibrary; }
}

class Profile2DWidget;
class ProfileImageWidget;
class PointCloudWidget;
class AxisControlWidget;
class VideoWidget;
class GCodeWidget;
class GCodeExecutor;
struct GCodeProgram;
class LogWidget;
class CalibrationWidget;
class PathPlanningWidget;
class PointCloudProcessWidget;

/**
 * @brief 主窗口
 * 
 * 负责整体 UI 布局和协调各组件。
 * 通过委托方式将具体功能分配给各个子系统。
 */
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override;

protected:
    void closeEvent(QCloseEvent* event) override;

private:
    QWidget* createRuntimeAxisCompensationPanel(QWidget* parent);
    bool runtimeCompensationUiBusy() const;
    bool loadRuntimeAxisCompensationBundle(bool enableAfterLoad);

    // V27.6.53 hardware-temporal runtime controls shared by Parameters and 3-D view.
    void refreshHardwareTemporalRuntimeUi();
    void onHardwareTemporalRuntimeToggled(bool checked);
    void clearPointCloudForTemporalModeChange();
    QLineEdit* m_runtimeCompBundlePath = nullptr;
    QCheckBox* m_runtimeCompLegacyAck = nullptr;
    QCheckBox* m_runtimeCompContinuousAck = nullptr;
    QSpinBox* m_runtimeCompSettle = nullptr;
    QLabel* m_runtimeCompStatus = nullptr;
    QString m_runtimeCompSummary;

private slots:
    // 连接管理
    void onControllerConnect();
    void onControllerDisconnect();
    void onSensorConnect();
    void onSensorDisconnect();
    
    // 传感器参数读写
    void onReadSensorParams();
    void onWriteSensorParams();
    void onResetSensorParams();

    // 轴参数
    void onReadAxisParams();
    void onWriteAxisParams();
    void onWriteAllAxisParams();

    // 全局控制
    void onZeroAll();
    void onHomeAll();
    void onStopAll();

    // 轴运动请求
    void onAxisHomeRequested(int axisId, const AxisParams& params);
    void onAxisAbsMoveRequested(int axisId, float pos, const AxisParams& params);
    void onAxisRelMoveRequested(int axisId, float delta, const AxisParams& params);
    void onAxisJogRequested(int axisId, bool positive, const AxisParams& params);
    void onAxisStopRequested(int axisId);
    void onRotaryServoEnableRequested(int axisId, bool enable);
    void onResyncRotaryAbsolute();

    // 五轴运动学 P1/P2/P3 + P4 关节空间同步定位
    void onKinematicsLoadCurrent();
    void onKinematicsForward();
    void onKinematicsRoundTrip();
    void onKinematicsPlanJointMove();
    void onKinematicsExecuteJointMove();
    void onKinematicsStopJointMove();

    // 轨迹插补
    void onLoadPathFromFile();
    void onAddPathPoint();
    void onRemovePathPoint();
    void onStartMerge();
    void onStopMerge();

    // 采集控制 (Profile 模式)
    void onStartCapture();
    void onStopCapture();
    void onClearData();
    void onAnalyzeData();
    
    // Video 模式控制
    void onStartVideo();
    void onStopVideo();
    void onVideoFrameReceived(const QImage& frame);
    
    // 轮廓预览模式控制
    void onStartProfilePreview();
    void onStartSingleSphereCalibrationPreview();
    void onStopProfilePreview();
    void onPreviewModeChanged();
    
    // 曝光测试
    void onStartExposureTest();
    void onStopExposureTest();
    void onExposureTestStep();
    void onAnalyzeExposureDirectory();
    void onMaterialExposureProfileChanged(int index);
    void onReloadMaterialExposureLibrary();

    // Full-field, software-internal-trigger moving-X sphere exposure sweep.
    void onPlanSphereExposureSweep();
    void onStartSphereExposureSweep();
    void onStopSphereExposureSweep();
    void onSphereExposureProfile(const LaserProfile& profile);
    void sphereExposurePopulatePlanPreview();
    void sphereExposureSetPassPreviewState(int passIndex, const QString& state);

    // 球面测量距离/截面角研究
    void onSphereStudyImportCloud();
    void onSphereStudyFitSphere();
    void onSphereStudyGeneratePath();
    void onSphereStudyMoveSelected();
    void onSphereStudyStopMove();
    void onSphereStudyCaptureSelected();
    void onSphereStudySimulationStart();
    void onSphereStudySimulationStop();
    void onSphereStudySimulationClear();
    void onSphereStudyAutoStart();
    void onSphereStudyAutoPauseResume();
    void onSphereStudyAutoStop();
    void onSphereStudyAnalyze();
    void onSphereStudyExport();

    // 定时器
    void onStatusTimer();
    void onPosTimer();
    void onProfileUiTimer();

    // 数据导出
    void onExportAlignmentData();
    void onExportPointCloud();
    void onImportPointCloud();

    // 轮廓更新 - 直接显示传感器原始数据
    void onProfileReceived(const LaserProfile& profile);

    // 轮廓更新 - 用于其他处理
    void onAlignedDataReady(const QVector<AlignedPoint>& points);

    // G代码执行控制
    void onGCodeDisableControls(bool disable);
    void onGCodeExecutionStateChanged(bool running);
    void onGCodeRequestSensorParams();
    void onGCodeHardwareNcProbeRequested(const GCodeProgram& nc,const QString& originalNcText);
    void onGCodeHardwareNcProbeStopRequested();
    void onGCodeFullNcGeometryAuditRequested(const QString& ncText);
    void onGCodeFullNcGeometryAuditCancelRequested();
    void onGCodeSiteExecutionAcceptanceRequested(const GCodeProgram& nc,
                                                const QByteArray& rawDiskDigest);
    void invalidateNcPreflightEvidence();

private:
    void createUi();
    void createMenus();
    
    // 新布局：左侧视图区域
    QWidget* createLeftViewPanel(QWidget* parent);
    
    // 新布局：右侧控制区域
    QWidget* createRightControlPanel(QWidget* parent);
    QWidget* createConnectionSection(QWidget* parent);
    QWidget* createParametersSection(QWidget* parent);
    QWidget* createControlSection(QWidget* parent);
    QWidget* createKinematicsSection(QWidget* parent);
    
    // 传感标定区域（V27.6.16R: frame-driven multi-sample exposure calibration）
    QWidget* createVideoControlSection(QWidget* parent);
    int sensorCalibrationRequestedRateHz() const;
    int sensorCalibrationIdleUnitsForExposure(int exposureUnits) const; // Profile only
    int sensorCalibrationVideoIdleUnitsForExposure(int exposureUnits) const;
    bool validateSensorCalibrationVideoTiming(int exposureUnits, QString* errMsg = nullptr) const;
    bool validateSensorCalibrationTiming(int exposureUnits, int requestedHz, QString* errMsg = nullptr) const;
    void updateSensorCalibrationTimingUi();
    bool verifySensorCalibrationRuntimeConstraints(const SensorParams& requested, QString* errMsg = nullptr, bool checkProfileResolution = false);
    int sensorCalibrationSelectedFieldIndex() const;
    bool enterSensorCalibrationMeasuringField(QString* errMsg = nullptr);
    void leaveSensorCalibrationMeasuringField();
    SensorAdvancedParams currentSensorAdvancedUiParams() const;
    bool writeSensorCalibrationSessionManifest(QString* errMsg = nullptr);
    void loadMaterialExposureLibrary();
    void refreshMaterialExposureProfileUi(const QString& selectKey = QString());
    bool runExposureAnalysisAndImport(const QString& sessionDir, bool showDialog, QString* summary = nullptr);
    void applyMaterialExposureProfileByKey(const QString& key);

    void initializeMachineDigitalTwin();
    void updateMachineDigitalTwinPose(const JointPose5D& pose, bool evaluateCollision = true);
    bool sphereStudySafetyReady(QString* error = nullptr) const;
    bool resolveSphereStudyPlanningLimits(bool intersectLiveControllerLimits,
                                          mls::calibration::SphereStudyPathOptions& options,
                                          QString* source = nullptr,
                                          QString* error = nullptr) const;
    bool checkSphereStudyPoseSafety(const JointPose5D& pose, QString* error = nullptr, bool updateView = false);
    bool checkSphereStudyTransitionSafety(const JointPose5D& from, const JointPose5D& to, QString* error = nullptr, bool updateView = false);

    // V27.6.47 hardware-temporal calibration executor.  The calibration widget
    // creates the plan; MainWindow remains the only owner of MachineSafety,
    // motion and the formal V32.11 hardware-sync start/stop sequence.
    void beginHardwareTemporalPlannedScan(quint64 requestId,
                                          const JointPose5D& start,
                                          const JointPose5D& end,
                                          double feedMmPerSec);
    void prepareHardwareTemporalScanAfterPreposition(quint64 generation, bool success,
                                                     const JointMoveResult5D& result,
                                                     const QString& message);
    bool startHardwareTemporalRawCapture(QString* error);
    void pollHardwareTemporalSyncReady(quint64 generation);
    void startHardwareTemporalLineMotion(quint64 generation);
    void pollHardwareTemporalLineComplete(quint64 generation);
    void finishHardwareTemporalPlannedScan(quint64 generation, bool success,
                                           const QString& message);
    void abortHardwareTemporalPlannedScan();
    bool sphereStudyArrivalWithinTolerance(const JointMoveResult5D& result, QString* error = nullptr) const;
    int applySphereStudySafetyFilterAndTransitionGate(const JointPose5D& startPose,
                                                       bool gateStartTransition = true);

    void updateSphereStudyUiState();
    int sphereStudySelectedPlanIndex() const;
    void rebuildSphereStudyPlanTable();
    void onSphereStudyProfileFrame(const LaserProfile& profile);
    bool startSphereStudyCaptureGroup(QString* errMsg = nullptr);
    bool beginSphereStudyCaptureAtPlanIndex(int planIndex, bool interactive, QString* errMsg = nullptr);
    void finishSphereStudyCapture(bool cancelled, const QString& reason = QString());
    bool writeSphereStudyRawGroup(int recordIndex, QString* errMsg = nullptr) const;
    void advanceSphereStudyAutoSequence();
    void startSphereStudyAutoMoveCurrent();
    void startSphereStudyAutoProbeCurrent(int planIndex);
    void onSphereStudyProbeProfileFrame(const LaserProfile& profile);
    void finishSphereStudyAutoProbe(bool timeout = false);
    void startSphereStudyAutoReacquireMove(int planIndex, const mls::calibration::Vec3d& sensorOffsetMm);
    mls::calibration::Vec3d sphereStudyReacquireOffsetForAttempt(int attempt) const;
    mls::calibration::SphereStudyPathOptions makeSphereStudyOpticalOptions(const JointPose5D& reference) const;
    bool refreshSphereStudyPlanningModel(QString* error = nullptr);
    bool showSphereStudySimulationPose(int planIndex, bool selectRow, QString* error = nullptr);
    void advanceSphereStudySimulation();
    void finishSphereStudySimulation(const QString& reason, bool restoreActualPose = true);
    bool ensureSphereStudyProfileStream(QString* errMsg = nullptr);
    bool makeSphereStudyRawProfile(const LaserProfile& profile, mls::calibration::SphereStudyRawProfile& raw) const;
    void handleSphereStudyAutoStepFailure(const QString& reason);
    void finishSphereStudyAuto(bool cancelled, const QString& reason = QString());
    int sphereStudyCapturedGroupCount(int planIndex) const;
    
    void sphereExposureAdvance();
    void sphereExposureStartMove(bool preposition);
    void sphereExposureMoveFinished(quint64 generation, bool preposition,
                                     bool success, const JointMoveResult5D& result,
                                     const QString& error);
    void sphereExposureEndPass(bool success, const QString& error);
    void sphereExposureFinish(bool success, const QString& reason);
    void sphereExposureSetStatus(const QString& message, bool warning = false);
    bool sphereExposurePlanValid(QString* reason = nullptr) const;

    // Experiment 3: optically admissible Z, real scanned-X peak, static refinement,
    // independent exposure and Profile/optional Video capture per height.
    QWidget* createSmall7ExperimentPanel(QWidget* parent);
    void small7InvalidatePlan();
    void small7SetStatus(const QString& message, bool error = false);
    void onPlanSmall7Experiment();
    void onStartSmall7Experiment();
    void onStopSmall7Experiment();
    void small7AdvanceHeight();
    enum class Small7Phase { Idle, ZPosition, XPosition, ScanWarmup, XScan,
                             ProbePosition, ProbeCapture, ExposurePosition,
                             ExposureCapture, VideoCapture };
    void small7Move(int axis, double targetMpos, Small7Phase phase);
    void small7PositionHeartbeat(quint64 generation, quint64 moveToken, int axis, double targetMpos);
    void small7MoveFinished(quint64 generation, int axis, double targetMpos,
                            Small7Phase phase, bool success,
                            const JointMoveResult5D& result, const QString& error);
    bool small7BeginCapture(bool scanning, int exposure10us, int goodFrames, QString* error);
    void small7StopCapture();
    void small7OnProfile(const LaserProfile& profile);
    void small7SensorWatchdog(quint64 captureToken);
    void small7CaptureComplete();
    void small7StartExposureGroup();
    void small7StartVideoGroup();
    void small7StopVideo();
    void small7FinishExposurePass();
    void small7Finish(bool success, const QString& reason);

    void connectAxisSignals(AxisControlWidget* widget);
    void connectController(bool interactive);
    void startRotaryAbsoluteInitialization(bool interactive);
    bool runAutomaticHardwareSyncPreflight(bool showFailureDialog);
    void tryFinishDeferredClose();
    AxisControlWidget* axisWidgetForId(int axisId) const;
    void pollAxisAlarmState(int axisId, bool forceLog = false);
    void pollAxisAlarmStates(bool forceLog = false);
    void resetAxisAlarmMonitor();

    struct AxisMoveDiagnosticSnapshot {
        bool valid = false;
        double dpos = 0.0;
        double mpos = 0.0;
        bool encoderValid = false;
        double encoder = std::numeric_limits<double>::quiet_NaN();
        bool idle = false;
        bool linearReferenced = false;
        std::uint64_t mposSetGeneration = 0;
        int datumInput = -1;
        int datumRaw = -1;
        QString utc;
        QString error;
    };

    struct AxisMoveDiagnosticRecord {
        int axisId = -1;
        QString command;
        QString phase;
        QString plan;
        QString role;
        QString stepLabel;
        int cycle = 0;
        double testBaseMm = std::numeric_limits<double>::quiet_NaN();
        double testDistanceMm = std::numeric_limits<double>::quiet_NaN();
        int testDirection = 0; // +1 increasing coordinate, -1 decreasing coordinate
        double requestedTarget = std::numeric_limits<double>::quiet_NaN();
        double requestedDelta = std::numeric_limits<double>::quiet_NaN();
        int settleMs = 0;
        AxisMoveDiagnosticSnapshot start;
        AxisMoveDiagnosticSnapshot endIdle;
        AxisMoveDiagnosticSnapshot endSettled;
        bool moveOk = false;
        QString moveError;
    };

    struct AxisMoveDiagnosticAutoStep {
        bool relative = false;
        double value = 0.0;
        QString role;
        QString label;
        int cycle = 0;
        double testBaseMm = std::numeric_limits<double>::quiet_NaN();
        double testDistanceMm = std::numeric_limits<double>::quiet_NaN();
        int testDirection = 0;
    };

    AxisMoveDiagnosticSnapshot captureAxisMoveDiagnosticSnapshot(ZmcController* controller, int axisId) const;
    void appendAxisMoveDiagnosticRecord(const AxisMoveDiagnosticRecord& record);
    void startManualAxisMoveDiagnostic();
    void finishManualAxisMoveDiagnostic();
    void startAutomaticAxisMoveDiagnostic();
    void stopAutomaticAxisMoveDiagnostic();
    QVector<AxisMoveDiagnosticAutoStep> buildAutomaticAxisMoveDiagnosticPlan(
        QString phase,
        QString plan,
        int repeats,
        const QVector<double>& basePositions,
        const QVector<double>& testDistances) const;
    void updateAutomaticAxisMoveDiagnosticUi();
    void saveAxisMoveDiagnosticsCsv();
    void clearAxisMoveDiagnostics();

    struct AxisCompensationSpeedTestSample {
        int axisId = -1;
        QString stage; // PRECISION or COARSE
        double candidateSpeedMmS = std::numeric_limits<double>::quiet_NaN();
        double coarseSpeedMmS = std::numeric_limits<double>::quiet_NaN();
        double precisionSpeedMmS = std::numeric_limits<double>::quiet_NaN();
        int direction = 0;
        int cycle = 0;
        int targetIndex = 0;
        double targetDposMm = std::numeric_limits<double>::quiet_NaN();
        double preloadDposMm = std::numeric_limits<double>::quiet_NaN();
        int settleMs = 0;
        AxisMoveDiagnosticSnapshot endIdle;
        AxisMoveDiagnosticSnapshot endSettled;
        bool moveOk = false;
        QString error;
    };

    struct AxisCompensationSpeedProfile {
        bool qualified = false;
        int axisId = -1;
        double coarseSpeedMmS = std::numeric_limits<double>::quiet_NaN();
        double precisionSpeedMmS = std::numeric_limits<double>::quiet_NaN();
        double precisionP95DriftMm = std::numeric_limits<double>::quiet_NaN();
        double precisionMaxDriftMm = std::numeric_limits<double>::quiet_NaN();
        double precisionRepeatabilityMm = std::numeric_limits<double>::quiet_NaN();
        double coarseP95DriftMm = std::numeric_limits<double>::quiet_NaN();
        double coarseMaxDriftMm = std::numeric_limits<double>::quiet_NaN();
        double coarseRepeatabilityMm = std::numeric_limits<double>::quiet_NaN();
        double rangeStartMm = std::numeric_limits<double>::quiet_NaN();
        double rangeEndMm = std::numeric_limits<double>::quiet_NaN();
        double preloadMm = std::numeric_limits<double>::quiet_NaN();
        QString status;
    };

    void startAxisCompensationSpeedQualification();
    void stopAxisCompensationSpeedQualification();
    void updateAxisCompensationSpeedQualificationUi();
    void updateAxisCompensationSpeedProfileTable();
    bool saveAxisCompensationSpeedQualificationFiles(const QString& outputRoot,
                                                      QString* savedRunDirectory,
                                                      QString* error = nullptr) const;

    struct AxisCompensationCalibrationSample {
        int axisId = -1;
        int direction = 0; // +1 increasing-coordinate scan, -1 decreasing-coordinate scan
        int cycle = 0;
        int pointIndex = 0;
        QString approachMode; // PRIME: one direction-establishing preload; SWEEP: monotonic precision step
        double requestedDposMm = std::numeric_limits<double>::quiet_NaN();
        double preloadDposMm = std::numeric_limits<double>::quiet_NaN();
        double coarseSpeedMmS = std::numeric_limits<double>::quiet_NaN();
        double precisionSpeedMmS = std::numeric_limits<double>::quiet_NaN();
        int settleMs = 0;
        AxisMoveDiagnosticSnapshot endIdle;
        AxisMoveDiagnosticSnapshot endSettled;
        bool moveOk = false;
        QString error;
    };

    struct AxisCompensationTableNode {
        int axisId = -1;
        int direction = 0;
        double nominalDposMm = std::numeric_limits<double>::quiet_NaN();
        double requiredDposMm = std::numeric_limits<double>::quiet_NaN();
        double desiredMposMm = std::numeric_limits<double>::quiet_NaN();
        double correctionMm = std::numeric_limits<double>::quiet_NaN();
        double correctionStdDevMm = std::numeric_limits<double>::quiet_NaN();
        double mposStdDevMm = std::numeric_limits<double>::quiet_NaN();
        double meanPostIdleMposDriftMm = std::numeric_limits<double>::quiet_NaN();
        int sampleCount = 0;
    };

    void startAxisCompensationCalibration();
    void stopAxisCompensationCalibration();
    void updateAxisCompensationCalibrationUi();
    void fillAxisCompensationRangesFromSoftLimits();
    void appendAxisCompensationCalibrationSample(const AxisCompensationCalibrationSample& sample);
    QVector<AxisCompensationTableNode> buildAxisCompensationTableNodes() const;
    void updateAxisCompensationSummaryTable();
    bool saveAxisCompensationCalibrationFiles(const QString& outputRoot,
                                              QString* savedRunDirectory,
                                              QString* error = nullptr) const;
    void saveAxisCompensationCalibrationFilesInteractive();

    enum AxisCompensationValidationMethod {
        AxisCompValAuto = 0,
        AxisCompValPointToPoint = 1,   // legacy: every validation point gets its own preload
        AxisCompValMonotonicSweep = 2 // V27.6.33+: one PRIME then monotonic off-grid sweep
    };

    struct AxisCompensationValidationRecord {
        int axisId = -1;
        int direction = 0; // +1 increasing-coordinate approach, -1 decreasing-coordinate approach
        int cycle = 0;
        int targetIndex = 0;
        QString mode; // RAW or COMPENSATED
        QString validationMethod; // POINT_TO_POINT_PRELOAD or MONOTONIC_SWEEP
        QString approachMode; // PRIME / SWEEP for monotonic; POINT for legacy
        double targetMposMm = std::numeric_limits<double>::quiet_NaN();
        double commandedDposMm = std::numeric_limits<double>::quiet_NaN();
        double preloadDposMm = std::numeric_limits<double>::quiet_NaN();
        double coarseSpeedMmS = std::numeric_limits<double>::quiet_NaN();
        double precisionSpeedMmS = std::numeric_limits<double>::quiet_NaN();
        int settleMs = 0;
        AxisMoveDiagnosticSnapshot endIdle;
        AxisMoveDiagnosticSnapshot endSettled;
        bool moveOk = false;
        QString error;
    };

    struct AxisCompensationValidationStats {
        int sampleCount = 0;
        double meanErrorMm = std::numeric_limits<double>::quiet_NaN();
        double rmsMm = std::numeric_limits<double>::quiet_NaN();
        double maeMm = std::numeric_limits<double>::quiet_NaN();
        double p95AbsMm = std::numeric_limits<double>::quiet_NaN();
        double maxAbsMm = std::numeric_limits<double>::quiet_NaN();
    };

    bool loadAxisCompensationValidationTable(const QString& filePath, QString* error = nullptr);
    int resolvedAxisCompensationValidationMethod() const;
    QString axisCompensationValidationMethodLabel(int method) const;
    bool interpolateAxisCompensationCommand(int direction, double targetMposMm, double& requiredDposMm, QString* error = nullptr) const;
    QVector<double> buildAxisCompensationValidationTargets(int directionSelection, int pointCount, double preloadMm, QString* error = nullptr) const;
    void startAxisCompensationValidation();
    void stopAxisCompensationValidation();
    void updateAxisCompensationValidationUi();
    void appendAxisCompensationValidationRecord(const AxisCompensationValidationRecord& record);
    AxisCompensationValidationStats computeAxisCompensationValidationStats(int direction, const QString& mode) const;
    void updateAxisCompensationValidationSummary();
    bool saveAxisCompensationValidationFiles(const QString& outputRoot, QString* savedRunDirectory, QString* error = nullptr) const;
    void saveAxisCompensationValidationFilesInteractive();

    void fillAxisTableFromParams();
    void readParamsFromTable();
    bool buildPathFromTable(std::vector<PathPoint>& outPath);
    std::array<AxisParams, Constants::kAxisParamArraySize> currentAxisParams5D() const;
    bool buildCurrentJointMovePlan(JointMotionPlan5D& plan, QString* err);
    void setJointMoveUiRunning(bool running);

    void updateLastProfile(const LaserProfile& profile);
    
    // Hardware-sync rate is an operator parameter; the allowed maximum follows
    // the selected scanCONTROL Measuring Field Quick Reference limit.
    int hardwareSyncFrequencyHz() const;
    void updateHardwareSyncFrequencyLimit();
    void stageGCodeHardwareSettings(const GCodeProgram& nc); // in-process only
    void finishGCodeHardwareNcProbe(bool explicitStop=false);

    // 更新 UI 状态
    void updateVideoControlState();
    // Keep scan reconstruction and RTCP on the same independently validated,
    // live-convention-bound Physical17 model. Never hot-swap while a task is running.
    void syncValidatedCalibrationToRuntime(bool usable, bool announce = false);
    
    // 数据分析
    QString analyzeHardwareSyncData();
    QString analyzeSoftwareSyncData();

private:
    // 核心组件
    std::unique_ptr<ZmcController> m_zmc;
    std::unique_ptr<LaserSensor> m_laser;
    std::unique_ptr<SyncManager> m_syncManager;
    std::unique_ptr<PointCloudBuilder> m_cloudBuilder;
    std::unique_ptr<mls::safety::MachineSafetyModel> m_machineSafety;
    std::unique_ptr<mls::calibration::MaterialExposureLibrary> m_materialExposureLibrary;
    QString m_machineModelConfigPath;
    bool m_machineSafetyReady = false;
    quint64 m_machineCollisionUiCounter = 0;

    // 轴参数缓存
    AxisParams m_axisParams[Constants::kAxisParamArraySize];

    // 定时器
    QTimer m_statusTimer;
    QTimer m_posTimer;
    QTimer m_profileUiTimer;

    // AXISSTATUS 事件式监视：不做周期轮询，只在连接、运动结束、使能、停止等事件点读取。
    std::array<int, Constants::kAxisParamArraySize> m_lastAxisStatus{};
    std::array<bool, Constants::kAxisParamArraySize> m_axisStatusSeen{};
    std::array<bool, Constants::kAxisParamArraySize> m_axisStatusReadFailed{};

    // ===== UI 控件 =====

    // 控制器连接
    QLineEdit* m_controllerIpEdit = nullptr;
    QPushButton* m_btnControllerConnect = nullptr;
    QPushButton* m_btnControllerDisconnect = nullptr;
    QLabel* m_lblControllerStatus = nullptr;
    
    // 传感器连接
    QLineEdit* m_sensorIpEdit = nullptr;
    QPushButton* m_btnSensorConnect = nullptr;
    QPushButton* m_btnSensorDisconnect = nullptr;
    QLabel* m_lblSensorStatus = nullptr;

    // 轴参数表格
    QTableWidget* m_axisTable = nullptr;
    QPushButton* m_btnReadParams = nullptr;
    QPushButton* m_btnWriteAxis = nullptr;
    QPushButton* m_btnWriteAll = nullptr;
    
    // 传感器参数
    QComboBox* m_comboResolution = nullptr;
    QComboBox* m_comboMaterialExposureProfile = nullptr;
    QPushButton* m_btnReloadMaterialExposureLibrary = nullptr;
    QSpinBox* m_spinExposure = nullptr;
    QSpinBox* m_spinIdle = nullptr;
    QSpinBox* m_spinHardwareSyncFrequency = nullptr;
    QComboBox* m_comboHardwareTopology = nullptr;
    QComboBox* m_comboCmmR1Edge = nullptr;
    QRadioButton* m_radioSoftwareSync = nullptr;
    QRadioButton* m_radioHardwareSync = nullptr;
    QComboBox* m_comboTriggerType = nullptr;
    bool m_hardwareSyncPreflightPassed = false; // valid only for current controller + XYZ MPOS reference + B/C absolute frame
    
    // 传感器高级参数
    QComboBox* m_comboMeasuringField = nullptr;  // 测量区域
    QComboBox* m_comboLaserPower = nullptr;
    QComboBox* m_comboReflection = nullptr;
    QDoubleSpinBox* m_spinThreshold = nullptr;
    QCheckBox* m_checkFlipX = nullptr;
    QCheckBox* m_checkFlipZ = nullptr;
    QCheckBox* m_checkAutoExposure = nullptr;
    QComboBox* m_comboShutterAlign = nullptr;
    QComboBox* m_comboAutoExpMode = nullptr;
    QSpinBox* m_spinPeakWidthMin = nullptr;
    QSpinBox* m_spinPeakWidthMax = nullptr;
    QSpinBox* m_spinPeakIntensityMin = nullptr;
    QSpinBox* m_spinPeakIntensityMax = nullptr;
    QCheckBox* m_checkHighResolution = nullptr;
    QCheckBox* m_checkCalibration = nullptr;
    QCheckBox* m_checkAmbientLight = nullptr;
    QCheckBox* m_checkVideoFilter = nullptr;
    QPushButton* m_btnReadSensorParams = nullptr;
    QPushButton* m_btnWriteSensorParams = nullptr;
    QPushButton* m_btnResetSensorParams = nullptr;

    // V27.6.53: active hardware-temporal calibration display + runtime switch.
    QLabel* m_lblHardwareTemporalStatus = nullptr;
    QLabel* m_lblHardwareTemporalValue = nullptr;
    QLabel* m_lblHardwareTemporalConfig = nullptr;
    QCheckBox* m_checkHardwareTemporalRuntime = nullptr;
    bool m_refreshingHardwareTemporalUi = false;

    // 单轴控制
    AxisControlWidget* m_xAxisWidget = nullptr;
    AxisControlWidget* m_yAxisWidget = nullptr;
    AxisControlWidget* m_zAxisWidget = nullptr;
    AxisControlWidget* m_cAxisWidget = nullptr;
    AxisControlWidget* m_bAxisWidget = nullptr;

    // DPOS/MPOS 单轴位移诊断：自动记录绝对/相对定位的起点、IDLE后立即值和稳定等待后的值。
    QCheckBox* m_checkAxisMoveDiagAuto = nullptr;
    QSpinBox* m_spinAxisMoveDiagSettleMs = nullptr;
    QComboBox* m_comboAxisMoveDiagManualAxis = nullptr;
    QPushButton* m_btnAxisMoveDiagManualStart = nullptr;
    QPushButton* m_btnAxisMoveDiagManualEnd = nullptr;
    QPushButton* m_btnAxisMoveDiagClear = nullptr;
    QPushButton* m_btnAxisMoveDiagSave = nullptr;
    QComboBox* m_comboAxisMoveDiagAutoAxis = nullptr;
    QComboBox* m_comboAxisMoveDiagPhase = nullptr;
    QComboBox* m_comboAxisMoveDiagPlan = nullptr;
    QLineEdit* m_editAxisMoveDiagBasePositions = nullptr;
    QLineEdit* m_editAxisMoveDiagDistances = nullptr;
    QSpinBox* m_spinAxisMoveDiagRepeats = nullptr;
    QPushButton* m_btnAxisMoveDiagAutoStart = nullptr;
    QPushButton* m_btnAxisMoveDiagAutoStop = nullptr;
    QProgressBar* m_progressAxisMoveDiagAuto = nullptr;
    QLabel* m_lblAxisMoveDiagPlan = nullptr;
    QLabel* m_lblAxisMoveDiagStatus = nullptr;
    QTableWidget* m_axisMoveDiagTable = nullptr;
    QVector<AxisMoveDiagnosticRecord> m_axisMoveDiagRecords;
    AxisMoveDiagnosticSnapshot m_axisMoveDiagManualStart;
    int m_axisMoveDiagManualAxis = -1;
    std::atomic_bool m_axisMoveDiagAutoCancel{false};
    bool m_axisMoveDiagAutoRunning = false;

    // V27.6.33: qualify per-axis motion speeds before collecting compensation tables.
    // The speed qualifier identifies (1) a fast automatic/coarse positioning speed and
    // (2) a precision final-approach speed.  Calibration then reproduces this two-stage
    // approach at every table node.
    QLineEdit* m_editAxisCompSpeedPrecisionCandidates = nullptr;
    QSpinBox* m_spinAxisCompSpeedTestPoints = nullptr;
    QSpinBox* m_spinAxisCompSpeedRepeats = nullptr;
    QDoubleSpinBox* m_spinAxisCompSpeedP95DriftUm = nullptr;
    QDoubleSpinBox* m_spinAxisCompSpeedMaxDriftUm = nullptr;
    QDoubleSpinBox* m_spinAxisCompSpeedRepeatabilityUm = nullptr;
    QTableWidget* m_axisCompSpeedTable = nullptr;
    QPushButton* m_btnAxisCompSpeedStart = nullptr;
    QPushButton* m_btnAxisCompSpeedStop = nullptr;
    QProgressBar* m_progressAxisCompSpeed = nullptr;
    QLabel* m_lblAxisCompSpeedStatus = nullptr;
    QVector<AxisCompensationSpeedTestSample> m_axisCompSpeedSamples;
    std::array<AxisCompensationSpeedProfile, Constants::kAxisParamArraySize> m_axisCompSpeedProfiles{};
    QString m_axisCompSpeedLastRunDir;
    std::atomic_bool m_axisCompSpeedCancel{false};
    bool m_axisCompSpeedRunning = false;

    // XYZ absolute position-compensation table calibration.  V27.6.33 retains the
    // candidate-table acquisition but now requires a qualified two-speed profile
    // for every selected axis before motion starts.
    QComboBox* m_comboAxisCompCalAxis = nullptr;
    QTableWidget* m_axisCompCalRangeTable = nullptr;
    QSpinBox* m_spinAxisCompCalRepeats = nullptr;
    QSpinBox* m_spinAxisCompCalSettleMs = nullptr;
    QDoubleSpinBox* m_spinAxisCompCalPreloadMm = nullptr;
    QLineEdit* m_editAxisCompCalOutputDir = nullptr;
    QPushButton* m_btnAxisCompCalUseSoftLimits = nullptr;
    QPushButton* m_btnAxisCompCalBrowseDir = nullptr;
    QPushButton* m_btnAxisCompCalStart = nullptr;
    QPushButton* m_btnAxisCompCalStop = nullptr;
    QPushButton* m_btnAxisCompCalSave = nullptr;
    QProgressBar* m_progressAxisCompCal = nullptr;
    QLabel* m_lblAxisCompCalStatus = nullptr;
    QTableWidget* m_axisCompCalSummaryTable = nullptr;
    QVector<AxisCompensationCalibrationSample> m_axisCompCalSamples;
    std::atomic_bool m_axisCompCalCancel{false};
    bool m_axisCompCalRunning = false;

    // V27.6.31 candidate compensation A/B validation.  This remains an
    // isolated verification path: it never enables runtime/global compensation.
    QLineEdit* m_editAxisCompValTablePath = nullptr;
    QPushButton* m_btnAxisCompValLoad = nullptr;
    QLabel* m_lblAxisCompValTableInfo = nullptr;
    QComboBox* m_comboAxisCompValMethod = nullptr;
    QComboBox* m_comboAxisCompValDirection = nullptr;
    QSpinBox* m_spinAxisCompValPoints = nullptr;
    QSpinBox* m_spinAxisCompValRepeats = nullptr;
    QSpinBox* m_spinAxisCompValSettleMs = nullptr;
    QDoubleSpinBox* m_spinAxisCompValPreloadMm = nullptr;
    QLineEdit* m_editAxisCompValOutputDir = nullptr;
    QPushButton* m_btnAxisCompValBrowseDir = nullptr;
    QPushButton* m_btnAxisCompValStart = nullptr;
    QPushButton* m_btnAxisCompValStop = nullptr;
    QPushButton* m_btnAxisCompValSave = nullptr;
    QProgressBar* m_progressAxisCompVal = nullptr;
    QLabel* m_lblAxisCompValStatus = nullptr;
    QTableWidget* m_axisCompValSummaryTable = nullptr;
    QVector<AxisCompensationTableNode> m_axisCompValNodes;
    QVector<AxisCompensationValidationRecord> m_axisCompValRecords;
    QString m_axisCompValSourcePath;
    QString m_axisCompValSourceMotionStrategy;
    int m_axisCompValAutoMethod = AxisCompValPointToPoint;
    int m_axisCompValLastResolvedMethod = AxisCompValPointToPoint;
    int m_axisCompValAxis = -1;
    std::atomic_bool m_axisCompValCancel{false};
    bool m_axisCompValRunning = false;

    // Separate from the legacy fixed-pose exposure test; never shares its timer
    // or modifies hardware H1/H3 capture, FIFO, or CMM configuration.
    QPushButton* m_btnSphereExpImport = nullptr;
    QPushButton* m_btnSphereExpPlan = nullptr;
    QPushButton* m_btnSphereExpStart = nullptr;
    QPushButton* m_btnSphereExpStop = nullptr;
    QDoubleSpinBox* m_spinSphereExpSpeed = nullptr;
    QComboBox* m_comboSphereExpPositionSpeed = nullptr;
    QDoubleSpinBox* m_spinSphereExpMargin = nullptr;
    QDoubleSpinBox* m_spinSphereExpMaxStep = nullptr;
    QSpinBox* m_spinSphereExpRepeats = nullptr;
    QLabel* m_lblSphereExpStatus = nullptr;
    QTableWidget* m_tableSphereExpRanking = nullptr;
    QTableWidget* m_tableSphereExpPlan = nullptr;
    bool m_sphereExpPlanValid = false;
    JointPose5D m_sphereExpStartPose;
    JointPose5D m_sphereExpEndPose;
    JointPose5D m_sphereExpMiddlePose;
    double m_sphereExpPredCx = 0.0;
    double m_sphereExpPredCz = 0.0;
    double m_sphereExpPhysical17Gradient = 0.0;
    bool m_sphereExpActive = false;
    bool m_sphereExpMotionWorker = false;
    bool m_sphereExpCapturing = false;
    bool m_sphereExpStopRequested = false;
    quint64 m_sphereExpGeneration = 0;
    int m_sphereExpPass = 0;
    int m_sphereExpTotalPasses = 0;
    int m_sphereExpRepeatCount = 2;
    int m_sphereExpExposureStart = 0;
    int m_sphereExpExposureStep = 0;
    int m_sphereExpRateHz = 0;
    double m_sphereExpFeedMmPerSec = 0.0;
    double m_sphereExpKnownDiameterMm = 19.05;
    QString m_sphereExpSessionDir;
    QString m_sphereExpCurrentPrefix;
    QFile* m_sphereExpDataFile = nullptr;
    QFile* m_sphereExpMetricsFile = nullptr;
    bool m_sphereExpFrameGap = false;
    bool m_sphereExpHasProfileIndex = false;
    quint32 m_sphereExpLastProfileIndex = 0;
    QMetaObject::Connection m_sphereExpProfileConnection;
    std::vector<mls::calibration::SphereExposureFrame> m_sphereExpFrames;
    std::vector<mls::calibration::SphereExposurePass> m_sphereExpPassResults;

    QDoubleSpinBox* m_spinSmall7ZStart = nullptr;
    QDoubleSpinBox* m_spinSmall7ZEnd = nullptr;
    QDoubleSpinBox* m_spinSmall7ZInterval = nullptr;
    QDoubleSpinBox* m_spinSmall7MaxSpacing = nullptr;
    QDoubleSpinBox* m_spinSmall7LineSpacing = nullptr;
    QDoubleSpinBox* m_spinSmall7DiameterTolerance = nullptr;
    QLabel* m_lblSmall7DerivedSpeed = nullptr;
    QSpinBox* m_spinSmall7ExpStart = nullptr;
    QSpinBox* m_spinSmall7ExpEnd = nullptr;
    QSpinBox* m_spinSmall7ExpStep = nullptr;
    QSpinBox* m_spinSmall7VideoFrames = nullptr;
    QCheckBox* m_checkSmall7SaveVideo = nullptr;
    QLineEdit* m_editSmall7OutputDir = nullptr;
    QComboBox* m_comboSmall7PositionSpeed = nullptr;
    QComboBox* m_comboSmall7Field = nullptr;
    QString m_small7SuggestedOpticalConfigPath;
    QSpinBox* m_spinSmall7Rate = nullptr;
    QSpinBox* m_spinSmall7ProbeExposure = nullptr;
    QSpinBox* m_spinSmall7ProbeFrames = nullptr;
    QSpinBox* m_spinSmall7ExposureFrames = nullptr;
    QSpinBox* m_spinSmall7StaticDiscard = nullptr;
    QSpinBox* m_spinSmall7Repeats = nullptr;
    QLineEdit* m_editSmall7OpticalConfig = nullptr;
    QLabel* m_lblSmall7Status = nullptr;
    QTableWidget* m_tableSmall7Plan = nullptr;
    QPushButton* m_btnSmall7Plan = nullptr;
    QPushButton* m_btnSmall7Start = nullptr;
    QPushButton* m_btnSmall7Stop = nullptr;
    QPushButton* m_btnSmall7Browse = nullptr;
    bool m_small7PlanValid = false;
    bool m_small7Active = false;
    bool m_small7WorkerActive = false;
    bool m_small7StopRequested = false;
    bool m_small7Capturing = false;
    bool m_small7FrameGap = false;
    int m_small7ProfileGapEvents = 0;
    int m_small7ProfileMissingEstimate = 0;
    bool m_small7HasProfileIndex = false;
    bool m_small7CompletionQueued = false;
    quint32 m_small7PreviousProfileIndex = 0;
    quint64 m_small7Generation = 0;
    quint64 m_small7CaptureToken = 0;
    quint64 m_small7MoveToken = 0;
    qint64 m_small7MoveStartedMs = 0;
    qint64 m_small7LastProfileMs = 0;
    int m_small7ScanMotionProfiles = 0;
    int m_small7ScanMotionGoodFrames = 0;
    struct Small7ScanProfile { qint64 pcUs=0; double radiusMm=0; };
    struct Small7AxisSample { qint64 pcUs=0; double machineX=0; };
    std::vector<Small7ScanProfile> m_small7ScanProfiles;
    std::vector<qint64> m_small7ScanAllFramePcUs;
    std::vector<Small7AxisSample> m_small7AxisSamples;
    std::array<double,7> m_small7ProbePositions{};
    double m_small7ScanPeakX = std::numeric_limits<double>::quiet_NaN();
    double m_small7ScanPeakRadius = 0.0;
    qint64 m_small7PreviousFrameUs = 0;
    double m_small7MaxObservedFrameSpacingMm = 0.0;
    int m_small7HeightIndex = 0;
    int m_small7ProbeIndex = 0;
    int m_small7ExposurePass = 0;
    int m_small7ReceivedFrames = 0;
    int m_small7StaticDiscardFrames = 10;
    int m_small7GoodFrames = 0;
    int m_small7RequiredFrames = 0;
    int m_small7ExpStart = 0;
    int m_small7ExpEnd = 0;
    int m_small7ExpStep = 1;
    int m_small7ExpRepeats = 2;
    int m_small7Rate = 200;
    int m_small7ProbeExposure = 145;
    int m_small7ProbeFrames = 30;
    int m_small7ExposureFrames = 50;
    double m_small7LastExposureRadiusMm = 0;
    int m_small7LastExposureGoodFrames = 0;
    int m_small7LastExposureReceivedFrames = 0;
    int m_small7LastExposureGapEvents = 0;
    int m_small7LastExposureMissingEstimate = 0;
    double m_small7DiameterToleranceMm = 0.50;
    bool m_small7VideoEnabled = false;
    int m_small7VideoFramesPerPass = 0;
    int m_small7VideoReceived = 0;
    int m_small7VideoSaved = 0;
    quint64 m_small7VideoToken = 0;
    QMetaObject::Connection m_small7VideoConnection;
    double m_small7PredictedCx = 0.0;
    double m_small7PredictedCz = 0.0;
    double m_small7KnownSphereRadius = 9.525;
    double m_small7ChosenPeakX = std::numeric_limits<double>::quiet_NaN();
    JointPose5D m_small7BasePose;
    std::array<double,17> m_small7PlannedPhysical17{};
    mls::calibration::Vec3d m_small7PlannedSphereCenter;
    Small7Phase m_small7Phase = Small7Phase::Idle;
    mls::calibration::small7::Settings m_small7Settings;
    mls::calibration::small7::OpticalWindow m_small7Optics;
    std::vector<mls::calibration::small7::HeightPlan> m_small7Heights;
    std::vector<double> m_small7Radii;
    std::array<double,7> m_small7ProbeBestRadii{};
    QString m_small7SessionDir;
    QString m_small7GroupPrefix;
    QString m_small7FaultHistory;
    QFile* m_small7Raw = nullptr;
    QFile* m_small7Metrics = nullptr;
    QMetaObject::Connection m_small7ProfileConnection;

    bool m_rotaryInitInProgress = false;
    bool m_controllerConnectInProgress = false;
    bool m_sensorConnectInProgress = false;
    // V27.6.47 automatic temporal-calibration path state.  While active the
    // normal merge-pump is gated until the V32.11 Probe has established sync, so
    // a pre-staged scan path cannot start before hardware acquisition is ready.
    bool m_hardwareTemporalScanActive = false;
    bool m_hardwareTemporalMotionStarted = false;
    quint64 m_hardwareTemporalGeneration = 0;
    quint64 m_hardwareTemporalRequestId = 0;
    JointPose5D m_hardwareTemporalStart;
    JointPose5D m_hardwareTemporalEnd;
    double m_hardwareTemporalFeedMmPerSec = 0.0;
    double m_hardwareTemporalPreparedDurationSec = 0.0;
    qint64 m_hardwareTemporalPhaseStartMs = 0;
    qint64 m_hardwareTemporalDeadlineMs = 0;

    int m_axisWorkerCount = 0;
    int m_backgroundIoCount = 0; // GUI-owned count; deferred close waits for callbacks
    bool m_positionPollPending = false;
    bool m_mergePumpPending = false;
    bool m_closeRequested = false;

    // 控制器不可达时，先用系统 ping 做异步网络预检查；只有 IP 确实响应时
    // 才在 GUI 线程调用 ZAux_OpenEth，避免“无控制器”场景把界面长时间卡死。
    QProcess* m_controllerProbeProcess = nullptr;

    // RS485 absolute-position reading and sensor discovery remain off the GUI thread.
    QFutureWatcher<void>* m_rotaryInitWatcher = nullptr;
    QFutureWatcher<void>* m_sensorConnectWatcher = nullptr;

    // 五轴运动学 / P4 五轴关节空间同步定位 UI
    QDoubleSpinBox* m_kinX = nullptr;
    QDoubleSpinBox* m_kinY = nullptr;
    QDoubleSpinBox* m_kinZ = nullptr;
    QDoubleSpinBox* m_kinC = nullptr;
    QDoubleSpinBox* m_kinB = nullptr;
    QDoubleSpinBox* m_kinSpeedScale = nullptr;
    QLabel* m_kinPlanSummary = nullptr;
    QPushButton* m_btnKinPlan = nullptr;
    QPushButton* m_btnKinMove = nullptr;
    QPushButton* m_btnKinStop = nullptr;
    QTextEdit* m_kinOutput = nullptr;
    bool m_jointMoveInProgress = false;

    // 全局控制按钮
    QPushButton* m_btnZeroAll = nullptr;
    QPushButton* m_btnHomeAll = nullptr;
    QPushButton* m_btnStopAll = nullptr;
    QPushButton* m_btnResyncRotaryAbs = nullptr;

    // 轨迹表格
    QTableWidget* m_pathTable = nullptr;
    QPushButton* m_btnLoadPath = nullptr;
    QPushButton* m_btnAddPoint = nullptr;
    QPushButton* m_btnRemovePoint = nullptr;
    QPushButton* m_btnStartMerge = nullptr;
    QPushButton* m_btnStopMerge = nullptr;

    // 采集控制 (Profile 模式)
    QPushButton* m_btnStartCapture = nullptr;
    QPushButton* m_btnStopCapture = nullptr;
    QPushButton* m_btnClearData = nullptr;
    QPushButton* m_btnAnalyzeData = nullptr;
    QLabel* m_lblCaptureStatus = nullptr;
    
    // Video 模式控制
    QGroupBox* m_videoControlGroup = nullptr;
    QSpinBox* m_spinSensorCalibRate = nullptr;
    QCheckBox* m_checkSensorCalibAutoFieldRate = nullptr;
    QSpinBox* m_spinVideoExposure = nullptr;
    QSpinBox* m_spinVideoIdle = nullptr;
    int m_sensorCalibExposureBeforeSweep = -1; // manual preview exposure before a test sweep
    QLabel* m_lblSensorCalibField = nullptr;
    int m_sensorCalibActiveFieldIndex = -1;
    QLabel* m_lblSensorCalibTiming = nullptr;
    bool m_sensorCalibFieldOverrideActive = false;
    int m_sensorCalibPreviousFieldIndex = 7;
    QPushButton* m_btnStartVideo = nullptr;
    QPushButton* m_btnStopVideo = nullptr;
    QLabel* m_lblVideoStatus = nullptr;
    
    // 轮廓预览模式控制
    QRadioButton* m_radioVideoMode = nullptr;
    QRadioButton* m_radioProfilePreviewMode = nullptr;
    QPushButton* m_btnStartProfilePreview = nullptr;
    QPushButton* m_btnStopProfilePreview = nullptr;
    QLabel* m_lblProfilePreviewStatus = nullptr;
    QStackedWidget* m_previewStack = nullptr;
    Profile2DWidget* m_profilePreviewView = nullptr;
    bool m_profilePreviewRunning = false;
    // True means display-only subscription to an already-running formal acquisition.
    // The preview NEVER owns or changes the external hardware trigger / CMM path.
    bool m_profilePreviewMirrorsCapture = false;
    quint64 m_profilePreviewMirrorBaselineOrdinal = 0;
    bool m_profilePreviewMirrorFirstFrameSeen = false;
    qint64 m_profilePreviewMirrorSinceMs = 0;
    
    // 曝光测试控件 - 基本参数
    QComboBox* m_comboLighting = nullptr;
    QLineEdit* m_editMaterial = nullptr;
    QSpinBox* m_spinExpStart = nullptr;
    QSpinBox* m_spinExpEnd = nullptr;
    QSpinBox* m_spinExpStep = nullptr;
    QSpinBox* m_spinStableDiscardFrames = nullptr;
    QSpinBox* m_spinVideoFramesPerExposure = nullptr;
    QSpinBox* m_spinProfileFramesPerExposure = nullptr;
    QPushButton* m_btnStartExpTest = nullptr;
    QPushButton* m_btnStopExpTest = nullptr;
    QPushButton* m_btnAnalyzeExpTestDir = nullptr;
    QLabel* m_lblExpTestStatus = nullptr;
    QLabel* m_lblExpAnalysisResult = nullptr;
    QLineEdit* m_editExpTestDir = nullptr;
    
    // 曝光测试控件 - 新增参数
    QDoubleSpinBox* m_spinTemperature = nullptr;      // 环境温度
    QDoubleSpinBox* m_spinPosX = nullptr;             // 机床位置X
    QDoubleSpinBox* m_spinPosY = nullptr;             // 机床位置Y
    QDoubleSpinBox* m_spinPosZ = nullptr;             // 机床位置Z (DPOS)
    QDoubleSpinBox* m_spinPosC = nullptr;             // 机床位置C (机械MPOS)
    QDoubleSpinBox* m_spinPosB = nullptr;             // 机床位置B (机械MPOS)
    QPushButton* m_btnReadPosition = nullptr;         // 读取当前XYZCB按钮
    
    // 曝光测试控件 - 保存选项
    QCheckBox* m_checkSaveProfile = nullptr;          // 保存轮廓数据
    QCheckBox* m_checkSaveVideo = nullptr;            // 保存Video图像
    
    // 曝光测试状态
    enum class ExpTestState {
        Idle,
        StartVideo,
        CaptureVideo,
        StopVideo,
        StartProfile,
        CaptureProfile,
        StopProfile,
        NextExposure,
        Complete
    };
    ExpTestState m_expTestState = ExpTestState::Idle;
    QTimer m_expTestTimer;
    int m_expTestCurrent = 0;      // 当前曝光值
    int m_expTestEnd = 0;          // 终止曝光值
    int m_expTestStep = 0;         // 曝光步进
    QString m_expTestDir;          // 保存目录
    QString m_expTestLighting;     // 光照条件
    QString m_expTestMaterial;     // 材料名称
    int m_expTestStableDiscardFrames = 3;      // 每种模式切换后先丢弃的真实新帧数
    int m_expTestVideoFramesPerExposure = 3;   // 每个曝光档保存的Video帧数
    int m_expTestProfileFramesPerExposure = 20;// 每个曝光档保存的Profile条数
    int m_expTestDiscardRemaining = 0;
    int m_expTestSavedInCurrentMode = 0;
    quint64 m_videoFrameOrdinal = 0;            // MainWindow实际收到的Video帧序号
    quint64 m_expTestLastVideoOrdinal = 0;
    quint64 m_expTestLastProfileOrdinal = 0;
    qint64 m_expTestCaptureDeadlineMs = 0;      // 防止设备无帧时状态机无限等待
    std::shared_ptr<std::atomic<int>> m_expTestPendingSaveJobs =
        std::make_shared<std::atomic<int>>(0);  // 多帧异步保存完成屏障
    
    // 曝光测试 - 新增参数缓存
    double m_expTestTemperature = 25.0;   // 环境温度
    double m_expTestPosX = 0.0;           // 机床位置X
    double m_expTestPosY = 0.0;           // 机床位置Y
    double m_expTestPosZ = 0.0;           // 机床位置Z (DPOS)
    double m_expTestPosC = 0.0;           // 机床位置C (机械MPOS)
    double m_expTestPosB = 0.0;           // 机床位置B (机械MPOS)
    bool m_expTestSaveProfile = true;     // 是否保存轮廓
    bool m_expTestSaveVideo = true;       // 是否保存Video

    // ===== 球面测量距离/截面角研究 =====
    QLineEdit* m_editSphereStudyCloud = nullptr;
    QPushButton* m_btnSphereStudyImport = nullptr;
    QPushButton* m_btnSphereStudyFit = nullptr;
    QDoubleSpinBox* m_spinSphereStudyDiameter = nullptr;
    QDoubleSpinBox* m_spinSphereStudyRansacThreshold = nullptr;
    QDoubleSpinBox* m_spinSphereStudyProfileThreshold = nullptr;
    QDoubleSpinBox* m_spinSphereStudyRadiusTolerance = nullptr;
    // Optical reference center in the scanCONTROL X-Z profile plane.  The
    // default (0,0,0) matches the calibrated sensor coordinate origin, but it
    // is explicit/configurable because study distance is ||C_circle-O_opt||.
    QDoubleSpinBox* m_spinSphereStudyOpticalCenterX = nullptr;
    QDoubleSpinBox* m_spinSphereStudyOpticalCenterZ = nullptr;
    QLabel* m_lblSphereStudyFit = nullptr;

    QDoubleSpinBox* m_spinSphereStudyDistanceStart = nullptr;
    QDoubleSpinBox* m_spinSphereStudyDistanceEnd = nullptr;
    QDoubleSpinBox* m_spinSphereStudyDistanceStep = nullptr;
    QDoubleSpinBox* m_spinSphereStudyGammaStart = nullptr;
    QDoubleSpinBox* m_spinSphereStudyGammaEnd = nullptr;
    QDoubleSpinBox* m_spinSphereStudyGammaStep = nullptr;
    QDoubleSpinBox* m_spinSphereStudyEtaStart = nullptr;
    QDoubleSpinBox* m_spinSphereStudyEtaEnd = nullptr;
    QDoubleSpinBox* m_spinSphereStudyEtaStep = nullptr;
    // V27.6.16Y: C/B are no longer a Cartesian experiment grid.  The planner
    // automatically selects a compact family of reachable, well-separated
    // laser-plane orientations inside the actual rotary software limits.
    QSpinBox* m_spinSphereStudyPlaneDirections = nullptr;
    QSpinBox* m_spinSphereStudyTargetPoseCount = nullptr;
    QDoubleSpinBox* m_spinSphereStudyRotarySearchStep = nullptr;
    QDoubleSpinBox* m_spinSphereStudyMinSimArcLength = nullptr;
    QSpinBox* m_spinSphereStudySimulationIntervalMs = nullptr;
    QDoubleSpinBox* m_spinSphereStudySpeedScale = nullptr;
    QLabel* m_lblSphereStudyModel = nullptr;
    QPushButton* m_btnSphereStudyGenerate = nullptr;
    QTableWidget* m_sphereStudyTable = nullptr;

    QSpinBox* m_spinSphereStudyGroups = nullptr;
    QSpinBox* m_spinSphereStudyProfilesPerGroup = nullptr;
    QSpinBox* m_spinSphereStudyDiscard = nullptr;
    QSpinBox* m_spinSphereStudySettleMs = nullptr;
    QSpinBox* m_spinSphereStudyAutoTimeoutMs = nullptr;
    QSpinBox* m_spinSphereStudyAutoRetries = nullptr;
    QSpinBox* m_spinSphereStudyProbeProfiles = nullptr;
    QSpinBox* m_spinSphereStudyProbeMinCircles = nullptr;
    QDoubleSpinBox* m_spinSphereStudyReacquireStepMm = nullptr;
    QDoubleSpinBox* m_spinSphereStudyReacquireMaxMm = nullptr;
    QCheckBox* m_checkSphereStudyAutoSkipCompleted = nullptr;
    QCheckBox* m_checkSphereStudyAutoContinueOnFailure = nullptr;
    QPushButton* m_btnSphereStudyMove = nullptr;
    QPushButton* m_btnSphereStudyStop = nullptr;
    QPushButton* m_btnSphereStudyCapture = nullptr;
    QPushButton* m_btnSphereStudySimulationStart = nullptr;
    QPushButton* m_btnSphereStudySimulationStop = nullptr;
    QPushButton* m_btnSphereStudySimulationClear = nullptr;
    QPushButton* m_btnSphereStudyAutoStart = nullptr;
    QPushButton* m_btnSphereStudyAutoPause = nullptr;
    QPushButton* m_btnSphereStudyAutoStop = nullptr;
    QPushButton* m_btnSphereStudyAnalyze = nullptr;
    QPushButton* m_btnSphereStudyExport = nullptr;
    QLabel* m_lblSphereStudyCapture = nullptr;
    QLineEdit* m_editSphereStudyDir = nullptr;

    QVector<QVector3D> m_sphereStudyImportedCloud;
    QString m_sphereStudyImportedCloudPath; // provenance for per-profile circle quality
    QString m_sphereStudyFitComparison; // legacy vs two-stage saved with study report
    mls::calibration::SphereCloudFitResult m_sphereStudySphereFit;
    mls::calibration::Physical17Parameters m_sphereStudyPlanningModel;
    QString m_sphereStudyPlanningModelSource;
    bool m_sphereStudyPlanningModelValidated = false;
    bool m_sphereStudyPlanningFlipX = false;
    bool m_sphereStudyPlanningFlipZ = true;
    QString m_sphereStudyPlanningConventionSource;
    bool m_sphereStudyHasPlanningModel = false;
    std::vector<mls::calibration::SphereStudyPose> m_sphereStudyPlan;

    struct SphereStudyCaptureRecord {
        int visitId = -1;
        int planIndex = -1;
        int groupIndex = -1;
        bool complete = false;
        JointPose5D actualPose;
        // Model-only estimate captured before fitting. Final D/gamma come from analysis.
        mls::calibration::SphereStudyObservedGeometry poseGeometryEstimate;
        int probeFrameCount = 0;
        int probeDetectedCircleCount = 0;
        int reacquireAttemptCount = 0;
        double reacquireNormalOffsetMm = 0.0;
        mls::calibration::Vec3d reacquireSensorOffsetMm;
        bool reacquired = false;
        std::vector<mls::calibration::SphereStudyRawProfile> profiles;
        mls::calibration::SphereStudyGroupAnalysis analysis;
        std::vector<mls::calibration::SphereStudyProfileMetrics> metrics;
    };
    std::vector<SphereStudyCaptureRecord> m_sphereStudyRecords;
    bool m_sphereStudyFitActive = false;
    bool m_sphereStudyCaptureActive = false;
    QMetaObject::Connection m_sphereStudyProfileConnection;
    int m_sphereStudyCapturePlanIndex = -1;
    int m_sphereStudyVisitCounter = 0;
    int m_sphereStudyCurrentVisitId = -1;
    int m_sphereStudyCaptureGroup = 0;
    int m_sphereStudyCaptureGroupTarget = 0;
    int m_sphereStudyCaptureProfilesTarget = 0;
    int m_sphereStudyCaptureDiscardRemaining = 0;
    quint64 m_sphereStudyLastProfileOrdinal = 0;
    bool m_sphereStudyStartedPreview = false;
    QString m_sphereStudySessionDir;

    // Automatic sphere-study workflow: valid path rows are executed as
    // move -> settle -> discard warm-up frames -> capture groups -> next pose.
    // Pause is cooperative (after the current move/pose), while Stop can abort
    // the current motion/capture and preserves any already collected raw frames.
    bool m_sphereStudyAutoActive = false;
    bool m_sphereStudyAutoPauseRequested = false;
    bool m_sphereStudyAutoPaused = false;
    bool m_sphereStudyAutoStopRequested = false;
    bool m_sphereStudyAutoOwnsPreview = false;
    bool m_sphereStudyAutoPoseReady = false;
    bool m_sphereStudyProbeActive = false;
    QMetaObject::Connection m_sphereStudyProbeConnection;
    std::vector<mls::calibration::SphereStudyRawProfile> m_sphereStudyProbeProfiles;
    int m_sphereStudyProbeDiscardRemaining = 0;
    int m_sphereStudyProbePlanIndex = -1;
    quint64 m_sphereStudyProbeLastOrdinal = 0;
    quint64 m_sphereStudyProbeWatchToken = 0;
    mls::calibration::JointPose5d m_sphereStudyProbeActualPose;
    int m_sphereStudyReacquireAttempt = 0;
    double m_sphereStudyAppliedNormalOffsetMm = 0.0;
    mls::calibration::Vec3d m_sphereStudyAppliedSensorOffsetMm;
    std::vector<int> m_sphereStudyAutoReacquireAttempts;
    std::vector<double> m_sphereStudyAutoReacquireOffsetsMm;
    std::vector<mls::calibration::Vec3d> m_sphereStudyAutoReacquireSensorOffsetsMm;
    std::vector<int> m_sphereStudyAutoProbeDetectedCircles;
    std::vector<int> m_sphereStudyAutoProbeFrames;
    std::vector<int> m_sphereStudyAutoQueue;
    int m_sphereStudyAutoQueuePos = 0;
    int m_sphereStudyAutoCurrentRetry = 0;
    int m_sphereStudyAutoCompleted = 0;
    int m_sphereStudyAutoFailed = 0;
    quint64 m_sphereStudyAutoSequenceToken = 0;
    quint64 m_sphereStudyCaptureWatchToken = 0;
    std::vector<QString> m_sphereStudyAutoPlanNotes;

    // V27.6.16AC: true offline scan-path digital-twin playback.  No controller
    // or sensor connection is required.  In addition to the transient laser
    // plane/section arc, every optically valid arc is accumulated as a synthetic
    // acquired point cloud in machine coordinates for direct 3-D inspection.
    bool m_sphereStudySimulationActive = false;
    std::vector<int> m_sphereStudySimulationQueue;
    int m_sphereStudySimulationQueuePos = 0;
    quint64 m_sphereStudySimulationToken = 0;
    QVector<QVector3D> m_sphereStudySimulationAcquiredCloud;
    int m_sphereStudySimulationAcquiredPoses = 0;
    int m_sphereStudySimulationRejectedPoses = 0;
    bool m_sphereStudyLastPlanWasOffline = false;

    struct SphereStudyPoseSummary {
        int groups = 0;
        int profiles = 0;
        int circleDetectedProfiles = 0;
        std::size_t evaluationPoints = 0;
        double meanGoodPointRatio = 0.0;
        double meanDropoutRatio = 0.0;
        double meanOutlierRatio = 0.0;
        double meanCircleRmsMm = 0.0;
        double meanAbsoluteP95Mm = 0.0;
        double meanRadiusBiasMm = 0.0;
        double centerRepeatabilityXmm = 0.0;
        double centerRepeatabilityZmm = 0.0;
        double meanActualDistanceMm = 0.0;
        double meanSectionAngleDeg = 0.0;
        double meanCenterBearingAngleDeg = 0.0;
    };
    std::vector<SphereStudyPoseSummary> m_sphereStudySummaries;

    // 视图控件
    QTabWidget* m_mainControlTabs = nullptr; // Parameters / Control / Calibration owner
    QTabWidget* m_viewTabs = nullptr;
    Profile2DWidget* m_profile2DView = nullptr;
    ProfileImageWidget* m_profileImageView = nullptr;
    PointCloudWidget* m_pointCloudView = nullptr;
    VideoWidget* m_videoWidget = nullptr;

    // 最后接收的轮廓（用于 2D 显示）
    LaserProfile m_lastProfile;
    bool m_hasProfile = false;
    quint64 m_lastDisplayedProfileOrdinal = 0;
    
    // 记录最后一次采集的同步模式
    SyncMode m_lastCaptureMode = SyncMode::Software;
    
    // G代码执行器
    std::unique_ptr<GCodeExecutor> m_gcodeExecutor;
    GCodeWidget* m_gcodeWidget = nullptr;
    QFutureWatcher<QString> m_gcodeOfflineGeometryAuditWatcher;
    std::shared_ptr<std::atomic<bool>> m_gcodeGeometryCancel;
    // Session-scoped proof material, not a trusted permission file. Exact NC
    // bytes, model numbers, freshness, and machine connection session bind it.
    struct NcProof {
        bool passed = false;
        QByteArray normalizedNcDigest;
        std::array<double,17> model{};
        std::chrono::steady_clock::time_point finishedAt{};
        // Exact compensation mode used while checking collision trajectories.
        bool compensationEnabled=false;
        // Only a completed, explicitly measured, session-bound on-site
        // acceptance workflow can set this; never NC comments or CAD preview.
        bool independentPartRegistrationCertified=false;
        QByteArray siteAcceptedCadIdentity;
        QByteArray siteAcceptedNcDigest;
        quint64 siteAcceptedConnectionGeneration=0;
        std::chrono::steady_clock::time_point siteAcceptedAt{};
        QString acceptedBy;
        QString measurementMethod;
        double worstRegistrationErrorMm=0.0;
        double measuredFixtureClearanceMm=0.0;
        quint64 connectionGeneration=0;
        int topology=0, frequency=0, edge=0;
        SensorParams sensorParams{};
    };
    NcProof m_geometryNcProof, m_hardwareNcProof;
    // The full-NC worker builds its own immutable FCL scene including the
    // imported STEP. This avoids altering other live machine/sphere studies.
    std::shared_ptr<mls::safety::MachineSafetyModel> m_ncWorkpieceSafety;
    std::shared_ptr<mls::safety::MachineSafetyModel> m_pendingNcWorkpieceSafety;
    QByteArray m_pendingGeometryCadIdentity;
    QByteArray m_geometryCadIdentity;
    QByteArray m_pendingGeometryNcDigest;
    std::array<double,17> m_pendingGeometryNcModel{};
    quint64 m_pendingGeometryConnectionGeneration=0;
    bool m_pendingGeometryCompensationEnabled=false;
    QByteArray m_probeNcDigest;
    std::array<double,17> m_probeNcModel{};
    SensorParams m_probeNcSensorParams{};
    quint64 m_controllerSensorConnectionGeneration=0;
    bool m_gcodeHardwareNcProbeActive = false;
    bool m_gcodeHardwareNcProbeStopping = false;
    SyncMode m_gcodeProbePreviousSyncMode = SyncMode::Software;
    int m_gcodeProbePreviousTopology = 1;
    int m_gcodeProbePreviousHz = 200;
    int m_gcodeProbePreviousEdge = 104;
    bool m_gcodeProbePreviousTriggerExplicit = false;
    bool m_gcodeProbePreviousAlignedEmission = true;
    bool m_gcodeProbePreviousRawCapture = true;
    quint64 m_gcodeHardwareNcProbeSequence = 0;
    QString m_gcodeHardwareNcProbeError;
    int m_gcodeHardwareNcProbeTopology=0;
    int m_gcodeHardwareNcProbeFrequency=0;
    int m_gcodeHardwareNcProbeEdge=0;
    bool m_runtimeCalibrationAutoApplied = false;
    // If calibration authorization is revoked during an active reconstruction/RTCP
    // task, queue one fail-closed stop instead of allowing the stale model to keep
    // running until normal completion.
    bool m_calibrationInvalidationStopQueued = false;
    
    // 日志控件
    LogWidget* m_logWidget = nullptr;
    
    // 算法模块控件
    CalibrationWidget* m_calibrationWidget = nullptr;
    PathPlanningWidget* m_pathPlanningWidget = nullptr;
    PointCloudProcessWidget* m_pointCloudProcessWidget = nullptr;
    
    // 需要在G代码执行时禁用的控件列表
    QList<QWidget*> m_controlsToDisable;
};
