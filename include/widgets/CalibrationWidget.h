#pragma once

/**
 * @file CalibrationWidget.h
 * @brief 系统标定控制面板（V27.6.10）
 *
 * 标定工作流重新按物理含义拆分：
 *   1) 初步手眼标定：单球阶段1，名义机床模型 + 手眼6DoF + 固定球心3D；
 *   2) 运动学标定：单球阶段2，D-optimal + Physical17/rank14 + 独立hold-out验证；
 *   3) 测量不确定度：保留原有独立功能。
 *
 * 旧版 Tsai-Lenz 手眼页和“指令/测量XYZ补偿”运动学页不再出现在UI中。
 */

#include <QWidget>
#include <QTabWidget>
#include <QPushButton>
#include <QLabel>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QTableWidget>
#include <QProgressBar>
#include <QVector>
#include <QPointF>
#include <QVector3D>

#include <array>
#include <memory>
#include <functional>

#include "core/DataTypes.h"

class UncertaintyCalibration;
class ZmcController;
class LaserSensor;
class PointCloudBuilder;
class SyncManager;
class SingleSphere14CalibrationWidget;
class HardwareSyncCalibrationWidget;

namespace mls { namespace calibration { struct PosePlannerConfig; } }

class CalibrationWidget : public QWidget
{
    Q_OBJECT

public:
    explicit CalibrationWidget(QWidget* parent = nullptr);
    ~CalibrationWidget() override;

    void setController(ZmcController* controller);
    void setSensor(LaserSensor* sensor);
    void setPointCloudBuilder(PointCloudBuilder* builder);
    void setSyncManager(SyncManager* manager);
    void setSingleSphereSafetyCheckers(
        std::function<bool(const JointPose5D&, QString&)> poseChecker,
        std::function<bool(const JointPose5D&, const JointPose5D&, QString&)> transitionChecker,
        std::function<bool(mls::calibration::PosePlannerConfig&, QString&, QString&)> planningBoundsResolver);

    // Refresh the live Stage-1 device gates after asynchronous connect/disconnect.
    void refreshSingleSphereDeviceState();

    // Cancel/join Stage-2 background planning before MainWindow tears down the
    // machine-safety model referenced by planning callbacks.
    void shutdownBackgroundWork();

    bool runtimeCompensationChangeBlocked() const;
    bool scanCalibration(std::array<double,17>& parameters) const;
    bool sphereStudyPlanningCalibration(std::array<double,17>& parameters,
                                        QString& source,
                                        bool& independentlyValidated) const;

    // V27.6.16AF1: forward the sensor coordinate convention owned by the
    // integrated single-sphere calibration widget. MainWindow only holds the
    // outer CalibrationWidget pointer, so the wrapper must expose this API.
    bool sphereStudySensorCoordinateConvention(bool& flipX,
                                               bool& flipZ,
                                               QString& source) const;

    // V27.6.16Q: first page of System Calibration is reserved for sensor calibration/debug.
    QWidget* sensorCalibrationPage() const { return m_sensorCalibrationPage; }

    void notifySingleSphereJointMoveFinished(bool ok, const QString& message,
                                             const JointState5D& finalState);
    void notifyHardwareTemporalScanFinished(quint64 requestId, bool success,
                                            const QString& message);
    bool setHardwareTemporalRuntimeEnabled(bool enabled, QString* error = nullptr);
    int hardwareTemporalCalibrationTopologyId() const;
    int hardwareTemporalExpectedR1Edge() const;

signals:
    void logMessage(const QString& message);
    void calibrationCompleted(const QString& type, bool success);

    void singleSphereProfilePreviewStartRequested();
    void singleSphereProfilePreviewStopRequested();
    void singleSphereProfileViewRequested();
    void singleSphereProfileOverlayRequested(const QVector<QPointF>& points,
                                             double centerX, double centerZ, double radius);
    void singleSphereProfileOverlayClearRequested();
    void singleSphereJointMoveRequested(const JointPose5D& target, double speedScale);
    void singleSphereIndependentJointMoveRequested(const JointPose5D& target,
                                                    double xyzSpeedScale,
                                                    double bcSpeedScale);
    void singleSphereJointStopRequested(bool emergency);
    void singleSphereMachineSafetySphereRequested(double xMm, double yMm, double zMm,
                                                  double radiusMm, bool enabled);
    void singleSphereStage2SimulationFrameRequested(const JointPose5D& pose,
                                                    const QVector<QVector3D>& laserFieldMachineMm,
                                                    const QVector<QVector3D>& sectionCircleMachineMm,
                                                    const QVector<QVector3D>& visibleArcMachineMm,
                                                    const QVector<QVector3D>& accumulatedArcMachineMm,
                                                    bool feasible, const QString& statusText);
    void singleSphereStage2SimulationClearRequested(bool clearAccumulatedCloud);
    void singleSphereStage2SimulationViewRequested();
    void singleSphereMotionControlViewRequested();
    void singleSphereRuntimeCalibrationUsableChanged(bool usable);

    // V27.6.47 staged hardware-temporal workflow. MainWindow remains the sole
    // owner of machine motion, MachineSafety checks and formal hardware-sync
    // acquisition. The calibration widget only plans/reduces data.
    void hardwareTemporalPlannedScanRequested(quint64 requestId,
                                              const JointPose5D& start,
                                              const JointPose5D& end,
                                              double feedMmPerSec);
    void hardwareTemporalAbortRequested();
    void hardwareTemporalReferenceSphereRequested(double xMm, double yMm, double zMm,
                                                  double radiusMm, bool enabled);
    void hardwareTemporalReferenceCloudViewRequested(const QVector<QVector3D>& pointsMachineMm);
    // V27.6.53: emitted when the active temporal calibration or runtime ON/OFF
    // state changes, so MainWindow can refresh Parameters and the 3-D badge.
    void hardwareTemporalRuntimeStateChanged();
    void hardwareTemporalRuntimeCorrectionChanged(bool enabled, double delayUs);

private slots:
    void onStartUncertaintyCalib();
    void onStopUncertaintyCalib();
    void onAddUncertaintyMeasurement();
    void onClearUncertaintyData();
    void onExportUncertaintyReport();
    void onTestUncertainty();

    void onProgressUpdated(int current, int total, const QString& message);
    void onCalibrationCompleted(bool success);

private:
    void setupUI();
    void setupUncertaintyTab(QWidget* parent);
    void connectSignals();
    void connectSingleSphereWorkflow();
    void updateUIState();
    void appendLog(const QString& message);

private:
    ZmcController* m_controller = nullptr;
    LaserSensor* m_sensor = nullptr;
    PointCloudBuilder* m_cloudBuilder = nullptr;

    std::unique_ptr<UncertaintyCalibration> m_uncertaintyCalib;
    SingleSphere14CalibrationWidget* m_singleSphere14Widget = nullptr;
    HardwareSyncCalibrationWidget* m_hardwareSyncCalibrationWidget = nullptr;

    QTabWidget* m_tabWidget = nullptr;
    QWidget* m_sensorCalibrationPage = nullptr;
    int m_sensorCalibrationTabIndex = -1;
    int m_handEyeTabIndex = -1;
    int m_kinematicTabIndex = -1;
    int m_hardwareSyncTabIndex = -1;
    int m_uncertaintyTabIndex = -1;

    QWidget* m_algorithmStatusPanel = nullptr;
    QProgressBar* m_progressBar = nullptr;
    QLabel* m_statusLabel = nullptr;

    // 测量不确定度标定 UI
    QDoubleSpinBox* m_spinResolution = nullptr;
    QDoubleSpinBox* m_spinAccuracy = nullptr;
    QDoubleSpinBox* m_spinTempVar = nullptr;
    QSpinBox* m_spinRepeatCount = nullptr;
    QTableWidget* m_uncertaintyTable = nullptr;
    QPushButton* m_btnAddMeasurement = nullptr;
    QPushButton* m_btnStartUncertainty = nullptr;
    QPushButton* m_btnStopUncertainty = nullptr;
    QPushButton* m_btnClearUncertainty = nullptr;
    QPushButton* m_btnExportUncertainty = nullptr;
    QPushButton* m_btnTestUncertainty = nullptr;
    QLabel* m_lblUncertaintyResult = nullptr;

    bool m_isCalibrating = false;
    QString m_currentCalibType;
};
