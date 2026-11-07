#pragma once

#include <QWidget>
#include <QVector>
#include <QVector3D>
#include <vector>

#include "calibration/HardwareTemporalCalibration.h"
#include "core/DataTypes.h"

class SyncManager;
class ZmcController;
class QTableWidget;
class QLabel;
class QPushButton;
class QDoubleSpinBox;
class QSpinBox;
class QLineEdit;
class QCheckBox;
class QComboBox;
struct HardwareTemporalCorrectionSettings;

/**
 * V27.6.53 auto-persisted/runtime-controllable drift-aware/free-effective-radius resumable hardware-temporal calibration workflow for the commissioned
 * ECI3808 V32.11 SINGLE-OP14 topology.
 *
 * Workflow:
 *   1. import Machine-frame sphere cloud -> RANSAC reference center;
 *   2. capture current XYZCB as scan-center and configure arbitrary XYZ line paths;
 *   3. generate paired forward/reverse calibration paths;
 *   4. automatically pre-position, arm the existing hardware-sync acquisition,
 *      execute the planned line, stop/drain, and ingest every path;
 *   5. fit dynamic scans with a free effective sphere and solve one global delay
 *      together with slow center drift, c(t)=c0+g*t-tau*v;
 *   6. generate independent hold-out paths and compare raw vs offline corrected
 *      reconstruction before any runtime correction is enabled;
 *   7. only a solved + validated result may be applied to runtime compensation.
 */
class HardwareSyncCalibrationWidget final : public QWidget
{
    Q_OBJECT
public:
    explicit HardwareSyncCalibrationWidget(QWidget* parent = nullptr);

    void setSyncManager(SyncManager* manager);
    void setController(ZmcController* controller);
    void notifyPlannedScanFinished(quint64 requestId, bool success, const QString& message);

    // V27.6.53: Parameters-page runtime switch.  The active calibration remains
    // loaded when disabled; only q(t+tau) application is toggled and persisted.
    bool setRuntimeCorrectionEnabled(bool enabled, QString* error = nullptr);
    // Calibration route selection independent of the ordinary scan topology UI.
    // H1=1, H3=3; H2=2 is retired. Only permitted between sessions.
    int selectedTemporalTopologyId() const { return m_selectedTopologyId; }
    int expectedCmmR1Edge() const { return m_sessionLatchEdge; }

signals:
    void logMessage(const QString& message);
    void runtimeCorrectionChanged(bool enabled, double delayUs);
    // Emitted when active_hardware_temporal.json is created/loaded/replaced, even
    // if runtime compensation itself remains OFF.
    void activeCalibrationChanged();

    // MainWindow owns machine safety, motion ownership and the formal hardware
    // acquisition start/stop sequence.  The widget only owns planning and data reduction.
    void requestPlannedHardwareScan(quint64 requestId,
                                    const JointPose5D& start,
                                    const JointPose5D& end,
                                    double feedMmPerSec);
    void requestAbortPlannedHardwareScan();
    void referenceSphereChanged(double xMm, double yMm, double zMm,
                                double radiusMm, bool enabled);
    void referenceCloudViewRequested(const QVector<QVector3D>& pointsMachineMm);

private slots:
    void importReferenceCloud();
    void fitReferenceSphere();
    void capturePlanCenter();
    void addDefaultPlanRows();
    void addPlanRow();
    void removePlanRow();
    void generatePaths();
    void startCalibrationAcquisition();
    void startValidationAcquisition();
    void abortWorkflow();
    void solveCalibration();
    void evaluateValidation();
    void applyCalibration();
    void disableCorrection();
    void saveCalibrationAs();
    void loadCalibrationFrom();
    void clearCalibrationData();
    void restoreCaptureSession();
    void newCaptureSession();
    void changeSelectedTopology(int index);

private:
    enum class ScanRole { Calibration, Validation };
    enum class WorkflowMode { Idle, Calibration, Validation };

    struct PlannedScan {
        quint64 id = 0;
        quint64 pairId = 0;
        ScanRole role = ScanRole::Calibration;
        QString name;
        int directionSign = +1;
        JointPose5D start;
        JointPose5D end;
        QVector3D unitDirection;
        double requestedSpeedMmPerSec = 0.0;
        double distanceMm = 0.0;
        bool completed = false;
        bool accepted = false;
        QString status;
        QString archivePath;
        QString metadataPath;
        int attemptCount = 0;
        int failureCount = 0;
        int consecutiveFailures = 0;
        QString lastError;
        bool restoredFromDisk = false;
    };

    struct ValidationCapture {
        mls::calibration::TemporalValidationObservation observation;
        double referenceRawRmsMm = 0.0;
        double referenceCorrectedRmsMm = 0.0;
        std::size_t inliers = 0;
    };

    void setupUi();
    void refreshReferenceUi();
    void refreshPathTable();
    void refreshCaptureTable();
    void refreshSpeedSummaryTable();
    void refreshResult();
    void updateWorkflowButtons();
    struct CaptureProcessResult {
        bool success = false;
        QString error;
        bool validation = false;
        mls::calibration::TemporalTrajectoryObservation calibrationObservation;
        ValidationCapture validationCapture;
        QString archivePath;
        QString metadataPath;
        std::size_t totalProfiles = 0;
        std::size_t totalValidPoints = 0;
        std::size_t analysisPoints = 0;
        std::size_t arcTotalProfiles = 0;
        std::size_t arcQualifiedProfiles = 0;
        std::size_t arcQualifiedPoints = 0;
        QString sphereFitComparisonText;
    };

    void requestNextWorkflowScan();
    void handlePlanFailure(PlannedScan& plan, const QString& reason, bool allowAutoRetry);
    bool hasAcceptedPlans(ScanRole role) const;
    int acceptedPlanCount(ScanRole role) const;
    QString sessionCheckpointPath() const;
    bool saveSessionCheckpoint(QString* error = nullptr) const;
    bool loadSessionCheckpoint(const QString& directory, QString* error = nullptr);
    bool recoverLegacyV48Session(const QString& directory, QString* error = nullptr);
    bool allPlansComplete(ScanRole role) const;
    std::size_t planCount(ScanRole role) const;
    void startCaptureProcessing(const PlannedScan& plan, const QString& scanMessage);
    void finishCaptureProcessing(quint64 planId, quint64 processingGeneration,
                                 CaptureProcessResult result, const QString& scanMessage);
    QString ensureCaptureSessionDirectory(QString* error = nullptr);
    QString captureArchiveBaseName(const PlannedScan& plan) const;
    static double radialRms(const std::vector<mls::calibration::Vec3d>& points,
                            const mls::calibration::Vec3d& center,
                            double radiusMm);
    QString activePath() const;
    HardwareTemporalCorrectionSettings correctionSettings(bool enabled) const;
    bool saveJson(const QString& path, bool runtimeEnabled, QString* error = nullptr) const;
    bool loadJson(const QString& path, bool applyRuntime, QString* error = nullptr);
    bool applyResult(bool persist, QString* error = nullptr);
    void loadActiveOnBind();
    mls::calibration::TemporalTrajectoryCriteria trajectoryCriteriaFromUi() const;
    mls::calibration::TemporalValidationCriteria validationCriteriaFromUi() const;

private:
    int m_selectedTopologyId = 1;
    int m_sessionLatchEdge = 104; // H1 fixed 104, H3 first capture locks 114/115
    QComboBox* m_topologyCombo = nullptr;
    QLabel* m_topologyDescription = nullptr;
    SyncManager* m_sync = nullptr;
    ZmcController* m_controller = nullptr;

    QVector<QVector3D> m_referenceCloud;
    QString m_referenceCloudPath;
    QString m_referenceTwoStageDiagnostic; // persisted old-vs-two-stage comparison
    bool m_referenceFitValid = false;
    mls::calibration::KnownSphereCenterFit m_referenceFit;
    double m_referenceFreeRadiusMm = 0.0;
    std::size_t m_referenceInliers = 0;

    bool m_planCenterValid = false;
    JointPose5D m_planCenter;
    std::vector<PlannedScan> m_plannedScans;
    std::vector<mls::calibration::TemporalTrajectoryObservation> m_calibrationObservations;
    std::vector<ValidationCapture> m_validationCaptures;
    mls::calibration::TemporalTrajectoryCalibrationResult m_result;
    mls::calibration::TemporalValidationResult m_validationResult;
    bool m_haveResult = false;
    bool m_validationPassed = false;
    QString m_calibrationId;

    WorkflowMode m_workflowMode = WorkflowMode::Idle;
    int m_workflowPlanIndex = -1;
    quint64 m_pendingRequestId = 0;
    bool m_captureProcessing = false;
    quint64 m_captureProcessingGeneration = 0;
    QString m_captureSessionId;
    QString m_captureSessionDirectory;
    quint64 m_nextPlanId = 1;
    quint64 m_nextPairId = 1;
    quint64 m_nextObservationId = 1;

    QLineEdit* m_referencePath = nullptr;
    QLabel* m_referenceResult = nullptr;
    QLabel* m_planCenterLabel = nullptr;
    QLabel* m_pathGenerationFeedback = nullptr;
    QTableWidget* m_planConfigTable = nullptr;
    QTableWidget* m_pathTable = nullptr;
    QTableWidget* m_captureTable = nullptr;
    QTableWidget* m_speedSummaryTable = nullptr;
    QLabel* m_status = nullptr;
    QLabel* m_resultLabel = nullptr;
    QLabel* m_validationLabel = nullptr;
    QLabel* m_runtimeLabel = nullptr;

    QDoubleSpinBox* m_sphereDiameter = nullptr;

    // V27.6.50 one-click multi-speed experiment template.  The table remains
    // editable, but the common 0.5..5.0 mm/s / 0.5 step / 3-pair experiment
    // no longer requires manual row creation.
    QDoubleSpinBox* m_templateMinSpeed = nullptr;
    QDoubleSpinBox* m_templateMaxSpeed = nullptr;
    QDoubleSpinBox* m_templateSpeedStep = nullptr;
    QDoubleSpinBox* m_templateDistance = nullptr;
    QSpinBox* m_templateCalibrationRepeats = nullptr;
    QDoubleSpinBox* m_templateValidationSpeed = nullptr;
    QSpinBox* m_templateValidationRepeats = nullptr;

    // V27.6.50 memory-safe archive/analysis and resumable-session controls.
    QLineEdit* m_captureRootDirectory = nullptr;
    QCheckBox* m_archiveEachScan = nullptr;
    QSpinBox* m_maxAnalysisPoints = nullptr;
    QSpinBox* m_autoRetryCount = nullptr;
    QPushButton* m_resumeSessionButton = nullptr;
    QPushButton* m_newSessionButton = nullptr;

    QDoubleSpinBox* m_ransacThreshold = nullptr;
    QSpinBox* m_minInliers = nullptr;
    QSpinBox* m_minPairs = nullptr;
    QSpinBox* m_minSpeedLevels = nullptr;
    QDoubleSpinBox* m_minR2 = nullptr;
    QDoubleSpinBox* m_maxPairResidual = nullptr;
    QDoubleSpinBox* m_maxPairRadiusDiff = nullptr;
    QDoubleSpinBox* m_maxStdUs = nullptr;
    QSpinBox* m_minValidationPairs = nullptr;
    QDoubleSpinBox* m_minValidationImprovement = nullptr;
    QDoubleSpinBox* m_maxValidationResidualUs = nullptr;

    QPushButton* m_generateButton = nullptr;
    QPushButton* m_startCalibrationButton = nullptr;
    QPushButton* m_startValidationButton = nullptr;
    QPushButton* m_abortButton = nullptr;
    QPushButton* m_solveButton = nullptr;
    QPushButton* m_validateButton = nullptr;
    QPushButton* m_applyButton = nullptr;
};
