#pragma once

#include <QWidget>
#include <QVector>
#include <QPointer>
#include <QPointF>
#include <QVector3D>
#include <QString>
#include <QStringList>
#include <QJsonObject>
#include <QtGlobal>
#include <vector>
#include <cstdint>
#include <memory>
#include <atomic>
#include <functional>
#include <string>

#include "core/DataTypes.h"
#include "calibration/SingleSphere17Calibration.h"

class QDoubleSpinBox;
class QSpinBox;
class QComboBox;
class QCheckBox;
class QPushButton;
class QLabel;
class QTableWidget;
class QProgressBar;
class QPlainTextEdit;
class QTimer;
template <typename T> class QFutureWatcher;
class ZmcController;
class LaserSensor;
class PointCloudBuilder;

/**
 * @brief Integrated V20.2 single-sphere physical-17 / rank-14 calibration panel.
 *
 * The historical class/file name is retained to avoid breaking MainWindow wiring.
 *
 * The widget supports checked automatic Stage-2 training and Stage-3 hold-out
 * acquisition. Planned pose changes are issued through the non-interpolated
 * independent-axis motion facade; static profile capture starts only after all
 * five axes report idle and the live sphere-quality gate passes.
 */
class SingleSphere14CalibrationWidget : public QWidget
{
    Q_OBJECT
public:
    explicit SingleSphere14CalibrationWidget(QWidget* parent = nullptr);
    ~SingleSphere14CalibrationWidget() override;

    void setController(ZmcController* controller);
    void setSensor(LaserSensor* sensor);
    void setPointCloudBuilder(PointCloudBuilder* builder);

    using PoseSafetyChecker = std::function<bool(const JointPose5D&, QString&)>;
    using TransitionSafetyChecker = std::function<bool(const JointPose5D&, const JointPose5D&, QString&)>;
    // Supplies finite Machine5X planning bounds for both online and offline
    // Stage-2 path generation.  Offline mode must not silently fall back to the
    // PosePlannerConfig +/-1e12 placeholders.
    using PlanningBoundsResolver = std::function<bool(mls::calibration::PosePlannerConfig&, QString&, QString&)>;
    void setStage2SafetyCheckers(PoseSafetyChecker poseChecker,
                                 TransitionSafetyChecker transitionChecker,
                                 PlanningBoundsResolver planningBoundsResolver = PlanningBoundsResolver());

    // Re-read the live connection/stream flags and refresh all Stage-1 gates.
    // MainWindow calls this after asynchronous controller/sensor connect/disconnect
    // operations, because the core device wrappers do not emit connectionChanged signals.
    void refreshDeviceState();

    // Called by MainWindow before its machine-safety/controller members are
    // destroyed. Stage-2 planning may execute safety callbacks on a worker
    // thread, so shutdown must cancel and join that worker first.
    void shutdownBackgroundWork();

    // GUI-thread query: freeze runtime compensation for complete multi-step
    // acquisition workflows, including idle/settling gaps between moves.
    bool runtimeCompensationChangeBlocked() const {
        return m_collecting || m_capturePendingStart || m_moveInProgress ||
               m_autoPlanMove || m_autoStage2Acquisition || m_autoValidationAcquisition ||
               m_stage1SolveRunning || m_stage2PlanRunning || m_stage2SolveRunning;
    }

    bool scanCalibration(std::array<double,17>& parameters) const;

    // Planning-only access for the sensor distance/section-angle study. Unlike
    // scanCalibration(), this may return the best approximate previous/current
    // Physical17 estimate even before a new hold-out validation has completed.
    // It must never be used to authorize formal reconstruction/RTCP.
    bool sphereStudyPlanningCalibration(std::array<double,17>& parameters,
                                        QString& source,
                                        bool& independentlyValidated) const;

    // Coordinate convention paired with the Physical17/stage-1 model used by
    // sphere-study planning.  Offline scan simulation must know FlipZ because
    // the physical scanCONTROL trapezoid is defined by optical near/far depth,
    // while the processed Sensor-Z ordering is reversed when FlipZ is active.
    bool sphereStudySensorCoordinateConvention(bool& flipX,
                                               bool& flipZ,
                                               QString& source) const;

    // V27.6.9: expose the two physical calibration stages as separate pages.
    // The same controller object owns all state, so Stage-1 results flow directly
    // into Stage-2 without files, duplicated widgets, or copied calibration state.
    QWidget* stage1Page() const { return m_stage1Page; }
    QWidget* stage2Page() const { return m_stage2Page; }

    void notifyJointMoveFinished(bool ok, const QString& message, const JointState5D& finalState);

signals:
    void logMessage(const QString& message);
    void requestProfilePreviewStart();
    void requestProfilePreviewStop();
    void requestProfileView();
    void requestProfileOverlay(const QVector<QPointF>& points,
                               double centerX, double centerZ, double radius);
    void requestProfileOverlayClear();
    void requestJointMove(const JointPose5D& target, double speedScale);
    // Stage-2/hold-out pose transit: command all five axes independently at
    // their own configured SPEED/ACCEL/DECEL * speedScale.  This deliberately
    // avoids five-axis interpolation, so a short-travel axis is not slowed to
    // the distance ratio of a long-travel axis.
    void requestIndependentJointMove(const JointPose5D& target,
                                     double xyzSpeedScale,
                                     double bcSpeedScale);
    void requestJointStop(bool emergency);
    // Keep the MachineSafety digital twin synchronized with a sphere cloud that
    // was imported directly on the kinematic-calibration page.
    void requestMachineSafetySphere(double xMm, double yMm, double zMm,
                                    double radiusMm, bool enabled);
    // GUI-only playback of generated kinematic-calibration scan poses. No controller command is sent.
    void requestStage2SimulationFrame(const JointPose5D& pose,
                                      const QVector<QVector3D>& laserFieldMachineMm,
                                      const QVector<QVector3D>& sectionCircleMachineMm,
                                      const QVector<QVector3D>& visibleArcMachineMm,
                                      const QVector<QVector3D>& accumulatedArcMachineMm,
                                      bool feasible, const QString& statusText);
    void requestStage2SimulationClear(bool clearAccumulatedCloud);
    // Request the 3-D point-cloud/digital-twin page once when Stage-2 playback starts.
    // Per-frame updates must never force the active view, so the operator can freely
    // switch between 3-D and 2-D while playback continues.
    void requestStage2SimulationView();
    // Navigate to the single authoritative machine-control page that owns
    // XYZ/five-axis homing, B/C absolute resync and manual servo enable.
    void requestMotionControlView();
    void calibrationReadyChanged(bool ready);
    // Runtime-safe signal: true only when the accepted Physical17 model is independently
    // validated and the saved controller/sensor coordinate convention is verified live.
    void runtimeCalibrationUsableChanged(bool usable);
    void stage1BootstrapCompleted(bool success);

private slots:
    void onSensorProfile(const LaserProfile& profile);
    void onStartPreview();
    void onStopPreview();
    void onReadCurrentPose();
    void onFitCurrentProfile();
    void onTeachReference();
    void onSolveBootstrap();
    void onGenerateStage1Candidates();
    void onMoveToSelectedStage1Candidate();
    void onSkipSelectedStage1Candidate();
    void onSaveStage1Result();
    void onLoadStage1Result();
    void onGeneratePlan();
    void onCancelStage2PlanGeneration();
    void onStartStage2Simulation();
    void onStopStage2Simulation();
    void onClearStage2Simulation();
    void onImportStage2SphereCloud();
    void onFitStage2SphereCloud();
    void onStartAutomaticStage2Acquisition();
    void onStopAutomaticStage2Acquisition();
    void onStartAutomaticValidationAcquisition();
    void onStopAutomaticValidationAcquisition();
    void onGenerateValidationPlan();
    void onMoveToSelectedPose();
    void onSkipSelectedPose();
    void onRecommendRecenter();
    void onCaptureCurrentPose();
    void onCaptureValidationPose();
    void onImportValidationCsv();
    void onExportValidationCsv();
    void onRunValidation();
    void onClearValidation();
    void onDeleteLastCapture();
    void onClearSession();
    void onSolve();
    void onCancelStage2Solve();
    void onSaveResult();
    void onLoadResult();
    void onBindLoadedResultToCurrentSession();
    void onExportSessionCsv();
    void onImportSessionCsv();
    void onPlanSelectionChanged();

private:
    enum class PendingMoveKind {
        None,
        TransitBC, // legacy segmented path; retained for compatibility only
        PlanDirect,
        RecenterXYZ,
        Stage1Nudge,
        Stage1CandidateDirect
    };

    // Distinguish global machine readiness from a pose-specific MachineSafety
    // rejection. Automatic Stage-2 stops on the former and preserves rows.
    enum class Stage2MovePreflightFailure {
        None,
        GlobalReadiness,
        TargetSafety,
        TransitionSafety,
        UserCancelled
    };

    struct CaptureRecord {
        int planIndex = -1;
        bool stage1Bootstrap = false;
        bool referenceCapture = false; // legacy/session display only
        JointState5D before;
        JointState5D after;
        JointPose5D usedPose;
        QVector<LaserProfile> rawProfiles;
        std::vector<mls::calibration::SphereProfileObservation> observations;
        mls::calibration::SphereCenterFit representativeFit;
        QString timeText;
    };

    struct ValidationSummary {
        bool computed = false;
        bool passed = false;
        int poseCount = 0;
        std::size_t profileCount = 0;
        std::size_t pointCount = 0;
        double surfaceMeanMm = 0.0;
        double surfaceMaeMm = 0.0;
        double surfaceRmsMm = 0.0;
        double surfaceMaxMm = 0.0;
        double centerXzRmsMm = 0.0;
        double center3dRmsMm = 0.0;
        double centerMaxMm = 0.0;
        mls::calibration::Vec3d centerBiasSensorMm;
        bool bCoverageOk = false;
        bool cCoverageOk = false;
        QString message;
    };

    void setupUi();
    void connectUi();
    void updateDeviceState();
    void updateCurrentPoseLabel(const JointState5D* state = nullptr);
    void updatePlanTable();
    void updateSessionSummary();
    void updateResultTable(const mls::calibration::SingleSphere17Result* result = nullptr);
    void updateCalibrationReadiness();
    void updateWorkflowGuide();
    void updateStage1Status();
    void updateStage1ResultDisplay();
    void updateStage1CandidateTable();
    void setStage1CandidateStatus(int index, const QString& status);
    int selectedStage1CandidateIndex() const;
    bool selectNextStage1Candidate(int afterIndex);
    bool selectNextPendingPlanRow(int afterIndex);
    bool writeStage1ResultJson(const QString& fileName, QString& error) const;
    bool loadStage1ResultJson(const QString& fileName,
                              mls::calibration::NominalHandEyeSphereResult& result,
                              std::vector<JointPose5D>& capturedPoses,
                              qint64& solveElapsedMs,
                              bool& flipX, bool& flipZ, std::uint8_t& feedbackMask,
                              QString& error) const;
    std::vector<mls::calibration::SphereProfileObservation> stage2TrainingObservations() const;
    std::vector<mls::calibration::SphereProfileObservation> reducedStage2Observations(
        int maxProfilesPerPose, std::size_t maxPointsPerProfile) const;
    int stage2TrainingPoseCount() const;
    void updateStage2SolveProgressUi(const mls::calibration::Physical17SolveProgress& progress);
    void updateStage2PlanProgressUi(const mls::calibration::Physical17PlanningProgress& progress);
    void appendStage2SolveDetail(const QString& text);
    void onStage1Nudge(int axis, double delta);
    QString stage1FailureAdvice(const mls::calibration::NominalHandEyeSphereResult& result) const;
    void appendLog(const QString& text);

    std::vector<mls::calibration::SensorProfilePoint> filteredPoints(
        const LaserProfile& profile, QVector<QPointF>* overlay = nullptr) const;
    mls::calibration::SphereCenterFit fitProfile(const LaserProfile& profile,
                                                  QVector<QPointF>* overlay = nullptr) const;
    mls::calibration::SphereArcRansacOptions ransacOptionsFromUi() const;
    bool readStableState(JointState5D& state, QString& error) const;
    bool checkStage2LiveMotionReadiness(QString& error, JointState5D* state = nullptr) const;
    void updateStage2MotionReadinessUi();
    bool readPoseSourceMask(std::uint8_t& feedbackMask, QString& error) const;
    JointPose5D calibrationPoseFromState(const JointState5D& state, std::uint8_t feedbackMask) const;
    JointPose5D averagedCalibrationPose(const JointState5D& a, const JointState5D& b,
                                        std::uint8_t feedbackMask) const;
    bool checkCaptureDrift(const JointState5D& before, const JointState5D& after,
                           QString& error) const;
    bool validateSessionConventions(QString& error) const;
    bool selectedPoseBCReached(const JointPose5D& current, int planIndex,
                               double toleranceDeg = 0.15) const;
    void beginFreshProfileCollection();
    void startCapture(bool validationOnly);
    void finishFreshProfileCollection();
    void rebuildCalibratorFromRecords();
    void updateValidationSummary();
    bool validationPoseSeparated(const JointPose5D& pose, QString& error) const;
    bool validationPoseOutsideRejectedTrainingHazards(const JointPose5D& pose,
                                                      double minBcDistanceDeg,
                                                      QString* reason = nullptr) const;
    void invalidateValidation(const QString& reason, bool clearRecords);
    QString physical17Fingerprint(const mls::calibration::SingleSphere17Result& result) const;
    QString trainingPoseFingerprint() const;
    bool writeValidationObservationCsv(const QString& fileName, QString& error) const;
    bool loadValidationObservationCsv(const QString& fileName,
                                      std::vector<CaptureRecord>& records,
                                      QString& error) const;
    bool persistValidationObservationCsv(QStringList& savedPaths, QString& error) const;
    bool writeValidationJson(const QString& fileName, QString& error) const;
    void markPlanStatus(int index, const QString& status, const QString& quality = QString());
    void continueAutoPlanMove();
    void stopAutoPlanMove(const QString& reason = QString());
    int selectedPlanIndex() const;
    bool startDirectMoveToPlanIndex(int index, bool askConfirmation,
                                    QString* failureReason = nullptr,
                                    Stage2MovePreflightFailure* failureKind = nullptr);
    void continueAutomaticStage2Acquisition();
    void stopAutomaticStage2Acquisition(const QString& reason = QString());
    void continueAutomaticValidationAcquisition();
    void stopAutomaticValidationAcquisition(const QString& reason = QString());
    void preserveAcceptedModelAsFallbackForRecalibration();
    bool showStage2SimulationPose(int planIndex, QString& error);
    void advanceStage2Simulation();
    void finishStage2Simulation(const QString& reason);
    bool preflightAndFilterGeneratedPlan(const JointPose5D& startPose,
                                         bool gateInitialTransition,
                                         int desiredPoseCount,
                                         QString& summary);
    mls::calibration::PosePlannerConfig plannerConfigFromUi(QString& warning) const;
    mls::calibration::Physical17SolverOptions solverOptionsFromUi() const;
    bool parsePhysical17JsonObject(const QJsonObject& root,
                                   mls::calibration::SingleSphere17Result& result,
                                   QString& error) const;
    bool loadPhysical17Json(const QString& fileName, mls::calibration::SingleSphere17Result& result,
                            QString& error) const;
    bool loadValidationJson(const QString& fileName,
                            const mls::calibration::SingleSphere17Result& result,
                            ValidationSummary& summary, QString& error,
                            bool requireStrongModelMatch = false) const;
    bool findMatchingValidationFile(const QString& stage2FileName,
                                    const mls::calibration::SingleSphere17Result& result,
                                    ValidationSummary& summary,
                                    QString& validationPath,
                                    QString& error) const;
    QString persistentCalibrationPath() const;
    QString portablePersistentCalibrationPath() const;
    QStringList persistentCalibrationPaths() const;
    QString persistentCalibrationRevocationPath() const;
    QStringList persistentCalibrationRevocationPaths() const;
    bool writePersistentCalibrationRevocation(QString& error) const;
    bool clearPersistentCalibrationRevocation(QString& error) const;
    bool savePersistentCalibration(QString& error);
    bool loadPersistentCalibrationFile(const QString& path, QString& error);
    bool loadLegacyCalibrationFromApplicationDir(QString& status, QString& error);
    bool loadPersistentCalibration(QString& error);
    bool tryBindPersistentCalibration(QString& error);
    bool tryBindCurrentSolvedResultForCalibrationMotion(QString& error);
    bool removePersistentCalibration(QString& error);
    void notifyRuntimeCalibrationUsability();

private:
    struct Stage2PlanGenerationResult {
        std::vector<mls::calibration::CalibrationPoseInstruction> plan;
        std::string error;
        int opticalChecked = 0;
        int opticalRejected = 0;
        int targetChecked = 0;
        int targetRejected = 0;
        int transitionChecked = 0;
        int transitionRejected = 0;
        bool cancelled = false;
    };

    QWidget* m_stage1Page = nullptr;
    QWidget* m_stage2Page = nullptr;

    QPointer<ZmcController> m_controller;
    QPointer<LaserSensor> m_sensor;
    QPointer<PointCloudBuilder> m_cloudBuilder;
    PoseSafetyChecker m_stage2PoseSafetyChecker;
    TransitionSafetyChecker m_stage2TransitionSafetyChecker;
    PlanningBoundsResolver m_stage2PlanningBoundsResolver;

    QLabel* m_lblDevice = nullptr;
    QLabel* m_lblCurrentPose = nullptr;
    QLabel* m_lblFit = nullptr;
    QLabel* m_lblSession = nullptr;
    QLabel* m_lblObservability = nullptr;
    QLabel* m_lblBootstrap = nullptr;
    QLabel* m_lblSolveState = nullptr;
    QLabel* m_lblAcquisitionMode = nullptr;
    QLabel* m_lblWorkflow = nullptr;
    QLabel* m_lblConvention = nullptr;

    QDoubleSpinBox* m_spinSphereDiameter = nullptr;
    QDoubleSpinBox* m_spinRoiXMin = nullptr;
    QDoubleSpinBox* m_spinRoiXMax = nullptr;
    QDoubleSpinBox* m_spinRoiZMin = nullptr;
    QDoubleSpinBox* m_spinRoiZMax = nullptr;
    QDoubleSpinBox* m_spinMaxCircleRms = nullptr;
    QSpinBox* m_spinRansacIterations = nullptr;
    QDoubleSpinBox* m_spinRansacThreshold = nullptr;
    QDoubleSpinBox* m_spinRansacMinRatio = nullptr;
    QDoubleSpinBox* m_spinRansacMinArc = nullptr;
    QSpinBox* m_spinCaptureFrames = nullptr;
    QSpinBox* m_spinSettleMs = nullptr;
    QDoubleSpinBox* m_spinLinearDrift = nullptr;
    QDoubleSpinBox* m_spinRotaryDrift = nullptr;

    QDoubleSpinBox* m_spinStage1LinearStep = nullptr;
    QDoubleSpinBox* m_spinStage1RotaryStep = nullptr;
    QDoubleSpinBox* m_spinStage1Speed = nullptr;

    QSpinBox* m_spinPoseCount = nullptr;
    QDoubleSpinBox* m_spinQualityDMin = nullptr;
    QDoubleSpinBox* m_spinQualityDMax = nullptr;
    QDoubleSpinBox* m_spinQualityDStep = nullptr;
    QDoubleSpinBox* m_spinQualityGammaMin = nullptr;
    QDoubleSpinBox* m_spinQualityGammaMax = nullptr;
    QDoubleSpinBox* m_spinQualityGammaStep = nullptr;
    QDoubleSpinBox* m_spinQualityEtaMin = nullptr;
    QDoubleSpinBox* m_spinQualityEtaMax = nullptr;
    QDoubleSpinBox* m_spinQualityEtaStep = nullptr;
    QDoubleSpinBox* m_spinStage2MinInlierRatio = nullptr;
    QDoubleSpinBox* m_spinStage2MaxRms = nullptr;
    QDoubleSpinBox* m_spinStage2MaxP95 = nullptr;
    QDoubleSpinBox* m_spinStage2EdgeTrimFraction = nullptr;
    QDoubleSpinBox* m_spinStage2MinValidFrameRatio = nullptr;
    QDoubleSpinBox* m_spinBMin = nullptr;
    QDoubleSpinBox* m_spinBMax = nullptr;
    QDoubleSpinBox* m_spinBStep = nullptr;
    QDoubleSpinBox* m_spinCMin = nullptr;
    QDoubleSpinBox* m_spinCMax = nullptr;
    QDoubleSpinBox* m_spinCStep = nullptr;
    QDoubleSpinBox* m_spinStage2XyzSpeedScale = nullptr;
    QDoubleSpinBox* m_spinStage2BcSpeedScale = nullptr;
    QDoubleSpinBox* m_spinRecenterMaxStep = nullptr;
    QSpinBox* m_spinStage2SimulationIntervalMs = nullptr;
    QDoubleSpinBox* m_spinTransitAngleStep = nullptr;
    QSpinBox* m_spinValidationPoseCount = nullptr;
    QDoubleSpinBox* m_spinValidationBCSep = nullptr;
    QCheckBox* m_checkValidationAvoidRejected = nullptr;
    QDoubleSpinBox* m_spinValidationHazardBCSep = nullptr;
    QDoubleSpinBox* m_spinValidationRms = nullptr;
    QDoubleSpinBox* m_spinValidationMax = nullptr;
    QDoubleSpinBox* m_spinValidationCenterXz = nullptr;

    QPushButton* m_btnStartPreview = nullptr;
    QPushButton* m_btnStopPreview = nullptr;
    QPushButton* m_btnStage1Capture = nullptr;
    std::vector<QPushButton*> m_stage1NudgeButtons;
    QPushButton* m_btnReadPose = nullptr;
    QPushButton* m_btnFit = nullptr;
    QPushButton* m_btnTeach = nullptr;
    QPushButton* m_btnBootstrapSolve = nullptr;
    QPushButton* m_btnGenerateStage1Candidates = nullptr;
    QPushButton* m_btnMoveStage1Candidate = nullptr;
    QPushButton* m_btnSkipStage1Candidate = nullptr;
    QPushButton* m_btnSaveStage1Result = nullptr;
    QPushButton* m_btnLoadStage1Result = nullptr;
    QPushButton* m_btnImportStage2Sphere = nullptr;
    QPushButton* m_btnFitStage2Sphere = nullptr;
    QLabel* m_lblStage2Sphere = nullptr;
    QLabel* m_lblStage2MotionReadiness = nullptr;
    QPushButton* m_btnStage2MotionControls = nullptr;
    QPushButton* m_btnGenerate = nullptr;
    QPushButton* m_btnCancelStage2Plan = nullptr;
    QPushButton* m_btnStage2SimulationStart = nullptr;
    QPushButton* m_btnStage2SimulationStop = nullptr;
    QPushButton* m_btnStage2SimulationClear = nullptr;
    QPushButton* m_btnAutoAcquireStage2 = nullptr;
    QPushButton* m_btnStopAutoAcquireStage2 = nullptr;
    QPushButton* m_btnGenerateValidation = nullptr;
    QPushButton* m_btnAutoAcquireValidation = nullptr;
    QPushButton* m_btnStopAutoAcquireValidation = nullptr;
    QPushButton* m_btnMove = nullptr;
    QPushButton* m_btnSkipPlanPose = nullptr;
    QPushButton* m_btnRecenter = nullptr;
    QPushButton* m_btnCapture = nullptr;
    QPushButton* m_btnCaptureValidation = nullptr;
    QPushButton* m_btnImportValidation = nullptr;
    QPushButton* m_btnExportValidation = nullptr;
    QPushButton* m_btnRunValidation = nullptr;
    QPushButton* m_btnClearValidation = nullptr;
    QPushButton* m_btnDeleteLast = nullptr;
    QPushButton* m_btnClear = nullptr;
    QPushButton* m_btnSolve = nullptr;
    QPushButton* m_btnCancelStage2Solve = nullptr;
    QPushButton* m_btnSaveResult = nullptr;
    QPushButton* m_btnLoadResult = nullptr;
    QPushButton* m_btnExportSession = nullptr;
    QPushButton* m_btnImportSession = nullptr;
    QPushButton* m_btnBindLoadedResult = nullptr;
    QPushButton* m_btnStopMove = nullptr;
    QPushButton* m_btnStage2StopMove = nullptr;

    QTableWidget* m_stage1CandidateTable = nullptr;
    QTableWidget* m_stage1ResultTable = nullptr;
    QTableWidget* m_planTable = nullptr;
    QTableWidget* m_resultTable = nullptr;
    QLabel* m_lblValidation = nullptr;
    QLabel* m_lblStage1Status = nullptr;
    QLabel* m_lblStage1ResultSummary = nullptr;
    QLabel* m_lblStage2Gate = nullptr;
    QLabel* m_lblKinematicGate = nullptr;
    QProgressBar* m_stage1CaptureProgress = nullptr;
    QProgressBar* m_stage2PlanProgress = nullptr;
    QLabel* m_lblStage2PlanProgress = nullptr;
    QProgressBar* m_stage2SolveProgress = nullptr;
    QLabel* m_lblStage2SolveProgress = nullptr;
    QLabel* m_lblStage2SolveElapsed = nullptr;
    QPlainTextEdit* m_stage2SolveDetails = nullptr;
    QTimer* m_stage2SolveTimer = nullptr;

    LaserProfile m_latestProfile;
    bool m_hasLatestProfile = false;
    QMetaObject::Connection m_profileCaptureConnection;
    mls::calibration::SphereCenterFit m_latestFit;
    bool m_hasLatestFit = false;
    QVector<QVector3D> m_stage2ImportedSphereCloud;
    QString m_stage2ImportedSphereCloudPath;

    bool m_referenceTaught = false;
    bool m_referenceCaptured = false;
    bool m_sensorConventionLocked = false;
    bool m_referenceFlipX = false;
    bool m_referenceFlipZ = false;
    // Live machine pose source used by the current session. After the XYZ grating
    // upgrade this is normally 31 (XYZ/C/B all measured MPOS).
    std::uint8_t m_referenceFeedbackMask = 0;
    // Provenance of the Physical17 model currently in m_lastResult. Keep this
    // separate from the live source so a legacy mask-24 model can run on a mask-31
    // machine without ever being relabelled as if it had been calibrated in MPOS.
    std::uint8_t m_modelFeedbackMask = 0;
    mls::calibration::PosePlannerSeed m_referenceSeed;
    mls::calibration::Vec3d m_sphereCenterGuessMachine;
    mls::calibration::NominalHandEyeSphereResult m_bootstrapResult;
    bool m_hasBootstrap = false;
    int m_bootstrapRecordCount = 0;
    qint64 m_stage1SolveElapsedMs = 0;
    std::vector<JointPose5D> m_stage1Candidates;
    std::vector<JointPose5D> m_loadedStage1CapturedPoses;
    std::vector<QString> m_stage1CandidateStatus;
    int m_activeStage1Candidate = -1;

    std::vector<mls::calibration::CalibrationPoseInstruction> m_plan;
    std::vector<CaptureRecord> m_records;
    std::vector<CaptureRecord> m_validationRecords;
    mls::calibration::Physical17Parameters m_currentEstimate;
    mls::calibration::SingleSphere17Result m_lastResult;
    bool m_hasResult = false;
    // Last successful model kept while a new MPOS calibration is in progress.
    // It may still be used at runtime only when its own hold-out validation passed.
    mls::calibration::SingleSphere17Result m_fallbackResult;
    ValidationSummary m_fallbackValidationSummary;
    bool m_hasFallbackResult = false;
    bool m_planBootstrap = true;
    bool m_fullPlanGenerated = false;
    bool m_planValidation = false;
    bool m_refinementSolved = false;
    bool m_needFitBeforeTransit = false;
    PendingMoveKind m_pendingMoveKind = PendingMoveKind::None;
    int m_pendingMovePlanIndex = -1;
    mls::calibration::SingleSphere17Calibrator m_calibrator;
    ValidationSummary m_validationSummary;

    // Last fully accepted calibration is persisted separately from the temporary
    // calibration session. It becomes usable only after the saved sensor/controller
    // coordinate convention has been re-verified in the current application run.
    bool m_persistentCalibrationLoaded = false;
    bool m_persistentCalibrationBound = false;
    ValidationSummary m_persistedValidationSummary;
    QString m_persistentBindError;
    QString m_persistentSourcePath;
    bool m_persistentNeedsCanonicalSave = false;
    bool m_lastRuntimeCalibrationUsable = false;
    bool m_runtimeUsableSignalInitialized = false;
    QString m_startupRestoreMessage;

    bool m_collecting = false;
    bool m_capturePendingStart = false;
    bool m_stage1SolveRunning = false;
    bool m_stage2PlanRunning = false;
    qint64 m_stage2PlanStartMs = 0;
    qint64 m_stage2PlanPhaseStartMs = 0;
    mls::calibration::Physical17PlanningPhase m_stage2PlanPhase =
        mls::calibration::Physical17PlanningPhase::Preparing;
    std::shared_ptr<std::atomic_bool> m_stage2PlanCancelFlag;
    QFutureWatcher<Stage2PlanGenerationResult>* m_stage2PlanWatcher = nullptr;
    bool m_stage2SolveRunning = false;
    qint64 m_stage2SolveStartMs = 0;
    std::shared_ptr<std::atomic_bool> m_stage2SolveCancelFlag;
    JointState5D m_captureBefore;
    QVector<LaserProfile> m_captureFrames;
    int m_capturePlanIndex = -1;
    bool m_captureValidation = false;
    bool m_moveInProgress = false;
    bool m_autoPlanMove = false;
    int m_autoPlanIndex = -1;
    bool m_autoStage2Acquisition = false;
    bool m_autoStage2StopRequested = false;
    bool m_autoValidationAcquisition = false;
    bool m_autoValidationStopRequested = false;
    bool m_stage2SimulationActive = false;
    int m_stage2SimulationIndex = 0;
    quint64 m_stage2SimulationToken = 0;
    QVector<QVector3D> m_stage2SimulationCloud;
};
