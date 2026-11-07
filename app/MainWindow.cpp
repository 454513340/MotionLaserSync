#include <array>
#include <exception>
#include "app/MainWindow.h"
#include "calibration/SphereTwoStageFit.h"


#include "core/ZmcController.h"
#include "core/LaserSensor.h"
#include "core/SyncManager.h"
#include "core/HardwareSyncReport.h"
#include "core/PointCloudBuilder.h"
#include "calibration/ExposureAnalysisEngine.h"
#include "calibration/ExposureTimingPolicy.h"
#include "calibration/SensorCalibrationTiming.h"
#include "calibration/MaterialExposureLibrary.h"
#include "kinematics/MachineKinematics5Axis.h"
#include "trajectory/JointTrajectoryPlanner5Axis.h"
#include "safety/MachineSafetyModel.h"
#include "safety/SiteAcceptancePolicy.h"
#include "widgets/Profile2DWidget.h"
#include "widgets/ProfileImageWidget.h"
#include "widgets/PointCloudWidget.h"
#include "widgets/AxisControlWidget.h"
#include "widgets/VideoWidget.h"
#include "widgets/GCodeWidget.h"
#include "widgets/LogWidget.h"
#include "widgets/SaveFormatDialog.h"
#include "widgets/CalibrationWidget.h"
#include "widgets/PathPlanningWidget.h"
#include "widgets/PointCloudProcessWidget.h"
#include "gcode/GCodeExecutor.h"
#include "gcode/GCodeSyncMetadata.h"
#include "gcode/HardwareNcProbeLogic.h"
#include "core/SensorTriggerReadbackLogic.h"
#include "twin/MultiPoseNcSimulation.h"
#include "twin/MultiPoseOpticalCoverage.h"
#include "calibration/SingleSphere17Calibration.h"
#include <QCryptographicHash>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QStandardPaths>
#include <QSaveFile>
#include <map>
#include <QStringList>
#include "utils/CsvExporter.h"
#include "utils/GlobalTimeSync.h"
#include "utils/LogManager.h"
#include "utils/AsyncSaveManager.h"
#include "utils/CompactScanFormat.h"
#include "utils/PointCloudLoader.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QTabWidget>
#include <QTableWidget>
#include <QHeaderView>
#include <QSet>
#include <QLineEdit>
#include <QLabel>
#include <QPushButton>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QProgressBar>
#include <QComboBox>
#include <QRadioButton>
#include <QButtonGroup>
#include <QCheckBox>
#include <QMenuBar>
#include <QMenu>
#include <QAction>
#include <QStatusBar>
#include <QMessageBox>
#include <QFileDialog>
#include <QFile>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonArray>
#include <QJsonObject>
#include <QDir>
#include <QDateTime>
#include <QFrame>
#include <QTextEdit>
#include <QTextStream>
#include <QStringConverter>
#include <QDialog>
#include <QDialogButtonBox>
#include <QFormLayout>
#include <QDoubleValidator>
#include <QSplitter>
#include <QScrollArea>
#include <QStackedWidget>
#include <QPixmap>
#include <QPointF>
#include <QVector>
#include <QScreen>
#include <QApplication>
#include <QCoreApplication>
#include <QEventLoop>
#include <QElapsedTimer>
#include <QRegularExpression>
#include <QSignalBlocker>
#include <cmath>
#include <algorithm>
#include <limits>
#include <filesystem>
#include <functional>
#include <QTextStream>
#include <QtConcurrent>
#include <QThreadPool>
#include <QCloseEvent>
#include <QFutureWatcher>
#include <QProcess>
#include <array>
#include <QtMath>

namespace {

    struct RotaryInitResult
    {
        bool ok = false;
        double cDeg = 0.0;
        double bDeg = 0.0;
        QString error;
    };

    struct SensorConnectResult
    {
        bool ok = false;
        QString error;
    };

    QString formatTransformMatrix(const QMatrix4x4& m)
    {
        QString out;
        QTextStream ts(&out);
        for (int r = 0; r < 4; ++r) {
            ts << "[ ";
            for (int c = 0; c < 4; ++c) {
                ts << QString::number(m(r, c), 'f', 8);
                if (c != 3) ts << "  ";
            }
            ts << " ]\n";
        }
        return out;
    }

    QString axisName5D(int axis)
    {
        switch (axis) {
        case AxisX: return QStringLiteral("X");
        case AxisY: return QStringLiteral("Y");
        case AxisZ: return QStringLiteral("Z");
        case AxisC: return QStringLiteral("C");
        case AxisB: return QStringLiteral("B");
        default: return QStringLiteral("?");
        }
    }

    bool parseDiagnosticDoubleList(const QString& text,
        QVector<double>& values,
        QString* error,
        bool requirePositive)
    {
        values.clear();
        const QStringList parts = text.split(
            QRegularExpression(QStringLiteral("[,;，；\\s]+")),
            Qt::SkipEmptyParts);
        if (parts.isEmpty()) {
            if (error) *error = QObject::tr("列表不能为空");
            return false;
        }
        if (parts.size() > 64) {
            if (error) *error = QObject::tr("列表最多允许64个数值");
            return false;
        }

        for (const QString& token : parts) {
            bool ok = false;
            const double value = token.toDouble(&ok);
            if (!ok || !std::isfinite(value)) {
                if (error) *error = QObject::tr("无法解析数值“%1”").arg(token);
                return false;
            }
            if (requirePositive && value <= 0.0) {
                if (error) *error = QObject::tr("测试距离必须大于0，当前值=%1").arg(value, 0, 'g', 12);
                return false;
            }
            bool duplicate = false;
            for (const double existing : values) {
                if (std::abs(existing - value) < 1.0e-9) {
                    duplicate = true;
                    break;
                }
            }
            if (!duplicate) values.push_back(value);
        }

        if (values.isEmpty()) {
            if (error) *error = QObject::tr("列表没有有效数值");
            return false;
        }
        return true;
    }

    QString formatDiagnosticDoubleList(const QVector<double>& values)
    {
        QStringList parts;
        for (const double value : values) parts << QString::number(value, 'g', 8);
        return parts.join(QStringLiteral(", "));
    }

    QStringList parseSimpleCsvRecord(const QString& line)
    {
        QStringList fields;
        QString current;
        bool quoted = false;
        for (int i = 0; i < line.size(); ++i) {
            const QChar ch = line.at(i);
            if (ch == QLatin1Char('"')) {
                if (quoted && i + 1 < line.size() && line.at(i + 1) == QLatin1Char('"')) {
                    current += QLatin1Char('"');
                    ++i;
                }
                else {
                    quoted = !quoted;
                }
            }
            else if (ch == QLatin1Char(',') && !quoted) {
                fields.push_back(current.trimmed());
                current.clear();
            }
            else {
                current += ch;
            }
        }
        fields.push_back(current.trimmed());
        return fields;
    }

    double transformPositionErrorMm(const QMatrix4x4& target, const QMatrix4x4& actual)
    {
        const double dx = static_cast<double>(target(0, 3) - actual(0, 3));
        const double dy = static_cast<double>(target(1, 3) - actual(1, 3));
        const double dz = static_cast<double>(target(2, 3) - actual(2, 3));
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    double transformOrientationErrorDeg(const QMatrix4x4& target, const QMatrix4x4& actual)
    {
        // relative R = R_target^T * R_actual，旋转角由 trace(relative) 得到。
        double trace = 0.0;
        for (int i = 0; i < 3; ++i) {
            double diagonal = 0.0;
            for (int k = 0; k < 3; ++k) {
                diagonal += static_cast<double>(target(k, i)) * static_cast<double>(actual(k, i));
            }
            trace += diagonal;
        }
        double cosAngle = (trace - 1.0) * 0.5;
        if (cosAngle > 1.0) cosAngle = 1.0;
        if (cosAngle < -1.0) cosAngle = -1.0;
        return qRadiansToDegrees(std::acos(cosAngle));
    }


    QString reflectionSelectionName(int value)
    {
        switch (value) {
        case 0: return QStringLiteral("All");
        case 1: return QStringLiteral("First");
        case 2: return QStringLiteral("Last");
        case 3: return QStringLiteral("LargestArea");
        case 4: return QStringLiteral("MaximumIntensity");
        default: return QStringLiteral("Unknown");
        }
    }

    QString laserPowerName(int value)
    {
        switch (value) {
        case 0: return QStringLiteral("Off");
        case 1: return QStringLiteral("Reduced");
        case 2: return QStringLiteral("Standard");
        default: return QStringLiteral("Unknown");
        }
    }

    QJsonObject advancedParamsReadableJson(const SensorAdvancedParams& p)
    {
        QJsonObject o;
        o[QStringLiteral("measuringFieldIndex")] = p.measuringFieldIndex;
        o[QStringLiteral("resolution")] = static_cast<int>(p.resolution);
        o[QStringLiteral("laserPower")] = p.laserPower;
        o[QStringLiteral("laserPowerName")] = laserPowerName(p.laserPower);
        o[QStringLiteral("thresholdPercent")] = p.thresholdPercent;
        o[QStringLiteral("reflectionSelect")] = p.reflectionSelect;
        o[QStringLiteral("reflectionSelectName")] = reflectionSelectionName(p.reflectionSelect);
        o[QStringLiteral("flipX")] = p.flipX;
        o[QStringLiteral("flipZ")] = p.flipZ;
        o[QStringLiteral("autoExposure")] = p.autoExposure;
        o[QStringLiteral("shutterAlign")] = p.shutterAlign;
        o[QStringLiteral("autoExposureMode")] = p.autoExpMode;
        o[QStringLiteral("highResolution")] = p.highResolution;
        o[QStringLiteral("calibration")] = p.calibration;
        o[QStringLiteral("ambientLightSuppression")] = p.ambientLightSuppression;
        o[QStringLiteral("videoFilter")] = p.videoFilter;
        return o;
    }

    QJsonObject writeOnlyPeakFilterJson(const SensorAdvancedParams& p)
    {
        QJsonObject o;
        o[QStringLiteral("minWidth")] = p.peakWidthMin;
        o[QStringLiteral("maxWidth")] = p.peakWidthMax;
        o[QStringLiteral("minIntensity")] = p.peakIntensityMin;
        o[QStringLiteral("maxIntensity")] = p.peakIntensityMax;
        o[QStringLiteral("provenance")] = QStringLiteral("UI/write-intent; reliable device readback intentionally not used on current 2950/SDK");
        return o;
    }

} // namespace

namespace {
QByteArray canonicalHardwareNcDigest(QString text)
{
    text.replace(QStringLiteral("\r\n"),QStringLiteral("\n"));
    return QCryptographicHash::hash(text.toUtf8(),QCryptographicHash::Sha256);
}
}

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
    , m_zmc(std::make_unique<ZmcController>())
    , m_laser(std::make_unique<LaserSensor>())
    , m_syncManager(std::make_unique<SyncManager>(m_laser.get(), m_zmc.get()))
    , m_cloudBuilder(std::make_unique<PointCloudBuilder>())
{
    // ===== 注册元类型（必须在日志系统初始化前） =====
    qRegisterMetaType<LogEntry>("LogEntry");
    qRegisterMetaType<LogLevel>("LogLevel");

    // ===== 初始化日志系统（放在try-catch中保护） =====
    try {
        LogManager::instance().initialize("./logs", "MotionLaserSync");
    }
    catch (...) {
        // 日志系统初始化失败不应阻止程序运行
    }

    // 初始化轴参数
    for (int ax = AxisX; ax <= AxisB; ++ax) {
        m_axisParams[ax] = AxisParams{};
    }

    // 注册元类型
    qRegisterMetaType<LaserProfile>("LaserProfile");
    qRegisterMetaType<QVector<AlignedPoint>>("QVector<AlignedPoint>");
    qRegisterMetaType<QVector<QVector3D>>("QVector<QVector3D>");
    qRegisterMetaType<AxisParams>("AxisParams");
    qRegisterMetaType<JointPose5D>("JointPose5D");
    qRegisterMetaType<EndPose5D>("EndPose5D");
    qRegisterMetaType<InverseKinematics5DResult>("InverseKinematics5DResult");
    qRegisterMetaType<JointState5D>("JointState5D");
    qRegisterMetaType<JointMotionPlan5D>("JointMotionPlan5D");
    qRegisterMetaType<JointMoveResult5D>("JointMoveResult5D");
    qRegisterMetaType<ExecutorState>("ExecutorState");
    qRegisterMetaType<GCodeBlock>("GCodeBlock");
    qRegisterMetaType<ExecutorProgress>("ExecutorProgress");

    createUi();
    createMenus();
    loadMaterialExposureLibrary();
    resetAxisAlarmMonitor();
    initializeMachineDigitalTwin();

    // 创建G代码执行器
    m_gcodeExecutor = std::make_unique<GCodeExecutor>(
        m_zmc.get(), m_laser.get(), m_syncManager.get(), m_cloudBuilder.get());
    // First G00 is a real positioning move: evaluate LIVE MPOS -> target using
    // the existing machine FCL and compensation envelope. This is independent
    // of whether the machine was already sitting at the NC start at import time.
    // Does not act as NC program release approval.
    m_gcodeExecutor->setSiteEntryMoveVerifier([this](const JointPose5D& live,
        const JointPose5D& target,const QByteArray&,QString* failure) -> bool {
        if(!m_zmc || !m_zmc->isConnected() || !m_zmc->areAllAxesHomed()){
            if(failure)*failure=tr("现场五轴连接/真实回零状态不满足首段定位条件");
            return false;
        }
        if(!m_zmc->checkMotionHealth5D(failure) || !m_zmc->areAllAxesIdle(failure))return false;
        if(!m_ncWorkpieceSafety || !m_ncWorkpieceSafety->isReady()){
            if(failure)*failure=tr("本次NC没有包含零件STEP的完整FCL安全模型");
            return false;
        }
        const bool compensationOn=m_zmc->axisCompensationEnabled();
        const auto scan=m_ncWorkpieceSafety->checkTransition(live,target,compensationOn);
        if(!scan.modelReady || !scan.safe){
            if(failure)*failure=tr("当前MPOS至首G00的机床/STEP碰撞复核失败：%1")
                .arg(QString::fromStdString(scan.message));
            return false;
        }
        return checkSphereStudyTransitionSafety(live,target,failure,false);
    });
    // An OfflineOnly NC can now be conditionally released, but NEVER simply
    // because its comment was changed or two UI parameters match. These proofs
    // must be from this app's real tests of the identical NC and applied model.
    m_gcodeExecutor->setSiteHardwareNcVerifier(
        [this](const GCodeProgram& nc,const QByteArray& rawDiskDigest,QString* failure)->bool {
        const auto reject=[&](const QString& reason)->bool{
            if(failure)*failure=reason;
            return false;
        };
        if(m_closeRequested || m_gcodeHardwareNcProbeActive ||
           m_gcodeOfflineGeometryAuditWatcher.isRunning())
            return reject(tr("试采、几何复核或程序关闭状态尚未结束"));
        if(!m_geometryNcProof.passed || !m_hardwareNcProof.passed)
            return reject(tr("此NC尚未分别通过整条NC机床FCL复核和实际硬件同步试采"));
        if(nc.filePath.isEmpty())return reject(tr("待执行NC没有可核验的磁盘文件"));
        QFile file(nc.filePath);
        if(!file.open(QIODevice::ReadOnly))return reject(tr("无法读取待执行NC磁盘原件"));
        const QByteArray raw=file.readAll();
        if(QCryptographicHash::hash(raw,QCryptographicHash::Sha256)!=rawDiskDigest)
            return reject(tr("文件版本在开始执行前发生变化"));
        const QString text=QString::fromUtf8(raw);
        const QByteArray key=canonicalHardwareNcDigest(text);
        if(key.isEmpty() || m_geometryNcProof.normalizedNcDigest!=key ||
           m_hardwareNcProof.normalizedNcDigest!=key)
            return reject(tr("FCL/试采所验证的NC版本与本次即将执行的原文不同"));
        const auto now=std::chrono::steady_clock::now();
        if(m_geometryNcProof.connectionGeneration!=m_controllerSensorConnectionGeneration ||
           m_hardwareNcProof.connectionGeneration!=m_controllerSensorConnectionGeneration)
            return reject(tr("试采/复核后设备发生断线或重连，证据失效"));
        if(m_hardwareNcProof.topology!=nc.hardwareTopologyId ||
           m_hardwareNcProof.frequency!=nc.hardwareFrequencyHz ||
           m_hardwareNcProof.edge!=(nc.hardwareTopologyId==1?104:nc.cmmR1Edge))
            return reject(tr("试采采用的H1/H3/Hz/R1不是本次NC实际要求"));
        const SensorParams& tested=m_hardwareNcProof.sensorParams;
        if(tested.exposureTime!=nc.sensorParams.exposureTime ||
           tested.idleTime!=nc.sensorParams.idleTime ||
           tested.resolutionIndex!=nc.sensorParams.resolutionIndex ||
           tested.measuringFieldIndex!=nc.sensorParams.measuringFieldIndex)
            return reject(tr("启动时传感器曝光/空闲/分辨率/测量场与真实试采不同"));
        std::array<double,17> current{};
        if(!m_syncManager || !m_syncManager->currentKinematicModel(&current))
            return reject(tr("当前生效的Physical17模型不可读取"));
        for(std::size_t i=0;i<17;++i){
            if(!std::isfinite(current[i]) ||
               current[i]!=m_geometryNcProof.model[i] ||
               current[i]!=m_hardwareNcProof.model[i])
                return reject(tr("当前生效的17参数与FCL/试采时的数值不同，需重新验证"));
        }
        // Stage 3 checks the machine and the immutable, imported STEP against
        // the exact NC. It does not depend on a prior optical-planning report.
        // Physical fixtures not represented in the collision scene remain
        // uncertified; simulation alone never releases real axis movement.
        if(!m_machineSafety || !m_machineSafety->isReady())
            return reject(tr("机床FCL碰撞模型目前不可用"));
        if(!m_ncWorkpieceSafety || !m_ncWorkpieceSafety->isReady() ||
           m_geometryCadIdentity.isEmpty())
            return reject(tr("STEP零件尚未与本次整条NC的碰撞模型绑定"));
        mls::twin::TwinScene currentPart;
        QByteArray currentCadIdentity;
        bool reviewedOffline=false;
        QString partError;
        if(!m_pathPlanningWidget ||
           !m_pathPlanningWidget->snapshotCollisionWorkpiece(
               currentPart,currentCadIdentity,reviewedOffline,&partError) ||
           currentCadIdentity!=m_geometryCadIdentity)
            return reject(tr("STEP原文件或CAD→MCS位姿在FCL复核后已变化：%1；请重新诊断")
                .arg(partError));
        // Step 4 verifies an independently measured installation and records
        // the human fixture/guarding inspection. Stage 3 PASS alone cannot set
        // this bit. Binding to actual NC/STEP/session prevents stale approvals.
        if(!m_geometryNcProof.independentPartRegistrationCertified)
            return reject(tr("请完成诊断第4步的工件独立测量、夹具和现场防护验收；第1~3步通过后可点击‘第4步：现场测量验收’补做"));
        if(m_geometryNcProof.siteAcceptedCadIdentity!=m_geometryCadIdentity ||
           m_geometryNcProof.siteAcceptedNcDigest!=key ||
           m_geometryNcProof.siteAcceptedConnectionGeneration!=m_controllerSensorConnectionGeneration)
            return reject(tr("现场验收对应的工件/NC/设备会话不一致；请重新验收"));
        if(now-m_geometryNcProof.siteAcceptedAt>std::chrono::minutes(30))
            return reject(tr("现场验收超过30分钟未启动执行；请确认摆放未变化并重新完成第4步"));
        if(!m_zmc || m_zmc->axisCompensationEnabled()!=m_geometryNcProof.compensationEnabled)
            return reject(tr("XYZ运动补偿模式与整条NC FCL复核时不同，必须重新检查"));
        if(!m_zmc || !m_zmc->isConnected() || !m_laser || !m_laser->isConnected() ||
           !m_zmc->areAllAxesHomed())
            return reject(tr("控制器/传感器断线或五轴未在本会话正确回零"));
        QString health,idle,table,tableError;bool tableReady=false;
        if(!m_zmc->checkMotionHealth5D(&health) || !m_zmc->areAllAxesIdle(&idle) ||
           !m_zmc->hwTableCommissioningPreflight(table,&tableError,&tableReady) || !tableReady)
            return reject(tr("实时轴状态或硬件同步TABLE不合格：%1 %2 %3").arg(health,idle,tableError));
        if(failure)failure->clear();
        return true;
    });

    // Re-evaluate the runtime calibration after a capture/G-code task stops. If
    // calibration became invalid while a task was active, this clears the stale
    // model before another task can start.
    connect(m_syncManager.get(), &SyncManager::runningChanged, this, [this](bool running) {
        if (running) {
            refreshHardwareTemporalRuntimeUi();
            return;
        }
        // SyncManager emits runningChanged(false) before its stop() stack unwinds.
        // Queue the calibration refresh so m_stopping has already been released;
        // otherwise a valid Physical17 model is spuriously rejected as "still stopping".
        QTimer::singleShot(0, this, [this]() {
            if (!m_syncManager || m_syncManager->isRunning()) return;
            std::array<double, 17> parameters{};
            const bool usable = m_calibrationWidget && m_calibrationWidget->scanCalibration(parameters);
            syncValidatedCalibrationToRuntime(usable, false);
            refreshHardwareTemporalRuntimeUi();
            });
        });
    connect(m_gcodeExecutor.get(), &GCodeExecutor::stateChanged, this,
        [this](ExecutorState, ExecutorState) {
            if (!m_gcodeExecutor || m_gcodeExecutor->isRunning() || m_gcodeExecutor->isPaused()) return;
            std::array<double, 17> parameters{};
            const bool usable = m_calibrationWidget && m_calibrationWidget->scanCalibration(parameters);
            syncValidatedCalibrationToRuntime(usable, false);
        });
    {
        std::array<double, 17> parameters{};
        const bool usable = m_calibrationWidget && m_calibrationWidget->scanCalibration(parameters);
        syncValidatedCalibrationToRuntime(usable, false);
    }

    // 设置G代码执行器到控件
    if (m_gcodeWidget) {
        m_gcodeWidget->setExecutor(m_gcodeExecutor.get());
    }

    // 连接控制器错误信号
    connect(m_zmc.get(), &ZmcController::errorOccurred,
        this, [this](const QString& msg) {
            statusBar()->showMessage(msg, 5000);
        });

    // 连接同步管理器信号
    connect(m_syncManager.get(), &SyncManager::alignedDataReady,
        this, &MainWindow::onAlignedDataReady,
        Qt::QueuedConnection);
    connect(m_syncManager.get(), &SyncManager::errorOccurred,
        this, [this](const QString& msg) {
            const auto stats = m_syncManager->getHardwareAlignStats();
            if (m_syncManager->syncMode() == SyncMode::Hardware && stats.alignmentFault) {
                m_lblCaptureStatus->setText(tr("采集无效（故障已锁存，请停止）"));
                statusBar()->showMessage(msg); // persistent until operator action
            }
            else {
                statusBar()->showMessage(msg, 5000);
            }
        });

    // 连接同步管理器到点云构建器
    connect(m_syncManager.get(), &SyncManager::alignedDataReady,
        m_cloudBuilder.get(), &PointCloudBuilder::onAlignedDataReady,
        Qt::QueuedConnection);

    // V27.6.16M: 2D GUI no longer consumes every 200 Hz profile via Qt queued events.
    // LaserSensor keeps the latest decoded profile in memory and the GUI samples it
    // at ~30 Hz. Formal acquisition still receives every profile independently.
    // 连接 Video 帧信号
    connect(m_laser.get(), &LaserSensor::videoFrameReceived,
        this, &MainWindow::onVideoFrameReceived,
        Qt::QueuedConnection);

    // 定时器
    connect(&m_statusTimer, &QTimer::timeout, this, &MainWindow::onStatusTimer);
    m_statusTimer.start(Constants::kStatusTimerIntervalMs);

    connect(&m_posTimer, &QTimer::timeout, this, &MainWindow::onPosTimer);
    m_posTimer.start(Constants::kPosTimerIntervalMs);

    connect(&m_profileUiTimer, &QTimer::timeout, this, &MainWindow::onProfileUiTimer);
    m_profileUiTimer.start(33); // ~30 Hz latest-only display; never backlogs 200 Hz frames

    statusBar()->showMessage(tr("未连接"));

    // 控制器连接改为完全人工触发。
    // 启动阶段不调用任何 ZAux 网络连接函数：即使 PC 网卡/IP 尚未配置、
    // 控制器未上电或网线未连接，主界面也应保持可操作并可正常退出。
    statusBar()->showMessage(tr("未连接；请配置控制器网络后点击“连接”手动连接"));

    // 全屏显示（在所有初始化完成后）
    showMaximized();
}


void MainWindow::initializeMachineDigitalTwin()
{
    m_machineSafety = std::make_unique<mls::safety::MachineSafetyModel>();

    const QString appDir = QCoreApplication::applicationDirPath();
    const QStringList candidates = {
        QDir(appDir).filePath(QStringLiteral("models/machine5x/machine_config.json")),
        QDir::current().filePath(QStringLiteral("models/machine5x/machine_config.json"))
    };
    QString configPath;
    for (const QString& candidate : candidates) {
        if (QFileInfo::exists(candidate)) { configPath = QFileInfo(candidate).absoluteFilePath(); break; }
    }
    if (configPath.isEmpty()) {
        m_machineSafetyReady = false;
        if (m_pointCloudView)
            m_pointCloudView->setMachineSafetyStatus(tr("机床模型：未找到 models/machine5x/machine_config.json"), true);
        return;
    }

    m_machineModelConfigPath = configPath;
    std::string safetyError;
    m_machineSafetyReady = m_machineSafety->load(
        std::filesystem::path(configPath.toStdWString()), &safetyError);

    QString visualError;
    const bool visualOk = m_pointCloudView
        ? m_pointCloudView->loadMachineModel(configPath, &visualError)
        : false;

    if (!m_machineSafetyReady) {
        const QString msg = tr("机床FCL安全模型加载失败：%1")
            .arg(QString::fromLocal8Bit(safetyError.c_str()));
        if (m_pointCloudView) m_pointCloudView->setMachineSafetyStatus(msg, true);
        LOG_ERROR(LogCategory::Controller, msg);
        return;
    }

    const auto& cfg = m_machineSafety->config();
    const QString msg = tr("机床数字孪生已加载：%1 个碰撞部件，轮廓=%2；模型零位=XYZBC MPOS零位；关节裕量=%3mm/%4°，球安全间隙=%5mm")
        .arg(cfg.links.size()).arg(QString::fromStdString(cfg.collisionProfile))
        .arg(cfg.safety.linearJointMarginMm, 0, 'f', 1).arg(cfg.safety.rotaryJointMarginDeg, 0, 'f', 1)
        .arg(cfg.safety.studySphereClearanceMm, 0, 'f', 1);
    if (m_pointCloudView) {
        if (visualOk) m_pointCloudView->setMachineSafetyStatus(msg, false);
        else m_pointCloudView->setMachineSafetyStatus(
            tr("FCL已加载；VTK可视模型加载失败：%1").arg(visualError), true);
    }
    LOG_INFO(LogCategory::Controller, msg);
}

void MainWindow::updateMachineDigitalTwinPose(const JointPose5D& pose, bool evaluateCollision)
{
    if (m_pointCloudView)
        m_pointCloudView->setMachineJointPose(pose.x, pose.y, pose.z, pose.c, pose.b);
    if (!evaluateCollision || !m_machineSafetyReady || !m_machineSafety) return;

    const auto result = m_machineSafety->checkPose(pose, false);
    QStringList links;
    for (const auto& name : result.collisionLinks)
        links << QString::fromStdString(name);
    if (m_pointCloudView) {
        m_pointCloudView->setMachineCollisionLinks(links);
        if (result.safe) {
            m_pointCloudView->setMachineSafetyStatus(
                tr("机床模型：XYZBC MPOS实时驱动 / 当前姿态无碰撞"), false);
        }
        else {
            m_pointCloudView->setMachineSafetyStatus(
                tr("碰撞警告：%1").arg(QString::fromStdString(result.message)), true);
        }
    }
}

bool MainWindow::sphereStudySafetyReady(QString* error) const
{
    if (!m_machineSafety || !m_machineSafetyReady || !m_machineSafety->isReady()) {
        if (error) *error = tr("机床FCL碰撞模型未加载。自动球面研究采用Fail-Closed策略，禁止生成/执行自动路径。");
        return false;
    }
    return true;
}

bool MainWindow::resolveSphereStudyPlanningLimits(
    bool intersectLiveControllerLimits,
    mls::calibration::SphereStudyPathOptions& options,
    QString* source,
    QString* error) const
{
    if (!m_machineSafety || !m_machineSafetyReady || !m_machineSafety->isReady()) {
        if (error) *error = tr("机床安全模型未就绪，无法取得离线规划限位。");
        return false;
    }

    const auto& cfg = m_machineSafety->config();
    const auto& p = cfg.planningLimits;
    auto validRange = [](double lo, double hi) {
        return std::isfinite(lo) && std::isfinite(hi) && lo < hi;
        };
    if (!p.configured || !validRange(p.xMinMm, p.xMaxMm) ||
        !validRange(p.yMinMm, p.yMaxMm) || !validRange(p.zMinMm, p.zMaxMm) ||
        !validRange(p.cMinDeg, p.cMaxDeg) || !validRange(p.bMinDeg, p.bMaxDeg)) {
        if (error) *error = tr(
            "Machine5X 未配置 offline_planning_limits。离线规划禁止使用 AxisParams 默认的 ±999999 占位软限位，"
            "请检查 models/machine5x/machine_config.json。");
        return false;
    }

    double xLo = p.xMinMm, xHi = p.xMaxMm;
    double yLo = p.yMinMm, yHi = p.yMaxMm;
    double zLo = p.zMinMm, zHi = p.zMaxMm;
    double cLo = p.cMinDeg, cHi = p.cMaxDeg;
    double bLo = p.bMinDeg, bHi = p.bMaxDeg;
    QString limitSource = tr("Machine5X离线规划限位(%1)")
        .arg(p.source.empty() ? tr("machine_config.json") : QString::fromStdString(p.source));

    if (intersectLiveControllerLimits) {
        const auto saneAxis = [&](int axis)->bool {
            const auto& a = m_axisParams[axis];
            return std::isfinite(static_cast<double>(a.softLimitNeg)) &&
                std::isfinite(static_cast<double>(a.softLimitPos)) &&
                a.softLimitNeg < a.softLimitPos &&
                // AxisParams{} uses +/-999999 as an explicit "not read" placeholder.
                std::abs(static_cast<double>(a.softLimitNeg)) < 100000.0 &&
                std::abs(static_cast<double>(a.softLimitPos)) < 100000.0;
            };
        if (!saneAxis(AxisX) || !saneAxis(AxisY) || !saneAxis(AxisZ) ||
            !saneAxis(AxisC) || !saneAxis(AxisB)) {
            if (error) *error = tr("控制器已作为实时参考，但五轴软限位仍是未读取/异常值；拒绝用占位限位规划。");
            return false;
        }
        xLo = std::max(xLo, static_cast<double>(m_axisParams[AxisX].softLimitNeg));
        xHi = std::min(xHi, static_cast<double>(m_axisParams[AxisX].softLimitPos));
        yLo = std::max(yLo, static_cast<double>(m_axisParams[AxisY].softLimitNeg));
        yHi = std::min(yHi, static_cast<double>(m_axisParams[AxisY].softLimitPos));
        zLo = std::max(zLo, static_cast<double>(m_axisParams[AxisZ].softLimitNeg));
        zHi = std::min(zHi, static_cast<double>(m_axisParams[AxisZ].softLimitPos));
        cLo = std::max(cLo, static_cast<double>(m_axisParams[AxisC].softLimitNeg));
        cHi = std::min(cHi, static_cast<double>(m_axisParams[AxisC].softLimitPos));
        bLo = std::max(bLo, static_cast<double>(m_axisParams[AxisB].softLimitNeg));
        bHi = std::min(bHi, static_cast<double>(m_axisParams[AxisB].softLimitPos));
        limitSource += tr(" ∩ 控制器实时软限位");
    }

    const double linearMargin = std::max(0.0, cfg.safety.linearJointMarginMm);
    const double rotaryMargin = std::max(0.0, cfg.safety.rotaryJointMarginDeg);
    options.xMinMm = xLo + linearMargin; options.xMaxMm = xHi - linearMargin;
    options.yMinMm = yLo + linearMargin; options.yMaxMm = yHi - linearMargin;
    options.zMinMm = zLo + linearMargin; options.zMaxMm = zHi - linearMargin;
    options.cMinDeg = cLo + rotaryMargin; options.cMaxDeg = cHi - rotaryMargin;
    options.bMinDeg = bLo + rotaryMargin; options.bMaxDeg = bHi - rotaryMargin;

    if (!validRange(options.xMinMm, options.xMaxMm) ||
        !validRange(options.yMinMm, options.yMaxMm) ||
        !validRange(options.zMinMm, options.zMaxMm) ||
        !validRange(options.cMinDeg, options.cMaxDeg) ||
        !validRange(options.bMinDeg, options.bMaxDeg)) {
        if (error) *error = tr("Machine5X/控制器联合限位减去安全裕量后没有有效的五轴规划范围。");
        return false;
    }

    if (source) *source = limitSource;
    if (error) error->clear();
    return true;
}

bool MainWindow::checkSphereStudyPoseSafety(const JointPose5D& pose, QString* error, bool updateView)
{
    if (!sphereStudySafetyReady(error)) return false;
    const auto result = m_machineSafety->checkPose(pose, true);
    if (updateView) {
        QStringList links;
        for (const auto& name : result.collisionLinks) links << QString::fromStdString(name);
        if (m_pointCloudView) {
            m_pointCloudView->setMachineJointPose(pose.x, pose.y, pose.z, pose.c, pose.b);
            m_pointCloudView->setMachineCollisionLinks(links);
        }
    }
    if (!result.safe) {
        if (error) *error = tr("目标姿态碰撞/间隙检查失败：%1").arg(QString::fromStdString(result.message));
        return false;
    }
    return true;
}

bool MainWindow::checkSphereStudyTransitionSafety(const JointPose5D& from, const JointPose5D& to,
    QString* error, bool updateView)
{
    if (!sphereStudySafetyReady(error)) return false;
    const bool compensationOn = m_zmc && m_zmc->axisCompensationEnabled();
    if (compensationOn) {
        double envelopeMm = 0.0;
        QString envelopeError;
        if (!m_zmc->axisCompensationSafetyEnvelopeMm(envelopeMm, &envelopeError)) {
            if (error) *error = tr("无法取得XYZ补偿安全包络：%1").arg(envelopeError);
            return false;
        }
        const double clearanceMm = std::max(0.0, m_machineSafety->config().safety.minimumStaticClearanceMm);
        // L1 demand-correction envelope is deliberately conservative for the
        // nested XYZ chain.  Requiring configured clearance to dominate it
        // prevents a command-space perturbation from invalidating a nominal
        // collision-free path.
        if (!(clearanceMm > envelopeMm + 1.0e-6)) {
            if (error) *error = tr("MachineSafety静态间隙%1 mm不足以覆盖XYZ补偿路径包络%2 mm；拒绝运动")
                .arg(clearanceMm, 0, 'f', 6).arg(envelopeMm, 0, 'f', 6);
            return false;
        }
    }
    const auto result = m_machineSafety->checkTransition(from, to, compensationOn);
    if (!result.safe) {
        if (updateView && m_pointCloudView && result.firstUnsafeSample >= 0) {
            const auto& q = result.firstUnsafePose;
            m_pointCloudView->setMachineJointPose(q.x, q.y, q.z, q.c, q.b);
            QStringList links;
            for (const auto& name : result.unsafePoseResult.collisionLinks)
                links << QString::fromStdString(name);
            m_pointCloudView->setMachineCollisionLinks(links);
            m_pointCloudView->setMachineSafetyStatus(
                tr("过渡路径碰撞：t=%1，%2")
                .arg(result.firstUnsafeT, 0, 'f', 3)
                .arg(QString::fromStdString(result.unsafePoseResult.message)), true);
        }
        if (error) *error = tr("相邻位姿整段扫掠碰撞检查失败：%1")
            .arg(QString::fromStdString(result.message));
        return false;
    }
    return true;
}


void MainWindow::beginHardwareTemporalPlannedScan(quint64 requestId,
    const JointPose5D& start,
    const JointPose5D& end,
    double feedMmPerSec)
{
    auto reject = [this, requestId](const QString& why) {
        LOG_WARNING(LogCategory::Sync, tr("硬件同步标定路径拒绝: %1").arg(why));
        if (m_calibrationWidget)
            m_calibrationWidget->notifyHardwareTemporalScanFinished(requestId, false, why);
        };

    if (m_hardwareTemporalScanActive) {
        reject(tr("已有硬件同步标定路径正在执行"));
        return;
    }
    if (!m_zmc || !m_zmc->isConnected() || !m_laser || !m_laser->isConnected()) {
        reject(tr("控制器和scanCONTROL必须同时连接"));
        return;
    }
    if (!m_syncManager) {
        reject(tr("SyncManager不可用"));
        return;
    }
    if (m_syncManager->isRunning() || m_jointMoveInProgress
        || (m_gcodeExecutor && m_gcodeExecutor->isRunning())) {
        reject(tr("已有运动、在线采集或G代码任务正在执行"));
        return;
    }
    if (m_laser->isVideoRunning() || m_profilePreviewRunning) {
        reject(tr("请先停止Video/轮廓预览"));
        return;
    }
    if (m_syncManager->hardwareTemporalCorrection().enabled) {
        reject(tr("标定/验证必须使用RAW R0或R1五轴锁存位姿，请先禁用运行时τ补偿"));
        return;
    }
    if (!std::isfinite(feedMmPerSec) || feedMmPerSec <= 0.0) {
        reject(tr("扫描速度必须为有限正数"));
        return;
    }
    if (!start.isFinite() || !end.isFinite()) {
        reject(tr("规划起点/终点含NaN或Inf"));
        return;
    }
    const double dx = end.x - start.x, dy = end.y - start.y, dz = end.z - start.z;
    const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (!(distance > 1.0e-6)) {
        reject(tr("扫描线距离过小"));
        return;
    }
    if (std::abs(end.c - start.c) > 1.0e-9 || std::abs(end.b - start.b) > 1.0e-9) {
        reject(tr("V27.6.50第一阶段轨迹求解仅允许XYZ空间直线，B/C必须固定；五轴旋转留给后续全点云验证"));
        return;
    }
    if (!m_zmc->areLinearMposReferenced()) {
        reject(tr("XYZ光栅MPOS尚未通过正规DATUM回零建立机床基准"));
        return;
    }
    if (!m_zmc->isRotaryAbsoluteReady(AxisC) || !m_zmc->isRotaryAbsoluteReady(AxisB)
        || !m_zmc->isRotaryServoEnabled(AxisC) || !m_zmc->isRotaryServoEnabled(AxisB)) {
        reject(tr("B/C必须完成绝对位置同步并人工使能"));
        return;
    }

    JointState5D state;
    QString err;
    if (!m_zmc->readJointState5D(state, &err) || !state.allIdle) {
        if (err.isEmpty()) err = tr("至少一个轴仍在运动");
        reject(err);
        return;
    }

    QString safetyError;
    if (!checkSphereStudyPoseSafety(start, &safetyError, false)
        || !checkSphereStudyPoseSafety(end, &safetyError, false)) {
        reject(tr("规划端点未通过MachineSafety：%1").arg(safetyError));
        return;
    }
    if (!checkSphereStudyTransitionSafety(state.mpos, start, &safetyError, false)) {
        reject(tr("当前位置→扫描起点未通过MachineSafety：%1").arg(safetyError));
        return;
    }
    if (!checkSphereStudyTransitionSafety(start, end, &safetyError, false)) {
        reject(tr("扫描起点→终点未通过MachineSafety：%1").arg(safetyError));
        return;
    }

    const double prepositionScale = 0.20;
    const JointMotionPlan5D preposition = JointTrajectoryPlanner5Axis::planPointToPoint(
        state.mpos, start, currentAxisParams5D(), prepositionScale);
    if (!preposition.valid) {
        reject(tr("扫描起点定位规划失败：%1").arg(preposition.error));
        return;
    }

    m_hardwareTemporalScanActive = true;
    m_hardwareTemporalMotionStarted = false;
    ++m_hardwareTemporalGeneration;
    const quint64 generation = m_hardwareTemporalGeneration;
    m_hardwareTemporalRequestId = requestId;
    m_hardwareTemporalStart = start;
    m_hardwareTemporalEnd = end;
    m_hardwareTemporalFeedMmPerSec = feedMmPerSec;
    m_hardwareTemporalPreparedDurationSec = 0.0;
    m_hardwareTemporalPhaseStartMs = QDateTime::currentMSecsSinceEpoch();
    m_hardwareTemporalDeadlineMs = 0;
    setJointMoveUiRunning(true);

    LOG_INFO(LogCategory::Sync,
        tr("[硬件同步标定] 路径%1：先定位扫描起点 XYZ=(%2,%3,%4)，随后才Arm V32.13硬件同步")
        .arg(requestId).arg(start.x, 0, 'f', 4).arg(start.y, 0, 'f', 4).arg(start.z, 0, 'f', 4));

    if (!preposition.hasMotion) {
        JointMoveResult5D result;
        result.completed = true;
        result.finalState = state;
        QTimer::singleShot(0, this, [this, generation, result] {
            prepareHardwareTemporalScanAfterPreposition(generation, true, result, tr("已在扫描起点"));
            });
        return;
    }

    ++m_axisWorkerCount;
    ZmcController* controller = m_zmc.get();
    (void)QtConcurrent::run([this, controller, generation, preposition]() {
        JointMoveResult5D result;
        QString moveErr;
        const bool ok = controller->moveJointAbs5D(preposition, result, &moveErr);
        QMetaObject::invokeMethod(this, [this, generation, result, ok, moveErr]() {
            --m_axisWorkerCount;
            setJointMoveUiRunning(false);
            if (generation != m_hardwareTemporalGeneration) {
                tryFinishDeferredClose();
                return;
            }
            prepareHardwareTemporalScanAfterPreposition(
                generation, ok, result, ok ? result.message : moveErr);
            tryFinishDeferredClose();
            }, Qt::QueuedConnection);
        });
}

void MainWindow::prepareHardwareTemporalScanAfterPreposition(quint64 generation, bool success,
    const JointMoveResult5D& result,
    const QString& message)
{
    if (!m_hardwareTemporalScanActive || generation != m_hardwareTemporalGeneration) return;
    if (!success) {
        finishHardwareTemporalPlannedScan(generation, false,
            tr("定位扫描起点失败：%1").arg(message));
        return;
    }
    QString arrivalError;
    if (result.completed && !sphereStudyArrivalWithinTolerance(result, &arrivalError)) {
        finishHardwareTemporalPlannedScan(generation, false,
            tr("扫描起点到位门禁失败：%1").arg(arrivalError));
        return;
    }

    // Stage exactly one straight XYZ line while the ordinary status-timer merge
    // pump is gated by m_hardwareTemporalMotionStarted=false.  This guarantees
    // that the path cannot move before SyncManager has received its Probe.
    PathPoint p;
    p.x = m_hardwareTemporalEnd.x; p.y = m_hardwareTemporalEnd.y; p.z = m_hardwareTemporalEnd.z;
    p.c = m_hardwareTemporalEnd.c; p.b = m_hardwareTemporalEnd.b;
    p.axisMask = 31;
    p.feed = m_hardwareTemporalFeedMmPerSec;
    p.feedMode = mls::runtime::FeedMode::LegacyPerSecond;
    p.continuous = false;
    AxisParams motion = m_axisParams[AxisX];
    motion.speed = static_cast<float>(m_hardwareTemporalFeedMmPerSec);
    QString err;
    if (!m_zmc->prepareMergePath({ p }, motion, &err, ZmcController::PathStartReference::MeasuredMpos)) {
        finishHardwareTemporalPlannedScan(generation, false,
            tr("扫描直线准备失败：%1").arg(err));
        return;
    }
    m_hardwareTemporalPreparedDurationSec = std::max(0.0, m_zmc->trajectoryDurationSec());

    if (!startHardwareTemporalRawCapture(&err)) {
        finishHardwareTemporalPlannedScan(generation, false,
            tr("V32.13硬件同步Arm失败：%1").arg(err));
        return;
    }
    m_hardwareTemporalPhaseStartMs = QDateTime::currentMSecsSinceEpoch();
    LOG_INFO(LogCategory::Sync,
        tr("[硬件同步标定] 路径%1已预装但保持静止；V32.13已Arm，等待Probe/运行时频率同步建立")
        .arg(m_hardwareTemporalRequestId));
    QTimer::singleShot(20, this, [this, generation] {pollHardwareTemporalSyncReady(generation); });
}

bool MainWindow::startHardwareTemporalRawCapture(QString* error)
{
    if (!m_laser || !m_laser->isConnected()) {
        if (error) *error = tr("scanCONTROL未连接");
        return false;
    }
    if (!m_zmc || !m_zmc->isConnected() || !m_syncManager) {
        if (error) *error = tr("控制器/SyncManager不可用");
        return false;
    }
    if (m_laser->isVideoRunning() || m_profilePreviewRunning || m_syncManager->isRunning()) {
        if (error) *error = tr("Video/轮廓预览/其它采集任务仍在运行");
        return false;
    }
    if (m_syncManager->hardwareTemporalCorrection().enabled) {
        if (error) *error = tr("运行时τ补偿仍处于启用状态，拒绝采集标定RAW数据");
        return false;
    }

    QString preflight, preflightErr;
    bool executionAuthorized = false;
    if (!m_zmc->hwTableCommissioningPreflight(preflight, &preflightErr, &executionAuthorized)
        || !executionAuthorized) {
        m_hardwareSyncPreflightPassed = false;
        if (error) *error = preflightErr.isEmpty()
            ? tr("控制器未发布V32.13 SINGLE-OP14硬件执行授权") : preflightErr;
        return false;
    }
    m_hardwareSyncPreflightPassed = true;

    // Select the topology explicitly requested in System Calibration, NOT the
    // normal-scan combobox or a hidden H1 default. H3 uses CMM/IN1/R1.
    const int calibrationMode = m_calibrationWidget
        ? m_calibrationWidget->hardwareTemporalCalibrationTopologyId() : 1;
    if (calibrationMode != 1 && calibrationMode != 3) {
        if (error) *error = tr("标定界面返回无效拓扑；仅允许H1或H3");
        return false;
    }
    const int edge = calibrationMode==1 ? 104
        : (m_comboCmmR1Edge ? m_comboCmmR1Edge->currentData().toInt() : 114);
    if(!mls::runtime::Table5D::validTopologyEdge(calibrationMode,edge)) {
        if(error)*error=tr("H3 R1锁存边沿无效，只允许114/115");
        return false;
    }
    const int lockedEdge = m_calibrationWidget
        ? m_calibrationWidget->hardwareTemporalExpectedR1Edge() : 104;
    if(calibrationMode==3 && lockedEdge!=0 && lockedEdge!=edge) {
        if(error)*error=tr("当前H3 Session已锁定R1边沿%1，当前主界面为%2；禁止混合不同边沿标定")
            .arg(lockedEdge).arg(edge);
        return false;
    }
    m_syncManager->setHardwareTopology(
        static_cast<mls::runtime::HardwareSyncTopology>(calibrationMode), edge);
    LOG_INFO(LogCategory::Calibration,
        tr("[硬件时域标定] 本条RAW扫描使用H%1，锁存边沿=%2（H1:R0，H3:R1/CMM）；运行时补偿OFF")
            .arg(calibrationMode).arg(edge));
    m_syncManager->setTriggerType(HardwareTriggerType::EdgeRising);
    m_syncManager->setSyncMode(SyncMode::Hardware);
    m_lastCaptureMode = SyncMode::Hardware;
    SensorParams params;
    params.resolutionIndex = static_cast<unsigned int>(qMax(0, m_comboResolution->currentIndex()));
    params.exposureTime = static_cast<unsigned int>(m_spinExposure->value());
    params.idleTime = static_cast<unsigned int>(m_spinIdle->value());
    params.measuringFieldIndex = m_comboMeasuringField ? m_comboMeasuringField->currentData().toInt() : 7;
    m_syncManager->setSensorParams(params);
    m_syncManager->setHardwareTriggerFrequencyHz(hardwareSyncFrequencyHz());

    // One planned line == one isolated hardware-sync session, so no data from a
    // previous line can leak into its sphere fit or REGIST velocity estimate.
    //
    // V27.6.50: the automatic temporal workflow intentionally suppresses the
    // live alignedDataReady fan-out.  The temporal workflow additionally uses
    // RAW-profile capture mode: ordinal pairing remains online/lossless, but the
    // GUI thread does not expand each valid sensor point into AlignedPoint.
    // PointCloudBuilder/VTK therefore receive no second million-point copy.
    m_syncManager->setHardwareAlignedDataEmissionEnabled(false);
    m_syncManager->setHardwareRawProfileCaptureEnabled(true);
    m_syncManager->clearData();
    if (m_cloudBuilder) m_cloudBuilder->clear();
    if (m_profile2DView) m_profile2DView->clear();
    if (m_profileImageView) m_profileImageView->clear();
    if (m_pointCloudView) m_pointCloudView->clear();
    GlobalTimeSync::instance().start();

    QString startErr;
    if (!m_syncManager->start(&startErr)) {
        m_syncManager->setHardwareAlignedDataEmissionEnabled(true);
        m_syncManager->setHardwareRawProfileCaptureEnabled(false);
        if (error) *error = startErr;
        return false;
    }
    if (m_lblCaptureStatus) m_lblCaptureStatus->setText(tr("硬件同步标定：等待Probe..."));
    updateVideoControlState();
    refreshHardwareTemporalRuntimeUi();
    if (error) error->clear();
    return true;
}

void MainWindow::pollHardwareTemporalSyncReady(quint64 generation)
{
    if (!m_hardwareTemporalScanActive || generation != m_hardwareTemporalGeneration) return;
    if (!m_syncManager || !m_syncManager->isRunning()) {
        finishHardwareTemporalPlannedScan(generation, false, tr("等待Probe期间硬件同步意外停止"));
        return;
    }
    const auto stats = m_syncManager->getHardwareAlignStats();
    if (stats.alignmentFault) {
        finishHardwareTemporalPlannedScan(generation, false,
            tr("Probe/对齐故障：%1").arg(stats.faultReason));
        return;
    }
    if (stats.syncEstablished && stats.totalProfiles >= 1 && stats.totalPoseSamples >= 1) {
        startHardwareTemporalLineMotion(generation);
        return;
    }
    const qint64 elapsed = QDateTime::currentMSecsSinceEpoch() - m_hardwareTemporalPhaseStartMs;
    if (elapsed > 5000) {
        finishHardwareTemporalPlannedScan(generation, false, tr("等待V32.13 Probe建立同步超时(5 s)"));
        return;
    }
    QTimer::singleShot(20, this, [this, generation] {pollHardwareTemporalSyncReady(generation); });
}

void MainWindow::startHardwareTemporalLineMotion(quint64 generation)
{
    if (!m_hardwareTemporalScanActive || generation != m_hardwareTemporalGeneration) return;
    m_hardwareTemporalMotionStarted = true; // releases the ordinary merge-pump gate
    m_hardwareTemporalPhaseStartMs = QDateTime::currentMSecsSinceEpoch();
    const qint64 plannedMs = qint64(std::ceil(std::max(0.05, m_hardwareTemporalPreparedDurationSec) * 1000.0));
    m_hardwareTemporalDeadlineMs = m_hardwareTemporalPhaseStartMs + std::max<qint64>(5000, plannedMs * 4 + 3000);

    // A single line normally fits in one refill batch.  Kick it immediately so
    // the scan begins just after syncEstablished instead of waiting for the next
    // periodic GUI timer; subsequent refill ownership remains unchanged.
    bool queued = false;
    QString err;
    if (!m_zmc->pumpMergeBuffer(queued, &err)) {
        finishHardwareTemporalPlannedScan(generation, false,
            tr("扫描直线下发失败：%1").arg(err));
        return;
    }
    LOG_INFO(LogCategory::Sync,
        tr("[硬件同步标定] Probe已建立；开始路径%1硬件同步扫描，V=%2 mm/s，预计运动%3 s")
        .arg(m_hardwareTemporalRequestId).arg(m_hardwareTemporalFeedMmPerSec, 0, 'f', 4)
        .arg(m_hardwareTemporalPreparedDurationSec, 0, 'f', 3));
    if (m_lblCaptureStatus) m_lblCaptureStatus->setText(tr("硬件同步标定：扫描中..."));

    // Avoid controller status polling during the main part of the 200-Hz TABLE
    // session.  First completion check is scheduled near the planned end.
    const int firstCheckMs = int(std::clamp<qint64>(plannedMs + 100, 100, 60000));
    QTimer::singleShot(firstCheckMs, this, [this, generation] {pollHardwareTemporalLineComplete(generation); });
}

void MainWindow::pollHardwareTemporalLineComplete(quint64 generation)
{
    if (!m_hardwareTemporalScanActive || generation != m_hardwareTemporalGeneration) return;
    if (!m_syncManager || !m_syncManager->isRunning()) {
        finishHardwareTemporalPlannedScan(generation, false, tr("扫描过程中硬件同步意外停止"));
        return;
    }
    const auto stats = m_syncManager->getHardwareAlignStats();
    if (stats.alignmentFault) {
        finishHardwareTemporalPlannedScan(generation, false,
            tr("扫描过程中硬件对齐故障：%1").arg(stats.faultReason));
        return;
    }
    if (QDateTime::currentMSecsSinceEpoch() > m_hardwareTemporalDeadlineMs) {
        m_zmc->requestStopJointMove5D(false);
        finishHardwareTemporalPlannedScan(generation, false, tr("规划扫描运动完成超时"));
        return;
    }
    if (!m_zmc->isTrajectoryComplete()) {
        QTimer::singleShot(100, this, [this, generation] {pollHardwareTemporalLineComplete(generation); });
        return;
    }

    // Stop() performs the existing tail drain/count closure.  The widget then
    // independently checks Pose/Profile/Aligned/Pending/Dropped before accepting.
    m_hardwareTemporalMotionStarted = false;
    m_syncManager->stop();
    updateVideoControlState();
    const auto finalStats = m_syncManager->getHardwareAlignStats();
    if (finalStats.alignmentFault) {
        finishHardwareTemporalPlannedScan(generation, false,
            tr("Stop/Drain后硬件同步闭环失败：%1").arg(finalStats.faultReason));
        return;
    }
    finishHardwareTemporalPlannedScan(generation, true,
        tr("扫描完成并Stop/Drain：Pose=%1, Profile=%2, Aligned=%3")
        .arg(finalStats.totalPoseSamples).arg(finalStats.totalProfiles).arg(finalStats.alignedProfiles));
}

void MainWindow::finishHardwareTemporalPlannedScan(quint64 generation, bool success,
    const QString& message)
{
    if (generation != m_hardwareTemporalGeneration) return;
    const quint64 requestId = m_hardwareTemporalRequestId;
    if (!success && m_hardwareTemporalMotionStarted && m_zmc)
        m_zmc->requestStopJointMove5D(false);
    if (m_syncManager && m_syncManager->isRunning())
        m_syncManager->stop();
    if (m_syncManager)
        m_syncManager->setHardwareAlignedDataEmissionEnabled(true);
    m_hardwareTemporalMotionStarted = false;
    m_hardwareTemporalScanActive = false;
    m_hardwareTemporalRequestId = 0;
    setJointMoveUiRunning(false);
    updateVideoControlState();
    refreshHardwareTemporalRuntimeUi();
    pollAxisAlarmStates(false);
    LOG_INFO(LogCategory::Sync,
        tr("[硬件同步标定] 路径%1结束：%2；%3")
        .arg(requestId).arg(success ? tr("成功") : tr("失败")).arg(message));
    if (m_calibrationWidget)
        m_calibrationWidget->notifyHardwareTemporalScanFinished(requestId, success, message);
    // notifyHardwareTemporalScanFinished() synchronously transfers the stopped
    // RAW Profile + REGIST capture to its background worker on success.
    if (m_syncManager) {
        m_syncManager->setHardwareRawProfileCaptureEnabled(false);
        if (!success) m_syncManager->clearData();
    }
}

void MainWindow::abortHardwareTemporalPlannedScan()
{
    if (!m_hardwareTemporalScanActive) return;
    ++m_hardwareTemporalGeneration; // invalidate every outstanding timer/worker callback

    // A pre-position move may still be blocking inside the controller worker.
    // Request the grouped five-axis stop, but keep the ordinary motion UI lock
    // until that worker returns.  Its generation-mismatch callback is the owner
    // that finally releases the lock, avoiding a short window in which another
    // manual move could be started while the old worker is still unwinding.
    const bool waitForMoveWorker = m_jointMoveInProgress && m_axisWorkerCount > 0;
    if (m_zmc) m_zmc->requestStopJointMove5D(false);
    if (m_syncManager && m_syncManager->isRunning()) m_syncManager->stop();
    if (m_syncManager) {
        m_syncManager->setHardwareAlignedDataEmissionEnabled(true);
        m_syncManager->setHardwareRawProfileCaptureEnabled(false);
        m_syncManager->clearData();
    }
    m_hardwareTemporalMotionStarted = false;
    m_hardwareTemporalScanActive = false;
    m_hardwareTemporalRequestId = 0;
    if (!waitForMoveWorker) setJointMoveUiRunning(false);
    updateVideoControlState();
    pollAxisAlarmStates(false);
    LOG_WARNING(LogCategory::Sync, tr("[硬件同步标定] 操作者已停止自动路径采集流程"));
}

bool MainWindow::sphereStudyArrivalWithinTolerance(const JointMoveResult5D& result, QString* error) const
{
    if (!m_machineSafetyReady || !m_machineSafety) {
        if (error) *error = tr("机床安全模型未就绪，无法执行自动到位门禁。");
        return false;
    }
    const auto& cfg = m_machineSafety->config().safety;
    const double linearTol = std::max(0.0, cfg.arrivalLinearToleranceMm);
    const double rotaryTol = std::max(0.0, cfg.arrivalRotaryToleranceDeg);
    if (result.maxLinearMposErrorMm > linearTol + 1e-12
        || result.maxRotaryMposErrorDeg > rotaryTol + 1e-12) {
        if (error) {
            *error = tr("五轴实际MPOS未进入自动采集到位窗口：XYZ最大误差=%1 mm（限%2），B/C最大误差=%3°（限%4）。")
                .arg(result.maxLinearMposErrorMm, 0, 'f', 6).arg(linearTol, 0, 'f', 4)
                .arg(result.maxRotaryMposErrorDeg, 0, 'f', 6).arg(rotaryTol, 0, 'f', 4);
        }
        return false;
    }
    return true;
}

int MainWindow::applySphereStudySafetyFilterAndTransitionGate(
    const JointPose5D& startPose, bool gateStartTransition)
{
    if (!m_machineSafetyReady || !m_machineSafety) return 0;
    JointPose5D cursor = startPose;
    bool haveTransitionCursor = gateStartTransition;
    int safeCount = 0;
    int safetyRow = 0;
    const int safetyRows = static_cast<int>(m_sphereStudyPlan.size());
    for (auto& p : m_sphereStudyPlan) {
        ++safetyRow;
        if ((safetyRow % 4) == 1) {
            statusBar()->showMessage(tr("正在执行Machine5X路径安全检查 %1/%2...").arg(safetyRow).arg(safetyRows));
            QCoreApplication::processEvents(QEventLoop::ExcludeUserInputEvents, 5);
        }
        if (!p.valid) continue;
        const JointPose5D target{ p.joints.x, p.joints.y, p.joints.z, p.joints.c, p.joints.b };
        QString safetyError;
        if (!checkSphereStudyPoseSafety(target, &safetyError, false)) {
            p.valid = false;
            p.message = std::string("MachineSafety pose gate: ") + safetyError.toStdString();
            continue;
        }

        // Online planning checks the real current-MPOS -> first-pose sweep.
        // Offline planning has no meaningful live start pose, so the first valid
        // pose becomes the virtual playback origin; every subsequent segment is
        // still checked with the exact same swept Machine5X collision gate.
        if (haveTransitionCursor && !checkSphereStudyTransitionSafety(cursor, target, &safetyError, false)) {
            p.valid = false;
            p.message = std::string("MachineSafety transition gate: ") + safetyError.toStdString();
            continue;
        }
        cursor = target;
        haveTransitionCursor = true;
        ++safeCount;
    }
    return safeCount;
}

MainWindow::~MainWindow()
{
    m_closeRequested = true;
    if(m_gcodeGeometryCancel)m_gcodeGeometryCancel->store(true);
    if(m_gcodeOfflineGeometryAuditWatcher.isRunning())m_gcodeOfflineGeometryAuditWatcher.waitForFinished();
    if(m_gcodeHardwareNcProbeActive)finishGCodeHardwareNcProbe(true);
    m_axisCompSpeedCancel.store(true);
    m_axisCompCalCancel.store(true);
    m_axisCompValCancel.store(true);
    m_axisMoveDiagAutoCancel.store(true);
    m_statusTimer.stop();
    m_posTimer.stop();
    m_expTestTimer.stop();
    if (m_syncManager) m_syncManager->stop();

    // CalibrationWidget 是 QObject 子对象，会在 QMainWindow 基类析构阶段才销毁；
    // m_laser/m_zmc/m_cloudBuilder 则是 MainWindow 的 unique_ptr 成员，会更早析构。
    // 因此先显式解除外部引用，避免子控件析构时持有悬空 QObject*。
    if (m_calibrationWidget) {
        // Stage-2 pose planning may currently be using the read-only
        // MachineSafety callbacks on a QtConcurrent worker. Cancel and join it
        // before m_machineSafety (a MainWindow member) can be destroyed.
        m_calibrationWidget->shutdownBackgroundWork();
        m_calibrationWidget->setSensor(nullptr);
        m_calibrationWidget->setController(nullptr);
        m_calibrationWidget->setPointCloudBuilder(nullptr);
    }
}

void MainWindow::closeEvent(QCloseEvent* event)
{
    m_closeRequested = true;
    if(m_gcodeGeometryCancel)m_gcodeGeometryCancel->store(true);
    if(m_gcodeOfflineGeometryAuditWatcher.isRunning())m_gcodeOfflineGeometryAuditWatcher.waitForFinished();
    if(m_gcodeHardwareNcProbeActive)finishGCodeHardwareNcProbe(true);
    if (m_sphereExpActive) onStopSphereExposureSweep();
    if (m_small7Active) onStopSmall7Experiment();

    // The staged temporal-calibration workflow may own both a joint move and an
    // isolated V32.13 hardware-sync session.  Cancel it before generic teardown
    // so no queued workflow timer can re-arm OP14/REGIST acquisition while the
    // application is closing.
    if (m_hardwareTemporalScanActive) {
        abortHardwareTemporalPlannedScan();
    }
    if (m_calibrationWidget) {
        // Stop the Stage-2 planning worker before any close-time teardown can
        // race its MachineSafety callbacks.
        m_calibrationWidget->shutdownBackgroundWork();
    }
    if (m_sphereStudyAutoActive) {
        m_sphereStudyAutoStopRequested = true;
        ++m_sphereStudyAutoSequenceToken;
    }
    if (m_axisMoveDiagAutoRunning) {
        m_axisMoveDiagAutoCancel.store(true);
        if (m_zmc && m_zmc->isConnected()) {
            for (int axis = AxisX; axis <= AxisZ; ++axis) m_zmc->requestStopAxis(axis, false);
        }
    }
    if (m_axisCompSpeedRunning) {
        m_axisCompSpeedCancel.store(true);
        if (m_zmc && m_zmc->isConnected()) {
            for (int axis = AxisX; axis <= AxisZ; ++axis) m_zmc->requestStopAxis(axis, false);
        }
    }
    if (m_axisCompCalRunning) {
        m_axisCompCalCancel.store(true);
        if (m_zmc && m_zmc->isConnected()) {
            for (int axis = AxisX; axis <= AxisZ; ++axis) m_zmc->requestStopAxis(axis, false);
        }
    }
    if (m_axisCompValRunning) {
        m_axisCompValCancel.store(true);
        if (m_zmc && m_zmc->isConnected() && m_axisCompValAxis >= AxisX && m_axisCompValAxis <= AxisZ) {
            m_zmc->requestStopAxis(m_axisCompValAxis, false);
        }
    }

    // The distance/angle study owns a temporary direct profile connection.
    // Tear it down before stopping/disconnecting the sensor so no queued study
    // callback can touch hardware while the window is closing.
    if (m_sphereStudyProbeActive) {
        ++m_sphereStudyProbeWatchToken;
        m_sphereStudyProbeActive = false;
        if (m_sphereStudyProbeConnection) { QObject::disconnect(m_sphereStudyProbeConnection); m_sphereStudyProbeConnection = {}; }
        m_sphereStudyProbeProfiles.clear();
    }
    if (m_sphereStudyCaptureActive) {
        finishSphereStudyCapture(true, tr("程序正在关闭"));
    }

    // Stop GUI polling immediately so closing the window never competes with a
    // vendor-DLL call that is already running in a worker thread.
    m_statusTimer.stop();
    m_posTimer.stop();
    m_expTestTimer.stop();
    m_syncManager->stop();

    // 网络预检查是独立的 ping 子进程；关闭窗口时直接终止，不等待也不阻塞 GUI。
    if (m_controllerProbeProcess) {
        QObject::disconnect(m_controllerProbeProcess, nullptr, this, nullptr);
        m_controllerProbeProcess->kill();
        m_controllerProbeProcess->deleteLater();
        m_controllerProbeProcess = nullptr;
        m_controllerConnectInProgress = false;
    }

    if (m_axisWorkerCount > 0 && m_zmc && m_zmc->isConnected()
        && !m_controllerConnectInProgress && !m_rotaryInitInProgress) {
        // Wake blocking home/move workers so they can finish and release their
        // captures before MainWindow/ZmcController are destroyed. P4 five-axis
        // interpolation must be cancelled as one motion group instead of issuing
        // five independent Single_Cancel commands.
        if (m_jointMoveInProgress) {
            m_zmc->requestStopJointMove5D(false);
        }
        else {
            for (int axis = AxisX; axis <= AxisB; ++axis) {
                if (m_zmc->isAxisBusy(axis)) m_zmc->requestStopAxis(axis, false);
            }
        }
    }

    if (m_controllerConnectInProgress || m_rotaryInitInProgress
        || m_sensorConnectInProgress || m_axisWorkerCount > 0 || m_backgroundIoCount > 0) {
        statusBar()->showMessage(tr("正在结束设备通信/运动任务，请稍候..."));
        event->ignore();
        return;
    }

    // No worker is using the hardware objects now. Normal disconnect is safe and
    // keeps B/C OP24/OP26 shutdown behavior on a healthy controller.
    if (m_laser && m_laser->isConnected()) {
        m_laser->stopAll();
        m_laser->disconnectSensor();
    }
    if (m_zmc && m_zmc->isConnected()) {
        m_zmc->disconnect();
    }

    event->accept();
}

void MainWindow::tryFinishDeferredClose()
{
    if (!m_closeRequested) return;
    if (m_controllerConnectInProgress || m_rotaryInitInProgress
        || m_sensorConnectInProgress || m_axisWorkerCount > 0 || m_backgroundIoCount > 0) return;
    QTimer::singleShot(0, this, [this]() { close(); });
}

void MainWindow::syncValidatedCalibrationToRuntime(bool usable, bool announce)
{
    if (!m_syncManager) return;

    const bool scanRunning = m_syncManager->isRunning();
    const bool gcodeBusy = m_gcodeExecutor
        && (m_gcodeExecutor->isRunning() || m_gcodeExecutor->isPaused());
    if (scanRunning || gcodeBusy) {
        // Never hot-swap a kinematic model underneath active reconstruction or RTCP.
        // More importantly, loss of calibration authorization is fail-closed: do not
        // let a task continue indefinitely on a model that the calibration subsystem
        // has just revoked.  Defer the stop to the event loop to avoid re-entering the
        // calibration signal stack, then clear both runtime models once idle.
        if (!usable) {
            m_runtimeCalibrationAutoApplied = false;
            LOG_WARNING(LogCategory::Calibration,
                tr("运行中的任务检测到标定可用性失效；已请求安全停止，停止完成后将清除扫描/RTCP旧模型。"));
            if (!m_calibrationInvalidationStopQueued) {
                m_calibrationInvalidationStopQueued = true;
                QTimer::singleShot(0, this, [this]() {
                    if (m_gcodeExecutor
                        && (m_gcodeExecutor->isRunning() || m_gcodeExecutor->isPaused())) {
                        m_gcodeExecutor->stop();
                    }
                    if (m_syncManager && m_syncManager->isRunning()) {
                        m_syncManager->stop();
                    }
                    m_calibrationInvalidationStopQueued = false;
                    syncValidatedCalibrationToRuntime(false, false);
                    });
            }
        }
        return;
    }
    m_calibrationInvalidationStopQueued = false;

    if (!usable) {
        QString syncError;
        QString rtcpError;
        const bool syncOk = m_syncManager->clearKinematicModel(&syncError);
        const bool rtcpOk = !m_gcodeExecutor || m_gcodeExecutor->clearRtcpKinematicModel(&rtcpError);
        if (!syncOk || !rtcpOk) {
            LOG_WARNING(LogCategory::Calibration,
                tr("清除失效运行标定失败：扫描=%1；RTCP=%2")
                .arg(syncOk ? tr("OK") : syncError, rtcpOk ? tr("OK") : rtcpError));
        }
        else if (m_runtimeCalibrationAutoApplied || announce) {
            LOG_INFO(LogCategory::Calibration,
                tr("当前标定未满足独立验证+实机坐标约定闸门；扫描/RTCP运行模型已清除。"));
        }
        m_runtimeCalibrationAutoApplied = false;
        return;
    }

    if (!m_gcodeExecutor || !m_calibrationWidget) return;

    std::array<double, 17> parameters{};
    if (!m_calibrationWidget->scanCalibration(parameters)) {
        syncValidatedCalibrationToRuntime(false, announce);
        return;
    }

    QString syncError;
    if (!m_syncManager->setKinematicModel(parameters, false, &syncError)) {
        QString rtcpClearError;
        m_gcodeExecutor->clearRtcpKinematicModel(&rtcpClearError);
        m_runtimeCalibrationAutoApplied = false;
        LOG_WARNING(LogCategory::Calibration,
            tr("自动注入17参数到扫描模块失败：%1").arg(syncError));
        return;
    }

    QString rtcpError;
    if (!m_gcodeExecutor->setRtcpKinematicModel(parameters, false, &rtcpError)) {
        QString rollbackError;
        m_syncManager->clearKinematicModel(&rollbackError);
        m_runtimeCalibrationAutoApplied = false;
        LOG_WARNING(LogCategory::Calibration,
            tr("自动注入17参数到RTCP失败：%1；扫描模块已回滚清除。").arg(rtcpError));
        return;
    }

    const bool firstApply = !m_runtimeCalibrationAutoApplied;
    m_runtimeCalibrationAutoApplied = true;
    if (firstApply || announce) {
        LOG_INFO(LogCategory::Calibration,
            tr("已自动将同一套已验证Physical17模型注入点云重建和RTCP。"));
        statusBar()->showMessage(
            tr("已自动应用有效17参数标定：点云重建 + RTCP（当前绝对平移规范未外部锁定）"),
            10000);
    }
}

void MainWindow::createMenus()
{
    QMenu* fileMenu = menuBar()->addMenu(tr("文件(&F)"));

    QAction* applyScanModel = fileMenu->addAction(tr("重新同步当前有效17参数到扫描/RTCP..."));
    connect(applyScanModel, &QAction::triggered, this, [this]() {
        std::array<double, 17> parameters{};
        const bool usable = m_calibrationWidget && m_calibrationWidget->scanCalibration(parameters);
        if (!usable) {
            QMessageBox::warning(this, tr("未就绪"), tr(
                "当前没有一套“独立验证已通过 + 当前实机Flip/DPOS-MPOS约定已核对”的17参数标定。\n\n"
                "V27.6.16会自动检查AppData、程序目录calibration子目录，以及程序目录中的阶段1/阶段2旧版JSON；"
                "只有找到匹配的hold-out验证并在设备连接后通过坐标约定核对，才允许注入扫描/RTCP。"));
            syncValidatedCalibrationToRuntime(false, false);
            return;
        }
        if (QMessageBox::question(this, tr("重新同步运行模型"), tr(
            "软件通常会自动同步当前有效Physical17模型。此操作只用于重新注入同一套已验证模型到点云重建和RTCP。\n"
            "硬件触发、锁存与曝光时序仍属于尚未完成的独立验证项，不能由几何标定结果替代。确认继续？"))
            != QMessageBox::Yes) return;
        syncValidatedCalibrationToRuntime(true, true);
        });

    QAction* loadMotionLimits = fileMenu->addAction(tr("载入五轴轨迹速度/软件限位..."));
    connect(loadMotionLimits, &QAction::triggered, this, [this]() {
        const QString path = QFileDialog::getOpenFileName(this, tr("五轴轨迹限制（不修改硬件UNITS/IO）"), QString(), tr("JSON (*.json)"));
        if (path.isEmpty())return;
        QFile file(path); if (!file.open(QIODevice::ReadOnly)) { QMessageBox::warning(this, tr("读取失败"), file.errorString()); return; }
        const auto document = QJsonDocument::fromJson(file.readAll()); const auto object = document.object();
        if (!document.isObject() || object.value("axis_order").toString() != "XYZCB") {
            QMessageBox::warning(this, tr("格式错误"), tr("axis_order必须为XYZCB；不可把B/C调换")); return;
        }
        mls::runtime::Limits limits;
        auto readArray = [&](const char* key, mls::runtime::Joint& values) {
            const auto a = object.value(key).toArray(); if (a.size() != 5)return false;
            for (int i = 0; i < 5; ++i) { if (!a[i].isDouble())return false; values[i] = a[i].toDouble(); if (!std::isfinite(values[i]))return false; }return true;
            };
        if (!readArray("maximum_velocity", limits.speed) || !readArray("maximum_acceleration", limits.accel) ||
            !readArray("maximum_deceleration", limits.decel) || !readArray("minimum_position", limits.minimum) || !readArray("maximum_position", limits.maximum)) {
            QMessageBox::warning(this, tr("格式错误"), tr("每组限制必须包含5个有限数值，XYZ为mm，C/B为deg")); return;
        }
        if (QMessageBox::question(this, tr("确认轨迹限制"), tr("仅修改上位机轨迹规划限制；不能扩大控制器硬限位。请确认数值来自本机调试记录，且机器静止。")) != QMessageBox::Yes)return;
        QString error; if (!m_zmc->setTrajectoryLimits5D(limits, &error)) { QMessageBox::warning(this, tr("拒绝修改"), error); return; }
        statusBar()->showMessage(tr("已载入XYZCB轨迹限制；与控制器实际软限位取交集"), 10000);
        });

    QAction* actIoStatus = new QAction(tr("V19 通信调度状态(&I)..."), this);
    fileMenu->addAction(actIoStatus);
    connect(actIoStatus, &QAction::triggered, this, [this]() {
        QDialog dialog(this);
        dialog.setWindowTitle(tr("ECI3808 软实时通信调度状态"));
        dialog.resize(960, 620);
        auto* layout = new QVBoxLayout(&dialog);
        auto* text = new QTextEdit(&dialog);
        text->setReadOnly(true);
        const auto report = syncdiag::schedulerReport(m_zmc->ioStatistics());
        text->setPlainText(QString::fromUtf8(report.c_str()));
        layout->addWidget(text);
        auto* buttons = new QDialogButtonBox(QDialogButtonBox::Close, &dialog);
        connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
        layout->addWidget(buttons);
        dialog.exec();
        });

    QAction* actExportAlignment = new QAction(tr("导出对齐数据(&A)..."), this);
    fileMenu->addAction(actExportAlignment);
    connect(actExportAlignment, &QAction::triggered, this, &MainWindow::onExportAlignmentData);

    QAction* actExportCloud = new QAction(tr("导出点云(&P)..."), this);
    fileMenu->addAction(actExportCloud);
    connect(actExportCloud, &QAction::triggered, this, &MainWindow::onExportPointCloud);

    QAction* actImportCloud = new QAction(tr("导入点云(&I)..."), this);
    fileMenu->addAction(actImportCloud);
    connect(actImportCloud, &QAction::triggered, this, &MainWindow::onImportPointCloud);

    fileMenu->addSeparator();

    QAction* actExit = new QAction(tr("退出(&X)"), this);
    fileMenu->addAction(actExit);
    connect(actExit, &QAction::triggered, this, &QMainWindow::close);
}

void MainWindow::createUi()
{
    setWindowTitle(tr("MotionLaserSync V30.0.4 (FCL Only NC Preflight) - ECI3808 V32.13 / M0-M8 Digital Twin Workflow"));

    // 设置最小尺寸，确保界面可用
    setMinimumSize(1200, 800);

    auto* central = new QWidget(this);
    setCentralWidget(central);

    auto* mainLayout = new QVBoxLayout(central);
    mainLayout->setContentsMargins(5, 5, 5, 5);
    mainLayout->setSpacing(5);

    // ===== 上部：主工作区 =====
    auto* workSplitter = new QSplitter(Qt::Horizontal, central);

    // 左侧视图区域 (3/4)
    QWidget* leftPanel = createLeftViewPanel(workSplitter);
    workSplitter->addWidget(leftPanel);

    // 右侧控制区域 (1/4)
    QWidget* rightPanel = createRightControlPanel(workSplitter);
    workSplitter->addWidget(rightPanel);

    // 右侧包含嵌套标定/规划表单，不能压缩到约 300 px，否则控件会被裁切。
    // 仍允许用户拖动 splitter，但给控制区足够的初始宽度与最小宽度。
    rightPanel->setMinimumWidth(560);
    workSplitter->setCollapsible(1, false);
    QList<int> hSizes;
    hSizes << 1100 << 600;
    workSplitter->setSizes(hSizes);
    workSplitter->setStretchFactor(0, 2);
    workSplitter->setStretchFactor(1, 1);

    // ===== 使用垂直分割器分隔工作区和日志区 =====
    auto* mainSplitter = new QSplitter(Qt::Vertical, central);
    mainSplitter->addWidget(workSplitter);

    // ===== 下部：日志面板 =====
    auto* logGroup = new QGroupBox(tr("系统日志"), central);
    auto* logLayout = new QVBoxLayout(logGroup);
    logLayout->setContentsMargins(2, 5, 2, 2);

    m_logWidget = new LogWidget(logGroup);
    logLayout->addWidget(m_logWidget);

    mainSplitter->addWidget(logGroup);

    // 设置工作区和日志区的比例 (4:1)
    QList<int> vSizes;
    vSizes << 600 << 150;
    mainSplitter->setSizes(vSizes);
    mainSplitter->setStretchFactor(0, 4);
    mainSplitter->setStretchFactor(1, 1);

    // 设置日志区最小高度
    logGroup->setMinimumHeight(100);

    mainLayout->addWidget(mainSplitter);

    // 界面初始化完成日志延迟输出（避免在初始化期间触发信号）
    // LOG_INFO(LogCategory::UI, tr("界面初始化完成"));
}

QWidget* MainWindow::createLeftViewPanel(QWidget* parent)
{
    auto* panel = new QWidget(parent);
    auto* layout = new QVBoxLayout(panel);
    layout->setContentsMargins(0, 0, 0, 0);

    // 视图 Tab
    m_viewTabs = new QTabWidget(panel);

    // 三维点云（放在第一个）
    m_pointCloudView = new PointCloudWidget(m_viewTabs);
    m_pointCloudView->setPointCloudBuilder(m_cloudBuilder.get());
    // 设置工作体积：X=270mm, Y=270mm, Z=50mm(传感器测量范围)
    // Z 轴范围使用 2950-50BL 传感器的有效量程：70 ~ 120 mm
    m_pointCloudView->setWorkingVolume(270.0, 270.0,
        Constants::kSensorZRange,
        Constants::kSensorZMin,
        Constants::kSensorZMax);
    m_viewTabs->addTab(m_pointCloudView, tr("三维点云"));

    // 二维轮廓
    m_profile2DView = new Profile2DWidget(m_viewTabs);
    m_viewTabs->addTab(m_profile2DView, tr("二维轮廓"));

    // 轮廓图像
    m_profileImageView = new ProfileImageWidget(m_viewTabs);
    m_viewTabs->addTab(m_profileImageView, tr("轮廓图像"));

    // 传感预览Tab（仅显示Video/轮廓；控制面板已迁移至系统标定->传感标定）
    auto* debugContainer = new QWidget(m_viewTabs);
    auto* debugLayout = new QVBoxLayout(debugContainer);
    debugLayout->setContentsMargins(0, 0, 0, 0);

    m_previewStack = new QStackedWidget(debugContainer);
    m_videoWidget = new VideoWidget(m_previewStack);
    m_profilePreviewView = new Profile2DWidget(m_previewStack);
    m_previewStack->addWidget(m_videoWidget);        // index 0
    m_previewStack->addWidget(m_profilePreviewView); // index 1

    debugLayout->addWidget(m_previewStack);
    m_viewTabs->addTab(debugContainer, tr("传感预览"));

    layout->addWidget(m_viewTabs);

    return panel;
}

QWidget* MainWindow::createRightControlPanel(QWidget* parent)
{
    auto* panel = new QWidget(parent);
    auto* layout = new QVBoxLayout(panel);
    layout->setContentsMargins(5, 5, 5, 5);
    layout->setSpacing(5);

    // 1. 连接区域（始终显示在顶部）
    layout->addWidget(createConnectionSection(panel));

    // 2. 使用 Tab 切换参数设置和控制
    auto* controlTabs = new QTabWidget(panel);
    m_mainControlTabs = controlTabs;
    controlTabs->setUsesScrollButtons(true);
    controlTabs->setElideMode(Qt::ElideRight);

    // Tab 1: 参数设置
    auto* paramScrollArea = new QScrollArea(controlTabs);
    paramScrollArea->setWidgetResizable(true);
    paramScrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    paramScrollArea->setWidget(createParametersSection(paramScrollArea));
    controlTabs->addTab(paramScrollArea, tr("参数设置"));

    // Tab 2: 运动与采集控制
    auto* ctrlScrollArea = new QScrollArea(controlTabs);
    ctrlScrollArea->setWidgetResizable(true);
    ctrlScrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    ctrlScrollArea->setWidget(createControlSection(ctrlScrollArea));
    controlTabs->addTab(ctrlScrollArea, tr("控制"));

    // Tab 3: 系统标定
    auto* calibScrollArea = new QScrollArea(controlTabs);
    calibScrollArea->setWidgetResizable(true);
    calibScrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    m_calibrationWidget = new CalibrationWidget(calibScrollArea);
    m_calibrationWidget->setController(m_zmc.get());
    m_calibrationWidget->setSensor(m_laser.get());
    m_calibrationWidget->setPointCloudBuilder(m_cloudBuilder.get());
    connect(m_calibrationWidget, &CalibrationWidget::hardwareTemporalRuntimeStateChanged,
        this, &MainWindow::refreshHardwareTemporalRuntimeUi);
    connect(m_calibrationWidget, &CalibrationWidget::hardwareTemporalRuntimeCorrectionChanged,
        this, [this](bool enabled, double delayUs) {
            clearPointCloudForTemporalModeChange();
            LOG_INFO(LogCategory::Calibration,
                tr("[硬件同步时域补偿] 运行时状态切换为%1，τ=%2 µs；当前三维点云已清空，防止RAW/补偿点混合。")
                .arg(enabled ? tr("ON") : tr("OFF/RAW")).arg(delayUs, 0, 'f', 2));
            refreshHardwareTemporalRuntimeUi();
        });
    m_calibrationWidget->setSyncManager(m_syncManager.get());
    refreshHardwareTemporalRuntimeUi();
    m_calibrationWidget->setSingleSphereSafetyCheckers(
        [this](const JointPose5D& pose, QString& error) {
            return checkSphereStudyPoseSafety(pose, &error, false);
        },
        [this](const JointPose5D& from, const JointPose5D& to, QString& error) {
            return checkSphereStudyTransitionSafety(from, to, &error, false);
        },
        [this](mls::calibration::PosePlannerConfig& cfg, QString& source, QString& error) {
            mls::calibration::SphereStudyPathOptions limits;
            const bool intersectLive = m_zmc && m_zmc->isConnected();
            if (!resolveSphereStudyPlanningLimits(intersectLive, limits, &source, &error))
                return false;

            cfg.xMinMm = limits.xMinMm; cfg.xMaxMm = limits.xMaxMm;
            cfg.yMinMm = limits.yMinMm; cfg.yMaxMm = limits.yMaxMm;
            cfg.zMinMm = limits.zMinMm; cfg.zMaxMm = limits.zMaxMm;
            cfg.cMinDeg = std::max(cfg.cMinDeg, limits.cMinDeg);
            cfg.cMaxDeg = std::min(cfg.cMaxDeg, limits.cMaxDeg);
            cfg.bMinDeg = std::max(cfg.bMinDeg, limits.bMinDeg);
            cfg.bMaxDeg = std::min(cfg.bMaxDeg, limits.bMaxDeg);

            if (!(cfg.xMinMm < cfg.xMaxMm && cfg.yMinMm < cfg.yMaxMm &&
                cfg.zMinMm < cfg.zMaxMm && cfg.cMinDeg <= cfg.cMaxDeg &&
                cfg.bMinDeg <= cfg.bMaxDeg)) {
                error = tr("Machine5X规划限位与运动学标定界面B/C范围求交后为空。");
                return false;
            }
            source += intersectLive
                ? tr("；运动学标定在线规划同时使用当前控制器限位")
                : tr("；运动学标定离线规划不依赖控制器连接");
            error.clear();
            return true;
        });
    m_calibrationWidget->setMinimumWidth(520);
    connect(m_calibrationWidget, &CalibrationWidget::singleSphereMotionControlViewRequested,
        this, [controlTabs]() {
            // Keep one authoritative control surface for homing / B/C
            // absolute resync / manual servo enable.
            controlTabs->setCurrentIndex(1); // “控制”
        });

    // V27.6.16R: sensor calibration remains the first System Calibration page; timing now follows official profile-rate semantics
    // out of the Control tab and into the first System Calibration tab.
    if (QWidget* sensorCalibPage = m_calibrationWidget->sensorCalibrationPage()) {
        auto* sensorCalibLayout = new QVBoxLayout(sensorCalibPage);
        sensorCalibLayout->setContentsMargins(4, 4, 4, 4);
        sensorCalibLayout->addWidget(createVideoControlSection(sensorCalibPage));
        sensorCalibLayout->addStretch();
    }

    // 单球17参数标定不再创建第二份2D控件，统一复用主视图区“二维轮廓”。
    connect(m_calibrationWidget, &CalibrationWidget::singleSphereProfileViewRequested,
        this, [this]() {
            if (m_viewTabs && m_profile2DView) m_viewTabs->setCurrentWidget(m_profile2DView);
        });
    connect(m_calibrationWidget, &CalibrationWidget::singleSphereProfileOverlayRequested,
        this, [this](const QVector<QPointF>& points, double cx, double cz, double radius) {
            // Updating a 2-D overlay is data delivery, not a navigation request.
            // Do not steal the active tab here: Stage-2 simulation updates both
            // 2-D and 3-D every frame and the operator must be free to inspect either.
            if (m_profile2DView) m_profile2DView->setCalibrationOverlay(points, cx, cz, radius);
        });
    connect(m_calibrationWidget, &CalibrationWidget::singleSphereProfileOverlayClearRequested,
        this, [this]() {
            if (m_profile2DView) m_profile2DView->clearCalibrationOverlay();
        });

    connect(m_calibrationWidget, &CalibrationWidget::singleSphereRuntimeCalibrationUsableChanged,
        this, [this](bool usable) {
            syncValidatedCalibrationToRuntime(usable, false);
        });

    // V20 单球17参数/rank14标定复用 MainWindow 已有的轮廓预览和五轴运动安全入口。
    connect(m_calibrationWidget, &CalibrationWidget::singleSphereProfilePreviewStartRequested,
        this, &MainWindow::onStartSingleSphereCalibrationPreview);
    connect(m_calibrationWidget, &CalibrationWidget::singleSphereProfilePreviewStopRequested,
        this, &MainWindow::onStopProfilePreview);
    connect(m_calibrationWidget, &CalibrationWidget::singleSphereJointStopRequested,
        this, [this](bool emergency) {
            if (!m_zmc || !m_zmc->isConnected()) return;
            QString err;
            if (!m_zmc->stopJointMove5D(emergency, &err) && !err.isEmpty()) {
                statusBar()->showMessage(tr("单球标定停止五轴失败: %1").arg(err), 5000);
            }
        });
    connect(m_calibrationWidget, &CalibrationWidget::singleSphereMachineSafetySphereRequested,
        this, [this](double x, double y, double z, double radius, bool enabled) {
            if (!m_machineSafety) return;
            if (!enabled) {
                m_machineSafety->clearStudySphere();
                if (m_pointCloudView) m_pointCloudView->clearMachineStudySphere();
                return;
            }
            m_machineSafety->setStudySphere(x, y, z, radius);
            if (m_pointCloudView) {
                const double clearance = m_machineSafety->config().safety.studySphereClearanceMm;
                m_pointCloudView->setMachineStudySphere(x, y, z, radius, clearance);
            }
        });
    // V27.6.50 hardware-temporal staged workflow.  The widget owns the path
    // table and the mathematics; MainWindow remains the sole motion/acquisition
    // authority so the existing MachineSafety + V32.13 SINGLE-OP14 gates cannot
    // be bypassed by calibration code.
    connect(m_calibrationWidget, &CalibrationWidget::hardwareTemporalReferenceCloudViewRequested,
        this, [this](const QVector<QVector3D>& points) {
            if (m_pointCloudView) m_pointCloudView->setPoints(points);
            if (m_viewTabs && m_pointCloudView) m_viewTabs->setCurrentWidget(m_pointCloudView);
        });
    connect(m_calibrationWidget, &CalibrationWidget::hardwareTemporalReferenceSphereRequested,
        this, [this](double x, double y, double z, double radius, bool enabled) {
            if (!m_machineSafety) return;
            if (!enabled) {
                m_machineSafety->clearStudySphere();
                if (m_pointCloudView) m_pointCloudView->clearMachineStudySphere();
                return;
            }
            m_machineSafety->setStudySphere(x, y, z, radius);
            if (m_pointCloudView) {
                const double clearance = m_machineSafety->config().safety.studySphereClearanceMm;
                m_pointCloudView->setMachineStudySphere(x, y, z, radius, clearance);
            }
        });
    connect(m_calibrationWidget, &CalibrationWidget::hardwareTemporalPlannedScanRequested,
        this, [this](quint64 requestId, const JointPose5D& start,
            const JointPose5D& end, double feedMmPerSec) {
                beginHardwareTemporalPlannedScan(requestId, start, end, feedMmPerSec);
        });
    connect(m_calibrationWidget, &CalibrationWidget::hardwareTemporalAbortRequested,
        this, [this]() { abortHardwareTemporalPlannedScan(); });

    connect(m_calibrationWidget, &CalibrationWidget::singleSphereStage2SimulationFrameRequested,
        this, [this](const JointPose5D& pose,
            const QVector<QVector3D>& laserField,
            const QVector<QVector3D>& sectionCircle,
            const QVector<QVector3D>& visibleArc,
            const QVector<QVector3D>& accumulatedArc,
            bool feasible, const QString& statusText) {
                // GUI-only playback: never forward this pose to ZMC.
                updateMachineDigitalTwinPose(pose, true);
                if (m_pointCloudView) {
                    m_pointCloudView->setSphereStudyScanSimulation(
                        laserField, sectionCircle, visibleArc, feasible, statusText);
                    m_pointCloudView->setSphereStudySimulatedAcquisitionCloud(accumulatedArc);
                }
                statusBar()->showMessage(statusText, 2500);
        });
    connect(m_calibrationWidget, &CalibrationWidget::singleSphereStage2SimulationClearRequested,
        this, [this](bool clearAccumulatedCloud) {
            if (!m_pointCloudView) return;
            m_pointCloudView->clearSphereStudyScanSimulation();
            if (clearAccumulatedCloud)
                m_pointCloudView->clearSphereStudySimulatedAcquisitionCloud();
        });
    connect(m_calibrationWidget, &CalibrationWidget::singleSphereStage2SimulationViewRequested,
        this, [this]() {
            // Stage-2 playback defaults to the 3-D digital twin once.  Thereafter
            // no simulation frame changes the selected tab, so the user can move
            // freely between 3-D point cloud and 2-D profile during playback.
            if (m_viewTabs && m_pointCloudView)
                m_viewTabs->setCurrentWidget(m_pointCloudView);
        });
    connect(m_calibrationWidget, &CalibrationWidget::singleSphereJointMoveRequested,
        this, [this](const JointPose5D& target, double speedScale) {
            if (!m_zmc || !m_zmc->isConnected()) {
                JointState5D empty;
                m_calibrationWidget->notifySingleSphereJointMoveFinished(false, tr("控制器未连接"), empty);
                return;
            }
            if (m_jointMoveInProgress || (m_syncManager && m_syncManager->isRunning())
                || (m_gcodeExecutor && m_gcodeExecutor->isRunning())) {
                JointState5D empty;
                m_calibrationWidget->notifySingleSphereJointMoveFinished(
                    false, tr("已有运动/在线扫描/G代码任务正在执行"), empty);
                return;
            }
            if (!m_zmc->areLinearMposReferenced()) {
                JointState5D empty;
                m_calibrationWidget->notifySingleSphereJointMoveFinished(
                    false, tr("XYZ光栅MPOS尚未通过正规DATUM回零建立机床基准"), empty);
                return;
            }
            if (!m_zmc->isRotaryAbsoluteReady(AxisC) || !m_zmc->isRotaryAbsoluteReady(AxisB)
                || !m_zmc->isRotaryServoEnabled(AxisC) || !m_zmc->isRotaryServoEnabled(AxisB)) {
                JointState5D empty;
                m_calibrationWidget->notifySingleSphereJointMoveFinished(
                    false, tr("B/C 必须完成绝对位置同步并人工使能"), empty);
                return;
            }

            JointState5D state;
            QString err;
            if (!m_zmc->readJointState5D(state, &err) || !state.allIdle) {
                if (err.isEmpty()) err = tr("至少一个轴仍在运动");
                m_calibrationWidget->notifySingleSphereJointMoveFinished(false, err, state);
                return;
            }
            const double scale = std::clamp(speedScale, 0.01, 1.0);
            JointPose5D current = state.mpos;
            QString safetyError;
            if (!checkSphereStudyPoseSafety(target, &safetyError, false)) {
                m_calibrationWidget->notifySingleSphereJointMoveFinished(
                    false, tr("运动学标定目标姿态未通过MachineSafety：%1").arg(safetyError), state);
                return;
            }
            if (!checkSphereStudyTransitionSafety(current, target, &safetyError, false)) {
                m_calibrationWidget->notifySingleSphereJointMoveFinished(
                    false, tr("运动学标定整段扫掠路径未通过MachineSafety：%1").arg(safetyError), state);
                return;
            }
            JointMotionPlan5D plan = JointTrajectoryPlanner5Axis::planPointToPoint(
                current, target, currentAxisParams5D(), scale);
            if (!plan.valid) {
                m_calibrationWidget->notifySingleSphereJointMoveFinished(false, plan.error, state);
                return;
            }

            setJointMoveUiRunning(true);
            ++m_axisWorkerCount;
            ZmcController* controller = m_zmc.get();
            CalibrationWidget* calib = m_calibrationWidget;
            (void)QtConcurrent::run([this, controller, calib, plan]() {
                JointMoveResult5D result;
                QString moveErr;
                const bool ok = controller->moveJointAbs5D(plan, result, &moveErr);
                QMetaObject::invokeMethod(this, [this, calib, result, ok, moveErr]() {
                    --m_axisWorkerCount;
                    setJointMoveUiRunning(false);
                    pollAxisAlarmStates(false);
                    if (calib) calib->notifySingleSphereJointMoveFinished(
                        ok, ok ? result.message : moveErr, result.finalState);
                    tryFinishDeferredClose();
                    }, Qt::QueuedConnection);
                });
        });

    connect(m_calibrationWidget, &CalibrationWidget::singleSphereIndependentJointMoveRequested,
        this, [this](const JointPose5D& target, double xyzSpeedScale, double bcSpeedScale) {
            if (!m_zmc || !m_zmc->isConnected()) {
                JointState5D empty;
                m_calibrationWidget->notifySingleSphereJointMoveFinished(false, tr("控制器未连接"), empty);
                return;
            }
            if (m_jointMoveInProgress || (m_syncManager && m_syncManager->isRunning())
                || (m_gcodeExecutor && m_gcodeExecutor->isRunning())) {
                JointState5D empty;
                m_calibrationWidget->notifySingleSphereJointMoveFinished(
                    false, tr("已有运动/在线扫描/G代码任务正在执行"), empty);
                return;
            }
            if (!m_zmc->areLinearMposReferenced()) {
                JointState5D empty;
                m_calibrationWidget->notifySingleSphereJointMoveFinished(
                    false, tr("XYZ光栅MPOS尚未通过正规DATUM回零建立机床基准"), empty);
                return;
            }
            if (!m_zmc->isRotaryAbsoluteReady(AxisC) || !m_zmc->isRotaryAbsoluteReady(AxisB)
                || !m_zmc->isRotaryServoEnabled(AxisC) || !m_zmc->isRotaryServoEnabled(AxisB)) {
                JointState5D empty;
                m_calibrationWidget->notifySingleSphereJointMoveFinished(
                    false, tr("B/C 必须完成绝对位置同步并人工使能"), empty);
                return;
            }

            JointState5D state;
            QString err;
            if (!m_zmc->readJointState5D(state, &err) || !state.allIdle) {
                if (err.isEmpty()) err = tr("至少一个轴仍在运动");
                m_calibrationWidget->notifySingleSphereJointMoveFinished(false, err, state);
                return;
            }

            const double xyzScale = std::clamp(xyzSpeedScale, 0.01, 1.0);
            const double bcScale = std::clamp(bcSpeedScale, 0.01, 1.0);
            auto axisParams = currentAxisParams5D();
            // Stage-2 has its own transient motion scales.  Scale a local copy
            // instead of writing AxisParams through the global parameter page;
            // therefore changing calibration speed never invalidates XYZ DATUM
            // or the B/C absolute-position binding.
            for (int axis = AxisX; axis <= AxisZ; ++axis) {
                axisParams[axis].speed = static_cast<float>(axisParams[axis].speed * xyzScale);
                axisParams[axis].accel = static_cast<float>(axisParams[axis].accel * xyzScale);
                axisParams[axis].decel = static_cast<float>(axisParams[axis].decel * xyzScale);
            }
            for (const int axis : {AxisC, AxisB}) {
                axisParams[axis].speed = static_cast<float>(axisParams[axis].speed * bcScale);
                axisParams[axis].accel = static_cast<float>(axisParams[axis].accel * bcScale);
                axisParams[axis].decel = static_cast<float>(axisParams[axis].decel * bcScale);
            }
            constexpr double executionScale = 1.0;
            std::array<AxisParams, Constants::kAxisParamArraySize> safetyAxisParams = axisParams;
            const JointPose5D current = state.mpos;
            QString safetyError;
            if (!m_zmc->effectiveIndependentAxisParams5D(
                axisParams, executionScale, safetyAxisParams, &safetyError)) {
                m_calibrationWidget->notifySingleSphereJointMoveFinished(
                    false, tr("无法建立与实际补偿速度一致的碰撞预测参数：%1").arg(safetyError), state);
                return;
            }
            if (!checkSphereStudyPoseSafety(target, &safetyError, false)) {
                m_calibrationWidget->notifySingleSphereJointMoveFinished(
                    false, tr("运动学标定目标姿态未通过MachineSafety：%1").arg(safetyError), state);
                return;
            }
            if (!m_machineSafety) {
                m_calibrationWidget->notifySingleSphereJointMoveFinished(
                    false, tr("MachineSafety模型不可用，拒绝执行非插补独立五轴移动"), state);
                return;
            }
            const bool compensationOn = m_zmc->axisCompensationEnabled();
            if (compensationOn) {
                double envelopeMm = 0.0;
                if (!m_zmc->axisCompensationSafetyEnvelopeMm(envelopeMm, &safetyError)) {
                    m_calibrationWidget->notifySingleSphereJointMoveFinished(
                        false, tr("无法取得XYZ补偿安全包络：%1").arg(safetyError), state);
                    return;
                }
                const double clearanceMm = std::max(0.0, m_machineSafety->config().safety.minimumStaticClearanceMm);
                if (!(clearanceMm > envelopeMm + 1.0e-6)) {
                    m_calibrationWidget->notifySingleSphereJointMoveFinished(
                        false, tr("MachineSafety静态间隙%1 mm不足以覆盖XYZ补偿路径包络%2 mm")
                        .arg(clearanceMm, 0, 'f', 6).arg(envelopeMm, 0, 'f', 6), state);
                    return;
                }
            }
            const auto transition = m_machineSafety->checkIndependentTransition(
                current, target, safetyAxisParams, executionScale, compensationOn);
            if (!transition.safe) {
                if (m_pointCloudView && transition.firstUnsafeSample >= 0) {
                    const auto& q = transition.firstUnsafePose;
                    m_pointCloudView->setMachineJointPose(q.x, q.y, q.z, q.c, q.b);
                    QStringList links;
                    for (const auto& name : transition.unsafePoseResult.collisionLinks)
                        links << QString::fromStdString(name);
                    m_pointCloudView->setMachineCollisionLinks(links);
                    m_pointCloudView->setMachineSafetyStatus(
                        tr("独立轴过渡路径碰撞：t=%1，%2")
                        .arg(transition.firstUnsafeT, 0, 'f', 3)
                        .arg(QString::fromStdString(transition.unsafePoseResult.message)), true);
                }
                m_calibrationWidget->notifySingleSphereJointMoveFinished(
                    false,
                    tr("运动学标定独立轴实际过渡路径未通过MachineSafety：%1")
                    .arg(QString::fromStdString(transition.message)),
                    state);
                return;
            }

            setJointMoveUiRunning(true);
            ++m_axisWorkerCount;
            ZmcController* controller = m_zmc.get();
            CalibrationWidget* calib = m_calibrationWidget;
            (void)QtConcurrent::run([this, controller, calib, target, axisParams]() {
                JointMoveResult5D result;
                QString moveErr;
                const bool ok = controller->moveJointAbs5DIndependent(
                    target, axisParams, 1.0, result, &moveErr);
                QMetaObject::invokeMethod(this, [this, calib, result, ok, moveErr]() {
                    --m_axisWorkerCount;
                    setJointMoveUiRunning(false);
                    pollAxisAlarmStates(false);
                    if (calib) calib->notifySingleSphereJointMoveFinished(
                        ok, ok ? result.message : moveErr, result.finalState);
                    tryFinishDeferredClose();
                    }, Qt::QueuedConnection);
                });
        });

    calibScrollArea->setWidget(m_calibrationWidget);
    controlTabs->addTab(calibScrollArea, tr("系统标定"));

    // Tab 4: 五轴运动学（P1/P2/P3 基础验证，不发送联动运动）
    auto* kinScrollArea = new QScrollArea(controlTabs);
    kinScrollArea->setWidgetResizable(true);
    kinScrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    kinScrollArea->setWidget(createKinematicsSection(kinScrollArea));
    controlTabs->addTab(kinScrollArea, tr("五轴运动学"));

    // Tab 5: 路径规划
    auto* pathScrollArea = new QScrollArea(controlTabs);
    pathScrollArea->setWidgetResizable(true);
    pathScrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    m_pathPlanningWidget = new PathPlanningWidget(pathScrollArea);
    m_pathPlanningWidget->setPointCloudBuilder(m_cloudBuilder.get());
    m_pathPlanningWidget->setMainPointCloudView(m_pointCloudView);
    pathScrollArea->setWidget(m_pathPlanningWidget);
    controlTabs->addTab(pathScrollArea, tr("路径规划"));

    // Multi-pose experiment is opened lazily from PathPlanningWidget,
    // directly above the existing M0-M8 digital twin launch button.
    // This avoids initializing experimental VTK/Qt components during startup.

    // Tab 6: 点云处理
    auto* pcProcessScrollArea = new QScrollArea(controlTabs);
    pcProcessScrollArea->setWidgetResizable(true);
    pcProcessScrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    m_pointCloudProcessWidget = new PointCloudProcessWidget(pcProcessScrollArea);
    m_pointCloudProcessWidget->setPointCloudBuilder(m_cloudBuilder.get());
    pcProcessScrollArea->setWidget(m_pointCloudProcessWidget);
    controlTabs->addTab(pcProcessScrollArea, tr("点云处理"));

    layout->addWidget(controlTabs, 1);  // 让Tab占据剩余空间

    return panel;
}

QWidget* MainWindow::createConnectionSection(QWidget* parent)
{
    auto* group = new QGroupBox(tr("设备连接"), parent);
    auto* layout = new QVBoxLayout(group);
    layout->setSpacing(8);

    // 控制器连接行
    auto* ctrlLayout = new QHBoxLayout;
    ctrlLayout->addWidget(new QLabel(tr("控制器:"), group));

    m_controllerIpEdit = new QLineEdit("192.168.0.11", group);
    m_controllerIpEdit->setFixedWidth(120);
    ctrlLayout->addWidget(m_controllerIpEdit);

    m_btnControllerConnect = new QPushButton(tr("连接"), group);
    m_btnControllerConnect->setFixedWidth(60);
    ctrlLayout->addWidget(m_btnControllerConnect);

    m_btnControllerDisconnect = new QPushButton(tr("断开"), group);
    m_btnControllerDisconnect->setFixedWidth(60);
    ctrlLayout->addWidget(m_btnControllerDisconnect);

    m_lblControllerStatus = new QLabel(tr("未连接"), group);
    m_lblControllerStatus->setStyleSheet("QLabel { color: red; font-weight: bold; }");
    m_lblControllerStatus->setFixedWidth(60);
    ctrlLayout->addWidget(m_lblControllerStatus);

    ctrlLayout->addStretch();
    layout->addLayout(ctrlLayout);

    // 传感器连接行
    auto* sensorLayout = new QHBoxLayout;
    sensorLayout->addWidget(new QLabel(tr("传感器:"), group));

    m_sensorIpEdit = new QLineEdit("192.168.1.200", group);
    m_sensorIpEdit->setFixedWidth(120);
    m_sensorIpEdit->setToolTip(tr("传感器 IP 地址（自动搜索，此处仅显示）"));
    m_sensorIpEdit->setReadOnly(true);
    sensorLayout->addWidget(m_sensorIpEdit);

    m_btnSensorConnect = new QPushButton(tr("连接"), group);
    m_btnSensorConnect->setFixedWidth(60);
    sensorLayout->addWidget(m_btnSensorConnect);

    m_btnSensorDisconnect = new QPushButton(tr("断开"), group);
    m_btnSensorDisconnect->setFixedWidth(60);
    sensorLayout->addWidget(m_btnSensorDisconnect);

    m_lblSensorStatus = new QLabel(tr("未连接"), group);
    m_lblSensorStatus->setStyleSheet("QLabel { color: red; font-weight: bold; }");
    m_lblSensorStatus->setFixedWidth(60);
    sensorLayout->addWidget(m_lblSensorStatus);

    sensorLayout->addStretch();
    layout->addLayout(sensorLayout);

    // 连接信号
    connect(m_btnControllerConnect, &QPushButton::clicked, this, &MainWindow::onControllerConnect);
    connect(m_btnControllerDisconnect, &QPushButton::clicked, this, &MainWindow::onControllerDisconnect);
    connect(m_btnSensorConnect, &QPushButton::clicked, this, &MainWindow::onSensorConnect);
    connect(m_btnSensorDisconnect, &QPushButton::clicked, this, &MainWindow::onSensorDisconnect);

    return group;
}

QWidget* MainWindow::createKinematicsSection(QWidget* parent)
{
    auto* widget = new QWidget(parent);
    auto* layout = new QVBoxLayout(widget);
    layout->setContentsMargins(5, 5, 5, 5);
    layout->setSpacing(8);

    auto* modelGroup = new QGroupBox(tr("当前五轴名义模型"), widget);
    auto* modelLayout = new QVBoxLayout(modelGroup);
    auto* modelLabel = new QLabel(
        tr("运动链: X→Y→Z→C→B\n"
            "C绕机床Z；C=0时B绕机床Y；B/C轴线相交\n"
            "控制器正方向: XYZ=数学正；C/B=数学右手负\n"
            "T = Trans(X,Y,Z) · Rz(-C) · Ry(-B)\n"
            "末端原点暂定义在B/C轴线交点（尚未加入传感器外参）"), modelGroup);
    modelLabel->setWordWrap(true);
    modelLayout->addWidget(modelLabel);
    layout->addWidget(modelGroup);

    auto* inputGroup = new QGroupBox(tr("关节坐标输入"), widget);
    auto* grid = new QGridLayout(inputGroup);

    auto makeSpin = [inputGroup](double minV, double maxV, const QString& suffix) {
        auto* s = new QDoubleSpinBox(inputGroup);
        s->setRange(minV, maxV);
        s->setDecimals(6);
        s->setSingleStep(suffix.contains(QChar(0x00B0)) ? 1.0 : 0.1);
        s->setSuffix(suffix);
        return s;
        };

    m_kinX = makeSpin(-1000.0, 1000.0, tr(" mm"));
    m_kinY = makeSpin(-1000.0, 1000.0, tr(" mm"));
    m_kinZ = makeSpin(-1000.0, 1000.0, tr(" mm"));
    m_kinC = makeSpin(-90.0, 90.0, QString::fromUtf8(" °"));
    m_kinB = makeSpin(-90.0, 90.0, QString::fromUtf8(" °"));

    grid->addWidget(new QLabel("X", inputGroup), 0, 0); grid->addWidget(m_kinX, 0, 1);
    grid->addWidget(new QLabel("Y", inputGroup), 1, 0); grid->addWidget(m_kinY, 1, 1);
    grid->addWidget(new QLabel("Z", inputGroup), 2, 0); grid->addWidget(m_kinZ, 2, 1);
    grid->addWidget(new QLabel("C", inputGroup), 3, 0); grid->addWidget(m_kinC, 3, 1);
    grid->addWidget(new QLabel("B", inputGroup), 4, 0); grid->addWidget(m_kinB, 4, 1);

    auto* btnRow = new QHBoxLayout;
    auto* btnCurrent = new QPushButton(tr("读取当前五轴"), inputGroup);
    auto* btnFk = new QPushButton(tr("正解 FK"), inputGroup);
    auto* btnRoundTrip = new QPushButton(tr("FK→IK 自检"), inputGroup);
    btnRow->addWidget(btnCurrent);
    btnRow->addWidget(btnFk);
    btnRow->addWidget(btnRoundTrip);
    grid->addLayout(btnRow, 5, 0, 1, 2);

    connect(btnCurrent, &QPushButton::clicked, this, &MainWindow::onKinematicsLoadCurrent);
    connect(btnFk, &QPushButton::clicked, this, &MainWindow::onKinematicsForward);
    connect(btnRoundTrip, &QPushButton::clicked, this, &MainWindow::onKinematicsRoundTrip);

    layout->addWidget(inputGroup);

    auto* moveGroup = new QGroupBox(tr("P4 五轴关节空间同步定位"), widget);
    auto* moveGrid = new QGridLayout(moveGroup);

    auto* warning = new QLabel(
        tr("该功能执行 XYZCB 关节空间同步定位，不是 TCP 恒速直线插补。\n"
            "B/C 必须已完成绝对位置同步并人工使能；扫描/G代码运行时禁止启动。"),
        moveGroup);
    warning->setWordWrap(true);
    moveGrid->addWidget(warning, 0, 0, 1, 4);

    moveGrid->addWidget(new QLabel(tr("同步速度倍率:"), moveGroup), 1, 0);
    m_kinSpeedScale = new QDoubleSpinBox(moveGroup);
    m_kinSpeedScale->setRange(1.0, 100.0);
    m_kinSpeedScale->setDecimals(1);
    m_kinSpeedScale->setSingleStep(5.0);
    m_kinSpeedScale->setValue(20.0);
    m_kinSpeedScale->setSuffix(" %");
    m_kinSpeedScale->setToolTip(tr("按五个轴各自 SPEED/ACCEL/DECEL 的百分比进行保守同步规划；不是 mm/s TCP 进给"));
    moveGrid->addWidget(m_kinSpeedScale, 1, 1);

    m_btnKinPlan = new QPushButton(tr("规划五轴定位"), moveGroup);
    m_btnKinMove = new QPushButton(tr("执行五轴同步定位"), moveGroup);
    m_btnKinStop = new QPushButton(tr("停止五轴"), moveGroup);
    m_btnKinStop->setStyleSheet("QPushButton { background-color: #ff6666; font-weight: bold; }");
    m_btnKinStop->setEnabled(false);
    moveGrid->addWidget(m_btnKinPlan, 2, 0);
    moveGrid->addWidget(m_btnKinMove, 2, 1);
    moveGrid->addWidget(m_btnKinStop, 2, 2);

    m_kinPlanSummary = new QLabel(tr("尚未规划"), moveGroup);
    m_kinPlanSummary->setWordWrap(true);
    moveGrid->addWidget(m_kinPlanSummary, 3, 0, 1, 4);

    connect(m_btnKinPlan, &QPushButton::clicked, this, &MainWindow::onKinematicsPlanJointMove);
    connect(m_btnKinMove, &QPushButton::clicked, this, &MainWindow::onKinematicsExecuteJointMove);
    connect(m_btnKinStop, &QPushButton::clicked, this, &MainWindow::onKinematicsStopJointMove);

    layout->addWidget(moveGroup);

    auto* outputGroup = new QGroupBox(tr("运动学 / P4 结果"), widget);
    auto* outputLayout = new QVBoxLayout(outputGroup);
    m_kinOutput = new QTextEdit(outputGroup);
    m_kinOutput->setReadOnly(true);
    m_kinOutput->setMinimumHeight(260);
    m_kinOutput->setPlainText(tr("输入 XYZCB 可执行 FK/IK 验证；P4 可规划并执行一次五轴关节空间同步绝对定位。"));
    outputLayout->addWidget(m_kinOutput);
    layout->addWidget(outputGroup);
    layout->addStretch();
    return widget;
}

QWidget* MainWindow::createParametersSection(QWidget* parent)
{
    auto* widget = new QWidget(parent);
    auto* layout = new QVBoxLayout(widget);
    layout->setContentsMargins(5, 5, 5, 5);
    layout->setSpacing(8);

    // ========== 机床参数 ==========
    auto* machineGroup = new QGroupBox(tr("机床参数"), widget);
    auto* machineLayout = new QVBoxLayout(machineGroup);

    // 轴参数表格
    m_axisTable = new QTableWidget(5, 10, machineGroup);
    QStringList headers;
    headers << "轴" << "脉冲当量" << "起始速度" << "运行速度"
        << "加速度" << "减速度" << "S曲线"
        << "爬行速度" << "正限位" << "负限位";
    m_axisTable->setHorizontalHeaderLabels(headers);
    m_axisTable->verticalHeader()->setVisible(false);
    m_axisTable->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    m_axisTable->setMaximumHeight(185);
    m_axisTable->setEditTriggers(QAbstractItemView::DoubleClicked | QAbstractItemView::SelectedClicked);

    const QStringList axisNames{ "X", "Y", "Z", "C", "B" };
    for (int row = 0; row < axisNames.size(); ++row) {
        auto* itemAxis = new QTableWidgetItem;
        itemAxis->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEnabled);
        itemAxis->setText(axisNames[row]);
        m_axisTable->setItem(row, 0, itemAxis);
        for (int col = 1; col < headers.size(); ++col) {
            m_axisTable->setItem(row, col, new QTableWidgetItem);
        }
    }
    machineLayout->addWidget(m_axisTable);

    // 参数按钮行
    auto* paramBtnLayout = new QHBoxLayout;
    m_btnReadParams = new QPushButton(tr("读取"), machineGroup);
    m_btnWriteAxis = new QPushButton(tr("写入选中轴"), machineGroup);
    m_btnWriteAll = new QPushButton(tr("写入全部"), machineGroup);
    paramBtnLayout->addWidget(m_btnReadParams);
    paramBtnLayout->addWidget(m_btnWriteAxis);
    paramBtnLayout->addWidget(m_btnWriteAll);
    paramBtnLayout->addStretch();
    machineLayout->addLayout(paramBtnLayout);

    connect(m_btnReadParams, &QPushButton::clicked, this, &MainWindow::onReadAxisParams);
    connect(m_btnWriteAxis, &QPushButton::clicked, this, &MainWindow::onWriteAxisParams);
    connect(m_btnWriteAll, &QPushButton::clicked, this, &MainWindow::onWriteAllAxisParams);

    layout->addWidget(machineGroup);

    // ========== 传感器参数 ==========
    auto* sensorGroup = new QGroupBox(tr("传感器参数"), widget);
    auto* sensorLayout = new QVBoxLayout(sensorGroup);
    sensorLayout->setSpacing(6);

    // ----- 第1行: 测量区域、分辨率 -----
    auto* sensorRow1 = new QHBoxLayout;
    sensorRow1->addWidget(new QLabel(tr("测量区域:"), sensorGroup));
    m_comboMeasuringField = new QComboBox(sensorGroup);
    m_comboMeasuringField->setMinimumWidth(180);
    m_comboMeasuringField->addItem(tr("Large / 全测量场预设 (0) - 参考144Hz"), 0);
    m_comboMeasuringField->addItem(tr("Standard (2) - 192Hz"), 2);
    m_comboMeasuringField->addItem(tr("Small (7) - 552Hz"), 7);
    m_comboMeasuringField->addItem(tr("快速1 (32) - 241Hz"), 32);
    m_comboMeasuringField->addItem(tr("快速2 (64) - 362Hz"), 64);
    m_comboMeasuringField->addItem(tr("快速3 (96) - 552Hz"), 96);
    m_comboMeasuringField->addItem(tr("高速 (122) - 2000Hz"), 122);
    m_comboMeasuringField->setCurrentIndex(2);  // 默认 Small (7)
    m_comboMeasuringField->setToolTip(tr("测量区域大小决定最大采样频率，较小区域=更高频率"));
    sensorRow1->addWidget(m_comboMeasuringField);

    sensorRow1->addSpacing(20);
    sensorRow1->addWidget(new QLabel(tr("分辨率:"), sensorGroup));
    m_comboResolution = new QComboBox(sensorGroup);
    m_comboResolution->setMinimumWidth(80);
    sensorRow1->addWidget(m_comboResolution);
    sensorRow1->addStretch();
    sensorLayout->addLayout(sensorRow1);

    // ----- V27.6.43: 材料曝光方案 -----
    auto* sensorMaterialRow = new QHBoxLayout;
    sensorMaterialRow->addWidget(new QLabel(tr("材料曝光方案:"), sensorGroup));
    m_comboMaterialExposureProfile = new QComboBox(sensorGroup);
    m_comboMaterialExposureProfile->setMinimumWidth(300);
    m_comboMaterialExposureProfile->addItem(tr("手动参数（不套用材料库）"), QString());
    m_comboMaterialExposureProfile->setToolTip(tr(
        "材料曝光库。选择条目后自动填入推荐Exposure/Idle；"
        "软件内部触发按当前标定目标频率自动计算Idle；硬件同步按当前硬件触发频率决定时序，绝不直接复制慢速标定Idle。"));
    sensorMaterialRow->addWidget(m_comboMaterialExposureProfile, 1);
    m_btnReloadMaterialExposureLibrary = new QPushButton(tr("刷新"), sensorGroup);
    m_btnReloadMaterialExposureLibrary->setFixedWidth(52);
    sensorMaterialRow->addWidget(m_btnReloadMaterialExposureLibrary);
    sensorLayout->addLayout(sensorMaterialRow);

    connect(m_comboMaterialExposureProfile, QOverload<int>::of(&QComboBox::currentIndexChanged),
        this, &MainWindow::onMaterialExposureProfileChanged);
    connect(m_btnReloadMaterialExposureLibrary, &QPushButton::clicked,
        this, &MainWindow::onReloadMaterialExposureLibrary);

    // ----- 第2行: 曝光、空闲时间 -----
    auto* sensorRow2 = new QHBoxLayout;
    sensorRow2->addWidget(new QLabel(tr("曝光(10µs):"), sensorGroup));
    m_spinExposure = new QSpinBox(sensorGroup);
    m_spinExposure->setRange(1, 100000);
    m_spinExposure->setValue(100);
    m_spinExposure->setFixedWidth(80);
    sensorRow2->addWidget(m_spinExposure);

    sensorRow2->addSpacing(20);
    sensorRow2->addWidget(new QLabel(tr("空闲(10µs):"), sensorGroup));
    m_spinIdle = new QSpinBox(sensorGroup);
    m_spinIdle->setRange(0, 100000);
    m_spinIdle->setValue(300);  // 默认300 (= 3ms)
    m_spinIdle->setFixedWidth(80);
    sensorRow2->addWidget(m_spinIdle);
    sensorRow2->addStretch();
    sensorLayout->addLayout(sensorRow2);

    // The measuring field and H1/H3 timing remain here.  Independent Video/Profile
    // preview timing (rate, exposure and derived Idle) is edited directly on the
    // Sensor Calibration page; it must not mutate formal machine sync settings.

    // ----- 第3行: 同步模式 -----
    auto* sensorRow3 = new QHBoxLayout;
    sensorRow3->addWidget(new QLabel(tr("同步模式:"), sensorGroup));

    m_radioSoftwareSync = new QRadioButton(tr("软件"), sensorGroup);
    m_radioHardwareSync = new QRadioButton(tr("硬件（H1/H3）"), sensorGroup);
    // V32.13 keeps hardware execution fail-closed until the current controller and
    // current B/C absolute software frame pass the read-only preflight AND the
    // downloaded BASIC explicitly publishes TABLE[342]=1.
    m_radioSoftwareSync->setChecked(true);
    m_radioHardwareSync->setEnabled(false);
    m_radioHardwareSync->setToolTip(tr("控制器连接且B/C绝对机械坐标建立成功后，软件会自动执行硬件同步只读预检；通过时静默解锁硬件模式，失败时才弹窗提示。仅接受V32.13 SINGLE-OP14 FULL-MPOS ColdPrime BASIC：XYZ/C/B五轴均由REGIST(104)保存MPOS；XYZ增量光栅必须先完成回零并建立MPOS机床零点。上位机实际位置/软件同步/标定均使用回零后的XYZ光栅MPOS；DPOS仅保留为命令轨迹和跟随误差诊断。正式采集前通过同一OP14自动预热传感器数据链。"));

    auto* syncGroup = new QButtonGroup(sensorGroup);
    syncGroup->addButton(m_radioSoftwareSync);
    syncGroup->addButton(m_radioHardwareSync);
    connect(m_radioHardwareSync, &QRadioButton::toggled, this, [this](bool) {
        if (m_comboMaterialExposureProfile && m_comboMaterialExposureProfile->currentIndex() > 0) {
            applyMaterialExposureProfileByKey(m_comboMaterialExposureProfile->currentData().toString());
        }
        refreshHardwareTemporalRuntimeUi();
        });

    sensorRow3->addWidget(m_radioSoftwareSync);
    sensorRow3->addWidget(m_radioHardwareSync);
    sensorRow3->addStretch();
    sensorLayout->addLayout(sensorRow3);

    // Only H1 and H3 are supported. Topology ID 2 is retired (not re-numbered).
    // This selection is committed to ECI
    // with Hz under one monotonically increasing request sequence.
    auto* topologyRow = new QHBoxLayout;
    topologyRow->addWidget(new QLabel(tr("硬件拓扑:"), sensorGroup));
    m_comboHardwareTopology = new QComboBox(sensorGroup);
    m_comboHardwareTopology->addItem(tr("H1 ECI PWM → Sensor + IN0"),1);
    m_comboHardwareTopology->addItem(tr("H3 Sensor Internal → CMM → IN1"),3);
    m_comboHardwareTopology->setCurrentIndex(0);
    topologyRow->addWidget(m_comboHardwareTopology,1);
    topologyRow->addWidget(new QLabel(tr("IN1锁存沿:"),sensorGroup));
    m_comboCmmR1Edge=new QComboBox(sensorGroup);
    m_comboCmmR1Edge->addItem(tr("R1内部上升沿(114)"),114);
    m_comboCmmR1Edge->addItem(tr("R1内部下降沿(115)"),115);
    m_comboCmmR1Edge->setToolTip(tr("NPN转换器与ECI IN1可能反相。需使用示波器确认；两种边沿不能同时开启。"));
    m_comboCmmR1Edge->setEnabled(false);
    topologyRow->addWidget(m_comboCmmR1Edge);
    connect(m_comboHardwareTopology,QOverload<int>::of(&QComboBox::currentIndexChanged),this,[this](int){
        const bool cmm=m_comboHardwareTopology->currentData().toInt()==3;
        m_comboCmmR1Edge->setEnabled(cmm && !m_syncManager->isRunning());
        if(m_comboTriggerType)m_comboTriggerType->setCurrentIndex(cmm?1:0);
        refreshHardwareTemporalRuntimeUi();
    });
    sensorLayout->addLayout(topologyRow);

    // ----- 第4行: 触发类型、硬件频率 -----
    auto* sensorRow3b = new QHBoxLayout;
    sensorRow3b->addWidget(new QLabel(tr("传感器触发方式:"), sensorGroup));
    m_comboTriggerType = new QComboBox(sensorGroup);
    m_comboTriggerType->addItem(tr("H1 外部上升沿"), QStringLiteral("HardwareExternal"));
    m_comboTriggerType->addItem(tr("H3 内部周期"), QStringLiteral("SoftwareInternal"));
    m_comboTriggerType->setCurrentIndex(0);
    m_comboTriggerType->setEnabled(false);
    m_comboTriggerType->setMinimumWidth(152);
    m_comboTriggerType->setToolTip(tr("此项由硬件拓扑自动决定：H1外部上升沿、IN0/R0/104；H3内部周期、IN1/R1/114或115。与位置锁存沿是两个不同参数。"));
    sensorRow3b->addWidget(m_comboTriggerType);

    sensorRow3b->addSpacing(20);
    sensorRow3b->addWidget(new QLabel(tr("硬件频率:"), sensorGroup));
    m_spinHardwareSyncFrequency = new QSpinBox(sensorGroup);
    m_spinHardwareSyncFrequency->setRange(mls::runtime::Table5D::minTriggerFrequencyHz,
        LaserSensor::getMaxFrequencyForIndex(7));
    m_spinHardwareSyncFrequency->setValue(mls::runtime::Table5D::defaultTriggerFrequencyHz);
    m_spinHardwareSyncFrequency->setSuffix(tr(" Hz"));
    m_spinHardwareSyncFrequency->setFixedWidth(105);
    sensorRow3b->addWidget(m_spinHardwareSyncFrequency);

    connect(m_comboMeasuringField, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int) {
        updateHardwareSyncFrequencyLimit();
        if (m_comboMaterialExposureProfile && m_comboMaterialExposureProfile->currentIndex() > 0)
            applyMaterialExposureProfileByKey(m_comboMaterialExposureProfile->currentData().toString());
        refreshHardwareTemporalRuntimeUi();
        });
    connect(m_spinHardwareSyncFrequency, QOverload<int>::of(&QSpinBox::valueChanged), this, [this](int) {
        updateHardwareSyncFrequencyLimit();
        if (m_comboMaterialExposureProfile && m_comboMaterialExposureProfile->currentIndex() > 0)
            applyMaterialExposureProfileByKey(m_comboMaterialExposureProfile->currentData().toString());
        refreshHardwareTemporalRuntimeUi();
        });
    updateHardwareSyncFrequencyLimit();

    sensorRow3b->addStretch();
    sensorLayout->addLayout(sensorRow3b);

    // ----- V27.6.53: 硬件同步时域补偿（活动标定显示 + 运行时开关） -----
    auto* temporalGroup = new QGroupBox(tr("硬件同步时域补偿"), sensorGroup);
    auto* temporalLayout = new QGridLayout(temporalGroup);
    temporalLayout->setContentsMargins(8, 5, 8, 5);
    temporalLayout->setHorizontalSpacing(8);
    temporalLayout->setVerticalSpacing(3);
    m_checkHardwareTemporalRuntime = new QCheckBox(tr("启用正式硬件同步时域补偿"), temporalGroup);
    m_checkHardwareTemporalRuntime->setEnabled(false);
    m_checkHardwareTemporalRuntime->setToolTip(tr(
        "ON：正式硬件同步扫描使用q(t+τ)后再进入Physical17；OFF：使用原始同ordinal REGIST(104)位姿。"
        "采集运行/Stop-Drain期间禁止切换。切换成功后会清空当前点云，防止RAW与补偿点混合显示。"));
    m_lblHardwareTemporalStatus = new QLabel(tr("活动标定：未加载"), temporalGroup);
    m_lblHardwareTemporalValue = new QLabel(tr("τ=--- µs    σ=--- µs"), temporalGroup);
    m_lblHardwareTemporalConfig = new QLabel(tr("标定配置：---"), temporalGroup);
    m_lblHardwareTemporalStatus->setWordWrap(true);
    m_lblHardwareTemporalValue->setTextInteractionFlags(Qt::TextSelectableByMouse);
    m_lblHardwareTemporalConfig->setWordWrap(true);
    temporalLayout->addWidget(m_checkHardwareTemporalRuntime, 0, 0, 1, 2);
    temporalLayout->addWidget(m_lblHardwareTemporalStatus, 1, 0, 1, 2);
    temporalLayout->addWidget(m_lblHardwareTemporalValue, 2, 0, 1, 2);
    temporalLayout->addWidget(m_lblHardwareTemporalConfig, 3, 0, 1, 2);
    sensorLayout->addWidget(temporalGroup);
    connect(m_checkHardwareTemporalRuntime, &QCheckBox::toggled,
        this, &MainWindow::onHardwareTemporalRuntimeToggled);

    // Current sensor timing edits immediately update the configuration-match hint.
    connect(m_comboResolution, QOverload<int>::of(&QComboBox::currentIndexChanged),
        this, [this](int) { refreshHardwareTemporalRuntimeUi(); });
    connect(m_spinExposure, QOverload<int>::of(&QSpinBox::valueChanged),
        this, [this](int) { refreshHardwareTemporalRuntimeUi(); });
    connect(m_spinIdle, QOverload<int>::of(&QSpinBox::valueChanged),
        this, [this](int) { refreshHardwareTemporalRuntimeUi(); });

    // ----- 第5行: 激光功率、反射选择、阈值 -----
    auto* sensorRow4 = new QHBoxLayout;
    sensorRow4->addWidget(new QLabel(tr("激光功率:"), sensorGroup));
    m_comboLaserPower = new QComboBox(sensorGroup);
    m_comboLaserPower->addItem(tr("关闭"), 0);
    m_comboLaserPower->addItem(tr("降低"), 1);
    m_comboLaserPower->addItem(tr("标准"), 2);
    m_comboLaserPower->setCurrentIndex(2);
    sensorRow4->addWidget(m_comboLaserPower);

    sensorRow4->addSpacing(15);
    sensorRow4->addWidget(new QLabel(tr("反射选择:"), sensorGroup));
    m_comboReflection = new QComboBox(sensorGroup);
    m_comboReflection->addItem(tr("全部"), 0);
    m_comboReflection->addItem(tr("第一个"), 1);
    m_comboReflection->addItem(tr("最后一个"), 2);
    m_comboReflection->addItem(tr("最大面积"), 3);
    m_comboReflection->addItem(tr("最大强度"), 4);
    m_comboReflection->setCurrentIndex(3);
    sensorRow4->addWidget(m_comboReflection);

    sensorRow4->addSpacing(15);
    sensorRow4->addWidget(new QLabel(tr("阈值[%]:"), sensorGroup));
    m_spinThreshold = new QDoubleSpinBox(sensorGroup);
    m_spinThreshold->setRange(0.0, 100.0);
    m_spinThreshold->setDecimals(1);
    m_spinThreshold->setValue(12.5);
    m_spinThreshold->setFixedWidth(65);
    sensorRow4->addWidget(m_spinThreshold);
    sensorRow4->addStretch();
    sensorLayout->addLayout(sensorRow4);

    // ----- 第6行: 翻转、处理选项 -----
    auto* sensorRow5 = new QHBoxLayout;
    m_checkFlipX = new QCheckBox(tr("X轴翻转"), sensorGroup);
    sensorRow5->addWidget(m_checkFlipX);

    m_checkFlipZ = new QCheckBox(tr("Z轴翻转"), sensorGroup);
    m_checkFlipZ->setChecked(true);
    sensorRow5->addWidget(m_checkFlipZ);

    sensorRow5->addSpacing(15);
    m_checkHighResolution = new QCheckBox(tr("高分辨率"), sensorGroup);
    m_checkHighResolution->setChecked(true);
    sensorRow5->addWidget(m_checkHighResolution);

    m_checkCalibration = new QCheckBox(tr("校准"), sensorGroup);
    m_checkCalibration->setChecked(true);
    sensorRow5->addWidget(m_checkCalibration);

    m_checkAmbientLight = new QCheckBox(tr("抑制背光"), sensorGroup);
    m_checkAmbientLight->setChecked(true);
    sensorRow5->addWidget(m_checkAmbientLight);

    m_checkVideoFilter = new QCheckBox(tr("视频滤波"), sensorGroup);
    m_checkVideoFilter->setChecked(true);
    sensorRow5->addWidget(m_checkVideoFilter);
    sensorRow5->addStretch();
    sensorLayout->addLayout(sensorRow5);

    // ----- 第6行: 自动曝光 -----
    auto* sensorRow6 = new QHBoxLayout;
    m_checkAutoExposure = new QCheckBox(tr("自动曝光"), sensorGroup);
    m_checkAutoExposure->setChecked(false);
    sensorRow6->addWidget(m_checkAutoExposure);

    sensorRow6->addSpacing(10);
    sensorRow6->addWidget(new QLabel(tr("快门对齐:"), sensorGroup));
    m_comboShutterAlign = new QComboBox(sensorGroup);
    m_comboShutterAlign->addItem(tr("中"), 0);
    m_comboShutterAlign->addItem(tr("右"), 1);
    m_comboShutterAlign->addItem(tr("左"), 2);
    m_comboShutterAlign->addItem(tr("关闭"), 3);
    m_comboShutterAlign->setCurrentIndex(1);
    m_comboShutterAlign->setEnabled(false);
    sensorRow6->addWidget(m_comboShutterAlign);

    sensorRow6->addSpacing(10);
    sensorRow6->addWidget(new QLabel(tr("模式:"), sensorGroup));
    m_comboAutoExpMode = new QComboBox(sensorGroup);
    m_comboAutoExpMode->addItem(tr("原始数据"), 0);
    m_comboAutoExpMode->addItem(tr("轮廓"), 1);
    m_comboAutoExpMode->addItem(tr("过滤的轮廓"), 2);
    m_comboAutoExpMode->setCurrentIndex(1);
    m_comboAutoExpMode->setEnabled(false);
    sensorRow6->addWidget(m_comboAutoExpMode);
    sensorRow6->addStretch();
    sensorLayout->addLayout(sensorRow6);

    // 自动曝光开关联动
    connect(m_checkAutoExposure, &QCheckBox::toggled, this, [this](bool checked) {
        m_comboShutterAlign->setEnabled(checked);
        m_comboAutoExpMode->setEnabled(checked);
        });

    // ----- 第7行: 有效反射范围（仅写入） -----
    auto* sensorRow7 = new QHBoxLayout;
    sensorRow7->addWidget(new QLabel(tr("反射宽度:"), sensorGroup));
    m_spinPeakWidthMin = new QSpinBox(sensorGroup);
    m_spinPeakWidthMin->setRange(0, 1023);
    m_spinPeakWidthMin->setValue(2);
    m_spinPeakWidthMin->setFixedWidth(55);
    sensorRow7->addWidget(m_spinPeakWidthMin);
    sensorRow7->addWidget(new QLabel(tr("~"), sensorGroup));
    m_spinPeakWidthMax = new QSpinBox(sensorGroup);
    m_spinPeakWidthMax->setRange(0, 1023);
    m_spinPeakWidthMax->setValue(256);
    m_spinPeakWidthMax->setFixedWidth(55);
    sensorRow7->addWidget(m_spinPeakWidthMax);

    sensorRow7->addSpacing(15);
    sensorRow7->addWidget(new QLabel(tr("反射强度:"), sensorGroup));
    m_spinPeakIntensityMin = new QSpinBox(sensorGroup);
    m_spinPeakIntensityMin->setRange(0, 1023);
    m_spinPeakIntensityMin->setValue(0);
    m_spinPeakIntensityMin->setFixedWidth(55);
    sensorRow7->addWidget(m_spinPeakIntensityMin);
    sensorRow7->addWidget(new QLabel(tr("~"), sensorGroup));
    m_spinPeakIntensityMax = new QSpinBox(sensorGroup);
    m_spinPeakIntensityMax->setRange(0, 1023);
    m_spinPeakIntensityMax->setValue(1023);
    m_spinPeakIntensityMax->setFixedWidth(55);
    sensorRow7->addWidget(m_spinPeakIntensityMax);
    sensorRow7->addStretch();
    sensorLayout->addLayout(sensorRow7);
    auto* peakWriteOnlyTip = new QLabel(tr("有效反射宽度/强度：2950-50/BL当前SDK/固件无法可靠回读；界面按写入值管理，不参与运行时约束校验。"), sensorGroup);
    peakWriteOnlyTip->setStyleSheet("color: #777; font-size: 9px;");
    peakWriteOnlyTip->setWordWrap(true);
    sensorLayout->addWidget(peakWriteOnlyTip);

    // ----- 第8行: 读取/写入按钮 -----
    auto* sensorBtnRow = new QHBoxLayout;
    m_btnReadSensorParams = new QPushButton(tr("从传感器读取"), sensorGroup);
    m_btnReadSensorParams->setToolTip(tr("读取传感器当前参数配置"));
    sensorBtnRow->addWidget(m_btnReadSensorParams);

    m_btnWriteSensorParams = new QPushButton(tr("写入传感器"), sensorGroup);
    m_btnWriteSensorParams->setToolTip(tr("将当前参数写入传感器"));
    sensorBtnRow->addWidget(m_btnWriteSensorParams);

    m_btnResetSensorParams = new QPushButton(tr("恢复默认"), sensorGroup);
    m_btnResetSensorParams->setToolTip(tr("恢复为默认参数"));
    sensorBtnRow->addWidget(m_btnResetSensorParams);

    sensorBtnRow->addStretch();
    sensorLayout->addLayout(sensorBtnRow);

    // 连接信号
    connect(m_btnReadSensorParams, &QPushButton::clicked, this, &MainWindow::onReadSensorParams);
    connect(m_btnWriteSensorParams, &QPushButton::clicked, this, &MainWindow::onWriteSensorParams);
    connect(m_btnResetSensorParams, &QPushButton::clicked, this, &MainWindow::onResetSensorParams);

    layout->addWidget(sensorGroup);
    layout->addStretch();

    return widget;
}

int MainWindow::hardwareSyncFrequencyHz() const
{
    return m_spinHardwareSyncFrequency
        ? m_spinHardwareSyncFrequency->value()
        : mls::runtime::Table5D::defaultTriggerFrequencyHz;
}

void MainWindow::updateHardwareSyncFrequencyLimit()
{
    if (!m_spinHardwareSyncFrequency) return;
    const int fieldIndex = (m_comboMeasuringField && m_comboMeasuringField->currentData().isValid())
        ? m_comboMeasuringField->currentData().toInt() : 7;
    int maxHz = LaserSensor::getMaxFrequencyForIndex(fieldIndex);
    if (maxHz <= 0) maxHz = mls::runtime::Table5D::maxTriggerFrequencyHz;
    maxHz = qBound(mls::runtime::Table5D::minTriggerFrequencyHz,
        maxHz,
        mls::runtime::Table5D::maxTriggerFrequencyHz);
    m_spinHardwareSyncFrequency->setRange(mls::runtime::Table5D::minTriggerFrequencyHz, maxHz);
    const int hz = m_spinHardwareSyncFrequency->value();
    const int topology = m_comboHardwareTopology ? m_comboHardwareTopology->currentData().toInt() : 1;
    const QString clockSource = topology==3
        ? tr("H3：传感器内部Profile时钟经CMM到IN1/R1")
        : tr("H1：ECI3808 PWM0/OP14外触发到传感器与IN0/R0");
    m_spinHardwareSyncFrequency->setToolTip(tr(
        "%1。当前Measuring Field索引%2的项目Quick Reference保守上限=%3 Hz；"
        "参数页只表示本次设置意图，不是物理时钟读回。正式采集启动时会另行验证控制器/传感器。当前周期=%4 ms。")
        .arg(clockSource).arg(fieldIndex).arg(maxHz)
        .arg(1000.0 / static_cast<double>(hz), 0, 'f', 6));
}

void MainWindow::refreshHardwareTemporalRuntimeUi()
{
    if (!m_checkHardwareTemporalRuntime || !m_lblHardwareTemporalStatus ||
        !m_lblHardwareTemporalValue || !m_lblHardwareTemporalConfig) return;

    m_refreshingHardwareTemporalUi = true;
    HardwareTemporalCorrectionSettings c;
    if (m_syncManager) c = m_syncManager->hardwareTemporalCorrection();
    const bool loaded = !c.calibrationId.isEmpty() && std::isfinite(c.delaySec)
        && c.triggerPeriodMs > 0;
    const bool running = m_syncManager && m_syncManager->isRunning();

    m_checkHardwareTemporalRuntime->setChecked(loaded && c.enabled);
    m_checkHardwareTemporalRuntime->setEnabled(loaded && !running);

    if (!loaded) {
        m_lblHardwareTemporalStatus->setText(tr("活动标定：未加载。完成硬件同步标定+Hold-out PASS后会自动保存并在下次启动加载。"));
        m_lblHardwareTemporalStatus->setStyleSheet(QStringLiteral("QLabel { color:#777; }"));
        m_lblHardwareTemporalValue->setText(tr("τ=--- µs    σ=--- µs"));
        m_lblHardwareTemporalConfig->setText(tr("标定配置：---"));
        if (m_pointCloudView) m_pointCloudView->setTemporalCorrectionStatus(false, false, 0.0, QString());
        m_refreshingHardwareTemporalUi = false;
        return;
    }

    const double delayUs = c.delaySec * 1e6;
    const double stdUs = c.delayStdSec * 1e6;
    const double alpha = c.triggerPeriodMs > 0
        ? c.delaySec / (static_cast<double>(c.triggerPeriodMs) / 1000.0) : 0.0;
    m_lblHardwareTemporalStatus->setText(c.enabled
        ? tr("活动标定：%1 / 已验证 / 运行时补偿 ON（仅硬件同步）%2")
        .arg(c.calibrationId).arg(running ? tr("（采集中锁定）") : QString())
        : tr("活动标定：%1 / 已验证 / 运行时补偿 OFF（RAW）%2")
        .arg(c.calibrationId).arg(running ? tr("（采集中锁定）") : QString()));
    m_lblHardwareTemporalStatus->setStyleSheet(c.enabled
        ? QStringLiteral("QLabel { color:#138a36; font-weight:600; }")
        : QStringLiteral("QLabel { color:#a06000; font-weight:600; }"));
    m_lblHardwareTemporalValue->setText(tr("τ=%1 µs    σ=%2 µs    α=τ/T=%3")
        .arg(delayUs, 0, 'f', 2).arg(stdUs, 0, 'f', 2).arg(alpha, 0, 'f', 6));

    const bool haveCurrentResolution = m_comboResolution && m_comboResolution->count() > 0;
    const int currentResolution = haveCurrentResolution ? m_comboResolution->currentIndex() : -1;
    const int currentExposure = m_spinExposure ? m_spinExposure->value() : -1;
    const int currentTopology = m_comboHardwareTopology
        ? m_comboHardwareTopology->currentData().toInt() : 1;
    const int currentEdge = currentTopology==1 ? 104
        : (m_comboCmmR1Edge?m_comboCmmR1Edge->currentData().toInt():114);
    // H3 overwrites idle on start so the UI raw idle control is not the
    // effective scan timing. Compare to the actual firmware-derived idle.
    const int currentIdle = currentTopology==3 && hardwareSyncFrequencyHz()>0 && m_spinExposure
        ? int(std::lround(100000.0 / hardwareSyncFrequencyHz())) - m_spinExposure->value()
        : (m_spinIdle ? m_spinIdle->value() : -1);
    const int currentField = (m_comboMeasuringField && m_comboMeasuringField->currentData().isValid())
        ? m_comboMeasuringField->currentData().toInt() : -1;
    const int currentFrequency = hardwareSyncFrequencyHz();
    const bool configMatch = haveCurrentResolution
        && currentResolution == static_cast<int>(c.sensorParams.resolutionIndex)
        && currentExposure == static_cast<int>(c.sensorParams.exposureTime)
        && currentIdle == static_cast<int>(c.sensorParams.idleTime)
        && currentField == c.sensorParams.measuringFieldIndex
        && c.triggerFrequencyHz == currentFrequency
        && static_cast<int>(c.topology)==currentTopology && c.latchEdge==currentEdge
        && (currentTopology!=3 || (c.cmmDivider==1 && c.cmmSkewHalfUs==0 && c.cmmRatio==1));
    // Always allow turning an already active correction OFF, even after UI
    // parameters/topology changed. Turning a mismatched correction ON is blocked.
    m_checkHardwareTemporalRuntime->setEnabled(loaded && !running && (configMatch || c.enabled));
    const QString currentText = haveCurrentResolution
        ? tr("当前=Field%1 / Res%2 / Exp=%3 / Idle=%4 / %5Hz (%6ms)")
        .arg(currentField).arg(currentResolution).arg(currentExposure).arg(currentIdle)
        .arg(currentFrequency).arg(1000.0 / static_cast<double>(currentFrequency), 0, 'f', 6)
        : tr("当前传感器分辨率尚未读取");
    m_lblHardwareTemporalConfig->setText(tr("标定H%9 / 锁存边沿%10；Field=%1 / ResolutionIndex=%2 / Exp=%3 / Idle=%4 / %5Hz / Period=%6ms；%7；%8")
        .arg(c.sensorParams.measuringFieldIndex).arg(c.sensorParams.resolutionIndex).arg(c.sensorParams.exposureTime).arg(c.sensorParams.idleTime)
        .arg(c.triggerFrequencyHz).arg(c.triggerPeriodMs, 0, 'f', 6).arg(currentText)
        .arg(configMatch ? tr("配置匹配") : tr("配置未匹配：禁止启用；若已启用，正式采集会Fail-Closed拒绝启动"))
        .arg(static_cast<int>(c.topology)).arg(c.latchEdge));
    m_lblHardwareTemporalConfig->setStyleSheet(configMatch
        ? QStringLiteral("QLabel { color:#286b3c; }")
        : QStringLiteral("QLabel { color:#a04020; }"));

    if (m_pointCloudView) {
        const bool hardwareModeSelected = m_radioHardwareSync && m_radioHardwareSync->isChecked();
        m_pointCloudView->setTemporalCorrectionStatus(true, c.enabled, delayUs, c.calibrationId, hardwareModeSelected);
    }
    m_refreshingHardwareTemporalUi = false;
}

void MainWindow::clearPointCloudForTemporalModeChange()
{
    // Never mix already-reconstructed RAW points and q(t+tau)-corrected points in
    // one VTK cloud.  Sensor/profile views are unaffected and are intentionally kept.
    if (m_cloudBuilder) m_cloudBuilder->clear();
    if (m_pointCloudView) m_pointCloudView->clear();
    if (m_syncManager) m_syncManager->clearData();
}

void MainWindow::onHardwareTemporalRuntimeToggled(bool checked)
{
    if (m_refreshingHardwareTemporalUi) return;
    const bool previous = m_syncManager && m_syncManager->hardwareTemporalCorrection().enabled;
    if (checked == previous) return;
    if (!m_calibrationWidget || !m_syncManager) {
        refreshHardwareTemporalRuntimeUi();
        return;
    }
    QString error;
    if (!m_calibrationWidget->setHardwareTemporalRuntimeEnabled(checked, &error)) {
        LOG_WARNING(LogCategory::Calibration,
            tr("[硬件同步时域补偿] 切换失败：%1").arg(error));
        QMessageBox::warning(this, tr("时域补偿切换失败"), error);
        refreshHardwareTemporalRuntimeUi();
        return;
    }

    const auto c = m_syncManager->hardwareTemporalCorrection();
    statusBar()->showMessage(c.enabled
        ? tr("硬件同步时域补偿已启用，后续硬件同步三维点云使用q(t+τ)重建")
        : tr("硬件同步时域补偿已关闭，后续硬件同步三维点云使用RAW REGIST位姿"), 5000);
    refreshHardwareTemporalRuntimeUi();
}

QWidget* MainWindow::createControlSection(QWidget* parent)
{
    auto* widget = new QWidget(parent);
    auto* layout = new QVBoxLayout(widget);
    layout->setContentsMargins(5, 5, 5, 5);
    layout->setSpacing(8);

    // ========== 机床控制 ==========
    auto* machineGroup = new QGroupBox(tr("机床控制"), widget);
    auto* machineLayout = new QVBoxLayout(machineGroup);
    machineLayout->setSpacing(5);

    // 单轴控制
    m_xAxisWidget = new AxisControlWidget(AxisX, "X", machineGroup);
    m_yAxisWidget = new AxisControlWidget(AxisY, "Y", machineGroup);
    m_zAxisWidget = new AxisControlWidget(AxisZ, "Z", machineGroup);
    m_cAxisWidget = new AxisControlWidget(AxisC, "C", machineGroup);
    m_bAxisWidget = new AxisControlWidget(AxisB, "B", machineGroup);

    m_xAxisWidget->setController(m_zmc.get());
    m_yAxisWidget->setController(m_zmc.get());
    m_zAxisWidget->setController(m_zmc.get());
    m_cAxisWidget->setController(m_zmc.get());
    m_bAxisWidget->setController(m_zmc.get());

    // C/B 无原点开关：必须先同步绝对位置，再由人工按钮使能；其“回零”是绝对定位到 0°。
    m_cAxisWidget->configureManualServoEnable(true);
    m_bAxisWidget->configureManualServoEnable(true);

    connectAxisSignals(m_xAxisWidget);
    connectAxisSignals(m_yAxisWidget);
    connectAxisSignals(m_zAxisWidget);
    connectAxisSignals(m_cAxisWidget);
    connectAxisSignals(m_bAxisWidget);

    machineLayout->addWidget(m_xAxisWidget);
    machineLayout->addWidget(m_yAxisWidget);
    machineLayout->addWidget(m_zAxisWidget);
    machineLayout->addWidget(m_cAxisWidget);
    machineLayout->addWidget(m_bAxisWidget);

    // 全局控制按钮
    auto* globalLayout = new QHBoxLayout;
    m_btnZeroAll = new QPushButton(tr("XYZ 清零"), machineGroup);
    m_btnHomeAll = new QPushButton(tr("五轴全部回零"), machineGroup);
    m_btnHomeAll->setToolTip(tr("XYZ 执行 DATUM 回零；B/C 在绝对位置同步且人工使能后绝对定位到 0°"));
    m_btnStopAll = new QPushButton(tr("五轴全部停止"), machineGroup);
    m_btnResyncRotaryAbs = new QPushButton(tr("重读 B/C 绝对位置"), machineGroup);
    m_btnResyncRotaryAbs->setEnabled(false);
    m_btnResyncRotaryAbs->setToolTip(tr("会先断开 B/C 使能，然后重新读取禾川绝对位置并重建上位机机械坐标框架；不会写 MPOS/DPOS"));
    m_btnStopAll->setStyleSheet("QPushButton { background-color: #ff6666; font-weight: bold; }");

    globalLayout->addWidget(m_btnZeroAll);
    globalLayout->addWidget(m_btnHomeAll);
    globalLayout->addWidget(m_btnStopAll);
    globalLayout->addWidget(m_btnResyncRotaryAbs);
    globalLayout->addStretch();
    machineLayout->addLayout(globalLayout);

    connect(m_btnZeroAll, &QPushButton::clicked, this, &MainWindow::onZeroAll);
    connect(m_btnHomeAll, &QPushButton::clicked, this, &MainWindow::onHomeAll);
    connect(m_btnStopAll, &QPushButton::clicked, this, &MainWindow::onStopAll);
    connect(m_btnResyncRotaryAbs, &QPushButton::clicked, this, &MainWindow::onResyncRotaryAbsolute);

    // 轨迹插补
    auto* pathGroup = new QGroupBox(tr("轨迹插补"), machineGroup);
    auto* pathLayout = new QVBoxLayout(pathGroup);

    m_pathTable = new QTableWidget(0, 4, pathGroup);
    m_pathTable->setHorizontalHeaderLabels({ "X", "Y", "Z", "Feed" });
    m_pathTable->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    m_pathTable->setMaximumHeight(80);
    m_pathTable->setEditTriggers(QAbstractItemView::DoubleClicked | QAbstractItemView::SelectedClicked);
    pathLayout->addWidget(m_pathTable);

    auto* pathBtnRow1 = new QHBoxLayout;
    m_btnLoadPath = new QPushButton(tr("导入"), pathGroup);
    m_btnAddPoint = new QPushButton(tr("添加"), pathGroup);
    m_btnRemovePoint = new QPushButton(tr("删除"), pathGroup);
    pathBtnRow1->addWidget(m_btnLoadPath);
    pathBtnRow1->addWidget(m_btnAddPoint);
    pathBtnRow1->addWidget(m_btnRemovePoint);
    pathBtnRow1->addStretch();
    pathLayout->addLayout(pathBtnRow1);

    auto* pathBtnRow2 = new QHBoxLayout;
    m_btnStartMerge = new QPushButton(tr("启动插补"), pathGroup);
    m_btnStopMerge = new QPushButton(tr("停止"), pathGroup);
    pathBtnRow2->addWidget(m_btnStartMerge);
    pathBtnRow2->addWidget(m_btnStopMerge);
    pathBtnRow2->addStretch();
    pathLayout->addLayout(pathBtnRow2);

    machineLayout->addWidget(pathGroup);

    connect(m_btnLoadPath, &QPushButton::clicked, this, &MainWindow::onLoadPathFromFile);
    connect(m_btnAddPoint, &QPushButton::clicked, this, &MainWindow::onAddPathPoint);
    connect(m_btnRemovePoint, &QPushButton::clicked, this, &MainWindow::onRemovePathPoint);
    connect(m_btnStartMerge, &QPushButton::clicked, this, &MainWindow::onStartMerge);
    connect(m_btnStopMerge, &QPushButton::clicked, this, &MainWindow::onStopMerge);

    layout->addWidget(machineGroup);

    // ========== 采集控制 ==========
    auto* captureGroup = new QGroupBox(tr("采集控制"), widget);
    auto* captureLayout = new QVBoxLayout(captureGroup);

    auto* captureBtnLayout = new QHBoxLayout;
    m_btnStartCapture = new QPushButton(tr("开始采集"), captureGroup);
    m_btnStartCapture->setStyleSheet("QPushButton { background-color: #66ff66; }");
    m_btnStopCapture = new QPushButton(tr("停止采集"), captureGroup);
    m_btnStopCapture->setStyleSheet("QPushButton { background-color: #ffaa66; }");
    m_btnClearData = new QPushButton(tr("清除数据"), captureGroup);
    m_btnAnalyzeData = new QPushButton(tr("数据分析"), captureGroup);

    captureBtnLayout->addWidget(m_btnStartCapture);
    captureBtnLayout->addWidget(m_btnStopCapture);
    captureBtnLayout->addWidget(m_btnClearData);
    captureBtnLayout->addWidget(m_btnAnalyzeData);
    captureBtnLayout->addStretch();
    captureLayout->addLayout(captureBtnLayout);

    // 采集状态
    auto* statusLayout = new QHBoxLayout;
    statusLayout->addWidget(new QLabel(tr("状态:"), captureGroup));
    m_lblCaptureStatus = new QLabel(tr("未采集"), captureGroup);
    m_lblCaptureStatus->setStyleSheet("QLabel { font-weight: bold; }");
    statusLayout->addWidget(m_lblCaptureStatus);
    statusLayout->addStretch();
    captureLayout->addLayout(statusLayout);

    connect(m_btnStartCapture, &QPushButton::clicked, this, &MainWindow::onStartCapture);
    connect(m_btnStopCapture, &QPushButton::clicked, this, &MainWindow::onStopCapture);
    connect(m_btnClearData, &QPushButton::clicked, this, &MainWindow::onClearData);
    connect(m_btnAnalyzeData, &QPushButton::clicked, this, &MainWindow::onAnalyzeData);

    layout->addWidget(captureGroup);

    // ========== G代码控制 ==========
    auto* gcodeGroup = new QGroupBox(tr("G代码控制"), widget);
    auto* gcodeLayout = new QVBoxLayout(gcodeGroup);

    m_gcodeWidget = new GCodeWidget(gcodeGroup);
    gcodeLayout->addWidget(m_gcodeWidget);

    // 设置G代码执行器（在createUi之后设置）
    // m_gcodeWidget->setExecutor(m_gcodeExecutor.get());

    // 连接G代码控件信号
    connect(m_gcodeWidget, &GCodeWidget::requestDisableControls,
        this, &MainWindow::onGCodeDisableControls);
    connect(m_gcodeWidget, &GCodeWidget::executionStateChanged,
        this, &MainWindow::onGCodeExecutionStateChanged);
    // 使用DirectConnection确保同步执行，在G代码执行前更新参数
    connect(m_gcodeWidget, &GCodeWidget::requestSensorParams,
        this, &MainWindow::onGCodeRequestSensorParams, Qt::DirectConnection);
    connect(m_gcodeWidget, &GCodeWidget::requestStageHardwareSettings,
        this, &MainWindow::stageGCodeHardwareSettings, Qt::DirectConnection);
    connect(m_gcodeWidget, &GCodeWidget::requestHardwareNcProbe,
        this, &MainWindow::onGCodeHardwareNcProbeRequested, Qt::DirectConnection);
    connect(m_gcodeWidget, &GCodeWidget::requestStopHardwareNcProbe,
        this, &MainWindow::onGCodeHardwareNcProbeStopRequested);
    connect(m_gcodeWidget, &GCodeWidget::requestFullNcGeometryAudit,
        this, &MainWindow::onGCodeFullNcGeometryAuditRequested);
    connect(m_gcodeWidget, &GCodeWidget::requestCancelFullNcGeometryAudit,
        this, &MainWindow::onGCodeFullNcGeometryAuditCancelRequested);
    connect(m_gcodeWidget, &GCodeWidget::requestSiteExecutionAcceptance,
        this, &MainWindow::onGCodeSiteExecutionAcceptanceRequested, Qt::DirectConnection);
    connect(m_gcodeWidget, &GCodeWidget::requestInvalidateNcPreflightEvidence,
        this, &MainWindow::invalidateNcPreflightEvidence,Qt::DirectConnection);
    connect(&m_gcodeOfflineGeometryAuditWatcher,&QFutureWatcher<QString>::finished,this,[this]() {
        const QString outcome=m_gcodeOfflineGeometryAuditWatcher.result();
        bool passed=!m_closeRequested && outcome.startsWith(QStringLiteral("PASS\n")) &&
            !m_pendingGeometryNcDigest.isEmpty() && !m_pendingGeometryCadIdentity.isEmpty();
        QString failure;
        if(passed){
            // CAD and its 6DoF pose must not have changed during the async scan.
            mls::twin::TwinScene current;
            QByteArray identity;
            bool reviewed=false;
            QString snapshotError;
            passed=m_pathPlanningWidget && m_pathPlanningWidget->snapshotCollisionWorkpiece(
                current,identity,reviewed,&snapshotError) &&
                identity==m_pendingGeometryCadIdentity &&
                m_pendingNcWorkpieceSafety && m_pendingNcWorkpieceSafety->isReady();
            if(!passed)failure=tr("复核期间STEP文件/工件位姿已发生变化，旧结果无效：%1")
                .arg(snapshotError);
        }
        m_geometryNcProof.passed=passed;
        m_geometryNcProof.independentPartRegistrationCertified=false;
        if(passed){
            m_geometryNcProof.normalizedNcDigest=m_pendingGeometryNcDigest;
            m_geometryNcProof.model=m_pendingGeometryNcModel;
            m_geometryNcProof.connectionGeneration=m_pendingGeometryConnectionGeneration;
            m_geometryNcProof.compensationEnabled=m_pendingGeometryCompensationEnabled;
            m_geometryNcProof.finishedAt=std::chrono::steady_clock::now();
            m_ncWorkpieceSafety=m_pendingNcWorkpieceSafety;
            m_geometryCadIdentity=m_pendingGeometryCadIdentity;
        }else{
            m_ncWorkpieceSafety.reset();
            m_geometryCadIdentity.clear();
        }
        m_pendingNcWorkpieceSafety.reset();
        m_pendingGeometryCadIdentity.clear();
        m_pendingGeometryNcDigest.clear();
        if(!m_closeRequested && m_gcodeWidget)m_gcodeWidget->showFullNcGeometryAuditResult(
            passed,failure.isEmpty() ? outcome.mid(outcome.indexOf(QLatin1Char('\n'))+1) : failure);
        m_gcodeGeometryCancel.reset();
    });
    connect(m_syncManager.get(), &SyncManager::errorOccurred, this, [this](const QString& message) {
        if (!m_gcodeHardwareNcProbeActive) return;
        m_gcodeHardwareNcProbeError=message;
        if(!m_gcodeHardwareNcProbeStopping)
            QTimer::singleShot(0,this,[this]() { if(m_gcodeHardwareNcProbeActive)finishGCodeHardwareNcProbe(true); });
    });

    layout->addWidget(gcodeGroup);

    // ========== DPOS / MPOS XYZ位移诊断 ==========
    // 目的：把“固定零偏”和“真实移动距离不一致”分开。手工记录覆盖XYZ；
    // 自动程序可单独测试X/Y/Z，或按X→Y→Z依次执行同一诊断计划。
    auto* axisDiagGroup = new QGroupBox(tr("DPOS / MPOS 位移诊断"), widget);
    auto* axisDiagLayout = new QVBoxLayout(axisDiagGroup);
    axisDiagLayout->setSpacing(5);

    auto* axisDiagTop = new QHBoxLayout;
    m_checkAxisMoveDiagAuto = new QCheckBox(tr("自动记录XYZ单轴绝对/相对定位"), axisDiagGroup);
    m_checkAxisMoveDiagAuto->setChecked(false);
    m_checkAxisMoveDiagAuto->setToolTip(tr("启用后，每次本控制页单轴绝对/相对定位会自动记录运动前、IDLE后立即、稳定等待后的DPOS/MPOS。不会改变运动。"));
    axisDiagTop->addWidget(m_checkAxisMoveDiagAuto);
    axisDiagTop->addWidget(new QLabel(tr("稳定等待:"), axisDiagGroup));
    m_spinAxisMoveDiagSettleMs = new QSpinBox(axisDiagGroup);
    m_spinAxisMoveDiagSettleMs->setRange(0, 10000);
    m_spinAxisMoveDiagSettleMs->setValue(2000);
    m_spinAxisMoveDiagSettleMs->setSingleStep(100);
    m_spinAxisMoveDiagSettleMs->setSuffix(tr(" ms"));
    m_spinAxisMoveDiagSettleMs->setToolTip(tr("轴报告IDLE后继续等待该时间，再读取稳定后的DPOS/MPOS。建议XYZ诊断统一使用2000ms，便于横向比较。"));
    axisDiagTop->addWidget(m_spinAxisMoveDiagSettleMs);
    axisDiagTop->addStretch();
    axisDiagLayout->addLayout(axisDiagTop);

    auto* axisDiagAuto = new QHBoxLayout;
    axisDiagAuto->addWidget(new QLabel(tr("自动轴:"), axisDiagGroup));
    m_comboAxisMoveDiagAutoAxis = new QComboBox(axisDiagGroup);
    m_comboAxisMoveDiagAutoAxis->addItem(QStringLiteral("X"), AxisX);
    m_comboAxisMoveDiagAutoAxis->addItem(QStringLiteral("Y"), AxisY);
    m_comboAxisMoveDiagAutoAxis->addItem(QStringLiteral("Z"), AxisZ);
    m_comboAxisMoveDiagAutoAxis->addItem(tr("XYZ依次"), -1);
    m_comboAxisMoveDiagAutoAxis->setCurrentIndex(2); // 保持V27.6.27默认行为：先测Z；需要时可一键依次测XYZ。
    m_comboAxisMoveDiagAutoAxis->setToolTip(tr("选择单独X/Y/Z，或按X→Y→Z依次执行同一诊断计划。XYZ依次模式在POST_HOME下会在每个轴测试结束后恢复该轴启动位置，再测试下一轴。"));
    axisDiagAuto->addWidget(m_comboAxisMoveDiagAutoAxis);
    axisDiagAuto->addWidget(new QLabel(tr("测试阶段:"), axisDiagGroup));
    m_comboAxisMoveDiagPhase = new QComboBox(axisDiagGroup);
    m_comboAxisMoveDiagPhase->addItem(tr("回零后（POST_HOME，推荐）"), QStringLiteral("POST_HOME"));
    m_comboAxisMoveDiagPhase->addItem(tr("上电未回零（PRE_HOME，多距离相对往返）"), QStringLiteral("PRE_HOME"));
    axisDiagAuto->addWidget(m_comboAxisMoveDiagPhase);
    axisDiagAuto->addWidget(new QLabel(tr("自动计划:"), axisDiagGroup));
    m_comboAxisMoveDiagPlan = new QComboBox(axisDiagGroup);
    m_comboAxisMoveDiagPlan->addItem(tr("多距离/多位置矩阵（推荐）"), QStringLiteral("DISTANCE_POSITION_MATRIX"));
    m_comboAxisMoveDiagPlan->addItem(tr("三段5mm对比：远离零点/零点附近/跨零"), QStringLiteral("THREE_SEGMENT_5MM"));
    m_comboAxisMoveDiagPlan->addItem(tr("零点0.5mm细扫：-2→+5→-2"), QStringLiteral("ZERO_FINE_SCAN"));
    m_comboAxisMoveDiagPlan->addItem(tr("当前位置相对±5mm往返"), QStringLiteral("RELATIVE_5MM"));
    axisDiagAuto->addWidget(m_comboAxisMoveDiagPlan);
    axisDiagAuto->addWidget(new QLabel(tr("重复:"), axisDiagGroup));
    m_spinAxisMoveDiagRepeats = new QSpinBox(axisDiagGroup);
    m_spinAxisMoveDiagRepeats->setRange(1, 20);
    m_spinAxisMoveDiagRepeats->setValue(2);
    axisDiagAuto->addWidget(m_spinAxisMoveDiagRepeats);
    m_btnAxisMoveDiagAutoStart = new QPushButton(tr("开始自动测试"), axisDiagGroup);
    m_btnAxisMoveDiagAutoStop = new QPushButton(tr("停止自动测试"), axisDiagGroup);
    m_btnAxisMoveDiagAutoStop->setEnabled(false);
    axisDiagAuto->addWidget(m_btnAxisMoveDiagAutoStart);
    axisDiagAuto->addWidget(m_btnAxisMoveDiagAutoStop);
    axisDiagAuto->addStretch();
    axisDiagLayout->addLayout(axisDiagAuto);

    auto* axisDiagMatrix = new QHBoxLayout;
    axisDiagMatrix->addWidget(new QLabel(tr("起点列表(mm):"), axisDiagGroup));
    m_editAxisMoveDiagBasePositions = new QLineEdit(QStringLiteral("-10,-7.5,-5,-2.5,0"), axisDiagGroup);
    m_editAxisMoveDiagBasePositions->setToolTip(
        tr("POST_HOME矩阵测试的绝对起点列表，可用逗号/分号/空格分隔。"
            "建议覆盖一个丝杠导程内的不同相位；Z轴10mm导程可先用 -10,-7.5,-5,-2.5,0。"));
    m_editAxisMoveDiagBasePositions->setMinimumWidth(190);
    axisDiagMatrix->addWidget(m_editAxisMoveDiagBasePositions);
    axisDiagMatrix->addWidget(new QLabel(tr("移动距离(mm):"), axisDiagGroup));
    m_editAxisMoveDiagDistances = new QLineEdit(QStringLiteral("5,8,10,20,30"), axisDiagGroup);
    m_editAxisMoveDiagDistances->setToolTip(
        tr("测试距离列表，可用逗号/分号/空格分隔。每个起点都会执行 起点→起点+距离→起点，"
            "因此同时得到正向和反向同距离数据。PRE_HOME只使用此距离列表，忽略绝对起点。"));
    m_editAxisMoveDiagDistances->setMinimumWidth(170);
    axisDiagMatrix->addWidget(m_editAxisMoveDiagDistances);
    axisDiagMatrix->addStretch();
    axisDiagLayout->addLayout(axisDiagMatrix);

    m_lblAxisMoveDiagPlan = new QLabel(axisDiagGroup);
    m_lblAxisMoveDiagPlan->setWordWrap(true);
    m_lblAxisMoveDiagPlan->setStyleSheet("QLabel { color: #444; font-size: 9px; }");
    axisDiagLayout->addWidget(m_lblAxisMoveDiagPlan);

    m_progressAxisMoveDiagAuto = new QProgressBar(axisDiagGroup);
    m_progressAxisMoveDiagAuto->setRange(0, 100);
    m_progressAxisMoveDiagAuto->setValue(0);
    m_progressAxisMoveDiagAuto->setFormat(tr("自动测试 %p%"));
    axisDiagLayout->addWidget(m_progressAxisMoveDiagAuto);

    auto* axisDiagManual = new QHBoxLayout;
    axisDiagManual->addWidget(new QLabel(tr("手工轴:"), axisDiagGroup));
    m_comboAxisMoveDiagManualAxis = new QComboBox(axisDiagGroup);
    m_comboAxisMoveDiagManualAxis->addItem(QStringLiteral("X"), AxisX);
    m_comboAxisMoveDiagManualAxis->addItem(QStringLiteral("Y"), AxisY);
    m_comboAxisMoveDiagManualAxis->addItem(QStringLiteral("Z"), AxisZ);
    m_comboAxisMoveDiagManualAxis->setCurrentIndex(2);
    axisDiagManual->addWidget(m_comboAxisMoveDiagManualAxis);
    m_btnAxisMoveDiagManualStart = new QPushButton(tr("记录起点"), axisDiagGroup);
    m_btnAxisMoveDiagManualEnd = new QPushButton(tr("记录终点并添加"), axisDiagGroup);
    m_btnAxisMoveDiagClear = new QPushButton(tr("清空"), axisDiagGroup);
    m_btnAxisMoveDiagSave = new QPushButton(tr("保存CSV"), axisDiagGroup);
    axisDiagManual->addWidget(m_btnAxisMoveDiagManualStart);
    axisDiagManual->addWidget(m_btnAxisMoveDiagManualEnd);
    axisDiagManual->addWidget(m_btnAxisMoveDiagClear);
    axisDiagManual->addWidget(m_btnAxisMoveDiagSave);
    axisDiagManual->addStretch();
    axisDiagLayout->addLayout(axisDiagManual);

    m_lblAxisMoveDiagStatus = new QLabel(
        tr("Δ误差 = ΔDPOS - ΔMPOS；矩阵计划支持任意起点与5/8/10/20/30mm等任意距离，并记录误差ppm、ENCODER、SetMpos generation。"),
        axisDiagGroup);
    m_lblAxisMoveDiagStatus->setWordWrap(true);
    m_lblAxisMoveDiagStatus->setStyleSheet("QLabel { color: #555; font-size: 9px; }");
    axisDiagLayout->addWidget(m_lblAxisMoveDiagStatus);

    m_axisMoveDiagTable = new QTableWidget(0, 29, axisDiagGroup);
    m_axisMoveDiagTable->setHorizontalHeaderLabels({
        tr("#"), tr("阶段"), tr("计划/步骤"), tr("轴"), tr("命令"),
        tr("基准位置"), tr("测试距离"), tr("方向"),
        tr("起点DPOS"), tr("起点MPOS"), tr("终点DPOS"), tr("终点MPOS"),
        tr("请求Δ"), tr("ΔDPOS"), tr("ΔMPOS"), tr("ΔD-ΔM"), tr("误差ppm"), tr("ΔENCODER"),
        tr("FE起点"), tr("FE终点"), tr("ΔFE"), tr("IDLE后MPOS漂移"),
        tr("过DPOS零"), tr("DATUM S→E"), tr("SetMpos# S→E"),
        tr("Ref S→E"), tr("角色"), tr("循环"), tr("状态")
        });
    m_axisMoveDiagTable->verticalHeader()->setVisible(false);
    m_axisMoveDiagTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_axisMoveDiagTable->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_axisMoveDiagTable->setHorizontalScrollMode(QAbstractItemView::ScrollPerPixel);
    m_axisMoveDiagTable->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    m_axisMoveDiagTable->horizontalHeader()->setStretchLastSection(true);
    m_axisMoveDiagTable->setMinimumHeight(150);
    m_axisMoveDiagTable->setMaximumHeight(230);
    axisDiagLayout->addWidget(m_axisMoveDiagTable);

    connect(m_btnAxisMoveDiagManualStart, &QPushButton::clicked, this, [this]() { startManualAxisMoveDiagnostic(); });
    connect(m_btnAxisMoveDiagManualEnd, &QPushButton::clicked, this, [this]() { finishManualAxisMoveDiagnostic(); });
    connect(m_btnAxisMoveDiagClear, &QPushButton::clicked, this, [this]() { clearAxisMoveDiagnostics(); });
    connect(m_btnAxisMoveDiagSave, &QPushButton::clicked, this, [this]() { saveAxisMoveDiagnosticsCsv(); });
    connect(m_btnAxisMoveDiagAutoStart, &QPushButton::clicked, this, [this]() { startAutomaticAxisMoveDiagnostic(); });
    connect(m_btnAxisMoveDiagAutoStop, &QPushButton::clicked, this, [this]() { stopAutomaticAxisMoveDiagnostic(); });
    connect(m_comboAxisMoveDiagAutoAxis, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int) { updateAutomaticAxisMoveDiagnosticUi(); });
    connect(m_comboAxisMoveDiagPhase, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int) { updateAutomaticAxisMoveDiagnosticUi(); });
    connect(m_comboAxisMoveDiagPlan, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int) { updateAutomaticAxisMoveDiagnosticUi(); });
    connect(m_editAxisMoveDiagBasePositions, &QLineEdit::textChanged, this, [this](const QString&) { updateAutomaticAxisMoveDiagnosticUi(); });
    connect(m_editAxisMoveDiagDistances, &QLineEdit::textChanged, this, [this](const QString&) { updateAutomaticAxisMoveDiagnosticUi(); });
    connect(m_spinAxisMoveDiagRepeats, qOverload<int>(&QSpinBox::valueChanged), this, [this](int) { updateAutomaticAxisMoveDiagnosticUi(); });
    updateAutomaticAxisMoveDiagnosticUi();

    layout->addWidget(axisDiagGroup);

    layout->addWidget(createRuntimeAxisCompensationPanel(widget));

    // ========== V27.6.33 XYZ 位置补偿表自动标定 + 局部速度资格测试 + 单向连续采表 ==========
    // 此采集工作流始终使用显式 RAW DPOS API；运行补偿在独立面板启用。
    auto* axisCompGroup = new QGroupBox(tr("XYZ 位置补偿表自动标定（RAW 建表，与运行补偿隔离）"), widget);
    auto* axisCompLayout = new QVBoxLayout(axisCompGroup);
    axisCompLayout->setSpacing(5);

    auto* axisCompTop = new QHBoxLayout;
    axisCompTop->addWidget(new QLabel(tr("标定轴:"), axisCompGroup));
    m_comboAxisCompCalAxis = new QComboBox(axisCompGroup);
    m_comboAxisCompCalAxis->addItem(QStringLiteral("X"), AxisX);
    m_comboAxisCompCalAxis->addItem(QStringLiteral("Y"), AxisY);
    m_comboAxisCompCalAxis->addItem(QStringLiteral("Z"), AxisZ);
    m_comboAxisCompCalAxis->addItem(tr("XYZ依次"), -1);
    m_comboAxisCompCalAxis->setCurrentIndex(2);
    m_comboAxisCompCalAxis->setToolTip(tr("可单独建立X/Y/Z表，也可以X→Y→Z依次完成。所有被选中的轴必须先正规回零。"));
    axisCompTop->addWidget(m_comboAxisCompCalAxis);

    axisCompTop->addWidget(new QLabel(tr("重复扫描:"), axisCompGroup));
    m_spinAxisCompCalRepeats = new QSpinBox(axisCompGroup);
    m_spinAxisCompCalRepeats->setRange(1, 10);
    m_spinAxisCompCalRepeats->setValue(3);
    m_spinAxisCompCalRepeats->setToolTip(tr("每次循环包含完整正向扫描和完整反向扫描。建议正式采表使用3次。"));
    axisCompTop->addWidget(m_spinAxisCompCalRepeats);

    axisCompTop->addWidget(new QLabel(tr("稳定等待:"), axisCompGroup));
    m_spinAxisCompCalSettleMs = new QSpinBox(axisCompGroup);
    m_spinAxisCompCalSettleMs->setRange(0, 10000);
    m_spinAxisCompCalSettleMs->setValue(2000);
    m_spinAxisCompCalSettleMs->setSingleStep(100);
    m_spinAxisCompCalSettleMs->setSuffix(tr(" ms"));
    axisCompTop->addWidget(m_spinAxisCompCalSettleMs);

    axisCompTop->addWidget(new QLabel(tr("换向预加载:"), axisCompGroup));
    m_spinAxisCompCalPreloadMm = new QDoubleSpinBox(axisCompGroup);
    m_spinAxisCompCalPreloadMm->setRange(0.1, 20.0);
    m_spinAxisCompCalPreloadMm->setDecimals(3);
    m_spinAxisCompCalPreloadMm->setValue(2.0);
    m_spinAxisCompCalPreloadMm->setSuffix(tr(" mm"));
    m_spinAxisCompCalPreloadMm->setToolTip(
        tr("正向扫描先到起点下方该距离，再正向进入起点；反向扫描先到终点上方该距离，再反向进入终点，保证方向表的接近方向一致。"));
    axisCompTop->addWidget(m_spinAxisCompCalPreloadMm);
    axisCompTop->addStretch();
    axisCompLayout->addLayout(axisCompTop);

    m_axisCompCalRangeTable = new QTableWidget(3, 4, axisCompGroup);
    m_axisCompCalRangeTable->setHorizontalHeaderLabels({ tr("轴"), tr("起始DPOS(mm)"), tr("终止DPOS(mm)"), tr("步长(mm)") });
    m_axisCompCalRangeTable->verticalHeader()->setVisible(false);
    m_axisCompCalRangeTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_axisCompCalRangeTable->setSelectionMode(QAbstractItemView::NoSelection);
    m_axisCompCalRangeTable->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    m_axisCompCalRangeTable->setMaximumHeight(128);
    const auto installRangeSpin = [&](int row, int col, double value, double minimum, double maximum, double step) {
        auto* spin = new QDoubleSpinBox(m_axisCompCalRangeTable);
        spin->setRange(minimum, maximum);
        spin->setDecimals(3);
        spin->setSingleStep(step);
        spin->setValue(value);
        spin->setKeyboardTracking(false);
        m_axisCompCalRangeTable->setCellWidget(row, col, spin);
        };
    const int linearAxes[] = { AxisX, AxisY, AxisZ };
    for (int row = 0; row < 3; ++row) {
        auto* axisItem = new QTableWidgetItem(axisName5D(linearAxes[row]));
        axisItem->setTextAlignment(Qt::AlignCenter);
        m_axisCompCalRangeTable->setItem(row, 0, axisItem);
        installRangeSpin(row, 1, -20.0, -100000.0, 100000.0, 1.0);
        installRangeSpin(row, 2, 20.0, -100000.0, 100000.0, 1.0);
        installRangeSpin(row, 3, 0.5, 0.05, 1000.0, 0.5);
    }
    axisCompLayout->addWidget(m_axisCompCalRangeTable);

    auto* axisCompRangeButtons = new QHBoxLayout;
    m_btnAxisCompCalUseSoftLimits = new QPushButton(tr("按当前软限位填充安全范围"), axisCompGroup);
    m_btnAxisCompCalUseSoftLimits->setToolTip(
        tr("读取当前XYZ软限位，并在两端自动留出“换向预加载+一个采样步长”的安全余量。请仍人工确认机械/工装安全。"));
    axisCompRangeButtons->addWidget(m_btnAxisCompCalUseSoftLimits);
    axisCompRangeButtons->addStretch();
    axisCompLayout->addLayout(axisCompRangeButtons);

    // V27.6.33: locally qualify a stable two-stage motion profile before collecting a table.
    auto* speedGroup = new QGroupBox(tr("采表前轴速度资格测试（自动定位 + 精密最终接近）"), axisCompGroup);
    auto* speedLayout = new QVBoxLayout(speedGroup);
    speedLayout->setSpacing(4);

    auto* speedTop1 = new QHBoxLayout;
    speedTop1->addWidget(new QLabel(tr("精密速度候选(mm/s):"), speedGroup));
    m_editAxisCompSpeedPrecisionCandidates = new QLineEdit(QStringLiteral("0.5, 1, 2, 3"), speedGroup);
    m_editAxisCompSpeedPrecisionCandidates->setToolTip(
        tr("仅测试这里明确给出的候选值，并从高到低测试；第一个满足稳定性门限的速度即被选为精密最终接近速度，不再继续测试更低速度。"));
    speedTop1->addWidget(m_editAxisCompSpeedPrecisionCandidates, 1);
    auto* coarseStrategy = new QLabel(
        tr("自动定位速度：先测试各轴“测速上限”；若不合格则自动减半，直到合格或降到精密速度。"), speedGroup);
    coarseStrategy->setWordWrap(true);
    coarseStrategy->setStyleSheet("QLabel { color: #555; font-size: 9px; }");
    speedTop1->addWidget(coarseStrategy, 2);
    speedLayout->addLayout(speedTop1);

    auto* speedTop2 = new QHBoxLayout;
    speedTop2->addWidget(new QLabel(tr("无重点位置时代表点:"), speedGroup));
    m_spinAxisCompSpeedTestPoints = new QSpinBox(speedGroup);
    m_spinAxisCompSpeedTestPoints->setRange(1, 3);
    m_spinAxisCompSpeedTestPoints->setValue(1);
    m_spinAxisCompSpeedTestPoints->setToolTip(
        tr("如果下表已填写“重点位置”，测速只使用重点位置；未填写时才在采表范围内部选择1~3个代表点，不再使用最负端/最正端。"));
    speedTop2->addWidget(m_spinAxisCompSpeedTestPoints);
    speedTop2->addWidget(new QLabel(tr("每候选重复:"), speedGroup));
    m_spinAxisCompSpeedRepeats = new QSpinBox(speedGroup);
    m_spinAxisCompSpeedRepeats->setRange(2, 5);
    m_spinAxisCompSpeedRepeats->setValue(2);
    speedTop2->addWidget(m_spinAxisCompSpeedRepeats);
    speedTop2->addWidget(new QLabel(tr("P95漂移≤"), speedGroup));
    m_spinAxisCompSpeedP95DriftUm = new QDoubleSpinBox(speedGroup);
    m_spinAxisCompSpeedP95DriftUm->setRange(0.2, 50.0);
    m_spinAxisCompSpeedP95DriftUm->setDecimals(2);
    m_spinAxisCompSpeedP95DriftUm->setValue(2.0);
    m_spinAxisCompSpeedP95DriftUm->setSuffix(QStringLiteral(" µm"));
    speedTop2->addWidget(m_spinAxisCompSpeedP95DriftUm);
    speedTop2->addWidget(new QLabel(tr("Max漂移≤"), speedGroup));
    m_spinAxisCompSpeedMaxDriftUm = new QDoubleSpinBox(speedGroup);
    m_spinAxisCompSpeedMaxDriftUm->setRange(0.5, 100.0);
    m_spinAxisCompSpeedMaxDriftUm->setDecimals(2);
    m_spinAxisCompSpeedMaxDriftUm->setValue(5.0);
    m_spinAxisCompSpeedMaxDriftUm->setSuffix(QStringLiteral(" µm"));
    speedTop2->addWidget(m_spinAxisCompSpeedMaxDriftUm);
    speedTop2->addWidget(new QLabel(tr("重复性σRMS≤"), speedGroup));
    m_spinAxisCompSpeedRepeatabilityUm = new QDoubleSpinBox(speedGroup);
    m_spinAxisCompSpeedRepeatabilityUm->setRange(0.2, 50.0);
    m_spinAxisCompSpeedRepeatabilityUm->setDecimals(2);
    m_spinAxisCompSpeedRepeatabilityUm->setValue(2.0);
    m_spinAxisCompSpeedRepeatabilityUm->setSuffix(QStringLiteral(" µm"));
    speedTop2->addWidget(m_spinAxisCompSpeedRepeatabilityUm);
    speedTop2->addStretch();
    speedLayout->addLayout(speedTop2);

    m_axisCompSpeedTable = new QTableWidget(3, 8, speedGroup);
    m_axisCompSpeedTable->setHorizontalHeaderLabels({
        tr("轴"), tr("测速上限(mm/s)"), tr("重点位置(mm,逗号)"), tr("自动定位(mm/s)"), tr("精密接近(mm/s)"),
        tr("P95漂移(µm)"), tr("重复性σRMS(µm)"), tr("状态")
        });
    m_axisCompSpeedTable->verticalHeader()->setVisible(false);
    m_axisCompSpeedTable->setSelectionMode(QAbstractItemView::NoSelection);
    m_axisCompSpeedTable->setMaximumHeight(125);
    m_axisCompSpeedTable->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    m_axisCompSpeedTable->horizontalHeader()->setStretchLastSection(true);
    for (int row = 0; row < 3; ++row) {
        const int axis = AxisX + row;
        auto* item = new QTableWidgetItem(axisName5D(axis));
        item->setTextAlignment(Qt::AlignCenter);
        item->setFlags(item->flags() & ~Qt::ItemIsEditable);
        m_axisCompSpeedTable->setItem(row, 0, item);
        auto* maxSpeed = new QDoubleSpinBox(m_axisCompSpeedTable);
        maxSpeed->setRange(0.05, 1000.0);
        maxSpeed->setDecimals(3);
        maxSpeed->setSingleStep(0.5);
        maxSpeed->setValue(std::max(0.05, static_cast<double>(m_axisParams[axis].speed)));
        maxSpeed->setToolTip(tr("测速不会超过该速度。建议填写该轴机械上已确认安全的正常自动定位速度上限，而不是临时降速后的值。"));
        m_axisCompSpeedTable->setCellWidget(row, 1, maxSpeed);
        auto* focus = new QLineEdit(m_axisCompSpeedTable);
        focus->setPlaceholderText(tr("可选，例如 78.5"));
        focus->setToolTip(tr("填写已知共振/可疑位置；只要这里非空，测速就只使用这些位置，不再自动加入采表端点或其它均匀点。"));
        m_axisCompSpeedTable->setCellWidget(row, 2, focus);
        for (int col = 3; col < 8; ++col) {
            auto* out = new QTableWidgetItem(QStringLiteral("—"));
            out->setFlags(out->flags() & ~Qt::ItemIsEditable);
            out->setTextAlignment(Qt::AlignCenter);
            m_axisCompSpeedTable->setItem(row, col, out);
        }
    }
    speedLayout->addWidget(m_axisCompSpeedTable);

    auto* speedActions = new QHBoxLayout;
    m_btnAxisCompSpeedStart = new QPushButton(tr("自动测定选中轴采表速度"), speedGroup);
    m_btnAxisCompSpeedStart->setToolTip(
        tr("先寻找最快稳定的精密最终接近速度，再在该精密速度下寻找最快稳定的自动/粗定位速度。测速不使用任何补偿。"));
    m_btnAxisCompSpeedStop = new QPushButton(tr("停止测速"), speedGroup);
    m_btnAxisCompSpeedStop->setEnabled(false);
    speedActions->addWidget(m_btnAxisCompSpeedStart);
    speedActions->addWidget(m_btnAxisCompSpeedStop);
    speedActions->addStretch();
    speedLayout->addLayout(speedActions);

    m_progressAxisCompSpeed = new QProgressBar(speedGroup);
    m_progressAxisCompSpeed->setRange(0, 100);
    m_progressAxisCompSpeed->setValue(0);
    m_progressAxisCompSpeed->setFormat(tr("采表速度资格测试 %p%"));
    speedLayout->addWidget(m_progressAxisCompSpeed);
    m_lblAxisCompSpeedStatus = new QLabel(
        tr("请先对准备建表的轴完成局部测速。优先填写已知共振/可疑位置；不填写时只使用当前位置/内部代表点。精密候选从高到低，首个合格即停止。"),
        speedGroup);
    m_lblAxisCompSpeedStatus->setWordWrap(true);
    m_lblAxisCompSpeedStatus->setStyleSheet("QLabel { color: #555; font-size: 9px; }");
    speedLayout->addWidget(m_lblAxisCompSpeedStatus);
    axisCompLayout->addWidget(speedGroup);

    auto* axisCompOutput = new QHBoxLayout;
    axisCompOutput->addWidget(new QLabel(tr("保存目录:"), axisCompGroup));
    m_editAxisCompCalOutputDir = new QLineEdit(
        QDir(QCoreApplication::applicationDirPath()).filePath(QStringLiteral("calibration/axis_compensation")),
        axisCompGroup);
    m_editAxisCompCalOutputDir->setToolTip(tr("每次采表会建立独立时间戳子目录，保存原始CSV、X/Y/Z候选补偿表和manifest.json。"));
    axisCompOutput->addWidget(m_editAxisCompCalOutputDir, 1);
    m_btnAxisCompCalBrowseDir = new QPushButton(tr("浏览..."), axisCompGroup);
    axisCompOutput->addWidget(m_btnAxisCompCalBrowseDir);
    axisCompLayout->addLayout(axisCompOutput);

    auto* axisCompActions = new QHBoxLayout;
    m_btnAxisCompCalStart = new QPushButton(tr("开始XYZ补偿表采集"), axisCompGroup);
    m_btnAxisCompCalStop = new QPushButton(tr("停止采集"), axisCompGroup);
    m_btnAxisCompCalStop->setEnabled(false);
    m_btnAxisCompCalSave = new QPushButton(tr("保存当前采表结果"), axisCompGroup);
    m_btnAxisCompCalSave->setEnabled(false);
    axisCompActions->addWidget(m_btnAxisCompCalStart);
    axisCompActions->addWidget(m_btnAxisCompCalStop);
    axisCompActions->addWidget(m_btnAxisCompCalSave);
    axisCompActions->addStretch();
    axisCompLayout->addLayout(axisCompActions);

    m_progressAxisCompCal = new QProgressBar(axisCompGroup);
    m_progressAxisCompCal->setRange(0, 100);
    m_progressAxisCompCal->setValue(0);
    m_progressAxisCompCal->setFormat(tr("补偿表采集 %p%"));
    axisCompLayout->addWidget(m_progressAxisCompCal);

    m_lblAxisCompCalStatus = new QLabel(
        tr("正式采表必须在XYZ回零并完成采表速度资格测试后进行；采集期间不调用SetMpos、不启用任何位置补偿。"
            "每个节点先用自动定位速度到反侧预加载点，再用精密最终接近速度到目标；正反向独立建表。"),
        axisCompGroup);
    m_lblAxisCompCalStatus->setWordWrap(true);
    m_lblAxisCompCalStatus->setStyleSheet("QLabel { color: #555; font-size: 9px; }");
    axisCompLayout->addWidget(m_lblAxisCompCalStatus);

    m_axisCompCalSummaryTable = new QTableWidget(0, 8, axisCompGroup);
    m_axisCompCalSummaryTable->setHorizontalHeaderLabels({
        tr("轴"), tr("方向"), tr("名义DPOS"), tr("平均MPOS"), tr("修正DPOS-MPOS"),
        tr("σ修正"), tr("平均IDLE后漂移"), tr("样本数")
        });
    m_axisCompCalSummaryTable->verticalHeader()->setVisible(false);
    m_axisCompCalSummaryTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_axisCompCalSummaryTable->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_axisCompCalSummaryTable->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    m_axisCompCalSummaryTable->horizontalHeader()->setStretchLastSection(true);
    m_axisCompCalSummaryTable->setMinimumHeight(130);
    m_axisCompCalSummaryTable->setMaximumHeight(220);
    axisCompLayout->addWidget(m_axisCompCalSummaryTable);

    connect(m_btnAxisCompCalUseSoftLimits, &QPushButton::clicked, this, [this]() { fillAxisCompensationRangesFromSoftLimits(); });
    connect(m_btnAxisCompSpeedStart, &QPushButton::clicked, this, [this]() { startAxisCompensationSpeedQualification(); });
    connect(m_btnAxisCompSpeedStop, &QPushButton::clicked, this, [this]() { stopAxisCompensationSpeedQualification(); });
    const auto invalidateSpeedProfiles = [this]() {
        if (m_axisCompSpeedRunning) return;
        bool changed = false;
        for (int axis = AxisX; axis <= AxisZ; ++axis) {
            if (m_axisCompSpeedProfiles[axis].qualified || !m_axisCompSpeedProfiles[axis].status.isEmpty()) {
                m_axisCompSpeedProfiles[axis].qualified = false;
                m_axisCompSpeedProfiles[axis].status = tr("参数已改变，请重新测速");
                changed = true;
            }
        }
        if (changed) updateAxisCompensationSpeedProfileTable();
        };
    connect(m_editAxisCompSpeedPrecisionCandidates, &QLineEdit::textChanged, this, [invalidateSpeedProfiles](const QString&) { invalidateSpeedProfiles(); });
    connect(m_spinAxisCompSpeedTestPoints, qOverload<int>(&QSpinBox::valueChanged), this, [invalidateSpeedProfiles](int) { invalidateSpeedProfiles(); });
    connect(m_spinAxisCompSpeedRepeats, qOverload<int>(&QSpinBox::valueChanged), this, [invalidateSpeedProfiles](int) { invalidateSpeedProfiles(); });
    connect(m_spinAxisCompSpeedP95DriftUm, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [invalidateSpeedProfiles](double) { invalidateSpeedProfiles(); });
    connect(m_spinAxisCompSpeedMaxDriftUm, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [invalidateSpeedProfiles](double) { invalidateSpeedProfiles(); });
    connect(m_spinAxisCompSpeedRepeatabilityUm, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [invalidateSpeedProfiles](double) { invalidateSpeedProfiles(); });
    connect(m_spinAxisCompCalSettleMs, qOverload<int>(&QSpinBox::valueChanged), this, [invalidateSpeedProfiles](int) { invalidateSpeedProfiles(); });
    connect(m_spinAxisCompCalPreloadMm, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [invalidateSpeedProfiles](double) { invalidateSpeedProfiles(); });
    for (int row = 0; row < 3; ++row) {
        for (int col = 1; col <= 2; ++col) {
            if (auto* spin = qobject_cast<QDoubleSpinBox*>(m_axisCompCalRangeTable->cellWidget(row, col))) {
                connect(spin, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [invalidateSpeedProfiles](double) { invalidateSpeedProfiles(); });
            }
        }
        if (auto* maxSpeed = qobject_cast<QDoubleSpinBox*>(m_axisCompSpeedTable->cellWidget(row, 1))) {
            connect(maxSpeed, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [invalidateSpeedProfiles](double) { invalidateSpeedProfiles(); });
        }
        if (auto* focus = qobject_cast<QLineEdit*>(m_axisCompSpeedTable->cellWidget(row, 2))) {
            connect(focus, &QLineEdit::textChanged, this, [invalidateSpeedProfiles](const QString&) { invalidateSpeedProfiles(); });
        }
    }
    connect(m_btnAxisCompCalBrowseDir, &QPushButton::clicked, this, [this]() {
        const QString initial = m_editAxisCompCalOutputDir ? m_editAxisCompCalOutputDir->text() : QDir::homePath();
        const QString dir = QFileDialog::getExistingDirectory(this, tr("选择XYZ补偿表保存目录"), initial);
        if (!dir.isEmpty() && m_editAxisCompCalOutputDir) m_editAxisCompCalOutputDir->setText(dir);
        });
    connect(m_btnAxisCompCalStart, &QPushButton::clicked, this, [this]() { startAxisCompensationCalibration(); });
    connect(m_btnAxisCompCalStop, &QPushButton::clicked, this, [this]() { stopAxisCompensationCalibration(); });
    connect(m_btnAxisCompCalSave, &QPushButton::clicked, this, [this]() { saveAxisCompensationCalibrationFilesInteractive(); });
    connect(m_comboAxisCompCalAxis, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int) { updateAxisCompensationCalibrationUi(); });
    updateAxisCompensationCalibrationUi();

    layout->addWidget(axisCompGroup);

    // ========== V27.6.34 候选补偿表双方法 A/B 独立验证 ==========
    // 旧式表（V27.6.30/Z）保持逐点preload验证；V27.6.33+单向SWEEP表（X/Y）
    // 默认自动切换为 PRIME once + monotonic off-grid sweep 验证。两种方法都不会开启runtime补偿。
    auto* axisCompValGroup = new QGroupBox(tr("候选补偿表 A/B 验证（自动适配旧式逐点 / 新式单向SWEEP）"), widget);
    auto* axisCompValLayout = new QVBoxLayout(axisCompValGroup);
    axisCompValLayout->setSpacing(5);

    auto* axisCompValFile = new QHBoxLayout;
    axisCompValFile->addWidget(new QLabel(tr("候选表:"), axisCompValGroup));
    m_editAxisCompValTablePath = new QLineEdit(axisCompValGroup);
    m_editAxisCompValTablePath->setReadOnly(true);
    m_editAxisCompValTablePath->setPlaceholderText(tr("请选择 X/Y/Z_compensation_candidate.csv"));
    axisCompValFile->addWidget(m_editAxisCompValTablePath, 1);
    m_btnAxisCompValLoad = new QPushButton(tr("加载候选表..."), axisCompValGroup);
    axisCompValFile->addWidget(m_btnAxisCompValLoad);
    axisCompValLayout->addLayout(axisCompValFile);

    m_lblAxisCompValTableInfo = new QLabel(tr("尚未加载候选补偿表。"), axisCompValGroup);
    m_lblAxisCompValTableInfo->setWordWrap(true);
    m_lblAxisCompValTableInfo->setStyleSheet("QLabel { color: #555; font-size: 9px; }");
    axisCompValLayout->addWidget(m_lblAxisCompValTableInfo);

    auto* axisCompValParams = new QHBoxLayout;
    axisCompValParams->addWidget(new QLabel(tr("验证方法:"), axisCompValGroup));
    m_comboAxisCompValMethod = new QComboBox(axisCompValGroup);
    m_comboAxisCompValMethod->addItem(tr("自动（按候选表manifest推荐）"), AxisCompValAuto);
    m_comboAxisCompValMethod->addItem(tr("旧式：逐点2mm预加载（V27.6.30 / Z旧表）"), AxisCompValPointToPoint);
    m_comboAxisCompValMethod->addItem(tr("新式：PRIME一次 + 单向连续SWEEP（V27.6.33+ / X/Y）"), AxisCompValMonotonicSweep);
    m_comboAxisCompValMethod->setToolTip(tr("自动模式读取候选表同目录axis_compensation_manifest.json：V27.6.33+ monotonic采表自动使用单向SWEEP验证；旧表自动保持逐点预加载验证。"));
    axisCompValParams->addWidget(m_comboAxisCompValMethod);

    axisCompValParams->addWidget(new QLabel(tr("验证方向:"), axisCompValGroup));
    m_comboAxisCompValDirection = new QComboBox(axisCompValGroup);
    m_comboAxisCompValDirection->addItem(tr("正向+反向"), 0);
    m_comboAxisCompValDirection->addItem(tr("仅正向"), +1);
    m_comboAxisCompValDirection->addItem(tr("仅反向"), -1);
    axisCompValParams->addWidget(m_comboAxisCompValDirection);

    axisCompValParams->addWidget(new QLabel(tr("独立验证点:"), axisCompValGroup));
    m_spinAxisCompValPoints = new QSpinBox(axisCompValGroup);
    m_spinAxisCompValPoints->setRange(6, 600);
    m_spinAxisCompValPoints->setValue(36);
    m_spinAxisCompValPoints->setToolTip(tr("验证点取相邻建表节点的中点，不与建表节点重合。新式SWEEP验证会按方向一次PRIME后连续通过这些off-grid点；点数越多越接近原0.5mm sweep机械状态。"));
    axisCompValParams->addWidget(m_spinAxisCompValPoints);

    axisCompValParams->addWidget(new QLabel(tr("重复:"), axisCompValGroup));
    m_spinAxisCompValRepeats = new QSpinBox(axisCompValGroup);
    m_spinAxisCompValRepeats->setRange(1, 10);
    m_spinAxisCompValRepeats->setValue(3);
    axisCompValParams->addWidget(m_spinAxisCompValRepeats);

    axisCompValParams->addWidget(new QLabel(tr("稳定等待:"), axisCompValGroup));
    m_spinAxisCompValSettleMs = new QSpinBox(axisCompValGroup);
    m_spinAxisCompValSettleMs->setRange(0, 10000);
    m_spinAxisCompValSettleMs->setValue(2000);
    m_spinAxisCompValSettleMs->setSingleStep(100);
    m_spinAxisCompValSettleMs->setSuffix(tr(" ms"));
    axisCompValParams->addWidget(m_spinAxisCompValSettleMs);

    axisCompValParams->addWidget(new QLabel(tr("最终接近预加载:"), axisCompValGroup));
    m_spinAxisCompValPreloadMm = new QDoubleSpinBox(axisCompValGroup);
    m_spinAxisCompValPreloadMm->setRange(0.1, 20.0);
    m_spinAxisCompValPreloadMm->setDecimals(3);
    m_spinAxisCompValPreloadMm->setValue(2.0);
    m_spinAxisCompValPreloadMm->setSuffix(tr(" mm"));
    axisCompValParams->addWidget(m_spinAxisCompValPreloadMm);
    axisCompValParams->addStretch();
    axisCompValLayout->addLayout(axisCompValParams);

    auto* axisCompValOutput = new QHBoxLayout;
    axisCompValOutput->addWidget(new QLabel(tr("验证结果目录:"), axisCompValGroup));
    m_editAxisCompValOutputDir = new QLineEdit(
        QDir(QCoreApplication::applicationDirPath()).filePath(QStringLiteral("calibration/axis_compensation_validation")),
        axisCompValGroup);
    axisCompValOutput->addWidget(m_editAxisCompValOutputDir, 1);
    m_btnAxisCompValBrowseDir = new QPushButton(tr("浏览..."), axisCompValGroup);
    axisCompValOutput->addWidget(m_btnAxisCompValBrowseDir);
    axisCompValLayout->addLayout(axisCompValOutput);

    auto* axisCompValActions = new QHBoxLayout;
    m_btnAxisCompValStart = new QPushButton(tr("开始 A/B 验证"), axisCompValGroup);
    m_btnAxisCompValStop = new QPushButton(tr("停止验证"), axisCompValGroup);
    m_btnAxisCompValStop->setEnabled(false);
    m_btnAxisCompValSave = new QPushButton(tr("保存当前验证结果"), axisCompValGroup);
    m_btnAxisCompValSave->setEnabled(false);
    axisCompValActions->addWidget(m_btnAxisCompValStart);
    axisCompValActions->addWidget(m_btnAxisCompValStop);
    axisCompValActions->addWidget(m_btnAxisCompValSave);
    axisCompValActions->addStretch();
    axisCompValLayout->addLayout(axisCompValActions);

    m_progressAxisCompVal = new QProgressBar(axisCompValGroup);
    m_progressAxisCompVal->setRange(0, 100);
    m_progressAxisCompVal->setValue(0);
    m_progressAxisCompVal->setFormat(tr("A/B验证 %p%"));
    axisCompValLayout->addWidget(m_progressAxisCompVal);

    m_lblAxisCompValStatus = new QLabel(
        tr("验证规则：RAW直接命令DPOS=目标MPOS；COMPENSATED仅在本次测试中按候选表插值命令DPOS。"
            "两者都使用相同方向预加载、相同2s稳定等待，不做二次MPOS trim。"),
        axisCompValGroup);
    m_lblAxisCompValStatus->setWordWrap(true);
    m_lblAxisCompValStatus->setStyleSheet("QLabel { color: #555; font-size: 9px; }");
    axisCompValLayout->addWidget(m_lblAxisCompValStatus);

    m_axisCompValSummaryTable = new QTableWidget(0, 9, axisCompValGroup);
    m_axisCompValSummaryTable->setHorizontalHeaderLabels({
        tr("方向"), tr("RAW RMS(µm)"), tr("补偿RMS(µm)"), tr("RAW P95(µm)"),
        tr("补偿P95(µm)"), tr("RAW Max(µm)"), tr("补偿Max(µm)"),
        tr("补偿均值(µm)"), tr("RMS改善")
        });
    m_axisCompValSummaryTable->verticalHeader()->setVisible(false);
    m_axisCompValSummaryTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_axisCompValSummaryTable->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    m_axisCompValSummaryTable->horizontalHeader()->setStretchLastSection(true);
    m_axisCompValSummaryTable->setMaximumHeight(150);
    axisCompValLayout->addWidget(m_axisCompValSummaryTable);

    connect(m_btnAxisCompValLoad, &QPushButton::clicked, this, [this]() {
        const QString initial = !m_axisCompValSourcePath.isEmpty() ? m_axisCompValSourcePath
            : (m_editAxisCompCalOutputDir ? m_editAxisCompCalOutputDir->text() : QDir::homePath());
        const QString fileName = QFileDialog::getOpenFileName(
            this, tr("加载候选补偿表"), initial, tr("候选补偿表 (*_compensation_candidate.csv);;CSV文件 (*.csv)"));
        if (fileName.isEmpty()) return;
        QString err;
        if (!loadAxisCompensationValidationTable(fileName, &err)) {
            QMessageBox::warning(this, tr("加载候选表失败"), err);
            return;
        }
        if (m_editAxisCompValOutputDir) {
            m_editAxisCompValOutputDir->setText(QDir(QFileInfo(fileName).absolutePath()).filePath(QStringLiteral("validation")));
        }
        updateAxisCompensationValidationUi();
        });
    connect(m_btnAxisCompValBrowseDir, &QPushButton::clicked, this, [this]() {
        const QString initial = m_editAxisCompValOutputDir ? m_editAxisCompValOutputDir->text() : QDir::homePath();
        const QString dir = QFileDialog::getExistingDirectory(this, tr("选择A/B验证结果目录"), initial);
        if (!dir.isEmpty() && m_editAxisCompValOutputDir) m_editAxisCompValOutputDir->setText(dir);
        });
    connect(m_btnAxisCompValStart, &QPushButton::clicked, this, [this]() { startAxisCompensationValidation(); });
    connect(m_btnAxisCompValStop, &QPushButton::clicked, this, [this]() { stopAxisCompensationValidation(); });
    connect(m_btnAxisCompValSave, &QPushButton::clicked, this, [this]() { saveAxisCompensationValidationFilesInteractive(); });
    connect(m_comboAxisCompValMethod, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int) { updateAxisCompensationValidationUi(); });
    updateAxisCompensationValidationUi();

    layout->addWidget(axisCompValGroup);

    // 收集需要在G代码执行时禁用的控件
    m_controlsToDisable.clear();
    m_controlsToDisable << m_xAxisWidget << m_yAxisWidget << m_zAxisWidget << m_cAxisWidget << m_bAxisWidget;
    m_controlsToDisable << m_btnZeroAll << m_btnHomeAll << m_btnResyncRotaryAbs;
    m_controlsToDisable << m_btnLoadPath << m_btnAddPoint << m_btnRemovePoint;
    m_controlsToDisable << m_btnStartMerge << m_btnStopMerge;
    m_controlsToDisable << m_btnStartCapture << m_btnStopCapture;
    m_controlsToDisable << m_btnClearData << m_btnAnalyzeData;
    m_controlsToDisable << m_btnKinPlan << m_btnKinMove;
    m_controlsToDisable << m_checkAxisMoveDiagAuto << m_comboAxisMoveDiagAutoAxis
        << m_editAxisMoveDiagBasePositions << m_editAxisMoveDiagDistances
        << m_btnAxisMoveDiagManualStart << m_btnAxisMoveDiagManualEnd
        << m_btnAxisMoveDiagAutoStart << m_btnAxisMoveDiagAutoStop;
    m_controlsToDisable << m_comboAxisCompCalAxis << m_axisCompCalRangeTable
        << m_btnAxisCompCalUseSoftLimits << m_editAxisCompCalOutputDir
        << m_editAxisCompSpeedPrecisionCandidates
        << m_spinAxisCompSpeedTestPoints << m_spinAxisCompSpeedRepeats
        << m_spinAxisCompSpeedP95DriftUm << m_spinAxisCompSpeedMaxDriftUm
        << m_spinAxisCompSpeedRepeatabilityUm << m_axisCompSpeedTable
        << m_btnAxisCompSpeedStart << m_btnAxisCompSpeedStop
        << m_btnAxisCompCalBrowseDir << m_btnAxisCompCalStart
        << m_btnAxisCompCalStop << m_btnAxisCompCalSave;
    m_controlsToDisable << m_editAxisCompValTablePath << m_btnAxisCompValLoad
        << m_comboAxisCompValDirection << m_spinAxisCompValPoints
        << m_spinAxisCompValRepeats << m_spinAxisCompValSettleMs
        << m_spinAxisCompValPreloadMm << m_editAxisCompValOutputDir
        << m_btnAxisCompValBrowseDir << m_btnAxisCompValStart
        << m_btnAxisCompValStop << m_btnAxisCompValSave;

    // V27.6.16R: 传感标定使用可调Profile Rate（默认25Hz）+ 自动Idle + 多帧曝光统计采样。
    layout->addStretch();

    return widget;
}

QWidget* MainWindow::createVideoControlSection(QWidget* parent)
{
    m_videoControlGroup = new QGroupBox(tr("scanCONTROL 传感标定 / 调试"), parent);
    auto* layout = new QVBoxLayout(m_videoControlGroup);
    layout->setSpacing(5);
    layout->setContentsMargins(6, 10, 6, 6);

    // Measuring field is still selected only in Parameters -> Sensor Parameters.
    // The calibration page owns separate editable INTERNAL-trigger preview timing.
    // This is intentionally NOT the H1/H3 exposure/Idle or hardware trigger rate.
    m_lblSensorCalibField = new QLabel(m_videoControlGroup);
    m_lblSensorCalibField->setStyleSheet("QLabel { font-weight: bold; color:#245; font-size:11px; }");
    m_lblSensorCalibField->setWordWrap(true);
    layout->addWidget(m_lblSensorCalibField);

    auto* timingInputs = new QGridLayout;
    timingInputs->setHorizontalSpacing(8);
    timingInputs->setVerticalSpacing(5);
    timingInputs->setColumnStretch(3, 1);

    timingInputs->addWidget(new QLabel(tr("轮廓频率:"), m_videoControlGroup), 0, 0);
    m_spinSensorCalibRate = new QSpinBox(m_videoControlGroup);
    m_spinSensorCalibRate->setRange(25, 2000);
    m_spinSensorCalibRate->setValue(25);
    m_spinSensorCalibRate->setSuffix(tr(" Hz"));
    m_spinSensorCalibRate->setMinimumWidth(95);
    m_spinSensorCalibRate->setToolTip(tr("内部触发Video/轮廓预览频率，不能超过参数页所选测量场的参考上限；H1/H3硬件触发频率不变。"));
    timingInputs->addWidget(m_spinSensorCalibRate, 0, 1);
    m_checkSensorCalibAutoFieldRate = new QCheckBox(tr("跟随测量场上限"), m_videoControlGroup);
    m_checkSensorCalibAutoFieldRate->setChecked(true);
    m_checkSensorCalibAutoFieldRate->setToolTip(tr("勾选后使用所选测量场的参考频率上限；取消后可手动设置较低频率。"));
    timingInputs->addWidget(m_checkSensorCalibAutoFieldRate, 0, 2, 1, 2);

    timingInputs->addWidget(new QLabel(tr("曝光 (10µs):"), m_videoControlGroup), 1, 0);
    m_spinVideoExposure = new QSpinBox(m_videoControlGroup);
    m_spinVideoExposure->setRange(1, 100000);
    m_spinVideoExposure->setValue(m_spinExposure ? m_spinExposure->value() : 100);
    m_spinVideoExposure->setMinimumWidth(95);
    m_spinVideoExposure->setToolTip(tr("标定预览专用曝光设置；曝光扫描测试可暂时覆盖，结束后恢复。不会改写H1/H3曝光参数。"));
    timingInputs->addWidget(m_spinVideoExposure, 1, 1);

    timingInputs->addWidget(new QLabel(tr("轮廓Idle (10µs):"), m_videoControlGroup), 1, 2);
    m_spinVideoIdle = new QSpinBox(m_videoControlGroup);
    m_spinVideoIdle->setRange(0, 100000);
    m_spinVideoIdle->setReadOnly(true);
    m_spinVideoIdle->setButtonSymbols(QAbstractSpinBox::NoButtons);
    m_spinVideoIdle->setMinimumWidth(86);
    m_spinVideoIdle->setToolTip(tr("轮廓专用的自动Idle：根据Profile目标频率与曝光计算；Video另用25Hz/40ms时序。"));
    timingInputs->addWidget(m_spinVideoIdle, 1, 3);
    layout->addLayout(timingInputs);

    m_lblSensorCalibTiming = new QLabel(m_videoControlGroup);
    m_lblSensorCalibTiming->setStyleSheet("QLabel { color:#555; font-size:10px; }");
    m_lblSensorCalibTiming->setWordWrap(true);
    layout->addWidget(m_lblSensorCalibTiming);

    // ===== 预览类型选择 =====
    auto* topRow = new QHBoxLayout;
    topRow->setSpacing(8);
    topRow->addWidget(new QLabel(tr("预览:"), m_videoControlGroup));
    m_radioVideoMode = new QRadioButton(tr("Video图像"), m_videoControlGroup);
    m_radioProfilePreviewMode = new QRadioButton(tr("轮廓"), m_videoControlGroup);
    m_radioVideoMode->setChecked(true);
    auto* modeGroup = new QButtonGroup(m_videoControlGroup);
    modeGroup->addButton(m_radioVideoMode, 0);
    modeGroup->addButton(m_radioProfilePreviewMode, 1);
    topRow->addWidget(m_radioVideoMode);
    topRow->addWidget(m_radioProfilePreviewMode);
    topRow->addStretch();
    layout->addLayout(topRow);

    // ===== Video 控制行 =====
    auto* videoRow = new QHBoxLayout;
    videoRow->setSpacing(4);
    auto* videoLabel = new QLabel(tr("Video:"), m_videoControlGroup);
    videoLabel->setFixedWidth(45);
    videoRow->addWidget(videoLabel);
    m_btnStartVideo = new QPushButton(tr("启动"), m_videoControlGroup);
    m_btnStartVideo->setFixedWidth(50);
    m_btnStartVideo->setStyleSheet("QPushButton { background-color: #aaddff; }");
    m_btnStopVideo = new QPushButton(tr("停止"), m_videoControlGroup);
    m_btnStopVideo->setFixedWidth(50);
    videoRow->addWidget(m_btnStartVideo);
    videoRow->addWidget(m_btnStopVideo);
    videoRow->addSpacing(6);
    m_lblVideoStatus = new QLabel(tr("未启动"), m_videoControlGroup);
    m_lblVideoStatus->setStyleSheet("QLabel { font-weight: bold; }");
    videoRow->addWidget(m_lblVideoStatus);
    videoRow->addStretch();
    layout->addLayout(videoRow);

    // ===== 轮廓预览控制行 =====
    auto* profileRow = new QHBoxLayout;
    profileRow->setSpacing(4);
    auto* profileLabel = new QLabel(tr("轮廓:"), m_videoControlGroup);
    profileLabel->setFixedWidth(45);
    profileRow->addWidget(profileLabel);
    m_btnStartProfilePreview = new QPushButton(tr("启动"), m_videoControlGroup);
    m_btnStartProfilePreview->setFixedWidth(50);
    m_btnStartProfilePreview->setStyleSheet("QPushButton { background-color: #aaffaa; }");
    m_btnStopProfilePreview = new QPushButton(tr("停止"), m_videoControlGroup);
    m_btnStopProfilePreview->setFixedWidth(50);
    profileRow->addWidget(m_btnStartProfilePreview);
    profileRow->addWidget(m_btnStopProfilePreview);
    profileRow->addSpacing(6);
    m_lblProfilePreviewStatus = new QLabel(tr("未启动"), m_videoControlGroup);
    m_lblProfilePreviewStatus->setStyleSheet("QLabel { font-weight: bold; }");
    profileRow->addWidget(m_lblProfilePreviewStatus);
    auto* tipLabel = new QLabel(tr("(独立预览用内部触发；采集中跟随正式同步轮廓，只读)"), m_videoControlGroup);
    tipLabel->setStyleSheet("color: gray; font-size: 9px;");
    profileRow->addStretch();
    layout->addLayout(profileRow);
    tipLabel->setWordWrap(true);
    layout->addWidget(tipLabel);

    auto* separator = new QFrame(m_videoControlGroup);
    separator->setFrameShape(QFrame::HLine);
    separator->setFrameShadow(QFrame::Sunken);
    layout->addWidget(separator);

    // ===== 曝光测试区域 =====
    auto* expTestLabel = new QLabel(tr("曝光扫描测试（逐档临时曝光，结束恢复正式参数）"), m_videoControlGroup);
    expTestLabel->setStyleSheet("font-weight: bold; color: #555;");
    layout->addWidget(expTestLabel);

    auto* conditionRow1 = new QHBoxLayout;
    conditionRow1->setSpacing(4);
    conditionRow1->addWidget(new QLabel(tr("光照:"), m_videoControlGroup));
    m_comboLighting = new QComboBox(m_videoControlGroup);
    m_comboLighting->addItems({
        tr("黑箱(无光)"), tr("室内-荧光灯"), tr("室内-LED"), tr("室内-白炽灯"),
        tr("自然光-阴天"), tr("自然光-晴天"), tr("强光-直射"), tr("强光-漫射"),
        tr("混合光照"), tr("其他")
        });
    m_comboLighting->setFixedWidth(100);
    conditionRow1->addWidget(m_comboLighting);
    conditionRow1->addSpacing(8);
    conditionRow1->addWidget(new QLabel(tr("材料:"), m_videoControlGroup));
    m_editMaterial = new QLineEdit(m_videoControlGroup);
    m_editMaterial->setPlaceholderText(tr("输入材料名称"));
    m_editMaterial->setText(tr("Aluminum"));
    m_editMaterial->setFixedWidth(90);
    conditionRow1->addWidget(m_editMaterial);
    conditionRow1->addSpacing(8);
    conditionRow1->addWidget(new QLabel(tr("温度:"), m_videoControlGroup));
    m_spinTemperature = new QDoubleSpinBox(m_videoControlGroup);
    m_spinTemperature->setRange(-50.0, 100.0);
    m_spinTemperature->setValue(25.0);
    m_spinTemperature->setSuffix(tr("°C"));
    m_spinTemperature->setFixedWidth(75);
    conditionRow1->addWidget(m_spinTemperature);
    conditionRow1->addStretch();
    layout->addLayout(conditionRow1);

    // 五轴机床实验位姿：XYZ使用回零后的增量光栅MPOS；C/B使用软件机械MPOS。
    auto makeLinearPos = [this]() {
        auto* s = new QDoubleSpinBox(m_videoControlGroup);
        s->setRange(-9999.0, 9999.0);
        s->setDecimals(3);
        s->setSuffix(tr(" mm"));
        s->setFixedWidth(86);
        return s;
        };
    auto makeRotaryPos = [this]() {
        auto* s = new QDoubleSpinBox(m_videoControlGroup);
        s->setRange(-3600.0, 3600.0);
        s->setDecimals(3);
        s->setSuffix(QString::fromUtf8(" °"));
        s->setFixedWidth(82);
        return s;
        };

    auto* xyzRow = new QHBoxLayout;
    xyzRow->setSpacing(4);
    xyzRow->addWidget(new QLabel(tr("机床位姿 X:"), m_videoControlGroup));
    m_spinPosX = makeLinearPos(); xyzRow->addWidget(m_spinPosX);
    xyzRow->addWidget(new QLabel(tr("Y:"), m_videoControlGroup));
    m_spinPosY = makeLinearPos(); xyzRow->addWidget(m_spinPosY);
    xyzRow->addWidget(new QLabel(tr("Z:"), m_videoControlGroup));
    m_spinPosZ = makeLinearPos(); xyzRow->addWidget(m_spinPosZ);
    xyzRow->addStretch();
    layout->addLayout(xyzRow);

    auto* cbRow = new QHBoxLayout;
    cbRow->setSpacing(4);
    cbRow->addWidget(new QLabel(tr("C:"), m_videoControlGroup));
    m_spinPosC = makeRotaryPos(); cbRow->addWidget(m_spinPosC);
    cbRow->addWidget(new QLabel(tr("B:"), m_videoControlGroup));
    m_spinPosB = makeRotaryPos(); cbRow->addWidget(m_spinPosB);
    m_btnReadPosition = new QPushButton(tr("读取当前XYZCB"), m_videoControlGroup);
    m_btnReadPosition->setToolTip(tr("XYZ读取回零后的光栅MPOS；C/B读取软件机械MPOS，与Physical17全MPOS标定坐标约定一致"));
    cbRow->addWidget(m_btnReadPosition);
    auto* poseSourceTip = new QLabel(tr("XYZ=光栅MPOS，C/B=机械MPOS"), m_videoControlGroup);
    poseSourceTip->setStyleSheet("color: gray; font-size: 9px;");
    cbRow->addWidget(poseSourceTip);
    cbRow->addStretch();
    layout->addLayout(cbRow);

    auto* expRangeRow = new QHBoxLayout;
    expRangeRow->setSpacing(4);
    expRangeRow->addWidget(new QLabel(tr("曝光:"), m_videoControlGroup));
    m_spinExpStart = new QSpinBox(m_videoControlGroup);
    m_spinExpStart->setRange(1, 4000);
    m_spinExpStart->setValue(50);
    m_spinExpStart->setFixedWidth(60);
    expRangeRow->addWidget(m_spinExpStart);
    expRangeRow->addWidget(new QLabel(tr("~"), m_videoControlGroup));
    m_spinExpEnd = new QSpinBox(m_videoControlGroup);
    m_spinExpEnd->setRange(1, 4000);
    m_spinExpEnd->setValue(500);
    m_spinExpEnd->setFixedWidth(60);
    expRangeRow->addWidget(m_spinExpEnd);
    expRangeRow->addWidget(new QLabel(tr("步长:"), m_videoControlGroup));
    m_spinExpStep = new QSpinBox(m_videoControlGroup);
    m_spinExpStep->setRange(1, 1000);
    m_spinExpStep->setValue(50);
    m_spinExpStep->setFixedWidth(50);
    expRangeRow->addWidget(m_spinExpStep);
    expRangeRow->addWidget(new QLabel(tr("稳定丢弃:"), m_videoControlGroup));
    m_spinStableDiscardFrames = new QSpinBox(m_videoControlGroup);
    m_spinStableDiscardFrames->setRange(0, 20);
    m_spinStableDiscardFrames->setValue(3);
    m_spinStableDiscardFrames->setSuffix(tr(" 帧"));
    m_spinStableDiscardFrames->setFixedWidth(72);
    m_spinStableDiscardFrames->setToolTip(tr("每次切换到Video或Profile后，先等待并丢弃这么多个真实新帧，再开始保存；默认3帧。"));
    expRangeRow->addWidget(m_spinStableDiscardFrames);
    expRangeRow->addStretch();
    layout->addLayout(expRangeRow);

    auto* saveOptionRow = new QHBoxLayout;
    saveOptionRow->setSpacing(4);
    saveOptionRow->addWidget(new QLabel(tr("保存:"), m_videoControlGroup));
    m_checkSaveProfile = new QCheckBox(tr("轮廓数据"), m_videoControlGroup);
    m_checkSaveProfile->setChecked(true);
    saveOptionRow->addWidget(m_checkSaveProfile);
    saveOptionRow->addWidget(new QLabel(tr("数量:"), m_videoControlGroup));
    m_spinProfileFramesPerExposure = new QSpinBox(m_videoControlGroup);
    m_spinProfileFramesPerExposure->setRange(1, 200);
    m_spinProfileFramesPerExposure->setValue(20);
    m_spinProfileFramesPerExposure->setSuffix(tr(" 条/档"));
    m_spinProfileFramesPerExposure->setFixedWidth(82);
    m_spinProfileFramesPerExposure->setToolTip(tr("每个曝光档位保存的独立Profile数量；默认20条，用于统计重复性/噪声。"));
    saveOptionRow->addWidget(m_spinProfileFramesPerExposure);
    saveOptionRow->addSpacing(8);
    m_checkSaveVideo = new QCheckBox(tr("Video图像"), m_videoControlGroup);
    m_checkSaveVideo->setChecked(true);
    saveOptionRow->addWidget(m_checkSaveVideo);
    saveOptionRow->addWidget(new QLabel(tr("数量:"), m_videoControlGroup));
    m_spinVideoFramesPerExposure = new QSpinBox(m_videoControlGroup);
    m_spinVideoFramesPerExposure->setRange(1, 50);
    m_spinVideoFramesPerExposure->setValue(3);
    m_spinVideoFramesPerExposure->setSuffix(tr(" 张/档"));
    m_spinVideoFramesPerExposure->setFixedWidth(82);
    m_spinVideoFramesPerExposure->setToolTip(tr("每个曝光档位保存的独立Video帧数量；默认3张。"));
    saveOptionRow->addWidget(m_spinVideoFramesPerExposure);
    connect(m_checkSaveProfile, &QCheckBox::toggled, this, [this](bool checked) {
        if (m_spinProfileFramesPerExposure && m_expTestState == ExpTestState::Idle) {
            m_spinProfileFramesPerExposure->setEnabled(checked);
        }
        });
    connect(m_checkSaveVideo, &QCheckBox::toggled, this, [this](bool checked) {
        if (m_spinVideoFramesPerExposure && m_expTestState == ExpTestState::Idle) {
            m_spinVideoFramesPerExposure->setEnabled(checked);
        }
        });
    saveOptionRow->addStretch();
    layout->addLayout(saveOptionRow);

    auto* dirRow = new QHBoxLayout;
    dirRow->setSpacing(4);
    dirRow->addWidget(new QLabel(tr("目录:"), m_videoControlGroup));
    m_editExpTestDir = new QLineEdit(m_videoControlGroup);
    m_editExpTestDir->setPlaceholderText(tr("选择保存目录..."));
    m_editExpTestDir->setText(QDir::currentPath() + "/exposure_test");
    m_editExpTestDir->setReadOnly(true);
    dirRow->addWidget(m_editExpTestDir, 1);
    auto* btnBrowse = new QPushButton(tr("..."), m_videoControlGroup);
    btnBrowse->setFixedWidth(30);
    connect(btnBrowse, &QPushButton::clicked, this, [this]() {
        const QString dir = QFileDialog::getExistingDirectory(
            this, tr("选择基础保存目录"), m_editExpTestDir->text());
        if (!dir.isEmpty()) m_editExpTestDir->setText(dir);
        });
    dirRow->addWidget(btnBrowse);
    layout->addLayout(dirRow);

    auto* testCtrlRow = new QHBoxLayout;
    testCtrlRow->setSpacing(4);
    m_btnStartExpTest = new QPushButton(tr("开始测试"), m_videoControlGroup);
    m_btnStartExpTest->setStyleSheet("QPushButton { background-color: #ffddaa; }");
    m_btnStopExpTest = new QPushButton(tr("停止测试"), m_videoControlGroup);
    m_btnAnalyzeExpTestDir = new QPushButton(tr("分析/导入已有数据"), m_videoControlGroup);
    m_btnAnalyzeExpTestDir->setToolTip(tr("选择已有sensor_calibration_session目录，分析Profile曝光扫频并写入材料曝光库。兼容当前v4和历史v3数据。"));
    testCtrlRow->addWidget(m_btnStartExpTest);
    testCtrlRow->addWidget(m_btnStopExpTest);
    testCtrlRow->addWidget(m_btnAnalyzeExpTestDir);
    testCtrlRow->addSpacing(10);
    m_lblExpTestStatus = new QLabel(tr("空闲"), m_videoControlGroup);
    m_lblExpTestStatus->setStyleSheet("QLabel { font-weight: bold; }");
    testCtrlRow->addWidget(m_lblExpTestStatus);
    testCtrlRow->addStretch();
    layout->addLayout(testCtrlRow);

    m_lblExpAnalysisResult = new QLabel(tr("分析结果：尚未分析；完成曝光测试后将自动生成 exposure_analysis.csv/json 并更新材料库。"), m_videoControlGroup);
    m_lblExpAnalysisResult->setWordWrap(true);
    m_lblExpAnalysisResult->setStyleSheet("color:#345; font-size:9px;");
    layout->addWidget(m_lblExpAnalysisResult);

    // Independent moving-X whole-sphere exposure experiment. Existing fixed-pose
    // full-measuring-field exposure test above is intentionally unchanged.
    auto* sphereExposureGroup = new QGroupBox(tr("标准球曝光测试（软件触发 / 所选测量场 / 仅X运动）"), m_videoControlGroup);
    auto* seLayout = new QVBoxLayout(sphereExposureGroup);
    auto* seHelp = new QLabel(tr(
        "步骤：在下方球面研究中导入机床坐标点云并RANSAC拟合球心；随后规划X轴扫过整球。"
        "逐曝光使用上方相同的起止值/步长及Profile频率，保存每条原始二维X/Z轮廓。"
        "B/C/Y/Z锁定，执行前必须通过Physical17、MachineSafety和到位检查；"
        "仅供曝光比较，不进行H1/H3时间标定、不自动写入材料库。"), sphereExposureGroup);
    seHelp->setWordWrap(true);
    seLayout->addWidget(seHelp);
    auto* seParams = new QGridLayout;
    seParams->addWidget(new QLabel(tr("X扫描速度"),sphereExposureGroup),0,0);
    m_spinSphereExpSpeed = new QDoubleSpinBox(sphereExposureGroup);
    m_spinSphereExpSpeed->setRange(0.1,10.0); m_spinSphereExpSpeed->setDecimals(2);
    m_spinSphereExpSpeed->setValue(1.0); m_spinSphereExpSpeed->setSuffix(tr(" mm/s"));
    seParams->addWidget(m_spinSphereExpSpeed,0,1);
    seParams->addWidget(new QLabel(tr("扫描起点定位速度"),sphereExposureGroup),2,0);
    m_comboSphereExpPositionSpeed=new QComboBox(sphereExposureGroup);
    for(int v=1;v<=5;++v)m_comboSphereExpPositionSpeed->addItem(tr("%1 mm/s").arg(v),v);
    m_comboSphereExpPositionSpeed->setCurrentIndex(2); // 3 mm/s, independent of scan rate
    seParams->addWidget(m_comboSphereExpPositionSpeed,2,1);

    seParams->addWidget(new QLabel(tr("球两侧余量"),sphereExposureGroup),0,2);
    m_spinSphereExpMargin = new QDoubleSpinBox(sphereExposureGroup);
    m_spinSphereExpMargin->setRange(0.2,3.0);m_spinSphereExpMargin->setDecimals(2);
    m_spinSphereExpMargin->setValue(0.8);m_spinSphereExpMargin->setSuffix(tr(" mm"));
    seParams->addWidget(m_spinSphereExpMargin,0,3);
    seParams->addWidget(new QLabel(tr("最大帧间距"),sphereExposureGroup),1,0);
    m_spinSphereExpMaxStep = new QDoubleSpinBox(sphereExposureGroup);
    m_spinSphereExpMaxStep->setRange(0.01,0.08);m_spinSphereExpMaxStep->setDecimals(3);
    m_spinSphereExpMaxStep->setValue(0.05);m_spinSphereExpMaxStep->setSuffix(tr(" mm"));
    seParams->addWidget(m_spinSphereExpMaxStep,1,1);
    seParams->addWidget(new QLabel(tr("每曝光扫描次数"),sphereExposureGroup),1,2);
    m_spinSphereExpRepeats = new QSpinBox(sphereExposureGroup);
    m_spinSphereExpRepeats->setRange(2,5);m_spinSphereExpRepeats->setValue(2);
    seParams->addWidget(m_spinSphereExpRepeats,1,3);
    seLayout->addLayout(seParams);
    auto* seButtons = new QGridLayout;
    m_btnSphereExpImport=new QPushButton(tr("导入球点云并拟合球心"),sphereExposureGroup);
    m_btnSphereExpPlan=new QPushButton(tr("生成/检查X扫描路径"),sphereExposureGroup);
    m_btnSphereExpStart=new QPushButton(tr("按曝光逐组自动扫描"),sphereExposureGroup);
    m_btnSphereExpStop=new QPushButton(tr("停止全扫描测试"),sphereExposureGroup);
    m_btnSphereExpStart->setStyleSheet("QPushButton { background:#dff0dd; }");
    m_btnSphereExpStop->setStyleSheet("QPushButton { background:#ffdddd; }");
    seButtons->addWidget(m_btnSphereExpImport,0,0);
    seButtons->addWidget(m_btnSphereExpPlan,0,1);
    seButtons->addWidget(m_btnSphereExpStart,1,0);
    seButtons->addWidget(m_btnSphereExpStop,1,1);
    seLayout->addLayout(seButtons);
    auto* planPreviewHelp=new QLabel(tr(
        "X扫描规划预览（仅显示，不驱动设备）。3D青色线=激光平面参考点轨迹，绿色=起点，红色=终点；"
        "每个曝光按正/反X交替扫描。每条路径必须通过MachineSafety校验后才能执行。"),sphereExposureGroup);
    planPreviewHelp->setWordWrap(true);
    seLayout->addWidget(planPreviewHelp);
    m_tableSphereExpPlan = new QTableWidget(sphereExposureGroup);
    m_tableSphereExpPlan->setColumnCount(8);
    m_tableSphereExpPlan->setHorizontalHeaderLabels({tr("序号"), tr("曝光×10µs"),tr("方向"),
        tr("起点X mm"),tr("终点X mm"),tr("采样Hz"),tr("帧距 mm"),tr("执行状态")});
    m_tableSphereExpPlan->setMinimumHeight(180);
    m_tableSphereExpPlan->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_tableSphereExpPlan->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_tableSphereExpPlan->setToolTip(tr("仅用于预览与进度，不替代MachineSafety或控制器到位检查。"));
    seLayout->addWidget(m_tableSphereExpPlan);
    m_lblSphereExpStatus=new QLabel(tr("未规划。请先导入球点云并完成Physical17球心拟合。"),sphereExposureGroup);
    m_lblSphereExpStatus->setWordWrap(true);
    seLayout->addWidget(m_lblSphereExpStatus);
    m_tableSphereExpRanking=new QTableWidget(sphereExposureGroup);
    m_tableSphereExpRanking->setColumnCount(6);
    m_tableSphereExpRanking->setHorizontalHeaderLabels({tr("曝光×10µs"),tr("拟合直径mm"),
        tr("直径误差µm"),tr("圆弧RMSµm"),tr("覆盖率"),tr("综合分/状态")});
    m_tableSphereExpRanking->setMinimumHeight(142);
    m_tableSphereExpRanking->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_tableSphereExpRanking->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_tableSphereExpRanking->setToolTip(tr("仅完成全部扫描后填充。RMS和直径偏差分别是不同指标；排序仅用于候选曝光选择。"));
    seLayout->addWidget(m_tableSphereExpRanking);
    layout->addWidget(sphereExposureGroup);
    connect(m_btnSphereExpImport,&QPushButton::clicked,this,&MainWindow::onSphereStudyImportCloud);
    connect(m_btnSphereExpPlan,&QPushButton::clicked,this,&MainWindow::onPlanSphereExposureSweep);
    connect(m_btnSphereExpStart,&QPushButton::clicked,this,&MainWindow::onStartSphereExposureSweep);
    connect(m_btnSphereExpStop,&QPushButton::clicked,this,&MainWindow::onStopSphereExposureSweep);
    m_btnSphereExpStop->setEnabled(false);
    m_btnSphereExpStart->setEnabled(false); // cannot scan before a validated visible plan exists
    // Never leave an outdated 3D path / task table visible after the operator
    // changes speed, exposure or frequency. A new safety preflight is required.
    const auto invalidateSphereSweepPlan=[this](){
        if(m_sphereExpActive)return;
        m_sphereExpPlanValid=false;
        if(m_btnSphereExpStart)m_btnSphereExpStart->setEnabled(false);
        if(m_pointCloudView)m_pointCloudView->clearSphereExposureSweepPath();
        if(m_tableSphereExpPlan)m_tableSphereExpPlan->setRowCount(0);
        if(m_lblSphereExpStatus)m_lblSphereExpStatus->setText(tr("参数改变，原路径预览失效，请重新生成/检查X扫描路径。"));
    };
    connect(m_comboSphereExpPositionSpeed,QOverload<int>::of(&QComboBox::currentIndexChanged),this,
        [invalidateSphereSweepPlan](int){invalidateSphereSweepPlan();});
    connect(m_spinSphereExpSpeed,QOverload<double>::of(&QDoubleSpinBox::valueChanged),this,
            [invalidateSphereSweepPlan](double){invalidateSphereSweepPlan();});
    connect(m_spinSphereExpMargin,QOverload<double>::of(&QDoubleSpinBox::valueChanged),this,
            [invalidateSphereSweepPlan](double){invalidateSphereSweepPlan();});
    connect(m_spinSphereExpMaxStep,QOverload<double>::of(&QDoubleSpinBox::valueChanged),this,
            [invalidateSphereSweepPlan](double){invalidateSphereSweepPlan();});
    connect(m_spinSphereExpRepeats,QOverload<int>::of(&QSpinBox::valueChanged),this,
            [invalidateSphereSweepPlan](int){invalidateSphereSweepPlan();});
    connect(m_spinExpStart,QOverload<int>::of(&QSpinBox::valueChanged),this,
            [invalidateSphereSweepPlan](int){invalidateSphereSweepPlan();});
    connect(m_spinExpEnd,QOverload<int>::of(&QSpinBox::valueChanged),this,
            [invalidateSphereSweepPlan](int){invalidateSphereSweepPlan();});
    connect(m_spinExpStep,QOverload<int>::of(&QSpinBox::valueChanged),this,
            [invalidateSphereSweepPlan](int){invalidateSphereSweepPlan();});

    // ===== 球面测量距离 / 截面角 / 圆心视场偏角 / 局部表面角研究 =====
    // V27.6.16Z: D/gamma/eta + optical intersection/FOV simulation + arrival Probe/reacquisition + one 20-profile batch per pose
    // -> Circle RANSAC只负责识别圆 -> 统一EvaluationROI -> Good/Outlier/Dropout全量评价。
    auto* studyGroup = new QGroupBox(tr("球面测量距离 / 截面角 / 圆心视场偏角研究"), m_videoControlGroup);
    auto* studyLayout = new QVBoxLayout(studyGroup);
    studyLayout->setSpacing(4);
    studyLayout->setContentsMargins(5, 8, 5, 5);

    auto* studyIntro = new QLabel(tr(
        "定义：D=RANSAC提取球弧后，拟合截面圆心到传感器光学参考中心的空间距离；"
        "η(CenterBearingAngle)=atan2(Xc-X0,Zc-Z0)，描述拟合圆心在传感器X-Z视场中的方向；"
        "γ(SectionAngle)=已知球心到当前激光轮廓平面的垂距h对应 asin(h/R)，无正负；"
        "θ(LocalSurfaceAngle)=EvaluationROI内每个理论球面点的局部法向与光学观察方向夹角。"
        "RANSAC只用于找圆和建立ROI，ROI内部的无效点/异常点不会删除，而是作为Dropout/Outlier质量指标。"
        "路径允许C/B变化，并在D/γ/η/截平面方向的可行空间中做maximin空间填充；"
        "最终分析只使用实际采集得到的D/γ/η/θ。"), studyGroup);
    studyIntro->setWordWrap(true);
    studyIntro->setStyleSheet("color:#345; font-size:9px;");
    studyLayout->addWidget(studyIntro);

    auto* cloudRow = new QHBoxLayout;
    m_editSphereStudyCloud = new QLineEdit(studyGroup);
    m_editSphereStudyCloud->setReadOnly(true);
    m_editSphereStudyCloud->setPlaceholderText(tr("先导入硬件同步扫描得到的球面机床坐标点云 (.scan/.ply/.csv)"));
    m_btnSphereStudyImport = new QPushButton(tr("导入球面点云"), studyGroup);
    m_btnSphereStudyFit = new QPushButton(tr("RANSAC拟合球心"), studyGroup);
    cloudRow->addWidget(m_editSphereStudyCloud, 1);
    cloudRow->addWidget(m_btnSphereStudyImport);
    cloudRow->addWidget(m_btnSphereStudyFit);
    studyLayout->addLayout(cloudRow);

    auto* fitParamRow = new QHBoxLayout;
    fitParamRow->addWidget(new QLabel(tr("球直径:"), studyGroup));
    m_spinSphereStudyDiameter = new QDoubleSpinBox(studyGroup);
    m_spinSphereStudyDiameter->setRange(1.0, 100.0);
    m_spinSphereStudyDiameter->setDecimals(3);
    m_spinSphereStudyDiameter->setValue(19.05);
    m_spinSphereStudyDiameter->setSuffix(tr(" mm"));
    m_spinSphereStudyDiameter->setFixedWidth(90);
    fitParamRow->addWidget(m_spinSphereStudyDiameter);
    connect(m_spinSphereStudyDiameter,QOverload<double>::of(&QDoubleSpinBox::valueChanged),this,[this](double){
        if(!m_sphereExpActive){
            m_sphereExpPlanValid=false;
            if(m_btnSphereExpStart)m_btnSphereExpStart->setEnabled(false);
            if(m_pointCloudView)m_pointCloudView->clearSphereExposureSweepPath();
            if(m_tableSphereExpPlan)m_tableSphereExpPlan->setRowCount(0);
        }
    });
    fitParamRow->addWidget(new QLabel(tr("3D内点阈值:"), studyGroup));
    m_spinSphereStudyRansacThreshold = new QDoubleSpinBox(studyGroup);
    m_spinSphereStudyRansacThreshold->setRange(0.01, 2.0);
    m_spinSphereStudyRansacThreshold->setDecimals(3);
    m_spinSphereStudyRansacThreshold->setValue(0.12);
    m_spinSphereStudyRansacThreshold->setSuffix(tr(" mm"));
    m_spinSphereStudyRansacThreshold->setFixedWidth(82);
    fitParamRow->addWidget(m_spinSphereStudyRansacThreshold);
    fitParamRow->addWidget(new QLabel(tr("轮廓阈值:"), studyGroup));
    m_spinSphereStudyProfileThreshold = new QDoubleSpinBox(studyGroup);
    m_spinSphereStudyProfileThreshold->setRange(0.005, 1.0);
    m_spinSphereStudyProfileThreshold->setDecimals(3);
    m_spinSphereStudyProfileThreshold->setValue(0.050);
    m_spinSphereStudyProfileThreshold->setSuffix(tr(" mm"));
    m_spinSphereStudyProfileThreshold->setFixedWidth(82);
    m_spinSphereStudyProfileThreshold->setToolTip(tr("每条Profile球面圆弧RANSAC的内点阈值；用于研究轮廓质量，建议比3-D点云阈值更严格。"));
    fitParamRow->addWidget(m_spinSphereStudyProfileThreshold);
    fitParamRow->addWidget(new QLabel(tr("半径容差:"), studyGroup));
    m_spinSphereStudyRadiusTolerance = new QDoubleSpinBox(studyGroup);
    m_spinSphereStudyRadiusTolerance->setRange(0.05, 10.0);
    m_spinSphereStudyRadiusTolerance->setDecimals(2);
    m_spinSphereStudyRadiusTolerance->setValue(1.50);
    m_spinSphereStudyRadiusTolerance->setSuffix(tr(" mm"));
    m_spinSphereStudyRadiusTolerance->setFixedWidth(82);
    fitParamRow->addWidget(m_spinSphereStudyRadiusTolerance);
    fitParamRow->addStretch();
    studyLayout->addLayout(fitParamRow);

    m_lblSphereStudyFit = new QLabel(tr("尚未拟合球面"), studyGroup);
    m_lblSphereStudyFit->setWordWrap(true);
    m_lblSphereStudyFit->setStyleSheet("QLabel { color:#555; } ");
    studyLayout->addWidget(m_lblSphereStudyFit);

    // Keep the optical reference explicit.  In the current calibrated
    // scanCONTROL frame it is normally X0=Z0=0; if the project later adopts a
    // different optical reference, the D definition remains unchanged.
    // Keep the geometry controls readable on the narrow right-side panel.
    // V27.6.16Z keeps the multi-row layout and adds optical-intersection/FOV planning plus real-profile Probe/reacquisition:
    // each study variable gets its own line (start/end/step), so labels and
    // spin boxes remain visible even at the minimum application width.
    auto* geometryParamGroup = new QGroupBox(tr("研究几何参数"), studyGroup);
    auto* geometryGrid = new QGridLayout(geometryParamGroup);
    geometryGrid->setContentsMargins(6, 8, 6, 6);
    geometryGrid->setHorizontalSpacing(6);
    geometryGrid->setVerticalSpacing(4);

    geometryGrid->addWidget(new QLabel(tr("传感器光学参考中心"), geometryParamGroup), 0, 0);
    geometryGrid->addWidget(new QLabel(tr("X0:"), geometryParamGroup), 0, 1);
    m_spinSphereStudyOpticalCenterX = new QDoubleSpinBox(geometryParamGroup);
    m_spinSphereStudyOpticalCenterX->setRange(-200.0, 200.0);
    m_spinSphereStudyOpticalCenterX->setDecimals(4);
    m_spinSphereStudyOpticalCenterX->setValue(0.0);
    m_spinSphereStudyOpticalCenterX->setSuffix(tr(" mm"));
    m_spinSphereStudyOpticalCenterX->setMinimumWidth(88);
    geometryGrid->addWidget(m_spinSphereStudyOpticalCenterX, 0, 2);
    geometryGrid->addWidget(new QLabel(tr("Z0:"), geometryParamGroup), 0, 3);
    m_spinSphereStudyOpticalCenterZ = new QDoubleSpinBox(geometryParamGroup);
    m_spinSphereStudyOpticalCenterZ->setRange(-200.0, 200.0);
    m_spinSphereStudyOpticalCenterZ->setDecimals(4);
    m_spinSphereStudyOpticalCenterZ->setValue(0.0);
    m_spinSphereStudyOpticalCenterZ->setSuffix(tr(" mm"));
    m_spinSphereStudyOpticalCenterZ->setMinimumWidth(88);
    geometryGrid->addWidget(m_spinSphereStudyOpticalCenterZ, 0, 4);
    auto* opticalHint = new QLabel(tr("D=√[(Xc-X0)²+(Zc-Z0)²]；Y0固定为轮廓平面0"), geometryParamGroup);
    opticalHint->setStyleSheet("color:#567; font-size:9px;");
    opticalHint->setWordWrap(true);
    geometryGrid->addWidget(opticalHint, 1, 0, 1, 5);

    auto makeStudyDouble = [geometryParamGroup](double lo, double hi, double value, double step,
        const QString& suffix, int decimals = 2) {
            auto* w = new QDoubleSpinBox(geometryParamGroup);
            w->setRange(lo, hi); w->setValue(value); w->setSingleStep(step);
            w->setDecimals(decimals); w->setSuffix(suffix); w->setMinimumWidth(82);
            return w;
        };

    auto* startHeader = new QLabel(tr("起点"), geometryParamGroup);
    auto* endHeader = new QLabel(tr("终点"), geometryParamGroup);
    auto* stepHeader = new QLabel(tr("步长"), geometryParamGroup);
    startHeader->setAlignment(Qt::AlignCenter); endHeader->setAlignment(Qt::AlignCenter); stepHeader->setAlignment(Qt::AlignCenter);
    geometryGrid->addWidget(startHeader, 2, 2);
    geometryGrid->addWidget(endHeader, 2, 3);
    geometryGrid->addWidget(stepHeader, 2, 4);

    geometryGrid->addWidget(new QLabel(tr("测量距离 D"), geometryParamGroup), 3, 0, 1, 2);
    m_spinSphereStudyDistanceStart = makeStudyDouble(70.0, 120.0, 82.0, 1.0, tr(" mm"));
    m_spinSphereStudyDistanceEnd = makeStudyDouble(70.0, 120.0, 107.0, 1.0, tr(" mm"));
    m_spinSphereStudyDistanceStep = makeStudyDouble(0.1, 30.0, 5.0, 0.5, tr(" mm"));
    geometryGrid->addWidget(m_spinSphereStudyDistanceStart, 3, 2);
    geometryGrid->addWidget(m_spinSphereStudyDistanceEnd, 3, 3);
    geometryGrid->addWidget(m_spinSphereStudyDistanceStep, 3, 4);

    geometryGrid->addWidget(new QLabel(tr("截面角 γ"), geometryParamGroup), 4, 0, 1, 2);
    m_spinSphereStudyGammaStart = makeStudyDouble(0.0, 85.0, 0.0, 1.0, QString::fromUtf8(" °"));
    m_spinSphereStudyGammaEnd = makeStudyDouble(0.0, 85.0, 50.0, 1.0, QString::fromUtf8(" °"));
    m_spinSphereStudyGammaStep = makeStudyDouble(0.5, 30.0, 10.0, 0.5, QString::fromUtf8(" °"));
    geometryGrid->addWidget(m_spinSphereStudyGammaStart, 4, 2);
    geometryGrid->addWidget(m_spinSphereStudyGammaEnd, 4, 3);
    geometryGrid->addWidget(m_spinSphereStudyGammaStep, 4, 4);

    geometryGrid->addWidget(new QLabel(tr("圆心视场偏角 η"), geometryParamGroup), 5, 0, 1, 2);
    m_spinSphereStudyEtaStart = makeStudyDouble(-45.0, 45.0, -15.0, 1.0, QString::fromUtf8(" °"));
    m_spinSphereStudyEtaEnd = makeStudyDouble(-45.0, 45.0, 15.0, 1.0, QString::fromUtf8(" °"));
    m_spinSphereStudyEtaStep = makeStudyDouble(0.5, 30.0, 7.5, 0.5, QString::fromUtf8(" °"));
    m_spinSphereStudyEtaStart->setToolTip(tr("η=atan2(Xc-X0,Zc-Z0)，0°表示圆心位于光学中心+Z方向；正值偏向+X。"));
    m_spinSphereStudyEtaEnd->setToolTip(m_spinSphereStudyEtaStart->toolTip());
    m_spinSphereStudyEtaStep->setToolTip(m_spinSphereStudyEtaStart->toolTip());
    geometryGrid->addWidget(m_spinSphereStudyEtaStart, 5, 2);
    geometryGrid->addWidget(m_spinSphereStudyEtaEnd, 5, 3);
    geometryGrid->addWidget(m_spinSphereStudyEtaStep, 5, 4);

    geometryGrid->addWidget(new QLabel(tr("候选截平面方向数"), geometryParamGroup), 6, 0, 1, 2);
    m_spinSphereStudyPlaneDirections = new QSpinBox(geometryParamGroup);
    m_spinSphereStudyPlaneDirections->setRange(1, 24);
    m_spinSphereStudyPlaneDirections->setValue(8);
    m_spinSphereStudyPlaneDirections->setSuffix(tr(" 个"));
    m_spinSphereStudyPlaneDirections->setMinimumWidth(82);
    m_spinSphereStudyPlaneDirections->setToolTip(tr(
        "不再做C×B笛卡尔积。先在B/C软件限位内建立可达平面方向候选，"
        "再由最终D/γ/η/Plane空间填充算法挑选研究姿态。"));
    geometryGrid->addWidget(m_spinSphereStudyPlaneDirections, 6, 2);

    geometryGrid->addWidget(new QLabel(tr("最终目标姿态数"), geometryParamGroup), 7, 0, 1, 2);
    m_spinSphereStudyTargetPoseCount = new QSpinBox(geometryParamGroup);
    m_spinSphereStudyTargetPoseCount->setRange(8, 400);
    m_spinSphereStudyTargetPoseCount->setValue(120);
    m_spinSphereStudyTargetPoseCount->setSuffix(tr(" 个"));
    m_spinSphereStudyTargetPoseCount->setMinimumWidth(82);
    m_spinSphereStudyTargetPoseCount->setToolTip(tr(
        "程序会先生成所有满足机床限位和传感器FOV约束的D/γ/η/Plane候选，"
        "再用maximin空间填充选出该数量的姿态；不是规则笛卡尔积。"));
    geometryGrid->addWidget(m_spinSphereStudyTargetPoseCount, 7, 2);

    geometryGrid->addWidget(new QLabel(tr("B/C候选搜索步长"), geometryParamGroup), 8, 0, 1, 2);
    m_spinSphereStudyRotarySearchStep = makeStudyDouble(2.0, 30.0, 10.0, 1.0, QString::fromUtf8(" °"), 1);
    m_spinSphereStudyRotarySearchStep->setToolTip(tr(
        "只用于自动建立可达平面方向候选集，不是实验角度步长；越小搜索越充分但生成路径稍慢。"));
    geometryGrid->addWidget(m_spinSphereStudyRotarySearchStep, 8, 2);

    geometryGrid->addWidget(new QLabel(tr("最小预测可见弧长"), geometryParamGroup), 9, 0, 1, 2);
    m_spinSphereStudyMinSimArcLength = makeStudyDouble(0.5, 30.0, 6.0, 0.5, tr(" mm"), 1);
    m_spinSphereStudyMinSimArcLength->setToolTip(tr(
        "除了可见圆弧角度，还要求仿真得到的实际圆弧物理长度达到该阈值。"
        "这样可以拒绝接近切平面时角度看似足够、但实际只有很短轮廓的姿态。"));
    geometryGrid->addWidget(m_spinSphereStudyMinSimArcLength, 9, 2);

    auto* rotaryHint = new QLabel(tr(
        "推荐规划：γ决定截平面到球心的距离h=R·sinγ；D与η共同决定圆心位置："
        "Xc=X0+D·sinη，Zc=Z0+D·cosη。程序先搜索可达激光平面方向，再在D/γ/η/Plane四维空间中"
        "做maximin选点，最后由Physical17/RTCP等价逆解得到XYZBC。AC版在完全离线状态下也使用相同光学/碰撞门禁；无相交/弧段过小候选自动丢弃，"
        "必要时在用户给定D/γ/η范围内插值生成替代姿态并再次仿真筛选。"), geometryParamGroup);
    rotaryHint->setWordWrap(true);
    rotaryHint->setStyleSheet("color:#567; font-size:9px;");
    geometryGrid->addWidget(rotaryHint, 10, 0, 1, 5);
    geometryGrid->setColumnStretch(0, 1);
    studyLayout->addWidget(geometryParamGroup);

    auto* planRow = new QHBoxLayout;
    m_lblSphereStudyModel = new QLabel(tr("规划模型：未读取"), studyGroup);
    m_lblSphereStudyModel->setWordWrap(true);
    planRow->addWidget(m_lblSphereStudyModel, 1);
    planRow->addWidget(new QLabel(tr("移动倍率:"), studyGroup));
    m_spinSphereStudySpeedScale = makeStudyDouble(1.0, 50.0, 10.0, 1.0, tr(" %"), 1);
    planRow->addWidget(m_spinSphereStudySpeedScale);
    m_btnSphereStudyGenerate = new QPushButton(tr("生成球面截平面族路径"), studyGroup);
    planRow->addWidget(m_btnSphereStudyGenerate);
    studyLayout->addLayout(planRow);

    auto* simulationRow = new QHBoxLayout;
    m_btnSphereStudySimulationStart = new QPushButton(tr("仿真自动采集路径"), studyGroup);
    m_btnSphereStudySimulationStart->setStyleSheet("QPushButton { background-color:#dceffc; font-weight:bold; }");
    m_btnSphereStudySimulationStart->setToolTip(tr(
        "纯离线功能：不要求控制器或传感器连接，也绝不发送运动/采集命令。数字孪生依次到达规划XYZCB姿态，"
        "显示scanCONTROL X-Z梯形测量场、球面截圆和“靠近五轴末端执行器一侧”的预测有效圆弧，并把每个有效圆弧累积成仿真采集点云叠加到三维视图。\n"
        "梯形方向按物理光学深度定义：靠近传感器/末端的一侧永远是小端，远离传感器的一侧永远是大端；若标定保存的FlipZ=true，则只翻转reported Sensor-Z数值顺序，不翻转实际梯形。"));
    m_btnSphereStudySimulationStop = new QPushButton(tr("停止仿真"), studyGroup);
    m_btnSphereStudySimulationClear = new QPushButton(tr("清除仿真点云"), studyGroup);
    m_btnSphereStudySimulationClear->setToolTip(tr("清除离线仿真累积得到的采集点云；不会清除导入的球面点云。"));
    simulationRow->addWidget(m_btnSphereStudySimulationStart);
    simulationRow->addWidget(m_btnSphereStudySimulationStop);
    simulationRow->addWidget(m_btnSphereStudySimulationClear);
    simulationRow->addSpacing(12);
    simulationRow->addWidget(new QLabel(tr("每姿态显示:"), studyGroup));
    m_spinSphereStudySimulationIntervalMs = new QSpinBox(studyGroup);
    m_spinSphereStudySimulationIntervalMs->setRange(150, 5000);
    m_spinSphereStudySimulationIntervalMs->setValue(800);
    m_spinSphereStudySimulationIntervalMs->setSingleStep(100);
    m_spinSphereStudySimulationIntervalMs->setSuffix(tr(" ms"));
    simulationRow->addWidget(m_spinSphereStudySimulationIntervalMs);
    simulationRow->addStretch();
    studyLayout->addLayout(simulationRow);

    m_sphereStudyTable = new QTableWidget(studyGroup);
    m_sphereStudyTable->setColumnCount(23);
    m_sphereStudyTable->setHorizontalHeaderLabels({
        tr("#"), tr("D目标"), tr("γ目标"), tr("η目标"), tr("X"), tr("Y"), tr("Z"), tr("C"), tr("B"),
        tr("末端侧有效弧°"), tr("末端侧弧长mm"), tr("相交余量"), tr("D实测"), tr("γ实测"), tr("η实测"), tr("已采集"),
        tr("Good%"), tr("Drop%"), tr("Outlier%"), tr("CircleRMS"), tr("P95绝对误差"), tr("半径偏差"), tr("状态") });
    m_sphereStudyTable->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_sphereStudyTable->setSelectionMode(QAbstractItemView::SingleSelection);
    m_sphereStudyTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_sphereStudyTable->verticalHeader()->setVisible(false);
    m_sphereStudyTable->setMinimumHeight(165);
    m_sphereStudyTable->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    m_sphereStudyTable->horizontalHeader()->setStretchLastSection(true);
    studyLayout->addWidget(m_sphereStudyTable);
    connect(m_sphereStudyTable, &QTableWidget::itemSelectionChanged,
        this, &MainWindow::updateSphereStudyUiState);

    auto* captureGroup = new QGroupBox(tr("采集与自动执行"), studyGroup);
    auto* captureGrid = new QGridLayout(captureGroup);
    captureGrid->setContentsMargins(6, 8, 6, 6);
    captureGrid->setHorizontalSpacing(6);
    captureGrid->setVerticalSpacing(4);

    // One independent acquisition batch per pose.  Twenty fresh profiles are
    // enough for the current sensor-quality study; repeating 5 groups without
    // moving adds many highly correlated frames but little new geometry.
    m_spinSphereStudyGroups = new QSpinBox(captureGroup);
    m_spinSphereStudyGroups->setRange(1, 1); m_spinSphereStudyGroups->setValue(1);
    m_spinSphereStudyGroups->hide();

    captureGrid->addWidget(new QLabel(tr("每姿态Profile:"), captureGroup), 0, 0);
    m_spinSphereStudyProfilesPerGroup = new QSpinBox(captureGroup);
    m_spinSphereStudyProfilesPerGroup->setRange(5, 200); m_spinSphereStudyProfilesPerGroup->setValue(20);
    m_spinSphereStudyProfilesPerGroup->setSuffix(tr(" 条")); m_spinSphereStudyProfilesPerGroup->setMinimumWidth(76);
    captureGrid->addWidget(m_spinSphereStudyProfilesPerGroup, 0, 1);

    captureGrid->addWidget(new QLabel(tr("起始丢弃:"), captureGroup), 0, 2);
    m_spinSphereStudyDiscard = new QSpinBox(captureGroup);
    m_spinSphereStudyDiscard->setRange(0, 100); m_spinSphereStudyDiscard->setValue(3);
    m_spinSphereStudyDiscard->setSuffix(tr(" 帧")); m_spinSphereStudyDiscard->setMinimumWidth(76);
    captureGrid->addWidget(m_spinSphereStudyDiscard, 0, 3);

    captureGrid->addWidget(new QLabel(tr("到位稳定等待:"), captureGroup), 1, 0);
    m_spinSphereStudySettleMs = new QSpinBox(captureGroup);
    m_spinSphereStudySettleMs->setRange(0, 10000); m_spinSphereStudySettleMs->setValue(800);
    m_spinSphereStudySettleMs->setSuffix(tr(" ms")); m_spinSphereStudySettleMs->setMinimumWidth(88);
    m_spinSphereStudySettleMs->setToolTip(tr("自动流程中五轴到位后等待机械/传感器状态稳定，再开始丢弃帧和正式采样。"));
    captureGrid->addWidget(m_spinSphereStudySettleMs, 1, 1);

    captureGrid->addWidget(new QLabel(tr("单姿态采集超时:"), captureGroup), 1, 2);
    m_spinSphereStudyAutoTimeoutMs = new QSpinBox(captureGroup);
    m_spinSphereStudyAutoTimeoutMs->setRange(2000, 120000); m_spinSphereStudyAutoTimeoutMs->setValue(15000);
    m_spinSphereStudyAutoTimeoutMs->setSingleStep(1000); m_spinSphereStudyAutoTimeoutMs->setSuffix(tr(" ms"));
    m_spinSphereStudyAutoTimeoutMs->setMinimumWidth(96);
    captureGrid->addWidget(m_spinSphereStudyAutoTimeoutMs, 1, 3);

    captureGrid->addWidget(new QLabel(tr("失败重试:"), captureGroup), 2, 0);
    m_spinSphereStudyAutoRetries = new QSpinBox(captureGroup);
    m_spinSphereStudyAutoRetries->setRange(0, 5); m_spinSphereStudyAutoRetries->setValue(1);
    m_spinSphereStudyAutoRetries->setSuffix(tr(" 次")); m_spinSphereStudyAutoRetries->setMinimumWidth(76);
    captureGrid->addWidget(m_spinSphereStudyAutoRetries, 2, 1);

    m_checkSphereStudyAutoSkipCompleted = new QCheckBox(tr("自动流程跳过已完成位姿"), captureGroup);
    m_checkSphereStudyAutoSkipCompleted->setChecked(true);
    m_checkSphereStudyAutoContinueOnFailure = new QCheckBox(tr("单个位姿失败后继续下一位姿"), captureGroup);
    m_checkSphereStudyAutoContinueOnFailure->setChecked(true);
    captureGrid->addWidget(m_checkSphereStudyAutoSkipCompleted, 2, 2);
    captureGrid->addWidget(m_checkSphereStudyAutoContinueOnFailure, 2, 3);

    captureGrid->addWidget(new QLabel(tr("到位Probe:"), captureGroup), 3, 0);
    m_spinSphereStudyProbeProfiles = new QSpinBox(captureGroup);
    m_spinSphereStudyProbeProfiles->setRange(3, 12); m_spinSphereStudyProbeProfiles->setValue(5);
    m_spinSphereStudyProbeProfiles->setSuffix(tr(" 帧")); m_spinSphereStudyProbeProfiles->setMinimumWidth(76);
    m_spinSphereStudyProbeProfiles->setToolTip(tr("自动到位后先采少量Profile做Circle RANSAC；只有确认真实看到球圆弧后才开始正式20条采集。"));
    captureGrid->addWidget(m_spinSphereStudyProbeProfiles, 3, 1);

    captureGrid->addWidget(new QLabel(tr("Probe至少成圆:"), captureGroup), 3, 2);
    m_spinSphereStudyProbeMinCircles = new QSpinBox(captureGroup);
    m_spinSphereStudyProbeMinCircles->setRange(1, 12); m_spinSphereStudyProbeMinCircles->setValue(3);
    m_spinSphereStudyProbeMinCircles->setSuffix(tr(" 帧")); m_spinSphereStudyProbeMinCircles->setMinimumWidth(76);
    captureGrid->addWidget(m_spinSphereStudyProbeMinCircles, 3, 3);

    captureGrid->addWidget(new QLabel(tr("自动找球步长:"), captureGroup), 4, 0);
    m_spinSphereStudyReacquireStepMm = new QDoubleSpinBox(captureGroup);
    m_spinSphereStudyReacquireStepMm->setRange(0.1, 2.0); m_spinSphereStudyReacquireStepMm->setDecimals(2);
    m_spinSphereStudyReacquireStepMm->setSingleStep(0.1); m_spinSphereStudyReacquireStepMm->setValue(0.5);
    m_spinSphereStudyReacquireStepMm->setSuffix(tr(" mm")); m_spinSphereStudyReacquireStepMm->setMinimumWidth(82);
    m_spinSphereStudyReacquireStepMm->setToolTip(tr("Probe找不到球时，在保持C/B不变的条件下，按Sensor局部X/Y/Z六方向分层搜索；每个候选姿态先通过球-激光平面相交、梯形FOV/末端侧有效弧长和Machine5X碰撞门禁。"));
    captureGrid->addWidget(m_spinSphereStudyReacquireStepMm, 4, 1);

    captureGrid->addWidget(new QLabel(tr("最大找球范围:"), captureGroup), 4, 2);
    m_spinSphereStudyReacquireMaxMm = new QDoubleSpinBox(captureGroup);
    m_spinSphereStudyReacquireMaxMm->setRange(0.0, 5.0); m_spinSphereStudyReacquireMaxMm->setDecimals(2);
    m_spinSphereStudyReacquireMaxMm->setSingleStep(0.5); m_spinSphereStudyReacquireMaxMm->setValue(2.0);
    m_spinSphereStudyReacquireMaxMm->setSuffix(tr(" mm")); m_spinSphereStudyReacquireMaxMm->setMinimumWidth(82);
    m_spinSphereStudyReacquireMaxMm->setToolTip(tr("Sensor局部X/Y/Z单轴自动找球的最大绝对偏移；搜索顺序为±Y、±X、±Z，并逐层扩大。"));
    captureGrid->addWidget(m_spinSphereStudyReacquireMaxMm, 4, 3);
    studyLayout->addWidget(captureGroup);

    auto* studyDirRow = new QHBoxLayout;
    studyDirRow->addWidget(new QLabel(tr("研究目录:"), studyGroup));
    m_editSphereStudyDir = new QLineEdit(studyGroup);
    m_editSphereStudyDir->setReadOnly(true);
    m_editSphereStudyDir->setText(QDir::currentPath() + "/sphere_distance_angle_study");
    studyDirRow->addWidget(m_editSphereStudyDir, 1);
    auto* btnStudyBrowse = new QPushButton(tr("..."), studyGroup);
    btnStudyBrowse->setFixedWidth(30);
    connect(btnStudyBrowse, &QPushButton::clicked, this, [this]() {
        const QString dir = QFileDialog::getExistingDirectory(
            this, tr("选择球面距离/角度研究目录"), m_editSphereStudyDir->text());
        if (!dir.isEmpty()) m_editSphereStudyDir->setText(dir);
        });
    studyDirRow->addWidget(btnStudyBrowse);
    studyLayout->addLayout(studyDirRow);

    auto* manualCtrlRow = new QHBoxLayout;
    manualCtrlRow->addWidget(new QLabel(tr("手动调试:"), studyGroup));
    m_btnSphereStudyMove = new QPushButton(tr("移动到选定位姿"), studyGroup);
    m_btnSphereStudyStop = new QPushButton(tr("停止移动"), studyGroup);
    m_btnSphereStudyCapture = new QPushButton(tr("采集选定位姿"), studyGroup);
    manualCtrlRow->addWidget(m_btnSphereStudyMove);
    manualCtrlRow->addWidget(m_btnSphereStudyStop);
    manualCtrlRow->addWidget(m_btnSphereStudyCapture);
    manualCtrlRow->addStretch();
    studyLayout->addLayout(manualCtrlRow);

    auto* autoCtrlRow = new QHBoxLayout;
    autoCtrlRow->addWidget(new QLabel(tr("自动流程:"), studyGroup));
    m_btnSphereStudyAutoStart = new QPushButton(tr("自动采集剩余路径"), studyGroup);
    m_btnSphereStudyAutoStart->setStyleSheet("QPushButton { background-color:#dff3df; font-weight:bold; }");
    m_btnSphereStudyAutoPause = new QPushButton(tr("暂停"), studyGroup);
    m_btnSphereStudyAutoStop = new QPushButton(tr("停止自动"), studyGroup);
    m_btnSphereStudyAutoStop->setStyleSheet("QPushButton { background-color:#ffd6d6; }");
    autoCtrlRow->addWidget(m_btnSphereStudyAutoStart);
    autoCtrlRow->addWidget(m_btnSphereStudyAutoPause);
    autoCtrlRow->addWidget(m_btnSphereStudyAutoStop);
    autoCtrlRow->addStretch();
    studyLayout->addLayout(autoCtrlRow);

    auto* analysisCtrlRow = new QHBoxLayout;
    analysisCtrlRow->addWidget(new QLabel(tr("分析/导出:"), studyGroup));
    m_btnSphereStudyAnalyze = new QPushButton(tr("RANSAC/ROI分析全部数据"), studyGroup);
    m_btnSphereStudyExport = new QPushButton(tr("导出研究结果"), studyGroup);
    analysisCtrlRow->addWidget(m_btnSphereStudyAnalyze);
    analysisCtrlRow->addWidget(m_btnSphereStudyExport);
    analysisCtrlRow->addStretch();
    studyLayout->addLayout(analysisCtrlRow);
    m_lblSphereStudyCapture = new QLabel(tr("研究状态：等待导入球面点云"), studyGroup);
    m_lblSphereStudyCapture->setWordWrap(true);
    m_lblSphereStudyCapture->setStyleSheet("QLabel { font-weight:bold; color:#345; }");
    studyLayout->addWidget(m_lblSphereStudyCapture);

    layout->addWidget(studyGroup);
    layout->addWidget(createSmall7ExperimentPanel(m_videoControlGroup));

    connect(m_btnSphereStudyImport, &QPushButton::clicked, this, &MainWindow::onSphereStudyImportCloud);
    connect(m_btnSphereStudyFit, &QPushButton::clicked, this, &MainWindow::onSphereStudyFitSphere);
    connect(m_btnSphereStudyGenerate, &QPushButton::clicked, this, &MainWindow::onSphereStudyGeneratePath);
    connect(m_btnSphereStudyMove, &QPushButton::clicked, this, &MainWindow::onSphereStudyMoveSelected);
    connect(m_btnSphereStudyStop, &QPushButton::clicked, this, &MainWindow::onSphereStudyStopMove);
    connect(m_btnSphereStudyCapture, &QPushButton::clicked, this, &MainWindow::onSphereStudyCaptureSelected);
    connect(m_btnSphereStudySimulationStart, &QPushButton::clicked, this, &MainWindow::onSphereStudySimulationStart);
    connect(m_btnSphereStudySimulationStop, &QPushButton::clicked, this, &MainWindow::onSphereStudySimulationStop);
    connect(m_btnSphereStudySimulationClear, &QPushButton::clicked, this, &MainWindow::onSphereStudySimulationClear);
    connect(m_btnSphereStudyAutoStart, &QPushButton::clicked, this, &MainWindow::onSphereStudyAutoStart);
    connect(m_btnSphereStudyAutoPause, &QPushButton::clicked, this, &MainWindow::onSphereStudyAutoPauseResume);
    connect(m_btnSphereStudyAutoStop, &QPushButton::clicked, this, &MainWindow::onSphereStudyAutoStop);
    connect(m_btnSphereStudyAnalyze, &QPushButton::clicked, this, &MainWindow::onSphereStudyAnalyze);
    connect(m_btnSphereStudyExport, &QPushButton::clicked, this, &MainWindow::onSphereStudyExport);

    connect(m_btnStartVideo, &QPushButton::clicked, this, &MainWindow::onStartVideo);
    connect(m_btnStopVideo, &QPushButton::clicked, this, &MainWindow::onStopVideo);
    connect(m_btnStartProfilePreview, &QPushButton::clicked, this, &MainWindow::onStartProfilePreview);
    connect(m_btnStopProfilePreview, &QPushButton::clicked, this, &MainWindow::onStopProfilePreview);
    connect(m_radioVideoMode, &QRadioButton::toggled, this, &MainWindow::onPreviewModeChanged);
    connect(m_btnStartExpTest, &QPushButton::clicked, this, &MainWindow::onStartExposureTest);
    connect(m_btnStopExpTest, &QPushButton::clicked, this, &MainWindow::onStopExposureTest);
    connect(m_btnAnalyzeExpTestDir, &QPushButton::clicked, this, &MainWindow::onAnalyzeExposureDirectory);
    connect(&m_expTestTimer, &QTimer::timeout, this, &MainWindow::onExposureTestStep);
    connect(m_spinVideoExposure, QOverload<int>::of(&QSpinBox::valueChanged),
        this, [this](int) { updateSensorCalibrationTimingUi(); });
    connect(m_comboResolution, QOverload<int>::of(&QComboBox::currentIndexChanged),
        this, [this](int) { updateSensorCalibrationTimingUi(); });
    connect(m_radioSoftwareSync, &QRadioButton::toggled,
        this, [this](bool) { updateSensorCalibrationTimingUi(); });
    connect(m_comboMeasuringField, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, [this](int) {
                updateSensorCalibrationTimingUi();
                if (!m_sphereExpActive) {
                    m_sphereExpPlanValid = false;
                    if (m_btnSphereExpStart) m_btnSphereExpStart->setEnabled(false);
                    if (m_tableSphereExpPlan) m_tableSphereExpPlan->setRowCount(0);
                    if (m_pointCloudView) m_pointCloudView->clearSphereExposureSweepPath();
                }
            });
    connect(m_checkSensorCalibAutoFieldRate, &QCheckBox::toggled,
            this, [this](bool) {
                updateSensorCalibrationTimingUi();
                if (!m_sphereExpActive) {
                    m_sphereExpPlanValid = false;
                    if (m_btnSphereExpStart) m_btnSphereExpStart->setEnabled(false);
                    if (m_tableSphereExpPlan) m_tableSphereExpPlan->setRowCount(0);
                    if (m_pointCloudView) m_pointCloudView->clearSphereExposureSweepPath();
                }
                updateVideoControlState();
            });
    connect(m_spinSensorCalibRate, QOverload<int>::of(&QSpinBox::valueChanged),
        this, [this](int) {
            updateSensorCalibrationTimingUi();
            if(!m_sphereExpActive){
                m_sphereExpPlanValid=false;
                if(m_btnSphereExpStart)m_btnSphereExpStart->setEnabled(false);
                if(m_tableSphereExpPlan)m_tableSphereExpPlan->setRowCount(0);
                if(m_pointCloudView)m_pointCloudView->clearSphereExposureSweepPath();
            }
        });

    connect(m_btnReadPosition, &QPushButton::clicked, this, [this]() {
        if (!m_zmc || !m_zmc->isConnected()) {
            QMessageBox::warning(this, tr("提示"), tr("请先连接控制器"));
            return;
        }
        SamplePoint pose;
        QString err;
        if (!m_zmc->readJointSample5D(pose, &err)) {
            QMessageBox::warning(this, tr("错误"), tr("读取XYZCB失败: %1").arg(err));
            return;
        }
        m_spinPosX->setValue(pose.x);
        m_spinPosY->setValue(pose.y);
        m_spinPosZ->setValue(pose.z);
        m_spinPosC->setValue(pose.c);
        m_spinPosB->setValue(pose.b);
        statusBar()->showMessage(
            tr("机床位姿已读取: X=%1 Y=%2 Z=%3 C=%4° B=%5°")
            .arg(pose.x, 0, 'f', 3).arg(pose.y, 0, 'f', 3).arg(pose.z, 0, 'f', 3)
            .arg(pose.c, 0, 'f', 3).arg(pose.b, 0, 'f', 3), 4000);
        });

    updateSensorCalibrationTimingUi();
    updateVideoControlState();
    updateSphereStudyUiState();
    return m_videoControlGroup;
}


void MainWindow::updateSphereStudyUiState()
{
    const bool hasCloud = !m_sphereStudyImportedCloud.isEmpty();
    const bool hasSphere = m_sphereStudySphereFit.success;
    const bool hasPlan = !m_sphereStudyPlan.empty();
    const bool selected = sphereStudySelectedPlanIndex() >= 0;
    const bool controllerReady = m_zmc && m_zmc->isConnected() && m_zmc->areLinearMposReferenced()
        && m_zmc->isRotaryAbsoluteReady(AxisC) && m_zmc->isRotaryAbsoluteReady(AxisB);
    const bool sensorReady = m_laser && m_laser->isConnected();
    const bool otherAcquisition = (m_syncManager && m_syncManager->isRunning())
        || (m_expTestState != ExpTestState::Idle)
        || (m_laser && m_laser->isVideoRunning());
    const bool studyBusy = m_sphereStudyFitActive || m_sphereStudyCaptureActive || m_jointMoveInProgress ||
        m_sphereStudyAutoActive || m_sphereStudySimulationActive;

    if (m_btnSphereStudyImport) m_btnSphereStudyImport->setEnabled(!studyBusy);
    if (m_btnSphereStudyFit) m_btnSphereStudyFit->setEnabled(hasCloud && !studyBusy);
    // V27.6.16AC: geometry/path generation is a local-model operation.  A live
    // controller reference is useful online but is no longer a prerequisite.
    if (m_btnSphereStudyGenerate) m_btnSphereStudyGenerate->setEnabled(hasSphere && m_machineSafetyReady && !studyBusy);
    if (m_btnSphereStudyMove) m_btnSphereStudyMove->setEnabled(hasPlan && selected && controllerReady && m_machineSafetyReady && !studyBusy && !otherAcquisition);
    if (m_btnSphereStudyStop) m_btnSphereStudyStop->setEnabled(m_jointMoveInProgress);
    if (m_btnSphereStudyCapture) m_btnSphereStudyCapture->setEnabled(hasPlan && selected && hasSphere
        && m_sphereStudyHasPlanningModel && controllerReady && sensorReady && !studyBusy
        && !(m_syncManager && m_syncManager->isRunning())
        && m_expTestState == ExpTestState::Idle && !(m_laser && m_laser->isVideoRunning()));

    const bool simulationReady = hasPlan && hasSphere && m_sphereStudyHasPlanningModel && m_machineSafetyReady;
    if (m_btnSphereStudySimulationStart) m_btnSphereStudySimulationStart->setEnabled(
        simulationReady && !m_sphereStudySimulationActive && !m_sphereStudyCaptureActive &&
        !m_jointMoveInProgress && !m_sphereStudyAutoActive);
    if (m_btnSphereStudySimulationStop) m_btnSphereStudySimulationStop->setEnabled(m_sphereStudySimulationActive);
    if (m_btnSphereStudySimulationClear) m_btnSphereStudySimulationClear->setEnabled(
        !m_sphereStudySimulationActive && !m_sphereStudySimulationAcquiredCloud.isEmpty());

    const bool gcodeBusy = m_gcodeExecutor && (m_gcodeExecutor->isRunning() || m_gcodeExecutor->isPaused());
    const bool autoReady = hasPlan && hasSphere && m_sphereStudyHasPlanningModel && controllerReady && sensorReady && m_machineSafetyReady
        && !otherAcquisition && !gcodeBusy;
    if (m_btnSphereStudyAutoStart) m_btnSphereStudyAutoStart->setEnabled(autoReady && !m_sphereStudyAutoActive
        && !m_sphereStudyCaptureActive && !m_jointMoveInProgress && !m_sphereStudySimulationActive);
    if (m_btnSphereStudyAutoPause) {
        m_btnSphereStudyAutoPause->setEnabled(m_sphereStudyAutoActive && !m_sphereStudyAutoStopRequested);
        m_btnSphereStudyAutoPause->setText(m_sphereStudyAutoPaused || m_sphereStudyAutoPauseRequested ? tr("继续") : tr("暂停"));
        m_btnSphereStudyAutoPause->setToolTip(m_sphereStudyAutoPauseRequested && !m_sphereStudyAutoPaused
            ? tr("已请求暂停；将在当前移动或当前位姿采集结束后暂停。")
            : tr("自动流程采用协作式暂停，不在一组Profile中间切断数据。"));
    }
    if (m_btnSphereStudyAutoStop) m_btnSphereStudyAutoStop->setEnabled(m_sphereStudyAutoActive);

    if (m_btnSphereStudyAnalyze) m_btnSphereStudyAnalyze->setEnabled(!m_sphereStudyRecords.empty() && !studyBusy);
    if (m_btnSphereStudyExport) m_btnSphereStudyExport->setEnabled((hasSphere || !m_sphereStudyRecords.empty()) && !studyBusy);

    const bool editGrid = !studyBusy;
    if (m_spinSphereStudyDiameter) m_spinSphereStudyDiameter->setEnabled(editGrid);
    if (m_spinSphereStudyRansacThreshold) m_spinSphereStudyRansacThreshold->setEnabled(editGrid);
    if (m_spinSphereStudyProfileThreshold) m_spinSphereStudyProfileThreshold->setEnabled(editGrid);
    if (m_spinSphereStudyRadiusTolerance) m_spinSphereStudyRadiusTolerance->setEnabled(editGrid);
    if (m_spinSphereStudyOpticalCenterX) m_spinSphereStudyOpticalCenterX->setEnabled(editGrid);
    if (m_spinSphereStudyOpticalCenterZ) m_spinSphereStudyOpticalCenterZ->setEnabled(editGrid);
    if (m_spinSphereStudyDistanceStart) m_spinSphereStudyDistanceStart->setEnabled(editGrid);
    if (m_spinSphereStudyDistanceEnd) m_spinSphereStudyDistanceEnd->setEnabled(editGrid);
    if (m_spinSphereStudyDistanceStep) m_spinSphereStudyDistanceStep->setEnabled(editGrid);
    if (m_spinSphereStudyGammaStart) m_spinSphereStudyGammaStart->setEnabled(editGrid);
    if (m_spinSphereStudyGammaEnd) m_spinSphereStudyGammaEnd->setEnabled(editGrid);
    if (m_spinSphereStudyGammaStep) m_spinSphereStudyGammaStep->setEnabled(editGrid);
    if (m_spinSphereStudyEtaStart) m_spinSphereStudyEtaStart->setEnabled(editGrid);
    if (m_spinSphereStudyEtaEnd) m_spinSphereStudyEtaEnd->setEnabled(editGrid);
    if (m_spinSphereStudyEtaStep) m_spinSphereStudyEtaStep->setEnabled(editGrid);
    if (m_spinSphereStudyPlaneDirections) m_spinSphereStudyPlaneDirections->setEnabled(editGrid);
    if (m_spinSphereStudyTargetPoseCount) m_spinSphereStudyTargetPoseCount->setEnabled(editGrid);
    if (m_spinSphereStudyRotarySearchStep) m_spinSphereStudyRotarySearchStep->setEnabled(editGrid);
    if (m_spinSphereStudyMinSimArcLength) m_spinSphereStudyMinSimArcLength->setEnabled(editGrid);
    if (m_spinSphereStudySimulationIntervalMs) m_spinSphereStudySimulationIntervalMs->setEnabled(editGrid);
    if (m_spinSphereStudySpeedScale) m_spinSphereStudySpeedScale->setEnabled(editGrid);
    if (m_spinSphereStudyGroups) m_spinSphereStudyGroups->setEnabled(editGrid);
    if (m_spinSphereStudyProfilesPerGroup) m_spinSphereStudyProfilesPerGroup->setEnabled(editGrid);
    if (m_spinSphereStudyDiscard) m_spinSphereStudyDiscard->setEnabled(editGrid);
    if (m_spinSphereStudySettleMs) m_spinSphereStudySettleMs->setEnabled(editGrid);
    if (m_spinSphereStudyAutoTimeoutMs) m_spinSphereStudyAutoTimeoutMs->setEnabled(editGrid);
    if (m_spinSphereStudyAutoRetries) m_spinSphereStudyAutoRetries->setEnabled(editGrid);
    if (m_checkSphereStudyAutoSkipCompleted) m_checkSphereStudyAutoSkipCompleted->setEnabled(editGrid);
    if (m_checkSphereStudyAutoContinueOnFailure) m_checkSphereStudyAutoContinueOnFailure->setEnabled(editGrid);
}

int MainWindow::sphereStudySelectedPlanIndex() const
{
    if (!m_sphereStudyTable) return -1;
    const auto rows = m_sphereStudyTable->selectionModel()
        ? m_sphereStudyTable->selectionModel()->selectedRows() : QModelIndexList{};
    if (rows.isEmpty()) return -1;
    const int row = rows.first().row();
    if (row < 0 || row >= static_cast<int>(m_sphereStudyPlan.size())) return -1;
    return row;
}

int MainWindow::sphereStudyCapturedGroupCount(int planIndex) const
{
    int count = 0;
    for (const auto& rec : m_sphereStudyRecords) {
        if (rec.complete && rec.planIndex == planIndex) ++count;
    }
    return count;
}

void MainWindow::rebuildSphereStudyPlanTable()
{
    if (!m_sphereStudyTable) return;
    m_sphereStudyTable->setRowCount(static_cast<int>(m_sphereStudyPlan.size()));

    std::vector<int> capturedGroups(m_sphereStudyPlan.size(), 0);
    for (const auto& rec : m_sphereStudyRecords) {
        if (rec.complete && rec.planIndex >= 0 && rec.planIndex < static_cast<int>(capturedGroups.size()))
            ++capturedGroups[static_cast<std::size_t>(rec.planIndex)];
    }
    auto put = [this](int row, int col, const QString& text) {
        auto* item = m_sphereStudyTable->item(row, col);
        if (!item) { item = new QTableWidgetItem; m_sphereStudyTable->setItem(row, col, item); }
        item->setText(text);
        };

    for (int row = 0; row < static_cast<int>(m_sphereStudyPlan.size()); ++row) {
        const auto& p = m_sphereStudyPlan[static_cast<std::size_t>(row)];
        put(row, 0, QString::number(row + 1));
        put(row, 1, QString::number(p.geometry.measurementDistanceMm, 'f', 2));
        put(row, 2, QString::number(p.geometry.sectionAngleDeg, 'f', 2));
        put(row, 3, QString::number(p.geometry.centerBearingAngleDeg, 'f', 2));
        put(row, 4, p.valid ? QString::number(p.joints.x, 'f', 3) : QStringLiteral("--"));
        put(row, 5, p.valid ? QString::number(p.joints.y, 'f', 3) : QStringLiteral("--"));
        put(row, 6, p.valid ? QString::number(p.joints.z, 'f', 3) : QStringLiteral("--"));
        put(row, 7, p.valid ? QString::number(p.joints.c, 'f', 3) : QStringLiteral("--"));
        put(row, 8, p.valid ? QString::number(p.joints.b, 'f', 3) : QStringLiteral("--"));
        put(row, 9, p.valid ? QString::number(p.predictedVisibleArcDeg, 'f', 1) : QStringLiteral("--"));
        put(row, 10, p.valid ? QString::number(p.predictedVisibleArcLengthMm, 'f', 2) : QStringLiteral("--"));
        put(row, 11, p.valid ? QString::number(p.predictedPlaneIntersectionMarginMm, 'f', 2) : QStringLiteral("--"));
        put(row, 15, QString::number(capturedGroups[static_cast<std::size_t>(row)]));

        QString dActual = QStringLiteral("--"), gActual = QStringLiteral("--"), eActual = QStringLiteral("--");
        QString good = QStringLiteral("--"), drop = QStringLiteral("--"), outlier = QStringLiteral("--");
        QString circle = QStringLiteral("--"), p95 = QStringLiteral("--"), radiusBias = QStringLiteral("--");
        if (m_sphereStudySummaries.size() == m_sphereStudyPlan.size()) {
            const auto& ss = m_sphereStudySummaries[static_cast<std::size_t>(row)];
            if (ss.profiles > 0) {
                dActual = QString::number(ss.meanActualDistanceMm, 'f', 3);
                gActual = QString::number(ss.meanSectionAngleDeg, 'f', 3);
                eActual = QString::number(ss.meanCenterBearingAngleDeg, 'f', 3);
                good = QString::number(ss.meanGoodPointRatio * 100.0, 'f', 1);
                drop = QString::number(ss.meanDropoutRatio * 100.0, 'f', 1);
                outlier = QString::number(ss.meanOutlierRatio * 100.0, 'f', 1);
                circle = QString::number(ss.meanCircleRmsMm, 'f', 4);
                p95 = QString::number(ss.meanAbsoluteP95Mm, 'f', 4);
                radiusBias = QString::number(ss.meanRadiusBiasMm, 'f', 4);
            }
        }
        else {
            double ds = 0.0, gs = 0.0, es = 0.0; int n = 0;
            for (const auto& r : m_sphereStudyRecords) {
                if (r.planIndex != row || !r.poseGeometryEstimate.valid) continue;
                ds += r.poseGeometryEstimate.measurementDistanceMm;
                gs += r.poseGeometryEstimate.sectionAngleDeg;
                es += r.poseGeometryEstimate.centerBearingAngleDeg; ++n;
            }
            if (n > 0) {
                dActual = QString::number(ds / n, 'f', 3);
                gActual = QString::number(gs / n, 'f', 3);
                eActual = QString::number(es / n, 'f', 3);
            }
        }
        put(row, 12, dActual); put(row, 13, gActual); put(row, 14, eActual);
        put(row, 16, good); put(row, 17, drop); put(row, 18, outlier);
        put(row, 19, circle); put(row, 20, p95); put(row, 21, radiusBias);

        QString status;
        if (!p.valid) status = tr("规划失败: %1").arg(QString::fromStdString(p.message));
        else if (capturedGroups[static_cast<std::size_t>(row)] > 0 && m_sphereStudySummaries.size() == m_sphereStudyPlan.size())
            status = tr("已分析；Plane=%1").arg(p.rotarySampleId + 1);
        else if (row < static_cast<int>(m_sphereStudyAutoPlanNotes.size()) && !m_sphereStudyAutoPlanNotes[static_cast<std::size_t>(row)].isEmpty())
            status = m_sphereStudyAutoPlanNotes[static_cast<std::size_t>(row)];
        else if (capturedGroups[static_cast<std::size_t>(row)] > 0) status = tr("已采集，待分析");
        else status = p.adaptivelyRegenerated
            ? tr("待移动/采集；自适应替代姿态；Plane=%1").arg(p.rotarySampleId + 1)
            : tr("待移动/采集；Plane=%1").arg(p.rotarySampleId + 1);
        put(row, 22, status);
    }
    updateSphereStudyUiState();
}

void MainWindow::onSphereStudyImportCloud()
{
    m_sphereExpPlanValid = false;
    if (m_pointCloudView) m_pointCloudView->clearSphereExposureSweepPath();
    if (m_tableSphereExpPlan) m_tableSphereExpPlan->setRowCount(0);
    if (m_btnSphereExpStart) m_btnSphereExpStart->setEnabled(false);
    const QString path = QFileDialog::getOpenFileName(
        this, tr("导入球面机床坐标点云"), QString(),
        tr("点云文件 (*.scan *.ply *.csv *.txt);;所有文件 (*.*)"));
    if (path.isEmpty()) return;

    QVector<QVector3D> points;
    QString error;
    if (!PointCloudLoader::load(path, points, &error) || points.size() < 4) {
        QMessageBox::warning(this, tr("导入失败"), error.isEmpty() ? tr("点云有效点不足4个") : error);
        return;
    }
    m_sphereStudyImportedCloud = points;
    m_sphereStudyImportedCloudPath = path;
    m_sphereStudyFitComparison.clear();
    // A newly imported cloud invalidates the previous collision obstacle until
    // the new RANSAC sphere has been accepted.  This prevents a stale sphere
    // from blocking or, worse, validating the next study path.
    if (m_machineSafety) m_machineSafety->clearStudySphere();
    if (m_pointCloudView) m_pointCloudView->clearMachineStudySphere();
    m_sphereStudySphereFit = {};
    m_sphereStudyPlan.clear();
    m_sphereStudyRecords.clear();
    m_sphereStudySummaries.clear();
    // The persisted Physical17/stage-1 navigation model is independent from the
    // imported sphere cloud.  Keep it available across offline cloud reloads.
    m_sphereStudySessionDir.clear();
    m_sphereStudySimulationAcquiredCloud.clear();
    m_sphereStudySimulationAcquiredPoses = 0;
    m_sphereStudySimulationRejectedPoses = 0;
    m_sphereStudyLastPlanWasOffline = false;
    if (m_pointCloudView) {
        m_pointCloudView->clearSphereStudyScanSimulation();
        m_pointCloudView->clearSphereStudySimulatedAcquisitionCloud();
    }
    m_sphereStudyVisitCounter = 0;
    m_sphereStudyCurrentVisitId = -1;
    if (m_editSphereStudyCloud) m_editSphereStudyCloud->setText(path);
    if (m_lblSphereStudyFit) m_lblSphereStudyFit->setText(tr("已导入 %1 点；准备自动执行3-D RANSAC球面提取").arg(points.size()));
    if (m_lblSphereStudyCapture) m_lblSphereStudyCapture->setText(tr(
        "研究状态：点云已导入。离线模式将自动拟合机床坐标系球心；无需连接控制器或传感器。"));
    if (m_pointCloudView) m_pointCloudView->setPoints(points);
    if (m_viewTabs && m_pointCloudView) m_viewTabs->setCurrentWidget(m_pointCloudView);
    rebuildSphereStudyPlanTable();
    updateSphereStudyUiState();

    // V27.6.16AC: importing a sphere cloud is enough to enter the offline
    // planning workflow.  Start the same RANSAC routine asynchronously with the
    // current UI thresholds; the explicit RANSAC button remains available for
    // manual re-fit after the operator changes thresholds/diameter.
    QTimer::singleShot(0, this, [this]() {
        if (!m_sphereStudyImportedCloud.isEmpty() && !m_sphereStudySphereFit.success &&
            !m_sphereStudyCaptureActive && !m_sphereStudyAutoActive &&
            !m_sphereStudySimulationActive) {
            onSphereStudyFitSphere();
        }
        });
}

void MainWindow::onSphereStudyFitSphere()
{
    m_sphereExpPlanValid = false;
    if (m_pointCloudView) m_pointCloudView->clearSphereExposureSweepPath();
    if (m_tableSphereExpPlan) m_tableSphereExpPlan->setRowCount(0);
    if (m_btnSphereExpStart) m_btnSphereExpStart->setEnabled(false);
    if (m_sphereStudyImportedCloud.size() < 4) {
        QMessageBox::warning(this, tr("球面拟合"), tr("请先导入球面点云。"));
        return;
    }
    std::vector<mls::calibration::Vec3d> points;
    points.reserve(static_cast<std::size_t>(m_sphereStudyImportedCloud.size()));
    for (const auto& p : m_sphereStudyImportedCloud) points.emplace_back(p.x(), p.y(), p.z());

    mls::calibration::SphereCloudRansacOptions options;
    options.inlierThresholdMm = m_spinSphereStudyRansacThreshold->value();
    options.expectedRadiusMm = 0.5 * m_spinSphereStudyDiameter->value();
    options.maxRadiusDeviationMm = m_spinSphereStudyRadiusTolerance->value();
    options.minInliers = std::min<int>(std::max(50, static_cast<int>(points.size() / 200)), 1000);
    options.minInlierRatio = 0.02;
    options.maxIterations = 1400;

    m_sphereStudyFitActive = true;
    m_btnSphereStudyFit->setEnabled(false);
    m_lblSphereStudyFit->setText(tr("3-D RANSAC球面提取中... 点数=%1").arg(points.size()));
    updateSphereStudyUiState();
    const QString importedPath = m_sphereStudyImportedCloudPath;
    auto* watcher = new QFutureWatcher<mls::calibration::SphereTwoStageResult>(this);
    connect(watcher, &QFutureWatcher<mls::calibration::SphereTwoStageResult>::finished,
        this, [this, watcher, importedPath]() {
            const auto fitResult = watcher->result(); watcher->deleteLater();
            // Another cloud was imported while the background fit was running.
            // Never turn a stale fit into an executable collision/reference sphere.
            if (importedPath != m_sphereStudyImportedCloudPath) {
                m_sphereStudyFitActive = false;
                updateSphereStudyUiState();
                LOG_WARNING(LogCategory::Calibration, tr("[两阶段参考球] 旧文件拟合结果已丢弃：已导入另一份点云"));
                return;
            }
            m_sphereStudyFitComparison = mls::calibration::SphereTwoStageFit::comparisonText(fitResult);
            const auto& fit = fitResult.accepted;
            const QString fitDiagnostic = mls::calibration::SphereTwoStageFit::comparisonText(fitResult);
            if (fit.success) LOG_INFO(LogCategory::Calibration, tr("[两阶段参考球/球面研究] %1").arg(fitDiagnostic));
            else LOG_WARNING(LogCategory::Calibration, tr("[两阶段参考球/球面研究][REJECT] %1").arg(fitDiagnostic));
            m_sphereStudyFitActive = false;
            m_sphereStudySphereFit = fit;
            if (!fit.success) {
                if (m_machineSafety) m_machineSafety->clearStudySphere();
                if (m_pointCloudView) m_pointCloudView->clearMachineStudySphere();
                m_lblSphereStudyFit->setText(tr("球面拟合未通过：%1").arg(
                    mls::calibration::SphereTwoStageFit::comparisonText(fitResult)));
                QMessageBox::warning(this, tr("球面拟合失败"),
                    mls::calibration::SphereTwoStageFit::comparisonText(fitResult));
                updateSphereStudyUiState(); return;
            }
            if (m_machineSafetyReady && m_machineSafety) {
                const double studyRadius = 0.5 * m_spinSphereStudyDiameter->value();
                m_machineSafety->setStudySphere(fit.centerMachineMm.x, fit.centerMachineMm.y,
                    fit.centerMachineMm.z, studyRadius);
                if (m_pointCloudView) {
                    m_pointCloudView->setMachineStudySphere(
                        fit.centerMachineMm.x, fit.centerMachineMm.y, fit.centerMachineMm.z,
                        studyRadius, m_machineSafety->config().safety.studySphereClearanceMm);
                }
            }
            m_lblSphereStudyFit->setText(tr(
                "球心M=[%1, %2, %3] mm；拟合R=%4 mm；内点=%5/%6 (%7%)；RMS=%8 mm。后续截面理论半径使用已知标准球直径=%9 mm。")
                .arg(fit.centerMachineMm.x, 0, 'f', 4).arg(fit.centerMachineMm.y, 0, 'f', 4)
                .arg(fit.centerMachineMm.z, 0, 'f', 4).arg(fit.radiusMm, 0, 'f', 4)
                .arg(static_cast<qulonglong>(fit.inlierPointCount)).arg(static_cast<qulonglong>(fit.candidatePointCount))
                .arg(fit.inlierRatio * 100.0, 0, 'f', 1).arg(fit.residualRmsMm, 0, 'f', 4)
                .arg(m_spinSphereStudyDiameter->value(), 0, 'f', 3)
                + QStringLiteral("\n") + mls::calibration::SphereTwoStageFit::comparisonText(fitResult));
            QVector<QVector3D> inliers;
            inliers.reserve(static_cast<int>(fit.inlierIndices.size()));
            for (std::size_t i : fit.inlierIndices)
                if (i < static_cast<std::size_t>(m_sphereStudyImportedCloud.size())) inliers.push_back(m_sphereStudyImportedCloud[static_cast<int>(i)]);
            if (!inliers.isEmpty() && m_pointCloudView) m_pointCloudView->setPoints(inliers);
            m_sphereStudyPlan.clear(); m_sphereStudyRecords.clear(); m_sphereStudySummaries.clear(); m_sphereStudyAutoPlanNotes.clear();
            m_sphereStudySessionDir.clear(); m_sphereStudyVisitCounter = 0; m_sphereStudyCurrentVisitId = -1;
            m_sphereStudySimulationAcquiredCloud.clear();
            m_sphereStudySimulationAcquiredPoses = 0;
            m_sphereStudySimulationRejectedPoses = 0;
            if (m_pointCloudView) m_pointCloudView->clearSphereStudySimulatedAcquisitionCloud();
            rebuildSphereStudyPlanTable();
            QString modelError;
            const bool modelReady = refreshSphereStudyPlanningModel(&modelError);
            m_lblSphereStudyCapture->setText(modelReady
                ? tr("研究状态：球心已确定，已读取本机持久化Physical17/阶段1规划模型；控制器和传感器均可保持离线，下一步可直接生成球面截平面族路径并仿真采集。")
                : tr("研究状态：球心已确定；离线路径仍需要本机已有的Physical17/阶段1初始模型：%1").arg(modelError));
            updateSphereStudyUiState();
        });
    watcher->setFuture(QtConcurrent::run([points = std::move(points), options, importedPath]() {
        return mls::calibration::SphereTwoStageFit::fitImportedSphere(importedPath, points, options);
        }));
}

bool MainWindow::refreshSphereStudyPlanningModel(QString* error)
{
    std::array<double, 17> p{};
    QString source;
    bool validated = false;
    if (!m_calibrationWidget ||
        !m_calibrationWidget->sphereStudyPlanningCalibration(p, source, validated)) {
        m_sphereStudyHasPlanningModel = false;
        m_sphereStudyPlanningModelSource.clear();
        m_sphereStudyPlanningModelValidated = false;
        if (m_lblSphereStudyModel) {
            m_lblSphereStudyModel->setText(tr("规划模型：未找到本机Physical17/阶段1结果"));
            m_lblSphereStudyModel->setStyleSheet(QStringLiteral("QLabel { color:#a33; }"));
        }
        if (error) *error = tr("没有可用的Physical17/阶段1初始模型。离线仿真不需要控制器/传感器，但仍需要已有手眼+机床运动学模型把传感器XZ平面映射到机床坐标系。");
        return false;
    }

    m_sphereStudyPlanningModel.value = p;
    m_sphereStudyPlanningModelSource = source;
    m_sphereStudyPlanningModelValidated = validated;

    bool flipX = false, flipZ = true;
    QString conventionSource;
    if (m_calibrationWidget->sphereStudySensorCoordinateConvention(
        flipX, flipZ, conventionSource)) {
        m_sphereStudyPlanningFlipX = flipX;
        m_sphereStudyPlanningFlipZ = flipZ;
        m_sphereStudyPlanningConventionSource = conventionSource;
    }
    else {
        // Planning should almost always have a saved convention together with
        // its Physical17 model.  Keep a deterministic fallback for very old
        // projects, matching the application's long-standing sensor default.
        m_sphereStudyPlanningFlipX = m_checkFlipX && m_checkFlipX->isChecked();
        m_sphereStudyPlanningFlipZ = !m_checkFlipZ || m_checkFlipZ->isChecked();
        m_sphereStudyPlanningConventionSource = tr("旧工程回退到当前界面Flip设置");
    }
    m_sphereStudyHasPlanningModel = true;
    if (m_lblSphereStudyModel) {
        m_lblSphereStudyModel->setText(tr("规划模型：%1（可离线使用，FlipZ=%2）")
            .arg(source).arg(m_sphereStudyPlanningFlipZ));
        m_lblSphereStudyModel->setStyleSheet(validated
            ? QStringLiteral("QLabel { color:#275; }")
            : QStringLiteral("QLabel { color:#a65a00; }"));
    }
    if (error) error->clear();
    return true;
}

void MainWindow::onSphereStudyGeneratePath()
{
    if (!m_sphereStudySphereFit.success) {
        QMessageBox::warning(this, tr("路径生成"), tr("请先完成球面RANSAC拟合。"));
        return;
    }

    QString machineSafetyError;
    if (!sphereStudySafetyReady(&machineSafetyError)) {
        QMessageBox::critical(this, tr("机床安全模型未就绪"), machineSafetyError);
        return;
    }
    m_machineSafety->setStudySphere(m_sphereStudySphereFit.centerMachineMm.x,
        m_sphereStudySphereFit.centerMachineMm.y,
        m_sphereStudySphereFit.centerMachineMm.z,
        0.5 * m_spinSphereStudyDiameter->value());

    // V27.6.16AC: planning/calibrated forward simulation is deliberately
    // independent from hardware connectivity.  The calibration widget restores
    // the persisted Physical17/stage-1 model at application startup, so an
    // offline operator can import a machine-frame sphere cloud and plan directly.
    QString modelError;
    if (!refreshSphereStudyPlanningModel(&modelError)) {
        QMessageBox::warning(this, tr("缺少离线规划模型"), modelError);
        return;
    }

    const bool controllerReferenceReady = m_zmc && m_zmc->isConnected()
        && m_zmc->areLinearMposReferenced()
        && m_zmc->isRotaryAbsoluteReady(AxisC)
        && m_zmc->isRotaryAbsoluteReady(AxisB);
    bool haveLiveReference = false;
    JointPose5D reference{};
    QString liveReferenceNote;
    if (controllerReferenceReady) {
        QString readError;
        if (m_zmc->readJointPose5D(reference, &readError)) {
            haveLiveReference = true;
        }
        else {
            liveReferenceNote = tr("控制器已连接但参考MPOS读取失败（%1），本次自动退回纯离线参考姿态。")
                .arg(readError);
        }
    }

    auto buildRange = [](double a, double b, double step, std::vector<double>& out)->bool {
        out.clear(); if (!(step > 0.0) || a > b)return false;
        for (int i = 0; i < 1000; ++i) { const double v = a + i * step; if (v > b + 1e-9)break; out.push_back(std::min(v, b)); }
        if (out.empty() || out.size() >= 1000)return false;
        if (std::abs(out.back() - b) > 1e-8)out.push_back(b); return out.size() <= 1000;
        };
    mls::calibration::SphereStudyPathOptions options;
    if (!buildRange(m_spinSphereStudyDistanceStart->value(), m_spinSphereStudyDistanceEnd->value(), m_spinSphereStudyDistanceStep->value(), options.measurementDistancesMm)
        || !buildRange(m_spinSphereStudyGammaStart->value(), m_spinSphereStudyGammaEnd->value(), m_spinSphereStudyGammaStep->value(), options.sectionAnglesDeg)
        || !buildRange(m_spinSphereStudyEtaStart->value(), m_spinSphereStudyEtaEnd->value(), m_spinSphereStudyEtaStep->value(), options.centerBearingAnglesDeg)) {
        QMessageBox::warning(this, tr("参数错误"), tr("D/γ/η范围必须满足起点<=终点且步长>0。")); return;
    }
    options.automaticPlaneFamily = true;
    options.planeDirectionsPerGeometry = m_spinSphereStudyPlaneDirections ? m_spinSphereStudyPlaneDirections->value() : 8;
    options.rotaryCandidateStepDeg = m_spinSphereStudyRotarySearchStep ? m_spinSphereStudyRotarySearchStep->value() : 10.0;
    options.minimumPlaneSeparationDeg = 8.0;
    options.spaceFillingSampling = true;
    options.targetPoseCount = m_spinSphereStudyTargetPoseCount ? m_spinSphereStudyTargetPoseCount->value() : 120;
    const std::size_t candidateUpper = options.measurementDistancesMm.size() * options.sectionAnglesDeg.size() *
        options.centerBearingAnglesDeg.size() * static_cast<std::size_t>(std::max(1, options.planeDirectionsPerGeometry));
    if (candidateUpper > 12000) {
        QMessageBox::warning(this, tr("候选空间过大"), tr(
            "当前D/γ/η/Plane候选上限约%1个；请增大D/γ/η步长、减少候选截平面方向数，或分批实验。单次候选上限12000。")
            .arg(candidateUpper)); return;
    }
    options.sphereRadiusMm = 0.5 * m_spinSphereStudyDiameter->value();

    // V27.6.16AD: disconnected AxisParams still contain the explicit
    // +/-999999 "not read" placeholders.  Never feed those values into the
    // automatic C/B lattice: doing so creates an effectively unbounded nested
    // loop.  Offline mode uses the finite limits stored with Machine5X; online
    // mode intersects those limits with the live controller limits.
    QString planningLimitSource, planningLimitError;
    if (!resolveSphereStudyPlanningLimits(haveLiveReference, options,
        &planningLimitSource, &planningLimitError)) {
        QMessageBox::critical(this, tr("规划限位不可用"), planningLimitError);
        return;
    }
    options.maxAutomaticRotaryCandidateCount = 2048;

    if (!haveLiveReference) {
        auto neutral = [](double lo, double hi) { return std::min(hi, std::max(lo, 0.0)); };
        reference.x = neutral(options.xMinMm, options.xMaxMm);
        reference.y = neutral(options.yMinMm, options.yMaxMm);
        reference.z = neutral(options.zMinMm, options.zMaxMm);
        reference.c = neutral(options.cMinDeg, options.cMaxDeg);
        reference.b = neutral(options.bMinDeg, options.bMaxDeg);
    }
    options.referenceJoint = { reference.x,reference.y,reference.z,reference.c,reference.b };

    options.opticalCenterSensorMm = { m_spinSphereStudyOpticalCenterX->value(),0.0,m_spinSphereStudyOpticalCenterZ->value() };
    // V27.6.16AF: the standard scanCONTROL field is defined by PHYSICAL optical
    // depth: small edge near the sensor, large edge away from the sensor.  The
    // calibrated profile convention can have FlipZ enabled, in which case the
    // numerical Sensor-Z order is reversed.  Pass that convention explicitly;
    // never attach the 42/58 mm widths blindly to numeric ZMin/ZMax.
    options.sensorXMinMm = Constants::kSensorXMin; options.sensorXMaxMm = Constants::kSensorXMax;
    options.sensorZMinMm = Constants::kSensorZMin; options.sensorZMaxMm = Constants::kSensorZMax;
    options.sensorZCoordinateFlipped = m_sphereStudyPlanningFlipZ;
    options.sensorXSpanNearMm = Constants::kSensorXSpanNear;
    options.sensorXSpanFarMm = Constants::kSensorXSpanFar;
    options.useTrapezoidalSensorField = true;
    options.requireEndEffectorFacingArc = true;
    options.sensorXSafetyMarginMm = 1.0; options.sensorZSafetyMarginMm = 1.0;
    options.minimumPlaneIntersectionMarginMm = 0.50;
    options.minimumPredictedVisibleArcDeg = 75.0;
    options.minimumPredictedVisibleArcLengthMm = m_spinSphereStudyMinSimArcLength
        ? m_spinSphereStudyMinSimArcLength->value() : 6.0;
    options.visibleArcSamples = 720;
    options.adaptiveOpticalResampling = true;
    options.adaptiveOpticalResamplingPasses = 2;
    options.maxAdaptiveCandidateCount = 24000;
    if (haveLiveReference) {
        const auto currentSphereS = mls::calibration::SingleSphere17Model::machinePointInSensor(
            { reference.x,reference.y,reference.z,reference.c,reference.b },
            m_sphereStudySphereFit.centerMachineMm, m_sphereStudyPlanningModel);
        options.sectionSideSign = currentSphereS.y < 0.0 ? -1 : +1;
    }
    else {
        // There is no physically meaningful "current sensor side" while offline.
        // Pick a deterministic side first; if no collision-safe pose survives we
        // retry the mirror side below before declaring failure.
        options.sectionSideSign = +1;
    }
    options.serpentine = true;

    auto generateAndSafetyGate = [&](int sectionSideSign, std::string& generateError)->int {
        options.sectionSideSign = sectionSideSign;
        m_sphereStudyPlan = mls::calibration::SphereDistanceAngleStudy::generatePath(
            m_sphereStudySphereFit.centerMachineMm, m_sphereStudyPlanningModel, options, generateError);
        const JointPose5D safetyStart{ reference.x,reference.y,reference.z,reference.c,reference.b };
        return applySphereStudySafetyFilterAndTransitionGate(safetyStart, haveLiveReference);
        };

    QElapsedTimer planningTimer;
    planningTimer.start();
    statusBar()->showMessage(tr("正在生成球面截平面族：%1；C=[%2,%3]°, B=[%4,%5]°，搜索步长=%6°...")
        .arg(planningLimitSource)
        .arg(options.cMinDeg, 0, 'f', 1).arg(options.cMaxDeg, 0, 'f', 1)
        .arg(options.bMinDeg, 0, 'f', 1).arg(options.bMaxDeg, 0, 'f', 1)
        .arg(options.rotaryCandidateStepDeg, 0, 'f', 1));
    QApplication::setOverrideCursor(Qt::WaitCursor);
    QCoreApplication::processEvents(QEventLoop::ExcludeUserInputEvents);

    std::string error;
    int safetyAccepted = generateAndSafetyGate(options.sectionSideSign, error);
    int validCount = 0; for (const auto& pose : m_sphereStudyPlan)if (pose.valid)++validCount;
    if (!haveLiveReference && validCount == 0) {
        std::string mirrorError;
        const int mirrorAccepted = generateAndSafetyGate(-options.sectionSideSign, mirrorError);
        int mirrorValid = 0; for (const auto& pose : m_sphereStudyPlan)if (pose.valid)++mirrorValid;
        safetyAccepted = mirrorAccepted;
        validCount = mirrorValid;
        if (!mirrorError.empty()) error = mirrorError;
    }

    QApplication::restoreOverrideCursor();
    const qint64 planningElapsedMs = planningTimer.elapsed();
    LOG_INFO(LogCategory::Controller,
        tr("球面截平面族规划完成：耗时=%1 ms，限位源=%2，生成行=%3，Machine5X安全通过=%4")
        .arg(planningElapsedMs).arg(planningLimitSource)
        .arg(static_cast<int>(m_sphereStudyPlan.size())).arg(safetyAccepted));

    m_sphereStudyLastPlanWasOffline = !haveLiveReference;
    m_sphereStudyRecords.clear(); m_sphereStudySummaries.clear(); m_sphereStudySessionDir.clear(); m_sphereStudyVisitCounter = 0; m_sphereStudyCurrentVisitId = -1;
    m_sphereStudyAutoPlanNotes.assign(m_sphereStudyPlan.size(), QString());
    m_sphereStudySimulationAcquiredCloud.clear();
    m_sphereStudySimulationAcquiredPoses = 0;
    m_sphereStudySimulationRejectedPoses = 0;
    if (m_pointCloudView) {
        m_pointCloudView->clearSphereStudyScanSimulation();
        m_pointCloudView->clearSphereStudySimulatedAcquisitionCloud();
    }
    rebuildSphereStudyPlanTable();
    if (validCount == 0) { QMessageBox::warning(this, tr("路径生成失败"), QString::fromStdString(error)); return; }
    for (int row = 0; row < static_cast<int>(m_sphereStudyPlan.size()); ++row)
        if (m_sphereStudyPlan[static_cast<std::size_t>(row)].valid) { m_sphereStudyTable->selectRow(row); break; }

    const QString validationNote = m_sphereStudyPlanningModelValidated
        ? tr("该模型已独立验证")
        : tr("该模型仅作为规划/仿真初值；实机实验仍以实际MPOS和真实Circle-RANSAC为准");
    const QString modeNote = haveLiveReference
        ? tr("在线参考：使用当前五轴MPOS，并检查当前姿态到首姿态的完整扫掠路径")
        : tr("纯离线规划：无需控制器/传感器；虚拟参考C=%1° B=%2°，首姿态仅做静态碰撞门禁，随后所有相邻XYZCB段仍做完整扫掠碰撞检查")
        .arg(reference.c, 0, 'f', 1).arg(reference.b, 0, 'f', 1);
    const QString fallbackNote = liveReferenceNote.isEmpty() ? QString() : tr("；%1").arg(liveReferenceNote);
    m_lblSphereStudyCapture->setText(tr(
        "研究状态：从约%1个D/γ/η/Plane候选中，经五轴可达性、球-激光平面相交、实际X/Z梯形测量场、末端执行器侧圆弧角度/物理弧长约束，"
        "再经Machine5X FCL静态碰撞/间隙与相邻XYZCB整段扫掠检查，保留%2个安全位姿；候选截平面方向=%3，目标姿态数=%4；%5；%6%7。"
        "现在可直接点击“仿真自动采集路径”，离线累积预测采集点云并在三维窗口观察覆盖结果。")
        .arg(candidateUpper).arg(validCount).arg(options.planeDirectionsPerGeometry)
        .arg(options.targetPoseCount).arg(validationNote).arg(modeNote)
        .arg(fallbackNote + tr("；规划限位=%1；耗时=%2 ms").arg(planningLimitSource).arg(planningElapsedMs)));
    Q_UNUSED(safetyAccepted);
    updateSphereStudyUiState();
}

mls::calibration::SphereStudyPathOptions MainWindow::makeSphereStudyOpticalOptions(
    const JointPose5D& reference) const
{
    mls::calibration::SphereStudyPathOptions options;
    options.sphereRadiusMm = 0.5 * (m_spinSphereStudyDiameter ? m_spinSphereStudyDiameter->value() : 19.05);
    options.referenceJoint = { reference.x,reference.y,reference.z,reference.c,reference.b };
    options.opticalCenterSensorMm = { m_spinSphereStudyOpticalCenterX ? m_spinSphereStudyOpticalCenterX->value() : 0.0,
                                   0.0,
                                   m_spinSphereStudyOpticalCenterZ ? m_spinSphereStudyOpticalCenterZ->value() : 0.0 };

    const bool liveLimitsReady = m_zmc && m_zmc->isConnected() && m_zmc->areLinearMposReferenced()
        && m_zmc->isRotaryAbsoluteReady(AxisC) && m_zmc->isRotaryAbsoluteReady(AxisB);
    // Keep simulation/reacquisition on the same finite limit resolver used by
    // path generation.  Optical-only callers do not consume these limits, but
    // solvePoseForSphereCenterTarget does.
    (void)resolveSphereStudyPlanningLimits(liveLimitsReady, options, nullptr, nullptr);

    options.sensorXMinMm = Constants::kSensorXMin;
    options.sensorXMaxMm = Constants::kSensorXMax;
    options.sensorZMinMm = Constants::kSensorZMin;
    options.sensorZMaxMm = Constants::kSensorZMax;
    options.sensorZCoordinateFlipped = m_sphereStudyPlanningFlipZ;
    options.sensorXSpanNearMm = Constants::kSensorXSpanNear;
    options.sensorXSpanFarMm = Constants::kSensorXSpanFar;
    options.useTrapezoidalSensorField = true;
    options.requireEndEffectorFacingArc = true;
    options.sensorXSafetyMarginMm = 1.0;
    options.sensorZSafetyMarginMm = 1.0;
    options.minimumPlaneIntersectionMarginMm = 0.50;
    options.minimumPredictedVisibleArcDeg = 75.0;
    options.minimumPredictedVisibleArcLengthMm = m_spinSphereStudyMinSimArcLength
        ? m_spinSphereStudyMinSimArcLength->value() : 6.0;
    options.visibleArcSamples = 720;
    return options;
}

bool MainWindow::showSphereStudySimulationPose(int planIndex, bool selectRow, QString* error)
{
    if (planIndex < 0 || planIndex >= static_cast<int>(m_sphereStudyPlan.size())) {
        if (error)*error = tr("仿真规划索引越界");
        return false;
    }
    if (!m_sphereStudyHasPlanningModel || !m_sphereStudySphereFit.success || !m_pointCloudView) {
        if (error)*error = tr("仿真需要已拟合球、初始Physical17模型和三维数字孪生视图");
        return false;
    }
    const auto& pose = m_sphereStudyPlan[static_cast<std::size_t>(planIndex)];
    if (!pose.valid) {
        if (error)*error = tr("规划位姿无效：%1").arg(QString::fromStdString(pose.message));
        return false;
    }

    const JointPose5D q{ pose.joints.x,pose.joints.y,pose.joints.z,pose.joints.c,pose.joints.b };
    auto options = makeSphereStudyOpticalOptions(q);
    const auto sim = mls::calibration::SphereDistanceAngleStudy::buildScanSimulation(
        pose.joints, m_sphereStudySphereFit.centerMachineMm, options.sphereRadiusMm,
        m_sphereStudyPlanningModel, options, 720);

    auto toMachine = [&](const std::vector<mls::calibration::Vec3d>& source) {
        QVector<QVector3D> points;
        points.reserve(static_cast<int>(source.size()));
        for (const auto& p : source) {
            const auto m = mls::calibration::SingleSphere17Model::mapSensorPoint(
                pose.joints, p, m_sphereStudyPlanningModel);
            if (std::isfinite(m.x) && std::isfinite(m.y) && std::isfinite(m.z))
                points.push_back(QVector3D(static_cast<float>(m.x), static_cast<float>(m.y), static_cast<float>(m.z)));
        }
        return points;
        };

    const QVector<QVector3D> laserFieldMachine = toMachine(sim.laserFieldCornersSensorMm);
    const QVector<QVector3D> sectionCircleMachine = toMachine(sim.sectionCircleSensorMm);
    const QVector<QVector3D> visibleArcMachine = toMachine(sim.visibleArcSensorMm);

    updateMachineDigitalTwinPose(q, true); // GUI-only: updates VTK actors, sends no controller command.

    if (sim.valid && !visibleArcMachine.isEmpty()) {
        m_sphereStudySimulationAcquiredCloud += visibleArcMachine;
        ++m_sphereStudySimulationAcquiredPoses;
        m_pointCloudView->setSphereStudySimulatedAcquisitionCloud(m_sphereStudySimulationAcquiredCloud);
    }
    else {
        ++m_sphereStudySimulationRejectedPoses;
    }

    const double fovNearReportedZ = m_sphereStudyPlanningFlipZ ? Constants::kSensorZMax : Constants::kSensorZMin;
    const double fovFarReportedZ = m_sphereStudyPlanningFlipZ ? Constants::kSensorZMin : Constants::kSensorZMax;
    m_pointCloudView->setSphereStudyScanSimulation(
        laserFieldMachine, sectionCircleMachine, visibleArcMachine, sim.valid,
        tr("#%1  Ys=%2 mm  截圆R=%3 mm  末端侧有效弧=%4°/%5 mm  相交裕量=%6 mm  %7  |  FOV近端小口Z=%8 / 远端大口Z=%9  |  累积=%10姿态/%11点")
        .arg(planIndex + 1)
        .arg(sim.optical.sphereCenterSensorMm.y, 0, 'f', 2)
        .arg(sim.optical.sectionRadiusMm, 0, 'f', 2)
        .arg(sim.optical.visibleArcDeg, 0, 'f', 1)
        .arg(sim.optical.visibleArcLengthMm, 0, 'f', 2)
        .arg(sim.optical.planeIntersectionMarginMm, 0, 'f', 2)
        .arg(sim.valid ? tr("通过") : tr("拒绝"))
        .arg(fovNearReportedZ, 0, 'f', 1)
        .arg(fovFarReportedZ, 0, 'f', 1)
        .arg(m_sphereStudySimulationAcquiredPoses)
        .arg(m_sphereStudySimulationAcquiredCloud.size()));
    if (selectRow && m_sphereStudyTable)m_sphereStudyTable->selectRow(planIndex);
    if (m_viewTabs)m_viewTabs->setCurrentWidget(m_pointCloudView);

    if (m_lblSphereStudyCapture)m_lblSphereStudyCapture->setText(
        tr("离线仿真采集 %1/%2：位姿 #%3，%4；末端侧有效弧=%5°/%6 mm，相交裕量=%7 mm；"
            "当前已累积 %8 个有效姿态、%9 个理想采样点（拒绝=%10）。")
        .arg(m_sphereStudySimulationQueuePos + 1)
        .arg(static_cast<int>(m_sphereStudySimulationQueue.size()))
        .arg(planIndex + 1).arg(sim.valid ? tr("模型预测可采并加入仿真点云") : tr("模型预测不可采，不加入点云"))
        .arg(sim.optical.visibleArcDeg, 0, 'f', 1).arg(sim.optical.visibleArcLengthMm, 0, 'f', 2)
        .arg(sim.optical.planeIntersectionMarginMm, 0, 'f', 2)
        .arg(m_sphereStudySimulationAcquiredPoses)
        .arg(m_sphereStudySimulationAcquiredCloud.size())
        .arg(m_sphereStudySimulationRejectedPoses));
    if (!sim.valid && error)*error = QString::fromStdString(sim.message);
    return true; // Rendering succeeded even when this pose is optically rejected.
}

void MainWindow::onSphereStudySimulationStart()
{
    if (m_sphereStudySimulationActive)return;
    if (m_sphereStudyPlan.empty() || !m_sphereStudySphereFit.success || !m_sphereStudyHasPlanningModel || !m_pointCloudView) {
        QMessageBox::warning(this, tr("扫描仿真"), tr("请先拟合球心并生成球面截平面族路径。"));
        return;
    }
    m_sphereStudySimulationQueue.clear();
    for (int i = 0; i < static_cast<int>(m_sphereStudyPlan.size()); ++i)
        if (m_sphereStudyPlan[static_cast<std::size_t>(i)].valid)m_sphereStudySimulationQueue.push_back(i);
    if (m_sphereStudySimulationQueue.empty()) {
        QMessageBox::warning(this, tr("扫描仿真"), tr("当前没有可用于仿真的有效规划位姿。"));
        return;
    }

    // A new playback represents one complete ideal acquisition.  Clear only the
    // previous synthetic result; the imported/fitted sphere cloud remains as the
    // reference surface underneath it.
    m_sphereStudySimulationAcquiredCloud.clear();
    m_sphereStudySimulationAcquiredPoses = 0;
    m_sphereStudySimulationRejectedPoses = 0;
    m_pointCloudView->clearSphereStudyScanSimulation();
    m_pointCloudView->clearSphereStudySimulatedAcquisitionCloud();

    m_sphereStudySimulationActive = true;
    m_sphereStudySimulationQueuePos = 0;
    ++m_sphereStudySimulationToken;
    if (m_lblSphereStudyCapture)m_lblSphereStudyCapture->setText(tr(
        "离线仿真自动采集已启动：控制器/传感器连接状态不会被使用；将逐姿态驱动数字孪生并把预测可见圆弧累积为三维仿真采集点云。"));
    updateSphereStudyUiState();
    advanceSphereStudySimulation();
}

void MainWindow::advanceSphereStudySimulation()
{
    if (!m_sphereStudySimulationActive)return;
    if (m_sphereStudySimulationQueuePos >= static_cast<int>(m_sphereStudySimulationQueue.size())) {
        finishSphereStudySimulation(tr(
            "离线扫描仿真完成：有效采集姿态=%1，拒绝姿态=%2，累积仿真点=%3。"
            "线激光瞬时平面已隐藏，仿真采集点云继续保留在三维窗口，可直接旋转/缩放观察覆盖结果。")
            .arg(m_sphereStudySimulationAcquiredPoses)
            .arg(m_sphereStudySimulationRejectedPoses)
            .arg(m_sphereStudySimulationAcquiredCloud.size()));
        return;
    }
    const quint64 token = m_sphereStudySimulationToken;
    const int index = m_sphereStudySimulationQueue[static_cast<std::size_t>(m_sphereStudySimulationQueuePos)];
    QString error;
    if (!showSphereStudySimulationPose(index, true, &error)) {
        finishSphereStudySimulation(tr("扫描仿真中止：%1").arg(error));
        return;
    }
    ++m_sphereStudySimulationQueuePos;
    const int interval = m_spinSphereStudySimulationIntervalMs ? m_spinSphereStudySimulationIntervalMs->value() : 800;
    QTimer::singleShot(interval, this, [this, token]() {
        if (m_sphereStudySimulationActive && token == m_sphereStudySimulationToken)advanceSphereStudySimulation();
        });
}

void MainWindow::finishSphereStudySimulation(const QString& reason, bool restoreActualPose)
{
    if (!m_sphereStudySimulationActive && m_sphereStudySimulationQueue.empty())return;
    m_sphereStudySimulationActive = false;
    m_sphereStudySimulationQueue.clear();
    m_sphereStudySimulationQueuePos = 0;
    ++m_sphereStudySimulationToken;
    // Remove the transient current laser plane/circle but deliberately retain the
    // accumulated synthetic acquisition point cloud for post-playback inspection.
    if (m_pointCloudView)m_pointCloudView->clearSphereStudyScanSimulation();
    if (restoreActualPose && m_zmc && m_zmc->isConnected()) {
        JointPose5D actual; QString readError;
        if (m_zmc->readJointPose5D(actual, &readError))updateMachineDigitalTwinPose(actual, true);
    }
    if (m_lblSphereStudyCapture)m_lblSphereStudyCapture->setText(reason);
    updateSphereStudyUiState();
}

void MainWindow::onSphereStudySimulationStop()
{
    const bool canRestore = m_zmc && m_zmc->isConnected();
    finishSphereStudySimulation(
        tr("离线扫描仿真已停止：已保留 %1 个仿真采集点；%2")
        .arg(m_sphereStudySimulationAcquiredCloud.size())
        .arg(canRestore ? tr("数字孪生已尝试恢复到当前机床MPOS。")
            : tr("当前未连接控制器，数字孪生停留在最后一个仿真姿态。")),
        canRestore);
}

void MainWindow::onSphereStudySimulationClear()
{
    if (m_sphereStudySimulationActive)return;
    m_sphereStudySimulationAcquiredCloud.clear();
    m_sphereStudySimulationAcquiredPoses = 0;
    m_sphereStudySimulationRejectedPoses = 0;
    if (m_pointCloudView) {
        m_pointCloudView->clearSphereStudyScanSimulation();
        m_pointCloudView->clearSphereStudySimulatedAcquisitionCloud();
    }
    if (m_lblSphereStudyCapture)m_lblSphereStudyCapture->setText(tr(
        "仿真采集点云已清除；导入的球面点云和已生成路径仍保留，可重新运行“仿真自动采集路径”。"));
    updateSphereStudyUiState();
}

void MainWindow::onSphereStudyMoveSelected()
{
    const int index = sphereStudySelectedPlanIndex();
    if (index < 0 || index >= static_cast<int>(m_sphereStudyPlan.size()))return;
    const auto& target = m_sphereStudyPlan[static_cast<std::size_t>(index)];
    if (!target.valid) { QMessageBox::warning(this, tr("研究位姿"), tr("该行规划无效，不能移动。")); return; }
    if (m_sphereStudyCaptureActive) { QMessageBox::warning(this, tr("研究位姿"), tr("正在采集，不能移动。")); return; }
    if (!m_zmc || !m_zmc->isConnected() || !m_zmc->areLinearMposReferenced()) { QMessageBox::warning(this, tr("研究位姿"), tr("XYZ光栅MPOS机床基准尚未建立。")); return; }
    if (m_syncManager && m_syncManager->isRunning()) { QMessageBox::warning(this, tr("研究位姿"), tr("同步扫描正在运行，请先停止。")); return; }
    if (m_gcodeExecutor && (m_gcodeExecutor->isRunning() || m_gcodeExecutor->isPaused())) { QMessageBox::warning(this, tr("研究位姿"), tr("G代码正在运行，请先停止。")); return; }
    if (!m_zmc->isRotaryServoEnabled(AxisC) || !m_zmc->isRotaryServoEnabled(AxisB)) { QMessageBox::warning(this, tr("研究位姿"), tr("五轴定位要求C/B已完成人工使能。")); return; }
    JointState5D state; QString error; if (!m_zmc->readJointState5D(state, &error) || !state.allIdle) { QMessageBox::warning(this, tr("研究位姿"), error.isEmpty() ? tr("至少一个轴仍在运动") : error); return; }
    JointPose5D q{ target.joints.x,target.joints.y,target.joints.z,target.joints.c,target.joints.b };
    QString safetyError;
    if (!checkSphereStudyPoseSafety(q, &safetyError, true) ||
        !checkSphereStudyTransitionSafety(state.mpos, q, &safetyError, true)) {
        QMessageBox::critical(this, tr("机床碰撞安全检查失败"), safetyError);
        updateMachineDigitalTwinPose(state.mpos, true);
        return;
    }
    const double scale = m_spinSphereStudySpeedScale->value() / 100.0;
    const JointMotionPlan5D plan = JointTrajectoryPlanner5Axis::planPointToPoint(state.mpos, q, currentAxisParams5D(), scale);
    if (!plan.valid) { QMessageBox::warning(this, tr("研究位姿规划失败"), plan.error); return; }
    QString confirm = tr("移动到球面研究位姿 #%1\n目标 D=%2 mm, γ=%3°, η=%4°\nXYZCB=[%5,%6,%7,%8,%9]\n速度倍率=%10%\n\n"
        "C/B由可达截平面族+空间填充自动选择。到位后最终D/γ/η均由实际RANSAC圆拟合与已知球心重新计算。")
        .arg(index + 1).arg(target.geometry.measurementDistanceMm, 0, 'f', 2).arg(target.geometry.sectionAngleDeg, 0, 'f', 2)
        .arg(target.geometry.centerBearingAngleDeg, 0, 'f', 2)
        .arg(q.x, 0, 'f', 3).arg(q.y, 0, 'f', 3).arg(q.z, 0, 'f', 3).arg(q.c, 0, 'f', 3).arg(q.b, 0, 'f', 3).arg(scale * 100.0, 0, 'f', 1);
    if (!m_sphereStudyPlanningModelValidated)confirm += tr("\n\n注意：当前Physical17只负责把机器送到目标附近，不把目标D/γ/η冒充为实测值。");
    if (QMessageBox::question(this, tr("确认研究位姿移动"), confirm, QMessageBox::Yes | QMessageBox::No, QMessageBox::No) != QMessageBox::Yes)return;

    setJointMoveUiRunning(true); ++m_axisWorkerCount; m_lblSphereStudyCapture->setText(tr("研究状态：正在移动到位姿 #%1...").arg(index + 1));
    ZmcController* controller = m_zmc.get();
    (void)QtConcurrent::run([this, controller, plan, index]() {
        JointMoveResult5D result; QString moveError; const bool ok = controller->moveJointAbs5D(plan, result, &moveError);
        QMetaObject::invokeMethod(this, [this, result, ok, moveError, index]() {
            --m_axisWorkerCount; setJointMoveUiRunning(false); pollAxisAlarmStates(false);
            if (!ok) { m_lblSphereStudyCapture->setText(tr("研究状态：位姿 #%1 移动失败：%2").arg(index + 1).arg(moveError)); if (!m_closeRequested)QMessageBox::warning(this, tr("研究位姿移动失败"), moveError); tryFinishDeferredClose(); return; }
            QString arrivalError;
            if (!sphereStudyArrivalWithinTolerance(result, &arrivalError)) {
                m_lblSphereStudyCapture->setText(tr("研究状态：位姿 #%1 未进入到位容差：%2").arg(index + 1).arg(arrivalError));
                if (!m_closeRequested)QMessageBox::warning(this, tr("研究位姿未可靠到位"), arrivalError);
                updateMachineDigitalTwinPose(result.finalState.mpos, true);
                tryFinishDeferredClose(); return;
            }
            const auto q = mls::calibration::JointPose5d{ result.finalState.mpos.x,result.finalState.mpos.y,result.finalState.mpos.z,result.finalState.mpos.c,result.finalState.mpos.b };
            const mls::calibration::Vec3d optical{ m_spinSphereStudyOpticalCenterX->value(),0.0,m_spinSphereStudyOpticalCenterZ->value() };
            const auto g = mls::calibration::SphereDistanceAngleStudy::observeGeometry(q, m_sphereStudySphereFit.centerMachineMm, 0.5 * m_spinSphereStudyDiameter->value(), m_sphereStudyPlanningModel, optical);
            if (g.valid) {
                m_lblSphereStudyCapture->setText(tr("研究状态：位姿 #%1 到位；模型估计 D≈%2 mm, γ≈%3°, η≈%4°。最终D/γ/η将在Rich Profile圆拟合后计算。")
                    .arg(index + 1).arg(g.measurementDistanceMm, 0, 'f', 3).arg(g.sectionAngleDeg, 0, 'f', 3)
                    .arg(g.centerBearingAngleDeg, 0, 'f', 3));
            }
            else m_lblSphereStudyCapture->setText(tr("研究状态：已到位；模型几何估计失败：%1，仍可通过实际轮廓检查后决定是否采集。")
                .arg(QString::fromStdString(g.message)));
            updateSphereStudyUiState(); tryFinishDeferredClose();
            }, Qt::QueuedConnection);
        });
}

void MainWindow::onSphereStudyStopMove()
{
    if (m_sphereStudyAutoActive) { onSphereStudyAutoStop(); return; }
    if (!m_jointMoveInProgress || !m_zmc)return;
    if (!m_zmc->requestStopJointMove5D(false))QMessageBox::warning(this, tr("停止失败"), tr("无法提交五轴减速停止请求。"));
    else if (m_lblSphereStudyCapture)m_lblSphereStudyCapture->setText(tr("研究状态：已请求五轴减速停止..."));
}

bool MainWindow::ensureSphereStudyProfileStream(QString* errMsg)
{
    if (!m_laser || !m_laser->isConnected()) { if (errMsg)*errMsg = tr("scanCONTROL未连接"); return false; }
    m_laser->setSensorCalibrationRichProfileEnabled(true);
    if (!m_profilePreviewRunning) {
        if (m_radioProfilePreviewMode)m_radioProfilePreviewMode->setChecked(true);
        onStartProfilePreview();
        if (!m_profilePreviewRunning) {
            m_laser->setSensorCalibrationRichProfileEnabled(false);
            if (errMsg)*errMsg = tr("无法启动Profile预览/采集流");
            return false;
        }
        if (m_sphereStudyAutoActive)m_sphereStudyAutoOwnsPreview = true;
    }
    return true;
}

bool MainWindow::makeSphereStudyRawProfile(const LaserProfile& profile, mls::calibration::SphereStudyRawProfile& raw) const
{
    if (!m_laser)return false;
    LaserSensor::SensorCalibrationProfileFrame rich;
    if (!m_laser->latestSensorCalibrationProfile(rich))return false;
    if (profile.callbackOrdinal != 0 && rich.profile.callbackOrdinal != profile.callbackOrdinal)return false;
    if (profile.callbackOrdinal == 0 && rich.profile.index != profile.index)return false;
    raw = {}; raw.callbackOrdinal = rich.profile.callbackOrdinal; raw.profileIndex = rich.profile.index; raw.tOpenSec = rich.profile.tOpen; raw.tCloseSec = rich.profile.tClose;
    const int n = std::min(rich.profile.x.size(), rich.profile.z.size()); raw.samples.reserve(static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i) {
        mls::calibration::SphereStudyRawSample s; s.index = static_cast<std::size_t>(i); s.xMm = rich.profile.x[i]; s.zMm = rich.profile.z[i];
        s.coordinateValid = SensorUtils::isValidPoint(s.xMm, s.zMm);
        if (rich.hasWidth() && i < rich.width.size()) { s.hasWidth = true; s.width = rich.width[i]; }
        if (rich.hasMaximum() && i < rich.maximum.size()) { s.hasMaximum = true; s.maximum = rich.maximum[i]; }
        if (rich.hasThreshold() && i < rich.threshold.size()) { s.hasThreshold = true; s.threshold = rich.threshold[i]; }
        if (rich.hasMoment0() && i < rich.moment0.size()) { s.hasMoment0 = true; s.moment0 = rich.moment0[i]; }
        if (rich.hasMoment1() && i < rich.moment1.size()) { s.hasMoment1 = true; s.moment1 = rich.moment1[i]; }
        raw.samples.push_back(s);
    }
    return !raw.samples.empty();
}

bool MainWindow::startSphereStudyCaptureGroup(QString* errMsg)
{
    if (!m_sphereStudyCaptureActive)return false;
    if (m_sphereStudyCaptureGroup >= m_sphereStudyCaptureGroupTarget) { finishSphereStudyCapture(false, tr("本姿态采集完成")); return true; }
    JointState5D state; QString error; if (!m_zmc->readJointState5D(state, &error)) { if (errMsg)*errMsg = error; return false; }
    if (!state.allIdle) { if (errMsg)*errMsg = tr("机床在组间发生运动；研究Profile只能在静止位姿采集"); return false; }
    SphereStudyCaptureRecord record; record.visitId = m_sphereStudyCurrentVisitId; record.planIndex = m_sphereStudyCapturePlanIndex; record.groupIndex = m_sphereStudyCaptureGroup; record.actualPose = state.mpos;
    if (record.planIndex >= 0 && record.planIndex < static_cast<int>(m_sphereStudyAutoProbeFrames.size())) {
        record.probeFrameCount = m_sphereStudyAutoProbeFrames[static_cast<std::size_t>(record.planIndex)];
        record.probeDetectedCircleCount = m_sphereStudyAutoProbeDetectedCircles[static_cast<std::size_t>(record.planIndex)];
        record.reacquireAttemptCount = m_sphereStudyAutoReacquireAttempts[static_cast<std::size_t>(record.planIndex)];
        record.reacquireNormalOffsetMm = m_sphereStudyAutoReacquireOffsetsMm[static_cast<std::size_t>(record.planIndex)];
        if (record.planIndex < static_cast<int>(m_sphereStudyAutoReacquireSensorOffsetsMm.size()))
            record.reacquireSensorOffsetMm = m_sphereStudyAutoReacquireSensorOffsetsMm[static_cast<std::size_t>(record.planIndex)];
        record.reacquired = record.reacquireAttemptCount > 0;
    }
    const mls::calibration::Vec3d optical{ m_spinSphereStudyOpticalCenterX->value(),0.0,m_spinSphereStudyOpticalCenterZ->value() };
    record.poseGeometryEstimate = mls::calibration::SphereDistanceAngleStudy::observeGeometry(
        { state.mpos.x,state.mpos.y,state.mpos.z,state.mpos.c,state.mpos.b }, m_sphereStudySphereFit.centerMachineMm,
        0.5 * m_spinSphereStudyDiameter->value(), m_sphereStudyPlanningModel, optical);
    m_sphereStudyRecords.push_back(std::move(record));
    m_sphereStudyCaptureDiscardRemaining = m_spinSphereStudyDiscard->value();
    const auto& g = m_sphereStudyRecords.back().poseGeometryEstimate;
    m_lblSphereStudyCapture->setText(g.valid
        ? tr("研究状态：Visit %1，位姿 #%2；模型估计D≈%3 mm, γ≈%4°, η≈%5°；先丢弃%6帧，再采%7条Rich Profile。")
        .arg(m_sphereStudyCurrentVisitId).arg(m_sphereStudyCapturePlanIndex + 1)
        .arg(g.measurementDistanceMm, 0, 'f', 3).arg(g.sectionAngleDeg, 0, 'f', 3)
        .arg(g.centerBearingAngleDeg, 0, 'f', 3)
        .arg(m_sphereStudyCaptureDiscardRemaining).arg(m_sphereStudyCaptureProfilesTarget)
        : tr("研究状态：Visit %1，位姿 #%2；开始Rich Profile采集（最终几何由数据分析确定）。")
        .arg(m_sphereStudyCurrentVisitId).arg(m_sphereStudyCapturePlanIndex + 1));

    // Automatic workflow watchdog.  A missing callback must not stall a
    // hundreds-pose experiment forever.  Manual capture remains un-timed.
    if (m_sphereStudyAutoActive && m_spinSphereStudyAutoTimeoutMs) {
        const quint64 token = ++m_sphereStudyCaptureWatchToken;
        const int visit = m_sphereStudyCurrentVisitId;
        const int group = m_sphereStudyCaptureGroup;
        const int timeoutMs = m_spinSphereStudyAutoTimeoutMs->value();
        QTimer::singleShot(timeoutMs, this, [this, token, visit, group, timeoutMs]() {
            if (!m_sphereStudyAutoActive || !m_sphereStudyCaptureActive)return;
            if (token != m_sphereStudyCaptureWatchToken || visit != m_sphereStudyCurrentVisitId || group != m_sphereStudyCaptureGroup)return;
            // Preserve partial raw data as a failed/partial group before retrying
            // or continuing; it remains valuable evidence of sensor dropouts.
            if (!m_sphereStudyRecords.empty() && !m_sphereStudyRecords.back().profiles.empty()) {
                QString saveError;
                (void)writeSphereStudyRawGroup(static_cast<int>(m_sphereStudyRecords.size()) - 1, &saveError);
            }
            finishSphereStudyCapture(true, tr("Rich Profile采集超时（%1 ms）").arg(timeoutMs));
            });
    }
    return true;
}

bool MainWindow::beginSphereStudyCaptureAtPlanIndex(int index, bool interactive, QString* errMsg)
{
    auto fail = [&](const QString& text)->bool {if (errMsg)*errMsg = text; return false; };
    if (index < 0 || index >= static_cast<int>(m_sphereStudyPlan.size()) || !m_sphereStudyPlan[static_cast<std::size_t>(index)].valid)
        return fail(tr("研究位姿无效。"));
    if (m_sphereStudyCaptureActive || m_jointMoveInProgress)return fail(tr("当前仍有研究采集或五轴运动任务。"));
    if (!m_sphereStudyHasPlanningModel || !m_sphereStudySphereFit.success)return fail(tr("请先完成球心拟合并生成研究路径。"));
    if (!m_zmc || !m_zmc->isConnected() || !m_zmc->areLinearMposReferenced())return fail(tr("XYZ光栅MPOS尚未通过正规回零建立机床基准。"));
    if (!m_zmc->isRotaryAbsoluteReady(AxisC) || !m_zmc->isRotaryAbsoluteReady(AxisB))return fail(tr("B/C绝对机械坐标尚未建立。"));
    if (!m_zmc->isRotaryServoEnabled(AxisC) || !m_zmc->isRotaryServoEnabled(AxisB))return fail(tr("C/B尚未人工使能。"));
    if (!m_laser || !m_laser->isConnected())return fail(tr("请先连接scanCONTROL传感器。"));
    if (m_syncManager && m_syncManager->isRunning())return fail(tr("请先停止正式硬件/软件同步扫描。研究位姿采用静止Rich Profile重复采样。"));
    if (m_laser->isVideoRunning() || m_expTestState != ExpTestState::Idle)return fail(tr("请先停止Video/曝光测试。"));
    if (m_gcodeExecutor && (m_gcodeExecutor->isRunning() || m_gcodeExecutor->isPaused()))return fail(tr("G代码正在运行，请先停止。"));

    JointState5D actualState; QString error;
    if (!m_zmc->readJointState5D(actualState, &error))return fail(error.isEmpty() ? tr("读取XYZCB实际位姿失败。") : error);
    if (!actualState.allIdle)return fail(tr("机床仍在运动。研究采集必须在静止位姿执行。"));

    const mls::calibration::Vec3d optical{ m_spinSphereStudyOpticalCenterX->value(),0.0,m_spinSphereStudyOpticalCenterZ->value() };
    const auto estimate = mls::calibration::SphereDistanceAngleStudy::observeGeometry(
        { actualState.mpos.x,actualState.mpos.y,actualState.mpos.z,actualState.mpos.c,actualState.mpos.b }, m_sphereStudySphereFit.centerMachineMm,
        0.5 * m_spinSphereStudyDiameter->value(), m_sphereStudyPlanningModel, optical);
    const auto& target = m_sphereStudyPlan[static_cast<std::size_t>(index)].geometry;
    if (interactive && estimate.valid) {
        const double dErr = std::abs(estimate.measurementDistanceMm - target.measurementDistanceMm);
        const double gErr = std::abs(estimate.sectionAngleDeg - target.sectionAngleDeg);
        const double eErr = std::abs(estimate.centerBearingAngleDeg - target.centerBearingAngleDeg);
        if (dErr > 1.0 || gErr > 3.0 || eErr > 3.0) {
            if (QMessageBox::question(this, tr("模型估计偏离目标"), tr("目标 D=%1 mm, γ=%2°, η=%3°；当前模型估计 D≈%4 mm, γ≈%5°, η≈%6°。\n"
                "这里不要求机床精确复现目标；继续采集时最终分析使用RANSAC拟合得到的D/η和实际平面γ。继续？")
                .arg(target.measurementDistanceMm, 0, 'f', 3).arg(target.sectionAngleDeg, 0, 'f', 3).arg(target.centerBearingAngleDeg, 0, 'f', 3)
                .arg(estimate.measurementDistanceMm, 0, 'f', 3).arg(estimate.sectionAngleDeg, 0, 'f', 3).arg(estimate.centerBearingAngleDeg, 0, 'f', 3), QMessageBox::Yes | QMessageBox::No, QMessageBox::No) != QMessageBox::Yes)
                return fail(tr("用户取消采集。"));
        }
    }

    // Rich channels are opt-in.  Enable before preview so every original sample
    // slot can retain Width/Maximum/Threshold/M0/M1.  No signal-quality filter
    // is applied before Circle RANSAC.
    m_laser->setSensorCalibrationRichProfileEnabled(true);
    m_sphereStudyStartedPreview = !m_profilePreviewRunning;
    if (!m_profilePreviewRunning) {
        if (m_radioProfilePreviewMode)m_radioProfilePreviewMode->setChecked(true);
        onStartProfilePreview();
        if (!m_profilePreviewRunning) {
            m_laser->setSensorCalibrationRichProfileEnabled(false);
            m_sphereStudyStartedPreview = false;
            return fail(tr("无法启动Profile预览/采集流。"));
        }
        if (m_sphereStudyAutoActive)m_sphereStudyAutoOwnsPreview = true;
    }
    if (m_sphereStudySessionDir.isEmpty()) {
        const QString base = m_editSphereStudyDir ? m_editSphereStudyDir->text() : QDir::currentPath();
        m_sphereStudySessionDir = QDir(base).filePath(QStringLiteral("session_%1").arg(QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss")));
        if (!QDir().mkpath(m_sphereStudySessionDir)) {
            if (m_sphereStudyStartedPreview)onStopProfilePreview();
            m_laser->setSensorCalibrationRichProfileEnabled(false); m_sphereStudyStartedPreview = false;
            return fail(tr("无法创建研究目录：%1").arg(m_sphereStudySessionDir));
        }
    }
    m_sphereStudyCaptureActive = true; m_sphereStudyCapturePlanIndex = index; m_sphereStudyCaptureGroup = 0;
    const int configuredGroups = m_spinSphereStudyGroups->value();
    if (m_sphereStudyAutoActive && m_checkSphereStudyAutoSkipCompleted && m_checkSphereStudyAutoSkipCompleted->isChecked()) {
        // Resume an interrupted/partially completed research pose instead of
        // silently collecting another full configured set.  Only groups that
        // were completely written count toward the configured target.
        const int completedBefore = sphereStudyCapturedGroupCount(index);
        m_sphereStudyCaptureGroupTarget = std::max(0, configuredGroups - completedBefore);
    }
    else {
        m_sphereStudyCaptureGroupTarget = configuredGroups;
    }
    m_sphereStudyCaptureProfilesTarget = m_spinSphereStudyProfilesPerGroup->value();
    if (m_sphereStudyCaptureGroupTarget <= 0) {
        m_sphereStudyCaptureActive = false; m_sphereStudyCapturePlanIndex = -1; m_sphereStudyCurrentVisitId = -1;
        if (m_sphereStudyProfileConnection) { QObject::disconnect(m_sphereStudyProfileConnection); m_sphereStudyProfileConnection = {}; }
        return fail(tr("该位姿已经完成本轮Profile采集，无需再次采集。"));
    }
    m_sphereStudyCaptureDiscardRemaining = -1; m_sphereStudyLastProfileOrdinal = 0;
    m_sphereStudyCurrentVisitId = ++m_sphereStudyVisitCounter;
    if (m_sphereStudyProfileConnection)QObject::disconnect(m_sphereStudyProfileConnection);
    m_sphereStudyProfileConnection = connect(m_laser.get(), &LaserSensor::profileReceived, this, [this](const LaserProfile& p) {onSphereStudyProfileFrame(p); }, Qt::QueuedConnection);
    if (!startSphereStudyCaptureGroup(&error)) {
        finishSphereStudyCapture(true, error);
        return fail(error);
    }
    updateSphereStudyUiState(); updateVideoControlState();
    return true;
}

void MainWindow::onSphereStudyCaptureSelected()
{
    const int index = sphereStudySelectedPlanIndex();
    if (index < 0)return;
    QString error;
    if (!beginSphereStudyCaptureAtPlanIndex(index, true, &error) && !error.isEmpty() && error != tr("用户取消采集。"))
        QMessageBox::warning(this, tr("研究采集启动失败"), error);
}

void MainWindow::onSphereStudyProfileFrame(const LaserProfile& profile)
{
    if (!m_sphereStudyCaptureActive || !m_laser)return;
    if (profile.callbackOrdinal != 0) { if (profile.callbackOrdinal == m_sphereStudyLastProfileOrdinal)return; m_sphereStudyLastProfileOrdinal = profile.callbackOrdinal; }
    if (m_sphereStudyCaptureDiscardRemaining < 0)return;
    if (m_sphereStudyCaptureDiscardRemaining > 0) { --m_sphereStudyCaptureDiscardRemaining; return; }
    if (m_sphereStudyRecords.empty())return;
    auto& record = m_sphereStudyRecords.back();
    if (record.planIndex != m_sphereStudyCapturePlanIndex || record.groupIndex != m_sphereStudyCaptureGroup || record.visitId != m_sphereStudyCurrentVisitId)return;

    mls::calibration::SphereStudyRawProfile raw;
    if (!makeSphereStudyRawProfile(profile, raw))return;
    record.profiles.push_back(std::move(raw));
    m_lblSphereStudyCapture->setText(tr("研究状态：Visit %1，位姿 #%2，已采 %3/%4 条Rich Profile")
        .arg(record.visitId).arg(m_sphereStudyCapturePlanIndex + 1)
        .arg(record.profiles.size()).arg(m_sphereStudyCaptureProfilesTarget));
    if (record.profiles.size() < static_cast<std::size_t>(m_sphereStudyCaptureProfilesTarget))return;

    m_sphereStudyCaptureDiscardRemaining = -1; QString saveError; const int recordIndex = static_cast<int>(m_sphereStudyRecords.size()) - 1;
    record.complete = true;
    ++m_sphereStudyCaptureWatchToken; // cancel this group's automatic watchdog
    if (!writeSphereStudyRawGroup(recordIndex, &saveError)) { finishSphereStudyCapture(true, saveError); if (!m_closeRequested && !m_sphereStudyAutoActive)QMessageBox::warning(this, tr("研究数据保存失败"), saveError); return; }
    ++m_sphereStudyCaptureGroup; rebuildSphereStudyPlanTable();
    if (m_sphereStudyCaptureGroup >= m_sphereStudyCaptureGroupTarget) { finishSphereStudyCapture(false, tr("选定位姿20条Profile采集完成")); return; }
    QTimer::singleShot(250, this, [this]() {if (!m_sphereStudyCaptureActive)return; QString error; if (!startSphereStudyCaptureGroup(&error)) { finishSphereStudyCapture(true, error); if (!m_closeRequested && !m_sphereStudyAutoActive)QMessageBox::warning(this, tr("研究采集失败"), error); }});
}

bool MainWindow::writeSphereStudyRawGroup(int recordIndex, QString* errMsg) const
{
    if (recordIndex < 0 || recordIndex >= static_cast<int>(m_sphereStudyRecords.size()))return false;
    const auto& r = m_sphereStudyRecords[static_cast<std::size_t>(recordIndex)];
    if (r.planIndex < 0 || r.planIndex >= static_cast<int>(m_sphereStudyPlan.size()))return false;
    const auto& target = m_sphereStudyPlan[static_cast<std::size_t>(r.planIndex)];
    QString gammaTag = QString::number(target.geometry.sectionAngleDeg, 'f', 1); gammaTag.replace('.', '_');
    QString etaTag = QString::number(target.geometry.centerBearingAngleDeg, 'f', 1); etaTag.replace('.', '_');
    const QString poseDir = QDir(m_sphereStudySessionDir).filePath(QStringLiteral("pose_%1_D%2_G%3_E%4_C%5_B%6")
        .arg(r.planIndex + 1, 3, 10, QLatin1Char('0')).arg(target.geometry.measurementDistanceMm, 0, 'f', 1).arg(gammaTag)
        .arg(etaTag).arg(target.joints.c, 0, 'f', 1).arg(target.joints.b, 0, 'f', 1));
    const QString visitDir = QDir(poseDir).filePath(QStringLiteral("visit_%1").arg(r.visitId, 3, 10, QLatin1Char('0')));
    if (!QDir().mkpath(visitDir)) { if (errMsg)*errMsg = tr("无法创建目录：%1").arg(visitDir); return false; }
    const QString fileName = QDir(visitDir).filePath(QStringLiteral("rich_profiles.csv"));
    QFile file(fileName); if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) { if (errMsg)*errMsg = file.errorString(); return false; }
    QTextStream out(&file); out.setRealNumberNotation(QTextStream::SmartNotation); out.setRealNumberPrecision(15);
    out << "# Schema: SphereDistanceAngleStudy.RawRichProfile.v4\n";
    out << "# VisitId: " << r.visitId << "\n# GroupComplete: " << (r.complete ? 1 : 0) << "\n# CapturedProfiles: " << r.profiles.size() << "\n# TargetDistance_mm: " << target.geometry.measurementDistanceMm << "\n# TargetSectionAngle_deg: " << target.geometry.sectionAngleDeg << "\n# TargetCenterBearing_deg: " << target.geometry.centerBearingAngleDeg << "\n";
    out << "# ActualXYZCB_MPOS: " << r.actualPose.x << "," << r.actualPose.y << "," << r.actualPose.z << "," << r.actualPose.c << "," << r.actualPose.b << "\n";
    out << "# SphereCenterMachine_mm: " << m_sphereStudySphereFit.centerMachineMm.x << "," << m_sphereStudySphereFit.centerMachineMm.y << "," << m_sphereStudySphereFit.centerMachineMm.z << "\n";
    out << "# Important: every original sample slot is retained; coordinate_valid=0 is a measurement dropout, not a deleted row.\n";
    out << "profile_ordinal,profile_index,point_index,coordinate_valid,sensor_x_mm,sensor_z_mm,width,maximum,threshold,moment0,moment1,t_open_s,t_close_s\n";
    auto writeMaybe = [&out](bool has, double v) {if (has && std::isfinite(v))out << v; };
    for (const auto& prof : r.profiles)for (const auto& s : prof.samples) {
        out << prof.callbackOrdinal << "," << prof.profileIndex << "," << s.index << "," << (s.coordinateValid ? 1 : 0) << ",";
        if (std::isfinite(s.xMm))out << s.xMm; out << ","; if (std::isfinite(s.zMm))out << s.zMm; out << ",";
        writeMaybe(s.hasWidth, s.width); out << ","; writeMaybe(s.hasMaximum, s.maximum); out << ","; writeMaybe(s.hasThreshold, s.threshold); out << ",";
        writeMaybe(s.hasMoment0, s.moment0); out << ","; writeMaybe(s.hasMoment1, s.moment1); out << "," << prof.tOpenSec << "," << prof.tCloseSec << "\n";
    }
    return true;
}

void MainWindow::onSphereStudyAutoStart()
{
    if (m_sphereStudyAutoActive)return;
    if (m_sphereStudyPlan.empty() || !m_sphereStudySphereFit.success || !m_sphereStudyHasPlanningModel) {
        QMessageBox::warning(this, tr("自动研究采集"), tr("请先拟合球心并生成有效的球面截平面族研究路径。")); return;
    }
    if (!m_zmc || !m_zmc->isConnected() || !m_zmc->areLinearMposReferenced()
        || !m_zmc->isRotaryAbsoluteReady(AxisC) || !m_zmc->isRotaryAbsoluteReady(AxisB)) {
        QMessageBox::warning(this, tr("自动研究采集"), tr("请先完成XYZ正规回零以及B/C绝对机械坐标建立。")); return;
    }
    if (!m_zmc->isRotaryServoEnabled(AxisC) || !m_zmc->isRotaryServoEnabled(AxisB)) {
        QMessageBox::warning(this, tr("自动研究采集"), tr("自动五轴路径执行前必须人工使能C/B。")); return;
    }
    if (!m_laser || !m_laser->isConnected()) {
        QMessageBox::warning(this, tr("自动研究采集"), tr("请先连接scanCONTROL传感器。")); return;
    }
    if ((m_syncManager && m_syncManager->isRunning()) || m_expTestState != ExpTestState::Idle
        || m_laser->isVideoRunning() || (m_gcodeExecutor && (m_gcodeExecutor->isRunning() || m_gcodeExecutor->isPaused()))) {
        QMessageBox::warning(this, tr("自动研究采集"), tr("请先停止正式同步扫描、Video、曝光测试和G代码任务。")); return;
    }
    JointState5D state; QString stateError;
    if (!m_zmc->readJointState5D(state, &stateError) || !state.allIdle) {
        QMessageBox::warning(this, tr("自动研究采集"), stateError.isEmpty() ? tr("至少一个轴仍在运动。") : stateError); return;
    }

    m_sphereStudyAutoQueue.clear();
    const bool skipCompleted = m_checkSphereStudyAutoSkipCompleted && m_checkSphereStudyAutoSkipCompleted->isChecked();
    const int groupsPerPose = m_spinSphereStudyGroups ? m_spinSphereStudyGroups->value() : 1;
    for (int i = 0; i < static_cast<int>(m_sphereStudyPlan.size()); ++i) {
        if (!m_sphereStudyPlan[static_cast<std::size_t>(i)].valid)continue;
        if (skipCompleted && sphereStudyCapturedGroupCount(i) >= groupsPerPose)continue;
        m_sphereStudyAutoQueue.push_back(i);
    }
    if (m_sphereStudyAutoQueue.empty()) {
        QMessageBox::information(this, tr("自动研究采集"), tr("没有需要自动采集的有效位姿。若要重新采集已完成位姿，请取消“跳过已完成位姿”。")); return;
    }

    const qulonglong poseCount = static_cast<qulonglong>(m_sphereStudyAutoQueue.size());
    const qulonglong profilesPerPose = static_cast<qulonglong>(m_spinSphereStudyProfilesPerGroup ? m_spinSphereStudyProfilesPerGroup->value() : 0);
    const qulonglong totalProfiles = poseCount * profilesPerPose;
    const QString confirm = tr("将自动执行 %1 个五轴研究位姿。\n\n"
        "流程：移动前再次做球-激光平面/梯形FOV/末端侧有效弧仿真门禁 → 移动 → 等待 %2 ms → 到位Probe/Circle RANSAC → 必要时Sensor局部X/Y/Z自动找球 → "
        "丢弃 %3 帧 → 正式采集 %4 条Rich Profile → 保存 → 下一位姿。\n"
        "预计正式保存约 %5 条Profile（不含Probe和丢弃帧）。\n\n"
        "路径生成阶段已经正向仿真球-激光平面相交，并使用程序实际X/Z测量范围筛选预测可见圆弧；"
        "实机Probe是第二道保险。是否开始？")
        .arg(poseCount).arg(m_spinSphereStudySettleMs ? m_spinSphereStudySettleMs->value() : 0)
        .arg(m_spinSphereStudyDiscard ? m_spinSphereStudyDiscard->value() : 0)
        .arg(profilesPerPose).arg(totalProfiles);
    if (QMessageBox::question(this, tr("确认自动研究采集"), confirm, QMessageBox::Yes | QMessageBox::No, QMessageBox::No) != QMessageBox::Yes)return;

    if (m_sphereStudyAutoPlanNotes.size() != m_sphereStudyPlan.size())m_sphereStudyAutoPlanNotes.assign(m_sphereStudyPlan.size(), QString());
    for (int idx : m_sphereStudyAutoQueue)m_sphereStudyAutoPlanNotes[static_cast<std::size_t>(idx)].clear();
    m_sphereStudyAutoReacquireAttempts.assign(m_sphereStudyPlan.size(), 0);
    m_sphereStudyAutoReacquireOffsetsMm.assign(m_sphereStudyPlan.size(), 0.0);
    m_sphereStudyAutoReacquireSensorOffsetsMm.assign(m_sphereStudyPlan.size(), mls::calibration::Vec3d{});
    m_sphereStudyAutoProbeDetectedCircles.assign(m_sphereStudyPlan.size(), 0);
    m_sphereStudyAutoProbeFrames.assign(m_sphereStudyPlan.size(), 0);
    m_sphereStudyReacquireAttempt = 0; m_sphereStudyAppliedNormalOffsetMm = 0.0; m_sphereStudyAppliedSensorOffsetMm = {};
    m_sphereStudyAutoActive = true;
    m_sphereStudyAutoPauseRequested = false;
    m_sphereStudyAutoPaused = false;
    m_sphereStudyAutoStopRequested = false;
    m_sphereStudyAutoPoseReady = false;
    m_sphereStudyAutoQueuePos = 0;
    m_sphereStudyAutoCurrentRetry = 0;
    m_sphereStudyAutoCompleted = 0;
    m_sphereStudyAutoFailed = 0;
    ++m_sphereStudyAutoSequenceToken;
    if (m_lblSphereStudyCapture)m_lblSphereStudyCapture->setText(tr("自动研究：已建立 %1 个位姿任务，准备开始...").arg(poseCount));
    setJointMoveUiRunning(false); // lock manual motion controls for the whole automatic workflow
    updateSphereStudyUiState();
    QTimer::singleShot(0, this, &MainWindow::advanceSphereStudyAutoSequence);
}

void MainWindow::onSphereStudyAutoPauseResume()
{
    if (!m_sphereStudyAutoActive)return;
    ++m_sphereStudyAutoSequenceToken; // cancel a pending settle timer
    if (m_sphereStudyAutoPaused || m_sphereStudyAutoPauseRequested) {
        m_sphereStudyAutoPauseRequested = false;
        m_sphereStudyAutoPaused = false;
        if (m_lblSphereStudyCapture)m_lblSphereStudyCapture->setText(tr("自动研究：继续执行。"));
        updateSphereStudyUiState();
        QTimer::singleShot(0, this, &MainWindow::advanceSphereStudyAutoSequence);
        return;
    }
    m_sphereStudyAutoPauseRequested = true;
    if (!m_jointMoveInProgress && !m_sphereStudyCaptureActive && !m_sphereStudyProbeActive) {
        m_sphereStudyAutoPaused = true;
        if (m_lblSphereStudyCapture)m_lblSphereStudyCapture->setText(
            m_sphereStudyAutoPoseReady ? tr("自动研究：已在当前位姿暂停；继续后将重新等待稳定并开始采样。")
            : tr("自动研究：已暂停；继续后执行下一位姿。"));
    }
    else if (m_lblSphereStudyCapture) {
        m_lblSphereStudyCapture->setText(tr("自动研究：已请求暂停，将在当前移动、Probe或当前位姿完整采集结束后暂停。"));
    }
    updateSphereStudyUiState();
}

void MainWindow::onSphereStudyAutoStop()
{
    if (!m_sphereStudyAutoActive)return;
    m_sphereStudyAutoStopRequested = true;
    m_sphereStudyAutoPauseRequested = false;
    m_sphereStudyAutoPaused = false;
    ++m_sphereStudyAutoSequenceToken;
    if (m_lblSphereStudyCapture)m_lblSphereStudyCapture->setText(tr("自动研究：正在停止..."));
    updateSphereStudyUiState();

    if (m_jointMoveInProgress && m_zmc) {
        (void)m_zmc->requestStopJointMove5D(false);
        return; // worker callback will close the automatic session
    }
    if (m_sphereStudyProbeActive) {
        ++m_sphereStudyProbeWatchToken;
        m_sphereStudyProbeActive = false;
        if (m_sphereStudyProbeConnection) { QObject::disconnect(m_sphereStudyProbeConnection); m_sphereStudyProbeConnection = {}; }
        m_sphereStudyProbeProfiles.clear(); m_sphereStudyProbePlanIndex = -1;
        finishSphereStudyAuto(true, tr("用户停止自动研究Probe"));
        return;
    }
    if (m_sphereStudyCaptureActive) {
        // Save any partial group before tearing down the direct profile stream.
        if (!m_sphereStudyRecords.empty() && !m_sphereStudyRecords.back().profiles.empty()) {
            QString ignored;
            (void)writeSphereStudyRawGroup(static_cast<int>(m_sphereStudyRecords.size()) - 1, &ignored);
        }
        finishSphereStudyCapture(true, tr("用户停止自动研究采集"));
        return;
    }
    finishSphereStudyAuto(true, tr("用户停止自动研究采集"));
}

void MainWindow::advanceSphereStudyAutoSequence()
{
    if (!m_sphereStudyAutoActive)return;
    if (m_sphereStudyAutoStopRequested) { finishSphereStudyAuto(true, tr("用户停止自动研究采集")); return; }
    if (m_sphereStudyAutoPauseRequested || m_sphereStudyAutoPaused) {
        m_sphereStudyAutoPaused = true; updateSphereStudyUiState(); return;
    }
    if (m_sphereStudyAutoQueuePos >= static_cast<int>(m_sphereStudyAutoQueue.size())) {
        finishSphereStudyAuto(false, tr("全部自动研究位姿采集完成")); return;
    }

    const int index = m_sphereStudyAutoQueue[static_cast<std::size_t>(m_sphereStudyAutoQueuePos)];
    if (index < 0 || index >= static_cast<int>(m_sphereStudyPlan.size()) || !m_sphereStudyPlan[static_cast<std::size_t>(index)].valid) {
        handleSphereStudyAutoStepFailure(tr("自动队列中的研究位姿无效")); return;
    }
    if (m_sphereStudyTable)m_sphereStudyTable->selectRow(index);

    // If the move already completed (for example Pause was requested just after
    // arrival), resume from the settle/capture phase without needlessly moving
    // away and back to the same nominal joint coordinates.
    if (m_sphereStudyAutoPoseReady) {
        const int settleMs = m_spinSphereStudySettleMs ? m_spinSphereStudySettleMs->value() : 0;
        const quint64 token = ++m_sphereStudyAutoSequenceToken;
        m_sphereStudyAutoPlanNotes[static_cast<std::size_t>(index)] = tr("自动到位，稳定等待");
        rebuildSphereStudyPlanTable();
        if (m_lblSphereStudyCapture)m_lblSphereStudyCapture->setText(tr("自动研究 %1/%2：位姿 #%3 已到位，等待稳定 %4 ms 后自动采样。")
            .arg(m_sphereStudyAutoQueuePos + 1).arg(static_cast<int>(m_sphereStudyAutoQueue.size())).arg(index + 1).arg(settleMs));
        QTimer::singleShot(settleMs, this, [this, token, index]() {
            if (!m_sphereStudyAutoActive || token != m_sphereStudyAutoSequenceToken)return;
            if (m_sphereStudyAutoStopRequested) { finishSphereStudyAuto(true, tr("用户停止自动研究采集")); return; }
            if (m_sphereStudyAutoPauseRequested) { m_sphereStudyAutoPaused = true; updateSphereStudyUiState(); return; }
            startSphereStudyAutoProbeCurrent(index);
            });
        return;
    }
    startSphereStudyAutoMoveCurrent();
}

void MainWindow::startSphereStudyAutoProbeCurrent(int planIndex)
{
    if (!m_sphereStudyAutoActive || planIndex < 0 || planIndex >= static_cast<int>(m_sphereStudyPlan.size()))return;
    if (m_sphereStudyProbeActive)return;
    QString error;
    if (!ensureSphereStudyProfileStream(&error)) { handleSphereStudyAutoStepFailure(error); return; }
    JointState5D state;
    if (!m_zmc || !m_zmc->readJointState5D(state, &error) || !state.allIdle) {
        handleSphereStudyAutoStepFailure(error.isEmpty() ? tr("Probe前至少一个轴未停止") : error); return;
    }

    m_sphereStudyProbeActualPose = { state.mpos.x,state.mpos.y,state.mpos.z,state.mpos.c,state.mpos.b };
    m_sphereStudyProbeProfiles.clear();
    m_sphereStudyProbePlanIndex = planIndex;
    m_sphereStudyProbeDiscardRemaining = 1;
    m_sphereStudyProbeLastOrdinal = 0;
    m_sphereStudyProbeActive = true;
    ++m_sphereStudyProbeWatchToken;
    const quint64 token = m_sphereStudyProbeWatchToken;
    if (m_sphereStudyProbeConnection)QObject::disconnect(m_sphereStudyProbeConnection);
    m_sphereStudyProbeConnection = connect(m_laser.get(), &LaserSensor::profileReceived, this,
        [this](const LaserProfile& p) {onSphereStudyProbeProfileFrame(p); }, Qt::QueuedConnection);
    const int target = m_spinSphereStudyProbeProfiles ? m_spinSphereStudyProbeProfiles->value() : 5;
    m_sphereStudyAutoPlanNotes[static_cast<std::size_t>(planIndex)] = tr("到位Probe：确认真实球圆弧");
    rebuildSphereStudyPlanTable();
    if (m_lblSphereStudyCapture)m_lblSphereStudyCapture->setText(tr(
        "自动研究：位姿 #%1 到位，先采 %2 帧Probe并做Circle RANSAC；通过后才正式采20条Profile。")
        .arg(planIndex + 1).arg(target));
    const int timeoutMs = std::max(3000, std::min(10000, m_spinSphereStudyAutoTimeoutMs ? m_spinSphereStudyAutoTimeoutMs->value() : 5000));
    QTimer::singleShot(timeoutMs, this, [this, token]() {
        if (m_sphereStudyProbeActive && token == m_sphereStudyProbeWatchToken)finishSphereStudyAutoProbe(true);
        });
}

void MainWindow::onSphereStudyProbeProfileFrame(const LaserProfile& profile)
{
    if (!m_sphereStudyProbeActive || !m_laser)return;
    if (profile.callbackOrdinal != 0) {
        if (profile.callbackOrdinal == m_sphereStudyProbeLastOrdinal)return;
        m_sphereStudyProbeLastOrdinal = profile.callbackOrdinal;
    }
    if (m_sphereStudyProbeDiscardRemaining > 0) { --m_sphereStudyProbeDiscardRemaining; return; }
    mls::calibration::SphereStudyRawProfile raw;
    if (!makeSphereStudyRawProfile(profile, raw))return;
    m_sphereStudyProbeProfiles.push_back(std::move(raw));
    const int target = m_spinSphereStudyProbeProfiles ? m_spinSphereStudyProbeProfiles->value() : 5;
    if (m_lblSphereStudyCapture)m_lblSphereStudyCapture->setText(tr("自动研究：Probe %1/%2 帧，正在确认球圆弧...")
        .arg(static_cast<int>(m_sphereStudyProbeProfiles.size())).arg(target));
    if (static_cast<int>(m_sphereStudyProbeProfiles.size()) >= target)finishSphereStudyAutoProbe(false);
}

mls::calibration::Vec3d MainWindow::sphereStudyReacquireOffsetForAttempt(int attempt) const
{
    const double step = m_spinSphereStudyReacquireStepMm ? m_spinSphereStudyReacquireStepMm->value() : 0.5;
    const double maxOffset = m_spinSphereStudyReacquireMaxMm ? m_spinSphereStudyReacquireMaxMm->value() : 2.0;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    if (attempt <= 0 || !(step > 0.0) || !(maxOffset > 0.0))return { nan,nan,nan };

    // Search sensor-local offsets at fixed C/B.  Y changes plane/sphere offset,
    // while X/Z compensate lateral/depth hand-eye or fitted-center bias.  Every
    // candidate is still solved through the Physical17 model and re-gated by
    // optical feasibility + Machine5X collision safety before any move.
    static const mls::calibration::Vec3d directions[] = { {0,+1,0},{0,-1,0},{+1,0,0},{-1,0,0},{0,0,+1},{0,0,-1} };
    const int zeroBased = attempt - 1;
    const int level = zeroBased / 6 + 1;
    const int directionIndex = zeroBased % 6;
    const double magnitude = level * step;
    if (magnitude > maxOffset + 1e-9)return { nan,nan,nan };
    return { directions[directionIndex].x * magnitude,
            directions[directionIndex].y * magnitude,
            directions[directionIndex].z * magnitude };
}

void MainWindow::finishSphereStudyAutoProbe(bool timeout)
{
    if (!m_sphereStudyProbeActive)return;
    const int index = m_sphereStudyProbePlanIndex;
    ++m_sphereStudyProbeWatchToken;
    m_sphereStudyProbeActive = false;
    if (m_sphereStudyProbeConnection) { QObject::disconnect(m_sphereStudyProbeConnection); m_sphereStudyProbeConnection = {}; }
    m_sphereStudyProbePlanIndex = -1;
    if (!m_sphereStudyAutoActive)return;
    if (index < 0 || index >= static_cast<int>(m_sphereStudyPlan.size())) { handleSphereStudyAutoStepFailure(tr("Probe对应的规划位姿无效")); return; }
    if (m_sphereStudyAutoStopRequested) { finishSphereStudyAuto(true, tr("用户停止自动研究Probe")); return; }
    if (m_sphereStudyAutoPauseRequested) {
        m_sphereStudyAutoPaused = true;
        m_sphereStudyAutoPoseReady = true;
        if (m_lblSphereStudyCapture)m_lblSphereStudyCapture->setText(tr("自动研究：Probe结束后已暂停；继续时会重新进行到位Probe。"));
        updateSphereStudyUiState(); return;
    }

    mls::calibration::SphereStudyProfileAnalysisOptions opts;
    opts.ransac.inlierThresholdMm = m_spinSphereStudyProfileThreshold ? m_spinSphereStudyProfileThreshold->value() : 0.050;
    opts.ransac.minInlierRatio = 0.10; opts.ransac.minInliers = 8; opts.ransac.minArcSpanDeg = 30.0;
    opts.ransac.maxIterations = 300; opts.predictedSearchMarginMm = 4.0;
    opts.adaptiveMinThresholdMm = 0.008; opts.adaptiveMadMultiplier = 4.0;
    const auto predicted = mls::calibration::SingleSphere17Model::machinePointInSensor(
        m_sphereStudyProbeActualPose, m_sphereStudySphereFit.centerMachineMm, m_sphereStudyPlanningModel);
    const int sideSign = predicted.y < 0.0 ? -1 : +1;
    const int minDetected = m_spinSphereStudyProbeMinCircles ? m_spinSphereStudyProbeMinCircles->value() : 3;
    const auto probe = mls::calibration::SphereDistanceAngleStudy::probeProfileGroup(
        m_sphereStudyProbeProfiles, predicted, 0.5 * m_spinSphereStudyDiameter->value(), sideSign, opts, minDetected);
    if (index >= 0 && index < static_cast<int>(m_sphereStudyAutoProbeDetectedCircles.size())) {
        m_sphereStudyAutoProbeDetectedCircles[static_cast<std::size_t>(index)] = probe.detectedCircleCount;
        m_sphereStudyAutoProbeFrames[static_cast<std::size_t>(index)] = probe.profileCount;
    }

    if (probe.success) {
        m_sphereStudyAutoPlanNotes[static_cast<std::size_t>(index)] = m_sphereStudyReacquireAttempt > 0
            ? tr("Probe通过；Sensor修正 Δ[%1,%2,%3] mm")
            .arg(m_sphereStudyAppliedSensorOffsetMm.x, 0, 'f', 2)
            .arg(m_sphereStudyAppliedSensorOffsetMm.y, 0, 'f', 2)
            .arg(m_sphereStudyAppliedSensorOffsetMm.z, 0, 'f', 2)
            : tr("Probe通过；真实球圆弧已确认");
        rebuildSphereStudyPlanTable();
        QString error;
        if (!beginSphereStudyCaptureAtPlanIndex(index, false, &error))handleSphereStudyAutoStepFailure(error);
        return;
    }

    ++m_sphereStudyReacquireAttempt;
    const auto offset = sphereStudyReacquireOffsetForAttempt(m_sphereStudyReacquireAttempt);
    if (!std::isfinite(offset.x) || !std::isfinite(offset.y) || !std::isfinite(offset.z)) {
        const double maxOffset = m_spinSphereStudyReacquireMaxMm ? m_spinSphereStudyReacquireMaxMm->value() : 2.0;
        handleSphereStudyAutoStepFailure(tr("到位Probe未找到球圆弧，Sensor X/Y/Z ±%1 mm自动找球范围已用尽（%2%3）")
            .arg(maxOffset, 0, 'f', 2).arg(timeout ? tr("Probe超时；") : QString()).arg(QString::fromStdString(probe.message)));
        return;
    }
    if (index >= 0 && index < static_cast<int>(m_sphereStudyAutoReacquireAttempts.size()))
        m_sphereStudyAutoReacquireAttempts[static_cast<std::size_t>(index)] = m_sphereStudyReacquireAttempt;
    if (m_lblSphereStudyCapture)m_lblSphereStudyCapture->setText(tr(
        "自动研究：位姿 #%1 Probe未找到稳定球圆弧；尝试Sensor局部偏移 ΔX=%2, ΔY=%3, ΔZ=%4 mm。")
        .arg(index + 1).arg(offset.x, 0, 'f', 2).arg(offset.y, 0, 'f', 2).arg(offset.z, 0, 'f', 2));
    startSphereStudyAutoReacquireMove(index, offset);
}

void MainWindow::startSphereStudyAutoReacquireMove(
    int planIndex, const mls::calibration::Vec3d& sensorOffsetMm)
{
    if (!m_sphereStudyAutoActive || planIndex < 0 || planIndex >= static_cast<int>(m_sphereStudyPlan.size()))return;
    const auto& target = m_sphereStudyPlan[static_cast<std::size_t>(planIndex)];
    JointState5D state; QString error;
    if (!m_zmc || !m_zmc->readJointState5D(state, &error) || !state.allIdle) { handleSphereStudyAutoStepFailure(error.isEmpty() ? tr("自动找球前机床未停止") : error); return; }

    auto offsetText = [](const mls::calibration::Vec3d& v) {
        return QStringLiteral("[%1,%2,%3]").arg(v.x, 0, 'f', 2).arg(v.y, 0, 'f', 2).arg(v.z, 0, 'f', 2);
        };
    auto queueNextOffset = [this, planIndex](const QString& why) {
        ++m_sphereStudyReacquireAttempt;
        const auto next = sphereStudyReacquireOffsetForAttempt(m_sphereStudyReacquireAttempt);
        if (!std::isfinite(next.x) || !std::isfinite(next.y) || !std::isfinite(next.z)) {
            handleSphereStudyAutoStepFailure(tr("Sensor X/Y/Z自动找球范围已用尽：%1").arg(why));
            return;
        }
        if (planIndex >= 0 && planIndex < static_cast<int>(m_sphereStudyAutoReacquireAttempts.size()))
            m_sphereStudyAutoReacquireAttempts[static_cast<std::size_t>(planIndex)] = m_sphereStudyReacquireAttempt;
        QTimer::singleShot(0, this, [this, planIndex, next]() {startSphereStudyAutoReacquireMove(planIndex, next); });
        };

    auto opts = makeSphereStudyOpticalOptions(state.mpos);
    // Reacquisition is allowed to use a smaller section than the nominal path,
    // but never an optically meaningless one.  The subsequent real Probe is
    // still the final acceptance gate before the formal acquisition starts.
    opts.minimumPlaneIntersectionMarginMm = 0.10;
    opts.minimumPredictedVisibleArcDeg = 30.0;
    opts.minimumPredictedVisibleArcLengthMm = std::max(3.0, 0.5 * (m_spinSphereStudyMinSimArcLength ? m_spinSphereStudyMinSimArcLength->value() : 6.0));

    auto targetSphere = target.geometry.targetSphereCenterSensorMm;
    targetSphere.x += sensorOffsetMm.x;
    targetSphere.y += sensorOffsetMm.y;
    targetSphere.z += sensorOffsetMm.z;
    mls::calibration::JointPose5d solved; mls::calibration::Vec3d predicted; double residual = 0.0; std::string solveError;
    if (!mls::calibration::SphereDistanceAngleStudy::solvePoseForSphereCenterTarget(
        m_sphereStudySphereFit.centerMachineMm, m_sphereStudyPlanningModel, opts, targetSphere,
        target.joints.c, target.joints.b, opts.referenceJoint, solved, predicted, residual, solveError)) {
        queueNextOffset(tr("当前Sensor偏置%1的RTCP反解失败：%2").arg(offsetText(sensorOffsetMm)).arg(QString::fromStdString(solveError))); return;
    }
    const auto optical = mls::calibration::SphereDistanceAngleStudy::simulateOpticalFeasibility(
        solved, m_sphereStudySphereFit.centerMachineMm, opts.sphereRadiusMm, m_sphereStudyPlanningModel, opts);
    if (!optical.valid) {
        queueNextOffset(tr("当前Sensor偏置%1未通过相交/梯形FOV/末端侧有效弧门禁：%2")
            .arg(offsetText(sensorOffsetMm)).arg(QString::fromStdString(optical.message))); return;
    }

    JointPose5D q{ solved.x,solved.y,solved.z,solved.c,solved.b };
    QString collisionError;
    if (!checkSphereStudyPoseSafety(q, &collisionError, false) ||
        !checkSphereStudyTransitionSafety(state.mpos, q, &collisionError, false)) {
        queueNextOffset(tr("当前Sensor偏置%1碰撞安全检查失败：%2").arg(offsetText(sensorOffsetMm)).arg(collisionError));
        return;
    }
    const double scale = m_spinSphereStudySpeedScale->value() / 100.0;
    const JointMotionPlan5D plan = JointTrajectoryPlanner5Axis::planPointToPoint(state.mpos, q, currentAxisParams5D(), scale);
    if (!plan.valid) { queueNextOffset(tr("当前Sensor偏置%1运动规划失败：%2").arg(offsetText(sensorOffsetMm)).arg(plan.error)); return; }
    m_sphereStudyAutoPoseReady = false;
    m_sphereStudyAutoPlanNotes[static_cast<std::size_t>(planIndex)] = tr("自动找球移动 Sensor Δ%1").arg(offsetText(sensorOffsetMm));
    rebuildSphereStudyPlanTable();
    setJointMoveUiRunning(true); ++m_axisWorkerCount;
    ZmcController* controller = m_zmc.get();
    (void)QtConcurrent::run([this, controller, plan, planIndex, sensorOffsetMm]() {
        JointMoveResult5D result; QString moveError; const bool ok = controller->moveJointAbs5D(plan, result, &moveError);
        QMetaObject::invokeMethod(this, [this, result, ok, moveError, planIndex, sensorOffsetMm]() {
            --m_axisWorkerCount; setJointMoveUiRunning(false); pollAxisAlarmStates(false);
            if (!m_sphereStudyAutoActive) { tryFinishDeferredClose(); return; }
            if (!ok) { handleSphereStudyAutoStepFailure(moveError.isEmpty() ? tr("自动找球移动失败") : moveError); tryFinishDeferredClose(); return; }
            QString arrivalError;
            if (!sphereStudyArrivalWithinTolerance(result, &arrivalError)) {
                handleSphereStudyAutoStepFailure(tr("自动找球到位门禁失败：%1").arg(arrivalError));
                tryFinishDeferredClose(); return;
            }
            m_sphereStudyAppliedSensorOffsetMm = sensorOffsetMm;
            m_sphereStudyAppliedNormalOffsetMm = sensorOffsetMm.y; // legacy CSV compatibility
            if (planIndex >= 0 && planIndex < static_cast<int>(m_sphereStudyAutoReacquireOffsetsMm.size()))
                m_sphereStudyAutoReacquireOffsetsMm[static_cast<std::size_t>(planIndex)] = sensorOffsetMm.y;
            if (planIndex >= 0 && planIndex < static_cast<int>(m_sphereStudyAutoReacquireSensorOffsetsMm.size()))
                m_sphereStudyAutoReacquireSensorOffsetsMm[static_cast<std::size_t>(planIndex)] = sensorOffsetMm;
            m_sphereStudyAutoPoseReady = true;
            m_sphereStudyAutoPlanNotes[static_cast<std::size_t>(planIndex)] = tr("自动找球到位 Sensor Δ[%1,%2,%3] mm，准备重新Probe")
                .arg(sensorOffsetMm.x, 0, 'f', 2).arg(sensorOffsetMm.y, 0, 'f', 2).arg(sensorOffsetMm.z, 0, 'f', 2);
            rebuildSphereStudyPlanTable();
            QTimer::singleShot(0, this, &MainWindow::advanceSphereStudyAutoSequence);
            tryFinishDeferredClose();
            }, Qt::QueuedConnection);
        });
}

void MainWindow::startSphereStudyAutoMoveCurrent()
{
    if (!m_sphereStudyAutoActive || m_sphereStudyAutoQueuePos >= static_cast<int>(m_sphereStudyAutoQueue.size()))return;
    m_sphereStudyReacquireAttempt = 0;
    m_sphereStudyAppliedNormalOffsetMm = 0.0;
    m_sphereStudyAppliedSensorOffsetMm = {};
    const int index = m_sphereStudyAutoQueue[static_cast<std::size_t>(m_sphereStudyAutoQueuePos)];
    const auto& target = m_sphereStudyPlan[static_cast<std::size_t>(index)];
    if (!m_zmc || !m_zmc->isConnected() || !m_zmc->areLinearMposReferenced()
        || !m_zmc->isRotaryAbsoluteReady(AxisC) || !m_zmc->isRotaryAbsoluteReady(AxisB)) {
        handleSphereStudyAutoStepFailure(tr("自动移动前五轴坐标基准失效")); return;
    }
    if (!m_zmc->isRotaryServoEnabled(AxisC) || !m_zmc->isRotaryServoEnabled(AxisB)) {
        handleSphereStudyAutoStepFailure(tr("C/B伺服已关闭")); return;
    }
    if ((m_syncManager && m_syncManager->isRunning()) || (m_gcodeExecutor && (m_gcodeExecutor->isRunning() || m_gcodeExecutor->isPaused()))) {
        handleSphereStudyAutoStepFailure(tr("存在冲突的同步扫描或G代码任务")); return;
    }
    JointState5D state; QString error;
    if (!m_zmc->readJointState5D(state, &error) || !state.allIdle) { handleSphereStudyAutoStepFailure(error.isEmpty() ? tr("自动移动前至少一个轴未停止") : error); return; }
    JointPose5D q{ target.joints.x,target.joints.y,target.joints.z,target.joints.c,target.joints.b };

    // Re-run the exact optical gate immediately before issuing motion.  This
    // catches a plan generated under stale study UI/model assumptions and is
    // intentionally independent from the later real-profile Probe gate.
    auto opticalOptions = makeSphereStudyOpticalOptions(state.mpos);
    const auto optical = mls::calibration::SphereDistanceAngleStudy::simulateOpticalFeasibility(
        target.joints, m_sphereStudySphereFit.centerMachineMm, opticalOptions.sphereRadiusMm,
        m_sphereStudyPlanningModel, opticalOptions);
    if (!optical.valid) {
        handleSphereStudyAutoStepFailure(tr("移动前扫描仿真门禁拒绝位姿 #%1：%2")
            .arg(index + 1).arg(QString::fromStdString(optical.message)));
        return;
    }
    QString collisionError;
    if (!checkSphereStudyPoseSafety(q, &collisionError, false) ||
        !checkSphereStudyTransitionSafety(state.mpos, q, &collisionError, false)) {
        handleSphereStudyAutoStepFailure(tr("Machine5X碰撞安全门禁拒绝该移动：%1").arg(collisionError));
        return;
    }
    const double scale = m_spinSphereStudySpeedScale->value() / 100.0;
    const JointMotionPlan5D plan = JointTrajectoryPlanner5Axis::planPointToPoint(state.mpos, q, currentAxisParams5D(), scale);
    if (!plan.valid) { handleSphereStudyAutoStepFailure(plan.error); return; }

    m_sphereStudyAutoPlanNotes[static_cast<std::size_t>(index)] = tr("自动移动中");
    rebuildSphereStudyPlanTable();
    setJointMoveUiRunning(true); ++m_axisWorkerCount;
    if (m_lblSphereStudyCapture)m_lblSphereStudyCapture->setText(tr("自动研究 %1/%2：正在移动到位姿 #%3，XYZCB=[%4,%5,%6,%7,%8]。")
        .arg(m_sphereStudyAutoQueuePos + 1).arg(static_cast<int>(m_sphereStudyAutoQueue.size())).arg(index + 1)
        .arg(q.x, 0, 'f', 3).arg(q.y, 0, 'f', 3).arg(q.z, 0, 'f', 3).arg(q.c, 0, 'f', 3).arg(q.b, 0, 'f', 3));
    ZmcController* controller = m_zmc.get();
    (void)QtConcurrent::run([this, controller, plan, index]() {
        JointMoveResult5D result; QString moveError; const bool ok = controller->moveJointAbs5D(plan, result, &moveError);
        QMetaObject::invokeMethod(this, [this, result, ok, moveError, index]() {
            --m_axisWorkerCount; setJointMoveUiRunning(false); pollAxisAlarmStates(false);
            if (!m_sphereStudyAutoActive) { tryFinishDeferredClose(); return; }
            if (m_sphereStudyAutoStopRequested) { finishSphereStudyAuto(true, tr("自动研究移动被停止")); tryFinishDeferredClose(); return; }
            if (!ok) { handleSphereStudyAutoStepFailure(moveError.isEmpty() ? tr("五轴自动移动失败") : moveError); tryFinishDeferredClose(); return; }
            QString arrivalError;
            if (!sphereStudyArrivalWithinTolerance(result, &arrivalError)) {
                handleSphereStudyAutoStepFailure(tr("自动采集到位门禁失败：%1").arg(arrivalError));
                tryFinishDeferredClose(); return;
            }
            m_sphereStudyAutoPoseReady = true;
            m_sphereStudyAutoPlanNotes[static_cast<std::size_t>(index)] = tr("自动到位");
            const auto q = mls::calibration::JointPose5d{ result.finalState.mpos.x,result.finalState.mpos.y,result.finalState.mpos.z,result.finalState.mpos.c,result.finalState.mpos.b };
            const mls::calibration::Vec3d optical{ m_spinSphereStudyOpticalCenterX->value(),0.0,m_spinSphereStudyOpticalCenterZ->value() };
            const auto g = mls::calibration::SphereDistanceAngleStudy::observeGeometry(q, m_sphereStudySphereFit.centerMachineMm, 0.5 * m_spinSphereStudyDiameter->value(), m_sphereStudyPlanningModel, optical);
            if (m_sphereStudyAutoPauseRequested) {
                m_sphereStudyAutoPaused = true;
                m_lblSphereStudyCapture->setText(tr("自动研究：位姿 #%1 已到位并暂停；继续后开始稳定等待和采样。").arg(index + 1));
                rebuildSphereStudyPlanTable(); updateSphereStudyUiState(); tryFinishDeferredClose(); return;
            }
            if (g.valid)m_lblSphereStudyCapture->setText(tr("自动研究：位姿 #%1 到位；模型估计 D≈%2 mm, γ≈%3°, η≈%4°，准备稳定等待。")
                .arg(index + 1).arg(g.measurementDistanceMm, 0, 'f', 3).arg(g.sectionAngleDeg, 0, 'f', 3)
                .arg(g.centerBearingAngleDeg, 0, 'f', 3));
            rebuildSphereStudyPlanTable();
            QTimer::singleShot(0, this, &MainWindow::advanceSphereStudyAutoSequence);
            tryFinishDeferredClose();
            }, Qt::QueuedConnection);
        });
}

void MainWindow::handleSphereStudyAutoStepFailure(const QString& reason)
{
    if (!m_sphereStudyAutoActive)return;
    if (m_sphereStudyAutoStopRequested) { finishSphereStudyAuto(true, tr("用户停止自动研究采集")); return; }
    m_sphereStudyAutoPoseReady = false;
    const int index = (m_sphereStudyAutoQueuePos >= 0 && m_sphereStudyAutoQueuePos < static_cast<int>(m_sphereStudyAutoQueue.size()))
        ? m_sphereStudyAutoQueue[static_cast<std::size_t>(m_sphereStudyAutoQueuePos)] : -1;
    const int maxRetries = m_spinSphereStudyAutoRetries ? m_spinSphereStudyAutoRetries->value() : 0;
    if (m_sphereStudyAutoCurrentRetry < maxRetries) {
        ++m_sphereStudyAutoCurrentRetry;
        if (index >= 0 && index < static_cast<int>(m_sphereStudyAutoPlanNotes.size()))m_sphereStudyAutoPlanNotes[static_cast<std::size_t>(index)] = tr("失败，自动重试 %1/%2").arg(m_sphereStudyAutoCurrentRetry).arg(maxRetries);
        if (m_lblSphereStudyCapture)m_lblSphereStudyCapture->setText(tr("自动研究：位姿 #%1 失败：%2；500 ms 后重试 %3/%4。")
            .arg(index + 1).arg(reason).arg(m_sphereStudyAutoCurrentRetry).arg(maxRetries));
        rebuildSphereStudyPlanTable();
        QTimer::singleShot(500, this, [this]() {if (m_sphereStudyAutoActive)advanceSphereStudyAutoSequence(); });
        return;
    }

    ++m_sphereStudyAutoFailed;
    m_sphereStudyAutoCurrentRetry = 0;
    if (index >= 0 && index < static_cast<int>(m_sphereStudyAutoPlanNotes.size()))m_sphereStudyAutoPlanNotes[static_cast<std::size_t>(index)] = tr("自动失败并跳过：%1").arg(reason);
    const bool continueOnFailure = m_checkSphereStudyAutoContinueOnFailure && m_checkSphereStudyAutoContinueOnFailure->isChecked();
    if (!continueOnFailure) { finishSphereStudyAuto(true, tr("位姿 #%1 自动采集失败：%2").arg(index + 1).arg(reason)); return; }
    ++m_sphereStudyAutoQueuePos;
    rebuildSphereStudyPlanTable();
    if (m_lblSphereStudyCapture)m_lblSphereStudyCapture->setText(tr("自动研究：位姿 #%1 已跳过（%2），继续下一位姿。")
        .arg(index + 1).arg(reason));
    QTimer::singleShot(300, this, &MainWindow::advanceSphereStudyAutoSequence);
}

void MainWindow::finishSphereStudyAuto(bool cancelled, const QString& reason)
{
    if (!m_sphereStudyAutoActive && !m_sphereStudyAutoOwnsPreview)return;
    m_sphereStudyAutoActive = false;
    m_sphereStudyAutoPauseRequested = false;
    m_sphereStudyAutoPaused = false;
    m_sphereStudyAutoStopRequested = false;
    m_sphereStudyAutoPoseReady = false;
    ++m_sphereStudyAutoSequenceToken;
    ++m_sphereStudyCaptureWatchToken;
    ++m_sphereStudyProbeWatchToken;
    m_sphereStudyProbeActive = false;
    if (m_sphereStudyProbeConnection) { QObject::disconnect(m_sphereStudyProbeConnection); m_sphereStudyProbeConnection = {}; }
    m_sphereStudyProbeProfiles.clear(); m_sphereStudyProbePlanIndex = -1;
    if (m_laser)m_laser->setSensorCalibrationRichProfileEnabled(false);
    if (m_sphereStudyAutoOwnsPreview && m_profilePreviewRunning)onStopProfilePreview();
    m_sphereStudyAutoOwnsPreview = false;
    const int total = static_cast<int>(m_sphereStudyAutoQueue.size());
    m_sphereStudyAutoQueue.clear(); m_sphereStudyAutoQueuePos = 0; m_sphereStudyAutoCurrentRetry = 0;
    if (!m_jointMoveInProgress)setJointMoveUiRunning(false); // restore manual controls only after motion has really ended
    rebuildSphereStudyPlanTable(); updateVideoControlState(); updateSphereStudyUiState();
    if (m_lblSphereStudyCapture)m_lblSphereStudyCapture->setText(cancelled
        ? tr("自动研究已停止：%1。已完成=%2，失败/跳过=%3。已保存的数据保留在当前session中。")
        .arg(reason).arg(m_sphereStudyAutoCompleted).arg(m_sphereStudyAutoFailed)
        : tr("自动研究完成：%1；任务=%2，完成=%3，失败/跳过=%4。下一步可执行RANSAC/ROI分析全部数据。")
        .arg(reason).arg(total).arg(m_sphereStudyAutoCompleted).arg(m_sphereStudyAutoFailed));
}

void MainWindow::finishSphereStudyCapture(bool cancelled, const QString& reason)
{
    const bool autoActive = m_sphereStudyAutoActive;
    const int finishedPlanIndex = m_sphereStudyCapturePlanIndex;
    ++m_sphereStudyCaptureWatchToken;
    if (m_sphereStudyProfileConnection) { QObject::disconnect(m_sphereStudyProfileConnection); m_sphereStudyProfileConnection = {}; }

    // During a successful automatic run keep the Profile stream and rich
    // channels alive between poses.  This avoids expensive sensor restart
    // cycles while still ensuring that no frames are recorded during motion.
    const bool keepStreamForAuto = autoActive && !cancelled && !m_sphereStudyAutoStopRequested;
    const bool stopOwnPreview = m_sphereStudyStartedPreview;
    if (keepStreamForAuto && stopOwnPreview)m_sphereStudyAutoOwnsPreview = true;
    if (!keepStreamForAuto && m_laser)m_laser->setSensorCalibrationRichProfileEnabled(false);

    m_sphereStudyCaptureActive = false;
    m_sphereStudyCaptureDiscardRemaining = -1;
    m_sphereStudyStartedPreview = false;
    m_sphereStudyCurrentVisitId = -1;
    m_sphereStudyCapturePlanIndex = -1;
    if (!keepStreamForAuto && stopOwnPreview && m_profilePreviewRunning)onStopProfilePreview();
    if (!keepStreamForAuto && autoActive && m_sphereStudyAutoOwnsPreview && m_profilePreviewRunning) {
        onStopProfilePreview(); m_sphereStudyAutoOwnsPreview = false;
    }
    rebuildSphereStudyPlanTable(); updateVideoControlState(); updateSphereStudyUiState();

    if (!autoActive) {
        if (m_lblSphereStudyCapture)m_lblSphereStudyCapture->setText(cancelled ? tr("研究状态：采集中止：%1").arg(reason)
            : tr("研究状态：%1。原始Rich Profile已逐槽保存；可继续采集或执行RANSAC/ROI分析。")
            .arg(reason.isEmpty() ? tr("采集完成") : reason));
        return;
    }

    if (cancelled) {
        if (m_sphereStudyAutoStopRequested)finishSphereStudyAuto(true, reason.isEmpty() ? tr("用户停止自动研究采集") : reason);
        else handleSphereStudyAutoStepFailure(reason.isEmpty() ? tr("自动Rich Profile采集失败") : reason);
        return;
    }

    m_sphereStudyAutoPoseReady = false;
    m_sphereStudyAutoCurrentRetry = 0;
    ++m_sphereStudyAutoCompleted;
    if (finishedPlanIndex >= 0 && finishedPlanIndex < static_cast<int>(m_sphereStudyAutoPlanNotes.size()))
        m_sphereStudyAutoPlanNotes[static_cast<std::size_t>(finishedPlanIndex)] = tr("自动采集完成");
    ++m_sphereStudyAutoQueuePos;
    rebuildSphereStudyPlanTable();
    if (m_sphereStudyAutoPauseRequested) {
        m_sphereStudyAutoPaused = true;
        if (m_lblSphereStudyCapture)m_lblSphereStudyCapture->setText(tr("自动研究：位姿 #%1 已完整采集，现已暂停。继续后进入下一位姿。").arg(finishedPlanIndex + 1));
        updateSphereStudyUiState();
        return;
    }
    if (m_lblSphereStudyCapture)m_lblSphereStudyCapture->setText(tr("自动研究：位姿 #%1 已完成，准备下一位姿...").arg(finishedPlanIndex + 1));
    QTimer::singleShot(150, this, &MainWindow::advanceSphereStudyAutoSequence);
}

void MainWindow::onSphereStudyAnalyze()
{
    if (m_sphereStudyCaptureActive || m_sphereStudyRecords.empty() || !m_sphereStudySphereFit.success || !m_sphereStudyHasPlanningModel)return;
    struct Payload { std::vector<SphereStudyCaptureRecord> records; std::vector<SphereStudyPoseSummary> summaries; QString error; };
    const auto records = m_sphereStudyRecords; const auto plan = m_sphereStudyPlan; const auto center = m_sphereStudySphereFit.centerMachineMm;
    const double radius = 0.5 * m_spinSphereStudyDiameter->value(); const auto model = m_sphereStudyPlanningModel;
    const mls::calibration::Vec3d optical{ m_spinSphereStudyOpticalCenterX->value(),0.0,m_spinSphereStudyOpticalCenterZ->value() };
    mls::calibration::SphereStudyProfileAnalysisOptions opts; opts.ransac.inlierThresholdMm = m_spinSphereStudyProfileThreshold->value();
    opts.ransac.minInlierRatio = 0.12; opts.ransac.minInliers = 8; opts.adaptiveMinThresholdMm = 0.008; opts.adaptiveMadMultiplier = 4.0; opts.minimumConsensusProfiles = 3;
    m_btnSphereStudyAnalyze->setEnabled(false); m_lblSphereStudyCapture->setText(tr("研究状态：多帧Circle RANSAC、EvaluationROI和D/γ/η/θ质量分析中..."));
    auto* watcher = new QFutureWatcher<Payload>(this);
    connect(watcher, &QFutureWatcher<Payload>::finished, this, [this, watcher]() {
        auto payload = watcher->result(); watcher->deleteLater();
        if (!payload.error.isEmpty()) { QMessageBox::warning(this, tr("分析失败"), payload.error); updateSphereStudyUiState(); return; }
        m_sphereStudyRecords = std::move(payload.records); m_sphereStudySummaries = std::move(payload.summaries); rebuildSphereStudyPlanTable();
        double best = std::numeric_limits<double>::infinity(); int bestIndex = -1;
        for (int i = 0; i < static_cast<int>(m_sphereStudySummaries.size()); ++i) {
            const auto& s = m_sphereStudySummaries[static_cast<std::size_t>(i)];
            if (s.profiles < 3 || s.meanGoodPointRatio < 0.80 || s.meanDropoutRatio>0.10)continue;
            const double score = s.meanAbsoluteP95Mm + std::abs(s.meanRadiusBiasMm) + 0.02 * s.meanDropoutRatio + 0.02 * s.meanOutlierRatio;
            if (score < best) { best = score; bestIndex = i; }
        }
        if (bestIndex >= 0) {
            const auto& p = m_sphereStudyPlan[static_cast<std::size_t>(bestIndex)]; const auto& s = m_sphereStudySummaries[static_cast<std::size_t>(bestIndex)];
            m_lblSphereStudyCapture->setText(tr("分析完成。当前低误差候选区域样本 #%1：D实测=%2 mm, γ=%3°, η=%4°, C=%5°, B=%6°；P95=%7 mm，半径偏差=%8 mm，Good=%9%，Drop=%10%。这只是候选样本，不是最终全局最优点。")
                .arg(bestIndex + 1).arg(s.meanActualDistanceMm, 0, 'f', 3).arg(s.meanSectionAngleDeg, 0, 'f', 2).arg(s.meanCenterBearingAngleDeg, 0, 'f', 2)
                .arg(p.joints.c, 0, 'f', 1).arg(p.joints.b, 0, 'f', 1)
                .arg(s.meanAbsoluteP95Mm, 0, 'f', 4).arg(s.meanRadiusBiasMm, 0, 'f', 4).arg(s.meanGoodPointRatio * 100.0, 0, 'f', 1).arg(s.meanDropoutRatio * 100.0, 0, 'f', 1));
        }
        else m_lblSphereStudyCapture->setText(tr("分析完成，但当前数据尚不足以形成Good>=80%且Drop<=10%的稳定候选区域。"));
        updateSphereStudyUiState();
        });
    watcher->setFuture(QtConcurrent::run([records, plan, center, radius, model, optical, opts]() mutable -> Payload {
        Payload out; out.records = records; out.summaries.resize(plan.size());
        struct Acc { int groups = 0, profiles = 0, circle = 0, centers = 0; std::size_t eval = 0; double good = 0, drop = 0, outlier = 0, circleRms = 0, p95 = 0, bias = 0, d = 0, g = 0, e = 0, cx = 0, cx2 = 0, cz = 0, cz2 = 0; };
        std::vector<Acc> acc(plan.size());
        for (auto& rec : out.records) {
            if (rec.planIndex < 0 || rec.planIndex >= static_cast<int>(plan.size()))continue; auto& a = acc[static_cast<std::size_t>(rec.planIndex)]; ++a.groups;
            rec.analysis = mls::calibration::SphereDistanceAngleStudy::analyzeProfileGroup(
                { rec.actualPose.x,rec.actualPose.y,rec.actualPose.z,rec.actualPose.c,rec.actualPose.b }, rec.profiles, center, radius, model, optical, opts);
            rec.metrics = rec.analysis.profiles;
            for (const auto& m : rec.metrics) {
                ++a.profiles; if (!m.valid)continue; a.eval += m.evaluationPointCount; a.good += m.goodPointRatio; a.drop += m.dropoutRatio; a.outlier += m.geometricOutlierRatio;
                a.p95 += m.absoluteSectionP95Mm; a.bias += m.radiusBiasMm; a.d += m.measurementDistanceMm; a.g += m.sectionAngleDeg; a.e += m.centerBearingAngleDeg;
                if (m.circleDetected) { ++a.circle; a.circleRms += m.circleRmsSensorMm; }
                a.cx += m.fittedCircleCenterSensorMm.x; a.cx2 += m.fittedCircleCenterSensorMm.x * m.fittedCircleCenterSensorMm.x;
                a.cz += m.fittedCircleCenterSensorMm.z; a.cz2 += m.fittedCircleCenterSensorMm.z * m.fittedCircleCenterSensorMm.z; ++a.centers;
            }
        }
        for (std::size_t i = 0; i < acc.size(); ++i) {
            const auto& a = acc[i]; auto& s = out.summaries[i]; s.groups = a.groups; s.profiles = a.profiles; s.circleDetectedProfiles = a.circle; s.evaluationPoints = a.eval;
            if (a.profiles > 0) { const double n = static_cast<double>(a.profiles); s.meanGoodPointRatio = a.good / n; s.meanDropoutRatio = a.drop / n; s.meanOutlierRatio = a.outlier / n; s.meanAbsoluteP95Mm = a.p95 / n; s.meanRadiusBiasMm = a.bias / n; s.meanActualDistanceMm = a.d / n; s.meanSectionAngleDeg = a.g / n; s.meanCenterBearingAngleDeg = a.e / n; }
            if (a.circle > 0)s.meanCircleRmsMm = a.circleRms / static_cast<double>(a.circle);
            if (a.centers > 0) { const double n = a.centers, mx = a.cx / n, mz = a.cz / n; s.centerRepeatabilityXmm = std::sqrt(std::max(0.0, a.cx2 / n - mx * mx)); s.centerRepeatabilityZmm = std::sqrt(std::max(0.0, a.cz2 / n - mz * mz)); }
        }
        return out;
        }));
}

void MainWindow::onSphereStudyExport()
{
    if (!m_sphereStudySphereFit.success && m_sphereStudyRecords.empty())return;
    if (m_sphereStudySessionDir.isEmpty()) { const QString base = m_editSphereStudyDir ? m_editSphereStudyDir->text() : QDir::currentPath(); m_sphereStudySessionDir = QDir(base).filePath(QStringLiteral("session_%1").arg(QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss"))); }
    if (!QDir().mkpath(m_sphereStudySessionDir)) { QMessageBox::warning(this, tr("导出失败"), tr("无法创建目录：%1").arg(m_sphereStudySessionDir)); return; }

    QJsonObject root; root["schema"] = "SphereDistanceAngleStudy.v11"; root["applicationVersion"] = "V27.6.16AF1"; root["createdAt"] = QDateTime::currentDateTime().toString(Qt::ISODateWithMs);
    QJsonObject opticalJson; opticalJson["x_mm"] = m_spinSphereStudyOpticalCenterX->value(); opticalJson["y_mm"] = 0.0; opticalJson["z_mm"] = m_spinSphereStudyOpticalCenterZ->value();
    root["sensorOpticalReferenceCenter"] = opticalJson;
    root["measurementDistanceDefinition"] = QStringLiteral("Euclidean distance from refined section-circle center to configured sensor optical reference center");
    root["centerBearingDefinition"] = QStringLiteral("eta=atan2(Xc-X0,Zc-Z0), measured from sensor +Z toward +X in the profile plane");
    root["geometryDefinition"] = QStringLiteral("D=Euclidean distance from sensor optical reference center to RANSAC/refined section-circle center; eta=atan2(Xc-X0,Zc-Z0); gamma=asin(abs(distance(known machine sphere center, transformed laser profile plane))/R); theta=local expected sphere normal versus optical viewing ray. RANSAC identifies circle/ROI only; every raw slot inside EvaluationROI remains in quality statistics.");
    QJsonObject simulationOptics;
    simulationOptics["measuringFieldShape"] = QStringLiteral("scanCONTROL 2950-50/BL standard trapezoid");
    simulationOptics["sensorZCoordinateFlipped"] = m_sphereStudyPlanningFlipZ;
    simulationOptics["sensorCoordinateConventionSource"] = m_sphereStudyPlanningConventionSource;
    simulationOptics["reportedZMin_mm"] = Constants::kSensorZMin;
    simulationOptics["reportedZMax_mm"] = Constants::kSensorZMax;
    simulationOptics["physicalNearEdgeReportedZ_mm"] = m_sphereStudyPlanningFlipZ ? Constants::kSensorZMax : Constants::kSensorZMin;
    simulationOptics["physicalFarEdgeReportedZ_mm"] = m_sphereStudyPlanningFlipZ ? Constants::kSensorZMin : Constants::kSensorZMax;
    simulationOptics["xSpanNearSensor_mm"] = Constants::kSensorXSpanNear;
    simulationOptics["xSpanMidDepth_mm"] = Constants::kSensorXSpanMid;
    simulationOptics["xSpanFarFromSensor_mm"] = Constants::kSensorXSpanFar;
    simulationOptics["xSafetyMargin_mm"] = 1.0;
    simulationOptics["zSafetyMargin_mm"] = 1.0;
    simulationOptics["trapezoidOrientationRule"] = QStringLiteral("small edge is physically nearest the sensor; large edge is physically farthest; map near/far to reported Sensor-Z through saved FlipZ convention before clipping or rendering");
    simulationOptics["visibleArcSideRule"] = QStringLiteral("keep only sphere-section samples inside the trapezoid with (P-Csphere) dot (Eend-Csphere) > 0, where Eend comes from calibrated ^E T_S");
    root["scanSimulationOptics"] = simulationOptics;
    QJsonObject automation;
    automation["settleMs"] = m_spinSphereStudySettleMs ? m_spinSphereStudySettleMs->value() : 0;
    automation["poseTimeoutMs"] = m_spinSphereStudyAutoTimeoutMs ? m_spinSphereStudyAutoTimeoutMs->value() : 0;
    automation["profilesPerPose"] = m_spinSphereStudyProfilesPerGroup ? m_spinSphereStudyProfilesPerGroup->value() : 0;
    automation["retryCount"] = m_spinSphereStudyAutoRetries ? m_spinSphereStudyAutoRetries->value() : 0;
    automation["skipCompleted"] = m_checkSphereStudyAutoSkipCompleted && m_checkSphereStudyAutoSkipCompleted->isChecked();
    automation["continueOnFailure"] = m_checkSphereStudyAutoContinueOnFailure && m_checkSphereStudyAutoContinueOnFailure->isChecked();
    automation["minimumPredictedVisibleArcLength_mm"] = m_spinSphereStudyMinSimArcLength ? m_spinSphereStudyMinSimArcLength->value() : 6.0;
    automation["reacquisitionSearch"] = QStringLiteral("fixed-CB Sensor-local +/-Y,+/-X,+/-Z layered search with optical + Machine5X safety gates");
    automation["simulationInterval_ms"] = m_spinSphereStudySimulationIntervalMs ? m_spinSphereStudySimulationIntervalMs->value() : 800;
    root["automaticAcquisition"] = automation;
    QJsonObject planeFamily;
    planeFamily["enabled"] = true;
    planeFamily["candidatePlaneDirections"] = m_spinSphereStudyPlaneDirections ? m_spinSphereStudyPlaneDirections->value() : 0;
    planeFamily["rotaryCandidateStep_deg"] = m_spinSphereStudyRotarySearchStep ? m_spinSphereStudyRotarySearchStep->value() : 0.0;
    planeFamily["targetPoseCount"] = m_spinSphereStudyTargetPoseCount ? m_spinSphereStudyTargetPoseCount->value() : 0;
    planeFamily["selection"] = QStringLiteral("reachable C/B plane candidates + maximin space filling over normalized D/gamma/eta/unoriented-plane-normal features; nearest-neighbor joint execution ordering");
    root["planeFamilyPlanning"] = planeFamily;
    root["sourceCloud"] = m_editSphereStudyCloud ? m_editSphereStudyCloud->text() : QString();
    QJsonObject sphere; sphere["centerX_mm"] = m_sphereStudySphereFit.centerMachineMm.x; sphere["centerY_mm"] = m_sphereStudySphereFit.centerMachineMm.y; sphere["centerZ_mm"] = m_sphereStudySphereFit.centerMachineMm.z; sphere["fittedRadius_mm"] = m_sphereStudySphereFit.radiusMm; sphere["knownDiameter_mm"] = m_spinSphereStudyDiameter->value(); sphere["inliers"] = static_cast<qint64>(m_sphereStudySphereFit.inlierPointCount); sphere["rms_mm"] = m_sphereStudySphereFit.residualRmsMm; sphere["twoStageComparison"] = m_sphereStudyFitComparison; root["sphereRansac"] = sphere;
    QJsonObject model; model["source"] = m_sphereStudyPlanningModelSource; model["independentlyValidated"] = m_sphereStudyPlanningModelValidated; QJsonArray pa; for (double v : m_sphereStudyPlanningModel.value)pa.append(v); model["physical17"] = pa; root["planningModel"] = model;
    QJsonArray plans; for (std::size_t i = 0; i < m_sphereStudyPlan.size(); ++i) {
        const auto& p = m_sphereStudyPlan[i]; QJsonObject o; o["index"] = static_cast<int>(i) + 1; o["valid"] = p.valid; o["planeFamilyId"] = p.rotarySampleId + 1; o["targetDistance_mm"] = p.geometry.measurementDistanceMm; o["targetSectionAngle_deg"] = p.geometry.sectionAngleDeg; o["targetCenterBearing_deg"] = p.geometry.centerBearingAngleDeg; o["x_mm"] = p.joints.x; o["y_mm"] = p.joints.y; o["z_mm"] = p.joints.z; o["c_deg"] = p.joints.c; o["b_deg"] = p.joints.b; o["solveResidual_mm"] = p.solveResidualMm; o["predictedVisibleArc_deg"] = p.predictedVisibleArcDeg; o["predictedVisibleArcLength_mm"] = p.predictedVisibleArcLengthMm; o["predictedPlaneIntersectionMargin_mm"] = p.predictedPlaneIntersectionMarginMm; o["adaptiveReplacement"] = p.adaptivelyRegenerated;
        if (i < m_sphereStudySummaries.size()) { const auto& s = m_sphereStudySummaries[i]; o["groups"] = s.groups; o["profiles"] = s.profiles; o["goodRatioMean"] = s.meanGoodPointRatio; o["dropoutRatioMean"] = s.meanDropoutRatio; o["outlierRatioMean"] = s.meanOutlierRatio; o["actualDistanceMean_mm"] = s.meanActualDistanceMm; o["sectionAngleMean_deg"] = s.meanSectionAngleDeg; o["centerBearingMean_deg"] = s.meanCenterBearingAngleDeg; o["circleRmsMean_mm"] = s.meanCircleRmsMm; o["absoluteP95Mean_mm"] = s.meanAbsoluteP95Mm; o["radiusBiasMean_mm"] = s.meanRadiusBiasMm; }plans.append(o);
    }root["planAndSummary"] = plans;
    QJsonArray captures; for (const auto& r : m_sphereStudyRecords) { QJsonObject o; o["visitId"] = r.visitId; o["planIndex"] = r.planIndex + 1; o["groupIndex"] = r.groupIndex + 1; o["complete"] = r.complete; o["profiles"] = static_cast<int>(r.profiles.size()); o["x_MPOS_mm"] = r.actualPose.x; o["y_MPOS_mm"] = r.actualPose.y; o["z_MPOS_mm"] = r.actualPose.z; o["c_MPOS_deg"] = r.actualPose.c; o["b_MPOS_deg"] = r.actualPose.b; o["reacquireAttempts"] = r.reacquireAttemptCount; o["reacquireSensorDx_mm"] = r.reacquireSensorOffsetMm.x; o["reacquireSensorDy_mm"] = r.reacquireSensorOffsetMm.y; o["reacquireSensorDz_mm"] = r.reacquireSensorOffsetMm.z; o["analysisValid"] = r.analysis.valid; o["evaluationStartIndex"] = r.analysis.evaluationStartIndex; o["evaluationEndIndex"] = r.analysis.evaluationEndIndex; captures.append(o); }root["captures"] = captures;
    QJsonObject offlineSimulation;
    offlineSimulation["planGeneratedWithoutLiveReference"] = m_sphereStudyLastPlanWasOffline;
    offlineSimulation["acquiredPoseCount"] = m_sphereStudySimulationAcquiredPoses;
    offlineSimulation["rejectedPoseCount"] = m_sphereStudySimulationRejectedPoses;
    offlineSimulation["syntheticPointCount"] = static_cast<qint64>(m_sphereStudySimulationAcquiredCloud.size());
    offlineSimulation["pointCloudFile"] = m_sphereStudySimulationAcquiredCloud.isEmpty() ? QString() : QStringLiteral("offline_simulated_acquisition_xyz.csv");
    offlineSimulation["definition"] = QStringLiteral("Ideal sphere/laser-plane section samples clipped by the scanCONTROL 2950-50/BL trapezoidal X-Z measuring field and by the hemisphere facing the calibrated five-axis end-effector, then transformed into machine coordinates; no controller or sensor data required.");
    root["offlineScanSimulation"] = offlineSimulation;
    QFile manifest(QDir(m_sphereStudySessionDir).filePath("sphere_distance_angle_study.json")); if (!manifest.open(QIODevice::WriteOnly)) { QMessageBox::warning(this, tr("导出失败"), manifest.errorString()); return; }manifest.write(QJsonDocument(root).toJson(QJsonDocument::Indented)); manifest.close();

    if (!m_sphereStudySimulationAcquiredCloud.isEmpty()) {
        QFile simulated(QDir(m_sphereStudySessionDir).filePath("offline_simulated_acquisition_xyz.csv"));
        if (simulated.open(QIODevice::WriteOnly | QIODevice::Text)) {
            QTextStream out(&simulated);
            out << "X_machine_mm,Y_machine_mm,Z_machine_mm\n";
            for (const auto& p : m_sphereStudySimulationAcquiredCloud)
                out << QString::number(p.x(), 'f', 6) << "," << QString::number(p.y(), 'f', 6) << "," << QString::number(p.z(), 'f', 6) << "\n";
        }
    }

    QFile summary(QDir(m_sphereStudySessionDir).filePath("distance_section_bearing_summary.csv")); if (summary.open(QIODevice::WriteOnly | QIODevice::Text)) { QTextStream out(&summary); out << "index,plane_family_id,target_D_mm,target_gamma_deg,target_eta_deg,X_mm,Y_mm,Z_mm,C_deg,B_deg,predicted_visible_arc_deg,predicted_visible_arc_length_mm,predicted_plane_margin_mm,adaptive_replacement,groups,profiles,circle_detected_profiles,actual_D_mean_mm,gamma_mean_deg,eta_mean_deg,good_ratio_mean,dropout_ratio_mean,outlier_ratio_mean,circle_rms_mean_mm,absolute_p95_mean_mm,radius_bias_mean_mm,centerX_repeat_std_mm,centerZ_repeat_std_mm\n"; for (std::size_t i = 0; i < m_sphereStudyPlan.size(); ++i) { const auto& p = m_sphereStudyPlan[i]; SphereStudyPoseSummary s; if (i < m_sphereStudySummaries.size())s = m_sphereStudySummaries[i]; out << i + 1 << "," << p.rotarySampleId + 1 << "," << p.geometry.measurementDistanceMm << "," << p.geometry.sectionAngleDeg << "," << p.geometry.centerBearingAngleDeg << "," << p.joints.x << "," << p.joints.y << "," << p.joints.z << "," << p.joints.c << "," << p.joints.b << "," << p.predictedVisibleArcDeg << "," << p.predictedVisibleArcLengthMm << "," << p.predictedPlaneIntersectionMarginMm << "," << (p.adaptivelyRegenerated ? 1 : 0) << "," << s.groups << "," << s.profiles << "," << s.circleDetectedProfiles << "," << s.meanActualDistanceMm << "," << s.meanSectionAngleDeg << "," << s.meanCenterBearingAngleDeg << "," << s.meanGoodPointRatio << "," << s.meanDropoutRatio << "," << s.meanOutlierRatio << "," << s.meanCircleRmsMm << "," << s.meanAbsoluteP95Mm << "," << s.meanRadiusBiasMm << "," << s.centerRepeatabilityXmm << "," << s.centerRepeatabilityZmm << "\n"; } }

    QFile visits(QDir(m_sphereStudySessionDir).filePath("visits.csv")); if (visits.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QTextStream out(&visits); out << "visit_id,plan_index,group_index,group_complete,target_D_mm,target_gamma_deg,target_eta_deg,target_C_deg,target_B_deg,predicted_visible_arc_deg,predicted_visible_arc_length_mm,predicted_plane_margin_mm,probe_frames,probe_circle_frames,reacquired,reacquire_attempts,reacquire_normal_offset_mm,reacquire_sensor_dx_mm,reacquire_sensor_dy_mm,reacquire_sensor_dz_mm,actual_X_MPOS_mm,actual_Y_MPOS_mm,actual_Z_MPOS_mm,actual_C_MPOS_deg,actual_B_MPOS_deg,model_D_estimate_mm,model_gamma_estimate_deg,model_eta_estimate_deg,analysis_valid,roi_start,roi_end,profile_count\n";
        for (const auto& r : m_sphereStudyRecords) { const mls::calibration::SphereStudyPose* pp = (r.planIndex >= 0 && r.planIndex < static_cast<int>(m_sphereStudyPlan.size())) ? &m_sphereStudyPlan[static_cast<std::size_t>(r.planIndex)] : nullptr; out << r.visitId << "," << r.planIndex + 1 << "," << r.groupIndex + 1 << "," << (r.complete ? 1 : 0) << "," << (pp ? pp->geometry.measurementDistanceMm : 0.0) << "," << (pp ? pp->geometry.sectionAngleDeg : 0.0) << "," << (pp ? pp->geometry.centerBearingAngleDeg : 0.0) << "," << (pp ? pp->joints.c : 0.0) << "," << (pp ? pp->joints.b : 0.0) << "," << (pp ? pp->predictedVisibleArcDeg : 0.0) << "," << (pp ? pp->predictedVisibleArcLengthMm : 0.0) << "," << (pp ? pp->predictedPlaneIntersectionMarginMm : 0.0) << "," << r.probeFrameCount << "," << r.probeDetectedCircleCount << "," << (r.reacquired ? 1 : 0) << "," << r.reacquireAttemptCount << "," << r.reacquireNormalOffsetMm << "," << r.reacquireSensorOffsetMm.x << "," << r.reacquireSensorOffsetMm.y << "," << r.reacquireSensorOffsetMm.z << "," << r.actualPose.x << "," << r.actualPose.y << "," << r.actualPose.z << "," << r.actualPose.c << "," << r.actualPose.b << ","; if (r.poseGeometryEstimate.valid)out << r.poseGeometryEstimate.measurementDistanceMm; out << ","; if (r.poseGeometryEstimate.valid)out << r.poseGeometryEstimate.sectionAngleDeg; out << ","; if (r.poseGeometryEstimate.valid)out << r.poseGeometryEstimate.centerBearingAngleDeg; out << "," << (r.analysis.valid ? 1 : 0) << "," << r.analysis.evaluationStartIndex << "," << r.analysis.evaluationEndIndex << "," << r.profiles.size() << "\n"; }
    }

    QFile metrics(QDir(m_sphereStudySessionDir).filePath("profile_metrics.csv")); if (metrics.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QTextStream out(&metrics); out << "visit_id,plan_index,group_index,profile_in_group,callback_ordinal,profile_index,valid,circle_detected,roi_start,roi_end,roi_points,coordinate_valid,good_points,outliers,dropouts,good_ratio,outlier_ratio,dropout_ratio,max_dropout_run,dropout_run_count,max_outlier_run,D_mm,gamma_deg,eta_deg,h_mm,r_fit_mm,r_ref_mm,radius_bias_mm,adaptive_threshold_mm,circle_rms_mm,circle_mad_mm,shape_p95_mm,absolute_mean_abs_mm,absolute_rms_mm,absolute_p95_mm,absolute_max_mm,maximum_all_median,maximum_good_median,width_all_median,width_good_median,m0_all_median,m0_good_median,m1_all_median,m1_good_median\n";
        for (const auto& r : m_sphereStudyRecords)for (int i = 0; i < static_cast<int>(r.metrics.size()); ++i) { const auto& m = r.metrics[static_cast<std::size_t>(i)]; auto sv = [](const mls::calibration::RobustScalarStats& s) {return s.valid ? s.median : std::numeric_limits<double>::quiet_NaN(); }; out << r.visitId << "," << r.planIndex + 1 << "," << r.groupIndex + 1 << "," << i + 1 << "," << m.callbackOrdinal << "," << m.profileIndex << "," << (m.valid ? 1 : 0) << "," << (m.circleDetected ? 1 : 0) << "," << m.evaluationStartIndex << "," << m.evaluationEndIndex << "," << m.evaluationPointCount << "," << m.coordinateValidCount << "," << m.goodPointCount << "," << m.geometricOutlierCount << "," << m.dropoutCount << "," << m.goodPointRatio << "," << m.geometricOutlierRatio << "," << m.dropoutRatio << "," << m.maxDropoutRun << "," << m.dropoutRunCount << "," << m.maxOutlierRun << "," << m.measurementDistanceMm << "," << m.sectionAngleDeg << "," << m.centerBearingAngleDeg << "," << m.sectionPlaneOffsetMm << "," << m.fittedRadiusMm << "," << m.theoreticalRadiusMm << "," << m.radiusBiasMm << "," << m.adaptiveInlierThresholdMm << "," << m.circleRmsSensorMm << "," << m.circleMadSensorMm << "," << m.shapeP95Mm << "," << m.absoluteSectionMeanAbsMm << "," << m.absoluteSectionRmsMm << "," << m.absoluteSectionP95Mm << "," << m.absoluteSectionMaxAbsMm << "," << sv(m.maximumAllValid) << "," << sv(m.maximumGood) << "," << sv(m.widthAllValid) << "," << sv(m.widthGood) << "," << sv(m.moment0AllValid) << "," << sv(m.moment0Good) << "," << sv(m.moment1AllValid) << "," << sv(m.moment1Good) << "\n"; }
    }

    QFile points(QDir(m_sphereStudySessionDir).filePath("point_samples.csv")); if (points.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QTextStream out(&points); out << "visit_id,plan_index,group_index,profile_in_group,point_index,D_mm,gamma_deg,eta_deg,local_theta_deg,coordinate_valid,good,outlier,dropout,x_measured_mm,z_measured_mm,x_expected_mm,z_expected_mm,shape_residual_mm,absolute_section_residual_mm,width,maximum,threshold,moment0,moment1\n";
        auto val = [&out](bool has, double v) {if (has && std::isfinite(v))out << v; };
        for (const auto& r : m_sphereStudyRecords)for (int pi = 0; pi < static_cast<int>(r.metrics.size()); ++pi) { const auto& m = r.metrics[static_cast<std::size_t>(pi)]; for (const auto& p : m.points) { out << r.visitId << "," << r.planIndex + 1 << "," << r.groupIndex + 1 << "," << pi + 1 << "," << p.sampleIndex << "," << m.measurementDistanceMm << "," << m.sectionAngleDeg << "," << m.centerBearingAngleDeg << ","; val(std::isfinite(p.localSurfaceAngleDeg), p.localSurfaceAngleDeg); out << "," << (p.coordinateValid ? 1 : 0) << "," << (p.geometricGood ? 1 : 0) << "," << (p.geometricOutlier ? 1 : 0) << "," << (p.dropout ? 1 : 0) << ","; val(std::isfinite(p.measuredXmm), p.measuredXmm); out << ","; val(std::isfinite(p.measuredZmm), p.measuredZmm); out << ","; val(std::isfinite(p.expectedXmm), p.expectedXmm); out << ","; val(std::isfinite(p.expectedZmm), p.expectedZmm); out << ","; val(std::isfinite(p.shapeResidualMm), p.shapeResidualMm); out << ","; val(std::isfinite(p.absoluteSectionResidualMm), p.absoluteSectionResidualMm); out << ","; val(p.hasWidth, p.width); out << ","; val(p.hasMaximum, p.maximum); out << ","; val(p.hasThreshold, p.threshold); out << ","; val(p.hasMoment0, p.moment0); out << ","; val(p.hasMoment1, p.moment1); out << "\n"; } }
    }

    // First empirical uncertainty/quality surface. This intentionally uses an
    // interpretable 3-D D/gamma/eta kernel regression; GP/boosted-tree models can be trained
    // later from these same lossless profile/point exports without changing acquisition.
    std::vector<mls::calibration::SphereStudyQualityObservation> observations;
    for (const auto& r : m_sphereStudyRecords)for (const auto& m : r.metrics)if (m.valid)observations.push_back({ m.measurementDistanceMm,m.sectionAngleDeg,m.centerBearingAngleDeg,m.absoluteSectionP95Mm,m.radiusBiasMm,m.dropoutRatio,m.geometricOutlierRatio,m.goodPointRatio });
    QFile map(QDir(m_sphereStudySessionDir).filePath("quality_map_kernel_D_gamma_eta.csv")); if (map.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QTextStream out(&map); out << "D_mm,gamma_deg,eta_deg,predicted_p95_abs_mm,predicted_radius_bias_mm,predicted_dropout_ratio,predicted_outlier_ratio,predicted_good_ratio,local_spread_mm,effective_samples,nearest_normalized_distance,low_support\n";
        const double hd = std::max(2.0, 1.5 * m_spinSphereStudyDistanceStep->value()), hg = std::max(5.0, 1.5 * m_spinSphereStudyGammaStep->value()), he = std::max(5.0, 1.5 * m_spinSphereStudyEtaStep->value());
        for (double d = m_spinSphereStudyDistanceStart->value(); d <= m_spinSphereStudyDistanceEnd->value() + 1e-9; d += m_spinSphereStudyDistanceStep->value())for (double g = m_spinSphereStudyGammaStart->value(); g <= m_spinSphereStudyGammaEnd->value() + 1e-9; g += m_spinSphereStudyGammaStep->value())for (double e = m_spinSphereStudyEtaStart->value(); e <= m_spinSphereStudyEtaEnd->value() + 1e-9; e += m_spinSphereStudyEtaStep->value()) { const auto pr = mls::calibration::SphereDistanceAngleStudy::predictQualityKernel(observations, d, g, e, hd, hg, he); if (pr.valid)out << d << "," << g << "," << e << "," << pr.predictedAbsoluteP95Mm << "," << pr.predictedRadiusBiasMm << "," << pr.predictedDropoutRatio << "," << pr.predictedOutlierRatio << "," << pr.predictedGoodPointRatio << "," << pr.localSpreadMm << "," << pr.effectiveSampleCount << "," << pr.nearestNormalizedDistance << "," << (pr.lowSupport ? 1 : 0) << "\n"; }
    }

    QFile inliers(QDir(m_sphereStudySessionDir).filePath("sphere_ransac_inliers.csv")); if (inliers.open(QIODevice::WriteOnly | QIODevice::Text)) { QTextStream out(&inliers); out << "X,Y,Z\n"; for (std::size_t idx : m_sphereStudySphereFit.inlierIndices)if (idx < static_cast<std::size_t>(m_sphereStudyImportedCloud.size())) { const auto& p = m_sphereStudyImportedCloud[static_cast<int>(idx)]; out << QString::number(p.x(), 'f', 6) << "," << QString::number(p.y(), 'f', 6) << "," << QString::number(p.z(), 'f', 6) << "\n"; } }
    m_lblSphereStudyCapture->setText(tr("研究结果已导出：%1。包含原始Rich槽、EvaluationROI点级数据和首版D-γ-η经验质量地图。").arg(m_sphereStudySessionDir));
    QMessageBox::information(this, tr("导出完成"), tr("球面测量距离/截面角研究结果已保存到：\n%1").arg(m_sphereStudySessionDir));
}


int MainWindow::sensorCalibrationRequestedRateHz() const
{
    return m_spinSensorCalibRate ? m_spinSensorCalibRate->value() : 25;
}

int MainWindow::sensorCalibrationVideoIdleUnitsForExposure(int exposureUnits) const
{
    const auto plan = mls::calibration::SensorCalibrationTiming::videoPlan(exposureUnits);
    return plan.valid ? plan.idle10us : 0;
}

bool MainWindow::validateSensorCalibrationVideoTiming(int exposureUnits, QString* errMsg) const
{
    const auto plan = mls::calibration::SensorCalibrationTiming::videoPlan(exposureUnits);
    if (plan.valid) return true;
    if (errMsg) *errMsg = tr("Video固定25Hz：曝光%1 (10µs)必须在1～4000且不超过40ms。%2")
        .arg(exposureUnits).arg(QString::fromLatin1(plan.reason));
    return false;
}

int MainWindow::sensorCalibrationIdleUnitsForExposure(int exposureUnits) const
{
    // Period quantisation uses 10 us units. This is a theoretical configuration,
    // not a measurement of the rate of received Profile callbacks.
    const auto plan = mls::calibration::SensorCalibrationTiming::plan(
        sensorCalibrationRequestedRateHz(), exposureUnits,
        LaserSensor::getMaxFrequencyForIndex(
            m_sensorCalibFieldOverrideActive ? m_sensorCalibActiveFieldIndex
                                             : sensorCalibrationSelectedFieldIndex()));
    // Callers must validate the timing *before* writing any SDK features.
    // The safe fallback avoids unsigned underflow if an unvalidated caller asks.
    return plan.valid ? plan.idle10us : 0;
}

bool MainWindow::validateSensorCalibrationTiming(int exposureUnits, int requestedHz, QString* errMsg) const
{
    const int field = m_sensorCalibFieldOverrideActive
        ? m_sensorCalibActiveFieldIndex : sensorCalibrationSelectedFieldIndex();
    const auto plan = mls::calibration::SensorCalibrationTiming::plan(
        requestedHz, exposureUnits, LaserSensor::getMaxFrequencyForIndex(field));
    if (plan.valid) return true;
    if (errMsg) {
        *errMsg = tr("频率/曝光设置不合法：测量场index=%1，频率=%2 Hz，曝光=%3 (×10µs=%4 ms)。%5。"
                     "系统不会自动修改用户输入的曝光时间；请主动降低曝光或频率。")
            .arg(field).arg(requestedHz).arg(exposureUnits)
            .arg(exposureUnits * 0.01, 0, 'f', 2)
            .arg(QString::fromLatin1(plan.reason));
    }
    return false;
}

void MainWindow::updateSensorCalibrationTimingUi()
{
    if (!m_spinVideoExposure || !m_spinVideoIdle || !m_spinSensorCalibRate) return;

    const int field = sensorCalibrationSelectedFieldIndex();
    const int presetMaxHz = LaserSensor::getMaxFrequencyForIndex(field);
    const int minHz = mls::calibration::SensorCalibrationTiming::kMinHz;
    const int deviceMaxHz = mls::calibration::SensorCalibrationTiming::kMaxHz;
    // The field's project reference limit is enforced in the EDITOR as well
    // as by the start-time validation.  Never allow a manual rate above it.
    // Invalid/unknown fields still fail validation below; do not infer a max
    // from the hardware-wide 2000 Hz limit in that case.
    const int permittedMaxHz = qBound(minHz, presetMaxHz, deviceMaxHz);
    const int previousRequestedHz = m_spinSensorCalibRate->value();
    const bool followFieldLimit = m_checkSensorCalibAutoFieldRate
        && m_checkSensorCalibAutoFieldRate->isChecked();
    {
        QSignalBlocker blocker(m_spinSensorCalibRate);
        m_spinSensorCalibRate->setRange(minHz, permittedMaxHz);
        if (followFieldLimit && presetMaxHz >= minHz && presetMaxHz <= deviceMaxHz)
            m_spinSensorCalibRate->setValue(presetMaxHz);
        // In manual mode QSpinBox preserves any admissible manual rate and
        // automatically limits a stale rate when switching to a smaller field.
    }
    const bool manualRateWasLimited = !followFieldLimit && previousRequestedHz > permittedMaxHz;
    const QString rateLimitedNote = manualRateWasLimited
        ? tr(" 测量场已切换：原频率 %1 Hz 超限，已调整到新测量场上限 %2 Hz。")
            .arg(previousRequestedHz).arg(permittedMaxHz)
        : QString();
    m_spinSensorCalibRate->setToolTip(
        tr("仅Profile使用的目标频率：%1～%2 Hz（当前测量场的参考上限）。"
           "Video始终单独配置为25Hz；Profile实际帧率以设备时间戳为准。")
            .arg(minHz).arg(permittedMaxHz));
    const int requestedHz = sensorCalibrationRequestedRateHz();
    const bool isExposureSweep = m_expTestState != ExpTestState::Idle;
    // In normal preview m_spinVideoExposure is the operator's own editable
    // calibration value.  Do not overwrite it with the formal H1/H3 exposure.
    // During an exposure sweep this same editor shows the current test value,
    // then restores the saved operator value on stop/completion.
    const int exposureUnits = m_spinVideoExposure->value();
    const auto plan = mls::calibration::SensorCalibrationTiming::plan(
        requestedHz, exposureUnits, presetMaxHz);

    // CRITICAL: do not silently clamp the operator's manually selected
    // exposure when changing the requested rate or measuring field.
    m_spinVideoIdle->setValue(plan.valid ? plan.idle10us : 0);
    // Video always has its own 25 Hz period and Idle; never reuse the
    // Profile 144/200/552 Hz plan for raw CMOS image acquisition.
    const auto videoPlan = mls::calibration::SensorCalibrationTiming::videoPlan(exposureUnits);

    // Never write the calibration-only Idle into the formal parameter page:
    // H1/H3 and other machine-run settings have separate timing ownership.
    if (m_lblSensorCalibField) {
        const int resolution = (m_comboResolution && m_comboResolution->currentData().isValid())
            ? m_comboResolution->currentData().toInt() : 1280;
        m_lblSensorCalibField->setText(
            tr("测量场（参数页）：%1 / index=%2 / %3点；上限%4 Hz")
            .arg(m_comboMeasuringField ? m_comboMeasuringField->currentText() : tr("未设置"))
            .arg(field).arg(resolution)
            .arg(presetMaxHz > 0 ? QString::number(presetMaxHz) : tr("未知")));
    }
    if (m_lblSensorCalibTiming) {
        if (plan.valid) {
            m_lblSensorCalibTiming->setStyleSheet("QLabel { color:#555; font-size:10px; }");
            m_lblSensorCalibTiming->setText(
                (isExposureSweep ? tr("曝光测试档位（临时）：") : QString()) +
                tr("周期 %1 ms；理论频率 %2 Hz（实际帧率以设备时间戳为准）。")
                .arg(plan.period10us * 0.01, 0, 'f', 2)
                .arg(plan.theoreticalRateHz, 0, 'f', 3) +
                (videoPlan.valid
                    ? tr(" Video固定25Hz，曝光%1、Video Idle=%2（×10µs）。")
                        .arg(exposureUnits).arg(videoPlan.idle10us)
                    : tr(" Video 25Hz曝光不合法。")) + rateLimitedNote);
        } else {
            m_lblSensorCalibTiming->setStyleSheet("QLabel { color:#b22222; font-size:10px; font-weight:bold; }");
            m_lblSensorCalibTiming->setText(
                tr("轮廓不可启动：%1 Hz + 曝光 %2 ms 不合法（%3）。"
                   "曝光不会被缩短。")
                .arg(requestedHz).arg(exposureUnits * 0.01, 0, 'f', 2)
                .arg(QString::fromLatin1(plan.reason)) +
                (videoPlan.valid
                    ? tr(" Video仍可单独以25Hz启动，Idle=%1（×10µs）。").arg(videoPlan.idle10us)
                    : tr(" Video 25Hz曝光也不合法。")) + rateLimitedNote);
        }
    }
}

bool MainWindow::verifySensorCalibrationRuntimeConstraints(const SensorParams& requested, QString* errMsg, bool checkProfileResolution)
{
    if (!m_laser || !m_laser->isConnected()) {
        if (errMsg) *errMsg = tr("传感器未连接");
        return false;
    }

    int liveField = -1;
    unsigned int liveExposure = 0;
    unsigned int liveIdle = 0;
    QString localErr;
    if (!m_laser->readTimingAndMeasuringField(liveExposure, liveIdle, liveField, &localErr)) {
        if (errMsg) *errMsg = tr("无法回读传感标定运行约束：%1").arg(localErr);
        return false;
    }

    const int expectedField = m_sensorCalibActiveFieldIndex;
    if (expectedField < 0 || liveField != expectedField) {
        if (errMsg) *errMsg = tr("标定测量场不一致：请求index=%1，设备回读index=%2")
            .arg(expectedField).arg(liveField);
        return false;
    }
    if (liveExposure != requested.exposureTime || liveIdle != requested.idleTime) {
        if (errMsg) {
            *errMsg = tr("传感器未接受请求的时序：请求 Exp/Idle=%1/%2，设备回读=%3/%4（单位10µs）")
                .arg(requested.exposureTime).arg(requested.idleTime)
                .arg(liveExposure).arg(liveIdle);
        }
        return false;
    }

    // The selected Measuring Field does not imply a fixed point count.
    // Verify the actual resolution only for Profile data, not VIDEO_IMAGE.
    if (checkProfileResolution) {
        const int requestedPoints = (m_comboResolution && m_comboResolution->currentData().isValid())
            ? m_comboResolution->currentData().toInt() : 0;
        const int actualPoints = static_cast<int>(m_laser->activeResolution());
        if (requestedPoints <= 0 || actualPoints != requestedPoints) {
            if (errMsg) *errMsg = tr("所选轮廓分辨率未生效：请求%1点，设备%2点")
                .arg(requestedPoints).arg(actualPoints);
            return false;
        }
    }

    const int requestedHz = sensorCalibrationRequestedRateHz();
    const double theoreticalHz = (liveExposure + liveIdle) > 0
        ? 100000.0 / static_cast<double>(liveExposure + liveIdle) : 0.0;
    if (theoreticalHz > requestedHz + 0.05) {
        if (errMsg) *errMsg = tr("设备回读时序对应%1 Hz，高于请求%2 Hz")
            .arg(theoreticalHz, 0, 'f', 3).arg(requestedHz);
        return false;
    }

    LOG_INFO(LogCategory::Sensor,
        tr("传感标定配置回读通过: field=%1, Exp=%2, Idle=%3, theoretical=%4Hz, requested=%5Hz（非实际采样率）")
        .arg(liveField).arg(liveExposure).arg(liveIdle)
        .arg(theoreticalHz, 0, 'f', 3).arg(requestedHz));
    return true;
}

int MainWindow::sensorCalibrationSelectedFieldIndex() const
{
    if (m_comboMeasuringField && m_comboMeasuringField->currentData().isValid())
        return m_comboMeasuringField->currentData().toInt();
    return -1;
}

bool MainWindow::enterSensorCalibrationMeasuringField(QString* errMsg)
{
    if (!m_laser || !m_laser->isConnected()) {
        if (errMsg) *errMsg = tr("传感器未连接");
        return false;
    }
    // Only the calibration workflow that already owns the stream may reuse
    // an existing field override.  All fresh field switches require an idle
    // sensor, so hardware-synchronized and other calibration streams are safe.
    if (!m_sensorCalibFieldOverrideActive &&
        ((m_syncManager && m_syncManager->isRunning()) ||
         m_laser->isVideoRunning() || m_laser->isProfileRunning())) {
        if (errMsg) *errMsg = tr("传感器正在采集，禁止切换测量场；请先在原任务中安全停止");
        return false;
    }
    const int requestedField = sensorCalibrationSelectedFieldIndex();
    if (requestedField < 0 || LaserSensor::getMaxFrequencyForIndex(requestedField) <= 0) {
        if (errMsg) *errMsg = tr("测量场index=%1不在当前支持的标定预设中").arg(requestedField);
        return false;
    }
    QString localErr;
    int livePreviousField = -1;
    unsigned int liveExposure = 0, liveIdle = 0;
    if (!m_laser->readTimingAndMeasuringField(liveExposure, liveIdle, livePreviousField, &localErr)) {
        if (errMsg) *errMsg = tr("无法读取设备当前测量场，拒绝切换：%1").arg(localErr);
        return false;
    }
    if (m_sensorCalibFieldOverrideActive) {
        if (m_sensorCalibActiveFieldIndex != requestedField || livePreviousField != requestedField) {
            if (errMsg) *errMsg = tr("标定进行中测量场不可更改：既定%1，设备实际%2")
                .arg(m_sensorCalibActiveFieldIndex).arg(livePreviousField);
            return false;
        }
        return true;  // Do not send SetFeature while an owned stream is active.
    }
    const bool newOverride = !m_sensorCalibFieldOverrideActive;
    if (newOverride) m_sensorCalibPreviousFieldIndex = livePreviousField;
    if (!m_laser->setMeasuringFieldIndex(requestedField, false, &localErr)) {
        QString restoreErr;
        m_laser->setMeasuringFieldIndex(m_sensorCalibPreviousFieldIndex, false, &restoreErr);
        if (errMsg) *errMsg = tr("设置选定测量场index=%1失败：%2；恢复原场号结果：%3")
            .arg(requestedField).arg(localErr).arg(restoreErr.isEmpty() ? tr("已尝试恢复") : restoreErr);
        return false;
    }
    int liveField = -1;
    // liveIdle is already declared above, together with liveExposure.
    unsigned int liveExp = 0;
    if (!m_laser->readTimingAndMeasuringField(liveExp, liveIdle, liveField, &localErr)
        || liveField != requestedField) {
        if (newOverride) {
            QString restoreErr;
            m_laser->setMeasuringFieldIndex(m_sensorCalibPreviousFieldIndex, false, &restoreErr);
        }
        if (errMsg) *errMsg = tr("所选测量场设备回读失败：期望%1，读到%2；%3")
            .arg(requestedField).arg(liveField).arg(localErr);
        return false;
    }
    m_sensorCalibFieldOverrideActive = true;
    m_sensorCalibActiveFieldIndex = requestedField;
    updateSensorCalibrationTimingUi();
    return true;
}

void MainWindow::leaveSensorCalibrationMeasuringField()
{
    if (!m_sensorCalibFieldOverrideActive) return;
    m_sensorCalibFieldOverrideActive = false;
    m_sensorCalibActiveFieldIndex = -1;
    if (!m_laser || !m_laser->isConnected()) return;

    QString err;
    if (!m_laser->setMeasuringFieldIndex(m_sensorCalibPreviousFieldIndex, false, &err)) {
        LOG_WARNING(LogCategory::Sensor,
            tr("传感标定结束后恢复测量区域索引%1失败：%2")
            .arg(m_sensorCalibPreviousFieldIndex).arg(err));
    }
    else {
        int liveField = -1;
        unsigned int liveExp = 0, liveIdle = 0;
        QString verifyErr;
        if (m_laser->readTimingAndMeasuringField(liveExp, liveIdle, liveField, &verifyErr)
            && liveField == m_sensorCalibPreviousFieldIndex) {
            LOG_INFO(LogCategory::Sensor,
                tr("传感标定结束：Measuring Field已恢复为索引%1（设备回读确认）")
                .arg(m_sensorCalibPreviousFieldIndex));
        }
        else {
            LOG_WARNING(LogCategory::Sensor,
                tr("传感标定结束后Measuring Field恢复未获设备确认：期望%1，回读%2，%3")
                .arg(m_sensorCalibPreviousFieldIndex).arg(liveField).arg(verifyErr));
        }
    }
}



SensorAdvancedParams MainWindow::currentSensorAdvancedUiParams() const
{
    SensorAdvancedParams params;
    params.measuringFieldIndex = m_comboMeasuringField ? m_comboMeasuringField->currentData().toInt() : 0;
    params.resolution = m_laser ? m_laser->activeResolution() : 0;
    params.laserPower = m_comboLaserPower ? m_comboLaserPower->currentData().toInt() : 2;
    params.reflectionSelect = m_comboReflection ? m_comboReflection->currentData().toInt() : 3;
    params.thresholdPercent = m_spinThreshold ? m_spinThreshold->value() : 12.5;
    params.flipX = m_checkFlipX && m_checkFlipX->isChecked();
    params.flipZ = !m_checkFlipZ || m_checkFlipZ->isChecked();
    params.autoExposure = m_checkAutoExposure && m_checkAutoExposure->isChecked();
    params.shutterAlign = m_comboShutterAlign ? m_comboShutterAlign->currentData().toInt() : 1;
    params.autoExpMode = m_comboAutoExpMode ? m_comboAutoExpMode->currentData().toInt() : 1;
    params.peakWidthMin = m_spinPeakWidthMin ? m_spinPeakWidthMin->value() : 2;
    params.peakWidthMax = m_spinPeakWidthMax ? m_spinPeakWidthMax->value() : 256;
    params.peakIntensityMin = m_spinPeakIntensityMin ? m_spinPeakIntensityMin->value() : 0;
    params.peakIntensityMax = m_spinPeakIntensityMax ? m_spinPeakIntensityMax->value() : 1023;
    params.highResolution = m_checkHighResolution && m_checkHighResolution->isChecked();
    params.calibration = m_checkCalibration && m_checkCalibration->isChecked();
    params.ambientLightSuppression = m_checkAmbientLight && m_checkAmbientLight->isChecked();
    params.videoFilter = m_checkVideoFilter && m_checkVideoFilter->isChecked();
    return params;
}

bool MainWindow::writeSensorCalibrationSessionManifest(QString* errMsg)
{
    if (m_expTestDir.isEmpty()) {
        if (errMsg) *errMsg = tr("传感标定保存目录为空");
        return false;
    }

    QJsonObject root;
    root[QStringLiteral("schema")] = QStringLiteral("MotionLaserSync.SensorCalibrationSession.v4");
    root[QStringLiteral("applicationVersion")] = QStringLiteral("V27.6.43");
    root[QStringLiteral("createdUtc")] = QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs);
    root[QStringLiteral("scope")] = QStringLiteral("sensor-calibration-debug-only");
    root[QStringLiteral("hardwareSyncAcquisitionModified")] = false;

    QJsonObject sensor;
    sensor[QStringLiteral("typeName")] = m_laser ? m_laser->sensorTypeName() : QStringLiteral("Unknown");
    sensor[QStringLiteral("ip")] = m_sensorIpEdit ? m_sensorIpEdit->text().trimmed() : QString();
    sensor[QStringLiteral("activeResolution")] = m_laser ? static_cast<int>(m_laser->activeResolution()) : 0;
    QString deviceName, serialNumber, identityErr;
    if (m_laser && m_laser->isConnected()) {
        m_laser->readDeviceIdentity(deviceName, serialNumber, &identityErr);
    }
    sensor[QStringLiteral("deviceName")] = deviceName;
    sensor[QStringLiteral("serialNumber")] = serialNumber;
    if (!identityErr.isEmpty()) sensor[QStringLiteral("identityNote")] = identityErr;
    root[QStringLiteral("sensor")] = sensor;

    QJsonObject measuringField;
    measuringField[QStringLiteral("mode")] = QStringLiteral("preset");
    measuringField[QStringLiteral("presetIndex")] = m_sensorCalibActiveFieldIndex;
    measuringField[QStringLiteral("displayName")] = m_comboMeasuringField ? m_comboMeasuringField->currentText() : QStringLiteral("unknown");
    measuringField[QStringLiteral("customRoiConfiguredByThisWorkflow")] = false;
    measuringField[QStringLiteral("previousFieldIndexToRestore")] = m_sensorCalibPreviousFieldIndex;
    root[QStringLiteral("measuringField")] = measuringField;

    QJsonObject sweep;
    sweep[QStringLiteral("targetProfileRateHz")] = sensorCalibrationRequestedRateHz();
    sweep[QStringLiteral("videoRateHz")] = mls::calibration::SensorCalibrationTiming::kVideoRateHz;
    sweep[QStringLiteral("videoPeriod10us")] = 4000;
    sweep[QStringLiteral("videoTimingIndependentOfProfile")] = true;
    sweep[QStringLiteral("rateSelectionMode")] =
        (m_checkSensorCalibAutoFieldRate && m_checkSensorCalibAutoFieldRate->isChecked())
            ? QStringLiteral("follow_field_reference_max")
            : QStringLiteral("manual_target_rate");
    sweep[QStringLiteral("fieldReferenceMaxHz")] = LaserSensor::getMaxFrequencyForIndex(m_sensorCalibActiveFieldIndex);
    sweep[QStringLiteral("autoIdle")] = true;
    sweep[QStringLiteral("exposureStart_10us")] = m_spinExpStart ? m_spinExpStart->value() : 0;
    sweep[QStringLiteral("exposureEnd_10us")] = m_spinExpEnd ? m_spinExpEnd->value() : 0;
    sweep[QStringLiteral("exposureStep_10us")] = m_spinExpStep ? m_spinExpStep->value() : 0;
    sweep[QStringLiteral("samplingMode")] = QStringLiteral("frame-driven");
    sweep[QStringLiteral("stableDiscardFramesPerMode")] = m_expTestStableDiscardFrames;
    sweep[QStringLiteral("videoFramesPerExposure")] = m_expTestVideoFramesPerExposure;
    sweep[QStringLiteral("profileFramesPerExposure")] = m_expTestProfileFramesPerExposure;
    const int rateHz = std::max(1, sensorCalibrationRequestedRateHz());
    sweep[QStringLiteral("nominalFramePeriodMs")] = 1000.0 / rateHz; // legacy: Profile
    sweep[QStringLiteral("nominalProfileFramePeriodMs")] = 1000.0 / rateHz;
    sweep[QStringLiteral("nominalVideoFramePeriodMs")] = 40.0;
    sweep[QStringLiteral("nominalStabilizationMs")] = (1000.0 / rateHz) * m_expTestStableDiscardFrames;
    root[QStringLiteral("sweep")] = sweep;

    QJsonObject environment;
    environment[QStringLiteral("materialFileSafe")] = m_expTestMaterial;
    environment[QStringLiteral("materialInput")] = m_editMaterial ? m_editMaterial->text().trimmed() : m_expTestMaterial;
    environment[QStringLiteral("lightingCode")] = m_expTestLighting;
    environment[QStringLiteral("lightingDisplay")] = m_comboLighting ? m_comboLighting->currentText() : m_expTestLighting;
    environment[QStringLiteral("temperatureC")] = m_expTestTemperature;
    root[QStringLiteral("environment")] = environment;

    QJsonObject pose;
    pose[QStringLiteral("x")] = m_expTestPosX;
    pose[QStringLiteral("y")] = m_expTestPosY;
    pose[QStringLiteral("z")] = m_expTestPosZ;
    pose[QStringLiteral("c")] = m_expTestPosC;
    pose[QStringLiteral("b")] = m_expTestPosB;
    pose[QStringLiteral("linearUnit")] = QStringLiteral("mm");
    pose[QStringLiteral("rotaryUnit")] = QStringLiteral("deg");
    pose[QStringLiteral("xSource")] = QStringLiteral("grating MPOS");
    pose[QStringLiteral("ySource")] = QStringLiteral("grating MPOS");
    pose[QStringLiteral("zSource")] = QStringLiteral("grating MPOS");
    pose[QStringLiteral("cSource")] = QStringLiteral("software mechanical MPOS");
    pose[QStringLiteral("bSource")] = QStringLiteral("software mechanical MPOS");
    root[QStringLiteral("machinePose")] = pose;

    const SensorAdvancedParams uiParams = currentSensorAdvancedUiParams();
    QJsonObject configured = advancedParamsReadableJson(uiParams);
    // Calibration uses the independently selected preset, not a forced full field.
    configured[QStringLiteral("measuringFieldIndex")] = m_sensorCalibActiveFieldIndex;
    configured[QStringLiteral("source")] = QStringLiteral("MotionLaserSync UI/configuration intent");
    configured[QStringLiteral("validReflectionFilter")] = writeOnlyPeakFilterJson(uiParams);
    root[QStringLiteral("configuredAdvancedSettings")] = configured;

    // Read readable settings once per experiment. Do not attempt the unreliable
    // PeakFilter width/intensity readback on this 2950/SDK combination.
    SensorAdvancedParams readback;
    QString readbackErr;
    QJsonObject actual;
    if (m_laser && m_laser->readAdvancedParams(readback, &readbackErr)) {
        actual = advancedParamsReadableJson(readback);
        actual[QStringLiteral("source")] = QStringLiteral("device readback at experiment start");
        actual[QStringLiteral("validReflectionFilterReadback")] = QStringLiteral("not attempted / unreliable on current 2950/SDK");
    }
    else {
        actual[QStringLiteral("available")] = false;
        actual[QStringLiteral("error")] = readbackErr;
    }
    root[QStringLiteral("deviceReadbackAdvancedSettings")] = actual;

    QJsonObject profileSchema;
    QJsonArray pointColumns;
    pointColumns.append(QStringLiteral("PointIndex"));
    pointColumns.append(QStringLiteral("X_mm"));
    pointColumns.append(QStringLiteral("Z_mm"));
    pointColumns.append(QStringLiteral("Width"));
    pointColumns.append(QStringLiteral("Maximum"));
    pointColumns.append(QStringLiteral("Threshold"));
    pointColumns.append(QStringLiteral("Moment0"));
    pointColumns.append(QStringLiteral("Moment1"));
    profileSchema[QStringLiteral("pointColumns")] = pointColumns;
    profileSchema[QStringLiteral("maximumSemantics")] = QStringLiteral("LLT SDK pMaximum (peak maximum / intensity-like output)");
    profileSchema[QStringLiteral("reflectionArgument")] = 0;
    profileSchema[QStringLiteral("auxiliaryChannelsAreRequestedOnlyDuringSensorCalibration")] = true;
    profileSchema[QStringLiteral("hardwareSyncProfilePathRemainsXZOnly")] = true;
    root[QStringLiteral("profileSchema")] = profileSchema;

    QJsonObject savePlan;
    savePlan[QStringLiteral("saveProfile")] = m_expTestSaveProfile;
    savePlan[QStringLiteral("saveVideo")] = m_expTestSaveVideo;
    savePlan[QStringLiteral("profileFramesPerExposure")] = m_expTestProfileFramesPerExposure;
    savePlan[QStringLiteral("videoFramesPerExposure")] = m_expTestVideoFramesPerExposure;
    savePlan[QStringLiteral("stableDiscardFramesBeforeEachMode")] = m_expTestStableDiscardFrames;
    root[QStringLiteral("savePlan")] = savePlan;

    QJsonObject analysisPlan;
    analysisPlan[QStringLiteral("automaticAfterSweep")] = m_expTestSaveProfile;
    analysisPlan[QStringLiteral("algorithmVersion")] = QStringLiteral("V27.6.43-center-segment-knee-v1");
    analysisPlan[QStringLiteral("roiPolicy")] = QStringLiteral("contiguous valid segment nearest optical X=0; split on invalid/index-gap/Z-jump");
    analysisPlan[QStringLiteral("saturationPolicy")] = QStringLiteral("infer clipping from Maximum top-plateau fraction; do not hard-code 1023");
    analysisPlan[QStringLiteral("materialLibraryPersistence")] = QStringLiteral("QStandardPaths::AppDataLocation/calibration/material_exposure_library.json");
    root[QStringLiteral("analysisPlan")] = analysisPlan;

    QJsonObject files;
    files[QStringLiteral("sessionManifest")] = QStringLiteral("sensor_calibration_session.json");
    files[QStringLiteral("profileCsvPattern")] = QStringLiteral("*_profile_fNNN.csv");
    files[QStringLiteral("profileMetaPattern")] = QStringLiteral("*_profile_fNNN_meta.json");
    files[QStringLiteral("profileImagePattern")] = QStringLiteral("*_profile_fNNN.png");
    files[QStringLiteral("videoImagePattern")] = QStringLiteral("*_video_fNNN.png");
    files[QStringLiteral("videoRawPattern")] = QStringLiteral("*_video_fNNN_raw.csv");
    files[QStringLiteral("exposureAnalysisCsv")] = QStringLiteral("exposure_analysis.csv");
    files[QStringLiteral("exposureAnalysisJson")] = QStringLiteral("exposure_analysis.json");
    root[QStringLiteral("files")] = files;

    QFile file(QDir(m_expTestDir).filePath(QStringLiteral("sensor_calibration_session.json")));
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        if (errMsg) *errMsg = tr("无法写入传感标定session清单: %1").arg(file.errorString());
        return false;
    }
    file.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
    file.close();
    return true;
}


void MainWindow::loadMaterialExposureLibrary()
{
    if (!m_materialExposureLibrary) {
        m_materialExposureLibrary = std::make_unique<mls::calibration::MaterialExposureLibrary>();
    }
    QString error;
    const bool ok = m_materialExposureLibrary->load(&error);
    refreshMaterialExposureProfileUi();
    const QString libraryPath = QDir::toNativeSeparators(m_materialExposureLibrary->storagePath());
    if (ok) {
        LOG_INFO(LogCategory::Sensor,
            tr("[材料曝光方案] 材料曝光库已加载：%1 条；用户库：%2")
            .arg(m_materialExposureLibrary->entries().size())
            .arg(libraryPath));
    }
    else {
        LOG_ERROR(LogCategory::Sensor,
            tr("[材料曝光方案] 材料曝光库加载失败：%1；目标用户库：%2")
            .arg(error, libraryPath));
    }
}

void MainWindow::refreshMaterialExposureProfileUi(const QString& selectKey)
{
    if (!m_comboMaterialExposureProfile) return;
    const QString previousKey = selectKey.isEmpty()
        ? m_comboMaterialExposureProfile->currentData().toString() : selectKey;
    QSignalBlocker blocker(m_comboMaterialExposureProfile);
    m_comboMaterialExposureProfile->clear();
    m_comboMaterialExposureProfile->addItem(tr("手动参数（不套用材料库）"), QString());

    if (!m_materialExposureLibrary) return;
    QVector<mls::calibration::MaterialExposureProfile> entries = m_materialExposureLibrary->entries();
    std::sort(entries.begin(), entries.end(), [](const auto& a, const auto& b) {
        const int materialCmp = QString::compare(a.material, b.material, Qt::CaseInsensitive);
        if (materialCmp != 0) return materialCmp < 0;
        const int lightCmp = QString::compare(a.lightingCode, b.lightingCode, Qt::CaseInsensitive);
        if (lightCmp != 0) return lightCmp < 0;
        return a.recommendedExposure10us < b.recommendedExposure10us;
        });
    int selectedIndex = 0;
    for (const auto& p : entries) {
        const QString material = p.material.isEmpty() ? p.materialFileSafe : p.material;
        const QString lighting = p.lightingDisplay.isEmpty() ? p.lightingCode : p.lightingDisplay;
        const QString label = tr("%1 | %2 | %3°C/%4Hz | MF%5 R%6 LP%7 | Exp=%8 (%9 ms)")
            .arg(material, lighting)
            .arg(p.temperatureC, 0, 'f', 1)
            .arg(p.referenceProfileRateHz)
            .arg(p.measuringFieldIndex)
            .arg(p.resolution)
            .arg(p.laserPower)
            .arg(p.recommendedExposure10us)
            .arg(p.recommendedExposure10us * 0.01, 0, 'f', 2);
        m_comboMaterialExposureProfile->addItem(label, p.key);
        const int itemIndex = m_comboMaterialExposureProfile->count() - 1;
        QString tooltip = tr("材料=%1\n光照=%2\n温度=%3°C\n传感器=%4 / %5\n测量场=%6, 分辨率=%7, 激光功率=%8, 反射=%9, 阈值=%10%\n推荐Exp=%11, 标定Idle=%12, 标定Rate=%13Hz\n安全候选=%14~%15, 首次饱和=%16")
            .arg(material, lighting)
            .arg(p.temperatureC, 0, 'f', 1)
            .arg(p.sensorTypeName, p.sensorSerialNumber)
            .arg(p.measuringFieldIndex)
            .arg(p.resolution)
            .arg(p.laserPower)
            .arg(p.reflectionSelect)
            .arg(p.thresholdPercent, 0, 'f', 2)
            .arg(p.recommendedExposure10us)
            .arg(p.referenceIdle10us)
            .arg(p.referenceProfileRateHz)
            .arg(p.safeCandidateMinExposure10us)
            .arg(p.safeCandidateMaxExposure10us)
            .arg(p.saturationOnsetExposure10us);
        if (p.sourcePoseValid) {
            tooltip += tr("\n标定位姿XYZBC=(%1, %2, %3, %4°, %5°)")
                .arg(p.sourceX, 0, 'f', 3).arg(p.sourceY, 0, 'f', 3).arg(p.sourceZ, 0, 'f', 3)
                .arg(p.sourceC, 0, 'f', 3).arg(p.sourceB, 0, 'f', 3);
        }
        m_comboMaterialExposureProfile->setItemData(itemIndex, tooltip, Qt::ToolTipRole);
        if (!previousKey.isEmpty() && p.key == previousKey)
            selectedIndex = itemIndex;
    }
    m_comboMaterialExposureProfile->setCurrentIndex(selectedIndex);
}

void MainWindow::onReloadMaterialExposureLibrary()
{
    const QString key = m_comboMaterialExposureProfile
        ? m_comboMaterialExposureProfile->currentData().toString() : QString();
    if (!m_materialExposureLibrary)
        m_materialExposureLibrary = std::make_unique<mls::calibration::MaterialExposureLibrary>();
    QString error;
    if (!m_materialExposureLibrary->load(&error)) {
        LOG_ERROR(LogCategory::Sensor,
            tr("[材料曝光方案] 材料曝光库刷新失败：%1；目标用户库：%2")
            .arg(error, QDir::toNativeSeparators(m_materialExposureLibrary->storagePath())));
        return;
    }
    refreshMaterialExposureProfileUi(key);
    LOG_INFO(LogCategory::Sensor,
        tr("[材料曝光方案] 材料曝光库已刷新：%1 条；用户库：%2")
        .arg(m_materialExposureLibrary->entries().size())
        .arg(QDir::toNativeSeparators(m_materialExposureLibrary->storagePath())));
}

void MainWindow::onMaterialExposureProfileChanged(int index)
{
    if (!m_comboMaterialExposureProfile || index <= 0) {
        LOG_INFO(LogCategory::Sensor,
            tr("[材料曝光方案] 已切换为手动Exposure/Idle：材料库不会覆盖当前参数。"));
        return;
    }
    applyMaterialExposureProfileByKey(m_comboMaterialExposureProfile->itemData(index).toString());
}

void MainWindow::applyMaterialExposureProfileByKey(const QString& key)
{
    if (key.isEmpty() || !m_materialExposureLibrary || !m_spinExposure || !m_spinIdle) return;
    mls::calibration::MaterialExposureProfile profile;
    if (!m_materialExposureLibrary->findByKey(key, profile)) {
        LOG_WARNING(LogCategory::Sensor,
            tr("[材料曝光方案] 方案不存在，请刷新材料库；key=%1").arg(key));
        return;
    }

    const bool hardwareSync = m_radioHardwareSync && m_radioHardwareSync->isChecked();
    auto timing = mls::calibration::ExposureTimingPolicy::decide(
        profile.recommendedExposure10us,
        profile.referenceIdle10us,
        profile.referenceProfileRateHz,
        hardwareSync,
        hardwareSyncFrequencyHz());
    // A saved material library Idle describes its historical experiment, not
    // the present software-internal requested rate.  Do not silently reapply
    // it over the user's current rate/exposure -> automatic Idle choice.
    if (!hardwareSync) {
        const auto internalPlan = mls::calibration::SensorCalibrationTiming::plan(
            sensorCalibrationRequestedRateHz(), profile.recommendedExposure10us,
            LaserSensor::getMaxFrequencyForIndex(sensorCalibrationSelectedFieldIndex()));
        if (!internalPlan.valid) {
            LOG_WARNING(LogCategory::Sensor,
                tr("[材料曝光方案] 当前目标频率与材料库曝光不兼容，未覆盖参数：%1")
                    .arg(QString::fromLatin1(internalPlan.reason)));
            return;
        }
        timing.valid = true;
        timing.exposure10us = internalPlan.exposure10us;
        timing.idle10us = internalPlan.idle10us;
        timing.period10us = internalPlan.period10us;
        timing.resultingRateHz = internalPlan.theoreticalRateHz;
    }
    if (!timing.valid) {
        LOG_WARNING(LogCategory::Sensor,
            tr("[材料曝光方案] 当前方案无法用于当前同步模式：%1；key=%2")
            .arg(QString::fromStdString(timing.explanation), key));
        return;
    }

    m_spinExposure->setValue(timing.exposure10us);
    m_spinIdle->setValue(timing.idle10us);

    QString conditionNote;
    if (m_comboLaserPower && m_comboLaserPower->currentData().toInt() != profile.laserPower)
        conditionNote += tr("；注意当前激光功率与标定条目不同");
    if (m_comboReflection && m_comboReflection->currentData().toInt() != profile.reflectionSelect)
        conditionNote += tr("；注意当前反射选择与标定条目不同");
    if (m_spinThreshold && std::abs(m_spinThreshold->value() - profile.thresholdPercent) > 0.051)
        conditionNote += tr("；注意当前阈值与标定条目不同");

    const QString modeText = hardwareSync
        ? tr("硬件同步：按%1 Hz（%2 ms）触发周期重新计算Idle")
        .arg(hardwareSyncFrequencyHz())
        .arg(1000.0 / static_cast<double>(hardwareSyncFrequencyHz()), 0, 'f', 6)
        : tr("内部触发：按当前目标%1 Hz自动计算Idle，不复制材料库旧Idle")
              .arg(sensorCalibrationRequestedRateHz());
    const QString material = profile.material.isEmpty() ? profile.materialFileSafe : profile.material;
    const QString lighting = profile.lightingDisplay.isEmpty() ? profile.lightingCode : profile.lightingDisplay;
    const QString appliedMessage =
        tr("[材料曝光方案] 已应用 %1 / %2：Exp=%3，Idle=%4，内部时序≈%5 Hz；%6；标定参考=%7 Hz / Idle=%8；置信度=%9%10；用户库=%11")
        .arg(material, lighting)
        .arg(timing.exposure10us)
        .arg(timing.idle10us)
        .arg(timing.resultingRateHz, 0, 'f', 1)
        .arg(modeText)
        .arg(profile.referenceProfileRateHz)
        .arg(profile.referenceIdle10us)
        .arg(profile.confidence)
        .arg(conditionNote)
        .arg(QDir::toNativeSeparators(m_materialExposureLibrary->storagePath()));
    LOG_INFO(LogCategory::Sensor, appliedMessage);
    if (!conditionNote.isEmpty()) {
        LOG_WARNING(LogCategory::Sensor,
            tr("[材料曝光方案] 当前传感器条件与该标定条目不完全一致%1").arg(conditionNote));
    }
}

void MainWindow::onAnalyzeExposureDirectory()
{
    if (m_expTestState != ExpTestState::Idle) {
        QMessageBox::warning(this, tr("曝光测试进行中"), tr("请等待当前曝光测试结束后再分析其他目录。"));
        return;
    }
    const QString startDir = (m_editExpTestDir && !m_editExpTestDir->text().isEmpty())
        ? m_editExpTestDir->text() : QDir::currentPath();
    const QString dir = QFileDialog::getExistingDirectory(
        this, tr("选择曝光扫频session目录"), startDir);
    if (dir.isEmpty()) return;
    QString summary;
    runExposureAnalysisAndImport(dir, true, &summary);
}

bool MainWindow::runExposureAnalysisAndImport(const QString& sessionDir,
    bool showDialog,
    QString* summary)
{
    if (summary) summary->clear();
    if (m_lblExpAnalysisResult) {
        m_lblExpAnalysisResult->setStyleSheet("color:#345; font-size:9px;");
        m_lblExpAnalysisResult->setText(tr("分析结果：正在解析Profile曝光扫频..."));
    }
    QCoreApplication::processEvents(QEventLoop::ExcludeUserInputEvents, 10);

#ifdef Q_OS_WIN
    const std::filesystem::path sessionPath(sessionDir.toStdWString());
#else
    const std::filesystem::path sessionPath(sessionDir.toStdString());
#endif
    const auto result = mls::calibration::ExposureAnalysisEngine::analyzeDirectory(sessionPath);
    if (!result.ok) {
        const QString error = QString::fromStdString(result.error);
        if (m_lblExpAnalysisResult) {
            m_lblExpAnalysisResult->setStyleSheet("color:#a33; font-size:9px;");
            m_lblExpAnalysisResult->setText(tr("分析失败：%1").arg(error));
        }
        if (summary) *summary = error;
        if (showDialog) QMessageBox::warning(this, tr("曝光分析失败"), error);
        return false;
    }

    std::string writeError;
    const auto csvPath = sessionPath / "exposure_analysis.csv";
    const auto jsonPath = sessionPath / "exposure_analysis.json";
    if (!mls::calibration::ExposureAnalysisEngine::writeSummaryCsv(result, csvPath, &writeError)
        || !mls::calibration::ExposureAnalysisEngine::writeResultJson(result, jsonPath, &writeError)) {
        const QString error = tr("分析完成，但结果文件保存失败：%1").arg(QString::fromStdString(writeError));
        if (m_lblExpAnalysisResult) {
            m_lblExpAnalysisResult->setStyleSheet("color:#a33; font-size:9px;");
            m_lblExpAnalysisResult->setText(error);
        }
        if (summary) *summary = error;
        if (showDialog) QMessageBox::warning(this, tr("曝光分析保存失败"), error);
        return false;
    }

    double recommendedCoverage = 0.0;
    double recommendedPlateau = 0.0;
    double recommendedSignal = 0.0;
    for (const auto& level : result.levels) {
        if (level.exposure10us == result.recommendedExposure10us) {
            recommendedCoverage = level.coverageVsSessionMax;
            recommendedPlateau = level.maxPlateauFraction;
            recommendedSignal = level.signalVsSessionMax;
            break;
        }
    }

    QString libraryNote;
    const QString manifestPath = QDir(sessionDir).filePath(QStringLiteral("sensor_calibration_session.json"));
    QFile manifestFile(manifestPath);
    if (manifestFile.open(QIODevice::ReadOnly)) {
        QJsonParseError parseError;
        const QJsonDocument doc = QJsonDocument::fromJson(manifestFile.readAll(), &parseError);
        if (parseError.error == QJsonParseError::NoError && doc.isObject()) {
            const QJsonObject root = doc.object();
            const QJsonObject environment = root.value(QStringLiteral("environment")).toObject();
            const QJsonObject sensor = root.value(QStringLiteral("sensor")).toObject();
            const QJsonObject measuringField = root.value(QStringLiteral("measuringField")).toObject();
            const QJsonObject configured = root.value(QStringLiteral("configuredAdvancedSettings")).toObject();
            const QJsonObject sweep = root.value(QStringLiteral("sweep")).toObject();
            const QJsonObject machinePose = root.value(QStringLiteral("machinePose")).toObject();

            mls::calibration::MaterialExposureProfile profile;
            profile.material = environment.value(QStringLiteral("materialInput")).toString();
            profile.materialFileSafe = environment.value(QStringLiteral("materialFileSafe")).toString();
            if (profile.material.isEmpty()) profile.material = profile.materialFileSafe;
            profile.lightingCode = environment.value(QStringLiteral("lightingCode")).toString();
            profile.lightingDisplay = environment.value(QStringLiteral("lightingDisplay")).toString();
            if (profile.lightingDisplay.isEmpty()) profile.lightingDisplay = profile.lightingCode;
            profile.temperatureC = environment.value(QStringLiteral("temperatureC")).toDouble(25.0);
            profile.sensorTypeName = sensor.value(QStringLiteral("typeName")).toString();
            profile.sensorSerialNumber = sensor.value(QStringLiteral("serialNumber")).toString();
            profile.resolution = sensor.value(QStringLiteral("activeResolution")).toInt(
                configured.value(QStringLiteral("resolution")).toInt(0));
            profile.measuringFieldIndex = measuringField.value(QStringLiteral("presetIndex")).toInt(
                configured.value(QStringLiteral("measuringFieldIndex")).toInt(0));
            profile.laserPower = configured.value(QStringLiteral("laserPower")).toInt(2);
            profile.reflectionSelect = configured.value(QStringLiteral("reflectionSelect")).toInt(3);
            profile.thresholdPercent = configured.value(QStringLiteral("thresholdPercent")).toDouble(12.5);
            profile.sourcePoseValid = machinePose.contains(QStringLiteral("x"))
                && machinePose.contains(QStringLiteral("y"))
                && machinePose.contains(QStringLiteral("z"))
                && machinePose.contains(QStringLiteral("c"))
                && machinePose.contains(QStringLiteral("b"));
            if (profile.sourcePoseValid) {
                profile.sourceX = machinePose.value(QStringLiteral("x")).toDouble();
                profile.sourceY = machinePose.value(QStringLiteral("y")).toDouble();
                profile.sourceZ = machinePose.value(QStringLiteral("z")).toDouble();
                profile.sourceC = machinePose.value(QStringLiteral("c")).toDouble();
                profile.sourceB = machinePose.value(QStringLiteral("b")).toDouble();
            }
            profile.recommendedExposure10us = result.recommendedExposure10us;
            profile.referenceProfileRateHz = sweep.value(QStringLiteral("targetProfileRateHz")).toInt(0);
            if (profile.referenceProfileRateHz <= 0) {
                const int nominalPeriodMs = sweep.value(QStringLiteral("nominalFramePeriodMs")).toInt(0);
                if (nominalPeriodMs > 0) {
                    profile.referenceProfileRateHz = static_cast<int>(std::lround(
                        1000.0 / static_cast<double>(nominalPeriodMs)));
                }
            }
            if (profile.referenceProfileRateHz > 0) {
                const int period10us = static_cast<int>(std::lround(
                    100000.0 / static_cast<double>(profile.referenceProfileRateHz)));
                profile.referenceIdle10us = std::max(0, period10us - profile.recommendedExposure10us);
            }
            profile.saturationOnsetExposure10us = result.saturationOnsetExposure10us;
            profile.safeCandidateMinExposure10us = result.safeCandidateMinExposure10us;
            profile.safeCandidateMaxExposure10us = result.safeCandidateMaxExposure10us;
            profile.inferredSaturationCeiling = result.inferredSaturationCeiling;
            profile.confidence = QString::fromStdString(result.confidence);
            profile.algorithmVersion = QStringLiteral("V27.6.43-center-segment-knee-v1");
            profile.sourceSessionDirectory = QDir::cleanPath(sessionDir);
            profile.updatedUtc = QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs);
            profile.key = mls::calibration::MaterialExposureLibrary::makeKey(profile);

            if (!m_materialExposureLibrary) {
                m_materialExposureLibrary = std::make_unique<mls::calibration::MaterialExposureLibrary>();
                m_materialExposureLibrary->load(nullptr);
            }
            QString libraryError;
            if (profile.material.trimmed().isEmpty() || profile.lightingCode.trimmed().isEmpty()) {
                libraryNote = tr("；session缺少材料或光照字段，未写材料库");
                LOG_WARNING(LogCategory::Sensor,
                    tr("[材料曝光方案] 曝光分析未写入材料库：session缺少材料或光照字段；session=%1")
                    .arg(sessionDir));
            }
            else if (profile.referenceProfileRateHz <= 0 || profile.referenceIdle10us < 0) {
                libraryNote = tr("；session缺少有效Profile Rate/Idle参考，未写材料库");
                LOG_WARNING(LogCategory::Sensor,
                    tr("[材料曝光方案] 曝光分析未写入材料库：session缺少有效Profile Rate/Idle参考；session=%1")
                    .arg(sessionDir));
            }
            else if (m_materialExposureLibrary->upsert(profile, &libraryError)) {
                refreshMaterialExposureProfileUi();
                libraryNote = tr("；已写入材料曝光库");
                LOG_INFO(LogCategory::Sensor,
                    tr("[材料曝光方案] 曝光分析推荐值已写入材料库：Exp=%1, Idle=%2, Rate=%3 Hz；用户库=%4")
                    .arg(profile.recommendedExposure10us)
                    .arg(profile.referenceIdle10us)
                    .arg(profile.referenceProfileRateHz)
                    .arg(QDir::toNativeSeparators(m_materialExposureLibrary->storagePath())));
            }
            else {
                libraryNote = tr("；材料库写入失败：%1").arg(libraryError);
                LOG_ERROR(LogCategory::Sensor,
                    tr("[材料曝光方案] 材料库写入失败：%1；目标=%2")
                    .arg(libraryError, QDir::toNativeSeparators(m_materialExposureLibrary->storagePath())));
            }
        }
        else {
            libraryNote = tr("；session清单JSON无效，未写材料库");
            LOG_WARNING(LogCategory::Sensor,
                tr("[材料曝光方案] session清单JSON无效，曝光分析结果未写入材料库：%1")
                .arg(manifestPath));
        }
    }
    else {
        libraryNote = tr("；未找到session清单，仅保存分析结果，未写材料库");
        LOG_WARNING(LogCategory::Sensor,
            tr("[材料曝光方案] 未找到session清单，仅保存曝光分析结果，未写材料库：%1")
            .arg(manifestPath));
    }

    const QString analysisSummary = tr(
        "推荐 Exp=%1 (%2 ms)；安全候选=%3~%4；首次饱和=%5；覆盖=%6%；信号=%7%；顶端平台=%8%；分析Profile=%9条%10")
        .arg(result.recommendedExposure10us)
        .arg(result.recommendedExposure10us * 0.01, 0, 'f', 2)
        .arg(result.safeCandidateMinExposure10us)
        .arg(result.safeCandidateMaxExposure10us)
        .arg(result.saturationOnsetExposure10us)
        .arg(recommendedCoverage * 100.0, 0, 'f', 1)
        .arg(recommendedSignal * 100.0, 0, 'f', 1)
        .arg(recommendedPlateau * 100.0, 0, 'f', 2)
        .arg(result.profileFilesAnalyzed)
        .arg(libraryNote);
    if (summary) *summary = analysisSummary;
    if (m_lblExpAnalysisResult) {
        m_lblExpAnalysisResult->setStyleSheet("color:#285; font-size:9px;");
        m_lblExpAnalysisResult->setText(tr("分析结果：%1").arg(analysisSummary));
    }
    LOG_INFO(LogCategory::Sensor, tr("V27.6.43曝光分析完成 - %1；目录=%2")
        .arg(analysisSummary, sessionDir));

    if (showDialog) {
        QMessageBox::information(this, tr("曝光分析完成"),
            tr("%1\n\n已保存：\n%2\n%3\n\n材料库：\n%4")
            .arg(analysisSummary)
            .arg(QDir::toNativeSeparators(QDir(sessionDir).filePath(QStringLiteral("exposure_analysis.csv"))))
            .arg(QDir::toNativeSeparators(QDir(sessionDir).filePath(QStringLiteral("exposure_analysis.json"))))
            .arg(m_materialExposureLibrary
                ? QDir::toNativeSeparators(m_materialExposureLibrary->storagePath())
                : tr("未初始化")));
    }
    return true;
}

void MainWindow::connectAxisSignals(AxisControlWidget* widget)
{
    connect(widget, &AxisControlWidget::homeRequested,
        this, &MainWindow::onAxisHomeRequested);
    connect(widget, &AxisControlWidget::absMoveRequested,
        this, &MainWindow::onAxisAbsMoveRequested);
    connect(widget, &AxisControlWidget::relMoveRequested,
        this, &MainWindow::onAxisRelMoveRequested);
    connect(widget, &AxisControlWidget::jogRequested,
        this, &MainWindow::onAxisJogRequested);
    connect(widget, &AxisControlWidget::stopRequested,
        this, &MainWindow::onAxisStopRequested);
    connect(widget, &AxisControlWidget::servoEnableRequested,
        this, &MainWindow::onRotaryServoEnableRequested);
}

AxisControlWidget* MainWindow::axisWidgetForId(int axisId) const
{
    switch (axisId) {
    case AxisX: return m_xAxisWidget;
    case AxisY: return m_yAxisWidget;
    case AxisZ: return m_zAxisWidget;
    case AxisC: return m_cAxisWidget;
    case AxisB: return m_bAxisWidget;
    default: return nullptr;
    }
}

MainWindow::AxisMoveDiagnosticSnapshot
MainWindow::captureAxisMoveDiagnosticSnapshot(ZmcController* controller, int axisId) const
{
    AxisMoveDiagnosticSnapshot snap;
    snap.utc = QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs);
    if (!controller || !controller->isConnected()) {
        snap.error = tr("控制器未连接");
        return snap;
    }

    float dpos = 0.0f, mpos = 0.0f;
    bool idle = false;
    QString err;
    if (!controller->getAxisPositionPair(axisId, dpos, mpos, idle, &err)) {
        snap.error = err.isEmpty() ? tr("读取DPOS/MPOS失败") : err;
        return snap;
    }

    snap.valid = true;
    snap.dpos = static_cast<double>(dpos);
    snap.mpos = static_cast<double>(mpos);
    snap.idle = idle;
    snap.linearReferenced = (axisId >= AxisX && axisId <= AxisZ)
        ? controller->isLinearMposReferenced(axisId) : false;
    snap.mposSetGeneration = (axisId >= AxisX && axisId <= AxisZ)
        ? controller->linearMposSetGeneration(axisId) : 0;

    float encoder = 0.0f;
    QString encoderErr;
    if (controller->readAxisEncoder(axisId, encoder, &encoderErr)) {
        snap.encoderValid = true;
        snap.encoder = static_cast<double>(encoder);
    }
    else if (!encoderErr.isEmpty()) {
        snap.error = tr("ENCODER读取失败: %1").arg(encoderErr);
    }

    // DATUM_IN 只用于诊断原点区域是否与误差跳变同步。读取失败不影响DPOS/MPOS记录。
    if (axisId >= AxisX && axisId <= AxisZ) {
        int inputNumber = -1, rawValue = -1;
        QString datumErr;
        if (controller->readAxisDatumInputState(axisId, inputNumber, rawValue, &datumErr)) {
            snap.datumInput = inputNumber;
            snap.datumRaw = rawValue;
        }
        else if (!datumErr.isEmpty()) {
            if (!snap.error.isEmpty()) snap.error += QStringLiteral("; ");
            snap.error += tr("DATUM_IN读取失败: %1").arg(datumErr);
        }
    }
    return snap;
}

void MainWindow::appendAxisMoveDiagnosticRecord(const AxisMoveDiagnosticRecord& record)
{
    m_axisMoveDiagRecords.push_back(record);
    if (!m_axisMoveDiagTable) return;

    const auto axisName = [](int axisId) -> QString {
        switch (axisId) {
        case AxisX: return QStringLiteral("X");
        case AxisY: return QStringLiteral("Y");
        case AxisZ: return QStringLiteral("Z");
        case AxisC: return QStringLiteral("C");
        case AxisB: return QStringLiteral("B");
        default: return QStringLiteral("?");
        }
        };
    const auto number = [](double v) -> QString {
        return std::isfinite(v) ? QString::number(v, 'f', 6) : QStringLiteral("-");
        };
    const auto fe = [](const AxisMoveDiagnosticSnapshot& s) -> double {
        return s.valid ? s.dpos - s.mpos : std::numeric_limits<double>::quiet_NaN();
        };

    const double deltaD = record.start.valid && record.endSettled.valid
        ? record.endSettled.dpos - record.start.dpos
        : std::numeric_limits<double>::quiet_NaN();
    const double deltaM = record.start.valid && record.endSettled.valid
        ? record.endSettled.mpos - record.start.mpos
        : std::numeric_limits<double>::quiet_NaN();
    const double deltaError = std::isfinite(deltaD) && std::isfinite(deltaM)
        ? deltaD - deltaM : std::numeric_limits<double>::quiet_NaN();
    const double errorPpm = std::isfinite(deltaError) && std::isfinite(deltaD) && std::abs(deltaD) > 1.0e-12
        ? deltaError / deltaD * 1.0e6
        : std::numeric_limits<double>::quiet_NaN();
    const double feStart = fe(record.start);
    const double feEnd = fe(record.endSettled);
    const double feChange = std::isfinite(feStart) && std::isfinite(feEnd)
        ? feEnd - feStart : std::numeric_limits<double>::quiet_NaN();
    const double postIdleMposDrift = record.endIdle.valid && record.endSettled.valid
        ? record.endSettled.mpos - record.endIdle.mpos
        : std::numeric_limits<double>::quiet_NaN();
    const double deltaEncoder = record.start.encoderValid && record.endSettled.encoderValid
        ? record.endSettled.encoder - record.start.encoder
        : std::numeric_limits<double>::quiet_NaN();
    const bool crossesDposZero = record.start.valid && record.endSettled.valid
        && ((record.start.dpos < 0.0 && record.endSettled.dpos >= 0.0)
            || (record.start.dpos > 0.0 && record.endSettled.dpos <= 0.0));

    QString datumText = QStringLiteral("-");
    if (record.start.datumInput >= 0 || record.endSettled.datumInput >= 0) {
        const int inputNo = record.endSettled.datumInput >= 0
            ? record.endSettled.datumInput : record.start.datumInput;
        datumText = QStringLiteral("IN%1:%2→%3")
            .arg(inputNo)
            .arg(record.start.datumRaw >= 0 ? QString::number(record.start.datumRaw) : QStringLiteral("?"))
            .arg(record.endSettled.datumRaw >= 0 ? QString::number(record.endSettled.datumRaw) : QStringLiteral("?"));
    }

    QString status;
    if (!record.moveOk) {
        status = record.moveError.isEmpty() ? tr("运动失败") : record.moveError;
    }
    else if (!record.start.valid || !record.endSettled.valid) {
        status = tr("位置读取失败");
    }
    else {
        status = tr("OK");
        QStringList notes;
        if (!record.start.error.isEmpty()) notes << record.start.error;
        if (!record.endIdle.error.isEmpty()) notes << record.endIdle.error;
        if (!record.endSettled.error.isEmpty()) notes << record.endSettled.error;
        if (!notes.isEmpty()) status += QStringLiteral("; ") + notes.join(QStringLiteral("; "));
    }

    const QString generationText = QStringLiteral("%1→%2")
        .arg(static_cast<qulonglong>(record.start.mposSetGeneration))
        .arg(static_cast<qulonglong>(record.endSettled.mposSetGeneration));
    const QString referenceText = QStringLiteral("%1→%2")
        .arg(record.start.linearReferenced ? QStringLiteral("Y") : QStringLiteral("N"))
        .arg(record.endSettled.linearReferenced ? QStringLiteral("Y") : QStringLiteral("N"));
    const QString planStep = record.stepLabel.isEmpty() ? record.plan
        : (record.plan.isEmpty() ? record.stepLabel : record.plan + QStringLiteral(" / ") + record.stepLabel);

    const int row = m_axisMoveDiagTable->rowCount();
    m_axisMoveDiagTable->insertRow(row);
    const QStringList values{
        QString::number(row + 1),
        record.phase.isEmpty() ? QStringLiteral("MANUAL") : record.phase,
        planStep,
        axisName(record.axisId),
        record.command,
        number(record.testBaseMm),
        number(record.testDistanceMm),
        record.testDirection > 0 ? QStringLiteral("+") : (record.testDirection < 0 ? QStringLiteral("-") : QString()),
        record.start.valid ? number(record.start.dpos) : QStringLiteral("-"),
        record.start.valid ? number(record.start.mpos) : QStringLiteral("-"),
        record.endSettled.valid ? number(record.endSettled.dpos) : QStringLiteral("-"),
        record.endSettled.valid ? number(record.endSettled.mpos) : QStringLiteral("-"),
        number(record.requestedDelta),
        number(deltaD),
        number(deltaM),
        number(deltaError),
        number(errorPpm),
        number(deltaEncoder),
        number(feStart),
        number(feEnd),
        number(feChange),
        number(postIdleMposDrift),
        crossesDposZero ? QStringLiteral("Y") : QStringLiteral("N"),
        datumText,
        generationText,
        referenceText,
        record.role,
        record.cycle > 0 ? QString::number(record.cycle) : QStringLiteral("-"),
        status
    };
    for (int col = 0; col < values.size(); ++col) {
        auto* item = new QTableWidgetItem(values[col]);
        if (col >= 5 && col <= 21 && col != 7) item->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);
        m_axisMoveDiagTable->setItem(row, col, item);
    }
    m_axisMoveDiagTable->scrollToBottom();

    if (m_lblAxisMoveDiagStatus) {
        m_lblAxisMoveDiagStatus->setText(
            tr("记录%1条；最新 %2 %3：ΔDPOS=%4，ΔMPOS=%5，Δ误差=%6，%7 ppm，ΔENC=%8，FE %9→%10，SetMpos# %11→%12")
            .arg(m_axisMoveDiagRecords.size())
            .arg(axisName(record.axisId), record.command)
            .arg(number(deltaD), number(deltaM), number(deltaError), number(errorPpm), number(deltaEncoder), number(feStart), number(feEnd))
            .arg(static_cast<qulonglong>(record.start.mposSetGeneration))
            .arg(static_cast<qulonglong>(record.endSettled.mposSetGeneration)));
    }
}

void MainWindow::startManualAxisMoveDiagnostic()
{
    if (!m_zmc || !m_zmc->isConnected()) {
        QMessageBox::warning(this, tr("位移诊断"), tr("请先连接控制器"));
        return;
    }
    const int axisId = m_comboAxisMoveDiagManualAxis
        ? m_comboAxisMoveDiagManualAxis->currentData().toInt() : AxisZ;
    if (m_zmc->isAxisBusy(axisId)) {
        QMessageBox::information(this, tr("位移诊断"), tr("当前轴仍有运动任务，请停稳后再记录起点"));
        return;
    }
    if (m_btnAxisMoveDiagManualStart) m_btnAxisMoveDiagManualStart->setEnabled(false);
    ++m_backgroundIoCount;
    ZmcController* controller = m_zmc.get();
    (void)QtConcurrent::run([this, controller, axisId]() {
        const auto snap = captureAxisMoveDiagnosticSnapshot(controller, axisId);
        QMetaObject::invokeMethod(this, [this, axisId, snap]() {
            --m_backgroundIoCount;
            if (m_btnAxisMoveDiagManualStart) m_btnAxisMoveDiagManualStart->setEnabled(true);
            if (!m_closeRequested) {
                if (!snap.valid) {
                    QMessageBox::warning(this, tr("记录起点失败"), snap.error);
                }
                else {
                    m_axisMoveDiagManualStart = snap;
                    m_axisMoveDiagManualAxis = axisId;
                    if (m_lblAxisMoveDiagStatus) {
                        m_lblAxisMoveDiagStatus->setText(
                            tr("手工起点已记录：DPOS=%1，MPOS=%2，FE=%3。现在移动轴，然后点击“记录终点并添加”。")
                            .arg(snap.dpos, 0, 'f', 6).arg(snap.mpos, 0, 'f', 6)
                            .arg(snap.dpos - snap.mpos, 0, 'f', 6));
                    }
                }
            }
            tryFinishDeferredClose();
            });
        });
}

void MainWindow::finishManualAxisMoveDiagnostic()
{
    if (!m_zmc || !m_zmc->isConnected()) {
        QMessageBox::warning(this, tr("位移诊断"), tr("请先连接控制器"));
        return;
    }
    const int axisId = m_comboAxisMoveDiagManualAxis
        ? m_comboAxisMoveDiagManualAxis->currentData().toInt() : AxisZ;
    if (m_zmc->isAxisBusy(axisId)) {
        QMessageBox::information(this, tr("位移诊断"), tr("当前轴仍有运动任务，请停稳后再记录终点"));
        return;
    }
    if (!m_axisMoveDiagManualStart.valid || m_axisMoveDiagManualAxis != axisId) {
        QMessageBox::information(this, tr("位移诊断"), tr("请先为当前选择的轴记录起点"));
        return;
    }
    const int settleMs = m_spinAxisMoveDiagSettleMs ? m_spinAxisMoveDiagSettleMs->value() : 2000;
    const QString phase = m_comboAxisMoveDiagPhase ? m_comboAxisMoveDiagPhase->currentData().toString() : QStringLiteral("MANUAL");
    const AxisMoveDiagnosticSnapshot start = m_axisMoveDiagManualStart;
    if (m_btnAxisMoveDiagManualEnd) m_btnAxisMoveDiagManualEnd->setEnabled(false);
    ++m_backgroundIoCount;
    ZmcController* controller = m_zmc.get();
    (void)QtConcurrent::run([this, controller, axisId, settleMs, start, phase]() {
        AxisMoveDiagnosticRecord record;
        record.axisId = axisId;
        record.command = QStringLiteral("manual");
        record.phase = phase;
        record.plan = QStringLiteral("MANUAL");
        record.role = QStringLiteral("manual");
        record.settleMs = settleMs;
        record.start = start;
        record.moveOk = true;
        record.endIdle = captureAxisMoveDiagnosticSnapshot(controller, axisId);
        if (settleMs > 0) QThread::msleep(static_cast<unsigned long>(settleMs));
        record.endSettled = captureAxisMoveDiagnosticSnapshot(controller, axisId);
        QMetaObject::invokeMethod(this, [this, record]() {
            --m_backgroundIoCount;
            if (m_btnAxisMoveDiagManualEnd) m_btnAxisMoveDiagManualEnd->setEnabled(true);
            if (!m_closeRequested) {
                appendAxisMoveDiagnosticRecord(record);
                m_axisMoveDiagManualStart = AxisMoveDiagnosticSnapshot{};
                m_axisMoveDiagManualAxis = -1;
            }
            tryFinishDeferredClose();
            });
        });
}

QVector<MainWindow::AxisMoveDiagnosticAutoStep>
MainWindow::buildAutomaticAxisMoveDiagnosticPlan(
    QString phase,
    QString plan,
    int repeats,
    const QVector<double>& basePositions,
    const QVector<double>& testDistances) const
{
    QVector<AxisMoveDiagnosticAutoStep> steps;
    repeats = std::max(1, repeats);

    const auto addAbs = [&](double target,
        const QString& role,
        const QString& label,
        int cycle,
        double base = std::numeric_limits<double>::quiet_NaN(),
        double distance = std::numeric_limits<double>::quiet_NaN(),
        int direction = 0) {
            AxisMoveDiagnosticAutoStep step;
            step.relative = false;
            step.value = target;
            step.role = role;
            step.label = label;
            step.cycle = cycle;
            step.testBaseMm = base;
            step.testDistanceMm = distance;
            step.testDirection = direction;
            steps.push_back(step);
        };
    const auto addRel = [&](double delta,
        const QString& role,
        const QString& label,
        int cycle,
        double distance = std::numeric_limits<double>::quiet_NaN(),
        int direction = 0) {
            AxisMoveDiagnosticAutoStep step;
            step.relative = true;
            step.value = delta;
            step.role = role;
            step.label = label;
            step.cycle = cycle;
            step.testDistanceMm = distance;
            step.testDirection = direction;
            steps.push_back(step);
        };

    // PRE_HOME: absolute machine coordinates are not trustworthy.  The matrix
    // therefore tests only commanded distances around the current location and
    // always returns to its starting DPOS after each forward/reverse pair.
    if (phase == QStringLiteral("PRE_HOME")) {
        const QVector<double> distances = testDistances.isEmpty()
            ? QVector<double>{5.0}
        : testDistances;
        for (const double distance : distances) {
            for (int cycle = 1; cycle <= repeats; ++cycle) {
                addRel(+distance, QStringLiteral("measured"),
                    QStringLiteral("PRE d=%1 forward").arg(distance, 0, 'g', 8),
                    cycle, distance, +1);
                addRel(-distance, QStringLiteral("measured"),
                    QStringLiteral("PRE d=%1 reverse").arg(distance, 0, 'g', 8),
                    cycle, distance, -1);
            }
        }
        return steps;
    }

    if (plan == QStringLiteral("DISTANCE_POSITION_MATRIX")) {
        // Each (base, distance) pair measures both directions on exactly the
        // same physical interval: base -> base+distance -> base.  This makes
        // distance-scaling and absolute-position/screw-phase effects separable.
        for (const double base : basePositions) {
            for (const double distance : testDistances) {
                addAbs(base, QStringLiteral("positioning"),
                    QStringLiteral("matrix base=%1 d=%2 reposition")
                    .arg(base, 0, 'g', 8).arg(distance, 0, 'g', 8),
                    0, base, distance, 0);
                for (int cycle = 1; cycle <= repeats; ++cycle) {
                    addAbs(base + distance, QStringLiteral("measured"),
                        QStringLiteral("matrix base=%1 d=%2 forward")
                        .arg(base, 0, 'g', 8).arg(distance, 0, 'g', 8),
                        cycle, base, distance, +1);
                    addAbs(base, QStringLiteral("measured"),
                        QStringLiteral("matrix base=%1 d=%2 reverse")
                        .arg(base, 0, 'g', 8).arg(distance, 0, 'g', 8),
                        cycle, base, distance, -1);
                }
            }
        }
        return steps;
    }

    if (plan == QStringLiteral("RELATIVE_5MM")) {
        for (int cycle = 1; cycle <= repeats; ++cycle) {
            addRel(+5.0, QStringLiteral("measured"), QStringLiteral("PRE +5"), cycle, 5.0, +1);
            addRel(-5.0, QStringLiteral("measured"), QStringLiteral("PRE return -5"), cycle, 5.0, -1);
        }
        return steps;
    }

    if (plan == QStringLiteral("ZERO_FINE_SCAN")) {
        QVector<double> targets;
        for (double q = -2.0; q <= 5.000001; q += 0.5) targets.push_back(q);
        for (int cycle = 1; cycle <= repeats; ++cycle) {
            addAbs(-2.0, QStringLiteral("positioning"), QStringLiteral("fine reposition -2"), cycle);
            for (int i = 1; i < targets.size(); ++i) {
                addAbs(targets[i], QStringLiteral("measured"),
                    QStringLiteral("fine forward %1").arg(targets[i], 0, 'f', 1),
                    cycle, targets[i - 1], 0.5, +1);
            }
            for (int i = targets.size() - 2; i >= 0; --i) {
                addAbs(targets[i], QStringLiteral("measured"),
                    QStringLiteral("fine reverse %1").arg(targets[i], 0, 'f', 1),
                    cycle, targets[i + 1], 0.5, -1);
            }
        }
        return steps;
    }

    // Legacy three-segment 5 mm test retained for direct comparison with
    // V27.6.27/V27.6.28 data.
    struct Segment { double a; double b; const char* name; };
    const Segment segments[] = {
        {-11.0, -6.0, "far_negative"},
        {-5.5, -0.5, "near_zero_no_cross"},
        {-0.5,  4.5, "cross_zero"},
    };
    for (const auto& seg : segments) {
        const QString name = QString::fromLatin1(seg.name);
        addAbs(seg.a, QStringLiteral("positioning"),
            QStringLiteral("%1 reposition").arg(name), 0, seg.a, 5.0, 0);
        for (int cycle = 1; cycle <= repeats; ++cycle) {
            addAbs(seg.b, QStringLiteral("measured"),
                QStringLiteral("%1 +5").arg(name), cycle, seg.a, 5.0, +1);
            addAbs(seg.a, QStringLiteral("measured"),
                QStringLiteral("%1 -5").arg(name), cycle, seg.a, 5.0, -1);
        }
    }
    return steps;
}

void MainWindow::updateAutomaticAxisMoveDiagnosticUi()
{
    if (!m_comboAxisMoveDiagAutoAxis || !m_comboAxisMoveDiagPhase
        || !m_comboAxisMoveDiagPlan || !m_lblAxisMoveDiagPlan) {
        return;
    }

    const int axisSelection = m_comboAxisMoveDiagAutoAxis->currentData().toInt();
    const QString axesText = axisSelection < 0 ? tr("X/Y/Z依次") : axisName5D(axisSelection);
    const QString phase = m_comboAxisMoveDiagPhase->currentData().toString();
    QString plan = m_comboAxisMoveDiagPlan->currentData().toString();

    if (phase == QStringLiteral("PRE_HOME")) {
        const int matrixIndex = m_comboAxisMoveDiagPlan->findData(QStringLiteral("DISTANCE_POSITION_MATRIX"));
        if (matrixIndex >= 0 && m_comboAxisMoveDiagPlan->currentIndex() != matrixIndex) {
            QSignalBlocker blocker(m_comboAxisMoveDiagPlan);
            m_comboAxisMoveDiagPlan->setCurrentIndex(matrixIndex);
        }
        plan = QStringLiteral("DISTANCE_POSITION_MATRIX");
        m_comboAxisMoveDiagPlan->setEnabled(false);
        if (m_editAxisMoveDiagBasePositions) m_editAxisMoveDiagBasePositions->setEnabled(false);
        if (m_editAxisMoveDiagDistances) m_editAxisMoveDiagDistances->setEnabled(!m_axisMoveDiagAutoRunning);

        QVector<double> distances;
        QString parseError;
        if (!m_editAxisMoveDiagDistances
            || !parseDiagnosticDoubleList(m_editAxisMoveDiagDistances->text(), distances, &parseError, true)) {
            m_lblAxisMoveDiagPlan->setText(
                tr("PRE_HOME %1：距离列表无效：%2").arg(axesText, parseError));
            return;
        }

        const int repeats = m_spinAxisMoveDiagRepeats ? m_spinAxisMoveDiagRepeats->value() : 1;
        const int stepsPerAxis = distances.size() * repeats * 2;
        const int settleMs = m_spinAxisMoveDiagSettleMs ? m_spinAxisMoveDiagSettleMs->value() : 2000;
        const double settleOnlySeconds = stepsPerAxis * settleMs / 1000.0;
        m_lblAxisMoveDiagPlan->setText(
            tr("PRE_HOME %1 多距离测试：不调用SetMpos、不依赖绝对坐标；从当前未知位置对每个距离执行 +d→返回，"
                "距离={%2} mm；每轴%3步，仅稳定等待约%4 s（另加实际运动时间）。"
                "开始前请确认正方向至少有“最大距离+安全余量”的机械空间。")
            .arg(axesText, formatDiagnosticDoubleList(distances))
            .arg(stepsPerAxis)
            .arg(settleOnlySeconds, 0, 'f', 1));
        return;
    }

    m_comboAxisMoveDiagPlan->setEnabled(!m_axisMoveDiagAutoRunning);
    plan = m_comboAxisMoveDiagPlan->currentData().toString();
    const bool matrixPlan = plan == QStringLiteral("DISTANCE_POSITION_MATRIX");
    if (m_editAxisMoveDiagBasePositions) {
        m_editAxisMoveDiagBasePositions->setEnabled(matrixPlan && !m_axisMoveDiagAutoRunning);
    }
    if (m_editAxisMoveDiagDistances) {
        m_editAxisMoveDiagDistances->setEnabled(matrixPlan && !m_axisMoveDiagAutoRunning);
    }

    if (matrixPlan) {
        QVector<double> bases;
        QVector<double> distances;
        QString error;
        if (!m_editAxisMoveDiagBasePositions
            || !parseDiagnosticDoubleList(m_editAxisMoveDiagBasePositions->text(), bases, &error, false)) {
            m_lblAxisMoveDiagPlan->setText(tr("POST_HOME %1：起点列表无效：%2").arg(axesText, error));
            return;
        }
        if (!m_editAxisMoveDiagDistances
            || !parseDiagnosticDoubleList(m_editAxisMoveDiagDistances->text(), distances, &error, true)) {
            m_lblAxisMoveDiagPlan->setText(tr("POST_HOME %1：距离列表无效：%2").arg(axesText, error));
            return;
        }

        const int repeats = m_spinAxisMoveDiagRepeats ? m_spinAxisMoveDiagRepeats->value() : 1;
        const int pairCount = bases.size() * distances.size();
        const int stepsPerAxis = pairCount * (1 + 2 * repeats);
        const int settleMs = m_spinAxisMoveDiagSettleMs ? m_spinAxisMoveDiagSettleMs->value() : 2000;
        const double settleOnlyMinutes = stepsPerAxis * settleMs / 60000.0;
        m_lblAxisMoveDiagPlan->setText(
            tr("POST_HOME %1 多距离/多位置矩阵：起点={%2} mm；距离={%3} mm。"
                "每个组合执行“起点→起点+d→起点”，正反向使用同一物理区间；共%4组合、每轴%5步，"
                "仅稳定等待约%6 min。10/20/30mm若接近整丝杠周期而误差显著减小，支持周期误差；"
                "若ΔD-ΔM随距离近似线性增长且ppm稳定，则更像比例/线性项。角度pitch/yaw仍需外部空间测量确认。")
            .arg(axesText,
                formatDiagnosticDoubleList(bases),
                formatDiagnosticDoubleList(distances))
            .arg(pairCount)
            .arg(stepsPerAxis)
            .arg(settleOnlyMinutes, 0, 'f', 1));
    }
    else if (plan == QStringLiteral("ZERO_FINE_SCAN")) {
        m_lblAxisMoveDiagPlan->setText(
            tr("POST_HOME %1 零点细扫：每个轴执行 -2.0→+5.0→-2.0 mm，0.5 mm步进。"
                "保留该计划用于与之前零点附近结果直接比较。")
            .arg(axesText));
    }
    else if (plan == QStringLiteral("RELATIVE_5MM")) {
        m_lblAxisMoveDiagPlan->setText(
            tr("POST_HOME %1：当前位置相对5mm往返；该旧计划仅用于兼容。建议改用“多距离/多位置矩阵”。")
            .arg(axesText));
    }
    else {
        m_lblAxisMoveDiagPlan->setText(
            tr("POST_HOME %1 三段5mm对比：A -11↔-6 mm；B -5.5↔-0.5 mm；C -0.5↔+4.5 mm。"
                "保留用于与V27.6.27/V27.6.28历史数据直接比较。")
            .arg(axesText));
    }
}

void MainWindow::startAutomaticAxisMoveDiagnostic()
{
    if (m_zmc && m_zmc->axisCompensationEnabled()) {
        QMessageBox::warning(this, tr("RAW 采集/诊断/验证"),
            tr("请先在 XYZ 运行补偿面板禁用运行补偿，再启动 RAW 采集或独立 A/B 验证。"));
        return;
    }
    if (m_axisMoveDiagAutoRunning) return;
    if (m_axisCompSpeedRunning || m_axisCompCalRunning || m_axisCompValRunning) {
        QMessageBox::information(this, tr("自动位移诊断"), tr("采表测速/补偿表采集/A/B验证仍在运行，请先停止。"));
        return;
    }
    if (!m_zmc || !m_zmc->isConnected()) {
        QMessageBox::warning(this, tr("自动位移诊断"), tr("请先连接控制器"));
        return;
    }
    for (int axis = AxisX; axis <= AxisB; ++axis) {
        if (m_zmc->isAxisBusy(axis)) {
            QMessageBox::information(this, tr("自动位移诊断"), tr("当前仍有轴运动任务，请全部停稳后再开始"));
            return;
        }
    }

    const int axisSelection = m_comboAxisMoveDiagAutoAxis ? m_comboAxisMoveDiagAutoAxis->currentData().toInt() : AxisZ;
    QVector<int> axes;
    if (axisSelection < 0) axes = { AxisX, AxisY, AxisZ };
    else if (axisSelection >= AxisX && axisSelection <= AxisZ) axes = { axisSelection };
    else {
        QMessageBox::warning(this, tr("自动位移诊断"), tr("自动测试仅支持X/Y/Z线性轴"));
        return;
    }
    QStringList axisNames;
    for (const int axis : axes) axisNames << axisName5D(axis);
    const QString axesText = axisNames.join(QStringLiteral("/"));

    const QString phase = m_comboAxisMoveDiagPhase ? m_comboAxisMoveDiagPhase->currentData().toString() : QStringLiteral("POST_HOME");
    QString plan = m_comboAxisMoveDiagPlan ? m_comboAxisMoveDiagPlan->currentData().toString() : QStringLiteral("DISTANCE_POSITION_MATRIX");
    const int repeats = m_spinAxisMoveDiagRepeats ? m_spinAxisMoveDiagRepeats->value() : 5;
    const int settleMs = m_spinAxisMoveDiagSettleMs ? m_spinAxisMoveDiagSettleMs->value() : 2000;

    QVector<double> basePositions;
    QVector<double> testDistances;
    QString listError;
    if (phase == QStringLiteral("PRE_HOME") || plan == QStringLiteral("DISTANCE_POSITION_MATRIX")) {
        if (!m_editAxisMoveDiagDistances
            || !parseDiagnosticDoubleList(m_editAxisMoveDiagDistances->text(), testDistances, &listError, true)) {
            QMessageBox::warning(this, tr("自动位移诊断"), tr("移动距离列表无效：%1").arg(listError));
            return;
        }
    }
    if (phase == QStringLiteral("POST_HOME") && plan == QStringLiteral("DISTANCE_POSITION_MATRIX")) {
        if (!m_editAxisMoveDiagBasePositions
            || !parseDiagnosticDoubleList(m_editAxisMoveDiagBasePositions->text(), basePositions, &listError, false)) {
            QMessageBox::warning(this, tr("自动位移诊断"), tr("起点列表无效：%1").arg(listError));
            return;
        }
    }

    QStringList wrongReferenceAxes;
    if (phase == QStringLiteral("PRE_HOME")) {
        plan = QStringLiteral("DISTANCE_POSITION_MATRIX");
        for (const int axis : axes) {
            if (m_zmc->isLinearMposReferenced(axis)) wrongReferenceAxes << axisName5D(axis);
        }
        if (!wrongReferenceAxes.isEmpty()) {
            QMessageBox::warning(this, tr("自动位移诊断"),
                tr("PRE_HOME要求所选轴尚未执行本次连接的DATUM/SetMpos。以下轴已经回零：%1。"
                    "若要比较上电未回零状态，请重新上电/重连后，在这些轴回零之前运行PRE_HOME。")
                .arg(wrongReferenceAxes.join(QStringLiteral(", "))));
            return;
        }
    }
    else {
        for (const int axis : axes) {
            if (!m_zmc->isLinearMposReferenced(axis)) wrongReferenceAxes << axisName5D(axis);
        }
        if (!wrongReferenceAxes.isEmpty()) {
            QMessageBox::warning(this, tr("自动位移诊断"),
                tr("POST_HOME要求所有选中轴都已经正规DATUM回零并建立MPOS零点。以下轴尚未完成：%1。")
                .arg(wrongReferenceAxes.join(QStringLiteral(", "))));
            return;
        }
    }

    const QVector<AxisMoveDiagnosticAutoStep> steps =
        buildAutomaticAxisMoveDiagnosticPlan(phase, plan, repeats, basePositions, testDistances);
    if (steps.isEmpty()) {
        QMessageBox::warning(this, tr("自动位移诊断"), tr("自动测试计划为空，请检查起点和距离列表。"));
        return;
    }
    const auto axisParams = currentAxisParams5D();

    if (phase == QStringLiteral("POST_HOME")) {
        double minTarget = std::numeric_limits<double>::infinity();
        double maxTarget = -std::numeric_limits<double>::infinity();
        for (const auto& step : steps) if (!step.relative) {
            minTarget = std::min(minTarget, step.value);
            maxTarget = std::max(maxTarget, step.value);
        }
        QStringList limitFailures;
        for (const int axis : axes) {
            const AxisParams& motion = axisParams[axis];
            if (minTarget < static_cast<double>(motion.softLimitNeg) || maxTarget > static_cast<double>(motion.softLimitPos)) {
                limitFailures << tr("%1:[%2,%3]")
                    .arg(axisName5D(axis))
                    .arg(motion.softLimitNeg, 0, 'f', 3).arg(motion.softLimitPos, 0, 'f', 3);
            }
        }
        if (!limitFailures.isEmpty()) {
            QMessageBox::warning(this, tr("自动位移诊断"),
                tr("测试绝对范围[%1,%2] mm超出以下轴当前软限位，已拒绝启动：%3")
                .arg(minTarget, 0, 'f', 3).arg(maxTarget, 0, 'f', 3)
                .arg(limitFailures.join(QStringLiteral("; "))));
            return;
        }
    }

    QString confirmation;
    if (phase == QStringLiteral("PRE_HOME")) {
        double maxDistance = 0.0;
        for (const double distance : testDistances) maxDistance = std::max(maxDistance, distance);
        confirmation = tr(
            "将运行PRE_HOME多距离诊断，轴=%1：\n"
            "• 不执行DATUM，也不调用SetMpos；\n"
            "• 距离={%2} mm；每个距离执行 +d→返回，重复%3次；\n"
            "• 每个轴最大正向偏离启动位置%4 mm，随后回到启动DPOS；\n"
            "• XYZ依次时按X→Y→Z执行；\n"
            "• 未回零时绝对坐标和软限位不能作为机床真实位置依据。\n\n"
            "请确认所有将测试的轴正方向至少有“%4 mm + 机械安全余量”，并有人监护急停。")
            .arg(axesText, formatDiagnosticDoubleList(testDistances))
            .arg(repeats)
            .arg(maxDistance, 0, 'g', 8);
    }
    else {
        QString planText;
        if (plan == QStringLiteral("DISTANCE_POSITION_MATRIX")) {
            planText = tr("多距离/多位置矩阵：起点={%1} mm；距离={%2} mm；每组合 起点→起点+d→起点")
                .arg(formatDiagnosticDoubleList(basePositions),
                    formatDiagnosticDoubleList(testDistances));
        }
        else if (plan == QStringLiteral("ZERO_FINE_SCAN")) {
            planText = tr("每轴 -2→+5→-2 mm，0.5 mm步进");
        }
        else if (plan == QStringLiteral("RELATIVE_5MM")) {
            planText = tr("当前位置相对5mm往返");
        }
        else {
            planText = tr("每轴三段5 mm：-11↔-6、-5.5↔-0.5、-0.5↔+4.5 mm");
        }
        confirmation = tr(
            "将运行POST_HOME诊断，轴=%1：\n"
            "• 所有选中轴均已建立MPOS机床零点；\n"
            "• 计划：%2；重复%3次；\n"
            "• 每步IDLE后再等待%4 ms；\n"
            "• 每个轴完成测试后恢复该轴开始诊断时的DPOS，再测试下一轴；\n"
            "• 运动期间本程序不会调用SetMpos；CSV会记录测试基准/距离/方向、误差ppm、ENCODER和DATUM_IN。\n\n"
            "请确认上述绝对位置和移动距离范围内机械、工装和传感器均安全，并有人监护急停。")
            .arg(axesText).arg(planText).arg(repeats).arg(settleMs);
    }
    if (QMessageBox::warning(this, tr("确认自动XYZ轴诊断"), confirmation,
        QMessageBox::Yes | QMessageBox::No, QMessageBox::No) != QMessageBox::Yes) {
        return;
    }

    m_axisMoveDiagAutoRunning = true;
    m_axisMoveDiagAutoCancel.store(false);
    updateAxisCompensationValidationUi();
    if (m_progressAxisMoveDiagAuto) m_progressAxisMoveDiagAuto->setValue(0);
    if (m_btnAxisMoveDiagAutoStart) m_btnAxisMoveDiagAutoStart->setEnabled(false);
    if (m_btnAxisMoveDiagAutoStop) m_btnAxisMoveDiagAutoStop->setEnabled(true);
    if (m_btnAxisMoveDiagManualStart) m_btnAxisMoveDiagManualStart->setEnabled(false);
    if (m_btnAxisMoveDiagManualEnd) m_btnAxisMoveDiagManualEnd->setEnabled(false);
    if (m_btnAxisMoveDiagClear) m_btnAxisMoveDiagClear->setEnabled(false);
    if (m_comboAxisMoveDiagAutoAxis) m_comboAxisMoveDiagAutoAxis->setEnabled(false);
    if (m_comboAxisMoveDiagPhase) m_comboAxisMoveDiagPhase->setEnabled(false);
    if (m_comboAxisMoveDiagPlan) m_comboAxisMoveDiagPlan->setEnabled(false);
    if (m_editAxisMoveDiagBasePositions) m_editAxisMoveDiagBasePositions->setEnabled(false);
    if (m_editAxisMoveDiagDistances) m_editAxisMoveDiagDistances->setEnabled(false);
    if (m_spinAxisMoveDiagRepeats) m_spinAxisMoveDiagRepeats->setEnabled(false);
    if (m_btnHomeAll) m_btnHomeAll->setEnabled(false);
    if (m_btnZeroAll) m_btnZeroAll->setEnabled(false);
    for (int axis = AxisX; axis <= AxisB; ++axis) {
        if (auto* w = axisWidgetForId(axis)) w->setControlsEnabled(false);
    }
    const int restorePerAxis = phase == QStringLiteral("POST_HOME") ? 1 : 0;
    const int totalSteps = axes.size() * (steps.size() + restorePerAxis);
    if (m_lblAxisMoveDiagStatus) {
        m_lblAxisMoveDiagStatus->setText(tr("自动XYZ位移诊断运行中：轴=%1；%2 / %3；共%4步。")
            .arg(axesText, phase, plan).arg(totalSteps));
    }

    ++m_axisWorkerCount;
    ZmcController* controller = m_zmc.get();
    (void)QtConcurrent::run([this, controller, axes, steps, axisParams, phase, plan, settleMs, totalSteps, axesText]() {
        bool cancelled = false;
        QString terminalError;
        int completed = 0;

        const auto publishRecord = [this, totalSteps](const AxisMoveDiagnosticRecord& record, int done) {
            QMetaObject::invokeMethod(this, [this, record, done, totalSteps]() {
                if (m_closeRequested) return;
                appendAxisMoveDiagnosticRecord(record);
                if (m_progressAxisMoveDiagAuto) {
                    m_progressAxisMoveDiagAuto->setValue(totalSteps > 0 ? (100 * done / totalSteps) : 0);
                }
                });
            };

        for (const int axisId : axes) {
            if (m_axisMoveDiagAutoCancel.load() || m_closeRequested) { cancelled = true; break; }
            const AxisParams motion = axisParams[axisId];
            const AxisMoveDiagnosticSnapshot axisInitial = captureAxisMoveDiagnosticSnapshot(controller, axisId);
            if (!axisInitial.valid) {
                terminalError = tr("%1轴测试开始前位置读取失败：%2")
                    .arg(axisName5D(axisId), axisInitial.error);
                break;
            }

            QMetaObject::invokeMethod(this, [this, axisId, axesText, phase, plan]() {
                if (m_closeRequested || !m_lblAxisMoveDiagStatus) return;
                m_lblAxisMoveDiagStatus->setText(tr("自动XYZ位移诊断：当前%1轴（选择=%2），%3 / %4")
                    .arg(axisName5D(axisId), axesText, phase, plan));
                });

            for (int i = 0; i < steps.size(); ++i) {
                if (m_axisMoveDiagAutoCancel.load() || m_closeRequested) { cancelled = true; break; }
                const auto step = steps[i];
                AxisMoveDiagnosticRecord record;
                record.axisId = axisId;
                record.command = step.relative ? QStringLiteral("auto_rel") : QStringLiteral("auto_abs");
                record.phase = phase;
                record.plan = plan;
                record.role = step.role;
                record.stepLabel = step.label;
                record.cycle = step.cycle;
                record.testBaseMm = step.testBaseMm;
                record.testDistanceMm = step.testDistanceMm;
                record.testDirection = step.testDirection;
                record.settleMs = settleMs;
                record.start = captureAxisMoveDiagnosticSnapshot(controller, axisId);
                if (step.relative) {
                    record.requestedDelta = step.value;
                    if (record.start.valid) record.requestedTarget = record.start.dpos + step.value;
                }
                else {
                    record.requestedTarget = step.value;
                    if (record.start.valid) record.requestedDelta = step.value - record.start.dpos;
                }

                QString err;
                bool ok = false;
                if (record.start.valid) {
                    ok = step.relative
                        ? controller->moveRelRawDpos(axisId, static_cast<float>(step.value), &motion, &err)
                        : controller->moveAbsRawDpos(axisId, static_cast<float>(step.value), &motion, &err);
                }
                else {
                    err = record.start.error.isEmpty() ? tr("起点读取失败") : record.start.error;
                }
                record.moveOk = ok;
                record.moveError = err;
                record.endIdle = captureAxisMoveDiagnosticSnapshot(controller, axisId);
                if (ok && settleMs > 0 && !m_axisMoveDiagAutoCancel.load()) {
                    QThread::msleep(static_cast<unsigned long>(settleMs));
                }
                record.endSettled = captureAxisMoveDiagnosticSnapshot(controller, axisId);
                ++completed;
                publishRecord(record, completed);

                if (!ok) {
                    if (m_axisMoveDiagAutoCancel.load()) cancelled = true;
                    else terminalError = err.isEmpty() ? tr("%1轴自动测试运动失败").arg(axisName5D(axisId)) : err;
                    break;
                }
            }
            if (cancelled || !terminalError.isEmpty()) break;

            // POST_HOME绝对计划可能把轴留在-0.5或-2mm。为了让XYZ依次测试相互独立，
            // 每个轴结束后恢复“该轴开始诊断时”的DPOS，并把恢复动作也作为一条记录保存。
            if (phase == QStringLiteral("POST_HOME")) {
                AxisMoveDiagnosticRecord restore;
                restore.axisId = axisId;
                restore.command = QStringLiteral("auto_restore");
                restore.phase = phase;
                restore.plan = plan;
                restore.role = QStringLiteral("restore");
                restore.stepLabel = QStringLiteral("restore_initial_%1").arg(axisName5D(axisId));
                restore.cycle = 0;
                restore.settleMs = settleMs;
                restore.start = captureAxisMoveDiagnosticSnapshot(controller, axisId);
                restore.requestedTarget = axisInitial.dpos;
                if (restore.start.valid) restore.requestedDelta = axisInitial.dpos - restore.start.dpos;

                QString err;
                bool ok = restore.start.valid
                    ? controller->moveAbsRawDpos(axisId, static_cast<float>(axisInitial.dpos), &motion, &err)
                    : false;
                if (!restore.start.valid && err.isEmpty()) err = restore.start.error;
                restore.moveOk = ok;
                restore.moveError = err;
                restore.endIdle = captureAxisMoveDiagnosticSnapshot(controller, axisId);
                if (ok && settleMs > 0 && !m_axisMoveDiagAutoCancel.load()) {
                    QThread::msleep(static_cast<unsigned long>(settleMs));
                }
                restore.endSettled = captureAxisMoveDiagnosticSnapshot(controller, axisId);
                ++completed;
                publishRecord(restore, completed);

                if (!ok) {
                    if (m_axisMoveDiagAutoCancel.load()) cancelled = true;
                    else terminalError = err.isEmpty() ? tr("%1轴恢复启动位置失败").arg(axisName5D(axisId)) : err;
                    break;
                }
            }
        }

        QMetaObject::invokeMethod(this, [this, cancelled, terminalError, completed, totalSteps, phase, plan, axesText]() {
            --m_axisWorkerCount;
            m_axisMoveDiagAutoRunning = false;
            updateAxisCompensationValidationUi();
            m_axisMoveDiagAutoCancel.store(false);
            if (m_btnAxisMoveDiagAutoStart) m_btnAxisMoveDiagAutoStart->setEnabled(true);
            if (m_btnAxisMoveDiagAutoStop) m_btnAxisMoveDiagAutoStop->setEnabled(false);
            if (m_btnAxisMoveDiagManualStart) m_btnAxisMoveDiagManualStart->setEnabled(true);
            if (m_btnAxisMoveDiagManualEnd) m_btnAxisMoveDiagManualEnd->setEnabled(true);
            if (m_btnAxisMoveDiagClear) m_btnAxisMoveDiagClear->setEnabled(true);
            if (m_comboAxisMoveDiagAutoAxis) m_comboAxisMoveDiagAutoAxis->setEnabled(true);
            if (m_comboAxisMoveDiagPhase) m_comboAxisMoveDiagPhase->setEnabled(true);
            if (m_spinAxisMoveDiagRepeats) m_spinAxisMoveDiagRepeats->setEnabled(true);
            if (m_btnHomeAll) m_btnHomeAll->setEnabled(true);
            if (m_btnZeroAll) m_btnZeroAll->setEnabled(true);
            for (int axis = AxisX; axis <= AxisB; ++axis) {
                if (auto* w = axisWidgetForId(axis)) w->setControlsEnabled(true);
            }
            updateAutomaticAxisMoveDiagnosticUi();
            if (m_closeRequested) { tryFinishDeferredClose(); return; }
            if (cancelled) {
                if (m_lblAxisMoveDiagStatus) m_lblAxisMoveDiagStatus->setText(tr("自动测试已取消；轴=%1；已完成%2/%3步。数据仍可保存。")
                    .arg(axesText).arg(completed).arg(totalSteps));
            }
            else if (!terminalError.isEmpty()) {
                if (m_lblAxisMoveDiagStatus) m_lblAxisMoveDiagStatus->setText(tr("自动测试中止：%1；已完成%2/%3步。")
                    .arg(terminalError).arg(completed).arg(totalSteps));
                QMessageBox::warning(this, tr("自动位移诊断中止"), terminalError);
            }
            else {
                if (m_progressAxisMoveDiagAuto) m_progressAxisMoveDiagAuto->setValue(100);
                if (m_lblAxisMoveDiagStatus) m_lblAxisMoveDiagStatus->setText(
                    tr("自动测试完成：轴=%1；%2 / %3，共%4步。请保存CSV后发送给我分析。")
                    .arg(axesText, phase, plan).arg(completed));
            }
            tryFinishDeferredClose();
            });
        });
}

void MainWindow::stopAutomaticAxisMoveDiagnostic()
{
    if (!m_axisMoveDiagAutoRunning) return;
    m_axisMoveDiagAutoCancel.store(true);
    if (m_btnAxisMoveDiagAutoStop) m_btnAxisMoveDiagAutoStop->setEnabled(false);
    // XYZ依次模式下当前正在运动的轴可能是任意一个；逐轴发减速停止最安全，也避免维护跨线程“当前轴”状态。
    if (m_zmc && m_zmc->isConnected()) {
        for (int axis = AxisX; axis <= AxisZ; ++axis) m_zmc->requestStopAxis(axis, false);
    }
    if (m_lblAxisMoveDiagStatus) m_lblAxisMoveDiagStatus->setText(tr("正在停止自动XYZ位移诊断..."));
}

void MainWindow::clearAxisMoveDiagnostics()
{
    if (m_axisMoveDiagAutoRunning) {
        QMessageBox::information(this, tr("位移诊断"), tr("自动测试运行中，不能清空；请先停止自动测试。"));
        return;
    }
    m_axisMoveDiagRecords.clear();
    m_axisMoveDiagManualStart = AxisMoveDiagnosticSnapshot{};
    m_axisMoveDiagManualAxis = -1;
    if (m_axisMoveDiagTable) m_axisMoveDiagTable->setRowCount(0);
    if (m_lblAxisMoveDiagStatus) {
        m_lblAxisMoveDiagStatus->setText(
            tr("诊断记录已清空。Δ误差 = ΔDPOS - ΔMPOS；FE = DPOS - MPOS。"));
    }
}

void MainWindow::saveAxisMoveDiagnosticsCsv()
{
    if (m_axisMoveDiagRecords.isEmpty()) {
        QMessageBox::information(this, tr("保存位移诊断"), tr("当前没有诊断记录"));
        return;
    }

    const QString suggested = QDir::home().filePath(
        QStringLiteral("axis_dpos_mpos_motion_diagnostics_%1.csv")
        .arg(QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss"))));
    const QString fileName = QFileDialog::getSaveFileName(
        this, tr("保存DPOS/MPOS位移诊断CSV"), suggested, tr("CSV文件 (*.csv)"));
    if (fileName.isEmpty()) return;

    QFile file(fileName);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate)) {
        QMessageBox::warning(this, tr("保存失败"), file.errorString());
        return;
    }
    QTextStream out(&file);
    out.setEncoding(QStringConverter::Utf8);
    const auto csv = [](QString value) {
        value.replace('"', QStringLiteral("\"\""));
        return QStringLiteral("\"") + value + QStringLiteral("\"");
        };
    const auto num = [](double v) -> QString {
        return std::isfinite(v) ? QString::number(v, 'f', 9) : QString();
        };
    const auto axisName = [](int axisId) -> QString {
        switch (axisId) {
        case AxisX: return QStringLiteral("X"); case AxisY: return QStringLiteral("Y");
        case AxisZ: return QStringLiteral("Z"); case AxisC: return QStringLiteral("C"); case AxisB: return QStringLiteral("B");
        default: return QStringLiteral("?");
        }
        };
    const auto fe = [](const AxisMoveDiagnosticSnapshot& s) -> double {
        return s.valid ? s.dpos - s.mpos : std::numeric_limits<double>::quiet_NaN();
        };

    out << "record,phase,plan,role,step_label,cycle,axis,command,test_base_mm,test_distance_mm,test_direction,requested_target_mm,requested_delta_mm,settle_ms,"
        "start_utc,start_dpos_mm,start_mpos_mm,start_encoder,start_fe_mm,start_idle,start_linear_referenced,start_setmpos_generation,start_datum_input,start_datum_raw,"
        "idle_utc,idle_dpos_mm,idle_mpos_mm,idle_encoder,idle_fe_mm,idle_flag,idle_linear_referenced,idle_setmpos_generation,idle_datum_raw,"
        "settled_utc,settled_dpos_mm,settled_mpos_mm,settled_encoder,settled_fe_mm,settled_idle,settled_linear_referenced,settled_setmpos_generation,settled_datum_raw,"
        "delta_dpos_mm,delta_mpos_mm,delta_encoder,delta_dpos_minus_delta_mpos_mm,error_ppm,fe_change_mm,"
        "post_idle_dpos_drift_mm,post_idle_mpos_drift_mm,crosses_dpos_zero,setmpos_generation_changed,move_ok,status\n";

    for (int i = 0; i < m_axisMoveDiagRecords.size(); ++i) {
        const auto& r = m_axisMoveDiagRecords[i];
        const double deltaD = r.start.valid && r.endSettled.valid ? r.endSettled.dpos - r.start.dpos : std::numeric_limits<double>::quiet_NaN();
        const double deltaM = r.start.valid && r.endSettled.valid ? r.endSettled.mpos - r.start.mpos : std::numeric_limits<double>::quiet_NaN();
        const double deltaErr = std::isfinite(deltaD) && std::isfinite(deltaM) ? deltaD - deltaM : std::numeric_limits<double>::quiet_NaN();
        const double errorPpm = std::isfinite(deltaErr) && std::isfinite(deltaD) && std::abs(deltaD) > 1.0e-12
            ? deltaErr / deltaD * 1.0e6
            : std::numeric_limits<double>::quiet_NaN();
        const double feStart = fe(r.start), feIdle = fe(r.endIdle), feEnd = fe(r.endSettled);
        const double feChange = std::isfinite(feStart) && std::isfinite(feEnd) ? feEnd - feStart : std::numeric_limits<double>::quiet_NaN();
        const double dDrift = r.endIdle.valid && r.endSettled.valid ? r.endSettled.dpos - r.endIdle.dpos : std::numeric_limits<double>::quiet_NaN();
        const double mDrift = r.endIdle.valid && r.endSettled.valid ? r.endSettled.mpos - r.endIdle.mpos : std::numeric_limits<double>::quiet_NaN();
        const double deltaEncoder = r.start.encoderValid && r.endSettled.encoderValid
            ? r.endSettled.encoder - r.start.encoder : std::numeric_limits<double>::quiet_NaN();
        const bool crossesZero = r.start.valid && r.endSettled.valid
            && ((r.start.dpos < 0.0 && r.endSettled.dpos >= 0.0) || (r.start.dpos > 0.0 && r.endSettled.dpos <= 0.0));
        const bool generationChanged = r.start.mposSetGeneration != r.endSettled.mposSetGeneration;
        QString status = r.moveError;
        if (status.isEmpty()) {
            QStringList notes;
            if (!r.start.error.isEmpty()) notes << r.start.error;
            if (!r.endIdle.error.isEmpty()) notes << r.endIdle.error;
            if (!r.endSettled.error.isEmpty()) notes << r.endSettled.error;
            status = notes.join(QStringLiteral("; "));
        }

        out << (i + 1) << ',' << csv(r.phase) << ',' << csv(r.plan) << ',' << csv(r.role) << ',' << csv(r.stepLabel) << ',' << r.cycle << ','
            << csv(axisName(r.axisId)) << ',' << csv(r.command) << ','
            << num(r.testBaseMm) << ',' << num(r.testDistanceMm) << ','
            << csv(r.testDirection > 0 ? QStringLiteral("+") : (r.testDirection < 0 ? QStringLiteral("-") : QString())) << ','
            << num(r.requestedTarget) << ',' << num(r.requestedDelta) << ',' << r.settleMs << ','
            << csv(r.start.utc) << ',' << num(r.start.dpos) << ',' << num(r.start.mpos) << ',' << (r.start.encoderValid ? num(r.start.encoder) : QString()) << ',' << num(feStart) << ',' << (r.start.idle ? 1 : 0) << ',' << (r.start.linearReferenced ? 1 : 0) << ',' << static_cast<qulonglong>(r.start.mposSetGeneration) << ',' << r.start.datumInput << ',' << r.start.datumRaw << ','
            << csv(r.endIdle.utc) << ',' << num(r.endIdle.dpos) << ',' << num(r.endIdle.mpos) << ',' << (r.endIdle.encoderValid ? num(r.endIdle.encoder) : QString()) << ',' << num(feIdle) << ',' << (r.endIdle.idle ? 1 : 0) << ',' << (r.endIdle.linearReferenced ? 1 : 0) << ',' << static_cast<qulonglong>(r.endIdle.mposSetGeneration) << ',' << r.endIdle.datumRaw << ','
            << csv(r.endSettled.utc) << ',' << num(r.endSettled.dpos) << ',' << num(r.endSettled.mpos) << ',' << (r.endSettled.encoderValid ? num(r.endSettled.encoder) : QString()) << ',' << num(feEnd) << ',' << (r.endSettled.idle ? 1 : 0) << ',' << (r.endSettled.linearReferenced ? 1 : 0) << ',' << static_cast<qulonglong>(r.endSettled.mposSetGeneration) << ',' << r.endSettled.datumRaw << ','
            << num(deltaD) << ',' << num(deltaM) << ',' << num(deltaEncoder) << ',' << num(deltaErr) << ',' << num(errorPpm) << ',' << num(feChange) << ','
            << num(dDrift) << ',' << num(mDrift) << ',' << (crossesZero ? 1 : 0) << ',' << (generationChanged ? 1 : 0) << ','
            << (r.moveOk ? 1 : 0) << ',' << csv(status) << '\n';
    }
    file.close();
    statusBar()->showMessage(tr("DPOS/MPOS位移诊断已保存: %1").arg(fileName), 5000);
}

void MainWindow::updateAxisCompensationSpeedProfileTable()
{
    if (!m_axisCompSpeedTable) return;
    for (int row = 0; row < 3; ++row) {
        const int axis = AxisX + row;
        const auto& p = m_axisCompSpeedProfiles[axis];
        const auto setText = [&](int col, const QString& text) {
            auto* item = m_axisCompSpeedTable->item(row, col);
            if (!item) {
                item = new QTableWidgetItem;
                item->setFlags(item->flags() & ~Qt::ItemIsEditable);
                item->setTextAlignment(Qt::AlignCenter);
                m_axisCompSpeedTable->setItem(row, col, item);
            }
            item->setText(text);
            };
        if (p.qualified) {
            setText(3, QString::number(p.coarseSpeedMmS, 'f', 3));
            setText(4, QString::number(p.precisionSpeedMmS, 'f', 3));
            setText(5, QString::number(p.coarseP95DriftMm * 1000.0, 'f', 2));
            setText(6, QString::number(p.coarseRepeatabilityMm * 1000.0, 'f', 2));
            setText(7, tr("已通过"));
        }
        else {
            setText(3, QStringLiteral("—"));
            setText(4, QStringLiteral("—"));
            setText(5, QStringLiteral("—"));
            setText(6, QStringLiteral("—"));
            setText(7, p.status.isEmpty() ? tr("未测速") : p.status);
        }
    }
}

void MainWindow::updateAxisCompensationSpeedQualificationUi()
{
    if (!m_comboAxisCompCalAxis) return;
    const int selection = m_comboAxisCompCalAxis->currentData().toInt();
    const bool idle = !m_axisCompSpeedRunning && !m_axisCompCalRunning && !m_axisCompValRunning;
    if (m_btnAxisCompSpeedStart) m_btnAxisCompSpeedStart->setEnabled(idle);
    if (m_btnAxisCompSpeedStop) m_btnAxisCompSpeedStop->setEnabled(m_axisCompSpeedRunning);
    if (m_editAxisCompSpeedPrecisionCandidates) m_editAxisCompSpeedPrecisionCandidates->setEnabled(idle);
    if (m_spinAxisCompSpeedTestPoints) m_spinAxisCompSpeedTestPoints->setEnabled(idle);
    if (m_spinAxisCompSpeedRepeats) m_spinAxisCompSpeedRepeats->setEnabled(idle);
    if (m_spinAxisCompSpeedP95DriftUm) m_spinAxisCompSpeedP95DriftUm->setEnabled(idle);
    if (m_spinAxisCompSpeedMaxDriftUm) m_spinAxisCompSpeedMaxDriftUm->setEnabled(idle);
    if (m_spinAxisCompSpeedRepeatabilityUm) m_spinAxisCompSpeedRepeatabilityUm->setEnabled(idle);
    if (m_axisCompSpeedTable) {
        for (int row = 0; row < 3; ++row) {
            const int axis = AxisX + row;
            const bool rowEnabled = idle && (selection < 0 || selection == axis);
            if (auto* maxSpeed = qobject_cast<QDoubleSpinBox*>(m_axisCompSpeedTable->cellWidget(row, 1))) maxSpeed->setEnabled(rowEnabled);
            if (auto* focus = qobject_cast<QLineEdit*>(m_axisCompSpeedTable->cellWidget(row, 2))) focus->setEnabled(rowEnabled);
        }
    }
    updateAxisCompensationSpeedProfileTable();
}

bool MainWindow::saveAxisCompensationSpeedQualificationFiles(const QString& outputRoot,
    QString* savedRunDirectory,
    QString* error) const
{
    if (m_axisCompSpeedSamples.isEmpty()) {
        if (error) *error = tr("当前没有采表速度资格测试数据");
        return false;
    }
    const QString root = outputRoot.trimmed().isEmpty()
        ? QDir(QCoreApplication::applicationDirPath()).filePath(QStringLiteral("calibration/axis_compensation"))
        : outputRoot.trimmed();
    QDir rootDir(root);
    if (!rootDir.exists() && !rootDir.mkpath(QStringLiteral("."))) {
        if (error) *error = tr("无法创建测速保存目录：%1").arg(root);
        return false;
    }
    const QString runName = QStringLiteral("speed_qualification_%1")
        .arg(QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss_zzz")));
    const QString runPath = rootDir.filePath(runName);
    if (!QDir().mkpath(runPath)) {
        if (error) *error = tr("无法创建本次测速目录：%1").arg(runPath);
        return false;
    }
    QDir runDir(runPath);
    const auto csvQuote = [](QString value) {
        value.replace('"', QStringLiteral("\"\""));
        return QStringLiteral("\"") + value + QStringLiteral("\"");
        };
    const auto num = [](double v) {
        return std::isfinite(v) ? QString::number(v, 'f', 9) : QString();
        };

    QFile rawFile(runDir.filePath(QStringLiteral("axis_compensation_speed_qualification_raw.csv")));
    if (!rawFile.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate)) {
        if (error) *error = tr("无法写入测速原始CSV：%1").arg(rawFile.errorString());
        return false;
    }
    QTextStream raw(&rawFile);
    raw.setEncoding(QStringConverter::Utf8);
    raw << "record,axis,stage,candidate_speed_mm_s,coarse_speed_mm_s,precision_speed_mm_s,direction,cycle,target_index,target_dpos_mm,preload_dpos_mm,settle_ms,"
        "idle_utc,idle_dpos_mm,idle_mpos_mm,idle_encoder,settled_utc,settled_dpos_mm,settled_mpos_mm,settled_encoder,post_idle_mpos_drift_mm,move_ok,status\n";
    int index = 0;
    for (const auto& s : m_axisCompSpeedSamples) {
        const double drift = s.endIdle.valid && s.endSettled.valid
            ? s.endSettled.mpos - s.endIdle.mpos : std::numeric_limits<double>::quiet_NaN();
        raw << ++index << ',' << csvQuote(axisName5D(s.axisId)) << ',' << csvQuote(s.stage) << ','
            << num(s.candidateSpeedMmS) << ',' << num(s.coarseSpeedMmS) << ',' << num(s.precisionSpeedMmS) << ','
            << csvQuote(s.direction > 0 ? QStringLiteral("+") : QStringLiteral("-")) << ','
            << s.cycle << ',' << s.targetIndex << ',' << num(s.targetDposMm) << ',' << num(s.preloadDposMm) << ',' << s.settleMs << ','
            << csvQuote(s.endIdle.utc) << ',' << num(s.endIdle.dpos) << ',' << num(s.endIdle.mpos) << ','
            << (s.endIdle.encoderValid ? num(s.endIdle.encoder) : QString()) << ','
            << csvQuote(s.endSettled.utc) << ',' << num(s.endSettled.dpos) << ',' << num(s.endSettled.mpos) << ','
            << (s.endSettled.encoderValid ? num(s.endSettled.encoder) : QString()) << ','
            << num(drift) << ',' << (s.moveOk ? 1 : 0) << ',' << csvQuote(s.error) << '\n';
    }
    rawFile.close();

    QFile summaryFile(runDir.filePath(QStringLiteral("axis_compensation_speed_profiles.csv")));
    if (!summaryFile.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate)) {
        if (error) *error = tr("无法写入测速汇总CSV：%1").arg(summaryFile.errorString());
        return false;
    }
    QTextStream summary(&summaryFile);
    summary.setEncoding(QStringConverter::Utf8);
    summary << "axis,qualified,coarse_speed_mm_s,precision_speed_mm_s,precision_p95_drift_mm,precision_max_drift_mm,precision_repeatability_rms_mm,"
        "coarse_p95_drift_mm,coarse_max_drift_mm,coarse_repeatability_rms_mm,range_start_mm,range_end_mm,preload_mm,status\n";
    QJsonObject axesJson;
    for (int axis = AxisX; axis <= AxisZ; ++axis) {
        const auto& p = m_axisCompSpeedProfiles[axis];
        if (p.axisId != axis && !p.qualified && p.status.isEmpty()) continue;
        summary << csvQuote(axisName5D(axis)) << ',' << (p.qualified ? 1 : 0) << ','
            << num(p.coarseSpeedMmS) << ',' << num(p.precisionSpeedMmS) << ','
            << num(p.precisionP95DriftMm) << ',' << num(p.precisionMaxDriftMm) << ',' << num(p.precisionRepeatabilityMm) << ','
            << num(p.coarseP95DriftMm) << ',' << num(p.coarseMaxDriftMm) << ',' << num(p.coarseRepeatabilityMm) << ','
            << num(p.rangeStartMm) << ',' << num(p.rangeEndMm) << ',' << num(p.preloadMm) << ',' << csvQuote(p.status) << '\n';
        QJsonObject obj;
        obj[QStringLiteral("qualified")] = p.qualified;
        obj[QStringLiteral("coarse_speed_mm_s")] = p.coarseSpeedMmS;
        obj[QStringLiteral("precision_speed_mm_s")] = p.precisionSpeedMmS;
        obj[QStringLiteral("precision_p95_drift_mm")] = p.precisionP95DriftMm;
        obj[QStringLiteral("precision_max_drift_mm")] = p.precisionMaxDriftMm;
        obj[QStringLiteral("precision_repeatability_rms_mm")] = p.precisionRepeatabilityMm;
        obj[QStringLiteral("coarse_p95_drift_mm")] = p.coarseP95DriftMm;
        obj[QStringLiteral("coarse_max_drift_mm")] = p.coarseMaxDriftMm;
        obj[QStringLiteral("coarse_repeatability_rms_mm")] = p.coarseRepeatabilityMm;
        obj[QStringLiteral("range_start_mm")] = p.rangeStartMm;
        obj[QStringLiteral("range_end_mm")] = p.rangeEndMm;
        obj[QStringLiteral("preload_mm")] = p.preloadMm;
        obj[QStringLiteral("status")] = p.status;
        axesJson[axisName5D(axis)] = obj;
    }
    summaryFile.close();

    QJsonObject manifest;
    manifest[QStringLiteral("format")] = QStringLiteral("MotionLaserSync.AxisCompensationSpeedQualification");
    manifest[QStringLiteral("version")] = 1;
    manifest[QStringLiteral("software_release")] = QStringLiteral("V27.6.35");
    manifest[QStringLiteral("created_utc")] = QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs);
    manifest[QStringLiteral("selection_rule")] = QStringLiteral("local qualification: precision candidates high-to-low with early stop at first pass; coarse speed starts at per-axis upper limit and halves only on failure");
    manifest[QStringLiteral("test_position_strategy")] = QStringLiteral("user focus positions only when provided; otherwise 1-3 interior/current representative positions; never implicit calibration-range endpoints");
    manifest[QStringLiteral("p95_drift_limit_um")] = m_spinAxisCompSpeedP95DriftUm ? m_spinAxisCompSpeedP95DriftUm->value() : 0.0;
    manifest[QStringLiteral("max_drift_limit_um")] = m_spinAxisCompSpeedMaxDriftUm ? m_spinAxisCompSpeedMaxDriftUm->value() : 0.0;
    manifest[QStringLiteral("repeatability_rms_limit_um")] = m_spinAxisCompSpeedRepeatabilityUm ? m_spinAxisCompSpeedRepeatabilityUm->value() : 0.0;
    manifest[QStringLiteral("axes")] = axesJson;
    QFile manifestFile(runDir.filePath(QStringLiteral("axis_compensation_speed_qualification.json")));
    if (!manifestFile.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        if (error) *error = tr("无法写入测速manifest：%1").arg(manifestFile.errorString());
        return false;
    }
    manifestFile.write(QJsonDocument(manifest).toJson(QJsonDocument::Indented));
    manifestFile.close();
    if (savedRunDirectory) *savedRunDirectory = runPath;
    return true;
}

void MainWindow::startAxisCompensationSpeedQualification()
{
    if (m_zmc && m_zmc->axisCompensationEnabled()) {
        QMessageBox::warning(this, tr("RAW 采集/诊断/验证"),
            tr("请先在 XYZ 运行补偿面板禁用运行补偿，再启动 RAW 采集或独立 A/B 验证。"));
        return;
    }
    if (m_axisCompSpeedRunning) return;
    if (m_axisCompCalRunning || m_axisCompValRunning || m_axisMoveDiagAutoRunning) {
        QMessageBox::information(this, tr("采表速度资格测试"), tr("补偿表采集/A-B验证/位移诊断仍在运行，请先停止。"));
        return;
    }
    if (!m_zmc || !m_zmc->isConnected()) {
        QMessageBox::warning(this, tr("采表速度资格测试"), tr("请先连接控制器。"));
        return;
    }
    if ((m_gcodeExecutor && (m_gcodeExecutor->isRunning() || m_gcodeExecutor->isPaused()))
        || (m_syncManager && m_syncManager->isRunning()) || m_jointMoveInProgress || m_sphereStudyAutoActive) {
        QMessageBox::information(this, tr("采表速度资格测试"), tr("当前有G代码/同步采集/五轴定位/自动球研究任务运行，请先完全停止。"));
        return;
    }
    for (int axis = AxisX; axis <= AxisB; ++axis) {
        if (m_zmc->isAxisBusy(axis)) {
            QMessageBox::information(this, tr("采表速度资格测试"), tr("当前仍有轴运动任务，请全部停稳后再开始。"));
            return;
        }
    }

    const int selection = m_comboAxisCompCalAxis ? m_comboAxisCompCalAxis->currentData().toInt() : AxisZ;
    QVector<int> axes;
    if (selection < 0) axes = { AxisX, AxisY, AxisZ };
    else if (selection >= AxisX && selection <= AxisZ) axes = { selection };
    else return;

    QStringList notReferenced;
    for (const int axis : axes) if (!m_zmc->isLinearMposReferenced(axis)) notReferenced << axisName5D(axis);
    if (!notReferenced.isEmpty()) {
        QMessageBox::warning(this, tr("采表速度资格测试"),
            tr("测速必须在正规回零并建立MPOS机床零点后进行。尚未回零：%1").arg(notReferenced.join(QStringLiteral(", "))));
        return;
    }

    QVector<double> precisionInput;
    QString parseError;
    if (!m_editAxisCompSpeedPrecisionCandidates
        || !parseDiagnosticDoubleList(m_editAxisCompSpeedPrecisionCandidates->text(), precisionInput, &parseError, true)) {
        QMessageBox::warning(this, tr("采表速度资格测试"), tr("精密速度候选无效：%1").arg(parseError));
        return;
    }

    struct SpeedAxisConfig {
        int axisId = -1;
        double start = 0.0;
        double end = 0.0;
        double initialDpos = 0.0;
        QVector<double> targets;
        QVector<double> precisionCandidates;
        double configuredMaxSpeed = 0.0;
    };

    QVector<SpeedAxisConfig> configs;
    const auto axisParams = currentAxisParams5D();
    const double preload = m_spinAxisCompCalPreloadMm ? m_spinAxisCompCalPreloadMm->value() : 2.0;
    const int representativeCount = m_spinAxisCompSpeedTestPoints ? m_spinAxisCompSpeedTestPoints->value() : 1;
    const int repeats = m_spinAxisCompSpeedRepeats ? m_spinAxisCompSpeedRepeats->value() : 2;
    const int settleMs = m_spinAxisCompCalSettleMs ? m_spinAxisCompCalSettleMs->value() : 2000;
    const double p95LimitMm = (m_spinAxisCompSpeedP95DriftUm ? m_spinAxisCompSpeedP95DriftUm->value() : 2.0) / 1000.0;
    const double maxLimitMm = (m_spinAxisCompSpeedMaxDriftUm ? m_spinAxisCompSpeedMaxDriftUm->value() : 5.0) / 1000.0;
    const double repeatLimitMm = (m_spinAxisCompSpeedRepeatabilityUm ? m_spinAxisCompSpeedRepeatabilityUm->value() : 2.0) / 1000.0;
    int estimatedSamples = 0;
    QStringList configDescriptions;

    for (const int axis : axes) {
        const int row = axis - AxisX;
        auto* startSpin = qobject_cast<QDoubleSpinBox*>(m_axisCompCalRangeTable->cellWidget(row, 1));
        auto* endSpin = qobject_cast<QDoubleSpinBox*>(m_axisCompCalRangeTable->cellWidget(row, 2));
        if (!startSpin || !endSpin) {
            QMessageBox::warning(this, tr("采表速度资格测试"), tr("%1轴范围控件无效。").arg(axisName5D(axis)));
            return;
        }

        const AxisMoveDiagnosticSnapshot initial = captureAxisMoveDiagnosticSnapshot(m_zmc.get(), axis);
        if (!initial.valid) {
            QMessageBox::warning(this, tr("采表速度资格测试"), tr("%1轴当前坐标读取失败：%2").arg(axisName5D(axis), initial.error));
            return;
        }

        SpeedAxisConfig cfg;
        cfg.axisId = axis;
        cfg.start = startSpin->value();
        cfg.end = endSpin->value();
        cfg.initialDpos = initial.dpos;
        auto* maxSpeedSpin = m_axisCompSpeedTable ? qobject_cast<QDoubleSpinBox*>(m_axisCompSpeedTable->cellWidget(row, 1)) : nullptr;
        cfg.configuredMaxSpeed = maxSpeedSpin ? maxSpeedSpin->value() : static_cast<double>(axisParams[axis].speed);
        if (!(cfg.start < cfg.end) || !(cfg.configuredMaxSpeed > 0.0)) {
            QMessageBox::warning(this, tr("采表速度资格测试"), tr("%1轴范围或测速上限无效。").arg(axisName5D(axis)));
            return;
        }

        const double softLo = static_cast<double>(axisParams[axis].softLimitNeg);
        const double softHi = static_cast<double>(axisParams[axis].softLimitPos);
        if (cfg.start - preload < softLo - 1e-9 || cfg.end + preload > softHi + 1e-9) {
            QMessageBox::warning(this, tr("采表速度资格测试"), tr("%1轴测速范围加预加载后超出软限位。").arg(axisName5D(axis)));
            return;
        }

        bool haveExplicitFocus = false;
        if (m_axisCompSpeedTable) {
            if (auto* focus = qobject_cast<QLineEdit*>(m_axisCompSpeedTable->cellWidget(row, 2))) {
                const QString focusText = focus->text().trimmed();
                if (!focusText.isEmpty()) {
                    QVector<double> extras;
                    QString focusError;
                    if (!parseDiagnosticDoubleList(focusText, extras, &focusError, false)) {
                        QMessageBox::warning(this, tr("采表速度资格测试"), tr("%1轴重点位置无效：%2").arg(axisName5D(axis), focusError));
                        return;
                    }
                    for (const double x : extras) {
                        if (x < cfg.start - 1e-9 || x > cfg.end + 1e-9) {
                            QMessageBox::warning(this, tr("采表速度资格测试"), tr("%1轴重点位置%2 mm不在采表范围[%3,%4]内。")
                                .arg(axisName5D(axis)).arg(x, 0, 'f', 3).arg(cfg.start, 0, 'f', 3).arg(cfg.end, 0, 'f', 3));
                            return;
                        }
                        bool duplicate = false;
                        for (const double existing : cfg.targets) if (std::abs(existing - x) < 1e-6) { duplicate = true; break; }
                        if (!duplicate) cfg.targets.push_back(x);
                    }
                    haveExplicitFocus = !cfg.targets.isEmpty();
                }
            }
        }

        // V27.6.33: speed qualification is local. Never use the calibration-range endpoints as implicit test points.
        // If the user provided known resonance/focus positions, use only those. Otherwise use 1~3 interior representatives.
        if (!haveExplicitFocus) {
            if (representativeCount <= 1) {
                const double midpoint = 0.5 * (cfg.start + cfg.end);
                const double current = cfg.initialDpos;
                cfg.targets.push_back((current >= cfg.start && current <= cfg.end) ? current : midpoint);
            }
            else {
                for (int i = 0; i < representativeCount; ++i) {
                    const double t = static_cast<double>(i + 1) / static_cast<double>(representativeCount + 1);
                    cfg.targets.push_back(cfg.start + t * (cfg.end - cfg.start));
                }
            }
        }

        // Visit the closest representative first instead of always driving to the most-negative end.
        std::sort(cfg.targets.begin(), cfg.targets.end(), [cur = cfg.initialDpos](double a, double b) {
            return std::abs(a - cur) < std::abs(b - cur);
            });

        for (const double speed : precisionInput) {
            if (speed > 0.0 && speed <= cfg.configuredMaxSpeed + 1e-9) cfg.precisionCandidates.push_back(speed);
        }
        std::sort(cfg.precisionCandidates.begin(), cfg.precisionCandidates.end(), std::greater<double>());
        cfg.precisionCandidates.erase(std::unique(cfg.precisionCandidates.begin(), cfg.precisionCandidates.end(), [](double a, double b) {
            return std::abs(a - b) < 1e-6;
            }), cfg.precisionCandidates.end());
        if (cfg.precisionCandidates.isEmpty()) {
            QMessageBox::warning(this, tr("采表速度资格测试"),
                tr("%1轴没有不超过测速上限%2 mm/s的精密速度候选。")
                .arg(axisName5D(axis)).arg(cfg.configuredMaxSpeed, 0, 'f', 3));
            return;
        }

        const int perCandidate = static_cast<int>(cfg.targets.size()) * repeats * 2;
        // Upper-bound estimate only. Precision stage stops at the first qualified speed; coarse stage normally uses 1~3 trials.
        estimatedSamples += perCandidate * (static_cast<int>(cfg.precisionCandidates.size()) + 4);
        QStringList targetText;
        for (const double t : cfg.targets) targetText << QString::number(t, 'f', 3);
        configDescriptions << tr("%1: 局部测速点={%2} mm，测速上限=%3 mm/s")
            .arg(axisName5D(axis), targetText.join(QStringLiteral(", "))).arg(cfg.configuredMaxSpeed, 0, 'f', 3);
        configs.push_back(cfg);
    }

    const QString confirm = tr(
        "将开始局部采表速度资格测试：\n%1\n\n"
        "• 不再从采表最负端开始，也不再为了测速横跨整个采表范围；\n"
        "• 若填写“重点位置”，只在这些位置测速；否则只使用当前位置/内部代表点；\n"
        "• 精密速度从高到低测试，第一个通过门限的候选立即采用；\n"
        "• 自动定位速度先测试各轴测速上限，若失败才逐次减半，直到通过或降到精密速度；\n"
        "• 每个目标都只在目标附近±%2 mm做最终方向建立，正反方向都测试，每候选重复%3次，稳定等待%4 ms。\n"
        "稳定门限：P95(IDLE后MPOS漂移)≤%5 µm，Max漂移≤%6 µm，重复性σRMS≤%7 µm。\n"
        "测速不使用位置补偿，也不会调用SetMpos。\n\n"
        "注意：绝对DPOS运动不会把DPOS-MPOS误差数值累加；但长距离往返会改变机械振动/摩擦/热状态，因此本版刻意取消这种无关长行程。")
        .arg(configDescriptions.join(QStringLiteral("\n")))
        .arg(preload, 0, 'f', 3).arg(repeats).arg(settleMs)
        .arg(p95LimitMm * 1000.0, 0, 'f', 2).arg(maxLimitMm * 1000.0, 0, 'f', 2).arg(repeatLimitMm * 1000.0, 0, 'f', 2);
    if (QMessageBox::warning(this, tr("确认采表速度资格测试"), confirm,
        QMessageBox::Yes | QMessageBox::No, QMessageBox::No) != QMessageBox::Yes) return;

    for (const int axis : axes) {
        m_axisCompSpeedProfiles[axis] = AxisCompensationSpeedProfile{};
        m_axisCompSpeedProfiles[axis].axisId = axis;
        m_axisCompSpeedProfiles[axis].status = tr("测速中");
    }
    m_axisCompSpeedSamples.clear();
    m_axisCompSpeedLastRunDir.clear();
    m_axisCompSpeedRunning = true;
    m_axisCompSpeedCancel.store(false);
    if (m_progressAxisCompSpeed) m_progressAxisCompSpeed->setValue(0);
    if (m_lblAxisCompSpeedStatus) m_lblAxisCompSpeedStatus->setText(tr("局部采表速度资格测试运行中..."));
    updateAxisCompensationSpeedQualificationUi();
    updateAxisCompensationCalibrationUi();
    updateAxisCompensationValidationUi();
    if (m_btnHomeAll) m_btnHomeAll->setEnabled(false);
    if (m_btnZeroAll) m_btnZeroAll->setEnabled(false);
    if (m_gcodeWidget) m_gcodeWidget->setEnabled(false);
    if (m_calibrationWidget) m_calibrationWidget->setEnabled(false);
    for (int axis = AxisX; axis <= AxisB; ++axis) if (auto* w = axisWidgetForId(axis)) w->setControlsEnabled(false);

    ++m_axisWorkerCount;
    ZmcController* controller = m_zmc.get();
    const QString outputRoot = m_editAxisCompCalOutputDir ? m_editAxisCompCalOutputDir->text() : QString();
    (void)QtConcurrent::run([this, controller, configs, axisParams, preload, repeats, settleMs,
        p95LimitMm, maxLimitMm, repeatLimitMm, estimatedSamples, outputRoot]() {
            struct CandidateStats {
                int sampleCount = 0;
                double p95DriftMm = std::numeric_limits<double>::quiet_NaN();
                double maxDriftMm = std::numeric_limits<double>::quiet_NaN();
                double repeatabilityMm = std::numeric_limits<double>::quiet_NaN();
                bool qualified = false;
            };
            bool cancelled = false;
            QString terminalError;
            int completed = 0;
            QVector<AxisCompensationSpeedTestSample> allSamples;
            auto profiles = m_axisCompSpeedProfiles;

            const auto makeMotion = [](const AxisParams& base, double speed) {
                AxisParams motion = base;
                motion.speed = static_cast<float>(std::max(0.001, speed));
                motion.lspeed = std::min(motion.lspeed, motion.speed);
                return motion;
                };
            const auto percentile95 = [](QVector<double> values) -> double {
                if (values.isEmpty()) return std::numeric_limits<double>::quiet_NaN();
                std::sort(values.begin(), values.end());
                const double pos = 0.95 * static_cast<double>(values.size() - 1);
                const int lo = static_cast<int>(std::floor(pos));
                const int hi = static_cast<int>(std::ceil(pos));
                if (lo == hi) return values[lo];
                const double t = pos - static_cast<double>(lo);
                return values[lo] * (1.0 - t) + values[hi] * t;
                };
            const auto computeStats = [&](const QVector<AxisCompensationSpeedTestSample>& samples) -> CandidateStats {
                CandidateStats st;
                QVector<double> absDrifts;
                for (const auto& s : samples) {
                    if (!s.moveOk || !s.endIdle.valid || !s.endSettled.valid) continue;
                    absDrifts.push_back(std::abs(s.endSettled.mpos - s.endIdle.mpos));
                }
                st.sampleCount = static_cast<int>(absDrifts.size());
                if (absDrifts.isEmpty()) return st;
                st.p95DriftMm = percentile95(absDrifts);
                st.maxDriftMm = *std::max_element(absDrifts.begin(), absDrifts.end());
                QVector<double> groupStd;
                for (const int direction : {+1, -1}) {
                    int maxTarget = -1;
                    for (const auto& s : samples) if (s.direction == direction) maxTarget = std::max(maxTarget, s.targetIndex);
                    for (int targetIndex = 0; targetIndex <= maxTarget; ++targetIndex) {
                        QVector<double> errors;
                        for (const auto& s : samples) {
                            if (s.direction == direction && s.targetIndex == targetIndex && s.moveOk && s.endSettled.valid) {
                                errors.push_back(s.endSettled.mpos - s.targetDposMm);
                            }
                        }
                        if (errors.size() < 2) continue;
                        double mean = 0.0;
                        for (const double e : errors) mean += e;
                        mean /= static_cast<double>(errors.size());
                        double ss = 0.0;
                        for (const double e : errors) { const double d = e - mean; ss += d * d; }
                        groupStd.push_back(std::sqrt(ss / static_cast<double>(errors.size() - 1)));
                    }
                }
                if (!groupStd.isEmpty()) {
                    double ss = 0.0;
                    for (const double v : groupStd) ss += v * v;
                    st.repeatabilityMm = std::sqrt(ss / static_cast<double>(groupStd.size()));
                }
                st.qualified = std::isfinite(st.p95DriftMm) && std::isfinite(st.maxDriftMm) && std::isfinite(st.repeatabilityMm)
                    && st.p95DriftMm <= p95LimitMm && st.maxDriftMm <= maxLimitMm && st.repeatabilityMm <= repeatLimitMm;
                return st;
                };
            const auto publishProgress = [this, estimatedSamples](int done, const QString& text) {
                QMetaObject::invokeMethod(this, [this, done, estimatedSamples, text]() {
                    if (m_closeRequested) return;
                    if (m_progressAxisCompSpeed) m_progressAxisCompSpeed->setValue(estimatedSamples > 0 ? std::min(99, 100 * done / estimatedSamples) : 0);
                    if (m_lblAxisCompSpeedStatus) m_lblAxisCompSpeedStatus->setText(text);
                    });
                };
            const auto moveOnly = [&](int axis, double target, const AxisParams& motion, QString& err) -> bool {
                if (m_axisCompSpeedCancel.load() || m_closeRequested) { err = tr("用户取消"); return false; }
                return controller->moveAbsRawDpos(axis, static_cast<float>(target), &motion, &err);
                };

            const auto executeCandidate = [&](const SpeedAxisConfig& cfg, const QString& stage, double candidateSpeed,
                double coarseSpeed, double precisionSpeed,
                QVector<AxisCompensationSpeedTestSample>& out) -> bool {
                    const AxisParams coarseMotion = makeMotion(axisParams[cfg.axisId], coarseSpeed);
                    const AxisParams precisionMotion = makeMotion(axisParams[cfg.axisId], precisionSpeed);
                    const AxisParams transitionMotion = makeMotion(axisParams[cfg.axisId], cfg.configuredMaxSpeed);

                    // Re-order each candidate from the axis' current location. This prevents every candidate from restarting at range-min.
                    QVector<int> remaining;
                    for (int i = 0; i < static_cast<int>(cfg.targets.size()); ++i) remaining.push_back(i);
                    QVector<int> order;
                    AxisMoveDiagnosticSnapshot here = captureAxisMoveDiagnosticSnapshot(controller, cfg.axisId);
                    double current = here.valid ? here.dpos : cfg.initialDpos;
                    while (!remaining.isEmpty()) {
                        int bestPos = 0;
                        double bestDist = std::numeric_limits<double>::infinity();
                        for (int j = 0; j < static_cast<int>(remaining.size()); ++j) {
                            const double dist = std::abs(cfg.targets[remaining[j]] - current);
                            if (dist < bestDist) { bestDist = dist; bestPos = j; }
                        }
                        const int idx = remaining[bestPos];
                        remaining.removeAt(bestPos);
                        order.push_back(idx);
                        current = cfg.targets[idx];
                    }

                    for (const int targetIndex : order) {
                        const double target = cfg.targets[targetIndex];
                        // Long transfer between separate focus points is not part of the speed metric.
                        // Park at the new local target with the per-axis safe upper-limit speed, then perform only local ±preload trials.
                        const AxisMoveDiagnosticSnapshot beforeTarget = captureAxisMoveDiagnosticSnapshot(controller, cfg.axisId);
                        if (beforeTarget.valid && std::abs(beforeTarget.dpos - target) > std::max(3.0 * preload, 1.0)) {
                            QString transitionError;
                            if (!moveOnly(cfg.axisId, target, transitionMotion, transitionError)) {
                                terminalError = tr("%1轴局部测速点转移失败：%2").arg(axisName5D(cfg.axisId), transitionError);
                                return false;
                            }
                        }
                        for (int cycle = 1; cycle <= repeats; ++cycle) {
                            const std::array<int, 2> directions = (cycle % 2 == 1)
                                ? std::array<int, 2>{+1, -1}
                            : std::array<int, 2>{ -1, +1 };
                            for (const int direction : directions) {
                                const double pre = target - static_cast<double>(direction) * preload;
                                AxisCompensationSpeedTestSample sample;
                                sample.axisId = cfg.axisId;
                                sample.stage = stage;
                                sample.candidateSpeedMmS = candidateSpeed;
                                sample.coarseSpeedMmS = coarseSpeed;
                                sample.precisionSpeedMmS = precisionSpeed;
                                sample.direction = direction;
                                sample.cycle = cycle;
                                sample.targetIndex = targetIndex;
                                sample.targetDposMm = target;
                                sample.preloadDposMm = pre;
                                sample.settleMs = settleMs;
                                QString err;
                                if (!moveOnly(cfg.axisId, pre, coarseMotion, err)) {
                                    sample.error = tr("局部预定位失败：%1").arg(err);
                                    out.push_back(sample); allSamples.push_back(sample); ++completed;
                                    return false;
                                }
                                sample.moveOk = moveOnly(cfg.axisId, target, precisionMotion, err);
                                sample.error = err;
                                sample.endIdle = captureAxisMoveDiagnosticSnapshot(controller, cfg.axisId);
                                if (sample.moveOk && settleMs > 0 && !m_axisCompSpeedCancel.load()) QThread::msleep(static_cast<unsigned long>(settleMs));
                                sample.endSettled = captureAxisMoveDiagnosticSnapshot(controller, cfg.axisId);
                                out.push_back(sample); allSamples.push_back(sample); ++completed;
                                publishProgress(completed, tr("%1轴 %2 候选=%3 mm/s：%4")
                                    .arg(axisName5D(cfg.axisId), stage).arg(candidateSpeed, 0, 'f', 3).arg(completed));
                                if (!sample.moveOk) return false;
                                if (!sample.endSettled.valid) {
                                    terminalError = tr("%1轴测速位置读取失败：%2").arg(axisName5D(cfg.axisId), sample.endSettled.error);
                                    return false;
                                }
                            }
                        }
                    }
                    return true;
                };

            for (const auto& cfg : configs) {
                if (m_axisCompSpeedCancel.load() || m_closeRequested) { cancelled = true; break; }
                const AxisMoveDiagnosticSnapshot initial = captureAxisMoveDiagnosticSnapshot(controller, cfg.axisId);
                if (!initial.valid) { terminalError = tr("%1轴测速前位置读取失败：%2").arg(axisName5D(cfg.axisId), initial.error); break; }
                const std::uint64_t generation = initial.mposSetGeneration;
                AxisCompensationSpeedProfile profile;
                profile.axisId = cfg.axisId;
                profile.rangeStartMm = cfg.start;
                profile.rangeEndMm = cfg.end;
                profile.preloadMm = preload;

                double selectedPrecision = std::numeric_limits<double>::quiet_NaN();
                CandidateStats selectedPrecisionStats;
                // V27.6.33: candidates are descending; stop immediately at the first qualified speed.
                for (const double speed : cfg.precisionCandidates) {
                    QVector<AxisCompensationSpeedTestSample> samples;
                    if (!executeCandidate(cfg, QStringLiteral("PRECISION"), speed, speed, speed, samples)) {
                        if (m_axisCompSpeedCancel.load() || m_closeRequested) cancelled = true;
                        else if (terminalError.isEmpty()) terminalError = tr("%1轴精密速度%2 mm/s测试运动失败").arg(axisName5D(cfg.axisId)).arg(speed, 0, 'f', 3);
                        break;
                    }
                    const CandidateStats st = computeStats(samples);
                    if (st.qualified) {
                        selectedPrecision = speed;
                        selectedPrecisionStats = st;
                        break;
                    }
                    const auto snap = captureAxisMoveDiagnosticSnapshot(controller, cfg.axisId);
                    if (snap.mposSetGeneration != generation) { terminalError = tr("%1轴测速期间SetMpos generation改变，已中止。").arg(axisName5D(cfg.axisId)); break; }
                }
                if (cancelled || !terminalError.isEmpty()) break;

                if (!std::isfinite(selectedPrecision)) {
                    profile.status = tr("精密速度无合格候选");
                    profiles[cfg.axisId] = profile;
                }
                else {
                    profile.precisionSpeedMmS = selectedPrecision;
                    profile.precisionP95DriftMm = selectedPrecisionStats.p95DriftMm;
                    profile.precisionMaxDriftMm = selectedPrecisionStats.maxDriftMm;
                    profile.precisionRepeatabilityMm = selectedPrecisionStats.repeatabilityMm;

                    QVector<double> coarseCandidates;
                    double coarse = std::max(selectedPrecision, cfg.configuredMaxSpeed);
                    while (coarse > selectedPrecision + 1e-9) {
                        coarseCandidates.push_back(coarse);
                        const double next = std::max(selectedPrecision, coarse * 0.5);
                        if (std::abs(next - coarse) < 1e-9) break;
                        coarse = next;
                    }
                    if (coarseCandidates.isEmpty() || std::abs(coarseCandidates.back() - selectedPrecision) > 1e-6)
                        coarseCandidates.push_back(selectedPrecision);

                    double selectedCoarse = std::numeric_limits<double>::quiet_NaN();
                    CandidateStats selectedCoarseStats;
                    // Start with the requested automatic-positioning upper limit. Only reduce it if the final precision state is not stable.
                    for (const double coarseSpeed : coarseCandidates) {
                        QVector<AxisCompensationSpeedTestSample> samples;
                        if (!executeCandidate(cfg, QStringLiteral("COARSE"), coarseSpeed, coarseSpeed, selectedPrecision, samples)) {
                            if (m_axisCompSpeedCancel.load() || m_closeRequested) cancelled = true;
                            else if (terminalError.isEmpty()) terminalError = tr("%1轴自动定位速度%2 mm/s测试运动失败").arg(axisName5D(cfg.axisId)).arg(coarseSpeed, 0, 'f', 3);
                            break;
                        }
                        const CandidateStats st = computeStats(samples);
                        if (st.qualified) {
                            selectedCoarse = coarseSpeed;
                            selectedCoarseStats = st;
                            break;
                        }
                    }
                    if (cancelled || !terminalError.isEmpty()) break;

                    if (std::isfinite(selectedCoarse)) {
                        profile.qualified = true;
                        profile.coarseSpeedMmS = selectedCoarse;
                        profile.coarseP95DriftMm = selectedCoarseStats.p95DriftMm;
                        profile.coarseMaxDriftMm = selectedCoarseStats.maxDriftMm;
                        profile.coarseRepeatabilityMm = selectedCoarseStats.repeatabilityMm;
                        profile.status = QStringLiteral("QUALIFIED");
                    }
                    else {
                        profile.status = tr("自动定位速度无合格候选");
                    }
                    profiles[cfg.axisId] = profile;
                }

                QString restoreError;
                AxisParams restoreMotion = makeMotion(axisParams[cfg.axisId], std::max(0.001, cfg.configuredMaxSpeed));
                if (!controller->moveAbsRawDpos(cfg.axisId, static_cast<float>(initial.dpos), &restoreMotion, &restoreError)) {
                    terminalError = tr("%1轴测速后恢复启动位置失败：%2").arg(axisName5D(cfg.axisId), restoreError);
                    break;
                }
            }

            QMetaObject::invokeMethod(this, [this, profiles, allSamples, cancelled, terminalError, completed, outputRoot]() {
                --m_axisWorkerCount;
                m_axisCompSpeedRunning = false;
                m_axisCompSpeedProfiles = profiles;
                m_axisCompSpeedSamples = allSamples;
                if (m_btnHomeAll) m_btnHomeAll->setEnabled(true);
                if (m_btnZeroAll) m_btnZeroAll->setEnabled(true);
                if (m_gcodeWidget) m_gcodeWidget->setEnabled(true);
                if (m_calibrationWidget) m_calibrationWidget->setEnabled(true);
                for (int axis = AxisX; axis <= AxisB; ++axis) if (auto* w = axisWidgetForId(axis)) w->setControlsEnabled(true);
                updateAxisCompensationSpeedQualificationUi();
                updateAxisCompensationCalibrationUi();
                updateAxisCompensationValidationUi();
                if (m_closeRequested) { tryFinishDeferredClose(); return; }
                if (cancelled) {
                    if (m_lblAxisCompSpeedStatus) m_lblAxisCompSpeedStatus->setText(tr("采表速度资格测试已取消；已完成%1条测量。").arg(completed));
                }
                else if (!terminalError.isEmpty()) {
                    if (m_lblAxisCompSpeedStatus) m_lblAxisCompSpeedStatus->setText(tr("采表速度资格测试中止：%1").arg(terminalError));
                    QMessageBox::warning(this, tr("采表速度资格测试中止"), terminalError);
                }
                else {
                    if (m_progressAxisCompSpeed) m_progressAxisCompSpeed->setValue(100);
                    QString savedDir, saveError;
                    if (saveAxisCompensationSpeedQualificationFiles(outputRoot, &savedDir, &saveError)) {
                        m_axisCompSpeedLastRunDir = savedDir;
                        if (m_lblAxisCompSpeedStatus) m_lblAxisCompSpeedStatus->setText(
                            tr("局部采表速度资格测试完成；结果已保存到 %1。只有状态“已通过”的轴才能开始正式采表。")
                            .arg(QDir::toNativeSeparators(savedDir)));
                    }
                    else if (m_lblAxisCompSpeedStatus) {
                        m_lblAxisCompSpeedStatus->setText(tr("测速完成，但保存失败：%1").arg(saveError));
                    }
                }
                tryFinishDeferredClose();
                });
        });
}

void MainWindow::stopAxisCompensationSpeedQualification()
{
    if (!m_axisCompSpeedRunning) return;
    m_axisCompSpeedCancel.store(true);
    if (m_btnAxisCompSpeedStop) m_btnAxisCompSpeedStop->setEnabled(false);
    if (m_zmc && m_zmc->isConnected()) {
        for (int axis = AxisX; axis <= AxisZ; ++axis) m_zmc->requestStopAxis(axis, false);
    }
    if (m_lblAxisCompSpeedStatus) m_lblAxisCompSpeedStatus->setText(tr("正在停止采表速度资格测试..."));
}


void MainWindow::updateAxisCompensationCalibrationUi()
{
    if (!m_comboAxisCompCalAxis || !m_axisCompCalRangeTable) return;
    const int selection = m_comboAxisCompCalAxis->currentData().toInt();
    const bool calibrationUiEnabled = !m_axisCompSpeedRunning && !m_axisCompCalRunning && !m_axisCompValRunning;
    m_comboAxisCompCalAxis->setEnabled(calibrationUiEnabled);
    for (int row = 0; row < 3; ++row) {
        const int axis = AxisX + row;
        const bool rowEnabled = calibrationUiEnabled && (selection < 0 || selection == axis);
        for (int col = 1; col <= 3; ++col) {
            if (auto* spin = qobject_cast<QDoubleSpinBox*>(m_axisCompCalRangeTable->cellWidget(row, col))) {
                spin->setEnabled(rowEnabled);
            }
        }
    }
    if (m_btnAxisCompCalStart) m_btnAxisCompCalStart->setEnabled(calibrationUiEnabled);
    if (m_btnAxisCompCalStop) m_btnAxisCompCalStop->setEnabled(m_axisCompCalRunning);
    if (m_btnAxisCompCalSave) m_btnAxisCompCalSave->setEnabled(calibrationUiEnabled && !m_axisCompCalSamples.isEmpty());
    if (m_btnAxisCompCalUseSoftLimits) m_btnAxisCompCalUseSoftLimits->setEnabled(calibrationUiEnabled);
    if (m_btnAxisCompCalBrowseDir) m_btnAxisCompCalBrowseDir->setEnabled(calibrationUiEnabled);
    if (m_editAxisCompCalOutputDir) m_editAxisCompCalOutputDir->setEnabled(calibrationUiEnabled);
    if (m_spinAxisCompCalRepeats) m_spinAxisCompCalRepeats->setEnabled(calibrationUiEnabled);
    if (m_spinAxisCompCalSettleMs) m_spinAxisCompCalSettleMs->setEnabled(calibrationUiEnabled);
    if (m_spinAxisCompCalPreloadMm) m_spinAxisCompCalPreloadMm->setEnabled(calibrationUiEnabled);
    updateAxisCompensationSpeedQualificationUi();
}

void MainWindow::fillAxisCompensationRangesFromSoftLimits()
{
    if (!m_axisCompCalRangeTable) return;
    const auto params = currentAxisParams5D();
    const double preload = m_spinAxisCompCalPreloadMm ? m_spinAxisCompCalPreloadMm->value() : 2.0;
    QStringList skipped;
    for (int row = 0; row < 3; ++row) {
        const int axis = AxisX + row;
        auto* startSpin = qobject_cast<QDoubleSpinBox*>(m_axisCompCalRangeTable->cellWidget(row, 1));
        auto* endSpin = qobject_cast<QDoubleSpinBox*>(m_axisCompCalRangeTable->cellWidget(row, 2));
        auto* stepSpin = qobject_cast<QDoubleSpinBox*>(m_axisCompCalRangeTable->cellWidget(row, 3));
        if (!startSpin || !endSpin || !stepSpin) continue;
        const double lo = static_cast<double>(params[axis].softLimitNeg);
        const double hi = static_cast<double>(params[axis].softLimitPos);
        const double margin = preload + std::max(0.05, stepSpin->value());
        if (!std::isfinite(lo) || !std::isfinite(hi) || lo >= hi
            || std::abs(lo) >= 100000.0 || std::abs(hi) >= 100000.0
            || lo + margin >= hi - margin) {
            skipped << axisName5D(axis);
            continue;
        }
        startSpin->setValue(lo + margin);
        endSpin->setValue(hi - margin);
    }
    if (!skipped.isEmpty()) {
        QMessageBox::information(this, tr("补偿表范围"),
            tr("以下轴当前软限位无效或范围不足，未自动填充：%1。请先读取/设置真实软限位后手工填写。")
            .arg(skipped.join(QStringLiteral(", "))));
    }
}

void MainWindow::appendAxisCompensationCalibrationSample(const AxisCompensationCalibrationSample& sample)
{
    m_axisCompCalSamples.push_back(sample);
    if (m_lblAxisCompCalStatus) {
        const double correction = sample.endSettled.valid
            ? sample.endSettled.dpos - sample.endSettled.mpos
            : std::numeric_limits<double>::quiet_NaN();
        const double drift = sample.endIdle.valid && sample.endSettled.valid
            ? sample.endSettled.mpos - sample.endIdle.mpos
            : std::numeric_limits<double>::quiet_NaN();
        m_lblAxisCompCalStatus->setText(
            tr("已采%1点；最新 %2 %3：DPOS=%4，MPOS=%5，修正=%6 mm，IDLE后漂移=%7 mm。"
                "当前节点采用自动定位=%8 mm/s、精密接近=%9 mm/s；当前版本仅生成候选表，不会全局启用补偿。")
            .arg(m_axisCompCalSamples.size())
            .arg(axisName5D(sample.axisId), sample.direction > 0 ? QStringLiteral("+") : QStringLiteral("-"))
            .arg(sample.endSettled.dpos, 0, 'f', 6)
            .arg(sample.endSettled.mpos, 0, 'f', 6)
            .arg(correction, 0, 'f', 6)
            .arg(drift, 0, 'f', 6)
            .arg(sample.coarseSpeedMmS, 0, 'f', 3).arg(sample.precisionSpeedMmS, 0, 'f', 3));
    }
    updateAxisCompensationCalibrationUi();
}

QVector<MainWindow::AxisCompensationTableNode> MainWindow::buildAxisCompensationTableNodes() const
{
    QVector<const AxisCompensationCalibrationSample*> valid;
    valid.reserve(m_axisCompCalSamples.size());
    for (const auto& s : m_axisCompCalSamples) {
        if (s.axisId < AxisX || s.axisId > AxisZ || (s.direction != 1 && s.direction != -1)) continue;
        if (!s.moveOk || !s.endSettled.valid || !std::isfinite(s.requestedDposMm)) continue;
        valid.push_back(&s);
    }
    std::sort(valid.begin(), valid.end(), [](const auto* a, const auto* b) {
        if (a->axisId != b->axisId) return a->axisId < b->axisId;
        if (a->direction != b->direction) return a->direction > b->direction; // + first
        return a->requestedDposMm < b->requestedDposMm;
        });

    QVector<AxisCompensationTableNode> nodes;
    int i = 0;
    while (i < valid.size()) {
        const int axis = valid[i]->axisId;
        const int direction = valid[i]->direction;
        const double nominal = valid[i]->requestedDposMm;
        int j = i;
        QVector<double> dposValues;
        QVector<double> mposValues;
        QVector<double> corrections;
        QVector<double> drifts;
        while (j < valid.size()
            && valid[j]->axisId == axis
            && valid[j]->direction == direction
            && std::abs(valid[j]->requestedDposMm - nominal) < 1.0e-7) {
            const auto& s = *valid[j];
            const double d = s.endSettled.dpos;
            const double m = s.endSettled.mpos;
            if (std::isfinite(d) && std::isfinite(m)) {
                dposValues.push_back(d);
                mposValues.push_back(m);
                corrections.push_back(d - m);
                if (s.endIdle.valid && std::isfinite(s.endIdle.mpos)) {
                    drifts.push_back(m - s.endIdle.mpos);
                }
            }
            ++j;
        }

        const auto mean = [](const QVector<double>& values) -> double {
            if (values.isEmpty()) return std::numeric_limits<double>::quiet_NaN();
            double sum = 0.0;
            for (const double v : values) sum += v;
            return sum / static_cast<double>(values.size());
            };
        const auto stddev = [&](const QVector<double>& values) -> double {
            if (values.size() < 2) return 0.0;
            const double mu = mean(values);
            double ss = 0.0;
            for (const double v : values) { const double dv = v - mu; ss += dv * dv; }
            return std::sqrt(ss / static_cast<double>(values.size() - 1));
            };

        if (!mposValues.isEmpty()) {
            AxisCompensationTableNode n;
            n.axisId = axis;
            n.direction = direction;
            n.nominalDposMm = nominal;
            n.requiredDposMm = mean(dposValues);
            n.desiredMposMm = mean(mposValues);
            n.correctionMm = mean(corrections);
            n.correctionStdDevMm = stddev(corrections);
            n.mposStdDevMm = stddev(mposValues);
            n.meanPostIdleMposDriftMm = mean(drifts);
            n.sampleCount = static_cast<int>(mposValues.size());
            nodes.push_back(n);
        }
        i = j;
    }
    return nodes;
}

void MainWindow::updateAxisCompensationSummaryTable()
{
    if (!m_axisCompCalSummaryTable) return;
    const QVector<AxisCompensationTableNode> nodes = buildAxisCompensationTableNodes();
    m_axisCompCalSummaryTable->setRowCount(0);
    const auto number = [](double v) {
        return std::isfinite(v) ? QString::number(v, 'f', 6) : QStringLiteral("-");
        };
    for (const auto& n : nodes) {
        const int row = m_axisCompCalSummaryTable->rowCount();
        m_axisCompCalSummaryTable->insertRow(row);
        const QStringList values{
            axisName5D(n.axisId),
            n.direction > 0 ? QStringLiteral("+") : QStringLiteral("-"),
            number(n.nominalDposMm),
            number(n.desiredMposMm),
            number(n.correctionMm),
            number(n.correctionStdDevMm),
            number(n.meanPostIdleMposDriftMm),
            QString::number(n.sampleCount)
        };
        for (int col = 0; col < values.size(); ++col) {
            auto* item = new QTableWidgetItem(values[col]);
            if (col >= 2 && col <= 6) item->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);
            m_axisCompCalSummaryTable->setItem(row, col, item);
        }
    }
}

bool MainWindow::saveAxisCompensationCalibrationFiles(const QString& outputRoot,
    QString* savedRunDirectory,
    QString* error) const
{
    if (m_axisCompCalSamples.isEmpty()) {
        if (error) *error = tr("没有可保存的补偿标定样本");
        return false;
    }
    const QString root = QDir::cleanPath(outputRoot.trimmed());
    if (root.isEmpty()) {
        if (error) *error = tr("保存目录为空");
        return false;
    }
    QDir rootDir;
    if (!rootDir.mkpath(root)) {
        if (error) *error = tr("无法创建保存目录：%1").arg(root);
        return false;
    }
    const QString runName = QStringLiteral("axis_comp_candidate_%1")
        .arg(QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss_zzz")));
    const QString runDirPath = QDir(root).filePath(runName);
    if (!rootDir.mkpath(runDirPath)) {
        if (error) *error = tr("无法创建本次采表目录：%1").arg(runDirPath);
        return false;
    }
    QDir runDir(runDirPath);
    const auto csvQuote = [](QString value) {
        value.replace('"', QStringLiteral("\"\""));
        return QStringLiteral("\"") + value + QStringLiteral("\"");
        };
    const auto num = [](double v) {
        return std::isfinite(v) ? QString::number(v, 'f', 9) : QString();
        };

    QFile rawFile(runDir.filePath(QStringLiteral("raw_axis_compensation_samples.csv")));
    if (!rawFile.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate)) {
        if (error) *error = tr("无法写入原始CSV：%1").arg(rawFile.errorString());
        return false;
    }
    QTextStream raw(&rawFile);
    raw.setEncoding(QStringConverter::Utf8);
    raw << "record,axis,direction,cycle,point_index,approach_mode,requested_dpos_mm,approach_start_dpos_mm,coarse_speed_mm_s,precision_speed_mm_s,settle_ms,"
        "idle_utc,idle_dpos_mm,idle_mpos_mm,idle_encoder,idle_fe_mm,"
        "settled_utc,settled_dpos_mm,settled_mpos_mm,settled_encoder,settled_fe_mm,"
        "correction_dpos_minus_mpos_mm,post_idle_mpos_drift_mm,linear_referenced,setmpos_generation,datum_input,datum_raw,move_ok,status\n";
    int rawIndex = 0;
    for (const auto& s : m_axisCompCalSamples) {
        const double idleFe = s.endIdle.valid ? s.endIdle.dpos - s.endIdle.mpos : std::numeric_limits<double>::quiet_NaN();
        const double settledFe = s.endSettled.valid ? s.endSettled.dpos - s.endSettled.mpos : std::numeric_limits<double>::quiet_NaN();
        const double correction = settledFe;
        const double drift = s.endIdle.valid && s.endSettled.valid
            ? s.endSettled.mpos - s.endIdle.mpos : std::numeric_limits<double>::quiet_NaN();
        QString status = s.error;
        if (status.isEmpty()) {
            QStringList notes;
            if (!s.endIdle.error.isEmpty()) notes << s.endIdle.error;
            if (!s.endSettled.error.isEmpty()) notes << s.endSettled.error;
            status = notes.join(QStringLiteral("; "));
        }
        raw << ++rawIndex << ',' << csvQuote(axisName5D(s.axisId)) << ','
            << csvQuote(s.direction > 0 ? QStringLiteral("+") : QStringLiteral("-")) << ','
            << s.cycle << ',' << s.pointIndex << ',' << csvQuote(s.approachMode) << ',' << num(s.requestedDposMm) << ','
            << num(s.preloadDposMm) << ',' << num(s.coarseSpeedMmS) << ',' << num(s.precisionSpeedMmS) << ',' << s.settleMs << ','
            << csvQuote(s.endIdle.utc) << ',' << num(s.endIdle.dpos) << ',' << num(s.endIdle.mpos) << ','
            << (s.endIdle.encoderValid ? num(s.endIdle.encoder) : QString()) << ',' << num(idleFe) << ','
            << csvQuote(s.endSettled.utc) << ',' << num(s.endSettled.dpos) << ',' << num(s.endSettled.mpos) << ','
            << (s.endSettled.encoderValid ? num(s.endSettled.encoder) : QString()) << ',' << num(settledFe) << ','
            << num(correction) << ',' << num(drift) << ','
            << (s.endSettled.linearReferenced ? 1 : 0) << ','
            << static_cast<qulonglong>(s.endSettled.mposSetGeneration) << ','
            << s.endSettled.datumInput << ',' << s.endSettled.datumRaw << ','
            << (s.moveOk ? 1 : 0) << ',' << csvQuote(status) << '\n';
    }
    rawFile.close();

    const QVector<AxisCompensationTableNode> nodes = buildAxisCompensationTableNodes();
    QJsonObject axesJson;
    for (int axis = AxisX; axis <= AxisZ; ++axis) {
        QVector<AxisCompensationTableNode> axisNodes;
        for (const auto& n : nodes) if (n.axisId == axis) axisNodes.push_back(n);
        if (axisNodes.isEmpty()) continue;

        QFile tableFile(runDir.filePath(QStringLiteral("%1_compensation_candidate.csv").arg(axisName5D(axis))));
        if (!tableFile.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate)) {
            if (error) *error = tr("无法写入%1补偿表：%2").arg(axisName5D(axis), tableFile.errorString());
            return false;
        }
        QTextStream table(&tableFile);
        table.setEncoding(QStringConverter::Utf8);
        table << "axis,direction,nominal_command_dpos_mm,desired_mpos_mm,required_dpos_mm,correction_required_dpos_minus_desired_mpos_mm,"
            "correction_stddev_mm,mpos_stddev_mm,mean_post_idle_mpos_drift_mm,samples\n";

        QJsonObject axisObj;
        for (const int direction : {+1, -1}) {
            QVector<AxisCompensationTableNode> dirNodes;
            for (const auto& n : axisNodes) if (n.direction == direction) dirNodes.push_back(n);
            std::sort(dirNodes.begin(), dirNodes.end(), [](const auto& a, const auto& b) {
                return a.desiredMposMm < b.desiredMposMm;
                });
            bool monotonic = true;
            for (int k = 1; k < dirNodes.size(); ++k) {
                if (!(dirNodes[k].desiredMposMm > dirNodes[k - 1].desiredMposMm)) monotonic = false;
            }
            double minCorrection = std::numeric_limits<double>::infinity();
            double maxCorrection = -std::numeric_limits<double>::infinity();
            for (const auto& n : dirNodes) {
                minCorrection = std::min(minCorrection, n.correctionMm);
                maxCorrection = std::max(maxCorrection, n.correctionMm);
                table << csvQuote(axisName5D(axis)) << ','
                    << csvQuote(direction > 0 ? QStringLiteral("+") : QStringLiteral("-")) << ','
                    << num(n.nominalDposMm) << ',' << num(n.desiredMposMm) << ',' << num(n.requiredDposMm) << ','
                    << num(n.correctionMm) << ',' << num(n.correctionStdDevMm) << ',' << num(n.mposStdDevMm) << ','
                    << num(n.meanPostIdleMposDriftMm) << ',' << n.sampleCount << '\n';
            }
            QJsonObject dirObj;
            dirObj[QStringLiteral("node_count")] = dirNodes.size();
            dirObj[QStringLiteral("desired_mpos_strictly_monotonic")] = monotonic;
            if (!dirNodes.isEmpty()) {
                dirObj[QStringLiteral("desired_mpos_min_mm")] = dirNodes.front().desiredMposMm;
                dirObj[QStringLiteral("desired_mpos_max_mm")] = dirNodes.back().desiredMposMm;
                dirObj[QStringLiteral("correction_min_mm")] = minCorrection;
                dirObj[QStringLiteral("correction_max_mm")] = maxCorrection;
                dirObj[QStringLiteral("correction_peak_to_peak_mm")] = maxCorrection - minCorrection;
            }
            axisObj[direction > 0 ? QStringLiteral("positive") : QStringLiteral("negative")] = dirObj;
        }
        tableFile.close();
        axesJson[axisName5D(axis)] = axisObj;
    }

    QJsonObject manifest;
    manifest[QStringLiteral("format")] = QStringLiteral("MotionLaserSync.AxisPositionCompensationCandidate");
    manifest[QStringLiteral("version")] = 3;
    manifest[QStringLiteral("software_release")] = QStringLiteral("V27.6.35");
    manifest[QStringLiteral("created_utc")] = QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs);
    manifest[QStringLiteral("status")] = QStringLiteral("CANDIDATE_UNVALIDATED");
    manifest[QStringLiteral("runtime_compensation_enabled")] = false;
    manifest[QStringLiteral("feedback_source")] = QStringLiteral("linear-axis MPOS / external grating");
    manifest[QStringLiteral("mapping")] = QStringLiteral("desired_mpos_mm -> required_dpos_mm");
    manifest[QStringLiteral("settle_ms")] = m_spinAxisCompCalSettleMs ? m_spinAxisCompCalSettleMs->value() : 0;
    manifest[QStringLiteral("repeats")] = m_spinAxisCompCalRepeats ? m_spinAxisCompCalRepeats->value() : 0;
    manifest[QStringLiteral("direction_preload_mm")] = m_spinAxisCompCalPreloadMm ? m_spinAxisCompCalPreloadMm->value() : 0.0;
    manifest[QStringLiteral("sample_count")] = m_axisCompCalSamples.size();
    QJsonObject gridJson;
    if (m_axisCompCalRangeTable) {
        for (int row = 0; row < 3; ++row) {
            const int axis = AxisX + row;
            auto* startSpin = qobject_cast<QDoubleSpinBox*>(m_axisCompCalRangeTable->cellWidget(row, 1));
            auto* endSpin = qobject_cast<QDoubleSpinBox*>(m_axisCompCalRangeTable->cellWidget(row, 2));
            auto* stepSpin = qobject_cast<QDoubleSpinBox*>(m_axisCompCalRangeTable->cellWidget(row, 3));
            if (!startSpin || !endSpin || !stepSpin) continue;
            QJsonObject g;
            g[QStringLiteral("start_dpos_mm")] = startSpin->value();
            g[QStringLiteral("end_dpos_mm")] = endSpin->value();
            g[QStringLiteral("step_mm")] = stepSpin->value();
            gridJson[axisName5D(axis)] = g;
        }
    }
    manifest[QStringLiteral("calibration_grid")] = gridJson;
    QJsonObject motionProfilesJson;
    for (int axis = AxisX; axis <= AxisZ; ++axis) {
        if (!axesJson.contains(axisName5D(axis))) continue;
        const auto& p = m_axisCompSpeedProfiles[axis];
        if (!p.qualified) continue;
        QJsonObject mp;
        mp[QStringLiteral("qualified")] = true;
        mp[QStringLiteral("coarse_speed_mm_s")] = p.coarseSpeedMmS;
        mp[QStringLiteral("precision_final_approach_speed_mm_s")] = p.precisionSpeedMmS;
        mp[QStringLiteral("final_approach_distance_mm")] = p.preloadMm;
        mp[QStringLiteral("coarse_p95_post_idle_drift_mm")] = p.coarseP95DriftMm;
        mp[QStringLiteral("coarse_repeatability_rms_mm")] = p.coarseRepeatabilityMm;
        mp[QStringLiteral("precision_p95_post_idle_drift_mm")] = p.precisionP95DriftMm;
        mp[QStringLiteral("precision_repeatability_rms_mm")] = p.precisionRepeatabilityMm;
        motionProfilesJson[axisName5D(axis)] = mp;
    }
    manifest[QStringLiteral("motion_profiles")] = motionProfilesJson;
    manifest[QStringLiteral("motion_strategy")] = QStringLiteral("direction-prime-once then monotonic precision sweep: one coarse preload at each direction/cycle start, then all remaining grid nodes are reached monotonically at precision speed");
    manifest[QStringLiteral("sampling_strategy")] = QStringLiteral("DPOS grid is absolute; no numerical DPOS-MPOS error accumulation. Long reposition moves are unmeasured and only establish the first directional preload; measured nodes use monotonic precision motion.");
    manifest[QStringLiteral("axes")] = axesJson;
    manifest[QStringLiteral("note")] = QStringLiteral(
        "V27.6.35 keeps this RAW acquisition workflow isolated from runtime compensation. Candidate generation does not approve or enable a production table. Validation auto-matches each axis manifest.");

    QFile manifestFile(runDir.filePath(QStringLiteral("axis_compensation_manifest.json")));
    if (!manifestFile.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        if (error) *error = tr("无法写入manifest：%1").arg(manifestFile.errorString());
        return false;
    }
    manifestFile.write(QJsonDocument(manifest).toJson(QJsonDocument::Indented));
    manifestFile.close();

    if (savedRunDirectory) *savedRunDirectory = runDirPath;
    return true;
}

void MainWindow::saveAxisCompensationCalibrationFilesInteractive()
{
    if (m_axisCompCalRunning) {
        QMessageBox::information(this, tr("保存补偿表"), tr("采集仍在运行，请先完成或停止。"));
        return;
    }
    const QString root = m_editAxisCompCalOutputDir ? m_editAxisCompCalOutputDir->text() : QString();
    QString runDir, err;
    if (!saveAxisCompensationCalibrationFiles(root, &runDir, &err)) {
        QMessageBox::warning(this, tr("保存补偿表失败"), err);
        return;
    }
    QMessageBox::information(this, tr("候选补偿表已保存"),
        tr("已保存到：\n%1\n\n注意：这些表仍是 CANDIDATE_UNVALIDATED；V27.6.35 的此采集工作流继续使用原始 DPOS。新表不会自动替换运行补偿表；必须独立验证后更新补偿包，再人工加载/启用。")
        .arg(QDir::toNativeSeparators(runDir)));
}

void MainWindow::startAxisCompensationCalibration()
{
    if (m_zmc && m_zmc->axisCompensationEnabled()) {
        QMessageBox::warning(this, tr("RAW 采集/诊断/验证"),
            tr("请先在 XYZ 运行补偿面板禁用运行补偿，再启动 RAW 采集或独立 A/B 验证。"));
        return;
    }
    if (m_axisCompCalRunning) return;
    if (m_axisCompSpeedRunning) {
        QMessageBox::information(this, tr("XYZ补偿表标定"), tr("采表速度资格测试仍在运行，请先完成或停止。"));
        return;
    }
    if (m_axisCompValRunning) {
        QMessageBox::information(this, tr("XYZ补偿表标定"), tr("候选补偿表A/B验证仍在运行，请先停止。"));
        return;
    }
    if (m_axisMoveDiagAutoRunning) {
        QMessageBox::information(this, tr("XYZ补偿表标定"), tr("DPOS/MPOS诊断自动测试仍在运行，请先停止。"));
        return;
    }
    if (!m_zmc || !m_zmc->isConnected()) {
        QMessageBox::warning(this, tr("XYZ补偿表标定"), tr("请先连接控制器。"));
        return;
    }
    if ((m_gcodeExecutor && (m_gcodeExecutor->isRunning() || m_gcodeExecutor->isPaused()))
        || (m_syncManager && m_syncManager->isRunning())
        || m_jointMoveInProgress || m_sphereStudyAutoActive) {
        QMessageBox::information(this, tr("XYZ补偿表标定"),
            tr("当前有G代码/同步采集/五轴定位/自动球研究任务运行，请先完全停止后再采补偿表。"));
        return;
    }
    for (int axis = AxisX; axis <= AxisB; ++axis) {
        if (m_zmc->isAxisBusy(axis)) {
            QMessageBox::information(this, tr("XYZ补偿表标定"), tr("当前仍有轴运动任务，请全部停稳后再开始。"));
            return;
        }
    }

    const int selection = m_comboAxisCompCalAxis ? m_comboAxisCompCalAxis->currentData().toInt() : AxisZ;
    QVector<int> axes;
    if (selection < 0) axes = { AxisX, AxisY, AxisZ };
    else if (selection >= AxisX && selection <= AxisZ) axes = { selection };
    else return;

    QStringList notReferenced;
    for (const int axis : axes) {
        if (!m_zmc->isLinearMposReferenced(axis)) notReferenced << axisName5D(axis);
    }
    if (!notReferenced.isEmpty()) {
        QMessageBox::warning(this, tr("XYZ补偿表标定"),
            tr("正式补偿表只能在正规回零并建立MPOS机床零点后采集。以下轴尚未回零：%1")
            .arg(notReferenced.join(QStringLiteral(", "))));
        return;
    }

    struct ScanConfig {
        int axisId = -1;
        double start = 0.0;
        double end = 0.0;
        double step = 0.5;
        double coarseSpeedMmS = 0.0;
        double precisionSpeedMmS = 0.0;
        QVector<double> points;
    };
    QVector<ScanConfig> configs;
    const auto axisParams = currentAxisParams5D();
    const double preload = m_spinAxisCompCalPreloadMm ? m_spinAxisCompCalPreloadMm->value() : 2.0;
    const int repeats = m_spinAxisCompCalRepeats ? m_spinAxisCompCalRepeats->value() : 3;
    const int settleMs = m_spinAxisCompCalSettleMs ? m_spinAxisCompCalSettleMs->value() : 2000;
    int totalSamples = 0;

    for (const int axis : axes) {
        const int row = axis - AxisX;
        auto* startSpin = qobject_cast<QDoubleSpinBox*>(m_axisCompCalRangeTable->cellWidget(row, 1));
        auto* endSpin = qobject_cast<QDoubleSpinBox*>(m_axisCompCalRangeTable->cellWidget(row, 2));
        auto* stepSpin = qobject_cast<QDoubleSpinBox*>(m_axisCompCalRangeTable->cellWidget(row, 3));
        if (!startSpin || !endSpin || !stepSpin) {
            QMessageBox::warning(this, tr("XYZ补偿表标定"), tr("%1轴范围控件无效。").arg(axisName5D(axis)));
            return;
        }
        ScanConfig cfg;
        cfg.axisId = axis;
        cfg.start = startSpin->value();
        cfg.end = endSpin->value();
        cfg.step = stepSpin->value();
        const auto& speedProfile = m_axisCompSpeedProfiles[axis];
        if (!speedProfile.qualified) {
            QMessageBox::warning(this, tr("XYZ补偿表标定"),
                tr("%1轴尚未通过“采表前轴速度资格测试”。请先自动测定该轴的自动定位速度和精密最终接近速度。")
                .arg(axisName5D(axis)));
            return;
        }
        if (std::abs(speedProfile.rangeStartMm - cfg.start) > 1e-6
            || std::abs(speedProfile.rangeEndMm - cfg.end) > 1e-6
            || std::abs(speedProfile.preloadMm - preload) > 1e-6) {
            QMessageBox::warning(this, tr("XYZ补偿表标定"),
                tr("%1轴测速结果与当前采表范围/预加载不一致。\n测速范围=[%2,%3]，preload=%4；当前=[%5,%6]，preload=%7。\n请重新测速。")
                .arg(axisName5D(axis))
                .arg(speedProfile.rangeStartMm, 0, 'f', 3).arg(speedProfile.rangeEndMm, 0, 'f', 3).arg(speedProfile.preloadMm, 0, 'f', 3)
                .arg(cfg.start, 0, 'f', 3).arg(cfg.end, 0, 'f', 3).arg(preload, 0, 'f', 3));
            return;
        }
        cfg.coarseSpeedMmS = speedProfile.coarseSpeedMmS;
        cfg.precisionSpeedMmS = speedProfile.precisionSpeedMmS;
        if (!(cfg.start < cfg.end) || !(cfg.step > 0.0)) {
            QMessageBox::warning(this, tr("XYZ补偿表标定"),
                tr("%1轴必须满足 起始位置 < 终止位置 且 步长 > 0。").arg(axisName5D(axis)));
            return;
        }
        const double softLo = static_cast<double>(axisParams[axis].softLimitNeg);
        const double softHi = static_cast<double>(axisParams[axis].softLimitPos);
        if (cfg.start - preload < softLo - 1e-9 || cfg.end + preload > softHi + 1e-9) {
            QMessageBox::warning(this, tr("XYZ补偿表标定"),
                tr("%1轴采表范围加换向预加载后超出软限位。\n采表=[%2,%3] mm，预加载=%4 mm，软限位=[%5,%6] mm。\n"
                    "请缩小采表范围或减小预加载。")
                .arg(axisName5D(axis))
                .arg(cfg.start, 0, 'f', 3).arg(cfg.end, 0, 'f', 3).arg(preload, 0, 'f', 3)
                .arg(softLo, 0, 'f', 3).arg(softHi, 0, 'f', 3));
            return;
        }
        const int maxPoints = 5001;
        for (int k = 0; k < maxPoints; ++k) {
            const double q = cfg.start + static_cast<double>(k) * cfg.step;
            if (q > cfg.end + 1e-9) break;
            cfg.points.push_back(std::min(q, cfg.end));
            if (std::abs(q - cfg.end) < 1e-9) break;
        }
        if (cfg.points.isEmpty() || cfg.points.back() < cfg.end - 1e-8) cfg.points.push_back(cfg.end);
        if (cfg.points.size() > maxPoints) {
            QMessageBox::warning(this, tr("XYZ补偿表标定"), tr("%1轴采样点超过%2，请增大步长。").arg(axisName5D(axis)).arg(maxPoints));
            return;
        }
        totalSamples += cfg.points.size() * repeats * 2;
        configs.push_back(cfg);
    }

    const double settleMinutes = static_cast<double>(totalSamples) * static_cast<double>(settleMs) / 60000.0;
    QStringList desc;
    for (const auto& c : configs) {
        desc << tr("%1:[%2,%3] step=%4 (%5点)，自动定位=%6 mm/s，精密接近=%7 mm/s")
            .arg(axisName5D(c.axisId))
            .arg(c.start, 0, 'f', 3).arg(c.end, 0, 'f', 3).arg(c.step, 0, 'f', 3)
            .arg(c.points.size()).arg(c.coarseSpeedMmS, 0, 'f', 3).arg(c.precisionSpeedMmS, 0, 'f', 3);
    }
    const QString confirm = tr(
        "将开始XYZ位置补偿表自动采集：\n%1\n\n"
        "• 所有轴必须已经正规回零；\n"
        "• 当前版本不应用任何补偿，直接使用原始DPOS绝对定位；\n"
        "• 每个方向/每个循环只在首个节点用“自动定位速度”移动到反侧%3 mm建立方向；之后所有网格节点只用“精密最终接近速度”单向连续扫描；\n"
        "• 正向递增、反向递减，不再逐点回退预加载，避免锯齿往返和不必要的机械状态扰动；重复%2次；\n"
        "• 每个测量点IDLE后再等待%4 ms；仅稳定等待预计约%5 min；\n"
        "• 共%6个测量样本；完成后自动生成 desired MPOS → required DPOS 候选表；\n"
        "• 采集过程中不会调用SetMpos，且补偿表不会自动启用。\n\n"
        "请确认整个扫描范围、预加载位置、工装/传感器和急停均安全。")
        .arg(desc.join(QStringLiteral("\n")))
        .arg(repeats).arg(preload, 0, 'f', 3).arg(settleMs)
        .arg(settleMinutes, 0, 'f', 1).arg(totalSamples);
    if (QMessageBox::warning(this, tr("确认XYZ补偿表自动采集"), confirm,
        QMessageBox::Yes | QMessageBox::No, QMessageBox::No) != QMessageBox::Yes) {
        return;
    }

    m_axisCompCalSamples.clear();
    if (m_axisCompCalSummaryTable) m_axisCompCalSummaryTable->setRowCount(0);
    m_axisCompCalRunning = true;
    m_axisCompCalCancel.store(false);
    if (m_progressAxisCompCal) m_progressAxisCompCal->setValue(0);
    updateAxisCompensationCalibrationUi();
    updateAxisCompensationValidationUi();
    if (m_btnHomeAll) m_btnHomeAll->setEnabled(false);
    if (m_btnZeroAll) m_btnZeroAll->setEnabled(false);
    if (m_btnAxisMoveDiagAutoStart) m_btnAxisMoveDiagAutoStart->setEnabled(false);
    if (m_btnAxisMoveDiagManualStart) m_btnAxisMoveDiagManualStart->setEnabled(false);
    if (m_btnAxisMoveDiagManualEnd) m_btnAxisMoveDiagManualEnd->setEnabled(false);
    if (m_gcodeWidget) m_gcodeWidget->setEnabled(false);
    if (m_calibrationWidget) m_calibrationWidget->setEnabled(false);
    if (m_btnStartMerge) m_btnStartMerge->setEnabled(false);
    if (m_btnStartCapture) m_btnStartCapture->setEnabled(false);
    for (int axis = AxisX; axis <= AxisB; ++axis) if (auto* w = axisWidgetForId(axis)) w->setControlsEnabled(false);
    if (m_lblAxisCompCalStatus) m_lblAxisCompCalStatus->setText(tr("XYZ补偿表采集运行中，共%1个测量样本...").arg(totalSamples));

    ++m_axisWorkerCount;
    ZmcController* controller = m_zmc.get();
    const QString outputRoot = m_editAxisCompCalOutputDir ? m_editAxisCompCalOutputDir->text() : QString();
    (void)QtConcurrent::run([this, controller, configs, axisParams, repeats, settleMs, preload, totalSamples, outputRoot]() {
        bool cancelled = false;
        QString terminalError;
        int completed = 0;

        const auto publish = [this, totalSamples](const AxisCompensationCalibrationSample& sample, int done) {
            QMetaObject::invokeMethod(this, [this, sample, done, totalSamples]() {
                if (m_closeRequested) return;
                appendAxisCompensationCalibrationSample(sample);
                if (m_progressAxisCompCal) m_progressAxisCompCal->setValue(totalSamples > 0 ? 100 * done / totalSamples : 0);
                });
            };
        const auto moveOnly = [&](int axis, double target, const AxisParams& motion, QString& err) -> bool {
            if (m_axisCompCalCancel.load() || m_closeRequested) { err = tr("用户取消"); return false; }
            return controller->moveAbsRawDpos(axis, static_cast<float>(target), &motion, &err);
            };
        const auto makeMotion = [](const AxisParams& base, double speed) {
            AxisParams motion = base;
            motion.speed = static_cast<float>(std::max(0.001, speed));
            motion.lspeed = std::min(motion.lspeed, motion.speed);
            return motion;
            };
        const auto measurePoint = [&](int axis, int direction, int cycle, int pointIndex,
            double target, bool primeDirection, double approachStart,
            double coarseSpeed, double precisionSpeed,
            const AxisParams& coarseMotion, const AxisParams& precisionMotion) -> bool {
                AxisCompensationCalibrationSample sample;
                sample.axisId = axis;
                sample.direction = direction;
                sample.cycle = cycle;
                sample.pointIndex = pointIndex;
                sample.approachMode = primeDirection ? QStringLiteral("PRIME") : QStringLiteral("SWEEP");
                sample.requestedDposMm = target;
                sample.preloadDposMm = primeDirection
                    ? target - static_cast<double>(direction) * preload
                    : approachStart;
                sample.coarseSpeedMmS = coarseSpeed;
                sample.precisionSpeedMmS = precisionSpeed;
                sample.settleMs = settleMs;
                QString err;

                // V27.6.33: only the first node of each direction/cycle performs a direction-establishing preload.
                // All later nodes are reached monotonically at precision speed; no saw-tooth backtracking between 0.5 mm nodes.
                if (primeDirection) {
                    if (!moveOnly(axis, sample.preloadDposMm, coarseMotion, err)) {
                        sample.moveOk = false;
                        sample.error = tr("方向预置点定位失败：%1").arg(err);
                        ++completed;
                        publish(sample, completed);
                        if (m_axisCompCalCancel.load() || m_closeRequested) cancelled = true;
                        else terminalError = sample.error;
                        return false;
                    }
                }

                sample.moveOk = moveOnly(axis, target, precisionMotion, err);
                sample.error = err;
                sample.endIdle = captureAxisMoveDiagnosticSnapshot(controller, axis);
                if (sample.moveOk && settleMs > 0 && !m_axisCompCalCancel.load()) {
                    QThread::msleep(static_cast<unsigned long>(settleMs));
                }
                sample.endSettled = captureAxisMoveDiagnosticSnapshot(controller, axis);
                ++completed;
                publish(sample, completed);
                if (!sample.moveOk) {
                    if (m_axisCompCalCancel.load() || m_closeRequested) cancelled = true;
                    else terminalError = err.isEmpty() ? tr("%1轴精密单向扫描失败").arg(axisName5D(axis)) : err;
                    return false;
                }
                if (!sample.endSettled.valid) {
                    terminalError = tr("%1轴位置读取失败：%2").arg(axisName5D(axis), sample.endSettled.error);
                    return false;
                }
                return true;
            };

        for (const auto& cfg : configs) {
            if (m_axisCompCalCancel.load() || m_closeRequested) { cancelled = true; break; }
            const int axis = cfg.axisId;
            const AxisParams coarseMotion = makeMotion(axisParams[axis], cfg.coarseSpeedMmS);
            const AxisParams precisionMotion = makeMotion(axisParams[axis], cfg.precisionSpeedMmS);
            const AxisMoveDiagnosticSnapshot initial = captureAxisMoveDiagnosticSnapshot(controller, axis);
            if (!initial.valid) {
                terminalError = tr("%1轴开始前位置读取失败：%2").arg(axisName5D(axis), initial.error);
                break;
            }
            const std::uint64_t generation = initial.mposSetGeneration;

            const double positivePrimeStart = cfg.points.front() - preload;
            const double negativePrimeStart = cfg.points.back() + preload;
            const bool positiveFirst = std::abs(initial.dpos - positivePrimeStart)
                <= std::abs(initial.dpos - negativePrimeStart);

            const auto runPositiveSweep = [&](int cycle) -> bool {
                for (int k = 0; k < static_cast<int>(cfg.points.size()); ++k) {
                    const bool prime = (k == 0);
                    const double approachStart = prime ? (cfg.points[k] - preload) : cfg.points[k - 1];
                    if (!measurePoint(axis, +1, cycle, k, cfg.points[k], prime, approachStart,
                        cfg.coarseSpeedMmS, cfg.precisionSpeedMmS, coarseMotion, precisionMotion)) return false;
                    const auto snap = captureAxisMoveDiagnosticSnapshot(controller, axis);
                    if (snap.mposSetGeneration != generation) {
                        terminalError = tr("%1轴采表期间SetMpos generation从%2变为%3，已中止；不能用改变坐标基准的数据建表。")
                            .arg(axisName5D(axis)).arg(static_cast<qulonglong>(generation)).arg(static_cast<qulonglong>(snap.mposSetGeneration));
                        return false;
                    }
                }
                return true;
                };

            const auto runNegativeSweep = [&](int cycle) -> bool {
                int reverseIndex = 0;
                for (int k = static_cast<int>(cfg.points.size()) - 1; k >= 0; --k, ++reverseIndex) {
                    const bool prime = (k == static_cast<int>(cfg.points.size()) - 1);
                    const double approachStart = prime ? (cfg.points[k] + preload) : cfg.points[k + 1];
                    if (!measurePoint(axis, -1, cycle, reverseIndex, cfg.points[k], prime, approachStart,
                        cfg.coarseSpeedMmS, cfg.precisionSpeedMmS, coarseMotion, precisionMotion)) return false;
                    const auto snap = captureAxisMoveDiagnosticSnapshot(controller, axis);
                    if (snap.mposSetGeneration != generation) {
                        terminalError = tr("%1轴采表期间SetMpos generation改变，已中止。").arg(axisName5D(axis));
                        return false;
                    }
                }
                return true;
                };

            for (int cycle = 1; cycle <= repeats; ++cycle) {
                if (m_axisCompCalCancel.load() || m_closeRequested) { cancelled = true; break; }
                // Start from whichever direction-prime endpoint is closer to the current machine location.
                // This removes the old hard-coded "always go to the most-negative end first" behaviour.
                if (positiveFirst) {
                    if (!runPositiveSweep(cycle)) break;
                    if (!runNegativeSweep(cycle)) break;
                }
                else {
                    if (!runNegativeSweep(cycle)) break;
                    if (!runPositiveSweep(cycle)) break;
                }
                if (cancelled || !terminalError.isEmpty()) break;
            }

            if (cancelled || !terminalError.isEmpty()) break;

            QString restoreError;
            if (!controller->moveAbsRawDpos(axis, static_cast<float>(initial.dpos), &coarseMotion, &restoreError)) {
                terminalError = tr("%1轴采表后恢复启动位置失败：%2").arg(axisName5D(axis), restoreError);
                break;
            }
        }

        QMetaObject::invokeMethod(this, [this, cancelled, terminalError, completed, totalSamples, outputRoot]() {
            --m_axisWorkerCount;
            m_axisCompCalRunning = false;
            if (m_btnHomeAll) m_btnHomeAll->setEnabled(true);
            if (m_btnZeroAll) m_btnZeroAll->setEnabled(true);
            for (int axis = AxisX; axis <= AxisB; ++axis) if (auto* w = axisWidgetForId(axis)) w->setControlsEnabled(true);
            if (m_btnAxisMoveDiagAutoStart) m_btnAxisMoveDiagAutoStart->setEnabled(true);
            if (m_btnAxisMoveDiagManualStart) m_btnAxisMoveDiagManualStart->setEnabled(true);
            if (m_btnAxisMoveDiagManualEnd) m_btnAxisMoveDiagManualEnd->setEnabled(true);
            if (m_gcodeWidget) m_gcodeWidget->setEnabled(true);
            if (m_calibrationWidget) m_calibrationWidget->setEnabled(true);
            if (m_btnStartMerge) m_btnStartMerge->setEnabled(true);
            if (m_btnStartCapture) m_btnStartCapture->setEnabled(true);
            updateAxisCompensationSummaryTable();
            updateAxisCompensationCalibrationUi();
            updateAxisCompensationValidationUi();
            if (m_closeRequested) { tryFinishDeferredClose(); return; }

            if (cancelled) {
                if (m_lblAxisCompCalStatus) m_lblAxisCompCalStatus->setText(
                    tr("补偿表采集已取消：完成%1/%2个测量样本。可保存当前部分原始数据，但不建议作为正式补偿表使用。")
                    .arg(completed).arg(totalSamples));
            }
            else if (!terminalError.isEmpty()) {
                if (m_lblAxisCompCalStatus) m_lblAxisCompCalStatus->setText(
                    tr("补偿表采集中止：%1；完成%2/%3个样本。")
                    .arg(terminalError).arg(completed).arg(totalSamples));
                QMessageBox::warning(this, tr("XYZ补偿表采集中止"), terminalError);
            }
            else {
                if (m_progressAxisCompCal) m_progressAxisCompCal->setValue(100);
                QString savedDir, saveError;
                if (saveAxisCompensationCalibrationFiles(outputRoot, &savedDir, &saveError)) {
                    if (m_lblAxisCompCalStatus) m_lblAxisCompCalStatus->setText(
                        tr("补偿表采集完成：%1个样本。候选表已自动保存到 %2。"
                            "状态=CANDIDATE_UNVALIDATED；当前版本不会应用补偿。")
                        .arg(completed).arg(QDir::toNativeSeparators(savedDir)));
                }
                else {
                    if (m_lblAxisCompCalStatus) m_lblAxisCompCalStatus->setText(
                        tr("补偿表采集完成，但自动保存失败：%1。请点击“保存当前采表结果”。").arg(saveError));
                }
            }
            tryFinishDeferredClose();
            });
        });
}

void MainWindow::stopAxisCompensationCalibration()
{
    if (!m_axisCompCalRunning) return;
    m_axisCompCalCancel.store(true);
    if (m_btnAxisCompCalStop) m_btnAxisCompCalStop->setEnabled(false);
    if (m_zmc && m_zmc->isConnected()) {
        for (int axis = AxisX; axis <= AxisZ; ++axis) m_zmc->requestStopAxis(axis, false);
    }
    if (m_lblAxisCompCalStatus) m_lblAxisCompCalStatus->setText(tr("正在停止XYZ补偿表自动采集..."));
}


bool MainWindow::loadAxisCompensationValidationTable(const QString& filePath, QString* error)
{
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        if (error) *error = tr("无法打开候选补偿表：%1").arg(file.errorString());
        return false;
    }
    QTextStream in(&file);
    in.setEncoding(QStringConverter::Utf8);
    if (in.atEnd()) {
        if (error) *error = tr("候选补偿表为空");
        return false;
    }
    const QStringList header = parseSimpleCsvRecord(in.readLine());
    const auto col = [&](const QString& name) { return header.indexOf(name); };
    const int cAxis = col(QStringLiteral("axis"));
    const int cDirection = col(QStringLiteral("direction"));
    const int cNominal = col(QStringLiteral("nominal_command_dpos_mm"));
    const int cDesired = col(QStringLiteral("desired_mpos_mm"));
    const int cRequired = col(QStringLiteral("required_dpos_mm"));
    const int cCorrection = col(QStringLiteral("correction_required_dpos_minus_desired_mpos_mm"));
    const int cCorrectionStd = col(QStringLiteral("correction_stddev_mm"));
    const int cMposStd = col(QStringLiteral("mpos_stddev_mm"));
    const int cDrift = col(QStringLiteral("mean_post_idle_mpos_drift_mm"));
    const int cSamples = col(QStringLiteral("samples"));
    if (cAxis < 0 || cDirection < 0 || cDesired < 0 || cRequired < 0) {
        if (error) *error = tr("CSV不是兼容的候选补偿表：缺少 axis/direction/desired_mpos_mm/required_dpos_mm 列");
        return false;
    }

    QVector<AxisCompensationTableNode> nodes;
    int tableAxis = -1;
    int lineNo = 1;
    while (!in.atEnd()) {
        ++lineNo;
        const QString line = in.readLine().trimmed();
        if (line.isEmpty()) continue;
        const QStringList f = parseSimpleCsvRecord(line);
        const int maxRequiredCol = std::max({ cAxis, cDirection, cDesired, cRequired });
        if (f.size() <= maxRequiredCol) {
            if (error) *error = tr("候选补偿表第%1行列数不足").arg(lineNo);
            return false;
        }
        const QString axisText = f[cAxis].trimmed().toUpper();
        const int axis = axisText == QStringLiteral("X") ? AxisX
            : axisText == QStringLiteral("Y") ? AxisY
            : axisText == QStringLiteral("Z") ? AxisZ : -1;
        if (axis < AxisX || axis > AxisZ) {
            if (error) *error = tr("候选补偿表第%1行轴名无效：%2").arg(lineNo).arg(axisText);
            return false;
        }
        if (tableAxis < 0) tableAxis = axis;
        if (axis != tableAxis) {
            if (error) *error = tr("一个候选表只能包含一个线性轴，文件中同时出现%1和%2")
                .arg(axisName5D(tableAxis), axisName5D(axis));
            return false;
        }
        const QString directionText = f[cDirection].trimmed();
        const int direction = directionText == QStringLiteral("+") ? +1
            : directionText == QStringLiteral("-") ? -1 : 0;
        if (direction == 0) {
            if (error) *error = tr("候选补偿表第%1行方向无效：%2").arg(lineNo).arg(directionText);
            return false;
        }
        bool okDesired = false, okRequired = false;
        const double desired = f[cDesired].toDouble(&okDesired);
        const double required = f[cRequired].toDouble(&okRequired);
        if (!okDesired || !okRequired || !std::isfinite(desired) || !std::isfinite(required)) {
            if (error) *error = tr("候选补偿表第%1行 desired/required 数值无效").arg(lineNo);
            return false;
        }
        const auto optionalDouble = [&](int index) {
            if (index < 0 || index >= f.size()) return std::numeric_limits<double>::quiet_NaN();
            bool ok = false;
            const double v = f[index].toDouble(&ok);
            return ok && std::isfinite(v) ? v : std::numeric_limits<double>::quiet_NaN();
            };
        AxisCompensationTableNode n;
        n.axisId = axis;
        n.direction = direction;
        n.nominalDposMm = optionalDouble(cNominal);
        n.desiredMposMm = desired;
        n.requiredDposMm = required;
        n.correctionMm = optionalDouble(cCorrection);
        n.correctionStdDevMm = optionalDouble(cCorrectionStd);
        n.mposStdDevMm = optionalDouble(cMposStd);
        n.meanPostIdleMposDriftMm = optionalDouble(cDrift);
        if (cSamples >= 0 && cSamples < f.size()) {
            bool ok = false;
            n.sampleCount = f[cSamples].toInt(&ok);
            if (!ok) n.sampleCount = 0;
        }
        nodes.push_back(n);
    }
    if (nodes.isEmpty()) {
        if (error) *error = tr("候选补偿表没有有效节点");
        return false;
    }

    for (const int direction : {+1, -1}) {
        QVector<AxisCompensationTableNode> dirNodes;
        for (const auto& n : nodes) if (n.direction == direction) dirNodes.push_back(n);
        if (dirNodes.size() < 2) continue;
        std::sort(dirNodes.begin(), dirNodes.end(), [](const auto& a, const auto& b) {
            return a.desiredMposMm < b.desiredMposMm;
            });
        for (int i = 1; i < dirNodes.size(); ++i) {
            if (!(dirNodes[i].desiredMposMm > dirNodes[i - 1].desiredMposMm)) {
                if (error) *error = tr("%1轴%2方向 desired_mpos_mm 不是严格单调，不能安全插值")
                    .arg(axisName5D(tableAxis), direction > 0 ? QStringLiteral("+") : QStringLiteral("-"));
                return false;
            }
        }
    }

    m_axisCompValNodes = nodes;
    m_axisCompValAxis = tableAxis;
    m_axisCompValSourcePath = QFileInfo(filePath).absoluteFilePath();
    m_axisCompValRecords.clear();
    if (m_editAxisCompValTablePath) m_editAxisCompValTablePath->setText(QDir::toNativeSeparators(m_axisCompValSourcePath));

    // V27.6.34: recover both the motion profile and the table-acquisition strategy.
    // Auto validation selects legacy point-to-point for old tables and monotonic
    // PRIME-once sweep for V27.6.33+ tables whose manifest explicitly says so.
    m_axisCompValSourceMotionStrategy.clear();
    m_axisCompValAutoMethod = AxisCompValPointToPoint;
    const QDir manifestDir(QFileInfo(filePath).absolutePath());
    QString manifestPath = manifestDir.filePath(axisName5D(tableAxis) + QStringLiteral("_axis_compensation_manifest.json"));
    if (!QFileInfo::exists(manifestPath))manifestPath = manifestDir.filePath(QStringLiteral("axis_compensation_manifest.json"));
    QFile manifestFile(manifestPath);
    if (manifestFile.open(QIODevice::ReadOnly)) {
        const QJsonDocument doc = QJsonDocument::fromJson(manifestFile.readAll());
        const QJsonObject root = doc.object();
        const QString motionStrategy = root.value(QStringLiteral("motion_strategy")).toString();
        const QString samplingStrategy = root.value(QStringLiteral("sampling_strategy")).toString();
        m_axisCompValSourceMotionStrategy = !motionStrategy.isEmpty() ? motionStrategy : samplingStrategy;
        const QString strategyProbe = motionStrategy + QStringLiteral(" ") + samplingStrategy;
        if (strategyProbe.contains(QStringLiteral("monotonic"), Qt::CaseInsensitive)
            || strategyProbe.contains(QStringLiteral("direction-prime-once"), Qt::CaseInsensitive)) {
            m_axisCompValAutoMethod = AxisCompValMonotonicSweep;
        }

        const QJsonObject profiles = root.value(QStringLiteral("motion_profiles")).toObject();
        const QJsonObject mp = profiles.value(axisName5D(tableAxis)).toObject();
        if (!mp.isEmpty() && mp.value(QStringLiteral("qualified")).toBool(false)) {
            AxisCompensationSpeedProfile p;
            p.axisId = tableAxis;
            p.qualified = true;
            p.coarseSpeedMmS = mp.value(QStringLiteral("coarse_speed_mm_s")).toDouble(std::numeric_limits<double>::quiet_NaN());
            p.precisionSpeedMmS = mp.value(QStringLiteral("precision_final_approach_speed_mm_s")).toDouble(std::numeric_limits<double>::quiet_NaN());
            p.preloadMm = mp.value(QStringLiteral("final_approach_distance_mm")).toDouble(2.0);
            p.coarseP95DriftMm = mp.value(QStringLiteral("coarse_p95_post_idle_drift_mm")).toDouble(std::numeric_limits<double>::quiet_NaN());
            p.coarseRepeatabilityMm = mp.value(QStringLiteral("coarse_repeatability_rms_mm")).toDouble(std::numeric_limits<double>::quiet_NaN());
            p.precisionP95DriftMm = mp.value(QStringLiteral("precision_p95_post_idle_drift_mm")).toDouble(std::numeric_limits<double>::quiet_NaN());
            p.precisionRepeatabilityMm = mp.value(QStringLiteral("precision_repeatability_rms_mm")).toDouble(std::numeric_limits<double>::quiet_NaN());
            const QJsonObject grid = root.value(QStringLiteral("calibration_grid")).toObject().value(axisName5D(tableAxis)).toObject();
            p.rangeStartMm = grid.value(QStringLiteral("start_dpos_mm")).toDouble(std::numeric_limits<double>::quiet_NaN());
            p.rangeEndMm = grid.value(QStringLiteral("end_dpos_mm")).toDouble(std::numeric_limits<double>::quiet_NaN());
            p.status = QStringLiteral("QUALIFIED_FROM_MANIFEST");
            if (std::isfinite(p.coarseSpeedMmS) && std::isfinite(p.precisionSpeedMmS)) {
                m_axisCompSpeedProfiles[tableAxis] = p;
                if (m_spinAxisCompValPreloadMm && std::isfinite(p.preloadMm)) m_spinAxisCompValPreloadMm->setValue(p.preloadMm);
                updateAxisCompensationSpeedProfileTable();
            }
        }
    }
    else {
        m_axisCompValSourceMotionStrategy = QStringLiteral("manifest未声明采表策略；按旧式逐点预加载验证");
    }
    if (m_comboAxisCompValMethod) m_comboAxisCompValMethod->setCurrentIndex(0);
    updateAxisCompensationValidationSummary();
    updateAxisCompensationValidationUi();
    return true;
}

int MainWindow::resolvedAxisCompensationValidationMethod() const
{
    const int selected = m_comboAxisCompValMethod ? m_comboAxisCompValMethod->currentData().toInt() : AxisCompValAuto;
    if (selected == AxisCompValPointToPoint || selected == AxisCompValMonotonicSweep) return selected;
    return m_axisCompValAutoMethod;
}

QString MainWindow::axisCompensationValidationMethodLabel(int method) const
{
    if (method == AxisCompValMonotonicSweep) return tr("新式单向SWEEP：每个方向/循环/模式仅PRIME一次，随后连续单向通过off-grid验证点");
    return tr("旧式逐点预加载：每个RAW/COMP目标各自执行一次最终接近预加载");
}

bool MainWindow::interpolateAxisCompensationCommand(int direction,
    double targetMposMm,
    double& requiredDposMm,
    QString* error) const
{
    // A/B retains its axis-specific acquisition/approach protocol and RAW SDK
    // bypass. Only the pure MPOS -> DPOS interpolation is shared with production.
    mls::axiscomp::Table table; table.axisIndex = m_axisCompValAxis - AxisX;
    for (const auto& n : m_axisCompValNodes) {
        if (n.axisId != m_axisCompValAxis)continue;
        if ((n.direction != 1 && n.direction != -1) || !std::isfinite(n.desiredMposMm) || !std::isfinite(n.requiredDposMm)) {
            if (error)*error = tr("候选表存在非法方向或非有限数值"); return false;
        }
        table.directions[n.direction > 0 ? 0 : 1].push_back({ n.desiredMposMm,n.requiredDposMm });
    }
    for (auto& branch : table.directions)
        std::sort(branch.begin(), branch.end(), [](const auto& a, const auto& b) {return a.desiredMpos < b.desiredMpos; });
    std::string detail;
    const bool ok = mls::axiscomp::interpolate(table, direction, targetMposMm, requiredDposMm, detail);
    if (!ok && error)*error = QString::fromStdString(detail);
    return ok;
}

QVector<double> MainWindow::buildAxisCompensationValidationTargets(int directionSelection,
    int pointCount,
    double preloadMm,
    QString* error) const
{
    QVector<int> directions;
    if (directionSelection == 0) directions = { +1, -1 };
    else directions = { directionSelection };
    double commonLo = -std::numeric_limits<double>::infinity();
    double commonHi = std::numeric_limits<double>::infinity();
    QVector<AxisCompensationTableNode> reference;
    for (const int direction : directions) {
        QVector<AxisCompensationTableNode> dirNodes;
        for (const auto& n : m_axisCompValNodes) if (n.axisId == m_axisCompValAxis && n.direction == direction) dirNodes.push_back(n);
        std::sort(dirNodes.begin(), dirNodes.end(), [](const auto& a, const auto& b) { return a.desiredMposMm < b.desiredMposMm; });
        if (dirNodes.size() < 2) {
            if (error) *error = tr("候选表缺少%1方向节点").arg(direction > 0 ? QStringLiteral("+") : QStringLiteral("-"));
            return {};
        }
        commonLo = std::max(commonLo, dirNodes.front().desiredMposMm);
        commonHi = std::min(commonHi, dirNodes.back().desiredMposMm);
        if (reference.isEmpty()) reference = dirNodes;
    }
    if (!(commonLo < commonHi)) {
        if (error) *error = tr("所选方向的候选表没有共同MPOS有效区间");
        return {};
    }

    const auto axisParams = currentAxisParams5D();
    const double softLo = static_cast<double>(axisParams[m_axisCompValAxis].softLimitNeg);
    const double softHi = static_cast<double>(axisParams[m_axisCompValAxis].softLimitPos);
    QVector<double> candidates;
    for (int i = 0; i + 1 < reference.size(); ++i) {
        const double target = 0.5 * (reference[i].desiredMposMm + reference[i + 1].desiredMposMm);
        if (!(target > commonLo + 1e-9 && target < commonHi - 1e-9)) continue;
        bool valid = true;
        for (const int direction : directions) {
            double command = std::numeric_limits<double>::quiet_NaN();
            QString interpolationError;
            if (!interpolateAxisCompensationCommand(direction, target, command, &interpolationError)) { valid = false; break; }
            const double rawCommand = target;
            const double lowCommand = std::min(rawCommand, command);
            const double highCommand = std::max(rawCommand, command);
            const double preloadPosition = direction > 0 ? lowCommand - preloadMm : highCommand + preloadMm;
            if (lowCommand < softLo - 1e-9 || highCommand > softHi + 1e-9
                || preloadPosition < softLo - 1e-9 || preloadPosition > softHi + 1e-9) {
                valid = false;
                break;
            }
        }
        if (valid) candidates.push_back(target);
    }
    if (candidates.size() < 2) {
        if (error) *error = tr("候选表有效范围在当前软限位/预加载条件下没有足够的独立验证中点");
        return {};
    }

    const int candidateCount = static_cast<int>(candidates.size());
    const int n = std::min(pointCount, candidateCount);
    QVector<double> selected;
    selected.reserve(n);
    int lastIndex = -1;
    for (int i = 0; i < n; ++i) {
        int index = static_cast<int>(std::floor((static_cast<double>(i) + 0.5) * candidates.size() / n));
        index = std::clamp(index, 0, candidateCount - 1);
        if (index <= lastIndex) index = std::min(lastIndex + 1, candidateCount - 1);
        if (index > lastIndex) {
            selected.push_back(candidates[index]);
            lastIndex = index;
        }
    }
    return selected;
}

void MainWindow::updateAxisCompensationValidationUi()
{
    const bool loaded = m_axisCompValAxis >= AxisX && m_axisCompValAxis <= AxisZ && !m_axisCompValNodes.isEmpty();
    const bool enabled = !m_axisCompValRunning && !m_axisCompSpeedRunning && !m_axisCompCalRunning && !m_axisMoveDiagAutoRunning;
    if (m_btnAxisCompValLoad) m_btnAxisCompValLoad->setEnabled(enabled);
    if (m_comboAxisCompValMethod) m_comboAxisCompValMethod->setEnabled(enabled && loaded);
    if (m_comboAxisCompValDirection) m_comboAxisCompValDirection->setEnabled(enabled && loaded);
    if (m_spinAxisCompValPoints) m_spinAxisCompValPoints->setEnabled(enabled && loaded);
    if (m_spinAxisCompValRepeats) m_spinAxisCompValRepeats->setEnabled(enabled && loaded);
    if (m_spinAxisCompValSettleMs) m_spinAxisCompValSettleMs->setEnabled(enabled && loaded);
    if (m_spinAxisCompValPreloadMm) m_spinAxisCompValPreloadMm->setEnabled(enabled && loaded);
    if (m_editAxisCompValOutputDir) m_editAxisCompValOutputDir->setEnabled(enabled);
    if (m_btnAxisCompValBrowseDir) m_btnAxisCompValBrowseDir->setEnabled(enabled);
    if (m_btnAxisCompValStart) m_btnAxisCompValStart->setEnabled(enabled && loaded);
    if (m_btnAxisCompValStop) m_btnAxisCompValStop->setEnabled(m_axisCompValRunning);
    if (m_btnAxisCompValSave) m_btnAxisCompValSave->setEnabled(enabled && !m_axisCompValRecords.isEmpty());

    if (m_lblAxisCompValTableInfo) {
        if (!loaded) {
            m_lblAxisCompValTableInfo->setText(tr("尚未加载候选补偿表。"));
        }
        else {
            QStringList pieces;
            for (const int direction : {+1, -1}) {
                QVector<double> values;
                for (const auto& n : m_axisCompValNodes) if (n.direction == direction) values.push_back(n.desiredMposMm);
                std::sort(values.begin(), values.end());
                if (!values.isEmpty()) {
                    pieces << tr("%1:%2节点 MPOS[%3,%4]")
                        .arg(direction > 0 ? QStringLiteral("+") : QStringLiteral("-"))
                        .arg(values.size()).arg(values.front(), 0, 'f', 3).arg(values.back(), 0, 'f', 3);
                }
            }
            const int resolved = resolvedAxisCompensationValidationMethod();
            const QString sourceStrategy = m_axisCompValSourceMotionStrategy.isEmpty()
                ? tr("manifest未声明采表策略") : m_axisCompValSourceMotionStrategy;
            m_lblAxisCompValTableInfo->setText(tr("已加载%1轴候选表：%2。\n自动验证方法=%3。\n采表策略=%4。\n来源=%5")
                .arg(axisName5D(m_axisCompValAxis))
                .arg(pieces.join(QStringLiteral("；")))
                .arg(axisCompensationValidationMethodLabel(resolved))
                .arg(sourceStrategy)
                .arg(QDir::toNativeSeparators(m_axisCompValSourcePath)));
        }
    }
}

void MainWindow::appendAxisCompensationValidationRecord(const AxisCompensationValidationRecord& record)
{
    m_axisCompValRecords.push_back(record);
    if (m_lblAxisCompValStatus && record.endSettled.valid) {
        const double errorMm = record.endSettled.mpos - record.targetMposMm;
        const double driftMm = record.endIdle.valid ? record.endSettled.mpos - record.endIdle.mpos
            : std::numeric_limits<double>::quiet_NaN();
        m_lblAxisCompValStatus->setText(tr("已完成%1条验证；最新 %2 %3 target=%4，commandDPOS=%5，finalMPOS=%6，误差=%7 µm，IDLE后漂移=%8 µm。")
            .arg(m_axisCompValRecords.size())
            .arg(record.direction > 0 ? QStringLiteral("+") : QStringLiteral("-"), record.mode)
            .arg(record.targetMposMm, 0, 'f', 6).arg(record.commandedDposMm, 0, 'f', 6).arg(record.endSettled.mpos, 0, 'f', 6)
            .arg(errorMm * 1000.0, 0, 'f', 3).arg(driftMm * 1000.0, 0, 'f', 3));
    }
    updateAxisCompensationValidationSummary();
}

MainWindow::AxisCompensationValidationStats MainWindow::computeAxisCompensationValidationStats(int direction,
    const QString& mode) const
{
    AxisCompensationValidationStats stats;
    QVector<double> errors;
    for (const auto& r : m_axisCompValRecords) {
        if (!r.moveOk || !r.endSettled.valid || r.mode != mode) continue;
        if (direction != 0 && r.direction != direction) continue;
        const double e = r.endSettled.mpos - r.targetMposMm;
        if (std::isfinite(e)) errors.push_back(e);
    }
    stats.sampleCount = errors.size();
    if (errors.isEmpty()) return stats;
    double sum = 0.0, sumSq = 0.0, sumAbs = 0.0, maxAbs = 0.0;
    QVector<double> absErrors;
    absErrors.reserve(errors.size());
    for (const double e : errors) {
        const double a = std::abs(e);
        sum += e;
        sumSq += e * e;
        sumAbs += a;
        maxAbs = std::max(maxAbs, a);
        absErrors.push_back(a);
    }
    std::sort(absErrors.begin(), absErrors.end());
    const int p95Index = std::clamp(static_cast<int>(std::ceil(0.95 * static_cast<double>(absErrors.size()))) - 1,
        0, static_cast<int>(absErrors.size()) - 1);
    stats.meanErrorMm = sum / errors.size();
    stats.rmsMm = std::sqrt(sumSq / errors.size());
    stats.maeMm = sumAbs / errors.size();
    stats.p95AbsMm = absErrors[p95Index];
    stats.maxAbsMm = maxAbs;
    return stats;
}

void MainWindow::updateAxisCompensationValidationSummary()
{
    if (!m_axisCompValSummaryTable) return;
    m_axisCompValSummaryTable->setRowCount(0);
    const auto addRow = [&](const QString& label, int direction) {
        const auto raw = computeAxisCompensationValidationStats(direction, QStringLiteral("RAW"));
        const auto comp = computeAxisCompensationValidationStats(direction, QStringLiteral("COMPENSATED"));
        if (raw.sampleCount == 0 && comp.sampleCount == 0) return;
        const int row = m_axisCompValSummaryTable->rowCount();
        m_axisCompValSummaryTable->insertRow(row);
        const double improvement = std::isfinite(raw.rmsMm) && raw.rmsMm > 1e-12 && std::isfinite(comp.rmsMm)
            ? 100.0 * (1.0 - comp.rmsMm / raw.rmsMm) : std::numeric_limits<double>::quiet_NaN();
        const QStringList values = {
            label,
            std::isfinite(raw.rmsMm) ? QString::number(raw.rmsMm * 1000.0, 'f', 3) : QString(),
            std::isfinite(comp.rmsMm) ? QString::number(comp.rmsMm * 1000.0, 'f', 3) : QString(),
            std::isfinite(raw.p95AbsMm) ? QString::number(raw.p95AbsMm * 1000.0, 'f', 3) : QString(),
            std::isfinite(comp.p95AbsMm) ? QString::number(comp.p95AbsMm * 1000.0, 'f', 3) : QString(),
            std::isfinite(raw.maxAbsMm) ? QString::number(raw.maxAbsMm * 1000.0, 'f', 3) : QString(),
            std::isfinite(comp.maxAbsMm) ? QString::number(comp.maxAbsMm * 1000.0, 'f', 3) : QString(),
            std::isfinite(comp.meanErrorMm) ? QString::number(comp.meanErrorMm * 1000.0, 'f', 3) : QString(),
            std::isfinite(improvement) ? QString::number(improvement, 'f', 1) + QStringLiteral("%") : QString()
        };
        for (int col = 0; col < values.size(); ++col) {
            auto* item = new QTableWidgetItem(values[col]);
            item->setTextAlignment(Qt::AlignCenter);
            m_axisCompValSummaryTable->setItem(row, col, item);
        }
        };
    addRow(QStringLiteral("+"), +1);
    addRow(QStringLiteral("-"), -1);
    addRow(QStringLiteral("ALL"), 0);
}

bool MainWindow::saveAxisCompensationValidationFiles(const QString& outputRoot,
    QString* savedRunDirectory,
    QString* error) const
{
    if (m_axisCompValRecords.isEmpty()) {
        if (error) *error = tr("当前没有A/B验证记录");
        return false;
    }
    const QString root = outputRoot.trimmed().isEmpty()
        ? QDir(QCoreApplication::applicationDirPath()).filePath(QStringLiteral("calibration/axis_compensation_validation"))
        : outputRoot.trimmed();
    QDir rootDir;
    if (!rootDir.mkpath(root)) {
        if (error) *error = tr("无法创建验证结果根目录：%1").arg(root);
        return false;
    }
    const QString runName = QStringLiteral("axis_comp_validation_%1")
        .arg(QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss_zzz")));
    const QString runDirPath = QDir(root).filePath(runName);
    if (!rootDir.mkpath(runDirPath)) {
        if (error) *error = tr("无法创建本次验证目录：%1").arg(runDirPath);
        return false;
    }
    QDir runDir(runDirPath);
    const auto csvQuote = [](QString value) {
        value.replace('"', QStringLiteral("\"\""));
        return QStringLiteral("\"") + value + QStringLiteral("\"");
        };
    const auto num = [](double v) { return std::isfinite(v) ? QString::number(v, 'f', 9) : QString(); };

    QFile detailed(runDir.filePath(QStringLiteral("axis_compensation_ab_validation.csv")));
    if (!detailed.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate)) {
        if (error) *error = tr("无法写入验证CSV：%1").arg(detailed.errorString());
        return false;
    }
    QTextStream out(&detailed);
    out.setEncoding(QStringConverter::Utf8);
    out << "record,axis,direction,cycle,target_index,mode,validation_method,approach_mode,target_mpos_mm,commanded_dpos_mm,preload_dpos_mm,coarse_speed_mm_s,precision_speed_mm_s,settle_ms,"
        "idle_utc,idle_dpos_mm,idle_mpos_mm,idle_encoder,idle_error_mpos_minus_target_mm,"
        "settled_utc,settled_dpos_mm,settled_mpos_mm,settled_encoder,settled_error_mpos_minus_target_mm,"
        "post_idle_mpos_drift_mm,linear_referenced,setmpos_generation,datum_input,datum_raw,move_ok,status\n";
    int recordIndex = 0;
    for (const auto& r : m_axisCompValRecords) {
        const double idleError = r.endIdle.valid ? r.endIdle.mpos - r.targetMposMm : std::numeric_limits<double>::quiet_NaN();
        const double settledError = r.endSettled.valid ? r.endSettled.mpos - r.targetMposMm : std::numeric_limits<double>::quiet_NaN();
        const double drift = r.endIdle.valid && r.endSettled.valid ? r.endSettled.mpos - r.endIdle.mpos : std::numeric_limits<double>::quiet_NaN();
        out << ++recordIndex << ',' << csvQuote(axisName5D(r.axisId)) << ','
            << csvQuote(r.direction > 0 ? QStringLiteral("+") : QStringLiteral("-")) << ','
            << r.cycle << ',' << r.targetIndex << ',' << csvQuote(r.mode) << ','
            << csvQuote(r.validationMethod) << ',' << csvQuote(r.approachMode) << ','
            << num(r.targetMposMm) << ',' << num(r.commandedDposMm) << ',' << num(r.preloadDposMm) << ','
            << num(r.coarseSpeedMmS) << ',' << num(r.precisionSpeedMmS) << ',' << r.settleMs << ','
            << csvQuote(r.endIdle.utc) << ',' << num(r.endIdle.dpos) << ',' << num(r.endIdle.mpos) << ','
            << (r.endIdle.encoderValid ? num(r.endIdle.encoder) : QString()) << ',' << num(idleError) << ','
            << csvQuote(r.endSettled.utc) << ',' << num(r.endSettled.dpos) << ',' << num(r.endSettled.mpos) << ','
            << (r.endSettled.encoderValid ? num(r.endSettled.encoder) : QString()) << ',' << num(settledError) << ','
            << num(drift) << ',' << (r.endSettled.linearReferenced ? 1 : 0) << ','
            << static_cast<qulonglong>(r.endSettled.mposSetGeneration) << ',' << r.endSettled.datumInput << ',' << r.endSettled.datumRaw << ','
            << (r.moveOk ? 1 : 0) << ',' << csvQuote(r.error) << '\n';
    }
    detailed.close();

    QFile summaryCsv(runDir.filePath(QStringLiteral("axis_compensation_ab_summary.csv")));
    if (!summaryCsv.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate)) {
        if (error) *error = tr("无法写入验证摘要CSV：%1").arg(summaryCsv.errorString());
        return false;
    }
    QTextStream summaryOut(&summaryCsv);
    summaryOut.setEncoding(QStringConverter::Utf8);
    summaryOut << "direction,mode,samples,mean_error_mm,rms_mm,mae_mm,p95_abs_mm,max_abs_mm\n";
    QJsonObject summaryJson;
    for (const auto& entry : QVector<QPair<QString, int>>{ {QStringLiteral("+"),+1},{QStringLiteral("-"),-1},{QStringLiteral("ALL"),0} }) {
        QJsonObject directionObj;
        for (const QString mode : {QStringLiteral("RAW"), QStringLiteral("COMPENSATED")}) {
            const auto st = computeAxisCompensationValidationStats(entry.second, mode);
            if (st.sampleCount == 0) continue;
            summaryOut << csvQuote(entry.first) << ',' << csvQuote(mode) << ',' << st.sampleCount << ','
                << num(st.meanErrorMm) << ',' << num(st.rmsMm) << ',' << num(st.maeMm) << ','
                << num(st.p95AbsMm) << ',' << num(st.maxAbsMm) << '\n';
            QJsonObject o;
            o[QStringLiteral("samples")] = st.sampleCount;
            o[QStringLiteral("mean_error_mm")] = st.meanErrorMm;
            o[QStringLiteral("rms_mm")] = st.rmsMm;
            o[QStringLiteral("mae_mm")] = st.maeMm;
            o[QStringLiteral("p95_abs_mm")] = st.p95AbsMm;
            o[QStringLiteral("max_abs_mm")] = st.maxAbsMm;
            directionObj[mode] = o;
        }
        if (!directionObj.isEmpty()) summaryJson[entry.first] = directionObj;
    }
    summaryCsv.close();

    QJsonObject manifest;
    manifest[QStringLiteral("format")] = QStringLiteral("MotionLaserSync.AxisPositionCompensationABValidation");
    manifest[QStringLiteral("version")] = 2;
    manifest[QStringLiteral("software_release")] = QStringLiteral("V27.6.35");
    manifest[QStringLiteral("created_utc")] = QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs);
    manifest[QStringLiteral("candidate_table")] = m_axisCompValSourcePath;
    manifest[QStringLiteral("axis")] = axisName5D(m_axisCompValAxis);
    manifest[QStringLiteral("runtime_global_compensation_enabled")] = false;
    manifest[QStringLiteral("mpos_trim_used")] = false;
    manifest[QStringLiteral("validation_method")] = m_axisCompValLastResolvedMethod == AxisCompValMonotonicSweep
        ? QStringLiteral("MONOTONIC_SWEEP") : QStringLiteral("POINT_TO_POINT_PRELOAD");
    manifest[QStringLiteral("validation_method_auto_recommendation")] = m_axisCompValAutoMethod == AxisCompValMonotonicSweep
        ? QStringLiteral("MONOTONIC_SWEEP") : QStringLiteral("POINT_TO_POINT_PRELOAD");
    manifest[QStringLiteral("candidate_sampling_strategy_detected")] = m_axisCompValSourceMotionStrategy;
    manifest[QStringLiteral("validation_motion_strategy")] = m_axisCompValLastResolvedMethod == AxisCompValMonotonicSweep
        ? QStringLiteral("each direction/cycle/mode: one coarse preload PRIME, precision move to first off-grid target, then all remaining off-grid targets reached monotonically with no per-point reverse preload; RAW and COMPENSATED are separate whole sweeps with cycle-wise order alternation")
        : QStringLiteral("legacy independent point-to-point: every RAW/COMPENSATED target gets its own preload and precision final approach");
    manifest[QStringLiteral("comparison")] = QStringLiteral("RAW: DPOS command = requested MPOS target; COMPENSATED: table interpolation desired MPOS -> required DPOS");
    if (m_axisCompValAxis >= AxisX && m_axisCompValAxis <= AxisZ) {
        const auto& p = m_axisCompSpeedProfiles[m_axisCompValAxis];
        if (p.qualified) {
            QJsonObject mp;
            mp[QStringLiteral("coarse_speed_mm_s")] = p.coarseSpeedMmS;
            mp[QStringLiteral("precision_final_approach_speed_mm_s")] = p.precisionSpeedMmS;
            mp[QStringLiteral("final_approach_distance_mm")] = p.preloadMm;
            manifest[QStringLiteral("validation_motion_profile")] = mp;
        }
    }
    manifest[QStringLiteral("summary")] = summaryJson;
    const auto overallRaw = computeAxisCompensationValidationStats(0, QStringLiteral("RAW"));
    const auto overallComp = computeAxisCompensationValidationStats(0, QStringLiteral("COMPENSATED"));
    const bool engineeringTarget = overallComp.sampleCount > 0
        && overallComp.rmsMm <= 0.003 && overallComp.p95AbsMm <= 0.005 && overallComp.maxAbsMm <= 0.010;
    manifest[QStringLiteral("meets_initial_engineering_target")] = engineeringTarget;
    manifest[QStringLiteral("initial_engineering_target_note")] = QStringLiteral("Informational only: compensated RMS<=0.003 mm, P95<=0.005 mm, max<=0.010 mm. Does not automatically enable runtime compensation.");

    QFile manifestFile(runDir.filePath(QStringLiteral("axis_compensation_ab_validation_manifest.json")));
    if (!manifestFile.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        if (error) *error = tr("无法写入验证manifest：%1").arg(manifestFile.errorString());
        return false;
    }
    manifestFile.write(QJsonDocument(manifest).toJson(QJsonDocument::Indented));
    manifestFile.close();
    if (savedRunDirectory) *savedRunDirectory = runDirPath;
    return true;
}

void MainWindow::saveAxisCompensationValidationFilesInteractive()
{
    if (m_axisCompValRunning) {
        QMessageBox::information(this, tr("保存A/B验证"), tr("验证仍在运行，请先完成或停止。"));
        return;
    }
    const QString root = m_editAxisCompValOutputDir ? m_editAxisCompValOutputDir->text() : QString();
    QString runDir, err;
    if (!saveAxisCompensationValidationFiles(root, &runDir, &err)) {
        QMessageBox::warning(this, tr("保存A/B验证失败"), err);
        return;
    }
    QMessageBox::information(this, tr("A/B验证已保存"), tr("验证结果已保存到：\n%1").arg(QDir::toNativeSeparators(runDir)));
}

void MainWindow::startAxisCompensationValidation()
{
    if (m_zmc && m_zmc->axisCompensationEnabled()) {
        QMessageBox::warning(this, tr("RAW 采集/诊断/验证"),
            tr("请先在 XYZ 运行补偿面板禁用运行补偿，再启动 RAW 采集或独立 A/B 验证。"));
        return;
    }
    if (m_axisCompValRunning) return;
    if (m_axisCompSpeedRunning || m_axisCompCalRunning || m_axisMoveDiagAutoRunning) {
        QMessageBox::information(this, tr("A/B验证"), tr("采表测速/补偿表采集/DPOS-MPOS自动诊断仍在运行，请先停止。"));
        return;
    }
    if (m_axisCompValAxis < AxisX || m_axisCompValAxis > AxisZ || m_axisCompValNodes.isEmpty()) {
        QMessageBox::warning(this, tr("A/B验证"), tr("请先加载一个X/Y/Z候选补偿表。"));
        return;
    }
    if (!m_zmc || !m_zmc->isConnected()) {
        QMessageBox::warning(this, tr("A/B验证"), tr("请先连接控制器。"));
        return;
    }
    if ((m_gcodeExecutor && (m_gcodeExecutor->isRunning() || m_gcodeExecutor->isPaused()))
        || (m_syncManager && m_syncManager->isRunning()) || m_jointMoveInProgress || m_sphereStudyAutoActive) {
        QMessageBox::information(this, tr("A/B验证"), tr("当前有G代码/同步采集/五轴定位/自动球研究任务运行，请先完全停止。"));
        return;
    }
    for (int axisId = AxisX; axisId <= AxisB; ++axisId) {
        if (m_zmc->isAxisBusy(axisId)) {
            QMessageBox::information(this, tr("A/B验证"), tr("当前仍有轴运动任务，请全部停稳后再开始。"));
            return;
        }
    }
    if (!m_zmc->isLinearMposReferenced(m_axisCompValAxis)) {
        QMessageBox::warning(this, tr("A/B验证"), tr("%1轴尚未正规回零并建立MPOS机床零点。候选补偿表验证必须在回零后进行。")
            .arg(axisName5D(m_axisCompValAxis)));
        return;
    }

    const int validationMethod = resolvedAxisCompensationValidationMethod();
    m_axisCompValLastResolvedMethod = validationMethod;
    const int directionSelection = m_comboAxisCompValDirection ? m_comboAxisCompValDirection->currentData().toInt() : 0;
    const int pointCount = m_spinAxisCompValPoints ? m_spinAxisCompValPoints->value() : 36;
    const int repeats = m_spinAxisCompValRepeats ? m_spinAxisCompValRepeats->value() : 3;
    const int settleMs = m_spinAxisCompValSettleMs ? m_spinAxisCompValSettleMs->value() : 2000;
    const double preload = m_spinAxisCompValPreloadMm ? m_spinAxisCompValPreloadMm->value() : 2.0;
    QString targetError;
    const QVector<double> targets = buildAxisCompensationValidationTargets(directionSelection, pointCount, preload, &targetError);
    if (targets.isEmpty()) {
        QMessageBox::warning(this, tr("A/B验证"), targetError);
        return;
    }
    QVector<int> directions = directionSelection == 0 ? QVector<int>{+1, -1} : QVector<int>{ directionSelection };
    const auto& validationSpeedProfile = m_axisCompSpeedProfiles[m_axisCompValAxis];
    const bool useQualifiedTwoStageSpeed = validationSpeedProfile.qualified
        && std::isfinite(validationSpeedProfile.coarseSpeedMmS)
        && std::isfinite(validationSpeedProfile.precisionSpeedMmS)
        && std::abs(validationSpeedProfile.preloadMm - preload) <= 1e-6;
    const int totalRecords = static_cast<int>(targets.size()) * static_cast<int>(directions.size()) * repeats * 2;
    const double settleMinutes = static_cast<double>(totalRecords) * settleMs / 60000.0;
    const QString methodText = axisCompensationValidationMethodLabel(validationMethod);
    const QString methodSelectionText = (m_comboAxisCompValMethod && m_comboAxisCompValMethod->currentData().toInt() == AxisCompValAuto)
        ? tr("自动识别 → %1").arg(methodText) : methodText;
    const QString executionText = validationMethod == AxisCompValMonotonicSweep
        ? tr("每个方向/循环分别执行整条RAW sweep和整条COMPENSATED sweep；每条sweep只在首点PRIME一次，随后不反向、不逐点preload；奇偶循环交换整条RAW/COMP顺序。")
        : tr("每个验证目标分别执行RAW与COMPENSATED；每次目标都重新做最终接近preload；奇偶循环交替A/B顺序。 ");
    const QString confirm = tr(
        "将开始%1轴候选补偿表 A/B 独立验证：\n"
        "• 验证方法=%10；\n"
        "• 验证点=%2个，取相邻建表节点中点，不与原建表节点重合；\n"
        "• 方向=%3；重复=%4；PRIME/最终接近预加载=%5 mm；\n"
        "• 运动速度=%9；\n"
        "• %11\n"
        "• RAW: DPOS命令=目标MPOS；COMPENSATED: 仅本次测试查表插值required DPOS；\n"
        "• 两种模式都IDLE后再等待%6 ms，不做MPOS二次trim；\n"
        "• 共%7条测量记录，仅稳定等待约%8 min；\n"
        "• 本验证绝不会打开全局补偿，也不会修改G-code/RTCP/标定运动。\n\n"
        "请确认候选表属于当前机床/当前回零基准，并有人监护急停。")
        .arg(axisName5D(m_axisCompValAxis)).arg(targets.size())
        .arg(directionSelection == 0 ? tr("正向+反向") : (directionSelection > 0 ? tr("正向") : tr("反向")))
        .arg(repeats).arg(preload, 0, 'f', 3).arg(settleMs).arg(totalRecords).arg(settleMinutes, 0, 'f', 1)
        .arg(useQualifiedTwoStageSpeed
            ? tr("已认证两阶段：自动定位 %1 mm/s + 精密 %2 mm/s")
            .arg(validationSpeedProfile.coarseSpeedMmS, 0, 'f', 3).arg(validationSpeedProfile.precisionSpeedMmS, 0, 'f', 3)
            : tr("当前轴参数单速度（候选表manifest没有匹配的两阶段速度配置）"))
        .arg(methodSelectionText).arg(executionText);
    if (QMessageBox::warning(this, tr("确认A/B验证"), confirm, QMessageBox::Yes | QMessageBox::No, QMessageBox::No) != QMessageBox::Yes) return;

    m_axisCompValRecords.clear();
    if (m_axisCompValSummaryTable) m_axisCompValSummaryTable->setRowCount(0);
    m_axisCompValRunning = true;
    m_axisCompValCancel.store(false);
    if (m_progressAxisCompVal) m_progressAxisCompVal->setValue(0);
    updateAxisCompensationValidationUi();
    updateAxisCompensationCalibrationUi();
    if (m_btnHomeAll) m_btnHomeAll->setEnabled(false);
    if (m_btnZeroAll) m_btnZeroAll->setEnabled(false);
    if (m_btnAxisMoveDiagAutoStart) m_btnAxisMoveDiagAutoStart->setEnabled(false);
    if (m_btnAxisCompCalStart) m_btnAxisCompCalStart->setEnabled(false);
    if (m_gcodeWidget) m_gcodeWidget->setEnabled(false);
    if (m_calibrationWidget) m_calibrationWidget->setEnabled(false);
    if (m_btnStartMerge) m_btnStartMerge->setEnabled(false);
    if (m_btnStartCapture) m_btnStartCapture->setEnabled(false);
    for (int axisId = AxisX; axisId <= AxisB; ++axisId) if (auto* w = axisWidgetForId(axisId)) w->setControlsEnabled(false);
    if (m_lblAxisCompValStatus) m_lblAxisCompValStatus->setText(tr("A/B验证运行中：%1，共%2条测量记录...").arg(methodText).arg(totalRecords));

    ++m_axisWorkerCount;
    ZmcController* controller = m_zmc.get();
    const auto axisParams = currentAxisParams5D();
    const int axis = m_axisCompValAxis;
    const auto makeValidationMotion = [](const AxisParams& base, double speed) {
        AxisParams m = base;
        m.speed = static_cast<float>(std::max(0.001, speed));
        m.lspeed = std::min(m.lspeed, m.speed);
        return m;
        };
    const AxisParams defaultMotion = axisParams[axis];
    const AxisParams coarseMotion = useQualifiedTwoStageSpeed
        ? makeValidationMotion(defaultMotion, validationSpeedProfile.coarseSpeedMmS) : defaultMotion;
    const AxisParams precisionMotion = useQualifiedTwoStageSpeed
        ? makeValidationMotion(defaultMotion, validationSpeedProfile.precisionSpeedMmS) : defaultMotion;
    const QString outputRoot = m_editAxisCompValOutputDir ? m_editAxisCompValOutputDir->text() : QString();
    (void)QtConcurrent::run([this, controller, coarseMotion, precisionMotion, axis, targets, directions, repeats, settleMs, preload, totalRecords, outputRoot, validationMethod]() {
        bool cancelled = false;
        QString terminalError;
        int completed = 0;
        const AxisMoveDiagnosticSnapshot initial = captureAxisMoveDiagnosticSnapshot(controller, axis);
        if (!initial.valid) terminalError = tr("%1轴验证开始前位置读取失败：%2").arg(axisName5D(axis), initial.error);
        const std::uint64_t generation = initial.mposSetGeneration;
        const QString methodCode = validationMethod == AxisCompValMonotonicSweep
            ? QStringLiteral("MONOTONIC_SWEEP") : QStringLiteral("POINT_TO_POINT_PRELOAD");

        const auto moveOnly = [&](double target, const AxisParams& motion, QString& err) -> bool {
            if (m_axisCompValCancel.load() || m_closeRequested) { err = tr("用户取消"); return false; }
            return controller->moveAbsRawDpos(axis, static_cast<float>(target), &motion, &err);
            };
        const auto publish = [this, totalRecords](const AxisCompensationValidationRecord& r, int done) {
            QMetaObject::invokeMethod(this, [this, r, done, totalRecords]() {
                if (m_closeRequested) return;
                appendAxisCompensationValidationRecord(r);
                if (m_progressAxisCompVal) m_progressAxisCompVal->setValue(totalRecords > 0 ? 100 * done / totalRecords : 0);
                });
            };
        const auto finalizeMeasuredRecord = [&](AxisCompensationValidationRecord& r) -> bool {
            r.endIdle = captureAxisMoveDiagnosticSnapshot(controller, axis);
            if (r.moveOk && settleMs > 0 && !m_axisCompValCancel.load()) QThread::msleep(static_cast<unsigned long>(settleMs));
            r.endSettled = captureAxisMoveDiagnosticSnapshot(controller, axis);
            ++completed;
            publish(r, completed);
            if (!r.moveOk) {
                if (m_axisCompValCancel.load() || m_closeRequested) cancelled = true;
                else terminalError = r.error.isEmpty() ? tr("%1轴A/B验证运动失败").arg(axisName5D(axis)) : r.error;
                return false;
            }
            if (!r.endSettled.valid) {
                terminalError = tr("%1轴A/B验证位置读取失败：%2").arg(axisName5D(axis), r.endSettled.error);
                return false;
            }
            if (r.endSettled.mposSetGeneration != generation) {
                terminalError = tr("%1轴A/B验证期间SetMpos generation从%2变为%3，已中止；候选表不能跨坐标基准验证。")
                    .arg(axisName5D(axis)).arg(static_cast<qulonglong>(generation)).arg(static_cast<qulonglong>(r.endSettled.mposSetGeneration));
                return false;
            }
            return true;
            };

        const auto commandFor = [&](int direction, double target, const QString& mode, double& command, QString& err) -> bool {
            command = target;
            if (mode == QStringLiteral("COMPENSATED")) {
                return interpolateAxisCompensationCommand(direction, target, command, &err);
            }
            return true;
            };

        const auto measureLegacyPoint = [&](int direction, int cycle, int targetIndex, double target, const QString& mode) -> bool {
            AxisCompensationValidationRecord r;
            r.axisId = axis;
            r.direction = direction;
            r.cycle = cycle;
            r.targetIndex = targetIndex;
            r.mode = mode;
            r.validationMethod = methodCode;
            r.approachMode = QStringLiteral("POINT");
            r.targetMposMm = target;
            r.settleMs = settleMs;
            QString err;
            double command = target;
            if (!commandFor(direction, target, mode, command, err)) {
                r.error = err;
                ++completed;
                publish(r, completed);
                terminalError = err;
                return false;
            }
            r.commandedDposMm = command;
            r.preloadDposMm = direction > 0 ? command - preload : command + preload;
            r.coarseSpeedMmS = coarseMotion.speed;
            r.precisionSpeedMmS = precisionMotion.speed;
            if (!moveOnly(r.preloadDposMm, coarseMotion, err)) {
                if (m_axisCompValCancel.load() || m_closeRequested) cancelled = true;
                else terminalError = tr("%1轴%2 %3逐点预加载失败：%4")
                    .arg(axisName5D(axis), direction > 0 ? QStringLiteral("+") : QStringLiteral("-"), mode, err);
                return false;
            }
            r.moveOk = moveOnly(command, precisionMotion, err);
            r.error = err;
            return finalizeMeasuredRecord(r);
            };

        const auto runMonotonicSweep = [&](int direction, int cycle, const QVector<double>& orderedTargets, const QString& mode) -> bool {
            if (orderedTargets.isEmpty()) return true;
            QVector<double> commands;
            commands.reserve(orderedTargets.size());
            for (const double target : orderedTargets) {
                double command = target;
                QString err;
                if (!commandFor(direction, target, mode, command, err)) {
                    terminalError = err;
                    return false;
                }
                commands.push_back(command);
            }
            for (int i = 1; i < commands.size(); ++i) {
                const bool monotonic = direction > 0 ? (commands[i] > commands[i - 1] - 1e-9)
                    : (commands[i] < commands[i - 1] + 1e-9);
                if (!monotonic) {
                    terminalError = tr("%1轴%2 %3补偿命令在单向SWEEP中不单调：%4 -> %5。")
                        .arg(axisName5D(axis), direction > 0 ? QStringLiteral("+") : QStringLiteral("-"), mode)
                        .arg(commands[i - 1], 0, 'f', 6).arg(commands[i], 0, 'f', 6);
                    return false;
                }
            }
            const double primePreload = direction > 0 ? commands.front() - preload : commands.front() + preload;
            QString err;
            if (!moveOnly(primePreload, coarseMotion, err)) {
                if (m_axisCompValCancel.load() || m_closeRequested) cancelled = true;
                else terminalError = tr("%1轴%2 %3单向SWEEP PRIME预加载失败：%4")
                    .arg(axisName5D(axis), direction > 0 ? QStringLiteral("+") : QStringLiteral("-"), mode, err);
                return false;
            }
            for (int i = 0; i < orderedTargets.size(); ++i) {
                if (m_axisCompValCancel.load() || m_closeRequested) { cancelled = true; return false; }
                AxisCompensationValidationRecord r;
                r.axisId = axis;
                r.direction = direction;
                r.cycle = cycle;
                r.targetIndex = direction > 0 ? i : (static_cast<int>(orderedTargets.size()) - 1 - i);
                r.mode = mode;
                r.validationMethod = methodCode;
                r.approachMode = i == 0 ? QStringLiteral("PRIME") : QStringLiteral("SWEEP");
                r.targetMposMm = orderedTargets[i];
                r.commandedDposMm = commands[i];
                r.preloadDposMm = i == 0 ? primePreload : commands[i - 1];
                r.coarseSpeedMmS = coarseMotion.speed;
                r.precisionSpeedMmS = precisionMotion.speed;
                r.settleMs = settleMs;
                r.moveOk = moveOnly(commands[i], precisionMotion, err);
                r.error = err;
                if (!finalizeMeasuredRecord(r)) return false;
            }
            return true;
            };

        if (terminalError.isEmpty()) {
            for (const int direction : directions) {
                QVector<double> ordered = targets;
                if (direction < 0) std::reverse(ordered.begin(), ordered.end());
                for (int cycle = 1; cycle <= repeats && terminalError.isEmpty() && !cancelled; ++cycle) {
                    const QString firstMode = (cycle % 2) ? QStringLiteral("RAW") : QStringLiteral("COMPENSATED");
                    const QString secondMode = (cycle % 2) ? QStringLiteral("COMPENSATED") : QStringLiteral("RAW");
                    if (validationMethod == AxisCompValMonotonicSweep) {
                        if (!runMonotonicSweep(direction, cycle, ordered, firstMode)) break;
                        if (!runMonotonicSweep(direction, cycle, ordered, secondMode)) break;
                    }
                    else {
                        for (int i = 0; i < ordered.size() && terminalError.isEmpty() && !cancelled; ++i) {
                            if (m_axisCompValCancel.load() || m_closeRequested) { cancelled = true; break; }
                            const double target = ordered[i];
                            const int targetIndex = direction > 0 ? i : (static_cast<int>(ordered.size()) - 1 - i);
                            if (!measureLegacyPoint(direction, cycle, targetIndex, target, firstMode)) break;
                            if (!measureLegacyPoint(direction, cycle, targetIndex, target, secondMode)) break;
                        }
                    }
                }
            }
        }

        if (!cancelled && terminalError.isEmpty()) {
            QString restoreError;
            if (!controller->moveAbsRawDpos(axis, static_cast<float>(initial.dpos), &coarseMotion, &restoreError)) {
                terminalError = tr("%1轴A/B验证完成后恢复初始DPOS失败：%2").arg(axisName5D(axis), restoreError);
            }
        }

        QMetaObject::invokeMethod(this, [this, cancelled, terminalError, completed, totalRecords, outputRoot]() {
            --m_axisWorkerCount;
            m_axisCompValRunning = false;
            if (m_btnHomeAll) m_btnHomeAll->setEnabled(true);
            if (m_btnZeroAll) m_btnZeroAll->setEnabled(true);
            for (int axisId = AxisX; axisId <= AxisB; ++axisId) if (auto* w = axisWidgetForId(axisId)) w->setControlsEnabled(true);
            if (m_btnAxisMoveDiagAutoStart) m_btnAxisMoveDiagAutoStart->setEnabled(true);
            if (m_btnAxisCompCalStart) m_btnAxisCompCalStart->setEnabled(true);
            if (m_gcodeWidget) m_gcodeWidget->setEnabled(true);
            if (m_calibrationWidget) m_calibrationWidget->setEnabled(true);
            if (m_btnStartMerge) m_btnStartMerge->setEnabled(true);
            if (m_btnStartCapture) m_btnStartCapture->setEnabled(true);
            updateAxisCompensationValidationSummary();
            updateAxisCompensationValidationUi();
            updateAxisCompensationCalibrationUi();
            if (m_closeRequested) { tryFinishDeferredClose(); return; }
            if (cancelled) {
                if (m_lblAxisCompValStatus) m_lblAxisCompValStatus->setText(tr("A/B验证已取消：完成%1/%2条记录。可保存部分结果，但不能作为完整验证结论。")
                    .arg(completed).arg(totalRecords));
            }
            else if (!terminalError.isEmpty()) {
                if (m_lblAxisCompValStatus) m_lblAxisCompValStatus->setText(tr("A/B验证中止：%1；完成%2/%3条记录。")
                    .arg(terminalError).arg(completed).arg(totalRecords));
                QMessageBox::warning(this, tr("A/B验证中止"), terminalError);
            }
            else {
                if (m_progressAxisCompVal) m_progressAxisCompVal->setValue(100);
                const auto raw = computeAxisCompensationValidationStats(0, QStringLiteral("RAW"));
                const auto comp = computeAxisCompensationValidationStats(0, QStringLiteral("COMPENSATED"));
                const double improvement = std::isfinite(raw.rmsMm) && raw.rmsMm > 1e-12 && std::isfinite(comp.rmsMm)
                    ? 100.0 * (1.0 - comp.rmsMm / raw.rmsMm) : std::numeric_limits<double>::quiet_NaN();
                QString savedDir, saveError;
                const bool saved = saveAxisCompensationValidationFiles(outputRoot, &savedDir, &saveError);
                if (m_lblAxisCompValStatus) {
                    m_lblAxisCompValStatus->setText(tr("A/B验证完成（%1）：RAW RMS=%2 µm，补偿RMS=%3 µm，补偿P95=%4 µm，补偿Max=%5 µm，RMS改善=%6。%7")
                        .arg(axisCompensationValidationMethodLabel(m_axisCompValLastResolvedMethod))
                        .arg(raw.rmsMm * 1000.0, 0, 'f', 3).arg(comp.rmsMm * 1000.0, 0, 'f', 3)
                        .arg(comp.p95AbsMm * 1000.0, 0, 'f', 3).arg(comp.maxAbsMm * 1000.0, 0, 'f', 3)
                        .arg(std::isfinite(improvement) ? QString::number(improvement, 'f', 1) + QStringLiteral("%") : QStringLiteral("n/a"))
                        .arg(saved ? tr("结果已保存到 %1").arg(QDir::toNativeSeparators(savedDir)) : tr("自动保存失败：%1").arg(saveError)));
                }
            }
            tryFinishDeferredClose();
            });
        });
}

void MainWindow::stopAxisCompensationValidation()
{
    if (!m_axisCompValRunning) return;
    m_axisCompValCancel.store(true);
    if (m_btnAxisCompValStop) m_btnAxisCompValStop->setEnabled(false);
    if (m_zmc && m_zmc->isConnected() && m_axisCompValAxis >= AxisX && m_axisCompValAxis <= AxisZ) {
        m_zmc->requestStopAxis(m_axisCompValAxis, false);
    }
    if (m_lblAxisCompValStatus) m_lblAxisCompValStatus->setText(tr("正在停止候选补偿表A/B验证..."));
}

void MainWindow::resetAxisAlarmMonitor()
{
    for (int axisId = 0; axisId < Constants::kAxisParamArraySize; ++axisId) {
        m_lastAxisStatus[axisId] = 0;
        m_axisStatusSeen[axisId] = false;
        m_axisStatusReadFailed[axisId] = false;
    }

    for (int axisId = AxisX; axisId <= AxisB; ++axisId) {
        if (AxisControlWidget* widget = axisWidgetForId(axisId)) {
            widget->updateAxisStatusWord(-1, tr("未检查"), false);
        }
    }
}

void MainWindow::pollAxisAlarmState(int axisId, bool forceLog)
{
    if (m_closeRequested || !m_zmc || !m_zmc->isConnected()) return;
    if (axisId < AxisX || axisId > AxisB) return;

    // 在线激光同步采集期间不读取 AXISSTATUS。TABLE/触发/轮廓同步通信优先，
    // 所有状态检查延后到采集停止或其它非扫描事件点。
    if (m_syncManager && m_syncManager->isRunning()) return;

    int statusWord = 0;
    QString err;
    if (!m_zmc->getAxisStatusWord(axisId, statusWord, &err)) {
        if (!m_axisStatusReadFailed[axisId]) {
            m_axisStatusReadFailed[axisId] = true;
            const QString axisName =
                axisId == AxisX ? QStringLiteral("X") :
                axisId == AxisY ? QStringLiteral("Y") :
                axisId == AxisZ ? QStringLiteral("Z") :
                axisId == AxisC ? QStringLiteral("C") :
                QStringLiteral("B");
            LOG_WARNING(LogCategory::Controller,
                tr("%1轴 AXISSTATUS 读取失败: %2")
                .arg(axisName, err));
        }
        if (AxisControlWidget* widget = axisWidgetForId(axisId)) {
            widget->updateAxisStatusWord(-1, tr("状态读取失败"), false);
        }
        return;
    }

    m_axisStatusReadFailed[axisId] = false;
    const bool seenBefore = m_axisStatusSeen[axisId];
    const int oldStatus = m_lastAxisStatus[axisId];
    const bool changed = !seenBefore || oldStatus != statusWord;
    const QString statusText = ZmcController::describeAxisStatusWord(statusWord);
    const bool alarm = ZmcController::axisStatusHasAlarm(statusWord);

    if (AxisControlWidget* widget = axisWidgetForId(axisId)) {
        widget->updateAxisStatusWord(statusWord, statusText, alarm);
    }

    if (forceLog || changed) {
        const QString axisName =
            axisId == AxisX ? QStringLiteral("X") :
            axisId == AxisY ? QStringLiteral("Y") :
            axisId == AxisZ ? QStringLiteral("Z") :
            axisId == AxisC ? QStringLiteral("C") :
            QStringLiteral("B");
        const QString hex = QStringLiteral("0x%1")
            .arg(static_cast<quint32>(statusWord), 0, 16)
            .toUpper();

        if (statusWord == 0) {
            if (seenBefore && oldStatus != 0) {
                LOG_INFO(LogCategory::Controller,
                    tr("%1轴报警/状态已清除，AXISSTATUS=0").arg(axisName));
            }
            else if (forceLog) {
                LOG_INFO(LogCategory::Controller,
                    tr("%1轴 AXISSTATUS=0：正常").arg(axisName));
            }
        }
        else if (alarm) {
            LOG_WARNING(LogCategory::Controller,
                tr("%1轴 AXISSTATUS=%2：%3")
                .arg(axisName, hex, statusText));
        }
        else {
            LOG_INFO(LogCategory::Controller,
                tr("%1轴 AXISSTATUS=%2：%3")
                .arg(axisName, hex, statusText));
        }
    }

    m_lastAxisStatus[axisId] = statusWord;
    m_axisStatusSeen[axisId] = true;
}

void MainWindow::pollAxisAlarmStates(bool forceLog)
{
    if (m_closeRequested || !m_zmc || !m_zmc->isConnected()) return;

    bool allNormal = true;
    bool anyReadFailure = false;
    for (int axisId = AxisX; axisId <= AxisB; ++axisId) {
        const bool failedBefore = m_axisStatusReadFailed[axisId];
        pollAxisAlarmState(axisId, forceLog);
        if (m_axisStatusReadFailed[axisId] && !failedBefore) anyReadFailure = true;
        if (m_axisStatusSeen[axisId] && m_lastAxisStatus[axisId] != 0) allNormal = false;
    }

    if (forceLog && allNormal && !anyReadFailure) {
        LOG_INFO(LogCategory::Controller,
            tr("五轴 AXISSTATUS 初检正常：X/Y/Z/C/B 均无报警"));
    }
}

// ==================== 连接管理 ====================

void MainWindow::onControllerConnect()
{
    ++m_controllerSensorConnectionGeneration;
    m_geometryNcProof.passed=false; m_hardwareNcProof.passed=false;
    connectController(true);
}

bool MainWindow::runAutomaticHardwareSyncPreflight(bool showFailureDialog)
{
    // V27.6.16U: hardware-sync authorization is a converged machine-state result,
    // not a one-shot event tied only to B/C initialization or the last XYZ home.
    // Any homing sequence may finish in a different order, so never run the TABLE
    // preflight while another axis worker is still active. The final home callback
    // will retry after m_axisWorkerCount reaches zero.
    m_hardwareSyncPreflightPassed = false;
    if (m_radioSoftwareSync) m_radioSoftwareSync->setChecked(true);

    if (!m_zmc || !m_zmc->isConnected()) {
        updateVideoControlState();
        return false;
    }

    if (m_axisWorkerCount > 0 || m_jointMoveInProgress || m_rotaryInitInProgress) {
        updateVideoControlState();
        statusBar()->showMessage(tr("硬件同步自动预检等待五轴运动/坐标初始化全部结束"), 3000);
        LOG_INFO(LogCategory::Sync,
            tr("硬件同步自动预检暂缓：仍有轴运动或B/C绝对坐标初始化任务，待最后一个任务完成后自动重试"));
        return false;
    }

    // Incremental XYZ gratings have no absolute machine coordinate after power-up.
    // Deferring here is expected, not a hardware-sync fault: the real V32.13
    // preflight is rerun automatically once X/Y/Z have all completed DATUM and
    // homeAxis() has established their MPOS origins.
    if (!m_zmc->areLinearMposReferenced()) {
        updateVideoControlState();
        statusBar()->showMessage(tr("硬件同步等待XYZ完成回零并建立光栅MPOS机床零点"), 6000);
        LOG_INFO(LogCategory::Sync, tr("硬件同步自动预检暂缓：XYZ增量光栅MPOS机床基准尚未全部建立"));
        return false;
    }
    if (!m_zmc->isRotaryAbsoluteReady(AxisC) || !m_zmc->isRotaryAbsoluteReady(AxisB)) {
        updateVideoControlState();
        statusBar()->showMessage(tr("硬件同步等待B/C绝对机械坐标建立"), 6000);
        LOG_INFO(LogCategory::Sync, tr("硬件同步自动预检暂缓：B/C绝对机械坐标尚未建立"));
        return false;
    }
    if (!m_zmc->isAxisHomed(AxisC) || !m_zmc->isAxisHomed(AxisB)) {
        updateVideoControlState();
        statusBar()->showMessage(tr("硬件同步等待B/C完成本次控制器上电会话回零(0°)"), 6000);
        LOG_INFO(LogCategory::Sync, tr("硬件同步自动预检暂缓：C/B Home bit尚未全部建立"));
        return false;
    }

    QString summary;
    QString err;
    bool executionAuthorized = false;
    const bool schemaOk = m_zmc->hwTableCommissioningPreflight(
        summary, &err, &executionAuthorized);

    if (!schemaOk || !executionAuthorized) {
        const QString reason = !schemaOk
            ? (err.isEmpty() ? tr("硬件同步只读预检失败，控制器未返回可用的V32.13协议状态。") : err)
            : tr("V32.13协议只读检查通过，但TABLE[342]=0，控制器未授权执行物理硬件同步。\n"
                "请下载/运行V32.13 SINGLE-OP14 FULL-MPOS ColdPrime BASIC。\n"
                "修复后断开重连控制器或执行B/C绝对坐标重同步，软件会自动重新预检。");

        LOG_WARNING(LogCategory::Sync,
            tr("硬件同步自动只读预检失败: %1").arg(reason));
        updateVideoControlState();
        statusBar()->showMessage(tr("硬件同步自动只读预检失败: %1").arg(reason), 10000);
        if (showFailureDialog && !m_closeRequested) {
            QMessageBox::warning(this, tr("硬件同步自动预检未通过"), reason);
        }
        return false;
    }

    m_hardwareSyncPreflightPassed = true;
    LOG_INFO(LogCategory::Sync, tr("硬件同步自动只读预检通过；硬件模式已静默解锁。\n%1").arg(summary));
    updateVideoControlState();
    statusBar()->showMessage(tr("硬件同步自动只读预检通过，硬件模式已解锁"), 5000);
    return true;
}

void MainWindow::connectController(bool interactive)
{
    if (m_closeRequested || m_controllerConnectInProgress || m_rotaryInitInProgress) return;

    const QString ip = m_controllerIpEdit->text().trimmed();
    if (ip.isEmpty()) {
        const QString msg = tr("请先输入控制器 IP 地址");
        if (interactive) QMessageBox::warning(this, tr("连接失败"), msg);
        statusBar()->showMessage(msg, 5000);
        return;
    }

    const QStringList parts = ip.split(QLatin1Char('.'));
    bool ipSyntaxOk = (parts.size() == 4);
    if (ipSyntaxOk) {
        for (const QString& part : parts) {
            bool ok = false;
            const int value = part.toInt(&ok);
            if (!ok || value < 0 || value > 255 || part.isEmpty()) {
                ipSyntaxOk = false;
                break;
            }
        }
    }
    if (!ipSyntaxOk) {
        const QString msg = tr("控制器 IP 地址格式无效: %1").arg(ip);
        if (interactive) QMessageBox::warning(this, tr("连接失败"), msg);
        statusBar()->showMessage(msg, 5000);
        return;
    }

    // ZAux_OpenEth 在控制器不存在/网卡未配置时会同步阻塞 GUI；
    // 但把厂商 DLL 放到 QtConcurrent 线程又在现场出现过进程级闪退。
    // 因此 V8 先用系统 ping 做“非阻塞预检查”：IP 不响应时根本不调用 ZAux_OpenEth；
    // 只有 IP 已响应时才在原 GUI 线程执行一次 ZAux_OpenEth，此时正常连接通常很快。
    // A preflight belongs to one concrete controller connection and one concrete
    // B/C absolute software frame.  Reconnect starts from fail-closed state.
    m_hardwareSyncPreflightPassed = false;
    if (m_radioSoftwareSync) m_radioSoftwareSync->setChecked(true);
    updateVideoControlState();

    LOG_INFO(LogCategory::Controller, tr("手动连接：先异步探测控制器 IP %1").arg(ip));
    m_controllerConnectInProgress = true;
    if (m_btnControllerConnect) m_btnControllerConnect->setEnabled(false);
    if (m_btnControllerDisconnect) m_btnControllerDisconnect->setEnabled(false);
    if (m_btnResyncRotaryAbs) m_btnResyncRotaryAbs->setEnabled(false);
    m_lblControllerStatus->setText(tr("探测中..."));
    m_lblControllerStatus->setStyleSheet("QLabel { color: #b36b00; font-weight: bold; }");
    statusBar()->showMessage(tr("正在异步探测 %1；界面可继续操作...").arg(ip));

    auto* probe = new QProcess(this);
    m_controllerProbeProcess = probe;
    probe->setProcessChannelMode(QProcess::SeparateChannels);

    auto failProbe = [this, probe, interactive](const QString& msg) {
        if (m_controllerProbeProcess != probe) return;
        m_controllerProbeProcess = nullptr;
        m_controllerConnectInProgress = false;
        probe->deleteLater();
        if (m_btnControllerConnect) m_btnControllerConnect->setEnabled(true);
        if (m_btnControllerDisconnect) m_btnControllerDisconnect->setEnabled(false);
        if (m_btnResyncRotaryAbs) m_btnResyncRotaryAbs->setEnabled(false);
        m_lblControllerStatus->setText(tr("未连接"));
        m_lblControllerStatus->setStyleSheet("QLabel { color: red; font-weight: bold; }");
        m_cAxisWidget->setManualServoState(false, false, tr("控制器未连接，禁止使能"));
        m_bAxisWidget->setManualServoState(false, false, tr("控制器未连接，禁止使能"));
        if (m_calibrationWidget) m_calibrationWidget->refreshSingleSphereDeviceState();
        LOG_WARNING(LogCategory::Controller, msg);
        statusBar()->showMessage(msg, 7000);
        if (interactive) QMessageBox::warning(this, tr("连接失败"), msg);
        };

    connect(probe, &QProcess::errorOccurred, this,
        [failProbe](QProcess::ProcessError) {
            failProbe(QObject::tr("无法启动系统 ping 预检查；为避免界面阻塞，本次未调用 ZAux_OpenEth"));
        });

    connect(probe, qOverload<int, QProcess::ExitStatus>(&QProcess::finished), this,
        [this, probe, ip, interactive, failProbe](int exitCode, QProcess::ExitStatus exitStatus) {
            if (m_controllerProbeProcess != probe) return;

            const bool reachable = (exitStatus == QProcess::NormalExit && exitCode == 0);
            if (!reachable) {
                failProbe(tr("控制器 IP %1 未响应；本次未调用 ZAux_OpenEth。请检查 PC 网卡/IP/网线后手动重试")
                    .arg(ip));
                return;
            }

            m_controllerProbeProcess = nullptr;
            probe->deleteLater();

            if (m_closeRequested) {
                m_controllerConnectInProgress = false;
                tryFinishDeferredClose();
                return;
            }

            LOG_INFO(LogCategory::Controller, tr("控制器 IP %1 已响应，开始 ZAux_OpenEth").arg(ip));
            statusBar()->showMessage(tr("控制器 IP 已响应，正在建立正运动连接..."));
            m_lblControllerStatus->setText(tr("连接中..."));

            QString err;
            QApplication::setOverrideCursor(Qt::WaitCursor);
            const bool connected = m_zmc->connectTcp(ip, 777, &err);
            QApplication::restoreOverrideCursor();
            m_controllerConnectInProgress = false;
            if (m_calibrationWidget) m_calibrationWidget->refreshSingleSphereDeviceState();

            if (m_closeRequested) {
                if (connected && m_zmc->isConnected()) m_zmc->disconnect();
                tryFinishDeferredClose();
                return;
            }

            if (m_btnControllerConnect) m_btnControllerConnect->setEnabled(!connected);
            if (m_btnControllerDisconnect) m_btnControllerDisconnect->setEnabled(connected);

            if (!connected) {
                LOG_ERROR(LogCategory::Controller, tr("控制器连接失败: %1").arg(err));
                if (interactive) QMessageBox::warning(this, tr("连接失败"), err);
                statusBar()->showMessage(tr("控制器连接失败: ") + err, 5000);
                m_lblControllerStatus->setText(tr("失败"));
                m_lblControllerStatus->setStyleSheet("QLabel { color: red; font-weight: bold; }");
                m_cAxisWidget->setManualServoState(false, false, tr("控制器未连接，禁止使能"));
                m_bAxisWidget->setManualServoState(false, false, tr("控制器未连接，禁止使能"));
                if (m_btnResyncRotaryAbs) m_btnResyncRotaryAbs->setEnabled(false);
                return;
            }

            LOG_INFO(LogCategory::Controller, tr("控制器连接成功，B/C 使能已强制关闭"));
            m_lblControllerStatus->setText(tr("已连接"));
            m_lblControllerStatus->setStyleSheet("QLabel { color: green; font-weight: bold; }");
            if (m_btnControllerDisconnect) m_btnControllerDisconnect->setEnabled(true);
            if (m_btnResyncRotaryAbs) m_btnResyncRotaryAbs->setEnabled(true);
            m_cAxisWidget->setManualServoState(false, false, tr("等待读取 C 轴绝对位置"));
            m_bAxisWidget->setManualServoState(false, false, tr("等待读取 B 轴绝对位置"));
            statusBar()->showMessage(tr("已连接到控制器，正在读取五轴参数..."));

            // 一旦 ZAux_OpenEth 成功，立即读取五轴 AXISSTATUS。
            // 这样即使后续参数读取或 B/C 绝对位置同步失败，上电时已经存在的
            // 硬限位、软限位、伺服报警等状态仍会马上进入日志。
            resetAxisAlarmMonitor();
            pollAxisAlarmStates(true);

            AxisParams localParams[Constants::kAxisParamArraySize];
            if (!m_zmc->readAllAxisParams(localParams, &err)) {
                LOG_WARNING(LogCategory::Controller, tr("五轴参数读取失败: %1").arg(err));
                m_cAxisWidget->setManualServoState(false, false, tr("轴参数读取失败，禁止使能"));
                m_bAxisWidget->setManualServoState(false, false, tr("轴参数读取失败，禁止使能"));
                statusBar()->showMessage(tr("已连接，但五轴参数读取失败，B/C 保持禁止使能: ") + err, 8000);
                if (interactive) {
                    QMessageBox::warning(this, tr("轴参数读取失败"),
                        tr("B/C 将保持断使能。\n%1").arg(err));
                }
                return;
            }

            for (int axisId = AxisX; axisId <= AxisB; ++axisId) {
                m_axisParams[axisId] = localParams[axisId];
            }
            fillAxisTableFromParams();
            for (int axisId = AxisX; axisId <= AxisB; ++axisId) {
                if (AxisControlWidget* widget = axisWidgetForId(axisId)) {
                    widget->setAxisParams(m_axisParams[axisId]);
                }
            }

            LOG_INFO(LogCategory::Controller, tr("五轴参数读取成功，开始读取 B/C 绝对位置并建立软件机械坐标"));
            startRotaryAbsoluteInitialization(interactive);
        });

#ifdef Q_OS_WIN
    probe->start(QStringLiteral("ping"),
        QStringList{ QStringLiteral("-n"), QStringLiteral("1"),
                    QStringLiteral("-w"), QStringLiteral("400"), ip });
#else
    probe->start(QStringLiteral("ping"),
        QStringList{ QStringLiteral("-c"), QStringLiteral("1"),
                    QStringLiteral("-W"), QStringLiteral("1"), ip });
#endif

    // 防止极端系统环境下 ping 子进程本身悬挂；kill 不阻塞 GUI。
    QTimer::singleShot(1500, probe, [probe]() {
        if (probe->state() != QProcess::NotRunning) probe->kill();
        });
}

void MainWindow::startRotaryAbsoluteInitialization(bool interactive)
{
    if (m_closeRequested || !m_zmc->isConnected()) {
        if (interactive && !m_closeRequested) QMessageBox::warning(this, tr("提示"), tr("请先连接控制器"));
        return;
    }
    if (m_rotaryInitInProgress || m_controllerConnectInProgress) return;
    if ((m_syncManager && m_syncManager->isRunning())
        || (m_gcodeExecutor && (m_gcodeExecutor->isRunning() || m_gcodeExecutor->isPaused()))
        || m_jointMoveInProgress || m_axisWorkerCount > 0) {
        if (interactive) {
            QMessageBox::warning(this, tr("暂不能重建B/C绝对坐标"),
                tr("仍有扫描、G代码或运动任务在执行。当前任务结束前禁止重建B/C绝对机械坐标。"));
        }
        return;
    }

    // Hardware-sync authorization is tied to the B/C software mechanical frame.
    // Any RS485 absolute re-sync changes (or at least re-proves) that frame, so a
    // previous preflight may not be reused.
    m_hardwareSyncPreflightPassed = false;
    if (m_radioSoftwareSync) m_radioSoftwareSync->setChecked(true);
    updateVideoControlState();

    // Rebuilding the rotary absolute frame invalidates the coordinate frame used
    // to authorize the calibration.  Clear both runtime consumers first; a
    // successful completion will refresh the calibration gate and re-apply only
    // if the live conventions still match.
    syncValidatedCalibrationToRuntime(false, false);

    m_rotaryInitInProgress = true;
    if (m_btnResyncRotaryAbs) m_btnResyncRotaryAbs->setEnabled(false);
    if (m_btnControllerDisconnect) m_btnControllerDisconnect->setEnabled(false);

    m_cAxisWidget->setManualServoState(false, false, tr("正在通过 RS485 读取 C 轴绝对位置..."));
    m_bAxisWidget->setManualServoState(false, false, tr("正在通过 RS485 读取 B 轴绝对位置..."));
    statusBar()->showMessage(tr("正在后台读取 B/C 禾川绝对位置并建立机械坐标框架；不会写 MPOS/DPOS，完成前禁止使能..."));

    auto result = std::make_shared<RotaryInitResult>();
    ZmcController* controller = m_zmc.get();

    auto* watcher = new QFutureWatcher<void>(this);
    m_rotaryInitWatcher = watcher;
    connect(watcher, &QFutureWatcher<void>::finished, this,
        [this, watcher, result, interactive]() {
            m_rotaryInitInProgress = false;
            if (m_rotaryInitWatcher == watcher) m_rotaryInitWatcher = nullptr;
            watcher->deleteLater();

            if (m_closeRequested) {
                if (m_zmc->isConnected()) m_zmc->disconnect();
                tryFinishDeferredClose();
                return;
            }

            if (m_btnControllerDisconnect) m_btnControllerDisconnect->setEnabled(m_zmc->isConnected());
            if (m_btnResyncRotaryAbs) m_btnResyncRotaryAbs->setEnabled(m_zmc->isConnected());

            if (!result->ok) {
                m_cAxisWidget->setManualServoState(false, false, tr("C轴绝对位置/机械坐标初始化失败，禁止使能"));
                m_bAxisWidget->setManualServoState(false, false, tr("B轴绝对位置/机械坐标初始化失败，禁止使能"));
                LOG_ERROR(LogCategory::Controller, tr("B/C 绝对位置/机械坐标初始化失败: %1").arg(result->error));
                statusBar()->showMessage(tr("B/C 绝对位置/机械坐标初始化失败，两个轴保持断使能: ") + result->error, 10000);
                if (interactive) {
                    QMessageBox::warning(this, tr("B/C 绝对位置/机械坐标初始化失败"),
                        tr("未完成 B/C 绝对位置读取与机械坐标框架建立前无法使能。\n%1").arg(result->error));
                }
                // The persistent-calibration binder may have attempted its first live
                // check before B/C absolute readiness existed.  Refresh again on this
                // definitive failure transition so the gate remains visibly fail-closed.
                if (m_calibrationWidget) m_calibrationWidget->refreshSingleSphereDeviceState();
                return;
            }

            float cDpos = static_cast<float>(result->cDeg);
            float cMpos = static_cast<float>(result->cDeg);
            float bDpos = static_cast<float>(result->bDeg);
            float bMpos = static_cast<float>(result->bDeg);
            bool cIdle = true;
            bool bIdle = true;
            (void)m_zmc->getAxisPositionPair(AxisC, cDpos, cMpos, cIdle, nullptr);
            (void)m_zmc->getAxisPositionPair(AxisB, bDpos, bMpos, bIdle, nullptr);
            m_cAxisWidget->updatePositionPair(cDpos, cMpos, cIdle);
            m_bAxisWidget->updatePositionPair(bDpos, bMpos, bIdle);
            const bool cHomeRestored = m_zmc->isAxisHomed(AxisC);
            const bool bHomeRestored = m_zmc->isAxisHomed(AxisB);
            m_cAxisWidget->setManualServoState(
                true, false, tr("机械绝对坐标已建立: %1° / Home=%2 / 等待人工使能")
                .arg(result->cDeg, 0, 'f', 4).arg(cHomeRestored ? tr("已恢复") : tr("未完成")));
            m_bAxisWidget->setManualServoState(
                true, false, tr("机械绝对坐标已建立: %1° / Home=%2 / 等待人工使能")
                .arg(result->bDeg, 0, 'f', 4).arg(bHomeRestored ? tr("已恢复") : tr("未完成")));

            LOG_INFO(LogCategory::Controller,
                tr("B/C 软件机械坐标建立成功: C=%1°, B=%2°；Home恢复 C=%3 B=%4；控制器 MPOS/DPOS 未写入，保持断使能")
                .arg(result->cDeg, 0, 'f', 6).arg(result->bDeg, 0, 'f', 6)
                .arg(cHomeRestored ? 1 : 0).arg(bHomeRestored ? 1 : 0));
            statusBar()->showMessage(
                tr("B/C 机械绝对坐标已建立：C=%1°，B=%2°；Home C=%3/B=%4；请人工使能后继续")
                .arg(result->cDeg, 0, 'f', 4).arg(result->bDeg, 0, 'f', 4)
                .arg(cHomeRestored ? tr("已恢复") : tr("未完成"))
                .arg(bHomeRestored ? tr("已恢复") : tr("未完成")), 9000);

            // 只在绝对坐标重建完成这个事件点复核 B/C，一次性读取，不做周期轮询。
            pollAxisAlarmState(AxisC, false);
            pollAxisAlarmState(AxisB, false);

            // Device connection can complete before the asynchronous absolute-frame
            // initialization.  This successful transition is therefore the reliable
            // event at which a persisted calibration can be rebound and authorized.
            if (m_calibrationWidget) m_calibrationWidget->refreshSingleSphereDeviceState();

            // V27.6.16N: the controller session is connected and the B/C absolute
            // software frame is now authoritative. This is the earliest safe point
            // to run the read-only V32.13 hardware-sync preflight automatically.
            // Success stays silent; failure alone opens a dialog.
            (void)runAutomaticHardwareSyncPreflight(true);
        });

    watcher->setFuture(QtConcurrent::run([controller, result]() {
        result->ok = controller->initializeRotaryAxesFromAbsolute(result->cDeg, result->bDeg, &result->error);
        }));
}

void MainWindow::onControllerDisconnect()
{
    ++m_controllerSensorConnectionGeneration;
    m_geometryNcProof.passed=false; m_hardwareNcProof.passed=false;
    if(m_small7Active){
        onStopSmall7Experiment();
        QMessageBox::warning(this,tr("实验3尚未结束"),
            tr("已请求停止X/Z运动与采集；请等待工作线程退出再断开控制器"));
        return;
    }
    if (m_sphereExpActive) {
        onStopSphereExposureSweep();
        QMessageBox::warning(this, tr("全球曝光扫描仍在收尾"), tr("已请求停止运动与采集。请等待后台运动任务退出后再断开控制器。"));
        return;
    }
    if (m_sphereStudyAutoActive && m_jointMoveInProgress) {
        onSphereStudyAutoStop();
        QMessageBox::warning(this, tr("暂不能断开控制器"),
            tr("自动球面研究正在执行五轴移动，已请求停止。请等待运动线程结束后再断开控制器。"));
        return;
    }
    if (m_sphereStudyAutoActive) {
        onSphereStudyAutoStop();
    }
    else if (m_sphereStudyCaptureActive) {
        finishSphereStudyCapture(true, tr("控制器断开，终止球面距离/角度研究采集"));
    }
    if (m_jointMoveInProgress || m_axisWorkerCount > 0) {
        QMessageBox::warning(this, tr("暂不能断开控制器"),
            tr("仍有运动任务在执行。请先停止并等待运动线程结束后再断开控制器。"));
        return;
    }
    if (m_controllerConnectInProgress || m_rotaryInitInProgress) {
        statusBar()->showMessage(tr("控制器通信任务进行中，请等待完成后再断开"), 3000);
        return;
    }

    LOG_INFO(LogCategory::Controller, tr("断开控制器连接"));
    m_hardwareSyncPreflightPassed = false;
    if (m_radioSoftwareSync) m_radioSoftwareSync->setChecked(true);
    m_syncManager->clearHardwareTriggerTypeConfiguration();
    m_syncManager->stop();
    m_zmc->disconnect();
    if (m_calibrationWidget) m_calibrationWidget->refreshSingleSphereDeviceState();
    resetAxisAlarmMonitor();
    m_cAxisWidget->setManualServoState(false, false, tr("控制器已断开，禁止使能"));
    m_bAxisWidget->setManualServoState(false, false, tr("控制器已断开，禁止使能"));
    if (m_btnResyncRotaryAbs) m_btnResyncRotaryAbs->setEnabled(false);
    if (m_btnControllerConnect) m_btnControllerConnect->setEnabled(true);
    if (m_btnControllerDisconnect) m_btnControllerDisconnect->setEnabled(false);
    m_lblControllerStatus->setText(tr("未连接"));
    m_lblControllerStatus->setStyleSheet("QLabel { color: red; font-weight: bold; }");
    updateSphereStudyUiState();
    statusBar()->showMessage(tr("控制器已断开，B/C 使能已关闭"));
}

// ==================== 轴参数 ====================

void MainWindow::fillAxisTableFromParams()
{
    for (int axisId = AxisX; axisId <= AxisB; ++axisId) {
        const AxisParams& p = m_axisParams[axisId];
        int row = axisId - AxisX;

        auto setNum = [this, row](int col, float v) {
            m_axisTable->item(row, col)->setText(QString::number(v, 'f', 4));
            };

        setNum(1, p.units);
        setNum(2, p.lspeed);
        setNum(3, p.speed);
        setNum(4, p.accel);
        setNum(5, p.decel);
        setNum(6, p.sramp);
        setNum(7, p.creep);
        setNum(8, p.softLimitPos);
        setNum(9, p.softLimitNeg);
    }
}

void MainWindow::readParamsFromTable()
{
    for (int axisId = AxisX; axisId <= AxisB; ++axisId) {
        int row = axisId - AxisX;
        AxisParams& p = m_axisParams[axisId];

        auto getNum = [this, row](int col) -> float {
            bool ok = false;
            float v = m_axisTable->item(row, col)->text().toFloat(&ok);
            return ok ? v : 0.0f;
            };

        p.units = getNum(1);
        p.lspeed = getNum(2);
        p.speed = getNum(3);
        p.accel = getNum(4);
        p.decel = getNum(5);
        p.sramp = getNum(6);
        p.creep = getNum(7);
        p.softLimitPos = getNum(8);
        p.softLimitNeg = getNum(9);
    }
}

void MainWindow::onReadAxisParams()
{
    if (!m_zmc->isConnected()) {
        QMessageBox::warning(this, tr("提示"), tr("请先连接控制器"));
        return;
    }

    QString err;
    if (!m_zmc->readAllAxisParams(m_axisParams, &err)) {
        QMessageBox::warning(this, tr("读取失败"), err);
        return;
    }
    fillAxisTableFromParams();

    // 更新五轴控制面板
    for (int axisId = AxisX; axisId <= AxisB; ++axisId) {
        if (AxisControlWidget* widget = axisWidgetForId(axisId)) {
            widget->setAxisParams(m_axisParams[axisId]);
        }
    }

    statusBar()->showMessage(tr("五轴参数已从控制器读取"), 3000);
}

void MainWindow::onWriteAxisParams()
{
    if (!m_zmc->isConnected()) {
        QMessageBox::warning(this, tr("提示"), tr("请先连接控制器"));
        return;
    }

    readParamsFromTable();

    int row = m_axisTable->currentRow();
    if (row < 0 || row > 4) row = 0;
    int axisId = row + AxisX;

    QString err;
    if (!m_zmc->writeAxisParams(axisId, m_axisParams[axisId], &err)) {
        // 旋转轴参数写入失败时，底层可能已经因安全策略撤销了 ready/使能。
        m_cAxisWidget->setManualServoState(m_zmc->isRotaryAbsoluteReady(AxisC),
            m_zmc->isRotaryServoEnabled(AxisC));
        m_bAxisWidget->setManualServoState(m_zmc->isRotaryAbsoluteReady(AxisB),
            m_zmc->isRotaryServoEnabled(AxisB));
        QMessageBox::warning(this, tr("写入失败"), err);
        return;
    }
    if (AxisControlWidget* widget = axisWidgetForId(axisId)) {
        widget->setAxisParams(m_axisParams[axisId]);
    }
    if (axisId >= AxisX && axisId <= AxisZ) {
        m_axisCompSpeedProfiles[axisId].qualified = false;
        m_axisCompSpeedProfiles[axisId].status = tr("轴运动参数已修改，请重新测速");
        updateAxisCompensationSpeedProfileTable();
        updateAxisCompensationCalibrationUi();
    }

    if (axisId == AxisB || axisId == AxisC) {
        // V27.6.38: speed/accel/decel-only writes no longer destroy the rotary
        // absolute-coordinate binding.  Only a coordinate-sensitive change
        // (UNITS / machine soft-limit frame) makes the controller revoke ready.
        const bool cReady = m_zmc->isRotaryAbsoluteReady(AxisC);
        const bool bReady = m_zmc->isRotaryAbsoluteReady(AxisB);
        m_cAxisWidget->setManualServoState(cReady, m_zmc->isRotaryServoEnabled(AxisC),
            cReady ? tr("旋转轴运动参数已更新；绝对位置保持有效")
            : tr("旋转轴坐标参数已修改，等待重新同步绝对位置"));
        m_bAxisWidget->setManualServoState(bReady, m_zmc->isRotaryServoEnabled(AxisB),
            bReady ? tr("旋转轴运动参数已更新；绝对位置保持有效")
            : tr("旋转轴坐标参数已修改，等待重新同步绝对位置"));
        if (!cReady || !bReady) startRotaryAbsoluteInitialization(true);
        else statusBar()->showMessage(tr("B/C运动参数已更新；无需重新回零/重读绝对位置"), 4000);
        return;
    }

    statusBar()->showMessage(tr("已写入轴 %1 参数").arg(axisId), 3000);
}

void MainWindow::onWriteAllAxisParams()
{
    if (!m_zmc->isConnected()) {
        QMessageBox::warning(this, tr("提示"), tr("请先连接控制器"));
        return;
    }

    readParamsFromTable();

    QString err;
    if (!m_zmc->writeAllAxisParams(m_axisParams, &err)) {
        m_cAxisWidget->setManualServoState(m_zmc->isRotaryAbsoluteReady(AxisC),
            m_zmc->isRotaryServoEnabled(AxisC));
        m_bAxisWidget->setManualServoState(m_zmc->isRotaryAbsoluteReady(AxisB),
            m_zmc->isRotaryServoEnabled(AxisB));
        QMessageBox::warning(this, tr("写入失败"), err);
        return;
    }
    for (int axisId = AxisX; axisId <= AxisB; ++axisId) {
        if (AxisControlWidget* widget = axisWidgetForId(axisId)) {
            widget->setAxisParams(m_axisParams[axisId]);
        }
    }
    for (int axisId = AxisX; axisId <= AxisZ; ++axisId) {
        m_axisCompSpeedProfiles[axisId].qualified = false;
        m_axisCompSpeedProfiles[axisId].status = tr("轴运动参数已修改，请重新测速");
    }
    updateAxisCompensationSpeedProfileTable();
    updateAxisCompensationCalibrationUi();

    const bool cReady = m_zmc->isRotaryAbsoluteReady(AxisC);
    const bool bReady = m_zmc->isRotaryAbsoluteReady(AxisB);
    m_cAxisWidget->setManualServoState(cReady, m_zmc->isRotaryServoEnabled(AxisC),
        cReady ? tr("五轴运动参数已更新；C绝对位置保持有效")
        : tr("五轴坐标参数已修改，等待重新同步绝对位置"));
    m_bAxisWidget->setManualServoState(bReady, m_zmc->isRotaryServoEnabled(AxisB),
        bReady ? tr("五轴运动参数已更新；B绝对位置保持有效")
        : tr("五轴坐标参数已修改，等待重新同步绝对位置"));
    if (!cReady || !bReady) startRotaryAbsoluteInitialization(true);
    else statusBar()->showMessage(tr("五轴运动参数已更新；XYZ DATUM 与 B/C 绝对位置保持有效"), 4000);
}

// ==================== 全局控制 ====================

void MainWindow::onZeroAll()
{
    if (!m_zmc->isConnected()) return;

    QString err;
    if (!m_zmc->zeroAllDpos(&err)) {
        QMessageBox::warning(this, tr("清零失败"), err);
    }
}

void MainWindow::onHomeAll()
{
    if (!m_zmc->isConnected()) {
        QMessageBox::warning(this, tr("提示"), tr("请先连接控制器"));
        return;
    }

    // “五轴全部回零”必须是真正的五轴动作，避免只回 XYZ 造成操作员误判。
    // B/C 没有原点开关，因此其回零定义为绝对定位到 0°；这两个轴必须已经
    // 完成绝对位置同步并由操作员人工使能。
    QStringList notReady;
    for (const int axisId : { AxisC, AxisB }) {
        const QString axisName = axisId == AxisC ? QStringLiteral("C") : QStringLiteral("B");
        if (!m_zmc->isRotaryAbsoluteReady(axisId)) {
            notReady << tr("%1轴绝对位置未同步").arg(axisName);
        }
        else if (!m_zmc->isRotaryServoEnabled(axisId)) {
            notReady << tr("%1轴未人工使能").arg(axisName);
        }
    }
    if (!notReady.isEmpty()) {
        QMessageBox::warning(
            this, tr("五轴全部回零条件不满足"),
            tr("XYZ 使用原点开关回零，B/C 使用绝对坐标回到 0°。\n"
                "执行“五轴全部回零”前必须先满足：\n%1")
            .arg(notReady.join(QStringLiteral("\n"))));
        return;
    }

    LOG_INFO(LogCategory::Controller,
        tr("开始五轴全部回零：XYZ 执行 DATUM，C/B 绝对定位到 0°"));

    onAxisHomeRequested(AxisX, m_axisParams[AxisX]);
    onAxisHomeRequested(AxisY, m_axisParams[AxisY]);
    onAxisHomeRequested(AxisZ, m_axisParams[AxisZ]);
    onAxisHomeRequested(AxisC, m_axisParams[AxisC]);
    onAxisHomeRequested(AxisB, m_axisParams[AxisB]);
}

void MainWindow::onStopAll()
{
    if(m_gcodeHardwareNcProbeActive)finishGCodeHardwareNcProbe(true);
    if (m_sphereExpActive) onStopSphereExposureSweep();
    if (m_small7Active) onStopSmall7Experiment();
    if (m_axisMoveDiagAutoRunning) m_axisMoveDiagAutoCancel.store(true);
    if (m_axisCompSpeedRunning) m_axisCompSpeedCancel.store(true);
    if (m_axisCompCalRunning) m_axisCompCalCancel.store(true);
    if (m_axisCompValRunning) m_axisCompValCancel.store(true);
    if (m_zmc->isConnected() && m_zmc->requestStopAllAxes(false))
        statusBar()->showMessage(tr("全轴停止请求已入队；请确认机床实际停稳"), 3000);
}

// ==================== 轴运动请求 ====================

void MainWindow::onAxisHomeRequested(int axisId, const AxisParams& params)
{
    if (!m_zmc->isConnected()) {
        QMessageBox::warning(this, tr("提示"), tr("请先连接控制器"));
        return;
    }

    const bool rotaryHome = (axisId == AxisB || axisId == AxisC);
    const QString axisName =
        axisId == AxisX ? QStringLiteral("X") :
        axisId == AxisY ? QStringLiteral("Y") :
        axisId == AxisZ ? QStringLiteral("Z") :
        axisId == AxisC ? QStringLiteral("C") :
        QStringLiteral("B");

    // B/C 没有原点开关。由于连接后已通过绝对编码器建立 DPOS/MPOS，
    // 这里的“回零”只表示把该旋转轴绝对定位到 0°。
    if (rotaryHome) {
        if (!m_zmc->isRotaryAbsoluteReady(axisId)) {
            QMessageBox::warning(this, tr("不能回零"),
                tr("%1轴绝对位置尚未同步，不能回到 0°").arg(axisName));
            return;
        }
        if (!m_zmc->isRotaryServoEnabled(axisId)) {
            QMessageBox::warning(this, tr("不能回零"),
                tr("%1轴尚未人工使能；请先人工使能，再执行回零(0°)")
                .arg(axisName));
            return;
        }
    }

    AxisControlWidget* widget = axisWidgetForId(axisId);
    if (widget) widget->setControlsEnabled(false);

    // V27.6.16U: a new homing operation invalidates any authorization derived
    // from the previous settled machine state. This also prevents the hardware
    // radio from remaining enabled while Home-All is still in progress.
    m_hardwareSyncPreflightPassed = false;
    if (m_radioSoftwareSync) m_radioSoftwareSync->setChecked(true);
    updateVideoControlState();

    HomeParams hp;
    hp.mode = 13;

    AxisParams motion = params;
    if (!rotaryHome) {
        // 保留原 XYZ 回零速度策略。
        motion.speed = 10.0f;
    }

    LOG_INFO(LogCategory::Controller,
        rotaryHome
        ? tr("%1轴回零请求：绝对定位到 0°").arg(axisName)
        : tr("%1轴回零请求：执行 DATUM 模式 %2").arg(axisName).arg(hp.mode));

    ++m_axisWorkerCount;
    ZmcController* controller = m_zmc.get();
    (void)QtConcurrent::run([this, controller, axisId, axisName, rotaryHome, hp, motion, widget]() {
        QString err;
        const bool ok = rotaryHome
            ? controller->homeRotaryAxisToZero(axisId, motion, &err)
            : controller->homeAxis(axisId, hp, motion, &err);

        QMetaObject::invokeMethod(this, [this, widget, axisId, axisName, rotaryHome, ok, err]() {
            --m_axisWorkerCount;
            if (widget) widget->setControlsEnabled(true);

            // 回零结束后只读取本轴一次 AXISSTATUS，不做周期轮询。
            pollAxisAlarmState(axisId, false);

            if (!m_closeRequested) {
                if (!ok) {
                    QMessageBox::warning(this, tr("回零失败"),
                        err.isEmpty() ? tr("未知错误") : err);
                }
                else {
                    const QString msg = rotaryHome
                        ? tr("%1轴已回到绝对 0°").arg(axisName)
                        : tr("%1轴原点回零完成，光栅MPOS机床零点已建立").arg(axisName);
                    LOG_INFO(LogCategory::Controller, msg);
                    statusBar()->showMessage(msg, 3000);

                    // V27.6.16U: converge authorization after ANY successful X/Y/Z/C/B
                    // home completion. In V27.6.16S this retry was restricted to XYZ.
                    // During Home-All the last XYZ could finish while C/B were still
                    // moving; the preflight then deferred/failed, and the later C/B
                    // completion never retried it. Only the last outstanding axis worker
                    // is allowed to run the TABLE preflight.
                    if (m_zmc && m_axisWorkerCount == 0
                        && m_zmc->areAllAxesHomed()
                        && m_zmc->isRotaryAbsoluteReady(AxisC)
                        && m_zmc->isRotaryAbsoluteReady(AxisB)) {
                        (void)runAutomaticHardwareSyncPreflight(true);
                        if (m_calibrationWidget) m_calibrationWidget->refreshSingleSphereDeviceState();
                    }
                }
            }
            updateVideoControlState();
            updateSphereStudyUiState();
            tryFinishDeferredClose();
            });
        });
}

void MainWindow::onAxisAbsMoveRequested(int axisId, float pos, const AxisParams& params)
{
    if (!m_zmc->isConnected()) {
        QMessageBox::warning(this, tr("提示"), tr("请先连接控制器"));
        return;
    }

    AxisControlWidget* widget = axisWidgetForId(axisId);

    if (widget) widget->setControlsEnabled(false);

    AxisParams motion = params;
    const bool diagEnabled = axisId <= AxisZ && m_checkAxisMoveDiagAuto && m_checkAxisMoveDiagAuto->isChecked();
    const int diagSettleMs = m_spinAxisMoveDiagSettleMs ? m_spinAxisMoveDiagSettleMs->value() : 2000;

    ++m_axisWorkerCount;
    ZmcController* controller = m_zmc.get();
    (void)QtConcurrent::run([this, controller, axisId, pos, motion, widget, diagEnabled, diagSettleMs]() {
        AxisMoveDiagnosticRecord diag;
        if (diagEnabled) {
            diag.axisId = axisId;
            diag.command = QStringLiteral("abs");
            diag.requestedTarget = static_cast<double>(pos);
            diag.settleMs = diagSettleMs;
            diag.start = captureAxisMoveDiagnosticSnapshot(controller, axisId);
            diag.phase = diag.start.linearReferenced ? QStringLiteral("POST_HOME") : QStringLiteral("PRE_HOME");
            diag.plan = QStringLiteral("MANUAL_UI");
            diag.role = QStringLiteral("manual_ui");
            if (diag.start.valid) diag.requestedDelta = static_cast<double>(pos) - diag.start.dpos;
        }

        QString err;
        const bool ok = controller->moveAbs(axisId, pos, &motion, &err);

        if (diagEnabled) {
            diag.moveOk = ok;
            diag.moveError = err;
            diag.endIdle = captureAxisMoveDiagnosticSnapshot(controller, axisId);
            if (ok && diagSettleMs > 0) QThread::msleep(static_cast<unsigned long>(diagSettleMs));
            diag.endSettled = captureAxisMoveDiagnosticSnapshot(controller, axisId);
        }

        QMetaObject::invokeMethod(this, [this, widget, axisId, ok, err, diagEnabled, diag]() {
            --m_axisWorkerCount;
            if (widget) widget->setControlsEnabled(true);
            // 单轴定位完成/失败后只检查本轴一次状态。
            pollAxisAlarmState(axisId, false);
            if (!m_closeRequested) {
                if (diagEnabled) appendAxisMoveDiagnosticRecord(diag);
                if (!ok) {
                    QMessageBox::warning(this, tr("绝对运动失败"),
                        err.isEmpty() ? tr("未知错误") : err);
                }
                else {
                    statusBar()->showMessage(tr("轴 %1 绝对定位完成").arg(axisId), 3000);
                }
            }
            tryFinishDeferredClose();
            });
        });
}

void MainWindow::onAxisRelMoveRequested(int axisId, float delta, const AxisParams& params)
{
    if (!m_zmc->isConnected()) {
        QMessageBox::warning(this, tr("提示"), tr("请先连接控制器"));
        return;
    }

    AxisControlWidget* widget = axisWidgetForId(axisId);

    if (widget) widget->setControlsEnabled(false);

    AxisParams motion = params;
    const bool diagEnabled = axisId <= AxisZ && m_checkAxisMoveDiagAuto && m_checkAxisMoveDiagAuto->isChecked();
    const int diagSettleMs = m_spinAxisMoveDiagSettleMs ? m_spinAxisMoveDiagSettleMs->value() : 2000;

    ++m_axisWorkerCount;
    ZmcController* controller = m_zmc.get();
    (void)QtConcurrent::run([this, controller, axisId, delta, motion, widget, diagEnabled, diagSettleMs]() {
        AxisMoveDiagnosticRecord diag;
        if (diagEnabled) {
            diag.axisId = axisId;
            diag.command = QStringLiteral("rel");
            diag.requestedDelta = static_cast<double>(delta);
            diag.settleMs = diagSettleMs;
            diag.start = captureAxisMoveDiagnosticSnapshot(controller, axisId);
            diag.phase = diag.start.linearReferenced ? QStringLiteral("POST_HOME") : QStringLiteral("PRE_HOME");
            diag.plan = QStringLiteral("MANUAL_UI");
            diag.role = QStringLiteral("manual_ui");
            if (diag.start.valid) diag.requestedTarget = diag.start.dpos + static_cast<double>(delta);
        }

        QString err;
        const bool ok = controller->moveRel(axisId, delta, &motion, &err);

        if (diagEnabled) {
            diag.moveOk = ok;
            diag.moveError = err;
            diag.endIdle = captureAxisMoveDiagnosticSnapshot(controller, axisId);
            if (ok && diagSettleMs > 0) QThread::msleep(static_cast<unsigned long>(diagSettleMs));
            diag.endSettled = captureAxisMoveDiagnosticSnapshot(controller, axisId);
        }

        QMetaObject::invokeMethod(this, [this, widget, axisId, ok, err, diagEnabled, diag]() {
            --m_axisWorkerCount;
            if (widget) widget->setControlsEnabled(true);
            // 单轴定位完成/失败后只检查本轴一次状态。
            pollAxisAlarmState(axisId, false);
            if (!m_closeRequested) {
                if (diagEnabled) appendAxisMoveDiagnosticRecord(diag);
                if (!ok) {
                    QMessageBox::warning(this, tr("相对运动失败"),
                        err.isEmpty() ? tr("未知错误") : err);
                }
                else {
                    statusBar()->showMessage(tr("轴 %1 相对运动完成").arg(axisId), 3000);
                }
            }
            tryFinishDeferredClose();
            });
        });
}

void MainWindow::onAxisJogRequested(int axisId, bool positive, const AxisParams& params)
{
    if (!m_zmc->isConnected()) return;

    QString err;
    if (!m_zmc->jog(axisId, positive, params, true, 0.0f, &err)) {
        pollAxisAlarmState(axisId, false);
        QMessageBox::warning(this, tr("点动失败"), err);
    }
}

void MainWindow::onAxisStopRequested(int axisId)
{
    if (!m_zmc->isConnected()) return;
    if (!m_zmc->requestStopAxis(axisId, false)) return;
    statusBar()->showMessage(tr("停止请求已排入高优先级队列；请确认轴实际停稳"), 3000);
}

void MainWindow::onRotaryServoEnableRequested(int axisId, bool enable)
{
    if (!m_zmc->isConnected()) {
        QMessageBox::warning(this, tr("提示"), tr("请先连接控制器"));
        return;
    }
    if (axisId != AxisB && axisId != AxisC) return;

    const QString axisName = axisId == AxisB ? QStringLiteral("B") : QStringLiteral("C");
    AxisControlWidget* widget = axisWidgetForId(axisId);

    if (enable && m_zmc->isAxisBusy(axisId)) {
        QMessageBox::warning(this, tr("不能使能"), tr("%1轴仍有运动任务未结束").arg(axisName));
        return;
    }

    // 人工断使能不允许“运动中直接掉使能”。若轴还在动，先只发减速停止，
    // 等界面显示停止后由操作员再次点击“断使能”。
    if (!enable) {
        float pos = 0.0f;
        bool idle = false;
        QString stateErr;
        if (!m_zmc->getAxisStatus(axisId, pos, idle, &stateErr)) {
            QMessageBox::warning(this, tr("无法断使能"), stateErr);
            return;
        }
        if (!idle) {
            QString stopErr;
            if (!m_zmc->stopAxis(axisId, false, &stopErr)) {
                QMessageBox::warning(this, tr("停止失败"), stopErr);
            }
            else {
                statusBar()->showMessage(
                    tr("%1轴正在减速停止；确认位置显示为“停止”后再次点击断使能").arg(axisName),
                    5000);
            }
            return;
        }
    }

    QString err;
    if (!m_zmc->setRotaryServoEnable(axisId, enable, &err)) {
        // 若上位机直接写使能输出失败，UI 同步当前本地安全状态。
        m_cAxisWidget->setManualServoState(m_zmc->isRotaryAbsoluteReady(AxisC),
            m_zmc->isRotaryServoEnabled(AxisC));
        m_bAxisWidget->setManualServoState(m_zmc->isRotaryAbsoluteReady(AxisB),
            m_zmc->isRotaryServoEnabled(AxisB));
        QMessageBox::warning(this, enable ? tr("使能失败") : tr("断使能失败"), err);
        return;
    }

    if (widget) {
        widget->setManualServoState(true, enable,
            enable ? tr("绝对位置已同步 / 伺服已人工使能")
            : tr("绝对位置已同步 / 已人工断使能"));
    }
    LOG_INFO(LogCategory::Controller,
        tr("%1轴人工%2，输出 OP(%3)")
        .arg(axisName)
        .arg(enable ? tr("使能") : tr("断使能"))
        .arg(axisId == AxisB ? Constants::kBServoEnableOutput
            : Constants::kCServoEnableOutput));
    statusBar()->showMessage(
        tr("%1轴已%2").arg(axisName, enable ? tr("人工使能") : tr("断使能")), 3000);

    // 人工使能/断使能是明确事件点，只复核本轴一次 AXISSTATUS。
    pollAxisAlarmState(axisId, false);
    if (m_calibrationWidget) m_calibrationWidget->refreshSingleSphereDeviceState();
}

void MainWindow::onResyncRotaryAbsolute()
{
    if (!m_zmc->isConnected()) {
        QMessageBox::warning(this, tr("提示"), tr("请先连接控制器"));
        return;
    }
    if (m_rotaryInitInProgress) return;

    // 重读会由上位机先关闭 B/C 使能输出，因此只有两轴都停稳时才允许开始。
    for (const int axisId : { AxisC, AxisB }) {
        float pos = 0.0f;
        bool idle = false;
        QString stateErr;
        if (!m_zmc->getAxisStatus(axisId, pos, idle, &stateErr)) {
            QMessageBox::warning(this, tr("无法重读绝对位置"), stateErr);
            return;
        }
        if (!idle) {
            QMessageBox::warning(this, tr("无法重读绝对位置"),
                tr("%1轴仍在运动，请先停止并等待停稳")
                .arg(axisId == AxisB ? QStringLiteral("B") : QStringLiteral("C")));
            return;
        }
    }

    // 重读前先重新读取 B/C 运动参数，避免使用陈旧参数。
    QString err;
    if (!m_zmc->readAxisParams(AxisC, m_axisParams[AxisC], &err)
        || !m_zmc->readAxisParams(AxisB, m_axisParams[AxisB], &err)) {
        QMessageBox::warning(this, tr("参数读取失败"),
            tr("B/C 保持断使能，未执行绝对位置重同步。\n%1").arg(err));
        return;
    }
    m_cAxisWidget->setAxisParams(m_axisParams[AxisC]);
    m_bAxisWidget->setAxisParams(m_axisParams[AxisB]);
    fillAxisTableFromParams();

    startRotaryAbsoluteInitialization(true);
}

// ==================== 五轴运动学 P1/P2/P3 + P4 ====================

void MainWindow::onKinematicsLoadCurrent()
{
    if (!m_zmc || !m_zmc->isConnected()) {
        QMessageBox::warning(this, tr("五轴运动学"), tr("请先连接控制器并完成 B/C 绝对位置初始化。"));
        return;
    }

    JointPose5D q;
    QString err;
    if (!m_zmc->readJointPose5D(q, &err)) {
        QMessageBox::warning(this, tr("读取五轴姿态失败"), err);
        return;
    }

    m_kinX->setValue(q.x);
    m_kinY->setValue(q.y);
    m_kinZ->setValue(q.z);
    m_kinC->setValue(q.c);
    m_kinB->setValue(q.b);
    onKinematicsForward();
}

void MainWindow::onKinematicsForward()
{
    if (!m_kinOutput) return;

    JointPose5D q;
    q.x = m_kinX->value();
    q.y = m_kinY->value();
    q.z = m_kinZ->value();
    q.c = m_kinC->value();
    q.b = m_kinB->value();

    MachineKinematics5Axis kinematics;
    const EndPose5D pose = kinematics.forwardPose(q);

    QString text;
    QTextStream ts(&text);
    ts << kinematics.describeModel() << "\n\n";
    ts << QStringLiteral("Controller joints: X=%1 Y=%2 Z=%3 C=%4 B=%5\n")
        .arg(q.x, 0, 'f', 6).arg(q.y, 0, 'f', 6).arg(q.z, 0, 'f', 6)
        .arg(q.c, 0, 'f', 6).arg(q.b, 0, 'f', 6);
    ts << QStringLiteral("Math angles: thetaC=%1 deg, thetaB=%2 deg\n\n")
        .arg(pose.mathCdeg, 0, 'f', 6).arg(pose.mathBdeg, 0, 'f', 6);
    ts << "^M T_E =\n" << formatTransformMatrix(pose.transform);
    m_kinOutput->setPlainText(text);
}

void MainWindow::onKinematicsRoundTrip()
{
    if (!m_kinOutput) return;

    JointPose5D input;
    input.x = m_kinX->value();
    input.y = m_kinY->value();
    input.z = m_kinZ->value();
    input.c = m_kinC->value();
    input.b = m_kinB->value();

    MachineKinematics5Axis kinematics;
    const QMatrix4x4 target = kinematics.forward(input);
    const auto ik = kinematics.inverse(target, true);

    QString text;
    QTextStream ts(&text);
    ts << "FK target ^M T_E:\n" << formatTransformMatrix(target) << "\n";
    if (!ik.success) {
        ts << "IK FAILED: " << ik.error << "\n";
    }
    else {
        ts << QStringLiteral("IK recovered: X=%1 Y=%2 Z=%3 C=%4 B=%5\n")
            .arg(ik.joints.x, 0, 'f', 9).arg(ik.joints.y, 0, 'f', 9)
            .arg(ik.joints.z, 0, 'f', 9).arg(ik.joints.c, 0, 'f', 9)
            .arg(ik.joints.b, 0, 'f', 9);
        ts << QStringLiteral("rotation residual=%1, position residual=%2\n")
            .arg(ik.rotationResidual, 0, 'g', 10).arg(ik.positionResidual, 0, 'g', 10);
        double maxJointResidual = 0.0;
        const double jointResiduals[5] = {
            std::abs(input.x - ik.joints.x),
            std::abs(input.y - ik.joints.y),
            std::abs(input.z - ik.joints.z),
            std::abs(MachineKinematics5Axis::normalizeDeg(input.c - ik.joints.c)),
            std::abs(MachineKinematics5Axis::normalizeDeg(input.b - ik.joints.b))
        };
        for (int i = 0; i < 5; ++i) {
            if (jointResiduals[i] > maxJointResidual) {
                maxJointResidual = jointResiduals[i];
            }
        }
        ts << QStringLiteral("max joint round-trip residual=%1\n")
            .arg(maxJointResidual, 0, 'g', 10);
        ts << (maxJointResidual < 1e-5 ? "SELF TEST: PASS\n" : "SELF TEST: CHECK\n");
    }
    m_kinOutput->setPlainText(text);
}


std::array<AxisParams, Constants::kAxisParamArraySize> MainWindow::currentAxisParams5D() const
{
    std::array<AxisParams, Constants::kAxisParamArraySize> params;
    for (int i = 0; i < Constants::kAxisParamArraySize; ++i) {
        params[i] = m_axisParams[i];
    }
    return params;
}

bool MainWindow::buildCurrentJointMovePlan(JointMotionPlan5D& plan, QString* err)
{
    plan = JointMotionPlan5D{};
    if (!m_zmc || !m_zmc->isConnected()) {
        if (err) *err = tr("请先连接控制器");
        return false;
    }
    if (m_jointMoveInProgress) {
        if (err) *err = tr("已有五轴同步定位正在执行");
        return false;
    }
    if (m_syncManager && m_syncManager->isRunning()) {
        if (err) *err = tr("在线扫描正在运行，禁止启动 P4 五轴人工同步定位");
        return false;
    }
    if (m_gcodeExecutor && m_gcodeExecutor->isRunning()) {
        if (err) *err = tr("G代码正在执行，禁止启动 P4 五轴人工同步定位");
        return false;
    }
    if (!m_zmc->isRotaryAbsoluteReady(AxisC) || !m_zmc->isRotaryAbsoluteReady(AxisB)) {
        if (err) *err = tr("B/C 绝对机械坐标尚未建立，请先完成绝对位置初始化");
        return false;
    }

    JointState5D state;
    if (!m_zmc->readJointState5D(state, err)) {
        return false;
    }
    if (!state.allIdle) {
        if (err) *err = tr("至少一个轴仍在运动，请先停止所有轴再规划五轴定位");
        return false;
    }

    JointPose5D target;
    target.x = m_kinX->value();
    target.y = m_kinY->value();
    target.z = m_kinZ->value();
    target.c = m_kinC->value();
    target.b = m_kinB->value();

    const double scale = m_kinSpeedScale ? m_kinSpeedScale->value() / 100.0 : 0.20;
    JointPose5D current = state.mpos;
    plan = JointTrajectoryPlanner5Axis::planPointToPoint(
        current, target, currentAxisParams5D(), scale);
    if (!plan.valid) {
        if (err) *err = plan.error;
        return false;
    }
    return true;
}

void MainWindow::setJointMoveUiRunning(bool running)
{
    m_jointMoveInProgress = running;
    const bool workflowLocksMotion = running || m_sphereStudyAutoActive || m_hardwareTemporalScanActive;
    if (m_btnKinPlan) m_btnKinPlan->setEnabled(!workflowLocksMotion);
    if (m_btnKinMove) m_btnKinMove->setEnabled(!workflowLocksMotion);
    if (m_btnKinStop) m_btnKinStop->setEnabled(running);
    if (m_kinSpeedScale) m_kinSpeedScale->setEnabled(!workflowLocksMotion);

    // During the automatic sphere study, manual axis commands remain locked
    // not only while the controller is moving but also during settle/capture.
    // This guarantees that one Visit has one stationary XYZCB MPOS pose.
    if (m_xAxisWidget) m_xAxisWidget->setControlsEnabled(!workflowLocksMotion);
    if (m_yAxisWidget) m_yAxisWidget->setControlsEnabled(!workflowLocksMotion);
    if (m_zAxisWidget) m_zAxisWidget->setControlsEnabled(!workflowLocksMotion);
    if (m_cAxisWidget) m_cAxisWidget->setControlsEnabled(!workflowLocksMotion);
    if (m_bAxisWidget) m_bAxisWidget->setControlsEnabled(!workflowLocksMotion);

    if (m_btnStartMerge) m_btnStartMerge->setEnabled(!workflowLocksMotion);
    if (m_btnStartCapture) m_btnStartCapture->setEnabled(!workflowLocksMotion);
    if (m_gcodeWidget) m_gcodeWidget->setEnabled(!workflowLocksMotion);
    updateSphereStudyUiState();
}

void MainWindow::onKinematicsPlanJointMove()
{
    JointMotionPlan5D plan;
    QString err;
    if (!buildCurrentJointMovePlan(plan, &err)) {
        QMessageBox::warning(this, tr("五轴规划失败"), err);
        if (m_kinPlanSummary) m_kinPlanSummary->setText(tr("规划失败: %1").arg(err));
        return;
    }

    const QString summary = tr("主轴=%1；倍率=%2%；控制器关节度量=%3；V=%4；A=%5；D=%6；预计=%7 s%8")
        .arg(axisName5D(plan.baseAxis))
        .arg(plan.speedScale * 100.0, 0, 'f', 1)
        .arg(plan.controllerMetricLength, 0, 'f', 4)
        .arg(plan.vectorSpeed, 0, 'f', 4)
        .arg(plan.vectorAccel, 0, 'f', 4)
        .arg(plan.vectorDecel, 0, 'f', 4)
        .arg(plan.estimatedDurationSec, 0, 'f', 3)
        .arg(plan.hasMotion ? QString() : tr("；已在目标位置"));
    if (m_kinPlanSummary) m_kinPlanSummary->setText(summary);

    QString text;
    QTextStream ts(&text);
    ts << "P4 five-axis joint-space plan\n";
    ts << QStringLiteral("start : X=%1 Y=%2 Z=%3 C=%4 B=%5\n")
        .arg(plan.start.x, 0, 'f', 6).arg(plan.start.y, 0, 'f', 6).arg(plan.start.z, 0, 'f', 6)
        .arg(plan.start.c, 0, 'f', 6).arg(plan.start.b, 0, 'f', 6);
    ts << QStringLiteral("target: X=%1 Y=%2 Z=%3 C=%4 B=%5\n")
        .arg(plan.target.x, 0, 'f', 6).arg(plan.target.y, 0, 'f', 6).arg(plan.target.z, 0, 'f', 6)
        .arg(plan.target.c, 0, 'f', 6).arg(plan.target.b, 0, 'f', 6);
    ts << summary << "\n\n";
    ts << "注意：这里的 controller joint metric / V/A/D 是 ZMotion 多轴数值插补参数，\n"
        "不是 TCP 的 mm 距离或 mm/s 速度。真正 TCP 轨迹将在后续阶段实现。\n";
    if (m_kinOutput) m_kinOutput->setPlainText(text);
}

void MainWindow::onKinematicsExecuteJointMove()
{
    JointMotionPlan5D plan;
    QString err;
    if (!buildCurrentJointMovePlan(plan, &err)) {
        QMessageBox::warning(this, tr("不能执行五轴定位"), err);
        return;
    }

    if (!m_zmc->isRotaryServoEnabled(AxisC) || !m_zmc->isRotaryServoEnabled(AxisB)) {
        QMessageBox::warning(this, tr("不能执行五轴定位"),
            tr("C轴和B轴都必须先人工使能。五轴联动不会绕过旋转轴安全联锁。"));
        return;
    }

    const QString confirm = tr(
        "即将执行五轴关节空间同步绝对定位（不是TCP直线）。\n\n"
        "目标：X=%1  Y=%2  Z=%3  C=%4°  B=%5°\n"
        "速度倍率：%6%\n预计时间：%7 s\n\n是否继续？")
        .arg(plan.target.x, 0, 'f', 3).arg(plan.target.y, 0, 'f', 3).arg(plan.target.z, 0, 'f', 3)
        .arg(plan.target.c, 0, 'f', 3).arg(plan.target.b, 0, 'f', 3)
        .arg(plan.speedScale * 100.0, 0, 'f', 1)
        .arg(plan.estimatedDurationSec, 0, 'f', 2);
    if (QMessageBox::question(this, tr("确认五轴同步定位"), confirm,
        QMessageBox::Yes | QMessageBox::No, QMessageBox::No)
        != QMessageBox::Yes) {
        return;
    }

    setJointMoveUiRunning(true);
    if (m_kinPlanSummary) {
        m_kinPlanSummary->setText(tr("五轴同步定位执行中... 主轴=%1").arg(axisName5D(plan.baseAxis)));
    }
    ++m_axisWorkerCount;

    ZmcController* controller = m_zmc.get();
    (void)QtConcurrent::run([this, controller, plan]() {
        JointMoveResult5D result;
        QString moveErr;
        const bool ok = controller->moveJointAbs5D(plan, result, &moveErr);

        QMetaObject::invokeMethod(this, [this, plan, result, ok, moveErr]() {
            --m_axisWorkerCount;
            setJointMoveUiRunning(false);

            // P4 运动结束/失败后只做一次事件式五轴状态检查，不增加周期轮询。
            pollAxisAlarmStates(false);

            QString text;
            QTextStream ts(&text);
            ts << "P4 five-axis joint-space execution\n";
            ts << QStringLiteral("target: X=%1 Y=%2 Z=%3 C=%4 B=%5\n")
                .arg(plan.target.x, 0, 'f', 6).arg(plan.target.y, 0, 'f', 6).arg(plan.target.z, 0, 'f', 6)
                .arg(plan.target.c, 0, 'f', 6).arg(plan.target.b, 0, 'f', 6);

            if (!ok) {
                ts << "FAILED: " << moveErr << "\n";
                if (m_kinOutput) m_kinOutput->setPlainText(text);
                if (m_kinPlanSummary) m_kinPlanSummary->setText(tr("执行失败: %1").arg(moveErr));
                if (!m_closeRequested) {
                    QMessageBox::warning(this, tr("五轴同步定位失败"),
                        moveErr.isEmpty() ? tr("未知错误") : moveErr);
                }
                tryFinishDeferredClose();
                return;
            }

            JointPose5D actual;
            actual.x = result.finalState.mpos.x;
            actual.y = result.finalState.mpos.y;
            actual.z = result.finalState.mpos.z;
            actual.c = result.finalState.mpos.c;
            actual.b = result.finalState.mpos.b;

            MachineKinematics5Axis kin;
            const QMatrix4x4 targetTf = kin.forward(plan.target);
            const QMatrix4x4 actualTf = kin.forward(actual);
            const double posErr = transformPositionErrorMm(targetTf, actualTf);
            const double oriErr = transformOrientationErrorDeg(targetTf, actualTf);

            ts << QStringLiteral("final measured pose: XYZ_MPOS=[%1,%2,%3] mm, C/B_MPOS=[%4,%5] deg\n")
                .arg(result.finalState.mpos.x, 0, 'f', 6).arg(result.finalState.mpos.y, 0, 'f', 6)
                .arg(result.finalState.mpos.z, 0, 'f', 6).arg(result.finalState.mpos.c, 0, 'f', 6)
                .arg(result.finalState.mpos.b, 0, 'f', 6);
            ts << QStringLiteral("joint errors: XYZ DPOS=%1 mm, XYZ MPOS=%2 mm, C/B MPOS=%3 deg\n")
                .arg(result.maxLinearDposErrorMm, 0, 'g', 8)
                .arg(result.maxLinearMposErrorMm, 0, 'g', 8)
                .arg(result.maxRotaryMposErrorDeg, 0, 'g', 8);
            ts << QStringLiteral("FK verification: position error=%1 mm, orientation error=%2 deg\n")
                .arg(posErr, 0, 'g', 8).arg(oriErr, 0, 'g', 8);
            ts << "actual ^M T_E:\n" << formatTransformMatrix(actualTf);

            if (m_kinOutput) m_kinOutput->setPlainText(text);
            if (m_kinPlanSummary) {
                m_kinPlanSummary->setText(
                    tr("完成；FK位置误差=%1 mm，姿态误差=%2°")
                    .arg(posErr, 0, 'g', 6).arg(oriErr, 0, 'g', 6));
            }

            const bool verificationOk =
                result.maxLinearDposErrorMm <= 0.10 &&
                result.maxRotaryMposErrorDeg <= 0.10 &&
                posErr <= 0.10 && oriErr <= 0.10;

            if (!m_closeRequested) {
                if (!verificationOk) {
                    QMessageBox::warning(this, tr("五轴定位完成但校验超差"),
                        tr("运动已结束，但软件层校验超过当前 0.10 阈值。\n"
                            "位置误差=%1 mm，姿态误差=%2°。\n请检查轴状态、反馈和零点后再继续。")
                        .arg(posErr, 0, 'g', 8).arg(oriErr, 0, 'g', 8));
                }
                else {
                    statusBar()->showMessage(tr("五轴同步定位完成，FK回读校验通过"), 4000);
                }
            }
            tryFinishDeferredClose();
            });
        });
}

void MainWindow::onKinematicsStopJointMove()
{
    if (!m_zmc || !m_zmc->isConnected() || !m_jointMoveInProgress) return;
    if (!m_zmc->requestStopJointMove5D(false)) return;
    if (m_kinPlanSummary) m_kinPlanSummary->setText(tr("五轴停止请求已入队，等待执行并确认各轴停稳..."));
    statusBar()->showMessage(tr("五轴停止请求已入队（不代表已发送或已停稳）"), 3000);
}

// ==================== 轨迹插补 ====================

bool MainWindow::buildPathFromTable(std::vector<PathPoint>& outPath)
{
    outPath.clear();
    int rows = m_pathTable->rowCount();

    for (int r = 0; r < rows; ++r) {
        auto getCell = [&](int c) -> QString {
            QTableWidgetItem* item = m_pathTable->item(r, c);
            return item ? item->text().trimmed() : QString();
            };

        bool okX = false, okY = false, okZ = false, okF = false;
        float x = getCell(0).toFloat(&okX);
        float y = getCell(1).toFloat(&okY);
        float z = getCell(2).toFloat(&okZ);
        QString feedStr = getCell(3);
        float feed = feedStr.isEmpty() ? 0.0f : feedStr.toFloat(&okF);

        if (!(okX && okY && okZ)) continue;
        if (!okF && !feedStr.isEmpty()) feed = 0.0f;

        PathPoint pt;
        pt.x = x;
        pt.y = y;
        pt.z = z;
        pt.feed = feed;
        outPath.push_back(pt);
    }
    return !outPath.empty();
}

void MainWindow::onLoadPathFromFile()
{
    QString fileName = QFileDialog::getOpenFileName(
        this, tr("选择轨迹文件"), QString(),
        tr("文本文件 (*.txt *.csv);;所有文件 (*.*)"));
    if (fileName.isEmpty()) return;

    QFile f(fileName);
    if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) {
        QMessageBox::warning(this, tr("打开失败"),
            tr("无法打开文件: %1").arg(fileName));
        return;
    }

    m_pathTable->setRowCount(0);

    QTextStream in(&f);
    int row = 0;
    while (!in.atEnd()) {
        QString line = in.readLine().trimmed();
        if (line.isEmpty() || line.startsWith('#')) continue;

        QChar sep = line.contains(',') ? QChar(',') : QChar(' ');
        QStringList parts = line.split(sep, Qt::SkipEmptyParts);
        if (parts.size() < 3) continue;

        m_pathTable->insertRow(row);
        for (int c = 0; c < 4 && c < parts.size(); ++c) {
            m_pathTable->setItem(row, c, new QTableWidgetItem(parts[c].trimmed()));
        }
        ++row;
    }

    QMessageBox::information(this, tr("导入完成"),
        tr("已导入 %1 个轨迹点").arg(row));
}

void MainWindow::onAddPathPoint()
{
    int row = m_pathTable->rowCount();
    m_pathTable->insertRow(row);
    for (int c = 0; c < 4; ++c) {
        m_pathTable->setItem(row, c, new QTableWidgetItem("0"));
    }
}

void MainWindow::onRemovePathPoint()
{
    auto ranges = m_pathTable->selectedRanges();
    if (ranges.isEmpty()) return;

    int firstRow = ranges.first().topRow();
    int lastRow = ranges.first().bottomRow();
    for (int r = lastRow; r >= firstRow; --r) {
        m_pathTable->removeRow(r);
    }
}

void MainWindow::onStartMerge()
{
    if (!m_zmc->isConnected()) {
        QMessageBox::warning(this, tr("提示"), tr("请先连接控制器"));
        return;
    }

    std::vector<PathPoint> path;
    if (!buildPathFromTable(path)) {
        QMessageBox::warning(this, tr("轨迹为空"),
            tr("请先添加轨迹点或从文件导入"));
        return;
    }

    AxisParams motion = m_axisParams[AxisX];

    QString err;
    if (!m_zmc->prepareMergePath(path, motion, &err)) {
        QMessageBox::warning(this, tr("连续插补初始化失败"), err);
        return;
    }

    statusBar()->showMessage(tr("连续插补已启动"), 3000);
}

void MainWindow::onStopMerge()
{
    if (m_zmc->isConnected() && m_zmc->requestStopAxis(AxisX, false))
        statusBar()->showMessage(tr("连续路径停止请求已入队；尚未确认停稳"), 3000);
}

// ==================== 传感器 ====================

void MainWindow::onSensorConnect()
{
    ++m_controllerSensorConnectionGeneration;
    m_geometryNcProof.passed=false; m_hardwareNcProof.passed=false;
    if (m_closeRequested || m_sensorConnectInProgress) return;

    LOG_INFO(LogCategory::Sensor, tr("尝试连接传感器"));
    m_sensorConnectInProgress = true;
    if (m_btnSensorConnect) m_btnSensorConnect->setEnabled(false);
    if (m_btnSensorDisconnect) m_btnSensorDisconnect->setEnabled(false);
    m_lblSensorStatus->setText(tr("连接中..."));
    m_lblSensorStatus->setStyleSheet("QLabel { color: #cc8800; font-weight: bold; }");
    statusBar()->showMessage(tr("正在后台搜索并连接传感器..."));

    auto result = std::make_shared<SensorConnectResult>();
    LaserSensor* laser = m_laser.get();

    auto* watcher = new QFutureWatcher<void>(this);
    m_sensorConnectWatcher = watcher;
    connect(watcher, &QFutureWatcher<void>::finished, this,
        [this, watcher, result]() {
            m_sensorConnectInProgress = false;
            if (m_sensorConnectWatcher == watcher) m_sensorConnectWatcher = nullptr;
            watcher->deleteLater();
            if (m_calibrationWidget) m_calibrationWidget->refreshSingleSphereDeviceState();

            if (m_closeRequested) {
                if (result->ok && m_laser->isConnected()) m_laser->disconnectSensor();
                tryFinishDeferredClose();
                return;
            }

            if (!result->ok) {
                LOG_ERROR(LogCategory::Sensor, tr("传感器连接失败: %1").arg(result->error));
                QMessageBox::warning(this, tr("连接失败"), result->error);
                m_lblSensorStatus->setText(tr("失败"));
                m_lblSensorStatus->setStyleSheet("QLabel { color: red; font-weight: bold; }");
                if (m_btnSensorConnect) m_btnSensorConnect->setEnabled(true);
                if (m_btnSensorDisconnect) m_btnSensorDisconnect->setEnabled(false);
                updateVideoControlState();
                return;
            }

            m_comboResolution->clear();
            const auto& res = m_laser->availableResolutions();
            for (int i = 0; i < res.size(); ++i) {
                m_comboResolution->addItem(QString::number(res[i]), res[i]);
            }
            if (!res.isEmpty()) m_comboResolution->setCurrentIndex(0);

            LOG_INFO(LogCategory::Sensor, tr("传感器连接成功: %1, 可用分辨率: %2种")
                .arg(m_laser->sensorTypeName())
                .arg(res.size()));
            m_lblSensorStatus->setText(tr("已连接 (%1)").arg(m_laser->sensorTypeName()));
            m_lblSensorStatus->setStyleSheet("QLabel { color: green; font-weight: bold; }");
            statusBar()->showMessage(tr("传感器已连接: %1").arg(m_laser->sensorTypeName()), 3000);
            if (m_btnSensorConnect) m_btnSensorConnect->setEnabled(false);
            if (m_btnSensorDisconnect) m_btnSensorDisconnect->setEnabled(true);
            updateVideoControlState();
        });

    watcher->setFuture(QtConcurrent::run([laser, result]() {
        QString err;
        result->ok = laser->connectSensor("LLT.dll", &err);
        result->error = err;
        }));
}

void MainWindow::onSensorDisconnect()
{
    ++m_controllerSensorConnectionGeneration;
    m_geometryNcProof.passed=false; m_hardwareNcProof.passed=false;
    if(m_small7Active){
        onStopSmall7Experiment();
        QMessageBox::warning(this,tr("实验3尚未结束"),
            tr("已请求停止X/Z运动与采集；请等待工作线程退出再断开传感器"));
        return;
    }
    if (m_sphereExpActive) {
        onStopSphereExposureSweep();
        QMessageBox::warning(this, tr("全球曝光扫描仍在收尾"), tr("已请求停止运动与采集。请等待运动任务退出后再断开传感器。"));
        return;
    }
    if (m_sensorConnectInProgress) {
        statusBar()->showMessage(tr("传感器连接任务进行中，请等待完成后再断开"), 3000);
        return;
    }
    if (m_sphereStudyAutoActive && m_jointMoveInProgress) {
        onSphereStudyAutoStop();
        QMessageBox::warning(this, tr("暂不能断开传感器"),
            tr("自动球面研究正在执行五轴移动，已请求停止。请等待运动结束后再断开传感器。"));
        return;
    }
    if (m_sphereStudyAutoActive) {
        onSphereStudyAutoStop();
    }
    else if (m_sphereStudyCaptureActive) {
        finishSphereStudyCapture(true, tr("传感器断开，终止球面距离/角度研究采集"));
    }
    LOG_INFO(LogCategory::Sensor, tr("断开传感器连接"));

    // 停止所有采集
    m_syncManager->stop();
    m_laser->stopAll();
    leaveSensorCalibrationMeasuringField();
    m_laser->disconnectSensor();
    if (m_calibrationWidget) m_calibrationWidget->refreshSingleSphereDeviceState();

    // 重置轮廓预览状态
    m_profilePreviewMirrorsCapture = false;
    m_profilePreviewMirrorBaselineOrdinal = 0;
    m_profilePreviewMirrorFirstFrameSeen = false;
    m_profilePreviewRunning = false;
    if (m_lblProfilePreviewStatus) {
        m_lblProfilePreviewStatus->setText(tr("未启动"));
        m_lblProfilePreviewStatus->setStyleSheet("QLabel { font-weight: bold; }");
    }
    if (m_profilePreviewView) {
        m_profilePreviewView->clear();
    }

    m_comboResolution->clear();
    m_lblCaptureStatus->setText(tr("未采集"));
    m_lblSensorStatus->setText(tr("未连接"));
    m_lblSensorStatus->setStyleSheet("QLabel { color: red; font-weight: bold; }");

    // 更新 Video 状态
    m_lblVideoStatus->setText(tr("未启动"));
    m_lblVideoStatus->setStyleSheet("QLabel { font-weight: bold; }");

    // 清空 Video 显示
    if (m_videoWidget) {
        m_videoWidget->clear();
    }

    // 更新控制按钮状态
    if (m_btnSensorConnect) m_btnSensorConnect->setEnabled(true);
    if (m_btnSensorDisconnect) m_btnSensorDisconnect->setEnabled(false);
    updateVideoControlState();
}

void MainWindow::onReadSensorParams()
{
    if (!m_laser->isConnected()) {
        QMessageBox::warning(this, tr("提示"), tr("请先连接传感器"));
        return;
    }

    LOG_INFO(LogCategory::Sensor, tr("正在从传感器读取参数..."));

    QString errMsg;
    SensorAdvancedParams params;
    if (!m_laser->readAdvancedParams(params, &errMsg)) {
        QMessageBox::warning(this, tr("读取失败"), errMsg);
        return;
    }

    // 测量区域
    for (int i = 0; i < m_comboMeasuringField->count(); ++i) {
        if (m_comboMeasuringField->itemData(i).toInt() == params.measuringFieldIndex) {
            m_comboMeasuringField->setCurrentIndex(i);
            break;
        }
    }

    // 激光功率
    for (int i = 0; i < m_comboLaserPower->count(); ++i) {
        if (m_comboLaserPower->itemData(i).toInt() == params.laserPower) {
            m_comboLaserPower->setCurrentIndex(i);
            break;
        }
    }

    // 反射选择
    for (int i = 0; i < m_comboReflection->count(); ++i) {
        if (m_comboReflection->itemData(i).toInt() == params.reflectionSelect) {
            m_comboReflection->setCurrentIndex(i);
            break;
        }
    }

    // 阈值
    m_spinThreshold->setValue(params.thresholdPercent);

    // 翻转选项
    m_checkFlipX->setChecked(params.flipX);
    m_checkFlipZ->setChecked(params.flipZ);

    // 自动曝光
    m_checkAutoExposure->setChecked(params.autoExposure);
    for (int i = 0; i < m_comboShutterAlign->count(); ++i) {
        if (m_comboShutterAlign->itemData(i).toInt() == params.shutterAlign) {
            m_comboShutterAlign->setCurrentIndex(i);
            break;
        }
    }
    for (int i = 0; i < m_comboAutoExpMode->count(); ++i) {
        if (m_comboAutoExpMode->itemData(i).toInt() == params.autoExpMode) {
            m_comboAutoExpMode->setCurrentIndex(i);
            break;
        }
    }

    // 有效反射(宽度/强度)在当前2950-50/BL + SDK/固件组合上无法可靠回读。
    // 保留界面现值，不再反复尝试读取；写入时仍按用户设置发送，并以SetFeature/命令返回值判断成功。
    LOG_INFO(LogCategory::Sensor, tr("有效反射宽度/强度为写入型参数：当前2950实机无法可靠回读，本次读取保留界面现值。"));

    // 处理选项
    m_checkHighResolution->setChecked(params.highResolution);
    m_checkCalibration->setChecked(params.calibration);
    m_checkAmbientLight->setChecked(params.ambientLightSuppression);
    m_checkVideoFilter->setChecked(params.videoFilter);

    LOG_INFO(LogCategory::Sensor, tr("传感器参数读取完成"));
    statusBar()->showMessage(tr("传感器参数读取完成"), 3000);
}

void MainWindow::onWriteSensorParams()
{
    if (!m_laser->isConnected()) {
        QMessageBox::warning(this, tr("提示"), tr("请先连接传感器"));
        return;
    }

    SensorAdvancedParams params;

    // 测量区域
    params.measuringFieldIndex = m_comboMeasuringField->currentData().toInt();

    // 分辨率使用当前传感器的值
    params.resolution = m_laser->activeResolution();

    params.laserPower = m_comboLaserPower->currentData().toInt();
    params.reflectionSelect = m_comboReflection->currentData().toInt();
    params.thresholdPercent = m_spinThreshold->value();
    params.flipX = m_checkFlipX->isChecked();
    params.flipZ = m_checkFlipZ->isChecked();
    params.autoExposure = m_checkAutoExposure->isChecked();
    params.shutterAlign = m_comboShutterAlign->currentData().toInt();
    params.autoExpMode = m_comboAutoExpMode->currentData().toInt();
    params.peakWidthMin = m_spinPeakWidthMin->value();
    params.peakWidthMax = m_spinPeakWidthMax->value();
    params.peakIntensityMin = m_spinPeakIntensityMin->value();
    params.peakIntensityMax = m_spinPeakIntensityMax->value();
    params.highResolution = m_checkHighResolution->isChecked();
    params.calibration = m_checkCalibration->isChecked();
    params.ambientLightSuppression = m_checkAmbientLight->isChecked();
    params.videoFilter = m_checkVideoFilter->isChecked();

    LOG_INFO(LogCategory::Sensor, tr("正在写入传感器参数..."));

    QString errMsg;
    if (!m_laser->applyAdvancedParams(params, &errMsg)) {
        QMessageBox::warning(this, tr("写入失败"), errMsg);
        return;
    }

    LOG_INFO(LogCategory::Sensor, tr("传感器参数写入完成"));
    statusBar()->showMessage(tr("传感器参数写入完成"), 3000);
}

void MainWindow::onResetSensorParams()
{
    // 恢复所有参数为默认值
    m_comboMeasuringField->setCurrentIndex(2);   // Small (7) - 552Hz
    if (m_spinHardwareSyncFrequency) m_spinHardwareSyncFrequency->setValue(mls::runtime::Table5D::defaultTriggerFrequencyHz);
    updateHardwareSyncFrequencyLimit();
    m_comboLaserPower->setCurrentIndex(2);       // 标准
    m_comboReflection->setCurrentIndex(3);       // 最大面积
    m_spinThreshold->setValue(12.5);             // 12.5%
    m_checkFlipX->setChecked(false);             // 不翻转X
    m_checkFlipZ->setChecked(true);              // 翻转Z（官方默认）
    m_checkAutoExposure->setChecked(false);      // 默认关闭自动曝光
    m_comboShutterAlign->setCurrentIndex(1);     // 快门对齐：右
    m_comboAutoExpMode->setCurrentIndex(1);      // 模式：轮廓
    m_spinPeakWidthMin->setValue(2);             // 最小宽度
    m_spinPeakWidthMax->setValue(256);           // 最大宽度
    m_spinPeakIntensityMin->setValue(0);         // 最小强度
    m_spinPeakIntensityMax->setValue(1023);      // 最大强度
    m_checkHighResolution->setChecked(true);     // 高分辨率
    m_checkCalibration->setChecked(true);        // 校准
    m_checkAmbientLight->setChecked(true);       // 抑制背光
    m_checkVideoFilter->setChecked(true);        // 视频滤波

    LOG_INFO(LogCategory::Sensor, tr("参数已恢复为默认值"));
    statusBar()->showMessage(tr("参数已恢复为默认值（点击\"写入传感器\"生效）"), 3000);
}

void MainWindow::onStartCapture()
{
    if(m_gcodeHardwareNcProbeActive){
        QMessageBox::warning(this,tr("采集冲突"),tr("NC硬件同步试采运行中，先停止试采"));return;
    }
    // 1. 基本检查：传感器是否连接
    if (!m_laser->isConnected()) {
        LOG_WARNING(LogCategory::Sync, tr("启动采集失败: 传感器未连接"));
        QMessageBox::warning(this, tr("错误"), tr("请先连接传感器"));
        updateVideoControlState();
        return;
    }

    // 软件同步同样依赖控制器五轴姿态。先失败，避免启动失败前清空已有视图。
    if (!m_zmc || !m_zmc->isConnected()) {
        LOG_WARNING(LogCategory::Sync, tr("启动采集失败: 控制器未连接"));
        QMessageBox::warning(this, tr("错误"), tr("控制器未连接，无法建立五轴位姿同步。"));
        updateVideoControlState();
        return;
    }

    // 2. 检查 Video 模式是否正在运行
    if (m_laser->isVideoRunning()) {
        LOG_WARNING(LogCategory::Sync, tr("启动采集失败: Video预览正在运行"));
        QMessageBox::warning(this, tr("模式冲突"),
            tr("Video 预览正在运行。\n请先停止预览，再启动采集。"));
        updateVideoControlState();
        return;
    }

    // 3. 检查轮廓预览是否正在运行
    if (m_profilePreviewRunning) {
        LOG_WARNING(LogCategory::Sync, tr("启动采集失败: 轮廓预览正在运行"));
        QMessageBox::warning(this, tr("模式冲突"),
            tr("轮廓预览正在运行。\n请先停止轮廓预览，再启动采集。"));
        updateVideoControlState();
        return;
    }

    // 4. 检查是否已经在采集
    if (m_syncManager->isRunning()) {
        QMessageBox::information(this, tr("提示"), tr("采集已在运行中"));
        updateVideoControlState();
        return;
    }

    // 5. 选择同步模式。V32.13只解锁“手动Profile开始采集”的SINGLE-OP14五轴MPOS硬件同步；
    // G-code硬件执行仍由其独立门控保持关闭。硬件模式在每次启动前重新做
    // 只读预检，防止UI上一次通过的结果跨BASIC/会话/B-C绝对坐标框架复用。
    const SyncMode mode = (m_radioHardwareSync && m_radioHardwareSync->isChecked())
        ? SyncMode::Hardware : SyncMode::Software;
    if (mode == SyncMode::Hardware) {
        QString preflight, preflightErr;
        bool executionAuthorized = false;
        if (!m_zmc->hwTableCommissioningPreflight(preflight, &preflightErr, &executionAuthorized)
            || !executionAuthorized) {
            m_hardwareSyncPreflightPassed = false;
            if (m_radioSoftwareSync) m_radioSoftwareSync->setChecked(true);
            m_syncManager->clearHardwareTriggerTypeConfiguration();
            updateVideoControlState();
            QMessageBox::warning(this, tr("硬件同步启动复核失败"),
                preflightErr.isEmpty()
                ? tr("当前控制器未发布V32.16执行授权；请下载controller/ECI3808_V32_16_H1_H3_ONLY.bas（仅H1/H3）。修复后断开重连控制器或执行B/C绝对坐标重同步，软件会自动重新预检。")
                : preflightErr);
            return;
        }
        m_hardwareSyncPreflightPassed = true;
        // V32.13现场只有OP14：scanCONTROL固定EdgeRising，ECI IN0固定REGIST(104)。
        // 硬件同步TABLE与上位机实际位置/软件同步/标定均使用XYZ/C/B五轴MPOS；DPOS保留给命令轨迹/诊断。
        // 不使用UI隐藏默认值，也不允许现场误选其它边沿。
        const int selected=m_comboHardwareTopology?m_comboHardwareTopology->currentData().toInt():1;
        if(selected==1)m_syncManager->setTriggerType(HardwareTriggerType::EdgeRising);
        else m_syncManager->clearHardwareTriggerTypeConfiguration();
        const int edge=(selected==1)?104:
            (m_comboCmmR1Edge?m_comboCmmR1Edge->currentData().toInt():114);
        m_syncManager->setHardwareTopology(static_cast<mls::runtime::HardwareSyncTopology>(selected),edge);
    }
    else {
        m_syncManager->clearHardwareTriggerTypeConfiguration();
    }
    m_syncManager->setSyncMode(mode);
    m_lastCaptureMode = mode;  // 记录采集模式

    // 设置传感器参数
    SensorParams params;
    params.resolutionIndex = static_cast<unsigned int>(
        qMax(0, m_comboResolution->currentIndex()));
    params.exposureTime = static_cast<unsigned int>(m_spinExposure->value());
    params.idleTime = static_cast<unsigned int>(m_spinIdle->value());
    params.measuringFieldIndex = m_comboMeasuringField ? m_comboMeasuringField->currentData().toInt() : 7;
    m_syncManager->setSensorParams(params);
    m_syncManager->setHardwareTriggerFrequencyHz(hardwareSyncFrequencyHz());

    // 清空之前的数据
    m_cloudBuilder->clear();
    m_profile2DView->clear();
    m_profileImageView->clear();
    m_pointCloudView->clear();

    // 启动全局时间同步
    GlobalTimeSync::instance().start();

    QString err;
    if (!m_syncManager->start(&err)) {
        LOG_ERROR(LogCategory::Sync, tr("启动采集失败: %1").arg(err));
        QMessageBox::warning(this, tr("启动失败"), err);
        updateVideoControlState();
        return;
    }

    if (mode == SyncMode::Hardware) {
        LOG_INFO(LogCategory::Sync,
            tr("H1/H3采集已Arm；H1等待单帧Probe，H3等待连续CMM窗口；名义采集频率=%1 Hz（%2 ms）")
            .arg(hardwareSyncFrequencyHz())
            .arg(1000.0 / static_cast<double>(hardwareSyncFrequencyHz()), 0, 'f', 6));
        m_lblCaptureStatus->setText(tr("硬件同步：等待Probe..."));
        statusBar()->showMessage(tr("硬件同步已Arm，等待Probe"), 3000);
    }
    else {
        LOG_INFO(LogCategory::Sync, tr("采集启动成功 - 模式: 软件同步, 曝光: %1us, 分辨率索引: %2")
            .arg(params.exposureTime).arg(params.resolutionIndex));
        m_lblCaptureStatus->setText(tr("采集中..."));
        statusBar()->showMessage(tr("采集已启动"), 3000);
    }

    // 更新 Video 控制状态（禁用 Video 启动按钮）
    updateVideoControlState();
    refreshHardwareTemporalRuntimeUi();
}

void MainWindow::onStopCapture()
{
    if(m_gcodeHardwareNcProbeActive){ finishGCodeHardwareNcProbe(true);return; }
    LOG_INFO(LogCategory::Sync, tr("停止采集"));
    if(m_profilePreviewMirrorsCapture)onStopProfilePreview();
    m_syncManager->stop();
    const auto stats = m_syncManager->getHardwareAlignStats();
    if (m_lastCaptureMode == SyncMode::Hardware && stats.alignmentFault) {
        m_lblCaptureStatus->setText(tr("已停止 / 本次采集无效"));
        statusBar()->showMessage(tr("采集无效：%1").arg(stats.faultReason));
    }
    else {
        m_lblCaptureStatus->setText(tr("已停止"));
        statusBar()->showMessage(tr("采集已停止"), 3000);
    }

    // 采集期间不轮询 AXISSTATUS；采集结束后做一次五轴状态快照。
    pollAxisAlarmStates(false);

    // 更新 Video 控制状态（重新启用 Video 启动按钮）
    updateVideoControlState();
    refreshHardwareTemporalRuntimeUi();
}

void MainWindow::onClearData()
{
    // 如果正在采集，先提示用户停止
    if (m_syncManager->isRunning()) {
        auto reply = QMessageBox::warning(this, tr("正在采集"),
            tr("当前正在采集数据，是否停止采集并清除数据？"),
            QMessageBox::Yes | QMessageBox::No, QMessageBox::No);

        if (reply != QMessageBox::Yes) {
            return;
        }

        // 先停止采集
        onStopCapture();
    }

    // 确认对话框
    auto reply = QMessageBox::question(this, tr("确认清除"),
        tr("确定要清除所有已采集的点云数据吗？\n此操作不可恢复。"),
        QMessageBox::Yes | QMessageBox::No, QMessageBox::No);

    if (reply != QMessageBox::Yes) {
        return;
    }

    // 清空所有数据
    m_cloudBuilder->clear();
    m_profile2DView->clear();
    m_profileImageView->clear();
    m_pointCloudView->clear();

    // 【关键修改】清除同步管理器的数据和状态
    m_syncManager->clearData();

    m_hasProfile = false;

    m_lblCaptureStatus->setText(tr("数据已清除"));
    statusBar()->showMessage(tr("点云数据已清除，同步状态已重置"), 3000);
}

void MainWindow::onAnalyzeData()
{
    QString report;

    if (m_lastCaptureMode == SyncMode::Hardware) {
        report = analyzeHardwareSyncData();
    }
    else {
        report = analyzeSoftwareSyncData();
    }

    // 显示分析结果对话框
    QDialog dialog(this);
    dialog.setWindowTitle(tr("数据分析报告"));
    dialog.resize(700, 500);

    auto* layout = new QVBoxLayout(&dialog);

    auto* textEdit = new QTextEdit(&dialog);
    textEdit->setReadOnly(true);
    textEdit->setFont(QFont("Consolas", 10));
    textEdit->setText(report);
    layout->addWidget(textEdit);

    auto* btnClose = new QPushButton(tr("关闭"), &dialog);
    connect(btnClose, &QPushButton::clicked, &dialog, &QDialog::accept);
    layout->addWidget(btnClose);

    dialog.exec();
}

QString MainWindow::analyzeHardwareSyncData()
{
    const auto stats = m_syncManager->getHardwareAlignStats();
    syncdiag::HardwareReportInput input;
    input.running = m_syncManager->isRunning();
    input.fault = stats.alignmentFault;
    input.faultReason = stats.faultReason.toStdString();
    input.synchronized = stats.syncEstablished;
    input.triggerCountValid = stats.zmcTriggerCountValid;
    input.poses = stats.totalPoseSamples; input.received = stats.totalProfiles;
    input.triggers = stats.zmcTriggerCount; input.aligned = stats.alignedProfiles;
    input.pending = stats.pendingProfiles; input.dropped = stats.droppedProfiles;
    input.validAligned = stats.validAlignedProfiles; input.emptyAligned = stats.emptyAlignedProfiles;
    input.temporalBoundarySkipped = stats.temporalBoundarySkippedProfiles;
    const auto temporal = m_syncManager->lastCaptureHardwareTemporalCorrection();
    input.temporalCorrectionEnabled = temporal.enabled;
    input.temporalDelayUs = temporal.delaySec * 1e6;
    input.temporalDelayStdUs = temporal.delayStdSec * 1e6;
    input.temporalCalibrationId = temporal.calibrationId.toStdString();
    input.missing = stats.missingProfileIndices; input.duplicates = stats.duplicateProfiles;
    input.backward = stats.outOfOrderProfiles; input.invalidIndices = stats.invalidProfileIndices;
    input.readErrors = stats.tableReadErrors; input.overflows = stats.tableOverflows;
    input.cloudPoints = static_cast<std::uint64_t>(m_cloudBuilder->pointCount());
    const auto params = m_syncManager->lastCaptureSensorParams();
    input.exposureMs = params.exposureTime * 0.01; input.idleMs = params.idleTime * 0.01;
    input.periodMs = m_syncManager->lastCaptureTriggerPeriodMs();
    const auto poses = m_syncManager->getHardwarePoseSamples();
    input.poseEstimateUs.reserve(static_cast<std::size_t>(poses.size()));
    for (const auto& pose : poses) input.poseEstimateUs.push_back(pose.tUs);
    input.frames = m_syncManager->getHardwareProfileRecords();
    const auto cbDiag = m_laser->profileCallbackDiagnostics();
    const auto hwDiag = m_laser->hardwareTriggerDiagnostics();
    input.sensorRawCallbacks = cbDiag.rawCallbacks;
    input.sensorConvertedProfiles = cbDiag.convertedProfiles;
    input.sensorConvertFailures = cbDiag.conversionFailures;
    input.sensorIngressDrops = cbDiag.ingressDrops;
    input.sensorRawRingDepth = cbDiag.rawRingDepth;
    input.sensorRawRingHighWater = cbDiag.rawRingHighWater;
    input.sensorRawRingCapacity = cbDiag.rawRingCapacity;
    input.sensorLastCallbackOrdinal = cbDiag.lastCallbackOrdinal;
    input.sensorMaxCallbackServiceUs = cbDiag.maxCallbackServiceUs;
    input.sensorMaxDecoderQueueUs = cbDiag.maxDecoderQueueUs;
    input.sensorMaxDecodeServiceUs = cbDiag.maxDecodeServiceUs;
    input.sensorLastConvertResult = cbDiag.lastConvertResult;
    input.sensorLastCallbackBytes = cbDiag.lastCallbackBytes;
    input.sensorLastResolution = cbDiag.lastResolution;
    input.sensorTriggerReadbackValid = hwDiag.readbackValid;
    input.topologyId = static_cast<int>(m_syncManager->lastCaptureTopology());
    input.cmmR1Edge = m_syncManager->lastCaptureCmmR1Edge();
    input.sensorTriggerConfigMatches = mls::sensor::trigger::sensorTriggerMatches(
        input.topologyId, hwDiag.triggerRequested, hwDiag.triggerReadback);
    // H1 requires external TRIGGER and DIGITAL_IO; H3 uses an independent CMM readback.
    if (input.topologyId == 1)
        input.sensorTriggerConfigMatches = input.sensorTriggerConfigMatches &&
            mls::sensor::trigger::digitalIoMatches(1, hwDiag.digitalIoRequested, hwDiag.digitalIoReadback);
    input.sensorCmmPortReadbackValid = hwDiag.cmmPortReadbackValid &&
        input.topologyId == 3 && mls::sensor::trigger::digitalIoMatches(
            3, hwDiag.digitalIoRequested, hwDiag.digitalIoReadback);
    input.sensorTriggerRequested = hwDiag.triggerRequested;
    input.sensorTriggerReadback = hwDiag.triggerReadback;
    input.sensorDigitalIoRequested = hwDiag.digitalIoRequested;
    input.sensorDigitalIoReadback = hwDiag.digitalIoReadback;
    input.io = m_zmc->ioStatistics();
    // O(frame count), not O(point count). Empty profiles retain their timestamps.
    const auto report = syncdiag::hardwareSyncReport(input);
    return QString::fromUtf8(report.c_str());
}

QString MainWindow::analyzeSoftwareSyncData()
{
    QString report;
    QTextStream ts(&report);

    ts << "═══════════════════════════════════════════════════════════════\n";
    ts << "              软件同步采集数据分析报告\n";
    ts << "═══════════════════════════════════════════════════════════════\n\n";

    // 获取对齐数据
    const auto& alignedPoints = m_cloudBuilder->getAlignmentLog();

    // 基本统计
    int profileCount = 0, poseCount = 0;
    m_syncManager->getStats(profileCount, poseCount);

    ts << "【基本统计】\n";
    ts << QString("  Profile 数量:    %1\n").arg(profileCount);
    ts << QString("  位置缓存数量:    %1 (环形缓冲区)\n").arg(poseCount);
    ts << QString("  点云总点数:      %1\n").arg(m_cloudBuilder->pointCount());
    ts << QString("  对齐记录数:      %1\n").arg(alignedPoints.size());
    ts << "\n";

    // 分析 Profile 时间间隔
    ts << "【Profile 时间间隔分析】\n";
    if (alignedPoints.size() >= 2) {
        // 按 profileIndex 分组，获取每个 Profile 的时间戳
        QMap<quint32, qint64> profileTimes;
        for (const auto& ap : alignedPoints) {
            if (!profileTimes.contains(ap.profileIndex)) {
                profileTimes[ap.profileIndex] = ap.tUs;
            }
        }

        QVector<qint64> sortedTimes;
        for (auto it = profileTimes.begin(); it != profileTimes.end(); ++it) {
            sortedTimes.append(it.value());
        }
        std::sort(sortedTimes.begin(), sortedTimes.end());

        if (sortedTimes.size() >= 2) {
            QVector<double> intervals;
            for (int i = 1; i < sortedTimes.size(); ++i) {
                double dt = (sortedTimes[i] - sortedTimes[i - 1]) / 1000.0;  // ms
                intervals.append(dt);
            }

            // 统计
            double sum = 0, minVal = 1e9, maxVal = -1e9;
            for (double v : intervals) {
                sum += v;
                if (v < minVal) minVal = v;
                if (v > maxVal) maxVal = v;
            }
            double mean = sum / intervals.size();

            // 标准差
            double variance = 0;
            for (double v : intervals) {
                variance += (v - mean) * (v - mean);
            }
            double stddev = std::sqrt(variance / intervals.size());

            ts << QString("  Profile 数:  %1\n").arg(sortedTimes.size());
            ts << QString("  间隔数:      %1\n").arg(intervals.size());
            ts << QString("  平均间隔:    %1 ms\n").arg(mean, 0, 'f', 3);
            ts << QString("  最小间隔:    %1 ms\n").arg(minVal, 0, 'f', 3);
            ts << QString("  最大间隔:    %1 ms\n").arg(maxVal, 0, 'f', 3);
            ts << QString("  标准差:      %1 ms\n").arg(stddev, 0, 'f', 3);
            ts << QString("  间隔范围:    %1 ms\n").arg(maxVal - minVal, 0, 'f', 3);

            // 软件同步的 Profile 间隔取决于传感器内部触发
            ts << "\n  注：软件同步模式下，Profile 间隔由传感器内部触发决定\n";
            ts << "      位置通过 PC 时间戳插值获得，非硬件同步\n";
        }
    }
    else {
        ts << "  数据不足，无法分析间隔\n";
    }

    // 与硬件同步的对比说明
    ts << "\n【软件同步 vs 硬件同步】\n";
    ts << "  软件同步特点:\n";
    ts << "    - Profile 由传感器内部定时器触发\n";
    ts << "    - 位置通过 PC 时间戳插值计算\n";
    ts << "    - 存在 PC 时间与传感器时间的偏差\n";
    ts << "    - 适合对精度要求不高的场景\n";
    ts << "\n";
    ts << "  硬件同步特点(V32.13 SINGLE-OP14 + FULL-MPOS-LATCH + COLD-PRIME；控制器/B-C坐标就绪后自动只读预检+TABLE[342]=1授权，正式启动前再次复核):\n";
    ts << "    - V32.13 BASIC严格按最终光栅尺方案：XYZ/C/B五轴均由REGIST(104)硬件锁存raw MPOS\n";
    ts << "    - 正式session前先用enable=3/OP14-only进行scanCONTROL冷启动预热；预热Profile全部丢弃，新session随后清空并重新Arm五轴REGIST FIFO\n";
    ts << "    - B/C为增量编码器：每次采集冻结RS485绝对坐标建立的软件offset，并按 machine=rawMPOS+offset 转成上位机机械绝对角\n";
    ts << "    - 上位机按Trigger/Pose/Profile/Aligned/Pending闭环计数，停止尾部未完全收敛时直接判本次数据无效\n";
    ts << "    - 硬件同步协议固定feedbackMask=31/latchMask=31/commandMask=0/timing=HardwareLatched；XYZ/C/B均记录实际MPOS\n    - 软件同步/轴状态/标定同样使用XYZ/C/B实际MPOS；XYZ DPOS仅作为控制器命令坐标与跟随误差诊断\n";
    ts << "    - 主机时间估计不参与硬件锁存姿态合成\n";
    ts << "    - 软件审计不能替代外部电气边沿、控制器锁存偏差、传感器曝光相位和动态重复性的独立硬件验证\n";

    ts << "\n═══════════════════════════════════════════════════════════════\n";

    return report;
}

// ==================== 定时器 ====================

void MainWindow::onStatusTimer()
{
    if (m_closeRequested || m_controllerConnectInProgress || m_rotaryInitInProgress) return;
    if (!m_zmc->isConnected()) return;
    // V27.6.50: a hardware-temporal scan path is intentionally staged before
    // SyncManager is armed.  Do not let the ordinary refill timer launch that
    // path until the V32.13 Probe has established the hardware-sync session.
    const bool allowMergePump = !m_hardwareTemporalScanActive || m_hardwareTemporalMotionStarted;
    if (allowMergePump && !m_mergePumpPending) {
        m_mergePumpPending = true; ++m_backgroundIoCount;
        ZmcController* controller = m_zmc.get();
        const auto generation = controller->ioStatistics().generation.session;
        (void)QtConcurrent::run([this, controller, generation] {
            bool finished = false; QString err;
            bool ok = false;
            try { ok = controller->pumpMergeBuffer(finished, &err); }
            catch (const std::exception& e) { err = QString::fromUtf8(e.what()); }
            catch (...) { err = QStringLiteral("补缓冲后台任务发生未知异常"); }
            QMetaObject::invokeMethod(this, [this, ok, err, generation] {
                m_mergePumpPending = false; --m_backgroundIoCount;
                if (!m_closeRequested && !ok && m_zmc->isConnected()
                    && generation == m_zmc->ioStatistics().generation.session) {
                    m_zmc->requestStopAxis(AxisX, true); // preserve legacy stop mode
                    statusBar()->showMessage(tr("连续插补出错/取消: ") + err, 5000);
                }
                tryFinishDeferredClose();
                }, Qt::QueuedConnection);
            });
    }
    static int syncCounter = 0;
    if (++syncCounter >= 10) { syncCounter = 0; updateVideoControlState(); }
}

void MainWindow::onPosTimer()
{
    if (m_closeRequested || m_controllerConnectInProgress || m_rotaryInitInProgress || m_positionPollPending) return;
    if (!m_zmc->isConnected()) return;

    // During hardware synchronization do NOT add GetMpos/GetDpos traffic to the ECI I/O
    // scheduler. The TABLE reader already owns the authoritative latch-time XYZ/C/B MPOS
    // samples, so reuse that same five-axis measured pose for the UI.
    if (m_syncManager && m_syncManager->isRunning()) {
        if (m_syncManager->syncMode() == SyncMode::Hardware) {
            SamplePoint latest;
            if (m_syncManager->latestHardwarePoseSample(latest)) {
                if (m_xAxisWidget) m_xAxisWidget->updateHardwareLatchedPosition(static_cast<float>(latest.x));
                if (m_yAxisWidget) m_yAxisWidget->updateHardwareLatchedPosition(static_cast<float>(latest.y));
                if (m_zAxisWidget) m_zAxisWidget->updateHardwareLatchedPosition(static_cast<float>(latest.z));
                if (m_cAxisWidget) m_cAxisWidget->updateHardwareLatchedPosition(static_cast<float>(latest.c));
                if (m_bAxisWidget) m_bAxisWidget->updateHardwareLatchedPosition(static_cast<float>(latest.b));
                // Hardware latch data are already five-axis MPOS in the same MCS
                // as the Machine5X zero-pose meshes.  Drive the digital twin from
                // this authoritative latch-time pose and do not inject extra ECI I/O.
                updateMachineDigitalTwinPose(
                    JointPose5D{ latest.x, latest.y, latest.z, latest.c, latest.b }, true);
            }
        }
        return;
    }

    struct PositionResult { bool ok = false; float dpos = 0, mpos = 0; bool idle = false; };
    m_positionPollPending = true; ++m_backgroundIoCount;
    ZmcController* controller = m_zmc.get();
    const auto generation = controller->ioStatistics().generation.session;
    (void)QtConcurrent::run([this, controller, generation] {
        std::array<PositionResult, 6> results{};
        try {
            for (int axis = AxisX; axis <= AxisB; ++axis) {
                if ((axis == AxisC || axis == AxisB) && !controller->isRotaryAbsoluteReady(axis)) continue;
                auto& r = results[axis];
                r.ok = controller->readAxisUiPosition(axis, r.dpos, r.mpos, r.idle, nullptr);
            }
        }
        catch (...) {
            for (auto& r : results) r.ok = false; // do not display a partial/failed snapshot
        }
        QMetaObject::invokeMethod(this, [this, results, generation] {
            m_positionPollPending = false; --m_backgroundIoCount;
            if (!m_closeRequested && m_zmc->isConnected()
                && generation == m_zmc->ioStatistics().generation.session
                && !(m_syncManager && m_syncManager->isRunning())) {
                for (int axis = AxisX; axis <= AxisB; ++axis) {
                    const auto& r = results[axis];
                    auto* widget = axisWidgetForId(axis);
                    if (!r.ok || !widget) continue;
                    // Show both command (DPOS) and measured (MPOS) coordinates on every axis;
                    // MPOS is the actual-position value used by XYZ/C/B measurement paths.
                    widget->updatePositionPair(r.dpos, r.mpos, r.idle);
                }
                // The imported Machine5X PLY/STL zero-pose is defined at XYZBC=0.
                // Only drive it after the incremental XYZ gratings and B/C absolute
                // frames are referenced; before homing, raw MPOS is not a machine
                // coordinate and must not move the digital twin.
                const bool allMposOk = results[AxisX].ok && results[AxisY].ok && results[AxisZ].ok
                    && results[AxisC].ok && results[AxisB].ok
                    && m_zmc->areLinearMposReferenced()
                    && m_zmc->isRotaryAbsoluteReady(AxisC)
                    && m_zmc->isRotaryAbsoluteReady(AxisB);
                if (allMposOk) {
                    updateMachineDigitalTwinPose(
                        JointPose5D{ results[AxisX].mpos, results[AxisY].mpos, results[AxisZ].mpos,
                                    results[AxisC].mpos, results[AxisB].mpos }, true);
                }
            }
            tryFinishDeferredClose();
            }, Qt::QueuedConnection);
        });
}

// ==================== 数据导出 ====================

void MainWindow::onExportAlignmentData()
{
    if (m_lastCaptureMode == SyncMode::Hardware) {
        const auto stats = m_syncManager->getHardwareAlignStats();
        if (stats.alignmentFault && QMessageBox::warning(this, tr("无效扫描：仅供诊断"),
            tr("本次扫描已锁存完整性故障：%1\n现有点云不能作为有效测量结果。\n"
                "现有文件格式不保存此故障标记，请同时保存分析报告/日志。是否仅为诊断而导出？")
            .arg(stats.faultReason), QMessageBox::Yes | QMessageBox::No, QMessageBox::No)
            != QMessageBox::Yes) return;
    }

    auto data = m_cloudBuilder->getAlignmentLog();
    if (data.isEmpty()) {
        QMessageBox::information(this, tr("提示"), tr("没有可导出的对齐数据"));
        return;
    }

    QString fileName = QFileDialog::getSaveFileName(
        this, tr("导出对齐数据"), QString(),
        tr("CSV 文件 (*.csv);;所有文件 (*.*)"));
    if (fileName.isEmpty()) return;

    QString err;
    if (!CsvExporter::exportAlignmentData(fileName, data, &err)) {
        QMessageBox::warning(this, tr("导出失败"), err);
        return;
    }

    QMessageBox::information(this, tr("完成"),
        tr("已导出 %1 条对齐数据").arg(data.size()));
}

void MainWindow::onExportPointCloud()
{
    if (m_syncManager && m_syncManager->isRunning()) {
        QMessageBox::information(this, tr("采集尚未收尾"),
            tr("请等待当前采集停止并完成计数/队列收敛后再导出。运行中的五轴数据不能作为完整扫描归档。"));
        return;
    }
    if (m_lastCaptureMode == SyncMode::Hardware) {
        const auto stats = m_syncManager->getHardwareAlignStats();
        if (stats.alignmentFault && QMessageBox::warning(this, tr("无效扫描：仅供诊断"),
            tr("本次扫描已锁存完整性故障：%1\n现有点云不能作为有效测量结果。\n"
                "现有文件格式不保存此故障标记，请同时保存分析报告/日志。是否仅为诊断而导出？")
            .arg(stats.faultReason), QMessageBox::Yes | QMessageBox::No, QMessageBox::No)
            != QMessageBox::Yes) return;
    }

    auto points = m_cloudBuilder->getPoints();
    auto alignedPoints = m_cloudBuilder->getAlignmentLog();

    if (points.isEmpty() && alignedPoints.isEmpty()) {
        QMessageBox::information(this, tr("提示"), tr("没有可导出的点云数据"));
        return;
    }

    // 显示格式选择对话框
    SaveFormatDialog formatDialog(this);
    formatDialog.setPointCount(points.isEmpty() ? alignedPoints.size() : points.size());

    if (formatDialog.exec() != QDialog::Accepted) {
        return;
    }

    PointCloudSaveFormat format = formatDialog.selectedFormat();
    const int compressionLevel = formatDialog.compressionLevel();
    const uint32_t scanIntervalUs = static_cast<uint32_t>(std::max(1.0, m_syncManager->lastCaptureTriggerPeriodMs()) * 1000.0);
    QString extension = getFormatExtension(format);

    // 构建文件过滤器
    QString filter;
    switch (format) {
    case PointCloudSaveFormat::CSV:
        filter = tr("CSV 文件 (*.csv)");
        break;
    case PointCloudSaveFormat::PLY_ASCII:
    case PointCloudSaveFormat::PLY_BINARY:
        filter = tr("PLY 文件 (*.ply)");
        break;
    case PointCloudSaveFormat::SCAN_UNCOMPRESSED:
    case PointCloudSaveFormat::SCAN_LZ4:
    case PointCloudSaveFormat::SCAN_ZSTD:
        filter = tr("紧凑扫描文件 (*.scan)");
        break;
    }
    filter += ";;" + tr("所有文件 (*.*)");

    QString fileName = QFileDialog::getSaveFileName(
        this, tr("导出点云"), QString(), filter);
    if (fileName.isEmpty()) return;

    // 确保扩展名正确
    if (!fileName.endsWith(extension, Qt::CaseInsensitive)) {
        fileName += extension;
    }

    // 根据格式执行保存
    QString taskId;
    auto& saveManager = AsyncSaveManager::instance();

    switch (format) {
    case PointCloudSaveFormat::CSV: {
        QString err;
        if (!CsvExporter::exportPointCloudCsv(fileName, points, &err)) {
            QMessageBox::warning(this, tr("导出失败"), err);
            return;
        }
        QMessageBox::information(this, tr("完成"),
            tr("已导出 %1 个点到 CSV 文件").arg(points.size()));
        return;
    }

    case PointCloudSaveFormat::PLY_ASCII: {
        QString err;
        if (!CsvExporter::exportPointCloudPly(fileName, points, &err)) {
            QMessageBox::warning(this, tr("导出失败"), err);
            return;
        }
        QMessageBox::information(this, tr("完成"),
            tr("已导出 %1 个点到 PLY 文件").arg(points.size()));
        return;
    }

    case PointCloudSaveFormat::PLY_BINARY:
        taskId = saveManager.submitSavePointCloudPLYBinary(fileName, points);
        break;

    case PointCloudSaveFormat::SCAN_UNCOMPRESSED:
        taskId = saveManager.submitSaveCompactScan(
            fileName, alignedPoints, CompressionType::None, 1, 0, scanIntervalUs);
        break;

    case PointCloudSaveFormat::SCAN_LZ4:
        taskId = saveManager.submitSaveCompactScan(
            fileName, alignedPoints, CompressionType::LZ4, compressionLevel, 0, scanIntervalUs);
        break;

    case PointCloudSaveFormat::SCAN_ZSTD:
        taskId = saveManager.submitSaveCompactScan(
            fileName, alignedPoints, CompressionType::ZSTD, compressionLevel, 0, scanIntervalUs);
        break;
    }

    if (!taskId.isEmpty()) {
        // 异步保存，显示进度提示
        statusBar()->showMessage(tr("正在保存点云数据..."), 5000);

        // 连接完成信号（一次性）
        QMetaObject::Connection* conn = new QMetaObject::Connection();
        *conn = connect(&saveManager, &AsyncSaveManager::taskCompleted,
            this, [this, conn, fileName](const QString& id, const QString& path, qint64 bytes) {
                Q_UNUSED(id)
                    if (path == fileName) {
                        QString sizeStr;
                        if (bytes >= 1024 * 1024) {
                            sizeStr = QString("%1 MB").arg(bytes / (1024.0 * 1024.0), 0, 'f', 2);
                        }
                        else if (bytes >= 1024) {
                            sizeStr = QString("%1 KB").arg(bytes / 1024.0, 0, 'f', 2);
                        }
                        else {
                            sizeStr = QString("%1 bytes").arg(bytes);
                        }
                        QMessageBox::information(this, tr("完成"),
                            tr("点云已保存\n文件: %1\n大小: %2").arg(path).arg(sizeStr));
                        disconnect(*conn);
                        delete conn;
                    }
            });
    }
}

void MainWindow::onImportPointCloud()
{
    // 构建文件过滤器
    QString filter = tr("所有支持的格式 (*.scan *.ply *.csv);;")
        + tr("紧凑扫描文件 (*.scan);;")
        + tr("PLY 文件 (*.ply);;")
        + tr("CSV 文件 (*.csv);;")
        + tr("所有文件 (*.*)");

    QString fileName = QFileDialog::getOpenFileName(
        this, tr("导入点云"), QString(), filter);
    if (fileName.isEmpty()) return;

    // 检测文件格式
    PointCloudFileFormat format = PointCloudLoader::detectFormat(fileName);
    if (format == PointCloudFileFormat::Unknown) {
        QMessageBox::warning(this, tr("导入失败"), tr("无法识别的文件格式"));
        return;
    }

    // 显示加载进度
    statusBar()->showMessage(tr("正在加载点云文件..."));
    QApplication::processEvents();

    QString errorMsg;
    QVector<QVector3D> points;
    PointCloudWidget::ImportedCloudLayout organized;

    // 根据格式加载
    if (format == PointCloudFileFormat::SCAN) {
        // 对于.scan格式，可以加载完整的扫描数据
        ScanData scanData;
        if (!PointCloudLoader::loadScan(fileName, scanData, &errorMsg)) {
            statusBar()->clearMessage();
            QMessageBox::warning(this, tr("导入失败"), errorMsg);
            return;
        }

        // Reconstruct from original per-profile ordinals. The old flattening
        // discarded invalid samples and silently shifted the next column,
        // which would connect unrelated vertices when creating a strip.
        QVector<qsizetype> offsets;
        QVector<quint32> originalColumns;
        QString topologyError;
        if (PointCloudLoader::reconstructOrganizedScan(scanData, points,
                offsets, originalColumns, &topologyError)) {
            organized.rowOffsets = std::move(offsets);
            organized.originalColumns = std::move(originalColumns);
            organized.pointsPerProfile = int(scanData.header.pointsPerProfile);
            organized.hasOriginalIndices = true;
        } else {
            // Preserve the original point-only import behavior for scans with
            // inconsistent indexing/invalid transforms (never invent topology).
            points = PointCloudReconstructor::reconstruct(scanData);
            organized.forcePointOnly = true;
            LOG_WARNING(LogCategory::File,
                tr("SCAN轮廓拓扑无法恢复，仅允许散点显示：%1").arg(topologyError));
        }

        // 显示详细信息
        QFileInfo fi(fileName);
        QString info = tr("已加载 .scan 文件:\n"
            "文件: %1\n"
            "大小: %2 KB\n"
            "轮廓数: %3\n"
            "每轮廓点数: %4\n"
            "重建点数: %5\n"
            "压缩: %6")
            .arg(fi.fileName())
            .arg(fi.size() / 1024.0, 0, 'f', 2)
            .arg(scanData.header.profileCount)
            .arg(scanData.header.pointsPerProfile)
            .arg(points.size())
            .arg(scanData.header.compression == CompressionType::None ? tr("无") :
                (scanData.header.version < 3 ? "Qt zlib (legacy mislabeled codec)" : "Qt zlib"));

        statusBar()->clearMessage();
        QMessageBox::information(this, tr("导入成功"), info);
    }
    else {
        // 对于其他格式，直接加载点云
        if (!PointCloudLoader::load(fileName, points, &errorMsg)) {
            statusBar()->clearMessage();
            QMessageBox::warning(this, tr("导入失败"), errorMsg);
            return;
        }

        QFileInfo fi(fileName);
        QString info = tr("已加载点云文件:\n"
            "文件: %1\n"
            "大小: %2 KB\n"
            "点数: %3")
            .arg(fi.fileName())
            .arg(fi.size() / 1024.0, 0, 'f', 2)
            .arg(points.size());

        statusBar()->clearMessage();
        QMessageBox::information(this, tr("导入成功"), info);
    }

    // 更新点云视图
    if (!points.isEmpty()) {
        // 清空现有点云
        m_cloudBuilder->clear();

        // 添加导入的点
        m_pointCloudView->setImportedPoints(points, organized);

        // Explicitly select the new display options inside the 3D tab.
        // Live capture continues to use PointCloudBuilder's old point actor.
        if (m_viewTabs) m_viewTabs->setCurrentWidget(m_pointCloudView);
        statusBar()->showMessage(tr("已导入 %1 个点；可在三维窗口切换散点/三角网格").arg(points.size()), 5000);
    }
}

// ==================== 轮廓和点云更新 ====================

void MainWindow::onProfileUiTimer()
{
    if (!m_laser) return;
    if (m_profilePreviewMirrorsCapture && (!m_syncManager || !m_syncManager->isRunning())) {
        // Detach when the owner stops; never let an old hardware frame appear as live.
        onStopProfilePreview();
        return;
    }
    LaserProfile profile;
    const bool have=m_laser->latestProfile(profile);
    if(m_profilePreviewMirrorsCapture){
        // The decoder may retain the last profile even after triggers stop.
        // Only NEW callback ordinals are considered a live hardware/software feed.
        const bool fresh=have && profile.callbackOrdinal!=0 &&
            profile.callbackOrdinal!=m_profilePreviewMirrorBaselineOrdinal &&
            profile.callbackOrdinal!=m_lastDisplayedProfileOrdinal;
        if(!fresh){
            if(QDateTime::currentMSecsSinceEpoch()-m_profilePreviewMirrorSinceMs>2500 &&
               m_lblProfilePreviewStatus && m_syncManager &&
               m_syncManager->syncMode()==SyncMode::Hardware){
                const QString warning=tr("硬件采集未收到新的轮廓（或已中断）：检查H1外部OP14/Probe或H3内部时钟与CMM传输；不会自动改触发设置");
                if(m_lblProfilePreviewStatus->text()!=warning)
                    m_lblProfilePreviewStatus->setText(warning);
            }
            return;
        }
        m_profilePreviewMirrorSinceMs=QDateTime::currentMSecsSinceEpoch();
        if(!m_profilePreviewMirrorFirstFrameSeen){
            m_profilePreviewMirrorFirstFrameSeen=true;
            if(m_lblProfilePreviewStatus)m_lblProfilePreviewStatus->setText(
                tr("正在显示正式同步采集的实时轮廓（只读）"));
        } else if(m_syncManager && m_syncManager->syncMode()==SyncMode::Hardware &&
                  m_lblProfilePreviewStatus &&
                  m_lblProfilePreviewStatus->text().contains(tr("未收到新的轮廓"))){
            m_lblProfilePreviewStatus->setText(tr("硬件轮廓已恢复，实时只读显示"));
        }
    }
    if(!have)return;
    if (profile.callbackOrdinal != 0 && profile.callbackOrdinal == m_lastDisplayedProfileOrdinal) return;
    m_lastDisplayedProfileOrdinal = profile.callbackOrdinal;
    onProfileReceived(profile);
}

void MainWindow::onProfileReceived(const LaserProfile& profile)
{
    // V27.6.16M: called by the ~30 Hz latest-only UI timer, not by every SDK profile.
    // Formal acquisition/forensics remain lossless and independent of display throttling.
    if (!profile.x.isEmpty() && !profile.z.isEmpty()) {
        m_profile2DView->setProfile(profile.x, profile.z);
        m_profileImageView->pushProfileLine(profile.z);

        // 如果轮廓预览正在运行，也更新预览控件
        if (m_profilePreviewRunning && m_profilePreviewView) {
            m_profilePreviewView->setProfile(profile.x, profile.z);
        }
    }

    // 保存最后的 profile
    m_lastProfile = profile;
    m_hasProfile = true;
}

void MainWindow::onAlignedDataReady(const QVector<AlignedPoint>& points)
{
    // 此函数现在只用于统计目的
    // 2D 显示已由 onProfileReceived 处理
    // 3D 显示由 PointCloudBuilder + PointCloudWidget 自动处理
    Q_UNUSED(points)
}

void MainWindow::updateLastProfile(const LaserProfile& profile)
{
    m_lastProfile = profile;
    m_hasProfile = true;
}

// ==================== Video 模式控制 ====================

void MainWindow::onStartVideo()
{
    // 1. 基本检查：传感器是否连接
    if (!m_laser->isConnected()) {
        QMessageBox::warning(this, tr("提示"), tr("请先连接传感器"));
        updateVideoControlState();
        return;
    }

    // 2. 检查 Profile 采集是否在运行
    if (m_syncManager->isRunning()) {
        QMessageBox::warning(this, tr("模式冲突"),
            tr("Profile 采集正在运行。\n请先停止采集，再启动 Video 预览。"));
        updateVideoControlState();
        return;
    }

    // 3. 检查轮廓预览是否在运行
    if (m_profilePreviewRunning) {
        QMessageBox::warning(this, tr("模式冲突"),
            tr("轮廓预览正在运行。\n请先停止轮廓预览，再启动 Video 预览。"));
        updateVideoControlState();
        return;
    }

    // 4. Never take ownership of a Profile stream started by another task.
    // A SyncManager session may be unwinding even when isRunning() is false.
    if (m_laser->isProfileRunning()) {
        QMessageBox::warning(this, tr("轮廓传输被占用"),
            tr("传感器正在进行其他Profile传输。请先在原采集任务中安全停止，Video预览不会擅自停流。"));
        updateVideoControlState();
        return;
    }

    // 5. 检查 Video 是否已经在运行
    if (m_laser->isVideoRunning()) {
        QMessageBox::information(this, tr("提示"), tr("Video 预览已在运行"));
        updateVideoControlState();
        return;
    }

    QString err;
    if (!enterSensorCalibrationMeasuringField(&err)) {
        QMessageBox::warning(this, tr("测量场设置失败"), err);
        updateVideoControlState();
        return;
    }

    // Video has an independent fixed 25Hz clock; Profile timing is irrelevant.
    updateSensorCalibrationTimingUi();
    SensorParams params;
    params.measuringFieldIndex = m_sensorCalibActiveFieldIndex;
    params.exposureTime = static_cast<unsigned int>(m_spinVideoExposure->value());
    params.idleTime = static_cast<unsigned int>(
        sensorCalibrationVideoIdleUnitsForExposure(static_cast<int>(params.exposureTime)));

    // 7. 配置 Video 模式
    QString timingErr;
    if (!validateSensorCalibrationVideoTiming(static_cast<int>(params.exposureTime), &timingErr)) {
        leaveSensorCalibrationMeasuringField();
        QMessageBox::warning(this, tr("时序约束失败"), timingErr);
        updateVideoControlState();
        return;
    }
    if (!m_laser->configureVideo(params, &err)) {
        leaveSensorCalibrationMeasuringField();
        QMessageBox::warning(this, tr("配置失败"), err);
        updateVideoControlState();
        return;
    }
    if (!verifySensorCalibrationRuntimeConstraints(params, &timingErr)) {
        leaveSensorCalibrationMeasuringField();
        QMessageBox::warning(this, tr("传感器约束校验失败"), timingErr);
        updateVideoControlState();
        return;
    }

    // 8. 启动 Video 流
    if (!m_laser->startVideoStream(&err)) {
        leaveSensorCalibrationMeasuringField();
        QMessageBox::warning(this, tr("启动失败"), err);
        updateVideoControlState();
        return;
    }

    // 9. 成功启动 - 切换到“传感预览”并显示Video
    if (m_viewTabs && m_previewStack) {
        m_viewTabs->setCurrentIndex(m_viewTabs->count() - 1); // “传感预览”是最后一个Tab
        m_previewStack->setCurrentIndex(0);  // Video页面
        if (m_radioVideoMode) m_radioVideoMode->setChecked(true);
    }

    m_lblVideoStatus->setText(tr("预览中（Video 25Hz）"));
    m_lblVideoStatus->setStyleSheet("QLabel { color: blue; font-weight: bold; }");

    updateVideoControlState();

    statusBar()->showMessage(tr("Video 25Hz预览已启动 (%1×%2)")
        .arg(m_laser->videoWidth())
        .arg(m_laser->videoHeight()));
}

void MainWindow::onStopVideo()
{
    // 检查 Video 是否真的在运行
    if (!m_laser->isVideoRunning()) {
        // Video 没有运行，只更新 UI 状态
        m_lblVideoStatus->setText(tr("未启动"));
        m_lblVideoStatus->setStyleSheet("QLabel { font-weight: bold; }");
        updateVideoControlState();
        return;
    }

    // 停止 Video 流并恢复进入传感标定前的测量区域。
    m_laser->stopVideoStream();
    if (m_expTestState == ExpTestState::Idle) leaveSensorCalibrationMeasuringField();

    // 更新 UI
    m_lblVideoStatus->setText(tr("已停止"));
    m_lblVideoStatus->setStyleSheet("QLabel { font-weight: bold; }");

    // 清空 Video 显示
    if (m_videoWidget) {
        m_videoWidget->clear();
    }

    // 更新所有按钮状态
    updateVideoControlState();

    statusBar()->showMessage(tr("Video 预览已停止"));
}

void MainWindow::onVideoFrameReceived(const QImage& frame)
{
    ++m_videoFrameOrdinal;
    if (m_videoWidget) {
        m_videoWidget->setFrame(frame);
    }
}

void MainWindow::updateVideoControlState()
{
    bool sensorConnected = m_laser->isConnected();
    bool videoRunning = m_laser->isVideoRunning();
    bool profileRunning = m_syncManager->isRunning();
    bool previewRunning = m_profilePreviewRunning;
    bool expTestRunning = (m_expTestState != ExpTestState::Idle);
    bool anyRunning = videoRunning || profileRunning || previewRunning || expTestRunning || m_sphereExpActive || m_small7Active;

    // ========== 预览模式选择 ==========
    if (m_radioVideoMode) m_radioVideoMode->setEnabled(!anyRunning);
    if (m_radioProfilePreviewMode) m_radioProfilePreviewMode->setEnabled(!anyRunning);

    // ========== Video 控制区域 ==========
    const bool profileTimingValid = m_spinVideoExposure
        && validateSensorCalibrationTiming(m_spinVideoExposure->value(),
                                          sensorCalibrationRequestedRateHz(), nullptr);
    const bool videoTimingValid = m_spinVideoExposure
        && validateSensorCalibrationVideoTiming(m_spinVideoExposure->value(), nullptr);
    m_btnStartVideo->setEnabled(sensorConnected && !anyRunning && videoTimingValid);
    m_btnStopVideo->setEnabled(videoRunning && !expTestRunning);

    // ========== 轮廓预览控制区域 ==========
    // When the formal hardware/software stream is running, allow non-invasive
    // read-only display attachment rather than disabling "启动轮廓显示".
    const bool canMirror=sensorConnected && profileRunning && !previewRunning &&
         !videoRunning && !expTestRunning && !m_sphereExpActive && !m_small7Active;
    if (m_btnStartProfilePreview)
        m_btnStartProfilePreview->setEnabled((sensorConnected && !anyRunning && profileTimingValid) || canMirror);
    if (m_btnStopProfilePreview)
        m_btnStopProfilePreview->setEnabled(previewRunning && !expTestRunning &&
            (!m_sphereStudyCaptureActive || m_profilePreviewMirrorsCapture));

    // 参数输入
    const bool rateFollowsField = m_checkSensorCalibAutoFieldRate && m_checkSensorCalibAutoFieldRate->isChecked();
    if (m_spinSensorCalibRate) m_spinSensorCalibRate->setEnabled(!anyRunning && !rateFollowsField);
    if (m_checkSensorCalibAutoFieldRate) m_checkSensorCalibAutoFieldRate->setEnabled(!anyRunning);
    if (m_spinVideoExposure) m_spinVideoExposure->setEnabled(!anyRunning);
    if (m_spinVideoIdle) m_spinVideoIdle->setEnabled(true); // display-only, always derived
    // Idle is derived from requested preview rate + exposure and stays read-only.

    // ========== 曝光测试控制区域 ==========
    if (m_btnStartExpTest) m_btnStartExpTest->setEnabled(sensorConnected && !anyRunning);
    if (m_btnStopExpTest) m_btnStopExpTest->setEnabled(expTestRunning);
    // Offline exposure analysis is intentionally available without a sensor,
    // but do not block the GUI with file analysis during any live acquisition.
    if (m_btnAnalyzeExpTestDir) m_btnAnalyzeExpTestDir->setEnabled(!anyRunning);
    if (m_spinExpStart) m_spinExpStart->setEnabled(!expTestRunning && !m_sphereExpActive && !m_small7Active);
    if (m_spinExpEnd) m_spinExpEnd->setEnabled(!expTestRunning && !m_sphereExpActive && !m_small7Active);
    if (m_spinExpStep) m_spinExpStep->setEnabled(!expTestRunning && !m_sphereExpActive && !m_small7Active);
    if (m_spinStableDiscardFrames) m_spinStableDiscardFrames->setEnabled(!expTestRunning && !m_sphereExpActive && !m_small7Active);
    if (m_spinVideoFramesPerExposure) {
        m_spinVideoFramesPerExposure->setEnabled(!expTestRunning && (!m_checkSaveVideo || m_checkSaveVideo->isChecked()));
    }
    if (m_spinProfileFramesPerExposure) {
        m_spinProfileFramesPerExposure->setEnabled(!expTestRunning && (!m_checkSaveProfile || m_checkSaveProfile->isChecked()));
    }
    if (m_editExpTestDir) m_editExpTestDir->setEnabled(!expTestRunning && !m_sphereExpActive && !m_small7Active);
    if (m_comboLighting) m_comboLighting->setEnabled(!expTestRunning && !m_sphereExpActive && !m_small7Active);
    if (m_editMaterial) m_editMaterial->setEnabled(!expTestRunning && !m_sphereExpActive && !m_small7Active);

    // ========== Profile 采集控制区域 ==========
    m_btnStartCapture->setEnabled(sensorConnected && !anyRunning);
    m_btnStopCapture->setEnabled(profileRunning);

    // ========== 传感器参数区域 ==========
    bool canEditParams = sensorConnected && !anyRunning;
    // Material presets may be prepared while disconnected, but must never
    // rewrite Exposure/Idle while a Video/Profile/calibration task is active.
    if (m_comboMaterialExposureProfile) m_comboMaterialExposureProfile->setEnabled(!anyRunning);
    if (m_btnReloadMaterialExposureLibrary) m_btnReloadMaterialExposureLibrary->setEnabled(!anyRunning);
    if (m_comboResolution) m_comboResolution->setEnabled(canEditParams);
    if (m_comboMeasuringField) m_comboMeasuringField->setEnabled(canEditParams);
    if (m_spinHardwareSyncFrequency) m_spinHardwareSyncFrequency->setEnabled(canEditParams);
    if(m_comboHardwareTopology) m_comboHardwareTopology->setEnabled(canEditParams);
    if(m_comboCmmR1Edge) m_comboCmmR1Edge->setEnabled(canEditParams &&
        m_comboHardwareTopology && m_comboHardwareTopology->currentData().toInt()!=1);
    if (m_spinExposure) m_spinExposure->setEnabled(canEditParams);
    if (m_spinIdle) m_spinIdle->setEnabled(canEditParams);
    if (m_radioSoftwareSync) m_radioSoftwareSync->setEnabled(canEditParams);
    if (m_radioHardwareSync) {
        const bool controllerConnected = m_zmc && m_zmc->isConnected();
        const bool linearReferenced = controllerConnected && m_zmc->areLinearMposReferenced();
        const bool rotaryReady = controllerConnected
            && m_zmc->isRotaryAbsoluteReady(AxisC)
            && m_zmc->isRotaryAbsoluteReady(AxisB);
        const bool allAxesHomed = controllerConnected && m_zmc->areAllAxesHomed();
        const bool hwSelectable = canEditParams && controllerConnected && linearReferenced
            && rotaryReady && allAxesHomed && m_hardwareSyncPreflightPassed;
        m_radioHardwareSync->setEnabled(hwSelectable);
        if (!controllerConnected) {
            m_radioHardwareSync->setToolTip(tr("硬件同步等待控制器连接。"));
        }
        else if (!linearReferenced) {
            m_radioHardwareSync->setToolTip(tr("硬件同步等待X/Y/Z完成正规DATUM并建立光栅MPOS机床零点。"));
        }
        else if (!rotaryReady) {
            m_radioHardwareSync->setToolTip(tr("硬件同步等待B/C绝对机械坐标框架建立。"));
        }
        else if (!allAxesHomed) {
            m_radioHardwareSync->setToolTip(tr("硬件同步等待XYZ/C/B五轴完成本次控制器上电会话回零；上位机重启后会从控制器TABLE[357]恢复。"));
        }
        else if (!m_hardwareSyncPreflightPassed) {
            m_radioHardwareSync->setToolTip(tr("五轴坐标已就绪，正在等待/执行V32.13 TABLE只读预检；Home-All最后一个轴完成后会自动重试。"));
        }
        else if (!canEditParams) {
            m_radioHardwareSync->setToolTip(tr("V32.13硬件同步预检已通过；当前Video/Profile/曝光测试正在运行，停止当前传感器任务后即可选择硬件模式。"));
        }
        else {
            m_radioHardwareSync->setToolTip(tr("V32.13硬件同步预检已通过：XYZ/C/B五轴使用REGIST(104)硬件锁存MPOS，可选择硬件同步。"));
        }
    }
    // ECI3808 V32.13 hardware path: single OP14 drives sensor EdgeRising and IN0; XYZ/C/B TABLE data are five-axis REGIST(104) MPOS.
    if (m_comboTriggerType) m_comboTriggerType->setEnabled(false);
    updateSphereStudyUiState();
}

// ==================== 轮廓预览模式控制 ====================

void MainWindow::onStartProfilePreview()
{
    if (!m_laser->isConnected()) {
        QMessageBox::warning(this, tr("提示"), tr("请先连接传感器"));
        updateVideoControlState();
        return;
    }

    if (m_syncManager->isRunning()) {
        // Hardware H1/H3 acquisition owns the trigger and sensor transfer.
        // Calibration preview only attaches to the already decoded profile stream;
        // NEVER reconfigure SoftwareInternal or stopContinuous in this branch.
        if (m_profilePreviewRunning) {
            updateVideoControlState();
            return;
        }
        if (m_laser->isVideoRunning()) {
            QMessageBox::warning(this,tr("预览冲突"),tr("Video正在运行，不能显示采集轮廓"));
            return;
        }
        m_profilePreviewMirrorsCapture = true;
        m_profilePreviewRunning = true;
        m_profilePreviewMirrorSinceMs = QDateTime::currentMSecsSinceEpoch();
        m_profilePreviewMirrorFirstFrameSeen = false;
        LaserProfile baseline;
        m_profilePreviewMirrorBaselineOrdinal = m_laser->latestProfile(baseline)
            ? baseline.callbackOrdinal : 0;
        m_lastDisplayedProfileOrdinal = m_profilePreviewMirrorBaselineOrdinal;
        if(m_profilePreviewView)m_profilePreviewView->clear();
        if(m_viewTabs&&m_previewStack){
            m_viewTabs->setCurrentIndex(m_viewTabs->count()-1);
            m_previewStack->setCurrentIndex(1);
            if(m_radioProfilePreviewMode)m_radioProfilePreviewMode->setChecked(true);
        }
        const bool hardware=(m_syncManager->syncMode()==SyncMode::Hardware);
        const QString msg=hardware
            ? tr("跟随正式硬件同步采集（H1:OP14外触发 / H3:内部轮廓时钟+CMM）：等待新轮廓；只读，不修改触发和TABLE")
            : tr("跟随正式软件同步采集的实时轮廓；显示只读，不重启传感器");
        if(m_lblProfilePreviewStatus){
            m_lblProfilePreviewStatus->setText(msg);
            m_lblProfilePreviewStatus->setStyleSheet("QLabel { color: #236; font-weight: bold; }");
        }
        LOG_INFO(LogCategory::Sensor,tr("[轮廓显示] %1").arg(msg));
        statusBar()->showMessage(msg,8000);
        updateVideoControlState();
        return;
    }

    if (m_laser->isVideoRunning()) {
        QMessageBox::warning(this, tr("模式冲突"), tr("Video 预览正在运行。\n请先停止 Video。"));
        updateVideoControlState();
        return;
    }

    if (m_profilePreviewRunning) {
        updateVideoControlState();
        return;
    }

    if (m_laser->isProfileRunning()) {
        // A stream without a SyncManager owner may belong to another calibration
        // or a partially stopped hardware session. Never commandeer its trigger.
        QMessageBox::warning(this, tr("轮廓传输被占用"),
            tr("检测到传感器已在Profile传输，但当前并非正式SyncManager采集。"
               "为避免覆盖外部触发设置，独立预览不会擅自停流；请先在原任务处安全停止。"));
        updateVideoControlState();
        return;
    }

    QString err;
    if (!enterSensorCalibrationMeasuringField(&err)) {
        QMessageBox::warning(this, tr("测量场设置失败"), err);
        updateVideoControlState();
        return;
    }
    updateSensorCalibrationTimingUi();

    SensorParams params;
    params.measuringFieldIndex = m_sensorCalibActiveFieldIndex;
    params.exposureTime = static_cast<unsigned int>(m_spinVideoExposure->value());
    params.idleTime = static_cast<unsigned int>(m_spinVideoIdle->value());
    if (m_comboResolution && m_comboResolution->count() > 0) {
        params.resolutionIndex = static_cast<unsigned int>(m_comboResolution->currentIndex());
    }

    QString timingErr;
    if (!validateSensorCalibrationTiming(static_cast<int>(params.exposureTime), sensorCalibrationRequestedRateHz(), &timingErr)) {
        leaveSensorCalibrationMeasuringField();
        QMessageBox::warning(this, tr("时序约束失败"), timingErr);
        updateVideoControlState();
        return;
    }
    if (!m_laser->configure(SensorTriggerMode::SoftwareInternal, params, false, HardwareTriggerType::EdgeRising, &err)) {
        leaveSensorCalibrationMeasuringField();
        QMessageBox::warning(this, tr("配置失败"), err);
        updateVideoControlState();
        return;
    }
    if (!verifySensorCalibrationRuntimeConstraints(params, &timingErr, true)) {
        leaveSensorCalibrationMeasuringField();
        QMessageBox::warning(this, tr("传感器约束校验失败"), timingErr);
        updateVideoControlState();
        return;
    }

    if (!m_laser->startContinuous(&err)) {
        leaveSensorCalibrationMeasuringField();
        QMessageBox::warning(this, tr("启动失败"), err);
        updateVideoControlState();
        return;
    }

    m_profilePreviewRunning = true;
    m_profilePreviewMirrorsCapture = false;
    // Independent sensor-calibration display uses the software-internal sensor
    // clock even if the operator selected hardware sync for the NEXT formal run.
    const bool hwSelected=m_radioHardwareSync&&m_radioHardwareSync->isChecked();

    if (m_viewTabs && m_previewStack) {
        m_viewTabs->setCurrentIndex(m_viewTabs->count() - 1);
        m_previewStack->setCurrentIndex(1);
        if (m_radioProfilePreviewMode) m_radioProfilePreviewMode->setChecked(true);
    }

    if (m_profilePreviewView) m_profilePreviewView->clear();

    const QString previewLabel=hwSelected
        ? tr("独立轮廓预览（软件内部触发；非硬件同步测量，正式硬件设置未启用）")
        : tr("软件内部触发：轮廓预览中");
    if (m_lblProfilePreviewStatus) {
        m_lblProfilePreviewStatus->setText(previewLabel);
        m_lblProfilePreviewStatus->setStyleSheet("QLabel { color: green; font-weight: bold; }");
    }
    LOG_INFO(LogCategory::Sensor,tr("[轮廓显示] %1").arg(previewLabel));
    updateVideoControlState();
    statusBar()->showMessage(previewLabel,8000);
}

void MainWindow::onStartSingleSphereCalibrationPreview()
{
    if (!m_laser || !m_laser->isConnected()) {
        QMessageBox::warning(this, tr("提示"), tr("请先连接传感器"));
        return;
    }
    if (m_syncManager && m_syncManager->isRunning()) {
        QMessageBox::warning(this, tr("模式冲突"), tr("在线 Profile 采集正在运行，请先停止采集。"));
        return;
    }
    if (m_laser->isVideoRunning()) {
        QMessageBox::warning(this, tr("模式冲突"), tr("Video 预览正在运行，请先停止 Video。"));
        return;
    }

    // Static calibration uses the same full-field timing contract as the new
    // “传感标定” page.  The helper also remembers the normal acquisition field
    // so the operator-selected Measuring Field / hardware-sync rate are restored after preview.
    if (m_profilePreviewRunning) {
        onStopProfilePreview();
    }
    else if (m_laser->isProfileRunning()) {
        // Do not stop a Profile stream owned by another calibration task.
        QMessageBox::warning(this, tr("轮廓传输被占用"),
            tr("传感器已有其他Profile流，无法接管为单球标定预览；请先从原任务安全停止。"));
        return;
    }

    QString fieldErr;
    if (!enterSensorCalibrationMeasuringField(&fieldErr)) {
        QMessageBox::warning(this, tr("测量场设置失败"), fieldErr);
        return;
    }

    // The normal preview entry configures TRIG_INTERNAL / SoftwareInternal.
    onStartProfilePreview();
    if (m_profilePreviewRunning) {
        if (m_viewTabs && m_profile2DView) m_viewTabs->setCurrentWidget(m_profile2DView);
        statusBar()->showMessage(tr("单球标定预览：软件内部触发，测量场index=%1；显示已切换到二维轮廓").arg(m_sensorCalibActiveFieldIndex), 5000);
    }
}

void MainWindow::onStopProfilePreview()
{
    if (m_profilePreviewMirrorsCapture) {
        // Detach UI only; formal hardware/software acquisition keeps running.
        m_profilePreviewMirrorsCapture = false;
        m_profilePreviewRunning = false;
        m_profilePreviewMirrorBaselineOrdinal = 0;
        m_profilePreviewMirrorFirstFrameSeen = false;
        if(m_lblProfilePreviewStatus){
            m_lblProfilePreviewStatus->setText(tr("已停止显示（正式采集不受影响）"));
            m_lblProfilePreviewStatus->setStyleSheet("QLabel { font-weight: bold; }");
        }
        LOG_INFO(LogCategory::Sensor,tr("[轮廓显示] 已取消只读订阅；采集触发和数据归档未改变"));
        updateVideoControlState();
        statusBar()->showMessage(tr("已停止轮廓显示；正式同步采集继续运行"),5000);
        return;
    }
    if (!m_profilePreviewRunning) {
        if (m_lblProfilePreviewStatus) {
            m_lblProfilePreviewStatus->setText(tr("未启动"));
            m_lblProfilePreviewStatus->setStyleSheet("QLabel { font-weight: bold; }");
        }
        updateVideoControlState();
        return;
    }

    m_laser->stopContinuous();
    m_profilePreviewRunning = false;
    if (m_expTestState == ExpTestState::Idle) leaveSensorCalibrationMeasuringField();

    if (m_lblProfilePreviewStatus) {
        m_lblProfilePreviewStatus->setText(tr("已停止"));
        m_lblProfilePreviewStatus->setStyleSheet("QLabel { font-weight: bold; }");
    }

    updateVideoControlState();
    statusBar()->showMessage(tr("轮廓预览已停止"));
}

void MainWindow::onPreviewModeChanged()
{
    if (!m_previewStack) return;

    if (m_radioVideoMode && m_radioVideoMode->isChecked()) {
        m_previewStack->setCurrentIndex(0);
    }
    else {
        m_previewStack->setCurrentIndex(1);
    }
}

// ==================== 曝光自动测试 ====================

// 辅助函数：将光照条件转换为英文
static QString lightingToEnglish(const QString& lighting) {
    static const QMap<QString, QString> lightingMap = {
        {"黑箱(无光)", "Darkbox"},
        {"室内-荧光灯", "Indoor_Fluorescent"},
        {"室内-LED", "Indoor_LED"},
        {"室内-白炽灯", "Indoor_Incandescent"},
        {"自然光-阴天", "Natural_Cloudy"},
        {"自然光-晴天", "Natural_Sunny"},
        {"强光-直射", "Strong_Direct"},
        {"强光-漫射", "Strong_Diffuse"},
        {"混合光照", "Mixed"},
        {"其他", "Other"}
    };
    return lightingMap.value(lighting, "Unknown");
}

// 辅助函数：生成安全的文件名（只保留英文、数字、下划线）
static QString toSafeFilename(const QString& str) {
    QString result = str;
    // 替换中文和特殊字符
    result.replace(QRegularExpression("[^a-zA-Z0-9_]"), "_");
    // 去除连续下划线
    result.replace(QRegularExpression("_+"), "_");
    // 去除首尾下划线
    result = result.trimmed();
    if (result.startsWith('_')) result = result.mid(1);
    if (result.endsWith('_')) result.chop(1);
    return result.isEmpty() ? "Unknown" : result;
}

void MainWindow::onStartExposureTest()
{
    if (m_sphereExpActive || m_small7Active) {
        QMessageBox::warning(this, tr("操作互锁"), tr("自动球扫描或Small7实验3正在运行；不能同时执行固定点曝光测试。"));
        return;
    }
    if (m_expTestPendingSaveJobs && m_expTestPendingSaveJobs->load() > 0) {
        QMessageBox::warning(this, tr("保存未完成"),
            tr("上一轮传感标定仍有%1个后台保存任务未完成，请等待保存结束后再开始新测试。")
            .arg(m_expTestPendingSaveJobs->load()));
        return;
    }

    // 检查传感器连接
    if (!m_laser->isConnected()) {
        QMessageBox::warning(this, tr("提示"), tr("请先连接传感器"));
        return;
    }

    // 检查是否有其他模式在运行
    if (m_laser->isVideoRunning() || m_profilePreviewRunning || m_syncManager->isRunning()) {
        QMessageBox::warning(this, tr("提示"), tr("请先停止当前运行的模式"));
        return;
    }

    // 检查保存选项
    const bool saveProfile = m_checkSaveProfile->isChecked();
    const bool saveVideo = m_checkSaveVideo->isChecked();
    if (!saveProfile && !saveVideo) {
        QMessageBox::warning(this, tr("参数错误"), tr("请至少选择一种保存类型（轮廓数据或Video图像）"));
        return;
    }

    // 验证参数
    const int expStart = m_spinExpStart->value();
    const int expEnd = m_spinExpEnd->value();
    const int expStep = m_spinExpStep->value();
    if (expStart >= expEnd) {
        QMessageBox::warning(this, tr("参数错误"), tr("起始曝光值必须小于终止值"));
        return;
    }
    // Video and Profile have different exposure/Idle periods. Validate only
    // the capture modes selected for this sweep; Video-only is allowed even
    // when the exposure would be too long for the chosen Profile frequency.
    QString timingConstraintError;
    if (saveVideo && !validateSensorCalibrationVideoTiming(expEnd, &timingConstraintError)) {
        QMessageBox::warning(this, tr("Video曝光/25Hz约束冲突"), timingConstraintError);
        return;
    }
    if (saveProfile && !validateSensorCalibrationTiming(expEnd, sensorCalibrationRequestedRateHz(), &timingConstraintError)) {
        QMessageBox::warning(this, tr("轮廓曝光/频率约束冲突"), timingConstraintError);
        return;
    }

    const QString material = m_editMaterial->text().trimmed();
    if (material.isEmpty()) {
        QMessageBox::warning(this, tr("参数错误"), tr("请输入材料名称"));
        return;
    }

    const QString baseDir = m_editExpTestDir->text();
    if (baseDir.isEmpty()) {
        QMessageBox::warning(this, tr("参数错误"), tr("请选择保存目录"));
        return;
    }

    // 标定使用用户选择的测量场预设。进入前保存原测量区域，完成/中止后恢复，
    // 避免改变正式采集所选 Measuring Field / 硬件同步频率配置。
    QString fieldErr;
    if (!enterSensorCalibrationMeasuringField(&fieldErr)) {
        QMessageBox::warning(this, tr("测量场设置失败"), fieldErr);
        return;
    }
    updateSensorCalibrationTimingUi();

    // 保存实验参数。坐标来源严格保持 Physical17 约定：XYZ=光栅MPOS，C/B=软件机械MPOS。
    m_expTestLighting = lightingToEnglish(m_comboLighting->currentText());
    m_expTestMaterial = toSafeFilename(material);
    m_expTestTemperature = m_spinTemperature->value();
    m_expTestPosX = m_spinPosX->value();
    m_expTestPosY = m_spinPosY->value();
    m_expTestPosZ = m_spinPosZ->value();
    m_expTestPosC = m_spinPosC->value();
    m_expTestPosB = m_spinPosB->value();
    m_expTestSaveProfile = saveProfile;
    m_expTestSaveVideo = saveVideo;
    m_expTestStableDiscardFrames = m_spinStableDiscardFrames ? m_spinStableDiscardFrames->value() : 3;
    m_expTestVideoFramesPerExposure = m_spinVideoFramesPerExposure ? m_spinVideoFramesPerExposure->value() : 3;
    m_expTestProfileFramesPerExposure = m_spinProfileFramesPerExposure ? m_spinProfileFramesPerExposure->value() : 20;

    // 目录名包含当前五轴机床位姿，保留三位小数；'.'/'-' 均为 Windows 合法文件名字符。
    const QString tempStr = QString("Temp%1C").arg(m_expTestTemperature, 0, 'f', 1);
    const QString poseStr = QString("XYZBC_X%1_Y%2_Z%3_C%4_B%5")
        .arg(m_expTestPosX, 0, 'f', 3)
        .arg(m_expTestPosY, 0, 'f', 3)
        .arg(m_expTestPosZ, 0, 'f', 3)
        .arg(m_expTestPosC, 0, 'f', 3)
        .arg(m_expTestPosB, 0, 'f', 3);
    const QString subDirName = QString("%1_%2_%3_%4")
        .arg(m_expTestLighting, m_expTestMaterial, tempStr, poseStr);

    m_expTestDir = baseDir + "/" + subDirName;
    QDir dir(m_expTestDir);
    if (!dir.exists() && !dir.mkpath(".")) {
        leaveSensorCalibrationMeasuringField();
        QMessageBox::warning(this, tr("错误"), tr("无法创建保存目录: %1").arg(m_expTestDir));
        return;
    }

    QString manifestErr;
    if (!writeSensorCalibrationSessionManifest(&manifestErr)) {
        leaveSensorCalibrationMeasuringField();
        QMessageBox::warning(this, tr("错误"), tr("无法建立传感标定session清单: %1").arg(manifestErr));
        return;
    }

    // 初始化测试状态
    m_expTestCurrent = expStart;
    m_expTestEnd = expEnd;
    m_expTestStep = expStep;
    m_expTestDiscardRemaining = 0;
    m_expTestSavedInCurrentMode = 0;
    m_expTestLastVideoOrdinal = m_videoFrameOrdinal;
    m_expTestLastProfileOrdinal = 0;
    m_expTestCaptureDeadlineMs = 0;
    if (m_expTestPendingSaveJobs) m_expTestPendingSaveJobs->store(0);
    // Exposure sweep temporarily drives the visible calibration editor.  Save
    // the operator's original value and restore it on either finish or abort.
    m_sensorCalibExposureBeforeSweep = m_spinVideoExposure ? m_spinVideoExposure->value() : -1;
    m_expTestState = m_expTestSaveVideo ? ExpTestState::StartVideo : ExpTestState::StartProfile;

    // 更新UI
    m_lblExpTestStatus->setText(tr("测试中: %1").arg(m_expTestCurrent));
    m_lblExpTestStatus->setStyleSheet("QLabel { color: blue; font-weight: bold; }");
    m_btnStartExpTest->setEnabled(false);
    m_btnStopExpTest->setEnabled(true);
    if (m_btnAnalyzeExpTestDir) m_btnAnalyzeExpTestDir->setEnabled(false);
    if (m_spinSensorCalibRate) m_spinSensorCalibRate->setEnabled(false);
    m_spinExpStart->setEnabled(false);
    m_spinExpEnd->setEnabled(false);
    m_spinExpStep->setEnabled(false);
    m_spinStableDiscardFrames->setEnabled(false);
    m_spinVideoFramesPerExposure->setEnabled(false);
    m_spinProfileFramesPerExposure->setEnabled(false);
    m_editExpTestDir->setEnabled(false);
    m_comboLighting->setEnabled(false);
    m_editMaterial->setEnabled(false);
    m_spinTemperature->setEnabled(false);
    m_spinPosX->setEnabled(false);
    m_spinPosY->setEnabled(false);
    m_spinPosZ->setEnabled(false);
    m_spinPosC->setEnabled(false);
    m_spinPosB->setEnabled(false);
    m_btnReadPosition->setEnabled(false);
    m_checkSaveProfile->setEnabled(false);
    m_checkSaveVideo->setEnabled(false);

    // 禁用手动预览控制
    m_btnStartVideo->setEnabled(false);
    m_btnStopVideo->setEnabled(false);
    m_btnStartProfilePreview->setEnabled(false);
    m_btnStopProfilePreview->setEnabled(false);

    // 切换到左侧“传感预览”Tab
    if (m_viewTabs) m_viewTabs->setCurrentIndex(m_viewTabs->count() - 1);

    const double firstHz = sensorCalibrationRequestedRateHz(); // profile only; Video is always 25Hz
    statusBar()->showMessage(tr("曝光测试开始: %1 ~ %2, 步长 %3；Video固定25Hz，Profile目标%4 Hz；稳定丢弃%5帧；每档Video=%6张/Profile=%7条\n保存目录: %8")
        .arg(expStart).arg(expEnd).arg(expStep).arg(firstHz, 0, 'f', 1)
        .arg(m_expTestStableDiscardFrames).arg(m_expTestVideoFramesPerExposure)
        .arg(m_expTestProfileFramesPerExposure).arg(m_expTestDir));

    LOG_INFO(LogCategory::Sensor,
        tr("曝光测试开始 - 所选测量场，目标Profile Rate=%1Hz；稳定丢弃=%2帧；每档Video=%3张/Profile=%4条；光照:%5 材料:%6 温度:%7°C XYZBC=(%8,%9,%10,%11,%12)，坐标源XYZ=光栅MPOS/C/B=机械MPOS")
        .arg(sensorCalibrationRequestedRateHz())
        .arg(m_expTestStableDiscardFrames)
        .arg(m_expTestVideoFramesPerExposure)
        .arg(m_expTestProfileFramesPerExposure)
        .arg(m_expTestLighting).arg(m_expTestMaterial).arg(m_expTestTemperature)
        .arg(m_expTestPosX).arg(m_expTestPosY).arg(m_expTestPosZ)
        .arg(m_expTestPosC).arg(m_expTestPosB));

    m_expTestTimer.setSingleShot(true);
    m_expTestTimer.start(100);
}

void MainWindow::onStopExposureTest()
{
    m_expTestTimer.stop();

    if (m_laser->isVideoRunning()) m_laser->stopVideoStream();
    if (m_profilePreviewRunning) {
        m_laser->stopContinuous();
        m_profilePreviewRunning = false;
    }
    m_laser->setSensorCalibrationRichProfileEnabled(false);

    m_expTestState = ExpTestState::Idle;
    leaveSensorCalibrationMeasuringField();
    if (m_sensorCalibExposureBeforeSweep > 0 && m_spinVideoExposure)
        m_spinVideoExposure->setValue(m_sensorCalibExposureBeforeSweep);
    m_sensorCalibExposureBeforeSweep = -1;
    updateSensorCalibrationTimingUi(); // restore operator's preview exposure

    m_lblExpTestStatus->setText(tr("已停止"));
    m_lblExpTestStatus->setStyleSheet("QLabel { font-weight: bold; }");
    m_btnStartExpTest->setEnabled(true);
    m_btnStopExpTest->setEnabled(false);
    if (m_btnAnalyzeExpTestDir) m_btnAnalyzeExpTestDir->setEnabled(true);
    if (m_spinSensorCalibRate) m_spinSensorCalibRate->setEnabled(
        !(m_checkSensorCalibAutoFieldRate && m_checkSensorCalibAutoFieldRate->isChecked()));
    m_spinExpStart->setEnabled(true);
    m_spinExpEnd->setEnabled(true);
    m_spinExpStep->setEnabled(true);
    m_spinStableDiscardFrames->setEnabled(true);
    m_spinVideoFramesPerExposure->setEnabled(true);
    m_spinProfileFramesPerExposure->setEnabled(true);
    m_editExpTestDir->setEnabled(true);
    m_comboLighting->setEnabled(true);
    m_editMaterial->setEnabled(true);
    m_spinTemperature->setEnabled(true);
    m_spinPosX->setEnabled(true);
    m_spinPosY->setEnabled(true);
    m_spinPosZ->setEnabled(true);
    m_spinPosC->setEnabled(true);
    m_spinPosB->setEnabled(true);
    m_btnReadPosition->setEnabled(true);
    m_checkSaveProfile->setEnabled(true);
    m_checkSaveVideo->setEnabled(true);

    updateVideoControlState();
    statusBar()->showMessage(tr("曝光测试已停止，已恢复正式采集测量区域"));
}

void MainWindow::onExposureTestStep()
{
    QString err;
    constexpr int kFramePollMs = 5;
    auto armCaptureDeadline = [this](int requestedSamples, int captureRateHz) {
        const int nominalFramePeriodMs = std::max(
            1, static_cast<int>(std::ceil(1000.0 / std::max(1, captureRateHz))));
        // Wait for actual new frames rather than a fixed wall-clock delay.  The
        // watchdog is intentionally generous so a transient GUI/network stall
        // does not abort a valid calibration run, but a dead stream cannot hang
        // the state machine forever.
        const qint64 nominalMs = static_cast<qint64>(nominalFramePeriodMs)
            * (m_expTestStableDiscardFrames + std::max(1, requestedSamples) + 10);
        const qint64 timeoutMs = std::max<qint64>(10000, nominalMs * 6);
        m_expTestCaptureDeadlineMs = QDateTime::currentMSecsSinceEpoch() + timeoutMs;
        };

    // 文件名前缀: 材料_光照_曝光值
    QString filePrefix = QString("%1/%2_%3_exp%4")
        .arg(m_expTestDir)
        .arg(m_expTestMaterial)
        .arg(m_expTestLighting)
        .arg(m_expTestCurrent, 4, 10, QChar('0'));

    // 计算进度
    int totalSteps = (m_expTestEnd - m_spinExpStart->value()) / m_expTestStep + 1;
    int currentStep = (m_expTestCurrent - m_spinExpStart->value()) / m_expTestStep + 1;

    switch (m_expTestState) {
    case ExpTestState::StartVideo: {
        // 只有需要保存Video时才启动
        if (!m_expTestSaveVideo) {
            // 跳过Video，直接进入Profile
            if (m_expTestSaveProfile) {
                m_expTestState = ExpTestState::StartProfile;
                m_expTestTimer.start(100);
            }
            else {
                // 都不保存，直接下一个曝光值
                m_expTestState = ExpTestState::NextExposure;
                m_expTestTimer.start(100);
            }
            break;
        }

        // Raw 29xx Video runs at its own 25Hz, NOT the selected Profile rate.
        m_spinVideoExposure->setValue(m_expTestCurrent);
        const int autoIdle = sensorCalibrationVideoIdleUnitsForExposure(m_expTestCurrent);
        const double estimatedHz = 100000.0 / std::max(1, m_expTestCurrent + autoIdle);
        m_lblExpTestStatus->setText(
            tr("[%1/%2] Video: Exp=%3 Idle=%4 (~%5Hz)")
            .arg(currentStep).arg(totalSteps).arg(m_expTestCurrent).arg(autoIdle)
            .arg(estimatedHz, 0, 'f', 1));

        SensorParams params;
        params.measuringFieldIndex = m_sensorCalibActiveFieldIndex;
        params.exposureTime = static_cast<unsigned int>(m_expTestCurrent);
        params.idleTime = static_cast<unsigned int>(autoIdle);

        QString timingErr;
        if (!validateSensorCalibrationVideoTiming(static_cast<int>(params.exposureTime), &timingErr)) {
            QMessageBox::warning(this, tr("错误"), tr("Video 25Hz时序约束失败: %1").arg(timingErr));
            onStopExposureTest();
            return;
        }
        if (!m_laser->configureVideo(params, &err)) {
            QMessageBox::warning(this, tr("错误"), tr("Video配置失败: %1").arg(err));
            onStopExposureTest();
            return;
        }
        if (!verifySensorCalibrationRuntimeConstraints(params, &timingErr)) {
            QMessageBox::warning(this, tr("错误"), tr("Video设备回读约束失败: %1").arg(timingErr));
            onStopExposureTest();
            return;
        }

        if (!m_laser->startVideoStream(&err)) {
            QMessageBox::warning(this, tr("错误"), tr("Video启动失败: %1").arg(err));
            onStopExposureTest();
            return;
        }

        if (m_videoWidget) m_videoWidget->clear();
        m_expTestLastVideoOrdinal = m_videoFrameOrdinal;
        m_expTestDiscardRemaining = m_expTestStableDiscardFrames;
        m_expTestSavedInCurrentMode = 0;
        armCaptureDeadline(m_expTestVideoFramesPerExposure,
            mls::calibration::SensorCalibrationTiming::kVideoRateHz);

        // 切换到Video显示
        if (m_previewStack) m_previewStack->setCurrentIndex(0);
        if (m_radioVideoMode) m_radioVideoMode->setChecked(true);

        m_expTestState = ExpTestState::CaptureVideo;
        m_expTestTimer.start(kFramePollMs);
        break;
    }

    case ExpTestState::CaptureVideo: {
        const qint64 nowMs = QDateTime::currentMSecsSinceEpoch();
        if (m_videoFrameOrdinal <= m_expTestLastVideoOrdinal) {
            if (nowMs > m_expTestCaptureDeadlineMs) {
                QMessageBox::warning(this, tr("错误"),
                    tr("Video等待真实新帧超时：Exp=%1，已保存%2/%3，稳定丢弃剩余%4")
                    .arg(m_expTestCurrent).arg(m_expTestSavedInCurrentMode)
                    .arg(m_expTestVideoFramesPerExposure).arg(m_expTestDiscardRemaining));
                onStopExposureTest();
                return;
            }
            m_expTestTimer.start(kFramePollMs);
            break;
        }

        const quint64 newFrames = m_videoFrameOrdinal - m_expTestLastVideoOrdinal;
        m_expTestLastVideoOrdinal = m_videoFrameOrdinal;
        if (m_expTestDiscardRemaining > 0) {
            const int discardedNow = std::min<int>(m_expTestDiscardRemaining,
                static_cast<int>(std::min<quint64>(newFrames, 1000000u)));
            m_expTestDiscardRemaining -= discardedNow;
            m_lblExpTestStatus->setText(
                tr("[%1/%2] Video稳定中: Exp=%3，剩余丢弃%4帧")
                .arg(currentStep).arg(totalSteps).arg(m_expTestCurrent).arg(m_expTestDiscardRemaining));
            if (m_expTestDiscardRemaining > 0 || newFrames <= static_cast<quint64>(discardedNow)) {
                m_expTestTimer.start(kFramePollMs);
                break;
            }
        }

        if (!m_videoWidget) {
            QMessageBox::warning(this, tr("错误"), tr("Video显示组件不可用"));
            onStopExposureTest();
            return;
        }

        QImage frame = m_videoWidget->currentFrame();
        if (frame.isNull()) {
            if (nowMs > m_expTestCaptureDeadlineMs) {
                QMessageBox::warning(this, tr("错误"), tr("Video已收到帧事件但图像为空，等待超时"));
                onStopExposureTest();
                return;
            }
            m_expTestTimer.start(kFramePollMs);
            break;
        }

        const int sampleIndex = ++m_expTestSavedInCurrentMode;
        const QString prefix = filePrefix + QString("_video_f%1").arg(sampleIndex, 3, 10, QChar('0'));
        const QString material = m_expTestMaterial;
        const QString lighting = m_expTestLighting;
        const int exposure = m_expTestCurrent;
        const int idleUnits = sensorCalibrationVideoIdleUnitsForExposure(exposure);
        const double derivedHz = 100000.0 / std::max(1, exposure + idleUnits);
        const int targetRateHz = sensorCalibrationRequestedRateHz();
        const int videoRateHz = mls::calibration::SensorCalibrationTiming::kVideoRateHz;
        const int fieldIndex = m_sensorCalibActiveFieldIndex;
        const double temperature = m_expTestTemperature;
        const double posX = m_expTestPosX;
        const double posY = m_expTestPosY;
        const double posZ = m_expTestPosZ;
        const double posC = m_expTestPosC;
        const double posB = m_expTestPosB;
        const quint64 receiveOrdinal = m_videoFrameOrdinal;
        const QString captureUtc = QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs);

        const auto pendingSaves = m_expTestPendingSaveJobs;
        pendingSaves->fetch_add(1);
        QThreadPool::globalInstance()->start(
            [frame, prefix, material, lighting, exposure, idleUnits, derivedHz, targetRateHz, videoRateHz, fieldIndex,
            temperature, posX, posY, posZ, posC, posB, sampleIndex, receiveOrdinal, captureUtc,
            pendingSaves]() {
                const QString pngFilename = prefix + ".png";
                if (!frame.save(pngFilename, "PNG")) {
                    qWarning() << "Failed to save video PNG:" << pngFilename;
                }

                const QString rawFilename = prefix + "_raw.csv";
                QFile rawFile(rawFilename);
                if (rawFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
                    QTextStream out(&rawFile);
                    out << "# Video Raw Data\n";
                    out << "# Schema: MotionLaserSync.SensorCalibrationVideo.v3\n";
                    out << "# SessionManifest: sensor_calibration_session.json\n";
                    out << "# CaptureUTC: " << captureUtc << "\n";
                    out << "# SampleIndex: " << sampleIndex << "\n";
                    out << "# VideoReceiveOrdinal: " << receiveOrdinal << "\n";
                    out << "# Material: " << material << "\n";
                    out << "# Lighting: " << lighting << "\n";
                    out << "# MeasuringField: MEASURINGFIELD index=" << fieldIndex << "\n";
                    out << "# Exposure: " << exposure << " (x10us)\n";
                    out << "# Idle_Auto: " << idleUnits << " (x10us)\n";
                    out << "# RequestedProfileRate: " << targetRateHz << " Hz (not used for Video)\n";
                    out << "# VideoRate: " << videoRateHz << " Hz\n";
                    out << "# VideoIdle_Auto: " << idleUnits << " (x10us)\n";
                    out << "# DerivedVideoRateFromExpIdle: " << derivedHz << " Hz\n";
                    out << "# Temperature: " << temperature << " C\n";
                    out << "# Position_X_MPOS: " << posX << " mm\n";
                    out << "# Position_Y_MPOS: " << posY << " mm\n";
                    out << "# Position_Z_MPOS: " << posZ << " mm\n";
                    out << "# Position_C_MechanicalMPOS: " << posC << " deg\n";
                    out << "# Position_B_MechanicalMPOS: " << posB << " deg\n";
                    out << "# Width: " << frame.width() << "\n";
                    out << "# Height: " << frame.height() << "\n";
                    out << "# Format: Row,Col,Value\n";

                    for (int y = 0; y < frame.height(); ++y) {
                        const uchar* line = frame.constScanLine(y);
                        for (int x = 0; x < frame.width(); ++x) {
                            int grayValue;
                            if (frame.format() == QImage::Format_Grayscale8 ||
                                frame.format() == QImage::Format_Indexed8) {
                                grayValue = line[x];
                            }
                            else {
                                grayValue = qGray(frame.pixel(x, y));
                            }
                            out << y << ',' << x << ',' << grayValue << '\n';
                        }
                    }
                    rawFile.close();
                }
                else {
                    qWarning() << "Failed to open file for writing:" << rawFilename;
                }
                pendingSaves->fetch_sub(1);
            });

        m_lblExpTestStatus->setText(
            tr("[%1/%2] Video保存: Exp=%3，%4/%5")
            .arg(currentStep).arg(totalSteps).arg(m_expTestCurrent)
            .arg(m_expTestSavedInCurrentMode).arg(m_expTestVideoFramesPerExposure));

        if (m_expTestSavedInCurrentMode >= m_expTestVideoFramesPerExposure) {
            m_expTestState = ExpTestState::StopVideo;
            m_expTestTimer.start(50);
        }
        else {
            m_expTestTimer.start(kFramePollMs);
        }
        break;
    }

    case ExpTestState::StopVideo: {
        m_laser->stopVideoStream();

        // 根据保存选项决定下一状态
        if (m_expTestSaveProfile) {
            m_expTestState = ExpTestState::StartProfile;
        }
        else {
            m_expTestState = ExpTestState::NextExposure;
        }
        m_expTestTimer.start(300);  // 等待Video完全停止
        break;
    }

    case ExpTestState::StartProfile: {
        // 启动轮廓预览；每个曝光点都按目标Profile Rate重新计算Idle。
        m_spinVideoExposure->setValue(m_expTestCurrent);
        const int autoIdle = sensorCalibrationIdleUnitsForExposure(m_expTestCurrent);
        m_spinVideoIdle->setValue(autoIdle);
        const double estimatedHz = 100000.0 / std::max(1, m_expTestCurrent + autoIdle);
        m_lblExpTestStatus->setText(
            tr("[%1/%2] 轮廓: Exp=%3 Idle=%4 (~%5Hz)")
            .arg(currentStep).arg(totalSteps).arg(m_expTestCurrent).arg(autoIdle)
            .arg(estimatedHz, 0, 'f', 1));

        SensorParams params;
        params.measuringFieldIndex = m_sensorCalibActiveFieldIndex;
        params.exposureTime = static_cast<unsigned int>(m_expTestCurrent);
        params.idleTime = static_cast<unsigned int>(autoIdle);
        if (m_comboResolution && m_comboResolution->count() > 0) {
            params.resolutionIndex = static_cast<unsigned int>(m_comboResolution->currentIndex());
        }

        QString timingErr;
        if (!validateSensorCalibrationTiming(static_cast<int>(params.exposureTime), sensorCalibrationRequestedRateHz(), &timingErr)) {
            QMessageBox::warning(this, tr("错误"), tr("轮廓时序约束失败: %1").arg(timingErr));
            onStopExposureTest();
            return;
        }
        if (!m_laser->configure(SensorTriggerMode::SoftwareInternal, params, false, HardwareTriggerType::EdgeRising, &err)) {
            QMessageBox::warning(this, tr("错误"), tr("轮廓配置失败: %1").arg(err));
            onStopExposureTest();
            return;
        }
        if (!verifySensorCalibrationRuntimeConstraints(params, &timingErr, true)) {
            QMessageBox::warning(this, tr("错误"), tr("轮廓设备回读约束失败: %1").arg(timingErr));
            onStopExposureTest();
            return;
        }

        // V27.6.16R: only the sensor-calibration exposure sweep requests the
        // auxiliary LLT profile channels (Width/Maximum/Threshold/M0/M1).
        // Hardware-sync acquisition never enables this flag.
        m_laser->setSensorCalibrationRichProfileEnabled(true);

        if (!m_laser->startContinuous(&err)) {
            QMessageBox::warning(this, tr("错误"), tr("轮廓启动失败: %1").arg(err));
            onStopExposureTest();
            return;
        }

        m_expTestLastProfileOrdinal = 0;
        m_expTestDiscardRemaining = m_expTestStableDiscardFrames;
        m_expTestSavedInCurrentMode = 0;
        armCaptureDeadline(m_expTestProfileFramesPerExposure, sensorCalibrationRequestedRateHz());

        m_profilePreviewRunning = true;

        // 切换到轮廓显示
        if (m_previewStack) m_previewStack->setCurrentIndex(1);
        if (m_radioProfilePreviewMode) m_radioProfilePreviewMode->setChecked(true);

        m_expTestState = ExpTestState::CaptureProfile;
        m_expTestTimer.start(kFramePollMs);
        break;
    }

    case ExpTestState::CaptureProfile: {
        LaserSensor::SensorCalibrationProfileFrame richFrame;
        const bool haveFrame = m_laser->latestSensorCalibrationProfile(richFrame)
            && !richFrame.profile.x.isEmpty() && !richFrame.profile.z.isEmpty();
        const qint64 nowMs = QDateTime::currentMSecsSinceEpoch();

        if (!haveFrame || richFrame.profile.callbackOrdinal <= m_expTestLastProfileOrdinal) {
            if (nowMs > m_expTestCaptureDeadlineMs) {
                QMessageBox::warning(this, tr("错误"),
                    tr("Profile等待真实新帧超时：Exp=%1，已保存%2/%3，稳定丢弃剩余%4")
                    .arg(m_expTestCurrent).arg(m_expTestSavedInCurrentMode)
                    .arg(m_expTestProfileFramesPerExposure).arg(m_expTestDiscardRemaining));
                onStopExposureTest();
                return;
            }
            m_expTestTimer.start(kFramePollMs);
            break;
        }

        const quint64 currentOrdinal = richFrame.profile.callbackOrdinal;
        const quint64 newProfiles = (m_expTestLastProfileOrdinal == 0)
            ? currentOrdinal
            : (currentOrdinal - m_expTestLastProfileOrdinal);
        m_expTestLastProfileOrdinal = currentOrdinal;

        if (m_expTestDiscardRemaining > 0) {
            const int discardedNow = std::min<int>(m_expTestDiscardRemaining,
                static_cast<int>(std::min<quint64>(newProfiles, 1000000u)));
            m_expTestDiscardRemaining -= discardedNow;
            m_lblExpTestStatus->setText(
                tr("[%1/%2] Profile稳定中: Exp=%3，剩余丢弃%4帧")
                .arg(currentStep).arg(totalSteps).arg(m_expTestCurrent).arg(m_expTestDiscardRemaining));
            if (m_expTestDiscardRemaining > 0 || newProfiles <= static_cast<quint64>(discardedNow)) {
                m_expTestTimer.start(kFramePollMs);
                break;
            }
        }

        const int expectedPoints = m_comboResolution && m_comboResolution->currentData().isValid()
            ? m_comboResolution->currentData().toInt() : 0;
        if (expectedPoints <= 0 || richFrame.profile.x.size() != expectedPoints
            || richFrame.profile.z.size() != expectedPoints) {
            QMessageBox::warning(this, tr("轮廓格式不匹配"),
                tr("测量场%1：要求分辨率%2点，实际X/Z=%3/%4，停止此档采集")
                .arg(m_sensorCalibActiveFieldIndex).arg(expectedPoints)
                .arg(richFrame.profile.x.size()).arg(richFrame.profile.z.size()));
            onStopExposureTest();
            return;
        }
        const int sampleIndex = ++m_expTestSavedInCurrentMode;
        // QPixmap is GUI-thread bound. Convert to QImage before handing to the
        // snapshot to the background save pool.
        QImage profilePreviewImage;
        if (m_profilePreviewView) profilePreviewImage = m_profilePreviewView->grab().toImage();

        const QString prefix = filePrefix + QString("_profile_f%1").arg(sampleIndex, 3, 10, QChar('0'));
        const QString material = m_expTestMaterial;
        const QString lighting = m_expTestLighting;
        const int exposure = m_expTestCurrent;
        const int idleUnits = sensorCalibrationIdleUnitsForExposure(exposure);
        const double derivedHz = 100000.0 / std::max(1, exposure + idleUnits);
        const int targetRateHz = sensorCalibrationRequestedRateHz();
        const int fieldIndex = m_sensorCalibActiveFieldIndex;
        const double temperature = m_expTestTemperature;
        const double posX = m_expTestPosX;
        const double posY = m_expTestPosY;
        const double posZ = m_expTestPosZ;
        const double posC = m_expTestPosC;
        const double posB = m_expTestPosB;
        const QString captureUtc = QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs);

        // Copy one immutable rich frame into the save job. Only this static
        // calibration workflow requests the auxiliary vectors. Formal hardware
        // sync remains on the unchanged X/Z-only path.
        const auto pendingSaves = m_expTestPendingSaveJobs;
        pendingSaves->fetch_add(1);
        QThreadPool::globalInstance()->start(
            [profilePreviewImage, prefix, material, lighting, exposure, idleUnits, derivedHz, targetRateHz, fieldIndex,
            temperature, posX, posY, posZ, posC, posB, captureUtc, sampleIndex, richFrame,
            pendingSaves]() {
                const LaserProfile& profile = richFrame.profile;
                const int pointCount = std::min(profile.x.size(), profile.z.size());
                const bool hasWidth = richFrame.hasWidth();
                const bool hasMaximum = richFrame.hasMaximum();
                const bool hasThreshold = richFrame.hasThreshold();
                const bool hasM0 = richFrame.hasMoment0();
                const bool hasM1 = richFrame.hasMoment1();

                if (!profilePreviewImage.isNull()) {
                    const QString imgFilename = prefix + ".png";
                    if (!profilePreviewImage.save(imgFilename, "PNG")) {
                        qWarning() << "Failed to save profile image:" << imgFilename;
                    }
                }

                QJsonObject meta;
                meta[QStringLiteral("schema")] = QStringLiteral("MotionLaserSync.SensorCalibrationProfileMeta.v3");
                meta[QStringLiteral("sessionManifest")] = QStringLiteral("sensor_calibration_session.json");
                meta[QStringLiteral("captureUtc")] = captureUtc;
                meta[QStringLiteral("sampleIndex")] = sampleIndex;

                QJsonObject experiment;
                experiment[QStringLiteral("material")] = material;
                experiment[QStringLiteral("lighting")] = lighting;
                experiment[QStringLiteral("temperatureC")] = temperature;
                experiment[QStringLiteral("measuringFieldIndex")] = fieldIndex;
                experiment[QStringLiteral("exposure_10us")] = exposure;
                experiment[QStringLiteral("idle_10us")] = idleUnits;
                experiment[QStringLiteral("requestedProfileRateHz")] = targetRateHz;
                experiment[QStringLiteral("derivedProfileRateFromExpIdleHz")] = derivedHz;
                meta[QStringLiteral("experiment")] = experiment;

                QJsonObject pose;
                pose[QStringLiteral("x_MPOS_mm")] = posX;
                pose[QStringLiteral("y_MPOS_mm")] = posY;
                pose[QStringLiteral("z_MPOS_mm")] = posZ;
                pose[QStringLiteral("c_MechanicalMPOS_deg")] = posC;
                pose[QStringLiteral("b_MechanicalMPOS_deg")] = posB;
                meta[QStringLiteral("machinePose")] = pose;

                QJsonObject pmeta;
                pmeta[QStringLiteral("index")] = static_cast<int>(profile.index);
                pmeta[QStringLiteral("sensorOpenSec")] = profile.tOpen;
                pmeta[QStringLiteral("sensorCloseSec")] = profile.tClose;
                pmeta[QStringLiteral("sensorExposureSec")] = profile.tClose - profile.tOpen;
                pmeta[QStringLiteral("pcCallbackEntryUs")] = static_cast<double>(profile.tPcUs);
                pmeta[QStringLiteral("pcDecodeDoneUs")] = static_cast<double>(profile.decodeDonePcUs);
                pmeta[QStringLiteral("callbackOrdinal")] = static_cast<double>(profile.callbackOrdinal);
                pmeta[QStringLiteral("rawBytes")] = static_cast<int>(profile.rawBytes);
                pmeta[QStringLiteral("ingressDepth")] = static_cast<int>(profile.ingressDepth);
                pmeta[QStringLiteral("callbackServiceUs")] = static_cast<double>(profile.callbackServiceUs);
                pmeta[QStringLiteral("decoderQueueUs")] = static_cast<double>(profile.decoderQueueUs);
                pmeta[QStringLiteral("decodeServiceUs")] = static_cast<double>(profile.decodeServiceUs);
                pmeta[QStringLiteral("convertFlags")] = richFrame.convertFlags;
                pmeta[QStringLiteral("pointCount")] = pointCount;
                QJsonObject channels;
                channels[QStringLiteral("X")] = true;
                channels[QStringLiteral("Z")] = true;
                channels[QStringLiteral("Width")] = hasWidth;
                channels[QStringLiteral("Maximum")] = hasMaximum;
                channels[QStringLiteral("Threshold")] = hasThreshold;
                channels[QStringLiteral("Moment0")] = hasM0;
                channels[QStringLiteral("Moment1")] = hasM1;
                pmeta[QStringLiteral("availableChannels")] = channels;
                meta[QStringLiteral("profile")] = pmeta;

                const QString metaFilename = prefix + "_meta.json";
                QFile metaFile(metaFilename);
                if (metaFile.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
                    metaFile.write(QJsonDocument(meta).toJson(QJsonDocument::Indented));
                    metaFile.close();
                }
                else {
                    qWarning() << "Failed to save profile metadata:" << metaFilename;
                }

                const QString csvFilename = prefix + ".csv";
                QFile file(csvFilename);
                if (file.open(QIODevice::WriteOnly | QIODevice::Text)) {
                    QTextStream out(&file);
                    out.setRealNumberNotation(QTextStream::SmartNotation);
                    out.setRealNumberPrecision(15);
                    out << "# Sensor Calibration Rich Profile Data\n";
                    out << "# Schema: MotionLaserSync.SensorCalibrationProfile.v4\n";
                    out << "# SessionManifest: sensor_calibration_session.json\n";
                    out << "# ProfileMeta: " << QFileInfo(metaFilename).fileName() << "\n";
                    out << "# CaptureUTC: " << captureUtc << "\n";
                    out << "# SampleIndex: " << sampleIndex << "\n";
                    out << "# Material: " << material << "\n";
                    out << "# Lighting: " << lighting << "\n";
                    out << "# MeasuringField: MEASURINGFIELD index=" << fieldIndex << "\n";
                    out << "# Exposure: " << exposure << " (x10us)\n";
                    out << "# Idle_Auto: " << idleUnits << " (x10us)\n";
                    out << "# RequestedProfileRate: " << targetRateHz << " Hz\n";
                    out << "# DerivedProfileRateFromExpIdle: " << derivedHz << " Hz\n";
                    out << "# Temperature: " << temperature << " C\n";
                    out << "# Position_X_MPOS: " << posX << " mm\n";
                    out << "# Position_Y_MPOS: " << posY << " mm\n";
                    out << "# Position_Z_MPOS: " << posZ << " mm\n";
                    out << "# Position_C_MechanicalMPOS: " << posC << " deg\n";
                    out << "# Position_B_MechanicalMPOS: " << posB << " deg\n";
                    out << "# ProfileIndex: " << profile.index << "\n";
                    out << "# SensorOpenSec: " << profile.tOpen << "\n";
                    out << "# SensorCloseSec: " << profile.tClose << "\n";
                    out << "# SensorExposureSec: " << (profile.tClose - profile.tOpen) << "\n";
                    out << "# PcCallbackEntryUs: " << profile.tPcUs << "\n";
                    out << "# PcDecodeDoneUs: " << profile.decodeDonePcUs << "\n";
                    out << "# CallbackOrdinal: " << profile.callbackOrdinal << "\n";
                    out << "# RawBytes: " << profile.rawBytes << "\n";
                    out << "# IngressDepth: " << profile.ingressDepth << "\n";
                    out << "# CallbackServiceUs: " << profile.callbackServiceUs << "\n";
                    out << "# DecoderQueueUs: " << profile.decoderQueueUs << "\n";
                    out << "# DecodeServiceUs: " << profile.decodeServiceUs << "\n";
                    out << "# ConvertFlags: 0x" << QString::number(richFrame.convertFlags, 16) << "\n";
                    out << "# HasWidth: " << (hasWidth ? 1 : 0) << "\n";
                    out << "# HasMaximum: " << (hasMaximum ? 1 : 0) << "\n";
                    out << "# HasThreshold: " << (hasThreshold ? 1 : 0) << "\n";
                    out << "# HasMoment0: " << (hasM0 ? 1 : 0) << "\n";
                    out << "# HasMoment1: " << (hasM1 ? 1 : 0) << "\n";
                    out << "# MaximumSemantics: LLT SDK pMaximum (peak maximum / intensity-like output)\n";
                    out << "# Points: " << pointCount << "\n";
                    out << "PointIndex,X_mm,Z_mm,Width,Maximum,Threshold,Moment0,Moment1\n";

                    auto writeU16 = [&out](const QVector<unsigned short>& v, bool valid, int i) {
                        if (valid && i >= 0 && i < v.size()) out << v[i];
                        };
                    auto writeU32 = [&out](const QVector<unsigned int>& v, bool valid, int i) {
                        if (valid && i >= 0 && i < v.size()) out << v[i];
                        };

                    for (int i = 0; i < pointCount; ++i) {
                        out << i << ',' << profile.x[i] << ',' << profile.z[i] << ',';
                        writeU16(richFrame.width, hasWidth, i); out << ',';
                        writeU16(richFrame.maximum, hasMaximum, i); out << ',';
                        writeU16(richFrame.threshold, hasThreshold, i); out << ',';
                        writeU32(richFrame.moment0, hasM0, i); out << ',';
                        writeU32(richFrame.moment1, hasM1, i); out << '\n';
                    }
                    file.close();
                }
                else {
                    qWarning() << "Failed to save rich profile CSV:" << csvFilename;
                }
                pendingSaves->fetch_sub(1);
            });

        m_lblExpTestStatus->setText(
            tr("[%1/%2] Profile保存: Exp=%3，%4/%5")
            .arg(currentStep).arg(totalSteps).arg(m_expTestCurrent)
            .arg(m_expTestSavedInCurrentMode).arg(m_expTestProfileFramesPerExposure));

        if (m_expTestSavedInCurrentMode >= m_expTestProfileFramesPerExposure) {
            m_expTestState = ExpTestState::StopProfile;
            m_expTestTimer.start(50);
        }
        else {
            m_expTestTimer.start(kFramePollMs);
        }
        break;
    }

    case ExpTestState::StopProfile: {
        m_laser->stopContinuous();
        m_laser->setSensorCalibrationRichProfileEnabled(false);
        m_profilePreviewRunning = false;
        m_expTestState = ExpTestState::NextExposure;
        m_expTestTimer.start(300);  // 等待保存完成
        break;
    }

    case ExpTestState::NextExposure: {
        // 下一个曝光值
        m_expTestCurrent += m_expTestStep;

        if (m_expTestCurrent > m_expTestEnd) {
            // 测试完成，等待所有后台保存完成
            m_lblExpTestStatus->setText(tr("保存文件中..."));
            m_expTestState = ExpTestState::Complete;
            m_expTestTimer.start(100);   // 由pending-save计数器确认全部落盘后再宣布完成
        }
        else {
            // 继续下一轮，根据保存选项决定起始状态
            if (m_expTestSaveVideo) {
                m_expTestState = ExpTestState::StartVideo;
            }
            else if (m_expTestSaveProfile) {
                m_expTestState = ExpTestState::StartProfile;
            }
            else {
                m_expTestState = ExpTestState::NextExposure;
            }
            m_expTestTimer.start(100);
        }
        break;
    }

    case ExpTestState::Complete: {
        const int pendingSaveJobs = m_expTestPendingSaveJobs ? m_expTestPendingSaveJobs->load() : 0;
        if (pendingSaveJobs > 0) {
            m_lblExpTestStatus->setText(tr("保存文件中... 剩余%1个任务").arg(pendingSaveJobs));
            m_expTestTimer.start(100);
            break;
        }

        QString analysisSummary;
        if (m_expTestSaveProfile) {
            m_lblExpTestStatus->setText(tr("数据已保存，正在分析最优曝光..."));
            runExposureAnalysisAndImport(m_expTestDir, false, &analysisSummary);
        }
        else {
            analysisSummary = tr("本次未保存Profile，因此未执行最优曝光分析");
            if (m_lblExpAnalysisResult) {
                m_lblExpAnalysisResult->setStyleSheet("color:#777; font-size:9px;");
                m_lblExpAnalysisResult->setText(tr("分析结果：%1").arg(analysisSummary));
            }
        }

        m_expTestState = ExpTestState::Idle;
        m_laser->setSensorCalibrationRichProfileEnabled(false);
        if (m_sensorCalibExposureBeforeSweep > 0 && m_spinVideoExposure)
            m_spinVideoExposure->setValue(m_sensorCalibExposureBeforeSweep);
        m_sensorCalibExposureBeforeSweep = -1;
        updateSensorCalibrationTimingUi(); // restore operator's preview exposure

        // 更新UI
        m_lblExpTestStatus->setText(tr("完成"));
        m_lblExpTestStatus->setStyleSheet("QLabel { color: green; font-weight: bold; }");
        m_btnStartExpTest->setEnabled(true);
        m_btnStopExpTest->setEnabled(false);
        if (m_btnAnalyzeExpTestDir) m_btnAnalyzeExpTestDir->setEnabled(true);
        if (m_spinSensorCalibRate) m_spinSensorCalibRate->setEnabled(
        !(m_checkSensorCalibAutoFieldRate && m_checkSensorCalibAutoFieldRate->isChecked()));
        m_spinExpStart->setEnabled(true);
        m_spinExpEnd->setEnabled(true);
        m_spinExpStep->setEnabled(true);
        m_spinStableDiscardFrames->setEnabled(true);
        m_spinVideoFramesPerExposure->setEnabled(true);
        m_spinProfileFramesPerExposure->setEnabled(true);
        m_editExpTestDir->setEnabled(true);
        m_comboLighting->setEnabled(true);
        m_editMaterial->setEnabled(true);
        m_spinTemperature->setEnabled(true);
        m_spinPosX->setEnabled(true);
        m_spinPosY->setEnabled(true);
        m_spinPosZ->setEnabled(true);
        m_spinPosC->setEnabled(true);
        m_spinPosB->setEnabled(true);
        m_btnReadPosition->setEnabled(true);
        m_checkSaveProfile->setEnabled(true);
        m_checkSaveVideo->setEnabled(true);

        // 曝光测试结束后恢复用户正式采集的测量区域（通常Small=7）。
        leaveSensorCalibrationMeasuringField();
        updateVideoControlState();

        int totalSteps = (m_expTestEnd - m_spinExpStart->value()) / m_expTestStep + 1;
        statusBar()->showMessage(tr("曝光测试完成! 共 %1 组数据保存至: %2").arg(totalSteps).arg(m_expTestDir));

        // 构建保存内容描述
        QString saveContent;
        if (m_expTestSaveProfile && m_expTestSaveVideo) {
            saveContent = tr("轮廓%1条/档 + Video%2张/档")
                .arg(m_expTestProfileFramesPerExposure).arg(m_expTestVideoFramesPerExposure);
        }
        else if (m_expTestSaveProfile) {
            saveContent = tr("仅轮廓%1条/档").arg(m_expTestProfileFramesPerExposure);
        }
        else if (m_expTestSaveVideo) {
            saveContent = tr("仅Video%1张/档").arg(m_expTestVideoFramesPerExposure);
        }

        QMessageBox::information(this, tr("测试完成"),
            tr("曝光测试完成!\n\n"
                "材料: %1\n"
                "光照: %2\n"
                "温度: %3°C\n"
                "机床位姿 XYZBC: (%4, %5, %6, %7°, %8°)\n"
                "坐标源: XYZ=光栅MPOS，C/B=机械MPOS\n"
                "测量区域: 使用选定测量场；目标轮廓频率=%9 Hz；Idle按Profile Rate+Exposure自动计算\n"
                "曝光范围: %10 ~ %11 (×10µs)\n"
                "共 %12 组数据\n"
                "稳定策略: 每次Video/Profile启动后丢弃%13个真实新帧\n"
                "保存内容: %14\n"
                "V27.6.43曝光分析: %15\n\n"
                "保存位置:\n%16")
            .arg(m_editMaterial->text())
            .arg(m_comboLighting->currentText())
            .arg(m_expTestTemperature)
            .arg(m_expTestPosX).arg(m_expTestPosY).arg(m_expTestPosZ)
            .arg(m_expTestPosC).arg(m_expTestPosB)
            .arg(sensorCalibrationRequestedRateHz())
            .arg(m_spinExpStart->value())
            .arg(m_expTestEnd)
            .arg(totalSteps)
            .arg(m_expTestStableDiscardFrames)
            .arg(saveContent)
            .arg(analysisSummary)
            .arg(m_expTestDir));
        break;
    }

    case ExpTestState::Idle:
    default:
        break;
    }
}

// ==================== G代码控制槽函数 ====================

void MainWindow::onGCodeDisableControls(bool disable)
{
    // 禁用/启用手动控制
    for (auto* widget : m_controlsToDisable) {
        if (widget) {
            widget->setEnabled(!disable);
        }
    }

    // 更新状态栏
    if (disable) {
        statusBar()->showMessage(tr("G代码执行中，手动控制已禁用"), 0);
    }
    else {
        statusBar()->showMessage(tr("G代码执行结束，手动控制已启用"), 3000);
    }
}

void MainWindow::onGCodeExecutionStateChanged(bool running)
{
    // 更新采集状态显示
    if (running) {
        m_lblCaptureStatus->setText(tr("G代码控制中"));
        m_lblCaptureStatus->setStyleSheet("QLabel { color: blue; font-weight: bold; }");
    }
    else {
        m_lblCaptureStatus->setText(tr("未采集"));
        m_lblCaptureStatus->setStyleSheet("QLabel { font-weight: bold; }");
    }
}

void MainWindow::onGCodeRequestSensorParams()
{
    if (!m_gcodeWidget) return;

    // 从界面参数设置区读取传感器参数
    SensorParams params;
    params.resolutionIndex = static_cast<unsigned int>(
        qMax(0, m_comboResolution->currentIndex()));
    params.exposureTime = static_cast<unsigned int>(m_spinExposure->value());
    params.idleTime = static_cast<unsigned int>(m_spinIdle->value());
    params.measuringFieldIndex = m_comboMeasuringField && m_comboMeasuringField->currentData().isValid()
        ? m_comboMeasuringField->currentData().toInt() : 7;
    m_gcodeWidget->setSensorParams(params);

    // Always forward the LIVE Parameters-page H1/H3/Hz selection. Formerly
    // this function hard-coded Software, while the preflight inspected stale
    // SyncManager defaults (often H1/200 even after the UI was H3/100).
    const bool manualHardware=m_radioHardwareSync && m_radioHardwareSync->isChecked();
    const SyncMode mode=manualHardware ? SyncMode::Hardware : SyncMode::Software;
    const int topology=m_comboHardwareTopology ? m_comboHardwareTopology->currentData().toInt() : 0;
    const int frequency=hardwareSyncFrequencyHz();
    const int edge=topology==1 ? 104 :
        (m_comboCmmR1Edge ? m_comboCmmR1Edge->currentData().toInt() : 0);
    m_gcodeWidget->setSyncMode(mode);
    m_gcodeWidget->setHardwareSyncUiIntent(topology,frequency,edge,manualHardware);

    LOG_INFO(LogCategory::GCode,
        tr("G代码参数页实时选择：H%1/%2Hz、R1=%3，手动采集模式=%4；仅UI意图，非物理硬件读回；曝光=%5us")
            .arg(topology).arg(frequency).arg(edge)
            .arg(mode==SyncMode::Hardware ? QStringLiteral("Hardware") : QStringLiteral("Software"))
            .arg(params.exposureTime));
}

void MainWindow::stageGCodeHardwareSettings(const GCodeProgram& nc)
{
    // The NC, not the UI, specifies capture wiring and trigger rate. No hardware command here.
    if(!m_syncManager || m_syncManager->isRunning() ||
       (m_gcodeExecutor && (m_gcodeExecutor->isRunning() || m_gcodeExecutor->isPaused()))) return;
    const int topology=nc.hardwareTopologyId, edge=(topology==1)?104:nc.cmmR1Edge;
    const int frequency=nc.hardwareFrequencyHz;
    if((topology!=1 && topology!=3) || (topology==3 && edge!=114 && edge!=115) ||
       frequency<mls::runtime::Table5D::minTriggerFrequencyHz ||
       frequency>mls::runtime::Table5D::maxTriggerFrequencyHz){
        LOG_WARNING(LogCategory::GCode,tr("NC中H1/H3、频率或H3 R1锁存沿非法；拒绝暂存"));
        return;
    }
    m_syncManager->setHardwareTopology(static_cast<mls::runtime::HardwareSyncTopology>(topology),edge);
    m_syncManager->setHardwareTriggerFrequencyHz(frequency);
    LOG_INFO(LogCategory::GCode,tr("按NC而不是UI暂存H%1/%2Hz/R1=%3（尚未触发硬件）")
        .arg(topology).arg(frequency).arg(edge));
}

void MainWindow::invalidateNcPreflightEvidence()
{
    // V30: restarting the all-in-one diagnosis must NEVER reuse a previous
    // probe/FCL PASS when the new attempt aborts or fails early.
    m_geometryNcProof.passed=false;
    m_geometryNcProof.independentPartRegistrationCertified=false;
    m_hardwareNcProof.passed=false;
    if(m_gcodeGeometryCancel)m_gcodeGeometryCancel->store(true);
    m_pendingGeometryNcDigest.clear();
    m_pendingGeometryCadIdentity.clear();
    m_pendingNcWorkpieceSafety.reset();
    m_ncWorkpieceSafety.reset();
    m_geometryCadIdentity.clear();
}

void MainWindow::onGCodeFullNcGeometryAuditCancelRequested()
{
    // V30 progress dialog cancellation requests an orderly worker abort.
    // No controller or motion command is issued by this cancellation.
    if(m_gcodeGeometryCancel)m_gcodeGeometryCancel->store(true);
}

void MainWindow::onGCodeFullNcGeometryAuditRequested(const QString& ncText)
{
    // Re-check the exact NC, including an immutable imported STEP/MCS placement.
    // No expensive virtual laser ray tracing is repeated here.
    if(m_gcodeOfflineGeometryAuditWatcher.isRunning()){
        if(m_gcodeWidget)m_gcodeWidget->showFullNcGeometryAuditResult(false,
            tr("另一个整条NC的FCL碰撞复核仍在运行，不能并发启动"));
        return;
    }
    m_geometryNcProof.passed=false;
    m_geometryNcProof.independentPartRegistrationCertified=false;
    m_pendingGeometryNcDigest.clear();
    m_pendingGeometryCadIdentity.clear();
    m_pendingNcWorkpieceSafety.reset();
    m_ncWorkpieceSafety.reset();
    m_geometryCadIdentity.clear();
    if(m_gcodeHardwareNcProbeActive || !m_machineSafety || !m_machineSafety->isReady()){
        if(m_gcodeWidget)m_gcodeWidget->showFullNcGeometryAuditResult(false,
            tr("硬件试采尚未结束，或Machine5X FCL碰撞模型未成功加载；不能跳过碰撞复核"));
        return;
    }
    std::array<double,17> active{};
    if(!m_syncManager || !m_syncManager->currentKinematicModel(&active)){
        if(m_gcodeWidget)m_gcodeWidget->showFullNcGeometryAuditResult(false,
            tr("当前生效的Physical17模型不可读；请检查启动自动标定恢复"));
        return;
    }
    for(const auto x:active)if(!std::isfinite(x)){
        if(m_gcodeWidget)m_gcodeWidget->showFullNcGeometryAuditResult(false,
            tr("当前应用的Physical17数值非有限值"));
        return;
    }
    const QByteArray digest=canonicalHardwareNcDigest(ncText);
    if(digest.isEmpty()){
        if(m_gcodeWidget)m_gcodeWidget->showFullNcGeometryAuditResult(false,
            tr("当前待复核NC内容无效，无法建立与原文件绑定的FCL证据"));
        return;
    }
    const bool compensationOn=m_zmc && m_zmc->isConnected() && m_zmc->axisCompensationEnabled();
    if(compensationOn){
        double envelopeMm=0.;
        QString envelopeError;
        if(!m_zmc->axisCompensationSafetyEnvelopeMm(envelopeMm,&envelopeError)){
            if(m_gcodeWidget)m_gcodeWidget->showFullNcGeometryAuditResult(false,
                tr("无法取得已启用的XYZ补偿安全包络：%1").arg(envelopeError));
            return;
        }
        const double clearanceMm=std::max(0.0,m_machineSafety->config().safety.minimumStaticClearanceMm);
        if(!(clearanceMm>envelopeMm+1.0e-6)){
            if(m_gcodeWidget)m_gcodeWidget->showFullNcGeometryAuditResult(false,
                tr("Machine5X FCL间隙%1mm不足覆盖XYZ补偿包络%2mm")
                    .arg(clearanceMm,0,'f',4).arg(envelopeMm,0,'f',4));
            return;
        }
    }

    // Import STEP ONCE in the multi-pose experiment (OCCT B-Rep triangulated
    // there). The full-NC worker receives a snapshot and never reads or changes
    // any live GUI pose while checking the motion path.
    mls::twin::TwinScene workpieceScene;
    QByteArray cadIdentity;
    bool previewPoseReviewed=false;
    QString stepError;
    if(!m_pathPlanningWidget ||
       !m_pathPlanningWidget->snapshotCollisionWorkpiece(
           workpieceScene,cadIdentity,previewPoseReviewed,&stepError)){
        if(m_gcodeWidget)m_gcodeWidget->showFullNcGeometryAuditResult(false,
            tr("缺少扫描工件STEP碰撞模型：%1").arg(stepError));
        return;
    }
    if(cadIdentity.isEmpty() || !workpieceScene.hasWorkpiece()){
        if(m_gcodeWidget)m_gcodeWidget->showFullNcGeometryAuditResult(false,
            tr("STEP模型或CAD→MCS位姿的身份校验失败"));
        return;
    }
    m_pendingGeometryNcDigest=digest;
    m_pendingGeometryNcModel=active;
    m_pendingGeometryConnectionGeneration=m_controllerSensorConnectionGeneration;
    m_pendingGeometryCompensationEnabled=compensationOn;
    m_pendingGeometryCadIdentity=cadIdentity;
    auto safety=std::make_shared<mls::safety::MachineSafetyModel>();
    auto scene=std::make_shared<const mls::twin::TwinScene>(std::move(workpieceScene));
    m_pendingNcWorkpieceSafety=safety;
    m_gcodeGeometryCancel=std::make_shared<std::atomic<bool>>(false);
    const auto cancel=m_gcodeGeometryCancel;
    const std::string program=ncText.toUtf8().toStdString();
    const std::filesystem::path modelConfig(m_machineModelConfigPath.toStdWString());
    auto work=QtConcurrent::run([this,program,safety,cancel,compensationOn,
        scene,modelConfig,previewPoseReviewed]() -> QString {
        const auto blocked=[](const QString& why){return QStringLiteral("BLOCK\n")+why;};
        const auto started=std::chrono::steady_clock::now();
        if(cancel->load())return blocked(QStringLiteral("已取消碰撞复核"));
        std::string loadError;
        if(!safety->load(modelConfig,&loadError))
            return blocked(QStringLiteral("无法创建本次独立机床FCL模型：")+QString::fromStdString(loadError));
        if(!safety->setWorkpiece(scene->mesh(),scene->cadToMachineEstimate(),&loadError))
            return blocked(QStringLiteral("无法装入STEP工件FCL网格：")+QString::fromStdString(loadError));
        if(cancel->load())return blocked(QStringLiteral("已取消碰撞复核"));
        const auto playback=mls::twin::simulateMultiPoseNc(program,200000);
        if(!playback.ok())
            return blocked(QStringLiteral("逐段NC解释失败：")+QString::fromStdString(playback.error));
        if(playback.vertices.size()<2 || playback.profiles.empty())
            return blocked(QStringLiteral("NC无有效运动轨迹或采集段"));
        const auto pose=[](const mls::twin::Joint5& q)->JointPose5D {
            return {q[0],q[1],q[2],q[3],q[4]};
        };
        std::size_t checked=0;
        for(std::size_t i=1;i<playback.vertices.size();++i){
            if(cancel->load())return blocked(QStringLiteral("用户取消整条NC FCL复核"));
            const auto c=safety->checkTransition(pose(playback.vertices[i-1].joints),
                pose(playback.vertices[i].joints),compensationOn,i>1);
            if(!c.modelReady || !c.safe)
                return blocked(QStringLiteral("FCL运动段 %1/%2（NC行%3）碰撞、限位或间隙失败：%4")
                    .arg(static_cast<qulonglong>(i))
                    .arg(static_cast<qulonglong>(playback.vertices.size()-1))
                    .arg(static_cast<qulonglong>(playback.vertices[i].sourceLine))
                    .arg(QString::fromStdString(c.message)));
            ++checked;
            if((checked & 255u)==0 || checked+1==playback.vertices.size()){
                const auto done=static_cast<quint64>(checked);
                const auto total=static_cast<quint64>(playback.vertices.size()-1);
                // Worker never touches QWidget; throttled GUI progress is queued.
                QMetaObject::invokeMethod(this,[this,done,total]() {
                    if(!m_closeRequested && m_gcodeWidget &&
                       m_gcodeOfflineGeometryAuditWatcher.isRunning())
                        m_gcodeWidget->showFullNcGeometryAuditProgress(done,total);
                },Qt::QueuedConnection);
            }
        }
        if(cancel->load())return blocked(QStringLiteral("用户取消整条NC FCL复核"));
        const auto elapsed=std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now()-started).count();
        // Geometry PASS is not a physical installation check. Stage 4 offers
        // a concrete operator-measurement and guarding acceptance workflow.
        QString report=QStringLiteral("PASS\n");
        report+=QStringLiteral("机床+STEP模型FCL扫掠检查 %1 个运动段（%2 ms），工件%3三角形，名义轮廓%4；"
                               "端点复用+AABB快速排除已启用。\n")
            .arg(static_cast<qulonglong>(checked))
            .arg(static_cast<qlonglong>(elapsed))
            .arg(static_cast<qulonglong>(scene->mesh().triangles.size()))
            .arg(static_cast<qulonglong>(playback.profiles.size()));
        report+=(previewPoseReviewed
            ? QStringLiteral("离线CAD位姿曾人工查看；")
            : QStringLiteral("离线CAD位姿尚未人工查看；"));
        report+=QStringLiteral("下一步将打开可完成的第4步：录入独立实测CAD→机床位姿，核对工件包络差，"
                               "记录夹具间隙、固定与现场安全防护。验收通过后允许申请实机启动，"
                               "正式下发前仍将重新FCL检查实时MPOS→首G00。未测量不得放行。");
        return report;
    });
    m_gcodeOfflineGeometryAuditWatcher.setFuture(work);
    statusBar()->showMessage(tr("正在执行包含多姿态导入STEP工件的Machine5X FCL碰撞复核"));
}

void MainWindow::onGCodeSiteExecutionAcceptanceRequested(const GCodeProgram& nc,
    const QByteArray& rawDiskDigest)
{
    const auto reject=[this](const QString& message) {
        LOG_WARNING(LogCategory::GCode,tr("第4步现场验收未通过：%1").arg(message));
        if(m_gcodeWidget)m_gcodeWidget->showSiteExecutionAcceptanceResult(false,message);
    };
    // A fresh attempt replaces, rather than silently reuses, any earlier
    // operator acceptance for this geometry. All previous hardware proofs stay
    // intact unless their own session or geometry changes.
    m_geometryNcProof.independentPartRegistrationCertified=false;
    if(!m_geometryNcProof.passed || !m_hardwareNcProof.passed ||
       !m_ncWorkpieceSafety || !m_ncWorkpieceSafety->isReady()){
        reject(tr("必须先通过第1~3步的真实试采和包含STEP的整条NC碰撞复核"));return;
    }
    if(m_geometryCadIdentity.isEmpty() || !m_pathPlanningWidget ||
       nc.filePath.isEmpty() || m_closeRequested ||
       (m_gcodeExecutor && (m_gcodeExecutor->isRunning() || m_gcodeExecutor->isPaused()))){
        reject(tr("工件/NC源文件不存在，或程序正在执行"));return;
    }
    QFile source(nc.filePath);
    if(!source.open(QIODevice::ReadOnly)){
        reject(tr("现场验收时无法读取NC磁盘原件"));return;
    }
    const QByteArray bytes=source.readAll();
    const QByteArray textDigest=canonicalHardwareNcDigest(QString::fromUtf8(bytes));
    if(QCryptographicHash::hash(bytes,QCryptographicHash::Sha256)!=rawDiskDigest ||
       textDigest!=m_geometryNcProof.normalizedNcDigest ||
       textDigest!=m_hardwareNcProof.normalizedNcDigest ||
       m_geometryNcProof.connectionGeneration!=m_controllerSensorConnectionGeneration ||
       m_hardwareNcProof.connectionGeneration!=m_controllerSensorConnectionGeneration){
        reject(tr("NC字节、试采、碰撞复核或控制器连接会话已改变；请重新做综合诊断"));return;
    }
    mls::twin::TwinScene scene;
    QByteArray identity;
    bool offlineReviewed=false;
    QString snapError;
    if(!m_pathPlanningWidget->snapshotCollisionWorkpiece(
           scene,identity,offlineReviewed,&snapError) || identity!=m_geometryCadIdentity){
        reject(tr("STEP或工件摆放在第三步后发生变化：%1").arg(snapError));return;
    }
    if(!m_zmc || !m_zmc->isConnected() || !m_laser || !m_laser->isConnected() ||
       !m_zmc->areAllAxesHomed()){
        reject(tr("现场验收要求当前控制器、传感器在线及五轴真实回零"));return;
    }
    QString motionError, idleError;
    if(!m_zmc->checkMotionHealth5D(&motionError) ||
       !m_zmc->areAllAxesIdle(&idleError)){
        reject(tr("机床有报警或正在运动：%1 %2").arg(motionError,idleError));return;
    }

    QDialog dialog(this);
    dialog.setWindowTitle(tr("第4/4步：实测工件定位与现场安全验收"));
    dialog.setMinimumWidth(640);
    auto* layout=new QVBoxLayout(&dialog);
    auto* instruction=new QLabel(tr(
        "本步骤可以真正完成；不是固定的false许可。请使用现场独立测量（量具、基准块、CMM或已验证的测量链）得到工件CAD→机床的六维位姿。"
        "所有数字均须人工依据实测记录填写，不得从下方离线CAD数值直接复制冒充实测。\n"
        "软件会比较整个STEP包围盒8个角点，最大位姿差必须小于碰撞间隙的一半（上限0.5mm）。"
        "还须核对未纳入STEP模型的夹具与整条运动扫掠包络、防护和急停。"
        "验收不会发出任何轴运动指令。"),&dialog);
    instruction->setWordWrap(true);
    layout->addWidget(instruction);
    const auto estimate=scene.estimatedPose();
    auto* expected=new QLabel(tr("第三步使用的 CAD→MCS：X=%1 Y=%2 Z=%3 mm，Rx=%4 Ry=%5 Rz=%6 °")
        .arg(estimate.x,0,'f',5).arg(estimate.y,0,'f',5).arg(estimate.z,0,'f',5)
        .arg(estimate.rxDeg,0,'f',5).arg(estimate.ryDeg,0,'f',5).arg(estimate.rzDeg,0,'f',5),&dialog);
    expected->setWordWrap(true);
    layout->addWidget(expected);
    auto* form=new QFormLayout;
    const std::array<QString,6> names{
        tr("独立实测 X (mm)"),tr("独立实测 Y (mm)"),tr("独立实测 Z (mm)"),
        tr("独立实测 Rx (°)"),tr("独立实测 Ry (°)"),tr("独立实测 Rz (°)")};
    std::array<QLineEdit*,6> poseFields{};
    for(std::size_t i=0;i<poseFields.size();++i){
        auto* editor=new QLineEdit(&dialog);
        auto* validator=new QDoubleValidator(-1.0e6,1.0e6,8,editor);
        validator->setNotation(QDoubleValidator::StandardNotation);
        editor->setValidator(validator);
        editor->setPlaceholderText(tr("必须输入现场独立测量值"));
        poseFields[i]=editor;
        form->addRow(names[i],editor);
    }
    auto* fixtureClearance=new QLineEdit(&dialog);
    fixtureClearance->setValidator(new QDoubleValidator(0.0,100000.0,4,fixtureClearance));
    fixtureClearance->setPlaceholderText(tr("全部未建模夹具与五轴扫掠区域的实测最小间隙，mm"));
    form->addRow(tr("实测夹具最小间隙 (mm)"),fixtureClearance);
    auto* operatorId=new QLineEdit(&dialog);
    operatorId->setPlaceholderText(tr("现场执行测量/验收人员编号或姓名"));
    form->addRow(tr("验收人员"),operatorId);
    auto* method=new QLineEdit(&dialog);
    method->setPlaceholderText(tr("记录独立测量的仪器、基准点和结果来源"));
    form->addRow(tr("测量方法/记录编号"),method);
    layout->addLayout(form);
    auto* independent=new QCheckBox(tr("六维位姿来自实际工件的独立测量，而非复制离线CAD初值"),&dialog);
    auto* secured=new QCheckBox(tr("已现场核对零件/夹具安装牢固、尺寸和实际装夹状态"),&dialog);
    auto* fixtureReviewed=new QCheckBox(tr("已逐段核对完整XYZCB扫掠包络与未建模夹具/台面/线缆的实物间隙"),&dialog);
    auto* guarding=new QCheckBox(tr("已核查人员隔离、限位、防护与急停，并准备低速首次空运行"),&dialog);
    for(auto* box:{independent,secured,fixtureReviewed,guarding}){
        box->setChecked(false);
        layout->addWidget(box);
    }
    auto* controls=new QDialogButtonBox(QDialogButtonBox::Ok|QDialogButtonBox::Cancel,&dialog);
    controls->button(QDialogButtonBox::Ok)->setText(tr("校验测量并记录本次现场验收"));
    layout->addWidget(controls);
    QString qualifiedSummary;
    connect(controls,&QDialogButtonBox::rejected,&dialog,&QDialog::reject);
    connect(controls,&QDialogButtonBox::accepted,&dialog,[&]() {
        std::array<double,6> values{};
        for(std::size_t i=0;i<values.size();++i){
            bool ok=false;
            const QString str=poseFields[i]->text().trimmed();
            values[i]=str.toDouble(&ok);
            if(!ok || str.isEmpty() || !std::isfinite(values[i])){
                QMessageBox::warning(&dialog,tr("现场验收缺少实测值"),tr("请逐项输入实际测得的6维CAD→机床位姿"));
                return;
            }
        }
        bool fixtureOk=false;
        const double clearance=fixtureClearance->text().toDouble(&fixtureOk);
        if(!fixtureOk || fixtureClearance->text().trimmed().isEmpty()){
            QMessageBox::warning(&dialog,tr("现场验收缺少夹具间隙"),tr("请填写最小实测夹具间隙"));return;
        }
        mls::safety::SiteAcceptanceInput data;
        data.independentlyMeasuredCadToMcs={values[0],values[1],values[2],
                                           values[3],values[4],values[5]};
        data.measuredMinimumFixtureClearanceMm=clearance;
        data.independentMeasurementConfirmed=independent->isChecked();
        data.partAndFixturesSecured=secured->isChecked();
        data.entireSweptEnvelopeInspected=fixtureReviewed->isChecked();
        data.guardingAndEStopVerified=guarding->isChecked();
        data.operatorIdentification=operatorId->text().trimmed().toStdString();
        data.measurementMethod=method->text().trimmed().toStdString();
        const auto verdict=mls::safety::evaluateSiteAcceptance(data,scene.mesh(),estimate,
            m_ncWorkpieceSafety->config().safety.minimumStaticClearanceMm);
        if(!verdict.accepted){
            QMessageBox::warning(&dialog,tr("实测验收不合格"),
                tr("数据或现场检查不满足条件：%1\n工件角点最大偏差=%2 mm，允许=%3 mm；夹具间隙要求≥%4 mm。\n"
                   "请重新测量、调整工件在多姿态扫描的CAD位姿并重新做第三步，或补齐现场检查。")
                    .arg(QString::fromStdString(verdict.reason))
                    .arg(verdict.worstCornerPositionErrorMm,0,'f',4)
                    .arg(verdict.allowedPositionErrorMm,0,'f',4)
                    .arg(verdict.minimumRequiredFixtureClearanceMm,0,'f',3));
            return;
        }
        // A physical setup change during the dialog always invalidates the
        // candidate, even if the old operator-entered measurements fit.
        mls::twin::TwinScene current;
        QByteArray currentIdentity;
        bool reviewed=false;
        QString error;
        if(!m_pathPlanningWidget->snapshotCollisionWorkpiece(
               current,currentIdentity,reviewed,&error) || currentIdentity!=identity ||
           !m_zmc->checkMotionHealth5D(&error) || !m_zmc->areAllAxesIdle(&error)){
            QMessageBox::warning(&dialog,tr("现场状态发生变化"),
                tr("请停止当前验收并重新做现场确认：%1").arg(error));return;
        }
        m_geometryNcProof.independentPartRegistrationCertified=true;
        m_geometryNcProof.siteAcceptedCadIdentity=identity;
        m_geometryNcProof.siteAcceptedNcDigest=textDigest;
        m_geometryNcProof.siteAcceptedConnectionGeneration=m_controllerSensorConnectionGeneration;
        m_geometryNcProof.siteAcceptedAt=std::chrono::steady_clock::now();
        m_geometryNcProof.acceptedBy=operatorId->text().trimmed();
        m_geometryNcProof.measurementMethod=method->text().trimmed();
        m_geometryNcProof.worstRegistrationErrorMm=verdict.worstCornerPositionErrorMm;
        m_geometryNcProof.measuredFixtureClearanceMm=clearance;
        QString startGateError;
        if(!m_gcodeExecutor || !m_gcodeExecutor->inspectSiteExecutionQualificationReadOnly(
               nc,rawDiskDigest,&startGateError)){
            m_geometryNcProof.independentPartRegistrationCertified=false;
            QMessageBox::warning(&dialog,tr("现场准入复核失败"),
                tr("测量合格，但控制器/同步或NC实时前提未通过：%1").arg(startGateError));return;
        }
        // Durable site evidence is part of approval, but a report file by
        // itself grants no privileges. Always re-read the physical controller
        // and verify the same STEP/NC/session at Start.
        QString directory=QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);
        if(directory.isEmpty())directory=QDir::tempPath()+QStringLiteral("/MotionLaserSync");
        directory+=QStringLiteral("/gcode_site_acceptance");
        QDir().mkpath(directory);
        const QString output=directory+QStringLiteral("/site_acceptance_")+
            QDateTime::currentDateTimeUtc().toString(QStringLiteral("yyyyMMdd_HHmmss_zzz"))+QStringLiteral(".json");
        QJsonObject record;
        record[QStringLiteral("schema")]=QStringLiteral("NC-Site-Measurements-v1");
        record[QStringLiteral("sourceNcSha256")]=QString::fromLatin1(rawDiskDigest.toHex());
        record[QStringLiteral("cadAndPlacementSha256")]=QString::fromLatin1(identity.toHex());
        record[QStringLiteral("operator")]=operatorId->text().trimmed();
        record[QStringLiteral("measurementMethod")]=method->text().trimmed();
        record[QStringLiteral("measuredFixtureClearanceMm")]=clearance;
        record[QStringLiteral("maxPoseDisagreementMm")]=verdict.worstCornerPositionErrorMm;
        record[QStringLiteral("maxAllowedPoseDisagreementMm")]=verdict.allowedPositionErrorMm;
        record[QStringLiteral("timestampUtc")]=QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs);
        QJsonArray poseArray;
        for(double value:values)poseArray.append(value);
        record[QStringLiteral("measuredCadToMcsXYZRxRyRz")]=poseArray;
        record[QStringLiteral("fixtureAndGuardingAttested")]=true;
        QSaveFile reportFile(output);
        if(!reportFile.open(QIODevice::WriteOnly) ||
           reportFile.write(QJsonDocument(record).toJson(QJsonDocument::Indented))<0 ||
           !reportFile.commit()){
            m_geometryNcProof.independentPartRegistrationCertified=false;
            QMessageBox::warning(&dialog,tr("无法保存现场验收记录"),
                tr("记录文件保存失败，未提供实机执行许可：%1").arg(output));return;
        }
        qualifiedSummary=tr("独立实测工件最大位姿差=%1 mm（上限%2 mm）；实测夹具间隙=%3 mm；验收人员=%4；"
                            "保存记录=%5。允许申请开始执行；启动时再次检查NC/STEP/设备状态以及实时MPOS→首G00路径FCL。")
            .arg(verdict.worstCornerPositionErrorMm,0,'f',4)
            .arg(verdict.allowedPositionErrorMm,0,'f',4)
            .arg(clearance,0,'f',3).arg(operatorId->text().trimmed()).arg(output);
        LOG_INFO(LogCategory::GCode,tr("NC现场测量验收通过：%1").arg(qualifiedSummary));
        dialog.accept();
    });
    if(dialog.exec()!=QDialog::Accepted){
        m_geometryNcProof.independentPartRegistrationCertified=false;
        reject(tr("现场测量或夹具/防护验收被取消；第1~3步结果仍在，可点击‘第4步：现场测量验收’补做"));
        return;
    }
    if(m_gcodeWidget)m_gcodeWidget->showSiteExecutionAcceptanceResult(true,qualifiedSummary);
}

void MainWindow::onGCodeHardwareNcProbeRequested(const GCodeProgram& nc,const QString& originalNcText)
{
    m_hardwareNcProof.passed=false; // a new or failed probe cannot reuse old PASS
    m_probeNcDigest.clear();
    const auto fail=[this](const QString& why) {
        if(m_gcodeWidget)m_gcodeWidget->showHardwareNcProbeResult(false,why,QString());
        LOG_WARNING(LogCategory::GCode,tr("NC硬件试采拒绝：%1").arg(why));
    };
    if(m_closeRequested || m_gcodeHardwareNcProbeActive || !m_syncManager ||
       m_syncManager->isRunning() || m_hardwareTemporalScanActive ||
       m_jointMoveInProgress || m_sphereStudyAutoActive ||
       (m_gcodeExecutor && (m_gcodeExecutor->isRunning() || m_gcodeExecutor->isPaused())) ||
       (m_laser && (m_laser->isVideoRunning() || m_profilePreviewRunning))){
        fail(tr("已有采集/视频预览/运动/标定任务，必须先停止；本次绝不抢占正在运行的同步会话"));return;
    }
    if(!m_zmc || !m_zmc->isConnected() || !m_laser || !m_laser->isConnected()){
        fail(tr("控制器和线激光传感器未同时连接"));return;
    }
    if(nc.syncMode!=SyncMode::Hardware || (nc.hardwareTopologyId!=1 && nc.hardwareTopologyId!=3) ||
       (nc.hardwareTopologyId==3 && nc.cmmR1Edge!=114 && nc.cmmR1Edge!=115) ||
       nc.hardwareFrequencyHz<mls::runtime::Table5D::minTriggerFrequencyHz ||
       nc.hardwareFrequencyHz>mls::runtime::Table5D::maxTriggerFrequencyHz ||
       !mls::gcode::validateSensorTriggerMetadata(nc)){
        fail(tr("NC不满足H1/H3、频率、H3锁存沿或传感器触发源合同"));return;
    }
    if(!m_zmc->areAllAxesHomed() || !m_syncManager->hasKinematicModel()){
        fail(tr("五轴本会话未全部完成回零，或本机尚未实际应用17参数运动学标定"));return;
    }
    QString health,idle;
    if(!m_zmc->checkMotionHealth5D(&health) || !m_zmc->areAllAxesIdle(&idle)){
        fail(tr("不能在运动或五轴故障期间开启静态硬件同步试采：%1 %2").arg(health,idle));return;
    }
    QString actualTable, tableError; bool authorized=false;
    if(!m_zmc->hwTableCommissioningPreflight(actualTable,&tableError,&authorized) || !authorized){
        fail(tr("实际控制器TABLE无法完成授权/健康检查：%1；%2").arg(tableError,actualTable));return;
    }
    // Snapshot in-memory state. This does not imply hardware readback.
    m_gcodeProbePreviousSyncMode=m_syncManager->syncMode();
    m_gcodeProbePreviousTopology=static_cast<int>(m_syncManager->hardwareTopology());
    m_gcodeProbePreviousHz=m_syncManager->hardwareTriggerFrequencyHz();
    m_gcodeProbePreviousEdge=m_syncManager->currentCmmR1Edge();
    m_gcodeProbePreviousTriggerExplicit=m_syncManager->hasExplicitHardwareTriggerType();
    // Do not clear accumulated cloud/UI data from other experiments.
    stageGCodeHardwareSettings(nc);
    const auto previousAligned=m_syncManager->hardwareAlignedDataEmissionEnabled();
    const auto previousRaw=m_syncManager->hardwareRawProfileCaptureEnabled();
    m_gcodeProbePreviousAlignedEmission=previousAligned;
    m_gcodeProbePreviousRawCapture=previousRaw;
    m_syncManager->setHardwareAlignedDataEmissionEnabled(false);
    m_syncManager->setHardwareRawProfileCaptureEnabled(false);
    m_syncManager->setSyncMode(SyncMode::Hardware);
    if(nc.hardwareTopologyId==1)m_syncManager->setTriggerType(HardwareTriggerType::EdgeRising);
    else m_syncManager->clearHardwareTriggerTypeConfiguration();
    SensorParams params;
    params.resolutionIndex=static_cast<unsigned int>(qMax(0,m_comboResolution->currentIndex()));
    params.exposureTime=static_cast<unsigned int>(m_spinExposure->value());
    params.idleTime=static_cast<unsigned int>(m_spinIdle->value());
    params.measuringFieldIndex=m_comboMeasuringField && m_comboMeasuringField->currentData().isValid()
        ? m_comboMeasuringField->currentData().toInt():7;
    m_syncManager->setSensorParams(params);
    m_probeNcSensorParams=params;
    m_probeNcDigest=canonicalHardwareNcDigest(originalNcText);
    if(!m_syncManager->currentKinematicModel(&m_probeNcModel)){
        fail(tr("当前已应用Physical17模型无法提供实时数值"));
        m_syncManager->setHardwareAlignedDataEmissionEnabled(previousAligned);
        m_syncManager->setHardwareRawProfileCaptureEnabled(previousRaw);
        m_syncManager->setSyncMode(m_gcodeProbePreviousSyncMode);
        m_syncManager->setHardwareTopology(static_cast<mls::runtime::HardwareSyncTopology>(m_gcodeProbePreviousTopology),m_gcodeProbePreviousEdge);
        m_syncManager->setHardwareTriggerFrequencyHz(m_gcodeProbePreviousHz);
        if(m_gcodeProbePreviousTriggerExplicit)m_syncManager->setTriggerType(HardwareTriggerType::EdgeRising);
        else m_syncManager->clearHardwareTriggerTypeConfiguration();
        return;
    }
    m_gcodeHardwareNcProbeError.clear();
    m_gcodeHardwareNcProbeTopology=nc.hardwareTopologyId;
    m_gcodeHardwareNcProbeFrequency=nc.hardwareFrequencyHz;
    m_gcodeHardwareNcProbeEdge=nc.hardwareTopologyId==1?104:nc.cmmR1Edge;
    GlobalTimeSync::instance().start();
    QString error;
    if(!m_syncManager->start(&error)){
        m_syncManager->setHardwareAlignedDataEmissionEnabled(previousAligned);
        m_syncManager->setHardwareRawProfileCaptureEnabled(previousRaw);
        m_syncManager->setSyncMode(m_gcodeProbePreviousSyncMode);
        m_syncManager->setHardwareTopology(static_cast<mls::runtime::HardwareSyncTopology>(m_gcodeProbePreviousTopology),m_gcodeProbePreviousEdge);
        m_syncManager->setHardwareTriggerFrequencyHz(m_gcodeProbePreviousHz);
        if(m_gcodeProbePreviousTriggerExplicit)m_syncManager->setTriggerType(HardwareTriggerType::EdgeRising);
        else m_syncManager->clearHardwareTriggerTypeConfiguration();
        fail(tr("硬件同步启动被实际控制器/传感器拒绝：%1").arg(error));return;
    }
    m_gcodeHardwareNcProbeActive=true;
    m_lastCaptureMode=SyncMode::Hardware;
    const quint64 sequence=++m_gcodeHardwareNcProbeSequence;
    LOG_INFO(LogCategory::GCode,tr("按NC启动真实H%1/%2Hz/R1=%3硬件同步试采：不调用任何G00/G05运动")
        .arg(nc.hardwareTopologyId).arg(nc.hardwareFrequencyHz).arg(m_gcodeHardwareNcProbeEdge));
    updateVideoControlState();
    QTimer::singleShot(3000,this,[this,sequence]() {
        if(m_gcodeHardwareNcProbeActive && sequence==m_gcodeHardwareNcProbeSequence)
            finishGCodeHardwareNcProbe(false);
    });
}

void MainWindow::onGCodeHardwareNcProbeStopRequested()
{
    if(m_gcodeHardwareNcProbeActive)finishGCodeHardwareNcProbe(true);
}

void MainWindow::finishGCodeHardwareNcProbe(bool explicitStop)
{
    if(!m_gcodeHardwareNcProbeActive || m_gcodeHardwareNcProbeStopping)return;
    m_gcodeHardwareNcProbeStopping=true;
    ++m_gcodeHardwareNcProbeSequence;
    // Stop first, then evaluate the fully drained hardware TABLE/FIFO session.
    m_syncManager->stop();
    m_gcodeHardwareNcProbeActive=false;
    m_gcodeHardwareNcProbeStopping=false;
    const auto stats=m_syncManager->getHardwareAlignStats();
    const auto cb=m_laser->profileCallbackDiagnostics();
    const auto hw=m_laser->hardwareTriggerDiagnostics();
    mls::gcode::probe::Evidence e;
    e.stopCompleted=!m_syncManager->isRunning();
    e.synchronized=stats.syncEstablished;
    e.alignmentFault=stats.alignmentFault;
    e.publishedTriggerCountValid=stats.zmcTriggerCountValid;
    e.topologyId=m_gcodeHardwareNcProbeTopology;
    e.sensorTriggerReadbackValid=hw.readbackValid;
    e.sensorTriggerConfigMatches=mls::sensor::trigger::sensorTriggerMatches(
        e.topologyId,hw.triggerRequested,hw.triggerReadback);
    if(e.topologyId==1)
        e.sensorTriggerConfigMatches=e.sensorTriggerConfigMatches &&
            mls::sensor::trigger::digitalIoMatches(1,hw.digitalIoRequested,hw.digitalIoReadback);
    e.sensorCmmPortReadbackValid=e.topologyId==3 && hw.cmmPortReadbackValid &&
        mls::sensor::trigger::digitalIoMatches(3,hw.digitalIoRequested,hw.digitalIoReadback);
    e.poses=stats.totalPoseSamples;
    e.profiles=stats.totalProfiles;
    e.publishedTriggers=stats.zmcTriggerCount;
    e.aligned=stats.alignedProfiles;
    e.validProfiles=stats.validAlignedProfiles;
    e.emptyProfiles=stats.emptyAlignedProfiles;
    e.boundarySkipped=stats.temporalBoundarySkippedProfiles;
    e.pending=stats.pendingProfiles;e.dropped=stats.droppedProfiles;
    e.overflows=stats.tableOverflows;
    e.missing=stats.missingProfileIndices;e.duplicates=stats.duplicateProfiles;
    e.outOfOrder=stats.outOfOrderProfiles;e.invalidIndices=stats.invalidProfileIndices;
    e.tableReadErrors=stats.tableReadErrors;
    e.rawCallbacks=cb.rawCallbacks;e.conversionFailures=cb.conversionFailures;
    e.ingressDrops=cb.ingressDrops;
    e.configuredHz=m_gcodeHardwareNcProbeFrequency;
    // Use actual sensor exposure timestamps, not PC-estimated TABLE timestamps.
    // The first Probe->continuous interval is a handshake and not a rate sample.
    std::vector<double> periods;
    const auto frames=m_syncManager->getHardwareProfileRecords();
    for(std::size_t i=2;i<frames.size();++i){
        const auto& a=frames[i-1];const auto& b=frames[i];
        if(b.index!=a.index+1 || !std::isfinite(a.shutterOpenSec) ||
           !std::isfinite(b.shutterOpenSec) || a.shutterOpenSec<=0.0)continue;
        const double dt=(b.shutterOpenSec-a.shutterOpenSec)*1000.0;
        if(dt>0 && std::isfinite(dt))periods.push_back(dt);
    }
    if(!periods.empty()){
        std::sort(periods.begin(),periods.end());
        e.measuredMedianPeriodMs=periods[periods.size()/2];
        e.measuredSteadyIntervals=static_cast<std::int64_t>(periods.size());
    }
    auto verdict=mls::gcode::probe::assess(e);
    if(!m_gcodeHardwareNcProbeError.isEmpty()){
        verdict.sessionValid=false;
        verdict.failures.emplace_back("hardware/sensor error signal during capture");
    }
    if(explicitStop){
        verdict.sessionValid=false;
        verdict.failures.emplace_back("probe aborted or stopped before scheduled acquisition window");
    }
    QString reason;
    for(const auto& failure:verdict.failures)reason+=QString::fromStdString(failure)+QLatin1Char('\n');
    QString report=tr("NC配置 H%1/%2Hz/R1=%3\n实际轮廓=%4 锁存位姿=%5 发布计数=%6 对齐=%7\n"
        "完整停机=%8 同步建立=%9 传感器触发模式读回=%10 错误信号=%11\n"
        "传感器实测连续期中位间隔=%14ms、有效间隔数=%15（非PC估计时间）\n"
        "验证详情：\n%12\n\n原控制界面分析：\n%13")
        .arg(m_gcodeHardwareNcProbeTopology).arg(m_gcodeHardwareNcProbeFrequency)
        .arg(m_gcodeHardwareNcProbeEdge).arg(e.profiles).arg(e.poses)
        .arg(e.publishedTriggers).arg(e.aligned).arg(e.stopCompleted).arg(e.synchronized)
        .arg(e.sensorTriggerReadbackValid&&e.sensorTriggerConfigMatches&&
             (e.topologyId!=3 || e.sensorCmmPortReadbackValid))
        .arg(m_gcodeHardwareNcProbeError).arg(reason).arg(analyzeHardwareSyncData())
        .arg(e.measuredMedianPeriodMs,0,'f',4).arg(e.measuredSteadyIntervals);
    QString base=QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);
    if(base.isEmpty())base=QDir::tempPath()+QStringLiteral("/MotionLaserSync");
    QDir().mkpath(base+QStringLiteral("/gcode_preflight"));
    const QString reportPath=base+QStringLiteral("/gcode_preflight/hardware_nc_probe_")+
        QDateTime::currentDateTimeUtc().toString(QStringLiteral("yyyyMMdd_HHmmss_zzz"))+QStringLiteral(".json");
    QJsonObject obj;
    obj.insert(QStringLiteral("schema"),QStringLiteral("V30-HardwareNcProbe-v1"));
    obj.insert(QStringLiteral("hardwareTest"),true);
    obj.insert(QStringLiteral("axisMotionRequested"),false);
    obj.insert(QStringLiteral("runAuthorized"),false);
    obj.insert(QStringLiteral("sessionValid"),verdict.sessionValid);
    obj.insert(QStringLiteral("topology"),m_gcodeHardwareNcProbeTopology);
    obj.insert(QStringLiteral("frequencyHz"),m_gcodeHardwareNcProbeFrequency);
    obj.insert(QStringLiteral("r1Edge"),m_gcodeHardwareNcProbeEdge);
    obj.insert(QStringLiteral("profiles"),double(e.profiles));
    obj.insert(QStringLiteral("poses"),double(e.poses));
    obj.insert(QStringLiteral("publishedLatches"),double(e.publishedTriggers));
    obj.insert(QStringLiteral("aligned"),double(e.aligned));
    obj.insert(QStringLiteral("sensorMedianPeriodMs"),e.measuredMedianPeriodMs);
    obj.insert(QStringLiteral("measuredSteadyIntervals"),double(e.measuredSteadyIntervals));
    obj.insert(QStringLiteral("sensorTriggerMode"),e.topologyId==3?QStringLiteral("SoftwareInternal"):QStringLiteral("HardwareExternal"));
    obj.insert(QStringLiteral("sensorTriggerReadbackValid"),e.sensorTriggerReadbackValid);
    obj.insert(QStringLiteral("sensorTriggerConfigMatches"),e.sensorTriggerConfigMatches);
    obj.insert(QStringLiteral("h3CmmPortReadbackValid"),e.sensorCmmPortReadbackValid);
    obj.insert(QStringLiteral("sensorReadbackVerified"),e.sensorTriggerReadbackValid&&e.sensorTriggerConfigMatches&&
        (e.topologyId!=3 || e.sensorCmmPortReadbackValid));
    QJsonArray failures;for(const auto& x:verdict.failures)failures.append(QString::fromStdString(x));
    obj.insert(QStringLiteral("failures"),failures);
    obj.insert(QStringLiteral("analysis"),report);
    QSaveFile file(reportPath);
    bool reportSaved=false;
    if(file.open(QIODevice::WriteOnly)){
        file.write(QJsonDocument(obj).toJson(QJsonDocument::Indented));
        reportSaved=file.commit();
        if(!reportSaved)report+=tr("\n警告：试采报告未成功保存，不能使用该文件作为验收证据");
    }else report+=tr("\n警告：试采JSON报告创建失败");
    m_hardwareNcProof.passed=verdict.sessionValid && reportSaved && !m_probeNcDigest.isEmpty();
    if(m_hardwareNcProof.passed){
        m_hardwareNcProof.normalizedNcDigest=m_probeNcDigest;
        m_hardwareNcProof.model=m_probeNcModel;
        m_hardwareNcProof.connectionGeneration=m_controllerSensorConnectionGeneration;
        m_hardwareNcProof.topology=m_gcodeHardwareNcProbeTopology;
        m_hardwareNcProof.frequency=m_gcodeHardwareNcProbeFrequency;
        m_hardwareNcProof.edge=m_gcodeHardwareNcProbeEdge;
        m_hardwareNcProof.sensorParams=m_probeNcSensorParams;
        m_hardwareNcProof.finishedAt=std::chrono::steady_clock::now();
    }
    m_syncManager->setHardwareAlignedDataEmissionEnabled(m_gcodeProbePreviousAlignedEmission);
    m_syncManager->setHardwareRawProfileCaptureEnabled(m_gcodeProbePreviousRawCapture);
    m_syncManager->setSyncMode(m_gcodeProbePreviousSyncMode);
    m_syncManager->setHardwareTopology(static_cast<mls::runtime::HardwareSyncTopology>(m_gcodeProbePreviousTopology),m_gcodeProbePreviousEdge);
    m_syncManager->setHardwareTriggerFrequencyHz(m_gcodeProbePreviousHz);
    if(m_gcodeProbePreviousTriggerExplicit)m_syncManager->setTriggerType(HardwareTriggerType::EdgeRising);
    else m_syncManager->clearHardwareTriggerTypeConfiguration();
    m_gcodeHardwareNcProbeError.clear();
    updateVideoControlState();
    if(!m_closeRequested && m_gcodeWidget)m_gcodeWidget->showHardwareNcProbeResult(verdict.sessionValid,report,reportPath);
}

