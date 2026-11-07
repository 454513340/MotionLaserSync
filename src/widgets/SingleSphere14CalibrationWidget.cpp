#include "widgets/SingleSphere14CalibrationWidget.h"

#include "core/ZmcController.h"
#include "core/LaserSensor.h"
#include "core/PointCloudBuilder.h"
#include "trajectory/JointTrajectoryPlanner5Axis.h"
#include "calibration/SingleSphere17QtAdapter.h"
#include "calibration/SphereDistanceAngleStudy.h"
#include "calibration/SphereTwoStageFit.h"
#include "utils/PointCloudLoader.h"
#include "runtime/CalibrationPersistence.h"
#include "runtime/Acquisition5D.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QLabel>
#include <QPushButton>
#include <QToolButton>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QComboBox>
#include <QCheckBox>
#include <QTableWidget>
#include <QAbstractItemView>
#include <QHeaderView>
#include <QMessageBox>
#include <QFileDialog>
#include <QFile>
#include <QSaveFile>
#include <QStandardPaths>
#include <QDir>
#include <QFileInfo>
#include <QCoreApplication>
#include <QTextStream>
#include <QDateTime>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QCryptographicHash>
#include <QTimer>
#include <QFutureWatcher>
#include <QtConcurrent>
#include <QSignalBlocker>
#include <QPointF>
#include <QProgressBar>
#include <QPlainTextEdit>
#include <QMetaObject>
#include <QSet>
#include <QMap>
#include <QStringList>

#include <fstream>
#include <sstream>
#include <cmath>
#include <algorithm>
#include <memory>
#include <utility>
#include <atomic>
#include <exception>
#include <limits>

using namespace mls::calibration;

namespace {

double poseValue(const JointPose5D& q, int axis)
{
    switch (axis) {
    case AxisX: return q.x;
    case AxisY: return q.y;
    case AxisZ: return q.z;
    case AxisC: return q.c;
    case AxisB: return q.b;
    default: return 0.0;
    }
}

template <typename PoseT>
QString poseTextImpl(const PoseT& q)
{
    return QStringLiteral("X=%1  Y=%2  Z=%3  C=%4°  B=%5°")
        .arg(q.x, 0, 'f', 4).arg(q.y, 0, 'f', 4).arg(q.z, 0, 'f', 4)
        .arg(q.c, 0, 'f', 4).arg(q.b, 0, 'f', 4);
}

QString poseText(const JointPose5D& q)
{
    return poseTextImpl(q);
}

QString poseText(const mls::calibration::JointPose5d& q)
{
    return poseTextImpl(q);
}

// The portable calibration planner intentionally uses mls::calibration::JointPose5d,
// while the controller/UI signal uses the application's JointPose5D.  These are
// different C++ types even though they currently contain the same five fields, so
// an explicit boundary conversion is required (MSVC correctly rejects binding the
// portable object to const JointPose5D&).
JointPose5D controllerPoseFromPortable(const mls::calibration::JointPose5d& q)
{
    JointPose5D out;
    out.x = q.x;
    out.y = q.y;
    out.z = q.z;
    out.c = q.c;
    out.b = q.b;
    return out;
}

QString centerText(const Vec3d& c)
{
    return QStringLiteral("[%1, %2, %3] mm")
        .arg(c.x, 0, 'f', 4).arg(c.y, 0, 'f', 4).arg(c.z, 0, 'f', 4);
}

struct Stage2TrimmedInliers
{
    std::vector<std::size_t> indices;
    double residualP95Mm = std::numeric_limits<double>::infinity();
    double residualRmsMm = std::numeric_limits<double>::infinity();
};

struct Stage2SolveTaskResult
{
    NominalHandEyeSphereResult bootstrap;
    bool bootstrapAccepted = false;
    std::vector<Physical17PoseScreeningReport> screeningReports;
    SingleSphere17Result result;
};

bool sameStage2Pose(const JointPose5d& a, const JointPose5d& b,
                    double linearToleranceMm = 0.03,
                    double angularToleranceDeg = 0.03)
{
    return std::abs(a.x-b.x) <= linearToleranceMm &&
           std::abs(a.y-b.y) <= linearToleranceMm &&
           std::abs(a.z-b.z) <= linearToleranceMm &&
           std::abs(a.c-b.c) <= angularToleranceDeg &&
           std::abs(a.b-b.b) <= angularToleranceDeg;
}

std::vector<SphereProfileObservation> retainScreenedStage2Poses(
    const std::vector<SphereProfileObservation>& observations,
    const Physical17PoseScreeningReport& report,
    double linearToleranceMm = 0.03,
    double angularToleranceDeg = 0.03)
{
    std::vector<SphereProfileObservation> kept;
    kept.reserve(observations.size());
    for (const auto& observation : observations) {
        bool rejected = false;
        for (const auto& pose : report.poses) {
            if (!pose.rejected) continue;
            if (sameStage2Pose(observation.joints, pose.pose,
                               linearToleranceMm, angularToleranceDeg)) {
                rejected = true;
                break;
            }
        }
        if (!rejected) kept.push_back(observation);
    }
    return kept;
}

Physical17PoseScreeningReport limitRejectedPoseCount(
    Physical17PoseScreeningReport report, std::size_t keepRejectedCount)
{
    std::vector<std::size_t> rejected;
    for (std::size_t i=0; i<report.poses.size(); ++i)
        if (report.poses[i].rejected) rejected.push_back(i);
    std::sort(rejected.begin(), rejected.end(), [&](std::size_t a, std::size_t b) {
        return report.poses[a].rejectionScore > report.poses[b].rejectionScore;
    });
    keepRejectedCount = std::min(keepRejectedCount, rejected.size());
    for (auto& pose : report.poses) pose.rejected = false;
    for (std::size_t i=0; i<keepRejectedCount; ++i)
        report.poses[rejected[i]].rejected = true;
    report.rejectedPoseCount = keepRejectedCount;
    report.retainedPoseCount = report.inputPoseCount >= keepRejectedCount
        ? report.inputPoseCount-keepRejectedCount : 0;
    return report;
}

Stage2TrimmedInliers trimArcEdgeInliers(
    const std::vector<SensorProfilePoint>& points,
    const SphereCenterFit& fit,
    const std::vector<std::size_t>& ransacInliers,
    double trimFraction)
{
    Stage2TrimmedInliers out;
    if (!fit.success || ransacInliers.empty()) return out;
    trimFraction = std::clamp(trimFraction, 0.0, 0.25);

    struct A { double angle = 0.0; std::size_t index = 0; };
    std::vector<A> angles;
    angles.reserve(ransacInliers.size());
    constexpr double twoPi = 6.283185307179586476925286766559;
    for (std::size_t idx : ransacInliers) {
        if (idx >= points.size()) continue;
        double a = std::atan2(points[idx].z - fit.centerSensorMm.z,
                              points[idx].x - fit.centerSensorMm.x);
        if (a < 0.0) a += twoPi;
        angles.push_back({a, idx});
    }
    if (angles.empty()) return out;
    std::sort(angles.begin(), angles.end(), [](const A& a, const A& b){ return a.angle < b.angle; });

    // The measured sphere is an arc, not a full circle.  Locate the largest
    // angular gap and unwrap the complementary measured arc monotonically.
    std::size_t largestGapAfter = 0;
    double largestGap = -1.0;
    for (std::size_t i=0; i<angles.size(); ++i) {
        const std::size_t j=(i+1)%angles.size();
        const double next = angles[j].angle + (j==0 ? twoPi : 0.0);
        const double gap = next - angles[i].angle;
        if (gap > largestGap) { largestGap = gap; largestGapAfter = i; }
    }
    const std::size_t start = (largestGapAfter + 1) % angles.size();
    std::vector<A> unwrapped;
    unwrapped.reserve(angles.size());
    double previous = angles[start].angle;
    for (std::size_t k=0; k<angles.size(); ++k) {
        const std::size_t pos=(start+k)%angles.size();
        double a=angles[pos].angle;
        if (k>0) {
            while (a < previous) a += twoPi;
        }
        unwrapped.push_back({a, angles[pos].index});
        previous=a;
    }

    const double arcStart=unwrapped.front().angle;
    const double arcEnd=unwrapped.back().angle;
    const double span=std::max(0.0,arcEnd-arcStart);
    const double trim=span*trimFraction;
    const double keepLo=arcStart+trim;
    const double keepHi=arcEnd-trim;
    for(const auto& a:unwrapped) {
        if (a.angle + 1e-12 < keepLo || a.angle - 1e-12 > keepHi) continue;
        out.indices.push_back(a.index);
    }
    if (out.indices.empty()) return out;

    std::vector<double> absResiduals;
    absResiduals.reserve(out.indices.size());
    double sq=0.0;
    for(std::size_t idx:out.indices) {
        const double dx=points[idx].x-fit.centerSensorMm.x;
        const double dz=points[idx].z-fit.centerSensorMm.z;
        const double residual=std::hypot(dx,dz)-fit.sectionRadiusMm;
        sq += residual*residual;
        absResiduals.push_back(std::abs(residual));
    }
    out.residualRmsMm=std::sqrt(sq/static_cast<double>(out.indices.size()));
    std::sort(absResiduals.begin(),absResiduals.end());
    const double p=0.95*static_cast<double>(absResiduals.size()-1);
    const std::size_t lo=static_cast<std::size_t>(std::floor(p));
    const std::size_t hi=static_cast<std::size_t>(std::ceil(p));
    const double t=p-static_cast<double>(lo);
    out.residualP95Mm=absResiduals[lo]*(1.0-t)+absResiduals[hi]*t;
    return out;
}

constexpr int feedbackBitForAxis(int axisId)
{
    // AxisId is controller-facing 1..5, while every persisted/runtime XYZCB mask
    // uses compact bits 0..4.  Never shift directly by AxisId.
    return axisId - AxisX;
}

bool normalizePersistedFeedbackMask(int rawMask, bool legacyOneBasedEncoding,
                                    std::uint8_t& normalized, bool& migratedLegacyOneBased)
{
    using Encoding = mls::runtime::FeedbackMaskEncoding;
    const auto decoded = mls::runtime::decodeFeedbackMask(
        rawMask, legacyOneBasedEncoding ? Encoding::LegacyControllerAxisOneBased
                                        : Encoding::CompactXYZCB);
    if (!decoded.ok) return false;
    normalized = decoded.normalized;
    migratedLegacyOneBased = decoded.migratedLegacyOneBased;
    return true;
}

QString sourceMaskText(std::uint8_t mask)
{
    QStringList parts;
    const char* names[5] = {"X", "Y", "Z", "C", "B"};
    for (int axis = 0; axis < 5; ++axis) {
        parts << QStringLiteral("%1=%2")
                    .arg(QString::fromLatin1(names[axis]))
                    .arg((mask & (1u << axis)) ? QStringLiteral("MPOS")
                                               : QStringLiteral("DPOS"));
    }
    return parts.join(QStringLiteral("  "));
}

bool legacyXyzDposModelCanSeedFullMpos(std::uint8_t savedMask, std::uint8_t currentMask)
{
    // V27.6.16R and earlier Physical17 results used XYZ DPOS + C/B MPOS (24).
    // With incremental XYZ gratings homed into the same machine frame, those results
    // remain useful as an approximate runtime model and, especially, as a sphere-center
    // / path-planning seed. Do not silently call them equivalent: bind them in
    // compatibility mode and recommend a new MPOS calibration.
    return savedMask == 24u && currentMask == 31u;
}

bool finiteJsonNumber(const QJsonObject& object, const QString& key, double& value)
{
    const QJsonValue v=object.value(key);
    if(!v.isDouble()) return false;
    value=v.toDouble();
    return std::isfinite(value);
}

bool finiteJsonInteger(const QJsonObject& object, const QString& key,
                       double minimum, double maximum, double& value)
{
    if(!finiteJsonNumber(object,key,value)) return false;
    return value>=minimum && value<=maximum && std::floor(value)==value;
}

bool finiteJsonVec3(const QJsonValue& value, Vec3d& out)
{
    if(!value.isArray()) return false;
    const QJsonArray a=value.toArray();
    if(a.size()!=3 || !a[0].isDouble() || !a[1].isDouble() || !a[2].isDouble()) return false;
    out={a[0].toDouble(),a[1].toDouble(),a[2].toDouble()};
    return std::isfinite(out.x) && std::isfinite(out.y) && std::isfinite(out.z);
}

bool validationNumericFieldsAreFinite(const QJsonObject& object, QString& badField)
{
    const QStringList keys={
        QStringLiteral("pose_count"), QStringLiteral("profile_count"), QStringLiteral("point_count"),
        QStringLiteral("surface_mean_mm"), QStringLiteral("surface_mae_mm"),
        QStringLiteral("surface_rms_mm"), QStringLiteral("surface_max_abs_mm"),
        QStringLiteral("center_xz_rms_mm"), QStringLiteral("center_3d_rms_mm"),
        QStringLiteral("center_max_mm"), QStringLiteral("training_surface_rms_mm"),
        QStringLiteral("required_pose_count"), QStringLiteral("threshold_surface_rms_mm"),
        QStringLiteral("threshold_surface_max_mm"), QStringLiteral("threshold_center_xz_rms_mm")
    };
    for(const QString& key:keys){
        double value=0.0;
        if(!finiteJsonNumber(object,key,value)){badField=key;return false;}
    }
    Vec3d bias;
    if(!finiteJsonVec3(object.value(QStringLiteral("center_bias_sensor_mm")),bias)){
        badField=QStringLiteral("center_bias_sensor_mm");
        return false;
    }
    badField.clear();
    return true;
}

bool sameStage1TranslationGauge(const Physical17Parameters& stage1,
                                const Physical17Parameters& stage2,
                                double* maxDeltaMm=nullptr)
{
    // Stage-2 intentionally keeps the Stage-1 hand-eye translation as the
    // translation-gauge anchor.  Therefore provenance matching must be much
    // tighter than a metrology acceptance tolerance: these values should be
    // the same serialized anchor, not merely physically close.
    double maxDelta=0.0;
    for(std::size_t i=Physical17Parameters::HandEyeTx;
        i<=Physical17Parameters::HandEyeTz;++i){
        const double a=stage1[i];
        const double b=stage2[i];
        if(!std::isfinite(a) || !std::isfinite(b)){
            if(maxDeltaMm) *maxDeltaMm=std::numeric_limits<double>::infinity();
            return false;
        }
        const double delta=std::abs(a-b);
        maxDelta=std::max(maxDelta,delta);
        const double tolerance=std::max(1.0e-6,
            std::max(std::abs(a),std::abs(b))*1.0e-10);
        if(delta>tolerance){
            if(maxDeltaMm) *maxDeltaMm=maxDelta;
            return false;
        }
    }
    if(maxDeltaMm) *maxDeltaMm=maxDelta;
    return true;
}

} // namespace

SingleSphere14CalibrationWidget::SingleSphere14CalibrationWidget(QWidget* parent)
    : QWidget(parent)
    , m_calibrator(Physical17SolverOptions())
{
    setupUi();
    connectUi();

    QString restoreError;
    const bool restored = loadPersistentCalibration(restoreError);

    updateDeviceState();
    updateSessionSummary();
    updateResultTable(m_hasResult ? &m_lastResult : nullptr);
    updateValidationSummary();
    updateCalibrationReadiness();
    updateWorkflowGuide();
    updateStage1Status();
    updateStage1ResultDisplay();

    QString startupMessage;
    if (restored) {
        startupMessage = m_startupRestoreMessage;
        if (startupMessage.isEmpty())
            startupMessage = tr("已发现可恢复的标定文件。");
    } else if (!restoreError.isEmpty()) {
        startupMessage = tr("自动恢复失败，已按‘未标定’启动：%1").arg(restoreError);
    } else {
        startupMessage = tr("未找到默认有效标定。启动时已检查 AppData、程序目录 calibration 子目录，并扫描程序目录中的阶段1/阶段2旧版JSON。");
    }
    if (m_lblConvention) m_lblConvention->setText(startupMessage);
    QTimer::singleShot(0, this, [this, startupMessage]() {
        appendLog(tr("启动标定恢复：%1").arg(startupMessage));
    });
}

SingleSphere14CalibrationWidget::~SingleSphere14CalibrationWidget()
{
    shutdownBackgroundWork();
}

void SingleSphere14CalibrationWidget::shutdownBackgroundWork()
{
    if (m_stage2PlanCancelFlag)
        m_stage2PlanCancelFlag->store(true, std::memory_order_relaxed);

    if (m_stage2PlanWatcher) {
        disconnect(m_stage2PlanWatcher, nullptr, this, nullptr);
        m_stage2PlanWatcher->waitForFinished();
        delete m_stage2PlanWatcher;
        m_stage2PlanWatcher = nullptr;
    }

    m_stage2PlanRunning = false;
    m_stage2PlanCancelFlag.reset();
}

void SingleSphere14CalibrationWidget::setController(ZmcController* controller)
{
    m_controller = controller;
    refreshDeviceState();
}

void SingleSphere14CalibrationWidget::setSensor(LaserSensor* sensor)
{
    // The object pointer normally stays the same while its internal connection state
    // changes asynchronously.  Do not silently skip the UI refresh in that case.
    if (m_sensor == sensor) {
        refreshDeviceState();
        return;
    }
    if (m_sensor) disconnect(m_sensor, nullptr, this, nullptr);
    if (m_profileCaptureConnection) {
        disconnect(m_profileCaptureConnection);
        m_profileCaptureConnection = {};
    }
    m_sensor = sensor;
    // V27.6.16M: do not enqueue every 200 Hz profile into the calibration GUI.
    // Fresh-profile capture arms a temporary subscription; preview/fit reads the
    // latest decoded in-memory snapshot from LaserSensor.
    refreshDeviceState();
}

void SingleSphere14CalibrationWidget::setPointCloudBuilder(PointCloudBuilder* builder)
{
    m_cloudBuilder = builder;
}

void SingleSphere14CalibrationWidget::setStage2SafetyCheckers(
    PoseSafetyChecker poseChecker, TransitionSafetyChecker transitionChecker,
    PlanningBoundsResolver planningBoundsResolver)
{
    m_stage2PoseSafetyChecker = std::move(poseChecker);
    m_stage2TransitionSafetyChecker = std::move(transitionChecker);
    m_stage2PlanningBoundsResolver = std::move(planningBoundsResolver);
}

void SingleSphere14CalibrationWidget::refreshDeviceState()
{
    updateDeviceState();

    // A persisted calibration is never blindly trusted. Once both devices are
    // online, bind it automatically only if the live Flip and DPOS/MPOS convention
    // exactly matches the convention saved with the accepted calibration.
    if (m_persistentCalibrationLoaded && !m_referenceTaught
        && m_controller && m_controller->isConnected()
        && m_sensor && m_sensor->isConnected()) {
        QString bindError;
        if (tryBindPersistentCalibration(bindError)) {
            m_persistentBindError.clear();
            appendLog(tr("已自动恢复上次有效标定：17参数（11项机床几何误差 + 6DoF手眼）和独立验证结论已绑定到当前设备坐标约定，可直接用于扫描/RTCP。"));
            if(m_persistentNeedsCanonicalSave){
                QString persistenceError;
                if(savePersistentCalibration(persistenceError)){
                    appendLog(tr("旧版阶段1/阶段2/验证文件已迁移为默认有效标定；以后启动将直接恢复。"));
                }else{
                    appendLog(tr("旧版标定已绑定，但升级保存失败：%1").arg(persistenceError));
                }
            }
        } else if (!bindError.isEmpty() && bindError != m_persistentBindError) {
            m_persistentBindError = bindError;
            appendLog(tr("上次有效标定已载入，但自动绑定被拒绝：%1").arg(bindError));
        }
    }

    // MainWindow calls this only on asynchronous connect/disconnect transitions.
    // Refresh the offline-result bind gate here without running observability on
    // every incoming laser profile (onSensorProfile calls updateDeviceState only).
    updateCalibrationReadiness();
    updateWorkflowGuide();
    updateStage1Status();
}

void SingleSphere14CalibrationWidget::setupUi()
{
    // V27.6.9: one calibration state machine, two UI pages.
    // Stage 1 is mounted in the outer "初步手眼标定" tab; Stage 2 +
    // independent validation are mounted in the outer "运动学标定" tab.
    m_stage1Page = new QWidget(this);
    auto* stage1PageLayout = new QVBoxLayout(m_stage1Page);
    stage1PageLayout->setContentsMargins(4, 4, 4, 4);
    stage1PageLayout->setSpacing(6);

    m_stage2Page = new QWidget(this);
    auto* stage2PageLayout = new QVBoxLayout(m_stage2Page);
    stage2PageLayout->setContentsMargins(4, 4, 4, 4);
    stage2PageLayout->setSpacing(6);

    // ================================================================
    // 阶段1统一操作区
    // 旧版曾同时保留“快捷操作台 + 设备状态 + RANSAC组 + 阶段1求解组”，
    // 导致同一阶段的按钮/状态重复出现。V27.6.8 将它们合并为一个工作流。
    // ================================================================
    auto* stage1Group = new QGroupBox(
        tr("阶段1：人工多姿态初标（名义机床模型 + 手眼6DoF + 固定球心3D）"), m_stage1Page);
    auto* stage1Layout = new QVBoxLayout(stage1Group);
    stage1Layout->setSpacing(6);

    m_lblWorkflow = new QLabel(stage1Group);
    m_lblWorkflow->setWordWrap(true);
    m_lblWorkflow->setStyleSheet(
        "QLabel { background:#eaf4ff; padding:6px; border:1px solid #8db6df; font-weight:bold; }");
    stage1Layout->addWidget(m_lblWorkflow);

    m_lblStage1Status = new QLabel(tr("阶段1实时状态：等待设备连接。"), stage1Group);
    m_lblStage1Status->setWordWrap(true);
    m_lblStage1Status->setStyleSheet(
        "QLabel { background:#f4f8fb; padding:6px; border:1px solid #9cb4c8; font-weight:bold; }");
    stage1Layout->addWidget(m_lblStage1Status);

    m_lblStage2Gate = new QLabel(
        tr("在线阶段2姿态规划/实机采集锁定：必须先完成阶段1手眼+球心 rank 9/9；已有Stage2-only CSV可直接离线自举并求解。"), stage1Group);
    m_lblStage2Gate->setWordWrap(true);
    m_lblStage2Gate->setStyleSheet(
        "QLabel { background:#fff3cd; color:#7a5200; padding:4px; border:1px solid #d5b95f; }");
    stage1Layout->addWidget(m_lblStage2Gate);

    // m_lblBootstrap 保留为内部详细状态载体，避免改动既有求解状态逻辑；
    // UI不再单独显示第二块“阶段1状态”，所有关键结果都汇总到上面的实时状态和弹窗。
    m_lblBootstrap = new QLabel(stage1Group);
    m_lblBootstrap->hide();

    // ---- 阶段1固定流程按钮：只保留一套 ----
    auto* stage1ActionGrid = new QGridLayout;

    m_btnTeach = new QPushButton(tr("① 开始/重置阶段1（锁定坐标约定）"), stage1Group);
    m_btnStartPreview = new QPushButton(tr("② 启动轮廓预览"), stage1Group);
    m_btnStopPreview = new QPushButton(tr("停止轮廓预览"), stage1Group);
    m_btnReadPose = new QPushButton(tr("读取当前XYZCB"), stage1Group);
    m_btnFit = new QPushButton(tr("拟合当前球"), stage1Group);
    m_btnStopMove = new QPushButton(tr("停止五轴运动"), stage1Group);
    m_btnStage1Capture = new QPushButton(tr("③ 采集当前阶段1姿态（自动拟合）"), stage1Group);
    m_btnBootstrapSolve = new QPushButton(tr("④ 求解阶段1（手眼6DoF + 球心3D，rank 9/9）"), stage1Group);

    m_btnStage1Capture->setStyleSheet(
        "QPushButton { background:#ffe0a8; font-weight:bold; min-height:28px; }");
    m_btnBootstrapSolve->setStyleSheet(
        "QPushButton { background:#d9ecff; font-weight:bold; min-height:28px; }");
    m_btnStopMove->setStyleSheet("QPushButton { background:#ffdddd; }");

    stage1ActionGrid->addWidget(m_btnTeach, 0, 0, 1, 2);
    stage1ActionGrid->addWidget(m_btnStartPreview, 0, 2);
    stage1ActionGrid->addWidget(m_btnStopPreview, 0, 3);
    stage1ActionGrid->addWidget(m_btnReadPose, 1, 0);
    stage1ActionGrid->addWidget(m_btnFit, 1, 1);
    stage1ActionGrid->addWidget(m_btnStopMove, 1, 2, 1, 2);
    stage1ActionGrid->addWidget(m_btnStage1Capture, 2, 0, 1, 2);
    stage1ActionGrid->addWidget(m_btnBootstrapSolve, 2, 2, 1, 2);
    stage1Layout->addLayout(stage1ActionGrid);

    // ---- 阶段1五轴人工小步 ----
    auto* motionBox = new QGroupBox(tr("阶段1人工找球 / 姿态激励"), stage1Group);
    auto* motionLayout = new QGridLayout(motionBox);

    m_spinStage1LinearStep = new QDoubleSpinBox(motionBox);
    m_spinStage1LinearStep->setRange(0.01, 10.0);
    m_spinStage1LinearStep->setDecimals(3);
    m_spinStage1LinearStep->setValue(0.50);
    m_spinStage1LinearStep->setSuffix(" mm");

    m_spinStage1RotaryStep = new QDoubleSpinBox(motionBox);
    m_spinStage1RotaryStep->setRange(0.1, 10.0);
    m_spinStage1RotaryStep->setDecimals(2);
    m_spinStage1RotaryStep->setValue(2.0);
    m_spinStage1RotaryStep->setSuffix("°");

    m_spinStage1Speed = new QDoubleSpinBox(motionBox);
    m_spinStage1Speed->setRange(0.01, 0.30);
    m_spinStage1Speed->setDecimals(2);
    m_spinStage1Speed->setSingleStep(0.01);
    m_spinStage1Speed->setValue(0.10);

    motionLayout->addWidget(new QLabel(tr("XYZ单步")), 0, 0);
    motionLayout->addWidget(m_spinStage1LinearStep, 0, 1);
    motionLayout->addWidget(new QLabel(tr("B/C单步")), 0, 2);
    motionLayout->addWidget(m_spinStage1RotaryStep, 0, 3);
    motionLayout->addWidget(new QLabel(tr("速度倍率")), 0, 4);
    motionLayout->addWidget(m_spinStage1Speed, 0, 5);

    auto addNudgePair = [this, motionBox, motionLayout](
                            int row, int col, const QString& axisText,
                            int axis, bool rotary) {
        auto* label = new QLabel(axisText, motionBox);
        auto* minus = new QPushButton(QStringLiteral("-"), motionBox);
        auto* plus = new QPushButton(QStringLiteral("+"), motionBox);
        minus->setMaximumWidth(46);
        plus->setMaximumWidth(46);
        const QString tip =
            tr("按当前实际静止姿态做相对小步；仍走五轴安全运动入口、软限位和轨迹检查。");
        minus->setToolTip(tip);
        plus->setToolTip(tip);
        motionLayout->addWidget(label, row, col);
        motionLayout->addWidget(minus, row, col + 1);
        motionLayout->addWidget(plus, row, col + 2);
        m_stage1NudgeButtons.push_back(minus);
        m_stage1NudgeButtons.push_back(plus);
        connect(minus, &QPushButton::clicked, this, [this, axis, rotary]() {
            const double step = rotary ? m_spinStage1RotaryStep->value()
                                       : m_spinStage1LinearStep->value();
            onStage1Nudge(axis, -step);
        });
        connect(plus, &QPushButton::clicked, this, [this, axis, rotary]() {
            const double step = rotary ? m_spinStage1RotaryStep->value()
                                       : m_spinStage1LinearStep->value();
            onStage1Nudge(axis, +step);
        });
    };

    addNudgePair(1, 0, QStringLiteral("X"), AxisX, false);
    addNudgePair(1, 3, QStringLiteral("Y"), AxisY, false);
    addNudgePair(2, 0, QStringLiteral("Z"), AxisZ, false);
    addNudgePair(2, 3, QStringLiteral("C"), AxisC, true);
    addNudgePair(3, 0, QStringLiteral("B"), AxisB, true);
    stage1Layout->addWidget(motionBox);

    // ---- 阶段1候选姿态：复用本机第一次成功标定的可见轨迹模板 ----
    auto* stage1PlanBox = new QGroupBox(tr("阶段1候选姿态（第一次成功标定轨迹模板 + 补充插值）"), stage1Group);
    auto* stage1PlanLayout = new QVBoxLayout(stage1PlanBox);
    auto* stage1PlanNote = new QLabel(
        tr("点击生成后，以①开始阶段1时的XYZCB为锚点，按本机第一次成功标定的相对轨迹生成候选姿态。"
           "每次只下发一个完整XYZCB目标，不做分段视觉推进；到位后自动检查球。若球未识别，可保留当前姿态用下方点动微调，或直接跳过并选择下一组。"),
        stage1PlanBox);
    stage1PlanNote->setWordWrap(true);
    stage1PlanLayout->addWidget(stage1PlanNote);
    auto* stage1PlanButtons = new QHBoxLayout;
    m_btnGenerateStage1Candidates = new QPushButton(tr("生成/重生成阶段1候选姿态"), stage1PlanBox);
    m_btnMoveStage1Candidate = new QPushButton(tr("直接移动到选中候选姿态"), stage1PlanBox);
    m_btnSkipStage1Candidate = new QPushButton(tr("跳过并选择下一候选"), stage1PlanBox);
    stage1PlanButtons->addWidget(m_btnGenerateStage1Candidates);
    stage1PlanButtons->addWidget(m_btnMoveStage1Candidate);
    stage1PlanButtons->addWidget(m_btnSkipStage1Candidate);
    stage1PlanLayout->addLayout(stage1PlanButtons);
    m_stage1CandidateTable = new QTableWidget(0, 7, stage1PlanBox);
    m_stage1CandidateTable->setHorizontalHeaderLabels({tr("序号"),tr("X"),tr("Y"),tr("Z"),tr("C"),tr("B"),tr("状态")});
    m_stage1CandidateTable->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    m_stage1CandidateTable->horizontalHeader()->setStretchLastSection(true);
    m_stage1CandidateTable->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_stage1CandidateTable->setSelectionMode(QAbstractItemView::SingleSelection);
    m_stage1CandidateTable->setMaximumHeight(190);
    stage1PlanLayout->addWidget(m_stage1CandidateTable);
    stage1Layout->addWidget(stage1PlanBox);

    // ---- 阶段1结果：以前只写入隐藏label/弹窗，现在常驻显示并可单独保存 ----
    auto* stage1ResultBox = new QGroupBox(tr("阶段1初步手眼标定结果"), stage1Group);
    auto* stage1ResultLayout = new QVBoxLayout(stage1ResultBox);
    m_lblStage1ResultSummary = new QLabel(tr("尚无阶段1结果。"), stage1ResultBox);
    m_lblStage1ResultSummary->setWordWrap(true);
    m_lblStage1ResultSummary->setStyleSheet("QLabel { background:#f7fbff; padding:5px; border:1px solid #9cb4c8; }");
    stage1ResultLayout->addWidget(m_lblStage1ResultSummary);
    m_stage1ResultTable = new QTableWidget(9, 3, stage1ResultBox);
    m_stage1ResultTable->setHorizontalHeaderLabels({tr("量"),tr("数值"),tr("单位")});
    m_stage1ResultTable->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    m_stage1ResultTable->verticalHeader()->setVisible(false);
    m_stage1ResultTable->setMaximumHeight(230);
    stage1ResultLayout->addWidget(m_stage1ResultTable);
    auto* stage1ResultActions = new QHBoxLayout;
    m_btnSaveStage1Result = new QPushButton(tr("保存阶段1结果 JSON"), stage1ResultBox);
    m_btnLoadStage1Result = new QPushButton(tr("加载阶段1结果 JSON（离线恢复）"), stage1ResultBox);
    stage1ResultActions->addWidget(m_btnSaveStage1Result);
    stage1ResultActions->addWidget(m_btnLoadStage1Result);
    stage1ResultLayout->addLayout(stage1ResultActions);
    stage1Layout->addWidget(stage1ResultBox);

    // ---- 当前拟合 + 采集进度始终可见 ----
    m_lblFit = new QLabel(tr("球拟合：尚未拟合"), stage1Group);
    m_lblFit->setWordWrap(true);
    m_lblFit->setStyleSheet(
        "QLabel { background:#fffdf2; padding:4px; border:1px solid #d8c36a; }");
    stage1Layout->addWidget(m_lblFit);

    m_stage1CaptureProgress = new QProgressBar(stage1Group);
    m_stage1CaptureProgress->setRange(0, 1);
    m_stage1CaptureProgress->setValue(0);
    m_stage1CaptureProgress->setFormat(tr("采集空闲"));
    stage1Layout->addWidget(m_stage1CaptureProgress);

    auto* stage1Hint = new QLabel(
        tr("日常阶段1只需要这一块：B/C改变姿态 → XYZ把球找回视场 → "
           "直接点击“采集当前阶段1姿态”。达到至少8组、建议12~20组且B/C独立激励后，点击“求解阶段1”。"
           "RANSAC和静态采集阈值通常保持默认，必要时再展开下方高级参数。"),
        stage1Group);
    stage1Hint->setWordWrap(true);
    stage1Hint->setStyleSheet("QLabel { color:#4b6278; }");
    stage1Layout->addWidget(stage1Hint);

    // ---- 低频使用的参数全部折叠，避免旧阶段1内容一直占满页面 ----
    auto* advancedButton = new QToolButton(stage1Group);
    advancedButton->setText(tr("展开：阶段1高级参数 / 坐标与设备明细"));
    advancedButton->setCheckable(true);
    advancedButton->setChecked(false);
    advancedButton->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    advancedButton->setArrowType(Qt::RightArrow);
    stage1Layout->addWidget(advancedButton);

    auto* advancedPanel = new QWidget(stage1Group);
    auto* advancedLayout = new QVBoxLayout(advancedPanel);
    advancedLayout->setContentsMargins(8, 0, 0, 0);

    auto* detailBox = new QGroupBox(tr("设备 / 坐标约定明细"), advancedPanel);
    auto* detailLayout = new QVBoxLayout(detailBox);
    m_lblDevice = new QLabel(tr("控制器/传感器状态未知"), detailBox);
    m_lblCurrentPose = new QLabel(tr("当前五轴：未读取"), detailBox);
    m_lblAcquisitionMode = new QLabel(
        tr("标定采集：软件内部触发 + 全测量场预设(index 0)；ROI只用于软件RANSAC，不改变传感器物理测量范围。"),
        detailBox);
    m_lblConvention = new QLabel(
        tr("坐标约定：阶段1尚未开始；Flip X/Flip Z 与 DPOS/MPOS 数据源尚未锁定。"),
        detailBox);
    for (auto* label : {m_lblDevice, m_lblCurrentPose, m_lblAcquisitionMode, m_lblConvention}) {
        label->setWordWrap(true);
        detailLayout->addWidget(label);
    }
    advancedLayout->addWidget(detailBox);

    auto* fitBox = new QGroupBox(tr("RANSAC球弧拟合参数"), advancedPanel);
    auto* fitGrid = new QGridLayout(fitBox);

    m_spinSphereDiameter = new QDoubleSpinBox(fitBox);
    m_spinSphereDiameter->setRange(1.0, 100.0);
    m_spinSphereDiameter->setDecimals(4);
    m_spinSphereDiameter->setValue(19.05);
    m_spinSphereDiameter->setSuffix(" mm");
    m_spinSphereDiameter->setEnabled(false);

    m_spinRoiXMin = new QDoubleSpinBox(fitBox);
    m_spinRoiXMax = new QDoubleSpinBox(fitBox);
    m_spinRoiZMin = new QDoubleSpinBox(fitBox);
    m_spinRoiZMax = new QDoubleSpinBox(fitBox);
    for (auto* s : {m_spinRoiXMin, m_spinRoiXMax, m_spinRoiZMin, m_spinRoiZMax}) {
        s->setRange(-500.0, 500.0);
        s->setDecimals(3);
    }
    m_spinRoiXMin->setValue(-15.0);
    m_spinRoiXMax->setValue(15.0);
    m_spinRoiZMin->setValue(80.0);
    m_spinRoiZMax->setValue(110.0);

    m_spinRansacIterations = new QSpinBox(fitBox);
    m_spinRansacIterations->setRange(20, 5000);
    m_spinRansacIterations->setValue(350);

    m_spinRansacThreshold = new QDoubleSpinBox(fitBox);
    m_spinRansacThreshold->setRange(0.001, 5.0);
    m_spinRansacThreshold->setDecimals(4);
    m_spinRansacThreshold->setValue(0.050);
    m_spinRansacThreshold->setSuffix(" mm");

    m_spinRansacMinRatio = new QDoubleSpinBox(fitBox);
    m_spinRansacMinRatio->setRange(0.01, 1.0);
    m_spinRansacMinRatio->setDecimals(2);
    m_spinRansacMinRatio->setValue(0.20);

    m_spinRansacMinArc = new QDoubleSpinBox(fitBox);
    m_spinRansacMinArc->setRange(1.0, 180.0);
    m_spinRansacMinArc->setDecimals(1);
    m_spinRansacMinArc->setValue(30.0);
    m_spinRansacMinArc->setSuffix("°");

    m_spinMaxCircleRms = new QDoubleSpinBox(fitBox);
    m_spinMaxCircleRms->setRange(0.001, 1.0);
    m_spinMaxCircleRms->setDecimals(4);
    m_spinMaxCircleRms->setValue(0.030);
    m_spinMaxCircleRms->setSuffix(" mm");

    int fr = 0;
    fitGrid->addWidget(new QLabel(tr("球直径")), fr, 0);
    fitGrid->addWidget(m_spinSphereDiameter, fr++, 1);
    fitGrid->addWidget(new QLabel(tr("ROI X min/max")), fr, 0);
    auto* xr = new QHBoxLayout;
    xr->addWidget(m_spinRoiXMin); xr->addWidget(m_spinRoiXMax);
    fitGrid->addLayout(xr, fr++, 1);
    fitGrid->addWidget(new QLabel(tr("ROI Z min/max")), fr, 0);
    auto* zr = new QHBoxLayout;
    zr->addWidget(m_spinRoiZMin); zr->addWidget(m_spinRoiZMax);
    fitGrid->addLayout(zr, fr++, 1);
    fitGrid->addWidget(new QLabel(tr("RANSAC次数")), fr, 0);
    fitGrid->addWidget(m_spinRansacIterations, fr++, 1);
    fitGrid->addWidget(new QLabel(tr("内点阈值")), fr, 0);
    fitGrid->addWidget(m_spinRansacThreshold, fr++, 1);
    fitGrid->addWidget(new QLabel(tr("最小内点比例")), fr, 0);
    fitGrid->addWidget(m_spinRansacMinRatio, fr++, 1);
    fitGrid->addWidget(new QLabel(tr("最小圆弧覆盖")), fr, 0);
    fitGrid->addWidget(m_spinRansacMinArc, fr++, 1);
    fitGrid->addWidget(new QLabel(tr("最大圆拟合RMS")), fr, 0);
    fitGrid->addWidget(m_spinMaxCircleRms, fr++, 1);
    advancedLayout->addWidget(fitBox);

    // 阶段1和阶段2共用同一套“静止采集质量”参数。
    auto* captureBox = new QGroupBox(tr("阶段1/阶段2共用静态采集参数"), advancedPanel);
    auto* captureGrid = new QGridLayout(captureBox);

    m_spinCaptureFrames = new QSpinBox(captureBox);
    m_spinCaptureFrames->setRange(1, 100);
    m_spinCaptureFrames->setValue(10);

    m_spinSettleMs = new QSpinBox(captureBox);
    m_spinSettleMs->setRange(0, 10000);
    m_spinSettleMs->setValue(500);
    m_spinSettleMs->setSuffix(" ms");

    m_spinLinearDrift = new QDoubleSpinBox(captureBox);
    m_spinLinearDrift->setRange(0.001, 1.0);
    m_spinLinearDrift->setDecimals(4);
    m_spinLinearDrift->setValue(0.020);
    m_spinLinearDrift->setSuffix(" mm");

    m_spinRotaryDrift = new QDoubleSpinBox(captureBox);
    m_spinRotaryDrift->setRange(0.001, 1.0);
    m_spinRotaryDrift->setDecimals(4);
    m_spinRotaryDrift->setValue(0.020);
    m_spinRotaryDrift->setSuffix("°");

    captureGrid->addWidget(new QLabel(tr("每姿态 fresh profiles")), 0, 0);
    captureGrid->addWidget(m_spinCaptureFrames, 0, 1);
    captureGrid->addWidget(new QLabel(tr("停稳等待")), 1, 0);
    captureGrid->addWidget(m_spinSettleMs, 1, 1);
    captureGrid->addWidget(new QLabel(tr("XYZ最大漂移")), 2, 0);
    captureGrid->addWidget(m_spinLinearDrift, 2, 1);
    captureGrid->addWidget(new QLabel(tr("B/C最大漂移")), 3, 0);
    captureGrid->addWidget(m_spinRotaryDrift, 3, 1);
    advancedLayout->addWidget(captureBox);

    advancedPanel->setVisible(false);
    connect(advancedButton, &QToolButton::toggled, this,
            [advancedButton, advancedPanel](bool on) {
        advancedPanel->setVisible(on);
        advancedButton->setArrowType(on ? Qt::DownArrow : Qt::RightArrow);
        advancedButton->setText(
            on ? QObject::tr("收起：阶段1高级参数 / 坐标与设备明细")
               : QObject::tr("展开：阶段1高级参数 / 坐标与设备明细"));
    });

    stage1Layout->addWidget(advancedPanel);
    stage1PageLayout->addWidget(stage1Group);
    stage1PageLayout->addStretch(1);

    m_lblKinematicGate = new QLabel(
        tr("运动学标定锁定：请先在“初步手眼标定”页完成阶段1 rank 9/9 初标。"), m_stage2Page);
    m_lblKinematicGate->setWordWrap(true);
    m_lblKinematicGate->setStyleSheet(
        "QLabel { background:#fff3cd; color:#7a5200; padding:6px; border:1px solid #d5b95f; font-weight:bold; }");
    stage2PageLayout->addWidget(m_lblKinematicGate);

    auto* stage2Intro = new QLabel(
        tr("本页只包含阶段2：D-optimal训练姿态规划、17物理参数/rank14联合求解，以及阶段3独立hold-out验证。"
           "阶段1的手眼6DoF和固定球心结果会在内存中直接传入本页，不需要导出/重新加载。"),
        m_stage2Page);
    stage2Intro->setWordWrap(true);
    stage2Intro->setStyleSheet("QLabel { color:#4b6278; padding:2px; }");
    stage2PageLayout->addWidget(stage2Intro);

    // V27.6.17: a new kinematic-calibration run may use a freshly measured
    // machine-frame sphere center while keeping the last validated Physical17
    // result as the navigation seed/fallback.  This mirrors the sensor-study
    // cloud -> RANSAC sphere workflow and removes the old "restart means no
    // re-calibration" dead end.
    auto* stage2SphereGroup = new QGroupBox(tr("阶段2球心参考（可选：导入当前安装的标准球点云）"), m_stage2Page);
    auto* stage2SphereLayout = new QVBoxLayout(stage2SphereGroup);
    auto* stage2SphereButtons = new QHBoxLayout;
    m_btnImportStage2Sphere = new QPushButton(tr("导入球面点云"), stage2SphereGroup);
    m_btnFitStage2Sphere = new QPushButton(tr("RANSAC拟合球心"), stage2SphereGroup);
    m_btnFitStage2Sphere->setEnabled(false);
    stage2SphereButtons->addWidget(m_btnImportStage2Sphere);
    stage2SphereButtons->addWidget(m_btnFitStage2Sphere);
    stage2SphereButtons->addStretch(1);
    stage2SphereLayout->addLayout(stage2SphereButtons);
    m_lblStage2Sphere = new QLabel(tr("未导入点云；姿态规划将使用阶段1/上次有效标定中的固定球心。"), stage2SphereGroup);
    m_lblStage2Sphere->setWordWrap(true);
    m_lblStage2Sphere->setStyleSheet("QLabel { color:#4b6278; }");
    stage2SphereLayout->addWidget(m_lblStage2Sphere);
    stage2PageLayout->addWidget(stage2SphereGroup);

    auto* plannerGroup = new QGroupBox(tr("阶段2：17参数D-optimal姿态规划与静态采集"), m_stage2Page);
    auto* plannerLayout = new QVBoxLayout(plannerGroup);
    auto* cfg = new QGridLayout;
    m_spinPoseCount = new QSpinBox(plannerGroup); m_spinPoseCount->setRange(14, 160); m_spinPoseCount->setValue(48);

    auto makeQualitySpin = [plannerGroup](double lo, double hi, double value, double step, const QString& suffix) {
        auto* w = new QDoubleSpinBox(plannerGroup);
        w->setRange(lo, hi); w->setDecimals(2); w->setValue(value); w->setSingleStep(step); w->setSuffix(suffix);
        return w;
    };
    // Defaults come from the completed sensor-quality study: keep the center
    // distance in 92..100 mm and avoid the empirically fragile |eta|=15 deg
    // edge by default. Gamma remains broad so kinematic observability is not
    // sacrificed; all candidates still pass D-optimal selection afterwards.
    m_spinQualityDMin = makeQualitySpin(70.0, 120.0, 92.0, 1.0, tr(" mm"));
    m_spinQualityDMax = makeQualitySpin(70.0, 120.0, 100.0, 1.0, tr(" mm"));
    m_spinQualityDStep = makeQualitySpin(0.5, 20.0, 4.0, 0.5, tr(" mm"));
    m_spinQualityGammaMin = makeQualitySpin(0.0, 80.0, 0.0, 5.0, QString::fromUtf8(" °"));
    m_spinQualityGammaMax = makeQualitySpin(0.0, 80.0, 50.0, 5.0, QString::fromUtf8(" °"));
    m_spinQualityGammaStep = makeQualitySpin(1.0, 40.0, 25.0, 1.0, QString::fromUtf8(" °"));
    m_spinQualityEtaMin = makeQualitySpin(-45.0, 45.0, -7.5, 2.5, QString::fromUtf8(" °"));
    m_spinQualityEtaMax = makeQualitySpin(-45.0, 45.0, +7.5, 2.5, QString::fromUtf8(" °"));
    m_spinQualityEtaStep = makeQualitySpin(0.5, 30.0, 7.5, 0.5, QString::fromUtf8(" °"));

    m_spinStage2MinInlierRatio = makeQualitySpin(0.10, 1.0, 0.80, 0.05, QString());
    m_spinStage2MaxRms = makeQualitySpin(0.001, 0.100, 0.010, 0.002, tr(" mm"));
    m_spinStage2MaxP95 = makeQualitySpin(0.001, 0.200, 0.020, 0.005, tr(" mm"));
    m_spinStage2EdgeTrimFraction = makeQualitySpin(0.0, 0.25, 0.05, 0.01, QString());
    m_spinStage2MinValidFrameRatio = makeQualitySpin(0.10, 1.0, 0.80, 0.05, QString());
    m_spinStage2MinInlierRatio->setToolTip(tr("阶段2正式采集的单帧RANSAC最低内点比例；只控制是否接受该帧，不改变RANSAC搜索本身。"));
    m_spinStage2MaxRms->setToolTip(tr("阶段2正式采集：裁边后内点径向RMS上限。默认10 μm，与质量分析的标定级硬门限一致。"));
    m_spinStage2MaxP95->setToolTip(tr("阶段2正式采集：裁掉圆弧两端后，RANSAC内点绝对径向残差P95上限。默认20 μm。"));
    m_spinStage2EdgeTrimFraction->setToolTip(tr("从RANSAC内点圆弧两端各裁掉的比例。默认0.05，即每端5%，避免边缘低信噪点进入17参数求解。"));
    m_spinStage2MinValidFrameRatio->setToolTip(tr("20帧正式采集中至少有多少比例的帧必须通过全部几何质量门禁。默认80%。"));

    m_spinBMin = new QDoubleSpinBox(plannerGroup); m_spinBMax = new QDoubleSpinBox(plannerGroup); m_spinBStep = new QDoubleSpinBox(plannerGroup);
    m_spinCMin = new QDoubleSpinBox(plannerGroup); m_spinCMax = new QDoubleSpinBox(plannerGroup); m_spinCStep = new QDoubleSpinBox(plannerGroup);
    for (auto* s : {m_spinBMin,m_spinBMax,m_spinBStep,m_spinCMin,m_spinCMax,m_spinCStep}) { s->setRange(-180,180); s->setDecimals(1); s->setSuffix("°"); }
    m_spinBMin->setValue(-30); m_spinBMax->setValue(30); m_spinBStep->setValue(10);
    m_spinCMin->setValue(-45); m_spinCMax->setValue(45); m_spinCStep->setValue(15);
    m_spinStage2XyzSpeedScale = new QDoubleSpinBox(plannerGroup);
    m_spinStage2XyzSpeedScale->setRange(0.01,1.0);
    m_spinStage2XyzSpeedScale->setSingleStep(0.05);
    m_spinStage2XyzSpeedScale->setDecimals(2);
    m_spinStage2XyzSpeedScale->setValue(0.15);
    m_spinStage2XyzSpeedScale->setSuffix(tr(" ×轴参数"));
    m_spinStage2XyzSpeedScale->setToolTip(tr(
        "只影响运动学标定的XYZ独立定位速度/加减速度倍率；不写控制器UNITS/软限位，不撤销XYZ DATUM。"));
    m_spinStage2BcSpeedScale = new QDoubleSpinBox(plannerGroup);
    m_spinStage2BcSpeedScale->setRange(0.01,1.0);
    m_spinStage2BcSpeedScale->setSingleStep(0.05);
    m_spinStage2BcSpeedScale->setDecimals(2);
    m_spinStage2BcSpeedScale->setValue(0.15);
    m_spinStage2BcSpeedScale->setSuffix(tr(" ×轴参数"));
    m_spinStage2BcSpeedScale->setToolTip(tr(
        "只影响运动学标定的B/C独立定位速度/加减速度倍率；不写B/C坐标参数，因此不会要求重新同步绝对位置。"));
    m_spinRecenterMaxStep = new QDoubleSpinBox(plannerGroup); m_spinRecenterMaxStep->setRange(0.05,10.0); m_spinRecenterMaxStep->setValue(1.0); m_spinRecenterMaxStep->setSuffix(" mm");
    // V27.6.10: 标准计划运动改为“一次下发一个完整XYZCB目标”。
    // 保留该控件对象仅兼容旧规划器字段，不再显示/用于运动分段。
    m_spinTransitAngleStep = new QDoubleSpinBox(plannerGroup);
    m_spinTransitAngleStep->setRange(0.5,180.0);
    m_spinTransitAngleStep->setValue(180.0);
    m_spinTransitAngleStep->hide();

    int cfgRow = 0;
    cfg->addWidget(new QLabel(tr("记录姿态数")), cfgRow, 0);
    cfg->addWidget(m_spinPoseCount, cfgRow++, 1);

    auto addTriple = [cfg, &cfgRow](const QString& label, QWidget* a, QWidget* b, QWidget* c) {
        cfg->addWidget(new QLabel(label), cfgRow, 0);
        auto* row = new QHBoxLayout; row->addWidget(a); row->addWidget(b); row->addWidget(c);
        cfg->addLayout(row, cfgRow++, 1);
    };
    addTriple(tr("质量候选 D min/max/step"), m_spinQualityDMin, m_spinQualityDMax, m_spinQualityDStep);
    addTriple(tr("质量候选 γ min/max/step"), m_spinQualityGammaMin, m_spinQualityGammaMax, m_spinQualityGammaStep);
    addTriple(tr("质量候选 η min/max/step"), m_spinQualityEtaMin, m_spinQualityEtaMax, m_spinQualityEtaStep);

    cfg->addWidget(new QLabel(tr("采集门禁：min内点 / RMS / P95 / 每端裁边 / min有效帧")), cfgRow, 0);
    auto* qr = new QHBoxLayout;
    qr->addWidget(m_spinStage2MinInlierRatio); qr->addWidget(m_spinStage2MaxRms); qr->addWidget(m_spinStage2MaxP95);
    qr->addWidget(m_spinStage2EdgeTrimFraction); qr->addWidget(m_spinStage2MinValidFrameRatio);
    cfg->addLayout(qr, cfgRow++, 1);

    cfg->addWidget(new QLabel(tr("B min/max/step")), cfgRow, 0);
    auto* br = new QHBoxLayout;
    br->addWidget(m_spinBMin); br->addWidget(m_spinBMax); br->addWidget(m_spinBStep);
    cfg->addLayout(br, cfgRow++, 1);

    cfg->addWidget(new QLabel(tr("C min/max/step")), cfgRow, 0);
    auto* cr = new QHBoxLayout;
    cr->addWidget(m_spinCMin); cr->addWidget(m_spinCMax); cr->addWidget(m_spinCStep);
    cfg->addLayout(cr, cfgRow++, 1);

    cfg->addWidget(new QLabel(tr("标定运动速度倍率 XYZ / B,C")), cfgRow, 0);
    auto* stage2SpeedRow = new QHBoxLayout;
    stage2SpeedRow->addWidget(new QLabel(tr("XYZ"), plannerGroup));
    stage2SpeedRow->addWidget(m_spinStage2XyzSpeedScale);
    stage2SpeedRow->addSpacing(12);
    stage2SpeedRow->addWidget(new QLabel(tr("B/C"), plannerGroup));
    stage2SpeedRow->addWidget(m_spinStage2BcSpeedScale);
    cfg->addLayout(stage2SpeedRow, cfgRow++, 1);
    cfg->addWidget(new QLabel(tr("单次 XYZ 微调最大步长")), cfgRow, 0);
    cfg->addWidget(m_spinRecenterMaxStep, cfgRow++, 1);
    cfg->setColumnStretch(1, 1);
    plannerLayout->addLayout(cfg);

    // V27.6.37: show machine readiness in Stage-2, but keep homing/servo
    // ownership in the existing global Control page.
    auto* motionReadyGroup = new QGroupBox(tr("真实运动准备（执行前门禁）"), plannerGroup);
    auto* motionReadyLayout = new QHBoxLayout(motionReadyGroup);
    m_lblStage2MotionReadiness = new QLabel(tr("运动准备状态：尚未刷新"), motionReadyGroup);
    m_lblStage2MotionReadiness->setWordWrap(true);
    m_btnStage2MotionControls = new QPushButton(
        tr("打开控制页：XYZ/五轴回零、重读B/C、人工使能"), motionReadyGroup);
    m_btnStage2MotionControls->setToolTip(tr(
        "阶段2不复制另一套回零/使能实现。点击后进入全局“控制”页，使用原有XYZ/五轴回零、B/C绝对位置重读和B/C人工使能。"));
    motionReadyLayout->addWidget(m_lblStage2MotionReadiness, 1);
    motionReadyLayout->addWidget(m_btnStage2MotionControls);
    plannerLayout->addWidget(motionReadyGroup);

    auto* stage2ActionGrid = new QGridLayout;
    m_btnGenerate = new QPushButton(tr("生成质量约束 + D-optimal 标定姿态"), plannerGroup);
    m_btnCancelStage2Plan = new QPushButton(tr("取消姿态生成"), plannerGroup);
    m_btnCancelStage2Plan->setEnabled(false);
    m_btnStage2SimulationStart = new QPushButton(tr("仿真运行标定姿态"), plannerGroup);
    m_btnStage2SimulationStart->setStyleSheet("QPushButton { background:#dceffc; font-weight:bold; }");
    m_btnStage2SimulationStart->setToolTip(tr("仅驱动GUI数字孪生，不发送任何控制器指令。逐姿态显示scanCONTROL梯形测量场、球面截圆、末端侧有效圆弧，并在二维轮廓页显示预计圆弧。"));
    m_btnStage2SimulationStop = new QPushButton(tr("停止姿态仿真"), plannerGroup);
    m_btnStage2SimulationStop->setEnabled(false);
    m_btnStage2SimulationClear = new QPushButton(tr("清除仿真显示"), plannerGroup);
    m_spinStage2SimulationIntervalMs = new QSpinBox(plannerGroup);
    m_spinStage2SimulationIntervalMs->setRange(150, 5000);
    m_spinStage2SimulationIntervalMs->setValue(800);
    m_spinStage2SimulationIntervalMs->setSingleStep(100);
    m_spinStage2SimulationIntervalMs->setSuffix(tr(" ms/姿态"));
    m_btnAutoAcquireStage2 = new QPushButton(tr("自动移动并采集阶段2计划"), plannerGroup);
    m_btnAutoAcquireStage2->setStyleSheet("QPushButton { background:#dceffc; font-weight:bold; }");
    m_btnStopAutoAcquireStage2 = new QPushButton(tr("停止自动移动/采集"), plannerGroup);
    m_btnStopAutoAcquireStage2->setStyleSheet("QPushButton { background:#ffdddd; font-weight:bold; }");
    m_btnStopAutoAcquireStage2->setEnabled(false);
    m_btnMove = new QPushButton(tr("直接移动到选中计划姿态"), plannerGroup);
    m_btnSkipPlanPose = new QPushButton(tr("跳过当前并选下一姿态"), plannerGroup);
    m_btnRecenter = new QPushButton(tr("按当前球心自动微调 XYZ"), plannerGroup);
    m_btnCapture = new QPushButton(tr("采集当前姿态"), plannerGroup);
    m_btnCapture->setStyleSheet("QPushButton { background:#ffe0a8; font-weight:bold; }");
    m_btnDeleteLast = new QPushButton(tr("删除最后一次采集"), plannerGroup);
    m_btnClear = new QPushButton(tr("清空会话"), plannerGroup);
    m_btnStage2StopMove = new QPushButton(tr("停止五轴运动"), plannerGroup);
    m_btnStage2StopMove->setStyleSheet("QPushButton { background:#ffdddd; font-weight:bold; }");
    stage2ActionGrid->addWidget(m_btnGenerate, 0, 0);
    stage2ActionGrid->addWidget(m_btnMove, 0, 1);
    stage2ActionGrid->addWidget(m_btnStage2SimulationStart, 1, 0);
    stage2ActionGrid->addWidget(m_btnStage2SimulationStop, 1, 1);
    stage2ActionGrid->addWidget(m_btnStage2SimulationClear, 2, 0);
    stage2ActionGrid->addWidget(m_spinStage2SimulationIntervalMs, 2, 1);
    stage2ActionGrid->addWidget(m_btnAutoAcquireStage2, 3, 0);
    stage2ActionGrid->addWidget(m_btnStopAutoAcquireStage2, 3, 1);
    stage2ActionGrid->addWidget(m_btnSkipPlanPose, 4, 0);
    stage2ActionGrid->addWidget(m_btnRecenter, 4, 1);
    stage2ActionGrid->addWidget(m_btnCapture, 5, 0);
    stage2ActionGrid->addWidget(m_btnDeleteLast, 5, 1);
    stage2ActionGrid->addWidget(m_btnStage2StopMove, 6, 0, 1, 2);
    stage2ActionGrid->addWidget(m_btnClear, 7, 0, 1, 2);
    plannerLayout->addLayout(stage2ActionGrid);

    auto* stage2PlanProgressRow = new QHBoxLayout;
    m_stage2PlanProgress = new QProgressBar(plannerGroup);
    m_stage2PlanProgress->setRange(0, 100);
    m_stage2PlanProgress->setValue(0);
    m_stage2PlanProgress->setFormat(tr("姿态生成 %p%"));
    m_lblStage2PlanProgress = new QLabel(tr("姿态生成：空闲"), plannerGroup);
    m_lblStage2PlanProgress->setMinimumWidth(300);
    m_lblStage2PlanProgress->setWordWrap(true);
    stage2PlanProgressRow->addWidget(m_stage2PlanProgress, 1);
    stage2PlanProgressRow->addWidget(m_btnCancelStage2Plan);
    plannerLayout->addLayout(stage2PlanProgressRow);
    plannerLayout->addWidget(m_lblStage2PlanProgress);

    m_planTable = new QTableWidget(0, 10, plannerGroup);
    m_planTable->setHorizontalHeaderLabels({tr("序号"),tr("X种子"),tr("Y种子"),tr("Z种子"),tr("C目标"),tr("B目标"),tr("目标球心S"),tr("信息增益"),tr("组"),tr("状态/质量")});
    m_planTable->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    m_planTable->horizontalHeader()->setStretchLastSection(true);
    m_planTable->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_planTable->setSelectionMode(QAbstractItemView::SingleSelection);
    m_planTable->setMinimumHeight(230);
    plannerLayout->addWidget(m_planTable);

    m_lblSession = new QLabel(plannerGroup);
    m_lblObservability = new QLabel(plannerGroup);
    m_lblObservability->setStyleSheet("QLabel { font-weight:bold; }");
    plannerLayout->addWidget(m_lblSession);
    plannerLayout->addWidget(m_lblObservability);
    stage2PageLayout->addWidget(plannerGroup);

    auto* solveGroup = new QGroupBox(tr("求解、结果与会话文件"), m_stage2Page);
    auto* solveLayout = new QVBoxLayout(solveGroup);
    auto* gaugeNote = new QLabel(tr(
        "阶段2的3D平移规范不再要求人工填写Tx/Ty/Tz：程序自动使用阶段1求得的手眼平移作为内部规范锚点。"
        "它足以保证球心规划和17参数联合求解自洽，但若以后要声明绝对RTCP机床坐标精度，仍需CAD/外部基准单独锁定绝对平移规范。"), solveGroup);
    gaugeNote->setWordWrap(true);
    solveLayout->addWidget(gaugeNote);
    auto* solveActions = new QGridLayout;
    m_btnSolve = new QPushButton(tr("阶段2最终求解：17物理参数 / 数据秩14"), solveGroup);
    m_btnSolve->setStyleSheet("QPushButton { background:#c9f7c9; font-weight:bold; }");
    m_btnCancelStage2Solve = new QPushButton(tr("取消阶段2求解"), solveGroup);
    m_btnCancelStage2Solve->setEnabled(false);
    m_btnSaveResult = new QPushButton(tr("保存结果 JSON"), solveGroup);
    m_btnLoadResult = new QPushButton(tr("加载阶段2结果 JSON"), solveGroup);
    m_btnBindLoadedResult = new QPushButton(tr("绑定已加载结果到当前实机坐标约定"), solveGroup);
    m_btnBindLoadedResult->setToolTip(tr("离线加载阶段1 JSON + 阶段2结果后，核对当前2950 Flip和五轴DPOS/MPOS数据源；一致时恢复实机验证/RTCP使用能力。不会重新求解。"));
    m_btnExportSession = new QPushButton(tr("导出阶段2训练 CSV"), solveGroup);
    m_btnImportSession = new QPushButton(tr("导入阶段2/旧版混合 CSV"), solveGroup);
    solveActions->addWidget(m_btnSolve, 0, 0);
    solveActions->addWidget(m_btnCancelStage2Solve, 0, 1);
    solveActions->addWidget(m_btnSaveResult, 1, 0);
    solveActions->addWidget(m_btnLoadResult, 1, 1);
    solveActions->addWidget(m_btnBindLoadedResult, 2, 0, 1, 2);
    solveActions->addWidget(m_btnExportSession, 3, 0);
    solveActions->addWidget(m_btnImportSession, 3, 1);
    solveLayout->addLayout(solveActions);
    m_lblSolveState = new QLabel(tr("尚未标定"), solveGroup);
    m_lblSolveState->setWordWrap(true);
    solveLayout->addWidget(m_lblSolveState);

    m_lblStage2SolveProgress = new QLabel(tr("阶段2求解空闲。"), solveGroup);
    m_lblStage2SolveProgress->setWordWrap(true);
    solveLayout->addWidget(m_lblStage2SolveProgress);
    m_stage2SolveProgress = new QProgressBar(solveGroup);
    m_stage2SolveProgress->setRange(0, 100);
    m_stage2SolveProgress->setValue(0);
    m_stage2SolveProgress->setFormat(tr("%p%"));
    solveLayout->addWidget(m_stage2SolveProgress);
    m_lblStage2SolveElapsed = new QLabel(tr("耗时：0.0 s"), solveGroup);
    solveLayout->addWidget(m_lblStage2SolveElapsed);
    m_stage2SolveDetails = new QPlainTextEdit(solveGroup);
    m_stage2SolveDetails->setReadOnly(true);
    m_stage2SolveDetails->setMaximumBlockCount(400);
    m_stage2SolveDetails->setMinimumHeight(150);
    m_stage2SolveDetails->setPlaceholderText(tr("这里会显示阶段2最终求解的准备、代表集优化、完整数据复核、rank/condition、成功或错误信息。"));
    solveLayout->addWidget(m_stage2SolveDetails);
    m_stage2SolveTimer = new QTimer(this);
    m_stage2SolveTimer->setInterval(500);

    m_resultTable = new QTableWidget(static_cast<int>(Physical17Parameters::Count), 3, solveGroup);
    m_resultTable->setHorizontalHeaderLabels({tr("参数"),tr("数值"),tr("单位")});
    m_resultTable->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    m_resultTable->setMaximumHeight(300);
    solveLayout->addWidget(m_resultTable);
    stage2PageLayout->addWidget(solveGroup);

    auto* validationGroup = new QGroupBox(tr("阶段3：独立验证（验证数据绝不参与求解）"), m_stage2Page);
    auto* validationLayout = new QVBoxLayout(validationGroup);
    auto* validationNote = new QLabel(tr(
        "最终17参数求解后，hold-out验证数据与训练数据严格分离。在线采集新的验证数据需要连接设备；"
        "已经用本软件单独保存的hold-out CSV可以离线导入并重新计算验证，不要求控制器或2950在线。"
        "训练CSV不能作为验证CSV导入。V27.6.42默认还会避开阶段2渐进筛选已经识别的高残差B/C危险邻域；"
        "因此通过结论只证明保留安全工作域，不能证明被排除危险区也有效。只有验证通过且当前实机坐标约定再次核对一致，标定才允许应用到受验证工作域。"), validationGroup);
    validationNote->setWordWrap(true);
    validationLayout->addWidget(validationNote);

    auto* validationCfg = new QGridLayout;
    m_spinValidationPoseCount = new QSpinBox(validationGroup);
    m_spinValidationPoseCount->setRange(6, 30);
    m_spinValidationPoseCount->setValue(8);
    m_spinValidationBCSep = new QDoubleSpinBox(validationGroup);
    m_spinValidationBCSep->setRange(0.5, 20.0);
    m_spinValidationBCSep->setDecimals(1);
    m_spinValidationBCSep->setValue(4.0);
    m_spinValidationBCSep->setSuffix("°");
    m_checkValidationAvoidRejected = new QCheckBox(tr("避开阶段2已剔除的高误差B/C区域"), validationGroup);
    m_checkValidationAvoidRejected->setChecked(true);
    m_checkValidationAvoidRejected->setToolTip(tr(
        "根据最终Physical17渐进筛选记录中的 rejectedTrainingPoses 建立B/C危险邻域。"
        "独立验证姿态不会落入这些已知高残差区域；因此验证结论只覆盖保留的安全B/C工作域，不能据此宣称危险区也通过。"));
    m_spinValidationHazardBCSep = new QDoubleSpinBox(validationGroup);
    m_spinValidationHazardBCSep->setRange(2.0, 30.0);
    m_spinValidationHazardBCSep->setDecimals(1);
    m_spinValidationHazardBCSep->setValue(10.0);
    m_spinValidationHazardBCSep->setSuffix("°");
    m_spinValidationHazardBCSep->setToolTip(tr(
        "验证候选与任一阶段2被剔除高误差姿态的B/C欧氏角距离必须不小于该值。默认10°用于避开已观察到的局部角度误差带。"));
    m_spinValidationRms = new QDoubleSpinBox(validationGroup);
    m_spinValidationRms->setRange(0.001, 2.0);
    m_spinValidationRms->setDecimals(4);
    m_spinValidationRms->setValue(0.080);
    m_spinValidationRms->setSuffix(" mm");
    m_spinValidationMax = new QDoubleSpinBox(validationGroup);
    m_spinValidationMax->setRange(0.001, 5.0);
    m_spinValidationMax->setDecimals(4);
    m_spinValidationMax->setValue(0.250);
    m_spinValidationMax->setSuffix(" mm");
    m_spinValidationCenterXz = new QDoubleSpinBox(validationGroup);
    m_spinValidationCenterXz->setRange(0.01, 5.0);
    m_spinValidationCenterXz->setDecimals(3);
    m_spinValidationCenterXz->setValue(0.500);
    m_spinValidationCenterXz->setSuffix(" mm");

    validationCfg->addWidget(new QLabel(tr("独立验证姿态数")), 0, 0);
    validationCfg->addWidget(m_spinValidationPoseCount, 0, 1);
    validationCfg->addWidget(new QLabel(tr("与训练姿态最小B/C间隔")), 1, 0);
    validationCfg->addWidget(m_spinValidationBCSep, 1, 1);
    validationCfg->addWidget(m_checkValidationAvoidRejected, 2, 0);
    validationCfg->addWidget(m_spinValidationHazardBCSep, 2, 1);
    validationCfg->addWidget(new QLabel(tr("验证球面 RMS 上限")), 3, 0);
    validationCfg->addWidget(m_spinValidationRms, 3, 1);
    validationCfg->addWidget(new QLabel(tr("验证球面 max 上限")), 4, 0);
    validationCfg->addWidget(m_spinValidationMax, 4, 1);
    validationCfg->addWidget(new QLabel(tr("预测球心 XZ RMS 上限")), 5, 0);
    validationCfg->addWidget(m_spinValidationCenterXz, 5, 1);
    validationCfg->setColumnStretch(1, 1);
    validationLayout->addLayout(validationCfg);

    auto* validationButtons = new QGridLayout;
    m_btnGenerateValidation = new QPushButton(tr("生成独立验证姿态"), validationGroup);
    m_btnCaptureValidation = new QPushButton(tr("采集当前姿态为验证集"), validationGroup);
    m_btnAutoAcquireValidation = new QPushButton(tr("自动移动 + 采集独立验证"), validationGroup);
    m_btnAutoAcquireValidation->setStyleSheet("QPushButton { background:#dff2ff; font-weight:bold; }");
    m_btnStopAutoAcquireValidation = new QPushButton(tr("停止自动验证采集"), validationGroup);
    m_btnStopAutoAcquireValidation->setEnabled(false);
    m_btnImportValidation = new QPushButton(tr("导入独立验证 CSV（离线）"), validationGroup);
    m_btnExportValidation = new QPushButton(tr("导出独立验证 CSV"), validationGroup);
    m_btnRunValidation = new QPushButton(tr("执行独立验证"), validationGroup);
    m_btnRunValidation->setStyleSheet("QPushButton { background:#d9f2d9; font-weight:bold; }");
    m_btnClearValidation = new QPushButton(tr("清空验证集"), validationGroup);
    validationButtons->addWidget(m_btnGenerateValidation, 0, 0);
    validationButtons->addWidget(m_btnCaptureValidation, 0, 1);
    validationButtons->addWidget(m_btnAutoAcquireValidation, 1, 0);
    validationButtons->addWidget(m_btnStopAutoAcquireValidation, 1, 1);
    validationButtons->addWidget(m_btnImportValidation, 2, 0);
    validationButtons->addWidget(m_btnExportValidation, 2, 1);
    validationButtons->addWidget(m_btnRunValidation, 3, 0);
    validationButtons->addWidget(m_btnClearValidation, 3, 1);
    validationLayout->addLayout(validationButtons);

    m_lblValidation = new QLabel(tr("独立验证：尚未执行。最终模型在通过hold-out验证前不会被视为可用于扫描/RTCP。"), validationGroup);
    m_lblValidation->setWordWrap(true);
    m_lblValidation->setStyleSheet("QLabel { color:#7a5200; font-weight:bold; }");
    validationLayout->addWidget(m_lblValidation);
    stage2PageLayout->addWidget(validationGroup);
    stage2PageLayout->addStretch(1);
}

void SingleSphere14CalibrationWidget::connectUi()
{
    connect(m_btnStartPreview, &QPushButton::clicked, this, &SingleSphere14CalibrationWidget::onStartPreview);
    connect(m_btnStopPreview, &QPushButton::clicked, this, &SingleSphere14CalibrationWidget::onStopPreview);
    connect(m_btnReadPose, &QPushButton::clicked, this, &SingleSphere14CalibrationWidget::onReadCurrentPose);
    connect(m_btnFit, &QPushButton::clicked, this, &SingleSphere14CalibrationWidget::onFitCurrentProfile);
    connect(m_btnTeach, &QPushButton::clicked, this, &SingleSphere14CalibrationWidget::onTeachReference);
    connect(m_btnBootstrapSolve, &QPushButton::clicked, this, &SingleSphere14CalibrationWidget::onSolveBootstrap);
    connect(m_btnGenerateStage1Candidates, &QPushButton::clicked, this, &SingleSphere14CalibrationWidget::onGenerateStage1Candidates);
    connect(m_btnMoveStage1Candidate, &QPushButton::clicked, this, &SingleSphere14CalibrationWidget::onMoveToSelectedStage1Candidate);
    connect(m_btnSkipStage1Candidate, &QPushButton::clicked, this, &SingleSphere14CalibrationWidget::onSkipSelectedStage1Candidate);
    connect(m_btnSaveStage1Result, &QPushButton::clicked, this, &SingleSphere14CalibrationWidget::onSaveStage1Result);
    connect(m_btnLoadStage1Result, &QPushButton::clicked, this, &SingleSphere14CalibrationWidget::onLoadStage1Result);
    connect(m_btnStage1Capture, &QPushButton::clicked, this, [this]() {
        if (m_hasBootstrap) {
            QMessageBox::information(this, tr("阶段1已经完成"),
                tr("阶段1已通过。当前应进入阶段2；若要补采阶段1人工数据，请重新开始阶段1。"));
            return;
        }
        onCaptureCurrentPose();
    });
    connect(m_btnImportStage2Sphere, &QPushButton::clicked, this, &SingleSphere14CalibrationWidget::onImportStage2SphereCloud);
    connect(m_btnFitStage2Sphere, &QPushButton::clicked, this, &SingleSphere14CalibrationWidget::onFitStage2SphereCloud);
    connect(m_btnGenerate, &QPushButton::clicked, this, &SingleSphere14CalibrationWidget::onGeneratePlan);
    connect(m_btnStage2MotionControls, &QPushButton::clicked, this, [this]() {
        refreshDeviceState();
        emit requestMotionControlView();
    });
    connect(m_btnCancelStage2Plan, &QPushButton::clicked, this, &SingleSphere14CalibrationWidget::onCancelStage2PlanGeneration);
    connect(m_btnStage2SimulationStart, &QPushButton::clicked, this, &SingleSphere14CalibrationWidget::onStartStage2Simulation);
    connect(m_btnStage2SimulationStop, &QPushButton::clicked, this, &SingleSphere14CalibrationWidget::onStopStage2Simulation);
    connect(m_btnStage2SimulationClear, &QPushButton::clicked, this, &SingleSphere14CalibrationWidget::onClearStage2Simulation);
    connect(m_btnAutoAcquireStage2, &QPushButton::clicked, this, &SingleSphere14CalibrationWidget::onStartAutomaticStage2Acquisition);
    connect(m_btnStopAutoAcquireStage2, &QPushButton::clicked, this, &SingleSphere14CalibrationWidget::onStopAutomaticStage2Acquisition);
    connect(m_btnGenerateValidation, &QPushButton::clicked, this, &SingleSphere14CalibrationWidget::onGenerateValidationPlan);
    connect(m_btnAutoAcquireValidation, &QPushButton::clicked, this, &SingleSphere14CalibrationWidget::onStartAutomaticValidationAcquisition);
    connect(m_btnStopAutoAcquireValidation, &QPushButton::clicked, this, &SingleSphere14CalibrationWidget::onStopAutomaticValidationAcquisition);
    connect(m_btnMove, &QPushButton::clicked, this, &SingleSphere14CalibrationWidget::onMoveToSelectedPose);
    connect(m_btnSkipPlanPose, &QPushButton::clicked, this, &SingleSphere14CalibrationWidget::onSkipSelectedPose);
    connect(m_btnRecenter, &QPushButton::clicked, this, &SingleSphere14CalibrationWidget::onRecommendRecenter);
    connect(m_btnCapture, &QPushButton::clicked, this, &SingleSphere14CalibrationWidget::onCaptureCurrentPose);
    connect(m_btnCaptureValidation, &QPushButton::clicked, this, &SingleSphere14CalibrationWidget::onCaptureValidationPose);
    connect(m_btnImportValidation, &QPushButton::clicked, this, &SingleSphere14CalibrationWidget::onImportValidationCsv);
    connect(m_btnExportValidation, &QPushButton::clicked, this, &SingleSphere14CalibrationWidget::onExportValidationCsv);
    connect(m_btnRunValidation, &QPushButton::clicked, this, &SingleSphere14CalibrationWidget::onRunValidation);
    connect(m_btnClearValidation, &QPushButton::clicked, this, &SingleSphere14CalibrationWidget::onClearValidation);
    auto validationCriteriaChanged = [this]() {
        if (!m_validationSummary.computed) return;
        m_validationSummary = ValidationSummary{};
        updateValidationSummary();
        appendLog(tr("独立验证判据已修改，旧通过/失败结论已失效；验证数据仍保留，请重新点击“执行独立验证”。"));
        updateCalibrationReadiness();
        updateWorkflowGuide();
    };
    connect(m_spinValidationPoseCount, &QSpinBox::valueChanged, this, [validationCriteriaChanged](int){ validationCriteriaChanged(); });
    connect(m_spinValidationBCSep, &QDoubleSpinBox::valueChanged, this, [validationCriteriaChanged](double){ validationCriteriaChanged(); });
    connect(m_checkValidationAvoidRejected, &QCheckBox::toggled, this, [validationCriteriaChanged](bool){ validationCriteriaChanged(); });
    connect(m_spinValidationHazardBCSep, &QDoubleSpinBox::valueChanged, this, [validationCriteriaChanged](double){ validationCriteriaChanged(); });
    connect(m_spinValidationRms, &QDoubleSpinBox::valueChanged, this, [validationCriteriaChanged](double){ validationCriteriaChanged(); });
    connect(m_spinValidationMax, &QDoubleSpinBox::valueChanged, this, [validationCriteriaChanged](double){ validationCriteriaChanged(); });
    connect(m_spinValidationCenterXz, &QDoubleSpinBox::valueChanged, this, [validationCriteriaChanged](double){ validationCriteriaChanged(); });
    connect(m_btnDeleteLast, &QPushButton::clicked, this, &SingleSphere14CalibrationWidget::onDeleteLastCapture);
    connect(m_btnClear, &QPushButton::clicked, this, &SingleSphere14CalibrationWidget::onClearSession);
    connect(m_btnSolve, &QPushButton::clicked, this, &SingleSphere14CalibrationWidget::onSolve);
    connect(m_btnCancelStage2Solve, &QPushButton::clicked, this, &SingleSphere14CalibrationWidget::onCancelStage2Solve);
    connect(m_stage2SolveTimer, &QTimer::timeout, this, [this]() {
        if (!m_stage2SolveRunning || !m_lblStage2SolveElapsed) return;
        const qint64 elapsed = QDateTime::currentMSecsSinceEpoch() - m_stage2SolveStartMs;
        m_lblStage2SolveElapsed->setText(tr("耗时：%1 s").arg(elapsed / 1000.0, 0, 'f', 1));
    });
    connect(m_btnSaveResult, &QPushButton::clicked, this, &SingleSphere14CalibrationWidget::onSaveResult);
    connect(m_btnLoadResult, &QPushButton::clicked, this, &SingleSphere14CalibrationWidget::onLoadResult);
    connect(m_btnBindLoadedResult, &QPushButton::clicked, this, &SingleSphere14CalibrationWidget::onBindLoadedResultToCurrentSession);
    connect(m_btnExportSession, &QPushButton::clicked, this, &SingleSphere14CalibrationWidget::onExportSessionCsv);
    connect(m_btnImportSession, &QPushButton::clicked, this, &SingleSphere14CalibrationWidget::onImportSessionCsv);
    auto stopCalibrationMotion = [this]() {
        m_autoPlanMove = false;
        m_autoPlanIndex = -1;
        m_autoStage2Acquisition = false;
        m_autoStage2StopRequested = true;
        m_autoValidationAcquisition = false;
        m_autoValidationStopRequested = true;
        if (m_btnStopAutoAcquireStage2) m_btnStopAutoAcquireStage2->setEnabled(false);
        if (m_btnAutoAcquireStage2) m_btnAutoAcquireStage2->setEnabled(true);
        if (m_btnStopAutoAcquireValidation) m_btnStopAutoAcquireValidation->setEnabled(false);
        if (m_btnAutoAcquireValidation) m_btnAutoAcquireValidation->setEnabled(true);
        emit requestJointStop(false);
    };
    connect(m_btnStopMove, &QPushButton::clicked, this, stopCalibrationMotion);
    connect(m_btnStage2StopMove, &QPushButton::clicked, this, stopCalibrationMotion);
    connect(m_planTable, &QTableWidget::itemSelectionChanged, this, &SingleSphere14CalibrationWidget::onPlanSelectionChanged);
}

void SingleSphere14CalibrationWidget::updateDeviceState()
{
    const QString controllerText = (m_controller && m_controller->isConnected()) ? tr("控制器已连接") : tr("控制器未连接");
    const QString sensorText = (m_sensor && m_sensor->isConnected()) ? tr("2950已连接") : tr("2950未连接");
    const QString streamText = (m_sensor && m_sensor->isProfileRunning()) ? tr("轮廓流运行中") : tr("轮廓流停止");
    m_lblDevice->setText(QStringLiteral("%1；%2；%3").arg(controllerText, sensorText, streamText));
    updateStage2MotionReadinessUi();
    updateStage1Status();
}

void SingleSphere14CalibrationWidget::onSensorProfile(const LaserProfile& profile)
{
    // This slot is subscribed only during deliberate fresh-profile capture.
    m_latestProfile = profile;
    m_hasLatestProfile = true;
    if (!m_collecting) return;
    m_captureFrames.push_back(profile);
    m_lblSolveState->setText((m_captureValidation ? tr("验证集采集中：%1/%2 fresh profiles")
                                                 : tr("采集中：%1/%2 fresh profiles"))
                                 .arg(m_captureFrames.size()).arg(m_spinCaptureFrames->value()));
    if (m_stage1CaptureProgress) {
        m_stage1CaptureProgress->setRange(0, m_spinCaptureFrames->value());
        m_stage1CaptureProgress->setValue(m_captureFrames.size());
        m_stage1CaptureProgress->setFormat(
            tr("fresh profiles %1/%2").arg(m_captureFrames.size()).arg(m_spinCaptureFrames->value()));
    }
    updateStage1Status();
    if (m_captureFrames.size() >= m_spinCaptureFrames->value()) {
        m_collecting = false;
        finishFreshProfileCollection();
    }
}

void SingleSphere14CalibrationWidget::onStartPreview()
{
    if (!m_sensor || !m_sensor->isConnected()) {
        QMessageBox::warning(this, tr("不能预览"), tr("请先连接 2950 传感器。"));
        return;
    }
    emit requestProfileView();
    emit requestProfilePreviewStart();
    appendLog(tr("请求启动单球标定轮廓：软件内部触发 + 全测量场预设(MEASURINGFIELD index=0)；主视图区切换到二维轮廓；候选 ROI 仅用于软件 RANSAC 提取。"));
    QTimer::singleShot(300, this, &SingleSphere14CalibrationWidget::updateDeviceState);
}

void SingleSphere14CalibrationWidget::onStopPreview()
{
    if (m_collecting) {
        QMessageBox::warning(this, tr("正在采集"), tr("当前正在采集 fresh profiles，请等待结束后再停止预览。"));
        return;
    }
    emit requestProfilePreviewStop();
    appendLog(tr("请求停止轮廓预览。"));
    QTimer::singleShot(300, this, &SingleSphere14CalibrationWidget::updateDeviceState);
}

bool SingleSphere14CalibrationWidget::readStableState(JointState5D& state, QString& error) const
{
    if (!m_controller || !m_controller->isConnected()) {
        error = tr("控制器未连接");
        return false;
    }
    if (!m_controller->readJointState5D(state, &error)) return false;
    if (!state.allIdle) {
        error = tr("X/Y/Z/C/B 尚未全部停止，禁止静态标定采样");
        return false;
    }
    return true;
}

bool SingleSphere14CalibrationWidget::checkStage2LiveMotionReadiness(
    QString& error, JointState5D* state) const
{
    error.clear();
    if (!m_controller || !m_controller->isConnected()) {
        error = tr("控制器未连接。请先连接控制器。");
        return false;
    }
    if (!m_controller->areLinearMposReferenced()) {
        error = tr("XYZ光栅MPOS机床基准尚未建立。请到“控制”页执行XYZ/五轴正规DATUM回零；仅清零显示值不能替代回零。");
        return false;
    }
    if (!m_controller->isRotaryAbsoluteReady(AxisC) ||
        !m_controller->isRotaryAbsoluteReady(AxisB)) {
        error = tr("B/C绝对机械坐标尚未就绪。请到“控制”页执行“重读 B/C 绝对位置”，等待两轴绝对坐标建立完成。");
        return false;
    }
    if (!m_controller->isRotaryServoEnabled(AxisC) ||
        !m_controller->isRotaryServoEnabled(AxisB)) {
        error = tr("B/C尚未完成人工使能。请到“控制”页分别人工使能 C 轴和 B 轴。");
        return false;
    }
    JointState5D local;
    if (!m_controller->readJointState5D(local, &error)) {
        if (error.isEmpty()) error = tr("读取五轴状态失败。");
        return false;
    }
    if (!local.allIdle) {
        error = tr("X/Y/Z/C/B 尚未全部停止。请等待全部轴停稳后再执行阶段2真实运动。");
        return false;
    }
    if (state) *state = local;
    return true;
}

void SingleSphere14CalibrationWidget::updateStage2MotionReadinessUi()
{
    if (!m_lblStage2MotionReadiness) return;
    if (!m_controller || !m_controller->isConnected()) {
        m_lblStage2MotionReadiness->setText(tr("运动准备：控制器未连接"));
        m_lblStage2MotionReadiness->setStyleSheet("QLabel { color:#a00000; font-weight:bold; }");
        return;
    }
    const bool xyzRef = m_controller->areLinearMposReferenced();
    const bool cAbs = m_controller->isRotaryAbsoluteReady(AxisC);
    const bool bAbs = m_controller->isRotaryAbsoluteReady(AxisB);
    const bool cServo = m_controller->isRotaryServoEnabled(AxisC);
    const bool bServo = m_controller->isRotaryServoEnabled(AxisB);
    const bool comp = m_controller->axisCompensationEnabled();
    const bool ready = xyzRef && cAbs && bAbs && cServo && bServo;
    m_lblStage2MotionReadiness->setText(tr(
        "运动准备：XYZ DATUM=%1；C绝对=%2 / 使能=%3；B绝对=%4 / 使能=%5；XYZ补偿=%6。%7")
        .arg(xyzRef ? tr("OK") : tr("未建立"))
        .arg(cAbs ? tr("OK") : tr("未就绪"))
        .arg(cServo ? tr("ON") : tr("OFF"))
        .arg(bAbs ? tr("OK") : tr("未就绪"))
        .arg(bServo ? tr("ON") : tr("OFF"))
        .arg(comp ? tr("ON") : tr("OFF"))
        .arg(ready ? tr("可以进入执行前MachineSafety门禁。")
                   : tr("真实运动未授权；请使用右侧按钮恢复缺失条件。")));
    m_lblStage2MotionReadiness->setStyleSheet(
        ready ? "QLabel { color:#087a28; font-weight:bold; }"
              : "QLabel { color:#a05a00; font-weight:bold; }");
}

bool SingleSphere14CalibrationWidget::readPoseSourceMask(std::uint8_t& feedbackMask,
                                                               QString& error) const
{
    // Static calibration uses the same measured-pose contract as software sampling:
    // XYZ are incremental-grating MPOS and C/B are mechanical MPOS. DPOS remains a
    // demand/diagnostic coordinate only.
    if (!m_controller || !m_controller->isConnected()) {
        error = tr("控制器未连接");
        return false;
    }
    static_assert(mls::runtime::Table5D::softwareSampleFeedbackMask == 31,
                  "Physical17 calibration requires XYZ/C/B MPOS");
    static_assert(mls::runtime::Table5D::softwareSampleCommandMask == 0,
                  "Measured calibration pose must not contain command coordinates");
    for (int axis = AxisX; axis <= AxisB; ++axis) {
        int atype = 0;
        if (!m_controller->readAxisType(axis, atype, &error)) return false;
        if (atype != 4) {
            error = tr("%1轴ATYPE=%2；当前Physical17要求XYZ/C/B全部为ATYPE=4反馈轴")
                .arg(axis == AxisX ? QStringLiteral("X") : axis == AxisY ? QStringLiteral("Y") :
                     axis == AxisZ ? QStringLiteral("Z") : axis == AxisC ? QStringLiteral("C") : QStringLiteral("B"))
                .arg(atype);
            return false;
        }
    }
    feedbackMask = static_cast<std::uint8_t>(mls::runtime::Table5D::softwareSampleFeedbackMask);
    return true;
}

JointPose5D SingleSphere14CalibrationWidget::calibrationPoseFromState(
    const JointState5D& state, std::uint8_t feedbackMask) const
{
    JointPose5D q;
    const JointPose5D& d = state.dpos;
    const JointPose5D& m = state.mpos;
    q.x = (feedbackMask & (1u << feedbackBitForAxis(AxisX))) ? m.x : d.x;
    q.y = (feedbackMask & (1u << feedbackBitForAxis(AxisY))) ? m.y : d.y;
    q.z = (feedbackMask & (1u << feedbackBitForAxis(AxisZ))) ? m.z : d.z;
    q.c = (feedbackMask & (1u << feedbackBitForAxis(AxisC))) ? m.c : d.c;
    q.b = (feedbackMask & (1u << feedbackBitForAxis(AxisB))) ? m.b : d.b;
    return q;
}

JointPose5D SingleSphere14CalibrationWidget::averagedCalibrationPose(
    const JointState5D& a, const JointState5D& b, std::uint8_t feedbackMask) const
{
    const JointPose5D qa = calibrationPoseFromState(a, feedbackMask);
    const JointPose5D qb = calibrationPoseFromState(b, feedbackMask);
    JointPose5D q;
    q.x = 0.5 * (qa.x + qb.x); q.y = 0.5 * (qa.y + qb.y); q.z = 0.5 * (qa.z + qb.z);
    q.c = 0.5 * (qa.c + qb.c); q.b = 0.5 * (qa.b + qb.b);
    return q;
}

bool SingleSphere14CalibrationWidget::validateSessionConventions(QString& error) const
{
    if (!m_referenceTaught) return true;

    std::uint8_t currentMask = 0;
    if (!readPoseSourceMask(currentMask, error)) return false;
    if (currentMask != m_referenceFeedbackMask) {
        error = tr("轴数据源在标定会话中发生变化：参考时 %1，当前 %2。请清空会话并重新示教。")
                    .arg(sourceMaskText(m_referenceFeedbackMask), sourceMaskText(currentMask));
        return false;
    }

    if (m_sensorConventionLocked && (!m_sensor || !m_sensor->isConnected())) {
        error = tr("2950未连接，无法确认保存的FlipX/FlipZ坐标约定；当前标定禁止用于运行。");
        return false;
    }
    if (m_sensorConventionLocked) {
        SensorAdvancedParams adv;
        QString sensorError;
        if (!m_sensor->readAdvancedParams(adv, &sensorError)) {
            error = tr("无法核对传感器 Flip X/Flip Z：%1").arg(sensorError);
            return false;
        }
        if (adv.flipX != m_referenceFlipX || adv.flipZ != m_referenceFlipZ) {
            error = tr("传感器坐标翻转在标定会话中改变：参考时 FlipX=%1 FlipZ=%2，当前 FlipX=%3 FlipZ=%4。"
                       "已有数据不能混用，请恢复原设置或清空会话重新标定。")
                    .arg(m_referenceFlipX).arg(m_referenceFlipZ).arg(adv.flipX).arg(adv.flipZ);
            return false;
        }
    }
    return true;
}

bool SingleSphere14CalibrationWidget::selectedPoseBCReached(
    const JointPose5D& current, int planIndex, double toleranceDeg) const
{
    if (planIndex < 0 || planIndex >= static_cast<int>(m_plan.size())) return false;
    const auto& target = m_plan[static_cast<std::size_t>(planIndex)].predictedJoint;
    return std::abs(current.b - target.b) <= toleranceDeg &&
           std::abs(current.c - target.c) <= toleranceDeg;
}

void SingleSphere14CalibrationWidget::onReadCurrentPose()
{
    JointState5D state;
    QString error;
    if (!readStableState(state, error)) {
        QMessageBox::warning(this, tr("读取五轴状态失败"), error);
        return;
    }
    updateCurrentPoseLabel(&state);
}

void SingleSphere14CalibrationWidget::updateCurrentPoseLabel(const JointState5D* state)
{
    if (!state) {
        m_lblCurrentPose->setText(tr("当前五轴：未读取"));
        return;
    }
    QString sourceText;
    if (m_referenceTaught) {
        sourceText = sourceMaskText(m_referenceFeedbackMask);
    } else {
        std::uint8_t mask = 0;
        QString error;
        sourceText = readPoseSourceMask(mask, error) ? sourceMaskText(mask) : tr("数据源未确认");
    }
    m_lblCurrentPose->setText(tr("MPOS：%1；DPOS：%2；%3；标定采用：%4")
        .arg(poseText(state->mpos), poseText(state->dpos),
             state->allIdle ? tr("Idle") : tr("运动中"), sourceText));
}

std::vector<SensorProfilePoint> SingleSphere14CalibrationWidget::filteredPoints(
    const LaserProfile& profile, QVector<QPointF>* overlay) const
{
    std::vector<SensorProfilePoint> out;
    const int count = std::min(profile.x.size(), profile.z.size());
    out.reserve(static_cast<std::size_t>(count));
    if (overlay) overlay->clear();
    const double xmin = std::min(m_spinRoiXMin->value(), m_spinRoiXMax->value());
    const double xmax = std::max(m_spinRoiXMin->value(), m_spinRoiXMax->value());
    const double zmin = std::min(m_spinRoiZMin->value(), m_spinRoiZMax->value());
    const double zmax = std::max(m_spinRoiZMin->value(), m_spinRoiZMax->value());
    for (int i = 0; i < count; ++i) {
        const double x = profile.x[i];
        const double z = profile.z[i];
        if (!SensorUtils::isValidPoint(x, z)) continue;
        if (x < xmin || x > xmax || z < zmin || z > zmax) continue;
        out.push_back({x, z, 1.0});
        if (overlay) overlay->push_back(QPointF(x, z));
    }
    return out;
}

SphereCenterFit SingleSphere14CalibrationWidget::fitProfile(const LaserProfile& profile,
                                                             QVector<QPointF>* overlay) const
{
    const auto points = filteredPoints(profile, overlay);
    std::vector<std::size_t> inlierIndices;
    // 单帧XZ轮廓只提供 |Y_S|。阶段1/阶段2联合求解均不要求操作者
    // 选择Y正负，因此这里固定返回 +|Y_S| 作为“幅值容器”；符号不进入
    // 原始球面残差。阶段1求解器内部会把每帧±Y作为隐变量初始化。
    const auto fit = SingleSphere17Calibrator::fitSphereCenterInSensor(
        points, m_spinSphereDiameter->value(), +1,
        ransacOptionsFromUi(), &inlierIndices);
    if (overlay) {
        overlay->clear();
        overlay->reserve(static_cast<int>(inlierIndices.size()));
        for (std::size_t index : inlierIndices) {
            if (index < points.size()) overlay->push_back(QPointF(points[index].x, points[index].z));
        }
    }
    return fit;
}

SphereArcRansacOptions SingleSphere14CalibrationWidget::ransacOptionsFromUi() const
{
    SphereArcRansacOptions o;
    o.maxIterations = m_spinRansacIterations ? m_spinRansacIterations->value() : 350;
    o.inlierThresholdMm = m_spinRansacThreshold ? m_spinRansacThreshold->value() : 0.050;
    o.minInlierRatio = m_spinRansacMinRatio ? m_spinRansacMinRatio->value() : 0.20;
    o.minArcSpanDeg = m_spinRansacMinArc ? m_spinRansacMinArc->value() : 30.0;
    o.minInliers = 10;
    return o;
}

void SingleSphere14CalibrationWidget::onFitCurrentProfile()
{
    if (m_sensor) {
        LaserProfile latest;
        if (m_sensor->latestProfile(latest)) {
            m_latestProfile = latest;
            m_hasLatestProfile = true;
        }
    }
    if (!m_hasLatestProfile) {
        QMessageBox::information(this, tr("没有轮廓"), tr("请先启动轮廓预览并让标准球进入视场。"));
        return;
    }
    QVector<QPointF> overlay;
    const auto fit = fitProfile(m_latestProfile, &overlay);
    m_latestFit = fit;
    m_hasLatestFit = fit.success;
    if (!fit.success) {
        m_lblFit->setText(tr("球拟合失败：%1").arg(QString::fromStdString(fit.message)));
        emit requestProfileOverlayClear();
        return;
    }
    // Automatic Stage-2 fitting is a background quality gate. Overlay updates
    // must not force the operator away from the 3-D/other active view.
    if (!m_autoStage2Acquisition && !m_autoValidationAcquisition) emit requestProfileView();
    emit requestProfileOverlay(overlay, fit.centerSensorMm.x, fit.centerSensorMm.z, fit.sectionRadiusMm);
    const bool quality = fit.circleRmsMm <= m_spinMaxCircleRms->value();
    const double absY = std::abs(fit.centerSensorMm.y);
    const double sphereRadius = 0.5 * m_spinSphereDiameter->value();
    const double radiusDeficit = sphereRadius - fit.sectionRadiusMm;
    m_lblFit->setText(tr("RANSAC球弧：内点=%1/%2 (%3%)；覆盖=%4°；圆心XZ=[%5,%6] mm；"
                         "截面半径=%7 mm；|Y_S|=%8 mm；内点RMS=%9 mm；%10\n"
                         "说明：此处只显示|Y_S|。右手传感器坐标系的Y法向由当前X/Z方向共同决定；阶段1自动处理每帧±Y，不要求人工选择。")
        .arg(fit.inlierPointCount).arg(fit.candidatePointCount).arg(fit.inlierRatio*100.0,0,'f',1)
        .arg(fit.arcSpanDeg,0,'f',1)
        .arg(fit.centerSensorMm.x,0,'f',4).arg(fit.centerSensorMm.z,0,'f',4)
        .arg(fit.sectionRadiusMm,0,'f',4).arg(absY,0,'f',4)
        .arg(fit.circleRmsMm,0,'f',5)
        .arg(quality ? tr("质量通过") : tr("RMS超阈值")));
    Q_UNUSED(radiusDeficit);
    if (quality) {
        // During B/C transit, merely "seeing the sphere" is not sufficient.
        // Keep it inside a conservative central window so the next <=5 deg
        // rotary step cannot silently push the arc out of the LLT field.
        const int planIndex = selectedPlanIndex();
        bool needsTransitCentering = false;
        if (planIndex >= 0 && m_referenceTaught && m_controller && m_controller->isConnected()) {
            JointState5D state;
            QString stateError;
            if (readStableState(state, stateError)) {
                const JointPose5D current = calibrationPoseFromState(state, m_referenceFeedbackMask);
                if (!selectedPoseBCReached(current, planIndex)) {
                    needsTransitCentering = std::abs(fit.centerSensorMm.x) > 8.0
                        || fit.centerSensorMm.z < 85.0 || fit.centerSensorMm.z > 105.0;
                }
            }
        }
        m_needFitBeforeTransit = needsTransitCentering;
        if (needsTransitCentering) {
            m_lblFit->setText(m_lblFit->text() + tr("\n球仍可见，但已靠近首次标定的安全中央窗口边缘。继续旋转前请先点“XYZ小步保持/找球”，把球移回X≈0、Z≈95 mm附近。"));
        }
    }
    updateWorkflowGuide();
    updateStage1Status();
}

void SingleSphere14CalibrationWidget::onTeachReference()
{
    // A previous Physical17 model is still useful after the XYZ source upgrade: it may
    // be less accurate in the new MPOS frame, but its sphere center/hand-eye estimate
    // is a valuable navigation seed. Keep the persisted accepted result as fallback
    // and reuse its numeric estimate while the new calibration is being collected.
    const SingleSphere17Result* previousModel =
        (m_hasResult && m_lastResult.success) ? &m_lastResult
        : (m_hasFallbackResult && m_fallbackResult.success) ? &m_fallbackResult
        : nullptr;
    const bool havePreviousSeed = previousModel != nullptr;
    const Physical17Parameters previousSeedParameters = havePreviousSeed
        ? previousModel->parameters : Physical17Parameters{};
    const Vec3d previousSeedCenter = havePreviousSeed
        ? previousModel->sphereCenterMachineMm : Vec3d{};

    // Preserve the last successful result separately before clearing the in-progress
    // session. If it was independently validated it remains available to scan/RTCP
    // as a clearly marked fallback until a new MPOS result is validated.
    if (m_hasResult && m_lastResult.success) {
        m_fallbackResult = m_lastResult;
        m_fallbackValidationSummary = m_validationSummary;
        m_hasFallbackResult = true;
    }

    if (!m_records.empty() || !m_plan.empty() || m_hasBootstrap || m_hasResult) {
        const auto answer = QMessageBox::question(
            this, tr("重新开始阶段1"),
            tr("重新开始阶段1会清空当前人工采集、阶段1手眼/球心结果、阶段2计划和最终17参数结果。\n\n继续？"),
            QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
        if (answer != QMessageBox::Yes) return;
        m_records.clear();
        m_validationRecords.clear();
        m_validationSummary = ValidationSummary{};
        m_plan.clear();
        m_planValidation = false;
        m_planTable->setRowCount(0);
        m_calibrator = SingleSphere17Calibrator(solverOptionsFromUi());
        m_hasBootstrap = false;
        m_bootstrapRecordCount = 0;
        m_bootstrapResult = NominalHandEyeSphereResult{};
        m_hasResult = false;
        m_currentEstimate = previousSeedParameters;
        m_lastResult = SingleSphere17Result{};
        m_stage1Candidates.clear();
        m_loadedStage1CapturedPoses.clear();
        m_stage1CandidateStatus.clear();
        m_activeStage1Candidate = -1;
        if (m_stage1CandidateTable) m_stage1CandidateTable->setRowCount(0);
        m_stage1SolveElapsedMs = 0;
        updateStage1ResultDisplay();
    }

    JointState5D state;
    QString error;
    if (!readStableState(state, error)) {
        QMessageBox::warning(this, tr("阶段1无法开始"), error);
        return;
    }

    std::uint8_t feedbackMask = 0;
    if (!readPoseSourceMask(feedbackMask, error)) {
        QMessageBox::warning(this, tr("阶段1无法开始"),
            tr("无法确定XYZCB应使用DPOS还是MPOS：%1").arg(error));
        return;
    }

    SensorAdvancedParams adv;
    if (!m_sensor || !m_sensor->isConnected()) {
        QMessageBox::warning(this, tr("阶段1无法开始"), tr("请先连接2950传感器。"));
        return;
    }
    QString sensorError;
    if (!m_sensor->readAdvancedParams(adv, &sensorError)) {
        QMessageBox::warning(this, tr("阶段1无法开始"),
            tr("无法读取传感器 Flip X/Flip Z，不能锁定标定坐标约定：%1").arg(sensorError));
        return;
    }

    // Do not revoke the previous accepted calibration merely because XYZ changed
    // from DPOS to grating MPOS. The old model remains a usable approximate fallback
    // and a seed for sphere-center/path planning. A newly validated calibration will
    // replace it only after the normal save/accept workflow completes.
    if (havePreviousSeed) {
        appendLog(tr("已开始新的MPOS标定会话；旧标定结果继续保留并作为初始模型使用（球心=%1）。建议完成新标定以获得更高精度。")
            .arg(centerText(previousSeedCenter)));
    } else {
        appendLog(tr("已开始新的MPOS标定会话；当前没有可复用的旧17参数结果。"));
    }

    m_referenceFlipX = adv.flipX;
    m_referenceFlipZ = adv.flipZ;
    m_sensorConventionLocked = true;
    m_referenceFeedbackMask = feedbackMask;
    m_modelFeedbackMask = feedbackMask;
    m_referenceTaught = true;        // 历史变量名保留：现在表示“阶段1会话已锁定”。
    m_referenceCaptured = false;
    m_planBootstrap = false;
    m_fullPlanGenerated = false;
    m_planValidation = false;
    m_refinementSolved = false;
    m_needFitBeforeTransit = false;
    m_pendingMoveKind = PendingMoveKind::None;
    m_pendingMovePlanIndex = -1;
    m_autoPlanMove = false;
    m_autoPlanIndex = -1;
    m_moveInProgress = false;

    m_stage1Candidates.clear();
    m_loadedStage1CapturedPoses.clear();
    m_stage1CandidateStatus.clear();
    m_activeStage1Candidate = -1;
    if (m_stage1CandidateTable) m_stage1CandidateTable->setRowCount(0);

    const JointPose5D calibrationPose = calibrationPoseFromState(state, feedbackMask);
    m_referenceSeed = PosePlannerSeed{};
    m_referenceSeed.taughtJoint = SingleSphere17QtAdapter::toPortable(calibrationPose);
    m_currentEstimate = previousSeedParameters;
    m_sphereCenterGuessMachine = previousSeedCenter;
    m_referenceSeed.hasSphereCenterMachine = havePreviousSeed;
    m_referenceSeed.sphereCenterMachineMm = previousSeedCenter;
    m_referenceSeed.taughtSphereCenterSensorMm = havePreviousSeed
        ? SingleSphere17Model::machinePointInSensor(
              m_referenceSeed.taughtJoint, previousSeedCenter, previousSeedParameters)
        : Vec3d{};

    // 处理后的传感器坐标必须解释成右手系。若只翻转X或只翻转Z，
    // Y法向相对原始传感器坐标也必须翻转；若X/Z同时翻转则Y不变。
    const int yParity = (m_referenceFlipX != m_referenceFlipZ) ? -1 : +1;
    m_lblConvention->setText(tr(
        "阶段1坐标约定已锁定：FlipX=%1，FlipZ=%2；%3。\n"
        "右手系定义采用 Y_S = Z_S × X_S，因此相对未翻转原始系的Y法向倍率=%4。"
        "单帧轮廓只显示|Y_S|，不再要求人工选择±Y。")
        .arg(m_referenceFlipX).arg(m_referenceFlipZ)
        .arg(sourceMaskText(m_referenceFeedbackMask)).arg(yParity));

    updateCurrentPoseLabel(&state);
    m_lblBootstrap->setText(tr("阶段1已开始：请人工移动XYZ/B/C，让球在多个静态姿态进入视场；每个姿态先确认拟合质量，再点击“采集当前姿态”。"));
    m_lblSolveState->setText(tr("阶段1采集中；尚未建立17参数计划。"));
    appendLog(tr("阶段1开始：锁定 FlipX=%1 FlipZ=%2，Y法向倍率=%3；姿态数据源：%4。阶段1不人工选择球心Y正负。")
              .arg(m_referenceFlipX).arg(m_referenceFlipZ).arg(yParity)
              .arg(sourceMaskText(m_referenceFeedbackMask)));
    updateSessionSummary();
    updateCalibrationReadiness();
    updateWorkflowGuide();
    updateStage1Status();
}

void SingleSphere14CalibrationWidget::updateStage1ResultDisplay()
{
    if (!m_lblStage1ResultSummary || !m_stage1ResultTable) return;
    const QStringList names = {
        tr("HandEye Rx"), tr("HandEye Ry"), tr("HandEye Rz"),
        tr("HandEye Tx"), tr("HandEye Ty"), tr("HandEye Tz"),
        tr("Sphere Mx"), tr("Sphere My"), tr("Sphere Mz")
    };
    const QStringList units = {tr("rad"),tr("rad"),tr("rad"),tr("mm"),tr("mm"),tr("mm"),tr("mm"),tr("mm"),tr("mm")};
    std::array<double,9> values{};
    if (m_hasBootstrap) {
        const auto& p = m_bootstrapResult.parameters;
        values = {{
            p[Physical17Parameters::HandEyeRx], p[Physical17Parameters::HandEyeRy], p[Physical17Parameters::HandEyeRz],
            p[Physical17Parameters::HandEyeTx], p[Physical17Parameters::HandEyeTy], p[Physical17Parameters::HandEyeTz],
            m_bootstrapResult.sphereCenterMachineMm.x, m_bootstrapResult.sphereCenterMachineMm.y, m_bootstrapResult.sphereCenterMachineMm.z
        }};
    }
    for (int row=0; row<9; ++row) {
        m_stage1ResultTable->setItem(row,0,new QTableWidgetItem(names[row]));
        m_stage1ResultTable->setItem(row,1,new QTableWidgetItem(m_hasBootstrap ? QString::number(values[static_cast<std::size_t>(row)],'g',10) : QStringLiteral("--")));
        m_stage1ResultTable->setItem(row,2,new QTableWidgetItem(units[row]));
    }
    if (m_hasBootstrap) {
        m_lblStage1ResultSummary->setText(tr(
            "阶段1通过：rank=%1/9；球面RMS=%2 mm；max=%3 mm；condition=%4；求解耗时=%5 s；代表profiles=%6；代表球面点=%7。\n"
            "该结果用于阶段2导航/初始化；最终几何精度仍以17参数求解 + 独立验证为准。")
            .arg(m_bootstrapResult.stateRank)
            .arg(m_bootstrapResult.residualRmsMm,0,'g',6)
            .arg(m_bootstrapResult.residualMaxAbsMm,0,'g',6)
            .arg(m_bootstrapResult.conditionNumber,0,'g',6)
            .arg(m_stage1SolveElapsedMs/1000.0,0,'f',2)
            .arg(static_cast<qulonglong>(m_bootstrapResult.profileCount))
            .arg(static_cast<qulonglong>(m_bootstrapResult.pointCount)));
        m_lblStage1ResultSummary->setStyleSheet("QLabel { background:#e8f7e8; color:#185c18; padding:5px; border:1px solid #6abf69; font-weight:bold; }");
    } else {
        m_lblStage1ResultSummary->setText(tr("尚无通过的阶段1结果。求解成功后，手眼6DoF、固定球心、rank/RMS/condition和耗时会固定显示在这里。"));
        m_lblStage1ResultSummary->setStyleSheet("QLabel { background:#f7fbff; padding:5px; border:1px solid #9cb4c8; }");
    }
    if (m_btnSaveStage1Result) m_btnSaveStage1Result->setEnabled(m_hasBootstrap);
}

bool SingleSphere14CalibrationWidget::writeStage1ResultJson(const QString& fileName, QString& error) const
{
    if (!m_hasBootstrap) { error=tr("没有可保存的阶段1结果。"); return false; }
    QJsonObject root;
    root.insert("model", QStringLiteral("single_sphere_stage1_handeye_sphere_v1"));
    root.insert("saved_at", QDateTime::currentDateTime().toString(Qt::ISODate));
    root.insert("success", m_bootstrapResult.success);
    root.insert("rank", m_bootstrapResult.stateRank);
    root.insert("expected_rank", m_bootstrapResult.expectedStateRank);
    root.insert("residual_rms_mm", m_bootstrapResult.residualRmsMm);
    root.insert("residual_max_abs_mm", m_bootstrapResult.residualMaxAbsMm);
    root.insert("center_init_rms_mm", m_bootstrapResult.centerInitRmsMm);
    root.insert("condition_number", m_bootstrapResult.conditionNumber);
    root.insert("solve_elapsed_ms", static_cast<double>(m_stage1SolveElapsedMs));
    root.insert("representative_profile_count", static_cast<double>(m_bootstrapResult.profileCount));
    root.insert("representative_point_count", static_cast<double>(m_bootstrapResult.pointCount));
    root.insert("stage1_pose_count", m_bootstrapRecordCount);
    QJsonObject convention;
    convention.insert("flip_x", m_referenceFlipX);
    convention.insert("flip_z", m_referenceFlipZ);
    convention.insert("feedback_mask", static_cast<int>(m_referenceFeedbackMask));
    convention.insert("feedback_mask_encoding", QStringLiteral("XYZCB_bit0_to_bit4"));
    convention.insert("pose_sources", sourceMaskText(m_referenceFeedbackMask));
    root.insert("coordinate_convention", convention);
    const auto& hp=m_bootstrapResult.parameters;
    QJsonObject handeye;
    handeye.insert("rx_rad",hp[Physical17Parameters::HandEyeRx]);
    handeye.insert("ry_rad",hp[Physical17Parameters::HandEyeRy]);
    handeye.insert("rz_rad",hp[Physical17Parameters::HandEyeRz]);
    handeye.insert("tx_mm",hp[Physical17Parameters::HandEyeTx]);
    handeye.insert("ty_mm",hp[Physical17Parameters::HandEyeTy]);
    handeye.insert("tz_mm",hp[Physical17Parameters::HandEyeTz]);
    root.insert("hand_eye",handeye);
    QJsonArray center;
    center.append(m_bootstrapResult.sphereCenterMachineMm.x);
    center.append(m_bootstrapResult.sphereCenterMachineMm.y);
    center.append(m_bootstrapResult.sphereCenterMachineMm.z);
    root.insert("sphere_center_machine_mm",center);
    QJsonArray sv; for(double v:m_bootstrapResult.singularValues) sv.append(v); root.insert("singular_values",sv);
    QJsonArray poses;
    for (const auto& r:m_records) {
        if (!r.stage1Bootstrap) continue;
        QJsonObject o;
        o.insert("x_mm",r.usedPose.x); o.insert("y_mm",r.usedPose.y); o.insert("z_mm",r.usedPose.z);
        o.insert("c_deg",r.usedPose.c); o.insert("b_deg",r.usedPose.b);
        o.insert("valid_profiles",static_cast<int>(r.observations.size()));
        poses.append(o);
    }
    root.insert("stage1_captured_poses",poses);
    QFile f(fileName);
    if(!f.open(QIODevice::WriteOnly|QIODevice::Truncate)){error=f.errorString();return false;}
    if(f.write(QJsonDocument(root).toJson(QJsonDocument::Indented))<0){error=f.errorString();return false;}
    return true;
}

void SingleSphere14CalibrationWidget::onSaveStage1Result()
{
    if (!m_hasBootstrap) { QMessageBox::information(this,tr("无阶段1结果"),tr("请先完成阶段1求解。")); return; }
    const QString fn=QFileDialog::getSaveFileName(this,tr("保存阶段1初步手眼结果"),QStringLiteral("stage1_handeye_sphere_result.json"),tr("JSON (*.json)"));
    if(fn.isEmpty())return;
    QString error;
    if(!writeStage1ResultJson(fn,error)) QMessageBox::warning(this,tr("保存失败"),error);
    else appendLog(tr("已保存阶段1初步手眼/固定球心结果：%1").arg(fn));
}


bool SingleSphere14CalibrationWidget::loadStage1ResultJson(
    const QString& fileName,
    NominalHandEyeSphereResult& result,
    std::vector<JointPose5D>& capturedPoses,
    qint64& solveElapsedMs,
    bool& flipX, bool& flipZ, std::uint8_t& feedbackMask,
    QString& error) const
{
    QFile f(fileName);
    if (!f.open(QIODevice::ReadOnly)) { error = f.errorString(); return false; }
    QJsonParseError pe;
    const auto doc = QJsonDocument::fromJson(f.readAll(), &pe);
    if (pe.error != QJsonParseError::NoError || !doc.isObject()) {
        error = tr("阶段1 JSON解析失败：%1").arg(pe.errorString());
        return false;
    }
    const auto root = doc.object();
    if (root.value("model").toString() != QStringLiteral("single_sphere_stage1_handeye_sphere_v1")) {
        error = tr("不是 single_sphere_stage1_handeye_sphere_v1 阶段1结果文件。");
        return false;
    }

    // Stage-1 observability is fixed by this model: importing JSON must not be
    // allowed to weaken the rank gate by supplying expected_rank=0 (or by
    // omitting/mistyping a metric and letting QJsonValue::toDouble() become 0).
    if(!root.value("success").isBool() || !root.value("success").toBool()){
        error=tr("阶段1 JSON 未声明成功的阶段1求解；禁止作为已完成标定来源。");
        return false;
    }
    if(!root.value("rank").isDouble()){
        error=tr("阶段1 JSON 缺少有效 rank。");
        return false;
    }
    if(root.contains("expected_rank")
        && (!root.value("expected_rank").isDouble() || root.value("expected_rank").toInt(-1)!=9)){
        error=tr("阶段1 JSON 的 expected_rank 与当前固定模型(rank=9)不一致。");
        return false;
    }
    const QStringList qualityKeys={QStringLiteral("residual_rms_mm"),
        QStringLiteral("residual_max_abs_mm"),QStringLiteral("center_init_rms_mm"),
        QStringLiteral("condition_number"),QStringLiteral("representative_profile_count"),
        QStringLiteral("representative_point_count"),QStringLiteral("solve_elapsed_ms"),
        QStringLiteral("stage1_pose_count")};
    QMap<QString,double> quality;
    for(const QString& key:qualityKeys){
        double value=0.0;
        if(!finiteJsonNumber(root,key,value)){
            error=tr("阶段1 JSON 的 %1 缺失、类型错误或不是有限数。").arg(key);
            return false;
        }
        quality.insert(key,value);
    }
    const double profileCountValue=quality.value(QStringLiteral("representative_profile_count"));
    const double pointCountValue=quality.value(QStringLiteral("representative_point_count"));
    const double elapsedValue=quality.value(QStringLiteral("solve_elapsed_ms"));
    const double poseCountValue=quality.value(QStringLiteral("stage1_pose_count"));
    const auto isSafeInteger=[](double value,double minimum,double maximum){
        return value>=minimum && value<=maximum && std::floor(value)==value;
    };
    // Keep imported counters comfortably inside exact-integer double range before
    // converting to C++ integer types; corrupted JSON must never reach an
    // out-of-range floating-to-integer conversion.
    constexpr double kMaxExactCounter=9.0e15;
    if(!isSafeInteger(profileCountValue,1.0,kMaxExactCounter)
        || !isSafeInteger(pointCountValue,1.0,kMaxExactCounter)
        || !isSafeInteger(elapsedValue,0.0,kMaxExactCounter)
        || !isSafeInteger(poseCountValue,0.0,static_cast<double>(std::numeric_limits<int>::max()))){
        error=tr("阶段1 JSON 的样本计数/求解时长字段必须是安全范围内的整数。");
        return false;
    }

    result = NominalHandEyeSphereResult{};
    result.stateRank = root.value("rank").toInt(-1);
    result.expectedStateRank = 9;
    result.residualRmsMm = quality.value(QStringLiteral("residual_rms_mm"));
    result.residualMaxAbsMm = quality.value(QStringLiteral("residual_max_abs_mm"));
    result.centerInitRmsMm = quality.value(QStringLiteral("center_init_rms_mm"));
    result.conditionNumber = quality.value(QStringLiteral("condition_number"));
    result.profileCount = static_cast<std::size_t>(profileCountValue);
    result.pointCount = static_cast<std::size_t>(pointCountValue);
    solveElapsedMs = static_cast<qint64>(elapsedValue);

    const auto handEye = root.value("hand_eye").toObject();
    const QStringList handEyeKeys={QStringLiteral("rx_rad"),QStringLiteral("ry_rad"),QStringLiteral("rz_rad"),
                                   QStringLiteral("tx_mm"),QStringLiteral("ty_mm"),QStringLiteral("tz_mm")};
    for(const QString& key:handEyeKeys){
        if(!handEye.contains(key) || !handEye.value(key).isDouble()){
            error=tr("阶段1 JSON缺少或损坏 hand_eye.%1。").arg(key);
            return false;
        }
    }
    result.parameters[Physical17Parameters::HandEyeRx] = handEye.value("rx_rad").toDouble();
    result.parameters[Physical17Parameters::HandEyeRy] = handEye.value("ry_rad").toDouble();
    result.parameters[Physical17Parameters::HandEyeRz] = handEye.value("rz_rad").toDouble();
    result.parameters[Physical17Parameters::HandEyeTx] = handEye.value("tx_mm").toDouble();
    result.parameters[Physical17Parameters::HandEyeTy] = handEye.value("ty_mm").toDouble();
    result.parameters[Physical17Parameters::HandEyeTz] = handEye.value("tz_mm").toDouble();
    for(std::size_t i=Physical17Parameters::HandEyeRx;i<=Physical17Parameters::HandEyeTz;++i){
        if(!std::isfinite(result.parameters[i])){
            error=tr("阶段1 JSON 的 hand_eye 含非有限数值。");
            return false;
        }
    }

    if (!finiteJsonVec3(root.value("sphere_center_machine_mm"), result.sphereCenterMachineMm)) {
        error = tr("阶段1 JSON缺少有效的3D固定球心，三个分量必须都是有限数值。");
        return false;
    }

    const auto singular = root.value("singular_values").toArray();
    for (int i = 0; i < singular.size() && i < 9; ++i)
        result.singularValues[static_cast<std::size_t>(i)] = singular[i].toDouble();

    const auto convention = root.value("coordinate_convention").toObject();
    if(!convention.contains("flip_x") || !convention.value("flip_x").isBool()
        || !convention.contains("flip_z") || !convention.value("flip_z").isBool()
        || !convention.contains("feedback_mask") || !convention.value("feedback_mask").isDouble()){
        error=tr("阶段1 JSON缺少完整 coordinate_convention（FlipX/FlipZ/feedback_mask）；禁止把缺省值当成真实标定坐标约定。");
        return false;
    }
    const int conventionMask=convention.value("feedback_mask").toInt(-1);
    const QString maskEncoding=convention.value("feedback_mask_encoding").toString();
    if(!maskEncoding.isEmpty() && maskEncoding!=QStringLiteral("XYZCB_bit0_to_bit4")){
        error=tr("阶段1 JSON 的 feedback_mask_encoding 不受支持：%1").arg(maskEncoding);
        return false;
    }
    // All historical Stage-1 files before the explicit encoding field used the
    // controller-axis one-based bit layout.  New files declare the compact layout.
    const bool legacyOneBasedMask=maskEncoding.isEmpty();
    bool migratedLegacyMask=false;
    std::uint8_t normalizedMask=0;
    if(!normalizePersistedFeedbackMask(conventionMask,legacyOneBasedMask,normalizedMask,migratedLegacyMask)){
        error=legacyOneBasedMask
            ? tr("阶段1旧版JSON的feedback_mask无效；历史格式必须是控制器轴1..5的一基位掩码。")
            : tr("阶段1 JSON 的 feedback_mask 无效；XYZCB_bit0_to_bit4要求0..31。");
        return false;
    }
    flipX = convention.value("flip_x").toBool();
    flipZ = convention.value("flip_z").toBool();
    feedbackMask = normalizedMask;

    capturedPoses.clear();
    const auto poses = root.value("stage1_captured_poses").toArray();
    capturedPoses.reserve(static_cast<std::size_t>(poses.size()));
    const QStringList poseKeys={QStringLiteral("x_mm"),QStringLiteral("y_mm"),QStringLiteral("z_mm"),
                                QStringLiteral("c_deg"),QStringLiteral("b_deg")};
    for (const auto& value : poses) {
        const auto o = value.toObject();
        if (o.isEmpty()) {
            error=tr("阶段1 JSON 的 stage1_captured_poses 含空记录。");
            return false;
        }
        for(const QString& key:poseKeys){
            if(!o.contains(key) || !o.value(key).isDouble() || !std::isfinite(o.value(key).toDouble())){
                error=tr("阶段1 JSON 的采集姿态字段 %1 缺失或不是有限数。").arg(key);
                return false;
            }
        }
        JointPose5D q;
        q.x = o.value("x_mm").toDouble();
        q.y = o.value("y_mm").toDouble();
        q.z = o.value("z_mm").toDouble();
        q.c = o.value("c_deg").toDouble();
        q.b = o.value("b_deg").toDouble();
        capturedPoses.push_back(q);
    }
    const double declaredPoseCountValue=poseCountValue;
    const int declaredPoseCount=static_cast<int>(declaredPoseCountValue);
    if(declaredPoseCount!=static_cast<int>(capturedPoses.size())){
        error=tr("阶段1 JSON 的 stage1_pose_count 与实际采集姿态数组不一致。");
        return false;
    }

    result.success = result.stateRank >= result.expectedStateRank
        && std::isfinite(result.residualRmsMm)
        && std::isfinite(result.sphereCenterMachineMm.x)
        && std::isfinite(result.sphereCenterMachineMm.y)
        && std::isfinite(result.sphereCenterMachineMm.z);
    result.message = result.success ? "Loaded Stage-1 hand-eye/sphere result." : "Stage-1 JSON is not a successful rank-9 result.";
    if (!result.success) {
        error = tr("阶段1结果未通过：rank=%1/%2，RMS=%3 mm。")
            .arg(result.stateRank).arg(result.expectedStateRank).arg(result.residualRmsMm, 0, 'g', 8);
        return false;
    }
    return true;
}

void SingleSphere14CalibrationWidget::onLoadStage1Result()
{
    if (m_referenceTaught || !m_records.empty() || m_stage1SolveRunning || m_stage2SolveRunning) {
        QMessageBox::information(this, tr("请先清空当前实机会话"),
            tr("加载阶段1 JSON用于离线恢复/离线求解。当前已有实机阶段1/阶段2会话，"
               "为避免混合不同坐标约定，请先清空会话。已经导入CSV但尚未建立实机会话时可以直接加载。"));
        return;
    }
    const QString fn = QFileDialog::getOpenFileName(this, tr("加载阶段1初步手眼结果"), QString(), tr("JSON (*.json)"));
    if (fn.isEmpty()) return;

    NominalHandEyeSphereResult loaded;
    std::vector<JointPose5D> poses;
    qint64 elapsed = 0;
    bool flipX = false, flipZ = false;
    std::uint8_t feedbackMask = 0;
    QString error;
    if (!loadStage1ResultJson(fn, loaded, poses, elapsed, flipX, flipZ, feedbackMask, error)) {
        QMessageBox::warning(this, tr("加载阶段1失败"), error);
        return;
    }

    m_persistentCalibrationLoaded = false;
    m_persistentCalibrationBound = false;
    m_persistedValidationSummary = ValidationSummary{};
    m_persistentBindError.clear();

    m_bootstrapResult = loaded;
    m_hasBootstrap = true;
    m_bootstrapRecordCount = static_cast<int>(poses.size());
    m_stage1SolveElapsedMs = elapsed;
    m_loadedStage1CapturedPoses = std::move(poses);
    m_referenceFlipX = flipX;
    m_referenceFlipZ = flipZ;
    m_referenceFeedbackMask = feedbackMask;
    m_modelFeedbackMask = feedbackMask;
    // 离线恢复不等价于重新锁定当前实机约定，因此不允许直接发运动命令。
    m_referenceTaught = false;
    m_sensorConventionLocked = false;
    m_currentEstimate = loaded.parameters;
    m_sphereCenterGuessMachine = loaded.sphereCenterMachineMm;
    m_hasResult = false;
    m_refinementSolved = false;
    m_planBootstrap = true;
    m_fullPlanGenerated = false;

    updateStage1ResultDisplay();
    updateSessionSummary();
    updateCalibrationReadiness();
    updateWorkflowGuide();
    const auto training = stage2TrainingObservations();
    appendLog(tr("已加载阶段1结果用于离线恢复：%1；rank=%2/9，RMS=%3 mm，球心=%4。"
                 "当前从已导入CSV中识别到阶段2训练profiles=%5（若CSV是旧版阶段1+阶段2混合文件，将按阶段1姿态自动剔除）。")
        .arg(fn).arg(loaded.stateRank).arg(loaded.residualRmsMm,0,'g',7).arg(centerText(loaded.sphereCenterMachineMm))
        .arg(static_cast<qulonglong>(training.size())));
}

std::vector<SphereProfileObservation> SingleSphere14CalibrationWidget::stage2TrainingObservations() const
{
    std::vector<SphereProfileObservation> out;
    if (!m_records.empty()) {
        for (const auto& record : m_records) {
            if (record.stage1Bootstrap) continue;
            out.insert(out.end(), record.observations.begin(), record.observations.end());
        }
        return out;
    }

    // Imported CSV has no stage flag.  New exports contain Stage-2 only.  For legacy
    // V27.6.10A mixed exports, a loaded Stage-1 JSON supplies the exact captured poses
    // so the Stage-1 profiles can be removed without asking the operator to recollect.
    const auto& imported = m_calibrator.observations();
    out.reserve(imported.size());
    const auto matchesStage1Pose = [this](const JointPose5d& q) {
        for (const auto& s : m_loadedStage1CapturedPoses) {
            if (std::abs(q.x - s.x) <= 0.03 && std::abs(q.y - s.y) <= 0.03 &&
                std::abs(q.z - s.z) <= 0.03 && std::abs(q.c - s.c) <= 0.03 &&
                std::abs(q.b - s.b) <= 0.03) return true;
        }
        return false;
    };
    for (const auto& observation : imported) {
        if (!m_loadedStage1CapturedPoses.empty() && matchesStage1Pose(observation.joints)) continue;
        out.push_back(observation);
    }
    return out;
}

int SingleSphere14CalibrationWidget::stage2TrainingPoseCount() const
{
    const auto observations = stage2TrainingObservations();
    std::vector<JointPose5d> poses;
    for (const auto& observation : observations) {
        bool found = false;
        for (const auto& p : poses) {
            if (std::abs(observation.joints.x-p.x)<=0.03 && std::abs(observation.joints.y-p.y)<=0.03 &&
                std::abs(observation.joints.z-p.z)<=0.03 && std::abs(observation.joints.c-p.c)<=0.03 &&
                std::abs(observation.joints.b-p.b)<=0.03) { found = true; break; }
        }
        if (!found) poses.push_back(observation.joints);
    }
    return static_cast<int>(poses.size());
}

std::vector<SphereProfileObservation> SingleSphere14CalibrationWidget::reducedStage2Observations(
    int maxProfilesPerPose, std::size_t maxPointsPerProfile) const
{
    const auto full = stage2TrainingObservations();
    if (maxProfilesPerPose <= 0 || maxPointsPerProfile == 0 || full.empty()) return {};

    struct PoseGroup { JointPose5d pose; std::vector<std::size_t> indices; };
    std::vector<PoseGroup> groups;
    for (std::size_t i=0; i<full.size(); ++i) {
        bool assigned = false;
        for (auto& g : groups) {
            const auto& a=full[i].joints; const auto& b=g.pose;
            if (std::abs(a.x-b.x)<=0.03 && std::abs(a.y-b.y)<=0.03 && std::abs(a.z-b.z)<=0.03 &&
                std::abs(a.c-b.c)<=0.03 && std::abs(a.b-b.b)<=0.03) {
                g.indices.push_back(i); assigned=true; break;
            }
        }
        if (!assigned) groups.push_back(PoseGroup{full[i].joints,{i}});
    }

    std::vector<SphereProfileObservation> reduced;
    for (const auto& g : groups) {
        const int take = std::min(maxProfilesPerPose, static_cast<int>(g.indices.size()));
        for (int k=0; k<take; ++k) {
            const std::size_t pos = (take == 1) ? g.indices.size()/2
                : static_cast<std::size_t>(std::llround(static_cast<double>(k) * static_cast<double>(g.indices.size()-1) / static_cast<double>(take-1)));
            SphereProfileObservation observation = full[g.indices[pos]];
            if (observation.points.size() > maxPointsPerProfile) {
                std::vector<SensorProfilePoint> points;
                points.reserve(maxPointsPerProfile);
                if (maxPointsPerProfile == 1) {
                    points.push_back(observation.points[observation.points.size()/2]);
                } else {
                    for (std::size_t j=0; j<maxPointsPerProfile; ++j) {
                        const std::size_t idx = static_cast<std::size_t>(std::llround(
                            static_cast<double>(j) * static_cast<double>(observation.points.size()-1) /
                            static_cast<double>(maxPointsPerProfile-1)));
                        points.push_back(observation.points[idx]);
                    }
                }
                observation.points = std::move(points);
            }
            reduced.push_back(std::move(observation));
        }
    }
    return reduced;
}

void SingleSphere14CalibrationWidget::appendStage2SolveDetail(const QString& text)
{
    const QString line = QStringLiteral("[%1] %2")
        .arg(QDateTime::currentDateTime().toString(QStringLiteral("HH:mm:ss.zzz")), text);
    if (m_stage2SolveDetails) m_stage2SolveDetails->appendPlainText(line);
    appendLog(tr("阶段2求解：%1").arg(text));
}

void SingleSphere14CalibrationWidget::updateStage2PlanProgressUi(
    const Physical17PlanningProgress& progress)
{
    if (!m_stage2PlanRunning) return;

    const qint64 now=QDateTime::currentMSecsSinceEpoch();
    if (progress.phase!=m_stage2PlanPhase) {
        m_stage2PlanPhase=progress.phase;
        m_stage2PlanPhaseStartMs=now;
    }

    const double fraction = progress.total>0
        ? std::clamp(static_cast<double>(progress.current)/static_cast<double>(progress.total),0.0,1.0)
        : 0.0;
    int percent=0;
    QString phase;
    switch (progress.phase) {
    case Physical17PlanningPhase::Preparing:
        phase=tr("准备候选网格"); percent=1; break;
    case Physical17PlanningPhase::FilteringCandidates:
        phase=tr("快速XYZ/FOV边界粗筛");
        percent=2+static_cast<int>(std::llround(38.0*fraction));
        break;
    case Physical17PlanningPhase::EvaluatingShortlist:
        phase=tr("shortlist光学/信息矩阵");
        percent=40+static_cast<int>(std::llround(25.0*fraction));
        break;
    case Physical17PlanningPhase::SelectingDOptimal:
        phase=tr("D-optimal + 延迟MachineSafety");
        percent=65+static_cast<int>(std::llround(33.0*fraction));
        break;
    case Physical17PlanningPhase::Finalizing:
        phase=tr("整理最终姿态");
        percent=98+static_cast<int>(std::llround(2.0*fraction));
        break;
    }
    percent=std::clamp(percent,0,100);
    if (m_stage2PlanProgress) m_stage2PlanProgress->setValue(percent);

    const double elapsedSec=std::max<qint64>(0,now-m_stage2PlanStartMs)/1000.0;
    QString etaText=tr("预计剩余：计算中");
    const qint64 phaseElapsedMs=std::max<qint64>(0,now-m_stage2PlanPhaseStartMs);
    if (progress.current>0 && progress.total>progress.current && phaseElapsedMs>=500) {
        const double remainMs=static_cast<double>(phaseElapsedMs)
            * static_cast<double>(progress.total-progress.current)
            / static_cast<double>(progress.current);
        if (std::isfinite(remainMs)) {
            const double remainSec=remainMs/1000.0;
            etaText=remainSec<90.0
                ? tr("本阶段预计剩余：%1 s").arg(remainSec,0,'f',1)
                : tr("本阶段预计剩余：%1 min").arg(remainSec/60.0,0,'f',1);
        }
    } else if (progress.total>0 && progress.current>=progress.total) {
        etaText=tr("本阶段即将完成");
    }

    QString detail=tr("%1 %2% | 耗时 %3 s | %4")
        .arg(phase).arg(percent).arg(elapsedSec,0,'f',1).arg(etaText);
    if (progress.phase==Physical17PlanningPhase::FilteringCandidates) {
        detail += tr(" | 已检查 %1/%2 | 粗筛可达 %3")
            .arg(static_cast<qulonglong>(progress.current))
            .arg(static_cast<qulonglong>(progress.total))
            .arg(static_cast<qulonglong>(progress.feasibleCandidates));
    } else if (progress.phase==Physical17PlanningPhase::EvaluatingShortlist) {
        detail += tr(" | 信息候选进度 %1/%2 | 已构建 %3")
            .arg(static_cast<qulonglong>(progress.current))
            .arg(static_cast<qulonglong>(progress.total))
            .arg(static_cast<qulonglong>(progress.feasibleCandidates));
    } else if (progress.phase==Physical17PlanningPhase::SelectingDOptimal) {
        detail += tr(" | 已选 %1/%2 | 信息候选 %3")
            .arg(progress.selectedPoses).arg(progress.requestedPoses)
            .arg(static_cast<qulonglong>(progress.feasibleCandidates));
    }
    if (m_lblStage2PlanProgress) m_lblStage2PlanProgress->setText(detail);
}

void SingleSphere14CalibrationWidget::updateStage2SolveProgressUi(const Physical17SolveProgress& progress)
{
    if (!m_stage2SolveRunning) return;
    if (m_stage2SolveProgress) m_stage2SolveProgress->setValue(std::max(0,std::min(100,progress.percent)));
    const QString message = QString::fromStdString(progress.message);
    if (m_lblStage2SolveProgress) m_lblStage2SolveProgress->setText(
        tr("阶段2求解 %1%：%2").arg(progress.percent).arg(message));

    QString phase;
    switch (progress.phase) {
    case Physical17SolveProgress::Phase::Preparing: phase=tr("准备"); break;
    case Physical17SolveProgress::Phase::MultiStart: phase=tr("多起点"); break;
    case Physical17SolveProgress::Phase::AlternatingMachine: phase=tr("机床几何块"); break;
    case Physical17SolveProgress::Phase::AlternatingHandEye: phase=tr("手眼姿态块"); break;
    case Physical17SolveProgress::Phase::JointRefinement: phase=tr("联合精修"); break;
    case Physical17SolveProgress::Phase::Observability: phase=tr("可观测性/rank"); break;
    case Physical17SolveProgress::Phase::Finished: phase=tr("完成"); break;
    }
    QString detail=tr("%1：%2%").arg(phase).arg(progress.percent);
    if (progress.startCount>0) detail += tr("；起点 %1/%2").arg(progress.startIndex).arg(progress.startCount);
    if (progress.outerCount>0) detail += tr("；外层 %1/%2").arg(progress.outerIteration).arg(progress.outerCount);
    if (progress.innerCount>0) detail += tr("；迭代 %1/%2").arg(progress.innerIteration).arg(progress.innerCount);
    if (std::isfinite(progress.robustCost) && progress.robustCost>0.0) detail += tr("；robust cost=%1").arg(progress.robustCost,0,'g',7);
    if (!message.isEmpty()) detail += tr("；%1").arg(message);
    appendStage2SolveDetail(detail);
}

void SingleSphere14CalibrationWidget::updateStage1CandidateTable()
{
    if (!m_stage1CandidateTable) return;
    QSignalBlocker blocker(m_stage1CandidateTable);
    m_stage1CandidateTable->setRowCount(static_cast<int>(m_stage1Candidates.size()));
    for (int i=0;i<static_cast<int>(m_stage1Candidates.size());++i) {
        const auto& q=m_stage1Candidates[static_cast<std::size_t>(i)];
        const QString status=(i<static_cast<int>(m_stage1CandidateStatus.size()))?m_stage1CandidateStatus[static_cast<std::size_t>(i)]:tr("待移动/采集");
        const QStringList vals={QString::number(i+1),QString::number(q.x,'f',3),QString::number(q.y,'f',3),QString::number(q.z,'f',3),QString::number(q.c,'f',2),QString::number(q.b,'f',2),status};
        for(int c=0;c<vals.size();++c)m_stage1CandidateTable->setItem(i,c,new QTableWidgetItem(vals[c]));
    }
}

void SingleSphere14CalibrationWidget::setStage1CandidateStatus(int index,const QString& status)
{
    if(index<0||index>=static_cast<int>(m_stage1Candidates.size()))return;
    if(m_stage1CandidateStatus.size()!=m_stage1Candidates.size())m_stage1CandidateStatus.assign(m_stage1Candidates.size(),tr("待移动/采集"));
    m_stage1CandidateStatus[static_cast<std::size_t>(index)]=status;
    if(m_stage1CandidateTable&&index<m_stage1CandidateTable->rowCount()){
        auto* item=m_stage1CandidateTable->item(index,6); if(!item){item=new QTableWidgetItem;m_stage1CandidateTable->setItem(index,6,item);} item->setText(status);
    }
}

int SingleSphere14CalibrationWidget::selectedStage1CandidateIndex() const
{
    if(!m_stage1CandidateTable||!m_stage1CandidateTable->selectionModel())return -1;
    const auto rows=m_stage1CandidateTable->selectionModel()->selectedRows();
    return rows.isEmpty()?-1:rows.first().row();
}

bool SingleSphere14CalibrationWidget::selectNextStage1Candidate(int afterIndex)
{
    if(!m_stage1CandidateTable||m_stage1Candidates.empty())return false;
    for(int pass=0;pass<2;++pass){
        const int begin=pass==0?afterIndex+1:0;
        const int end=pass==0?static_cast<int>(m_stage1Candidates.size()):std::min(afterIndex+1,static_cast<int>(m_stage1Candidates.size()));
        for(int i=begin;i<end;++i){
            const QString st=i<static_cast<int>(m_stage1CandidateStatus.size())?m_stage1CandidateStatus[static_cast<std::size_t>(i)]:QString();
            if(st.contains(tr("已采集"))||st.contains(tr("跳过")))continue;
            m_stage1CandidateTable->selectRow(i); return true;
        }
    }
    return false;
}

void SingleSphere14CalibrationWidget::onGenerateStage1Candidates()
{
    if(!m_referenceTaught){QMessageBox::warning(this,tr("阶段1尚未开始"),tr("请先执行①开始/重置阶段1，程序需要以该时刻XYZCB作为候选轨迹锚点。"));return;}
    if(m_hasBootstrap){QMessageBox::information(this,tr("阶段1已经完成"),tr("阶段1已经求解通过，不建议再改变其训练集。若要重做，请先重新开始阶段1。"));return;}
    const auto& a=m_referenceSeed.taughtJoint;
    // 相对模板来源于本机第一次成功的11组阶段1实测轨迹；另外加入5个相邻成功姿态的中点。
    // 相对化后可复用于同一装夹/同一球位置附近的新会话，并保留软限位/主运动入口检查。
    const std::array<std::array<double,5>,16> d{{
        {{0.0,0.0,0.0,0.0,0.0}}, {{0.0,0.0,0.0,2.0,4.0}},
        {{-12.5,1.5,1.5,2.0,6.0}}, {{-12.5,1.5,1.5,-10.0,8.0}},
        {{-21.5,-3.5,4.0,-20.0,10.0}}, {{-32.5,-10.5,7.5,-26.0,12.0}},
        {{43.5,-13.0,4.0,-26.0,-9.0}}, {{47.0,-4.0,2.5,-20.0,-11.0}},
        {{56.5,3.0,6.0,-24.0,-15.0}}, {{60.0,5.5,0.5,-29.0,-15.0}},
        {{58.9,9.2,1.9,7.1,-14.0}},
        {{-12.5,1.5,1.5,-4.0,7.0}}, {{-17.0,-1.0,2.75,-15.0,9.0}},
        {{45.25,-8.5,3.25,-23.0,-10.0}}, {{51.75,-0.5,4.25,-22.0,-13.0}},
        {{59.45,7.35,1.2,-11.0,-14.5}}
    }};
    m_stage1Candidates.clear(); m_stage1CandidateStatus.clear();
    for(const auto& v:d){JointPose5D q; q.x=a.x+v[0];q.y=a.y+v[1];q.z=a.z+v[2];q.c=a.c+v[3];q.b=a.b+v[4];m_stage1Candidates.push_back(q);m_stage1CandidateStatus.push_back(tr("待移动/采集"));}
    // If the operator already captured a few manual poses before generating the table,
    // mark near-identical template entries as completed instead of asking for duplicates.
    for(int i=0;i<static_cast<int>(m_stage1Candidates.size());++i){
        const auto& q=m_stage1Candidates[static_cast<std::size_t>(i)];
        for(const auto& r:m_records){
            if(!r.stage1Bootstrap)continue;
            if(std::abs(q.x-r.usedPose.x)<0.8&&std::abs(q.y-r.usedPose.y)<0.8&&std::abs(q.z-r.usedPose.z)<0.8&&
               std::abs(q.b-r.usedPose.b)<0.8&&std::abs(q.c-r.usedPose.c)<0.8){m_stage1CandidateStatus[static_cast<std::size_t>(i)]=tr("已采集(匹配现有)");break;}
        }
    }
    updateStage1CandidateTable();
    selectNextStage1Candidate(-1);
    appendLog(tr("阶段1已生成%1个候选姿态：11个来自第一次成功标定的相对轨迹模板，另加5个相邻成功姿态插值点。每次运动仍需人工确认，并经过现有软限位/五轴运动入口。").arg(m_stage1Candidates.size()));
}

void SingleSphere14CalibrationWidget::onMoveToSelectedStage1Candidate()
{
    if(m_moveInProgress||m_collecting||m_stage1SolveRunning)return;
    if(!m_referenceTaught||m_hasBootstrap)return;
    const int i=selectedStage1CandidateIndex();
    if(i<0||i>=static_cast<int>(m_stage1Candidates.size())){QMessageBox::information(this,tr("未选择候选"),tr("请先生成并选择阶段1候选姿态。"));return;}
    QString conventionError; if(!validateSessionConventions(conventionError)){QMessageBox::warning(this,tr("坐标约定已改变"),conventionError);return;}
    JointState5D state; QString error; if(!readStableState(state,error)){QMessageBox::warning(this,tr("不能移动"),error);return;}
    const JointPose5D current=calibrationPoseFromState(state,m_referenceFeedbackMask);
    const JointPose5D target=m_stage1Candidates[static_cast<std::size_t>(i)];
    const QString confirm=tr("当前：%1\n目标：%2\n\n将一次下发完整XYZCB绝对目标，不做中间视觉分段。现有控制器软限位和五轴运动检查仍然保留，但程序没有夹具/工件三维碰撞模型。请确认从当前姿态到目标姿态的机械路径安全。\n\n继续？").arg(poseText(current),poseText(target));
    if(QMessageBox::question(this,tr("确认阶段1候选移动"),confirm,QMessageBox::Yes|QMessageBox::No,QMessageBox::No)!=QMessageBox::Yes)return;
    m_moveInProgress=true; m_pendingMoveKind=PendingMoveKind::Stage1CandidateDirect; m_pendingMovePlanIndex=i; m_activeStage1Candidate=i;
    setStage1CandidateStatus(i,tr("直接移动中")); updateStage1Status();
    emit requestJointMove(target,m_spinStage1Speed?m_spinStage1Speed->value():0.10);
}

void SingleSphere14CalibrationWidget::onSkipSelectedStage1Candidate()
{
    const int i=selectedStage1CandidateIndex(); if(i<0)return;
    setStage1CandidateStatus(i,tr("已跳过"));
    if(m_activeStage1Candidate==i)m_activeStage1Candidate=-1;
    selectNextStage1Candidate(i);
}

void SingleSphere14CalibrationWidget::onStage1Nudge(int axis, double delta)
{
    if (m_moveInProgress || m_collecting || m_stage1SolveRunning) return;
    if (!m_controller || !m_controller->isConnected()) {
        QMessageBox::warning(this, tr("不能移动"), tr("控制器未连接。"));
        return;
    }
    JointState5D state;
    QString error;
    if (!readStableState(state, error)) {
        QMessageBox::warning(this, tr("不能移动"), error);
        return;
    }

    JointPose5D target = state.mpos;
    switch (axis) {
    case AxisX: target.x += delta; break;
    case AxisY: target.y += delta; break;
    case AxisZ: target.z += delta; break;
    case AxisC: target.c += delta; break;
    case AxisB: target.b += delta; break;
    default: return;
    }

    m_moveInProgress = true;
    m_pendingMoveKind = PendingMoveKind::Stage1Nudge;
    m_pendingMovePlanIndex = -1;
    updateStage1Status();
    emit requestJointMove(target, m_spinStage1Speed ? m_spinStage1Speed->value() : 0.10);
}

QString SingleSphere14CalibrationWidget::stage1FailureAdvice(
    const NominalHandEyeSphereResult& result) const
{
    int count = 0;
    double bMin = +1.0e9, bMax = -1.0e9, cMin = +1.0e9, cMax = -1.0e9;
    QSet<int> bBins, cBins;
    double sb=0.0, sc=0.0, sbb=0.0, scc=0.0, sbc=0.0;

    for (const auto& r : m_records) {
        if (!r.stage1Bootstrap) continue;
        ++count;
        const double b = r.usedPose.b;
        const double c = r.usedPose.c;
        bMin = std::min(bMin, b); bMax = std::max(bMax, b);
        cMin = std::min(cMin, c); cMax = std::max(cMax, c);
        bBins.insert(qRound(b * 2.0));
        cBins.insert(qRound(c * 2.0));
        sb += b; sc += c; sbb += b*b; scc += c*c; sbc += b*c;
    }

    const double bSpan = count > 0 ? bMax - bMin : 0.0;
    const double cSpan = count > 0 ? cMax - cMin : 0.0;
    double corr = 0.0;
    if (count >= 3) {
        const double n = static_cast<double>(count);
        const double vb = sbb - sb*sb/n;
        const double vc = scc - sc*sc/n;
        const double cov = sbc - sb*sc/n;
        if (vb > 1e-12 && vc > 1e-12) corr = cov / std::sqrt(vb * vc);
    }

    QStringList why;
    if (result.stateRank < 9) {
        why << tr("• rank只有%1/9：采集组数够不等于几何激励够。需要B单独变化、C单独变化和B/C交叉组合。")
               .arg(result.stateRank);
    }
    if (bSpan < 10.0)
        why << tr("• B跨度只有%1°，建议至少10~20°。").arg(bSpan,0,'f',1);
    if (cSpan < 15.0)
        why << tr("• C跨度只有%1°，建议至少15~30°。").arg(cSpan,0,'f',1);
    if (bBins.size() < 3)
        why << tr("• B只有约%1个明显角度层级，建议至少3~5个。").arg(bBins.size());
    if (cBins.size() < 3)
        why << tr("• C只有约%1个明显角度层级，建议至少3~5个。").arg(cBins.size());
    if (std::abs(corr) > 0.95 && count >= 6) {
        why << tr("• B/C高度相关（相关系数≈%1）。若总是按固定比例一起变化，会近似落在一条轨迹上，降低可辨识性。")
               .arg(corr,0,'f',3);
    }
    if (!std::isfinite(result.residualRmsMm) || result.residualRmsMm > 0.50) {
        why << tr("• 球面RMS=%1 mm超过阶段1接受阈值0.50 mm。请检查静止采集、RANSAC、Flip X/Z、DPOS/MPOS数据源和遮挡。")
               .arg(result.residualRmsMm,0,'g',6);
    }
    if (!std::isfinite(result.conditionNumber) || result.conditionNumber > 1.0e10) {
        why << tr("• condition=%1过大，数据病态。请扩大且去相关的B/C激励，不要反复采相邻姿态。")
               .arg(result.conditionNumber,0,'g',6);
    }
    if (!result.message.empty())
        why << tr("• 求解器：%1").arg(QString::fromStdString(result.message));
    if (why.isEmpty())
        why << tr("• GUI质量门限未通过。建议删除球拟合质量最差的组，再补采正/负B、正/负C及交叉组合。");

    return tr("当前阶段1数据：%1组；B跨度=%2°（约%3个层级）；C跨度=%4°（约%5个层级）；B/C相关=%6。\n"
              "%7\n\n"
              "推荐补采结构：(B,C)≈(0,0),(+10,0),(-10,0),(0,+15),(0,-15),"
              "(+10,+15),(+10,-15),(-10,+15),(-10,-15)，再按机械安全范围扩大。")
        .arg(count).arg(bSpan,0,'f',1).arg(bBins.size())
        .arg(cSpan,0,'f',1).arg(cBins.size()).arg(corr,0,'f',3)
        .arg(why.join(QStringLiteral("\n")));
}

void SingleSphere14CalibrationWidget::updateStage1Status()
{
    if (!m_lblStage1Status) return;

    int stage1 = 0;
    int profiles = 0;
    std::size_t points = 0;
    double bMin=+1.0e9, bMax=-1.0e9, cMin=+1.0e9, cMax=-1.0e9;
    QSet<int> bBins, cBins;
    for (const auto& r : m_records) {
        if (!r.stage1Bootstrap) continue;
        ++stage1;
        bMin=std::min(bMin,r.usedPose.b); bMax=std::max(bMax,r.usedPose.b);
        cMin=std::min(cMin,r.usedPose.c); cMax=std::max(cMax,r.usedPose.c);
        bBins.insert(qRound(r.usedPose.b*2.0));
        cBins.insert(qRound(r.usedPose.c*2.0));
        profiles += static_cast<int>(r.observations.size());
        for (const auto& o : r.observations) points += o.points.size();
    }
    const double bSpan = stage1 ? bMax-bMin : 0.0;
    const double cSpan = stage1 ? cMax-cMin : 0.0;

    const QString dev = tr("%1 | %2 | %3")
        .arg((m_controller&&m_controller->isConnected()) ? tr("控制器✓") : tr("控制器×"))
        .arg((m_sensor&&m_sensor->isConnected()) ? tr("2950✓") : tr("2950×"))
        .arg((m_sensor&&m_sensor->isProfileRunning()) ? tr("轮廓流✓") : tr("轮廓流×"));

    QString fit = tr("最近拟合：无");
    if (m_hasLatestFit) {
        fit = tr("最近拟合：RMS=%1 mm，X=%2，Z=%3，|Y|=%4")
            .arg(m_latestFit.circleRmsMm,0,'g',5)
            .arg(m_latestFit.centerSensorMm.x,0,'f',3)
            .arg(m_latestFit.centerSensorMm.z,0,'f',3)
            .arg(std::abs(m_latestFit.centerSensorMm.y),0,'f',3);
    }

    QString solve = tr("求解：尚未执行");
    if (m_stage1SolveRunning) {
        solve = tr("求解：正在后台计算...");
    } else if (m_hasBootstrap) {
        solve = tr("求解：通过 rank=%1/9，RMS=%2，cond=%3")
            .arg(m_bootstrapResult.stateRank)
            .arg(m_bootstrapResult.residualRmsMm,0,'g',5)
            .arg(m_bootstrapResult.conditionNumber,0,'g',5);
    } else if (m_bootstrapResult.profileCount > 0 || !m_bootstrapResult.message.empty()) {
        solve = tr("求解：未通过 rank=%1/9，RMS=%2，cond=%3")
            .arg(m_bootstrapResult.stateRank)
            .arg(m_bootstrapResult.residualRmsMm,0,'g',5)
            .arg(m_bootstrapResult.conditionNumber,0,'g',5);
    }

    QString collecting = tr("采集空闲");
    if (m_capturePendingStart) {
        collecting = tr("采集：等待停稳");
    } else if (m_collecting) {
        collecting = tr("采集%1/%2").arg(m_captureFrames.size())
            .arg(m_spinCaptureFrames ? m_spinCaptureFrames->value() : 0);
    }

    m_lblStage1Status->setText(
        tr("%1 | %2\n"
           "阶段1：%3组；B跨度=%4°(%5层)；C跨度=%6°(%7层)；有效profiles=%8；球面点=%9\n"
           "%10\n%11")
        .arg(dev).arg(collecting).arg(stage1)
        .arg(bSpan,0,'f',1).arg(bBins.size())
        .arg(cSpan,0,'f',1).arg(cBins.size())
        .arg(profiles).arg(static_cast<qulonglong>(points))
        .arg(fit, solve));

    const bool idle = !m_moveInProgress && !m_collecting && !m_stage1SolveRunning && !m_stage2SolveRunning;
    const bool connected = m_controller && m_controller->isConnected();
    const bool sensorConnected = m_sensor && m_sensor->isConnected();
    const bool profileRunning = sensorConnected && m_sensor->isProfileRunning();

    if (m_btnTeach) {
        m_btnTeach->setEnabled(connected && sensorConnected && idle);
        if (!connected)
            m_btnTeach->setToolTip(tr("请先在右上角“设备连接”区域连接控制器。"));
        else if (!sensorConnected)
            m_btnTeach->setToolTip(tr("请先在右上角“设备连接”区域连接2950传感器。"));
        else if (!idle)
            m_btnTeach->setToolTip(tr("当前正在运动、采集或求解；结束后才能开始/重置阶段1。"));
        else
            m_btnTeach->setToolTip(tr("读取当前XYZCB数据源和传感器Flip X/Flip Z，并锁定本次阶段1坐标约定。"));
    }

    if (m_btnStartPreview) {
        m_btnStartPreview->setEnabled(sensorConnected && !profileRunning && !m_collecting);
        m_btnStartPreview->setToolTip(!sensorConnected
            ? tr("请先连接2950传感器。")
            : (profileRunning ? tr("轮廓预览已经在运行。") : tr("启动阶段1二维轮廓预览。")));
    }
    if (m_btnStopPreview) {
        m_btnStopPreview->setEnabled(profileRunning && !m_collecting);
        m_btnStopPreview->setToolTip(!sensorConnected
            ? tr("2950传感器未连接。")
            : (!profileRunning ? tr("轮廓预览尚未启动。") : tr("停止当前轮廓预览。")));
    }

    for (auto* button : m_stage1NudgeButtons) {
        if (!button) continue;
        button->setEnabled(connected && idle);
        button->setToolTip(!connected
            ? tr("请先连接控制器。")
            : (!idle ? tr("运动、采集或求解期间禁止点动。") : tr("阶段1人工找球/姿态激励点动。")));
    }

    if (m_btnReadPose) {
        m_btnReadPose->setEnabled(connected && idle);
        m_btnReadPose->setToolTip(connected ? tr("读取当前稳定XYZCB位置。") : tr("请先连接控制器。"));
    }
    if (m_btnFit) {
        m_btnFit->setEnabled((m_hasLatestProfile || (m_sensor && m_sensor->isProfileRunning())) && idle);
        m_btnFit->setToolTip((m_hasLatestProfile || (m_sensor && m_sensor->isProfileRunning()))
            ? tr("对最近一帧轮廓执行球截面RANSAC/圆拟合。")
            : tr("尚未收到轮廓数据；请先连接2950并启动轮廓预览。"));
    }
    if (m_btnStage1Capture) {
        const bool canCapture = connected && sensorConnected && m_referenceTaught
            && !m_hasBootstrap && idle && profileRunning;
        m_btnStage1Capture->setEnabled(canCapture);
        if (!connected || !sensorConnected)
            m_btnStage1Capture->setToolTip(tr("控制器和2950传感器都必须已连接。"));
        else if (!m_referenceTaught)
            m_btnStage1Capture->setToolTip(tr("请先执行①开始/重置阶段1，锁定坐标约定。"));
        else if (!profileRunning)
            m_btnStage1Capture->setToolTip(tr("请先执行②启动轮廓预览。"));
        else if (m_hasBootstrap)
            m_btnStage1Capture->setToolTip(tr("阶段1已求解通过；如需重采请先重置阶段1。"));
        else
            m_btnStage1Capture->setToolTip(tr("采集当前静止姿态的fresh profiles并自动拟合球。"));
    }
    if (m_btnBootstrapSolve) {
        const bool canSolveStage1 = m_referenceTaught && stage1 >= 8 && idle;
        m_btnBootstrapSolve->setEnabled(canSolveStage1);
        if (!m_referenceTaught)
            m_btnBootstrapSolve->setToolTip(tr("请先执行①开始/重置阶段1。"));
        else if (stage1 < 8)
            m_btnBootstrapSolve->setToolTip(tr("当前只有%1组阶段1数据；至少需要8组，建议12~20组并充分变化B/C。")
                                                .arg(stage1));
        else if (!idle)
            m_btnBootstrapSolve->setToolTip(tr("当前正在运动、采集或求解。"));
        else
            m_btnBootstrapSolve->setToolTip(tr("使用已采集的阶段1数据求解手眼6DoF和固定球心3D。"));
    }

    if (m_btnGenerateStage1Candidates) {
        m_btnGenerateStage1Candidates->setEnabled(m_referenceTaught && !m_hasBootstrap && idle);
        m_btnGenerateStage1Candidates->setToolTip(!m_referenceTaught
            ? tr("请先执行①开始阶段1，锁定本次锚点和坐标约定。")
            : tr("按本机第一次成功阶段1轨迹的相对模板生成16个候选（含5个插值补充姿态）。"));
    }
    if (m_btnMoveStage1Candidate) {
        const bool canMoveCandidate=connected && m_referenceTaught && !m_hasBootstrap && idle && !m_stage1Candidates.empty();
        m_btnMoveStage1Candidate->setEnabled(canMoveCandidate);
        m_btnMoveStage1Candidate->setToolTip(canMoveCandidate
            ? tr("一次下发选中候选的完整XYZCB目标；仍经过现有软限位和五轴运动检查。")
            : tr("请先开始阶段1、生成候选，并确保控制器空闲。"));
    }
    if (m_btnSkipStage1Candidate) m_btnSkipStage1Candidate->setEnabled(!m_stage1Candidates.empty() && idle && !m_hasBootstrap);
    if (m_btnSaveStage1Result) m_btnSaveStage1Result->setEnabled(m_hasBootstrap && idle);
    if (m_btnLoadStage1Result) m_btnLoadStage1Result->setEnabled(idle && !m_referenceTaught && m_records.empty());

    if (m_stage1CaptureProgress && !m_collecting && !m_capturePendingStart) {
        m_stage1CaptureProgress->setRange(0,1);
        m_stage1CaptureProgress->setValue(0);
        m_stage1CaptureProgress->setFormat(tr("采集空闲；已保存%1组").arg(stage1));
    }

    if (m_hasBootstrap) {
        const QString gateText =
            tr("阶段2已解锁：阶段1 rank=%1/9，RMS=%2 mm。现在可以生成D-optimal姿态。")
                .arg(m_bootstrapResult.stateRank)
                .arg(m_bootstrapResult.residualRmsMm,0,'g',5);
        const QString gateStyle =
            "QLabel { background:#dff5df; color:#185c18; padding:4px; border:1px solid #6abf69; font-weight:bold; }";
        m_lblStage2Gate->setText(gateText);
        m_lblStage2Gate->setStyleSheet(gateStyle);
        if (m_lblKinematicGate) {
            m_lblKinematicGate->setText(
                tr("运动学标定已解锁：初步手眼标定 rank=%1/9，RMS=%2 mm。")
                    .arg(m_bootstrapResult.stateRank)
                    .arg(m_bootstrapResult.residualRmsMm,0,'g',5));
            m_lblKinematicGate->setStyleSheet(gateStyle);
        }
        if (m_btnGenerate)
            m_btnGenerate->setToolTip(tr("阶段1已通过，可以生成阶段2D-optimal训练姿态。"));
    } else {
        QString why = tr("阶段2锁定：阶段1尚未通过。");
        if (!m_stage1SolveRunning &&
            (m_bootstrapResult.profileCount > 0 || !m_bootstrapResult.message.empty())) {
            why += tr(" 上次 rank=%1/9，RMS=%2，cond=%3。")
                .arg(m_bootstrapResult.stateRank)
                .arg(m_bootstrapResult.residualRmsMm,0,'g',5)
                .arg(m_bootstrapResult.conditionNumber,0,'g',5);
        }
        const QString gateStyle =
            "QLabel { background:#fff3cd; color:#7a5200; padding:4px; border:1px solid #d5b95f; }";
        m_lblStage2Gate->setText(why);
        m_lblStage2Gate->setStyleSheet(gateStyle);
        if (m_lblKinematicGate) {
            m_lblKinematicGate->setText(
                tr("运动学标定尚未解锁。请返回“初步手眼标定”页完成阶段1。 %1").arg(why));
            m_lblKinematicGate->setStyleSheet(gateStyle);
        }
        if (m_btnGenerate) {
            m_btnGenerate->setToolTip(stage1 < 8
                ? tr("阶段1只有%1组，至少8组且建议12~20组。").arg(stage1)
                : tr("阶段1求解尚未通过。请查看顶部求解状态或失败诊断弹窗。"));
        }
    }
}

void SingleSphere14CalibrationWidget::onSolveBootstrap()
{
    if (!m_referenceTaught) {
        QMessageBox::warning(this, tr("阶段1尚未开始"), tr("请先点击阶段1统一操作区的“① 开始/重置阶段1（锁定坐标约定）”。"));
        return;
    }
    QString conventionError;
    if (!validateSessionConventions(conventionError)) {
        QMessageBox::warning(this, tr("不能求解阶段1"), conventionError);
        return;
    }

    NominalHandEyeSphereCalibrator bootstrap(solverOptionsFromUi());
    int poseRecords = 0;
    int originalProfiles = 0;
    std::size_t originalPoints = 0;
    double bMin = +1.0e9, bMax = -1.0e9, cMin = +1.0e9, cMax = -1.0e9;
    // Stage-1 only needs a stable navigation bootstrap. Ten fresh profiles captured at one
    // static pose are strongly correlated and previously caused the numerical-Jacobian solver
    // to repeat almost the same work ~10x. Keep at most three representative profiles/pose and
    // at most 180 evenly-spaced inlier points/profile. Pose diversity is preserved exactly.
    for (const auto& record : m_records) {
        if (!record.stage1Bootstrap) continue; // 阶段2计划数据不回灌到阶段1定义。
        ++poseRecords;
        bMin = std::min(bMin, record.usedPose.b); bMax = std::max(bMax, record.usedPose.b);
        cMin = std::min(cMin, record.usedPose.c); cMax = std::max(cMax, record.usedPose.c);
        originalProfiles += static_cast<int>(record.observations.size());
        for (const auto& obs : record.observations) originalPoints += obs.points.size();
        if (record.observations.empty()) continue;
        std::vector<std::size_t> picks{0};
        if (record.observations.size() >= 3) picks.push_back(record.observations.size()/2);
        if (record.observations.size() >= 2) picks.push_back(record.observations.size()-1);
        std::sort(picks.begin(),picks.end()); picks.erase(std::unique(picks.begin(),picks.end()),picks.end());
        for (std::size_t pi : picks) {
            SphereProfileObservation reduced = record.observations[pi];
            constexpr std::size_t kMaxStage1Points = 180;
            if (reduced.points.size() > kMaxStage1Points) {
                std::vector<SensorProfilePoint> decimated;
                decimated.reserve(kMaxStage1Points);
                const double step = static_cast<double>(reduced.points.size()-1) / static_cast<double>(kMaxStage1Points-1);
                for (std::size_t k=0;k<kMaxStage1Points;++k) {
                    const std::size_t index=std::min(reduced.points.size()-1,static_cast<std::size_t>(std::llround(k*step)));
                    decimated.push_back(reduced.points[index]);
                }
                reduced.points=std::move(decimated);
            }
            bootstrap.addObservation(reduced);
        }
    }
    if (poseRecords < 8) {
        QMessageBox::information(this, tr("阶段1数据不足"),
            tr("当前只有 %1 个独立人工姿态。至少需要8个，建议12~20个，并且B、C都要独立变化。")
                .arg(poseRecords));
        return;
    }
    if ((bMax - bMin) < 10.0 || (cMax - cMin) < 15.0) {
        const auto answer = QMessageBox::warning(
            this, tr("阶段1旋转激励偏小"),
            tr("当前人工数据的B跨度=%1°，C跨度=%2°。\n"
               "名义机床下的手眼6DoF+球心3D共9个状态，需要两个旋转轴都有变化。"
               "建议B跨度至少约10~20°、C跨度至少约15~30°。\n\n仍然尝试求解？")
                .arg(bMax-bMin,0,'f',1).arg(cMax-cMin,0,'f',1),
            QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
        if (answer != QMessageBox::Yes) return;
    }

    m_stage1SolveRunning = true;
    const qint64 solveStartMs = QDateTime::currentMSecsSinceEpoch();
    appendLog(tr("阶段1快速求解开始：%1个独立姿态；原始%2 profiles/%3点 -> 代表集%4 profiles/%5点。保留全部姿态，只压缩同一静止姿态内的重复帧/点。")
              .arg(poseRecords).arg(originalProfiles).arg(static_cast<qulonglong>(originalPoints))
              .arg(static_cast<qulonglong>(bootstrap.observations().size()))
              .arg(static_cast<qulonglong>([&bootstrap](){std::size_t n=0;for(const auto&o:bootstrap.observations())n+=o.points.size();return n;}())));
    m_btnBootstrapSolve->setEnabled(false);
    if (m_btnBootstrapSolve) m_btnBootstrapSolve->setEnabled(false);
    m_lblBootstrap->setText(tr("阶段1求解中：固定11项机床几何为名义值，自动推断每帧±Y初始化，随后使用原始球面残差优化9状态..."));
    updateStage1Status();
    const auto solver = bootstrap;
    auto* watcher = new QFutureWatcher<NominalHandEyeSphereResult>(this);
    connect(watcher, &QFutureWatcher<NominalHandEyeSphereResult>::finished, this,
            [this, watcher, poseRecords, bMin, bMax, cMin, cMax, solveStartMs]() {
        m_bootstrapResult = watcher->result();
        watcher->deleteLater();
        m_stage1SolveRunning = false;
        m_stage1SolveElapsedMs = std::max<qint64>(0, QDateTime::currentMSecsSinceEpoch()-solveStartMs);
        // 阶段1只是“名义机床+手眼+固定球心”的导航初标，不作为最终几何精度结论。
        // 真实机床的11项几何误差此时被故意固定为0，因此允许比最终17参数求解更宽的RMS。
        const bool quality = m_bootstrapResult.success
            && m_bootstrapResult.stateRank == 9
            && std::isfinite(m_bootstrapResult.residualRmsMm)
            && m_bootstrapResult.residualRmsMm <= 0.50
            && std::isfinite(m_bootstrapResult.conditionNumber)
            && m_bootstrapResult.conditionNumber <= 1.0e10;
        const bool highQualityBootstrap = quality && m_bootstrapResult.residualRmsMm <= 0.15;
        m_hasBootstrap = quality;
        m_bootstrapRecordCount = quality ? poseRecords : 0;
        if (quality) {
            m_currentEstimate = m_bootstrapResult.parameters;
            m_sphereCenterGuessMachine = m_bootstrapResult.sphereCenterMachineMm;
            m_referenceSeed.hasSphereCenterMachine = true;
            m_referenceSeed.sphereCenterMachineMm = m_bootstrapResult.sphereCenterMachineMm;
            m_plan.clear();
            m_planTable->setRowCount(0);
            m_fullPlanGenerated = false;
            m_hasResult = false;

            const auto& p = m_bootstrapResult.parameters;
            int plusCount = 0;
            for (int sign : m_bootstrapResult.inferredCenterYSigns) if (sign > 0) ++plusCount;
            const int minusCount = static_cast<int>(m_bootstrapResult.inferredCenterYSigns.size()) - plusCount;
            m_lblBootstrap->setText(tr(
                "阶段1成功：rank=%1/9，raw球面RMS=%2 mm，center-init RMS=%3 mm，cond=%4。\n"
                "初始手眼 r=[%5,%6,%7] rad，t=[%8,%9,%10] mm；固定球心M=%11。\n"
                "每帧±Y由程序自动推断：+=%12，-=%13。%14 现在可以生成阶段2的17参数D-optimal计划。")
                .arg(m_bootstrapResult.stateRank)
                .arg(m_bootstrapResult.residualRmsMm,0,'g',6)
                .arg(m_bootstrapResult.centerInitRmsMm,0,'g',6)
                .arg(m_bootstrapResult.conditionNumber,0,'g',6)
                .arg(p[Physical17Parameters::HandEyeRx],0,'g',7)
                .arg(p[Physical17Parameters::HandEyeRy],0,'g',7)
                .arg(p[Physical17Parameters::HandEyeRz],0,'g',7)
                .arg(p[Physical17Parameters::HandEyeTx],0,'g',8)
                .arg(p[Physical17Parameters::HandEyeTy],0,'g',8)
                .arg(p[Physical17Parameters::HandEyeTz],0,'g',8)
                .arg(centerText(m_bootstrapResult.sphereCenterMachineMm))
                .arg(plusCount).arg(minusCount)
                .arg(highQualityBootstrap
                     ? tr("初标一致性良好。")
                     : tr("初标RMS偏大，说明名义机床与真实几何已有可见差异；阶段2先使用保守角度范围并尽早做17参数中间求解。")));
            appendLog(tr("阶段1手眼+球心初标成功：rank=9/9，RMS=%1 mm（阶段1用于在线导航/姿态规划，不代表最终几何精度），HE t=%2，球心M=%3。V27.6.41最终阶段2求解会再从当前Stage2训练数据独立自举hand-eye，并以新的Tx/Ty/Tz作为translation-gauge anchor。")
                      .arg(m_bootstrapResult.residualRmsMm,0,'g',6)
                      .arg(centerText(Vec3d{p[Physical17Parameters::HandEyeTx],p[Physical17Parameters::HandEyeTy],p[Physical17Parameters::HandEyeTz]}))
                      .arg(centerText(m_bootstrapResult.sphereCenterMachineMm)));
            QMessageBox::information(this, tr("阶段1求解成功"),
                tr("阶段1已经通过，阶段2D-optimal按钮现在会解锁。\n\n"
                   "采集组数：%1\nB跨度：%2°\nC跨度：%3°\nrank：%4/9\nRMS：%5 mm\ncondition：%6\n球心M：%7\n\n"
                   "阶段1只用于导航/初始化，不是最终17参数精度结论。")
                .arg(poseRecords).arg(bMax-bMin,0,'f',1).arg(cMax-cMin,0,'f',1)
                .arg(m_bootstrapResult.stateRank).arg(m_bootstrapResult.residualRmsMm,0,'g',6)
                .arg(m_bootstrapResult.conditionNumber,0,'g',6).arg(centerText(m_bootstrapResult.sphereCenterMachineMm)));
            updateStage1ResultDisplay();
            emit stage1BootstrapCompleted(true);
        } else {
            m_lblBootstrap->setText(tr("阶段1未通过：rank=%1/9，RMS=%2 mm，cond=%3。%4\n"
                                       "请优先增加同时包含B变化和C变化的人工姿态，而不是只增加同方向XYZ平移。")
                .arg(m_bootstrapResult.stateRank)
                .arg(m_bootstrapResult.residualRmsMm,0,'g',6)
                .arg(m_bootstrapResult.conditionNumber,0,'g',6)
                .arg(QString::fromStdString(m_bootstrapResult.message)));
            const QString advice = stage1FailureAdvice(m_bootstrapResult);
            appendLog(tr("阶段1初标未通过：%1\n%2").arg(QString::fromStdString(m_bootstrapResult.message), advice));
            QMessageBox::warning(this, tr("阶段1求解未通过"),
                tr("阶段1没有成功，因此阶段2D-optimal按钮保持锁定。\n\nrank=%1/9\nRMS=%2 mm\ncondition=%3\n\n%4")
                .arg(m_bootstrapResult.stateRank).arg(m_bootstrapResult.residualRmsMm,0,'g',6)
                .arg(m_bootstrapResult.conditionNumber,0,'g',6).arg(advice));
            updateStage1ResultDisplay();
            emit stage1BootstrapCompleted(false);
        }
        appendLog(tr("阶段1求解结束：耗时%1 s，rank=%2/9，RMS=%3 mm，condition=%4。")
                  .arg(m_stage1SolveElapsedMs/1000.0,0,'f',2).arg(m_bootstrapResult.stateRank)
                  .arg(m_bootstrapResult.residualRmsMm,0,'g',6).arg(m_bootstrapResult.conditionNumber,0,'g',6));
        m_btnBootstrapSolve->setEnabled(true);
        updateCalibrationReadiness();
        updateWorkflowGuide();
        updateStage1Status();
    });
    watcher->setFuture(QtConcurrent::run([solver]() mutable { return solver.solve(); }));
}

PosePlannerConfig SingleSphere14CalibrationWidget::plannerConfigFromUi(QString& warning) const
{
    PosePlannerConfig cfg;
    cfg.sphereDiameterMm = m_spinSphereDiameter->value();
    cfg.requestedRecordedPoses = m_spinPoseCount->value();
    cfg.bMinDeg = m_spinBMin->value(); cfg.bMaxDeg = m_spinBMax->value(); cfg.bStepDeg = std::abs(m_spinBStep->value());
    cfg.cMinDeg = m_spinCMin->value(); cfg.cMaxDeg = m_spinCMax->value(); cfg.cStepDeg = std::abs(m_spinCStep->value());
    cfg.useQualityStudyGeometry = true;
    cfg.qualityDMinMm = std::min(m_spinQualityDMin->value(), m_spinQualityDMax->value());
    cfg.qualityDMaxMm = std::max(m_spinQualityDMin->value(), m_spinQualityDMax->value());
    cfg.qualityDStepMm = std::max(0.1, std::abs(m_spinQualityDStep->value()));
    cfg.qualityGammaMinDeg = std::min(m_spinQualityGammaMin->value(), m_spinQualityGammaMax->value());
    cfg.qualityGammaMaxDeg = std::max(m_spinQualityGammaMin->value(), m_spinQualityGammaMax->value());
    cfg.qualityGammaStepDeg = std::max(0.1, std::abs(m_spinQualityGammaStep->value()));
    cfg.qualityEtaMinDeg = std::min(m_spinQualityEtaMin->value(), m_spinQualityEtaMax->value());
    cfg.qualityEtaMaxDeg = std::max(m_spinQualityEtaMin->value(), m_spinQualityEtaMax->value());
    cfg.qualityEtaStepDeg = std::max(0.1, std::abs(m_spinQualityEtaStep->value()));
    // The core planner still has a legacy rectangular fast screen.  Give it the
    // bounding box of the real LLT2950 trapezoid rather than the old +/-18 mm
    // central window; the integrated preflight below performs the authoritative
    // trapezoidal-FOV + visible-arc simulation with safety margins.  This avoids
    // discarding valid D=92..100, |eta|~=7.5 deg candidates before simulation.
    cfg.sensorZMinMm = 70.0;
    cfg.sensorZMaxMm = 120.0;
    cfg.sensorXHalfWidthMm = 29.0;
    cfg.useEndEffectorFacingArc = true;
    cfg.useTrapezoidalSensorField = true;
    cfg.sensorZCoordinateFlipped = m_referenceFlipZ;
    cfg.sensorXSpanNearMm = 42.0;
    cfg.sensorXSpanFarMm = 58.0;
    cfg.sensorXSafetyMarginMm = 1.0;
    cfg.sensorZSafetyMarginMm = 1.0;
    cfg.reorderRecordedPosesForTravel = false;

    if (m_controller && m_controller->isConnected()) {
        AxisParams p[Constants::kAxisParamArraySize];
        QString err;
        if (m_controller->readAllAxisParams(p, &err)) {
            cfg.xMinMm = p[AxisX].softLimitNeg; cfg.xMaxMm = p[AxisX].softLimitPos;
            cfg.yMinMm = p[AxisY].softLimitNeg; cfg.yMaxMm = p[AxisY].softLimitPos;
            cfg.zMinMm = p[AxisZ].softLimitNeg; cfg.zMaxMm = p[AxisZ].softLimitPos;
            cfg.bMinDeg = std::max(cfg.bMinDeg, static_cast<double>(p[AxisB].softLimitNeg));
            cfg.bMaxDeg = std::min(cfg.bMaxDeg, static_cast<double>(p[AxisB].softLimitPos));
            cfg.cMinDeg = std::max(cfg.cMinDeg, static_cast<double>(p[AxisC].softLimitNeg));
            cfg.cMaxDeg = std::min(cfg.cMaxDeg, static_cast<double>(p[AxisC].softLimitPos));
        } else {
            warning = tr("无法读取真实软限位，姿态规划只使用界面范围：%1").arg(err);
        }
    }
    return cfg;
}

void SingleSphere14CalibrationWidget::onImportStage2SphereCloud()
{
    const QString path = QFileDialog::getOpenFileName(
        this, tr("导入运动学标定球面点云"), QString(),
        tr("点云文件 (*.scan *.ply *.csv *.txt);;所有文件 (*.*)"));
    if (path.isEmpty()) return;

    QVector<QVector3D> points;
    QString error;
    if (!PointCloudLoader::load(path, points, &error) || points.size() < 4) {
        QMessageBox::warning(this, tr("导入失败"),
            error.isEmpty() ? tr("点云有效点不足4个") : error);
        return;
    }
    m_stage2ImportedSphereCloud = points;
    m_stage2ImportedSphereCloudPath = path;
    m_referenceSeed.hasSphereCenterMachine = false; // reject stale imported sphere center
    if (m_lblStage2Sphere) {
        m_lblStage2Sphere->setText(tr("已导入 %1 点：%2；准备执行3-D RANSAC球面提取。")
            .arg(points.size()).arg(QFileInfo(path).fileName()));
    }
    if (m_btnFitStage2Sphere) m_btnFitStage2Sphere->setEnabled(true);
    emit requestMachineSafetySphere(0.0, 0.0, 0.0, 0.0, false);
    QTimer::singleShot(0, this, &SingleSphere14CalibrationWidget::onFitStage2SphereCloud);
}

void SingleSphere14CalibrationWidget::onFitStage2SphereCloud()
{
    if (m_stage2ImportedSphereCloud.size() < 4) {
        QMessageBox::warning(this, tr("球面拟合"), tr("请先导入球面点云。"));
        return;
    }
    std::vector<Vec3d> points;
    points.reserve(static_cast<std::size_t>(m_stage2ImportedSphereCloud.size()));
    for (const QVector3D& p : m_stage2ImportedSphereCloud)
        points.emplace_back(p.x(), p.y(), p.z());

    SphereCloudRansacOptions options;
    options.inlierThresholdMm = 0.12;
    options.expectedRadiusMm = 0.5 * m_spinSphereDiameter->value();
    options.maxRadiusDeviationMm = 1.50;
    options.minInliers = std::min<int>(std::max(50, static_cast<int>(points.size()/200)), 1000);
    options.minInlierRatio = 0.02;
    options.maxIterations = 1400;

    if (m_btnFitStage2Sphere) m_btnFitStage2Sphere->setEnabled(false);
    if (m_btnImportStage2Sphere) m_btnImportStage2Sphere->setEnabled(false);
    if (m_lblStage2Sphere) m_lblStage2Sphere->setText(tr("3-D RANSAC球心拟合中... 点数=%1").arg(points.size()));

    const QString inputPath = m_stage2ImportedSphereCloudPath;
    auto* watcher = new QFutureWatcher<SphereTwoStageResult>(this);
    connect(watcher, &QFutureWatcher<SphereTwoStageResult>::finished, this, [this, watcher, inputPath]() {
        const auto comparison = watcher->result();
        if (m_stage2ImportedSphereCloudPath != inputPath) {
            watcher->deleteLater();
            if (m_btnFitStage2Sphere) m_btnFitStage2Sphere->setEnabled(true);
            if (m_btnImportStage2Sphere) m_btnImportStage2Sphere->setEnabled(true);
            appendLog(tr("[两阶段参考球] 舍弃旧点云的后台拟合结果"));
            return;
        }
        const SphereCloudFitResult& fit = comparison.accepted;
        appendLog(tr("[两阶段参考球/Physical17] %1")
            .arg(SphereTwoStageFit::comparisonText(comparison)));
        watcher->deleteLater();
        if (m_btnFitStage2Sphere) m_btnFitStage2Sphere->setEnabled(!m_stage2ImportedSphereCloud.isEmpty());
        if (m_btnImportStage2Sphere) m_btnImportStage2Sphere->setEnabled(true);
        if (!fit.success) {
            if (m_lblStage2Sphere) m_lblStage2Sphere->setText(tr("球面拟合未通过：%1").arg(
                SphereTwoStageFit::comparisonText(comparison)));
            emit requestMachineSafetySphere(0.0, 0.0, 0.0, 0.0, false);
            QMessageBox::warning(this, tr("球面拟合失败"),
                SphereTwoStageFit::comparisonText(comparison));
            return;
        }

        m_sphereCenterGuessMachine = fit.centerMachineMm;
        m_referenceSeed.hasSphereCenterMachine = true;
        m_referenceSeed.sphereCenterMachineMm = fit.centerMachineMm;
        const double radius = 0.5 * m_spinSphereDiameter->value();
        if (m_lblStage2Sphere) {
            m_lblStage2Sphere->setText(tr(
                "当前运动学标定参考球心 M=[%1, %2, %3] mm；拟合R=%4 mm；内点=%5/%6 (%7%)；RMS=%8 mm。"
                " 后续重新生成姿态将使用该球心；上次验证通过的17参数仅作为导航初值/运行回退。")
                .arg(fit.centerMachineMm.x,0,'f',4).arg(fit.centerMachineMm.y,0,'f',4)
                .arg(fit.centerMachineMm.z,0,'f',4).arg(fit.radiusMm,0,'f',4)
                .arg(static_cast<qulonglong>(fit.inlierPointCount))
                .arg(static_cast<qulonglong>(fit.candidatePointCount))
                .arg(fit.inlierRatio*100.0,0,'f',1).arg(fit.residualRmsMm,0,'f',4)
                + QStringLiteral("\n") + SphereTwoStageFit::comparisonText(comparison));
        }
        emit requestMachineSafetySphere(fit.centerMachineMm.x, fit.centerMachineMm.y,
                                        fit.centerMachineMm.z, radius, true);
        appendLog(tr("阶段2已用导入点云更新固定球心：%1；该操作不覆盖已验证生产标定，只改变下一轮姿态规划参考。")
                  .arg(centerText(fit.centerMachineMm)));
    });
    watcher->setFuture(QtConcurrent::run([points=std::move(points), options, inputPath]() {
        return SphereTwoStageFit::fitImportedSphere(inputPath, points, options);
    }));
}

void SingleSphere14CalibrationWidget::preserveAcceptedModelAsFallbackForRecalibration()
{
    if (!m_hasResult || !m_lastResult.success || !m_validationSummary.passed) return;
    m_fallbackResult = m_lastResult;
    m_fallbackValidationSummary = m_validationSummary;
    m_hasFallbackResult = true;
    appendLog(tr("开始新的阶段2标定：当前已验证17参数已复制为运行回退模型；新训练/验证完成前不会丢失旧生产标定。"));
}

bool SingleSphere14CalibrationWidget::preflightAndFilterGeneratedPlan(
    const JointPose5D& startPose, bool gateInitialTransition,
    int desiredPoseCount, QString& summary)
{
    if (m_plan.empty()) {
        summary = tr("D-optimal规划器没有返回候选姿态。");
        return false;
    }

    SphereStudyPathOptions opticalOptions;
    opticalOptions.sphereRadiusMm = 0.5 * m_spinSphereDiameter->value();
    opticalOptions.sensorZMinMm = 70.0;
    opticalOptions.sensorZMaxMm = 120.0;
    opticalOptions.sensorZSafetyMarginMm = 1.0;
    opticalOptions.sensorXMinMm = -25.0;
    opticalOptions.sensorXMaxMm = +25.0;
    opticalOptions.sensorXSafetyMarginMm = 1.0;
    opticalOptions.useTrapezoidalSensorField = true;
    opticalOptions.sensorZCoordinateFlipped = m_referenceFlipZ;
    opticalOptions.sensorXSpanNearMm = 42.0;
    opticalOptions.sensorXSpanFarMm = 58.0;
    opticalOptions.requireEndEffectorFacingArc = true;
    opticalOptions.minimumPlaneIntersectionMarginMm = 0.50;
    opticalOptions.minimumPredictedVisibleArcDeg = 75.0;
    opticalOptions.minimumPredictedVisibleArcLengthMm = 6.0;
    opticalOptions.visibleArcSamples = 720;

    std::vector<CalibrationPoseInstruction> accepted;
    accepted.reserve(static_cast<std::size_t>(desiredPoseCount));
    JointPose5D cursor = startPose;
    bool haveTransitionCursor = gateInitialTransition;
    int opticalRejected = 0;
    int poseSafetyRejected = 0;
    int transitionRejected = 0;

    for (const auto& pose : m_plan) {
        const auto optical = SphereDistanceAngleStudy::simulateOpticalFeasibility(
            pose.predictedJoint, m_sphereCenterGuessMachine,
            0.5 * m_spinSphereDiameter->value(), m_currentEstimate, opticalOptions);
        if (!optical.valid) {
            ++opticalRejected;
            continue;
        }

        const JointPose5D target = controllerPoseFromPortable(pose.predictedJoint);
        QString safetyError;
        if (m_stage2PoseSafetyChecker && !m_stage2PoseSafetyChecker(target, safetyError)) {
            ++poseSafetyRejected;
            appendLog(tr("规划预检剔除目标姿态：%1；%2").arg(poseText(target), safetyError));
            continue;
        }
        if (haveTransitionCursor && m_stage2TransitionSafetyChecker &&
            !m_stage2TransitionSafetyChecker(cursor, target, safetyError)) {
            ++transitionRejected;
            appendLog(tr("规划预检剔除过渡路径：%1 -> %2；%3")
                      .arg(poseText(cursor), poseText(target), safetyError));
            continue;
        }

        CalibrationPoseInstruction kept = pose;
        kept.note = std::string("Optical trapezoid + end-effector visible-arc preflight passed. ") + kept.note;
        accepted.push_back(std::move(kept));
        cursor = target;
        haveTransitionCursor = true;
        if (static_cast<int>(accepted.size()) >= desiredPoseCount) break;
    }

    if (static_cast<int>(accepted.size()) < desiredPoseCount) {
        summary = tr("预检后仅剩 %1/%2 个安全高质量候选；光学剔除=%3，目标碰撞/间隙剔除=%4，过渡路径剔除=%5。"
                     "请扩大B/C范围、减小请求姿态数，或适当放宽D-γ-η候选窗后重新生成。")
            .arg(accepted.size()).arg(desiredPoseCount).arg(opticalRejected)
            .arg(poseSafetyRejected).arg(transitionRejected);
        return false;
    }

    for (std::size_t i=0; i<accepted.size(); ++i) accepted[i].sequence = static_cast<int>(i+1);
    m_plan = std::move(accepted);
    summary = tr("%1预检通过：保留%2个姿态；光学剔除=%3，目标碰撞/间隙剔除=%4，过渡路径剔除=%5。")
        .arg(gateInitialTransition ? tr("在线") : tr("离线"))
        .arg(m_plan.size()).arg(opticalRejected).arg(poseSafetyRejected).arg(transitionRejected);
    return true;
}

void SingleSphere14CalibrationWidget::onGeneratePlan()
{
    if (m_stage2PlanRunning) {
        QMessageBox::information(this,tr("正在生成姿态"),
            tr("阶段2 D-optimal姿态仍在后台生成。请查看进度条，或点击“取消姿态生成”。"));
        return;
    }
    if (!m_hasBootstrap) {
        QMessageBox::warning(this, tr("不能生成阶段2姿态"),
            tr("请先完成/加载阶段1手眼+固定球心结果，或加载包含有效阶段1摘要的已验证17参数标定。"));
        return;
    }

    // Path generation and machine execution are intentionally separate gates.
    // A real start pose is available only when the current live session is bound
    // and the controller can actually provide a stable measured XYZCB state.
    // Otherwise we generate an OFFLINE design from the saved model + fitted
    // machine-space sphere center.  Offline generation never authorizes motion.
    const bool controllerOnline = m_controller && m_controller->isConnected();
    const bool sensorOnline = m_sensor && m_sensor->isConnected();
    const bool liveLinearReference = controllerOnline && m_controller->areLinearMposReferenced();
    const bool liveRotaryReference = controllerOnline
        && m_controller->isRotaryAbsoluteReady(AxisC)
        && m_controller->isRotaryAbsoluteReady(AxisB);
    const bool haveLiveStartPose = m_referenceTaught && controllerOnline && sensorOnline
        && liveLinearReference && liveRotaryReference;
    if (m_referenceTaught && controllerOnline && sensorOnline && !haveLiveStartPose) {
        appendLog(tr("阶段2当前设备会话已绑定，但真实机床起点未获授权：XYZ DATUM=%1，B/C绝对坐标=%2。"
                     "本次仅按离线模式生成D-optimal计划；生成计划不会清除回零/使能状态，也不会授权真实运动。")
                  .arg(liveLinearReference ? tr("OK") : tr("未建立"))
                  .arg(liveRotaryReference ? tr("OK") : tr("未就绪")));
    }

    JointPose5D startPose{};
    if (haveLiveStartPose) {
        QString conventionError;
        if (!validateSessionConventions(conventionError)) {
            QMessageBox::warning(this, tr("坐标约定已改变"), conventionError);
            return;
        }
        JointState5D state;
        QString stateError;
        if (!readStableState(state, stateError)) {
            QMessageBox::warning(this, tr("不能生成姿态"), stateError);
            return;
        }
        startPose = calibrationPoseFromState(state, m_referenceFeedbackMask);
    } else {
        appendLog(tr("阶段2进入离线路径规划：不读取控制器当前XYZCB，也不伪造(0,0,0,0,0)为真实起点；"
                     "首个姿态只做目标姿态安全检查，随后各姿态之间仍执行完整MachineSafety扫掠检查。"
                     "生成计划本身不会解锁真实运动/采集。"));
    }

    // A previously accepted calibration is a valid seed for another calibration
    // run, not a reason to lock the workflow. Keep it as a transactional runtime
    // fallback before the new training set changes.
    preserveAcceptedModelAsFallbackForRecalibration();

    QString warning;
    auto cfg = plannerConfigFromUi(warning);
    if (!warning.isEmpty()) appendLog(warning);

    // The Stage-2 GUI must have finite machine limits in BOTH online and offline
    // planning.  In the integrated application MainWindow resolves these from the
    // Machine5X offline_planning_limits, intersecting live controller soft limits
    // when those are available.  This prevents offline planning from inheriting
    // PosePlannerConfig's intentionally huge placeholder XYZ range.
    QString boundsSource;
    QString boundsError;
    if (m_stage2PlanningBoundsResolver) {
        if (!m_stage2PlanningBoundsResolver(cfg, boundsSource, boundsError)) {
            QMessageBox::warning(this, tr("不能生成阶段2姿态"),
                boundsError.isEmpty() ? tr("无法取得有效的五轴规划限位。") : boundsError);
            return;
        }
        if (!boundsSource.isEmpty()) appendLog(tr("阶段2规划限位：%1").arg(boundsSource));
    } else if (!haveLiveStartPose) {
        QMessageBox::warning(this, tr("不能离线生成姿态"),
            tr("当前没有Machine5X离线规划限位提供器。离线规划禁止使用未约束XYZ范围。"));
        return;
    }

    const int desiredPoseCount = cfg.requestedRecordedPoses;
    // Select exactly the requested design with optical and motion gates inside
    // the greedy loop; do not truncate a travel-sorted reserve pool.
    cfg.requestedRecordedPoses = desiredPoseCount;
    cfg.requireRequestedPoseCount = true;
    cfg.maxTransitAngleStepDeg = 180.0; // direct target moves; no segmented transit
    cfg.useBothLaserPlaneSides = true;
    cfg.targetCenterYRadiusRatio = 0.40;
    cfg.maxRecordedPosesPerBCPair = 3; // 48-pose default remains achievable in conservative first-stage range

    // Before the 17-parameter model exists, the nominal-machine Stage-1 result is a
    // navigation approximation. Avoid extrapolating far outside the actually observed
    // Stage-1 B/C envelope. Offline Stage-1 JSON keeps those captured poses separately,
    // so use them too rather than silently losing this protection after a restart.
    if (!m_hasResult) {
        double sbMin=+1.0e9,sbMax=-1.0e9,scMin=+1.0e9,scMax=-1.0e9;
        int n=0;
        for (const auto& r:m_records) {
            if (!r.stage1Bootstrap) continue;
            ++n;
            sbMin=std::min(sbMin,r.usedPose.b); sbMax=std::max(sbMax,r.usedPose.b);
            scMin=std::min(scMin,r.usedPose.c); scMax=std::max(scMax,r.usedPose.c);
        }
        if (n==0) {
            for (const auto& q:m_loadedStage1CapturedPoses) {
                ++n;
                sbMin=std::min(sbMin,q.b); sbMax=std::max(sbMax,q.b);
                scMin=std::min(scMin,q.c); scMax=std::max(scMax,q.c);
            }
        }
        if (n>=8) {
            const double oldBMin=cfg.bMinDeg,oldBMax=cfg.bMaxDeg,oldCMin=cfg.cMinDeg,oldCMax=cfg.cMaxDeg;
            cfg.bMinDeg=std::max(cfg.bMinDeg,sbMin-5.0); cfg.bMaxDeg=std::min(cfg.bMaxDeg,sbMax+5.0);
            cfg.cMinDeg=std::max(cfg.cMinDeg,scMin-8.0); cfg.cMaxDeg=std::min(cfg.cMaxDeg,scMax+8.0);
            if (cfg.bMinDeg<cfg.bMaxDeg && cfg.cMinDeg<cfg.cMaxDeg &&
                (cfg.bMinDeg!=oldBMin || cfg.bMaxDeg!=oldBMax || cfg.cMinDeg!=oldCMin || cfg.cMaxDeg!=oldCMax)) {
                appendLog(tr("首次阶段2计划采用阶段1覆盖保护：界面请求B=[%1,%2] C=[%3,%4]，实际首批限制为B=[%5,%6] C=[%7,%8]。完成17参数求解后再生成计划将使用界面完整范围。")
                          .arg(oldBMin,0,'f',1).arg(oldBMax,0,'f',1).arg(oldCMin,0,'f',1).arg(oldCMax,0,'f',1)
                          .arg(cfg.bMinDeg,0,'f',1).arg(cfg.bMaxDeg,0,'f',1).arg(cfg.cMinDeg,0,'f',1).arg(cfg.cMaxDeg,0,'f',1));
            }
        }
    }

    if (!(cfg.xMinMm < cfg.xMaxMm && cfg.yMinMm < cfg.yMaxMm && cfg.zMinMm < cfg.zMaxMm &&
          cfg.bMinDeg <= cfg.bMaxDeg && cfg.cMinDeg <= cfg.cMaxDeg)) {
        QMessageBox::warning(this, tr("阶段2规划范围无效"),
            tr("Machine5X/控制器限位与界面B/C范围求交后为空，请检查限位和B/C设置。"));
        return;
    }

    PosePlannerSeed seed;
    seed.hasTaughtJoint = haveLiveStartPose;
    if (haveLiveStartPose)
        seed.taughtJoint = SingleSphere17QtAdapter::toPortable(startPose);
    seed.hasSphereCenterMachine = true;
    seed.sphereCenterMachineMm = m_sphereCenterGuessMachine;
    if (haveLiveStartPose) {
        seed.taughtSphereCenterSensorMm = SingleSphere17Model::machinePointInSensor(
            seed.taughtJoint, seed.sphereCenterMachineMm, m_currentEstimate);
    }

    SphereStudyPathOptions opticalOptions;
    opticalOptions.sphereRadiusMm=0.5*cfg.sphereDiameterMm;
    opticalOptions.sensorZMinMm=cfg.sensorZMinMm;
    opticalOptions.sensorZMaxMm=cfg.sensorZMaxMm;
    opticalOptions.sensorZSafetyMarginMm=1.0;
    opticalOptions.sensorXMinMm=-25.0;
    opticalOptions.sensorXMaxMm=25.0;
    opticalOptions.sensorXSafetyMarginMm=1.0;
    opticalOptions.useTrapezoidalSensorField=true;
    opticalOptions.sensorZCoordinateFlipped=m_referenceFlipZ;
    opticalOptions.sensorXSpanNearMm=42.0;
    opticalOptions.sensorXSpanFarMm=58.0;
    opticalOptions.requireEndEffectorFacingArc=true;
    opticalOptions.minimumPlaneIntersectionMarginMm=0.50;
    opticalOptions.minimumPredictedVisibleArcDeg=75.0;
    opticalOptions.minimumPredictedVisibleArcLengthMm=6.0;
    opticalOptions.visibleArcSamples=720;

    // V27.6.25: the worker-thread execution from V27.6.24 is retained, but
    // dense grids now use hierarchical planning: cheap raw geometry -> bounded
    // optical/information shortlist -> D-optimal selection with lazy exact
    // MachineSafety.  No worker thread reads mutable UI state directly.
    m_stage2PlanRunning=true;
    m_stage2PlanStartMs=QDateTime::currentMSecsSinceEpoch();
    m_stage2PlanPhaseStartMs=m_stage2PlanStartMs;
    m_stage2PlanPhase=Physical17PlanningPhase::Preparing;
    m_stage2PlanCancelFlag=std::make_shared<std::atomic_bool>(false);
    if (m_btnImportStage2Sphere) m_btnImportStage2Sphere->setEnabled(false);
    if (m_btnFitStage2Sphere) m_btnFitStage2Sphere->setEnabled(false);
    if (m_stage2PlanProgress) {
        m_stage2PlanProgress->setRange(0,100);
        m_stage2PlanProgress->setValue(0);
        m_stage2PlanProgress->setFormat(tr("姿态生成 %p%"));
    }
    if (m_lblStage2PlanProgress)
        m_lblStage2PlanProgress->setText(tr("姿态生成已启动：正在后台准备候选网格..."));
    appendLog(tr("阶段2姿态生成已转入后台线程：密集网格采用快速粗筛→shortlist光学/信息矩阵→D-optimal延迟MachineSafety；界面保持响应并可随时取消。"));
    updateCalibrationReadiness();

    const auto estimate=m_currentEstimate;
    const auto sphereCenter=m_sphereCenterGuessMachine;
    const auto poseSafetyChecker=m_stage2PoseSafetyChecker;
    const auto transitionSafetyChecker=m_stage2TransitionSafetyChecker;
    const auto cancelFlag=m_stage2PlanCancelFlag;
    QPointer<SingleSphere14CalibrationWidget> self(this);

    auto postProgress=[self](Physical17PlanningProgress p) {
        if (!self) return;
        QMetaObject::invokeMethod(self,[self,p]() {
            if (self) self->updateStage2PlanProgressUi(p);
        },Qt::QueuedConnection);
    };

    auto* watcher=new QFutureWatcher<Stage2PlanGenerationResult>(this);
    m_stage2PlanWatcher=watcher;
    connect(watcher,&QFutureWatcher<Stage2PlanGenerationResult>::finished,this,
            [this,watcher,cancelFlag,haveLiveStartPose,seed,cfg]() mutable {
        const Stage2PlanGenerationResult task=watcher->result();
        if (m_stage2PlanWatcher==watcher) m_stage2PlanWatcher=nullptr;
        watcher->deleteLater();

        const qint64 elapsedMs=QDateTime::currentMSecsSinceEpoch()-m_stage2PlanStartMs;
        const bool cancelRequested=task.cancelled ||
            (cancelFlag && cancelFlag->load(std::memory_order_relaxed));
        m_stage2PlanRunning=false;
        m_stage2PlanCancelFlag.reset();
        if (m_btnCancelStage2Plan) m_btnCancelStage2Plan->setEnabled(false);
        if (m_btnImportStage2Sphere) m_btnImportStage2Sphere->setEnabled(true);
        if (m_btnFitStage2Sphere) m_btnFitStage2Sphere->setEnabled(!m_stage2ImportedSphereCloud.isEmpty());

        appendLog(tr("D-optimal分层规划完成：模式=%1；shortlist光学=%2次/剔除%3，精确目标安全=%4次/剔除%5，精确过渡=%6次/拒绝%7；耗时=%8 s。")
            .arg(haveLiveStartPose?tr("在线-含当前姿态首段"):tr("离线-首姿态无虚假起点，后续段全检查"))
            .arg(task.opticalChecked).arg(task.opticalRejected)
            .arg(task.targetChecked).arg(task.targetRejected)
            .arg(task.transitionChecked).arg(task.transitionRejected)
            .arg(elapsedMs/1000.0,0,'f',2));

        if (cancelRequested) {
            if (m_lblStage2PlanProgress)
                m_lblStage2PlanProgress->setText(tr("姿态生成已取消；耗时 %1 s。旧计划（如有）保持不变。")
                    .arg(elapsedMs/1000.0,0,'f',1));
            appendLog(tr("阶段2姿态生成已由操作员取消；旧计划未被覆盖。"));
            updateCalibrationReadiness();
            updateWorkflowGuide();
            return;
        }

        if (task.plan.empty()) {
            if (m_lblStage2PlanProgress)
                m_lblStage2PlanProgress->setText(tr("姿态生成失败；耗时 %1 s：%2")
                    .arg(elapsedMs/1000.0,0,'f',1)
                    .arg(QString::fromStdString(task.error)));
            updateCalibrationReadiness();
            QMessageBox::warning(this,tr("阶段2姿态生成失败"),QString::fromStdString(task.error));
            return;
        }

        for (auto& record:m_records) {
            if (!record.stage1Bootstrap) record.planIndex=-1;
        }
        m_plan=task.plan;
        for (auto& pose:m_plan) {
            pose.note=std::string("Hierarchical shortlist optical preflight + lazy exact MachineSafety target/transition checks passed. ")
                +pose.note;
        }
        appendLog(tr("%1后台分层规划通过：保留%2个姿态。光学可见弧只在有界shortlist上精检；真正进入计划的姿态均已通过精确MachineSafety目标检查，计划内部相邻过渡均已做精确扫掠检查。")
            .arg(haveLiveStartPose?tr("在线"):tr("离线"))
            .arg(m_plan.size()));
        m_planValidation=false;
        m_planBootstrap=false;
        m_fullPlanGenerated=true;
        m_refinementSolved=false;
        m_needFitBeforeTransit=false;
        updatePlanTable();
        if (m_planTable->rowCount()>0) m_planTable->selectRow(0);

        m_btnGenerate->setText(m_hasResult
            ? tr("用当前17参数结果重新生成 D-optimal 姿态")
            : tr("用阶段1结果重新生成17参数 D-optimal 姿态"));
        if (m_stage2PlanProgress) m_stage2PlanProgress->setValue(100);
        if (m_lblStage2PlanProgress)
            m_lblStage2PlanProgress->setText(tr("姿态生成完成：%1 个姿态；总耗时 %2 s。")
                .arg(m_plan.size()).arg(elapsedMs/1000.0,0,'f',2));
        appendLog(tr("阶段2已%1生成 %2 个质量约束 + D-optimal记录姿态。候选几何：D=[%3,%4] mm，γ=[%5,%6]°，η=[%7,%8]°；"
                     "规划使用%9和固定球心M=%10。全部保留姿态已通过光学可见弧预检和MachineSafety目标检查；%11。")
                  .arg(haveLiveStartPose?tr("在线"):tr("离线"))
                  .arg(m_plan.size())
                  .arg(cfg.qualityDMinMm,0,'f',1).arg(cfg.qualityDMaxMm,0,'f',1)
                  .arg(cfg.qualityGammaMinDeg,0,'f',1).arg(cfg.qualityGammaMaxDeg,0,'f',1)
                  .arg(cfg.qualityEtaMinDeg,0,'f',1).arg(cfg.qualityEtaMaxDeg,0,'f',1)
                  .arg(m_hasResult?tr("当前17参数模型"):tr("阶段1名义机床手眼模型"))
                  .arg(centerText(seed.sphereCenterMachineMm))
                  .arg(haveLiveStartPose
                       ? tr("包含当前实机姿态到首姿态的扫掠检查")
                       : tr("离线没有真实当前姿态，因此不检查虚构的起始段；计划内部所有相邻段仍已扫掠检查，真实执行前还会从当时MPOS重新做首段安全门禁")));
        updateCalibrationReadiness();
        updateWorkflowGuide();
    });

    watcher->setFuture(QtConcurrent::run([
        cfg,seed,estimate,sphereCenter,opticalOptions,
        poseSafetyChecker,transitionSafetyChecker,cancelFlag,postProgress]() mutable -> Stage2PlanGenerationResult {
        Stage2PlanGenerationResult out;
        try {
            Physical17PlanningChecks checks;
            // Medium-cost optical simulation runs only on the bounded shortlist.
            checks.shortlistAllowed=[&](const JointPose5d& q,const Vec3d&) {
                if (cancelFlag && cancelFlag->load(std::memory_order_relaxed)) return false;
                ++out.opticalChecked;
                const auto optical=SphereDistanceAngleStudy::simulateOpticalFeasibility(
                    q,sphereCenter,opticalOptions.sphereRadiusMm,estimate,opticalOptions);
                if (!optical.valid) { ++out.opticalRejected; return false; }
                return true;
            };
            // Expensive STL/FCL target safety is lazy: only candidates that are
            // about to enter the D-optimal sequence reach this callback.
            checks.poseAllowed=[&](const JointPose5d& q,const Vec3d&) {
                if (cancelFlag && cancelFlag->load(std::memory_order_relaxed)) return false;
                if (!poseSafetyChecker) return true;
                ++out.targetChecked;
                QString reason;
                if (!poseSafetyChecker(controllerPoseFromPortable(q),reason)) {
                    ++out.targetRejected;
                    return false;
                }
                return true;
            };
            checks.transitionAllowed=[&](const JointPose5d& from,const JointPose5d& to) {
                if (cancelFlag && cancelFlag->load(std::memory_order_relaxed)) return false;
                if (!transitionSafetyChecker) return true;
                ++out.transitionChecked;
                QString reason;
                if (!transitionSafetyChecker(
                        controllerPoseFromPortable(from),controllerPoseFromPortable(to),reason)) {
                    ++out.transitionRejected;
                    return false;
                }
                return true;
            };

            SingleSphere17PosePlanner planner(cfg);
            out.plan=planner.generate(seed,estimate,out.error,checks,postProgress,cancelFlag.get());
            out.cancelled=cancelFlag && cancelFlag->load(std::memory_order_relaxed);
            return out;
        } catch (const std::exception& e) {
            out.error=std::string("Stage-2 pose planning exception: ")+e.what();
        } catch (...) {
            out.error="Stage-2 pose planning failed with an unknown exception.";
        }
        out.cancelled=cancelFlag && cancelFlag->load(std::memory_order_relaxed);
        return out;
    }));
}

void SingleSphere14CalibrationWidget::onCancelStage2PlanGeneration()
{
    if (!m_stage2PlanRunning || !m_stage2PlanCancelFlag) return;
    m_stage2PlanCancelFlag->store(true,std::memory_order_relaxed);
    if (m_btnCancelStage2Plan) m_btnCancelStage2Plan->setEnabled(false);
    if (m_lblStage2PlanProgress)
        m_lblStage2PlanProgress->setText(tr("正在取消姿态生成... 等待当前候选/安全检查返回。"));
    appendLog(tr("已请求取消阶段2姿态生成。"));
}



bool SingleSphere14CalibrationWidget::showStage2SimulationPose(int planIndex, QString& error)
{
    if (planIndex < 0 || planIndex >= static_cast<int>(m_plan.size())) {
        error = tr("运动学标定仿真索引越界");
        return false;
    }
    if (!m_hasBootstrap) {
        error = tr("运动学标定仿真需要阶段1手眼/球心结果");
        return false;
    }

    const auto& instruction = m_plan[static_cast<std::size_t>(planIndex)];
    const auto& q = instruction.predictedJoint;

    SphereStudyPathOptions options;
    options.sphereRadiusMm = 0.5 * m_spinSphereDiameter->value();
    options.sensorZMinMm = -48.0;
    options.sensorZMaxMm = 48.0;
    options.sensorZSafetyMarginMm = 1.0;
    options.sensorXMinMm = -25.0;
    options.sensorXMaxMm = 25.0;
    options.sensorXSafetyMarginMm = 1.0;
    options.useTrapezoidalSensorField = true;
    options.sensorZCoordinateFlipped = m_referenceFlipZ;
    options.sensorXSpanNearMm = 42.0;
    options.sensorXSpanFarMm = 58.0;
    options.requireEndEffectorFacingArc = true;
    options.minimumPlaneIntersectionMarginMm = 0.50;
    options.minimumPredictedVisibleArcDeg = 75.0;
    options.minimumPredictedVisibleArcLengthMm = 6.0;
    options.visibleArcSamples = 720;

    // Keep the playback geometry identical to the actual Stage-2 planning gate.
    QString plannerWarning;
    const auto cfg = plannerConfigFromUi(plannerWarning);
    options.sphereRadiusMm = 0.5 * cfg.sphereDiameterMm;
    options.sensorZMinMm = cfg.sensorZMinMm;
    options.sensorZMaxMm = cfg.sensorZMaxMm;

    const auto sim = SphereDistanceAngleStudy::buildScanSimulation(
        q, m_sphereCenterGuessMachine, options.sphereRadiusMm,
        m_currentEstimate, options, 720);

    auto toMachine = [&](const std::vector<Vec3d>& source) {
        QVector<QVector3D> points;
        points.reserve(static_cast<int>(source.size()));
        for (const auto& p : source) {
            const auto m = SingleSphere17Model::mapSensorPoint(q, p, m_currentEstimate);
            if (std::isfinite(m.x) && std::isfinite(m.y) && std::isfinite(m.z)) {
                points.push_back(QVector3D(static_cast<float>(m.x),
                                           static_cast<float>(m.y),
                                           static_cast<float>(m.z)));
            }
        }
        return points;
    };

    const QVector<QVector3D> laserFieldMachine = toMachine(sim.laserFieldCornersSensorMm);
    const QVector<QVector3D> sectionCircleMachine = toMachine(sim.sectionCircleSensorMm);
    const QVector<QVector3D> visibleArcMachine = toMachine(sim.visibleArcSensorMm);

    if (sim.valid && !visibleArcMachine.isEmpty()) {
        m_stage2SimulationCloud += visibleArcMachine;
    }

    // Also show the same predicted arc in the main 2-D profile view.  This makes
    // it easy to judge whether the generated pose produces the expected circle,
    // rather than only inspecting the 3-D digital twin.
    QVector<QPointF> visibleArc2d;
    visibleArc2d.reserve(static_cast<int>(sim.visibleArcSensorMm.size()));
    for (const auto& p : sim.visibleArcSensorMm) {
        if (std::isfinite(p.x) && std::isfinite(p.z))
            visibleArc2d.push_back(QPointF(p.x, p.z));
    }
    const auto sphereS = SingleSphere17Model::machinePointInSensor(
        q, m_sphereCenterGuessMachine, m_currentEstimate);
    const double rr = options.sphereRadiusMm * options.sphereRadiusMm - sphereS.y * sphereS.y;
    const double sectionRadius = rr > 0.0 ? std::sqrt(rr) : 0.0;
    // Keep the 2-D prediction fresh in the background, but never force the
    // main view back to the 2-D tab on every simulation frame.
    emit requestProfileOverlay(visibleArc2d, sphereS.x, sphereS.z, sectionRadius);

    const QString status = tr("运动学标定仿真 %1/%2：B=%3° C=%4°；%5；可见弧=%6° / %7 mm；累计仿真点=%8")
        .arg(planIndex + 1).arg(m_plan.size())
        .arg(q.b, 0, 'f', 2).arg(q.c, 0, 'f', 2)
        .arg(QString::fromStdString(sim.message))
        .arg(sim.optical.visibleArcDeg, 0, 'f', 1)
        .arg(sim.optical.visibleArcLengthMm, 0, 'f', 2)
        .arg(m_stage2SimulationCloud.size());

    emit requestStage2SimulationFrame(controllerPoseFromPortable(q),
                                      laserFieldMachine,
                                      sectionCircleMachine,
                                      visibleArcMachine,
                                      m_stage2SimulationCloud,
                                      sim.valid,
                                      status);
    appendLog(status);
    if (m_planTable && planIndex < m_planTable->rowCount()) m_planTable->selectRow(planIndex);
    return true;
}

void SingleSphere14CalibrationWidget::onStartStage2Simulation()
{
    if (m_stage2SimulationActive) return;
    if (!m_hasBootstrap || m_plan.empty() || m_planValidation) {
        QMessageBox::information(this, tr("运动学标定仿真"),
            tr("请先生成阶段2质量约束 + D-optimal 标定姿态。"));
        return;
    }

    m_stage2SimulationCloud.clear();
    m_stage2SimulationIndex = 0;
    m_stage2SimulationActive = true;
    ++m_stage2SimulationToken;
    emit requestStage2SimulationClear(true);
    // Start from the 3-D digital twin once.  Subsequent frames update both the
    // 3-D scene and 2-D overlay without changing the currently selected view.
    emit requestStage2SimulationView();
    appendLog(tr("运动学标定姿态仿真已启动：默认进入三维点云；二维轮廓与三维数字孪生将同步更新，播放期间可自由切换视图；不会向控制器发送运动命令。"));
    updateCalibrationReadiness();
    advanceStage2Simulation();
}

void SingleSphere14CalibrationWidget::advanceStage2Simulation()
{
    if (!m_stage2SimulationActive) return;
    if (m_stage2SimulationIndex >= static_cast<int>(m_plan.size())) {
        finishStage2Simulation(tr("运动学标定姿态仿真完成：%1 个姿态，累计预测圆弧点=%2。")
                               .arg(m_plan.size()).arg(m_stage2SimulationCloud.size()));
        return;
    }

    const quint64 token = m_stage2SimulationToken;
    QString error;
    if (!showStage2SimulationPose(m_stage2SimulationIndex, error)) {
        finishStage2Simulation(tr("运动学标定姿态仿真中止：%1").arg(error));
        return;
    }
    ++m_stage2SimulationIndex;
    const int interval = m_spinStage2SimulationIntervalMs
        ? m_spinStage2SimulationIntervalMs->value() : 800;
    QTimer::singleShot(interval, this, [this, token]() {
        if (m_stage2SimulationActive && token == m_stage2SimulationToken)
            advanceStage2Simulation();
    });
}

void SingleSphere14CalibrationWidget::finishStage2Simulation(const QString& reason)
{
    m_stage2SimulationActive = false;
    m_stage2SimulationIndex = 0;
    ++m_stage2SimulationToken;
    appendLog(reason);
    updateCalibrationReadiness();
}

void SingleSphere14CalibrationWidget::onStopStage2Simulation()
{
    if (!m_stage2SimulationActive) return;
    finishStage2Simulation(tr("运动学标定姿态仿真已停止；已保留累计预测圆弧点=%1，可继续在三维视图检查覆盖。")
                           .arg(m_stage2SimulationCloud.size()));
}

void SingleSphere14CalibrationWidget::onClearStage2Simulation()
{
    if (m_stage2SimulationActive) return;
    m_stage2SimulationCloud.clear();
    emit requestStage2SimulationClear(true);
    emit requestProfileOverlayClear();
    appendLog(tr("运动学标定仿真显示和累计预测圆弧点云已清除。"));
    updateCalibrationReadiness();
}

void SingleSphere14CalibrationWidget::onGenerateValidationPlan()
{
    if (!m_hasResult || !m_lastResult.success) {
        QMessageBox::warning(this, tr("不能生成验证姿态"),
            tr("请先完成阶段2最终17参数求解。独立验证必须针对一个冻结后的最终模型进行。"));
        return;
    }
    if (m_records.empty() && stage2TrainingObservations().empty()) {
        QMessageBox::warning(this, tr("缺少阶段2训练姿态来源"),
            tr("当前17参数是从JSON恢复的，但没有加载对应的阶段2训练CSV。为了保证hold-out验证姿态与训练网格真正错开，"
               "请先导入对应的阶段2训练CSV，再生成独立验证姿态。"));
        return;
    }
    if (!m_validationRecords.empty() || m_validationSummary.computed) {
        if (QMessageBox::question(this, tr("重新生成独立验证计划"),
            tr("重新生成验证姿态会清空当前hold-out验证数据和验证结论，但不会改变训练数据或17参数结果。\n\n继续？"),
            QMessageBox::Yes|QMessageBox::No, QMessageBox::No) != QMessageBox::Yes) return;
        invalidateValidation(tr("开始新的独立验证计划，旧hold-out数据已清空。"), true);
    }
    // V27.6.42B: validation-pose DESIGN is read-only, exactly like Stage-2
    // D-optimal planning.  Importing a Stage-2 CSV deliberately leaves the live
    // calibration/motion session unbound, so requiring m_referenceTaught here made
    // the "生成独立验证姿态" button unusable immediately after a successful offline
    // re-solve.  Generate an offline plan when no authenticated live start pose is
    // available; real execution still has to re-bind conventions and re-check the
    // current MPOS -> first-target MachineSafety transition.
    const bool controllerOnline = m_controller && m_controller->isConnected();
    const bool sensorOnline = m_sensor && m_sensor->isConnected();
    const bool liveLinearReference = controllerOnline && m_controller->areLinearMposReferenced();
    const bool liveRotaryReference = controllerOnline
        && m_controller->isRotaryAbsoluteReady(AxisC)
        && m_controller->isRotaryAbsoluteReady(AxisB);
    const bool haveLiveStartPose = m_referenceTaught && controllerOnline && sensorOnline
        && liveLinearReference && liveRotaryReference;

    JointState5D state{};
    if (haveLiveStartPose) {
        QString conventionError;
        if (!validateSessionConventions(conventionError)) {
            QMessageBox::warning(this, tr("坐标约定已改变"), conventionError);
            return;
        }
        QString stateError;
        if (!readStableState(state, stateError)) {
            QMessageBox::warning(this, tr("不能生成验证姿态"), stateError);
            return;
        }
    } else {
        appendLog(tr("独立验证姿态按离线模式生成：当前没有已认证的实机起点，因此不会伪造XYZCB起点，也不会把计划生成本身当作运动授权；"
                     "首个验证姿态将在真正自动/手动执行前，以当时实机MPOS重新做坐标约定和MachineSafety首段检查。"));
    }

    QString warning;
    auto cfg = plannerConfigFromUi(warning);
    if (!warning.isEmpty()) appendLog(warning);

    // Keep the same finite Machine5X planning envelope used by Stage-2 planning.
    // Offline hold-out generation must never fall back to the planner's intentionally
    // broad placeholder XYZ limits.
    QString boundsSource;
    QString boundsError;
    if (m_stage2PlanningBoundsResolver) {
        if (!m_stage2PlanningBoundsResolver(cfg, boundsSource, boundsError)) {
            QMessageBox::warning(this, tr("不能生成验证姿态"),
                boundsError.isEmpty() ? tr("无法取得有效的五轴规划限位。") : boundsError);
            return;
        }
        if (!boundsSource.isEmpty()) appendLog(tr("独立验证规划限位：%1").arg(boundsSource));
    } else if (!haveLiveStartPose) {
        QMessageBox::warning(this, tr("不能离线生成验证姿态"),
            tr("当前没有Machine5X离线规划限位提供器。离线hold-out规划禁止使用未约束XYZ范围。"));
        return;
    }

    const double bStep = std::max(1.0, std::abs(m_spinBStep->value()));
    const double cStep = std::max(1.0, std::abs(m_spinCStep->value()));

    // Hold-out plan uses a half-step shifted B/C lattice and different sensor
    // target centers.  It therefore tests interpolation/generalization rather
    // than simply repeating the D-optimal training grid.
    if (cfg.bMaxDeg - cfg.bMinDeg > bStep) {
        cfg.bMinDeg += 0.5 * bStep;
        cfg.bMaxDeg -= 0.5 * bStep;
    }
    if (cfg.cMaxDeg - cfg.cMinDeg > cStep) {
        cfg.cMinDeg += 0.5 * cStep;
        cfg.cMaxDeg -= 0.5 * cStep;
    }
    cfg.bStepDeg = bStep;
    cfg.cStepDeg = cStep;
    cfg.useQualityStudyGeometry = false; // use the independent X/Z/Y targets below
    cfg.targetCenterXmm = {-3.5, +3.5};
    cfg.targetCenterZmm = {90.0, 100.0};
    cfg.targetCenterYRadiusRatio = 0.25;
    cfg.useBothLaserPlaneSides = true;
    cfg.maxRecordedPosesPerBCPair = 1;
    cfg.maxTransitAngleStepDeg = 180.0;
    const int wanted = m_spinValidationPoseCount->value();
    // V27.6.42 may remove entire B/C neighborhoods around Stage-2 rejected poses.
    // Ask the planner for a wider candidate pool so safe hold-out coverage remains
    // available after training-grid separation and hazard-zone filtering.
    cfg.requestedRecordedPoses = std::max(48, wanted * 8);

    PosePlannerSeed seed;
    seed.hasTaughtJoint = haveLiveStartPose;
    if (haveLiveStartPose) {
        seed.taughtJoint = SingleSphere17QtAdapter::toPortable(
            calibrationPoseFromState(state, m_referenceFeedbackMask));
    }
    seed.hasSphereCenterMachine = true;
    seed.sphereCenterMachineMm = m_lastResult.sphereCenterMachineMm;
    if (haveLiveStartPose) {
        seed.taughtSphereCenterSensorMm = SingleSphere17Model::machinePointInSensor(
            seed.taughtJoint, seed.sphereCenterMachineMm, m_lastResult.parameters);
    }

    SingleSphere17PosePlanner planner(cfg);
    std::string error;
    const auto candidates = planner.generate(seed, m_lastResult.parameters, error);
    if (candidates.empty()) {
        QMessageBox::warning(this, tr("验证姿态生成失败"), QString::fromStdString(error));
        return;
    }

    const double minSep = m_spinValidationBCSep->value();
    const bool avoidRejected = m_checkValidationAvoidRejected && m_checkValidationAvoidRejected->isChecked()
        && !m_lastResult.rejectedTrainingPoses.empty();
    const double hazardSep = m_spinValidationHazardBCSep ? m_spinValidationHazardBCSep->value() : 10.0;
    int hazardRejectedCandidates = 0;
    std::vector<CalibrationPoseInstruction> validationPlan;
    validationPlan.reserve(static_cast<std::size_t>(wanted));
    const auto importedTraining = m_records.empty() ? stage2TrainingObservations()
                                                    : std::vector<SphereProfileObservation>{};
    for (const auto& candidate : candidates) {
        if (avoidRejected) {
            JointPose5D q = controllerPoseFromPortable(candidate.predictedJoint);
            QString hazardReason;
            if (!validationPoseOutsideRejectedTrainingHazards(q, hazardSep, &hazardReason)) {
                ++hazardRejectedCandidates;
                continue;
            }
        }
        bool separated = true;
        if (!m_records.empty()) {
            for (const auto& training : m_records) {
                if (training.stage1Bootstrap) continue;
                const double db = candidate.predictedJoint.b - training.usedPose.b;
                const double dc = candidate.predictedJoint.c - training.usedPose.c;
                if (std::hypot(db, dc) < minSep) {
                    separated = false;
                    break;
                }
            }
        } else {
            for (const auto& training : importedTraining) {
                const double db = candidate.predictedJoint.b - training.joints.b;
                const double dc = candidate.predictedJoint.c - training.joints.c;
                if (std::hypot(db, dc) < minSep) {
                    separated = false;
                    break;
                }
            }
        }
        if (!separated) continue;
        for (const auto& existing : validationPlan) {
            const double db = candidate.predictedJoint.b - existing.predictedJoint.b;
            const double dc = candidate.predictedJoint.c - existing.predictedJoint.c;
            if (std::hypot(db, dc) < 0.5 * minSep) {
                separated = false;
                break;
            }
        }
        if (!separated) continue;
        validationPlan.push_back(candidate);
        if (static_cast<int>(validationPlan.size()) >= wanted) break;
    }

    if (validationPlan.size() < 6) {
        QMessageBox::warning(this, tr("独立验证覆盖不足"),
            tr("只找到 %1 个满足验证约束的安全姿态。训练网格最小B/C间隔=%2°；"
               "已知高误差B/C危险区排除=%3（半径=%4°，过滤候选=%5）。\n"
               "请优先扩大B/C可用范围或适当减小危险区半径；不要为了凑数量把已知高误差区重新作为验证区。")
                .arg(validationPlan.size()).arg(minSep,0,'f',1)
                .arg(avoidRejected?tr("开启"):tr("未开启/无筛除记录"))
                .arg(hazardSep,0,'f',1).arg(hazardRejectedCandidates));
        return;
    }

    if (static_cast<int>(validationPlan.size()) < wanted) {
        appendLog(tr("验证计划请求%1个姿态，但经过训练集隔离筛选后只保留%2个；仍可执行，但覆盖度低于目标。")
                  .arg(wanted).arg(validationPlan.size()));
        m_spinValidationPoseCount->setValue(static_cast<int>(validationPlan.size()));
    }

    m_plan = std::move(validationPlan);
    QString validationPreflight;
    JointPose5D validationStart{};
    if (haveLiveStartPose)
        validationStart=calibrationPoseFromState(state,m_referenceFeedbackMask);
    if (!preflightAndFilterGeneratedPlan(validationStart,haveLiveStartPose,
                                         static_cast<int>(m_plan.size()),validationPreflight)) {
        m_plan.clear();
        QMessageBox::warning(this,tr("验证姿态预检不足"),validationPreflight);
        updatePlanTable();
        updateCalibrationReadiness();
        return;
    }
    appendLog(validationPreflight);
    m_planValidation = true;
    m_needFitBeforeTransit = false;
    updatePlanTable();
    if (m_planTable->rowCount() > 0) m_planTable->selectRow(0);
    appendLog(tr("已%1生成%2个独立验证姿态：B/C网格相对训练网格半步错开，且每个验证姿态与全部训练记录的B/C距离≥%3°；"
                 "高误差危险区排除=%4，危险B/C半径=%5°，因此额外过滤候选=%6；目标球心也使用独立的X/Z/Y位置。"
                 "验证采集不会进入17参数求解器。注意：若启用危险区排除，验证结论只覆盖保留的安全B/C工作域，不代表被排除区域有效。")
              .arg(haveLiveStartPose?tr("在线"):tr("离线"))
              .arg(m_plan.size()).arg(minSep,0,'f',1)
              .arg(avoidRejected?tr("开启"):tr("未开启/无筛除记录"))
              .arg(hazardSep,0,'f',1).arg(hazardRejectedCandidates));
    updateValidationSummary();
    updateCalibrationReadiness();
    updateWorkflowGuide();
}

void SingleSphere14CalibrationWidget::updatePlanTable()
{
    QSignalBlocker blocker(m_planTable);
    m_planTable->setRowCount(static_cast<int>(m_plan.size()));
    for (int i = 0; i < static_cast<int>(m_plan.size()); ++i) {
        const auto& p = m_plan[static_cast<std::size_t>(i)];
        const auto& q = p.predictedJoint;
        const QStringList values = {
            QString::number(p.sequence),
            QString::number(q.x,'f',3), QString::number(q.y,'f',3), QString::number(q.z,'f',3),
            QString::number(q.c,'f',2), QString::number(q.b,'f',2),
            centerText(p.targetSphereCenterSensorMm), QString::number(p.informationGain,'g',5),
            QString::fromStdString(p.group), m_planValidation ? tr("待验证") : tr("待采集")
        };
        for (int c = 0; c < values.size(); ++c) {
            auto* item = new QTableWidgetItem(values[c]);
            if (c >= 1 && c <= 3) {
                item->setToolTip(tr("XYZ由阶段1手眼+固定球心模型（或后续17参数模型）预测。点击移动后对XYZCB分别下发独立绝对目标；各轴使用本轴速度参数，控制器软限位/MachineSafety检查仍保留。"));
            }
            m_planTable->setItem(i, c, item);
        }
    }
}

int SingleSphere14CalibrationWidget::selectedPlanIndex() const
{
    const auto rows = m_planTable->selectionModel() ? m_planTable->selectionModel()->selectedRows() : QModelIndexList{};
    if (rows.isEmpty()) return -1;
    return rows.first().row();
}

void SingleSphere14CalibrationWidget::onPlanSelectionChanged()
{
    const int i = selectedPlanIndex();
    if (i < 0 || i >= static_cast<int>(m_plan.size())) return;
    const auto& p = m_plan[static_cast<std::size_t>(i)];
    const double radius = 0.5 * m_spinSphereDiameter->value();
    const double d = std::hypot(p.targetSphereCenterSensorMm.x, p.targetSphereCenterSensorMm.z);
    const double gamma = radius > 0.0
        ? std::asin(std::clamp(std::abs(p.targetSphereCenterSensorMm.y) / radius, 0.0, 1.0)) * 180.0 / 3.14159265358979323846
        : 0.0;
    const double eta = std::atan2(p.targetSphereCenterSensorMm.x, p.targetSphereCenterSensorMm.z)
        * 180.0 / 3.14159265358979323846;
    const QString purpose = m_planValidation ? tr("独立验证") : tr("阶段2标定");
    const QString captureAction = m_planValidation
        ? tr("到位后点击“采集当前姿态为验证集”；数据不会参与求解。")
        : tr("到位后点击“采集当前姿态”加入17参数训练集。");
    m_lblSolveState->setText(tr(
        "选中%1姿态 %2/%3：D=%4 mm，γ=%5°，η=%6°；预测XYZ=[%7,%8,%9] mm，C=%10°，B=%11°；目标球心S=%12。\n"
        "点击“直接移动到选中计划姿态”后，XYZCB按各自轴速度执行独立绝对定位；五轴全部停稳后自动检查一次球拟合。"
        "无合格轮廓时该姿态会跳过并选中下一条，自动模式会继续下一姿态。%13")
        .arg(purpose)
        .arg(i+1).arg(m_plan.size())
        .arg(d,0,'f',2).arg(gamma,0,'f',2).arg(eta,0,'f',2)
        .arg(p.predictedJoint.x,0,'f',3).arg(p.predictedJoint.y,0,'f',3).arg(p.predictedJoint.z,0,'f',3)
        .arg(p.predictedJoint.c,0,'f',2).arg(p.predictedJoint.b,0,'f',2)
        .arg(centerText(p.targetSphereCenterSensorMm)).arg(captureAction));
}

void SingleSphere14CalibrationWidget::stopAutoPlanMove(const QString& reason)
{
    m_autoPlanMove = false;
    m_autoPlanIndex = -1;
    if (!m_moveInProgress) m_btnMove->setEnabled(true);
    if (!reason.isEmpty()) {
        m_lblSolveState->setText(reason);
        appendLog(reason);
    }
}

void SingleSphere14CalibrationWidget::continueAutoPlanMove()
{
    if (!m_autoPlanMove || m_moveInProgress) return;
    const int i = m_autoPlanIndex;
    if (i < 0 || i >= static_cast<int>(m_plan.size())) {
        stopAutoPlanMove(m_planValidation ? tr("验证姿态自动移动取消：计划索引失效。")
                                          : tr("阶段2自动移动取消：计划索引失效。"));
        return;
    }
    if (m_needFitBeforeTransit) {
        stopAutoPlanMove(m_planValidation
            ? tr("验证姿态自动移动已暂停：当前球拟合未通过安全视场检查。请先找回球再继续。")
            : tr("阶段2自动移动已暂停：当前球拟合未通过安全视场检查。请先用小幅手动XYZ或“按当前球心自动微调XYZ”找回球，再重新点击移动。"));
        return;
    }

    QString conventionError;
    if (!validateSessionConventions(conventionError)) {
        stopAutoPlanMove(tr("阶段2自动移动取消：%1").arg(conventionError));
        return;
    }

    JointState5D state;
    QString error;
    if (!readStableState(state, error)) {
        stopAutoPlanMove(tr("阶段2自动移动取消：%1").arg(error));
        return;
    }
    const JointPose5D current = calibrationPoseFromState(state, m_referenceFeedbackMask);
    const auto& finalTarget = m_plan[static_cast<std::size_t>(i)].predictedJoint;

    const double dx = finalTarget.x - current.x;
    const double dy = finalTarget.y - current.y;
    const double dz = finalTarget.z - current.z;
    const double db = finalTarget.b - current.b;
    const double dc = finalTarget.c - current.c;
    const double maxAngleRemain = std::max(std::abs(db), std::abs(dc));
    const double maxLinearRemain = std::max({std::abs(dx), std::abs(dy), std::abs(dz)});

    if (maxAngleRemain <= 0.15 && maxLinearRemain <= 0.30) {
        markPlanStatus(i, m_planValidation ? tr("验证姿态已到位，待验证采集") : tr("计划姿态已到位，待采集"));
        stopAutoPlanMove(m_planValidation
            ? tr("独立验证姿态%1已到达模型预测XYZCB目标。请确认球拟合质量，然后点击“采集当前姿态为验证集”。").arg(i+1)
            : tr("阶段2姿态%1已到达模型预测XYZCB目标。请确认当前球拟合质量，然后点击“采集当前姿态”。").arg(i+1));
        return;
    }

    const double maxAngleStep = std::max(0.5, m_spinTransitAngleStep->value());
    const double maxLinearStep = std::max(0.5, std::min(3.0, 3.0 * m_spinRecenterMaxStep->value()));
    double alpha = 1.0;
    if (maxAngleRemain > maxAngleStep) alpha = std::min(alpha, maxAngleStep / maxAngleRemain);
    if (maxLinearRemain > maxLinearStep) alpha = std::min(alpha, maxLinearStep / maxLinearRemain);

    JointPose5D target;
    target.x = current.x + alpha * dx;
    target.y = current.y + alpha * dy;
    target.z = current.z + alpha * dz;
    target.c = current.c + alpha * dc;
    target.b = current.b + alpha * db;

    m_moveInProgress = true;
    m_pendingMoveKind = PendingMoveKind::TransitBC;
    m_pendingMovePlanIndex = i;
    m_btnMove->setEnabled(false);
    markPlanStatus(i, tr("自动分段移动中"));
    m_lblSolveState->setText(tr("%1姿态%2：自动子步 %3 -> %4。到位后将自动重新拟合并决定是否继续。")
                             .arg(m_planValidation ? tr("验证") : tr("阶段2"))
                             .arg(i+1).arg(poseText(current), poseText(target)));
    const double transitScale = std::min(m_spinStage2XyzSpeedScale->value(),
                                         m_spinStage2BcSpeedScale->value());
    emit requestJointMove(target, std::min(0.15, transitScale));
}

bool SingleSphere14CalibrationWidget::selectNextPendingPlanRow(int afterIndex)
{
    if(!m_planTable||m_plan.empty())return false;
    auto eligible=[this](int i){
        auto* item=m_planTable->item(i,9);
        const QString st=item?item->text():QString();
        return !(st.contains(tr("已采集")) || st.contains(tr("验证已采集")) ||
                 st.contains(tr("已跳过")) || st.contains(tr("移动失败")) ||
                 st.contains(tr("安全门禁失败")));
    };
    for(int pass=0;pass<2;++pass){
        const int begin=pass==0?afterIndex+1:0;
        const int finish=pass==0?static_cast<int>(m_plan.size()):std::min(afterIndex+1,static_cast<int>(m_plan.size()));
        for(int i=begin;i<finish;++i){if(eligible(i)){m_planTable->selectRow(i);return true;}}
    }
    return false;
}

void SingleSphere14CalibrationWidget::onSkipSelectedPose()
{
    if(m_moveInProgress||m_collecting)return;
    const int i=selectedPlanIndex(); if(i<0||i>=static_cast<int>(m_plan.size()))return;
    markPlanStatus(i,tr("已跳过"),tr("未加入数据集"));
    appendLog(tr("%1姿态%2已人工跳过；该姿态不会加入%3数据集。")
              .arg(m_planValidation?tr("验证"):tr("阶段2")).arg(i+1)
              .arg(m_planValidation?tr("hold-out"):tr("训练")));
    if(!selectNextPendingPlanRow(i))m_lblSolveState->setText(tr("当前计划中没有剩余待处理姿态。"));
}

bool SingleSphere14CalibrationWidget::startDirectMoveToPlanIndex(
    int i, bool askConfirmation, QString* failureReason,
    Stage2MovePreflightFailure* failureKind)
{
    if (failureReason) failureReason->clear();
    if (failureKind) *failureKind = Stage2MovePreflightFailure::None;
    auto fail = [&](Stage2MovePreflightFailure kind, const QString& reason) {
        if (failureReason) *failureReason = reason;
        if (failureKind) *failureKind = kind;
        return false;
    };
    if (m_moveInProgress || m_autoPlanMove || m_collecting)
        return fail(Stage2MovePreflightFailure::GlobalReadiness, tr("标定运动/采集任务仍在进行，请等待当前任务结束。"));
    if (!m_hasBootstrap) {
        const QString reason = tr("阶段2计划姿态必须建立在阶段1手眼+球心初标结果上。");
        if (askConfirmation) QMessageBox::warning(this, tr("阶段1尚未完成"), reason);
        return fail(Stage2MovePreflightFailure::GlobalReadiness, reason);
    }
    if (i < 0 || i >= static_cast<int>(m_plan.size())) {
        const QString reason = tr("请先生成并选择一个计划姿态。");
        if (askConfirmation) QMessageBox::information(this, tr("没有姿态"), reason);
        return fail(Stage2MovePreflightFailure::GlobalReadiness, reason);
    }
    QString conventionError;
    if (!validateSessionConventions(conventionError)) {
        if (askConfirmation) QMessageBox::warning(this, tr("坐标约定已改变"), conventionError);
        return fail(Stage2MovePreflightFailure::GlobalReadiness, conventionError);
    }
    JointState5D state;
    QString error;
    if (!checkStage2LiveMotionReadiness(error, &state)) {
        updateStage2MotionReadinessUi();
        if (askConfirmation) QMessageBox::warning(this, tr("真实运动未就绪"), error);
        return fail(Stage2MovePreflightFailure::GlobalReadiness, error);
    }
    const JointPose5D current = calibrationPoseFromState(state, m_referenceFeedbackMask);
    const JointPose5D finalTarget = controllerPoseFromPortable(m_plan[static_cast<std::size_t>(i)].predictedJoint);
    if (m_stage2PoseSafetyChecker && !m_stage2PoseSafetyChecker(finalTarget, error)) {
        markPlanStatus(i, tr("已跳过：目标安全门禁失败"), error);
        appendLog(tr("姿态%1执行前安全门禁拒绝目标：%2").arg(i+1).arg(error));
        return fail(Stage2MovePreflightFailure::TargetSafety, tr("目标姿态MachineSafety拒绝：%1").arg(error));
    }
    if (m_stage2TransitionSafetyChecker && !m_stage2TransitionSafetyChecker(current, finalTarget, error)) {
        markPlanStatus(i, tr("已跳过：路径碰撞门禁失败"), error);
        appendLog(tr("姿态%1执行前扫掠路径门禁拒绝：%2").arg(i+1).arg(error));
        return fail(Stage2MovePreflightFailure::TransitionSafety,
                    tr("当前姿态到目标姿态的MachineSafety扫掠路径被拒绝：%1").arg(error));
    }
    if (askConfirmation) {
        const QString confirm = tr(
            "%1\n\n当前：%2\n目标：%3\n\n"
            "该目标已通过生成阶段的光学可见弧预检，并在可用时通过MachineSafety目标/过渡路径检查；"
            "真正下发前还会按独立轴实际有效速度轨迹再做一次碰撞门禁。到达后会自动检查球轮廓质量。\n\n继续？")
            .arg(m_planValidation ? tr("这是独立验证姿态；数据只进入hold-out验证集。")
                                  : tr("这是阶段2训练姿态；上次有效17参数仅作为导航初值/运行回退。"))
            .arg(poseText(current)).arg(poseText(finalTarget));
        if (QMessageBox::question(this, m_planValidation ? tr("确认验证姿态移动") : tr("确认阶段2移动"), confirm,
                                  QMessageBox::Yes|QMessageBox::No, QMessageBox::No) != QMessageBox::Yes)
            return fail(Stage2MovePreflightFailure::UserCancelled, tr("操作者取消了本次移动。"));
    }
    m_needFitBeforeTransit = false;
    m_autoPlanMove = false;
    m_autoPlanIndex = -1;
    m_moveInProgress = true;
    m_pendingMoveKind = PendingMoveKind::PlanDirect;
    m_pendingMovePlanIndex = i;
    if (m_btnMove) m_btnMove->setEnabled(false);
    markPlanStatus(i, (m_autoStage2Acquisition || m_autoValidationAcquisition) ? tr("自动移动中") : tr("直接移动中"));
    m_lblSolveState->setText(tr("%1姿态%2：五轴独立绝对定位到 %3；XYZ倍率=%4，B/C倍率=%5；不做五轴插补；全部停稳后自动检查球。")
                             .arg(m_planValidation?tr("验证"):tr("阶段2")).arg(i+1).arg(poseText(finalTarget))
                             .arg(m_spinStage2XyzSpeedScale->value(),0,'f',2)
                             .arg(m_spinStage2BcSpeedScale->value(),0,'f',2));
    updateCalibrationReadiness();
    emit requestIndependentJointMove(finalTarget,
                                     m_spinStage2XyzSpeedScale->value(),
                                     m_spinStage2BcSpeedScale->value());
    return true;
}

void SingleSphere14CalibrationWidget::onMoveToSelectedPose()
{
    if (m_autoStage2Acquisition || m_autoValidationAcquisition) return;
    (void)startDirectMoveToPlanIndex(selectedPlanIndex(), true);
}

void SingleSphere14CalibrationWidget::onStartAutomaticStage2Acquisition()
{
    if (m_autoStage2Acquisition || m_autoValidationAcquisition || m_moveInProgress || m_collecting) return;
    if (m_planValidation) {
        QMessageBox::information(this, tr("当前是独立验证计划"),
            tr("当前表格是独立hold-out验证计划。本按钮只执行阶段2训练自动采集；请使用阶段3的“自动移动 + 采集独立验证”。"));
        return;
    }
    if (!m_referenceTaught || !m_hasBootstrap || m_plan.empty()) {
        QMessageBox::warning(this, tr("不能自动采集"), tr("请先绑定当前设备坐标约定，并生成阶段2质量约束+D-optimal计划。"));
        return;
    }
    QString readinessError;
    if (!checkStage2LiveMotionReadiness(readinessError, nullptr)) {
        updateStage2MotionReadinessUi();
        QMessageBox::warning(this, tr("真实运动未就绪"),
            tr("阶段2自动采集尚未启动，计划姿态不会被标记为失败或跳过。\n\n%1\n\n"
               "请点击“打开控制页：XYZ/五轴回零、重读B/C、人工使能”恢复条件后再启动。")
                .arg(readinessError));
        return;
    }
    if (!m_sensor || !m_sensor->isProfileRunning()) {
        QMessageBox::warning(this, tr("不能自动采集"), tr("请先启动单球标定轮廓预览。自动流程需要到位后立即做轮廓质量检查和fresh-profile采集。"));
        return;
    }
    const auto reply = QMessageBox::question(this, tr("启动阶段2自动移动/采集"),
        tr("程序将按计划逐姿态执行：\n"
           "1) 执行前再次检查MachineSafety目标姿态；真正下发前再按各轴独立速度轨迹检查整段路径；\n"
           "2) 到位后RANSAC检查球弧；轮廓流需要运行，但无需把主视图停留在二维轮廓Tab；\n"
           "3) 自动采集fresh profiles；只保留RANSAC内点，并裁掉圆弧两端低质量点；\n"
           "4) RMS/P95/内点比例/有效帧比例不满足门禁时整个位姿丢弃并继续下一姿态。\n\n"
           "请确认急停和机床安全条件已就绪。是否开始？"),
        QMessageBox::Yes|QMessageBox::No, QMessageBox::No);
    if (reply != QMessageBox::Yes) return;

    m_autoStage2Acquisition = true;
    m_autoStage2StopRequested = false;
    if (m_btnAutoAcquireStage2) m_btnAutoAcquireStage2->setEnabled(false);
    if (m_btnStopAutoAcquireStage2) m_btnStopAutoAcquireStage2->setEnabled(true);
    appendLog(tr("阶段2自动移动/采集开始。"));
    continueAutomaticStage2Acquisition();
}

void SingleSphere14CalibrationWidget::onStopAutomaticStage2Acquisition()
{
    stopAutomaticStage2Acquisition(tr("操作者停止了阶段2自动移动/采集。"));
    if (m_moveInProgress) emit requestJointStop(false);
}

void SingleSphere14CalibrationWidget::stopAutomaticStage2Acquisition(const QString& reason)
{
    m_autoStage2Acquisition = false;
    m_autoStage2StopRequested = true;
    if (m_btnStopAutoAcquireStage2) m_btnStopAutoAcquireStage2->setEnabled(false);
    if (m_btnAutoAcquireStage2) m_btnAutoAcquireStage2->setEnabled(!m_plan.empty() && !m_planValidation);
    if (!reason.isEmpty()) {
        m_lblSolveState->setText(reason);
        appendLog(reason);
    }
    updateCalibrationReadiness();
}

void SingleSphere14CalibrationWidget::continueAutomaticStage2Acquisition()
{
    if (!m_autoStage2Acquisition || m_autoStage2StopRequested || m_moveInProgress || m_collecting || m_capturePendingStart) return;
    if (m_planValidation || m_plan.empty()) {
        stopAutomaticStage2Acquisition(tr("阶段2自动采集结束：没有可执行的训练计划。"));
        return;
    }

    int index = selectedPlanIndex();
    auto pending = [this](int i) {
        if (i < 0 || i >= m_planTable->rowCount()) return false;
        const auto* item = m_planTable->item(i, 9);
        const QString st = item ? item->text() : QString();
        return !(st.contains(tr("已采集")) || st.contains(tr("已跳过")) ||
                 st.contains(tr("移动失败")) || st.contains(tr("安全门禁失败")));
    };
    if (!pending(index)) {
        if (!selectNextPendingPlanRow(index)) {
            stopAutomaticStage2Acquisition(tr("阶段2自动采集完成：当前计划已无待处理姿态。"));
            return;
        }
        index = selectedPlanIndex();
    }

    QString failureReason;
    Stage2MovePreflightFailure failureKind = Stage2MovePreflightFailure::None;
    if (!startDirectMoveToPlanIndex(index, false, &failureReason, &failureKind)) {
        if (failureKind == Stage2MovePreflightFailure::TargetSafety ||
            failureKind == Stage2MovePreflightFailure::TransitionSafety) {
            if (!selectNextPendingPlanRow(index)) {
                stopAutomaticStage2Acquisition(tr("阶段2自动采集结束：剩余待处理姿态均被MachineSafety目标/路径门禁拒绝。"));
                return;
            }
            QTimer::singleShot(50, this, &SingleSphere14CalibrationWidget::continueAutomaticStage2Acquisition);
            return;
        }
        const QString reason = failureReason.isEmpty() ? tr("未知的全局执行前条件失败。") : failureReason;
        stopAutomaticStage2Acquisition(
            tr("阶段2自动采集暂停：%1 当前姿态及剩余姿态均保留为待处理；恢复运动准备条件后可直接重新启动自动采集。")
                .arg(reason));
        updateStage2MotionReadinessUi();
    }
}

void SingleSphere14CalibrationWidget::onStartAutomaticValidationAcquisition()
{
    if (m_autoValidationAcquisition || m_autoStage2Acquisition || m_moveInProgress || m_collecting) return;
    if (!m_planValidation || m_plan.empty()) {
        QMessageBox::information(this, tr("没有独立验证计划"),
            tr("请先完成阶段2最终求解并点击“生成独立验证姿态”。V27.6.42会自动避开阶段2筛出的高误差B/C危险区。"));
        return;
    }
    if (!m_hasResult || !m_lastResult.success) {
        QMessageBox::warning(this, tr("不能自动验证采集"),
            tr("独立验证自动采集需要存在冻结后的成功Physical17结果。"));
        return;
    }
    if (!m_referenceTaught) {
        QString bindError;
        if (!tryBindCurrentSolvedResultForCalibrationMotion(bindError)) {
            QMessageBox::warning(this, tr("不能重新绑定验证运动会话"), bindError);
            updateCalibrationReadiness();
            return;
        }
        updateSessionSummary();
        updateCalibrationReadiness();
        updateWorkflowGuide();
    }
    QString readinessError;
    if (!checkStage2LiveMotionReadiness(readinessError, nullptr)) {
        updateStage2MotionReadinessUi();
        QMessageBox::warning(this, tr("真实运动未就绪"), readinessError);
        return;
    }
    if (!m_sensor || !m_sensor->isProfileRunning()) {
        QMessageBox::warning(this, tr("不能自动验证采集"),
            tr("请先启动单球标定轮廓预览。自动验证需要到位后执行球拟合质量门禁和fresh-profile采集。"));
        return;
    }

    // Re-check every pending validation row against the current rejected-pose
    // hazard map.  This catches the case where a newer Stage-2 result was loaded
    // after an older validation plan had already been generated.
    for (std::size_t i = 0; i < m_plan.size(); ++i) {
        const JointPose5D q = controllerPoseFromPortable(m_plan[i].predictedJoint);
        QString separationError;
        if (!validationPoseSeparated(q, separationError)) {
            QMessageBox::warning(this, tr("验证计划包含危险/非独立姿态"),
                tr("验证姿态%1不能用于自动采集：\n%2\n\n请重新生成独立验证姿态。")
                    .arg(i + 1).arg(separationError));
            return;
        }
    }

    const auto reply = QMessageBox::question(this, tr("启动独立验证自动移动/采集"),
        tr("程序将自动执行当前hold-out计划：\n"
           "1) 每个姿态仍经过MachineSafety目标/过渡路径门禁；\n"
           "2) 运动使用与阶段2相同的独立轴绝对定位：X/Y/Z/C/B分别下发Single_MoveAbs，各轴使用自己的速度/加减速度，不做五轴插补；\n"
           "3) 五轴全部Idle后检查球弧，随后自动采集fresh profiles；\n"
           "4) 质量不合格的验证姿态只会被跳过，不会进入训练求解器；\n"
           "5) 已知阶段2高误差B/C危险区不会作为验证区域。\n\n"
           "请确认急停、软限位和机械安全条件已就绪。是否开始？"),
        QMessageBox::Yes|QMessageBox::No, QMessageBox::No);
    if (reply != QMessageBox::Yes) return;

    m_autoValidationAcquisition = true;
    m_autoValidationStopRequested = false;
    if (m_btnAutoAcquireValidation) m_btnAutoAcquireValidation->setEnabled(false);
    if (m_btnStopAutoAcquireValidation) m_btnStopAutoAcquireValidation->setEnabled(true);
    appendLog(tr("独立验证自动移动/采集开始：使用阶段2同一独立轴运动入口，验证数据只进入hold-out集合。"));
    continueAutomaticValidationAcquisition();
}

void SingleSphere14CalibrationWidget::onStopAutomaticValidationAcquisition()
{
    stopAutomaticValidationAcquisition(tr("操作者停止了独立验证自动移动/采集。"));
    if (m_moveInProgress) emit requestJointStop(false);
}

void SingleSphere14CalibrationWidget::stopAutomaticValidationAcquisition(const QString& reason)
{
    m_autoValidationAcquisition = false;
    m_autoValidationStopRequested = true;
    if (m_btnStopAutoAcquireValidation) m_btnStopAutoAcquireValidation->setEnabled(false);
    if (m_btnAutoAcquireValidation) m_btnAutoAcquireValidation->setEnabled(m_planValidation && !m_plan.empty());
    if (!reason.isEmpty()) {
        m_lblSolveState->setText(reason);
        appendLog(reason);
    }
    updateCalibrationReadiness();
}

void SingleSphere14CalibrationWidget::continueAutomaticValidationAcquisition()
{
    if (!m_autoValidationAcquisition || m_autoValidationStopRequested || m_moveInProgress || m_collecting || m_capturePendingStart) return;
    if (!m_planValidation || m_plan.empty()) {
        stopAutomaticValidationAcquisition(tr("独立验证自动采集结束：当前没有可执行的hold-out计划。"));
        return;
    }

    int index = selectedPlanIndex();
    auto pending = [this](int i) {
        if (i < 0 || i >= m_planTable->rowCount()) return false;
        const auto* item = m_planTable->item(i, 9);
        const QString st = item ? item->text() : QString();
        return !(st.contains(tr("验证已采集")) || st.contains(tr("已采集")) ||
                 st.contains(tr("已跳过")) || st.contains(tr("移动失败")) ||
                 st.contains(tr("安全门禁失败")));
    };
    if (!pending(index)) {
        if (!selectNextPendingPlanRow(index)) {
            stopAutomaticValidationAcquisition(
                tr("独立验证自动采集完成：当前计划已无待处理姿态；已采集%1/%2组，可点击“执行独立验证”。")
                    .arg(m_validationRecords.size())
                    .arg(m_spinValidationPoseCount ? m_spinValidationPoseCount->value() : 8));
            return;
        }
        index = selectedPlanIndex();
    }

    QString failureReason;
    Stage2MovePreflightFailure failureKind = Stage2MovePreflightFailure::None;
    if (!startDirectMoveToPlanIndex(index, false, &failureReason, &failureKind)) {
        if (failureKind == Stage2MovePreflightFailure::TargetSafety ||
            failureKind == Stage2MovePreflightFailure::TransitionSafety) {
            if (!selectNextPendingPlanRow(index)) {
                stopAutomaticValidationAcquisition(tr("独立验证自动采集结束：剩余验证姿态均被MachineSafety目标/路径门禁拒绝。"));
                return;
            }
            QTimer::singleShot(50, this, &SingleSphere14CalibrationWidget::continueAutomaticValidationAcquisition);
            return;
        }
        const QString reason = failureReason.isEmpty() ? tr("未知的全局执行前条件失败。") : failureReason;
        stopAutomaticValidationAcquisition(
            tr("独立验证自动采集暂停：%1 当前及剩余验证姿态保持待处理；恢复条件后可以继续启动自动验证采集。")
                .arg(reason));
        updateStage2MotionReadinessUi();
    }
}

void SingleSphere14CalibrationWidget::notifyJointMoveFinished(bool ok, const QString& message, const JointState5D& finalState)
{
    const PendingMoveKind moveKind = m_pendingMoveKind;
    const int movePlanIndex = m_pendingMovePlanIndex;
    m_pendingMoveKind = PendingMoveKind::None;
    m_pendingMovePlanIndex = -1;
    m_moveInProgress = false;
    updateCurrentPoseLabel(&finalState);

    if (!ok) {
        if (moveKind == PendingMoveKind::Stage1CandidateDirect) {
            if (movePlanIndex >= 0) setStage1CandidateStatus(movePlanIndex, tr("移动失败：%1").arg(message));
            appendLog(tr("阶段1候选姿态移动失败：%1").arg(message));
        } else {
            if (movePlanIndex >= 0) markPlanStatus(movePlanIndex, tr("移动失败"), message);
            stopAutoPlanMove(tr("标定姿态移动失败：%1").arg(message));
        }
        if (m_autoStage2Acquisition && moveKind == PendingMoveKind::PlanDirect) {
            appendLog(tr("自动阶段2：姿态%1移动失败，本轮不再自动重试；可在自动流程结束后手动选择该姿态重试。原因：%2")
                      .arg(movePlanIndex+1).arg(message));
            const bool hasNext = movePlanIndex >= 0 && selectNextPendingPlanRow(movePlanIndex);
            if (hasNext) {
                QTimer::singleShot(50, this, &SingleSphere14CalibrationWidget::continueAutomaticStage2Acquisition);
            } else {
                stopAutomaticStage2Acquisition(tr("阶段2自动采集结束：其余姿态均已采集、跳过或本轮执行失败。失败姿态仍可手动选择后重试。"));
            }
        } else if (m_autoValidationAcquisition && moveKind == PendingMoveKind::PlanDirect) {
            appendLog(tr("自动独立验证：姿态%1移动失败，本轮跳过并继续；原因：%2")
                      .arg(movePlanIndex+1).arg(message));
            const bool hasNext = movePlanIndex >= 0 && selectNextPendingPlanRow(movePlanIndex);
            if (hasNext) {
                QTimer::singleShot(50, this, &SingleSphere14CalibrationWidget::continueAutomaticValidationAcquisition);
            } else {
                stopAutomaticValidationAcquisition(tr("独立验证自动采集结束：其余姿态均已采集、跳过或本轮执行失败。"));
            }
        } else {
            QMessageBox::warning(this,tr("五轴移动失败"),message);
        }
        updateCalibrationReadiness();
        updateWorkflowGuide();
        updateStage1Status();
        return;
    }

    if (moveKind == PendingMoveKind::PlanDirect) {
        const JointPose5D current = calibrationPoseFromState(finalState, m_referenceFeedbackMask);
        if (movePlanIndex >= 0) markPlanStatus(movePlanIndex, tr("已到位，自动检查球"));
        appendLog(tr("%1姿态%2直接移动完成：%3；执行一次轮廓拟合检查。")
                  .arg(m_planValidation?tr("验证"):tr("阶段2")).arg(movePlanIndex+1).arg(poseText(current)));
        if (m_sensor && m_sensor->isProfileRunning()) {
            QTimer::singleShot(300, this, [this, movePlanIndex]() {
                onFitCurrentProfile();
                if (!m_hasLatestFit || m_latestFit.circleRmsMm > m_spinMaxCircleRms->value()) {
                    if (movePlanIndex >= 0) markPlanStatus(movePlanIndex, tr("已跳过：无合格球拟合"));
                    appendLog(tr("%1姿态%2到位后没有合格球拟合：本姿态已跳过，不加入数据集。")
                              .arg(m_planValidation?tr("验证"):tr("阶段2")).arg(movePlanIndex+1));
                    const bool next=selectNextPendingPlanRow(movePlanIndex);
                    if (m_autoStage2Acquisition && !m_planValidation) {
                        if (next) QTimer::singleShot(50, this, &SingleSphere14CalibrationWidget::continueAutomaticStage2Acquisition);
                        else stopAutomaticStage2Acquisition(tr("阶段2自动采集完成：没有剩余待处理姿态。"));
                    } else if (m_autoValidationAcquisition && m_planValidation) {
                        if (next) QTimer::singleShot(50, this, &SingleSphere14CalibrationWidget::continueAutomaticValidationAcquisition);
                        else stopAutomaticValidationAcquisition(tr("独立验证自动采集完成：没有剩余待处理姿态。"));
                    } else {
                        m_lblSolveState->setText(next
                            ? tr("姿态%1无合格轮廓，已跳过并自动选中下一姿态；请确认后点击“直接移动”。").arg(movePlanIndex+1)
                            : tr("姿态%1无合格轮廓，已跳过；当前计划没有剩余待处理姿态。").arg(movePlanIndex+1));
                    }
                } else {
                    if (movePlanIndex >= 0) markPlanStatus(movePlanIndex, m_planValidation?tr("验证姿态已到位，待验证采集"):tr("计划姿态已到位，待采集"),
                        tr("RMS=%1 mm；arc=%2°").arg(m_latestFit.circleRmsMm,0,'g',5).arg(m_latestFit.arcSpanDeg,0,'f',1));
                    if (m_autoStage2Acquisition && !m_planValidation) {
                        m_lblSolveState->setText(tr("阶段2姿态%1预检查合格；自动开始fresh-profile正式采集。").arg(movePlanIndex+1));
                        startCapture(false);
                    } else if (m_autoValidationAcquisition && m_planValidation) {
                        m_lblSolveState->setText(tr("独立验证姿态%1预检查合格；自动开始fresh-profile hold-out采集。").arg(movePlanIndex+1));
                        startCapture(true);
                    } else {
                        m_lblSolveState->setText(m_planValidation
                            ? tr("独立验证姿态%1已到位且球拟合合格。点击“采集当前姿态为验证集”。").arg(movePlanIndex+1)
                            : tr("阶段2姿态%1已到位且球拟合合格。点击“采集当前姿态”。").arg(movePlanIndex+1));
                    }
                }
                if (m_btnMove) m_btnMove->setEnabled(!m_autoStage2Acquisition && !m_autoValidationAcquisition);
                updateCalibrationReadiness();
                updateWorkflowGuide();
            });
        } else {
            if (movePlanIndex >= 0) markPlanStatus(movePlanIndex, tr("已到位；轮廓流未运行"));
            if (m_autoStage2Acquisition) {
                stopAutomaticStage2Acquisition(tr("阶段2自动采集停止：轮廓预览未运行，无法执行质量门禁。"));
            } else if (m_autoValidationAcquisition) {
                stopAutomaticValidationAcquisition(tr("独立验证自动采集停止：轮廓预览未运行，无法执行质量门禁。"));
            } else {
                m_lblSolveState->setText(tr("已到位，但轮廓预览未运行，无法自动判断球是否可见。请先启动轮廓预览并拟合。"));
                m_btnMove->setEnabled(true);
            }
        }
    } else if (moveKind == PendingMoveKind::Stage1CandidateDirect) {
        const JointPose5D current = calibrationPoseFromState(finalState, m_referenceFeedbackMask);
        appendLog(tr("阶段1候选姿态%1直接移动完成：%2；自动检查球。")
                  .arg(movePlanIndex+1).arg(poseText(current)));
        if(m_sensor&&m_sensor->isProfileRunning()){
            QTimer::singleShot(300,this,[this,movePlanIndex](){
                onFitCurrentProfile();
                if(m_hasLatestFit&&m_latestFit.circleRmsMm<=m_spinMaxCircleRms->value())
                    setStage1CandidateStatus(movePlanIndex,tr("到位/球可见，待采集"));
                else
                    setStage1CandidateStatus(movePlanIndex,tr("到位/无合格球；请点动微调或跳过"));
                updateStage1Status();
            });
        }else{
            setStage1CandidateStatus(movePlanIndex,tr("已到位；请启动轮廓预览"));
            updateStage1Status();
        }
    } else if (moveKind == PendingMoveKind::TransitBC) {
        m_needFitBeforeTransit = true;
        const JointPose5D current = calibrationPoseFromState(finalState, m_referenceFeedbackMask);
        if (movePlanIndex >= 0) markPlanStatus(movePlanIndex, tr("子步完成，自动检查球"));
        appendLog(tr("%1自动子步完成：%2；等待新轮廓重新拟合。")
                  .arg(m_planValidation ? tr("验证姿态") : tr("阶段2"))
                  .arg(poseText(current)));

        if (m_sensor && m_sensor->isProfileRunning()) {
            QTimer::singleShot(300, this, [this, movePlanIndex]() {
                onFitCurrentProfile();
                if (!m_hasLatestFit || m_latestFit.circleRmsMm > m_spinMaxCircleRms->value()) {
                    m_needFitBeforeTransit = true;
                    if (movePlanIndex >= 0) markPlanStatus(movePlanIndex, tr("自动移动暂停：球拟合失败"));
                    stopAutoPlanMove(m_planValidation
                        ? tr("验证姿态自动移动已暂停：子步后没有合格球拟合。请先找回球，再重新启动该验证姿态。")
                        : tr("阶段2自动移动已暂停：子步后没有合格球拟合。请先用小幅手动XYZ找回球，再重新启动该计划姿态。"));
                    return;
                }
                if (m_needFitBeforeTransit) {
                    if (movePlanIndex >= 0) markPlanStatus(movePlanIndex, tr("自动移动暂停：球靠近视场边缘"));
                    stopAutoPlanMove(m_planValidation
                        ? tr("验证姿态自动移动已暂停：球仍可见但已靠近保守视场边缘。请先微调XYZ，拟合稳定后再继续。")
                        : tr("阶段2自动移动已暂停：球仍可见但已靠近保守视场边缘。请点“按当前球心自动微调XYZ”，拟合稳定后再继续。"));
                    return;
                }
                if (m_autoPlanMove && m_autoPlanIndex == movePlanIndex) {
                    continueAutoPlanMove();
                } else {
                    m_btnMove->setEnabled(true);
                }
            });
        } else {
            stopAutoPlanMove(tr("阶段2自动移动暂停：轮廓预览未运行，无法在子步之间验证标准球是否仍可见。"));
        }
    } else if (moveKind == PendingMoveKind::Stage1Nudge) {
        appendLog(tr("阶段1手动小步运动完成：%1").arg(poseText(calibrationPoseFromState(finalState, m_referenceFeedbackMask))));
        if (m_sensor && m_sensor->isProfileRunning()) {
            QTimer::singleShot(250, this, [this]() {
                onFitCurrentProfile();
                updateStage1Status();
            });
        } else {
            updateStage1Status();
        }
    } else if (moveKind == PendingMoveKind::RecenterXYZ) {
        m_needFitBeforeTransit = true;
        if (movePlanIndex >= 0) markPlanStatus(movePlanIndex, tr("XYZ找球一步完成，待重新拟合"));
        appendLog(tr("XYZ小步找球完成：%1；正在重新拟合。")
                  .arg(poseText(calibrationPoseFromState(finalState, m_referenceFeedbackMask))));
        if (m_sensor && m_sensor->isProfileRunning()) {
            QTimer::singleShot(300, this, [this]() {
                onFitCurrentProfile();
                m_btnMove->setEnabled(!m_moveInProgress && !m_autoPlanMove);
            });
        }
    } else {
        m_btnMove->setEnabled(true);
        appendLog(tr("标定姿态移动完成：%1")
                  .arg(poseText(calibrationPoseFromState(finalState, m_referenceFeedbackMask))));
    }
    updateCalibrationReadiness();
    updateWorkflowGuide();
    updateStage1Status();
}

void SingleSphere14CalibrationWidget::onRecommendRecenter()
{
    if (m_moveInProgress) return;
    const int i = selectedPlanIndex();
    if (i < 0 || i >= static_cast<int>(m_plan.size())) {
        QMessageBox::information(this,tr("没有姿态"),tr("请先生成并选择一个姿态。"));
        return;
    }
    QString conventionError;
    if (!validateSessionConventions(conventionError)) {
        QMessageBox::warning(this,tr("坐标约定已改变"),conventionError);
        return;
    }

    JointState5D state;
    QString error;
    if (!readStableState(state,error)) {
        QMessageBox::warning(this,tr("不能找球"),error);
        return;
    }
    const JointPose5D currentQt = calibrationPoseFromState(state, m_referenceFeedbackMask);
    const bool bcReached = selectedPoseBCReached(currentQt, i);

    onFitCurrentProfile();
    if (!m_hasLatestFit || m_latestFit.circleRmsMm > m_spinMaxCircleRms->value()) {
        QMessageBox::warning(this,tr("无法闭环找球"),
            tr("当前轮廓没有质量合格的球拟合。此时软件不知道球心误差方向，不能自动给XYZ修正。请先用很小的手动XYZ找回球。"));
        return;
    }

    Vec3d measured = m_latestFit.centerSensorMm;
    if (m_hasBootstrap) {
        // 阶段1以后固定球心和手眼模型已经能够给出当前传感器右手坐标系
        // 中的signed Y_S，因此不再需要人工选择±Y。
        const Vec3d sphereCenter = m_sphereCenterGuessMachine;
        const Vec3d modelCenter = SingleSphere17Model::machinePointInSensor(
            SingleSphere17QtAdapter::toPortable(currentQt), sphereCenter, m_currentEstimate);
        measured.y = std::copysign(std::abs(measured.y), modelCenter.y);
    }

    Vec3d target;
    if (bcReached) {
        // At the final B/C orientation, close the loop to the actual D-optimal
        // target center, including the requested off-plane |Y_S| excitation.
        target = m_plan[static_cast<std::size_t>(i)].targetSphereCenterSensorMm;
    } else {
        // During rotary transit, do not attempt the unobservable +/-Y move.
        // Only re-center X/Z and keep the currently measured signed Y_S.
        target = Vec3d{0.0, measured.y, 95.0};
    }

    JointPose5d suggestion;
    Vec3d predicted;
    std::string coreError;
    const double maxStep = m_spinRecenterMaxStep->value();
    const bool ok = SingleSphere17PosePlanner::recommendXYZRecentering(
        SingleSphere17QtAdapter::toPortable(currentQt), measured,
        target, m_currentEstimate, maxStep, suggestion, predicted, coreError);
    if (!ok) {
        QMessageBox::warning(this,tr("XYZ 找球计算失败"),QString::fromStdString(coreError));
        return;
    }

    JointPose5D targetJoint{suggestion.x,suggestion.y,suggestion.z,suggestion.c,suggestion.b};
    const QString modeText = bcReached
        ? tr("已经到达计划B/C：signed Y_S由阶段1/最终模型自动判断，现在向D-optimal目标球心微调。")
        : tr("尚未处于所选计划B/C：signed Y_S由阶段1模型自动判断，本次只做小量XYZ修正，不需要人工选择Y正负。");
    const QString msg = tr("当前球心S=%1\n本步目标球心S=%2\n建议下一XYZ小步：%3\n预测下一球心S=%4\n"
                           "最大XYZ步长=%5 mm。\n%6\n\n执行后必须重新拟合；只有新拟合通过且球位于安全窗口，才允许继续下一步B/C。")
        .arg(centerText(measured)).arg(centerText(target))
        .arg(poseText(targetJoint)).arg(centerText(predicted))
        .arg(maxStep,0,'f',2).arg(modeText);
    if (QMessageBox::question(this,tr("确认 XYZ 小步闭环"),msg,
                              QMessageBox::Yes|QMessageBox::No,QMessageBox::No) != QMessageBox::Yes) return;

    m_moveInProgress = true;
    m_pendingMoveKind = PendingMoveKind::RecenterXYZ;
    m_pendingMovePlanIndex = i;
    emit requestJointMove(targetJoint, std::min(0.10, m_spinStage2XyzSpeedScale->value()));
}

bool SingleSphere14CalibrationWidget::checkCaptureDrift(const JointState5D& before, const JointState5D& after,
                                                        QString& error) const
{
    const std::uint8_t mask = m_referenceTaught ? m_referenceFeedbackMask : 0;
    const JointPose5D qb = calibrationPoseFromState(before, mask);
    const JointPose5D qa = calibrationPoseFromState(after, mask);
    double maxLin=0.0, maxRot=0.0;
    for (int axis : {AxisX,AxisY,AxisZ})
        maxLin=std::max(maxLin,std::abs(poseValue(qb,axis)-poseValue(qa,axis)));
    for (int axis : {AxisC,AxisB})
        maxRot=std::max(maxRot,std::abs(poseValue(qb,axis)-poseValue(qa,axis)));
    if (!after.allIdle || maxLin > m_spinLinearDrift->value() || maxRot > m_spinRotaryDrift->value()) {
        error = tr("静态采集期间位置漂移超限：XYZ max=%1 mm（限%2），B/C max=%3°（限%4），Idle=%5。\n"
                   "漂移检查使用本会话锁定的数据源：%6")
            .arg(maxLin,0,'g',6).arg(m_spinLinearDrift->value())
            .arg(maxRot,0,'g',6).arg(m_spinRotaryDrift->value()).arg(after.allIdle?tr("是"):tr("否"))
            .arg(sourceMaskText(mask));
        return false;
    }
    return true;
}

void SingleSphere14CalibrationWidget::onCaptureCurrentPose()
{
    startCapture(false);
}

void SingleSphere14CalibrationWidget::onCaptureValidationPose()
{
    startCapture(true);
}

QString SingleSphere14CalibrationWidget::physical17Fingerprint(
    const SingleSphere17Result& result) const
{
    if (!result.success) return {};
    QCryptographicHash hash(QCryptographicHash::Sha256);
    hash.addData(QByteArrayLiteral("MotionLaserSync/Physical17/v1\n"));
    for (double value : result.parameters.value) {
        hash.addData(QString::number(value, 'g', 12).toUtf8());
        hash.addData(QByteArrayLiteral("\n"));
    }
    hash.addData(QString::number(result.sphereCenterMachineMm.x, 'g', 12).toUtf8());
    hash.addData(QByteArrayLiteral("\n"));
    hash.addData(QString::number(result.sphereCenterMachineMm.y, 'g', 12).toUtf8());
    hash.addData(QByteArrayLiteral("\n"));
    hash.addData(QString::number(result.sphereCenterMachineMm.z, 'g', 12).toUtf8());
    return QString::fromLatin1(hash.result().toHex());
}

QString SingleSphere14CalibrationWidget::trainingPoseFingerprint() const
{
    const auto observations = stage2TrainingObservations();
    if (observations.empty()) return {};

    std::vector<JointPose5d> poses;
    poses.reserve(observations.size());
    for (const auto& observation : observations) {
        bool duplicate = false;
        for (const auto& p : poses) {
            if (std::abs(observation.joints.x-p.x)<=0.03 && std::abs(observation.joints.y-p.y)<=0.03 &&
                std::abs(observation.joints.z-p.z)<=0.03 && std::abs(observation.joints.c-p.c)<=0.03 &&
                std::abs(observation.joints.b-p.b)<=0.03) {
                duplicate = true;
                break;
            }
        }
        if (!duplicate) poses.push_back(observation.joints);
    }
    std::sort(poses.begin(), poses.end(), [](const JointPose5d& a, const JointPose5d& b) {
        if (a.x != b.x) return a.x < b.x;
        if (a.y != b.y) return a.y < b.y;
        if (a.z != b.z) return a.z < b.z;
        if (a.c != b.c) return a.c < b.c;
        return a.b < b.b;
    });

    QCryptographicHash hash(QCryptographicHash::Sha256);
    hash.addData(QByteArrayLiteral("MotionLaserSync/Stage2TrainingPoseSet/v1\n"));
    hash.addData(QByteArray::number(static_cast<qulonglong>(poses.size())));
    hash.addData(QByteArrayLiteral("\n"));
    for (const auto& p : poses) {
        hash.addData(QStringLiteral("%1,%2,%3,%4,%5\n")
            .arg(p.x,0,'g',12).arg(p.y,0,'g',12).arg(p.z,0,'g',12)
            .arg(p.c,0,'g',12).arg(p.b,0,'g',12).toUtf8());
    }
    return QString::fromLatin1(hash.result().toHex());
}

bool SingleSphere14CalibrationWidget::writeValidationObservationCsv(
    const QString& fileName, QString& error) const
{
    if (!m_hasResult || !m_lastResult.success) {
        error = tr("没有冻结后的Physical17结果，不能导出独立验证数据。");
        return false;
    }
    if (m_validationRecords.empty()) {
        error = tr("独立验证集为空。");
        return false;
    }
    const QString modelFingerprint = physical17Fingerprint(m_lastResult);
    const QString trainingFingerprint = trainingPoseFingerprint();
    if (modelFingerprint.isEmpty() || trainingFingerprint.isEmpty()) {
        error = tr("缺少当前Physical17模型或阶段2训练姿态来源，无法生成可安全重载的hold-out CSV。");
        return false;
    }

    QDir parent = QFileInfo(fileName).dir();
    if (!parent.exists() && !parent.mkpath(QStringLiteral("."))) {
        error = tr("无法创建目录：%1").arg(parent.absolutePath());
        return false;
    }
    QSaveFile file(fileName);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        error = file.errorString();
        return false;
    }
    QTextStream out(&file);
    out.setRealNumberPrecision(15);
    out << "dataset_role,format_version,model_fingerprint,training_pose_fingerprint,validation_pose_id,source_profile_id,profile_id,"
           "x_mm,y_mm,z_mm,c_deg,b_deg,"
           "mpos_x_mm,mpos_y_mm,mpos_z_mm,mpos_c_deg,mpos_b_deg,"
           "dpos_x_mm,dpos_y_mm,dpos_z_mm,dpos_c_deg,dpos_b_deg,"
           "sensor_x_mm,sensor_z_mm,weight\n";

    qint64 csvProfileId = 1;
    for (std::size_t poseIndex = 0; poseIndex < m_validationRecords.size(); ++poseIndex) {
        const auto& record = m_validationRecords[poseIndex];
        for (const auto& observation : record.observations) {
            const qint64 exportedProfileId = csvProfileId++;
            for (const auto& point : observation.points) {
                out << "holdout_validation,1," << modelFingerprint << ',' << trainingFingerprint << ','
                    << static_cast<qulonglong>(poseIndex) << ',' << observation.profileId << ',' << exportedProfileId << ','
                    << observation.joints.x << ',' << observation.joints.y << ',' << observation.joints.z << ','
                    << observation.joints.c << ',' << observation.joints.b << ','
                    << observation.joints.x << ',' << observation.joints.y << ',' << observation.joints.z << ','
                    << observation.joints.c << ',' << observation.joints.b << ',';
                if (observation.hasDposJoints) {
                    out << observation.dposJoints.x << ',' << observation.dposJoints.y << ',' << observation.dposJoints.z << ','
                        << observation.dposJoints.c << ',' << observation.dposJoints.b << ',';
                } else {
                    out << ",,,,,";
                }
                out
                    << point.x << ',' << point.z << ',' << point.weight << '\n';
            }
        }
    }
    if (out.status() != QTextStream::Ok) {
        error = tr("写入验证CSV失败。");
        file.cancelWriting();
        return false;
    }
    if (!file.commit()) {
        error = file.errorString();
        return false;
    }
    error.clear();
    return true;
}

bool SingleSphere14CalibrationWidget::loadValidationObservationCsv(
    const QString& fileName, std::vector<CaptureRecord>& records, QString& error) const
{
    records.clear();
    if (!m_hasResult || !m_lastResult.success) {
        error = tr("请先加载或求得冻结后的Physical17结果，再导入独立验证CSV。");
        return false;
    }
    const QString expectedModel = physical17Fingerprint(m_lastResult);
    const QString expectedTraining = trainingPoseFingerprint();
    if (expectedTraining.isEmpty()) {
        error = tr("当前没有阶段2训练姿态来源。为了重新检查hold-out与训练网格的独立性，请先导入与该Physical17结果对应的阶段2训练CSV。");
        return false;
    }

    QFile metadataFile(fileName);
    if (!metadataFile.open(QIODevice::ReadOnly | QIODevice::Text)) {
        error = metadataFile.errorString();
        return false;
    }
    QTextStream metadata(&metadataFile);
    if (metadata.atEnd()) {
        error = tr("验证CSV为空。");
        return false;
    }
    const QStringList headers = metadata.readLine().trimmed().split(',', Qt::KeepEmptyParts);
    QMap<QString,int> column;
    for (int i=0; i<headers.size(); ++i) column.insert(headers[i].trimmed(), i);
    const QStringList required = {
        QStringLiteral("dataset_role"), QStringLiteral("format_version"),
        QStringLiteral("model_fingerprint"), QStringLiteral("training_pose_fingerprint"),
        QStringLiteral("validation_pose_id"), QStringLiteral("profile_id"),
        QStringLiteral("x_mm"), QStringLiteral("y_mm"), QStringLiteral("z_mm"),
        QStringLiteral("c_deg"), QStringLiteral("b_deg"),
        QStringLiteral("sensor_x_mm"), QStringLiteral("sensor_z_mm")
    };
    for (const QString& name : required) {
        if (!column.contains(name)) {
            error = tr("这不是V27.6.15独立验证CSV：缺少列 %1。训练CSV不能作为验证CSV导入。").arg(name);
            return false;
        }
    }

    QMap<qint64,int> profilePoseIds;
    int lineNumber = 1;
    int dataRows = 0;
    while (!metadata.atEnd()) {
        const QString line = metadata.readLine();
        ++lineNumber;
        if (line.trimmed().isEmpty() || line.trimmed().startsWith('#')) continue;
        const QStringList fields = line.split(',', Qt::KeepEmptyParts);
        auto field = [&](const QString& name) -> QString {
            const int index = column.value(name, -1);
            return (index >= 0 && index < fields.size()) ? fields[index].trimmed() : QString();
        };
        if (field(QStringLiteral("dataset_role")) != QStringLiteral("holdout_validation")) {
            error = tr("验证CSV第%1行dataset_role不是holdout_validation；拒绝把训练数据冒充独立验证数据。").arg(lineNumber);
            return false;
        }
        bool versionOk = false;
        const int version = field(QStringLiteral("format_version")).toInt(&versionOk);
        if (!versionOk || version != 1) {
            error = tr("验证CSV第%1行format_version无效或不受支持。").arg(lineNumber);
            return false;
        }
        if (field(QStringLiteral("model_fingerprint")) != expectedModel) {
            error = tr("验证CSV第%1行绑定的Physical17模型与当前17参数结果不一致。").arg(lineNumber);
            return false;
        }
        if (field(QStringLiteral("training_pose_fingerprint")) != expectedTraining) {
            error = tr("验证CSV第%1行记录的阶段2训练姿态集合与当前导入训练CSV不一致；无法证明hold-out独立性。").arg(lineNumber);
            return false;
        }
        bool poseOk = false, profileOk = false;
        const int poseId = field(QStringLiteral("validation_pose_id")).toInt(&poseOk);
        const qint64 profileId = field(QStringLiteral("profile_id")).toLongLong(&profileOk);
        if (!poseOk || poseId < 0 || !profileOk) {
            error = tr("验证CSV第%1行pose/profile标识无效。").arg(lineNumber);
            return false;
        }
        if (profilePoseIds.contains(profileId) && profilePoseIds.value(profileId) != poseId) {
            error = tr("验证CSV中profile_id=%1被分配到多个验证姿态。").arg(profileId);
            return false;
        }
        profilePoseIds.insert(profileId, poseId);
        ++dataRows;
    }
    if (dataRows <= 0) {
        error = tr("验证CSV没有数据行。");
        return false;
    }

    std::ifstream input(fileName.toStdString());
    std::vector<SphereProfileObservation> observations;
    std::string csvError;
    if (!SingleSphere17Calibrator::readObservationCsv(input, observations, csvError)) {
        error = tr("验证CSV球面观测解析失败：%1").arg(QString::fromStdString(csvError));
        return false;
    }

    QMap<int,CaptureRecord> grouped;
    auto fitOptions = ransacOptionsFromUi();
    fitOptions.maxIterations = std::max(350, fitOptions.maxIterations);
    fitOptions.minInliers = 5;
    fitOptions.minInlierRatio = 0.10;
    fitOptions.minArcSpanDeg = 10.0;
    for (const auto& observation : observations) {
        const auto poseIt = profilePoseIds.constFind(observation.profileId);
        if (poseIt == profilePoseIds.constEnd()) {
            error = tr("验证CSV内部profile_id映射不完整。");
            return false;
        }
        const int poseId = poseIt.value();
        if (!grouped.contains(poseId)) {
            CaptureRecord record;
            record.planIndex = -1;
            record.stage1Bootstrap = false;
            record.referenceCapture = false;
            record.usedPose = controllerPoseFromPortable(observation.joints);
            record.timeText = tr("离线导入：%1").arg(QFileInfo(fileName).fileName());
            grouped.insert(poseId, record);
        } else {
            const auto& q = grouped[poseId].usedPose;
            if (std::abs(q.x-observation.joints.x)>0.03 || std::abs(q.y-observation.joints.y)>0.03 ||
                std::abs(q.z-observation.joints.z)>0.03 || std::abs(q.c-observation.joints.c)>0.03 ||
                std::abs(q.b-observation.joints.b)>0.03) {
                error = tr("验证姿态ID=%1包含不一致的XYZCB位置。").arg(poseId);
                return false;
            }
        }

        auto& record = grouped[poseId];
        record.observations.push_back(observation);
        std::vector<std::size_t> inliers;
        const auto fit = SingleSphere17Calibrator::fitSphereCenterInSensor(
            observation.points, m_spinSphereDiameter->value(), +1, fitOptions, &inliers);
        if (fit.success && (!record.representativeFit.success || fit.circleRmsMm < record.representativeFit.circleRmsMm))
            record.representativeFit = fit;
    }

    records.reserve(static_cast<std::size_t>(grouped.size()));
    for (auto it = grouped.cbegin(); it != grouped.cend(); ++it) {
        const CaptureRecord& record = it.value();
        if (record.observations.empty() || !record.representativeFit.success) {
            error = tr("验证姿态ID=%1无法从保存的球面点重建代表球心拟合。").arg(it.key());
            records.clear();
            return false;
        }
        QString separationError;
        if (!validationPoseSeparated(record.usedPose, separationError)) {
            error = tr("导入的hold-out数据不满足当前训练集独立性：%1").arg(separationError);
            records.clear();
            return false;
        }
        records.push_back(record);
    }
    if (records.empty()) {
        error = tr("验证CSV没有可用的独立姿态。");
        return false;
    }
    error.clear();
    return true;
}

bool SingleSphere14CalibrationWidget::persistValidationObservationCsv(
    QStringList& savedPaths, QString& error) const
{
    savedPaths.clear();
    const QString fingerprint = physical17Fingerprint(m_lastResult);
    if (fingerprint.isEmpty()) {
        error = tr("没有可用于命名持久化验证CSV的Physical17指纹。");
        return false;
    }
    QString base = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);
    if (base.isEmpty()) base = QDir::home().filePath(QStringLiteral(".MotionLaserSync"));
    const QString leaf = QStringLiteral("single_sphere17_holdout_%1.csv").arg(fingerprint.left(12));
    QStringList targets;
    targets << QDir(base).filePath(QStringLiteral("calibration/") + leaf)
            << QDir(QCoreApplication::applicationDirPath()).filePath(QStringLiteral("calibration/") + leaf);
    targets.removeDuplicates();

    QStringList failures;
    for (const QString& path : targets) {
        QString writeError;
        if (writeValidationObservationCsv(path, writeError)) savedPaths << path;
        else failures << tr("%1（%2）").arg(path, writeError);
    }
    if (savedPaths.isEmpty()) {
        error = tr("独立验证CSV未能写入任何持久位置：%1").arg(failures.join(QStringLiteral("；")));
        return false;
    }
    error.clear();
    return true;
}

void SingleSphere14CalibrationWidget::onImportValidationCsv()
{
    if (m_stage2SolveRunning) {
        QMessageBox::information(this, tr("阶段2正在求解"), tr("请等待当前求解完成或取消后再导入独立验证CSV。"));
        return;
    }
    if (!m_hasResult || !m_lastResult.success) {
        QMessageBox::warning(this, tr("不能导入验证集"), tr("请先加载或求得冻结后的Physical17结果。"));
        return;
    }
    if (stage2TrainingObservations().empty()) {
        QMessageBox::warning(this, tr("缺少训练数据来源"),
            tr("离线验证需要重新检查hold-out与训练网格的B/C独立性。请先导入与当前Physical17结果对应的阶段2训练CSV，再导入独立验证CSV。"));
        return;
    }
    const QString fn = QFileDialog::getOpenFileName(
        this, tr("导入独立验证CSV（只接受holdout_validation格式）"), QString(), tr("CSV (*.csv)"));
    if (fn.isEmpty()) return;

    std::vector<CaptureRecord> imported;
    QString error;
    if (!loadValidationObservationCsv(fn, imported, error)) {
        QMessageBox::warning(this, tr("导入独立验证CSV失败"), error);
        return;
    }
    if ((!m_validationRecords.empty() || m_validationSummary.computed) &&
        QMessageBox::question(this, tr("替换当前独立验证集"),
            tr("当前已经存在hold-out验证数据或结论。导入新的验证CSV会替换当前验证集，并使旧验证结论失效。\n\n继续？"),
            QMessageBox::Yes|QMessageBox::No, QMessageBox::No) != QMessageBox::Yes) return;

    if (!m_validationRecords.empty() || m_validationSummary.computed || m_persistentCalibrationLoaded)
        invalidateValidation(tr("已导入新的独立验证CSV，旧验证结论已失效。"), true);
    else {
        m_validationRecords.clear();
        m_validationSummary = ValidationSummary{};
    }
    m_validationRecords = std::move(imported);
    if (m_planValidation) {
        m_plan.clear();
        m_planValidation = false;
        if (m_planTable) m_planTable->setRowCount(0);
    }

    std::size_t profiles = 0, points = 0;
    for (const auto& record : m_validationRecords) {
        profiles += record.observations.size();
        for (const auto& observation : record.observations) points += observation.points.size();
    }
    appendLog(tr("已离线导入独立验证CSV：%1；hold-out姿态=%2，profiles=%3，points=%4。"
                 "模型指纹和训练姿态指纹均匹配，且每个验证姿态已重新检查与训练B/C网格的最小间隔。"
                 "达到设置数量后可直接点击“执行独立验证”，无需连接控制器或2950。")
        .arg(fn).arg(m_validationRecords.size())
        .arg(static_cast<qulonglong>(profiles)).arg(static_cast<qulonglong>(points)));
    updateValidationSummary();
    updateSessionSummary();
    updateCalibrationReadiness();
    updateWorkflowGuide();
}

void SingleSphere14CalibrationWidget::onExportValidationCsv()
{
    if (m_validationRecords.empty()) {
        QMessageBox::information(this, tr("无验证数据"), tr("当前没有可导出的独立hold-out验证观测。"));
        return;
    }
    const QString defaultName = QStringLiteral("single_sphere17_holdout_validation.csv");
    const QString fn = QFileDialog::getSaveFileName(this, tr("导出独立验证CSV"), defaultName, tr("CSV (*.csv)"));
    if (fn.isEmpty()) return;
    QString error;
    if (!writeValidationObservationCsv(fn, error)) {
        QMessageBox::warning(this, tr("导出失败"), error);
        return;
    }
    appendLog(tr("已导出独立验证CSV：%1。该文件带holdout角色、Physical17模型指纹和训练姿态指纹；"
                 "以后可在相同17参数结果+相同训练CSV下离线重算验证，普通训练CSV不会被此入口接受。")
        .arg(fn));
}

bool SingleSphere14CalibrationWidget::validationPoseOutsideRejectedTrainingHazards(
    const JointPose5D& pose, double minBcDistanceDeg, QString* reason) const
{
    if (!(minBcDistanceDeg > 0.0) || m_lastResult.rejectedTrainingPoses.empty()) return true;

    auto angularDiffDeg = [](double a, double b) {
        return std::remainder(a - b, 360.0);
    };

    double nearest = std::numeric_limits<double>::infinity();
    const Physical17PoseResidualSummary* nearestRejected = nullptr;
    for (const auto& rejected : m_lastResult.rejectedTrainingPoses) {
        const double db = pose.b - rejected.pose.b;
        const double dc = angularDiffDeg(pose.c, rejected.pose.c);
        const double distance = std::hypot(db, dc);
        if (distance < nearest) {
            nearest = distance;
            nearestRejected = &rejected;
        }
    }

    if (nearest >= minBcDistanceDeg || !nearestRejected) return true;
    if (reason) {
        *reason = tr(
            "当前验证姿态 B=%1° C=%2° 距离阶段2已剔除的高误差姿态 B=%3° C=%4° 只有 %5°，"
            "小于危险区排除半径 %6°（该坏姿态训练RMS=%7 mm，P95=%8 mm）。")
            .arg(pose.b,0,'f',2).arg(pose.c,0,'f',2)
            .arg(nearestRejected->pose.b,0,'f',2).arg(nearestRejected->pose.c,0,'f',2)
            .arg(nearest,0,'f',2).arg(minBcDistanceDeg,0,'f',2)
            .arg(nearestRejected->residualRmsMm,0,'g',6)
            .arg(nearestRejected->residualP95AbsMm,0,'g',6);
    }
    return false;
}

bool SingleSphere14CalibrationWidget::validationPoseSeparated(const JointPose5D& pose, QString& error) const
{
    if (m_checkValidationAvoidRejected && m_checkValidationAvoidRejected->isChecked()
        && !m_lastResult.rejectedTrainingPoses.empty()) {
        const double hazardSep = m_spinValidationHazardBCSep ? m_spinValidationHazardBCSep->value() : 10.0;
        QString hazardError;
        if (!validationPoseOutsideRejectedTrainingHazards(pose, hazardSep, &hazardError)) {
            error = hazardError + tr("\n该姿态属于已知高残差B/C邻域，不用于V27.6.42安全工作域的独立验证。请重新生成验证计划。");
            return false;
        }
    }

    const double minSep = m_spinValidationBCSep ? m_spinValidationBCSep->value() : 4.0;
    double nearest = 1.0e12;
    bool haveTraining = false;

    if (!m_records.empty()) {
        for (const auto& training : m_records) {
            if (training.stage1Bootstrap) continue;
            const double db = pose.b - training.usedPose.b;
            const double dc = pose.c - training.usedPose.c;
            nearest = std::min(nearest, std::hypot(db, dc));
            haveTraining = true;
        }
    } else {
        // Offline restore path: imported Stage-2 CSV lives in m_calibrator rather than
        // CaptureRecord objects.  Use those B/C poses too; otherwise a loaded model
        // could accidentally validate on points sitting directly on its training grid.
        const auto training = stage2TrainingObservations();
        for (const auto& observation : training) {
            const double db = pose.b - observation.joints.b;
            const double dc = pose.c - observation.joints.c;
            nearest = std::min(nearest, std::hypot(db, dc));
            haveTraining = true;
        }
    }

    if (!haveTraining) return true;
    if (nearest < minSep) {
        error = tr("当前姿态与训练集中最近B/C姿态只相差 %1°，小于独立验证最小间隔 %2°。\n"
                   "请使用“生成独立验证姿态”得到错开训练网格的hold-out姿态，不能拿训练附近的重复姿态冒充独立验证。")
                    .arg(nearest,0,'f',2).arg(minSep,0,'f',2);
        return false;
    }
    return true;
}

void SingleSphere14CalibrationWidget::startCapture(bool validationOnly)
{
    if (m_collecting) return;
    if (!m_referenceTaught) {
        QMessageBox::warning(this,tr("不能采集"),tr("请先点击“开始阶段1：锁定坐标约定”，固定Flip X/Flip Z和DPOS/MPOS数据源。"));
        return;
    }
    if (validationOnly && (!m_hasResult || !m_lastResult.success)) {
        QMessageBox::warning(this,tr("不能采集验证集"),tr("请先完成阶段2最终17参数求解。验证集必须在模型冻结后另外采集。"));
        return;
    }
    if (!validationOnly && m_planValidation) {
        QMessageBox::warning(this,tr("当前是验证计划"),
            tr("当前计划表显示的是hold-out验证姿态。请使用“采集当前姿态为验证集”，不要把验证姿态加入训练求解器。\n"
               "若要继续补充训练数据，请重新生成阶段2 D-optimal 标定计划。"));
        return;
    }
    if (validationOnly && !m_planValidation) {
        QMessageBox::warning(this,tr("当前不是验证计划"),
            tr("请先点击“生成独立验证姿态”，再移动到其中一个验证姿态并采集。"));
        return;
    }

    QString conventionError;
    if (!validateSessionConventions(conventionError)) {
        if (m_autoValidationAcquisition && validationOnly)
            stopAutomaticValidationAcquisition(tr("独立验证自动采集停止：坐标约定已改变：%1").arg(conventionError));
        else
            QMessageBox::warning(this,tr("坐标约定已改变"),conventionError);
        return;
    }
    if (!m_sensor || !m_sensor->isProfileRunning()) {
        if (m_autoValidationAcquisition && validationOnly)
            stopAutomaticValidationAcquisition(tr("独立验证自动采集停止：轮廓预览未运行。"));
        else
            QMessageBox::warning(this,tr("不能采集"),tr("请先启动单球标定轮廓预览。静态标定固定使用软件内部触发 + 全测量场预设连续 Profile。"));
        return;
    }

    QString error;
    if (!readStableState(m_captureBefore,error)) {
        if (m_autoValidationAcquisition && validationOnly)
            stopAutomaticValidationAcquisition(tr("独立验证自动采集停止：无法确认静止五轴状态：%1").arg(error));
        else
            QMessageBox::warning(this,tr("不能采集"),error);
        return;
    }
    updateCurrentPoseLabel(&m_captureBefore);
    const JointPose5D current = calibrationPoseFromState(m_captureBefore, m_referenceFeedbackMask);

    if (validationOnly && !validationPoseSeparated(current, error)) {
        if (m_autoValidationAcquisition) {
            const int failedIndex = selectedPlanIndex();
            if (failedIndex >= 0) markPlanStatus(failedIndex, tr("已跳过：危险/非独立验证姿态"), error);
            appendLog(tr("自动独立验证跳过姿态%1：%2").arg(failedIndex+1).arg(error));
            if (failedIndex >= 0) selectNextPendingPlanRow(failedIndex);
            QTimer::singleShot(50, this, &SingleSphere14CalibrationWidget::continueAutomaticValidationAcquisition);
        } else {
            QMessageBox::warning(this, tr("验证姿态不独立"), error);
        }
        return;
    }

    // Every accepted pose must have a fresh, quality-checked sphere fit at the
    // current stationary pose. This prevents accidentally recording a plan row
    // after the sphere has drifted out of the intended sensor field.
    onFitCurrentProfile();
    if (!m_hasLatestFit || m_latestFit.circleRmsMm > m_spinMaxCircleRms->value()) {
        if (m_autoStage2Acquisition && !validationOnly) {
            const int failedIndex = selectedPlanIndex();
            if (failedIndex >= 0) markPlanStatus(failedIndex, tr("已跳过：正式采集前球拟合失败"));
            appendLog(tr("自动阶段2：正式采集前重复球拟合未通过，当前姿态丢弃并继续。"));
            if (failedIndex >= 0) selectNextPendingPlanRow(failedIndex);
            QTimer::singleShot(50, this, &SingleSphere14CalibrationWidget::continueAutomaticStage2Acquisition);
        } else if (m_autoValidationAcquisition && validationOnly) {
            const int failedIndex = selectedPlanIndex();
            if (failedIndex >= 0) markPlanStatus(failedIndex, tr("已跳过：验证采集前球拟合失败"));
            appendLog(tr("自动独立验证：正式采集前重复球拟合未通过，当前验证姿态丢弃并继续。"));
            if (failedIndex >= 0) selectNextPendingPlanRow(failedIndex);
            QTimer::singleShot(50, this, &SingleSphere14CalibrationWidget::continueAutomaticValidationAcquisition);
        } else {
            QMessageBox::warning(this,tr("不能采集"),tr("当前姿态没有质量合格的球拟合。请先找回球并重新拟合。"));
        }
        return;
    }

    m_capturePlanIndex = selectedPlanIndex();
    if (m_capturePlanIndex >= 0) {
        if (!selectedPoseBCReached(current, m_capturePlanIndex)) {
            const QString reachError = tr("选中计划姿态的B/C尚未到位。请先完成独立轴定位并确认到位，再进行静态采集。");
            if (m_autoValidationAcquisition && validationOnly)
                stopAutomaticValidationAcquisition(tr("独立验证自动采集停止：%1").arg(reachError));
            else
                QMessageBox::warning(this,tr("B/C尚未到目标"), reachError);
            return;
        }
        const Vec3d& targetCenter = m_plan[static_cast<std::size_t>(m_capturePlanIndex)].targetSphereCenterSensorMm;
        const Vec3d measuredCenter = m_latestFit.centerSensorMm;
        const double xErr = std::abs(measuredCenter.x - targetCenter.x);
        const double zErr = std::abs(measuredCenter.z - targetCenter.z);
        // A single XZ laser profile determines only |Y_S| from the section radius.
        // Its sign is unobservable in one frame and must never be used as a hard
        // acquisition gate.  V27.6.37 used the fit helper's +|Y_S| branch and
        // compared it directly with the signed D-optimal target; for a negative-Y
        // target an otherwise excellent circular profile was rejected with
        // |dY| ~= 2*|Y_S| (often 10-15 mm).  Gate on the observable magnitude instead.
        const double yMagnitudeErr = std::abs(std::abs(measuredCenter.y) - std::abs(targetCenter.y));
        const double fieldTol = 1.5;
        if (xErr > fieldTol || zErr > fieldTol || yMagnitudeErr > 2.5) {
            const QString centerError = tr("当前与计划球心差异：|dX|=%1 mm，|dZ|=%2 mm，|d|Y_S||=%3 mm（单帧轮廓不观测Y符号）。")
                .arg(xErr,0,'f',3).arg(zErr,0,'f',3).arg(yMagnitudeErr,0,'f',3);
            if (m_autoStage2Acquisition && !validationOnly) {
                if (m_capturePlanIndex>=0) markPlanStatus(m_capturePlanIndex,tr("已跳过：球心未进入计划窗口"),centerError);
                appendLog(tr("自动阶段2丢弃姿态%1：%2").arg(m_capturePlanIndex+1).arg(centerError));
                if (m_capturePlanIndex>=0) selectNextPendingPlanRow(m_capturePlanIndex);
                QTimer::singleShot(50, this, &SingleSphere14CalibrationWidget::continueAutomaticStage2Acquisition);
            } else if (m_autoValidationAcquisition && validationOnly) {
                if (m_capturePlanIndex>=0) markPlanStatus(m_capturePlanIndex,tr("已跳过：验证球心未进入计划窗口"),centerError);
                appendLog(tr("自动独立验证丢弃姿态%1：%2").arg(m_capturePlanIndex+1).arg(centerError));
                if (m_capturePlanIndex>=0) selectNextPendingPlanRow(m_capturePlanIndex);
                QTimer::singleShot(50, this, &SingleSphere14CalibrationWidget::continueAutomaticValidationAcquisition);
            } else {
                QMessageBox::warning(this,tr("球心尚未接近计划位置"),
                    centerError + tr("\n请继续使用XYZ小步找球后再采集，以保证训练/验证姿态都真正位于计划视场位置。"));
            }
            return;
        }
    }

    m_captureValidation = validationOnly;
    m_captureFrames.clear();
    m_capturePendingStart = true;
    if (m_stage1CaptureProgress) {
        m_stage1CaptureProgress->setRange(0, m_spinCaptureFrames->value());
        m_stage1CaptureProgress->setValue(0);
        m_stage1CaptureProgress->setFormat(tr("等待停稳 %1 ms...").arg(m_spinSettleMs->value()));
    }
    updateStage1Status();
    m_lblSolveState->setText(validationOnly
        ? tr("验证集：等待停稳 %1 ms，随后只接收新的轮廓...").arg(m_spinSettleMs->value())
        : tr("等待停稳 %1 ms，随后只接收新的轮廓...").arg(m_spinSettleMs->value()));
    QTimer::singleShot(m_spinSettleMs->value(), this, &SingleSphere14CalibrationWidget::beginFreshProfileCollection);
}

void SingleSphere14CalibrationWidget::beginFreshProfileCollection()
{
    if (m_collecting) return;
    m_capturePendingStart = false;
    QString error;
    if (!validateSessionConventions(error)) {
        m_captureValidation = false;
        updateStage1Status();
        if (m_autoStage2Acquisition) stopAutomaticStage2Acquisition(tr("阶段2自动采集停止：坐标约定检查失败：%1").arg(error));
        else if (m_autoValidationAcquisition) stopAutomaticValidationAcquisition(tr("独立验证自动采集停止：坐标约定检查失败：%1").arg(error));
        else QMessageBox::warning(this,tr("采集取消"),error);
        return;
    }
    JointState5D state;
    if (!readStableState(state,error)) {
        m_captureValidation = false;
        updateStage1Status();
        if (m_autoStage2Acquisition) stopAutomaticStage2Acquisition(tr("阶段2自动采集停止：无法确认静止五轴状态：%1").arg(error));
        else if (m_autoValidationAcquisition) stopAutomaticValidationAcquisition(tr("独立验证自动采集停止：无法确认静止五轴状态：%1").arg(error));
        else QMessageBox::warning(this,tr("采集取消"),error);
        return;
    }
    m_captureBefore = state;
    m_captureFrames.clear();
    if (m_profileCaptureConnection) {
        disconnect(m_profileCaptureConnection);
        m_profileCaptureConnection = {};
    }
    if (!m_sensor) {
        m_captureValidation = false;
        if (m_autoStage2Acquisition) stopAutomaticStage2Acquisition(tr("阶段2自动采集停止：传感器对象不可用。"));
        else if (m_autoValidationAcquisition) stopAutomaticValidationAcquisition(tr("独立验证自动采集停止：传感器对象不可用。"));
        else QMessageBox::warning(this, tr("采集取消"), tr("传感器对象不可用"));
        return;
    }
    m_profileCaptureConnection = connect(m_sensor, &LaserSensor::profileReceived,
                                         this, &SingleSphere14CalibrationWidget::onSensorProfile,
                                         Qt::QueuedConnection);
    m_collecting = true;
    if (m_stage1CaptureProgress) {
        m_stage1CaptureProgress->setRange(0, m_spinCaptureFrames->value());
        m_stage1CaptureProgress->setValue(0);
        m_stage1CaptureProgress->setFormat(tr("fresh profiles 0/%1").arg(m_spinCaptureFrames->value()));
    }
    m_lblSolveState->setText(tr("正在采集 fresh profiles 0/%1...").arg(m_spinCaptureFrames->value()));
    updateStage1Status();
}

void SingleSphere14CalibrationWidget::finishFreshProfileCollection()
{
    if (m_profileCaptureConnection) {
        disconnect(m_profileCaptureConnection);
        m_profileCaptureConnection = {};
    }
    const bool validationCapture = m_captureValidation;
    m_captureValidation = false;
    JointState5D after;
    QString error;
    if (!readStableState(after,error)) {
        if (m_autoStage2Acquisition) stopAutomaticStage2Acquisition(tr("阶段2自动采集停止：采集结束时五轴状态无效：%1").arg(error));
        else if (m_autoValidationAcquisition && validationCapture) stopAutomaticValidationAcquisition(tr("独立验证自动采集停止：采集结束时五轴状态无效：%1").arg(error));
        else QMessageBox::warning(this,tr("采集无效"),error);
        return;
    }
    if (!checkCaptureDrift(m_captureBefore,after,error)) {
        if (m_capturePlanIndex >= 0) markPlanStatus(m_capturePlanIndex, tr("已跳过：采集期间发生位姿漂移"), error);
        if (m_autoStage2Acquisition) stopAutomaticStage2Acquisition(tr("阶段2自动采集停止：检测到采集期间位姿漂移：%1").arg(error));
        else if (m_autoValidationAcquisition && validationCapture) stopAutomaticValidationAcquisition(tr("独立验证自动采集停止：检测到采集期间位姿漂移：%1").arg(error));
        else QMessageBox::warning(this,tr("采集无效"),error);
        return;
    }

    CaptureRecord record;
    record.planIndex=m_capturePlanIndex;
    record.stage1Bootstrap = !validationCapture && !m_hasBootstrap && (m_capturePlanIndex < 0);
    record.referenceCapture = false;
    record.before=m_captureBefore; record.after=after;
    record.usedPose=averagedCalibrationPose(m_captureBefore,after,m_referenceFeedbackMask); record.rawProfiles=m_captureFrames;
    JointPose5D averagedDpos;
    averagedDpos.x = 0.5 * (m_captureBefore.dpos.x + after.dpos.x);
    averagedDpos.y = 0.5 * (m_captureBefore.dpos.y + after.dpos.y);
    averagedDpos.z = 0.5 * (m_captureBefore.dpos.z + after.dpos.z);
    averagedDpos.c = 0.5 * (m_captureBefore.dpos.c + after.dpos.c);
    averagedDpos.b = 0.5 * (m_captureBefore.dpos.b + after.dpos.b);
    record.timeText=QDateTime::currentDateTime().toString(Qt::ISODate);

    double cx=0,cy=0,cz=0,rms=0; int validFits=0;
    double p95Sum=0.0;
    std::size_t keptPointCount=0;
    int rejectedFit=0, rejectedRatio=0, rejectedRms=0, rejectedP95=0, rejectedTrim=0;
    SphereCenterFit bestFit;
    for (const auto& profile : m_captureFrames) {
        const auto points=filteredPoints(profile,nullptr);
        std::vector<std::size_t> inlierIndices;
        const auto fit=SingleSphere17Calibrator::fitSphereCenterInSensor(
            points, m_spinSphereDiameter->value(), +1,
            ransacOptionsFromUi(), &inlierIndices);
        if (!fit.success || inlierIndices.size() < 8) { ++rejectedFit; continue; }
        if (fit.inlierRatio < m_spinStage2MinInlierRatio->value()) { ++rejectedRatio; continue; }

        // RANSAC removes gross background/outliers first.  The remaining arc is
        // then trimmed at both angular ends before any point is allowed into the
        // Physical17 solve, because the sensor-quality study showed that arc
        // endpoints are where Maximum/Width collapse and geometric residuals rise.
        const auto trimmed = trimArcEdgeInliers(points, fit, inlierIndices,
                                                m_spinStage2EdgeTrimFraction->value());
        if (trimmed.indices.size() < 8) { ++rejectedTrim; continue; }
        if (trimmed.residualRmsMm > m_spinStage2MaxRms->value()) { ++rejectedRms; continue; }
        if (trimmed.residualP95Mm > m_spinStage2MaxP95->value()) { ++rejectedP95; continue; }

        SphereProfileObservation obs;
        obs.profileId=static_cast<std::int64_t>(profile.index);
        obs.joints=SingleSphere17QtAdapter::toPortable(record.usedPose);
        obs.dposJoints=SingleSphere17QtAdapter::toPortable(averagedDpos);
        obs.hasDposJoints=true;
        obs.points.reserve(trimmed.indices.size());
        for (std::size_t index : trimmed.indices) {
            if (index < points.size()) obs.points.push_back(points[index]);
        }
        keptPointCount += obs.points.size();
        record.observations.push_back(std::move(obs));
        cx+=fit.centerSensorMm.x; cy+=fit.centerSensorMm.y; cz+=fit.centerSensorMm.z;
        rms+=trimmed.residualRmsMm; p95Sum+=trimmed.residualP95Mm; ++validFits;
        if (!bestFit.success || trimmed.residualRmsMm < bestFit.circleRmsMm) {
            bestFit=fit;
            bestFit.circleRmsMm=trimmed.residualRmsMm;
            bestFit.inlierPointCount=trimmed.indices.size();
        }
    }
    const int requiredValidFits = std::max(1, static_cast<int>(std::ceil(
        static_cast<double>(m_spinCaptureFrames->value()) * m_spinStage2MinValidFrameRatio->value())));
    if (validFits < requiredValidFits) {
        const QString qualityError = tr(
            "%1 条 fresh profiles 中只有 %2 条通过阶段2严格质量门禁（至少需要%3）。"
            "拒绝原因：RANSAC/点数=%4，内点比例=%5，裁边后点数=%6，RMS=%7，P95=%8。"
            "请调整D-γ-η、ROI/曝光/球位置后重采。")
            .arg(m_captureFrames.size()).arg(validFits).arg(requiredValidFits)
            .arg(rejectedFit).arg(rejectedRatio).arg(rejectedTrim).arg(rejectedRms).arg(rejectedP95);
        if (m_capturePlanIndex>=0) markPlanStatus(m_capturePlanIndex,tr("已跳过：采集质量不足"),qualityError);
        appendLog(qualityError);
        if (m_autoStage2Acquisition && !validationCapture) {
            if (m_capturePlanIndex>=0) selectNextPendingPlanRow(m_capturePlanIndex);
            QTimer::singleShot(50, this, &SingleSphere14CalibrationWidget::continueAutomaticStage2Acquisition);
        } else if (m_autoValidationAcquisition && validationCapture) {
            if (m_capturePlanIndex>=0) selectNextPendingPlanRow(m_capturePlanIndex);
            QTimer::singleShot(50, this, &SingleSphere14CalibrationWidget::continueAutomaticValidationAcquisition);
        } else {
            QMessageBox::warning(this,tr("采集质量不足"),qualityError);
        }
        return;
    }
    record.representativeFit=bestFit;

    if (validationCapture) {
        m_validationRecords.push_back(std::move(record));
        m_validationSummary = ValidationSummary{};
        if (m_capturePlanIndex>=0) {
            markPlanStatus(m_capturePlanIndex,tr("验证已采集"),
                tr("%1帧通过严格门禁；best kept=%2/%3；arc=%4°；RMS=%5 mm；mean P95=%6 mm")
                    .arg(validFits).arg(bestFit.inlierPointCount).arg(bestFit.candidatePointCount)
                    .arg(bestFit.arcSpanDeg,0,'f',1).arg(bestFit.circleRmsMm,0,'g',5).arg(p95Sum/validFits,0,'g',5));
            selectNextPendingPlanRow(m_capturePlanIndex);
        }
        appendLog(tr("接受独立验证采样：%1；有效轮廓=%2；截面圆中心XZ≈[%3,%5] mm；|Y_S|≈%4 mm；平均内点RMS=%6 mm。"
                     "该数据只进入hold-out验证集，不会加入17参数求解器。")
            .arg(poseText(m_validationRecords.back().usedPose)).arg(validFits)
            .arg(cx/validFits,0,'f',4).arg(cy/validFits,0,'f',4).arg(cz/validFits,0,'f',4)
            .arg(rms/validFits,0,'g',5));
        QStringList savedPaths;
        QString autosaveError;
        if (persistValidationObservationCsv(savedPaths, autosaveError)) {
            appendLog(tr("独立验证原始观测已自动持久化：%1。后续即使验证报告JSON丢失，也可导入该hold-out CSV离线重新计算。")
                .arg(savedPaths.join(QStringLiteral("；"))));
        } else {
            appendLog(tr("警告：本次hold-out采集已保留在内存，但自动持久化验证CSV失败：%1。请立即使用“导出独立验证 CSV”另存。")
                .arg(autosaveError));
        }
        updateValidationSummary();
        updateSessionSummary();
        updateCalibrationReadiness();
        updateWorkflowGuide();
        if (m_autoValidationAcquisition && !m_autoValidationStopRequested) {
            QTimer::singleShot(50, this, &SingleSphere14CalibrationWidget::continueAutomaticValidationAcquisition);
        }
        return;
    }

    // Any change to the training set invalidates previous hold-out evidence.
    // Require a new independent validation after the final training data are frozen.
    if (!m_validationRecords.empty() || m_validationSummary.computed) {
        invalidateValidation(tr("训练数据集已改变，旧独立验证结果作废；最终求解后需要重新采集hold-out验证姿态。"), true);
    }

    m_records.push_back(std::move(record));
    // Stage-1 observations are used only by the nominal hand-eye/sphere bootstrap.
    // The final Physical17 training set must contain Stage-2 observations only.
    if (!m_records.back().stage1Bootstrap) {
        for (const auto& obs : m_records.back().observations) m_calibrator.addObservation(obs);
    }
    m_latestFit=bestFit; m_hasLatestFit=true;
    if (!m_records.back().stage1Bootstrap && m_hasResult) {
        // 新数据加入后，旧17参数结果仍可作为当前规划初值，但不再代表当前数据集的最终解。
        m_hasResult = false;
        m_refinementSolved = false;
        m_lblSolveState->setText(tr("已加入新的阶段2数据；之前17参数结果保留为规划初值，请在数据充分后重新最终求解。"));
    }
    if (m_records.back().stage1Bootstrap) {
        appendLog(tr("阶段1人工姿态已采集：当前阶段1独立姿态数=%1。继续改变B/C与XYZ，或在数据充分后点击阶段1求解。")
                  .arg(std::count_if(m_records.begin(), m_records.end(), [](const CaptureRecord& r){ return r.stage1Bootstrap; })));
        if (m_activeStage1Candidate >= 0 && m_activeStage1Candidate < static_cast<int>(m_stage1Candidates.size())) {
            const int capturedCandidate = m_activeStage1Candidate;
            setStage1CandidateStatus(capturedCandidate, tr("已采集"));
            m_activeStage1Candidate = -1;
            selectNextStage1Candidate(capturedCandidate);
        }
    }
    if (m_capturePlanIndex>=0) {
        markPlanStatus(m_capturePlanIndex,tr("已采集"),
            tr("%1帧通过严格门禁；best kept=%2/%3；arc=%4°；RMS=%5 mm；mean P95=%6 mm")
                .arg(validFits).arg(bestFit.inlierPointCount).arg(bestFit.candidatePointCount)
                .arg(bestFit.arcSpanDeg,0,'f',1).arg(bestFit.circleRmsMm,0,'g',5).arg(p95Sum/validFits,0,'g',5));
        selectNextPendingPlanRow(m_capturePlanIndex);
    }
    appendLog(tr("接受静态采样：%1；有效轮廓=%2/%3；进入求解器的裁边RANSAC内点总数=%4；截面圆中心XZ≈[%5,%7] mm；|Y_S|≈%6 mm；平均RMS=%8 mm；平均P95=%9 mm；best kept=%10/%11；arc=%12°")
        .arg(poseText(m_records.back().usedPose)).arg(validFits).arg(m_captureFrames.size())
        .arg(static_cast<qulonglong>(keptPointCount))
        .arg(cx/validFits,0,'f',4).arg(cy/validFits,0,'f',4).arg(cz/validFits,0,'f',4)
        .arg(rms/validFits,0,'g',5).arg(p95Sum/validFits,0,'g',5)
        .arg(bestFit.inlierPointCount).arg(bestFit.candidatePointCount).arg(bestFit.arcSpanDeg,0,'f',1));
    updateSessionSummary();
    updateCalibrationReadiness();
    updateWorkflowGuide();
    updateStage1Status();
    if (m_autoStage2Acquisition && !m_autoStage2StopRequested) {
        QTimer::singleShot(50, this, &SingleSphere14CalibrationWidget::continueAutomaticStage2Acquisition);
    }
}

void SingleSphere14CalibrationWidget::rebuildCalibratorFromRecords()
{
    m_calibrator = SingleSphere17Calibrator(solverOptionsFromUi());
    for (const auto& r : m_records) {
        if (r.stage1Bootstrap) continue;
        for (const auto& o : r.observations) m_calibrator.addObservation(o);
    }
}

void SingleSphere14CalibrationWidget::invalidateValidation(const QString& reason, bool clearRecords)
{
    // Validation belongs to the model currently being trained. Do not revoke the
    // previous accepted production calibration just because a new MPOS calibration
    // changes its training/hold-out set. The accepted bundle is transactional fallback:
    // it remains usable until a newly validated model is successfully persisted over it.
    if (clearRecords) m_validationRecords.clear();
    m_validationSummary = ValidationSummary{};

    if (m_lblValidation) {
        m_lblValidation->setText(reason.isEmpty()
            ? tr("独立验证：尚未执行。")
            : tr("独立验证无效：%1").arg(reason));
        m_lblValidation->setStyleSheet("QLabel { color:#7a5200; font-weight:bold; }");
    }
    if (!reason.isEmpty()) appendLog(reason);
}

void SingleSphere14CalibrationWidget::updateValidationSummary()
{
    if (!m_lblValidation) return;
    if (m_validationSummary.computed) {
        m_lblValidation->setText(m_validationSummary.message);
        m_lblValidation->setStyleSheet(m_validationSummary.passed
            ? "QLabel { color:green; font-weight:bold; }"
            : "QLabel { color:red; font-weight:bold; }");
        return;
    }
    const int wanted = m_spinValidationPoseCount ? m_spinValidationPoseCount->value() : 8;
    const bool safeDomain = m_checkValidationAvoidRejected && m_checkValidationAvoidRejected->isChecked()
        && !m_lastResult.rejectedTrainingPoses.empty();
    m_lblValidation->setText(tr("独立验证：已载入/采集 %1/%2 个hold-out姿态；这些数据未进入求解器。%3"
                                "达到要求数量后点击“执行独立验证”，也可使用自动移动+采集。")
                             .arg(m_validationRecords.size()).arg(wanted)
                             .arg(safeDomain
                                  ? tr(" 已启用高误差B/C危险区排除，当前验证仅覆盖安全工作域。")
                                  : QString()));
    m_lblValidation->setStyleSheet("QLabel { color:#7a5200; font-weight:bold; }");
}

void SingleSphere14CalibrationWidget::onClearValidation()
{
    if (m_validationRecords.empty() && !m_validationSummary.computed) return;
    if (QMessageBox::question(this, tr("清空独立验证集"),
        tr("确定清空全部hold-out验证数据和验证结论吗？训练/标定数据不会受到影响。"),
        QMessageBox::Yes|QMessageBox::No, QMessageBox::No) != QMessageBox::Yes) return;
    invalidateValidation(tr("独立验证集已清空。"), true);
    if (m_planValidation) updatePlanTable();
    updateSessionSummary();
    updateCalibrationReadiness();
    updateWorkflowGuide();
}

void SingleSphere14CalibrationWidget::onRunValidation()
{
    if (!m_hasResult || !m_lastResult.success) {
        QMessageBox::warning(this, tr("不能验证"), tr("请先完成阶段2最终17参数求解。"));
        return;
    }
    if (stage2TrainingObservations().empty()) {
        QMessageBox::warning(this, tr("不能验证"),
            tr("当前没有阶段2训练姿态来源。独立验证必须能够重新检查hold-out与训练网格的分离关系；"
               "请先导入与当前Physical17结果对应的阶段2训练CSV。"));
        return;
    }
    const int required = m_spinValidationPoseCount->value();
    if (static_cast<int>(m_validationRecords.size()) < required) {
        QMessageBox::information(this, tr("验证姿态不足"),
            tr("当前只有 %1 个独立验证姿态，设置要求 %2 个。请继续采集。")
                .arg(m_validationRecords.size()).arg(required));
        return;
    }

    for (const auto& record : m_validationRecords) {
        QString separationError;
        if (!validationPoseSeparated(record.usedPose, separationError)) {
            QMessageBox::warning(this, tr("验证集不再满足独立性"), separationError);
            return;
        }
    }

    std::vector<SphereProfileObservation> observations;
    for (const auto& record : m_validationRecords) {
        observations.insert(observations.end(), record.observations.begin(), record.observations.end());
    }
    const auto surface = SingleSphere17Calibrator::validateModel(
        observations, m_lastResult.parameters, m_lastResult.sphereCenterMachineMm,
        m_spinSphereDiameter->value());
    if (!surface.valid) {
        QMessageBox::warning(this, tr("验证失败"), QString::fromStdString(surface.message));
        return;
    }

    double centerXzSq = 0.0;
    double center3dSq = 0.0;
    double centerMax = 0.0;
    Vec3d bias{};
    bool hasBPos=false, hasBNeg=false, hasCPos=false, hasCNeg=false;
    int centerCount = 0;
    for (const auto& record : m_validationRecords) {
        const auto q = SingleSphere17QtAdapter::toPortable(record.usedPose);
        const Vec3d predicted = SingleSphere17Model::machinePointInSensor(
            q, m_lastResult.sphereCenterMachineMm, m_lastResult.parameters);
        Vec3d measured = record.representativeFit.centerSensorMm;
        measured.y = std::copysign(std::abs(measured.y), predicted.y);
        const double dx = measured.x - predicted.x;
        const double dy = measured.y - predicted.y;
        const double dz = measured.z - predicted.z;
        const double e3 = std::sqrt(dx*dx + dy*dy + dz*dz);
        centerXzSq += dx*dx + dz*dz;
        center3dSq += e3*e3;
        centerMax = std::max(centerMax, e3);
        bias.x += dx; bias.y += dy; bias.z += dz;
        ++centerCount;
        if (record.usedPose.b > 0.5) hasBPos=true;
        if (record.usedPose.b < -0.5) hasBNeg=true;
        if (record.usedPose.c > 0.5) hasCPos=true;
        if (record.usedPose.c < -0.5) hasCNeg=true;
        appendLog(tr("独立验证姿态：%1；预测球心S=%2；实测球心S≈%3；d=[%4,%5,%6] mm；|d|=%7 mm")
            .arg(poseText(record.usedPose)).arg(centerText(predicted)).arg(centerText(measured))
            .arg(dx,0,'f',4).arg(dy,0,'f',4).arg(dz,0,'f',4).arg(e3,0,'f',4));
    }

    ValidationSummary summary;
    summary.computed = true;
    summary.poseCount = static_cast<int>(m_validationRecords.size());
    summary.profileCount = surface.profileCount;
    summary.pointCount = surface.pointCount;
    summary.surfaceMeanMm = surface.residualMeanMm;
    summary.surfaceMaeMm = surface.residualMeanAbsMm;
    summary.surfaceRmsMm = surface.residualRmsMm;
    summary.surfaceMaxMm = surface.residualMaxAbsMm;
    if (centerCount > 0) {
        const double n = static_cast<double>(centerCount);
        summary.centerXzRmsMm = std::sqrt(centerXzSq / n);
        summary.center3dRmsMm = std::sqrt(center3dSq / n);
        summary.centerMaxMm = centerMax;
        summary.centerBiasSensorMm = {bias.x/n, bias.y/n, bias.z/n};
    }

    const bool needBSigns = m_spinBMin->value() < -0.5 && m_spinBMax->value() > 0.5;
    const bool needCSigns = m_spinCMin->value() < -0.5 && m_spinCMax->value() > 0.5;
    summary.bCoverageOk = !needBSigns || (hasBPos && hasBNeg);
    summary.cCoverageOk = !needCSigns || (hasCPos && hasCNeg);

    summary.passed = summary.poseCount >= required
        && summary.surfaceRmsMm <= m_spinValidationRms->value()
        && summary.surfaceMaxMm <= m_spinValidationMax->value()
        && summary.centerXzRmsMm <= m_spinValidationCenterXz->value()
        && summary.bCoverageOk && summary.cCoverageOk;

    const double trainRms = std::max(1e-12, m_lastResult.residualRmsMm);
    const double ratio = summary.surfaceRmsMm / trainRms;
    summary.message = tr(
        "%1：hold-out姿态=%2，profiles=%3，points=%4；球面 mean=%5 mm，MAE=%6 mm，RMS=%7 mm（上限%8），max=%9 mm（上限%10）；"
        "预测球心 XZ-RMS=%11 mm（上限%12），3D-RMS=%13 mm，max=%14 mm，biasS=%15；"
        "验证RMS/训练RMS=%16；B正负覆盖=%17，C正负覆盖=%18；验证范围=%19。")
        .arg(summary.passed ? tr("独立验证通过") : tr("独立验证未通过"))
        .arg(summary.poseCount).arg(summary.profileCount).arg(summary.pointCount)
        .arg(summary.surfaceMeanMm,0,'g',6).arg(summary.surfaceMaeMm,0,'g',6)
        .arg(summary.surfaceRmsMm,0,'g',6).arg(m_spinValidationRms->value(),0,'g',6)
        .arg(summary.surfaceMaxMm,0,'g',6).arg(m_spinValidationMax->value(),0,'g',6)
        .arg(summary.centerXzRmsMm,0,'g',6).arg(m_spinValidationCenterXz->value(),0,'g',6)
        .arg(summary.center3dRmsMm,0,'g',6).arg(summary.centerMaxMm,0,'g',6)
        .arg(centerText(summary.centerBiasSensorMm)).arg(ratio,0,'g',5)
        .arg(summary.bCoverageOk?tr("是"):tr("否"))
        .arg(summary.cCoverageOk?tr("是"):tr("否"))
        .arg((m_checkValidationAvoidRejected && m_checkValidationAvoidRejected->isChecked()
              && !m_lastResult.rejectedTrainingPoses.empty())
                 ? tr("已排除阶段2高残差B/C危险邻域的安全工作域")
                 : tr("配置B/C范围（仅要求与训练网格分离）"));

    m_validationSummary = summary;
    updateValidationSummary();
    updateResultTable(&m_lastResult);
    appendLog(summary.message);
    if (summary.passed) {
        // Validation itself is a pure offline computation.  Runtime authorization is
        // deliberately a separate gate: only save an active calibration when the
        // controller/sensor convention is currently bound and can be re-read live.
        QString conventionError;
        const bool liveConventionVerified = m_referenceTaught && validateSessionConventions(conventionError);
        QString persistenceError;
        if (liveConventionVerified && savePersistentCalibration(persistenceError)) {
            m_persistentCalibrationLoaded = true;
            m_persistentCalibrationBound = true;
            m_persistedValidationSummary = m_validationSummary;
            m_persistentBindError.clear();
            appendLog(tr("已自动保存当前有效标定：%1。下次启动会自动载入，并在设备连接后核对坐标约定。")
                      .arg(persistentCalibrationPath()));
            QMessageBox::information(this, tr("独立验证通过"),
                tr("当前17参数模型已经通过独立hold-out验证，且当前实机坐标约定已核对一致。"
                   "有效标定已经持久化；扫描/RTCP仍会在实际运行前经过运行时闸门。\n\n%1")
                    .arg(summary.message));
        } else if (liveConventionVerified) {
            appendLog(tr("独立验证已通过，但自动保存有效标定失败：%1").arg(persistenceError));
            QMessageBox::warning(this, tr("独立验证通过，但持久化失败"),
                tr("hold-out验证已经通过，但默认有效标定文件保存失败，因此不要依赖重启自动恢复。\n\n%1\n\n%2")
                    .arg(summary.message, persistenceError));
        } else {
            appendLog(tr("独立验证已离线通过；当前没有可实时复核的控制器/2950坐标约定，因此只记录验证结论，"
                         "不会生成可直接用于运行的默认有效标定。连接设备后使用“绑定已加载结果到当前实机坐标约定”完成最后闸门。%1")
                .arg(conventionError.isEmpty() ? QString() : tr(" 当前复核状态：%1").arg(conventionError)));
            QMessageBox::information(this, tr("独立验证离线通过"),
                tr("当前17参数模型已经通过独立hold-out验证。验证计算不需要控制器或2950在线。\n\n"
                   "但当前尚未实时核对设备FlipX/FlipZ和XYZCB的DPOS/MPOS来源，所以扫描/RTCP仍保持锁定。"
                   "连接设备并完成坐标约定绑定后，软件才会保存为默认有效标定。\n\n%1")
                    .arg(summary.message));
        }
    } else {
        // Failure applies to the newly trained candidate only. Keep the last accepted
        // calibration as fallback/seed instead of deleting it.
        appendLog(tr("当前新模型独立验证未通过；上一套已接受标定不会被撤销，仍可作为运行fallback和下一轮标定初值。"));
        QMessageBox::warning(this, tr("独立验证未通过"),
            tr("不要把当前模型直接用于最终扫描/RTCP。请检查具有方向性的误差，补充训练姿态后重新求解，再重新采集独立验证集。\n\n%1")
                .arg(summary.message));
    }
    updateCalibrationReadiness();
    updateWorkflowGuide();
}

bool SingleSphere14CalibrationWidget::writeValidationJson(const QString& fileName, QString& error) const
{
    if (!m_validationSummary.computed) {
        error = tr("尚未执行独立验证。");
        return false;
    }
    QJsonObject root;
    root.insert("model", QStringLiteral("single_sphere_physical17_holdout_validation"));
    root.insert("passed", m_validationSummary.passed);
    root.insert("pose_count", m_validationSummary.poseCount);
    root.insert("profile_count", static_cast<double>(m_validationSummary.profileCount));
    root.insert("point_count", static_cast<double>(m_validationSummary.pointCount));
    root.insert("surface_mean_mm", m_validationSummary.surfaceMeanMm);
    root.insert("surface_mae_mm", m_validationSummary.surfaceMaeMm);
    root.insert("surface_rms_mm", m_validationSummary.surfaceRmsMm);
    root.insert("surface_max_abs_mm", m_validationSummary.surfaceMaxMm);
    root.insert("center_xz_rms_mm", m_validationSummary.centerXzRmsMm);
    root.insert("center_3d_rms_mm", m_validationSummary.center3dRmsMm);
    root.insert("center_max_mm", m_validationSummary.centerMaxMm);
    root.insert("center_bias_sensor_mm", QJsonArray{
        m_validationSummary.centerBiasSensorMm.x,
        m_validationSummary.centerBiasSensorMm.y,
        m_validationSummary.centerBiasSensorMm.z});
    root.insert("b_positive_negative_coverage", m_validationSummary.bCoverageOk);
    root.insert("c_positive_negative_coverage", m_validationSummary.cCoverageOk);
    root.insert("training_surface_rms_mm", m_lastResult.residualRmsMm);
    QJsonArray physical17Parameters;
    for (double v : m_lastResult.parameters.value) physical17Parameters.append(v);
    root.insert("physical17_parameters", physical17Parameters);
    root.insert("required_pose_count", m_spinValidationPoseCount ? m_spinValidationPoseCount->value() : 8);
    root.insert("threshold_surface_rms_mm", m_spinValidationRms->value());
    root.insert("threshold_surface_max_mm", m_spinValidationMax->value());
    root.insert("threshold_center_xz_rms_mm", m_spinValidationCenterXz->value());
    root.insert("minimum_training_bc_separation_deg", m_spinValidationBCSep->value());
    const bool avoidRejected = m_checkValidationAvoidRejected && m_checkValidationAvoidRejected->isChecked()
        && !m_lastResult.rejectedTrainingPoses.empty();
    root.insert("avoid_rejected_training_bc_regions", avoidRejected);
    root.insert("rejected_training_bc_exclusion_deg",
                m_spinValidationHazardBCSep ? m_spinValidationHazardBCSep->value() : 10.0);
    root.insert("rejected_training_pose_count", static_cast<double>(m_lastResult.rejectedTrainingPoses.size()));
    root.insert("validation_scope", avoidRejected
        ? QStringLiteral("retained_safe_bc_domain_excluding_stage2_rejected_neighborhoods")
        : QStringLiteral("configured_bc_domain_with_training_grid_separation_only"));

    QJsonArray poses;
    for (const auto& record : m_validationRecords) {
        QJsonObject p;
        p.insert("time", record.timeText);
        p.insert("x_mm", record.usedPose.x);
        p.insert("y_mm", record.usedPose.y);
        p.insert("z_mm", record.usedPose.z);
        p.insert("c_deg", record.usedPose.c);
        p.insert("b_deg", record.usedPose.b);
        p.insert("valid_profiles", static_cast<int>(record.observations.size()));
        const auto q = SingleSphere17QtAdapter::toPortable(record.usedPose);
        const Vec3d predicted = SingleSphere17Model::machinePointInSensor(
            q, m_lastResult.sphereCenterMachineMm, m_lastResult.parameters);
        Vec3d measured = record.representativeFit.centerSensorMm;
        measured.y = std::copysign(std::abs(measured.y), predicted.y);
        p.insert("predicted_center_sensor_mm", QJsonArray{predicted.x,predicted.y,predicted.z});
        p.insert("measured_center_sensor_mm", QJsonArray{measured.x,measured.y,measured.z});
        const double dx=measured.x-predicted.x, dy=measured.y-predicted.y, dz=measured.z-predicted.z;
        p.insert("center_error_sensor_mm", QJsonArray{dx,dy,dz});
        p.insert("center_error_norm_mm", std::sqrt(dx*dx+dy*dy+dz*dz));
        const auto perPose = SingleSphere17Calibrator::validateModel(
            record.observations, m_lastResult.parameters, m_lastResult.sphereCenterMachineMm,
            m_spinSphereDiameter->value());
        p.insert("surface_rms_mm", perPose.residualRmsMm);
        p.insert("surface_max_abs_mm", perPose.residualMaxAbsMm);
        poses.append(p);
    }
    root.insert("validation_poses", poses);

    QFile f(fileName);
    if (!f.open(QIODevice::WriteOnly|QIODevice::Truncate)) {
        error = f.errorString();
        return false;
    }
    if (f.write(QJsonDocument(root).toJson(QJsonDocument::Indented)) < 0) {
        error = f.errorString();
        return false;
    }
    return true;
}

void SingleSphere14CalibrationWidget::onDeleteLastCapture()
{
    if (m_records.empty()) return;
    if (!m_validationRecords.empty() || m_validationSummary.computed) {
        invalidateValidation(tr("训练数据被删除，旧独立验证结果作废；最终模型重新求解后需要重新验证。"), true);
    }
    const CaptureRecord removed = m_records.back();
    const int pi = removed.planIndex;
    m_records.pop_back();
    rebuildCalibratorFromRecords();
    if (pi>=0) markPlanStatus(pi,tr("待采集"));

    if (removed.stage1Bootstrap) {
        // 阶段1解依赖全部人工姿态。删除其中任何一组后必须重新求阶段1，
        // 否则手眼/球心与实际数据集不一致。
        m_hasBootstrap=false;
        m_bootstrapRecordCount=0;
        m_bootstrapResult=NominalHandEyeSphereResult{};
        m_hasResult=false;
        m_currentEstimate=Physical17Parameters{};
        m_sphereCenterGuessMachine=Vec3d{};
        m_plan.clear();
        m_planValidation=false;
        m_planTable->setRowCount(0);
        m_lblBootstrap->setText(tr("阶段1数据已删除一组；请重新执行阶段1手眼+球心求解。"));
        m_stage1SolveElapsedMs=0;
        updateStage1ResultDisplay();
    } else if (m_hasResult) {
        m_hasResult=false;
        m_refinementSolved=false;
        if (m_planValidation) {
            m_plan.clear();
            m_planValidation=false;
            m_planTable->setRowCount(0);
        }
        m_lblSolveState->setText(tr("阶段2数据已删除一组；旧17参数结果仅保留为初值，请重新最终求解。"));
    }
    updateSessionSummary();
    updateCalibrationReadiness();
    updateWorkflowGuide();
}

void SingleSphere14CalibrationWidget::onClearSession()
{
    if (m_stage2SimulationActive) {
        finishStage2Simulation(tr("清空会话前已停止运动学标定姿态仿真。"));
    }
    if (!m_stage2SimulationCloud.isEmpty()) {
        m_stage2SimulationCloud.clear();
        emit requestStage2SimulationClear(true);
        emit requestProfileOverlayClear();
    }
    if (m_stage2SolveRunning) {
        QMessageBox::information(this,tr("阶段2正在求解"),tr("请先等待求解完成，或点击“取消阶段2求解”后再清空会话。"));
        return;
    }
    if (QMessageBox::question(this,tr("清空标定会话"),
        tr("确定清空阶段1人工数据、阶段1手眼/球心结果、阶段2训练计划/采集、独立验证集和最终17参数结果吗？"),
        QMessageBox::Yes|QMessageBox::No,QMessageBox::No)!=QMessageBox::Yes) return;

    m_records.clear();
    m_validationRecords.clear();
    m_validationSummary=ValidationSummary{};
    m_persistentCalibrationLoaded=false;
    m_persistentCalibrationBound=false;
    m_persistedValidationSummary=ValidationSummary{};
    m_persistentBindError.clear();
    m_plan.clear();
    m_referenceTaught=false;
    m_referenceCaptured=false;
    m_sensorConventionLocked=false;
    m_referenceFeedbackMask=0;
    m_modelFeedbackMask=0;
    m_hasBootstrap=false;
    m_bootstrapRecordCount=0;
    m_bootstrapResult=NominalHandEyeSphereResult{};
    m_hasResult=false;
    m_hasFallbackResult=false;
    m_fallbackResult=SingleSphere17Result{};
    m_fallbackValidationSummary=ValidationSummary{};
    m_planBootstrap=false;
    m_fullPlanGenerated=false;
    m_planValidation=false;
    m_refinementSolved=false;
    m_hasLatestFit=false;
    m_needFitBeforeTransit=false;
    m_pendingMoveKind=PendingMoveKind::None;
    m_pendingMovePlanIndex=-1;
    m_autoPlanMove=false;
    m_autoPlanIndex=-1;
    m_autoStage2Acquisition=false;
    m_autoStage2StopRequested=false;
    m_autoValidationAcquisition=false;
    m_autoValidationStopRequested=false;
    m_moveInProgress=false;
    m_capturePendingStart=false;
    m_stage1SolveRunning=false;
    m_stage1SolveElapsedMs=0;
    m_stage1Candidates.clear();
    m_loadedStage1CapturedPoses.clear();
    m_stage1CandidateStatus.clear();
    m_activeStage1Candidate=-1;
    m_stage2SolveRunning=false;
    m_stage2SolveCancelFlag.reset();
    m_stage2SolveStartMs=0;
    if(m_stage2SolveTimer)m_stage2SolveTimer->stop();
    if(m_stage2SolveProgress)m_stage2SolveProgress->setValue(0);
    if(m_lblStage2SolveProgress)m_lblStage2SolveProgress->setText(tr("阶段2求解空闲。"));
    if(m_lblStage2SolveElapsed)m_lblStage2SolveElapsed->setText(tr("耗时：0.0 s"));
    if(m_stage2SolveDetails)m_stage2SolveDetails->clear();
    if(m_stage1CandidateTable)m_stage1CandidateTable->setRowCount(0);
    m_currentEstimate=Physical17Parameters{};
    m_sphereCenterGuessMachine=Vec3d{};
    m_stage2ImportedSphereCloud.clear();
    m_stage2ImportedSphereCloudPath.clear();
    if (m_lblStage2Sphere) m_lblStage2Sphere->setText(tr("未导入点云；姿态规划将使用阶段1/上次有效标定中的固定球心。"));
    if (m_btnFitStage2Sphere) m_btnFitStage2Sphere->setEnabled(false);
    emit requestMachineSafetySphere(0.0, 0.0, 0.0, 0.0, false);
    m_lastResult=SingleSphere17Result{};
    rebuildCalibratorFromRecords();
    m_planTable->setRowCount(0);
    m_btnGenerate->setText(tr("生成质量约束 + D-optimal 标定姿态"));
    m_lblConvention->setText(tr("坐标约定：阶段1尚未开始；Flip X/Flip Z 与 DPOS/MPOS 数据源尚未锁定。"));
    m_lblBootstrap->setText(tr("阶段1：尚未求解"));
    emit requestProfileOverlayClear();
    updateValidationSummary();
    updateSessionSummary();
    updateCalibrationReadiness();
    updateResultTable(nullptr);
    updateWorkflowGuide();
    updateStage1Status();
    updateStage1ResultDisplay();
}

Physical17SolverOptions SingleSphere14CalibrationWidget::solverOptionsFromUi() const
{
    Physical17SolverOptions o;
    o.sphereDiameterMm = m_spinSphereDiameter->value();
    // Keep the established Stage-1/default schedule.  Stage 2 overrides the
    // representative joint budget locally in onSolve(), then performs a
    // separate full-training-data refinement with the profile-block solver.
    o.alternatingIterations = 2;
    o.blockIterations = 6;
    o.jointIterations = 18;
    o.multiStartCount = 2;
    return o;
}

void SingleSphere14CalibrationWidget::updateSessionSummary()
{
    int stage1 = 0;
    for (const auto& r : m_records) if (r.stage1Bootstrap) ++stage1;
    const auto training = stage2TrainingObservations();
    std::size_t points=0;
    for(const auto& o:training) points+=o.points.size();
    const int stage2Poses = stage2TrainingPoseCount();
    m_lblSession->setText(tr("阶段1会话=%1；阶段1人工姿态=%2；阶段1求解=%3；阶段2训练姿态=%4（求解硬下限8，最终仍须rank/condition通过）；阶段2有效profiles=%5；球面点=%6；当前计划=%7；独立验证=%8/%9")
        .arg(m_referenceTaught?tr("已锁定"):(m_hasBootstrap?tr("离线恢复"):tr("未开始")))
        .arg(stage1>0?stage1:m_bootstrapRecordCount)
        .arg(m_hasBootstrap?tr("rank9通过"):tr("未通过"))
        .arg(stage2Poses)
        .arg(static_cast<qulonglong>(training.size())).arg(static_cast<qulonglong>(points)).arg(m_plan.size())
        .arg(m_validationRecords.size())
        .arg(m_validationSummary.passed?tr("通过"):tr("未通过/未执行")));
}

void SingleSphere14CalibrationWidget::updateCalibrationReadiness()
{
    updateStage2MotionReadinessUi();
    const bool idleUi = !m_moveInProgress && !m_collecting && !m_stage1SolveRunning
        && !m_stage2PlanRunning && !m_stage2SolveRunning && !m_stage2SimulationActive
        && !m_autoStage2Acquisition && !m_autoValidationAcquisition;
    const bool controllerOnline = m_controller && m_controller->isConnected();
    const bool sensorOnline = m_sensor && m_sensor->isConnected();
    const bool liveMotionSession = m_referenceTaught && controllerOnline;
    const bool liveAcquisitionSession = liveMotionSession && sensorOnline;
    const bool canRebindValidationMotion = !m_referenceTaught && controllerOnline && sensorOnline
        && m_hasResult && m_lastResult.success && m_hasBootstrap && m_bootstrapResult.success;
    const bool hasStage2Training = !stage2TrainingObservations().empty();
    // Planning/simulation is a read-only operation and is intentionally allowed
    // after offline Stage-1/Physical17 restoration.  Motion/capture remains gated
    // by the live session below.
    const bool canGenerateStage2Plan = m_hasBootstrap && idleUi;

    if (m_btnStage2SimulationStart)
        m_btnStage2SimulationStart->setEnabled(m_hasBootstrap && !m_plan.empty() && !m_planValidation && idleUi);
    if (m_btnStage2SimulationStop)
        m_btnStage2SimulationStop->setEnabled(m_stage2SimulationActive);
    if (m_btnStage2SimulationClear)
        m_btnStage2SimulationClear->setEnabled(!m_stage2SimulationActive && !m_stage2SimulationCloud.isEmpty());
    if (m_spinStage2SimulationIntervalMs)
        m_spinStage2SimulationIntervalMs->setEnabled(!m_stage2SimulationActive);
    if (m_spinStage2XyzSpeedScale) m_spinStage2XyzSpeedScale->setEnabled(idleUi);
    if (m_spinStage2BcSpeedScale) m_spinStage2BcSpeedScale->setEnabled(idleUi);
    if (m_btnAutoAcquireStage2)
        m_btnAutoAcquireStage2->setEnabled(liveAcquisitionSession && m_hasBootstrap && !m_plan.empty()
                                          && !m_planValidation && idleUi);
    if (m_btnStopAutoAcquireStage2)
        m_btnStopAutoAcquireStage2->setEnabled(m_autoStage2Acquisition);
    if (m_btnAutoAcquireValidation)
        m_btnAutoAcquireValidation->setEnabled((liveAcquisitionSession || canRebindValidationMotion)
                                               && m_hasResult && !m_plan.empty()
                                               && m_planValidation && idleUi);
    if (m_btnStopAutoAcquireValidation)
        m_btnStopAutoAcquireValidation->setEnabled(m_autoValidationAcquisition);
    if (m_btnMove) m_btnMove->setEnabled(liveMotionSession && m_hasBootstrap && !m_plan.empty() && idleUi);
    if (m_btnSkipPlanPose) m_btnSkipPlanPose->setEnabled(!m_plan.empty() && idleUi);
    if (m_btnGenerateValidation) {
        m_btnGenerateValidation->setEnabled(m_hasResult && idleUi && hasStage2Training);
        if (!m_hasResult)
            m_btnGenerateValidation->setToolTip(tr("请先完成阶段2最终Physical17求解。"));
        else if (!hasStage2Training)
            m_btnGenerateValidation->setToolTip(tr("请先加载与当前Physical17对应的阶段2训练CSV，用于hold-out训练网格隔离。"));
        else if (!m_referenceTaught)
            m_btnGenerateValidation->setToolTip(tr("可离线生成独立验证姿态；计划生成不会授权运动。真正执行时将重新核对当前实机坐标约定和首段MachineSafety。"));
        else
            m_btnGenerateValidation->setToolTip(tr("生成与训练网格错开的独立验证姿态，并避开阶段2筛出的高残差B/C危险区。"));
    }
    if (m_checkValidationAvoidRejected) m_checkValidationAvoidRejected->setEnabled(idleUi);
    if (m_spinValidationHazardBCSep)
        m_spinValidationHazardBCSep->setEnabled(idleUi && m_checkValidationAvoidRejected
                                                && m_checkValidationAvoidRejected->isChecked());
    if (m_btnCaptureValidation)
        m_btnCaptureValidation->setEnabled(liveAcquisitionSession && m_hasResult && m_planValidation && idleUi);
    if (m_btnImportValidation)
        m_btnImportValidation->setEnabled(m_hasResult && idleUi && hasStage2Training);
    if (m_btnExportValidation)
        m_btnExportValidation->setEnabled(m_hasResult && idleUi && !m_validationRecords.empty());
    if (m_btnRunValidation)
        m_btnRunValidation->setEnabled(m_hasResult && idleUi
            && static_cast<int>(m_validationRecords.size()) >= (m_spinValidationPoseCount ? m_spinValidationPoseCount->value() : 8));
    if (m_btnClearValidation)
        m_btnClearValidation->setEnabled(!m_validationRecords.empty() || m_validationSummary.computed);
    if (m_btnCapture)
        m_btnCapture->setEnabled(liveAcquisitionSession && !m_planValidation && idleUi);
    if (m_btnDeleteLast)
        m_btnDeleteLast->setEnabled(!m_planValidation && !m_records.empty() && idleUi);
    if (m_btnCancelStage2Solve) m_btnCancelStage2Solve->setEnabled(m_stage2SolveRunning);
    if (m_btnCancelStage2Plan) m_btnCancelStage2Plan->setEnabled(m_stage2PlanRunning);
    if (m_btnLoadStage1Result) m_btnLoadStage1Result->setEnabled(idleUi && !m_referenceTaught && m_records.empty());
    if (m_btnClear) m_btnClear->setEnabled(idleUi);
    if (m_btnExportSession) m_btnExportSession->setEnabled(idleUi && !stage2TrainingObservations().empty());
    // V27.6.42A: Stage-2 CSV import is an offline-data operation and must remain
    // available even when a previously validated calibration has auto-bound to the
    // connected machine.  The import handler explicitly switches out of the live
    // motion session after confirmation; live captured records still block import.
    if (m_btnImportSession) m_btnImportSession->setEnabled(idleUi && m_records.empty());
    if (m_btnSaveResult) m_btnSaveResult->setEnabled(idleUi && m_hasResult);
    if (m_btnLoadResult) m_btnLoadResult->setEnabled(idleUi && !m_referenceTaught && m_records.empty());
    if (m_btnBindLoadedResult) {
        const bool devicesReady = m_controller && m_controller->isConnected()
            && m_sensor && m_sensor->isConnected();
        const bool hasStage1Convention = m_persistentCalibrationLoaded
            || (m_hasBootstrap && m_bootstrapResult.success);
        m_btnBindLoadedResult->setEnabled(idleUi && !m_referenceTaught && m_hasResult
            && hasStage1Convention && devicesReady);
    }

    if (m_stage2PlanRunning) {
        if (m_lblObservability) {
            m_lblObservability->setText(tr("阶段2 D-optimal姿态正在后台生成；界面保持响应，可查看规划进度或取消。"));
            m_lblObservability->setStyleSheet("QLabel {color:#245b8a;font-weight:bold;}");
        }
        m_btnBootstrapSolve->setEnabled(false);
        m_btnGenerate->setEnabled(false);
        m_btnSolve->setEnabled(false);
        emit calibrationReadyChanged(false);
        updateWorkflowGuide();
        notifyRuntimeCalibrationUsability();
        return;
    }

    if (m_stage2SolveRunning) {
        if (m_lblObservability) {
            m_lblObservability->setText(tr("阶段2最终求解正在后台运行；界面保持响应。请查看下方进度与详细输出。"));
            m_lblObservability->setStyleSheet("QLabel {color:#245b8a;font-weight:bold;}");
        }
        m_btnBootstrapSolve->setEnabled(false);
        m_btnGenerate->setEnabled(false);
        m_btnSolve->setEnabled(false);
        emit calibrationReadyChanged(false);
        updateWorkflowGuide();
        notifyRuntimeCalibrationUsability();
        return;
    }

    if (m_referenceTaught && controllerOnline && sensorOnline) {
        QString conventionError;
        if (!validateSessionConventions(conventionError)) {
            m_lblObservability->setText(tr("标定暂停：%1").arg(conventionError));
            m_lblObservability->setStyleSheet("QLabel {color:red;font-weight:bold;}");
            m_btnBootstrapSolve->setEnabled(false);
            m_btnGenerate->setEnabled(false);
            m_btnSolve->setEnabled(false);
            if (m_btnGenerateValidation) m_btnGenerateValidation->setEnabled(false);
            if (m_btnCaptureValidation) m_btnCaptureValidation->setEnabled(false);
            if (m_btnRunValidation) m_btnRunValidation->setEnabled(false);
            emit calibrationReadyChanged(false);
            updateWorkflowGuide();
            notifyRuntimeCalibrationUsability();
            return;
        }
    }

    int stage1Records = 0;
    for (const auto& r : m_records) if (r.stage1Bootstrap) ++stage1Records;
    m_btnBootstrapSolve->setEnabled(m_referenceTaught && stage1Records >= 8 && idleUi);
    // Generating a plan is not a machine move. Offline-restored Stage-1/Physical17
    // may generate and simulate a plan; only move/capture buttons require live binding.
    m_btnGenerate->setEnabled(canGenerateStage2Plan);
    m_btnGenerate->setToolTip(liveMotionSession
        ? tr("使用当前实机MPOS作为首段起点生成D-optimal计划；生成后仍建议先运行仿真。")
        : tr("离线生成D-optimal计划：使用保存模型、当前参考球心和Machine5X离线限位；不会发送运动命令。"));

    if (!m_hasBootstrap) {
        // V27.6.41: a Stage-1 file is no longer a prerequisite for OFFLINE final
        // solve.  If Stage-2 training observations are already available, onSolve()
        // first solves a nominal-machine 9-state hand-eye+sphere bootstrap from
        // those SAME observations and uses its translation as the rank-14 gauge
        // anchor.  Stage-1 remains required for safe online planning/motion because
        // it carries the live coordinate-convention provenance.
        const auto directTraining = stage2TrainingObservations();
        std::size_t directPoints = 0;
        for (const auto& o : directTraining) directPoints += o.points.size();
        const int directPoses = stage2TrainingPoseCount();
        const bool canSelfBootstrapSolve = directPoses >= 8
            && directTraining.size() >= 8 && directPoints >= 80 && idleUi;

        if (m_hasResult && m_lastResult.success) {
            m_lblObservability->setText(tr(
                "当前Physical17结果已存在，但没有加载阶段1/实机坐标约定来源。"
                "可继续离线复核训练数据或导入hold-out验证；真实运动、扫描/RTCP和新姿态规划仍需绑定坐标约定。"
                "若重新点击最终求解，V27.6.41会再次从当前阶段2训练数据自举hand-eye并建立新的平移规范锚点。"));
            m_lblObservability->setStyleSheet("QLabel {color:#245b8a;font-weight:bold;}");
            m_btnSolve->setEnabled(canSelfBootstrapSolve);
            emit calibrationReadyChanged(false);
            updateWorkflowGuide();
            notifyRuntimeCalibrationUsability();
            return;
        }

        if (!directTraining.empty() && !m_referenceTaught) {
            m_lblObservability->setText(tr(
                "已导入阶段2训练CSV：姿态=%1，profiles=%2，点=%3。"
                "%4 最终求解将自动先用这些数据求9状态hand-eye+球心，再把其Tx/Ty/Tz固定为translation-gauge anchor，"
                "不需要另行加载阶段1 JSON。")
                .arg(directPoses)
                .arg(static_cast<qulonglong>(directTraining.size()))
                .arg(static_cast<qulonglong>(directPoints))
                .arg(canSelfBootstrapSolve ? tr("数据量达到启动下限，可以直接点击“阶段2最终求解”。")
                                           : tr("当前数据量还没达到8姿态/8 profiles/80点的启动下限。")));
            m_lblObservability->setStyleSheet(canSelfBootstrapSolve
                ? "QLabel {color:green;font-weight:bold;}"
                : "QLabel {color:#b06b00;font-weight:bold;}");
            m_btnSolve->setEnabled(canSelfBootstrapSolve);
        } else {
            m_lblObservability->setText(tr("阶段1：人工姿态=%1；在线生成/执行阶段2姿态仍需要先得到手眼+球心 rank 9/9。"
                                            "如果已有阶段2 CSV，也可以直接离线导入并由最终求解自动自举hand-eye。")
                                        .arg(stage1Records));
            m_lblObservability->setStyleSheet("QLabel {color:#245b8a;font-weight:bold;}");
            m_btnSolve->setEnabled(false);
        }
        emit calibrationReadyChanged(false);
        updateWorkflowGuide();
        notifyRuntimeCalibrationUsability();
        return;
    }

    // A fully validated persisted model does not need the original Stage-2 CSV
    // to be used again. The CSV is training provenance; runtime needs the accepted
    // Physical17 vector plus the verified device coordinate convention.
    if (m_persistentCalibrationLoaded && m_hasResult && m_validationSummary.passed) {
        QString liveConventionError;
        const bool liveConventionVerified = m_referenceTaught && validateSessionConventions(liveConventionError);
        if (liveConventionVerified) {
            m_lblObservability->setText(tr(
                "已恢复有效标定：Physical17训练求解和独立hold-out验证均已通过；"
                "当前 Flip/DPOS-MPOS 坐标约定已核对，可继续用于扫描/RTCP；同时允许直接启动新一轮阶段2重新标定，旧结果会作为导航初值和运行回退。"));
            m_lblObservability->setStyleSheet("QLabel {color:green;font-weight:bold;}");
            m_btnSolve->setEnabled(false);
            if (m_btnGenerate) {
                m_btnGenerate->setEnabled(canGenerateStage2Plan);
                m_btnGenerate->setToolTip(tr("当前已验证17参数继续作为运行模型和新一轮标定初值；可直接重新生成质量约束+D-optimal训练姿态，无需重复阶段1。"));
            }
            emit calibrationReadyChanged(true);
        } else {
            m_lblObservability->setText(tr(
                "已自动载入上次验证通过的17参数标定。当前尚未完成实时设备坐标约定复核，因此禁止真实运动/采集/扫描/RTCP；"
                "但可以离线导入标准球点云、更新球心、生成D-optimal姿态并运行数字孪生/圆轮廓仿真。%1")
                .arg(liveConventionError.isEmpty()?QString():tr(" 当前状态：%1").arg(liveConventionError)));
            m_lblObservability->setStyleSheet("QLabel {color:#245b8a;font-weight:bold;}");
            m_btnSolve->setEnabled(false);
            emit calibrationReadyChanged(false);
        }
        updateWorkflowGuide();
        notifyRuntimeCalibrationUsability();
        return;
    }

    // One-time migration path for an older Stage-2 JSON whose companion
    // *_validation.json was successfully recovered. It is usable without the old
    // training CSV once the matching Stage-1 convention has been bound to hardware.
    if (m_hasResult && m_validationSummary.passed) {
        QString liveConventionError;
        const bool liveConventionVerified = m_referenceTaught && validateSessionConventions(liveConventionError);
        if (liveConventionVerified) {
            m_lblObservability->setText(tr(
                "已验证标定已绑定当前实机：11项机床几何误差 + 6DoF手眼关系可直接用于扫描/RTCP；也可直接用该结果作为初值重新生成阶段2训练姿态。"));
            m_lblObservability->setStyleSheet("QLabel {color:green;font-weight:bold;}");
            m_btnSolve->setEnabled(false);
            if (m_btnGenerate) {
                m_btnGenerate->setEnabled(canGenerateStage2Plan);
                m_btnGenerate->setToolTip(tr("当前已验证17参数继续作为运行模型和新一轮标定初值；可直接重新生成质量约束+D-optimal训练姿态，无需重复阶段1。"));
            }
            emit calibrationReadyChanged(true);
        } else {
            m_lblObservability->setText(tr(
                "阶段2结果的独立hold-out验证已经通过，但当前尚未实时核对设备坐标约定。"
                "离线规划/仿真仍可使用该模型与当前球心；真实运行前必须加载对应阶段1来源并完成Flip/DPOS-MPOS绑定。%1")
                .arg(liveConventionError.isEmpty()?QString():tr(" 当前状态：%1").arg(liveConventionError)));
            m_lblObservability->setStyleSheet("QLabel {color:#245b8a;font-weight:bold;}");
            m_btnSolve->setEnabled(false);
            emit calibrationReadyChanged(false);
        }
        updateWorkflowGuide();
        notifyRuntimeCalibrationUsability();
        return;
    }

    const auto quick = reducedStage2Observations(1, 80);
    if (quick.empty()) {
        m_lblObservability->setText(m_referenceTaught
            ? tr("阶段2：尚无有效训练球面数据。")
            : tr("已离线加载阶段1/导航模型：可导入标准球点云后生成并仿真D-optimal姿态；如需离线求解已有数据，再导入阶段2 CSV。"));
        m_lblObservability->setStyleSheet("QLabel {color:#245b8a;font-weight:bold;}");
        m_btnSolve->setEnabled(false);
        emit calibrationReadyChanged(false);
        updateWorkflowGuide();
        notifyRuntimeCalibrationUsability();
        return;
    }

    SingleSphere17Calibrator quickCalibrator(solverOptionsFromUi());
    for (const auto& o : quick) quickCalibrator.addObservation(o);
    const Vec3d center = m_sphereCenterGuessMachine;
    const auto report = quickCalibrator.analyzeObservability(m_currentEstimate, center);
    // Finite-difference Jacobians may numerically leak one tiny gauge singular value,
    // reporting 15 rather than 14.  Rank >= expected is sufficient; rank < expected is not.
    const bool ready = report.targetRank >= report.expectedTargetRank
        && report.fullStateRank >= report.expectedFullStateRank;
    m_lblObservability->setText(tr(
        "阶段2快速可观测性（每姿态1帧/最多80点）：物理 rank=%1/17（至少14）；完整状态 rank=%2/20（至少17）；"
        "σmin=%3；cond=%4；%5。最终求解会在后台使用每姿态代表帧优化，并用全部阶段2数据复核RMS/rank。")
        .arg(report.targetRank).arg(report.fullStateRank)
        .arg(report.smallestSingularValue,0,'g',5).arg(report.conditionNumber,0,'g',5)
        .arg(ready?tr("可以执行最终联合标定"):tr("继续采集不同B/C和视场位置")));
    m_lblObservability->setStyleSheet(ready?"QLabel {color:green;font-weight:bold;}":"QLabel {color:#b06b00;font-weight:bold;}");
    m_btnSolve->setEnabled(ready && idleUi);
    emit calibrationReadyChanged(ready);
    updateWorkflowGuide();
    updateStage1Status();
    notifyRuntimeCalibrationUsability();
}

void SingleSphere14CalibrationWidget::updateWorkflowGuide()
{
    if (!m_lblWorkflow) return;
    QString text;
    if (m_stage2SolveRunning) {
        text = tr("阶段2最终求解正在后台执行：请查看本页下方进度条和详细输出。求解期间机械运动/采集按钮被锁定；需要停止时点击“取消阶段2求解”。");
    } else if (!m_referenceTaught && m_persistentCalibrationLoaded && m_hasResult && m_validationSummary.passed) {
        text = tr("已发现上次完成且独立验证通过的标定：17参数中同时包含11项机床几何误差和6DoF手眼关系。"
                  "离线状态下现在可以先导入当前标准球点云、RANSAC更新机床坐标球心，然后直接生成D-optimal运动学标定姿态并运行数字孪生/圆轮廓仿真；此过程不会发送运动命令。"
                  "真实移动/采集/扫描/RTCP仍必须连接控制器和2950，并核对保存时的FlipX/FlipZ及XYZCB DPOS/MPOS来源后才能解锁。");
    } else if (!m_referenceTaught && m_hasResult && m_validationSummary.passed) {
        text = tr("离线验证已经通过：阶段2的17参数已经具备hold-out证据。现在可以离线导入标准球点云、更新球心并生成/仿真新的D-optimal训练姿态；路径生成与真实运行授权彼此分离。"
                  "若要真实移动、采集、扫描或RTCP，请加载对应阶段1来源信息并连接控制器和2950，再完成当前FlipX/FlipZ、DPOS/MPOS来源和平移规范锚点绑定。 ");
    } else if (!m_referenceTaught && m_hasResult) {
        if (stage2TrainingObservations().empty()) {
            text = tr("离线恢复：阶段2的17参数结果已经加载。先导入与该结果对应的阶段2训练CSV，软件会用训练集profile/point计数和全训练集RMS/max残差复核来源。"
                      "复核通过后，可选择导入V27.6.15单独保存的hold-out CSV并直接离线执行验证；也可以之后连接设备重新采集验证集。"
                      "无论哪种方式，扫描/RTCP都必须再经过实机坐标约定绑定。 ");
        } else {
            text = tr("离线/CSV阶段2求解结果和对应训练数据已就绪。现在可以直接点击“生成独立验证姿态”；V27.6.42B允许在未绑定实机运动会话时做只读hold-out规划，"
                      "并继续避开阶段2筛出的高残差B/C危险区。若设备已连接且保留了阶段1/默认标定坐标约定来源，点击“自动移动 + 采集独立验证”时会先严格核对FlipX/FlipZ和XYZCB DPOS/MPOS来源，"
                      "重新绑定本次新Physical17仅用于验证运动，再从当时MPOS对首姿态重新做MachineSafety检查；不会恢复旧hold-out通过结论。"
                      "也可以点击“导入独立验证 CSV（离线）”恢复以前单独保存的hold-out数据。验证通过以前扫描/RTCP仍保持锁定。 ");
        }
    } else if (!m_referenceTaught && m_hasBootstrap) {
        if (stage2TrainingObservations().empty())
            text = tr("离线恢复：阶段1 JSON 已加载。你可以①导入标准球点云并RANSAC更新球心，然后直接生成D-optimal运动学标定姿态并运行圆轮廓仿真；或②导入阶段2训练CSV后离线执行最终求解。V27.6.41最终求解仍会从当前阶段2数据重新自举hand-eye和平移规范锚点。离线模式不会发送任何机械运动。 ");
        else
            text = tr("离线恢复：阶段1 JSON 与采集CSV均已加载。阶段1结果可用于规划/导航；最终17参数求解会从当前阶段2训练数据重新求9状态hand-eye+球心，并用该Tx/Ty/Tz作为translation-gauge anchor。 ");
    } else if (!m_referenceTaught && !stage2TrainingObservations().empty()) {
        text = tr("离线直接求解：阶段2训练CSV已经导入，不需要额外的阶段1 JSON。点击“阶段2最终求解”后，程序会先在当前训练数据上执行名义机床9状态 hand-eye 6DoF + 固定球心自举，"
                  "然后令11项机床几何误差初值为0、hand-eye初值为自举结果，并固定自举得到的Tx/Ty/Tz作为单球rank-14模型的3D translation gauge，再优化其余14个可辨识物理自由度。"
                  "这一步和Physical17使用同一训练集，因此完成后仍必须使用独立hold-out数据验证。 ");
    } else if (!m_referenceTaught) {
        text = tr("步骤1/6：在线采集流程仍先在本页“阶段1”统一操作区确认设备并锁定坐标约定；也可以直接导入已有阶段2训练CSV做离线自举+Physical17求解。"
                  "真实运动/扫描/RTCP仍必须具有可验证的FlipX/FlipZ和XYZCB DPOS/MPOS来源。 ");
    } else if (!m_hasBootstrap) {
        int stage1 = 0;
        for (const auto& r : m_records) if (r.stage1Bootstrap) ++stage1;
        text = tr("步骤2/6：可点击“生成/重生成阶段1候选姿态”，逐条直接移动；到位后球可见就点击“③ 采集当前阶段1姿态（自动拟合）”。"
                  "候选找不到球时保留XYZ/B/C点动微调，或跳过该候选。也可完全手动采集。不需要判断球心Y正负。当前人工姿态=%1；建议12~20个，且B、C都独立变化。达到至少8个后可尝试阶段1求解。")
                  .arg(stage1);
    } else if (m_plan.empty()) {
        text = tr("步骤3/6：阶段1已经得到手眼6DoF和固定球心。现在点击“根据阶段1结果生成17参数D-optimal姿态”。"
                  "阶段1结果用于把球可靠地保持在传感器视场内并生成/执行安全采集姿态；采集完成后的V27.6.41最终求解会从实际Stage2训练数据重新求9状态hand-eye+球心，并以该次自举Tx/Ty/Tz作为内部translation-gauge anchor，无需人工填写Tx/Ty/Tz。 ");
    } else if (!m_hasResult) {
        text = tr("步骤4/6：按质量约束 + D-optimal计划采集17参数训练数据。默认D限定92~100 mm，并在设置的γ/η候选域内先做scanCONTROL梯形FOV/可见弧仿真和MachineSafety目标/扫掠路径检查。"
                  "生成计划后建议先点击“仿真运行标定姿态”，在三维数字孪生和二维轮廓页逐姿态检查预测截圆/有效圆弧，再进行真实运动与采集。"
                  "可逐姿态人工确认，也可使用“自动移动并采集阶段2计划”。正式采集只把RANSAC圆内点中裁掉两端后的高质量点送入Physical17求解器；RMS/P95/内点比例/有效帧比例不合格的整个位姿自动丢弃。"
                  "同时保持不同B/C和视场位置的D-optimal信息量，直到物理rank=14/17、完整rank=17/20。");
    } else if (!m_validationSummary.passed) {
        text = tr("步骤5/6：17参数训练求解已完成，但还不能直接用于最终扫描。V27.6.42生成hold-out时默认避开阶段2渐进筛选已经识别的高残差B/C危险邻域，"
                  "并保留与训练网格的最小B/C间隔；因此验证通过只代表保留的安全工作域。在线方式可点击“自动移动 + 采集独立验证”，运动与阶段2相同，"
                  "X/Y/Z/C/B分别使用Single_MoveAbs独立轴绝对定位，不进行五轴插补；全部停稳并通过球拟合后自动采集fresh profiles。"
                  "离线方式仍可导入单独保存的hold-out CSV。达到设置姿态数后点击“执行独立验证”。");
    } else {
        text = tr("步骤6/6：独立hold-out验证已通过。若启用了高误差B/C危险区排除，该结论只覆盖保留的安全B/C工作域；被排除区域仍应视为未验证/不推荐运行。当前17参数模型可在受验证工作域和当前translation gauge下应用到五轴扫描/RTCP。"
                  "如果要再次标定，不需要删除这个结果或重复阶段1：在设备坐标约定实时绑定后可直接重新生成质量约束+D-optimal阶段2姿态；旧验证模型会作为导航初值和运行回退，直到新模型完成独立验证并被接受。"
                  "若修改ATYPE、Flip、机械结构或传感器安装，仍应按坐标/结构变更规则重新完成标定与验证。 ");
    }
    m_lblWorkflow->setText(text);
}

void SingleSphere14CalibrationWidget::onSolve()
{
    if (m_stage2SolveRunning) {
        QMessageBox::information(this, tr("阶段2正在求解"), tr("当前阶段2最终求解仍在运行，请查看进度区；如需停止请点击“取消阶段2求解”。"));
        return;
    }
    if (m_referenceTaught) {
        QString conventionError;
        if (!validateSessionConventions(conventionError)) {
            appendStage2SolveDetail(tr("拒绝启动：坐标约定检查失败：%1").arg(conventionError));
            QMessageBox::warning(this, tr("不能求解"), conventionError);
            return;
        }
    }

    auto fullObservations = stage2TrainingObservations();
    // Bootstrap uses one representative profile from every distinct Stage-2 pose.
    // This keeps the 9-state solve comparable in cost to Stage-1 while still using
    // the whole pose distribution.  Physical17 then uses a denser representative set.
    auto bootstrapObservations = reducedStage2Observations(1, 180);
    auto representativeObservations = reducedStage2Observations(3, 180);
    std::size_t fullPoints = 0, bootstrapPoints = 0, representativePoints = 0;
    for (const auto& o : fullObservations) fullPoints += o.points.size();
    for (const auto& o : bootstrapObservations) bootstrapPoints += o.points.size();
    for (const auto& o : representativeObservations) representativePoints += o.points.size();
    const int poseCount = stage2TrainingPoseCount();

    if (poseCount < 8 || fullObservations.size() < 8 || fullPoints < 80
        || bootstrapObservations.size() < 8 || representativeObservations.size() < 8) {
        const QString reason = tr("阶段2有效数据不足：姿态=%1，完整profiles=%2，完整点=%3，自举profiles=%4，代表profiles=%5。至少需要8个有差异的姿态和80个球面点。")
            .arg(poseCount)
            .arg(static_cast<qulonglong>(fullObservations.size()))
            .arg(static_cast<qulonglong>(fullPoints))
            .arg(static_cast<qulonglong>(bootstrapObservations.size()))
            .arg(static_cast<qulonglong>(representativeObservations.size()));
        appendStage2SolveDetail(tr("拒绝启动：%1").arg(reason));
        QMessageBox::warning(this, tr("阶段2数据不足"), reason);
        return;
    }

    if (!m_validationRecords.empty() || m_validationSummary.computed) {
        invalidateValidation(tr("17参数模型正在重新求解；已有hold-out数据仍保留，但旧验证结论已失效，求解完成后必须重新执行独立验证。"), false);
    }

    if (m_stage2SolveDetails) m_stage2SolveDetails->clear();
    if (m_stage2SolveProgress) m_stage2SolveProgress->setValue(0);
    if (m_lblStage2SolveProgress) m_lblStage2SolveProgress->setText(tr("阶段2求解 0%：正在从当前训练数据自举手眼初值..."));
    if (m_lblStage2SolveElapsed) m_lblStage2SolveElapsed->setText(tr("耗时：0.0 s"));
    m_stage2SolveRunning = true;
    m_stage2SolveStartMs = QDateTime::currentMSecsSinceEpoch();
    m_stage2SolveCancelFlag = std::make_shared<std::atomic_bool>(false);
    if (m_stage2SolveTimer) m_stage2SolveTimer->start();

    const std::size_t storedProfiles = m_calibrator.observations().size();
    const std::size_t filteredOut = storedProfiles > fullObservations.size() ? storedProfiles - fullObservations.size() : 0;
    appendStage2SolveDetail(tr(
        "最终求解已启动：阶段2独立姿态=%1；完整训练数据=%2 profiles / %3点；手眼自举集=%4 profiles / %5点；Physical17代表优化集=%6 profiles / %7点。"
        "V27.6.41先从每个Stage2姿态抽取1个代表profile，按名义机床模型求9状态 hand-eye 6DoF + 固定球心，再以该结果构造17参数初值。"
        "初步Physical17拟合后，会按完整姿态组统计球面残差，并执行渐进式trimmed least-squares：先用更严格的RMS/P95门限找坏姿态；如果整体RMS仍偏高，则主动测试最差尾部的一小批姿态。每次剔除后都重新自举hand-eye、重新求14个可辨识物理自由度，并且只有重拟合后的惩罚目标确实改善才接受删除。")
        .arg(poseCount)
        .arg(static_cast<qulonglong>(fullObservations.size()))
        .arg(static_cast<qulonglong>(fullPoints))
        .arg(static_cast<qulonglong>(bootstrapObservations.size()))
        .arg(static_cast<qulonglong>(bootstrapPoints))
        .arg(static_cast<qulonglong>(representativeObservations.size()))
        .arg(static_cast<qulonglong>(representativePoints)));
    if (m_hasBootstrap && m_bootstrapResult.success) {
        appendStage2SolveDetail(tr("已有阶段1结果仍保留用于姿态规划/导航，但本次最终17参数求解不会把它作为强制初值或平移规范锚点；锚点将由当前阶段2训练数据重新自举。"));
    }
    if (filteredOut > 0) {
        appendStage2SolveDetail(tr("检测到旧版混合CSV/缓存中额外的%1个profiles；已依据已加载阶段1姿态从最终训练集中自动剔除。")
            .arg(static_cast<qulonglong>(filteredOut)));
    }
    appendStage2SolveDetail(tr("优化策略：全姿态自举 -> 初步Physical17 -> 渐进式姿态trim -> 保留姿态重新自举 -> Physical17短全量重拟合 -> 最终全量精修。"
                               "V27.6.41不再只删除超过宽松MAD门限的极端异常点：每轮可测试约12%的最差姿态（通常3~6组），最多4轮，累计最多删除40%，至少保留18组。"
                               "删除必须同时满足：B/C正负覆盖和角度跨度保留、9状态bootstrap rank保持9/9、Physical17 rank保持14/17与17/20、condition不过门限，并且RMS+删除比例惩罚后的选择目标确实改善。原始CSV/采集数据不会被物理删除，只从本次训练子集中排除。"
                               "hand-eye Tx/Ty/Tz在每次Physical17过程中保持等式固定。"));
    m_lblSolveState->setText(tr("阶段2最终求解已启动：正在先从当前训练数据求手眼初值，再进行14自由度Physical17优化。"));
    updateCalibrationReadiness();

    Physical17SolverOptions options = solverOptionsFromUi();
    options.jointIterations = 60;
    // Real laser data reach a numerical plateau long before 1e-12 relative cost.
    // V27.6.41 keeps the stopping rule strict but realistic for finite-difference
    // Jacobians + Huber IRLS so a stable plateau is not mislabeled as failure.
    options.convergenceStep = std::max(options.convergenceStep, 1e-7);
    options.convergenceCost = std::max(options.convergenceCost, 1e-8);
    options.convergenceGradient = std::max(options.convergenceGradient, 1e-7);
    const auto cancelFlag = m_stage2SolveCancelFlag;
    QPointer<SingleSphere14CalibrationWidget> self(this);

    auto postProgress = [self](Physical17SolveProgress p) {
        if (!self) return;
        QMetaObject::invokeMethod(self, [self, p]() {
            if (self) self->updateStage2SolveProgressUi(p);
        }, Qt::QueuedConnection);
    };

    auto* watcher = new QFutureWatcher<Stage2SolveTaskResult>(this);
    connect(watcher, &QFutureWatcher<Stage2SolveTaskResult>::finished, this,
            [this, watcher, cancelFlag]() {
        Stage2SolveTaskResult task = watcher->result();
        watcher->deleteLater();
        SingleSphere17Result result = task.result;
        if (m_stage2SolveTimer) m_stage2SolveTimer->stop();
        const qint64 elapsedMs = QDateTime::currentMSecsSinceEpoch() - m_stage2SolveStartMs;
        const bool cancelRequested = cancelFlag && cancelFlag->load(std::memory_order_relaxed);
        m_stage2SolveRunning = false;
        m_stage2SolveCancelFlag.reset();

        if (task.bootstrapAccepted) {
            const auto& p = task.bootstrap.parameters;
            appendStage2SolveDetail(tr(
                "阶段2数据自举通过：rank=%1/9，RMS=%2 mm，max=%3 mm，cond=%4；"
                "hand-eye r=[%5,%6,%7] rad，t=[%8,%9,%10] mm；球心M=[%11,%12,%13] mm。"
                "这组三维平移即本次Physical17的translation-gauge anchor。")
                .arg(task.bootstrap.stateRank)
                .arg(task.bootstrap.residualRmsMm,0,'g',8)
                .arg(task.bootstrap.residualMaxAbsMm,0,'g',8)
                .arg(task.bootstrap.conditionNumber,0,'g',8)
                .arg(p[Physical17Parameters::HandEyeRx],0,'g',10)
                .arg(p[Physical17Parameters::HandEyeRy],0,'g',10)
                .arg(p[Physical17Parameters::HandEyeRz],0,'g',10)
                .arg(p[Physical17Parameters::HandEyeTx],0,'g',12)
                .arg(p[Physical17Parameters::HandEyeTy],0,'g',12)
                .arg(p[Physical17Parameters::HandEyeTz],0,'g',12)
                .arg(task.bootstrap.sphereCenterMachineMm.x,0,'g',12)
                .arg(task.bootstrap.sphereCenterMachineMm.y,0,'g',12)
                .arg(task.bootstrap.sphereCenterMachineMm.z,0,'g',12));
            if (task.bootstrap.residualRmsMm > 0.50) {
                appendStage2SolveDetail(tr("提示：名义机床自举RMS超过0.50 mm。这不直接判定最终失败，因为11项机床几何误差尚未进入自举模型；最终仍以Physical17全量RMS/rank/condition和独立hold-out为准。"));
            }
        } else if (!task.bootstrap.message.empty()) {
            appendStage2SolveDetail(tr("阶段2数据自举未通过：%1；rank=%2/9，RMS=%3，cond=%4。")
                .arg(QString::fromStdString(task.bootstrap.message))
                .arg(task.bootstrap.stateRank)
                .arg(task.bootstrap.residualRmsMm,0,'g',8)
                .arg(task.bootstrap.conditionNumber,0,'g',8));
        }

        for (int round = 0; round < static_cast<int>(task.screeningReports.size()); ++round) {
            const auto& report = task.screeningReports[static_cast<std::size_t>(round)];
            if (!report.valid) {
                appendStage2SolveDetail(tr("姿态筛选第%1轮无法执行：%2")
                    .arg(round+1).arg(QString::fromStdString(report.message)));
                continue;
            }
            appendStage2SolveDetail(tr(
                "姿态筛选第%1轮：输入=%2组，标记=%3组，保留=%4组；全体RMS=%5 mm；"
                "RMS median=%6 mm，MAD=%7 mm，门限=%8 mm；P95 median=%9 mm，门限=%10 mm；"
                "统计候选=%11，渐进补充=%12；重拟合RMS=%13 -> %14 mm；选择目标=%15 -> %16；%17。")
                .arg(round+1)
                .arg(static_cast<qulonglong>(report.inputPoseCount))
                .arg(static_cast<qulonglong>(report.rejectedPoseCount))
                .arg(static_cast<qulonglong>(report.retainedPoseCount))
                .arg(report.aggregateRmsMm,0,'g',7)
                .arg(report.medianRmsMm,0,'g',7)
                .arg(report.madRmsMm,0,'g',7)
                .arg(report.rmsThresholdMm,0,'g',7)
                .arg(report.medianP95AbsMm,0,'g',7)
                .arg(report.p95ThresholdMm,0,'g',7)
                .arg(static_cast<qulonglong>(report.statisticalCandidateCount))
                .arg(static_cast<qulonglong>(report.progressiveCandidateCount))
                .arg(report.refitRmsBeforeMm,0,'g',7)
                .arg(report.refitRmsAfterMm,0,'g',7)
                .arg(report.selectionObjectiveBefore,0,'g',7)
                .arg(report.selectionObjectiveAfter,0,'g',7)
                .arg(report.applied ? tr("已接受该批删除并重新求解") : tr("未接受/无需继续删除")));
            if (report.applied) {
                std::vector<Physical17PoseResidualSummary> rejected;
                for (const auto& pose : report.poses) if (pose.rejected) rejected.push_back(pose);
                std::sort(rejected.begin(), rejected.end(), [](const auto& a, const auto& b) {
                    return a.rejectionScore > b.rejectionScore;
                });
                for (const auto& pose : rejected) {
                    appendStage2SolveDetail(tr(
                        "  剔除姿态：%1；profiles=%2，points=%3，RMS=%4 mm，P95=%5 mm，max=%6 mm，score=%7")
                        .arg(poseText(pose.pose))
                        .arg(static_cast<qulonglong>(pose.profileCount))
                        .arg(static_cast<qulonglong>(pose.pointCount))
                        .arg(pose.residualRmsMm,0,'g',7)
                        .arg(pose.residualP95AbsMm,0,'g',7)
                        .arg(pose.residualMaxAbsMm,0,'g',7)
                        .arg(pose.rejectionScore,0,'g',5));
                }
            }
        }

        const bool rankOk = result.observability.targetRank >= result.observability.expectedTargetRank
            && result.observability.fullStateRank >= result.observability.expectedFullStateRank;
        const bool qualityOk = result.success && rankOk
            && std::isfinite(result.residualRmsMm) && result.residualRmsMm <= 0.050
            && std::isfinite(result.observability.conditionNumber) && result.observability.conditionNumber <= 1.0e9;
        if (!qualityOk) {
            result.success = false;
            if (result.message.empty())
                result.message = "GUI acceptance failed: full Stage-2 RMS/rank/condition threshold.";
        }
        m_lastResult = result;
        m_hasResult = m_lastResult.success;
        if (m_hasResult) m_modelFeedbackMask = m_referenceFeedbackMask;

        if (m_lblStage2SolveElapsed)
            m_lblStage2SolveElapsed->setText(tr("耗时：%1 s").arg(elapsedMs / 1000.0, 0, 'f', 2));
        if (m_stage2SolveProgress) m_stage2SolveProgress->setValue(100);

        if (m_hasResult) {
            m_currentEstimate = m_lastResult.parameters;
            m_sphereCenterGuessMachine = m_lastResult.sphereCenterMachineMm;
            m_refinementSolved = true;
            m_btnGenerate->setText(tr("用当前17参数结果重新生成 D-optimal 姿态"));
            if (m_lblStage2SolveProgress) m_lblStage2SolveProgress->setText(tr("阶段2求解 100%：训练数据自举 + Physical17求解通过，等待独立hold-out验证。"));
            appendStage2SolveDetail(tr(
                "求解成功：完整阶段2 RMS=%1 mm，max=%2 mm；物理rank=%3/17（至少14），完整rank=%4/20（至少17）；condition=%5；总耗时=%6 s。")
                .arg(m_lastResult.residualRmsMm,0,'g',8)
                .arg(m_lastResult.residualMaxAbsMm,0,'g',8)
                .arg(m_lastResult.observability.targetRank)
                .arg(m_lastResult.observability.fullStateRank)
                .arg(m_lastResult.observability.conditionNumber,0,'g',8)
                .arg(elapsedMs/1000.0,0,'f',2));
            appendStage2SolveDetail(tr("注意：这只是训练数据通过。自举与17参数优化使用的是同一训练集，因此它不是独立验证；下一步仍必须执行hold-out验证。"));
        } else {
            if (m_lblStage2SolveProgress) m_lblStage2SolveProgress->setText(cancelRequested
                ? tr("阶段2求解已取消。") : tr("阶段2求解失败；请查看下方详细错误。"));
            appendStage2SolveDetail(tr(
                "%1：message=%2；RMS=%3 mm；max=%4 mm；物理rank=%5/%6；完整rank=%7/%8；condition=%9；耗时=%10 s。")
                .arg(cancelRequested ? tr("求解取消") : tr("求解失败"))
                .arg(QString::fromStdString(m_lastResult.message))
                .arg(m_lastResult.residualRmsMm,0,'g',8)
                .arg(m_lastResult.residualMaxAbsMm,0,'g',8)
                .arg(m_lastResult.observability.targetRank)
                .arg(m_lastResult.observability.expectedTargetRank)
                .arg(m_lastResult.observability.fullStateRank)
                .arg(m_lastResult.observability.expectedFullStateRank)
                .arg(m_lastResult.observability.conditionNumber,0,'g',8)
                .arg(elapsedMs/1000.0,0,'f',2));
        }

        updateResultTable(&m_lastResult);
        updateValidationSummary();
        updateCalibrationReadiness();
        updateWorkflowGuide();

        const QString summary = tr("RMS=%1 mm\nmax=%2 mm\n物理rank=%3/17（要求≥14）\n完整rank=%4/20（要求≥17）\ncondition=%5\n耗时=%6 s\n\n%7")
            .arg(m_lastResult.residualRmsMm,0,'g',8)
            .arg(m_lastResult.residualMaxAbsMm,0,'g',8)
            .arg(m_lastResult.observability.targetRank)
            .arg(m_lastResult.observability.fullStateRank)
            .arg(m_lastResult.observability.conditionNumber,0,'g',8)
            .arg(elapsedMs/1000.0,0,'f',2)
            .arg(QString::fromStdString(m_lastResult.message));
        if (m_hasResult) {
            QMessageBox::information(this, tr("阶段2最终求解完成"),
                tr("当前阶段2数据已先完成手眼+球心自举，再完成Physical17训练求解。\n\n%1\n\n请继续做独立hold-out验证。 ").arg(summary));
        } else if (!cancelRequested) {
            QMessageBox::warning(this, tr("阶段2最终求解失败"), summary);
        }
    });

    watcher->setFuture(QtConcurrent::run([
        full = std::move(fullObservations),
        bootstrapData = std::move(bootstrapObservations),
        representative = std::move(representativeObservations),
        options, cancelFlag, postProgress]() mutable -> Stage2SolveTaskResult {
        Stage2SolveTaskResult task;
        try {
            const auto cancelled = [&]() {
                return cancelFlag && cancelFlag->load(std::memory_order_relaxed);
            };
            const auto bootstrapAccepted = [](const NominalHandEyeSphereResult& b) {
                return b.success && b.stateRank == b.expectedStateRank
                    && std::isfinite(b.residualRmsMm)
                    && std::isfinite(b.conditionNumber)
                    && b.conditionNumber <= 1.0e12;
            };
            const auto solveBootstrap = [&](const std::vector<SphereProfileObservation>& data) {
                SingleSphere17Calibrator calibrator(options);
                for (const auto& o : data) calibrator.addObservation(o);
                return calibrator.solveNominalHandEyeBootstrap();
            };
            const auto solveRepresentative = [&](const std::vector<SphereProfileObservation>& data,
                                                  const NominalHandEyeSphereResult& bootstrap,
                                                  int basePercent, int spanPercent,
                                                  const std::string& label) {
                SingleSphere17Calibrator calibrator(options);
                for (const auto& o : data) calibrator.addObservation(o);
                auto mappedProgress = [postProgress, basePercent, spanPercent, label](
                    const Physical17SolveProgress& in) {
                    Physical17SolveProgress p = in;
                    p.percent = basePercent + std::max(0, std::min(spanPercent,
                        static_cast<int>(std::llround((static_cast<double>(spanPercent) * in.percent) / 100.0))));
                    p.message = label + in.message;
                    postProgress(p);
                };
                return calibrator.solve(bootstrap.parameters, &bootstrap.sphereCenterMachineMm,
                                        mappedProgress, cancelFlag.get());
            };

            Physical17SolveProgress bootstrapStart;
            bootstrapStart.phase = Physical17SolveProgress::Phase::Preparing;
            bootstrapStart.percent = 1;
            bootstrapStart.message = "Stage-2 data-driven bootstrap: solving nominal-machine hand-eye 6DoF + sphere center.";
            postProgress(bootstrapStart);
            if (cancelled()) {
                task.result.success = false;
                task.result.message = "Stage-2 calibration cancelled before hand-eye bootstrap.";
                return task;
            }

            task.bootstrap = solveBootstrap(bootstrapData);
            task.bootstrapAccepted = bootstrapAccepted(task.bootstrap);
            if (!task.bootstrapAccepted) {
                task.result.success = false;
                task.result.message = std::string("Stage-2 training-data hand-eye bootstrap failed: ")
                    + task.bootstrap.message;
                return task;
            }
            if (cancelled()) {
                task.result.success = false;
                task.result.message = "Stage-2 calibration cancelled after hand-eye bootstrap.";
                return task;
            }

            Physical17SolveProgress bootstrapDone;
            bootstrapDone.phase = Physical17SolveProgress::Phase::Preparing;
            bootstrapDone.percent = 5;
            {
                std::ostringstream msg;
                msg << "Stage-2 bootstrap accepted: rank=" << task.bootstrap.stateRank
                    << "/9, RMS=" << task.bootstrap.residualRmsMm
                    << " mm; hand-eye translation fixed as gauge anchor ["
                    << task.bootstrap.parameters[Physical17Parameters::HandEyeTx] << ", "
                    << task.bootstrap.parameters[Physical17Parameters::HandEyeTy] << ", "
                    << task.bootstrap.parameters[Physical17Parameters::HandEyeTz] << "] mm.";
                bootstrapDone.message = msg.str();
            }
            postProgress(bootstrapDone);

            SingleSphere17Result result = solveRepresentative(
                representative, task.bootstrap, 5, 30, "Initial representative Physical17: ");
            if (cancelled()) {
                result.success = false;
                if (result.message.empty()) result.message = "Stage-2 calibration cancelled by operator.";
                task.result = result;
                return task;
            }
            if (!result.success) {
                task.result = result;
                return task;
            }

            // V27.6.41: progressive rank-aware trimmed least-squares at the
            // complete XYZBC-pose level.  The previous version used a single
            // permissive MAD gate; several moderately bad poses could therefore
            // mask one another and only the worst pose was removed.  Here we
            // (1) use stricter robust/relative gates, (2) force a small worst-tail
            // batch while training RMS remains above target, (3) re-bootstrap and
            // re-fit that subset, and (4) accept the trim only when a penalized
            // fit-quality objective improves while rank/conditioning are preserved.
            Physical17PoseScreeningOptions screeningOptions;
            screeningOptions.minRetainedPoseCount = 18;
            screeningOptions.maxRejectedFraction = 0.15; // per round
            screeningOptions.madSigma = 1.75;
            screeningOptions.relativeRmsFactor = 1.30;
            screeningOptions.relativeP95Factor = 1.30;
            screeningOptions.absoluteRmsFloorMm = 0.050;
            screeningOptions.absoluteP95FloorMm = 0.080;
            screeningOptions.enableProgressiveTrim = true;
            screeningOptions.targetTrainingRmsMm = 0.045;
            screeningOptions.progressiveTrimFraction = 0.12;
            screeningOptions.minProgressiveTrimCount = 3;
            screeningOptions.maxProgressiveTrimCount = 6;
            screeningOptions.minProgressiveSpreadMm = 0.010;
            screeningOptions.progressiveTailFactor = 1.02;
            screeningOptions.minRotarySpanFraction = 0.65;

            constexpr int kMaxScreeningRounds = 4;
            constexpr double kMaxCumulativeRejectedFraction = 0.40;
            constexpr double kSelectionPenaltyMm = 0.020;
            constexpr double kMinObjectiveImprovementMm = 0.00025;
            constexpr double kMaxConditionNumber = 1.0e9;

            std::vector<Physical17PoseResidualSummary> rejectedPoses;
            std::size_t inputPoseCount = 0;
            int appliedRounds = 0;
            double lastRmsThreshold = 0.0;
            double lastP95Threshold = 0.0;

            auto currentValidation = SingleSphere17Calibrator::validateModel(
                full, result.parameters, result.sphereCenterMachineMm, options.sphereDiameterMm);
            double currentTrainingRms = currentValidation.valid
                ? currentValidation.residualRmsMm : result.residualRmsMm;
            double currentObjective = currentTrainingRms;

            for (int round = 0; round < kMaxScreeningRounds; ++round) {
                if (cancelled()) {
                    result.success = false;
                    result.message = "Stage-2 calibration cancelled during progressive pose screening.";
                    task.result = result;
                    return task;
                }

                auto roundOptions = screeningOptions;
                auto report = SingleSphere17Calibrator::screenPoseOutliers(
                    full, result.parameters, result.sphereCenterMachineMm,
                    options.sphereDiameterMm, roundOptions);
                if (round == 0) inputPoseCount = report.inputPoseCount;

                // Enforce a cumulative 40% cap in addition to the per-round 15% cap.
                const std::size_t cumulativeLimit = static_cast<std::size_t>(std::floor(
                    kMaxCumulativeRejectedFraction * static_cast<double>(inputPoseCount) + 1e-12));
                if (rejectedPoses.size() >= cumulativeLimit) {
                    report = limitRejectedPoseCount(report, 0);
                } else {
                    const std::size_t remainingRejectBudget = cumulativeLimit - rejectedPoses.size();
                    if (report.rejectedPoseCount > remainingRejectBudget)
                        report = limitRejectedPoseCount(report, remainingRejectBudget);
                }
                lastRmsThreshold = report.rmsThresholdMm;
                lastP95Threshold = report.p95ThresholdMm;
                report.refitRmsBeforeMm = currentTrainingRms;
                report.selectionObjectiveBefore = currentObjective;

                if (!report.valid || report.rejectedPoseCount == 0) {
                    report.refitRmsAfterMm = currentTrainingRms;
                    report.selectionObjectiveAfter = currentObjective;
                    task.screeningReports.push_back(report);
                    break;
                }

                // Try the full proposed batch first.  If the penalized objective
                // says it is too aggressive, retry half the batch, then one pose.
                std::vector<std::size_t> trialCounts;
                trialCounts.push_back(report.rejectedPoseCount);
                if (report.rejectedPoseCount > 3)
                    trialCounts.push_back(std::max<std::size_t>(2, (2*report.rejectedPoseCount)/3));
                if (report.rejectedPoseCount > 2)
                    trialCounts.push_back(std::max<std::size_t>(2, report.rejectedPoseCount/2));
                if (report.rejectedPoseCount > 1) trialCounts.push_back(1);
                std::sort(trialCounts.begin(), trialCounts.end(), std::greater<std::size_t>());
                trialCounts.erase(std::unique(trialCounts.begin(), trialCounts.end()), trialCounts.end());

                bool accepted = false;
                Physical17PoseScreeningReport acceptedReport = report;
                std::vector<SphereProfileObservation> acceptedFull;
                std::vector<SphereProfileObservation> acceptedBootstrapData;
                std::vector<SphereProfileObservation> acceptedRepresentative;
                NominalHandEyeSphereResult acceptedBootstrap;
                SingleSphere17Result acceptedResult;
                double acceptedRms = currentTrainingRms;
                double acceptedObjective = currentObjective;

                for (std::size_t trialCount : trialCounts) {
                    auto trialReport = limitRejectedPoseCount(report, trialCount);
                    auto proposedFull = retainScreenedStage2Poses(full, trialReport,
                        roundOptions.poseLinearToleranceMm, roundOptions.poseAngularToleranceDeg);
                    auto proposedBootstrapData = retainScreenedStage2Poses(bootstrapData, trialReport,
                        roundOptions.poseLinearToleranceMm, roundOptions.poseAngularToleranceDeg);
                    auto proposedRepresentative = retainScreenedStage2Poses(representative, trialReport,
                        roundOptions.poseLinearToleranceMm, roundOptions.poseAngularToleranceDeg);

                    if (proposedFull.size() < 8 || proposedBootstrapData.size() < 8 ||
                        proposedRepresentative.size() < 8) continue;

                    const NominalHandEyeSphereResult filteredBootstrap = solveBootstrap(proposedBootstrapData);
                    if (!bootstrapAccepted(filteredBootstrap)) continue;
                    if (cancelled()) {
                        result.success = false;
                        result.message = "Stage-2 calibration cancelled while re-bootstrapping retained poses.";
                        task.result = result;
                        return task;
                    }

                    const int base = 30 + round * 10;
                    SingleSphere17Result filteredResult = solveRepresentative(
                        proposedRepresentative, filteredBootstrap, base, 6,
                        std::string("Progressive screened representative round ") +
                            std::to_string(round+1) + ": ");
                    const bool representativeRankOk = filteredResult.success
                        && filteredResult.observability.targetRank >= filteredResult.observability.expectedTargetRank
                        && filteredResult.observability.fullStateRank >= filteredResult.observability.expectedFullStateRank;
                    if (!representativeRankOk) continue;

                    // Short full-retained-data refit makes the accept/reject decision
                    // on the same data density as the eventual final solve, rather
                    // than judging a subset from the sparse representative profiles.
                    Physical17SolverOptions trialOptions = options;
                    trialOptions.multiStartCount = 1;
                    trialOptions.alternatingIterations = 0;
                    trialOptions.jointIterations = 25;
                    SingleSphere17Calibrator trialCalibrator(trialOptions);
                    for (const auto& o : proposedFull) trialCalibrator.addObservation(o);
                    SingleSphere17Result trialResult = trialCalibrator.solve(
                        filteredResult.parameters, &filteredResult.sphereCenterMachineMm,
                        Physical17ProgressCallback(), cancelFlag.get());
                    if (cancelled()) {
                        result.success = false;
                        result.message = "Stage-2 calibration cancelled during screening trial refit.";
                        task.result = result;
                        return task;
                    }
                    const bool trialRankOk = trialResult.success
                        && trialResult.observability.targetRank >= trialResult.observability.expectedTargetRank
                        && trialResult.observability.fullStateRank >= trialResult.observability.expectedFullStateRank;
                    const double relativeConditionLimit = std::min(
                        kMaxConditionNumber,
                        std::max(1.0e6, 5.0*std::max(1.0, result.observability.conditionNumber)));
                    if (!trialRankOk || !std::isfinite(trialResult.residualRmsMm) ||
                        !std::isfinite(trialResult.observability.conditionNumber) ||
                        trialResult.observability.conditionNumber > relativeConditionLimit) continue;

                    const std::size_t cumulativeRejected = rejectedPoses.size() + trialCount;
                    const double rejectedFraction = inputPoseCount > 0
                        ? static_cast<double>(cumulativeRejected)/static_cast<double>(inputPoseCount) : 0.0;
                    const double objective = trialResult.residualRmsMm +
                        kSelectionPenaltyMm * rejectedFraction;
                    const bool objectiveImproved = objective + kMinObjectiveImprovementMm < currentObjective;
                    const bool hitTarget = trialResult.residualRmsMm <= roundOptions.targetTrainingRmsMm
                        && trialResult.residualRmsMm + 1e-9 < currentTrainingRms;
                    if (!objectiveImproved && !hitTarget) continue;

                    trialReport.applied = true;
                    trialReport.refitRmsBeforeMm = currentTrainingRms;
                    trialReport.refitRmsAfterMm = trialResult.residualRmsMm;
                    trialReport.selectionObjectiveBefore = currentObjective;
                    trialReport.selectionObjectiveAfter = objective;
                    {
                        std::ostringstream msg;
                        msg << " Accepted progressive trim " << trialCount
                            << " pose(s): retained-data RMS " << currentTrainingRms
                            << " -> " << trialResult.residualRmsMm
                            << " mm; penalized objective " << currentObjective
                            << " -> " << objective << ".";
                        trialReport.message += msg.str();
                    }

                    accepted = true;
                    acceptedReport = std::move(trialReport);
                    acceptedFull = std::move(proposedFull);
                    acceptedBootstrapData = std::move(proposedBootstrapData);
                    acceptedRepresentative = std::move(proposedRepresentative);
                    acceptedBootstrap = filteredBootstrap;
                    acceptedResult = std::move(trialResult);
                    acceptedRms = acceptedResult.residualRmsMm;
                    acceptedObjective = objective;
                    break;
                }

                if (!accepted) {
                    report.applied = false;
                    report.refitRmsAfterMm = currentTrainingRms;
                    report.selectionObjectiveAfter = currentObjective;
                    report.message += " Progressive trim candidates did not improve the penalized retained-data fit; screening stopped.";
                    task.screeningReports.push_back(report);
                    break;
                }

                Physical17SolveProgress screenProgress;
                screenProgress.phase = Physical17SolveProgress::Phase::Preparing;
                screenProgress.percent = 36 + round * 10;
                {
                    std::ostringstream msg;
                    msg << "Progressive pose screening round " << (round+1)
                        << ": accepted " << acceptedReport.rejectedPoseCount
                        << "/" << acceptedReport.inputPoseCount
                        << " poses; retained-data RMS " << acceptedReport.refitRmsBeforeMm
                        << " -> " << acceptedReport.refitRmsAfterMm
                        << " mm; objective " << acceptedReport.selectionObjectiveBefore
                        << " -> " << acceptedReport.selectionObjectiveAfter << ".";
                    screenProgress.message = msg.str();
                }
                postProgress(screenProgress);

                task.screeningReports.push_back(acceptedReport);
                for (const auto& pose : acceptedReport.poses)
                    if (pose.rejected) rejectedPoses.push_back(pose);
                ++appliedRounds;
                full = std::move(acceptedFull);
                bootstrapData = std::move(acceptedBootstrapData);
                representative = std::move(acceptedRepresentative);
                task.bootstrap = acceptedBootstrap;
                task.bootstrapAccepted = true;
                result = std::move(acceptedResult);
                currentTrainingRms = acceptedRms;
                currentObjective = acceptedObjective;

                // Once a margin below the normal 0.050 mm acceptance gate is
                // reached, do not keep deleting good data merely to lower the
                // training residual further.
                if (currentTrainingRms <= screeningOptions.targetTrainingRmsMm) break;
            }

            // Final refinement uses only retained pose groups.  Excluded raw data
            // remain in the session/CSV and are recorded in result provenance.
            Physical17SolverOptions fullOptions = options;
            fullOptions.multiStartCount = 1;
            fullOptions.alternatingIterations = 0;
            fullOptions.jointIterations = 60;
            SingleSphere17Calibrator fullCalibrator(fullOptions);
            for (const auto& o : full) fullCalibrator.addObservation(o);
            auto fullProgress = [postProgress](const Physical17SolveProgress& in) {
                Physical17SolveProgress p = in;
                p.percent = 70 + std::max(0, std::min(30,
                    static_cast<int>(std::llround(0.30 * in.percent))));
                p.message = std::string("Screened full-data Physical17 refinement: ") + in.message;
                postProgress(p);
            };
            result = fullCalibrator.solve(result.parameters, &result.sphereCenterMachineMm,
                                          fullProgress, cancelFlag.get());
            if (cancelled()) {
                result.success = false;
                result.message = "Stage-2 calibration cancelled during screened full-data refinement.";
                task.result = result;
                return task;
            }

            result.hasInitializationBootstrap = true;
            result.initializationSource = appliedRounds > 0
                ? "stage2_pose_screened_training_data_nominal_machine_handeye"
                : "stage2_training_data_nominal_machine_handeye";
            result.initializationParameters = task.bootstrap.parameters;
            result.initializationSphereCenterMachineMm = task.bootstrap.sphereCenterMachineMm;
            result.initializationResidualRmsMm = task.bootstrap.residualRmsMm;
            result.initializationResidualMaxAbsMm = task.bootstrap.residualMaxAbsMm;
            result.initializationStateRank = task.bootstrap.stateRank;
            result.initializationConditionNumber = task.bootstrap.conditionNumber;
            result.initializationProfileCount = task.bootstrap.profileCount;
            result.initializationPointCount = task.bootstrap.pointCount;
            result.poseScreeningRounds = appliedRounds;
            result.trainingPoseCountBeforeScreening = inputPoseCount;
            result.trainingPoseCountAfterScreening = inputPoseCount >= rejectedPoses.size()
                ? inputPoseCount - rejectedPoses.size() : 0;
            result.poseScreeningRmsThresholdMm = lastRmsThreshold;
            result.poseScreeningP95ThresholdMm = lastP95Threshold;
            result.rejectedTrainingPoses = rejectedPoses;
            result.gaugeDescription =
                "V27.6.41 Stage-2 progressive trimmed screening: complete XYZBC pose groups are ranked by RMS/P95 residual quality; "
                "strict robust gates plus a small worst-tail trim are trial-refit, and deletions are accepted only when a penalized retained-data objective improves while preserving B/C excitation and rank, "
                "then the nominal-machine hand-eye+sphere bootstrap is recomputed from retained poses. Its hand-eye translation "
                "is retained as the 3-D gauge anchor. " + result.gaugeDescription;

            if (!result.optimizationConverged) {
                result.success = false;
                result.message = "Screened full-data joint refinement did not meet a stopping tolerance. " + result.message;
            }
            {
                std::ostringstream prefix;
                prefix << "Stage-2 self-bootstrap -> preliminary Physical17 -> progressive rank-aware trimmed screening ("
                       << rejectedPoses.size() << " poses excluded in " << appliedRounds
                       << " applied round(s)) -> retained-pose re-bootstrap -> retained full-data Physical17 refinement. ";
                result.message = prefix.str() + result.message;
            }
            task.result = result;
            return task;
        } catch (const std::exception& e) {
            task.result.success = false;
            task.result.message = std::string("Unhandled Stage-2 solver exception: ") + e.what();
            return task;
        } catch (...) {
            task.result.success = false;
            task.result.message = "Unhandled unknown exception in Stage-2 solver.";
            return task;
        }
    }));

}

void SingleSphere14CalibrationWidget::onCancelStage2Solve()
{
    if (!m_stage2SolveRunning || !m_stage2SolveCancelFlag) return;
    m_stage2SolveCancelFlag->store(true, std::memory_order_relaxed);
    if (m_btnCancelStage2Solve) m_btnCancelStage2Solve->setEnabled(false);
    if (m_lblStage2SolveProgress)
        m_lblStage2SolveProgress->setText(tr("正在取消阶段2求解：等待当前轮廓计算安全结束..."));
    appendStage2SolveDetail(tr("操作者请求取消阶段2求解。当前轮廓信息累加完成后会安全退出；已采集数据不会被删除。"));
}

void SingleSphere14CalibrationWidget::updateResultTable(const SingleSphere17Result* result)
{
    const auto& names=Physical17Parameters::names(); const auto& units=Physical17Parameters::units();
    for(int i=0;i<static_cast<int>(Physical17Parameters::Count);++i){
        m_resultTable->setItem(i,0,new QTableWidgetItem(QString::fromLatin1(names[static_cast<std::size_t>(i)])));
        m_resultTable->setItem(i,1,new QTableWidgetItem(result?QString::number(result->parameters[static_cast<std::size_t>(i)],'g',12):QStringLiteral("---")));
        m_resultTable->setItem(i,2,new QTableWidgetItem(QString::fromLatin1(units[static_cast<std::size_t>(i)])));
    }
    if(!result){
        m_lblSolveState->setText(tr("尚未标定"));
        return;
    }
    m_lblSolveState->setText(tr("%1；RMS=%2 mm；max=%3 mm；物理rank=%4/17（要求≥14）；完整rank=%5/20（要求≥17）；cond=%6")
        .arg(result->success ? (m_validationSummary.passed ? tr("标定+独立验证通过") : tr("训练求解通过，待独立验证")) : tr("标定失败"))
        .arg(result->residualRmsMm,0,'g',6).arg(result->residualMaxAbsMm,0,'g',6)
        .arg(result->observability.targetRank).arg(result->observability.fullStateRank)
        .arg(result->observability.conditionNumber,0,'g',6));
    QString text=tr("Translation gauge: %1\nMessage: %2\nSphere center(machine)=%3\nHand-eye translation anchor=[%4, %5, %6] mm\n")
        .arg(QString::fromStdString(result->gaugeDescription))
        .arg(QString::fromStdString(result->message))
        .arg(centerText(result->sphereCenterMachineMm))
        .arg(result->parameters[Physical17Parameters::HandEyeTx],0,'g',10)
        .arg(result->parameters[Physical17Parameters::HandEyeTy],0,'g',10)
        .arg(result->parameters[Physical17Parameters::HandEyeTz],0,'g',10);
    if (result->hasInitializationBootstrap) {
        const auto& p0=result->initializationParameters;
        text += tr("Initialization bootstrap: %1; rank=%2/9; RMS=%3 mm; cond=%4\n"
                   "  HE0 r=[%5,%6,%7] rad; t=[%8,%9,%10] mm; center0=%11\n")
            .arg(QString::fromStdString(result->initializationSource))
            .arg(result->initializationStateRank)
            .arg(result->initializationResidualRmsMm,0,'g',8)
            .arg(result->initializationConditionNumber,0,'g',8)
            .arg(p0[Physical17Parameters::HandEyeRx],0,'g',10)
            .arg(p0[Physical17Parameters::HandEyeRy],0,'g',10)
            .arg(p0[Physical17Parameters::HandEyeRz],0,'g',10)
            .arg(p0[Physical17Parameters::HandEyeTx],0,'g',12)
            .arg(p0[Physical17Parameters::HandEyeTy],0,'g',12)
            .arg(p0[Physical17Parameters::HandEyeTz],0,'g',12)
            .arg(centerText(result->initializationSphereCenterMachineMm));
    }
    if (result->trainingPoseCountBeforeScreening > 0) {
        text += tr("Pose screening: rounds=%1; poses %2 -> %3; rejected=%4; last RMS/P95 threshold=%5/%6 mm\n")
            .arg(result->poseScreeningRounds)
            .arg(static_cast<qulonglong>(result->trainingPoseCountBeforeScreening))
            .arg(static_cast<qulonglong>(result->trainingPoseCountAfterScreening))
            .arg(static_cast<qulonglong>(result->rejectedTrainingPoses.size()))
            .arg(result->poseScreeningRmsThresholdMm,0,'g',7)
            .arg(result->poseScreeningP95ThresholdMm,0,'g',7);
    }
    text += tr("Physical singular values (17):\n");
    for(double singular:result->observability.targetSingularValues)
        text += QString::number(singular,'g',8) + " ";
    text += tr("\nFull state singular values (20):\n");
    for(double singular:result->observability.fullStateSingularValues)
        text += QString::number(singular,'g',8) + " ";

    // 原右侧“详情/日志”文本框已移除。
    // 详细可观测性、规范自由度和奇异值统一进入底部系统日志。
    emit logMessage(tr("单球17参数求解详情：\n%1").arg(text));
}

void SingleSphere14CalibrationWidget::onSaveResult()
{
    if(!m_hasResult){QMessageBox::information(this,tr("无结果"),tr("请先完成标定。"));return;}
    if (!m_validationSummary.passed) {
        if (QMessageBox::warning(this, tr("结果尚未通过独立验证"),
            tr("当前17参数只通过了训练数据上的求解质量检查，还没有通过hold-out独立验证。\n"
               "可以保存为候选结果用于继续标定，但不应直接应用到最终扫描/RTCP。\n\n仍然保存候选结果？"),
            QMessageBox::Yes|QMessageBox::No, QMessageBox::No) != QMessageBox::Yes) return;
    }
    const QString fn=QFileDialog::getSaveFileName(this,tr("保存单球17参数结果"),QStringLiteral("single_sphere17_rank14_result.json"),tr("JSON (*.json)"));
    if(fn.isEmpty())return;
    std::ofstream out(fn.toStdString(),std::ios::binary); std::string err;
    if(!SingleSphere17Calibrator::writeResultJson(out,m_lastResult,err)) QMessageBox::warning(this,tr("保存失败"),QString::fromStdString(err));
    else {
        appendLog(tr("已保存标定结果：%1%2").arg(fn, m_validationSummary.passed ? tr("（独立验证已通过）") : tr("（候选/未验证）")));
        if (m_validationSummary.computed) {
            QString validationFile = fn;
            if (validationFile.endsWith(QStringLiteral(".json"), Qt::CaseInsensitive)) validationFile.chop(5);
            validationFile += QStringLiteral("_validation.json");
            QString validationError;
            if (!writeValidationJson(validationFile, validationError)) {
                appendLog(tr("验证报告自动保存失败：%1").arg(validationError));
            } else {
                appendLog(tr("已保存独立验证报告：%1").arg(validationFile));
            }
        }
    }
}

bool SingleSphere14CalibrationWidget::parsePhysical17JsonObject(
    const QJsonObject& root, SingleSphere17Result& result, QString& error) const
{
    if(root.value("model").toString()!=QStringLiteral("single_sphere_physical17_rank14")){
        error=tr("不是 single_sphere_physical17_rank14 结果文件");
        return false;
    }
    result=SingleSphere17Result{};
    result.optimizationConverged=root.value("optimization_converged").toBool(false);
    result.robustCost=root.value("robust_cost").toDouble(0.0);
    result.success=root.value("success").toBool(false);
    result.gaugeDescription=root.value("gauge_description").toString().toStdString();
    result.message=root.value("message").toString().toStdString();

    const auto params=root.value("parameters").toObject();
    const auto& names=Physical17Parameters::names();
    for(std::size_t i=0;i<Physical17Parameters::Count;++i){
        const auto po=params.value(QString::fromLatin1(names[i])).toObject();
        if(!po.contains("value") || !po.value("value").isDouble()){
            error=tr("缺少或无效参数 %1").arg(QString::fromLatin1(names[i]));
            return false;
        }
        const double value=po.value("value").toDouble();
        if(!std::isfinite(value)){
            error=tr("参数 %1 不是有限数").arg(QString::fromLatin1(names[i]));
            return false;
        }
        result.parameters[i]=value;
    }

    if(!finiteJsonVec3(root.value("sphere_center_machine_mm"),result.sphereCenterMachineMm)){
        error=tr("缺少有效的3D机床坐标球心，三个分量必须都是有限数值");
        return false;
    }

    // V27.6.41 optional provenance: the nominal-machine hand-eye+sphere seed
    // solved from the same Stage-2 training data. Older JSON files simply omit it.
    const auto init=root.value("initialization_bootstrap").toObject();
    if(!init.isEmpty()) {
        double rankValue=0.0, rms=0.0, maxAbs=0.0, cond=0.0, profiles=0.0, points=0.0;
        Vec3d center0;
        const auto handEye=init.value("hand_eye").toObject();
        const QStringList heKeys={QStringLiteral("rx_rad"),QStringLiteral("ry_rad"),QStringLiteral("rz_rad"),
                                  QStringLiteral("tx_mm"),QStringLiteral("ty_mm"),QStringLiteral("tz_mm")};
        std::array<double,6> he{};
        bool validInit=!handEye.isEmpty()
            && finiteJsonInteger(init,QStringLiteral("state_rank"),0.0,9.0,rankValue)
            && finiteJsonNumber(init,QStringLiteral("residual_rms_mm"),rms)
            && finiteJsonNumber(init,QStringLiteral("residual_max_abs_mm"),maxAbs)
            && finiteJsonNumber(init,QStringLiteral("condition_number"),cond)
            && finiteJsonInteger(init,QStringLiteral("profile_count"),0.0,9.0e15,profiles)
            && finiteJsonInteger(init,QStringLiteral("point_count"),0.0,9.0e15,points)
            && finiteJsonVec3(init.value("sphere_center_machine_mm"),center0);
        for(int i=0; validInit && i<heKeys.size(); ++i) {
            validInit=finiteJsonNumber(handEye,heKeys[i],he[static_cast<std::size_t>(i)]);
        }
        if(validInit) {
            result.hasInitializationBootstrap=true;
            result.initializationSource=init.value("source").toString().toStdString();
            result.initializationStateRank=static_cast<int>(rankValue);
            result.initializationResidualRmsMm=rms;
            result.initializationResidualMaxAbsMm=maxAbs;
            result.initializationConditionNumber=cond;
            result.initializationProfileCount=static_cast<std::size_t>(profiles);
            result.initializationPointCount=static_cast<std::size_t>(points);
            result.initializationSphereCenterMachineMm=center0;
            result.initializationParameters[Physical17Parameters::HandEyeRx]=he[0];
            result.initializationParameters[Physical17Parameters::HandEyeRy]=he[1];
            result.initializationParameters[Physical17Parameters::HandEyeRz]=he[2];
            result.initializationParameters[Physical17Parameters::HandEyeTx]=he[3];
            result.initializationParameters[Physical17Parameters::HandEyeTy]=he[4];
            result.initializationParameters[Physical17Parameters::HandEyeTz]=he[5];
        }
    }

    const auto screening=root.value("pose_screening").toObject();
    if(!screening.isEmpty()) {
        const double rounds=screening.value("rounds_applied").toDouble(0.0);
        const double inputPoses=screening.value("input_pose_count").toDouble(0.0);
        const double retainedPoses=screening.value("retained_pose_count").toDouble(0.0);
        result.poseScreeningRounds=std::max(0,static_cast<int>(std::llround(rounds)));
        result.trainingPoseCountBeforeScreening=static_cast<std::size_t>(std::max(0.0,inputPoses));
        result.trainingPoseCountAfterScreening=static_cast<std::size_t>(std::max(0.0,retainedPoses));
        result.poseScreeningRmsThresholdMm=screening.value("last_rms_threshold_mm").toDouble(0.0);
        result.poseScreeningP95ThresholdMm=screening.value("last_p95_threshold_mm").toDouble(0.0);
        const auto rejected=screening.value("rejected_poses").toArray();
        for(const auto& value:rejected) {
            const auto po=value.toObject();
            Physical17PoseResidualSummary summary;
            summary.pose.x=po.value("x_mm").toDouble();
            summary.pose.y=po.value("y_mm").toDouble();
            summary.pose.z=po.value("z_mm").toDouble();
            summary.pose.c=po.value("c_deg").toDouble();
            summary.pose.b=po.value("b_deg").toDouble();
            summary.profileCount=static_cast<std::size_t>(std::max(0.0,po.value("profile_count").toDouble()));
            summary.pointCount=static_cast<std::size_t>(std::max(0.0,po.value("point_count").toDouble()));
            summary.residualRmsMm=po.value("rms_mm").toDouble();
            summary.residualP95AbsMm=po.value("p95_abs_mm").toDouble();
            summary.residualMaxAbsMm=po.value("max_abs_mm").toDouble();
            summary.rejectionScore=po.value("score").toDouble();
            summary.rejected=true;
            if(std::isfinite(summary.pose.x) && std::isfinite(summary.pose.y) &&
               std::isfinite(summary.pose.z) && std::isfinite(summary.pose.c) &&
               std::isfinite(summary.pose.b) && std::isfinite(summary.residualRmsMm))
                result.rejectedTrainingPoses.push_back(summary);
        }
    }

    if(!root.value("success").isBool() || !root.value("success").toBool()){
        error=tr("17参数 JSON 未声明成功求解。");
        return false;
    }
    double residualRms=0.0, residualMax=0.0, profileCountValue=0.0, pointCountValue=0.0;
    constexpr double kMaxExactCounter=9.0e15;
    if(!finiteJsonNumber(root,QStringLiteral("residual_rms_mm"),residualRms)
        || !finiteJsonNumber(root,QStringLiteral("residual_max_abs_mm"),residualMax)
        || !finiteJsonInteger(root,QStringLiteral("profile_count"),1.0,kMaxExactCounter,profileCountValue)
        || !finiteJsonInteger(root,QStringLiteral("point_count"),1.0,kMaxExactCounter,pointCountValue)
        || residualRms<0.0 || residualMax<0.0){
        error=tr("17参数 JSON 的RMS/max/profile_count/point_count缺失、类型错误或超出安全范围。");
        return false;
    }
    result.residualRmsMm=residualRms;
    result.residualMaxAbsMm=residualMax;
    result.profileCount=static_cast<std::size_t>(profileCountValue);
    result.pointCount=static_cast<std::size_t>(pointCountValue);

    const auto oo=root.value("observability").toObject();
    if(oo.isEmpty()){
        error=tr("17参数 JSON 缺少 observability。");
        return false;
    }
    double targetRankValue=0.0, fullRankValue=0.0, conditionNumber=0.0;
    if(!finiteJsonInteger(oo,QStringLiteral("rank_after_sphere_center_elimination"),0.0,100.0,targetRankValue)
        || !finiteJsonInteger(oo,QStringLiteral("full_state_rank"),0.0,100.0,fullRankValue)
        || !finiteJsonNumber(oo,QStringLiteral("nonzero_condition_number"),conditionNumber)
        || conditionNumber<=0.0){
        error=tr("17参数 JSON 的rank/condition字段缺失、类型错误或无效。");
        return false;
    }
    result.observability.targetRank=static_cast<int>(targetRankValue);
    // The Physical17 model has fixed mathematical rank requirements. Do not trust
    // editable "expected_*" values from an imported JSON to weaken this gate.
    result.observability.expectedTargetRank=14;
    result.observability.fullStateRank=static_cast<int>(fullRankValue);
    result.observability.expectedFullStateRank=17;
    result.observability.conditionNumber=conditionNumber;
    const auto sv=oo.value("physical_singular_values").toArray();
    for(int i=0;i<sv.size()&&i<17;++i)
        result.observability.targetSingularValues[static_cast<std::size_t>(i)]=sv[i].toDouble();
    const auto fsv=oo.value("full_state_singular_values").toArray();
    for(int i=0;i<fsv.size()&&i<20;++i)
        result.observability.fullStateSingularValues[static_cast<std::size_t>(i)]=fsv[i].toDouble();

    const bool finiteQuality=std::isfinite(result.residualRmsMm)
        && std::isfinite(result.residualMaxAbsMm)
        && std::isfinite(result.observability.conditionNumber);
    result.success=result.success && finiteQuality
        && result.profileCount>0 && result.pointCount>0
        && result.residualRmsMm<=0.050
        && result.observability.conditionNumber<=1.0e9
        && result.observability.targetRank>=result.observability.expectedTargetRank
        && result.observability.fullStateRank>=result.observability.expectedFullStateRank;
    if(!result.success){
        error=tr("17参数结果未通过success/rank/RMS≤0.050mm/condition≤1e9/有限数完整性检查");
        return false;
    }
    error.clear();
    return true;
}

bool SingleSphere14CalibrationWidget::loadPhysical17Json(
    const QString& fileName, SingleSphere17Result& result, QString& error) const
{
    QFile f(fileName);
    if(!f.open(QIODevice::ReadOnly)){error=f.errorString();return false;}
    QJsonParseError pe;
    const auto doc=QJsonDocument::fromJson(f.readAll(),&pe);
    if(pe.error!=QJsonParseError::NoError||!doc.isObject()){
        error=tr("JSON解析失败：%1").arg(pe.errorString());
        return false;
    }
    return parsePhysical17JsonObject(doc.object(), result, error);
}

bool SingleSphere14CalibrationWidget::loadValidationJson(
    const QString& fileName, const SingleSphere17Result& result,
    ValidationSummary& summary, QString& error, bool requireStrongModelMatch) const
{
    summary=ValidationSummary{};
    QFile f(fileName);
    if(!f.open(QIODevice::ReadOnly)){error=f.errorString();return false;}
    QJsonParseError pe;
    const auto doc=QJsonDocument::fromJson(f.readAll(),&pe);
    if(pe.error!=QJsonParseError::NoError || !doc.isObject()){
        error=tr("验证JSON解析失败：%1").arg(pe.errorString());
        return false;
    }
    const auto root=doc.object();
    if(root.value("model").toString()!=QStringLiteral("single_sphere_physical17_holdout_validation")){
        error=tr("不是 single_sphere_physical17_holdout_validation 验证报告");
        return false;
    }
    QString badValidationField;
    if(!validationNumericFieldsAreFinite(root,badValidationField)
        || !root.value("passed").isBool()
        || !root.value("b_positive_negative_coverage").isBool()
        || !root.value("c_positive_negative_coverage").isBool()){
        error=tr("验证报告字段缺失/类型错误/非有限数：%1").arg(
            badValidationField.isEmpty()?QStringLiteral("boolean acceptance field"):badValidationField);
        return false;
    }

    double poseCountValue=0.0, profileCountValue=0.0, pointCountValue=0.0, requiredPoseValue=0.0;
    constexpr double kMaxValidationCounter=9.0e15;
    if(!finiteJsonInteger(root,QStringLiteral("pose_count"),0.0,
                          static_cast<double>(std::numeric_limits<int>::max()),poseCountValue)
        || !finiteJsonInteger(root,QStringLiteral("profile_count"),1.0,kMaxValidationCounter,profileCountValue)
        || !finiteJsonInteger(root,QStringLiteral("point_count"),1.0,kMaxValidationCounter,pointCountValue)
        || !finiteJsonInteger(root,QStringLiteral("required_pose_count"),6.0,100.0,requiredPoseValue)){
        error=tr("验证报告的姿态/轮廓/点数计数字段不是安全范围内的整数。");
        return false;
    }
    summary.computed=true;
    summary.passed=root.value("passed").toBool(false);
    summary.poseCount=static_cast<int>(poseCountValue);
    summary.profileCount=static_cast<std::size_t>(profileCountValue);
    summary.pointCount=static_cast<std::size_t>(pointCountValue);
    summary.surfaceMeanMm=root.value("surface_mean_mm").toDouble();
    summary.surfaceMaeMm=root.value("surface_mae_mm").toDouble();
    summary.surfaceRmsMm=root.value("surface_rms_mm").toDouble();
    summary.surfaceMaxMm=root.value("surface_max_abs_mm").toDouble();
    summary.centerXzRmsMm=root.value("center_xz_rms_mm").toDouble();
    summary.center3dRmsMm=root.value("center_3d_rms_mm").toDouble();
    summary.centerMaxMm=root.value("center_max_mm").toDouble();
    if(!finiteJsonVec3(root.value("center_bias_sensor_mm"),summary.centerBiasSensorMm)){
        error=tr("验证报告缺少有效的3D球心bias");
        return false;
    }
    summary.bCoverageOk=root.value("b_positive_negative_coverage").toBool(false);
    summary.cCoverageOk=root.value("c_positive_negative_coverage").toBool(false);

    const double trainRms=root.value("training_surface_rms_mm").toDouble(std::numeric_limits<double>::quiet_NaN());
    const int requiredPoses=static_cast<int>(requiredPoseValue);
    const double rmsLimit=root.value("threshold_surface_rms_mm").toDouble(std::numeric_limits<double>::quiet_NaN());
    const double maxLimit=root.value("threshold_surface_max_mm").toDouble(std::numeric_limits<double>::quiet_NaN());
    const double centerLimit=root.value("threshold_center_xz_rms_mm").toDouble(std::numeric_limits<double>::quiet_NaN());
    const bool finite=std::isfinite(summary.surfaceMeanMm) && std::isfinite(summary.surfaceMaeMm)
        && std::isfinite(summary.surfaceRmsMm) && std::isfinite(summary.surfaceMaxMm)
        && std::isfinite(summary.centerXzRmsMm) && std::isfinite(summary.center3dRmsMm)
        && std::isfinite(summary.centerMaxMm) && std::isfinite(summary.centerBiasSensorMm.x)
        && std::isfinite(summary.centerBiasSensorMm.y) && std::isfinite(summary.centerBiasSensorMm.z)
        && std::isfinite(trainRms) && std::isfinite(rmsLimit) && std::isfinite(maxLimit)
        && std::isfinite(centerLimit) && requiredPoses>=6 && requiredPoses<=100
        && rmsLimit>0.0 && maxLimit>0.0 && centerLimit>0.0;
    const double trainTolerance=std::max(1e-9,std::abs(result.residualRmsMm)*1e-6);
    const bool sameTrainingResult=finite && std::abs(trainRms-result.residualRmsMm)<=trainTolerance;
    bool strongModelMatch=false;
    const auto referencedParameters=root.value("physical17_parameters").toArray();
    const bool hasStrongModelFingerprint=
        referencedParameters.size()==static_cast<int>(Physical17Parameters::Count);
    if(hasStrongModelFingerprint){
        strongModelMatch=true;
        for(int i=0;i<referencedParameters.size();++i){
            const double expected=result.parameters[static_cast<std::size_t>(i)];
            const double actual=referencedParameters[i].toDouble(std::numeric_limits<double>::quiet_NaN());
            const double tolerance=std::max(1e-10,std::abs(expected)*1e-9);
            if(!std::isfinite(actual) || std::abs(actual-expected)>tolerance){
                strongModelMatch=false;
                break;
            }
        }
    }
    if((hasStrongModelFingerprint && !strongModelMatch)
        || (requireStrongModelMatch && !hasStrongModelFingerprint)){
        error=hasStrongModelFingerprint
            ? tr("验证报告携带的Physical17参数指纹与当前阶段2结果不一致")
            : tr("验证报告缺少与当前Physical17参数向量一致的强匹配指纹");
        summary=ValidationSummary{};
        return false;
    }
    const bool stillPassesRecordedLimits=finite && summary.poseCount>=requiredPoses
        && summary.profileCount>0 && summary.pointCount>0
        && summary.surfaceRmsMm<=rmsLimit && summary.surfaceMaxMm<=maxLimit
        && summary.centerXzRmsMm<=centerLimit && summary.bCoverageOk && summary.cCoverageOk;
    if(!summary.passed || !sameTrainingResult || !stillPassesRecordedLimits){
        error=tr("验证报告未通过完整性检查，或与当前阶段2结果不匹配");
        summary=ValidationSummary{};
        return false;
    }
    const double ratio=summary.surfaceRmsMm/std::max(1e-12,result.residualRmsMm);
    summary.message=tr("已恢复旧版独立验证：hold-out姿态=%1；RMS=%2 mm（上限%3）；max=%4 mm（上限%5）；中心XZ-RMS=%6 mm（上限%7）；验证/训练RMS=%8。")
        .arg(summary.poseCount).arg(summary.surfaceRmsMm,0,'g',6).arg(rmsLimit,0,'g',6)
        .arg(summary.surfaceMaxMm,0,'g',6).arg(maxLimit,0,'g',6)
        .arg(summary.centerXzRmsMm,0,'g',6).arg(centerLimit,0,'g',6).arg(ratio,0,'g',5);
    error.clear();
    return true;
}

QString SingleSphere14CalibrationWidget::persistentCalibrationPath() const
{
    // Keep the V27.6.12 AppData location for backward compatibility.
    QString base=QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);
    if(base.isEmpty()) base=QDir::home().filePath(QStringLiteral(".MotionLaserSync"));
    return QDir(base).filePath(QStringLiteral("calibration/active_single_sphere17.json"));
}

QString SingleSphere14CalibrationWidget::portablePersistentCalibrationPath() const
{
    return QDir(QCoreApplication::applicationDirPath())
        .filePath(QStringLiteral("calibration/active_single_sphere17.json"));
}

QStringList SingleSphere14CalibrationWidget::persistentCalibrationPaths() const
{
    QStringList paths;
    paths << persistentCalibrationPath()
          << portablePersistentCalibrationPath()
          // Also accept a root-level active file if a previous deployment copied it there.
          << QDir(QCoreApplication::applicationDirPath())
                .filePath(QStringLiteral("active_single_sphere17.json"));
    paths.removeDuplicates();
    return paths;
}

QString SingleSphere14CalibrationWidget::persistentCalibrationRevocationPath() const
{
    QString base=QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);
    if(base.isEmpty()) base=QDir::home().filePath(QStringLiteral(".MotionLaserSync"));
    return QDir(base).filePath(QStringLiteral("calibration/active_single_sphere17.revoked"));
}

QStringList SingleSphere14CalibrationWidget::persistentCalibrationRevocationPaths() const
{
    QStringList paths;
    paths << persistentCalibrationRevocationPath()
          << QDir(QCoreApplication::applicationDirPath())
                .filePath(QStringLiteral("calibration/active_single_sphere17.revoked"))
          << QDir(QCoreApplication::applicationDirPath())
                .filePath(QStringLiteral("active_single_sphere17.revoked"));
    paths.removeDuplicates();
    return paths;
}

bool SingleSphere14CalibrationWidget::writePersistentCalibrationRevocation(QString& error) const
{
    QJsonObject marker;
    marker.insert("schema",QStringLiteral("motion_laser_sync_calibration_revocation"));
    marker.insert("schema_version",1);
    marker.insert("revoked_at_utc",QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs));
    marker.insert("application_version",QCoreApplication::applicationVersion());
    marker.insert("reason",QStringLiteral("previous accepted calibration invalidated; block automatic active/legacy resurrection"));
    const QByteArray payload=QJsonDocument(marker).toJson(QJsonDocument::Indented);

    QStringList successes;
    QStringList failures;
    for(const QString& path:persistentCalibrationRevocationPaths()){
        QDir dir=QFileInfo(path).dir();
        if(!dir.exists() && !dir.mkpath(QStringLiteral("."))){
            failures << tr("%1（无法创建目录）").arg(path);
            continue;
        }
        QSaveFile file(path);
        if(!file.open(QIODevice::WriteOnly)){
            failures << tr("%1（%2）").arg(path,file.errorString());
            continue;
        }
        if(file.write(payload)<0){
            failures << tr("%1（%2）").arg(path,file.errorString());
            file.cancelWriting();
            continue;
        }
        if(!file.commit()){
            failures << tr("%1（%2）").arg(path,file.errorString());
            continue;
        }
        successes << path;
    }
    if(successes.isEmpty()){
        error=tr("无法在任何持久位置写入标定撤销标记：%1").arg(failures.join(QStringLiteral("；")));
        return false;
    }
    error.clear();
    return true;
}

bool SingleSphere14CalibrationWidget::clearPersistentCalibrationRevocation(QString& error) const
{
    QStringList failures;
    for(const QString& path:persistentCalibrationRevocationPaths()){
        if(QFile::exists(path) && !QFile::remove(path)) failures << path;
    }
    if(!failures.isEmpty()){
        error=tr("无法清除全部标定撤销标记：%1").arg(failures.join(QStringLiteral("；")));
        return false;
    }
    error.clear();
    return true;
}

bool SingleSphere14CalibrationWidget::savePersistentCalibration(QString& error)
{
    if(!m_referenceTaught || !m_hasResult || !m_lastResult.success
        || !m_validationSummary.computed || !m_validationSummary.passed){
        error=tr("只有已绑定当前坐标约定、阶段2成功且独立验证通过的结果才能保存为默认有效标定。");
        return false;
    }
    QString conventionError;
    if(!validateSessionConventions(conventionError)){
        error=tr("保存前坐标约定复核失败：%1").arg(conventionError);
        return false;
    }

    std::ostringstream resultStream;
    std::string resultError;
    if(!SingleSphere17Calibrator::writeResultJson(resultStream,m_lastResult,resultError)){
        error=QString::fromStdString(resultError);
        return false;
    }
    QJsonParseError resultParseError;
    const auto resultDoc=QJsonDocument::fromJson(
        QByteArray::fromStdString(resultStream.str()), &resultParseError);
    if(resultParseError.error!=QJsonParseError::NoError || !resultDoc.isObject()){
        error=tr("内部17参数JSON序列化失败：%1").arg(resultParseError.errorString());
        return false;
    }

    QJsonObject root;
    root.insert("schema",QStringLiteral("motion_laser_sync_active_calibration"));
    root.insert("schema_version",2);
    root.insert("saved_at_utc",QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs));
    root.insert("application_version",QCoreApplication::applicationVersion());
    root.insert("model",QStringLiteral("single_sphere_physical17_rank14"));
    root.insert("accepted",true);

    QJsonObject convention;
    convention.insert("flip_x",m_referenceFlipX);
    convention.insert("flip_z",m_referenceFlipZ);
    const std::uint8_t modelFeedbackMask = m_modelFeedbackMask != 0
        ? m_modelFeedbackMask : m_referenceFeedbackMask;
    convention.insert("feedback_mask",static_cast<int>(modelFeedbackMask));
    convention.insert("feedback_mask_encoding",QStringLiteral("XYZCB_bit0_to_bit4"));
    convention.insert("pose_sources",sourceMaskText(modelFeedbackMask));
    convention.insert("runtime_binding_feedback_mask",static_cast<int>(m_referenceFeedbackMask));
    convention.insert("runtime_binding_note", modelFeedbackMask == m_referenceFeedbackMask
        ? QStringLiteral("model and live pose sources match")
        : QStringLiteral("legacy model retained on compatible live pose source; recalibration recommended"));
    root.insert("coordinate_convention",convention);
    root.insert("physical17_result",resultDoc.object());

    QJsonObject stage1;
    stage1.insert("success",m_hasBootstrap && m_bootstrapResult.success);
    stage1.insert("state_rank",m_bootstrapResult.stateRank);
    stage1.insert("expected_state_rank",m_bootstrapResult.expectedStateRank);
    stage1.insert("residual_rms_mm",m_bootstrapResult.residualRmsMm);
    stage1.insert("residual_max_abs_mm",m_bootstrapResult.residualMaxAbsMm);
    stage1.insert("center_init_rms_mm",m_bootstrapResult.centerInitRmsMm);
    stage1.insert("condition_number",m_bootstrapResult.conditionNumber);
    stage1.insert("profile_count",static_cast<double>(m_bootstrapResult.profileCount));
    stage1.insert("point_count",static_cast<double>(m_bootstrapResult.pointCount));
    stage1.insert("captured_pose_count",m_bootstrapRecordCount);
    stage1.insert("solve_elapsed_ms",static_cast<double>(m_stage1SolveElapsedMs));
    QJsonArray stage1Parameters;
    for(double v:m_bootstrapResult.parameters.value) stage1Parameters.append(v);
    stage1.insert("parameters",stage1Parameters);
    stage1.insert("sphere_center_machine_mm",QJsonArray{
        m_bootstrapResult.sphereCenterMachineMm.x,
        m_bootstrapResult.sphereCenterMachineMm.y,
        m_bootstrapResult.sphereCenterMachineMm.z});
    root.insert("stage1_summary",stage1);

    QJsonObject validation;
    validation.insert("computed",m_validationSummary.computed);
    validation.insert("passed",m_validationSummary.passed);
    validation.insert("pose_count",m_validationSummary.poseCount);
    validation.insert("profile_count",static_cast<double>(m_validationSummary.profileCount));
    validation.insert("point_count",static_cast<double>(m_validationSummary.pointCount));
    validation.insert("surface_mean_mm",m_validationSummary.surfaceMeanMm);
    validation.insert("surface_mae_mm",m_validationSummary.surfaceMaeMm);
    validation.insert("surface_rms_mm",m_validationSummary.surfaceRmsMm);
    validation.insert("surface_max_abs_mm",m_validationSummary.surfaceMaxMm);
    validation.insert("center_xz_rms_mm",m_validationSummary.centerXzRmsMm);
    validation.insert("center_3d_rms_mm",m_validationSummary.center3dRmsMm);
    validation.insert("center_max_mm",m_validationSummary.centerMaxMm);
    validation.insert("center_bias_sensor_mm",QJsonArray{
        m_validationSummary.centerBiasSensorMm.x,
        m_validationSummary.centerBiasSensorMm.y,
        m_validationSummary.centerBiasSensorMm.z});
    validation.insert("b_positive_negative_coverage",m_validationSummary.bCoverageOk);
    validation.insert("c_positive_negative_coverage",m_validationSummary.cCoverageOk);
    validation.insert("training_surface_rms_mm",m_lastResult.residualRmsMm);
    QJsonArray validationParameters;
    for(double v:m_lastResult.parameters.value) validationParameters.append(v);
    validation.insert("physical17_parameters",validationParameters);
    validation.insert("required_pose_count",m_spinValidationPoseCount ? m_spinValidationPoseCount->value() : 8);
    validation.insert("threshold_surface_rms_mm",m_spinValidationRms ? m_spinValidationRms->value() : 0.080);
    validation.insert("threshold_surface_max_mm",m_spinValidationMax ? m_spinValidationMax->value() : 0.250);
    validation.insert("threshold_center_xz_rms_mm",m_spinValidationCenterXz ? m_spinValidationCenterXz->value() : 0.500);
    validation.insert("minimum_training_bc_separation_deg",m_spinValidationBCSep ? m_spinValidationBCSep->value() : 4.0);
    validation.insert("message",m_validationSummary.message);
    root.insert("validation",validation);

    const QByteArray payload=QJsonDocument(root).toJson(QJsonDocument::Indented);
    // Keep one visible mirror beside the executable as well as AppData and the
    // calibration subdirectory.  This directly covers portable build folders like
    // x64-Release while retaining AppData for read-only deployment directories.
    const QString rootPortablePath=QDir(QCoreApplication::applicationDirPath())
        .filePath(QStringLiteral("active_single_sphere17.json"));
    const QStringList targets={persistentCalibrationPath(),portablePersistentCalibrationPath(),rootPortablePath};
    QStringList successes;
    QStringList failures;
    for(const QString& path:targets){
        QDir dir=QFileInfo(path).dir();
        if(!dir.exists() && !dir.mkpath(QStringLiteral("."))){
            failures << tr("%1（无法创建目录）").arg(path);
            continue;
        }
        QSaveFile file(path);
        if(!file.open(QIODevice::WriteOnly)){
            failures << tr("%1（%2）").arg(path,file.errorString());
            continue;
        }
        if(file.write(payload)<0){
            failures << tr("%1（%2）").arg(path,file.errorString());
            file.cancelWriting();
            continue;
        }
        if(!file.commit()){
            failures << tr("%1（%2）").arg(path,file.errorString());
            continue;
        }
        successes << path;
    }
    if(successes.isEmpty()){
        error=tr("默认有效标定保存失败：%1").arg(failures.join(QStringLiteral("；")));
        return false;
    }
    // A revocation tombstone deliberately outranks every active/legacy file on
    // startup.  Remove it only after a newly accepted bundle has been committed.
    QString revocationError;
    if(!clearPersistentCalibrationRevocation(revocationError)){
        error=tr("新的有效标定文件已写入，但撤销标记无法清除；为避免重启误判，当前不宣称自动恢复已重新启用：%1")
            .arg(revocationError);
        return false;
    }

    m_persistentCalibrationLoaded=true;
    m_persistentCalibrationBound=true;
    m_persistedValidationSummary=m_validationSummary;
    m_persistentSourcePath=successes.first();
    m_persistentNeedsCanonicalSave=false;
    if(!failures.isEmpty()){
        appendLog(tr("默认标定已保存，但部分镜像路径不可写：%1").arg(failures.join(QStringLiteral("；"))));
    }
    error.clear();
    return true;
}

bool SingleSphere14CalibrationWidget::loadPersistentCalibrationFile(const QString& path, QString& error)
{
    QFile file(path);
    if(!file.open(QIODevice::ReadOnly)){
        error=tr("无法读取 %1：%2").arg(path,file.errorString());
        return false;
    }
    QJsonParseError pe;
    const auto doc=QJsonDocument::fromJson(file.readAll(),&pe);
    if(pe.error!=QJsonParseError::NoError || !doc.isObject()){
        error=tr("默认标定JSON解析失败：%1").arg(pe.errorString());
        return false;
    }
    const auto root=doc.object();
    const int schemaVersion=root.value("schema_version").toInt(-1);
    if(root.value("schema").toString()!=QStringLiteral("motion_laser_sync_active_calibration")
        || (schemaVersion!=1 && schemaVersion!=2)
        || !root.value("accepted").toBool(false)){
        error=tr("默认标定文件schema/version/accepted无效");
        return false;
    }

    SingleSphere17Result loadedResult;
    QString resultError;
    if(!parsePhysical17JsonObject(root.value("physical17_result").toObject(),loadedResult,resultError)){
        error=tr("默认标定中的17参数无效：%1").arg(resultError);
        return false;
    }

    const auto convention=root.value("coordinate_convention").toObject();
    if(!convention.contains("flip_x") || !convention.value("flip_x").isBool()
        || !convention.contains("flip_z") || !convention.value("flip_z").isBool()
        || !convention.contains("feedback_mask") || !convention.value("feedback_mask").isDouble()){
        error=tr("默认标定缺少或损坏Flip/DPOS-MPOS坐标约定");
        return false;
    }
    const int mask=convention.value("feedback_mask").toInt(-1);
    const QString maskEncoding=convention.value("feedback_mask_encoding").toString();
    const bool maskEncodingValid = schemaVersion>=2
        ? (maskEncoding==QStringLiteral("XYZCB_bit0_to_bit4"))
        : (maskEncoding.isEmpty() || maskEncoding==QStringLiteral("XYZCB_bit0_to_bit4"));
    if(!maskEncodingValid){
        error=schemaVersion>=2
            ? tr("默认标定V2缺少或包含不受支持的feedback_mask_encoding；必须明确为XYZCB_bit0_to_bit4")
            : tr("默认标定feedback_mask_encoding不受支持：%1").arg(maskEncoding);
        return false;
    }
    // Official schema-v1 files were emitted by V27.6.12's one-based mask bug.
    // If a schema-v1 file explicitly carries the compact encoding marker, honor it;
    // otherwise migrate the historical representation deterministically.
    const bool legacyOneBasedMask=(schemaVersion==1 && maskEncoding.isEmpty());
    bool migratedLegacyMask=false;
    std::uint8_t normalizedMask=0;
    if(!normalizePersistedFeedbackMask(mask,legacyOneBasedMask,normalizedMask,migratedLegacyMask)){
        error=legacyOneBasedMask
            ? tr("默认标定V1的旧版一基feedback_mask无效")
            : tr("默认标定feedback_mask无效");
        return false;
    }

    const auto validation=root.value("validation").toObject();
    QString badValidationField;
    if(!validationNumericFieldsAreFinite(validation,badValidationField)
        || !validation.value("computed").isBool() || !validation.value("passed").isBool()
        || !validation.value("b_positive_negative_coverage").isBool()
        || !validation.value("c_positive_negative_coverage").isBool()){
        error=tr("默认标定独立验证字段缺失/类型错误/非有限数：%1").arg(
            badValidationField.isEmpty()?QStringLiteral("boolean acceptance field"):badValidationField);
        return false;
    }
    double activePoseCountValue=0.0, activeProfileCountValue=0.0, activePointCountValue=0.0;
    double activeRequiredPoseValue=0.0;
    constexpr double kMaxActiveValidationCounter=9.0e15;
    if(!finiteJsonInteger(validation,QStringLiteral("pose_count"),0.0,
                          static_cast<double>(std::numeric_limits<int>::max()),activePoseCountValue)
        || !finiteJsonInteger(validation,QStringLiteral("profile_count"),1.0,kMaxActiveValidationCounter,activeProfileCountValue)
        || !finiteJsonInteger(validation,QStringLiteral("point_count"),1.0,kMaxActiveValidationCounter,activePointCountValue)
        || !finiteJsonInteger(validation,QStringLiteral("required_pose_count"),6.0,100.0,activeRequiredPoseValue)){
        error=tr("默认标定独立验证的计数字段不是安全范围内的整数。");
        return false;
    }
    ValidationSummary summary;
    summary.computed=validation.value("computed").toBool(false);
    summary.passed=validation.value("passed").toBool(false);
    summary.poseCount=static_cast<int>(activePoseCountValue);
    summary.profileCount=static_cast<std::size_t>(activeProfileCountValue);
    summary.pointCount=static_cast<std::size_t>(activePointCountValue);
    summary.surfaceMeanMm=validation.value("surface_mean_mm").toDouble();
    summary.surfaceMaeMm=validation.value("surface_mae_mm").toDouble();
    summary.surfaceRmsMm=validation.value("surface_rms_mm").toDouble();
    summary.surfaceMaxMm=validation.value("surface_max_abs_mm").toDouble();
    summary.centerXzRmsMm=validation.value("center_xz_rms_mm").toDouble();
    summary.center3dRmsMm=validation.value("center_3d_rms_mm").toDouble();
    summary.centerMaxMm=validation.value("center_max_mm").toDouble();
    if(!finiteJsonVec3(validation.value("center_bias_sensor_mm"),summary.centerBiasSensorMm)){
        error=tr("默认标定独立验证缺少有效的3D球心bias");
        return false;
    }
    summary.bCoverageOk=validation.value("b_positive_negative_coverage").toBool(false);
    summary.cCoverageOk=validation.value("c_positive_negative_coverage").toBool(false);
    summary.message=validation.value("message").toString();
    const double trainRms=validation.value("training_surface_rms_mm").toDouble(std::numeric_limits<double>::quiet_NaN());
    const int requiredPoses=static_cast<int>(activeRequiredPoseValue);
    const double rmsLimit=validation.value("threshold_surface_rms_mm").toDouble(std::numeric_limits<double>::quiet_NaN());
    const double maxLimit=validation.value("threshold_surface_max_mm").toDouble(std::numeric_limits<double>::quiet_NaN());
    const double centerLimit=validation.value("threshold_center_xz_rms_mm").toDouble(std::numeric_limits<double>::quiet_NaN());
    const bool finiteValidation=std::isfinite(summary.surfaceMeanMm)
        && std::isfinite(summary.surfaceMaeMm) && std::isfinite(summary.surfaceRmsMm)
        && std::isfinite(summary.surfaceMaxMm) && std::isfinite(summary.centerXzRmsMm)
        && std::isfinite(summary.center3dRmsMm) && std::isfinite(summary.centerMaxMm)
        && std::isfinite(summary.centerBiasSensorMm.x) && std::isfinite(summary.centerBiasSensorMm.y)
        && std::isfinite(summary.centerBiasSensorMm.z) && std::isfinite(trainRms)
        && std::isfinite(rmsLimit) && std::isfinite(maxLimit) && std::isfinite(centerLimit)
        && requiredPoses>=6 && requiredPoses<=100
        && rmsLimit>0.0 && maxLimit>0.0 && centerLimit>0.0;
    const double trainTolerance=std::max(1e-9,std::abs(loadedResult.residualRmsMm)*1e-6);
    const bool validationMatchesResult=finiteValidation
        && std::abs(trainRms-loadedResult.residualRmsMm)<=trainTolerance;
    bool strongModelMatch=false;
    const auto validationParameters=validation.value("physical17_parameters").toArray();
    if(validationParameters.size()==static_cast<int>(Physical17Parameters::Count)){
        strongModelMatch=true;
        for(int i=0;i<validationParameters.size();++i){
            const double expected=loadedResult.parameters[static_cast<std::size_t>(i)];
            const double actual=validationParameters[i].toDouble(std::numeric_limits<double>::quiet_NaN());
            const double tolerance=std::max(1e-10,std::abs(expected)*1e-9);
            if(!std::isfinite(actual) || std::abs(actual-expected)>tolerance){
                strongModelMatch=false;
                break;
            }
        }
    }
    const bool acceptanceStillValid=finiteValidation && summary.poseCount>=requiredPoses
        && summary.profileCount>0 && summary.pointCount>0
        && summary.surfaceRmsMm<=rmsLimit && summary.surfaceMaxMm<=maxLimit
        && summary.centerXzRmsMm<=centerLimit && summary.bCoverageOk && summary.cCoverageOk;
    if(!summary.computed || !summary.passed || !validationMatchesResult || !acceptanceStillValid
        || (schemaVersion>=2 && !strongModelMatch)){
        error=schemaVersion>=2 && !strongModelMatch
            ? tr("默认标定V2的独立验证缺少与Physical17参数完全一致的指纹")
            : tr("默认标定没有有效且与17参数匹配的独立验证通过结论");
        return false;
    }

    m_lastResult=loadedResult;
    m_hasResult=true;
    m_currentEstimate=loadedResult.parameters;
    m_sphereCenterGuessMachine=loadedResult.sphereCenterMachineMm;
    m_validationSummary=summary;
    m_persistedValidationSummary=summary;
    m_referenceFlipX=convention.value("flip_x").toBool();
    m_referenceFlipZ=convention.value("flip_z").toBool();
    m_referenceFeedbackMask=normalizedMask;
    m_modelFeedbackMask=normalizedMask;
    m_referenceTaught=false;
    m_referenceCaptured=false;
    m_sensorConventionLocked=true;
    m_plan.clear();
    m_planValidation=false;
    m_fullPlanGenerated=false;
    m_refinementSolved=true;

    // The accepted Physical17 model remains authorized by Physical17 + hold-out +
    // live convention checks above.  Do not synthesize Stage-1 from the already-refined
    // Stage-2 result: Stage-1 is still a distinct online planning/convention artifact.
    // V27.6.41 no longer needs Stage-1 as the numerical seed for a NEW final solve;
    // when Stage-2 training data are present, onSolve() derives a fresh nominal-machine
    // hand-eye+sphere bootstrap directly from those observations.
    m_bootstrapResult=NominalHandEyeSphereResult{};
    m_hasBootstrap=false;
    m_bootstrapRecordCount=0;
    m_stage1SolveElapsedMs=0;

    const auto stage1=root.value("stage1_summary").toObject();
    bool stage1SummaryValid=!stage1.isEmpty()
        && stage1.value("success").isBool() && stage1.value("success").toBool();
    double stage1Rank=0.0, stage1ExpectedRank=0.0, stage1Rms=0.0, stage1Max=0.0;
    double stage1CenterRms=0.0, stage1Condition=0.0, stage1Profiles=0.0, stage1Points=0.0;
    double stage1PoseCount=0.0, stage1Elapsed=0.0;
    if(stage1SummaryValid){
        stage1SummaryValid=finiteJsonInteger(stage1,QStringLiteral("state_rank"),9.0,100.0,stage1Rank)
            && finiteJsonInteger(stage1,QStringLiteral("expected_state_rank"),9.0,9.0,stage1ExpectedRank)
            && finiteJsonNumber(stage1,QStringLiteral("residual_rms_mm"),stage1Rms)
            && finiteJsonNumber(stage1,QStringLiteral("residual_max_abs_mm"),stage1Max)
            && finiteJsonNumber(stage1,QStringLiteral("center_init_rms_mm"),stage1CenterRms)
            && finiteJsonNumber(stage1,QStringLiteral("condition_number"),stage1Condition)
            && finiteJsonInteger(stage1,QStringLiteral("profile_count"),0.0,kMaxActiveValidationCounter,stage1Profiles)
            && finiteJsonInteger(stage1,QStringLiteral("point_count"),0.0,kMaxActiveValidationCounter,stage1Points)
            && finiteJsonInteger(stage1,QStringLiteral("captured_pose_count"),0.0,
                                 static_cast<double>(std::numeric_limits<int>::max()),stage1PoseCount)
            && finiteJsonInteger(stage1,QStringLiteral("solve_elapsed_ms"),0.0,kMaxActiveValidationCounter,stage1Elapsed)
            && stage1Rms>=0.0 && stage1Max>=0.0 && stage1CenterRms>=0.0 && stage1Condition>=0.0;
    }
    Physical17Parameters stage1Parameters=loadedResult.parameters;
    const auto stage1ParameterArray=stage1.value("parameters").toArray();
    if(stage1SummaryValid && stage1ParameterArray.size()==static_cast<int>(Physical17Parameters::Count)){
        for(int i=0;i<stage1ParameterArray.size();++i){
            if(!stage1ParameterArray[i].isDouble() || !std::isfinite(stage1ParameterArray[i].toDouble())){
                stage1SummaryValid=false;
                break;
            }
            stage1Parameters[static_cast<std::size_t>(i)]=stage1ParameterArray[i].toDouble();
        }
    }else if(stage1SummaryValid){
        stage1SummaryValid=false;
    }
    Vec3d stage1Center;
    if(stage1SummaryValid && !finiteJsonVec3(stage1.value("sphere_center_machine_mm"),stage1Center))
        stage1SummaryValid=false;

    if(stage1SummaryValid){
        m_bootstrapResult.success=true;
        m_bootstrapResult.stateRank=static_cast<int>(stage1Rank);
        m_bootstrapResult.expectedStateRank=9;
        m_bootstrapResult.residualRmsMm=stage1Rms;
        m_bootstrapResult.residualMaxAbsMm=stage1Max;
        m_bootstrapResult.centerInitRmsMm=stage1CenterRms;
        m_bootstrapResult.conditionNumber=stage1Condition;
        m_bootstrapResult.profileCount=static_cast<std::size_t>(stage1Profiles);
        m_bootstrapResult.pointCount=static_cast<std::size_t>(stage1Points);
        m_bootstrapResult.parameters=stage1Parameters;
        m_bootstrapResult.sphereCenterMachineMm=stage1Center;
        m_bootstrapRecordCount=static_cast<int>(stage1PoseCount);
        m_stage1SolveElapsedMs=static_cast<qint64>(stage1Elapsed);
        m_hasBootstrap=true;
    }
    m_loadedStage1CapturedPoses.clear();
    m_persistentCalibrationLoaded=true;
    m_persistentCalibrationBound=false;
    m_persistentSourcePath=path;
    // V27.6.12 schema-v1 files remain readable, but after live convention binding
    // rewrite them as schema-v2 so future loads also verify the full 17-vector.
    m_persistentNeedsCanonicalSave=(schemaVersion<2) || migratedLegacyMask;
    if(m_btnGenerate) m_btnGenerate->setText(tr("已恢复有效17参数，可重新生成标定姿态"));
    error.clear();
    return true;
}

bool SingleSphere14CalibrationWidget::findMatchingValidationFile(
    const QString& stage2FileName, const SingleSphere17Result& result,
    ValidationSummary& summary, QString& validationPath, QString& error) const
{
    summary=ValidationSummary{};
    validationPath.clear();
    error.clear();

    QString exact=stage2FileName;
    if(exact.endsWith(QStringLiteral(".json"),Qt::CaseInsensitive)) exact.chop(5);
    exact += QStringLiteral("_validation.json");
    if(QFileInfo::exists(exact)){
        QString exactError;
        if(loadValidationJson(exact,result,summary,exactError,false)){
            validationPath=exact;
            return true;
        }
        error=tr("%1：%2").arg(exact,exactError);
    }

    // Generic discovery is intentionally stricter than the legacy exact-name path:
    // only V27.6.13+ validation JSONs carrying the full 17-parameter fingerprint
    // may be paired by scanning, which prevents accidentally combining two runs
    // that merely happen to have similar training RMS.
    QDir dir=QFileInfo(stage2FileName).dir();
    const auto candidates=dir.entryInfoList(
        QStringList{QStringLiteral("*validation*.json")},
        QDir::Files|QDir::Readable,QDir::Time);
    for(const QFileInfo& info:candidates){
        if(QFileInfo(exact).absoluteFilePath()==info.absoluteFilePath()) continue;
        ValidationSummary candidate;
        QString candidateError;
        if(loadValidationJson(info.absoluteFilePath(),result,candidate,candidateError,true)){
            summary=candidate;
            validationPath=info.absoluteFilePath();
            error.clear();
            return true;
        }
    }
    return false;
}

bool SingleSphere14CalibrationWidget::loadLegacyCalibrationFromApplicationDir(
    QString& status, QString& error)
{
    status.clear();
    error.clear();

    QStringList dirPaths;
    dirPaths << QCoreApplication::applicationDirPath()
             << QDir(QCoreApplication::applicationDirPath()).filePath(QStringLiteral("calibration"));
    dirPaths.removeDuplicates();

    auto collect=[&](const QStringList& nameFilters){
        QList<QFileInfo> found;
        QSet<QString> seen;
        for(const QString& dirPath:dirPaths){
            QDir dir(dirPath);
            if(!dir.exists()) continue;
            const auto infos=dir.entryInfoList(nameFilters,QDir::Files|QDir::Readable,QDir::Time);
            for(const QFileInfo& info:infos){
                const QString path=info.absoluteFilePath();
                if(seen.contains(path)) continue;
                seen.insert(path);
                found.push_back(info);
            }
        }
        std::sort(found.begin(),found.end(),[](const QFileInfo& a,const QFileInfo& b){
            return a.lastModified()>b.lastModified();
        });
        return found;
    };

    const auto stage1Files=collect(QStringList{
        QStringLiteral("stage1_handeye_sphere_result*.json")
    });
    const auto stage2Files=collect(QStringList{
        QStringLiteral("stage2_physical17_result*.json"),
        QStringLiteral("single_sphere17_rank14_result*.json")
    });

    struct Stage1Candidate {
        QFileInfo info;
        NominalHandEyeSphereResult result;
        std::vector<JointPose5D> poses;
        qint64 elapsed=0;
        bool flipX=false;
        bool flipZ=false;
        std::uint8_t feedbackMask=0;
    };
    std::vector<Stage1Candidate> stage1Candidates;
    QStringList rejected;
    for(const QFileInfo& info:stage1Files){
        Stage1Candidate c;
        c.info=info;
        QString e;
        if(loadStage1ResultJson(info.absoluteFilePath(),c.result,c.poses,c.elapsed,
                                c.flipX,c.flipZ,c.feedbackMask,e)){
            stage1Candidates.push_back(std::move(c));
        }else{
            rejected << tr("%1（%2）").arg(info.fileName(),e);
        }
    }

    struct LegacyPairCandidate {
        QFileInfo stage2Info;
        SingleSphere17Result stage2;
        std::vector<const Stage1Candidate*> matchingStage1;
        ValidationSummary validation;
        QString validationPath;
        QString validationError;
        bool validated=false;
        int priority=0; // 3=validated+unique provenance, 2=validated display-only, 1=unvalidated display-only
    };
    std::vector<LegacyPairCandidate> pairCandidates;

    for(const QFileInfo& stage2Info:stage2Files){
        if(stage2Info.fileName().contains(QStringLiteral("_validation"),Qt::CaseInsensitive))
            continue;

        LegacyPairCandidate pair;
        pair.stage2Info=stage2Info;
        QString stage2Error;
        if(!loadPhysical17Json(stage2Info.absoluteFilePath(),pair.stage2,stage2Error)){
            rejected << tr("%1（%2）").arg(stage2Info.fileName(),stage2Error);
            continue;
        }

        for(const auto& candidate:stage1Candidates){
            if(sameStage1TranslationGauge(candidate.result.parameters,pair.stage2.parameters))
                pair.matchingStage1.push_back(&candidate);
        }
        if(pair.matchingStage1.empty()) continue;

        pair.validated=findMatchingValidationFile(
            stage2Info.absoluteFilePath(),pair.stage2,pair.validation,
            pair.validationPath,pair.validationError);
        const bool provenanceComplete=pair.validated
            && pair.matchingStage1.size()==1
            && !pair.matchingStage1.front()->poses.empty();
        pair.priority=provenanceComplete ? 3 : (pair.validated ? 2 : 1);
        pairCandidates.push_back(std::move(pair));
    }

    if(!pairCandidates.empty()){
        // Prefer a complete, independently validated provenance chain over a merely
        // newer display candidate.  This prevents an unvalidated Stage-2 file left
        // in the executable directory from shadowing an older accepted calibration.
        // For equal safety priority, stage2Files/pairCandidates remain newest-first.
        const LegacyPairCandidate* selected=&pairCandidates.front();
        for(const auto& candidate:pairCandidates){
            if(candidate.priority>selected->priority) selected=&candidate;
        }

        const Stage1Candidate& stage1=*selected->matchingStage1.front();
        const bool ambiguousStage1=selected->matchingStage1.size()!=1;
        const bool provenanceComplete=selected->validated
            && !ambiguousStage1 && !stage1.poses.empty();

        m_bootstrapResult=stage1.result;
        m_hasBootstrap=true;
        m_loadedStage1CapturedPoses=stage1.poses;
        m_bootstrapRecordCount=static_cast<int>(stage1.poses.size());
        m_stage1SolveElapsedMs=stage1.elapsed;
        m_referenceFlipX=stage1.flipX;
        m_referenceFlipZ=stage1.flipZ;
        m_referenceFeedbackMask=stage1.feedbackMask;
        m_modelFeedbackMask=stage1.feedbackMask;
        m_currentEstimate=selected->stage2.parameters;
        m_sphereCenterGuessMachine=selected->stage2.sphereCenterMachineMm;
        m_lastResult=selected->stage2;
        m_hasResult=true;
        m_refinementSolved=true;
        m_referenceTaught=false;
        m_referenceCaptured=false;
        m_sensorConventionLocked=true;
        m_plan.clear();
        m_planValidation=false;
        m_fullPlanGenerated=false;
        m_persistentSourcePath=selected->stage2Info.absoluteFilePath();

        if(selected->validated){
            m_validationSummary=selected->validation;
            m_persistedValidationSummary=selected->validation;
            m_persistentCalibrationLoaded=provenanceComplete;
            m_persistentCalibrationBound=false;
            m_persistentNeedsCanonicalSave=provenanceComplete;
            if(ambiguousStage1){
                status=tr("已恢复阶段2和验证报告，但发现%1个可能匹配的阶段1文件；为防止把不同标定批次混在一起，当前仅离线显示，禁止自动用于扫描/RTCP。")
                    .arg(selected->matchingStage1.size());
            }else if(provenanceComplete){
                status=tr("已从程序目录自动恢复阶段1、阶段2和匹配的独立验证：%1；等待连接设备核对坐标约定，核对通过后将升级保存为默认有效标定。")
                    .arg(selected->stage2Info.fileName());
            }else{
                status=tr("已从程序目录恢复阶段1、阶段2及验证报告，但阶段1文件缺少采集姿态来源；仅离线显示，禁止自动用于扫描/RTCP。");
            }
        }else{
            m_validationSummary=ValidationSummary{};
            m_persistedValidationSummary=ValidationSummary{};
            m_persistentCalibrationLoaded=false;
            m_persistentCalibrationBound=false;
            m_persistentNeedsCanonicalSave=false;
            status=ambiguousStage1
                ? tr("已从程序目录恢复阶段2以及一个阶段1候选，但另有%1个阶段1候选同样可能匹配，且未找到通过的hold-out验证；当前仅离线显示。")
                    .arg(selected->matchingStage1.size()-1)
                : tr("已从程序目录自动恢复阶段1和阶段2：%1；未找到与该17参数匹配且通过的hold-out验证JSON，因此当前仅作离线结果显示，禁止用于扫描/RTCP。")
                    .arg(selected->stage2Info.fileName());
        }

        if(selected!=&pairCandidates.front() && selected->priority>pairCandidates.front().priority){
            status += tr(" 为避免较新的未验证/来源不完整文件遮蔽有效标定，启动恢复已优先选择安全等级更高的历史候选。");
        }
        if(m_btnGenerate) m_btnGenerate->setText(tr("已恢复17参数结果，可作为初值重新标定"));
        error.clear();
        return true;
    }

    // Stage-1 alone is still useful to show the user that the file was found. It is
    // never treated as a completed calibration.
    if(!stage1Candidates.empty()){
        auto& stage1=stage1Candidates.front();
        m_bootstrapResult=stage1.result;
        m_hasBootstrap=true;
        m_loadedStage1CapturedPoses=stage1.poses;
        m_bootstrapRecordCount=static_cast<int>(stage1.poses.size());
        m_stage1SolveElapsedMs=stage1.elapsed;
        m_referenceFlipX=stage1.flipX;
        m_referenceFlipZ=stage1.flipZ;
        m_referenceFeedbackMask=stage1.feedbackMask;
        m_modelFeedbackMask=stage1.feedbackMask;
        m_referenceTaught=false;
        m_sensorConventionLocked=false;
        m_currentEstimate=stage1.result.parameters;
        m_sphereCenterGuessMachine=stage1.result.sphereCenterMachineMm;
        status=tr("已从程序目录自动恢复阶段1结果 %1；尚未找到与其规范锚点匹配的阶段2结果，因此仍未形成可用的17参数标定。")
            .arg(stage1.info.fileName());
        return true;
    }

    if(!rejected.isEmpty())
        error=tr("程序目录中的旧版标定候选文件无效：%1").arg(rejected.join(QStringLiteral("；")));
    return false;
}

bool SingleSphere14CalibrationWidget::loadPersistentCalibration(QString& error)
{
    m_persistentCalibrationLoaded=false;
    m_persistentCalibrationBound=false;
    m_persistedValidationSummary=ValidationSummary{};
    m_persistentBindError.clear();
    m_persistentSourcePath.clear();
    m_persistentNeedsCanonicalSave=false;
    m_startupRestoreMessage.clear();

    // Revocation marker is stored only in the writable per-user location and
    // intentionally outranks all mirrored active files and legacy Stage1/Stage2
    // candidates.  This prevents an undeletable portable mirror or an old
    // *_validation.json from resurrecting a model that was invalidated later.
    QStringList existingRevocations;
    for(const QString& revocationPath:persistentCalibrationRevocationPaths()){
        if(QFileInfo::exists(revocationPath)) existingRevocations << revocationPath;
    }
    if(!existingRevocations.isEmpty()){
        m_startupRestoreMessage=tr("检测到标定撤销标记：旧默认标定及旧版阶段1/阶段2文件不会自动恢复。必须由新的完整标定+独立验证重新建立有效标定。标记：%1")
            .arg(existingRevocations.join(QStringLiteral("；")));
        error.clear();
        return true;
    }

    QList<QFileInfo> activeCandidates;
    for(const QString& path:persistentCalibrationPaths()){
        QFileInfo info(path);
        if(info.exists() && info.isFile()) activeCandidates.push_back(info);
    }
    std::sort(activeCandidates.begin(),activeCandidates.end(),[](const QFileInfo& a,const QFileInfo& b){
        return a.lastModified()>b.lastModified();
    });

    QStringList activeErrors;
    for(const QFileInfo& info:activeCandidates){
        QString e;
        if(loadPersistentCalibrationFile(info.absoluteFilePath(),e)){
            m_startupRestoreMessage=tr("已自动载入默认有效标定：%1").arg(info.absoluteFilePath());
            error.clear();
            return true;
        }
        activeErrors << tr("%1（%2）").arg(info.absoluteFilePath(),e);
    }

    QString legacyStatus;
    QString legacyError;
    if(loadLegacyCalibrationFromApplicationDir(legacyStatus,legacyError)){
        m_startupRestoreMessage=legacyStatus;
        error.clear();
        return true;
    }

    if(!activeErrors.isEmpty() || !legacyError.isEmpty()){
        QStringList all=activeErrors;
        if(!legacyError.isEmpty()) all << legacyError;
        error=all.join(QStringLiteral("；"));
        return false;
    }
    error.clear();
    return false;
}

bool SingleSphere14CalibrationWidget::tryBindPersistentCalibration(QString& error)
{
    if(!m_persistentCalibrationLoaded || !m_hasResult || !m_lastResult.success
        || !m_persistedValidationSummary.passed){
        error=tr("没有可绑定的默认有效标定");
        return false;
    }
    if(m_referenceTaught){
        error.clear();
        return true;
    }
    if(!m_controller || !m_controller->isConnected() || !m_sensor || !m_sensor->isConnected()){
        error.clear();
        return false;
    }

    JointState5D state;
    QString stateError;
    if(!readStableState(state,stateError)){
        error=tr("无法读取稳定五轴状态：%1").arg(stateError);
        return false;
    }
    std::uint8_t currentMask=0;
    if(!readPoseSourceMask(currentMask,stateError)){
        error=tr("无法核对XYZCB的DPOS/MPOS来源：%1").arg(stateError);
        return false;
    }
    SensorAdvancedParams adv;
    QString sensorError;
    if(!m_sensor->readAdvancedParams(adv,&sensorError)){
        error=tr("无法核对2950 FlipX/FlipZ：%1").arg(sensorError);
        return false;
    }

    const bool legacySourceCompatibility =
        legacyXyzDposModelCanSeedFullMpos(m_referenceFeedbackMask, currentMask);
    QStringList mismatch;
    if(currentMask!=m_referenceFeedbackMask && !legacySourceCompatibility){
        mismatch << tr("XYZCB数据源：保存时=%1；当前=%2")
            .arg(sourceMaskText(m_referenceFeedbackMask),sourceMaskText(currentMask));
    }
    if(adv.flipX!=m_referenceFlipX || adv.flipZ!=m_referenceFlipZ){
        mismatch << tr("2950 Flip：保存时 FlipX=%1 FlipZ=%2；当前 FlipX=%3 FlipZ=%4")
            .arg(m_referenceFlipX).arg(m_referenceFlipZ).arg(adv.flipX).arg(adv.flipZ);
    }
    if(!mismatch.isEmpty()){
        error=tr("保存标定与当前设备坐标约定不一致，禁止自动使用。请恢复保存时设置或重新完成标定。\n%1")
            .arg(mismatch.join(QStringLiteral("\n")));
        m_persistentCalibrationBound=false;
        return false;
    }

    if (legacySourceCompatibility) {
        appendLog(tr("兼容加载旧标定：保存结果使用XYZ DPOS + C/B MPOS，当前实机使用XYZ/C/B全MPOS。"
                     "旧17参数继续允许用于扫描/RTCP及新标定初值，但建议尽快完成一次全MPOS更新标定。"));
    }
    m_referenceTaught=true;
    m_referenceCaptured=false;
    m_sensorConventionLocked=true;
    // Bind the live session to current measured MPOS while retaining the model's
    // original feedback source in m_modelFeedbackMask for persistence/provenance.
    m_referenceFeedbackMask=currentMask;
    m_persistentCalibrationBound=true;
    m_validationSummary=m_persistedValidationSummary;
    m_referenceSeed=PosePlannerSeed{};
    m_referenceSeed.taughtJoint=SingleSphere17QtAdapter::toPortable(
        calibrationPoseFromState(state,currentMask));
    m_referenceSeed.hasSphereCenterMachine=true;
    m_referenceSeed.sphereCenterMachineMm=m_lastResult.sphereCenterMachineMm;
    m_referenceSeed.taughtSphereCenterSensorMm=SingleSphere17Model::machinePointInSensor(
        m_referenceSeed.taughtJoint,m_lastResult.sphereCenterMachineMm,m_lastResult.parameters);
    m_sphereCenterGuessMachine=m_lastResult.sphereCenterMachineMm;
    m_currentEstimate=m_lastResult.parameters;
    m_refinementSolved=true;
    if(m_lblConvention){
        m_lblConvention->setText(tr(
            "默认有效标定已绑定：FlipX=%1，FlipZ=%2；%3。已恢复11项机床几何误差 + 6DoF手眼关系。")
            .arg(m_referenceFlipX).arg(m_referenceFlipZ).arg(sourceMaskText(m_referenceFeedbackMask)));
    }
    updateCurrentPoseLabel(&state);
    error.clear();
    return true;
}

bool SingleSphere14CalibrationWidget::tryBindCurrentSolvedResultForCalibrationMotion(QString& error)
{
    // This binding is deliberately weaker than production authorization and stronger
    // than an offline display state.  Its only purpose is to allow a newly solved,
    // not-yet-validated Physical17 model to drive Stage-2/hold-out calibration motion
    // after a CSV import.  It NEVER restores a previous validation verdict and therefore
    // cannot by itself unlock scan/RTCP (scanCalibration() still requires validation).
    if (m_referenceTaught) {
        if (!validateSessionConventions(error)) return false;
        error.clear();
        return true;
    }
    if (!m_hasResult || !m_lastResult.success) {
        error = tr("当前没有成功冻结的Physical17结果。请先完成阶段2最终求解。");
        return false;
    }
    if (!m_hasBootstrap || !m_bootstrapResult.success) {
        error = tr("缺少可验证的阶段1/默认标定坐标约定来源。离线Stage2-only CSV可以完成数值求解和验证姿态设计，"
                   "但要执行真实自动移动/采集，必须先加载对应阶段1来源或从已绑定有效标定进入该CSV流程，以核对FlipX/FlipZ和XYZCB DPOS/MPOS来源。");
        return false;
    }
    if (!m_controller || !m_controller->isConnected() || !m_sensor || !m_sensor->isConnected()) {
        error = tr("控制器或2950传感器未连接，无法重新绑定当前阶段2结果用于验证运动。");
        return false;
    }

    JointState5D state;
    QString stateError;
    if (!readStableState(state, stateError)) {
        error = tr("无法读取稳定五轴状态：%1").arg(stateError);
        return false;
    }

    std::uint8_t currentMask = 0;
    if (!readPoseSourceMask(currentMask, stateError)) {
        error = tr("无法核对XYZCB的DPOS/MPOS来源：%1").arg(stateError);
        return false;
    }
    SensorAdvancedParams adv;
    QString sensorError;
    if (!m_sensor->readAdvancedParams(adv, &sensorError)) {
        error = tr("无法核对2950 FlipX/FlipZ：%1").arg(sensorError);
        return false;
    }

    QStringList mismatch;
    // Hold-out validation must use the SAME pose-source convention as the training
    // model.  Unlike runtime compatibility mode, do not silently accept XYZ-DPOS ->
    // MPOS migration here because that would make validation and training coordinates
    // different populations.
    if (currentMask != m_referenceFeedbackMask) {
        mismatch << tr("XYZCB数据源：训练/阶段1来源=%1；当前=%2")
            .arg(sourceMaskText(m_referenceFeedbackMask), sourceMaskText(currentMask));
    }
    if (adv.flipX != m_referenceFlipX || adv.flipZ != m_referenceFlipZ) {
        mismatch << tr("2950 Flip：训练来源 FlipX=%1 FlipZ=%2；当前 FlipX=%3 FlipZ=%4")
            .arg(m_referenceFlipX).arg(m_referenceFlipZ).arg(adv.flipX).arg(adv.flipZ);
    }

    // V27.6.39+ Stage-2 self-bootstrap intentionally chooses its OWN translation
    // gauge from retained Stage-2 data; it no longer has to equal Stage-1 Tx/Ty/Tz.
    // What must be exact is that the final Physical17 translation still equals the
    // recorded Stage-2 initialization anchor.  Legacy results continue to use the
    // Stage-1 gauge-integrity check.
    const bool stage2SelfBootstrapGauge = m_lastResult.hasInitializationBootstrap
        && m_lastResult.initializationSource.rfind("stage2_", 0) == 0;
    double maxGaugeDeltaMm = 0.0;
    if (stage2SelfBootstrapGauge) {
        if (!sameStage1TranslationGauge(m_lastResult.initializationParameters,
                                        m_lastResult.parameters, &maxGaugeDeltaMm)) {
            mismatch << tr("阶段2自举平移规范锚点与最终Physical17不一致：最大|ΔT|=%1 mm。"
                           "这说明结果文件/内存状态已损坏，禁止用于验证运动。")
                .arg(maxGaugeDeltaMm, 0, 'g', 10);
        }
    } else {
        if (!sameStage1TranslationGauge(m_bootstrapResult.parameters,
                                        m_lastResult.parameters, &maxGaugeDeltaMm)) {
            mismatch << tr("旧版阶段1平移规范锚点与Physical17不一致：最大|ΔT|=%1 mm。")
                .arg(maxGaugeDeltaMm, 0, 'g', 10);
        }
    }

    if (!mismatch.isEmpty()) {
        error = tr("当前设备/模型不能安全重新绑定用于独立验证运动：\n%1")
            .arg(mismatch.join(QStringLiteral("\n")));
        return false;
    }

    m_referenceTaught = true;
    m_referenceCaptured = false;
    m_sensorConventionLocked = true;
    m_referenceFeedbackMask = currentMask;
    m_persistentCalibrationBound = false; // current result is not production-authorized yet
    m_referenceSeed = PosePlannerSeed{};
    m_referenceSeed.hasTaughtJoint = true;
    m_referenceSeed.taughtJoint = SingleSphere17QtAdapter::toPortable(
        calibrationPoseFromState(state, currentMask));
    m_referenceSeed.hasSphereCenterMachine = true;
    m_referenceSeed.sphereCenterMachineMm = m_lastResult.sphereCenterMachineMm;
    m_referenceSeed.taughtSphereCenterSensorMm = SingleSphere17Model::machinePointInSensor(
        m_referenceSeed.taughtJoint, m_lastResult.sphereCenterMachineMm, m_lastResult.parameters);
    m_sphereCenterGuessMachine = m_lastResult.sphereCenterMachineMm;
    m_currentEstimate = m_lastResult.parameters;
    m_refinementSolved = true;
    if (m_lblConvention) {
        m_lblConvention->setText(tr(
            "本次Physical17已重新绑定用于标定/独立验证：FlipX=%1，FlipZ=%2；%3。"
            "旧hold-out结论未恢复，扫描/RTCP仍需本次验证通过。")
            .arg(m_referenceFlipX).arg(m_referenceFlipZ)
            .arg(sourceMaskText(m_referenceFeedbackMask)));
    }
    updateCurrentPoseLabel(&state);

    appendLog(tr("当前阶段2求解结果已重新绑定到实机，仅用于标定/独立验证运动：FlipX=%1 FlipZ=%2；%3。"
                 "未恢复任何旧hold-out通过结论，扫描/RTCP仍保持锁定，直到本次独立验证真正通过。")
              .arg(m_referenceFlipX).arg(m_referenceFlipZ)
              .arg(sourceMaskText(m_referenceFeedbackMask)));
    error.clear();
    return true;
}

bool SingleSphere14CalibrationWidget::removePersistentCalibration(QString& error)
{
    // Write a fail-closed tombstone BEFORE attempting to delete mirrors. Portable
    // deployment folders may be read-only, and legacy Stage1/Stage2+validation
    // files are intentionally retained as records. Without this marker either one
    // could otherwise re-authorize a calibration after restart.
    QString revocationError;
    const bool revocationWritten=writePersistentCalibrationRevocation(revocationError);

    QStringList failures;
    for(const QString& path:persistentCalibrationPaths()){
        if(QFile::exists(path) && !QFile::remove(path))
            failures << path;
    }

    // RAM authorization is always cleared even when filesystem cleanup is only
    // partially successful. The revocation marker protects subsequent startups.
    m_persistentCalibrationLoaded=false;
    m_persistentCalibrationBound=false;
    m_persistedValidationSummary=ValidationSummary{};
    m_persistentBindError.clear();
    m_persistentSourcePath.clear();
    m_persistentNeedsCanonicalSave=false;

    if(!revocationWritten){
        QStringList problems;
        problems << tr("撤销标记写入失败：%1").arg(revocationError);
        if(!failures.isEmpty()) problems << tr("部分旧镜像也无法删除：%1").arg(failures.join(QStringLiteral("；")));
        error=problems.join(QStringLiteral("；"));
        return false;
    }
    // Once at least one tombstone is durably written, lingering read-only active
    // mirrors and legacy Stage1/Stage2 files are blocked on every subsequent start.
    // Their deletion is best-effort cleanup rather than the authorization barrier.
    if(!failures.isEmpty()){
        appendLog(tr("标定撤销标记已写入；以下只读旧镜像未删除但不会在重启后重新授权：%1")
            .arg(failures.join(QStringLiteral("；"))));
    }
    error.clear();
    return true;
}

void SingleSphere14CalibrationWidget::onLoadResult()
{
    if (m_referenceTaught || !m_records.empty() || !m_plan.empty()) {
        QMessageBox::information(this, tr("请先清空当前标定会话"),
            tr("当前会话已经锁定了传感器Flip、DPOS/MPOS数据源或规范锚点。为避免把不同坐标约定/不同规范的结果混用，"
               "请先点击“清空会话”，再加载已有17参数JSON。"));
        return;
    }
    const QString fn=QFileDialog::getOpenFileName(this,tr("加载单球17参数结果"),QString(),tr("JSON (*.json)"));
    if(fn.isEmpty())return;
    SingleSphere17Result r;
    QString err;
    if(!loadPhysical17Json(fn,r,err)){
        QMessageBox::warning(this,tr("加载失败"),err);
        return;
    }
    ValidationSummary recoveredValidation;
    QString validationPath;
    QString validationError;
    const bool recoveredValidatedResult=findMatchingValidationFile(
        fn,r,recoveredValidation,validationPath,validationError);

    const bool preserveLoadedStage1 = m_hasBootstrap && m_bootstrapResult.success;
    m_persistentCalibrationLoaded = false;
    m_persistentCalibrationBound = false;
    m_persistedValidationSummary = ValidationSummary{};
    m_persistentBindError.clear();
    m_lastResult=r;
    m_hasResult=r.success;
    m_modelFeedbackMask = preserveLoadedStage1 ? m_referenceFeedbackMask : 0;
    // Keep the loaded Stage-2 model as the best current model for display/planning.
    // V27.6.41 onSolve() does not reuse this refined model as its numerical seed;
    // it derives a fresh nominal-machine hand-eye+sphere bootstrap from the current
    // Stage-2 training observations.
    m_currentEstimate=r.parameters;
    m_sphereCenterGuessMachine=r.sphereCenterMachineMm;
    m_planBootstrap=false;
    m_fullPlanGenerated=false;
    m_refinementSolved=r.success;
    m_hasBootstrap=preserveLoadedStage1;
    m_validationRecords.clear();
    m_validationSummary=recoveredValidatedResult ? recoveredValidation : ValidationSummary{};
    m_planValidation=false;
    if (!preserveLoadedStage1) {
        // Stage-2-only JSON may be inspected, but it must never masquerade as an
        // original Stage-1 planning/convention result.  A future final solve can still
        // proceed after Stage-2 training CSV import via the V27.6.41 data self-bootstrap.
        m_bootstrapResult=NominalHandEyeSphereResult{};
        m_bootstrapRecordCount=0;
        m_stage1SolveElapsedMs=0;
        m_loadedStage1CapturedPoses.clear();
    }
    m_btnGenerate->setText(tr("先锁定当前坐标约定，再可用已加载模型生成姿态"));
    updateResultTable(&m_lastResult);
    updateValidationSummary();
    updateCalibrationReadiness();
    updateWorkflowGuide();
    if(recoveredValidatedResult){
        appendLog(tr("已加载阶段2标定结果：%1，并自动恢复匹配的旧版独立验证报告：%2。加载对应阶段1 JSON并绑定当前设备后，可直接升级为默认有效标定，无需重做hold-out。")
            .arg(fn,validationPath));
    }else{
        appendLog(tr("已加载阶段2标定结果：%1。未恢复可接受的同名独立验证报告（%2）；如需最终使用，仍须完成hold-out验证。")
            .arg(fn, validationError.isEmpty()?tr("文件不存在"):validationError));
    }
}

void SingleSphere14CalibrationWidget::onBindLoadedResultToCurrentSession()
{
    if (m_referenceTaught) {
        QMessageBox::information(this, tr("已经绑定"), tr("当前标定结果已经绑定到本次实机坐标约定，无需重复绑定。"));
        return;
    }
    if (!m_hasResult || !m_lastResult.success) {
        QMessageBox::warning(this, tr("无法绑定"), tr("请先加载成功的阶段2 single_sphere_physical17_rank14 结果 JSON。"));
        return;
    }
    if (m_persistentCalibrationLoaded && m_validationSummary.passed) {
        QString bindError;
        if (!tryBindPersistentCalibration(bindError)) {
            QMessageBox::critical(this, tr("自动标定无法绑定"),
                bindError.isEmpty() ? tr("设备尚未就绪，无法核对保存标定的坐标约定。") : bindError);
            return;
        }
        updateSessionSummary();
        updateCalibrationReadiness();
        updateWorkflowGuide();
        updateValidationSummary();
        QMessageBox::information(this, tr("有效标定已恢复"),
            tr("保存的17参数（11项机床几何误差 + 6DoF手眼）及独立验证结论已与当前设备坐标约定核对。\n\n可继续用于扫描/RTCP；若属于旧XYZ-DPOS结果，建议更新为全MPOS标定。"));
        return;
    }

    // V27.6.42B: a newly solved Stage-2 CSV result is intentionally unbound while
    // the offline solve runs.  Re-bind only the coordinate convention + current
    // measured start state; never import the previous production hold-out verdict.
    const bool stage2SelfBootstrapGauge = m_lastResult.hasInitializationBootstrap
        && m_lastResult.initializationSource.rfind("stage2_", 0) == 0;
    if (stage2SelfBootstrapGauge || (m_hasBootstrap && m_bootstrapResult.success
                                     && m_loadedStage1CapturedPoses.empty())) {
        QString bindError;
        if (!tryBindCurrentSolvedResultForCalibrationMotion(bindError)) {
            QMessageBox::critical(this, tr("当前阶段2结果无法绑定"), bindError);
            return;
        }
        updateSessionSummary();
        updateCalibrationReadiness();
        updateWorkflowGuide();
        updateValidationSummary();

        if (m_validationSummary.passed) {
            QString persistenceError;
            const bool persisted = savePersistentCalibration(persistenceError);
            QMessageBox::information(this, tr("已验证结果绑定完成"),
                persisted
                    ? tr("当前Physical17结果及本次hold-out通过结论已绑定并保存为默认有效标定。")
                    : tr("当前Physical17结果已绑定，但默认有效标定保存失败：%1").arg(persistenceError));
        } else {
            QMessageBox::information(this, tr("阶段2结果已绑定用于验证运动"),
                tr("当前FlipX/FlipZ与XYZCB DPOS/MPOS来源已重新核对。现在可以生成/执行独立hold-out验证姿态。\n\n"
                   "注意：这只恢复标定/验证运动权限，没有恢复旧验证结论；扫描/RTCP仍保持锁定。"));
        }
        return;
    }
    if (!m_hasBootstrap || !m_bootstrapResult.success || m_loadedStage1CapturedPoses.empty()) {
        QMessageBox::warning(this, tr("缺少阶段1来源信息"),
            tr("为了安全恢复到实机，必须先加载与该阶段2结果对应的 stage1_handeye_sphere_result.json。"
               "阶段2结果本身不保存当时的FlipX/FlipZ和各轴DPOS/MPOS来源，不能在不知道坐标约定的情况下直接用于运动/RTCP。"));
        return;
    }
    if (!m_controller || !m_controller->isConnected() || !m_sensor || !m_sensor->isConnected()) {
        QMessageBox::warning(this, tr("设备未连接"), tr("请先连接控制器和2950传感器，再绑定已加载结果。"));
        return;
    }

    JointState5D state;
    QString error;
    if (!readStableState(state, error)) {
        QMessageBox::warning(this, tr("五轴状态不安全"), error);
        return;
    }
    std::uint8_t currentMask = 0;
    if (!readPoseSourceMask(currentMask, error)) {
        QMessageBox::warning(this, tr("无法读取DPOS/MPOS约定"), error);
        return;
    }
    SensorAdvancedParams adv;
    QString sensorError;
    if (!m_sensor->readAdvancedParams(adv, &sensorError)) {
        QMessageBox::warning(this, tr("无法读取2950坐标约定"), sensorError);
        return;
    }

    const bool legacySourceCompatibility =
        legacyXyzDposModelCanSeedFullMpos(m_referenceFeedbackMask, currentMask);
    QStringList mismatch;
    if (currentMask != m_referenceFeedbackMask && !legacySourceCompatibility) {
        mismatch << tr("五轴数据源：阶段1=%1，当前=%2")
            .arg(sourceMaskText(m_referenceFeedbackMask), sourceMaskText(currentMask));
    }
    if (adv.flipX != m_referenceFlipX || adv.flipZ != m_referenceFlipZ) {
        mismatch << tr("2950 Flip：阶段1 FlipX=%1 FlipZ=%2，当前 FlipX=%3 FlipZ=%4")
            .arg(m_referenceFlipX).arg(m_referenceFlipZ).arg(adv.flipX).arg(adv.flipZ);
    }

    const auto& stage1p = m_bootstrapResult.parameters;
    const auto& stage2p = m_lastResult.parameters;
    double maxGaugeDeltaMm=0.0;
    if(!sameStage1TranslationGauge(stage1p,stage2p,&maxGaugeDeltaMm)){
        mismatch << tr("阶段1平移规范锚点与阶段2文件不一致：最大|ΔT|=%1 mm。阶段2必须继承同一阶段1的平移规范锚点，而不是只要求数值接近。")
            .arg(maxGaugeDeltaMm,0,'g',10);
    }

    if (!mismatch.isEmpty()) {
        const QString detail = mismatch.join(QStringLiteral("\n"));
        appendLog(tr("拒绝绑定已加载阶段2结果：%1").arg(detail));
        QMessageBox::critical(this, tr("坐标约定不一致，禁止绑定"),
            tr("当前设备状态与保存标定时的坐标约定不一致。为了避免把错误的17参数用于五轴运动/RTCP，本次绑定被拒绝。\n\n%1")
                .arg(detail));
        return;
    }

    if (legacySourceCompatibility) {
        appendLog(tr("已按兼容模式绑定旧XYZ-DPOS标定到当前全MPOS实机坐标：结果仍可使用，并作为球心/路径规划初值；建议随后更新标定结果。"));
    }
    m_referenceTaught = true;
    m_sensorConventionLocked = true;
    m_referenceCaptured = false;
    m_referenceFlipX = adv.flipX;
    m_referenceFlipZ = adv.flipZ;
    if (m_modelFeedbackMask == 0) m_modelFeedbackMask = legacySourceCompatibility ? std::uint8_t{24} : currentMask;
    m_referenceFeedbackMask = currentMask;
    m_referenceSeed.taughtJoint = SingleSphere17QtAdapter::toPortable(
        calibrationPoseFromState(state, currentMask));
    m_referenceSeed.hasSphereCenterMachine = true;
    m_referenceSeed.sphereCenterMachineMm = m_lastResult.sphereCenterMachineMm;
    m_referenceSeed.taughtSphereCenterSensorMm = SingleSphere17Model::machinePointInSensor(
        m_referenceSeed.taughtJoint, m_lastResult.sphereCenterMachineMm, m_lastResult.parameters);

    updateCurrentPoseLabel(&state);
    updateSessionSummary();
    updateCalibrationReadiness();
    updateWorkflowGuide();
    updateValidationSummary();
    if(m_validationSummary.passed){
        QString persistenceError;
        const bool persisted=savePersistentCalibration(persistenceError);
        if(persisted){
            m_persistentCalibrationLoaded=true;
            m_persistentCalibrationBound=true;
            m_persistedValidationSummary=m_validationSummary;
            m_persistentBindError.clear();
        }
        appendLog(tr("已把旧版已验证标定绑定到当前实机坐标约定：%1；FlipX=%2 FlipZ=%3；训练RMS=%4 mm，rank=%5/17。%6")
            .arg(sourceMaskText(currentMask)).arg(adv.flipX).arg(adv.flipZ)
            .arg(m_lastResult.residualRmsMm,0,'g',8).arg(m_lastResult.observability.targetRank)
            .arg(persisted
                ? (legacySourceCompatibility
                    ? tr("已保存为默认兼容标定，并保留原XYZ-DPOS来源标记；今后启动仍会提示建议更新。")
                    : tr("已保存为默认有效标定，今后启动自动恢复。"))
                : tr("本次会话可直接使用，但默认标定自动保存失败：%1").arg(persistenceError)));
        QMessageBox::information(this,tr("已验证标定恢复完成"),
            tr("阶段1坐标约定、阶段2的17参数以及旧版hold-out验证报告均已核对通过。\n\n"
               "本次结果可以继续用于扫描/RTCP；若属于旧XYZ-DPOS结果，建议后续更新为全MPOS标定。%1")
                .arg(persisted ? tr("今后启动程序也会自动载入这套有效标定。")
                               : tr("但自动持久化失败：%1").arg(persistenceError)));
    }else{
        appendLog(tr("已把加载的阶段2结果绑定到当前实机坐标约定：%1；FlipX=%2 FlipZ=%3；训练结果RMS=%4 mm，rank=%5/17。"
                     "当前仍属于‘训练求解通过、待独立hold-out验证’，验证通过后 scanCalibration() 才会向RTCP/扫描模块提供17参数。")
            .arg(sourceMaskText(currentMask)).arg(adv.flipX).arg(adv.flipZ)
            .arg(m_lastResult.residualRmsMm,0,'g',8).arg(m_lastResult.observability.targetRank));
        QMessageBox::information(this, tr("阶段2结果已绑定"),
            tr("当前设备Flip和位置数据源已核对；若为旧XYZ-DPOS结果则按兼容模式恢复到当前全MPOS实机会话，并建议更新标定。\n\n"
               "不需要重新进行阶段2求解。下一步请生成并采集独立hold-out验证姿态；验证通过后才会允许最终RTCP/扫描使用。"));
    }
}

void SingleSphere14CalibrationWidget::onExportSessionCsv()
{
    const auto observations = stage2TrainingObservations();
    if(observations.empty()){
        QMessageBox::information(this,tr("无阶段2数据"),tr("没有可导出的阶段2训练观测。阶段1bootstrap数据不会再导出到Physical17训练CSV。"));
        return;
    }
    const QString fn=QFileDialog::getSaveFileName(this,tr("导出阶段2训练CSV"),QStringLiteral("single_sphere17_stage2_profiles.csv"),tr("CSV (*.csv)"));
    if(fn.isEmpty())return;
    QFile f(fn);
    if(!f.open(QIODevice::WriteOnly|QIODevice::Text)){QMessageBox::warning(this,tr("保存失败"),f.errorString());return;}
    QTextStream out(&f);
    out.setRealNumberPrecision(15);
    out<<"profile_id,x_mm,y_mm,z_mm,c_deg,b_deg,"
          "mpos_x_mm,mpos_y_mm,mpos_z_mm,mpos_c_deg,mpos_b_deg,"
          "dpos_x_mm,dpos_y_mm,dpos_z_mm,dpos_c_deg,dpos_b_deg,"
          "sensor_x_mm,sensor_z_mm,weight\n";
    std::size_t points=0;
    std::size_t profilesWithDpos=0;
    for(const auto& o:observations) {
        if (o.hasDposJoints) ++profilesWithDpos;
        for(const auto& p:o.points) {
            out<<o.profileId<<','
               <<o.joints.x<<','<<o.joints.y<<','<<o.joints.z<<','<<o.joints.c<<','<<o.joints.b<<','
               <<o.joints.x<<','<<o.joints.y<<','<<o.joints.z<<','<<o.joints.c<<','<<o.joints.b<<',';
            if (o.hasDposJoints) {
                out<<o.dposJoints.x<<','<<o.dposJoints.y<<','<<o.dposJoints.z<<','<<o.dposJoints.c<<','<<o.dposJoints.b<<',';
            } else {
                out<<",,,,,";
            }
            out<<p.x<<','<<p.z<<','<<p.weight<<'\n';
            ++points;
        }
    }
    const std::size_t stored=m_calibrator.observations().size();
    const std::size_t omitted=stored>observations.size()?stored-observations.size():0;
    appendLog(tr("已导出阶段2训练数据：%1 profiles / %2点 -> %3。其中%4 profiles同时保存MPOS+DPOS；x_mm...b_deg继续作为求解MPOS兼容列。阶段1bootstrap profiles不会进入17参数训练CSV。%5")
        .arg(static_cast<qulonglong>(observations.size()))
        .arg(static_cast<qulonglong>(points)).arg(fn)
        .arg(static_cast<qulonglong>(profilesWithDpos))
        .arg(omitted>0?tr(" 本次自动剔除旧混合数据中的%1个阶段1 profiles。").arg(static_cast<qulonglong>(omitted)):QString()));
}

void SingleSphere14CalibrationWidget::onImportSessionCsv()
{
    if (m_stage2SolveRunning) {
        QMessageBox::information(this,tr("阶段2正在求解"),tr("请等待当前求解完成或取消后再导入CSV。"));
        return;
    }
    if (!m_records.empty()) {
        QMessageBox::information(this,tr("请先处理当前实机采集数据"),
            tr("当前会话已经包含阶段1/阶段2实机采集记录。为避免把现场采集与离线CSV混合，请先导出或清空这些记录后再导入阶段2CSV。"));
        return;
    }

    // A persisted calibration may auto-bind as soon as controller + sensor are online.
    // In V27.6.42 that set m_referenceTaught=true and unintentionally disabled the
    // Stage-2 CSV button, even though CSV import itself is an offline-data operation.
    // Allow the import, but make the mode transition explicit and safe: devices stay
    // connected, while the calibration/motion session becomes unbound after import.
    const bool wasLiveBound = m_referenceTaught;
    if (wasLiveBound) {
        const auto answer = QMessageBox::question(
            this, tr("切换到离线阶段2CSV模式"),
            tr("当前已验证标定已经绑定到实机。导入阶段2CSV不会断开控制器或传感器，但会：\n"
               "1) 退出当前标定运动会话；离线求解/姿态设计仍可继续，真实移动前会重新核对坐标约定；\n"
               "2) 清空当前阶段2计划/验证内存集；\n"
               "3) 保留已加载的阶段1坐标约定用于旧混合CSV筛选；\n"
               "4) 若CSV与当前冻结Physical17结果匹配，则保留该结果；不匹配则仅取消本次冻结结果并要求重新阶段2求解。\n\n"
               "确定导入吗？"),
            QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
        if (answer != QMessageBox::Yes) return;
    }

    const QString fn=QFileDialog::getOpenFileName(this,tr("导入阶段2/旧版混合CSV"),QString(),tr("CSV (*.csv)"));
    if(fn.isEmpty())return;
    std::ifstream in(fn.toStdString());
    std::vector<SphereProfileObservation> obs;
    std::string error;
    if(!SingleSphere17Calibrator::readObservationCsv(in,obs,error)){
        QMessageBox::warning(this,tr("导入失败"),QString::fromStdString(error));
        return;
    }

    // V27.6.15: when a valid Stage-2 JSON is already loaded offline, importing its
    // training CSV is normally provenance recovery for hold-out separation, not a
    // request to throw the frozen model away.  Preserve it temporarily and only
    // restore it if the imported training set reproduces the stored full-data
    // residual/count signature.  A mismatched CSV therefore cannot silently bind
    // itself to an unrelated Physical17 result.
    const bool hadFrozenOfflineResult = m_hasResult && m_lastResult.success;
    const auto frozenOfflineResult = m_lastResult;
    const auto frozenValidationSummary = m_validationSummary;

    // If a validated production model was bound, keep it as a separate fallback
    // before the imported dataset replaces the current Stage-2 working result.
    if (wasLiveBound && m_hasResult && m_lastResult.success && m_validationSummary.passed) {
        preserveAcceptedModelAsFallbackForRecalibration();
    }

    // Preserve Stage-1 provenance/convention regardless of whether the previous
    // validated model was offline or auto-bound to live hardware.  This is required
    // to filter Stage-1 profiles from legacy mixed CSVs and is also useful for later
    // rebinding after an offline Stage-2 re-solve.
    const bool preserveBootstrap = m_hasBootstrap && m_bootstrapResult.success;
    const auto bootstrap = m_bootstrapResult;
    const auto loadedStage1Poses = m_loadedStage1CapturedPoses;
    const int loadedStage1RecordCount = m_bootstrapRecordCount;
    const qint64 loadedStage1Elapsed = m_stage1SolveElapsedMs;
    const bool savedFlipX = m_referenceFlipX;
    const bool savedFlipZ = m_referenceFlipZ;
    const std::uint8_t savedFeedbackMask = m_referenceFeedbackMask;
    const std::uint8_t savedModelFeedbackMask = m_modelFeedbackMask;

    m_records.clear();
    m_validationRecords.clear();
    m_validationSummary=ValidationSummary{};
    m_plan.clear();
    m_planTable->setRowCount(0);
    m_referenceTaught=false;
    m_referenceCaptured=false;
    m_sensorConventionLocked=false;
    // Devices may remain physically connected, but imported CSV data must never be
    // mistaken for a live motion session.  Explicitly drop the persistent live bind;
    // the user can bind a matching/final result again after offline processing.
    // V27.6.42B also drops the in-RAM "persistent model is current" flag here.
    // Otherwise a later device-state refresh could call tryBindPersistentCalibration()
    // after the NEW CSV solve and accidentally attach the OLD persisted hold-out verdict
    // to the newly solved parameter vector.  The accepted production model is already
    // preserved transactionally in m_fallbackResult and remains on disk.
    m_persistentCalibrationBound=false;
    m_persistentCalibrationLoaded=false;
    m_persistedValidationSummary=ValidationSummary{};
    m_persistentBindError.clear();
    m_hasResult=false;
    m_fullPlanGenerated=false;
    m_planValidation=false;
    m_refinementSolved=false;
    m_planBootstrap=false;
    m_calibrator=SingleSphere17Calibrator(solverOptionsFromUi());
    for(const auto& o:obs)m_calibrator.addObservation(o);

    if (preserveBootstrap) {
        m_bootstrapResult=bootstrap;
        m_hasBootstrap=true;
        m_loadedStage1CapturedPoses=loadedStage1Poses;
        m_stage1SolveElapsedMs=loadedStage1Elapsed;
        m_bootstrapRecordCount=loadedStage1RecordCount;
        m_referenceFlipX=savedFlipX;
        m_referenceFlipZ=savedFlipZ;
        m_referenceFeedbackMask=savedFeedbackMask;
        m_modelFeedbackMask=savedModelFeedbackMask;
        // Preserve Stage-1 as the offline planning/observability estimate.  The
        // V27.6.41 final solve itself will ignore this as a forced numerical seed and
        // re-bootstrap hand-eye+sphere from the imported Stage-2 training observations.
        m_currentEstimate=bootstrap.parameters;
        m_sphereCenterGuessMachine=bootstrap.sphereCenterMachineMm;
    } else {
        m_hasBootstrap=false;
        m_bootstrapRecordCount=0;
        m_bootstrapResult=NominalHandEyeSphereResult{};
        m_loadedStage1CapturedPoses.clear();
        if(!obs.empty()) m_sphereCenterGuessMachine={0,0,95};
    }

    bool restoredFrozenResult = false;
    QString frozenRestoreDetail;
    if (hadFrozenOfflineResult) {
        const auto trainingForCheck = stage2TrainingObservations();
        const auto report = SingleSphere17Calibrator::validateModel(
            trainingForCheck, frozenOfflineResult.parameters,
            frozenOfflineResult.sphereCenterMachineMm, m_spinSphereDiameter->value());
        const double rmsTol = std::max(1e-8, std::abs(frozenOfflineResult.residualRmsMm) * 1e-5);
        const double maxTol = std::max(1e-8, std::abs(frozenOfflineResult.residualMaxAbsMm) * 1e-5);
        const bool sameSignature = report.valid
            && report.profileCount == frozenOfflineResult.profileCount
            && report.pointCount == frozenOfflineResult.pointCount
            && std::abs(report.residualRmsMm - frozenOfflineResult.residualRmsMm) <= rmsTol
            && std::abs(report.residualMaxAbsMm - frozenOfflineResult.residualMaxAbsMm) <= maxTol;
        if (sameSignature) {
            m_lastResult = frozenOfflineResult;
            m_hasResult = true;
            m_modelFeedbackMask = savedModelFeedbackMask != 0 ? savedModelFeedbackMask : savedFeedbackMask;
            m_refinementSolved = true;
            m_currentEstimate = frozenOfflineResult.parameters;
            m_sphereCenterGuessMachine = frozenOfflineResult.sphereCenterMachineMm;
            m_validationSummary = frozenValidationSummary;
            restoredFrozenResult = true;
            frozenRestoreDetail = tr("已用profile/point计数以及全训练集RMS/max残差复核该CSV确实对应当前Stage2结果；冻结的17参数结果已保留，可直接导入hold-out CSV做离线验证。");
        } else {
            frozenRestoreDetail = tr("该CSV没有复现当前Stage2 JSON的训练数据签名，因此为避免把错误训练集与17参数绑定，已取消原Stage2冻结结果；如确认CSV正确，请重新执行阶段2最终求解。");
        }
    }

    updateSessionSummary();
    updateValidationSummary();
    updateCalibrationReadiness();
    updateWorkflowGuide();
    const auto training=stage2TrainingObservations();
    std::size_t trainingPoints=0; for(const auto& o:training)trainingPoints+=o.points.size();
    if (wasLiveBound) {
        appendLog(tr("已从实机绑定模式切换到离线阶段2CSV模式：控制器/传感器连接保持，但当前标定运动会话已解除绑定。"
                     "阶段2求解通过后可以直接离线生成独立验证姿态；启动自动验证采集时，程序会先严格核对保存的FlipX/FlipZ和XYZCB DPOS/MPOS来源，再重新绑定本次新Physical17用于验证运动。"));
    }
    appendLog(tr("已导入CSV：原始=%1 profiles；当前识别为阶段2训练=%2 profiles / %3点。%4%5")
        .arg(static_cast<qulonglong>(obs.size()))
        .arg(static_cast<qulonglong>(training.size()))
        .arg(static_cast<qulonglong>(trainingPoints))
        .arg(m_hasBootstrap
            ? tr("阶段1JSON已加载，因此旧V27.6.10A混合CSV中的阶段1姿态可自动排除；V27.6.42B最终求解仍会从剩余阶段2训练数据重新自举hand-eye和平移规范锚点。")
            : tr("V27.6.42B可直接离线最终求解：如果这是Stage2-only训练CSV，程序会先从当前CSV自举9状态hand-eye+球心，再以自举Tx/Ty/Tz作为translation-gauge anchor进行Physical17 rank-14优化，无需为了求解额外加载阶段1 JSON。若文件是旧V27.6.10A阶段1+阶段2混合CSV，则仍应先加载对应阶段1 JSON，以便可靠剔除阶段1 profiles。"))
        .arg(hadFrozenOfflineResult ? QStringLiteral(" ") + frozenRestoreDetail : QString()));

    if (restoredFrozenResult) {
        updateResultTable(&m_lastResult);
        updateValidationSummary();
        updateCalibrationReadiness();
        updateWorkflowGuide();
    }
}

void SingleSphere14CalibrationWidget::markPlanStatus(int index,const QString& status,const QString& quality)
{
    if(index<0||index>=m_planTable->rowCount())return;
    auto* item=m_planTable->item(index,9); if(!item){item=new QTableWidgetItem;m_planTable->setItem(index,9,item);} item->setText(quality.isEmpty()?status:status+QStringLiteral("；")+quality);
}

void SingleSphere14CalibrationWidget::appendLog(const QString& text)
{
    emit logMessage(text);
}

void SingleSphere14CalibrationWidget::notifyRuntimeCalibrationUsability()
{
    std::array<double,17> parameters{};
    const bool usable=scanCalibration(parameters);
    if(!m_runtimeUsableSignalInitialized || usable!=m_lastRuntimeCalibrationUsable){
        m_runtimeUsableSignalInitialized=true;
        m_lastRuntimeCalibrationUsable=usable;
        emit runtimeCalibrationUsableChanged(usable);
    }
}

bool SingleSphere14CalibrationWidget::scanCalibration(std::array<double,17>& parameters) const
{
    // Prefer the newly solved/validated model. During a recalibration session keep the
    // last independently validated model available as a runtime fallback; changing
    // XYZ from DPOS to homed grating MPOS is not treated as automatic invalidation.
    if(!m_referenceTaught) return false;
    QString conventionError;
    if(!validateSessionConventions(conventionError)) return false;
    if(m_hasResult && m_lastResult.success && m_validationSummary.passed){
        parameters=m_lastResult.parameters.value;
        return true;
    }
    if(m_hasFallbackResult && m_fallbackResult.success && m_fallbackValidationSummary.passed){
        parameters=m_fallbackResult.parameters.value;
        return true;
    }
    return false;
}

bool SingleSphere14CalibrationWidget::sphereStudyPlanningCalibration(
    std::array<double,17>& parameters, QString& source, bool& independentlyValidated) const
{
    independentlyValidated = false;

    // For experimental path generation an approximate prior is useful even when a
    // new MPOS calibration has not yet completed. Keep this deliberately separate
    // from scanCalibration(), whose stronger gate still protects formal scan/RTCP.
    // Prefer an independently validated prior over a newer but unfinished solve: it
    // is a safer navigation seed for the sphere distance/angle experiment.
    if (m_hasResult && m_lastResult.success && m_validationSummary.passed) {
        parameters = m_lastResult.parameters.value;
        independentlyValidated = true;
        source = tr("当前Physical17（独立验证已通过，模型feedback mask=%1）").arg(m_modelFeedbackMask);
        return true;
    }
    if (m_hasFallbackResult && m_fallbackResult.success && m_fallbackValidationSummary.passed) {
        parameters = m_fallbackResult.parameters.value;
        independentlyValidated = true;
        source = tr("上一套Physical17兼容模型（独立验证已通过，作为研究路径初值）");
        return true;
    }
    if (m_hasResult && m_lastResult.success) {
        parameters = m_lastResult.parameters.value;
        source = tr("当前Physical17求解结果（仅作研究路径初值，尚未通过独立验证，模型feedback mask=%1）")
                     .arg(m_modelFeedbackMask);
        return true;
    }
    if (m_hasFallbackResult && m_fallbackResult.success) {
        parameters = m_fallbackResult.parameters.value;
        source = tr("上一套Physical17兼容模型（未验证，仅作研究路径初值）");
        return true;
    }
    if (m_hasBootstrap && m_bootstrapResult.success) {
        parameters = m_bootstrapResult.parameters.value;
        source = tr("阶段1初步手眼+名义机床模型（仅作研究路径初值）");
        return true;
    }

    source = tr("没有可用的Physical17/阶段1初始模型");
    return false;
}

bool SingleSphere14CalibrationWidget::sphereStudySensorCoordinateConvention(
    bool& flipX, bool& flipZ, QString& source) const
{
    // All calibration paths that can provide a sphere-study model also carry
    // the sensor coordinate convention in m_referenceFlipX/Z.  For persisted
    // accepted calibration this is restored from coordinate_convention;
    // Stage-1 JSON restores the same pair even when the device is offline.
    if (m_hasResult || m_hasFallbackResult || (m_hasBootstrap && m_bootstrapResult.success)
        || m_sensorConventionLocked) {
        flipX = m_referenceFlipX;
        flipZ = m_referenceFlipZ;
        source = tr("标定模型保存的坐标约定：FlipX=%1 FlipZ=%2")
                     .arg(flipX).arg(flipZ);
        return true;
    }

    // If no saved calibration convention exists but the sensor is connected,
    // use the live processing flags.  This branch is mainly for an early
    // Stage-1 workflow; normal offline planning uses the saved convention.
    if (m_sensor && m_sensor->isConnected()) {
        SensorAdvancedParams adv;
        QString error;
        if (m_sensor->readAdvancedParams(adv, &error)) {
            flipX = adv.flipX;
            flipZ = adv.flipZ;
            source = tr("2950实时坐标约定：FlipX=%1 FlipZ=%2")
                         .arg(flipX).arg(flipZ);
            return true;
        }
    }

    source = tr("没有可用的FlipX/FlipZ坐标约定");
    return false;
}
