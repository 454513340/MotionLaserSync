#include "app/MainWindow.h"

#include "core/ZmcController.h"
#include "core/LaserSensor.h"
#include "core/SyncManager.h"
#include "widgets/PointCloudWidget.h"
#include "gcode/GCodeExecutor.h"
#include "calibration/SingleSphere17Calibration.h"
#include "trajectory/JointTrajectoryPlanner5Axis.h"
#include "utils/LogManager.h"

#include <QCheckBox>
#include <QComboBox>
#include <QColor>
#include <QDataStream>
#include <QDateTime>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFileInfo>
#include <QVector>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QMetaObject>
#include <QPushButton>
#include <QRadioButton>
#include <QSpinBox>
#include <QVector3D>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QTextStream>
#include <QTimer>
#include <QtConcurrent>
#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <vector>

namespace {
QString fmtPose(const JointPose5D& p) {
    return QStringLiteral("XYZ=[%1,%2,%3] C/B=[%4,%5]")
        .arg(p.x,0,'f',4).arg(p.y,0,'f',4).arg(p.z,0,'f',4)
        .arg(p.c,0,'f',4).arg(p.b,0,'f',4);
}

constexpr double kNonXLinearToleranceMm = 0.005;    // 5 µm: feedback quantization/settling
constexpr double kNonXRotaryToleranceDeg = 0.005;  // 0.005 degrees; still fail-closed
bool lockedAxesMatch(const JointPose5D& actual,const JointPose5D& planned){
    return std::abs(actual.y-planned.y) <= kNonXLinearToleranceMm
        && std::abs(actual.z-planned.z) <= kNonXLinearToleranceMm
        && std::abs(actual.c-planned.c) <= kNonXRotaryToleranceDeg
        && std::abs(actual.b-planned.b) <= kNonXRotaryToleranceDeg;
}
QString lockedAxisDeltaText(const JointPose5D& actual,const JointPose5D& planned){
    return QStringLiteral("ΔY=%1mm ΔZ=%2mm ΔC=%3° ΔB=%4° (limits: XY/Z=%5mm C/B=%6°)")
       .arg(actual.y-planned.y,0,'f',6).arg(actual.z-planned.z,0,'f',6)
       .arg(actual.c-planned.c,0,'f',6).arg(actual.b-planned.b,0,'f',6)
       .arg(kNonXLinearToleranceMm,0,'f',3).arg(kNonXRotaryToleranceDeg,0,'f',3);
}

QJsonObject toJson(const mls::calibration::SphereExposurePass& p) {
    QJsonObject o;
    o["exposure10us"]=p.exposure10us; o["rateHz"]=p.rateHz;
    o["repeat"]=p.repeat; o["direction"]=p.direction;
    o["speedMmPerSec"]=p.speedMmPerSec;
    o["nominalSpacingMm"]=p.nominalSpacingMm;
    o["frames"]=static_cast<qint64>(p.frames);
    o["acceptedFrames"]=static_cast<qint64>(p.acceptedFrames);
    o["observedMaxDiameterMm"]=p.observedMaxDiameterMm;
    o["fittedPeakDiameterMm"]=p.fittedPeakDiameterMm;
    o["medianArcRmsMm"]=p.medianArcRmsMm;
    o["medianArcSpanDeg"]=p.medianArcSpanDeg;
    o["validFraction"]=p.validFraction;
    o["signalFrames"]=static_cast<qint64>(p.signalFrames);
    o["medianMaximum"]=p.medianMaximum;
    o["medianWidth"]=p.medianWidth;
    o["medianMaximumPlateauFraction"]=p.medianMaximumPlateauFraction;
    o["peakInterpolated"]=p.peakInterpolated;
    o["complete"]=p.complete;
    o["note"]=QString::fromStdString(p.note);
    return o;
}

bool saveJsonFile(const QString& path,const QJsonObject& obj,QString* err) {
    QFile file(path);
    if(!file.open(QIODevice::WriteOnly|QIODevice::Truncate)) {
        if(err)*err=file.errorString();return false;
    }
    if(file.write(QJsonDocument(obj).toJson(QJsonDocument::Indented))<0){
        if(err)*err=file.errorString();return false;
    }
    return file.flush();
}
} // namespace

void MainWindow::sphereExposureSetStatus(const QString& text,bool warning) {
    if(m_lblSphereExpStatus){
        m_lblSphereExpStatus->setText(text);
        m_lblSphereExpStatus->setStyleSheet(warning ? QStringLiteral("QLabel{color:#a33;font-weight:bold;}")
            : QStringLiteral("QLabel{color:#245;}") );
    }
    if(warning)LOG_WARNING(LogCategory::Calibration,tr("[全标准球曝光][REJECT/FAULT] %1").arg(text));
    else LOG_INFO(LogCategory::Calibration,tr("[全标准球曝光] %1").arg(text));
}

void MainWindow::sphereExposurePopulatePlanPreview() {
    if(!m_tableSphereExpPlan)return;
    m_tableSphereExpPlan->setRowCount(0);
    if(!m_sphereExpPlanValid)return;
    const int expStart=m_spinExpStart->value();
    const int expEnd=m_spinExpEnd->value();
    const int expStep=m_spinExpStep->value();
    const int repeats=m_spinSphereExpRepeats->value();
    if(expStart>expEnd || expStep<=0 || repeats<1)return;
    const int levels=(expEnd-expStart)/expStep+1;
    if(levels<1 || levels>40)return;
    m_tableSphereExpPlan->setRowCount(levels*repeats);
    const int rate=sensorCalibrationRequestedRateHz();
    const double spacing=m_spinSphereExpSpeed->value()/double(std::max(1,rate));
    for(int i=0;i<levels*repeats;++i){
        const int rep=i%repeats;
        const bool forward=(rep%2==0);
        const QStringList values={QString::number(i+1),
            QString::number(expStart+i/repeats*expStep),forward?"+X":"-X",
            QString::number(forward?m_sphereExpStartPose.x:m_sphereExpEndPose.x,'f',4),
            QString::number(forward?m_sphereExpEndPose.x:m_sphereExpStartPose.x,'f',4),
            QString::number(rate),QString::number(spacing,'f',4),tr("待执行")};
        for(int j=0;j<values.size();++j){
            auto* item=new QTableWidgetItem(values[j]);
            item->setFlags(item->flags() & ~Qt::ItemIsEditable);
            m_tableSphereExpPlan->setItem(i,j,item);
        }
    }
}

void MainWindow::sphereExposureSetPassPreviewState(int passIndex,const QString& state){
    if(!m_tableSphereExpPlan || passIndex<0 || passIndex>=m_tableSphereExpPlan->rowCount())return;
    if(auto* item=m_tableSphereExpPlan->item(passIndex,7))item->setText(state);
    m_tableSphereExpPlan->selectRow(passIndex);
}

bool MainWindow::sphereExposurePlanValid(QString* err) const {
    if(!m_sphereExpPlanValid || !m_sphereStudySphereFit.success
       || !m_sphereStudyHasPlanningModel || !m_sphereStudyPlanningModelValidated) {
        if(err)*err=tr("请先完成球点云拟合，加载已验证Physical17模型，并重新生成X扫描路径");
        return false;
    }
    if(!m_sphereExpStartPose.isFinite() || !m_sphereExpEndPose.isFinite()){
        if(err)*err=tr("路径端点无效");return false;
    }
    const double speed=m_spinSphereExpSpeed->value();
    const int rate=sensorCalibrationRequestedRateHz();
    if(!(speed>0.0) || rate<25 || speed/rate>m_spinSphereExpMaxStep->value()+1.0e-9) {
        if(err)*err=tr("速度/频率不满足最大帧间距要求：V/f=%1 mm，限制%2 mm")
            .arg(speed/double(std::max(1,rate)),0,'f',5)
            .arg(m_spinSphereExpMaxStep->value(),0,'f',5);
        return false;
    }
    return true;
}

void MainWindow::onPlanSphereExposureSweep() {
    m_sphereExpPlanValid=false;
    if(m_btnSphereExpStart)m_btnSphereExpStart->setEnabled(false);
    if(m_pointCloudView)m_pointCloudView->clearSphereExposureSweepPath();
    if(m_tableSphereExpPlan)m_tableSphereExpPlan->setRowCount(0);
    auto fail=[this](const QString& reason) {
        sphereExposureSetStatus(tr("路径规划拒绝：%1").arg(reason),true);
        QMessageBox::warning(this,tr("整球曝光路径规划失败"),reason);
    };
    if(m_sphereExpActive || m_small7Active || m_jointMoveInProgress || m_sphereStudyAutoActive
        || (m_syncManager&&m_syncManager->isRunning()) || m_expTestState!=ExpTestState::Idle)
        return fail(tr("其它运动、扫描或曝光自动测试尚未结束"));
    if(!m_sphereStudySphereFit.success)return fail(tr("尚未成功拟合球心：请先点击『导入球点云并拟合球心』"));
    QString error;
    if(!refreshSphereStudyPlanningModel(&error) || !m_sphereStudyPlanningModelValidated)
        return fail(tr("缺少已验证的完整Physical17/手眼标定：%1").arg(error));
    if(!m_zmc || !m_zmc->isConnected() || !m_zmc->areLinearMposReferenced()
       || !m_zmc->isRotaryAbsoluteReady(AxisC) || !m_zmc->isRotaryAbsoluteReady(AxisB))
        return fail(tr("控制器未连接或五轴机械坐标/XYZ MPOS 回零认证无效"));
    if(!sphereStudySafetyReady(&error))return fail(error);
    JointState5D s;
    if(!m_zmc->readJointState5D(s,&error) || !s.allIdle)
        return fail(error.isEmpty()?tr("五轴尚未停稳"):error);

    const auto center=m_sphereStudySphereFit.centerMachineMm;
    const auto toSensorY=[&](double x) {
        auto q=s.mpos; q.x=x;
        return mls::calibration::SingleSphere17Model::machinePointInSensor(
            {q.x,q.y,q.z,q.c,q.b},center,m_sphereStudyPlanningModel).y;
    };
    const double dy=0.5*(toSensorY(s.mpos.x+0.5)-toSensorY(s.mpos.x-0.5));
    const double here=toSensorY(s.mpos.x);
    if(!std::isfinite(dy) || std::abs(dy)<0.15 || !std::isfinite(here))
        return fail(tr("X轴几乎平行于激光测量平面；扫描X不足以跨越球截面，dSensorY/dMachineX=%1")
            .arg(dy,0,'f',6));
    const double radius=m_spinSphereStudyDiameter->value()/2.0;
    const double margin=m_spinSphereExpMargin->value();
    const double centerX=s.mpos.x-here/dy;
    const double halfspan=(radius+margin)/std::abs(dy);
    if(!(halfspan>0.5) || halfspan>28.0 || !std::isfinite(centerX))
        return fail(tr("Physical17规划出的X行程过大或无效：%1 mm（单侧）").arg(halfspan,0,'f',3));
    JointPose5D mid=s.mpos;mid.x=centerX;
    JointPose5D a=mid,b=mid;a.x-=halfspan;b.x+=halfspan;
    if(!checkSphereStudyPoseSafety(a,&error,false)||!checkSphereStudyPoseSafety(b,&error,false)
        ||!checkSphereStudyTransitionSafety(s.mpos,a,&error,false)
        ||!checkSphereStudyTransitionSafety(a,b,&error,false)
        ||!checkSphereStudyTransitionSafety(b,a,&error,false))
        return fail(tr("MachineSafety路径完整性检查不通过：%1").arg(error));
    const auto sensorCenter=mls::calibration::SingleSphere17Model::machinePointInSensor(
        {mid.x,mid.y,mid.z,mid.c,mid.b},center,m_sphereStudyPlanningModel);
    if(!std::isfinite(sensorCenter.x)||!std::isfinite(sensorCenter.z))
        return fail(tr("手眼变换到传感器测量平面失败"));
    m_sphereExpStartPose=a;m_sphereExpEndPose=b;m_sphereExpMiddlePose=mid;
    m_sphereExpPredCx=sensorCenter.x;m_sphereExpPredCz=sensorCenter.z;
    m_sphereExpPhysical17Gradient=dy;
    m_sphereExpPlanValid=true;
    QString spacingError;
    if (!sphereExposurePlanValid(&spacingError)) {
        m_sphereExpPlanValid=false;
        return fail(spacingError);
    }
    sphereExposurePopulatePlanPreview();
    if(m_btnSphereExpStart)m_btnSphereExpStart->setEnabled(true);
    if(m_pointCloudView){
        QVector<QVector3D> trace;
        trace.reserve(41);
        for(int k=0;k<=40;++k){
            const double t=k/40.0;
            JointPose5D pose=mid;
            pose.x=a.x+(b.x-a.x)*t;
            const auto tf=mls::calibration::SingleSphere17Model::sensorToMachineTransform(
                {pose.x,pose.y,pose.z,pose.c,pose.b},m_sphereStudyPlanningModel);
            // A fixed point on the laser plane, not the carriage tool-tip path.
            trace.append(QVector3D(float(tf[0]*sensorCenter.x+tf[2]*sensorCenter.z+tf[3]),
                float(tf[4]*sensorCenter.x+tf[6]*sensorCenter.z+tf[7]),
                float(tf[8]*sensorCenter.x+tf[10]*sensorCenter.z+tf[11])));
        }
        m_pointCloudView->setSphereExposureSweepPath(trace);
        if(m_viewTabs)m_viewTabs->setCurrentWidget(m_pointCloudView);
    }
    LOG_INFO(LogCategory::Calibration,tr("[全标准球曝光][PATH-PREVIEW] 已生成%1条X扫描；XYZ路径端点 %2 → %3；"
         "3D青线为传感器激光平面参考点轨迹（不代表已执行），Y/Z/C/B保持规划值，未驱动机床。")
       .arg(m_tableSphereExpPlan?m_tableSphereExpPlan->rowCount():0)
       .arg(fmtPose(a)).arg(fmtPose(b)));
    sphereExposureSetStatus(tr("规划通过（尚未运动）：起点 %1 → 终点 %2；球心截面X≈%3，扫描X行程%4 mm；Sensor圆心XZ≈[%5,%6]；"
                               "V=%7mm/s、f=%8Hz、Δx≤%9mm。执行前将再次核查碰撞安全。")
        .arg(fmtPose(a)).arg(fmtPose(b)).arg(centerX,0,'f',4)
        .arg(2*halfspan,0,'f',3).arg(sensorCenter.x,0,'f',3).arg(sensorCenter.z,0,'f',3)
        .arg(m_spinSphereExpSpeed->value(),0,'f',3).arg(sensorCalibrationRequestedRateHz())
        .arg(m_spinSphereExpSpeed->value()/sensorCalibrationRequestedRateHz(),0,'f',4));
}

void MainWindow::onStartSphereExposureSweep() {
    if(m_sphereExpActive || m_small7Active) return;
    QString err;
    if(!sphereExposurePlanValid(&err))return sphereExposureSetStatus(err,true);
    if(!m_laser||!m_laser->isConnected()||!m_zmc||!m_zmc->isConnected())
        return sphereExposureSetStatus(tr("传感器和控制器必须同时连接"),true);
    if(m_laser->isVideoRunning()||m_laser->isProfileRunning()||m_profilePreviewRunning||m_sphereStudyCaptureActive
       ||m_sphereStudyAutoActive||m_jointMoveInProgress||m_axisWorkerCount>0
       ||(m_syncManager&&m_syncManager->isRunning())||m_expTestState!=ExpTestState::Idle
       ||(m_gcodeExecutor&&(m_gcodeExecutor->isRunning()||m_gcodeExecutor->isPaused())))
        return sphereExposureSetStatus(tr("存在其它运动、GCode、预览或采集任务"),true);
    if(!m_zmc->isRotaryServoEnabled(AxisC)||!m_zmc->isRotaryServoEnabled(AxisB))
        return sphereExposureSetStatus(tr("B/C未人工使能，禁止X轴自动扫球"),true);
    JointState5D state;
    if(!m_zmc->readJointState5D(state,&err)||!state.allIdle)
        return sphereExposureSetStatus(tr("机床未停稳或五轴MPOS读取失败：%1").arg(err),true);
    if(!lockedAxesMatch(state.mpos,m_sphereExpStartPose))
        return sphereExposureSetStatus(tr("规划后非X轴位置变化，拒绝执行：%1；请检查Y/Z/C/B机械或伺服状态并重新规划")
            .arg(lockedAxisDeltaText(state.mpos,m_sphereExpStartPose)),true);
    if(!checkSphereStudyTransitionSafety(state.mpos,m_sphereExpStartPose,&err,false)
       ||!checkSphereStudyTransitionSafety(state.mpos,m_sphereExpEndPose,&err,false)
       ||!checkSphereStudyTransitionSafety(m_sphereExpStartPose,m_sphereExpEndPose,&err,false))
        return sphereExposureSetStatus(tr("执行前安全复核不通过：%1").arg(err),true);
    if(!validateSensorCalibrationTiming(m_spinExpEnd->value(),sensorCalibrationRequestedRateHz(),&err))
        return sphereExposureSetStatus(tr("频率/曝光组合不合法：%1").arg(err),true);
    if(m_spinExpStart->value()>m_spinExpEnd->value())
        return sphereExposureSetStatus(tr("曝光起点必须不大于终点"),true);
    const int levels=(m_spinExpEnd->value()-m_spinExpStart->value())/m_spinExpStep->value()+1;
    const int repeats=m_spinSphereExpRepeats->value();
    if(levels<1||levels>40)return sphereExposureSetStatus(tr("单次最多允许40个曝光等级"),true);
    const QString base=m_editExpTestDir->text().trimmed();
    if(base.isEmpty())return sphereExposureSetStatus(tr("RAW归档根目录为空"),true);
    const double speed=m_spinSphereExpSpeed->value();
    if(!(m_axisParams[AxisX].speed>=speed))
        return sphereExposureSetStatus(tr("当前X轴速度上限%1mm/s低于请求%2mm/s；禁止启动")
            .arg(m_axisParams[AxisX].speed).arg(speed),true);
    const double length=std::abs(m_sphereExpEndPose.x-m_sphereExpStartPose.x);
    const auto approxSec=double(levels*repeats)*(length/speed+5.0);
    if(QMessageBox::question(this,tr("确认自动全标准球曝光扫描"),
        tr("将按软件内部触发在选定的测量场(index=%7)执行 %1 档曝光×%2次=%3条X轴扫描。\n"
           "预计约%4分钟。严格锁定 Y/Z/C/B。\n"
           "每条扫描都保存原始X/Z Profile与圆弧指标，不会开启H1/H3硬件锁存或时间补偿。\n"
           "机床将沿 %5 与 %6 之间移动。\n\n确认机床及工件安全后继续？")
           .arg(levels).arg(repeats).arg(levels*repeats).arg(approxSec/60.0,0,'f',1)
           .arg(m_sphereExpStartPose.x,0,'f',3).arg(m_sphereExpEndPose.x,0,'f',3)
           .arg(sensorCalibrationSelectedFieldIndex()),
        QMessageBox::Yes|QMessageBox::No,QMessageBox::No)!=QMessageBox::Yes)return;

    const QString dir=QDir(base).filePath("SphereExposureSweep/SEXP_"+
       QDateTime::currentDateTimeUtc().toString("yyyyMMdd_HHmmss_zzz"));
    if(!QDir().mkpath(dir))return sphereExposureSetStatus(tr("无法创建归档目录：%1").arg(dir),true);
    // The calibration sweep explicitly requests SoftwareInternal for its own
    // sensor configuration. Do not change the operator's formal H1/H3 sync
    // selection: the next formal scan must retain its previous UI mode.
    if(!enterSensorCalibrationMeasuringField(&err))
        return sphereExposureSetStatus(tr("无法启用所选测量场：%1").arg(err),true);

    m_sphereExpSessionDir=dir;
    m_sphereExpRateHz=sensorCalibrationRequestedRateHz();
    m_sphereExpRepeatCount=repeats;
    m_sphereExpExposureStart=m_spinExpStart->value();
    m_sphereExpExposureStep=m_spinExpStep->value();
    m_sphereExpFeedMmPerSec=speed;
    m_sphereExpKnownDiameterMm=m_spinSphereStudyDiameter->value();
    m_sphereExpTotalPasses=levels*repeats;
    sphereExposurePopulatePlanPreview();
    m_sphereExpPass=0;
    m_sphereExpPassResults.clear();
    if(m_tableSphereExpRanking)m_tableSphereExpRanking->setRowCount(0);
    m_sphereExpActive=true; m_sphereExpStopRequested=false;
    m_sphereExpMotionWorker=false; m_sphereExpCapturing=false;
    ++m_sphereExpGeneration;
    setJointMoveUiRunning(true);
    m_btnSphereExpStart->setEnabled(false);
    m_btnSphereExpStop->setEnabled(true);
    m_btnSphereExpPlan->setEnabled(false);
    m_btnSphereExpImport->setEnabled(false);
    m_spinSphereExpSpeed->setEnabled(false);
    if(m_comboSphereExpPositionSpeed)m_comboSphereExpPositionSpeed->setEnabled(false);
    m_spinSphereExpMaxStep->setEnabled(false);
    m_spinSphereExpMargin->setEnabled(false);
    m_spinSphereExpRepeats->setEnabled(false);
    m_spinSphereStudyDiameter->setEnabled(false);
    if(m_spinSensorCalibRate)m_spinSensorCalibRate->setEnabled(false);
    if(m_comboResolution)m_comboResolution->setEnabled(false);
    // Freeze source exposure parameters until all configured passes are finished.
    // The existing fixed-pose exposure test remains a separate, unchanged workflow.
    // Keep the legacy fixed-point controls idle rather than sharing state.
    updateVideoControlState();
    m_sphereExpProfileConnection=connect(m_laser.get(),&LaserSensor::profileReceived,this,
        [this](const LaserProfile& p){onSphereExposureProfile(p);},Qt::QueuedConnection);
    QJsonObject session;
    session["schema"]="SphereExposureSweep.v1";
    session["trigger"]="sensor software internal";
    session["measuringFieldIndex"]=m_sensorCalibActiveFieldIndex;
    session["rateHz"]=m_sphereExpRateHz;
    session["knownSphereDiameterMm"]=m_sphereExpKnownDiameterMm;
    session["referenceCloudPath"]=m_editSphereStudyCloud ? m_editSphereStudyCloud->text() : QString();
    QJsonArray physical17Params;
    for (double v : m_sphereStudyPlanningModel.value) physical17Params.append(v);
    session["physical17Parameters"]=physical17Params;
    session["lighting"]=m_comboLighting->currentText();
    session["material"]=m_editMaterial->text();
    session["temperatureC"]=m_spinTemperature->value();
    session["sphereCenterX"]=m_sphereStudySphereFit.centerMachineMm.x;
    session["sphereCenterY"]=m_sphereStudySphereFit.centerMachineMm.y;
    session["sphereCenterZ"]=m_sphereStudySphereFit.centerMachineMm.z;
    session["modelValidated"]=m_sphereStudyPlanningModelValidated;
    session["physical17ModelSource"]=m_sphereStudyPlanningModelSource;
    session["planStartX"]=m_sphereExpStartPose.x;session["planEndX"]=m_sphereExpEndPose.x;
    session["fixedY"]=m_sphereExpStartPose.y;session["fixedZ"]=m_sphereExpStartPose.z;
    session["fixedC"]=m_sphereExpStartPose.c;session["fixedB"]=m_sphereExpStartPose.b;
    session["marginMm"]=m_spinSphereExpMargin->value();
    session["speedMmPerSec"]=speed;
    session["positioningSpeedMmPerSec"]=m_comboSphereExpPositionSpeed
        ? m_comboSphereExpPositionSpeed->currentData().toDouble():3.0;
    session["nominalStepMm"]=speed/m_sphereExpRateHz;
    session["maxAllowedStepMm"]=m_spinSphereExpMaxStep->value();
    session["exposureStart10us"]=m_sphereExpExposureStart;
    session["exposureEnd10us"]=m_spinExpEnd->value();
    session["exposureStep10us"]=m_sphereExpExposureStep;
    session["repeatsPerExposure"]=repeats;
    session["note"]="Do not interpret observed max slice diameter as traceable sphere diameter without uncertainty assessment";
    if(!saveJsonFile(QDir(dir).filePath("session.json"),session,&err)){
        sphereExposureFinish(false,tr("无法写入Session清单：%1").arg(err));return;
    }
    sphereExposureSetStatus(tr("开始软件触发全标准球曝光扫描，共%1条。RAW: %2")
       .arg(m_sphereExpTotalPasses).arg(dir));
    QTimer::singleShot(0,this,[this]{sphereExposureAdvance();});
}

void MainWindow::sphereExposureAdvance() {
    if(!m_sphereExpActive)return;
    if(m_sphereExpStopRequested)return sphereExposureFinish(false,tr("用户停止"));
    if(m_sphereExpPass>=m_sphereExpTotalPasses)return sphereExposureFinish(true,tr("完成所有曝光扫描"));
    sphereExposureStartMove(true);
}

void MainWindow::sphereExposureStartMove(bool preposition) {
    if(!m_sphereExpActive||m_sphereExpMotionWorker)return;
    if(sensorCalibrationRequestedRateHz()!=m_sphereExpRateHz)
        return sphereExposureFinish(false,tr("采集期间传感器频率被改变，拒绝继续自动曝光扫描"));
    const int repeat=m_sphereExpPass % m_sphereExpRepeatCount;
    const bool forward=(repeat%2==0);
    JointPose5D target=preposition ? (forward ? m_sphereExpStartPose:m_sphereExpEndPose)
                                            : (forward ? m_sphereExpEndPose:m_sphereExpStartPose);
    JointState5D current;QString err;
    if(!m_zmc->readJointState5D(current,&err)||!current.allIdle)
        return sphereExposureFinish(false,tr("移动前五轴状态不可靠：%1").arg(err));
    if(!lockedAxesMatch(current.mpos,m_sphereExpStartPose))
        return sphereExposureFinish(false,tr("非X轴位置保护触发：%1。禁止自动补偿移动Y/Z/C/B。")
             .arg(lockedAxisDeltaText(current.mpos,m_sphereExpStartPose)));
    // Only X has a commanded displacement. The other axes retain their
    // contemporaneous actual MPOS; never command their stored planning offsets.
    target.y=current.mpos.y;target.z=current.mpos.z;
    target.c=current.mpos.c;target.b=current.mpos.b;
    if(!checkSphereStudyPoseSafety(target,&err,false)
       ||!checkSphereStudyTransitionSafety(current.mpos,target,&err,false))
        return sphereExposureFinish(false,tr("MachineSafety移动前检查失败：%1").arg(err));
    const double xSpeed=m_sphereExpFeedMmPerSec;
    const double maxSpeed=m_axisParams[AxisX].speed;
    if(!preposition && (!(maxSpeed>=xSpeed)||!(xSpeed>0)))
        return sphereExposureFinish(false,tr("X轴当前允许速度%1mm/s，小于请求扫球速度%2mm/s")
            .arg(maxSpeed).arg(xSpeed));
    // Exposure-test motion is a TRUE X-only Single_MoveAbs.  Never send
    // five-axis MOVEABS, which can subtly reposition Y/Z/C/B despite unchanged
    // nominal values and then trip the stationary-axis interlock.
    AxisParams xMotion=m_axisParams[AxisX];
    // Positioning is explicitly independent from the 1mm/s (or requested) scan.
    // Do not silently scale acceleration by 20% during positioning.
    const double positionV=m_comboSphereExpPositionSpeed
        ? m_comboSphereExpPositionSpeed->currentData().toDouble():3.0;
    if(preposition && positionV>maxSpeed+1.0e-6)
        return sphereExposureFinish(false,tr("X起点定位请求%1mm/s高于当前X轴认证上限%2mm/s")
                   .arg(positionV).arg(maxSpeed));
    xMotion.speed=static_cast<float>(preposition?positionV:xSpeed);
    xMotion.accel=m_axisParams[AxisX].accel;
    xMotion.decel=m_axisParams[AxisX].decel;
    xMotion.lspeed=0.0f;
    if(!preposition) {
        const int exp=m_sphereExpExposureStart+(m_sphereExpPass/m_sphereExpRepeatCount)*m_sphereExpExposureStep;
        const int idle=sensorCalibrationIdleUnitsForExposure(exp);
        SensorParams params;
        params.measuringFieldIndex=m_sensorCalibActiveFieldIndex;params.exposureTime=static_cast<unsigned>(exp);
        params.idleTime=static_cast<unsigned>(idle);
        if(m_comboResolution && m_comboResolution->count()>0)
            params.resolutionIndex=static_cast<unsigned>(m_comboResolution->currentIndex());
        if(!validateSensorCalibrationTiming(exp,m_sphereExpRateHz,&err)
           ||!m_laser->configure(SensorTriggerMode::SoftwareInternal,params,false,
                                HardwareTriggerType::EdgeRising,&err)
           ||!verifySensorCalibrationRuntimeConstraints(params,&err,true))
            return sphereExposureFinish(false,tr("软件触发/曝光设备回读失败：%1").arg(err));
        const QString prefix=QString("EXP_%1_R%2_%3")
            .arg(exp,4,10,QChar('0')).arg(repeat+1,2,10,QChar('0')).arg(forward?"PLUS":"MINUS");
        m_sphereExpCurrentPrefix=QDir(m_sphereExpSessionDir).filePath(prefix);
        m_sphereExpFrames.clear();m_sphereExpFrameGap=false;m_sphereExpHasProfileIndex=false;
        m_sphereExpDataFile=new QFile(m_sphereExpCurrentPrefix+".sxraw");
        m_sphereExpMetricsFile=new QFile(m_sphereExpCurrentPrefix+"_arcs.csv");
        if(!m_sphereExpDataFile->open(QIODevice::WriteOnly)
           || !m_sphereExpMetricsFile->open(QIODevice::WriteOnly|QIODevice::Text))
            return sphereExposureFinish(false,tr("无法打开RAW/圆弧输出文件：%1 / %2")
                    .arg(m_sphereExpDataFile->errorString(),m_sphereExpMetricsFile->errorString()));
        QDataStream stream(m_sphereExpDataFile);
        stream.setByteOrder(QDataStream::LittleEndian);
        stream.setFloatingPointPrecision(QDataStream::SinglePrecision);
        m_sphereExpDataFile->write("SXP1",4);
        stream<<quint32(1)<<qint32(exp)<<qint32(m_sphereExpRateHz)<<qint32(repeat)
              <<qint32(forward?1:-1)<<float(m_sphereExpFeedMmPerSec);
        QTextStream meta(m_sphereExpMetricsFile);
        meta<<"ProfileIndex,PcUs,Valid,CircleCxSensorMm,CircleCzSensorMm,RadiusMm,DiameterMm,ArcRmsMm,ArcSpanDeg,Inliers,RichMatched,MaximumMean,WidthMedian,MaximumPlateauFraction\n";
        // Request the same auxiliary peak/width channels used by the legacy
        // fixed-pose exposure test. Missing channels do not invalidate geometry.
        m_laser->setSensorCalibrationRichProfileEnabled(true);
        if(!m_laser->startContinuous(&err))
            return sphereExposureFinish(false,tr("软件轮廓采集启动失败：%1").arg(err));
        m_sphereExpCapturing=true;
        sphereExposureSetPassPreviewState(m_sphereExpPass,tr("采集中"));
        sphereExposureSetStatus(tr("[%1/%2] 软件触发扫球中：Exp=%3×10µs, Idle=%4, f=%5Hz, V=%6mm/s, 方向=%7")
            .arg(m_sphereExpPass+1).arg(m_sphereExpTotalPasses).arg(exp).arg(idle)
            .arg(m_sphereExpRateHz).arg(m_sphereExpFeedMmPerSec,0,'f',2)
            .arg(forward?"+X":"-X"));
    } else {
        sphereExposureSetPassPreviewState(m_sphereExpPass,tr("起点定位"));
        sphereExposureSetStatus(tr("[%1/%2] 定位扫描起点（仅X轴；不采集）：%3")
            .arg(m_sphereExpPass+1).arg(m_sphereExpTotalPasses).arg(fmtPose(target)));
    }
    m_sphereExpMotionWorker=true;
    ++m_axisWorkerCount;
    const quint64 generation=m_sphereExpGeneration;
    auto* controller=m_zmc.get();
    const double requestedX=target.x;
    (void)QtConcurrent::run([this,controller,generation,preposition,requestedX,xMotion]{
        JointMoveResult5D result;QString error;
        bool ok=controller->moveLinearMposAbs(AxisX,static_cast<float>(requestedX),&xMotion,&error);
        if(ok){
            ok=controller->readJointState5D(result.finalState,&error);
            if(ok){
                result.completed=result.finalState.allIdle;
                if(!result.completed)error=QStringLiteral("X轴完成后仍有轴未停稳");
                result.maxLinearMposErrorMm=std::abs(result.finalState.mpos.x-requestedX);
            }
        }
        QMetaObject::invokeMethod(this,[this,generation,preposition,ok,result,error]{
            --m_axisWorkerCount;
            m_sphereExpMotionWorker=false;
            if(generation!=m_sphereExpGeneration){
                if(m_sphereExpActive&&m_sphereExpStopRequested)sphereExposureFinish(false,tr("停止后运动线程退出"));
                tryFinishDeferredClose();return;
            }
            sphereExposureMoveFinished(generation,preposition,ok,result,error);
            tryFinishDeferredClose();
        },Qt::QueuedConnection);
    });
}

void MainWindow::sphereExposureMoveFinished(quint64 generation,bool preposition,
        bool success,const JointMoveResult5D& result,const QString& message) {
    if(generation!=m_sphereExpGeneration||!m_sphereExpActive)return;
    if(m_sphereExpStopRequested)return sphereExposureFinish(false,tr("收到用户停止请求"));
    if(!success || !result.completed)return sphereExposureFinish(false,
        tr("%1运动失败：%2").arg(preposition?tr("定位"):tr("扫描"),message));
    if(!lockedAxesMatch(result.finalState.mpos,m_sphereExpStartPose))
        return sphereExposureFinish(false,tr("完成X移动后非X轴位置保护触发：%1")
            .arg(lockedAxisDeltaText(result.finalState.mpos,m_sphereExpStartPose)));
    LOG_INFO(LogCategory::Calibration,tr("[全标准球曝光][X-ONLY-MOVE] %1完成，X实际=%2mm；%3")
        .arg(preposition?tr("起点定位"):tr("扫描"))
        .arg(result.finalState.mpos.x,0,'f',6)
        .arg(lockedAxisDeltaText(result.finalState.mpos,m_sphereExpStartPose)));
    QString error;
    if(!sphereStudyArrivalWithinTolerance(result,&error))
        return sphereExposureFinish(false,tr("到位检查失败：%1").arg(error));
    if(preposition){
        QTimer::singleShot(80,this,[this,generation]{
            if(generation==m_sphereExpGeneration&&m_sphereExpActive&&!m_sphereExpStopRequested)
                sphereExposureStartMove(false);
        });
    }else{
        // Stop acquisition BEFORE evaluating files or changing exposure.
        m_sphereExpCapturing=false;
        if(m_laser){m_laser->stopContinuous();m_laser->setSensorCalibrationRichProfileEnabled(false);}
        sphereExposureEndPass(!m_sphereExpFrameGap,
            m_sphereExpFrameGap?tr("Profile索引存在缺口，当前轨迹失效"):QString());
    }
}

void MainWindow::onSphereExposureProfile(const LaserProfile& p) {
    if(!m_sphereExpActive||!m_sphereExpCapturing||!m_sphereExpDataFile||!m_sphereExpMetricsFile)return;
    const quint32 delta=(p.index-m_sphereExpLastProfileIndex)&0x00ffffffu;
    if(m_sphereExpHasProfileIndex && delta!=1u) {
        m_sphereExpFrameGap=true;
        // Fail-closed: do not wait for the end of a long scan with a corrupt profile stream.
        QTimer::singleShot(0,this,[this]{
            if(m_sphereExpActive&&!m_sphereExpStopRequested)
                sphereExposureFinish(false,tr("软件触发Profile索引不连续，停止自动曝光扫描"));
        });
    }
    m_sphereExpHasProfileIndex=true;m_sphereExpLastProfileIndex=p.index;
    std::vector<mls::calibration::SphereExposurePoint> points;
    const int n=std::min(p.x.size(),p.z.size());
    const int expectedPoints = m_comboResolution && m_comboResolution->currentData().isValid()
        ? m_comboResolution->currentData().toInt() : 0;
    if (expectedPoints <= 0 || p.x.size() != expectedPoints || p.z.size() != expectedPoints) {
        QTimer::singleShot(0, this, [this] {
            if (m_sphereExpActive) sphereExposureFinish(false, tr("轮廓点数与所选分辨率不一致，终止本次扫描"));
        });
        return;
    }
    points.reserve(static_cast<std::size_t>(n));
    for(int i=0;i<n;++i) {
        // Keep the original samples in the RAW stream, even if invalid.
        points.push_back({p.x[i],p.z[i],i});
    }
    const double R=m_sphereExpKnownDiameterMm/2.0;
    const auto circle=mls::calibration::SphereExposureSweep::fitArc(
        points,m_sphereExpPredCx,m_sphereExpPredCz,R);
    mls::calibration::SphereExposureFrame frame;
    frame.index=p.index;frame.pcUs=static_cast<double>(p.tPcUs);frame.arc=circle;
    if(circle.valid){
        LaserSensor::SensorCalibrationProfileFrame rich;
        if(m_laser->latestSensorCalibrationProfile(rich)
            && rich.profile.callbackOrdinal==p.callbackOrdinal
            && rich.hasMaximum() && rich.maximum.size()==p.x.size()){
            std::vector<double> maximums,widths;
            const double expectedCx=m_sphereExpPredCx,expectedCz=m_sphereExpPredCz;
            for(int k=0;k<n;++k){
                if(!std::isfinite(p.x[k])||!std::isfinite(p.z[k]))continue;
                const double d=std::hypot(p.x[k]-expectedCx,p.z[k]-expectedCz);
                if(d>m_sphereExpKnownDiameterMm/2.0+0.8 || d<1.4)continue;
                maximums.push_back(rich.maximum[k]);
                if(rich.hasWidth()&&rich.width.size()==p.x.size())widths.push_back(rich.width[k]);
            }
            if(maximums.size()>=40){
                const double maxSignal=*std::max_element(maximums.begin(),maximums.end());
                int plateau=0;
                for(double v:maximums) if(v>=maxSignal-5.0)++plateau;
                std::sort(widths.begin(),widths.end());
                frame.signalAvailable=true;
                frame.maximumMean=std::accumulate(maximums.begin(),maximums.end(),0.0)/maximums.size();
                frame.widthMedian=widths.empty()?0.0:widths[widths.size()/2];
                frame.maximumPlateauFraction=double(plateau)/maximums.size();
            }
        }
    }
    m_sphereExpFrames.push_back(frame);
    QDataStream data(m_sphereExpDataFile);
    data.setByteOrder(QDataStream::LittleEndian);
    data.setFloatingPointPrecision(QDataStream::SinglePrecision);
    data<<quint32(p.index)<<qint64(p.tPcUs)<<quint32(n);
    for(int i=0;i<n;++i)data<<float(p.x[i])<<float(p.z[i]);
    QTextStream row(m_sphereExpMetricsFile);
    row<<p.index<<','<<p.tPcUs<<','<<int(circle.valid)<<','
       <<circle.centerX<<','<<circle.centerZ<<','<<circle.radius<<','
       <<(2*circle.radius)<<','<<circle.rms<<','<<circle.arcSpanDeg<<','
       <<static_cast<qulonglong>(circle.inliers)<<','<<int(frame.signalAvailable)<<','
       <<frame.maximumMean<<','<<frame.widthMedian<<','<<frame.maximumPlateauFraction<<'\n';
    if(m_sphereExpDataFile->error()!=QFileDevice::NoError
       ||m_sphereExpMetricsFile->error()!=QFileDevice::NoError) {
        QTimer::singleShot(0,this,[this]{
            if(m_sphereExpActive)sphereExposureFinish(false,tr("RAW归档写入失败，停止采集"));
        });
    }
    if(m_sphereExpFrames.size()>120000) {
        QTimer::singleShot(0,this,[this]{
            if(m_sphereExpActive)sphereExposureFinish(false,tr("单条扫描超过120000帧上限"));
        });
    }
}

void MainWindow::sphereExposureEndPass(bool success,const QString& error) {
    if(m_sphereExpDataFile){m_sphereExpDataFile->flush();m_sphereExpDataFile->close();
        delete m_sphereExpDataFile;m_sphereExpDataFile=nullptr;}
    if(m_sphereExpMetricsFile){m_sphereExpMetricsFile->flush();m_sphereExpMetricsFile->close();
        delete m_sphereExpMetricsFile;m_sphereExpMetricsFile=nullptr;}
    const int repeat=m_sphereExpPass%m_sphereExpRepeatCount;
    const int exposure=m_sphereExpExposureStart+(m_sphereExpPass/m_sphereExpRepeatCount)*m_sphereExpExposureStep;
    const int direction=(repeat%2==0)?1:-1;
    auto record=mls::calibration::SphereExposureSweep::summarize(
        m_sphereExpFrames,exposure,m_sphereExpRateHz,repeat+1,direction,
        m_sphereExpFeedMmPerSec,m_sphereExpKnownDiameterMm);
    // A truncated/failed pass cannot be ranked as a successful exposure.
    record.complete &= success;
    if(!success)record.note=error.toStdString();
    m_sphereExpPassResults.push_back(record);
    sphereExposureSetPassPreviewState(m_sphereExpPass,record.complete?tr("完成"):tr("数据不合格"));
    QString writeError;
    QJsonObject perPass=toJson(record);
    perPass["rawPath"]=m_sphereExpCurrentPrefix+".sxraw";
    perPass["arcsPath"]=m_sphereExpCurrentPrefix+"_arcs.csv";
    if(!saveJsonFile(m_sphereExpCurrentPrefix+"_summary.json",perPass,&writeError))
        return sphereExposureFinish(false,tr("无法保存每组指标：%1").arg(writeError));
    sphereExposureSetStatus(tr("[%1/%2] Exp=%3 R%4采集结束：Profile=%5，有效圆弧=%6，最大实测截面直径=%7mm，"
                               "抛物线峰值直径=%8mm，圆弧RMS中位=%9mm；%10")
        .arg(m_sphereExpPass+1).arg(m_sphereExpTotalPasses).arg(exposure).arg(repeat+1)
        .arg(record.frames).arg(record.acceptedFrames)
        .arg(record.observedMaxDiameterMm,0,'f',5).arg(record.fittedPeakDiameterMm,0,'f',5)
        .arg(record.medianArcRmsMm,0,'f',5)
        .arg(record.complete?tr("质量基本合格"):tr("质量不足")),!record.complete);
    if(!record.complete)return sphereExposureFinish(false,
        tr("曝光=%1 第%2次扫描圆弧覆盖/采集完整性不通过；停止后续自动扫描").arg(exposure).arg(repeat+1));
    ++m_sphereExpPass;
    m_sphereExpFrames.clear();
    QTimer::singleShot(160,this,[this]{sphereExposureAdvance();});
}

void MainWindow::onStopSphereExposureSweep() {
    if(!m_sphereExpActive)return;
    m_sphereExpStopRequested=true;
    m_sphereExpCapturing=false;
    if(m_laser){m_laser->stopContinuous();m_laser->setSensorCalibrationRichProfileEnabled(false);}
    if(m_zmc)m_zmc->requestStopAxis(AxisX,false);
    sphereExposureSetStatus(tr("已请求停止软件轮廓与X轴运动，等待运动任务退出"),true);
    if(!m_sphereExpMotionWorker)sphereExposureFinish(false,tr("操作者停止全标准球曝光测试"));
}

void MainWindow::sphereExposureFinish(bool success,const QString& reason) {
    if(!m_sphereExpActive)return;
    // A blocked vendor motion DLL call cannot be cancelled synchronously.
    // Do not unlock the UI / disconnect controller while it remains in-flight.
    if(m_sphereExpMotionWorker) {
        m_sphereExpStopRequested=true;
        m_sphereExpCapturing=false;
        if(m_laser){m_laser->stopContinuous();m_laser->setSensorCalibrationRichProfileEnabled(false);}
        if(m_zmc)m_zmc->requestStopAxis(AxisX,false);
        return;
    }
    m_sphereExpCapturing=false;
    if(!success)sphereExposureSetPassPreviewState(m_sphereExpPass,tr("中止/故障"));
    if(m_laser){m_laser->stopContinuous();m_laser->setSensorCalibrationRichProfileEnabled(false);}
    if(m_sphereExpProfileConnection){QObject::disconnect(m_sphereExpProfileConnection);m_sphereExpProfileConnection={};}
    if(m_sphereExpDataFile){m_sphereExpDataFile->close();delete m_sphereExpDataFile;m_sphereExpDataFile=nullptr;}
    if(m_sphereExpMetricsFile){m_sphereExpMetricsFile->close();delete m_sphereExpMetricsFile;m_sphereExpMetricsFile=nullptr;}
    leaveSensorCalibrationMeasuringField();
    const auto ranks=mls::calibration::SphereExposureSweep::rank(m_sphereExpPassResults,
         m_sphereExpKnownDiameterMm);
    QJsonObject root;
    root["schema"]="SphereExposureSweep.Ranking.v1";
    root["finished"]=success;
    root["reason"]=reason;
    QJsonArray passArray,rankArray;
    for(const auto& p:m_sphereExpPassResults)passArray.append(toJson(p));
    root["passes"]=passArray;
    QString csv="Exposure10us,Passes,Eligible,MedianDiameterMm,DiameterErrorMm,RepeatabilityMm,MedianArcRmsMm,Coverage,SignalCoverage,MaximumPlateauFraction,MedianWidth,Score,Note\n";
    for(const auto& r:ranks) {
        QJsonObject v;
        v["exposure10us"]=r.exposure10us;v["passes"]=r.passes;
        v["eligible"]=r.eligible;v["medianDiameterMm"]=r.medianDiameterMm;
        v["diameterErrorMm"]=r.diameterErrorMm;
        v["diameterRepeatabilityMm"]=r.diameterRepeatabilityMm;
        v["medianArcRmsMm"]=r.medianArcRmsMm;
        v["coverageFraction"]=r.coverageFraction;
        v["signalCoverageFraction"]=r.signalCoverageFraction;
        v["medianMaximumPlateauFraction"]=r.medianMaximumPlateauFraction;
        v["medianWidth"]=r.medianWidth;
        v["score"]=r.score;v["note"]=QString::fromStdString(r.note);
        rankArray.append(v);
        csv+=QString("%1,%2,%3,%4,%5,%6,%7,%8,%9,%10,%11,%12,%13\n")
          .arg(r.exposure10us).arg(r.passes).arg(r.eligible?1:0)
          .arg(r.medianDiameterMm,0,'f',6).arg(r.diameterErrorMm,0,'f',6)
          .arg(r.diameterRepeatabilityMm,0,'f',6).arg(r.medianArcRmsMm,0,'f',6)
          .arg(r.coverageFraction,0,'f',5).arg(r.signalCoverageFraction,0,'f',5)
          .arg(r.medianMaximumPlateauFraction,0,'f',5).arg(r.medianWidth,0,'f',5)
          .arg(r.score,0,'f',6).arg(QString::fromStdString(r.note));
    }
    root["ranking"]=rankArray;
    QString writeError;
    saveJsonFile(QDir(m_sphereExpSessionDir).filePath("sphere_exposure_ranking.json"),root,&writeError);
    QFile csvFile(QDir(m_sphereExpSessionDir).filePath("sphere_exposure_ranking.csv"));
    if(csvFile.open(QIODevice::WriteOnly|QIODevice::Text)){
        csvFile.write(csv.toUtf8());csvFile.close();
    }
    if(m_tableSphereExpRanking){
        m_tableSphereExpRanking->setRowCount(static_cast<int>(ranks.size()));
        for(int i=0;i<static_cast<int>(ranks.size());++i){
            const auto& r=ranks[static_cast<std::size_t>(i)];
            const QStringList columns={QString::number(r.exposure10us),
                QString::number(r.medianDiameterMm,'f',5),
                QString::number(1000.0*r.diameterErrorMm,'f',2),
                QString::number(1000.0*r.medianArcRmsMm,'f',2),
                QString::number(100.0*r.coverageFraction,'f',1)+"%",
                r.eligible?QString::number(r.score,'f',5):tr("不合格")};
            for(int c=0;c<columns.size();++c){
                auto* item=new QTableWidgetItem(columns[c]);
                if(i==0&&r.eligible)item->setBackground(QColor(220,245,220));
                m_tableSphereExpRanking->setItem(i,c,item);
            }
            LOG_INFO(LogCategory::Calibration,
                tr("[全标准球曝光][RANK] 曝光=%1 合格=%2 直径=%3mm |偏差|=%4µm "
                   "单帧圆弧RMS=%5µm 有效率=%6% 评分=%7")
                .arg(r.exposure10us).arg(r.eligible?1:0)
                .arg(r.medianDiameterMm,0,'f',6).arg(1000.0*r.diameterErrorMm,0,'f',2)
                .arg(1000.0*r.medianArcRmsMm,0,'f',2)
                .arg(r.coverageFraction*100.0,0,'f',1).arg(r.score,0,'f',5));
        }
    }
    m_sphereExpActive=false;m_sphereExpStopRequested=false;
    m_sphereExpPlanValid=false; // force replan after moving or changing exposure
    if(m_pointCloudView)m_pointCloudView->clearSphereExposureSweepPath();
    setJointMoveUiRunning(false);
    if(m_btnSphereExpStart)m_btnSphereExpStart->setEnabled(false);
    if(m_btnSphereExpStop)m_btnSphereExpStop->setEnabled(false);
    if(m_btnSphereExpPlan)m_btnSphereExpPlan->setEnabled(true);
    if(m_btnSphereExpImport)m_btnSphereExpImport->setEnabled(true);
    if(m_spinSphereExpSpeed)m_spinSphereExpSpeed->setEnabled(true);
    if(m_comboSphereExpPositionSpeed)m_comboSphereExpPositionSpeed->setEnabled(true);
    if(m_spinSphereExpMaxStep)m_spinSphereExpMaxStep->setEnabled(true);
    if(m_spinSphereExpMargin)m_spinSphereExpMargin->setEnabled(true);
    if(m_spinSphereExpRepeats)m_spinSphereExpRepeats->setEnabled(true);
    if(m_spinSphereStudyDiameter)m_spinSphereStudyDiameter->setEnabled(true);
    if(m_spinSensorCalibRate)m_spinSensorCalibRate->setEnabled(true);
    if(m_comboResolution)m_comboResolution->setEnabled(true);
    updateVideoControlState();
    QString note=tr("%1。已完成%2/%3条；归档：%4。")
      .arg(reason).arg(m_sphereExpPassResults.size()).arg(m_sphereExpTotalPasses).arg(m_sphereExpSessionDir);
    if(success&&!ranks.empty()&&ranks.front().eligible)note+=tr(" 候选曝光=%1×10µs；仅供测量数据质量比较，不自动写入材料库。")
        .arg(ranks.front().exposure10us);
    else note+=tr(" 无足够的合格重复扫描，不给出合格曝光推荐。");
    sphereExposureSetStatus(note,!success);
}
