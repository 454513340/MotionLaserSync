#include "app/MainWindow.h"
#include "core/ZmcController.h"
#include "core/SyncManager.h"
#include "gcode/GCodeExecutor.h"
#include "widgets/CalibrationWidget.h"
#include "utils/LogManager.h"
#include <QCheckBox>
#include <QCoreApplication>
#include <QCryptographicHash>
#include <QDir>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QSpinBox>
#include <QTimer>
#include <QVBoxLayout>
#include <algorithm>
#include <cmath>
#include <sstream>

namespace {
bool readJsonFile(const QString& path,QJsonObject& object,QString& error) {
    QFile file(path);
    if(!file.open(QIODevice::ReadOnly)){error=QStringLiteral("无法读取 %1: %2").arg(path,file.errorString());return false;}
    if(file.size()>2*1024*1024){error=QStringLiteral("JSON 超过大小限制: %1").arg(path);return false;}
    QJsonParseError parse;const auto doc=QJsonDocument::fromJson(file.readAll(),&parse);
    if(parse.error!=QJsonParseError::NoError || !doc.isObject()){
        error=QStringLiteral("无效 JSON: %1 / %2").arg(path,parse.errorString());return false;
    }
    object=doc.object();return true;
}
bool verifiedBundleFile(const QDir& root,const QString& relative,const QString& hash,QByteArray& bytes,QString& error) {
    if(relative.isEmpty() || QDir::isAbsolutePath(relative) || hash.size()!=64){
        error=QStringLiteral("补偿包必须提供相对路径和 SHA-256");return false;
    }
    const QString canonicalRoot=QFileInfo(root.absolutePath()).canonicalFilePath();
    const QString full=QFileInfo(root.filePath(relative)).canonicalFilePath();
    if(canonicalRoot.isEmpty() || full.isEmpty() || !full.startsWith(canonicalRoot+QLatin1Char('/'))){
        error=QStringLiteral("文件不存在或位于补偿包之外: %1").arg(relative);return false;
    }
    QFile file(full);
    if(!file.open(QIODevice::ReadOnly) || file.size()>32*1024*1024){error=QStringLiteral("无法读取补偿文件或文件过大: %1").arg(relative);return false;}
    bytes=file.readAll();
    if(QCryptographicHash::hash(bytes,QCryptographicHash::Sha256).toHex()!=hash.toLatin1().toLower()){
        error=QStringLiteral("补偿文件 SHA-256 不匹配，拒绝加载: %1").arg(relative);return false;
    }
    return true;
}
}

bool MainWindow::runtimeCompensationUiBusy() const
{
    return m_closeRequested || m_axisWorkerCount>0 || m_jointMoveInProgress ||
        m_axisCompCalRunning || m_axisCompSpeedRunning || m_axisCompValRunning ||
        m_axisMoveDiagAutoRunning || m_sphereStudyAutoActive ||
        (m_calibrationWidget && m_calibrationWidget->runtimeCompensationChangeBlocked()) ||
        (m_syncManager && m_syncManager->isRunning()) ||
        (m_gcodeExecutor && (m_gcodeExecutor->isRunning() || m_gcodeExecutor->isPaused()));
}

QWidget* MainWindow::createRuntimeAxisCompensationPanel(QWidget* parent)
{
    auto* group=new QGroupBox(tr("XYZ 运行补偿 V27.6.43（MPOS→DPOS + Stage2 数据自举）"),parent);
    auto* layout=new QVBoxLayout(group);
    auto* row=new QHBoxLayout;
    QString path=QDir(QCoreApplication::applicationDirPath()).filePath(QStringLiteral("calibration/runtime_xyz/bundle.json"));
    if(!QFileInfo::exists(path))path=QDir::current().filePath(QStringLiteral("calibration/runtime_xyz/bundle.json"));
    m_runtimeCompBundlePath=new QLineEdit(QDir::toNativeSeparators(path),group);
    row->addWidget(m_runtimeCompBundlePath,1);
    auto* browse=new QPushButton(tr("选择补偿包…"),group);row->addWidget(browse);layout->addLayout(row);
    m_runtimeCompLegacyAck=new QCheckBox(tr("允许 Z 旧式表用于无逐点预加载的计划路径（实验开关，需工况复验）"),group);
    m_runtimeCompContinuousAck=new QCheckBox(tr("允许连续扫描 / G-code / RTCP 的路径补偿（实验开关，静态 A/B 不代表动态精度）"),group);
    layout->addWidget(m_runtimeCompLegacyAck);layout->addWidget(m_runtimeCompContinuousAck);
    auto* controls=new QHBoxLayout;
    controls->addWidget(new QLabel(tr("阻塞定位后静置(ms):"),group));
    m_runtimeCompSettle=new QSpinBox(group);m_runtimeCompSettle->setRange(2000,60000);m_runtimeCompSettle->setValue(2000);
    controls->addWidget(m_runtimeCompSettle);
    auto* load=new QPushButton(tr("加载检查（不启用）"),group);
    auto* enable=new QPushButton(tr("启用 XYZ 运行补偿"),group);
    auto* disable=new QPushButton(tr("禁用运行补偿"),group);
    controls->addWidget(load);controls->addWidget(enable);controls->addWidget(disable);layout->addLayout(controls);
    auto* note=new QLabel(tr("先连接 → 加载三轴表 → XYZ 正规回零 → 人工启用。回零、重连或写入轴配置会撤销启用。"
                             "程序不修改反馈/零点，不自动添加 Z 预加载位移；连续 JOG 仍是原始速度运动。"
                             "采表、测速、DPOS 诊断和 A/B 验证前必须禁用运行补偿。"),group);
    note->setWordWrap(true);layout->addWidget(note);
    m_runtimeCompStatus=new QLabel(tr("XYZ 运行补偿：OFF / 尚未加载"),group);m_runtimeCompStatus->setWordWrap(true);
    m_runtimeCompStatus->setTextInteractionFlags(Qt::TextSelectableByMouse);layout->addWidget(m_runtimeCompStatus);
    connect(browse,&QPushButton::clicked,this,[this]{
        const QString selected=QFileDialog::getOpenFileName(this,tr("选择 XYZ 补偿包 bundle.json"),m_runtimeCompBundlePath->text(),tr("JSON (*.json)"));
        if(!selected.isEmpty())m_runtimeCompBundlePath->setText(QDir::toNativeSeparators(selected));
    });
    connect(load,&QPushButton::clicked,this,[this]{loadRuntimeAxisCompensationBundle(false);});
    connect(enable,&QPushButton::clicked,this,[this]{
        if(runtimeCompensationUiBusy())return;
        const QString warning=tr("将把普通 XYZ 定位和五轴目标解释为期望光栅 MPOS，并使用三轴方向表计算 DPOS。\n\n"
            "X/Y：单调 SWEEP 表；Z：旧式逐点预加载表。执行层不自动改变已检查过的安全路径，也不暗中插入±2 mm位移。\n"
            "允许不同运动历史或连续轨迹仅表示启用相应代码，不能继承原静态验证的精度保证。\n"
            "超表范围、补偿后越限、方向冲突或动态未获准时会拒绝运动，不会退回 RAW。\n\n"
            "请先关闭激光、低速空载检查实际方向/限位，并保持硬件急停可用。确认启用？");
        if(QMessageBox::question(this,tr("启用 XYZ 运行补偿"),warning,QMessageBox::Yes|QMessageBox::No,QMessageBox::No)==QMessageBox::Yes)
            loadRuntimeAxisCompensationBundle(true);
    });
    connect(disable,&QPushButton::clicked,this,[this]{
        if(runtimeCompensationUiBusy()){QMessageBox::warning(this,tr("XYZ 补偿"),tr("请先停止所有任务，不能在运动中切换补偿。"));return;}
        QString error;if(!m_zmc || !m_zmc->setAxisCompensationEnabled(false,&error))QMessageBox::warning(this,tr("禁用失败"),error);
    });
    auto* timer=new QTimer(group);timer->setInterval(500);
    connect(timer,&QTimer::timeout,this,[this,browse,load,enable,disable]{
        const bool on=m_zmc && m_zmc->axisCompensationEnabled();
        const bool connected=m_zmc && m_zmc->isConnected();const bool busy=runtimeCompensationUiBusy();
        browse->setEnabled(!busy&&!on);m_runtimeCompBundlePath->setEnabled(!busy&&!on);
        load->setEnabled(connected&&!busy&&!on);enable->setEnabled(connected&&!busy&&!on);
        disable->setEnabled(connected&&!busy&&on);
        m_runtimeCompLegacyAck->setEnabled(!busy&&!on);m_runtimeCompContinuousAck->setEnabled(!busy&&!on);m_runtimeCompSettle->setEnabled(!busy&&!on);
        m_runtimeCompStatus->setText((on?tr("XYZ 运行补偿：ON"):tr("XYZ 运行补偿：OFF"))+
            (connected?QString():tr(" / 未连接"))+QStringLiteral("\n")+m_runtimeCompSummary);
    });timer->start();
    return group;
}

bool MainWindow::loadRuntimeAxisCompensationBundle(bool enableAfterLoad)
{
    QString error;
    const auto fail=[&](const QString& message){QMessageBox::warning(this,tr("XYZ 补偿包"),message);return false;};
    if(runtimeCompensationUiBusy())return fail(tr("任务运行中，不能加载或切换补偿。"));
    if(!m_zmc || !m_zmc->isConnected())return fail(tr("请先连接控制器。"));
    if(m_zmc->axisCompensationEnabled())return fail(tr("请先禁用运行补偿，再更换数据/选项。"));
    const QString path=QDir::fromNativeSeparators(m_runtimeCompBundlePath->text().trimmed());
    QJsonObject bundle;if(!readJsonFile(path,bundle,error))return fail(error);
    if(bundle.value(QStringLiteral("format")).toString()!=QStringLiteral("MotionLaserSync.XYZRuntimeCompensationBundle") ||
       bundle.value(QStringLiteral("version")).toInt()!=1 ||
       bundle.value(QStringLiteral("mapping")).toString()!=QStringLiteral("desired_mpos_mm -> required_dpos_mm"))
        return fail(tr("不是受支持的 XYZ runtime 补偿包或映射方向错误。"));
    const QDir root(QFileInfo(path).absolutePath());const QJsonObject axes=bundle.value(QStringLiteral("axes")).toObject();
    mls::axiscomp::Tables tables;
    mls::axiscomp::Options options;
    options.allowLegacyPointTableOnPlannedMotion=m_runtimeCompLegacyAck->isChecked();
    options.allowContinuous=m_runtimeCompContinuousAck->isChecked();options.pointSettleMs=m_runtimeCompSettle->value();
    for(int j=0;j<3;++j){
        const QString axis=QString::fromLatin1("XYZ"+j,1);
        const QJsonObject entry=axes.value(axis).toObject();QByteArray csv,manifestBytes;
        if(!verifiedBundleFile(root,entry.value(QStringLiteral("table")).toString(),entry.value(QStringLiteral("table_sha256")).toString(),csv,error) ||
           !verifiedBundleFile(root,entry.value(QStringLiteral("manifest")).toString(),entry.value(QStringLiteral("manifest_sha256")).toString(),manifestBytes,error))return fail(error);
        std::istringstream stream(csv.toStdString());std::string detail;
        if(!mls::axiscomp::readCandidateCsv(stream,j,tables[j],detail))return fail(QString::fromStdString(detail));
        QJsonParseError parse;const auto doc=QJsonDocument::fromJson(manifestBytes,&parse);
        if(parse.error!=QJsonParseError::NoError || !doc.isObject())return fail(tr("%1 manifest 无效").arg(axis));
        const QJsonObject manifest=doc.object();
        if(manifest.value(QStringLiteral("format")).toString()!=QStringLiteral("MotionLaserSync.AxisPositionCompensationCandidate") ||
           manifest.value(QStringLiteral("mapping")).toString()!=QStringLiteral("desired_mpos_mm -> required_dpos_mm"))return fail(tr("%1 manifest 格式/映射不正确").arg(axis));
        const QJsonObject axisMeta=manifest.value(QStringLiteral("axes")).toObject().value(axis).toObject();
        if(axisMeta.isEmpty())return fail(tr("%1 manifest 不包含对应轴").arg(axis));
        for(int d=0;d<2;++d){
            const auto branch=axisMeta.value(d==0?QStringLiteral("positive"):QStringLiteral("negative")).toObject();
            if(branch.value(QStringLiteral("node_count")).toInt(-1)!=static_cast<int>(tables[j].directions[d].size()))
                return fail(tr("%1 manifest 节点数与 CSV 不一致").arg(axis));
        }
        const QString strategy=manifest.value(QStringLiteral("motion_strategy")).toString()+QStringLiteral(" ")+manifest.value(QStringLiteral("sampling_strategy")).toString();
        if(strategy.contains(QStringLiteral("monotonic"),Qt::CaseInsensitive)||strategy.contains(QStringLiteral("direction-prime-once"),Qt::CaseInsensitive))
            tables[j].method=mls::axiscomp::SamplingMethod::MonotonicSweep;
        else if(manifest.value(QStringLiteral("version")).toInt(-1)==1)
            tables[j].method=mls::axiscomp::SamplingMethod::PointToPointPreload;
        else return fail(tr("%1 manifest 的采集策略未知；不允许猜测为 XY 或 Z 方法").arg(axis));
        tables[j].settleMs=manifest.value(QStringLiteral("settle_ms")).toInt(-1);
        tables[j].preloadMm=manifest.value(QStringLiteral("direction_preload_mm")).toDouble(-1);
        const auto profile=manifest.value(QStringLiteral("motion_profiles")).toObject().value(axis).toObject();
        if(!profile.isEmpty()){
            if(!profile.value(QStringLiteral("qualified")).toBool(false))return fail(tr("%1 速度资格标志未通过").arg(axis));
            tables[j].precisionSpeedMmS=profile.value(QStringLiteral("precision_final_approach_speed_mm_s")).toDouble(-1);
        }
        tables[j].sourceId=entry.value(QStringLiteral("table_sha256")).toString().toStdString();
    }
    std::string detail;
    if(!mls::axiscomp::validateTables(tables,options,detail))return fail(QString::fromStdString(detail));
    if(!m_zmc->installAxisCompensation(tables,options,&error))return fail(error);
    m_runtimeCompSummary=m_zmc->axisCompensationStatus();
    LOG_INFO(LogCategory::Controller,tr("XYZ runtime bundle: %1\n%2").arg(path,m_runtimeCompSummary));
    if(enableAfterLoad && !m_zmc->setAxisCompensationEnabled(true,&error))return fail(error);
    m_runtimeCompSummary=m_zmc->axisCompensationStatus();
    return true;
}
