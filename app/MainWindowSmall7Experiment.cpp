#include "app/MainWindow.h"
#include "calibration/SensorCalibrationTiming.h"
#include "core/ZmcController.h"
#include "core/LaserSensor.h"
#include "core/SyncManager.h"
#include "gcode/GCodeExecutor.h"
#include "calibration/SingleSphere17Calibration.h"
#include "utils/GlobalTimeSync.h"
#include "utils/LogManager.h"

#include <QCheckBox>
#include <QComboBox>
#include <QCoreApplication>
#include <QDataStream>
#include <QDateTime>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QImage>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QMetaObject>
#include <QPushButton>
#include <QSpinBox>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QTimer>
#include <QVBoxLayout>
#include <QtConcurrent>
#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <numeric>
#include <vector>

namespace {
namespace s7=mls::calibration::small7;
using mls::calibration::SingleSphere17Model;

QString poseText(const JointPose5D& p){
    return QStringLiteral("X=%1 Y=%2 Z=%3 C=%4 B=%5")
      .arg(p.x,0,'f',4).arg(p.y,0,'f',4).arg(p.z,0,'f',4)
      .arg(p.c,0,'f',4).arg(p.b,0,'f',4);
}
mls::calibration::JointPose5d portablePose(const JointPose5D& p){
    return {p.x,p.y,p.z,p.c,p.b};
}
bool fixedYCB(const JointPose5D& now,const JointPose5D& base){
    return std::abs(now.y-base.y)<=0.005 &&
           std::abs(now.c-base.c)<=0.005 &&
           std::abs(now.b-base.b)<=0.005;
}
bool writeJson(const QString& fileName,const QJsonObject& obj,QString* err){
    QFile f(fileName);
    if(!f.open(QIODevice::WriteOnly|QIODevice::Truncate)){
        if(err)*err=f.errorString();return false;
    }
    if(f.write(QJsonDocument(obj).toJson(QJsonDocument::Indented))<0 || !f.flush()){
        if(err)*err=f.errorString();return false;
    }
    return true;
}
double median(std::vector<double> x){
    if(x.empty())return 0.0;
    std::sort(x.begin(),x.end());
    const auto k=x.size()/2;
    return x.size()%2 ? x[k] : 0.5*(x[k-1]+x[k]);
}
bool readOptics(const QString& file,s7::OpticalWindow& w,QString* error){
    QFile f(file);
    if(!f.open(QIODevice::ReadOnly)){
        if(error)*error=QStringLiteral("无法读取实验3的测量场光学配置 %1: %2").arg(file,f.errorString());
        return false;
    }
    QJsonParseError pe;
    const QJsonDocument doc=QJsonDocument::fromJson(f.readAll(),&pe);
    if(pe.error!=QJsonParseError::NoError||!doc.isObject()){
        if(error)*error=QStringLiteral("实验3视场JSON解析失败: %1").arg(pe.errorString());
        return false;
    }
    const auto o=doc.object();
    // Deliberately no numeric defaults: 0/0 is NOT a valid optical calibration.
    for(const auto* key:{"xMinMm","xMaxMm","zMinMm","zMaxMm","marginMm"}){
        if(!o.contains(QLatin1String(key))||!o.value(QLatin1String(key)).isDouble()){
            if(error)*error=QStringLiteral("所选模式光学配置缺少实测数值 %1").arg(QLatin1String(key));
            return false;
        }
    }
    if(!o.contains("verified")||!o.value("verified").isBool()||
       !o.contains("flipZ")||!o.value("flipZ").isBool()||
       !o.contains("flipX")||!o.value("flipX").isBool()||
       !o.contains("measuringFieldIndex")||!o.value("measuringFieldIndex").isDouble()){
        if(error)*error=QStringLiteral("光学配置必须声明verified、measuringFieldIndex、FlipX和FlipZ");
        return false;
    }
    w.verified=o.value("verified").toBool(false);
    w.flipX=o.value("flipX").toBool();w.flipZ=o.value("flipZ").toBool();
    const double fieldNumber=o.value("measuringFieldIndex").toDouble(-1);
    if(!std::isfinite(fieldNumber) || std::floor(fieldNumber)!=fieldNumber ||
       fieldNumber<0 || fieldNumber>127){
        if(error)*error=QStringLiteral("Measuring Field必须是0～127的整数，不接受截断或非法场号");
        return false;
    }
    w.measuringFieldIndex=static_cast<int>(fieldNumber);
    if(s7::referenceMaxProfileHz(w.measuringFieldIndex)<=0){
        if(error)*error=QStringLiteral("JSON的Measuring Field索引不受当前2950预设表支持");
        return false;
    }
    w.xMinMm=o.value("xMinMm").toDouble();w.xMaxMm=o.value("xMaxMm").toDouble();
    w.zMinMm=o.value("zMinMm").toDouble();w.zMaxMm=o.value("zMaxMm").toDouble();
    w.marginMm=o.value("marginMm").toDouble();
    w.verificationId=o.value("verificationId").toString().trimmed().toStdString();
    if(!w.verified||w.verificationId.empty()){
        if(error)*error=QStringLiteral("所选模式光学边界尚未实测验证：verified必须为true且有verificationId");
        return false;
    }
    return true;
}
QJsonObject opticsJson(const s7::OpticalWindow& w){
    QJsonObject o;
    o["fieldIndex"]=w.measuringFieldIndex;o["verificationId"]=QString::fromStdString(w.verificationId);
    o["reportedXMinMm"]=w.xMinMm;o["reportedXMaxMm"]=w.xMaxMm;
    o["reportedZMinMm"]=w.zMinMm;o["reportedZMaxMm"]=w.zMaxMm;
    o["flipZ"]=w.flipZ;o["flipX"]=w.flipX;o["marginMm"]=w.marginMm;
    return o;
}
QString phaseText(int axis,bool scan){
    return scan?QStringLiteral("SCAN_X_DERIVED_FEED"):(axis==AxisZ?QStringLiteral("FAST_Z_POSITION"):QStringLiteral("FAST_X_OR_STATIC_POSITION"));
}
}

QWidget* MainWindow::createSmall7ExperimentPanel(QWidget* parent){
    auto* box=new QGroupBox(tr("第三实验：可选轮廓测量场 × 机床Z × 最大圆截面静态曝光"),parent);
    auto* layout=new QVBoxLayout(box);
    auto* intro=new QLabel(tr("先在『球面研究』导入标准球点云并拟合球心；用Physical17和当前光学ROI验证全部候选Z的最大圆有效可见弧。"
        "每个Z先扫过整球X，使用扫描轮廓与轴实测位置拟合最大截面，再做7点静态复核并移动至最大圆X位置；"
        "停稳后按本面板的曝光起点/终点/步长逐档采集Profile，可选择附加Video。"
        "下一Z必须重新扫球定位；扫描速度按Profile采样周期×相邻轮廓目标间距计算。"
        "光学理论边界不代表现场遮挡/直径验证，规划不触发运动。"),box);
    intro->setWordWrap(true);layout->addWidget(intro);
    auto* grid=new QGridLayout;
    auto makeZ=[box](double def){
        auto* w=new QDoubleSpinBox(box);w->setDecimals(3);w->setRange(-500,500);
        w->setSingleStep(1);w->setSuffix(QStringLiteral(" mm"));w->setValue(def);return w;
    };
    grid->addWidget(new QLabel(tr("机床Z起点"),box),0,0);
    m_spinSmall7ZStart=makeZ(0);grid->addWidget(m_spinSmall7ZStart,0,1);
    grid->addWidget(new QLabel(tr("机床Z终点"),box),0,2);
    m_spinSmall7ZEnd=makeZ(0);grid->addWidget(m_spinSmall7ZEnd,0,3);
    grid->addWidget(new QLabel(tr("Z高度间隔"),box),1,0);
    m_spinSmall7ZInterval=makeZ(2);m_spinSmall7ZInterval->setRange(0.1,50);
    grid->addWidget(m_spinSmall7ZInterval,1,1);
    grid->addWidget(new QLabel(tr("定位速度"),box),1,2);
    m_comboSmall7PositionSpeed=new QComboBox(box);
    for(int speed=1;speed<=5;++speed)m_comboSmall7PositionSpeed->addItem(tr("%1 mm/s").arg(speed),speed);
    m_comboSmall7PositionSpeed->setCurrentIndex(2);grid->addWidget(m_comboSmall7PositionSpeed,1,3);
    grid->addWidget(new QLabel(tr("X扫描速度（自动）"),box),2,0);
    m_lblSmall7DerivedSpeed=new QLabel(box);
    grid->addWidget(m_lblSmall7DerivedSpeed,2,1);
    grid->addWidget(new QLabel(tr("轮廓采样率"),box),2,2);
    m_spinSmall7Rate=new QSpinBox(box);m_spinSmall7Rate->setRange(25,552);
    m_spinSmall7Rate->setValue(200);m_spinSmall7Rate->setSuffix(tr(" Hz"));
    grid->addWidget(m_spinSmall7Rate,2,3);
    grid->addWidget(new QLabel(tr("最大允许轮廓间距"),box),3,0);
    m_spinSmall7MaxSpacing=makeZ(0.010);m_spinSmall7MaxSpacing->setDecimals(3);
    m_spinSmall7MaxSpacing->setRange(0.001,0.100);m_spinSmall7MaxSpacing->setSingleStep(.005);
    grid->addWidget(m_spinSmall7MaxSpacing,3,1);
    grid->addWidget(new QLabel(tr("寻峰参考曝光"),box),3,2);
    m_spinSmall7ProbeExposure=new QSpinBox(box);
    m_spinSmall7ProbeExposure->setRange(10,1000);
    m_spinSmall7ProbeExposure->setValue(145);m_spinSmall7ProbeExposure->setSuffix(tr(" ×10µs"));
    grid->addWidget(m_spinSmall7ProbeExposure,3,3);
    grid->addWidget(new QLabel(tr("相邻轮廓目标间距"),box),7,0);
    m_spinSmall7LineSpacing=makeZ(0.005);
    m_spinSmall7LineSpacing->setDecimals(3);m_spinSmall7LineSpacing->setRange(0.001,0.100);
    m_spinSmall7LineSpacing->setSingleStep(0.001);
    grid->addWidget(m_spinSmall7LineSpacing,7,1);
    grid->addWidget(new QLabel(tr("最大圆直径容差"),box),7,2);
    m_spinSmall7DiameterTolerance=makeZ(0.500);
    m_spinSmall7DiameterTolerance->setDecimals(3);
    m_spinSmall7DiameterTolerance->setRange(0.050,2.000);
    grid->addWidget(m_spinSmall7DiameterTolerance,7,3);

    // These are intentionally independent of the Sensor Calibration tab's
    // m_spinExpStart/End/Step: Experiment 3 must be configurable on its own.
    grid->addWidget(new QLabel(tr("曝光起点(×10µs)"),box),8,0);
    m_spinSmall7ExpStart=new QSpinBox(box);
    m_spinSmall7ExpStart->setRange(1,4000);m_spinSmall7ExpStart->setValue(50);
    grid->addWidget(m_spinSmall7ExpStart,8,1);
    grid->addWidget(new QLabel(tr("曝光终点(×10µs)"),box),8,2);
    m_spinSmall7ExpEnd=new QSpinBox(box);
    // At 200Hz the entire 5ms period is 500×10us, and Idle must be >0.
    m_spinSmall7ExpEnd->setRange(1,4000);m_spinSmall7ExpEnd->setValue(450);
    grid->addWidget(m_spinSmall7ExpEnd,8,3);
    grid->addWidget(new QLabel(tr("曝光增量(×10µs)"),box),9,0);
    m_spinSmall7ExpStep=new QSpinBox(box);
    m_spinSmall7ExpStep->setRange(1,1000);m_spinSmall7ExpStep->setValue(50);
    grid->addWidget(m_spinSmall7ExpStep,9,1);
    grid->addWidget(new QLabel(tr("每档Video图像"),box),9,2);
    auto* videoChoice=new QWidget(box);auto* videoChoiceLayout=new QHBoxLayout(videoChoice);
    videoChoiceLayout->setContentsMargins(0,0,0,0);
    m_checkSmall7SaveVideo=new QCheckBox(tr("保存"),videoChoice);
    m_checkSmall7SaveVideo->setChecked(false);
    m_spinSmall7VideoFrames=new QSpinBox(videoChoice);
    m_spinSmall7VideoFrames->setRange(1,50);m_spinSmall7VideoFrames->setValue(3);
    m_spinSmall7VideoFrames->setEnabled(false);
    videoChoiceLayout->addWidget(m_checkSmall7SaveVideo);
    videoChoiceLayout->addWidget(m_spinSmall7VideoFrames);
    grid->addWidget(videoChoice,9,3);
    connect(m_checkSmall7SaveVideo,&QCheckBox::toggled,m_spinSmall7VideoFrames,&QWidget::setEnabled);
    grid->addWidget(new QLabel(tr("每点静止有效帧"),box),4,0);
    m_spinSmall7ProbeFrames=new QSpinBox(box);m_spinSmall7ProbeFrames->setRange(10,200);
    m_spinSmall7ProbeFrames->setValue(30);grid->addWidget(m_spinSmall7ProbeFrames,4,1);
    grid->addWidget(new QLabel(tr("每曝光测量Profile数"),box),4,2);
    m_spinSmall7ExposureFrames=new QSpinBox(box);m_spinSmall7ExposureFrames->setRange(10,500);
    m_spinSmall7ExposureFrames->setValue(20);grid->addWidget(m_spinSmall7ExposureFrames,4,3);
    m_spinSmall7ExposureFrames->setToolTip(tr("每个曝光档采集指定数量的实际Profile（不含本实验设置的稳定丢弃帧）；有效/无效拟合均记录，以研究曝光影响"));
    grid->addWidget(new QLabel(tr("稳定丢弃Profile数"),box),10,0);
    m_spinSmall7StaticDiscard=new QSpinBox(box);
    m_spinSmall7StaticDiscard->setRange(0,100);m_spinSmall7StaticDiscard->setValue(3);
    grid->addWidget(m_spinSmall7StaticDiscard,10,1);
    grid->addWidget(new QLabel(tr("采集数据保存"),box),10,2);
    auto* mandatorySave=new QLabel(tr("Profile RAW + 拟合CSV（始终保存）"),box);
    mandatorySave->setWordWrap(true);grid->addWidget(mandatorySave,10,3);
    grid->addWidget(new QLabel(tr("每曝光重复次数"),box),5,0);
    m_spinSmall7Repeats=new QSpinBox(box);m_spinSmall7Repeats->setRange(1,5);
    m_spinSmall7Repeats->setValue(2);grid->addWidget(m_spinSmall7Repeats,5,1);
    grid->addWidget(new QLabel(tr("实验3轮廓测量场"),box),6,0);
    m_comboSmall7Field=new QComboBox(box);
    // All 128 presets in the bundled 2950 Quick Reference are selectable.
    // The common Large/Standard/Small presets come first for easier use.
    auto displayField=[](int field){
        switch(field){
        case 0:return QStringLiteral("Large / 全测量场 (0)");
        case 2:return QStringLiteral("Standard (2)");
        case 7:return QStringLiteral("Small (7)");
        default:return QStringLiteral("Measuring Field %1").arg(field);
        }
    };
    for(int field : {0,2,7}){
        m_comboSmall7Field->addItem(tr("%1 · 项目参考上限%2 Hz")
            .arg(displayField(field)).arg(s7::referenceMaxProfileHz(field)),field);
    }
    for(int field=0;field<128;++field){
        if(field==0||field==2||field==7)continue;
        m_comboSmall7Field->addItem(tr("%1 · 项目参考上限%2 Hz")
            .arg(displayField(field)).arg(s7::referenceMaxProfileHz(field)),field);
    }
    m_comboSmall7Field->setCurrentIndex(2); // Legacy Small7 default
    m_comboSmall7Field->setToolTip(tr("本机预设索引0～127均可选；每个场必须使用其自身的已验证X/Z光学边界JSON"));
    grid->addWidget(m_comboSmall7Field,6,1,1,3);
    layout->addLayout(grid);
    auto* opticsRow=new QHBoxLayout;
    opticsRow->addWidget(new QLabel(tr("当前场已验证的X/Z光学配置"),box));
    m_editSmall7OpticalConfig=new QLineEdit(box);
    m_small7SuggestedOpticalConfigPath=QDir(QCoreApplication::applicationDirPath())
        .filePath(QStringLiteral("config/small7_verified_roi.json"));
    m_editSmall7OpticalConfig->setText(m_small7SuggestedOpticalConfigPath);
    opticsRow->addWidget(m_editSmall7OpticalConfig,1);
    m_btnSmall7Browse=new QPushButton(tr("选择JSON"),box);
    auto* browse=m_btnSmall7Browse;opticsRow->addWidget(browse);
    connect(browse,&QPushButton::clicked,this,[this]{
        const QString path=QFileDialog::getOpenFileName(this,tr("选择当前Measuring Field已验证的光学边界"),
            m_editSmall7OpticalConfig->text(),tr("JSON (*.json)"));
        if(!path.isEmpty())m_editSmall7OpticalConfig->setText(path);
    });
    layout->addLayout(opticsRow);
    auto* outputRow=new QHBoxLayout;
    outputRow->addWidget(new QLabel(tr("实验3数据保存目录"),box));
    m_editSmall7OutputDir=new QLineEdit(QDir::currentPath()+QStringLiteral("/exposure_test"),box);
    outputRow->addWidget(m_editSmall7OutputDir,1);
    auto* outputBrowse=new QPushButton(tr("选择目录"),box);
    outputRow->addWidget(outputBrowse);
    connect(outputBrowse,&QPushButton::clicked,this,[this]{
        const QString folder=QFileDialog::getExistingDirectory(this,tr("选择实验3数据保存根目录"),m_editSmall7OutputDir->text());
        if(!folder.isEmpty())m_editSmall7OutputDir->setText(folder);
    });
    layout->addLayout(outputRow);
    auto* buttons=new QHBoxLayout;
    m_btnSmall7Plan=new QPushButton(tr("生成/检查第三实验Z×X路径"),box);
    m_btnSmall7Start=new QPushButton(tr("开始第三实验"),box);
    m_btnSmall7Stop=new QPushButton(tr("停止第三实验"),box);
    m_btnSmall7Start->setEnabled(false);m_btnSmall7Stop->setEnabled(false);
    buttons->addWidget(m_btnSmall7Plan);buttons->addWidget(m_btnSmall7Start);
    buttons->addWidget(m_btnSmall7Stop);layout->addLayout(buttons);
    m_lblSmall7Status=new QLabel(tr("未规划：必须有Physical17、有效机床基准和当前所选Measuring Field独立验证的光学JSON。"),box);
    m_lblSmall7Status->setWordWrap(true);layout->addWidget(m_lblSmall7Status);
    m_tableSmall7Plan=new QTableWidget(box);m_tableSmall7Plan->setColumnCount(7);
    m_tableSmall7Plan->setHorizontalHeaderLabels({tr("序号"),tr("机床Z mm"),tr("传感球心Z mm"),
        tr("X起点mm"),tr("X终点mm"),tr("可见弧角°"),tr("执行状态")});
    m_tableSmall7Plan->setMinimumHeight(155);
    m_tableSmall7Plan->setEditTriggers(QAbstractItemView::NoEditTriggers);
    layout->addWidget(m_tableSmall7Plan);
    connect(m_btnSmall7Plan,&QPushButton::clicked,this,&MainWindow::onPlanSmall7Experiment);
    connect(m_btnSmall7Start,&QPushButton::clicked,this,&MainWindow::onStartSmall7Experiment);
    connect(m_btnSmall7Stop,&QPushButton::clicked,this,&MainWindow::onStopSmall7Experiment);
    auto invalidate=[this]{small7InvalidatePlan();};
    for(auto* w:{m_spinSmall7ZStart,m_spinSmall7ZEnd,m_spinSmall7ZInterval,m_spinSmall7MaxSpacing,
                 m_spinSmall7LineSpacing,m_spinSmall7DiameterTolerance})
        connect(w,QOverload<double>::of(&QDoubleSpinBox::valueChanged),this,[invalidate](double){invalidate();});
    for(auto* w:{m_spinSmall7Rate,m_spinSmall7ProbeExposure,m_spinSmall7ProbeFrames,
                 m_spinSmall7ExposureFrames,m_spinSmall7StaticDiscard,m_spinSmall7Repeats,m_spinSmall7ExpStart,
                 m_spinSmall7ExpEnd,m_spinSmall7ExpStep,m_spinSmall7VideoFrames})
        if(w)connect(w,QOverload<int>::of(&QSpinBox::valueChanged),this,[invalidate](int){invalidate();});
    connect(m_checkSmall7SaveVideo,&QCheckBox::toggled,this,[invalidate](bool){invalidate();});
    connect(m_editSmall7OutputDir,&QLineEdit::textChanged,this,[invalidate](const QString&){invalidate();});
    auto refreshFeed=[this]{
        s7::ScanTiming t;std::string err;
        if(s7::calculateScanTiming(m_spinSmall7Rate->value(),m_spinSmall7LineSpacing->value(),t,&err))
            m_lblSmall7DerivedSpeed->setText(tr("%1 mm/s (≈%2Hz)")
                .arg(t.scanSpeedMmS,0,'f',3).arg(t.effectiveProfileHz,0,'f',2));
        else m_lblSmall7DerivedSpeed->setText(tr("参数不满足0.1～5mm/s"));
    };
    connect(m_spinSmall7Rate,QOverload<int>::of(&QSpinBox::valueChanged),this,[refreshFeed](int){refreshFeed();});
    connect(m_spinSmall7LineSpacing,QOverload<double>::of(&QDoubleSpinBox::valueChanged),this,[refreshFeed](double){refreshFeed();});
    refreshFeed();
    connect(m_comboSmall7PositionSpeed,QOverload<int>::of(&QComboBox::currentIndexChanged),
        this,[invalidate](int){invalidate();});
    connect(m_comboSmall7Field,QOverload<int>::of(&QComboBox::currentIndexChanged),this,[this,invalidate](int){
        const int field=m_comboSmall7Field->currentData().toInt();
        const int maximum=s7::referenceMaxProfileHz(field);
        m_spinSmall7Rate->setMaximum(maximum);
        if(m_editSmall7OpticalConfig->text()==m_small7SuggestedOpticalConfigPath){
            const auto configName=field==7?QStringLiteral("small7_verified_roi.json"):
                QStringLiteral("experiment3_field_%1_verified_roi.json").arg(field);
            m_small7SuggestedOpticalConfigPath=QDir(QCoreApplication::applicationDirPath())
                .filePath(QStringLiteral("config/")+configName);
            m_editSmall7OpticalConfig->setText(m_small7SuggestedOpticalConfigPath);
        }
        invalidate();
    });
    connect(m_editSmall7OpticalConfig,&QLineEdit::textChanged,this,[invalidate](const QString&){invalidate();});
    connect(m_spinSphereStudyDiameter,QOverload<double>::of(&QDoubleSpinBox::valueChanged),
        this,[invalidate](double){invalidate();});
    return box;
}

void MainWindow::small7InvalidatePlan(){
    if(m_small7Active)return;
    m_small7PlanValid=false;m_small7Heights.clear();
    if(m_btnSmall7Start)m_btnSmall7Start->setEnabled(false);
    if(m_tableSmall7Plan)m_tableSmall7Plan->setRowCount(0);
    if(m_lblSmall7Status)m_lblSmall7Status->setText(tr("参数或校准文件路径已变化，请重新生成并验证第三实验路径。"));
}
void MainWindow::small7SetStatus(const QString& text,bool error){
    if(m_lblSmall7Status){
        m_lblSmall7Status->setText(text);
        m_lblSmall7Status->setStyleSheet(error?QStringLiteral("color:#a33;font-weight:bold;"):
                                             QStringLiteral("color:#245;"));
    }
    const QString s=tr("[第三实验 Profile Field] %1").arg(text);
    if(error)LOG_WARNING(LogCategory::Calibration,s);
    else LOG_INFO(LogCategory::Calibration,s);
}

void MainWindow::onPlanSmall7Experiment(){
    small7InvalidatePlan();
    auto fail=[this](const QString& reason){small7SetStatus(tr("规划拒绝：%1").arg(reason),true);};
    if(m_small7Active||m_sphereExpActive||m_sphereStudyAutoActive||m_jointMoveInProgress||
       m_axisWorkerCount>0||(m_syncManager&&m_syncManager->isRunning())||m_expTestState!=ExpTestState::Idle)
        return fail(tr("另一运动/采集任务正在运行"));
    QString error;
    if(!m_sphereStudySphereFit.success||!refreshSphereStudyPlanningModel(&error)||
       !m_sphereStudyPlanningModelValidated)
        return fail(tr("必须先导入球云拟合球心并验证完整Physical17模型：%1").arg(error));
    if(!m_zmc||!m_zmc->isConnected()||!m_zmc->areLinearMposReferenced()||
       !m_zmc->isRotaryAbsoluteReady(AxisB)||!m_zmc->isRotaryAbsoluteReady(AxisC))
        return fail(tr("控制器未连接、XYZ未回零或B/C绝对位置无效"));
    if(!sphereStudySafetyReady(&error))return fail(error);
    JointState5D state;
    if(!m_zmc->readJointState5D(state,&error)||!state.allIdle)
        return fail(tr("规划必须从五轴静止MPOS开始：%1").arg(error));
    if(!readOptics(m_editSmall7OpticalConfig->text().trimmed(),m_small7Optics,&error))
        return fail(error);
    const int selectedField=m_comboSmall7Field->currentData().toInt();
    if(m_small7Optics.measuringFieldIndex!=selectedField)
        return fail(tr("光学配置field=%1与选择的扫描轮廓field=%2不一致；禁止混用不同视场的边界")
            .arg(m_small7Optics.measuringFieldIndex).arg(selectedField));
    if(m_small7Optics.flipX!=m_sphereStudyPlanningFlipX||
       m_small7Optics.flipZ!=m_sphereStudyPlanningFlipZ)
        return fail(tr("所选场光学配置FlipX/FlipZ与已验证Physical17轮廓坐标约定不一致"));
    s7::Settings conf;
    conf.measuringFieldIndex=selectedField;
    conf.zBeginMm=m_spinSmall7ZStart->value();conf.zEndMm=m_spinSmall7ZEnd->value();
    conf.zIntervalMm=m_spinSmall7ZInterval->value();
    conf.positioningSpeedMmS=m_comboSmall7PositionSpeed->currentData().toDouble();
    conf.profileRateHz=m_spinSmall7Rate->value();
    conf.requestedLineSpacingMm=m_spinSmall7LineSpacing->value();
    s7::ScanTiming timing;
    std::string coreError;
    if(!s7::calculateScanTiming(conf.profileRateHz,conf.requestedLineSpacingMm,timing,&coreError))
        return fail(QString::fromStdString(coreError));
    conf.scanSpeedMmS=timing.scanSpeedMmS;
    conf.maximumFrameSpacingMm=m_spinSmall7MaxSpacing->value();
    conf.knownSphereDiameterMm=m_spinSphereStudyDiameter->value();
    conf.scanMarginMm=0.8;conf.sevenPointStepMm=0.20;
    // Evaluate a wider *optical-only* search domain. This suggests a usable
    // machine-Z window instead of simply echoing the operator's endpoints.
    // It does not override MachineSafety limits or authorize motion.
    auto opticalSearch=conf;
    opticalSearch.zBeginMm=std::max(-500.0,std::min(conf.zBeginMm,state.mpos.z)-30.0);
    opticalSearch.zEndMm=std::min(500.0,std::max(conf.zEndMm,state.mpos.z)+30.0);
    const auto possibleZ=s7::findOpticalZIntervals(opticalSearch,m_small7Optics,portablePose(state.mpos),
        m_sphereStudySphereFit.centerMachineMm,m_sphereStudyPlanningModel);
    QStringList opticalBandDescriptions;
    for(const auto& band:possibleZ)
        opticalBandDescriptions<<tr("[%1,%2]mm")
            .arg(band.zStartMm,0,'f',3).arg(band.zEndMm,0,'f',3);
    const QString opticalBandText=opticalBandDescriptions.isEmpty()?
        tr("无"):opticalBandDescriptions.join(QStringLiteral(", "));
    // A single optically valid CONTIGUOUS Z band must cover all requested
    // heights, not just the discrete start/end or arbitrarily good sub-layers.
    const bool entireZAllowed=std::any_of(possibleZ.begin(),possibleZ.end(),[&](const auto& band){
        return band.zStartMm<=conf.zBeginMm+1e-7 && band.zEndMm>=conf.zEndMm-1e-7;
    });
    if(!entireZAllowed){
        return fail(tr("当前Z区间无法保证已验证X/Z光学ROI中有至少%2°的理论最大圆连续可见弧。"
                       "理论可用连续机床Z区间：%1。请调整视场/高度，并在每层通过实际直径复核。")
                       .arg(opticalBandText).arg(s7::kMinPlannedArcDeg,0,'f',0));
    }
    std::vector<s7::HeightPlan> heights;
    if(!s7::planHeights(conf,m_small7Optics,portablePose(state.mpos),
         m_sphereStudySphereFit.centerMachineMm,m_sphereStudyPlanningModel,heights,&coreError))
        return fail(QString::fromStdString(coreError));
    // Safety is validated for the entire XYZCB trajectory at each layer, including
    // all seven possible static peak positions and all subsequent Z transitions.
    JointPose5D cursor=state.mpos;
    for(std::size_t i=0;i<heights.size();++i){
        const auto& h=heights[i];
        // All possible previous static peak locations are checked at the NEXT Z.
        if(i>0){
            const auto& previous=heights[i-1];
            for(int k=-7;k<=7;++k){
                // The measured peak may deviate up to 0.8mm from model, and
                // static probes extend a further ±0.6mm.
                const double possibleX=previous.centerXMachineMm+0.2*k;
                auto from=cursor;from.x=possibleX;from.z=previous.machineZMm;
                auto to=from;to.z=h.machineZMm;
                if(!checkSphereStudyPoseSafety(from,&error,false)||
                   !checkSphereStudyTransitionSafety(from,to,&error,false))
                    return fail(tr("高度%1候选峰位X=%2→新Z的垂直路径不安全：%3")
                         .arg(i+1).arg(possibleX,0,'f',3).arg(error));
            }
        }
        auto z=cursor;z.z=h.machineZMm;
        auto start=z;start.x=h.scanStartXMachineMm;
        auto end=start;end.x=h.scanEndXMachineMm;
        if(!checkSphereStudyTransitionSafety(cursor,z,&error,false)||
           !checkSphereStudyTransitionSafety(z,start,&error,false)||
           !checkSphereStudyTransitionSafety(start,end,&error,false))
            return fail(tr("高度%1 Z/X整段路径不安全：%2").arg(i+1).arg(error));
        for(double probeX:h.sevenPointX){
            auto probe=end;probe.x=probeX;
            if(!checkSphereStudyTransitionSafety(end,probe,&error,false))
                return fail(tr("高度%1的7点寻峰路径不安全：%2").arg(i+1).arg(error));
        }
        cursor=end;
        cursor.x=h.centerXMachineMm;
    }
    // Frozen optical model & plan will be revalidated before each actual move.
    m_small7Settings=conf;m_small7BasePose=state.mpos;m_small7Heights=std::move(heights);
    m_small7PlannedPhysical17=m_sphereStudyPlanningModel.value;
    m_small7PlannedSphereCenter=m_sphereStudySphereFit.centerMachineMm;
    m_small7PlanValid=true;
    m_tableSmall7Plan->setRowCount(static_cast<int>(m_small7Heights.size()));
    for(int i=0;i<static_cast<int>(m_small7Heights.size());++i){
        const auto& h=m_small7Heights[static_cast<std::size_t>(i)];
        QStringList columns={QString::number(i+1),QString::number(h.machineZMm,'f',3),
            QString::number(h.predictedSensorCenterZ,'f',3),
            QString::number(h.scanStartXMachineMm,'f',3),
            QString::number(h.scanEndXMachineMm,'f',3),
            QString::number(h.visibleArcSpanDeg,'f',1),tr("已规划/待执行")};
        for(int j=0;j<columns.size();++j)m_tableSmall7Plan->setItem(i,j,new QTableWidgetItem(columns[j]));
    }
    m_btnSmall7Start->setEnabled(true);
    small7SetStatus(tr("规划通过（没有运动）：%1层，以0.1mm步长采样Z=%2～%3mm最大圆理论可见球弧≥50°；"
        "定位%4mm/s、扫描%5mm/s（依量化频率和轮廓目标间距计算）、"
        "Measuring Field=%6、名义f=%7Hz、帧距=%8mm；光学验证ID=%9。"
        "光学理论可用机床Z区间=%10（非MachineSafety认证）；每个Z的实测最大直径仍必须再次验证。")
        .arg(m_small7Heights.size()).arg(conf.zBeginMm,0,'f',3).arg(conf.zEndMm,0,'f',3)
        .arg(conf.positioningSpeedMmS,0,'f',1).arg(conf.scanSpeedMmS,0,'f',3)
        .arg(conf.measuringFieldIndex).arg(timing.effectiveProfileHz,0,'f',2)
        .arg(timing.nominalLineSpacingMm,0,'f',4)
        .arg(QString::fromStdString(m_small7Optics.verificationId)).arg(opticalBandText));
}

void MainWindow::onStartSmall7Experiment(){
    if(m_small7Active||!m_small7PlanValid||m_small7Heights.empty())
        return small7SetStatus(tr("请先生成与所选测量场匹配的有效实验3路径"),true);
    QString err;
    if(!m_laser||!m_laser->isConnected()||!m_zmc||!m_zmc->isConnected())
        return small7SetStatus(tr("控制器与轮廓传感器都必须连接"),true);
    if(!m_sphereStudyPlanningModelValidated||!m_sphereStudySphereFit.success||
       m_sphereStudyPlanningModel.value!=m_small7PlannedPhysical17||
       m_sphereStudySphereFit.centerMachineMm.x!=m_small7PlannedSphereCenter.x||
       m_sphereStudySphereFit.centerMachineMm.y!=m_small7PlannedSphereCenter.y||
       m_sphereStudySphereFit.centerMachineMm.z!=m_small7PlannedSphereCenter.z)
        return small7SetStatus(tr("规划后的Physical17或标准球心发生变化，请重新规划"),true);
    if(m_sphereExpActive||m_sphereStudyCaptureActive||m_sphereStudyAutoActive||
       m_profilePreviewRunning||m_jointMoveInProgress||m_axisWorkerCount>0||
       m_laser->isVideoRunning()||m_laser->isProfileRunning()||
       (m_syncManager&&m_syncManager->isRunning())||m_expTestState!=ExpTestState::Idle||
       (m_gcodeExecutor&&(m_gcodeExecutor->isRunning()||m_gcodeExecutor->isPaused())))
        return small7SetStatus(tr("另一运动/扫描/预览任务未停止"),true);
    if(!m_zmc->isRotaryServoEnabled(AxisB)||!m_zmc->isRotaryServoEnabled(AxisC)||
       !m_zmc->areLinearMposReferenced())
        return small7SetStatus(tr("XYZ机床基准或B/C伺服使能不满足自动运动互锁"),true);
    JointState5D s;
    if(!m_zmc->readJointState5D(s,&err)||!s.allIdle)
        return small7SetStatus(tr("执行前无法确认五轴静止：%1").arg(err),true);
    const auto& p=m_small7BasePose;
    if(std::abs(s.mpos.x-p.x)>0.005||std::abs(s.mpos.y-p.y)>0.005||
       std::abs(s.mpos.z-p.z)>0.005||std::abs(s.mpos.c-p.c)>0.005||
       std::abs(s.mpos.b-p.b)>0.005)
        return small7SetStatus(tr("规划后机床实际MPOS变化（原%1；今%2）；必须重新规划")
            .arg(poseText(p),poseText(s.mpos)),true);
    s7::OpticalWindow fresh;
    if(!readOptics(m_editSmall7OpticalConfig->text().trimmed(),fresh,&err)||
       fresh.verificationId!=m_small7Optics.verificationId||
       fresh.xMinMm!=m_small7Optics.xMinMm||fresh.xMaxMm!=m_small7Optics.xMaxMm||
       fresh.zMinMm!=m_small7Optics.zMinMm||fresh.zMaxMm!=m_small7Optics.zMaxMm||
       fresh.marginMm!=m_small7Optics.marginMm||fresh.flipZ!=m_small7Optics.flipZ||
       fresh.flipX!=m_small7Optics.flipX||
       fresh.measuringFieldIndex!=m_small7Optics.measuringFieldIndex||
       m_comboSmall7Field->currentData().toInt()!=m_small7Settings.measuringFieldIndex)
        return small7SetStatus(tr("所选测量场的验证配置已改变或无法回读，必须重新规划：%1").arg(err),true);
    SensorAdvancedParams adv;
    if(!m_laser->readAdvancedParams(adv,&err)||
       adv.flipX!=m_small7Optics.flipX||adv.flipZ!=m_small7Optics.flipZ)
        return small7SetStatus(tr("实时传感器FlipX/FlipZ无法回读或与所选场光学标定不一致：%1").arg(err),true);
    const int rate=m_small7Settings.profileRateHz;
    const int maxHz=LaserSensor::getMaxFrequencyForIndex(m_small7Settings.measuringFieldIndex);
    if(maxHz<=0||maxHz!=s7::referenceMaxProfileHz(m_small7Settings.measuringFieldIndex))
        return small7SetStatus(tr("所选场的保守频率表不一致，禁止运动"),true);
    const int expStart=m_spinSmall7ExpStart->value(),expEnd=m_spinSmall7ExpEnd->value();
    const int expStep=m_spinSmall7ExpStep->value();
    const int levels=expStep>0?(expEnd-expStart)/expStep+1:0;
    if(expStart>expEnd||expStep<=0||levels<1||levels>40||
       expEnd>=static_cast<int>(std::ceil(100000.0/rate))||
       m_spinSmall7ProbeExposure->value()>=static_cast<int>(std::ceil(100000.0/rate))||
       (maxHz>0&&rate>maxHz))
        return small7SetStatus(tr("曝光范围、参考曝光或所选场采样周期不兼容：ExpEnd=%1、Rate=%2Hz、档数=%3")
                 .arg(expEnd).arg(rate).arg(levels),true);
    if(m_axisParams[AxisX].speed+1e-6<m_small7Settings.positioningSpeedMmS||
       m_axisParams[AxisZ].speed+1e-6<m_small7Settings.positioningSpeedMmS||
       m_axisParams[AxisX].speed+1e-6<m_small7Settings.scanSpeedMmS)
        return small7SetStatus(tr("X/Z轴认证速度小于选择的定位速度，请检查轴速度参数"),true);
    if(m_axisParams[AxisX].accel<=0||m_axisParams[AxisZ].accel<=0||
       m_axisParams[AxisX].decel<=0||m_axisParams[AxisZ].decel<=0)
        return small7SetStatus(tr("X/Z轴加减速度尚未校准"),true);
    if(!checkSphereStudyTransitionSafety(s.mpos,s.mpos,&err,false))
        return small7SetStatus(tr("执行前MachineSafety复核失败：%1").arg(err),true);
    const int passes=levels*m_spinSmall7Repeats->value();
    if(QMessageBox::question(this,tr("确认启动多测量场实验3"),
        tr("将运动到%1个机床Z高度，每层完整扫球X一次（由轮廓间距计算速度%6mm/s），扫描X/轮廓拟合最大圆，"
           "再做7点静态复核，然后执行%2组静态曝光Profile采集%7。定位速度%3mm/s；Measuring Field=%4，采样%5Hz。\n"
           "机床Y/C/B保持锁定；不会自动越过已验证的光学视场、机床限位和碰撞门禁。\n"
           "设备/工件已远离危险区域、急停可用，并已实机确认当前场光学配置后，是否继续？")
        .arg(m_small7Heights.size()).arg(passes).arg(m_small7Settings.positioningSpeedMmS)
        .arg(m_small7Settings.measuringFieldIndex).arg(rate)
        .arg(m_small7Settings.scanSpeedMmS,0,'f',3)
        .arg(m_checkSmall7SaveVideo->isChecked()?tr("，并顺序保存Video图像"):QString()),
        QMessageBox::Yes|QMessageBox::No,QMessageBox::No)!=QMessageBox::Yes)return;
    const QString root=m_editSmall7OutputDir->text().trimmed();
    if(root.isEmpty())return small7SetStatus(tr("RAW归档路径未设置"),true);
    const QString dir=QDir(root).filePath(QStringLiteral("Experiment3/Field%1_")
       .arg(m_small7Settings.measuringFieldIndex)+
       QDateTime::currentDateTimeUtc().toString("yyyyMMdd_HHmmss_zzz"));
    if(!QDir().mkpath(dir))return small7SetStatus(tr("无法创建归档目录：%1").arg(dir),true);
    QJsonObject session;
    session["schema"]="Experiment3.MultiField.v3";
    session["measuringFieldIndex"]=m_small7Settings.measuringFieldIndex;
    session["profileAcquisitionMode"]="sensor_software_internal";
    session["profileFieldMaxReferenceHz"]=maxHz;
    session["legacyRawContainer"]="S7P1";
    session["optics"]=opticsJson(m_small7Optics);
    session["physical17Source"]=m_sphereStudyPlanningModelSource;
    QJsonArray params;for(double v:m_sphereStudyPlanningModel.value)params.append(v);
    session["physical17Parameters"]=params;
    session["positioningSpeedMmS"]=m_small7Settings.positioningSpeedMmS;
    session["scanSpeedMmS"]=m_small7Settings.scanSpeedMmS;session["profileRateHz"]=rate;
    session["nominalProfilePeriod10us"]=static_cast<int>(std::ceil(100000.0/rate));
    session["nominalEffectiveProfileHz"]=100000.0/std::ceil(100000.0/rate);
    session["requestedAdjacentProfileSpacingMm"]=m_small7Settings.requestedLineSpacingMm;
    session["maxScanFrameSpacingMm"]=m_small7Settings.maximumFrameSpacingMm;
    session["sensorTrigger"]="software_internal";
    session["machineZStartMm"]=m_small7Settings.zBeginMm;
    session["machineZEndMm"]=m_small7Settings.zEndMm;
    session["machineZIntervalMm"]=m_small7Settings.zIntervalMm;
    session["sphereDiameterMm"]=m_small7Settings.knownSphereDiameterMm;
    session["sphereCenterMachineX"]=m_sphereStudySphereFit.centerMachineMm.x;
    session["sphereCenterMachineY"]=m_sphereStudySphereFit.centerMachineMm.y;
    session["sphereCenterMachineZ"]=m_sphereStudySphereFit.centerMachineMm.z;
    session["exposureStart10us"]=expStart;session["exposureEnd10us"]=expEnd;
    session["exposureStep10us"]=expStep;session["repeatCount"]=m_spinSmall7Repeats->value();
    session["staticFramesPerExposure"]=m_spinSmall7ExposureFrames->value();
    session["stableDiscardProfilesPerStaticCapture"]=m_spinSmall7StaticDiscard->value();
    session["staticFramesPerProbe"]=m_spinSmall7ProbeFrames->value();
    session["maximumDiameterErrorMm"]=m_spinSmall7DiameterTolerance->value();
    session["saveProfileRaw"]=true;
    session["saveVideo"]=m_checkSmall7SaveVideo->isChecked();
    session["videoFramesPerPass"]=m_checkSmall7SaveVideo->isChecked()?m_spinSmall7VideoFrames->value():0;
    session["videoCaptureMode"]="sequential_after_stationary_profile_not_simultaneous";
    session["scanPeakPositionBasis"]="interpolated_controller_MPOS_at_sensor_PC_callback_time_static_confirmed";
    session["opticalCoverage"]="theoretical_arc_at_least_50deg_in_verified_ROI;actual_arc_and_diameter_rechecked_per_Z";
    if(!writeJson(QDir(dir).filePath("session.json"),session,&err))
        return small7SetStatus(tr("Session归档失败：%1").arg(err),true);
    QFile comparison(QDir(dir).filePath(QStringLiteral("experiment3_Z_exposure_comparison.csv")));
    if(!comparison.open(QIODevice::WriteOnly|QIODevice::Text)||
       comparison.write("HeightIndex,MachineZ_MPOS_mm,MeasuringFieldIndex,ScanPeakX_mm,RefinedPeakX_mm,"
           "Exposure_10us,Repeat,ProfileRate_Hz,RequestedProfileCount,TotalReceivedProfiles,"
           "ValidArcProfiles,ValidArcFraction,MedianDiameter_mm,DiameterError_mm,"
           "ProfileIndexGapEvents,EstimatedMissingProfiles,VideoPngCount,ProfileRawPrefix\n")<0||!comparison.flush())
        return small7SetStatus(tr("实验3对比结果CSV无法创建：%1").arg(comparison.errorString()),true);
    m_small7SessionDir=dir;m_small7HeightIndex=0;m_small7ProbeIndex=0;m_small7ExposurePass=0;
    m_small7ExpStart=expStart;m_small7ExpEnd=expEnd;m_small7ExpStep=expStep;
    m_small7ExpRepeats=m_spinSmall7Repeats->value();m_small7Rate=rate;
    m_small7ProbeExposure=m_spinSmall7ProbeExposure->value();
    m_small7ProbeFrames=m_spinSmall7ProbeFrames->value();
    m_small7ExposureFrames=m_spinSmall7ExposureFrames->value();
    m_small7StaticDiscardFrames=m_spinSmall7StaticDiscard->value();
    m_small7DiameterToleranceMm=m_spinSmall7DiameterTolerance->value();
    m_small7VideoEnabled=m_checkSmall7SaveVideo->isChecked();
    m_small7VideoFramesPerPass=m_small7VideoEnabled?m_spinSmall7VideoFrames->value():0;
    m_small7KnownSphereRadius=m_small7Settings.knownSphereDiameterMm/2.0;
    m_small7FaultHistory.clear();m_small7ChosenPeakX=std::numeric_limits<double>::quiet_NaN();
    m_small7Active=true;m_small7StopRequested=false;
    m_small7Phase=Small7Phase::Idle;m_small7Capturing=false;++m_small7Generation;
    setJointMoveUiRunning(true);
    for(auto* widget:{m_spinSmall7ZStart,m_spinSmall7ZEnd,m_spinSmall7ZInterval,m_spinSmall7MaxSpacing,
                     m_spinSmall7LineSpacing,m_spinSmall7DiameterTolerance})widget->setEnabled(false);
    for(auto* widget:{m_spinSmall7Rate,m_spinSmall7ProbeExposure,m_spinSmall7ProbeFrames,
                     m_spinSmall7ExposureFrames,m_spinSmall7StaticDiscard,m_spinSmall7Repeats,m_spinSmall7ExpStart,
                     m_spinSmall7ExpEnd,m_spinSmall7ExpStep,m_spinSmall7VideoFrames})widget->setEnabled(false);
    m_checkSmall7SaveVideo->setEnabled(false);m_editSmall7OutputDir->setEnabled(false);
    m_comboSmall7PositionSpeed->setEnabled(false);
    m_comboSmall7Field->setEnabled(false);
    m_editSmall7OpticalConfig->setEnabled(false);
    m_btnSmall7Browse->setEnabled(false);
    m_btnSmall7Start->setEnabled(false);m_btnSmall7Stop->setEnabled(true);
    m_btnSmall7Plan->setEnabled(false);
    m_small7ProfileConnection=connect(m_laser.get(),&LaserSensor::profileReceived,this,
          [this](const LaserProfile& p){small7OnProfile(p);},Qt::QueuedConnection);
    small7SetStatus(tr("已建立场%1静态曝光实验会话：%2；准备定位首个Z高度")
        .arg(m_small7Settings.measuringFieldIndex).arg(dir));
    QTimer::singleShot(0,this,[this]{small7AdvanceHeight();});
}

void MainWindow::small7AdvanceHeight(){
    if(!m_small7Active)return;
    if(m_small7StopRequested)return small7Finish(false,tr("操作者停止"));
    if(m_small7HeightIndex>=static_cast<int>(m_small7Heights.size()))
        return small7Finish(true,tr("所有机床Z高度完成"));
    m_small7ProbeIndex=0;m_small7ExposurePass=0;
    m_small7ChosenPeakX=std::numeric_limits<double>::quiet_NaN();
    m_small7ScanPeakX=std::numeric_limits<double>::quiet_NaN();
    m_small7ScanPeakRadius=0;
    m_small7ProbeBestRadii.fill(0.0);
    const auto& h=m_small7Heights[static_cast<std::size_t>(m_small7HeightIndex)];
    small7SetStatus(tr("[%1/%2] FAST_Z_POSITION: 前往机床Z=%3mm，定位速度=%4mm/s")
        .arg(m_small7HeightIndex+1).arg(m_small7Heights.size())
        .arg(h.machineZMm,0,'f',4).arg(m_small7Settings.positioningSpeedMmS));
    m_tableSmall7Plan->selectRow(m_small7HeightIndex);
    m_tableSmall7Plan->item(m_small7HeightIndex,6)->setText(tr("定位Z"));
    small7Move(AxisZ,h.machineZMm,Small7Phase::ZPosition);
}

void MainWindow::small7Move(int axis,double targetMpos,Small7Phase phase){
    if(!m_small7Active||m_small7WorkerActive)return;
    if(m_small7StopRequested)return small7Finish(false,tr("收到停止请求"));
    if(axis!=AxisX&&axis!=AxisZ)return small7Finish(false,tr("实验3禁止运动除X/Z之外的轴"));
    // The model and fitted reference sphere are immutable inputs for the entire
    // planned session. Reject edits/rebinding before issuing another machine move.
    const auto& center=m_sphereStudySphereFit.centerMachineMm;
    if(!m_sphereStudyPlanningModelValidated ||
       m_sphereStudyPlanningModel.value!=m_small7PlannedPhysical17 ||
       center.x!=m_small7PlannedSphereCenter.x ||
       center.y!=m_small7PlannedSphereCenter.y ||
       center.z!=m_small7PlannedSphereCenter.z ||
       std::abs(m_spinSphereStudyDiameter->value()-m_small7Settings.knownSphereDiameterMm)>1e-9)
        return small7Finish(false,tr("实验过程中Physical17/球心/球直径发生变化，禁止发出下一条运动指令"));
    JointState5D state;QString error;
    if(!m_zmc->readJointState5D(state,&error)||!state.allIdle)
        return small7Finish(false,tr("移动阶段=%1，不能读取五轴停稳的MPOS/DPOS：%2")
                 .arg(phaseText(axis,phase==Small7Phase::XScan),error));
    if(!fixedYCB(state.mpos,m_small7BasePose))
        return small7Finish(false,tr("Y/C/B偏离规划锁定位置：%1；计划=%2")
                 .arg(poseText(state.mpos),poseText(m_small7BasePose)));
    if(axis==AxisX && std::abs(state.mpos.z-
        m_small7Heights[static_cast<std::size_t>(m_small7HeightIndex)].machineZMm)>0.005)
        return small7Finish(false,tr("运动X前Z尚未进入目标高度：当前%1mm")
                 .arg(state.mpos.z,0,'f',6));
    auto target=state.mpos;
    if(axis==AxisX)target.x=targetMpos;else target.z=targetMpos;
    if(!checkSphereStudyPoseSafety(target,&error,false)||
       !checkSphereStudyTransitionSafety(state.mpos,target,&error,false))
        return small7Finish(false,tr("阶段=%1，轴%2，目标MPOS=%3mm，MachineSafety拒绝：%4")
                 .arg(phaseText(axis,phase==Small7Phase::XScan))
                 .arg(axis).arg(targetMpos,0,'f',5).arg(error));
    const double speed=(phase==Small7Phase::XScan)?m_small7Settings.scanSpeedMmS:
        m_small7Settings.positioningSpeedMmS;
    if(m_axisParams[axis].speed+1e-6<speed||m_axisParams[axis].accel<=0||
       m_axisParams[axis].decel<=0)
        return small7Finish(false,tr("轴%1认证速度/加减速度不允许请求%2mm/s")
                 .arg(axis).arg(speed));
    AxisParams motion=m_axisParams[axis];motion.speed=static_cast<float>(speed);
    motion.lspeed=0.0f;  // no unexpected minimum speed larger than scan speed
    m_small7Phase=phase;m_small7WorkerActive=true;++m_axisWorkerCount;
    if(phase==Small7Phase::XScan){
        m_small7ScanMotionProfiles=0;
        m_small7ScanMotionGoodFrames=0;
        m_small7ScanProfiles.clear();m_small7ScanAllFramePcUs.clear();m_small7AxisSamples.clear();
        m_small7MaxObservedFrameSpacingMm=0.0;
        m_small7PreviousFrameUs=0;
        GlobalTimeSync::instance().start();
        const qint64 now=GlobalTimeSync::instance().timestampUs();
        m_small7AxisSamples.push_back({now,state.mpos.x});
    }
    const auto generation=m_small7Generation;
    const auto moveToken=++m_small7MoveToken;
    m_small7MoveStartedMs=QDateTime::currentMSecsSinceEpoch();
    QTimer::singleShot(phase==Small7Phase::XScan?120:600,this,[this,generation,moveToken,axis,targetMpos]{
        small7PositionHeartbeat(generation,moveToken,axis,targetMpos);
    });
    auto* controller=m_zmc.get();
    small7SetStatus(tr("运动开始：阶段=%1 轴%2，目标MPOS=%3mm，当前MPOS=%4mm，"
        "当前DPOS=%5mm，请求速度=%6mm/s")
        .arg(phaseText(axis,phase==Small7Phase::XScan)).arg(axis)
        .arg(targetMpos,0,'f',5).arg(axis==AxisX?state.mpos.x:state.mpos.z,0,'f',5)
        .arg(axis==AxisX?state.dpos.x:state.dpos.z,0,'f',5).arg(speed,0,'f',2));
    (void)QtConcurrent::run([this,controller,generation,axis,targetMpos,motion,phase]{
        JointMoveResult5D result;QString error;
        bool ok=controller->moveLinearMposAbs(axis,static_cast<float>(targetMpos),&motion,&error);
        if(ok){
            ok=controller->readJointState5D(result.finalState,&error);
            if(ok){
                result.completed=result.finalState.allIdle;
                if(!result.completed)error=QStringLiteral("移动返回时五轴未全部IDLE");
                result.maxLinearMposErrorMm=std::abs((axis==AxisX?
                    result.finalState.mpos.x:result.finalState.mpos.z)-targetMpos);
            }
        }
        QMetaObject::invokeMethod(this,[this,generation,axis,targetMpos,phase,ok,result,error]{
            --m_axisWorkerCount;m_small7WorkerActive=false;
            if(generation==m_small7Generation)
                small7MoveFinished(generation,axis,targetMpos,phase,ok,result,error);
            tryFinishDeferredClose();
        },Qt::QueuedConnection);
    });
}

void MainWindow::small7PositionHeartbeat(quint64 generation,quint64 moveToken,
                                                   int axis,double targetMpos){
    if(generation!=m_small7Generation||moveToken!=m_small7MoveToken||
       !m_small7Active||!m_small7WorkerActive)return;
    float dpos=0,mpos=0;bool idle=false;QString error;
    const qint64 readStartUs=GlobalTimeSync::instance().timestampUs();
    const bool readOk=m_zmc&&m_zmc->readAxisUiPosition(axis,dpos,mpos,idle,&error);
    const qint64 readEndUs=GlobalTimeSync::instance().timestampUs();
    if(readOk && axis==AxisX && m_small7Phase==Small7Phase::XScan &&
       readStartUs>0 && readEndUs>=readStartUs && readEndUs-readStartUs<=100000)
        m_small7AxisSamples.push_back({(readStartUs+readEndUs)/2,double(mpos)});
    const qint64 elapsed=QDateTime::currentMSecsSinceEpoch()-m_small7MoveStartedMs;
    if(readOk){
        small7SetStatus(tr("正在运动（进度）：轴%1，目标MPOS=%2，实测MPOS=%3，DPOS=%4，"
            "剩余误差=%5mm，IDLE=%6，已等待%7s。X定位/扫描使用独立速度。")
            .arg(axis).arg(targetMpos,0,'f',5).arg(mpos,0,'f',5)
            .arg(dpos,0,'f',5).arg(targetMpos-double(mpos),0,'f',5)
            .arg(idle?1:0).arg(elapsed/1000.0,0,'f',1));
    }else{
        small7SetStatus(tr("运动中DPOS/MPOS进度回读失败：轴%1，目标%2，%3；"
             "后台超时与停止保护仍然生效")
             .arg(axis).arg(targetMpos,0,'f',5).arg(error),true);
    }
    QTimer::singleShot(m_small7Phase==Small7Phase::XScan?120:1000,
        this,[this,generation,moveToken,axis,targetMpos]{
        small7PositionHeartbeat(generation,moveToken,axis,targetMpos);
    });
}

void MainWindow::small7MoveFinished(quint64 generation,int axis,double target,
      Small7Phase phase,bool success,const JointMoveResult5D& result,const QString& error){
    if(generation!=m_small7Generation||!m_small7Active)return;
    if(m_small7StopRequested)return small7Finish(false,m_small7FaultHistory.isEmpty()?
          tr("停机请求已执行"):m_small7FaultHistory);
    if(!success||!result.completed)
        return small7Finish(false,tr("阶段=%1、轴%2目标%3mm，运动未完成：%4")
            .arg(phaseText(axis,phase==Small7Phase::XScan)).arg(axis)
            .arg(target,0,'f',5).arg(error));
    QString arrival;
    if(!sphereStudyArrivalWithinTolerance(result,&arrival)||
       !fixedYCB(result.finalState.mpos,m_small7BasePose))
        return small7Finish(false,tr("轴%1到位/锁轴失败：目标%2，实测%3；%4")
           .arg(axis).arg(target,0,'f',5).arg(poseText(result.finalState.mpos),arrival));
    const auto& h=m_small7Heights[static_cast<std::size_t>(m_small7HeightIndex)];
    const auto sensor=SingleSphere17Model::machinePointInSensor(
       portablePose(result.finalState.mpos),m_sphereStudySphereFit.centerMachineMm,
       m_sphereStudyPlanningModel);
    if(!std::isfinite(sensor.x)||!std::isfinite(sensor.z))
        return small7Finish(false,tr("运动后Physical17预测传感器球心失败"));
    m_small7PredictedCx=sensor.x;m_small7PredictedCz=sensor.z;
    switch(phase){
    case Small7Phase::ZPosition:
        return small7Move(AxisX,h.scanStartXMachineMm,Small7Phase::XPosition);
    case Small7Phase::XPosition:{
        QString captureErr;
        m_small7Phase=Small7Phase::ScanWarmup;
        if(!small7BeginCapture(true,m_small7ProbeExposure,0,&captureErr))
            return small7Finish(false,tr("X扫描前实验3轮廓预热失败：%1").arg(captureErr));
        small7SetStatus(tr("SCAN_WARMUP：等待连续4帧，然后按%1mm/s（轮廓间距%2mm）完整扫描球的X截面")
            .arg(m_small7Settings.scanSpeedMmS,0,'f',3)
            .arg(m_small7Settings.requestedLineSpacingMm,0,'f',3));
        return;
    }
    case Small7Phase::XScan:
    {
        m_small7AxisSamples.push_back({GlobalTimeSync::instance().timestampUs(),
                                       result.finalState.mpos.x});
        small7StopCapture();
        if(m_small7FrameGap||m_small7ScanMotionGoodFrames<15||m_small7ScanMotionProfiles<20)
            return small7Finish(false,tr("SCAN_X无效：实际X运动期间有效圆弧帧=%1、运动轮廓=%2、"
                "总轮廓（含预热）=%3、序号丢帧=%4；不能使用扫描前预热帧冒充扫球数据")
                .arg(m_small7ScanMotionGoodFrames).arg(m_small7ScanMotionProfiles)
                .arg(m_small7ReceivedFrames).arg(m_small7FrameGap));
        if(m_small7MaxObservedFrameSpacingMm>m_small7Settings.maximumFrameSpacingMm+1e-9)
            return small7Finish(false,tr("扫描时实测相邻Profile回调时间对应位移=%1mm，超过上限%2mm。"
                "软件内部触发/PC回调无法保证间距时拒绝把该扫描用于峰位定位")
                .arg(m_small7MaxObservedFrameSpacingMm,0,'f',5)
                .arg(m_small7Settings.maximumFrameSpacingMm,0,'f',5));
        std::vector<s7::ScanPeakSample> labelled;
        QString scanCsv=QDir(m_small7SessionDir).filePath(
            QStringLiteral("Z%1_X_SCAN_MPOS_MAP.csv").arg(m_small7HeightIndex+1,3,10,QChar('0')));
        QFile mapped(scanCsv);
        if(!mapped.open(QIODevice::WriteOnly|QIODevice::Text))
            return small7Finish(false,tr("不能写X扫描实测MPOS时间映射：%1").arg(mapped.errorString()));
        mapped.write("ProfileCallbackPcUs,EstimatedMachineX_MPOS_mm,FittedArcRadius_mm\n");
        const auto xAtPc=[this](qint64 pcUs,double& x)->bool{
            // Host callback timestamp is only an estimate of exposure location.
            // Require feedback before and after; never invent an X coordinate.
            const auto upper=std::lower_bound(m_small7AxisSamples.begin(),m_small7AxisSamples.end(),pcUs,
                [](const Small7AxisSample& sample,qint64 t){return sample.pcUs<t;});
            if(upper==m_small7AxisSamples.begin()||upper==m_small7AxisSamples.end())return false;
            const auto lower=std::prev(upper);
            const qint64 width=upper->pcUs-lower->pcUs;
            if(width<=0 || width>400000)return false;
            const double alpha=double(pcUs-lower->pcUs)/double(width);
            x=lower->machineX+alpha*(upper->machineX-lower->machineX);
            return std::isfinite(x);
        };
        // IMPORTANT: check X spacing across ALL consecutively collected profiles,
        // not consecutive valid fitted arcs: the latter may have invalid arcs near
        // the edge of a sphere, which does not imply missing sensor frames.
        double previousX=0;bool havePreviousX=false;
        int mappedAll=0;
        for(const auto pcUs:m_small7ScanAllFramePcUs){
            double x=0;
            if(!xAtPc(pcUs,x))return small7Finish(false,tr("扫描轮廓无法全部以有界时差映射控制器MPOS：t=%1us；禁止用不确定位置估计最大圆").arg(pcUs));
            if(havePreviousX){
                const double dx=std::abs(x-previousX);
                m_small7MaxObservedFrameSpacingMm=std::max(m_small7MaxObservedFrameSpacingMm,dx);
                if(dx>m_small7Settings.maximumFrameSpacingMm+1e-9)
                    return small7Finish(false,tr("扫描相邻Profile估算MPOS帧距%1mm超过限制%2mm；拒绝定位")
                        .arg(dx,0,'f',5).arg(m_small7Settings.maximumFrameSpacingMm,0,'f',5));
            }
            previousX=x;havePreviousX=true;++mappedAll;
        }
        for(const auto& p:m_small7ScanProfiles){
            double x=0;
            if(!xAtPc(p.pcUs,x))
                return small7Finish(false,tr("有效圆弧缺少可靠MPOS映射，拒绝最大圆拟合"));
            labelled.push_back({x,p.radiusMm});
            const auto line=QStringLiteral("%1,%2,%3\n")
                .arg(p.pcUs).arg(x,0,'f',6).arg(p.radiusMm,0,'f',6).toUtf8();
            if(mapped.write(line)!=line.size())
                return small7Finish(false,tr("扫描映射数据写盘失败：%1").arg(mapped.errorString()));
        }
        if(!mapped.flush()||mapped.error()!=QFileDevice::NoError)
            return small7Finish(false,tr("X扫描映射文件写入失败：%1").arg(mapped.errorString()));
        mapped.close();
        s7::ScanPeak peak;std::string peakError;
        if(!s7::findScannedMaximumCircle(labelled,h.scanStartXMachineMm,h.scanEndXMachineMm,
            h.centerXMachineMm,m_small7KnownSphereRadius,m_small7DiameterToleranceMm,peak,&peakError))
            return small7Finish(false,tr("整球X扫描未能可靠拟合最大圆，不允许使用固定理论7点替代实测结果：%1；"
                "可映射有效帧=%2、轴位置样本=%3")
                .arg(QString::fromStdString(peakError)).arg(labelled.size()).arg(m_small7AxisSamples.size()));
        m_small7ScanPeakX=peak.machineX;
        m_small7ScanPeakRadius=peak.fittedRadiusMm;
        for(int k=0;k<7;++k){
            const double candidate=peak.machineX+(k-3)*m_small7Settings.sevenPointStepMm;
            if(candidate<h.scanStartXMachineMm || candidate>h.scanEndXMachineMm)
                return small7Finish(false,tr("扫描峰位周围静态细寻点超出认证X扫描区间"));
            m_small7ProbePositions[static_cast<std::size_t>(k)]=candidate;
        }
        QJsonObject scanSummary;
        scanSummary["schema"]="Experiment3.ScannedMaximum.v1";
        scanSummary["heightIndex"]=m_small7HeightIndex;
        scanSummary["machineZMm"]=h.machineZMm;
        scanSummary["nominalModelPeakXmm"]=h.centerXMachineMm;
        scanSummary["fittedScanPeakXmm"]=peak.machineX;
        scanSummary["scanPeakDiameterMm"]=peak.fittedRadiusMm*2;
        scanSummary["knownDiameterMm"]=m_small7KnownSphereRadius*2;
        scanSummary["mappedValidProfiles"]=peak.usedSamples;
        scanSummary["maxObservedProfileSpacingMm"]=m_small7MaxObservedFrameSpacingMm;
        scanSummary["mappedAllMotionProfileCount"]=mappedAll;
        scanSummary["mappingBasis"]="PC_sensor_callback_to_bracketed_controller_MPOS_interpolation_not_hardware_synced";
        scanSummary["mappedProfileCsv"]=scanCsv;
        QString scanErr;
        if(!writeJson(QDir(m_small7SessionDir).filePath(
            QStringLiteral("Z%1_scan_peak.json").arg(m_small7HeightIndex+1,3,10,QChar('0'))),
            scanSummary,&scanErr))
            return small7Finish(false,tr("扫描峰位归档失败：%1").arg(scanErr));
        small7SetStatus(tr("第%1层扫描估计最大圆：X≈%2mm，直径≈%3mm，扫描位置配对%4帧。"
            "开始7点静态确认，避免PC回调时延导致的扫描定位误差。")
            .arg(m_small7HeightIndex+1).arg(peak.machineX,0,'f',5)
            .arg(2*peak.fittedRadiusMm,0,'f',4).arg(peak.usedSamples));
        m_small7ProbeIndex=0;
        return small7Move(AxisX,m_small7ProbePositions[0],Small7Phase::ProbePosition);
    }
    case Small7Phase::ProbePosition:{
        // Wait for mechanical vibration to settle after the single-axis MPOS
        // arrival gate, then discard the operator-configured transient frames.
        const auto expectedGen=m_small7Generation;
        QTimer::singleShot(150,this,[this,expectedGen]{
            if(expectedGen!=m_small7Generation||!m_small7Active||m_small7StopRequested)return;
            JointState5D state;QString captureErr;
            if(!m_zmc->readJointState5D(state,&captureErr)||!state.allIdle){
                small7Finish(false,tr("静态寻峰前五轴未稳定：%1").arg(captureErr));return;
            }
            m_small7Phase=Small7Phase::ProbeCapture;
            if(!small7BeginCapture(false,m_small7ProbeExposure,m_small7ProbeFrames,&captureErr))
                small7Finish(false,tr("7点寻峰第%1点开始采集失败：%2")
                      .arg(m_small7ProbeIndex+1).arg(captureErr));
        });
        return;
    }
    case Small7Phase::ExposurePosition:
        m_small7ExposurePass=0;
        return QTimer::singleShot(150,this,[this]{small7StartExposureGroup();});
    default:
        return small7Finish(false,tr("异常运动回调阶段，拒绝继续"));
    }
}

bool MainWindow::small7BeginCapture(bool scanning,int exposure10us,int goodFrames,QString* error){
    if(!m_small7Active||m_small7StopRequested||m_small7Capturing){
        if(error)*error=tr("状态不允许启动新的实验3轮廓采集");return false;
    }
    if(m_small7Rate<25||
       m_small7Rate>s7::referenceMaxProfileHz(m_small7Settings.measuringFieldIndex)||
       exposure10us<=0||
       exposure10us>=static_cast<int>(std::ceil(100000.0/m_small7Rate))){
        if(error)*error=tr("当前场参考最高频率与曝光时间不兼容");return false;
    }
    const int idle=static_cast<int>(std::ceil(100000.0/m_small7Rate))-exposure10us;
    SensorParams params;params.measuringFieldIndex=m_small7Settings.measuringFieldIndex;
    params.exposureTime=static_cast<unsigned>(exposure10us);
    params.idleTime=static_cast<unsigned>(idle);
    if(m_comboResolution&&m_comboResolution->count()>0)
        params.resolutionIndex=static_cast<unsigned>(m_comboResolution->currentIndex());
    if(!m_laser->configure(SensorTriggerMode::SoftwareInternal,params,false,
        HardwareTriggerType::EdgeRising,error))return false;
    unsigned liveExposure=0,liveIdle=0;int liveField=-1;
    if(!m_laser->readTimingAndMeasuringField(liveExposure,liveIdle,liveField,error))return false;
    if(liveField!=params.measuringFieldIndex||liveExposure!=params.exposureTime||liveIdle!=params.idleTime){
        if(error)*error=tr("所选场设备回读不符合请求：field=%1（应%2）、Exp=%3（应%4）、Idle=%5（应%6）")
            .arg(liveField).arg(params.measuringFieldIndex)
            .arg(liveExposure).arg(params.exposureTime)
            .arg(liveIdle).arg(params.idleTime);
        return false;
    }
    const QString name=scanning?QStringLiteral("X_SCAN"):
        (m_small7Phase==Small7Phase::ProbeCapture?QStringLiteral("PROBE_%1")
              .arg(m_small7ProbeIndex+1,2,10,QChar('0')):
         QStringLiteral("EXPOSURE_%1_R%2")
              .arg(exposure10us,4,10,QChar('0'))
              .arg(m_small7ExposurePass%m_small7ExpRepeats+1,2,10,QChar('0')));
    m_small7GroupPrefix=QDir(m_small7SessionDir).filePath(QStringLiteral("Z%1_%2")
        .arg(m_small7HeightIndex+1,3,10,QChar('0')).arg(name));
    m_small7Raw=new QFile(m_small7GroupPrefix+".s7raw");
    m_small7Metrics=new QFile(m_small7GroupPrefix+"_arcs.csv");
    if(!m_small7Raw->open(QIODevice::WriteOnly)||
       !m_small7Metrics->open(QIODevice::WriteOnly|QIODevice::Text)){
        if(error)*error=tr("不能创建采集输出文件：%1 / %2")
                  .arg(m_small7Raw->errorString(),m_small7Metrics->errorString());
        small7StopCapture();return false;
    }
    m_small7Raw->write("S7P1",4);
    QDataStream ds(m_small7Raw);ds.setByteOrder(QDataStream::LittleEndian);
    ds.setFloatingPointPrecision(QDataStream::SinglePrecision);
    ds<<quint32(1)<<qint32(exposure10us)<<qint32(m_small7Rate)
      <<qint32(m_small7HeightIndex)<<qint32(scanning?1:0);
    m_small7Metrics->write("ProfileIndex,PcUs,Valid,CenterX,CenterZ,Radius,ArcRms,ArcSpanDeg,Inliers\n");
    m_small7Radii.clear();m_small7FrameGap=false;m_small7HasProfileIndex=false;
    m_small7ProfileGapEvents=0;m_small7ProfileMissingEstimate=0;
    m_small7CompletionQueued=false;m_small7ReceivedFrames=0;m_small7GoodFrames=0;
    m_small7RequiredFrames=goodFrames;
    m_laser->setSensorCalibrationRichProfileEnabled(false);
    if(!m_laser->startContinuous(error)){
        small7StopCapture();return false;
    }
    m_small7Capturing=true;
    m_small7LastProfileMs=QDateTime::currentMSecsSinceEpoch();
    const auto token=++m_small7CaptureToken;
    QTimer::singleShot(700,this,[this,token]{small7SensorWatchdog(token);});
    const int timeoutMs=scanning?10000:20000;
    QTimer::singleShot(timeoutMs,this,[this,token]{
        if(!m_small7Active||!m_small7Capturing||token!=m_small7CaptureToken)return;
        // X scanning has its own motion timeout after successful warm-up.
        if(m_small7Phase==Small7Phase::ScanWarmup||
           m_small7Phase==Small7Phase::ProbeCapture||
           m_small7Phase==Small7Phase::ExposureCapture)
            small7Finish(false,tr("实验3轮廓采集超时：阶段=%1，收到=%2，有效=%3，需求=%4，"
                "检查软件触发、当前模式频率、Measuring Field与轮廓解码")
                .arg(m_small7Phase==Small7Phase::ScanWarmup?tr("预热"):
                     (m_small7Phase==Small7Phase::ProbeCapture?tr("7点寻峰"):tr("静态曝光")))
                .arg(m_small7ReceivedFrames).arg(m_small7GoodFrames).arg(m_small7RequiredFrames));
    });
    small7SetStatus(tr("实验3采集：%1，field=%2，Exp=%3，Idle=%4，Rate≈%5Hz，需求帧数=%6（曝光统计含拟合无效帧）；RAW=%7")
         .arg(name).arg(liveField).arg(liveExposure).arg(liveIdle)
         .arg(100000.0/double(liveExposure+liveIdle),0,'f',2)
         .arg(goodFrames).arg(m_small7GroupPrefix));
    return true;
}

void MainWindow::small7StopCapture(){
    // Closing files is idempotent. Deferred queued callbacks see Capturing=false.
    m_small7Capturing=false;++m_small7CaptureToken;
    if(m_laser&&m_laser->isConnected()){
        m_laser->stopContinuous();m_laser->setSensorCalibrationRichProfileEnabled(false);
    }
    if(m_small7Raw){m_small7Raw->flush();m_small7Raw->close();delete m_small7Raw;m_small7Raw=nullptr;}
    if(m_small7Metrics){m_small7Metrics->flush();m_small7Metrics->close();
        delete m_small7Metrics;m_small7Metrics=nullptr;}
}

void MainWindow::small7SensorWatchdog(quint64 token){
    if(!m_small7Active||!m_small7Capturing||m_small7StopRequested||token!=m_small7CaptureToken)return;
    const auto age=QDateTime::currentMSecsSinceEpoch()-m_small7LastProfileMs;
    const bool liveScan=(m_small7Phase==Small7Phase::ScanWarmup||m_small7Phase==Small7Phase::XScan);
    const qint64 maxGapMs=liveScan?1500:3000;
    if(age>maxGapMs){
        small7Finish(false,tr("实验3轮廓流中断：%1ms没有收到有效Profile信号，"
            "phase=%2，已请求X/Z停轴；检查当前Measuring Field、软件触发、传感器网络与回调线程")
            .arg(age).arg(liveScan?QStringLiteral("SCAN"):QStringLiteral("STATIC")));
        return;
    }
    QTimer::singleShot(700,this,[this,token]{small7SensorWatchdog(token);});
}

void MainWindow::small7OnProfile(const LaserProfile& profile){
    if(!m_small7Active||!m_small7Capturing||!m_small7Raw||!m_small7Metrics)return;
    m_small7LastProfileMs=QDateTime::currentMSecsSinceEpoch();
    ++m_small7ReceivedFrames;
    const bool scanMotion=m_small7Phase==Small7Phase::XScan&&m_small7WorkerActive;
    if(scanMotion){
        ++m_small7ScanMotionProfiles;
        if(profile.tPcUs<=0)return small7Finish(false,tr("扫描轮廓缺少PC回调时间戳，无法与控制器MPOS配对"));
        m_small7ScanAllFramePcUs.push_back(profile.tPcUs);
        if(m_small7PreviousFrameUs>0){
            const qint64 dt=profile.tPcUs-m_small7PreviousFrameUs;
            if(dt<=0)return small7Finish(false,tr("扫描Profile回调时间不单调"));
            m_small7MaxObservedFrameSpacingMm=std::max(m_small7MaxObservedFrameSpacingMm,
                m_small7Settings.scanSpeedMmS*dt/1.0e6);
        }
        m_small7PreviousFrameUs=profile.tPcUs;
    }
    const quint32 delta=(profile.index-m_small7PreviousProfileIndex)&0x00ffffffu;
    if(m_small7HasProfileIndex&&delta!=1u){
        m_small7FrameGap=true;
        ++m_small7ProfileGapEvents;
        if(delta>1u)m_small7ProfileMissingEstimate=std::min(10000000,
            m_small7ProfileMissingEstimate+static_cast<int>(std::min(delta-1u,100000u)));
        // An exposure sweep MUST retain dark/overexposed conditions as data.
        // During scanning or peak positioning, a gap invalidates location/fit.
        if(m_small7Phase!=Small7Phase::ExposureCapture){
            QTimer::singleShot(0,this,[this]{if(m_small7Active)
                small7Finish(false,tr("实验3扫描或寻峰Profile索引存在缺口；已请求停止并保存RAW"));});
        }
    }
    m_small7HasProfileIndex=true;m_small7PreviousProfileIndex=profile.index;
    const int n=std::min(profile.x.size(),profile.z.size());
    std::vector<mls::calibration::SphereExposurePoint> points;
    points.reserve(static_cast<std::size_t>(n));
    for(int i=0;i<n;++i){
        const double x=profile.x[i],z=profile.z[i];
        const bool inVerifiedWindow=x>=m_small7Optics.xMinMm+m_small7Optics.marginMm &&
             x<=m_small7Optics.xMaxMm-m_small7Optics.marginMm &&
             z>=m_small7Optics.zMinMm+m_small7Optics.marginMm &&
             z<=m_small7Optics.zMaxMm-m_small7Optics.marginMm;
        const double invalid=std::numeric_limits<double>::quiet_NaN();
        points.push_back({inVerifiedWindow?x:invalid,inVerifiedWindow?z:invalid,i});
    }
    const auto arc=mls::calibration::SphereExposureSweep::fitArc(points,
        m_small7PredictedCx,m_small7PredictedCz,m_small7KnownSphereRadius);
    const bool unsettled=m_small7Phase!=Small7Phase::ScanWarmup &&
          m_small7Phase!=Small7Phase::XScan &&
          m_small7ReceivedFrames<=m_small7StaticDiscardFrames;
    if(arc.valid && !unsettled){
        ++m_small7GoodFrames;
        m_small7Radii.push_back(arc.radius);
        if(scanMotion){
            ++m_small7ScanMotionGoodFrames;
            m_small7ScanProfiles.push_back({profile.tPcUs,arc.radius});
        }
    }
    QDataStream raw(m_small7Raw);raw.setByteOrder(QDataStream::LittleEndian);
    raw.setFloatingPointPrecision(QDataStream::SinglePrecision);
    raw<<quint32(profile.index)<<qint64(profile.tPcUs)<<quint32(n);
    for(int i=0;i<n;++i)raw<<float(profile.x[i])<<float(profile.z[i]);
    const QString line=QStringLiteral("%1,%2,%3,%4,%5,%6,%7,%8,%9\n")
        .arg(profile.index).arg(profile.tPcUs).arg(arc.valid?1:0)
        .arg(arc.centerX,0,'f',6).arg(arc.centerZ,0,'f',6)
        .arg(arc.radius,0,'f',6).arg(arc.rms,0,'f',6)
        .arg(arc.arcSpanDeg,0,'f',3).arg(static_cast<qulonglong>(arc.inliers));
    m_small7Metrics->write(line.toUtf8());
    if(m_small7Raw->error()!=QFileDevice::NoError||
       m_small7Metrics->error()!=QFileDevice::NoError)
        return small7Finish(false,tr("实验3磁盘写入错误，禁止继续本层机床运动"));
    if(m_small7ReceivedFrames>120000)return small7Finish(false,tr("实验3单组轮廓超过120000帧"));
    if(m_small7Phase==Small7Phase::ScanWarmup&&m_small7ReceivedFrames>=4&&!m_small7CompletionQueued){
        m_small7CompletionQueued=true;
        QTimer::singleShot(0,this,[this]{
            if(!m_small7Active||m_small7StopRequested||
               m_small7Phase!=Small7Phase::ScanWarmup)return;
            const auto& h=m_small7Heights[static_cast<std::size_t>(m_small7HeightIndex)];
            small7Move(AxisX,h.scanEndXMachineMm,Small7Phase::XScan);
        });
    }
    const bool probeComplete=m_small7Phase==Small7Phase::ProbeCapture&&
        m_small7GoodFrames>=m_small7RequiredFrames;
    // Bad exposure belongs in the measured dataset: save the same fixed number
    // of actual post-settle profiles even when zero of their arcs can be fitted.
    const bool exposureComplete=m_small7Phase==Small7Phase::ExposureCapture&&
        m_small7ReceivedFrames>=m_small7RequiredFrames+m_small7StaticDiscardFrames;
    if((probeComplete||exposureComplete)&&!m_small7CompletionQueued){
        m_small7CompletionQueued=true;
        QTimer::singleShot(0,this,[this]{small7CaptureComplete();});
    }
}

void MainWindow::small7CaptureComplete(){
    if(!m_small7Active||m_small7StopRequested||!m_small7Capturing||
       (m_small7Phase!=Small7Phase::ProbeCapture&&m_small7Phase!=Small7Phase::ExposureCapture))return;
    const auto phase=m_small7Phase;
    const double r=median(m_small7Radii);
    const int good=m_small7GoodFrames,received=m_small7ReceivedFrames;
    const QString prefix=m_small7GroupPrefix;
    small7StopCapture();
    QJsonObject summary;
    summary["schema"]="Experiment3.MultiField.Pass.v3";
    summary["measuringFieldIndex"]=m_small7Settings.measuringFieldIndex;
    summary["profileRateHz"]=m_small7Rate;
    summary["verificationId"]=QString::fromStdString(m_small7Optics.verificationId);
    summary["heightIndex"]=m_small7HeightIndex;
    summary["machineZMm"]=m_small7Heights[static_cast<std::size_t>(m_small7HeightIndex)].machineZMm;
    summary["stage"]=phase==Small7Phase::ProbeCapture?"seven_point_probe":"static_exposure";
    summary["scanMaximumXMachineMm"]=m_small7ScanPeakX;
    summary["refinedMaximumXMachineMm"]=std::isfinite(m_small7ChosenPeakX)?
        m_small7ChosenPeakX:m_small7ScanPeakX;
    summary["diameterErrorToReferenceMm"]=2*r-2*m_small7KnownSphereRadius;
    summary["configuredDiameterToleranceMm"]=m_small7DiameterToleranceMm;
    if(phase==Small7Phase::ExposureCapture){
        summary["exposure10us"]=m_small7ExpStart+(m_small7ExposurePass/m_small7ExpRepeats)*m_small7ExpStep;
        summary["repeatIndex"]=m_small7ExposurePass%m_small7ExpRepeats+1;
    }else summary["exposure10us"]=m_small7ProbeExposure;
    summary["goodFrames"]=good;summary["receivedFrames"]=received;
    summary["discardedInitialStaticFrames"]=m_small7StaticDiscardFrames;
    summary["medianRadiusMm"]=r;summary["medianDiameterMm"]=2*r;
    summary["measurementProfilesRequested"]=m_small7RequiredFrames;
    summary["fitValidFractionAfterWarmup"]=double(good)/std::max(1,received-m_small7StaticDiscardFrames);
    summary["diameterWithinTolerance"]=good>0&&std::abs(2*r-2*m_small7KnownSphereRadius)<=m_small7DiameterToleranceMm;
    summary["measurementResultValid"]=good>0;
    summary["missingFitMeansNoDiameter"]=good==0;
    summary["frameGap"]=m_small7FrameGap;
    summary["profileIndexGapEvents"]=m_small7ProfileGapEvents;
    summary["estimatedMissingProfileIndices"]=m_small7ProfileMissingEstimate;
    summary["rawPath"]=prefix+".s7raw";
    QString err;
    if(!writeJson(prefix+"_summary.json",summary,&err))
        return small7Finish(false,tr("无法保存实验3曝光结果：%1").arg(err));
    if(phase==Small7Phase::ProbeCapture &&
       (m_small7FrameGap||good<m_small7RequiredFrames||r<=0))
        return small7Finish(false,tr("实验3寻峰帧质量不足或丢帧：有效%1/%2、R=%3mm")
                 .arg(good).arg(received).arg(r,0,'f',5));
    const auto& h=m_small7Heights[static_cast<std::size_t>(m_small7HeightIndex)];
    if(phase==Small7Phase::ProbeCapture){
        m_small7ProbeBestRadii[static_cast<std::size_t>(m_small7ProbeIndex)]=r;
        small7SetStatus(tr("7点寻峰 第%1/7点：X=%2mm，圆弧中位半径=%3mm，有效%4/%5")
             .arg(m_small7ProbeIndex+1).arg(m_small7ProbePositions[static_cast<std::size_t>(m_small7ProbeIndex)],0,'f',5)
             .arg(r,0,'f',5).arg(good).arg(received));
        ++m_small7ProbeIndex;
        if(m_small7ProbeIndex<7)return small7Move(AxisX,
           m_small7ProbePositions[static_cast<std::size_t>(m_small7ProbeIndex)],Small7Phase::ProbePosition);
        const auto best=std::max_element(m_small7ProbeBestRadii.begin(),m_small7ProbeBestRadii.end());
        if(std::abs(2*(*best)-2*m_small7KnownSphereRadius)>m_small7DiameterToleranceMm)
            return small7Finish(false,tr("该Z的7点静态实测最大直径%1mm与已知球直径%2mm差异超出%3mm；"
                "不能在未获得完整最大圆的位置进行曝光比较")
                .arg(2*(*best),0,'f',4).arg(2*m_small7KnownSphereRadius,0,'f',4)
                .arg(m_small7DiameterToleranceMm,0,'f',3));
        const int index=static_cast<int>(std::distance(m_small7ProbeBestRadii.begin(),best));
        m_small7ChosenPeakX=m_small7ProbePositions[static_cast<std::size_t>(index)];
        small7SetStatus(tr("7点寻峰完成：最大截面候选点%1，X=%2mm，R≈%3mm；准备静止曝光")
            .arg(index+1).arg(m_small7ChosenPeakX,0,'f',5).arg(*best,0,'f',5));
        return small7Move(AxisX,m_small7ChosenPeakX,Small7Phase::ExposurePosition);
    }
    const int exposure=m_small7ExpStart+(m_small7ExposurePass/m_small7ExpRepeats)*m_small7ExpStep;
    m_small7LastExposureRadiusMm=r;
    m_small7LastExposureGoodFrames=good;
    m_small7LastExposureReceivedFrames=received;
    m_small7LastExposureGapEvents=m_small7ProfileGapEvents;
    m_small7LastExposureMissingEstimate=m_small7ProfileMissingEstimate;
    small7SetStatus(tr("Z层%1静止曝光 Exp=%2×10µs、重复=%3，圆弧中位半径=%4mm，"
        "有效帧=%5/%6（低质量曝光仍归档）：已保存RAW和指标")
        .arg(m_small7HeightIndex+1).arg(exposure)
        .arg(m_small7ExposurePass%m_small7ExpRepeats+1)
        .arg(r,0,'f',5).arg(good).arg(received));
    if(m_small7VideoEnabled){
        QTimer::singleShot(100,this,[this]{small7StartVideoGroup();});
        return;
    }
    small7FinishExposurePass();
}

void MainWindow::small7FinishExposurePass(){
    if(!m_small7Active||m_small7StopRequested)return;
    const int exposure=m_small7ExpStart+(m_small7ExposurePass/m_small7ExpRepeats)*m_small7ExpStep;
    const auto& h=m_small7Heights[static_cast<std::size_t>(m_small7HeightIndex)];
    QFile comparison(QDir(m_small7SessionDir).filePath(QStringLiteral("experiment3_Z_exposure_comparison.csv")));
    if(!comparison.open(QIODevice::WriteOnly|QIODevice::Append|QIODevice::Text))
        return small7Finish(false,tr("实验3跨Z/曝光结果CSV无法追加：%1").arg(comparison.errorString()));
    const QString record=QStringLiteral("%1,%2,%3,%4,%5,%6,%7,%8,%9,%10,%11,%12,%13,%14,%15,%16,%17,%18\n")
        .arg(m_small7HeightIndex+1)
        .arg(h.machineZMm,0,'f',5).arg(m_small7Settings.measuringFieldIndex)
        .arg(m_small7ScanPeakX,0,'f',6).arg(m_small7ChosenPeakX,0,'f',6)
        .arg(exposure).arg(m_small7ExposurePass%m_small7ExpRepeats+1)
        .arg(m_small7Rate).arg(m_small7ExposureFrames)
        .arg(m_small7LastExposureReceivedFrames).arg(m_small7LastExposureGoodFrames)
        .arg(double(m_small7LastExposureGoodFrames)/std::max(1,m_small7LastExposureReceivedFrames-m_small7StaticDiscardFrames),0,'f',6)
        .arg(2*m_small7LastExposureRadiusMm,0,'f',6)
        .arg(2*m_small7LastExposureRadiusMm-2*m_small7KnownSphereRadius,0,'f',6)
        .arg(m_small7LastExposureGapEvents).arg(m_small7LastExposureMissingEstimate)
        .arg(m_small7VideoEnabled?m_small7VideoSaved:0)
        // Quote CSV paths, which can contain commas or quotes.
        .arg(QStringLiteral("\"")+QString(m_small7GroupPrefix).replace('"',QStringLiteral("\"\""))+
             QStringLiteral("\""));
    if(comparison.write(record.toUtf8())<0||!comparison.flush())
        return small7Finish(false,tr("实验3对比CSV写盘失败：%1").arg(comparison.errorString()));
    ++m_small7ExposurePass;
    const int levels=(m_small7ExpEnd-m_small7ExpStart)/m_small7ExpStep+1;
    if(m_small7ExposurePass<levels*m_small7ExpRepeats)
        return QTimer::singleShot(80,this,[this]{small7StartExposureGroup();});
    m_tableSmall7Plan->item(m_small7HeightIndex,6)->setText(tr("已完成静止曝光"));
    ++m_small7HeightIndex;
    QTimer::singleShot(80,this,[this]{small7AdvanceHeight();});
}

void MainWindow::small7StartVideoGroup(){
    if(!m_small7Active||m_small7StopRequested||!m_small7VideoEnabled)return;
    if(m_small7Capturing||m_small7WorkerActive||m_laser->isProfileRunning())
        return small7Finish(false,tr("Video开始时轮廓采集或机床运动尚未停止"));
    JointState5D state;QString err;
    if(!m_zmc->readJointState5D(state,&err)||!state.allIdle||
       !fixedYCB(state.mpos,m_small7BasePose)||
       std::abs(state.mpos.x-m_small7ChosenPeakX)>0.005||
       std::abs(state.mpos.z-m_small7Heights[static_cast<std::size_t>(m_small7HeightIndex)].machineZMm)>0.005)
        return small7Finish(false,tr("Video前五轴未停稳/离开最大圆位置：%1").arg(err));
    const int exposure=m_small7ExpStart+(m_small7ExposurePass/m_small7ExpRepeats)*m_small7ExpStep;
    // Small7 auxiliary Video also uses the LLT 29xx Video 25Hz clock, not
    // the Small7 Profile trigger frequency.
    const auto videoTiming = mls::calibration::SensorCalibrationTiming::videoPlan(exposure);
    if (!videoTiming.valid) return small7Finish(false, tr("Video 25Hz曝光非法"));
    SensorParams params;params.exposureTime=static_cast<unsigned>(exposure);
    params.idleTime=static_cast<unsigned>(videoTiming.idle10us);
    // The 2950 Video transport is independent of the chosen Profile field;
    // no measured circle is derived from these supplemental video frames.
    if(!m_laser->configureVideo(params,&err))
        return small7Finish(false,tr("附加Video模式配置失败：%1").arg(err));
    unsigned liveExposure=0,liveIdle=0;int liveField=-1;
    if(!m_laser->readTimingAndMeasuringField(liveExposure,liveIdle,liveField,&err)||
       liveExposure!=params.exposureTime||liveIdle!=params.idleTime)
        return small7Finish(false,tr("附加Video曝光/空闲时间回读不一致：%1").arg(err));
    m_small7Phase=Small7Phase::VideoCapture;
    m_small7VideoReceived=0;m_small7VideoSaved=0;
    const auto token=++m_small7VideoToken;
    m_small7VideoConnection=connect(m_laser.get(),&LaserSensor::videoFrameReceived,this,
        [this,token](const QImage& frame){
            if(!m_small7Active||m_small7StopRequested||m_small7Phase!=Small7Phase::VideoCapture||
               token!=m_small7VideoToken)return;
            ++m_small7VideoReceived;
            if(m_small7VideoReceived<=3)return; // discard after Profile→Video switch
            if(frame.isNull())return small7Finish(false,tr("实验3收到空Video图像"));
            const QString fileName=m_small7GroupPrefix+QStringLiteral("_video_%1.png")
               .arg(m_small7VideoSaved+1,3,10,QChar('0'));
            if(!frame.save(fileName,"PNG"))
                return small7Finish(false,tr("实验3Video图像保存失败：%1").arg(fileName));
            ++m_small7VideoSaved;
            if(m_small7VideoSaved>=m_small7VideoFramesPerPass){
                small7StopVideo();
                small7FinishExposurePass();
            }
        },Qt::QueuedConnection);
    if(!m_laser->startVideoStream(&err))
        return small7Finish(false,tr("实验3Video流启动失败：%1").arg(err));
    QTimer::singleShot(10000+m_small7VideoFramesPerPass*1000,this,[this,token]{
        if(m_small7Active&&!m_small7StopRequested&&m_small7Phase==Small7Phase::VideoCapture&&
           token==m_small7VideoToken)
            small7Finish(false,tr("实验3Video图像收集超时：保存%1/%2")
                .arg(m_small7VideoSaved).arg(m_small7VideoFramesPerPass));
    });
    small7SetStatus(tr("静态轮廓归档完成，独立切换Video Exp=%1×10µs，保存%2张PNG（先丢弃3帧）")
        .arg(exposure).arg(m_small7VideoFramesPerPass));
}

void MainWindow::small7StopVideo(){
    ++m_small7VideoToken;
    if(m_small7VideoConnection){QObject::disconnect(m_small7VideoConnection);m_small7VideoConnection={};}
    if(m_laser&&m_laser->isVideoRunning())m_laser->stopVideoStream();
}

void MainWindow::small7StartExposureGroup(){
    if(!m_small7Active||m_small7StopRequested)return;
    JointState5D state;QString err;
    if(!m_zmc->readJointState5D(state,&err)||!state.allIdle||
       !fixedYCB(state.mpos,m_small7BasePose)||
       !std::isfinite(m_small7ChosenPeakX)||
       std::abs(state.mpos.x-m_small7ChosenPeakX)>0.005||
       std::abs(state.mpos.z-m_small7Heights[static_cast<std::size_t>(m_small7HeightIndex)].machineZMm)>0.005)
        return small7Finish(false,tr("逐曝光要求XYZCB完全静止、X峰位及机床Z锁定：%1；峰X=%2mm，%3")
            .arg(poseText(state.mpos)).arg(m_small7ChosenPeakX,0,'f',5).arg(err));
    const int exposure=m_small7ExpStart+(m_small7ExposurePass/m_small7ExpRepeats)*m_small7ExpStep;
    m_small7VideoSaved=0;
    m_small7Phase=Small7Phase::ExposureCapture;
    if(!small7BeginCapture(false,exposure,m_small7ExposureFrames,&err))
        return small7Finish(false,tr("静态曝光=%1配置/采集失败：%2").arg(exposure).arg(err));
    m_tableSmall7Plan->item(m_small7HeightIndex,6)->setText(
        tr("曝光%1，第%2次").arg(exposure).arg(m_small7ExposurePass%m_small7ExpRepeats+1));
}

void MainWindow::onStopSmall7Experiment(){
    if(!m_small7Active)return;
    m_small7StopRequested=true;
    m_small7FaultHistory=tr("操作者停止第三实验");
    small7StopCapture();
    small7StopVideo();
    if(m_zmc&&m_zmc->isConnected()){
        m_zmc->requestStopAxis(AxisX,false);
        m_zmc->requestStopAxis(AxisZ,false);
    }
    small7SetStatus(tr("已发送X/Z停轴命令，正在等待运动工作线程安全退出"),true);
    if(!m_small7WorkerActive)small7Finish(false,m_small7FaultHistory);
}

void MainWindow::small7Finish(bool success,const QString& reason){
    if(!m_small7Active)return;
    if(m_small7WorkerActive){
        m_small7StopRequested=true;
        m_small7FaultHistory=reason;
        small7StopCapture();
        small7StopVideo();
        if(m_zmc&&m_zmc->isConnected()){
            m_zmc->requestStopAxis(AxisX,false);m_zmc->requestStopAxis(AxisZ,false);
        }
        return; // never free hardware while its blocking move worker still runs
    }
    small7StopCapture();
    small7StopVideo();
    if(m_small7ProfileConnection){
        QObject::disconnect(m_small7ProfileConnection);m_small7ProfileConnection={};
    }
    QJsonObject completion;
    completion["schema"]="Experiment3.MultiField.Completion.v3";
    completion["success"]=success;completion["reason"]=reason;
    completion["heightCompletedCount"]=m_small7HeightIndex;
    completion["heightTotalCount"]=static_cast<int>(m_small7Heights.size());
    completion["utc"]=QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs);
    QString writeError;
    if(!m_small7SessionDir.isEmpty())
        writeJson(QDir(m_small7SessionDir).filePath("completion.json"),completion,&writeError);
    if(!success&&m_tableSmall7Plan&&m_small7HeightIndex<m_tableSmall7Plan->rowCount())
        m_tableSmall7Plan->item(m_small7HeightIndex,6)->setText(tr("故障中止"));
    m_small7Active=false;m_small7StopRequested=false;m_small7PlanValid=false;
    m_small7Phase=Small7Phase::Idle;++m_small7Generation;
    setJointMoveUiRunning(false);
    for(auto* widget:{m_spinSmall7ZStart,m_spinSmall7ZEnd,m_spinSmall7ZInterval,m_spinSmall7MaxSpacing,
                     m_spinSmall7LineSpacing,m_spinSmall7DiameterTolerance})widget->setEnabled(true);
    for(auto* widget:{m_spinSmall7Rate,m_spinSmall7ProbeExposure,m_spinSmall7ProbeFrames,
                     m_spinSmall7ExposureFrames,m_spinSmall7StaticDiscard,m_spinSmall7Repeats,m_spinSmall7ExpStart,
                     m_spinSmall7ExpEnd,m_spinSmall7ExpStep})widget->setEnabled(true);
    m_spinSmall7VideoFrames->setEnabled(m_checkSmall7SaveVideo->isChecked());
    m_checkSmall7SaveVideo->setEnabled(true);m_editSmall7OutputDir->setEnabled(true);
    m_comboSmall7PositionSpeed->setEnabled(true);
    m_comboSmall7Field->setEnabled(true);
    m_editSmall7OpticalConfig->setEnabled(true);
    m_btnSmall7Browse->setEnabled(true);
    m_btnSmall7Start->setEnabled(false);m_btnSmall7Stop->setEnabled(false);
    m_btnSmall7Plan->setEnabled(true);
    updateVideoControlState();
    small7SetStatus(tr("%1。完成高度%2/%3；归档：%4。%5")
       .arg(reason).arg(m_small7HeightIndex).arg(m_small7Heights.size())
       .arg(m_small7SessionDir).arg(writeError.isEmpty()?QString():tr("归档错误：%1").arg(writeError)),!success);
}
