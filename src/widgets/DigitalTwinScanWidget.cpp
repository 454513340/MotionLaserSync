#include "twin/TwinH1Geometry.h"
#include "widgets/DigitalTwinScanWidget.h"
#include "widgets/PointCloudWidget.h"
#include "twin/CadGeometryAdapter.h"
#include "twin/ScanReplayAdapter.h"
#include "twin/TwinSimulationAdapter.h"
#include "twin/TwinM4Adapter.h"
#include "twin/TwinM5Adapter.h"
#include "twin/TwinM51Adapter.h"
#include "twin/TwinM52Adapter.h"
#include "twin/TwinM6Adapter.h"
#include "twin/TwinM7Adapter.h"
#include "twin/TwinM8Workflow.h"
#include <QCheckBox>
#include <QCoreApplication>
#include <QCloseEvent>
#include <QDateTime>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QGridLayout>
#include <QHeaderView>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QAbstractItemView>
#include <QStringList>
#include <QColor>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QSlider>
#include <QSpinBox>
#include <QPlainTextEdit>
#include <QProgressBar>
#include <QComboBox>
#include <QTimer>
#include <QFutureWatcher>
#include <QtConcurrent>
#include <QFileInfo>
#include <QMessageBox>
#include <QPushButton>
#include <QSaveFile>
#include <QTextStream>
#include <QStringConverter>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QTemporaryDir>
#include <QSplitter>
#include <QVBoxLayout>
#include <QDir>
#include <QApplication>
#include <utility>
#include <initializer_list>
#include <unordered_set>
#include <string>
#include <algorithm>
#include <cmath>
#include <exception>

using mls::twin::Pose6;
namespace {
QString fmt(double x){return QString::number(x,'f',3);}
QJsonObject jsonPose(const Pose6& p){
    return {{"x",p.x},{"y",p.y},{"z",p.z},
            {"rxDeg",p.rxDeg},{"ryDeg",p.ryDeg},{"rzDeg",p.rzDeg}};
}
bool readPose(const QJsonObject& o,Pose6* p){
    if(!p)return false;
    for(const char* k:{"x","y","z","rxDeg","ryDeg","rzDeg"}){
        if(!o.contains(QLatin1String(k))||!o.value(QLatin1String(k)).isDouble())return false;
    }
    *p={o.value("x").toDouble(),o.value("y").toDouble(),o.value("z").toDouble(),
        o.value("rxDeg").toDouble(),o.value("ryDeg").toDouble(),o.value("rzDeg").toDouble()};
    return true;
}
QString localPath(const QString& s){return QDir::toNativeSeparators(s);}
std::string nativePath(const QString& s){
    // STEPControl_Reader accepts a narrow path; toLocal8Bit supports common
    // Windows ANSI file names (including GBK on Chinese Windows installations).
    return localPath(s).toLocal8Bit().toStdString();
}
}
DigitalTwinScanWidget::DigitalTwinScanWidget(QWidget* parent):QDialog(parent){
    setWindowTitle(tr("MotionLaserSync V28.9 | M0～M8 连续五轴数字孪生与验收（离线）"));
    setMinimumSize(1000,650);
    resize(1280,800);
    buildUI();
    setUiPose(Pose6{}); // M0-M5: do NOT silently reuse last experiments' fixture placement
    updateSupport();
    refreshInfo(tr("导入 STEP 设置工件位置；M2 可导入真实 SCAN v3 回放五轴位置及点云。全程离线，不运行真实扫描。"));
}

void DigitalTwinScanWidget::buildUI(){
    auto* outer=new QVBoxLayout(this);
    auto* title=new QLabel(tr("数字孪生场景配置  |  坐标单位：mm、角度：°  |  无设备指令"),this);
    title->setStyleSheet("QLabel {font-weight:bold; padding:6px; color:#206070; background:#e9f5fb;}");
    outer->addWidget(title);
    auto* splitter=new QSplitter(Qt::Horizontal,this);
    auto* scroll=new QScrollArea(splitter);
    scroll->setWidgetResizable(true);
    auto* panel=new QWidget(scroll);
    auto* vbox=new QVBoxLayout(panel);
    scroll->setWidget(panel);
    scroll->setMinimumWidth(340);
    scroll->setMaximumWidth(490);

    auto* cadGroup=new QGroupBox(tr("1. STEP 工件 / CAD 几何"),panel);
    auto* cadLayout=new QVBoxLayout(cadGroup);
    auto* buttons=new QHBoxLayout;
    auto* importButton=new QPushButton(tr("导入 STEP…"),cadGroup);
    auto* exampleButton=new QPushButton(tr("载入测试零件"),cadGroup);
    buttons->addWidget(importButton);buttons->addWidget(exampleButton);
    cadLayout->addLayout(buttons);
    auto* loadObjButton=new QPushButton(tr("导入预三角化 OBJ（离线调试）…"),cadGroup);
    cadLayout->addWidget(loadObjButton);
    m_fileLabel=new QLabel(tr("当前未加载零件"),cadGroup);
    m_fileLabel->setWordWrap(true);m_fileLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
    cadLayout->addWidget(m_fileLabel);
    auto* toleranceRow=new QHBoxLayout;
    toleranceRow->addWidget(new QLabel(tr("STEP 三角化弦高(mm):"),cadGroup));
    m_meshTolerance=new QDoubleSpinBox(cadGroup);
    m_meshTolerance->setRange(0.01,3.0);m_meshTolerance->setDecimals(3);m_meshTolerance->setSingleStep(0.05);m_meshTolerance->setValue(0.12);
    toleranceRow->addWidget(m_meshTolerance);cadLayout->addLayout(toleranceRow);
    cadLayout->addWidget(new QLabel(mls::twin::CadGeometryAdapter::stepSupported()
      ? tr("Open CASCADE: 已启用，支持真正 STEP B-Rep 导入")
      : tr("Open CASCADE: 未检测到。STEP 导入暂不可用，示例 OBJ 可以正常使用。"),cadGroup));
    vbox->addWidget(cadGroup);

    auto* poseGroup=new QGroupBox(tr("2. CAD → 机床坐标 (MCS) 六自由度放置"),panel);
    auto* form=new QFormLayout(poseGroup);
    const char* labels[]={"X (mm)","Y (mm)","Z (mm)","Rx (deg)","Ry (deg)","Rz (deg)"};
    for(int i=0;i<6;++i){
        m_pose[i]=new QDoubleSpinBox(poseGroup);
        m_pose[i]->setDecimals(4);
        m_pose[i]->setRange(i<3 ? -10000.0 : -360.0, i<3 ? 10000.0 : 360.0);
        m_pose[i]->setSingleStep(i<3?0.1:0.5);
        form->addRow(tr(labels[i]),m_pose[i]);
        connect(m_pose[i],qOverload<double>(&QDoubleSpinBox::valueChanged),this,[this](double){updatePlacement();});
    }
    m_idealRegistration=new QCheckBox(tr("理想定位：规划估计位姿 = 场景真值"),poseGroup);
    m_idealRegistration->setChecked(true);
    m_idealRegistration->setToolTip(tr("只适用于理想仿真；真实 SCAN 的工件位姿必须独立配准。启用后将估计位置直接设为场景占位真值。"));
    form->addRow(m_idealRegistration);
    auto* poseButtons=new QHBoxLayout;
    auto* zeroPose=new QPushButton(tr("CAD 原点=机床原点"),poseGroup);
    auto* referencePose=new QPushButton(tr("旧 H1 参考位置"),poseGroup);
    auto* newH1Pose=new QPushButton(tr("新 H1 四姿态配准位置（初步）"),poseGroup);
    poseButtons->addWidget(zeroPose);poseButtons->addWidget(referencePose);form->addRow(poseButtons);
    form->addRow(newH1Pose);
    auto* poseNote=new QLabel(tr("采用 p_M = T_M←CAD × p_CAD；旋转顺序 Rz·Ry·Rx（固定机床轴）。这不是机床 B/C 轴关节角。"),poseGroup);
    poseNote->setWordWrap(true);form->addRow(poseNote);
    connect(zeroPose,&QPushButton::clicked,this,[this]{setUiPose(Pose6{});updatePlacement();});
    connect(referencePose,&QPushButton::clicked,this,[this]{setUiPose({44.3195,10.2369,-225.05,0,0,-89.2265});updatePlacement();});
    connect(newH1Pose,&QPushButton::clicked,this,[this]{
        const auto geom=mls::twin::inferH1Geometry(m_session.scene);
        if(!geom.valid||std::abs(geom.outerRadius-3.35)>.035){
            QMessageBox::warning(this,tr("CAD 不一致"),tr("此位置仅对应当前四姿态实验的新 H1 STEP（中心孔外半径 3.35 mm）。"));return;
        }
        const Pose6 fitted{6.8750148,8.9513313,-235.2629748,-0.0259378,0.2286994,-89.4160791};
        setUiPose(fitted);
        m_idealRegistration->setChecked(false); // experimental fit is NOT an observed ground truth
        m_session.scene.setEstimatedPose(fitted);
        updatePlacement();
        refreshInfo(tr("新 H1 初步六自由度配准已载入。非计量真值，使用前须确认 STEP 和装夹状态。"));
    });
    connect(m_idealRegistration,&QCheckBox::toggled,this,[this](bool enabled){if(enabled)updatePlacement();});
    vbox->addWidget(poseGroup);

    auto* supportGroup=new QGroupBox(tr("3. 支承面（机床 XY 平面，可选）"),panel);
    auto* supportForm=new QFormLayout(supportGroup);
    m_supportCheck=new QCheckBox(tr("启用支承面"),supportGroup);
    m_supportCheck->setChecked(false); // default OFF until support/fixture is characterized
    supportForm->addRow(m_supportCheck);
    auto addParam=[&](const QString& label,double value,double min,double max){
        auto* spin=new QDoubleSpinBox(supportGroup);
        spin->setDecimals(3);spin->setRange(min,max);spin->setValue(value);spin->setSingleStep(.5);
        supportForm->addRow(label,spin);
        connect(spin,qOverload<double>(&QDoubleSpinBox::valueChanged),this,[this](double){updateSupport();});
        return spin;
    };
    m_supportZ=addParam(tr("Z (mm)"),-225.05,-10000,10000);
    m_supportX=addParam(tr("中心 X (mm)"),42.0,-10000,10000);
    m_supportY=addParam(tr("中心 Y (mm)"),10.0,-10000,10000);
    m_supportW=addParam(tr("X 尺寸 (mm)"),140.0,.01,10000);
    m_supportH=addParam(tr("Y 尺寸 (mm)"),90.0,.01,10000);
    connect(m_supportCheck,&QCheckBox::toggled,this,[this](bool){updateSupport();});
    vbox->addWidget(supportGroup);

    auto* actions=new QGroupBox(tr("4. 会话 / 显示"),panel);
    auto* av=new QVBoxLayout(actions);
    auto* saveButton=new QPushButton(tr("保存场景与位姿 JSON…"),actions);
    auto* openButton=new QPushButton(tr("载入场景与位姿 JSON…"),actions);
    auto* machineButton=new QPushButton(tr("尝试加载机床视觉模型（可选）"),actions);
    auto* cameraButton=new QPushButton(tr("聚焦弹窗工件"),actions);
    auto* mainFocusButton=new QPushButton(tr("同步并聚焦【主界面】三维工件"),actions);
    av->addWidget(saveButton);av->addWidget(openButton);av->addWidget(machineButton);av->addWidget(cameraButton);av->addWidget(mainFocusButton);
    vbox->addWidget(actions);

    auto* replayGroup=new QGroupBox(tr("5. M2：真实 SCAN v3 离线回放（非虚拟光学扫描）"),panel);
    auto* replayLayout=new QVBoxLayout(replayGroup);
    m_replayOpen=new QPushButton(tr("导入实测 .scan…"),replayGroup);
    m_replayPlay=new QPushButton(tr("播放"),replayGroup);
    auto* pauseButton=new QPushButton(tr("暂停"),replayGroup);
    auto* resetButton=new QPushButton(tr("回到第一帧"),replayGroup);
    m_replayExport=new QPushButton(tr("导出完整重建 PLY…"),replayGroup);
    m_replayPlay->setEnabled(false);
    m_replayExport->setEnabled(false);
    auto* rr=new QHBoxLayout;
    rr->addWidget(m_replayOpen);rr->addWidget(m_replayPlay);rr->addWidget(pauseButton);
    replayLayout->addLayout(rr);
    auto* rr2=new QHBoxLayout;
    rr2->addWidget(resetButton);rr2->addWidget(new QLabel(tr("回放速率:"),replayGroup));
    m_replaySpeed=new QComboBox(replayGroup);
    for(const auto& p:std::initializer_list<std::pair<const char*,double>>{
        {"0.25×",.25},{"1×",1.0},{"4×",4.0},{"20×",20.0}})
        m_replaySpeed->addItem(QString::fromLatin1(p.first),p.second);
    m_replaySpeed->setCurrentIndex(1);
    rr2->addWidget(m_replaySpeed);replayLayout->addLayout(rr2);
    m_replaySlider=new QSlider(Qt::Horizontal,replayGroup);
    m_replaySlider->setRange(0,0);m_replaySlider->setEnabled(false);
    replayLayout->addWidget(m_replaySlider);
    replayLayout->addWidget(m_replayExport);
    m_replayInfo=new QLabel(tr("未导入 SCAN；回放显示最多25万预览点，导出不降采样。"),replayGroup);
    m_replayInfo->setWordWrap(true);
    m_replayInfo->setTextInteractionFlags(Qt::TextSelectableByMouse);
    replayLayout->addWidget(m_replayInfo);
    vbox->addWidget(replayGroup);

    auto* clearGroup=new QGroupBox(tr("6. 分层清除 / 恢复工作区（不清除真实采集点云）"),panel);
    auto* clearLayout=new QGridLayout(clearGroup);
    auto* clearCadButton=new QPushButton(tr("清除 CAD 工件"),clearGroup);
    auto* clearSupportButton=new QPushButton(tr("清除支承面"),clearGroup);
    auto* clearReplayButton=new QPushButton(tr("清除 M2 实测回放"),clearGroup);
    auto* clearVirtualButton=new QPushButton(tr("清除 M3 虚拟点云"),clearGroup);
    auto* clearAllButton=new QPushButton(tr("一键清空数字孪生数据"),clearGroup);
    clearLayout->addWidget(clearCadButton,0,0);clearLayout->addWidget(clearSupportButton,0,1);
    clearLayout->addWidget(clearReplayButton,1,0);clearLayout->addWidget(clearVirtualButton,1,1);
    clearLayout->addWidget(clearAllButton,2,0,1,2);
    connect(clearCadButton,&QPushButton::clicked,this,&DigitalTwinScanWidget::clearWorkpiece);
    connect(clearSupportButton,&QPushButton::clicked,this,&DigitalTwinScanWidget::clearSupport);
    connect(clearReplayButton,&QPushButton::clicked,this,&DigitalTwinScanWidget::clearReplay);
    connect(clearVirtualButton,&QPushButton::clicked,this,&DigitalTwinScanWidget::clearVirtual);
    connect(clearAllButton,&QPushButton::clicked,this,&DigitalTwinScanWidget::clearAllTwinData);
    vbox->addWidget(clearGroup);

    auto* virtualGroup=new QGroupBox(tr("7. M3：虚拟 2950-50/BL Profile 扫描（光学几何近似）"),panel);
    auto* virtualLayout=new QVBoxLayout(virtualGroup);
    auto* virtualNote=new QLabel(tr("利用 M2 每条 Profile 的真实曝光位姿扫描当前 STEP 工件和支承面。"
      "每帧 1280 点；测量场 Z=70~120 mm，宽度 58/50/42 mm（报告 Z 的 FlipZ 约定）。"
      "注意：相机/激光器的光学中心是待标定近似值，不能用于未经验证的实机补扫。"),virtualGroup);
    virtualNote->setWordWrap(true);virtualLayout->addWidget(virtualNote);
    auto* cameraRow=new QHBoxLayout;
    cameraRow->addWidget(new QLabel(tr("模拟相机 Y 偏置(mm, 未标定):"),virtualGroup));
    m_cameraBaselineY=new QDoubleSpinBox(virtualGroup);
    m_cameraBaselineY->setDecimals(2);m_cameraBaselineY->setRange(-100.,100.);
    // Share the default with the portable M3 optics model (single source of truth).
    m_cameraBaselineY->setValue(mls::twin::Sensor2950Geometry{}.cameraOriginSensor.y);
    m_cameraBaselineY->setSingleStep(5.);
    cameraRow->addWidget(m_cameraBaselineY);virtualLayout->addLayout(cameraRow);
    auto* runRow=new QHBoxLayout;
    m_virtualRunButton=new QPushButton(tr("运行 M3 虚拟扫描"),virtualGroup);
    m_virtualCancelButton=new QPushButton(tr("取消仿真"),virtualGroup);
    m_virtualCancelButton->setEnabled(false);
    runRow->addWidget(m_virtualRunButton);runRow->addWidget(m_virtualCancelButton);
    virtualLayout->addLayout(runRow);
    m_virtualExportButton=new QPushButton(tr("导出 M3 PLY + 完整 Profile…"),virtualGroup);
    m_virtualExportButton->setEnabled(false);
    virtualLayout->addWidget(m_virtualExportButton);
    m_virtualInfo=new QLabel(tr("未仿真。先导入 STEP/测试零件 + M2 .scan。橙红色=M3 虚拟点；绿色=M2 实测点。"),virtualGroup);
    m_virtualInfo->setWordWrap(true);m_virtualInfo->setTextInteractionFlags(Qt::TextSelectableByMouse);
    virtualLayout->addWidget(m_virtualInfo);
    vbox->addWidget(virtualGroup);
    connect(m_virtualRunButton,&QPushButton::clicked,this,&DigitalTwinScanWidget::startVirtualSimulation);
    connect(m_virtualCancelButton,&QPushButton::clicked,this,&DigitalTwinScanWidget::cancelVirtualSimulation);
    connect(m_virtualExportButton,&QPushButton::clicked,this,&DigitalTwinScanWidget::exportVirtualResults);
    m_virtualWatcher=new QFutureWatcher<std::shared_ptr<mls::twin::TwinSimulationData>>(this);
    connect(m_virtualWatcher,&QFutureWatcher<std::shared_ptr<mls::twin::TwinSimulationData>>::finished,
            this,[this]{
                if(m_virtualCancelButton)m_virtualCancelButton->setEnabled(false);
                onVirtualFinished();
            });
    m_virtualExportWatcher=new QFutureWatcher<QString>(this);
    connect(m_virtualExportWatcher,&QFutureWatcher<QString>::finished,this,[this]{
        const QString err=m_virtualExportWatcher->result();
        m_virtualExportButton->setEnabled(bool(m_virtual));
        if(err.isEmpty())QMessageBox::information(this,tr("M3 导出完成"),
            tr("已导出 PLY、每帧完整采样状态与 X/Z 的 .m3prof，以及光学近似模型参数 .m3.json。"));
        else QMessageBox::warning(this,tr("M3 导出失败"),err);
    });
    m_virtualProgressTimer=new QTimer(this);m_virtualProgressTimer->setInterval(250);
    connect(m_virtualProgressTimer,&QTimer::timeout,this,&DigitalTwinScanWidget::updateVirtualProgress);
    // M4 is diagnostic, not a physical optical calibration or a command path.
    auto* m4Group=new QGroupBox(tr("8. M4：实测/虚拟 Profile 对比 + 等效相机偏置拟合（离线）"),panel);
    auto* m4Layout=new QVBoxLayout(m4Group);
    auto* m4Note=new QLabel(tr("按相同曝光位姿、帧序号和原始采样索引比较 M2/M3。真实缺失点原因未知，"
        "不能直接认定为相机遮挡。四孔 ROI/台阶分区仅适用于随包 H1 参考 STEP。"
        "拟合仅搜索等效相机 Sensor-Y，保留训练/验证子集，不代表标定真实光心。"),m4Group);
    m4Note->setWordWrap(true);m4Layout->addWidget(m4Note);
    auto* m4Buttons=new QHBoxLayout;
    m_m4RunButton=new QPushButton(tr("运行 M4 全量对比"),m4Group);
    m_m4FitButton=new QPushButton(tr("拟合等效相机偏置"),m4Group);
    m_m4CancelButton=new QPushButton(tr("取消 M4"),m4Group);
    m_m4RunButton->setEnabled(false);
    m_m4FitButton->setEnabled(false);
    m_m4CancelButton->setEnabled(false);
    m4Buttons->addWidget(m_m4RunButton);m4Buttons->addWidget(m_m4FitButton);m4Buttons->addWidget(m_m4CancelButton);
    m4Layout->addLayout(m4Buttons);
    auto* rangeGrid=new QGridLayout;
    auto m4Spin=[&](double v){auto* sp=new QDoubleSpinBox(m4Group);
        sp->setRange(-100,100);sp->setDecimals(1);sp->setSingleStep(5);sp->setValue(v);return sp;};
    m_m4MinY=m4Spin(35);m_m4MaxY=m4Spin(95);m_m4StepY=m4Spin(10);
    rangeGrid->addWidget(new QLabel(tr("偏置起点 Y(mm)"),m4Group),0,0);
    rangeGrid->addWidget(m_m4MinY,0,1);
    rangeGrid->addWidget(new QLabel(tr("终点 Y(mm)"),m4Group),0,2);
    rangeGrid->addWidget(m_m4MaxY,0,3);
    rangeGrid->addWidget(new QLabel(tr("步长 (mm)"),m4Group),1,0);
    rangeGrid->addWidget(m_m4StepY,1,1);
    m_m4StepY->setRange(1,100);
    m_m4FrameCap=new QSpinBox(m4Group);
    m_m4FrameCap->setRange(20,1200);m_m4FrameCap->setValue(180);
    rangeGrid->addWidget(new QLabel(tr("最多训练/验证帧"),m4Group),1,2);
    rangeGrid->addWidget(m_m4FrameCap,1,3);
    m4Layout->addLayout(rangeGrid);
    auto* m4ExportRow=new QHBoxLayout;
    m_m4ExportButton=new QPushButton(tr("导出 M4 对比 JSON/CSV"),m4Group);
    m_m4FitExportButton=new QPushButton(tr("导出拟合曲线 JSON/CSV"),m4Group);
    m_m4ApplyButton=new QPushButton(tr("应用拟合偏置（需重跑 M3）"),m4Group);
    m_m4ExportButton->setEnabled(false);
    m_m4FitExportButton->setEnabled(false);
    m_m4ApplyButton->setEnabled(false);
    m4ExportRow->addWidget(m_m4ExportButton);m4ExportRow->addWidget(m_m4FitExportButton);
    m4Layout->addLayout(m4ExportRow);
    m4Layout->addWidget(m_m4ApplyButton);
    m_m4Info=new QPlainTextEdit(m4Group);
    m_m4Info->setReadOnly(true);m_m4Info->setMinimumHeight(125);
    m_m4Info->setPlainText(tr("M4 尚未运行。先导入 STEP 和真实 .scan，再运行 M3。"));
    m4Layout->addWidget(m_m4Info);
    vbox->addWidget(m4Group);
    // M5: independently associate observed / simulated returns with the
    // tessellated CAD surface. Missing is UNKNOWN, not a proven shadow.
    auto* m5Group=new QGroupBox(tr("9. M5：CAD 表面覆盖与缺失区域（离线诊断）"),panel);
    auto* m5Layout=new QVBoxLayout(m5Group);
    auto* m5Note=new QLabel(tr("基于 STEP 三角网格等面积单元统计；将 M2 实测和 M3 虚拟点分别关联 CAD。"
        "真实缺失原因未知；覆盖率不是孔径或 Z 测量精度。红色=实测未覆盖，青色=实测覆盖，金色=仅虚拟覆盖。"),m5Group);
    m5Note->setWordWrap(true);m5Layout->addWidget(m5Note);
    auto* m5Grid=new QGridLayout;
    m_m5Pitch=new QDoubleSpinBox(m5Group);
    m_m5Pitch->setRange(.15,3.);m_m5Pitch->setDecimals(2);
    m_m5Pitch->setSingleStep(.1);m_m5Pitch->setValue(.6);
    m5Grid->addWidget(new QLabel(tr("CAD 单元尺寸 mm"),m5Group),0,0);
    m5Grid->addWidget(m_m5Pitch,0,1);
    m_m5MatchDistance=new QDoubleSpinBox(m5Group);
    m_m5MatchDistance->setRange(.05,2.);m_m5MatchDistance->setDecimals(2);
    m_m5MatchDistance->setSingleStep(.05);m_m5MatchDistance->setValue(.65);
    m5Grid->addWidget(new QLabel(tr("最近表面距离上限 mm"),m5Group),1,0);
    m5Grid->addWidget(m_m5MatchDistance,1,1);
    m_m5MinHits=new QSpinBox(m5Group);
    m_m5MinHits->setRange(1,100);m_m5MinHits->setValue(2);
    m5Grid->addWidget(new QLabel(tr("每单元至少有效点数"),m5Group),2,0);
    m5Grid->addWidget(m_m5MinHits,2,1);
    m_m5ExcludeBottom=new QCheckBox(tr("默认排除 CAD 底面（避免支承面回波）"),m5Group);
    m_m5ExcludeBottom->setChecked(true);
    m5Grid->addWidget(m_m5ExcludeBottom,3,0,1,2);
    m5Layout->addLayout(m5Grid);
    auto* m5Buttons=new QGridLayout;
    m_m5RunButton=new QPushButton(tr("运行 M5 覆盖分析"),m5Group);
    m_m5CancelButton=new QPushButton(tr("取消 M5"),m5Group);
    m_m5ClearButton=new QPushButton(tr("清除 M5 热图/结果"),m5Group);
    m_m5ExportButton=new QPushButton(tr("导出 M5 JSON + 单元 CSV"),m5Group);
    m_m5CancelButton->setEnabled(false);m_m5ExportButton->setEnabled(false);
    m5Buttons->addWidget(m_m5RunButton,0,0);m5Buttons->addWidget(m_m5CancelButton,0,1);
    m5Buttons->addWidget(m_m5ClearButton,1,0);m5Buttons->addWidget(m_m5ExportButton,1,1);
    m5Layout->addLayout(m5Buttons);
    m_m5Info=new QPlainTextEdit(m5Group);
    m_m5Info->setReadOnly(true);m_m5Info->setMinimumHeight(155);
    m_m5Info->setPlainText(tr("M5 尚未运行。先导入 STEP 和 M2 SCAN，可选先运行 M3。"));
    m5Layout->addWidget(m_m5Info);vbox->addWidget(m5Group);
    connect(m_m5RunButton,&QPushButton::clicked,this,&DigitalTwinScanWidget::startM5);
    connect(m_m5CancelButton,&QPushButton::clicked,this,&DigitalTwinScanWidget::cancelM5);
    connect(m_m5ClearButton,&QPushButton::clicked,this,&DigitalTwinScanWidget::clearM5);
    connect(m_m5ExportButton,&QPushButton::clicked,this,&DigitalTwinScanWidget::exportM5);
    m_m5Watcher=new QFutureWatcher<std::shared_ptr<mls::twin::TwinM5Data>>(this);
    connect(m_m5Watcher,&QFutureWatcher<std::shared_ptr<mls::twin::TwinM5Data>>::finished,
            this,&DigitalTwinScanWidget::onM5Finished);
    m_m5ExportWatcher=new QFutureWatcher<QString>(this);
    connect(m_m5ExportWatcher,&QFutureWatcher<QString>::finished,this,[this]{
        const auto error=m_m5ExportWatcher->result();
        if(m_m5ExportButton)m_m5ExportButton->setEnabled(bool(m_m5Data));
        if(error.isEmpty())QMessageBox::information(this,tr("M5 导出完成"),
            tr("已导出 JSON 汇总、完整表面单元 CSV 和特征分组 CSV。"));
        else QMessageBox::warning(this,tr("M5 导出失败"),error);
    });
    m_m5ProgressTimer=new QTimer(this);m_m5ProgressTimer->setInterval(350);
    connect(m_m5ProgressTimer,&QTimer::timeout,this,&DigitalTwinScanWidget::updateM5Progress);

    // V28.5 M5.1: H1 center chamfer, cylinder and lower chamfer.
    // Display raw density, not the M5 coarse binary occupancy indicator.
    auto* m51Group=new QGroupBox(tr("10. M5.1：中心孔倒角精细诊断（H1 离线）"),panel);
    auto* m51Layout=new QVBoxLayout(m51Group);
    auto* m51Note=new QLabel(tr("仅针对当前 30×95×10 H1 STEP：上倒角72角度×4径向层；另检查直孔壁/下倒角。"
        "M2 未返回采样原因未知；关联距离为几何筛选，不是精度公差。"),m51Group);
    m51Note->setWordWrap(true);m51Layout->addWidget(m51Note);
    auto* m51Form=new QGridLayout;
    m_m51Threshold=new QComboBox(m51Group);
    m_m51Threshold->addItems(QStringList{tr("严格 0.15 mm"),tr("中等 0.30 mm"),tr("宽松 0.65 mm")});
    m_m51Threshold->setCurrentIndex(1);
    m51Form->addWidget(new QLabel(tr("曲面关联阈值"),m51Group),0,0);
    m51Form->addWidget(m_m51Threshold,0,1);
    m_m51SparseRatio=new QDoubleSpinBox(m51Group);
    m_m51SparseRatio->setRange(.05,.95);m_m51SparseRatio->setDecimals(2);
    m_m51SparseRatio->setSingleStep(.05);m_m51SparseRatio->setValue(.30);
    m51Form->addWidget(new QLabel(tr("稀疏阈值 / 参考密度"),m51Group),1,0);
    m51Form->addWidget(m_m51SparseRatio,1,1);
    m_m51ViewMode=new QComboBox(m51Group);
    m_m51ViewMode->addItems(QStringList{tr("M2 实测密度"),tr("M3 虚拟密度"),tr("虚拟−实测（归一化）")});
    m51Form->addWidget(new QLabel(tr("热图内容"),m51Group),2,0);
    m51Form->addWidget(m_m51ViewMode,2,1);
    m51Layout->addLayout(m51Form);
    m_m51Heatmap=new QTableWidget(4,72,m51Group);
    m_m51Heatmap->setRowCount(4);m_m51Heatmap->setColumnCount(72);
    m_m51Heatmap->setMinimumHeight(180);m_m51Heatmap->setMaximumHeight(210);
    m_m51Heatmap->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_m51Heatmap->setSelectionMode(QAbstractItemView::NoSelection);
    m_m51Heatmap->verticalHeader()->setDefaultSectionSize(28);
    m_m51Heatmap->horizontalHeader()->setDefaultSectionSize(27);
    for(int a=0;a<72;++a)m_m51Heatmap->setHorizontalHeaderItem(a,new QTableWidgetItem(QString::number(a*5)));
    for(int j=0;j<4;++j)m_m51Heatmap->setVerticalHeaderItem(j,new QTableWidgetItem(tr("层%1").arg(j+1)));
    m51Layout->addWidget(m_m51Heatmap);
    auto* m51Buttons=new QGridLayout;
    m_m51RunButton=new QPushButton(tr("运行 M5.1 中心孔分析"),m51Group);
    m_m51CancelButton=new QPushButton(tr("取消 M5.1"),m51Group);
    m_m51ClearButton=new QPushButton(tr("清除 M5.1 数据"),m51Group);
    m_m51ExportButton=new QPushButton(tr("导出 JSON / 分箱 CSV"),m51Group);
    m_m51CancelButton->setEnabled(false);m_m51ExportButton->setEnabled(false);
    m51Buttons->addWidget(m_m51RunButton,0,0);m51Buttons->addWidget(m_m51CancelButton,0,1);
    m51Buttons->addWidget(m_m51ClearButton,1,0);m51Buttons->addWidget(m_m51ExportButton,1,1);
    m51Layout->addLayout(m51Buttons);
    m_m51Info=new QPlainTextEdit(m51Group);
    m_m51Info->setReadOnly(true);m_m51Info->setMinimumHeight(165);
    m_m51Info->setPlainText(tr("先导入 H1 STEP 与 M2 SCAN；M3 虚拟数据可选。运行后显示逐角度点密度与关联阈值诊断。"));
    m51Layout->addWidget(m_m51Info);vbox->addWidget(m51Group);
    connect(m_m51RunButton,&QPushButton::clicked,this,&DigitalTwinScanWidget::startM51);
    connect(m_m51CancelButton,&QPushButton::clicked,this,&DigitalTwinScanWidget::cancelM51);
    connect(m_m51ClearButton,&QPushButton::clicked,this,&DigitalTwinScanWidget::clearM51);
    connect(m_m51ExportButton,&QPushButton::clicked,this,&DigitalTwinScanWidget::exportM51);
    connect(m_m51ViewMode,QOverload<int>::of(&QComboBox::currentIndexChanged),
            this,[this](int){refreshM51Heatmap();});
    m_m51Watcher=new QFutureWatcher<std::shared_ptr<mls::twin::TwinM51Data>>(this);
    connect(m_m51Watcher,&QFutureWatcher<std::shared_ptr<mls::twin::TwinM51Data>>::finished,
            this,&DigitalTwinScanWidget::onM51Finished);
    m_m51ExportWatcher=new QFutureWatcher<QString>(this);
    connect(m_m51ExportWatcher,&QFutureWatcher<QString>::finished,this,[this]{
        const auto error=m_m51ExportWatcher->result();
        if(m_m51ExportButton)m_m51ExportButton->setEnabled(bool(m_m51Data));
        if(error.isEmpty())QMessageBox::information(this,tr("M5.1 导出"),tr("已导出 JSON 和全部表面分箱 CSV。"));
        else QMessageBox::warning(this,tr("M5.1 导出失败"),error);
    });
    m_m51ProgressTimer=new QTimer(this);m_m51ProgressTimer->setInterval(350);
    connect(m_m51ProgressTimer,&QTimer::timeout,this,&DigitalTwinScanWidget::updateM51Progress);

    // V28.6 M5.2: top-plane anchoring + center hole exploratory geometric fit.
    // Never applies or exports executable machine motion.
    auto* m52Group=new QGroupBox(tr("11. M5.2：中心孔几何一致性与配准残差（只读诊断）"),panel);
    auto* m52Layout=new QVBoxLayout(m52Group);
    auto* m52Note=new QLabel(tr("先用远离孔口的 STEP 顶面区域估计高度/微倾角，再用中心孔倒角估计 XY 候选偏移和剩余截面残差。"
       "所有偏移均为诊断量，绝不自动更改 STEP、工件放置或标定。"),m52Group);
    m52Note->setWordWrap(true);m52Layout->addWidget(m52Note);
    auto* m52Controls=new QHBoxLayout;
    m_m52ViewMode=new QComboBox(m52Group);
    m_m52ViewMode->addItems(QStringList{tr("实测：原始"),tr("实测：顶面校正后"),
        tr("实测：再去 XY 候选偏移"),tr("实测：拟合剩余偏差"),tr("虚拟：原始")});
    m52Controls->addWidget(new QLabel(tr("5°残差曲线"),m52Group));
    m52Controls->addWidget(m_m52ViewMode,1);
    m52Layout->addLayout(m52Controls);
    m_m52Heatmap=new QTableWidget(1,72,m52Group);
    m_m52Heatmap->setMinimumHeight(82);m_m52Heatmap->setMaximumHeight(114);
    m_m52Heatmap->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_m52Heatmap->setSelectionMode(QAbstractItemView::NoSelection);
    m_m52Heatmap->verticalHeader()->setDefaultSectionSize(38);
    m_m52Heatmap->horizontalHeader()->setDefaultSectionSize(33);
    for(int a=0;a<72;++a)m_m52Heatmap->setHorizontalHeaderItem(a,new QTableWidgetItem(QString::number(a*5)));
    m_m52Heatmap->setVerticalHeaderItem(0,new QTableWidgetItem(tr("mm")));
    m52Layout->addWidget(m_m52Heatmap);
    auto* m52Buttons=new QGridLayout;
    m_m52RunButton=new QPushButton(tr("运行 M5.2 几何诊断"),m52Group);
    m_m52CancelButton=new QPushButton(tr("取消 M5.2"),m52Group);
    m_m52ClearButton=new QPushButton(tr("清除 M5.2"),m52Group);
    m_m52ExportButton=new QPushButton(tr("导出 JSON + 角度 CSV"),m52Group);
    m_m52CancelButton->setEnabled(false);m_m52ExportButton->setEnabled(false);
    m52Buttons->addWidget(m_m52RunButton,0,0);m52Buttons->addWidget(m_m52CancelButton,0,1);
    m52Buttons->addWidget(m_m52ClearButton,1,0);m52Buttons->addWidget(m_m52ExportButton,1,1);
    m52Layout->addLayout(m52Buttons);
    m_m52Info=new QPlainTextEdit(m52Group);
    m_m52Info->setReadOnly(true);m_m52Info->setMinimumHeight(160);
    m_m52Info->setPlainText(tr("M5.2 未运行。请先导入 H1 STEP 和 M2 .scan；M3 可选。不会对 CAD 或轴坐标施加修正。"));
    m52Layout->addWidget(m_m52Info);vbox->addWidget(m52Group);
    connect(m_m52RunButton,&QPushButton::clicked,this,&DigitalTwinScanWidget::startM52);
    connect(m_m52CancelButton,&QPushButton::clicked,this,&DigitalTwinScanWidget::cancelM52);
    connect(m_m52ClearButton,&QPushButton::clicked,this,&DigitalTwinScanWidget::clearM52);
    connect(m_m52ExportButton,&QPushButton::clicked,this,&DigitalTwinScanWidget::exportM52);
    connect(m_m52ViewMode,QOverload<int>::of(&QComboBox::currentIndexChanged),
            this,[this](int){refreshM52Heatmap();});
    m_m52Watcher=new QFutureWatcher<std::shared_ptr<mls::twin::TwinM52Data>>(this);
    connect(m_m52Watcher,&QFutureWatcher<std::shared_ptr<mls::twin::TwinM52Data>>::finished,
            this,&DigitalTwinScanWidget::onM52Finished);
    m_m52ExportWatcher=new QFutureWatcher<QString>(this);
    connect(m_m52ExportWatcher,&QFutureWatcher<QString>::finished,this,[this]{
        const auto err=m_m52ExportWatcher->result();
        if(m_m52ExportButton)m_m52ExportButton->setEnabled(static_cast<bool>(m_m52Data));
        if(err.isEmpty())QMessageBox::information(this,tr("M5.2 导出"),tr("JSON 和逐 5° 残差 CSV 已导出。"));
        else QMessageBox::warning(this,tr("M5.2 导出失败"),err);
    });
    m_m52ProgressTimer=new QTimer(this);m_m52ProgressTimer->setInterval(350);
    connect(m_m52ProgressTimer,&QTimer::timeout,this,&DigitalTwinScanWidget::updateM52Progress);

    // V28.7 M6: truly continuous XYZCB scan. Offline M3 ray casting only.
    auto* m6Group=new QGroupBox(tr("12. M6：连续 XYZ/C/B 五轴自适应补扫（离线）"),panel);
    auto* m6Layout=new QVBoxLayout(m6Group);
    auto* m6Help=new QLabel(tr("从 M5 未覆盖 CAD 面选择扫描目标。所有采集段均连续改变 XYZ/C/B，并通过完整五轴位姿模拟 M3 激光 Profile。"
        "不向真实控制器发送运动；即使机床 FCL 检查通过，夹具和传感器外壳仍需独立认证。"
        "200Hz 完整零件多段仿真计算量和点云磁盘空间可能很大；随时可取消。"),m6Group);
    m6Help->setWordWrap(true);m6Layout->addWidget(m6Help);
    auto* m6Grid=new QGridLayout;
    m_m6Hz=new QDoubleSpinBox(m6Group);m_m6Hz->setRange(20.,2000.);
    m_m6Hz->setDecimals(0);m_m6Hz->setValue(200.);
    m_m6Tilt=new QDoubleSpinBox(m6Group);m_m6Tilt->setRange(0.,90.);
    m_m6Tilt->setDecimals(1);m_m6Tilt->setValue(22.);
    m_m6Yaw=new QDoubleSpinBox(m6Group);m_m6Yaw->setRange(0.,90.);
    m_m6Yaw->setDecimals(1);m_m6Yaw->setValue(65.);
    m_m6Segments=new QSpinBox(m6Group);m_m6Segments->setRange(1,24);m_m6Segments->setValue(10);
    m_m6ExportStride=new QSpinBox(m6Group);
    m_m6ExportStride->setRange(1,32);m_m6ExportStride->setValue(8);
    m_m6ExportStride->setToolTip(tr("可选的额外 PLY 预览抽样；不影响无损分卷 SCAN、覆盖率或轨迹 CSV。"));
    m_m6AlsoPly=new QCheckBox(tr("另存抽样 PLY（可选，默认不生成）"),m6Group);
    m_m6AlsoPly->setChecked(false);
    m_m6ExportStride->setEnabled(false);
    connect(m_m6AlsoPly,&QCheckBox::toggled,m_m6ExportStride,&QSpinBox::setEnabled);
    m6Grid->addWidget(new QLabel(tr("虚拟采样频率 Hz"),m6Group),0,0);
    m6Grid->addWidget(m_m6Hz,0,1);
    m6Grid->addWidget(new QLabel(tr("B 倾角幅值 °"),m6Group),1,0);
    m6Grid->addWidget(m_m6Tilt,1,1);
    m6Grid->addWidget(new QLabel(tr("C 方位幅值 °"),m6Group),2,0);
    m6Grid->addWidget(m_m6Yaw,2,1);
    m6Grid->addWidget(new QLabel(tr("最大扫描段数量"),m6Group),3,0);
    m6Grid->addWidget(m_m6Segments,3,1);
    m6Grid->addWidget(m_m6AlsoPly,4,0,1,2);
    m6Grid->addWidget(new QLabel(tr("附加 PLY 每 N 点保留 1 点"),m6Group),5,0);
    m6Grid->addWidget(m_m6ExportStride,5,1);
    m6Layout->addLayout(m6Grid);
    m_m6SkipFcl=new QCheckBox(tr("仅离线调试：跳过机床 FCL 检查（未验证碰撞，不可执行）"),m6Group);
    m_m6SkipFcl->setChecked(false);
    m6Layout->addWidget(m_m6SkipFcl);
    auto* m6Buttons=new QGridLayout;
    m_m6RunButton=new QPushButton(tr("运行 M6 连续五轴规划+模拟"),m6Group);
    m_m6CancelButton=new QPushButton(tr("取消 M6"),m6Group);
    m_m6ClearButton=new QPushButton(tr("清除 M6 图层/轨迹"),m6Group);
    m_m6ExportButton=new QPushButton(tr("保存报告 + 轨迹 CSV + 无损压缩 SCAN 分卷"),m6Group);
    m_m6CancelButton->setEnabled(false);m_m6ExportButton->setEnabled(false);
    m6Buttons->addWidget(m_m6RunButton,0,0);m6Buttons->addWidget(m_m6CancelButton,0,1);
    m6Buttons->addWidget(m_m6ClearButton,1,0);m6Buttons->addWidget(m_m6ExportButton,1,1);
    m6Layout->addLayout(m6Buttons);
    m_m6ProgressBar=new QProgressBar(m6Group);
    m_m6ProgressBar->setRange(0,100);m_m6ProgressBar->setValue(0);
    m_m6ProgressBar->setFormat(tr("等待 M6 任务"));
    m6Layout->addWidget(m_m6ProgressBar);
    m_m6SegmentsProgressBar=new QProgressBar(m6Group);
    m_m6SegmentsProgressBar->setRange(0,100);
    m_m6SegmentsProgressBar->setValue(0);
    m_m6SegmentsProgressBar->setFormat(tr("扫描段：0 / 上限"));
    m6Layout->addWidget(m_m6SegmentsProgressBar);
    m_m6CurrentProgressBar=new QProgressBar(m6Group);
    m_m6CurrentProgressBar->setRange(0,100);
    m_m6CurrentProgressBar->setValue(0);
    m_m6CurrentProgressBar->setFormat(tr("当前扫描段：等待中"));
    m6Layout->addWidget(m_m6CurrentProgressBar);
    m_m6ArchiveProgressBar=new QProgressBar(m6Group);
    m_m6ArchiveProgressBar->setRange(0,100);
    m_m6ArchiveProgressBar->setValue(0);
    m_m6ArchiveProgressBar->setFormat(tr("模拟 SCAN 分卷：等待中"));
    m6Layout->addWidget(m_m6ArchiveProgressBar);
    auto* m6PathNotice=new QLabel(tr("黄色线＝激光名义测量参考点（贴近工件）的分段轨迹。传感器原点与其相隔约95mm；段间过渡不作为激光扫描线显示。"),m6Group);
    m6PathNotice->setWordWrap(true);m6Layout->addWidget(m6PathNotice);
    m_m6Info=new QPlainTextEdit(m6Group);m_m6Info->setReadOnly(true);m_m6Info->setMinimumHeight(150);
    m_m6Info->setPlainText(tr("先导入 STEP、H1 SCAN，并运行 M5 覆盖分析。M6 只读现有数据，只能进行离线仿真。"));
    m6Layout->addWidget(m_m6Info);vbox->addWidget(m6Group);
    connect(m_m6RunButton,&QPushButton::clicked,this,&DigitalTwinScanWidget::startM6);
    connect(m_m6CancelButton,&QPushButton::clicked,this,&DigitalTwinScanWidget::cancelM6);
    connect(m_m6ClearButton,&QPushButton::clicked,this,&DigitalTwinScanWidget::clearM6);
    connect(m_m6ExportButton,&QPushButton::clicked,this,&DigitalTwinScanWidget::exportM6);
    m_m6Watcher=new QFutureWatcher<std::shared_ptr<mls::twin::TwinM6Data>>(this);
    connect(m_m6Watcher,&QFutureWatcher<std::shared_ptr<mls::twin::TwinM6Data>>::finished,
            this,&DigitalTwinScanWidget::onM6Finished);
    m_m6ExportWatcher=new QFutureWatcher<QString>(this);
    connect(m_m6ExportWatcher,&QFutureWatcher<QString>::finished,this,[this]{
        if(m_m6ProgressTimer)m_m6ProgressTimer->stop();
        const QString err=m_m6ExportWatcher->result();
        if(m_m6ExportButton)m_m6ExportButton->setEnabled(static_cast<bool>(m_m6Data));
        if(m_m6CancelButton)m_m6CancelButton->setEnabled(false);
        if(m_m6ArchiveProgressBar){
            m_m6ArchiveProgressBar->setRange(0,100);
            m_m6ArchiveProgressBar->setValue(err.isEmpty()?100:0);
            m_m6ArchiveProgressBar->setFormat(err.isEmpty()?tr("无损 SCAN 分卷导出完成"):
                                                      tr("SCAN 导出未完成"));
        }
        if(err.isEmpty())QMessageBox::information(this,tr("M6 导出完成"),
            tr("已导出无损压缩 SCAN 分卷、离线轨迹 CSV、JSON 清单及报告。所有轨迹均不可直接执行。"));
        else QMessageBox::warning(this,tr("M6 导出失败"),err);
    });
    m_m6ProgressTimer=new QTimer(this);m_m6ProgressTimer->setInterval(500);
    connect(m_m6ProgressTimer,&QTimer::timeout,this,&DigitalTwinScanWidget::updateM6Progress);

    connect(m_m4RunButton,&QPushButton::clicked,this,&DigitalTwinScanWidget::startM4Comparison);
    connect(m_m4FitButton,&QPushButton::clicked,this,&DigitalTwinScanWidget::startM4Fit);
    connect(m_m4CancelButton,&QPushButton::clicked,this,&DigitalTwinScanWidget::cancelM4);
    connect(m_m4ExportButton,&QPushButton::clicked,this,&DigitalTwinScanWidget::exportM4Report);
    connect(m_m4FitExportButton,&QPushButton::clicked,this,&DigitalTwinScanWidget::exportM4Fit);
    connect(m_m4ApplyButton,&QPushButton::clicked,this,&DigitalTwinScanWidget::applyM4Fit);
    m_m4CompareWatcher=new QFutureWatcher<std::shared_ptr<mls::twin::TwinM4ComparisonData>>(this);
    connect(m_m4CompareWatcher,&QFutureWatcher<std::shared_ptr<mls::twin::TwinM4ComparisonData>>::finished,
            this,&DigitalTwinScanWidget::onM4ComparisonFinished);
    m_m4FitWatcher=new QFutureWatcher<std::shared_ptr<mls::twin::TwinM4FitData>>(this);
    connect(m_m4FitWatcher,&QFutureWatcher<std::shared_ptr<mls::twin::TwinM4FitData>>::finished,
            this,&DigitalTwinScanWidget::onM4FitFinished);
    m_m4ProgressTimer=new QTimer(this);m_m4ProgressTimer->setInterval(300);
    connect(m_m4ProgressTimer,&QTimer::timeout,this,&DigitalTwinScanWidget::updateM4Progress);
    // V28.8 M7: point-cloud provenance-aware fusion & iterative coverage acceptance.
    auto* m7Group=new QGroupBox(tr("14. M7：M2 实测 + M6 连续多视角虚拟补扫 / 融合验收（离线）"),panel);
    auto* m7Layout=new QVBoxLayout(m7Group);
    auto* m7Help=new QLabel(tr("完整覆盖使用 M5+M6 的真实/虚拟 CAD 命中单元；"
        "融合点云为独立的空间抽样预览，真实 M2 数据优先，绝不使用 CAD 虚构测量点。"
        "所有 M6 多角度结果仅为未标定光学模拟，不能直接控制机床。"),m7Group);
    m7Help->setWordWrap(true);m7Layout->addWidget(m7Help);
    auto* m7Form=new QFormLayout;
    m_m7VoxelPitch=new QDoubleSpinBox(m7Group);
    m_m7VoxelPitch->setRange(.05,2.0);m_m7VoxelPitch->setDecimals(2);
    m_m7VoxelPitch->setSingleStep(.05);m_m7VoxelPitch->setValue(.25);
    m7Form->addRow(tr("融合预览体素尺寸(mm)"),m_m7VoxelPitch);
    m_m7VoxelCap=new QSpinBox(m7Group);m_m7VoxelCap->setRange(2000,250000);
    m_m7VoxelCap->setSingleStep(10000);m_m7VoxelCap->setValue(65000);
    m7Form->addRow(tr("每一来源最多显示体素"),m_m7VoxelCap);
    m_m7Goal=new QDoubleSpinBox(m7Group);m_m7Goal->setRange(30.,100.);
    m_m7Goal->setDecimals(1);m_m7Goal->setValue(95.);
    m7Form->addRow(tr("目标 CAD 面积覆盖率(%)"),m_m7Goal);
    m7Layout->addLayout(m7Form);
    auto* m7Buttons=new QGridLayout;
    m_m7RunButton=new QPushButton(tr("运行 M7 多视角融合与覆盖验收"),m7Group);
    m_m7CancelButton=new QPushButton(tr("取消 M7"),m7Group);
    m_m7ClearButton=new QPushButton(tr("清除 M7 独立图层"),m7Group);
    m_m7ExportButton=new QPushButton(tr("导出 M7 JSON/CSV + 融合预览 PLY"),m7Group);
    m_m7CancelButton->setEnabled(false);m_m7ExportButton->setEnabled(false);
    m7Buttons->addWidget(m_m7RunButton,0,0);m7Buttons->addWidget(m_m7CancelButton,0,1);
    m7Buttons->addWidget(m_m7ClearButton,1,0);m7Buttons->addWidget(m_m7ExportButton,1,1);
    m7Layout->addLayout(m7Buttons);
    m_m7ProgressBar=new QProgressBar(m7Group);m_m7ProgressBar->setRange(0,100);
    m_m7ProgressBar->setValue(0);m_m7ProgressBar->setFormat(tr("等待 M7"));
    m7Layout->addWidget(m_m7ProgressBar);
    m_m7FeatureTable=new QTableWidget(0,4,m7Group);
    m_m7FeatureTable->setHorizontalHeaderLabels({tr("CAD 特征"),tr("M2 实测 mm²"),
         tr("M6 虚拟新增 mm²"),tr("仍未覆盖 mm²")});
    m_m7FeatureTable->setMinimumHeight(180);
    m_m7FeatureTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_m7FeatureTable->horizontalHeader()->setStretchLastSection(true);
    m7Layout->addWidget(m_m7FeatureTable);
    m_m7SegmentTable=new QTableWidget(0,4,m7Group);
    m_m7SegmentTable->setHorizontalHeaderLabels({tr("扫描段"),tr("目标特征"),
        tr("预计新增 mm²"),tr("实际模拟新增 mm²")});
    m_m7SegmentTable->setMinimumHeight(160);
    m_m7SegmentTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_m7SegmentTable->horizontalHeader()->setStretchLastSection(true);
    m7Layout->addWidget(m_m7SegmentTable);
    m_m7Info=new QPlainTextEdit(m7Group);m_m7Info->setReadOnly(true);
    m_m7Info->setMinimumHeight(140);
    m_m7Info->setPlainText(tr("请先完成 M2 实测 SCAN、M5 CAD 覆盖及 M6 连续五轴虚拟补扫。"));
    m7Layout->addWidget(m_m7Info);
    vbox->addWidget(m7Group);
    connect(m_m7RunButton,&QPushButton::clicked,this,&DigitalTwinScanWidget::startM7);
    connect(m_m7CancelButton,&QPushButton::clicked,this,&DigitalTwinScanWidget::cancelM7);
    connect(m_m7ClearButton,&QPushButton::clicked,this,&DigitalTwinScanWidget::clearM7);
    connect(m_m7ExportButton,&QPushButton::clicked,this,&DigitalTwinScanWidget::exportM7);
    m_m7Watcher=new QFutureWatcher<std::shared_ptr<mls::twin::TwinM7Data>>(this);
    connect(m_m7Watcher,&QFutureWatcher<std::shared_ptr<mls::twin::TwinM7Data>>::finished,
            this,&DigitalTwinScanWidget::onM7Finished);
    m_m7ExportWatcher=new QFutureWatcher<QString>(this);
    connect(m_m7ExportWatcher,&QFutureWatcher<QString>::finished,this,[this]{
        const QString result=m_m7ExportWatcher->result();
        m_m7ExportButton->setEnabled(bool(m_m7Data));
        if(result.isEmpty())QMessageBox::information(this,tr("M7 导出"),
            tr("融合预览 PLY、逐特征/段 CSV 和 M7 验收 JSON 已保存。完整点保留在 M2 与 M6 SCAN 中。"));
        else QMessageBox::warning(this,tr("M7 导出失败"),result);
    });
    m_m7ProgressTimer=new QTimer(this);m_m7ProgressTimer->setInterval(350);
    connect(m_m7ProgressTimer,&QTimer::timeout,this,&DigitalTwinScanWidget::updateM7Progress);
    buildTwinInspector(vbox,panel);
    buildM8Workflow(vbox,panel);
    vbox->addStretch();
    m_replayTimer=new QTimer(this);
    m_replayTimer->setInterval(45); // UI refresh ~22 FPS, not one render per 5ms Profile
    connect(m_replayTimer,&QTimer::timeout,this,&DigitalTwinScanWidget::onReplayTick);
    connect(m_replayOpen,&QPushButton::clicked,this,&DigitalTwinScanWidget::browseReplayScan);
    connect(m_replayPlay,&QPushButton::clicked,this,&DigitalTwinScanWidget::startReplay);
    connect(pauseButton,&QPushButton::clicked,this,&DigitalTwinScanWidget::stopReplay);
    connect(resetButton,&QPushButton::clicked,this,&DigitalTwinScanWidget::resetReplay);
    connect(m_replaySlider,&QSlider::valueChanged,this,&DigitalTwinScanWidget::seekReplay);
    connect(m_replayExport,&QPushButton::clicked,this,&DigitalTwinScanWidget::exportReplayPly);
    m_replayWatcher=new QFutureWatcher<std::shared_ptr<mls::twin::ScanReplayData>>(this);
    connect(m_replayWatcher,&QFutureWatcher<std::shared_ptr<mls::twin::ScanReplayData>>::finished,
            this,&DigitalTwinScanWidget::onReplayLoaded);
    m_exportWatcher=new QFutureWatcher<QString>(this);
    connect(m_exportWatcher,&QFutureWatcher<QString>::finished,this,[this]{
        const QString message=m_exportWatcher->result();
        m_replayExport->setEnabled(static_cast<bool>(m_replay));
        if(message.isEmpty())QMessageBox::information(this,tr("导出完成"),tr("已完成不降采样的全分辨率 PLY 重建导出。"));
        else QMessageBox::warning(this,tr("导出失败"),message);
    });
    connect(importButton,&QPushButton::clicked,this,[this]{
        const QString path=QFileDialog::getOpenFileName(this,tr("导入 STEP CAD"),QString(),tr("STEP 模型 (*.step *.stp *.STEP *.STP)"));
        if(!path.isEmpty())importStepFile(path);
    });
    connect(exampleButton,&QPushButton::clicked,this,&DigitalTwinScanWidget::loadReference);
    connect(loadObjButton,&QPushButton::clicked,this,[this]{
        const QString path=QFileDialog::getOpenFileName(this,tr("导入 OBJ 调试网格"),QString(),tr("OBJ 网格 (*.obj)"));
        if(!path.isEmpty())importObjFile(path);
    });
    connect(saveButton,&QPushButton::clicked,this,&DigitalTwinScanWidget::saveSession);
    connect(openButton,&QPushButton::clicked,this,&DigitalTwinScanWidget::loadSession);
    connect(machineButton,&QPushButton::clicked,this,&DigitalTwinScanWidget::tryLoadMachine);
    connect(cameraButton,&QPushButton::clicked,this,[this]{if(m_view)m_view->focusTwinWorkpiece();});
    connect(mainFocusButton,&QPushButton::clicked,this,[this]{
        if(!m_mainView){QMessageBox::warning(this,tr("无主界面"),tr("主三维视图未连接。"));return;}
        syncMainScene(true);
        refreshInfo(tr("主界面三维点云视图已更新；关闭此窗口即可查看。"));
    });

    m_status=new QLabel(panel);m_status->setWordWrap(true);
    m_status->setTextInteractionFlags(Qt::TextSelectableByMouse);
    m_status->setStyleSheet("QLabel { padding:8px; background:#f2f7f9; border:1px solid #d5e3e9; }");
    vbox->addWidget(m_status);

    m_view=new PointCloudWidget(splitter);
    // Inspector controls are built before the VTK widget exists. Apply their
    // persisted initial values now, including intentionally hidden overlays.
    for(auto it=m_twinLayerChecks.constBegin();it!=m_twinLayerChecks.constEnd();++it)
        m_view->setTwinOverlayVisible(it.key(),it.value()->isChecked());
    splitter->addWidget(scroll);splitter->addWidget(m_view);
    splitter->setStretchFactor(0,0);splitter->setStretchFactor(1,1);
    outer->addWidget(splitter,1);
    auto* footer=new QDialogButtonBox(QDialogButtonBox::Close,this);
    connect(footer,&QDialogButtonBox::rejected,this,&QDialog::close);
    outer->addWidget(footer);
}

QString DigitalTwinScanWidget::findExample(const QString& basename) const {
    const QStringList dirs={
        QDir(QCoreApplication::applicationDirPath()).filePath("examples/digital_twin"),
        QDir::current().filePath("examples/digital_twin"),
        QDir(QCoreApplication::applicationDirPath()).filePath("../../examples/digital_twin")
    };
    for(const auto& dir:dirs){const auto path=QDir(dir).filePath(basename);if(QFileInfo::exists(path))return path;}
    return {};
}

void DigitalTwinScanWidget::loadReference(){
    if(m_m8Running){QMessageBox::warning(this,tr("M8 运行中"),tr("请先取消 M8 再更换输入文件。"));return;}
    const auto step=findExample("6061_sensor_connector.step");
    if(mls::twin::CadGeometryAdapter::stepSupported()&&!step.isEmpty()){
        if(importStepFile(step,false)){refreshInfo(tr("已通过 Open CASCADE 导入参考 STEP（真实 B-Rep 三角化）"));return;}
    }
    const auto obj=findExample("6061_reference_mesh.obj");
    if(obj.isEmpty()||!importObjFile(obj,false)){
        QMessageBox::warning(this,tr("找不到测试网格"),tr("找不到或无法载入 examples/digital_twin/6061_reference_mesh.obj"));
        return;
    }
    refreshInfo(tr("已载入随包 OBJ 测试网格（由同一 STEP 预先三角化）。此模式不表示已解析 STEP。"));
}

bool DigitalTwinScanWidget::importStepFile(const QString& path,bool showDialog){
    if(m_m8Running){QMessageBox::warning(this,tr("M8 运行中"),tr("停止 M8 后再更换 CAD 场景。"));return false;}
    if(!mls::twin::CadGeometryAdapter::stepSupported()){
        const QString e=tr("当前构建没有 Open CASCADE；请安装 OCCT 并重新执行 CMake 配置。可以先点击“载入测试零件”验证场景。");
        refreshInfo(e);if(showDialog)QMessageBox::warning(this,tr("STEP 导入不可用"),e);return false;
    }
    // OCCT exposes a narrow ReadFile(char*) path. Stage the input through Qt
    // into a simple ASCII basename so Unicode STEP filenames are supported.
    QTemporaryDir staging(QDir(QDir::tempPath()).filePath("MLS_V28_STEP_XXXXXX"));
    if(!staging.isValid()) {
        if(showDialog)QMessageBox::warning(this,tr("STEP 导入失败"),tr("无法建立 STEP 临时目录。"));
        return false;
    }
    const QString stagedPath=QDir(staging.path()).filePath("import.step");
    if(!QFile::copy(path,stagedPath)) {
        if(showDialog)QMessageBox::warning(this,tr("STEP 导入失败"),tr("无法读取或暂存 STEP 文件。"));
        return false;
    }
    QApplication::setOverrideCursor(Qt::WaitCursor);
    auto result=mls::twin::CadGeometryAdapter::loadStep(nativePath(stagedPath),{m_meshTolerance->value(),.25});
    QApplication::restoreOverrideCursor();
    if(!result.ok){
        const QString e=QString::fromLocal8Bit(result.message.c_str());
        refreshInfo(tr("STEP 导入失败：")+e);
        if(showDialog)QMessageBox::warning(this,tr("STEP 导入失败"),e);
        return false;
    }
    result.mesh.source=nativePath(path); // preserve original, not temporary staging filename
    std::string error;
    if(!m_session.scene.setWorkpiece(std::move(result.mesh),&error)){
        const QString e=QString::fromStdString(error);
        if(showDialog)QMessageBox::warning(this,tr("几何模型错误"),e);
        return false;
    }
    clearVirtual();
    m_filePath=QFileInfo(path).absoluteFilePath();m_sourceMode="STEP";
    m_importNote=tr("STEP B-Rep 导入成功");
    m_fileLabel->setText(tr("STEP：")+m_filePath);
    updateSceneGraphics(true);
    return true;
}

bool DigitalTwinScanWidget::importObjFile(const QString& path,bool showDialog){
    if(m_m8Running){QMessageBox::warning(this,tr("M8 运行中"),tr("停止 M8 后再更换 CAD 场景。"));return false;}
    std::string error;
    if(!m_session.scene.loadObjMesh(nativePath(path),&error)){
        const QString e=QString::fromLocal8Bit(error.c_str());
        refreshInfo(tr("OBJ 导入失败：")+e);
        if(showDialog)QMessageBox::warning(this,tr("网格导入失败"),e);
        return false;
    }
    clearVirtual();
    m_filePath=QFileInfo(path).absoluteFilePath();m_sourceMode="OBJ";
    m_importNote=tr("预三角化 OBJ 网格导入成功（不是 B-Rep STEP）");
    m_fileLabel->setText(tr("OBJ：")+m_filePath);
    updateSceneGraphics(true);
    return true;
}

Pose6 DigitalTwinScanWidget::uiPose() const {
    return {m_pose[0]->value(),m_pose[1]->value(),m_pose[2]->value(),
        m_pose[3]->value(),m_pose[4]->value(),m_pose[5]->value()};
}
void DigitalTwinScanWidget::setUiPose(const Pose6& p){
    const std::array<double,6> values{{p.x,p.y,p.z,p.rxDeg,p.ryDeg,p.rzDeg}};
    for(int i=0;i<6;++i){QSignalBlocker blocker(m_pose[i]);m_pose[i]->setValue(values[static_cast<size_t>(i)]);}
}
void DigitalTwinScanWidget::updatePlacement(){
    if(m_m8Running){cancelM8();return;}
    clearM6();clearM5();clearM51();clearM52();
    if(m_virtual||(m_virtualWatcher&&m_virtualWatcher->isRunning()))clearVirtual();
    m_session.scene.setTruePose(uiPose());
    if(m_idealRegistration->isChecked())m_session.scene.setEstimatedPose(uiPose());
    if(m_view&&m_session.scene.hasWorkpiece())m_view->setTwinWorkpiecePose(m_session.scene.cadToMachineTruth());
    if(m_mainView&&m_session.scene.hasWorkpiece())m_mainView->setTwinWorkpiecePose(m_session.scene.cadToMachineTruth());
    refreshInfo();
}
void DigitalTwinScanWidget::updateSupport(){
    if(m_m8Running){cancelM8();return;}
    clearM6();clearM5();clearM51();clearM52();
    if(m_virtual||(m_virtualWatcher&&m_virtualWatcher->isRunning()))clearVirtual();
    if(!m_supportCheck||!m_supportZ)return;
    m_session.scene.setSupportPlane(m_supportCheck->isChecked(),m_supportZ->value(),
        m_supportX->value(),m_supportY->value(),m_supportW->value(),m_supportH->value());
    if(m_view)m_view->setTwinSupportPlane(m_session.scene.supportEnabled(),
       m_session.scene.supportZ(),m_session.scene.supportCenter(),m_session.scene.supportSize());
    // The support actor is an independent scene layer. It MUST be removed from
    // the main viewer even after CAD has already been cleared.
    if(m_mainView)m_mainView->setTwinSupportPlane(m_session.scene.supportEnabled(),
       m_session.scene.supportZ(),m_session.scene.supportCenter(),m_session.scene.supportSize());
    refreshInfo();
}
void DigitalTwinScanWidget::updateSceneGraphics(bool focus){
    updatePlacement();
    if(m_view&&m_session.scene.hasWorkpiece()){
        m_view->setTwinWorkpieceMesh(m_session.scene.mesh(),m_session.scene.cadToMachineTruth());
        if(focus)m_view->focusTwinWorkpiece();
    }
    syncMainScene(focus);
    refreshInfo();
}
void DigitalTwinScanWidget::refreshInfo(const QString& note){
    if(!m_status)return;
    QString msg=tr("运行模式：完全离线 / M0～M5.1 场景/回放/仿真/对比/CAD覆盖/中心孔诊断（不下发运动指令）\n");
    if(m_session.scene.hasWorkpiece()){
        const auto& mesh=m_session.scene.mesh();const auto local=mesh.bounds;
        const auto world=m_session.scene.workpieceBoundsMachine();
        msg+=tr("三角形：%1；CAD尺寸(mm)：%2 × %3 × %4\n")
            .arg(static_cast<qulonglong>(mesh.triangles.size())).arg(fmt(local.size().x),fmt(local.size().y),fmt(local.size().z));
        msg+=tr("MCS包围盒：[%1, %2, %3] → [%4, %5, %6]\n")
            .arg(fmt(world.min.x),fmt(world.min.y),fmt(world.min.z),fmt(world.max.x),fmt(world.max.y),fmt(world.max.z));
        msg+=tr("当前来源：%1\n").arg(m_importNote);
    } else msg+=tr("当前未加载工件。\n");
    msg+=tr("主视图：%1；CAD模型与采集点云使用不同图层。\n")
       .arg(m_mainView ? tr("已连接/自动同步") : tr("尚未连接"));
    msg+=tr("坐标约定：X/Y/Z 毫米；Rz·Ry·Rx 工件姿态；与 B/C 运动学独立。\n");
    if(!note.isEmpty())msg+=note;
    m_status->setText(msg);
}

void DigitalTwinScanWidget::saveSession(){
    if(!m_session.scene.hasWorkpiece()){
        QMessageBox::information(this,tr("未加载工件"),tr("请先导入工件，再保存场景。"));return;
    }
    const QString file=QFileDialog::getSaveFileName(this,tr("保存数字孪生场景"),
                                                    QString(),tr("数字孪生场景 (*.twin.json *.json)"));
    if(file.isEmpty())return;
    const auto truth=m_session.scene.truePose();
    const auto estimate=m_session.scene.estimatedPose();
    QJsonObject support{{"enabled",m_session.scene.supportEnabled()},
        {"z",m_session.scene.supportZ()},{"x",m_session.scene.supportCenter().x},
        {"y",m_session.scene.supportCenter().y},{"width",m_session.scene.supportSize().x},
        {"height",m_session.scene.supportSize().y}};
    QJsonObject json{{"format","MotionLaserSync.DigitalTwinScene"},{"version",1},
        {"unit","mm"},{"rotationConvention","Rz*Ry*Rx_degrees"},
        {"cadPath",QDir(QFileInfo(file).absolutePath()).relativeFilePath(m_filePath)},
        {"sourceMode",m_sourceMode},{"poseTruth",jsonPose(truth)},
        {"poseEstimate",jsonPose(estimate)},{"idealRegistration",m_idealRegistration->isChecked()},
        {"support",support},{"stepTessellationMm",m_meshTolerance->value()}};
    QSaveFile out(file);
    if(!out.open(QIODevice::WriteOnly)||out.write(QJsonDocument(json).toJson(QJsonDocument::Indented))<0||!out.commit()){
        QMessageBox::warning(this,tr("保存失败"),out.errorString());return;
    }
    refreshInfo(tr("已保存场景：")+file);
}

void DigitalTwinScanWidget::loadSession(){
    if(m_m8Running){QMessageBox::warning(this,tr("M8 运行中"),tr("请先取消 M8 再更换输入文件。"));return;}
    const QString file=QFileDialog::getOpenFileName(this,tr("载入数字孪生场景"),
        QString(),tr("数字孪生场景 (*.twin.json *.json)"));
    if(file.isEmpty())return;
    QFile f(file);if(!f.open(QIODevice::ReadOnly)){
        QMessageBox::warning(this,tr("打开失败"),f.errorString());return;
    }
    QJsonParseError err{};
    const QJsonDocument doc=QJsonDocument::fromJson(f.readAll(),&err);
    const auto o=doc.object();
    Pose6 truth{},estimate{};
    if(err.error!=QJsonParseError::NoError||!doc.isObject()||o.value("format").toString()!="MotionLaserSync.DigitalTwinScene"
       ||o.value("version").toInt()!=1||o.value("unit").toString()!="mm"
       ||o.value("rotationConvention").toString()!="Rz*Ry*Rx_degrees"
       ||!readPose(o.value("poseTruth").toObject(),&truth)
       ||!readPose(o.value("poseEstimate").toObject(),&estimate)){
        QMessageBox::warning(this,tr("场景格式错误"),tr("不是有效的 V28 数字孪生场景 JSON。"));return;
    }
    QString path=o.value("cadPath").toString();
    if(QDir::isRelativePath(path))path=QDir(QFileInfo(file).absolutePath()).filePath(path);
    const auto type=o.value("sourceMode").toString();
    // Import first. The existing scene remains intact if an import fails.
    if(type=="STEP"){
        if(!importStepFile(path))return;
    }else if(type=="OBJ"){
        if(!importObjFile(path))return;
    }else{
        QMessageBox::warning(this,tr("场景格式错误"),tr("未知 CAD sourceMode。"));return;
    }
    setUiPose(truth);
    m_idealRegistration->setChecked(false);
    m_session.scene.setTruePose(truth);
    m_session.scene.setEstimatedPose(estimate);
    {
        QSignalBlocker lock(m_supportCheck);
        const auto sup=o.value("support").toObject();
        m_supportCheck->setChecked(sup.value("enabled").toBool());
        for(const auto& pair:std::initializer_list<std::pair<QDoubleSpinBox*,double>>{
            {m_supportZ,sup.value("z").toDouble(-225.05)},
            {m_supportX,sup.value("x").toDouble(42.)},
            {m_supportY,sup.value("y").toDouble(10.)},
            {m_supportW,sup.value("width").toDouble(140.)},
            {m_supportH,sup.value("height").toDouble(90.)}}){
            QSignalBlocker block(pair.first);pair.first->setValue(pair.second);
        }
    }
    m_idealRegistration->setChecked(o.value("idealRegistration").toBool(false));
    if(m_idealRegistration->isChecked())m_session.scene.setEstimatedPose(truth);
    updatePlacement();updateSupport();
    m_view->resetCamera();
    refreshInfo(tr("已载入场景：")+file);
}

void DigitalTwinScanWidget::tryLoadMachine(){
    const QStringList files={
        QDir(QCoreApplication::applicationDirPath()).filePath("models/machine5x/machine_config.json"),
        QDir::current().filePath("models/machine5x/machine_config.json")
    };
    for(const auto& path:files){
        if(!QFileInfo::exists(path))continue;
        QString error;
        if(m_view->loadMachineModel(path,&error)){
            refreshInfo(tr("可选机床视觉模型已加载，机床姿态保持模拟零位。"));
            return;
        }
        QMessageBox::warning(this,tr("机床视觉资源缺失"),tr("配置文件存在，但视觉 PLY/STL 未就绪：\n%1\n当前 STEP 场景仍可以独立显示。")
                             .arg(error));return;
    }
    QMessageBox::information(this,tr("机床模型不可用"),tr("尚未找到 models/machine5x/machine_config.json；工件仍使用机床 XYZ 坐标。"));
}


DigitalTwinScanWidget::~DigitalTwinScanWidget(){
    stopReplay();
    // The QtConcurrent tasks capture only file paths or immutable shared replay
    // sessions. QFutureWatcher itself does not cancel a running task on delete.
    if(m_replayWatcher && m_replayWatcher->isRunning())m_replayWatcher->waitForFinished();
    if(m_exportWatcher && m_exportWatcher->isRunning())m_exportWatcher->waitForFinished();
    if(m_virtualCancel)m_virtualCancel->store(true);
    if(m_virtualWatcher && m_virtualWatcher->isRunning())m_virtualWatcher->waitForFinished();
    if(m_virtualExportWatcher && m_virtualExportWatcher->isRunning())m_virtualExportWatcher->waitForFinished();
    if(m_m4Cancel)m_m4Cancel->store(true);
    if(m_m4CompareWatcher&&m_m4CompareWatcher->isRunning())m_m4CompareWatcher->waitForFinished();
    if(m_m4FitWatcher&&m_m4FitWatcher->isRunning())m_m4FitWatcher->waitForFinished();
    if(m_m5Cancel)m_m5Cancel->store(true);
    if(m_m5Watcher&&m_m5Watcher->isRunning())m_m5Watcher->waitForFinished();
    if(m_m5ExportWatcher&&m_m5ExportWatcher->isRunning())m_m5ExportWatcher->waitForFinished();
    if(m_m51Cancel)m_m51Cancel->store(true);
    if(m_m51Watcher&&m_m51Watcher->isRunning())m_m51Watcher->waitForFinished();
    if(m_m51ExportWatcher&&m_m51ExportWatcher->isRunning())m_m51ExportWatcher->waitForFinished();
    if(m_m7Cancel)m_m7Cancel->store(true);
    if(m_m7Watcher&&m_m7Watcher->isRunning())m_m7Watcher->waitForFinished();
    if(m_m7ExportWatcher&&m_m7ExportWatcher->isRunning())m_m7ExportWatcher->waitForFinished();
    if(m_m6Cancel)m_m6Cancel->store(true);
    if(m_m6ExportCancel)m_m6ExportCancel->store(true);
    if(m_m6Watcher&&m_m6Watcher->isRunning())m_m6Watcher->waitForFinished();
    if(m_m6ExportWatcher&&m_m6ExportWatcher->isRunning())m_m6ExportWatcher->waitForFinished();
    if(m_m52Cancel)m_m52Cancel->store(true);
    if(m_m52Watcher&&m_m52Watcher->isRunning())m_m52Watcher->waitForFinished();
    if(m_m52ExportWatcher&&m_m52ExportWatcher->isRunning())m_m52ExportWatcher->waitForFinished();
    if(m_twinTargetsExportWatcher && m_twinTargetsExportWatcher->isRunning())
        m_twinTargetsExportWatcher->waitForFinished();
}

void DigitalTwinScanWidget::setMainPointCloudView(PointCloudWidget* view){
    m_mainView=view;
    if(m_session.scene.hasWorkpiece())syncMainScene(true);
    if(m_mainView && m_replay && m_replayDisplayedEnd>0)
        m_mainView->appendTwinReplayPoints(m_replay->previewPoints,0,m_replayDisplayedEnd);
    if(m_mainView && m_virtual)m_mainView->setTwinVirtualPoints(m_virtual->previewPoints);
    if(m_mainView && m_m7Data && m_m7Data->ok())
        m_mainView->setTwinM7Results(m_m7Data->realPoints,m_m7Data->simulatedPoints,
            m_m7Data->overlapPoints,m_m7Data->newlyCoveredCad,m_m7Data->stillMissingCad,
            m_m7Data->fusedPreview);
    if(m_mainView && m_m6Data && m_m6Data->ok())
        m_mainView->setTwinM6DetailedResults(m_m6Data->segmentPointPreviews,
            m_m6Data->segmentLaserPaths,m_m6Data->transferLaserPath);
    if(m_mainView && m_m5Data && m_m5Data->ok())
        m_mainView->setTwinCoveragePoints(m_m5Data->realObservedPreview,
            m_m5Data->realMissingPreview,m_m5Data->virtualOnlyPreview);
    if(m_mainView){
        for(auto it=m_twinLayerChecks.constBegin();it!=m_twinLayerChecks.constEnd();++it)
            m_mainView->setTwinOverlayVisible(it.key(),it.value()->isChecked());
        refreshTwinInspector();
    }
}

void DigitalTwinScanWidget::syncMainScene(bool focus){
    if(!m_mainView)return;
    if(m_session.scene.hasWorkpiece()){
        m_mainView->setTwinWorkpieceMesh(m_session.scene.mesh(),m_session.scene.cadToMachineTruth());
        if(focus)m_mainView->focusTwinWorkpiece();
    }
    m_mainView->setTwinSupportPlane(m_session.scene.supportEnabled(),
       m_session.scene.supportZ(),m_session.scene.supportCenter(),m_session.scene.supportSize());
}

void DigitalTwinScanWidget::browseReplayScan(){
    if(m_m8Running){QMessageBox::warning(this,tr("M8 运行中"),tr("请先取消 M8 再更换输入文件。"));return;}
    if(m_replayWatcher->isRunning()||(m_virtualWatcher&&m_virtualWatcher->isRunning()))return;
    const QString file=QFileDialog::getOpenFileName(this,tr("选择 SCAN v3 实测数据"),
        m_replayFilePath.isEmpty()?QString():QFileInfo(m_replayFilePath).absolutePath(),tr("SCAN 实测数据 (*.scan)"));
    if(file.isEmpty())return;
    stopReplay();
    m_discardReplayOnFinish=false;
    m_replayOpen->setEnabled(false);
    m_replayPlay->setEnabled(false);
    m_replayExport->setEnabled(false);
    m_replayInfo->setText(tr("正在后台读取与 CRC 验证 SCAN v3，生成有限预览点，请稍候…"));
    // Worker does not access widgets, machine controllers, or a PointCloudBuilder.
    m_replayWatcher->setFuture(QtConcurrent::run([file]{
        return mls::twin::ScanReplayAdapter::load(file,250000);
    }));
}

void DigitalTwinScanWidget::onReplayLoaded(){
    m_replayOpen->setEnabled(true);
    auto result=m_replayWatcher->result();
    if(m_discardReplayOnFinish){
        m_discardReplayOnFinish=false;
        m_replayInfo->setText(tr("已忽略刚导入的 SCAN；离线回放数据已清除。"));
        return;
    }
    if(!result || !result->ok()){
        const QString message=result ? result->error : tr("未知读取错误");
        m_replayInfo->setText(tr("无法载入：")+message);
        QMessageBox::warning(this,tr("M2 回放导入失败"),message);
        m_replayPlay->setEnabled(static_cast<bool>(m_replay));
        m_replayExport->setEnabled(static_cast<bool>(m_replay));
        return;
    }
    clearVirtual();
    if(m_view)m_view->clearTwinReplayPoints();
    if(m_mainView)m_mainView->clearTwinReplayPoints();
    m_replay=std::move(result);
    m_replayFilePath=m_replay->sourcePath;
    m_replayIndex=-1;m_replayDisplayedEnd=0;
    m_replaySlider->setEnabled(true);
    m_replaySlider->setRange(0,m_replay->frames.size()-1);
    m_replayPlay->setEnabled(true);
    m_replayExport->setEnabled(true);
    if(m_replay->hasPreviewBounds){
        if(m_view)m_view->focusTwinRegion(m_replay->previewBoundsMin,m_replay->previewBoundsMax);
        if(m_mainView)m_mainView->focusTwinRegion(m_replay->previewBoundsMin,m_replay->previewBoundsMax);
    }
    {QSignalBlocker b(m_replaySlider);m_replaySlider->setValue(0);}
    applyReplayFrame(0);
}

void DigitalTwinScanWidget::seekReplay(int index){
    if(!m_replay)return;
    stopReplay();
    applyReplayFrame(index);
}

void DigitalTwinScanWidget::applyReplayFrame(int index){
    if(!m_replay||m_replay->frames.isEmpty())return;
    index=std::max(0,std::min(index,int(m_replay->frames.size())-1));
    const auto& frame=m_replay->frames[index];
    if(index<m_replayIndex){
        m_replayDisplayedEnd=0;
        if(m_view)m_view->clearTwinReplayPoints();
        if(m_mainView)m_mainView->clearTwinReplayPoints();
    }
    if(frame.previewEnd>m_replayDisplayedEnd){
        if(m_view)m_view->appendTwinReplayPoints(m_replay->previewPoints,m_replayDisplayedEnd,frame.previewEnd);
        if(m_mainView)m_mainView->appendTwinReplayPoints(m_replay->previewPoints,m_replayDisplayedEnd,frame.previewEnd);
        m_replayDisplayedEnd=frame.previewEnd;
    }
    m_replayIndex=index;
    if(m_view)m_view->setMachineJointPose(frame.joints[0],frame.joints[1],frame.joints[2],frame.joints[3],frame.joints[4]);
    if(m_mainView)m_mainView->setMachineJointPose(frame.joints[0],frame.joints[1],frame.joints[2],frame.joints[3],frame.joints[4]);
    {QSignalBlocker b(m_replaySlider);m_replaySlider->setValue(index);}
    updateReplayInfo();
}

void DigitalTwinScanWidget::updateReplayInfo(){
    if(!m_replay)return;
    const auto& f=m_replay->frames[m_replayIndex<0?0:m_replayIndex];
    const bool simulated=m_replay && !m_replay->scan.profiles.isEmpty() &&
        m_replay->scan.profiles.first().fiveAxis.frame.timing==mls::runtime::PoseTiming::Simulated;
    m_replayInfo->setText((simulated
        ?tr("⚠ SIMULATED 虚拟 SCAN（无硬件反馈/锁存，不可作为 M6 实测初扫）\n")
        :QString())+tr("来源：%1\n帧 %2/%3 | 采集序号 %4 | 虚拟时间 %5/%6 s\n"
        "XYZCB: [%7, %8, %9, %10°, %11°]\n"
        "原始有效点：%12 | 当前显示预览点：%13/%14（最大25万，视觉降采样）\n"
        "绿色点=按 SCAN Frame5D.sensorToMachine 单次转换的返回点；实际或模拟来源以 PoseTiming 为准。")
        .arg(QFileInfo(m_replayFilePath).fileName())
        .arg(m_replayIndex+1).arg(m_replay->frames.size())
        .arg(static_cast<qulonglong>(f.ordinal))
        .arg(f.elapsedSeconds,0,'f',3)
        .arg(m_replay->frames.last().elapsedSeconds,0,'f',3)
        .arg(f.joints[0],0,'f',3).arg(f.joints[1],0,'f',3).arg(f.joints[2],0,'f',3)
        .arg(f.joints[3],0,'f',3).arg(f.joints[4],0,'f',3)
        .arg(static_cast<qulonglong>(m_replay->totalRawPoints))
        .arg(static_cast<qlonglong>(m_replayDisplayedEnd))
        .arg(m_replay->previewPoints.size()));
}

void DigitalTwinScanWidget::startReplay(){
    if(!m_replay || m_replay->frames.isEmpty())return;
    if(m_replayIndex>=m_replay->frames.size()-1)applyReplayFrame(0);
    m_playVirtualStart=m_replay->frames[std::max(0,m_replayIndex)].elapsedSeconds;
    m_replayClock.start();m_replayTimer->start();
}
void DigitalTwinScanWidget::stopReplay(){
    if(m_replayTimer)m_replayTimer->stop();
}
void DigitalTwinScanWidget::resetReplay(){stopReplay();applyReplayFrame(0);}
void DigitalTwinScanWidget::onReplayTick(){
    if(!m_replay){stopReplay();return;}
    const double goal=m_playVirtualStart+(.001*m_replayClock.elapsed())*m_replaySpeed->currentData().toDouble();
    const auto& frames=m_replay->frames;
    auto it=std::upper_bound(frames.cbegin(),frames.cend(),goal,
        [](double t,const mls::twin::ScanReplayFrame& f){return t<f.elapsedSeconds;});
    const int index=std::max(0,int(it-frames.cbegin())-1);
    if(index!=m_replayIndex)applyReplayFrame(index);
    if(index>=frames.size()-1)stopReplay();
}
void DigitalTwinScanWidget::exportReplayPly(){
    if(!m_replay||m_exportWatcher->isRunning())return;
    const QString file=QFileDialog::getSaveFileName(this,tr("导出完整实测重建 PLY"),
        QFileInfo(m_replayFilePath).absolutePath()+"/scan_reconstructed_full.ply",tr("PLY 二进制点云 (*.ply)"));
    if(file.isEmpty())return;
    m_replayExport->setEnabled(false);
    const auto replay=m_replay;
    m_exportWatcher->setFuture(QtConcurrent::run([replay,file]{
        return mls::twin::ScanReplayAdapter::exportFullPly(*replay,file);
    }));
}

void DigitalTwinScanWidget::closeEvent(QCloseEvent* event){
    if(m_m8Running)cancelM8();
    // Preserve M1 placement and M2 accumulated replay actor on the main viewer,
    // but do not keep advancing the offline machine when the editor is hidden.
    stopReplay();
    if(m_m7Cancel && m_m7Watcher && m_m7Watcher->isRunning())m_m7Cancel->store(true);
    if(m_m6Cancel && m_m6Watcher && m_m6Watcher->isRunning())m_m6Cancel->store(true);
    QDialog::closeEvent(event);
}

// V28 M0/M1/M2 cleanup: clear ONLY offline digital-twin overlays, never live acquisition.
void DigitalTwinScanWidget::clearWorkpiece(){
    if(m_m8Running){cancelM8();return;}
    cancelVirtualSimulation();
    clearVirtual();clearM51();clearM52();
    m_session.scene.clear(); // re-apply UI support settings below
    m_filePath.clear();m_sourceMode.clear();m_importNote.clear();
    if(m_fileLabel)m_fileLabel->setText(tr("当前未加载零件"));
    if(m_view)m_view->clearTwinWorkpiece();
    if(m_mainView)m_mainView->clearTwinWorkpiece();
    updateSupport();
    refreshInfo(tr("已从主视图与编辑视图同时删除 STEP/OBJ 工件；未删除磁盘文件。"));
}
void DigitalTwinScanWidget::clearSupport(){
    cancelVirtualSimulation();clearVirtual();
    if(m_supportCheck){QSignalBlocker b(m_supportCheck);m_supportCheck->setChecked(false);}
    updateSupport();
    refreshInfo(tr("已隐藏数字孪生支承面。"));
}
void DigitalTwinScanWidget::clearReplay(){
    if(m_m8Running){cancelM8();return;}
    stopReplay();cancelVirtualSimulation();clearVirtual();
    // Import worker must not race with a UI-side reset; defer cleanup until it finishes.
    if(m_replayWatcher&&m_replayWatcher->isRunning()){
        m_discardReplayOnFinish=true;
        m_replayInfo->setText(tr("正在导入文件；完成后将自动丢弃该数据。"));
    }
    m_replay.reset();m_replayFilePath.clear();
    m_replayIndex=-1;m_replayDisplayedEnd=0;
    if(m_replaySlider){QSignalBlocker b(m_replaySlider);m_replaySlider->setRange(0,0);
        m_replaySlider->setValue(0);m_replaySlider->setEnabled(false);}
    if(m_replayPlay)m_replayPlay->setEnabled(false);
    if(m_replayExport)m_replayExport->setEnabled(false);
    if(m_view)m_view->clearTwinReplayPoints();
    if(m_mainView)m_mainView->clearTwinReplayPoints();
    if(m_replayInfo)m_replayInfo->setText(tr("已清除 M2 导入的原始 SCAN 数据和回放点云。"
        "机床视觉姿态保留最后状态，不向真实控制器发送任何指令。"));
}
void DigitalTwinScanWidget::clearVirtual(){
    if(m_virtualCancel)m_virtualCancel->store(true);
    m_virtualDiscard=true; // also suppress a queued finished() after the worker exits
    if(m_virtualProgressTimer)m_virtualProgressTimer->stop();
    m_virtual.reset();
    clearM4(); // M4 metrics/calibration become stale when the M3 scan changes.
    clearM5(); // M5 CAD occupancy depends on M3 and scene contents.
    clearM51(); // M5.1 direct M2/M3 density depends on scene and virtual data.
    clearM52(); // M5.2 real/M3 geometry residuals depend on scene and source data.
    if(m_view)m_view->clearTwinVirtualPoints();
    if(m_mainView)m_mainView->clearTwinVirtualPoints();
    if(m_virtualExportButton)m_virtualExportButton->setEnabled(false);
    if(m_virtualRunButton)m_virtualRunButton->setEnabled(!m_virtualWatcher || !m_virtualWatcher->isRunning());
    if(m_virtualCancelButton && (!m_virtualWatcher || !m_virtualWatcher->isRunning()))
        m_virtualCancelButton->setEnabled(false);
    if(m_virtualInfo)m_virtualInfo->setText(tr("M3 虚拟点云/结果已清除，保留 STEP 和 M2 数据。"));
}
void DigitalTwinScanWidget::clearAllTwinData(){
    if(m_m8Running){cancelM8();return;}
    if(QMessageBox::question(this,tr("清空数字孪生数据"),
         tr("清除 CAD、支承面、M2 回放和 M3 虚拟点云吗？"
            "\n不会删除磁盘原文件，也不会清空真实采集点云或操作机床。"),
         QMessageBox::Yes|QMessageBox::No,QMessageBox::No)!=QMessageBox::Yes)return;
    clearReplay();clearWorkpiece();clearSupport();
    setUiPose(Pose6{});
    refreshInfo(tr("已清空所有离线数字孪生图层（仅内存中的加载内容）。"));
}
void DigitalTwinScanWidget::cancelVirtualSimulation(){
    if(m_virtualCancel)m_virtualCancel->store(true);
    if(m_virtualWatcher&&m_virtualWatcher->isRunning()&&m_virtualInfo)
       m_virtualInfo->setText(tr("正在请求取消 M3 仿真；后台任务将在下一条 Profile 前停止…"));
}
void DigitalTwinScanWidget::startVirtualSimulation(){
    if(m_m8Running && (m_m8Index>=m_m8Sequence.size() ||
        m_m8Sequence[m_m8Index]!=mls::twin::M8Stage::VirtualM3))return;
    if(!m_session.scene.hasWorkpiece()){
        QMessageBox::information(this,tr("缺少 CAD 场景"),tr("请先导入 STEP 或参考测试工件。"));return;
    }
    if(!m_replay || !m_replay->ok()){
        QMessageBox::information(this,tr("缺少扫描位姿"),tr("请先在 M2 导入带五轴曝光位姿的 SCAN v3。"));return;
    }
    if(m_virtualWatcher->isRunning()||m_replayWatcher->isRunning())return;
    clearVirtual();
    // This M3 stage simulates the ALREADY RECORDED optical/pose sweep,
    // it does not generate new machine trajectories or issue commands.
    mls::twin::Sensor2950Geometry optics;
    optics.cameraOriginSensor.y=m_cameraBaselineY->value();
    std::string error;
    if(!optics.validate(&error)){
        QMessageBox::warning(this,tr("光学参数不合法"),QString::fromStdString(error));return;
    }
    const auto scene=m_session.scene;  // immutable snapshot for the worker
    const auto replay=m_replay;
    m_virtualCancel=std::make_shared<std::atomic<bool>>(false);
    m_virtualProgress=std::make_shared<std::atomic<std::size_t>>(0);
    m_virtualDiscard=false;
    auto cancel=m_virtualCancel;
    auto progress=m_virtualProgress;
    m_virtualRunButton->setEnabled(false);
    if(m_virtualCancelButton)m_virtualCancelButton->setEnabled(true);
    m_virtualExportButton->setEnabled(false);
    m_virtualInfo->setText(tr("M3 后台扫描已启动：依据逐帧曝光位姿生成 1280 个索引，分别检测激光命中与相机遮挡…"));
    m_virtualProgressTimer->start();
    m_virtualWatcher->setFuture(QtConcurrent::run([scene,replay,optics,cancel,progress]{
        // Keep an allocation/geometry error from escaping the QtConcurrent worker
        // and leaving the GUI waiting on a failed future without a useful report.
        try {
            return mls::twin::TwinSimulationAdapter::run(scene,optics,*replay,
                                                         cancel.get(),progress.get(),250000);
        } catch(const std::exception& e) {
            auto failed=std::make_shared<mls::twin::TwinSimulationData>();
            failed->error=QStringLiteral("M3 后台异常：")+QString::fromLocal8Bit(e.what());
            return failed;
        } catch(...) {
            auto failed=std::make_shared<mls::twin::TwinSimulationData>();
            failed->error=QStringLiteral("M3 后台发生未知异常");
            return failed;
        }
    }));
}
void DigitalTwinScanWidget::updateVirtualProgress(){
    // If the user cleared the layer, do not show stale progress from a cancelled worker.
    if(m_virtualDiscard||!m_virtualWatcher||!m_virtualWatcher->isRunning())return;
    const auto completed=m_virtualProgress?m_virtualProgress->load():0;
    const auto total=m_replay?m_replay->scan.profiles.size():0;
    if(m_virtualInfo)m_virtualInfo->setText(tr("M3 近似光学仿真进行中：%1 / %2 帧；可点击取消。"
        "\n基于当前 STEP 场景与真实 SCAN 五轴曝光位姿；不使用设备。")
        .arg(static_cast<qulonglong>(completed)).arg(total));
}
void DigitalTwinScanWidget::onVirtualFinished(){
    if(m_virtualProgressTimer)m_virtualProgressTimer->stop();
    if(m_virtualRunButton)m_virtualRunButton->setEnabled(true);
    const auto result=m_virtualWatcher->result();
    if(m_virtualDiscard){m_virtualDiscard=false;onM8StageFinished(mls::twin::M8Stage::VirtualM3,false,tr("旧结果已清除"));return;}
    if(!result||!result->ok()){
        if(m_virtualInfo)m_virtualInfo->setText(tr("M3 未完成：%1")
            .arg(result?result->error:tr("未知错误")));
        onM8StageFinished(mls::twin::M8Stage::VirtualM3,false,tr("运行失败或取消"));return;
    }
    m_virtual=result;
    if(m_view)m_view->setTwinVirtualPoints(result->previewPoints);
    if(m_mainView)m_mainView->setTwinVirtualPoints(result->previewPoints);
    if(m_virtualExportButton)m_virtualExportButton->setEnabled(true);
    if(m_m4RunButton)m_m4RunButton->setEnabled(true);
    if(m_m4FitButton)m_m4FitButton->setEnabled(true);
    const auto& c=result->scan.counts;
    m_virtualInfo->setText(tr("M3 光学几何仿真完成（未校准真实光心！）\n"
        "轮廓：%1；有效回波：%2；激光无交点：%3；相机遮挡：%4；超出测量场：%5；坏位姿：%6。\n"
        "界面预览点：%7（最多25万）。橙红色=M3，绿色=M2；原始有效点可导出完整 PLY。")
        .arg(static_cast<qulonglong>(result->scan.totalProfiles))
        .arg(static_cast<qulonglong>(c[0]))
        .arg(static_cast<qulonglong>(c[1]))
        .arg(static_cast<qulonglong>(c[2]))
        .arg(static_cast<qulonglong>(c[3]))
        .arg(static_cast<qulonglong>(c[4]))
        .arg(result->previewPoints.size()));
    onM8StageFinished(mls::twin::M8Stage::VirtualM3,true);
}
void DigitalTwinScanWidget::exportVirtualResults(){
    if(!m_virtual||!m_virtual->ok()||m_virtualExportWatcher->isRunning())return;
    const auto target=QFileDialog::getSaveFileName(this,tr("M3 导出完整虚拟 PLY"),
         QFileInfo(m_replayFilePath).absolutePath()+"/twin_m3_virtual_full.ply",
         tr("点云 PLY (*.ply)"));
    if(target.isEmpty())return;
    m_virtualExportButton->setEnabled(false);
    const auto snapshot=m_virtual;
    m_virtualExportWatcher->setFuture(QtConcurrent::run([snapshot,target]{
        QString err=mls::twin::TwinSimulationAdapter::exportFullPly(*snapshot,target);
        if(!err.isEmpty())return err;
        QString mask=target;
        if(mask.endsWith(".ply",Qt::CaseInsensitive))mask.chop(4);
        mask+=".m3prof";
        err=mls::twin::TwinSimulationAdapter::exportMaskBinary(*snapshot,mask);
        if(!err.isEmpty())return err;
        QString meta=target;
        if(meta.endsWith(".ply",Qt::CaseInsensitive))meta.chop(4);
        meta+=".m3.json";
        return mls::twin::TwinSimulationAdapter::exportMetadataJson(*snapshot,meta);
    }));
}

// -------------------- M4: real vs virtual indexed Profile validation --------------------
void DigitalTwinScanWidget::clearM4(){
    for(auto* v:{m_view,m_mainView.data()})if(v)v->setTwinStageMarkers(QStringLiteral("M4_ROI"),{});
    if(m_m4Cancel)m_m4Cancel->store(true);
    m_m4Discard=true;
    if(m_m4ProgressTimer)m_m4ProgressTimer->stop();
    m_m4Comparison.reset();m_m4Fit.reset();
    if(m_m4ExportButton)m_m4ExportButton->setEnabled(false);
    if(m_m4FitExportButton)m_m4FitExportButton->setEnabled(false);
    if(m_m4ApplyButton)m_m4ApplyButton->setEnabled(false);
    const bool active=(m_m4CompareWatcher&&m_m4CompareWatcher->isRunning())||
                      (m_m4FitWatcher&&m_m4FitWatcher->isRunning());
    if(m_m4RunButton)m_m4RunButton->setEnabled(!active && bool(m_virtual));
    if(m_m4FitButton)m_m4FitButton->setEnabled(!active && bool(m_virtual));
    if(m_m4CancelButton)m_m4CancelButton->setEnabled(active);
    if(m_m4Info)m_m4Info->setPlainText(tr("M4 分析已清除。原始 SCAN 和 CAD/M3 数据保留，数据已变更时需重新运行。"));
}
void DigitalTwinScanWidget::cancelM4(){
    if(m_m4Cancel)m_m4Cancel->store(true);
    if(m_m4Info)m_m4Info->appendPlainText(tr("正在取消 M4 后台任务，请等待正在计算的轮廓完成…"));
}
void DigitalTwinScanWidget::startM4Comparison(){
    if(m_m8Running && (m_m8Index>=m_m8Sequence.size() ||
        m_m8Sequence[m_m8Index]!=mls::twin::M8Stage::ComparisonM4))return;
    if(!m_replay||!m_replay->ok()||!m_virtual||!m_virtual->ok()){
        QMessageBox::information(this,tr("缺少 M4 输入"),tr("请先导入 M2 .scan 并完成 M3 虚拟扫描。"));return;
    }
    if(m_m4CompareWatcher->isRunning()||m_m4FitWatcher->isRunning()||m_virtualWatcher->isRunning())return;
    m_m4Comparison.reset();
    m_m4Discard=false;
    m_m4Cancel=std::make_shared<std::atomic<bool>>(false);
    m_m4Progress=std::make_shared<std::atomic<std::size_t>>(0);
    const auto cancel=m_m4Cancel;
    const auto progress=m_m4Progress;
    const auto real=m_replay;
    const auto virtualData=m_virtual;
    const auto scene=m_session.scene;
    m_m4RunButton->setEnabled(false);m_m4FitButton->setEnabled(false);
    m_m4CancelButton->setEnabled(true);m_m4ExportButton->setEnabled(false);
    m_m4Info->setPlainText(tr("M4 正在后台逐帧比对 1280 个索引及孔区域。\n真实扫描缺失原因未知，不推断成 CameraOccluded。"));
    m_m4ProgressTimer->start();
    m_m4CompareWatcher->setFuture(QtConcurrent::run([scene,real,virtualData,cancel,progress]{
        try{
            return mls::twin::TwinM4Adapter::compare(*real,*virtualData,scene,cancel.get(),progress.get());
        }catch(const std::exception& e){
            auto failed=std::make_shared<mls::twin::TwinM4ComparisonData>();
            failed->message=QStringLiteral("M4 后台异常：")+QString::fromLocal8Bit(e.what());return failed;
        }catch(...){
            auto failed=std::make_shared<mls::twin::TwinM4ComparisonData>();
            failed->message=QStringLiteral("M4 未知异常");return failed;
        }
    }));
}
void DigitalTwinScanWidget::onM4ComparisonFinished(){
    if(m_m4ProgressTimer)m_m4ProgressTimer->stop();
    m_m4CancelButton->setEnabled(false);
    const auto result=m_m4CompareWatcher->result();
    if(m_m4Discard){m_m4Discard=false;onM8StageFinished(mls::twin::M8Stage::ComparisonM4,false,tr("旧结果已清除"));return;}
    m_m4RunButton->setEnabled(bool(m_virtual));m_m4FitButton->setEnabled(bool(m_virtual));
    if(!result||!result->ok()){
        m_m4Info->setPlainText(tr("M4 比较未完成：%1").arg(result?result->message:tr("未知错误")));
        onM8StageFinished(mls::twin::M8Stage::ComparisonM4,false,tr("运行失败或取消"));return;
    }
    m_m4Comparison=result;
    m_m4ExportButton->setEnabled(true);
    const auto& r=result->result;
    const auto metric=[](const mls::twin::M4Metrics& m){
        return QStringLiteral("TP=%1, FP=%2, FN=%3, TN=%4，F1=%5，IoU=%6，配对Z绝对误差=%7mm")
            .arg(static_cast<qulonglong>(m.truePositive))
            .arg(static_cast<qulonglong>(m.falsePositive))
            .arg(static_cast<qulonglong>(m.falseNegative))
            .arg(static_cast<qulonglong>(m.trueNegative))
            .arg(QString::number(m.f1(),'f',4))
            .arg(QString::number(m.iou(),'f',4))
            .arg(QString::number(m.meanAbsZ(),'f',4));
    };
    QString summary=tr("M4 全量对比完成：%1 帧；拟合相机 Y=%2mm (仅为当前 M3 近似模型)。\n")
        .arg(static_cast<qulonglong>(r.profilesCompared))
        .arg(result->model.cameraOriginSensor.y,0,'f',1);
    summary+=tr("全局：")+metric(r.global)+"\n";
    if(!r.perHole.empty()){
        summary+=tr("四沉孔投影 ROI：")+metric(r.holeRegions)+"\n";
        for(std::size_t i=0;i<r.perHole.size();++i){
            const auto& f=r.floor[i];
            summary+=tr("沉孔 %1：ROI F1=%2；台阶真实/虚拟采样=%3/%4；"
                        "台阶扇区 IoU=%5；+Y 点数比例=%6/%7\n")
                .arg(static_cast<qulonglong>(i+1))
                .arg(r.perHole[i].f1(),0,'f',3)
                .arg(static_cast<qulonglong>(f.realFloorPoints))
                .arg(static_cast<qulonglong>(f.simFloorPoints))
                .arg(f.intersectionOverUnion(),0,'f',3)
                .arg(f.realPositiveRatio(),0,'f',3)
                .arg(f.simulatedPositiveRatio(),0,'f',3);
        }
    }else summary+=tr("当前工件未匹配 H1 模板：仅报告全局对比，不能输出四孔结果。\n");
    summary+=tr("注意：没有原始返回值的采样只计为实测未知缺失；"
                "台阶 36×3 分箱为近似覆盖诊断，不是 CAD 曲面测量精度。\n可导出 JSON 和逐帧 CSV。");
    m_m4Info->setPlainText(summary);
    onM8StageFinished(mls::twin::M8Stage::ComparisonM4,true);
    refreshTwinInspector();
}
void DigitalTwinScanWidget::startM4Fit(){
    if(!m_replay||!m_replay->ok()||!m_virtual||!m_virtual->ok()||!m_session.scene.hasWorkpiece()){
        QMessageBox::information(this,tr("缺少拟合输入"),tr("请先载入 H1 STEP、M2 SCAN，并完成 M3。"));return;
    }
    if(m_m4CompareWatcher->isRunning()||m_m4FitWatcher->isRunning()||m_virtualWatcher->isRunning())return;
    mls::twin::M4FitOptions options;
    options.minSensorY=m_m4MinY->value();
    options.maxSensorY=m_m4MaxY->value();
    options.stepSensorY=m_m4StepY->value();
    options.maxFrames=std::size_t(m_m4FrameCap->value());
    if(options.minSensorY>=options.maxSensorY){
        QMessageBox::warning(this,tr("拟合范围无效"),tr("必须满足最小 Y 小于最大 Y。"));return;
    }
    if(std::floor((options.maxSensorY-options.minSensorY)/options.stepSensorY)>40){
        QMessageBox::warning(this,tr("候选数量过多"),tr("请增大步长，最多支持 41 个候选参数。"));return;
    }
    m_m4Fit.reset();m_m4Discard=false;
    m_m4Cancel=std::make_shared<std::atomic<bool>>(false);
    m_m4Progress=std::make_shared<std::atomic<std::size_t>>(0);
    const auto scene=m_session.scene;
    const auto replay=m_replay;
    const auto model=m_virtual->opticalModel;
    const auto cancel=m_m4Cancel;
    const auto progress=m_m4Progress;
    m_m4RunButton->setEnabled(false);m_m4FitButton->setEnabled(false);
    m_m4CancelButton->setEnabled(true);m_m4FitExportButton->setEnabled(false);m_m4ApplyButton->setEnabled(false);
    m_m4Info->setPlainText(tr("M4 正在拟合等效相机 Y，逐候选模拟 H1 四沉孔附近的真实曝光位姿。\n"
                               "仅搜索单参数，保留部分帧用于独立验证；不会触发真实扫描。"));
    m_m4ProgressTimer->start();
    m_m4FitWatcher->setFuture(QtConcurrent::run([scene,replay,model,options,cancel,progress]{
        try{
            return mls::twin::TwinM4Adapter::fitEquivalentCameraY(*replay,scene,model,options,
                                                                    cancel.get(),progress.get());
        }catch(const std::exception& e){
            auto failed=std::make_shared<mls::twin::TwinM4FitData>();
            failed->message=QStringLiteral("M4 拟合异常：")+QString::fromLocal8Bit(e.what());return failed;
        }catch(...){
            auto failed=std::make_shared<mls::twin::TwinM4FitData>();
            failed->message=QStringLiteral("M4 未知拟合异常");return failed;
        }
    }));
}
void DigitalTwinScanWidget::onM4FitFinished(){
    if(m_m4ProgressTimer)m_m4ProgressTimer->stop();
    m_m4CancelButton->setEnabled(false);
    const auto fit=m_m4FitWatcher->result();
    if(m_m4Discard){m_m4Discard=false;return;}
    m_m4RunButton->setEnabled(bool(m_virtual));m_m4FitButton->setEnabled(bool(m_virtual));
    if(!fit||!fit->ok()){
        m_m4Info->setPlainText(tr("M4 拟合未完成：%1").arg(fit?fit->message:tr("未知错误")));
        return;
    }
    m_m4Fit=fit;m_m4FitExportButton->setEnabled(true);
    m_m4ApplyButton->setEnabled(fit->result.holdoutSupportsBest);
    QString report=tr("M4 等效光学参数拟合完成：%1 帧（训练 %2，留出验证 %3）。\n"
                      "训练集最优等效相机 Y=%4mm。\n")
        .arg(static_cast<qulonglong>(fit->result.selectedFrames))
        .arg(static_cast<qulonglong>(fit->result.trainingFrames))
        .arg(static_cast<qulonglong>(fit->result.validationFrames))
        .arg(fit->result.bestBaselineY,0,'f',1);
    report+=tr("相机 Y(mm) | 训练得分 | 验证得分 | 验证孔区域 F1\n");
    for(const auto& c:fit->result.candidates){
        report+=QStringLiteral("%1 | %2 | %3 | %4\n")
            .arg(c.cameraSensorY,0,'f',1)
            .arg(c.trainingScore,0,'f',4)
            .arg(c.validationScore,0,'f',4)
            .arg(c.validation.holeRegions.f1(),0,'f',4);
    }
    report+=tr("留出验证最优 Y=%1mm；是否支持训练最优：%2。\n")
        .arg(fit->result.bestHoldoutBaselineY,0,'f',1)
        .arg(fit->result.holdoutSupportsBest?tr("基本一致"):tr("不支持或验证帧不足，禁止自动应用"));
    report+=QString::fromStdString(fit->result.validationNote)+"\n";
    report+=tr("\n此结果只是在 H1 单次固定姿态数据下的近似模型拟合；"
               "没有识别真实相机/激光光心。可手动应用最优 Y，再运行 M3 和 M4 全量对比。"
               "应用操作会清除旧 M3/M4 结果以免混用。");
    m_m4Info->setPlainText(report);
}
void DigitalTwinScanWidget::updateM4Progress(){
    const bool running=(m_m4CompareWatcher&&m_m4CompareWatcher->isRunning())||
                       (m_m4FitWatcher&&m_m4FitWatcher->isRunning());
    if(!running||m_m4Discard||!m_m4Info)return;
    const std::size_t count=m_m4Progress?m_m4Progress->load():0;
    if(m_m4CompareWatcher->isRunning())
        m_m4Info->setPlainText(tr("M4 全量对比中：%1 / %2 个 Profile；按原始索引统计。")
            .arg(static_cast<qulonglong>(count))
            .arg(m_replay?m_replay->scan.profiles.size():0));
    else m_m4Info->setPlainText(tr("M4 相机等效偏置搜索中：已模拟 %1 个候选 Profile；可点击取消。")
        .arg(static_cast<qulonglong>(count)));
}
void DigitalTwinScanWidget::exportM4Report(){
    if(!m_m4Comparison||!m_m4Comparison->ok())return;
    const QString file=QFileDialog::getSaveFileName(this,tr("导出 M4 对比报告"),
        QFileInfo(m_replayFilePath).absolutePath()+"/twin_m4_compare.json",tr("JSON 报告 (*.json)"));
    if(file.isEmpty())return;
    const QString error=mls::twin::TwinM4Adapter::exportComparison(*m_m4Comparison,file);
    if(error.isEmpty())QMessageBox::information(this,tr("M4 导出完成"),tr("已导出 JSON 与逐帧 CSV。"));
    else QMessageBox::warning(this,tr("M4 导出失败"),error);
}
void DigitalTwinScanWidget::exportM4Fit(){
    if(!m_m4Fit||!m_m4Fit->ok())return;
    const QString file=QFileDialog::getSaveFileName(this,tr("导出 M4 等效参数搜索报告"),
        QFileInfo(m_replayFilePath).absolutePath()+"/twin_m4_fit.json",tr("JSON 报告 (*.json)"));
    if(file.isEmpty())return;
    const QString error=mls::twin::TwinM4Adapter::exportFit(*m_m4Fit,file);
    if(error.isEmpty())QMessageBox::information(this,tr("M4 导出完成"),tr("已导出参数搜索 JSON 与候选评分 CSV。"));
    else QMessageBox::warning(this,tr("M4 导出失败"),error);
}
void DigitalTwinScanWidget::applyM4Fit(){
    if(!m_m4Fit||!m_m4Fit->ok())return;
    const double selected=m_m4Fit->result.bestBaselineY;
    if(QMessageBox::question(this,tr("仅应用等效偏置"),
        tr("将 M3 等效相机 Y 改为 %1 mm？\n旧虚拟扫描和 M4 分析会清空，需再次运行 M3。"
           "这不是厂家/多姿态真实光学校准。")
        .arg(selected,0,'f',1),QMessageBox::Yes|QMessageBox::No,QMessageBox::No)!=QMessageBox::Yes)return;
    m_cameraBaselineY->setValue(selected);
    clearVirtual();
    m_virtualInfo->setText(tr("已应用 M4 搜索得到的等效相机 Y=%1 mm。请重新运行 M3。")
        .arg(selected,0,'f',1));
}

// V28.4 M5: no hardware access; analysis keeps an immutable snapshot of M2,
// optional M3, CAD tessellation and estimated placement until it finishes.
void DigitalTwinScanWidget::startM5(){
    if(m_m8Running && (m_m8Index>=m_m8Sequence.size() ||
        m_m8Sequence[m_m8Index]!=mls::twin::M8Stage::CoverageM5))return;
    if(!m_session.scene.hasWorkpiece()||!m_replay||!m_replay->ok()){
        QMessageBox::information(this,tr("缺少 M5 输入"),tr("请先加载 STEP/OBJ 和 M2 实测 .scan。M3 虚拟点云为可选输入。"));
        return;
    }
    if(m_m5Watcher->isRunning()||
       (m_m52Watcher&&m_m52Watcher->isRunning())||
       (m_m51Watcher&&m_m51Watcher->isRunning())||
       (m_virtualWatcher&&m_virtualWatcher->isRunning())||
       (m_m4CompareWatcher&&m_m4CompareWatcher->isRunning())||
       (m_m4FitWatcher&&m_m4FitWatcher->isRunning())){
        QMessageBox::information(this,tr("后台任务未完成"),tr("请先等待或取消当前数字孪生后台任务。"));
        return;
    }
    mls::twin::M5Options options;
    options.cellPitchMm=m_m5Pitch->value();
    options.maxSurfaceDistanceMm=m_m5MatchDistance->value();
    options.minHitsPerCell=static_cast<std::uint32_t>(m_m5MinHits->value());
    options.excludeBottom=m_m5ExcludeBottom->isChecked();
    options.maxCells=250000;
    clearM5();
    m_m5Discard=false;
    m_m5Cancel=std::make_shared<std::atomic<bool>>(false);
    m_m5Progress=std::make_shared<std::atomic<std::size_t>>(0);
    const auto cancel=m_m5Cancel;
    const auto progress=m_m5Progress;
    const auto scene=m_session.scene;
    const auto real=m_replay;
    const auto virtualData=m_virtual;
    m_m5RunButton->setEnabled(false);m_m5CancelButton->setEnabled(true);
    m_m5ExportButton->setEnabled(false);
    m_m5Info->setPlainText(tr("M5 正在使用 CAD 估计位姿关联实测点到三角化表面；未返回的实测采样原因未知。"));
    m_m5ProgressTimer->start();
    m_m5Watcher->setFuture(QtConcurrent::run([scene,real,virtualData,options,cancel,progress]{
        try{
            return mls::twin::TwinM5Adapter::analyze(*real,virtualData.get(),scene,options,
                cancel.get(),progress.get());
        }catch(const std::exception& e){
            auto failed=std::make_shared<mls::twin::TwinM5Data>();
            failed->message=QStringLiteral("M5 后台异常：")+QString::fromLocal8Bit(e.what());
            return failed;
        }catch(...){
            auto failed=std::make_shared<mls::twin::TwinM5Data>();
            failed->message=QStringLiteral("M5 未知后台异常");return failed;
        }
    }));
}
void DigitalTwinScanWidget::onM5Finished(){
    if(m_m5ProgressTimer)m_m5ProgressTimer->stop();
    m_m5CancelButton->setEnabled(false);
    m_m5RunButton->setEnabled(true);
    const auto data=m_m5Watcher->result();
    if(m_m5Discard){m_m5Discard=false;onM8StageFinished(mls::twin::M8Stage::CoverageM5,false,tr("旧结果已清除"));return;}
    if(!data||!data->ok()){
        m_m5Info->setPlainText(tr("M5 未完成：%1").arg(data?data->message:tr("未知错误")));
        onM8StageFinished(mls::twin::M8Stage::CoverageM5,false,tr("运行失败或取消"));return;
    }
    m_m5Data=data;
    m_m5ExportButton->setEnabled(true);
    const auto& r=data->report;
    if(m_view)m_view->setTwinCoveragePoints(data->realObservedPreview,
        data->realMissingPreview,data->virtualOnlyPreview);
    if(m_mainView)m_mainView->setTwinCoveragePoints(data->realObservedPreview,
        data->realMissingPreview,data->virtualOnlyPreview);
    QString summary=tr("M5 CAD 表面覆盖分析完成\n"
      "目标表面面积：%1 mm²；覆盖单元：%2；有效单元间距约 %3 mm\n"
      "M2 实测覆盖：%4%%，M3 虚拟覆盖：%5\n"
      "实测未覆盖：%6 mm²；仅实测覆盖：%7 mm²；仅虚拟覆盖：%8 mm²\n"
      "实测点关联/未匹配/排除：%9 / %10 / %11\n")
        .arg(r.targetAreaMm2,0,'f',2)
        .arg(static_cast<qulonglong>(r.cells.size()))
        .arg(r.effectiveCellPitchMm,0,'f',3)
        .arg(r.realCoverage()*100,0,'f',2)
        .arg(data->containsVirtualProfiles?QString::number(r.virtualCoverage()*100,'f',2)+"%":tr("未输入 M3"))
        .arg(r.targetAreaMm2-r.realAreaMm2,0,'f',2)
        .arg(r.realOnlyAreaMm2,0,'f',2)
        .arg(r.virtualOnlyAreaMm2,0,'f',2)
        .arg(static_cast<qulonglong>(r.realAccepted))
        .arg(static_cast<qulonglong>(r.realNoAssociation))
        .arg(static_cast<qulonglong>(r.realExcluded));
    summary+=tr("\nCAD 表面分组（实测覆盖 / 虚拟覆盖）：\n");
    for(const auto& f:r.features){
        if(!f.target||f.totalAreaMm2<=1e-9)continue;
        summary+=QStringLiteral("%1 : %2% / %3%（%4 mm²）\n")
            .arg(QString::fromStdString(f.name))
            .arg(f.realCoverage()*100,0,'f',1)
            .arg(data->containsVirtualProfiles?QString::number(f.virtualCoverage()*100,'f',1):tr("N/A"))
            .arg(f.totalAreaMm2,0,'f',1);
    }
    summary+=tr("\n注意：区域和法向由 STEP 三角化代理产生；"
        "M2 没有返回的点不等于相机遮挡。面积覆盖不是精度/公差报告，"
        "底面默认排除。H1 孔特征分组为已知零件模板；可导出逐单元 CSV。" );
    m_m5Info->setPlainText(summary);
    refreshTwinInspector();
    onM8StageFinished(mls::twin::M8Stage::CoverageM5,true);
}
void DigitalTwinScanWidget::cancelM5(){
    if(m_m5Cancel)m_m5Cancel->store(true);
    if(m_m5Info&&m_m5Watcher&&m_m5Watcher->isRunning())
        m_m5Info->setPlainText(tr("已请求取消 M5；后台任务将在下一批 Profile 前停止。"));
}
void DigitalTwinScanWidget::clearM5(){
    clearM6();
    clearM51();
    if(m_m5Cancel)m_m5Cancel->store(true);
    m_m5Discard=true;
    if(m_m5ProgressTimer)m_m5ProgressTimer->stop();
    m_m5Data.reset();
    if(m_view)m_view->clearTwinCoveragePoints();
    if(m_mainView)m_mainView->clearTwinCoveragePoints();
    for(auto* view:{m_view,m_mainView.data()})if(view)view->setTwinStageMarkers(QStringLiteral("M5_FOCUS"),{});
    if(m_twinFeatureTable)m_twinFeatureTable->setRowCount(0);
    if(m_m5ExportButton)m_m5ExportButton->setEnabled(false);
    if(m_m5RunButton)m_m5RunButton->setEnabled(!m_m5Watcher||!m_m5Watcher->isRunning());
    if(m_m5CancelButton)m_m5CancelButton->setEnabled(m_m5Watcher&&m_m5Watcher->isRunning());
    if(m_m5Info)m_m5Info->setPlainText(tr("M5 热图与分析结果已清除；保留 STEP、M2 和 M3 数据。"));
}
void DigitalTwinScanWidget::exportM5(){
    if(!m_m5Data||!m_m5Data->ok()||m_m5ExportWatcher->isRunning())return;
    const auto path=QFileDialog::getSaveFileName(this,tr("导出 M5 CAD 表面覆盖"),
        QFileInfo(m_replayFilePath).absolutePath()+"/twin_m5_coverage.json",tr("JSON 报告 (*.json)"));
    if(path.isEmpty())return;
    m_m5ExportButton->setEnabled(false);
    // Snapshot ownership: clearing M5 in the GUI does not dereference freed
    // data while the QtConcurrent worker writes a full-resolution CSV.
    const auto data=m_m5Data;
    m_m5ExportWatcher->setFuture(QtConcurrent::run([data,path]{
        try{return mls::twin::TwinM5Adapter::exportReport(*data,path);}
        catch(const std::exception& e){return QStringLiteral("M5 导出异常：")+QString::fromLocal8Bit(e.what());}
        catch(...){return QStringLiteral("M5 未知导出异常");}
    }));
}
void DigitalTwinScanWidget::updateM5Progress(){
    if(!m_m5Watcher||!m_m5Watcher->isRunning()||m_m5Discard||!m_m5Info)return;
    m_m5Info->setPlainText(tr("M5 表面覆盖计算中：%1 / %2 条 Profile；"
        "正在将实测点与 CAD 三角网格最近表面关联，可取消。")
        .arg(static_cast<qulonglong>(m_m5Progress?m_m5Progress->load():0))
        .arg(m_replay?m_replay->scan.profiles.size():0));
}

// V28.5 / M5.1: high-resolution H1 center-hole density diagnostic.
// Only existing M2/M3 returns are ingested. No inferred points or controller IO.
void DigitalTwinScanWidget::startM51(){
    if(m_m8Running && (m_m8Index>=m_m8Sequence.size() ||
        m_m8Sequence[m_m8Index]!=mls::twin::M8Stage::ChamferM51))return;
    if(!m_session.scene.hasWorkpiece()||!m_replay||!m_replay->ok()){
        QMessageBox::information(this,tr("M5.1 缺少输入"),tr("请先导入 H1 STEP 与 M2 实测 SCAN；M3 虚拟数据可选。"));return;
    }
    if(m_m51Watcher->isRunning()||
       (m_m52Watcher&&m_m52Watcher->isRunning())||
       (m_m5Watcher&&m_m5Watcher->isRunning())||
       (m_virtualWatcher&&m_virtualWatcher->isRunning())||
       (m_m4CompareWatcher&&m_m4CompareWatcher->isRunning())||
       (m_m4FitWatcher&&m_m4FitWatcher->isRunning())){
        QMessageBox::information(this,tr("后台任务未完成"),tr("请先等待或取消正在运行的扫描分析任务。"));return;
    }
    mls::twin::M51Options options;
    options.primaryThreshold=m_m51Threshold->currentIndex();
    options.sparseRatio=m_m51SparseRatio->value();
    clearM51();
    m_m51Discard=false;
    m_m51Cancel=std::make_shared<std::atomic<bool>>(false);
    m_m51Progress=std::make_shared<std::atomic<std::size_t>>(0);
    const auto cancel=m_m51Cancel;
    const auto progress=m_m51Progress;
    const auto scene=m_session.scene;
    const auto real=m_replay;
    const auto virtualData=m_virtual;
    m_m51RunButton->setEnabled(false);
    m_m51CancelButton->setEnabled(true);
    m_m51ExportButton->setEnabled(false);
    m_m51Info->setPlainText(tr("M5.1 处理中：使用真实 Profile + 实际曝光位姿匹配中心孔名义圆锥面。"));
    m_m51ProgressTimer->start();
    m_m51Watcher->setFuture(QtConcurrent::run([scene,real,virtualData,options,cancel,progress]{
        try{
            return mls::twin::TwinM51Adapter::analyze(*real,virtualData.get(),scene,options,
                cancel.get(),progress.get());
        }catch(const std::exception& e){
            auto failed=std::make_shared<mls::twin::TwinM51Data>();
            failed->message=QStringLiteral("M5.1 运行异常：")+QString::fromLocal8Bit(e.what());return failed;
        }catch(...){
            auto failed=std::make_shared<mls::twin::TwinM51Data>();
            failed->message=QStringLiteral("M5.1 未知异常");return failed;
        }
    }));
}
void DigitalTwinScanWidget::onM51Finished(){
    if(m_m51ProgressTimer)m_m51ProgressTimer->stop();
    m_m51CancelButton->setEnabled(false);
    m_m51RunButton->setEnabled(true);
    const auto data=m_m51Watcher->result();
    if(m_m51Discard){m_m51Discard=false;onM8StageFinished(mls::twin::M8Stage::ChamferM51,false,tr("旧结果已清除"));return;}
    if(!data||!data->ok()){
        m_m51Info->setPlainText(tr("M5.1 未完成：%1").arg(data?data->message:tr("未知异常")));onM8StageFinished(mls::twin::M8Stage::ChamferM51,false,tr("运行失败或取消"));return;
    }
    m_m51Data=data;
    m_m51ExportButton->setEnabled(true);
    const auto& r=data->report;
    const int threshold=r.options.primaryThreshold;
    QString summary=tr("M5.1 H1 中心孔精细诊断完成；直接分析 M2 / M3 原始有效点。\n"
        "中心孔 CAD 位置：(0,-38)，孔口 Ø6.5，内孔 Ø5.5，上下倒角 0.5×45°。\n"
        "角度分辨率 5°，上倒角 4 径向层，直孔壁 20 深度层。\n"
        "主要关联距离：%1 mm；稀疏判定比例：%2。\n"
        "真实上倒角稀疏角度格：%3 / 72（粗密度诊断，不代表面积未覆盖率）。\n"
        "阈值敏感性（实测/模拟被分配至中心孔的命中数）：\n")
        .arg(r.options.thresholdsMm[std::size_t(threshold)],0,'f',2)
        .arg(r.options.sparseRatio,0,'f',2)
        .arg(r.realLowDensityAngles);
    for(int k=0;k<mls::twin::M51ThresholdCount;++k){
        summary+=QStringLiteral("  %1 mm : %2 / %3\n")
            .arg(r.options.thresholdsMm[std::size_t(k)],0,'f',2)
            .arg(static_cast<qulonglong>(r.realAssociated[std::size_t(k)]))
            .arg(data->report.hasVirtual?QString::number(r.virtualAssociated[std::size_t(k)]):tr("未导入 M3"));
    }
    summary+=tr("上倒角方程 r+深度−3.25 的探索性残差中位数：实测 %1 mm，虚拟 %2 mm；"
         "实测 MAD=%3 mm（包含配准与标定偏差，不能直接修改 CAD）。\n")
        .arg(r.realConeEquationMedianMm,0,'f',3)
        .arg(r.virtualConeEquationMedianMm,0,'f',3)
        .arg(r.realConeEquationMadMm,0,'f',3);
    summary+=tr("STEP CAD 面验证（网格面积 / 名义面积）：\n");
    for(const auto& feature:r.surfaces){
        QString faceIds;
        for(const auto id:feature.cadFaceIds){
            if(!faceIds.isEmpty())faceIds+=QStringLiteral(" / ");
            faceIds+=QString::number(id);
        }
        summary+=QStringLiteral("%1: %2 / %3 mm²，CAD face=%4\n")
            .arg(QString::fromStdString(feature.name))
            .arg(feature.meshAreaMm2,0,'f',2).arg(feature.nominalAreaMm2,0,'f',2).arg(faceIds);
    }
    summary+=tr("\n红色分箱代表局部点密度低，不应直接解释为相机遮挡；"
        "绿色/青色代表相对较密集。请导出 JSON + 分箱 CSV 检查 M2/M3 不一致的扇区。"
        "该分析不能取代尺寸检验或重新校准 STEP。" );
    m_m51Info->setPlainText(summary);
    refreshM51Heatmap();
    refreshTwinInspector();
    onM8StageFinished(mls::twin::M8Stage::ChamferM51,true);
}
void DigitalTwinScanWidget::cancelM51(){
    if(m_m51Cancel)m_m51Cancel->store(true);
    if(m_m51Info&&m_m51Watcher&&m_m51Watcher->isRunning())
        m_m51Info->setPlainText(tr("已请求取消 M5.1；下一批 Profile 前将停止。"));
}
void DigitalTwinScanWidget::clearM51(){
    for(auto* v:{m_view,m_mainView.data()})if(v)v->setTwinStageMarkers(QStringLiteral("M51_SPARSE"),{});
    if(m_m51Cancel)m_m51Cancel->store(true);
    m_m51Discard=true;
    if(m_m51ProgressTimer)m_m51ProgressTimer->stop();
    m_m51Data.reset();
    if(m_m51Heatmap)m_m51Heatmap->clearContents();
    if(m_m51ExportButton)m_m51ExportButton->setEnabled(false);
    if(m_m51RunButton)m_m51RunButton->setEnabled(!m_m51Watcher||!m_m51Watcher->isRunning());
    if(m_m51CancelButton)m_m51CancelButton->setEnabled(m_m51Watcher&&m_m51Watcher->isRunning());
    if(m_m51Info)m_m51Info->setPlainText(tr("M5.1 中心孔热图与分析结果已清除；STEP/M2/M3 未被删除。"));
}
void DigitalTwinScanWidget::exportM51(){
    if(!m_m51Data||!m_m51Data->ok()||m_m51ExportWatcher->isRunning())return;
    const QString path=QFileDialog::getSaveFileName(this,tr("导出 M5.1 中心孔报告"),
        QFileInfo(m_replayFilePath).absolutePath()+"/twin_m51_center_chamfer.json",
        tr("JSON 报告 (*.json)"));
    if(path.isEmpty())return;
    m_m51ExportButton->setEnabled(false);
    const auto data=m_m51Data;
    m_m51ExportWatcher->setFuture(QtConcurrent::run([data,path]{
        try{return mls::twin::TwinM51Adapter::exportReport(*data,path);}
        catch(const std::exception& e){return QStringLiteral("M5.1 导出异常：")+QString::fromLocal8Bit(e.what());}
        catch(...){return QStringLiteral("M5.1 未知导出异常");}
    }));
}
void DigitalTwinScanWidget::updateM51Progress(){
    if(!m_m51Watcher||!m_m51Watcher->isRunning()||m_m51Discard||!m_m51Info)return;
    m_m51Info->setPlainText(tr("M5.1 分析中：%1 / %2 Profile。使用中心孔圆锥/圆柱几何计算角度与深度密度，支持取消。")
       .arg(static_cast<qulonglong>(m_m51Progress?m_m51Progress->load():0))
       .arg(m_replay?m_replay->scan.profiles.size():0));
}
void DigitalTwinScanWidget::refreshM51Heatmap(){
    if(!m_m51Heatmap||!m_m51Data||!m_m51Data->ok())return;
    const auto& r=m_m51Data->report;
    const auto index=r.options.primaryThreshold;
    const int display=m_m51ViewMode?m_m51ViewMode->currentIndex():0;
    for(int layer=0;layer<4;++layer){
        for(int a=0;a<72;++a){
            const auto& b=r.bins[std::size_t(layer*72+a)];
            const double real=b.realRelativeDensity,virt=b.virtualRelativeDensity;
            double value=display==0?real:display==1?virt:std::max(0.,virt-real);
            value=std::clamp(value,0.,1.);
            const int intensity=int(150.*std::sqrt(value));
            QColor color;
            if(display==2) color=QColor(240,225-intensity,120-intensity/2);
            else if(value<r.options.sparseRatio)color=QColor(248,130+intensity/3,115+intensity/4);
            else color=QColor(110-intensity/4,175+intensity/3,190+intensity/3);
            auto* item=new QTableWidgetItem(QString::number(
                display==0?b.realHits[index]:display==1?b.virtualHits[index]:
                static_cast<int>(b.virtualHits[index])-static_cast<int>(b.realHits[index])));
            item->setTextAlignment(Qt::AlignCenter);
            item->setBackground(color);
            item->setToolTip(tr("上倒角: %1°～%2°，径向层 %3/4\n实测=%4，虚拟=%5\n相对密度=%6 / %7\n实测稀疏=%8")
                .arg(a*5).arg((a+1)*5).arg(layer+1)
                .arg(b.realHits[index]).arg(b.virtualHits[index])
                .arg(real,0,'f',3).arg(virt,0,'f',3)
                .arg(b.realSparse?tr("是"):tr("否")));
            m_m51Heatmap->setItem(layer,a,item);
        }
    }
}

// M5.2 offline geometry consistency; independent of M5.1 coarse coverage.
void DigitalTwinScanWidget::startM52(){
    if(m_m8Running && (m_m8Index>=m_m8Sequence.size() ||
        m_m8Sequence[m_m8Index]!=mls::twin::M8Stage::GeometryM52))return;
    if(!m_session.scene.hasWorkpiece()||!m_replay||!m_replay->ok()){
        QMessageBox::information(this,tr("M5.2 输入不足"),tr("请先导入 H1 STEP 与实测 SCAN；M3 可选。"));return;
    }
    if(m_m52Watcher->isRunning()||
       (m_m51Watcher&&m_m51Watcher->isRunning())||
       (m_m5Watcher&&m_m5Watcher->isRunning())||
       (m_virtualWatcher&&m_virtualWatcher->isRunning())||
       (m_m4CompareWatcher&&m_m4CompareWatcher->isRunning())||
       (m_m4FitWatcher&&m_m4FitWatcher->isRunning())){
        QMessageBox::information(this,tr("后台任务尚未结束"),tr("请先等待或取消其他分析任务。"));return;
    }
    clearM52();
    m_m52Discard=false;
    m_m52Cancel=std::make_shared<std::atomic<bool>>(false);
    m_m52Progress=std::make_shared<std::atomic<std::size_t>>(0);
    // MSVC: the shared_ptr types differ and cannot share one `auto` declaration.
    const auto cancel = m_m52Cancel;
    const auto progress = m_m52Progress;
    const auto scene = m_session.scene;
    const auto real = m_replay;
    const auto virt = m_virtual;
    m_m52RunButton->setEnabled(false);m_m52CancelButton->setEnabled(true);
    m_m52ExportButton->setEnabled(false);
    m_m52Info->setPlainText(tr("M5.2 正在计算稳定顶面与中心孔倒角独立拟合，所有结果只读。"));
    m_m52ProgressTimer->start();
    m_m52Watcher->setFuture(QtConcurrent::run([scene,real,virt,cancel,progress]{
        try{return mls::twin::TwinM52Adapter::analyze(*real,virt.get(),scene,{},cancel.get(),progress.get());}
        catch(const std::exception& e){
            auto failed=std::make_shared<mls::twin::TwinM52Data>();
            failed->message=QStringLiteral("M5.2 异常：")+QString::fromLocal8Bit(e.what());return failed;
        }catch(...){
            auto failed=std::make_shared<mls::twin::TwinM52Data>();
            failed->message=QStringLiteral("M5.2 未知异常");return failed;
        }
    }));
}
void DigitalTwinScanWidget::onM52Finished(){
    if(m_m52ProgressTimer)m_m52ProgressTimer->stop();
    m_m52RunButton->setEnabled(true);m_m52CancelButton->setEnabled(false);
    const auto data=m_m52Watcher->result();
    if(m_m52Discard){m_m52Discard=false;onM8StageFinished(mls::twin::M8Stage::GeometryM52,false,tr("旧结果已清除"));return;}
    if(!data||!data->ok()){
        m_m52Info->setPlainText(tr("M5.2 失败：%1").arg(data?data->message:tr("未知错误")));onM8StageFinished(mls::twin::M8Stage::GeometryM52,false,tr("运行失败或取消"));return;
    }
    m_m52Data=data;m_m52ExportButton->setEnabled(true);
    const auto& r=data->report.real;
    QString message=tr("M5.2 完成（仅几何一致性诊断，绝不自动改变工件位姿或名义尺寸）：\n"
        "参与顶面估计的稳定采样格=%1，顶面拟合 MAD=%2 mm；顶面偏差在中心孔处=%3 mm。\n"
        "顶面斜率 X=%4，Y=%5（mm/mm）。\n"
        "倒角参与点=%6；覆盖角度格=%7/72；拟合训练=%8、验证=%9 格。\n"
        "修正前倒角方程残差中位数=%10 mm；顶面修正后=%11 mm。\n"
        "候选孔中心 XY 偏移=(%12,%13) mm；消除 XY 变化项后的截面残差中位数=%14 mm，MAD=%15 mm。\n"
        "角度留出验证 MAD=%16 mm；剩余常数偏差=%17 mm。\n")
        .arg(static_cast<qulonglong>(r.topCellsUsed))
        .arg(r.topPlaneMadMm,0,'f',4)
        .arg(r.topOffsetAtHoleMm,0,'f',4)
        .arg(r.topSlopeX,0,'f',6).arg(r.topSlopeY,0,'f',6)
        .arg(static_cast<qulonglong>(r.conePointsUsed))
        .arg(static_cast<qulonglong>(r.angularBinsUsed))
        .arg(static_cast<qulonglong>(r.trainBins))
        .arg(static_cast<qulonglong>(r.validationBins))
        .arg(r.coneRawMedianMm,0,'f',4).arg(r.coneAfterTopMedianMm,0,'f',4)
        .arg(r.candidateCenterShiftXmm,0,'f',4).arg(r.candidateCenterShiftYmm,0,'f',4)
        .arg(r.coneAfterXYMedianMm,0,'f',4).arg(r.coneAfterXYMadMm,0,'f',4)
        .arg(r.coneFitValidationMadMm,0,'f',4).arg(r.coneInterceptMm,0,'f',4);
    if(data->report.hasVirtual){
        const auto& v=data->report.virtualFit;
        message+=tr("虚拟模型：%1；顶面偏差=%2 mm；倒角常数残差=%3 mm。\n")
            .arg(v.ok?tr("分析成功"):QString::fromStdString(v.error))
            .arg(v.topOffsetAtHoleMm,0,'f',4).arg(v.coneInterceptMm,0,'f',4);
    }
    message+=tr("注意：单一光学姿态不能唯一分离 CAD 加工偏差、标定误差和配准偏差。"
        "这里的偏移仅是候选诊断量；不提供自动应用按钮。请查看 JSON 和逐角度 CSV。");
    m_m52Info->setPlainText(message);
    refreshM52Heatmap();
    refreshTwinInspector();
    onM8StageFinished(mls::twin::M8Stage::GeometryM52,true);
}
void DigitalTwinScanWidget::cancelM52(){
    if(m_m52Cancel)m_m52Cancel->store(true);
    if(m_m52Watcher&&m_m52Watcher->isRunning()&&m_m52Info)
        m_m52Info->setPlainText(tr("已请求取消 M5.2；后台将停止。"));
}
void DigitalTwinScanWidget::clearM52(){
    for(auto* v:{m_view,m_mainView.data()})if(v)v->setTwinStageMarkers(QStringLiteral("M52_RESIDUAL"),{});
    if(m_m52Cancel)m_m52Cancel->store(true);
    m_m52Discard=true;
    if(m_m52ProgressTimer)m_m52ProgressTimer->stop();
    m_m52Data.reset();
    if(m_m52Heatmap)m_m52Heatmap->clearContents();
    if(m_m52RunButton)m_m52RunButton->setEnabled(!m_m52Watcher||!m_m52Watcher->isRunning());
    if(m_m52CancelButton)m_m52CancelButton->setEnabled(m_m52Watcher&&m_m52Watcher->isRunning());
    if(m_m52ExportButton)m_m52ExportButton->setEnabled(false);
    if(m_m52Info)m_m52Info->setPlainText(tr("M5.2 诊断已清除，STEP、M2 和 M3 数据保留。"));
}
void DigitalTwinScanWidget::exportM52(){
    if(!m_m52Data||!m_m52Data->ok()||m_m52ExportWatcher->isRunning())return;
    const auto path=QFileDialog::getSaveFileName(this,tr("导出 M5.2 几何一致性报告"),
        QFileInfo(m_replayFilePath).absolutePath()+"/twin_m52_geometry_consistency.json",
        tr("JSON 报告 (*.json)"));
    if(path.isEmpty())return;
    m_m52ExportButton->setEnabled(false);
    const auto data=m_m52Data;
    m_m52ExportWatcher->setFuture(QtConcurrent::run([data,path]{
        try{return mls::twin::TwinM52Adapter::exportReport(*data,path);}
        catch(const std::exception& e){return QStringLiteral("M5.2 导出异常：")+QString::fromLocal8Bit(e.what());}
        catch(...){return QStringLiteral("M5.2 导出未知异常");}
    }));
}
void DigitalTwinScanWidget::updateM52Progress(){
    if(!m_m52Watcher||!m_m52Watcher->isRunning()||m_m52Discard||!m_m52Info)return;
    m_m52Info->setPlainText(tr("M5.2 读取并分区分析 Profile：%1 / %2，可取消。")
        .arg(static_cast<qulonglong>(m_m52Progress?m_m52Progress->load():0))
        .arg(m_replay?m_replay->scan.profiles.size():0));
}
void DigitalTwinScanWidget::refreshM52Heatmap(){
    if(!m_m52Heatmap||!m_m52Data||!m_m52Data->ok())return;
    const auto& rr=m_m52Data->report;
    const int mode=m_m52ViewMode?m_m52ViewMode->currentIndex():0;
    for(int a=0;a<mls::twin::M52AngularBins;++a){
        const auto& r=rr.real.angular[std::size_t(a)];
        const auto& v=rr.virtualFit.angular[std::size_t(a)];
        double value=mode==0?r.medianRawMm:
                     mode==1?r.medianTopCorrectedMm:
                     mode==2?r.medianAfterXYMm:
                     mode==3?r.medianAfterFullFitMm:v.medianRawMm;
        const bool observed=mode==4?(rr.hasVirtual&&v.hits>=3):r.hits>=3;
        QColor color=QColor(180,180,180);
        if(observed){
            const double ratio=std::clamp(std::abs(value)/.35,0.,1.);
            const int shade=int(150*ratio);
            color=value>=0?QColor(220,210-shade,110-shade/2):QColor(105,210-shade,215);
        }
        auto* item=new QTableWidgetItem(observed?QString::number(value,'f',3):tr("—"));
        item->setTextAlignment(Qt::AlignCenter);item->setBackground(color);
        item->setToolTip(tr("CAD 角度 %1°～%2°；实测=%3 点，虚拟=%4 点；训练角度=%5；"
            "修正前=%6 mm，顶面修正后=%7 mm，XY 候选修正后=%8 mm")
            .arg(a*5).arg((a+1)*5).arg(r.hits).arg(v.hits)
            .arg(r.training?tr("是"):tr("否"))
            .arg(r.medianRawMm,0,'f',4).arg(r.medianTopCorrectedMm,0,'f',4)
            .arg(r.medianAfterXYMm,0,'f',4));
        m_m52Heatmap->setItem(0,a,item);
    }
}

// V28.7 M6: continuous XYZCB scanning with retained true M2/M5 input.
void DigitalTwinScanWidget::startM6(){
    if(m_m8Running && (m_m8Index>=m_m8Sequence.size() ||
        m_m8Sequence[m_m8Index]!=mls::twin::M8Stage::ContinuousM6))return;
    if(!m_replay||!m_replay->ok()||!m_session.scene.hasWorkpiece()||!m_m5Data||!m_m5Data->ok()){
        QMessageBox::information(this,tr("缺少 M6 输入"),tr("请先导入 STEP 和 M2 SCAN，并完成 M5 CAD 覆盖分析。"));return;
    }
    if(m_m6Watcher->isRunning()||m_m6ExportWatcher->isRunning()||
       (m_m5Watcher&&m_m5Watcher->isRunning())||
       (m_m52Watcher&&m_m52Watcher->isRunning())||
       (m_virtualWatcher&&m_virtualWatcher->isRunning())){
        QMessageBox::information(this,tr("后台任务运行中"),tr("请等待正在运行的数字孪生任务完成。"));return;
    }
    clearM6();
    mls::twin::M6Options opt;
    opt.scanRateHz=m_m6Hz->value();
    opt.tiltAmplitudeDeg=m_m6Tilt->value();
    opt.yawAmplitudeDeg=m_m6Yaw->value();
    opt.maxSelectedSegments=static_cast<unsigned int>(m_m6Segments->value());
    opt.exportPointStride=static_cast<unsigned int>(m_m6ExportStride->value());
    opt.alsoExportPly=m_m6AlsoPly->isChecked();
    // Full 200-Hz multi-feature scans may take many minutes and generate a
    // large float32 point spool. Hard limits bound memory/disk usage.
    opt.maxFullProfiles=180000;
    opt.maxTotalDurationSec=1500.;
    opt.candidateEvaluationFrames=36;
    opt.candidateCoarseFrames=8;
    opt.maxRefinedCandidates=40;
    opt.maxCandidateSearchSeconds=150.;
    if(!m_replay->scan.profiles.isEmpty()){
        opt.hasInitialJoints=true;
        opt.initialJoints=m_replay->scan.profiles[0].fiveAxis.frame.joints;
        opt.returnToStartAfterScan=true;
    }
    if(m_m51Data && m_m51Data->ok()){
        // Re-open M5.1's under-sampled chamfer bins for NBV, instead of
        // treating M5's coarse 2-hit cell as automatically complete.
        for(const auto& bin:m_m51Data->report.bins){
            if(bin.surface!=mls::twin::M51Surface::TopChamfer)continue;
            if(bin.angle>=72||bin.layer>=4)continue;
            if(bin.realRelativeDensity<0.65){
                const std::size_t idx=std::size_t(bin.layer)*72+std::size_t(bin.angle);
                opt.prioritizeSparseChamfer[idx]=1;
            }
        }
    }
    mls::twin::Sensor2950Geometry optics;
    optics.cameraOriginSensor.y=m_cameraBaselineY->value();
    const auto scene=m_session.scene;
    const auto real=m_replay;
    const auto coverage=m_m5Data;
    const auto config1=QDir(QCoreApplication::applicationDirPath()).filePath("models/machine5x/machine_config.json");
    const auto config2=QDir::current().filePath("models/machine5x/machine_config.json");
    const QString modelPath=(m_m6SkipFcl&&m_m6SkipFcl->isChecked())
        ?QString():(QFileInfo::exists(config1)?config1:(QFileInfo::exists(config2)?config2:QString()));
    m_m6Cancel=std::make_shared<std::atomic<bool>>(false);
    m_m6Progress=std::make_shared<std::atomic<std::size_t>>(0);
    m_m6DetailedProgress=std::make_shared<mls::twin::M6ProgressState>();
    m_m6Elapsed.start();
    const auto cancel=m_m6Cancel;
    const auto progress=m_m6Progress;
    const auto detail=m_m6DetailedProgress;
    m_m6Discard=false;
    m_m6RunButton->setEnabled(false);
    m_m6CancelButton->setEnabled(true);
    m_m6ExportButton->setEnabled(false);
    if(m_m6ProgressBar){m_m6ProgressBar->setRange(0,0);m_m6ProgressBar->setFormat(tr("正在初始化 M6...") );}
    if(m_m6SegmentsProgressBar){m_m6SegmentsProgressBar->setRange(0,100);m_m6SegmentsProgressBar->setValue(0);}
    if(m_m6CurrentProgressBar){m_m6CurrentProgressBar->setRange(0,100);m_m6CurrentProgressBar->setValue(0);}
    if(m_m6ArchiveProgressBar){m_m6ArchiveProgressBar->setRange(0,100);m_m6ArchiveProgressBar->setValue(0);m_m6ArchiveProgressBar->setFormat(tr("待生成 SCAN 分卷"));}
    m_m6Info->setPlainText(tr("M6 正在搜索连续五轴候选视角；每条轨迹用 M3 双光路模拟 Profile，按新增 CAD 面积评分。无设备指令。"));
    m_m6ProgressTimer->start();
    m_m6Watcher->setFuture(QtConcurrent::run([scene,real,coverage,optics,opt,modelPath,cancel,progress,detail]{
        try{return mls::twin::TwinM6Adapter::run(scene,coverage->report,*real,
                     optics,opt,modelPath,cancel.get(),progress.get(),detail.get());}
        catch(const std::exception& e){
            auto fail=std::make_shared<mls::twin::TwinM6Data>();
            fail->status=QStringLiteral("M6 后台异常：")+QString::fromLocal8Bit(e.what());return fail;
        }catch(...){
            auto fail=std::make_shared<mls::twin::TwinM6Data>();
            fail->status=QStringLiteral("M6 未知异常");return fail;
        }
    }));
}
void DigitalTwinScanWidget::onM6Finished(){
    if(m_m6ProgressTimer)m_m6ProgressTimer->stop();
    if(m_m6RunButton)m_m6RunButton->setEnabled(true);
    if(m_m6CancelButton)m_m6CancelButton->setEnabled(false);
    const auto result=m_m6Watcher->result();
    if(m_m6Discard){m_m6Discard=false;onM8StageFinished(mls::twin::M8Stage::ContinuousM6,false,tr("旧结果已清除"));return;}
    if(!result||!result->ok()){
        if(m_m6ProgressBar){m_m6ProgressBar->setRange(0,100);m_m6ProgressBar->setValue(0);m_m6ProgressBar->setFormat(tr("M6 未完成 / 已取消"));}
        for(QProgressBar* bar:{m_m6SegmentsProgressBar,m_m6CurrentProgressBar,m_m6ArchiveProgressBar}){
            if(bar){bar->setRange(0,100);bar->setValue(0);bar->setFormat(tr("运行失败或取消"));}
        }
        m_m6Info->setPlainText(result?result->status:tr("M6 没有返回有效结果"));onM8StageFinished(mls::twin::M8Stage::ContinuousM6,false,tr("运行失败或取消"));return;
    }
    if(m_m6ProgressBar){m_m6ProgressBar->setRange(0,100);m_m6ProgressBar->setValue(100);m_m6ProgressBar->setFormat(tr("候选搜索完成"));}
    if(m_m6SegmentsProgressBar){m_m6SegmentsProgressBar->setRange(0,100);m_m6SegmentsProgressBar->setValue(100);m_m6SegmentsProgressBar->setFormat(tr("完成 %1 个实际扫描段").arg(static_cast<int>(result->report.segments.size())));}
    if(m_m6CurrentProgressBar){m_m6CurrentProgressBar->setRange(0,100);m_m6CurrentProgressBar->setValue(100);m_m6CurrentProgressBar->setFormat(tr("最后一段采集完成"));}
    if(m_m6ArchiveProgressBar){m_m6ArchiveProgressBar->setRange(0,100);m_m6ArchiveProgressBar->setValue(100);m_m6ArchiveProgressBar->setFormat(tr("已无损暂存 %1 卷 SCAN").arg(static_cast<int>(result->scanVolumes.size())));}
    m_m6Data=result;
    for(auto* v:{m_view,m_mainView.data()})if(v)
        v->setTwinM6DetailedResults(result->segmentPointPreviews,
            result->segmentLaserPaths,result->transferLaserPath);
    refreshTwinInspector();
    // Make the simulated M6 point cloud unmistakable on completion; user can
    // restore any earlier stage without deleting its data.
    setTwinViewPreset(2);
    m_m6ExportButton->setEnabled(true);
    const auto& r=result->report;
    QString text=tr("M6 连续 XYZCB 扫描完成（仅离线）\n段数=%1；模拟曝光 Profile=%2；有效虚拟点=%3\n"
        "M5 已覆盖面积=%4 mm²；新增=%5 mm²；累计=%6 / %7 mm²\n"
        "完整 Physical17 回放验证=%8；机床 FCL 加载=%9；工件夹具/外壳碰撞认证=否\n")
        .arg(static_cast<int>(r.segments.size())).arg(static_cast<qulonglong>(r.fullProfiles))
        .arg(static_cast<qulonglong>(r.fullValidPoints))
        .arg(r.initialObservedAreaMm2,0,'f',1).arg(r.newAreaMm2,0,'f',1)
        .arg(r.finalObservedAreaMm2,0,'f',1).arg(r.targetAreaMm2,0,'f',1)
        .arg(result->fullPhysical17Validated?tr("是"):tr("否 / 名义回退"))
        .arg(result->machineFclLoaded?tr("是"):tr("否 / 未验证"));
    text+=tr("\n新增面积依据虚拟双光路有效 Profile，至少 %1 次有效命中/单元；未保证所有 CAD 面可测。")
        .arg(r.options.minNewHitsPerCell);
    text+=tr("\nM5.1 强化补扫优先区域=%1 mm²（如已运行 M5.1）。")
        .arg(r.reopenedSparseChamferAreaMm2,0,'f',2);
    for(const auto& seg:r.segments){
        text+=tr("\n[方位%1] %2 | 时长%3 s | 预计新增%4 mm² | 仿真实际新增%5 mm²")
            .arg(seg.phaseId).arg(QString::fromStdString(seg.feature))
            .arg(seg.durationSec,0,'f',2)
            .arg(seg.predictedNewAreaMm2,0,'f',2)
            .arg(seg.measuredNewAreaMm2,0,'f',2);
    }
    text+=tr("\nM6 候选粗筛 %1 条，通过 %2 条，精评通过 %3 条；候选光学采样 %4 帧。")
        .arg(static_cast<qulonglong>(r.candidateCurvesAttempted))
        .arg(static_cast<qulonglong>(r.candidateCurvesFeasible))
        .arg(static_cast<qulonglong>(r.candidateCurvesRefined))
        .arg(static_cast<qulonglong>(r.candidateProfilesSimulated));
    if(r.candidateSearchTruncated)
        text+=tr("\n注意：候选评估达到时间/数量预算，仅使用已通过精评的部分轨迹；不是穷尽搜索。");
    text+=tr("\n无损 SCAN 已暂存 %1 卷，%2 条 Profile、%3 个有效返回采样；覆盖率使用全部 %4 个工件点。")
        .arg(static_cast<int>(result->scanVolumes.size()))
        .arg(static_cast<qulonglong>(r.fullProfiles))
        .arg(static_cast<qulonglong>(result->archivedValidPoints))
        .arg(static_cast<qulonglong>(r.fullValidPoints));
    if(r.options.alsoExportPly)text+=tr("\n额外 PLY 抽样：每 %1 个点保存 1 点，计 %2 点。")
        .arg(r.options.exportPointStride).arg(static_cast<qulonglong>(r.exportedValidPoints));
    text+=tr("\nM3 光学模型仍未经多姿态标定；原 H1 仅 X 移动，不能独立验证 B/C 旋转精度。所有导出轨迹标记为 NOT HARDWARE AUTHORIZED。");
    m_m6Info->setPlainText(text);
    onM8StageFinished(mls::twin::M8Stage::ContinuousM6,true);
}
void DigitalTwinScanWidget::cancelM6(){
    if(m_m6ExportCancel)m_m6ExportCancel->store(true);
    if(m_m6Cancel)m_m6Cancel->store(true);
    if(m_m6ProgressBar&&m_m6Watcher&&m_m6Watcher->isRunning())m_m6ProgressBar->setFormat(tr("正在取消..."));
    if(m_m6Info&&m_m6Watcher&&m_m6Watcher->isRunning())
        m_m6Info->setPlainText(tr("M6 正在取消，请等待正在计算的虚拟 Profile 结束。"));
}
void DigitalTwinScanWidget::clearM6(){
    clearM7();
    if(m_m6ExportCancel)m_m6ExportCancel->store(true);
    if(m_m6Cancel)m_m6Cancel->store(true);
    m_m6Discard=true;
    if(m_m6ProgressTimer)m_m6ProgressTimer->stop();
    m_m6Data.reset();
    if(m_view)m_view->clearTwinM6Results();
    if(m_mainView)m_mainView->clearTwinM6Results();
    if(m_twinSegmentPicker){QSignalBlocker b(m_twinSegmentPicker);m_twinSegmentPicker->clear();
        m_twinSegmentPicker->addItem(tr("全部扫描段"),-1);}
    if(m_twinPoseSlider){m_twinPoseSlider->setRange(0,0);m_twinPoseSlider->setValue(0);}
    if(m_m6RunButton)m_m6RunButton->setEnabled(!m_m6Watcher||!m_m6Watcher->isRunning());
    if(m_m6CancelButton)m_m6CancelButton->setEnabled(m_m6Watcher&&m_m6Watcher->isRunning());
    if(m_m6ExportButton)m_m6ExportButton->setEnabled(false);
    if(m_m6ProgressBar){m_m6ProgressBar->setRange(0,100);m_m6ProgressBar->setValue(0);m_m6ProgressBar->setFormat(tr("等待 M6 任务"));}
    for(QProgressBar* bar:{m_m6SegmentsProgressBar,m_m6CurrentProgressBar,m_m6ArchiveProgressBar}){
        if(bar){bar->setRange(0,100);bar->setValue(0);bar->setFormat(tr("等待任务"));}
    }
    if(m_m6Info)m_m6Info->setPlainText(tr("M6 轨迹及模拟点云已清除，不影响 STEP、M2、M3、M5 及真实点云。"));
    if(m_m5Data&&m_m5Data->ok())refreshM5FeatureTable();
}
void DigitalTwinScanWidget::exportM6(){
    if(!m_m6Data||!m_m6Data->ok()||m_m6ExportWatcher->isRunning())return;
    const auto file=QFileDialog::getSaveFileName(this,tr("导出 M6 连续五轴仿真报告"),
        QFileInfo(m_replayFilePath).absolutePath()+"/twin_m6_continuous_5axis.json",
        tr("JSON 报告 (*.json)"));
    if(file.isEmpty())return;
    const auto result=m_m6Data;
    m_m6ExportCancel=std::make_shared<std::atomic<bool>>(false);
    m_m6ExportProgress=std::make_shared<mls::twin::M6ExportProgress>();
    const auto cancel=m_m6ExportCancel;
    const auto progress=m_m6ExportProgress;
    m_m6ExportButton->setEnabled(false);
    if(m_m6CancelButton)m_m6CancelButton->setEnabled(true);
    if(m_m6ArchiveProgressBar){m_m6ArchiveProgressBar->setRange(0,0);m_m6ArchiveProgressBar->setFormat(tr("复制压缩 SCAN 分卷...") );}
    m_m6ProgressTimer->start();
    m_m6ExportWatcher->setFuture(QtConcurrent::run([result,file,cancel,progress]{
        try{return mls::twin::TwinM6Adapter::exportReport(*result,file,cancel.get(),progress.get());}
        catch(const std::exception& e){return QStringLiteral("M6 导出异常：")+QString::fromLocal8Bit(e.what());}
        catch(...){return QStringLiteral("M6 导出未知异常");}
    }));
}
void DigitalTwinScanWidget::updateM6Progress(){
    if(m_m6ExportWatcher && m_m6ExportWatcher->isRunning()){
        if(m_m6ArchiveProgressBar && m_m6ExportProgress){
            const auto& p=*m_m6ExportProgress;
            const auto total=p.bytesTotal.load();
            const auto done=p.bytesDone.load();
            if(total){
                m_m6ArchiveProgressBar->setRange(0,100);
                m_m6ArchiveProgressBar->setValue(static_cast<int>(std::min<std::uint64_t>(100,100*done/total)));
            }else m_m6ArchiveProgressBar->setRange(0,0);
            m_m6ArchiveProgressBar->setFormat(tr("导出压缩 SCAN：%1/%2 卷及附属文件，已复制 %3/%4 MiB")
                .arg(static_cast<qulonglong>(p.filesDone.load()))
                .arg(static_cast<qulonglong>(p.filesTotal.load()))
                .arg(static_cast<qulonglong>(done/(1024*1024)))
                .arg(static_cast<qulonglong>(total/(1024*1024))));
        }
        return;
    }
    if(!m_m6Watcher||!m_m6Watcher->isRunning()||m_m6Discard||!m_m6Info)return;
    if(!m_m6DetailedProgress){
        m_m6Info->setPlainText(tr("M6 后台任务初始化中..."));return;
    }
    const auto& p=*m_m6DetailedProgress;
    const auto stage=static_cast<mls::twin::M6ProgressStage>(p.stage.load());
    QString stageText;
    switch(stage){
    case mls::twin::M6ProgressStage::Starting: stageText=tr("初始化运动学和场景");break;
    case mls::twin::M6ProgressStage::Geometry: stageText=tr("生成 CAD 缺失特征路径");break;
    case mls::twin::M6ProgressStage::CandidateCoarse: stageText=tr("候选轨迹快速筛选（少量光学 Profile）");break;
    case mls::twin::M6ProgressStage::CandidateRefine: stageText=tr("优选轨迹精评 + 完整运动/FCL 验证");break;
    case mls::twin::M6ProgressStage::FullSimulation: stageText=tr("完整频率连续五轴虚拟扫描");break;
    case mls::twin::M6ProgressStage::Cancelled: stageText=tr("取消中");break;
    case mls::twin::M6ProgressStage::Finished: stageText=tr("完成");break;
    case mls::twin::M6ProgressStage::Failed: stageText=tr("失败");break;
    default: stageText=tr("未知阶段");break;
    }
    // Percent denotes progress WITHIN THE CURRENT STAGE/SCAN SEGMENT, not
    // a fabricated percentage of the entire adaptive multi-segment search.
    if(m_m6ProgressBar){
        auto setPhaseProgress=[this](std::uint64_t done,std::uint64_t total,const QString& name){
            if(total==0){
                m_m6ProgressBar->setRange(0,0);
                m_m6ProgressBar->setFormat(name+tr("：计算中..."));
                return;
            }
            const int value=static_cast<int>(std::min<std::uint64_t>(100,done*100/total));
            m_m6ProgressBar->setRange(0,100);
            m_m6ProgressBar->setValue(value);
            m_m6ProgressBar->setFormat(name+tr("：%p% (本阶段)，已用时 %1 秒")
                .arg(static_cast<qint64>(m_m6Elapsed.isValid()?m_m6Elapsed.elapsed()/1000:0)));
        };
        if(stage==mls::twin::M6ProgressStage::CandidateCoarse){
            setPhaseProgress(p.candidatesDone.load(),p.candidatesTotal.load(),tr("候选粗筛"));
        }else if(stage==mls::twin::M6ProgressStage::CandidateRefine){
            setPhaseProgress(p.refinementsDone.load(),p.refinementsTotal.load(),tr("候选精评"));
        }else if(stage==mls::twin::M6ProgressStage::FullSimulation){
            setPhaseProgress(p.activeSegmentProfilesDone.load(),
                             p.activeSegmentProfilesTotal.load(),tr("当前连续扫描段"));
        }else{
            m_m6ProgressBar->setRange(0,0);
            m_m6ProgressBar->setFormat(stageText);
        }
    }
    if(m_m6SegmentsProgressBar){
        const auto completed=p.segmentsCompleted.load();
        const auto maxCount=static_cast<std::size_t>(m_m6Segments->value());
        m_m6SegmentsProgressBar->setRange(0,100);
        m_m6SegmentsProgressBar->setValue(static_cast<int>(std::min<std::size_t>(100,completed*100/std::max<std::size_t>(1,maxCount))));
        m_m6SegmentsProgressBar->setFormat(tr("已完成扫描段 %1 / 上限 %2（自适应可提前结束）")
            .arg(static_cast<qulonglong>(completed)).arg(static_cast<qulonglong>(maxCount)));
    }
    if(m_m6CurrentProgressBar){
        const auto total=p.activeSegmentProfilesTotal.load();
        const auto done=p.activeSegmentProfilesDone.load();
        if(stage==mls::twin::M6ProgressStage::FullSimulation && total){
            m_m6CurrentProgressBar->setRange(0,100);
            m_m6CurrentProgressBar->setValue(static_cast<int>(std::min<std::uint64_t>(100,done*100/total)));
            m_m6CurrentProgressBar->setFormat(tr("扫描段 %1：Profile %2/%3 (%p%)")
                .arg(static_cast<qulonglong>(p.activeSegmentNumber.load()))
                .arg(static_cast<qulonglong>(done)).arg(static_cast<qulonglong>(total)));
        }else{
            m_m6CurrentProgressBar->setRange(0,100);
            m_m6CurrentProgressBar->setValue(0);
            m_m6CurrentProgressBar->setFormat(tr("当前扫描段：等待候选完成"));
        }
    }
    if(m_m6ArchiveProgressBar){
        m_m6ArchiveProgressBar->setRange(
            stage==mls::twin::M6ProgressStage::FullSimulation?0:0,
            stage==mls::twin::M6ProgressStage::FullSimulation?0:100);
        if(stage!=mls::twin::M6ProgressStage::FullSimulation)m_m6ArchiveProgressBar->setValue(0);
        m_m6ArchiveProgressBar->setFormat(tr("无损暂存：%1 条 Profile，%2 卷压缩 SCAN")
            .arg(static_cast<qulonglong>(p.archiveProfilesSaved.load()))
            .arg(static_cast<qulonglong>(p.archiveVolumesSaved.load())));
    }
    m_m6Info->setPlainText(tr("M6 当前阶段：%1\n"
        "CAD 特征组=%2；粗筛候选=%3/%4；粗筛通过=%5\n"
        "精评候选=%6/%7；候选预扫描 Profile=%8\n"
        "正式生成 Profile=%9；已完成扫描段=%10\n"
        "这两类 Profile 不能混为一谈；候选阶段显示 0 条正式 Profile 是正常的。可取消。")
        .arg(stageText)
        .arg(static_cast<qulonglong>(p.featuresTotal.load()))
        .arg(static_cast<qulonglong>(p.candidatesDone.load()))
        .arg(static_cast<qulonglong>(p.candidatesTotal.load()))
        .arg(static_cast<qulonglong>(p.candidatesFeasible.load()))
        .arg(static_cast<qulonglong>(p.refinementsDone.load()))
        .arg(static_cast<qulonglong>(p.refinementsTotal.load()))
        .arg(static_cast<qulonglong>(p.candidateProfiles.load()))
        .arg(static_cast<qulonglong>(p.fullProfiles.load()))
        .arg(static_cast<qulonglong>(p.segmentsCompleted.load())));
}

// V28.7.4 detailed digital twin visibility, CAD rescan diagnostics and M6
// segment/pose inspection.  All controls are read-only VTK overlays.
void DigitalTwinScanWidget::buildTwinInspector(QVBoxLayout* vbox,QWidget* panel){
    auto* group=new QGroupBox(tr("13. M0～M6 图层与复扫检视（显示/隐藏不会删除数据）"),panel);
    auto* layout=new QVBoxLayout(group);
    auto* explanation=new QLabel(tr("CAD 未覆盖区域是待评价的补扫目标，并非已证实的相机遮挡。"
        "绿色=M6 仿真新增 CAD 单元，红色=M6 后仍未达到命中门限；"
        "它们是 CAD 诊断标记，不是扫描回波。"),group);
    explanation->setWordWrap(true);layout->addWidget(explanation);
    struct OverlayItem {const char* key;const char* title;};
    const OverlayItem overlays[]={
        {"M0_MACHINE","M0 机床视觉模型"},
        {"M1_CAD","M1 STEP/OBJ 工件实体"},
        {"M1_SUPPORT","M1 支承平面"},
        {"M2_REAL","M2 原始采集点云"},
        {"M3_VIRTUAL","M3 虚拟光学点云"},
        {"M4_ROI","M4 四沉孔对比 ROI (CAD 诊断)"},
        {"M5_OBSERVED","M5 实测已覆盖 (青色)"},
        {"M5_MISSING","M5 实测未覆盖 (红色)"},
        {"M5_VIRTUAL_ONLY","M5 仅虚拟覆盖 (黄色)"},
        {"M5_FOCUS","M5 当前选中特征 (黄色高亮)"},
        {"M51_SPARSE","M5.1 上倒角稀疏分箱 (橙红)"},
        {"M52_RESIDUAL","M5.2 孔口截面偏差位置 (紫色)"},
        {"M6_RAW","M6 各段真实模拟回波预览 (多色)"},
        {"M6_NEW_COVERAGE","M6 补扫新增 CAD 单元 (绿色)"},
        {"M6_REMAINING","M6 补扫后仍缺失 CAD 单元 (红色)"},
        {"M6_PATH","M6 各段激光测量参考轨迹 (多色)"},
        {"M6_TRANSFER","M6 激光关闭时的转场路径 (灰色)"},
        {"M6_ENDPOINTS","M6 扫描起点/终点 (绿/红)"},
        {"M6_CURSOR","M6 当前时间位姿 (亮蓝色)"},
        {"M7_FUSED","M7 融合后的去重预览（白色，实测优先）"},
        {"M7_REAL","M7 融合预览·实测 M2 (青绿)"},
        {"M7_SIMULATED","M7 融合预览·虚拟 M6 (紫)"},
        {"M7_OVERLAP","M7 M2/M6 体素重叠 (黄色诊断)"},
        {"M7_NEW_CAD","M7 补扫新增 CAD 区域 (绿色标记)"},
        {"M7_REMAINING_CAD","M7 补扫后仍缺失 CAD 区域 (红色标记)"}
    };
    auto* grid=new QGridLayout;
    for(std::size_t i=0;i<sizeof(overlays)/sizeof(overlays[0]);++i){
        const auto& item=overlays[i];
        const QString key=QString::fromLatin1(item.key);
        auto* cb=new QCheckBox(QString::fromUtf8(item.title),group);
        const bool checked=key!=QStringLiteral("M6_TRANSFER")&&
            key!=QStringLiteral("M6_REMAINING")&&key!=QStringLiteral("M5_OBSERVED") &&
            key!=QStringLiteral("M5_VIRTUAL_ONLY")&&
            key!=QStringLiteral("M7_REAL")&&key!=QStringLiteral("M7_SIMULATED")&&
            key!=QStringLiteral("M7_OVERLAP");
        cb->setChecked(checked);
        m_twinLayerChecks.insert(key,cb);
        grid->addWidget(cb,int(i/2),int(i%2));
        connect(cb,&QCheckBox::toggled,this,[this,key](bool visible){
            setTwinLayerVisibility(key,visible);
        });
    }
    layout->addLayout(grid);
    auto* presets=new QGridLayout;
    const QStringList names{tr("只看 M2 初扫"),tr("只看 M5 待补扫"),
        tr("只看 M6 点云/路径"),tr("M2 + M6 对比"),tr("恢复全部图层"),tr("只看 M7 融合/验收")};
    for(int i=0;i<names.size();++i){
        auto* button=new QPushButton(names[i],group);
        presets->addWidget(button,i/2,i%2);
        connect(button,&QPushButton::clicked,this,[this,i]{setTwinViewPreset(i);});
    }
    layout->addLayout(presets);
    auto* detail=new QLabel(tr("M5 按特征待补扫面积（mm²）：点击下表行，在三维工件中高亮当前特征。"),group);
    detail->setWordWrap(true);layout->addWidget(detail);
    m_twinFeatureTable=new QTableWidget(0,6,group);
    m_twinFeatureTable->setHorizontalHeaderLabels({tr("CAD 特征"),tr("初扫缺失 mm²"),
        tr("M5.1 重开 mm²"),tr("M6 新覆盖 mm²"),tr("扫描后尚缺 mm²"),tr("初扫覆盖率")});
    m_twinFeatureTable->setMinimumHeight(205);
    m_twinFeatureTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_twinFeatureTable->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_twinFeatureTable->setSelectionMode(QAbstractItemView::SingleSelection);
    m_twinFeatureTable->horizontalHeader()->setStretchLastSection(true);
    layout->addWidget(m_twinFeatureTable);
    connect(m_twinFeatureTable,&QTableWidget::cellClicked,
        this,[this](int row,int){inspectM5Feature(row);});
    m_twinFeatureSummary=new QLabel(tr("等待 M5 计算后显示每个特征的待补扫区域。"),group);
    m_twinFeatureSummary->setWordWrap(true);layout->addWidget(m_twinFeatureSummary);
    m_twinExportTargetsButton=new QPushButton(tr("导出 M5/M6 待补扫 CAD 单元 CSV（非扫描点）"),group);
    m_twinExportTargetsButton->setEnabled(false);
    layout->addWidget(m_twinExportTargetsButton);
    connect(m_twinExportTargetsButton,&QPushButton::clicked,
        this,&DigitalTwinScanWidget::exportM5RescanTargets);
    m_twinTargetsExportWatcher=new QFutureWatcher<QString>(this);
    connect(m_twinTargetsExportWatcher,&QFutureWatcher<QString>::finished,this,[this]{
        m_twinExportTargetsButton->setEnabled(bool(m_m5Data));
        const QString error=m_twinTargetsExportWatcher->result();
        if(error.isEmpty())QMessageBox::information(this,tr("CAD 补扫目标"),tr("待补扫单元 CSV 导出成功。"));
        else QMessageBox::warning(this,tr("CAD 目标导出失败"),error);
    });
    auto* segmentRow=new QHBoxLayout;
    segmentRow->addWidget(new QLabel(tr("M6 显示扫描段"),group));
    m_twinSegmentPicker=new QComboBox(group);
    m_twinSegmentPicker->addItem(tr("全部扫描段"),-1);
    segmentRow->addWidget(m_twinSegmentPicker,1);
    layout->addLayout(segmentRow);
    connect(m_twinSegmentPicker,QOverload<int>::of(&QComboBox::currentIndexChanged),
        this,&DigitalTwinScanWidget::selectM6Segment);
    m_twinPoseSlider=new QSlider(Qt::Horizontal,group);
    m_twinPoseSlider->setRange(0,0);
    layout->addWidget(new QLabel(tr("M6 全五轴姿态时间轴（包含激光关闭转场）"),group));
    layout->addWidget(m_twinPoseSlider);
    m_twinPoseInfo=new QLabel(tr("等待 M6 连续路径；可在此查看每一时刻 XYZ/C/B、曝光状态和测量参考位置。"),group);
    m_twinPoseInfo->setWordWrap(true);m_twinPoseInfo->setTextInteractionFlags(Qt::TextSelectableByMouse);
    layout->addWidget(m_twinPoseInfo);
    connect(m_twinPoseSlider,&QSlider::valueChanged,this,&DigitalTwinScanWidget::seekM6Pose);
    vbox->insertWidget(0,group);
    for(auto it=m_twinLayerChecks.constBegin();it!=m_twinLayerChecks.constEnd();++it)
        setTwinLayerVisibility(it.key(),it.value()->isChecked());
}

void DigitalTwinScanWidget::setTwinLayerVisibility(const QString& key,bool visible){
    for(auto* view:{m_view,m_mainView.data()})if(view)view->setTwinOverlayVisible(key,visible);
}

void DigitalTwinScanWidget::setTwinViewPreset(int preset){
    QHash<QString,bool> batch;
    for(auto it=m_twinLayerChecks.begin();it!=m_twinLayerChecks.end();++it){
        const QString& id=it.key();
        bool value=true;
        if(preset==0)value=(id==QStringLiteral("M2_REAL")||id==QStringLiteral("M1_CAD"));
        else if(preset==1)value=(id==QStringLiteral("M1_CAD")||id==QStringLiteral("M5_MISSING")||
            id==QStringLiteral("M5_FOCUS")||id==QStringLiteral("M51_SPARSE"));
        else if(preset==2)value=(id==QStringLiteral("M6_RAW") ||
            id==QStringLiteral("M6_PATH") || id==QStringLiteral("M6_ENDPOINTS"));
        else if(preset==3)value=(id==QStringLiteral("M2_REAL")||id==QStringLiteral("M6_RAW")||
            id==QStringLiteral("M6_NEW_COVERAGE"));
        else if(preset==4)value=(id!=QStringLiteral("M6_TRANSFER"));
        else if(preset==5)value=(id==QStringLiteral("M7_FUSED")||
            id==QStringLiteral("M7_NEW_CAD")||
            id==QStringLiteral("M7_REMAINING_CAD"));
        {QSignalBlocker b(it.value());it.value()->setChecked(value);}
        batch.insert(id,value);
    }
    for(auto* view:{m_view,m_mainView.data()})if(view)
        view->setTwinOverlayVisibilityBatch(batch);
}

void DigitalTwinScanWidget::refreshM5FeatureTable(){
    if(!m_twinFeatureTable)return;
    QSignalBlocker blocker(m_twinFeatureTable);
    m_twinFeatureTable->setRowCount(0);
    if(m_twinExportTargetsButton)m_twinExportTargetsButton->setEnabled(bool(m_m5Data));
    if(!m_m5Data||!m_m5Data->ok())return;
    const auto& r=m_m5Data->report;
    const auto* map=m_m6Data&&m_m6Data->ok()?&m_m6Data->report.firstCoveredSegmentByCell:nullptr;
    std::vector<double> newArea(r.features.size(),0.);
    std::vector<double> baselineMissing(r.features.size(),0.);
    std::vector<double> reopenedArea(r.features.size(),0.);
    std::vector<double> remaining(r.features.size(),0.);
    const std::unordered_set<std::uint32_t> reopened=(m_m6Data&&m_m6Data->ok())
        ?std::unordered_set<std::uint32_t>(m_m6Data->report.reopenedSparseCellIds.begin(),
                                          m_m6Data->report.reopenedSparseCellIds.end())
        :std::unordered_set<std::uint32_t>{};
    for(std::size_t i=0;i<r.cells.size();++i){
        const auto& cell=r.cells[i];
        if(!cell.target||cell.featureId>=r.features.size())continue;
        const bool missing=cell.realHits<r.requestedOptions.minHitsPerCell;
        const bool reopenedM51=reopened.count(static_cast<std::uint32_t>(i))>0;
        if(!missing&&!reopenedM51)continue;
        const auto f=std::size_t(cell.featureId);
        if(missing)baselineMissing[f]+=cell.areaMm2;
        if(reopenedM51)reopenedArea[f]+=cell.areaMm2;
        if(map&&i<map->size()&&(*map)[i])newArea[f]+=cell.areaMm2;
        else remaining[f]+=cell.areaMm2;
    }
    for(const auto& feature:r.features){
        if(!feature.target||feature.totalAreaMm2<=0.)continue;
        const int row=m_twinFeatureTable->rowCount();
        m_twinFeatureTable->insertRow(row);
        const std::size_t id=feature.id;
        const QStringList values{QString::fromStdString(feature.name),
            QString::number(baselineMissing[id],'f',2),QString::number(reopenedArea[id],'f',2),
            QString::number(newArea[id],'f',2),
            QString::number(remaining[id],'f',2),QString::number(feature.realCoverage()*100.,'f',1)+"%"};
        for(int j=0;j<values.size();++j){
            auto* cell=new QTableWidgetItem(values[j]);
            cell->setData(Qt::UserRole,int(feature.id));
            m_twinFeatureTable->setItem(row,j,cell);
        }
    }
    m_twinFeatureTable->resizeColumnsToContents();
    if(m_twinFeatureSummary)
        m_twinFeatureSummary->setText(tr("M5 特征数=%1；单元数=%2；红色表示未达到有效命中门限。M6 绿色是新增 CAD 覆盖标记。")
            .arg(m_twinFeatureTable->rowCount()).arg(static_cast<qulonglong>(r.cells.size())));
}

void DigitalTwinScanWidget::inspectM5Feature(int row){
    if(!m_m5Data||!m_m5Data->ok()||!m_twinFeatureTable||row<0||row>=m_twinFeatureTable->rowCount())return;
    auto* item=m_twinFeatureTable->item(row,0);if(!item)return;
    const int feature=item->data(Qt::UserRole).toInt();
    QVector<QVector3D> highlight;
    const auto& report=m_m5Data->report;
    const auto transform=m_session.scene.cadToMachineEstimate();
    QVector3D lo(1e8f,1e8f,1e8f),hi(-1e8f,-1e8f,-1e8f);
    const std::unordered_set<std::uint32_t> reopened=(m_m6Data&&m_m6Data->ok())
        ?std::unordered_set<std::uint32_t>(m_m6Data->report.reopenedSparseCellIds.begin(),
                                          m_m6Data->report.reopenedSparseCellIds.end())
        :std::unordered_set<std::uint32_t>{};
    std::size_t count=0;
    for(std::size_t i=0;i<report.cells.size();++i){
        const auto& cell=report.cells[i];
        if(!cell.target||cell.featureId!=feature||
           (cell.realHits>=report.requestedOptions.minHitsPerCell &&
            reopened.count(static_cast<std::uint32_t>(i))==0))continue;
        const auto p=transform.point(cell.cadCenter+cell.cadNormal*0.12);
        const QVector3D q{float(p.x),float(p.y),float(p.z)};
        lo.setX(std::min(lo.x(),q.x()));lo.setY(std::min(lo.y(),q.y()));lo.setZ(std::min(lo.z(),q.z()));
        hi.setX(std::max(hi.x(),q.x()));hi.setY(std::max(hi.y(),q.y()));hi.setZ(std::max(hi.z(),q.z()));
        if(count%std::max<std::size_t>(1,report.cells.size()/30000)==0)highlight.append(q);
        ++count;
    }
    for(auto* v:{m_view,m_mainView.data()})if(v){
        v->setTwinStageMarkers(QStringLiteral("M5_FOCUS"),highlight);
        if(count && v==m_view)v->focusTwinRegion(lo,hi);
    }
    if(m_twinFeatureSummary)m_twinFeatureSummary->setText(
        tr("选中特征：%1，初扫未满足命中门限的 CAD 单元 %2 个（高亮显示）。"
            "这些目标是否真实可测，还需 M6 光学与运动学验证。")
            .arg(item->text()).arg(static_cast<qulonglong>(count)));
}

void DigitalTwinScanWidget::selectM6Segment(int comboIndex){
    if(!m_twinSegmentPicker||comboIndex<0)return;
    const int segment=m_twinSegmentPicker->itemData(comboIndex).toInt();
    for(auto* v:{m_view,m_mainView.data()})if(v)v->setTwinM6SegmentSelection(segment);
}

void DigitalTwinScanWidget::seekM6Pose(int position){
    if(!m_m6Data||!m_m6Data->ok()||!m_twinPoseInfo)return;
    const auto& path=m_m6Data->report.path;
    if(path.empty())return;
    const auto index=std::min<std::size_t>(path.size()-1,
        std::size_t(double(std::max(0,position))/1000.0*(path.size()-1)));
    const auto& p=path[index];
    const auto focus=p.sensorToMachine.point({0,0,m_m6Data->report.options.focusSensorZmm});
    m_twinPoseInfo->setText(tr("t=%1s；XYZ=(%2,%3,%4) mm；C=%5°, B=%6°；%7；"
        "激光名义参考点=(%8,%9,%10) mm。\n注意：记录为离线位姿，并非硬件轴反馈。")
        .arg(p.seconds,0,'f',3)
        .arg(p.joints[0],0,'f',3).arg(p.joints[1],0,'f',3).arg(p.joints[2],0,'f',3)
        .arg(p.joints[3],0,'f',3).arg(p.joints[4],0,'f',3)
        .arg(p.role==mls::twin::M6SegmentRole::Scan?tr("曝光扫描"):tr("关闭激光转场"))
        .arg(focus.x,0,'f',3).arg(focus.y,0,'f',3).arg(focus.z,0,'f',3));
    QVector<QVector3D> marker;
    marker.append(QVector3D{float(focus.x),float(focus.y),float(focus.z)});
    for(auto* v:{m_view,m_mainView.data()})if(v)v->setTwinStageMarkers(QStringLiteral("M6_CURSOR"),marker);
}

void DigitalTwinScanWidget::refreshTwinInspector(){
    // The inspector never modifies M5/M6 coverage masks; these are CAD markers
    // derived from real M2 hits and recorded full M6 virtual-profile results.
    refreshM5FeatureTable();
    for(auto* v:{m_view,m_mainView.data()})if(v){
        if(m_m5Data&&m_m5Data->ok()){
            QVector<QVector3D> newCovered,remaining;
            const auto& r=m_m5Data->report;
            const auto* first=m_m6Data&&m_m6Data->ok()?&m_m6Data->report.firstCoveredSegmentByCell:nullptr;
            const auto T=m_session.scene.cadToMachineEstimate();
            const std::unordered_set<std::uint32_t> reopened=(m_m6Data&&m_m6Data->ok())
                ?std::unordered_set<std::uint32_t>(m_m6Data->report.reopenedSparseCellIds.begin(),
                                                  m_m6Data->report.reopenedSparseCellIds.end())
                :std::unordered_set<std::uint32_t>{};
            for(std::size_t i=0;i<r.cells.size();++i){
                const auto& cell=r.cells[i];if(!cell.target)continue;
                const bool initial=cell.realHits>=r.requestedOptions.minHitsPerCell;
                const bool prioritized=reopened.count(static_cast<std::uint32_t>(i))>0;
                const bool added=first&&i<first->size()&&(*first)[i]>0;
                if(initial && !prioritized && !added)continue;
                if((i%3)!=0)continue; // bounded VTK marker preview, not area scoring
                const auto p=T.point(cell.cadCenter+cell.cadNormal*0.10);
                const QVector3D q{float(p.x),float(p.y),float(p.z)};
                if(added)newCovered.append(q);
                else remaining.append(q);
            }
            v->setTwinStageMarkers(QStringLiteral("M6_NEW_COVERAGE"),newCovered);
            v->setTwinStageMarkers(QStringLiteral("M6_REMAINING"),remaining);
        }else{
            v->setTwinStageMarkers(QStringLiteral("M6_NEW_COVERAGE"),{});
            v->setTwinStageMarkers(QStringLiteral("M6_REMAINING"),{});
        }
        if(m_m51Data&&m_m51Data->ok()){
            QVector<QVector3D> sparse;
            const auto& r=m_m51Data->report;
            const auto T=m_session.scene.cadToMachineEstimate();
            for(const auto& b:r.bins){
                if(b.surface!=mls::twin::M51Surface::TopChamfer||!b.realSparse)continue;
                const double theta=(double(b.angle)+.5)*5.*mls::twin::kPi/180.;
                const double radial=(double(b.layer)+.5)*.125;
                const double radius=3.25-radial;
                const auto q=T.point({r.xCenterCadMm+radius*std::cos(theta),
                    r.yCenterCadMm+radius*std::sin(theta),r.cadTopZMm-radial});
                sparse.push_back(QVector3D{float(q.x),float(q.y),float(q.z)});
            }
            v->setTwinStageMarkers(QStringLiteral("M51_SPARSE"),sparse);
        }else v->setTwinStageMarkers(QStringLiteral("M51_SPARSE"),{});
        if(m_m52Data&&m_m52Data->ok()){
            QVector<QVector3D> points;
            const auto& report=m_m52Data->report;
            const auto T=m_session.scene.cadToMachineEstimate();
            for(const auto& b:report.real.angular){
                if(b.hits<8 || std::abs(b.medianTopCorrectedMm)<.10)continue;
                const double a=(double(b.angleStartDeg)+2.5)*mls::twin::kPi/180.;
                const auto q=T.point({report.nominalHoleXmm+3.0*std::cos(a),
                    report.nominalHoleYmm+3.0*std::sin(a),report.nominalTopZmm-.25});
                points.push_back(QVector3D{float(q.x),float(q.y),float(q.z)});
            }
            v->setTwinStageMarkers(QStringLiteral("M52_RESIDUAL"),points);
        }else v->setTwinStageMarkers(QStringLiteral("M52_RESIDUAL"),{});
        if(m_m4Comparison&&m_m4Comparison->ok()&&m_session.scene.hasWorkpiece()){
            QVector<QVector3D> circles;
            const auto ctx=mls::twin::h1ReferenceM4Context(m_session.scene,
                mls::twin::Sensor2950Geometry{});
            for(const auto& h:ctx.holes){
                for(int k=0;k<72;++k){
                    const double a=double(k)*5.*mls::twin::kPi/180.;
                    const auto q=ctx.cadToMachine.point({h.x+h.radiusMm*std::cos(a),
                        h.y+h.radiusMm*std::sin(a),ctx.cadTopZ+.12});
                    circles.push_back(QVector3D{float(q.x),float(q.y),float(q.z)});
                }
            }
            v->setTwinStageMarkers(QStringLiteral("M4_ROI"),circles);
        }else v->setTwinStageMarkers(QStringLiteral("M4_ROI"),{});
    }
    if(m_twinSegmentPicker){
        QSignalBlocker block(m_twinSegmentPicker);
        m_twinSegmentPicker->clear();m_twinSegmentPicker->addItem(tr("全部扫描段"),-1);
        if(m_m6Data&&m_m6Data->ok()){
            for(std::size_t i=0;i<m_m6Data->report.segments.size();++i){
                const auto& s=m_m6Data->report.segments[i];
                m_twinSegmentPicker->addItem(tr("段 %1 / %2 / %3 s")
                    .arg(int(i+1)).arg(QString::fromStdString(s.feature))
                    .arg(s.durationSec,0,'f',1),int(i));
            }
        }
    }
    if(m_twinPoseSlider){
        {QSignalBlocker b(m_twinPoseSlider);
            m_twinPoseSlider->setRange(0,(m_m6Data&&m_m6Data->ok())?1000:0);
            m_twinPoseSlider->setValue(0);
        }
        if(m_m6Data&&m_m6Data->ok())seekM6Pose(0);
    }
}

void DigitalTwinScanWidget::exportM5RescanTargets(){
    if(!m_m5Data||!m_m5Data->ok()||!m_twinTargetsExportWatcher||m_twinTargetsExportWatcher->isRunning())return;
    const QString file=QFileDialog::getSaveFileName(this,tr("导出逐 CAD 单元待补扫诊断"),
        QFileInfo(m_replayFilePath).absolutePath()+"/twin_m5_m6_rescan_targets.csv",
        tr("CSV UTF-8 (*.csv)"));
    if(file.isEmpty())return;
    m_twinExportTargetsButton->setEnabled(false);
    const auto baseline=m_m5Data;
    const auto rescue=m_m6Data;
    const auto pose=m_session.scene.cadToMachineEstimate();
    m_twinTargetsExportWatcher->setFuture(QtConcurrent::run([baseline,rescue,pose,file]{
        QSaveFile out(file);
        if(!out.open(QIODevice::WriteOnly|QIODevice::Text))return out.errorString();
        QTextStream ts(&out);
        ts.setEncoding(QStringConverter::Utf8);
        ts<<"cellIndex,featureId,featureName,cadX,cadY,cadZ,machineX,machineY,machineZ,"
          "normalX,normalY,normalZ,areaMm2,realHits,virtualHits,initialMissing,"
          "m51SparseReopened,firstM6CoveringSegment,stillMissingAfterM6,category\n";
        const auto& r=baseline->report;
        const auto* covered=rescue&&rescue->ok()?&rescue->report.firstCoveredSegmentByCell:nullptr;
        const std::unordered_set<std::uint32_t> reopened=(rescue&&rescue->ok())
            ?std::unordered_set<std::uint32_t>(rescue->report.reopenedSparseCellIds.begin(),
                                              rescue->report.reopenedSparseCellIds.end())
            :std::unordered_set<std::uint32_t>{};
        for(std::size_t i=0;i<r.cells.size();++i){
            const auto& c=r.cells[i];if(!c.target)continue;
            const bool missing=c.realHits<r.requestedOptions.minHitsPerCell;
            const bool sparseAgain=reopened.count(static_cast<std::uint32_t>(i))>0;
            if(!missing&&!sparseAgain)continue;
            const int segment=covered && i<covered->size()?int((*covered)[i]):0;
            const auto world=pose.point(c.cadCenter);
            QString name=c.featureId<r.features.size()?QString::fromStdString(r.features[c.featureId].name):QString();
            name.replace('"',QStringLiteral("''"));
            ts<<qulonglong(i)<<','<<c.featureId<<",\""<<name<<"\","<<c.cadCenter.x<<','
              <<c.cadCenter.y<<','<<c.cadCenter.z<<','<<world.x<<','<<world.y<<','<<world.z<<','
              <<c.cadNormal.x<<','<<c.cadNormal.y<<','<<c.cadNormal.z<<','<<c.areaMm2<<','
              <<c.realHits<<','<<c.virtualHits<<','<<(missing?1:0)<<','<<(sparseAgain?1:0)
              <<','<<segment<<','<<(segment?0:1)<<','
              <<(segment?"SIMULATED_NEW_COVERAGE":"UNOBSERVED_CAUSE_UNKNOWN")<<'\n';
            if(ts.status()!=QTextStream::Ok)return QStringLiteral("写入 CAD 单元 CSV 失败");
        }
        ts.flush();
        if(ts.status()!=QTextStream::Ok || !out.commit())return QStringLiteral("导出提交失败：")+out.errorString();
        return QString();
    }));
}

// V28.8 / M7: fusion is an OFFLINE comparison of measured and simulated
// returns. It never writes real hardware state or modifies the M2 SCAN.
void DigitalTwinScanWidget::startM7(){
    if(m_m8Running && (m_m8Index>=m_m8Sequence.size() ||
        m_m8Sequence[m_m8Index]!=mls::twin::M8Stage::FusionM7))return;
    if(!m_m7Watcher||m_m7Watcher->isRunning())return;
    if(!m_replay||!m_replay->ok()||!m_m5Data||!m_m5Data->ok()||
       !m_m6Data||!m_m6Data->ok()||!m_session.scene.hasWorkpiece()){
        QMessageBox::information(this,tr("M7 缺少数据"),
            tr("先导入真实 M2 SCAN，运行 M5 覆盖分析和 M6 连续五轴仿真后，再进行 M7 融合。"));return;
    }
    clearM7();
    const auto scene=m_session.scene;
    const auto real = m_replay;
    const auto m5 = m_m5Data;
    const auto m6 = m_m6Data;
    mls::twin::M7VoxelOptions options;
    options.voxelPitchMm=m_m7VoxelPitch->value();
    options.maxRetainedVoxels=std::size_t(m_m7VoxelCap->value());
    options.sampledProfileModulo=1;
    options.sampledIndexModulo=1;
    const double goal=m_m7Goal->value()/100.;
    m_m7Cancel=std::make_shared<std::atomic<bool>>(false);
    m_m7Progress=std::make_shared<mls::twin::M7Progress>();
    const auto cancel = m_m7Cancel;
    const auto progress = m_m7Progress;
    m_m7Discard=false;
    m_m7RunButton->setEnabled(false);m_m7CancelButton->setEnabled(true);
    m_m7ProgressBar->setRange(0,100);m_m7ProgressBar->setValue(0);
    m_m7Info->setPlainText(tr("M7：检查 M5/M6 CAD 覆盖；整理 M2 与 M6 实际采样预览。"));
    m_m7ProgressTimer->start();
    m_m7Watcher->setFuture(QtConcurrent::run([scene,real,m5,m6,options,goal,cancel,progress]{
        try{return mls::twin::TwinM7Adapter::fuse(scene,*real,*m5,*m6,options,goal,
                                                   cancel.get(),progress.get());}
        catch(const std::exception& e){
            auto fail=std::make_shared<mls::twin::TwinM7Data>();
            fail->message=QStringLiteral("M7 后台异常：")+QString::fromLocal8Bit(e.what());return fail;
        }catch(...){
            auto fail=std::make_shared<mls::twin::TwinM7Data>();
            fail->message=QStringLiteral("M7 未知异常");return fail;
        }
    }));
}
void DigitalTwinScanWidget::onM7Finished(){
    if(m_m7ProgressTimer)m_m7ProgressTimer->stop();
    m_m7RunButton->setEnabled(true);m_m7CancelButton->setEnabled(false);
    const auto result=m_m7Watcher->result();
    if(m_m7Discard){m_m7Discard=false;onM8StageFinished(mls::twin::M8Stage::FusionM7,false,tr("旧结果已清除"));return;}
    if(!result||!result->ok()){
        m_m7ProgressBar->setRange(0,100);m_m7ProgressBar->setValue(0);
        m_m7ProgressBar->setFormat(tr("M7 未完成"));
        m_m7Info->setPlainText(result?result->message:tr("M7 无有效结果"));onM8StageFinished(mls::twin::M8Stage::FusionM7,false,tr("运行失败或取消"));return;
    }
    m_m7Data=result;
    m_m7ProgressBar->setRange(0,100);m_m7ProgressBar->setValue(100);
    m_m7ProgressBar->setFormat(tr("M7 融合与 CAD 验收完成"));
    for(auto* view:{m_view,m_mainView.data()})if(view)view->setTwinM7Results(
        result->realPoints,result->simulatedPoints,result->overlapPoints,
        result->newlyCoveredCad,result->stillMissingCad,result->fusedPreview);
    const auto& c=result->coverage;
    m_m7FeatureTable->setRowCount(int(c.features.size()));
    for(std::size_t i=0;i<c.features.size();++i){
        const auto& f=c.features[i];
        const QStringList cells{QString::fromStdString(f.name),
            QString::number(f.measuredAreaMm2,'f',2),
            QString::number(f.simulatedNewAreaMm2,'f',2),
            QString::number(f.stillMissingAreaMm2,'f',2)};
        for(int j=0;j<cells.size();++j)m_m7FeatureTable->setItem(int(i),j,
            new QTableWidgetItem(cells[j]));
    }
    m_m7SegmentTable->setRowCount(int(m_m6Data->report.segments.size()));
    for(std::size_t i=0;i<m_m6Data->report.segments.size();++i){
        const auto& s=m_m6Data->report.segments[i];
        const QStringList columns{QString::number(i+1),QString::fromStdString(s.feature),
            QString::number(s.predictedNewAreaMm2,'f',2),
            QString::number(result->coverage.addedAreaBySegmentMm2[i],'f',2)};
        for(int j=0;j<columns.size();++j)m_m7SegmentTable->setItem(int(i),j,
            new QTableWidgetItem(columns[j]));
    }
    const QString details=tr("M7 结果：仅离线数字孪生\n"
        "M2 实测 CAD 面积=%1 mm²；M6 新增虚拟覆盖=%2 mm²；累计预测=%3 / %4 mm² (%5%)。\n"
        "M6 后仍缺失=%6 mm²；M5.1 稀疏重开目标=%7 mm²。\n"
        "融合预览：真实 M2 体素=%8，虚拟 M6 体素=%9，重叠体素=%10；"
        "重叠位置平均差异=%11 mm（非尺寸精度指标）。\n"
        "覆盖目标是否达到：%12。\n"
        "注意：M6 光学模型未经 B/C 多姿态实测标定，模拟点与真实点不平均，CAD 标记绝不作为测量点。")
        .arg(c.measuredAreaMm2,0,'f',2)
        .arg(c.simulatedNewAreaMm2,0,'f',2)
        .arg(c.measuredAreaMm2+c.simulatedNewAreaMm2,0,'f',2)
        .arg(c.targetAreaMm2,0,'f',2)
        .arg(100.*c.finalFraction(),0,'f',2)
        .arg(c.remainingAreaMm2,0,'f',2)
        .arg(c.reopenedSparseAreaMm2,0,'f',2)
        .arg(static_cast<qulonglong>(result->previewM2Points))
        .arg(static_cast<qulonglong>(result->previewM6Points))
        .arg(static_cast<qulonglong>(result->overlapPoints.size()))
        .arg(result->overlapMeanMm,0,'f',3)
        .arg(c.needsAdditionalView?tr("否，仍需下一视角"):tr("满足当前离线目标"));
    m_m7Info->setPlainText(details);
    m_m7ExportButton->setEnabled(true);
    setTwinViewPreset(5);
    onM8StageFinished(mls::twin::M8Stage::FusionM7,true);
}
void DigitalTwinScanWidget::cancelM7(){
    if(m_m7Cancel)m_m7Cancel->store(true);
    if(m_m7Info&&m_m7Watcher&&m_m7Watcher->isRunning())
        m_m7Info->setPlainText(tr("正在取消 M7 数据分析..."));
}
void DigitalTwinScanWidget::clearM7(){
    if(m_m7Cancel)m_m7Cancel->store(true);
    m_m7Discard=true;
    if(m_m7ProgressTimer)m_m7ProgressTimer->stop();
    m_m7Data.reset();
    for(auto* view:{m_view,m_mainView.data()})if(view)view->clearTwinM7Results();
    if(m_m7FeatureTable)m_m7FeatureTable->setRowCount(0);
    if(m_m7SegmentTable)m_m7SegmentTable->setRowCount(0);
    if(m_m7ExportButton)m_m7ExportButton->setEnabled(false);
    if(m_m7RunButton)m_m7RunButton->setEnabled(!m_m7Watcher||!m_m7Watcher->isRunning());
    if(m_m7CancelButton)m_m7CancelButton->setEnabled(m_m7Watcher&&m_m7Watcher->isRunning());
    if(m_m7ProgressBar){m_m7ProgressBar->setValue(0);m_m7ProgressBar->setFormat(tr("等待 M7"));}
    if(m_m7Info)m_m7Info->setPlainText(tr("已清除 M7 预览和验收图层。原始 M2/M6 SCAN 及 CAD 文件均未删除。"));
}
void DigitalTwinScanWidget::exportM7(){
    if(!m_m7Data||!m_m7Data->ok()||!m_m6Data||!m_m6Data->ok()||
       !m_m7ExportWatcher||m_m7ExportWatcher->isRunning())return;
    const auto path=QFileDialog::getSaveFileName(this,tr("保存 M7 多视角融合验收报告"),
        QFileInfo(m_replayFilePath).absolutePath()+"/twin_m7_fusion_report.json",
        tr("JSON 报告 (*.json)"));
    if(path.isEmpty())return;
    const auto data=m_m7Data;
    const auto m6=m_m6Data;
    const auto cancel=m_m7Cancel;
    m_m7ExportButton->setEnabled(false);
    m_m7ExportWatcher->setFuture(QtConcurrent::run([data,m6,path,cancel]{
        try{return mls::twin::TwinM7Adapter::exportReport(*data,*m6,path,cancel.get());}
        catch(const std::exception& e){return QStringLiteral("M7 导出异常：")+QString::fromLocal8Bit(e.what());}
        catch(...){return QStringLiteral("M7 导出发生未知异常");}
    }));
}
void DigitalTwinScanWidget::updateM7Progress(){
    if(!m_m7Watcher||!m_m7Watcher->isRunning()||!m_m7Progress||!m_m7ProgressBar)return;
    const auto phase=m_m7Progress->stage.load();
    const auto n=m_m7Progress->done.load();
    const auto total=m_m7Progress->total.load();
    const QString label=phase==1?tr("校验 CAD 覆盖"):
        phase==2?tr("整理真实 M2 点云"):
        phase==3?tr("融合虚拟 M6 点云"):tr("完成中");
    const int percentage=total?int(std::min<std::uint64_t>(100,100*n/total)):0;
    m_m7ProgressBar->setRange(0,100);m_m7ProgressBar->setValue(percentage);
    m_m7ProgressBar->setFormat(tr("%1：%2 / %3 (%p%)").arg(label)
        .arg(static_cast<qulonglong>(n)).arg(static_cast<qulonglong>(total)));
}
// V28.9 M8: sequential OFFLINE workflow integrating already-tested stages.
// M0/M1 CAD placement and REAL M2 loading always require explicit user action.
// Every stage keeps its own asynchronous worker and existing cancellation logic.
void DigitalTwinScanWidget::buildM8Workflow(QVBoxLayout* vbox,QWidget* panel){
    auto* group=new QGroupBox(tr("15. M8：完整数字孪生流程 / 分段验收（纯离线）"),panel);
    auto* layout=new QVBoxLayout(group);
    auto* help=new QLabel(tr("先手动导入 STEP 并设置位置，再导入真实 M2 SCAN。"
        "M8 依次运行 M3/M4（可选）、M5、M5.1/M5.2（H1 专用、可选）、M6 连续五轴及 M7。"
        "每个阶段成功后才会进入下一阶段；原始测量与模拟点始终区分。"
        "已有有效结果可以复用；未达到覆盖率目标时只报告剩余区域，不会循环伪造测量或控制机床。"),group);
    help->setWordWrap(true);layout->addWidget(help);
    auto* options=new QGridLayout;
    m_m8IncludeM3=new QCheckBox(tr("包含 M3 虚拟初扫"),group);
    m_m8IncludeM4=new QCheckBox(tr("包含 M4 实测/虚拟对比"),group);
    m_m8IncludeM51=new QCheckBox(tr("包含 M5.1 H1 倒角诊断"),group);
    m_m8IncludeM52=new QCheckBox(tr("包含 M5.2 H1 几何诊断（不自动补偿）"),group);
    m_m8Reuse=new QCheckBox(tr("复用当前会话已完成的有效阶段"),group);
    m_m8IncludeM3->setChecked(true);
    m_m8IncludeM4->setChecked(true);
    m_m8IncludeM51->setChecked(true);
    m_m8IncludeM52->setChecked(true);
    m_m8Reuse->setChecked(true);
    options->addWidget(m_m8IncludeM3,0,0);
    options->addWidget(m_m8IncludeM4,0,1);
    options->addWidget(m_m8IncludeM51,1,0);
    options->addWidget(m_m8IncludeM52,1,1);
    options->addWidget(m_m8Reuse,2,0,1,2);
    layout->addLayout(options);
    auto* controls=new QGridLayout;
    m_m8PreflightButton=new QPushButton(tr("M8 输入/安全边界预检查"),group);
    m_m8RunButton=new QPushButton(tr("启动 M8 一键离线流程"),group);
    m_m8CancelButton=new QPushButton(tr("停止 M8（取消当前后台步骤）"),group);
    m_m8ExportButton=new QPushButton(tr("导出 M8 验收 JSON + 剩余特征 CSV"),group);
    m_m8CancelButton->setEnabled(false);m_m8ExportButton->setEnabled(false);
    controls->addWidget(m_m8PreflightButton,0,0);
    controls->addWidget(m_m8RunButton,0,1);
    controls->addWidget(m_m8CancelButton,1,0);
    controls->addWidget(m_m8ExportButton,1,1);
    layout->addLayout(controls);
    m_m8ProgressBar=new QProgressBar(group);
    m_m8ProgressBar->setRange(0,100);m_m8ProgressBar->setValue(0);
    m_m8ProgressBar->setFormat(tr("等待 M8（按已完成阶段计数，非剩余时间）"));
    layout->addWidget(m_m8ProgressBar);
    m_m8StageTable=new QTableWidget(0,3,group);
    m_m8StageTable->setHorizontalHeaderLabels({tr("阶段"),tr("状态"),tr("数据作用")});
    m_m8StageTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_m8StageTable->horizontalHeader()->setStretchLastSection(true);
    m_m8StageTable->setMinimumHeight(210);layout->addWidget(m_m8StageTable);
    m_m8Info=new QPlainTextEdit(group);m_m8Info->setReadOnly(true);
    m_m8Info->setMinimumHeight(145);
    m_m8Info->setPlainText(tr("M8 尚未开始。STEP 与真实 M2 是前置条件。"
        " M6 的 B/C 虽可设置名义最大 90°，仍会根据五轴限位、动态约束和碰撞筛选。"
        "M6/M7 结果始终属于未经多角度实测光学校验的离线预测。"));
    layout->addWidget(m_m8Info);
    vbox->addWidget(group);
    connect(m_m8PreflightButton,&QPushButton::clicked,this,&DigitalTwinScanWidget::preflightM8);
    connect(m_m8RunButton,&QPushButton::clicked,this,&DigitalTwinScanWidget::startM8);
    connect(m_m8CancelButton,&QPushButton::clicked,this,&DigitalTwinScanWidget::cancelM8);
    connect(m_m8ExportButton,&QPushButton::clicked,this,&DigitalTwinScanWidget::exportM8Report);
    m_m8Timer=new QTimer(this);m_m8Timer->setInterval(400);
    connect(m_m8Timer,&QTimer::timeout,this,&DigitalTwinScanWidget::updateM8Progress);
}
QString DigitalTwinScanWidget::m8StepName(mls::twin::M8Stage stage)const{
    using S=mls::twin::M8Stage;
    switch(stage){
    case S::VirtualM3: return tr("M3 双光路初扫仿真");
    case S::ComparisonM4: return tr("M4 Profile 有效性比对");
    case S::CoverageM5: return tr("M5 CAD 表面覆盖");
    case S::ChamferM51: return tr("M5.1 中心倒角稀疏");
    case S::GeometryM52: return tr("M5.2 几何一致性（仅诊断）");
    case S::ContinuousM6: return tr("M6 连续 XYZ/C/B 补扫");
    case S::FusionM7: return tr("M7 来源区分融合与验收");
    }
    return tr("未知阶段");
}
bool DigitalTwinScanWidget::m8StageReady(mls::twin::M8Stage stage)const{
    using S=mls::twin::M8Stage;
    switch(stage){
    case S::VirtualM3:
        return m_virtual && m_virtual->ok() &&
               std::abs(m_virtual->opticalModel.cameraOriginSensor.y-
                        m_cameraBaselineY->value())<1e-7;
    case S::ComparisonM4:return m_m4Comparison&&m_m4Comparison->ok();
    case S::CoverageM5:
        if(!m_m5Data||!m_m5Data->ok())return false;
        return std::abs(m_m5Data->report.requestedOptions.cellPitchMm-m_m5Pitch->value())<1e-7 &&
               std::abs(m_m5Data->report.requestedOptions.maxSurfaceDistanceMm-
                        m_m5MatchDistance->value())<1e-7 &&
               m_m5Data->report.requestedOptions.minHitsPerCell==
                   static_cast<unsigned int>(m_m5MinHits->value()) &&
               m_m5Data->report.requestedOptions.excludeBottom==m_m5ExcludeBottom->isChecked();
    case S::ChamferM51:
        return m_m51Data && m_m51Data->ok() &&
               m_m51Data->report.options.primaryThreshold==m_m51Threshold->currentIndex() &&
               std::abs(m_m51Data->report.options.sparseRatio-m_m51SparseRatio->value())<1e-7;
    case S::GeometryM52:return m_m52Data&&m_m52Data->ok();
    case S::ContinuousM6:
        if(!m_m6Data||!m_m6Data->ok())return false;
        return std::abs(m_m6Data->report.options.scanRateHz-m_m6Hz->value())<1e-7 &&
               std::abs(m_m6Data->report.options.tiltAmplitudeDeg-m_m6Tilt->value())<1e-7 &&
               std::abs(m_m6Data->report.options.yawAmplitudeDeg-m_m6Yaw->value())<1e-7 &&
               m_m6Data->report.options.maxSelectedSegments==
                   static_cast<unsigned int>(m_m6Segments->value());
    case S::FusionM7:
        return m_m7Data && m_m7Data->ok() &&
               std::abs(m_m7Data->coverage.coverageThreshold-m_m7Goal->value()/100.)<1e-7;
    }
    return false;
}
bool DigitalTwinScanWidget::isM8SceneUnchanged()const{
    if(m_m8CadFile!=m_filePath || m_m8RealScanFile!=m_replayFilePath ||
       m_m8M2AtStart!=m_replay)return false;
    const auto a=m_m8PoseAtStart;
    const auto b=uiPose();
    const double aa[]={a.x,a.y,a.z,a.rxDeg,a.ryDeg,a.rzDeg};
    const double bb[]={b.x,b.y,b.z,b.rxDeg,b.ryDeg,b.rzDeg};
    for(int i=0;i<6;++i)if(std::abs(aa[i]-bb[i])>1e-8)return false;
    return true;
}
void DigitalTwinScanWidget::refreshM8StageTable(){
    if(!m_m8StageTable)return;
    m_m8StageTable->setRowCount(static_cast<int>(m_m8Sequence.size()));
    for(std::size_t i=0;i<m_m8Sequence.size();++i){
        const QStringList row{
            m8StepName(m_m8Sequence[i]),
            i<m_m8StepStates.size()?m_m8StepStates[i]:tr("等待"),
            (m_m8Sequence[i]==mls::twin::M8Stage::ContinuousM6 ||
             m_m8Sequence[i]==mls::twin::M8Stage::FusionM7)
              ?tr("模拟 / 离线预测，不是硬件测量")
              :tr("已导入数据分析，不修改原始文件")
        };
        for(int col=0;col<row.size();++col){
            auto* item=new QTableWidgetItem(row[col]);
            m_m8StageTable->setItem(static_cast<int>(i),col,item);
        }
    }
}
void DigitalTwinScanWidget::preflightM8(){
    mls::twin::M8WorkflowOptions options;
    options.simulateM3=m_m8IncludeM3->isChecked();
    options.compareM4=m_m8IncludeM4->isChecked();
    options.inspectChamferM51=m_m8IncludeM51->isChecked();
    options.inspectGeometryM52=m_m8IncludeM52->isChecked();
    options.reuseValidResults=m_m8Reuse->isChecked();
    options.coverageGoal=m_m7Goal->value()/100.0;
    const auto plan=mls::twin::makeM8Plan(options);
    mls::twin::M8InputState input;
    input.cadPlaced=m_session.scene.hasWorkpiece();
    input.realM2Loaded=m_replay&&m_replay->ok();
    input.hasM3=m_virtual&&m_virtual->ok();
    input.h1SpecificGeometry=(!m_m5Data || !m_m5Data->ok()) ||
                             m_m5Data->report.h1FeatureTemplate;
    input.otherTaskBusy=m_m8Running ||
        (m_replayWatcher&&m_replayWatcher->isRunning()) ||
        (m_virtualWatcher&&m_virtualWatcher->isRunning()) ||
        (m_m4CompareWatcher&&m_m4CompareWatcher->isRunning()) ||
        (m_m4FitWatcher&&m_m4FitWatcher->isRunning()) ||
        (m_m5Watcher&&m_m5Watcher->isRunning()) ||
        (m_m51Watcher&&m_m51Watcher->isRunning()) ||
        (m_m52Watcher&&m_m52Watcher->isRunning()) ||
        (m_m6Watcher&&m_m6Watcher->isRunning()) ||
        (m_m7Watcher&&m_m7Watcher->isRunning()) ||
        (m_m6ExportWatcher&&m_m6ExportWatcher->isRunning()) ||
        (m_m7ExportWatcher&&m_m7ExportWatcher->isRunning());
    if(input.realM2Loaded){
        for(const auto& p:m_replay->scan.profiles){
            if(p.fiveAxis.frame.timing==mls::runtime::PoseTiming::Simulated){
                input.m2SourceIsSimulated=true;break;
            }
        }
    }
    const auto check=mls::twin::inspectM8Preflight(plan,options,input);
    QString msg=check.permitted?tr("M8 预检查通过：只允许离线仿真。\n")
                               :tr("M8 预检查未通过：\n");
    for(const auto& b:check.blockers)msg+=tr("阻止：")+QString::fromStdString(b)+"\n";
    for(const auto& w:check.warnings)msg+=tr("提示：")+QString::fromStdString(w)+"\n";
    if(input.h1SpecificGeometry && !m_m5Data &&
        (options.inspectChamferM51||options.inspectGeometryM52))
        msg+=tr("提示：目前尚未有 M5 特征报告，M5 完成后将重新验证 H1 专用几何。\n");
    msg+=tr("M6 设置：%1 Hz，至多 %2 段，B 幅值 %3°，C 幅值 %4°。"
        "完整仿真可能生成很多压缩 SCAN 分卷。")
        .arg(m_m6Hz->value(),0,'f',0).arg(m_m6Segments->value())
        .arg(m_m6Tilt->value(),0,'f',1).arg(m_m6Yaw->value(),0,'f',1);
    m_m8Info->setPlainText(msg);
}
void DigitalTwinScanWidget::startM8(){
    if(m_m8Running)return;
    preflightM8();
    mls::twin::M8WorkflowOptions options;
    options.simulateM3=m_m8IncludeM3->isChecked();
    options.compareM4=m_m8IncludeM4->isChecked();
    options.inspectChamferM51=m_m8IncludeM51->isChecked();
    options.inspectGeometryM52=m_m8IncludeM52->isChecked();
    options.reuseValidResults=m_m8Reuse->isChecked();
    options.coverageGoal=m_m7Goal->value()/100.;
    const auto plan=mls::twin::makeM8Plan(options);
    mls::twin::M8InputState input;
    input.cadPlaced=m_session.scene.hasWorkpiece();
    input.realM2Loaded=m_replay&&m_replay->ok();
    input.hasM3=m_virtual&&m_virtual->ok();
    input.h1SpecificGeometry=(!m_m5Data||!m_m5Data->ok())||m_m5Data->report.h1FeatureTemplate;
    input.otherTaskBusy=(m_replayWatcher&&m_replayWatcher->isRunning()) ||
        (m_virtualWatcher&&m_virtualWatcher->isRunning()) ||
        (m_m4CompareWatcher&&m_m4CompareWatcher->isRunning()) ||
        (m_m4FitWatcher&&m_m4FitWatcher->isRunning()) ||
        (m_m5Watcher&&m_m5Watcher->isRunning()) ||
        (m_m51Watcher&&m_m51Watcher->isRunning()) ||
        (m_m52Watcher&&m_m52Watcher->isRunning()) ||
        (m_m6Watcher&&m_m6Watcher->isRunning()) ||
        (m_m7Watcher&&m_m7Watcher->isRunning()) ||
        (m_m6ExportWatcher&&m_m6ExportWatcher->isRunning()) ||
        (m_m7ExportWatcher&&m_m7ExportWatcher->isRunning());
    if(input.realM2Loaded)for(const auto& p:m_replay->scan.profiles){
        if(p.fiveAxis.frame.timing==mls::runtime::PoseTiming::Simulated){
            input.m2SourceIsSimulated=true;break;
        }
    }
    const auto check=mls::twin::inspectM8Preflight(plan,options,input);
    if(!check.permitted){
        QMessageBox::warning(this,tr("M8 不能启动"),
            tr("STEP、真实 M2、H1 专用选项或后台任务状态不满足要求。请先点击预检查并修正。"));
        return;
    }
    if(QMessageBox::question(this,tr("M8 离线全流程确认"),
        tr("这将按顺序启动 M3～M7 中所选的后台阶段，可能运行较长时间、使用大量 CPU 和磁盘。"
           "本流程只读取 STEP/实测 SCAN，不向真实机床发指令。是否继续？"),
        QMessageBox::Yes|QMessageBox::No,QMessageBox::No)!=QMessageBox::Yes)return;
    m_m8Options=options;
    m_m8Sequence=plan.stages;
    m_m8StepStates.assign(m_m8Sequence.size(),tr("等待"));
    m_m8Index=0;m_m8Cancelling=false;m_m8Completed=false;m_m8Running=true;
    m_m8Acceptance=mls::twin::M8Acceptance{};
    m_m8CadFile=m_filePath;m_m8RealScanFile=m_replayFilePath;
    m_m8PoseAtStart=uiPose();m_m8M2AtStart=m_replay;
    m_m8RunButton->setEnabled(false);
    m_m8CancelButton->setEnabled(true);
    // Lock configuration widgets while the snapshot-driven asynchronous pipeline runs.
    for(auto* box:m_pose)if(box)box->setEnabled(false);
    for(QDoubleSpinBox* box:{m_cameraBaselineY,m_m5Pitch,m_m5MatchDistance,
                             m_m6Hz,m_m6Tilt,m_m6Yaw,m_m7Goal})if(box)box->setEnabled(false);
    if(m_m6Segments)m_m6Segments->setEnabled(false);
    for(QDoubleSpinBox* box:{m_supportZ,m_supportX,m_supportY,m_supportW,m_supportH,
                             m_m51SparseRatio,m_m7VoxelPitch})if(box)box->setEnabled(false);
    for(QSpinBox* box:{m_m5MinHits,m_m6ExportStride,m_m7VoxelCap})if(box)box->setEnabled(false);
    for(QCheckBox* box:{m_supportCheck,m_idealRegistration,m_m5ExcludeBottom,
                        m_m6SkipFcl,m_m6AlsoPly})if(box)box->setEnabled(false);
    if(m_m51Threshold)m_m51Threshold->setEnabled(false);
    for(QCheckBox* box:{m_m8IncludeM3,m_m8IncludeM4,m_m8IncludeM51,
                        m_m8IncludeM52,m_m8Reuse})if(box)box->setEnabled(false);
    m_m8ExportButton->setEnabled(false);
    m_m8PreflightButton->setEnabled(false);
    m_m8ProgressBar->setValue(0);
    refreshM8StageTable();
    m_m8Info->appendPlainText(tr("\nM8 自动流程已经启动（SIMULATED ONLY）。"));
    m_m8Timer->start();
    QTimer::singleShot(0,this,[this]{dispatchM8Stage();});
}
void DigitalTwinScanWidget::dispatchM8Stage(){
    if(!m_m8Running)return;
    if(m_m8Cancelling){finishM8(tr("用户取消离线流程"),true);return;}
    if(!isM8SceneUnchanged()){
        finishM8(tr("STEP、位姿或 M2 数据在自动流程中发生变更，已停止"));return;
    }
    if(m_m8Index>=m_m8Sequence.size()){
        if(!m_m5Data||!m_m6Data||!m_m7Data||
           !m_m5Data->ok()||!m_m6Data->ok()||!m_m7Data->ok()){
            finishM8(tr("M7 验收缺少必要数据"));return;
        }
        m_m8Acceptance=mls::twin::evaluateM8Acceptance(
            m_m5Data->report,m_m6Data->report,m_m7Data->coverage,m_m8Options.coverageGoal);
        if(!m_m8Acceptance.ok()){
            finishM8(tr("M5/M6/M7 面积或数据来源核算失败：")+
                     QString::fromStdString(m_m8Acceptance.error));return;
        }
        m_m8Completed=true;
        const auto& a=m_m8Acceptance;
        QString verdict=tr("M8 统一验收完成：\n"
            "M2 实测已覆盖 %1 mm²；M6 虚拟新增 %2 mm²；剩余缺失 %3 mm²。\n"
            "预测最终面积覆盖率 %4%%；目标 %5%%；预测状态：%6。\n"
            "重要：光学多姿态尚未实测标定、完整夹具/外壳碰撞尚未认证，"
            "所以硬件授权恒为否。M7 的预测合格不等于实体工件完成真实检测。\n")
            .arg(a.realAreaMm2,0,'f',2).arg(a.simulatedNewAreaMm2,0,'f',2)
            .arg(a.stillMissingAreaMm2,0,'f',2).arg(a.predictedFraction*100.,0,'f',2)
            .arg(a.targetFraction*100.,0,'f',1)
            .arg(a.predictedTargetReached?tr("达到离线面积目标"):tr("未达到，需下一视角/重新评估"));
        for(std::size_t i=0;i<std::min<std::size_t>(a.priorities.size(),5);++i){
            const auto& f=a.priorities[i];
            verdict+=tr("剩余目标 %1：%2 mm²（占该特征 %3%）\n")
                .arg(QString::fromStdString(f.feature))
                .arg(f.stillMissingAreaMm2,0,'f',2)
                .arg(f.fractionMissing*100.,0,'f',1);
        }
        finishM8(verdict);return;
    }
    const auto stage=m_m8Sequence[m_m8Index];
    if((stage==mls::twin::M8Stage::ChamferM51||stage==mls::twin::M8Stage::GeometryM52) &&
       m_m5Data && m_m5Data->ok() && !m_m5Data->report.h1FeatureTemplate){
        finishM8(tr("当前 STEP 不是 M5 识别的 H1 特征模板；请取消 H1 专用 M5.1/M5.2 选项"));return;
    }
    if(m_m8Options.reuseValidResults && m8StageReady(stage)){
        m_m8StepStates[m_m8Index]=tr("复用会话内已有结果");
        ++m_m8Index;refreshM8StageTable();updateM8Progress();
        QTimer::singleShot(0,this,[this]{dispatchM8Stage();});return;
    }
    m_m8StepStates[m_m8Index]=tr("正在执行");refreshM8StageTable();
    m_m8Info->appendPlainText(tr("执行 %1 …").arg(m8StepName(stage)));
    using S=mls::twin::M8Stage;
    switch(stage){
    case S::VirtualM3:startVirtualSimulation();break;
    case S::ComparisonM4:startM4Comparison();break;
    case S::CoverageM5:startM5();break;
    case S::ChamferM51:
        // Recomputing the chamfer sparsity mask changes which "already-covered"
        // coarse M5 cells must be re-opened. Invalidate any old M6/M7 result.
        clearM6();startM51();break;
    case S::GeometryM52:startM52();break;
    case S::ContinuousM6:startM6();break;
    case S::FusionM7:startM7();break;
    }
    // All start methods were already checked for M0/M1/M2 prerequisites.
    // Their QFutureWatcher callbacks call onM8StageFinished() exactly once.
}
void DigitalTwinScanWidget::onM8StageFinished(mls::twin::M8Stage stage,bool ok,
                                               const QString& reason){
    if(!m_m8Running||m_m8Index>=m_m8Sequence.size()||
       m_m8Sequence[m_m8Index]!=stage)return;
    if(m_m8Cancelling){finishM8(tr("后台任务已响应取消"),true);return;}
    if(!ok||!m8StageReady(stage)){
        m_m8StepStates[m_m8Index]=tr("失败或已取消");refreshM8StageTable();
        finishM8(tr("阶段 %1 未成功完成：%2").arg(m8StepName(stage),reason));return;
    }
    m_m8StepStates[m_m8Index]=tr("完成");
    ++m_m8Index;refreshM8StageTable();updateM8Progress();
    QTimer::singleShot(0,this,[this]{dispatchM8Stage();});
}
void DigitalTwinScanWidget::cancelM8(){
    if(!m_m8Running)return;
    m_m8Cancelling=true;
    if(m_m8Index<m_m8Sequence.size()){
        using S=mls::twin::M8Stage;
        switch(m_m8Sequence[m_m8Index]){
        case S::VirtualM3:cancelVirtualSimulation();break;
        case S::ComparisonM4:cancelM4();break;
        case S::CoverageM5:cancelM5();break;
        case S::ChamferM51:cancelM51();break;
        case S::GeometryM52:cancelM52();break;
        case S::ContinuousM6:cancelM6();break;
        case S::FusionM7:cancelM7();break;
        }
    }
    m_m8Info->appendPlainText(tr("已请求 M8 取消，等待当前后台阶段结束。"));
}
void DigitalTwinScanWidget::finishM8(const QString& reason,bool cancelled){
    if(!m_m8Running)return;
    m_m8Running=false;m_m8Cancelling=false;
    if(m_m8Timer)m_m8Timer->stop();
    if(m_m8RunButton)m_m8RunButton->setEnabled(true);
    if(m_m8CancelButton)m_m8CancelButton->setEnabled(false);
    for(auto* box:m_pose)if(box)box->setEnabled(true);
    for(QDoubleSpinBox* box:{m_cameraBaselineY,m_m5Pitch,m_m5MatchDistance,
                             m_m6Hz,m_m6Tilt,m_m6Yaw,m_m7Goal})if(box)box->setEnabled(true);
    if(m_m6Segments)m_m6Segments->setEnabled(true);
    for(QDoubleSpinBox* box:{m_supportZ,m_supportX,m_supportY,m_supportW,m_supportH,
                             m_m51SparseRatio,m_m7VoxelPitch})if(box)box->setEnabled(true);
    for(QSpinBox* box:{m_m5MinHits,m_m6ExportStride,m_m7VoxelCap})if(box)box->setEnabled(true);
    for(QCheckBox* box:{m_supportCheck,m_idealRegistration,m_m5ExcludeBottom,
                        m_m6SkipFcl,m_m6AlsoPly})if(box)box->setEnabled(true);
    if(m_m51Threshold)m_m51Threshold->setEnabled(true);
    for(QCheckBox* box:{m_m8IncludeM3,m_m8IncludeM4,m_m8IncludeM51,
                        m_m8IncludeM52,m_m8Reuse})if(box)box->setEnabled(true);
    if(m_m8PreflightButton)m_m8PreflightButton->setEnabled(true);
    if(m_m8ExportButton)m_m8ExportButton->setEnabled(true);
    if(m_m8ProgressBar){
        m_m8ProgressBar->setValue(m_m8Completed?100:int(100*m_m8Index/std::max<std::size_t>(1,m_m8Sequence.size())));
        m_m8ProgressBar->setFormat(m_m8Completed?tr("M8 所有选定阶段完成"):
                                   cancelled?tr("M8 已取消"):tr("M8 未完成"));
    }
    if(m_m8Info)m_m8Info->appendPlainText(reason);
    m_m8M2AtStart.reset();
}
void DigitalTwinScanWidget::updateM8Progress(){
    if(!m_m8Running||!m_m8ProgressBar)return;
    const std::size_t n=std::max<std::size_t>(1,m_m8Sequence.size());
    m_m8ProgressBar->setRange(0,100);
    m_m8ProgressBar->setValue(int(std::min<std::size_t>(100,m_m8Index*100/n)));
    if(m_m8Index>=m_m8Sequence.size())return;
    const auto stage=m_m8Sequence[m_m8Index];
    QString status=tr("阶段 %1/%2：%3（按完成阶段计进度，并非剩余时间）")
        .arg(static_cast<qulonglong>(m_m8Index+1))
        .arg(static_cast<qulonglong>(n)).arg(m8StepName(stage));
    if(stage==mls::twin::M8Stage::ContinuousM6 && m_m6DetailedProgress){
        const auto& pr=*m_m6DetailedProgress;
        status+=tr(" | 已完成扫描段 %1，正式曝光 %2")
            .arg(static_cast<qulonglong>(pr.segmentsCompleted.load()))
            .arg(static_cast<qulonglong>(pr.fullProfiles.load()));
    }
    m_m8ProgressBar->setFormat(status);
}
void DigitalTwinScanWidget::exportM8Report(){
    if(m_m8Running){QMessageBox::information(this,tr("M8 正在运行"),tr("请先等待流程完成或取消。"));return;}
    if(m_m8Sequence.empty()){
        QMessageBox::information(this,tr("没有 M8 流程"),tr("请先运行 M8 流程。"));return;
    }
    const QString file=QFileDialog::getSaveFileName(this,tr("导出 M8 整体验收报告"),
        QFileInfo(m_replayFilePath).absolutePath()+"/twin_m8_workflow_acceptance.json",
        tr("JSON 报告 (*.json)"));
    if(file.isEmpty())return;
    QJsonObject root;
    root.insert("schema",QStringLiteral("MotionLaserSync.M8OfflineWorkflow"));
    root.insert("schemaVersion",1);
    root.insert("sourceCAD",m_m8CadFile);
    root.insert("sourceRealM2",m_m8RealScanFile);
    root.insert("offlineOnly",true);
    root.insert("hardwareAuthorized",false);
    root.insert("completed",m_m8Completed);
    root.insert("multiAngleOpticsValidated",false);
    root.insert("fixtureHousingCollisionCertified",false);
    root.insert("poseAtStart",jsonPose(m_m8PoseAtStart));
    root.insert("goalFraction",m_m8Options.coverageGoal);
    QJsonArray stages;
    for(std::size_t i=0;i<m_m8Sequence.size();++i){
        QJsonObject row;
        row.insert("name",m8StepName(m_m8Sequence[i]));
        row.insert("status",i<m_m8StepStates.size()?m_m8StepStates[i]:QString());
        stages.append(row);
    }
    root.insert("stages",stages);
    root.insert("message",m_m8Info?m_m8Info->toPlainText():QString());
    root.insert("coverageBookkeepingValid",m_m8Acceptance.coverageBookkeepingValid);
    root.insert("predictedCoverageReached",m_m8Acceptance.predictedTargetReached);
    root.insert("realMeasuredAreaMm2",m_m8Acceptance.realAreaMm2);
    root.insert("simulatedNewAreaMm2",m_m8Acceptance.simulatedNewAreaMm2);
    root.insert("remainingAreaMm2",m_m8Acceptance.stillMissingAreaMm2);
    root.insert("predictedFinalFraction",m_m8Acceptance.predictedFraction);
    root.insert("acceptanceError",QString::fromStdString(m_m8Acceptance.error));
    QJsonArray features;
    for(const auto& feature:m_m8Acceptance.priorities){
        QJsonObject f;
        f.insert("id",int(feature.id));
        f.insert("name",QString::fromStdString(feature.feature));
        f.insert("remainingAreaMm2",feature.stillMissingAreaMm2);
        f.insert("simulatedNewAreaMm2",feature.simulatedNewAreaMm2);
        f.insert("targetAreaMm2",feature.targetAreaMm2);
        f.insert("fractionMissing",feature.fractionMissing);
        features.append(f);
    }
    root.insert("remainingFeatures",features);
    if(m_m6Data && m_m6Data->ok()){
        root.insert("m6SimulatedProfiles",double(m_m6Data->report.fullProfiles));
        root.insert("m6Segments",int(m_m6Data->report.segments.size()));
        root.insert("m6MachineTransferAuthorized",false);
    }
    QSaveFile jsonOut(file);
    if(!jsonOut.open(QIODevice::WriteOnly)){
        QMessageBox::warning(this,tr("M8 导出失败"),jsonOut.errorString());return;
    }
    const QByteArray bytes=QJsonDocument(root).toJson(QJsonDocument::Indented);
    if(jsonOut.write(bytes)!=bytes.size() || !jsonOut.commit()){
        QMessageBox::warning(this,tr("M8 导出失败"),jsonOut.errorString());return;
    }
    QString csv=file;
    if(csv.endsWith(".json",Qt::CaseInsensitive))csv.chop(5);
    csv+="_remaining_features.csv";
    QSaveFile csvOut(csv);
    if(!csvOut.open(QIODevice::WriteOnly|QIODevice::Text)){
        QMessageBox::warning(this,tr("M8 CSV 导出失败"),csvOut.errorString());return;
    }
    QTextStream ts(&csvOut);ts.setEncoding(QStringConverter::Utf8);
    ts<<"featureId,featureName,targetAreaMm2,simulatedNewAreaMm2,stillMissingAreaMm2,missingFraction\n";
    for(const auto& f:m_m8Acceptance.priorities){
        QString name=QString::fromStdString(f.feature);
        name.replace('"',QStringLiteral("''"));
        ts<<f.id<<",\""<<name<<"\","<<f.targetAreaMm2<<","<<f.simulatedNewAreaMm2
          <<","<<f.stillMissingAreaMm2<<","<<f.fractionMissing<<'\n';
    }
    ts.flush();
    if(ts.status()!=QTextStream::Ok||!csvOut.commit()){
        QMessageBox::warning(this,tr("M8 CSV 导出失败"),csvOut.errorString());return;
    }
    QMessageBox::information(this,tr("M8 导出完成"),
        tr("已保存 M8 JSON 和逐 CAD 特征剩余目标 CSV。全部数值属于离线诊断，真实 M2/M6 SCAN 原文件未被修改。"));
}
