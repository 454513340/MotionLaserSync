#include "widgets/MultiPoseExperimentWidget.h"
#include "widgets/PointCloudWidget.h"
#include "twin/CadGeometryAdapter.h"
#include "twin/TwinM6NcPreview.h"
#include <QApplication>
#include <QCryptographicHash>
#include <QCheckBox>
#include <QComboBox>
#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QProgressBar>
#include <QPushButton>
#include <QSaveFile>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QSlider>
#include <QSpinBox>
#include <QTableWidget>
#include <QTemporaryDir>
#include <QTimer>
#include <QVBoxLayout>
#include <QtConcurrent>
#include <algorithm>
#include <cmath>
#include <limits>

namespace {
QString inputPath(const QString& name){return QDir::toNativeSeparators(name);}
std::string nativeFile(const QString& name){return inputPath(name).toLocal8Bit().toStdString();}
QDoubleSpinBox* numeric(QWidget* p,double value,double lo,double hi,int precision=3){
    auto* s=new QDoubleSpinBox(p);s->setDecimals(precision);s->setRange(lo,hi);
    s->setValue(value);s->setSingleStep(precision>=3?0.5:5.0);return s;
}
}
MultiPoseExperimentWidget::MultiPoseExperimentWidget(QWidget* parent):QWidget(parent){
    buildUi();
    connect(&m_realWatcher,&QFutureWatcher<std::shared_ptr<mls::twin::ScanReplayData>>::finished,
            this,&MultiPoseExperimentWidget::scanLoaded);
    connect(&m_regWatcher,&QFutureWatcher<std::shared_ptr<mls::twin::RegistrationAssistReport>>::finished,
            this,&MultiPoseExperimentWidget::roughRegistrationFinished);
    connect(&m_planWatcher,&QFutureWatcher<std::shared_ptr<mls::twin::MultiPoseQtReport>>::finished,
            this,&MultiPoseExperimentWidget::planFinished);
    connect(&m_simWatcher,&QFutureWatcher<QString>::finished,
            this,&MultiPoseExperimentWidget::simulationFinished);
    connect(&m_compareWatcher,&QFutureWatcher<std::shared_ptr<mls::twin::MultiPoseCompareArtifacts>>::finished,
            this,&MultiPoseExperimentWidget::comparisonFinished);
}
MultiPoseExperimentWidget::~MultiPoseExperimentWidget(){
    stopPlayback();
    if(m_cancel)m_cancel->store(true);
    if(m_realWatcher.isRunning())m_realWatcher.waitForFinished();
    if(m_regWatcher.isRunning())m_regWatcher.waitForFinished();
    if(m_planWatcher.isRunning())m_planWatcher.waitForFinished();
    if(m_simWatcher.isRunning())m_simWatcher.waitForFinished();
    if(m_compareWatcher.isRunning())m_compareWatcher.waitForFinished();
}
bool MultiPoseExperimentWidget::snapshotCollisionWorkpiece(mls::twin::TwinScene& scene,
    QByteArray& identity, bool& offlinePoseReviewed, QString* error) const
{
    offlinePoseReviewed=false;
    identity.clear();
    if(!m_scene.hasWorkpiece() || m_cadFile.isEmpty() || m_cadSha256.isEmpty()){
        if(error)*error=tr("请先在路径规划→多姿态实验扫描导入真实零件STEP并设置工件CAD→机床位姿");
        return false;
    }
    QFile file(m_cadFile);
    if(!file.open(QIODevice::ReadOnly)){
        if(error)*error=tr("无法重新读取STEP：%1").arg(m_cadFile);
        return false;
    }
    QCryptographicHash hash(QCryptographicHash::Sha256);
    if(!hash.addData(&file) || hash.result()!=m_cadSha256){
        if(error)*error=tr("STEP文件在导入后被修改；请重新导入并核对CAD位姿");
        return false;
    }
    QCryptographicHash proof(QCryptographicHash::Sha256);
    proof.addData(m_cadSha256);
    const auto pose=m_scene.estimatedPose();
    const std::array<double,6> values{{pose.x,pose.y,pose.z,pose.rxDeg,pose.ryDeg,pose.rzDeg}};
    for(double v:values){
        if(!std::isfinite(v)){
            if(error)*error=tr("STEP工件摆放含非有限值");
            return false;
        }
        proof.addData(QByteArray::number(v,'g',17));
        proof.addData(";",1);
    }
    identity=proof.result();
    offlinePoseReviewed=m_previewPoseAcknowledged;
    scene=m_scene;
    if(error)error->clear();
    return true;
}
bool MultiPoseExperimentWidget::snapshotOpticalForExactNc(const QString& text,
    mls::twin::TwinScene& scene,std::shared_ptr<const mls::twin::MultiPoseQtReport>& report,
    QString* error) const
{
    if(!m_report || !m_report->ok() || !m_report->physical17ReplayConsistent ||
       !m_scene.hasWorkpiece()){
        if(error)*error=tr("多姿态实验没有完成STEP/SCAN配准及规划；必须先加载真实CAD场景与标定数据");
        return false;
    }
    QString candidate=text;
    candidate.replace(QStringLiteral("\r\n"),QStringLiteral("\n"));
    auto same=[&](const std::string& raw){
        QString expected=QString::fromUtf8(raw.data(),static_cast<qsizetype>(raw.size()));
        expected.replace(QStringLiteral("\r\n"),QStringLiteral("\n"));
        return candidate==expected;
    };
    bool matched=same(m_connectedNcText);
    for(const auto& raw:m_activeNcTexts)if(same(raw)){matched=true;break;}
    if(!matched){
        if(error)*error=tr("G代码与当前多姿态实验保留的NC文本不同，不能用旧STEP/ROI覆盖证据证明新的轨迹；请在实验中重新导入/规划");
        return false;
    }
    scene=m_scene;
    report=m_report;
    if(error)error->clear();
    return true;
}

void MultiPoseExperimentWidget::setMainPointCloudView(PointCloudWidget* v){m_view=v;syncScene(true);}
void MultiPoseExperimentWidget::buildUi(){
    auto* box=new QVBoxLayout(this);box->setContentsMargins(5,5,5,5);
    auto* note=new QLabel(tr("多姿态实验扫描（用于 M3/M4 不同视角研究）——独立于 M0–M8 全覆盖流程；只生成离线轨迹/仿真/NC，禁止直接执行。"),this);
    note->setWordWrap(true);note->setStyleSheet("QLabel{background:#fff1d6;color:#6c3700;padding:7px;font-weight:bold;}");box->addWidget(note);
    auto* imports=new QGroupBox(tr("1 研究数据与工件位置"),this);
    auto* imp=new QVBoxLayout(imports);
    auto* ir=new QHBoxLayout;
    auto* step=new QPushButton(tr("导入 STEP"),imports);
    auto* scan=new QPushButton(tr("导入真实 .scan"),imports);
    auto* fromM0=new QPushButton(tr("导入旧 M0–M8 保存的 CAD 初位姿（非配准）"),imports);
    fromM0->setToolTip(tr("仅把旧 DigitalTwinScene JSON 的 poseEstimate 作为人工初值。\n"
       "不会读取 M3/M4 分析结果，也不会执行 STEP/SCAN 配准。"));
    ir->addWidget(step);ir->addWidget(scan);imp->addLayout(ir);imp->addWidget(fromM0);
    m_inputs=new QLabel(tr("未加载 STEP / SCAN。必须获得真实原始 Profile 与 Physical17，CAD 放置需复核。"),imports);
    m_inputs->setWordWrap(true);imp->addWidget(m_inputs);
    auto* poseLayout=new QGridLayout;
    const char* labels[]={"工件 X(mm)","Y(mm)","Z(mm)","Rx(°)","Ry(°)","Rz(°)"};
    for(int j=0;j<6;++j){
        poseLayout->addWidget(new QLabel(tr(labels[j]),imports),j/2,j%2*2);
        m_pose[j]=numeric(imports,0.,j<3?-10000.:-360.,j<3?10000.:360.,4);
        poseLayout->addWidget(m_pose[j],j/2,j%2*2+1);
        connect(m_pose[j],qOverload<double>(&QDoubleSpinBox::valueChanged),this,[this](double){
            m_scene.setEstimatedPose(uiCadPose());syncScene();invalidate(true);
        });
    }
    imp->addLayout(poseLayout);
    auto* poseHelp=new QLabel(tr("CAD 位姿采用 Rz·Ry·Rx，≠机床 C/B。手工输入和旧 M0 场景均不等于独立 6DoF 配准验收。"),imports);
    poseHelp->setWordWrap(true);imp->addWidget(poseHelp);
    auto* registrationRow=new QHBoxLayout;
    m_btnEstimate=new QPushButton(tr("① 从 STEP + .scan 提议粗配准"),imports);
    m_btnApplyEstimate=new QPushButton(tr("② 复核后应用候选位姿"),imports);
    m_btnManualPreview=new QPushButton(tr("只对手工位姿做离线预览"),imports);
    m_btnApplyEstimate->setEnabled(false);
    registrationRow->addWidget(m_btnEstimate);
    registrationRow->addWidget(m_btnApplyEstimate);
    registrationRow->addWidget(m_btnManualPreview);
    imp->addLayout(registrationRow);
    m_registrationStatus=new QLabel(tr("配准状态：未计算。请先导入 STEP 与真实 SCAN，再进行粗配准或人工复核。"),imports);
    m_registrationStatus->setWordWrap(true);
    m_registrationStatus->setStyleSheet("QLabel{background:#e8f0fa;padding:6px;color:#173f63;}");
    imp->addWidget(m_registrationStatus);box->addWidget(imports);
    connect(m_btnEstimate,&QPushButton::clicked,this,&MultiPoseExperimentWidget::startRoughRegistration);
    connect(m_btnApplyEstimate,&QPushButton::clicked,this,&MultiPoseExperimentWidget::applyRoughRegistration);
    connect(m_btnManualPreview,&QPushButton::clicked,this,&MultiPoseExperimentWidget::acceptManualPreviewPose);
    connect(step,&QPushButton::clicked,this,&MultiPoseExperimentWidget::importStep);
    connect(scan,&QPushButton::clicked,this,&MultiPoseExperimentWidget::importRealScan);
    connect(fromM0,&QPushButton::clicked,this,&MultiPoseExperimentWidget::loadTwinSession);

    auto* orientations=new QGroupBox(tr("2 多姿态实验矩阵（同一 ROI，独立 C/B）"),this);
    auto* ol=new QVBoxLayout(orientations);
    m_poses=new QTableWidget(0,3,orientations);
    m_poses->setHorizontalHeaderLabels({tr("姿态 ID"),tr("C (度)"),tr("B (度)")});
    m_poses->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    m_poses->setMinimumHeight(136);ol->addWidget(m_poses);
    auto* orientActions=new QHBoxLayout;
    auto* add=new QPushButton(tr("增加姿态"),orientations);
    auto* del=new QPushButton(tr("删除选中"),orientations);
    auto* defaults=new QPushButton(tr("恢复 B/C 37组（B/C±10°、步长4°、另加0/0）"),orientations);
    orientActions->addWidget(defaults);
    orientActions->addWidget(add);
    orientActions->addWidget(del);
    ol->addLayout(orientActions);box->addWidget(orientations);
    connect(defaults,&QPushButton::clicked,this,&MultiPoseExperimentWidget::restoreStandardMatrix);
    connect(add,&QPushButton::clicked,this,[this]{addOrientation();});
    connect(del,&QPushButton::clicked,this,&MultiPoseExperimentWidget::removeOrientation);
    connect(m_poses,&QTableWidget::itemChanged,this,[this](QTableWidgetItem*){invalidate();});

    auto* recipe=new QGroupBox(tr("3 单姿态轨迹配方（CAD 局部 ROI）"),this);
    auto* form=new QGridLayout(recipe);
    const char* roiLabel[]={"Xmin","Xmax","Ymin","Ymax","顶面参考 Z"};
    for(int j=0;j<5;++j){
        m_roi[j]=numeric(recipe,j%2?10.:-10.,-10000.,10000.,3);
        form->addWidget(new QLabel(tr(roiLabel[j]),recipe),j/2,2*(j%2));
        form->addWidget(m_roi[j],j/2,2*(j%2)+1);
        connect(m_roi[j],qOverload<double>(&QDoubleSpinBox::valueChanged),this,[this](double){invalidate();});
    }
    m_axis=new QComboBox(recipe);
    m_axis->addItems({tr("按机床 X+ → X− 自动选择（推荐）"),
                      tr("沿 CAD Y（需与机床 X 对齐）"),
                      tr("沿 CAD X（需与机床 X 对齐）")});
    form->addWidget(new QLabel(tr("扫描方向"),recipe),3,0);form->addWidget(m_axis,3,1);
    m_speed=numeric(recipe,5.,.01,1000.);
    m_rate=numeric(recipe,200.,1.,2000.,0);
    m_transferSpeed=numeric(recipe,5.,.01,1000.);
    m_pitch=numeric(recipe,32.,.1,200.);
    m_focus=numeric(recipe,95.,1.,500.);
    // The requested scan recipe has no Z lift/approach/retraction.
    // Preserve the member for compatibility with existing code, but do not
    // expose a misleading retract field or include it in any planned motion.
    m_retract=numeric(recipe,0.,0.,0.);
    m_leadIn=numeric(recipe,5.,0.,100.);
    m_leadOut=numeric(recipe,5.,0.,100.);
    const std::array<std::pair<QString,QDoubleSpinBox*>,5> rows{{
        {tr("扫速 mm/s"),m_speed},{tr("频率 Hz"),m_rate},
        {tr("转场速度 mm/s"),m_transferSpeed},{tr("条带间距 mm"),m_pitch},
        {tr("传感器聚焦 Z mm"),m_focus}}};
    for(int j=0;j<int(rows.size());++j){
        const int row=4+j/2,col=(j%2)*2;
        form->addWidget(new QLabel(rows[j].first,recipe),row,col);
        form->addWidget(rows[j].second,row,col+1);
        connect(rows[j].second,qOverload<double>(&QDoubleSpinBox::valueChanged),this,[this](double){invalidate();});
    }
    m_maxProfiles=new QSpinBox(recipe);
    m_maxProfiles->setRange(100,1000000);
    m_maxProfiles->setValue(160000);
    form->addWidget(new QLabel(tr("全部姿态 Profile 预算"),recipe),7,0);
    form->addWidget(m_maxProfiles,7,1);
    form->addWidget(new QLabel(tr("扫码前延伸 mm"),recipe),7,2);
    form->addWidget(m_leadIn,7,3);
    form->addWidget(new QLabel(tr("扫码后延伸 mm"),recipe),8,2);
    form->addWidget(m_leadOut,8,3);
    connect(m_leadIn,qOverload<double>(&QDoubleSpinBox::valueChanged),this,[this](double){invalidate();});
    connect(m_leadOut,qOverload<double>(&QDoubleSpinBox::valueChanged),this,[this](double){invalidate();});
    m_h3Edge=new QComboBox(recipe);m_h3Edge->addItem("REGIST(114)",114);m_h3Edge->addItem("REGIST(115)",115);
    form->addWidget(new QLabel(tr("H3 锁存沿（仅提议）"),recipe),9,0);form->addWidget(m_h3Edge,9,1);
    connect(m_axis,qOverload<int>(&QComboBox::currentIndexChanged),this,[this](int){invalidate();});
    connect(m_maxProfiles,qOverload<int>(&QSpinBox::valueChanged),this,[this](int){invalidate();});
    connect(m_h3Edge,qOverload<int>(&QComboBox::currentIndexChanged),this,[this](int){invalidate();});
    box->addWidget(recipe);
    auto* previewBox=new QGroupBox(tr("4 共享三维窗口中的多姿态实验图层"),this);
    auto* pv=new QVBoxLayout(previewBox);
    m_previewPose=new QComboBox(previewBox);
    m_previewPose->addItem(tr("显示全部姿态"));
    pv->addWidget(m_previewPose);
    connect(m_previewPose,qOverload<int>(&QComboBox::currentIndexChanged),this,[this](int index){
        if(m_view)m_view->setTwinMultiPosePoseSelection(index-1);
    });
    auto* layers=new QGridLayout;
    const struct {const char* name;const char* label;} visibleLayers[]={
        {"MULTIPOSE_CAD","实验 CAD"},{"MULTIPOSE_REAL_REFERENCE","预扫实测"},
        {"MULTIPOSE_VIRTUAL","虚拟轮廓"},{"MULTIPOSE_PATH","扫描轨迹"},
        {"MULTIPOSE_TRANSFER","非采集转场"},
        {"MULTIPOSE_PLAYBACK","回放实时轮廓"},
        {"MULTIPOSE_COMPARISON","实测-仿真差值"}};
    int layerIndex=0;
    for(const auto& layer:visibleLayers){
        auto* ck=new QCheckBox(tr(layer.label),previewBox);
        ck->setChecked(true);layers->addWidget(ck,layerIndex/3,layerIndex%3);
        ++layerIndex;
        const QString key=QString::fromLatin1(layer.name);
        connect(ck,&QCheckBox::toggled,this,[this,key](bool visible){
            if(m_view)m_view->setTwinOverlayVisible(key,visible);
        });
    }
    pv->addLayout(layers);
    box->addWidget(previewBox);
    auto* actions=new QHBoxLayout;
    m_btnPlan=new QPushButton(tr("规划五轴轨迹 + 逐姿态光学检查（离线）"),this);
    m_btnExport=new QPushButton(tr("导出 H1/H3 NC + 清单"),this);
    m_btnSim=new QPushButton(tr("输出模拟 SCAN→M3/M4"),this);
    m_btnCancel=new QPushButton(tr("取消后台"),this);
    m_btnCancel->setEnabled(false);m_btnExport->setEnabled(false);m_btnSim->setEnabled(false);
    actions->addWidget(m_btnPlan);actions->addWidget(m_btnExport);box->addLayout(actions);
    auto* ncOptions=new QHBoxLayout;
    m_previewTopology=new QComboBox(this);m_previewTopology->addItem(tr("H3：传感器主动触发"),3);
    m_previewTopology->addItem(tr("H1：控制器 PWM 触发"),1);
    m_btnPreviewNc=new QPushButton(tr("查看所选姿态 / 连续会话 NC"),this);
    m_btnImportNc=new QPushButton(tr("导入 NC 文件并逐段校验"),this);
    m_btnPreviewNc->setEnabled(false);
    m_btnImportNc->setEnabled(false);
    ncOptions->addWidget(m_previewTopology);ncOptions->addWidget(m_btnPreviewNc);
    ncOptions->addWidget(m_btnImportNc);
    box->addLayout(ncOptions);
    m_ncText=new QPlainTextEdit(this);m_ncText->setReadOnly(true);
    m_ncText->setPlaceholderText(tr("生成规划后，选择姿态和 H1/H3，查看离线 G05/G93 + M50/M51 NC 文本片段。"));
    m_ncText->setMaximumHeight(160);box->addWidget(m_ncText);
    // Virtual NC interpretation only: uses shared NC parser and no motor I/O.
    auto* simGroup=new QGroupBox(tr("5 G代码离线执行仿真 / 虚拟扫描 / 实测对比"),this);
    auto* simLayout=new QVBoxLayout(simGroup);
    auto* playbackActions=new QHBoxLayout;
    m_btnPlay=new QPushButton(tr("按已解析 G代码播放五轴+传感器"),simGroup);
    m_btnPause=new QPushButton(tr("暂停/继续"),simGroup);
    m_btnStep=new QPushButton(tr("单步"),simGroup);
    m_btnStop=new QPushButton(tr("停止"),simGroup);
    m_btnCompare=new QPushButton(tr("导入所选姿态实测 .scan 并对比"),simGroup);
    m_btnPlay->setEnabled(false);m_btnPause->setEnabled(false);m_btnStep->setEnabled(false);
    m_btnStop->setEnabled(false);m_btnCompare->setEnabled(false);
    m_playSpeed=new QComboBox(simGroup);
    m_playSpeed->addItem(tr("1×"),1.0);m_playSpeed->addItem(tr("5×"),5.0);
    m_playSpeed->addItem(tr("20×"),20.0);
    playbackActions->addWidget(m_btnPlay);playbackActions->addWidget(m_btnPause);
    playbackActions->addWidget(m_btnStep);
    playbackActions->addWidget(m_btnStop);playbackActions->addWidget(m_playSpeed);
    simLayout->addLayout(playbackActions);
    m_playSeek=new QSlider(Qt::Horizontal,simGroup);
    m_playSeek->setRange(0,1000);m_playSeek->setValue(0);
    simLayout->addWidget(m_playSeek);
    m_playStatus=new QLabel(tr("先生成并校验 G代码；回放时间轴、XYZCB 和扫描开关全部来自 NC 文本解析，绝不控制机床。"),simGroup);
    m_playStatus->setWordWrap(true);simLayout->addWidget(m_playStatus);
    auto* actions2=new QHBoxLayout;
    m_btnSim->setText(tr("执行 G代码并导出模拟 .scan + PLY"));
    actions2->addWidget(m_btnSim);actions2->addWidget(m_btnCompare);
    actions2->addWidget(m_btnCancel);simLayout->addLayout(actions2);
    box->addWidget(simGroup);
    m_playTimer=new QTimer(this);m_playTimer->setInterval(80);
    connect(m_playTimer,&QTimer::timeout,this,&MultiPoseExperimentWidget::playbackTick);
    connect(m_btnPlay,&QPushButton::clicked,this,&MultiPoseExperimentWidget::startPlayback);
    connect(m_btnPause,&QPushButton::clicked,this,&MultiPoseExperimentWidget::togglePlaybackPause);
    connect(m_btnStep,&QPushButton::clicked,this,&MultiPoseExperimentWidget::stepPlayback);
    connect(m_btnStop,&QPushButton::clicked,this,[this]{stopPlayback();});
    connect(m_btnCompare,&QPushButton::clicked,this,&MultiPoseExperimentWidget::startCompare);
    connect(m_playSeek,&QSlider::sliderReleased,this,[this]{seekPlayback(m_playSeek->value());});
    connect(m_btnPlan,&QPushButton::clicked,this,&MultiPoseExperimentWidget::startPlan);
    connect(m_btnExport,&QPushButton::clicked,this,&MultiPoseExperimentWidget::exportNc);
    connect(m_btnPreviewNc,&QPushButton::clicked,this,&MultiPoseExperimentWidget::previewNc);
    connect(m_btnImportNc,&QPushButton::clicked,this,&MultiPoseExperimentWidget::importNcForPose);
    connect(m_previewTopology,qOverload<int>(&QComboBox::currentIndexChanged),this,[this](int){
        ++m_recipeRevision; // in-flight simulations of the old topology must be discarded
        if(m_cancel && m_simWatcher.isRunning())m_cancel->store(true);
        if(m_report){
            if(!m_overriddenNcByPose.empty()){
                m_overriddenNcByPose.clear();
                log(tr("切换 H1/H3，已清除另一同步拓扑下导入的 NC 文件，请重新选择对应文件。"));
            }
            rebuildNcReplays();
        }
    });
    connect(m_btnSim,&QPushButton::clicked,this,&MultiPoseExperimentWidget::startFullSimulation);
    connect(m_btnCancel,&QPushButton::clicked,this,&MultiPoseExperimentWidget::cancelTask);
    m_progress=new QProgressBar(this);m_progress->setRange(0,100);m_progress->setValue(0);box->addWidget(m_progress);
    m_progressTimer=new QTimer(this);
    m_progressTimer->setInterval(200);
    connect(m_progressTimer,&QTimer::timeout,this,[this]{
        if(!m_simWatcher.isRunning()||!m_simProgress||m_simTotalProfiles==0)return;
        const auto finished=m_simProgress->load();
        const int percent=std::min(99,int(100.0*double(finished)/double(m_simTotalProfiles)));
        m_progress->setValue(percent);
    });
    m_summary=new QLabel(tr("未规划"),this);m_summary->setWordWrap(true);box->addWidget(m_summary);
    m_log=new QPlainTextEdit(this);m_log->setReadOnly(true);m_log->setMaximumHeight(135);box->addWidget(m_log);
    box->addStretch(1);
    // Startup fix: addOrientation() invalidates the recipe, and invalidate()
    // touches m_btnExport/m_btnSim/m_btnPreviewNc/m_summary.  Those widgets
    // must exist before the first orientation row is added.  The old code
    // inserted rows above while those pointers were still null, crashing
    // before MainWindow was shown on Windows.
    restoreStandardMatrix();
    log(tr("已载入标准37组B/C实验矩阵（6×6+0/0）；扫描默认机床 X+→X−，前后各预留5mm，并按各姿态 M3 投影自动增加斜扫补偿。全部结果仅供离线研究。"));
}
void MultiPoseExperimentWidget::invalidate(bool placementChanged){
    ++m_recipeRevision;
    stopPlayback(true);
    m_simArtifacts.reset();
    m_activeNcTexts.clear();m_activeNcReplays.clear();m_overriddenNcByPose.clear();
    m_connectedNcUsable=false;m_connectedNcText.clear();m_connectedNcReplay={};
    m_overriddenConnectedNcByTopology.clear();
    if(m_btnImportNc)m_btnImportNc->setEnabled(false);
    if(m_btnCompare)m_btnCompare->setEnabled(false);
    if(m_view)m_view->clearTwinMultiPoseComparisonHeatmap();
    if(!m_planWatcher.isRunning()&&!m_simWatcher.isRunning()){
        if(placementChanged){
            m_previewPoseAcknowledged=false;
            m_registrationProposal.reset();
            if(m_btnApplyEstimate)m_btnApplyEstimate->setEnabled(false);
            if(m_registrationStatus)m_registrationStatus->setText(
                tr("CAD 放置/实测输入发生变化，原配准建议和离线确认已失效。"));
        }
        m_report.reset();
        // The pointers can still be null while a QWidget is being built.
        // Guard the entire invalidation path, not just the original crash site.
        if(m_btnExport)m_btnExport->setEnabled(false);
        if(m_btnSim)m_btnSim->setEnabled(false);
        if(m_btnPreviewNc)m_btnPreviewNc->setEnabled(false);
        if(m_btnPlay)m_btnPlay->setEnabled(false);
        if(m_btnStep)m_btnStep->setEnabled(false);
        if(m_ncText)m_ncText->clear();
        if(m_view)m_view->clearTwinMultiPoseResults();
        if(m_previewPose){QSignalBlocker guard(m_previewPose);
            m_previewPose->clear();m_previewPose->addItem(tr("显示全部姿态"));}
        if(m_summary)m_summary->setText(tr("输入已变化，需要重新规划和仿真。"));
    }
}
void MultiPoseExperimentWidget::log(const QString& text){
    if(m_log)m_log->appendPlainText(QDateTime::currentDateTime().toString("hh:mm:ss ")+text);
}
mls::twin::Pose6 MultiPoseExperimentWidget::uiCadPose()const{
    return {m_pose[0]->value(),m_pose[1]->value(),m_pose[2]->value(),
            m_pose[3]->value(),m_pose[4]->value(),m_pose[5]->value()};
}
void MultiPoseExperimentWidget::addOrientation(double c,double b,const QString& name){
    const int r=m_poses->rowCount();if(r>=81){log(tr("最多支持81种姿态"));return;}
    m_poses->insertRow(r);
    const QString id=name.isEmpty()?QStringLiteral("P%1").arg(r+1,2,10,QChar('0')):name;
    m_poses->setItem(r,0,new QTableWidgetItem(id));
    m_poses->setItem(r,1,new QTableWidgetItem(QString::number(c,'f',3)));
    m_poses->setItem(r,2,new QTableWidgetItem(QString::number(b,'f',3)));
    invalidate();
}
void MultiPoseExperimentWidget::restoreStandardMatrix(){
    // Deterministic B-major/C-minor order: -10..+10 step 4, plus P37 at B=C=0.
    // Preserve experiments while a computation is running; no mid-task edits.
    if(m_planWatcher.isRunning()||m_simWatcher.isRunning()){
        log(tr("任务正在运行，不能重置实验矩阵"));
        return;
    }
    {
        QSignalBlocker block(m_poses);
        m_poses->setRowCount(0);
        for(const auto& p:mls::twin::standardBCExperimentMatrix()){
            const int row=m_poses->rowCount();
            m_poses->insertRow(row);
            m_poses->setItem(row,0,new QTableWidgetItem(QString::fromStdString(p.id)));
            m_poses->setItem(row,1,new QTableWidgetItem(QString::number(p.cDeg,'f',1)));
            m_poses->setItem(row,2,new QTableWidgetItem(QString::number(p.bDeg,'f',1)));
        }
    }
    invalidate();
    log(tr("已装入37组：B/C=-10,-6,-2,2,6,10°（36组）；另有 P37：B=0°、C=0°。"));
}
void MultiPoseExperimentWidget::removeOrientation(){
    if(m_poses->currentRow()<0)return;
    m_poses->removeRow(m_poses->currentRow());invalidate();
}
void MultiPoseExperimentWidget::syncScene(bool focus){
    if(!m_view)return;
    if(m_scene.hasWorkpiece()){
        m_view->setTwinMultiPoseWorkpieceMesh(m_scene.mesh(),m_scene.cadToMachineEstimate());
        if(focus)m_view->focusTwinMultiPoseWorkpiece();
    }
    if(m_real&&m_real->ok()){
        m_view->setTwinMultiPoseRecordedPoints(m_real->previewPoints);
    }
}
bool MultiPoseExperimentWidget::loadStep(const QString& path){
    if(!mls::twin::CadGeometryAdapter::stepSupported()){
        QMessageBox::warning(this,tr("STEP 导入不可用"),tr("当前 Windows 构建未链接 Open CASCADE，需重配 OCCT；没有以文本方式假装 STEP 已解析。"));return false;
    }
    QTemporaryDir staging(QDir(QDir::tempPath()).filePath("MLS_MPOSE_STEP_XXXXXX"));
    if(!staging.isValid())return false;
    const QString temp=QDir(staging.path()).filePath("workpiece.step");
    if(!QFile::copy(path,temp))return false;
    QApplication::setOverrideCursor(Qt::WaitCursor);
    auto result=mls::twin::CadGeometryAdapter::loadStep(nativeFile(temp));
    QApplication::restoreOverrideCursor();
    if(!result.ok){QMessageBox::warning(this,tr("STEP 错误"),QString::fromLocal8Bit(result.message.c_str()));return false;}
    result.mesh.source=nativeFile(path);
    mls::twin::TwinScene scene;
    std::string err;
    if(!scene.setWorkpiece(std::move(result.mesh),&err))return false;
    scene.setEstimatedPose(uiCadPose());
    m_scene=std::move(scene);m_cadFile=QFileInfo(path).absoluteFilePath();
    // Bind later FCL checks to the imported bytes, not an editable CAD path.
    QFile original(m_cadFile);
    if(!original.open(QIODevice::ReadOnly)){
        m_scene.clear();m_cadFile.clear();m_cadSha256.clear();return false;
    }
    QCryptographicHash hash(QCryptographicHash::Sha256);
    if(!hash.addData(&original)){
        m_scene.clear();m_cadFile.clear();m_cadSha256.clear();return false;
    }
    m_cadSha256=hash.result();
    const auto b=m_scene.mesh().bounds;
    // For complete-part inspection the default ROI must NOT crop the CAD
    // bounding box by 1 mm at both ends (the old code omitted real edges).
    std::array<double,5> v{{b.min.x,b.max.x,b.min.y,b.max.y,b.max.z}};
    for(int j=0;j<5;++j){QSignalBlocker blocker(m_roi[j]);m_roi[j]->setValue(v[j]);}
    invalidate(true);syncScene(true);
    m_inputs->setText(tr("STEP：%1\nSCAN：%2").arg(m_cadFile,m_real?m_real->sourcePath:tr("尚未载入")));
    log(tr("STEP B-Rep 已导入，三角面=%1；ROI 默认覆盖完整 CAD XY 包围盒（不再裁去边缘）；Z 为名义最高面，异形曲面需逐姿态光学复核。")
        .arg(static_cast<qulonglong>(m_scene.mesh().triangles.size())));
    return true;
}
void MultiPoseExperimentWidget::importStep(){
    if(m_planWatcher.isRunning()||m_simWatcher.isRunning())return;
    const auto file=QFileDialog::getOpenFileName(this,tr("导入研究零件 STEP"),QString(),tr("STEP (*.step *.stp *.STEP *.STP)"));
    if(!file.isEmpty())loadStep(file);
}
void MultiPoseExperimentWidget::importRealScan(){
    if(m_realWatcher.isRunning()||m_planWatcher.isRunning()||m_simWatcher.isRunning())return;
    const auto file=QFileDialog::getOpenFileName(this,tr("导入真实 SCAN v3"),QString(),tr("真实 SCAN (*.scan)"));
    if(file.isEmpty())return;
    m_real.reset();invalidate(true);
    m_btnPlan->setEnabled(false);m_progress->setRange(0,0);
    log(tr("后台读取 SCAN v3 和 CRC：")+file);
    m_realWatcher.setFuture(QtConcurrent::run([file]{return mls::twin::ScanReplayAdapter::load(file,90000);}));
}
void MultiPoseExperimentWidget::scanLoaded(){
    m_btnPlan->setEnabled(true);m_progress->setRange(0,100);m_progress->setValue(0);
    const auto result=m_realWatcher.result();
    if(!result||!result->ok()){
        QMessageBox::warning(this,tr("SCAN 导入失败"),result?result->error:tr("没有数据"));return;
    }
    m_real=result;invalidate(true);syncScene();
    const auto first=m_real->scan.profiles.first().fiveAxis.frame.joints;
    // Do not overwrite the controlled B/C 37-case experiment matrix with
    // historical .scan C/B! Recorded axes are observations, not a recipe.
    m_inputs->setText(tr("STEP：%1\n实测 SCAN：%2，%3 条 Profile（历史 C/B：%4°, %5°）")
        .arg(m_cadFile.isEmpty()?tr("尚未载入"):m_cadFile,m_real->sourcePath)
        .arg(static_cast<qlonglong>(m_real->scan.profiles.size())).arg(first[3],0,'f',3).arg(first[4],0,'f',3));
    log(tr("真实 SCAN 读取完成（历史记录不等于当前机床位置）；Profile=%1，保留原始 XYZCB 与 Physical17。")
        .arg(static_cast<qlonglong>(m_real->scan.profiles.size())));
}
void MultiPoseExperimentWidget::loadTwinSession(){
    if(m_planWatcher.isRunning()||m_simWatcher.isRunning())return;
    const auto file=QFileDialog::getOpenFileName(this,tr("读取已有 M0–M8 场景"),QString(),tr("场景 (*.twin.json *.json)"));
    if(file.isEmpty())return;
    QFile f(file);if(!f.open(QIODevice::ReadOnly))return;
    const auto o=QJsonDocument::fromJson(f.readAll()).object();
    if(o.value("format").toString()!=QStringLiteral("MotionLaserSync.DigitalTwinScene")||
       o.value("unit").toString()!=QStringLiteral("mm")||
       o.value("rotationConvention").toString()!=QStringLiteral("Rz*Ry*Rx_degrees")||o.value("version").toInt()!=1){
        QMessageBox::warning(this,tr("格式错误"),tr("只能读原项目 DigitalTwinScene v1 场景"));return;
    }
    const auto p=o.value("poseEstimate").toObject();
    const char* keys[]={"x","y","z","rxDeg","ryDeg","rzDeg"};
    for(const auto key:keys)if(!p.value(QLatin1String(key)).isDouble()){
        QMessageBox::warning(this,tr("格式错误"),tr("poseEstimate 六自由度字段缺失"));return;
    }
    auto path=o.value("cadPath").toString();
    if(QDir::isRelativePath(path))path=QDir(QFileInfo(file).absolutePath()).filePath(path);
    if(o.value("sourceMode").toString()==QStringLiteral("STEP")&&!loadStep(path))return;
    if(o.value("sourceMode").toString()!=QStringLiteral("STEP")){
        QMessageBox::warning(this,tr("未导入"),tr("该版本实验界面要求 STEP 原始 CAD；旧 OBJ 场景未自动加载。"));return;
    }
    for(int i=0;i<6;++i){QSignalBlocker blocker(m_pose[i]);m_pose[i]->setValue(p.value(QLatin1String(keys[i])).toDouble());}
    m_scene.setEstimatedPose(uiCadPose());syncScene(true);invalidate(true);
    log(tr("已读取旧 M0–M8 的 poseEstimate；这是共享坐标输入，不代表配准已验收。"));
}
void MultiPoseExperimentWidget::startRoughRegistration(){
    if(m_regWatcher.isRunning()||m_realWatcher.isRunning()||m_planWatcher.isRunning()||m_simWatcher.isRunning())return;
    if(!m_scene.hasWorkpiece()||!m_real||!m_real->ok()){
        QMessageBox::information(this,tr("配准输入不足"),tr("请先导入实际零件 STEP 和对应的真实 SCAN v3。"));return;
    }
    // Only a bounded VIEW sample is copied. Never transform or overwrite SCAN raw profiles.
    std::vector<mls::twin::Vec3> sample;sample.reserve(std::size_t(m_real->previewPoints.size()));
    for(const auto& p:m_real->previewPoints) sample.push_back({p.x(),p.y(),p.z()});
    const auto cad=m_scene.mesh();
    m_registrationRevision=m_recipeRevision;
    m_registrationProposal.reset();m_previewPoseAcknowledged=false;
    m_btnApplyEstimate->setEnabled(false);m_btnEstimate->setEnabled(false);
    m_registrationStatus->setText(tr("正在从实测点云与 CAD 顶面计算四种水平旋转粗配准候选……"));
    log(tr("粗配准候选后台计算开始。仅计算初位姿，不构成自动六自由度配准验收。"));
    m_regWatcher.setFuture(QtConcurrent::run([cad,sample=std::move(sample)]{
        return std::make_shared<mls::twin::RegistrationAssistReport>(
            mls::twin::proposeTopSurfaceRegistration(cad,sample,2000));
    }));
}
void MultiPoseExperimentWidget::roughRegistrationFinished(){
    m_btnEstimate->setEnabled(true);
    if(m_registrationRevision!=m_recipeRevision){
        m_registrationStatus->setText(tr("粗配准期间输入已改变，请重新计算。"));return;
    }
    const auto candidate=m_regWatcher.result();
    if(!candidate||!candidate->proposedAvailable){
        const QString reason=candidate?QString::fromStdString(candidate->message):tr("计算无结果");
        m_registrationStatus->setText(tr("无法给出可信粗配准候选：")+reason);
        log(tr("粗配准候选拒绝：")+reason);
        QMessageBox::warning(this,tr("粗配准未通过初筛"),tr("无法提议零件位姿。\n%1\n"
            "原因可能是预扫包含夹具、没有完整顶面、零件倾斜、CAD 不匹配或数据覆盖不足。\n"
            "请用可靠特征确定粗位姿，不要直接用 XYZ/RxRyRz=0 规划。").arg(reason));
        return;
    }
    m_registrationProposal=candidate;m_btnApplyEstimate->setEnabled(true);
    const auto& p=candidate->proposed;
    const QString text=tr("粗配准建议（尚未应用，更不是已验证配准）：\n"
        "XYZ=(%1,%2,%3) mm，Rx/Ry/Rz=(%4,%5,%6)°；\n"
        "顶面采样=%7，1mm 粗邻近比例=%8%（非配准准确率），中位近似距离=%9mm，朝向可能歧义=%10。")
        .arg(p.x,0,'f',3).arg(p.y,0,'f',3).arg(p.z,0,'f',3)
        .arg(p.rxDeg,0,'f',2).arg(p.ryDeg,0,'f',2).arg(p.rzDeg,0,'f',2)
        .arg(static_cast<qulonglong>(candidate->observedSamples))
        .arg(100.*candidate->fractionWithinOneMm,0,'f',1)
        .arg(candidate->medianSurfaceDistanceMm,0,'f',3)
        .arg(candidate->orientationAmbiguous?tr("是，需要核对孔槽方向"):tr("未明显检测到"));
    m_registrationStatus->setText(text);
    log(tr("粗配准已提议，等待人工复核。%1").arg(text));
}
void MultiPoseExperimentWidget::applyRoughRegistration(){
    if(!m_registrationProposal||!m_registrationProposal->proposedAvailable)return;
    const auto& candidate=*m_registrationProposal;
    if(QMessageBox::warning(this,tr("仅应用离线粗位姿"),
        tr("该估计仅基于近水平顶面和有限取样。未完成六自由度精配准、孔槽方向辨识或独立误差验收。\n"
           "接受后只能用于离线规划研究，NC 继续 OfflineOnly，禁止下发机床。\n"
           "请先对照三维窗口人工确认 CAD 边界和孔位方向。是否应用候选？"),
        QMessageBox::Yes|QMessageBox::No,QMessageBox::No)!=QMessageBox::Yes)return;
    const auto p=candidate.proposed;
    const double values[]{p.x,p.y,p.z,p.rxDeg,p.ryDeg,p.rzDeg};
    for(int k=0;k<6;++k){QSignalBlocker block(m_pose[k]);m_pose[k]->setValue(values[k]);}
    m_scene.setEstimatedPose(uiCadPose());invalidate(true);syncScene(true);
    m_previewPoseAcknowledged=true;
    m_registrationStatus->setText(tr("已应用粗配准候选，用于离线轨迹/光学检查。\n"
        "未完成精配准与验收，绝对禁止用于实际运动。"));
    log(tr("已将粗配准初位姿应用于 CAD 场景（仅离线，registrationVerified=false）。"));
}
void MultiPoseExperimentWidget::acceptManualPreviewPose(){
    if(!m_scene.hasWorkpiece()||!m_real||!m_real->ok()){
        QMessageBox::information(this,tr("缺少数据"),tr("请先导入 STEP 和真实 SCAN。"));return;
    }
    if(QMessageBox::warning(this,tr("未验证人工位姿"),tr("您正在使用手工输入或旧 M0 场景的初位姿，"
            "目前没有完成自动配准。继续只允许离线规划，所有 G 代码均不可执行。\n"
            "请确认三维窗口 CAD 与实测点云有合理重叠。"),
            QMessageBox::Yes|QMessageBox::No,QMessageBox::No)!=QMessageBox::Yes)return;
    m_previewPoseAcknowledged=true;
    m_registrationStatus->setText(tr("人工已确认此初位姿仅供离线规划；配准/实机安全验收仍未通过。"));
    log(tr("已确认人工初位姿仅作离线仿真。"));
}
bool MultiPoseExperimentWidget::readRecipe(mls::twin::ExperimentOptions& o,
    std::vector<mls::twin::ExperimentPose>& views,QString* error) const{
    if(!m_scene.hasWorkpiece()||!m_real||!m_real->ok()){
        if(error)*error=tr("请先导入 STEP 与同零件的实测 SCAN v3");return false;
    }
    o.roiCad.min={m_roi[0]->value(),m_roi[2]->value(),m_roi[4]->value()};
    o.roiCad.max={m_roi[1]->value(),m_roi[3]->value(),m_roi[4]->value()};
    o.surfaceZCadMm=m_roi[4]->value();
    o.direction=m_axis->currentIndex()==0
        ?mls::twin::ExperimentSweepAxis::AutoMachineX
        :(m_axis->currentIndex()==1
            ?mls::twin::ExperimentSweepAxis::CadY
            :mls::twin::ExperimentSweepAxis::CadX);
    // Generate scan G93 from the real five-axis SP acceleration envelope,
    // not by assuming that sampled waypoints can be traversed at zero ramp time.
    o.enforceHardwareG93=true;
    o.hardwareG93Limits=mls::runtime::Limits{}; // current V30 default; execution re-reads live caps
    o.hardwareVectorAccelCap=25.;o.hardwareVectorDecelCap=25.;
    o.scanSpeedMmPerSec=m_speed->value();o.transferSpeedMmPerSec=m_transferSpeed->value();
    o.profileRateHz=m_rate->value();o.lanePitchMm=m_pitch->value();
    o.retractFromSurfaceCadMm=0.; // explicit: no automatic entry/exit Z move
    o.focusSensorZmm=m_focus->value();
    o.leadInMm=m_leadIn->value();
    o.leadOutMm=m_leadOut->value();
    o.maximumProfiles=std::size_t(m_maxProfiles->value());
    // Historic .scan pose is used ONLY as an offline trace anchor, never live MPOS.
    o.hasArchivedInitialJoints=true;
    o.archivedInitialJoints=m_real->scan.profiles.first().fiveAxis.frame.joints;
    if(m_poses->rowCount()==0){if(error)*error=tr("至少需要一个姿态");return false;}
    for(int row=0;row<m_poses->rowCount();++row){
        bool cOk=false,bOk=false;
        const auto* id=m_poses->item(row,0);
        const auto* c=m_poses->item(row,1);
        const auto* b=m_poses->item(row,2);
        if(!id||!c||!b){if(error)*error=tr("姿态表存在空列");return false;}
        const double cv=c->text().toDouble(&cOk),bv=b->text().toDouble(&bOk);
        if(!cOk||!bOk){if(error)*error=tr("姿态 C/B 必须是数字");return false;}
        views.push_back({id->text().trimmed().toStdString(),cv,bv});
    }
    return true;
}
void MultiPoseExperimentWidget::startPlan(){
    if(m_planWatcher.isRunning()||m_realWatcher.isRunning()||m_regWatcher.isRunning()||m_simWatcher.isRunning())return;
    if(!m_previewPoseAcknowledged){
        QMessageBox::warning(this,tr("必须先检查工件位姿"),tr("目前只有导入数据，没有完成配准检查。\n"
            "请先使用‘从 STEP + .scan 提议粗配准’，检查叠加效果后应用候选；\n"
            "或对已知的手工初位姿明确选择‘只对手工位姿做离线预览’。\n"
            "不允许直接以 XYZ/RxRyRz=0 的默认位姿生成五轴运动。"));
        return;
    }
    mls::twin::ExperimentOptions opts;
    std::vector<mls::twin::ExperimentPose> views;
    QString error;
    if(!readRecipe(opts,views,&error)){
        QMessageBox::information(this,tr("输入不完整"),error);return;
    }
    invalidate();m_planningRevision=m_recipeRevision;m_scene.setEstimatedPose(uiCadPose());
    const auto scene=m_scene;const auto real=m_real;
    const mls::twin::Sensor2950Geometry optics{};
    const auto a=QDir(QCoreApplication::applicationDirPath()).filePath("models/machine5x/machine_config.json");
    const auto b=QDir::current().filePath("models/machine5x/machine_config.json");
    const auto config=QFileInfo::exists(a)?a:(QFileInfo::exists(b)?b:QString());
    m_cancel=std::make_shared<std::atomic<bool>>(false);
    const auto stop=m_cancel;
    m_btnPlan->setEnabled(false);m_btnCancel->setEnabled(true);m_progress->setRange(0,0);
    log(tr("多姿态实验离线规划与 M3 光学仿真开始；姿态数=%1，机床 FCL=%2。")
        .arg(static_cast<qulonglong>(views.size())).arg(config.isEmpty()?tr("未加载（只做离线预览）"):tr("要求检验")));
    m_planWatcher.setFuture(QtConcurrent::run([scene,real,views,opts,optics,config,stop]{
        return mls::twin::MultiPoseExperimentQtAdapter::planAndSimulatePreview(
            scene,*real,views,opts,optics,config,stop.get());
    }));
}
void MultiPoseExperimentWidget::planFinished(){
    m_progress->setRange(0,100);m_progress->setValue(100);
    m_btnPlan->setEnabled(true);m_btnCancel->setEnabled(false);
    const auto d=m_planWatcher.result();
    if(m_recipeRevision!=m_planningRevision){
        m_report.reset();m_btnExport->setEnabled(false);m_btnSim->setEnabled(false);
        m_btnPreviewNc->setEnabled(false);
        if(m_ncText)m_ncText->clear();
        if(m_view)m_view->clearTwinMultiPoseResults();
        m_summary->setText(tr("规划中输入参数发生变化，旧规划已丢弃。请重新生成。"));
        return;
    }
    if(!d||!d->ok()){
        const auto status=d?d->status:tr("未知错误");
        QString guidance;
        if(status.contains("M3 光学模型初始化失败"))guidance=tr("\n阶段：轨迹规划已经通过，失败发生在 M3 光学模型构建。需核对 CAD 三角网格、光线几何和传感器模型。");
        else if(status.contains("outside configured planning bounds"))guidance=tr(
            "\n原因：当前 B/C 姿态在 Physical17 运动学下需要的轴位置超出机器配置的安全规划区间。"
            "请先查看报错列出的具体轴、请求值和允许区间；调整工件安装位置、可用 ROI 或经确认的光学工作距离，"
            "不要通过扩大限位或删除安全余量强制通过。已取消多余的 Z 进退刀动作。");
        else if(status.contains("collision"))guidance=tr("\n原因：机床碰撞/间隙检查拒绝路径，请检查装夹与路径；不要关闭碰撞检查强制通过。");
        else if(status.contains("basis singular")||status.contains("FK/IK"))guidance=tr("\n原因：Physical17 对应姿态的逆解失败，检查标定及旋转角度。");
        else if(status.contains("ROI"))guidance=tr("\n原因：指定的 CAD 局部扫描区域不在零件有效范围内。");
        else if(status.contains("projected optical fan")||status.contains("tilted laser footprint"))
            guidance=tr("\n原因：当前 B/C 姿态下投影激光线不能覆盖 ROI 横向范围，不能仅凭端点命中生成 NC。请检查光学校准、姿态与扫描方向。");
        else if(status.contains("profile limit"))
            guidance=tr("\n原因：37 组姿态与斜投影安全余量增加了 Profile 数，请检查采集预算，不得自动稀释轮廓频率。");
        m_summary->setText(tr("轨迹规划阶段失败：")+status+guidance);
        log(tr("轨迹规划阶段失败：")+status+guidance);
        QMessageBox::warning(this,tr("多姿态轨迹规划失败"),status+guidance);
        return;
    }
    m_report=d;
    // The planner is NOT the playback source. Only parsed/checked NC text can
    // create the executable digital-twin time axis and enable these controls.
    const bool ncReady=rebuildNcReplays(true);
    if(!ncReady)log(tr("五轴规划已得到，但 NC 执行级仿真未通过；播放与导出保持禁用。"));
    {QSignalBlocker guard(m_previewPose);
     m_previewPose->clear();m_previewPose->addItem(tr("显示全部姿态"));
     for(const auto& view:d->plan.views)m_previewPose->addItem(QString::fromStdString(view.id));}
    if(m_view)m_view->setTwinMultiPosePoseSelection(-1);
    showResult();
    if(!d->opticalPreviewUsable)
        QMessageBox::warning(this,tr("轨迹成功，但光学预览失败"),
           tr("五轴轨迹已经生成，但至少一个目标姿态在 ROI 的二维覆盖网格、边界支持或激光命中检查中未通过。\n"
              "一条倾斜的激光线触及零件末端，并不意味着整个 ROI 已扫完。请检查 STEP/SCAN 配准、Z 参考、传感器扇面、B/C 和扫描方向。\n"
              "这不是可执行的扫描路径；NC 和模拟 SCAN 已被禁止导出。"));
    log(tr("导入数据身份已建立：PartPoseId=%1；CalibrationId=%2；模型摘要=%3。"
           "这是源数据版本绑定，不是独立配准/计量合格签署。")
        .arg(d->partPoseId,d->calibrationId,d->physical17Sha256));
    log(tr("多姿态实验规划成功（仅离线）：%1 个姿态，FCL=%2，BC 实扫激励=%3")
        .arg(static_cast<qulonglong>(d->plan.views.size())).arg(d->machineCollisionModelLoaded?tr("加载"):tr("未加载"))
        .arg(d->rotaryReplayExcited?tr("有"):tr("无")));
}
void MultiPoseExperimentWidget::showResult(){
    if(!m_report||!m_report->ok())return;
    QString summary=tr("独立多姿态实验 | 非 M0–M8 全覆盖：\n")+m_report->kinematicsSummary+"\n";
    summary+=tr("本机导入候选标识（非现场签署）：\nPartPoseId=%1\nCalibrationId=%2\nPhysical17Sha256=%3\n")
        .arg(m_report->partPoseId,m_report->calibrationId,m_report->physical17Sha256);
    for(std::size_t i=0;i<m_report->plan.views.size();++i){
        const auto& v=m_report->plan.views[i];
        summary+=tr("%1: B=%2° C=%3° | X+→X− | 预/后延伸 %4/%5 mm | 扫描段 %6 | Profiles %7 | ROI网格 %8% | 边缘 %9% | 光学命中 %10/%11\n")
            .arg(QString::fromStdString(v.id))
            .arg(v.bDeg,0,'f',1).arg(v.cDeg,0,'f',1)
            .arg(v.beforePartMarginMm,0,'f',1).arg(v.afterPartMarginMm,0,'f',1)
            .arg(static_cast<qulonglong>(v.scanGroups))
            .arg(static_cast<qulonglong>(v.plannedProfiles))
            .arg(m_report->previewRoiCoverageFractions.at(qsizetype(i))*100.,0,'f',1)
            .arg(m_report->previewBoundaryCoverageFractions.at(qsizetype(i))*100.,0,'f',1)
            .arg(m_report->previewProfilesWithWorkpieceHit.at(qsizetype(i)))
            .arg(static_cast<qulonglong>(v.opticalPreviewPoses.size()));
        if(m_report->previewRoiCoverageFractions.at(qsizetype(i))<0.95 ||
           m_report->previewBoundaryCoverageFractions.at(qsizetype(i))<0.95){
            summary+=tr("    此姿态 ROI/边界覆盖不足：%1\n")
                .arg(m_report->previewCoverageWarnings.at(qsizetype(i)));
        }
    }
    if(!m_report->opticalPreviewUsable)
        summary+=tr("\n[覆盖校验未通过] 至少一个姿态在 CAD 预计表面 ROI 网格、边缘覆盖或光学命中方面不足。"
                    "可检查轨迹，但禁止导出 NC 或合成 SCAN。请核查配准、传感器姿态与光学模型。\n");
    summary+=m_report->warning;
    m_summary->setText(summary);
    if(m_view){
        m_view->setTwinMultiPoseResults(m_report->virtualPointPreviews,m_report->scanFocusPaths,
                                         m_report->transferFocusPath);
        m_view->focusTwinMultiPoseWorkpiece();
    }
}
bool MultiPoseExperimentWidget::rebuildNcReplays(bool notify){
    stopPlayback(true);
    m_activeNcTexts.clear();m_activeNcReplays.clear();
    m_playbackTimeline={};
    m_simArtifacts.reset();
    if(m_btnCompare)m_btnCompare->setEnabled(false);
    if(m_view)m_view->clearTwinMultiPoseComparisonHeatmap();
    if(!m_report||!m_report->ok())return false;
    const int topo=m_previewTopology?m_previewTopology->currentData().toInt():3;
    const int edge=m_h3Edge?m_h3Edge->currentData().toInt():115;
    std::vector<std::string> texts;
    std::vector<mls::twin::NcSimResult> executions;
    for(std::size_t k=0;k<m_report->plan.views.size();++k){
        const auto overrideIt=m_overriddenNcByPose.find(k);
        const std::string* exact=overrideIt==m_overriddenNcByPose.end()?nullptr:&overrideIt->second;
        const auto run=mls::twin::MultiPoseExperimentQtAdapter::executeNcForPose(*m_report,k,topo,edge,exact);
        if(!run.ok()){
            const auto why=tr("姿态 %1 的 G代码执行仿真检查失败：%2")
                .arg(QString::fromStdString(m_report->plan.views[k].id)).arg(run.error);
            if(m_playStatus)m_playStatus->setText(why);
            if(m_ncText)m_ncText->setPlainText(why);
            if(notify)QMessageBox::warning(this,tr("NC 指令检查未通过"),why);
            if(m_btnPlay)m_btnPlay->setEnabled(false);
            if(m_btnStep)m_btnStep->setEnabled(false);
            if(m_btnSim)m_btnSim->setEnabled(false);
            if(m_btnExport)m_btnExport->setEnabled(false);
            return false;
        }
        texts.push_back(run.text);
        executions.push_back(run.execution);
    }
    // Independently generated P01..P37 files have separate first G00 anchors.
    // They MUST NOT be spliced into a moving five-axis animation. Construct
    // a separately saved and verified continuous NC with real G05 S0 joints.
    m_connectedNcUsable=false;m_connectedNcReplay={};m_connectedNcText.clear();
    auto timeline=mls::twin::makeNcDrivenPlaybackTimeline(executions);
    if(!timeline.ok()){
        if(notify)QMessageBox::warning(this,tr("NC 回放时间轴错误"),QString::fromStdString(timeline.error));
        return false;
    }
    const auto chosen=m_overriddenConnectedNcByTopology.find(topo);
    const std::string* savedText=chosen==m_overriddenConnectedNcByTopology.end()?nullptr:&chosen->second;
    const auto merged=mls::twin::MultiPoseExperimentQtAdapter::executeConnectedNc(
        *m_report,topo,edge,savedText);
    if(merged.ok()){
        const auto continuous=mls::twin::makeConnectedNcPlaybackTimeline(
            m_report->connectedSession,merged.execution);
        if(continuous.ok()){
            m_connectedNcUsable=true;
            m_connectedNcText=merged.text;
            m_connectedNcReplay=merged.execution;
            timeline=continuous;
        }
    }
    if(!m_connectedNcUsable){
        const auto why=merged.error.isEmpty()?m_report->continuousSessionIssue:merged.error;
        log(tr("不能连续播放全部姿态：缺少经过验证的姿态间转场 NC。请从姿态选择框中单独播放。原因：")+why);
    }else{
        log(tr("已生成并逐段核验连续实验 NC：姿态间包含有限时间 G05 S0 插补；总时长=%1s，激光在转场时关闭。")
            .arg(timeline.totalSeconds,0,'f',2));
    }
    m_activeNcTexts=std::move(texts);m_activeNcReplays=std::move(executions);
    m_playbackTimeline=std::move(timeline);
    m_playSeconds=0.;m_playLimitSeconds=m_playbackTimeline.totalSeconds;
    m_nextPlaybackFrame.assign(m_activeNcReplays.size(),0);
    if(m_btnPlay)m_btnPlay->setEnabled(true);
    if(m_btnStep)m_btnStep->setEnabled(true);
    if(m_btnSim)m_btnSim->setEnabled(m_report->opticalPreviewUsable&&!m_simWatcher.isRunning());
    if(m_btnExport)m_btnExport->setEnabled(m_report->opticalPreviewUsable);
    if(m_btnPreviewNc)m_btnPreviewNc->setEnabled(true);
    if(m_btnImportNc)m_btnImportNc->setEnabled(true);
    if(m_ncText)m_ncText->clear();
    std::size_t count=0,groups=0;
    for(const auto& run:m_activeNcReplays){count+=run.profiles.size();groups+=run.scanGroups;}
    if(m_playStatus)m_playStatus->setText(tr("已对 H%1 G代码文本解析并逐段比对：%2个姿态、%3个M50/M51组、%4条理想触发轮廓；%5秒。首段G00的实际起点未知。")
        .arg(topo).arg(static_cast<qulonglong>(m_activeNcReplays.size()))
        .arg(static_cast<qulonglong>(groups)).arg(static_cast<qulonglong>(count))
        .arg(m_playbackTimeline.totalSeconds,0,'f',2));
    log(tr("NC 逐段重放验证通过：H%1，姿态=%2，理想触发=%3；G05/G93 XYZCB 与规划一致。")
        .arg(topo).arg(static_cast<qulonglong>(m_activeNcReplays.size()))
        .arg(static_cast<qulonglong>(count)));
    return true;
}
void MultiPoseExperimentWidget::importNcForPose(){
    if(!m_report||!m_report->ok())return;
    if(m_simWatcher.isRunning()||m_planWatcher.isRunning()){
        QMessageBox::information(this,tr("任务仍在运行"),tr("请先结束当前仿真，再切换 NC 输入文件。"));return;
    }
    const int selected=m_previewPose?m_previewPose->currentIndex()-1:-1;
    const bool complete=selected<0;
    if(complete&&!m_report->connectedSession.ok()){
        QMessageBox::information(this,tr("连续 NC 尚未通过检查"),
            tr("没有合格的姿态间转场，不能导入多姿态连续 NC；可以单独选择 P01..P37 导入。"));
        return;
    }
    if(!complete&&std::size_t(selected)>=m_report->plan.views.size())return;
    const auto path=QFileDialog::getOpenFileName(this,
        complete?tr("导入含 G05 S0 转场的整组实验 NC"):tr("加载需要验证的单姿态 NC 文件"),
        {},tr("G 代码 (*.nc *.gcode *.tap);;所有文件 (*)"));
    if(path.isEmpty())return;
    QFile input(path);
    if(!input.open(QIODevice::ReadOnly)||input.size()>64*1024*1024){
        QMessageBox::warning(this,tr("导入失败"),tr("无法读取或 NC 超过 64MiB"));return;
    }
    const QByteArray bytes=input.readAll();
    const std::string content(bytes.constData(),std::size_t(bytes.size()));
    const int topo=m_previewTopology?m_previewTopology->currentData().toInt():3;
    if(complete){
        const auto checked=mls::twin::MultiPoseExperimentQtAdapter::executeConnectedNc(
            *m_report,topo,m_h3Edge->currentData().toInt(),&content);
        if(!checked.ok()){
            QMessageBox::warning(this,tr("整组 NC 转场/扫描校验失败"),checked.error);
            log(tr("连续 NC 拒绝：")+checked.error);return;
        }
        m_overriddenConnectedNcByTopology[topo]=content;
    }else{
        const auto checked=mls::twin::MultiPoseExperimentQtAdapter::executeNcForPose(
            *m_report,std::size_t(selected),topo,m_h3Edge->currentData().toInt(),&content);
        if(!checked.ok()){
            QMessageBox::warning(this,tr("G代码与规划不一致 / 执行失败"),checked.error);
            log(tr("NC 校验未通过：")+checked.error);return;
        }
        m_overriddenNcByPose[std::size_t(selected)]=content;
    }
    ++m_recipeRevision;
    if(!rebuildNcReplays())return;
    const auto hash=QCryptographicHash::hash(bytes,QCryptographicHash::Sha256).toHex();
    log(tr("已采用%1离线 NC：%2，SHA256=%3")
        .arg(complete?tr("连续会话"):QString::fromStdString(m_report->plan.views[std::size_t(selected)].id),
             path,QString::fromLatin1(hash)));
    previewNc();
}
void MultiPoseExperimentWidget::previewNc(){
    if(!m_report||m_activeNcTexts.empty())return;
    const int selected=m_previewPose?m_previewPose->currentIndex()-1:-1;
    if(selected<0&&!m_connectedNcUsable){
        m_ncText->setPlainText(tr("不能把多个独立 NC 当成连续执行文件。"
            "请先通过姿态间转场的轴限位与 FCL 检查，或选择一个姿态查看其单独 NC。"));
        return;
    }
    const std::size_t index=std::size_t(std::max(0,selected));
    if(index>=m_activeNcTexts.size())return;
    const QByteArray raw=QByteArray::fromStdString(
        selected<0?m_connectedNcText:m_activeNcTexts[index]);
    const auto digest=QCryptographicHash::hash(raw,QCryptographicHash::Sha256).toHex();
    const auto lines=QString::fromUtf8(raw).split('\n');
    constexpr qsizetype kDisplay=170;
    QString content;
    if(lines.size()<=kDisplay)content=lines.join('\n');
    else content=lines.mid(0,kDisplay-30).join('\n')+tr("\n... 省略 %1 行；完整内容仍用于仿真 ...\n")
       .arg(lines.size()-kDisplay)+lines.mid(lines.size()-30).join('\n');
    m_ncText->setPlainText(tr("[已被 G代码解释器执行仿真 + 与规划逐段比对；仍是 OfflineOnly，不能用于机床授权]\nSHA256=%1\n")
         .arg(QString::fromLatin1(digest))+content);
}
void MultiPoseExperimentWidget::exportNc(){
    if(!m_report||!m_report->ok())return;
    const auto parent=QFileDialog::getExistingDirectory(this,tr("选择多姿态 NC 导出父目录"));
    if(parent.isEmpty())return;
    const auto name=QStringLiteral("MultiPose_%1").arg(QDateTime::currentDateTime().toString("yyyyMMdd_hhmmsszzz"));
    const auto folder=QDir(parent).filePath(name);
    const auto error=mls::twin::MultiPoseExperimentQtAdapter::exportOfflineNcAndManifest(
        *m_report,folder,m_h3Edge->currentData().toInt(),
        &m_activeNcTexts,m_previewTopology->currentData().toInt(),
        m_connectedNcUsable?&m_connectedNcText:nullptr);
    if(!error.isEmpty()){QMessageBox::warning(this,tr("导出失败"),error);return;}
    log(tr("离线 G 代码 H1/H3 + 清单已导出：")+folder);
    QMessageBox::information(this,tr("离线预览导出完成"),
        tr("已生成 H1/H3 各37姿态独立 NC，并分别导出 H1/H3 连续 NC（包含姿态间真实 G05 S0 转场），写入可追溯清单。\n"
           "这些文件仍为 OfflineOnly。H1/H3与频率必须匹配本机参数页并核对实际电气状态，不能只改NC头部。\n"
           "第一条G00必须由真实MPOS和独立进场路径验证，不能由此导出自动批准。\n")+folder);
}
void MultiPoseExperimentWidget::startFullSimulation(){
    if(!m_report||!m_report->ok()||m_simWatcher.isRunning())return;
    const auto parent=QFileDialog::getExistingDirectory(this,tr("选择模拟 SCAN 导出父目录"));
    if(parent.isEmpty())return;
    m_simTarget=QDir(parent).filePath(QStringLiteral("M3M4_SIM_%1")
        .arg(QDateTime::currentDateTime().toString("yyyyMMdd_hhmmsszzz")));
    if(m_activeNcTexts.size()!=m_report->plan.views.size()||!m_playbackTimeline.ok()){
        QMessageBox::warning(this,tr("未完成 NC 仿真验证"),tr("请先生成并逐段校验 G代码。"));return;
    }
    const auto report=m_report;
    const auto scene=m_scene;
    const auto exactNc=m_activeNcTexts;
    const int topology=m_previewTopology->currentData().toInt();
    m_simArtifacts.reset();
    m_simRevision=m_recipeRevision;
    m_simPending=std::make_shared<mls::twin::MultiPoseSimArtifacts>();
    if(m_btnCompare)m_btnCompare->setEnabled(false);
    m_cancel=std::make_shared<std::atomic<bool>>(false);
    m_simProgress=std::make_shared<std::atomic<std::size_t>>(0);
    m_simTotalProfiles=0;
    for(const auto& execution:m_activeNcReplays)m_simTotalProfiles+=execution.profiles.size();
    const auto stop=m_cancel;
    const auto progress=m_simProgress;
    const auto artifacts=m_simPending;
    m_btnSim->setEnabled(false);m_btnCancel->setEnabled(true);
    m_progress->setRange(0,100);m_progress->setValue(0);
    m_progressTimer->start();
    log(tr("按 H%1 解析后的 NC 时间/XYZCB/M50/M51 逐帧模拟激光；输出独立 PoseTiming::Simulated 的 .scan 和 PLY。")
        .arg(topology));
    m_simWatcher.setFuture(QtConcurrent::run([report,scene,exactNc,topology,stop,progress,artifacts,path=m_simTarget]{
        return mls::twin::MultiPoseExperimentQtAdapter::exportSimulatedProfilesForM3M4(
            *report,scene,mls::twin::Sensor2950Geometry{},path,exactNc,topology,
            stop.get(),progress.get(),artifacts.get());
    }));
}
void MultiPoseExperimentWidget::simulationFinished(){
    if(m_progressTimer)m_progressTimer->stop();
    m_btnSim->setEnabled(bool(m_report)&&m_playbackTimeline.ok()&&m_report->opticalPreviewUsable);
    m_btnCancel->setEnabled(false);
    m_progress->setRange(0,100);m_progress->setValue(100);
    const auto err=m_simWatcher.result();
    if(!err.isEmpty()){
        m_simPending.reset();
        log(tr("模拟 SCAN 未完成：")+err);
        QMessageBox::warning(this,tr("模拟失败"),err);return;
    }
    if(m_recipeRevision!=m_simRevision){
        log(tr("仿真进行时输入发生变化，不发布旧结果，请重新规划及生成。"));
        m_simPending.reset();return;
    }
    m_simArtifacts=m_simPending;
    if(m_btnCompare)m_btnCompare->setEnabled(m_simArtifacts&&!m_simArtifacts->poses.isEmpty());
    log(tr("按姿态完成 M3/M4 虚拟 .scan + 全分辨率 PLY：")+m_simTarget);
    QMessageBox::information(this,tr("模拟 SCAN 已生成"),
        tr("已写入模拟轮廓 SCAN-v3、工作件有效点二进制 PLY 和 manifest。\n"
           "注意所有帧 PoseTiming=Simulated，不得作为 M4 的独立真实验证集。\n")+m_simTarget);
}
void MultiPoseExperimentWidget::cancelTask(){
    if(m_cancel)m_cancel->store(true);
    log(tr("已请求取消后台规划/模拟，停止前不发布有效结果。"));
}

mls::twin::MultiPosePlaybackState MultiPoseExperimentWidget::sampleCurrentNcPlayback(double at) const {
    if(!m_report||!m_playbackTimeline.ok())return {};
    if(m_connectedNcUsable)
        return mls::twin::MultiPoseExperimentQtAdapter::replayConnectedNcAt(
            *m_report,m_connectedNcReplay,m_playbackTimeline,at);
    return mls::twin::MultiPoseExperimentQtAdapter::replayStateAt(
        *m_report,m_activeNcReplays,m_playbackTimeline,at);
}
void MultiPoseExperimentWidget::startPlayback(){
    if(!m_report||!m_report->ok()||!m_playbackTimeline.ok()||!m_view)return;
    const int selected=m_previewPose?m_previewPose->currentIndex()-1:-1;
    if(selected<0&&m_activeNcReplays.size()>1&&!m_connectedNcUsable){
        QMessageBox::warning(this,tr("姿态间运动尚未验证"),
            tr("每个姿态是独立 G代码，不能在无转场指令时瞬间跳到下一起点。"
               "请先完成姿态间转场的轴限位与 FCL 检查；目前只能单独播放所选姿态。"));
        return;
    }
    stopPlayback(true);
    m_playLimitSeconds=m_playbackTimeline.totalSeconds;
    if(selected>=0&&std::size_t(selected)<m_playbackTimeline.segments.size()){
        m_playSeconds=m_playbackTimeline.segments[std::size_t(selected)].globalStart;
        m_playLimitSeconds=m_playbackTimeline.segments[std::size_t(selected)].globalEnd;
    }else m_playSeconds=0.;
    m_nextPlaybackFrame.assign(m_report->plan.views.size(),0);
    m_lastDrawnPose=std::numeric_limits<std::size_t>::max();
    m_playCurrentPoseOnly=selected>=0;
    m_view->setTwinMultiPosePlaybackActive(true,true);
    m_playing=true;
    if(m_btnPause)m_btnPause->setEnabled(true);
    if(m_btnStop)m_btnStop->setEnabled(true);
    if(m_btnPlay)m_btnPlay->setEnabled(false);
    m_playClock.start();m_playTimer->start();
    updatePlaybackFrame();
    log(m_connectedNcUsable?
        tr("开始执行连续 NC 仿真：姿态间实际 G05 S0 插补，M51 后转场，M50 后采集；不输出硬件指令。"):
        tr("开始执行单姿态独立 NC：只播放所选姿态，不跨姿态跳跃；不输出硬件指令。"));
}
void MultiPoseExperimentWidget::togglePlaybackPause(){
    if(!m_playing||!m_playTimer)return;
    if(m_playTimer->isActive()){
        m_playTimer->stop();
        if(m_btnPause)m_btnPause->setText(tr("继续"));
    }else{
        m_playClock.restart();m_playTimer->start();
        if(m_btnPause)m_btnPause->setText(tr("暂停"));
    }
}
void MultiPoseExperimentWidget::stepPlayback(){
    if(!m_report||!m_playbackTimeline.ok()||!m_view)return;
    if(!m_playing){
        const int selected=m_previewPose?m_previewPose->currentIndex()-1:-1;
        if(selected<0&&!m_connectedNcUsable&&m_activeNcReplays.size()>1){
            QMessageBox::warning(this,tr("连续转场未验证"),
                tr("不能在独立姿态的两份 NC 之间单步瞬移。请先完成姿态间转场验证，或选择一个姿态。"));
            return;
        }
        m_playCurrentPoseOnly=selected>=0;
        if(selected>=0&&std::size_t(selected)<m_playbackTimeline.segments.size()){
            m_playSeconds=m_playbackTimeline.segments[std::size_t(selected)].globalStart;
            m_playLimitSeconds=m_playbackTimeline.segments[std::size_t(selected)].globalEnd;
        }else{
            m_playSeconds=0.;m_playLimitSeconds=m_playbackTimeline.totalSeconds;
        }
        m_playing=true;
        m_view->setTwinMultiPosePlaybackActive(true,true);
        m_nextPlaybackFrame.assign(m_report->plan.views.size(),0);
        m_lastDrawnPose=std::numeric_limits<std::size_t>::max();
        if(m_btnPlay)m_btnPlay->setEnabled(false);
        if(m_btnStep)m_btnStep->setEnabled(true);
        if(m_btnPause)m_btnPause->setEnabled(true);
        if(m_btnStop)m_btnStop->setEnabled(true);
    }
    if(m_playTimer)m_playTimer->stop();
    const auto state=sampleCurrentNcPlayback(m_playSeconds);
    if(!state.valid)return;
    const auto& section=m_playbackTimeline.segments[state.poseIndex];
    if(m_connectedNcUsable){
        const auto& vertices=m_connectedNcReplay.vertices;
        auto it=std::upper_bound(vertices.begin(),vertices.end(),m_playSeconds+1.e-7,
            [](double time,const mls::twin::NcSimVertex& p){return time<p.seconds;});
        m_playSeconds=it!=vertices.end()?it->seconds:m_playbackTimeline.totalSeconds;
    }else{
        const auto& vertices=m_activeNcReplays[state.poseIndex].vertices;
        auto it=std::upper_bound(vertices.begin(),vertices.end(),state.localSeconds+1.e-7,
            [](double time,const mls::twin::NcSimVertex& p){return time<p.seconds;});
        if(it!=vertices.end())m_playSeconds=section.globalStart+it->seconds;
        else m_playSeconds=section.globalEnd;
    }
    m_playSeconds=std::min(m_playSeconds,m_playLimitSeconds);
    updatePlaybackFrame();
}
void MultiPoseExperimentWidget::stopPlayback(bool clearCloud){
    if(m_playTimer)m_playTimer->stop();
    m_playing=false;
    if(m_view)m_view->setTwinMultiPosePlaybackActive(false,clearCloud);
    if(m_btnPlay&&m_report&&m_playbackTimeline.ok())m_btnPlay->setEnabled(true);
    if(m_btnStep&&m_report&&m_playbackTimeline.ok())m_btnStep->setEnabled(true);
    if(m_btnPause){m_btnPause->setEnabled(false);m_btnPause->setText(tr("暂停/继续"));}
    if(m_btnStop)m_btnStop->setEnabled(false);
}
void MultiPoseExperimentWidget::playbackTick(){
    if(!m_playing||!m_report||!m_playTimer)return;
    const double multiplier=m_playSpeed?m_playSpeed->currentData().toDouble():1.;
    const double dt=std::min(.25,double(m_playClock.restart())*.001)*multiplier;
    m_playSeconds=std::min(m_playLimitSeconds,m_playSeconds+dt);
    updatePlaybackFrame();
    if(m_playSeconds>=m_playLimitSeconds-1.e-7){
        stopPlayback();
        if(m_playStatus)m_playStatus->setText(m_playStatus->text()+tr("（回放已结束；模型已恢复原显示轴位）"));
    }
}
void MultiPoseExperimentWidget::seekPlayback(int percent){
    if(!m_report||!m_playbackTimeline.ok()||!m_view)return;
    if(!m_connectedNcUsable&&m_activeNcReplays.size()>1){
        // Do not create fake motion while seeking between independently anchored G00 files.
        return;
    }
    const bool wasRunning=m_playing&&m_playTimer&&m_playTimer->isActive();
    if(!m_playing){
        m_playCurrentPoseOnly=false;
        m_playing=true;
        m_playLimitSeconds=m_playbackTimeline.totalSeconds;
        m_view->setTwinMultiPosePlaybackActive(true,true);
        if(m_btnPause)m_btnPause->setEnabled(true);
        if(m_btnStop)m_btnStop->setEnabled(true);
        if(m_btnPlay)m_btnPlay->setEnabled(false);
        if(m_btnStep)m_btnStep->setEnabled(true);
    }
    if(m_playTimer)m_playTimer->stop();
    // The seek bar spans the full sequence, so seeking leaves the
    // single-pose mode instead of creating a misleading time clamp.
    m_playCurrentPoseOnly=false;
    m_playLimitSeconds=m_playbackTimeline.totalSeconds;
    m_playSeconds=m_playbackTimeline.totalSeconds*std::clamp(percent,0,1000)/1000.;
    m_nextPlaybackFrame.assign(m_report->plan.views.size(),0);
    m_lastDrawnPose=std::numeric_limits<std::size_t>::max();
    m_view->setTwinMultiPosePlaybackActive(true,true);
    updatePlaybackFrame(true);
    if(wasRunning){m_playClock.restart();m_playTimer->start();}
}
void MultiPoseExperimentWidget::updatePlaybackFrame(bool){
    if(!m_view||!m_report||!m_playbackTimeline.ok())return;
    const auto state=sampleCurrentNcPlayback(m_playSeconds);
    if(!state.valid||state.poseIndex>=m_report->plan.views.size())return;
    const auto idx=state.poseIndex;
    const auto& active=m_report->plan.views[idx];
    QVector<QVector3D> newlyObserved;
    for(std::size_t k=0;k<=idx;++k){
        if(m_playCurrentPoseOnly&&k!=idx)continue;
        const bool useFull=m_simArtifacts&&qsizetype(k)<m_simArtifacts->poses.size();
        const QVector<double>* times= useFull?
           &m_simArtifacts->poses[qsizetype(k)].replayTimes:
           &m_report->opticalPlaybackTimes[qsizetype(k)];
        const QVector<QVector<QVector3D>>* frames=useFull?
           &m_simArtifacts->poses[qsizetype(k)].replayFrames:
           &m_report->opticalPlaybackFrames[qsizetype(k)];
        if(k>=m_nextPlaybackFrame.size())break;
        auto& next=m_nextPlaybackFrame[k];
        const double currentTime=(k==idx?state.localSeconds:std::numeric_limits<double>::infinity());
        const double exposureTarget=currentTime; // parsed NC clock starts at zero
        while(next<std::size_t(times->size())&&next<std::size_t(frames->size())&&
              times->at(qsizetype(next))<=exposureTarget+1.e-8){
            newlyObserved+=frames->at(qsizetype(next));
            ++next;
        }
    }
    if(idx!=m_lastDrawnPose){
        m_view->setTwinMultiPosePoseSelection(int(idx));
        m_lastDrawnPose=idx;
    }
    const mls::twin::Sensor2950Geometry sensor;
    m_view->setTwinMultiPosePlaybackFrame(state.joints,state.sensorToMachine,
        m_report->options.focusSensorZmm,sensor.widthAtZ(m_report->options.focusSensorZmm),
        newlyObserved,state.scanning);
    if(m_playSeek){QSignalBlocker hold(m_playSeek);
        const int pct=m_playbackTimeline.totalSeconds>0?
          int(1000.*m_playSeconds/m_playbackTimeline.totalSeconds):0;
        m_playSeek->setValue(std::clamp(pct,0,1000));}
    if(m_playStatus){
        QString lastEvent=tr("尚未触发 M 指令");
        const auto& events=m_activeNcReplays[idx].events;
        for(const auto& event:events){
            if(event.seconds>state.localSeconds+1.e-8)break;
            lastEvent=QString::fromStdString(event.code)+tr(" (NC 第%1行)").arg(static_cast<qulonglong>(event.sourceLine));
        }
        if(state.betweenPoses)
            lastEvent=tr("姿态间 G05 S0 插补：P%1→P%2，M51 已关闭采集")
                .arg(static_cast<qulonglong>(state.previousPoseIndex+1),2,10,QChar('0'))
                .arg(static_cast<qulonglong>(state.poseIndex+1),2,10,QChar('0'));
        m_playStatus->setText(tr("[G代码解析执行仿真 / 不授权实机] %1 | t=%2 / %3 s | %4 | Profile=%5 | "
             "X=%6 Y=%7 Z=%8 C=%9 B=%10 | 本次新增虚拟点=%11%12 | 最后M码=%13")
            .arg(QString::fromStdString(active.id))
            .arg(m_playSeconds,0,'f',2)
            .arg(m_playbackTimeline.totalSeconds,0,'f',2)
            .arg(state.scanning?tr("激光/采集段"):
                 (state.betweenPoses?tr("姿态间连续转场（激光 OFF）"):tr("扫描带转场（激光 OFF）")))
            .arg(static_cast<qulonglong>(state.exposureCount))
            .arg(state.joints[0],0,'f',3).arg(state.joints[1],0,'f',3)
            .arg(state.joints[2],0,'f',3).arg(state.joints[3],0,'f',2)
            .arg(state.joints[4],0,'f',2).arg(newlyObserved.size())
            .arg(m_simArtifacts?tr("；完整模拟缓存已加载"):tr("；仅稀疏光学预览缓存"))
            .arg(lastEvent));
    }
}
void MultiPoseExperimentWidget::startCompare(){
    if(!m_report||!m_simArtifacts||m_compareWatcher.isRunning())return;
    const int selected=m_previewPose?m_previewPose->currentIndex()-1:-1;
    if(selected<0||qsizetype(selected)>=m_simArtifacts->poses.size()){
        QMessageBox::information(this,tr("选择对比姿态"),tr("请先在‘显示全部姿态’下拉框中选择 P01/P02/P03 中具体一个，"
             "再导入这个姿态独立采集的真实 .scan。不能把历史 P01 数据当成 P02/P03。"));
        return;
    }
    const auto file=QFileDialog::getOpenFileName(this,tr("为所选姿态导入独立实测 SCAN"),{},tr("SCAN v3 (*.scan)"));
    if(file.isEmpty())return;
    m_compareRevision=m_recipeRevision;
    const auto plan=m_report;
    const auto sim=m_simArtifacts;
    m_btnCompare->setEnabled(false);
    m_progress->setRange(0,0);
    log(tr("开始姿态 %1 的实测/仿真对比；首先核实 SCAN 真实来源及 C/B。")
        .arg(QString::fromStdString(plan->plan.views[std::size_t(selected)].id)));
    m_compareWatcher.setFuture(QtConcurrent::run([plan,sim,selected,file]{
        return std::make_shared<mls::twin::MultiPoseCompareArtifacts>(
            mls::twin::MultiPoseExperimentQtAdapter::compareRealScanForPose(
                *plan,*sim,std::size_t(selected),file));
    }));
}
void MultiPoseExperimentWidget::comparisonFinished(){
    m_progress->setRange(0,100);m_progress->setValue(100);
    m_btnCompare->setEnabled(bool(m_simArtifacts));
    const auto result=m_compareWatcher.result();
    if(m_recipeRevision!=m_compareRevision){log(tr("对比过程中实验参数改变，丢弃旧结果。"));return;}
    if(!result||!result->ok()){
        const auto error=result?result->error:tr("未收到对比结果");
        log(tr("实测-仿真对比未通过：")+error);
        QMessageBox::warning(this,tr("实测对比失败"),error);return;
    }
    if(m_view)m_view->setTwinMultiPoseComparisonHeatmap(result->realPoints,
        result->realDistancesMm,1.0);
    log(tr("实测与仿真对比完成：")+result->description);
    QMessageBox::information(this,tr("离线几何对比已完成"),
        result->description+tr("\n对比 JSON 已写入仿真导出目录。颜色是实测点到仿真点的近邻距离：绿色较小，红色较大。"));
}
