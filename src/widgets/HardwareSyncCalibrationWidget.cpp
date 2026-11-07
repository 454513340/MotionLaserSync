#include "widgets/HardwareSyncCalibrationWidget.h"

#include "calibration/SphereDistanceAngleStudy.h"
#include "calibration/SphereTwoStageFit.h"
#include "calibration/SphereArcQualityGate.h"
#include "core/SyncManager.h"
#include "core/ZmcController.h"
#include "runtime/Acquisition5D.h"
#include "runtime/HardwareTemporalCorrection.h"
#include "utils/PointCloudLoader.h"
#include "utils/CompactScanFormat.h"

#include <QAbstractItemView>
#include <QDateTime>
#include <QCheckBox>
#include <QComboBox>
#include <QSignalBlocker>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFileDialog>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPointer>
#include <QPushButton>
#include <QSizePolicy>
#include <QSaveFile>
#include <QSpinBox>
#include <QStandardPaths>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QTimer>
#include <QFileInfo>
#include <QVBoxLayout>
#include <QtConcurrent/QtConcurrent>

#include <algorithm>
#include <cmath>
#include <map>
#include <limits>
#include <sstream>
#include <numeric>
#include <new>
#include <exception>

using namespace mls::calibration;

namespace {
constexpr int kPlanCols = 10;
constexpr int kPathCols = 12;
constexpr int kCaptureCols = 14;
constexpr int kSpeedSummaryCols = 6;
// Versioned numeric contract for dynamic H1/H3 sphere observations.
// Old saved observations cannot be mixed with new arc-qualified observations.
const QString kDynamicSphereFitContract =
    QStringLiteral("H1H3-ARC120-RMS010-RANSAC020-MATCHED-HOLDOUT-V1");

QString roleText(bool validation) { return validation ? QObject::tr("验证") : QObject::tr("标定"); }

// Keep long Chinese button captions legible in the ~520px calibration panel.
// Do not rely on OS/Qt automatic text eliding for safety-critical actions.
void readableAction(QPushButton* button, const QString& tooltip = QString())
{
    if (!button) return;
    button->setMinimumHeight(34);
    button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    button->setToolTip(tooltip.isEmpty() ? button->text() : tooltip);
}

bool retryablePlannedScanFailure(const QString& reason)
{
    // Only retry acquisition/tail-closure transients automatically. Geometry,
    // collision, limits, disconnected hardware and operator/safety failures must
    // pause for inspection instead of repeating a potentially unsafe move.
    return reason.contains(QStringLiteral("Stop/Drain"),Qt::CaseInsensitive)
        || reason.contains(QStringLiteral("完整闭环"))
        || reason.contains(QStringLiteral("Probe"),Qt::CaseInsensitive)
        || reason.contains(QStringLiteral("同步意外停止"))
        || reason.contains(QStringLiteral("RAW Profile/REGIST"));
}

Vec3d toVec3(const QVector3D& v) { return {v.x(),v.y(),v.z()}; }
QVector3D toQVec3(const Vec3d& v) { return QVector3D(float(v.x),float(v.y),float(v.z)); }

double norm3(const Vec3d& v){return std::sqrt(v.x*v.x+v.y*v.y+v.z*v.z);}
double dot3(const Vec3d& a,const Vec3d& b){return a.x*b.x+a.y*b.y+a.z*b.z;}
double distance3(const Vec3d&a,const Vec3d&b){return norm3({a.x-b.x,a.y-b.y,a.z-b.z});}

QTableWidgetItem* editableNumber(double v,int decimals=4){return new QTableWidgetItem(QString::number(v,'f',decimals));}
QTableWidgetItem* checkedItem(bool checked){auto* i=new QTableWidgetItem;i->setFlags(i->flags()|Qt::ItemIsUserCheckable);i->setCheckState(checked?Qt::Checked:Qt::Unchecked);return i;}

QJsonArray vecJson(const Vec3d& v){QJsonArray a;a.append(v.x);a.append(v.y);a.append(v.z);return a;}
Vec3d jsonVec(const QJsonValue& v){const auto a=v.toArray();return a.size()==3?Vec3d{a[0].toDouble(),a[1].toDouble(),a[2].toDouble()}:Vec3d{};}

QJsonObject jointPoseJson(const JointPose5D& q){
    QJsonObject j;j["x"]=q.x;j["y"]=q.y;j["z"]=q.z;j["c"]=q.c;j["b"]=q.b;return j;
}
JointPose5D jointPoseFromJson(const QJsonObject& j){
    return {j.value("x").toDouble(),j.value("y").toDouble(),j.value("z").toDouble(),
            j.value("c").toDouble(),j.value("b").toDouble()};
}

bool inferPlanCenterFromArchivedScan(const QString& scanPath, JointPose5D& center, QString* error)
{
    ScanData data;
    QString loadError;
    if(!CompactScanIO::load(scanPath,data,CompactScanIO::ReadOptions(),&loadError)){
        if(error)*error=QObject::tr("读取旧RAW失败：%1").arg(loadError);
        return false;
    }
    bool have=false;
    double lo[5]={0,0,0,0,0}, hi[5]={0,0,0,0,0};
    for(const auto& profile:data.profiles){
        if(!profile.hasFiveAxis)continue;
        const auto& q=profile.fiveAxis.frame.joints;
        bool finite=true;for(double v:q)if(!std::isfinite(v)){finite=false;break;}
        if(!finite)continue;
        if(!have){for(int k=0;k<5;++k)lo[k]=hi[k]=q[std::size_t(k)];have=true;}
        else for(int k=0;k<5;++k){lo[k]=std::min(lo[k],q[std::size_t(k)]);hi[k]=std::max(hi[k],q[std::size_t(k)]);}
    }
    if(!have){if(error)*error=QObject::tr("旧RAW中没有可用五轴Frame5D位姿");return false;}
    center.x=0.5*(lo[0]+hi[0]);center.y=0.5*(lo[1]+hi[1]);center.z=0.5*(lo[2]+hi[2]);
    center.c=0.5*(lo[3]+hi[3]);center.b=0.5*(lo[4]+hi[4]);
    if(!center.isFinite()){if(error)*error=QObject::tr("由旧RAW推导扫描轨迹中点失败：结果非有限值");return false;}
    if(error)error->clear();
    return true;
}


bool inferCaptureMidPcUsFromArchivedScan(const QString& scanPath, qint64& midPcUs, QString* error)
{
    ScanData data;QString loadError;
    if(!CompactScanIO::load(scanPath,data,CompactScanIO::ReadOptions(),&loadError)){
        if(error)*error=QObject::tr("读取RAW时间失败：%1").arg(loadError);return false;
    }
    qint64 first=0,last=0;
    for(const auto& profile:data.profiles){
        if(!profile.hasFiveAxis)continue;
        const qint64 t=qint64(profile.fiveAxis.timeUs);
        if(t<=0)continue;
        if(first==0)first=t;last=t;
    }
    if(first<=0||last<first){if(error)*error=QObject::tr("RAW中没有有效PC时间戳");return false;}
    midPcUs=first+(last-first)/2;if(error)error->clear();return true;
}

QJsonObject trajectoryObservationJson(const TemporalTrajectoryObservation& o)
{
    QJsonObject j;j["id"]=QString::number(qulonglong(o.id));j["planId"]=QString::number(qulonglong(o.planId));j["pairId"]=QString::number(qulonglong(o.pairId));j["directionSign"]=o.directionSign;
    j["velocityMmPerSec"]=vecJson(o.velocityMmPerSec);j["sphereCenterMachineMm"]=vecJson(o.sphereCenterMachineMm);j["sphereRmsMm"]=o.sphereRmsMm;j["fittedRadiusMm"]=o.fittedRadiusMm;
    j["sphereInliers"]=QString::number(qulonglong(o.sphereInliers));j["profileCount"]=QString::number(qulonglong(o.profileCount));j["resolutionIndex"]=int(o.resolutionIndex);j["exposureTime10Us"]=int(o.exposureTime10Us);j["idleTime10Us"]=int(o.idleTime10Us);j["measuringFieldIndex"]=o.measuringFieldIndex;j["triggerFrequencyHz"]=o.triggerFrequencyHz;j["triggerPeriodMs"]=o.triggerPeriodMs;j["captureMidPcUs"]=QString::number(qlonglong(o.captureMidPcUs));return j;
}


QString safeFileToken(QString text)
{
    text = text.trimmed();
    if (text.isEmpty()) text = QStringLiteral("scan");
    for (qsizetype i=0;i<text.size();++i) {
        const QChar ch=text.at(i);
        if (!(ch.isLetterOrNumber() || ch == QLatin1Char('_') || ch == QLatin1Char('-') || ch == QLatin1Char('.')))
            text[i] = QLatin1Char('_');
    }
    while (text.contains(QStringLiteral("__"))) text.replace(QStringLiteral("__"), QStringLiteral("_"));
    return text.left(80);
}

double radialRmsLocal(const std::vector<Vec3d>& points, const Vec3d& c, double r)
{
    if (points.empty()) return std::numeric_limits<double>::infinity();
    long double sum = 0.0L;
    std::size_t n = 0;
    for (const auto& p : points) {
        const double e = distance3(p,c) - r;
        if (std::isfinite(e)) { sum += static_cast<long double>(e) * static_cast<long double>(e); ++n; }
    }
    return n ? std::sqrt(static_cast<double>(sum / static_cast<long double>(n)))
             : std::numeric_limits<double>::infinity();
}

std::vector<std::size_t> evenlySpacedIndices(std::size_t count, std::size_t cap)
{
    std::vector<std::size_t> out;
    if (count == 0 || cap == 0) return out;
    const std::size_t n = std::min(count, cap);
    out.reserve(n);
    if (n == count) {
        for (std::size_t i=0;i<count;++i) out.push_back(i);
        return out;
    }
    for (std::size_t k=0;k<n;++k) {
        const long double t = n==1 ? 0.0L
                                   : static_cast<long double>(k) / static_cast<long double>(n-1);
        std::size_t idx = static_cast<std::size_t>(
            std::llround(t * static_cast<long double>(count-1)));
        if (!out.empty() && idx <= out.back()) idx = std::min(count-1, out.back()+1);
        out.push_back(idx);
    }
    return out;
}

bool saveJsonObjectAtomic(const QString& path, const QJsonObject& object, QString* error)
{
    QSaveFile f(path);
    if (!f.open(QIODevice::WriteOnly)) {
        if (error) *error = f.errorString();
        return false;
    }
    const QByteArray bytes = QJsonDocument(object).toJson(QJsonDocument::Indented);
    if (f.write(bytes) != bytes.size() || !f.commit()) {
        if (error) *error = f.errorString();
        return false;
    }
    if (error) error->clear();
    return true;
}

bool trajectoryObservationFromJson(const QJsonObject& j,TemporalTrajectoryObservation& o)
{
    o.id=j.value("id").toString().toULongLong();o.planId=j.value("planId").toString().toULongLong();o.pairId=j.value("pairId").toString().toULongLong();o.directionSign=j.value("directionSign").toInt();
    o.velocityMmPerSec=jsonVec(j.value("velocityMmPerSec"));o.sphereCenterMachineMm=jsonVec(j.value("sphereCenterMachineMm"));o.sphereRmsMm=j.value("sphereRmsMm").toDouble();o.fittedRadiusMm=j.value("fittedRadiusMm").toDouble();
    o.sphereInliers=std::size_t(j.value("sphereInliers").toString().toULongLong());o.profileCount=std::size_t(j.value("profileCount").toString().toULongLong());o.resolutionIndex=unsigned(j.value("resolutionIndex").toInt());o.exposureTime10Us=unsigned(j.value("exposureTime10Us").toInt());o.idleTime10Us=unsigned(j.value("idleTime10Us").toInt());o.measuringFieldIndex=j.value("measuringFieldIndex").toInt(7);o.triggerFrequencyHz=j.value("triggerFrequencyHz").toInt(0);o.triggerPeriodMs=j.value("triggerPeriodMs").toDouble(1000.0/double(mls::runtime::Table5D::defaultTriggerFrequencyHz));if(o.triggerFrequencyHz<=0&&o.triggerPeriodMs>0.0)o.triggerFrequencyHz=int(std::lround(1000.0/o.triggerPeriodMs));if(o.triggerPeriodMs<=0.0&&o.triggerFrequencyHz>0)o.triggerPeriodMs=1000.0/double(o.triggerFrequencyHz);o.captureMidPcUs=j.value("captureMidPcUs").toString().toLongLong();
    return o.id>0&&o.pairId>0&&(o.directionSign==1||o.directionSign==-1)&&norm3(o.velocityMmPerSec)>0.0;
}
}

HardwareSyncCalibrationWidget::HardwareSyncCalibrationWidget(QWidget* parent):QWidget(parent)
{
    setupUi();addDefaultPlanRows();refreshReferenceUi();refreshPathTable();refreshCaptureTable();refreshSpeedSummaryTable();refreshResult();
}

void HardwareSyncCalibrationWidget::setupUi()
{
    auto* root=new QVBoxLayout(this);root->setContentsMargins(4,4,4,4);
    auto* topology=new QGroupBox(tr("硬件同步标定拓扑（H1/H3 独立，H2 已移除）"),this);auto* tl=new QVBoxLayout(topology);
    m_topologyCombo=new QComboBox(topology);
    m_topologyCombo->addItem(tr("H1 控制器主触发：OP14 → IN0 / R0 (104)"), 1);
    m_topologyCombo->addItem(tr("H3 传感器主触发：CMM → IN1 / R1 (114/115)"), 3);
    m_topologyDescription=new QLabel(topology);
    m_topologyDescription->setWordWrap(true);
    tl->addWidget(m_topologyCombo);tl->addWidget(m_topologyDescription);
    root->addWidget(topology);
    m_topologyDescription->setText(tr("H1: τ=t_sensor,effective−t_R0,latch；H3需切换选择并单独标定：τ=t_sensor,effective−t_R1,latch，可为负值。所有采集保持RAW，Hold-out通过后才可启用补偿。"));
    connect(m_topologyCombo,QOverload<int>::of(&QComboBox::currentIndexChanged),
        this,&HardwareSyncCalibrationWidget::changeSelectedTopology);

    auto* ref=new QGroupBox(tr("1. 参考球：导入点云与拟合球心"),this);auto* rg=new QGridLayout(ref);
    m_sphereDiameter=new QDoubleSpinBox(ref);m_sphereDiameter->setRange(1,100);m_sphereDiameter->setDecimals(4);m_sphereDiameter->setValue(19.050);m_sphereDiameter->setSuffix(" mm");
    m_ransacThreshold=new QDoubleSpinBox(ref);m_ransacThreshold->setRange(0.005,2);m_ransacThreshold->setDecimals(3);m_ransacThreshold->setValue(0.120);m_ransacThreshold->setSuffix(" mm");
    m_minInliers=new QSpinBox(ref);m_minInliers->setRange(20,1000000);m_minInliers->setValue(200);
    m_referencePath=new QLineEdit(ref);m_referencePath->setReadOnly(true);auto* importBtn=new QPushButton(tr("导入球面点云"),ref);auto* fitBtn=new QPushButton(tr("RANSAC拟合球心"),ref);
    readableAction(importBtn);readableAction(fitBtn);
    m_referenceResult=new QLabel(ref);m_referenceResult->setWordWrap(true);
    m_referenceResult->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    rg->addWidget(new QLabel(tr("球直径"),ref),0,0);rg->addWidget(m_sphereDiameter,0,1);
    rg->addWidget(new QLabel(tr("RANSAC阈值"),ref),0,2);rg->addWidget(m_ransacThreshold,0,3);
    rg->addWidget(new QLabel(tr("最少内点"),ref),1,0);rg->addWidget(m_minInliers,1,1);
    rg->addWidget(m_referencePath,2,0,1,4);
    rg->addWidget(importBtn,3,0,1,2);rg->addWidget(fitBtn,3,2,1,2);
    rg->addWidget(m_referenceResult,4,0,1,4);root->addWidget(ref);

    auto* plan=new QGroupBox(tr("2. 扫描配置与可执行路径生成"),this);auto* pv=new QVBoxLayout(plan);
    auto* centerBtn=new QPushButton(tr("示教当前XYZCB为扫描轨迹中点"),plan);
    m_planCenterLabel=new QLabel(tr("扫描轨迹中点：未读取"),plan);
    auto* defaults=new QPushButton(tr("生成多速度实验模板"),plan);
    auto* add=new QPushButton(tr("添加配置"),plan);
    auto* remove=new QPushButton(tr("删除配置"),plan);
    m_generateButton=new QPushButton(tr("根据配置生成路径"),plan);
    for(auto* action:{centerBtn,defaults,add,remove,m_generateButton})readableAction(action);
    centerBtn->setToolTip(tr("只读取当前五轴实际姿态作为路径中点，不会移动机床"));
    m_generateButton->setToolTip(tr("只生成路径起点、终点、速度和Pair ID，不会启动运动或扫描。失败时写入系统日志。"));
    m_planCenterLabel->setWordWrap(true);
    m_planCenterLabel->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Preferred);
    auto* planActions=new QGridLayout;
    planActions->setColumnStretch(0,1);planActions->setColumnStretch(1,1);
    planActions->addWidget(centerBtn,0,0);planActions->addWidget(defaults,0,1);
    planActions->addWidget(add,1,0);planActions->addWidget(remove,1,1);
    planActions->addWidget(m_generateButton,2,0,1,2);
    planActions->addWidget(m_planCenterLabel,3,0,1,2);
    pv->addLayout(planActions);
    // Clear, word-wrapped feedback occupies its own full-width row.
    m_pathGenerationFeedback=new QLabel(tr("当前尚未生成可执行路径；配置表不是路径队列。"),plan);
    m_pathGenerationFeedback->setWordWrap(true);
    m_pathGenerationFeedback->setTextInteractionFlags(Qt::TextSelectableByMouse);
    m_pathGenerationFeedback->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Preferred);
    pv->addWidget(m_pathGenerationFeedback);

    auto* templateLine=new QGridLayout;
    m_templateMinSpeed=new QDoubleSpinBox(plan);m_templateMinSpeed->setRange(0.1,100.0);m_templateMinSpeed->setDecimals(2);m_templateMinSpeed->setSingleStep(0.5);m_templateMinSpeed->setValue(0.5);m_templateMinSpeed->setSuffix(" mm/s");
    m_templateMaxSpeed=new QDoubleSpinBox(plan);m_templateMaxSpeed->setRange(0.1,100.0);m_templateMaxSpeed->setDecimals(2);m_templateMaxSpeed->setSingleStep(0.5);m_templateMaxSpeed->setValue(5.0);m_templateMaxSpeed->setSuffix(" mm/s");
    m_templateSpeedStep=new QDoubleSpinBox(plan);m_templateSpeedStep->setRange(0.1,20.0);m_templateSpeedStep->setDecimals(2);m_templateSpeedStep->setSingleStep(0.1);m_templateSpeedStep->setValue(0.5);m_templateSpeedStep->setSuffix(" mm/s");
    m_templateCalibrationRepeats=new QSpinBox(plan);m_templateCalibrationRepeats->setRange(1,10);m_templateCalibrationRepeats->setValue(3);m_templateCalibrationRepeats->setSuffix(tr(" Pair/速度"));
    m_templateDistance=new QDoubleSpinBox(plan);m_templateDistance->setRange(1.0,500.0);m_templateDistance->setDecimals(1);m_templateDistance->setValue(30.0);m_templateDistance->setSuffix(" mm");
    m_templateValidationSpeed=new QDoubleSpinBox(plan);m_templateValidationSpeed->setRange(0.1,100.0);m_templateValidationSpeed->setDecimals(2);m_templateValidationSpeed->setValue(4.0);m_templateValidationSpeed->setSuffix(" mm/s");
    m_templateValidationRepeats=new QSpinBox(plan);m_templateValidationRepeats->setRange(1,10);m_templateValidationRepeats->setValue(3);m_templateValidationRepeats->setSuffix(tr(" Pair/轨迹"));
    // Two label/input pairs per row: 8-column horizontal layout previously
    // squeezed spin boxes and chopped Chinese captions on 520px panels.
    templateLine->setColumnStretch(1,1);templateLine->setColumnStretch(3,1);
    templateLine->addWidget(new QLabel(tr("起始速度"),plan),0,0);templateLine->addWidget(m_templateMinSpeed,0,1);
    templateLine->addWidget(new QLabel(tr("终止速度"),plan),0,2);templateLine->addWidget(m_templateMaxSpeed,0,3);
    templateLine->addWidget(new QLabel(tr("速度步长"),plan),1,0);templateLine->addWidget(m_templateSpeedStep,1,1);
    templateLine->addWidget(new QLabel(tr("标定重复"),plan),1,2);templateLine->addWidget(m_templateCalibrationRepeats,1,3);
    templateLine->addWidget(new QLabel(tr("扫描距离"),plan),2,0);templateLine->addWidget(m_templateDistance,2,1);
    templateLine->addWidget(new QLabel(tr("验证速度"),plan),2,2);templateLine->addWidget(m_templateValidationSpeed,2,3);
    templateLine->addWidget(new QLabel(tr("验证重复"),plan),3,0);templateLine->addWidget(m_templateValidationRepeats,3,1);
    pv->addLayout(templateLine);
    auto* templateHint=new QLabel(tr("推荐实验：X主扫掠 0.5~5.0 mm/s、0.5步长、每速度3个Pair；默认低/高速度交错执行，3次重复按 +−/−+/+− 交替。V27.6.53动态球心沿用“自由有效半径”拟合，不强迫动态点云半径等于19.05/2；正反Pair只要求有效半径一致。求解同时估计慢漂移 c(t)=c0+g·t−τ·v，0.5 mm/s低速主要用于观测低速/漂移耦合。"),plan);templateHint->setWordWrap(true);pv->addWidget(templateHint);

    auto* configExplanation=new QLabel(tr("扫描配置模板：这里只定义速度、方向和正反Pair，尚未生成可执行路径。"),plan);
    configExplanation->setWordWrap(true);pv->addWidget(configExplanation);
    m_planConfigTable=new QTableWidget(0,kPlanCols,plan);m_planConfigTable->setHorizontalHeaderLabels({tr("启用"),tr("用途"),tr("名称"),tr("X权重"),tr("Y权重"),tr("Z权重"),tr("速度 mm/s"),tr("距离 mm"),tr("重复Pair"),tr("说明")});m_planConfigTable->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);m_planConfigTable->horizontalHeader()->setStretchLastSection(true);m_planConfigTable->setMaximumHeight(260);pv->addWidget(m_planConfigTable);
    auto* pathExplanation=new QLabel(tr("可执行路径队列：必须点击“根据配置生成路径”成功后才显示起点、终点、Plan ID和待执行状态；RAW列在采集归档后才出现文件名。"),plan);
    pathExplanation->setWordWrap(true);pv->addWidget(pathExplanation);
    m_pathTable=new QTableWidget(0,kPathCols,plan);m_pathTable->setHorizontalHeaderLabels({tr("#"),tr("用途"),tr("Pair"),tr("名称"),tr("方向"),tr("起点XYZ"),tr("终点XYZ"),tr("速度"),tr("距离"),tr("状态"),tr("ID"),tr("RAW归档")});m_pathTable->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);m_pathTable->horizontalHeader()->setStretchLastSection(true);m_pathTable->setMaximumHeight(210);pv->addWidget(m_pathTable);root->addWidget(plan);

    auto* acquire=new QGroupBox(tr("3. 标定/验证路径采集与断点Session"),this);
    auto* av=new QVBoxLayout(acquire);
    auto* ah=new QGridLayout;
    m_startCalibrationButton=new QPushButton(tr("开始/继续标定路径采集"),acquire);
    m_startValidationButton=new QPushButton(tr("开始/继续验证路径采集"),acquire);
    m_resumeSessionButton=new QPushButton(tr("恢复/继续已有Session"),acquire);
    m_newSessionButton=new QPushButton(tr("新建/重置本轮Session"),acquire);
    m_abortButton=new QPushButton(tr("暂停当前流程"),acquire);
    auto* clear=new QPushButton(tr("清空标定/验证数据"),acquire);
    for(auto* action:{m_startCalibrationButton,m_startValidationButton,m_resumeSessionButton,m_newSessionButton,m_abortButton,clear})readableAction(action);
    m_newSessionButton->setToolTip(tr("清空当前界面历史Session/采集进度，不删除以前归档到磁盘的RAW .scan数据"));
    m_resumeSessionButton->setToolTip(tr("从已有session_manifest_v49.json及RAW档案恢复未完成的采集"));
    m_status=new QLabel(tr("等待步骤1"),acquire);m_status->setWordWrap(true);
    m_status->setTextInteractionFlags(Qt::TextSelectableByMouse);
    m_status->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Preferred);
    ah->setColumnStretch(0,1);ah->setColumnStretch(1,1);
    ah->addWidget(m_startCalibrationButton,0,0);ah->addWidget(m_startValidationButton,0,1);
    ah->addWidget(m_resumeSessionButton,1,0);ah->addWidget(m_newSessionButton,1,1);
    ah->addWidget(m_abortButton,2,0);ah->addWidget(clear,2,1);
    av->addLayout(ah);
    auto* workflowHint=new QLabel(tr("路径采集与RAW归档分开执行；已有Session时不能覆盖Plan ID，请先恢复旧轮次或明确重置。"),acquire);
    workflowHint->setWordWrap(true);av->addWidget(workflowHint);
    av->addWidget(m_status);

    auto* storage=new QGridLayout;
    m_captureRootDirectory=new QLineEdit(QStringLiteral("E:/Projects/CData"),acquire);
    auto* browseCaptureDir=new QPushButton(tr("选择目录"),acquire);
    m_archiveEachScan=new QCheckBox(tr("每条扫描自动保存完整RAW .scan"),acquire);m_archiveEachScan->setChecked(true);
    m_maxAnalysisPoints=new QSpinBox(acquire);m_maxAnalysisPoints->setRange(10000,1000000);m_maxAnalysisPoints->setSingleStep(50000);m_maxAnalysisPoints->setValue(250000);m_maxAnalysisPoints->setSuffix(tr(" 点/条"));
    m_autoRetryCount=new QSpinBox(acquire);m_autoRetryCount->setRange(0,5);m_autoRetryCount->setValue(2);m_autoRetryCount->setSuffix(tr(" 次/路径"));
    readableAction(browseCaptureDir);
    storage->setColumnStretch(1,1);
    storage->addWidget(new QLabel(tr("RAW归档根目录"),acquire),0,0);
    storage->addWidget(m_captureRootDirectory,0,1);
    storage->addWidget(browseCaptureDir,0,2);
    storage->addWidget(m_archiveEachScan,1,0,1,3);
    storage->addWidget(new QLabel(tr("最大分析点数"),acquire),2,0);storage->addWidget(m_maxAnalysisPoints,2,1);
    storage->addWidget(new QLabel(tr("失败自动重试"),acquire),3,0);storage->addWidget(m_autoRetryCount,3,1);
    av->addLayout(storage);
    auto* rawHint=new QLabel(tr("RAW归档=完成一次硬件扫描后保存的原始Profile与五轴锁存位姿（.scan），供求解、校验和断点恢复；不是扫描路径文件。未执行扫描时RAW列为空是正常的。"),acquire);
    rawHint->setWordWrap(true);av->addWidget(rawHint);
    auto* memoryHint=new QLabel(tr("每条扫描Stop/Drain后立即把完整数据从SyncManager“转移”到后台工作线程；GUI不再累计实时点云。每条成功扫描都会原子更新Session断点文件；单条失败只重试/暂停该条，之前成功路径不会清空。程序重启后可从已有Session目录继续。"),acquire);
    memoryHint->setWordWrap(true);av->addWidget(memoryHint);
    root->addWidget(acquire);

    connect(browseCaptureDir,&QPushButton::clicked,this,[this](){
        const QString base=m_captureRootDirectory?m_captureRootDirectory->text():QStringLiteral("E:/Projects/CData");
        const QString p=QFileDialog::getExistingDirectory(this,tr("选择硬件同步标定RAW归档目录"),base);
        if(!p.isEmpty()&&m_captureRootDirectory)m_captureRootDirectory->setText(QDir::toNativeSeparators(p));
    });

    auto* captureExplanation=new QLabel(tr("已完成扫描的球心拟合结果（可能由旧Session/标定JSON恢复）；Vx/Vy/Vz与球RMS表示测量结果，不代表本轮已生成路径。"),this);
    captureExplanation->setWordWrap(true);root->addWidget(captureExplanation);
    m_captureTable=new QTableWidget(0,kCaptureCols,this);m_captureTable->setHorizontalHeaderLabels({tr("用途"),tr("Pair"),tr("方向"),tr("Vx"),tr("Vy"),tr("Vz"),tr("球心X"),tr("球心Y"),tr("球心Z"),tr("有效R"),tr("球RMS"),tr("校正后RMS"),tr("内点"),tr("Profiles")});m_captureTable->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);m_captureTable->setMaximumHeight(220);root->addWidget(m_captureTable);

    auto* solve=new QGroupBox(tr("4. 求解与质量门限"),this);auto* sf=new QFormLayout(solve);
    m_minPairs=new QSpinBox(solve);m_minPairs->setRange(1,100);m_minPairs->setValue(20);m_minSpeedLevels=new QSpinBox(solve);m_minSpeedLevels->setRange(1,30);m_minSpeedLevels->setValue(8);m_minR2=new QDoubleSpinBox(solve);m_minR2->setRange(-1,1);m_minR2->setDecimals(3);m_minR2->setValue(0.70);m_maxPairResidual=new QDoubleSpinBox(solve);m_maxPairResidual->setRange(0.001,5);m_maxPairResidual->setDecimals(3);m_maxPairResidual->setValue(0.050);m_maxPairResidual->setSuffix(" mm");m_maxPairRadiusDiff=new QDoubleSpinBox(solve);m_maxPairRadiusDiff->setRange(0.001,2.0);m_maxPairRadiusDiff->setDecimals(3);m_maxPairRadiusDiff->setValue(0.050);m_maxPairRadiusDiff->setSuffix(" mm");m_maxStdUs=new QDoubleSpinBox(solve);m_maxStdUs->setRange(1,5000);m_maxStdUs->setValue(150);m_maxStdUs->setSuffix(" µs");
    auto* solveLine=new QWidget(solve);auto* sl=new QHBoxLayout(solveLine);sl->setContentsMargins(0,0,0,0);m_solveButton=new QPushButton(tr("求解 τ"),solveLine);m_resultLabel=new QLabel(solveLine);m_resultLabel->setWordWrap(true);sl->addWidget(m_solveButton);sl->addWidget(m_resultLabel,1);
    m_speedSummaryTable=new QTableWidget(0,kSpeedSummaryCols,solve);m_speedSummaryTable->setHorizontalHeaderLabels({tr("速度档 mm/s"),tr("Pair数"),tr("实际速度均值"),tr("τ均值 µs"),tr("τ标准差 µs"),tr("τ范围 µs")});m_speedSummaryTable->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);m_speedSummaryTable->horizontalHeader()->setStretchLastSection(true);m_speedSummaryTable->setMaximumHeight(210);m_speedSummaryTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    sf->addRow(tr("最少正反Pair"),m_minPairs);sf->addRow(tr("最少不同速度等级"),m_minSpeedLevels);sf->addRow(tr("最小模型R²"),m_minR2);sf->addRow(tr("最大单Pair三维残差"),m_maxPairResidual);sf->addRow(tr("最大Pair有效半径差"),m_maxPairRadiusDiff);sf->addRow(tr("最大τ标准差"),m_maxStdUs);sf->addRow(solveLine);sf->addRow(tr("各速度重复性统计"),m_speedSummaryTable);root->addWidget(solve);

    auto* val=new QGroupBox(tr("5. Hold-out校验：独立路径 raw 与候选τ离线补偿对比"),this);auto* vf=new QFormLayout(val);
    m_minValidationPairs=new QSpinBox(val);m_minValidationPairs->setRange(1,30);m_minValidationPairs->setValue(2);m_minValidationImprovement=new QDoubleSpinBox(val);m_minValidationImprovement->setRange(0,100);m_minValidationImprovement->setValue(50);m_minValidationImprovement->setSuffix(" %");m_maxValidationResidualUs=new QDoubleSpinBox(val);m_maxValidationResidualUs->setRange(1,5000);m_maxValidationResidualUs->setValue(150);m_maxValidationResidualUs->setSuffix(" µs");
    auto* valLine=new QWidget(val);auto* vl=new QHBoxLayout(valLine);vl->setContentsMargins(0,0,0,0);m_validateButton=new QPushButton(tr("评估验证结果"),valLine);m_validationLabel=new QLabel(valLine);m_validationLabel->setWordWrap(true);vl->addWidget(m_validateButton);vl->addWidget(m_validationLabel,1);
    vf->addRow(tr("最少验证Pair"),m_minValidationPairs);vf->addRow(tr("最小残余时延改善"),m_minValidationImprovement);vf->addRow(tr("最大校正后残余τ"),m_maxValidationResidualUs);vf->addRow(valLine);root->addWidget(val);

    auto* finalBox=new QGroupBox(tr("6. 运行时补偿 / 保存 / 加载"),this);auto* fh=new QGridLayout(finalBox);m_applyButton=new QPushButton(tr("验证通过后启用补偿"),finalBox);auto* disable=new QPushButton(tr("禁用补偿"),finalBox);auto* save=new QPushButton(tr("另存JSON"),finalBox);auto* load=new QPushButton(tr("加载JSON"),finalBox);m_runtimeLabel=new QLabel(finalBox);m_runtimeLabel->setWordWrap(true);m_runtimeLabel->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Preferred);
    for(auto* action:{m_applyButton,disable,save,load})readableAction(action);
    fh->setColumnStretch(0,1);fh->setColumnStretch(1,1);
    fh->addWidget(m_applyButton,0,0);fh->addWidget(disable,0,1);
    fh->addWidget(save,1,0);fh->addWidget(load,1,1);
    fh->addWidget(m_runtimeLabel,2,0,1,2);root->addWidget(finalBox);root->addStretch();

    connect(importBtn,&QPushButton::clicked,this,&HardwareSyncCalibrationWidget::importReferenceCloud);connect(fitBtn,&QPushButton::clicked,this,&HardwareSyncCalibrationWidget::fitReferenceSphere);connect(centerBtn,&QPushButton::clicked,this,&HardwareSyncCalibrationWidget::capturePlanCenter);connect(defaults,&QPushButton::clicked,this,&HardwareSyncCalibrationWidget::addDefaultPlanRows);connect(add,&QPushButton::clicked,this,&HardwareSyncCalibrationWidget::addPlanRow);connect(remove,&QPushButton::clicked,this,&HardwareSyncCalibrationWidget::removePlanRow);connect(m_generateButton,&QPushButton::clicked,this,&HardwareSyncCalibrationWidget::generatePaths);connect(m_startCalibrationButton,&QPushButton::clicked,this,&HardwareSyncCalibrationWidget::startCalibrationAcquisition);connect(m_startValidationButton,&QPushButton::clicked,this,&HardwareSyncCalibrationWidget::startValidationAcquisition);connect(m_resumeSessionButton,&QPushButton::clicked,this,&HardwareSyncCalibrationWidget::restoreCaptureSession);connect(m_newSessionButton,&QPushButton::clicked,this,&HardwareSyncCalibrationWidget::newCaptureSession);connect(m_abortButton,&QPushButton::clicked,this,&HardwareSyncCalibrationWidget::abortWorkflow);connect(clear,&QPushButton::clicked,this,&HardwareSyncCalibrationWidget::clearCalibrationData);connect(m_solveButton,&QPushButton::clicked,this,&HardwareSyncCalibrationWidget::solveCalibration);connect(m_validateButton,&QPushButton::clicked,this,&HardwareSyncCalibrationWidget::evaluateValidation);connect(m_applyButton,&QPushButton::clicked,this,&HardwareSyncCalibrationWidget::applyCalibration);connect(disable,&QPushButton::clicked,this,&HardwareSyncCalibrationWidget::disableCorrection);connect(save,&QPushButton::clicked,this,&HardwareSyncCalibrationWidget::saveCalibrationAs);connect(load,&QPushButton::clicked,this,&HardwareSyncCalibrationWidget::loadCalibrationFrom);
}

void HardwareSyncCalibrationWidget::setSyncManager(SyncManager* manager){m_sync=manager;if(manager)loadActiveOnBind();refreshResult();updateWorkflowButtons();}
void HardwareSyncCalibrationWidget::setController(ZmcController* controller){m_controller=controller;updateWorkflowButtons();}

void HardwareSyncCalibrationWidget::changeSelectedTopology(int index)
{
    if(!m_topologyCombo || index<0)return;
    const int next=m_topologyCombo->itemData(index).toInt();
    if(next==m_selectedTopologyId)return;
    if((next!=1&&next!=3) || m_workflowMode!=WorkflowMode::Idle ||
        m_captureProcessing || (m_sync && m_sync->isRunning()) ||
        hasAcceptedPlans(ScanRole::Calibration) || hasAcceptedPlans(ScanRole::Validation)) {
        const QSignalBlocker blocked(m_topologyCombo);
        m_topologyCombo->setCurrentIndex(m_topologyCombo->findData(m_selectedTopologyId));
        if(m_status)m_status->setText(tr("切换H1/H3需要停止采集和后台任务；已有接受的路径必须先完成/归档后新建Session，不允许混用拓扑。"));
        return;
    }
    if(!m_calibrationObservations.empty() || !m_validationCaptures.empty() ||
       !m_captureSessionId.isEmpty()){
        const auto answer=QMessageBox::question(this,tr("切换时域标定拓扑"),
            tr("切换将清空当前页面的候选解、会话进度与路径配置；已保存的H1/H3标定文件和RAW归档不会删除。继续吗？"));
        if(answer!=QMessageBox::Yes){
            const QSignalBlocker blocked(m_topologyCombo);
            m_topologyCombo->setCurrentIndex(m_topologyCombo->findData(m_selectedTopologyId));
            return;
        }
    }
    if(m_sync){
        QString err;
        if(!m_sync->setHardwareTemporalCorrection(HardwareTemporalCorrectionSettings{},&err)){
            const QSignalBlocker blocked(m_topologyCombo);
            m_topologyCombo->setCurrentIndex(m_topologyCombo->findData(m_selectedTopologyId));
            if(m_status)m_status->setText(err);
            return;
        }
    }
    m_selectedTopologyId=next;
    m_sessionLatchEdge=(next==1?104:0);
    m_captureSessionId.clear();m_captureSessionDirectory.clear();m_plannedScans.clear();
    m_calibrationObservations.clear();m_validationCaptures.clear();
    m_result={};m_validationResult={};m_haveResult=false;m_validationPassed=false;
    m_calibrationId.clear();m_referenceFitValid=false;m_referenceCloud.clear();
    m_referenceCloudPath.clear();m_referenceFit={};m_referenceTwoStageDiagnostic.clear();m_planCenterValid=false;
    if(m_planCenterLabel)m_planCenterLabel->setText(tr("扫描轨迹中点：未读取（切换拓扑后必须重新示教）"));
    if(m_pathGenerationFeedback)m_pathGenerationFeedback->setText(tr("已切换H%1：请重新拟合球心、示教安全扫描中点，再生成路径。").arg(next));
    m_planConfigTable->setRowCount(0);
    if(m_referencePath)m_referencePath->clear();
    if(m_topologyDescription)m_topologyDescription->setText(next==1
        ? tr("H1: OP14同时触发scanCONTROL与IN0/R0，延迟 τ>=0，旧H1标定不会被改写为H3。")
        : tr("H3: scanCONTROL内部时钟+CMM→IN1/R1。必须独立标定有符号τ，并绑定实际R1边沿114/115；禁用H1标定复用。"));
    emit runtimeCorrectionChanged(false,0.0);
    loadActiveOnBind();
    refreshReferenceUi();refreshPathTable();refreshCaptureTable();refreshResult();updateWorkflowButtons();
    if(m_status)m_status->setText(tr("已切换为H%1标定；请确认主采集参数及CMM边沿后执行RAW标定。")
        .arg(next));
}

void HardwareSyncCalibrationWidget::importReferenceCloud()
{
    const QString p=QFileDialog::getOpenFileName(this,tr("导入标准球机床坐标点云"),QString(),tr("点云 (*.scan *.ply *.csv *.txt);;所有文件 (*)"));if(p.isEmpty())return;
    QVector<QVector3D> pts;QString err;if(!PointCloudLoader::load(p,pts,&err)||pts.size()<4){QMessageBox::warning(this,tr("导入失败"),err.isEmpty()?tr("点云有效点不足"):err);return;}
    m_referenceCloud=pts;m_referenceCloudPath=p;m_referencePath->setText(p);m_referenceFitValid=false;
    m_referenceTwoStageDiagnostic.clear();emit referenceSphereChanged(0,0,0,0,false);emit referenceCloudViewRequested(pts);refreshReferenceUi();m_status->setText(tr("已导入%1点；下一步点击RANSAC拟合球心。 ").arg(pts.size()));emit logMessage(tr("[硬件同步标定] 标准球点云已导入：%1，N=%2").arg(p).arg(pts.size()));updateWorkflowButtons();
}

void HardwareSyncCalibrationWidget::fitReferenceSphere()
{
    if(m_referenceCloud.size()<4){m_status->setText(tr("请先导入标准球点云"));return;}
    std::vector<Vec3d> cloud;cloud.reserve(std::size_t(m_referenceCloud.size()));for(const auto&p:m_referenceCloud)cloud.push_back({p.x(),p.y(),p.z()});
    SphereCloudRansacOptions opt;opt.expectedRadiusMm=m_sphereDiameter->value()/2.0;opt.inlierThresholdMm=m_ransacThreshold->value();opt.minInliers=std::size_t(m_minInliers->value());opt.maxRadiusDeviationMm=std::max(0.25,5.0*opt.inlierThresholdMm);
    const auto compared=SphereTwoStageFit::fitImportedSphere(m_referenceCloudPath,cloud,opt);
    emit logMessage(tr("[硬件同步标定][两阶段参考球] %1").arg(SphereTwoStageFit::comparisonText(compared)));
    const auto& r=compared.accepted;
    if(!r.success){
        m_status->setText(tr("参考球拟合不通过：%1").arg(SphereTwoStageFit::comparisonText(compared)));
        m_referenceFitValid=false;
        emit referenceSphereChanged(0,0,0,0,false);
        updateWorkflowButtons();return;
    }
    // Preserve binary64 archived coordinates instead of refining display floats.
    const auto& in=compared.acceptedInlierMachinePoints;
    const double acceptedThreshold=compared.twoStageAccepted?0.020:opt.inlierThresholdMm;
    const auto f=HardwareTemporalCalibration::refineKnownSphereCenter(in,r.centerMachineMm,opt.expectedRadiusMm,acceptedThreshold,30);if(!f.success){m_status->setText(tr("参考球固定半径精化失败：%1").arg(QString::fromStdString(f.message)));return;}
    m_referenceFit=f;m_referenceFitValid=true;m_referenceFreeRadiusMm=r.radiusMm;m_referenceInliers=r.inlierPointCount;
    m_referenceTwoStageDiagnostic=SphereTwoStageFit::comparisonText(compared);emit referenceSphereChanged(f.centerMachineMm.x,f.centerMachineMm.y,f.centerMachineMm.z,opt.expectedRadiusMm,true);refreshReferenceUi();m_status->setText(tr("两阶段参考球拟合已完成；下一步示教扫描轨迹中点。\n%1")
        .arg(m_referenceTwoStageDiagnostic));emit logMessage(tr("[硬件同步标定] 参考球心=(%1,%2,%3) mm，固定R=%4 mm，RMS=%5 mm，内点=%6").arg(f.centerMachineMm.x,0,'f',6).arg(f.centerMachineMm.y,0,'f',6).arg(f.centerMachineMm.z,0,'f',6).arg(f.knownRadiusMm,0,'f',6).arg(f.residualRmsMm,0,'f',6).arg(qulonglong(f.pointCount)));updateWorkflowButtons();
}

void HardwareSyncCalibrationWidget::refreshReferenceUi()
{
    if(!m_referenceFitValid)m_referenceResult->setText(tr("球心：未建立。导入静态/高质量机床坐标球面点云后再拟合。"));
    else m_referenceResult->setText(tr("球心 M=(%1, %2, %3) mm；固定R=%4 mm；自由R=%5 mm；RMS=%6 mm；内点=%7。注意：球心用于求解/验证，扫描轨迹中点使用当前XYZCB示教位姿，因为存在手眼偏置；它不是标准球球心。")
        .arg(m_referenceFit.centerMachineMm.x,0,'f',6).arg(m_referenceFit.centerMachineMm.y,0,'f',6).arg(m_referenceFit.centerMachineMm.z,0,'f',6).arg(m_referenceFit.knownRadiusMm,0,'f',6).arg(m_referenceFreeRadiusMm,0,'f',6).arg(m_referenceFit.residualRmsMm,0,'f',6).arg(qulonglong(m_referenceInliers))
        + QStringLiteral("\n") + m_referenceTwoStageDiagnostic);
}

void HardwareSyncCalibrationWidget::capturePlanCenter()
{
    const auto rejectCenter=[this](const QString& reason){
        if(m_status)m_status->setText(reason);
        if(m_pathGenerationFeedback)m_pathGenerationFeedback->setText(reason);
        emit logMessage(tr("[硬件同步标定][CENTER-REJECT][H%1] %2").arg(m_selectedTopologyId).arg(reason));
        QMessageBox::warning(this,tr("无法示教扫描中点"),reason);
    };
    if(!m_controller||!m_controller->isConnected()){
        rejectCenter(tr("控制器未连接，不能读取实时XYZCB；请先连接并完成五轴回零。"));return;
    }
    if(m_captureProcessing||(m_sync&&m_sync->isRunning())){
        rejectCenter(tr("采集/后台处理运行中不能读取规划中心；请等待安全停止。"));return;
    }
    if(hasAcceptedPlans(ScanRole::Calibration)||hasAcceptedPlans(ScanRole::Validation)||
       !m_captureSessionId.isEmpty()||!m_calibrationObservations.empty()||!m_validationCaptures.empty()){
        rejectCenter(tr("当前已有Session/标定结果，禁止更改旧轮次的扫描中点。请先点击“新建/重置本轮Session”，磁盘RAW不会删除。"));return;
    }
    JointPose5D q;QString err;
    if(!m_controller->readJointPose5D(q,&err)){
        rejectCenter(tr("读取XYZCB失败：%1").arg(err));return;
    }
    m_planCenter=q;m_planCenterValid=true;
    m_planCenterLabel->setText(tr("扫描轨迹中点 XYZ=(%1,%2,%3) C/B=(%4,%5)")
        .arg(q.x,0,'f',4).arg(q.y,0,'f',4).arg(q.z,0,'f',4).arg(q.c,0,'f',3).arg(q.b,0,'f',3));
    m_status->setText(tr("已读取扫描轨迹中点；模板会以该位姿为中心对称生成正反扫描端点。请确认 ±距离/2 范围安全后生成路径。"));
    if(m_pathGenerationFeedback)m_pathGenerationFeedback->setText(tr("扫描中点已就绪：H%1；可点击“根据配置生成路径”（尚未生成路径）。").arg(m_selectedTopologyId));
    emit logMessage(tr("[硬件同步标定][CENTER-OK][H%1] 示教扫描中点XYZ=(%2,%3,%4)，C/B=(%5,%6)°")
        .arg(m_selectedTopologyId).arg(q.x,0,'f',4).arg(q.y,0,'f',4).arg(q.z,0,'f',4).arg(q.c,0,'f',3).arg(q.b,0,'f',3));
    updateWorkflowButtons();
}

void HardwareSyncCalibrationWidget::addPlanRow()
{
    const int r=m_planConfigTable->rowCount();m_planConfigTable->insertRow(r);m_planConfigTable->setItem(r,0,checkedItem(true));m_planConfigTable->setItem(r,1,new QTableWidgetItem(tr("标定")));m_planConfigTable->setItem(r,2,new QTableWidgetItem(tr("自定义")));m_planConfigTable->setItem(r,3,editableNumber(1,3));m_planConfigTable->setItem(r,4,editableNumber(0,3));m_planConfigTable->setItem(r,5,editableNumber(0,3));m_planConfigTable->setItem(r,6,editableNumber(3,3));m_planConfigTable->setItem(r,7,editableNumber(30,3));m_planConfigTable->setItem(r,8,new QTableWidgetItem("1"));m_planConfigTable->setItem(r,9,new QTableWidgetItem(tr("0权重=不选该轴；自动生成正反Pair")));
}

void HardwareSyncCalibrationWidget::addDefaultPlanRows()
{
    if(!m_planConfigTable)return;
    const auto rejectTemplate=[this](const QString& reason){
        if(m_status)m_status->setText(reason);
        if(m_pathGenerationFeedback)m_pathGenerationFeedback->setText(reason);
        emit logMessage(tr("[硬件同步标定][TEMPLATE-REJECT][H%1] %2").arg(m_selectedTopologyId).arg(reason));
    };
    if(m_workflowMode!=WorkflowMode::Idle||m_captureProcessing||(m_sync&&m_sync->isRunning())){rejectTemplate(tr("当前硬件采集/后台处理流程运行中，不能重建实验模板。"));return;}
    if(hasAcceptedPlans(ScanRole::Calibration)||hasAcceptedPlans(ScanRole::Validation)||!m_captureSessionId.isEmpty()||
       !m_calibrationObservations.empty()||!m_validationCaptures.empty()){
        rejectTemplate(tr("当前已有历史Session或标定/验证结果。请先确认旧数据无需继续，然后点击“新建/重置本轮Session”，再重建实验模板。"));return;
    }
    const double v0=m_templateMinSpeed?m_templateMinSpeed->value():0.5;
    const double v1=m_templateMaxSpeed?m_templateMaxSpeed->value():5.0;
    const double dv=m_templateSpeedStep?m_templateSpeedStep->value():0.5;
    const double dist=m_templateDistance?m_templateDistance->value():30.0;
    const int calRep=m_templateCalibrationRepeats?m_templateCalibrationRepeats->value():3;
    const double valSpeed=m_templateValidationSpeed?m_templateValidationSpeed->value():4.0;
    const int valRep=m_templateValidationRepeats?m_templateValidationRepeats->value():3;
    if(!(v0>0.0)||!(v1>=v0)||!(dv>0.0)||!(dist>0.0)||calRep<1||valRep<1){
        if(m_status)m_status->setText(tr("多速度实验模板参数无效：要求0<起始速度≤终止速度、步长/距离>0、重复≥1。"));
        return;
    }
    const int levels=int(std::floor((v1-v0)/dv+1e-9))+1;
    if(levels<1||levels>100){if(m_status)m_status->setText(tr("速度档数量异常，请检查起止速度和步长。"));return;}
    m_planConfigTable->setRowCount(0);
    // V27.6.50: deliberately interleave low/high speeds so elapsed experiment
    // time (thermal drift) is not strongly correlated with commanded speed.
    // Example for 0.5..5.0 / 0.5: 0.5,5.0,1.0,4.5,...,2.5,3.0.
    std::vector<double> speedOrder;
    speedOrder.reserve(std::size_t(levels));
    int lo=0,hi=levels-1;
    while(lo<=hi){
        speedOrder.push_back(v0+double(lo)*dv);
        ++lo;
        if(lo<=hi){speedOrder.push_back(v0+double(hi)*dv);--hi;}
    }
    for(const double speed:speedOrder){
        if(speed>v1+1e-9)continue;
        addPlanRow();const int r=m_planConfigTable->rowCount()-1;
        m_planConfigTable->item(r,1)->setText(tr("标定"));
        m_planConfigTable->item(r,2)->setText(tr("X_%1").arg(speed,0,'f',1));
        m_planConfigTable->item(r,3)->setText("1");m_planConfigTable->item(r,4)->setText("0");m_planConfigTable->item(r,5)->setText("0");
        m_planConfigTable->item(r,6)->setText(QString::number(speed,'f',2));m_planConfigTable->item(r,7)->setText(QString::number(dist,'f',1));m_planConfigTable->item(r,8)->setText(QString::number(calRep));
        m_planConfigTable->item(r,9)->setText(tr("X主扫掠；%1个相邻正反Pair；速度档按低/高交错，重复方向顺序按 +−/−+/+− 交替以抑制慢漂移").arg(calRep));
    }
    struct V{const char* name;double x,y,z;};
    const V validations[]={{"XY斜线",1,0.20,0},{"XZ斜线",1,0,0.20},{"XYZ斜线",1,0.20,0.20}};
    for(const auto& d:validations){
        addPlanRow();const int r=m_planConfigTable->rowCount()-1;
        m_planConfigTable->item(r,1)->setText(tr("验证"));m_planConfigTable->item(r,2)->setText(QString::fromUtf8(d.name));
        m_planConfigTable->item(r,3)->setText(QString::number(d.x));m_planConfigTable->item(r,4)->setText(QString::number(d.y));m_planConfigTable->item(r,5)->setText(QString::number(d.z));
        m_planConfigTable->item(r,6)->setText(QString::number(valSpeed,'f',2));m_planConfigTable->item(r,7)->setText(QString::number(dist,'f',1));m_planConfigTable->item(r,8)->setText(QString::number(valRep));
        m_planConfigTable->item(r,9)->setText(tr("独立Hold-out；%1个相邻正反Pair，不参与τ求解").arg(valRep));
    }
    const int calPairs=levels*calRep;const int valPairs=3*valRep;
    double pureScanSec=0.0;for(int i=0;i<levels;++i){const double speed=v0+double(i)*dv;if(speed>v1+1e-9)break;pureScanSec+=2.0*double(calRep)*dist/speed;}pureScanSec+=2.0*double(valPairs)*dist/std::max(1e-9,valSpeed);
    m_plannedScans.clear();m_calibrationObservations.clear();m_validationCaptures.clear();m_result={};m_validationResult={};m_haveResult=false;m_validationPassed=false;m_calibrationId.clear();refreshPathTable();refreshCaptureTable();refreshResult();
    if(m_status)m_status->setText(tr("多速度实验模板已生成：%1个速度档 × %2 Pair = %3个标定Pair（%4条扫描）；验证=%5个Pair（%6条扫描）；纯扫描时间约%7 min（不含定位/Arm/Drain）。下一步读取/确认扫描轨迹中点并生成路径。")
        .arg(levels).arg(calRep).arg(calPairs).arg(calPairs*2).arg(valPairs).arg(valPairs*2).arg(pureScanSec/60.0,0,'f',1));
    if(m_pathGenerationFeedback)m_pathGenerationFeedback->setText(tr("实验配置模板已生成，但可执行路径还没有生成。请点击“根据配置生成路径”。"));
    emit logMessage(tr("[硬件同步标定][TEMPLATE][H%1] %2个标定Pair / %3个验证Pair；仅生成配置，未生成路径。")
        .arg(m_selectedTopologyId).arg(calPairs).arg(valPairs));
    if(m_minPairs)m_minPairs->setValue(std::max(3,int(std::ceil(calPairs*2.0/3.0))));
    if(m_minSpeedLevels)m_minSpeedLevels->setValue(std::max(2,int(std::ceil(levels*0.75))));
    if(m_minValidationPairs)m_minValidationPairs->setValue(std::max(2,int(std::ceil(valPairs*2.0/3.0))));
}
void HardwareSyncCalibrationWidget::removePlanRow(){const int r=m_planConfigTable->currentRow();if(r>=0)m_planConfigTable->removeRow(r);}

void HardwareSyncCalibrationWidget::generatePaths()
{
    // All early-return reasons must be visible at the clicked button and in
    // exported logs. A hidden status label previously looked like a dead button.
    const QString sessionLabel=m_captureSessionId.isEmpty() ? tr("无") : m_captureSessionId;
    emit logMessage(tr("[硬件同步标定][PATH-REQUEST][H%1] 球心有效=%2 中点有效=%3 配置行=%4 现有路径=%5 旧Session=%6 已采标定=%7 已采验证=%8 运行中=%9")
        .arg(m_selectedTopologyId).arg(m_referenceFitValid).arg(m_planCenterValid)
        .arg(m_planConfigTable?m_planConfigTable->rowCount():0).arg(m_plannedScans.size())
        .arg(sessionLabel).arg(m_calibrationObservations.size()).arg(m_validationCaptures.size())
        .arg(m_workflowMode!=WorkflowMode::Idle||m_captureProcessing));
    const auto reject = [this,sessionLabel](const QString& reason) {
        const QString message=tr("路径生成失败 [H%1]：%2").arg(m_selectedTopologyId).arg(reason);
        if(m_pathGenerationFeedback)m_pathGenerationFeedback->setText(message);
        if(m_status)m_status->setText(message);
        emit logMessage(tr("[硬件同步标定][PATH-REJECT][H%1] %2；Session=%3；标定结果=%4；验证结果=%5")
            .arg(m_selectedTopologyId).arg(reason).arg(sessionLabel)
            .arg(m_calibrationObservations.size()).arg(m_validationCaptures.size()));
        QMessageBox::warning(this,tr("无法生成扫描路径"),message);
    };
    if(m_workflowMode!=WorkflowMode::Idle || m_captureProcessing || (m_sync&&m_sync->isRunning())){
        reject(tr("当前硬件采集/后台处理流程运行中，不能重新生成路径"));return;
    }
    if(!m_referenceFitValid){reject(tr("请先完成步骤1参考球球心拟合"));return;}
    if(!m_planCenterValid){reject(tr("请先点击“示教当前XYZCB为扫描轨迹中点”"));return;}
    if(hasAcceptedPlans(ScanRole::Calibration)||hasAcceptedPlans(ScanRole::Validation)||!m_captureSessionId.isEmpty()||
       !m_calibrationObservations.empty()||!m_validationCaptures.empty()){
        reject(tr("检测到旧Session或已保存的球心采集结果：Session=%1、标定结果%2条、验证结果%3条、规划路径%4条。新建路径会覆盖Plan ID，可能混用历史RAW。请先确认旧Session不需要继续；再点击第3步“新建/重置本轮Session”，然后重新生成路径。磁盘上已保存的RAW不会删除。")
            .arg(sessionLabel).arg(m_calibrationObservations.size()).arg(m_validationCaptures.size()).arg(m_plannedScans.size()));
        return;
    }
    m_plannedScans.clear();m_nextPlanId=1;m_nextPairId=1;
    for(int r=0;r<m_planConfigTable->rowCount();++r){auto* en=m_planConfigTable->item(r,0);if(!en||en->checkState()!=Qt::Checked)continue;const QString role=m_planConfigTable->item(r,1)?m_planConfigTable->item(r,1)->text().trimmed():QString();const bool validation=role.contains(tr("验证"));const QString name=m_planConfigTable->item(r,2)?m_planConfigTable->item(r,2)->text():tr("路径%1").arg(r+1);
        bool complete=true;
        for(int c=3;c<=8;++c)complete=complete && (m_planConfigTable->item(r,c)!=nullptr);
        if(!complete){
            m_plannedScans.clear();refreshPathTable();
            reject(tr("第%1行缺少方向权重、速度、距离或重复Pair单元格").arg(r+1));return;
        }
        bool okx=false,oky=false,okz=false,oks=false,okd=false,okr=false;
        const double x=m_planConfigTable->item(r,3)->text().toDouble(&okx);
        const double y=m_planConfigTable->item(r,4)->text().toDouble(&oky);
        const double z=m_planConfigTable->item(r,5)->text().toDouble(&okz);
        const double speed=m_planConfigTable->item(r,6)->text().toDouble(&oks);
        const double dist=m_planConfigTable->item(r,7)->text().toDouble(&okd);
        const int reps=m_planConfigTable->item(r,8)->text().toInt(&okr);
        if(!okx||!oky||!okz||!oks||!okd||!okr||!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(z)||!std::isfinite(speed)||!std::isfinite(dist)||speed<=0||dist<=0||reps<1){
            m_plannedScans.clear();refreshPathTable();
            reject(tr("配置第%1行参数无效：检查XYZ方向权重、速度、距离与重复Pair").arg(r+1));return;
        }
        const double n=std::sqrt(x*x+y*y+z*z);
        if(n<1e-9||!std::isfinite(n)){
            m_plannedScans.clear();refreshPathTable();
            reject(tr("配置第%1行XYZ方向权重全为0或存在非有限值").arg(r+1));return;
        }
        QVector3D u(float(x/n),float(y/n),float(z/n));
        for(int rep=0;rep<reps;++rep){
            const quint64 pair=m_nextPairId++;
            JointPose5D a=m_planCenter,b=m_planCenter;
            a.x-=0.5*dist*u.x();a.y-=0.5*dist*u.y();a.z-=0.5*dist*u.z();
            b.x+=0.5*dist*u.x();b.y+=0.5*dist*u.y();b.z+=0.5*dist*u.z();
            PlannedScan plus;plus.pairId=pair;plus.role=validation?ScanRole::Validation:ScanRole::Calibration;
            plus.name=name+(reps>1?tr(" R%1").arg(rep+1):QString());plus.directionSign=+1;
            plus.start=a;plus.end=b;plus.unitDirection=u;plus.requestedSpeedMmPerSec=speed;plus.distanceMm=dist;plus.status=tr("待执行");
            PlannedScan minus=plus;minus.directionSign=-1;minus.start=b;minus.end=a;
            // Alternate pair acquisition order (+,-), (-,+), (+,-)... so a
            // monotonic thermal drift does not enter every pair with the same sign.
            if((rep&1)==0){plus.id=m_nextPlanId++;m_plannedScans.push_back(plus);minus.id=m_nextPlanId++;m_plannedScans.push_back(minus);}
            else{minus.id=m_nextPlanId++;m_plannedScans.push_back(minus);plus.id=m_nextPlanId++;m_plannedScans.push_back(plus);}
        }
    }
    if(m_plannedScans.empty()){
        refreshPathTable();reject(tr("当前没有任何已启用的有效路径配置；请检查配置表“启用”列"));return;
    }
    refreshPathTable();
    const QString okText=tr("路径生成成功：%1条扫描（%2个正反Pair）。请先核对全部起止点、行程和碰撞安全，再启动标定采集。")
        .arg(m_plannedScans.size()).arg(m_plannedScans.size()/2);
    if(m_pathGenerationFeedback)m_pathGenerationFeedback->setText(okText);
    if(m_status)m_status->setText(okText);
    emit logMessage(tr("[硬件同步标定][PATH-OK][H%1] 可执行路径队列已生成：%2条（%3个Pair），扫描轨迹中点XYZ=(%4,%5,%6)，C/B=%7/%8°，尚未执行、RAW列应为空。")
        .arg(m_selectedTopologyId).arg(m_plannedScans.size()).arg(m_plannedScans.size()/2)
        .arg(m_planCenter.x,0,'f',3).arg(m_planCenter.y,0,'f',3).arg(m_planCenter.z,0,'f',3)
        .arg(m_planCenter.c,0,'f',3).arg(m_planCenter.b,0,'f',3));
    updateWorkflowButtons();
}

void HardwareSyncCalibrationWidget::startCalibrationAcquisition()
{
    if(m_captureProcessing){m_status->setText(tr("后台仍在处理上一条扫描，请等待完成"));return;}
    if(!m_sync||!m_controller){m_status->setText(tr("控制器/同步管理器未绑定"));return;}
    if(m_sync->hardwareTemporalCorrection().enabled){m_status->setText(tr("标定采集必须使用RAW REGIST位姿；请先禁用时域补偿"));return;}
    if(!m_referenceFitValid||m_plannedScans.empty()){m_status->setText(tr("先完成参考球和路径生成"));return;}
    if(allPlansComplete(ScanRole::Calibration)){
        m_status->setText(tr("所有标定路径已经完成，无需重复采集。可直接点击“求解 τ”；如需重做请先点击“新建/重置本轮Session”。"));
        return;
    }

    // V27.6.50: starting again means RESUME by default.  A new session is only
    // created when there is no current checkpoint/session.  Existing accepted
    // scans and their lightweight observations are never silently cleared.
    if(m_captureSessionId.isEmpty()){
        m_captureSessionId=QStringLiteral("HTRUN51-%1").arg(QDateTime::currentDateTimeUtc().toString("yyyyMMdd-hhmmsszzz"));
        m_captureSessionDirectory.clear();
    }
    {
        QString dirError;
        if(ensureCaptureSessionDirectory(&dirError).isEmpty()){m_status->setText(dirError);return;}
        emit logMessage(tr("[硬件同步标定] 当前断点Session：%1；RAW归档=%2")
            .arg(QDir::toNativeSeparators(m_captureSessionDirectory))
            .arg(m_archiveEachScan&&m_archiveEachScan->isChecked()?tr("ON"):tr("OFF")));
    }

    // Any new calibration observation invalidates a previously solved candidate
    // and every validation result, but already accepted calibration scans remain.
    m_haveResult=false;m_result={};m_validationPassed=false;m_validationResult={};
    m_validationCaptures.clear();m_calibrationId.clear();
    m_workflowMode=WorkflowMode::Calibration;m_workflowPlanIndex=-1;m_pendingRequestId=0;
    for(auto&p:m_plannedScans)if(p.role==ScanRole::Calibration&&!p.accepted){
        p.completed=false;p.consecutiveFailures=0;
        if(p.status.startsWith(tr("失败"))||p.status.startsWith(tr("后台处理失败")))p.status=tr("待重试");
    }
    refreshCaptureTable();refreshPathTable();refreshResult();
    QString checkpointError;
    if(!saveSessionCheckpoint(&checkpointError) && !checkpointError.isEmpty())
        emit logMessage(tr("[硬件同步标定][断点] Session检查点保存失败：%1").arg(checkpointError));
    const int done=acceptedPlanCount(ScanRole::Calibration);
    emit logMessage(tr("[硬件同步标定][断点] 开始/继续标定：已完成%1/%2条；将从首个未完成路径继续。")
        .arg(done).arg(planCount(ScanRole::Calibration)));
    requestNextWorkflowScan();
}

void HardwareSyncCalibrationWidget::startValidationAcquisition()
{
    if(m_captureProcessing){m_status->setText(tr("后台仍在处理上一条扫描，请等待完成"));return;}
    if(!m_haveResult||!m_result.qualityPassed){m_status->setText(tr("必须先完成标定求解并通过质量门限"));return;}
    if(m_sync&&m_sync->hardwareTemporalCorrection().enabled){m_status->setText(tr("Hold-out验证必须RAW采集并离线对比；请先禁用运行时补偿"));return;}
    bool any=false;for(const auto&p:m_plannedScans)if(p.role==ScanRole::Validation)any=true;
    if(!any){m_status->setText(tr("当前队列没有验证路径；可修改配置为“验证”后重新生成路径"));return;}
    if(allPlansComplete(ScanRole::Validation)){
        m_status->setText(tr("所有验证路径已经完成。可直接点击“评估验证结果”；如需重做请新建Session或清空验证数据。"));
        return;
    }
    if(m_captureSessionId.isEmpty())m_captureSessionId=QStringLiteral("HTRUN51-%1").arg(QDateTime::currentDateTimeUtc().toString("yyyyMMdd-hhmmsszzz"));
    {
        QString dirError;if(ensureCaptureSessionDirectory(&dirError).isEmpty()){m_status->setText(dirError);return;}
    }
    m_validationPassed=false;m_validationResult={};m_workflowMode=WorkflowMode::Validation;m_workflowPlanIndex=-1;m_pendingRequestId=0;
    for(auto&p:m_plannedScans)if(p.role==ScanRole::Validation&&!p.accepted){
        p.completed=false;p.consecutiveFailures=0;
        if(p.status.startsWith(tr("失败"))||p.status.startsWith(tr("后台处理失败")))p.status=tr("待重试");
    }
    refreshCaptureTable();refreshPathTable();refreshResult();
    QString checkpointError;if(!saveSessionCheckpoint(&checkpointError)&&!checkpointError.isEmpty())
        emit logMessage(tr("[硬件同步标定][断点] Session检查点保存失败：%1").arg(checkpointError));
    emit logMessage(tr("[硬件同步标定][断点] 开始/继续验证：已完成%1/%2条；将从首个未完成路径继续。")
        .arg(acceptedPlanCount(ScanRole::Validation)).arg(planCount(ScanRole::Validation)));
    requestNextWorkflowScan();
}

std::size_t HardwareSyncCalibrationWidget::planCount(ScanRole role) const
{
    std::size_t n=0;
    for(const auto& p:m_plannedScans) if(p.role==role) ++n;
    return n;
}

bool HardwareSyncCalibrationWidget::hasAcceptedPlans(ScanRole role) const
{
    for(const auto& p:m_plannedScans) if(p.role==role&&p.accepted&&p.completed) return true;
    return false;
}

int HardwareSyncCalibrationWidget::acceptedPlanCount(ScanRole role) const
{
    int n=0;for(const auto& p:m_plannedScans) if(p.role==role&&p.accepted&&p.completed) ++n;return n;
}

bool HardwareSyncCalibrationWidget::allPlansComplete(ScanRole role) const
{
    const auto n=planCount(role);
    if(n==0) return false;
    for(const auto& p:m_plannedScans)
        if(p.role==role && (!p.completed || !p.accepted)) return false;
    return true;
}

void HardwareSyncCalibrationWidget::requestNextWorkflowScan()
{
    if(m_workflowMode==WorkflowMode::Idle)return;
    const ScanRole role=m_workflowMode==WorkflowMode::Calibration?ScanRole::Calibration:ScanRole::Validation;
    int next=-1;
    for(int i=m_workflowPlanIndex+1;i<int(m_plannedScans.size());++i){
        const auto& p=m_plannedScans[std::size_t(i)];
        if(p.role==role&&!p.completed&&!p.accepted){next=i;break;}
    }
    if(next<0){
        m_workflowMode=WorkflowMode::Idle;m_pendingRequestId=0;m_workflowPlanIndex=-1;
        refreshPathTable();
        if(role==ScanRole::Calibration)m_status->setText(tr("所有标定路径采集完成。现在点击“求解 τ”。"));
        else m_status->setText(tr("所有Hold-out验证路径采集完成。现在点击“评估验证结果”。"));
        QString checkpointError;if(!saveSessionCheckpoint(&checkpointError)&&!checkpointError.isEmpty())
            emit logMessage(tr("[硬件同步标定][断点] 完成态检查点保存失败：%1").arg(checkpointError));
        updateWorkflowButtons();return;
    }
    m_workflowPlanIndex=next;auto& p=m_plannedScans[std::size_t(next)];
    ++p.attemptCount;
    p.status=p.attemptCount>1?tr("执行中(A%1)").arg(p.attemptCount):tr("执行中");
    p.lastError.clear();
    m_pendingRequestId=p.id;refreshPathTable();
    m_status->setText(tr("请求执行 #%1 %2 Pair%3 %4（第%5次尝试）；将自动定位起点→Arm原硬件同步→扫描→Stop/Drain→采入。")
        .arg(p.id).arg(p.name).arg(p.pairId).arg(p.directionSign>0?"+":"-").arg(p.attemptCount));
    QString checkpointError;if(!saveSessionCheckpoint(&checkpointError)&&!checkpointError.isEmpty())
        emit logMessage(tr("[硬件同步标定][断点] 启动路径前检查点保存失败：%1").arg(checkpointError));
    emit requestPlannedHardwareScan(p.id,p.start,p.end,p.requestedSpeedMmPerSec);updateWorkflowButtons();
}

void HardwareSyncCalibrationWidget::handlePlanFailure(PlannedScan& p,const QString& reason,bool allowAutoRetry)
{
    p.completed=false;p.accepted=false;++p.failureCount;++p.consecutiveFailures;p.lastError=reason;
    const int maxRetry=m_autoRetryCount?m_autoRetryCount->value():0;

    if(allowAutoRetry&&m_workflowMode!=WorkflowMode::Idle&&p.consecutiveFailures<=maxRetry){
        p.status=tr("失败，等待自动重试 %1/%2：%3").arg(p.consecutiveFailures).arg(maxRetry).arg(reason);
        QString checkpointError;if(!saveSessionCheckpoint(&checkpointError)&&!checkpointError.isEmpty())
            emit logMessage(tr("[硬件同步标定][断点] 失败态检查点保存失败：%1").arg(checkpointError));
        m_status->setText(tr("路径 #%1 本次失败，但前面已完成数据均保留。%2 ms后自动重试本路径（%3/%4）：%5")
            .arg(p.id).arg(1500).arg(p.consecutiveFailures).arg(maxRetry).arg(reason));
        emit logMessage(tr("[硬件同步标定][断点] %1").arg(m_status->text()));
        const WorkflowMode mode=m_workflowMode;
        const int failedIndex=int(std::distance(m_plannedScans.begin(),std::find_if(m_plannedScans.begin(),m_plannedScans.end(),[&p](const PlannedScan& q){return q.id==p.id;})));
        m_workflowPlanIndex=failedIndex-1;m_pendingRequestId=0;refreshPathTable();updateWorkflowButtons();
        QTimer::singleShot(1500,this,[this,mode](){
            if(m_workflowMode==mode&&!m_captureProcessing&&m_pendingRequestId==0)requestNextWorkflowScan();
        });
        return;
    }

    p.status=tr("失败待继续：%1").arg(reason);
    QString checkpointError;if(!saveSessionCheckpoint(&checkpointError)&&!checkpointError.isEmpty())
        emit logMessage(tr("[硬件同步标定][断点] 暂停失败态检查点保存失败：%1").arg(checkpointError));
    const quint64 failedId=p.id;
    m_workflowMode=WorkflowMode::Idle;m_workflowPlanIndex=-1;m_pendingRequestId=0;
    refreshPathTable();
    m_status->setText(tr("路径 #%1 失败并暂停。本Session已完成的数据和RAW归档全部保留；修复原因后再次点击“开始/继续”会从本路径重试，不会从路径1重来。原因：%2")
        .arg(failedId).arg(reason));
    emit logMessage(tr("[硬件同步标定][断点] %1").arg(m_status->text()));updateWorkflowButtons();
}

void HardwareSyncCalibrationWidget::notifyPlannedScanFinished(quint64 requestId,bool success,const QString& message)
{
    if(m_workflowMode==WorkflowMode::Idle||requestId!=m_pendingRequestId||m_workflowPlanIndex<0||m_workflowPlanIndex>=int(m_plannedScans.size()))return;
    auto& p=m_plannedScans[std::size_t(m_workflowPlanIndex)];
    m_pendingRequestId=0;
    if(!success){
        handlePlanFailure(p,message,retryablePlannedScanFailure(message));
        return;
    }

    // A successful hardware closure is not marked completed until archive +
    // analysis also succeeds. This makes the session manifest transactional.
    startCaptureProcessing(p,message);
}

void HardwareSyncCalibrationWidget::abortWorkflow()
{
    if(m_workflowMode==WorkflowMode::Idle && !m_captureProcessing)return;
    m_workflowMode=WorkflowMode::Idle;
    m_workflowPlanIndex=-1;
    m_pendingRequestId=0;
    ++m_captureProcessingGeneration; // any worker result becomes non-authoritative
    emit requestAbortPlannedHardwareScan();
    QString checkpointError;if(!saveSessionCheckpoint(&checkpointError)&&!checkpointError.isEmpty())
        emit logMessage(tr("[硬件同步标定][断点] 暂停检查点保存失败：%1").arg(checkpointError));
    m_status->setText(m_captureProcessing
        ? tr("已暂停自动路径流程；后台正在结束当前条的归档/分析。之前已完成数据不会清空，稍后可点击“开始/继续”恢复。")
        : tr("已暂停当前硬件同步标定流程；之前已完成数据和RAW归档均保留，可随时从首个未完成路径继续。"));
    updateWorkflowButtons();
}

QString HardwareSyncCalibrationWidget::ensureCaptureSessionDirectory(QString* error)
{
    QString root=m_captureRootDirectory?m_captureRootDirectory->text().trimmed():QString();
    if(root.isEmpty())root=QStringLiteral("E:/Projects/CData");
    QDir rootDir(root);
    if(!rootDir.exists()&&!rootDir.mkpath(QStringLiteral("."))){
        if(error)*error=tr("无法创建RAW归档根目录：%1").arg(root);
        return {};
    }
    if(m_captureSessionId.isEmpty())
        m_captureSessionId=QStringLiteral("HTRUN51-%1").arg(QDateTime::currentDateTimeUtc().toString("yyyyMMdd-hhmmsszzz"));
    const QString sub=QStringLiteral("HardwareTemporalCalibration/%1").arg(m_captureSessionId);
    if(!rootDir.mkpath(sub)){
        if(error)*error=tr("无法创建本次硬件同步标定目录：%1").arg(rootDir.filePath(sub));
        return {};
    }
    m_captureSessionDirectory=rootDir.filePath(sub);
    if(error)error->clear();
    return m_captureSessionDirectory;
}

QString HardwareSyncCalibrationWidget::captureArchiveBaseName(const PlannedScan& plan) const
{
    const QString role=plan.role==ScanRole::Validation?QStringLiteral("VAL"):QStringLiteral("CAL");
    const QString sign=plan.directionSign>0?QStringLiteral("PLUS"):QStringLiteral("MINUS");
    const QString attempt=plan.attemptCount>1?QStringLiteral("_A%1").arg(plan.attemptCount):QString();
    return QStringLiteral("%1_%2_Pair%3_%4_%5%6")
        .arg(role)
        .arg(qulonglong(plan.id),3,10,QLatin1Char('0'))
        .arg(qulonglong(plan.pairId),3,10,QLatin1Char('0'))
        .arg(safeFileToken(plan.name))
        .arg(sign)
        .arg(attempt);
}


QString HardwareSyncCalibrationWidget::sessionCheckpointPath() const
{
    if(m_captureSessionDirectory.trimmed().isEmpty()) return {};
    return QDir(m_captureSessionDirectory).filePath(QStringLiteral("session_manifest_v49.json"));
}

bool HardwareSyncCalibrationWidget::saveSessionCheckpoint(QString* error) const
{
    const QString path=sessionCheckpointPath();
    if(path.isEmpty()){if(error)error->clear();return true;}

    QJsonObject root;
    root["schemaVersion"]=2;
    root["softwareVersion"]=QStringLiteral("V32.16-H1-H3-SIGNED-TEMPORAL");
    root["dynamicSphereFitContract"]=kDynamicSphereFitContract;
    root["topologyId"]=m_selectedTopologyId;
    root["latchEdge"]=m_sessionLatchEdge;
    root["cmmDivider"]=(m_selectedTopologyId==3?1:0);
    root["cmmSkewHalfUs"]=0;
    root["cmmRatio"]=1;
    root["topology"]=m_selectedTopologyId==1
        ? QStringLiteral("H1 OP14->IN0/R0 REGIST(104) full-MPOS")
        : QStringLiteral("H3 sensor CMM->IN1/R1 REGIST(114/115) full-MPOS");
    root["delayDefinition"]=QStringLiteral("t_sensor_effective - t_eci_latch");
    root["sessionId"]=m_captureSessionId;
    root["sessionDirectory"]=m_captureSessionDirectory;
    root["checkpointUtc"]=QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs);
    root["calibrationId"]=m_calibrationId;
    root["validationPassed"]=m_validationPassed;
    root["nextObservationId"]=QString::number(qulonglong(m_nextObservationId));
    root["nextPlanId"]=QString::number(qulonglong(m_nextPlanId));
    root["nextPairId"]=QString::number(qulonglong(m_nextPairId));

    QJsonObject ref;
    ref["valid"]=m_referenceFitValid;ref["cloudPath"]=m_referenceCloudPath;
    ref["centerMachineMm"]=vecJson(m_referenceFit.centerMachineMm);
    ref["radiusMm"]=m_referenceFit.knownRadiusMm;ref["rmsMm"]=m_referenceFit.residualRmsMm;
    ref["inliers"]=QString::number(qulonglong(m_referenceFit.pointCount));
    ref["freeRadiusMm"]=m_referenceFreeRadiusMm;
    ref["sphereFitComparison"]=m_referenceTwoStageDiagnostic;
    root["referenceSphere"]=ref;

    QJsonObject center=jointPoseJson(m_planCenter);center["valid"]=m_planCenterValid;root["planCenter"]=center;

    QJsonObject experiment;
    experiment["minSpeedMmPerSec"]=m_templateMinSpeed?m_templateMinSpeed->value():0.5;
    experiment["maxSpeedMmPerSec"]=m_templateMaxSpeed?m_templateMaxSpeed->value():5.0;
    experiment["speedStepMmPerSec"]=m_templateSpeedStep?m_templateSpeedStep->value():0.5;
    experiment["calibrationRepeatsPerSpeed"]=m_templateCalibrationRepeats?m_templateCalibrationRepeats->value():3;
    experiment["distanceMm"]=m_templateDistance?m_templateDistance->value():30.0;
    experiment["validationSpeedMmPerSec"]=m_templateValidationSpeed?m_templateValidationSpeed->value():4.0;
    experiment["validationRepeatsPerTrajectory"]=m_templateValidationRepeats?m_templateValidationRepeats->value():3;
    root["experimentTemplate"]=experiment;

    QJsonObject archive;
    archive["rootDirectory"]=m_captureRootDirectory?m_captureRootDirectory->text():QStringLiteral("E:/Projects/CData");
    archive["archiveEachScan"]=m_archiveEachScan&&m_archiveEachScan->isChecked();
    archive["maxAnalysisPoints"]=m_maxAnalysisPoints?m_maxAnalysisPoints->value():250000;
    archive["autoRetryCount"]=m_autoRetryCount?m_autoRetryCount->value():2;
    root["captureArchive"]=archive;

    QJsonArray configs;
    for(int r=0;r<m_planConfigTable->rowCount();++r){
        QJsonObject j;j["enabled"]=m_planConfigTable->item(r,0)&&m_planConfigTable->item(r,0)->checkState()==Qt::Checked;
        for(int c=1;c<kPlanCols;++c)j[QStringLiteral("c%1").arg(c)]=m_planConfigTable->item(r,c)?m_planConfigTable->item(r,c)->text():QString();
        configs.append(j);
    }
    root["planConfigs"]=configs;

    QJsonArray plans;
    for(const auto& p:m_plannedScans){
        QJsonObject j;
        j["id"]=QString::number(qulonglong(p.id));j["pairId"]=QString::number(qulonglong(p.pairId));
        j["role"]=p.role==ScanRole::Validation?QStringLiteral("validation"):QStringLiteral("calibration");
        j["name"]=p.name;j["directionSign"]=p.directionSign;
        j["start"]=jointPoseJson(p.start);j["end"]=jointPoseJson(p.end);
        QJsonArray u;u.append(p.unitDirection.x());u.append(p.unitDirection.y());u.append(p.unitDirection.z());j["unitDirection"]=u;
        j["requestedSpeedMmPerSec"]=p.requestedSpeedMmPerSec;j["distanceMm"]=p.distanceMm;
        j["completed"]=p.completed;j["accepted"]=p.accepted;j["status"]=p.status;
        j["archivePath"]=p.archivePath;j["metadataPath"]=p.metadataPath;
        j["attemptCount"]=p.attemptCount;j["failureCount"]=p.failureCount;
        j["consecutiveFailures"]=p.consecutiveFailures;j["lastError"]=p.lastError;
        j["restoredFromDisk"]=p.restoredFromDisk;
        plans.append(j);
    }
    root["plannedScans"]=plans;

    QJsonArray obs;for(const auto&o:m_calibrationObservations)obs.append(trajectoryObservationJson(o));root["calibrationObservations"]=obs;
    QJsonArray vals;
    for(const auto&vc:m_validationCaptures){
        const auto&o=vc.observation;QJsonObject j;
        j["id"]=QString::number(qulonglong(o.id));j["pairId"]=QString::number(qulonglong(o.pairId));j["directionSign"]=o.directionSign;
        j["velocityMmPerSec"]=vecJson(o.velocityMmPerSec);j["rawCenter"]=vecJson(o.rawSphereCenterMachineMm);j["correctedCenter"]=vecJson(o.correctedSphereCenterMachineMm);
        j["rawRmsMm"]=o.rawSphereRmsMm;j["correctedRmsMm"]=o.correctedSphereRmsMm;j["captureMidPcUs"]=QString::number(qlonglong(o.captureMidPcUs));
        j["referenceRawRmsMm"]=vc.referenceRawRmsMm;j["referenceCorrectedRmsMm"]=vc.referenceCorrectedRmsMm;j["inliers"]=QString::number(qulonglong(vc.inliers));
        vals.append(j);
    }
    root["validationObservations"]=vals;

    QJsonObject solved;
    solved["valid"]=m_haveResult&&m_result.success;
    if(m_haveResult&&m_result.success){solved["delaySec"]=m_result.delaySec;solved["delayStdSec"]=m_result.delayStdSec;solved["qualityPassed"]=m_result.qualityPassed;}
    root["candidateResult"]=solved;

    return saveJsonObjectAtomic(path,root,error);
}

bool HardwareSyncCalibrationWidget::loadSessionCheckpoint(const QString& directory,QString* error)
{
    const QString path=QDir(directory).filePath(QStringLiteral("session_manifest_v49.json"));
    QFile f(path);if(!f.open(QIODevice::ReadOnly)){if(error)*error=f.errorString();return false;}
    QJsonParseError pe;const auto doc=QJsonDocument::fromJson(f.readAll(),&pe);
    if(pe.error!=QJsonParseError::NoError||!doc.isObject()){if(error)*error=tr("Session断点JSON解析失败：%1").arg(pe.errorString());return false;}
    const auto root=doc.object();
    const int schema=root.value("schemaVersion").toInt();
    const int storedTopology=schema==1&&root.value("topology").toString().contains("REGIST(104)")
        ? 1 : root.value("topologyId").toInt();
    const int storedEdge=schema==1?104:root.value("latchEdge").toInt();
    if((schema!=1&&schema!=2) || storedTopology!=m_selectedTopologyId ||
        (schema==1 && m_selectedTopologyId!=1) ||
        (storedEdge!=0 && !mls::runtime::Table5D::validTopologyEdge(storedTopology,storedEdge)) ||
        (schema==2 && storedTopology==3 && (root.value("cmmDivider").toInt()!=1 ||
            root.value("cmmSkewHalfUs").toInt()!=0 || root.value("cmmRatio").toInt()!=1))) {
        if(error)*error=tr("Session的拓扑/锁存边沿/CMM设置与当前H%1不匹配；禁止混合H1/H3训练数据")
            .arg(m_selectedTopologyId);
        return false;
    }
    // A persisted sphere-center observation reflects a specific input-selection
    // algorithm. Mixing old unfiltered and new arc-qualified centers can create
    // a fictitious delay, even when all hardware timestamp checks pass.
    const bool hasOldObservations = !root.value("calibrationObservations").toArray().isEmpty()
        || !root.value("validationObservations").toArray().isEmpty();
    if(hasOldObservations &&
       root.value("dynamicSphereFitContract").toString()!=kDynamicSphereFitContract) {
        if(error)*error=tr("旧Session中包含旧版全点RANSAC球心，禁止与新的H1/H3两阶段球心混合求解τ。"
            "原始RAW保持不变；请新建Session重新采集，或使用独立离线程序统一重分析旧RAW。");
        return false;
    }
    m_sessionLatchEdge=storedEdge;

    const auto ref=root.value("referenceSphere").toObject();
    m_referenceFitValid=ref.value("valid").toBool(false);
    m_referenceCloudPath=ref.value("cloudPath").toString();
    m_referenceFit.centerMachineMm=jsonVec(ref.value("centerMachineMm"));
    m_referenceFit.knownRadiusMm=ref.value("radiusMm").toDouble();m_referenceFit.residualRmsMm=ref.value("rmsMm").toDouble();
    m_referenceFit.pointCount=std::size_t(ref.value("inliers").toString().toULongLong());m_referenceFit.success=m_referenceFitValid;
    m_referenceFreeRadiusMm=ref.value("freeRadiusMm").toDouble();m_referenceInliers=m_referenceFit.pointCount;
    m_referenceTwoStageDiagnostic=ref.value("sphereFitComparison").toString();
    if(m_referenceTwoStageDiagnostic.isEmpty() && m_referenceFitValid)
        m_referenceTwoStageDiagnostic=tr("旧Session恢复的原参考球结果，未执行本版两阶段圆弧筛选；若重新导入，请重新拟合并复核规划路径。");
    m_referencePath->setText(m_referenceCloudPath);
    if(m_referenceFitValid)emit referenceSphereChanged(m_referenceFit.centerMachineMm.x,m_referenceFit.centerMachineMm.y,m_referenceFit.centerMachineMm.z,m_referenceFit.knownRadiusMm,true);

    const auto center=root.value("planCenter").toObject();m_planCenterValid=center.value("valid").toBool(false);m_planCenter=jointPoseFromJson(center);
    if(m_planCenterValid)m_planCenterLabel->setText(tr("扫描轨迹中点 XYZ=(%1,%2,%3) C/B=(%4,%5)").arg(m_planCenter.x,0,'f',4).arg(m_planCenter.y,0,'f',4).arg(m_planCenter.z,0,'f',4).arg(m_planCenter.c,0,'f',3).arg(m_planCenter.b,0,'f',3));

    const auto ex=root.value("experimentTemplate").toObject();
    if(m_templateMinSpeed)m_templateMinSpeed->setValue(ex.value("minSpeedMmPerSec").toDouble(0.5));if(m_templateMaxSpeed)m_templateMaxSpeed->setValue(ex.value("maxSpeedMmPerSec").toDouble(5.0));if(m_templateSpeedStep)m_templateSpeedStep->setValue(ex.value("speedStepMmPerSec").toDouble(0.5));if(m_templateCalibrationRepeats)m_templateCalibrationRepeats->setValue(ex.value("calibrationRepeatsPerSpeed").toInt(3));if(m_templateDistance)m_templateDistance->setValue(ex.value("distanceMm").toDouble(30.0));if(m_templateValidationSpeed)m_templateValidationSpeed->setValue(ex.value("validationSpeedMmPerSec").toDouble(4.0));if(m_templateValidationRepeats)m_templateValidationRepeats->setValue(ex.value("validationRepeatsPerTrajectory").toInt(3));
    const auto ar=root.value("captureArchive").toObject();if(m_captureRootDirectory)m_captureRootDirectory->setText(ar.value("rootDirectory").toString(QStringLiteral("E:/Projects/CData")));if(m_archiveEachScan)m_archiveEachScan->setChecked(ar.value("archiveEachScan").toBool(true));if(m_maxAnalysisPoints)m_maxAnalysisPoints->setValue(ar.value("maxAnalysisPoints").toInt(250000));if(m_autoRetryCount)m_autoRetryCount->setValue(ar.value("autoRetryCount").toInt(2));

    m_planConfigTable->setRowCount(0);for(const auto&v:root.value("planConfigs").toArray()){const auto j=v.toObject();addPlanRow();const int r=m_planConfigTable->rowCount()-1;m_planConfigTable->item(r,0)->setCheckState(j.value("enabled").toBool(true)?Qt::Checked:Qt::Unchecked);for(int c=1;c<kPlanCols;++c)m_planConfigTable->item(r,c)->setText(j.value(QStringLiteral("c%1").arg(c)).toString());}

    m_plannedScans.clear();
    for(const auto&v:root.value("plannedScans").toArray()){
        const auto j=v.toObject();PlannedScan p;
        p.id=j.value("id").toString().toULongLong();p.pairId=j.value("pairId").toString().toULongLong();p.role=j.value("role").toString()==QStringLiteral("validation")?ScanRole::Validation:ScanRole::Calibration;p.name=j.value("name").toString();p.directionSign=j.value("directionSign").toInt(1);p.start=jointPoseFromJson(j.value("start").toObject());p.end=jointPoseFromJson(j.value("end").toObject());const auto u=j.value("unitDirection").toArray();if(u.size()==3)p.unitDirection=QVector3D(float(u[0].toDouble()),float(u[1].toDouble()),float(u[2].toDouble()));p.requestedSpeedMmPerSec=j.value("requestedSpeedMmPerSec").toDouble();p.distanceMm=j.value("distanceMm").toDouble();p.completed=j.value("completed").toBool(false);p.accepted=j.value("accepted").toBool(false);p.status=j.value("status").toString();p.archivePath=j.value("archivePath").toString();p.metadataPath=j.value("metadataPath").toString();p.attemptCount=j.value("attemptCount").toInt();p.failureCount=j.value("failureCount").toInt();p.consecutiveFailures=0;p.lastError=j.value("lastError").toString();p.restoredFromDisk=true;
        if(p.accepted&&p.completed)p.status=tr("完成/断点恢复");else if(!p.accepted&&p.failureCount>0)p.status=tr("失败待继续");else p.status=tr("待执行");
        m_plannedScans.push_back(p);
    }

    m_calibrationObservations.clear();for(const auto&v:root.value("calibrationObservations").toArray()){TemporalTrajectoryObservation o;if(trajectoryObservationFromJson(v.toObject(),o))m_calibrationObservations.push_back(o);}
    // V49 manifests did not store capture midpoint timestamps. When RAW files are
    // still present, recover host-clock midpoints from ArchiveProfile::timeUs so
    // the V50 drift-aware solver can reuse the old session without re-acquiring.
    for(auto& o:m_calibrationObservations)if(o.captureMidPcUs<=0){
        auto pit=std::find_if(m_plannedScans.begin(),m_plannedScans.end(),[&](const PlannedScan& p){return p.id==o.planId;});
        if(pit!=m_plannedScans.end()&&!pit->archivePath.isEmpty()){QString raw=pit->archivePath;if(!QFileInfo::exists(raw))raw=QDir(directory).filePath(QFileInfo(raw).fileName());qint64 mid=0;QString te;if(QFileInfo::exists(raw)&&inferCaptureMidPcUsFromArchivedScan(raw,mid,&te))o.captureMidPcUs=mid;}
    }
    m_validationCaptures.clear();for(const auto&v:root.value("validationObservations").toArray()){const auto j=v.toObject();ValidationCapture vc;vc.observation.id=j.value("id").toString().toULongLong();vc.observation.pairId=j.value("pairId").toString().toULongLong();vc.observation.directionSign=j.value("directionSign").toInt();vc.observation.velocityMmPerSec=jsonVec(j.value("velocityMmPerSec"));vc.observation.rawSphereCenterMachineMm=jsonVec(j.value("rawCenter"));vc.observation.correctedSphereCenterMachineMm=jsonVec(j.value("correctedCenter"));vc.observation.rawSphereRmsMm=j.value("rawRmsMm").toDouble();vc.observation.correctedSphereRmsMm=j.value("correctedRmsMm").toDouble();vc.observation.captureMidPcUs=j.value("captureMidPcUs").toString().toLongLong();vc.referenceRawRmsMm=j.value("referenceRawRmsMm").toDouble();vc.referenceCorrectedRmsMm=j.value("referenceCorrectedRmsMm").toDouble();vc.inliers=std::size_t(j.value("inliers").toString().toULongLong());m_validationCaptures.push_back(vc);}

    m_captureSessionId=root.value("sessionId").toString(QFileInfo(directory).fileName());m_captureSessionDirectory=QDir(directory).absolutePath();
    m_calibrationId=root.value("calibrationId").toString();m_nextObservationId=root.value("nextObservationId").toString().toULongLong();m_nextPlanId=root.value("nextPlanId").toString().toULongLong();m_nextPairId=root.value("nextPairId").toString().toULongLong();
    if(m_nextObservationId<1)m_nextObservationId=1;if(m_nextPlanId<1)m_nextPlanId=1;if(m_nextPairId<1)m_nextPairId=1;
    m_result={};m_haveResult=false;m_validationResult={};m_validationPassed=false;
    if(allPlansComplete(ScanRole::Calibration)&&m_calibrationObservations.size()>=2){m_result=HardwareTemporalCalibration::solveTrajectoryPairs(m_calibrationObservations);m_haveResult=m_result.success;if(m_haveResult)HardwareTemporalCalibration::evaluateTrajectoryQuality(m_result,trajectoryCriteriaFromUi());}
    if(m_haveResult&&m_result.qualityPassed&&allPlansComplete(ScanRole::Validation)&&!m_validationCaptures.empty()){std::vector<TemporalValidationObservation> v;for(const auto&c:m_validationCaptures)v.push_back(c.observation);m_validationResult=HardwareTemporalCalibration::evaluateValidationPairs(v,validationCriteriaFromUi());m_validationPassed=m_validationResult.success&&m_validationResult.passed&&root.value("validationPassed").toBool(false);}
    m_workflowMode=WorkflowMode::Idle;m_workflowPlanIndex=-1;m_pendingRequestId=0;m_captureProcessing=false;

    // V27.6.53 migration convenience: restoring a completed V49/V50/V51 Session
    // that already contains a solved + Hold-out PASS result promotes that result
    // to the active calibration automatically, but keeps runtime application OFF.
    if(m_haveResult&&m_result.qualityPassed&&m_validationPassed){
        if(m_calibrationId.isEmpty())
            m_calibrationId=QStringLiteral("HTCAL51-RESTORE-%1").arg(QDateTime::currentDateTimeUtc().toString("yyyyMMdd-hhmmsszzz"));
        if(m_sync){
            HardwareTemporalCorrectionSettings staged=correctionSettings(false);QString stageError;
            if(!m_sync->setHardwareTemporalCorrection(staged,&stageError))
                emit logMessage(tr("[硬件同步标定] 已恢复验证通过Session，但活动τ暂存失败：%1").arg(stageError));
        }
        QString activeError;
        if(saveJson(activePath(),false,&activeError))
            emit logMessage(tr("[硬件同步标定] 已恢复的验证PASS Session已自动保存为活动标定（运行时OFF）：%1").arg(QDir::toNativeSeparators(activePath())));
        else
            emit logMessage(tr("[硬件同步标定] 恢复Session后自动保存活动标定失败：%1").arg(activeError));
        emit runtimeCorrectionChanged(false,m_result.delaySec*1e6);
        emit activeCalibrationChanged();
    }

    refreshReferenceUi();refreshPathTable();refreshCaptureTable();refreshResult();updateWorkflowButtons();
    if(error)error->clear();return true;
}

bool HardwareSyncCalibrationWidget::recoverLegacyV48Session(const QString& /*directory*/,QString* error)
{
    // V48 sidecars have no dynamic-sphere fitting contract and contain only
    // previously fitted centers. Never treat them as two-stage observations.
    // Existing disk RAW archives are preserved for independent offline replay.
    if(error)*error=tr("旧V48 Session包含全点RANSAC球心，不能直接恢复为两阶段动态标定。"
        "不会删除旧RAW；请新建Session，或先离线统一重分析全部旧RAW与Hold-out。 ");
    return false;
}

void HardwareSyncCalibrationWidget::restoreCaptureSession()
{
    if(m_workflowMode!=WorkflowMode::Idle||m_captureProcessing){m_status->setText(tr("采集/后台处理运行中，不能恢复Session"));return;}
    if(hasAcceptedPlans(ScanRole::Calibration)||hasAcceptedPlans(ScanRole::Validation)||!m_captureSessionId.isEmpty()){
        const auto answer=QMessageBox::question(this,tr("切换恢复Session"),
            tr("当前界面已经有Session/采集进度。恢复另一个Session会替换当前界面状态，但不会删除任何E盘RAW/JSON文件。确定继续吗？"));
        if(answer!=QMessageBox::Yes)return;
    }
    QString base=m_captureRootDirectory?m_captureRootDirectory->text().trimmed():QStringLiteral("E:/Projects/CData");
    QDir b(base);if(b.dirName()!=QStringLiteral("HardwareTemporalCalibration"))base=b.filePath(QStringLiteral("HardwareTemporalCalibration"));
    const QString dir=QFileDialog::getExistingDirectory(this,tr("选择要恢复/继续的硬件时域标定Session目录"),base);
    if(dir.isEmpty())return;
    QString e;const QString manifest=QDir(dir).filePath(QStringLiteral("session_manifest_v49.json"));
    const bool ok=QFile::exists(manifest)?loadSessionCheckpoint(dir,&e):recoverLegacyV48Session(dir,&e);
    if(!ok){QMessageBox::warning(this,tr("恢复Session失败"),e);return;}
    m_status->setText(tr("Session已恢复：标定%1/%2条、验证%3/%4条。点击“开始/继续标定路径采集”或“开始/继续验证路径采集”会从首个未完成路径继续。")
        .arg(acceptedPlanCount(ScanRole::Calibration)).arg(planCount(ScanRole::Calibration)).arg(acceptedPlanCount(ScanRole::Validation)).arg(planCount(ScanRole::Validation)));
    emit logMessage(tr("[硬件同步标定][断点] %1").arg(m_status->text()));
}

void HardwareSyncCalibrationWidget::newCaptureSession()
{
    const QString previousSession=m_captureSessionId.isEmpty()?tr("无"):m_captureSessionId;
    if(m_workflowMode!=WorkflowMode::Idle||m_captureProcessing||(m_sync&&m_sync->isRunning())){
        const QString reason=tr("硬件采集/后台处理运行中，不能新建Session");
        m_status->setText(reason);
        emit logMessage(tr("[硬件同步标定][SESSION-RESET-REJECT][H%1] %2").arg(m_selectedTopologyId).arg(reason));
        return;
    }
    if(hasAcceptedPlans(ScanRole::Calibration)||hasAcceptedPlans(ScanRole::Validation)||
       !m_captureSessionId.isEmpty()||!m_calibrationObservations.empty()||!m_validationCaptures.empty()){
        const auto r=QMessageBox::question(this,tr("新建实验Session"),
            tr("旧Session=%1，标定结果%2条、验证结果%3条。新建本轮会清空界面中的旧路径和测量结果，但不会删除任何磁盘RAW、旧Session目录或历史标定JSON。请先确认不需继续旧Session。确定新建吗？")
              .arg(previousSession).arg(m_calibrationObservations.size()).arg(m_validationCaptures.size()));
        if(r!=QMessageBox::Yes){
            emit logMessage(tr("[硬件同步标定][SESSION-RESET-CANCEL][H%1] 用户保留旧Session=%2")
                .arg(m_selectedTopologyId).arg(previousSession));
            return;
        }
    }
    m_captureSessionId.clear();m_captureSessionDirectory.clear();m_calibrationObservations.clear();m_validationCaptures.clear();m_result={};m_validationResult={};m_haveResult=false;m_validationPassed=false;m_calibrationId.clear();m_nextObservationId=1;m_sessionLatchEdge=m_selectedTopologyId==1?104:0;
    // New experiment starts without executable Plan IDs. Keep only the user-
    // editable configuration template and the safely taught scan center.
    m_plannedScans.clear();m_nextPlanId=1;m_nextPairId=1;
    refreshPathTable();refreshCaptureTable();refreshResult();updateWorkflowButtons();
    m_status->setText(tr("已准备新Session；旧Session/RAW磁盘文件未删除。请先点击“根据配置生成路径”，再开始标定采集。"));
    if(m_pathGenerationFeedback)m_pathGenerationFeedback->setText(tr("Session已重置，旧RAW文件保留。可以重新点击“根据配置生成路径”。"));
    emit logMessage(tr("[硬件同步标定][SESSION-RESET-OK][H%1] 已清空旧会话进度及旧路径队列（旧Session=%2）；磁盘RAW/旧JSON未删除；配置模板与扫描中点保留。")
        .arg(m_selectedTopologyId).arg(previousSession));
}

void HardwareSyncCalibrationWidget::startCaptureProcessing(const PlannedScan& plan,const QString& scanMessage)
{
    if(!m_sync){m_status->setText(tr("SyncManager未绑定"));m_workflowMode=WorkflowMode::Idle;updateWorkflowButtons();return;}
    if(m_captureProcessing){m_status->setText(tr("上一条扫描仍在后台归档/分析，拒绝重入"));m_workflowMode=WorkflowMode::Idle;updateWorkflowButtons();return;}
    if(m_sync->isRunning()||m_sync->syncMode()!=SyncMode::Hardware){
        m_status->setText(tr("主程序尚未完成Hardware Stop/Drain"));m_workflowMode=WorkflowMode::Idle;updateWorkflowButtons();return;
    }

    const auto st=m_sync->getHardwareAlignStats();
    if(st.alignmentFault||!st.syncEstablished||st.totalProfiles<10||
       st.totalProfiles!=st.totalPoseSamples||st.alignedProfiles!=st.totalProfiles||
       st.pendingProfiles!=0||st.droppedProfiles!=0){
        auto& p=m_plannedScans[std::size_t(m_workflowPlanIndex)];
        const QString reason=tr("硬件完整性闭环失败 Pose=%1 Profile=%2 Aligned=%3 Pending=%4 Dropped=%5 Fault=%6")
            .arg(st.totalPoseSamples).arg(st.totalProfiles).arg(st.alignedProfiles)
            .arg(st.pendingProfiles).arg(st.droppedProfiles).arg(st.alignmentFault);
        m_sync->clearData(); // release stopped RAW buffers on failed closure
        handlePlanFailure(p,reason,true);return;
    }

    const int completedTopology=static_cast<int>(m_sync->lastCaptureTopology());
    const int completedEdge=m_sync->lastCaptureCmmR1Edge();
    if(completedTopology!=m_selectedTopologyId ||
       !mls::runtime::Table5D::validTopologyEdge(completedTopology,completedEdge) ||
       (m_sessionLatchEdge!=0 && completedEdge!=m_sessionLatchEdge)) {
        m_sync->clearData();
        auto& p=m_plannedScans[std::size_t(m_workflowPlanIndex)];
        handlePlanFailure(p,tr("RAW硬件锁存拓扑/边沿发生变化：预期H%1 edge=%2，实际H%3 edge=%4；该条禁止进入标定")
            .arg(m_selectedTopologyId).arg(m_sessionLatchEdge).arg(completedTopology).arg(completedEdge),false);
        return;
    }
    m_sessionLatchEdge=completedEdge;
    const SensorParams sp=m_sync->lastCaptureSensorParams();
    const int triggerFrequencyHz=m_sync->lastCaptureTriggerFrequencyHz();
    const double triggerPeriodMs=m_sync->lastCaptureTriggerPeriodMs();
    QVector<LaserProfile> profiles;
    QVector<SamplePoint> poses;
    QString takeError;
    if(!m_sync->takeHardwareRawCaptureData(profiles,poses,&takeError)){
        auto& p=m_plannedScans[std::size_t(m_workflowPlanIndex)];
        m_sync->clearData();
        handlePlanFailure(p,tr("RAW数据所有权转移失败：%1").arg(takeError),true);return;
    }

    // The large arrays have now left SyncManager by swap. Clear diagnostics and
    // any retained capacity there immediately; this worker is the sole owner.
    m_sync->clearData();

    if(profiles.size()<10||poses.size()<10||profiles.size()!=poses.size()){
        auto& p=m_plannedScans[std::size_t(m_workflowPlanIndex)];
        handlePlanFailure(p,tr("RAW Profile/REGIST位姿不足或数量不一致"),true);return;
    }

    QString sessionError;
    QString sessionDir;
    const bool archiveEnabled=m_archiveEachScan&&m_archiveEachScan->isChecked();
    if(archiveEnabled){
        sessionDir=ensureCaptureSessionDirectory(&sessionError);
        if(sessionDir.isEmpty()){
            auto& p=m_plannedScans[std::size_t(m_workflowPlanIndex)];
            handlePlanFailure(p,tr("归档目录失败：%1").arg(sessionError),false);return;
        }
    }

    const PlannedScan planCopy=plan;
    const QString base=captureArchiveBaseName(planCopy);
    const QString archivePath=archiveEnabled?QDir(sessionDir).filePath(base+QStringLiteral(".scan")):QString();
    const QString metadataPath=archiveEnabled?QDir(sessionDir).filePath(base+QStringLiteral(".json")):QString();
    const std::size_t analysisCap=std::size_t(m_maxAnalysisPoints?m_maxAnalysisPoints->value():250000);
    const double threshold=m_ransacThreshold->value();
    const std::size_t minInliers=std::size_t(m_minInliers->value());
    const double expectedRadius=m_sphereDiameter->value()/2.0;
    const bool referenceValid=m_referenceFitValid;
    const Vec3d referenceCenter=m_referenceFit.centerMachineMm;
    const double candidateDelay=m_result.delaySec;
    const int calibrationTopology=m_selectedTopologyId;
    const int calibrationLatchEdge=m_sessionLatchEdge;
    const bool candidateValid=m_haveResult&&m_result.qualityPassed;
    const quint64 observationId=m_nextObservationId++;

    m_captureProcessing=true;
    const quint64 processingGeneration=++m_captureProcessingGeneration;
    if(m_workflowPlanIndex>=0&&m_workflowPlanIndex<int(m_plannedScans.size())){
        auto& p=m_plannedScans[std::size_t(m_workflowPlanIndex)];
        p.status=archiveEnabled?tr("后台归档/分析中"):tr("后台分析中");
    }
    refreshPathTable();
    m_status->setText(tr("路径 #%1 Stop/Drain完成；已把 %2 个RAW Profile / %3个REGIST位姿从SyncManager转移到后台线程。下一条路径会等待归档、分析和内存释放。")
        .arg(planCopy.id).arg(profiles.size()).arg(poses.size()));
    emit logMessage(tr("[硬件同步标定][内存] #%1 RAW所有权已转移：Profile=%2 Pose=%3；采集中未生成百万级AlignedPoint，也未向GUI实时点云扇出。")
        .arg(planCopy.id).arg(profiles.size()).arg(poses.size()));
    updateWorkflowButtons();

    QPointer<HardwareSyncCalibrationWidget> self(this);
    (void)QtConcurrent::run([self,planCopy,scanMessage,st,sp,triggerFrequencyHz,triggerPeriodMs,
                             profiles=std::move(profiles),poses=std::move(poses),
                             archiveEnabled,archivePath,metadataPath,analysisCap,
                             threshold,minInliers,expectedRadius,referenceValid,referenceCenter,
                             candidateDelay,candidateValid,observationId,processingGeneration,
                             calibrationTopology,calibrationLatchEdge]() mutable {
        CaptureProcessResult out;
        out.validation=planCopy.role==ScanRole::Validation;
        out.archivePath=archivePath;
        out.metadataPath=metadataPath;
        out.totalProfiles=std::size_t(profiles.size());

        struct SampleRef { std::size_t profileIndex=0; qsizetype pointIndex=0; };

        try {
            // Persist the complete RAW capture first. CompactScanIO stores one
            // Frame5D + compact sensor X/Z points per profile, not expanded XYZ
            // AlignedPoint records. The temporary archive buffers die before the
            // RANSAC analysis begins.
            if(archiveEnabled){
                {
                    auto scanData=CompactScanIO::fromProfilesAndPoses(
                        profiles,poses,0u,std::uint32_t(std::max(1.0,std::round(1.0e6/static_cast<double>(std::max(1,triggerFrequencyHz))))));
                    if(!scanData.validationError.isEmpty()){
                        out.error=QObject::tr("RAW .scan构建失败：%1").arg(scanData.validationError);
                    }else{
                        CompactScanIO::WriteOptions wo;
                        wo.compression=CompressionType::QtZlib;
                        wo.compressionLevel=1;
                        wo.scanName=planCopy.name;
                        wo.blockName=QStringLiteral("HardwareTemporalCalibration");
                        QString saveError;
                        if(!CompactScanIO::save(archivePath,scanData,wo,&saveError))
                            out.error=QObject::tr("RAW .scan保存失败：%1").arg(saveError);
                    }
                }
            }

            if(out.error.isEmpty()){
                // Validate one-to-one raw pairing and count valid sensor points.
                std::size_t validPointCount=0;
                for(qsizetype i=0;i<profiles.size();++i){
                    if(poses[i].frame.ordinal!=std::uint64_t(i)){
                        out.error=QObject::tr("RAW内部ordinal不连续：i=%1 poseOrdinal=%2")
                            .arg(i).arg(qulonglong(poses[i].frame.ordinal));
                        break;
                    }
                    const auto& pr=profiles[i];
                    const qsizetype n=qMin(pr.x.size(),pr.z.size());
                    for(qsizetype k=0;k<n;++k)
                        if(SensorUtils::isValidPoint(pr.x[k],pr.z[k])) ++validPointCount;
                }
                out.totalValidPoints=validPointCount;
                if(out.error.isEmpty()&&validPointCount<minInliers){
                    out.error=QObject::tr("有效球面点过少：N=%1 < 最少内点%2")
                        .arg(qulonglong(validPointCount)).arg(qulonglong(minInliers));
                }

                if(out.error.isEmpty()){
                    // Deterministic uniform sampling by VALID-point ordinal. This
                    // bounds RANSAC memory even for 30 s / 200 Hz captures.
                    const auto targets=evenlySpacedIndices(validPointCount,analysisCap);
                    out.analysisPoints=targets.size();
                    std::vector<Vec3d> cloud;
                    std::vector<SampleRef> sampleRefs;
                    cloud.reserve(targets.size());
                    sampleRefs.reserve(targets.size());
                    std::size_t seenValid=0;
                    std::size_t targetPos=0;
                    for(qsizetype i=0;i<profiles.size()&&targetPos<targets.size();++i){
                        const auto& pr=profiles[i];
                        const qsizetype n=qMin(pr.x.size(),pr.z.size());
                        for(qsizetype k=0;k<n&&targetPos<targets.size();++k){
                            const double sx=pr.x[k], sz=pr.z[k];
                            if(!SensorUtils::isValidPoint(sx,sz))continue;
                            if(seenValid==targets[targetPos]){
                                const auto mapped=mls::runtime::mapProfilePoint(poses[i].frame,sx,sz);
                                cloud.push_back({mapped[0],mapped[1],mapped[2]});
                                sampleRefs.push_back({std::size_t(i),k});
                                ++targetPos;
                            }
                            ++seenValid;
                        }
                    }

                    SphereCloudRansacOptions opt;
                    opt.inlierThresholdMm=threshold;
                    opt.minInliers=std::min(minInliers,cloud.size());
                    opt.expectedRadiusMm=expectedRadius;
                    opt.maxRadiusDeviationMm=std::max(0.25,5.0*opt.inlierThresholdMm);
                    // H1/H3 BOTH use the same two-stage RAW profile gate.
                    // Keep the unfiltered RANSAC for comparison, never silently use
                    // it for a new temporal observation when the arc gate fails.
                    const auto legacyFit=SphereDistanceAngleStudy::fitMachineSphereRansac(cloud,opt);
                    SphereCloudFitResult fit;
                    std::vector<Vec3d> selectedCloud;
                    std::vector<SampleRef> selectedRefs;
                    const SphereTwoStageQuality quality{}; // diagnostic defaults: 120 deg, 10um, 80 points
                    out.arcTotalProfiles=std::size_t(profiles.size());
                    if(!legacyFit.success){
                        out.error=QObject::tr("动态球原始全点RANSAC失败：%1")
                            .arg(QString::fromStdString(legacyFit.message));
                    }else{
                        // Calculate the quality decision from ORIGINAL sensor X/Z,
                        // before any time correction or machine-frame point weighting.
                        std::vector<unsigned char> acceptedProfile(std::size_t(profiles.size()),0);
                        for(qsizetype i=0;i<profiles.size();++i){
                            const auto& pr=profiles[i];
                            double expectedX=0.0,expectedZ=0.0;
                            if(!mls::runtime::validTransform(poses[i].frame.sensorToMachine) ||
                               !projectSphereCenterToSensorArcXZ(poses[i].frame.sensorToMachine,
                                    legacyFit.centerMachineMm.x,legacyFit.centerMachineMm.y,
                                    legacyFit.centerMachineMm.z,expectedX,expectedZ)) continue;
                            std::vector<SphereExposurePoint> rawArc;
                            const qsizetype n=qMin(pr.x.size(),pr.z.size());
                            rawArc.reserve(std::size_t(n));
                            for(qsizetype k=0;k<n;++k){
                                if(SensorUtils::isValidPoint(pr.x[k],pr.z[k]))
                                    rawArc.push_back({pr.x[k],pr.z[k],int(k)});
                            }
                            const auto arc=SphereExposureSweep::fitArc(
                                rawArc,expectedX,expectedZ,expectedRadius);
                            if(qualifySphereArc(arc,quality,expectedRadius)){
                                acceptedProfile[std::size_t(i)]=1;
                                ++out.arcQualifiedProfiles;
                            }
                        }
                        selectedCloud.reserve(cloud.size());
                        selectedRefs.reserve(sampleRefs.size());
                        for(std::size_t idx=0;idx<sampleRefs.size();++idx){
                            if(acceptedProfile[sampleRefs[idx].profileIndex]){
                                selectedCloud.push_back(cloud[idx]);
                                selectedRefs.push_back(sampleRefs[idx]);
                            }
                        }
                        out.arcQualifiedPoints=selectedCloud.size();
                        if(out.arcQualifiedProfiles < quality.minimumAcceptedProfiles ||
                           selectedCloud.size() < minInliers){
                            out.error=QObject::tr("动态球单帧圆弧质量闸门未通过：%1/%2帧，%3分析点；"
                                 "不采用原始RANSAC回退求解τ。请检查RAW曝光/扫球路径/圆弧覆盖")
                                .arg(qulonglong(out.arcQualifiedProfiles))
                                .arg(qulonglong(out.arcTotalProfiles))
                                .arg(qulonglong(out.arcQualifiedPoints));
                        }else{
                            SphereCloudRansacOptions refined=opt;
                            refined.inlierThresholdMm=quality.refinedRansacThresholdMm;
                            refined.minInliers=std::min(minInliers,selectedCloud.size());
                            fit=SphereDistanceAngleStudy::fitMachineSphereRansac(selectedCloud,refined);
                            if(!fit.success)
                                out.error=QObject::tr("动态球两阶段RANSAC失败：%1")
                                    .arg(QString::fromStdString(fit.message));
                        }
                    }
                    const auto describeFit=[](const SphereCloudFitResult& f) {
                        if(!f.success) return QStringLiteral("FAIL");
                        return QObject::tr("D=%1mm, RMS=%2mm, inliers=%3/%4")
                            .arg(2*f.radiusMm,0,'f',6).arg(f.residualRmsMm,0,'f',6)
                            .arg(qulonglong(f.inlierPointCount)).arg(qulonglong(f.candidatePointCount));
                    };
                    out.sphereFitComparisonText=QObject::tr(
                        "H%1 动态/验证球：原始全点RANSAC %2；两阶段RANSAC %3；"
                        "合格圆弧%4/%5，候选分析点%6；"
                        "判据跨度≥120°/RMS≤0.010mm/≥80点，三维新阈值0.020mm")
                        .arg(calibrationTopology)
                        .arg(describeFit(legacyFit)).arg(describeFit(fit))
                        .arg(qulonglong(out.arcQualifiedProfiles)).arg(qulonglong(out.arcTotalProfiles))
                        .arg(qulonglong(out.arcQualifiedPoints));
                    if(!fit.success){
                        if(out.error.isEmpty()) out.error=QObject::tr("动态球两阶段拟合未通过");
                    }else{
                        std::vector<Vec3d> in;
                        std::vector<SampleRef> inlierRefs;
                        in.reserve(fit.inlierIndices.size());
                        inlierRefs.reserve(fit.inlierIndices.size());
                        for(const auto sampleIdx:fit.inlierIndices){
                            if(sampleIdx>=selectedCloud.size()||sampleIdx>=selectedRefs.size())continue;
                            in.push_back(selectedCloud[sampleIdx]);
                            inlierRefs.push_back(selectedRefs[sampleIdx]);
                        }
                        // V27.6.50: dynamic clouds can be geometrically deformed by residual
                        // kinematic/hand-eye errors. Do NOT force the physical nominal radius
                        // on every moving scan; refine a free effective sphere instead. The
                        // forward/reverse pair later checks that this effective deformation is
                        // common (small radius difference), while center difference carries tau.
                        const auto dynamicFit=HardwareTemporalCalibration::refineFreeSphere(
                            in,fit.centerMachineMm,fit.radiusMm,opt.inlierThresholdMm,25);
                        if(!dynamicFit.success){
                            out.error=QObject::tr("动态自由有效半径球拟合失败：%1").arg(QString::fromStdString(dynamicFit.message));
                        }else if(referenceValid&&distance3(dynamicFit.centerMachineMm,referenceCenter)>5.0){
                            out.error=QObject::tr("动态球心距离参考球心超过5mm，疑似扫到背景/球座");
                        }else{
                            std::vector<TemporalPoseSample> tp;
                            tp.reserve(std::size_t(poses.size()));
                            for(const auto& p:poses){
                                TemporalPoseSample q;
                                q.ordinal=p.frame.ordinal;
                                q.joints={p.x,p.y,p.z,p.c,p.b};
                                tp.push_back(q);
                            }
                            const auto motion=HardwareTemporalCalibration::estimateLinearMotionVector(
                                tp,1.0/static_cast<double>(std::max(1,triggerFrequencyHz)));
                            if(!motion.success){
                                out.error=QObject::tr("当前H%1硬件锁存（R0/R1）无法识别恒速空间直线：%2")
                                    .arg(calibrationTopology)
                                    .arg(QString::fromStdString(motion.message));
                            }else{
                                Vec3d expected{
                                    planCopy.unitDirection.x()*planCopy.requestedSpeedMmPerSec*planCopy.directionSign,
                                    planCopy.unitDirection.y()*planCopy.requestedSpeedMmPerSec*planCopy.directionSign,
                                    planCopy.unitDirection.z()*planCopy.requestedSpeedMmPerSec*planCopy.directionSign};
                                const double cs=dot3(expected,motion.velocityMmPerSec)/
                                    std::max(1e-12,norm3(expected)*norm3(motion.velocityMmPerSec));
                                if(cs<0.95){
                                    out.error=QObject::tr("实际REGIST速度方向与规划路径不一致 cos=%1").arg(cs,0,'f',4);
                                }else if(motion.speedMmPerSec<0.5*planCopy.requestedSpeedMmPerSec||
                                         motion.speedMmPerSec>1.5*planCopy.requestedSpeedMmPerSec){
                                    out.error=QObject::tr("实际速度%1 mm/s与规划%2 mm/s偏差过大")
                                        .arg(motion.speedMmPerSec,0,'f',4)
                                        .arg(planCopy.requestedSpeedMmPerSec,0,'f',4);
                                }else if(planCopy.role==ScanRole::Calibration){
                                    TemporalTrajectoryObservation o;
                                    o.id=observationId;o.planId=planCopy.id;o.pairId=planCopy.pairId;
                                    o.directionSign=planCopy.directionSign;
                                    o.velocityMmPerSec=motion.velocityMmPerSec;
                                    o.sphereCenterMachineMm=dynamicFit.centerMachineMm;
                                    o.sphereRmsMm=dynamicFit.residualRmsMm;o.fittedRadiusMm=dynamicFit.radiusMm;
                                    qint64 firstPc=0,lastPc=0;for(const auto& pr:profiles){if(pr.tPcUs>0){if(firstPc==0)firstPc=pr.tPcUs;lastPc=pr.tPcUs;}}
                                    o.captureMidPcUs=(firstPc>0&&lastPc>=firstPc)?firstPc+(lastPc-firstPc)/2:0;
                                    o.sphereInliers=fit.inlierPointCount;
                                    o.profileCount=std::size_t(st.totalProfiles);
                                    o.resolutionIndex=sp.resolutionIndex;o.exposureTime10Us=sp.exposureTime;
                                    o.idleTime10Us=sp.idleTime;o.measuringFieldIndex=sp.measuringFieldIndex;o.triggerFrequencyHz=triggerFrequencyHz;o.triggerPeriodMs=triggerPeriodMs;
                                    out.calibrationObservation=o;
                                    out.success=true;
                                }else{
                                    if(!candidateValid){
                                        out.error=QObject::tr("验证采集时没有有效候选τ");
                                    }else{
                                        mls::runtime::HardwareTemporalCorrectionConfig cfg;
                                        cfg.enabled=true;cfg.delaySec=candidateDelay;
                                        cfg.triggerPeriodSec=1.0/static_cast<double>(std::max(1,triggerFrequencyHz));
                                        std::vector<Vec3d> rawMatched;
                                        std::vector<Vec3d> corr;
                                        rawMatched.reserve(inlierRefs.size());
                                        corr.reserve(inlierRefs.size());
                                        for(std::size_t sampleNo=0;sampleNo<inlierRefs.size();++sampleNo){
                                            const auto& ref=inlierRefs[sampleNo];
                                            const std::size_t i=ref.profileIndex;
                                            // Probe ordinal 0 cannot be used as an adjacent 5ms sample.
                                            // +tau: [i,i+1], -tau: [i-1,i].
                                            const bool forward=candidateDelay>=0.0;
                                            if((forward && (i==0 || i+1>=std::size_t(poses.size()))) ||
                                               (!forward && i<2))continue;
                                            const std::size_t neighbor=forward?i+1:i-1;
                                            const auto c=mls::runtime::interpolateHardwareTemporalFrame(
                                                poses[qsizetype(i)].frame,poses[qsizetype(neighbor)].frame,cfg);
                                            if(!c.success)continue;
                                            auto frame=c.frame;
                                            std::string completeError;
                                            if(!mls::runtime::completeFrame(frame,frame.physicalParameters,completeError))continue;
                                            const auto& pr=profiles[qsizetype(i)];
                                            if(ref.pointIndex<0||ref.pointIndex>=pr.x.size()||ref.pointIndex>=pr.z.size())continue;
                                            const auto mapped=mls::runtime::mapProfilePoint(
                                                frame,pr.x[ref.pointIndex],pr.z[ref.pointIndex]);
                                            if(!std::isfinite(mapped[0])||!std::isfinite(mapped[1])||
                                               !std::isfinite(mapped[2]))continue;
                                            // EXACT same original (profile, pixel) after the
                                            // signed interpolation check, for both hypotheses.
                                            rawMatched.push_back(in[sampleNo]);
                                            corr.push_back({mapped[0],mapped[1],mapped[2]});
                                        }
                                        if(corr.size()<minInliers){
                                            out.error=QObject::tr("可离线时域校正的球面分析点不足，N=%1")
                                                .arg(qulonglong(corr.size()));
                                        }else{
                                            const auto rawMatchedFit=HardwareTemporalCalibration::refineFreeSphere(
                                                rawMatched,dynamicFit.centerMachineMm,dynamicFit.radiusMm,
                                                quality.refinedRansacThresholdMm,25);
                                            if(!rawMatchedFit.success){
                                                out.error=QObject::tr("Hold-out同点集 RAW 球拟合失败：%1")
                                                    .arg(QString::fromStdString(rawMatchedFit.message));
                                            }else{
                                                const auto correctedFit=HardwareTemporalCalibration::refineFreeSphere(
                                                    corr,rawMatchedFit.centerMachineMm,rawMatchedFit.radiusMm,
                                                    quality.refinedRansacThresholdMm,25);
                                                if(!correctedFit.success){
                                                    out.error=QObject::tr("Hold-out同点集校正球拟合失败：%1")
                                                        .arg(QString::fromStdString(correctedFit.message));
                                                }else{
                                                ValidationCapture vc;
                                                vc.observation.id=observationId;
                                                vc.observation.pairId=planCopy.pairId;
                                                vc.observation.directionSign=planCopy.directionSign;
                                                vc.observation.velocityMmPerSec=motion.velocityMmPerSec;
                                                vc.observation.rawSphereCenterMachineMm=rawMatchedFit.centerMachineMm;
                                                vc.observation.correctedSphereCenterMachineMm=correctedFit.centerMachineMm;
                                                vc.observation.rawSphereRmsMm=rawMatchedFit.residualRmsMm;
                                                vc.observation.correctedSphereRmsMm=correctedFit.residualRmsMm;
                                                qint64 firstPc=0,lastPc=0;for(const auto& pr:profiles){if(pr.tPcUs>0){if(firstPc==0)firstPc=pr.tPcUs;lastPc=pr.tPcUs;}}
                                                vc.observation.captureMidPcUs=(firstPc>0&&lastPc>=firstPc)?firstPc+(lastPc-firstPc)/2:0;
                                                vc.referenceRawRmsMm=radialRmsLocal(rawMatched,referenceCenter,opt.expectedRadiusMm);
                                                vc.referenceCorrectedRmsMm=radialRmsLocal(corr,referenceCenter,opt.expectedRadiusMm);
                                                vc.inliers=rawMatched.size();
                                                out.validationCapture=vc;
                                                out.success=true;
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }

            if(out.success&&archiveEnabled){
                QJsonObject meta;
                meta["softwareVersion"]=QStringLiteral("V32.16-H1-H3-SIGNED-TEMPORAL");
                meta["topology"]=calibrationTopology==3
                    ? QStringLiteral("scanCONTROL CMM → IN1/R1 REGIST(114/115) full-MPOS")
                    : QStringLiteral("ECI OP14 → IN0/R0 REGIST(104) full-MPOS");
                meta["role"]=planCopy.role==ScanRole::Validation?QStringLiteral("validation"):QStringLiteral("calibration");
                meta["planId"]=QString::number(qulonglong(planCopy.id));
                meta["pairId"]=QString::number(qulonglong(planCopy.pairId));
                meta["name"]=planCopy.name;
                meta["directionSign"]=planCopy.directionSign;
                meta["requestedSpeedMmPerSec"]=planCopy.requestedSpeedMmPerSec;
                meta["distanceMm"]=planCopy.distanceMm;
                meta["rawScanPath"]=archivePath;
                meta["rawProfileCount"]=QString::number(qulonglong(out.totalProfiles));
                meta["totalValidSensorPoints"]=QString::number(qulonglong(out.totalValidPoints));
                meta["analysisPoints"]=QString::number(qulonglong(out.analysisPoints));
                meta["profileCount"]=QString::number(qulonglong(st.totalProfiles));
                meta["poseCount"]=QString::number(qulonglong(st.totalPoseSamples));
                meta["resolutionIndex"]=int(sp.resolutionIndex);
                meta["exposureTime10Us"]=int(sp.exposureTime);
                meta["idleTime10Us"]=int(sp.idleTime);
                meta["triggerFrequencyHz"]=triggerFrequencyHz;meta["triggerPeriodMs"]=triggerPeriodMs;meta["measuringFieldIndex"]=sp.measuringFieldIndex;
                meta["topologyId"]=calibrationTopology;meta["latchEdge"]=calibrationLatchEdge;
                meta["cmmDivider"]=(calibrationTopology==3?1:0);meta["cmmSkewHalfUs"]=0;meta["cmmRatio"]=1;meta["dynamicCenterFitMode"]=QStringLiteral("arc-qualified-RANSAC+free-effective-radius");
                meta["sphereFitComparison"]=out.sphereFitComparisonText;
                meta["dynamicSphereFitContract"]=kDynamicSphereFitContract;
                meta["arcTotalProfiles"]=QString::number(qulonglong(out.arcTotalProfiles));
                meta["arcQualifiedProfiles"]=QString::number(qulonglong(out.arcQualifiedProfiles));
                meta["arcQualifiedAnalysisPoints"]=QString::number(qulonglong(out.arcQualifiedPoints));
                meta["holdoutIdenticalPointIndices"]=out.validation;
                if(out.validation){
                    const auto& o=out.validationCapture.observation;
                    meta["velocityMmPerSec"]=vecJson(o.velocityMmPerSec);
                    meta["rawCenter"]=vecJson(o.rawSphereCenterMachineMm);
                    meta["correctedCenter"]=vecJson(o.correctedSphereCenterMachineMm);
                    meta["rawRmsMm"]=o.rawSphereRmsMm;
                    meta["correctedRmsMm"]=o.correctedSphereRmsMm;
                    meta["candidateDelayUs"]=candidateDelay*1e6;
                    meta["captureMidPcUs"]=QString::number(qlonglong(o.captureMidPcUs));
                }else{
                    const auto& o=out.calibrationObservation;
                    meta["velocityMmPerSec"]=vecJson(o.velocityMmPerSec);
                    meta["sphereCenterMachineMm"]=vecJson(o.sphereCenterMachineMm);
                    meta["sphereRmsMm"]=o.sphereRmsMm;
                    meta["fittedRadiusMm"]=o.fittedRadiusMm;
                    meta["sphereInliers"]=QString::number(qulonglong(o.sphereInliers));
                    meta["captureMidPcUs"]=QString::number(qlonglong(o.captureMidPcUs));
                }
                QString metaError;
                if(!saveJsonObjectAtomic(metadataPath,meta,&metaError)){
                    out.success=false;
                    out.error=QObject::tr("RAW已保存，但sidecar JSON保存失败：%1").arg(metaError);
                }
            }
        } catch(const std::bad_alloc&) {
            out.success=false;
            out.error=QObject::tr("后台处理内存分配失败；本条RAW已与GUI隔离。建议降低“最大分析点数”，并检查E盘可用空间。");
        } catch(const std::exception& e) {
            out.success=false;
            out.error=QObject::tr("后台处理异常：%1").arg(QString::fromUtf8(e.what()));
        }

        if(self){
            QMetaObject::invokeMethod(self.data(),[self,planId=planCopy.id,processingGeneration,out=std::move(out),scanMessage]() mutable {
                if(self)self->finishCaptureProcessing(planId,processingGeneration,std::move(out),scanMessage);
            },Qt::QueuedConnection);
        }
        // profiles/poses and all archive/RANSAC temporaries are destroyed here.
        // Only compact statistics survive in HardwareSyncCalibrationWidget.
    });
}

void HardwareSyncCalibrationWidget::finishCaptureProcessing(quint64 planId,quint64 processingGeneration,
                                                             CaptureProcessResult result,const QString& scanMessage)
{
    if(processingGeneration!=m_captureProcessingGeneration){
        m_captureProcessing=false;
        emit logMessage(tr("[硬件同步标定][内存] 后台任务 #%1 已结束，但流程已被操作者取消；完整点云现已释放。").arg(planId));
        updateWorkflowButtons();
        return;
    }

    m_captureProcessing=false;
    auto it=std::find_if(m_plannedScans.begin(),m_plannedScans.end(),
                         [planId](const PlannedScan& p){return p.id==planId;});
    if(it==m_plannedScans.end()){
        m_workflowMode=WorkflowMode::Idle;
        m_status->setText(tr("后台处理完成但找不到对应规划路径 #%1").arg(planId));
        updateWorkflowButtons();
        return;
    }

    auto& p=*it;
    if(!result.sphereFitComparisonText.isEmpty())
        emit logMessage(tr("[硬件同步标定][两阶段球拟合] #%1 %2")
            .arg(planId).arg(result.sphereFitComparisonText));
    if(!result.success){
        if(!result.archivePath.isEmpty())p.archivePath=result.archivePath;
        if(!result.metadataPath.isEmpty())p.metadataPath=result.metadataPath;
        handlePlanFailure(p,tr("后台归档/分析失败：%1").arg(result.error),true);
        return;
    }

    if(result.validation)m_validationCaptures.push_back(result.validationCapture);
    else m_calibrationObservations.push_back(result.calibrationObservation);

    p.completed=true;
    p.accepted=true;
    p.consecutiveFailures=0;
    p.lastError.clear();
    p.archivePath=result.archivePath;
    p.metadataPath=result.metadataPath;
    p.status=result.archivePath.isEmpty()?tr("完成/已分析"):tr("完成/已归档/分析");
    refreshPathTable();refreshCaptureTable();

    emit logMessage(tr("[硬件同步标定] #%1采入成功：%2；RAW有效点=%3，后台分析=%4；%5")
        .arg(planId).arg(scanMessage)
        .arg(qulonglong(result.totalValidPoints))
        .arg(qulonglong(result.analysisPoints))
        .arg(result.archivePath.isEmpty()?tr("未启用RAW归档"):tr("RAW=%1").arg(QDir::toNativeSeparators(result.archivePath))));
    emit logMessage(tr("[硬件同步标定][内存] #%1 后台处理结束；该条RAW Profile、REGIST位姿及临时归档/RANSAC数组已释放，内存中仅保留轻量球心/速度/统计结果。").arg(planId));
    QString checkpointError;
    if(!saveSessionCheckpoint(&checkpointError)&&!checkpointError.isEmpty())
        emit logMessage(tr("[硬件同步标定][断点] #%1成功后检查点保存失败：%2").arg(planId).arg(checkpointError));
    else
        emit logMessage(tr("[硬件同步标定][断点] #%1已提交到Session；当前标定完成%2/%3，验证完成%4/%5。")
            .arg(planId).arg(acceptedPlanCount(ScanRole::Calibration)).arg(planCount(ScanRole::Calibration))
            .arg(acceptedPlanCount(ScanRole::Validation)).arg(planCount(ScanRole::Validation)));

    requestNextWorkflowScan();
}

double HardwareSyncCalibrationWidget::radialRms(const std::vector<Vec3d>& points,const Vec3d& c,double r){if(points.empty())return std::numeric_limits<double>::infinity();double s=0;std::size_t n=0;for(const auto&p:points){const double e=distance3(p,c)-r;if(std::isfinite(e)){s+=e*e;++n;}}return n?std::sqrt(s/double(n)):std::numeric_limits<double>::infinity();}

TemporalTrajectoryCriteria HardwareSyncCalibrationWidget::trajectoryCriteriaFromUi() const{TemporalTrajectoryCriteria c;c.minPairs=std::size_t(m_minPairs->value());c.minDistinctSpeedLevels=std::size_t(m_minSpeedLevels->value());c.minRSquared=m_minR2->value();c.maxPairResidualMm=m_maxPairResidual->value();c.maxPairRadiusDifferenceMm=m_maxPairRadiusDiff?m_maxPairRadiusDiff->value():0.050;c.maxDelayStdSec=m_maxStdUs->value()*1e-6;c.maxDelayFractionOfPeriod=0.95;c.allowSignedDelay=(m_selectedTopologyId==3);return c;}
TemporalValidationCriteria HardwareSyncCalibrationWidget::validationCriteriaFromUi() const{TemporalValidationCriteria c;c.minPairs=std::size_t(m_minValidationPairs->value());c.minImprovementFraction=m_minValidationImprovement->value()/100.0;c.maxResidualDelaySec=m_maxValidationResidualUs->value()*1e-6;return c;}

void HardwareSyncCalibrationWidget::solveCalibration(){if(!allPlansComplete(ScanRole::Calibration)){m_status->setText(tr("所有启用的标定路径必须完成且采入通过后才能求解τ。"));return;}m_result=HardwareTemporalCalibration::solveTrajectoryPairs(m_calibrationObservations);m_haveResult=m_result.success;if(m_result.success)HardwareTemporalCalibration::evaluateTrajectoryQuality(m_result,trajectoryCriteriaFromUi());m_validationPassed=false;m_validationCaptures.clear();m_validationResult={};m_calibrationId=m_result.success?QStringLiteral("HTCAL-H%1-%2").arg(m_selectedTopologyId)
    .arg(QDateTime::currentDateTimeUtc().toString("yyyyMMdd-hhmmsszzz")):QString();refreshResult();m_status->setText(m_result.success?tr("求解完成：%1；请先检查下方各速度τ均值/标准差，再执行独立验证路径。 ").arg(QString::fromStdString(m_result.qualityMessage)):tr("求解失败：%1").arg(QString::fromStdString(m_result.message)));emit logMessage(tr("[硬件同步标定] %1").arg(m_status->text()));if(m_result.success&&m_speedSummaryTable){for(int r=0;r<m_speedSummaryTable->rowCount();++r){emit logMessage(tr("[硬件同步标定][速度重复性] 速度档=%1 mm/s, Pair=%2, 实际速度均值=%3 mm/s, τ均值=%4 µs, τ标准差=%5 µs, 范围=%6").arg(m_speedSummaryTable->item(r,0)?m_speedSummaryTable->item(r,0)->text():QString()).arg(m_speedSummaryTable->item(r,1)?m_speedSummaryTable->item(r,1)->text():QString()).arg(m_speedSummaryTable->item(r,2)?m_speedSummaryTable->item(r,2)->text():QString()).arg(m_speedSummaryTable->item(r,3)?m_speedSummaryTable->item(r,3)->text():QString()).arg(m_speedSummaryTable->item(r,4)?m_speedSummaryTable->item(r,4)->text():QString()).arg(m_speedSummaryTable->item(r,5)?m_speedSummaryTable->item(r,5)->text():QString()));}}QString checkpointError;if(!saveSessionCheckpoint(&checkpointError)&&!checkpointError.isEmpty())emit logMessage(tr("[硬件同步标定][断点] 求解后检查点保存失败：%1").arg(checkpointError));}

void HardwareSyncCalibrationWidget::evaluateValidation()
{
    if(!allPlansComplete(ScanRole::Validation)){
        m_status->setText(tr("所有启用的Hold-out验证路径必须完成且采入通过后才能评估验证。"));
        return;
    }
    std::vector<TemporalValidationObservation> v;
    v.reserve(m_validationCaptures.size());
    for(const auto& c : m_validationCaptures) v.push_back(c.observation);
    m_validationResult=HardwareTemporalCalibration::evaluateValidationPairs(v,validationCriteriaFromUi());
    m_validationPassed=m_validationResult.success&&m_validationResult.passed;

    // V27.6.53: a solved + independently validated candidate automatically becomes
    // the active calibration, but remains runtime-OFF by default.  This separates
    // "what tau is loaded" from "whether q(t+tau) is actually applied".
    if(m_validationPassed && m_haveResult && m_result.qualityPassed){
        QString stageError;
        if(m_sync){
            HardwareTemporalCorrectionSettings staged=correctionSettings(false);
            if(!m_sync->setHardwareTemporalCorrection(staged,&stageError)){
                emit logMessage(tr("[硬件同步标定] 验证已PASS，但活动τ暂存到SyncManager失败：%1").arg(stageError));
            }
        }
        QString activeError;
        if(!saveJson(activePath(),false,&activeError)){
            emit logMessage(tr("[硬件同步标定] 验证已PASS，但自动保存活动时域标定失败：%1").arg(activeError));
        }else{
            emit logMessage(tr("[硬件同步标定] 验证PASS结果已自动保存为活动标定（默认运行时OFF）：%1").arg(QDir::toNativeSeparators(activePath())));
        }
        emit runtimeCorrectionChanged(false,m_result.delaySec*1e6);
        emit activeCalibrationChanged();
    }

    refreshResult();
    m_status->setText(tr("Hold-out验证：%1").arg(QString::fromStdString(m_validationResult.message)));
    emit logMessage(tr("[硬件同步标定] %1").arg(m_status->text()));
    QString checkpointError;
    if(!saveSessionCheckpoint(&checkpointError)&&!checkpointError.isEmpty())
        emit logMessage(tr("[硬件同步标定][断点] 验证后检查点保存失败：%1").arg(checkpointError));
}

bool HardwareSyncCalibrationWidget::applyResult(bool persist,QString* error)
{
    if(!m_sync){if(error)*error=tr("未绑定SyncManager");return false;}
    if(!m_haveResult||!m_result.qualityPassed){if(error)*error=tr("候选τ未通过求解质量门限");return false;}
    if(!m_validationPassed){if(error)*error=tr("Hold-out验证未通过，禁止启用补偿");return false;}
    HardwareTemporalCorrectionSettings s=correctionSettings(true);
    QString e;
    if(!m_sync->setHardwareTemporalCorrection(s,&e)){if(error)*error=e;return false;}
    if(persist&&!saveJson(activePath(),true,&e)){
        // Keep the calibration loaded, but fail safe to runtime OFF.
        s.enabled=false;
        QString ignored;
        m_sync->setHardwareTemporalCorrection(s,&ignored);
        if(error)*error=tr("运行时应用成功但保存失败，已回退为OFF且保留活动τ：%1").arg(e);
        emit runtimeCorrectionChanged(false,m_result.delaySec*1e6);
        emit activeCalibrationChanged();
        return false;
    }
    emit runtimeCorrectionChanged(true,m_result.delaySec*1e6);
    emit activeCalibrationChanged();
    refreshResult();
    return true;
}

bool HardwareSyncCalibrationWidget::setRuntimeCorrectionEnabled(bool enabled, QString* error)
{
    if(enabled) return applyResult(true,error);
    if(!m_sync){if(error)*error=tr("未绑定SyncManager");return false;}
    if(!m_haveResult||!m_result.qualityPassed||!m_validationPassed){
        if(error)*error=tr("当前没有已求解且Hold-out验证通过的活动硬件时域标定");
        return false;
    }
    HardwareTemporalCorrectionSettings s=correctionSettings(false);
    QString e;
    if(!m_sync->setHardwareTemporalCorrection(s,&e)){if(error)*error=e;return false;}
    if(!saveJson(activePath(),false,&e)){if(error)*error=e;return false;}
    emit runtimeCorrectionChanged(false,m_result.delaySec*1e6);
    emit activeCalibrationChanged();
    refreshResult();
    if(error)error->clear();
    return true;
}

void HardwareSyncCalibrationWidget::applyCalibration()
{
    QString e;
    if(!setRuntimeCorrectionEnabled(true,&e)){m_status->setText(e);return;}
    m_status->setText(tr("验证通过并已启用时域补偿：τ=%1 µs；正式采集继续检查sensor/频率配置指纹。 ").arg(m_result.delaySec*1e6,0,'f',2));
    emit logMessage(tr("[硬件同步标定] %1").arg(m_status->text()));
}

void HardwareSyncCalibrationWidget::disableCorrection()
{
    QString e;
    if(!setRuntimeCorrectionEnabled(false,&e)){m_status->setText(e);return;}
    m_status->setText(tr("已关闭运行时补偿，活动H%1 τ仍保留；采集将使用原始R%2硬件锁存位姿。")
        .arg(m_selectedTopologyId).arg(m_selectedTopologyId==1?0:1));
    emit logMessage(tr("[硬件同步标定] %1").arg(m_status->text()));
}

void HardwareSyncCalibrationWidget::clearCalibrationData(){
    if(m_workflowMode!=WorkflowMode::Idle||m_captureProcessing){m_status->setText(tr("采集/后台处理运行中，不能清空数据"));return;}
    if(hasAcceptedPlans(ScanRole::Calibration)||hasAcceptedPlans(ScanRole::Validation)||!m_captureSessionId.isEmpty()){
        const auto r=QMessageBox::question(this,tr("清空当前Session进度"),tr("这会清空当前界面的标定/验证进度并开始新的Session；E盘已经归档的旧RAW和旧Session目录不会删除。确定继续吗？"));
        if(r!=QMessageBox::Yes)return;
    }
    m_captureSessionId.clear();m_captureSessionDirectory.clear();m_calibrationObservations.clear();m_validationCaptures.clear();m_result={};m_validationResult={};m_haveResult=false;m_validationPassed=false;m_calibrationId.clear();m_nextObservationId=1;m_sessionLatchEdge=m_selectedTopologyId==1?104:0;
    for(auto&p:m_plannedScans){p.completed=false;p.accepted=false;p.status=tr("待执行");p.archivePath.clear();p.metadataPath.clear();p.attemptCount=0;p.failureCount=0;p.consecutiveFailures=0;p.lastError.clear();p.restoredFromDisk=false;}
    refreshPathTable();refreshCaptureTable();refreshResult();m_status->setText(tr("当前Session进度已清空；参考球和路径配置保留。旧RAW磁盘文件未删除。"));updateWorkflowButtons();
}


void HardwareSyncCalibrationWidget::refreshPathTable()
{
    if(!m_pathTable)return;
    m_pathTable->setRowCount(int(m_plannedScans.size()));
    for(int r=0;r<int(m_plannedScans.size());++r){
        const auto&p=m_plannedScans[std::size_t(r)];
        QString vals[kPathCols]={QString::number(r+1),roleText(p.role==ScanRole::Validation),QString::number(qulonglong(p.pairId)),p.name,p.directionSign>0?"+":"-",
            QStringLiteral("(%1,%2,%3)").arg(p.start.x,0,'f',3).arg(p.start.y,0,'f',3).arg(p.start.z,0,'f',3),
            QStringLiteral("(%1,%2,%3)").arg(p.end.x,0,'f',3).arg(p.end.y,0,'f',3).arg(p.end.z,0,'f',3),
            QString::number(p.requestedSpeedMmPerSec,'f',3),QString::number(p.distanceMm,'f',3),p.status,
            QString::number(qulonglong(p.id)),p.archivePath.isEmpty()?QString():QDir::toNativeSeparators(p.archivePath)};
        for(int c=0;c<kPathCols;++c)m_pathTable->setItem(r,c,new QTableWidgetItem(vals[c]));
    }
}

void HardwareSyncCalibrationWidget::refreshCaptureTable()
{
    if(!m_captureTable)return;
    const int rows=int(m_calibrationObservations.size()+m_validationCaptures.size());
    m_captureTable->setRowCount(rows);int r=0;
    for(const auto&o:m_calibrationObservations){
        QString vals[kCaptureCols]={tr("标定"),QString::number(qulonglong(o.pairId)),o.directionSign>0?"+":"-",
            QString::number(o.velocityMmPerSec.x,'f',4),QString::number(o.velocityMmPerSec.y,'f',4),QString::number(o.velocityMmPerSec.z,'f',4),
            QString::number(o.sphereCenterMachineMm.x,'f',5),QString::number(o.sphereCenterMachineMm.y,'f',5),QString::number(o.sphereCenterMachineMm.z,'f',5),
            QString::number(o.fittedRadiusMm,'f',6),QString::number(o.sphereRmsMm,'f',5),"-",
            QString::number(qulonglong(o.sphereInliers)),QString::number(qulonglong(o.profileCount))};
        for(int c=0;c<kCaptureCols;++c)m_captureTable->setItem(r,c,new QTableWidgetItem(vals[c]));++r;
    }
    for(const auto&v:m_validationCaptures){
        const auto&o=v.observation;
        QString vals[kCaptureCols]={tr("验证"),QString::number(qulonglong(o.pairId)),o.directionSign>0?"+":"-",
            QString::number(o.velocityMmPerSec.x,'f',4),QString::number(o.velocityMmPerSec.y,'f',4),QString::number(o.velocityMmPerSec.z,'f',4),
            QString::number(o.rawSphereCenterMachineMm.x,'f',5),QString::number(o.rawSphereCenterMachineMm.y,'f',5),QString::number(o.rawSphereCenterMachineMm.z,'f',5),"-",
            QString::number(o.rawSphereRmsMm,'f',5),QString::number(o.correctedSphereRmsMm,'f',5),QString::number(qulonglong(v.inliers)),"-"};
        for(int c=0;c<kCaptureCols;++c)m_captureTable->setItem(r,c,new QTableWidgetItem(vals[c]));++r;
    }
}

void HardwareSyncCalibrationWidget::refreshSpeedSummaryTable()
{
    if(!m_speedSummaryTable)return;
    m_speedSummaryTable->setRowCount(0);
    if(!m_haveResult||m_result.pairs.empty())return;
    const double step=(m_templateSpeedStep&&m_templateSpeedStep->value()>1e-9)?m_templateSpeedStep->value():0.5;
    struct G{double nominal=0.0;std::vector<double> speeds;std::vector<double> delaysUs;};
    std::map<long long,G> groups;
    for(const auto& p:m_result.pairs){
        if(!p.valid||!std::isfinite(p.meanSpeedMmPerSec)||!std::isfinite(p.pairDelaySec))continue;
        const long long key=std::llround(p.meanSpeedMmPerSec/step);
        auto& g=groups[key];g.nominal=double(key)*step;g.speeds.push_back(p.meanSpeedMmPerSec);g.delaysUs.push_back(p.pairDelaySec*1e6);
    }
    m_speedSummaryTable->setRowCount(int(groups.size()));int row=0;
    for(const auto& kv:groups){
        const auto& g=kv.second;if(g.delaysUs.empty())continue;
        double meanV=0.0,meanT=0.0,minT=std::numeric_limits<double>::infinity(),maxT=-std::numeric_limits<double>::infinity();
        for(double v:g.speeds)meanV+=v;
        for(double t:g.delaysUs){meanT+=t;minT=std::min(minT,t);maxT=std::max(maxT,t);}
        meanV/=double(g.speeds.size());meanT/=double(g.delaysUs.size());
        double s2=0.0;for(double t:g.delaysUs){const double d=t-meanT;s2+=d*d;}
        const double sd=g.delaysUs.size()>1?std::sqrt(s2/double(g.delaysUs.size()-1)):0.0;
        const QString vals[kSpeedSummaryCols]={QString::number(g.nominal,'f',2),QString::number(qulonglong(g.delaysUs.size())),
            QString::number(meanV,'f',4),QString::number(meanT,'f',2),QString::number(sd,'f',2),
            QStringLiteral("%1 ~ %2").arg(minT,0,'f',2).arg(maxT,0,'f',2)};
        for(int c=0;c<kSpeedSummaryCols;++c)m_speedSummaryTable->setItem(row,c,new QTableWidgetItem(vals[c]));++row;
    }
}

void HardwareSyncCalibrationWidget::refreshResult()
{
    refreshSpeedSummaryTable();
    if(!m_haveResult){
        m_resultLabel->setText(tr("尚未求解。V27.6.53默认0.5~5.0 mm/s。动态扫描不强迫拟合半径等于标准球物理半径，而使用自由“有效半径”；正反Pair检查有效半径一致性。主模型：c(t)=c0+g·t−τ·v，同时估计慢漂移g和硬件时延τ。"));
    }else{
        const QString drift=m_result.driftModelUsed
            ? tr("漂移g=(%1,%2,%3) µm/min，时间跨度=%4 s").arg(m_result.centerDriftMmPerSec.x*60000.0,0,'f',3).arg(m_result.centerDriftMmPerSec.y*60000.0,0,'f',3).arg(m_result.centerDriftMmPerSec.z*60000.0,0,'f',3).arg(m_result.timeSpanSec,0,'f',1)
            : tr("未获得完整扫描中点时间戳，回退到正反Pair差分模型");
        m_resultLabel->setText(tr("候选τ=%1 µs（未校正漂移的Pair估计=%2 µs），std=%3 µs，Pair=%4，速度等级=%5，R²=%6，模型RMS=%7 mm，质量=%8\n%9；动态有效R均值=%10 mm，最大正反ΔR=%11 mm。定义：τ=t_sensor,effective−t_ECI,latch；H1要求0≤τ<T；H3允许−T<τ<T，正负分别使用下一/上一帧。")
            .arg(m_result.delaySec*1e6,0,'f',2).arg(m_result.naivePairDelaySec*1e6,0,'f',2).arg(m_result.delayStdSec*1e6,0,'f',2)
            .arg(qulonglong(m_result.pairCount)).arg(qulonglong(m_result.distinctSpeedLevels)).arg(m_result.rSquared,0,'f',4).arg(m_result.residualRmsMm,0,'f',6)
            .arg(m_result.qualityPassed?tr("PASS"):tr("FAIL")).arg(drift).arg(m_result.meanEffectiveRadiusMm,0,'f',6).arg(m_result.maxPairRadiusDifferenceMm,0,'f',6));
        if(m_referenceFitValid){m_resultLabel->setText(m_resultLabel->text()+tr("；有效R相对物理标称R偏差=%1 µm（仅诊断，不作为时延失败条件）。").arg((m_result.meanEffectiveRadiusMm-m_referenceFit.knownRadiusMm)*1000.0,0,'f',2));}
    }
    if(!m_validationResult.success)m_validationLabel->setText(tr("尚未验证。求解PASS后，可修改/生成新的XY、XZ、XYZ斜线等Hold-out路径；验证采集保持运行时补偿禁用，软件离线同时计算raw与候选τ校正结果。"));else m_validationLabel->setText(tr("验证%1：Pair=%2；raw等效τ=%3 µs；校正后残余τ=%4 µs；改善=%5%；Pair残差RMS raw/corr=%6/%7 mm。")
        .arg(m_validationPassed?tr("PASS"):tr("FAIL")).arg(qulonglong(m_validationResult.pairCount)).arg(m_validationResult.rawEquivalentDelaySec*1e6,0,'f',2).arg(m_validationResult.correctedResidualDelaySec*1e6,0,'f',2).arg(m_validationResult.improvementFraction*100,0,'f',1).arg(m_validationResult.rawPairResidualRmsMm,0,'f',5).arg(m_validationResult.correctedPairResidualRmsMm,0,'f',5));
    if(m_sync){
        const auto c=m_sync->hardwareTemporalCorrection();
        if(!c.calibrationId.isEmpty() && std::isfinite(c.delaySec) &&
            static_cast<int>(c.topology)==m_selectedTopologyId){
            m_runtimeLabel->setText(c.enabled
                ? tr("运行时补偿：ENABLED，τ=%1 µs，σ=%2 µs，ID=%3")
                    .arg(c.delaySec*1e6,0,'f',2).arg(c.delayStdSec*1e6,0,'f',2).arg(c.calibrationId)
                : tr("运行时补偿：DISABLED / RAW；活动τ仍已加载=%1 µs，σ=%2 µs，ID=%3")
                    .arg(c.delaySec*1e6,0,'f',2).arg(c.delayStdSec*1e6,0,'f',2).arg(c.calibrationId));
        }else{
            m_runtimeLabel->setText(tr("运行时补偿：DISABLED；当前未加载已验证的活动时域标定"));
        }
    }else m_runtimeLabel->setText(tr("运行时补偿：SyncManager未绑定"));
    updateWorkflowButtons();
}

void HardwareSyncCalibrationWidget::updateWorkflowButtons()
{
    const bool idle=m_workflowMode==WorkflowMode::Idle&&!m_captureProcessing&&!(m_sync&&m_sync->isRunning());
    const bool ready=m_referenceFitValid&&m_planCenterValid&&!m_plannedScans.empty()&&m_sync&&m_controller;
    const bool rawRuntime=!m_sync||!m_sync->hardwareTemporalCorrection().enabled;
    // Let operators click to see/log missing prerequisites when the hardware
    // is idle. During acquisition disable modal dialogs that could block the
    // GUI; generatePaths() independently rechecks this guard before writing.
    m_generateButton->setEnabled(idle);
    if(m_pathGenerationFeedback && m_plannedScans.empty()){
        if(!idle)m_pathGenerationFeedback->setText(tr("暂不能生成路径：采集或后台处理尚未停止；停止后可点击生成并记录原因。"));
        else if(!m_referenceFitValid)m_pathGenerationFeedback->setText(tr("路径未生成：先导入点云并拟合标准球（可点按钮查看错误）。"));
        else if(!m_planCenterValid)m_pathGenerationFeedback->setText(tr("路径未生成：先示教当前XYZCB扫描中点（可点按钮查看错误）。"));
        else if(!m_captureSessionId.isEmpty()||!m_calibrationObservations.empty()||!m_validationCaptures.empty())
            m_pathGenerationFeedback->setText(tr("路径未生成：检测到历史Session/采集结果，必须先明确重置本轮Session。"));
    }
    m_startCalibrationButton->setEnabled(idle&&ready&&planCount(ScanRole::Calibration)>0&&rawRuntime);
    m_solveButton->setEnabled(idle&&allPlansComplete(ScanRole::Calibration)&&m_calibrationObservations.size()>=2);
    m_startValidationButton->setEnabled(idle&&ready&&planCount(ScanRole::Validation)>0&&m_haveResult&&m_result.qualityPassed&&rawRuntime);
    m_validateButton->setEnabled(idle&&allPlansComplete(ScanRole::Validation)&&!m_validationCaptures.empty());
    m_applyButton->setEnabled(idle&&m_haveResult&&m_result.qualityPassed&&m_validationPassed&&m_sync&&!m_sync->isRunning());
    m_abortButton->setEnabled(m_workflowMode!=WorkflowMode::Idle||m_captureProcessing);
    if(m_resumeSessionButton)m_resumeSessionButton->setEnabled(idle);
    if(m_newSessionButton)m_newSessionButton->setEnabled(idle);
}

HardwareTemporalCorrectionSettings HardwareSyncCalibrationWidget::correctionSettings(bool enabled) const
{
    HardwareTemporalCorrectionSettings s;
    s.enabled=enabled;
    s.delaySec=m_result.delaySec;
    s.delayStdSec=m_result.delayStdSec;
    s.calibrationId=m_calibrationId;
    s.sensorParams.resolutionIndex=m_result.resolutionIndex;
    s.sensorParams.exposureTime=m_result.exposureTime10Us;
    s.sensorParams.idleTime=m_result.idleTime10Us;
    s.sensorParams.measuringFieldIndex=m_result.measuringFieldIndex;
    s.triggerFrequencyHz=m_result.triggerFrequencyHz;
    s.triggerPeriodMs=m_result.triggerPeriodMs;
    s.topology=static_cast<mls::runtime::HardwareSyncTopology>(m_selectedTopologyId);
    s.latchEdge=m_selectedTopologyId==1 ? 104 : m_sessionLatchEdge;
    s.cmmDivider=m_selectedTopologyId==3 ? 1 : 0;
    s.cmmSkewHalfUs=0;
    s.cmmRatio=1;
    return s;
}

QString HardwareSyncCalibrationWidget::activePath() const
{
    QString base=QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);
    if(base.isEmpty())base=QDir::homePath()+"/.motionlasersync";
    QDir d(base);d.mkpath("calibration");
    return d.filePath(m_selectedTopologyId==3
        ? "calibration/active_hardware_temporal_H3.json"
        : "calibration/active_hardware_temporal_H1.json");
}

bool HardwareSyncCalibrationWidget::saveJson(const QString& path,bool runtimeEnabled,QString* error) const
{
    if(!m_haveResult||!m_result.success){if(error)*error=tr("没有可保存的求解结果");return false;}QJsonObject root;root["schemaVersion"]=4;root["softwareVersion"]="V32.16-H1-H3-SIGNED-TEMPORAL";root["calibrationId"]=m_calibrationId;root["runtimeEnabled"]=runtimeEnabled;
    root["topologyId"]=m_selectedTopologyId;root["latchEdge"]=m_sessionLatchEdge;
    root["cmmDivider"]=(m_selectedTopologyId==3?1:0);root["cmmSkewHalfUs"]=0;root["cmmRatio"]=1;
    root["topology"]=m_selectedTopologyId==1
        ? "H1 ECI OP14->sensor + IN0/R0 REGIST(104)" : "H3 scanCONTROL CMM->IN1/R1 REGIST(114/115)";
    root["delayDefinition"]="t_sensor_effective - t_eci_latch";root["delaySec"]=m_result.delaySec;root["delayStdSec"]=m_result.delayStdSec;root["naivePairDelaySec"]=m_result.naivePairDelaySec;root["driftModelUsed"]=m_result.driftModelUsed;root["centerDriftMmPerSec"]=vecJson(m_result.centerDriftMmPerSec);root["timeSpanSec"]=m_result.timeSpanSec;root["meanEffectiveRadiusMm"]=m_result.meanEffectiveRadiusMm;root["maxPairRadiusDifferenceMm"]=m_result.maxPairRadiusDifferenceMm;root["qualityPassed"]=m_result.qualityPassed;root["validationPassed"]=m_validationPassed;
    QJsonObject ref;ref["cloudPath"]=m_referenceCloudPath;ref["centerMachineMm"]=vecJson(m_referenceFit.centerMachineMm);ref["radiusMm"]=m_referenceFit.knownRadiusMm;ref["rmsMm"]=m_referenceFit.residualRmsMm;ref["inliers"]=QString::number(qulonglong(m_referenceFit.pointCount));root["referenceSphere"]=ref;
    QJsonObject center;center["valid"]=m_planCenterValid;center["x"]=m_planCenter.x;center["y"]=m_planCenter.y;center["z"]=m_planCenter.z;center["c"]=m_planCenter.c;center["b"]=m_planCenter.b;root["planCenter"]=center;
    QJsonObject experiment;experiment["minSpeedMmPerSec"]=m_templateMinSpeed?m_templateMinSpeed->value():0.5;experiment["maxSpeedMmPerSec"]=m_templateMaxSpeed?m_templateMaxSpeed->value():5.0;experiment["speedStepMmPerSec"]=m_templateSpeedStep?m_templateSpeedStep->value():0.5;experiment["calibrationRepeatsPerSpeed"]=m_templateCalibrationRepeats?m_templateCalibrationRepeats->value():3;experiment["distanceMm"]=m_templateDistance?m_templateDistance->value():30.0;experiment["validationSpeedMmPerSec"]=m_templateValidationSpeed?m_templateValidationSpeed->value():4.0;experiment["validationRepeatsPerTrajectory"]=m_templateValidationRepeats?m_templateValidationRepeats->value():3;root["experimentTemplate"]=experiment;
    QJsonObject archive;archive["rootDirectory"]=m_captureRootDirectory?m_captureRootDirectory->text():QStringLiteral("E:/Projects/CData");archive["sessionId"]=m_captureSessionId;archive["sessionDirectory"]=m_captureSessionDirectory;archive["archiveEachScan"]=m_archiveEachScan&&m_archiveEachScan->isChecked();archive["maxAnalysisPoints"]=m_maxAnalysisPoints?m_maxAnalysisPoints->value():250000;root["captureArchive"]=archive;
    QJsonArray captureFiles;for(const auto&p:m_plannedScans){if(p.archivePath.isEmpty()&&p.metadataPath.isEmpty())continue;QJsonObject j;j["planId"]=QString::number(qulonglong(p.id));j["pairId"]=QString::number(qulonglong(p.pairId));j["role"]=p.role==ScanRole::Validation?QStringLiteral("validation"):QStringLiteral("calibration");j["name"]=p.name;j["directionSign"]=p.directionSign;j["rawScanPath"]=p.archivePath;j["metadataPath"]=p.metadataPath;captureFiles.append(j);}root["captureFiles"]=captureFiles;
    QJsonArray configs;for(int r=0;r<m_planConfigTable->rowCount();++r){QJsonObject j;j["enabled"]=m_planConfigTable->item(r,0)&&m_planConfigTable->item(r,0)->checkState()==Qt::Checked;for(int c=1;c<kPlanCols;++c)j[QStringLiteral("c%1").arg(c)]=m_planConfigTable->item(r,c)?m_planConfigTable->item(r,c)->text():QString();configs.append(j);}root["planConfigs"]=configs;
    QJsonArray obs;for(const auto&o:m_calibrationObservations)obs.append(trajectoryObservationJson(o));root["calibrationObservations"]=obs;
    QJsonArray vals;for(const auto&vc:m_validationCaptures){const auto&o=vc.observation;QJsonObject j;j["id"]=QString::number(qulonglong(o.id));j["pairId"]=QString::number(qulonglong(o.pairId));j["directionSign"]=o.directionSign;j["velocityMmPerSec"]=vecJson(o.velocityMmPerSec);j["rawCenter"]=vecJson(o.rawSphereCenterMachineMm);j["correctedCenter"]=vecJson(o.correctedSphereCenterMachineMm);j["rawRmsMm"]=o.rawSphereRmsMm;j["correctedRmsMm"]=o.correctedSphereRmsMm;j["captureMidPcUs"]=QString::number(qlonglong(o.captureMidPcUs));vals.append(j);}root["validationObservations"]=vals;
    QJsonObject sensor;sensor["resolutionIndex"]=int(m_result.resolutionIndex);sensor["exposureTime10Us"]=int(m_result.exposureTime10Us);sensor["idleTime10Us"]=int(m_result.idleTime10Us);sensor["measuringFieldIndex"]=m_result.measuringFieldIndex;sensor["triggerFrequencyHz"]=m_result.triggerFrequencyHz;sensor["triggerPeriodMs"]=m_result.triggerPeriodMs;root["sensorTiming"]=sensor;
    QSaveFile f(path);if(!f.open(QIODevice::WriteOnly)){if(error)*error=f.errorString();return false;}const auto bytes=QJsonDocument(root).toJson(QJsonDocument::Indented);if(f.write(bytes)!=bytes.size()||!f.commit()){if(error)*error=f.errorString();return false;}if(error)error->clear();return true;
}

bool HardwareSyncCalibrationWidget::loadJson(const QString& path,bool applyRuntime,QString* error)
{
    QFile f(path);if(!f.open(QIODevice::ReadOnly)){if(error)*error=f.errorString();return false;}QJsonParseError pe;const auto doc=QJsonDocument::fromJson(f.readAll(),&pe);if(pe.error!=QJsonParseError::NoError||!doc.isObject()){if(error)*error=tr("JSON解析失败：%1").arg(pe.errorString());return false;}const auto root=doc.object();
    const int schema=root.value("schemaVersion").toInt();
    const int storedTopology=schema==3 ? 1 : root.value("topologyId").toInt();
    const int storedEdge=schema==3 ? 104 : root.value("latchEdge").toInt();
    const bool validType=mls::runtime::Table5D::validTopologyEdge(storedTopology,storedEdge);
    if((schema!=3&&schema!=4) || (schema==3 && !root.value("topology").toString().contains("REGIST(104)")) ||
       storedTopology!=m_selectedTopologyId || !validType ||
       root.value("delayDefinition").toString()!=QStringLiteral("t_sensor_effective - t_eci_latch") ||
       (storedTopology==3 && (schema!=4 || root.value("cmmDivider").toInt()!=1 ||
           root.value("cmmSkewHalfUs").toInt()!=0 || root.value("cmmRatio").toInt()!=1))){
        if(error)*error=tr("标定文件拓扑/CMM配置不匹配：H1与H3严禁互相加载，H3必须使用包含R1边沿指纹的新版JSON");
        return false;
    }
    m_sessionLatchEdge=storedEdge;
    const auto ref=root.value("referenceSphere").toObject();m_referenceFit.centerMachineMm=jsonVec(ref.value("centerMachineMm"));m_referenceFit.knownRadiusMm=ref.value("radiusMm").toDouble();m_referenceFit.residualRmsMm=ref.value("rmsMm").toDouble();m_referenceFit.pointCount=std::size_t(ref.value("inliers").toString().toULongLong());m_referenceFit.success=m_referenceFit.knownRadiusMm>0;m_referenceFitValid=m_referenceFit.success;m_referenceCloudPath=ref.value("cloudPath").toString();m_referencePath->setText(m_referenceCloudPath);if(m_referenceFitValid)emit referenceSphereChanged(m_referenceFit.centerMachineMm.x,m_referenceFit.centerMachineMm.y,m_referenceFit.centerMachineMm.z,m_referenceFit.knownRadiusMm,true);
    const auto ce=root.value("planCenter").toObject();m_planCenterValid=ce.value("valid").toBool(false);m_planCenter={ce.value("x").toDouble(),ce.value("y").toDouble(),ce.value("z").toDouble(),ce.value("c").toDouble(),ce.value("b").toDouble()};if(m_planCenterValid)m_planCenterLabel->setText(tr("扫描轨迹中点 XYZ=(%1,%2,%3) C/B=(%4,%5)").arg(m_planCenter.x,0,'f',4).arg(m_planCenter.y,0,'f',4).arg(m_planCenter.z,0,'f',4).arg(m_planCenter.c,0,'f',3).arg(m_planCenter.b,0,'f',3));
    const auto ex=root.value("experimentTemplate").toObject();if(!ex.isEmpty()){if(m_templateMinSpeed)m_templateMinSpeed->setValue(ex.value("minSpeedMmPerSec").toDouble(0.5));if(m_templateMaxSpeed)m_templateMaxSpeed->setValue(ex.value("maxSpeedMmPerSec").toDouble(5.0));if(m_templateSpeedStep)m_templateSpeedStep->setValue(ex.value("speedStepMmPerSec").toDouble(0.5));if(m_templateCalibrationRepeats)m_templateCalibrationRepeats->setValue(ex.value("calibrationRepeatsPerSpeed").toInt(3));if(m_templateDistance)m_templateDistance->setValue(ex.value("distanceMm").toDouble(30.0));if(m_templateValidationSpeed)m_templateValidationSpeed->setValue(ex.value("validationSpeedMmPerSec").toDouble(4.0));if(m_templateValidationRepeats)m_templateValidationRepeats->setValue(ex.value("validationRepeatsPerTrajectory").toInt(3));}
    const auto ar=root.value("captureArchive").toObject();if(!ar.isEmpty()){if(m_captureRootDirectory)m_captureRootDirectory->setText(ar.value("rootDirectory").toString(QStringLiteral("E:/Projects/CData")));if(m_archiveEachScan)m_archiveEachScan->setChecked(ar.value("archiveEachScan").toBool(true));if(m_maxAnalysisPoints)m_maxAnalysisPoints->setValue(ar.value("maxAnalysisPoints").toInt(250000));m_captureSessionId=ar.value("sessionId").toString();m_captureSessionDirectory=ar.value("sessionDirectory").toString();}
    m_planConfigTable->setRowCount(0);for(const auto&v:root.value("planConfigs").toArray()){const auto j=v.toObject();addPlanRow();const int r=m_planConfigTable->rowCount()-1;m_planConfigTable->item(r,0)->setCheckState(j.value("enabled").toBool(true)?Qt::Checked:Qt::Unchecked);for(int c=1;c<kPlanCols;++c)m_planConfigTable->item(r,c)->setText(j.value(QStringLiteral("c%1").arg(c)).toString());}
    m_calibrationObservations.clear();for(const auto&v:root.value("calibrationObservations").toArray()){TemporalTrajectoryObservation o;if(trajectoryObservationFromJson(v.toObject(),o))m_calibrationObservations.push_back(o);}m_result=HardwareTemporalCalibration::solveTrajectoryPairs(m_calibrationObservations);m_haveResult=m_result.success;if(m_haveResult)HardwareTemporalCalibration::evaluateTrajectoryQuality(m_result,trajectoryCriteriaFromUi());m_calibrationId=root.value("calibrationId").toString();m_validationCaptures.clear();for(const auto&v:root.value("validationObservations").toArray()){const auto j=v.toObject();ValidationCapture vc;vc.observation.id=j.value("id").toString().toULongLong();vc.observation.pairId=j.value("pairId").toString().toULongLong();vc.observation.directionSign=j.value("directionSign").toInt();vc.observation.velocityMmPerSec=jsonVec(j.value("velocityMmPerSec"));vc.observation.rawSphereCenterMachineMm=jsonVec(j.value("rawCenter"));vc.observation.correctedSphereCenterMachineMm=jsonVec(j.value("correctedCenter"));vc.observation.rawSphereRmsMm=j.value("rawRmsMm").toDouble();vc.observation.correctedSphereRmsMm=j.value("correctedRmsMm").toDouble();vc.observation.captureMidPcUs=j.value("captureMidPcUs").toString().toLongLong();m_validationCaptures.push_back(vc);}std::vector<TemporalValidationObservation> vo;for(const auto&v:m_validationCaptures)vo.push_back(v.observation);m_validationResult=HardwareTemporalCalibration::evaluateValidationPairs(vo,validationCriteriaFromUi());m_validationPassed=m_validationResult.success&&m_validationResult.passed&&root.value("validationPassed").toBool(false);

    // V27.6.53: always stage a validated active calibration into SyncManager, even
    // when runtimeEnabled=false.  This lets the Parameters page show tau/sigma and
    // lets the operator toggle application without reloading/re-solving anything.
    if(m_sync && m_haveResult && m_result.qualityPassed && m_validationPassed){
        HardwareTemporalCorrectionSettings staged=correctionSettings(applyRuntime&&root.value("runtimeEnabled").toBool(false));
        QString e;
        if(!m_sync->setHardwareTemporalCorrection(staged,&e)){if(error)*error=e;return false;}
        emit runtimeCorrectionChanged(staged.enabled,m_result.delaySec*1e6);
    }
    refreshReferenceUi();refreshCaptureTable();refreshResult();
    emit activeCalibrationChanged();
    if(error)error->clear();return true;
}

void HardwareSyncCalibrationWidget::saveCalibrationAs(){if(!m_haveResult)return;const auto p=QFileDialog::getSaveFileName(this,tr("保存硬件同步时域标定"),QStringLiteral("hardware_temporal_calibration_v51.json"),tr("JSON (*.json)"));if(p.isEmpty())return;QString e;if(!saveJson(p,m_sync&&m_sync->hardwareTemporalCorrection().enabled,&e))QMessageBox::warning(this,tr("保存失败"),e);}
void HardwareSyncCalibrationWidget::loadCalibrationFrom(){const auto p=QFileDialog::getOpenFileName(this,tr("加载硬件同步时域标定"),QString(),tr("JSON (*.json)"));if(p.isEmpty())return;QString e;if(!loadJson(p,false,&e))QMessageBox::warning(this,tr("加载失败"),e);else m_status->setText(tr("已加载兼容的硬件时域标定工作流；运行时补偿未自动开启，请检查验证结果后手动启用。"));}
void HardwareSyncCalibrationWidget::loadActiveOnBind()
{
    QString p=activePath();
    if(!QFile::exists(p) && m_selectedTopologyId==1){
        QString base=QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);
        if(base.isEmpty())base=QDir::homePath()+"/.motionlasersync";
        const QString legacy=QDir(base).filePath("calibration/active_hardware_temporal.json");
        if(QFile::exists(legacy))p=legacy; // read-only legacy fallback, never overwrite
    }
    if(!QFile::exists(p)){emit activeCalibrationChanged();return;}
    QString e;
    if(!loadJson(p,true,&e)){
        m_status->setText(tr("活动硬件时域标定自动加载失败：%1").arg(e));
        emit logMessage(tr("[硬件同步标定] %1").arg(m_status->text()));
    }else{
        const auto c=m_sync?m_sync->hardwareTemporalCorrection():HardwareTemporalCorrectionSettings{};
        m_status->setText(tr("已自动加载活动硬件时域标定：%1；运行时=%2，τ=%3 µs")
            .arg(m_calibrationId).arg(c.enabled?tr("ON"):tr("OFF")).arg(m_result.delaySec*1e6,0,'f',2));
        emit logMessage(tr("[硬件同步标定] %1").arg(m_status->text()));
    }
    emit activeCalibrationChanged();
}
