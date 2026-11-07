#include "widgets/PointCloudWidget.h"
#include "core/PointCloudBuilder.h"
#include "machine_config.h"
#include "machine_kinematics.h"
#include "stl_mesh.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QCheckBox>
#include <QLabel>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QSignalBlocker>
#include <QtConcurrentRun>
#include <QFileInfo>
#include <QResizeEvent>
#include <QDebug>
#include <QCursor>
#include <QWaitCondition>
#include <algorithm>
#include <cmath>
#include <limits>
#include <filesystem>
#include <vector>
#include <stdexcept>

// VTK 头文件
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkProperty.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkLookupTable.h>
#include <vtkScalarBarActor.h>
#include <vtkAxesActor.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkCamera.h>
#include <vtkFloatArray.h>
#include <vtkUnsignedCharArray.h>
#include <vtkPointData.h>
#include <vtkTextProperty.h>
#include <vtkNew.h>
#include <vtkSmartPointer.h>
#include <vtkPLYReader.h>
#include <vtkPolyDataNormals.h>
#include <vtkMatrix4x4.h>
#include <vtkSphereSource.h>

#include <QVTKOpenGLNativeWidget.h>

// 渲染帧率限制（毫秒）
static constexpr int kRenderIntervalMs = 200;  // 5 FPS

namespace {
void eigenToVtkMatrix(const Eigen::Isometry3d& T, vtkMatrix4x4* M)
{
    if (!M) return;
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            M->SetElement(r, c, T.matrix()(r, c));
}

std::array<double,3> machineLinkColor(const QString& name)
{
    if (name == QStringLiteral("01_BASE")) return {0.40,0.43,0.47};
    if (name == QStringLiteral("02_X_AXIS")) return {0.48,0.55,0.62};
    if (name == QStringLiteral("03_Y_AXIS")) return {0.53,0.60,0.67};
    if (name == QStringLiteral("04_Z_AXIS")) return {0.60,0.65,0.70};
    if (name == QStringLiteral("05_C_AXIS")) return {0.55,0.50,0.42};
    if (name == QStringLiteral("06_B_AXIS")) return {0.47,0.55,0.45};
    if (name == QStringLiteral("07_END_SENSOR")) return {0.82,0.73,0.28};
    return {0.70,0.72,0.75};
}

vtkSmartPointer<vtkPolyData> loadMachineCollisionPolyData(const std::filesystem::path& stlPath)
{
    const auto triangles = loadStlTriangles(stlPath);
    if (triangles.empty()) throw std::runtime_error("Collision STL is empty: " + stlPath.string());

    vtkNew<vtkPoints> points;
    points->SetDataTypeToDouble();
    points->Allocate(static_cast<vtkIdType>(triangles.size() * 3));
    vtkNew<vtkCellArray> cells;
    cells->AllocateEstimate(static_cast<vtkIdType>(triangles.size()), 3);
    for (const auto& t : triangles) {
        vtkIdType ids[3];
        ids[0] = points->InsertNextPoint(t.a.x(), t.a.y(), t.a.z());
        ids[1] = points->InsertNextPoint(t.b.x(), t.b.y(), t.b.z());
        ids[2] = points->InsertNextPoint(t.c.x(), t.c.y(), t.c.z());
        cells->InsertNextCell(3, ids);
    }
    auto poly = vtkSmartPointer<vtkPolyData>::New();
    poly->SetPoints(points);
    poly->SetPolys(cells);
    return poly;
}

vtkActor* makePolylineActor(const QVector<QVector3D>& points,
                            double r, double g, double b,
                            double width, double opacity)
{
    if (points.size() < 2) return nullptr;
    vtkNew<vtkPoints> vtkPts;
    vtkPts->SetDataTypeToDouble();
    vtkNew<vtkCellArray> lines;
    std::vector<vtkIdType> run;
    run.reserve(static_cast<std::size_t>(points.size()));
    auto flush=[&]{
        if(run.size()>=2)
            lines->InsertNextCell(static_cast<vtkIdType>(run.size()),run.data());
        run.clear();
    };
    for(const auto& p:points){
        // NaN is a polyline separator, not a VTK vertex. In particular, do
        // not connect two M6 laser-on passes over an unexposed transfer.
        if(!std::isfinite(p.x())||!std::isfinite(p.y())||!std::isfinite(p.z())){flush();continue;}
        run.push_back(vtkPts->InsertNextPoint(p.x(),p.y(),p.z()));
    }
    flush();
    if(lines->GetNumberOfCells()==0)return nullptr;
    vtkNew<vtkPolyData> poly;
    poly->SetPoints(vtkPts);
    poly->SetLines(lines);
    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputData(poly);
    mapper->ScalarVisibilityOff();
    vtkActor* actor = vtkActor::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(r,g,b);
    actor->GetProperty()->SetLineWidth(width);
    actor->GetProperty()->SetOpacity(opacity);
    return actor;
}

vtkActor* makeQuadActor(const QVector<QVector3D>& corners, bool feasible)
{
    if (corners.size() != 4) return nullptr;
    vtkNew<vtkPoints> vtkPts;
    vtkPts->SetDataTypeToDouble();
    for (const auto& p : corners)
        vtkPts->InsertNextPoint(p.x(), p.y(), p.z());
    vtkNew<vtkCellArray> polys;
    vtkIdType ids[4] = {0,1,2,3};
    polys->InsertNextCell(4, ids);
    vtkNew<vtkPolyData> poly;
    poly->SetPoints(vtkPts);
    poly->SetPolys(polys);
    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputData(poly);
    mapper->ScalarVisibilityOff();
    vtkActor* actor = vtkActor::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(feasible ? 0.15 : 0.95,
                                   feasible ? 0.72 : 0.20,
                                   feasible ? 0.95 : 0.10);
    actor->GetProperty()->SetOpacity(0.18);
    actor->GetProperty()->SetEdgeVisibility(1);
    actor->GetProperty()->SetLineWidth(1.4);
    return actor;
}

vtkActor* makePointCloudActor(const QVector<QVector3D>& points,
                              double r, double g, double b,
                              double pointSize, double opacity)
{
    if (points.isEmpty()) return nullptr;
    vtkNew<vtkPoints> vtkPts;
    vtkPts->SetDataTypeToDouble();
    vtkNew<vtkCellArray> verts;
    for (const auto& p : points) {
        const vtkIdType id = vtkPts->InsertNextPoint(p.x(), p.y(), p.z());
        verts->InsertNextCell(1);
        verts->InsertCellPoint(id);
    }
    vtkNew<vtkPolyData> poly;
    poly->SetPoints(vtkPts);
    poly->SetVerts(verts);
    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputData(poly);
    mapper->ScalarVisibilityOff();
    vtkActor* actor = vtkActor::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(r,g,b);
    actor->GetProperty()->SetPointSize(pointSize);
    actor->GetProperty()->SetOpacity(opacity);
    return actor;
}
}

// ============================================================================
// PointCloudPrepareThread - 后台数据准备线程
// ============================================================================

PointCloudPrepareThread::PointCloudPrepareThread(QObject* parent)
    : QThread(parent)
{
}

PointCloudPrepareThread::~PointCloudPrepareThread()
{
    requestStop();
    wait(3000);
}

void PointCloudPrepareThread::requestUpdate()
{
    QMutexLocker locker(&m_mutex);
    m_updateRequested.store(true);
    m_condition.wakeOne();
}

void PointCloudPrepareThread::requestStop()
{
    QMutexLocker locker(&m_mutex);
    m_stopRequested.store(true);
    m_condition.wakeAll();
}

void PointCloudPrepareThread::run()
{
    while (!m_stopRequested.load()) {
        // 等待更新请求
        {
            QMutexLocker locker(&m_mutex);
            while (!m_updateRequested.load() && !m_stopRequested.load()) {
                m_condition.wait(&m_mutex, 100);  // 100ms 超时
            }
            
            if (m_stopRequested.load()) break;
            m_updateRequested.store(false);
        }
        
        if (!m_builder) continue;
        
        // 获取当前点数
        int totalPoints = m_builder->pointCount();
        
        // 如果点数没变化，跳过
        if (totalPoints == m_lastProcessedCount) continue;
        if (totalPoints == 0) {
            m_lastProcessedCount = 0;
            emit dataReady(QVector<QVector3D>(), 0, 0);
            continue;
        }
        
        // 【关键】在后台线程拷贝数据，不阻塞主线程
        QVector<QVector3D> pointsCopy;
        float zMin = std::numeric_limits<float>::max();
        float zMax = std::numeric_limits<float>::lowest();
        
        {
            QReadLocker locker(&m_builder->rwLock());
            const QVector<QVector3D>& points = m_builder->getPointsRef();
            
            const int numPoints = points.size();
            
            // 最大显示点数（VTK 渲染的合理上限）
            constexpr int kMaxDisplayPoints = 2000000;  // 200万点
            
            if (numPoints <= kMaxDisplayPoints) {
                // 点数不多，全部显示
                pointsCopy.reserve(numPoints);
                for (int i = 0; i < numPoints; ++i) {
                    const auto& p = points[i];
                    pointsCopy.append(p);
                    if (p.z() < zMin) zMin = p.z();
                    if (p.z() > zMax) zMax = p.z();
                }
            } else {
                // 需要降采样：使用分层采样策略
                // 
                // 策略说明：
                // 1. 将点云按 Profile 分组（假设每个 Profile 约 1280 点）
                // 2. 确保每个 Profile 都有足够的点显示
                // 3. 在每个 Profile 内均匀采样
                //
                // 假设：
                // - 每条 Profile 约 1280 个点
                // - 典型采集 50000 条 Profile = 6400万点
                // - 目标显示 200万点
                //
                // 计算：
                // - 估算 Profile 数量 = totalPoints / 1280
                // - 每条 Profile 采样点数 = maxPoints / profileCount
                // - 但每条 Profile 至少保留 50 个点以保证轮廓形状
                
                constexpr int kPointsPerProfile = 1280;  // 每条 Profile 的典型点数
                constexpr int kMinPointsPerProfile = 100; // 每条 Profile 最少保留的点数
                
                int estimatedProfiles = numPoints / kPointsPerProfile;
                if (estimatedProfiles < 1) estimatedProfiles = 1;
                
                // 计算每条 Profile 应该保留的点数
                int pointsPerProfile = kMaxDisplayPoints / estimatedProfiles;
                if (pointsPerProfile < kMinPointsPerProfile) {
                    pointsPerProfile = kMinPointsPerProfile;
                }
                if (pointsPerProfile > kPointsPerProfile) {
                    pointsPerProfile = kPointsPerProfile;
                }
                
                // Profile 内的采样步长
                int profileStep = kPointsPerProfile / pointsPerProfile;
                if (profileStep < 1) profileStep = 1;
                
                // 如果 Profile 太多，也需要跳过一些 Profile
                int maxProfiles = kMaxDisplayPoints / pointsPerProfile;
                int profileSkip = 1;
                if (estimatedProfiles > maxProfiles) {
                    profileSkip = (estimatedProfiles + maxProfiles - 1) / maxProfiles;
                }
                
                // 预估输出大小
                int estimatedOutput = (estimatedProfiles / profileSkip) * pointsPerProfile;
                pointsCopy.reserve(estimatedOutput);
                
                // 分层采样
                int currentProfile = 0;
                int pointInProfile = 0;
                int profilesProcessed = 0;
                
                for (int i = 0; i < numPoints; ++i) {
                    // 判断是否进入新的 Profile
                    int newProfile = i / kPointsPerProfile;
                    if (newProfile != currentProfile) {
                        currentProfile = newProfile;
                        pointInProfile = 0;
                        profilesProcessed++;
                    }
                    
                    // 跳过部分 Profile
                    if (profileSkip > 1 && (currentProfile % profileSkip) != 0) {
                        pointInProfile++;
                        continue;
                    }
                    
                    // Profile 内采样
                    if ((pointInProfile % profileStep) == 0) {
                        const auto& p = points[i];
                        pointsCopy.append(p);
                        if (p.z() < zMin) zMin = p.z();
                        if (p.z() > zMax) zMax = p.z();
                    }
                    
                    pointInProfile++;
                }
            }
        }
        
        m_lastProcessedCount = totalPoints;
        
        // 发送数据到主线程
        emit dataReady(pointsCopy, zMin, zMax);
    }
}

// ============================================================================
// PointCloudWidget
// ============================================================================

PointCloudWidget::PointCloudWidget(QWidget* parent)
    : QWidget(parent)
{
    initializeVTK();
    
    // 创建后台线程
    m_prepareThread = new PointCloudPrepareThread(this);
    connect(m_prepareThread, &PointCloudPrepareThread::dataReady,
            this, &PointCloudWidget::onDataReady,
            Qt::QueuedConnection);
    m_prepareThread->start();
    
    // 渲染定时器
    m_renderTimer.setInterval(kRenderIntervalMs);
    connect(&m_renderTimer, &QTimer::timeout, this, &PointCloudWidget::onRenderTimer);
    m_renderTimer.start();
}

PointCloudWidget::~PointCloudWidget()
{
    m_renderTimer.stop();
    
    // 停止后台线程
    if (m_prepareThread) {
        m_prepareThread->requestStop();
        m_prepareThread->wait(3000);
        delete m_prepareThread;
        m_prepareThread = nullptr;
    }
    
    // 清理 VTK 对象
    if (m_axesWidget) {
        m_axesWidget->SetEnabled(false);
        m_axesWidget->Delete();
    }
    for (auto& item : m_machineActors) {
        if (item.actor) {
            if (m_renderer) m_renderer->RemoveActor(item.actor);
            item.actor->Delete();
            item.actor = nullptr;
        }
    }
    m_machineActors.clear();
    for (auto& item : m_machineCollisionActors) {
        if (item.actor) {
            if (m_renderer) m_renderer->RemoveActor(item.actor);
            item.actor->Delete();
            item.actor = nullptr;
        }
    }
    m_machineCollisionActors.clear();
    if (m_machineStudySphereActor) { if (m_renderer) m_renderer->RemoveActor(m_machineStudySphereActor); m_machineStudySphereActor->Delete(); m_machineStudySphereActor=nullptr; }
    if (m_machineStudySafetySphereActor) { if (m_renderer) m_renderer->RemoveActor(m_machineStudySafetySphereActor); m_machineStudySafetySphereActor->Delete(); m_machineStudySafetySphereActor=nullptr; }
    if (m_machineStudySphereSource) { m_machineStudySphereSource->Delete(); m_machineStudySphereSource=nullptr; }
    if (m_machineStudySafetySphereSource) { m_machineStudySafetySphereSource->Delete(); m_machineStudySafetySphereSource=nullptr; }
    clearTwinReplayPoints();
    clearTwinVirtualPoints();
    clearTwinCoveragePoints();
    clearTwinM6Results();
    setTwinMultiPosePlaybackActive(false,true);
    clearTwinMultiPoseComparisonHeatmap();
    if(m_multiPoseLaserActor){if(m_renderer)m_renderer->RemoveActor(m_multiPoseLaserActor);m_multiPoseLaserActor->Delete();m_multiPoseLaserActor=nullptr;}
    if(m_multiPoseLaserPoints){m_multiPoseLaserPoints->Delete();m_multiPoseLaserPoints=nullptr;}
    if(m_multiPoseAccumActor){if(m_renderer)m_renderer->RemoveActor(m_multiPoseAccumActor);m_multiPoseAccumActor->Delete();m_multiPoseAccumActor=nullptr;}
    if(m_multiPoseAccumPoly){m_multiPoseAccumPoly->Delete();m_multiPoseAccumPoly=nullptr;}
    if(m_multiPoseAccumCells){m_multiPoseAccumCells->Delete();m_multiPoseAccumCells=nullptr;}
    if(m_multiPoseAccumPoints){m_multiPoseAccumPoints->Delete();m_multiPoseAccumPoints=nullptr;}
    clearTwinMultiPoseResults();
    clearTwinMultiPoseWorkpiece();
    clearTwinMultiPoseRecordedPoints();
    clearTwinM7Results();
    for(const auto& layer:QStringList{QStringLiteral("M4_ROI"),QStringLiteral("M5_FOCUS"),
         QStringLiteral("M51_SPARSE"),QStringLiteral("M52_RESIDUAL")})
        setTwinStageMarkers(layer,{});
    clearTwinWorkpiece();
    setTwinSupportPlane(false,0,{},{});
    clearSphereStudyScanSimulation();
    clearSphereStudySimulatedAcquisitionCloud();
    clearSphereExposureSweepPath();
    if (m_scalarBar) m_scalarBar->Delete();
    if (m_axesActor) m_axesActor->Delete();
    leaveImportedMode();
    if (m_importMeshActor) { m_renderer->RemoveActor(m_importMeshActor); m_importMeshActor->Delete(); }
    if (m_importMeshMapper) m_importMeshMapper->Delete();
    if (m_importMeshPolyData) m_importMeshPolyData->Delete();
    if (m_lut) m_lut->Delete();
    if (m_actor) m_actor->Delete();
    if (m_mapper) m_mapper->Delete();
    if (m_glyphFilter) m_glyphFilter->Delete();
    if (m_polyData) m_polyData->Delete();
    if (m_vtkPoints) m_vtkPoints->Delete();
    if (m_renderer) m_renderer->Delete();
}

void PointCloudWidget::initializeVTK()
{
    m_vtkWidget = new QVTKOpenGLNativeWidget(this);
    // Pure graphics surface: it never needs text input/IME.  Keeping IME off here
    // also avoids third-party IME cursor/bitmap hooks being attached to the VTK HWND.
    m_vtkWidget->setAttribute(Qt::WA_InputMethodEnabled, false);
    m_vtkWidget->setDefaultCursor(QCursor(Qt::ArrowCursor));
    m_vtkWidget->setCursor(Qt::ArrowCursor);
    
    QVBoxLayout* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(2);

    auto* machineBar = new QWidget(this);
    auto* machineBarLayout = new QHBoxLayout(machineBar);
    machineBarLayout->setContentsMargins(6, 2, 6, 2);
    m_machineVisibleCheck = new QCheckBox(tr("机床3D模型"), machineBar);
    m_machineVisibleCheck->setChecked(true);
    m_machineCollisionVisibleCheck = new QCheckBox(tr("FCL碰撞包络"), machineBar);
    m_machineCollisionVisibleCheck->setChecked(false);
    m_machineCollisionVisibleCheck->setToolTip(tr("显示与FCL碰撞引擎完全相同的STL碰撞网格；默认关闭以减少画面遮挡。"));
    m_machineSafetyLabel = new QLabel(tr("机床模型：未加载"), machineBar);
    m_machineSafetyLabel->setStyleSheet(QStringLiteral("QLabel { color:#777; }"));
    m_temporalCorrectionLabel = new QLabel(tr("Temporal Runtime: OFF / 未加载"), machineBar);
    m_temporalCorrectionLabel->setToolTip(tr("硬件同步时域补偿状态。ON表示三维点云使用q(t+τ)经Physical17重建；OFF表示使用原始同ordinal REGIST(104)位姿。"));
    m_temporalCorrectionLabel->setStyleSheet(QStringLiteral("QLabel { color:#777; font-weight:600; padding-left:8px; }"));
    machineBarLayout->addWidget(m_machineVisibleCheck);
    machineBarLayout->addSpacing(6);
    machineBarLayout->addWidget(m_machineCollisionVisibleCheck);
    machineBarLayout->addSpacing(8);
    machineBarLayout->addWidget(m_machineSafetyLabel, 1);
    machineBarLayout->addWidget(m_temporalCorrectionLabel);
    connect(m_machineVisibleCheck, &QCheckBox::toggled, this, &PointCloudWidget::setMachineModelVisible);
    connect(m_machineCollisionVisibleCheck, &QCheckBox::toggled, this, &PointCloudWidget::setMachineCollisionModelVisible);
    layout->addWidget(machineBar);

    // A second, compact row belongs to imported CLOUD data only. During
    // real-time capture it is hidden and the original vertex-glyph path is
    // untouched (including the existing 5 FPS preview throttling).
    m_importControls = new QWidget(this);
    auto* importBar = new QHBoxLayout(m_importControls);
    importBar->setContentsMargins(6, 2, 6, 2);
    importBar->setSpacing(5);
    importBar->addWidget(new QLabel(tr("导入点云："), m_importControls));
    m_importDisplayMode = new QComboBox(m_importControls);
    m_importDisplayMode->addItems({tr("散点"), tr("三角网格"), tr("网格+边线")});
    m_importDisplayMode->setToolTip(tr("仅修改导入文件的显示。实时硬件采集始终使用原来的散点方式。"));
    importBar->addWidget(m_importDisplayMode);
    importBar->addWidget(new QLabel(tr("轮廓宽度"), m_importControls));
    m_importProfileWidth = new QComboBox(m_importControls);
    m_importProfileWidth->addItem(tr("自动"), 0);
    m_importProfileWidth->addItem(QStringLiteral("1280"), 1280);
    m_importProfileWidth->addItem(QStringLiteral("640"), 640);
    m_importProfileWidth->setToolTip(tr("SCAN始终使用文件中逐轮廓的原始采样编号；PLY/CSV按保存顺序组织为640或1280列，必须保证未排序、未丢弃样本。"));
    importBar->addWidget(m_importProfileWidth);
    importBar->addWidget(new QLabel(tr("边长上限"), m_importControls));
    m_importEdgeLimit = new QDoubleSpinBox(m_importControls);
    m_importEdgeLimit->setDecimals(2);
    m_importEdgeLimit->setRange(0.0, 1000.0);
    m_importEdgeLimit->setSingleStep(0.5);
    m_importEdgeLimit->setValue(0.0);
    m_importEdgeLimit->setSpecialValueText(tr("自动"));
    m_importEdgeLimit->setSuffix(QStringLiteral(" mm"));
    m_importEdgeLimit->setToolTip(tr("0表示按轮廓内/轮廓间典型间距分别自动确定断边阈值；正数为手动限制。"));
    importBar->addWidget(m_importEdgeLimit);
    importBar->addWidget(new QLabel(tr("颜色"), m_importControls));
    m_importColorMode = new QComboBox(m_importControls);
    m_importColorMode->addItems({tr("Z高度伪彩"), tr("单色")});
    importBar->addWidget(m_importColorMode);
    m_importStatus = new QLabel(tr("未导入"), m_importControls);
    m_importStatus->setMinimumWidth(130);
    m_importStatus->setToolTip(tr("网格仅供查看。安全阈值过滤跨越/缺测连接；不改变XYZ及原始SCAN记录。"));
    importBar->addWidget(m_importStatus, 1);
    m_importControls->setVisible(false);
    layout->addWidget(m_importControls);

    connect(m_importDisplayMode, &QComboBox::currentIndexChanged,
            this, [this](int){ refreshImportedRepresentation(); });
    connect(m_importProfileWidth, &QComboBox::currentIndexChanged,
            this, [this](int){
        if (!m_importActive) return;
        invalidateImportedMesh();
        refreshImportedRepresentation();
    });
    connect(m_importEdgeLimit, &QDoubleSpinBox::valueChanged,
            this, [this](double){
        if (!m_importActive) return;
        invalidateImportedMesh();
        refreshImportedRepresentation();
    });
    connect(m_importColorMode, &QComboBox::currentIndexChanged,
            this, [this](int index){
        if (m_importActive) setColorMode(index==0 ? ColorMode::ByHeight : ColorMode::Uniform);
    });

    m_sphereStudySimulationLabel = new QLabel(tr("扫描仿真：未启用"), this);
    m_sphereStudySimulationLabel->setWordWrap(true);
    m_sphereStudySimulationLabel->setStyleSheet(QStringLiteral(
        "QLabel { color:#315a72; padding:2px 6px; background:#eef7fb; border:1px solid #c7dce8; }"));
    m_sphereStudySimulationLabel->setVisible(false);
    layout->addWidget(m_sphereStudySimulationLabel);
    layout->addWidget(m_vtkWidget, 1);

    vtkNew<vtkGenericOpenGLRenderWindow> renderWindow;
    m_vtkWidget->setRenderWindow(renderWindow);
    m_renderWindow = renderWindow;

    m_renderer = vtkRenderer::New();
    m_renderer->SetBackground(0.1, 0.1, 0.15);
    m_renderer->SetBackground2(0.2, 0.2, 0.25);
    m_renderer->GradientBackgroundOn();
    m_renderWindow->AddRenderer(m_renderer);

    m_interactor = m_renderWindow->GetInteractor();
    
    vtkNew<vtkInteractorStyleTrackballCamera> style;
    m_interactor->SetInteractorStyle(style);

    m_vtkPoints = vtkPoints::New();
    m_vtkPoints->SetDataTypeToFloat();
    
    m_polyData = vtkPolyData::New();
    m_polyData->SetPoints(m_vtkPoints);

    m_glyphFilter = vtkVertexGlyphFilter::New();
    m_glyphFilter->SetInputData(m_polyData);

    m_lut = vtkLookupTable::New();
    m_lut->SetHueRange(0.667, 0.0);
    m_lut->SetSaturationRange(1.0, 1.0);
    m_lut->SetValueRange(1.0, 1.0);
    m_lut->SetNumberOfTableValues(256);
    m_lut->Build();

    m_mapper = vtkPolyDataMapper::New();
    m_mapper->SetInputConnection(m_glyphFilter->GetOutputPort());
    m_mapper->SetLookupTable(m_lut);
    m_mapper->SetScalarVisibility(true);
    // 注意：VTK 9.x 已移除 ImmediateModeRenderingOff()，默认使用显示列表模式

    m_actor = vtkActor::New();
    m_actor->SetMapper(m_mapper);
    m_actor->GetProperty()->SetPointSize(m_pointSize);
    m_actor->GetProperty()->SetColor(0.9, 0.9, 0.9);
    m_renderer->AddActor(m_actor);

    // Use the SAME XYZ tuples and Height scalars for points and surfaces;
    // only connectivity is different. Avoid a second 3D-coordinate VBO.
    m_importMeshPolyData = vtkPolyData::New();
    m_importMeshPolyData->SetPoints(m_vtkPoints);
    m_importMeshMapper = vtkPolyDataMapper::New();
    m_importMeshMapper->SetInputData(m_importMeshPolyData);
    m_importMeshMapper->SetLookupTable(m_lut);
    m_importMeshMapper->SetScalarModeToUsePointData();
    m_importMeshMapper->SelectColorArray("Height");
    m_importMeshMapper->InterpolateScalarsBeforeMappingOn();
    m_importMeshActor = vtkActor::New();
    m_importMeshActor->SetMapper(m_importMeshMapper);
    m_importMeshActor->GetProperty()->BackfaceCullingOff();
    m_importMeshActor->GetProperty()->SetEdgeColor(0.12, 0.14, 0.17);
    m_importMeshActor->GetProperty()->SetEdgeVisibility(0);
    m_importMeshActor->SetVisibility(0);
    m_renderer->AddActor(m_importMeshActor);

    setupAxes();
    setupScalarBar();
    setupInitialCamera();  // 设置基于工作范围的初始相机视角
}

void PointCloudWidget::setupAxes()
{
    m_axesActor = vtkAxesActor::New();
    m_axesActor->SetTotalLength(10, 10, 10);
    m_axesActor->SetShaftTypeToCylinder();
    m_axesActor->SetCylinderRadius(0.02);
    m_axesActor->SetConeRadius(0.2);
    m_axesActor->SetXAxisLabelText("X");
    m_axesActor->SetYAxisLabelText("Y");
    m_axesActor->SetZAxisLabelText("Z");

    m_axesWidget = vtkOrientationMarkerWidget::New();
    m_axesWidget->SetOrientationMarker(m_axesActor);
    m_axesWidget->SetInteractor(m_interactor);
    m_axesWidget->SetViewport(0.0, 0.0, 0.15, 0.15);

    // VTK requires Interactor + Enabled before SetInteractive().  Enable first,
    // then immediately make the marker passive; this removes its mouse/cursor
    // observers without producing vtkOrientationMarkerWidget.cxx:544 warnings.
    m_axesWidget->SetEnabled(m_axesVisible ? 1 : 0);
    if (m_axesVisible) m_axesWidget->InteractiveOff();
    m_vtkWidget->setCursor(Qt::ArrowCursor);
}

void PointCloudWidget::setupScalarBar()
{
    m_scalarBar = vtkScalarBarActor::New();
    m_scalarBar->SetLookupTable(m_lut);
    m_scalarBar->SetTitle("Z (mm)");
    m_scalarBar->SetNumberOfLabels(5);
    m_scalarBar->SetWidth(0.08);
    m_scalarBar->SetHeight(0.4);
    m_scalarBar->SetPosition(0.9, 0.3);
    m_scalarBar->GetTitleTextProperty()->SetFontSize(12);
    m_scalarBar->GetTitleTextProperty()->SetColor(1, 1, 1);
    m_scalarBar->GetTitleTextProperty()->SetBold(0);
    m_scalarBar->GetLabelTextProperty()->SetFontSize(10);
    m_scalarBar->GetLabelTextProperty()->SetColor(1, 1, 1);
    m_scalarBar->SetVisibility(m_scalarBarVisible && m_colorMode == ColorMode::ByHeight ? 1 : 0);
    m_renderer->AddActor2D(m_scalarBar);
}

void PointCloudWidget::setPointCloudBuilder(PointCloudBuilder* builder)
{
    m_builder = builder;
    
    if (m_prepareThread) {
        m_prepareThread->setBuilder(builder);
    }
    
    if (m_builder) {
        connect(m_builder, &PointCloudBuilder::pointsAdded,
                this, &PointCloudWidget::onPointsAdded);
        connect(m_builder, &PointCloudBuilder::cleared,
                this, &PointCloudWidget::onCleared);
    }
}

void PointCloudWidget::onPointsAdded(int addedCount, int totalCount)
{
    Q_UNUSED(addedCount)
    Q_UNUSED(totalCount)
    m_needsUpdate.store(true);
}

void PointCloudWidget::onCleared()
{
    leaveImportedMode();
    m_needsUpdate.store(true);
    m_initialCameraSet = false;  // 重置标志，下次采集时重新设置相机
    
    // 清空待渲染数据
    {
        QMutexLocker locker(&m_pendingMutex);
        m_pendingPoints.clear();
        m_pendingZMin = 0;
        m_pendingZMax = 0;
    }
    
    // 立即清空显示
    m_vtkPoints->Reset();
    m_polyData->GetPointData()->Reset();
    m_polyData->Modified();
    
    // 重新设置初始相机视角
    setupInitialCamera();
    
    if (m_renderWindow) {
        m_renderWindow->Render();
    }
}

void PointCloudWidget::clear()
{
    onCleared();
}

int PointCloudWidget::pointCount() const
{
    if (m_importActive) return int(m_importedPoints.size());
    if (m_builder) {
        return m_builder->pointCount();
    }
    return 0;
}

void PointCloudWidget::setPoints(const QVector<QVector3D>& points)
{
    leaveImportedMode();
    if (points.isEmpty()) {
        clear();
        return;
    }
    
    // 计算Z范围
    float zMin = std::numeric_limits<float>::max();
    float zMax = std::numeric_limits<float>::lowest();
    for (const auto& p : points) {
        zMin = std::min(zMin, p.z());
        zMax = std::max(zMax, p.z());
    }
    
    // Direct, non-hardware preview (the QThread's queued signal must not
    // overwrite an imported cloud after it has been displayed).
    uploadPointsToVTK(points, zMin, zMax);
    
    // 重置相机以显示所有点
    resetCamera();
}


// ============================================================================
// Imported laser scan / offline XYZ triangle-strip preview
// ============================================================================

void PointCloudWidget::invalidateImportedMesh()
{
    ++m_meshGeneration;
    if (m_meshCancel) m_meshCancel->store(true);
    m_meshCancel.reset();
    m_meshWatcher = nullptr; // old watcher finishes + deleteLater; no GUI access
    m_meshReady = false;
    m_meshAttempted = false;
    m_meshSummary.clear();
    if (m_importMeshPolyData) {
        m_importMeshPolyData->SetStrips(static_cast<vtkCellArray*>(nullptr));
        m_importMeshPolyData->Modified();
    }
    if (m_importMeshActor) m_importMeshActor->SetVisibility(0);
}

void PointCloudWidget::leaveImportedMode()
{
    invalidateImportedMesh();
    m_importActive = false;
    m_importedPoints.clear();
    m_importLayout = ImportedCloudLayout();
    m_inferredProfileWidth = 0;
    if (m_importControls) m_importControls->setVisible(false);
    if (m_actor) m_actor->SetVisibility(1);
}

int PointCloudWidget::effectiveImportedWidth() const
{
    if (m_importLayout.forcePointOnly) return 0;
    if (m_importLayout.hasOriginalIndices)
        return m_importLayout.pointsPerProfile;
    const int selected = m_importProfileWidth->currentData().toInt();
    return selected > 0 ? selected : m_inferredProfileWidth;
}

void PointCloudWidget::setImportedPoints(const QVector<QVector3D>& points,
                                         const ImportedCloudLayout& layout)
{
    // setPoints() is deliberately still the original immediate point path.
    setPoints(points);
    if (points.isEmpty()) return;
    m_importedPoints = points; // Qt implicit sharing; worker holds an immutable snapshot
    m_importLayout = layout;
    m_importActive = true;
    m_inferredProfileWidth = layout.forcePointOnly ? 0 :
        layout.hasOriginalIndices ? layout.pointsPerProfile :
        mls::geometry::guessFlatProfileWidth(std::size_t(points.size()),
            [&points](std::size_t i) {
                const auto& p = points.at(qsizetype(i));
                return mls::geometry::Point3{p.x(),p.y(),p.z()};
            });
    {
        const QSignalBlocker blockMode(m_importDisplayMode);
        const QSignalBlocker blockWidth(m_importProfileWidth);
        const QSignalBlocker blockEdge(m_importEdgeLimit);
        const QSignalBlocker blockColor(m_importColorMode);
        m_importProfileWidth->setCurrentIndex(0);
        m_importDisplayMode->setCurrentIndex(effectiveImportedWidth() > 0 ? 1 : 0);
        m_importProfileWidth->setEnabled(!layout.hasOriginalIndices && !layout.forcePointOnly);
        m_importDisplayMode->setEnabled(!layout.forcePointOnly);
        m_importEdgeLimit->setValue(0.0);
        m_importColorMode->setCurrentIndex(0);
    }
    m_importControls->setVisible(true);
    setColorMode(ColorMode::ByHeight);
    if (layout.forcePointOnly) {
        m_importStatus->setText(tr("SCAN原始轮廓索引不可用，仅可显示散点"));
    }
    if (layout.hasOriginalIndices)
        m_importProfileWidth->setToolTip(tr("SCAN原始采样索引已保留，轮廓宽度为%1；不能用猜测的行宽覆盖。").arg(layout.pointsPerProfile));
    else if (!layout.forcePointOnly)
        m_importProfileWidth->setToolTip(tr("PLY/CSV没有原始轮廓信息。请保留逐行扫描顺序和全部点；自动识别失败时可以手动选择640/1280，但无法恢复丢失的原始索引。"));
    refreshImportedRepresentation();
}

void PointCloudWidget::refreshImportedRepresentation()
{
    if (!m_importActive) return;
    const int mode = m_importDisplayMode->currentIndex();
    m_importMeshActor->GetProperty()->SetEdgeVisibility(mode==2 ? 1 : 0);
    if (mode == 0) {
        m_actor->SetVisibility(1);
        m_importMeshActor->SetVisibility(0);
        if (!m_importLayout.forcePointOnly)
            m_importStatus->setText(tr("散点 · %1点").arg(m_importedPoints.size()));
    } else if (m_meshReady) {
        m_actor->SetVisibility(0);
        m_importMeshActor->SetVisibility(1);
        m_importStatus->setText(m_meshSummary);
    } else {
        // Keep showing source points while the separate offline mesh is built.
        m_actor->SetVisibility(1);
        m_importMeshActor->SetVisibility(0);
        if (!m_meshWatcher && !m_meshAttempted) startImportedMeshBuild();
    }
    if (m_renderWindow) m_renderWindow->Render();
}

void PointCloudWidget::startImportedMeshBuild()
{
    if (!m_importActive || m_meshWatcher || m_meshReady || m_meshAttempted) return;
    m_meshAttempted = true;
    const int width = effectiveImportedWidth();
    if (width<2) {
        m_importStatus->setText(tr("XYZ无可靠轮廓索引：仅散点；可选择640/1280"));
        return;
    }
    const bool indexed=m_importLayout.hasOriginalIndices;
    if ((!indexed && (m_importedPoints.size()%width!=0 || m_importedPoints.size()<width*2)) ||
        (indexed && (m_importLayout.rowOffsets.size()<3 ||
                     m_importLayout.rowOffsets.constLast()!=m_importedPoints.size() ||
                     m_importLayout.originalColumns.size()!=m_importedPoints.size()))) {
        m_importStatus->setText(tr("行宽/索引不匹配：保留散点，未生成网格"));
        return;
    }
    m_importStatus->setText(tr("正在后台生成Triangle Strip…"));
    const std::uint64_t generation=++m_meshGeneration;
    auto cancel=std::make_shared<std::atomic_bool>(false);
    m_meshCancel=cancel;
    auto* watcher=new QFutureWatcher<mls::geometry::StripMeshResult>(this);
    m_meshWatcher=watcher;
    connect(watcher, &QFutureWatcher<mls::geometry::StripMeshResult>::finished,
            this, [this, watcher, generation] {
        if (m_meshWatcher != watcher || generation != m_meshGeneration || !m_importActive) {
            watcher->deleteLater();
            return;
        }
        m_meshWatcher=nullptr;
        try {
        auto mesh=watcher->future().takeResult();
        watcher->deleteLater();
        if (mesh.cancelled) return;
        if (mesh.invalidLayout || mesh.triangles==0) {
            m_importStatus->setText(mesh.invalidLayout ?
                tr("轮廓索引错误/乱序：未连接，仍可看散点") :
                tr("没有满足边长/有效性条件的三角形，仍可看散点"));
            m_actor->SetVisibility(1);
            m_importMeshActor->SetVisibility(0);
            return;
        }
        vtkNew<vtkCellArray> strips;
        const auto count=mesh.stripCount();
        strips->AllocateEstimate(vtkIdType(count),
            int(std::min<std::size_t>(8192,mesh.vertexIds.size()/count)));
        std::vector<vtkIdType> ids;
        for(std::size_t k=0;k<count;++k) {
            const auto first=mesh.offsets[k],last=mesh.offsets[k+1];
            ids.resize(last-first);
            for(std::size_t j=0;j<ids.size();++j)
                ids[j]=vtkIdType(mesh.vertexIds[first+j]);
            strips->InsertNextCell(vtkIdType(ids.size()),ids.data());
        }
        m_importMeshPolyData->SetStrips(strips);
        m_importMeshPolyData->Modified();
        m_meshReady=true;
        QString summary=tr("%1轮廓 · %2三角面 · %3条带")
            .arg(mesh.rows).arg(mesh.triangles).arg(count);
        if(mesh.reducedForPreview)
            summary+=tr(" [预览抽样 %1×%2]").arg(mesh.rowStride).arg(mesh.columnStride);
        summary+=tr(" · 断边%1").arg(mesh.rejectedConnections+mesh.rejectedSamples);
        m_meshSummary = summary;
        m_importStatus->setText(m_importDisplayMode->currentIndex() > 0 ?
            summary : tr("散点 · %1点（网格已生成）").arg(m_importedPoints.size()));
        m_importStatus->setToolTip(tr("网格连接: 轮廓内阈值%1mm；相邻轮廓阈值%2mm。仅改变显示；原始点完整保留。")
            .arg(mesh.profileEdgeLimitMm,0,'f',3).arg(mesh.betweenEdgeLimitMm,0,'f',3));
        if (m_importDisplayMode->currentIndex()>0) {
            m_actor->SetVisibility(0);
            m_importMeshActor->SetVisibility(1);
        }
        if (m_renderWindow) m_renderWindow->Render();
        } catch (const std::exception& e) {
            watcher->deleteLater();
            m_importStatus->setText(tr("网格生成失败（仍可看散点）：%1").arg(QString::fromUtf8(e.what())));
            m_actor->SetVisibility(1);
            m_importMeshActor->SetVisibility(0);
        } catch (...) {
            watcher->deleteLater();
            m_importStatus->setText(tr("网格生成失败：未知异常，仍可看散点"));
            m_actor->SetVisibility(1);
            m_importMeshActor->SetVisibility(0);
        }
    });

    // NEVER access a VTK object or any QWidget inside QtConcurrent. Only
    // immutable copied Qt containers and the standalone C++ geometry helper.
    const auto points=m_importedPoints;
    const auto offsets=m_importLayout.rowOffsets;
    const auto columns=m_importLayout.originalColumns;
    const double maxEdge=m_importEdgeLimit->value();
    watcher->setFuture(QtConcurrent::run([points,offsets,columns,indexed,width,maxEdge,cancel]() {
        const auto count=std::size_t(points.size());
        const auto rows=indexed ? std::size_t(offsets.size()-1) : count/std::size_t(width);
        auto p=[&](std::size_t i) {
            const auto& v=points.at(qsizetype(i));
            return mls::geometry::Point3{v.x(),v.y(),v.z()};
        };
        auto row=[&](std::size_t i)->std::size_t {
            return indexed?std::size_t(offsets.at(qsizetype(i))):i*std::size_t(width);
        };
        auto col=[&](std::size_t i)->std::uint32_t {
            return indexed?std::uint32_t(columns.at(qsizetype(i))):
                std::uint32_t(i%std::size_t(width));
        };
        mls::geometry::StripMeshOptions options;
        options.maximumEdgeMm=maxEdge;
        return mls::geometry::buildTriangleStrips(count,rows,std::size_t(width),p,row,col,
            options,[&]{return cancel->load();});
    }));
}

bool PointCloudWidget::loadMachineModel(const QString& machineConfigPath, QString* error)
{
    try {
        const MachineConfig cfg = loadMachineConfig(std::filesystem::path(machineConfigPath.toStdWString()));
        for (auto& item : m_machineActors) {
            if (item.actor) {
                m_renderer->RemoveActor(item.actor);
                item.actor->Delete();
            }
        }
        m_machineActors.clear();
        for (auto& item : m_machineCollisionActors) {
            if (item.actor) {
                m_renderer->RemoveActor(item.actor);
                item.actor->Delete();
            }
        }
        m_machineCollisionActors.clear();

        for (const auto& link : cfg.links) {
            if (!std::filesystem::exists(link.visualPath))
                throw std::runtime_error("Visual PLY not found: " + link.visualPath.string());

            vtkNew<vtkPLYReader> reader;
            reader->SetFileName(link.visualPath.string().c_str());
            reader->Update();

            vtkNew<vtkPolyDataNormals> normals;
            normals->SetInputConnection(reader->GetOutputPort());
            normals->ConsistencyOn();
            normals->SplittingOff();
            normals->Update();

            vtkNew<vtkPolyDataMapper> mapper;
            mapper->SetInputConnection(normals->GetOutputPort());
            mapper->ScalarVisibilityOff();

            vtkActor* actor = vtkActor::New();
            actor->SetMapper(mapper);
            const QString qname = QString::fromStdString(link.name);
            const auto color = machineLinkColor(qname);
            actor->GetProperty()->SetColor(color[0], color[1], color[2]);
            actor->GetProperty()->SetAmbient(0.25);
            actor->GetProperty()->SetDiffuse(0.75);
            actor->GetProperty()->SetSpecular(0.08);
            actor->GetProperty()->SetOpacity(0.72);
            actor->SetVisibility(m_machineModelVisible ? 1 : 0);
            m_renderer->AddActor(actor);

            MachineVisualActor item;
            item.name = qname;
            item.actor = actor;
            item.baseColor = color;
            m_machineActors.push_back(item);

            // Render the exact STL used by FCL as an optional wireframe overlay.
            // This makes the safety geometry auditable in the same MCS as the point cloud.
            if (!std::filesystem::exists(link.collisionPath))
                throw std::runtime_error("Collision STL not found: " + link.collisionPath.string());
            auto collisionPoly = loadMachineCollisionPolyData(link.collisionPath);
            vtkNew<vtkPolyDataMapper> collisionMapper;
            collisionMapper->SetInputData(collisionPoly);
            collisionMapper->ScalarVisibilityOff();
            vtkActor* collisionActor = vtkActor::New();
            collisionActor->SetMapper(collisionMapper);
            collisionActor->GetProperty()->SetRepresentationToWireframe();
            collisionActor->GetProperty()->SetColor(0.15, 0.95, 0.25);
            collisionActor->GetProperty()->SetOpacity(0.38);
            collisionActor->GetProperty()->SetLineWidth(1.0);
            collisionActor->SetVisibility(m_machineCollisionModelVisible ? 1 : 0);
            m_renderer->AddActor(collisionActor);
            m_machineCollisionActors.push_back(MachineCollisionActor{qname, collisionActor});
        }

        m_machineModelLoaded = !m_machineActors.empty();
        updateMachineActorTransforms();
        applyMachineActorStyles();
        if (m_machineSafetyLabel) {
            m_machineSafetyLabel->setText(tr("机床模型：已加载 %1 个部件 / PLY+FCL-STL同源 / MPOS实时驱动").arg(m_machineActors.size()));
            m_machineSafetyLabel->setStyleSheet(QStringLiteral("QLabel { color:#286b3c; }"));
        }
        if (m_machineModelLoaded && m_renderer) {
            // First load should frame the real machine instead of preserving the
            // pre-model point-cloud-only camera.  Subsequent MPOS updates never
            // reset the camera.
            m_renderer->ResetCamera();
            m_initialCameraSet = true;
        }
        if (m_renderWindow) m_renderWindow->Render();
        return m_machineModelLoaded;
    } catch (const std::exception& e) {
        m_machineModelLoaded = false;
        const QString msg = QString::fromLocal8Bit(e.what());
        if (error) *error = msg;
        if (m_machineSafetyLabel) {
            m_machineSafetyLabel->setText(tr("机床模型加载失败：%1").arg(msg));
            m_machineSafetyLabel->setStyleSheet(QStringLiteral("QLabel { color:#a03030; }"));
        }
        return false;
    }
}

void PointCloudWidget::setMachineModelVisible(bool visible)
{
    m_machineModelVisible = visible;
    for (auto& item : m_machineActors)
        if (item.actor) item.actor->SetVisibility(visible ? 1 : 0);
    if (m_machineVisibleCheck && m_machineVisibleCheck->isChecked() != visible)
        m_machineVisibleCheck->setChecked(visible);
    if (m_renderWindow) m_renderWindow->Render();
}

void PointCloudWidget::setMachineCollisionModelVisible(bool visible)
{
    m_machineCollisionModelVisible = visible;
    for (auto& item : m_machineCollisionActors)
        if (item.actor) item.actor->SetVisibility(visible ? 1 : 0);
    if (m_machineCollisionVisibleCheck && m_machineCollisionVisibleCheck->isChecked() != visible)
        m_machineCollisionVisibleCheck->setChecked(visible);
    if (m_renderWindow) m_renderWindow->Render();
}

void PointCloudWidget::setMachineJointPose(double xMm, double yMm, double zMm, double cDeg, double bDeg)
{
    m_machineX = xMm; m_machineY = yMm; m_machineZ = zMm;
    m_machineC = cDeg; m_machineB = bDeg;
    if(!m_multiPosePlaybackActive) updateMachineActorTransforms();
    if (m_renderWindow && m_machineModelVisible && !m_multiPosePlaybackActive) m_renderWindow->Render();
}

void PointCloudWidget::updateMachineActorTransforms()
{
    if (!m_machineModelLoaded) return;
    MachineState q;
    q.x_mm = m_machineX; q.y_mm = m_machineY; q.z_mm = m_machineZ;
    q.c_deg = m_machineC; q.b_deg = m_machineB;
    for (auto& item : m_machineActors) {
        if (!item.actor) continue;
        vtkNew<vtkMatrix4x4> M;
        eigenToVtkMatrix(linkDelta(item.name.toStdString(), q), M);
        item.actor->SetUserMatrix(M);
    }
    for (auto& item : m_machineCollisionActors) {
        if (!item.actor) continue;
        vtkNew<vtkMatrix4x4> M;
        eigenToVtkMatrix(linkDelta(item.name.toStdString(), q), M);
        item.actor->SetUserMatrix(M);
    }
}


// V28 M1: isolated CAD actor. Geometry stays in CAD-local coordinates and is
// placed into MCS by a single rigid transform; no machine kinematics applied.
// Persistent M2 point actor: append only the next bounded preview range.
// It never replaces the regular live-acquisition cloud, and never calls devices.
void PointCloudWidget::appendTwinReplayPoints(const QVector<QVector3D>& preview,
                                              qsizetype begin, qsizetype end)
{
    if (!m_renderer) return;
    begin=std::max<qsizetype>(0,begin);
    end=std::min<qsizetype>(end,preview.size());
    if(end<=begin)return;
    if(!m_twinReplayActor){
        m_twinReplayPoints=vtkPoints::New();
        m_twinReplayPoints->SetDataTypeToFloat();
        m_twinReplayCells=vtkCellArray::New();
        m_twinReplayPoly=vtkPolyData::New();
        m_twinReplayPoly->SetPoints(m_twinReplayPoints);
        m_twinReplayPoly->SetVerts(m_twinReplayCells);
        vtkNew<vtkPolyDataMapper> map;
        map->SetInputData(m_twinReplayPoly);
        map->ScalarVisibilityOff();
        m_twinReplayActor=vtkActor::New();
        m_twinReplayActor->SetMapper(map);
        m_twinReplayActor->GetProperty()->SetColor(0.10,0.95,0.77);
        m_twinReplayActor->GetProperty()->SetPointSize(2.2);
        m_renderer->AddActor(m_twinReplayActor);
        refreshTwinOverlayVisibility();
    }
    for(qsizetype i=begin;i<end;++i){
        const auto& q=preview[i];
        const vtkIdType id=m_twinReplayPoints->InsertNextPoint(q.x(),q.y(),q.z());
        m_twinReplayCells->InsertNextCell(1);
        m_twinReplayCells->InsertCellPoint(id);
    }
    m_twinReplayPoints->Modified();
    m_twinReplayCells->Modified();
    m_twinReplayPoly->Modified();
    if(m_renderWindow)m_renderWindow->Render();
}

void PointCloudWidget::clearTwinReplayPoints()
{
    if(m_twinReplayActor){
        if(m_renderer)m_renderer->RemoveActor(m_twinReplayActor);
        m_twinReplayActor->Delete();m_twinReplayActor=nullptr;
    }
    if(m_twinReplayPoly){m_twinReplayPoly->Delete();m_twinReplayPoly=nullptr;}
    if(m_twinReplayPoints){m_twinReplayPoints->Delete();m_twinReplayPoints=nullptr;}
    if(m_twinReplayCells){m_twinReplayCells->Delete();m_twinReplayCells=nullptr;}
    if(m_renderWindow)m_renderWindow->Render();
}

// M3 virtual profiles are rendered in a separate layer from M2 measured data.
void PointCloudWidget::setTwinVirtualPoints(const QVector<QVector3D>& preview) {
    clearTwinVirtualPoints();
    if(!m_renderer || preview.isEmpty())return;
    m_twinVirtualActor=makePointCloudActor(preview,1.0,0.31,0.60,2.3,0.98);
    if(m_twinVirtualActor)m_renderer->AddActor(m_twinVirtualActor);
    refreshTwinOverlayVisibility();
    if(m_renderWindow)m_renderWindow->Render();
}
void PointCloudWidget::clearTwinVirtualPoints() {
    if(m_twinVirtualActor){
        if(m_renderer)m_renderer->RemoveActor(m_twinVirtualActor);
        m_twinVirtualActor->Delete();m_twinVirtualActor=nullptr;
    }
    if(m_renderWindow)m_renderWindow->Render();
}

void PointCloudWidget::setTwinCoveragePoints(const QVector<QVector3D>& realObserved,
                                               const QVector<QVector3D>& realMissing,
                                               const QVector<QVector3D>& virtualOnly) {
    clearTwinCoveragePoints();
    if(!m_renderer)return;
    // Real observed = cyan, real missing = red, virtual-only = gold.
    // No surface points are inserted into the live acquisition cloud.
    if(!realObserved.isEmpty()){
        m_twinM5ObservedActor=makePointCloudActor(realObserved,.11,.91,.94,2.7,.85);
        if(m_twinM5ObservedActor)m_renderer->AddActor(m_twinM5ObservedActor);
    }
    if(!realMissing.isEmpty()){
        m_twinM5MissingActor=makePointCloudActor(realMissing,.98,.12,.12,3.0,.93);
        if(m_twinM5MissingActor)m_renderer->AddActor(m_twinM5MissingActor);
    }
    if(!virtualOnly.isEmpty()){
        m_twinM5VirtualOnlyActor=makePointCloudActor(virtualOnly,1.,.79,.14,3.3,.95);
        if(m_twinM5VirtualOnlyActor)m_renderer->AddActor(m_twinM5VirtualOnlyActor);
    }
    refreshTwinOverlayVisibility();
    if(m_renderWindow)m_renderWindow->Render();
}
void PointCloudWidget::clearTwinCoveragePoints() {
    auto clearOne=[this](vtkActor*& actor){
        if(!actor)return;
        if(m_renderer)m_renderer->RemoveActor(actor);
        actor->Delete();actor=nullptr;
    };
    clearOne(m_twinM5ObservedActor);
    clearOne(m_twinM5MissingActor);
    clearOne(m_twinM5VirtualOnlyActor);
    if(m_renderWindow)m_renderWindow->Render();
}

void PointCloudWidget::focusTwinWorkpiece()
{
    if(!m_twinWorkpieceActor||!m_renderer)return;
    double bounds[6];m_twinWorkpieceActor->GetBounds(bounds);
    if(!std::isfinite(bounds[0])||!std::isfinite(bounds[1]))return;
    const double dx=bounds[1]-bounds[0],dy=bounds[3]-bounds[2],dz=bounds[5]-bounds[4];
    Q_UNUSED(dx);Q_UNUSED(dy);Q_UNUSED(dz);
    focusTwinRegion(QVector3D(float(bounds[0]),float(bounds[2]),float(bounds[4])),
                    QVector3D(float(bounds[1]),float(bounds[3]),float(bounds[5])));
}

void PointCloudWidget::focusTwinRegion(const QVector3D& a,const QVector3D& b)
{
    if(!m_renderer)return;
    const double span=std::max({double(b.x()-a.x()),double(b.y()-a.y()),double(b.z()-a.z()),10.0});
    const double cx=(double(a.x())+b.x())*.5,cy=(double(a.y())+b.y())*.5,cz=(double(a.z())+b.z())*.5;
    if(!std::isfinite(span)||!std::isfinite(cx)||!std::isfinite(cy)||!std::isfinite(cz))return;
    auto* camera=m_renderer->GetActiveCamera();
    camera->SetFocalPoint(cx,cy,cz);
    camera->SetPosition(cx+1.45*span,cy-1.65*span,cz+1.55*span);
    camera->SetViewUp(0,0,1);
    m_renderer->ResetCameraClippingRange();
    if(m_renderWindow)m_renderWindow->Render();
}

void PointCloudWidget::setTwinWorkpieceMesh(const mls::twin::TriangleMesh& mesh,
                                            const mls::twin::Transform& placement)
{
    clearTwinWorkpiece();
    if(!m_renderer || mesh.empty()) return;
    vtkNew<vtkPoints> pts;
    pts->SetDataTypeToDouble();
    vtkNew<vtkCellArray> tris;
    for(const auto& t: mesh.triangles) {
        vtkIdType ids[3];
        ids[0]=pts->InsertNextPoint(t.a.x,t.a.y,t.a.z);
        ids[1]=pts->InsertNextPoint(t.b.x,t.b.y,t.b.z);
        ids[2]=pts->InsertNextPoint(t.c.x,t.c.y,t.c.z);
        tris->InsertNextCell(3,ids);
    }
    vtkNew<vtkPolyData> poly;
    poly->SetPoints(pts);
    poly->SetPolys(tris);
    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputData(poly);
    mapper->ScalarVisibilityOff();
    m_twinWorkpieceActor=vtkActor::New();
    m_twinWorkpieceActor->SetMapper(mapper);
    m_twinWorkpieceActor->GetProperty()->SetColor(0.96,0.58,0.20);
    m_twinWorkpieceActor->GetProperty()->SetOpacity(0.92);
    m_twinWorkpieceActor->GetProperty()->SetAmbient(0.22);
    m_twinWorkpieceActor->GetProperty()->SetDiffuse(0.70);
    m_twinWorkpieceActor->GetProperty()->SetSpecular(0.12);
    m_renderer->AddActor(m_twinWorkpieceActor);
    setTwinWorkpiecePose(placement);
    refreshTwinOverlayVisibility();
}

void PointCloudWidget::setTwinWorkpiecePose(const mls::twin::Transform& pose)
{
    if(!m_twinWorkpieceActor)return;
    vtkNew<vtkMatrix4x4> M;
    for(int row=0;row<4;++row)
        for(int col=0;col<4;++col)
            M->SetElement(row,col,pose.m[static_cast<size_t>(row*4+col)]);
    m_twinWorkpieceActor->SetUserMatrix(M);
    if(m_renderWindow)m_renderWindow->Render();
}

void PointCloudWidget::clearTwinWorkpiece()
{
    if(!m_twinWorkpieceActor)return;
    if(m_renderer)m_renderer->RemoveActor(m_twinWorkpieceActor);
    m_twinWorkpieceActor->Delete();
    m_twinWorkpieceActor=nullptr;
    if(m_renderWindow)m_renderWindow->Render();
}

void PointCloudWidget::setTwinSupportPlane(bool enabled, double z,
                                           mls::twin::Vec3 center,mls::twin::Vec3 size)
{
    if(m_twinSupportActor) {
        if(m_renderer)m_renderer->RemoveActor(m_twinSupportActor);
        m_twinSupportActor->Delete();m_twinSupportActor=nullptr;
    }
    if(!enabled||!m_renderer||size.x<=0||size.y<=0) {
        if(m_renderWindow)m_renderWindow->Render();
        return;
    }
    const double a=center.x-size.x*.5,b=center.y-size.y*.5;
    vtkNew<vtkPoints> pts;
    pts->InsertNextPoint(a,b,z);
    pts->InsertNextPoint(a+size.x,b,z);
    pts->InsertNextPoint(a+size.x,b+size.y,z);
    pts->InsertNextPoint(a,b+size.y,z);
    vtkIdType ids[]={0,1,2,3};
    vtkNew<vtkCellArray> faces;
    faces->InsertNextCell(4,ids);
    vtkNew<vtkPolyData> poly;
    poly->SetPoints(pts);poly->SetPolys(faces);
    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputData(poly);mapper->ScalarVisibilityOff();
    m_twinSupportActor=vtkActor::New();m_twinSupportActor->SetMapper(mapper);
    m_twinSupportActor->GetProperty()->SetColor(.30,.59,.79);
    m_twinSupportActor->GetProperty()->SetOpacity(.18);
    m_twinSupportActor->GetProperty()->EdgeVisibilityOn();
    m_twinSupportActor->GetProperty()->SetEdgeColor(.14,.44,.61);
    m_renderer->AddActor(m_twinSupportActor);
    refreshTwinOverlayVisibility();
    if(m_renderWindow)m_renderWindow->Render();
}

void PointCloudWidget::setMachineCollisionLinks(const QStringList& links)
{
    m_machineCollisionLinks = links;
    applyMachineActorStyles();
    if (m_renderWindow && m_machineModelVisible) m_renderWindow->Render();
}

void PointCloudWidget::applyMachineActorStyles()
{
    for (auto& item : m_machineActors) {
        if (!item.actor) continue;
        const bool collided = m_machineCollisionLinks.contains(item.name);
        if (collided) {
            item.actor->GetProperty()->SetColor(1.0, 0.08, 0.04);
            item.actor->GetProperty()->SetOpacity(0.92);
        } else {
            item.actor->GetProperty()->SetColor(item.baseColor[0], item.baseColor[1], item.baseColor[2]);
            item.actor->GetProperty()->SetOpacity(0.72);
        }
    }
    for (auto& item : m_machineCollisionActors) {
        if (!item.actor) continue;
        const bool collided = m_machineCollisionLinks.contains(item.name);
        item.actor->GetProperty()->SetColor(collided ? 1.0 : 0.15, collided ? 0.08 : 0.95, collided ? 0.04 : 0.25);
        item.actor->GetProperty()->SetOpacity(collided ? 0.95 : 0.38);
        item.actor->GetProperty()->SetLineWidth(collided ? 2.4 : 1.0);
    }
    if (m_machineStudySphereActor) {
        const bool hitSphere = m_machineCollisionLinks.contains(QStringLiteral("STUDY_SPHERE"));
        m_machineStudySphereActor->GetProperty()->SetColor(hitSphere ? 1.0 : 0.25,
                                                            hitSphere ? 0.10 : 0.65,
                                                            hitSphere ? 0.05 : 0.95);
        m_machineStudySphereActor->GetProperty()->SetOpacity(hitSphere ? 0.70 : 0.30);
    }
}

void PointCloudWidget::setMachineStudySphere(double xMm, double yMm, double zMm,
                                                 double radiusMm, double safetyClearanceMm)
{
    if (!m_renderer || !(radiusMm > 0.0)) return;
    clearMachineStudySphere();

    m_machineStudySphereSource = vtkSphereSource::New();
    m_machineStudySphereSource->SetCenter(xMm, yMm, zMm);
    m_machineStudySphereSource->SetRadius(radiusMm);
    m_machineStudySphereSource->SetThetaResolution(32);
    m_machineStudySphereSource->SetPhiResolution(20);
    vtkNew<vtkPolyDataMapper> sphereMapper;
    sphereMapper->SetInputConnection(m_machineStudySphereSource->GetOutputPort());
    m_machineStudySphereActor = vtkActor::New();
    m_machineStudySphereActor->SetMapper(sphereMapper);
    m_machineStudySphereActor->GetProperty()->SetColor(0.25, 0.65, 0.95);
    m_machineStudySphereActor->GetProperty()->SetOpacity(0.30);
    m_renderer->AddActor(m_machineStudySphereActor);

    const double inflated = radiusMm + std::max(0.0, safetyClearanceMm);
    if (inflated > radiusMm + 1e-9) {
        m_machineStudySafetySphereSource = vtkSphereSource::New();
        m_machineStudySafetySphereSource->SetCenter(xMm, yMm, zMm);
        m_machineStudySafetySphereSource->SetRadius(inflated);
        m_machineStudySafetySphereSource->SetThetaResolution(32);
        m_machineStudySafetySphereSource->SetPhiResolution(20);
        vtkNew<vtkPolyDataMapper> safetyMapper;
        safetyMapper->SetInputConnection(m_machineStudySafetySphereSource->GetOutputPort());
        m_machineStudySafetySphereActor = vtkActor::New();
        m_machineStudySafetySphereActor->SetMapper(safetyMapper);
        m_machineStudySafetySphereActor->GetProperty()->SetRepresentationToWireframe();
        m_machineStudySafetySphereActor->GetProperty()->SetColor(1.0, 0.55, 0.05);
        m_machineStudySafetySphereActor->GetProperty()->SetOpacity(0.65);
        m_machineStudySafetySphereActor->GetProperty()->SetLineWidth(1.5);
        m_renderer->AddActor(m_machineStudySafetySphereActor);
    }
    if (m_renderWindow) m_renderWindow->Render();
}

void PointCloudWidget::clearMachineStudySphere()
{
    if (m_machineStudySphereActor) {
        if (m_renderer) m_renderer->RemoveActor(m_machineStudySphereActor);
        m_machineStudySphereActor->Delete(); m_machineStudySphereActor = nullptr;
    }
    if (m_machineStudySafetySphereActor) {
        if (m_renderer) m_renderer->RemoveActor(m_machineStudySafetySphereActor);
        m_machineStudySafetySphereActor->Delete(); m_machineStudySafetySphereActor = nullptr;
    }
    if (m_machineStudySphereSource) { m_machineStudySphereSource->Delete(); m_machineStudySphereSource = nullptr; }
    if (m_machineStudySafetySphereSource) { m_machineStudySafetySphereSource->Delete(); m_machineStudySafetySphereSource = nullptr; }
    if (m_renderWindow) m_renderWindow->Render();
}

void PointCloudWidget::setSphereStudyScanSimulation(
    const QVector<QVector3D>& laserFieldCornersMachineMm,
    const QVector<QVector3D>& sectionCircleMachineMm,
    const QVector<QVector3D>& visibleArcMachineMm,
    bool feasible,
    const QString& statusText)
{
    clearSphereStudyScanSimulation();
    if (!m_renderer) return;

    m_sphereStudyLaserPlaneActor = makeQuadActor(laserFieldCornersMachineMm, feasible);
    if (m_sphereStudyLaserPlaneActor) m_renderer->AddActor(m_sphereStudyLaserPlaneActor);

    m_sphereStudySectionCircleActor = makePolylineActor(
        sectionCircleMachineMm, 0.95, 0.66, 0.12, 1.6, 0.78);
    if (m_sphereStudySectionCircleActor) m_renderer->AddActor(m_sphereStudySectionCircleActor);

    m_sphereStudyVisibleArcActor = makePolylineActor(
        visibleArcMachineMm,
        feasible ? 0.15 : 0.95,
        feasible ? 0.95 : 0.18,
        feasible ? 0.25 : 0.12,
        4.0, 0.98);
    if (m_sphereStudyVisibleArcActor) m_renderer->AddActor(m_sphereStudyVisibleArcActor);

    if (m_sphereStudySimulationLabel) {
        m_sphereStudySimulationLabel->setVisible(true);
        m_sphereStudySimulationLabel->setText(statusText);
        m_sphereStudySimulationLabel->setStyleSheet(feasible
            ? QStringLiteral("QLabel { color:#245f3d; padding:2px 6px; background:#eef9f2; border:1px solid #bfdfc8; font-weight:600; }")
            : QStringLiteral("QLabel { color:#9d2a24; padding:2px 6px; background:#fff0ee; border:1px solid #efc1bc; font-weight:600; }"));
    }
    if (m_renderWindow) m_renderWindow->Render();
}

void PointCloudWidget::clearSphereStudyScanSimulation()
{
    auto clearActor=[this](vtkActor*& actor){
        if (!actor) return;
        if (m_renderer) m_renderer->RemoveActor(actor);
        actor->Delete();
        actor=nullptr;
    };
    clearActor(m_sphereStudyLaserPlaneActor);
    clearActor(m_sphereStudySectionCircleActor);
    clearActor(m_sphereStudyVisibleArcActor);
    if (m_sphereStudySimulationLabel) {
        m_sphereStudySimulationLabel->setVisible(false);
        m_sphereStudySimulationLabel->setText(tr("扫描仿真：未启用"));
    }
    if (m_renderWindow) m_renderWindow->Render();
}

void PointCloudWidget::setSphereStudySimulatedAcquisitionCloud(
    const QVector<QVector3D>& pointsMachineMm)
{
    clearSphereStudySimulatedAcquisitionCloud();
    if (!m_renderer || pointsMachineMm.isEmpty()) return;

    // Keep the synthetic acquisition visually distinct from both the imported
    // source cloud (height-colored) and the transient current end-effector-side visible arc.
    m_sphereStudySimulatedCloudActor = makePointCloudActor(
        pointsMachineMm, 1.0, 0.32, 0.92, 4.0, 0.96);
    if (m_sphereStudySimulatedCloudActor)
        m_renderer->AddActor(m_sphereStudySimulatedCloudActor);
    if (m_renderWindow) m_renderWindow->Render();
}

void PointCloudWidget::clearSphereStudySimulatedAcquisitionCloud()
{
    if (!m_sphereStudySimulatedCloudActor) return;
    if (m_renderer) m_renderer->RemoveActor(m_sphereStudySimulatedCloudActor);
    m_sphereStudySimulatedCloudActor->Delete();
    m_sphereStudySimulatedCloudActor = nullptr;
    if (m_renderWindow) m_renderWindow->Render();
}

void PointCloudWidget::setSphereExposureSweepPath(
    const QVector<QVector3D>& sensorTraceMachineMm)
{
    clearSphereExposureSweepPath();
    if (!m_renderer || sensorTraceMachineMm.size() < 2) return;
    // This is a VISUALIZATION of a fixed point on the laser plane, not a
    // commanded tool-tip trajectory or a collision-safety proof.
    m_sphereExposureSweepPathActor = makePolylineActor(
        sensorTraceMachineMm, 0.08, 0.92, 0.94, 5.0, 0.98);
    m_sphereExposureStartActor = makePointCloudActor(
        QVector<QVector3D>{sensorTraceMachineMm.front()}, 0.10, 0.90, 0.25, 13.0, 1.0);
    m_sphereExposureEndActor = makePointCloudActor(
        QVector<QVector3D>{sensorTraceMachineMm.back()}, 1.0, 0.25, 0.20, 13.0, 1.0);
    if (m_sphereExposureSweepPathActor) m_renderer->AddActor(m_sphereExposureSweepPathActor);
    if (m_sphereExposureStartActor) m_renderer->AddActor(m_sphereExposureStartActor);
    if (m_sphereExposureEndActor) m_renderer->AddActor(m_sphereExposureEndActor);
    if (m_renderWindow) m_renderWindow->Render();
}

void PointCloudWidget::clearSphereExposureSweepPath()
{
    auto remove=[this](vtkActor*& actor){
        if (!actor) return;
        if (m_renderer) m_renderer->RemoveActor(actor);
        actor->Delete();
        actor=nullptr;
    };
    remove(m_sphereExposureSweepPathActor);
    remove(m_sphereExposureStartActor);
    remove(m_sphereExposureEndActor);
    if (m_renderWindow) m_renderWindow->Render();
}

void PointCloudWidget::setMachineSafetyStatus(const QString& text, bool collision)
{
    if (!m_machineSafetyLabel) return;
    m_machineSafetyLabel->setText(text);
    m_machineSafetyLabel->setStyleSheet(collision
        ? QStringLiteral("QLabel { color:#b02020; font-weight:600; }")
        : QStringLiteral("QLabel { color:#286b3c; }"));
}

void PointCloudWidget::setTemporalCorrectionStatus(bool loaded, bool enabled, double delayUs,
                                                   const QString& calibrationId,
                                                   bool hardwareSyncSelected)
{
    if (!m_temporalCorrectionLabel) return;
    if (!loaded) {
        m_temporalCorrectionLabel->setText(tr("Temporal Runtime: OFF / 未加载"));
        m_temporalCorrectionLabel->setStyleSheet(QStringLiteral(
            "QLabel { color:#777; font-weight:600; padding-left:8px; }"));
        return;
    }
    if (!hardwareSyncSelected) {
        m_temporalCorrectionLabel->setText(tr("Temporal Runtime: N/A / 软件同步  τ=%1 µs loaded").arg(delayUs,0,'f',2));
        m_temporalCorrectionLabel->setStyleSheet(QStringLiteral(
            "QLabel { color:#315a72; font-weight:700; padding-left:8px; }"));
    } else if (enabled) {
        m_temporalCorrectionLabel->setText(tr("Temporal Runtime: ON  τ=%1 µs").arg(delayUs,0,'f',2));
        m_temporalCorrectionLabel->setStyleSheet(QStringLiteral(
            "QLabel { color:#138a36; font-weight:700; padding-left:8px; }"));
    } else {
        m_temporalCorrectionLabel->setText(tr("Temporal Runtime: OFF / RAW  τ=%1 µs loaded").arg(delayUs,0,'f',2));
        m_temporalCorrectionLabel->setStyleSheet(QStringLiteral(
            "QLabel { color:#a06000; font-weight:700; padding-left:8px; }"));
    }
    if (!calibrationId.isEmpty())
        m_temporalCorrectionLabel->setToolTip(tr("活动硬件时域标定：%1\nτ=%2 µs\n%3")
            .arg(calibrationId).arg(delayUs,0,'f',2)
            .arg(!hardwareSyncSelected?tr("当前选择软件同步，本时域补偿不参与本次三维重建。")
                 : enabled?tr("三维点云将使用q(t+τ)重建。")
                          :tr("三维点云将使用RAW同ordinal REGIST(104)位姿重建。")));
}

void PointCloudWidget::onRenderTimer()
{
    if (!m_needsUpdate.load()) return;
    m_needsUpdate.store(false);
    
    // 请求后台线程准备数据
    if (m_prepareThread) {
        m_prepareThread->requestUpdate();
    }
}

void PointCloudWidget::onDataReady(const QVector<QVector3D>& points, float zMin, float zMax)
{
    // The worker's pending live preview must NEVER replace imported XYZ.
    if (m_importActive) return;
    uploadPointsToVTK(points, zMin, zMax);
}

void PointCloudWidget::uploadPointsToVTK(const QVector<QVector3D>& points, float zMin, float zMax)
{
    if (points.isEmpty()) {
        m_vtkPoints->Reset();
        m_polyData->GetPointData()->Reset();
        m_polyData->Modified();
        if (m_renderWindow) {
            m_renderWindow->Render();
        }
        return;
    }
    
    // 更新 VTK 数据
    const int numPoints = points.size();
    m_vtkPoints->SetNumberOfPoints(numPoints);
    
    vtkNew<vtkFloatArray> scalars;
    scalars->SetName("Height");
    scalars->SetNumberOfTuples(numPoints);
    
    for (int i = 0; i < numPoints; ++i) {
        const auto& p = points[i];
        m_vtkPoints->SetPoint(i, p.x(), p.y(), p.z());
        scalars->SetValue(i, p.z());
    }
    
    m_polyData->GetPointData()->SetScalars(scalars);
    
    // 更新颜色范围
    if (zMax - zMin < 0.001f) {
        zMin -= 0.5f;
        zMax += 0.5f;
    }
    m_lut->SetTableRange(zMin, zMax);
    m_mapper->SetScalarRange(zMin, zMax);
    m_importMeshMapper->SetScalarRange(zMin, zMax);
    m_importMeshPolyData->GetPointData()->SetScalars(scalars);
    
    m_vtkPoints->Modified();
    m_polyData->Modified();
    
    updateColorMapping();
    
    if (m_renderWindow) {
        m_renderWindow->Render();
    }
}

void PointCloudWidget::updateColorMapping()
{
    switch (m_colorMode) {
    case ColorMode::Uniform:
        m_importMeshMapper->SetScalarVisibility(false);
        m_importMeshActor->GetProperty()->SetColor(
            m_uniformColor.redF(), m_uniformColor.greenF(), m_uniformColor.blueF());
        m_mapper->SetScalarVisibility(false);
        m_actor->GetProperty()->SetColor(
            m_uniformColor.redF(),
            m_uniformColor.greenF(),
            m_uniformColor.blueF());
        m_scalarBar->SetVisibility(false);
        break;
        
    case ColorMode::ByHeight:
        m_importMeshMapper->SetScalarVisibility(true);
        m_importMeshMapper->SetScalarModeToUsePointData();
        m_importMeshMapper->SelectColorArray("Height");
        m_mapper->SetScalarVisibility(true);
        m_mapper->SetScalarModeToUsePointData();
        m_mapper->SelectColorArray("Height");
        m_scalarBar->SetVisibility(m_scalarBarVisible ? 1 : 0);
        break;
    }
}

void PointCloudWidget::setColorMode(ColorMode mode)
{
    m_colorMode = mode;
    if (m_importActive && m_importColorMode) {
        const QSignalBlocker guard(m_importColorMode);
        m_importColorMode->setCurrentIndex(mode == ColorMode::ByHeight ? 0 : 1);
    }
    updateColorMapping();
    if (m_renderWindow) {
        m_renderWindow->Render();
    }
}

void PointCloudWidget::setUniformColor(const QColor& color)
{
    m_uniformColor = color;
    if (m_colorMode == ColorMode::Uniform) {
        m_actor->GetProperty()->SetColor(color.redF(), color.greenF(), color.blueF());
        m_importMeshActor->GetProperty()->SetColor(color.redF(), color.greenF(), color.blueF());
        if (m_renderWindow) {
            m_renderWindow->Render();
        }
    }
}

void PointCloudWidget::setPointSize(float size)
{
    m_pointSize = size;
    m_actor->GetProperty()->SetPointSize(size);
    if (m_renderWindow) {
        m_renderWindow->Render();
    }
}

void PointCloudWidget::setAxesVisible(bool visible)
{
    m_axesVisible = visible;
    if (m_axesWidget) {
        m_axesWidget->SetEnabled(visible ? 1 : 0);
        if (visible) m_axesWidget->InteractiveOff();
        if (m_vtkWidget) m_vtkWidget->setCursor(Qt::ArrowCursor);
    }
    if (m_renderWindow) {
        m_renderWindow->Render();
    }
}

void PointCloudWidget::setScalarBarVisible(bool visible)
{
    m_scalarBarVisible = visible;
    if (m_colorMode == ColorMode::ByHeight && m_scalarBar) {
        m_scalarBar->SetVisibility(visible ? 1 : 0);
    }
    if (m_renderWindow) {
        m_renderWindow->Render();
    }
}

void PointCloudWidget::resetCamera()
{
    m_renderer->ResetCamera();
    if (m_renderWindow) {
        m_renderWindow->Render();
    }
}

void PointCloudWidget::setTopView()
{
    vtkCamera* camera = m_renderer->GetActiveCamera();
    
    // 计算工作区域中心
    double centerX = m_workXRange / 2.0;
    double centerY = m_workYRange / 2.0;
    double centerZ = m_workZRange / 2.0 + (m_sensorZMin + m_sensorZMax) / 2.0;
    double maxRange = qMax(m_workXRange, m_workYRange);
    double distance = maxRange * 1.5;
    
    camera->SetPosition(centerX, centerY, centerZ + distance);
    camera->SetFocalPoint(centerX, centerY, centerZ);
    camera->SetViewUp(0, 1, 0);
    camera->SetClippingRange(1.0, distance * 5.0);
    
    if (m_renderWindow) m_renderWindow->Render();
}

void PointCloudWidget::setFrontView()
{
    vtkCamera* camera = m_renderer->GetActiveCamera();
    
    // 计算工作区域中心
    double centerX = m_workXRange / 2.0;
    double centerY = m_workYRange / 2.0;
    double centerZ = m_workZRange / 2.0 + (m_sensorZMin + m_sensorZMax) / 2.0;
    double maxRange = qMax(m_workXRange, m_workZRange + m_sensorZMax);
    double distance = maxRange * 1.5;
    
    camera->SetPosition(centerX, centerY - distance, centerZ);
    camera->SetFocalPoint(centerX, centerY, centerZ);
    camera->SetViewUp(0, 0, 1);
    camera->SetClippingRange(1.0, distance * 5.0);
    
    if (m_renderWindow) m_renderWindow->Render();
}

void PointCloudWidget::setSideView()
{
    vtkCamera* camera = m_renderer->GetActiveCamera();
    
    // 计算工作区域中心
    double centerX = m_workXRange / 2.0;
    double centerY = m_workYRange / 2.0;
    double centerZ = m_workZRange / 2.0 + (m_sensorZMin + m_sensorZMax) / 2.0;
    double maxRange = qMax(m_workYRange, m_workZRange + m_sensorZMax);
    double distance = maxRange * 1.5;
    
    camera->SetPosition(centerX + distance, centerY, centerZ);
    camera->SetFocalPoint(centerX, centerY, centerZ);
    camera->SetViewUp(0, 0, 1);
    camera->SetClippingRange(1.0, distance * 5.0);
    
    if (m_renderWindow) m_renderWindow->Render();
}

void PointCloudWidget::setIsometricView()
{
    // 使用 setupInitialCamera 实现等轴测视角
    setupInitialCamera();
}

void PointCloudWidget::setWorkingVolume(double xRange, double yRange, double zRange,
                                         double sensorZMin, double sensorZMax)
{
    m_workXRange = xRange;
    m_workYRange = yRange;
    m_workZRange = zRange;
    m_sensorZMin = sensorZMin;
    m_sensorZMax = sensorZMax;
    
    // 重新设置相机
    setupInitialCamera();
}

void PointCloudWidget::setupInitialCamera()
{
    if (!m_renderer) return;
    
    vtkCamera* camera = m_renderer->GetActiveCamera();
    if (!camera) return;
    
    // =========================================================================
    // 工作区域分析（基于机床坐标系）
    // =========================================================================
    // 机床 X/Y/Z 范围: [0, m_workXRange], [0, m_workYRange], [0, m_workZRange]
    // 传感器测量的 Z 值范围: [m_sensorZMin, m_sensorZMax]
    // 
    // 点云的实际 Z 坐标 = 机床Z + 安装偏移 + 传感器Z
    // 假设安装偏移为0，则点云 Z 范围约为:
    //   最小: 0 + m_sensorZMin
    //   最大: m_workZRange + m_sensorZMax
    // =========================================================================
    
    // 计算工作区域的边界
    double xMin = 0.0, xMax = m_workXRange;
    double yMin = 0.0, yMax = m_workYRange;
    double zMin = m_sensorZMin;  // 点云最低可能高度
    double zMax = m_workZRange + m_sensorZMax;  // 点云最高可能高度
    
    // 计算中心点
    double centerX = (xMin + xMax) / 2.0;
    double centerY = (yMin + yMax) / 2.0;
    double centerZ = (zMin + zMax) / 2.0;
    
    // 计算包围盒的对角线长度，用于确定观察距离
    double dx = xMax - xMin;
    double dy = yMax - yMin;
    double dz = zMax - zMin;
    double diagonal = std::sqrt(dx*dx + dy*dy + dz*dz);
    
    // 观察距离：对角线的1.5倍，确保整个工作区域可见
    double distance = diagonal * 1.5;
    
    // =========================================================================
    // 设置等轴测视角
    // =========================================================================
    // 从右前上方观察（典型的三维视图）
    // 相机方向向量（归一化）: (0.6, -0.6, 0.5) 大约
    // =========================================================================
    
    double camX = centerX + distance * 0.5;    // 向右（X正方向）
    double camY = centerY - distance * 0.7;    // 向前（Y负方向，假设Y正为后）
    double camZ = centerZ + distance * 0.5;    // 向上（Z正方向）
    
    camera->SetPosition(camX, camY, camZ);
    camera->SetFocalPoint(centerX, centerY, centerZ);
    camera->SetViewUp(0, 0, 1);  // Z轴向上
    
    // 设置裁剪范围
    double nearClip = distance * 0.01;  // 近裁剪面
    double farClip = distance * 10.0;   // 远裁剪面
    camera->SetClippingRange(nearClip, farClip);
    
    // 设置视角（FOV），较小的角度减少透视变形
    camera->SetViewAngle(30.0);
    
    // 并行投影模式（可选，取消注释可使用正交投影）
    // camera->ParallelProjectionOn();
    // camera->SetParallelScale(diagonal / 2.0);
    
    m_initialCameraSet = true;
    
    qDebug() << "Initial camera setup:";
    qDebug() << "  Work volume: X[0," << m_workXRange << "] Y[0," << m_workYRange 
             << "] Z[0," << m_workZRange << "]";
    qDebug() << "  Sensor Z range: [" << m_sensorZMin << "," << m_sensorZMax << "]";
    qDebug() << "  Point cloud bounds: X[" << xMin << "," << xMax << "] Y[" << yMin 
             << "," << yMax << "] Z[" << zMin << "," << zMax << "]";
    qDebug() << "  Center:" << centerX << centerY << centerZ;
    qDebug() << "  Camera:" << camX << camY << camZ;
    qDebug() << "  Distance:" << distance;
    
    if (m_renderWindow) {
        m_renderWindow->Render();
    }
}

void PointCloudWidget::resizeEvent(QResizeEvent* event)
{
    QWidget::resizeEvent(event);
    if (m_renderWindow) {
        m_renderWindow->Render();
    }
}

// V28.7.4: independent CAD diagnostic layers and uniformly sampled M6 segments.
// These are read-only VTK overlays; none is a hardware-control or measurement source.
namespace {
const std::array<std::array<double,3>,12> kM6Colors{{
    {{0.91,0.74,0.19}},{{0.33,0.81,0.97}},{{0.92,0.48,0.89}},
    {{0.48,0.86,0.35}},{{0.98,0.55,0.35}},{{0.62,0.62,1.00}},
    {{0.96,0.80,0.48}},{{0.52,0.92,0.78}},{{1.00,0.55,0.72}},
    {{0.78,0.72,0.95}},{{0.98,0.60,0.23}},{{0.51,0.77,0.98}}
}};
}

bool PointCloudWidget::twinOverlayVisible(const QString& layer) const {
    return m_twinVisibility.value(layer,true);
}
void PointCloudWidget::setTwinOverlayVisible(const QString& layer,bool visible){
    m_twinVisibility.insert(layer,visible);
    if(layer==QStringLiteral("M0_MACHINE"))setMachineModelVisible(visible);
    refreshTwinOverlayVisibility();
    if(m_renderWindow)m_renderWindow->Render();
}
void PointCloudWidget::setTwinOverlayVisibilityBatch(const QHash<QString,bool>& layers){
    for(auto it=layers.constBegin();it!=layers.constEnd();++it)
        m_twinVisibility.insert(it.key(),it.value());
    const auto it=layers.constFind(QStringLiteral("M0_MACHINE"));
    if(it!=layers.constEnd()&&m_machineModelVisible!=it.value())
        setMachineModelVisible(it.value());
    refreshTwinOverlayVisibility();
    if(m_renderWindow)m_renderWindow->Render();
}
void PointCloudWidget::refreshTwinOverlayVisibility(){
    const auto show=[this](vtkActor* a,const QString& key){
        if(a)a->SetVisibility(twinOverlayVisible(key)?1:0);
    };
    show(m_twinWorkpieceActor,QStringLiteral("M1_CAD"));
    show(m_twinSupportActor,QStringLiteral("M1_SUPPORT"));
    show(m_twinReplayActor,QStringLiteral("M2_REAL"));
    show(m_twinVirtualActor,QStringLiteral("M3_VIRTUAL"));
    show(m_twinM4RoiActor,QStringLiteral("M4_ROI"));
    show(m_twinM5ObservedActor,QStringLiteral("M5_OBSERVED"));
    show(m_twinM5MissingActor,QStringLiteral("M5_MISSING"));
    show(m_twinM5VirtualOnlyActor,QStringLiteral("M5_VIRTUAL_ONLY"));
    show(m_twinM5FeatureActor,QStringLiteral("M5_FOCUS"));
    show(m_twinM51SparseActor,QStringLiteral("M51_SPARSE"));
    show(m_twinM52ResidualActor,QStringLiteral("M52_RESIDUAL"));
    show(m_twinM6Actor,QStringLiteral("M6_RAW"));
    show(m_twinM6PathActor,QStringLiteral("M6_PATH"));
    show(m_twinM6NewCoverageActor,QStringLiteral("M6_NEW_COVERAGE"));
    show(m_twinM6RemainingActor,QStringLiteral("M6_REMAINING"));
    show(m_twinM6TransferActor,QStringLiteral("M6_TRANSFER"));
    show(m_twinM6CursorActor,QStringLiteral("M6_CURSOR"));
    show(m_twinM7FusedActor,QStringLiteral("M7_FUSED"));
    show(m_twinM7RealActor,QStringLiteral("M7_REAL"));
    show(m_twinM7SimActor,QStringLiteral("M7_SIMULATED"));
    show(m_twinM7OverlapActor,QStringLiteral("M7_OVERLAP"));
    show(m_twinM7NewCadActor,QStringLiteral("M7_NEW_CAD"));
    show(m_twinM7MissingActor,QStringLiteral("M7_REMAINING_CAD"));
    show(m_twinM6StartActor,QStringLiteral("M6_ENDPOINTS"));
    show(m_twinM6EndActor,QStringLiteral("M6_ENDPOINTS"));
    for(std::size_t i=0;i<m_multiPoseCloudActors.size();++i){
        auto* actor=m_multiPoseCloudActors[i];
        if(actor)actor->SetVisibility(!m_multiPosePlaybackActive && twinOverlayVisible(QStringLiteral("MULTIPOSE_VIRTUAL"))&&
            (m_multiPoseActivePose<0||std::size_t(m_multiPoseActivePose)==i));
    }
    for(std::size_t i=0;i<m_multiPosePathActors.size();++i){
        auto* actor=m_multiPosePathActors[i];
        if(actor)actor->SetVisibility(twinOverlayVisible(QStringLiteral("MULTIPOSE_PATH"))&&
            (m_multiPoseActivePose<0||std::size_t(m_multiPoseActivePose)==i));
    }
    show(m_multiPoseTransferActor,QStringLiteral("MULTIPOSE_TRANSFER"));
    show(m_multiPoseWorkpieceActor,QStringLiteral("MULTIPOSE_CAD"));
    show(m_multiPoseRecordedActor,QStringLiteral("MULTIPOSE_REAL_REFERENCE"));
    show(m_multiPoseLaserActor,QStringLiteral("MULTIPOSE_PLAYBACK"));
    show(m_multiPoseAccumActor,QStringLiteral("MULTIPOSE_PLAYBACK"));
    show(m_multiPoseCompareActor,QStringLiteral("MULTIPOSE_COMPARISON"));
    for(std::size_t i=0;i<m_twinM6SegmentCloudActors.size();++i){
        vtkActor* a=m_twinM6SegmentCloudActors[i];
        if(a)a->SetVisibility(twinOverlayVisible(QStringLiteral("M6_RAW"))&&
            (m_twinM6ActiveSegment<0||std::size_t(m_twinM6ActiveSegment)==i));
    }
    for(std::size_t i=0;i<m_twinM6SegmentPathActors.size();++i){
        vtkActor* a=m_twinM6SegmentPathActors[i];
        if(a)a->SetVisibility(twinOverlayVisible(QStringLiteral("M6_PATH"))&&
            (m_twinM6ActiveSegment<0||std::size_t(m_twinM6ActiveSegment)==i));
    }
}
void PointCloudWidget::setTwinStageMarkers(const QString& layer,const QVector<QVector3D>& points){
    vtkActor** slot=nullptr;
    std::array<double,3> rgb{{1.,1.,1.}};
    double pointSize=4.0;
    if(layer==QStringLiteral("M4_ROI")){slot=&m_twinM4RoiActor;rgb={{0.96,0.78,0.19}};}
    else if(layer==QStringLiteral("M5_FOCUS")){slot=&m_twinM5FeatureActor;rgb={{0.95,0.95,0.15}};pointSize=5.;}
    else if(layer==QStringLiteral("M51_SPARSE")){slot=&m_twinM51SparseActor;rgb={{0.98,0.30,0.13}};pointSize=5.;}
    else if(layer==QStringLiteral("M52_RESIDUAL")){slot=&m_twinM52ResidualActor;rgb={{0.79,0.31,0.99}};pointSize=4.;}
    else if(layer==QStringLiteral("M6_NEW_COVERAGE")){slot=&m_twinM6NewCoverageActor;rgb={{0.31,0.95,0.29}};pointSize=4.;}
    else if(layer==QStringLiteral("M6_REMAINING")){slot=&m_twinM6RemainingActor;rgb={{0.96,0.12,0.17}};pointSize=4.;}
    else if(layer==QStringLiteral("M6_CURSOR")){slot=&m_twinM6CursorActor;rgb={{0.05,0.92,1.}};pointSize=11.;}
    else return;
    if(*slot){if(m_renderer)m_renderer->RemoveActor(*slot);(*slot)->Delete();*slot=nullptr;}
    if(!points.isEmpty()&&m_renderer){
        *slot=makePointCloudActor(points,rgb[0],rgb[1],rgb[2],pointSize,0.98);
        if(*slot)m_renderer->AddActor(*slot);
    }
    refreshTwinOverlayVisibility();
    if(m_renderWindow)m_renderWindow->Render();
}
void PointCloudWidget::setTwinM6SegmentSelection(int segmentIndex){
    m_twinM6ActiveSegment=segmentIndex;
    refreshTwinOverlayVisibility();
    if(m_renderWindow)m_renderWindow->Render();
}
void PointCloudWidget::setTwinM6DetailedResults(
    const QVector<QVector<QVector3D>>& segmentPoints,
    const QVector<QVector<QVector3D>>& segmentLaserPaths,
    const QVector<QVector3D>& transferPath){
    clearTwinM6Results();
    if(!m_renderer)return;
    m_twinM6SegmentCloudActors.reserve(std::size_t(segmentPoints.size()));
    m_twinM6SegmentPathActors.reserve(std::size_t(segmentLaserPaths.size()));
    QVector<QVector3D> starts,ends;
    for(qsizetype i=0;i<segmentPoints.size();++i){
        const auto rgb=kM6Colors[std::size_t(i)%kM6Colors.size()];
        auto* actor=makePointCloudActor(segmentPoints[i],rgb[0],rgb[1],rgb[2],2.5,0.99);
        if(actor)m_renderer->AddActor(actor);
        m_twinM6SegmentCloudActors.push_back(actor);
    }
    for(qsizetype i=0;i<segmentLaserPaths.size();++i){
        const auto rgb=kM6Colors[std::size_t(i)%kM6Colors.size()];
        const auto& path=segmentLaserPaths[i];
        auto* actor=makePolylineActor(path,rgb[0],rgb[1],rgb[2],3.,1.);
        if(actor)m_renderer->AddActor(actor);
        m_twinM6SegmentPathActors.push_back(actor);
        if(!path.isEmpty()){starts.push_back(path.front());ends.push_back(path.back());}
    }
    if(transferPath.size()>1){
        m_twinM6TransferActor=makePolylineActor(transferPath,.68,.69,.71,1.1,.65);
        if(m_twinM6TransferActor)m_renderer->AddActor(m_twinM6TransferActor);
    }
    m_twinM6StartActor=makePointCloudActor(starts,.12,.97,.30,9.,1.);
    m_twinM6EndActor=makePointCloudActor(ends,1.,.20,.16,9.,1.);
    if(m_twinM6StartActor)m_renderer->AddActor(m_twinM6StartActor);
    if(m_twinM6EndActor)m_renderer->AddActor(m_twinM6EndActor);
    refreshTwinOverlayVisibility();
    if(m_renderWindow)m_renderWindow->Render();
}
void PointCloudWidget::setTwinM6Results(const QVector<QVector3D>& points,
                                       const QVector<QVector3D>& sensorPath){
    clearTwinM6Results();
    if(!m_renderer)return;
    m_twinM6Actor=makePointCloudActor(points,0.55,0.25,1.0,2.8,0.9);
    if(m_twinM6Actor)m_renderer->AddActor(m_twinM6Actor);
    m_twinM6PathActor=makePolylineActor(sensorPath,1.,.75,.15,2.5,.95);
    if(m_twinM6PathActor)m_renderer->AddActor(m_twinM6PathActor);
    refreshTwinOverlayVisibility();
    if(m_renderWindow)m_renderWindow->Render();
}
void PointCloudWidget::setTwinMultiPoseWorkpieceMesh(const mls::twin::TriangleMesh& mesh,
    const mls::twin::Transform& pose){
    clearTwinMultiPoseWorkpiece();
    if(!m_renderer||mesh.empty())return;
    vtkNew<vtkPoints> pts;pts->SetDataTypeToDouble();
    vtkNew<vtkCellArray> tris;
    for(const auto& tri:mesh.triangles){
        vtkIdType v[3];
        v[0]=pts->InsertNextPoint(tri.a.x,tri.a.y,tri.a.z);
        v[1]=pts->InsertNextPoint(tri.b.x,tri.b.y,tri.b.z);
        v[2]=pts->InsertNextPoint(tri.c.x,tri.c.y,tri.c.z);
        tris->InsertNextCell(3,v);
    }
    vtkNew<vtkPolyData> poly;poly->SetPoints(pts);poly->SetPolys(tris);
    vtkNew<vtkPolyDataMapper> mapper;mapper->SetInputData(poly);mapper->ScalarVisibilityOff();
    m_multiPoseWorkpieceActor=vtkActor::New();m_multiPoseWorkpieceActor->SetMapper(mapper);
    m_multiPoseWorkpieceActor->GetProperty()->SetColor(.31,.73,.91);
    m_multiPoseWorkpieceActor->GetProperty()->SetOpacity(.60);
    vtkNew<vtkMatrix4x4> M;
    for(int row=0;row<4;++row)for(int col=0;col<4;++col)
        M->SetElement(row,col,pose.m[std::size_t(row*4+col)]);
    m_multiPoseWorkpieceActor->SetUserMatrix(M);
    m_renderer->AddActor(m_multiPoseWorkpieceActor);
    refreshTwinOverlayVisibility();
    if(m_renderWindow)m_renderWindow->Render();
}
void PointCloudWidget::clearTwinMultiPoseWorkpiece(){
    if(!m_multiPoseWorkpieceActor)return;
    if(m_renderer)m_renderer->RemoveActor(m_multiPoseWorkpieceActor);
    m_multiPoseWorkpieceActor->Delete();m_multiPoseWorkpieceActor=nullptr;
    if(m_renderWindow)m_renderWindow->Render();
}
void PointCloudWidget::focusTwinMultiPoseWorkpiece(){
    if(!m_multiPoseWorkpieceActor||!m_renderer)return;
    double b[6];m_multiPoseWorkpieceActor->GetBounds(b);
    if(!std::isfinite(b[0])||!std::isfinite(b[1]))return;
    focusTwinRegion({float(b[0]),float(b[2]),float(b[4])},
                    {float(b[1]),float(b[3]),float(b[5])});
}
void PointCloudWidget::setTwinMultiPoseRecordedPoints(const QVector<QVector3D>& points){
    clearTwinMultiPoseRecordedPoints();
    if(!m_renderer||points.isEmpty())return;
    m_multiPoseRecordedActor=makePointCloudActor(points,.83,.85,.89,1.8,.62);
    if(m_multiPoseRecordedActor)m_renderer->AddActor(m_multiPoseRecordedActor);
    refreshTwinOverlayVisibility();
    if(m_renderWindow)m_renderWindow->Render();
}
void PointCloudWidget::clearTwinMultiPoseRecordedPoints(){
    if(!m_multiPoseRecordedActor)return;
    if(m_renderer)m_renderer->RemoveActor(m_multiPoseRecordedActor);
    m_multiPoseRecordedActor->Delete();m_multiPoseRecordedActor=nullptr;
    if(m_renderWindow)m_renderWindow->Render();
}

// Dedicated multi-orientation research overlay; retaining original M6 display
// matters when comparing ROI experiments with the original M0-M8 workflow.
void PointCloudWidget::setTwinMultiPoseResults(
    const QVector<QVector<QVector3D>>& perPoseVirtualClouds,
    const QVector<QVector<QVector3D>>& perPoseScanPaths,
    const QVector<QVector3D>& laserOffTransferPath){
    clearTwinMultiPoseResults();
    if(!m_renderer)return;
    m_multiPoseCloudActors.reserve(std::size_t(perPoseVirtualClouds.size()));
    m_multiPosePathActors.reserve(std::size_t(perPoseScanPaths.size()));
    for(qsizetype i=0;i<perPoseVirtualClouds.size();++i){
        const auto rgb=kM6Colors[std::size_t(i)%kM6Colors.size()];
        auto* actor=makePointCloudActor(perPoseVirtualClouds[i],rgb[0],rgb[1],rgb[2],3.,.98);
        if(actor)m_renderer->AddActor(actor);
        m_multiPoseCloudActors.push_back(actor);
    }
    for(qsizetype i=0;i<perPoseScanPaths.size();++i){
        const auto rgb=kM6Colors[std::size_t(i)%kM6Colors.size()];
        auto* actor=makePolylineActor(perPoseScanPaths[i],rgb[0],rgb[1],rgb[2],3.2,.96);
        if(actor)m_renderer->AddActor(actor);
        m_multiPosePathActors.push_back(actor);
    }
    m_multiPoseTransferActor=makePolylineActor(laserOffTransferPath,.83,.81,.76,1.25,.70);
    if(m_multiPoseTransferActor)m_renderer->AddActor(m_multiPoseTransferActor);
    refreshTwinOverlayVisibility();
    if(m_renderWindow)m_renderWindow->Render();
}
void PointCloudWidget::setTwinMultiPosePoseSelection(int poseIndex){
    m_multiPoseActivePose=poseIndex;
    refreshTwinOverlayVisibility();
    if(m_renderWindow)m_renderWindow->Render();
}
void PointCloudWidget::clearTwinMultiPoseResults(){
    auto clearOne=[this](vtkActor*& actor){
        if(actor){if(m_renderer)m_renderer->RemoveActor(actor);actor->Delete();actor=nullptr;}
    };
    for(auto*& actor:m_multiPoseCloudActors)clearOne(actor);
    for(auto*& actor:m_multiPosePathActors)clearOne(actor);
    clearOne(m_multiPoseTransferActor);
    m_multiPoseCloudActors.clear();m_multiPosePathActors.clear();
    m_multiPoseActivePose=-1;
    if(m_renderWindow)m_renderWindow->Render();
}

void PointCloudWidget::setTwinMultiPosePlaybackActive(bool active,bool clearCloud){
    m_multiPosePlaybackActive=active;
    if(!active){
        // Restore the latest real machine presentation state. Offline playback
        // must never change the controller-backed position members.
        updateMachineActorTransforms();
        if(m_multiPoseLaserActor)m_multiPoseLaserActor->SetVisibility(false);
    }
    if(clearCloud){
        if(m_multiPoseAccumPoints)m_multiPoseAccumPoints->Reset();
        if(m_multiPoseAccumCells)m_multiPoseAccumCells->Reset();
        if(m_multiPoseAccumPoly)m_multiPoseAccumPoly->Modified();
    }
    refreshTwinOverlayVisibility();
    if(!active&&m_multiPoseLaserActor)m_multiPoseLaserActor->SetVisibility(false);
    if(m_renderWindow)m_renderWindow->Render();
}
void PointCloudWidget::setTwinMultiPosePlaybackFrame(const std::array<double,5>& q,
    const mls::twin::Transform& sensor,double focus,double width,
    const QVector<QVector3D>& observed,bool laserOn){
    if(!m_renderer||!m_multiPosePlaybackActive)return;
    // Only an ephemeral MODEL transform changes. Real machine state is intact.
    if(m_machineModelLoaded){
        const auto saveX=m_machineX;
        const auto saveY=m_machineY;
        const auto saveZ=m_machineZ;
        const auto saveC=m_machineC;
        const auto saveB=m_machineB;
        m_machineX=q[0];m_machineY=q[1];m_machineZ=q[2];m_machineC=q[3];m_machineB=q[4];
        updateMachineActorTransforms();
        m_machineX=saveX;m_machineY=saveY;m_machineZ=saveZ;m_machineC=saveC;m_machineB=saveB;
    }
    if(!m_multiPoseLaserActor){
        m_multiPoseLaserPoints=vtkPoints::New();
        m_multiPoseLaserPoints->SetDataTypeToDouble();
        for(int j=0;j<4;++j)m_multiPoseLaserPoints->InsertNextPoint(0.,0.,0.);
        vtkNew<vtkCellArray> cells;
        vtkIdType p[4]={0,1,2,3};cells->InsertNextCell(4,p);
        vtkNew<vtkPolyData> poly;poly->SetPoints(m_multiPoseLaserPoints);poly->SetLines(cells);
        vtkNew<vtkPolyDataMapper> mapper;mapper->SetInputData(poly);mapper->ScalarVisibilityOff();
        m_multiPoseLaserActor=vtkActor::New();m_multiPoseLaserActor->SetMapper(mapper);
        m_multiPoseLaserActor->GetProperty()->SetColor(1.,.32,.13);
        m_multiPoseLaserActor->GetProperty()->SetLineWidth(2.3);
        m_renderer->AddActor(m_multiPoseLaserActor);
    }
    const mls::twin::Vec3 laser= sensor.point({0.,0.,251.25});
    const mls::twin::Vec3 a=sensor.point({-width*.5,0.,focus});
    const mls::twin::Vec3 b=sensor.point({width*.5,0.,focus});
    const mls::twin::Vec3 vertices[4]={laser,a,b,laser};
    for(int j=0;j<4;++j)m_multiPoseLaserPoints->SetPoint(j,vertices[j].x,vertices[j].y,vertices[j].z);
    m_multiPoseLaserPoints->Modified();
    m_multiPoseLaserActor->SetVisibility(laserOn&&twinOverlayVisible(QStringLiteral("MULTIPOSE_PLAYBACK")));
    if(!m_multiPoseAccumActor){
        m_multiPoseAccumPoints=vtkPoints::New();m_multiPoseAccumPoints->SetDataTypeToFloat();
        m_multiPoseAccumCells=vtkCellArray::New();
        m_multiPoseAccumPoly=vtkPolyData::New();
        m_multiPoseAccumPoly->SetPoints(m_multiPoseAccumPoints);
        m_multiPoseAccumPoly->SetVerts(m_multiPoseAccumCells);
        vtkNew<vtkPolyDataMapper> mapper;mapper->SetInputData(m_multiPoseAccumPoly);
        mapper->ScalarVisibilityOff();
        m_multiPoseAccumActor=vtkActor::New();m_multiPoseAccumActor->SetMapper(mapper);
        m_multiPoseAccumActor->GetProperty()->SetColor(.12,1.,.76);
        m_multiPoseAccumActor->GetProperty()->SetPointSize(2.3);
        m_renderer->AddActor(m_multiPoseAccumActor);
    }
    for(const auto& p:observed){
        const vtkIdType id=m_multiPoseAccumPoints->InsertNextPoint(p.x(),p.y(),p.z());
        m_multiPoseAccumCells->InsertNextCell(1);m_multiPoseAccumCells->InsertCellPoint(id);
    }
    if(!observed.isEmpty()){
        m_multiPoseAccumPoints->Modified();m_multiPoseAccumCells->Modified();
        m_multiPoseAccumPoly->Modified();
    }
    m_multiPoseAccumActor->SetVisibility(twinOverlayVisible(QStringLiteral("MULTIPOSE_PLAYBACK")));
    if(m_renderWindow)m_renderWindow->Render();
}
void PointCloudWidget::clearTwinMultiPoseComparisonHeatmap(){
    if(m_multiPoseCompareActor){
        if(m_renderer)m_renderer->RemoveActor(m_multiPoseCompareActor);
        m_multiPoseCompareActor->Delete();m_multiPoseCompareActor=nullptr;
    }
    if(m_renderWindow)m_renderWindow->Render();
}
void PointCloudWidget::setTwinMultiPoseComparisonHeatmap(const QVector<QVector3D>& points,
    const QVector<float>& residual,double scale){
    clearTwinMultiPoseComparisonHeatmap();
    if(!m_renderer||points.isEmpty()||points.size()!=residual.size()||scale<=0)return;
    vtkNew<vtkPoints> vertices;vertices->SetDataTypeToFloat();
    vtkNew<vtkCellArray> cells;
    vtkNew<vtkUnsignedCharArray> rgb;rgb->SetName("Real-to-synthetic NN distance (uncertified)");
    rgb->SetNumberOfComponents(3);
    for(qsizetype i=0;i<points.size();++i){
        const auto& p=points[i];
        const auto id=vertices->InsertNextPoint(p.x(),p.y(),p.z());
        cells->InsertNextCell(1);cells->InsertCellPoint(id);
        const double u=std::clamp(double(residual[i])/scale,0.,1.);
        const unsigned char rr=static_cast<unsigned char>(std::min(255.,510.*u));
        const unsigned char gg=static_cast<unsigned char>(255.*(1.-u));
        const unsigned char bb=static_cast<unsigned char>(155.*(1.-u));
        rgb->InsertNextTuple3(rr,gg,bb);
    }
    vtkNew<vtkPolyData> poly;poly->SetPoints(vertices);poly->SetVerts(cells);
    poly->GetPointData()->SetScalars(rgb);
    vtkNew<vtkPolyDataMapper> mapper;mapper->SetInputData(poly);
    mapper->SetScalarModeToUsePointData();mapper->ScalarVisibilityOn();
    m_multiPoseCompareActor=vtkActor::New();m_multiPoseCompareActor->SetMapper(mapper);
    m_multiPoseCompareActor->GetProperty()->SetPointSize(3.);
    m_renderer->AddActor(m_multiPoseCompareActor);
    refreshTwinOverlayVisibility();
    if(m_renderWindow)m_renderWindow->Render();
}

void PointCloudWidget::clearTwinM6Results(){
    auto clearOne=[this](vtkActor*& actor){
        if(actor){if(m_renderer)m_renderer->RemoveActor(actor);actor->Delete();actor=nullptr;}
    };
    clearOne(m_twinM6Actor);clearOne(m_twinM6PathActor);
    clearOne(m_twinM6TransferActor);clearOne(m_twinM6StartActor);clearOne(m_twinM6EndActor);
    clearOne(m_twinM6NewCoverageActor);clearOne(m_twinM6RemainingActor);
    clearOne(m_twinM6CursorActor);
    for(auto*& a:m_twinM6SegmentCloudActors)clearOne(a);
    for(auto*& a:m_twinM6SegmentPathActors)clearOne(a);
    m_twinM6SegmentCloudActors.clear();m_twinM6SegmentPathActors.clear();
    m_twinM6ActiveSegment=-1;
    if(m_renderWindow)m_renderWindow->Render();
}

// M7 point/diagnostic overlays: data from genuine M2 preview, genuine M3/M6
// simulated ray returns, and (separately) CAD coverage markers. No device IO.
void PointCloudWidget::clearTwinM7Results(){
    for(auto** ptr:{&m_twinM7FusedActor,&m_twinM7RealActor,&m_twinM7SimActor,&m_twinM7OverlapActor,
                    &m_twinM7NewCadActor,&m_twinM7MissingActor}){
        if(*ptr){if(m_renderer)m_renderer->RemoveActor(*ptr);(*ptr)->Delete();*ptr=nullptr;}
    }
    if(m_renderWindow)m_renderWindow->Render();
}
void PointCloudWidget::setTwinM7Results(const QVector<QVector3D>& real,
    const QVector<QVector3D>& simulated,const QVector<QVector3D>& overlap,
    const QVector<QVector3D>& newCad,const QVector<QVector3D>& missingCad,
    const QVector<QVector3D>& fused){
    clearTwinM7Results();
    if(!m_renderer)return;
    const auto insert=[this](vtkActor** ptr,const QVector<QVector3D>& pts,
                             double r,double g,double b,double size,double alpha){
        if(pts.isEmpty())return;
        *ptr=makePointCloudActor(pts,r,g,b,size,alpha);
        if(*ptr)m_renderer->AddActor(*ptr);
    };
    insert(&m_twinM7FusedActor,fused,.95,.92,.79,2.0,1.);
    insert(&m_twinM7RealActor,real,0.10,0.88,0.84,2.5,1.);
    insert(&m_twinM7SimActor,simulated,0.74,0.43,0.96,2.5,0.93);
    insert(&m_twinM7OverlapActor,overlap,0.99,0.94,0.32,3.2,1.);
    insert(&m_twinM7NewCadActor,newCad,0.20,0.94,0.25,4.0,0.95);
    insert(&m_twinM7MissingActor,missingCad,0.98,0.20,0.20,4.0,0.95);
    refreshTwinOverlayVisibility();
    if(m_renderWindow)m_renderWindow->Render();
}
