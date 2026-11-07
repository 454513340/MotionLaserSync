#pragma once

#include <QWidget>
#include <QVector>
#include <QVector3D>
#include <QHash>
#include <QMutex>
#include <QWaitCondition>
#include <QTimer>
#include <QThread>
#include <QStringList>
#include <QFutureWatcher>
#include <memory>
#include <cstdint>
#include "geometry/OrganizedCloudMesher.h"
#include <array>
#include <vector>
#include <atomic>
#include "twin/TwinTypes.h"

// VTK 前向声明
class vtkRenderer;
class vtkRenderWindow;
class vtkRenderWindowInteractor;
class vtkPoints;
class vtkCellArray;
class vtkPolyData;
class vtkPolyDataMapper;
class vtkActor;
class vtkVertexGlyphFilter;
class vtkUnsignedCharArray;
class vtkLookupTable;
class vtkScalarBarActor;
class vtkAxesActor;
class vtkOrientationMarkerWidget;
class vtkSphereSource;
class QVTKOpenGLNativeWidget;
class QCheckBox;
class QLabel;
class QComboBox;
class QDoubleSpinBox;

class PointCloudBuilder;

/**
 * @brief 后台数据准备线程
 */
class PointCloudPrepareThread : public QThread
{
    Q_OBJECT
public:
    explicit PointCloudPrepareThread(QObject* parent = nullptr);
    ~PointCloudPrepareThread() override;
    
    void setBuilder(PointCloudBuilder* builder) { m_builder = builder; }
    void requestUpdate();
    void requestStop();
    
signals:
    /**
     * @brief 数据准备完成，可以渲染
     * @param points 点数据（已拷贝，可安全使用）
     * @param zMin 最小Z值
     * @param zMax 最大Z值
     */
    void dataReady(const QVector<QVector3D>& points, float zMin, float zMax);
    
protected:
    void run() override;
    
private:
    PointCloudBuilder* m_builder = nullptr;
    std::atomic<bool> m_stopRequested{false};
    std::atomic<bool> m_updateRequested{false};
    QMutex m_mutex;
    QWaitCondition m_condition;
    int m_lastProcessedCount = 0;
};

/**
 * @brief 基于 VTK 的高性能点云显示组件（多线程版）
 */
class PointCloudWidget : public QWidget
{
    Q_OBJECT

public:
    enum class ColorMode {
        Uniform,
        ByHeight
    };

    explicit PointCloudWidget(QWidget* parent = nullptr);
    ~PointCloudWidget() override;

    void setPointCloudBuilder(PointCloudBuilder* builder);
    void clear();
    int pointCount() const;
    
    /**
     * @brief 直接设置点云数据（用于导入外部点云）
     * @param points 点云数据
     */
    void setPoints(const QVector<QVector3D>& points);

    // Only the explicit File > Import Cloud path uses this API. .scan retains
    // exact archived row offsets + original sensor columns; plain XYZ files
    // have empty metadata and must pass the organized-width sanity check.
    struct ImportedCloudLayout {
        QVector<qsizetype> rowOffsets;
        QVector<quint32> originalColumns;
        int pointsPerProfile = 0;
        bool hasOriginalIndices = false;
        bool forcePointOnly = false; // damaged .scan: do not guess flattened topology
    };
    void setImportedPoints(const QVector<QVector3D>& points,
                           const ImportedCloudLayout& layout);


    /** Load the exact Machine5X visual model used by the FCL collision model. */
    bool loadMachineModel(const QString& machineConfigPath, QString* error = nullptr);
    bool machineModelLoaded() const { return m_machineModelLoaded; }
    void setMachineModelVisible(bool visible);
    bool machineModelVisible() const { return m_machineModelVisible; }
    void setMachineCollisionModelVisible(bool visible);
    bool machineCollisionModelVisible() const { return m_machineCollisionModelVisible; }
    void setMachineJointPose(double xMm, double yMm, double zMm, double cDeg, double bDeg);
    void setMachineCollisionLinks(const QStringList& links);
    void setMachineSafetyStatus(const QString& text, bool collision);
    // V27.6.53: visible provenance for whether the currently acquired cloud is
    // reconstructed from raw same-ordinal REGIST(104) poses or q(t+tau).
    void setTemporalCorrectionStatus(bool loaded, bool enabled, double delayUs,
                                     const QString& calibrationId = QString(),
                                     bool hardwareSyncSelected = true);
    void setMachineStudySphere(double xMm, double yMm, double zMm,
                               double radiusMm, double safetyClearanceMm);
    void clearMachineStudySphere();

    /** V28 M1 offline STEP / OBJ workpiece visualization in MCS. No motor control. */
    void setTwinWorkpieceMesh(const mls::twin::TriangleMesh& mesh,
                              const mls::twin::Transform& cadToMachine);
    void setTwinWorkpiecePose(const mls::twin::Transform& cadToMachine);
    void clearTwinWorkpiece();
    void setTwinSupportPlane(bool enabled, double machineZ,
                             mls::twin::Vec3 center, mls::twin::Vec3 size);

    // V28 M1/M2: main viewport overlays are separate from live acquisition.
    // These functions never issue hardware commands or alter PointCloudBuilder.
    void focusTwinWorkpiece();
    void focusTwinRegion(const QVector3D& minMcs,const QVector3D& maxMcs);
    void appendTwinReplayPoints(const QVector<QVector3D>& allPreview,
                                qsizetype begin, qsizetype end);
    void clearTwinReplayPoints();
    // V28 M3: separate orange virtual scan actor; does not alter live/M2 clouds.
    void setTwinVirtualPoints(const QVector<QVector3D>& preview);
    void clearTwinVirtualPoints();
    // M5 area-weighted CAD proxy occupancy; separate from live/M2/M3 actors.
    void setTwinCoveragePoints(const QVector<QVector3D>& realObserved,
                               const QVector<QVector3D>& realMissing,
                               const QVector<QVector3D>& virtualOnly);
    void clearTwinCoveragePoints();
    // M6 independent simulated rescue cloud and continuous sensor-space trajectory.
    void setTwinM6Results(const QVector<QVector3D>& points,
                          const QVector<QVector3D>& sensorPath);
    void clearTwinM6Results();
    // V28.7.4: persistent independent visibility for each digital twin layer.
    // Hiding never deletes a scan, CAD model or user data.
    void setTwinOverlayVisible(const QString& layer,bool visible);
    void setTwinOverlayVisibilityBatch(const QHash<QString,bool>& layers);
    bool twinOverlayVisible(const QString& layer) const;
    void setTwinStageMarkers(const QString& layer,const QVector<QVector3D>& points);
    void setTwinM6DetailedResults(const QVector<QVector<QVector3D>>& segmentPoints,
         const QVector<QVector<QVector3D>>& segmentLaserPaths,
         const QVector<QVector3D>& transferPath);
    void setTwinM6SegmentSelection(int segmentIndex); // -1=all, otherwise 0-based
    // Independent multi-orientation experiment actors. Never reuses or clears
    // M0-M8 M6/M7 layers, real M2 replay or the live acquisition view.
    // Dedicated recorded/CAD experiment evidence, separate from original M1/M2.
    void setTwinMultiPoseWorkpieceMesh(const mls::twin::TriangleMesh& mesh,
                                     const mls::twin::Transform& cadToMachine);
    void clearTwinMultiPoseWorkpiece();
    void focusTwinMultiPoseWorkpiece();
    void setTwinMultiPoseRecordedPoints(const QVector<QVector3D>& points);
    void clearTwinMultiPoseRecordedPoints();
    void setTwinMultiPoseResults(const QVector<QVector<QVector3D>>& perPoseVirtualClouds,
                                 const QVector<QVector<QVector3D>>& perPoseScanPaths,
                                 const QVector<QVector3D>& laserOffTransferPath);
    void clearTwinMultiPoseResults();
    void setTwinMultiPosePoseSelection(int poseIndex); // -1 = all independent pose actors
    // Offline simulation ONLY: draw planned machine joints and the sensor's
    // ideal laser fan. Never updates or commands any physical machine axes.
    void setTwinMultiPosePlaybackActive(bool active, bool clearCloud=false);
    void setTwinMultiPosePlaybackFrame(const std::array<double,5>& virtualJoints,
        const mls::twin::Transform& sensorToMachine, double focusSensorZmm,
        double scanWidthMm,const QVector<QVector3D>& newProfilePoints,bool laserOn);
    void setTwinMultiPoseComparisonHeatmap(const QVector<QVector3D>& realPoints,
        const QVector<float>& distanceMm,double colorLimitMm);
    void clearTwinMultiPoseComparisonHeatmap();
    // M7 independent preview: real M2 and simulated M6 are never silently averaged.
    void setTwinM7Results(const QVector<QVector3D>& real,
        const QVector<QVector3D>& simulated,const QVector<QVector3D>& overlap,
        const QVector<QVector3D>& newCad,const QVector<QVector3D>& remainingCad,
        const QVector<QVector3D>& fused);
    void clearTwinM7Results();



    /**
     * V27.6.16AC scan digital twin.  All points are already expressed in the
     * machine coordinate system by the calibrated hand-eye/Physical17 model.
     * This only renders the sensor X-Z measuring plane and its sphere
     * intersection; it never commands hardware motion.
     */
    void setSphereStudyScanSimulation(const QVector<QVector3D>& laserFieldCornersMachineMm,
                                      const QVector<QVector3D>& sectionCircleMachineMm,
                                      const QVector<QVector3D>& visibleArcMachineMm,
                                      bool feasible,
                                      const QString& statusText);
    void clearSphereStudyScanSimulation();

    /**
     * V27.6.16AC offline acquisition preview.  This point actor is deliberately
     * separate from the transient laser plane / current section arc so the
     * accumulated ideal sensor samples remain visible after playback stops.
     */
    void setSphereStudySimulatedAcquisitionCloud(const QVector<QVector3D>& pointsMachineMm);
    void clearSphereStudySimulatedAcquisitionCloud();

    /** Read-only 3D preview of the fixed sensor-frame point's planned X sweep. */
    void setSphereExposureSweepPath(const QVector<QVector3D>& sensorTraceMachineMm);
    void clearSphereExposureSweepPath();

    void setColorMode(ColorMode mode);
    ColorMode colorMode() const { return m_colorMode; }
    void setUniformColor(const QColor& color);
    void setPointSize(float size);
    float pointSize() const { return m_pointSize; }
    void setAxesVisible(bool visible);
    bool axesVisible() const { return m_axesVisible; }
    void setScalarBarVisible(bool visible);
    bool scalarBarVisible() const { return m_scalarBarVisible; }

    void resetCamera();
    void setTopView();
    void setFrontView();
    void setSideView();
    void setIsometricView();
    
    /**
     * @brief 设置机床工作范围，用于初始化相机视角
     * @param xRange X轴范围 (mm)
     * @param yRange Y轴范围 (mm)
     * @param zRange Z轴范围 (mm)
     * @param sensorZMin 传感器Z最小值 (mm)
     * @param sensorZMax 传感器Z最大值 (mm)
     */
    void setWorkingVolume(double xRange, double yRange, double zRange,
                          double sensorZMin, double sensorZMax);
    
    /**
     * @brief 设置初始相机视角到工作区域中心
     */
    void setupInitialCamera();

public slots:
    void onPointsAdded(int addedCount, int totalCount);
    void onCleared();

protected:
    void resizeEvent(QResizeEvent* event) override;

private slots:
    void onRenderTimer();
    void onDataReady(const QVector<QVector3D>& points, float zMin, float zMax);

private:
    void initializeVTK();
    void updateColorMapping();
    void uploadPointsToVTK(const QVector<QVector3D>& points,float zMin,float zMax);
    void leaveImportedMode();
    void invalidateImportedMesh();
    void startImportedMeshBuild();
    void refreshImportedRepresentation();
    int effectiveImportedWidth() const;

    void setupAxes();
    void setupScalarBar();
    void updateMachineActorTransforms();
    void applyMachineActorStyles();
    void refreshTwinOverlayVisibility();

private:
    QVTKOpenGLNativeWidget* m_vtkWidget = nullptr;
    vtkRenderer* m_renderer = nullptr;
    vtkRenderWindow* m_renderWindow = nullptr;
    vtkRenderWindowInteractor* m_interactor = nullptr;
    vtkPoints* m_vtkPoints = nullptr;
    vtkPolyData* m_polyData = nullptr;
    vtkVertexGlyphFilter* m_glyphFilter = nullptr;
    vtkPolyDataMapper* m_mapper = nullptr;
    vtkActor* m_actor = nullptr;
    // Imported scan mesh: shares m_vtkPoints and its Height scalars, but uses
    // an independent strip-cell actor. Never edits the live point actor.
    vtkPolyData* m_importMeshPolyData = nullptr;
    vtkPolyDataMapper* m_importMeshMapper = nullptr;
    vtkActor* m_importMeshActor = nullptr;
    QWidget* m_importControls = nullptr;
    QComboBox* m_importDisplayMode = nullptr;
    QComboBox* m_importProfileWidth = nullptr;
    QComboBox* m_importColorMode = nullptr;
    QDoubleSpinBox* m_importEdgeLimit = nullptr;
    QLabel* m_importStatus = nullptr;
    ImportedCloudLayout m_importLayout;
    QVector<QVector3D> m_importedPoints;
    bool m_importActive = false;
    bool m_meshReady = false;
    bool m_meshAttempted = false;
    QString m_meshSummary;
    int m_inferredProfileWidth = 0;
    std::uint64_t m_meshGeneration = 0;
    std::shared_ptr<std::atomic_bool> m_meshCancel;
    QFutureWatcher<mls::geometry::StripMeshResult>* m_meshWatcher = nullptr;

    vtkLookupTable* m_lut = nullptr;
    vtkScalarBarActor* m_scalarBar = nullptr;
    vtkAxesActor* m_axesActor = nullptr;
    vtkOrientationMarkerWidget* m_axesWidget = nullptr;


    struct MachineVisualActor {
        QString name;
        vtkActor* actor = nullptr;
        std::array<double,3> baseColor{{0.70,0.72,0.75}};
    };
    struct MachineCollisionActor {
        QString name;
        vtkActor* actor = nullptr;
    };
    std::vector<MachineVisualActor> m_machineActors;
    std::vector<MachineCollisionActor> m_machineCollisionActors;
    QStringList m_machineCollisionLinks;
    bool m_machineModelLoaded = false;
    bool m_machineModelVisible = true;
    bool m_machineCollisionModelVisible = false;
    double m_machineX = 0.0;
    double m_machineY = 0.0;
    double m_machineZ = 0.0;
    double m_machineC = 0.0;
    double m_machineB = 0.0;
    QCheckBox* m_machineVisibleCheck = nullptr;
    QCheckBox* m_machineCollisionVisibleCheck = nullptr;
    QLabel* m_machineSafetyLabel = nullptr;
    QLabel* m_temporalCorrectionLabel = nullptr;
    vtkActor* m_twinWorkpieceActor = nullptr;
    vtkActor* m_twinSupportActor = nullptr;
    vtkActor* m_twinReplayActor = nullptr;
    vtkActor* m_twinVirtualActor = nullptr;
    vtkActor* m_twinM5ObservedActor = nullptr;
    vtkActor* m_twinM5MissingActor = nullptr;
    vtkActor* m_twinM5VirtualOnlyActor = nullptr;
    vtkActor* m_twinM6Actor = nullptr;
    vtkActor* m_twinM6PathActor = nullptr;
    std::vector<vtkActor*> m_twinM6SegmentCloudActors;
    std::vector<vtkActor*> m_twinM6SegmentPathActors;
    vtkActor* m_twinM6TransferActor = nullptr;
    std::vector<vtkActor*> m_multiPoseCloudActors;
    std::vector<vtkActor*> m_multiPosePathActors;
    vtkActor* m_multiPoseTransferActor = nullptr;
    vtkActor* m_multiPoseWorkpieceActor = nullptr;
    vtkActor* m_multiPoseRecordedActor = nullptr;
    // Independent research actors, never shared with live hardware or M0-M8.
    vtkActor* m_multiPoseLaserActor = nullptr;
    vtkPoints* m_multiPoseLaserPoints = nullptr;
    vtkActor* m_multiPoseAccumActor = nullptr;
    vtkPoints* m_multiPoseAccumPoints = nullptr;
    vtkCellArray* m_multiPoseAccumCells = nullptr;
    vtkPolyData* m_multiPoseAccumPoly = nullptr;
    vtkActor* m_multiPoseCompareActor = nullptr;
    bool m_multiPosePlaybackActive=false;
    int m_multiPoseActivePose = -1;
    vtkActor* m_twinM6StartActor = nullptr;
    vtkActor* m_twinM6EndActor = nullptr;
    vtkActor* m_twinM4RoiActor = nullptr;
    vtkActor* m_twinM5FeatureActor = nullptr;
    vtkActor* m_twinM51SparseActor = nullptr;
    vtkActor* m_twinM52ResidualActor = nullptr;
    vtkActor* m_twinM6NewCoverageActor = nullptr;
    vtkActor* m_twinM6RemainingActor = nullptr;
    vtkActor* m_twinM6CursorActor = nullptr;
    vtkActor* m_twinM7FusedActor = nullptr;
    vtkActor* m_twinM7RealActor = nullptr;
    vtkActor* m_twinM7SimActor = nullptr;
    vtkActor* m_twinM7OverlapActor = nullptr;
    vtkActor* m_twinM7NewCadActor = nullptr;
    vtkActor* m_twinM7MissingActor = nullptr;
    QHash<QString,bool> m_twinVisibility;
    int m_twinM6ActiveSegment = -1;
    vtkPoints* m_twinReplayPoints = nullptr;
    vtkCellArray* m_twinReplayCells = nullptr;
    vtkPolyData* m_twinReplayPoly = nullptr;
    vtkSphereSource* m_machineStudySphereSource = nullptr;
    vtkActor* m_machineStudySphereActor = nullptr;
    vtkSphereSource* m_machineStudySafetySphereSource = nullptr;
    vtkActor* m_machineStudySafetySphereActor = nullptr;
    vtkActor* m_sphereStudyLaserPlaneActor = nullptr;
    vtkActor* m_sphereStudySectionCircleActor = nullptr;
    vtkActor* m_sphereStudyVisibleArcActor = nullptr;
    vtkActor* m_sphereStudySimulatedCloudActor = nullptr;
    vtkActor* m_sphereExposureSweepPathActor = nullptr;
    vtkActor* m_sphereExposureStartActor = nullptr;
    vtkActor* m_sphereExposureEndActor = nullptr;
    QLabel* m_sphereStudySimulationLabel = nullptr;

    // 后台线程
    PointCloudPrepareThread* m_prepareThread = nullptr;
    PointCloudBuilder* m_builder = nullptr;
    
    // 渲染控制
    QTimer m_renderTimer;
    std::atomic<bool> m_needsUpdate{false};
    std::atomic<bool> m_dataReady{false};
    
    // 待渲染数据（由后台线程准备）
    QVector<QVector3D> m_pendingPoints;
    float m_pendingZMin = 0;
    float m_pendingZMax = 0;
    QMutex m_pendingMutex;

    ColorMode m_colorMode = ColorMode::ByHeight;
    QColor m_uniformColor = QColor(230, 230, 230);
    float m_pointSize = 2.0f;
    bool m_axesVisible = true;
    bool m_scalarBarVisible = true;
    
    // 机床工作范围（用于初始相机设置）
    double m_workXRange = 270.0;      // X轴范围 (mm)
    double m_workYRange = 270.0;      // Y轴范围 (mm)
    double m_workZRange = 90.0;       // Z轴范围 (mm)
    double m_sensorZMin = 50.0;       // 传感器Z最小值 (mm)
    double m_sensorZMax = 120.0;      // 传感器Z最大值 (mm)
    bool m_initialCameraSet = false;  // 初始相机是否已设置
};
