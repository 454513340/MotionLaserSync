#pragma once
// V28 M0/M1: offline scene setup only. No motion/laser/controller actions.
#include <QDialog>
#include <QString>
#include "twin/TwinSession.h"
#include "twin/TwinM8Workflow.h"
#include <array>
#include <memory>
#include <atomic>
#include <QPointer>
#include <QHash>
#include <QElapsedTimer>
#include <QFutureWatcher>
class PointCloudWidget;
class QDoubleSpinBox;
class QCheckBox;
class QLabel;
class QPushButton;
class QLineEdit;
class QCloseEvent;
class QComboBox;
class QSlider;
class QTimer;
class QSpinBox;
class QPlainTextEdit;
class QProgressBar;
class QVBoxLayout;
class QTableWidget;
namespace mls::twin { struct ScanReplayData; struct TwinSimulationData; struct TwinM4ComparisonData; struct TwinM4FitData; struct TwinM5Data; struct TwinM51Data; struct TwinM52Data; struct TwinM6Data; struct M6ProgressState; struct M6ExportProgress; struct TwinM7Data; struct M7Progress; }

class DigitalTwinScanWidget final : public QDialog {
    Q_OBJECT
public:
    explicit DigitalTwinScanWidget(QWidget* parent=nullptr);
    ~DigitalTwinScanWidget() override;
    // Only visual overlays are mirrored to the main viewport. The main cloud
    // remains owned by the live acquisition system.
    void setMainPointCloudView(PointCloudWidget* view);
protected:
    void closeEvent(QCloseEvent* event) override;
private:
    void buildUI();
    bool importStepFile(const QString& path, bool showDialog=true);
    bool importObjFile(const QString& path, bool showDialog=true);
    void loadReference();
    void updatePlacement();
    void updateSupport();
    void updateSceneGraphics(bool focus);
    void refreshInfo(const QString& note=QString());
    void saveSession();
    void loadSession();
    QString findExample(const QString& basename) const;
    mls::twin::Pose6 uiPose() const;
    void setUiPose(const mls::twin::Pose6& pose);
    void tryLoadMachine();
    void syncMainScene(bool focus=false);
    void browseReplayScan();
    void onReplayLoaded();
    void seekReplay(int frame);
    void onReplayTick();
    void startReplay();
    void stopReplay();
    void resetReplay();
    void exportReplayPly();
    void applyReplayFrame(int frame);
    void updateReplayInfo();
    // M3 offline virtual-profile pipeline & independent layer cleanup.
    void clearWorkpiece();
    void clearSupport();
    void clearReplay();
    void clearVirtual();
    void clearAllTwinData();
    void startVirtualSimulation();
    void cancelVirtualSimulation();
    void onVirtualFinished();
    void exportVirtualResults();
    void updateVirtualProgress();
    // M4 real-vs-virtual indexed profile validation and optional baseline fit.
    void startM4Comparison();
    void onM4ComparisonFinished();
    void startM4Fit();
    void onM4FitFinished();
    void cancelM4();
    void clearM4();
    void exportM4Report();
    void exportM4Fit();
    void applyM4Fit();
    void updateM4Progress();
    // M5 triangulated CAD surface occupancy; diagnostic, no point synthesis.
    void startM5();
    void onM5Finished();
    void cancelM5();
    void clearM5();
    void exportM5();
    void updateM5Progress();
    // M5.1: H1 center hole surface-specific angular/radial density diagnostic.
    void startM51();
    void onM51Finished();
    void cancelM51();
    void clearM51();
    void exportM51();
    void updateM51Progress();
    void refreshM51Heatmap();
    // M5.2: independent, no-pose-write geometry consistency diagnosis.
    void startM52();
    void onM52Finished();
    void cancelM52();
    void clearM52();
    void exportM52();
    void updateM52Progress();
    void refreshM52Heatmap();

    // M6: continuous and fully simulated XYZ/C/B offline planning.
    void startM6();
    void onM6Finished();
    void cancelM6();
    void clearM6();
    void exportM6();
    void updateM6Progress();
    // M7: provenance-aware real/simulated fusion + multi-view coverage acceptance.
    void startM7();
    void onM7Finished();
    void cancelM7();
    void clearM7();
    void exportM7();
    void updateM7Progress();
    // V28.7.4 visualization / rescanning diagnostics, no hardware operations.
    void buildTwinInspector(QVBoxLayout* vbox,QWidget* panel);
    void setTwinLayerVisibility(const QString& id,bool visible);
    void setTwinViewPreset(int preset);
    void refreshTwinInspector();
    void refreshM5FeatureTable();
    void inspectM5Feature(int row);
    void selectM6Segment(int comboIndex);
    void seekM6Pose(int sliderValue);
    void exportM5RescanTargets();
    // M8 offline end-to-end workflow: serial async stages, explicit provenance
    // guard, predicted acceptance report; never touches a live controller.
    void buildM8Workflow(QVBoxLayout* vbox,QWidget* panel);
    void preflightM8();
    void startM8();
    void cancelM8();
    void dispatchM8Stage();
    void onM8StageFinished(mls::twin::M8Stage stage,bool ok,const QString& reason=QString());
    void finishM8(const QString& reason,bool cancelled=false);
    void updateM8Progress();
    void exportM8Report();
    bool isM8SceneUnchanged() const;
    bool m8StageReady(mls::twin::M8Stage stage) const;
    QString m8StepName(mls::twin::M8Stage stage) const;
    void refreshM8StageTable();

    mls::twin::TwinSession m_session;
    PointCloudWidget* m_view=nullptr;
    QPointer<PointCloudWidget> m_mainView;
    std::array<QDoubleSpinBox*,6> m_pose{};
    QCheckBox* m_idealRegistration=nullptr;
    QCheckBox* m_supportCheck=nullptr;
    QDoubleSpinBox* m_supportZ=nullptr;
    QDoubleSpinBox* m_supportX=nullptr;
    QDoubleSpinBox* m_supportY=nullptr;
    QDoubleSpinBox* m_supportW=nullptr;
    QDoubleSpinBox* m_supportH=nullptr;
    QDoubleSpinBox* m_meshTolerance=nullptr;
    QLabel* m_status=nullptr;
    QLabel* m_fileLabel=nullptr;
    QString m_filePath;
    QString m_sourceMode;
    QString m_importNote;
    std::shared_ptr<mls::twin::ScanReplayData> m_replay;
    bool m_discardReplayOnFinish=false;
    QFutureWatcher<std::shared_ptr<mls::twin::ScanReplayData>>* m_replayWatcher=nullptr;
    QFutureWatcher<QString>* m_exportWatcher=nullptr;
    QTimer* m_replayTimer=nullptr;
    QElapsedTimer m_replayClock;
    QSlider* m_replaySlider=nullptr;
    QComboBox* m_replaySpeed=nullptr;
    QLabel* m_replayInfo=nullptr;
    QPushButton* m_replayOpen=nullptr;
    QPushButton* m_replayPlay=nullptr;
    QPushButton* m_replayExport=nullptr;
    int m_replayIndex=-1;
    qsizetype m_replayDisplayedEnd=0;
    double m_playVirtualStart=0.0;
    QString m_replayFilePath;
    std::shared_ptr<mls::twin::TwinSimulationData> m_virtual;
    QFutureWatcher<std::shared_ptr<mls::twin::TwinSimulationData>>* m_virtualWatcher=nullptr;
    QFutureWatcher<QString>* m_virtualExportWatcher=nullptr;
    std::shared_ptr<std::atomic<bool>> m_virtualCancel;
    std::shared_ptr<std::atomic<std::size_t>> m_virtualProgress;
    bool m_virtualDiscard=false;
    QTimer* m_virtualProgressTimer=nullptr;
    QDoubleSpinBox* m_cameraBaselineY=nullptr;
    QPushButton* m_virtualRunButton=nullptr;
    QPushButton* m_virtualCancelButton=nullptr;
    QPushButton* m_virtualExportButton=nullptr;
    QLabel* m_virtualInfo=nullptr;
    std::shared_ptr<mls::twin::TwinM4ComparisonData> m_m4Comparison;
    std::shared_ptr<mls::twin::TwinM4FitData> m_m4Fit;
    QFutureWatcher<std::shared_ptr<mls::twin::TwinM4ComparisonData>>* m_m4CompareWatcher=nullptr;
    QFutureWatcher<std::shared_ptr<mls::twin::TwinM4FitData>>* m_m4FitWatcher=nullptr;
    std::shared_ptr<std::atomic<bool>> m_m4Cancel;
    std::shared_ptr<std::atomic<std::size_t>> m_m4Progress;
    QTimer* m_m4ProgressTimer=nullptr;
    bool m_m4Discard=false;
    QPushButton* m_m4RunButton=nullptr;
    QPushButton* m_m4FitButton=nullptr;
    QPushButton* m_m4ExportButton=nullptr;
    QPushButton* m_m4FitExportButton=nullptr;
    QPushButton* m_m4ApplyButton=nullptr;
    QPushButton* m_m4CancelButton=nullptr;
    QDoubleSpinBox* m_m4MinY=nullptr;
    QDoubleSpinBox* m_m4MaxY=nullptr;
    QDoubleSpinBox* m_m4StepY=nullptr;
    QSpinBox* m_m4FrameCap=nullptr;
    QPlainTextEdit* m_m4Info=nullptr;
    std::shared_ptr<mls::twin::TwinM5Data> m_m5Data;
    QFutureWatcher<std::shared_ptr<mls::twin::TwinM5Data>>* m_m5Watcher=nullptr;
    QFutureWatcher<QString>* m_m5ExportWatcher=nullptr;
    std::shared_ptr<std::atomic<bool>> m_m5Cancel;
    std::shared_ptr<std::atomic<std::size_t>> m_m5Progress;
    QTimer* m_m5ProgressTimer=nullptr;
    bool m_m5Discard=false;
    QPushButton* m_m5RunButton=nullptr;
    QPushButton* m_m5ClearButton=nullptr;
    QPushButton* m_m5ExportButton=nullptr;
    QPushButton* m_m5CancelButton=nullptr;
    QDoubleSpinBox* m_m5Pitch=nullptr;
    QDoubleSpinBox* m_m5MatchDistance=nullptr;
    QSpinBox* m_m5MinHits=nullptr;
    QCheckBox* m_m5ExcludeBottom=nullptr;
    QPlainTextEdit* m_m5Info=nullptr;
    std::shared_ptr<mls::twin::TwinM51Data> m_m51Data;
    QFutureWatcher<std::shared_ptr<mls::twin::TwinM51Data>>* m_m51Watcher=nullptr;
    QFutureWatcher<QString>* m_m51ExportWatcher=nullptr;
    std::shared_ptr<std::atomic<bool>> m_m51Cancel;
    std::shared_ptr<std::atomic<std::size_t>> m_m51Progress;
    QTimer* m_m51ProgressTimer=nullptr;
    bool m_m51Discard=false;
    QPushButton* m_m51RunButton=nullptr;
    QPushButton* m_m51CancelButton=nullptr;
    QPushButton* m_m51ClearButton=nullptr;
    QPushButton* m_m51ExportButton=nullptr;
    QDoubleSpinBox* m_m51SparseRatio=nullptr;
    QComboBox* m_m51Threshold=nullptr;
    QComboBox* m_m51ViewMode=nullptr;
    QTableWidget* m_m51Heatmap=nullptr;
    QPlainTextEdit* m_m51Info=nullptr;
    std::shared_ptr<mls::twin::TwinM52Data> m_m52Data;
    QFutureWatcher<std::shared_ptr<mls::twin::TwinM52Data>>* m_m52Watcher=nullptr;
    QFutureWatcher<QString>* m_m52ExportWatcher=nullptr;
    std::shared_ptr<std::atomic<bool>> m_m52Cancel;
    std::shared_ptr<std::atomic<std::size_t>> m_m52Progress;
    QTimer* m_m52ProgressTimer=nullptr;
    bool m_m52Discard=false;
    QPushButton* m_m52RunButton=nullptr;
    QPushButton* m_m52CancelButton=nullptr;
    QPushButton* m_m52ClearButton=nullptr;
    QPushButton* m_m52ExportButton=nullptr;
    QTableWidget* m_m52Heatmap=nullptr;
    QPlainTextEdit* m_m52Info=nullptr;
    QComboBox* m_m52ViewMode=nullptr;

    std::shared_ptr<mls::twin::TwinM6Data> m_m6Data;
    QFutureWatcher<std::shared_ptr<mls::twin::TwinM6Data>>* m_m6Watcher=nullptr;
    QFutureWatcher<QString>* m_m6ExportWatcher=nullptr;
    std::shared_ptr<std::atomic<bool>> m_m6Cancel;
    std::shared_ptr<std::atomic<bool>> m_m6ExportCancel;
    std::shared_ptr<mls::twin::M6ExportProgress> m_m6ExportProgress;
    std::shared_ptr<std::atomic<std::size_t>> m_m6Progress;
    std::shared_ptr<mls::twin::M6ProgressState> m_m6DetailedProgress;
    QElapsedTimer m_m6Elapsed;
    QTimer* m_m6ProgressTimer=nullptr;
    bool m_m6Discard=false;
    QPushButton* m_m6RunButton=nullptr;
    QPushButton* m_m6CancelButton=nullptr;
    QPushButton* m_m6ClearButton=nullptr;
    QPushButton* m_m6ExportButton=nullptr;
    QDoubleSpinBox* m_m6Hz=nullptr;
    QDoubleSpinBox* m_m6Tilt=nullptr;
    QDoubleSpinBox* m_m6Yaw=nullptr;
    QSpinBox* m_m6Segments=nullptr;
    QSpinBox* m_m6ExportStride=nullptr;
    QCheckBox* m_m6AlsoPly=nullptr;
    QProgressBar* m_m6ProgressBar=nullptr; // candidate coarse/refine
    QProgressBar* m_m6SegmentsProgressBar=nullptr; // completed/max segments
    QProgressBar* m_m6CurrentProgressBar=nullptr; // active segment frames
    QProgressBar* m_m6ArchiveProgressBar=nullptr; // archived volumes / export copy
    QCheckBox* m_m6SkipFcl=nullptr;
    QPlainTextEdit* m_m6Info=nullptr;
    std::shared_ptr<mls::twin::TwinM7Data> m_m7Data;
    QFutureWatcher<std::shared_ptr<mls::twin::TwinM7Data>>* m_m7Watcher=nullptr;
    QFutureWatcher<QString>* m_m7ExportWatcher=nullptr;
    std::shared_ptr<std::atomic<bool>> m_m7Cancel;
    std::shared_ptr<mls::twin::M7Progress> m_m7Progress;
    bool m_m7Discard=false;
    QTimer* m_m7ProgressTimer=nullptr;
    QPushButton* m_m7RunButton=nullptr;
    QPushButton* m_m7CancelButton=nullptr;
    QPushButton* m_m7ClearButton=nullptr;
    QPushButton* m_m7ExportButton=nullptr;
    QDoubleSpinBox* m_m7VoxelPitch=nullptr;
    QDoubleSpinBox* m_m7Goal=nullptr;
    QSpinBox* m_m7VoxelCap=nullptr;
    QProgressBar* m_m7ProgressBar=nullptr;
    QPlainTextEdit* m_m7Info=nullptr;
    QTableWidget* m_m7FeatureTable=nullptr;
    QTableWidget* m_m7SegmentTable=nullptr;
    QHash<QString,QCheckBox*> m_twinLayerChecks;
    QTableWidget* m_twinFeatureTable=nullptr;
    QLabel* m_twinFeatureSummary=nullptr;
    QComboBox* m_twinSegmentPicker=nullptr;
    QSlider* m_twinPoseSlider=nullptr;
    QLabel* m_twinPoseInfo=nullptr;
    QPushButton* m_twinExportTargetsButton=nullptr;
    QFutureWatcher<QString>* m_twinTargetsExportWatcher=nullptr;
    // M8 workflow UI and guarded state machine.
    bool m_m8Running=false;
    bool m_m8Cancelling=false;
    bool m_m8Completed=false;
    std::size_t m_m8Index=0;
    mls::twin::M8WorkflowOptions m_m8Options{};
    std::vector<mls::twin::M8Stage> m_m8Sequence;
    std::vector<QString> m_m8StepStates;
    mls::twin::M8Acceptance m_m8Acceptance{};
    QString m_m8CadFile;
    QString m_m8RealScanFile;
    mls::twin::Pose6 m_m8PoseAtStart{};
    std::shared_ptr<mls::twin::ScanReplayData> m_m8M2AtStart;
    QCheckBox* m_m8IncludeM3=nullptr;
    QCheckBox* m_m8IncludeM4=nullptr;
    QCheckBox* m_m8IncludeM51=nullptr;
    QCheckBox* m_m8IncludeM52=nullptr;
    QCheckBox* m_m8Reuse=nullptr;
    QPushButton* m_m8PreflightButton=nullptr;
    QPushButton* m_m8RunButton=nullptr;
    QPushButton* m_m8CancelButton=nullptr;
    QPushButton* m_m8ExportButton=nullptr;
    QProgressBar* m_m8ProgressBar=nullptr;
    QTableWidget* m_m8StageTable=nullptr;
    QPlainTextEdit* m_m8Info=nullptr;
    QTimer* m_m8Timer=nullptr;
};
