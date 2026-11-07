#pragma once
// Dedicated M3/M4 multi-orientation experiment recipe editor, wholly offline.
// NOT the M0-M8 coverage optimizer; never sends motor or laser instructions.
#include <QWidget>
#include <QByteArray>
#include <QPointer>
#include <QFutureWatcher>
#include <array>
#include <cstdint>
#include <limits>
#include <atomic>
#include <memory>
#include <map>
#include <vector>
#include <string>
#include "twin/TwinScene.h"
#include "twin/MultiPoseExperimentQtAdapter.h"
#include "twin/MultiPoseRegistrationAssist.h"
#include "twin/MultiPosePlayback.h"
#include <QElapsedTimer>
class PointCloudWidget;
class QLabel;
class QPushButton;
class QDoubleSpinBox;
class QComboBox;
class QSpinBox;
class QTableWidget;
class QPlainTextEdit;
class QProgressBar;
class QCheckBox;
class QTimer;
class QSlider;

class MultiPoseExperimentWidget final : public QWidget {
    Q_OBJECT
public:
    explicit MultiPoseExperimentWidget(QWidget* parent=nullptr);
    ~MultiPoseExperimentWidget() override;
    void setMainPointCloudView(PointCloudWidget* view);
    // Immutable offline geometry snapshot; no device commands or authorization.
    bool snapshotOpticalForExactNc(const QString& text,mls::twin::TwinScene& scene,
        std::shared_ptr<const mls::twin::MultiPoseQtReport>& report,QString* error) const;
    // Unlike the NC-specific optical report, this returns the currently
    // imported STEP and its CAD->MCS placement for full-NC collision diagnosis.
    // It does NOT claim independently certified registration or NC identity.
    bool snapshotCollisionWorkpiece(mls::twin::TwinScene& scene,
        QByteArray& identity, bool& offlinePoseReviewed, QString* error) const;
private:
    void buildUi();
    void importStep();
    bool loadStep(const QString& file);
    void importRealScan();
    void scanLoaded();
    void loadTwinSession();
    void startRoughRegistration();
    void roughRegistrationFinished();
    void applyRoughRegistration();
    void acceptManualPreviewPose();
    void addOrientation(double c=0,double b=0,const QString& name={});
    void restoreStandardMatrix();
    void removeOrientation();
    void startPlan();
    void planFinished();
    void exportNc();
    void previewNc();
    void importNcForPose();
    bool rebuildNcReplays(bool notify=true);
    void startFullSimulation();
    void simulationFinished();
    void startPlayback();
    void togglePlaybackPause();
    void stepPlayback();
    void stopPlayback(bool clearCloud=false);
    void playbackTick();
    void seekPlayback(int percent);
    void updatePlaybackFrame(bool forceReset=false);
    mls::twin::MultiPosePlaybackState sampleCurrentNcPlayback(double time) const;
    void startCompare();
    void comparisonFinished();
    void cancelTask();
    void invalidate(bool placementChanged=false);
    void syncScene(bool focus=false);
    void showResult();
    void log(const QString& message);
    mls::twin::Pose6 uiCadPose() const;
    bool readRecipe(mls::twin::ExperimentOptions& opt,
                    std::vector<mls::twin::ExperimentPose>& orientations,
                    QString* error) const;
    QPointer<PointCloudWidget> m_view;
    mls::twin::TwinScene m_scene;
    QString m_cadFile;
    QByteArray m_cadSha256;
    std::shared_ptr<mls::twin::ScanReplayData> m_real;
    std::shared_ptr<mls::twin::MultiPoseQtReport> m_report;
    std::array<QDoubleSpinBox*,6> m_pose{};
    std::array<QDoubleSpinBox*,5> m_roi{};
    QDoubleSpinBox *m_speed=nullptr,*m_rate=nullptr,*m_transferSpeed=nullptr,
        *m_pitch=nullptr,*m_retract=nullptr,*m_focus=nullptr,*m_leadIn=nullptr,*m_leadOut=nullptr;
    QComboBox *m_axis=nullptr,*m_h3Edge=nullptr,*m_previewPose=nullptr,*m_previewTopology=nullptr;
    QSpinBox* m_maxProfiles=nullptr;
    QTableWidget* m_poses=nullptr;
    QLabel *m_inputs=nullptr,*m_summary=nullptr,*m_registrationStatus=nullptr;
    QPushButton *m_btnPlan=nullptr,*m_btnExport=nullptr,*m_btnSim=nullptr,*m_btnCancel=nullptr,
                *m_btnEstimate=nullptr,*m_btnApplyEstimate=nullptr,*m_btnManualPreview=nullptr,
                *m_btnPreviewNc=nullptr;
    QPlainTextEdit* m_log=nullptr;
    QPlainTextEdit* m_ncText=nullptr;
    QPushButton* m_btnImportNc=nullptr;
    std::vector<std::string> m_activeNcTexts;
    std::vector<mls::twin::NcSimResult> m_activeNcReplays;
    mls::twin::NcSimResult m_connectedNcReplay;
    std::string m_connectedNcText;
    bool m_connectedNcUsable=false;
    std::map<std::size_t,std::string> m_overriddenNcByPose;
    std::map<int,std::string> m_overriddenConnectedNcByTopology;
    QProgressBar* m_progress=nullptr;
    QTimer* m_playTimer=nullptr;
    QElapsedTimer m_playClock;
    QSlider* m_playSeek=nullptr;
    QComboBox* m_playSpeed=nullptr;
    QLabel* m_playStatus=nullptr;
    QPushButton *m_btnPlay=nullptr,*m_btnPause=nullptr,*m_btnStep=nullptr,*m_btnStop=nullptr,
                *m_btnCompare=nullptr;
    bool m_playing=false,m_playCurrentPoseOnly=false;
    double m_playSeconds=0.,m_playLimitSeconds=0.;
    std::size_t m_lastDrawnPose=std::numeric_limits<std::size_t>::max();
    std::uint64_t m_simRevision=0,m_compareRevision=0;
    std::vector<std::size_t> m_nextPlaybackFrame;
    mls::twin::MultiPosePlaybackTimeline m_playbackTimeline;
    std::shared_ptr<mls::twin::MultiPoseSimArtifacts> m_simArtifacts,m_simPending;
    QFutureWatcher<std::shared_ptr<mls::twin::MultiPoseCompareArtifacts>> m_compareWatcher;
    QTimer* m_progressTimer=nullptr;
    QFutureWatcher<std::shared_ptr<mls::twin::ScanReplayData>> m_realWatcher;
    QFutureWatcher<std::shared_ptr<mls::twin::RegistrationAssistReport>> m_regWatcher;
    QFutureWatcher<std::shared_ptr<mls::twin::MultiPoseQtReport>> m_planWatcher;
    QFutureWatcher<QString> m_simWatcher;
    std::shared_ptr<std::atomic<bool>> m_cancel;
    std::shared_ptr<std::atomic<std::size_t>> m_simProgress;
    std::shared_ptr<mls::twin::RegistrationAssistReport> m_registrationProposal;
    bool m_previewPoseAcknowledged=false; // user-reviewed for OFFLINE testing only
    std::uint64_t m_registrationRevision=0;
    QString m_simTarget;
    std::size_t m_simTotalProfiles=0;
    std::uint64_t m_recipeRevision=0, m_planningRevision=0;
};
