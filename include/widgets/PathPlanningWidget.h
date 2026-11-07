#pragma once

/**
 * @file PathPlanningWidget.h
 * @brief 路径规划控制面板
 */

#include <QWidget>
#include <QByteArray>
#include <QGroupBox>
#include <QPushButton>
#include <QLabel>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QComboBox>
#include <QTextEdit>
#include <QProgressBar>
#include <QTableWidget>
#include <QCheckBox>
#include <memory>
#include <QPointer>
#include "twin/MultiPoseExperimentQtAdapter.h"

class PathPlanner;
class GCodeGenerator;
class PathOptimizer;
class PointCloudBuilder;
class DigitalTwinScanWidget;
class MultiPoseExperimentWidget;
class QDialog;
class PointCloudWidget;

class PathPlanningWidget : public QWidget
{
    Q_OBJECT

public:
    explicit PathPlanningWidget(QWidget* parent = nullptr);
    ~PathPlanningWidget() override;

    void setPointCloudBuilder(PointCloudBuilder* builder) { m_cloudBuilder = builder; }
    void setMainPointCloudView(PointCloudWidget* view);
    bool snapshotOpticalForExactNc(const QString& text,mls::twin::TwinScene& scene,
        std::shared_ptr<const mls::twin::MultiPoseQtReport>& report,QString* error) const;
    bool snapshotCollisionWorkpiece(mls::twin::TwinScene& scene,
        QByteArray& identity, bool& offlinePoseReviewed, QString* error) const;

signals:
    void logMessage(const QString& message);
    void pathGenerated(const QString& gcode);
    void requestExecuteGCode(const QString& gcode);

private slots:
    void onGeneratePath();
    void onOptimizePath();
    void onGenerateGCode();
    void onExportGCode();
    void onPreviewPath();
    void onLoadPointCloud();
    void onTestPathPlanning();
    void onOpenDigitalTwin();
    void onOpenMultiPoseExperiment();
    void onProgressUpdated(int current, int total, const QString& message);
    void onPlanningCompleted(bool success);

private:
    void setupUI();
    void connectSignals();
    void updateUIState();
    void appendLog(const QString& message);
    void updatePathStatistics();

private:
    PointCloudBuilder* m_cloudBuilder = nullptr;
    std::unique_ptr<PathPlanner> m_pathPlanner;
    std::unique_ptr<GCodeGenerator> m_gcodeGenerator;
    std::unique_ptr<PathOptimizer> m_pathOptimizer;

    // UI组件
    QComboBox* m_comboPathType = nullptr;
    QComboBox* m_comboDirection = nullptr;
    QDoubleSpinBox* m_spinLineSpacing = nullptr;
    QDoubleSpinBox* m_spinPointSpacing = nullptr;
    QDoubleSpinBox* m_spinScanSpeed = nullptr;
    QDoubleSpinBox* m_spinRapidSpeed = nullptr;
    QDoubleSpinBox* m_spinSafeHeight = nullptr;
    QDoubleSpinBox* m_spinNominalHeight = nullptr;
    QCheckBox* m_chkAdaptiveHeight = nullptr;
    
    QDoubleSpinBox* m_spinBoundMinX = nullptr;
    QDoubleSpinBox* m_spinBoundMinY = nullptr;
    QDoubleSpinBox* m_spinBoundMaxX = nullptr;
    QDoubleSpinBox* m_spinBoundMaxY = nullptr;
    QPushButton* m_btnAutobound = nullptr;
    
    QPushButton* m_btnGeneratePath = nullptr;
    QPushButton* m_btnOptimize = nullptr;
    QPushButton* m_btnGenerateGCode = nullptr;
    QPushButton* m_btnExportGCode = nullptr;
    QPushButton* m_btnPreview = nullptr;
    QPushButton* m_btnLoadCloud = nullptr;
    QPushButton* m_btnTest = nullptr;
    QPushButton* m_btnDigitalTwin = nullptr;
    QPushButton* m_btnMultiPoseExperiment = nullptr;
    QPointer<QDialog> m_multiPoseExperimentDialog;
    QPointer<MultiPoseExperimentWidget> m_multiPoseExperimentWindow;
    QPointer<DigitalTwinScanWidget> m_digitalTwinWindow;
    QPointer<PointCloudWidget> m_mainCloudView;
    
    QLabel* m_lblStatistics = nullptr;
    QTextEdit* m_gcodePreview = nullptr;
    QTextEdit* m_logView = nullptr;
    QProgressBar* m_progressBar = nullptr;
    QLabel* m_statusLabel = nullptr;

    bool m_isProcessing = false;
};
