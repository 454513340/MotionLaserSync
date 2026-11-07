#pragma once

/**
 * @file PointCloudProcessWidget.h
 * @brief 点云处理控制面板 - 深度学习与目标检测
 */

#include <QWidget>
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
#include <QListWidget>
#include <QTabWidget>
#include <QLineEdit>
#include <memory>

class PointCloudPreprocessor;
class PointCloudSegmenter;
class DeepLearningDetector;
class PointCloudBuilder;

class PointCloudProcessWidget : public QWidget
{
    Q_OBJECT

public:
    explicit PointCloudProcessWidget(QWidget* parent = nullptr);
    ~PointCloudProcessWidget() override;

    void setPointCloudBuilder(PointCloudBuilder* builder) { m_cloudBuilder = builder; }

signals:
    void logMessage(const QString& message);
    void detectionCompleted(int objectCount);

private slots:
    // 预处理
    void onApplyFilter();
    void onDownsample();
    void onEstimateNormals();
    
    // 分割
    void onSegmentCloud();
    
    // 深度学习
    void onLoadModel();
    void onRunInference();
    void onExportResults();
    
    // 测试
    void onTestPreprocess();
    void onTestSegmentation();
    void onTestDetection();
    
    void onProgressUpdated(int current, int total, const QString& message);
    void onProcessingCompleted(bool success);

private:
    void setupUI();
    void setupPreprocessTab(QWidget* parent);
    void setupSegmentTab(QWidget* parent);
    void setupDeepLearningTab(QWidget* parent);
    void connectSignals();
    void updateUIState();
    void appendLog(const QString& message);
    void updateDetectionResults();

private:
    PointCloudBuilder* m_cloudBuilder = nullptr;
    std::unique_ptr<PointCloudPreprocessor> m_preprocessor;
    std::unique_ptr<PointCloudSegmenter> m_segmenter;
    std::unique_ptr<DeepLearningDetector> m_detector;

    QTabWidget* m_tabWidget = nullptr;

    // 预处理UI
    QComboBox* m_comboFilterType = nullptr;
    QSpinBox* m_spinStatK = nullptr;
    QDoubleSpinBox* m_spinStatStdMul = nullptr;
    QDoubleSpinBox* m_spinRadius = nullptr;
    QSpinBox* m_spinMinNeighbors = nullptr;
    QDoubleSpinBox* m_spinVoxelSize = nullptr;
    QPushButton* m_btnApplyFilter = nullptr;
    QPushButton* m_btnDownsample = nullptr;
    QPushButton* m_btnEstimateNormals = nullptr;
    QPushButton* m_btnTestPreprocess = nullptr;
    QLabel* m_lblPreprocessResult = nullptr;

    // 分割UI
    QComboBox* m_comboSegmentMethod = nullptr;
    QDoubleSpinBox* m_spinClusterTolerance = nullptr;
    QSpinBox* m_spinMinClusterSize = nullptr;
    QSpinBox* m_spinMaxClusterSize = nullptr;
    QPushButton* m_btnSegment = nullptr;
    QPushButton* m_btnTestSegment = nullptr;
    QTableWidget* m_segmentTable = nullptr;
    QLabel* m_lblSegmentResult = nullptr;

    // 深度学习UI
    QComboBox* m_comboNetworkType = nullptr;
    QLineEdit* m_editModelPath = nullptr;
    QPushButton* m_btnBrowseModel = nullptr;
    QPushButton* m_btnLoadModel = nullptr;
    QDoubleSpinBox* m_spinConfThreshold = nullptr;
    QPushButton* m_btnRunInference = nullptr;
    QPushButton* m_btnExportResults = nullptr;
    QPushButton* m_btnTestDetection = nullptr;
    QTableWidget* m_detectionTable = nullptr;
    QLabel* m_lblModelStatus = nullptr;
    QLabel* m_lblInferenceTime = nullptr;

    // 通用UI
    QTextEdit* m_logView = nullptr;
    QProgressBar* m_progressBar = nullptr;
    QLabel* m_statusLabel = nullptr;

    bool m_isProcessing = false;
};
