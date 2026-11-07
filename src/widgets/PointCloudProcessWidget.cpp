#include "widgets/PointCloudProcessWidget.h"
#include "algorithms/PointCloudProcessingAlgorithms.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QFileDialog>
#include <QDateTime>
#include <QHeaderView>
#include <QLineEdit>
#include <QTabWidget>

PointCloudProcessWidget::PointCloudProcessWidget(QWidget* parent)
    : QWidget(parent)
    , m_preprocessor(std::make_unique<PointCloudPreprocessor>())
    , m_segmenter(std::make_unique<PointCloudSegmenter>())
    , m_detector(std::make_unique<DeepLearningDetector>())
{
    setupUI();
    connectSignals();
    updateUIState();
}

PointCloudProcessWidget::~PointCloudProcessWidget() = default;

void PointCloudProcessWidget::setupUI()
{
    auto* mainLayout = new QVBoxLayout(this);
    m_tabWidget = new QTabWidget(this);
    
    auto* preprocessTab = new QWidget();
    setupPreprocessTab(preprocessTab);
    m_tabWidget->addTab(preprocessTab, tr("预处理"));
    
    auto* segmentTab = new QWidget();
    setupSegmentTab(segmentTab);
    m_tabWidget->addTab(segmentTab, tr("分割"));
    
    auto* dlTab = new QWidget();
    setupDeepLearningTab(dlTab);
    m_tabWidget->addTab(dlTab, tr("深度学习检测"));
    
    mainLayout->addWidget(m_tabWidget, 1);

    auto* statusLayout = new QHBoxLayout;
    m_statusLabel = new QLabel(tr("就绪"));
    m_progressBar = new QProgressBar;
    statusLayout->addWidget(m_statusLabel);
    statusLayout->addWidget(m_progressBar, 1);
    mainLayout->addLayout(statusLayout);

    auto* logGroup = new QGroupBox(tr("处理日志"));
    auto* logLayout = new QVBoxLayout(logGroup);
    m_logView = new QTextEdit;
    m_logView->setReadOnly(true);
    m_logView->setMaximumHeight(80);
    logLayout->addWidget(m_logView);
    mainLayout->addWidget(logGroup);
}

void PointCloudProcessWidget::setupPreprocessTab(QWidget* parent)
{
    auto* layout = new QVBoxLayout(parent);
    auto* filterGroup = new QGroupBox(tr("滤波设置"));
    auto* filterLayout = new QGridLayout(filterGroup);
    
    filterLayout->addWidget(new QLabel(tr("滤波类型:")), 0, 0);
    m_comboFilterType = new QComboBox;
    m_comboFilterType->addItem(tr("统计离群点"), 0);
    m_comboFilterType->addItem(tr("半径离群点"), 1);
    m_comboFilterType->addItem(tr("直通滤波"), 2);
    m_comboFilterType->addItem(tr("体素降采样"), 3);
    filterLayout->addWidget(m_comboFilterType, 0, 1);
    
    filterLayout->addWidget(new QLabel(tr("K近邻数:")), 1, 0);
    m_spinStatK = new QSpinBox; m_spinStatK->setRange(1, 200); m_spinStatK->setValue(50);
    filterLayout->addWidget(m_spinStatK, 1, 1);
    
    filterLayout->addWidget(new QLabel(tr("标准差倍数:")), 1, 2);
    m_spinStatStdMul = new QDoubleSpinBox; m_spinStatStdMul->setRange(0.1, 10); m_spinStatStdMul->setValue(1.0);
    filterLayout->addWidget(m_spinStatStdMul, 1, 3);
    
    filterLayout->addWidget(new QLabel(tr("搜索半径:")), 2, 0);
    m_spinRadius = new QDoubleSpinBox; m_spinRadius->setRange(0.1, 100); m_spinRadius->setValue(1.0);
    filterLayout->addWidget(m_spinRadius, 2, 1);
    
    filterLayout->addWidget(new QLabel(tr("体素大小:")), 2, 2);
    m_spinVoxelSize = new QDoubleSpinBox; m_spinVoxelSize->setRange(0.1, 50); m_spinVoxelSize->setValue(1.0);
    filterLayout->addWidget(m_spinVoxelSize, 2, 3);
    
    m_spinMinNeighbors = new QSpinBox; m_spinMinNeighbors->setValue(5);
    layout->addWidget(filterGroup);

    auto* ctrlLayout = new QHBoxLayout;
    m_btnApplyFilter = new QPushButton(tr("应用滤波"));
    m_btnApplyFilter->setStyleSheet("background-color: #4CAF50; color: white;");
    m_btnDownsample = new QPushButton(tr("降采样"));
    m_btnEstimateNormals = new QPushButton(tr("估计法向量"));
    m_btnTestPreprocess = new QPushButton(tr("测试"));
    m_btnTestPreprocess->setStyleSheet("background-color: #2196F3; color: white;");
    ctrlLayout->addWidget(m_btnApplyFilter);
    ctrlLayout->addWidget(m_btnDownsample);
    ctrlLayout->addWidget(m_btnEstimateNormals);
    ctrlLayout->addStretch();
    ctrlLayout->addWidget(m_btnTestPreprocess);
    layout->addLayout(ctrlLayout);

    m_lblPreprocessResult = new QLabel(tr("输入: 0 | 输出: 0 | 移除: 0"));
    layout->addWidget(m_lblPreprocessResult);
    layout->addStretch();
}

void PointCloudProcessWidget::setupSegmentTab(QWidget* parent)
{
    auto* layout = new QVBoxLayout(parent);
    auto* segGroup = new QGroupBox(tr("分割设置"));
    auto* segLayout = new QGridLayout(segGroup);
    
    segLayout->addWidget(new QLabel(tr("分割方法:")), 0, 0);
    m_comboSegmentMethod = new QComboBox;
    m_comboSegmentMethod->addItem(tr("欧氏聚类"), 0);
    m_comboSegmentMethod->addItem(tr("区域生长"), 1);
    m_comboSegmentMethod->addItem(tr("RANSAC"), 2);
    segLayout->addWidget(m_comboSegmentMethod, 0, 1);
    
    segLayout->addWidget(new QLabel(tr("聚类容差:")), 1, 0);
    m_spinClusterTolerance = new QDoubleSpinBox; m_spinClusterTolerance->setRange(0.1, 100); m_spinClusterTolerance->setValue(2.0);
    segLayout->addWidget(m_spinClusterTolerance, 1, 1);
    
    segLayout->addWidget(new QLabel(tr("最小聚类:")), 2, 0);
    m_spinMinClusterSize = new QSpinBox; m_spinMinClusterSize->setRange(1, 100000); m_spinMinClusterSize->setValue(100);
    segLayout->addWidget(m_spinMinClusterSize, 2, 1);
    
    segLayout->addWidget(new QLabel(tr("最大聚类:")), 2, 2);
    m_spinMaxClusterSize = new QSpinBox; m_spinMaxClusterSize->setRange(1, 10000000); m_spinMaxClusterSize->setValue(100000);
    segLayout->addWidget(m_spinMaxClusterSize, 2, 3);
    layout->addWidget(segGroup);

    auto* ctrlLayout = new QHBoxLayout;
    m_btnSegment = new QPushButton(tr("执行分割"));
    m_btnSegment->setStyleSheet("background-color: #4CAF50; color: white;");
    m_btnTestSegment = new QPushButton(tr("测试"));
    m_btnTestSegment->setStyleSheet("background-color: #2196F3; color: white;");
    ctrlLayout->addWidget(m_btnSegment);
    ctrlLayout->addStretch();
    ctrlLayout->addWidget(m_btnTestSegment);
    layout->addLayout(ctrlLayout);

    auto* resultGroup = new QGroupBox(tr("分割结果"));
    auto* resultLayout = new QVBoxLayout(resultGroup);
    m_segmentTable = new QTableWidget(0, 4);
    m_segmentTable->setHorizontalHeaderLabels({tr("ID"), tr("点数"), tr("质心"), tr("颜色")});
    m_segmentTable->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    m_segmentTable->setMaximumHeight(120);
    resultLayout->addWidget(m_segmentTable);
    m_lblSegmentResult = new QLabel(tr("聚类: 0"));
    resultLayout->addWidget(m_lblSegmentResult);
    layout->addWidget(resultGroup);
    layout->addStretch();
}

void PointCloudProcessWidget::setupDeepLearningTab(QWidget* parent)
{
    auto* layout = new QVBoxLayout(parent);
    auto* modelGroup = new QGroupBox(tr("模型设置"));
    auto* modelLayout = new QGridLayout(modelGroup);
    
    modelLayout->addWidget(new QLabel(tr("网络类型:")), 0, 0);
    m_comboNetworkType = new QComboBox;
    m_comboNetworkType->addItem(tr("PointNet"), 0);
    m_comboNetworkType->addItem(tr("PointNet++"), 1);
    m_comboNetworkType->addItem(tr("DGCNN"), 2);
    modelLayout->addWidget(m_comboNetworkType, 0, 1);
    
    modelLayout->addWidget(new QLabel(tr("模型路径:")), 1, 0);
    m_editModelPath = new QLineEdit;
    modelLayout->addWidget(m_editModelPath, 1, 1, 1, 2);
    m_btnBrowseModel = new QPushButton(tr("浏览"));
    modelLayout->addWidget(m_btnBrowseModel, 1, 3);
    
    m_btnLoadModel = new QPushButton(tr("加载模型"));
    modelLayout->addWidget(m_btnLoadModel, 2, 0);
    m_lblModelStatus = new QLabel(tr("模型: 未加载"));
    modelLayout->addWidget(m_lblModelStatus, 2, 1, 1, 3);
    
    modelLayout->addWidget(new QLabel(tr("置信度阈值:")), 3, 0);
    m_spinConfThreshold = new QDoubleSpinBox; m_spinConfThreshold->setRange(0, 1); m_spinConfThreshold->setValue(0.5);
    modelLayout->addWidget(m_spinConfThreshold, 3, 1);
    layout->addWidget(modelGroup);

    auto* ctrlLayout = new QHBoxLayout;
    m_btnRunInference = new QPushButton(tr("运行推理"));
    m_btnRunInference->setStyleSheet("background-color: #4CAF50; color: white;");
    m_btnExportResults = new QPushButton(tr("导出结果"));
    m_btnTestDetection = new QPushButton(tr("测试"));
    m_btnTestDetection->setStyleSheet("background-color: #2196F3; color: white;");
    ctrlLayout->addWidget(m_btnRunInference);
    ctrlLayout->addWidget(m_btnExportResults);
    ctrlLayout->addStretch();
    ctrlLayout->addWidget(m_btnTestDetection);
    layout->addLayout(ctrlLayout);

    auto* resultGroup = new QGroupBox(tr("检测结果"));
    auto* resultLayout = new QVBoxLayout(resultGroup);
    m_detectionTable = new QTableWidget(0, 5);
    m_detectionTable->setHorizontalHeaderLabels({tr("ID"), tr("类别"), tr("置信度"), tr("位置"), tr("点数")});
    m_detectionTable->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    m_detectionTable->setMaximumHeight(120);
    resultLayout->addWidget(m_detectionTable);
    m_lblInferenceTime = new QLabel(tr("检测: 0 | 时间: 0ms"));
    resultLayout->addWidget(m_lblInferenceTime);
    layout->addWidget(resultGroup);
    layout->addStretch();
}

void PointCloudProcessWidget::connectSignals()
{
    connect(m_btnApplyFilter, &QPushButton::clicked, this, &PointCloudProcessWidget::onApplyFilter);
    connect(m_btnDownsample, &QPushButton::clicked, this, &PointCloudProcessWidget::onDownsample);
    connect(m_btnTestPreprocess, &QPushButton::clicked, this, &PointCloudProcessWidget::onTestPreprocess);
    connect(m_btnSegment, &QPushButton::clicked, this, &PointCloudProcessWidget::onSegmentCloud);
    connect(m_btnTestSegment, &QPushButton::clicked, this, &PointCloudProcessWidget::onTestSegmentation);
    connect(m_btnBrowseModel, &QPushButton::clicked, this, [this]() {
        QString file = QFileDialog::getOpenFileName(this, tr("选择模型"), QString(), tr("ONNX (*.onnx)"));
        if (!file.isEmpty()) m_editModelPath->setText(file);
    });
    connect(m_btnLoadModel, &QPushButton::clicked, this, &PointCloudProcessWidget::onLoadModel);
    connect(m_btnRunInference, &QPushButton::clicked, this, &PointCloudProcessWidget::onRunInference);
    connect(m_btnTestDetection, &QPushButton::clicked, this, &PointCloudProcessWidget::onTestDetection);
    connect(m_preprocessor.get(), &IAlgorithm::logMessage, this, &PointCloudProcessWidget::appendLog);
    connect(m_detector.get(), &IAlgorithm::logMessage, this, &PointCloudProcessWidget::appendLog);
}

void PointCloudProcessWidget::updateUIState()
{
    m_btnApplyFilter->setEnabled(!m_isProcessing);
    m_btnRunInference->setEnabled(!m_isProcessing && m_detector->isModelLoaded());
}

void PointCloudProcessWidget::appendLog(const QString& msg)
{
    m_logView->append(QString("[%1] %2").arg(QDateTime::currentDateTime().toString("hh:mm:ss"), msg));
}

void PointCloudProcessWidget::onProgressUpdated(int c, int t, const QString& m)
{
    m_progressBar->setMaximum(t); m_progressBar->setValue(c); m_statusLabel->setText(m);
}

void PointCloudProcessWidget::onProcessingCompleted(bool) { m_isProcessing = false; updateUIState(); }

void PointCloudProcessWidget::onApplyFilter()
{
    m_preprocessor->setFilterType(static_cast<PointCloudPreprocessor::FilterType>(m_comboFilterType->currentIndex()));
    m_isProcessing = true;
    appendLog(tr("应用滤波..."));
    m_preprocessor->initialize();
    m_preprocessor->execute();
}

void PointCloudProcessWidget::onDownsample() { appendLog(tr("降采样...")); }
void PointCloudProcessWidget::onEstimateNormals() { appendLog(tr("估计法向量...")); }

void PointCloudProcessWidget::onSegmentCloud()
{
    m_segmenter->setMethod(static_cast<PointCloudSegmenter::SegmentMethod>(m_comboSegmentMethod->currentIndex()));
    m_segmenter->setClusterTolerance(m_spinClusterTolerance->value());
    appendLog(tr("执行分割..."));
    m_segmenter->initialize();
    m_segmenter->execute();
    const auto& r = m_segmenter->result();
    m_segmentTable->setRowCount(r.segmentCount);
    for (int i = 0; i < r.segmentCount; ++i) {
        m_segmentTable->setItem(i, 0, new QTableWidgetItem(QString::number(i)));
        m_segmentTable->setItem(i, 1, new QTableWidgetItem(QString::number(r.segments[i].size())));
        m_segmentTable->setItem(i, 2, new QTableWidgetItem(QString("(%1,%2,%3)").arg(r.centroids[i].x(),0,'f',1).arg(r.centroids[i].y(),0,'f',1).arg(r.centroids[i].z(),0,'f',1)));
        m_segmentTable->setItem(i, 3, new QTableWidgetItem("-"));
    }
    m_lblSegmentResult->setText(tr("聚类: %1").arg(r.segmentCount));
}

void PointCloudProcessWidget::onLoadModel()
{
    if (m_detector->loadModel(m_editModelPath->text().isEmpty() ? "test.onnx" : m_editModelPath->text())) {
        m_lblModelStatus->setText(tr("模型: 已加载")); m_lblModelStatus->setStyleSheet("color:green;");
    }
    updateUIState();
}

void PointCloudProcessWidget::onRunInference()
{
    m_detector->setConfidenceThreshold(m_spinConfThreshold->value());
    appendLog(tr("运行推理..."));
    m_detector->initialize();
    m_detector->execute();
    updateDetectionResults();
}

void PointCloudProcessWidget::updateDetectionResults()
{
    const auto& r = m_detector->result();
    m_detectionTable->setRowCount(r.detections.size());
    for (int i = 0; i < r.detections.size(); ++i) {
        const auto& d = r.detections[i];
        m_detectionTable->setItem(i, 0, new QTableWidgetItem(QString::number(i)));
        m_detectionTable->setItem(i, 1, new QTableWidgetItem(d.className));
        m_detectionTable->setItem(i, 2, new QTableWidgetItem(QString::number(d.confidence, 'f', 3)));
        m_detectionTable->setItem(i, 3, new QTableWidgetItem(QString("(%1,%2,%3)").arg(d.centroid.x(),0,'f',1).arg(d.centroid.y(),0,'f',1).arg(d.centroid.z(),0,'f',1)));
        m_detectionTable->setItem(i, 4, new QTableWidgetItem(QString::number(d.pointIndices.size())));
    }
    m_lblInferenceTime->setText(tr("检测: %1 | 时间: %2ms").arg(r.detections.size()).arg(r.inferenceTime, 0, 'f', 1));
}

void PointCloudProcessWidget::onExportResults() { appendLog(tr("导出结果...")); }

void PointCloudProcessWidget::onTestPreprocess()
{
    appendLog(tr("=== 测试预处理 ==="));
    QVector<QVector3D> testCloud;
    for (int i = 0; i < 1000; ++i) testCloud.append(QVector3D(rand()%100, rand()%100, rand()%50));
    for (int i = 0; i < 50; ++i) testCloud.append(QVector3D(rand()%100+200, rand()%100+200, rand()%50+100));
    m_preprocessor->setInputCloud(testCloud);
    m_preprocessor->initialize();
    m_preprocessor->execute();
    m_lblPreprocessResult->setText(tr("输入: %1 | 输出: %2 | 移除: %3").arg(testCloud.size()).arg(m_preprocessor->outputCloud().size()).arg(m_preprocessor->removedPointCount()));
}

void PointCloudProcessWidget::onTestSegmentation()
{
    appendLog(tr("=== 测试分割 ==="));
    QVector<QVector3D> testCloud;
    for (int i = 0; i < 200; ++i) testCloud.append(QVector3D(25+(rand()%20-10), 25+(rand()%20-10), 25+(rand()%10-5)));
    for (int i = 0; i < 200; ++i) testCloud.append(QVector3D(75+(rand()%20-10), 75+(rand()%20-10), 25+(rand()%10-5)));
    m_segmenter->setInputCloud(testCloud);
    m_segmenter->setClusterTolerance(15.0);
    m_segmenter->initialize();
    m_segmenter->execute();
    onSegmentCloud();
}

void PointCloudProcessWidget::onTestDetection()
{
    appendLog(tr("=== 测试深度学习检测 ==="));
    m_detector->loadModel("test_model.onnx");
    m_lblModelStatus->setText(tr("模型: 测试模式")); m_lblModelStatus->setStyleSheet("color:green;");
    m_detector->setConfidenceThreshold(m_spinConfThreshold->value());
    m_detector->runTestInference();
    updateDetectionResults();
    updateUIState();
}
