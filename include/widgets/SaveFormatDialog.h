#pragma once

#include <QDialog>
#include <QComboBox>
#include <QSpinBox>
#include <QLabel>
#include <QCheckBox>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QFormLayout>

#include "utils/CompactScanFormat.h"

/**
 * @brief 点云保存格式选择对话框
 * 
 * 允许用户选择保存格式和压缩选项
 */
class SaveFormatDialog : public QDialog
{
    Q_OBJECT

public:
    explicit SaveFormatDialog(QWidget* parent = nullptr);
    ~SaveFormatDialog() override = default;

    // 获取选择的格式
    PointCloudSaveFormat selectedFormat() const { return m_selectedFormat; }
    
    // 获取压缩级别 (1-9)
    int compressionLevel() const { return m_compressionLevel; }
    
    // 设置点数用于估算存储大小
    void setPointCount(uint64_t count);
    
    // 设置默认格式
    void setDefaultFormat(PointCloudSaveFormat format);

private slots:
    void onFormatChanged(int index);
    void updateStorageEstimate();

private:
    void setupUI();

    QComboBox* m_formatCombo = nullptr;
    QSpinBox* m_compressionLevelSpin = nullptr;
    QLabel* m_descriptionLabel = nullptr;
    QLabel* m_estimateLabel = nullptr;
    QLabel* m_compressionLabel = nullptr;
    
    PointCloudSaveFormat m_selectedFormat = PointCloudSaveFormat::SCAN_LZ4;
    int m_compressionLevel = 1;
    uint64_t m_pointCount = 0;
};

// ==================== 实现 ====================

inline SaveFormatDialog::SaveFormatDialog(QWidget* parent)
    : QDialog(parent)
{
    setWindowTitle(tr("选择保存格式"));
    setMinimumWidth(450);
    setupUI();
}

inline void SaveFormatDialog::setupUI()
{
    auto* mainLayout = new QVBoxLayout(this);
    
    // 格式选择
    auto* formatGroup = new QGroupBox(tr("保存格式"));
    auto* formatLayout = new QFormLayout(formatGroup);
    
    m_formatCombo = new QComboBox();
    m_formatCombo->addItem(tr("CSV 文本格式"), static_cast<int>(PointCloudSaveFormat::CSV));
    m_formatCombo->addItem(tr("PLY ASCII 格式"), static_cast<int>(PointCloudSaveFormat::PLY_ASCII));
    m_formatCombo->addItem(tr("PLY 二进制格式"), static_cast<int>(PointCloudSaveFormat::PLY_BINARY));
    m_formatCombo->addItem(tr("紧凑扫描格式 (无压缩)"), static_cast<int>(PointCloudSaveFormat::SCAN_UNCOMPRESSED));
    m_formatCombo->addItem(tr("五轴扫描格式 + Qt zlib (无损)"), static_cast<int>(PointCloudSaveFormat::SCAN_LZ4));
    m_formatCombo->addItem(tr("五轴扫描格式 + Qt zlib (旧选项兼容)"), static_cast<int>(PointCloudSaveFormat::SCAN_ZSTD));
    
    // 默认选择 LZ4
    m_formatCombo->setCurrentIndex(4);
    
    formatLayout->addRow(tr("格式:"), m_formatCombo);
    
    // 格式描述
    m_descriptionLabel = new QLabel();
    m_descriptionLabel->setWordWrap(true);
    m_descriptionLabel->setStyleSheet("color: #666; font-size: 11px;");
    formatLayout->addRow(m_descriptionLabel);
    
    mainLayout->addWidget(formatGroup);
    
    // 压缩选项
    auto* compressionGroup = new QGroupBox(tr("压缩选项"));
    auto* compressionLayout = new QFormLayout(compressionGroup);
    
    m_compressionLevelSpin = new QSpinBox();
    m_compressionLevelSpin->setRange(1, 9);
    m_compressionLevelSpin->setValue(1);
    m_compressionLevelSpin->setToolTip(tr("1 = 最快速度, 9 = 最高压缩比"));
    
    m_compressionLabel = new QLabel(tr("压缩级别:"));
    compressionLayout->addRow(m_compressionLabel, m_compressionLevelSpin);
    
    mainLayout->addWidget(compressionGroup);
    
    // 存储估算（压缩格式显示未压缩估算）
    auto* estimateGroup = new QGroupBox(tr("存储估算（压缩格式显示未压缩估算）"));
    auto* estimateLayout = new QVBoxLayout(estimateGroup);
    
    m_estimateLabel = new QLabel(tr("选择格式后显示估算大小"));
    m_estimateLabel->setStyleSheet("font-weight: bold;");
    estimateLayout->addWidget(m_estimateLabel);
    
    mainLayout->addWidget(estimateGroup);
    
    // 按钮
    auto* buttonLayout = new QHBoxLayout();
    buttonLayout->addStretch();
    
    auto* okButton = new QPushButton(tr("确定"));
    okButton->setDefault(true);
    connect(okButton, &QPushButton::clicked, this, &QDialog::accept);
    
    auto* cancelButton = new QPushButton(tr("取消"));
    connect(cancelButton, &QPushButton::clicked, this, &QDialog::reject);
    
    buttonLayout->addWidget(okButton);
    buttonLayout->addWidget(cancelButton);
    mainLayout->addLayout(buttonLayout);
    
    // 连接信号
    connect(m_formatCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &SaveFormatDialog::onFormatChanged);
    connect(m_compressionLevelSpin, QOverload<int>::of(&QSpinBox::valueChanged),
            this, [this](int value) {
                m_compressionLevel = value;
            });
    
    // 初始化
    onFormatChanged(m_formatCombo->currentIndex());
}

inline void SaveFormatDialog::onFormatChanged(int index)
{
    m_selectedFormat = static_cast<PointCloudSaveFormat>(m_formatCombo->itemData(index).toInt());
    
    // 更新描述
    m_descriptionLabel->setText(getFormatDescription(m_selectedFormat));
    
    // 显示/隐藏压缩选项
    bool showCompression = (m_selectedFormat == PointCloudSaveFormat::SCAN_LZ4 ||
                           m_selectedFormat == PointCloudSaveFormat::SCAN_ZSTD);
    m_compressionLevelSpin->setEnabled(showCompression);
    m_compressionLabel->setEnabled(showCompression);
    
    updateStorageEstimate();
}

inline void SaveFormatDialog::setPointCount(uint64_t count)
{
    m_pointCount = count;
    updateStorageEstimate();
}

inline void SaveFormatDialog::setDefaultFormat(PointCloudSaveFormat format)
{
    for (int i = 0; i < m_formatCombo->count(); ++i) {
        if (m_formatCombo->itemData(i).toInt() == static_cast<int>(format)) {
            m_formatCombo->setCurrentIndex(i);
            break;
        }
    }
}

inline void SaveFormatDialog::updateStorageEstimate()
{
    if (m_pointCount == 0) {
        m_estimateLabel->setText(tr("请先加载数据"));
        return;
    }
    
    uint64_t estimatedSize = estimateStorageSize(m_pointCount, m_selectedFormat);
    
    QString sizeStr;
    if (estimatedSize >= 1024 * 1024 * 1024) {
        sizeStr = QString("%1 GB").arg(estimatedSize / (1024.0 * 1024.0 * 1024.0), 0, 'f', 2);
    } else if (estimatedSize >= 1024 * 1024) {
        sizeStr = QString("%1 MB").arg(estimatedSize / (1024.0 * 1024.0), 0, 'f', 2);
    } else if (estimatedSize >= 1024) {
        sizeStr = QString("%1 KB").arg(estimatedSize / 1024.0, 0, 'f', 2);
    } else {
        sizeStr = QString("%1 bytes").arg(estimatedSize);
    }
    
    // 计算相对于CSV的节省
    uint64_t csvSize = estimateStorageSize(m_pointCount, PointCloudSaveFormat::CSV);
    double savings = csvSize > 0 ? (1.0 - static_cast<double>(estimatedSize) / csvSize) * 100.0 : 0;
    
    QString text = tr("点数: %1\n估算大小: %2")
                   .arg(m_pointCount)
                   .arg(sizeStr);
    
    if (m_selectedFormat != PointCloudSaveFormat::CSV) {
        text += tr("\n相比CSV节省: %1%").arg(savings, 0, 'f', 1);
    }
    
    m_estimateLabel->setText(text);
}
