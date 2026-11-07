#pragma once

#include <QWidget>
#include <QTextEdit>
#include <QComboBox>
#include <QLineEdit>
#include <QPushButton>
#include <QCheckBox>
#include <QLabel>
#include <QTimer>

#include "utils/LogManager.h"

/**
 * @file LogWidget.h
 * @brief 日志显示控件
 * 
 * 提供日志的实时显示、过滤、搜索、导出等功能。
 */
class LogWidget : public QWidget
{
    Q_OBJECT

public:
    explicit LogWidget(QWidget* parent = nullptr);
    ~LogWidget() override;
    
    /**
     * @brief 设置最大显示行数
     */
    void setMaxLines(int maxLines);
    int maxLines() const { return m_maxLines; }
    
    /**
     * @brief 设置自动滚动
     */
    void setAutoScroll(bool enable);
    bool autoScroll() const { return m_autoScroll; }
    
    /**
     * @brief 设置显示时间戳
     */
    void setShowTimestamp(bool show);
    bool showTimestamp() const { return m_showTimestamp; }
    
    /**
     * @brief 手动添加日志（用于不通过LogManager的日志）
     */
    void appendLog(const LogEntry& entry);
    
    /**
     * @brief 获取选中的文本
     */
    QString selectedText() const;
    
    /**
     * @brief 获取所有文本
     */
    QString allText() const;

public slots:
    /**
     * @brief 清空日志显示
     */
    void clear();
    
    /**
     * @brief 滚动到底部
     */
    void scrollToBottom();
    
    /**
     * @brief 滚动到顶部
     */
    void scrollToTop();
    
    /**
     * @brief 刷新显示（重新从LogManager加载）
     */
    void refresh();

signals:
    /**
     * @brief 双击日志条目
     */
    void logDoubleClicked(const LogEntry& entry);

private slots:
    void onLogAdded(const LogEntry& entry);
    void onLogsCleared();
    void onLevelFilterChanged(int index);
    void onCategoryFilterChanged(const QString& text);
    void onSearchTextChanged(const QString& text);
    void onClearClicked();
    void onExportClicked();
    void onCopyClicked();
    void onAutoScrollToggled(bool checked);
    void onPauseToggled(bool checked);
    void onOpenLogFolder();
    void onSearchNext();
    void onSearchPrev();
    void applyFilters();

protected:
    void resizeEvent(QResizeEvent* event) override;

private:
    void createUi();
    void connectSignals();
    bool matchesFilter(const LogEntry& entry) const;
    void appendFormattedLog(const LogEntry& entry);
    void highlightSearch(const QString& text);
    QString formatLogEntry(const LogEntry& entry) const;
    void updateCountLabel();
    
private:
    // UI 控件
    QTextEdit* m_logView = nullptr;
    QComboBox* m_comboLevel = nullptr;
    QComboBox* m_comboCategory = nullptr;
    QLineEdit* m_editSearch = nullptr;
    QPushButton* m_btnSearchNext = nullptr;
    QPushButton* m_btnSearchPrev = nullptr;
    QPushButton* m_btnClear = nullptr;
    QPushButton* m_btnExport = nullptr;
    QPushButton* m_btnCopy = nullptr;
    QPushButton* m_btnOpenFolder = nullptr;
    QCheckBox* m_chkAutoScroll = nullptr;
    QCheckBox* m_chkPause = nullptr;
    QLabel* m_lblCount = nullptr;
    
    // 配置
    int m_maxLines = 5000;
    bool m_autoScroll = true;
    bool m_showTimestamp = true;
    bool m_paused = false;
    
    // 过滤
    LogLevel m_filterLevel = LogLevel::Debug;
    QString m_filterCategory;
    QString m_searchText;
    
    // 缓存（暂停时使用）
    QList<LogEntry> m_pendingLogs;
    
    // 统计
    int m_totalCount = 0;
    int m_displayedCount = 0;
    
    // 搜索
    int m_currentSearchIndex = -1;
    QList<int> m_searchResults;
};
