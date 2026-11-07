#include "widgets/LogWidget.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QScrollBar>
#include <QFileDialog>
#include <QMessageBox>
#include <QClipboard>
#include <QApplication>
#include <QDesktopServices>
#include <QUrl>
#include <QDir>
#include <QTextDocument>
#include <QTextCursor>
#include <QTextBlock>

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
#include <QStringConverter>
#endif

LogWidget::LogWidget(QWidget* parent)
    : QWidget(parent)
{
    // 先创建UI，此时不连接任何信号
    createUi();
    
    // 使用定时器延迟初始化，确保所有控件都已完成创建
    QTimer::singleShot(200, this, [this]() {
        // 延迟连接信号，避免初始化期间的信号触发
        connectSignals();
        
        // 连接到 LogManager
        connect(&LogManager::instance(), &LogManager::logAdded,
                this, &LogWidget::onLogAdded, Qt::QueuedConnection);
        connect(&LogManager::instance(), &LogManager::logsCleared,
                this, &LogWidget::onLogsCleared, Qt::QueuedConnection);
        
        // 加载历史日志
        refresh();
    });
}

LogWidget::~LogWidget()
{
}

void LogWidget::createUi()
{
    auto* mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(2, 2, 2, 2);
    mainLayout->setSpacing(2);
    
    // ===== 工具栏 =====
    auto* toolbarLayout = new QHBoxLayout();
    toolbarLayout->setSpacing(5);
    
    // 级别过滤
    toolbarLayout->addWidget(new QLabel(tr("级别:")));
    m_comboLevel = new QComboBox();
    m_comboLevel->addItem(tr("全部"), static_cast<int>(LogLevel::Debug));
    m_comboLevel->addItem(tr("调试"), static_cast<int>(LogLevel::Debug));
    m_comboLevel->addItem(tr("信息"), static_cast<int>(LogLevel::Info));
    m_comboLevel->addItem(tr("警告"), static_cast<int>(LogLevel::Warning));
    m_comboLevel->addItem(tr("错误"), static_cast<int>(LogLevel::Error));
    m_comboLevel->addItem(tr("致命"), static_cast<int>(LogLevel::Fatal));
    m_comboLevel->setCurrentIndex(0);
    m_comboLevel->setMinimumWidth(70);
    toolbarLayout->addWidget(m_comboLevel);
    
    // 分类过滤 - 不使用editable，避免信号问题
    toolbarLayout->addWidget(new QLabel(tr("分类:")));
    m_comboCategory = new QComboBox();
    m_comboCategory->setEditable(false);  // 禁用可编辑，避免信号问题
    m_comboCategory->addItem(tr("全部"));
    m_comboCategory->addItem(LogCategory::System);
    m_comboCategory->addItem(LogCategory::Controller);
    m_comboCategory->addItem(LogCategory::Sensor);
    m_comboCategory->addItem(LogCategory::Sync);
    m_comboCategory->addItem(LogCategory::GCode);
    m_comboCategory->addItem(LogCategory::PointCloud);
    m_comboCategory->addItem(LogCategory::Calibration);
    m_comboCategory->addItem(LogCategory::UI);
    m_comboCategory->addItem(LogCategory::File);
    m_comboCategory->setMinimumWidth(100);
    toolbarLayout->addWidget(m_comboCategory);
    
    // 分隔
    toolbarLayout->addSpacing(10);
    
    // 搜索
    toolbarLayout->addWidget(new QLabel(tr("搜索:")));
    m_editSearch = new QLineEdit();
    m_editSearch->setPlaceholderText(tr("输入关键字..."));
    m_editSearch->setClearButtonEnabled(true);
    m_editSearch->setMinimumWidth(150);
    toolbarLayout->addWidget(m_editSearch);
    
    m_btnSearchPrev = new QPushButton(tr("<"));
    m_btnSearchPrev->setFixedWidth(30);
    m_btnSearchPrev->setToolTip(tr("上一个"));
    toolbarLayout->addWidget(m_btnSearchPrev);
    
    m_btnSearchNext = new QPushButton(tr(">"));
    m_btnSearchNext->setFixedWidth(30);
    m_btnSearchNext->setToolTip(tr("下一个"));
    toolbarLayout->addWidget(m_btnSearchNext);
    
    // 弹性空间
    toolbarLayout->addStretch();
    
    // 统计标签
    m_lblCount = new QLabel(tr("0 / 0"));
    m_lblCount->setToolTip(tr("显示数 / 总数"));
    toolbarLayout->addWidget(m_lblCount);
    
    // 分隔
    toolbarLayout->addSpacing(10);
    
    // 控制按钮
    m_chkAutoScroll = new QCheckBox(tr("自动滚动"));
    m_chkAutoScroll->setChecked(true);
    toolbarLayout->addWidget(m_chkAutoScroll);
    
    m_chkPause = new QCheckBox(tr("暂停"));
    m_chkPause->setChecked(false);
    toolbarLayout->addWidget(m_chkPause);
    
    // 分隔
    toolbarLayout->addSpacing(10);
    
    // 操作按钮
    m_btnCopy = new QPushButton(tr("复制"));
    m_btnCopy->setToolTip(tr("复制选中内容到剪贴板"));
    toolbarLayout->addWidget(m_btnCopy);
    
    m_btnExport = new QPushButton(tr("导出"));
    m_btnExport->setToolTip(tr("导出日志到文件"));
    toolbarLayout->addWidget(m_btnExport);
    
    m_btnClear = new QPushButton(tr("清空"));
    m_btnClear->setToolTip(tr("清空日志显示"));
    toolbarLayout->addWidget(m_btnClear);
    
    m_btnOpenFolder = new QPushButton(tr("..."));
    m_btnOpenFolder->setFixedWidth(30);
    m_btnOpenFolder->setToolTip(tr("打开日志文件夹"));
    toolbarLayout->addWidget(m_btnOpenFolder);
    
    mainLayout->addLayout(toolbarLayout);
    
    // ===== 日志显示区域 =====
    m_logView = new QTextEdit();
    m_logView->setReadOnly(true);
    m_logView->setFont(QFont("Consolas", 9));
    m_logView->setLineWrapMode(QTextEdit::NoWrap);
    
    mainLayout->addWidget(m_logView, 1);
}

void LogWidget::connectSignals()
{
    // 使用lambda包装，添加初始化检查
    connect(m_comboLevel, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, [this](int) { 
                if (m_logView) applyFilters(); 
            });
    connect(m_comboCategory, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, [this](int index) {
                if (!m_logView) return;
                QString text = m_comboCategory->itemText(index);
                m_filterCategory = (text == tr("全部")) ? QString() : text;
                applyFilters();
            });
    connect(m_editSearch, &QLineEdit::textChanged,
            this, &LogWidget::onSearchTextChanged);
    connect(m_editSearch, &QLineEdit::returnPressed,
            this, &LogWidget::onSearchNext);
    connect(m_btnSearchNext, &QPushButton::clicked,
            this, &LogWidget::onSearchNext);
    connect(m_btnSearchPrev, &QPushButton::clicked,
            this, &LogWidget::onSearchPrev);
    connect(m_btnClear, &QPushButton::clicked,
            this, &LogWidget::onClearClicked);
    connect(m_btnExport, &QPushButton::clicked,
            this, &LogWidget::onExportClicked);
    connect(m_btnCopy, &QPushButton::clicked,
            this, &LogWidget::onCopyClicked);
    connect(m_btnOpenFolder, &QPushButton::clicked,
            this, &LogWidget::onOpenLogFolder);
    connect(m_chkAutoScroll, &QCheckBox::toggled,
            this, &LogWidget::onAutoScrollToggled);
    connect(m_chkPause, &QCheckBox::toggled,
            this, &LogWidget::onPauseToggled);
}

void LogWidget::onLogAdded(const LogEntry& entry)
{
    if (!m_logView || !m_lblCount) {
        return;
    }
    
    m_totalCount++;
    
    // 如果暂停，添加到待处理队列
    if (m_paused) {
        m_pendingLogs.append(entry);
        while (m_pendingLogs.size() > 1000) {
            m_pendingLogs.removeFirst();
        }
        updateCountLabel();
        return;
    }
    
    // 检查过滤条件
    if (!matchesFilter(entry)) {
        updateCountLabel();
        return;
    }
    
    appendFormattedLog(entry);
}

void LogWidget::onLogsCleared()
{
    if (m_logView) {
        m_logView->clear();
    }
    m_pendingLogs.clear();
    m_totalCount = 0;
    m_displayedCount = 0;
    m_searchResults.clear();
    m_currentSearchIndex = -1;
    updateCountLabel();
}

void LogWidget::onLevelFilterChanged(int index)
{
    Q_UNUSED(index)
    if (m_logView) {
        applyFilters();
    }
}

void LogWidget::onCategoryFilterChanged(const QString& text)
{
    m_filterCategory = (text == tr("全部")) ? QString() : text;
    if (m_logView) {
        applyFilters();
    }
}

void LogWidget::onSearchTextChanged(const QString& text)
{
    m_searchText = text;
    m_searchResults.clear();
    m_currentSearchIndex = -1;
    
    if (!text.isEmpty()) {
        highlightSearch(text);
    } else if (m_logView) {
        applyFilters();
    }
}

void LogWidget::onClearClicked()
{
    int ret = QMessageBox::question(this, tr("确认清空"),
                                    tr("确定要清空日志显示吗？\n（不会删除日志文件）"),
                                    QMessageBox::Yes | QMessageBox::No);
    if (ret == QMessageBox::Yes) {
        clear();
    }
}

void LogWidget::onExportClicked()
{
    QString fileName = QFileDialog::getSaveFileName(this, tr("导出日志"),
                                                    QString("log_export_%1.txt")
                                                        .arg(QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss")),
                                                    tr("文本文件 (*.txt);;所有文件 (*.*)"));
    if (fileName.isEmpty()) {
        return;
    }
    
    // 导出当前显示的日志
    QFile file(fileName);
    if (file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QTextStream out(&file);
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
        out.setEncoding(QStringConverter::Utf8);
#else
        out.setCodec("UTF-8");
#endif
        out << m_logView->toPlainText();
        file.close();
        
        QMessageBox::information(this, tr("导出成功"),
                                tr("日志已导出到:\n%1").arg(fileName));
    } else {
        QMessageBox::warning(this, tr("导出失败"),
                            tr("无法写入文件:\n%1").arg(fileName));
    }
}

void LogWidget::onCopyClicked()
{
    QString text = m_logView->textCursor().selectedText();
    if (text.isEmpty()) {
        text = m_logView->toPlainText();
    }
    
    QApplication::clipboard()->setText(text);
}

void LogWidget::onAutoScrollToggled(bool checked)
{
    m_autoScroll = checked;
}

void LogWidget::onPauseToggled(bool checked)
{
    m_paused = checked;
    
    if (!checked && !m_pendingLogs.isEmpty()) {
        // 恢复时显示待处理的日志
        for (const LogEntry& entry : m_pendingLogs) {
            if (matchesFilter(entry)) {
                appendFormattedLog(entry);
            }
        }
        m_pendingLogs.clear();
    }
    
    updateCountLabel();
}

void LogWidget::onOpenLogFolder()
{
    QString logDir = LogManager::instance().currentLogFile();
    if (logDir.isEmpty()) {
        logDir = "./logs";
    } else {
        QFileInfo fi(logDir);
        logDir = fi.absolutePath();
    }
    
    QDir dir(logDir);
    if (!dir.exists()) {
        dir.mkpath(".");
    }
    
    QDesktopServices::openUrl(QUrl::fromLocalFile(dir.absolutePath()));
}

void LogWidget::onSearchNext()
{
    if (m_searchText.isEmpty() || !m_logView) return;
    
    QTextDocument* doc = m_logView->document();
    QTextCursor cursor = m_logView->textCursor();
    
    cursor = doc->find(m_searchText, cursor);
    if (cursor.isNull()) {
        // 从头开始搜索
        cursor = doc->find(m_searchText);
    }
    
    if (!cursor.isNull()) {
        m_logView->setTextCursor(cursor);
    }
}

void LogWidget::onSearchPrev()
{
    if (m_searchText.isEmpty() || !m_logView) return;
    
    QTextDocument* doc = m_logView->document();
    QTextCursor cursor = m_logView->textCursor();
    
    cursor = doc->find(m_searchText, cursor, QTextDocument::FindBackward);
    if (cursor.isNull()) {
        // 从末尾开始搜索
        cursor.movePosition(QTextCursor::End);
        cursor = doc->find(m_searchText, cursor, QTextDocument::FindBackward);
    }
    
    if (!cursor.isNull()) {
        m_logView->setTextCursor(cursor);
    }
}

void LogWidget::applyFilters()
{
    if (!m_comboLevel || !m_logView) return;
    
    // 获取过滤级别
    m_filterLevel = static_cast<LogLevel>(m_comboLevel->currentData().toInt());
    
    // 重新加载日志
    refresh();
}

void LogWidget::setMaxLines(int maxLines)
{
    m_maxLines = maxLines;
}

void LogWidget::setAutoScroll(bool enable)
{
    m_autoScroll = enable;
    if (m_chkAutoScroll) {
        m_chkAutoScroll->setChecked(enable);
    }
}

void LogWidget::setShowTimestamp(bool show)
{
    m_showTimestamp = show;
    refresh();
}

void LogWidget::appendLog(const LogEntry& entry)
{
    if (matchesFilter(entry)) {
        appendFormattedLog(entry);
    }
}

QString LogWidget::selectedText() const
{
    return m_logView ? m_logView->textCursor().selectedText() : QString();
}

QString LogWidget::allText() const
{
    return m_logView ? m_logView->toPlainText() : QString();
}

void LogWidget::clear()
{
    if (!m_logView) return;
    
    m_logView->clear();
    m_displayedCount = 0;
    m_searchResults.clear();
    m_currentSearchIndex = -1;
    updateCountLabel();
}

void LogWidget::scrollToBottom()
{
    if (!m_logView) return;
    
    QScrollBar* sb = m_logView->verticalScrollBar();
    if (sb) {
        sb->setValue(sb->maximum());
    }
}

void LogWidget::scrollToTop()
{
    if (!m_logView) return;
    
    QScrollBar* sb = m_logView->verticalScrollBar();
    if (sb) {
        sb->setValue(sb->minimum());
    }
}

void LogWidget::refresh()
{
    if (!m_logView || !m_lblCount) {
        return;
    }
    
    m_logView->clear();
    m_displayedCount = 0;
    
    // 从 LogManager 获取日志
    QList<LogEntry> logs = LogManager::instance().recentLogs(m_maxLines, m_filterLevel);
    
    for (const LogEntry& entry : logs) {
        if (matchesFilter(entry)) {
            appendFormattedLog(entry);
        }
    }
    
    m_totalCount = static_cast<int>(LogManager::instance().recentLogs(0, LogLevel::Debug).size());
    updateCountLabel();
    
    if (m_autoScroll) {
        scrollToBottom();
    }
}

void LogWidget::resizeEvent(QResizeEvent* event)
{
    QWidget::resizeEvent(event);
}

bool LogWidget::matchesFilter(const LogEntry& entry) const
{
    // 级别过滤
    if (entry.level < m_filterLevel) {
        return false;
    }
    
    // 分类过滤
    if (!m_filterCategory.isEmpty() && entry.category != m_filterCategory) {
        return false;
    }
    
    // 搜索过滤
    if (!m_searchText.isEmpty()) {
        if (!entry.message.contains(m_searchText, Qt::CaseInsensitive) &&
            !entry.category.contains(m_searchText, Qt::CaseInsensitive)) {
            return false;
        }
    }
    
    return true;
}

void LogWidget::appendFormattedLog(const LogEntry& entry)
{
    if (!m_logView) return;
    
    // 限制行数
    QTextDocument* doc = m_logView->document();
    if (doc) {
        while (doc->blockCount() > m_maxLines) {
            QTextCursor cursor(doc->firstBlock());
            cursor.select(QTextCursor::BlockUnderCursor);
            cursor.removeSelectedText();
            cursor.deleteChar();
        }
    }
    
    // 追加新日志 - 使用纯文本避免HTML问题
    QString text = entry.toString();
    m_logView->append(text);
    
    m_displayedCount++;
    updateCountLabel();
    
    if (m_autoScroll) {
        scrollToBottom();
    }
}

void LogWidget::highlightSearch(const QString& text)
{
    Q_UNUSED(text)
    // 简化实现，不做高亮
}

QString LogWidget::formatLogEntry(const LogEntry& entry) const
{
    if (m_showTimestamp) {
        return entry.toString();
    } else {
        return QString("[%1] [%2] %3")
            .arg(LogEntry::levelName(entry.level), -5)
            .arg(entry.category, -12)
            .arg(entry.message);
    }
}

void LogWidget::updateCountLabel()
{
    if (!m_lblCount) return;
    
    m_lblCount->setText(QString("%1 / %2")
                       .arg(m_displayedCount)
                       .arg(m_totalCount));
    
    if (m_paused && !m_pendingLogs.isEmpty()) {
        m_lblCount->setText(m_lblCount->text() + QString(" (+%1)").arg(m_pendingLogs.size()));
    }
}
