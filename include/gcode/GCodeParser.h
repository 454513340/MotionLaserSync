#pragma once

#include <QObject>
#include <QString>
#include <QStringList>
#include <QTextStream>
#include <QRegularExpression>

#include "gcode/GCodeTypes.h"
#include "runtime/GCode5D.h"

/**
 * @file GCodeParser.h
 * @brief G代码解析器
 * 
 * 支持解析标准G代码文本和简化CSV格式
 */

/**
 * @brief 解析错误信息
 */
struct ParseError {
    int lineNumber = 0;         // 行号
    QString rawLine;            // 原始行内容
    QString errorMessage;       // 错误信息
};

/**
 * @brief G代码解析器
 */
class GCodeParser : public QObject
{
    Q_OBJECT

public:
    explicit GCodeParser(QObject* parent = nullptr);
    ~GCodeParser() override = default;

    /**
     * @brief 从文本解析G代码程序
     * @param text G代码文本内容
     * @param program 输出的程序结构
     * @param errors 输出的错误列表
     * @return 解析成功返回true（可能有警告但无致命错误）
     */
    bool parseText(const QString& text, 
                   GCodeProgram& program, 
                   QList<ParseError>* errors = nullptr);

    /**
     * @brief 从文件解析G代码程序
     * @param filePath 文件路径
     * @param program 输出的程序结构
     * @param errors 输出的错误列表
     * @return 解析成功返回true
     */
    bool parseFile(const QString& filePath,
                   GCodeProgram& program,
                   QList<ParseError>* errors = nullptr);

    /**
     * @brief 从CSV格式解析（简化格式）
     */
    bool parseCSV(const QString& text,
                  GCodeProgram& program,
                  QList<ParseError>* errors = nullptr);

    /**
     * @brief 解析单行G代码
     * @param line 输入行
     * @param block 输出的程序段
     * @param error 输出的错误信息
     * @return 解析成功返回true
     */
    bool parseLine(const QString& line,
                   GCodeBlock& block,
                   QString* error = nullptr);

    /**
     * @brief 将程序导出为G代码文本
     */
    QString exportToText(const GCodeProgram& program);

    /**
     * @brief 将程序导出为CSV格式
     */
    QString exportToCSV(const GCodeProgram& program);

    /**
     * @brief 验证程序逻辑
     */
    bool validateProgram(const GCodeProgram& program,
                         QList<ParseError>* errors = nullptr);

private:
    // 解析辅助函数
    bool parseGCode(const QString& line, GCodeBlock& block, QString* error);
    bool parseMCode(const QString& line, GCodeBlock& block, QString* error);
    bool parseParameters(const QString& params, GCodeBlock& block, QString* error);
    
    // 提取参数值
    bool extractFloat(const QString& params, QChar letter, float& value);
    bool extractInt(const QString& params, QChar letter, int& value);
    bool extractString(const QString& params, QChar letter, QString& value);
    
    // 解析头部注释（提取程序信息）
    bool parseHeaderComment(const QString& comment, GCodeProgram& program, QString* error = nullptr);
    
    // 解析连续轨迹点
    bool parseContinuousPathPoints(const QStringList& lines, 
                                   int startIndex,
                                   GCodeBlock& block,
                                   int& endIndex,
                                   QString* error);

private:
    // 正则表达式
    QRegularExpression m_reLineNumber;      // N行号
    QRegularExpression m_reGCode;           // G代码
    QRegularExpression m_reMCode;           // M代码
    QRegularExpression m_reFloat;           // 浮点参数
    QRegularExpression m_reInt;             // 整数参数
    QRegularExpression m_reString;          // 字符串参数 L"..."
    QRegularExpression m_reComment;         // 注释
    
    mls::runtime::NcState m_ncState;
    // 当前解析状态
    bool m_absoluteMode = true;             // 当前坐标模式
    float m_currentX = 0, m_currentY = 0, m_currentZ = 0;  // 当前位置
};
