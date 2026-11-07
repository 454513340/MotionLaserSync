#include "gcode/GCodeParser.h"
#include "gcode/GCodeSyncMetadata.h"
#include <QFile>
#include <QFileInfo>
#include <QTextStream>
#include <map>
#include <set>
#include <algorithm>
#include <cmath>

namespace {
using namespace mls::runtime;
QString number(double v){return QString::number(v,'g',17);}
QString axisWords(const PathPoint& p,std::uint8_t mask){
    const double values[5]={p.x,p.y,p.z,p.c,p.b};QString s;
    for(int i=0;i<5;++i)if(mask&(1u<<i))s+=" "+QString(QChar("XYZCB"[i]))+number(values[i]);
    return s;
}
std::uint8_t maskOf(const GCodeBlock& b){return (b.hasX?1:0)|(b.hasY?2:0)|(b.hasZ?4:0)|(b.hasC?8:0)|(b.hasB?16:0);}
bool csvFields(const QString& line,QStringList& out){
    out.clear();QString current;bool quoted=false,afterQuote=false;
    for(qsizetype i=0;i<line.size();++i){const QChar ch=line[i];
        if(quoted){if(ch=='"'){if(i+1<line.size()&&line[i+1]=='"'){current+='"';++i;}else{quoted=false;afterQuote=true;}}else current+=ch;}
        else if(ch==','){out.append(current);current.clear();afterQuote=false;}
        else if(ch=='"'){if(!current.isEmpty()||afterQuote)return false;quoted=true;}
        else {if(afterQuote && !ch.isSpace())return false;if(!afterQuote)current+=ch;}
    }
    if(quoted)return false;out.append(current);return true;
}
QString csvQuote(QString v){v.replace("\"","\"\"");return "\""+v+"\"";}
}
GCodeParser::GCodeParser(QObject* parent):QObject(parent){}

bool GCodeParser::parseLine(const QString& line,GCodeBlock& block,QString* error){
    using namespace mls::runtime;
    NcBlock n;std::string detail;
    if(!parseNcBlock(line.toStdString(),m_ncState,n,detail)){if(error)*error=QString::fromStdString(detail);return false;}
    block={};block.rawLine=line;block.lineNumber=n.lineNumber;
    block.absoluteModeSnapshot=n.absolute;block.rtcpModeSnapshot=n.rtcp;block.feedMode=n.feedMode;block.target.feedMode=n.feedMode;
    block.target={n.words[0],n.words[1],n.words[2],n.feed.value_or(0),n.words[3],n.words[4]};block.target.axisMask=n.axisMask;block.target.feedMode=n.feedMode;
    block.hasX=n.axisMask&1;block.hasY=n.axisMask&2;block.hasZ=n.axisMask&4;block.hasC=n.axisMask&8;block.hasB=n.axisMask&16;
    block.feedRate=n.feed;block.syncCapture=n.capture;block.dwellTimeMs=n.dwellMs;
    block.arc.plane=n.plane==Plane::XY?ArcPlane::XY:(n.plane==Plane::ZX?ArcPlane::XZ:ArcPlane::YZ);
    block.arc.centerI=n.arc.centerOffset[0];block.arc.centerJ=n.arc.centerOffset[1];block.arc.centerK=n.arc.centerOffset[2];block.arc.radius=n.arc.radius;
    block.arc.midX=n.mid[0];block.arc.midY=n.mid[1];block.arc.midZ=n.mid[2];
    block.arc.endX=n.words[0];block.arc.endY=n.words[1];block.arc.endZ=n.words[2];
    if(n.arcDefinition==NcBlock::ArcDefinition::Center)block.arc.defineMode=ArcDefineMode::CenterEnd;
    if(n.arcDefinition==NcBlock::ArcDefinition::Radius)block.arc.defineMode=ArcDefineMode::RadiusEnd;
    if(n.arcDefinition==NcBlock::ArcDefinition::ThreePoint)block.arc.defineMode=ArcDefineMode::ThreePoint;
    switch(n.kind){
    case NcBlock::Kind::Empty:block.type=BlockType::Comment;block.comment=line;break;
    case NcBlock::Kind::Modal:block.type=BlockType::Modal;break;
    case NcBlock::Kind::Dwell:block.type=BlockType::G04_Dwell;break;
    case NcBlock::Kind::SetWorkPosition:block.type=BlockType::G92_SetPosition;break;
    case NcBlock::Kind::Motion:
        switch(n.motion){case 0:block.type=BlockType::G00_RapidMove;break;case 1:block.type=BlockType::G01_LinearInterp;break;case 2:block.type=BlockType::G02_ArcCW;break;case 3:block.type=BlockType::G03_ArcCCW;break;case 5:block.type=BlockType::G05_ContinuousPath;break;default:return false;}break;
    case NcBlock::Kind::MCode:
        switch(n.mcode){case 0:block.type=BlockType::M00_ProgramStop;break;case 2:case 30:block.type=BlockType::M30_ProgramEnd;break;
        case 50:block.type=BlockType::M50_SyncOn;break;case 51:block.type=BlockType::M51_SyncOff;break;case 60:block.type=BlockType::M60_SavePointCloud;break;
        case 61:block.type=BlockType::M61_ClearPointCloud;break;case 62:block.type=BlockType::M62_SetSavePath;break;case 63:block.type=BlockType::M63_WaitSaveComplete;break;default:return false;}break;
    }
    auto text=[&](char c){auto i=n.strings.find(c);return i==n.strings.end()?QString():QString::fromStdString(i->second);};
    block.name=text('L');
    if(n.mcode==62){block.saveParams.baseDir=text('P');block.saveParams.prefix=text('Q');block.saveParams.suffix=text('R');if(!text('T').isEmpty())block.saveParams.format=text('T').toLower();}
    m_absoluteMode=n.absolute;return true;
}

bool GCodeParser::parseText(const QString& text,GCodeProgram& program,QList<ParseError>* errors){
    program={};program.createTime=QDateTime::currentDateTime();m_ncState={};m_absoluteMode=true;
    const auto lines=text.split('\n');
    for(const auto& raw:lines){const auto l=raw.trimmed();if(l.isEmpty())continue;
        if(l.startsWith('#')||l.toUpper().startsWith("CODE,")||l.toUpper()=="CODE"||l.toUpper().startsWith("N,TYPE,"))return parseCSV(text,program,errors);break;}
    bool valid=true;
    for(int i=0;i<lines.size();++i){QString l=lines[i].trimmed();if(l.isEmpty())continue;
        GCodeBlock b;QString detail;
        if(!parseLine(l,b,&detail)){if(errors)errors->append({i+1,l,detail});valid=false;continue;}
        if(b.lineNumber==0)b.lineNumber=i+1;
        if(b.type==BlockType::Comment){
            if(!parseHeaderComment(l,program,&detail)){
                if(errors)errors->append({i+1,l,detail});valid=false;
            }
            program.blocks.push_back(b);continue;
        }
        if(b.type==BlockType::G05_ContinuousPath){int last=i;
            if(!parseContinuousPathPoints(lines,i+1,b,last,&detail)){if(errors)errors->append({i+1,l,detail});valid=false;}
            i=last;
        }
        program.blocks.push_back(std::move(b));
    }
    if(program.name.isEmpty())program.name=tr("五轴扫描程序");
    if(program.defaultSaveParams.baseDir.isEmpty())program.defaultSaveParams.baseDir="./scans";
    if(program.defaultSaveParams.prefix.isEmpty())program.defaultSaveParams.prefix="scan";
    return valid && validateProgram(program,errors);
}
bool GCodeParser::parseFile(const QString& path,GCodeProgram& program,QList<ParseError>* errors){
    QFile f(path);if(!f.open(QIODevice::ReadOnly|QIODevice::Text)){if(errors)errors->append({0,path,f.errorString()});return false;}
    if(!parseText(QString::fromUtf8(f.readAll()),program,errors))return false;
    program.filePath=QFileInfo(path).absoluteFilePath();
    return true;
}
bool GCodeParser::parseContinuousPathPoints(const QStringList& lines,int start,GCodeBlock& b,int& end,QString* error){
    end=start-1;double feed=b.feedRate.value_or(0);
    if(b.target.axisMask)b.continuousPath.points.push_back(b.target);
    for(int i=start;i<lines.size();++i){std::vector<mls::runtime::Word> words;std::string detail;
        if(!mls::runtime::lex(lines[i].toStdString(),words,detail)){if(error)*error=QString::fromStdString(detail);return false;}
        bool command=false;for(const auto& w:words)if(w.letter=='G'||w.letter=='M')command=true;
        if(command)break;
        end=i;if(words.empty())continue;
        auto state=m_ncState;state.modalMotion=5;mls::runtime::NcBlock n;
        if(!mls::runtime::parseNcBlock(lines[i].toStdString(),state,n,detail)||n.kind!=mls::runtime::NcBlock::Kind::Motion||n.axisMask==0){if(error)*error=tr("G05轨迹点错误：%1").arg(QString::fromStdString(detail));return false;}
        if(n.feed)feed=*n.feed;
        PathPoint p;p.x=n.words[0];p.y=n.words[1];p.z=n.words[2];p.c=n.words[3];p.b=n.words[4];p.axisMask=n.axisMask;p.feed=feed;p.feedMode=n.feedMode;
        b.continuousPath.points.push_back(p);
    }
    if(b.continuousPath.points.empty()){if(error)*error=tr("G05必须包含至少一个轨迹点");return false;}
    return true;
}

bool GCodeParser::parseCSV(const QString& text,GCodeProgram& program,QList<ParseError>* errors){
    QStringList headers;QString converted;bool header=false;int lineNo=0;
    for(const QString& raw:text.split('\n')){++lineNo;QString line=raw.trimmed();if(line.isEmpty())continue;
        if(line.startsWith('#')){converted+=";"+line.mid(1)+"\n";continue;}
        QStringList values;if(!csvFields(line,values)){if(errors)errors->append({lineNo,raw,tr("CSV引号不匹配")});return false;}
        if(!header){for(auto& v:values)v=v.trimmed().toUpper();headers=values;header=true;continue;}
        if(values.size()!=headers.size()){if(errors)errors->append({lineNo,raw,tr("CSV列数不匹配，拒绝跳过该行")});return false;}
        const int codeIndex=headers.indexOf("CODE");if(codeIndex>=0){converted+=values[codeIndex]+"\n";continue;}
        std::map<QString,QString> row;for(int k=0;k<headers.size();++k)row[headers[k]]=values[k].trimmed();
        if(row["TYPE"].isEmpty()){if(errors)errors->append({lineNo,raw,tr("CSV缺少Type，不再默认为G01")});return false;}
        QString code=row["TYPE"];if(!row["N"].isEmpty())code="N"+row["N"]+" "+code;
        for(const auto& kv:row){const auto& key=kv.first;const auto& value=kv.second;if(value.isEmpty()||key=="N"||key=="TYPE"||key=="COMMENT")continue;
            if(key.size()==1 && QString("XYZCBIJKR").contains(key))code+=" "+key+value;
            else if(key=="FEED"||key=="F")code+=" F"+value;
            else if(key=="SYNC")code+=" S"+(value.toLower()=="true"?QString("1"):(value.toLower()=="false"?QString("0"):value));
            else if(key=="ARCMIDX")code+=" P"+value;
            else if(key=="ARCMIDY")code+=" Q"+value;
            else if(key=="SAVENAME"||key=="NAME"||key=="L")code+=" L\""+value+"\"";
            else {if(errors)errors->append({lineNo,raw,tr("不支持的CSV非空列：%1").arg(key)});return false;}
        }
        if(!row["COMMENT"].isEmpty())code+=" ;"+row["COMMENT"];
        converted+=code+"\n";
    }
    if(!header){if(errors)errors->append({0,{},tr("CSV缺少表头")});return false;}
    return parseText(converted,program,errors);
}

QString GCodeParser::exportToText(const GCodeProgram& p){
    QString out="; Program: "+p.name+"\n; SyncMode: "+(p.syncMode==SyncMode::Hardware?QString("Hardware"):QString("Software"))+"\n";
    if(p.offlineOnly)out+="; OfflineOnly: true\n";
    if(p.hardwareTopologyId)out+="; HardwareTopology: "+QString(p.hardwareTopologyId==1?"H1":"H3")+"\n";
    if(p.hardwareFrequencyHz>0)out+="; HardwareFrequencyHz: "+QString::number(p.hardwareFrequencyHz)+"\n";
    if(p.hardwareTopologyId==1 || p.hardwareTopologyId==3) {
        const auto sensorMode=p.sensorTriggerMode.isEmpty()
            ? mls::gcode::sensorTriggerModeForTopology(p.hardwareTopologyId) : p.sensorTriggerMode;
        out+="; SensorTriggerMode: "+sensorMode+"\n";
    }
    if(p.hardwareTopologyId==1 && !p.hardwareTriggerType.isEmpty())out+="; HardwareTriggerType: "+p.hardwareTriggerType+"\n";
    if(p.cmmR1Edge>0)out+="; CmmR1Edge: "+QString::number(p.cmmR1Edge)+"\n";
    if(!p.programFrame.isEmpty())out+="; ProgramFrame: "+p.programFrame+"\n";
    if(!p.partPoseId.isEmpty())out+="; PartPoseId: "+p.partPoseId+"\n";
    if(!p.calibrationId.isEmpty())out+="; CalibrationId: "+p.calibrationId+"\n";
    if(!p.physical17Sha256.isEmpty())out+="; Physical17Sha256: "+p.physical17Sha256+"\n";
    out+="; SaveDir: "+p.defaultSaveParams.baseDir+"\n; SavePrefix: "+p.defaultSaveParams.prefix+"\n";
    out+="; Feed: "+number(p.motionParams.feedSpeed)+"\n; Accel: "+number(p.motionParams.accel)+"\n; Decel: "+number(p.motionParams.decel)+"\n";
    bool absolute=true,rtcp=false;double modalFeed=p.motionParams.feedSpeed;
    for(const auto& b:p.blocks){
        if(b.feedRate)modalFeed=*b.feedRate;
        if(b.type==BlockType::Comment){out+="; "+b.comment+"\n";continue;}
        if(b.type==BlockType::Empty)continue;
        if(b.type==BlockType::Modal){out+=b.rawLine+"\n";rtcp=b.rtcpModeSnapshot.value_or(rtcp);continue;}
        if(b.type==BlockType::G90_AbsoluteMode)absolute=true;if(b.type==BlockType::G91_IncrementalMode)absolute=false;
        QString line=b.lineNumber>0?"N"+QString::number(b.lineNumber)+" ":QString();
        const bool motion=GCodeUtils::isMotionBlock(b.type);
        if(motion){
            const bool requestedRtcp=b.rtcpModeSnapshot.value_or(rtcp);
            if(requestedRtcp!=rtcp){line+=requestedRtcp?"G43.4 ":"G49 ";rtcp=requestedRtcp;}
            absolute=b.absoluteModeSnapshot.value_or(absolute);line+=absolute?"G90 ":"G91 ";
            line+=b.arc.plane==ArcPlane::XY?"G17 ":(b.arc.plane==ArcPlane::XZ?"G18 ":"G19 ");
            line+=b.feedMode==FeedMode::InverseMinute?"G93 ":"G94 ";
        }
        line+=GCodeUtils::blockTypeName(b.type);
        if((motion && b.type!=BlockType::G05_ContinuousPath)||b.type==BlockType::G92_SetPosition)line+=axisWords(b.target,maskOf(b));
        if(b.type==BlockType::G02_ArcCW||b.type==BlockType::G03_ArcCCW){
            if(b.arc.defineMode==ArcDefineMode::CenterEnd)line+=" I"+number(b.arc.centerI)+" J"+number(b.arc.centerJ)+" K"+number(b.arc.centerK);
            else if(b.arc.defineMode==ArcDefineMode::RadiusEnd)line+=" R"+number(b.arc.radius);
            else line+=" P"+number(b.arc.midX)+" Q"+number(b.arc.midY);
        }
        if(motion){double f=modalFeed;
            if(b.feedMode==FeedMode::LegacyPerSecond)f*=60;
            line+=" F"+number(f)+" S"+(b.syncCapture?"1":"0");
        }
        if(b.type==BlockType::G04_Dwell)line+=" P"+QString::number(b.dwellTimeMs);
        if(!b.name.isEmpty())line+=" L\""+b.name+"\"";
        if(b.type==BlockType::M62_SetSavePath){line+=" P\""+b.saveParams.baseDir+"\" Q\""+b.saveParams.prefix+"\" R\""+b.saveParams.suffix+"\" T\""+b.saveParams.format+"\"";}
        if(!b.comment.isEmpty())line+=" ;"+b.comment;
        out+=line+"\n";
        if(b.type==BlockType::G05_ContinuousPath){for(const auto& pt:b.continuousPath.points){
            if(pt.feed>0)modalFeed=pt.feed;double f=modalFeed;if(b.feedMode==FeedMode::LegacyPerSecond)f*=60;
            out+=axisWords(pt,pt.axisMask)+" F"+number(f)+"\n";
        }}
    }
    return out;
}
QString GCodeParser::exportToCSV(const GCodeProgram& p){
    // Code is authoritative and round-trips modal words, quoted paths and G05 points.
    QString out="Code\n";for(const auto& l:exportToText(p).split('\n'))if(!l.isEmpty())out+=csvQuote(l)+"\n";return out;
}
bool GCodeParser::validateProgram(const GCodeProgram& p,QList<ParseError>* errors){
    auto bad=[&](const QString& e){if(errors)errors->append({0,{},e});return false;};
    if(p.blocks.empty())return bad(tr("程序为空"));
    for(double v:{double(p.motionParams.feedSpeed),double(p.motionParams.rapidSpeed),double(p.motionParams.accel),double(p.motionParams.decel)})
        if(!std::isfinite(v)||v<=0)return bad(tr("程序默认速度及加减速度必须为有限正数"));
    if(!std::isfinite(p.motionParams.sramp)||p.motionParams.sramp<0)return bad(tr("SRAMP参数非法"));
    if(p.hardwareTopologyId!=0&&p.hardwareTopologyId!=1&&p.hardwareTopologyId!=3)
        return bad(tr("硬件同步拓扑只能是H1(1)或H3(3)，H2(2)已退役"));
    if(p.syncMode==SyncMode::Hardware){
        if(!p.syncModeSpecified)return bad(tr("硬件采集必须显式写入 ; SyncMode: Hardware"));
        if(p.hardwareTopologyId!=1&&p.hardwareTopologyId!=3)
            return bad(tr("硬件采集必须显式指定 ; HardwareTopology: H1 或 H3"));
        if(p.hardwareFrequencyHz<1||p.hardwareFrequencyHz>2000)
            return bad(tr("硬件采集必须显式指定有效 HardwareFrequencyHz；设备测量场上限须另行验证"));
        if(p.hardwareTopologyId==3 && p.cmmR1Edge!=114 && p.cmmR1Edge!=115)
            return bad(tr("H3必须显式指定 CmmR1Edge: 114 或 115"));
        if(p.hardwareTopologyId==1 && p.cmmR1Edge!=0)
            return bad(tr("H1使用IN0/R0/104，不允许CmmR1Edge"));
        if(p.programFrame.isEmpty())return bad(tr("硬件采集必须明确 ProgramFrame 坐标定义"));
        QString triggerProblem;
        if(!mls::gcode::validateSensorTriggerMetadata(p,&triggerProblem))
            return bad(tr("NC传感器触发方式不符合H1/H3拓扑：%1").arg(triggerProblem));
    }
    if(!p.programFrame.isEmpty()&&p.programFrame!="MACHINE_JOINTS_XYZCB"&&p.programFrame!="RTCP_TCP_XYZCB")
        return bad(tr("不支持的 ProgramFrame；必须明确XYZCB关节空间或RTCP任务空间"));
    bool capture=false,hasMotion=false,hasEnd=false;
    for(const auto& b:p.blocks){
        if(b.type==BlockType::M50_SyncOn){
            if(capture)return bad(tr("连续M50不允许；必须先M51完成Stop/Drain"));
            capture=true;continue;
        }
        if(b.type==BlockType::M51_SyncOff){
            if(!capture)return bad(tr("M51之前没有对应的M50"));
            capture=false;continue;
        }
        if(b.type==BlockType::M60_SavePointCloud || b.type==BlockType::M61_ClearPointCloud ||
           b.type==BlockType::G92_SetPosition || b.type==BlockType::M00_ProgramStop ||
           b.type==BlockType::G04_Dwell || b.type==BlockType::M30_ProgramEnd){
            if(capture)return bad(tr("采集时禁止保存/清空/G92/停留/暂停/结束，先M51并完成尾帧验证"));
        }
        if(GCodeUtils::isMotionBlock(b.type)){
            hasMotion=true;
            const bool isRapid=b.type==BlockType::G00_RapidMove;
            if(isRapid && (b.syncCapture||capture))
                return bad(tr("G00不是采集运动；禁止在M50开启时空移，请先M51"));
            if(capture && !b.syncCapture)
                return bad(tr("M50期间运动段必须显式S1；若需不曝光传送，应先M51并等待采集停止"));
            if(p.syncMode==SyncMode::Hardware && b.syncCapture && !capture)
                return bad(tr("硬件采集运动必须由显式M50开始，不能隐式启用"));
            if(p.programFrame=="MACHINE_JOINTS_XYZCB" && b.rtcpModeSnapshot.value_or(false))
                return bad(tr("ProgramFrame为关节空间，却使用了G43.4 RTCP"));
            if(p.programFrame=="RTCP_TCP_XYZCB" && !b.rtcpModeSnapshot.value_or(false))
                return bad(tr("ProgramFrame为RTCP任务空间，却未启用G43.4"));
            const double axes[5]={b.target.x,b.target.y,b.target.z,b.target.c,b.target.b};
            for(double a:axes)if(!std::isfinite(a))return bad(tr("运动目标包含非有限XYZCB数值"));
            if(b.type==BlockType::G05_ContinuousPath){
                if(b.continuousPath.points.empty())return bad(tr("G05缺少五轴插补点"));
                for(const auto& q:b.continuousPath.points){
                    if(q.axisMask==0)return bad(tr("G05包含无轴坐标点"));
                    if(p.syncMode==SyncMode::Hardware && q.axisMask!=31)
                        return bad(tr("硬件五轴G05需要逐点完整XYZCB，避免模态轴不明确"));
                    for(double a:{q.x,q.y,q.z,q.c,q.b,q.feed})
                        if(!std::isfinite(a))return bad(tr("G05点包含NaN/Inf"));
                }
            }
        }
        if(b.type==BlockType::M30_ProgramEnd){hasEnd=true;break;}
    }
    if(capture)return bad(tr("程序结束时M50/M51不配对"));
    if(!hasMotion)return bad(tr("程序没有任何五轴运动"));
    if(!hasEnd)return bad(tr("程序必须显式包含M30结束和保存屏障"));
    return true;
}

bool GCodeParser::parseHeaderComment(const QString& comment, GCodeProgram& program, QString* error)
{
    QString content = comment;
    if (content.startsWith(';')) {
        content = content.mid(1).trimmed();
    }
    if (content.startsWith('(')) {
        content = content.mid(1);
        if (content.endsWith(')')) {
            content.chop(1);
        }
        content = content.trimmed();
    }
    
    // 解析键值对
    if (content.contains(':')) {
        int colonPos = content.indexOf(':');
        QString key = content.left(colonPos).trimmed().toLower();
        QString value = content.mid(colonPos + 1).trimmed();
        
        // 程序信息
        if (key == "program" || key == "程序名称" || key == "name") {
            program.name = value;
        } else if (key == "author" || key == "作者") {
            program.author = value;
        } else if (key == "description" || key == "描述") {
            program.description = value;
        } else if (key == "version" || key == "版本") {
            program.version = value;
        } else if (key == "syncmode" || key == "同步模式") {
            const auto mode=value.trimmed().toLower();
            if(mode!="hardware" && mode!="software"){
                if(error)*error=tr("SyncMode只能是Hardware或Software，不能模糊匹配");return false;
            }
            program.syncMode=mode=="hardware"?SyncMode::Hardware:SyncMode::Software;
            program.syncModeSpecified=true;
        }else if(key=="hardwaretopology"){
            const auto topology=value.trimmed().toUpper();
            if(topology!="H1" && topology!="H3"){
                if(error)*error=tr("仅允许HardwareTopology: H1或H3；H2已退役");return false;
            }
            program.hardwareTopologyId=topology=="H1"?1:3;
        }else if(key=="hardwarefrequencyhz"){
            bool ok=false;const int hz=value.toInt(&ok);
            if(!ok||hz<1||hz>2000){if(error)*error=tr("HardwareFrequencyHz必须是1..2000的整数");return false;}
            program.hardwareFrequencyHz=hz;
        }else if(key=="sensortriggermode"){
            const auto v=value.trimmed().toLower();
            if(v!="hardwareexternal" && v!="softwareinternal"){
                if(error)*error=tr("SensorTriggerMode只能是HardwareExternal(H1)或SoftwareInternal(H3)");return false;
            }
            program.sensorTriggerMode = v=="hardwareexternal" ? QStringLiteral("HardwareExternal")
                : QStringLiteral("SoftwareInternal");
        }else if(key=="hardwaretriggertype"){
            if(value.compare(QStringLiteral("EdgeRising"),Qt::CaseInsensitive)!=0){
                if(error)*error=tr("遗留HardwareTriggerType仅支持H1外部上升沿EdgeRising；H3请使用SensorTriggerMode: SoftwareInternal");return false;
            }
            program.hardwareTriggerType=QStringLiteral("EdgeRising");
        }else if(key=="cmmr1edge"){
            bool ok=false;const int edge=value.toInt(&ok);
            if(!ok || (edge!=114&&edge!=115)){
                if(error)*error=tr("H3 CmmR1Edge必须为114或115");return false;
            }
            program.cmmR1Edge=edge;
        }else if(key=="offlineonly"){
            const auto flag=value.trimmed().toLower();
            if(flag!="true"&&flag!="false"){
                if(error)*error=tr("OfflineOnly只能是true/false");return false;
            }
            program.offlineOnly=flag=="true";
        }else if(key=="programframe"){
            program.programFrame=value.trimmed().toUpper();
        }else if(key=="partposeid"){
            program.partPoseId=value.trimmed();
        }else if(key=="calibrationid"){
            program.calibrationId=value.trimmed();
        }else if(key=="physical17sha256"){
            const QString hash=value.trimmed().toLower();
            const bool wellFormed=hash.size()==64 &&
                std::all_of(hash.cbegin(),hash.cend(),[](QChar c){
                    return (c>=QLatin1Char('0')&&c<=QLatin1Char('9')) ||
                           (c>=QLatin1Char('a')&&c<=QLatin1Char('f'));
                });
            if(!wellFormed){if(error)*error=tr("Physical17Sha256必须是64位十六进制摘要");return false;}
            program.physical17Sha256=hash;
        }
        // 保存设置
        else if (key == "savedir" || key == "保存目录") {
            program.defaultSaveParams.baseDir = value;
        } else if (key == "saveprefix" || key == "保存前缀") {
            program.defaultSaveParams.prefix = value;
        }
        // ===== 运动参数 =====
        else if (key == "feed" || key == "进给速度" || key == "feedspeed") {
            program.motionParams.feedSpeed = value.toFloat();
            program.headerFeedSpecified = true;
        } else if (key == "rapid" || key == "快速速度" || key == "rapidspeed") {
            program.motionParams.rapidSpeed = value.toFloat();
            program.headerRapidSpecified = true;
        } else if (key == "accel" || key == "加速度") {
            program.motionParams.accel = value.toFloat();
            program.headerAccelSpecified = true;
        } else if (key == "decel" || key == "减速度") {
            program.motionParams.decel = value.toFloat();
            program.headerDecelSpecified = true;
        } else if (key == "sramp" || key == "s曲线") {
            program.motionParams.sramp = value.toFloat();
        } else if (key == "lspeed" || key == "起始速度") {
            program.motionParams.lspeed = value.toFloat();
        }
        // 复合格式：Feed: 500, Accel: 1000, Decel: 1000
        else if (key == "motion" || key == "运动参数") {
            // 解析逗号分隔的参数
            QStringList parts = value.split(',');
            for (const QString& part : parts) {
                QString p = part.trimmed();
                if (p.contains(':') || p.contains('=')) {
                    int sep = p.contains(':') ? p.indexOf(':') : p.indexOf('=');
                    QString pKey = p.left(sep).trimmed().toLower();
                    float pVal = p.mid(sep + 1).trimmed().toFloat();
                    
                    if (pKey == "f" || pKey == "feed") {
                        program.motionParams.feedSpeed = pVal;
                        program.headerFeedSpecified = true;
                    } else if (pKey == "r" || pKey == "rapid") {
                        program.motionParams.rapidSpeed = pVal;
                        program.headerRapidSpecified = true;
                    } else if (pKey == "a" || pKey == "accel") {
                        program.motionParams.accel = pVal;
                        program.headerAccelSpecified = true;
                    } else if (pKey == "d" || pKey == "decel") {
                        program.motionParams.decel = pVal;
                        program.headerDecelSpecified = true;
                    }
                }
            }
        }
    }
    return true;
}
