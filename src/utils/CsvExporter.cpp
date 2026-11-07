#include "utils/CsvExporter.h"

#include <QFile>
#include <QSaveFile>
#include <QTextStream>
#include <QVector3D>

namespace CsvExporter {

namespace {
void frameHeader(QTextStream& o) {
    o << ",has_pose,session,ordinal,timing,feedback_mask,latch_mask,command_mask,absolute_gauge_known";
    for(const char* prefix : {"joint_", "command_", "raw_"})for(const char* axis:{"x_mm","y_mm","z_mm","c_deg","b_deg"})o << ',' << prefix << axis;
    o << ",c_offset_deg,b_offset_deg,pose_host_estimate_us";
    for(int i=0;i<16;++i)o << ",T_MS_" << i;
    for(int i=0;i<17;++i)o << ",physical17_" << i;
}
void frameRow(QTextStream& o,const mls::runtime::Frame5D* f) {
    if(!f){o << ",0";for(int i=0;i<58;++i)o << ',';return;}
    o << ",1," << quint64(f->session) << ',' << quint64(f->ordinal) << ',' << unsigned(f->timing) << ',' << f->feedbackMask << ',' << f->latchMask << ',' << f->commandMask << ',' << int(f->absoluteGaugeKnown);
    for(const auto* a:{&f->joints,&f->command,&f->raw})for(double x:*a)o << ',' << x;
    for(double x:f->rotaryOffsets)o << ',' << x;
    o << ',' << qint64(f->hostEstimateUs);
    for(double x:f->sensorToMachine)o << ',' << x;
    for(double x:f->physicalParameters)o << ',' << x;
}
}
void writeAlignmentHeader(QTextStream& o){o << "t_us,profile_index,point_index,sensor_time_s,sensor_x,sensor_z,machine_x,machine_y,machine_z";frameHeader(o);o << '\n';}
void writeAlignmentRow(QTextStream& o,const AlignedPoint& p){o << p.tUs << ',' << p.profileIndex << ',' << p.pointIndex << ',' << p.tSensorSec << ',' << p.sensorX << ',' << p.sensorZ << ',' << p.machineX << ',' << p.machineY << ',' << p.machineZ;frameRow(o,p.frame.get());o << '\n';}
void writePoseHeader(QTextStream& o){o << "t_us,x_mm,y_mm,z_mm,c_deg,b_deg";frameHeader(o);o << '\n';}
void writePoseRow(QTextStream& o,const SamplePoint& p){o << p.tUs << ',' << p.x << ',' << p.y << ',' << p.z << ',' << p.c << ',' << p.b;frameRow(o,p.hasRotary?&p.frame:nullptr);o << '\n';}
bool exportAlignmentData(const QString& path,const QVector<AlignedPoint>& data,QString* e){
    QSaveFile f(path);if(!f.open(QIODevice::WriteOnly|QIODevice::Text)){if(e)*e=f.errorString();return false;}
    QTextStream o(&f);o.setRealNumberNotation(QTextStream::SmartNotation);o.setRealNumberPrecision(17);writeAlignmentHeader(o);
    for(const auto& p:data)writeAlignmentRow(o,p);o.flush();
    if(o.status()!=QTextStream::Ok||!f.commit()){if(e)*e=f.errorString();return false;}return true;
}
bool exportPoseStream(const QString& path,const QVector<SamplePoint>& data,QString* e){
    QSaveFile f(path);if(!f.open(QIODevice::WriteOnly|QIODevice::Text)){if(e)*e=f.errorString();return false;}
    QTextStream o(&f);o.setRealNumberNotation(QTextStream::SmartNotation);o.setRealNumberPrecision(17);writePoseHeader(o);
    for(const auto& p:data)writePoseRow(o,p);o.flush();
    if(o.status()!=QTextStream::Ok||!f.commit()){if(e)*e=f.errorString();return false;}return true;
}

bool exportPointCloudPly(const QString& fileName,
                         const QVector<QVector3D>& points,
                         QString* errMsg)
{
    QFile file(fileName);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        if (errMsg) *errMsg = QObject::tr("无法打开文件：%1").arg(fileName);
        return false;
    }

    QTextStream ts(&file);

    // PLY 文件头
    ts << "ply\n";
    ts << "format ascii 1.0\n";
    ts << "element vertex " << points.size() << "\n";
    ts << "property float x\n";
    ts << "property float y\n";
    ts << "property float z\n";
    ts << "end_header\n";

    ts.setRealNumberNotation(QTextStream::FixedNotation);
    ts.setRealNumberPrecision(6);

    for (const auto& pt : points) {
        ts << pt.x() << ' ' << pt.y() << ' ' << pt.z() << '\n';
    }

    file.close();
    return true;
}

bool exportPointCloudCsv(const QString& fileName,
                         const QVector<QVector3D>& points,
                         QString* errMsg)
{
    QFile file(fileName);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        if (errMsg) *errMsg = QObject::tr("无法打开文件：%1").arg(fileName);
        return false;
    }

    QTextStream ts(&file);
    ts.setRealNumberNotation(QTextStream::FixedNotation);
    ts.setRealNumberPrecision(6);

    // CSV 表头
    ts << "X,Y,Z\n";

    for (const auto& pt : points) {
        ts << pt.x() << ',' << pt.y() << ',' << pt.z() << '\n';
    }

    file.close();
    return true;
}

} // namespace CsvExporter
