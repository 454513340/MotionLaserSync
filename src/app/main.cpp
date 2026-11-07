#include <QApplication>
#include <QCoreApplication>
#include <QSurfaceFormat>
#include <QTimer>
#include <QFile>
#include <QDir>
#include <QPushButton>
#include <QDialog>
#include <QPointer>

#include <QVTKOpenGLNativeWidget.h>

#include "app/MainWindow.h"

int main(int argc, char* argv[])
{
    // VTK requires the QVTK surface format before QApplication / widgets exist.
    QCoreApplication::setAttribute(Qt::AA_ShareOpenGLContexts, true);
    QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat());

    QApplication app(argc, argv);

    app.setApplicationName("MotionLaserSync");
    app.setOrganizationName("YourCompany");
    app.setApplicationVersion("V30.0.4");

    // Optional field smoke test: verifies startup and the lazy multi-pose window
    // without connecting a controller or requesting motion.  The test trace is
    // written to the OS temporary directory so it also works for a WIN32
    // subsystem executable without a console.
    const bool guiSmoke = app.arguments().contains(QStringLiteral("--smoke-ui"));
    const QString smokeTrace = QDir(QDir::tempPath()).filePath(
        QStringLiteral("MotionLaserSync_gui_smoke.txt"));
    const auto writeSmoke = [guiSmoke, smokeTrace](const QString& line) {
        if (!guiSmoke) return;
        QFile f(smokeTrace);
        if (f.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text)) {
            f.write(line.toUtf8());
            f.write("\n");
        }
    };
    if (guiSmoke) {
        QFile f(smokeTrace);
        if (f.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text))
            f.write("BEGIN: construct MainWindow\n");
    }

    MainWindow w;
    w.show();
    writeSmoke(QStringLiteral("PASS: MainWindow constructed and shown"));

    if (guiSmoke) {
        QTimer::singleShot(250, &app, [&]() {
            auto* button = w.findChild<QPushButton*>(
                QStringLiteral("btnOpenMultiPoseExperiment"));
            if (!button) {
                writeSmoke(QStringLiteral("FAIL: multi-pose button missing from Path Planning"));
                app.exit(5);
                return;
            }
            button->click(); // constructs the experimental window only now
            auto* dialog = w.findChild<QDialog*>(
                QStringLiteral("multiPoseExperimentWindow"));
            if (!dialog || !dialog->isVisible()) {
                writeSmoke(QStringLiteral("FAIL: multi-pose dialog did not open"));
                app.exit(6);
                return;
            }
            // Verify the newly inserted replay controls exist AFTER lazy UI
            // initialization. It is a construction check, not path execution.
            bool hasPlay=false,hasPause=false,hasStep=false,hasCompare=false;
            for(auto* control:dialog->findChildren<QPushButton*>()){
                const QString label=control->text();
                hasPlay |= label.contains(QStringLiteral("播放五轴"));
                hasPause |= label.contains(QStringLiteral("暂停/继续"));
                hasStep |= label==QStringLiteral("单步");
                hasCompare |= label.contains(QStringLiteral("实测 .scan 并对比"));
            }
            if(!hasPlay||!hasPause||!hasStep||!hasCompare){
                writeSmoke(QStringLiteral("FAIL: simulation playback controls missing"));
                app.exit(8);return;
            }
            writeSmoke(QStringLiteral("PASS: multi-pose playback, step and comparison UI constructed"));
            writeSmoke(QStringLiteral("PASS: lazy multi-pose dialog constructed and shown"));
            // Allow Qt/VTK one more event cycle to expose a paint-time crash,
            // rather than reporting success and exiting in the click callback.
            const QPointer<QDialog> observed(dialog);
            QTimer::singleShot(450, &app, [&, observed]() {
                if (!observed || !observed->isVisible()) {
                    writeSmoke(QStringLiteral("FAIL: dialog was lost before paint smoke elapsed"));
                    app.exit(7);
                    return;
                }
                observed->close();
                writeSmoke(QStringLiteral("PASS: GUI smoke test complete (no hardware operations)"));
                app.exit(0);
            });
        });
    }

    return app.exec();
}
