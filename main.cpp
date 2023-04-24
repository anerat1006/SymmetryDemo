#include "mainwindow.h"

#include <QApplication>
#include <QSurfaceFormat>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    QSurfaceFormat fmt;
    fmt.setSamples(24);
    fmt.setDepthBufferSize(24);
    fmt.setMajorVersion(4);
    fmt.setMinorVersion(0);
    QSurfaceFormat::setDefaultFormat(fmt);
    MainWindow w;
    w.show();
    return a.exec();
}
