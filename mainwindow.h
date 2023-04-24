#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "3D/mesh3d.h"
#include "3D/symetry3d.h"
#include "3D/voxelgrid.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_loadMeshButton_clicked();

    void on_MeshLightDirX_valueChanged(int value);

    void on_MeshLightDirY_valueChanged(int value);

    void on_MeshLightDirZ_valueChanged(int value);

    void on_generateVoxelsButton_clicked();

    void on_VoxelLightDirX_valueChanged(int value);

    void on_VoxelLightDirY_valueChanged(int value);

    void on_VoxelLightDirZ_valueChanged(int value);

    void on_checkSymetryButton_clicked();

    void on_SymetryPlanes_currentTextChanged(const QString &currentText);

private:
    Ui::MainWindow *ui;

    Mesh3D mesh;

    Symetry3D sym3dCalc;

    VoxelGrid grid,coarseGrid;
};
#endif // MAINWINDOW_H
