#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <QFileDialog>
#include <QMessageBox>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    ui->displayVoxelsWidget->setVoxGrid(&grid);

    ui->displayVoxelsSymWidget->setVoxGrid(&grid);
}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::on_loadMeshButton_clicked()
{
    QString fileName=QFileDialog::getOpenFileName((QWidget*)this,"Open mesh file",".","Mesh files (*.ply *.glb *.stl *.obj *.off)");
    if(fileName != "") {
        mesh.loadTriangleMeshAssimp(fileName,1.0);

        grid.generateVoxels(&mesh, ui->voxelSizeSlider->value() / 1000.0);

        coarseGrid.generateVoxels(&mesh,  ui->voxelSizeSlider->value() / 100.0);

        ui->meshViewWidget->prepareObject(&mesh);

        ui->displayVoxelsWidget->updateVoxelGrid();

        ui->meshViewWidget->update();
        ui->displayVoxelsWidget->update();

        ui->generateVoxelsButton->setEnabled(true);
    }
}


void MainWindow::on_MeshLightDirX_valueChanged(int value)
{
    QVector3D pos(ui->MeshLightDirX->value() / 100.0f,ui->MeshLightDirY->value() / 100.0f,ui->MeshLightDirZ->value() / 100.0f);
    ui->meshViewWidget->setLightPosition(pos);
}


void MainWindow::on_MeshLightDirY_valueChanged(int value)
{
    QVector3D pos(ui->MeshLightDirX->value() / 100.0f,ui->MeshLightDirY->value() / 100.0f,ui->MeshLightDirZ->value() / 100.0f);
    ui->meshViewWidget->setLightPosition(pos);
}


void MainWindow::on_MeshLightDirZ_valueChanged(int value)
{
    QVector3D pos(ui->MeshLightDirX->value() / 100.0f,ui->MeshLightDirY->value() / 100.0f,ui->MeshLightDirZ->value() / 100.0f);
    ui->meshViewWidget->setLightPosition(pos);
}


void MainWindow::on_generateVoxelsButton_clicked()
{
    grid.generateVoxels(&mesh, ui->voxelSizeSlider->value() / 1000.0);
    coarseGrid.generateVoxels(&mesh,  ui->voxelSizeSlider->value() / 100.0);
    ui->displayVoxelsWidget->updateVoxelGrid();
    ui->displayVoxelsWidget->update();
}


void MainWindow::on_VoxelLightDirX_valueChanged(int value)
{
    ui->displayVoxelsWidget->setLightPosX(value / 100.0);
}


void MainWindow::on_VoxelLightDirY_valueChanged(int value)
{
    ui->displayVoxelsWidget->setLightPosY(value / 100.0);
}


void MainWindow::on_VoxelLightDirZ_valueChanged(int value)
{
    ui->displayVoxelsWidget->setLightPosZ(value / 100.0);
}


void MainWindow::on_checkSymetryButton_clicked()
{
    ui->SymetryPlanes->clear();
    std::vector<SymetryPlaneData> symsCan=sym3dCalc.determineSymetriesDC(&coarseGrid,ui->CoarseSymTollerance3d->value()/100.0,1, SYM3D_DC_MIN_ANGLE);

    if(symsCan.size() < 1) {
        QMessageBox::information((QWidget*)this,"Information","No symetry found. Try readjusting tolerance in 1. step.");
        return;
    }

    std::vector<SymetryPlaneData> syms=sym3dCalc.refineSymetries(&grid,ui->voxelSymTolerance->value() / 10.0,symsCan);

    if(syms.size() < 1) {
        QMessageBox::information((QWidget*)this,"Information","No symetry found. Try readjusting tolerance in 2. step.");
        return;
    }

    int i;

    for(i=0;i<syms.size();i++) {
        QString pom="(" + QString::number(syms[i].angles[0]) + "," + QString::number(syms[i].angles[1]) + ")->";
        QString str=pom+QString::number(syms[i].normal.x) + "," + QString::number(syms[i].normal.y) + "," + QString::number(syms[i].normal.z);
        ui->SymetryPlanes->addItem(str);

    }

    if(syms.size() > 0) {
        ui->SymetryPlanes->setCurrentRow(0);
    }
}


void MainWindow::on_SymetryPlanes_currentTextChanged(const QString &currentText)
{
    if(ui->SymetryPlanes->currentRow() < 0) {
        ui->displayVoxelsSymWidget->clean();
        ui->displayVoxelsSymWidget->update();
        return;
    }
    std::vector<Eigen::Vector3d> vL,vR,vSP;
    vL.clear();
    vR.clear();
    vSP.clear();
    QStringList l2=currentText.split("->");
    QStringList list=l2[1].split(",");
    QVector3D n(list[0].toDouble(),list[1].toDouble(),list[2].toDouble());

    grid.makeSplitForVis(n,vL,vR,vSP);

    ui->displayVoxelsSymWidget->setVoxelsL(vL);
    ui->displayVoxelsSymWidget->setVoxelsR(vR);
    ui->displayVoxelsSymWidget->setVoxelsSP(vSP);

    ui->displayVoxelsSymWidget->update();
}

