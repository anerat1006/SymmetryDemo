#ifndef VOXELGRID_H
#define VOXELGRID_H

#include <Eigen/Eigen>
#include <open3d/Open3D.h>
#include <vector>
#include <QVector3D>
#include "3D/mesh3d.h"
#include <omp.h>

struct Vector3Di {
    Vector3Di() {
        x=0;y=0;z=0;
    }

    int x,y,z;
};

struct Vector3D {
    Vector3D() {
        x=0;y=0;z=0;
    }

    double distanceToPoint(Vector3D p2) {
        double val=(x-p2.x)*(x-p2.x) + (y-p2.y)*(y-p2.y) + (z-p2.z)*(z-p2.z);
        return sqrt(val);
    }
    double x,y,z;
};

class VoxelGrid
{
public:
    VoxelGrid();
    ~VoxelGrid();
    void generateVoxels(Mesh3D *mesh, double voxel_size);

    std::vector<Eigen::Vector3d> *getVoxelGrid();
    Eigen::Vector3d *getVoxelGridData();
    std::vector<Eigen::Vector3d> *getVoxelGridL();
    std::vector<Eigen::Vector3d> *getVoxelGridR();
    std::vector<Eigen::Vector3d> *getVoxelGridLU();
    std::vector<Eigen::Vector3d> *getVoxelGridRU();
    std::vector<Eigen::Vector3d> *getVoxelGridLD();
    std::vector<Eigen::Vector3d> *getVoxelGridRD();
    std::vector<Eigen::Vector3d> *getVoxelGridSP();
    const Eigen::Vector3d &getVoxCenter() const;
    int getVoxSX() const;
    int getVoxSY() const;
    int getVoxSZ() const;
    void recalculateVGSize(Mesh3D *mesh, int &sx, int &sy, int &sz, double voxel_size);
    void rotateSPNormal(int fi, int theta);
    void makeSplitForVis(QVector3D n, std::vector<Eigen::Vector3d> &voxelL, std::vector<Eigen::Vector3d> &voxelR, std::vector<Eigen::Vector3d> &voxelSP);
    void prepareVoxelSplit(QVector3D symN, std::vector<Eigen::Vector3d> *voxelL, std::vector<Eigen::Vector3d> *voxelR);
    void prepareVoxelSplit(Vector3D symN, Vector3D splitN, int* indices, int &low, int &mid, int &high, double &dMin, double &dMax);
    Eigen::Vector3d *getCenterVectors();
    void generateCoarseVoxelGrid(VoxelGrid *fineGrid, int factor);

    const Eigen::Vector3d &getMinPoint() const;

    void generateBoundaryVoxels(VoxelGrid *fullGrid);

    void setVoxCenter(const Eigen::Vector3d &newVoxCenter);

    const Eigen::Vector3d &getDisplacedVoxCenter() const;

protected:
    void calcVoxCenter();
    void swapValues(int &a, int &b);
    void splitVoxelsSymetryPlane();
    void setSymetryPlane();
    void voxelCoords(Eigen::Vector3d r1, Eigen::Vector3d r2, Eigen::Vector3d r3, double alpha, double beta, double gamma, double vox_size);

    void floodFill(int ***grid, int ***fullGrid, int minTresh);

    int voxSX,voxSY,voxSZ;
    int ***vox;
    std::vector<Eigen::Vector3d> voxelGrid, voxelGridL, voxelGridR, voxelGridSP;
    std::vector<Eigen::Vector3d> voxelGridLU, voxelGridRU, voxelGridLD, voxelGridRD;
    QVector3D normalSP, normalSplitPlane;
    double dSP, dSplitPlane;
    Eigen::Vector3d voxCenter, centerVectors[8], minPoint, displacedVoxCenter;

    double round_off(double val, unsigned int prec);

    double sinTable[361],cosTable[361];

    omp_lock_t mutex;
};

#endif // VOXELGRID_H
