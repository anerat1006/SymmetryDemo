#ifndef SYMETRY3D_H
#define SYMETRY3D_H

#include <open3d/Open3D.h>
#include <QVector3D>
#include "voxelgrid.h"
#include "3D/mesh3d.h"
#include <omp.h>
#include <math.h>

struct SymetryPlaneData {
    SymetryPlaneData() {
        symetry=0;
    }
    float symetry;
    Vector3D normal;
    Eigen::Vector2i angles;
};

struct SymClass {
    Vector3D COG;
    int counter;
};

struct Symetry3DDebugData
{
    Symetry3DDebugData() {}
    std::vector<Eigen::Vector3d> voxelsL, voxelsR;
    Vector3D lCOM,rCOM,lRefl,rRefl;
    double dMin,dMax;
    double dist;
};

class Symetry3D
{
public:
    Symetry3D();
    ~Symetry3D();
    std::vector<SymetryPlaneData> determineSymetries(VoxelGrid *voxGrid, double tolerance, int angleStep);
    std::vector<SymetryPlaneData> determineSymetries_SFor(VoxelGrid *voxGrid, double tolerance, int angleStep);
    std::vector<SymetryPlaneData> determineSymetries_SFor_SingleThread(VoxelGrid *voxGrid, double tolerance, int angleStep);
    std::vector<SymetryPlaneData> determineSymetriesDC(VoxelGrid *voxGrid, double tolerance, int angleStep, int minAngleDiff);
    std::vector<SymetryPlaneData> determineSymetriesDC_SingleThread(VoxelGrid *voxGrid, double tolerance, int angleStep, int minAngleDiff);
    std::vector<SymetryPlaneData> refineSymetries(VoxelGrid *voxGrid, double tolerance, std::vector<SymetryPlaneData> symNorms);
    std::vector<SymetryPlaneData> refineSymetries_SingleThread(VoxelGrid *voxGrid, double tolerance, std::vector<SymetryPlaneData> symNorms);
    SymetryPlaneData estimateBestSymetries(VoxelGrid *voxGrid, int angleStep);
    SymetryPlaneData estimateBestSymetries_SFor(VoxelGrid *voxGrid, int angleStep);
    SymetryPlaneData estimateBestSymetries2Stage(VoxelGrid *coarseVoxGrid, VoxelGrid *voxGrid, int angleStep);
    std::vector<SymetryPlaneData> prepare1stEstimateFunction(VoxelGrid *voxGrid, int angleStep);

    SymetryPlaneData estimateBestSymetriesMesh(Mesh3D *mesh, int angleStep);

    std::vector<SymetryPlaneData> calcAllSymetriesMesh(Mesh3D *mesh, int angleStep);

    void prepareDebugData(VoxelGrid *voxGrid, double tolerance, int fi, int theta);
    void prepareDebugDataNR(VoxelGrid *voxGrid, double tolerance,int fi, int theta);
    std::vector<Symetry3DDebugData> *getDebugData();

    std::vector<SymetryPlaneData> estimateSymetriesNClass(VoxelGrid *voxGrid, int angleStep, double interval, double &minSym, double &maxSym);
    void evaluateSymetriesNClass(VoxelGrid *voxGrid,double array[92][362], int angleStep, double interval, double &minSym, double &maxSym);
    SymetryPlaneData estimateBestSymetryNClass(VoxelGrid *voxGrid, double array[92][362], int angleStep, double interval, double minSym, double maxSym);

    std::vector<SymetryPlaneData> progressiveDetermineSymetryPlanes(Mesh3D *mesh, VoxelGrid *coarseVoxGrid, VoxelGrid *voxGrid, int angleStep, double voxSize, double tolerance);

protected:
    double round_off(double val, unsigned int prec);
    void symetryPlaneDC(int fi, int theta1, int theta2, bool sym1, bool sym2, std::vector<QVector3D> *result);
    bool checkSymetry(std::vector<Eigen::Vector3d> voxelL, std::vector<Eigen::Vector3d> voxelR, Vector3D symN, Vector3D splitN, double dMin, double dMax, double tolerance, Vector3D &diffVect);
    bool checkSymetry(Eigen::Vector3d *voxGrid, int *indices, int leftF, int leftL, int rightF, int rightL, Vector3D symN, Vector3D splitN, double dMin, double dMax, double tolerance, Vector3D &diffVect);
    bool checkSymetryNonRecursive(Eigen::Vector3d *voxGrd, int N, Vector3D symN, Vector3D splitN, double dMin, double dMax, double tolerance, Vector3D &diffVect);
    bool checkSymetryDebug(Eigen::Vector3d *voxGrid, int *indices, int leftF, int leftL, int rightF, int rightL, Vector3D symN, Vector3D splitN, double dMin, double dMax, double tolerance, Vector3D &diffVect);
    bool checkSymetryAtPlane(VoxelGrid *voxGrid, int* indices, int N, double tolerance, int fi, int theta, Vector3D &symN, Vector3D &diffVect, bool debug=false);
    Vector3D calcVoxCOM(std::vector<Eigen::Vector3d> voxel);
    Vector3D calcVoxCOM(Eigen::Vector3d *voxGrid, int *indices, int first, int last, bool &calcTrue);
    void determineSymetryDC(VoxelGrid *voxGrid, int *indices, int N, double tolerance, std::vector<SymetryPlaneData> *symNormals, int fi, int theta1, int theta2, Vector3D diffVect1, Vector3D diffVect2, int minAngleDiff);
    bool splitUpDown(Eigen::Vector3d *voxGrid, int *indices, int first, int last, Vector3D splitN, double dMean, int &splitIndex);

    float estimateSymetry(VoxelGrid *voxGrid, int *indices, int N, Vector3D &symN, int fi, int theta);
    float evaluateSymetryNClass(VoxelGrid *voxGrid, int N, Vector3D &symN, int fi, int theta, double interval);

    float estimateSymetryMesh(Mesh3D *mesh, Vector3D &symN, int fi, int theta);

    void calcMeshCOMLR(Mesh3D *mesh, Vector3D symN, Vector3D &lCOM, Vector3D &rCOM, bool convHull);

    void insertSymPlane(std::vector<SymetryPlaneData> *planes, SymetryPlaneData n);
    void insertSymPlane(std::vector<Eigen::Vector2i> *planes, Eigen::Vector2i n);

    void calculateGaussFunction(double val, double c, double &res, double &deriv);

    void performLocalSearch(VoxelGrid *voxGrid, int startPhi, int startTheta, double symsEst[91][360], double &bestSym);

    void roundRobin2D(int i, int j, int M, int N, int &ii, int &jj);

    void swapValues(int &a, int &b);

    void calcDMinDMax(Eigen::Vector3d *voxGrd, int N, Vector3D splitN, double &dMin, double &dMax);

    bool checkSymetryNonRecursiveDebugData(Eigen::Vector3d *voxGrd, int N, Vector3D symN, Vector3D splitN, double dMin, double dMax, double tolerance, Vector3D &diffVect);

    double sinTable[361],cosTable[361];

    double symetryEstimate[91][361];

    std::vector<Symetry3DDebugData> debugData;

    omp_lock_t mutex,mutex2;
};

#endif // SYMETRY3D_H
