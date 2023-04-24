#include "voxelgrid.h"
#include <iostream>
#include "open3d/geometry/IntersectionTest.h"
#include <QVector3D>
#include <QStack>
#include <math.h>
#include "constants.h"

VoxelGrid::VoxelGrid()
{
    int i;
    for(i=0;i<361;i++) {
        sinTable[i]=round_off(sin(i * PI / 180.0),5);
        cosTable[i]=round_off(cos(i * PI / 180.0),5);
    }

    vox=nullptr;
    voxSX=0;
    voxSY=0;
    voxSZ=0;

    normalSP.setX(sinTable[0] * sinTable[0]);
    normalSP.setY(cosTable[0]);
    normalSP.setZ(cosTable[0] * sinTable[0]);

    normalSplitPlane.setX(sinTable[0] * sinTable[0+90]);
    normalSplitPlane.setY(cosTable[0+90]);
    normalSplitPlane.setZ(cosTable[0] * sinTable[0+90]);

    omp_init_lock(&mutex);
}

double VoxelGrid::round_off(double val, unsigned int prec) {
    double pow_10 = pow(10.0f, (double)prec);
    return round(val * pow_10) / pow_10;
}

VoxelGrid::~VoxelGrid() {
    int i,j;
    if(vox != nullptr) {
        for(i=0;i<voxSX;i++) {
            for(j=0;j<voxSY;j++)
                delete[] vox[i][j];
            delete[] vox[i];
        }
        delete[] vox;
        vox=nullptr;
    }

    omp_destroy_lock(&mutex);
}

void VoxelGrid::generateBoundaryVoxels(VoxelGrid *fullGrid) {
    int ***fullVox=fullGrid->vox;

    int i,j,k;
    if(vox != nullptr) {
        for(i=0;i<voxSX;i++) {
            for(j=0;j<voxSY;j++)
                delete[] vox[i][j];
            delete[] vox[i];
        }
        delete[] vox;
        vox=nullptr;
    }

    voxSX=fullGrid->voxSX;
    voxSY=fullGrid->voxSY;
    voxSZ=fullGrid->voxSZ;

    vox=new int**[voxSX];
    for(i=0;i<voxSX;i++) {
        vox[i]=new int*[voxSY];
        for(j=0;j<voxSY;j++) {
            vox[i][j]=new int[voxSZ];
            for(k=0;k<voxSZ;k++)
                vox[i][j][k]=0;
        }
    }

    voxelGrid.clear();

    floodFill(vox,fullVox, 1);

    for(i=0;i<voxSX;i++) {
        for(j=0;j<voxSY;j++) {
            for(k=0;k<voxSZ;k++) {
                if(vox[i][j][k] > 0) {
                    Eigen::Vector3d v(i,j,k);
                    voxelGrid.push_back(v);
                }
                else {
                    vox[i][j][k]=0;
                }
            }
        }
    }

    calcVoxCenter();
}

void VoxelGrid::floodFill(int ***grid, int ***fullGrid, int minTresh) {
    QStack<Vector3Di> stack;
    stack.clear();

    const int offsets=6;

    int offsetX[]={ 0,0,-1,1, 0,0},i;
    int offsetY[]={-1,1, 0,0, 0,0};
    int offsetZ[]={ 0,0, 0,0,-1,1};

    Vector3Di point,point2;
    stack.push(point);
    grid[0][0][0]=-1;

    while(!stack.empty()) {
        point=stack.pop();

        for(i=0;i<offsets;i++) {
            int nx=point.x + offsetX[i],ny=point.y + offsetY[i],nz=point.z + offsetZ[i];
            if(nx < 0 || nx >= voxSX || ny < 0 || ny >= voxSY || nz < 0 || nz >= voxSZ)
                continue;

            if(fullGrid[nx][ny][nz] < minTresh && grid[nx][ny][nz] != -1) {
                grid[nx][ny][nz]=-1;
                point2.x=nx;
                point2.y=ny;
                point2.z=nz;
                stack.push(point2);
            }
            else if(fullGrid[nx][ny][nz] >= minTresh) {
                grid[nx][ny][nz]=1;
            }
        }
    }
}

const Eigen::Vector3d &VoxelGrid::getDisplacedVoxCenter() const
{
    return displacedVoxCenter;
}

void VoxelGrid::setVoxCenter(const Eigen::Vector3d &newVoxCenter)
{
    int i;
    voxCenter = newVoxCenter;
    displacedVoxCenter=newVoxCenter;
    double vx=voxSX + 1,vy=voxSY + 1, vz=voxSZ + 1;

    centerVectors[0]=Eigen::Vector3d(0,0,0) - voxCenter;
    centerVectors[1]=Eigen::Vector3d(vx,0,0) - voxCenter;
    centerVectors[2]=Eigen::Vector3d(0,vy,0) - voxCenter;
    centerVectors[3]=Eigen::Vector3d(vx,vy,0) - voxCenter;
    centerVectors[4]=Eigen::Vector3d(0,0,vz) - voxCenter;
    centerVectors[5]=Eigen::Vector3d(vx,0,vz) - voxCenter;
    centerVectors[6]=Eigen::Vector3d(0,vy,vz) - voxCenter;
    centerVectors[7]=Eigen::Vector3d(vx,vy,vz) - voxCenter;

    for(i=0;i<voxelGrid.size();i++) {
        voxelGrid[i] -= voxCenter;
//        voxelGrid[i][0] -= (int)voxCenter[0];
//        voxelGrid[i][1] -= (int)voxCenter[1];
//        voxelGrid[i][2] -= (int)voxCenter[2];
    }

    minPoint = voxCenter;

    voxCenter=Eigen::Vector3d(0,0,0);

    setSymetryPlane();
}

void VoxelGrid::generateVoxels(Mesh3D *mesh, double voxel_size) {
    double minX=mesh->getMinX();
    double minY=mesh->getMinY();
    double minZ=mesh->getMinZ();

    double maxX=mesh->getMaxX();
    double maxY=mesh->getMaxY();
    double maxZ=mesh->getMaxZ();

    int i,j,k;
    if(vox != nullptr) {
        for(i=0;i<voxSX;i++) {
            for(j=0;j<voxSY;j++)
                delete[] vox[i][j];
            delete[] vox[i];
        }
        delete[] vox;
        vox=nullptr;
    }

    voxSX=(int)((maxX-minX)/voxel_size)+1;
    voxSY=(int)((maxY-minY)/voxel_size)+1;
    voxSZ=(int)((maxZ-minZ)/voxel_size)+1;

    vox=new int**[voxSX];
    for(i=0;i<voxSX;i++) {
        vox[i]=new int*[voxSY];
        for(j=0;j<voxSY;j++) {
            vox[i][j]=new int[voxSZ];
            for(k=0;k<voxSZ;k++)
                vox[i][j][k]=0;
        }
    }

    voxelGrid.clear();

    Eigen::Vector3d minV(minX,minY,minZ);

    open3d::geometry::TriangleMesh *m=mesh->getMesh();

    #pragma omp parallel
    {
        #pragma omp for
        for(i=0;i<m->triangles_.size();i++) {
            double korak, alpha, beta, gamma;
            Eigen::Vector3d r1,r2,r3;
            double r1r2,r2r3,r1r3,maxL;

            r1=m->vertices_[m->triangles_[i][0]] - minV;
            r2=m->vertices_[m->triangles_[i][1]] - minV;
            r3=m->vertices_[m->triangles_[i][2]] - minV;

            r1r2=QVector3D(r2[0]-r1[0],r2[1]-r1[1],r2[2]-r1[2]).length();
            r1r3=QVector3D(r3[0]-r1[0],r3[1]-r1[1],r3[2]-r1[2]).length();
            r2r3=QVector3D(r3[0]-r2[0],r3[1]-r2[1],r3[2]-r2[2]).length();

            maxL=r1r2 > r1r3 ? r1r2 : r1r3;
            maxL=maxL > r2r3 ? maxL : r2r3;
            maxL*=2;
            korak=1.0-(maxL-(voxel_size / 1.0)) / maxL;

            for(alpha=0;alpha<1;alpha+=korak)
                for(beta=0;beta<1;beta+=korak) {
                    if(alpha + beta > 1)
                        break;
                    gamma=1-alpha-beta;
                    voxelCoords(r1,r2,r3,alpha,beta,gamma,voxel_size);
                }
            voxelCoords(r1,r2,r3,1,0,0,voxel_size);
            voxelCoords(r1,r2,r3,0,1,0,voxel_size);
        }
    }

    for(i=0;i<voxSX;i++)
        for(j=0;j<voxSY;j++)
            for(k=0;k<voxSZ;k++)
                if(vox[i][j][k] == 1) {
                    Eigen::Vector3d v(i,j,k);
                    voxelGrid.push_back(v);
                }

    calcVoxCenter();

    splitVoxelsSymetryPlane();
}

void VoxelGrid::voxelCoords(Eigen::Vector3d r1, Eigen::Vector3d r2, Eigen::Vector3d r3, double alpha, double beta, double gamma, double vox_size) {
    Eigen::Vector3d p=alpha * r1 + beta * r2 + gamma * r3;

    int x=(int)round(p[0] / vox_size),y=(int)round(p[1] / vox_size),z=(int)round(p[2] / vox_size);
    if(x < 0)
        x=0;
    if(x >= voxSX)
        x=voxSX - 1;
    if(y < 0)
        y=0;
    if(y >= voxSY)
        y=voxSY - 1;
    if(z < 0)
        z=0;
    if(z >= voxSZ)
        z=voxSZ - 1;

/*    omp_set_lock(&mutex);
    if(vox[x][y][z] == 0) {
        Eigen::Vector3d v(x,y,z);
        voxelGrid.push_back(v);
        vox[x][y][z] = 1;
    }
    omp_unset_lock(&mutex);*/
    vox[x][y][z] = 1;
}

const Eigen::Vector3d &VoxelGrid::getMinPoint() const
{
    return minPoint;
}

int VoxelGrid::getVoxSZ() const
{
    return voxSZ;
}

int VoxelGrid::getVoxSY() const
{
    return voxSY;
}

int VoxelGrid::getVoxSX() const
{
    return voxSX;
}

std::vector<Eigen::Vector3d> *VoxelGrid::getVoxelGrid()
{
    return &voxelGrid;
}

std::vector<Eigen::Vector3d> *VoxelGrid::getVoxelGridL()
{
    return &voxelGridL;
}

std::vector<Eigen::Vector3d> *VoxelGrid::getVoxelGridR()
{
    return &voxelGridR;
}

std::vector<Eigen::Vector3d> *VoxelGrid::getVoxelGridLU()
{
    return &voxelGridLU;
}

std::vector<Eigen::Vector3d> *VoxelGrid::getVoxelGridRU()
{
    return &voxelGridRU;
}

std::vector<Eigen::Vector3d> *VoxelGrid::getVoxelGridLD()
{
    return &voxelGridLD;
}

std::vector<Eigen::Vector3d> *VoxelGrid::getVoxelGridRD()
{
    return &voxelGridRD;
}

std::vector<Eigen::Vector3d> *VoxelGrid::getVoxelGridSP()
{
    return &voxelGridSP;
}

const Eigen::Vector3d &VoxelGrid::getVoxCenter() const
{
    return voxCenter;
}

void VoxelGrid::splitVoxelsSymetryPlane() {
    int i;
    voxelGridL.clear();
    voxelGridR.clear();

    voxelGridLU.clear();
    voxelGridRU.clear();
    voxelGridLD.clear();
    voxelGridRD.clear();

    voxelGridSP.clear();
    QVector3D c;
    double r=VOX_RADIUS*fabs(normalSP.x()) + VOX_RADIUS*fabs(normalSP.y()) + VOX_RADIUS*fabs(normalSP.z()),s;
    for(i=0;i<voxelGrid.size();i++) {
        c=QVector3D(voxelGrid[i][0] + 0.5,voxelGrid[i][1] + 0.5,voxelGrid[i][2] + 0.5);
        s=QVector3D::dotProduct(normalSP,c) - dSP;
        if(fabs(s) <= r)
            voxelGridSP.push_back(voxelGrid[i]);
        else if(s < 0)
            voxelGridL.push_back(voxelGrid[i]);
        else
            voxelGridR.push_back(voxelGrid[i]);
    }

    for(i=0;i<voxelGridL.size();i++) {
        c=QVector3D(voxelGridL[i][0] + 0.5,voxelGridL[i][1] + 0.5,voxelGridL[i][2] + 0.5);
        s=QVector3D::dotProduct(normalSplitPlane,c) - dSplitPlane;
        if(s < 0)
            voxelGridLU.push_back(voxelGridL[i]);
        else
            voxelGridLD.push_back(voxelGridL[i]);
    }

    for(i=0;i<voxelGridR.size();i++) {
        c=QVector3D(voxelGridR[i][0] + 0.5,voxelGridR[i][1] + 0.5,voxelGridR[i][2] + 0.5);
        s=QVector3D::dotProduct(normalSplitPlane,c) - dSplitPlane;
        if(s < 0)
            voxelGridRU.push_back(voxelGridR[i]);
        else
            voxelGridRD.push_back(voxelGridR[i]);
    }
}

void VoxelGrid::calcVoxCenter() {
    int i;
    Eigen::Vector3d p(0,0,0);
    int n=0;

    for(i=0;i<voxelGrid.size();i++) {
        p[0]+=voxelGrid[i][0] + 0.5;
        p[1]+=voxelGrid[i][1] + 0.5;
        p[2]+=voxelGrid[i][2] + 0.5;
        n++;
    }

    p/=(double)n;
    voxCenter=p;
    displacedVoxCenter=p;
    double vx=voxSX + 1,vy=voxSY + 1, vz=voxSZ + 1;

    centerVectors[0]=Eigen::Vector3d(0,0,0) - voxCenter;
    centerVectors[1]=Eigen::Vector3d(vx,0,0) - voxCenter;
    centerVectors[2]=Eigen::Vector3d(0,vy,0) - voxCenter;
    centerVectors[3]=Eigen::Vector3d(vx,vy,0) - voxCenter;
    centerVectors[4]=Eigen::Vector3d(0,0,vz) - voxCenter;
    centerVectors[5]=Eigen::Vector3d(vx,0,vz) - voxCenter;
    centerVectors[6]=Eigen::Vector3d(0,vy,vz) - voxCenter;
    centerVectors[7]=Eigen::Vector3d(vx,vy,vz) - voxCenter;

    for(i=0;i<voxelGrid.size();i++) {
        voxelGrid[i] -= voxCenter;
//        voxelGrid[i][0] -= (int)voxCenter[0];
//        voxelGrid[i][1] -= (int)voxCenter[1];
//        voxelGrid[i][2] -= (int)voxCenter[2];
    }

    minPoint = voxCenter;

    voxCenter=Eigen::Vector3d(0,0,0);

    setSymetryPlane();
}

void VoxelGrid::setSymetryPlane() {
    normalSP.setX(sinTable[0] * sinTable[0]);
    normalSP.setY(cosTable[0]);
    normalSP.setZ(cosTable[0] * sinTable[0]);

    dSP=(voxCenter[0] * normalSP.x() + voxCenter[1] * normalSP.y() + voxCenter[2] * normalSP.z());

    normalSplitPlane.setX(sinTable[0] * sinTable[0+90]);
    normalSplitPlane.setY(cosTable[0+90]);
    normalSplitPlane.setZ(cosTable[0] * sinTable[0+90]);

    dSplitPlane=(voxCenter[0] * normalSplitPlane.x() + voxCenter[1] * normalSplitPlane.y() + voxCenter[2] * normalSplitPlane.z());
}

void VoxelGrid::rotateSPNormal(int fi, int theta) {
//    QVector3D pom(1,0,0);
//    QVector3D pom2(0,0,1);
//    QMatrix4x4 m;
//    m.setToIdentity();
//    m.rotate(rx, 1, 0, 0);
//    m.rotate(ry, 0, 1, 0);
//    m.rotate(rz, 0, 0, 1);

//    normalSP=m.map(pom);

//    normalSplitPlane = m.map(pom2);

    normalSP.setX(sinTable[theta] * sinTable[fi]);
    normalSP.setY(cosTable[fi]);
    normalSP.setZ(cosTable[theta] * sinTable[fi]);

    normalSplitPlane.setX(sinTable[theta] * sinTable[fi+90]);
    normalSplitPlane.setY(cosTable[fi+90]);
    normalSplitPlane.setZ(cosTable[theta] * sinTable[fi+90]);

    dSP=(voxCenter[0] * normalSP.x() + voxCenter[1] * normalSP.y() + voxCenter[2] * normalSP.z());

    dSplitPlane=(voxCenter[0] * normalSplitPlane.x() + voxCenter[1] * normalSplitPlane.y() + voxCenter[2] * normalSplitPlane.z());

    splitVoxelsSymetryPlane();
}

void VoxelGrid::recalculateVGSize(Mesh3D *mesh, int &sx, int &sy, int &sz, double voxel_size) {
    Eigen::Vector3d minB(mesh->getMinX(),mesh->getMinY(),mesh->getMinZ());
    Eigen::Vector3d maxB(mesh->getMaxX(),mesh->getMaxY(),mesh->getMaxZ());

    Eigen::Vector3d l=(maxB - minB) / voxel_size;

    sx=(int)(l.x());
    sy=(int)(l.y());
    sz=(int)(l.z());
}

void VoxelGrid::prepareVoxelSplit(QVector3D symN, std::vector<Eigen::Vector3d> *voxelL, std::vector<Eigen::Vector3d> *voxelR) {
    QVector3D c;
    int i;

    voxelL->clear();
    voxelR->clear();

    double r=VOX_RADIUS*fabs(symN.x()) + VOX_RADIUS*fabs(symN.y()) + VOX_RADIUS*fabs(symN.z()),s;
    for(i=0;i<voxelGrid.size();i++) {
        c=QVector3D(voxelGrid[i][0] + 0.5,voxelGrid[i][1] + 0.5,voxelGrid[i][2] + 0.5);
        s=QVector3D::dotProduct(symN,c);
        if(fabs(s) <= r)
            continue;
        else if(s < 0)
            voxelL->push_back(voxelGrid[i]);
        else
            voxelR->push_back(voxelGrid[i]);
    }
}

void VoxelGrid::prepareVoxelSplit(Vector3D symN, Vector3D splitN, int *indices, int &low, int &mid, int &high, double &dMin, double &dMax) {
    Vector3D c;
    int l=0,h=voxelGrid.size()-1,m=0;
    dMin=100000;
    dMax=-100000;
    double r=VOX_RADIUS*fabs(symN.x) + VOX_RADIUS*fabs(symN.y) + VOX_RADIUS*fabs(symN.z),s,d;
    while(m <= h) {
        c.x=voxelGrid[indices[m]][0] + 0.5;
        c.y=voxelGrid[indices[m]][1] + 0.5;
        c.z=voxelGrid[indices[m]][2] + 0.5;
        s=symN.x * c.x + symN.y * c.y + symN.z * c.z;
        d=splitN.x * c.x + splitN.y * c.y + splitN.z * c.z;
        if(dMin > d)
            dMin=d;
        if(dMax < d)
            dMax=d;

        if(fabs(s) <= r) {
            m++;
        }
        else if(s < 0) {
            swapValues(indices[l], indices[m]);
            l++;
            m++;
        }
        else {
            swapValues(indices[h], indices[m]);
            h--;
        }
    }
    low=l;
    mid=m;
    high=h;
}

void VoxelGrid::swapValues(int &a, int &b) {
    int t=a;
    a=b;
    b=t;
}

void VoxelGrid::makeSplitForVis(QVector3D n, std::vector<Eigen::Vector3d> &voxelL, std::vector<Eigen::Vector3d> &voxelR, std::vector<Eigen::Vector3d> &voxelSP) {
    QVector3D c;
    int i;
    double r=VOX_RADIUS*fabs(n.x()) + VOX_RADIUS*fabs(n.y()) + VOX_RADIUS*fabs(n.z()),s;
    for(i=0;i<voxelGrid.size();i++) {
        c=QVector3D(voxelGrid[i][0] + 0.5,voxelGrid[i][1] + 0.5,voxelGrid[i][2] + 0.5);
        s=QVector3D::dotProduct(n,c);
        if(fabs(s) <= r)
            voxelSP.push_back(voxelGrid[i]);
        else if(s < 0)
            voxelL.push_back(voxelGrid[i]);
        else
            voxelR.push_back(voxelGrid[i]);
    }
}

Eigen::Vector3d *VoxelGrid::getCenterVectors() {
    return &centerVectors[0];
}

void VoxelGrid::generateCoarseVoxelGrid(VoxelGrid *fineGrid, int factor) {
    int i,j,k;
    if(vox != nullptr) {
        for(i=0;i<voxSX;i++) {
            for(j=0;j<voxSY;j++)
                delete[] vox[i][j];
            delete[] vox[i];
        }
        delete[] vox;
        vox=nullptr;
    }

    voxSX=(fineGrid->getVoxSX() / factor) + 1;
    voxSY=(fineGrid->getVoxSY() / factor) + 1;
    voxSZ=(fineGrid->getVoxSZ() / factor) + 1;

    vox=new int**[voxSX];
    for(i=0;i<voxSX;i++) {
        vox[i]=new int*[voxSY];
        for(j=0;j<voxSY;j++) {
            vox[i][j]=new int[voxSZ];
            for(k=0;k<voxSZ;k++)
                vox[i][j][k]=0;
        }
    }

    std::vector<Eigen::Vector3d> *fGrid=fineGrid->getVoxelGrid();
    Eigen::Vector3d minP=fineGrid->getMinPoint(),p;
    for(i=0;i<fGrid->size();i++) {
        p=fGrid->at(i) + minP;
        int x=p[0] / factor,y=p[1] / factor,z=p[2] / factor;
        vox[x][y][z]=1;
    }

    voxelGrid.clear();
    for(i=0;i<voxSX;i++)
        for(j=0;j<voxSY;j++)
            for(k=0;k<voxSZ;k++)
                if(vox[i][j][k] == 1) {
                    Eigen::Vector3d v(i,j,k);
                    voxelGrid.push_back(v);
                }

    calcVoxCenter();
}

Eigen::Vector3d *VoxelGrid::getVoxelGridData() {
    return voxelGrid.data();
}
