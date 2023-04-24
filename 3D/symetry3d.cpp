#include "symetry3d.h"
#include "constants.h"
#include <QMatrix4x4>
#include <iostream>
#include <math.h>
#include <omp.h>

Symetry3D::Symetry3D()
{
    int i;
    for(i=0;i<361;i++) {
        sinTable[i]=round_off(sin(i * PI / 180.0),5);
        cosTable[i]=round_off(cos(i * PI / 180.0),5);
    }

    omp_init_lock(&mutex);
    omp_init_lock(&mutex2);
}

Symetry3D::~Symetry3D() {
    omp_destroy_lock(&mutex);
    omp_destroy_lock(&mutex2);
}

void Symetry3D::insertSymPlane(std::vector<SymetryPlaneData> *planes, SymetryPlaneData n) {
    omp_set_lock(&mutex);
    planes->push_back(n);
    omp_unset_lock(&mutex);
}

void Symetry3D::insertSymPlane(std::vector<Eigen::Vector2i> *planes, Eigen::Vector2i n) {
    omp_set_lock(&mutex2);
    planes->push_back(n);
    omp_unset_lock(&mutex2);
}

double Symetry3D::round_off(double val, unsigned int prec) {
    double pow_10 = pow(10.0f, (double)prec);
    if(fabs(val) < 1E-7)
        val=0;
    return round(val * pow_10) / pow_10;
}

Vector3D Symetry3D::calcVoxCOM(std::vector<Eigen::Vector3d> voxel) {
    int i;
    Vector3D v;

    if(voxel.size() < 1)
        return v;

    int n=0;

    for(i=0;i<voxel.size();i++) {
        v.x+=voxel[i][0] + 0.5;
        v.y+=voxel[i][1] + 0.5;
        v.z+=voxel[i][2] + 0.5;
        n++;
    }
    v.x/=(double)n;
    v.y/=(double)n;
    v.z/=(double)n;

    return v;
}

Vector3D Symetry3D::calcVoxCOM(Eigen::Vector3d *voxGrid, int *indices, int first, int last, bool &calcTrue) {
    int i;
    Vector3D v;
    calcTrue=true;

    if(last - first < 0) {
        calcTrue=false;
        return v;
    }

    int n=0;

    for(i=first;i<=last;i++) {
        v.x+=voxGrid[indices[i]][0] + 0.5;
        v.y+=voxGrid[indices[i]][1] + 0.5;
        v.z+=voxGrid[indices[i]][2] + 0.5;
        n++;
    }
    v.x/=(double)n;
    v.y/=(double)n;
    v.z/=(double)n;

    return v;
}

void Symetry3D::swapValues(int &a, int &b) {
    int t=a;
    a=b;
    b=t;
}

std::vector<Symetry3DDebugData> *Symetry3D::getDebugData()
{
    return &debugData;
}

bool Symetry3D::splitUpDown(Eigen::Vector3d *voxGrid, int *indices, int first, int last, Vector3D splitN, double dMean, int &splitIndex) {
    int l=first, h=last;
    double s;
    Vector3D c;
    while(l <= h) {
        c.x=voxGrid[indices[l]][0] + 0.5;
        c.y=voxGrid[indices[l]][1] + 0.5;
        c.z=voxGrid[indices[l]][2] + 0.5;
        s=splitN.x*c.x + splitN.y*c.y + splitN.z*c.z - dMean;
        if(fabs(s) < 0.5)
            s=0;

        if(s < 0) l++;
        else {
            swapValues(indices[l],indices[h]);
            h--;
        }
    }
    splitIndex=h + 1;
    if(l > last || h < first)
        return false;
    return true;
}

bool Symetry3D::checkSymetry(Eigen::Vector3d *voxGrid, int *indices, int leftF, int leftL, int rightF, int rightL, Vector3D symN, Vector3D splitN, double dMin, double dMax, double tolerance, Vector3D &diffVect) {
    //Two empty sets are symetrical
    if((leftL - leftF < 1 && rightL - rightF < 1))
        return true;

    if(fabs(dMax - dMin) < 2)
        return true;

    int splitIndL, splitIndR;
    bool splitL,splitR;
    Vector3D lCOM=calcVoxCOM(voxGrid, indices, leftF, leftL, splitL),rCOM=calcVoxCOM(voxGrid, indices, rightF, rightL, splitR), lRefl;

    if(splitL && splitR) {
        lRefl.x=(1-2*symN.x*symN.x)*lCOM.x    -2*symN.x*symN.y  *lCOM.y     -2*symN.x*symN.z *lCOM.z;
        lRefl.y=  -2*symN.x*symN.y *lCOM.x + (1-2*symN.y*symN.y)*lCOM.y     -2*symN.y*symN.z *lCOM.z;
        lRefl.z=  -2*symN.x*symN.z *lCOM.x     -2*symN.z*symN.y *lCOM.y + (1-2*symN.z*symN.z)*lCOM.z;

        if(lRefl.distanceToPoint(rCOM) > tolerance) {
            diffVect.x = lRefl.x - rCOM.x;
            diffVect.y = lRefl.y - rCOM.y;
            diffVect.z = lRefl.z - rCOM.z;
            return false;
        }
    }
    else if(splitL) {
        double dist=symN.x * lCOM.x + symN.y * lCOM.y + symN.z * lCOM.z;
        if(fabs(dist) > tolerance) {
            diffVect.x = dist * symN.x;
            diffVect.y = dist * symN.y;
            diffVect.z = dist * symN.z;
            return false;
        }
    }
    else {
        double dist=symN.x * rCOM.x + symN.y * rCOM.y + symN.z * rCOM.z;
        if(fabs(dist) > tolerance) {
            diffVect.x = dist * symN.x;
            diffVect.y = dist * symN.y;
            diffVect.z = dist * symN.z;
            return false;
        }
    }

    if(fabs(dMax-dMin) > SPLIT3D_LIMIT) {
        double dMean=(dMax + dMin) / 2.0;

        splitL=splitUpDown(voxGrid, indices, leftF, leftL, splitN, dMean,splitIndL);
        splitR=splitUpDown(voxGrid, indices, rightF, rightL, splitN, dMean,splitIndR);

        if(!splitL && !splitR)
            return true;

        if(!checkSymetry(voxGrid,indices,leftF,splitIndL-1,rightF,splitIndR-1,symN,splitN,dMin,dMean,tolerance,diffVect)) {
            return false;
        }

        if(!checkSymetry(voxGrid,indices,splitIndL,leftL,splitIndR,rightL,symN,splitN,dMean,dMax,tolerance,diffVect)) {
            diffVect.x = lRefl.x - rCOM.x;
            diffVect.y = lRefl.y - rCOM.y;
            diffVect.z = lRefl.z - rCOM.z;
            return false;
        }
    }

    return true;
}

bool Symetry3D::checkSymetryNonRecursiveDebugData(Eigen::Vector3d *voxGrd, int N, Vector3D symN, Vector3D splitN, double dMin, double dMax, double tolerance, Vector3D &diffVect) {
    int M,i,index;
    double d;
    M=(int)((dMax - dMin) / SPLIT3D_LIMIT) + 1;

    SymClass *classesL=new SymClass[M],*classesR=new SymClass[M];

    Vector3D v,lRefl,rRefl;

    double r=VOX_RADIUS*fabs(symN.x) + VOX_RADIUS*fabs(symN.y) + VOX_RADIUS*fabs(symN.z),s;

    for(i=0;i<M;i++) {
        Symetry3DDebugData data;
        data.voxelsL.clear();
        data.voxelsR.clear();
        data.dMax=-10000;
        data.dMin=100000;
        classesL[i].counter=0;
        classesR[i].counter=0;
        debugData.push_back(data);
    }

    for(i=0;i<N;i++) {
        v.x=voxGrd[i][0] + 0.5;
        v.y=voxGrd[i][1] + 0.5;
        v.z=voxGrd[i][2] + 0.5;

        s=symN.x * v.x + symN.y * v.y + symN.z * v.z;

        if(fabs(s) <= r) {
            //continue;
            d=splitN.x * v.x + splitN.y * v.y + splitN.z * v.z;

            if(d>dMax)
                d=dMax;
            if(d<dMin)
                d=dMin;

            index=(int)((d-dMin) / SPLIT3D_LIMIT);

            if(index > 1 && index == M-1)
                index=M-2;

            if(d < debugData[index].dMin)
                debugData[index].dMin=d;
            if(d > debugData[index].dMax)
                debugData[index].dMax=d;

            if(index < 0 || index >= M)
                std::cout << "Napacen index: " << index << std::endl;

            classesL[index].COG.x += v.x;
            classesL[index].COG.y += v.y;
            classesL[index].COG.z += v.z;
            classesL[index].counter++;

            classesR[index].COG.x += v.x;
            classesR[index].COG.y += v.y;
            classesR[index].COG.z += v.z;
            classesR[index].counter++;
        }
        //left
        else if(s < 0) {
            d=splitN.x * v.x + splitN.y * v.y + splitN.z * v.z;

            if(d>dMax)
                d=dMax;
            if(d<dMin)
                d=dMin;

            index=(int)((d-dMin) / SPLIT3D_LIMIT);

            if(index > 1 && index == M-1)
                index=M-2;

            if(d < debugData[index].dMin)
                debugData[index].dMin=d;
            if(d > debugData[index].dMax)
                debugData[index].dMax=d;

            if(index < 0 || index >= M)
                std::cout << "Napacen index: " << index << std::endl;

            classesL[index].COG.x += v.x;
            classesL[index].COG.y += v.y;
            classesL[index].COG.z += v.z;
            classesL[index].counter++;

            Eigen::Vector3d vect(v.x,v.y,v.z);
            debugData[index].voxelsL.push_back(vect);
        }
        //right
        else {
            d=splitN.x * v.x + splitN.y * v.y + splitN.z * v.z;

            if(d>dMax)
                d=dMax;
            if(d<dMin)
                d=dMin;

            index=(int)((d-dMin) / SPLIT3D_LIMIT);

            if(index > 1 && index == M-1)
                index=M-2;

            if(d < debugData[index].dMin)
                debugData[index].dMin=d;
            if(d > debugData[index].dMax)
                debugData[index].dMax=d;

            if(index < 0 || index >= M)
                std::cout << "Napacen index: " << index << std::endl;

            classesR[index].COG.x += v.x;
            classesR[index].COG.y += v.y;
            classesR[index].COG.z += v.z;
            classesR[index].counter++;

            Eigen::Vector3d vect(v.x,v.y,v.z);
            debugData[index].voxelsR.push_back(vect);
        }
    }

    for(i=0;i<M;i++) {
        if(classesL[i].counter > 0) {
            classesL[i].COG.x /= (double)classesL[i].counter;
            classesL[i].COG.y /= (double)classesL[i].counter;
            classesL[i].COG.z /= (double)classesL[i].counter;
        }
        if(classesR[i].counter > 0) {
            classesR[i].COG.x /= (double)classesR[i].counter;
            classesR[i].COG.y /= (double)classesR[i].counter;
            classesR[i].COG.z /= (double)classesR[i].counter;
        }
    }


    for(i=0;i<M;i++) {
        if(classesL[i].counter != 0 && classesR[i].counter != 0) {
            lRefl.x=(1-2*symN.x*symN.x)*classesL[i].COG.x     -2*symN.x*symN.y *classesL[i].COG.y     -2*symN.x*symN.z *classesL[i].COG.z;
            lRefl.y=  -2*symN.x*symN.y *classesL[i].COG.x + (1-2*symN.y*symN.y)*classesL[i].COG.y     -2*symN.y*symN.z *classesL[i].COG.z;
            lRefl.z=  -2*symN.x*symN.z *classesL[i].COG.x     -2*symN.z*symN.y *classesL[i].COG.y + (1-2*symN.z*symN.z)*classesL[i].COG.z;

            rRefl.x=(1-2*symN.x*symN.x)*classesR[i].COG.x     -2*symN.x*symN.y *classesR[i].COG.y     -2*symN.x*symN.z *classesR[i].COG.z;
            rRefl.y=  -2*symN.x*symN.y *classesR[i].COG.x + (1-2*symN.y*symN.y)*classesR[i].COG.y     -2*symN.y*symN.z *classesR[i].COG.z;
            rRefl.z=  -2*symN.x*symN.z *classesR[i].COG.x     -2*symN.z*symN.y *classesR[i].COG.y + (1-2*symN.z*symN.z)*classesR[i].COG.z;

            debugData[i].lCOM=classesL[i].COG;
            debugData[i].rCOM=classesR[i].COG;
            debugData[i].dist=lRefl.distanceToPoint(classesR[i].COG);
        }
        else if(classesL[i].counter != 0) {
            double d = symN.x * classesL[i].COG.x + symN.y * classesL[i].COG.y + symN.z * classesL[i].COG.z;
            debugData[i].lCOM=classesL[i].COG;
            debugData[i].rCOM=classesR[i].COG;
            debugData[i].dist=fabs(d);
        }
        else if(classesR[i].counter != 0) {
            double d=symN.x * classesR[i].COG.x + symN.y * classesR[i].COG.y + symN.z * classesR[i].COG.z;
            debugData[i].lCOM=classesL[i].COG;
            debugData[i].rCOM=classesR[i].COG;
            debugData[i].dist=fabs(d);
        }
    }

    delete[] classesL;
    delete[] classesR;
    return true;
}

bool Symetry3D::checkSymetryNonRecursive(Eigen::Vector3d *voxGrd, int N, Vector3D symN, Vector3D splitN, double dMin, double dMax, double tolerance, Vector3D &diffVect) {
    int M,i,index;
    double d;
    M=(int)((dMax - dMin) / SPLIT3D_LIMIT) + 1;

    SymClass *classesL=new SymClass[M],*classesR=new SymClass[M];

    Vector3D v,lRefl;

    double r=VOX_RADIUS*fabs(symN.x) + VOX_RADIUS*fabs(symN.y) + VOX_RADIUS*fabs(symN.z),s;

    for(i=0;i<M;i++) {
        classesL[i].counter=0;
        classesR[i].counter=0;
    }

    for(i=0;i<N;i++) {
        v.x=voxGrd[i][0] + 0.5;
        v.y=voxGrd[i][1] + 0.5;
        v.z=voxGrd[i][2] + 0.5;

        s=symN.x * v.x + symN.y * v.y + symN.z * v.z;

        if(fabs(s) <= r) {
            //continue;
            d=splitN.x * v.x + splitN.y * v.y + splitN.z * v.z;

            if(d>dMax)
                d=dMax;
            if(d<dMin)
                d=dMin;

            index=(int)((d-dMin) / SPLIT3D_LIMIT);

            if(index > 1 && index == M-1)
                index=M-2;

            if(index < 0 || index >= M)
                std::cout << "Napacen index: " << index << std::endl;

            classesL[index].COG.x += v.x;
            classesL[index].COG.y += v.y;
            classesL[index].COG.z += v.z;
            classesL[index].counter++;

            classesR[index].COG.x += v.x;
            classesR[index].COG.y += v.y;
            classesR[index].COG.z += v.z;
            classesR[index].counter++;
        }
        //left
        else if(s < 0) {
            d=splitN.x * v.x + splitN.y * v.y + splitN.z * v.z;

            if(d>dMax)
                d=dMax;
            if(d<dMin)
                d=dMin;

            index=(int)((d-dMin) / SPLIT3D_LIMIT);

            if(index > 1 && index == M-1)
                index=M-2;

            if(index < 0 || index >= M)
                std::cout << "Napacen index: " << index << std::endl;

            classesL[index].COG.x += v.x;
            classesL[index].COG.y += v.y;
            classesL[index].COG.z += v.z;
            classesL[index].counter++;
        }
        //right
        else {
            d=splitN.x * v.x + splitN.y * v.y + splitN.z * v.z;

            if(d>dMax)
                d=dMax;
            if(d<dMin)
                d=dMin;

            index=(int)((d-dMin) / SPLIT3D_LIMIT);

            if(index > 1 && index == M-1)
                index=M-2;

            if(index < 0 || index >= M)
                std::cout << "Napacen index: " << index << std::endl;

            classesR[index].COG.x += v.x;
            classesR[index].COG.y += v.y;
            classesR[index].COG.z += v.z;
            classesR[index].counter++;
        }
    }

    for(i=0;i<M;i++) {
        if(classesL[i].counter > 0) {
            classesL[i].COG.x /= (double)classesL[i].counter;
            classesL[i].COG.y /= (double)classesL[i].counter;
            classesL[i].COG.z /= (double)classesL[i].counter;
        }
        if(classesR[i].counter > 0) {
            classesR[i].COG.x /= (double)classesR[i].counter;
            classesR[i].COG.y /= (double)classesR[i].counter;
            classesR[i].COG.z /= (double)classesR[i].counter;
        }
    }


    for(i=0;i<M;i++) {
        if(classesL[i].counter != 0 && classesR[i].counter != 0) {
            lRefl.x=(1-2*symN.x*symN.x)*classesL[i].COG.x     -2*symN.x*symN.y *classesL[i].COG.y     -2*symN.x*symN.z *classesL[i].COG.z;
            lRefl.y=  -2*symN.x*symN.y *classesL[i].COG.x + (1-2*symN.y*symN.y)*classesL[i].COG.y     -2*symN.y*symN.z *classesL[i].COG.z;
            lRefl.z=  -2*symN.x*symN.z *classesL[i].COG.x     -2*symN.z*symN.y *classesL[i].COG.y + (1-2*symN.z*symN.z)*classesL[i].COG.z;

            v.x=0;//(classesL[i].COG.x * classesL[i].counter + classesR[i].COG.x  * classesR[i].counter) / ((double)(classesL[i].counter + classesR[i].counter));
            v.y=0;//(classesL[i].COG.y * classesL[i].counter + classesR[i].COG.y  * classesR[i].counter) / ((double)(classesL[i].counter + classesR[i].counter));
            v.z=0;//(classesL[i].COG.z * classesL[i].counter + classesR[i].COG.z  * classesR[i].counter) / ((double)(classesL[i].counter + classesR[i].counter));

            if(lRefl.distanceToPoint(classesR[i].COG) > tolerance || fabs(v.x*symN.x + v.y*symN.y + v.z * symN.z) > tolerance) {
                diffVect.x = lRefl.x - classesR[i].COG.x;
                diffVect.y = lRefl.y - classesR[i].COG.y;
                diffVect.z = lRefl.z - classesR[i].COG.z;

                delete[] classesL;
                delete[] classesR;
                return false;
            }
        }
        else if(classesL[i].counter != 0) {
            double d = symN.x * classesL[i].COG.x + symN.y * classesL[i].COG.y + symN.z * classesL[i].COG.z;
            if(fabs(d) > tolerance) {
                double sgn=d / fabs(d);
                diffVect.x = sgn * symN.x;
                diffVect.y = sgn * symN.y;
                diffVect.z = sgn * symN.z;

                delete[] classesL;
                delete[] classesR;
                return false;
            }
        }
        else if(classesR[i].counter != 0) {
            double d=symN.x * classesR[i].COG.x + symN.y * classesR[i].COG.y + symN.z * classesR[i].COG.z;
            if(fabs(d) > tolerance) {
                double sgn=d / fabs(d);
                diffVect.x = sgn * symN.x;
                diffVect.y = sgn * symN.y;
                diffVect.z = sgn * symN.z;

                delete[] classesL;
                delete[] classesR;
                return false;
            }
        }
    }

    delete[] classesL;
    delete[] classesR;
    return true;
}

void Symetry3D::calcDMinDMax(Eigen::Vector3d *voxGrd, int N, Vector3D splitN, double &dMin, double &dMax) {
    int i;
    Vector3D v;
    double s;

    dMin=100000;
    dMax=-100000;

    for(i=0;i<N;i++) {
        v.x=voxGrd[i][0] + 0.5;
        v.y=voxGrd[i][1] + 0.5;
        v.z=voxGrd[i][2] + 0.5;

        s=splitN.x * v.x + splitN.y * v.y + splitN.z * v.z;

        if(s < dMin)
            dMin=s;
        if(s > dMax)
            dMax=s;
    }
}

bool Symetry3D::checkSymetryDebug(Eigen::Vector3d *voxGrid, int *indices, int leftF, int leftL, int rightF, int rightL, Vector3D symN, Vector3D splitN, double dMin, double dMax, double tolerance, Vector3D &diffVect) {
    //Two empty sets are symetrical
    if((leftL - leftF < 1 && rightL - rightF < 1))
        return true;

    if(fabs(dMax - dMin) < 2)
        return true;

    bool splitL, splitR;
    int splitIndL,splitIndR;
    Vector3D lCOM=calcVoxCOM(voxGrid, indices, leftF, leftL,splitL),rCOM=calcVoxCOM(voxGrid, indices, rightF, rightL,splitR), lRefl,rRefl;

    double dist=0;

    if(splitL && splitR) {
        lRefl.x=(1-2*symN.x*symN.x)*lCOM.x    -2*symN.x*symN.y  *lCOM.y     -2*symN.x*symN.z *lCOM.z;
        lRefl.y=  -2*symN.x*symN.y *lCOM.x + (1-2*symN.y*symN.y)*lCOM.y     -2*symN.y*symN.z *lCOM.z;
        lRefl.z=  -2*symN.x*symN.z *lCOM.x     -2*symN.z*symN.y *lCOM.y + (1-2*symN.z*symN.z)*lCOM.z;

        rRefl.x=(1-2*symN.x*symN.x)*rCOM.x    -2*symN.x*symN.y  *rCOM.y     -2*symN.x*symN.z *rCOM.z;
        rRefl.y=  -2*symN.x*symN.y *rCOM.x + (1-2*symN.y*symN.y)*rCOM.y     -2*symN.y*symN.z *rCOM.z;
        rRefl.z=  -2*symN.x*symN.z *rCOM.x     -2*symN.z*symN.y *rCOM.y + (1-2*symN.z*symN.z)*rCOM.z;

        dist=lRefl.distanceToPoint(rCOM);
    }
    else if(splitL) {
        lRefl.x=(1-2*symN.x*symN.x)*lCOM.x    -2*symN.x*symN.y  *lCOM.y     -2*symN.x*symN.z *lCOM.z;
        lRefl.y=  -2*symN.x*symN.y *lCOM.x + (1-2*symN.y*symN.y)*lCOM.y     -2*symN.y*symN.z *lCOM.z;
        lRefl.z=  -2*symN.x*symN.z *lCOM.x     -2*symN.z*symN.y *lCOM.y + (1-2*symN.z*symN.z)*lCOM.z;

        rRefl.x=0;
        rRefl.y=0;
        rRefl.z=0;

        dist=fabs(symN.x * lCOM.x + symN.y * lCOM.y + symN.z * lCOM.z);
    }
    else {
        lRefl.x=0;
        lRefl.y=0;
        lRefl.z=0;

        rRefl.x=(1-2*symN.x*symN.x)*rCOM.x    -2*symN.x*symN.y  *rCOM.y     -2*symN.x*symN.z *rCOM.z;
        rRefl.y=  -2*symN.x*symN.y *rCOM.x + (1-2*symN.y*symN.y)*rCOM.y     -2*symN.y*symN.z *rCOM.z;
        rRefl.z=  -2*symN.x*symN.z *rCOM.x     -2*symN.z*symN.y *rCOM.y + (1-2*symN.z*symN.z)*rCOM.z;

        dist=fabs(symN.x * rCOM.x + symN.y * rCOM.y + symN.z * rCOM.z);
    }

    Symetry3DDebugData data;
    data.lCOM=lCOM;
    data.rCOM=rCOM;
    data.lRefl=lRefl;
    data.rRefl=rRefl;
    data.dMin=dMin;
    data.dMax=dMax;
    data.dist=dist;
    data.voxelsL.clear();
    data.voxelsR.clear();
    int i;
    for(i=leftF;i<=leftL;i++)
        data.voxelsL.push_back(voxGrid[indices[i]]);
    for(i=rightF;i<=rightL;i++)
        data.voxelsR.push_back(voxGrid[indices[i]]);

    debugData.push_back(data);

    if(fabs(dMax-dMin) > SPLIT3D_LIMIT) {
        double dMean=(dMax + dMin) / 2.0;

        splitL=splitUpDown(voxGrid, indices, leftF, leftL, splitN, dMean,splitIndL);
        splitR=splitUpDown(voxGrid, indices, rightF, rightL, splitN, dMean,splitIndR);

        if(!splitL && !splitR)
            return true;

        if(!checkSymetryDebug(voxGrid,indices,leftF,splitIndL-1,rightF,splitIndR-1,symN,splitN,dMin,dMean,tolerance,diffVect)) {
            diffVect.x = lRefl.x - rCOM.x;
            diffVect.y = lRefl.y - rCOM.y;
            diffVect.z = lRefl.z - rCOM.z;
            return false;
        }

        if(!checkSymetryDebug(voxGrid,indices,splitIndL,leftL,splitIndR,rightL,symN,splitN,dMean,dMax,tolerance,diffVect)) {
            diffVect.x = lRefl.x - rCOM.x;
            diffVect.y = lRefl.y - rCOM.y;
            diffVect.z = lRefl.z - rCOM.z;
            return false;
        }
    }
    return true;
}

bool Symetry3D::checkSymetry(std::vector<Eigen::Vector3d> voxelL, std::vector<Eigen::Vector3d> voxelR, Vector3D symN,
                          Vector3D splitN, double dMin, double dMax, double tolerance, Vector3D &diffVect) {
    //Two empty sets are symetrical
    if((voxelL.size() == 0 && voxelR.size() == 0))
        return true;

    if(fabs(dMax - dMin) < SPLIT3D_LIMIT)
        return true;

    Vector3D lCOM=calcVoxCOM(voxelL),rCOM=calcVoxCOM(voxelR), lRefl;

    lRefl.x=(1-2*symN.x*symN.x)*lCOM.x    -2*symN.x*symN.y  *lCOM.y     -2*symN.x*symN.z *lCOM.z;
    lRefl.y=  -2*symN.x*symN.y *lCOM.x + (1-2*symN.y*symN.y)*lCOM.y     -2*symN.y*symN.z *lCOM.z;
    lRefl.z=  -2*symN.x*symN.z *lCOM.x     -2*symN.z*symN.y *lCOM.y + (1-2*symN.z*symN.z)*lCOM.z;

    int i;

    std::vector<Eigen::Vector3d> voxelLU,voxelLD,voxelRU,voxelRD;
    Vector3D c;

    if(lRefl.distanceToPoint(rCOM) > tolerance) {
        diffVect.x = lRefl.x - rCOM.x;
        diffVect.y = lRefl.y - rCOM.y;
        diffVect.z = lRefl.z - rCOM.z;
        return false;
    }

    if(fabs(dMax-dMin) > SPLIT3D_LIMIT) {
        double dMean=(dMax + dMin) / 2.0;

        voxelLU.clear();
        voxelRU.clear();
        voxelLD.clear();
        voxelRD.clear();

        double s;
        for(i=0;i<voxelL.size();i++) {
            c.x=voxelL[i][0] + 0.5;
            c.y=voxelL[i][1] + 0.5;
            c.z=voxelL[i][2] + 0.5;
            s=splitN.x * c.x + splitN.y * c.y + splitN.z * c.z - dMean;

            if(fabs(s) < 1E-5)
                s=0;

            if(s < 0)
                voxelLD.push_back(voxelL[i]);
            else
                voxelLU.push_back(voxelL[i]);
        }

        for(i=0;i<voxelR.size();i++) {
            c.x=voxelR[i][0] + 0.5;
            c.y=voxelR[i][1] + 0.5;
            c.z=voxelR[i][2] + 0.5;
            s=splitN.x * c.x + splitN.y * c.y + splitN.z * c.z - dMean;

            if(fabs(s) < 1E-5)
                s=0;

            if(s < 0)
                voxelRD.push_back(voxelR[i]);
            else
                voxelRU.push_back(voxelR[i]);
        }

        if(!checkSymetry(voxelLD,voxelRD,symN,splitN,dMin,dMean,tolerance, diffVect)) {
//            diffVect = lRefl - rCOM;
            return false;
        }
        if(!checkSymetry(voxelLU,voxelRU,symN,splitN,dMean,dMax,tolerance, diffVect)) {
//            diffVect = lRefl - rCOM;
            return false;
        }
    }
    return true;
}

std::vector<SymetryPlaneData> Symetry3D::refineSymetries(VoxelGrid *voxGrid, double tolerance, std::vector<SymetryPlaneData> symNorms) {
    if(tolerance < MIN_TOLERANCE)
        tolerance=MIN_TOLERANCE;
    std::vector<SymetryPlaneData> symNormals;
    symNormals.clear();

    int N=voxGrid->getVoxelGrid()->size();

    #pragma omp parallel for
    for(int i=0;i<symNorms.size();i++) {
        int *indices=new int[N];
        for(int j=0;j<N;j++)
            indices[j]=j;

        Vector3D symN,diffVect;
        if(checkSymetryAtPlane(voxGrid,indices,N,tolerance,symNorms[i].angles[0],symNorms[i].angles[1],symN,diffVect)) {
            SymetryPlaneData d;
            d.angles=symNorms[i].angles;
            d.normal=symN;
            insertSymPlane(&symNormals,d);
        }

        delete[] indices;
    }
    return symNormals;
}

std::vector<SymetryPlaneData> Symetry3D::refineSymetries_SingleThread(VoxelGrid *voxGrid, double tolerance, std::vector<SymetryPlaneData> symNorms) {
    if(tolerance < MIN_TOLERANCE)
        tolerance=MIN_TOLERANCE;
    std::vector<SymetryPlaneData> symNormals;
    symNormals.clear();

    int N=voxGrid->getVoxelGrid()->size();

    #pragma omp parallel for
    for(int i=0;i<symNorms.size();i++) {
        int *indices=new int[N];
        for(int j=0;j<N;j++)
            indices[j]=j;

        Vector3D symN,diffVect;
        if(checkSymetryAtPlane(voxGrid,indices,N,tolerance,symNorms[i].angles[0],symNorms[i].angles[1],symN,diffVect)) {
            SymetryPlaneData d;
            d.angles=symNorms[i].angles;
            d.normal=symN;
            insertSymPlane(&symNormals,d);
        }

        delete[] indices;
    }
    return symNormals;
}

std::vector<SymetryPlaneData> Symetry3D::determineSymetriesDC(VoxelGrid *voxGrid, double tolerance, int angleStep, int minAngleDiff) {
    if(tolerance < MIN_TOLERANCE)
        tolerance=MIN_TOLERANCE;
    std::vector<SymetryPlaneData> symNormals;
    symNormals.clear();

    int N=voxGrid->getVoxelGrid()->size();

    #pragma omp parallel for
    for(int fi=0;fi<=90;fi+=angleStep) {

        int *indices=new int[N];
        for(int j=0;j<N;j++)
            indices[j]=j;

        Vector3D symN, diffVects[4];

        if(fi == 0) {
            if(checkSymetryAtPlane(voxGrid, indices, N, tolerance, 0, 0, symN, diffVects[0])) {
                SymetryPlaneData d;
                d.angles=Eigen::Vector2i(0,0);
                d.normal=symN;
                insertSymPlane(&symNormals,d);
            }
        }
        else if(fi < 90) {
            if(checkSymetryAtPlane(voxGrid, indices, N, tolerance, fi, 0, symN, diffVects[0])) {
                SymetryPlaneData d;
                d.angles=Eigen::Vector2i(fi,0);
                d.normal=symN;
                insertSymPlane(&symNormals,d);
                diffVects[0].x=0;
                diffVects[0].y=0;
                diffVects[0].z=0;
            }
            if(checkSymetryAtPlane(voxGrid, indices, N, tolerance, fi, 90, symN, diffVects[1])) {
                SymetryPlaneData d;
                d.angles=Eigen::Vector2i(fi,90);
                d.normal=symN;
                insertSymPlane(&symNormals,d);
                diffVects[1].x=0;
                diffVects[1].y=0;
                diffVects[1].z=0;
            }
            if(checkSymetryAtPlane(voxGrid, indices, N, tolerance, fi, 180, symN, diffVects[2])) {
                SymetryPlaneData d;
                d.angles=Eigen::Vector2i(fi,180);
                d.normal=symN;
                insertSymPlane(&symNormals,d);
                diffVects[2].x=0;
                diffVects[2].y=0;
                diffVects[2].z=0;
            }
            if(checkSymetryAtPlane(voxGrid, indices, N, tolerance, fi, 270, symN, diffVects[3])) {
                SymetryPlaneData d;
                d.angles=Eigen::Vector2i(fi,270);
                d.normal=symN;
                insertSymPlane(&symNormals,d);
                diffVects[3].x=0;
                diffVects[3].y=0;
                diffVects[3].z=0;
            }

            determineSymetryDC(voxGrid,indices,N,tolerance,&symNormals, fi, 0, 90, diffVects[0], diffVects[1], minAngleDiff);
            determineSymetryDC(voxGrid,indices,N,tolerance,&symNormals, fi, 90, 180, diffVects[1], diffVects[2], minAngleDiff);
            determineSymetryDC(voxGrid,indices,N,tolerance,&symNormals, fi, 180, 270, diffVects[2], diffVects[3], minAngleDiff);
            determineSymetryDC(voxGrid,indices,N,tolerance,&symNormals, fi, 270, 360, diffVects[3], diffVects[0], minAngleDiff);
        }
        else {
            if(checkSymetryAtPlane(voxGrid, indices, N, tolerance, fi, 0, symN, diffVects[0])) {
                SymetryPlaneData d;
                d.angles=Eigen::Vector2i(fi,0);
                d.normal=symN;
                insertSymPlane(&symNormals,d);
                diffVects[0].x=0;
                diffVects[0].y=0;
                diffVects[0].z=0;
            }
            if(checkSymetryAtPlane(voxGrid, indices, N, tolerance, fi, 90, symN, diffVects[1])) {
                SymetryPlaneData d;
                d.angles=Eigen::Vector2i(fi,90);
                d.normal=symN;
                insertSymPlane(&symNormals,d);
                diffVects[1].x=0;
                diffVects[1].y=0;
                diffVects[1].z=0;
            }
            if(checkSymetryAtPlane(voxGrid, indices, N, tolerance, fi, 180, symN, diffVects[2])) {
                diffVects[2].x=0;
                diffVects[2].y=0;
                diffVects[2].z=0;
            }

            determineSymetryDC(voxGrid,indices,N,tolerance,&symNormals, fi, 0, 90, diffVects[0], diffVects[1], minAngleDiff);
            determineSymetryDC(voxGrid,indices,N,tolerance,&symNormals, fi, 90, 180, diffVects[1], diffVects[2], minAngleDiff);
        }

        delete[] indices;
    }

    return symNormals;
}

std::vector<SymetryPlaneData> Symetry3D::determineSymetriesDC_SingleThread(VoxelGrid *voxGrid, double tolerance, int angleStep, int minAngleDiff) {
    if(tolerance < MIN_TOLERANCE)
        tolerance=MIN_TOLERANCE;
    std::vector<SymetryPlaneData> symNormals;
    symNormals.clear();

    int N=voxGrid->getVoxelGrid()->size();

    //#pragma omp parallel for
    for(int fi=0;fi<=90;fi+=angleStep) {

        int *indices=new int[N];
        for(int j=0;j<N;j++)
            indices[j]=j;

        Vector3D symN, diffVects[4];

        if(fi == 0) {
            if(checkSymetryAtPlane(voxGrid, indices, N, tolerance, 0, 0, symN, diffVects[0])) {
                SymetryPlaneData d;
                d.angles=Eigen::Vector2i(0,0);
                d.normal=symN;
                insertSymPlane(&symNormals,d);
            }
        }
        else if(fi < 90) {
            if(checkSymetryAtPlane(voxGrid, indices, N, tolerance, fi, 0, symN, diffVects[0])) {
                SymetryPlaneData d;
                d.angles=Eigen::Vector2i(fi,0);
                d.normal=symN;
                insertSymPlane(&symNormals,d);
                diffVects[0].x=0;
                diffVects[0].y=0;
                diffVects[0].z=0;
            }
            if(checkSymetryAtPlane(voxGrid, indices, N, tolerance, fi, 90, symN, diffVects[1])) {
                SymetryPlaneData d;
                d.angles=Eigen::Vector2i(fi,90);
                d.normal=symN;
                insertSymPlane(&symNormals,d);
                diffVects[1].x=0;
                diffVects[1].y=0;
                diffVects[1].z=0;
            }
            if(checkSymetryAtPlane(voxGrid, indices, N, tolerance, fi, 180, symN, diffVects[2])) {
                SymetryPlaneData d;
                d.angles=Eigen::Vector2i(fi,180);
                d.normal=symN;
                insertSymPlane(&symNormals,d);
                diffVects[2].x=0;
                diffVects[2].y=0;
                diffVects[2].z=0;
            }
            if(checkSymetryAtPlane(voxGrid, indices, N, tolerance, fi, 270, symN, diffVects[3])) {
                SymetryPlaneData d;
                d.angles=Eigen::Vector2i(fi,270);
                d.normal=symN;
                insertSymPlane(&symNormals,d);
                diffVects[3].x=0;
                diffVects[3].y=0;
                diffVects[3].z=0;
            }

            determineSymetryDC(voxGrid,indices,N,tolerance,&symNormals, fi, 0, 90, diffVects[0], diffVects[1], minAngleDiff);
            determineSymetryDC(voxGrid,indices,N,tolerance,&symNormals, fi, 90, 180, diffVects[1], diffVects[2], minAngleDiff);
            determineSymetryDC(voxGrid,indices,N,tolerance,&symNormals, fi, 180, 270, diffVects[2], diffVects[3], minAngleDiff);
            determineSymetryDC(voxGrid,indices,N,tolerance,&symNormals, fi, 270, 360, diffVects[3], diffVects[0], minAngleDiff);
        }
        else {
            if(checkSymetryAtPlane(voxGrid, indices, N, tolerance, fi, 0, symN, diffVects[0])) {
                SymetryPlaneData d;
                d.angles=Eigen::Vector2i(fi,0);
                d.normal=symN;
                insertSymPlane(&symNormals,d);
                diffVects[0].x=0;
                diffVects[0].y=0;
                diffVects[0].z=0;
            }
            if(checkSymetryAtPlane(voxGrid, indices, N, tolerance, fi, 90, symN, diffVects[1])) {
                SymetryPlaneData d;
                d.angles=Eigen::Vector2i(fi,90);
                d.normal=symN;
                insertSymPlane(&symNormals,d);
                diffVects[1].x=0;
                diffVects[1].y=0;
                diffVects[1].z=0;
            }
            if(checkSymetryAtPlane(voxGrid, indices, N, tolerance, fi, 180, symN, diffVects[2])) {
                diffVects[2].x=0;
                diffVects[2].y=0;
                diffVects[2].z=0;
            }

            determineSymetryDC(voxGrid,indices,N,tolerance,&symNormals, fi, 0, 90, diffVects[0], diffVects[1], minAngleDiff);
            determineSymetryDC(voxGrid,indices,N,tolerance,&symNormals, fi, 90, 180, diffVects[1], diffVects[2], minAngleDiff);
        }

        delete[] indices;
    }

    return symNormals;
}


void Symetry3D::determineSymetryDC(VoxelGrid *voxGrid, int* indices, int N, double tolerance, std::vector<SymetryPlaneData> *symNormals, int fi, int theta1, int theta2, Vector3D diffVect1, Vector3D diffVect2, int minAngleDiff) {

    float dot=diffVect1.x*diffVect2.x + diffVect1.y*diffVect2.y + diffVect1.z*diffVect2.z;
    //std::cout << fi << "," << theta1 << "," << theta2 << "," << dot  << "," << diffVect1.x() << "," << diffVect1.y() << "," << diffVect1.z() << "," << diffVect2.x() << "," << diffVect2.y() << "," << diffVect2.z() << std::endl;

    if(abs(theta2 - theta1) > minAngleDiff || (dot <= 0 && abs(theta2 - theta1) >= 2)) {
        Vector3D symN, diffVectMid;
        int thetaMid=(theta1 + theta2) / 2;
        bool res=checkSymetryAtPlane(voxGrid, indices, N, tolerance, fi, thetaMid, symN, diffVectMid);

        if((res)) {
            SymetryPlaneData d;
            d.angles=Eigen::Vector2i(fi,thetaMid);
            d.normal=symN;
            insertSymPlane(symNormals,d);
            diffVectMid.x=0;
            diffVectMid.y=0;
            diffVectMid.z=0;
        }

        //std::cout << fi << "," << theta1 << "," << theta2 << "," << dot  << "," << diffVect1.x() << "," << diffVect1.y() << "," << diffVect1.z() << "," << diffVect2.x() << "," << diffVect2.y() << "," << diffVect2.z() << "," << thetaMid << "," << res << std::endl;

        determineSymetryDC(voxGrid, indices, N, tolerance, symNormals, fi, theta1, thetaMid, diffVect1, diffVectMid, minAngleDiff);
        determineSymetryDC(voxGrid, indices, N, tolerance, symNormals, fi, thetaMid, theta2, diffVectMid, diffVect2, minAngleDiff);
    }
}

bool Symetry3D::checkSymetryAtPlane(VoxelGrid *voxGrid, int *indices, int N, double tolerance, int fi, int theta, Vector3D &symN, Vector3D &diffVect, bool debug) {
    //std::vector<Eigen::Vector3d> voxelL,voxelR;
    double dMax,dMin;
    int l, m, h;
    Vector3D splitN;
//    int i;
//    Eigen::Vector3d *centerVectors=voxGrid->getCenterVectors();

    dMax=0;
    dMin=0;

    symN.x=sinTable[theta] * sinTable[fi];
    symN.y=(cosTable[fi]);
    symN.z=(cosTable[theta] * sinTable[fi]);

//    splitN.x=(sinTable[theta] * sinTable[fi+90]);
//    splitN.y=(cosTable[fi+90]);
//    splitN.z=(cosTable[theta] * sinTable[fi+90]);

    splitN.x=cosTable[theta];
    splitN.y=0;
    splitN.z=-sinTable[theta];

    //voxGrid->prepareVoxelSplit(symN,&voxelL,&voxelR);
//    voxGrid->prepareVoxelSplit(symN,splitN,indices, l, m, h,dMin,dMax);

//    for(i=0;i<8;i++) {
//        val=splitN.x*centerVectors[i][0] + splitN.y*centerVectors[i][1] + splitN.z*centerVectors[i][2];
//        if(val > dMax) dMax=val;
//        if(val < dMin) dMin=val;
//    }

    //return checkSymetry(voxelL, voxelR, symN, splitN, dMin, dMax, tolerance, diffVect);
    if(debug) {
        voxGrid->prepareVoxelSplit(symN,splitN,indices, l, m, h,dMin,dMax);
        return checkSymetryDebug(voxGrid->getVoxelGridData(), indices, 0, l-1, h+1, N-1, symN, splitN, dMin, dMax, tolerance, diffVect);
    }
    else {
//        return checkSymetry(voxGrid->getVoxelGridData(), indices, 0, l-1, h+1, N-1, symN, splitN, dMin, dMax, tolerance, diffVect);
        calcDMinDMax(voxGrid->getVoxelGridData(), N, splitN, dMin, dMax);
        return checkSymetryNonRecursive(voxGrid->getVoxelGridData(), N, symN, splitN, dMin, dMax, tolerance, diffVect);
    }
}

void Symetry3D::prepareDebugData(VoxelGrid *voxGrid, double tolerance,int fi, int theta) {
    debugData.clear();

    int N=voxGrid->getVoxelGrid()->size();
    int *indices=new int[N];
    for(int j=0;j<N;j++)
        indices[j]=j;

    Vector3D symN, diffVect;

    checkSymetryAtPlane(voxGrid, indices, N, tolerance, fi, theta, symN, diffVect, true);
}

void Symetry3D::prepareDebugDataNR(VoxelGrid *voxGrid, double tolerance,int fi, int theta) {
    debugData.clear();

    double dMax,dMin;
    Vector3D splitN,symN,diffVect;

    int N=voxGrid->getVoxelGrid()->size();

    dMax=0;
    dMin=0;

    symN.x=sinTable[theta] * sinTable[fi];
    symN.y=(cosTable[fi]);
    symN.z=(cosTable[theta] * sinTable[fi]);

//    splitN.x=(sinTable[theta] * sinTable[fi+90]);
//    splitN.y=(cosTable[fi+90]);
//    splitN.z=(cosTable[theta] * sinTable[fi+90]);

    splitN.x=cosTable[theta];
    splitN.y=0;
    splitN.z=-sinTable[theta];
    calcDMinDMax(voxGrid->getVoxelGridData(), N, splitN, dMin, dMax);

    checkSymetryNonRecursiveDebugData(voxGrid->getVoxelGridData(), N, symN, splitN, dMin, dMax, tolerance, diffVect);

    if(debugData.size() > 1)
        debugData.pop_back();
}

std::vector<SymetryPlaneData> Symetry3D::determineSymetries(VoxelGrid *voxGrid, double tolerance, int angleStep) {
    if(tolerance < MIN_TOLERANCE)
        tolerance=MIN_TOLERANCE;

    std::vector<SymetryPlaneData> symNormals;
    symNormals.clear();

    int N=voxGrid->getVoxelGrid()->size();

    for(int fi=0;fi<=90;fi+=angleStep) {

        int maxAng=360;

        if(fi < 1) maxAng=1;
        else if(fi < 90) maxAng=360;
        else maxAng=180;
        #pragma omp parallel for
        for(int theta=0;theta<maxAng;theta+=angleStep) {
            int *indices=new int[N];
            for(int j=0;j<N;j++)
                indices[j]=j;

            Vector3D symN, diffVect;
            if(checkSymetryAtPlane(voxGrid, indices, N, tolerance, fi, theta, symN, diffVect)) {
                SymetryPlaneData d;
                d.angles=Eigen::Vector2i(fi,theta);
                d.normal=symN;
                insertSymPlane(&symNormals,d);
            }

            delete[] indices;
        }

    }
    return symNormals;
}

std::vector<SymetryPlaneData> Symetry3D::determineSymetries_SFor(VoxelGrid *voxGrid, double tolerance, int angleStep) {
    if(tolerance < MIN_TOLERANCE)
        tolerance=MIN_TOLERANCE;

    std::vector<SymetryPlaneData> symNormals;
    symNormals.clear();

    int N=voxGrid->getVoxelGrid()->size();

    #pragma omp parallel for
    for(int index=359;index<32580;index++) {
        int fi=index/360,theta=index%360;

        int *indices=new int[N];
        for(int j=0;j<N;j++)
            indices[j]=j;

        Vector3D symN, diffVect;
        if(checkSymetryAtPlane(voxGrid, indices, N, tolerance, fi, theta, symN, diffVect)) {
            SymetryPlaneData d;
            d.angles=Eigen::Vector2i(fi,theta);
            d.normal=symN;
            insertSymPlane(&symNormals,d);
        }

        delete[] indices;

    }
    return symNormals;
}

std::vector<SymetryPlaneData> Symetry3D::determineSymetries_SFor_SingleThread(VoxelGrid *voxGrid, double tolerance, int angleStep) {
    if(tolerance < MIN_TOLERANCE)
        tolerance=MIN_TOLERANCE;

    std::vector<SymetryPlaneData> symNormals;
    symNormals.clear();

    int N=voxGrid->getVoxelGrid()->size();

    //#pragma omp parallel for
    for(int index=359;index<32580;index++) {
        int fi=index/360,theta=index%360;

        int *indices=new int[N];
        for(int j=0;j<N;j++)
            indices[j]=j;

        Vector3D symN, diffVect;
        if(checkSymetryAtPlane(voxGrid, indices, N, tolerance, fi, theta, symN, diffVect)) {
            SymetryPlaneData d;
            d.angles=Eigen::Vector2i(fi,theta);
            d.normal=symN;
            insertSymPlane(&symNormals,d);
        }

        delete[] indices;

    }
    return symNormals;
}

float Symetry3D::estimateSymetry(VoxelGrid *voxGrid,int *indices, int N, Vector3D &symN, int fi, int theta) {
    int l, m, h;

    Vector3D splitN;
    double dMin, dMax;

    symN.x=(sinTable[theta] * sinTable[fi]);
    symN.y=(cosTable[fi]);
    symN.z=(cosTable[theta] * sinTable[fi]);

    splitN.x=(sinTable[theta] * sinTable[fi+90]);
    splitN.y=(cosTable[fi+90]);
    splitN.z=(cosTable[theta] * sinTable[fi+90]);

    bool splitL,splitD;

    voxGrid->prepareVoxelSplit(symN, splitN,indices, l, m, h, dMin, dMax);

    Vector3D lCOM=calcVoxCOM(voxGrid->getVoxelGridData(), indices, 0, l-1,splitL),rCOM=calcVoxCOM(voxGrid->getVoxelGridData(), indices, h+1, N-1,splitD), lRefl;

    lRefl.x=(1-2*symN.x*symN.x)*lCOM.x    -2*symN.x*symN.y  *lCOM.y     -2*symN.x*symN.z *lCOM.z;
    lRefl.y=  -2*symN.x*symN.y *lCOM.x + (1-2*symN.y*symN.y)*lCOM.y     -2*symN.y*symN.z *lCOM.z;
    lRefl.z=  -2*symN.x*symN.z *lCOM.x     -2*symN.z*symN.y *lCOM.y + (1-2*symN.z*symN.z)*lCOM.z;

    return lRefl.distanceToPoint(rCOM);
}

float Symetry3D::estimateSymetryMesh(Mesh3D *mesh, Vector3D &symN, int fi, int theta) {
    symN.x=(sinTable[theta] * sinTable[fi]);
    symN.y=(cosTable[fi]);
    symN.z=(cosTable[theta] * sinTable[fi]);

    Vector3D lCOM,rCOM,lRefl;

    calcMeshCOMLR(mesh,symN,lCOM,rCOM,true);
    lRefl.x=(1-2*symN.x*symN.x)*lCOM.x    -2*symN.x*symN.y  *lCOM.y     -2*symN.x*symN.z *lCOM.z;
    lRefl.y=  -2*symN.x*symN.y *lCOM.x + (1-2*symN.y*symN.y)*lCOM.y     -2*symN.y*symN.z *lCOM.z;
    lRefl.z=  -2*symN.x*symN.z *lCOM.x     -2*symN.z*symN.y *lCOM.y + (1-2*symN.z*symN.z)*lCOM.z;

    return lRefl.distanceToPoint(rCOM);
}

void Symetry3D::calcMeshCOMLR(Mesh3D *mesh, Vector3D symN, Vector3D &lCOM, Vector3D &rCOM, bool convHull) {
    open3d::geometry::TriangleMesh *m;
    if(convHull)
        m=mesh->getConvHullMesh();
    else
        m=mesh->getMesh();
    Eigen::Vector3d p;
    int i, lN=0, rN=0;
    double dot;
    for(i=0;i<m->vertices_.size();i++) {
        p=m->vertices_[i];
        dot=p[0] * symN.x + p[1] * symN.y + p[2] * symN.z;

        if(fabs(dot) < 1E-3) {
//            continue;
            lCOM.x+=p[0];
            lCOM.y+=p[1];
            lCOM.z+=p[2];
            lN++;

            rCOM.x+=p[0];
            rCOM.y+=p[1];
            rCOM.z+=p[2];
            rN++;
        }
        else if(dot > 0) {
            lCOM.x+=p[0];
            lCOM.y+=p[1];
            lCOM.z+=p[2];
            lN++;
        }
        else {
            rCOM.x+=p[0];
            rCOM.y+=p[1];
            rCOM.z+=p[2];
            rN++;
        }
    }

    if(lN > 0) {
        lCOM.x /= (double)lN;
        lCOM.y /= (double)lN;
        lCOM.z /= (double)lN;
    }
    if(rN > 0) {
        rCOM.x /= (double)rN;
        rCOM.y /= (double)rN;
        rCOM.z /= (double)rN;
    }
}

SymetryPlaneData Symetry3D::estimateBestSymetriesMesh(Mesh3D *mesh, int angleStep) {
    SymetryPlaneData symNormal;
    symNormal.symetry=100000;

    #pragma omp parallel for
    for(int val=359;val<32580;val++) {
        int fi=val/360,theta=val%360;
        Vector3D symN;
        float symetry=estimateSymetryMesh(mesh, symN, fi, theta);

        omp_set_lock(&mutex);
        if(symetry < symNormal.symetry) {
            symNormal.symetry=symetry;
            symNormal.normal=symN;
            symNormal.angles=Eigen::Vector2i(fi,theta);
        }
        omp_unset_lock(&mutex);
    }
    return symNormal;
}

std::vector<SymetryPlaneData> Symetry3D::calcAllSymetriesMesh(Mesh3D *mesh, int angleStep) {

    std::vector<SymetryPlaneData> symNormals;
    symNormals.clear();

    #pragma omp parallel for
    for(int val=0;val<32760;val++) {
        int fi=val/360,theta=val%360;
        Vector3D symN;
        float symetry=estimateSymetryMesh(mesh, symN, fi, theta);

        omp_set_lock(&mutex);
        SymetryPlaneData d;
        d.symetry=symetry;
        d.angles=Eigen::Vector2i(fi,theta);
        d.normal=symN;
        symNormals.push_back(d);
        omp_unset_lock(&mutex);
    }
    return symNormals;
}

SymetryPlaneData Symetry3D::estimateBestSymetries(VoxelGrid *voxGrid, int angleStep) {

    SymetryPlaneData symNormal;
    symNormal.symetry=100000;

    int N=voxGrid->getVoxelGrid()->size();

    for(int fi=0;fi<=90;fi+=angleStep) {

        int maxAng=360;

        if(fi < 1) maxAng=1;
        else if(fi < 90) maxAng=360;
        else maxAng=180;
        #pragma omp parallel for
        for(int theta=0;theta<maxAng;theta+=angleStep) {
            int *indices=new int[N];
            for(int j=0;j<N;j++)
                indices[j]=j;

            Vector3D symN;
            float symetry=estimateSymetry(voxGrid, indices, N, symN, fi, theta);

            omp_set_lock(&mutex);
            if(symetry < symNormal.symetry) {
                symNormal.symetry=symetry;
                symNormal.normal=symN;
                symNormal.angles=Eigen::Vector2i(fi,theta);
            }
            omp_unset_lock(&mutex);

            delete[] indices;
        }

    }
    return symNormal;
}

SymetryPlaneData Symetry3D::estimateBestSymetries_SFor(VoxelGrid *voxGrid, int angleStep) {

    SymetryPlaneData symNormal;
    symNormal.symetry=100000;

    int N=voxGrid->getVoxelGrid()->size();

    #pragma omp parallel for
    for(int index=359;index<32580;index++) {
        int fi=index/360,theta=index%360;

        int *indices=new int[N];
        for(int j=0;j<N;j++)
            indices[j]=j;

        Vector3D symN;
        float symetry=estimateSymetry(voxGrid, indices, N, symN, fi, theta);

        omp_set_lock(&mutex);
        if(symetry < symNormal.symetry) {
            symNormal.symetry=symetry;
            symNormal.normal=symN;
            symNormal.angles=Eigen::Vector2i(fi,theta);
        }
        omp_unset_lock(&mutex);

        delete[] indices;
    }
    return symNormal;
}

SymetryPlaneData Symetry3D::estimateBestSymetries2Stage(VoxelGrid *coarseVoxGrid,VoxelGrid *voxGrid, int angleStep) {

    SymetryPlaneData symNormal;
    symNormal.symetry=100000;

    double maxSym=0,bestRelativeSym=1000000;

    int N=coarseVoxGrid->getVoxelGrid()->size();

    int numThrs=24;

    int *indices=new int[numThrs * N];

    #pragma omp parallel for
    for(int val=359;val < 32580;val++) {
        int fi=val / 360,theta=val % 360;
        int index=omp_get_thread_num();

        for(int j=0;j<N;j++)
            indices[index * N + j]=j;

        Vector3D symN;
        float symetry=estimateSymetry(coarseVoxGrid, &indices[index * N], N, symN, fi, theta);

        omp_set_lock(&mutex);

        symetryEstimate[fi][theta]=symetry;
        if(symetry > maxSym)
            maxSym=symetry;

        omp_unset_lock(&mutex);
    }

    delete[] indices;

    #pragma omp parallel for
    for(int val=359;val < 32580;val++) {
        int fi=val / 360,theta=val % 360;

        symetryEstimate[fi][theta] /= maxSym;
        omp_set_lock(&mutex);
        if(symetryEstimate[fi][theta] < bestRelativeSym)
            bestRelativeSym=symetryEstimate[fi][theta];
        omp_unset_lock(&mutex);
    }

    N=voxGrid->getVoxelGrid()->size();
    indices=new int[numThrs * N];

    #pragma omp parallel for
    for(int val=359;val < 32580;val++) {
        int fi=val / 360,theta=val % 360;
        if(symetryEstimate[fi][theta] - bestRelativeSym < 0.1) {
            int index=omp_get_thread_num();
            for(int j=0;j<N;j++)
                indices[index * N + j]=j;

            Vector3D symN;
            float symetry=estimateSymetry(voxGrid, &indices[index * N], N, symN, fi, theta);

            omp_set_lock(&mutex);
            if(symetry < symNormal.symetry) {
                symNormal.symetry=symetry;
                symNormal.normal=symN;
                symNormal.angles=Eigen::Vector2i(fi,theta);
            }
            omp_unset_lock(&mutex);
        }
    }

    delete[] indices;
    return symNormal;
}

std::vector<SymetryPlaneData> Symetry3D::prepare1stEstimateFunction(VoxelGrid *voxGrid, int angleStep) {

    std::vector<SymetryPlaneData> symNormals;
    symNormals.clear();

    int N=voxGrid->getVoxelGrid()->size();
    #pragma omp parallel for
    for(int val=0;val<32760;val++) {
        int fi=val/360,theta=val%360;

        int *indices=new int[N];
        for(int j=0;j<N;j++)
            indices[j]=j;

        Vector3D symN;
        float symetry=estimateSymetry(voxGrid, indices, N, symN, fi, theta);

        omp_set_lock(&mutex);
        SymetryPlaneData d;
        d.symetry=symetry;
        d.angles=Eigen::Vector2i(fi,theta);
        d.normal=symN;
        symNormals.push_back(d);
        omp_unset_lock(&mutex);

        delete[] indices;
    }
    return symNormals;
}

std::vector<SymetryPlaneData> Symetry3D::estimateSymetriesNClass(VoxelGrid *voxGrid, int angleStep, double interval, double &minSym, double &maxSym) {
    std::vector<SymetryPlaneData> symNormals;
    symNormals.clear();

    minSym=100000;
    maxSym=-100000;

    int N=voxGrid->getVoxelGrid()->size();

    #pragma omp parallel for
    for(int val=0;val<32760;val++) {
        int fi=val/360,theta=val%360;
        Vector3D symN;
        float symetry=evaluateSymetryNClass(voxGrid, N, symN, fi, theta, interval);

        omp_set_lock(&mutex);
        SymetryPlaneData d;
        d.symetry=symetry;
        d.angles=Eigen::Vector2i(fi,theta);
        d.normal=symN;
        symNormals.push_back(d);

        if(minSym > symetry)
            minSym=symetry;
        if(maxSym < symetry)
            maxSym=symetry;

        omp_unset_lock(&mutex);
    }
    return symNormals;
}

void Symetry3D::evaluateSymetriesNClass(VoxelGrid *voxGrid, double array[92][362], int angleStep, double interval, double &minSym, double &maxSym) {
    minSym=100000;
    maxSym=-100000;

    int N=voxGrid->getVoxelGrid()->size();

    #pragma omp parallel for
    for(int val=359;val<32580;val++) {
        int fi=val/360,theta=val%360;

        Vector3D symN;
        float symetry=evaluateSymetryNClass(voxGrid, N, symN, fi, theta, interval);

        omp_set_lock(&mutex);
        array[fi][theta]=symetry;

        if(minSym > symetry)
            minSym=symetry;
        if(maxSym < symetry)
            maxSym=symetry;

        omp_unset_lock(&mutex);
    }
}

SymetryPlaneData Symetry3D::estimateBestSymetryNClass(VoxelGrid *voxGrid, double array[92][362], int angleStep, double interval, double minSym, double maxSym) {
    SymetryPlaneData symNormal;
    symNormal.symetry=100000;

    double bestRelSym=minSym / maxSym;
    int N=voxGrid->getVoxelGrid()->size();

    int counter=0;

    #pragma omp parallel for
    for(int val=359;val<32580;val++) {
        int fi=val/360,theta=val%360;
        double relSym=array[fi][theta] / maxSym;
        if(relSym - bestRelSym < 0.05) {
            Vector3D symN;
            float symetry=evaluateSymetryNClass(voxGrid, N, symN, fi, theta, interval);

            omp_set_lock(&mutex);
            counter++;
            if(symetry < symNormal.symetry) {
                symNormal.symetry=symetry;
                symNormal.normal=symN;
                symNormal.angles=Eigen::Vector2i(fi,theta);
            }
            omp_unset_lock(&mutex);
        }
    }

    std::cout << "Second check: " << counter << std::endl;
    return symNormal;
}

float Symetry3D::evaluateSymetryNClass(VoxelGrid *voxGrid, int N, Vector3D &symN, int fi, int theta, double interval) {
    int M,i,index;

    Vector3D splitN;
    double dMin, dMax, d, sumOcena;

    symN.x=(sinTable[theta] * sinTable[fi]);
    symN.y=(cosTable[fi]);
    symN.z=(cosTable[theta] * sinTable[fi]);

//    splitN.x=(sinTable[theta] * sinTable[fi+90]);
//    splitN.y=(cosTable[fi+90]);
//    splitN.z=(cosTable[theta] * sinTable[fi+90]);

    splitN.x=cosTable[theta];
    splitN.y=0;
    splitN.z=-sinTable[theta];
    dMax=-10000;
    dMin=10000;

    double val;

    Eigen::Vector3d *centerVectors=voxGrid->getCenterVectors();

    for(i=0;i<8;i++) {
        val=splitN.x*centerVectors[i][0] + splitN.y*centerVectors[i][1] + splitN.z*centerVectors[i][2];
        if(val > dMax) dMax=val;
        if(val < dMin) dMin=val;
    }

    M=(int)((dMax - dMin) / interval) + 1;

    SymClass *classesL=new SymClass[M],*classesR=new SymClass[M];

    for(i=0;i<M;i++) {
        classesL[i].counter=0;
        classesR[i].counter=0;
    }

    Eigen::Vector3d *voxGrd=voxGrid->getVoxelGridData();
    Vector3D v,lRefl;

    double r=VOX_RADIUS*fabs(symN.x) + VOX_RADIUS*fabs(symN.y) + VOX_RADIUS*fabs(symN.z),s;

    for(i=0;i<N;i++) {
        v.x=voxGrd[i][0] + 0.5;
        v.y=voxGrd[i][1] + 0.5;
        v.z=voxGrd[i][2] + 0.5;

        s=symN.x * v.x + symN.y * v.y + symN.z * v.z;

        if(fabs(s) <= r) {
            d=splitN.x * v.x + splitN.y * v.y + splitN.z * v.z;

            if(d>dMax)
                d=dMax;
            if(d<dMin)
                d=dMin;

            index=(int)((d-dMin) / interval);

            if(index < 0 || index >= M)
                std::cout << "Napacen index: " << index << std::endl;

            classesL[index].COG.x += v.x;
            classesL[index].COG.y += v.y;
            classesL[index].COG.z += v.z;
            classesL[index].counter++;

            classesR[index].COG.x += v.x;
            classesR[index].COG.y += v.y;
            classesR[index].COG.z += v.z;
            classesR[index].counter++;
            //continue;
        }
        //left
        else if(s < 0) {
            d=splitN.x * v.x + splitN.y * v.y + splitN.z * v.z;

            if(d>dMax)
                d=dMax;
            if(d<dMin)
                d=dMin;

            index=(int)((d-dMin) / interval);

            if(index < 0 || index >= M)
                std::cout << "Napacen index: " << index << std::endl;

            classesL[index].COG.x += v.x;
            classesL[index].COG.y += v.y;
            classesL[index].COG.z += v.z;
            classesL[index].counter++;
        }
        //right
        else {
            d=splitN.x * v.x + splitN.y * v.y + splitN.z * v.z;

            if(d>dMax)
                d=dMax;
            if(d<dMin)
                d=dMin;

            index=(int)((d-dMin) / interval);

            if(index < 0 || index >= M)
                std::cout << "Napacen index: " << index << std::endl;

            classesR[index].COG.x += v.x;
            classesR[index].COG.y += v.y;
            classesR[index].COG.z += v.z;
            classesR[index].counter++;
        }
    }

    for(i=0;i<M;i++) {
        if(classesL[i].counter > 0) {
            classesL[i].COG.x /= (double)classesL[i].counter;
            classesL[i].COG.y /= (double)classesL[i].counter;
            classesL[i].COG.z /= (double)classesL[i].counter;
        }
        if(classesR[i].counter > 0) {
            classesR[i].COG.x /= (double)classesR[i].counter;
            classesR[i].COG.y /= (double)classesR[i].counter;
            classesR[i].COG.z /= (double)classesR[i].counter;
        }
    }

    sumOcena=0;
    int myCounter=0;
    for(i=0;i<M;i++) {
        if(classesL[i].counter != 0 && classesR[i].counter != 0) {
            lRefl.x=(1-2*symN.x*symN.x)*classesL[i].COG.x    -2*symN.x*symN.y  *classesL[i].COG.y     -2*symN.x*symN.z *classesL[i].COG.z;
            lRefl.y=  -2*symN.x*symN.y *classesL[i].COG.x + (1-2*symN.y*symN.y)*classesL[i].COG.y     -2*symN.y*symN.z *classesL[i].COG.z;
            lRefl.z=  -2*symN.x*symN.z *classesL[i].COG.x     -2*symN.z*symN.y *classesL[i].COG.y + (1-2*symN.z*symN.z)*classesL[i].COG.z;
            sumOcena += lRefl.distanceToPoint(classesR[i].COG);
            myCounter++;
        }
        else if(classesL[i].counter != 0) {
            sumOcena += fabs(symN.x * classesL[i].COG.x + symN.y * classesL[i].COG.y + symN.z * classesL[i].COG.z);
            myCounter++;
        }
        else if(classesR[i].counter != 0) {
            sumOcena += fabs(symN.x * classesR[i].COG.x + symN.y * classesR[i].COG.y + symN.z * classesR[i].COG.z);
            myCounter++;
        }
    }

    delete[] classesL;
    delete[] classesR;

    return (sumOcena / (double)myCounter);
}

void Symetry3D::calculateGaussFunction(double val, double c, double &res, double &deriv) {
    res=exp(-(val*val)/(2 * c * c));
    deriv=-val * 1.0 / (c*c) * res;
}

void Symetry3D::roundRobin2D(int i, int j, int M, int N, int &ii, int &jj) {
    ii=i;
    jj=j;
    if(ii < 0)
        ii+=M;
    if(jj < 0)
        jj+=N;

    ii=ii % M;
    jj=jj % N;
}

void Symetry3D::performLocalSearch(VoxelGrid *voxGrid, int startPhi, int startTheta, double symsEst[91][360], double &bestSym) {
    int N=voxGrid->getVoxelGrid()->size();

    int fi,theta;

    //bool top=false;
    fi=startPhi;
    theta=startTheta;

    if(symsEst[fi][theta] < 0) {
        Vector3D symN;
        symsEst[fi][theta]=evaluateSymetryNClass(voxGrid, N, symN, fi, theta, SPLIT3D_LIMIT);
        if(bestSym > symsEst[fi][theta])
            bestSym=symsEst[fi][theta];
    }
/*
    double res, deriv;
    calculateGaussFunction(symsEst[fi][theta], SYM3D_FINE_VARIANCE, res, deriv);
    if(fabs(deriv) < 1E-3 && res < 0.5)
        return;

    while(!top) {
        #pragma omp parallel for
        for(int i=fi-1;i<=fi+1;i++)
            for(int j=theta-1;j<=theta+1;j++) {
                int ii,jj;
                Vector3D symN;
                roundRobin2D(i,j,91,360,ii,jj);
                if(symsEst[ii][jj] < 0) {
                    symsEst[ii][jj]=evaluateSymetryNClass(voxGrid, N, symN, ii, jj, SPLIT3D_LIMIT);
                    omp_set_lock(&mutex);
                    if(bestSym > symsEst[ii][jj])
                        bestSym=symsEst[ii][jj];
                    omp_unset_lock(&mutex);
                }
            }

        double maxSym=symsEst[fi][theta];
        int maxI=0,maxJ=0;
        for(int i=fi-1;i<=fi+1;i++)
            for(int j=theta-1;j<=theta+1;j++) {
                int ii,jj;
                roundRobin2D(i,j,91,360,ii,jj);
                if(symsEst[ii][jj] < maxSym) {
                    maxSym=symsEst[ii][jj];
                    maxI=i-fi;
                    maxJ=j-theta;
                }
            }

        if(maxI == 0 && maxJ == 0) {
            top=true;
            return;
        }

        roundRobin2D(fi+maxI,theta+maxJ,91,360,fi,theta);
    }
    */
}

std::vector<SymetryPlaneData> Symetry3D::progressiveDetermineSymetryPlanes(Mesh3D *mesh, VoxelGrid *coarseVoxGrid, VoxelGrid *voxGrid, int angleStep, double voxSize, double tolerance) {
    std::vector<SymetryPlaneData> symNormals;
    symNormals.clear();

    const int N_PHI=((int)(91 / PROG_SYM3D_PHI_DIV)) + 1;
    const int N_THETA=((int)(360 / PROG_SYM3D_THETA_DIV)) + 1;
    SymetryPlaneData candidates[N_PHI][N_THETA];
    int i,j;
    for(i=0;i<N_PHI;i++)
        for(j=0;j<N_THETA;j++)
            candidates[i][j].symetry=-1;

    #pragma omp parallel for
    for(int val=359;val<32580;val++) {
        int fi=val / 360,theta=val % 360;

        Vector3D symN;
        float symetry=estimateSymetryMesh(mesh, symN, fi, theta);

        int p=((int)(fi / PROG_SYM3D_PHI_DIV)),t=((int)(theta / PROG_SYM3D_THETA_DIV));

        omp_set_lock(&mutex);
        if(candidates[p][t].symetry < 0) {
            candidates[p][t].symetry=symetry;
            candidates[p][t].angles=Eigen::Vector2i(fi,theta);
            candidates[p][t].normal=symN;
        }
        else if(candidates[p][t].symetry > symetry) {
            candidates[p][t].symetry=symetry;
            candidates[p][t].angles=Eigen::Vector2i(fi,theta);
            candidates[p][t].normal=symN;
        }
        omp_unset_lock(&mutex);
    }

    int N=coarseVoxGrid->getVoxelGrid()->size();

    SymetryPlaneData bestSymCandidate;
    bestSymCandidate.symetry=100000;
    int counter=0;

    #pragma omp parallel for
    for(int val=0;val<N_PHI*N_THETA;val++) {
        int fi=val / N_THETA,theta=val % N_THETA;
        if(candidates[fi][theta].symetry >= 0) {
            double res, deriv;

            calculateGaussFunction(candidates[fi][theta].symetry, 1.0, res, deriv);

            if(fabs(deriv) < 1E-2 && res < 0.5) {
                candidates[fi][theta].symetry=-1;
                continue;
            }

            Vector3D symN;
            float symetry=evaluateSymetryNClass(coarseVoxGrid, N, symN, candidates[fi][theta].angles[0], candidates[fi][theta].angles[1], SPLIT3D_LIMIT);

            candidates[fi][theta].symetry=symetry;

            calculateGaussFunction(symetry, SYM3D_COARSE_VARIANCE, res, deriv);

            omp_set_lock(&mutex);

            if(bestSymCandidate.symetry > symetry) {
                bestSymCandidate.symetry=symetry;
                bestSymCandidate.angles=candidates[fi][theta].angles;
                bestSymCandidate.normal=symN;
            }

            if(fabs(deriv) < 1E-2 && res < 0.5)
                candidates[fi][theta].symetry = -1;
            else
                counter++;

            omp_unset_lock(&mutex);
        }
    }

    std::cout << counter << std::endl;

    double sym[91][360],bestSym=100000;
    #pragma omp parallel for
    for(int val=0;val < 32580;val++) {
        int fi=val/360, theta=val%360;
        sym[fi][theta]=-1;
    }

    if(counter == 0) {
        performLocalSearch(voxGrid,bestSymCandidate.angles[0],bestSymCandidate.angles[1],sym,bestSym);
    }
    else {
        for(int fi=0;fi<N_PHI;fi++) {
            for(int theta=0;theta<N_THETA;theta++) {
                if(candidates[fi][theta].symetry >= 0) {
                    performLocalSearch(voxGrid, candidates[fi][theta].angles[0], candidates[fi][theta].angles[1],sym,bestSym);
                }
            }
        }
    }

    #pragma omp parallel for
    for(int val=0;val < 32580;val++) {
        int fi=val/360, theta=val%360;
        if(sym[fi][theta] >= 0 && sym[fi][theta] - bestSym <= tolerance) {
            SymetryPlaneData data;
            Vector3D symN;
            symN.x=(sinTable[theta] * sinTable[fi]);
            symN.y=(cosTable[fi]);
            symN.z=(cosTable[theta] * sinTable[fi]);
            data.symetry=sym[fi][theta];
            data.angles=Eigen::Vector2i(fi,theta);
            data.normal=symN;

            omp_set_lock(&mutex);
            symNormals.push_back(data);
            omp_unset_lock(&mutex);
        }
    }

    return symNormals;
}
