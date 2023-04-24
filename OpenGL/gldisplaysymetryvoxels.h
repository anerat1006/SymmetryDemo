#ifndef GLDISPLAYSYMETRYVOXELS_H
#define GLDISPLAYSYMETRYVOXELS_H

#include "gldisplayvoxels.h"
#include <vector>

class GLDisplaySymetryVoxels : public GLDisplayVoxels
{
public:
    GLDisplaySymetryVoxels(QWidget *parent=nullptr);

    ~GLDisplaySymetryVoxels();

    void setVoxelsL(const std::vector<Eigen::Vector3d> &newVoxelsL);

    void setVoxelsR(const std::vector<Eigen::Vector3d> &newVoxelsR);

    void setVoxelsSP(const std::vector<Eigen::Vector3d> &newVoxelsSP);

    void clean();

protected:
    void paintGL() override;
    void initializeGL() override;

    QOpenGLBuffer *offsetBufL,*offsetBufR;
    int numInstancesL, numInstancesR;

    std::vector<Eigen::Vector3d> VoxelsSP,VoxelsL,VoxelsR;
};

#endif // GLDISPLAYSYMETRYVOXELS_H
