#ifndef GLDISPLAYVOXELS_H
#define GLDISPLAYVOXELS_H

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QMatrix4x4>
#include <QVector3D>
#include <QWheelEvent>
#include <QMouseEvent>
//#include "cubegeometry.h"
#include "anchorgeometry.h"
#include "abstractopenglwidget.h"
#include "3D/mesh3d.h"
#include "3D/voxelgrid.h"


class GLDisplayVoxels : public AbstractOpenGLWidget
{
    Q_OBJECT
public:
    GLDisplayVoxels(QWidget *parent=nullptr);
    ~GLDisplayVoxels();

    void setLightPosX(float val);
    void setLightPosY(float val);
    void setLightPosZ(float val);

    void setVoxGrid(VoxelGrid *newGrid);

    void setDiffuseLight(const QVector3D &newDiffuseLight);

    virtual void updateVoxelGrid();

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;

protected:
    QMatrix4x4 projMatrix, worldMatrix, viewMatrix;

    QVector3D ambientLight, diffuseLight, specularLight, lightDirection;
    GLfloat shininess;

    AnchorGeometry *anchorGeometry;

    float unitDim;

//    Mesh3D *mesh;
    VoxelGrid *voxGrid;


    QOpenGLShaderProgram *lightShaderProgram;

    int uniformAmbient, uniformDifuse, uniformSpecular, uniformShininess;
    int lightUniformProjection, lightUniformModel, lightUniformView, lightUniformNormalMat;
    int uniformLightDir;
    int lightShaderVertexLocation,lightShaderColorLocation,lightShaderNormalLocation,lightShaderOffsetLocation;

    QOpenGLVertexArrayObject *vao;
    QOpenGLBuffer *arrayBuf;
    QOpenGLBuffer *indexBufFull;
    QOpenGLBuffer *offsetBuf;

    bool dirtyBit;

    int numInstances;
};

#endif // GLDISPLAYVOXELS_H
