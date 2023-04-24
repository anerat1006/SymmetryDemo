#ifndef GLDISPLAYLIDAR_H
#define GLDISPLAYLIDAR_H

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLBuffer>
#include <QOpenGLVertexArrayObject>
#include <QMatrix4x4>
#include <QVector3D>
#include <QVector>
#include "anchorgeometry.h"

#include "abstractopenglwidget.h"
#include "3D/mesh3d.h"

class GLDisplayMesh : public AbstractOpenGLWidget
{
    Q_OBJECT

public:
    GLDisplayMesh(QWidget *parent=nullptr);

    ~GLDisplayMesh();

    void prepareObject(Mesh3D *mesh, bool convHull=false);

    void writeObject(Mesh3D *mesh);

    void setLightPosition(const QVector3D &newLightPosition);

    void setDiffuseLight(const QVector3D &newDiffuseLight);

    void setPolyFill(bool newPolyFill);

public slots:

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;

    QMatrix4x4 m_proj,worldMatrix,viewMatrix;

    QOpenGLShaderProgram /**colorShaderProgram, */*lightShaderProgram;
//    int colorShaderVertexLocation,colorShaderColorLocation;
    QOpenGLBuffer *vbo,*ibo;

    int numIndices;

//    int colorUniformProjection, colorUniformModelView;

    bool polyFill;

    int numLidarPoints;

    AnchorGeometry *anchorGeometry;

    int uniformAmbient, uniformDifuse, uniformSpecular, uniformShininess;
    int lightUniformProjection, lightUniformModelView, lightUniformNormalMat;
    int uniformLightDir;
    int lightShaderVertexLocation,lightShaderColorLocation,lightShaderNormalLocation;

    QVector3D ambientLight, diffuseLight, specularLight, lightDirection;
    GLfloat shininess;
};

#endif // GLDISPLAYLIDAR_H
