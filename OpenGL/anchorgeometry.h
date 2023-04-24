#ifndef ANCHORGEOMETRY_H
#define ANCHORGEOMETRY_H

#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLBuffer>
#include <QOpenGLTexture>
#include <QOpenGLVertexArrayObject>
#include <QMatrix4x4>
#include "OpenGL/oglobject.h"

class AnchorGeometry : public OGLObject
{
public:
    AnchorGeometry();
    ~AnchorGeometry();

    void draw(QMatrix4x4 projMat, QMatrix4x4 modelViewMat, float scaleFactor);

private:
    QOpenGLVertexArrayObject vaoAnchor, vaoQuad;
    QOpenGLBuffer anchorArrayBuf,quadArrayBuf;
    QOpenGLBuffer anchorIndexBuf,quadIndexBuf;
    QOpenGLTexture *text[3];

    QOpenGLShaderProgram *bilbShader, *colorShaderProgram;

    int colorUniformProjection, colorUniformModelView;
    int colorShaderVertexLocation,colorShaderColorLocation;

    int bilbUniformProjection, bilbUniformModelView;
    int bilbUniformScaleFactor;
    int bilbInPosAttribute,bilbTexCoordAttribute;
};

#endif // ANCHORGEOMETRY_H
