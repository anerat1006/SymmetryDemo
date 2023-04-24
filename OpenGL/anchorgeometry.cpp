#include "anchorgeometry.h"
#include <QFile>
#include <QTextStream>

AnchorGeometry::AnchorGeometry() :
    anchorIndexBuf(QOpenGLBuffer::IndexBuffer),
    quadIndexBuf(QOpenGLBuffer::IndexBuffer)
{
    GLushort indicesLine[]={0,1,0,2,0,3};
    GLfloat anchor_points[] = {
        0.0f, 0.0f, 0.0f,
        1.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 1.0f
    };

    GLushort indicesQuad[]={0,1,2,3};
    GLfloat quad_points[] = {
        -1.0f, 1.0f, 0.0f, 0.0f, 0.0f,
        1.0f, 1.0f, 0.0f, 0.9f, 0.0f,
        1.0f, -1.0f, 0.0f, 0.9f, 0.9f,
        -1.0f, -1.0f, 0.0f, 0.0f, 0.9f
    };

    initializeOpenGLFunctions();

    bilbShader=new QOpenGLShaderProgram;
    BuildShaders(":/glsl/Billboard",bilbShader);

    colorShaderProgram=new QOpenGLShaderProgram;
    BuildShaders(":/glsl/ColorVis",colorShaderProgram);

    colorUniformProjection=colorShaderProgram->uniformLocation("ProjectionMat");
    colorUniformModelView=colorShaderProgram->uniformLocation("ModelViewMat");
    colorShaderVertexLocation=colorShaderProgram->attributeLocation("inPos");
    colorShaderColorLocation=colorShaderProgram->attributeLocation("inColor");

    bilbUniformProjection=bilbShader->uniformLocation("ProjectionMat");
    bilbUniformModelView=bilbShader->uniformLocation("ModelViewMat");
    bilbUniformScaleFactor=bilbShader->uniformLocation("scaleFactor");
    bilbInPosAttribute=bilbShader->attributeLocation("inPos");
    bilbTexCoordAttribute=bilbShader->attributeLocation("texCoord");

    colorShaderProgram->bind();
    vaoAnchor.create();
    vaoAnchor.bind();

    anchorArrayBuf.create();
    anchorArrayBuf.bind();
    anchorArrayBuf.allocate(anchor_points, 12 * sizeof(GLfloat));

    colorShaderProgram->enableAttributeArray(colorShaderVertexLocation);
    colorShaderProgram->setAttributeBuffer(colorShaderVertexLocation, GL_FLOAT, 0, 3, 0);
    colorShaderProgram->setAttributeValue(colorShaderColorLocation,1,1,1);

    anchorArrayBuf.release();

    anchorIndexBuf.create();
    anchorIndexBuf.bind();
    anchorIndexBuf.allocate(indicesLine, 6 * sizeof(GLushort));
    anchorIndexBuf.release();

    vaoAnchor.release();
    colorShaderProgram->release();

    bilbShader->bind();
    vaoQuad.create();
    vaoQuad.bind();
    quadArrayBuf.create();

    quadArrayBuf.bind();
    quadArrayBuf.allocate(quad_points, 20 * sizeof(GLfloat));
    quadArrayBuf.release();

    quadIndexBuf.create();
    quadIndexBuf.bind();
    quadIndexBuf.allocate(indicesQuad, 4 * sizeof(GLushort));
    quadIndexBuf.release();

    text[0]=new QOpenGLTexture(QImage(":/textures/letterX.png"));
    text[1]=new QOpenGLTexture(QImage(":/textures/letterY.png"));
    text[2]=new QOpenGLTexture(QImage(":/textures/letterZ.png"));

    bilbShader->enableAttributeArray(bilbInPosAttribute);
    bilbShader->setAttributeBuffer(bilbInPosAttribute, GL_FLOAT, 0, 3, 5 * sizeof(GLfloat));
    bilbShader->enableAttributeArray(bilbTexCoordAttribute);
    bilbShader->setAttributeBuffer(bilbTexCoordAttribute, GL_FLOAT, 3 * sizeof(GLfloat), 2, 5 * sizeof(GLfloat));
    vaoQuad.release();
    bilbShader->release();
}

AnchorGeometry::~AnchorGeometry() {
    delete text[0];
    delete text[1];
    delete text[2];
    anchorArrayBuf.destroy();
    anchorIndexBuf.destroy();
    quadArrayBuf.destroy();
    quadIndexBuf.destroy();
    vaoAnchor.destroy();
    vaoQuad.destroy();
    delete bilbShader;
    delete colorShaderProgram;
}

void AnchorGeometry::draw(QMatrix4x4 projMat, QMatrix4x4 modelViewMat, float scaleFactor) {
    glDisable(GL_CULL_FACE);

    colorShaderProgram->bind();
    colorShaderProgram->setUniformValue(colorUniformProjection,projMat);
    colorShaderProgram->setUniformValue(colorUniformModelView,modelViewMat);

    vaoAnchor.bind();
    anchorArrayBuf.bind();
    anchorIndexBuf.bind();
    colorShaderProgram->setAttributeValue(colorShaderColorLocation,1,1,1);
    glDrawElements(GL_LINES, 6, GL_UNSIGNED_SHORT, 0);
    anchorArrayBuf.release();
    anchorIndexBuf.release();
    vaoAnchor.release();
    colorShaderProgram->release();

    bilbShader->bind();
    vaoQuad.bind();
    quadArrayBuf.bind();
    quadIndexBuf.bind();

    text[0]->bind();
    QMatrix4x4 mat=modelViewMat;
    mat.translate(0.9f,0.1f,0.0f);
    bilbShader->setUniformValue(bilbUniformProjection,projMat);
    bilbShader->setUniformValue(bilbUniformModelView,mat);
    bilbShader->setUniformValue(bilbUniformScaleFactor,scaleFactor);

    bilbShader->enableAttributeArray(bilbInPosAttribute);
    bilbShader->setAttributeBuffer(bilbInPosAttribute, GL_FLOAT, 0, 3, 5 * sizeof(GLfloat));
    bilbShader->enableAttributeArray(bilbTexCoordAttribute);
    bilbShader->setAttributeBuffer(bilbTexCoordAttribute, GL_FLOAT, 3 * sizeof(GLfloat), 2, 5 * sizeof(GLfloat));

    glDrawElements(GL_QUADS, 4, GL_UNSIGNED_SHORT, 0);

    text[1]->bind();
    mat=modelViewMat;
    mat.translate(0.1f,0.9f,0.0f);
    bilbShader->setUniformValue(bilbUniformModelView,mat);
    glDrawElements(GL_QUADS, 4, GL_UNSIGNED_SHORT, 0);

    text[2]->bind();
    mat=modelViewMat;
    mat.translate(0.0f,0.1f,0.9f);
    bilbShader->setUniformValue(bilbUniformModelView,mat);
    glDrawElements(GL_QUADS, 4, GL_UNSIGNED_SHORT, 0);

    bilbShader->disableAttributeArray(bilbInPosAttribute);
    bilbShader->disableAttributeArray(bilbTexCoordAttribute);
    glEnable(GL_CULL_FACE);

    quadArrayBuf.release();
    quadIndexBuf.release();
    bilbShader->release();
    vaoQuad.release();
}
