#include "gldisplaymesh.h"
#include <QFile>
#include <QTextStream>
#include <QString>
#include <QStringList>
#include <iostream>
#include "glm/glm/glm.hpp"
#include <QMessageBox>

using namespace glm;
using namespace std;

GLDisplayMesh::GLDisplayMesh(QWidget *parent):
    AbstractOpenGLWidget(parent),
    ambientLight(QVector3D(0.2f,0.2f,0.2f)),
    diffuseLight(QVector3D(1.0f,1.0f,1.0f)),
    specularLight(QVector3D(0.2f,0.2f,0.2f)),
    lightDirection(QVector3D(4.0f,4.0f,4.0f))
{
    vbo=nullptr;
    ibo=nullptr;
    camDist=10;
    thetaAngle=0;
    phiAngle=90;
    lightShaderProgram=nullptr;
    updateEyePos();
    anchorGeometry=nullptr;
    shininess=0.5;
    polyFill=true;
}

GLDisplayMesh::~GLDisplayMesh() {
    makeCurrent();
    if(lightShaderProgram != nullptr)
        delete lightShaderProgram;
    if(vbo != nullptr) {
        vbo->destroy();
        delete vbo;
    }

    if(ibo != nullptr) {
        ibo->destroy();
        delete ibo;
    }

    if(anchorGeometry != nullptr)
        delete anchorGeometry;
}

void GLDisplayMesh::initializeGL() {
    initializeOpenGLFunctions();
    glClearColor(0.5f,0.5f,1.0f,1.0f);

    lightShaderProgram=new QOpenGLShaderProgram();
    BuildShaders(":/glsl/LightVis",lightShaderProgram);

    uniformAmbient=lightShaderProgram->uniformLocation("AmbientProd");
    uniformDifuse=lightShaderProgram->uniformLocation("DiffuseProd");
    uniformSpecular=lightShaderProgram->uniformLocation("SpecularProd");
    lightUniformModelView=lightShaderProgram->uniformLocation("ModelViewMat");
    lightUniformNormalMat=lightShaderProgram->uniformLocation("NormalMat");
    uniformLightDir=lightShaderProgram->uniformLocation("LightDirection");
    uniformShininess=lightShaderProgram->uniformLocation("Shininess");
    lightUniformProjection=lightShaderProgram->uniformLocation("ProjectionMat");

    lightShaderVertexLocation=lightShaderProgram->attributeLocation("inPos");
    lightShaderColorLocation=lightShaderProgram->attributeLocation("inColor");
    lightShaderNormalLocation=lightShaderProgram->attributeLocation("inNormal");

    updateEyePos();

    anchorGeometry=new AnchorGeometry();
}

void GLDisplayMesh::prepareObject(Mesh3D *mesh, bool convHull) {
    makeCurrent();

    if(vbo) {
        vbo->destroy();
        delete vbo;
        vbo=nullptr;
    }

    if(ibo) {
        ibo->destroy();
        delete ibo;
        ibo=nullptr;
    }

    std::vector<float> vertices;
    std::vector<unsigned int> indices;
    if(convHull)
        vertices=mesh->getConvVertices();
    else
        vertices=mesh->getVertex_Normals();
    vbo=new QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);
    vbo->create();
    vbo->bind();
    vbo->setUsagePattern(QOpenGLBuffer::DynamicDraw);
    vbo->allocate(vertices.data(),vertices.size() * sizeof (float));
    vbo->release();

    if(convHull)
        indices=mesh->getConvIndices();
    else
        indices=mesh->getFaceIndices();

    ibo=new QOpenGLBuffer(QOpenGLBuffer::IndexBuffer);
    ibo->create();
    ibo->bind();
    ibo->allocate(indices.data(),indices.size() * sizeof (unsigned int));
    ibo->release();

    numIndices=indices.size();

    objMin=QVector3D(mesh->getMinX(),mesh->getMinY(),mesh->getMinZ());
    objMax=QVector3D(mesh->getMaxX(),mesh->getMaxY(),mesh->getMaxZ());

    calcCamDist();

    camDist += 50;

    updateEyePos();
}

void GLDisplayMesh::writeObject(Mesh3D *mesh) {
    makeCurrent();
    if(!vbo)
        return;

    std::vector<float> vertices=mesh->getVertex_Normals();
    vbo->bind();
    vbo->write(0,vertices.data(),vertices.size() * sizeof (float));
    vbo->release();
}

void GLDisplayMesh::resizeGL(int w, int h) {
    m_proj.setToIdentity();
    m_proj.perspective(viewAngle,GLfloat(w) / GLfloat(h), 0.01f, 10000.0f);
}

void GLDisplayMesh::paintGL() {
    glEnable(GL_MULTISAMPLE);
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);

    QMatrix4x4 modelViewMat;
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    viewMatrix.setToIdentity();
    viewMatrix.lookAt(eyePos,centerEye,upEye);
    glPointSize(1);
    worldMatrix.setToIdentity();
    modelViewMat=viewMatrix * worldMatrix;

    anchorGeometry->draw(m_proj,modelViewMat,0.05);

    if(vbo == nullptr)
        return;

    lightShaderProgram->bind();
    lightShaderProgram->setUniformValue(lightUniformProjection,m_proj);
    lightShaderProgram->setUniformValue(lightUniformModelView,modelViewMat);
    lightShaderProgram->setUniformValue(lightUniformNormalMat,worldMatrix.normalMatrix());
    lightShaderProgram->setUniformValue(uniformAmbient,ambientLight);
    lightShaderProgram->setUniformValue(uniformDifuse,diffuseLight);
    lightShaderProgram->setUniformValue(uniformSpecular,specularLight);
    lightShaderProgram->setUniformValue(uniformLightDir,lightDirection);
    lightShaderProgram->setUniformValue(uniformShininess,shininess);

    vbo->bind();
    ibo->bind();

    glVertexAttribPointer(lightShaderVertexLocation, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(lightShaderVertexLocation);
    glVertexAttribPointer(lightShaderNormalLocation, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(lightShaderNormalLocation);

    glEnable(GL_MULTISAMPLE);
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);

    lightShaderProgram->setAttributeValue(lightShaderColorLocation,0.5f,0.5f,0.5f);

    glPolygonOffset(10.0,1.0);
    glEnable(GL_POLYGON_OFFSET_LINE);

    if(!polyFill)
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    glDrawElements(GL_TRIANGLES, numIndices, GL_UNSIGNED_INT, 0);

    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glDisable(GL_POLYGON_OFFSET_LINE);

    vbo->release();
    ibo->release();

    lightShaderProgram->release();
}

void GLDisplayMesh::setPolyFill(bool newPolyFill)
{
    polyFill = newPolyFill;
}

void GLDisplayMesh::setDiffuseLight(const QVector3D &newDiffuseLight)
{
    diffuseLight = newDiffuseLight;
    update();
}

void GLDisplayMesh::setLightPosition(const QVector3D &newLightPosition)
{
    lightDirection = newLightPosition;
    update();
}
