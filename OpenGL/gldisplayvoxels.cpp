#include "gldisplayvoxels.h"
#include <QFile>
#include <QTextStream>
#include "glm/glm/glm.hpp"

using namespace glm;

GLDisplayVoxels::GLDisplayVoxels(QWidget *parent):
    AbstractOpenGLWidget(parent),
    ambientLight(QVector3D(0.2f,0.2f,0.2f)),
    diffuseLight(QVector3D(1.0f,1.0f,1.0f)),
    specularLight(QVector3D(0.2f,0.2f,0.2f)),
    lightDirection(QVector3D(4.0f,4.0f,4.0f))
{
  lightShaderProgram=nullptr;

  anchorGeometry=nullptr;

  camDist=10;
  thetaAngle=0;
  phiAngle=90;
  shininess=0.3f;

  buttonPressed=false;

  unitDim=0.5;

  numInstances=0;
  vao=nullptr;
  arrayBuf=nullptr;
  indexBufFull=nullptr;
  offsetBuf=nullptr;

  dirtyBit=false;
}

void GLDisplayVoxels::setLightPosX(float val) {
    lightDirection.setX(val);
    update();
}

void GLDisplayVoxels::setLightPosY(float val) {
    lightDirection.setY(val);
    update();
}

void GLDisplayVoxels::setLightPosZ(float val) {
    lightDirection.setZ(val);
    update();
}

GLDisplayVoxels::~GLDisplayVoxels() {
    makeCurrent();
    if(anchorGeometry != nullptr)
        delete anchorGeometry;
    anchorGeometry=nullptr;

    if(lightShaderProgram != nullptr)
        delete lightShaderProgram;
    lightShaderProgram=nullptr;

    if(vao != nullptr) {
        vao->destroy();
        delete vao;
    }
    vao=nullptr;

    if(arrayBuf != nullptr) {
        arrayBuf->destroy();
        delete arrayBuf;
    }
    arrayBuf=nullptr;

    if(indexBufFull != nullptr) {
        indexBufFull->destroy();
        delete indexBufFull;
    }
    indexBufFull=nullptr;

    if(offsetBuf != nullptr) {
        offsetBuf->destroy();
        delete offsetBuf;
    }
    offsetBuf=nullptr;
}

void GLDisplayVoxels::initializeGL() {
    GLushort indicesCube[]={3,2,6,7,4,2,0,3,1,6,5,4,1,0};
    GLfloat cube_strip[] = {
        -1.0f,  1.0f,  1.0f,
         1.0f,  1.0f,  1.0f,
        -1.0f, -1.0f,  1.0f,
         1.0f, -1.0f,  1.0f,
        -1.0f,  1.0f, -1.0f,
         1.0f,  1.0f, -1.0f,
         1.0f, -1.0f, -1.0f,
        -1.0f, -1.0f, -1.0f,
    };

    initializeOpenGLFunctions();
    glClearColor(0.5f,0.5f,1.0f,1.0f);
    //glClearColor(1.0f,1.0f,1.0f,1.0f);

    anchorGeometry=new AnchorGeometry();

    worldMatrix.setToIdentity();
    updateEyePos();

    centerEye.setX(0);
    centerEye.setY(0);
    centerEye.setZ(0);
    upEye.setX(0);
    upEye.setY(1);
    upEye.setZ(0);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);

    lightShaderProgram=new QOpenGLShaderProgram();
    BuildShaders(":/glsl/LightVisInstanced",lightShaderProgram);

    uniformAmbient=lightShaderProgram->uniformLocation("AmbientProd");
    uniformDifuse=lightShaderProgram->uniformLocation("DiffuseProd");
    uniformSpecular=lightShaderProgram->uniformLocation("SpecularProd");
    lightUniformModel=lightShaderProgram->uniformLocation("ModelMat");
    lightUniformView=lightShaderProgram->uniformLocation("ViewMat");
    lightUniformNormalMat=lightShaderProgram->uniformLocation("NormalMat");
    uniformLightDir=lightShaderProgram->uniformLocation("LightDirection");
    uniformShininess=lightShaderProgram->uniformLocation("Shininess");
    lightUniformProjection=lightShaderProgram->uniformLocation("ProjectionMat");

    lightShaderVertexLocation=lightShaderProgram->attributeLocation("inPos");
    lightShaderNormalLocation=lightShaderProgram->attributeLocation("inNormal");
    lightShaderColorLocation=lightShaderProgram->attributeLocation("inColor");
    lightShaderOffsetLocation=lightShaderProgram->attributeLocation("inOffset");

    lightShaderProgram->bind();
    if(vao == nullptr) {
        vao=new QOpenGLVertexArrayObject();
        vao->create();
    }
    vao->bind();

    // Generate VBOs
    if(arrayBuf == nullptr) {
        arrayBuf=new QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);
        arrayBuf->create();
    }
    arrayBuf->bind();
    arrayBuf->allocate(cube_strip, 24 * sizeof(GLfloat));

    lightShaderProgram->enableAttributeArray(lightShaderVertexLocation);
    lightShaderProgram->setAttributeBuffer(lightShaderVertexLocation, GL_FLOAT, 0, 3, 0);
    lightShaderProgram->enableAttributeArray(lightShaderNormalLocation);
    lightShaderProgram->setAttributeBuffer(lightShaderNormalLocation, GL_FLOAT, 0, 3, 0);
    arrayBuf->release();

    if(indexBufFull == nullptr) {
        indexBufFull=new QOpenGLBuffer(QOpenGLBuffer::IndexBuffer);
        indexBufFull->create();
    }
    indexBufFull->bind();
    indexBufFull->allocate(indicesCube, 14 * sizeof(GLushort));

    if(offsetBuf == nullptr) {
        offsetBuf=new QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);
        offsetBuf->create();
    }
    offsetBuf->bind();
    lightShaderProgram->enableAttributeArray(lightShaderOffsetLocation);
    lightShaderProgram->setAttributeBuffer(lightShaderOffsetLocation, GL_FLOAT, 0, 3, 0);
    glVertexAttribDivisor(lightShaderOffsetLocation, 1);
    offsetBuf->release();

    //lightShaderProgram->setAttributeValue(lightShaderColorLocation,0,0,0);
    lightShaderProgram->release();
    indexBufFull->release();
    vao->release();

    if(dirtyBit)
        updateVoxelGrid();
}

void GLDisplayVoxels::updateVoxelGrid() {
    makeCurrent();
    std::vector<Eigen::Vector3d> *voxels=voxGrid->getVoxelGrid();
    Eigen::Vector3d voxCenter=voxGrid->getVoxCenter();
    int k;
    float x,y,z;
    std::vector<float> data;
    data.clear();


    for(k=0;k<voxels->size();k++) {
        x=0.5*unitDim + (voxels->at(k)[0]-voxCenter[0])*unitDim;
        y=0.5*unitDim + (voxels->at(k)[1]-voxCenter[1])*unitDim;
        z=0.5*unitDim + (voxels->at(k)[2]-voxCenter[2])*unitDim;

        data.push_back(x);
        data.push_back(y);
        data.push_back(z);
    }

    if(vao == nullptr) {
        dirtyBit=true;
        return;
//        vao=new QOpenGLVertexArrayObject();
//        vao->create();
    }
    vao->bind();

    if(offsetBuf == nullptr) {
        dirtyBit=true;
        vao->release();
        return;
//        offsetBuf=new QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);
//        offsetBuf->create();
    }
    offsetBuf->bind();

    offsetBuf->allocate(data.data(), data.size() * sizeof(float));

    offsetBuf->release();
    vao->release();

    numInstances=voxels->size();

    objMin=QVector3D(0,0,0);
    objMax=QVector3D(voxGrid->getVoxSX() * unitDim / 1.2,voxGrid->getVoxSY() * unitDim / 1.2,voxGrid->getVoxSZ() * unitDim / 1.2);

    calcCamDist();

    updateEyePos();
}

void GLDisplayVoxels::resizeGL(int w, int h) {
    projMatrix.setToIdentity();
    projMatrix.perspective(viewAngle,GLfloat(w) / GLfloat(h), 1.0f, 1000.0f);
}

void GLDisplayVoxels::paintGL() {
    glEnable(GL_MULTISAMPLE);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glDepthFunc(GL_LESS);     // The Type Of Depth Test To Do
    glShadeModel(GL_SMOOTH);  // Enables Smooth Color Shading
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

    viewMatrix.setToIdentity();
    viewMatrix.lookAt(eyePos,centerEye,upEye);

    QMatrix4x4 modelMatrix;

    QMatrix4x4 modelViewMatrix;

    worldMatrix.setToIdentity();
    worldMatrix.scale(scaleFactor);
    modelViewMatrix=viewMatrix * worldMatrix;
    anchorGeometry->draw(projMatrix,modelViewMatrix,0.05);

    if(numInstances < 1)
        return;

    lightShaderProgram->bind();
    lightShaderProgram->setUniformValue(lightUniformProjection,projMatrix);
    lightShaderProgram->setUniformValue(uniformAmbient,ambientLight);
    lightShaderProgram->setUniformValue(uniformDifuse,diffuseLight);
    lightShaderProgram->setUniformValue(uniformSpecular,specularLight);
    lightShaderProgram->setUniformValue(uniformLightDir,lightDirection);
    lightShaderProgram->setUniformValue(uniformShininess,shininess);

    glEnable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);

    vao->bind();
    arrayBuf->bind();
    indexBufFull->bind();
    offsetBuf->bind();


    float r=1,g=1,b=1;
    lightShaderProgram->setAttributeValue(lightShaderColorLocation,r,g,b);

    modelMatrix.setToIdentity();
    modelMatrix.scale(0.5*unitDim,0.5*unitDim,0.5*unitDim);
    modelViewMatrix=viewMatrix * worldMatrix;

    lightShaderProgram->setUniformValue(lightUniformView,modelViewMatrix);
    lightShaderProgram->setUniformValue(lightUniformModel,modelMatrix);
    lightShaderProgram->setUniformValue(lightUniformNormalMat,modelMatrix.normalMatrix());

    glDrawElementsInstanced(GL_TRIANGLE_STRIP, 14, GL_UNSIGNED_SHORT, 0, numInstances);

    lightShaderProgram->release();

    arrayBuf->release();
    indexBufFull->release();
    offsetBuf->release();
    vao->release();
}

void GLDisplayVoxels::setDiffuseLight(const QVector3D &newDiffuseLight)
{
    diffuseLight = newDiffuseLight;
    update();
}

void GLDisplayVoxels::setVoxGrid(VoxelGrid *newGrid)
{
    voxGrid = newGrid;
}
