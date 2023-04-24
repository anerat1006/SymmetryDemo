#include "gldisplaysymetryvoxels.h"

GLDisplaySymetryVoxels::GLDisplaySymetryVoxels(QWidget *parent) :
    GLDisplayVoxels(parent)
{
    numInstances=0;
    numInstancesL=0;
    numInstancesR=0;
    offsetBufL=nullptr;
    offsetBufR=nullptr;
}

GLDisplaySymetryVoxels::~GLDisplaySymetryVoxels() {
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

    if(offsetBufL != nullptr) {
        offsetBufL->destroy();
        delete offsetBufL;
    }
    offsetBufL=nullptr;

    if(offsetBufR != nullptr) {
        offsetBufR->destroy();
        delete offsetBufR;
    }
    offsetBufR=nullptr;
}

void GLDisplaySymetryVoxels::initializeGL() {
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

    if(offsetBufL == nullptr) {
        offsetBufL=new QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);
        offsetBufL->create();
    }
    offsetBufL->bind();
    lightShaderProgram->enableAttributeArray(lightShaderOffsetLocation);
    lightShaderProgram->setAttributeBuffer(lightShaderOffsetLocation, GL_FLOAT, 0, 3, 0);
    glVertexAttribDivisor(lightShaderOffsetLocation, 1);
    offsetBufL->release();

    if(offsetBufR == nullptr) {
        offsetBufR=new QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);
        offsetBufR->create();
    }
    offsetBufR->bind();
    lightShaderProgram->enableAttributeArray(lightShaderOffsetLocation);
    lightShaderProgram->setAttributeBuffer(lightShaderOffsetLocation, GL_FLOAT, 0, 3, 0);
    glVertexAttribDivisor(lightShaderOffsetLocation, 1);
    offsetBufR->release();

    //lightShaderProgram->setAttributeValue(lightShaderColorLocation,0,0,0);
    lightShaderProgram->release();
    indexBufFull->release();
    vao->release();

    if(dirtyBit)
        updateVoxelGrid();
}

void GLDisplaySymetryVoxels::paintGL() {
    glEnable(GL_MULTISAMPLE);

    glDepthFunc(GL_LESS);     // The Type Of Depth Test To Do
    glShadeModel(GL_SMOOTH);  // Enables Smooth Color Shading
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

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

    glEnable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);

    lightShaderProgram->bind();
    lightShaderProgram->setUniformValue(lightUniformProjection,projMatrix);
    lightShaderProgram->setUniformValue(uniformAmbient,ambientLight);
    lightShaderProgram->setUniformValue(uniformDifuse,diffuseLight);
    lightShaderProgram->setUniformValue(uniformSpecular,specularLight);
    lightShaderProgram->setUniformValue(uniformLightDir,lightDirection);
    lightShaderProgram->setUniformValue(uniformShininess,shininess);


    modelMatrix.setToIdentity();
    modelMatrix.scale(0.5*unitDim,0.5*unitDim,0.5*unitDim);
    modelViewMatrix=viewMatrix * worldMatrix;

    lightShaderProgram->setUniformValue(lightUniformView,modelViewMatrix);
    lightShaderProgram->setUniformValue(lightUniformModel,modelMatrix);
    lightShaderProgram->setUniformValue(lightUniformNormalMat,modelMatrix.normalMatrix());

    vao->bind();
    arrayBuf->bind();
    indexBufFull->bind();

    offsetBuf->bind();
    float r=0,g=0,b=1;
    lightShaderProgram->setAttributeValue(lightShaderColorLocation,r,g,b);
    lightShaderProgram->enableAttributeArray(lightShaderOffsetLocation);
    lightShaderProgram->setAttributeBuffer(lightShaderOffsetLocation, GL_FLOAT, 0, 3, 0);
    glVertexAttribDivisor(lightShaderOffsetLocation, 1);

    glDrawElementsInstanced(GL_TRIANGLE_STRIP, 14, GL_UNSIGNED_SHORT, 0, numInstances);
    offsetBuf->release();

    offsetBufR->bind();
    r=0;g=1;b=0;
    lightShaderProgram->setAttributeValue(lightShaderColorLocation,r,g,b);
    lightShaderProgram->enableAttributeArray(lightShaderOffsetLocation);
    lightShaderProgram->setAttributeBuffer(lightShaderOffsetLocation, GL_FLOAT, 0, 3, 0);
    glVertexAttribDivisor(lightShaderOffsetLocation, 1);

    glDrawElementsInstanced(GL_TRIANGLE_STRIP, 14, GL_UNSIGNED_SHORT, 0, numInstancesR);
    offsetBufR->release();

    offsetBufL->bind();
    r=1;g=0;b=0;
    lightShaderProgram->setAttributeValue(lightShaderColorLocation,r,g,b);
    lightShaderProgram->enableAttributeArray(lightShaderOffsetLocation);
    lightShaderProgram->setAttributeBuffer(lightShaderOffsetLocation, GL_FLOAT, 0, 3, 0);
    glVertexAttribDivisor(lightShaderOffsetLocation, 1);

    glDrawElementsInstanced(GL_TRIANGLE_STRIP, 14, GL_UNSIGNED_SHORT, 0, numInstancesL);
    offsetBufL->release();

    lightShaderProgram->release();

    arrayBuf->release();
    indexBufFull->release();
    vao->release();

    if(dirtyBit) {
        setVoxelsSP(VoxelsSP);
        setVoxelsL(VoxelsL);
        setVoxelsR(VoxelsR);
    }
}

void GLDisplaySymetryVoxels::setVoxelsSP(const std::vector<Eigen::Vector3d> &newVoxelsSP) {
    makeCurrent();
    Eigen::Vector3d voxCenter=voxGrid->getVoxCenter();
    int k;
    float x,y,z;
    std::vector<float> data;

    VoxelsSP=newVoxelsSP;

    if(vao == nullptr) {
        dirtyBit=true;
        return;
    }
    vao->bind();

    data.clear();
    for(k=0;k<newVoxelsSP.size();k++) {
        x=0.5*unitDim + (newVoxelsSP.at(k)[0]-voxCenter[0])*unitDim;
        y=0.5*unitDim + (newVoxelsSP.at(k)[1]-voxCenter[1])*unitDim;
        z=0.5*unitDim + (newVoxelsSP.at(k)[2]-voxCenter[2])*unitDim;

        data.push_back(x);
        data.push_back(y);
        data.push_back(z);
    }
    if(offsetBuf == nullptr) {
        dirtyBit=true;
        vao->release();
        return;
    }
    offsetBuf->bind();

    offsetBuf->allocate(data.data(), data.size() * sizeof(float));

    offsetBuf->release();
    numInstances=newVoxelsSP.size();

    vao->release();

    objMin=QVector3D(0,0,0);
    objMax=QVector3D(voxGrid->getVoxSX() * unitDim / 1.2,voxGrid->getVoxSY() * unitDim / 1.2,voxGrid->getVoxSZ() * unitDim / 1.2);

    calcCamDist();

    updateEyePos();
}

void GLDisplaySymetryVoxels::setVoxelsR(const std::vector<Eigen::Vector3d> &newVoxelsR)
{
    makeCurrent();
    Eigen::Vector3d voxCenter=voxGrid->getVoxCenter();
    int k;
    float x,y,z;
    std::vector<float> data;

    VoxelsR=newVoxelsR;

    if(vao == nullptr) {
        dirtyBit=true;
        return;
    }
    vao->bind();

    data.clear();
    for(k=0;k<newVoxelsR.size();k++) {
        x=0.5*unitDim + (newVoxelsR.at(k)[0]-voxCenter[0])*unitDim;
        y=0.5*unitDim + (newVoxelsR.at(k)[1]-voxCenter[1])*unitDim;
        z=0.5*unitDim + (newVoxelsR.at(k)[2]-voxCenter[2])*unitDim;

        data.push_back(x);
        data.push_back(y);
        data.push_back(z);
    }
    if(offsetBufR == nullptr) {
        dirtyBit=true;
        vao->release();
        return;
    }
    offsetBufR->bind();

    offsetBufR->allocate(data.data(), data.size() * sizeof(float));

    offsetBufR->release();
    numInstancesR=newVoxelsR.size();

    vao->release();

    objMin=QVector3D(0,0,0);
    objMax=QVector3D(voxGrid->getVoxSX() * unitDim / 1.2,voxGrid->getVoxSY() * unitDim / 1.2,voxGrid->getVoxSZ() * unitDim / 1.2);

    calcCamDist();

    updateEyePos();
}

void GLDisplaySymetryVoxels::setVoxelsL(const std::vector<Eigen::Vector3d> &newVoxelsL)
{
    makeCurrent();
    Eigen::Vector3d voxCenter=voxGrid->getVoxCenter();
    int k;
    float x,y,z;
    std::vector<float> data;

    VoxelsL=newVoxelsL;

    if(vao == nullptr) {
        dirtyBit=true;
        return;
    }
    vao->bind();

    data.clear();
    for(k=0;k<newVoxelsL.size();k++) {
        x=0.5*unitDim + (newVoxelsL.at(k)[0]-voxCenter[0])*unitDim;
        y=0.5*unitDim + (newVoxelsL.at(k)[1]-voxCenter[1])*unitDim;
        z=0.5*unitDim + (newVoxelsL.at(k)[2]-voxCenter[2])*unitDim;

        data.push_back(x);
        data.push_back(y);
        data.push_back(z);
    }
    if(offsetBufL == nullptr) {
        dirtyBit=true;
        vao->release();
        return;
    }
    offsetBufL->bind();

    offsetBufL->allocate(data.data(), data.size() * sizeof(float));

    offsetBufL->release();
    numInstancesL=newVoxelsL.size();

    vao->release();

    objMin=QVector3D(0,0,0);
    objMax=QVector3D(voxGrid->getVoxSX() * unitDim / 1.2,voxGrid->getVoxSY() * unitDim / 1.2,voxGrid->getVoxSZ() * unitDim / 1.2);

    calcCamDist();

    updateEyePos();
}

void GLDisplaySymetryVoxels::clean() {
    numInstances=0;
    numInstancesL=0;
    numInstancesR=0;
}
