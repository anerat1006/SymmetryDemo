#include "abstractopenglwidget.h"
#include "glm/glm/glm.hpp"
#include <QFile>
#include <QTextStream>
#include <math.h>

using namespace glm;

AbstractOpenGLWidget::AbstractOpenGLWidget(QWidget *parent)
    :QOpenGLWidget(parent)
{
    centerEye.setX(0);
    centerEye.setY(0);
    centerEye.setZ(0);
    tranOffset.setX(0);
    tranOffset.setY(0);
    tranOffset.setZ(0);
    upEye.setX(0);
    upEye.setY(1);
    upEye.setZ(0);
    scaleFactor=1;
    tranSpeed=0.001f;
    lockRotation=false;

    viewAngle=45;
}

void AbstractOpenGLWidget::updateEyePos() {
    GLfloat x,y,z;
    z=camDist*sin(radians(phiAngle))*cos(radians(thetaAngle));
    x=camDist*sin(radians(phiAngle))*sin(radians(thetaAngle));
    y=camDist*cos(radians(phiAngle));

    eyePos.setX(x);
    eyePos.setY(y);
    eyePos.setZ(z);
}

void AbstractOpenGLWidget::setPhiTheta(double phi, double theta) {
    phiAngle=phi;
    thetaAngle=theta;

    updateEyePos();
}

GLfloat AbstractOpenGLWidget::getPhiAngle() const
{
    return phiAngle;
}

GLfloat AbstractOpenGLWidget::getThetaAngle() const
{
    return thetaAngle;
}

void AbstractOpenGLWidget::calcCamDist() {
    calcObjBB();

    double vAn = viewAngle * PI / 180.0, r = ((objMax - objMin).length()) / 2.0;

    camDist=r / (2*tan(vAn / 2.0));

    if(camDist < 0.5f)
        camDist = 0.5f;
}


void AbstractOpenGLWidget::wheelEvent(QWheelEvent *event) {
    QOpenGLWidget::wheelEvent(event);
    float numStep = (event->angleDelta().y() / 8) / 15;

    if(event->modifiers() == Qt::ShiftModifier)
        numStep *=100;

    camDist-=numStep*0.05;
    if(camDist < 0.5f)
        camDist = 0.5f;

    updateEyePos();
    update();
}

void AbstractOpenGLWidget::setScaleFactor(float value)
{
    scaleFactor = value;
    update();
}

void AbstractOpenGLWidget::mousePressEvent(QMouseEvent *event) {
    QOpenGLWidget::mousePressEvent(event);
    if(event->button() == Qt::LeftButton) {
        if(event->modifiers() == Qt::ControlModifier) {
            mouseX=event->position().x();
            mouseY=event->position().y();
            translationButtonPressed=true;
        }
        else {
            mouseX=event->position().x();
            mouseY=event->position().y();
            buttonPressed=true;
        }
    }
}

void AbstractOpenGLWidget::mouseReleaseEvent(QMouseEvent *event) {
    QOpenGLWidget::mouseReleaseEvent(event);
    if(event->button() == Qt::LeftButton) {
        buttonPressed=false;
        translationButtonPressed=false;
    }
}

void AbstractOpenGLWidget::mouseMoveEvent(QMouseEvent *event) {
    QOpenGLWidget::mouseMoveEvent(event);
    int x=event->position().x(),y=event->position().y();
    if(buttonPressed) {
        if(lockRotation)
            return;
        thetaAngle+=mouseX-x;
        phiAngle+=mouseY-y;
        if(phiAngle > 179)
            phiAngle=179;
        if(phiAngle < 1)
            phiAngle=1;
        mouseX=x;
        mouseY=y;
        updateEyePos();
        update();
    }
    else if(translationButtonPressed) {
        tranOffset += (x-mouseX) * tranSpeed * rightDirEye;
        tranOffset += (y-mouseY) * tranSpeed * upDirEye;
        mouseX=x;
        mouseY=y;
        updateEyePos();
        update();
    }
}

void AbstractOpenGLWidget::BuildShaders(QString fn, QOpenGLShaderProgram *shaderProgram) {
    QString shaderSource="";

    QFile vsFile(fn+".vsh");
    vsFile.open(QIODevice::ReadOnly | QIODevice::Text);
    QTextStream vsIn(&vsFile);
    while (!vsIn.atEnd()) {
        shaderSource+=vsIn.readLine();
        shaderSource+="\n";
    }
    shaderProgram->addShaderFromSourceCode(QOpenGLShader::Vertex,shaderSource);

    shaderSource="";
    QFile fsFile(fn+".fsh");
    fsFile.open(QIODevice::ReadOnly | QIODevice::Text);
    QTextStream fsIn(&fsFile);
    while (!fsIn.atEnd()) {
        shaderSource+=fsIn.readLine();
        shaderSource+="\n";
    }
    shaderProgram->addShaderFromSourceCode(QOpenGLShader::Fragment,shaderSource);
    if(!shaderProgram->link())
    {
        qDebug() << shaderProgram->log();
    }
}
