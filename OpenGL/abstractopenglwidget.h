#ifndef ABSTRACTOPENGLWIDGET_H
#define ABSTRACTOPENGLWIDGET_H

#include <QOpenGLWidget>
#include <QOpenGLFunctions_4_0_Compatibility>
#include <QWheelEvent>
#include <QMouseEvent>
#include <QVector3D>
#include <QOpenGLShaderProgram>
#include "constants.h"

class AbstractOpenGLWidget : public QOpenGLWidget, protected QOpenGLFunctions_4_0_Compatibility
{
public:
    AbstractOpenGLWidget(QWidget *parent=nullptr);

    void setScaleFactor(float value);

    QVector<QVector3D> getIzbraneTocke() const;

    GLfloat getThetaAngle() const;

    GLfloat getPhiAngle() const;

    void setPhiTheta(double phi, double theta);

protected:
    void wheelEvent(QWheelEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;

    void BuildShaders(QString fn, QOpenGLShaderProgram *shaderProgram);
    void virtual updateEyePos();

    void virtual calcCamDist();
    void virtual calcObjBB() {}

    int mouseX,mouseY;
    bool buttonPressed,translationButtonPressed;

    QVector3D eyePos,centerEye,upEye,rightDirEye,upDirEye,tranOffset;
    GLfloat camDist,thetaAngle,phiAngle;

    QVector3D objMin,objMax;

    double viewAngle;

    float scaleFactor, tranSpeed;

    bool lockRotation;
};

#endif // ABSTRACTOPENGLWIDGET_H
