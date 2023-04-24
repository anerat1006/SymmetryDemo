#include "oglobject.h"
#include<QFile>

OGLObject::OGLObject()
{

}

void OGLObject::BuildShaders(QString fn, QOpenGLShaderProgram *shaderProgram) {
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
