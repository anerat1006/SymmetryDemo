#ifndef OGLOBJECT_H
#define OGLOBJECT_H

#include <QOpenGLFunctions_4_0_Compatibility>
#include <QOpenGLShaderProgram>

class OGLObject : protected QOpenGLFunctions_4_0_Compatibility
{
public:
    OGLObject();

protected:
    void BuildShaders(QString fn, QOpenGLShaderProgram *shaderProgram);
};

#endif // OGLOBJECT_H
