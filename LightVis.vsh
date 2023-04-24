#ifdef GL_ES
// Set default precision to medium
precision mediump int;
precision mediump float;
#endif

attribute vec3 inPos;
attribute vec3 inNormal;
attribute vec3 inColor;

uniform mat4 ProjectionMat;
uniform mat4 ModelViewMat;
uniform mat3 NormalMat;

varying vec3 cNormal;
varying vec4 cEye;
varying vec3 cColor;

void main(void)
{
    cNormal = normalize (NormalMat * inNormal);
    cEye = -(ModelViewMat * vec4(inPos.xyz,1));
    cColor = inColor;

    gl_Position = ProjectionMat * ModelViewMat * vec4(inPos.xyz,1);
}
