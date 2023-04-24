#ifdef GL_ES
// Set default precision to medium
precision mediump int;
precision mediump float;
#endif

attribute vec3 inPos;
attribute vec3 inNormal;
attribute vec3 inColor;
attribute vec3 inOffset;

uniform mat4 ProjectionMat;
uniform mat4 ViewMat;
uniform mat4 ModelMat;
uniform mat3 NormalMat;

varying vec3 cNormal;
varying vec4 cEye;
varying vec3 cColor;

void main(void)
{
    cNormal = normalize (NormalMat * inNormal);

    vec4 pom=ViewMat * (ModelMat * vec4(inPos.xyz,1) + vec4(inOffset.xyz,1));
    cEye = -pom;
    cColor = inColor;

    gl_Position = ProjectionMat * pom;
}
