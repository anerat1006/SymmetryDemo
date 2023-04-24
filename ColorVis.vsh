#ifdef GL_ES
// Set default precision to medium
precision mediump int;
precision mediump float;
#endif

//uniform mat4 PVM;

uniform mat4 ProjectionMat;
uniform mat4 ModelViewMat;

attribute vec3 inPos;
attribute vec3 inColor;
varying vec3 color;

void main()
{
    gl_Position = ProjectionMat * ModelViewMat * vec4(inPos.xyz,1.0);

    color = inColor;
}
