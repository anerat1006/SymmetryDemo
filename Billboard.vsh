#ifdef GL_ES
// Set default precision to medium
precision mediump int;
precision mediump float;
#endif

attribute vec3 inPos;
attribute vec2 texCoord;

uniform mat4 ProjectionMat;
uniform mat4 ModelViewMat;
uniform float scaleFactor;
varying vec2 tCoord;

void main(void)
{
    vec4 p=vec4(inPos * scaleFactor,1.0);
    mat4 ModelView=ModelViewMat;
    ModelView[0][0]=1.0f;
    ModelView[0][1]=0.0f;
    ModelView[0][2]=0.0f;

    ModelView[1][0]=0.0f;
    ModelView[1][1]=1.0f;
    ModelView[1][2]=0.0f;

    ModelView[2][0]=0.0f;
    ModelView[2][1]=0.0f;
    ModelView[2][2]=1.0f;

    gl_Position = ProjectionMat * ModelView * p;
    tCoord = texCoord;
}
