#ifdef GL_ES
// Set default precision to medium
precision mediump int;
precision mediump float;
#endif

uniform sampler2D texture0;
varying vec2 tCoord;

void main(void)
{
    vec4 color=texture2D(texture0, tCoord.st);
    if(color.a < 0.1)
        discard;

    gl_FragColor = color * vec4(1.0, 0.0, 0.0, 1.0);
}
