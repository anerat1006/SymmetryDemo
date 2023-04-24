#ifdef GL_ES
// Set default precision to medium
precision mediump int;
precision mediump float;
#endif

uniform vec3 LightDirection;
uniform float Shininess;
uniform vec3 AmbientProd, DiffuseProd, SpecularProd;

varying vec3 cNormal;
varying vec4 cEye;
varying vec3 cColor;

void main()
{
    vec3 spec = vec3(0.0);

    vec3 l_dir=normalize(LightDirection);

    // normalize both input vectors
    vec3 n = normalize(cNormal);
    vec3 e = normalize(vec3(cEye));

    float intensity = max(dot(n,l_dir), 0.0);

    // if the vertex is lit compute the specular color
    if (intensity > 0.0) {
        // compute the half vector
        vec3 h = normalize(l_dir + e);
        // compute the specular term into spec
        float intSpec = max(dot(h,n), 0.0);
        spec = SpecularProd * pow(intSpec, Shininess);
    }
    //vec3 color = max(intensity *  DiffuseProd + spec, AmbientProd);
    vec3 color = cColor * max(intensity *  DiffuseProd, AmbientProd);
    // Set fragment color from texture
    gl_FragColor = vec4(color.xyz, 1.0);
}
