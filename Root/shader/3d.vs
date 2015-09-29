#version 140
//vertex shader

#define PER_FRAG 1

#if PER_FRAG
smooth out vec3 v;
smooth out vec3 n;
#else
uniform vec4 fog;
uniform float start;
uniform float k;
#endif

void main() {
#if PER_FRAG
    v = (gl_ModelViewMatrix * gl_Vertex).xyz;
    n = normalize(gl_NormalMatrix * gl_Normal);
    gl_FrontColor = gl_Color;
#else
    vec3 v = (gl_ModelViewMatrix * gl_Vertex).xyz;
    vec3 n = normalize(gl_NormalMatrix * gl_Normal);
    vec3 ee = -normalize(v); // eye direction in unit length
    vec4 ad = vec4(0.0, 0.0, 0.0, 1.0); // ambient & diffusion, need to * gl_Color
    vec4 es = gl_FrontMaterial.emission; // emission & specular, do not need to * gl_Color
    float shininess = cos(gl_FrontMaterial.shininess);
    float inv_1_minus_sh = 1.0 / (1.0 - shininess);
    for (int i = 0; i < 2; ++i) {
        ad += gl_LightSource[i].ambient;
        vec3 el = normalize(gl_LightSource[i].position.xyz - v * gl_LightSource[i].position.w); // light direction in unit length
        float diff = dot(el, n);
        if (diff > 0.0) {
            ad += gl_LightSource[i].diffuse * diff;
            float spec = dot(ee, 2.0 * diff * n - el);
            if (spec > shininess) {
                es += gl_FrontLightProduct[i].specular * (spec - shininess) * inv_1_minus_sh;
            }
        }
    }
    gl_FrontColor = vec4((ad * gl_Color + es).rgb, gl_Color.a);
    
    float c = length(v);
    if (c > start) {
        c = exp(k * (c - start));
        gl_FrontColor = mix(fog, gl_FrontColor, c);
    }
#endif
    gl_Position = gl_ProjectionMatrix * gl_ModelViewMatrix * gl_Vertex;
}