#version 140
//fragment shader

#define PER_FRAG 1
#define COLOR_MATERIAL 1

#if PER_FRAG
uniform vec4 fog;
uniform float start;
uniform float k;

smooth in vec3 v;
smooth in vec3 n;
#endif

void main() {
#if PER_FRAG
    vec3 en = normalize(n);
    vec3 ee = -normalize(v); // eye direction in unit length
    float shininess = cos(gl_FrontMaterial.shininess);
    float inv_1_minus_sh = 1.0 / (1.0 - shininess);
#if COLOR_MATERIAL
    vec4 ad = vec4(0.0, 0.0, 0.0, 1.0); // ambient & diffusion, need to * gl_Color
    vec4 es = gl_FrontMaterial.emission; // emission & specular, do not need to * gl_Color
    for (int i = 0; i < 2; ++i) {
        ad += gl_LightSource[i].ambient;
        vec3 el = normalize(gl_LightSource[i].position.xyz - v * gl_LightSource[i].position.w); // light direction in unit length
        float diff = dot(el, en);
        if (diff > 0.0) {
            ad += gl_LightSource[i].diffuse * diff;
            if (gl_FrontMaterial.specular != vec4(0, 0, 0, 0)) {
                float spec = dot(ee, 2.0 * diff * en - el);
                if (spec > shininess) {
                    es += gl_FrontLightProduct[i].specular * (spec - shininess) * inv_1_minus_sh;
                }
            }
        }
    }
    gl_FragColor = vec4((ad * gl_Color + es).xyz, gl_Color.a);
#else
    gl_FragColor = gl_FrontLightModelProduct.sceneColor;
    for (int i = 0; i < 2; ++i) {
        vec3 el = normalize(gl_LightSource[i].position.xyz - v * gl_LightSource[i].position.w); // light direction in unit length
        float diff = dot(el, en);
        if (diff > 0.0) {
            gl_FragColor += gl_FrontLightProduct[i].diffuse * diff;
            float spec = dot(ee, 2.0 * diff * en - el);
            if (spec > shininess) {
                gl_FragColor += gl_FrontLightProduct[i].specular * (spec - shininess) * inv_1_minus_sh;
            }
        }
    }
    gl_FragColor.a = gl_Color.a;
#endif
    
    float c = length(v);
    if (c > start) {
        c = exp(k * (c - start));
        gl_FragColor = mix(fog, gl_FragColor, c);
    }
#else
    gl_FragColor = gl_Color;
#endif

// todo: texture
}