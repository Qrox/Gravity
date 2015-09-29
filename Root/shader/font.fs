#version 140
/******************************************************
 * fragment shader for truetype rendering in 3d space *
 ******************************************************/

#define TT_PRIM_LINE    1u
#define TT_PRIM_QSPLINE 2u
#define TT_PRIM_CSPLINE 3u

#define ANTIALIAS
//#define TRANSPARENCY_INF_DEPTH      // give antialiased part infinite depth, so as not to mask primitives behind drawn later
//#define SHOW_POINTS
//#define SQR_POINT_SIZE  1e12
//#define TRANSITION_DEBUG

uniform float emsize; // emSize * 65536 (= fixed point emSize)
uniform uint i32cnt;

/* in std140 layout, arrays are packed by the size of its elements, rounded up to the size of a vec4 (16 base-machine-units).
 * so we have to have this struct of 4 ints (and not an array of 4 ints) to access all the data.
 */
struct fourints {
    int a;
    int b;
    int c;
    int d;
};
    
// specify std140 layout for defined structure
layout(std140) uniform outline {
     fourints data[1024];
};

int getdata(uint ind) {
    uint i = ind >> 2u;
    uint j = ind & 3u;
    switch (j) {
    case 0:
        return data[i].a;
    case 1:
        return data[i].b;
    case 2:
        return data[i].c;
    case 3:
        return data[i].d; 
    }
}

smooth in vec2 fcoord;
flat in mat4 mtextoscr;
//layout (depth_greater) out float gl_FragDepth;

int transition_of_line(vec2 coord, vec2 start, vec2 end) {
    if (end.y >= coord.y ^^ start.y >= coord.y) {
        float diffy = (end - start).y;
        float crs = cross(vec3(start - coord, 0), vec3(end - coord, 0)).z;
        if (crs >= 0) {
            if (diffy > 0) return 1;
        } else {
            if (diffy < 0) return -1;
        }
    }
    return 0;
}

// p0 = start, p1 = control, p2 = end
int transition_of_qspline(vec2 coord, vec2 p0, vec2 p1, vec2 p2) {
//    if (p2.y == coord.y && p2.x >= coord.x && p1.y > coord.y && p0.y > coord.y) return -1;
//    if (p0.y == coord.y && p0.x >= coord.x && p1.y < coord.y && p2.y < coord.y) return 1;
    if (p0.x < coord.x && p1.x < coord.x && p2.x < coord.x) return 0;
    if (p0.y < coord.y && p1.y < coord.y && p2.y < coord.y) return 0;
    if (p0.y > coord.y && p1.y > coord.y && p2.y > coord.y) return 0;
//    float a = (p0 - p1 * 2 + p2).y;
//    float b = 2 * (p1 - p0).y;
//    float c = (p0 - coord).y;
//    if (a == 0) {
//        if (b != 0) {
//            float t = -c / b;
//            if (0 <= t && t <= 1) {
//                float crossx = ((1 - t) * (1 - t) * p0 + 2 * t * (1 - t) * p1 + t * t * p2).x;
//                if (crossx >= coord.x) {
//                    if (b > 0 && t != 1) return 1;
//                    else if (b < 0 && t != 0) return -1;
//                }
//            }
//        }
//    } else {
//        float delta = b * b - 4 * a * c;
//        if (delta > prec) {
//            int res = 0;
//            float sqrtdelta = sqrt(delta);
//            float t = (sqrtdelta - b) / a * .5;
//            if (0 <= t && t < 1) {
//                float crossx = ((1 - t) * (1 - t) * p0 + 2 * t * (1 - t) * p1 + t * t * p2).x;
//                if (crossx >= coord.x) ++res;
//            }
//            t = (-sqrtdelta - b) / a * .5;
//            if (0 < t && t <= 1) {
//                float crossx = ((1 - t) * (1 - t) * p0 + 2 * t * (1 - t) * p1 + t * t * p2).x;
//                if (crossx >= coord.x) --res;
//            }
//            return res;
//        }
//    }
    
    vec2 a = p0 - p1 * 2 + p2;
    vec2 b = (p1 - p0) * 2;
    vec2 c = p0 - coord;
    if (a.y == 0) {
        if (b.y != 0) {
            float t = -c.y / b.y;
            if (0 <= t && t <= 1) {
                float crossx = dot(vec3(a.x, b.x, c.x), vec3(t * t, t, 1));
                if (crossx >= 0) {
                    if (b.y > 0 && t > 0) return 1;
                    else if (b.y < 0 && t < 1) return -1;
                }
            }
        }
    } else {
        float delta = (b * b - 4 * a * c).y;
        if (delta >= 0) {
            int res = 0;
            float sqrtdelta = sqrt(delta);
            float t = (sqrtdelta - b.y) / a.y * .5;
            if (0 < t && t <= 1) {
                float crossx = dot(vec3(a.x, b.x, c.x), vec3(t * t, t, 1));
                if (crossx >= 0) ++res;
            }
            t = (-sqrtdelta - b.y) / a.y * .5;
            if (0 <= t && t < 1) {
                float crossx = dot(vec3(a.x, b.x, c.x), vec3(t * t, t, 1));
                if (crossx >= 0) --res;
            }
            return res;
        }
    }
    return 0;
}

const float pi = 3.1415926535897932384626433832795, nan = 0. / 0., inf = 1. / 0.,
             d3 = 1. / 3., nd3 = -1. / 3., pi2d3 = pi * 2. / 3.;
const vec3 cva = vec3(-1. / 27., 1. / 6., -1. / 2.), ones = vec3(1, 1, 1);
const vec2 cvb = vec2(1. / 9., -1. / 3.);

float cbrt(float a) {
    if (a < 0) return -pow(-a, 1. / 3.);
    else return pow(a, 1. / 3.);
}

vec2 solve_quadric_equation(vec3 k) {
    if (k.x == 0) return -k.z / k.y;
    float d = k.y * k.y - k.x * k.z * 4;    // delta
    if (d < 0) return vec2(nan, nan);
    else if (d == 0) return vec2(-k.y / k.x * .5, nan);
    else {
        float sd = sqrt(d);
        return (vec2(sd, -sd) - k.yy) / k.x * .5;
    }
}
    
vec3 solve_cubic_equation(vec4 k) {         // solve k0 x^3 + k1 x^2 + k2 x + k3 = 0
    if (k.x == 0) return vec3(solve_quadric_equation(k.yzw), nan);
    
//    float a, b, d;
//    k /= k.x;
//    a = -k.y * k.y * k.y / 27. + k.y * k.z / 6. - k.w / 2.;
//    b = -k.y * k.y / 9. + k.z / 3.;
//    d = a * a + b * b * b;
//    if (d > 0) {
//        float sd = sqrt(d);
//        return vec3(-k.y / 3. + cbrt(a + sd) + cbrt(a - sd), nan, nan);
//    } else if (d == 0) {
//        if (a == 0) {
//            return vec3(-k.y / 3., nan, nan);
//        } else {
//            float ca = cbrt(a);
//            return vec3(ca * 2 - k.y / 3., -k.y / 3. - ca, nan);
//        }
//    } else {    // b always < 0
//        float snb2 = sqrt(-b) * 2;
//        float ac = acos(a * pow(-b, -1.5)) / 3.;
//        return vec3(cos(ac) * snb2 - k.y / 3., cos(ac + pi2d3) * snb2 - k.y / 3., cos(ac - pi2d3) * snb2 - k.y / 3.);
//    }
    
    float a, nb, c, d;
    k /= k.x;
    vec2 vb = vec2(k.y * k.y, k.z);
    a = dot(vec3(vb * k.y, k.w), cva);          // alpha
    nb = dot(vb, cvb);                          // -beta
    c = k.y * nd3;                              // -b/3a
    d = dot(vec2(a, nb * nb), vec2(a, -nb));    // delta = alpha^2 + beta^3
    if (d > 0) {
        float sd = sqrt(d);
        return vec3(
            dot(vec3(c, cbrt(a + sd), cbrt(a - sd)), ones),
            nan, nan
        );
    } else if (d == 0) {
        if (a == 0) return vec3(c, nan, nan);
        else {
            float ca = cbrt(a);
            return vec3(ca * 2 + c, c - ca, nan);
        }
    } else {    // b always < 0
        float snb2 = sqrt(nb) * 2;
        float ac = acos(a * pow(nb, -1.5)) * d3;
        return vec3(
            cos(ac) * snb2 + c,
            cos(ac + pi2d3) * snb2 + c,
            cos(ac - pi2d3) * snb2 + c
        );
    }
}

float min_dist_to_line(vec2 o, vec2 a, vec2 b) {
    vec2 AO = o - a, AB = b - a;
    float mAB = length(AB);
    if (mAB == 0) return length(AO);
    vec2 eAB = AB / mAB;
    float u = dot(AO, eAB) / mAB;
    if (u <= 0) return length(AO);
    else if (u >= 1) return length(o - b);
    else return abs(cross(vec3(AO, 0), vec3(eAB, 0)).z);
}

float min_dist_to_qspline(vec2 o, vec2 a, vec2 b, vec2 c) {
    vec2 A = a - o, B = b - o, C = c - o;
    float A2 = dot(A, A), B2 = dot(B, B), C2 = dot(C, C);
    float AB = dot(A, B), BC = dot(B, C), CA = dot(C, A);
    vec2 gtmp = B * -2 + A + C;
    float d = AB - A2, e = dot(vec4(A2, B2, AB, CA), vec4(3, 2, -6, 1)),
           f = A2 * -3 + dot(vec4(B2, AB, BC, CA), vec4(-6, 9, 3, -3)),
           g = dot(gtmp, gtmp);
    vec3 roots = solve_cubic_equation(vec4(g, f, e, d));
    float dmin = inf;//sqrt(min(A2, C2));
    for (uint i = 0u; i < 3u; ++i) {
        float u = roots[i];
        if (isnan(u)) break;
        if (u > 0 && u < 1) {
            float v = 1 - u;
            vec2 P = A * v * v + B * v * u * 2 + C * u * u;
            dmin = min(dmin, length(P));
        }
    }
    return dmin;
}

vec2 textoscr(vec2 t) {
    vec4 tmp = mtextoscr * vec4(t, 0, 1);
    if (tmp.z == 0) return tmp.xy;
    else return tmp.xy / tmp.z;
}

void main() {
#ifdef TRANSPARENCY_INF_DEPTH
    gl_FragDepth = gl_FragCoord.z;  // 'statically' write the depth
#endif
    int winding = 0;
    for (uint i = 0u; i < i32cnt;) {
        uint cb = uint(getdata(i));
        i += 2u; // ignore dwType
        if (cb > 16u) {
            vec2 start = vec2(float(getdata(i)), float(getdata(i + 1u)));
            vec2 last = start;
            i += 2u;
#ifdef SHOW_POINTS
            vec2 diff = fcoord - start;
            if (dot(diff, diff) <= SQR_POINT_SIZE) {
                gl_FragColor = vec4(0, 0, 0, 1);
                return;
            }
#endif
            uint cnt = (cb >> 2u) - 4u;
            uint end = i + cnt;
            while (i < end) {
                uint curveheader = uint(getdata(i++));
                uint wType = curveheader & 0xFFFFU;
                uint cpfx = curveheader >> 16u;
                switch (wType) {
                case TT_PRIM_LINE:
                    while (cpfx != 0u) {
                        --cpfx;
                        vec2 curr = vec2(float(getdata(i)), float(getdata(i + 1u)));
                        i += 2u; 
                        winding += transition_of_line(fcoord, last, curr);
                        last = curr;
#ifdef SHOW_POINTS
                        vec2 diff = fcoord - curr;
                        if (dot(diff, diff) <= SQR_POINT_SIZE) {
                            gl_FragColor = vec4(float(i - (end - cnt)) / float(cnt), float(i - (end - cnt)) / float(cnt), 0, 1);
                            return;
                        }
#endif
#ifdef TRANSITION_DEBUG
                        if (winding > 0) {
                            gl_FragColor = vec4(0, 0, float(i - (end - cnt)) / float(cnt), 1);
                            return;
                        } else if (winding < 0) {
                            gl_FragColor = vec4(float(i - (end - cnt)) / float(cnt), 0, 0, 1);
                            return;
                        }
#endif
                    }
                    break;
                case TT_PRIM_QSPLINE:
                    vec2 ctrl;
                    bool hasctrl = false;
                    while (cpfx != 0u) {
                        --cpfx;
                        vec2 curr = vec2(float(getdata(i)), float(getdata(i + 1u)));
                        i += 2u;
                        if (hasctrl) {
                            if (cpfx == 0u) {
                                winding += transition_of_qspline(fcoord, last, ctrl, curr);
                                last = curr;
                            } else {
                                vec2 p2 = (curr + ctrl) * .5;
                                winding += transition_of_qspline(fcoord, last, ctrl, p2);
                                last = p2;
                                ctrl = curr;
                            }
                        } else {
                            ctrl = curr;
                            hasctrl = true;
                        }
#ifdef SHOW_POINTS
                        vec2 diff = fcoord - curr;
                        if (dot(diff, diff) <= SQR_POINT_SIZE) {
                            gl_FragColor = vec4(0, float(i - (end - cnt)) / float(cnt), 0, 1);
                            return;
                        }
#endif
#ifdef TRANSITION_DEBUG
                        if (winding > 0) {
                            gl_FragColor = vec4(0, 0, float(i - (end - cnt)) / float(cnt), 1);
                            return;
                        } else if (winding < 0) {
                            gl_FragColor = vec4(float(i - (end - cnt)) / float(cnt), 0, 0, 1);
                            return;
                        }
#endif
                    }
                    break;
                default:
                    gl_FragColor = gl_Color;
                    return;
                }
            }
            winding += transition_of_line(fcoord, last, start);
#ifdef TRANSITION_DEBUG
            if (winding > 0) {
                gl_FragColor = vec4(0, 0, float(i - (end - cnt)) / float(cnt), 1);
                return;
            } else if (winding < 0) {
                gl_FragColor = vec4(float(i - (end - cnt)) / float(cnt), 0, 0, 1);
                return;
            }
#endif
        } else {
            gl_FragColor = gl_Color;
            return;
        }
    }
    if (winding != 0) gl_FragColor = gl_Color;
#ifndef ANTIALIAS
    else discard;
#else
    else {
        vec2 scrcoord = textoscr(fcoord);
        float dmin = inf;
        for (uint i = 0u; i < i32cnt;) {
            uint cb = uint(getdata(i));
            i += 2u; // ignore dwType
            vec2 start = textoscr(vec2(float(getdata(i)), float(getdata(i + 1u))));
            vec2 last = start;
            i += 2u;
            uint cnt = (cb >> 2u) - 4u;
            uint end = i + cnt;
            while (i < end) {
                uint curveheader = uint(getdata(i++));
                uint wType = curveheader & 0xFFFFU;
                uint cpfx = curveheader >> 16u;
                switch (wType) {
                case TT_PRIM_LINE:
                    while (cpfx != 0u) {
                        --cpfx;
                        vec2 curr = textoscr(vec2(float(getdata(i)), float(getdata(i + 1u))));
                        i += 2u;
                        dmin = min(dmin, min_dist_to_line(scrcoord, last, curr));
                        last = curr;
                    }
                    break;
                case TT_PRIM_QSPLINE:
                    vec2 ctrl;
                    bool hasctrl = false;
                    while (cpfx != 0u) {
                        --cpfx;
                        vec2 curr = textoscr(vec2(float(getdata(i)), float(getdata(i + 1u))));
                        i += 2u;
                        if (hasctrl) {
                            if (cpfx == 0u) {
                                dmin = min(dmin, min_dist_to_qspline(scrcoord, last, ctrl, curr));
                                last = curr;
                            } else {
                                vec2 p2 = (curr + ctrl) * .5;
                                dmin = min(dmin, min_dist_to_qspline(scrcoord, last, ctrl, p2));
                                last = p2;
                                ctrl = curr;
                            }
                        } else {
                            ctrl = curr;
                            hasctrl = true;
                        }
                    }
                    break;
                }
            }
            dmin = min(dmin, min_dist_to_line(scrcoord, last, start));
        }
        dmin = exp(-dmin);
        if (dmin > .25) {
            gl_FragColor = gl_Color * vec4(1, 1, 1, dmin);  // component-wise
#ifdef TRANSPARENCY_INF_DEPTH
            gl_FragDepth = inf;
#endif
        } else discard;
    }
#endif
}
