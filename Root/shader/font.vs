#version 140
/****************************************************
 * vertex shader for truetype rendering in 3d space *
 ****************************************************/

uniform mat4 mscreen;
uniform mat4 mtextomdl;

in vec2 coord;
smooth out vec2 fcoord;
flat out mat4 mtextoscr;

void main() {
    gl_Position = gl_ProjectionMatrix * gl_ModelViewMatrix * gl_Vertex;
    gl_FrontColor = gl_Color;
    fcoord = coord;
    mtextoscr = mscreen * gl_ProjectionMatrix * gl_ModelViewMatrix * mtextomdl;
}