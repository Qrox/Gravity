#version 400

/******************************************************
 * vertex shader for quadric bezier surface rendering *
 ******************************************************/

void main() {
    gl_Position = gl_ProjectionMatrix * gl_ModelViewMatrix * gl_Vertex;
    gl_FrontColor = gl_Color;
}