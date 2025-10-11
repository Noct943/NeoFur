// src/neofur_display/resource/shaders/pbr.vert
#version 300 es
layout (location = 0) in vec3 a_Position;
layout (location = 1) in vec3 a_Normal;
layout (location = 2) in vec2 a_TexCoord;

uniform mat4 u_ProjectionMatrix;
uniform mat4 u_ViewMatrix;
uniform mat4 u_ModelMatrix;
uniform mat3 u_NormalMatrix;

out vec3 v_WorldPos;
out vec3 v_Normal;
out vec2 v_TexCoord;

void main() {
    v_WorldPos = (u_ModelMatrix * vec4(a_Position, 1.0)).xyz;
    v_Normal = normalize(u_NormalMatrix * a_Normal);
    v_TexCoord = a_TexCoord;
    gl_Position = u_ProjectionMatrix * u_ViewMatrix * vec4(v_WorldPos, 1.0);
}
