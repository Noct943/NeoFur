// src/neofur_display/resource/shaders/pbr.frag
#version 300 es
precision highp float;

in vec3 v_WorldPos;
in vec3 v_Normal;
in vec2 v_TexCoord;

// PBR 材质属性
uniform sampler2D u_AlbedoTexture;
uniform vec3 u_LightDirection; // 简单的平行光
uniform vec3 u_CameraPosition;

out vec4 o_FragColor;

const float PI = 3.14159265359;

// (这里是一个简化的PBR实现，用于教学目的。
// 完整的IBL等高级特性可以后续添加)

void main() {
    vec3 albedo = texture(u_AlbedoTexture, v_TexCoord).rgb;
    vec3 normal = normalize(v_Normal);
    vec3 viewDir = normalize(u_CameraPosition - v_WorldPos);
    
    // 简单的 Lambertian 漫反射
    vec3 lightDir = normalize(u_LightDirection);
    float diff = max(dot(normal, lightDir), 0.0);
    vec3 diffuse = diff * albedo;

    // 环境光
    vec3 ambient = 0.1 * albedo;
    
    vec3 color = ambient + diffuse;
    
    // 色调映射和伽马校正 (简化)
    color = color / (color + vec3(1.0));
    color = pow(color, vec3(1.0/2.2));
    
    o_FragColor = vec4(color, 1.0);
}
