// src/graphics/material.cc
#include "neofur_display/graphics/material.h"

#include <Magnum/Trade/PbrMetallicRoughnessMaterialData.h>

#include "neofur_display/graphics/resource_manager.h"
#include "neofur_utils/log_manager.h"

namespace neofur {
namespace graphics {

PbrMaterial::PbrMaterial(ResourceManager& resource_manager,
                         const Magnum::Trade::MaterialData& material_data) {
  // --- 1. 确认材质类型并获取数据 ---
  if (!(material_data.types() & Magnum::Trade::MaterialType::PbrMetallicRoughness)) {
    utils::LogManager::Warning("材质不是 PBR 金属粗糙度类型，将使用默认 fallback 材质。");
    shader_ = std::make_unique<Magnum::Shaders::PbrMetallicRoughness>();
    shader_->setBaseColor({1.0f, 0.0f, 1.0f, 1.0f}); // 使用亮粉色表示错误/默认材质
    return;
  }
  const auto& pbr_material = material_data.as<Magnum::Trade::PbrMetallicRoughnessMaterialData>();

  // --- 2. 根据可用纹理，动态构建着色器配置 ---
  Magnum::Shaders::PbrMetallicRoughness::Flags flags;
  if (pbr_material.hasAttribute("BaseColorTexture"))
    flags |= Magnum::Shaders::PbrMetallicRoughness::Flag::BaseColorTexture;
  if (pbr_material.hasAttribute("MetalRoughnessTexture"))
    flags |= Magnum::Shaders::PbrMetallicRoughness::Flag::MetalRoughnessTexture;
  if (pbr_material.hasAttribute("NormalTexture"))
    flags |= Magnum::Shaders::PbrMetallicRoughness::Flag::NormalTexture;
  if (pbr_material.hasAttribute("OcclusionTexture"))
    flags |= Magnum::Shaders::PbrMetallicRoughness::Flag::OcclusionTexture;
  if (pbr_material.hasAttribute("EmissiveTexture"))
    flags |= Magnum::Shaders::PbrMetallicRoughness::Flag::EmissiveTexture;
  
  // 假设场景中最多有4个光源
  const UnsignedInt light_count = 4;
  shader_ = std::make_unique<Magnum::Shaders::PbrMetallicRoughness>(
      Magnum::Shaders::PbrMetallicRoughness::Configuration{}
          .setFlags(flags)
          .setLightCount(light_count));

  // --- 3. 向 ResourceManager 请求并缓存纹理指针 ---
  // 对于每种纹理，如果材质中定义了它，就通过 ResourceManager 获取。
  // 失败也无需中断，着色器会优雅地处理纹理缺失的情况。
  if (pbr_material.hasAttribute("BaseColorTexture")) {
    auto texture_res = resource_manager.get_texture(pbr_material.baseColorTexture());
    if (texture_res) base_color_texture_ = *texture_res;
    else utils::LogManager::Error(texture_res.error().message());
  }
  if (pbr_material.hasAttribute("MetalRoughnessTexture")) {
    auto texture_res = resource_manager.get_texture(pbr_material.metalRoughnessTexture());
    if (texture_res) metallic_roughness_texture_ = *texture_res;
    else utils::LogManager::Error(texture_res.error().message());
  }
  if (pbr_material.hasAttribute("NormalTexture")) {
    auto texture_res = resource_manager.get_texture(pbr_material.normalTexture());
    if (texture_res) normal_texture_ = *texture_res;
    else utils::LogManager::Error(texture_res.error().message());
  }
  if (pbr_material.hasAttribute("OcclusionTexture")) {
    auto texture_res = resource_manager.get_texture(pbr_material.occlusionTexture());
    if (texture_res) occlusion_texture_ = *texture_res;
    else utils::LogManager::Error(texture_res.error().message());
  }
  if (pbr_material.hasAttribute("EmissiveTexture")) {
    auto texture_res = resource_manager.get_texture(pbr_material.emissiveTexture());
    if (texture_res) emissive_texture_ = *texture_res;
    else utils::LogManager::Error(texture_res.error().message());
  }

  // --- 4. 设置材质的非纹理属性到到着色器 ---
  shader_->setBaseColor(pbr_material.baseColor())
      .setMetalness(pbr_material.metalness())
      .setRoughness(pbr_material.roughness())
      .setEmissiveColor(pbr_material.emissiveColor());
}

PbrMaterial::~PbrMaterial() = default;

Magnum::Shaders::PbrMetallicRoughness& PbrMaterial::bind_and_configure_shader(
    const Magnum::Matrix4& transformation,
    Magnum::SceneGraph::Camera3D& camera,
    const std::vector<Magnum::Vector4>& light_positions,
    const std::vector<Magnum::Color3>& light_colors) {
  // --- 绑定所有可用的纹理到PBR着色器固定的纹理单元 ---
  if (base_color_texture_) shader_->bindBaseColorTexture(*base_color_texture_);
  if (metallic_roughness_texture_) shader_->bindMetalRoughnessTexture(*metallic_roughness_texture_);
  if (normal_texture_) shader_->bindNormalTexture(*normal_texture_);
  if (occlusion_texture_) shader_->bindOcclusionTexture(*occlusion_texture_);
  if (emissive_texture_) shader_->bindEmissiveTexture(*emissive_texture_);

  // --- 设置每一帧都可能变化的 Uniforms ---
  shader_->setTransformationMatrix(transformation)
      .setNormalMatrix(transformation.normalMatrix())
      .setProjectionMatrix(camera.projectionMatrix())
      .setCameraPosition(camera.cameraMatrix().translation())
      .setLightPositions(light_positions)
      .setLightColors(light_colors);

  return *shader_;
}

}  // namespace graphics
}  // namespace neofur
