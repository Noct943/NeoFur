// include/neofur_display/graphics/material.h
#pragma once

#include <Magnum/GL/Texture.h>
#include <Magnum/Shaders/PbrMetallicRoughness.h>
#include <Magnum/Trade/MaterialData.h>
#include <memory>
#include <vector>

#include "neofur_display/graphics/camera.h"

// 前向声明，以避免在头文件中包含重量级的 resource_manager.h
namespace neofur {
namespace graphics {
class ResourceManager;
}
}

namespace neofur {
namespace graphics {

/**
 * @class PbrMaterial
 * @brief PBR（基于物理的渲染）材质的完整封装。
 *
 * 负责持有渲染一个PBR表面所需的所有属性（纹理、颜色、参数）
 * 以及配置和管理专用的 PBR 着色器。它通过 ResourceManager
 * 按需加载纹理，并将渲染逻辑与几何数据（Mesh）完全解耦。
 */
class PbrMaterial {
 public:
  /**
   * @brief 从 glTF 材质数据中创建并配置一个PBR材质。
   * @param resource_manager 用于按需加载纹理的资源管理器。
   * @param material_data 从模型文件解析出的原始材质数据。
   */
  PbrMaterial(ResourceManager& resource_manager,
              const Magnum::Trade::MaterialData& material_data);

  PbrMaterial(const PbrMaterial&) = delete;
  PbrMaterial(PbrMaterial&&) = default;
  PbrMaterial& operator=(const PbrMaterial&) = delete;
  PbrMaterial& operator=(PbrMaterial&&) = default;
  ~PbrMaterial();

  /**
   * @brief 在绘制前绑定所有纹理并配置着色器参数。
   * @param transformation 模型的变换矩阵。
   * @param camera 场景相机。
   * @param light_positions 场景光源位置。
   * @param light_colors 场景光源颜色。
   * @return 返回配置好的着色器引用，供 Model 类调用 draw()。
   */
  Magnum::Shaders::PbrMetallicRoughness& bind_and_configure_shader(
      const Magnum::Matrix4& transformation,
      Magnum::SceneGraph::Camera3D& camera,
      const std::vector<Magnum::Vector4>& light_positions,
      const std::vector<Magnum::Color3>& light_colors);

 private:
  // 材质持有着色器实例
  std::unique_ptr<Magnum::Shaders::PbrMetallicRoughness> shader_;

  // --- PBR 材质属性 ---
  // 指向由 ResourceManager 统一管理的纹理，Material 不拥有它们的所有权。
  Magnum::GL::Texture2D* base_color_texture_ = nullptr;
  Magnum::GL::Texture2D* metallic_roughness_texture_ = nullptr;
  Magnum::GL::Texture2D* normal_texture_ = nullptr;
  Magnum::GL::Texture2D* occlusion_texture_ = nullptr;
  Magnum::GL::Texture2D* emissive_texture_ = nullptr;
};

}  // namespace graphics
}  // namespace neofur
