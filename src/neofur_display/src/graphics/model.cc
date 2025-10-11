// src/graphics/model.cc
#include "neofur_display/graphics/model.h"

namespace neofur {
namespace graphics {

Model::Model(Object3D& object, Magnum::SceneGraph::DrawableGroup3D& drawables,
             std::shared_ptr<Mesh> mesh, std::shared_ptr<PbrMaterial> material)
    : Magnum::SceneGraph::Drawable3D{object, &drawables}, // 关键：在构造时将自己注册到场景图
      mesh_(std::move(mesh)),
      material_(std::move(material)) 
{
    // 在此可以设置默认光照，或从场景中获取
    light_positions_ = {{5.0f, 5.0f, 7.0f, 0.0f}};
    light_colors_ = {{1.0f, 1.0f, 1.0f}};
}

Model::~Model() = default;

void Model::draw(const Magnum::Matrix4& transformationMatrix,
                 Magnum::SceneGraph::Camera3D& camera) {
  if (!mesh_ || !material_) return;

  // 1. 从材质获取并配置着色器
  //    注意：transformationMatrix 是由场景图传入的，已经包含了所有父节点的变换
  auto& shader = material_->bind_and_configure_shader(
      transformationMatrix, camera, light_positions_, light_colors_);

  // 2. 使用配置好的着色器，绘制网格
  mesh_->get_magnum_mesh().draw(shader);
}

}  // namespace graphics
}  // namespace neofur
