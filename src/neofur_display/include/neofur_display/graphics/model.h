// include/neofur_display/graphics/model.h
#pragma once

#include <Magnum/SceneGraph/Drawable.h>
#include <Magnum/SceneGraph/Object.h>
#include <memory>
#include <vector>

#include "neofur_display/graphics/camera.h"
#include "neofur_display/graphics/material.h"
#include "neofur_display/graphics/mesh.h"

namespace neofur {
namespace graphics {

/**
 * @class Model
 * @brief 场景图中一个可渲染对象的最终形态。
 *
 * 它将一个共享的 Mesh（几何形状）和一个共享的 PbrMaterial（表面外观）
 * 组合在一起，并作为一个可绘制对象（Drawable）附加到场景图节点上。
 * Model 本身不包含变换信息，其位置、旋转、缩放完全由其所在的
 * 场景图节点（Object3D）决定。
 */
class Model : public Magnum::SceneGraph::Drawable3D {
 public:
  /**
   * @brief 构造一个模型并将其附加到场景图节点。
   * @param object 依附的场景图节点（Object3D）。Model将随此节点移动。
   * @param drawables 场景的可绘制对象组，Model会在此注册自己。
   * @param mesh 指向模型使用的共享 Mesh 资源。
   * @param material 指向模型使用的共享 PbrMaterial 资源。
   */
  Model(Object3D& object, Magnum::SceneGraph::DrawableGroup3D& drawables,
        std::shared_ptr<Mesh> mesh, std::shared_ptr<PbrMaterial> material);

  Model(const Model&) = delete;
  Model(Model&&) = default;
  Model& operator=(const Model&) = delete;
  Model& operator=(Model&&) = default;
  ~Model();

 private:
  /**
   * @brief Magnum::SceneGraph::Drawable3D 的核心虚函数。
   * 当相机调用 draw() 时，场景图会为每个可见的 Drawable 调用此函数。
   * @param transformationMatrix 场景图自动计算出的、包含父节点层级变换的最终世界矩阵。
   * @param camera 正在进行绘制的相机。
   */
  void draw(const Magnum::Matrix4& transformationMatrix,
            Magnum::SceneGraph::Camera3D& camera) override;

  std::shared_ptr<Mesh> mesh_;
  std::shared_ptr<PbrMaterial> material_;
  
  // 临时存储光照信息，实际应用中可能由 Scene 或更高级的灯光系统管理
  std::vector<Magnum::Vector4> light_positions_;
  std::vector<Magnum::Color3> light_colors_;
};

}  // namespace graphics
}  // namespace neofur
