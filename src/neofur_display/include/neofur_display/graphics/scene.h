// include/neofur_display/graphics/scene.h
#pragma once

#include <Magnum/Trade/SceneData.h>
#include <memory>
#include <string>
#include <vector>

#include "neofur_display/graphics/camera.h"
#include "neofur_display/graphics/model.h"
#include "neofur_display/graphics/resource_manager.h"

namespace neofur {
namespace graphics {

/**
 * @class Scene
 * @brief 您的视觉世界中所有对象的容器和管理器。
 *
 * Scene 负责从一个文件中（如 .glb）加载整个场景的层级结构、
 * 模型、材质和灯光。它拥有 ResourceManager 来保证资源的高效共享，
 * 并管理场景中所有对象的生命周期和渲染流程。
 */
class Scene {
 public:
  /**
   * @brief 构造一个场景。
   * @param resource_path 指向资源文件根目录的路径。
   */
  explicit Scene(const std::string& resource_path);

  Scene(const Scene&) = delete;
  Scene(Scene&&) = default;
  Scene& operator=(const Scene&) = delete;
  Scene& operator=(Scene&&) = default;
  ~Scene();

  /**
   * @brief 从 glTF 文件加载整个场景。
   * @param path 相对于 resource_path 的场景文件路径。
   * @return 成功返回 void，失败返回错误。
   */
  ResultErr<void> Load(const std::string& path);

  /**
   * @brief 绘制整个场景。
   * @param camera 用于观察场景的相机。
   */
  void Draw(Camera& camera);

  /**
   * @brief 根据名称查找场景中的一个节点（Object3D）。
   * 这是应用层与场景交互的关键接口，例如用于控制骨骼。
   * @param name 节点在 glTF 文件中定义的名字。
   * @return 成功返回节点指针，失败返回 nullptr。
   */
  Object3D* FindNode(const std::string& name);
  
  /**
   * @brief 获取场景图的根节点，用于挂载相机等非模型对象。
   */
  Scene3D* get_magnum_scene();

 private:
  // 场景图根节点
  std::unique_ptr<Scene3D> magnum_scene_;
  // 场景中的可绘制对象
  std::vector<std::unique_ptr<Model>> models_;
  // 场景中的变换节点（代表层级关系）
  std::vector<std::unique_ptr<Object3D>> objects_;
  // 场景中的光源
  std::vector<Magnum::Vector4> light_positions_;
  std::vector<Magnum::Color3> light_colors_;

  // 场景拥有资源管理器，确保资源在场景的生命周期内有效
  std::unique_ptr<ResourceManager> resource_manager_;
};

}  // namespace graphics
}  // namespace neofur
