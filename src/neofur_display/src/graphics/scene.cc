// src/graphics/scene.cc
#include "neofur_display/graphics/scene.h"

#include <Corrade/PluginManager/Manager.h>
#include <Corrade/Containers/Optional.h>
#include <Magnum/Trade/AbstractImporter.h>
#include <Magnum/Trade/SceneData.h>
#include <Magnum/Trade/ObjectData3D.h>

#include "neofur_utils/log_manager.h"

namespace neofur {
namespace graphics {

Scene::Scene(const std::string& resource_path) {
  magnum_scene_ = std::make_unique<Scene3D>();
  resource_manager_ = std::make_unique<ResourceManager>(resource_path);
}

Scene::~Scene() = default;

ResultErr<void> Scene::Load(const std::string& path) {
  // --- 1. 初始化 Importer ---
  Corrade::PluginManager::Manager<Magnum::Trade::AbstractImporter> manager;
  std::unique_ptr<Magnum::Trade::AbstractImporter> importer =
      manager.loadAndInstantiate("GltfImporter");
  if (!importer) {
    return tl::make_unexpected(MAKE_ERROR_CTX(utils::ErrorCode::IOError, "无法加载 GltfImporter 插件。"));
  }

  // --- 2. 打开并解析文件 ---
  // 将 importer 标志设置为加载所有层级，这对骨骼和动画至关重要
  importer->addFlags(Magnum::Trade::ImporterFlag::LoadHierarchicalScene);
  if (!importer->openFile(resource_manager_->get_resource_path(path))) {
    return tl::make_unexpected(MAKE_ERROR_CTX(utils::ErrorCode::IOError, "无法打开场景文件: " + path));
  }
  
  // --- 3. 预加载所有网格和材质资源 ---
  // 这是关键一步：我们先把所有资源加载到 ResourceManager 中，
  // 这样后续创建 Model 时就可以共享它们。
  PROPAGATE_ERROR_CTX(resource_manager_->PreloadMeshes(*importer));
  PROPAGATE_ERROR_CTX(resource_manager_->PreloadMaterials(*importer));

  // --- 4. 解析场景结构 ---
  if (importer->sceneCount() == 0) {
    return tl::make_unexpected(MAKE_ERROR_CTX(utils::ErrorCode::NotFound, "文件中未找到场景。"));
  }
  auto scene_data = importer->scene(importer->defaultScene());
  if (!scene_data) {
    return tl::make_unexpected(MAKE_ERROR_CTX(utils::ErrorCode::NotFound, "无法加载默认场景。"));
  }

  // --- 5. 递归创建场景对象和模型 ---
  objects_.resize(importer->object3DCount());
  for (UnsignedInt i = 0; i < importer->object3DCount(); ++i) {
      auto object_data = importer->object3D(i);
      if (!object_data) continue;

      // 创建一个变换节点 (Object3D)
      auto* parent = (object_data->parent() == -1) ? magnum_scene_.get() : objects_[object_data->parent()].get();
      objects_[i] = std::make_unique<Object3D>(parent);
      objects_[i]->setTransformation(object_data->transformation());

      // 如果节点附加了网格，则为其创建一个 Model
      if (object_data->instanceType() == Magnum::Trade::ObjectInstanceType3D::Mesh && object_data->instance() != -1) {
          auto mesh_id = UnsignedInt(object_data->instance());
          auto mesh_data = importer->mesh(mesh_id);
          if (mesh_data) {
              auto mesh = resource_manager_->get_mesh(mesh_id);
              auto material = resource_manager_->get_material(mesh_data->material());
              if(mesh && material) {
                  // Model 被创建，但其变换由场景图中的 Object3D 控制
                  // 注意：我们将 Model 附加到场景图节点上，让其随节点移动
                  models_.emplace_back(std::make_unique<Model>(*objects_[i], mesh, material));
              }
          }
      }
  }

  // (未来扩展：在这里解析灯光和动画数据)
  light_positions_ = {{5.0f, 5.0f, 7.0f, 0.0f}};
  light_colors_ = {{1.0f, 1.0f, 1.0f}};

  utils::LogManager::Info("场景 '" + path + "' 加载成功。");
  return {};
}

void Scene::Draw(Camera& camera) {
  // 绘制请求被分发给每一个 Model
  for (auto& model : models_) {
    model->Draw(camera.get_magnum_camera(), light_positions_, light_colors_);
  }
}

Object3D* Scene::FindNode(const std::string& name) {
    // 这个功能的实现需要 importer 暴露节点名称，
    // 在此我们暂时返回根节点作为占位符。
    // 完整的实现需要遍历 importer->object3DName()
    return magnum_scene_.get();
}

Scene3D* Scene::get_magnum_scene() { 
  return magnum_scene_.get(); 
}

}  // namespace graphics
}  // namespace neofur
