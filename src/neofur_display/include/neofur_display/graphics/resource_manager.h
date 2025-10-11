// include/neofur_display/graphics/resource_manager.h
#pragma once

#include <Magnum/Trade/AbstractImporter.h>
#include <memory>
#include <string>
#include <vector>

#include "neofur_display/graphics/material.h"
#include "neofur_display/graphics/mesh.h"
#include "neofur_utils/types.h"

namespace neofur {
namespace graphics {

/**
 * @class ResourceManager
 * @brief 场景资源的统一管理器。
 *
 * 负责从文件系统中加载、缓存并提供对 Mesh, PbrMaterial, 和 Texture
 * 等资源的共享访问。所有资源都以 std::shared_ptr 的形式管理，
 * 以实现高效的内存复用和自动的生命周期管理。
 */
class ResourceManager {
 public:
  /**
   * @brief 构造一个资源管理器。
   * @param resource_base_path 资源文件的根目录路径。
   */
  explicit ResourceManager(const std::string& resource_base_path);

  ResourceManager(const ResourceManager&) = delete;
  ResourceManager(ResourceManager&&) = default;
  ResourceManager& operator=(const ResourceManager&) = delete;
  ResourceManager& operator=(ResourceManager&&) = default;
  ~ResourceManager();

  /**
   * @brief 预加载一个模型文件中所有的网格。
   * @param importer 已打开模型文件的 Magnum importer。
   */
  ResultErr<void> PreloadMeshes(Magnum::Trade::AbstractImporter& importer);

  /**
   * @brief 预加载一个模型文件中所有的材质。
   * @param importer 已打开模型文件的 Magnum importer。
   */
  ResultErr<void> PreloadMaterials(Magnum::Trade::AbstractImporter& importer);

  /**
   * @brief 按 ID 获取一个已预加载的网格。
   * @param id 网格在模型文件中的索引。
   */
  std::shared_ptr<Mesh> get_mesh(UnsignedInt id) const;

  /**
   * @brief 按 ID 获取一个已预加载的材质。
   * @param id 材质在模型文件中的索引。
   */
  std::shared_ptr<PbrMaterial> get_material(Int id) const;

  /**
   * @brief 按名称获取一个纹理，如果未缓存则从文件加载。
   * @param name 纹理的文件名。
   * @return 成功返回纹理指针，失败返回错误。
   */
  ResultErr<Magnum::GL::Texture2D*> get_texture(const std::string& name);

  /**
   * @brief 按 ID 获取一个纹理，如果未缓存则从文件加载。
   * @param id 纹理在模型文件中的索引。
   * @param importer 已打开模型文件的 Magnum importer。
   * @return 成功返回纹理指针，失败返回错误。
   */
  ResultErr<Magnum::GL::Texture2D*> get_texture(UnsignedInt id, Magnum::Trade::AbstractImporter& importer);

  /**
   * @brief 获取资源的绝对路径。
   * @param relative_path 相对于资源根目录的路径。
   */
  std::string get_resource_path(const std::string& relative_path) const;


 private:
  std::string resource_base_path_;

  // --- 资源缓存 ---
  // 使用 std::shared_ptr 来管理共享资源的生命周期
  std::vector<std::shared_ptr<Mesh>> meshes_;
  std::vector<std::shared_ptr<PbrMaterial>> materials_;
  std::unordered_map<std::string, std::shared_ptr<Magnum::GL::Texture2D>> textures_;

  // Magnum 插件管理器，用于动态加载图像导入器
  Corrade::PluginManager::Manager<Magnum::Trade::AbstractImporter> image_importer_manager_;
};

}  // namespace graphics
}  // namespace neofur
