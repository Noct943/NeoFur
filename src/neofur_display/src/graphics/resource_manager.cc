// src/graphics/resource_manager.cc
#include "neofur_display/graphics/resource_manager.h"

#include <Corrade/Containers/Optional.h>
#include <Corrade/Utility/Path.h>
#include <Magnum/GL/TextureFormat.h>
#include <Magnum/ImageView.h>
#include <Magnum/Trade/ImageData.h>
#include <Magnum/Trade/TextureData.h>

#include "neofur_utils/log_manager.h"

namespace neofur {
namespace graphics {

ResourceManager::ResourceManager(const std::string& resource_base_path)
    : resource_base_path_(resource_base_path),
      image_importer_manager_("MagnumTrade::AbstractImporter") {}

ResourceManager::~ResourceManager() = default;

ResultErr<void> ResourceManager::PreloadMeshes(Magnum::Trade::AbstractImporter& importer) {
  meshes_.reserve(importer.meshCount());
  for (UnsignedInt i = 0; i < importer.meshCount(); ++i) {
    auto mesh_data = importer.mesh(i);
    if (!mesh_data) {
      return tl::make_unexpected(MAKE_ERROR_CTX(
          utils::ErrorCode::IOError, "无法加载 ID 为 " + std::to_string(i) + " 的网格"));
    }
    meshes_.emplace_back(std::make_shared<Mesh>(*mesh_data));
  }
  return {};
}

ResultErr<void> ResourceManager::PreloadMaterials(Magnum::Trade::AbstractImporter& importer) {
  materials_.reserve(importer.materialCount());
  for (UnsignedInt i = 0; i < importer.materialCount(); ++i) {
    auto material_data = importer.material(i);
    if (!material_data) {
      return tl::make_unexpected(MAKE_ERROR_CTX(
          utils::ErrorCode::IOError, "无法加载 ID 为 " + std::to_string(i) + " 的材质"));
    }
    materials_.emplace_back(std::make_shared<PbrMaterial>(*this, *material_data));
  }
  return {};
}

std::shared_ptr<Mesh> ResourceManager::get_mesh(UnsignedInt id) const {
  if (id >= meshes_.size()) return nullptr;
  return meshes_[id];
}

std::shared_ptr<PbrMaterial> ResourceManager::get_material(Int id) const {
  if (id < 0 || UnsignedInt(id) >= materials_.size()) return nullptr;
  return materials_[UnsignedInt(id)];
}

ResultErr<Magnum::GL::Texture2D*> ResourceManager::get_texture(const std::string& name) {
  // 检查缓存
  if (auto it = textures_.find(name); it != textures_.end()) {
    return it->second.get();
  }
  
  // 从文件加载
  std::unique_ptr<Magnum::Trade::AbstractImporter> importer = image_importer_manager_.loadAndInstantiate("AnyImageImporter");
  if (!importer) {
    return tl::make_unexpected(MAKE_ERROR_CTX(utils::ErrorCode::NotFound, "无法加载 AnyImageImporter 插件。"));
  }
  
  const std::string full_path = get_resource_path(name);
  if (!importer->openFile(full_path)) {
    return tl::make_unexpected(MAKE_ERROR_CTX(utils::ErrorCode::IOError, "无法打开纹理文件: " + full_path));
  }
  
  auto image = importer->image2D(0);
  if (!image) {
    return tl::make_unexpected(MAKE_ERROR_CTX(utils::ErrorCode::RenderError, "无法从文件加载图像数据: " + full_path));
  }

  // 创建 GPU 纹理
  auto texture = std::make_shared<Magnum::GL::Texture2D>();
  texture->setWrapping(Magnum::GL::SamplerWrapping::Repeat)
         .setMagnificationFilter(Magnum::GL::SamplerFilter::Linear)
         .setMinificationFilter(Magnum::GL::SamplerFilter::Linear, Magnum::GL::SamplerMipmap::Linear)
         .setStorage(Magnum::Math::log2(image->size().max()) + 1, Magnum::GL::textureFormat(image->format()), image->size())
         .setSubImage(0, {}, *image)
         .generateMipmap();
         
  textures_[name] = texture;
  return texture.get();
}

ResultErr<Magnum::GL::Texture2D*> ResourceManager::get_texture(UnsignedInt id, Magnum::Trade::AbstractImporter& importer) {
    // glTF importer 将纹理与其文件名关联
    auto texture_name = importer.texture(id)->image();
    return get_texture(importer.image2DName(texture_name));
}


std::string ResourceManager::get_resource_path(const std::string& relative_path) const {
  return Corrade::Utility::Path::join(resource_base_path_, relative_path);
}

}  // namespace graphics
}  // namespace neofur
