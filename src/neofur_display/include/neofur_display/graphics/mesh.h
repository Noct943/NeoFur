// include/neofur_display/graphics/mesh.h
#pragma once

#include <Magnum/GL/Mesh.h>
#include <Magnum/Trade/MeshData.h>

namespace neofur {
namespace graphics {

/**
 * @class Mesh
 * @brief 几何数据的GPU资源封装。
 *
 * 负责持有渲染所需的顶点数据、索引数据等，并将其编译为
 * 一个高效的 Magnum::GL::Mesh 对象。该对象是不可变的，一旦创建，
 * 其几何数据就不会改变。
 */
class Mesh {
 public:
  /**
   * @brief 从 Magnum 的 Trade::MeshData 创建一个 Mesh。
   * Trade::MeshData 是从模型文件（如glTF）中解析出的原始几何数据。
   * @param mesh_data 原始网格数据。
   */
  explicit Mesh(const Magnum::Trade::MeshData& mesh_data);

  Mesh(const Mesh&) = delete;
  Mesh(Mesh&&) = default;
  Mesh& operator=(const Mesh&) = delete;
  Mesh& operator=(Mesh&&) = default;
  ~Mesh();
  
  /**
   * @brief 获取底层的 Magnum::GL::Mesh 对象以供绘制。
   */
  Magnum::GL::Mesh& magnum_mesh();

 private:
  Magnum::GL::Mesh magnum_mesh_;
};

}  // namespace graphics
}  // namespace neofur
