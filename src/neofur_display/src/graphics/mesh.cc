// src/graphics/mesh.cc
#include "neofur_display/graphics/mesh.h"
#include <Magnum/MeshTools/Compile.h>

namespace neofur {
namespace graphics {

Mesh::Mesh(const Magnum::Trade::MeshData& mesh_data)
    : magnum_mesh_(Magnum::MeshTools::compile(mesh_data)) {}

Mesh::~Mesh() = default;

Magnum::GL::Mesh& Mesh::magnum_mesh() {
  return magnum_mesh_;
}

}  // namespace graphics
}  // namespace neofur
