// include/neofur_display/rendering/renderer.h
#pragma once
#include <Magnum/Platform/GLContext.h>
#include <memory>
#include "neofur_display/graphics/camera.h"
#include "neofur_display/graphics/scene.h"
#include "neofur_display/platform/surface.h"
#include "neofur_utils/types.h"

namespace neofur {
namespace rendering {

/**
 * @class Renderer
 * @brief 渲染抽象层核心。管理OpenGL上下文和主渲染循环。
 * 通过静态 Create 方法进行初始化，保证构造出的对象总是有效的。
 */
class Renderer {
 public:
  ~Renderer();
  Renderer(const Renderer&) = delete;
  Renderer& operator=(const Renderer&) = delete;

  /**
   * @brief 创建并完全初始化一个渲染器实例。
   * 这是获取 Renderer 实例的唯一方式。
   * @return 成功则返回渲染器实例，失败则返回错误。
   */
  static ResultErr<std::unique_ptr<Renderer>> Create();

  /**
   * @brief 使用指定的相机渲染一个场景。
   * @param scene 要渲染的场景。
   * @param camera 观察场景的相机。
   */
  ResultErr<void> Render(graphics::Scene& scene, graphics::Camera& camera);
  
  // 获取渲染表面的尺寸
  uint32_t GetWidth() const;
  uint32_t GetHeight() const;

 private:
  // 构造函数变为私有，传入已初始化好的资源。
  Renderer(std::unique_ptr<platform::DisplaySurface> surface,
           std::unique_ptr<Magnum::Platform::GLContext> magnum_context);

  std::unique_ptr<platform::DisplaySurface> surface_;
  std::unique_ptr<Magnum::Platform::GLContext> magnum_context_;
};

}  // namespace rendering
}  // namespace neofur
