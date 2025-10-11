// src/rendering/renderer.cc
#include "neofur_display/rendering/renderer.h"
#include <Magnum/GL/DefaultFramebuffer.h>
#include <Magnum/GL/Renderer.h>
#include <Magnum/Math/Color.h>

namespace neofur {
namespace rendering {

// 私有构造函数，仅接收已初始化完成的资源
Renderer::Renderer(std::unique_ptr<platform::DisplaySurface> surface,
                   std::unique_ptr<Magnum::Platform::GLContext> magnum_context)
    : surface_(std::move(surface)), magnum_context_(std::move(magnum_context)) {}

Renderer::~Renderer() = default;

ResultErr<std::unique_ptr<Renderer>> Renderer::Create() {
  // 步骤 1: 初始化平台层 (PAL)
  auto surface_res = platform::DisplaySurface::Create("/dev/dri/card0");
  if (!surface_res) {
    // 若失败，则直接返回错误，不会创建 Renderer 对象
    return tl::make_unexpected(surface_res.error());
  }
  auto surface = std::move(*surface_res);

  // 步骤 2: 桥接 Magnum 上下文
  auto magnum_context = std::make_unique<Magnum::Platform::GLContext>();

  // 步骤 3: 配置全局GL状态
  Magnum::GL::Renderer::enable(Magnum::GL::Renderer::Feature::DepthTest);
  Magnum::GL::Renderer::enable(Magnum::GL::Renderer::Feature::FaceCulling);
  
  // 步骤 4: 所有初始化成功，使用私有构造函数创建实例
  // std::unique_ptr<Renderer>(new Renderer(...)) 模式可以访问私有构造函数
  return std::unique_ptr<Renderer>(new Renderer(std::move(surface), std::move(magnum_context)));
}

ResultErr<void> Renderer::Render(graphics::Scene& scene, graphics::Camera& camera) {
  // 1. 绑定帧缓冲并清屏
  Magnum::GL::defaultFramebuffer.bind();
  Magnum::GL::defaultFramebuffer.setClearColor({0.1f, 0.1f, 0.15f, 1.0f}); // 深灰色背景
  Magnum::GL::defaultFramebuffer.clear(
      Magnum::GL::FramebufferClear::Color | Magnum::GL::FramebufferClear::Depth);

  // 2. 委托场景进行绘制
  scene.Draw(camera);

  // 3. 通过平台层交换缓冲区
  PROPAGATE_ERROR_CTX(surface_->SwapBuffers());

  return {};
}

uint32_t Renderer::GetWidth() const { return surface_->width(); }
uint32_t Renderer::GetHeight() const { return surface_->height(); }

}  // namespace rendering
}  // namespace neofur
