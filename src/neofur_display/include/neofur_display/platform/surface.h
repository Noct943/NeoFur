#pragma once

#include <EGL/egl.h>

#include <memory>

#include "neofur_utils/types.h"

namespace neofur {
namespace renderer {
namespace platform {
/**
 * @class DisplaySurface
 * @brief 渲染器顶层硬件接口，融合DRM Atomic与EGL/GBM。
 */
class DisplaySurface {
 public:
  ~DisplaySurface();

  DisplaySurface(const DisplaySurface&) = delete;
  DisplaySurface& operator=(const DisplaySurface&) = delete;
  DisplaySurface(DisplaySurface&&) noexcept;
  DisplaySurface& operator=(DisplaySurface&&) noexcept;

  static ResultErr<std::unique_ptr<DisplaySurface>> Create(
      const char* drm_path);

  // 高性能交换缓冲区，内部使用DRM Atomic Page Flip
  ResultErr<void> SwapBuffers();

  uint32_t width() const;
  uint32_t height() const;

  //----------------------------- Getter -----------------------------//
  EGLDisplay egl_display() const;
  EGLSurface egl_surface() const;
  EGLContext egl_context() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> pimpl_;

  explicit DisplaySurface(std::unique_ptr<Impl> pimpl);
};

}  // namespace platform
}  // namespace renderer
}  // namespace neofur
