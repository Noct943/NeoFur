#pragma once

#include <EGL/egl.h>
#include <memory>
#include "neofur_utils/types.h"

namespace neofur {
namespace renderer {
namespace platform {

class GbmDevice;
class GbmSurface;

// RAII封装 EGLDisplay
class EglDisplay {
 public:
  ~EglDisplay();
  static ResultErr<std::unique_ptr<EglDisplay>> Create(const GbmDevice& gbm_device);
  EGLDisplay get() const { return display_; }
 private:
  explicit EglDisplay(EGLDisplay display);
  EGLDisplay display_ = EGL_NO_DISPLAY;
};

// RAII封装 EGLContext
class EglContext {
 public:
  ~EglContext();
  static ResultErr<std::unique_ptr<EglContext>> Create(const EglDisplay& display, EGLConfig config);
  EGLContext get() const { return context_; }
 private:
  EglContext(const EglDisplay& display, EGLContext context);
  const EglDisplay* display_;
  EGLContext context_;
};

// RAII封装 EGLSurface
class EglSurface {
 public:
  ~EglSurface();
  static ResultErr<std::unique_ptr<EglSurface>> Create(const EglDisplay& display, EGLConfig config, const GbmSurface& gbm_surface);
  EGLSurface get() const { return surface_; }
 private:
  EglSurface(const EglDisplay& display, EGLSurface surface);
  const EglDisplay* display_;
  EGLSurface surface_;
};

/**
 * @class EglBackend
 * @brief 组合了EGL所有核心资源，作为EGL的统一管理器。
 */
class EglBackend {
 public:
  ~EglBackend();
  static ResultErr<std::unique_ptr<EglBackend>> Create(const GbmDevice& gbm_device, const GbmSurface& gbm_surface);

  ResultErr<void> MakeCurrent();
  ResultErr<void> SwapBuffers();

  //----------------------------- Getter -----------------------------//
  EGLDisplay display() const { return display_->get(); }
  EGLSurface surface() const { return surface_->get(); }
  EGLContext context() const { return context_->get(); }
  EGLConfig config() const { return config_; }

 private:
  EglBackend(std::unique_ptr<EglDisplay> display, std::unique_ptr<EglContext> context, std::unique_ptr<EglSurface> surface, EGLConfig config);
  
  std::unique_ptr<EglDisplay> display_;
  std::unique_ptr<EglContext> context_;
  std::unique_ptr<EglSurface> surface_;
  EGLConfig config_;
};

}  // namespace platform
}  // namespace renderer
}  // namespace neofur
