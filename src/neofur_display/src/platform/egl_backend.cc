#include "neofur_display/platform/egl_backend.h"

#include <EGL/eglext.h>

#include "neofur_display/platform/gbm_wrapper.h"

namespace neofur {
namespace renderer {
namespace platform {

// --- EglDisplay ---
EglDisplay::EglDisplay(EGLDisplay display) : display_(display) {}

EglDisplay::~EglDisplay() {
  if (display_ != EGL_NO_DISPLAY) {
    eglTerminate(display_);
  }
}

ResultErr<std::unique_ptr<EglDisplay>> EglDisplay::Create(
    const GbmDevice& gbm_device) {
  EGLDisplay display =
      eglGetPlatformDisplay(EGL_PLATFORM_GBM_KHR, gbm_device.get(), nullptr);
  if (display == EGL_NO_DISPLAY) {
    return tl::make_unexpected(MAKE_ERROR_CTX(utils::ErrorCode::RenderError,
                                              "eglGetPlatformDisplay失败"));
  }

  if (eglInitialize(display, nullptr, nullptr) == EGL_FALSE) {
    return tl::make_unexpected(
        MAKE_ERROR_CTX(utils::ErrorCode::RenderError, "eglInitialize失败"));
  }
  return std::unique_ptr<EglDisplay>(new EglDisplay(display));
}

// --- EglContext ---
EglContext::EglContext(const EglDisplay& display, EGLContext context)
    : display_(&display), context_(context) {}

EglContext::~EglContext() {
  if (display_ && display_->get() != EGL_NO_DISPLAY &&
      context_ != EGL_NO_CONTEXT) {
    eglDestroyContext(display_->get(), context_);
  }
}

ResultErr<std::unique_ptr<EglContext>> EglContext::Create(
    const EglDisplay& display, EGLConfig config) {
  const EGLint context_attribs[] = {EGL_CONTEXT_CLIENT_VERSION, 2, EGL_NONE};
  EGLContext context =
      eglCreateContext(display.get(), config, EGL_NO_CONTEXT, context_attribs);
  if (context == EGL_NO_CONTEXT) {
    return tl::make_unexpected(
        MAKE_ERROR_CTX(utils::ErrorCode::RenderError, "eglCreateContext失败"));
  }
  return std::unique_ptr<EglContext>(new EglContext(display, context));
}

// --- EglSurface ---
EglSurface::EglSurface(const EglDisplay& display, EGLSurface surface)
    : display_(&display), surface_(surface) {}

EglSurface::~EglSurface() {
  if (display_ && display_->get() != EGL_NO_DISPLAY &&
      surface_ != EGL_NO_SURFACE) {
    eglDestroySurface(display_->get(), surface_);
  }
}

ResultErr<std::unique_ptr<EglSurface>> EglSurface::Create(
    const EglDisplay& display, EGLConfig config,
    const GbmSurface& gbm_surface) {
  EGLSurface surface = eglCreateWindowSurface(
      display.get(), config,
      reinterpret_cast<EGLNativeWindowType>(gbm_surface.get()), nullptr);
  if (surface == EGL_NO_SURFACE) {
    return tl::make_unexpected(MAKE_ERROR_CTX(utils::ErrorCode::RenderError,
                                              "eglCreateWindowSurface失败"));
  }
  return std::unique_ptr<EglSurface>(new EglSurface(display, surface));
}

// --- EglBackend ---
EglBackend::EglBackend(std::unique_ptr<EglDisplay> display,
                       std::unique_ptr<EglContext> context,
                       std::unique_ptr<EglSurface> surface, EGLConfig config)
    : display_(std::move(display)),
      context_(std::move(context)),
      surface_(std::move(surface)),
      config_(config) {}

EglBackend::~EglBackend() {
  // 在销毁任何EGL资源之前，必须先解除当前上下文
  if (display_ && display_->get() != EGL_NO_DISPLAY) {
    eglMakeCurrent(display_->get(), EGL_NO_SURFACE, EGL_NO_SURFACE,
                   EGL_NO_CONTEXT);
  }
  // RAII成员会自动按逆序销毁: surface -> context -> display
}

ResultErr<std::unique_ptr<EglBackend>> EglBackend::Create(
    const GbmDevice& gbm_device, const GbmSurface& gbm_surface) {
  // 1. 创建Display
  auto display_res = EglDisplay::Create(gbm_device);
  if (!display_res) return tl::make_unexpected(display_res.error());
  auto display = std::move(*display_res);

  // 2. 选择Config
  const EGLint config_attribs[] = {EGL_SURFACE_TYPE,
                                   EGL_WINDOW_BIT,
                                   EGL_RED_SIZE,
                                   8,
                                   EGL_GREEN_SIZE,
                                   8,
                                   EGL_BLUE_SIZE,
                                   8,
                                   EGL_ALPHA_SIZE,
                                   0,
                                   EGL_RENDERABLE_TYPE,
                                   EGL_OPENGL_ES2_BIT,
                                   EGL_NONE};
  EGLConfig config;
  EGLint num_config;
  if (eglChooseConfig(display->get(), config_attribs, &config, 1,
                      &num_config) != EGL_TRUE ||
      num_config != 1) {
    return tl::make_unexpected(
        MAKE_ERROR_CTX(utils::ErrorCode::RenderError, "eglChooseConfig失败"));
  }

  // 3. 绑定API
  if (eglBindAPI(EGL_OPENGL_ES_API) == EGL_FALSE) {
    return tl::make_unexpected(
        MAKE_ERROR_CTX(utils::ErrorCode::RenderError, "eglBindAPI失败"));
  }

  // 4. 创建Context
  auto context_res = EglContext::Create(*display, config);
  if (!context_res) return tl::make_unexpected(context_res.error());
  auto context = std::move(*context_res);

  // 5. 创建Surface
  auto surface_res = EglSurface::Create(*display, config, gbm_surface);
  if (!surface_res) return tl::make_unexpected(surface_res.error());
  auto surface = std::move(*surface_res);

  return std::unique_ptr<EglBackend>(new EglBackend(
      std::move(display), std::move(context), std::move(surface), config));
}

ResultErr<void> EglBackend::MakeCurrent() {
  if (eglMakeCurrent(display_->get(), surface_->get(), surface_->get(),
                     context_->get()) == EGL_FALSE) {
    return tl::make_unexpected(
        MAKE_ERROR_CTX(utils::ErrorCode::RenderError, "eglMakeCurrent失败"));
  }
  return {};
}

ResultErr<void> EglBackend::SwapBuffers() {
  if (eglSwapBuffers(display_->get(), surface_->get()) == EGL_FALSE) {
    return tl::make_unexpected(
        MAKE_ERROR_CTX(utils::ErrorCode::RenderError, "eglSwapBuffers失败"));
  }
  return {};
}

}  // namespace platform
}  // namespace renderer
}  // namespace neofur
