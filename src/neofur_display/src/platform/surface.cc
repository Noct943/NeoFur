#include "neofur_display/surface.h"

// 包含所有platform层模块的头文件
#include <vector>

#include "neofur_display/platform/drm_display.h"
#include "neofur_display/platform/egl_backend.h"
#include "neofur_display/platform/gbm_wrapper.h"

namespace neofur {
namespace renderer {
namespace platform {

namespace {
// RAII封装一个与DRM Framebuffer绑定的GBM Buffer Object (BO)
class GbmFramebuffer {
 public:
  ~GbmFramebuffer() {
    if (bo_ && gbm_surface_) {
      gbm_surface_release_buffer(gbm_surface_, bo_);
    }
    if (fb_id_ > 0 && drm_fd_ >= 0) {
      drmModeRmFB(drm_fd_, fb_id_);
    }
  }

  static ResultErr<std::unique_ptr<GbmFramebuffer>> CreateFromNextBuffer(
      int drm_fd, gbm_surface* surface) {
    // 锁定GBM的下一个可用缓冲区
    gbm_bo* bo = gbm_surface_lock_front_buffer(surface);
    if (!bo) {
      return tl::make_unexpected(
          MAKE_ERROR_CTX(utils::ErrorCode::RenderError, "GBM锁定BO失败"));
    }

    uint32_t handle = gbm_bo_get_handle(bo).u32;
    uint32_t pitch = gbm_bo_get_stride(bo);
    uint32_t width = gbm_bo_get_width(bo);
    uint32_t height = gbm_bo_get_height(bo);
    uint32_t fb_id;

    if (drmModeAddFB(drm_fd, width, height, 24, 32, pitch, handle, &fb_id) !=
        0) {
      gbm_surface_release_buffer(surface, bo);
      return tl::make_unexpected(
          MAKE_ERROR_CTX(utils::ErrorCode::RenderError, "DRM AddFB失败"));
    }
    return std::unique_ptr<GbmFramebuffer>(
        new GbmFramebuffer(drm_fd, surface, bo, fb_id));
  }

  uint32_t id() const { return fb_id_; }

 private:
  GbmFramebuffer(int drm_fd, gbm_surface* surface, gbm_bo* bo, uint32_t fb_id)
      : drm_fd_(drm_fd), gbm_surface_(surface), bo_(bo), fb_id_(fb_id) {}
  int drm_fd_;
  gbm_surface* gbm_surface_;
  gbm_bo* bo_;
  uint32_t fb_id_;
};
}  // namespace

// PIMPL实现，组合所有底层模块
struct DisplaySurface::Impl {
  DrmDevPtr drm_device;
  std::unique_ptr<DrmDisplay> drm_display;
  std::unique_ptr<GbmDevice> gbm_device;
  std::unique_ptr<GbmSurface> gbm_surface;
  std::unique_ptr<EglBackend> egl_backend;

  // 预先创建的帧缓冲队列，用于实现高性能双缓冲/三缓冲
  std::vector<std::unique_ptr<GbmFramebuffer>> framebuffers;
  size_t buffer_count = 2;  // 使用双缓冲
  size_t current_buffer_idx = 0;
};

// --- 公开接口实现 ---

DisplaySurface::DisplaySurface(std::unique_ptr<Impl> pimpl)
    : pimpl_(std::move(pimpl)) {}
DisplaySurface::~DisplaySurface() = default;
DisplaySurface::DisplaySurface(DisplaySurface&&) noexcept = default;
DisplaySurface& DisplaySurface::operator=(DisplaySurface&&) noexcept = default;

uint32_t DisplaySurface::width() const {
  return pimpl_->drm_display->mode().hdisplay;
}
uint32_t DisplaySurface::height() const {
  return pimpl_->drm_display->mode().vdisplay;
}

ResultErr<std::unique_ptr<DisplaySurface>> DisplaySurface::Create(
    const char* drm_path) {
  auto impl = std::make_unique<Impl>();

  // 1. 初始化DRM核心
  auto drm_device_res = DrmDevice::Create(drm_path);
  if (!drm_device_res) return tl::make_unexpected(drm_device_res.error());
  impl->drm_device = std::move(*drm_device_res);

  // 2. 初始化DRM显示管线 (执行首次原子模式设置)
  auto drm_display_res = DrmDisplay::Create(impl->drm_device);
  if (!drm_display_res) return tl::make_unexpected(drm_display_res.error());
  impl->drm_display = std::move(*drm_display_res);

  // 3. 初始化GBM
  auto gbm_device_res = GbmDevice::Create(*impl->drm_device);
  if (!gbm_device_res) return tl::make_unexpected(gbm_device_res.error());
  impl->gbm_device = std::move(*gbm_device_res);

  auto gbm_surface_res =
      GbmSurface::Create(*impl->gbm_device, impl->drm_display->mode().hdisplay,
                         impl->drm_display->mode().vdisplay);
  if (!gbm_surface_res) return tl::make_unexpected(gbm_surface_res.error());
  impl->gbm_surface = std::move(*gbm_surface_res);

  // 4. 初始化EGL后端
  auto egl_backend_res =
      EglBackend::Create(*impl->gbm_device, *impl->gbm_surface);
  if (!egl_backend_res) return tl::make_unexpected(egl_backend_res.error());
  impl->egl_backend = std::move(*egl_backend_res);

  // 5. 激活EGL上下文
  PROPAGATE_ERROR_CTX(impl->egl_backend->MakeCurrent());

  // 6. 预先创建一组DRM帧缓冲，避免在渲染循环中重复创建
  for (size_t i = 0; i < impl->buffer_count; ++i) {
    auto fb_res = GbmFramebuffer::CreateFromNextBuffer(
        impl->drm_device->fd(), impl->gbm_surface->get());
    if (!fb_res) return tl::make_unexpected(fb_res.error());
    impl->framebuffers.push_back(std::move(*fb_res));
  }

  // 7. 将第一个预创建的帧缓冲提交到屏幕，完成初始化
  PROPAGATE_ERROR_CTX(impl->drm_display->PageFlip(impl->framebuffers[0]->id()));
  PROPAGATE_ERROR_CTX(impl->drm_display->WaitForFlip());

  return std::unique_ptr<DisplaySurface>(new DisplaySurface(std::move(impl)));
}

ResultErr<void> DisplaySurface::SwapBuffers() {
  auto& impl = *pimpl_;

  // 1. EGL渲染完成，后台缓冲区已准备就绪
  PROPAGATE_ERROR_CTX(impl.egl_backend->SwapBuffers());

  // 2. 轮换到下一个预创建的帧缓冲进行显示
  impl.current_buffer_idx = (impl.current_buffer_idx + 1) % impl.buffer_count;
  uint32_t next_fb_id = impl.framebuffers[impl.current_buffer_idx]->id();

  // 3. 执行高性能原子页面翻转
  PROPAGATE_ERROR_CTX(impl.drm_display->PageFlip(next_fb_id));

  // 4. 等待翻转完成，以同步渲染循环
  return impl.drm_display->WaitForFlip();
}

EGLDisplay DisplaySurface::egl_display() const {
  return pimpl_->egl_backend ? pimpl_->egl_backend->display() : EGL_NO_DISPLAY;
}

EGLSurface DisplaySurface::egl_surface() const {
  return pimpl_->egl_backend ? pimpl_->egl_backend->surface() : EGL_NO_SURFACE;
}

EGLContext DisplaySurface::egl_context() const {
  return pimpl_->egl_backend ? pimpl_->egl_backend->context() : EGL_NO_CONTEXT;
}

}  // namespace platform
}  // namespace renderer
}  // namespace neofur
