#include "neofur_display/platform/gbm_wrapper.h"

#include "neofur_display/platform/drm_device.h"

namespace neofur {
namespace renderer {
namespace platform {

// --- GbmDevice ---
GbmDevice::~GbmDevice() {
  if (device_) gbm_device_destroy(device_);
}
ResultErr<std::unique_ptr<GbmDevice>> GbmDevice::Create(
    const DrmDevice& drm_device) {
  auto* device = gbm_create_device(drm_device.fd());
  if (!device)
    return tl::make_unexpected(
        MAKE_ERROR_CTX(utils::ErrorCode::RenderError, "创建GBM设备失败"));
  return std::unique_ptr<GbmDevice>(new GbmDevice(device));
}
GbmDevice::GbmDevice(gbm_device* device) : device_(device) {}

// --- GbmSurface ---
GbmSurface::~GbmSurface() {
  if (surface_) gbm_surface_destroy(surface_);
}
ResultErr<std::unique_ptr<GbmSurface>> GbmSurface::Create(
    const GbmDevice& device, uint32_t width, uint32_t height) {
  gbm_surface* surface =
      gbm_surface_create(device.get(), width, height, GBM_FORMAT_XRGB8888,
                         GBM_BO_USE_SCANOUT | GBM_BO_USE_RENDERING);
  if (!surface)
    return tl::make_unexpected(
        MAKE_ERROR_CTX(utils::ErrorCode::RenderError, "创建GBM表面失败"));
  return std::unique_ptr<GbmSurface>(new GbmSurface(surface));
}
GbmSurface::GbmSurface(gbm_surface* surface) : surface_(surface) {}

}  // namespace platform
}  // namespace renderer
}  // namespace neofur
