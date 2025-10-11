#pragma once

#include <gbm.h>

#include <memory>

#include "neofur_utils/types.h"

namespace neofur {
namespace renderer {
namespace platform {

class DrmDevice;  // 前向声明

// RAII封装 gbm_device
class GbmDevice {
 public:
  ~GbmDevice();
  static ResultErr<std::unique_ptr<GbmDevice>> Create(
      const DrmDevice& drm_device);
  gbm_device* get() const { return device_; }

 private:
  explicit GbmDevice(gbm_device* device);
  gbm_device* device_ = nullptr;
};

// RAII封装 gbm_surface
class GbmSurface {
 public:
  ~GbmSurface();
  static ResultErr<std::unique_ptr<GbmSurface>> Create(const GbmDevice& device,
                                                       uint32_t width,
                                                       uint32_t height);
  gbm_surface* get() const { return surface_; }

 private:
  explicit GbmSurface(gbm_surface* surface);
  gbm_surface* surface_ = nullptr;
};

}  // namespace platform
}  // namespace renderer
}  // namespace neofur
