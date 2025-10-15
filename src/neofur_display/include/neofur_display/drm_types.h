// In drm/include/drm_types.h
#ifndef DRM_TYPES_H
#define DRM_TYPES_H
#include <cstdint>
#include <memory>
#include <string>

#include "neofur_utils/my_error.h"

namespace drm {
class DrmDevice;
class Layer;
class Plane;
class Display;

enum class Color : uint32_t {
  RED = 0xFF0000,
  GREEN = 0x00FF00,
  BLUE = 0x0000FF,
  WHITE = 0xFFFFFF,
};

// DrmDevice的共享指针，用于在模块间安全地共享设备所有权
using DevPtr = std::shared_ptr<DrmDevice>;
using LayerPtr = std::unique_ptr<Layer>;
using PlanePtr = std::unique_ptr<Plane>;
using DisplayPtr = std::unique_ptr<Display>;

using utils::ResultErr;

/**
 * @brief 存储图像数据，用于线程间传递。
 */
struct ImageData {
  std::string path;
  uint32_t width;
  uint32_t height;
};

using Logger = std::function<void(const std::string& error_msg)>;
}  // namespace drm
#endif
