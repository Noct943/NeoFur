#pragma once

#include <poll.h>
#include <xf86drm.h>
#include <atomic>
#include <memory>

#include "neofur_display/platform/drm_device.h"
#include "neofur_utils/types.h"

namespace neofur {
namespace renderer {
namespace platform {

class DrmAtomicRequest; // 前向声明

/**
 * @class DrmDisplay
 * @brief 封装DRM显示输出的核心功能，管理一条显示管线。
 *
 * 负责自动查找并初始化一个可用的显示管线(Connector, CRTC, Plane)，
 * 并提供高性能的原子页面翻转接口。
 */
class DrmDisplay {
 public:
  ~DrmDisplay();
  DrmDisplay(const DrmDisplay&) = delete;
  DrmDisplay& operator=(const DrmDisplay&) = delete;

  /**
   * @brief 创建一个DrmDisplay实例。
   * @param device 共享的DrmDevice实例。
   */
  static ResultErr<std::unique_ptr<DrmDisplay>> Create(DrmDevPtr device);

  /**
   * @brief 执行一次原子页面翻转。
   * @param fb_id 要显示的新帧缓冲区的ID。
   * @return 成功返回void，失败返回Error。
   */
  ResultErr<void> PageFlip(uint32_t fb_id);

  /**
   * @brief 阻塞等待上一次页面翻转完成事件。
   */
  ResultErr<void> WaitForFlip();

  // --- Getters ---
  DrmDevPtr device() const { return device_; }
  const Crtc& crtc() const { return *crtc_; }
  const Connector& connector() const { return *connector_; }
  const Plane& plane() const { return *plane_; }
  const drmModeModeInfo& mode() const { return mode_; }

 private:
  DrmDisplay(DrmDevPtr device, std::unique_ptr<Connector> conn,
             std::unique_ptr<Crtc> crtc, std::unique_ptr<Plane> plane,
             const drmModeModeInfo& mode);

  ResultErr<void> InitialCommit();

  // 底层DRM资源
  DrmDevPtr device_;
  std::unique_ptr<Connector> connector_;
  std::unique_ptr<Crtc> crtc_;
  std::unique_ptr<Plane> plane_; // 我们需要一个平面来显示内容
  drmModeModeInfo mode_{};
  uint32_t mode_blob_id_ = 0;
  bool needs_modeset_ = true;

  // 事件处理相关的资源
  pollfd pfd_{};
  drmEventContext ev_ctx_{};
  std::atomic<bool> flip_completed_{false};

  // 内部回调处理
  void OnFlip();
  static void PageFlipHandler(int fd, unsigned int frame, unsigned int sec,
                              unsigned int usec, void* user_data);
};

}  // namespace platform
}  // namespace renderer
}  // namespace neofur
