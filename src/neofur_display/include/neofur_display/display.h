#ifndef display_h_H
#define display_h_H

#include <poll.h>
#include <xf86drm.h>
#include <xf86drmMode.h>

#include <atomic>
#include <memory>

#include "neofur_display/atomic.h"
#include "neofur_display/device.h"
#include "neofur_display/drm_types.h"

namespace drm {

/**
 * @class Display
 * @brief 封装了DRM显示输出的核心功能，管理一个CRTC和Connector。
 *
 * Display类负责初始化显示设备，并协调多个Layer的原子更新请求。
 * 它不直接处理绘制或缓冲管理，而是作为一个协调者，将多个图层的状态
 * 一次性提交到硬件。
 */
class Display {
 public:
  /**
   * @brief 创建一个Display实例。
   *
   * 自动查找第一个可用的、已连接的Connector以及与之兼容的
   * Encoder和CRTC，构成一条完整的显示管线。
   */
  static ResultErr<std::unique_ptr<Display>> Create(DevPtr dev);

  ~Display() = default;

  Display(const Display&) = delete;
  Display& operator=(const Display&) = delete;
  Display(Display&&) = delete;
  Display& operator=(Display&&) = delete;

  /**
   * @brief 异步提交一个构建好的原子请求。
   *
   * @param req 包含所有待更新属性的原子请求。
   * @return 成功时返回void，失败时返回错误。
   */
  ResultErr<void> Commit(AtomicRequest& req,bool page_flip);

  ResultErr<void> WaitForFlip();

  /**
   * @brief 计算并返回当前 CRTC 在设备资源列表中的索引。
   * @return 成功时返回索引值，失败时返回错误。
   */
  ResultErr<uint32_t> GetCrtcIndex() const;  // <--- 新增函数声明

  //------------------- Setters -------------------//
  void set_brightness(uint8_t level) { brightness_ = level; }

  //------------ Getters ------------//
  const drmModeModeInfo& mode() const { return mode_; }
  uint32_t crtc_id() const { return crtc_->id(); }
  uint32_t conn_id() const { return conn_->id(); }
  const Crtc& crtc() const { return *crtc_; }
  const Connector& connector() const { return *conn_; }
  DevPtr device() const { return dev_; }

 private:
  Display(DevPtr dev, std::unique_ptr<Connector> conn,
          std::unique_ptr<Crtc> crtc, const drmModeModeInfo& mode);

  DevPtr dev_;
  std::unique_ptr<Connector> conn_;
  std::unique_ptr<Crtc> crtc_;
  drmModeModeInfo mode_;
  uint32_t mode_blob_id_ = 0;

  uint8_t brightness_=0;

  // 判斷是否首次提交
  bool need_modeset_ = true;

  // 事件处理相关的资源
  pollfd pfd_{};
  drmEventContext ev_ctx_{};

  // 翻转完成的标志位
  std::atomic<bool> flip_completed_{false};

  // 内部回调处理
  void OnFlip();
  static void FlipHandler(int fd, unsigned int fraame, unsigned int sec,
                          unsigned int usec, void* user_data);

  // 为此Display设置必要的属性 (如CRTC模式)
  ResultErr<void> AddProps(AtomicRequest& req);
};

}  // namespace drm

#endif  // display_h_H
