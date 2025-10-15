#include "neofur_display/display.h"

#include <xf86drmMode.h>

#include <cerrno>
#include <memory>
#include <tuple>
#include <utility>
#include <vector>

#include "neofur_display/atomic.h"
#include "neofur_display/device.h"
#include "neofur_display/drm_types.h"
#include "neofur_utils/expected.hpp"
#include "neofur_utils/my_error.h"
#include "neofur_utils/utils.h"

namespace drm {
namespace {

// 在给定的显示模式中，查找首选模式。
// 若无首选，则返回分辨率最高、刷新率最高的模式。
auto FindPreferredMode(gsl::span<const drmModeModeInfo> modes) {
  uint32_t best_id = 0;
  for (uint32_t i = 0; i < modes.size(); ++i) {
    const auto& mode = modes[i];
    if (mode.type & DRM_MODE_TYPE_PREFERRED) return i;
    const auto& best_mode = modes[best_id];
    long long res = static_cast<long long>(mode.hdisplay) * mode.vdisplay;
    long long best_res =
        static_cast<long long>(best_mode.hdisplay) * best_mode.vdisplay;
    if (res > best_res) {
      best_id = i;
    } else if (res == best_res && mode.vrefresh > best_mode.vrefresh) {
      best_id = i;
    }
  }
  return best_id;
}

// 查找一条完整且有效的显示管线（Connector -> Encoder -> CRTC）。
ResultErr<std::tuple<std::unique_ptr<Connector>, std::unique_ptr<Crtc>,
                     drmModeModeInfo>>
FindDisplayConfig(const DevPtr& dev) {
  for (auto conn_id : dev->GetConnectorIds()) {
    auto conn_res = Connector::Create(dev, conn_id);
    if (!conn_res) return tl::make_unexpected(conn_res.error());
    auto& conn = *conn_res;

    if (conn->state() != DRM_MODE_CONNECTED || conn->modes().empty()) continue;

    for (auto encoder_id : conn->encoder_ids()) {
      auto encoder = drmModeGetEncoder(dev->fd(), encoder_id);
      if (!encoder) {
        return tl::make_unexpected(MAKE_ERROR_CTX(
            utils::ErrorCode::IOError, "Failed to get DRM encoder resource"));
      }
      utils::ScopeGuard enc_guard([&]() { drmModeFreeEncoder(encoder); });

      for (size_t j = 0; j < dev->GetCrtcIds().size(); ++j) {
        if (encoder->possible_crtcs & (1 << j)) {
          auto crtc_res = Crtc::Create(dev, dev->GetCrtcIds()[j]);
          if (!crtc_res) return tl::make_unexpected(crtc_res.error());

          auto mode_id = FindPreferredMode(conn->modes());
          return std::make_tuple(std::move(*conn_res), std::move(*crtc_res),
                                 conn->modes()[mode_id]);
        }
      }
    }
  }
  return tl::make_unexpected(MAKE_ERROR_CTX(
      utils::ErrorCode::NotFound, "No suitable display pipeline found"));
}
}  // namespace

ResultErr<std::unique_ptr<Display>> Display::Create(DevPtr dev) {
  auto config_res = FindDisplayConfig(dev);
  if (!config_res) return tl::make_unexpected(config_res.error());

  auto [conn, crtc, mode] = std::move(*config_res);

  return std::unique_ptr<Display>(
      new Display(dev, std::move(conn), std::move(crtc), mode));
}

Display::Display(DevPtr dev, std::unique_ptr<Connector> conn,
                 std::unique_ptr<Crtc> crtc, const drmModeModeInfo& mode)
    : dev_(std::move(dev)),
      conn_(std::move(conn)),
      crtc_(std::move(crtc)),
      mode_(mode) {
  pfd_.fd = dev_->fd();
  pfd_.events = POLLIN;

  ev_ctx_.version = DRM_EVENT_CONTEXT_VERSION;
  ev_ctx_.page_flip_handler = &Display::FlipHandler;
}

ResultErr<void> Display::AddProps(AtomicRequest& req) {
  // 如果 mode blob id 还未创建，则创建它
  if (mode_blob_id_ == 0) {
    if (drmModeCreatePropertyBlob(dev_->fd(), &mode_, sizeof(mode_),
                                  &mode_blob_id_) != 0)
      return tl::make_unexpected(MAKE_ERROR_CTX(
          utils::ErrorCode::IOError, "Failed to create DRM property blob"));
  }

  req.SetProperty(*crtc_, "ACTIVE", 1);
  req.SetProperty(*crtc_, "MODE_ID", mode_blob_id_);
  req.SetProperty(*conn_, "CRTC_ID", crtc_->id());
  req.SetProperty(*conn_, "brightness", brightness_);

  return {};
}

ResultErr<void> Display::Commit(AtomicRequest& req, bool page_flip) {
  auto add_res = AddProps(req);
  if (!add_res) return tl::make_unexpected(add_res.error());

  // 根据 page_flip 参数动态构建 flags
  uint32_t flags = DRM_MODE_ATOMIC_NONBLOCK;
  if (page_flip) {
    flags |= DRM_MODE_PAGE_FLIP_EVENT;
  }
  if (need_modeset_) {
    flags |= DRM_MODE_ATOMIC_ALLOW_MODESET;
  }

  if (page_flip) {
    flip_completed_ = false;
  }

  // 只有在请求页面翻转时，才传递 this 指针作为 user_data
  auto commit_res = req.Commit(*dev_, flags, page_flip ? this : nullptr);
  if (!commit_res) return tl::make_unexpected(commit_res.error());

  if (need_modeset_) need_modeset_ = false;

  return {};
}

ResultErr<uint32_t> Display::GetCrtcIndex() const {
  auto crtc_ids = dev_->GetCrtcIds();
  for (uint32_t i = 0; i < crtc_ids.size(); ++i) {
    if (crtc_ids[i] == crtc_->id()) {
      return i;  // 找到了, 返回索引
    }
  }
  // 遍历完都没找到, 这通常不应该发生
  return tl::make_unexpected(MAKE_ERROR_CTX(utils::ErrorCode::NotFound,
                                            "无法为当前显示器找到CRTC索引"));
}

// 静态回调函数，由 libdrm 在发生页面翻转事件时调用
void Display::FlipHandler([[maybe_unused]] int fd,
                          [[maybe_unused]] unsigned int frame,
                          [[maybe_unused]] unsigned int sec,
                          [[maybe_unused]] unsigned int usec, void* user_data) {
  if (user_data) {
    static_cast<Display*>(user_data)->OnFlip();
  }
}

// 成员函数，处理翻转完成后的逻辑
void Display::OnFlip() { flip_completed_ = true; }

ResultErr<void> Display::WaitForFlip() {
  while (!flip_completed_) {
    int ret = poll(&pfd_, 1, 2000);  // 设置2秒超时

    if (ret < 0) {
      if (errno == EINTR) continue;  // 如果被信号中断，则继续等待
      return tl::make_unexpected(
          MAKE_ERROR_CTX(utils::ErrorCode::RuntimeError,
                         "Poll failed while waiting for DRM event"));
    }
    if (ret == 0) {
      return tl::make_unexpected(MAKE_ERROR_CTX(
          utils::ErrorCode::RuntimeError, "Timeout waiting for page flip"));
    }

    // 当 poll 返回有事件时，调用 drmHandleEvent 来处理事件队列，
    // 这会触发上面注册的 FlipHandler 回调
    if (pfd_.revents & POLLIN) {
      drmHandleEvent(dev_->fd(), &ev_ctx_);
    }
  }
  return {};
}
}  // namespace drm
