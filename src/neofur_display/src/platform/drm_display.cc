#include "neofur_display/platform/drm_display.h"

#include <cerrno>
#include <tuple>

#include "neofur_display/platform/drm_atomic.h"
#include "neofur_utils/utils.h"

namespace neofur {
namespace renderer {
namespace platform {

namespace {

// [采纳] 完整移植您优雅的FindPreferredMode实现。
uint32_t FindPreferredModeIndex(gsl::span<const drmModeModeInfo> modes) {
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

int GetCrtcIndex(const DrmDevPtr& device, uint32_t crtc_id) {
  const auto& ids = device->GetCrtcIds();
  for (size_t i = 0; i < ids.size(); ++i) {
    if (ids[i] == crtc_id) return static_cast<int>(i);
  }
  return -1;
}

ResultErr<std::tuple<std::unique_ptr<Connector>, std::unique_ptr<Crtc>,
                     drmModeModeInfo>>
FindDisplayConfig(const DrmDevPtr& device) {
  for (auto conn_id : device->GetConnectorIds()) {
    auto conn_res = Connector::Create(device, conn_id);
    if (!conn_res) continue;
    auto& conn = *conn_res;

    if (conn->state() != DRM_MODE_CONNECTED || conn->modes().empty()) continue;

    for (auto encoder_id : conn->encoder_ids()) {
      drmModeEncoderPtr encoder = drmModeGetEncoder(device->fd(), encoder_id);
      if (!encoder) continue;
      utils::ScopeGuard enc_guard([&]() { drmModeFreeEncoder(encoder); });

      for (uint32_t crtc_id : device->GetCrtcIds()) {
        int crtc_idx = GetCrtcIndex(device, crtc_id);
        if (crtc_idx < 0 || !(encoder->possible_crtcs & (1 << crtc_idx)))
          continue;

        auto crtc_res = Crtc::Create(device, crtc_id);
        if (!crtc_res) continue;

        uint32_t mode_idx = FindPreferredModeIndex(conn->modes());
        return std::make_tuple(std::move(*conn_res), std::move(*crtc_res),
                               conn->modes()[mode_idx]);
      }
    }
  }
  return tl::make_unexpected(MAKE_ERROR_CTX(
      utils::ErrorCode::NotFound, "未找到有效的显示管线 (Connector-CRTC)"));
}

// 查找与指定CRTC兼容的主平面
ResultErr<std::unique_ptr<Plane>> FindCompatiblePlane(const DrmDevPtr& device,
                                                      uint32_t crtc_id) {
  int crtc_idx = GetCrtcIndex(device, crtc_id);
  if (crtc_idx < 0) {
    return tl::make_unexpected(
        MAKE_ERROR_CTX(utils::ErrorCode::NotFound, "无效的CRTC ID"));
  }

  for (uint32_t plane_id : device->GetPlaneIds()) {
    auto plane_res = Plane::Create(device, plane_id);
    if (!plane_res) continue;

    // 只需检查平面是否与CRTC兼容
    if ((*plane_res)->possible_crtcs() & (1 << crtc_idx)) {
      return plane_res;  // 找到第一个就返回
    }
  }
  return tl::make_unexpected(
      MAKE_ERROR_CTX(utils::ErrorCode::NotFound, "未找到兼容的平面"));
}

}  // namespace

DrmDisplay::DrmDisplay(DrmDevPtr device, std::unique_ptr<Connector> conn,
                       std::unique_ptr<Crtc> crtc, std::unique_ptr<Plane> plane,
                       const drmModeModeInfo& mode)
    : device_(std::move(device)),
      connector_(std::move(conn)),
      crtc_(std::move(crtc)),
      plane_(std::move(plane)),
      mode_(mode) {
  pfd_.fd = device_->fd();
  pfd_.events = POLLIN;
  ev_ctx_.version = DRM_EVENT_CONTEXT_VERSION;
  ev_ctx_.page_flip_handler = &DrmDisplay::PageFlipHandler;
}

DrmDisplay::~DrmDisplay() {
  if (mode_blob_id_ != 0) {
    drmModeDestroyPropertyBlob(device_->fd(), mode_blob_id_);
  }
}

ResultErr<std::unique_ptr<DrmDisplay>> DrmDisplay::Create(DrmDevPtr device) {
  // 1. 查找 Connector-CRTC 管线
  auto config_res = FindDisplayConfig(device);
  if (!config_res) return tl::make_unexpected(config_res.error());
  auto [conn, crtc, mode] = std::move(*config_res);

  // 2. 查找兼容的主平面
  auto plane_res = FindCompatiblePlane(device, crtc->id());
  if (!plane_res) return tl::make_unexpected(plane_res.error());
  auto plane = std::move(*plane_res);

  // 3. 构造Display对象
  // 使用私有构造函数创建实例
  struct MakeSharedEnabler : public DrmDisplay {
    MakeSharedEnabler(DrmDevPtr device, std::unique_ptr<Connector> conn,
                      std::unique_ptr<Crtc> crtc, std::unique_ptr<Plane> plane,
                      const drmModeModeInfo& mode)
        : DrmDisplay(std::move(device), std::move(conn), std::move(crtc),
                     std::move(plane), mode) {}
  };
  auto display = std::make_unique<MakeSharedEnabler>(
      device, std::move(conn), std::move(crtc), std::move(plane), mode);

  // 4. 执行初始的模式设置
  auto commit_res = display->InitialCommit();
  if (!commit_res) return tl::make_unexpected(commit_res.error());

  return display;
}

ResultErr<void> DrmDisplay::InitialCommit() {
  if (drmModeCreatePropertyBlob(device_->fd(), &mode_, sizeof(mode_),
                                &mode_blob_id_) != 0) {
    return tl::make_unexpected(
        MAKE_ERROR_CTX(utils::ErrorCode::IOError, "创建DRM模式Blob失败"));
  }

  DrmAtomicRequest req;
  req.AddProperty(*connector_, "CRTC_ID", crtc_->id());
  req.AddProperty(*crtc_, "MODE_ID", mode_blob_id_);
  req.AddProperty(*crtc_, "ACTIVE", 1);
  req.AddProperty(*plane_, "CRTC_ID", crtc_->id());
  // 清空帧缓冲，避免初始显示垃圾数据
  req.AddProperty(*plane_, "FB_ID", 0);
  // 设置平面尺寸，以防万一
  req.AddProperty(*plane_, "CRTC_W", mode_.hdisplay);
  req.AddProperty(*plane_, "CRTC_H", mode_.vdisplay);

  uint32_t flags = DRM_MODE_ATOMIC_NONBLOCK | DRM_MODE_ATOMIC_ALLOW_MODESET;
  return req.Commit(device_->fd(), flags, nullptr);
}

ResultErr<void> DrmDisplay::PageFlip(uint32_t fb_id) {
  DrmAtomicRequest req;

  req.AddProperty(*plane_, "FB_ID", fb_id);
  req.AddProperty(*plane_, "CRTC_ID", crtc_->id());
  req.AddProperty(*plane_, "SRC_X", 0);
  req.AddProperty(*plane_, "SRC_Y", 0);
  req.AddProperty(*plane_, "SRC_W", (uint64_t)mode_.hdisplay << 16);
  req.AddProperty(*plane_, "SRC_H", (uint64_t)mode_.vdisplay << 16);
  req.AddProperty(*plane_, "CRTC_X", 0);
  req.AddProperty(*plane_, "CRTC_Y", 0);
  req.AddProperty(*plane_, "CRTC_W", mode_.hdisplay);
  req.AddProperty(*plane_, "CRTC_H", mode_.vdisplay);

  flip_completed_ = false;
  uint32_t flags = DRM_MODE_PAGE_FLIP_EVENT | DRM_MODE_ATOMIC_NONBLOCK;

  // 使用我们修正后的Commit接口
  return req.Commit(device_->fd(), flags, this);
}

ResultErr<void> DrmDisplay::WaitForFlip() {
  while (!flip_completed_) {
    int ret = poll(&pfd_, 1, 2000);
    if (ret < 0) {
      if (errno == EINTR) continue;
      return tl::make_unexpected(MAKE_ERROR_CTX(utils::ErrorCode::RuntimeError,
                                                "等待DRM事件时Poll失败"));
    }
    if (ret == 0) {
      return tl::make_unexpected(
          MAKE_ERROR_CTX(utils::ErrorCode::RuntimeError, "等待页面翻转超时"));
    }
    if (pfd_.revents & POLLIN) {
      drmHandleEvent(device_->fd(), &ev_ctx_);
    }
  }
  return {};
}

void DrmDisplay::OnFlip() { flip_completed_ = true; }

void DrmDisplay::PageFlipHandler(int fd, unsigned int frame, unsigned int sec,
                                 unsigned int usec, void* user_data) {
  (void)fd;
  (void)frame;
  (void)sec;
  (void)usec;
  if (user_data) {
    static_cast<DrmDisplay*>(user_data)->OnFlip();
  }
}

}  // namespace platform
}  // namespace renderer
}  // namespace neofur
