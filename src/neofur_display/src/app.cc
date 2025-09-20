#include "neofur_display/app.h"

#include <neofur_utils/log_manager.h>
#include <neofur_utils/my_error.h>
#include <neofur_utils/thread_pool_manager.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <ratio>
#include <string>
#include <string_view>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "neofur_display/atomic.h"
#include "neofur_display/device.h"
#include "neofur_display/display.h"
#include "neofur_display/drm_types.h"
#include "neofur_display/layer.h"
#include "neofur_utils/expected.hpp"
#include "neofur_utils/log_manager.h"
#include "neofur_utils/my_error.h"

namespace drm {
namespace {

constexpr int kRequiredPlaneCount = 1;

/**
 * @brief 解析图片的尺寸大小
 * @return 返回图片的width和height
 */
ResultErr<std::pair<uint32_t, uint32_t>> ParseDimension(
    std::string_view filename) {
  size_t ext_pos = filename.rfind(".rgba");
  if (ext_pos == std::string_view::npos) {
    return tl::make_unexpected(
        MAKE_ERROR_CTX(utils::ErrorCode::InvalidArguments,
                       "无效的文件扩展名: " + std::string(filename)));
  }
  size_t x_pos = filename.rfind('x', ext_pos);
  if (x_pos == std::string_view::npos) {
    return tl::make_unexpected(
        MAKE_ERROR_CTX(utils::ErrorCode::InvalidArguments,
                       "文件名中未找到 'x': " + std::string(filename)));
  }
  size_t under_pos = filename.rfind('_', x_pos);
  if (under_pos == std::string_view::npos) {
    return tl::make_unexpected(
        MAKE_ERROR_CTX(utils::ErrorCode::InvalidArguments,
                       "文件名中未找到 '_': " + std::string(filename)));
  }
  std::string_view width_str =
      filename.substr(under_pos + 1, x_pos - under_pos - 1);
  std::string_view height_str = filename.substr(x_pos + 1, ext_pos - x_pos - 1);
  uint32_t width = std::stoul(std::string(width_str));
  uint32_t height = std::stoul(std::string(height_str));
  return std::make_pair(width, height);
}
/**
 * @brief 查找并返回指定数量的与显示器兼容的DRM平面.
 * @param dev DRM设备指针.
 * @param display 显示器对象.
 * @return 成功时返回包含平面的数组，否则返回错误.
 */
ResultErr<std::array<PlanePtr, kRequiredPlaneCount>> FindAndSortPlanes(
    const DevPtr &dev, const Display &display) {
  auto plane_ids = dev->GetPlaneIds();
  auto crtc_index_res = display.GetCrtcIndex();
  if (!crtc_index_res) {
    return tl::make_unexpected(crtc_index_res.error());
  }
  uint32_t crtc_mask = 1 << (*crtc_index_res);

  std::array<PlanePtr, kRequiredPlaneCount> result{};
  int planes_found = 0;
  for (uint32_t plane_id : plane_ids) {
    auto plane_res = Plane::Create(dev, plane_id);
    if (!plane_res) {
      continue;
    }
    // 确保平面与当前CRTC兼容
    if (((*plane_res)->possible_crtcs() & crtc_mask) == 0) {
      continue;
    }

    if (planes_found < kRequiredPlaneCount) {
      result[planes_found] = std::move(*plane_res);
      planes_found++;
    } else {
      break;  // 已找到足够数量的平面
    }
  }

  if (planes_found < kRequiredPlaneCount) {
    std::string err_msg =
        "未能找到 " + std::to_string(kRequiredPlaneCount) +
        " 个兼容的硬件图层，实际找到: " + std::to_string(planes_found);
    return tl::make_unexpected(
        MAKE_ERROR_CTX(utils::ErrorCode::NotFound, err_msg));
  }
  return result;
}

// 辅助函数：创建支持双缓冲的动态图层
ResultErr<LayerPtr> CreateLayer(const DevPtr &dev, std::unique_ptr<Plane> plane,
                                uint32_t width, uint32_t height) {
  std::vector<FrameBuffer> bufs;
  bufs.reserve(2);

  // [核心修复] 计算满足硬件对齐要求的 pitch
  // Rockchip 通常要求 64 字节对齐
  const uint32_t bpp = 4;  // ARGB8888
  const uint32_t alignment = 64;
  uint32_t pitch = (width * bpp + alignment - 1) & ~(alignment - 1);

  for (int i = 0; i < 2; ++i) {
    // 1. 创建 DmaBuffer 时传入 pitch 以便计算总大小
    auto dma_buf_res = DmaBuffer::Create(width, height, pitch);
    if (!dma_buf_res) return tl::make_unexpected(dma_buf_res.error());

    // 2. 从 DmaBuffer 创建 FrameBuffer 时传入 pitch
    auto fb_res = FrameBuffer::Create(dev, std::move(*dma_buf_res), pitch);
    if (!fb_res) return tl::make_unexpected(fb_res.error());

    bufs.push_back(std::move(*fb_res));
  }
  return Layer::Create(dev, std::move(plane), std::move(bufs));
}
}  // namespace

ResultErr<std::unique_ptr<DrmApp>> DrmApp::Create(std::string_view drm_path,
                                                  std::string_view image_path) {
  auto dev_res = DrmDevice::Create(drm_path);
  if (!dev_res) return tl::make_unexpected(dev_res.error());
  auto &dev = *dev_res;

  auto display_res = Display::Create(dev);
  if (!display_res) return tl::make_unexpected(display_res.error());
  auto &display = *display_res;

  auto planes_res = FindAndSortPlanes(dev, *display);
  if (!planes_res) return tl::make_unexpected(planes_res.error());
  auto &planes = *planes_res;

  const auto &mode = display->mode();

  // 使用双缓冲创建图层
  auto dim_res = ParseDimension(image_path);
  if (!dim_res) return tl::make_unexpected(dim_res.error());
  auto [w, h] = *dim_res;

  auto layer_res = CreateLayer(dev, std::move(planes[1]), w, h);
  if (!layer_res) return tl::make_unexpected(layer_res.error());
  auto &layer = *layer_res;
  ImageData initial_image = {std::string(image_path), w, h};
  PROPAGATE_ERROR_CTX((*layer_res)->DrawRgba(initial_image));

  return std::unique_ptr<DrmApp>(
      new DrmApp(std::move(display), std::move(layer)));
}

DrmApp::DrmApp(DisplayPtr display, LayerPtr layer)
    : display_(std::move(display)), layer_(std::move(layer)), running_(true) {}

DrmApp::~DrmApp() {
  if (running_.load()) Stop();
}

void DrmApp::Stop() {
  running_ = false;
  image_queue_.cv.notify_one();
}

ResultErr<void> DrmApp::UpdateScene(const ImageRequest &request) {
  bool needs_commit = false;

  if (request.data) {
    PROPAGATE_ERROR_CTX(layer_->DrawRgba(*request.data));
    needs_commit = true;
  }

  if (request.brightness) {
    display_->set_brightness(*request.brightness);
    needs_commit = true;
  }

  if (needs_commit) {
    AtomicRequest req;
    PrepareSceneRequest(req);
    // 任何更新都需要页面翻转来使其可见
    return CommitScene(req, true);
  }

  return {};
}

void DrmApp::RequestUpdate(std::optional<std::string> image_path,
                           std::optional<uint8_t> brightness) {
  auto &pool = utils::ThreadPoolManager::Instance();
  pool.detach_task([=] {
    if (!running_) return;

    ImageRequest request;
    request.brightness = brightness;

    // 解析图像数据
    if (image_path) {
      auto dim_res = ParseDimension(*image_path);
      if (!dim_res) {
        utils::LogManager::Error("图片尺寸解析失败：" +
                                 dim_res.error().message());
        return;
      }
      auto [width, height] = *dim_res;
      request.data = {*image_path, width, height};
    }

    // 将请求放入队列
    std::unique_lock lock(image_queue_.mutex);
    if (image_queue_.images.size() < image_queue_.kCapacity) {
      image_queue_.images.push(std::move(request));
      lock.unlock();
      image_queue_.cv.notify_one();
    } else {
      utils::LogManager::Error("图像队列已满，更新失败：" + *image_path);
    }
  });
}

ResultErr<void> DrmApp::Run() {
  while (running_) {
    ImageRequest request;
    {
      std::unique_lock lock(image_queue_.mutex);
      image_queue_.cv.wait(
          lock, [this] { return !image_queue_.images.empty() || !running_; });

      if (!running_) break;

      request = std::move(image_queue_.images.front());
      image_queue_.images.pop();
    }

    if (auto res = UpdateScene(request); !res) {
      utils::LogManager::Error("更新图片失败：" + res.error().message());
    }
  }

  return {};
}

// [新增] 辅助函数：准备场景的原子请求
void DrmApp::PrepareSceneRequest(AtomicRequest &req) {
  const auto &mode = display_->mode();
  // 图层始终从(0,0)开始，并填满整个屏幕
  layer_->AddToRequest(req, display_->crtc_id(), 0, 0, mode.hdisplay,
                       mode.vdisplay);
}

// [新增] 辅助函数：提交原子请求并处理翻转后的事宜
ResultErr<void> DrmApp::CommitScene(AtomicRequest &req, bool page_flip) {
  PROPAGATE_ERROR_CTX(display_->Commit(req, page_flip));
  if (page_flip) {
    PROPAGATE_ERROR_CTX(display_->WaitForFlip());
    layer_->PostFlip();
  }
  return {};
}

}  // namespace drm
