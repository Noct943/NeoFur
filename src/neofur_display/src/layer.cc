#include "neofur_display/layer.h"

#include <drm/drm_fourcc.h>
#include <librga/rga.h>

#include <algorithm>
#include <cassert>
#include <cstring>
#include <fstream>
#include <librga/im2d.hpp>
#include <utility>
#include <vector>

#include "neofur_display/atomic.h"
#include "neofur_display/device.h"
#include "neofur_display/drm_types.h"
#include "neofur_utils/my_error.h"
#include "neofur_utils/utils.h"

namespace drm {

ResultErr<LayerPtr> Layer::Create(DevPtr dev, std::unique_ptr<Plane> plane,
                                  std::vector<FrameBuffer> &&buffers) {
  if (buffers.empty() || !plane) {
    return tl::make_unexpected(
        MAKE_ERROR_CTX(utils::ErrorCode::InvalidArguments,
                       "创建Layer失败：帧缓冲区列表为空或plane指针为空"));
  }

  LayerPtr layer(
      new Layer(std::move(dev), std::move(plane), std::move(buffers)));

  layer->rga_buffer_handles_.reserve(layer->buffers_.size());
  utils::ScopeGuard guard([&layer]() {
    for (auto handle : layer->rga_buffer_handles_) {
      if (handle > 0) releasebuffer_handle(handle);
    }
  });

  for (const auto &buffer : layer->buffers_) {
    auto dma_fd_res = buffer.ExportDmaFd();
    if (!dma_fd_res) return tl::make_unexpected(dma_fd_res.error());

    // 使用 buffer 的 pitch 来导入，确保 RGA 知道正确的行 stride
    auto handle = importbuffer_fd(*dma_fd_res, buffer.size());
    if (handle <= 0) {
      return tl::make_unexpected(MAKE_ERROR_CTX(utils::ErrorCode::RuntimeError,
                                                "RGA importbuffer_fd 失败"));
    }
    layer->rga_buffer_handles_.push_back(handle);
  }

  guard.Dismiss();
  return layer;
}

Layer::~Layer() {
  for (auto handle : rga_buffer_handles_) {
    if (handle > 0) releasebuffer_handle(handle);
  }
}

void Layer::DrawSolidColor(Color color) {
  // 按照约定，buffers_[0] 是后台缓冲区，可用于绘制
  auto &back_buffer = buffers_[0];

  for (uint32_t y = 0; y < back_buffer.height(); ++y) {
    auto row = back_buffer.row_span(y);
    std::fill(row.begin(), row.end(), static_cast<uint32_t>(color));
  }
}

ResultErr<void> Layer::DrawRgba(const ImageData &image) {
  if (buffers_.empty() || rga_buffer_handles_.empty()) {
    return tl::make_unexpected(
        MAKE_ERROR_CTX(utils::ErrorCode::RuntimeError,
                       "Layer RGA handles are not initialized."));
  }

  // --- 步骤 1: 将文件安全地读入 CPU 的“中介缓冲区” ---
  const size_t unaligned_row_size = image.width * 4;
  const size_t total_file_size = unaligned_row_size * image.height;
  std::vector<char> staging_buffer(total_file_size);

  std::ifstream file(image.path, std::ios::binary);
  if (!file.read(staging_buffer.data(), total_file_size)) {
    return tl::make_unexpected(MAKE_ERROR_CTX(
        utils::ErrorCode::IOError, "读取图像文件失败: " + image.path));
  }

  // --- 步骤 2: 将数据从“中介缓冲区”逐行拷贝到硬件（DMA）缓冲区 ---
  // 这样做可以确保内存布局完全符合硬件的 pitch 对齐要求
  auto &back_buffer = buffers_[0];  // 目标是屏幕的后台缓冲区
  char *dma_ptr = reinterpret_cast<char *>(back_buffer.span().data());
  uint32_t dma_pitch = back_buffer.pitch();

  for (uint32_t i = 0; i < image.height; ++i) {
    memcpy(dma_ptr + i * dma_pitch,  // 目标：DMA缓冲区的第 i 行起始地址
           staging_buffer.data() +
               i * unaligned_row_size,  // 源：CPU缓冲区的第 i 行起始地址
           unaligned_row_size);  // 拷贝长度：一行的实际像素数据
  }

  // --- 步骤 3: 准备 RGA 操作，源和目标都是同一个显示缓冲区 ---
  // 我们要做的缩放，实际上是 RGA 从同一个 buffer 的一个区域（源）
  // 读取数据，然后写回到同一个 buffer 的另一个区域（目标）。
  auto buffer_handle = rga_buffer_handles_[0];
  // 准备一个 rga_buffer_t 来描述整个后台缓冲区
  rga_buffer_t buf = wrapbuffer_handle(
      buffer_handle, back_buffer.width(), back_buffer.height(),
      RK_FORMAT_RGBA_8888, back_buffer.pitch(), back_buffer.height());

  // --- 步骤 4: 执行硬件缩放 ---
  // RGA 从同一个 buffer 的一个子区域（我们刚拷贝进去的图像）读取数据
  rga_buffer_t src_region = buf;
  src_region.width = image.width;
  src_region.height = image.height;
  // 重要：源区域的 pitch 必须和整个 buffer 的 pitch 保持一致
  src_region.wstride = back_buffer.pitch();
  src_region.hstride = back_buffer.height();

  // [核心修复] 使用所有版本都兼容的 imresize 函数
  IM_STATUS resize_status = imresize(src_region, buf);
  if (resize_status != IM_STATUS_SUCCESS) {
    return tl::make_unexpected(MAKE_ERROR_CTX(
        utils::ErrorCode::RuntimeError,
        "RGA imresize 失败: " + std::string(imStrError(resize_status))));
  }
  return {};
}

ResultErr<void> Layer::SyncBufferOnMove() {
  if (buffers_.size() < 2) {
    return {};
  }

  rga_buffer_handle_t dst_handle = rga_buffer_handles_[0];
  rga_buffer_handle_t src_handle = rga_buffer_handles_[1];

  const auto &back_buffer = buffers_[0];
  const auto &front_buffer = buffers_[1];

  const int bytes_per_pixel = 4;
  int dst_pixel_stride = back_buffer.pitch() / bytes_per_pixel;
  int src_pixel_stride = front_buffer.pitch() / bytes_per_pixel;

  rga_buffer_t dst = wrapbuffer_handle(
      dst_handle, back_buffer.width(), back_buffer.height(),
      RK_FORMAT_RGBA_8888, dst_pixel_stride, back_buffer.height());
  rga_buffer_t src = wrapbuffer_handle(
      src_handle, front_buffer.width(), front_buffer.height(),
      RK_FORMAT_RGBA_8888, src_pixel_stride, front_buffer.height());

  IM_STATUS status = imcopy(src, dst);
  if (status != IM_STATUS_SUCCESS) {
    return tl::make_unexpected(
        MAKE_ERROR_CTX(utils::ErrorCode::RuntimeError,
                       "RGA imcopy for SyncBufferOnMove 失败: " +
                           std::string(imStrError(status))));
  }

  return {};
}

void Layer::AddToRequest(AtomicRequest &req, uint32_t crtc_id, uint32_t crtc_x,
                         uint32_t crtc_y, uint32_t crtc_w, uint32_t crtc_h) {
  // 使用后台缓冲区 (buffers_[0]) 的内容准备提交
  const auto &buffer_to_commit = buffers_[0];
  req.SetProperty(*plane_, "FB_ID", buffer_to_commit.id());
  req.SetProperty(*plane_, "CRTC_ID", crtc_id);
  // 源坐标（SRC_*）是16.16定点格式，因此需要左移16位
  req.SetProperty(*plane_, "SRC_X", 0);
  req.SetProperty(*plane_, "SRC_Y", 0);
  req.SetProperty(*plane_, "SRC_W", buffer_to_commit.width() << 16);
  req.SetProperty(*plane_, "SRC_H", buffer_to_commit.height() << 16);
  // CRTC坐标（CRTC_*）是像素单位
  req.SetProperty(*plane_, "CRTC_X", crtc_x);
  req.SetProperty(*plane_, "CRTC_Y", crtc_y);
  req.SetProperty(*plane_, "CRTC_W", crtc_w);
  req.SetProperty(*plane_, "CRTC_H", crtc_h);
}

void Layer::PostFlip() {
  // 如果我们有多个缓冲区（即双缓冲或三缓冲），
  // 就旋转缓冲区列表。旧的后台缓冲区将成为新的前台缓冲区。
  if (buffers_.size() > 1) {
    std::rotate(buffers_.begin(), buffers_.begin() + 1, buffers_.end());
    std::rotate(rga_buffer_handles_.begin(), rga_buffer_handles_.begin() + 1,
                rga_buffer_handles_.end());
  }
}
}  // namespace drm
