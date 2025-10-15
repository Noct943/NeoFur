#ifndef DRM_LAYER_H
#define DRM_LAYER_H

#include <neofur_utils/my_error.h>
#include <memory>
#include <string_view>

#include <librga/im2d.h>

#include "neofur_display/atomic.h"
#include "neofur_display/device.h"
#include "neofur_display/drm_types.h"

namespace drm {

/**
 * @class Layer
 * @brief 封装了一个独立的硬件显示图层(Plane)及其缓冲资源。
 *
 * Layer是渲染的基本单元，它唯一地拥有一个硬件Plane和管理其对应的
 * 一个或多个帧缓冲(FrameBuffer)。它负责自身的绘制和原子更新逻辑。
 */
class Layer {
 public:
  // 创建Layer时，将获得Plane的所有权
  static ResultErr<LayerPtr> Create(DevPtr dev, std::unique_ptr<Plane> plane,
                                    std::vector<FrameBuffer>&& buffers);
  ~Layer();

  // 在后台缓冲区填充纯色
  void DrawSolidColor(Color color);

  // 此图层的显示状态添加到DRM原子请求中
  void AddToRequest(AtomicRequest& req, uint32_t crtc_id, uint32_t crtc_x,
                    uint32_t crtc_y, uint32_t crtc_w, uint32_t crtc_h);

  ResultErr<void> DrawRgba(const ImageData &image);

  /**
   * @brief 只移动图片时，将前台缓冲区内容拷贝到后台
   *
   * Description.
   *
   */
  ResultErr<void> SyncBufferOnMove();

  /**
   * @brief 在一次成功的页面翻转后，更新缓冲区角色。
   *
   * 此函数应该在DRM原子提交成功并完成显示后被调用。
   * 它通过循环移动缓冲区列表，将旧的后台缓冲区变成新的前台缓冲区。
   */
  void PostFlip();

  // 释放并返回Plane的所有权
  std::unique_ptr<Plane> release_plane() { return std::move(plane_); }

  //------------------- 获取函数 -------------------//
  const Plane& plane() const { return *plane_; }
  auto width() const { return buffers_[0].width(); }
  auto height() const { return buffers_[0].height(); }

 private:
  Layer(DevPtr dev, std::unique_ptr<Plane> plane,
        std::vector<FrameBuffer>&& buffers)
      : dev_(std::move(dev)),
        plane_(std::move(plane)),
        buffers_(std::move(buffers)) {}

  DevPtr dev_;
  std::unique_ptr<Plane> plane_;  // 通过 unique_ptr 拥有 Plane
  // 帧缓冲区的集合。约定：[0]为后台，[back]为前台
  std::vector<FrameBuffer> buffers_;
  // 存储与 FrameBuffer 对应的 RGA 缓冲区句柄
  std::vector<rga_buffer_handle_t> rga_buffer_handles_;
};

}  // namespace drm

#endif  // DRM_LAYER_H
