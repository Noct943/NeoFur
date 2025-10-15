#ifndef DRM_DEVICE_H_
#define DRM_DEVICE_H_

#include <xf86drm.h>
#include <xf86drmMode.h>

#include <cstddef>
#include <cstdint>
#include <gsl/gsl>
#include <memory>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "neofur_display/drm_types.h"

namespace drm {

// 管理DRM设备文件描述符和核心资源
class DrmDevice {
 public:
  static ResultErr<DevPtr> Create(std::string_view path);
  ~DrmDevice();

  DrmDevice(const DrmDevice &) = delete;
  DrmDevice &operator=(const DrmDevice &) = delete;

  int fd() const { return fd_; }

  std::vector<uint32_t> GetConnectorIds() const;
  std::vector<uint32_t> GetCrtcIds() const;
  std::vector<uint32_t> GetPlaneIds() const;

 private:
  DrmDevice(int fd, drmModeResPtr res, drmModePlaneResPtr plane_res);

  int fd_;
  drmModeResPtr res_;
  drmModePlaneResPtr plane_res_;
};

// RAII封装 drmModePropertyRes
class Property {
 public:
  explicit Property(drmModePropertyPtr prop);
  ~Property() = default;

  Property(const Property &) = delete;
  Property &operator=(const Property &) = delete;
  Property(Property &&) = default;
  Property &operator=(Property &&) = default;

  uint32_t id() const { return prop_->prop_id; }
  std::string_view name() const { return prop_->name; }
  bool is_immutable() const { return prop_->flags & DRM_MODE_PROP_IMMUTABLE; }

 private:
  struct Deleter {
    void operator()(drmModePropertyPtr prop) const;
  };
  std::unique_ptr<drmModePropertyRes, Deleter> prop_;
};

// DRM对象 (Crtc, Connector, Plane) 的基类
class DrmObject {
 public:
  virtual ~DrmObject() = default;

  DevPtr device() const { return dev_; }
  ResultErr<const Property *> GetProperty(std::string_view name) const;

  virtual uint32_t id() const = 0;

 protected:
  DrmObject(DevPtr dev,
            std::unordered_map<std::string, std::unique_ptr<Property>> &&props);

  DevPtr dev_;
  std::unordered_map<std::string, std::unique_ptr<Property>> props_;
};

// RAII封装 drmModeCrtc
class Crtc : public DrmObject {
 public:
  static ResultErr<std::unique_ptr<Crtc>> Create(DevPtr dev, uint32_t crtc_id);
  ~Crtc() = default;

  uint32_t id() const override { return crtc_->crtc_id; }
  uint32_t buffer_id() const { return crtc_->buffer_id; }

 private:
  Crtc(DevPtr dev, drmModeCrtcPtr crtc,
       std::unordered_map<std::string, std::unique_ptr<Property>> &&props);

  struct Deleter {
    void operator()(drmModeCrtcPtr ptr) const;
  };
  std::unique_ptr<drmModeCrtc, Deleter> crtc_;
};

// RAII封装 drmModeConnector
class Connector : public DrmObject {
 public:
  static ResultErr<std::unique_ptr<Connector>> Create(DevPtr dev,
                                                      uint32_t conn_id);
  ~Connector() = default;

  uint32_t id() const override { return conn_->connector_id; }
  drmModeConnection state() const { return conn_->connection; }
  gsl::span<const drmModeModeInfo> modes() const;
  gsl::span<const uint32_t> encoder_ids() const;

 private:
  Connector(DevPtr dev, drmModeConnectorPtr conn,
            std::unordered_map<std::string, std::unique_ptr<Property>> &&props);

  struct Deleter {
    void operator()(drmModeConnectorPtr ptr) const;
  };
  std::unique_ptr<drmModeConnector, Deleter> conn_;
};

// RAII封装 drmModePlane
class Plane : public DrmObject {
 public:
  static ResultErr<std::unique_ptr<Plane>> Create(DevPtr dev,
                                                  uint32_t plane_id);
  ~Plane() = default;

  uint32_t id() const override { return plane_->plane_id; }
  uint32_t crtc_id() const { return plane_->crtc_id; }
  uint32_t fb_id() const { return plane_->fb_id; }

  // --- 恢复的接口 ---
  uint32_t possible_crtcs() const { return plane_->possible_crtcs; }

 private:
  Plane(DevPtr dev, drmModePlanePtr plane,
        std::unordered_map<std::string, std::unique_ptr<Property>> &&props);

  struct Deleter {
    void operator()(drmModePlanePtr ptr) const;
  };
  std::unique_ptr<drmModePlane, Deleter> plane_;
};

// 管理 dma buffer 的内存和生命周期
class DmaBuffer {
 public:
  static ResultErr<DmaBuffer> Create(uint32_t width, uint32_t height,
                                     uint32_t pitch);
  ~DmaBuffer();

  DmaBuffer(const DmaBuffer &) = delete;
  DmaBuffer &operator=(const DmaBuffer &) = delete;
  DmaBuffer(DmaBuffer &&other) noexcept;
  DmaBuffer &operator=(DmaBuffer &&other) noexcept;

  int fd() const { return fd_; }
  uint32_t width() const { return width_; }
  uint32_t height() const { return height_; }
  size_t size() const { return size_; }
  void *data() const { return mapped_ptr_; }

 private:
  DmaBuffer(int fd, uint32_t width, uint32_t height, size_t size,
            void *mapped_ptr);
  void release();

  int fd_ = -1;
  uint32_t width_ = 0;
  uint32_t height_ = 0;
  size_t size_ = 0;
  void *mapped_ptr_ = nullptr;
};

// 管理 Framebuffer ID 和其关联的 DmaBuffer
class FrameBuffer {
 public:
  static ResultErr<FrameBuffer> Create(DevPtr dev, DmaBuffer &&buffer,
                                       uint32_t pitch);
  ~FrameBuffer();

  FrameBuffer(const FrameBuffer &) = delete;
  FrameBuffer &operator=(const FrameBuffer &) = delete;
  FrameBuffer(FrameBuffer &&other) noexcept;
  FrameBuffer &operator=(FrameBuffer &&other) noexcept;

  ResultErr<int> ExportDmaFd() const;

  uint32_t id() const { return fb_id_; }
  uint32_t width() const { return buffer_.width(); }
  uint32_t height() const { return buffer_.height(); }
  uint32_t pitch() const { return pitch_; }
  size_t size() const { return buffer_.size(); }
  gsl::span<std::byte> span() const;
  gsl::span<uint32_t> row_span(uint32_t y) const;

 private:
  FrameBuffer(DmaBuffer &&buffer, DevPtr dev, uint32_t fb_id, uint32_t pitch);
  void release();

  DmaBuffer buffer_;
  DevPtr dev_;
  uint32_t fb_id_ = 0;
  uint32_t pitch_ = 0;
};

}  // namespace drm

#endif  // DRM_DEVICE_H_
