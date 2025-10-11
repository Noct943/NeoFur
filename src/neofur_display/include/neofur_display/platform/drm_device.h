#pragma once

#include <xf86drm.h>
#include <xf86drmMode.h>

#include <gsl/span>
#include <memory>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "neofur_utils/types.h"

namespace neofur {
namespace renderer {
namespace platform {

/**
 * @class DrmDevice
 * @brief 管理DRM设备文件描述符和核心资源。
 */
class DrmDevice {
 public:
  ~DrmDevice();
  DrmDevice(const DrmDevice&) = delete;
  DrmDevice& operator=(const DrmDevice&) = delete;

  static ResultErr<std::shared_ptr<DrmDevice>> Create(std::string_view path);

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
using DrmDevPtr = std::shared_ptr<DrmDevice>;

/**
 * @class Property
 * @brief DRM属性的RAII封装。
 */
class Property {
 public:
  explicit Property(drmModePropertyPtr prop);

  uint32_t id() const { return prop_->prop_id; }
  const char* name() const { return prop_->name; }

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

  DrmDevPtr device() const { return dev_; }
  ResultErr<const Property *> GetProperty(std::string_view name) const;

  virtual uint32_t id() const = 0;

 protected:
  DrmObject(DrmDevPtr dev,
            std::unordered_map<std::string, std::unique_ptr<Property>> &&props);

  DrmDevPtr dev_;
  std::unordered_map<std::string, std::unique_ptr<Property>> props_;
};

// RAII封装 drmModeCrtc
class Crtc : public DrmObject {
 public:
  static ResultErr<std::unique_ptr<Crtc>> Create(DrmDevPtr dev, uint32_t crtc_id);
  ~Crtc() = default;

  uint32_t id() const override { return crtc_->crtc_id; }
  uint32_t buffer_id() const { return crtc_->buffer_id; }

 private:
  Crtc(DrmDevPtr dev, drmModeCrtcPtr crtc,
       std::unordered_map<std::string, std::unique_ptr<Property>> &&props);

  struct Deleter {
    void operator()(drmModeCrtcPtr ptr) const;
  };
  std::unique_ptr<drmModeCrtc, Deleter> crtc_;
};

// RAII封装 drmModeConnector
class Connector : public DrmObject {
 public:
  static ResultErr<std::unique_ptr<Connector>> Create(DrmDevPtr dev,
                                                      uint32_t conn_id);
  ~Connector() = default;

  uint32_t id() const override { return conn_->connector_id; }
  drmModeConnection state() const { return conn_->connection; }
  gsl::span<const drmModeModeInfo> modes() const;
  gsl::span<const uint32_t> encoder_ids() const;

 private:
  Connector(DrmDevPtr dev, drmModeConnectorPtr conn,
            std::unordered_map<std::string, std::unique_ptr<Property>> &&props);

  struct Deleter {
    void operator()(drmModeConnectorPtr ptr) const;
  };
  std::unique_ptr<drmModeConnector, Deleter> conn_;
};

// RAII封装 drmModePlane
class Plane : public DrmObject {
 public:
  static ResultErr<std::unique_ptr<Plane>> Create(DrmDevPtr dev,
                                                  uint32_t plane_id);
  ~Plane() = default;

  uint32_t id() const override { return plane_->plane_id; }
  uint32_t crtc_id() const { return plane_->crtc_id; }
  uint32_t fb_id() const { return plane_->fb_id; }

  uint32_t possible_crtcs() const { return plane_->possible_crtcs; }

 private:
  Plane(DrmDevPtr dev, drmModePlanePtr plane,
        std::unordered_map<std::string, std::unique_ptr<Property>> &&props);

  struct Deleter {
    void operator()(drmModePlanePtr ptr) const;
  };
  std::unique_ptr<drmModePlane, Deleter> plane_;
};


}  // namespace platform
}  // namespace renderer
}  // namespace neofur
