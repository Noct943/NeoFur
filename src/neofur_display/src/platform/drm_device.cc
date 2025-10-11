#include "neofur_display/platform/drm_device.h"

#include "neofur_utils/utils.h"

namespace neofur {
namespace renderer {
namespace platform {
namespace {
// 本地辅助函数，用于获取DRM对象的属性列表
ResultErr<std::unordered_map<std::string, std::unique_ptr<Property>>>
FetchProperties(int fd, uint32_t object_id, uint32_t object_type) {
  drmModeObjectPropertiesPtr props_p =
      drmModeObjectGetProperties(fd, object_id, object_type);
  if (!props_p) {
    return tl::make_unexpected(MAKE_ERROR_CTX(
        utils::ErrorCode::IOError, "DRM: get object properties failed"));
  }
  utils::ScopeGuard props_guard(
      [&]() { drmModeFreeObjectProperties(props_p); });

  std::unordered_map<std::string, std::unique_ptr<Property>> props;
  props.reserve(props_p->count_props);
  for (uint32_t i = 0; i < props_p->count_props; ++i) {
    drmModePropertyPtr prop_p = drmModeGetProperty(fd, props_p->props[i]);
    if (!prop_p) {
      return tl::make_unexpected(MAKE_ERROR_CTX(utils::ErrorCode::IOError,
                                                "DRM: get property failed"));
    }
    props.emplace(prop_p->name, std::make_unique<Property>(prop_p));
  }
  return props;
}
}  // namespace

// DrmDevice
DrmDevice::DrmDevice(int fd, drmModeResPtr res, drmModePlaneResPtr plane_res)
    : fd_(fd), res_(res), plane_res_(plane_res) {}

DrmDevice::~DrmDevice() {
  drmModeFreePlaneResources(plane_res_);
  drmModeFreeResources(res_);
  close(fd_);
}

ResultErr<DrmDevPtr> DrmDevice::Create(std::string_view path) {
  int fd = open(path.data(), O_RDWR | O_CLOEXEC);
  if (fd < 0) {
    return tl::make_unexpected(
        MAKE_ERROR_CTX(utils::ErrorCode::IOError, "DRM: device open failed"));
  }
  utils::ScopeGuard fd_guard([&]() { close(fd); });

  if (drmSetClientCap(fd, DRM_CLIENT_CAP_ATOMIC, 1) != 0 ||
      drmSetClientCap(fd, DRM_CLIENT_CAP_UNIVERSAL_PLANES, 1) != 0) {
    return tl::make_unexpected(MAKE_ERROR_CTX(
        utils::ErrorCode::IOError, "DRM: failed to set client caps"));
  }

  drmModeResPtr res = drmModeGetResources(fd);
  if (!res) {
    return tl::make_unexpected(
        MAKE_ERROR_CTX(utils::ErrorCode::IOError, "DRM: get resources failed"));
  }
  utils::ScopeGuard res_guard([&]() { drmModeFreeResources(res); });

  drmModePlaneResPtr plane_res = drmModeGetPlaneResources(fd);
  if (!plane_res) {
    return tl::make_unexpected(MAKE_ERROR_CTX(
        utils::ErrorCode::IOError, "DRM: get plane resources failed"));
  }

  fd_guard.Dismiss();
  res_guard.Dismiss();

  // 使用自定义删除器，确保 shared_ptr 能正确调用 private 的构造函数
  return std::shared_ptr<DrmDevice>(new DrmDevice(fd, res, plane_res));
}

std::vector<uint32_t> DrmDevice::GetConnectorIds() const {
  return {res_->connectors, res_->connectors + res_->count_connectors};
}
std::vector<uint32_t> DrmDevice::GetCrtcIds() const {
  return {res_->crtcs, res_->crtcs + res_->count_crtcs};
}
std::vector<uint32_t> DrmDevice::GetPlaneIds() const {
  return {plane_res_->planes, plane_res_->planes + plane_res_->count_planes};
}

// Property
Property::Property(drmModePropertyPtr prop) : prop_(prop) {}
void Property::Deleter::operator()(drmModePropertyPtr prop) const {
  if (prop) drmModeFreeProperty(prop);
}

// DrmObject
DrmObject::DrmObject(
    DrmDevPtr dev,
    std::unordered_map<std::string, std::unique_ptr<Property>>&& props)
    : dev_(std::move(dev)), props_(std::move(props)) {}

ResultErr<const Property*> DrmObject::GetProperty(std::string_view name) const {
  auto it = props_.find(std::string(name));
  if (it == props_.end()) {
    return tl::make_unexpected(
        MAKE_ERROR_CTX(utils::ErrorCode::NotFound, "DRM: property not found"));
  }
  return it->second.get();
}

// Crtc
Crtc::Crtc(DrmDevPtr dev, drmModeCrtcPtr crtc,
           std::unordered_map<std::string, std::unique_ptr<Property>>&& props)
    : DrmObject(std::move(dev), std::move(props)), crtc_(crtc) {}

void Crtc::Deleter::operator()(drmModeCrtcPtr ptr) const {
  if (ptr) drmModeFreeCrtc(ptr);
}

ResultErr<std::unique_ptr<Crtc>> Crtc::Create(DrmDevPtr dev, uint32_t crtc_id) {
  drmModeCrtcPtr crtc_p = drmModeGetCrtc(dev->fd(), crtc_id);
  if (!crtc_p) {
    return tl::make_unexpected(
        MAKE_ERROR_CTX(utils::ErrorCode::IOError, "DRM: get CRTC failed"));
  }
  utils::ScopeGuard crtc_guard([&]() { drmModeFreeCrtc(crtc_p); });

  auto props_result = FetchProperties(dev->fd(), crtc_id, DRM_MODE_OBJECT_CRTC);
  if (!props_result) return tl::make_unexpected(props_result.error());

  crtc_guard.Dismiss();
  return std::unique_ptr<Crtc>(new Crtc(dev, crtc_p, std::move(*props_result)));
}

// Connector
Connector::Connector(
    DrmDevPtr dev, drmModeConnectorPtr conn,
    std::unordered_map<std::string, std::unique_ptr<Property>>&& props)
    : DrmObject(std::move(dev), std::move(props)), conn_(conn) {}

void Connector::Deleter::operator()(drmModeConnectorPtr ptr) const {
  if (ptr) drmModeFreeConnector(ptr);
}

ResultErr<std::unique_ptr<Connector>> Connector::Create(DrmDevPtr dev,
                                                        uint32_t conn_id) {
  drmModeConnectorPtr conn_p = drmModeGetConnector(dev->fd(), conn_id);
  if (!conn_p) {
    return tl::make_unexpected(
        MAKE_ERROR_CTX(utils::ErrorCode::IOError, "DRM: get connector failed"));
  }
  utils::ScopeGuard conn_guard([&]() { drmModeFreeConnector(conn_p); });

  auto props_result =
      FetchProperties(dev->fd(), conn_id, DRM_MODE_OBJECT_CONNECTOR);
  if (!props_result) return tl::make_unexpected(props_result.error());

  conn_guard.Dismiss();
  return std::unique_ptr<Connector>(
      new Connector(dev, conn_p, std::move(*props_result)));
}

gsl::span<const drmModeModeInfo> Connector::modes() const {
  return {conn_->modes, static_cast<size_t>(conn_->count_modes)};
}
gsl::span<const uint32_t> Connector::encoder_ids() const {
  return {conn_->encoders, static_cast<size_t>(conn_->count_encoders)};
}

// Plane
Plane::Plane(DrmDevPtr dev, drmModePlanePtr plane,
             std::unordered_map<std::string, std::unique_ptr<Property>>&& props)
    : DrmObject(std::move(dev), std::move(props)), plane_(plane) {}

void Plane::Deleter::operator()(drmModePlanePtr ptr) const {
  if (ptr) drmModeFreePlane(ptr);
}

ResultErr<std::unique_ptr<Plane>> Plane::Create(DrmDevPtr dev,
                                                uint32_t plane_id) {
  drmModePlanePtr plane_p = drmModeGetPlane(dev->fd(), plane_id);
  if (!plane_p) {
    return tl::make_unexpected(
        MAKE_ERROR_CTX(utils::ErrorCode::IOError, "DRM: get plane failed"));
  }
  utils::ScopeGuard plane_guard([&]() { drmModeFreePlane(plane_p); });

  auto props_result =
      FetchProperties(dev->fd(), plane_id, DRM_MODE_OBJECT_PLANE);
  if (!props_result) return tl::make_unexpected(props_result.error());

  plane_guard.Dismiss();
  return std::unique_ptr<Plane>(
      new Plane(dev, plane_p, std::move(*props_result)));
}

}  // namespace platform
}  // namespace renderer
}  // namespace neofur
