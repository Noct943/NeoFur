#include "neofur_display/device.h"

#include <drm/drm_fourcc.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <xf86drm.h>
#include <xf86drmMode.h>

#include <string_view>
#include <utility>

#include "neofur_utils/log_manager.h"
#include "neofur_utils/utils.h"

struct dma_heap_allocation_data {
  __u64 len;
  __u32 fd;
  __u32 fd_flags;
  __u64 heap_flags;
};

#define DMA_HEAP_IOC_MAGIC 'H'
#define DMA_HEAP_IOCTL_ALLOC \
  _IOWR(DMA_HEAP_IOC_MAGIC, 0x0, struct dma_heap_allocation_data)

namespace drm {

// DrmDevice
DrmDevice::DrmDevice(int fd, drmModeResPtr res, drmModePlaneResPtr plane_res)
    : fd_(fd), res_(res), plane_res_(plane_res) {}

DrmDevice::~DrmDevice() {
  drmModeFreePlaneResources(plane_res_);
  drmModeFreeResources(res_);
  close(fd_);
}

ResultErr<DevPtr> DrmDevice::Create(std::string_view path) {
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
    DevPtr dev,
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

namespace {  // 匿名命名空间，隐藏实现细节
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

// Crtc
Crtc::Crtc(DevPtr dev, drmModeCrtcPtr crtc,
           std::unordered_map<std::string, std::unique_ptr<Property>>&& props)
    : DrmObject(std::move(dev), std::move(props)), crtc_(crtc) {}

void Crtc::Deleter::operator()(drmModeCrtcPtr ptr) const {
  if (ptr) drmModeFreeCrtc(ptr);
}

ResultErr<std::unique_ptr<Crtc>> Crtc::Create(DevPtr dev, uint32_t crtc_id) {
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
    DevPtr dev, drmModeConnectorPtr conn,
    std::unordered_map<std::string, std::unique_ptr<Property>>&& props)
    : DrmObject(std::move(dev), std::move(props)), conn_(conn) {}

void Connector::Deleter::operator()(drmModeConnectorPtr ptr) const {
  if (ptr) drmModeFreeConnector(ptr);
}

ResultErr<std::unique_ptr<Connector>> Connector::Create(DevPtr dev,
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
Plane::Plane(DevPtr dev, drmModePlanePtr plane,
             std::unordered_map<std::string, std::unique_ptr<Property>>&& props)
    : DrmObject(std::move(dev), std::move(props)), plane_(plane) {}

void Plane::Deleter::operator()(drmModePlanePtr ptr) const {
  if (ptr) drmModeFreePlane(ptr);
}

ResultErr<std::unique_ptr<Plane>> Plane::Create(DevPtr dev, uint32_t plane_id) {
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

// DmaBuffer
ResultErr<DmaBuffer> DmaBuffer::Create(uint32_t width, uint32_t height,
                                       uint32_t pitch) {
  constexpr const char* heap_path = "/dev/dma_heap/system";
  int heap_fd = open(heap_path, O_RDWR | O_CLOEXEC);
  if (heap_fd < 0) {
    return tl::make_unexpected(
        MAKE_ERROR_CTX(utils::ErrorCode::IOError,
                       "无法打开 dma-heap 设备: " + std::string(heap_path)));
  }
  utils::ScopeGuard heap_fd_guard([&]() { close(heap_fd); });

  // [核心修复] 根据对齐后的 pitch 计算总内存大小
  size_t len = pitch * height;

  dma_heap_allocation_data alloc_data = {
      .len = len,
      .fd_flags = O_RDWR | O_CLOEXEC,
  };

  if (ioctl(heap_fd, DMA_HEAP_IOCTL_ALLOC, &alloc_data) < 0) {
    return tl::make_unexpected(
        MAKE_ERROR_CTX(utils::ErrorCode::IOError, "dma-heap 分配失败"));
  }

  int buffer_fd = static_cast<int>(alloc_data.fd);

  void* mapped_ptr =
      mmap(nullptr, len, PROT_READ | PROT_WRITE, MAP_SHARED, buffer_fd, 0);
  if (mapped_ptr == MAP_FAILED) {
    close(buffer_fd);
    return tl::make_unexpected(
        MAKE_ERROR_CTX(utils::ErrorCode::IOError, "mmap dma-buf 失败"));
  }

  return DmaBuffer(buffer_fd, width, height, len, mapped_ptr);
}

DmaBuffer::DmaBuffer(int fd, uint32_t width, uint32_t height, size_t size,
                     void* mapped_ptr)
    : fd_(fd),
      width_(width),
      height_(height),
      size_(size),
      mapped_ptr_(mapped_ptr) {}

DmaBuffer::~DmaBuffer() { release(); }

void DmaBuffer::release() {
  if (mapped_ptr_) {
    munmap(mapped_ptr_, size_);
    mapped_ptr_ = nullptr;
  }
  if (fd_ >= 0) {
    close(fd_);
    fd_ = -1;
  }
}

DmaBuffer::DmaBuffer(DmaBuffer&& other) noexcept
    : fd_(other.fd_),
      width_(other.width_),
      height_(other.height_),
      size_(other.size_),
      mapped_ptr_(other.mapped_ptr_) {
  other.fd_ = -1;
  other.mapped_ptr_ = nullptr;
}

DmaBuffer& DmaBuffer::operator=(DmaBuffer&& other) noexcept {
  if (this != &other) {
    release();
    fd_ = other.fd_;
    width_ = other.width_;
    height_ = other.height_;
    size_ = other.size_;
    mapped_ptr_ = other.mapped_ptr_;
    other.fd_ = -1;
    other.mapped_ptr_ = nullptr;
  }
  return *this;
}

// --- FrameBuffer 实现 ---
ResultErr<FrameBuffer> FrameBuffer::Create(DevPtr dev, DmaBuffer&& buffer,
                                           uint32_t pitch) {
  uint32_t fb_id;
  uint32_t handles[4] = {0};
  uint32_t pitches[4] = {0};
  uint32_t offsets[4] = {0};

  // 将 dma-buf fd 导入到 DRM，获取一个 GEM handle
  if (drmPrimeFDToHandle(dev->fd(), buffer.fd(), &handles[0]) != 0) {
    return tl::make_unexpected(
        MAKE_ERROR_CTX(utils::ErrorCode::IOError, "DRM: 导入 prime fd 失败"));
  }

  // Rockchip 的 stride (pitch) 通常是 64 字节对齐
  pitches[0] = pitch;

  if (drmModeAddFB2(dev->fd(), buffer.width(), buffer.height(),
                    DRM_FORMAT_ARGB8888, handles, pitches, offsets, &fb_id,
                    0) != 0) {
    // 如果失败，需要释放 handle
    drm_gem_close gem_close = {.handle = handles[0]};
    ioctl(dev->fd(), DRM_IOCTL_GEM_CLOSE, &gem_close);
    return tl::make_unexpected(
        MAKE_ERROR_CTX(utils::ErrorCode::IOError, "DRM: drmModeAddFB2 失败"));
  }

  return FrameBuffer(std::move(buffer), dev, fb_id, pitches[0]);
}

FrameBuffer::FrameBuffer(DmaBuffer&& buffer, DevPtr dev, uint32_t fb_id,
                         uint32_t pitch)
    : buffer_(std::move(buffer)),
      dev_(std::move(dev)),
      fb_id_(fb_id),
      pitch_(pitch) {}

FrameBuffer::~FrameBuffer() { release(); }

void FrameBuffer::release() {
  if (fb_id_ != 0 && dev_) {
    drmModeRmFB(dev_->fd(), fb_id_);
  }
  fb_id_ = 0;
}

FrameBuffer::FrameBuffer(FrameBuffer&& other) noexcept
    : buffer_(std::move(other.buffer_)),
      dev_(std::move(other.dev_)),
      fb_id_(other.fb_id_),
      pitch_(other.pitch_) {
  other.fb_id_ = 0;
}

FrameBuffer& FrameBuffer::operator=(FrameBuffer&& other) noexcept {
  if (this != &other) {
    release();
    buffer_ = std::move(other.buffer_);
    dev_ = std::move(other.dev_);
    fb_id_ = other.fb_id_;
    pitch_ = other.pitch_;
    other.fb_id_ = 0;
  }
  return *this;
}

ResultErr<int> FrameBuffer::ExportDmaFd() const { return buffer_.fd(); }

gsl::span<std::byte> FrameBuffer::span() const {
  return {static_cast<std::byte*>(buffer_.data()), buffer_.size()};
}

gsl::span<uint32_t> FrameBuffer::row_span(uint32_t y) const {
  if (!buffer_.data() || y >= height()) {
    return {};
  }
  auto* ptr = reinterpret_cast<uint32_t*>(
      static_cast<std::byte*>(buffer_.data()) + y * pitch());
  return {ptr, width()};
}
}  // namespace drm
