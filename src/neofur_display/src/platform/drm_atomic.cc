#include "neofur_display/platform/drm_atomic.h"

#include <string.h>
#include "neofur_display/platform/drm_device.h"

namespace neofur {
namespace renderer {
namespace platform{

DrmAtomicRequest::DrmAtomicRequest() : req_(drmModeAtomicAlloc()) {}

DrmAtomicRequest::~DrmAtomicRequest() {
  if (req_) {
    drmModeAtomicFree(req_);
  }
}

// [修改] 底层AddProperty实现，返回ResultErr
ResultErr<void> DrmAtomicRequest::AddProperty(uint32_t obj_id,
                                              uint32_t prop_id,
                                              uint64_t value) {
  if (drmModeAtomicAddProperty(req_, obj_id, prop_id, value) < 0) {
    return tl::make_unexpected(MAKE_ERROR_CTX(
        utils::ErrorCode::IOError, "向原子请求添加属性失败"));
  }
  return {};
}

// [修改] 上层AddProperty实现，返回ResultErr
ResultErr<void> DrmAtomicRequest::AddProperty(const DrmObject& obj,
                                              const std::string& prop_name,
                                              uint64_t value) {
   auto prop_result = obj.GetProperty(prop_name);
  if (!prop_result) {
    return tl::make_unexpected(prop_result.error());
  }
  const uint32_t prop_id = (*prop_result)->id();
  const uint32_t obj_id = obj.id();
  return AddProperty(obj_id, prop_id, value);

}

ResultErr<void> DrmAtomicRequest::Commit(int drm_fd, uint32_t flags,void* user_data) {
  int ret = drmModeAtomicCommit(drm_fd, req_, flags, user_data);
  if (ret != 0) {
    return tl::make_unexpected(MAKE_ERROR_CTX(
        utils::ErrorCode::IOError,
        "DRM原子提交失败: " + std::string(strerror(-ret))));
  }
  return {};
}
} //namespace platform
}  // namespace renderer
}  // namespace neofur
