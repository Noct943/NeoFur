#include "neofur_display/atomic.h"

#include "neofur_utils/my_error.h"

namespace drm {

AtomicRequest::~AtomicRequest() = default;

ResultErr<void> AtomicRequest::SetProperty(const DrmObject& object,
                                           const std::string& prop_name,
                                           uint64_t value) {
  auto prop_result = object.GetProperty(prop_name);
  if (!prop_result) {
    return tl::make_unexpected(prop_result.error());
  }
  const uint32_t prop_id = (*prop_result)->id();
  const uint32_t obj_id = object.id();
  return AddProperty(obj_id, prop_id, value);
}

ResultErr<void> AtomicRequest::AddProperty(uint32_t obj_id,
                                           uint32_t prop_id,
                                           uint64_t value) {
  if (drmModeAtomicAddProperty(req_.get(), obj_id, prop_id, value) < 0) {
    return tl::make_unexpected(MAKE_ERROR_CTX(
        utils::ErrorCode::IOError, "Failed to add property to atomic request"));
  }
  return {};
}

ResultErr<void> AtomicRequest::Commit(DrmDevice& dev, uint32_t flags,
                                      void* user_data) {
  // 将 user_data 传递给 libdrm
  if (drmModeAtomicCommit(dev.fd(), req_.get(), flags, user_data) != 0) {
    return tl::make_unexpected(MAKE_ERROR_CTX(
        utils::ErrorCode::RuntimeError, "DRM atomic commit failed"));
  }
  return {};
}
}  // namespace drm
