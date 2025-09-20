#ifndef ATOMIC_H_H
#define ATOMIC_H_H

#include <xf86drmMode.h>

#include <memory>

#include "neofur_display/device.h"
#include "neofur_display/drm_types.h"

namespace drm {

// AtomicRequest 类用于构建和提交一个原子性的DRM状态更新
class AtomicRequest {
 public:
  AtomicRequest() : req_(drmModeAtomicAlloc()) {}
  ~AtomicRequest();

  AtomicRequest(const AtomicRequest&) = delete;
  AtomicRequest& operator=(const AtomicRequest&) = delete;
  AtomicRequest(AtomicRequest&&) = default;
  AtomicRequest& operator=(AtomicRequest&&) = default;

  ResultErr<void> SetProperty(const DrmObject& object,
                              const std::string& prop_name, uint64_t value);

  /**
   * @brief 提交原子请求
   * @param dev DrmDevice的引用
   * @param flags 提交标志 (e.g., DRM_MODE_PAGE_FLIP_EVENT)
   * @param user_data 传递给DRM事件上下文的用户数据指针
   */
  ResultErr<void> Commit(DrmDevice& dev, uint32_t flags, void* user_data);

 private:
  struct Deleter {
    void operator()(drmModeAtomicReqPtr ptr) {
      if (ptr) drmModeAtomicFree(ptr);
    }
  };
  std::unique_ptr<drmModeAtomicReq, Deleter> req_;

  ResultErr<void> AddProperty(uint32_t obj_id, uint32_t prop_id,
                              uint64_t value);
};
}  // namespace drm
#endif  // ATOMIC_H_H
