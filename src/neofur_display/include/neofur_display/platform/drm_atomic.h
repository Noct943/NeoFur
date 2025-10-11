#pragma once

#include <xf86drm.h>
#include <xf86drmMode.h>

#include "neofur_utils/types.h"

namespace neofur {
namespace renderer {
namespace platform{

class DrmObject;  // Forward declaration

/**
 * @class DrmAtomicRequest
 * @brief 封装一个DRM原子模式设置请求。
 */
class DrmAtomicRequest {
 public:
  DrmAtomicRequest();
  ~DrmAtomicRequest();

  DrmAtomicRequest(const DrmAtomicRequest&) = delete;
  DrmAtomicRequest& operator=(const DrmAtomicRequest&) = delete;

  /**
   * @brief 为DRM对象添加一个属性设置。
   * @return 成功返回void，失败返回Error对象。
   */
  ResultErr<void> AddProperty(uint32_t obj_id, uint32_t prop_id,
                              uint64_t value);
  /**
   * @brief (重载) 通过属性名给DRM对象添加一个属性设置。
   * @return 成功返回void，失败返回Error对象。
   */
  ResultErr<void> AddProperty(const DrmObject& obj,
                              const std::string& prop_name, uint64_t value);

  /**
   * @brief 提交这个原子请求。
   * @param drm_fd DRM设备的文件描述符。
   * @param flags 提交标志 (例如，用于页面翻转的 event 标志)。
   * @return 成功返回void，失败返回Error。
   */
  ResultErr<void> Commit(int drm_fd, uint32_t flags,void* user_data);

    drmModeAtomicReqPtr get() const { return req_; }

 private:
  drmModeAtomicReqPtr req_;
};
} //namespace platform
}  // namespace renderer
}  // namespace neofur
