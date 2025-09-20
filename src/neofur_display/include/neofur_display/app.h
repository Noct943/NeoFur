#ifndef NEOFUR_DISPLAY_APP_H_
#define NEOFUR_DISPLAY_APP_H_

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <string_view>
#include <thread>
#include <unordered_map>
#include <vector>

#include "neofur_display/atomic.h"
#include "neofur_display/display.h"
#include "neofur_display/drm_types.h"  // LayerType 在此文件中定义
#include "neofur_display/layer.h"
#include "neofur_utils/expected.hpp"
#include "neofur_utils/my_error.h"

namespace drm {

class DrmApp {
 public:
  ~DrmApp();

  /**
   * @brief 创建 DrmApp 实例.
   * @param drm_path DRM设备路径 (例如 "/dev/dri/card0").
   * @param image_path 初始要显示的前景图片路径.
   * @return 成功时返回 DrmApp 实例，否则返回错误.
   */
  static ResultErr<std::unique_ptr<DrmApp>> Create(std::string_view drm_path,
                                                   std::string_view image_path);

  /**
   * @brief 启动应用的主循环.
   * 此函数将阻塞，直到 Stop() 被调用.
   * @return 成功时返回 void, 否则返回错误.
   */
  ResultErr<void> Run();

  /**
   * @brief 异步停止应用的主循环.
   */
  void Stop();

  /**
   * @brief 异步请求更新前景图像.
   *
   * 这是一个非阻塞函数，它会将图像路径添加到一个队列中，
   * 由Run()循环在显示线程中处理.
   * @param image_path 新的前景图片路径.
   */
  void RequestUpdate(std::optional<std::string> image_path,
                     std::optional<uint8_t> brightness);

 private:
  DrmApp(DisplayPtr display,LayerPtr layers);

  /**
   * @brief 一个完整的显示更新请求
   *
   */
  struct ImageRequest {
    std::optional<ImageData> data;
    std::optional<uint8_t> brightness;
  };

  /**
   * @brief 使用已准备好的图像数据更新场景。
   * @param image 预加载的图像数据。
   * @return 成功时返回 void, 否则返回错误。
   */
  ResultErr<void> UpdateScene(const ImageRequest &image);

  void PrepareSceneRequest(AtomicRequest &req);

  ResultErr<void> CommitScene(AtomicRequest &req, bool page_flip);

  DisplayPtr display_;
  LayerPtr layer_;
  std::atomic<bool> running_;

  // 图像加载队列 (主线程 -> 显示线程)
  struct ImageQueue {
    static constexpr size_t kCapacity = 5;
    std::queue<ImageRequest> images;
    std::mutex mutex;
    std::condition_variable cv;
  } image_queue_;
};

}  // namespace drm

#endif  // NEOFUR_DISPLAY_APP_H_
