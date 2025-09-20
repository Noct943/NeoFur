#ifndef NEOFUR_DISPLAY_NODE_H
#define NEOFUR_DISPLAY_NODE_H

#include <filesystem>
#include <memory>
#include <rclcpp/node.hpp>
#include <thread>

#include "neofur_interfaces/msg/display_msg.hpp"
#include "neofur_utils/my_error.h"
#include "neofur_display/app.h"

namespace neofur {

using utils::ResultErr;

class DisplayNode : public rclcpp::Node {
 public:
  using DisplayMsg = neofur_interfaces::msg::DisplayMsg;

  static ResultErr<std::shared_ptr<DisplayNode>> Create(
      const rclcpp::NodeOptions &options);

  ~DisplayNode() override;

  // --- 禁用拷贝和移动语义 ---
  DisplayNode(const DisplayNode &) = delete;
  DisplayNode &operator=(const DisplayNode &) = delete;
  DisplayNode(DisplayNode &&) = delete;
  DisplayNode &operator=(DisplayNode &&) = delete;

 private:
  /**
   * @brief 用于封装从ROS参数服务器加载的所有配置。
   */
  struct NodeParameters {
    std::filesystem::path image_base_path;
    std::string drm_device;
    std::string image_name;
    int init_brightness;
  };

  // --- 私有构造函数 ---
  explicit DisplayNode(const rclcpp::NodeOptions &options)
      : Node("neofur_display_node", options) {}

  // --- 私有辅助函数 (静态) ---
  // 这些静态函数作为 Create() 的帮手，将初始化逻辑分解。
  static ResultErr<NodeParameters> LoadParameters(DisplayNode *node);
  static ResultErr<void> InitDrmApp(DisplayNode *node,
                                    const NodeParameters &params);
  static void SetupSubscription(DisplayNode *node);
  static void StartDisplayThread(DisplayNode *node);

  // --- 回调与线程实现 (非静态) ---
  void UpdateCallback(const std::shared_ptr<DisplayMsg> msg);
  void DisplayThreadLoop();

  // --- 成员变量 ---
  std::unique_ptr<drm::DrmApp> app_;
  std::thread display_thread_;
  rclcpp::Subscription<DisplayMsg>::SharedPtr subscription_;
  std::filesystem::path image_base_path_;
};

}  // namespace node
#endif  // NEOFUR_DISPLAY_NODE_H
