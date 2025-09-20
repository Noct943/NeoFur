#include "neofur_display/node.h"

#include <filesystem>
#include <memory>
#include <optional>
#include <rclcpp/logging.hpp>
#include <rclcpp/utilities.hpp>
#include <streambuf>

#include "neofur_display/display.h"
#include "neofur_display/drm_types.h"
#include "neofur_utils/log_manager.h"
#include "neofur_utils/my_error.h"

namespace neofur {

ResultErr<std::shared_ptr<DisplayNode>> DisplayNode::Create(
    const rclcpp::NodeOptions& options) {
  auto node = std::shared_ptr<DisplayNode>(new DisplayNode(options));

  auto params_res = LoadParameters(node.get());
  if (!params_res) return tl::make_unexpected(params_res.error());
  const auto params = *params_res;

  node->image_base_path_ = params.image_base_path;

  if (auto res = InitDrmApp(node.get(), params); !res)
    return tl::make_unexpected(res.error());

  SetupSubscription(node.get());

  StartDisplayThread(node.get());

  // 设置统一的全局日志记录器
  utils::LogManager::SetLogger(
      [logger = node->get_logger()](utils::LogLevel level,
                                    std::string_view msg) {
        // 将 C++ string_view 转换为 C-style string for RCLCPP macros
        std::string msg_str(msg);
        switch (level) {
          case utils::LogLevel::Info:
            RCLCPP_INFO(logger, "%s", msg_str.c_str());
            break;
          case utils::LogLevel::Warning:
            RCLCPP_WARN(logger, "%s", msg_str.c_str());
            break;
          case utils::LogLevel::Error:
            RCLCPP_ERROR(logger, "%s", msg_str.c_str());
            break;
        }
      });

  RCLCPP_INFO(node->get_logger(), "节点初始化成功。");
  return node;
}

DisplayNode::~DisplayNode() {
  RCLCPP_INFO(this->get_logger(), "节点析构，开始关停显示线程...");
  if (app_) {
    app_->Stop();
  }
  if (display_thread_.joinable()) {
    display_thread_.join();
    RCLCPP_INFO(this->get_logger(), "显示线程已成功汇合 (joined)。");
  }
}

//------------------- 私有静态辅助函数 -------------------//
ResultErr<DisplayNode::NodeParameters> DisplayNode::LoadParameters(
    DisplayNode* node) {
  NodeParameters params;

  params.image_base_path = node->declare_parameter("image_base_path", "");
  params.drm_device = node->declare_parameter("drm_device", "/dev/dri/card0");
  params.image_name = node->declare_parameter("image_name", "");
  params.init_brightness = node->declare_parameter("init_brightness", 50);

  if (params.image_base_path.empty() || params.image_name.empty()) {
    return tl::make_unexpected(
        MAKE_ERROR_CTX(utils::ErrorCode::InvalidArguments, "节点参数未设置"));
  }

  if (params.init_brightness < 0 || params.init_brightness > 100) {
    RCLCPP_WARN(node->get_logger(),
                "初始亮度值 %d 超出范围 [0, 100]，将使用默认值 0",
                params.init_brightness);
    params.init_brightness = 50;
  }

  RCLCPP_INFO(node->get_logger(), "节点参数加载成功。");
  return params;
}

ResultErr<void> DisplayNode::InitDrmApp(
    DisplayNode* node, const DisplayNode::NodeParameters& params) {
  const auto image_path = params.image_base_path / params.image_name;

  RCLCPP_INFO(node->get_logger(), "DRM 设备: %s", params.drm_device.c_str());
  RCLCPP_INFO(node->get_logger(), "尝试加载初始图片：%s", image_path.c_str());

  auto app_res = drm::DrmApp::Create(params.drm_device, image_path.c_str());
  if (!app_res) return tl::make_unexpected(app_res.error());
  node->app_ = std::move(*app_res);

  // 设置初始亮度
  node->app_->RequestUpdate(std::nullopt,
                            static_cast<uint8_t>(params.init_brightness));

  return {};
}

void DisplayNode::SetupSubscription(DisplayNode* node) {
  node->subscription_ = node->create_subscription<DisplayMsg>(
      "~/update_display", 10, [node](const std::shared_ptr<DisplayMsg> msg) {
        node->UpdateCallback(msg);
      });

  RCLCPP_INFO(node->get_logger(), "已订阅Topic:%s",
              node->subscription_->get_topic_name());
}

void DisplayNode::StartDisplayThread(DisplayNode* node) {
  node->display_thread_ = std::thread(&DisplayNode::DisplayThreadLoop, node);
}

void DisplayNode::DisplayThreadLoop() {
  RCLCPP_INFO(get_logger(), "显示线程启动");
  if (auto run_res = app_->Run(); !run_res) {
    if (rclcpp::ok()) {
      RCLCPP_ERROR(get_logger(), "DrmApp运行出错：%s",
                   run_res.error().message().c_str());
    }
  }
  RCLCPP_INFO(get_logger(), "显示线程已结束");
}

void DisplayNode::UpdateCallback(const std::shared_ptr<DisplayMsg> msg) {
  std::optional<std::string> full_path;
  std::optional<uint8_t> brightness;

  // 处理图片路径
  if (!msg->image_path.empty()) {
    std::string path_str = (image_base_path_ / msg->image_path).string();
    if (std::filesystem::exists(path_str)) {
      full_path = std::move(path_str);
      RCLCPP_INFO(get_logger(), "收到图片更新请求: %s", full_path->c_str());
    } else {
      RCLCPP_ERROR(get_logger(), "图片文件不存在: %s", path_str.c_str());
    }
  }

  // 处理亮度
  if (msg->brightness != -1) {
    brightness = std::max(0, std::min(100, (int)msg->brightness));
    RCLCPP_INFO(get_logger(), "收到亮度更新请求: %d", *brightness);
  }

  // 4. 仅当有有效更新时才调用
  if (full_path.has_value() || brightness.has_value()) {
    app_->RequestUpdate(full_path, brightness);
  }
}

}  // namespace neofur
