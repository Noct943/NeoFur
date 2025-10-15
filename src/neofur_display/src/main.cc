#include <rclcpp/rclcpp.hpp>

#include "neofur_display/node.h"  // 请替换为您的节点头文件路径

// 引入 RGA 头文件和标准输出
#include <librga/im2d.h>

#include <iostream>

int main(int argc, char **argv) {
  // --- RGA 环境自检 ---
  // 在初始化任何业务逻辑之前，首先校验 RGA 环境
  IM_STATUS status = imcheckHeader();
  if (status != IM_STATUS_SUCCESS) {
    // 使用 RCLCPP_FATAL 记录致命错误并终止
    RCLCPP_FATAL(rclcpp::get_logger("main"),
                 "RGA environment check failed: Header and library version "
                 "mismatch! Error: %s",
                 imStrError(status));
    return -1;
  }
  // 打印 RGA 硬件信息，便于调试和确认
  RCLCPP_INFO(rclcpp::get_logger("main"), "RGA Info:\n%s",
              querystring(RGA_ALL));

  // --- ROS 节点初始化 ---
  rclcpp::init(argc, argv);

  auto node_result = neofur::DisplayNode::Create(rclcpp::NodeOptions());
  if (node_result) {
    rclcpp::spin(*node_result);
  } else {
    RCLCPP_FATAL(rclcpp::get_logger("main"), "节点创建失败: %s",
                 node_result.error().message().c_str());
  }

  rclcpp::shutdown();
  return 0;
}
