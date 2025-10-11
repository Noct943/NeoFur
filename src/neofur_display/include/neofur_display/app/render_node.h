// include/neofur_display/app/renderer_node.h
#pragma once

#include <rclcpp/rclcpp.hpp>
#include "neofur_display/graphics/camera.h"
#include "neofur_display/graphics/scene.h"
#include "neofur_display/rendering/renderer.h"

namespace neofur {
namespace app {

/**
 * @class RendererNode
 * @brief 应用层核心，作为 ROS 2 节点运行。
 *
 * 负责初始化整个渲染管线（从平台层到场景层），
 * 驱动主渲染循环，并作为与 ROS 系统其余部分交互的接口。
 */
class RendererNode : public rclcpp::Node {
 public:

  RendererNode();
  ~RendererNode() override;

 private:
  /**
   * @brief 由 ROS 2 定时器周期性调用的主渲染循环。
   */
  void render_loop();

  // 指向渲染管线各核心层对象的智能指针
  std::unique_ptr<rendering::Renderer> renderer_;
  std::unique_ptr<graphics::Scene> scene_;
  std::unique_ptr<graphics::Camera> camera_;
  
  // ROS 2 定时器，用于驱动渲染循环
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace app
}  // namespace neofur
