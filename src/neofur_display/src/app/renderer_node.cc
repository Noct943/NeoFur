// src/app/renderer_node.cc
#include "neofur_display/app/renderer_node.h"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <Magnum/Platform/Application.h>
#include <chrono>

#include "neofur_utils/log_manager.h"

using namespace std::chrono_literals;

namespace neofur {
namespace app {

RendererNode::RendererNode() : Node("renderer_node") {
  // --- 1. 桥接日志系统 ---
  // 设置全局日志记录器，将所有底层的日志消息重定向到 ROS 2 的日志系统中。
  utils::LogManager::SetLogger([this](utils::LogLevel level, std::string_view msg) {
    switch (level) {
      case utils::LogLevel::Info:
        RCLCPP_INFO(this->get_logger(), "%s", msg.data());
        break;
      case utils::LogLevel::Warning:
        RCLCPP_WARN(this->get_logger(), "%s", msg.data());
        break;
      case utils::LogLevel::Error:
        RCLCPP_ERROR(this->get_logger(), "%s", msg.data());
        break;
    }
  });

  // --- 2. 初始化渲染管线 (RAL & PAL) ---
  // Create 方法保证了返回的 renderer 对象是完全初始化且可用的。
  auto renderer_res = rendering::Renderer::Create();
  if (!renderer_res) {
    utils::LogManager::Error("渲染器核心(RAL)创建失败: " + renderer_res.error().message());
    rclcpp::shutdown(); // 关键：初始化失败时，安全关闭节点。
    return;
  }
  renderer_ = std::move(*renderer_res);

  // --- 3. 初始化场景 (SGL) ---
  try {
    const auto package_share_dir = ament_index_cpp::get_package_share_directory("neofur_display");
    scene_ = std::make_unique<graphics::Scene>(package_share_dir + "/resource/assets");
    
    auto load_res = scene_->Load("scenes/default_eye.glb");
    if (!load_res) {
      utils::LogManager::Error("场景(SGL)加载失败: " + load_res.error().message());
      rclcpp::shutdown();
      return;
    }
  } catch (const std::exception& e) {
    utils::LogManager::Error("获取 'neofur_display' 包路径失败: " + std::string(e.what()));
    rclcpp::shutdown();
    return;
  }

  // --- 4. 初始化相机 (SGL) ---
  camera_ = std::make_unique<graphics::Camera>(scene_->get_magnum_scene());
  camera_->SetProjection(renderer_->get_width(), renderer_->get_height());
  // 设置一个合适的初始机位，让相机看向原点
  camera_->LookAt({0.0f, 0.5f, 3.0f},  // 相机位置 (eye)
                  {0.0f, 0.5f, 0.0f},  // 目标位置 (target)
                  {0.0f, 1.0f, 0.0f}); // 上方向 (up)

  // --- 5. 启动渲染循环 ---
  // 创建一个高频定时器，以大约60FPS的频率驱动 render_loop 函数。
  timer_ = this->create_wall_timer(16ms, std::bind(&RendererNode::render_loop, this));

  RCLCPP_INFO(this->get_logger(), "RendererNode 初始化成功，已进入渲染循环。");
}

RendererNode::~RendererNode() {
  utils::LogManager::Info("RendererNode 正在析构。");
}

void RendererNode::render_loop() {
  // --- A. 应用逻辑更新 (未来扩展) ---
  // 在这里，我们可以根据ROS消息来更新场景状态。
  // 例如：
  // float time = this->now().seconds();
  // Object3D* eye_bone = scene_->FindNode("Eye_Bone");
  // if(eye_bone) {
  //   eye_bone->setRotation(Magnum::Quaternion::rotation(Magnum::Rad(time), {0,1,0}));
  // }
  // scene_->Update(deltaTime);

  // --- B. 渲染 ---
  // 指挥渲染器使用指定的相机来绘制更新后的场景。
  auto res = renderer_->Render(*scene_, *camera_);
  if (!res) {
    // 如果渲染循环中出现不可恢复的错误，记录并关闭节点。
    utils::LogManager::Error("渲染循环发生严重错误: " + res.error().message());
    rclcpp::shutdown();
  }
}

}  // namespace app
}  // namespace neofur

// --- 主函数入口 ---
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  // Magnum 要求在任何 GL 操作之前创建 Platform::Application 实例。
  Magnum::Platform::Application::instance().tryCreate();
  // 创建并运行 RendererNode。
  auto node = std::make_shared<neofur::app::RendererNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
