// include/neofur_display/graphics/camera.h
#pragma once

#include <Magnum/Math/Matrix4.h>
#include <Magnum/SceneGraph/Camera.h>
#include <Magnum/SceneGraph/Object.h>
#include <Magnum/SceneGraph/Scene.h>

namespace neofur {
namespace graphics {

// 为了方便，在Camera的头文件中定义场景图的类型别名
using Scene3D = Magnum::SceneGraph::Scene<Magnum::SceneGraph::MatrixTransformation3D>;
using Object3D = Magnum::SceneGraph::Object<Magnum::SceneGraph::MatrixTransformation3D>;

/**
 * @class Camera
 * @brief 场景中的观察者。封装了相机在场景图中的对象、投影和视图变换。
 *
 * 这个类提供了一套完整的接口来控制相机的位置、朝向和镜头属性，
 * 将底层的 Magnum::SceneGraph::Camera3D 和 Object3D 的操作进行了优雅的封装。
 */
class Camera {
 public:
  /**
   * @brief 构造一个相机。
   * @param scene 指向相机所属的场景图根节点。
   * 相机的变换将在此场景图内进行。
   */
  explicit Camera(Scene3D* scene)
      : object_(std::make_unique<Object3D>(scene)),
        camera_(std::make_unique<Magnum::SceneGraph::Camera3D>(*object_)) {}

  Camera(const Camera&) = delete;
  Camera(Camera&&) = default;
  Camera& operator=(const Camera&) = delete;
  Camera& operator=(Camera&&) = default;
  ~Camera() = default;

  // --- 核心功能 ---

  /**
   * @brief 设置相机的视口和投影矩阵。
   * @param width 视口宽度。
   * @param height 视口高度。
   * @param fovY 垂直视场角 (Field of View)，以度为单位。
   * @param nearPlane 近裁剪面。
   * @param farPlane 远裁剪面。
   */
  void SetProjection(int width, int height, float fovY = 35.0f,
                     float nearPlane = 0.01f, float farPlane = 100.0f) {
    using namespace Magnum::Math::Literals;
    camera_->setViewport({width, height});
    camera_->setProjectionMatrix(Magnum::Matrix4::perspectiveProjection(
        Magnum::Deg{fovY}, 1.0f * width / height, nearPlane, farPlane));
  }

  /**
   * @brief 让相机看向一个世界坐标系中的目标点。
   * @param eyePosition 相机自身的位置。
   * @param targetPosition 要看向的目标点。
   * @param upDirection 定义“上方”的向量，通常是 (0, 1, 0)。
   */
  void LookAt(const Magnum::Vector3& eyePosition,
              const Magnum::Vector3& targetPosition,
              const Magnum::Vector3& upDirection = {0.0f, 1.0f, 0.0f}) {
    object_->setTransformation(
        Magnum::Matrix4::lookAt(eyePosition, targetPosition, upDirection));
  }

  // --- 变换操作 ---

  /**
   * @brief 重置相机的位置和朝向。
   * @param transform 新的变换矩阵。
   */
  void SetTransform(const Magnum::Matrix4& transform) {
    object_->setTransformation(transform);
  }

  /**
   * @brief 沿相机的本地坐标系移动相机。
   * @param translation 移动向量。
   */
  void TranslateLocal(const Magnum::Vector3& translation) {
    object_->translateLocal(translation);
  }

  /**
   * @brief 绕相机的本地坐标轴旋转。
   * @param angle 旋转角度。
   * @param normalizedAxis 旋转轴（应为单位向量）。
   */
  void RotateLocal(Magnum::Rad angle, const Magnum::Vector3& normalizedAxis) {
    object_->rotateLocal(angle, normalizedAxis);
  }

  // --- Getter ---

  /**
   * @brief 获取底层的 Magnum 相机对象。
   * 主要由 Renderer 在绘制时使用。
   */
  Magnum::SceneGraph::Camera3D& camera() { return *camera_; }

  /**
   * @brief 获取相机在场景图中的变换节点。
   * 在需要将其他物体附加到相机上时（如UI元素）非常有用。
   */
  Object3D& object() { return *object_; }

  /**
   * @brief 获取相机当前的世界坐标位置。
   */
  Magnum::Vector3 position() const {
    return object_->transformation().translation();
  }

 private:
  std::unique_ptr<Object3D> object_;
  std::unique_ptr<Magnum::SceneGraph::Camera3D> camera_;
};

}  // namespace graphics
}  // namespace neofur
