cmake_minimum_required(VERSION 3.16)
project(neofur_display)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- 1. 查找 ROS 和底层系统依赖 ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(neofur_utils REQUIRED)
find_package(neofur_interfaces REQUIRED)

# 使用 PkgConfig 查找 DRM/GBM/EGL/GLESv2 (保持不变)
find_package(PkgConfig REQUIRED)
pkg_check_modules(DRM REQUIRED libdrm)
pkg_check_modules(GBM REQUIRED gbm)
pkg_check_modules(EGL REQUIRED egl)
pkg_check_modules(GLESv2 REQUIRED glesv2)

# --- 2. 查找由 vcpkg 管理的 C++ 库 ---
# find_package 会通过 CMAKE_TOOLCHAIN_FILE 自动找到这些库
find_package(ms-gsl REQUIRED)
find_package(Corrade REQUIRED Utility)
find_package(Magnum REQUIRED GL MeshTools Primitives SceneGraph Shaders Trade)
find_package(MagnumPlugins REQUIRED AnyImageImporter GltfImporter)
find_package(MagnumIntegration REQUIRED)
find_package(TinyGLTF REQUIRED)

# --- 3. 可执行文件定义 ---

# a) logic_node (保持不变, 职责清晰)
add_executable(logic_node src/logic/logic_node.cc)

target_include_directories(logic_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(logic_node
  rclcpp::rclcpp
  neofur_interfaces::neofur_interfaces__rosidl_generator_cpp
  neofur_utils::neofur_utils
)


# b) renderer_node (更新以链接所有新库)
#    确保 GLOB 包含所有子目录的源文件
file(GLOB RENDERER_SOURCES
  "src/renderer/*.cc"
  "src/renderer/platform/*.cc"
  "src/renderer/core/*.cc"      # 假设您的新渲染核心代码在此
)
add_executable(renderer_node ${RENDERER_SOURCES})

target_include_directories(renderer_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  # 系统库的头文件路径
  ${DRM_INCLUDE_DIRS}
  ${GBM_INCLUDE_DIRS}
  ${EGL_INCLUDE_DIRS}
  ${GLESv2_INCLUDE_DIRS}
)

target_link_libraries(renderer_node
  # ROS 库
  rclcpp::rclcpp
  neofur_interfaces::neofur_interfaces__rosidl_generator_cpp
  neofur_utils::neofur_utils

  # 底层平台库
  ${DRM_LIBRARIES}
  ${GBM_LIBRARIES}
  ${EGL_LIBRARIES}
  ${GLESv2_LIBRARIES}

  # vcpkg 提供的现代 C++ 库
  GSL::gsl
  Corrade::Utility
  Magnum::GL
  Magnum::MeshTools
  Magnum::Primitives
  Magnum::SceneGraph
  Magnum::Shaders
  Magnum::Trade
  Magnum::Integration
  MagnumPlugins::AnyImageImporter
  MagnumPlugins::GltfImporter
  tinygltf # tinygltf 的 CMake 目标
)

# --- 4. 安装规则 (保持不变) ---
install(
  TARGETS
    logic_node
    renderer_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY resource
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


# --- 5. Ament 打包 ---
ament_package()
