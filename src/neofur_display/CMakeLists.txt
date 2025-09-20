cmake_minimum_required(VERSION 3.8)
project(neofur_display)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- 依赖包查找 ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(neofur_utils REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(neofur_interfaces REQUIRED)

pkg_check_modules(DRM REQUIRED libdrm)

# --- [改进] 使用 find_library 手动查找 RGA 库 ---
# 由于 librga 是我们手动 COPY 到 /usr/local 下的，它没有 .pc 文件
# 因此我们使用 find_library 直接在指定路径下查找
find_library(RGA_LIBRARY
             NAMES rga
             PATHS /usr/local/lib
             REQUIRED)
message(STATUS "Found RGA library: ${RGA_LIBRARY}")

# --- 可执行文件定义 ---
add_executable(neofur_display_node
  src/app.cc
  src/atomic.cc
  src/device.cc
  src/display.cc
  src/layer.cc
  src/node.cc
  src/main.cc
)

# --- 依赖项 ---
ament_target_dependencies(
  neofur_display_node
  rclcpp
  neofur_utils
  neofur_interfaces
)

# --- 头文件路径 ---
target_include_directories(neofur_display_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${neofur_utils_INCLUDE_DIRS}
  ${DRM_INCLUDE_DIRS}
  # --- [改进] 直接添加 RGA 头文件路径 ---
  # 我们明确知道头文件被安装到了 /usr/local/include
  /usr/local/include
  /usr/include)

# --- 链接库 ---
target_link_libraries(neofur_display_node
  ${DRM_LIBRARIES}
  # --- [改进] 链接找到的 RGA 库 ---
  ${RGA_LIBRARY}
)

# --- 安装规则 ---
install(
  TARGETS neofur_display_node
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

# --- ament 包 ---
ament_package()
