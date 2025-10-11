cmake_minimum_required(VERSION 3.8)
project(neofur_utils)

# 1. 查找依赖
find_package(ament_cmake REQUIRED)

# 2. 定义 INTERFACE 库
add_library(${PROJECT_NAME} INTERFACE)

# 3. 关联头文件目录
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 4. 安装头文件
install(
  DIRECTORY include/
  DESTINATION include
)

# 5. 【关键】安装目标并创建导出集
# 对于 INTERFACE 库，这不会安装任何文件，但会创建一个包含其属性（如 include 目录）的导出集。
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}_targets  # 创建名为 "neofur_utils_targets" 的导出集
)

# 6. 【关键】使用 ament_export_targets 导出上面创建的“导出集”
# 注意，参数是导出集的名称，而不是目标本身的名称。
# HAS_LIBRARY_TARGET FALSE 明确告知 ament 这是一个没有 .so/.a 文件的接口库。
ament_export_targets(${PROJECT_NAME}_targets)

# 7. ament 打包
# ament_package() 会找到由 ament_export_targets 注册的导出集并正确处理。
ament_package()
