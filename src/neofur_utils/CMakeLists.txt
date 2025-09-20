# neofur_utils/CMakeLists.txt

cmake_minimum_required(VERSION 3.8)
project(neofur_utils)

# 1. 查找 ament_cmake
find_package(ament_cmake REQUIRED)

# 2. 导出 include 目录，让其他包能找到头文件 (关键补充)
ament_export_include_directories(include)

# 3. 安装头文件
install(
  DIRECTORY include/
  DESTINATION include
)

# 4. 注册包
ament_package()
