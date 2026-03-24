cmake_minimum_required(VERSION 3.8)
project(kuka_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clash")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# 安装目录到 share 文件夹下，这样 $(find ...) 才能找到它们
install(
  DIRECTORY meshes urdf launch
  DESTINATION share/${PROJECT_NAME}
)


ament_package()